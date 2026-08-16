"""Source/component tests for the latent-live V12R9 morphology runtime."""

from __future__ import annotations

import ast
import copy
import hashlib
import json
import os
import struct
import subprocess
import sys
from pathlib import Path
from typing import Any
from unittest import mock

import pytest


HERE = Path(__file__).resolve().parent
BASELINE_ROOT = HERE.parents[1]
for _root in (HERE, BASELINE_ROOT):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import audit_h0_v12r9_morphology_readiness as readiness_auditor  # noqa: E402
import freeze_h0_v12r9_morphology as freezer  # noqa: E402
import h0_v12r9_morphology_causal_runtime as causal_runtime  # noqa: E402
import h0_v12r9_morphology_contract as contract  # noqa: E402
import h0_v12r9_morphology_gates as gates  # noqa: E402
import run_h0_v12r9_morphology as runner  # noqa: E402
import run_h0_v12r9_morphology_training as training_launcher  # noqa: E402


def _sha(label: str) -> str:
    return hashlib.sha256(label.encode("utf-8")).hexdigest()


def _artifact(path: str, label: str | None = None) -> dict[str, Any]:
    return {
        "path": path,
        "sha256": _sha(label or path),
        "size_bytes": max(1, len(label or path)),
    }


def _tree(path: str, names: list[str]) -> dict[str, Any]:
    rows = [
        {"path": name, "sha256": _sha(f"{path}:{name}"), "size_bytes": index + 1}
        for index, name in enumerate(sorted(names))
    ]
    digest = hashlib.sha256()
    for row in rows:
        digest.update(row["path"].encode())
        digest.update(b"\0")
        digest.update(row["sha256"].encode())
        digest.update(b"\0")
        digest.update(str(row["size_bytes"]).encode())
        digest.update(b"\n")
    return {
        "path": path,
        "tree_sha256": digest.hexdigest(),
        "file_count": len(rows),
        "files": rows,
    }


def _candidate() -> dict[str, Any]:
    return _tree(
        contract.CANDIDATE_MODULE_PATH.as_posix(),
        list(contract.CANDIDATE_REQUIRED_FILES),
    )


def _checkpoint() -> dict[str, Any]:
    return _tree(
        contract.CHECKPOINT_ZERO_PATH.as_posix(),
        list(contract.zero.CHECKPOINT_REQUIRED_SUFFIXES),
    )


def _binding() -> dict[str, Any]:
    candidate = _candidate()
    return {
        "candidate_id": contract.candidate_id_for_tree(candidate["tree_sha256"]),
        "candidate_module": candidate,
        "actor_digest": _sha("actor"),
        "checkpoint": _checkpoint(),
        "resolved_config": _artifact(contract.TRAINING_CONFIG_PATH.as_posix()),
    }


def _upstream_payload() -> dict[str, Any]:
    binding = _binding()
    candidate = binding["candidate_module"]
    endpoints = contract.UPSTREAM_ENDPOINTS
    r9_endpoint = endpoints["v12r9_terminal"]
    q3_endpoint = endpoints["v12r9_q3_terminal"]
    zero_endpoint = endpoints["v12r9_zero_terminal"]
    r9 = {
        "schema_version": r9_endpoint["schema_version"],
        "status": r9_endpoint["required_status"],
        "passed": True,
        "terminal": True,
        "protocol_id": r9_endpoint["protocol_id"],
        "pipeline_id": r9_endpoint["pipeline_id"],
        "error": None,
        "candidate_id": binding["candidate_id"],
        "candidate_module": candidate,
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "runtime_promoted": False,
        "qualification_executed": False,
    }
    q3 = {
        "schema_version": q3_endpoint["schema_version"],
        "status": q3_endpoint["required_status"],
        "passed": True,
        "terminal": True,
        "protocol_id": q3_endpoint["protocol_id"],
        "pipeline_id": q3_endpoint["pipeline_id"],
        "error": None,
        "error_type": None,
        "candidate_id": binding["candidate_id"],
        "candidate_module": candidate,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "morphology_weight": 0.0,
        "positive_morphology_enabled": False,
        "checkpoint_zero_created": False,
        "runtime_promoted": False,
    }
    zero = {
        "schema_version": zero_endpoint["schema_version"],
        "status": zero_endpoint["required_status"],
        "passed": True,
        "terminal": True,
        "protocol_id": zero_endpoint["protocol_id"],
        "pipeline_id": zero_endpoint["pipeline_id"],
        "error": None,
        "binding": {
            "candidate_id": binding["candidate_id"],
            "candidate_module": candidate,
            "actor_digest": binding["actor_digest"],
        },
        "checkpoint": binding["checkpoint"],
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "environment_samples": 0,
        "training_executed": False,
        "required_working_directory": contract.REQUIRED_WORKING_DIRECTORY,
        "positive_live_config_restore_smoke_passed": True,
        "training_authorized": False,
        "training_command_published": False,
        "next_stage": contract.zero.NEXT_STAGE_AFTER_ZERO_PASS,
    }
    results = {
        "v12r9_terminal": r9,
        "v12r9_q3_terminal": q3,
        "v12r9_zero_terminal": zero,
    }
    attestations = {}
    for name, result in results.items():
        endpoint = endpoints[name]
        attestations[name] = {
            "endpoint": copy.deepcopy(endpoint),
            "artifact": _artifact(endpoint["path"]),
            "verifier_module": endpoint["verifier_module"],
            "verifier": endpoint["verifier"],
            "verified_result_sha256": gates.canonical_json_sha256(result),
            "verifier_returned_mapping": True,
        }
    return {
        **results,
        "semantic_attestations": attestations,
        "resolved_config": binding["resolved_config"],
    }


def _source_closure() -> dict[str, Any]:
    value = {
        "sources": {
            name: _artifact(path)
            for name, path in contract.SOURCE_RELATIVE_PATHS.items()
        },
        "inputs": {
            name: _artifact(path)
            for name, path in contract.INPUT_RELATIVE_PATHS.items()
        },
        "candidate_module": _candidate(),
        "checkpoint": _checkpoint(),
    }
    for name, expected in contract.PROFILE_ATTESTATIONS.items():
        value["inputs"][name]["sha256"] = expected["sha256"]
    value["sources"]["causal_corridor"]["sha256"] = contract.FROZEN_CORRIDOR_SHA256
    return value


def _condition(case_id: str) -> dict[str, Any]:
    case = contract.canonical_case(case_id)
    noise = {
        "artifact": _artifact(case["noise_tape"]),
        "array_sha256": contract.EXPECTED_TAPE_ARRAY_SHA256[
            Path(case["noise_tape"]).name
        ],
    }
    core = {"holdout_case": case, "noise_record": noise}
    return {**core, "condition_sha256": gates.canonical_json_sha256(core)}


def _conditions() -> dict[str, Any]:
    return {case_id: _condition(case_id) for case_id in contract.CASE_IDS}


def _capabilities() -> dict[str, Any]:
    observed = {name: name == "actions" for name in contract.Q3_REQUIRED_REUSE_STREAMS}
    missing = [name for name in contract.Q3_REQUIRED_REUSE_STREAMS if name != "actions"]
    return {
        "status": "INSPECTED_H0_V12R9_Q3_CONTROL_CAPABILITY",
        "q3_schema_version": contract.q3.SCHEMA_VERSION,
        "q3_protocol_id": contract.q3.PROTOCOL_ID,
        "case_ids": list(contract.CASE_IDS),
        "trace_artifacts": {
            case_id: _artifact(
                (
                    Path(
                        contract.q3.canonical_rollout(
                            contract.q3.CANDIDATE_ROLE, case_id
                        )["destination"]
                    )
                    / "trace.json"
                ).as_posix()
            )
            for case_id in contract.CASE_IDS
        },
        "observed_streams": {
            case_id: copy.deepcopy(observed) for case_id in contract.CASE_IDS
        },
        "missing_required_streams": {
            case_id: list(missing) for case_id in contract.CASE_IDS
        },
        "q3_trace_sufficient": False,
        "selected_control_source_mode": "paired_rerun",
    }


def _module_evidence() -> dict[str, Any]:
    return {
        "zero_audit": _artifact(contract.zero.AUDIT_PATH.as_posix()),
        "checkpoint": _checkpoint(),
        "actor_digest": _sha("actor"),
        "module_export": _tree(
            contract.MODULE_EXPORT_PATH.as_posix(),
            ["actor_feature_manifest.json", "module_state.pkl"],
        ),
        "module_manifest": _artifact(
            (contract.MODULE_EXPORT_PATH / "actor_feature_manifest.json").as_posix()
        ),
    }


def _protocol_and_lock() -> tuple[dict[str, Any], dict[str, Any]]:
    protocol = freezer.build_protocol_freeze_payload(
        upstream_payload=_upstream_payload(),
        source_closure=_source_closure(),
        conditions=_conditions(),
        q3_capabilities=_capabilities(),
        checkpoint_module=_module_evidence(),
    )
    record = _artifact(contract.PROTOCOL_FREEZE_PATH.as_posix())
    return protocol, freezer.build_lock_payload(protocol, protocol_artifact=record)


def _diagnostics(*, resolved: int) -> dict[str, Any]:
    return {
        "event_contract_id": contract.V26_EVENT_CONTRACT_ID,
        "delay_s": 0.04,
        "failed_closed": False,
        "failure_reason": "",
        "drop_reason": "",
        "dropped_sample_count": 0,
        "dropped_pending_sample_count": 0,
        "dropped_wait_hs_sample_count": 0,
        "pending_sample_count": 0,
        "resolved_sample_count": resolved,
        "total_resolved_sample_count": 1,
        "total_dropped_sample_count": 0,
        "cancelled_transition_count": 0,
        "total_cancelled_transition_count": 0,
        "timeout_transition_count": 0,
        "terminal_flushed": False,
        "actor_state_name": "SWING_AFTER_TO",
        "partial_stance_active": False,
    }


def _sample() -> dict[str, Any]:
    fields = {name: 0.0 for name in contract.CAUSAL_SAMPLE_FIELDS}
    fields.update(
        {
            "time_s": 1.0,
            "emitted_time_s": 1.04,
            "delay_s": 0.04,
            "terminal_flush": False,
            "segment_type": "swing",
            "segment_start_time_s": 0.8,
            "anchor_confirmed_time_s": 1.0,
            "anchor_delivered_time_s": 1.005,
            "duration_basis_s": 0.5,
            "phase": 0.7,
            "knee_loss": 1.0,
            "ankle_loss": 1.0,
        }
    )
    return fields


def _ledgers(role: str) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    rewards = []
    causal = []
    for index in range(contract.EXPECTED_STEPS_PER_ROLLOUT):
        loss = 1.0 if index == 0 else 0.0
        term = 0.0 if role == contract.CONTROL_ROLE else 0.0025 * loss
        base = float(1.0 - index / 1000.0)
        reward = float(base - term)
        rewards.append(
            {
                "step": index + 1,
                "actual_reward": reward,
                "recomputed_reward": reward,
                "reward_without_morphology": base,
                "morphology_loss": loss,
                "morphology_term": float(term),
            }
        )
        samples = [_sample()] if index == 0 else []
        causal.append(
            {
                "step": index + 1,
                "samples": samples,
                "diagnostics": _diagnostics(resolved=len(samples)),
            }
        )
    return rewards, causal


def _stream(label: str) -> dict[str, Any]:
    return {
        "sha256": _sha(label),
        "size_bytes": 100,
        "sample_count": contract.EXPECTED_STEPS_PER_ROLLOUT,
        "encoding": contract.STREAM_ENCODING,
    }


def _detector() -> dict[str, Any]:
    return {
        "runtime_config": copy.deepcopy(contract.V26_RUNTIME_CONFIG),
        "sample_count": contract.EXPECTED_RAW_SAMPLES_PER_ROLLOUT,
        "fallback_count": 0,
        "non_v26_source_count": 0,
        "duplicate_event_count": 0,
        "out_of_order_event_count": 0,
        "hard_invalid_count": 0,
        "passed": True,
    }


def _runtime_attestation() -> dict[str, Any]:
    return {
        **copy.deepcopy(contract.CAUSAL_RUNTIME_CONFIG),
        "live_attestation": {
            "passed": True,
            "runtime_id": contract.CAUSAL_RUNTIME_ID,
            "checks": {"runtime_id": True, "corridor": True, "reward": True},
        },
    }


def _arm(
    case_id: str,
    role: str,
    index: int,
    *,
    streams: dict[str, Any],
) -> tuple[dict[str, Any], list[dict[str, Any]], list[dict[str, Any]]]:
    rewards, causal = _ledgers(role)
    return (
        {
            "schema_version": contract.SCHEMA_VERSION,
            "status": contract.ARM_COMPLETE_STATUS,
            "protocol_id": contract.PROTOCOL_ID,
            "case_id": case_id,
            "role": role,
            "local_execution_index": index,
            "binding": _binding(),
            "condition": _condition(case_id),
            "reward_config": contract.reward_config_for_role(role),
            "streams": copy.deepcopy(streams),
            "detector_audit": _detector(),
            "causal_audit": gates.causal_ledger_gate(causal, reward_rows=rewards),
            "causal_runtime": _runtime_attestation(),
            "artifacts": {
                "trace": _artifact(f"trace/{case_id}/{role}.json"),
                "reward_ledger": _artifact(f"reward/{case_id}/{role}.json"),
                "causal_ledger": _artifact(f"causal/{case_id}/{role}.json"),
                "noise_tape": _artifact(contract.canonical_case(case_id)["noise_tape"]),
                "module_export": _module_evidence()["module_export"],
            },
            "started_unix_s": float(index * 2),
            "completed_unix_s": float(index * 2 + 1),
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
            "training_executed": False,
        },
        rewards,
        causal,
    )


def _pair(case_id: str, case_index: int) -> dict[str, Any]:
    streams = {name: _stream(f"{case_id}:{name}") for name in contract.STREAM_NAMES}
    control, control_rewards, control_causal = _arm(
        case_id, contract.CONTROL_ROLE, 2 * case_index, streams=streams
    )
    positive, positive_rewards, positive_causal = _arm(
        case_id, contract.POSITIVE_ROLE, 2 * case_index + 1, streams=streams
    )
    positive["started_unix_s"] = control["completed_unix_s"] + 1.0
    positive["completed_unix_s"] = positive["started_unix_s"] + 1.0
    return gates.pair_gate(
        {
            "schema_version": contract.SCHEMA_VERSION,
            "status": contract.PAIR_COMPLETE_STATUS,
            "protocol_id": contract.PROTOCOL_ID,
            "case_id": case_id,
            "control_source_mode": "paired_rerun",
            "control": control,
            "positive": positive,
        },
        expected_binding=_binding(),
        expected_condition=_condition(case_id),
        control_reward_rows=control_rewards,
        positive_reward_rows=positive_rewards,
        control_causal_rows=control_causal,
        positive_causal_rows=positive_causal,
    )


def test_contract_is_exact_forced_physical_and_resume_launcher() -> None:
    assert contract.contract_self_check()["passed"] is True
    assert contract.CONTROL_SOURCE_MODE == "paired_rerun"
    assert contract.EXPECTED_LOCAL_ROLLOUT_COUNT == 12
    assert contract.POSITIVE_CONFIG_DELTA_FIELDS == {
        "morphology_weight",
        "morphology_causal_allow_effects",
    }
    for platform in ("macos_arm64", "windows_x86_64"):
        argv = contract.final_training_argv(platform)
        assert "--resume-from" in argv
        assert "--warm-start" not in argv
        assert "--warm-start-raw" not in argv
        assert "run_h0_v12r9_morphology_training.py" in argv[1]
    contract_source = (HERE / "h0_v12r9_morphology_contract.py").read_text(
        encoding="utf-8"
    )
    assert contract_source.count('"child_process_site_hook"') == 1
    assert contract.REQUIRED_WORKING_DIRECTORY == "repository_root"
    assert contract.AUTHORITY["retry_authorized"] is False
    assert contract.AUTHORITY["training_authorized_before_terminal_pass"] is False
    assert contract.TRAINING_VALIDATION_ENDPOINT == {
        "namespace_root": (
            "Trajectory Generator/baseline_MLP/validation/v12r9training"
        ),
        "protocol_id": "AB06_H0_V12R9_MORPH_TRAINING_50_UPDATE_READINESS",
        "verifier_module": (
            "Trajectory Generator/baseline_MLP/validation/v12r9training/"
            "validate_h0_v12r9_training.py"
        ),
        "preflight_receipt_path": (
            "Trajectory Generator/baseline_MLP/validation/v12r9training/"
            "h0_v12r9_training_preflight_20260814/receipt.json"
        ),
        "preflight_required_status": (
            "PASS_H0_V12R9_TRAINING_PREFLIGHT_12_RUNNER_RESTORE"
        ),
        "postrun_audit_path": (
            "Trajectory Generator/runs/training/v12r9_morphology_0025_50update/"
            "v12r9_training_integrity_audit.json"
        ),
        "postrun_required_status": "PASS_H0_V12R9_TRAINING_50_UPDATE_INTEGRITY",
        "expected_new_updates": 50,
        "preflight_required_before_training": True,
        "postrun_audit_required": True,
    }


def test_r9_endpoint_abi_is_exclusive_and_native() -> None:
    assert set(contract.UPSTREAM_ENDPOINTS) == {
        "v12r9_terminal",
        "v12r9_q3_terminal",
        "v12r9_zero_terminal",
    }
    assert (
        contract.R9_TERMINAL_ENDPOINT == contract.UPSTREAM_ENDPOINTS["v12r9_terminal"]
    )
    assert contract.R9_TERMINAL_ENDPOINT["schema_version"] == contract.r9.SCHEMA_VERSION
    assert contract.Q3_TERMINAL_ENDPOINT["schema_version"] == contract.q3.SCHEMA_VERSION
    assert (
        contract.ZERO_TERMINAL_ENDPOINT["schema_version"]
        == contract.zero.SCHEMA_VERSION
    )
    for endpoint in contract.UPSTREAM_ENDPOINTS.values():
        assert "v12r9" in endpoint["verifier_module"]
        assert endpoint["required_status"].startswith("PASS_H0_V12R9")

    stale_lower = "v12" + "r8"
    stale_upper = "V12" + "R8"
    for source in HERE.glob("*.py"):
        text = source.read_text(encoding="utf-8")
        assert stale_lower not in text
        assert stale_upper not in text


def test_q3_cases_are_independent_and_disjoint_from_r9_development() -> None:
    assert contract.CASE_IDS == (
        "deterministic_offset_minus_0p30",
        "deterministic_offset_plus_0p30",
        "stochastic_nominal_seed_130",
        "stochastic_nominal_seed_131",
        "stochastic_nominal_seed_132",
        "stochastic_nominal_seed_133",
    )
    assert not set(contract.CASE_IDS).intersection(contract.r9.COLLECTION_CASE_IDS)
    assert not set(contract.CASE_IDS).intersection(contract.r9.DEVELOPMENT_CASE_IDS)


def test_detector_and_positive_only_morphology_contract_are_exact() -> None:
    assert contract.V26_RUNTIME_CONFIG["binary_phase_fsm_mode"] == "binary_active"
    assert contract.V26_RUNTIME_CONFIG["actor_event_source"] == "binary_active_v26"
    assert (
        contract.V26_RUNTIME_CONFIG["binary_phase_detector_profile_sha256"]
        == contract.q3.DETECTOR_PROFILE_SHA256
    )
    assert contract.MORPHOLOGY_REWARD_DELAY_S == 0.04
    assert contract.CAUSAL_RUNTIME_CONFIG["terminal_samples_younger_than_delay"] == (
        "drop_fail_safe"
    )
    assert contract.CONTROL_REWARD_CONFIG["morphology_weight"] == 0.0
    assert contract.CONTROL_REWARD_CONFIG["morphology_causal_allow_effects"] == 0.0
    assert contract.POSITIVE_REWARD_CONFIG["morphology_weight"] == 0.0025
    assert contract.POSITIVE_REWARD_CONFIG["morphology_causal_allow_effects"] == 1.0
    assert gates.exact_config_delta_gate(
        contract.CONTROL_REWARD_CONFIG, contract.POSITIVE_REWARD_CONFIG
    )


def test_readiness_auditor_static_abi_and_required_paths_are_exact() -> None:
    static = readiness_auditor.static_abi_audit()
    assert static["passed"] is True
    assert all(static["checks"].values())
    files = readiness_auditor.required_file_paths()
    directories = readiness_auditor.required_directory_paths()
    assert (
        files["training_validation::verifier"]
        == (contract.TRAINING_VALIDATION_ENDPOINT["verifier_module"])
    )
    assert files["training_validation::contract"].endswith(
        "/v12r9training/h0_v12r9_training_contract.py"
    )
    assert files["training_validation::tests"].endswith(
        "/v12r9training/test_h0_v12r9_training.py"
    )
    assert {
        name.removeprefix("q3_trace::")
        for name in files
        if name.startswith("q3_trace::")
    } == set(contract.CASE_IDS)
    assert directories == {
        "candidate_module": contract.CANDIDATE_MODULE_PATH.as_posix(),
        "checkpoint_zero": contract.CHECKPOINT_ZERO_PATH.as_posix(),
        "checkpoint_module_export": contract.MODULE_EXPORT_PATH.as_posix(),
    }
    assert contract.PROTOCOL_FREEZE_PATH.as_posix() not in files.values()
    assert contract.EXECUTION_LOCK_PATH.as_posix() not in files.values()


def test_readiness_auditor_defers_before_semantic_verification_when_missing(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def forbidden() -> dict[str, Any]:
        raise AssertionError("semantic verifier must not run while inputs are absent")

    monkeypatch.setattr(
        readiness_auditor.runner, "semantic_upstream_payload", forbidden
    )
    result = readiness_auditor.audit_live(
        file_predicate=lambda _path: False,
        directory_predicate=lambda _path: False,
    )
    assert result["status"] == readiness_auditor.DEFERRED_STATUS
    assert result["passed"] is False
    assert result["ready"] is False
    assert result["availability"]["missing"]
    assert result["semantic_verifiers_invoked"] is False
    assert result["write_operations"] == 0
    assert result["freeze_published"] is False
    assert result["lock_published"] is False
    assert result["rollouts_executed"] == 0
    assert result["training_updates_executed"] == 0


def test_training_validation_public_source_abi_is_hash_bound_and_read_only() -> None:
    result = readiness_auditor.training_validation_source_audit()
    assert result["passed"] is True
    assert all(result["checks"].values())
    assert result["source_check"]["status"] == ("PASS_H0_V12R9_TRAINING_SOURCE_CHECK")
    assert result["source_check"]["source_only"] is True
    assert result["source_check"]["training_ready"] is False
    assert (
        result["verifier"]["path"]
        == (contract.TRAINING_VALIDATION_ENDPOINT["verifier_module"])
    )
    assert result["contract"]["path"].endswith(
        "/v12r9training/h0_v12r9_training_contract.py"
    )


def test_native_upstream_semantics_and_same_candidate_are_required() -> None:
    payload = _upstream_payload()
    assert gates.upstream_terminal_gate(payload)["passed"] is True
    tampered = copy.deepcopy(payload)
    tampered["v12r9_q3_terminal"]["passed"] = False
    assert gates.upstream_terminal_gate(tampered)["passed"] is False
    forged = copy.deepcopy(payload)
    forged["semantic_attestations"]["v12r9_terminal"]["verified_result_sha256"] = _sha(
        "forged"
    )
    assert gates.upstream_terminal_gate(forged)["passed"] is False


def test_q3_trace_abi_forces_twelve_physical_rollouts() -> None:
    value = _capabilities()
    gate = gates.q3_control_capability_gate(value)
    assert gate["passed"] is True
    assert gate["selected_control_source_mode"] == "paired_rerun"
    mutation = copy.deepcopy(value)
    mutation["observed_streams"][contract.CASE_IDS[0]]["dynamics"] = True
    mutation["missing_required_streams"][contract.CASE_IDS[0]].remove("dynamics")
    assert gates.q3_control_capability_gate(mutation)["passed"] is False


def test_protocol_lock_and_plan_are_pure_and_source_closed() -> None:
    protocol, lock = _protocol_and_lock()
    assert freezer.verify_protocol_freeze_payload(protocol) == protocol
    assert runner.verify_lock_payload(lock) == lock
    plan = runner.build_execution_plan(lock)
    assert len(plan["runs"]) == 12
    assert [row["role"] for row in plan["runs"][:2]] == ["control", "positive"]
    assert plan["training_executed"] is False
    tampered = copy.deepcopy(lock)
    tampered["source_closure"]["sources"]["gates"]["sha256"] = _sha("drift")
    with pytest.raises(runner.MorphologyExecutionError):
        runner.verify_lock_payload(
            tampered, observed_source_closure=lock["source_closure"]
        )


@pytest.mark.parametrize(
    ("mutation", "expected_failure"),
    [
        ("emission_delay", True),
        ("delivery_latency", True),
        ("failed_closed", True),
        ("diagnostic_delay", True),
    ],
)
def test_causal_gate_is_per_sample_and_fail_closed(
    mutation: str, expected_failure: bool
) -> None:
    rewards, causal = _ledgers(contract.POSITIVE_ROLE)
    assert gates.causal_ledger_gate(causal, reward_rows=rewards)["passed"] is True
    changed = copy.deepcopy(causal)
    if mutation == "emission_delay":
        changed[0]["samples"][0]["emitted_time_s"] = 1.039
    elif mutation == "delivery_latency":
        changed[0]["samples"][0]["anchor_delivered_time_s"] = 1.011
    elif mutation == "failed_closed":
        changed[0]["diagnostics"]["failed_closed"] = True
        changed[0]["diagnostics"]["failure_reason"] = "injected"
    else:
        changed[0]["diagnostics"]["delay_s"] = 0.039
    assert (
        gates.causal_ledger_gate(changed, reward_rows=rewards)["passed"] is False
    ) is expected_failure


def test_reward_recomposition_is_ieee_exact_and_nonzero() -> None:
    positive, _causal = _ledgers(contract.POSITIVE_ROLE)
    gate = gates.reward_ledger_gate(positive, role=contract.POSITIVE_ROLE)
    assert gate["passed"] is True
    assert gate["nonzero_effect_sample_count"] == 1
    mutated = copy.deepcopy(positive)
    mutated[0]["actual_reward"] = struct.unpack(
        "!d",
        (
            struct.unpack("!Q", struct.pack("!d", mutated[0]["actual_reward"]))[0] + 1
        ).to_bytes(8, "big"),
    )[0]
    assert (
        gates.reward_ledger_gate(mutated, role=contract.POSITIVE_ROLE)["passed"]
        is False
    )


def test_pair_hash_identity_and_config_delta_are_mandatory() -> None:
    case_id = contract.CASE_IDS[0]
    pair = _pair(case_id, 0)
    assert pair["passed"] is True
    assert all(pair["checks"].values())
    streams = {name: _stream(f"{case_id}:{name}") for name in contract.STREAM_NAMES}
    control, cr, cc = _arm(case_id, "control", 0, streams=streams)
    positive, pr, pc = _arm(case_id, "positive", 1, streams=streams)
    positive["started_unix_s"] = 2.0
    positive["completed_unix_s"] = 3.0
    positive["streams"]["dynamics"]["sha256"] = _sha("different")
    result = gates.pair_gate(
        {
            "schema_version": contract.SCHEMA_VERSION,
            "status": contract.PAIR_COMPLETE_STATUS,
            "protocol_id": contract.PROTOCOL_ID,
            "case_id": case_id,
            "control_source_mode": "paired_rerun",
            "control": control,
            "positive": positive,
        },
        expected_binding=_binding(),
        expected_condition=_condition(case_id),
        control_reward_rows=cr,
        positive_reward_rows=pr,
        control_causal_rows=cc,
        positive_causal_rows=pc,
    )
    assert result["passed"] is False
    assert result["checks"]["dynamics_identical"] is False


def test_aggregate_rejects_all_zero_morphology_effect() -> None:
    pairs = [_pair(case_id, index) for index, case_id in enumerate(contract.CASE_IDS)]
    gate = gates.aggregate_gate(pairs, expected_binding=_binding())
    assert gate["passed"] is True
    zero = copy.deepcopy(pairs)
    for pair in zero:
        pair["effect"] = {
            "nonzero_effect_sample_count": 0,
            "total_morphology_loss": 0.0,
            "total_morphology_penalty": 0.0,
            "observed": False,
        }
    rejected = gates.aggregate_gate(zero, expected_binding=_binding())
    assert rejected["passed"] is False
    assert rejected["checks"]["effect_observed"] is False


def test_terminal_pass_alone_builds_training_handoff() -> None:
    _protocol, lock = _protocol_and_lock()
    pairs = [_pair(case_id, index) for index, case_id in enumerate(contract.CASE_IDS)]
    receipt = runner.build_terminal_receipt(lock_payload=lock, pair_results=pairs)
    assert gates.terminal_receipt_gate(receipt)["passed"] is True
    artifact = _artifact(contract.FINAL_RECEIPT_PATH.as_posix())
    handoff = runner.build_training_handoff(receipt, receipt_artifact=artifact)
    assert handoff["training_authorized"] is True
    assert handoff["initialization_mode"] == "resume_from_full_checkpoint_zero"
    assert handoff["required_working_directory"] == "repository_root"
    assert handoff["required_training_launcher"] == (
        "Trajectory Generator/baseline_MLP/validation/v12r9morph/"
        "run_h0_v12r9_morphology_training.py"
    )
    assert handoff["training_validation_endpoint"] == (
        contract.TRAINING_VALIDATION_ENDPOINT
    )
    for platform in handoff["platforms"].values():
        assert (
            platform["argv"][1].replace("\\", "/")
            == handoff["required_training_launcher"]
        )
    failed = copy.deepcopy(receipt)
    failed["passed"] = False
    with pytest.raises(runner.MorphologyExecutionError):
        runner.build_training_handoff(failed, receipt_artifact=artifact)


def test_freeze_execute_handoff_and_training_reject_non_root_cwd(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _protocol, lock = _protocol_and_lock()
    pairs = [_pair(case_id, index) for index, case_id in enumerate(contract.CASE_IDS)]
    receipt = runner.build_terminal_receipt(lock_payload=lock, pair_results=pairs)
    artifact = _artifact(contract.FINAL_RECEIPT_PATH.as_posix())
    monkeypatch.chdir(BASELINE_ROOT)
    with pytest.raises(runner.MorphologyExecutionError, match="repository_root"):
        runner.require_repository_root_cwd()
    with pytest.raises(runner.MorphologyExecutionError, match="repository_root"):
        freezer.freeze()
    with pytest.raises(runner.MorphologyExecutionError, match="repository_root"):
        runner.execute()
    with pytest.raises(runner.MorphologyExecutionError, match="repository_root"):
        runner.build_training_handoff(receipt, receipt_artifact=artifact)
    with pytest.raises(runner.MorphologyExecutionError, match="repository_root"):
        training_launcher.configure_process()


def test_additive_runtime_drops_early_terminal_flush_without_frozen_edit() -> None:
    corridor = causal_runtime.corridor
    sample = corridor.MorphologySample(time_s=1.0, knee_rad=0.0, ankle_rad=0.0)
    resolved = corridor.ResolvedCausalMorphologySample(
        sample=sample,
        segment_type="swing",
        segment_start_time_s=0.8,
        anchor_confirmed_time_s=1.0,
        anchor_delivered_time_s=1.005,
        emitted_time_s=1.02,
        duration_basis_s=0.5,
        phase=0.7,
        terminal_flush=True,
    )
    update = corridor.CausalMorphologyUpdate(
        resolved_samples=(resolved,),
        total_resolved_sample_count=1,
        terminal_flushed=True,
    )
    buffer = object.__new__(causal_runtime.StrictDelayCausalMorphologyBuffer)
    buffer.delay_s = 0.04
    buffer._total_resolved_sample_count = 1
    buffer._total_dropped_sample_count = 0
    buffer._last_emitted_sample_time_s = 0.8
    with mock.patch.object(
        causal_runtime._ORIGINAL_BUFFER, "update", return_value=update
    ):
        guarded = buffer.update()
    assert guarded.resolved_samples == ()
    assert guarded.dropped_sample_count == 1
    assert guarded.drop_reason == "episode_end_before_delay"
    assert guarded.total_resolved_sample_count == 0
    assert guarded.total_dropped_sample_count == 1
    assert buffer._last_emitted_sample_time_s == 0.8


def test_nonterminal_delay_violation_is_permanently_failed_closed() -> None:
    corridor = causal_runtime.corridor
    sample = corridor.MorphologySample(time_s=1.0, knee_rad=0.0, ankle_rad=0.0)
    resolved = corridor.ResolvedCausalMorphologySample(
        sample=sample,
        segment_type="swing",
        segment_start_time_s=0.8,
        anchor_confirmed_time_s=1.0,
        anchor_delivered_time_s=1.005,
        emitted_time_s=1.02,
        duration_basis_s=0.5,
        phase=0.7,
        terminal_flush=False,
    )
    first_update = corridor.CausalMorphologyUpdate(
        resolved_samples=(resolved,),
        pending_sample_count=1,
        total_resolved_sample_count=1,
    )
    buffer = object.__new__(causal_runtime.StrictDelayCausalMorphologyBuffer)
    buffer.delay_s = 0.04
    buffer._samples = [sample]
    buffer._failed_reason = ""
    buffer._total_resolved_sample_count = 1
    buffer._total_dropped_sample_count = 0
    buffer._total_cancelled_transition_count = 0
    buffer._last_emitted_sample_time_s = 0.8

    def frozen_update(instance: Any, *_args: Any, **_kwargs: Any) -> Any:
        if instance._failed_reason:
            return instance._snapshot(failed_closed=True)
        return first_update

    with mock.patch.object(
        causal_runtime._ORIGINAL_BUFFER, "update", new=frozen_update
    ):
        first = buffer.update()
        second = buffer.update()
    assert first.failed_closed is True
    assert first.failure_reason == "causal_delay_guard_nonterminal"
    assert first.pending_sample_count == 0
    assert buffer._failed_reason == "causal_delay_guard_nonterminal"
    assert buffer._last_emitted_sample_time_s == 0.8
    assert second.failed_closed is True
    assert second.failure_reason == "causal_delay_guard_nonterminal"


def test_launcher_propagates_opt_in_site_hook(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv(causal_runtime.SITE_MARKER_ENV, raising=False)
    monkeypatch.setenv("PYTHONPATH", "")
    observed = training_launcher.configure_process()
    assert observed["marker"] == contract.CAUSAL_RUNTIME_ID
    assert observed["pythonpath_propagated"] is True
    assert causal_runtime.assert_installed()["passed"] is True


def _site_hook_environment() -> dict[str, str]:
    env = os.environ.copy()
    env[causal_runtime.SITE_MARKER_ENV] = causal_runtime.RUNTIME_ID
    inherited = [item for item in env.get("PYTHONPATH", "").split(os.pathsep) if item]
    env["PYTHONPATH"] = os.pathsep.join([str(HERE), *inherited])
    return env


def test_sitecustomize_installs_runtime_in_real_child_process() -> None:
    code = (
        "import json,os,reward_function;"
        "c=reward_function.CausalDelayedMorphologyBuffer;"
        "print(json.dumps({'pid':os.getpid(),'runtime_id':getattr(c,'runtime_id',None)}))"
    )
    completed = subprocess.run(
        [sys.executable, "-c", code],
        cwd=runner.REPO_ROOT,
        env=_site_hook_environment(),
        check=True,
        capture_output=True,
        text=True,
        timeout=30,
    )
    observed = json.loads(completed.stdout.strip().splitlines()[-1])
    assert observed["pid"] != os.getpid()
    assert observed["runtime_id"] == contract.CAUSAL_RUNTIME_ID


def _ray_python() -> Path | None:
    candidates = [
        Path(sys.executable),
        Path("/opt/anaconda3/envs/envCMC-rllib/bin/python"),
    ]
    for candidate in candidates:
        if not candidate.is_file():
            continue
        result = subprocess.run(
            [str(candidate), "-c", "import ray"],
            capture_output=True,
            timeout=15,
        )
        if result.returncode == 0:
            return candidate
    return None


def test_sitecustomize_installs_runtime_in_real_ray_worker() -> None:
    python = _ray_python()
    if python is None:
        pytest.skip("Ray interpreter is unavailable")
    code = """
import json
import os
import ray

ray.init(include_dashboard=False, log_to_driver=False, num_cpus=1, num_gpus=0)

@ray.remote
def probe():
    import os
    import reward_function
    cls = reward_function.CausalDelayedMorphologyBuffer
    return {"pid": os.getpid(), "runtime_id": getattr(cls, "runtime_id", None)}

try:
    print("RAY_RESULT=" + json.dumps(ray.get(probe.remote())))
finally:
    ray.shutdown()
"""
    completed = subprocess.run(
        [str(python), "-c", code],
        cwd=runner.REPO_ROOT,
        env=_site_hook_environment(),
        check=False,
        capture_output=True,
        text=True,
        timeout=120,
    )
    if (
        completed.returncode != 0
        and "PermissionError" in completed.stderr
        and "sysctl" in completed.stderr
    ):
        pytest.skip("sandbox blocks Ray worker process inspection")
    assert completed.returncode == 0, completed.stderr
    line = next(
        item for item in completed.stdout.splitlines() if item.startswith("RAY_RESULT=")
    )
    observed = json.loads(line.removeprefix("RAY_RESULT="))
    assert observed["pid"] != os.getpid()
    assert observed["runtime_id"] == contract.CAUSAL_RUNTIME_ID


def test_arm_gate_fails_closed_when_runtime_injection_is_not_attested() -> None:
    case_id = contract.CASE_IDS[0]
    streams = {name: _stream(f"{case_id}:{name}") for name in contract.STREAM_NAMES}
    arm, rewards, causal = _arm(case_id, "control", 0, streams=streams)
    assert (
        gates.arm_gate(
            arm,
            expected_binding=_binding(),
            expected_condition=_condition(case_id),
            reward_rows=rewards,
            causal_rows=causal,
        )["passed"]
        is True
    )
    arm["causal_runtime"]["live_attestation"]["checks"]["reward"] = False
    rejected = gates.arm_gate(
        arm,
        expected_binding=_binding(),
        expected_condition=_condition(case_id),
        reward_rows=rewards,
        causal_rows=causal,
    )
    assert rejected["passed"] is False
    assert rejected["checks"]["causal_runtime"] is False


def test_imports_are_inert_and_canonical_execute_is_not_called() -> None:
    for path in (
        HERE / "audit_h0_v12r9_morphology_readiness.py",
        HERE / "freeze_h0_v12r9_morphology.py",
        HERE / "run_h0_v12r9_morphology.py",
        HERE / "h0_v12r9_morphology_physical_rollout.py",
    ):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        top_calls = [
            node
            for node in tree.body
            if isinstance(node, ast.Expr) and isinstance(node.value, ast.Call)
        ]
        assert not top_calls
    runner_source = (HERE / "run_h0_v12r9_morphology.py").read_text(encoding="utf-8")
    assert ".train(" not in runner_source
    assert "collect_physical_arm" in runner_source


def test_strict_json_reader_rejects_duplicate_keys(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(runner.zero_runner, "REPO_ROOT", tmp_path)
    path = tmp_path / "duplicate.json"
    path.write_text('{"a":1,"a":2}', encoding="utf-8")
    with pytest.raises(runner.MorphologyExecutionError):
        runner.strict_json_any(path)


def test_no_frozen_morphology_source_was_modified_by_additive_runtime() -> None:
    source = Path(causal_runtime.corridor.__file__).resolve()
    assert source.name == "experimental_morphology_corridor.py"
    assert hashlib.sha256(source.read_bytes()).hexdigest() == (
        contract.FROZEN_CORRIDOR_SHA256
    )
    assert source not in {
        HERE / "h0_v12r9_morphology_causal_runtime.py",
        HERE / "sitecustomize.py",
    }
