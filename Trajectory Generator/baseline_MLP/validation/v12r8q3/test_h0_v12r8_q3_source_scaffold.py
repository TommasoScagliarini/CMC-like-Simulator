"""OpenSim-free tests for the fail-closed V12R8-Q3 source scaffold."""

from __future__ import annotations

import copy
import hashlib
import sys
from collections.abc import Mapping
from pathlib import Path
from typing import Any

import pytest


LOCAL_ROOT = Path(__file__).resolve().parent
if str(LOCAL_ROOT) not in sys.path:
    sys.path.insert(0, str(LOCAL_ROOT))

import h0_v12r8_q3_artifacts as artifacts  # noqa: E402
import h0_v12r8_q3_prerequisites as prerequisites  # noqa: E402
import h0_v12r8_q3_qualification_contract as contract  # noqa: E402
import h0_v12r8_q3_qualification_gates as gates  # noqa: E402


def _digest(label: str) -> str:
    return hashlib.sha256(label.encode("utf-8")).hexdigest()


def _record(path: str, label: str | None = None, size: int = 1) -> dict[str, Any]:
    return {
        "path": path,
        "sha256": _digest(path if label is None else label),
        "size_bytes": size,
    }


def _candidate() -> tuple[str, dict[str, Any], dict[str, Any], dict[str, Any]]:
    rows = [
        {
            "path": name,
            "sha256": _digest(name),
            "size_bytes": index + 1,
        }
        for index, name in enumerate(sorted(contract.CANDIDATE_REQUIRED_FILES))
    ]
    module = {
        "path": contract.CANDIDATE_MODULE_PATH.as_posix(),
        "tree_sha256": artifacts.tree_digest(rows),
        "file_count": 5,
        "files": rows,
    }
    candidate_id = contract.r8.candidate_id(module["tree_sha256"])
    manifest = {
        "schema_version": 1,
        "status": contract.r8.ACTOR_FEATURE_MANIFEST_STATUS,
        "topology_id": contract.r8.TOPOLOGY_ID,
        "fit_contract_id": contract.r8.FIT_CONTRACT_ID,
        "actor_feature_count": 35,
        "actor_feature_names": list(contract.ACTOR_FEATURE_NAMES),
        "fcnet_hiddens": [512, 512],
        "disabled_clock_columns": [0, 1],
        "actor_digest": _digest("actor"),
        "module_state_sha256": next(
            row["sha256"] for row in rows if row["path"] == "module_state.pkl"
        ),
    }
    feature_row = next(
        row for row in rows if row["path"] == "actor_feature_manifest.json"
    )
    manifest_record = {
        "path": (
            f"{contract.CANDIDATE_MODULE_PATH.as_posix()}/actor_feature_manifest.json"
        ),
        "sha256": feature_row["sha256"],
        "size_bytes": feature_row["size_bytes"],
    }
    return candidate_id, module, manifest, manifest_record


def _semantic_bundle() -> tuple[
    dict[str, dict[str, Any]],
    dict[str, dict[str, Any]],
    dict[str, Any],
    dict[str, Any],
]:
    candidate_id, module, manifest, manifest_record = _candidate()
    requirements = contract.prerequisite_requirements()
    records = {row["name"]: _record(row["path"], row["name"]) for row in requirements}
    protocol = {
        "status": contract.r8.PROTOCOL_FREEZE_PASS_STATUS,
        "passed": True,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_module_path": contract.CANDIDATE_MODULE_PATH.as_posix(),
    }
    lock = {
        "status": contract.r8.EXECUTION_LOCK_PASS_STATUS,
        "passed": True,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_module_path": contract.CANDIDATE_MODULE_PATH.as_posix(),
    }
    candidate = {
        "status": contract.r8.CANDIDATE_FREEZE_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.r8.PROTOCOL_ID,
        "pipeline_id": contract.r8.PIPELINE_ID,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_id": candidate_id,
        "candidate_module": module,
        "actor_feature_manifest": manifest_record,
        "actor_digest": manifest["actor_digest"],
        "candidate_frozen": True,
        "fit_gate_passed": True,
        "standard_actor": True,
        "warm_start_target_512_compatible": True,
        "actor_fit_count": 1,
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "q3_paths_opened": [],
        "runtime_promoted": False,
    }
    final = {
        "status": contract.r8.DEVELOPMENT_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.r8.PROTOCOL_ID,
        "pipeline_id": contract.r8.PIPELINE_ID,
        "candidate_id": candidate_id,
        "candidate_module": module,
        "candidate_freeze": records["r8_candidate_freeze_receipt"],
        "case_gates": [
            {"case_id": case_id, "passed": True}
            for case_id in contract.r8.DEVELOPMENT_CASE_IDS
        ],
        "collection_round_count": 6,
        "actor_fit_count": 1,
        "development_count": 6,
        "passing_development_count": 6,
        "failed_development_count": 0,
        "development_actor_updates": 0,
        "development_critic_updates": 0,
        "development_ppo_updates": 0,
        "teacher_query_count": 0,
        "mean_blend_count": 0,
        "safety_intervention_count": 0,
        "q3_paths_opened": [],
        "runtime_promoted": False,
    }
    terminal = {
        "status": contract.r8.PIPELINE_TERMINAL_PASS_STATUS,
        "passed": True,
        "terminal": True,
        "protocol_id": contract.r8.PROTOCOL_ID,
        "pipeline_id": contract.r8.PIPELINE_ID,
        "candidate_id": candidate_id,
        "candidate_module": module,
        "protocol_freeze": records["r8_protocol_freeze"],
        "execution_lock": records["r8_execution_lock"],
        "candidate_freeze": records["r8_candidate_freeze_receipt"],
        "final_development_receipt": records["r8_final_development_receipt"],
        "actor_fit_count": 1,
        "actor_updates": 1,
        "collection_round_count": 6,
        "critic_updates": 0,
        "ppo_updates": 0,
        "q3_paths_opened": [],
        "runtime_promoted": False,
    }
    payloads = {
        "r8_protocol_freeze": protocol,
        "r8_execution_lock": lock,
        "r8_candidate_freeze_receipt": candidate,
        "r8_final_development_receipt": final,
        "r8_terminal_pass_ledger": terminal,
    }
    return payloads, records, manifest, manifest_record


def _morphology_ab() -> dict[str, Any]:
    return {
        "contract": copy.deepcopy(contract.MORPHOLOGY_ZERO_AB_CONTRACT),
        "profile_loaded": True,
        "causal_buffer_active": True,
        "phase_mode": contract.MORPHOLOGY_PHASE_MODE,
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "detector_sample_count": 5_000,
        "reward_sample_count": 500,
        "action_sample_count": 500,
        "observation_sample_count": 500,
        "corridor_evaluation_count": 496,
        "corridor_unavailable_count": 4,
        "baseline_reward_bytes_sha256": _digest("reward"),
        "candidate_reward_bytes_sha256": _digest("reward"),
        "reward_bit_mismatch_count": 0,
        "baseline_action_bytes_sha256": _digest("action"),
        "candidate_action_bytes_sha256": _digest("action"),
        "action_bit_mismatch_count": 0,
        "baseline_observation_bytes_sha256": _digest("observation"),
        "candidate_observation_bytes_sha256": _digest("observation"),
        "observation_bit_mismatch_count": 0,
        "morphology_weight": 0.0,
        "morphology_causal_allow_effects": 0.0,
        "morphology_hard_termination_enabled": 0.0,
        "morphology_term_nonzero_count": 0,
        "morphology_hard_termination_count": 0,
        "corridor_finite_count": 496,
        "corridor_nonfinite_count": 0,
        "passed": True,
    }


def _v26() -> dict[str, Any]:
    return {
        "passed": True,
        "sample_count": 5_000,
        "detector_profile": contract.DETECTOR_PROFILE_PATH.as_posix(),
        "detector_profile_sha256": contract.DETECTOR_PROFILE_SHA256,
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "actor_event_source": contract.V26_ACTOR_EVENT_SOURCE,
        "binary_phase_fsm_mode": "binary_active",
        "duplicate_event_count": 0,
        "out_of_order_event_count": 0,
        "left_non_v26_source_count": 0,
        "fallback_count": 0,
        "hard_invalid_count": 0,
        "terminal_pending_rule": "drop_samples_at_or_after_pending_onset",
        "terminal_pending_sample_drop_count": 2,
    }


def _rollout_summary(role: str) -> dict[str, Any]:
    candidate_id, module, _manifest, _manifest_record = _candidate()
    case = contract.canonical_rollout(role, contract.CASE_IDS[0])
    root = case["destination"]
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.ROLLOUT_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": candidate_id,
        "actor_module": module
        if role == contract.CANDIDATE_ROLE
        else copy.deepcopy(contract.SOURCE_H0_MODULE),
        "role": role,
        "case_id": case["case_id"],
        "action_selection": case["action_selection"],
        "episode_start_offset_s": case["episode_start_offset_s"],
        "action_seed": case["action_seed"],
        "runtime_seed": case["runtime_seed"],
        "sigma": case["sigma"],
        "resolved_env_config": copy.deepcopy(case["resolved_env_config"]),
        "n_actor": 35,
        "n_observation": 84,
        "observation_dtype": "float32",
        "action_shape": [2],
        "action_dtype": "float32",
        "fcnet_hiddens": [512, 512],
        "teacher_enabled": False,
        "teacher_loaded_during_rollout": False,
        "blending_enabled": False,
        "safety_latch_enabled": False,
        "actor_query_count": 500,
        "steps": 500,
        "trace_step_count": 500,
        "control_window_count": 5_000,
        "raw_sensor_sample_count": 5_000,
        "end_reason": "episode_time_limit",
        "terminated": False,
        "truncated": True,
        "phase_valid_cycle_count": 2,
        "grf_penetration_max_m": 0.024,
        "binary_phase_event_gate": _v26(),
        "legacy_event_integrity_passed": True,
        "morphology_zero_ab": _morphology_ab(),
        "morphology_disabled_bit_identity_passed": True,
        "random_noise_draw_count": 0,
        "single_noise_application_count": 500,
        "noise_tape": _record(case["noise_tape"], "noise"),
        "noise_tape_array_sha256": contract.EXPECTED_TAPE_ARRAY_SHA256[
            "deterministic_all_zero.npz"
        ],
        "protocol_freeze": _record(contract.PROTOCOL_FREEZE_PATH.as_posix()),
        "execution_lock": _record(contract.EXECUTION_LOCK_PATH.as_posix()),
        "pipeline_claim": _record(contract.PIPELINE_CLAIM_PATH.as_posix()),
        "run_start": _record(f"{root}/run_start.json"),
        "trace": _record(f"{root}/trace.json"),
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "morphology_weight": 0.0,
        "positive_morphology_enabled": False,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "post_hoc_tuning_authorized": False,
        "compensation_or_averaging_used": False,
        "comparison_metrics": {
            name: 1.0
            for name, _absolute, _relative in contract.NONINFERIORITY_TOLERANCES
        },
    }
    summary.update({name: 0 for name in contract.ZERO_REQUIRED_COUNTS})
    return summary


def test_contract_is_source_only_deferred_and_baseline_first() -> None:
    audit = contract.contract_self_check()
    assert audit["passed"] is True
    assert contract.SCHEMA_VERSION == 1283
    assert contract.LINEAGE_STATE == "DEFERRED_UNTIL_R8_TERMINAL_PASS"
    assert contract.PROTOCOL_ID == (
        "AB06_H0_V12R8_Q3_V26_MORPHOLOGY_ZERO_QUALIFICATION"
    )
    assert contract.PIPELINE_ID == "H0_V12R8_Q3_BASELINE_FIRST_SIX_CASE_PAIRED"
    assert contract.ROOT.as_posix().endswith("/v12r8q3")
    assert contract.CANDIDATE_ID is None
    assert contract.CANDIDATE_MODULE is None
    assert [row["role"] for row in contract.ROLLOUT_MATRIX] == [
        *(["baseline"] * 6),
        *(["candidate"] * 6),
    ]
    assert contract.CANDIDATE_RESOLVED_ENV_CONFIG == {
        **contract.CANDIDATE_RESOLVED_ENV_CONFIG,
        "binary_phase_fsm_mode": "binary_active",
        "morphology_phase_mode": ("event_anchored_causal_delayed_experimental"),
        "morphology_reward_delay_s": 0.04,
        "morphology_max_delivery_latency_s": 0.01,
        "morphology_causal_allow_effects": 0.0,
        "morphology_weight": 0.0,
        "morphology_hard_termination_enabled": 0.0,
    }


def test_canonical_sources_contain_no_predecessor_lineage_literals() -> None:
    for path in LOCAL_ROOT.rglob("*.py"):
        if path.name.startswith("test_"):
            continue
        source = path.read_text(encoding="utf-8")
        assert "V12R7" not in source
        assert "v12r7" not in source


def test_candidate_identity_uses_the_r8_prefix_only() -> None:
    candidate_id, module, _manifest, _manifest_record = _candidate()
    assert candidate_id == f"AB06_H0_V12R8_RECOVERY_W512:{module['tree_sha256']}"
    predecessor_id = f"AB06_H0_V12R7_RECOVERY_W512:{module['tree_sha256']}"
    with pytest.raises(artifacts.V12R8Q3ArtifactError):
        prerequisites.validate_candidate_tree(predecessor_id, module)


@pytest.mark.parametrize(
    "path",
    (
        "",
        ".",
        "/absolute/file.json",
        "../escape.json",
        "root/../escape.json",
        "root\\windows.json",
        "C:/drive.json",
        "root//double.json",
        "./root/file.json",
    ),
)
def test_path_hardening_rejects_noncanonical_forms(path: str) -> None:
    assert artifacts.canonical_relative_path(path) is False


def test_artifact_record_rejects_bool_size_and_uppercase_hash() -> None:
    path = "safe/file.json"
    assert artifacts.artifact_record_matches(_record(path), path)
    bool_size = _record(path)
    bool_size["size_bytes"] = True
    assert not artifacts.artifact_record_matches(bool_size, path)
    uppercase = _record(path)
    uppercase["sha256"] = uppercase["sha256"].upper()
    assert not artifacts.artifact_record_matches(uppercase, path)


def test_candidate_tree_and_actor_manifest_are_exact() -> None:
    candidate_id, module, manifest, manifest_record = _candidate()
    assert prerequisites.validate_candidate_tree(candidate_id, module) == {
        "candidate_id": candidate_id,
        "candidate_module": module,
    }
    assert (
        prerequisites.validate_actor_feature_manifest(
            manifest,
            manifest_record=manifest_record,
            candidate_module=module,
        )
        == manifest
    )

    wrong_file = copy.deepcopy(module)
    wrong_file["files"][0]["path"] = "..\\actor_feature_manifest.json"
    with pytest.raises(artifacts.V12R8Q3ArtifactError):
        prerequisites.validate_candidate_tree(candidate_id, wrong_file)

    bool_count = copy.deepcopy(module)
    bool_count["file_count"] = True
    with pytest.raises(artifacts.V12R8Q3ArtifactError):
        prerequisites.validate_candidate_tree(candidate_id, bool_count)

    wrong_width = copy.deepcopy(manifest)
    wrong_width["fcnet_hiddens"] = [256, 256]
    with pytest.raises(artifacts.V12R8Q3ArtifactError):
        prerequisites.validate_actor_feature_manifest(
            wrong_width,
            manifest_record=manifest_record,
            candidate_module=module,
        )


def test_actor_manifest_schema_is_local_and_cross_bound_to_r8() -> None:
    _candidate_id, module, manifest, manifest_record = _candidate()
    assert manifest["schema_version"] == 1
    assert manifest["schema_version"] != contract.r8.SCHEMA_VERSION
    assert manifest["status"] == contract.r8.ACTOR_FEATURE_MANIFEST_STATUS
    assert manifest["topology_id"] == contract.r8.TOPOLOGY_ID
    assert manifest["fit_contract_id"] == contract.r8.FIT_CONTRACT_ID
    assert set(manifest) == set(contract.ACTOR_FEATURE_MANIFEST_KEYS)
    assert (
        prerequisites.validate_actor_feature_manifest(
            manifest,
            manifest_record=manifest_record,
            candidate_module=module,
        )["schema_version"]
        == 1
    )


def test_deferred_state_and_rollout_gate_fail_closed() -> None:
    prerequisites.clear_candidate_binding_for_tests()
    state = prerequisites.deferred_prerequisite_state()
    assert state["passed"] is False
    assert len(state["missing_prerequisites"]) == 5
    assert "missing_runtime_sources" not in state
    assert set(state["declared_runtime_sources"]) == set(
        contract.DEFERRED_RUNTIME_SOURCE_PATHS
    )
    assert (
        state["runtime_source_verification"] == "DEFERRED_TO_SOURCE_CLOSURE_READINESS"
    )
    with pytest.raises(prerequisites.V12R8Q3PrerequisiteError):
        prerequisites.current_candidate_binding()
    gate = gates.common_rollout_gate(
        {}, role=contract.CANDIDATE_ROLE, case_id=contract.CASE_IDS[0]
    )
    assert gate["passed"] is False
    assert gate["checks"] == {"r8_candidate_bound": False}


def test_source_closure_requires_every_deferred_runtime_hook() -> None:
    expected = {
        **contract.QUALIFICATION_SOURCE_PATHS,
        **contract.DEFERRED_RUNTIME_SOURCE_PATHS,
        **contract.QUALIFICATION_INPUT_PATHS,
    }
    records = {name: _record(path, name) for name, path in expected.items()}
    for name, digest in contract.QUALIFICATION_INPUT_SHA256.items():
        records[name]["sha256"] = digest
    assert prerequisites.validate_source_closure(records)["passed"] is True
    missing = copy.deepcopy(records)
    missing.pop("q3_runner")
    result = prerequisites.validate_source_closure(missing)
    assert result["passed"] is False
    assert result["checks"]["exact_keys"] is False

    wrong_hash = copy.deepcopy(records)
    wrong_hash["detector_profile"]["sha256"] = _digest("wrong-profile")
    result = prerequisites.validate_source_closure(wrong_hash)
    assert result["passed"] is False
    assert result["checks"]["input_hash::detector_profile"] is False


def test_semantic_prerequisite_bundle_binds_only_exact_terminal_r8() -> None:
    prerequisites.clear_candidate_binding_for_tests()
    payloads, records, manifest, manifest_record = _semantic_bundle()
    result = prerequisites.validate_verified_payloads(
        payloads,
        records,
        actor_manifest=manifest,
        actor_manifest_record=manifest_record,
    )
    assert result["passed"] is True
    assert result["official_verifier_count"] == 5
    assert (
        prerequisites.current_candidate_binding()["candidate_id"]
        == result["candidate_id"]
    )

    prerequisites.clear_candidate_binding_for_tests()
    tampered = copy.deepcopy(payloads)
    tampered["r8_final_development_receipt"]["case_gates"][0]["passed"] = 1
    failed = prerequisites.validate_verified_payloads(
        tampered,
        records,
        actor_manifest=manifest,
        actor_manifest_record=manifest_record,
    )
    assert failed["passed"] is False
    assert failed["checks"]["final_receipt_semantic"] is False
    with pytest.raises(prerequisites.V12R8Q3PrerequisiteError):
        prerequisites.current_candidate_binding()


def test_morphology_zero_ab_requires_live_corridor_and_bit_identity() -> None:
    valid = _morphology_ab()
    assert gates.morphology_zero_ab_gate(valid)["passed"] is True

    drifted = copy.deepcopy(valid)
    drifted["candidate_reward_bytes_sha256"] = _digest("changed")
    result = gates.morphology_zero_ab_gate(drifted)
    assert result["passed"] is False
    assert result["checks"]["reward_bit_identity"] is False

    bool_counter = copy.deepcopy(valid)
    bool_counter["reward_bit_mismatch_count"] = False
    result = gates.morphology_zero_ab_gate(bool_counter)
    assert result["passed"] is False
    assert result["checks"]["reward_bit_identity"] is False

    inactive = copy.deepcopy(valid)
    inactive["causal_buffer_active"] = False
    result = gates.morphology_zero_ab_gate(inactive)
    assert result["passed"] is False
    assert result["checks"]["live_causal_corridor"] is False

    bool_weight = copy.deepcopy(valid)
    bool_weight["morphology_weight"] = False
    bool_weight["contract"]["morphology_weight"] = False
    result = gates.morphology_zero_ab_gate(bool_weight)
    assert result["passed"] is False
    assert result["checks"]["contract"] is False
    assert result["checks"]["zero_effects"] is False


def test_v26_gate_requires_exact_profile_contract_and_5000_strict_samples() -> None:
    valid = _v26()
    assert gates.v26_event_gate(valid)["passed"] is True

    wrong_profile = copy.deepcopy(valid)
    wrong_profile["detector_profile_sha256"] = _digest("wrong")
    result = gates.v26_event_gate(wrong_profile)
    assert result["passed"] is False
    assert result["checks"]["profile"] is False

    bool_sample_count = copy.deepcopy(valid)
    bool_sample_count["sample_count"] = True
    result = gates.v26_event_gate(bool_sample_count)
    assert result["passed"] is False
    assert result["checks"]["samples"] is False

    wrong_terminal_rule = copy.deepcopy(valid)
    wrong_terminal_rule["terminal_pending_rule"] = "flush_pending"
    result = gates.v26_event_gate(wrong_terminal_rule)
    assert result["passed"] is False
    assert result["checks"]["terminal_pending_policy"] is False


def test_candidate_resolved_env_config_is_exact_and_portable() -> None:
    config = contract.CANDIDATE_RESOLVED_ENV_CONFIG
    assert config["binary_phase_detector_profile"] == (
        contract.DETECTOR_PROFILE_PATH.as_posix()
    )
    assert artifacts.canonical_relative_path(config["binary_phase_detector_profile"])
    assert config["binary_phase_event_contract_id"] == contract.EVENT_CONTRACT_ID
    assert config["morphology_profile_sha256"] == contract.MORPHOLOGY_PROFILE_SHA256
    assert config["morphology_causal_allow_effects"] == 0.0
    assert config["morphology_experimental_allow_effects"] == 0.0


def test_common_and_pair_gates_enforce_resolved_config_and_zero_morphology() -> None:
    prerequisites.clear_candidate_binding_for_tests()
    candidate_id, module, _manifest, _manifest_record = _candidate()
    prerequisites.bind_candidate(candidate_id, module)
    baseline = _rollout_summary(contract.BASELINE_ROLE)
    candidate = _rollout_summary(contract.CANDIDATE_ROLE)
    case_id = contract.CASE_IDS[0]
    assert (
        gates.common_rollout_gate(
            baseline, role=contract.BASELINE_ROLE, case_id=case_id
        )["passed"]
        is True
    )
    assert (
        gates.common_rollout_gate(
            candidate, role=contract.CANDIDATE_ROLE, case_id=case_id
        )["passed"]
        is True
    )
    assert (
        gates.condition_matched_gate(baseline, candidate, case_id=case_id)["passed"]
        is True
    )

    drifted = copy.deepcopy(candidate)
    drifted["resolved_env_config"]["morphology_reward_delay_s"] = 0.03
    result = gates.common_rollout_gate(
        drifted, role=contract.CANDIDATE_ROLE, case_id=case_id
    )
    assert result["passed"] is False
    assert result["checks"]["resolved_env_config"] is False

    bool_alias = copy.deepcopy(candidate)
    bool_alias["resolved_env_config"]["morphology_causal_allow_effects"] = False
    result = gates.common_rollout_gate(
        bool_alias, role=contract.CANDIDATE_ROLE, case_id=case_id
    )
    assert result["passed"] is False
    assert result["checks"]["resolved_env_config"] is False
    prerequisites.clear_candidate_binding_for_tests()


def test_aggregate_handoff_interface_is_exact_for_checkpoint_zero() -> None:
    prerequisites.clear_candidate_binding_for_tests()
    candidate_id, module, _manifest, _manifest_record = _candidate()
    prerequisites.bind_candidate(candidate_id, module)
    pair_bindings = [
        {
            "case_id": case_id,
            "passed": True,
            "pair_gate": _record(contract.pair_gate_path(case_id).as_posix()),
            "baseline_receipt": _record(
                contract.rollout_receipt_path(
                    contract.BASELINE_ROLE, case_id
                ).as_posix()
            ),
            "candidate_receipt": _record(
                contract.rollout_receipt_path(
                    contract.CANDIDATE_ROLE, case_id
                ).as_posix()
            ),
        }
        for case_id in contract.CASE_IDS
    ]
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.AGGREGATE_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": candidate_id,
        "candidate_module": module,
        "rollout_matrix": list(contract.ROLLOUT_MATRIX),
        "baseline_rollout_count": 6,
        "candidate_rollout_count": 6,
        "total_rollout_count": 12,
        "pair_bindings": pair_bindings,
        "pair_count": 6,
        "passing_pair_count": 6,
        "failed_pair_count": 0,
        "compensation_or_averaging_used": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "morphology_weight": 0.0,
        "positive_morphology_enabled": False,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "post_hoc_tuning_authorized": False,
    }
    result = gates.aggregate_qualification_gate(summary)
    assert result["passed"] is True
    assert result["status"] == contract.AGGREGATE_PASS_STATUS
    assert result["next_stage"] == contract.NEXT_STAGE_AFTER_Q3_PASS
    assert contract.FINAL_RECEIPT_PATH == contract.FINAL_ROOT / "receipt.json"
    assert (
        contract.PIPELINE_TERMINAL_PASS_STATUS == "PASS_H0_V12R8_Q3_PIPELINE_TERMINAL"
    )
    prerequisites.clear_candidate_binding_for_tests()


def test_invoke_verifier_hooks_rejects_non_mapping_result() -> None:
    payloads, _records, _manifest, _manifest_record = _semantic_bundle()
    hooks = prerequisites.R8SemanticVerifierHooks(
        verify_protocol_freeze=lambda: payloads["r8_protocol_freeze"],
        verify_execution_lock=lambda: payloads["r8_execution_lock"],
        verify_candidate_freeze_receipt=lambda: payloads["r8_candidate_freeze_receipt"],
        verify_final_development_receipt=lambda: payloads[
            "r8_final_development_receipt"
        ],
        verify_terminal_ledger=lambda: None,  # type: ignore[arg-type,return-value]
    )
    with pytest.raises(prerequisites.V12R8Q3PrerequisiteError):
        prerequisites.invoke_verifier_hooks(hooks)


def test_payload_order_is_part_of_prerequisite_contract() -> None:
    prerequisites.clear_candidate_binding_for_tests()
    payloads, records, manifest, manifest_record = _semantic_bundle()
    reordered = {name: payloads[name] for name in reversed(tuple(payloads))}
    result = prerequisites.validate_verified_payloads(
        reordered,
        records,
        actor_manifest=manifest,
        actor_manifest_record=manifest_record,
    )
    assert result["passed"] is False
    assert result["checks"]["exact_five_ordered_payloads"] is False


def test_non_mapping_verifier_payload_fails_closed_without_exception() -> None:
    prerequisites.clear_candidate_binding_for_tests()
    payloads, records, manifest, manifest_record = _semantic_bundle()
    payloads["r8_protocol_freeze"] = None  # type: ignore[assignment]
    result = prerequisites.validate_verified_payloads(
        payloads,
        records,
        actor_manifest=manifest,
        actor_manifest_record=manifest_record,
    )
    assert result["passed"] is False
    assert result["checks"]["status::r8_protocol_freeze"] is False
    with pytest.raises(prerequisites.V12R8Q3PrerequisiteError):
        prerequisites.current_candidate_binding()


def test_semantic_helpers_do_not_accept_mapping_substitutes_for_true() -> None:
    payloads, _records, manifest, manifest_record = _semantic_bundle()
    candidate = copy.deepcopy(payloads["r8_candidate_freeze_receipt"])
    candidate["passed"] = 1
    result = prerequisites.candidate_freeze_semantics(
        candidate,
        actor_manifest=manifest,
        actor_manifest_record=manifest_record,
    )
    assert result["passed"] is False
    assert result["checks"]["status"] is False


def test_exact_candidate_tree_rejects_duplicate_and_extra_files() -> None:
    candidate_id, module, _manifest, _record_value = _candidate()
    duplicate = copy.deepcopy(module)
    duplicate["files"][-1] = copy.deepcopy(duplicate["files"][0])
    duplicate["tree_sha256"] = artifacts.tree_digest(duplicate["files"])
    with pytest.raises(artifacts.V12R8Q3ArtifactError):
        prerequisites.validate_candidate_tree(candidate_id, duplicate)

    extra = copy.deepcopy(module)
    extra["files"].append(
        {"path": "optimizer.pkl", "sha256": _digest("optimizer"), "size_bytes": 1}
    )
    extra["file_count"] = 6
    extra["tree_sha256"] = artifacts.tree_digest(extra["files"])
    with pytest.raises(artifacts.V12R8Q3ArtifactError):
        prerequisites.validate_candidate_tree(candidate_id, extra)


def test_artifact_record_requires_exact_schema() -> None:
    value: Mapping[str, Any] = _record("safe/file.json")
    with_extra = {**value, "kind": "file"}
    assert not artifacts.artifact_record_matches(with_extra, "safe/file.json")
