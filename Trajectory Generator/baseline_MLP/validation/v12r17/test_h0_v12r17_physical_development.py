"""Semantic, execution-free tests for the V12R17 P3 physical runner."""

from __future__ import annotations

import copy
import json
import sys
from pathlib import Path
from typing import Any, Mapping

import pytest


LOCAL_ROOT = Path(__file__).resolve().parent
if str(LOCAL_ROOT) not in sys.path:
    sys.path.insert(0, str(LOCAL_ROOT))

import h0_v12r17_physical_development as contract  # noqa: E402
import run_h0_v12r17_physical_development as runner  # noqa: E402


def _trace_row(case_id: str, step: int) -> dict[str, Any]:
    return {
        "step": step,
        "schema_version": contract.SCHEMA_VERSION,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": f"development__{case_id}",
        "case_id": case_id,
        "v26_observation": [0.0] * contract.EXPECTED_ACTOR_FEATURES,
        "candidate_mean": [0.1, -0.2],
        "candidate_std": [contract.EXPECTED_SIGMA] * 2,
        "standard_normal": [0.0, 0.0],
        "single_noise": [0.0, 0.0],
        "raw_action": [0.1, -0.2],
        "applied_action": [0.1, -0.2],
        "teacher_enabled": False,
        "blending_enabled": False,
        "safety_latch_enabled": False,
        **{name: 0 for name in contract.PURE_POLICY_COUNTER_FIELDS},
        "raw_sensor_sample_count": contract.EXPECTED_RAW_SENSOR_SAMPLES_PER_STEP,
        "observer_raw_sensor_journal": {
            "samples": [
                {"sensor_index": index}
                for index in range(1, contract.EXPECTED_RAW_SENSOR_SAMPLES_PER_STEP + 1)
            ]
        },
        "phase_fsm": {"event_source": contract.EVENT_SOURCE},
        "checks": {name: True for name in contract.REQUIRED_STEP_CHECKS},
        "grf_penetration_m": 0.01,
        "time_s": step * 0.01,
        "terminated": False,
        "truncated": step == contract.EXPECTED_STEPS,
        "end_reason": (
            "episode_time_limit" if step == contract.EXPECTED_STEPS else None
        ),
    }


def _passing_trace(case_id: str) -> list[dict[str, Any]]:
    return [_trace_row(case_id, step) for step in range(1, contract.EXPECTED_STEPS + 1)]


def _passing_summary(case_id: str, trace: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "case": contract.canonical_case(case_id),
        "p2_candidate_only": True,
        "p0_used": False,
        "pure_policy_trace_audit": {
            "passed": True,
            "row_count": contract.EXPECTED_STEPS,
        },
        "steps": contract.EXPECTED_STEPS,
        "control_window_count": contract.EXPECTED_CONTROL_WINDOWS,
        "raw_sensor_sample_count": contract.EXPECTED_RAW_SENSOR_SAMPLES,
        "end_reason": "episode_time_limit",
        "terminated": False,
        "truncated": True,
        "phase_valid_cycle_count": contract.MINIMUM_VALID_CYCLES,
        "grf_penetration_max_m": contract.PENETRATION_LIMIT_M - 1.0e-9,
        **{name: 0 for name in contract.ZERO_INVALID_SUMMARY_FIELDS},
        "sea_reserve_gate_passed": True,
        "binary_phase_event_gate": {
            "passed": True,
            "sample_count": contract.EXPECTED_RAW_SENSOR_SAMPLES,
            "duplicate_event_count": 0,
            "out_of_order_event_count": 0,
            "left_non_v26_source_count": 0,
            "fallback_count": 0,
            "hard_invalid_count": 0,
        },
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "target_contract_id": contract.TARGET_CONTRACT_ID,
        "detector_or_fsm_modified": False,
        "morphology_weight": 0.0,
        "teacher_enabled": False,
        "blending_enabled": False,
        "safety_latch_enabled": False,
        **{name: 0 for name in contract.PURE_POLICY_COUNTER_FIELDS},
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "q2_paths_opened": [],
        "q3_paths_opened": [],
    }


def _paths(tmp_path: Path) -> contract.DevelopmentPaths:
    revision = tmp_path / "Trajectory Generator/baseline_MLP/validation/v12r17"
    run_root = revision / "h0_v12r17_run_20260816"
    p2 = run_root / "p2_fit"
    return contract.DevelopmentPaths(
        artifact_root=tmp_path,
        revision_root=revision,
        run_root=run_root,
        p2_output=p2,
        p2_candidate=p2 / "rl_module_p2_masked_candidate_exposed",
        protocol_freeze=revision / "h0_v12r17_p3_protocol_freeze.json",
        execution_lock=revision / "h0_v12r17_p3_execution_lock.json",
        p3_root=run_root / "p3_development",
    )


def _write_json(path: Path, value: Mapping[str, Any] | list[Any]) -> None:
    contract.write_json_exclusive(path, value)


def _mapping(path: Path) -> dict[str, Any]:
    value = contract.strict_json_load(path)
    assert isinstance(value, dict)
    return value


def _overwrite_json(path: Path, value: Mapping[str, Any]) -> None:
    path.write_text(
        json.dumps(value, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8"
    )


def _fake_p2(paths: contract.DevelopmentPaths) -> dict[str, Any]:
    paths.p2_candidate.mkdir(parents=True)
    for name in ("class_and_ctor_args.pkl", "metadata.json", "module_state.pkl"):
        (paths.p2_candidate / name).write_bytes(name.encode("ascii"))
    _write_json(
        paths.p2_candidate / "actor_feature_manifest.json",
        {
            "fit_stage": "p2",
            "fcnet_hiddens": [256, 256],
            "actor_feature_count": contract.EXPECTED_ACTOR_FEATURES,
            "topology_id": contract.TOPOLOGY_ID,
            "standard_rlmodule": True,
            "fcnet_activation": "tanh",
            "legacy_shadow_runtime_dependency": False,
        },
    )
    _write_json(
        paths.p2_candidate / "candidate_build_manifest.json",
        {
            "fit_stage": "p2",
            "protocol_id": contract.PROTOCOL_ID,
            "no_teacher_runtime_dependency": True,
        },
    )
    tree = contract.tree_record(paths.p2_candidate, artifact_root=paths.artifact_root)
    candidate_id = f"{contract.PROTOCOL_ID}:{tree['tree_sha256'][:16]}"
    (paths.p2_output / "corpus.npz").write_bytes(b"p2-corpus")
    summary = {
        "status": contract.P2_SUMMARY_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "fit_stage": "p2",
        "candidate_id": candidate_id,
        "candidate_module": tree,
        "collector_only": False,
        "non_promotable": False,
        "candidate_promoted": False,
        "next_stage": contract.P2_NEXT_STAGE,
        "environment_steps": 0,
        "policy_rollouts": 0,
    }
    gate = {
        "status": contract.P2_GATE_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "fit_stage": "p2",
    }
    _write_json(paths.p2_summary, summary)
    _write_json(paths.p2_gate, gate)
    receipt = {
        "status": contract.P2_RECEIPT_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "fit_stage": "p2",
        "candidate_id": candidate_id,
        "artifacts": {
            "corpus": contract.artifact_record(
                paths.p2_output / "corpus.npz", artifact_root=paths.artifact_root
            ),
            "candidate_module": tree,
            "summary": contract.artifact_record(
                paths.p2_summary, artifact_root=paths.artifact_root
            ),
            "gate": contract.artifact_record(
                paths.p2_gate, artifact_root=paths.artifact_root
            ),
        },
        "environment_steps": 0,
        "policy_rollouts": 0,
        "retry_authorized": False,
        "collector_only": False,
        "non_promotable": False,
        "candidate_promoted": False,
        "next_stage": contract.P2_NEXT_STAGE,
    }
    _write_json(paths.p2_receipt, receipt)
    return contract.verify_p2(paths)


def test_protocol_order_and_describe_are_execution_inert(tmp_path: Path) -> None:
    assert contract.DEVELOPMENT_CASE_IDS == (
        "deterministic_offset_plus_0p20",
        "deterministic_offset_minus_0p20",
        "deterministic_offset_nominal",
        "stochastic_nominal_seed_126",
        "stochastic_nominal_seed_128",
    )
    assert contract.KNOWN_OPEN_BOUNDARY_CASES == ("stochastic_nominal_seed_127",)
    assert contract.DEVELOPMENT_CASE_COUNT == 5
    described = contract.describe_protocol(_paths(tmp_path))
    assert described["passed"] is True
    assert described["environment_calls"] == 0
    assert described["policy_queries"] == 0
    assert described["artifacts_written"] == 0
    assert not _paths(tmp_path).revision_root.exists()


def test_case_gate_is_strict_and_causally_pure() -> None:
    case_id = contract.DISCRIMINATOR_CASE_ID
    trace = _passing_trace(case_id)
    summary = _passing_summary(case_id, trace)
    assert contract.development_gate(summary, case_id=case_id, trace=trace)["passed"]

    at_limit = copy.deepcopy(summary)
    at_limit["grf_penetration_max_m"] = contract.PENETRATION_LIMIT_M
    assert not contract.development_gate(at_limit, case_id=case_id, trace=trace)[
        "passed"
    ]

    teacher = copy.deepcopy(trace)
    teacher[0]["teacher_mean"] = [0.0, 0.0]
    assert not contract.trace_audit(teacher, case_id=case_id)["passed"]

    clipped = copy.deepcopy(trace)
    clipped[0]["applied_action"][0] = -0.1
    assert not contract.trace_audit(clipped, case_id=case_id)["passed"]

    wrong_detector = copy.deepcopy(trace)
    wrong_detector[0]["phase_fsm"]["event_source"] = "legacy"
    assert not contract.trace_audit(wrong_detector, case_id=case_id)["passed"]


def test_stochastic_trace_requires_single_sigma_noise() -> None:
    case_id = "stochastic_nominal_seed_126"
    trace = _passing_trace(case_id)
    trace[0]["standard_normal"] = [2.0, -1.0]
    trace[0]["single_noise"] = [0.01, -0.005]
    trace[0]["raw_action"] = [0.11, -0.205]
    trace[0]["applied_action"] = [0.11, -0.205]
    assert contract.trace_audit(trace, case_id=case_id)["passed"]
    trace[0]["single_noise"][0] = 0.02
    assert not contract.trace_audit(trace, case_id=case_id)["passed"]


def test_aggregate_gate_requires_exact_order_and_activity() -> None:
    bindings = [
        {"case_id": case_id, "passed": True}
        for case_id in contract.DEVELOPMENT_CASE_IDS
    ]
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "protocol_id": contract.PROTOCOL_ID,
        "case_bindings": bindings,
        "candidate_tree_unique_count": 1,
        "p0_used": False,
        "development_count": contract.DEVELOPMENT_CASE_COUNT,
        "environment_reset_calls": contract.DEVELOPMENT_CASE_COUNT,
        "environment_step_calls": contract.DEVELOPMENT_CASE_COUNT
        * contract.EXPECTED_STEPS,
        "raw_sensor_sample_count": contract.DEVELOPMENT_CASE_COUNT
        * contract.EXPECTED_RAW_SENSOR_SAMPLES,
        "actor_query_count": contract.DEVELOPMENT_CASE_COUNT * contract.EXPECTED_STEPS,
        **{name: 0 for name in contract.PURE_POLICY_COUNTER_FIELDS},
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "q3_paths_opened": [],
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
    }
    assert contract.aggregate_gate(summary)["passed"]
    summary["case_bindings"] = list(reversed(bindings))
    assert not contract.aggregate_gate(summary)["passed"]


def test_p2_adapter_accepts_only_canonical_p2_w256(tmp_path: Path) -> None:
    paths = _paths(tmp_path)
    p2 = _fake_p2(paths)
    assert p2["passed"] is True
    assert p2["p0_used"] is False
    assert p2["manifest_checks"]["standard_w256"] is True

    p0_paths = contract.DevelopmentPaths(
        artifact_root=paths.artifact_root,
        revision_root=paths.revision_root,
        run_root=paths.run_root,
        p2_output=paths.p2_output,
        p2_candidate=paths.p2_candidate.with_name("rl_module_p0_forbidden"),
        protocol_freeze=paths.protocol_freeze,
        execution_lock=paths.execution_lock,
        p3_root=paths.p3_root,
    )
    paths.p2_candidate.rename(p0_paths.p2_candidate)
    with pytest.raises(contract.V12R17PhysicalDevelopmentError, match="P2 candidate"):
        contract.verify_p2(p0_paths)


@pytest.mark.parametrize(
    ("artifact", "field", "bad_value"),
    (
        ("receipt", "status", "PASS_H0_V12R17_MASKED_SAFE_TEACHER_FIT_RECEIPT"),
        ("gate", "fit_stage", "p0"),
        ("summary", "collector_only", True),
    ),
)
def test_p2_adapter_rejects_status_stage_and_collector_aliases(
    tmp_path: Path, artifact: str, field: str, bad_value: Any
) -> None:
    paths = _paths(tmp_path / artifact)
    _fake_p2(paths)
    target = {
        "receipt": paths.p2_receipt,
        "gate": paths.p2_gate,
        "summary": paths.p2_summary,
    }[artifact]
    payload = _mapping(target)
    payload[field] = bad_value
    _overwrite_json(target, payload)
    with pytest.raises(
        contract.V12R17PhysicalDevelopmentError,
        match="P2 public terminal contract",
    ):
        contract.verify_p2(paths)


def test_execute_closes_first_failed_prefix_without_loading_environment(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    paths = _paths(tmp_path)
    paths.revision_root.mkdir(parents=True)
    paths.p2_candidate.mkdir(parents=True)
    (paths.p2_candidate / "module_state.pkl").write_bytes(b"p2")
    _write_json(paths.protocol_freeze, {"placeholder": True})
    _write_json(paths.execution_lock, {"placeholder": True})
    p2_tree = contract.tree_record(
        paths.p2_candidate, artifact_root=paths.artifact_root
    )
    p2 = {
        "passed": True,
        "candidate_id": "test-p2",
        "candidate_module": p2_tree,
        "receipt": {"path": "p2-receipt", "sha256": "a" * 64, "size_bytes": 1},
        "gate": {"path": "p2-gate", "sha256": "b" * 64, "size_bytes": 1},
    }
    locked = {
        "passed": True,
        "freeze": contract.artifact_record(
            paths.protocol_freeze, artifact_root=paths.artifact_root
        ),
        "execution_lock": contract.artifact_record(
            paths.execution_lock, artifact_root=paths.artifact_root
        ),
        "p2": p2,
        "production_source_closure_sha256": "c" * 64,
    }
    monkeypatch.setattr(
        contract,
        "preflight",
        lambda _paths: {
            "passed": True,
            "locked_inputs": locked,
        },
    )
    monkeypatch.setattr(
        contract,
        "verify_execution_lock",
        lambda _paths, p2=None: copy.deepcopy(locked),
    )
    monkeypatch.setattr(
        runner,
        "_load_physical_runtime",
        lambda: (_ for _ in ()).throw(AssertionError("environment import")),
    )

    calls: list[str] = []

    def failed_case_runner(**kwargs: Any) -> Mapping[str, Any]:
        calls.append(str(kwargs["case"]["case_id"]))
        destination = Path(kwargs["destination"])
        destination.mkdir()
        _write_json(
            destination / "run_start.json",
            {
                "schema_version": contract.SCHEMA_VERSION,
                "protocol_id": contract.PROTOCOL_ID,
            },
        )
        row = _trace_row(str(kwargs["case"]["case_id"]), 1)
        _write_json(destination / "steps/000001.json", row)
        kwargs["activity_callback"]("environment_reset_calls", 1)
        kwargs["activity_callback"]("environment_step_calls", 1)
        kwargs["activity_callback"]("raw_sensor_sample_count", 10)
        raise RuntimeError("synthetic physical stop")

    with pytest.raises(
        contract.V12R17PhysicalDevelopmentError, match="stopped terminally"
    ):
        runner.execute(paths, case_runner=failed_case_runner)

    ledger = _mapping(paths.pipeline_ledger)
    assert ledger["passed"] is False
    assert ledger["attempted_case"] == contract.DISCRIMINATOR_CASE_ID
    assert ledger["completed_case_count"] == 0
    assert ledger["activity_totals"]["environment_step_calls"] == 1
    assert ledger["activity_totals"]["actor_query_count"] == 1
    assert calls == [contract.DISCRIMINATOR_CASE_ID]
    failed_root = paths.case_root(contract.DISCRIMINATOR_CASE_ID)
    assert _mapping(failed_root / "gate.json")["passed"] is False
    assert _mapping(failed_root / "receipt.json")["retry_authorized"] is False
    persisted_trace = contract.strict_json_load(failed_root / "trace.json")
    assert isinstance(persisted_trace, list)
    assert len(persisted_trace) == 1
    assert _mapping(failed_root / "summary.json")["status"] == (
        contract.EARLY_COMPLETE_STATUS
    )
    for later_case in contract.DEVELOPMENT_CASE_IDS[1:]:
        assert not paths.case_root(later_case).exists()

    # A repeated execute is verification-only: it cannot call the runner or
    # open cases 2-6 after the discriminator's terminal FAIL.
    repeated = runner.execute(paths, case_runner=failed_case_runner)
    assert repeated["passed"] is False
    assert calls == [contract.DISCRIMINATOR_CASE_ID]


def test_preflight_reports_missing_p2_without_writes(tmp_path: Path) -> None:
    paths = _paths(tmp_path)
    with pytest.raises(
        contract.V12R17PhysicalDevelopmentError, match="P2 prerequisite"
    ):
        contract.preflight(paths)
    assert not paths.p3_root.exists()


@pytest.mark.parametrize(
    "nested_field",
    (
        "duplicate_event_count",
        "out_of_order_event_count",
        "left_non_v26_source_count",
        "fallback_count",
        "hard_invalid_count",
    ),
)
def test_development_gate_rejects_each_nested_v26_counter_drift(
    nested_field: str,
) -> None:
    """V12R7 lesson: the gate must read and enforce the NESTED V26 counters."""

    case_id = contract.DISCRIMINATOR_CASE_ID
    trace = _passing_trace(case_id)
    summary = _passing_summary(case_id, trace)
    summary["binary_phase_event_gate"][nested_field] = 1
    gate = contract.development_gate(summary, case_id=case_id, trace=trace)
    assert gate["passed"] is False
    assert gate["checks"]["binary_v26"] is False


def test_aggregate_gate_risk_first_is_data_driven() -> None:
    """risk_first must attest the OBSERVED first execution, not module constants."""

    bindings = [
        {"case_id": case_id, "passed": True}
        for case_id in contract.DEVELOPMENT_CASE_IDS
    ]
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "protocol_id": contract.PROTOCOL_ID,
        "case_bindings": bindings,
        "candidate_tree_unique_count": 1,
        "p0_used": False,
        "development_count": contract.DEVELOPMENT_CASE_COUNT,
        "environment_reset_calls": contract.DEVELOPMENT_CASE_COUNT,
        "environment_step_calls": contract.DEVELOPMENT_CASE_COUNT
        * contract.EXPECTED_STEPS,
        "raw_sensor_sample_count": contract.DEVELOPMENT_CASE_COUNT
        * contract.EXPECTED_RAW_SENSOR_SAMPLES,
        "actor_query_count": contract.DEVELOPMENT_CASE_COUNT * contract.EXPECTED_STEPS,
        **{name: 0 for name in contract.PURE_POLICY_COUNTER_FIELDS},
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "q3_paths_opened": [],
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
    }
    gate = contract.aggregate_gate(summary)
    assert gate["passed"] is True
    assert gate["checks"]["risk_first"] is True

    rotated = dict(summary)
    rotated["case_bindings"] = bindings[1:] + bindings[:1]
    gate = contract.aggregate_gate(rotated)
    assert gate["checks"]["risk_first"] is False
    assert gate["passed"] is False

    empty = dict(summary)
    empty["case_bindings"] = []
    gate = contract.aggregate_gate(empty)
    assert gate["checks"]["risk_first"] is False
    assert gate["passed"] is False
