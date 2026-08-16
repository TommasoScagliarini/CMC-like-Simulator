from __future__ import annotations

import copy
import sys
from collections.abc import Mapping
from pathlib import Path, PurePosixPath
from typing import Any

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parent
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

import probe  # noqa: E402


class FakeEarlyTermination(RuntimeError):
    pass


def _row(
    step: int,
    *,
    final_step: int = probe.EXPECTED_STEPS,
    penetration_m: float = 0.01,
    valid_cycles: int = 2,
    clipped: bool = False,
    anomaly: str | None = None,
    end_reason: str | None = None,
) -> dict[str, Any]:
    final = step == final_step
    raw = [1.1, 0.0] if clipped else [0.2, -0.1]
    applied = [1.0, 0.0] if clipped else list(raw)
    checks = {name: True for name in probe.REQUIRED_STEP_CHECKS}
    checks["action_unclipped"] = not clipped
    if anomaly is not None:
        checks[anomaly] = False
    return {
        "schema_version": probe.SCHEMA_VERSION,
        "protocol_id": probe.PROTOCOL_ID,
        "stage_id": probe.STAGE_ID,
        "case_id": probe.CASE_ID,
        "step": step,
        "v26_observation": [0.0] * probe.EXPECTED_ACTOR_FEATURES,
        "candidate_mean": list(raw),
        "candidate_std": [probe.EXPECTED_H0_STD, probe.EXPECTED_H0_STD],
        "standard_normal": [0.0, 0.0],
        "single_noise": [0.0, 0.0],
        "raw_action": list(raw),
        "applied_action": applied,
        "teacher_enabled": False,
        "blending_enabled": False,
        "safety_latch_enabled": False,
        "teacher_query_count": 0,
        "served_action_teacher_dependency_count": 0,
        "mean_blend_count": 0,
        "safety_intervention_count": 0,
        "safety_latch_activation_count": 0,
        "safety_latch_release_count": 0,
        "raw_sensor_sample_count": probe.EXPECTED_RAW_SENSOR_SAMPLES_PER_STEP,
        "reward": 0.0,
        "time_s": float(step) * 0.01,
        "grf_penetration_m": penetration_m,
        "reserve_norm_nm": 1.0,
        "residual_norm_nm": 0.0,
        "phase_fsm": {
            "state_name": "STANCE_AFTER_HS",
            "expected_next_event": "toe_off",
            "event_source": probe.EVENT_SOURCE,
            "accepted_transitions_this_step": [],
            "invalid_event_count": 0.0,
            "valid_cycle_count": float(valid_cycles),
            "timeout_exceeded": 0.0,
        },
        "observer_raw_sensor_journal": {
            "samples": [{} for _ in range(probe.EXPECTED_RAW_SENSOR_SAMPLES_PER_STEP)]
        },
        "checks": checks,
        "terminated": bool(final and end_reason != "episode_time_limit"),
        "truncated": bool(final and end_reason == "episode_time_limit"),
        "end_reason": end_reason if final else None,
    }


def _make_protocol(tmp_path: Path) -> tuple[probe.ProbeProtocol, Path]:
    source = tmp_path / "source_h0"
    source.mkdir(parents=True)
    (source / "class_and_ctor_args.pkl").write_bytes(b"ctor")
    (source / "metadata.json").write_text("{}\n", encoding="utf-8")
    (source / "module_state.pkl").write_bytes(b"state")
    record = probe.tree_record(source, artifact_root=tmp_path)
    protocol = probe.ProbeProtocol(
        artifact_root=tmp_path,
        allowed_destination_root=tmp_path / "outputs",
        source_h0_id="TEST_SOURCE_H0",
        source_h0_relative_path=PurePosixPath("source_h0"),
        locked_source_h0_tree=record,
        enforce_physical_runtime_lock=False,
    )
    return protocol, source


def _write_one_step_prefix(
    kwargs: Mapping[str, Any],
    *,
    artifact_root: Path,
    source: Path,
) -> None:
    writer = probe.forensic.ForensicRolloutWriter(
        kwargs["destination"], artifact_root=artifact_root
    )
    writer.start(
        {
            "schema_version": probe.SCHEMA_VERSION,
            "status": probe.START_STATUS,
            "protocol_id": probe.PROTOCOL_ID,
            "stage_id": probe.STAGE_ID,
            "case": kwargs["case"],
            "module_checkpoint_path": source.as_posix(),
            "teacher_enabled": False,
            "blending_enabled": False,
            "safety_latch_enabled": False,
            **kwargs["start_metadata"],
        }
    )
    kwargs["activity_callback"]("environment_reset_calls", 1)
    kwargs["activity_callback"]("environment_step_calls", 1)
    kwargs["activity_callback"]("raw_sensor_sample_count", 10)
    writer.write_step(
        1,
        _row(
            1,
            final_step=1,
            penetration_m=0.026,
            valid_cycles=0,
            end_reason="grf_penetration",
        ),
    )


def _contract(protocol: probe.ProbeProtocol) -> dict[str, Any]:
    return {
        "source_h0_id": protocol.source_h0_id,
        "source_h0": copy.deepcopy(dict(protocol.locked_source_h0_tree)),
        "source_h0_role": "DIRECT_POLICY_NOT_TEACHER",
        "inference_route": "RUNTIME_V26_FIRST_35_FLOAT32_FEATURES_DIRECT",
        "event_contract_id": probe.EVENT_CONTRACT_ID,
        "target_contract_id": probe.TARGET_CONTRACT_ID,
        "morphology_weight_required": 0.0,
        "fit_executed": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "production_physical_runtime": {
            "path": "TEST_ADAPTER",
            "sha256": "0" * 64,
            "size_bytes": 0,
        },
        "direct_query_runtime_evidence": copy.deepcopy(
            probe.TEST_DIRECT_QUERY_RUNTIME_EVIDENCE
        ),
        "transitive_runtime_source_closure": copy.deepcopy(
            probe.TEST_TRANSITIVE_RUNTIME_SOURCE_CLOSURE
        ),
    }


def _gate(
    rows: list[dict[str, Any]],
    protocol: probe.ProbeProtocol,
    *,
    runtime_error: BaseException | None = None,
    case_overrides: Mapping[str, Any] | None = None,
    summary_overrides: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    contract = _contract(protocol)
    case = probe.canonical_case(protocol.allowed_destination_root / "case")
    if case_overrides is not None:
        case.update(case_overrides)
    run_start = {
        "schema_version": probe.SCHEMA_VERSION,
        "protocol_id": probe.PROTOCOL_ID,
        "stage_id": probe.STAGE_ID,
        "case": case,
        "teacher_enabled": False,
        "blending_enabled": False,
        "safety_latch_enabled": False,
        "probe_contract": contract,
    }
    summary = {
        "schema_version": probe.SCHEMA_VERSION,
        "protocol_id": probe.PROTOCOL_ID,
        "stage_id": probe.STAGE_ID,
        "steps": len(rows),
        "morphology_weight": 0.0,
        "event_contract_id": probe.EVENT_CONTRACT_ID,
        "target_contract_id": probe.TARGET_CONTRACT_ID,
        "binary_phase_fsm_mode": "binary_active",
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "probe_contract": contract,
    }
    if summary_overrides is not None:
        summary.update(summary_overrides)
    activity = {
        "environment_reset_calls": 1,
        "environment_step_calls": len(rows),
        "raw_sensor_sample_count": (
            len(rows) * probe.EXPECTED_RAW_SENSOR_SAMPLES_PER_STEP
        ),
    }
    return probe.evaluate_gate(
        summary=summary,
        run_start=run_start,
        rows=rows,
        activity=activity,
        protocol=protocol,
        source_after=protocol.locked_source_h0_tree,
        runtime_error=runtime_error,
    )


def test_locked_production_source_and_runtime_are_exact() -> None:
    assert probe.verify_source_h0() == probe.LOCKED_SOURCE_H0_TREE
    assert probe.verify_physical_runtime() == probe.LOCKED_PHYSICAL_RUNTIME
    assert (
        probe.verify_direct_query_runtime_evidence()
        == probe.LOCKED_DIRECT_QUERY_RUNTIME_EVIDENCE
    )
    closure = probe.verify_transitive_runtime_source_closure()
    assert closure["anchor_execution_lock"] == probe.LOCKED_R10_EXECUTION_LOCK
    assert closure["source_file_count"] == 94


def test_full_clean_500_step_gate_passes(tmp_path: Path) -> None:
    protocol, _source = _make_protocol(tmp_path)
    rows = [_row(step) for step in range(1, probe.EXPECTED_STEPS)]
    rows.append(
        _row(
            probe.EXPECTED_STEPS,
            end_reason="episode_time_limit",
        )
    )
    gate = _gate(rows, protocol)
    assert gate["passed"] is True
    assert gate["physical_passed"] is True
    assert gate["diagnostic_integrity_passed"] is True
    assert gate["checks"]["direct_source_h0_query_evidence_per_step"] is True
    assert gate["checks"]["detector_target_contracts_exact"] is True
    assert gate["trace_audit"]["deterministic_h0_distribution_exact"] is True
    assert gate["trace_audit"]["candidate_mean_journal_row_count"] == 500
    assert gate["trace_audit"]["direct_mean_action_binding_row_count"] == 500
    assert gate["checks"]["zero_timeouts"] is True


@pytest.mark.parametrize(
    ("mutation", "failed_check"),
    (
        ("early", "full_horizon"),
        ("penetration", "penetration"),
        ("cycles", "cycles"),
        ("clipping", "zero_clipping"),
        ("anomaly", "zero_anomalies"),
        ("timeout", "zero_timeouts"),
    ),
)
def test_physical_gate_blockers_are_independent(
    tmp_path: Path,
    mutation: str,
    failed_check: str,
) -> None:
    protocol, _source = _make_protocol(tmp_path)
    count = 3 if mutation == "early" else probe.EXPECTED_STEPS
    rows = [_row(step, final_step=count) for step in range(1, count)]
    terminal = _row(count, final_step=count, end_reason="episode_time_limit")
    if mutation == "penetration":
        terminal["grf_penetration_m"] = probe.PENETRATION_LIMIT_M
    elif mutation == "cycles":
        for row in rows:
            row["phase_fsm"]["valid_cycle_count"] = 1.0
        terminal["phase_fsm"]["valid_cycle_count"] = 1.0
    elif mutation == "clipping":
        terminal = _row(
            count,
            final_step=count,
            clipped=True,
            end_reason="episode_time_limit",
        )
    elif mutation == "anomaly":
        terminal = _row(
            count,
            final_step=count,
            anomaly="no_unaccepted_so",
            end_reason="episode_time_limit",
        )
    elif mutation == "timeout":
        terminal["checks"]["no_timeout"] = False
        terminal["phase_fsm"]["timeout_exceeded"] = 1.0
    rows.append(terminal)
    gate = _gate(rows, protocol)
    assert gate["passed"] is False
    assert gate["checks"][failed_check] is False


@pytest.mark.parametrize(
    ("mutation", "failed_check"),
    (
        ("distribution", "direct_v26_35"),
        ("sensor_journal", "direct_v26_35"),
        ("offset", "hardest_case_exact"),
        ("event_contract", "detector_target_contracts_exact"),
        ("updates", "zero_fit_and_updates"),
    ),
)
def test_integrity_gate_rejects_semantic_drift(
    tmp_path: Path,
    mutation: str,
    failed_check: str,
) -> None:
    protocol, _source = _make_protocol(tmp_path)
    rows = [_row(step) for step in range(1, probe.EXPECTED_STEPS)]
    rows.append(_row(probe.EXPECTED_STEPS, end_reason="episode_time_limit"))
    case_overrides = None
    summary_overrides = None
    if mutation == "distribution":
        rows[0]["candidate_std"] = [0.01, 0.01]
    elif mutation == "sensor_journal":
        rows[0]["observer_raw_sensor_journal"]["samples"].pop()
    elif mutation == "offset":
        case_overrides = {"episode_start_offset_s": 2.156}
    elif mutation == "event_contract":
        summary_overrides = {"event_contract_id": "drifted"}
    elif mutation == "updates":
        summary_overrides = {"actor_updates": 1}
    gate = _gate(
        rows,
        protocol,
        case_overrides=case_overrides,
        summary_overrides=summary_overrides,
    )
    assert gate["passed"] is False
    assert gate["checks"][failed_check] is False


def test_early_physical_termination_still_closes_summary_and_gate(
    tmp_path: Path,
) -> None:
    protocol, source = _make_protocol(tmp_path)
    destination = protocol.allowed_destination_root / "early"
    captured: dict[str, Any] = {}

    def fake_runner(**kwargs: Any) -> Mapping[str, Any]:
        captured.update(kwargs)
        writer = probe.forensic.ForensicRolloutWriter(
            kwargs["destination"], artifact_root=tmp_path
        )
        run_start = {
            "schema_version": probe.SCHEMA_VERSION,
            "status": probe.START_STATUS,
            "protocol_id": probe.PROTOCOL_ID,
            "stage_id": probe.STAGE_ID,
            "case": kwargs["case"],
            "module_checkpoint_path": source.as_posix(),
            "teacher_enabled": False,
            "blending_enabled": False,
            "safety_latch_enabled": False,
            **kwargs["start_metadata"],
        }
        writer.start(run_start)
        kwargs["activity_callback"]("environment_reset_calls", 1)
        for step in (1, 2):
            kwargs["activity_callback"]("environment_step_calls", 1)
            kwargs["activity_callback"]("raw_sensor_sample_count", 10)
            writer.write_step(
                step,
                _row(
                    step,
                    final_step=2,
                    penetration_m=(0.01 if step == 1 else 0.026),
                    valid_cycles=0,
                    end_reason=("grf_penetration" if step == 2 else None),
                ),
            )
        raise FakeEarlyTermination("pure-policy trace audit failed")

    result = probe.run_probe(
        destination=destination,
        protocol=protocol,
        physical_runner=fake_runner,
    )
    assert captured["module_path"] == source
    assert result["summary"]["status"] == probe.EARLY_COMPLETE_STATUS
    assert result["summary"]["steps"] == 2
    assert result["gate"]["passed"] is False
    assert result["gate"]["diagnostic_integrity_passed"] is True
    assert result["gate"]["physical_passed"] is False
    assert result["gate"]["checks"]["full_horizon"] is False
    assert result["gate"]["checks"]["penetration"] is False
    assert result["closure_verification"]["passed"] is True
    assert (destination / "trace.json").is_file()
    assert (destination / "partial_summary.json").is_file()
    assert (destination / "summary.json").is_file()
    assert (destination / "gate.json").is_file()
    assert (destination / "closure_receipt.json").is_file()
    assert result["closure_verification"]["closure_receipt"] == (
        probe.forensic.artifact_record(
            destination / "closure_receipt.json",
            artifact_root=tmp_path,
        )
    )
    trace = probe.forensic.strict_json_load(destination / "trace.json")
    assert trace == [
        probe.forensic.strict_json_load(destination / "steps/000001.json"),
        probe.forensic.strict_json_load(destination / "steps/000002.json"),
    ]


def test_source_drift_and_occupied_destination_fail_before_runner(
    tmp_path: Path,
) -> None:
    protocol, source = _make_protocol(tmp_path)
    calls = 0

    def forbidden_runner(**_kwargs: Any) -> Mapping[str, Any]:
        nonlocal calls
        calls += 1
        raise AssertionError("physical runner must not be reached")

    source.joinpath("module_state.pkl").write_bytes(b"drift")
    with pytest.raises(probe.ProbeError, match="source H0 tree drifted"):
        probe.run_probe(
            destination=protocol.allowed_destination_root / "drift",
            protocol=protocol,
            physical_runner=forbidden_runner,
        )
    assert calls == 0

    protocol, _source = _make_protocol(tmp_path / "second")
    occupied = protocol.allowed_destination_root / "occupied"
    occupied.mkdir(parents=True)
    with pytest.raises(probe.ProbeError, match="already exists/no retry"):
        probe.run_probe(
            destination=occupied,
            protocol=protocol,
            physical_runner=forbidden_runner,
        )
    assert calls == 0


def test_post_rollout_closure_error_publishes_failure_receipt(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    protocol, source = _make_protocol(tmp_path)
    destination = protocol.allowed_destination_root / "closure_failure"

    def fake_runner(**kwargs: Any) -> Mapping[str, Any]:
        writer = probe.forensic.ForensicRolloutWriter(
            kwargs["destination"], artifact_root=tmp_path
        )
        writer.start(
            {
                "schema_version": probe.SCHEMA_VERSION,
                "status": probe.START_STATUS,
                "protocol_id": probe.PROTOCOL_ID,
                "stage_id": probe.STAGE_ID,
                "case": kwargs["case"],
                "module_checkpoint_path": source.as_posix(),
                "teacher_enabled": False,
                "blending_enabled": False,
                "safety_latch_enabled": False,
                **kwargs["start_metadata"],
            }
        )
        kwargs["activity_callback"]("environment_reset_calls", 1)
        kwargs["activity_callback"]("environment_step_calls", 1)
        kwargs["activity_callback"]("raw_sensor_sample_count", 10)
        writer.write_step(
            1,
            _row(
                1,
                final_step=1,
                penetration_m=0.026,
                valid_cycles=0,
                end_reason="grf_penetration",
            ),
        )
        raise FakeEarlyTermination("pure-policy trace audit failed")

    def fail_gate(**_kwargs: Any) -> dict[str, Any]:
        raise RuntimeError("injected gate closure failure")

    monkeypatch.setattr(probe, "evaluate_gate", fail_gate)
    with pytest.raises(probe.ProbeError, match="failure receipt sha256"):
        probe.run_probe(
            destination=destination,
            protocol=protocol,
            physical_runner=fake_runner,
        )
    failure = probe.forensic.strict_json_load(destination / "failure.json")
    assert failure["status"] == probe.CLOSURE_FAIL_STATUS
    assert failure["last_completed_step"] == 1
    assert failure["end_reason"] == "post_rollout_closure_failure"
    assert failure["error"] == {
        "type": "RuntimeError",
        "message": "injected gate closure failure",
    }
    assert (destination / "summary.json").is_file()
    assert not (destination / "gate.json").exists()


def test_post_rollout_source_drift_publishes_failure_receipt(tmp_path: Path) -> None:
    protocol, source = _make_protocol(tmp_path)
    destination = protocol.allowed_destination_root / "post_rollout_source_drift"

    def drifting_runner(**kwargs: Any) -> Mapping[str, Any]:
        _write_one_step_prefix(kwargs, artifact_root=tmp_path, source=source)
        source.joinpath("module_state.pkl").write_bytes(b"post-rollout drift")
        return {}

    with pytest.raises(probe.ProbeError, match="failure receipt sha256"):
        probe.run_probe(
            destination=destination,
            protocol=protocol,
            physical_runner=drifting_runner,
        )
    failure = probe.forensic.strict_json_load(destination / "failure.json")
    assert failure["last_completed_step"] == 1
    assert failure["error"] == {
        "type": "ProbeError",
        "message": "locked source H0 tree drifted",
    }
    assert not (destination / "summary.json").exists()
    assert not (destination / "gate.json").exists()


def test_post_gate_tamper_is_caught_by_end_to_end_verifier(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    protocol, source = _make_protocol(tmp_path)
    destination = protocol.allowed_destination_root / "post_gate_tamper"

    def fake_runner(**kwargs: Any) -> Mapping[str, Any]:
        _write_one_step_prefix(kwargs, artifact_root=tmp_path, source=source)
        return {}

    original_run_gate = probe.forensic.ForensicRolloutWriter.run_gate

    def tampering_run_gate(self: Any, callback: Any) -> dict[str, Any]:
        record = original_run_gate(self, callback)
        self.summary_path.write_text('{"tampered":true}\n', encoding="utf-8")
        return record

    monkeypatch.setattr(
        probe.forensic.ForensicRolloutWriter,
        "run_gate",
        tampering_run_gate,
    )
    with pytest.raises(probe.ProbeError, match="failure receipt sha256"):
        probe.run_probe(
            destination=destination,
            protocol=protocol,
            physical_runner=fake_runner,
        )
    failure = probe.forensic.strict_json_load(destination / "failure.json")
    assert failure["last_completed_step"] == 1
    assert failure["error"] == {
        "type": "ProbeError",
        "message": "closed rollout summary binding failed",
    }
    assert (destination / "gate.json").is_file()


def test_description_is_explicitly_source_only_and_unexecuted() -> None:
    description = probe.describe_protocol()
    assert description["status"] == "READY_SOURCE_ONLY_NOT_EXECUTED"
    assert description["rollout_executed"] is False
    assert description["source_h0_id"] == probe.SOURCE_H0_ID
    assert description["source_h0"] == probe.LOCKED_SOURCE_H0_TREE
    assert description["gate"] == {
        "steps": 500,
        "penetration_strictly_below_m": 0.025,
        "minimum_valid_cycles": 2,
        "zero_clipping": True,
        "zero_anomalies": True,
        "zero_timeouts": True,
    }
