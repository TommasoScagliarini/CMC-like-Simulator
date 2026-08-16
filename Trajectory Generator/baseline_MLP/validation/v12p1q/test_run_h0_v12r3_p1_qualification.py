"""Pure and mocked-execution tests for the V12P1Q one-shot supervisor."""

from __future__ import annotations

import copy
import hashlib
import json
import os
import sys
from pathlib import Path, PurePosixPath
from types import SimpleNamespace
from typing import Any

import pytest


def _discover_repo_root(source: Path) -> Path:
    for candidate in source.resolve().parents:
        if (
            (candidate / "AGENTS.md").is_file()
            and (candidate / "validation").is_dir()
            and (candidate / "Trajectory Generator").is_dir()
        ):
            return candidate
    raise RuntimeError("repository root could not be discovered")


REPO_ROOT = _discover_repo_root(Path(__file__))
ROOT_VALIDATION = REPO_ROOT / "validation"
LOCAL_VALIDATION = REPO_ROOT / "Trajectory Generator" / "baseline_MLP" / "validation"
V12P1Q_ROOT = Path(__file__).resolve().parent
for _root in (REPO_ROOT, ROOT_VALIDATION, LOCAL_VALIDATION, V12P1Q_ROOT):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_forensic_rollout as forensic  # noqa: E402
import h0_v12r3_p1_qualification_contract as contract  # noqa: E402
import h0_v12r3_p1_qualification_gates as gates  # noqa: E402
import prepare_h0_v12r3_p1_qualification_noise_tapes as noise  # noqa: E402
import run_h0_v12r3_p1_qualification as runner  # noqa: E402


_SHA = "a" * 64


def _artifact(path: str | PurePosixPath) -> dict[str, Any]:
    return {
        "path": PurePosixPath(path).as_posix(),
        "sha256": _SHA,
        "size_bytes": 1,
    }


def _source_h0_tree() -> dict[str, Any]:
    return {
        **contract.SOURCE_H0_MODULE,
        "file_count": 1,
        "files": [{"path": "module_state.pkl", "sha256": _SHA, "size_bytes": 1}],
    }


def _metric(samples: int, rms: float, absolute: float) -> dict[str, Any]:
    return {"sample_count": samples, "rms": rms, "abs_max": absolute}


def _physical(role: str) -> dict[str, Any]:
    sea = {}
    for joint in contract.JOINTS:
        row = {
            signal: _metric(contract.SEA_EXPECTED_SAMPLE_COUNTS[signal], 1.0, 2.0)
            for signal in contract.SEA_SIGNALS
        }
        row["tau_input_saturated"] = {
            "sample_count": contract.EXPECTED_RAW_SENSOR_SAMPLES,
            "count": 0,
            "fraction": 0.0,
        }
        sea[joint] = row
    result = {
        "steps": 500,
        "end_reason": "episode_time_limit",
        "terminated": False,
        "truncated": True,
        "phase_valid_cycle_count": 2,
        "grf_penetration_max_m": 0.024,
        "control_window_count": 5_000,
        "raw_sensor_sample_count": 5_000,
        "binary_phase_sensor_sample_count": 5_000,
        "action_clipped_values": 0,
        "fallback_count": 0,
        "timeout_count": 0,
        "safety_stop_count": 0,
        "sea_plugin_fallback_count": 0,
        "so_solver_unaccepted_count": 0,
        "hard_invalid_count": 0,
        "invalid_event_count": 0,
        "nonfinite_count": 0,
        "routing_failure_count": 0,
        "step_contract_failure_count": 0,
        "n_actor": 35,
        "n_observation": 84,
        "observation_dtype": "float32",
        "action_shape": [2],
        "action_dtype": "float32",
        "episode_metrics": {
            "reserve_norm_nm": _metric(500, 10.0, 20.0),
            "residual_norm_nm": _metric(500, 1.0e-7, 2.0e-7),
        },
        "sea_episode_metrics": sea,
        "legacy_event_integrity_passed": True,
    }
    if role == contract.CANDIDATE_ROLE:
        result["binary_phase_event_gate"] = {
            "passed": True,
            "sample_count": 5_000,
            "duplicate_event_count": 0,
            "out_of_order_event_count": 0,
            "left_non_v26_source_count": 0,
            "fallback_count": 0,
            "hard_invalid_count": 0,
        }
    return result


def _summary(role: str, case_id: str) -> dict[str, Any]:
    root = contract.rollout_root(role, case_id)
    case = contract.canonical_case(case_id)
    return runner._build_rollout_summary(
        role=role,
        case_id=case_id,
        physical=_physical(role),
        artifacts={
            "protocol_freeze": _artifact(contract.PROTOCOL_FREEZE_PATH),
            "execution_lock": _artifact(contract.EXECUTION_LOCK_PATH),
            "pipeline_claim": _artifact(contract.PIPELINE_CLAIM_PATH),
            "worker_claim": _artifact(
                contract.worker_claim_path(f"rollout__{role}__{case_id}")
            ),
            "run_start": _artifact(root / "run_start.json"),
            "trace": _artifact(root / "trace.json"),
            "partial_summary": _artifact(root / "partial_summary.json"),
        },
        noise_tape=_artifact(case["noise_tape"]),
        noise_tape_array_sha256=_SHA,
        actor_module=(
            copy.deepcopy(contract.P1_CANDIDATE_MODULE)
            if role == contract.CANDIDATE_ROLE
            else _source_h0_tree()
        ),
    )


def test_frozen_five_sources_and_design_freeze_are_unchanged() -> None:
    expected = {
        "freeze_h0_v12r3_p1_qualification_design.py": (
            "653e4a8193c87fb6cb367ca2d3f9c8d3476ee22a506cd47633e60bf949b78b7a"
        ),
        "h0_v12r3_p1_qualification_contract.py": (
            "fd0f47e3324e9befd71d1c943a71dbb5dc426cc94f1bbacc4353b54072ed3db9"
        ),
        "h0_v12r3_p1_qualification_gates.py": (
            "1d12d002cb7b84dee5b095e0f6ae516aedefa9688dfe869b2f08271d35b65487"
        ),
        "test_freeze_h0_v12r3_p1_qualification_design.py": (
            "5a73a99a280edac906a9731429a993221e4f40fe0dfb8820f974e9c51e4ce4be"
        ),
        "test_h0_v12r3_p1_qualification_contract_and_gates.py": (
            "f1aa19319c1c1d78eccaaee211310895f932440cfb0ec99798bb03f302ec911e"
        ),
        "h0_v12r3_p1_qualification_design_freeze.json": (
            "c64928cb85df0e1f5d53c5f2e6eba52172b7c831ccf712a0113522bcff4ef686"
        ),
    }
    for name, sha256 in expected.items():
        assert hashlib.sha256((V12P1Q_ROOT / name).read_bytes()).hexdigest() == sha256
    assert (
        V12P1Q_ROOT / "h0_v12r3_p1_qualification_design_freeze.json"
    ).stat().st_size == 21_303


def test_stage_order_is_baseline_first_six_candidate_six_then_final() -> None:
    assert list(contract.STAGE_IDS[:6]) == [
        f"rollout__baseline__{case_id}" for case_id in contract.CASE_IDS
    ]
    assert list(contract.STAGE_IDS[6:12]) == [
        f"rollout__candidate__{case_id}" for case_id in contract.CASE_IDS
    ]
    assert contract.STAGE_IDS[12] == "finalize_qualification"
    assert len(contract.STAGE_IDS) == 13
    assert runner.EXECUTION_AUTHORITY["baseline_first"] is True
    assert runner.EXECUTION_AUTHORITY["retry_authorized"] is False
    assert runner.EXECUTION_AUTHORITY["resume_authorized"] is False


@pytest.mark.parametrize("role", contract.ROLE_ORDER)
@pytest.mark.parametrize("case_id", contract.CASE_IDS)
def test_normalized_runtime_summary_passes_frozen_common_gate(
    role: str, case_id: str
) -> None:
    summary = _summary(role, case_id)
    result = gates.common_rollout_gate(summary, role=role, case_id=case_id)
    assert result["passed"] is True
    assert summary["teacher_enabled"] is False
    assert summary["blending_enabled"] is False
    assert summary["safety_latch_enabled"] is False
    assert summary["morphology_weight"] == 0.0
    assert all(summary[field] == 0 for field in contract.ZERO_REQUIRED_COUNTS)


@pytest.mark.parametrize("case_id", contract.CASE_IDS)
def test_same_tape_pair_passes_every_noninferiority_row(case_id: str) -> None:
    baseline = _summary(contract.BASELINE_ROLE, case_id)
    candidate = _summary(contract.CANDIDATE_ROLE, case_id)
    result = gates.condition_matched_gate(baseline, candidate, case_id=case_id)
    assert result["passed"] is True
    assert result["checks"]["condition_exact"] is True
    assert result["checks"]["all_noninferiority_rows"] is True
    assert result["checks"]["saturation_nonincreasing_and_zero"] is True
    assert result["compensation_or_averaging_used"] is False


def test_physical_or_sea_regression_fails_after_normalization() -> None:
    case_id = contract.CASE_IDS[0]
    summary = _summary(contract.CANDIDATE_ROLE, case_id)
    summary["grf_penetration_max_m"] = 0.025
    assert not gates.common_rollout_gate(
        summary, role=contract.CANDIDATE_ROLE, case_id=case_id
    )["passed"]

    summary = _summary(contract.CANDIDATE_ROLE, case_id)
    summary["sea_episode_metrics"][contract.JOINTS[0]]["tau_input_saturated"].update(
        count=1, fraction=0.0002
    )
    assert not gates.common_rollout_gate(
        summary, role=contract.CANDIDATE_ROLE, case_id=case_id
    )["passed"]


def test_summary_builder_refuses_missing_physical_counter() -> None:
    case_id = contract.CASE_IDS[0]
    physical = _physical(contract.CANDIDATE_ROLE)
    physical.pop("fallback_count")
    root = contract.rollout_root(contract.CANDIDATE_ROLE, case_id)
    with pytest.raises(
        runner.V12R3P1QualificationExecutionError, match="lacks required counter"
    ):
        runner._build_rollout_summary(
            role=contract.CANDIDATE_ROLE,
            case_id=case_id,
            physical=physical,
            artifacts={
                "protocol_freeze": _artifact(contract.PROTOCOL_FREEZE_PATH),
                "execution_lock": _artifact(contract.EXECUTION_LOCK_PATH),
                "pipeline_claim": _artifact(contract.PIPELINE_CLAIM_PATH),
                "worker_claim": _artifact(
                    contract.worker_claim_path(f"rollout__candidate__{case_id}")
                ),
                "run_start": _artifact(root / "run_start.json"),
                "trace": _artifact(root / "trace.json"),
                "partial_summary": _artifact(root / "partial_summary.json"),
            },
            noise_tape=_artifact(contract.canonical_case(case_id)["noise_tape"]),
            noise_tape_array_sha256=_SHA,
            actor_module=contract.P1_CANDIDATE_MODULE,
        )


@pytest.mark.parametrize("field", ["teacher_query_count", "actor_updates"])
def test_summary_builder_does_not_mask_nonzero_procedural_counter(
    field: str,
) -> None:
    case_id = contract.CASE_IDS[0]
    physical = _physical(contract.CANDIDATE_ROLE)
    physical[field] = 1
    root = contract.rollout_root(contract.CANDIDATE_ROLE, case_id)
    with pytest.raises(
        runner.V12R3P1QualificationExecutionError,
        match="non-zero procedural autonomy counter",
    ):
        runner._build_rollout_summary(
            role=contract.CANDIDATE_ROLE,
            case_id=case_id,
            physical=physical,
            artifacts={
                "protocol_freeze": _artifact(contract.PROTOCOL_FREEZE_PATH),
                "execution_lock": _artifact(contract.EXECUTION_LOCK_PATH),
                "pipeline_claim": _artifact(contract.PIPELINE_CLAIM_PATH),
                "worker_claim": _artifact(
                    contract.worker_claim_path(f"rollout__candidate__{case_id}")
                ),
                "run_start": _artifact(root / "run_start.json"),
                "trace": _artifact(root / "trace.json"),
                "partial_summary": _artifact(root / "partial_summary.json"),
            },
            noise_tape=_artifact(contract.canonical_case(case_id)["noise_tape"]),
            noise_tape_array_sha256=_SHA,
            actor_module=contract.P1_CANDIDATE_MODULE,
        )


def test_live_step_is_durable_before_in_memory_append() -> None:
    rows: list[dict[str, Any]] = []
    observed: list[tuple[int, dict[str, Any]]] = []

    def persist(step: int, row: Any) -> None:
        assert rows == []
        observed.append((step, dict(row)))

    runner._append_live_step(rows, {"step": 1, "value": 2}, persist_step=persist)
    assert observed == [(1, {"step": 1, "value": 2})]
    assert rows == [{"step": 1, "value": 2}]

    def fail_before_append(_step: int, _row: Any) -> None:
        raise RuntimeError("disk failure")

    with pytest.raises(RuntimeError, match="disk failure"):
        runner._append_live_step(
            rows,
            {"step": 2, "value": 3},
            persist_step=fail_before_append,
        )
    assert rows == [{"step": 1, "value": 2}]


def test_step_journal_verifier_requires_exact_files_and_canonical_bytes(
    tmp_path: Path,
) -> None:
    writer = forensic.ForensicRolloutWriter(
        tmp_path / "rollout", artifact_root=tmp_path
    )
    writer.start({"status": "started"})
    writer.write_step(1, {"value": 1})
    writer.write_step(2, {"value": 2})
    assert runner._verify_step_journal_canonical(writer, expected_steps=2) == [
        {"step": 1, "value": 1},
        {"step": 2, "value": 2},
    ]
    step_two = writer.steps_directory / "000002.json"
    step_two.write_text(
        json.dumps({"step": 2, "value": 2}, sort_keys=True), encoding="utf-8"
    )
    with pytest.raises(
        runner.V12R3P1QualificationExecutionError, match="byte-canonical"
    ):
        runner._verify_step_journal_canonical(writer, expected_steps=2)


def test_baseline_so_classification_accepts_verified_bounded_ls() -> None:
    keys = (
        "control_window_count",
        "bounded_ls_invocation_count",
        "verified_bounded_ls_count",
        "unaccepted_hard_so_fallback_count",
        "unaccepted_bounded_ls_count",
    )
    counters = {
        "control_window_count": 10,
        "bounded_ls_invocation_count": 1,
        "verified_bounded_ls_count": 1,
        "unaccepted_hard_so_fallback_count": 0,
        "unaccepted_bounded_ls_count": 0,
    }
    validated: list[tuple[Any, int, bool]] = []
    v3 = SimpleNamespace(
        SO_RECOVERY_COUNTER_KEYS=keys,
        _validate_so_solver_audit_entries=lambda entries, **kwargs: validated.append(
            (entries, kwargs["step_index"], kwargs["selected_fallback"])
        ),
    )
    recovery = SimpleNamespace(
        classify_policy_step=lambda _entries, **_kwargs: {"counters": counters}
    )
    totals = {key: 0 for key in keys}
    unaccepted = runner._accumulate_baseline_so_recovery(
        totals,
        entries=[{"solver": "bounded_ls"}],
        step=4,
        selected_fallback=True,
        policy_id="frozen-policy",
        v3=v3,
        so_recovery=recovery,
    )
    assert validated == [([{"solver": "bounded_ls"}], 4, True)]
    assert totals == counters
    assert unaccepted == 0


def test_build_execution_lock_binds_protocol_runtime_and_no_update_authority(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    sources = {
        name: _artifact(path)
        for name, path in runner.protocol_freezer.RUNTIME_SOURCE_RELATIVE_PATHS.items()
    }
    inputs = {
        "candidate_module": copy.deepcopy(contract.P1_CANDIDATE_MODULE),
        "source_h0_module": _source_h0_tree(),
        "source_h0_config": _artifact("source.yaml"),
        "historical_analog_profile": _artifact("analog.json"),
        "baseline_shadow_v25_profile": _artifact("shadow.json"),
    }
    fake_freeze = {
        "status": contract.PROTOCOL_FREEZE_PASS_STATUS,
        "passed": True,
        "qualification_design_freeze": copy.deepcopy(noise.DESIGN_FREEZE_RECORD),
        "selected_candidate_id": contract.P1_CANDIDATE_ID,
        "selected_candidate": copy.deepcopy(contract.P1_CANDIDATE_MODULE),
        "noise_manifest": _artifact(PurePosixPath(contract.NOISE_MANIFEST_PATH).name),
        "runtime_sources": sources,
        "runtime_inputs": inputs,
        "rollout_matrix": list(contract.ROLLOUT_MATRIX),
    }
    monkeypatch.setattr(runner, "_verified_protocol_freeze", lambda: fake_freeze)
    monkeypatch.setattr(runner, "_current_runtime_sources", lambda _freeze: sources)
    monkeypatch.setattr(runner, "_current_runtime_inputs", lambda _freeze: inputs)
    monkeypatch.setattr(
        runner.noise,
        "verify_manifest",
        lambda: {"passed": True, "status": noise.NOISE_TAPES_PASS_STATUS},
    )
    monkeypatch.setattr(
        runner,
        "_runtime_record",
        lambda: {
            "inference_stack_ready": True,
            "platform_plugin_readiness": {"passed": True},
        },
    )
    monkeypatch.setattr(
        runner,
        "_record",
        lambda path: _artifact(Path(path).name),
    )
    payload = runner.build_execution_lock(require_unoccupied=False)
    assert payload["status"] == contract.EXECUTION_LOCK_PASS_STATUS
    assert payload["passed"] is True
    assert payload["candidate_module"] == contract.P1_CANDIDATE_MODULE
    assert payload["rollout_matrix"] == list(contract.ROLLOUT_MATRIX)
    assert payload["stage_order"] == list(contract.STAGE_IDS)
    assert payload["authority"] == runner.EXECUTION_AUTHORITY
    assert payload["retry_authorized"] is False
    assert payload["resume_authorized"] is False
    assert payload["runtime_promoted"] is False


def test_worker_claim_chains_every_prior_receipt_in_frozen_order(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(runner, "_record", lambda path: _artifact(Path(path).name))
    stage = contract.STAGE_IDS[6]
    payload = runner._worker_claim_payload(stage, "a" * 64)
    assert payload["stage_id"] == stage
    assert payload["stage_index"] == 6
    assert [row["stage_id"] for row in payload["previous_receipts"]] == list(
        contract.STAGE_IDS[:6]
    )
    assert payload["retry_authorized"] is False
    assert payload["resume_authorized"] is False


def test_execute_calls_all_thirteen_stages_once_and_terminalizes(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    events: list[tuple[str, str | None]] = []
    captured: dict[str, Any] = {}
    ledger_path = tmp_path / "ledger.json"
    monkeypatch.setattr(runner, "PIPELINE_LEDGER_PATH", ledger_path)
    monkeypatch.setattr(
        runner,
        "verify_execution_lock",
        lambda **_kwargs: {"passed": True},
    )
    monkeypatch.setattr(runner, "_claim_run_root", lambda: "x" * 48)
    monkeypatch.setattr(
        runner,
        "_write_worker_claim",
        lambda stage, _token: events.append(("claim", stage)) or tmp_path / stage,
    )

    def run_stage(stage: str) -> dict[str, Any]:
        events.append(("run", stage))
        if stage.startswith("rollout__"):
            runner._ACTIVITY["environment_reset_calls"] += 1
            runner._ACTIVITY["environment_step_calls"] += 500
            key = (
                "baseline_actor_queries"
                if "__baseline__" in stage
                else "candidate_actor_queries"
            )
            runner._ACTIVITY[key] += 500
        return {"passed": True}

    monkeypatch.setattr(runner, "_run_stage", run_stage)
    monkeypatch.setattr(
        runner,
        "_verify_rollout_receipt",
        lambda stage: {"stage_id": stage, "passed": True},
    )
    monkeypatch.setattr(
        runner,
        "_verify_aggregate_receipt",
        lambda stage="finalize_qualification": {"stage_id": stage, "passed": True},
    )

    def ledger_payload(**kwargs: Any) -> dict[str, Any]:
        captured.update(kwargs)
        return {
            "status": runner.LEDGER_PASS_STATUS,
            "passed": kwargs["passed"],
        }

    monkeypatch.setattr(runner, "_ledger_payload", ledger_payload)
    result = runner.execute_qualification_once()
    assert result["passed"] is True
    assert [stage for kind, stage in events if kind == "run"] == list(
        contract.STAGE_IDS
    )
    assert [stage for kind, stage in events if kind == "claim"] == list(
        contract.STAGE_IDS
    )
    assert captured["passed"] is True
    assert captured["attempted_stage"] is None
    assert captured["completed_stages"] == list(contract.STAGE_IDS)


def test_execute_failure_writes_one_terminal_ledger_without_retry(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    ledger_path = tmp_path / "ledger.json"
    monkeypatch.setattr(runner, "PIPELINE_LEDGER_PATH", ledger_path)
    monkeypatch.setattr(
        runner,
        "verify_execution_lock",
        lambda **_kwargs: {"passed": True},
    )
    monkeypatch.setattr(runner, "_claim_run_root", lambda: "x" * 48)
    monkeypatch.setattr(
        runner, "_write_worker_claim", lambda *_args: tmp_path / "claim"
    )
    failed_stage = contract.STAGE_IDS[2]

    def run_stage(stage: str) -> dict[str, Any]:
        if stage == failed_stage:
            raise RuntimeError("terminal stage failure")
        return {"passed": True}

    monkeypatch.setattr(runner, "_run_stage", run_stage)
    monkeypatch.setattr(
        runner,
        "_verify_rollout_receipt",
        lambda stage: {"stage_id": stage, "passed": True},
    )
    captured: dict[str, Any] = {}

    def ledger_payload(**kwargs: Any) -> dict[str, Any]:
        captured.update(kwargs)
        return {
            "status": runner.LEDGER_FAIL_STATUS,
            "passed": False,
            "terminal": True,
            "retry_authorized": False,
            "resume_authorized": False,
        }

    monkeypatch.setattr(runner, "_ledger_payload", ledger_payload)
    with pytest.raises(RuntimeError, match="terminal stage failure"):
        runner.execute_qualification_once()
    ledger = forensic.strict_json_load(ledger_path)
    assert ledger["terminal"] is True
    assert ledger["retry_authorized"] is False
    assert ledger["resume_authorized"] is False
    assert captured["attempted_stage"] == failed_stage
    assert captured["completed_stages"] == list(contract.STAGE_IDS[:2])


def test_aggregate_verifier_requires_exact_canonical_closure(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    final_root = tmp_path / "finalize"
    final_root.mkdir()
    summary = {"kind": "aggregate-summary"}
    gate = {"status": contract.AGGREGATE_PASS_STATUS, "passed": True}
    receipt = {"kind": "aggregate-receipt", "passed": True}
    forensic.write_json_exclusive(final_root / "summary.json", summary)
    forensic.write_json_exclusive(final_root / "gate.json", gate)
    forensic.write_json_exclusive(final_root / "receipt.json", receipt)
    monkeypatch.setattr(runner, "_stage_root", lambda _stage: final_root)
    monkeypatch.setattr(
        runner, "_stage_receipt_path", lambda _stage: final_root / "receipt.json"
    )
    monkeypatch.setattr(runner, "_verify_worker_claim", lambda _stage: {})
    monkeypatch.setattr(runner, "_verify_rollout_receipt", lambda _stage: {})
    monkeypatch.setattr(runner, "_aggregate_summary_payload", lambda _stage: summary)
    monkeypatch.setattr(
        runner.gates, "aggregate_qualification_gate", lambda _summary: gate
    )
    monkeypatch.setattr(
        runner, "_aggregate_receipt_payload", lambda _stage, _gate: receipt
    )
    assert runner._verify_aggregate_receipt() == receipt
    (final_root / "receipt.json").write_text(
        json.dumps(receipt, sort_keys=True), encoding="utf-8"
    )
    with pytest.raises(
        runner.V12R3P1QualificationExecutionError, match="byte-canonical"
    ):
        runner._verify_aggregate_receipt()


def test_aggregate_failure_is_terminalized_and_bound(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    final_root = tmp_path / "finalize"
    summary = {"kind": "aggregate-summary"}
    gate = {"status": contract.AGGREGATE_FAIL_STATUS, "passed": False}
    monkeypatch.setattr(runner, "_stage_root", lambda _stage: final_root)
    monkeypatch.setattr(runner, "_verify_worker_claim", lambda _stage: {})
    monkeypatch.setattr(runner, "_verify_rollout_receipt", lambda _stage: {})
    monkeypatch.setattr(runner, "_aggregate_summary_payload", lambda _stage: summary)
    monkeypatch.setattr(
        runner.gates, "aggregate_qualification_gate", lambda _summary: gate
    )
    monkeypatch.setattr(runner, "_record", lambda path: _artifact(Path(path).name))
    with pytest.raises(
        runner.V12R3P1QualificationExecutionError, match="aggregate 6/6 gate failed"
    ):
        runner._run_aggregate("finalize_qualification")
    failure = runner._mapping(final_root / "failure.json")
    assert failure["status"] == runner.STAGE_FAILURE_STATUS
    assert failure["passed"] is False
    assert failure["artifacts"] == {
        "summary": _artifact("summary.json"),
        "gate": _artifact("gate.json"),
    }
    assert failure["retry_authorized"] is False
    assert failure["resume_authorized"] is False


def test_terminal_ledger_verifier_requires_exact_canonical_bytes(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    path = tmp_path / "pipeline_ledger.json"
    payload = {
        "status": runner.LEDGER_FAIL_STATUS,
        "passed": False,
        "terminal": True,
    }
    monkeypatch.setattr(runner, "PIPELINE_LEDGER_PATH", path)
    monkeypatch.setattr(runner, "_ledger_payload", lambda **_kwargs: payload)
    forensic.write_json_exclusive(path, payload)
    assert (
        runner._verify_pipeline_ledger(
            passed=False,
            attempted_stage=contract.STAGE_IDS[0],
            completed_stages=[],
            error=RuntimeError("failure"),
        )
        == payload
    )
    path.write_text(json.dumps(payload, sort_keys=True), encoding="utf-8")
    with pytest.raises(
        runner.V12R3P1QualificationExecutionError, match="byte-canonical"
    ):
        runner._verify_pipeline_ledger(
            passed=False,
            attempted_stage=contract.STAGE_IDS[0],
            completed_stages=[],
            error=RuntimeError("failure"),
        )


def test_link_or_windows_reparse_path_is_rejected(tmp_path: Path) -> None:
    target = tmp_path / "target"
    target.mkdir()
    link = tmp_path / "link"
    try:
        link.symlink_to(target, target_is_directory=True)
    except OSError:
        pytest.skip("symlink creation unavailable")
    with pytest.raises(runner.V12R3P1QualificationExecutionError, match="link/reparse"):
        runner._assert_no_link_components(link / "claim.json")


def test_no_canonical_runtime_artifact_was_published() -> None:
    assert not os.path.lexists(runner.PROTOCOL_FREEZE_PATH)
    assert not os.path.lexists(runner.LOCK_PATH)
    assert not os.path.lexists(runner.NOISE_ROOT)
    assert not os.path.lexists(runner.RUN_ROOT)
