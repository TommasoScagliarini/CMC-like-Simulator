"""Non-mutating orchestration tests for the isolated P1 salvage runner.

The tests use only pure gates, stubs, and pytest temporary directories.  They
never publish the canonical protocol freeze/lock, load P1, build OpenSim, or
claim the canonical V12P1S run root.
"""

from __future__ import annotations

import copy
import inspect
import sys
from pathlib import Path, PurePosixPath
from typing import Any

import numpy as np
import pytest


LOCAL_ROOT = Path(__file__).resolve().parent
PARENT_VALIDATION = LOCAL_ROOT.parent
R3_ROOT = PARENT_VALIDATION / "v12r3"
ROOT_VALIDATION = next(
    parent / "validation"
    for parent in LOCAL_ROOT.parents
    if (parent / "AGENTS.md").is_file() and (parent / "validation").is_dir()
)
for _root in (LOCAL_ROOT, PARENT_VALIDATION, R3_ROOT, ROOT_VALIDATION):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_v12r3_p1_salvage_contract as contract  # noqa: E402
import run_h0_v12r3_p1_salvage_development as runner  # noqa: E402


_SHA = "ab" * 32


def test_failure_publication_errors_are_never_suppressed() -> None:
    source = "\n".join(
        (
            inspect.getsource(runner._run_rollout),
            inspect.getsource(runner.execute_development_once),
        )
    )
    assert "except Exception" not in source
    assert "failure = None" not in inspect.getsource(runner.execute_development_once)


def _artifact(path: str | PurePosixPath) -> dict[str, object]:
    return {
        "path": PurePosixPath(path).as_posix(),
        "sha256": _SHA,
        "size_bytes": 1,
    }


def _qualification_binding() -> dict[str, Any]:
    payload = {
        "schema_version": contract.QUALIFICATION_DESIGN_SCHEMA_VERSION,
        "status": contract.QUALIFICATION_DESIGN_FREEZE_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.QUALIFICATION_DESIGN_PROTOCOL_ID,
        "freeze_kind": contract.QUALIFICATION_DESIGN_FREEZE_KIND,
        "publication_destination": contract.QUALIFICATION_DESIGN_FREEZE_PATH.as_posix(),
        "checks": {"salvage_run_root_absent": True},
        "zero_design_activity": {"rollout_executions": 0},
        "zero_design_activity_gate": {"passed": True},
        "qualification_access": {
            "status": contract.QUALIFICATION_DESIGN_LOCKED_ACCESS_STATUS,
            "qualification_design_frozen": True,
            "qualification_protocol_freeze": None,
            "qualification_execution_lock": None,
            "future_prerequisite_hashes": None,
            "hash_binding_deferred_until_salvage_terminal_pass": True,
            "required_salvage_rollout_count": 6,
            "required_salvage_passing_rollout_count": 6,
            "required_salvage_failed_rollout_count": 0,
            "noise_materialization_authorized": False,
            "qualification_execution_authorized": False,
            "runtime_promotion_authorized": False,
        },
        "qualification_protocol_freeze": None,
        "qualification_execution_lock": None,
        "noise_manifest": None,
        "salvage_artifact_hashes": None,
        "runtime_promoted": False,
    }
    gate = contract.qualification_design_freeze_gate(payload)
    assert gate["passed"] is True
    return {
        "record": _artifact(contract.QUALIFICATION_DESIGN_FREEZE_PATH),
        "gate": gate,
    }


def _valid_physical(case_id: str) -> dict[str, Any]:
    case = contract.canonical_development_case(case_id)
    physical: dict[str, Any] = {
        "case_id": case_id,
        "action_selection": case["action_selection"],
        "episode_start_offset_s": case["episode_start_offset_s"],
        "action_seed": case["action_seed"],
        "runtime_seed": case["runtime_seed"],
        "sigma": case["sigma"],
        "steps": contract.EXPECTED_STEPS,
        "control_window_count": contract.EXPECTED_CONTROL_WINDOWS,
        "raw_sensor_sample_count": contract.EXPECTED_RAW_SENSOR_SAMPLES,
        "end_reason": "episode_time_limit",
        "terminated": False,
        "truncated": True,
        "phase_valid_cycle_count": 2,
        "grf_penetration_max_m": 0.024,
        "binary_phase_fsm_mode": "binary_active",
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "morphology_weight": contract.MORPHOLOGY_WEIGHT,
        "n_actor": contract.EXPECTED_ACTOR_FEATURES,
        "n_observation": contract.EXPECTED_FULL_FEATURES,
        "observation_dtype": contract.EXPECTED_DTYPE,
        "random_noise_draw_count": (
            contract.EXPECTED_STEPS if case["action_selection"] == "stochastic" else 0
        ),
        "single_noise_application_count": contract.EXPECTED_STEPS,
        "binary_phase_event_gate": {
            "passed": True,
            "sample_count": contract.EXPECTED_RAW_SENSOR_SAMPLES,
            "duplicate_event_count": 0,
            "out_of_order_event_count": 0,
            "left_non_v26_source_count": 0,
            "fallback_count": 0,
            "hard_invalid_count": 0,
        },
        "binary_event_prefix_integrity": {
            "passed": True,
            "sample_count": contract.EXPECTED_RAW_SENSOR_SAMPLES,
            "raw_sensor_sample_count": contract.EXPECTED_RAW_SENSOR_SAMPLES,
            "control_window_count": contract.EXPECTED_CONTROL_WINDOWS,
            "expected_sample_count": contract.EXPECTED_RAW_SENSOR_SAMPLES,
        },
        "episode_metrics": {
            "reserve_norm_nm": {
                "sample_count": contract.EXPECTED_STEPS,
                "rms": 210.0,
                "abs_max": 573.0,
            },
            "residual_norm_nm": {
                "sample_count": contract.EXPECTED_STEPS,
                "rms": 1.0e-10,
                "abs_max": 1.0e-8,
            },
        },
        "sea_episode_metrics": {
            joint: {
                **{
                    signal: {
                        "sample_count": contract.SEA_EXPECTED_SAMPLE_COUNTS[signal],
                        "rms": 1.0,
                        "abs_max": 2.0,
                    }
                    for signal in contract.SEA_SIGNALS
                },
                "tau_input_saturated": {
                    "sample_count": contract.EXPECTED_RAW_SENSOR_SAMPLES,
                    "count": 0,
                    "fraction": 0.0,
                },
            }
            for joint in contract.JOINTS
        },
    }
    physical.update({name: 0 for name in contract._ZERO_ROLLOUT_COUNTERS})
    return physical


def _built_valid_summary(case_id: str) -> dict[str, Any]:
    case = contract.canonical_development_case(case_id)
    root = PurePosixPath(case["destination"])
    stage_id = f"development__{case_id}"
    qualification = _qualification_binding()
    return runner._build_rollout_summary(
        physical=_valid_physical(case_id),
        case_id=case_id,
        stage_id=stage_id,
        row_count=contract.EXPECTED_STEPS,
        candidate_tree=copy.deepcopy(contract.P1_CANDIDATE_MODULE),
        artifacts={
            "run_start": _artifact(root / "run_start.json"),
            "trace": _artifact(root / "trace.json"),
            "partial_summary": _artifact(root / "partial_summary.json"),
        },
        protocol_freeze=_artifact(contract.PROTOCOL_FREEZE_PATH),
        execution_lock=_artifact(contract.EXECUTION_LOCK_PATH),
        pipeline_claim=_artifact(contract.PIPELINE_CLAIM_PATH),
        worker_claim=_artifact(contract.worker_claim_path(stage_id)),
        qualification_design_freeze=qualification["record"],
        qualification_design_freeze_gate=qualification["gate"],
    )


def test_topology_authority_and_isolated_paths_are_exact() -> None:
    assert len(contract.CASE_IDS) == 6
    assert len(contract.STAGE_IDS) == 7
    assert contract.STAGE_IDS[-1] == "finalize_development"
    assert [
        contract.stage_descriptor(stage)["kind"] for stage in contract.STAGE_IDS
    ] == ["development"] * 6 + ["finalize_development"]
    assert runner.EXECUTION_AUTHORITY["one_shot"] is True
    for name in (
        "retry_authorized",
        "resume_authorized",
        "fit_authorized",
        "actor_updates_authorized",
        "critic_updates_authorized",
        "ppo_updates_authorized",
        "teacher_authorized",
        "blending_authorized",
        "safety_latch_authorized",
        "runtime_promotion_authorized",
    ):
        assert runner.EXECUTION_AUTHORITY[name] is False
    source = Path(runner.__file__).read_text(encoding="utf-8")
    assert "/fit/p2" not in source
    assert "rl_module_target_adapted_p2" not in source
    assert contract.RUN_ROOT.as_posix().startswith(
        "Trajectory Generator/baseline_MLP/validation/v12p1s/"
    )


def test_transitive_runtime_source_closure_and_helper_surface_are_explicit() -> None:
    sources = runner.EXECUTION_SOURCE_RELATIVE_PATHS
    assert len(sources) == 69
    assert {
        "q_design_freezer",
        "q_contract",
        "v12r3_runtime_runner",
        "v10s_legacy_runner",
        "v9_environment_source",
        "v8r1p1_runner",
        "v8r1_runner",
        "v8_runner",
        "v6_runner",
        "v25_env_builder",
        "rollout_eval",
        "env_factory",
        "osim_environment",
        "simulation_runner",
    } <= set(sources)
    assert all((runner.REPO_ROOT / path).is_file() for path in sources.values())
    records = runner._execution_source_records()
    assert set(records) == set(sources)
    assert all(records[name]["path"] == sources[name] for name in sources)
    surface = runner._runtime_helper_surface_gate()
    assert surface["passed"] is True
    assert all(surface["checks"].values())


def test_execution_lock_requires_canonical_bytes(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    lock = tmp_path / "lock.json"
    payload = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.EXECUTION_LOCK_PASS_STATUS,
        "passed": True,
    }
    runner.forensic.write_json_exclusive(lock, payload)
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(runner, "LOCK_PATH", lock)
    monkeypatch.setattr(runner, "build_execution_lock", lambda **_kwargs: payload)
    assert runner.verify_execution_lock() == payload

    lock.write_bytes(lock.read_bytes() + b" ")
    with pytest.raises(runner.P1SalvageDevelopmentError, match="not canonical"):
        runner.verify_execution_lock()


def test_q_binding_mismatch_between_lock_and_current_fails_claim(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    current = _qualification_binding()
    frozen = copy.deepcopy(current)
    frozen["record"]["sha256"] = "cd" * 32
    monkeypatch.setattr(runner, "_qualification_design_freeze_binding", lambda: current)
    monkeypatch.setattr(
        runner,
        "_mapping",
        lambda _path: {
            "qualification_design_freeze": frozen["record"],
            "qualification_design_freeze_gate": frozen["gate"],
        },
    )
    with pytest.raises(runner.P1SalvageDevelopmentError, match="lock/current"):
        runner._pipeline_claim_payload("ab" * 32)


def test_q_byte_binding_mutation_after_lock_fails_lock_verification(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    lock = tmp_path / "lock.json"
    frozen = _qualification_binding()
    observed = {
        "schema_version": contract.SCHEMA_VERSION,
        "passed": True,
        "qualification_design_freeze": frozen["record"],
        "qualification_design_freeze_gate": frozen["gate"],
    }
    runner.forensic.write_json_exclusive(lock, observed)
    current = copy.deepcopy(observed)
    current["qualification_design_freeze"]["sha256"] = "ef" * 32
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(runner, "LOCK_PATH", lock)
    monkeypatch.setattr(runner, "build_execution_lock", lambda **_kwargs: current)
    with pytest.raises(runner.P1SalvageDevelopmentError, match="bindings drifted"):
        runner.verify_execution_lock()


@pytest.mark.parametrize("case_id", contract.CASE_IDS)
def test_runner_built_summary_passes_full_physical_and_sea_gate(
    case_id: str,
) -> None:
    summary = _built_valid_summary(case_id)
    gate = contract.development_rollout_gate(summary, case_id=case_id)
    assert gate["passed"] is True
    assert gate["checks"]["persisted_artifacts"] is True
    assert gate["checks"]["episode_metrics_finite_exact"] is True
    assert gate["checks"]["sea_metrics_finite_exact"] is True
    assert gate["checks"]["sea_saturation_zero"] is True
    assert summary["pipeline_claim"]["path"] == contract.PIPELINE_CLAIM_PATH.as_posix()
    assert summary["p2_artifacts_opened"] == []


def test_sea_saturation_or_sample_regression_fails_runner_summary_gate() -> None:
    summary = _built_valid_summary(contract.CASE_IDS[0])
    summary["sea_episode_metrics"]["pros_knee_angle"]["tau_input_saturated"][
        "count"
    ] = 1
    assert contract.development_rollout_gate(summary)["passed"] is False

    summary = _built_valid_summary(contract.CASE_IDS[0])
    summary["episode_metrics"]["reserve_norm_nm"]["sample_count"] = 499
    assert contract.development_rollout_gate(summary)["passed"] is False


def test_trace_row_requires_replayable_v26_observation() -> None:
    row = {
        "protocol_id": contract.PROTOCOL_ID,
        "v26_observation": [0.0] * contract.EXPECTED_ACTOR_FEATURES,
        "candidate_mean": [0.0, 0.0],
        "candidate_std": [0.005, 0.005],
        "standard_normal": [0.0, 0.0],
        "single_noise": [0.0, 0.0],
        "raw_action": [0.0, 0.0],
        "applied_action": [0.0, 0.0],
        "teacher_enabled": False,
        "teacher_query_count": 0,
        "served_action_teacher_dependency_count": 0,
        "blending_enabled": False,
        "safety_latch_enabled": False,
    }
    assert runner._trace_row_schema_valid(row) is True
    row.pop("v26_observation")
    assert runner._trace_row_schema_valid(row) is False


def test_frozen_innovations_are_exact_reproducible_and_condition_bound() -> None:
    deterministic = contract.canonical_development_case(contract.CASE_IDS[0])
    zero = runner._frozen_innovations(deterministic, np=np)
    assert zero.shape == (500, 2)
    assert zero.dtype == np.float32
    assert np.count_nonzero(zero) == 0

    stochastic = contract.canonical_development_case("stochastic_nominal_seed_127")
    first = runner._frozen_innovations(stochastic, np=np)
    second = runner._frozen_innovations(stochastic, np=np)
    expected = runner.r3_runner._frozen_innovations(
        "stochastic_nominal_seed_127", action_selection="stochastic", np=np
    )
    assert first.flags.c_contiguous
    assert first.tobytes() == second.tobytes() == expected.tobytes()
    fresh_draw = np.random.default_rng(127).standard_normal((500, 2)).astype(np.float32)
    assert first.tobytes() != fresh_draw.tobytes()


def test_rollout_publisher_rereads_run_start_and_detects_toctou(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    writer = runner.forensic.ForensicRolloutWriter(
        tmp_path / "rollout", artifact_root=tmp_path
    )
    writer.start({"kind": "development", "closed": False})
    writer.write_step(1, {"value": 7})
    original = writer.finalize_before_gate

    def finalize_then_tamper(**kwargs: object) -> dict[str, object]:
        records = original(**kwargs)
        writer.run_start_path.write_bytes(b'{"closed":true,"kind":"development"}\n')
        return records

    monkeypatch.setattr(writer, "finalize_before_gate", finalize_then_tamper)
    with pytest.raises(runner.P1SalvageDevelopmentError, match="run_start"):
        runner._publish_rollout_before_gate(
            writer,
            rows=[{"step": 1, "value": 7}],
            partial={"steps": 1},
            summary={"steps": 1},
        )


def test_symlink_or_windows_reparse_paths_fail_closed(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    target = tmp_path / "target"
    target.mkdir()
    link = tmp_path / "link"
    try:
        link.symlink_to(target, target_is_directory=True)
    except OSError:
        pytest.skip("symlink creation unavailable on this platform")
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    with pytest.raises(runner.P1SalvageDevelopmentError, match="link/reparse"):
        runner._assert_no_link_components(link / "artifact.json")

    monkeypatch.setattr(
        runner.r3_runner,
        "_runtime_record",
        lambda: {
            "inference_stack_ready": True,
            "platform_plugin_readiness": {"passed": False},
        },
    )
    with pytest.raises(runner.P1SalvageDevelopmentError, match="plugins"):
        runner._runtime_record()


def test_execute_verifies_lock_before_claim_and_success_has_no_attempted_stage(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    events: list[tuple[str, object]] = []
    captured: dict[str, Any] = {}
    ledger_path = tmp_path / "pipeline_ledger.json"
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(runner, "PIPELINE_LEDGER_PATH", ledger_path)
    monkeypatch.setattr(
        runner,
        "verify_execution_lock",
        lambda **kwargs: events.append(("verify_lock", kwargs)) or {"passed": True},
    )
    monkeypatch.setattr(
        runner,
        "_claim_run_root",
        lambda: events.append(("claim", None)) or "x" * 48,
    )
    monkeypatch.setattr(
        runner,
        "_write_worker_claim",
        lambda stage, _digest: events.append(("worker_claim", stage)),
    )

    def run_stage(stage: str) -> dict[str, Any]:
        events.append(("run", stage))
        if contract.stage_descriptor(stage)["kind"] == "development":
            runner._ACTIVITY["environment_reset_calls"] += 1
            runner._ACTIVITY["environment_step_calls"] += 500
            runner._ACTIVITY["candidate_mean_queries"] += 500
        return {"passed": True}

    monkeypatch.setattr(runner, "_run_stage", run_stage)
    monkeypatch.setattr(
        runner,
        "_verify_stage_receipt",
        lambda stage, **_kwargs: {"stage_id": stage, "passed": True},
    )

    def ledger_payload(**kwargs: Any) -> dict[str, Any]:
        captured.update(kwargs)
        return {"status": "PASS_TEST", "passed": kwargs["passed"]}

    monkeypatch.setattr(runner, "_ledger_payload", ledger_payload)
    monkeypatch.setattr(
        runner,
        "verify_terminal_ledger",
        lambda **_kwargs: runner.forensic.strict_json_load(ledger_path),
    )
    result = runner.execute_development_once()
    assert result == {"passed": True, "status": "PASS_TEST"}
    assert events[:2] == [
        ("verify_lock", {"require_run_root_absent": True}),
        ("claim", None),
    ]
    assert [row[1] for row in events if row[0] == "run"] == list(contract.STAGE_IDS)
    assert captured["attempted_stage"] is None
    assert captured["completed_stages"] == list(contract.STAGE_IDS)


def test_claim_failure_after_root_creation_publishes_terminal_preclaim_ledger(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    run_root = tmp_path / "run"
    lock = tmp_path / "lock.json"
    protocol = tmp_path / "protocol.json"
    runner.forensic.write_json_exclusive(lock, {"passed": True})
    runner.forensic.write_json_exclusive(protocol, {"passed": True})
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(runner, "RUN_ROOT", run_root)
    monkeypatch.setattr(runner, "LOCK_PATH", lock)
    monkeypatch.setattr(runner, "PROTOCOL_FREEZE_PATH", protocol)
    monkeypatch.setattr(runner, "PIPELINE_CLAIM_PATH", run_root / "claim.json")
    monkeypatch.setattr(runner, "PIPELINE_LEDGER_PATH", run_root / "ledger.json")
    monkeypatch.setattr(
        runner,
        "_pipeline_claim_payload",
        lambda _digest: (_ for _ in ()).throw(RuntimeError("claim payload failed")),
    )
    monkeypatch.setattr(
        runner, "_frozen_qualification_design_binding", _qualification_binding
    )

    with pytest.raises(RuntimeError, match="claim payload failed"):
        runner._claim_run_root()
    ledger = runner.forensic.strict_json_load(run_root / "ledger.json")
    assert ledger["passed"] is False
    assert ledger["terminal"] is True
    assert ledger["attempted_stage"] is None
    assert ledger["claim_completed"] is False
    assert ledger["claim_published"] is False
    assert ledger["claim_verified"] is False
    assert ledger["retry_authorized"] is False


def test_preclaim_exclusive_write_failure_is_not_misreported_as_published(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    run_root = tmp_path / "run"
    lock = runner.forensic.write_json_exclusive(tmp_path / "lock.json", {"ok": True})
    protocol = runner.forensic.write_json_exclusive(
        tmp_path / "protocol.json", {"ok": True}
    )
    claim = run_root / "claim.json"
    ledger = run_root / "ledger.json"
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(runner, "RUN_ROOT", run_root)
    monkeypatch.setattr(runner, "LOCK_PATH", lock)
    monkeypatch.setattr(runner, "PROTOCOL_FREEZE_PATH", protocol)
    monkeypatch.setattr(runner, "PIPELINE_CLAIM_PATH", claim)
    monkeypatch.setattr(runner, "PIPELINE_LEDGER_PATH", ledger)
    monkeypatch.setattr(
        runner, "_frozen_qualification_design_binding", _qualification_binding
    )
    monkeypatch.setattr(
        runner,
        "_pipeline_claim_payload",
        lambda digest: {"execution_token_sha256": digest},
    )
    original_write = runner.forensic.write_json_exclusive

    def fail_claim_write(path: Path, payload: Any) -> Path:
        if Path(path) == claim:
            raise OSError("exclusive claim write failed")
        return original_write(path, payload)

    monkeypatch.setattr(runner.forensic, "write_json_exclusive", fail_claim_write)
    with pytest.raises(OSError, match="exclusive claim write failed"):
        runner._claim_run_root()
    observed = runner.forensic.strict_json_load(ledger)
    assert observed["claim_published"] is False
    assert observed["claim_verified"] is False
    assert observed["claim_completed"] is False
    assert observed["pipeline_claim"] is None
    assert not claim.exists()


def test_preclaim_post_write_verification_failure_preserves_claim_snapshot(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    run_root = tmp_path / "run"
    lock = runner.forensic.write_json_exclusive(tmp_path / "lock.json", {"ok": True})
    protocol = runner.forensic.write_json_exclusive(
        tmp_path / "protocol.json", {"ok": True}
    )
    claim = run_root / "claim.json"
    ledger = run_root / "ledger.json"
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(runner, "RUN_ROOT", run_root)
    monkeypatch.setattr(runner, "LOCK_PATH", lock)
    monkeypatch.setattr(runner, "PROTOCOL_FREEZE_PATH", protocol)
    monkeypatch.setattr(runner, "PIPELINE_CLAIM_PATH", claim)
    monkeypatch.setattr(runner, "PIPELINE_LEDGER_PATH", ledger)
    monkeypatch.setattr(
        runner, "_frozen_qualification_design_binding", _qualification_binding
    )
    monkeypatch.setattr(
        runner,
        "_pipeline_claim_payload",
        lambda digest: {"execution_token_sha256": digest},
    )
    monkeypatch.setattr(
        runner,
        "_verify_pipeline_claim",
        lambda: (_ for _ in ()).throw(RuntimeError("post-write verify failed")),
    )
    with pytest.raises(RuntimeError, match="post-write verify failed"):
        runner._claim_run_root()
    observed = runner.forensic.strict_json_load(ledger)
    assert observed["claim_published"] is True
    assert observed["claim_verified"] is False
    assert observed["claim_completed"] is False
    assert observed["pipeline_claim"] == runner._record(claim)


def test_stage_failure_receipt_is_full_terminal_and_p2_closed(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    lock = runner.forensic.write_json_exclusive(tmp_path / "lock.json", {"ok": True})
    freeze = runner.forensic.write_json_exclusive(
        tmp_path / "freeze.json", {"ok": True}
    )
    claim = runner.forensic.write_json_exclusive(
        tmp_path / "pipeline_claim.json", {"ok": True}
    )
    worker = runner.forensic.write_json_exclusive(
        tmp_path / "worker_claim.json", {"ok": True}
    )
    failure = tmp_path / "failure.json"
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(runner, "LOCK_PATH", lock)
    monkeypatch.setattr(runner, "PROTOCOL_FREEZE_PATH", freeze)
    monkeypatch.setattr(runner, "PIPELINE_CLAIM_PATH", claim)
    monkeypatch.setattr(runner, "_claim_path", lambda _stage: worker)
    monkeypatch.setattr(runner, "_stage_failure_path", lambda _stage: failure)
    monkeypatch.setattr(
        runner,
        "_candidate_tree",
        lambda: copy.deepcopy(contract.P1_CANDIDATE_MODULE),
    )
    monkeypatch.setattr(
        runner, "_frozen_qualification_design_binding", _qualification_binding
    )

    payload = runner._publish_stage_failure(
        contract.STAGE_IDS[0], RuntimeError("terminal test failure")
    )
    assert payload["passed"] is False
    assert payload["terminal"] is True
    assert payload["candidate_module"] == contract.P1_CANDIDATE_MODULE
    assert payload["p2_artifacts_opened"] == []
    assert payload["p2_module_loaded"] is False
    assert payload["p2_corpus_loaded"] is False
    assert payload["fit_executions"] == 0
    assert payload["actor_updates"] == 0
    assert payload["retry_authorized"] is False


def test_run_root_claim_is_one_shot_without_resume(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    run_root = tmp_path / "run"
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(runner, "RUN_ROOT", run_root)
    monkeypatch.setattr(runner, "PIPELINE_CLAIM_PATH", run_root / "claim.json")
    monkeypatch.setattr(runner, "PIPELINE_LEDGER_PATH", run_root / "ledger.json")
    monkeypatch.setattr(
        runner,
        "_pipeline_claim_payload",
        lambda digest: {"execution_token_sha256": digest, "passed": True},
    )
    monkeypatch.setattr(
        runner,
        "_verify_pipeline_claim",
        lambda: runner.forensic.strict_json_load(run_root / "claim.json"),
    )
    token = runner._claim_run_root()
    assert len(token) >= 32
    with pytest.raises(runner.P1SalvageDevelopmentError, match="no-resume"):
        runner._claim_run_root()


def test_stage_dispatch_exposes_only_six_rollouts_and_aggregate(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[tuple[str, str]] = []
    monkeypatch.setattr(
        runner,
        "_run_rollout",
        lambda case, stage: calls.append(("rollout", case)) or {"passed": True},
    )
    monkeypatch.setattr(
        runner,
        "_run_aggregate",
        lambda stage: calls.append(("aggregate", stage)) or {"passed": True},
    )
    for stage in contract.STAGE_IDS:
        assert runner._run_stage(stage)["passed"] is True
    assert calls == [("rollout", case_id) for case_id in contract.CASE_IDS] + [
        ("aggregate", "finalize_development")
    ]


def test_aggregate_refuses_publication_before_terminal_activity_is_exact(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    for name in runner._ACTIVITY:
        monkeypatch.setitem(runner._ACTIVITY, name, 0)
    final_root = tmp_path.joinpath(*contract.FINAL_ROOT.parts)
    with pytest.raises(runner.P1SalvageDevelopmentError, match="activity counters"):
        runner._run_aggregate("finalize_development")
    assert not final_root.exists()


def test_terminal_pass_ledger_is_reconstructed_and_byte_canonical(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    protocol = runner.forensic.write_json_exclusive(
        tmp_path / "protocol.json", {"ok": True}
    )
    lock = runner.forensic.write_json_exclusive(tmp_path / "lock.json", {"ok": True})
    claim = runner.forensic.write_json_exclusive(
        tmp_path / "pipeline_claim.json", {"ok": True}
    )
    ledger = tmp_path / "pipeline_ledger.json"
    receipt_root = tmp_path / "receipts"
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(runner, "PROTOCOL_FREEZE_PATH", protocol)
    monkeypatch.setattr(runner, "LOCK_PATH", lock)
    monkeypatch.setattr(runner, "PIPELINE_CLAIM_PATH", claim)
    monkeypatch.setattr(runner, "PIPELINE_LEDGER_PATH", ledger)
    monkeypatch.setattr(
        runner,
        "_stage_receipt_path",
        lambda stage: receipt_root / f"{stage}.json",
    )
    for stage in contract.STAGE_IDS:
        runner.forensic.write_json_exclusive(
            receipt_root / f"{stage}.json", {"stage_id": stage, "passed": True}
        )
    monkeypatch.setattr(
        runner, "_frozen_qualification_design_binding", _qualification_binding
    )
    monkeypatch.setattr(
        runner,
        "_candidate_tree",
        lambda: copy.deepcopy(contract.P1_CANDIDATE_MODULE),
    )
    monkeypatch.setattr(
        runner,
        "_verify_stage_receipt",
        lambda stage, **_kwargs: {"stage_id": stage, "passed": True},
    )
    for name, value in runner.EXPECTED_TERMINAL_ACTIVITY.items():
        monkeypatch.setitem(runner._ACTIVITY, name, value)
    payload = runner._ledger_payload(
        passed=True,
        attempted_stage=None,
        completed_stages=contract.STAGE_IDS,
        error=None,
        stage_failure=None,
    )
    runner.forensic.write_json_exclusive(ledger, payload)
    assert runner.verify_terminal_ledger() == payload

    ledger.write_bytes(ledger.read_bytes() + b" ")
    with pytest.raises(runner.P1SalvageDevelopmentError, match="not canonical"):
        runner.verify_terminal_ledger()


def test_terminal_failure_ledger_validates_prefix_failure_and_activity(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    protocol = runner.forensic.write_json_exclusive(
        tmp_path / "protocol.json", {"ok": True}
    )
    lock = runner.forensic.write_json_exclusive(tmp_path / "lock.json", {"ok": True})
    claim = runner.forensic.write_json_exclusive(
        tmp_path / "pipeline_claim.json", {"ok": True}
    )
    failure = runner.forensic.write_json_exclusive(
        tmp_path / "failure.json", {"passed": False}
    )
    ledger = tmp_path / "pipeline_ledger.json"
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(runner, "PROTOCOL_FREEZE_PATH", protocol)
    monkeypatch.setattr(runner, "LOCK_PATH", lock)
    monkeypatch.setattr(runner, "PIPELINE_CLAIM_PATH", claim)
    monkeypatch.setattr(runner, "PIPELINE_LEDGER_PATH", ledger)
    monkeypatch.setattr(runner, "_stage_failure_path", lambda _stage: failure)
    monkeypatch.setattr(
        runner, "_frozen_qualification_design_binding", _qualification_binding
    )
    for name in runner._ACTIVITY:
        monkeypatch.setitem(runner._ACTIVITY, name, 0)
    payload = runner._ledger_payload(
        passed=False,
        attempted_stage=contract.STAGE_IDS[0],
        completed_stages=[],
        error=RuntimeError("terminal failure"),
        stage_failure=runner._record(failure),
    )
    runner.forensic.write_json_exclusive(ledger, payload)
    assert (
        runner.verify_terminal_ledger(require_pass=False, expected_failure=payload)
        == payload
    )

    broken = copy.deepcopy(payload)
    broken["activity"]["environment_step_calls"] = 1
    ledger.write_bytes(runner.forensic.canonical_json_bytes(broken))
    with pytest.raises(runner.P1SalvageDevelopmentError, match="failure ledger"):
        runner.verify_terminal_ledger(require_pass=False)


def test_aggregate_receipt_verifier_requires_exact_six_case_receipts(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    virtual = Path("/virtual")
    final_root = virtual / "final"
    receipt_root = virtual / "receipts"
    claim_root = virtual / "claims"
    pipeline_claim = virtual / "pipeline_claim.json"
    qualification = _qualification_binding()
    stage_id = "finalize_development"
    receipt_path = receipt_root / f"{stage_id}.json"
    summary_path = final_root / "summary.json"
    gate_path = final_root / "gate.json"

    def record(path: str | Path) -> dict[str, object]:
        return _artifact(PurePosixPath(Path(path).as_posix()))

    summary = {
        "qualification_design_freeze": qualification["record"],
        "qualification_design_freeze_gate": qualification["gate"],
    }
    gate = {"passed": True, "status": contract.FINAL_DEVELOPMENT_PASS_STATUS}
    expected_case_receipts = [
        record(receipt_root / f"{stage}.json") for stage in contract.STAGE_IDS[:-1]
    ]
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": stage_id,
        "candidate_id": contract.P1_CANDIDATE_ID,
        "pipeline_claim": record(pipeline_claim),
        "worker_claim": record(claim_root / f"{stage_id}.json"),
        "qualification_design_freeze": qualification["record"],
        "qualification_design_freeze_gate": qualification["gate"],
        "execution_authority": copy.deepcopy(runner.EXECUTION_AUTHORITY),
        "retry_authorized": False,
        "resume_authorized": False,
        "fit_executions": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "teacher_queries": 0,
        "blend_count": 0,
        "latch_count": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "summary": record(summary_path),
        "gate": record(gate_path),
        "passed": True,
        "case_count": 6,
        "case_receipts": expected_case_receipts,
    }
    payloads = {receipt_path: receipt, summary_path: summary, gate_path: gate}
    monkeypatch.setattr(
        runner, "_stage_receipt_path", lambda stage: receipt_root / f"{stage}.json"
    )
    monkeypatch.setattr(
        runner, "_claim_path", lambda stage: claim_root / f"{stage}.json"
    )
    monkeypatch.setattr(
        runner,
        "resolve_relative",
        lambda path: final_root if path == contract.FINAL_ROOT else Path(path),
    )
    monkeypatch.setattr(runner, "PIPELINE_CLAIM_PATH", pipeline_claim)
    monkeypatch.setattr(runner, "_canonical_mapping", lambda path: payloads[Path(path)])
    monkeypatch.setattr(runner, "_record", record)
    monkeypatch.setattr(
        runner,
        "_record_matches",
        lambda value, path: value == record(path),
    )
    monkeypatch.setattr(runner, "_verify_worker_claim", lambda _stage: {})
    monkeypatch.setattr(
        runner, "_qualification_design_freeze_binding", lambda: qualification
    )
    monkeypatch.setattr(runner, "_aggregate_gate", lambda _summary: gate)
    assert runner._verify_stage_receipt(stage_id)["passed"] is True

    receipt["case_receipts"] = expected_case_receipts[:-1]
    with pytest.raises(runner.P1SalvageDevelopmentError, match="aggregate receipt"):
        runner._verify_stage_receipt(stage_id)


def test_fake_env_rollout_uses_only_exact_p1_mean_plus_noise_surface(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    case_id = contract.CASE_IDS[0]
    stage_id = f"development__{case_id}"
    qualification = _qualification_binding()

    def local(path: PurePosixPath) -> Path:
        return tmp_path.joinpath(*path.parts)

    protocol = runner.forensic.write_json_exclusive(
        local(contract.PROTOCOL_FREEZE_PATH), {"passed": True}
    )
    lock = runner.forensic.write_json_exclusive(
        local(contract.EXECUTION_LOCK_PATH), {"passed": True}
    )
    pipeline_claim = runner.forensic.write_json_exclusive(
        local(contract.PIPELINE_CLAIM_PATH), {"passed": True}
    )
    worker_claim = runner.forensic.write_json_exclusive(
        local(contract.worker_claim_path(stage_id)), {"passed": True}
    )
    candidate_path = local(PurePosixPath(contract.P1_CANDIDATE_MODULE["path"]))
    candidate_path.mkdir(parents=True)

    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(runner, "PROTOCOL_FREEZE_PATH", protocol)
    monkeypatch.setattr(runner, "LOCK_PATH", lock)
    monkeypatch.setattr(runner, "PIPELINE_CLAIM_PATH", pipeline_claim)
    monkeypatch.setattr(
        runner, "_qualification_design_freeze_binding", lambda: qualification
    )
    monkeypatch.setattr(
        runner,
        "_verify_pipeline_claim",
        lambda: {
            "qualification_design_freeze": qualification["record"],
            "qualification_design_freeze_gate": qualification["gate"],
        },
    )
    monkeypatch.setattr(
        runner,
        "_candidate_tree",
        lambda: copy.deepcopy(contract.P1_CANDIDATE_MODULE),
    )
    monkeypatch.setattr(runner, "_candidate_module_path", lambda: candidate_path)
    monkeypatch.setattr(
        runner,
        "_frozen_innovations",
        lambda _case, *, np: np.zeros((contract.EXPECTED_STEPS, 2), dtype=np.float32),
    )
    monkeypatch.setattr(
        runner,
        "_verify_stage_receipt",
        lambda _stage, **_kwargs: {"passed": True},
    )
    for name in runner._ACTIVITY:
        monkeypatch.setitem(runner._ACTIVITY, name, 0)

    calls = {"checkpoint": 0, "query": 0, "action": 0}
    checkpoint_paths: list[Path] = []

    class FakeModule:
        @classmethod
        def from_checkpoint(cls, path: Path) -> "FakeModule":
            calls["checkpoint"] += 1
            checkpoint_paths.append(Path(path))
            return cls()

        def eval(self) -> None:
            return None

    class FakeActionSpace:
        low = np.asarray([-1.0, -1.0], dtype=np.float32)
        high = np.asarray([1.0, 1.0], dtype=np.float32)

    class FakeEnv:
        action_space = FakeActionSpace()

        def reset(self, *, seed: int) -> tuple[np.ndarray, dict[str, Any]]:
            assert seed == contract.canonical_development_case(case_id)["runtime_seed"]
            return np.zeros(contract.EXPECTED_FULL_FEATURES, dtype=np.float32), {}

        def step(
            self, action: np.ndarray
        ) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
            calls["action"] += 1
            assert action.shape == (2,)
            assert action.dtype == np.float32
            assert np.count_nonzero(action) == 0
            return (
                np.zeros(contract.EXPECTED_FULL_FEATURES, dtype=np.float32),
                0.0,
                False,
                calls["action"] == contract.EXPECTED_STEPS,
                {
                    "time": calls["action"] * 0.01,
                    "end_reason": "episode_time_limit",
                },
            )

        def close(self) -> None:
            return None

    env = FakeEnv()

    class FakeFactory:
        @staticmethod
        def make_cmc_env(_config: dict[str, Any]) -> FakeEnv:
            return env

    class FakeLegacy:
        @staticmethod
        def _jsonable(value: Any) -> Any:
            return value

    def query_mean_std(
        _module: Any, actor: np.ndarray, *, np: Any, torch: Any
    ) -> tuple[np.ndarray, np.ndarray]:
        del torch
        calls["query"] += 1
        assert actor.shape == (contract.EXPECTED_ACTOR_FEATURES,)
        return (
            np.zeros(2, dtype=np.float32),
            np.full(2, 0.005, dtype=np.float32),
        )

    monkeypatch.setattr(
        runner.r3_runner,
        "_load_rollout_stack",
        lambda: (
            object(),
            np,
            object(),
            FakeModule,
            FakeFactory,
            FakeLegacy,
            object(),
        ),
    )
    monkeypatch.setattr(
        runner.r3_runner,
        "_validate_runtime_layout",
        lambda **_kwargs: (
            tuple(f"actor_{index}" for index in range(35)),
            tuple(f"full_{index}" for index in range(84)),
        ),
    )
    monkeypatch.setattr(runner.r3_runner, "_new_physical_audit", lambda **_kwargs: {})
    monkeypatch.setattr(runner.r3_runner, "_query_mean_std", query_mean_std)
    monkeypatch.setattr(
        runner.r3_runner,
        "_consume_physical_step",
        lambda _audit, **_kwargs: {
            "penetration_m": 0.0,
            "phase": {"state_name": "STANCE"},
            "reserve_norm_nm": 0.0,
            "residual_norm_nm": 0.0,
            "checks": {"passed": True},
        },
    )
    monkeypatch.setattr(
        runner.r3_runner,
        "_physical_summary",
        lambda _audit, **_kwargs: _valid_physical(case_id),
    )

    forbidden_calls: list[str] = []

    def forbidden(name: str) -> Any:
        def trap(*_args: Any, **_kwargs: Any) -> None:
            forbidden_calls.append(name)
            raise AssertionError(f"forbidden runtime API invoked: {name}")

        return trap

    sensitive = (
        (runner.r3_runner, "_run_fit"),
        (runner.r3_runner, "_run_label"),
        (runner.r3_runner, "_run_collect"),
        (runner.r3_runner, "_run_stage"),
        (runner.r3_runner.fit_engine, "run_fit_stage"),
        (runner.r3_runner.label_engine, "run_observer_label_stage"),
        (runner.r3_runner.v10s_fit, "_write_npz_exclusive"),
        (runner.r3_runner.coherent_teacher, "build_teacher_view"),
        (runner.r3_runner.coherent_teacher, "assert_coherent_pair"),
        (runner.r3_runner.coherent_teacher, "LegacyGaitShadow"),
        (runner.r3_runner.safe_dagger, "SafetyLatchState"),
        (runner.r3_runner.safe_dagger, "select_safe_dagger_action"),
        (runner.r3_runner.safe_dagger, "blend_policy_means"),
        (runner.r3_runner.safe_dagger, "apply_single_noise"),
    )
    for owner, name in sensitive:
        monkeypatch.setattr(owner, name, forbidden(name), raising=False)

    receipt = runner._run_rollout(case_id, stage_id)
    assert receipt["passed"] is True
    assert calls == {"checkpoint": 1, "query": 500, "action": 500}
    assert checkpoint_paths == [candidate_path]
    assert forbidden_calls == []
    assert receipt["qualification_design_freeze"] == qualification["record"]
    assert worker_claim.is_file()
