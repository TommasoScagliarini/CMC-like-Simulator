from __future__ import annotations

import ast
import copy
from pathlib import Path

import pytest

import h0_v12r5_case_balanced_contract as contract
import run_h0_v12r5_case_balanced as runner


def _configure_claim_fault_paths(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> tuple[Path, Path, Path]:
    run_root = tmp_path / "run"
    claim_path = run_root / "pipeline_claim.json"
    failure_path = run_root / "pipeline_claim_failure.json"
    worker_root = run_root / "worker_claims"
    lock_path = tmp_path / "execution_lock.json"
    protocol_path = tmp_path / "protocol_freeze.json"
    design_audit_path = tmp_path / "design_audit.json"
    lock_path.write_text("{}", encoding="utf-8")
    protocol_path.write_text("{}", encoding="utf-8")
    design_audit_path.write_text("{}", encoding="utf-8")

    def fake_record(path: str | Path) -> dict[str, object]:
        source = Path(path)
        data = source.read_bytes()
        return {
            "path": source.as_posix(),
            "sha256": runner.hashlib.sha256(data).hexdigest(),
            "size_bytes": len(data),
        }

    monkeypatch.setattr(runner, "RUN_ROOT", run_root)
    monkeypatch.setattr(runner, "PIPELINE_CLAIM_PATH", claim_path)
    monkeypatch.setattr(runner, "PIPELINE_CLAIM_FAILURE_PATH", failure_path)
    monkeypatch.setattr(runner, "WORKER_CLAIMS_ROOT", worker_root)
    monkeypatch.setattr(runner, "LOCK_PATH", lock_path)
    monkeypatch.setattr(runner, "PROTOCOL_FREEZE_PATH", protocol_path)
    monkeypatch.setattr(runner, "DESIGN_AUDIT_PATH", design_audit_path)
    monkeypatch.setattr(runner, "_record", fake_record)
    monkeypatch.setattr(
        runner, "verify_execution_lock", lambda **_kwargs: {"passed": True}
    )
    monkeypatch.setattr(runner, "_qualification_unopened", lambda: True)
    monkeypatch.setattr(
        runner,
        "_opened_qualification_paths",
        lambda: {"q2": [], "q3": []},
    )
    runner._reset_activity()
    return claim_path, failure_path, worker_root


def _configure_terminal_fault_paths(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> tuple[Path, Path]:
    run_root = tmp_path / "run"
    run_root.mkdir()
    worker_claims_root = run_root / "worker_claims"
    worker_claims_root.mkdir()
    ledger_path = run_root / "pipeline_ledger.json"
    failure_path = run_root / "terminal_publication_failure.json"
    lock_path = tmp_path / "execution_lock.json"
    protocol_path = tmp_path / "protocol_freeze.json"
    claim_path = run_root / "pipeline_claim.json"
    for path in (lock_path, protocol_path, claim_path):
        path.write_text("{}", encoding="utf-8")
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(runner, "RUN_ROOT", run_root)
    monkeypatch.setattr(runner, "WORKER_CLAIMS_ROOT", worker_claims_root)
    monkeypatch.setattr(
        runner, "CANDIDATE_FREEZE_PATH", run_root / "candidate_freeze_receipt.json"
    )
    monkeypatch.setattr(runner, "PIPELINE_LEDGER_PATH", ledger_path)
    monkeypatch.setattr(
        runner,
        "PIPELINE_TERMINAL_PUBLICATION_FAILURE_PATH",
        failure_path,
    )
    monkeypatch.setattr(runner, "LOCK_PATH", lock_path)
    monkeypatch.setattr(runner, "PROTOCOL_FREEZE_PATH", protocol_path)
    monkeypatch.setattr(runner, "PIPELINE_CLAIM_PATH", claim_path)
    monkeypatch.setattr(
        runner, "PIPELINE_CLAIM_FAILURE_PATH", run_root / "pipeline_claim_failure.json"
    )
    receipt_root = tmp_path / "receipts"
    receipt_root.mkdir()
    receipt_paths = {
        stage_id: receipt_root / f"{index:02d}_{stage_id}.json"
        for index, stage_id in enumerate(contract.STAGE_IDS)
    }
    for path in receipt_paths.values():
        path.write_text("{}", encoding="utf-8")
    monkeypatch.setattr(
        runner, "_stage_receipt_path", lambda stage_id: receipt_paths[stage_id]
    )
    monkeypatch.setattr(
        runner,
        "_verify_stage_receipt_claim_bindings",
        lambda stage_id, *_args, **_kwargs: {"stage_id": stage_id},
    )
    monkeypatch.setattr(runner, "_completed_fit_lbfgs_closure_calls_current", lambda: 1)
    monkeypatch.setattr(runner, "_pipeline_claim_payload_valid", lambda _value: True)
    monkeypatch.setattr(
        runner, "_verify_worker_claims_root_exact", lambda **_kwargs: None
    )
    monkeypatch.setattr(runner, "_qualification_unopened", lambda: True)
    monkeypatch.setattr(
        runner,
        "_opened_qualification_paths",
        lambda: {"q2": [], "q3": []},
    )
    monkeypatch.setattr(
        runner,
        "_verify_terminal_ledger_payload",
        lambda payload, **_kwargs: dict(payload),
    )
    return ledger_path, failure_path


def _configure_lock_publication_fault_paths(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> tuple[Path, Path, dict[str, object]]:
    lock_path = tmp_path / "execution_lock.json"
    failure_path = tmp_path / "execution_lock_publication_failure.json"
    protocol_path = tmp_path / "protocol_freeze.json"
    design_path = tmp_path / "design_audit.json"
    protocol_path.write_text("{}", encoding="utf-8")
    design_path.write_text("{}", encoding="utf-8")
    expected = dict.fromkeys(runner._EXECUTION_LOCK_FIELDS)
    expected["schema_version"] = contract.SCHEMA_VERSION
    expected["status"] = contract.EXECUTION_LOCK_STATUS
    expected["passed"] = True
    expected["protocol_id"] = contract.PROTOCOL_ID
    expected["pipeline_id"] = contract.PIPELINE_ID
    expected["checks"] = dict.fromkeys(runner._EXECUTION_LOCK_CHECK_FIELDS, True)
    expected["occupancy"] = dict.fromkeys(runner._EXECUTION_LOCK_OCCUPANCY_FIELDS, True)
    expected["platform"] = dict.fromkeys(runner._PLATFORM_FIELDS, "value")
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(runner, "LOCK_PATH", lock_path)
    monkeypatch.setattr(runner, "LOCK_PUBLICATION_FAILURE_PATH", failure_path)
    monkeypatch.setattr(runner, "PROTOCOL_FREEZE_PATH", protocol_path)
    monkeypatch.setattr(runner, "DESIGN_AUDIT_PATH", design_path)
    monkeypatch.setattr(runner, "build_execution_lock", lambda **_kwargs: expected)
    monkeypatch.setattr(runner, "_execution_lock_payload_valid", lambda _value: True)
    monkeypatch.setattr(
        runner,
        "_opened_qualification_paths",
        lambda: {"q2": [], "q3": []},
    )
    monkeypatch.setattr(runner, "_qualification_unopened", lambda: True)
    return lock_path, failure_path, expected


def _completed_stage_activity_rows() -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for stage_id in contract.STAGE_IDS:
        row: dict[str, object] = {
            "stage_id": stage_id,
            "stage_kind": contract.stage_descriptor(stage_id)["kind"],
            **{name: 0 for name in runner._ACTIVITY_NAMES},
        }
        if stage_id == "fit_case_balanced_candidate":
            row.update(
                {
                    "actor_fit_stage_calls_attempted": 1,
                    "actor_fit_executions_confirmed": 1,
                    "actor_updates_attempted": 1,
                    "actor_updates_confirmed": 1,
                    "adamw_epochs_completed": 3000,
                    "lbfgs_closure_calls": 1,
                }
            )
        if contract.stage_descriptor(stage_id)["kind"] == "development":
            row.update(
                {
                    "environment_reset_calls": 1,
                    "environment_step_calls": contract.EXPECTED_STEPS,
                    "raw_sensor_sample_count": contract.EXPECTED_RAW_SENSOR_SAMPLES,
                }
            )
        rows.append(row)
    return rows


def _seed_stage_activity(*, completed_count: int) -> None:
    runner._reset_activity()
    rows = _completed_stage_activity_rows()
    for row in rows[:completed_count]:
        runner._begin_stage_activity(str(row["stage_id"]))
        for name in runner._ACTIVITY_NAMES:
            if row[name]:
                runner._activity_increment(name, int(row[name]))
    if completed_count < len(contract.STAGE_IDS):
        runner._begin_stage_activity(contract.STAGE_IDS[completed_count])


def _all_stages_intended_pass() -> dict[str, object]:
    stage_activity = _completed_stage_activity_rows()
    totals = {
        name: sum(int(row[name]) for row in stage_activity)
        for name in runner._ACTIVITY_NAMES
    }
    payload = dict.fromkeys(runner._TERMINAL_LEDGER_FIELDS)
    payload.update(
        {
            "passed": True,
            "completed_stages": [
                {
                    "stage_id": stage_id,
                    "receipt": runner._record(runner._stage_receipt_path(stage_id)),
                }
                for stage_id in contract.STAGE_IDS
            ],
            "completed_stage_count": len(contract.STAGE_IDS),
            "attempted_stage": contract.STAGE_IDS[-1],
            "activity_totals": totals,
            "stage_activity": stage_activity,
            "error": None,
            "attempted_stage_failure_publication_error": None,
            "next_stage": "WAIT_SEPARATE_Q3_PROTOCOL",
        }
    )
    return payload


def _valid_development_summary(
    *,
    case_id: str,
    module: dict[str, object],
    candidate_id: str,
    candidate_freeze: dict[str, object],
    trace_audit: dict[str, object],
) -> dict[str, object]:
    case = contract.canonical_development_case(case_id)

    def metric(sample_count: int) -> dict[str, object]:
        return {"abs_max": 0.0, "rms": 0.0, "sample_count": sample_count}

    actuator_metrics = {
        "motor_accel_rad_s2": metric(contract.EXPECTED_RAW_SENSOR_SAMPLES),
        "motor_power_w": metric(contract.EXPECTED_RAW_SENSOR_SAMPLES),
        "motor_speed_rad_s": metric(contract.EXPECTED_RAW_SENSOR_SAMPLES),
        "tau_spring_nm": metric(contract.EXPECTED_RAW_SENSOR_SAMPLES),
        "tau_spring_rate_nm_s": metric(
            contract.EXPECTED_RAW_SENSOR_SAMPLES - contract.EXPECTED_STEPS
        ),
        "torque_error_nm": metric(contract.EXPECTED_RAW_SENSOR_SAMPLES),
        "tau_input_saturated": {
            "count": 0,
            "fraction": 0.0,
            "sample_count": contract.EXPECTED_RAW_SENSOR_SAMPLES,
        },
    }
    event_prefix_checks = {
        "sample_count_positive",
        "sample_count_matches_raw_sensor",
        "raw_sensor_matches_control_windows",
        "ten_samples_per_control_step",
        "zero_duplicate_event_count",
        "zero_out_of_order_event_count",
        "zero_left_non_v26_source_count",
        "zero_fallback_count",
        "zero_hard_invalid_count",
    }
    summary = dict.fromkeys(runner._DEVELOPMENT_SUMMARY_FIELDS)
    summary.update(
        {
            "schema_version": contract.SCHEMA_VERSION,
            "status": contract.DEVELOPMENT_COMPLETE_STATUS,
            "protocol_id": contract.PROTOCOL_ID,
            "case_id": case_id,
            "case": case,
            "action_selection": case["action_selection"],
            "episode_start_offset_s": case["episode_start_offset_s"],
            "action_seed": case["action_seed"],
            "runtime_seed": case["runtime_seed"],
            "sigma": case["sigma"],
            "candidate_id": candidate_id,
            "candidate_module": module,
            "candidate_freeze": candidate_freeze,
            "teacher_enabled": False,
            "blending_enabled": False,
            "safety_latch_enabled": False,
            "pure_policy_trace_audit": trace_audit,
            "pure_policy_trace_row_count": contract.EXPECTED_STEPS,
            "steps": contract.EXPECTED_STEPS,
            "control_window_count": contract.EXPECTED_CONTROL_WINDOWS,
            "raw_sensor_sample_count": contract.EXPECTED_RAW_SENSOR_SAMPLES,
            "phase_valid_cycle_count": 1,
            "grf_penetration_max_m": 0.0,
            "end_reason": "episode_time_limit",
            "terminated": False,
            "truncated": True,
            "n_actor": contract.EXPECTED_ACTOR_FEATURES,
            "n_observation": contract.EXPECTED_FULL_FEATURES,
            "observation_dtype": contract.EXPECTED_DTYPE,
            "binary_phase_fsm_mode": "binary_active",
            "event_contract_id": contract.EVENT_CONTRACT_ID,
            "target_contract_id": contract.TARGET_CONTRACT_ID,
            "detector_or_fsm_modified": False,
            "morphology_weight": contract.MORPHOLOGY_WEIGHT,
            "random_noise_draw_count": (
                contract.EXPECTED_STEPS
                if case["action_selection"] == "stochastic"
                else 0
            ),
            "single_noise_application_count": contract.EXPECTED_STEPS,
            "protected_trials_opened": [],
            "reserve_trials_opened": [],
            "q2_paths_opened": [],
            "q3_paths_opened": [],
            "sea_reserve_gate_passed": True,
            "episode_metrics": {
                "reserve_norm_nm": metric(contract.EXPECTED_STEPS),
                "residual_norm_nm": metric(contract.EXPECTED_STEPS),
            },
            "sea_episode_metrics": {
                "pros_ankle_angle": copy.deepcopy(actuator_metrics),
                "pros_knee_angle": copy.deepcopy(actuator_metrics),
            },
            "binary_phase_event_gate": {
                "duplicate_event_count": 0,
                "event_count": 0,
                "events": [],
                "fallback_count": 0,
                "hard_invalid_count": 0,
                "left_non_v26_source_count": 0,
                "out_of_order_event_count": 0,
                "passed": True,
                "sample_count": contract.EXPECTED_RAW_SENSOR_SAMPLES,
            },
            "binary_event_prefix_integrity": {
                "passed": True,
                "sample_count": contract.EXPECTED_RAW_SENSOR_SAMPLES,
                "raw_sensor_sample_count": contract.EXPECTED_RAW_SENSOR_SAMPLES,
                "control_window_count": contract.EXPECTED_RAW_SENSOR_SAMPLES,
                "expected_sample_count": contract.EXPECTED_RAW_SENSOR_SAMPLES,
                "checks": {name: True for name in event_prefix_checks},
            },
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
            "binary_event_failure_count": 0,
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
            **{name: 0 for name in contract.PURE_POLICY_COUNTER_FIELDS},
        }
    )
    return summary


def test_stage_receipts_match_the_five_q3_prerequisites() -> None:
    assert runner.PROTOCOL_FREEZE_PATH == runner.resolve_relative(
        contract.PROTOCOL_FREEZE_PATH
    )
    assert runner.LOCK_PATH == runner.resolve_relative(contract.EXECUTION_LOCK_PATH)
    assert runner._stage_receipt_path("freeze_case_balanced_candidate") == (
        runner.resolve_relative(contract.CANDIDATE_FREEZE_PATH)
    )
    assert runner._stage_receipt_path("finalize_development") == (
        runner.resolve_relative(contract.FINAL_DEVELOPMENT_RECEIPT_PATH)
    )
    assert runner.PIPELINE_LEDGER_PATH == runner.resolve_relative(
        contract.PIPELINE_LEDGER_PATH
    )
    assert set(contract.Q3_PREREQUISITES) == {
        "protocol_freeze",
        "execution_lock",
        "candidate_freeze",
        "final_development_receipt",
        "terminal_ledger",
    }


def test_raw_journal_is_observer_only_and_tolerates_missing_clearance() -> None:
    journal = runner._diagnostic_raw_journal(
        {
            "binary_phase_sensor_samples": [
                {
                    "time_s": 1.0,
                    "left_heel_contact": True,
                    "left_toe_contact": False,
                }
            ],
            "binary_phase_fsm": {
                "events_this_step": [{"event": "heel_strike"}],
                "pending_event": None,
            },
        },
        step=1,
    )
    assert journal["observer_only"] is True
    assert journal["control_dependency"] is False
    assert journal["gate_dependency"] is False
    assert journal["blocker_if_field_unavailable"] is False
    assert journal["samples"][0]["left_heel_clearance_m"] is None
    assert journal["accepted_events"] == [{"event": "heel_strike"}]


def test_runner_has_no_collection_surface_or_qualification_execution() -> None:
    source = Path(runner.__file__).read_text(encoding="utf-8")
    tree = ast.parse(source)
    function_names = {
        node.name for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)
    }
    imported = {
        alias.name
        for node in ast.walk(tree)
        if isinstance(node, (ast.Import, ast.ImportFrom))
        for alias in node.names
    }
    assert "_run_collection" not in function_names
    assert "COLLECTION_CASE_IDS" not in source
    assert "collection_execution_authorized" not in source
    assert "safe_dagger" not in imported
    assert "coherent_teacher" not in imported
    assert contract.AUTHORITY["new_environment_collection_authorized"] is False
    assert contract.AUTHORITY["qualification_execution_authorized"] is False
    assert 'checkpoint_zero_created": True' not in source


def test_lock_binds_q3_design_candidate_rule_and_runtime_source_closure() -> None:
    source = Path(runner.__file__).read_text(encoding="utf-8")
    assert '"DEFERRED_UNTIL_CASE_BALANCED_FIT"' in source
    assert "contract.Q3_DESIGN_FREEZE_ARTIFACT" in source
    assert "contract.Q3_DESIGN_FREEZE_STATUS" in source
    assert "contract.FROZEN_EXTERNAL_RUNTIME_SOURCES" in source
    assert "contract.SAFE_V8R1P1_EXECUTION_LOCK_ARTIFACT" in source
    assert runner._external_runtime_sources_exact() is True
    snapshot = runner._locked_input_snapshot()
    assert snapshot["q3_design"] == contract.Q3_DESIGN_FREEZE_ARTIFACT
    assert snapshot["external_runtime_sources"] == (
        contract.FROZEN_EXTERNAL_RUNTIME_SOURCES
    )
    assert len(snapshot["safe_environment_source_closure"]) == 29
    assert len(snapshot["external_runtime_sources"]) == 64
    assert len(snapshot["production_source_closure"]) == 68
    assert runner.freezer._production_source_closure_exact(
        snapshot["production_source_closure"]
    )


def test_live_activity_counts_only_actual_stage_work() -> None:
    runner._reset_activity()
    stage_id = "development__deterministic_offset_plus_0p20"
    runner._begin_stage_activity(stage_id)
    runner._activity_increment("environment_reset_calls")
    runner._activity_increment("environment_step_calls", 2)
    runner._activity_increment("raw_sensor_sample_count", 20)
    assert runner._ACTIVITY_TOTALS["environment_reset_calls"] == 1
    assert runner._ACTIVITY_TOTALS["teacher_query_count"] == 0
    assert runner._ACTIVITY_TOTALS["environment_step_calls"] == 2
    assert runner._ACTIVITY_TOTALS["raw_sensor_sample_count"] == 20
    assert runner._activity_totals_snapshot() == runner._ACTIVITY_TOTALS
    row = runner._STAGE_ACTIVITY[stage_id]
    assert row["stage_kind"] == "development"
    assert row["teacher_query_count"] == 0
    source = Path(runner.__file__).read_text(encoding="utf-8")
    assert '"activity_totals": _activity_totals_snapshot()' in source
    assert '"environment_reset_calls": 6' in source
    assert '"environment_step_calls": 3000' in source
    assert '"raw_sensor_sample_count": 30_000' in source
    assert '"teacher_query_count": 0' in source


def test_raw_sensor_counter_precedes_consumer_and_pure_counters_are_explicit() -> None:
    source = Path(runner.__file__).read_text(encoding="utf-8")
    start = source.index("def _run_development")
    end = source.index("def _run_finalize_development")
    development = source[start:end]
    assert development.index(
        'raw_samples = info.get("binary_phase_sensor_samples")'
    ) < (development.index("runtime._consume_physical_step"))
    assert "len(raw_samples) != contract.EXPECTED_RAW_SENSOR_SAMPLES_PER_STEP" in (
        development
    )
    for name in contract.PURE_POLICY_COUNTER_FIELDS:
        assert f'"{name}": 0' in development or (
            "**{name: 0 for name in contract.PURE_POLICY_COUNTER_FIELDS}" in development
        )


def test_error_record_has_a_nonempty_fallback_and_terminal_rechecks_lock() -> None:
    assert runner._error_record(RuntimeError()) == {
        "type": "RuntimeError",
        "message": "RuntimeError()",
    }
    source = Path(runner.__file__).read_text(encoding="utf-8")
    terminal = source[source.index("def _verify_terminal_ledger_payload") :]
    development = source[
        source.index("def _run_development") : source.index(
            "def _run_finalize_development"
        )
    ]
    assert "writer.publish_failure" not in development
    assert "def _ensure_development_failure_artifact" in source
    assert '"attempted_stage_receipt": attempted_receipt' in source
    assert '"attempted_stage_receipt_created": attempted_receipt is not None' in source
    assert "terminal ledger attempted output binding drifted" in terminal
    assert terminal.index(
        "verify_execution_lock(require_run_root_absent=False)"
    ) < terminal.index("ledger = dict(ledger)")


def test_development_failure_artifact_allows_missing_worker_claim(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    destination = tmp_path / "development"
    missing_claim = tmp_path / "missing-worker-claim.json"
    pipeline_claim = tmp_path / "pipeline-claim.json"
    pipeline_claim.write_text("{}", encoding="utf-8")
    monkeypatch.setattr(runner, "resolve_relative", lambda _path: destination)
    monkeypatch.setattr(runner, "_claim_path", lambda _stage_id: missing_claim)
    monkeypatch.setattr(runner, "PIPELINE_CLAIM_PATH", pipeline_claim)
    monkeypatch.setattr(
        runner,
        "_record",
        lambda path: {
            "path": Path(path).as_posix(),
            "sha256": runner.hashlib.sha256(
                Path(path).read_bytes() if Path(path).is_file() else b""
            ).hexdigest(),
            "size_bytes": (len(Path(path).read_bytes()) if Path(path).is_file() else 0),
        },
    )
    monkeypatch.setattr(runner, "_pipeline_claim_payload_valid", lambda _value: True)
    monkeypatch.setattr(runner, "_qualification_unopened", lambda: True)
    monkeypatch.setattr(
        runner,
        "_opened_qualification_paths",
        lambda: {"q2": [], "q3": []},
    )
    stage_id = "development__deterministic_offset_plus_0p20"
    record = runner._ensure_development_failure_artifact(stage_id, RuntimeError())
    payload = runner.forensic.strict_json_load(destination / "failure.json")
    assert record["path"] == (destination / "failure.json").as_posix()
    assert record["size_bytes"] > 0
    assert payload["worker_claim_observation"] == {
        "state": "ABSENT",
        "artifact": None,
    }
    assert payload["error"] == {
        "type": "RuntimeError",
        "message": "RuntimeError()",
    }


def test_failure_artifact_write_error_is_captured_for_terminal_ledger(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def fail_publication(_stage_id: str, _error: BaseException) -> None:
        raise RuntimeError()

    monkeypatch.setattr(
        runner, "_ensure_development_failure_artifact", fail_publication
    )
    assert runner._attempt_development_failure_artifact(
        "development__deterministic_offset_plus_0p20", ValueError("stage")
    ) == {"type": "RuntimeError", "message": "RuntimeError()"}


@pytest.mark.parametrize("leave_partial_claim", [False, True])
def test_claim_write_failure_publishes_bound_no_clobber_evidence(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    leave_partial_claim: bool,
) -> None:
    claim_path, failure_path, worker_root = _configure_claim_fault_paths(
        tmp_path, monkeypatch
    )
    original_write = runner.forensic.write_json_exclusive

    def injected_write(path: str | Path, payload: object) -> Path:
        destination = Path(path)
        if destination == claim_path:
            if leave_partial_claim:
                destination.touch(exist_ok=False)
            raise RuntimeError()
        return original_write(destination, payload)

    monkeypatch.setattr(runner.forensic, "write_json_exclusive", injected_write)
    with pytest.raises(RuntimeError):
        runner._claim_run_root()
    evidence = runner.forensic.strict_json_load(failure_path)
    expected_state = "PARTIAL_OR_INVALID_REGULAR" if leave_partial_claim else "ABSENT"
    assert evidence["pipeline_claim_observation"]["state"] == expected_state
    if leave_partial_claim:
        assert evidence["pipeline_claim_observation"]["artifact"]["size_bytes"] == 0
    else:
        assert evidence["pipeline_claim_observation"]["artifact"] is None
    assert evidence["worker_claims_root_state"] == "ABSENT"
    assert not worker_root.exists()
    assert runner.verify_pipeline_claim_failure() == evidence
    rogue = failure_path.parent / "rogue.bin"
    rogue.write_bytes(b"rogue")
    with pytest.raises(runner.V12R5ExecutionError, match="run_root_closed"):
        runner.verify_pipeline_claim_failure()
    rogue.unlink()
    evidence["error"]["extra"] = "not allowed"
    failure_path.write_bytes(runner.forensic.canonical_json_bytes(evidence))
    with pytest.raises(runner.V12R5ExecutionError, match="drifted"):
        runner.verify_pipeline_claim_failure()


def test_worker_root_failure_preserves_valid_claim_and_failure_evidence(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    claim_path, failure_path, worker_root = _configure_claim_fault_paths(
        tmp_path, monkeypatch
    )

    def fail_worker_root() -> None:
        raise RuntimeError()

    monkeypatch.setattr(runner, "_create_worker_claims_root", fail_worker_root)
    with pytest.raises(RuntimeError):
        runner._claim_run_root()
    evidence = runner.forensic.strict_json_load(failure_path)
    assert claim_path.is_file()
    assert evidence["pipeline_claim_observation"]["state"] == "VALID_REGULAR"
    assert evidence["pipeline_claim_observation"]["artifact"]["size_bytes"] > 0
    assert evidence["worker_claims_root_state"] == "ABSENT"
    assert not worker_root.exists()
    assert runner.verify_pipeline_claim_failure() == evidence


def test_qualification_outputs_are_closed_but_q3_design_is_present() -> None:
    assert runner._qualification_unopened() is True
    assert runner._opened_qualification_paths() == {"q2": [], "q3": []}
    assert runner.resolve_relative(contract.Q3_DESIGN_FREEZE_PATH).is_file()


def test_frozen_protocol_gate_survives_lock_and_downstream_occupancy(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    freezer = runner.freezer
    protocol_path = tmp_path / "protocol_freeze.json"
    audit_path = tmp_path / "design_audit.json"
    protocol_failure_path = tmp_path / "protocol_publication_failure.json"
    lock_path = tmp_path / "execution_lock.json"
    lock_failure_path = tmp_path / "execution_lock_publication_failure.json"
    run_root = tmp_path / "run"
    claim_path = run_root / "pipeline_claim.json"
    claim_failure_path = run_root / "pipeline_claim_failure.json"
    ledger_path = run_root / "pipeline_ledger.json"
    terminal_failure_path = run_root / "terminal_publication_failure.json"
    worker_root = run_root / "worker_claims"
    q3_paths = {name: tmp_path / "q3" / name for name in contract.Q3_UNOPENED_PATHS}

    monkeypatch.setattr(freezer, "PROTOCOL_FREEZE_FILE", protocol_path)
    monkeypatch.setattr(freezer, "DESIGN_AUDIT_FILE", audit_path)
    monkeypatch.setattr(
        freezer,
        "PROTOCOL_PUBLICATION_FAILURE_FILE",
        protocol_failure_path,
    )
    original_freezer_resolve = freezer.resolve_relative
    downstream_paths = {
        contract.EXECUTION_LOCK_PATH.as_posix(): lock_path,
        contract.EXECUTION_LOCK_PUBLICATION_FAILURE_PATH.as_posix(): lock_failure_path,
        contract.RUN_ROOT.as_posix(): run_root,
        contract.PIPELINE_CLAIM_FAILURE_PATH.as_posix(): claim_failure_path,
        **{
            path.as_posix(): q3_paths[name]
            for name, path in contract.Q3_UNOPENED_PATHS.items()
        },
    }
    monkeypatch.setattr(
        freezer,
        "resolve_relative",
        lambda path: downstream_paths.get(str(path), original_freezer_resolve(path)),
    )

    monkeypatch.setattr(runner, "PROTOCOL_FREEZE_PATH", protocol_path)
    monkeypatch.setattr(runner, "DESIGN_AUDIT_PATH", audit_path)
    monkeypatch.setattr(runner, "LOCK_PATH", lock_path)
    monkeypatch.setattr(runner, "LOCK_PUBLICATION_FAILURE_PATH", lock_failure_path)
    monkeypatch.setattr(runner, "RUN_ROOT", run_root)
    monkeypatch.setattr(runner, "PIPELINE_CLAIM_PATH", claim_path)
    monkeypatch.setattr(runner, "PIPELINE_CLAIM_FAILURE_PATH", claim_failure_path)
    monkeypatch.setattr(runner, "PIPELINE_LEDGER_PATH", ledger_path)
    monkeypatch.setattr(
        runner,
        "PIPELINE_TERMINAL_PUBLICATION_FAILURE_PATH",
        terminal_failure_path,
    )
    monkeypatch.setattr(runner, "WORKER_CLAIMS_ROOT", worker_root)
    original_runner_resolve = runner.resolve_relative
    monkeypatch.setattr(
        runner,
        "resolve_relative",
        lambda path: downstream_paths.get(str(path), original_runner_resolve(path)),
    )
    original_runner_record = runner._record
    logical_paths = {
        protocol_path.resolve(): contract.PROTOCOL_FREEZE_PATH.as_posix(),
        audit_path.resolve(): contract.DESIGN_AUDIT_PATH.as_posix(),
        lock_path.resolve(): contract.EXECUTION_LOCK_PATH.as_posix(),
    }

    def redirected_record(path: str | Path) -> dict[str, object]:
        source = Path(path).resolve()
        logical = logical_paths.get(source)
        if logical is None:
            return original_runner_record(path)
        data = source.read_bytes()
        return {
            "path": logical,
            "sha256": runner.hashlib.sha256(data).hexdigest(),
            "size_bytes": len(data),
        }

    monkeypatch.setattr(runner, "_record", redirected_record)

    assert freezer.publish_protocol_freeze() == protocol_path
    assert freezer.verify_protocol_freeze()["passed"] is True
    lock_payload = runner.prepare_execution_lock()
    assert lock_payload["passed"] is True
    assert runner.verify_execution_lock(require_run_root_absent=True)["passed"] is True
    for mutation in (
        lambda value: value.update(schema_version=float(contract.SCHEMA_VERSION)),
        lambda value: value["authority"].update(source_implementation_authorized=1),
        lambda value: value["protocol_freeze"].update(
            size_bytes=float(value["protocol_freeze"]["size_bytes"])
        ),
        lambda value: value["occupancy"].update(lock_absent=1),
        lambda value: value.update(environment_reset_calls=False),
    ):
        tampered = copy.deepcopy(lock_payload)
        mutation(tampered)
        assert runner._execution_lock_payload_valid(tampered) is False
    assert freezer.source_precondition_gate()["passed"] is False
    assert freezer.verify_protocol_freeze()["passed"] is True

    run_root.mkdir()
    claim_path.write_text("{}", encoding="utf-8")
    ledger_path.write_text("{}", encoding="utf-8")
    assert freezer.source_precondition_gate()["passed"] is False
    assert freezer.verify_protocol_freeze()["passed"] is True
    assert runner.verify_execution_lock(require_run_root_absent=False)["passed"] is True
    with pytest.raises(runner.V12R5ExecutionError, match="already claimed"):
        runner.verify_execution_lock(require_run_root_absent=True)

    for name, path in q3_paths.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        if name in {"run_root", "noise_root"}:
            path.mkdir()
        else:
            path.write_text("{}", encoding="utf-8")
    assert freezer.source_precondition_gate()["passed"] is False
    assert freezer.verify_protocol_freeze()["passed"] is True
    assert runner.verify_execution_lock(require_run_root_absent=False)["passed"] is True


def test_execution_lock_write_failure_is_bound_by_dominant_receipt(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    lock_path, failure_path, expected = _configure_lock_publication_fault_paths(
        tmp_path, monkeypatch
    )
    original_write = runner.freezer._write_json_exclusive

    def injected_write(path: Path, payload: object) -> None:
        if Path(path) == lock_path:
            raise OSError("injected execution-lock link failure")
        original_write(Path(path), payload)

    monkeypatch.setattr(runner.freezer, "_write_json_exclusive", injected_write)
    with pytest.raises(OSError, match="injected execution-lock link failure"):
        runner.prepare_execution_lock()
    assert not lock_path.exists()
    evidence = runner.forensic.strict_json_load(failure_path)
    assert evidence["expected_execution_lock"] == expected
    assert evidence["execution_lock_observation"] == {
        "state": "ABSENT",
        "artifact": None,
    }
    assert runner.verify_execution_lock_publication_failure() == evidence
    with pytest.raises(runner.V12R5ExecutionError, match="dominant"):
        runner.verify_execution_lock()


def test_execution_lock_postverify_failure_is_terminal_even_with_valid_lock(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    lock_path, failure_path, _expected = _configure_lock_publication_fault_paths(
        tmp_path, monkeypatch
    )

    def injected_postverify(**_kwargs: object) -> dict[str, object]:
        raise RuntimeError("injected execution-lock postverify failure")

    with monkeypatch.context() as fault:
        fault.setattr(runner, "verify_execution_lock", injected_postverify)
        with pytest.raises(
            RuntimeError, match="injected execution-lock postverify failure"
        ):
            runner.prepare_execution_lock()
    evidence = runner.forensic.strict_json_load(failure_path)
    assert lock_path.is_file() and lock_path.stat().st_size > 0
    assert evidence["execution_lock_observation"]["state"] == "VALID"
    assert runner.verify_execution_lock_publication_failure() == evidence
    with pytest.raises(runner.V12R5ExecutionError, match="dominant"):
        runner.verify_execution_lock()
    evidence["error"]["extra"] = "not allowed"
    failure_path.write_bytes(runner.forensic.canonical_json_bytes(evidence))
    with pytest.raises(runner.V12R5ExecutionError, match="evidence drifted"):
        runner.verify_execution_lock_publication_failure()


def test_terminal_failure_structure_is_symmetric_and_tamper_closed() -> None:
    completed_ids = list(contract.STAGE_IDS[:4])
    attempted = contract.STAGE_IDS[4]
    activity = [
        {
            "stage_id": stage_id,
            "stage_kind": contract.stage_descriptor(stage_id)["kind"],
            **{name: 0 for name in runner._ACTIVITY_NAMES},
        }
        for stage_id in contract.STAGE_IDS[:5]
    ]
    ledger = {
        "status": contract.TERMINAL_FAIL_STATUS,
        "passed": False,
        "terminal": True,
        "completed_stages": [
            {"stage_id": stage_id, "receipt": {}} for stage_id in completed_ids
        ],
        "attempted_stage": attempted,
        "stage_activity": activity,
        "error": {"type": "RuntimeError", "message": "boom"},
        "next_stage": "STOP_TERMINAL",
        "q2_paths_opened": [],
        "q3_paths_opened": [],
        "qualification_violation_detected": False,
        "new_collection_count": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "qualification_executed": False,
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
        "attempted_stage_failure_observation": {
            "state": "VALID",
            "artifact": {"path": "failure.json"},
        },
        "attempted_stage_failure_artifact": {"path": "failure.json"},
        "attempted_stage_failure_publication_error": None,
    }
    assert runner._terminal_failure_structure_gate(ledger)["passed"] is True
    publication_failed = copy.deepcopy(ledger)
    publication_failed["attempted_stage_failure_observation"] = {
        "state": "ABSENT",
        "artifact": None,
    }
    publication_failed["attempted_stage_failure_artifact"] = None
    publication_failed["attempted_stage_failure_publication_error"] = {
        "type": "OSError",
        "message": "write failed",
    }
    assert runner._terminal_failure_structure_gate(publication_failed)["passed"] is True
    for mutation in (
        lambda value: value.update(attempted_stage=contract.STAGE_IDS[5]),
        lambda value: value.update(error=None),
        lambda value: value["error"].update(extra="not allowed"),
        lambda value: value.update(new_collection_count=False),
        lambda value: value.update(next_stage="WAIT_SEPARATE_Q3_PROTOCOL"),
        lambda value: value.update(q3_paths_opened=["run_root"]),
        lambda value: value.update(retry_authorized=True),
        lambda value: value.update(attempted_stage_failure_artifact=None),
        lambda value: value.update(
            attempted_stage_failure_artifact=None,
            attempted_stage_failure_publication_error={
                "type": "OSError",
                "message": "",
            },
        ),
    ):
        tampered = copy.deepcopy(ledger)
        mutation(tampered)
        assert runner._terminal_failure_structure_gate(tampered)["passed"] is False


def test_terminal_ledger_schema_rejects_missing_and_extra_fields() -> None:
    exact = dict.fromkeys(runner._TERMINAL_LEDGER_FIELDS)
    assert runner._terminal_ledger_schema_exact(exact) is True
    missing = copy.deepcopy(exact)
    missing.pop(next(iter(missing)))
    assert runner._terminal_ledger_schema_exact(missing) is False
    extra = copy.deepcopy(exact)
    extra["unregistered_field"] = None
    assert runner._terminal_ledger_schema_exact(extra) is False


def test_terminal_intent_rejects_forged_or_misbound_completed_receipt(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _configure_terminal_fault_paths(tmp_path, monkeypatch)
    intent = runner._terminal_intent_from_ledger(_all_stages_intended_pass())
    assert runner._terminal_intent_valid(intent) is True

    forged = copy.deepcopy(intent)
    forged["completed_stages"][0]["receipt"] = "FORGED_NOT_AN_ARTIFACT_RECORD"
    assert runner._terminal_intent_valid(forged) is False

    wrong_path = copy.deepcopy(intent)
    wrong_path["completed_stages"][0]["receipt"]["path"] = "wrong/receipt.json"
    assert runner._terminal_intent_valid(wrong_path) is False

    coerced_size = copy.deepcopy(intent)
    coerced_size["completed_stages"][0]["receipt"]["size_bytes"] = False
    assert runner._terminal_intent_valid(coerced_size) is False

    forged_activity = copy.deepcopy(intent)
    forged_activity["stage_activity"] = [{"totally": "invalid"}]
    assert runner._terminal_intent_valid(forged_activity) is False

    drifted_totals = copy.deepcopy(intent)
    drifted_totals["activity_totals"]["environment_step_calls"] += 1
    assert runner._terminal_intent_valid(drifted_totals) is False

    forged_lbfgs = copy.deepcopy(intent)
    fit_activity = next(
        row
        for row in forged_lbfgs["stage_activity"]
        if row["stage_id"] == "fit_case_balanced_candidate"
    )
    fit_activity["lbfgs_closure_calls"] = 999
    forged_lbfgs["activity_totals"]["lbfgs_closure_calls"] = 999
    assert runner._terminal_intent_valid(forged_lbfgs) is False

    rewritten_as_empty_fail = copy.deepcopy(intent)
    rewritten_as_empty_fail.update(
        {
            "intended_outcome": "FAIL",
            "completed_stages": [],
            "completed_stage_count": 0,
            "attempted_stage": contract.STAGE_IDS[0],
            "stage_error": {"type": "RuntimeError", "message": "forged"},
            "activity_totals": {name: 0 for name in runner._ACTIVITY_NAMES},
            "stage_activity": [
                {
                    "stage_id": contract.STAGE_IDS[0],
                    "stage_kind": contract.stage_descriptor(contract.STAGE_IDS[0])[
                        "kind"
                    ],
                    **{name: 0 for name in runner._ACTIVITY_NAMES},
                }
            ],
            "next_stage": "STOP_TERMINAL",
        }
    )
    assert runner._terminal_intent_valid(rewritten_as_empty_fail) is False


def test_terminal_intent_gate_failure_still_publishes_terminal_evidence(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    ledger_path, failure_path = _configure_terminal_fault_paths(tmp_path, monkeypatch)
    for stage_id in contract.STAGE_IDS:
        runner._stage_receipt_path(stage_id).unlink()
    runner._reset_activity()
    attempted = contract.STAGE_IDS[0]
    runner._begin_stage_activity(attempted)
    monkeypatch.setattr(
        runner, "_completed_stage_prefix_current", lambda *_a, **_k: False
    )

    with pytest.raises(runner.V12R5ExecutionError, match="in-memory gate"):
        runner._terminalize(
            passed=False,
            attempted_stage=attempted,
            completed=[],
            error=RuntimeError("original stage failure"),
            failure_publication_error=None,
        )

    assert not ledger_path.exists()
    evidence = runner.forensic.strict_json_load(failure_path)
    assert evidence["terminal_intent_valid"] is False
    assert evidence["intended_ledger"] is None
    assert evidence["intended_ledger_record"] is None
    assert evidence["terminal_intent"]["stage_error"] == {
        "type": "RuntimeError",
        "message": "original stage failure",
    }
    assert runner.verify_terminal_publication_failure() == evidence


def test_unreadable_pipeline_claim_is_bound_by_emergency_terminal_evidence(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    ledger_path, failure_path = _configure_terminal_fault_paths(tmp_path, monkeypatch)
    for stage_id in contract.STAGE_IDS:
        runner._stage_receipt_path(stage_id).unlink()
    runner._reset_activity()
    attempted = contract.STAGE_IDS[0]
    runner._begin_stage_activity(attempted)
    original_record = runner._record

    def unreadable_claim(path: str | Path) -> dict[str, object]:
        if Path(path) == runner.PIPELINE_CLAIM_PATH:
            raise PermissionError("injected unreadable regular claim")
        return original_record(path)

    monkeypatch.setattr(runner, "_record", unreadable_claim)
    with pytest.raises(runner.V12R5ExecutionError, match="in-memory gate"):
        runner._terminalize(
            passed=False,
            attempted_stage=attempted,
            completed=[],
            error=PermissionError("claim became unreadable"),
            failure_publication_error=None,
        )

    assert not ledger_path.exists()
    evidence = runner.forensic.strict_json_load(failure_path)
    assert evidence["terminal_intent_valid"] is False
    assert evidence["pipeline_claim"] is None
    assert evidence["pipeline_claim_observation"] == {
        "state": "UNSAFE_UNREADABLE_REGULAR",
        "artifact": None,
    }
    assert runner.verify_terminal_publication_failure() == evidence


def test_attempted_stage_outputs_require_a_valid_worker_claim(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    claim_path = tmp_path / "worker_claim.json"
    claim_path.write_text("{}", encoding="utf-8")
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(runner, "_claim_path", lambda _stage_id: claim_path)
    monkeypatch.setattr(
        runner,
        "_stage_output_observations",
        lambda _stage_id: {
            "receipt": {
                "state": "INVALID_JSON_REGULAR",
                "artifact": runner._record(claim_path),
            }
        },
    )
    monkeypatch.setattr(
        runner,
        "_worker_claim_observation",
        lambda _stage_id: {"state": "ABSENT", "artifact": None},
    )
    assert not runner._attempted_stage_ownership_coherent(contract.STAGE_IDS[0])

    monkeypatch.setattr(
        runner,
        "_worker_claim_observation",
        lambda _stage_id: {
            "state": "VALID_REGULAR",
            "artifact": runner._record(claim_path),
        },
    )
    assert runner._attempted_stage_ownership_coherent(contract.STAGE_IDS[0])


def test_attempted_activity_accepts_interruptible_prefixes_only() -> None:
    fit_id = "fit_case_balanced_candidate"
    fit = {
        "stage_id": fit_id,
        "stage_kind": "fit",
        **{name: 0 for name in runner._ACTIVITY_NAMES},
    }
    fit["actor_fit_stage_calls_attempted"] = 1
    assert runner._attempted_stage_activity_valid(fit, stage_id=fit_id)
    impossible = copy.deepcopy(fit)
    impossible["actor_fit_stage_calls_attempted"] = 0
    impossible["actor_updates_attempted"] = 1
    assert not runner._attempted_stage_activity_valid(impossible, stage_id=fit_id)
    impossible = copy.deepcopy(fit)
    impossible["actor_fit_executions_confirmed"] = 1
    assert not runner._attempted_stage_activity_valid(impossible, stage_id=fit_id)

    development_id = "development__deterministic_offset_plus_0p20"
    development = {
        "stage_id": development_id,
        "stage_kind": "development",
        **{name: 0 for name in runner._ACTIVITY_NAMES},
    }
    development.update(
        environment_reset_calls=1,
        environment_step_calls=2,
        raw_sensor_sample_count=10,
    )
    assert runner._attempted_stage_activity_valid(development, stage_id=development_id)
    development["environment_reset_calls"] = 0
    assert not runner._attempted_stage_activity_valid(
        development, stage_id=development_id
    )


def test_regular_observers_classify_record_read_failures_without_throwing(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    artifact = tmp_path / "artifact.json"
    artifact.write_text("{}", encoding="utf-8")
    monkeypatch.setattr(runner, "PIPELINE_CLAIM_PATH", artifact)
    monkeypatch.setattr(runner, "_claim_path", lambda _stage_id: artifact)
    monkeypatch.setattr(
        runner,
        "_record",
        lambda _path: (_ for _ in ()).throw(PermissionError("unreadable")),
    )
    expected = {"path": "artifact.json", "sha256": "0" * 64, "size_bytes": 2}
    assert runner._expected_json_observation(artifact, expected_record=expected) == {
        "state": "UNSAFE_UNREADABLE_REGULAR",
        "artifact": None,
    }
    assert runner._pipeline_claim_observation() == {
        "state": "UNSAFE_UNREADABLE_REGULAR",
        "artifact": None,
    }
    assert runner._worker_claim_observation(contract.STAGE_IDS[0]) == {
        "state": "UNSAFE_UNREADABLE_REGULAR",
        "artifact": None,
    }


def test_execution_lock_schema_rejects_nested_missing_extra_and_wrong_types() -> None:
    exact = dict.fromkeys(runner._EXECUTION_LOCK_FIELDS)
    exact["checks"] = dict.fromkeys(runner._EXECUTION_LOCK_CHECK_FIELDS, True)
    exact["occupancy"] = dict.fromkeys(runner._EXECUTION_LOCK_OCCUPANCY_FIELDS, True)
    exact["platform"] = dict.fromkeys(runner._PLATFORM_FIELDS, "value")
    assert runner._execution_lock_schema_exact(exact) is True
    for key in ("checks", "occupancy", "platform"):
        missing = copy.deepcopy(exact)
        missing[key].pop(next(iter(missing[key])))
        assert runner._execution_lock_schema_exact(missing) is False
        extra = copy.deepcopy(exact)
        extra[key]["unregistered_field"] = None
        assert runner._execution_lock_schema_exact(extra) is False
        wrong_type = copy.deepcopy(exact)
        wrong_type[key] = []
        assert runner._execution_lock_schema_exact(wrong_type) is False


def test_pipeline_and_worker_claim_semantics_are_exact_and_root_is_closed(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _claim_path, _failure_path, worker_root = _configure_claim_fault_paths(
        tmp_path, monkeypatch
    )
    token_hash, claim = runner._claim_run_root()
    assert runner._pipeline_claim_payload_valid(claim) is True
    for mutation in (
        lambda value: value.pop("design_audit"),
        lambda value: value.update(unregistered_field=None),
        lambda value: value.update(execution_token_sha256="A" * 64),
        lambda value: value.update(one_shot=1),
        lambda value: value["execution_lock"].update(
            size_bytes=float(value["execution_lock"]["size_bytes"])
        ),
    ):
        tampered = copy.deepcopy(claim)
        mutation(tampered)
        assert runner._pipeline_claim_payload_valid(tampered) is False

    stage_id = contract.STAGE_IDS[0]
    worker = runner._write_worker_claim(stage_id, token_hash)
    assert runner._worker_claim_payload_valid(worker, stage_id=stage_id) is True
    monkeypatch.setattr(
        runner,
        "_historical_locked_input_attestation",
        lambda: {"semantics": {"closed": True}},
    )
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.SOURCE_ATTEST_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "locked_input_attestation": {"semantics": {"closed": True}},
        "failed_plus_prefix_rows_loaded": 0,
        "new_collection_count": 0,
        "pipeline_claim": runner._record(runner.PIPELINE_CLAIM_PATH),
        "worker_claim": runner._record(runner._claim_path(stage_id)),
        "q2_paths_opened": [],
        "q3_paths_opened": [],
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    runner.forensic.write_json_exclusive(runner._stage_receipt_path(stage_id), receipt)
    assert runner._verify_stage_receipt_claim_bindings(stage_id, receipt) == receipt
    for mutation in (
        lambda value: value.pop("status"),
        lambda value: value.update(unregistered_field=None),
        lambda value: value.update(status="PASS_BUT_WRONG_STATUS"),
        lambda value: value.update(stage_id=contract.STAGE_IDS[1]),
        lambda value: value.update(worker_claim=None),
    ):
        tampered_receipt = copy.deepcopy(receipt)
        mutation(tampered_receipt)
        with pytest.raises(runner.V12R5ExecutionError, match="receipt claim binding"):
            runner._verify_stage_receipt_claim_bindings(stage_id, tampered_receipt)
    runner._verify_worker_claims_root_exact(
        completed_stage_ids=[], attempted_stage=stage_id
    )
    for mutation in (
        lambda value: value.update(execution_token_sha256="0" * 64),
        lambda value: value.update(stage_index=1),
        lambda value: value.update(stage_index=False),
        lambda value: value.update(stage_kind="development"),
        lambda value: value.update(previous_receipts=[{}]),
        lambda value: value.update(unregistered_field=None),
    ):
        tampered = copy.deepcopy(worker)
        mutation(tampered)
        assert runner._worker_claim_payload_valid(tampered, stage_id=stage_id) is False
    (worker_root / "extra.json").write_text("{}", encoding="utf-8")
    with pytest.raises(runner.V12R5ExecutionError, match="root closure drifted"):
        runner._verify_worker_claims_root_exact(
            completed_stage_ids=[], attempted_stage=stage_id
        )


def test_candidate_freeze_receipt_and_summary_are_deep_strict(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    run_root = tmp_path / "run"
    fit_root = tmp_path / "fit"
    module_path = tmp_path / "candidate_module"
    worker_root = run_root / "worker_claims"
    for path in (run_root, fit_root, module_path, worker_root):
        path.mkdir(parents=True, exist_ok=True)
    claim_path = run_root / "pipeline_claim.json"
    worker_path = worker_root / "freeze_case_balanced_candidate.json"
    fit_receipt_path = fit_root / "receipt.json"
    for path in (claim_path, worker_path, fit_receipt_path):
        path.write_text("{}", encoding="utf-8")

    def fake_record(path: str | Path) -> dict[str, object]:
        source = Path(path)
        data = source.read_bytes()
        return {
            "path": source.as_posix(),
            "sha256": runner.hashlib.sha256(data).hexdigest(),
            "size_bytes": len(data),
        }

    resolution = {
        contract.FIT_ROOT.as_posix(): fit_root,
        contract.CANDIDATE_MODULE_PATH.as_posix(): module_path,
    }
    monkeypatch.setattr(runner, "resolve_relative", lambda path: resolution[str(path)])
    monkeypatch.setattr(runner, "RUN_ROOT", run_root)
    monkeypatch.setattr(runner, "WORKER_CLAIMS_ROOT", worker_root)
    monkeypatch.setattr(runner, "PIPELINE_CLAIM_PATH", claim_path)
    monkeypatch.setattr(runner, "_record", fake_record)
    monkeypatch.setattr(
        runner, "_worker_claim_payload_valid", lambda *_args, **_kwargs: True
    )
    module = {
        "tree_sha256": "a" * 64,
        "file_count": 1,
        "total_size_bytes": 1,
    }
    monkeypatch.setattr(runner, "_tree", lambda _path: module)
    identity = contract.candidate_id(module["tree_sha256"])
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.CANDIDATE_FREEZE_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_id": identity,
        "candidate_module": module,
        "fit_receipt": fake_record(fit_receipt_path),
        "fit_passed": True,
        "candidate_frozen": True,
        "source_h0_byte_exact": True,
        "logstd_byte_exact": True,
        "critic_present": False,
        "save_reload_exact": True,
        "pipeline_claim": fake_record(claim_path),
        "worker_claim": fake_record(worker_path),
        "q2_paths_opened": [],
        "q3_paths_opened": [],
        "runtime_promoted": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    summary_path = run_root / "candidate_freeze_summary.json"
    gate_path = run_root / "candidate_freeze_gate.json"
    summary_path.write_bytes(runner.forensic.canonical_json_bytes(summary))
    gate = contract.candidate_freeze_gate(summary)
    gate_path.write_bytes(runner.forensic.canonical_json_bytes(gate))
    receipt = {
        **copy.deepcopy(summary),
        "status": contract.CANDIDATE_FREEZE_STATUS,
        "passed": True,
        "stage_id": "freeze_case_balanced_candidate",
        "summary": fake_record(summary_path),
        "gate": fake_record(gate_path),
    }
    assert runner._candidate_freeze_payload_valid(receipt) is True
    for key, replacement in (
        ("fit_passed", False),
        ("candidate_frozen", False),
        ("source_h0_byte_exact", False),
        ("logstd_byte_exact", False),
        ("save_reload_exact", False),
        ("critic_present", True),
        ("fit_receipt", None),
    ):
        tampered = copy.deepcopy(receipt)
        tampered[key] = replacement
        assert runner._candidate_freeze_payload_valid(tampered) is False
    for mutation in (
        lambda value: value.update(schema_version=float(contract.SCHEMA_VERSION)),
        lambda value: value.update(actor_updates=False),
        lambda value: value.update(critic_present=0),
        lambda value: value.update(unregistered_field=None),
    ):
        tampered_summary = copy.deepcopy(summary)
        mutation(tampered_summary)
        assert (
            runner._candidate_freeze_summary_payload_valid(
                tampered_summary,
                module=module,
                identity=identity,
                stage_id="freeze_case_balanced_candidate",
            )
            is False
        )


def test_attestation_and_corpus_receipts_bind_exact_historical_evidence(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    run_root = tmp_path / "run"
    worker_root = run_root / "worker_claims"
    worker_root.mkdir(parents=True)
    claim_path = run_root / "pipeline_claim.json"
    claim_path.write_text("{}", encoding="utf-8")

    def fake_record(path: str | Path) -> dict[str, object]:
        source = Path(path)
        data = source.read_bytes()
        return {
            "path": source.as_posix(),
            "sha256": runner.hashlib.sha256(data).hexdigest(),
            "size_bytes": len(data),
        }

    monkeypatch.setattr(runner, "RUN_ROOT", run_root)
    monkeypatch.setattr(runner, "WORKER_CLAIMS_ROOT", worker_root)
    monkeypatch.setattr(runner, "PIPELINE_CLAIM_PATH", claim_path)
    monkeypatch.setattr(runner, "_record", fake_record)
    monkeypatch.setattr(
        runner, "_worker_claim_payload_valid", lambda *_args, **_kwargs: True
    )
    attestation = {"semantics": {"locked": True}, "records": [{"path": "a"}]}
    monkeypatch.setattr(
        runner, "_historical_locked_input_attestation", lambda: attestation
    )
    attestation_stage = contract.STAGE_IDS[0]
    attestation_worker = worker_root / f"{attestation_stage}.json"
    attestation_worker.write_text("{}", encoding="utf-8")
    attestation_receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.SOURCE_ATTEST_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": attestation_stage,
        "locked_input_attestation": copy.deepcopy(attestation),
        "failed_plus_prefix_rows_loaded": 0,
        "new_collection_count": 0,
        "pipeline_claim": fake_record(claim_path),
        "worker_claim": fake_record(attestation_worker),
        "q2_paths_opened": [],
        "q3_paths_opened": [],
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    assert (
        runner._stage_receipt_payload_valid(
            attestation_receipt, stage_id=attestation_stage
        )
        is True
    )
    tampered_attestation = copy.deepcopy(attestation_receipt)
    tampered_attestation["locked_input_attestation"] = {"arbitrary": True}
    assert (
        runner._stage_receipt_payload_valid(
            tampered_attestation, stage_id=attestation_stage
        )
        is False
    )

    corpus_stage = contract.STAGE_IDS[1]
    corpus_worker = worker_root / f"{corpus_stage}.json"
    corpus_worker.write_text("{}", encoding="utf-8")

    class Corpus:
        audit = {"canonical_row_count": 9232, "source_count": 2}
        source_records = [{"path": "p2"}, {"path": "r4"}]

    monkeypatch.setattr(runner, "_historical_case_balanced_corpus", Corpus)
    corpus_receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "PASS_H0_V12R5_CASE_BALANCED_CORPUS_ASSEMBLY",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": corpus_stage,
        "fit_counts": contract.expected_corpus_counts(),
        "corpus_audit": copy.deepcopy(Corpus.audit),
        "source_records": copy.deepcopy(Corpus.source_records),
        "pipeline_claim": fake_record(claim_path),
        "worker_claim": fake_record(corpus_worker),
        "q2_paths_opened": [],
        "q3_paths_opened": [],
        "new_collection_count": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    assert runner._stage_receipt_payload_valid(corpus_receipt, stage_id=corpus_stage)
    for key, replacement in (
        ("corpus_audit", {"canonical_row_count": 9232}),
        ("source_records", [{"path": "arbitrary"}]),
    ):
        tampered = copy.deepcopy(corpus_receipt)
        tampered[key] = replacement
        assert not runner._stage_receipt_payload_valid(tampered, stage_id=corpus_stage)


def test_development_receipt_and_summary_reject_identity_and_type_drift(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    case_id = contract.DEVELOPMENT_CASE_IDS[0]
    stage_id = f"development__{case_id}"
    descriptor = contract.stage_descriptor(stage_id)
    destination = tmp_path / "development"
    module_path = tmp_path / "candidate_module"
    worker_root = tmp_path / "worker_claims"
    for path in (destination, module_path, worker_root):
        path.mkdir()
    claim_path = tmp_path / "pipeline_claim.json"
    freeze_path = tmp_path / "candidate_freeze.json"
    worker_path = worker_root / f"{stage_id}.json"
    for path in (claim_path, freeze_path, worker_path):
        path.write_text("{}", encoding="utf-8")

    def fake_record(path: str | Path) -> dict[str, object]:
        source = Path(path)
        data = source.read_bytes()
        return {
            "path": source.as_posix(),
            "sha256": runner.hashlib.sha256(data).hexdigest(),
            "size_bytes": len(data),
        }

    resolution = {
        str(descriptor["case"]["destination"]): destination,
        contract.CANDIDATE_MODULE_PATH.as_posix(): module_path,
    }
    monkeypatch.setattr(runner, "resolve_relative", lambda path: resolution[str(path)])
    monkeypatch.setattr(runner, "WORKER_CLAIMS_ROOT", worker_root)
    monkeypatch.setattr(runner, "PIPELINE_CLAIM_PATH", claim_path)
    monkeypatch.setattr(runner, "CANDIDATE_FREEZE_PATH", freeze_path)
    monkeypatch.setattr(runner, "_record", fake_record)
    monkeypatch.setattr(
        runner, "_worker_claim_payload_valid", lambda *_args, **_kwargs: True
    )
    module = {
        "tree_sha256": "b" * 64,
        "file_count": 1,
        "total_size_bytes": 1,
    }
    monkeypatch.setattr(runner, "_tree", lambda _path: module)
    identity = contract.candidate_id(module["tree_sha256"])
    trace_audit = {"schema_version": 1, "passed": True, "row_count": 500}
    monkeypatch.setattr(
        contract,
        "pure_policy_trace_audit",
        lambda _trace, **_kwargs: trace_audit,
    )
    gate = {"passed": True, "checks": {"strict": True}}
    monkeypatch.setattr(contract, "development_gate", lambda *_args, **_kwargs: gate)
    monkeypatch.setattr(
        runner, "_development_forensic_artifacts_valid", lambda _case_id: True
    )
    summary = _valid_development_summary(
        case_id=case_id,
        module=module,
        candidate_id=identity,
        candidate_freeze=fake_record(freeze_path),
        trace_audit=trace_audit,
    )
    summary_path = destination / "summary.json"
    gate_path = destination / "gate.json"
    trace_path = destination / "trace.json"
    summary_path.write_bytes(runner.forensic.canonical_json_bytes(summary))
    gate_path.write_bytes(runner.forensic.canonical_json_bytes(gate))
    trace_path.write_text("[{}]", encoding="utf-8")
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.DEVELOPMENT_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "case_id": case_id,
        "candidate_id": identity,
        "candidate_module": module,
        "candidate_freeze": fake_record(freeze_path),
        "summary": fake_record(summary_path),
        "gate": fake_record(gate_path),
        "trace": fake_record(trace_path),
        "pipeline_claim": fake_record(claim_path),
        "worker_claim": fake_record(worker_path),
        "pure_policy_trace_audit": trace_audit,
        "pure_policy_trace_row_count": contract.EXPECTED_STEPS,
        **{name: 0 for name in contract.PURE_POLICY_COUNTER_FIELDS},
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    assert runner._stage_receipt_payload_valid(receipt, stage_id=stage_id)
    for key in ("candidate_id", "candidate_module", "candidate_freeze"):
        tampered = copy.deepcopy(receipt)
        tampered[key] = None
        assert not runner._stage_receipt_payload_valid(tampered, stage_id=stage_id)
    for mutation in (
        lambda value: value.update(schema_version=float(contract.SCHEMA_VERSION)),
        lambda value: value.update(pure_policy_trace_row_count=500.0),
        lambda value: value.update(steps=500.0),
        lambda value: value.update(raw_sensor_sample_count=5000.0),
        lambda value: value.update(actor_updates=False),
        lambda value: value.update(action_clipped_values=False),
        lambda value: value.update(
            **{next(iter(contract.PURE_POLICY_COUNTER_FIELDS)): False}
        ),
    ):
        tampered_summary = copy.deepcopy(summary)
        mutation(tampered_summary)
        assert not runner._development_summary_payload_valid(
            tampered_summary,
            case_id=case_id,
            module=module,
            identity=identity,
            freeze_record=fake_record(freeze_path),
            trace_audit=trace_audit,
        )


def test_development_forensic_artifacts_bind_full_journal_and_failure_absence(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    case_id = contract.DEVELOPMENT_CASE_IDS[0]
    stage_id = f"development__{case_id}"
    descriptor = contract.stage_descriptor(stage_id)
    destination = tmp_path / "development"
    module_path = tmp_path / "candidate_module"
    module_path.mkdir()
    freeze_path = tmp_path / "candidate_freeze.json"
    claim_path = tmp_path / "pipeline_claim.json"
    worker_root = tmp_path / "worker_claims"
    worker_root.mkdir()
    worker_path = worker_root / f"{stage_id}.json"
    for path in (freeze_path, claim_path, worker_path):
        path.write_text("{}", encoding="utf-8")

    resolution = {
        str(descriptor["case"]["destination"]): destination,
        contract.CANDIDATE_MODULE_PATH.as_posix(): module_path,
    }
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(runner, "resolve_relative", lambda path: resolution[str(path)])
    monkeypatch.setattr(runner, "CANDIDATE_FREEZE_PATH", freeze_path)
    monkeypatch.setattr(runner, "PIPELINE_CLAIM_PATH", claim_path)
    monkeypatch.setattr(runner, "WORKER_CLAIMS_ROOT", worker_root)
    module = {
        "tree_sha256": "d" * 64,
        "file_count": 3,
        "total_size_bytes": 3,
    }
    monkeypatch.setattr(runner, "_tree", lambda _path: module)
    identity = contract.candidate_id(module["tree_sha256"])
    writer = runner.forensic.ForensicRolloutWriter(destination, artifact_root=tmp_path)
    writer.start(
        {
            "schema_version": contract.SCHEMA_VERSION,
            "status": "STARTED_H0_V12R5_CASE_BALANCED_DEVELOPMENT",
            "protocol_id": contract.PROTOCOL_ID,
            "stage_id": stage_id,
            "case": contract.canonical_development_case(case_id),
            "candidate_id": identity,
            "candidate_module": module,
            "candidate_freeze": runner._record(freeze_path),
            "teacher_enabled": False,
            "blending_enabled": False,
            "safety_latch_enabled": False,
            "pipeline_claim": runner._record(claim_path),
            "worker_claim": runner._record(worker_path),
        }
    )
    rows = []
    for step in range(1, contract.EXPECTED_STEPS + 1):
        row = {"step": step, "value": step}
        writer.write_step(step, row)
        rows.append(row)
    writer.finalize_before_gate(
        trace=rows,
        partial_summary={
            "schema_version": contract.SCHEMA_VERSION,
            "status": "PERSISTED_H0_V12R5_DEVELOPMENT_BEFORE_GATE",
            "protocol_id": contract.PROTOCOL_ID,
            "stage_id": stage_id,
            "steps": contract.EXPECTED_STEPS,
            "gate_evaluated": False,
        },
        summary={"complete": True},
    )
    writer.publish_gate({"passed": True})
    runner.forensic.write_json_exclusive(destination / "receipt.json", {"passed": True})

    assert runner._development_forensic_artifacts_valid(case_id) is True
    first_step = destination / "steps" / "000001.json"
    first_step.write_bytes(b"NOT_JSON")
    assert runner._development_forensic_artifacts_valid(case_id) is False
    first_step.write_bytes(
        runner.forensic.canonical_json_bytes({"step": 1, "value": 1})
    )
    assert runner._development_forensic_artifacts_valid(case_id) is True
    (destination / "failure.json").write_text("{}", encoding="utf-8")
    assert runner._development_forensic_artifacts_valid(case_id) is False


def test_final_receipt_and_summary_reject_identity_and_type_drift(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    run_root = tmp_path / "run"
    module_path = tmp_path / "candidate_module"
    worker_root = run_root / "worker_claims"
    for path in (run_root, module_path, worker_root):
        path.mkdir(parents=True, exist_ok=True)
    claim_path = run_root / "pipeline_claim.json"
    freeze_path = run_root / "candidate_freeze.json"
    worker_path = worker_root / "finalize_development.json"
    for path in (claim_path, freeze_path, worker_path):
        path.write_text("{}", encoding="utf-8")

    def fake_record(path: str | Path) -> dict[str, object]:
        source = Path(path)
        data = source.read_bytes()
        return {
            "path": source.as_posix(),
            "sha256": runner.hashlib.sha256(data).hexdigest(),
            "size_bytes": len(data),
        }

    monkeypatch.setattr(
        runner,
        "resolve_relative",
        lambda path: module_path
        if str(path) == contract.CANDIDATE_MODULE_PATH.as_posix()
        else tmp_path / str(path),
    )
    monkeypatch.setattr(runner, "RUN_ROOT", run_root)
    monkeypatch.setattr(runner, "WORKER_CLAIMS_ROOT", worker_root)
    monkeypatch.setattr(runner, "PIPELINE_CLAIM_PATH", claim_path)
    monkeypatch.setattr(runner, "CANDIDATE_FREEZE_PATH", freeze_path)
    monkeypatch.setattr(runner, "_record", fake_record)
    monkeypatch.setattr(
        runner, "_worker_claim_payload_valid", lambda *_args, **_kwargs: True
    )
    module = {
        "tree_sha256": "c" * 64,
        "file_count": 1,
        "total_size_bytes": 1,
    }
    monkeypatch.setattr(runner, "_tree", lambda _path: module)
    identity = contract.candidate_id(module["tree_sha256"])
    bindings = [
        {
            "case_id": case_id,
            "passed": True,
            "receipt": {"path": f"{case_id}/receipt.json"},
            "gate": {"path": f"{case_id}/gate.json"},
            "summary": {"path": f"{case_id}/summary.json"},
            "trace": {"path": f"{case_id}/trace.json"},
            "pure_policy_trace_audit": {"passed": True},
        }
        for case_id in contract.DEVELOPMENT_CASE_IDS
    ]
    monkeypatch.setattr(runner, "_expected_development_bindings", lambda: bindings)
    summary = runner._expected_final_development_summary(
        module=module, identity=identity, bindings=bindings
    )
    gate = {"passed": True, "checks": {"strict": True}}
    monkeypatch.setattr(contract, "aggregate_development_gate", lambda _summary: gate)
    summary_path = run_root / "final_development_summary.json"
    gate_path = run_root / "final_development_gate.json"
    summary_path.write_bytes(runner.forensic.canonical_json_bytes(summary))
    gate_path.write_bytes(runner.forensic.canonical_json_bytes(gate))
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.FINAL_DEVELOPMENT_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": "finalize_development",
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_id": identity,
        "candidate_module": module,
        "candidate_freeze": fake_record(freeze_path),
        "summary": fake_record(summary_path),
        "gate": fake_record(gate_path),
        "rollout_bindings": bindings,
        "pipeline_claim": fake_record(claim_path),
        "worker_claim": fake_record(worker_path),
        "development_only": True,
        "qualification_executed": False,
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "pure_policy_trace_row_count": len(contract.DEVELOPMENT_CASE_IDS)
        * contract.EXPECTED_STEPS,
        **{name: 0 for name in contract.PURE_POLICY_COUNTER_FIELDS},
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    assert runner._stage_receipt_payload_valid(receipt, stage_id="finalize_development")
    for key in ("candidate_id", "candidate_module", "candidate_freeze"):
        tampered = copy.deepcopy(receipt)
        tampered[key] = None
        assert not runner._stage_receipt_payload_valid(
            tampered, stage_id="finalize_development"
        )
    for mutation in (
        lambda value: value.update(schema_version=float(contract.SCHEMA_VERSION)),
        lambda value: value.update(candidate_tree_unique_count=True),
        lambda value: value.update(new_collection_count=False),
        lambda value: value.update(development_count=6.0),
        lambda value: value.update(environment_reset_calls=6.0),
        lambda value: value.update(actor_updates=True),
        lambda value: value.update(
            **{next(iter(contract.PURE_POLICY_COUNTER_FIELDS)): False}
        ),
    ):
        tampered_summary = copy.deepcopy(summary)
        mutation(tampered_summary)
        assert not runner._final_development_summary_payload_valid(
            tampered_summary,
            module=module,
            identity=identity,
            bindings=bindings,
        )


@pytest.mark.parametrize(
    ("kind", "expected_state"),
    [
        ("absent", "ABSENT"),
        ("zero", "ZERO_BYTE_REGULAR"),
        ("invalid_json", "INVALID_JSON_REGULAR"),
        ("invalid_schema", "INVALID_SCHEMA_REGULAR"),
        ("symlink", "UNSAFE_SYMLINK"),
        ("directory", "UNSAFE_NONREGULAR"),
    ],
)
def test_development_failure_observation_is_schema_aware_and_no_clobber(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    kind: str,
    expected_state: str,
) -> None:
    destination = tmp_path / "development"
    destination.mkdir()
    failure_path = destination / "failure.json"
    if kind == "zero":
        failure_path.touch()
    elif kind == "invalid_json":
        failure_path.write_text("{", encoding="utf-8")
    elif kind == "invalid_schema":
        failure_path.write_text("{}", encoding="utf-8")
    elif kind == "symlink":
        target = tmp_path / "target.json"
        target.write_text("{}", encoding="utf-8")
        failure_path.symlink_to(target)
    elif kind == "directory":
        failure_path.mkdir()

    def fake_record(path: str | Path) -> dict[str, object]:
        source = Path(path)
        data = source.read_bytes()
        return {
            "path": source.as_posix(),
            "sha256": runner.hashlib.sha256(data).hexdigest(),
            "size_bytes": len(data),
        }

    monkeypatch.setattr(runner, "resolve_relative", lambda _path: destination)
    monkeypatch.setattr(runner, "_record", fake_record)
    stage_id = "development__deterministic_offset_plus_0p20"
    observation = runner._development_failure_observation(stage_id)
    assert observation["state"] == expected_state
    if kind != "absent":
        with pytest.raises(runner.V12R5ExecutionError, match="existing development"):
            runner._ensure_development_failure_artifact(stage_id, RuntimeError())
        assert failure_path.exists() or failure_path.is_symlink()


def test_valid_pass_ledger_with_postwrite_verification_error_is_dominated(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    ledger_path, failure_path = _configure_terminal_fault_paths(tmp_path, monkeypatch)
    intended = _all_stages_intended_pass()

    def fail_verification() -> dict[str, object]:
        raise RuntimeError("postwrite verification failed")

    with monkeypatch.context() as fault:
        fault.setattr(runner, "verify_terminal_ledger", fail_verification)
        with pytest.raises(RuntimeError, match="postwrite verification failed"):
            runner._publish_terminal_ledger(intended)
    evidence = runner.forensic.strict_json_load(failure_path)
    assert ledger_path.is_file()
    assert evidence["intended_outcome"] == "PASS"
    assert evidence["completed_stage_count"] == len(contract.STAGE_IDS)
    assert evidence["attempted_stage"] is None
    assert evidence["pipeline_ledger_observation"]["state"] == "VALID"
    assert runner.verify_terminal_publication_failure() == evidence
    with pytest.raises(runner.V12R5ExecutionError, match="dominates"):
        runner.verify_terminal_ledger()
    evidence["retry_authorized"] = True
    failure_path.write_bytes(runner.forensic.canonical_json_bytes(evidence))
    with pytest.raises(runner.V12R5ExecutionError, match="evidence drifted"):
        runner.verify_terminal_publication_failure()


def test_zero_byte_terminal_ledger_gets_separate_failure_receipt(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    ledger_path, failure_path = _configure_terminal_fault_paths(tmp_path, monkeypatch)
    intended = _all_stages_intended_pass()
    original_write = runner.freezer._write_json_exclusive

    def injected_write(path: str | Path, payload: object) -> Path:
        destination = Path(path)
        if destination == ledger_path:
            destination.touch(exist_ok=False)
            raise OSError("injected zero-byte ledger")
        return original_write(destination, payload)

    monkeypatch.setattr(runner.freezer, "_write_json_exclusive", injected_write)
    with pytest.raises(OSError, match="injected zero-byte ledger"):
        runner._publish_terminal_ledger(intended)
    evidence = runner.forensic.strict_json_load(failure_path)
    assert ledger_path.stat().st_size == 0
    assert evidence["intended_outcome"] == "PASS"
    assert evidence["pipeline_ledger_observation"]["state"] == ("ZERO_BYTE_REGULAR")
    assert runner.verify_terminal_publication_failure() == evidence


@pytest.mark.parametrize("failure_point", ["ledger_build", "ledger_preverify"])
@pytest.mark.parametrize("passed", [False, True])
def test_terminalization_boundary_covers_build_and_preverify_failures(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    failure_point: str,
    passed: bool,
) -> None:
    ledger_path, failure_path = _configure_terminal_fault_paths(tmp_path, monkeypatch)
    if passed:
        _seed_stage_activity(completed_count=len(contract.STAGE_IDS))
        attempted = contract.STAGE_IDS[-1]
        completed = _all_stages_intended_pass()["completed_stages"]
        stage_error = None
    else:
        runner._reset_activity()
        for stage_id in contract.STAGE_IDS:
            runner._stage_receipt_path(stage_id).unlink()
        attempted = contract.STAGE_IDS[0]
        completed = []
        stage_error = ValueError("injected stage failure")
        runner._begin_stage_activity(attempted)
    synthetic = dict.fromkeys(runner._TERMINAL_LEDGER_FIELDS)
    synthetic.update(
        {
            "passed": passed,
            "completed_stages": copy.deepcopy(completed),
            "completed_stage_count": len(completed),
            "attempted_stage": attempted,
            "error": None if stage_error is None else runner._error_record(stage_error),
            "activity_totals": copy.deepcopy(runner._ACTIVITY_TOTALS),
            "stage_activity": [
                copy.deepcopy(runner._STAGE_ACTIVITY[stage_id])
                for stage_id in contract.STAGE_IDS
                if stage_id in runner._STAGE_ACTIVITY
            ],
            "attempted_stage_failure_publication_error": None,
            "next_stage": ("WAIT_SEPARATE_Q3_PROTOCOL" if passed else "STOP_TERMINAL"),
        }
    )

    if failure_point == "ledger_build":
        monkeypatch.setattr(
            runner,
            "_terminal_ledger",
            lambda **_kwargs: (_ for _ in ()).throw(
                RuntimeError("injected terminal ledger build failure")
            ),
        )
        expected_error = "injected terminal ledger build failure"
    else:
        monkeypatch.setattr(runner, "_terminal_ledger", lambda **_kwargs: synthetic)
        monkeypatch.setattr(
            runner,
            "_verify_terminal_ledger_payload",
            lambda *_args, **_kwargs: (_ for _ in ()).throw(
                RuntimeError("injected terminal ledger preverify failure")
            ),
        )
        expected_error = "injected terminal ledger preverify failure"

    with pytest.raises(RuntimeError, match=expected_error):
        runner._terminalize(
            passed=passed,
            attempted_stage=attempted,
            completed=completed,
            error=stage_error,
            failure_publication_error=None,
        )
    assert not ledger_path.exists()
    evidence = runner.forensic.strict_json_load(failure_path)
    assert evidence["intended_ledger"] is None
    assert evidence["intended_ledger_record"] is None
    assert evidence["intended_outcome"] == ("PASS" if passed else "FAIL")
    assert evidence["attempted_stage"] == (None if passed else attempted)
    assert evidence["terminal_intent"]["stage_error"] == (
        None if passed else runner._error_record(stage_error)
    )
    assert evidence["pipeline_ledger_observation"]["state"] == "ABSENT"
    assert runner.verify_terminal_publication_failure() == evidence


@pytest.mark.parametrize(
    ("kind", "expected_state"),
    [
        ("zero", "ZERO_BYTE_REGULAR"),
        ("invalid_json", "INVALID_JSON_REGULAR"),
        ("invalid_schema", "INVALID_SCHEMA_REGULAR"),
        ("symlink", "UNSAFE_SYMLINK"),
        ("directory", "UNSAFE_NONREGULAR"),
    ],
)
def test_partial_candidate_freeze_never_creates_a_terminal_gap(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    kind: str,
    expected_state: str,
) -> None:
    ledger_path, failure_path = _configure_terminal_fault_paths(tmp_path, monkeypatch)
    candidate_path = tmp_path / "candidate_freeze_receipt.json"
    if kind == "zero":
        candidate_path.touch()
    elif kind == "invalid_json":
        candidate_path.write_text("{", encoding="utf-8")
    elif kind == "invalid_schema":
        candidate_path.write_text("{}", encoding="utf-8")
    elif kind == "symlink":
        target = tmp_path / "candidate_target.json"
        target.write_text("{}", encoding="utf-8")
        candidate_path.symlink_to(target)
    else:
        candidate_path.mkdir()
    monkeypatch.setattr(runner, "CANDIDATE_FREEZE_PATH", candidate_path)
    monkeypatch.setattr(runner, "RUN_ROOT", tmp_path)
    attempted_claim = tmp_path / "freeze_case_balanced_candidate_claim.json"
    attempted_claim.write_text("{}", encoding="utf-8")
    monkeypatch.setattr(
        runner,
        "_claim_path",
        lambda stage_id: (
            attempted_claim
            if stage_id == "freeze_case_balanced_candidate"
            else tmp_path / f"{stage_id}_claim.json"
        ),
    )
    monkeypatch.setattr(
        runner,
        "_worker_claim_observation",
        lambda stage_id: (
            {
                "state": "VALID_REGULAR",
                "artifact": runner._record(attempted_claim),
            }
            if stage_id == "freeze_case_balanced_candidate"
            else {"state": "ABSENT", "artifact": None}
        ),
    )
    monkeypatch.setattr(
        runner,
        "_stage_output_observations",
        lambda stage_id: (
            {"receipt": runner._candidate_freeze_observation()}
            if stage_id == "freeze_case_balanced_candidate"
            else {"receipt": {"state": "ABSENT", "artifact": None}}
        ),
    )
    _seed_stage_activity(completed_count=3)
    for stage_id in contract.STAGE_IDS[3:]:
        runner._stage_receipt_path(stage_id).unlink()
    completed = [
        {
            "stage_id": stage_id,
            "receipt": runner._record(runner._stage_receipt_path(stage_id)),
        }
        for stage_id in contract.STAGE_IDS[:3]
    ]
    result = runner._terminalize(
        passed=False,
        attempted_stage="freeze_case_balanced_candidate",
        completed=completed,
        error=RuntimeError("candidate freeze failed"),
        failure_publication_error=None,
    )
    assert result["passed"] is False
    assert result["candidate_freeze_observation"]["state"] == expected_state
    assert ledger_path.is_file()
    assert not failure_path.exists()


def test_terminal_pass_verifier_is_historical_after_q3_outputs_open(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    run_root = tmp_path / "run"
    run_root.mkdir()
    receipt_root = tmp_path / "receipts"
    receipt_root.mkdir()
    receipt_paths = {
        stage_id: receipt_root / f"{index:02d}.json"
        for index, stage_id in enumerate(contract.STAGE_IDS)
    }
    for path in receipt_paths.values():
        path.write_text("{}", encoding="utf-8")
    candidate_path = tmp_path / "candidate_freeze.json"
    final_path = tmp_path / "final_receipt.json"
    final_summary_path = run_root / "final_development_summary.json"
    final_gate_path = run_root / "final_development_gate.json"
    claim_path = tmp_path / "pipeline_claim.json"
    protocol_path = tmp_path / "protocol.json"
    lock_path = tmp_path / "lock.json"
    module_path = tmp_path / "candidate_module"
    module_path.mkdir()
    for path in (
        candidate_path,
        final_path,
        final_summary_path,
        final_gate_path,
        claim_path,
        protocol_path,
        lock_path,
    ):
        path.write_text("{}", encoding="utf-8")
    q2_paths = {name: tmp_path / "q2" / name for name in contract.Q2_UNOPENED_PATHS}
    q3_paths = {name: tmp_path / "q3" / name for name in contract.Q3_UNOPENED_PATHS}
    resolution = {
        contract.CANDIDATE_MODULE_PATH.as_posix(): module_path,
        contract.FINAL_DEVELOPMENT_RECEIPT_PATH.as_posix(): final_path,
        **{
            path.as_posix(): q2_paths[name]
            for name, path in contract.Q2_UNOPENED_PATHS.items()
        },
        **{
            path.as_posix(): q3_paths[name]
            for name, path in contract.Q3_UNOPENED_PATHS.items()
        },
    }

    def fake_record(path: str | Path) -> dict[str, object]:
        source = Path(path)
        data = source.read_bytes() if source.is_file() else source.as_posix().encode()
        return {
            "path": source.resolve().relative_to(tmp_path.resolve()).as_posix(),
            "sha256": runner.hashlib.sha256(data).hexdigest(),
            "size_bytes": len(data),
        }

    module = {"tree_sha256": "a" * 64, "file_count": 1, "total_size_bytes": 1}
    candidate_id = contract.candidate_id(module["tree_sha256"])
    final_summary = {
        "rollout_bindings": [],
        **{name: 0 for name in contract.PURE_POLICY_COUNTER_FIELDS},
    }
    final_gate = {"passed": True}
    candidate = {
        "status": contract.CANDIDATE_FREEZE_STATUS,
        "passed": True,
        "candidate_id": candidate_id,
        "candidate_module": module,
    }
    final = {
        "status": contract.FINAL_DEVELOPMENT_PASS_STATUS,
        "passed": True,
        "candidate_id": candidate_id,
        "candidate_module": module,
        "candidate_freeze": fake_record(candidate_path),
        "summary": fake_record(final_summary_path),
        "gate": fake_record(final_gate_path),
        "rollout_bindings": [],
        "pipeline_claim": fake_record(claim_path),
        "worker_claim": fake_record(receipt_paths[contract.STAGE_IDS[-1]]),
        "pure_policy_trace_row_count": 3000,
        **{name: 0 for name in contract.PURE_POLICY_COUNTER_FIELDS},
    }
    mappings = {
        candidate_path: candidate,
        final_path: final,
        final_summary_path: final_summary,
        final_gate_path: final_gate,
        claim_path: {},
    }
    monkeypatch.setattr(
        runner,
        "resolve_relative",
        lambda path: resolution.get(str(path), tmp_path / "unused" / str(path)),
    )
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(runner, "CANDIDATE_FREEZE_PATH", candidate_path)
    monkeypatch.setattr(runner, "RUN_ROOT", run_root)
    monkeypatch.setattr(runner, "PROTOCOL_FREEZE_PATH", protocol_path)
    monkeypatch.setattr(runner, "LOCK_PATH", lock_path)
    monkeypatch.setattr(runner, "PIPELINE_CLAIM_PATH", claim_path)
    monkeypatch.setattr(
        runner, "PIPELINE_CLAIM_FAILURE_PATH", tmp_path / "claim_failure.json"
    )
    monkeypatch.setattr(
        runner,
        "PIPELINE_TERMINAL_PUBLICATION_FAILURE_PATH",
        tmp_path / "terminal_failure.json",
    )
    monkeypatch.setattr(runner, "_record", fake_record)
    monkeypatch.setattr(runner, "_tree", lambda _path: module)
    monkeypatch.setattr(
        runner, "_mapping", lambda path: copy.deepcopy(mappings[Path(path)])
    )
    monkeypatch.setattr(
        runner, "_stage_receipt_path", lambda stage_id: receipt_paths[stage_id]
    )
    monkeypatch.setattr(runner, "_claim_path", lambda stage_id: receipt_paths[stage_id])
    monkeypatch.setattr(
        runner,
        "_verify_stage_receipt_claim_bindings",
        lambda stage_id, *_args, **_kwargs: {"stage_id": stage_id},
    )
    worker_observation = {
        "state": "VALID_REGULAR",
        "artifact": fake_record(receipt_paths[contract.STAGE_IDS[-1]]),
    }
    monkeypatch.setattr(
        runner, "_worker_claim_observation", lambda _stage_id: worker_observation
    )
    monkeypatch.setattr(
        runner, "_verify_worker_claims_root_exact", lambda **_kwargs: None
    )
    monkeypatch.setattr(runner, "_completed_fit_lbfgs_closure_calls_current", lambda: 1)
    attempted_outputs = {
        "receipt": {"state": "VALID_JSON_REGULAR", "artifact": fake_record(final_path)}
    }
    monkeypatch.setattr(
        runner, "_stage_output_observations", lambda _stage_id: attempted_outputs
    )
    candidate_observation = {
        "state": "VALID_REGULAR",
        "artifact": fake_record(candidate_path),
    }
    monkeypatch.setattr(
        runner, "_candidate_freeze_observation", lambda: candidate_observation
    )
    monkeypatch.setattr(runner, "_pipeline_claim_payload_valid", lambda _value: True)
    monkeypatch.setattr(runner, "_external_runtime_sources_exact", lambda: True)
    monkeypatch.setattr(
        runner, "verify_execution_lock", lambda **_kwargs: {"passed": True}
    )
    monkeypatch.setattr(
        contract, "aggregate_development_gate", lambda _summary: final_gate
    )

    stage_activity = []
    totals = {name: 0 for name in runner._ACTIVITY_NAMES}
    for stage_id in contract.STAGE_IDS:
        row = {
            "stage_id": stage_id,
            "stage_kind": contract.stage_descriptor(stage_id)["kind"],
            **{name: 0 for name in runner._ACTIVITY_NAMES},
        }
        if stage_id == "fit_case_balanced_candidate":
            row.update(
                {
                    "actor_fit_stage_calls_attempted": 1,
                    "actor_fit_executions_confirmed": 1,
                    "actor_updates_attempted": 1,
                    "actor_updates_confirmed": 1,
                    "adamw_epochs_completed": 3000,
                    "lbfgs_closure_calls": 1,
                }
            )
        if contract.stage_descriptor(stage_id)["kind"] == "development":
            row.update(
                {
                    "environment_reset_calls": 1,
                    "environment_step_calls": 500,
                    "raw_sensor_sample_count": 5000,
                }
            )
        for name in runner._ACTIVITY_NAMES:
            totals[name] += row[name]
        stage_activity.append(row)
    completed = [
        {"stage_id": stage_id, "receipt": fake_record(receipt_paths[stage_id])}
        for stage_id in contract.STAGE_IDS
    ]
    ledger = dict.fromkeys(runner._TERMINAL_LEDGER_FIELDS)
    ledger.update(
        {
            "schema_version": contract.SCHEMA_VERSION,
            "status": contract.PIPELINE_PASS_STATUS,
            "passed": True,
            "terminal": True,
            "protocol_id": contract.PROTOCOL_ID,
            "pipeline_id": contract.PIPELINE_ID,
            "attempted_stage": contract.STAGE_IDS[-1],
            "completed_stages": completed,
            "completed_stage_count": len(completed),
            "stage_order": list(contract.STAGE_IDS),
            "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
            "candidate_id": candidate_id,
            "candidate_module": module,
            "candidate_freeze": fake_record(candidate_path),
            "candidate_freeze_observation": candidate_observation,
            "final_development_receipt": fake_record(final_path),
            "protocol_freeze": fake_record(protocol_path),
            "execution_lock": fake_record(lock_path),
            "pipeline_claim": fake_record(claim_path),
            "q2_paths_opened": [],
            "q3_paths_opened": [],
            "qualification_violation_detected": False,
            "preterminal_run_root_inventory": (
                runner._preterminal_run_root_inventory()
            ),
            "error": None,
            "activity_totals": totals,
            "stage_activity": stage_activity,
            "attempted_stage_output_observations": attempted_outputs,
            "attempted_stage_worker_claim_observation": worker_observation,
            "attempted_stage_worker_claim": worker_observation["artifact"],
            "attempted_stage_worker_claim_created": True,
            "attempted_stage_receipt": fake_record(final_path),
            "attempted_stage_receipt_created": True,
            "attempted_stage_failure_observation": None,
            "attempted_stage_failure_artifact": None,
            "attempted_stage_failure_publication_error": None,
            "new_collection_count": 0,
            "development_count": 6,
            "environment_reset_calls": totals["environment_reset_calls"],
            "environment_step_calls": totals["environment_step_calls"],
            "raw_sensor_sample_count": totals["raw_sensor_sample_count"],
            "teacher_query_count": totals["teacher_query_count"],
            "actor_fit_stage_calls_attempted": totals[
                "actor_fit_stage_calls_attempted"
            ],
            "actor_fit_executions_confirmed": totals["actor_fit_executions_confirmed"],
            "actor_updates_attempted": totals["actor_updates_attempted"],
            "actor_updates": totals["actor_updates_confirmed"],
            "adamw_epochs_completed": totals["adamw_epochs_completed"],
            "lbfgs_closure_calls": totals["lbfgs_closure_calls"],
            "critic_updates": 0,
            "ppo_updates": 0,
            "retry_authorized": False,
            "resume_authorized": False,
            "rescue_authorized": False,
            "sweep_authorized": False,
            "qualification_executed": False,
            "runtime_promoted": False,
            "checkpoint_zero_created": False,
            "positive_morphology_enabled": False,
            "next_stage": "WAIT_SEPARATE_Q3_PROTOCOL",
        }
    )
    assert runner._qualification_unopened() is True
    assert (
        runner._verify_terminal_ledger_payload(
            ledger, require_publication_failure_absent=True
        )["passed"]
        is True
    )
    rogue_path = run_root / "fit" / "rogue_checkpoint.bin"
    rogue_path.parent.mkdir()
    rogue_path.write_bytes(b"rogue")
    rogue_inventory = runner._preterminal_run_root_inventory()
    assert rogue_inventory["complete"] is True
    assert rogue_inventory["semantic_closed"] is False
    assert rogue_inventory["unexpected_relative_paths"] == ["fit/rogue_checkpoint.bin"]
    assert runner._preterminal_run_root_inventory_schema_valid(rogue_inventory)
    ledger["preterminal_run_root_inventory"] = rogue_inventory
    with pytest.raises(
        runner.V12R5ExecutionError, match="preterminal_inventory_current"
    ):
        runner._verify_terminal_ledger_payload(
            ledger, require_publication_failure_absent=True
        )
    rogue_path.unlink()
    ledger["preterminal_run_root_inventory"] = runner._preterminal_run_root_inventory()
    development_failure = (
        runner.resolve_relative(
            contract.DEVELOPMENT_PATHS[contract.DEVELOPMENT_CASE_IDS[0]]
        )
        / "failure.json"
    )
    development_failure.parent.mkdir(parents=True)
    development_failure.write_text("{}", encoding="utf-8")
    with pytest.raises(
        runner.V12R5ExecutionError, match="development_failure_artifacts_absent"
    ):
        runner._verify_terminal_ledger_payload(
            ledger, require_publication_failure_absent=True
        )
    development_failure.unlink()
    for path in q3_paths.values():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("{}", encoding="utf-8")
    assert runner._qualification_unopened() is False
    assert (
        runner._verify_terminal_ledger_payload(
            ledger, require_publication_failure_absent=True
        )["passed"]
        is True
    )


def test_partial_terminal_failure_receipt_blocks_ledger_resolution(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _ledger_path, failure_path = _configure_terminal_fault_paths(tmp_path, monkeypatch)
    failure_path.touch()
    with pytest.raises(runner.V12R5ExecutionError, match="invalid strict JSON"):
        runner.verify_terminal_ledger()
