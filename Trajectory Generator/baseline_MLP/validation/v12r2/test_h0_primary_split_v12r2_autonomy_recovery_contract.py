from __future__ import annotations

import copy
from pathlib import Path, PurePosixPath

import pytest

import h0_primary_split_v12r2_autonomy_recovery_contract as contract
import test_h0_primary_split_v12_autonomy_recovery_contract as v12_fixture


_SHA = "ab" * 32


def _artifact(path: str | PurePosixPath) -> dict[str, object]:
    return {
        "path": PurePosixPath(path).as_posix(),
        "sha256": _SHA,
        "size_bytes": 1,
    }


def _execution_sources() -> dict[str, dict[str, object]]:
    return {
        name: _artifact(contract.FUTURE_EXECUTION_SOURCE_RELATIVE_PATHS[name])
        for name in contract.FUTURE_EXECUTION_SOURCES_REQUIRED
    }


def _valid_replay_payload() -> dict[str, list[object]]:
    return {
        "boundary_time_s": [0.0, 0.1, 0.2],
        "pros_knee_angle_rad": [0.0, 0.1, 0.2],
        "pros_knee_speed_rad_s": [0.0, 1.0, 1.0],
        "pros_ankle_angle_rad": [0.0, -0.1, -0.2],
        "pros_ankle_speed_rad_s": [0.0, -1.0, -1.0],
        "legacy_left_normal_grf_bw": [0.0, 0.4, 0.0],
        "legacy_left_in_contact": [False, True, False],
        "actor_observations": [[0.0] * 35, [0.0] * 35],
        "previous_penetration_m": [0.0, 0.01],
        "legacy_left_event_boundary_offsets": [0, 0, 1, 2],
        "legacy_left_event_side": ["left", "left"],
        "legacy_left_event_type": ["heel_strike", "toe_off"],
        "legacy_left_event_onset_time_s": [0.09, 0.19],
        "legacy_left_event_confirmed_time_s": [0.095, 0.195],
        "legacy_left_event_delivered_time_s": [0.1, 0.2],
        "legacy_left_event_cycle_duration_present": [False, False],
        "legacy_left_event_cycle_duration_s": [0.0, 0.0],
        "legacy_left_event_contact_duration_present": [False, True],
        "legacy_left_event_contact_duration_s": [0.0, 0.1],
        "legacy_left_event_startup_contact_present": [True, False],
        "legacy_left_event_startup_contact": [True, False],
    }


def _valid_design_audit() -> dict[str, object]:
    metrics = {
        "rmse": 0.001,
        "max_abs_error": 0.01,
        "reset_max_abs_error": 0.001,
    }
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.DESIGN_AUDIT_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "contract_id": contract.FIT_CONTRACT_ID,
        "design_audit_id": contract.DESIGN_AUDIT_ID,
        "protocol_freeze": _artifact(contract.PROTOCOL_FREEZE_PATH),
        "v12_protocol_freeze": {
            "path": contract.V12_PROTOCOL_FREEZE_PATH.as_posix(),
            "sha256": contract.V12_PROTOCOL_FREEZE_SHA256,
            "size_bytes": contract.V12_PROTOCOL_FREEZE_SIZE_BYTES,
        },
        "execution_sources": _execution_sources(),
        "fit_contract_self_check": contract.fit_contract_self_check(),
        "v12r1_terminal_failure_lineage": copy.deepcopy(
            contract.V12R1_LINEAGE_ARTIFACTS
        ),
        "fit_design": copy.deepcopy(contract.FIT),
        "probe_replay_schema": copy.deepcopy(contract.PROBE_REPLAY_SCHEMA),
        "observed_metrics": metrics,
        "p0_reproduction_reference_metrics": copy.deepcopy(metrics),
        "p0_reproduction_tolerance": copy.deepcopy(contract.P0_REPRODUCTION_TOLERANCE),
        "dry_run": True,
        "actor_fit_executions": 1,
        "actor_updates": 1,
        "no_candidate_checkpoint": True,
        "candidate_checkpoints_persisted": 0,
        "candidate_checkpoint_paths": [],
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "offline_teacher_label_calls": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "retry_authorized": False,
        "sweep_authorized": False,
        "rescue_authorized": False,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "artifacts_written": [contract.DESIGN_AUDIT_RECEIPT_PATH.as_posix()],
    }


def _valid_execution_lock() -> dict[str, object]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.EXECUTION_LOCK_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "protocol_freeze": _artifact(contract.PROTOCOL_FREEZE_PATH),
        "v12_protocol_freeze": {
            "path": contract.V12_PROTOCOL_FREEZE_PATH.as_posix(),
            "sha256": contract.V12_PROTOCOL_FREEZE_SHA256,
            "size_bytes": contract.V12_PROTOCOL_FREEZE_SIZE_BYTES,
        },
        "design_audit": _artifact(contract.DESIGN_AUDIT_RECEIPT_PATH),
        "design_audit_gate_passed": True,
        "execution_sources": _execution_sources(),
        "fit_contract_self_check": contract.fit_contract_self_check(),
        "v12r1_terminal_failure_lineage": copy.deepcopy(
            contract.V12R1_LINEAGE_ARTIFACTS
        ),
        "inherited_runtime_evidence": {
            "passed": True,
            "v11_execution_lock": copy.deepcopy(contract.V11_EXECUTION_LOCK_ARTIFACT),
            "source_count": 66,
            "input_count": 54,
            "sources_exact": True,
            "inputs_exact": True,
        },
        "execution_authority": {
            "authority_date": "2026-08-09",
            "authority_text": contract.AUTHORITY_TEXT,
            "authority_scope": contract.EXECUTION_AUTHORITY_SCOPE,
            "one_shot": True,
        },
        "runtime": {
            "system": "test",
            "machine": "test",
            "executable": "/test/python",
            "inference_stack_ready": True,
            "platform_plugin_readiness": {"passed": True},
        },
        "training_ready": True,
        "training_ready_scope": {
            "system": "test",
            "machine": "test",
            "executable": "/test/python",
        },
        "retry_authorized": False,
        "sweep_authorized": False,
        "rescue_authorized": False,
        "post_hoc_retuning_authorized": False,
        "declared_mutation_paths": {
            name: path.as_posix()
            for name, path in contract.declared_mutation_paths().items()
        },
        "pipeline_claim_preexisting": False,
        "actor_fit_executions": 0,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "offline_teacher_label_calls": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "p0_reproduction_tolerance": copy.deepcopy(contract.P0_REPRODUCTION_TOLERANCE),
    }


def test_r2_is_additive_and_preserves_v12_topology_and_science() -> None:
    assert contract.SCHEMA_VERSION == 122
    assert "V12R2" in contract.PROTOCOL_ID
    assert "v12r2" in contract.RUN_ROOT.as_posix()
    assert contract.RUN_ROOT != contract._v12.RUN_ROOT
    assert contract.STAGE_IDS == contract._v12.STAGE_IDS
    assert contract.RECOVERY_WEIGHTING == contract._v12.RECOVERY_WEIGHTING
    assert contract.COVERAGE_WEIGHTING == contract._v12.COVERAGE_WEIGHTING
    assert contract.ROUND_ALPHAS == contract._v12.ROUND_ALPHAS
    assert contract.FIT_COMPLETED_ROUNDS == contract._v12.FIT_COMPLETED_ROUNDS
    assert contract.AUTHORITY["design_and_freeze_authorized"] is True
    assert contract.AUTHORITY["pipeline_execution_authorized"] is False


def test_teacher_mutable_range_is_inclusive_10_through_24() -> None:
    assert contract.TEACHER_MUTABLE_ACTOR_COLUMNS == tuple(range(10, 25))
    assert contract.PROBE_REPLAY_SCHEMA["teacher_mutable_actor_columns"] == list(
        range(10, 25)
    )
    assert 24 in contract.PROBE_REPLAY_SCHEMA["teacher_mutable_actor_columns"]
    assert 25 not in contract.PROBE_REPLAY_SCHEMA["teacher_mutable_actor_columns"]
    assert contract.replay_schema_gate(contract.PROBE_REPLAY_SCHEMA)["passed"] is True

    regression = copy.deepcopy(contract.PROBE_REPLAY_SCHEMA)
    regression["teacher_mutable_actor_columns"] = list(range(10, 24))
    gate = contract.replay_schema_gate(regression)
    assert gate["passed"] is False
    assert gate["checks"]["inclusive_teacher_columns"] is False


@pytest.mark.parametrize(
    ("group", "field"),
    [
        ("legacy_fsm_config_arrays", "legacy_fsm_config_json_utf8"),
        ("legacy_fsm_config_arrays", "legacy_fsm_config_sha256_ascii"),
        ("left_event_journal_arrays", "legacy_left_event_boundary_offsets"),
        ("left_event_journal_arrays", "legacy_left_event_confirmed_time_s"),
        ("left_event_journal_arrays", "legacy_left_event_delivered_time_s"),
        ("left_event_journal_arrays", "legacy_left_event_cycle_duration_present"),
        ("left_event_journal_arrays", "legacy_left_event_contact_duration_present"),
        ("left_event_journal_arrays", "legacy_left_event_startup_contact_present"),
    ],
)
def test_replay_schema_rejects_missing_config_or_event_field(
    group: str, field: str
) -> None:
    broken = copy.deepcopy(contract.PROBE_REPLAY_SCHEMA)
    del broken[group][field]
    assert contract.replay_schema_gate(broken)["passed"] is False


def test_replay_event_topology_is_lossless_and_fail_closed() -> None:
    payload = _valid_replay_payload()
    gate = contract.replay_event_topology_gate(payload, n_steps=2)
    assert gate["passed"] is True
    assert gate["event_count"] == 2

    missing = copy.deepcopy(payload)
    del missing["legacy_left_event_contact_duration_present"]
    assert contract.replay_event_topology_gate(missing, n_steps=2)["passed"] is False

    wrong_owner = copy.deepcopy(payload)
    wrong_owner["legacy_left_event_delivered_time_s"][0] = 0.2
    assert (
        contract.replay_event_topology_gate(wrong_owner, n_steps=2)["passed"] is False
    )

    noncanonical_absent = copy.deepcopy(payload)
    noncanonical_absent["legacy_left_event_cycle_duration_s"][0] = 1.0
    assert (
        contract.replay_event_topology_gate(noncanonical_absent, n_steps=2)["passed"]
        is False
    )


def test_inherited_probe_gate_uses_r2_schema_and_paths() -> None:
    case = contract.canonical_probe_case("p0")
    assert case["destination"].startswith(contract.PROBE_ROOT.as_posix())
    assert case["destination"].startswith(
        "Trajectory Generator/baseline_MLP/validation/v12r2/h0_v12r2_run_"
    )
    assert (
        contract.stage_receipt_path("probe_p0")
        .as_posix()
        .startswith(contract.RUN_ROOT.as_posix())
    )


def test_design_audit_gate_binds_replay_sources_and_tolerance() -> None:
    payload = _valid_design_audit()
    assert contract.design_audit_gate(payload)["passed"] is True

    wrong_range = copy.deepcopy(payload)
    wrong_range["probe_replay_schema"]["teacher_mutable_actor_columns"] = list(
        range(10, 24)
    )
    assert contract.design_audit_gate(wrong_range)["passed"] is False

    missing_source = copy.deepcopy(payload)
    missing_source["execution_sources"].pop(
        contract.FUTURE_EXECUTION_SOURCES_REQUIRED[-1]
    )
    assert contract.design_audit_gate(missing_source)["passed"] is False


def test_execution_lock_gate_requires_exact_five_sources_and_one_shot() -> None:
    payload = _valid_execution_lock()
    assert contract.execution_lock_gate(payload)["passed"] is True
    assert len(contract.FUTURE_EXECUTION_SOURCES_REQUIRED) == 7
    assert tuple(contract.FUTURE_EXECUTION_SOURCE_RELATIVE_PATHS) == (
        contract.FUTURE_EXECUTION_SOURCES_REQUIRED
    )

    extra = copy.deepcopy(payload)
    extra["execution_sources"]["unbound_overlay"] = _artifact("overlay.py")
    assert contract.execution_lock_gate(extra)["passed"] is False

    retry = copy.deepcopy(payload)
    retry["retry_authorized"] = True
    assert contract.execution_lock_gate(retry)["passed"] is False

    inherited_drift = copy.deepcopy(payload)
    inherited_drift["inherited_runtime_evidence"]["sources_exact"] = False
    assert contract.execution_lock_gate(inherited_drift)["passed"] is False


def test_p0_reproduction_tolerance_is_explicit_and_symmetric() -> None:
    reference = {
        "rmse": 0.001,
        "max_abs_error": 0.01,
        "reset_max_abs_error": 0.001,
    }
    observed = copy.deepcopy(reference)
    observed["rmse"] += 1.0e-10
    assert contract._metric_triplet_close(observed, reference) is True
    assert (
        contract.p0_reproduction_gate(
            observed, reference, contract.P0_REPRODUCTION_TOLERANCE
        )["passed"]
        is True
    )
    observed["rmse"] += 1.0e-6
    assert contract._metric_triplet_close(observed, reference) is False
    assert (
        contract.p0_reproduction_gate(
            observed, reference, contract.P0_REPRODUCTION_TOLERANCE
        )["passed"]
        is False
    )

    wrong_tolerance = copy.deepcopy(contract.P0_REPRODUCTION_TOLERANCE)
    wrong_tolerance["absolute"] = 1.0e-6
    assert (
        contract.p0_reproduction_gate(reference, reference, wrong_tolerance)["passed"]
        is False
    )


def _synthetic_passing_fit(
    monkeypatch: pytest.MonkeyPatch,
    stage: str,
    labelled: dict[str, int] | None = None,
) -> dict[str, object]:
    monkeypatch.setattr(v12_fixture, "contract", contract)
    summary = v12_fixture._passing_fit(stage, labelled)
    if stage == "p0":
        summary["p0_reproduction_reference_metrics"] = copy.deepcopy(summary["metrics"])
        summary["p0_reproduction_tolerance"] = copy.deepcopy(
            contract.P0_REPRODUCTION_TOLERANCE
        )
    return summary


def test_inherited_code_objects_are_not_literal_rewritten() -> None:
    assert (
        contract.expected_fit_counts.__code__
        is contract._v12.expected_fit_counts.__code__
    )
    assert contract._fit_corpus_audit_matches.__code__ is (
        contract._v12._fit_corpus_audit_matches.__code__
    )
    source = Path(contract.__file__).read_text(encoding="utf-8")
    assert "def _rewrite_code_strings" not in source


@pytest.mark.parametrize("stage", contract.FIT_STAGES)
def test_real_fit_gate_is_total_and_fail_closed(stage: str) -> None:
    gate = contract.fit_gate({}, stage=stage)
    assert gate["passed"] is False
    assert gate["status"] == contract.FIT_FAIL_STATUS
    assert gate["next_stage"] == "STOP_V12R2_FIT_FAILURE"


def test_real_contract_fit_gate_p0_synthetic_passes(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    summary = _synthetic_passing_fit(monkeypatch, "p0")
    gate = contract.fit_gate(summary, stage="p0")
    assert gate["passed"] is True
    assert gate["p0_reproduction_gate"]["passed"] is True

    broken = copy.deepcopy(summary)
    broken["fit_counts"]["completed_v12_collection_rounds"] = [1]
    assert contract.fit_gate(broken, stage="p0")["passed"] is False


def test_real_contract_fit_gate_p3_synthetic_passes(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    labelled = {"p0": 259, "p1": 500, "p2": 411}
    summary = _synthetic_passing_fit(monkeypatch, "p3", labelled)
    gate = contract.fit_gate(summary, stage="p3")
    assert gate["passed"] is True
    assert summary["fit_counts"]["completed_v12_collection_rounds"] == [1, 2, 3]
    assert "completed_v12r2_collection_rounds" not in summary["fit_counts"]

    broken = copy.deepcopy(summary)
    broken["corpus_audit"]["v12_dagger_sample_count"] += 1
    assert contract.fit_gate(broken, stage="p3")["passed"] is False


def test_fit_contract_self_check_covers_every_fit_stage() -> None:
    result = contract.fit_contract_self_check()
    assert result["passed"] is True
    assert set(result["stages"]) == set(contract.FIT_STAGES)
    assert all(stage["passed"] is True for stage in result["stages"].values())
