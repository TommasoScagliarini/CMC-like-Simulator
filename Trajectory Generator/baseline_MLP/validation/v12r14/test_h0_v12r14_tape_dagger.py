"""Source-only tests for the V12R14 P1 tape-reference collector."""

from __future__ import annotations

import copy
import json
import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest


REVISION_ROOT = Path(__file__).resolve().parent
if str(REVISION_ROOT) not in sys.path:
    sys.path.insert(0, str(REVISION_ROOT))

import h0_v12r14_tape_dagger as contract  # noqa: E402
import run_h0_v12r14_tape_dagger as runner  # noqa: E402


@pytest.fixture(scope="module")
def locked_tape() -> contract.LockedTapeCorpus:
    return contract.load_locked_tape_corpus()


@pytest.fixture(scope="module")
def locked_support(
    locked_tape: contract.LockedTapeCorpus,
) -> contract.SupportEnvelope:
    return contract.build_support_envelope(locked_tape)


def _small_envelope(*, shifted: bool = False) -> contract.SupportEnvelope:
    reference = np.zeros((2, len(contract.INVARIANT_COLUMNS)), dtype=np.float64)
    if shifted:
        reference[:] = -2.0
    return contract.SupportEnvelope(
        raw=reference.copy(),
        mean=np.zeros(len(contract.INVARIANT_COLUMNS), dtype=np.float64),
        scale=np.ones(len(contract.INVARIANT_COLUMNS), dtype=np.float64),
        standardized=reference,
        loo_nearest=np.ones(2, dtype=np.float64),
        p99=contract.SUPPORT_P99_THRESHOLD,
        loo_max=contract.SUPPORT_LOO_MAX,
        array_hashes={},
        case_ids=np.asarray([contract.CASE_IDS[0], contract.CASE_IDS[1]], dtype="U40"),
        step_indices=np.asarray([1, 1], dtype=np.int64),
    )


def _reference() -> contract.TapeReferenceStep:
    target = np.asarray([0.2, -0.1], dtype=np.float32)
    noise = np.asarray([0.003, -0.002], dtype=np.float32)
    raw = np.add(target, noise, dtype=np.float32)
    return contract.TapeReferenceStep(
        case_id=contract.CASE_IDS[0],
        step=1,
        observation=np.zeros(contract.EXPECTED_ACTOR_FEATURES, dtype=np.float32),
        target_mean=target,
        raw_action=raw,
        teacher_std=np.full(
            contract.EXPECTED_ACTION_DIM,
            contract.EXPECTED_SIGMA,
            dtype=np.float32,
        ),
        frozen_noise=noise,
        runtime_time_s=1.0,
    )


def _actor_observation(*, invariant_value: float = 0.0) -> np.ndarray:
    value = np.zeros(contract.EXPECTED_ACTOR_FEATURES, dtype=np.float32)
    value[np.asarray(contract.INVARIANT_COLUMNS)] = np.float32(invariant_value)
    return value


def _passing_summary(plan_row: dict) -> dict:
    zero_fields = (
        "action_clipped_values",
        "fallback_count",
        "timeout_count",
        "safety_stop_count",
        "sea_plugin_fallback_count",
        "so_solver_unaccepted_count",
        "hard_invalid_count",
        "invalid_event_count",
        "nonfinite_count",
        "routing_failure_count",
        "step_contract_failure_count",
        "binary_event_failure_count",
        "target_provenance_mismatch_count",
        "tape_time_alignment_mismatch_count",
        "candidate_query_mismatch_count",
        "support_decision_mismatch_count",
        "latch_rule_violation_count",
        "blend_mismatch_count",
        "noise_application_mismatch_count",
        "source_or_candidate_drift_count",
    )
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.COLLECTION_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "behavior_id": contract.BEHAVIOR_ID,
        "trajectory_id": plan_row["trajectory_id"],
        "case_id": plan_row["case_id"],
        "requested_alpha": plan_row["requested_alpha"],
        "steps": contract.EXPECTED_STEPS,
        "sample_count": contract.EXPECTED_STEPS,
        "persisted_label_count": contract.EXPECTED_STEPS,
        "control_window_count": contract.EXPECTED_CONTROL_WINDOWS,
        "raw_sensor_sample_count": contract.EXPECTED_RAW_SENSOR_SAMPLES,
        "end_reason": "episode_time_limit",
        "terminated": False,
        "truncated": True,
        "phase_valid_cycle_count": 2,
        "grf_penetration_max_m": 0.024,
        "binary_phase_fsm_mode": "binary_active",
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "morphology_weight": 0.0,
        "candidate_mean_query_count": contract.EXPECTED_STEPS,
        "tape_reference_lookup_count": contract.EXPECTED_STEPS,
        "tape_target_binding_count": contract.EXPECTED_STEPS,
        "teacher_model_query_count": 0,
        "legacy_shadow_query_count": 0,
        "support_query_count": contract.EXPECTED_STEPS,
        "support_id": contract.SUPPORT_ID,
        "support_p99": contract.SUPPORT_P99_THRESHOLD,
        "mean_blend_count": contract.EXPECTED_STEPS,
        "single_noise_application_count": contract.EXPECTED_STEPS,
        "frozen_noise_lookup_count": contract.EXPECTED_STEPS,
        "random_noise_draw_count": 0,
        "safety_latch_activation_m": contract.v10s_blend.SAFETY_LATCH_ACTIVATION_M,
        "safety_latch_release_m": contract.v10s_blend.SAFETY_LATCH_RELEASE_M,
        "safety_latch_release_phase": contract.v10s_blend.SAFETY_LATCH_RELEASE_PHASE,
        "safety_signal_lag_steps": 1,
        "candidate_update_count_before": 0,
        "candidate_update_count_after": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "source_snapshot_before": {"sha": "same"},
        "source_snapshot_after": {"sha": "same"},
        "support_fallback_count": 10,
        "safety_fallback_count": 5,
        "recovery_row_count": 12,
        "safety_latch_activation_count": 1,
        "safety_latch_release_count": 1,
    }
    summary.update({field: 0 for field in zero_fields})
    return summary


def test_plan_is_exactly_18_alpha_major_canonical_cases() -> None:
    plan = contract.collection_plan()
    assert len(plan) == 18
    assert [row["ordinal"] for row in plan] == list(range(1, 19))
    assert [row["requested_alpha"] for row in plan] == [
        alpha for alpha in contract.ROUND_ALPHAS for _case in contract.CASE_IDS
    ]
    assert [row["case_id"] for row in plan] == list(contract.CASE_IDS) * 3
    assert len({row["trajectory_id"] for row in plan}) == 18
    assert all(row["candidate_stage"] == "p0_fit" for row in plan)
    assert all(row["candidate_update_count_before"] == 0 for row in plan)
    assert all(row["candidate_update_count_after"] == 0 for row in plan)


def test_protocol_description_is_read_only_and_interoperable() -> None:
    description = contract.build_protocol_description()
    assert description["p1_rows"] == 9_000
    assert description["expected_p2_rows"] == 12_000
    assert description["teacher_model_queries"] == 0
    assert description["legacy_shadow_queries"] == 0
    assert description["execution_mode"] == {
        "description_only": True,
        "checkpoint_loaded": False,
        "policy_queried": False,
        "environment_built": False,
        "environment_reset": False,
        "environment_step": False,
        "artifact_written": False,
    }
    assert description["outputs"]["receipt"].endswith(
        "/p1_candidate_exposed/receipt.json"
    )


def test_locked_tape_is_exact_3000_rows_and_never_uses_noisy_target(
    locked_tape: contract.LockedTapeCorpus,
) -> None:
    assert locked_tape.observations.shape == (3_000, 35)
    assert locked_tape.targets.shape == (3_000, 2)
    assert locked_tape.array_hashes == contract.EXPECTED_TAPE_ARRAY_HASHES
    assert locked_tape.source_closure["transitive_step_files_verified"] == 3_000
    assert list(dict.fromkeys(locked_tape.case_ids.tolist())) == list(contract.CASE_IDS)
    stochastic = locked_tape.reference("stochastic_nominal_seed_126", 1)
    assert not np.array_equal(stochastic.target_mean, stochastic.raw_action)
    assert np.array_equal(
        np.add(stochastic.target_mean, stochastic.frozen_noise, dtype=np.float32),
        stochastic.raw_action,
    )


def test_global_support_recalibration_is_exact_and_supersedes_plus_only(
    locked_support: contract.SupportEnvelope,
) -> None:
    assert locked_support.raw.shape == (3_000, 18)
    assert locked_support.p99 == 0.1937808123139821
    assert locked_support.loo_max == 0.9882838030060314
    assert locked_support.array_hashes["raw"] == contract.EXPECTED_SUPPORT_RAW_HASH
    manifest = contract.support_manifest(locked_support)
    assert manifest["reference_scope"] == "all_six_v8r1p1_cases_3000_rows"
    assert manifest["calibration_scope"] == "same_global_3000_rows_leave_one_out"
    assert manifest["legacy_plus_0p20_only_p99_excluded"] == pytest.approx(
        0.20330878485396986
    )
    assert manifest["p99_hex"] == float(manifest["p99"]).hex()
    assert manifest["loo_max_hex"] == float(manifest["loo_max"]).hex()


def test_exact_safe_row_has_zero_global_support_distance(
    locked_tape: contract.LockedTapeCorpus,
    locked_support: contract.SupportEnvelope,
) -> None:
    query = locked_support.query(locked_tape.observations[0])
    assert query.distance_rms_z == 0.0
    assert query.within_p99 is True


def test_support_boundary_is_closed_and_nextafter_is_fallback() -> None:
    threshold = contract.SUPPORT_P99_THRESHOLD
    assert contract.support_within_p99(np.nextafter(threshold, -math.inf)) is True
    assert contract.support_within_p99(threshold) is True
    assert contract.support_within_p99(np.nextafter(threshold, math.inf)) is False
    with pytest.raises(contract.V12R14TapeDaggerError):
        contract.support_within_p99(float("nan"))


def test_nearest_tie_and_near_tie_are_resolved_portably() -> None:
    query = np.zeros(len(contract.INVARIANT_COLUMNS), dtype=np.float64)
    reference = np.stack(
        (
            np.full(len(contract.INVARIANT_COLUMNS), 1.0e-13),
            np.zeros(len(contract.INVARIANT_COLUMNS)),
        )
    )
    index, distance = contract._canonical_nearest_rms(  # noqa: SLF001
        query,
        reference,
        approximate_distances=np.asarray([0.0, 5.0e-13]),
    )
    assert index == 1
    assert distance == 0.0
    tied = np.zeros((2, len(contract.INVARIANT_COLUMNS)), dtype=np.float64)
    index, distance = contract._canonical_nearest_rms(  # noqa: SLF001
        query,
        tied,
        approximate_distances=np.zeros(2),
    )
    assert index == 0
    assert distance == 0.0


def test_regular_blend_uses_student_weight_then_one_frozen_noise() -> None:
    reference = _reference()
    candidate = np.asarray([0.6, 0.3], dtype=np.float32)
    result = contract.select_tape_dagger_action(
        candidate_mean=candidate,
        candidate_std=reference.teacher_std,
        observation=_actor_observation(),
        reference=reference,
        requested_alpha=0.25,
        latch_state=contract.v10s_blend.SafetyLatchState(),
        previous_penetration_m=0.0,
        active_v26_phase="WAIT_HS",
        support_envelope=_small_envelope(),
    )
    expected_mean = np.add(
        np.multiply(reference.target_mean, np.float32(0.75), dtype=np.float32),
        np.multiply(candidate, np.float32(0.25), dtype=np.float32),
        dtype=np.float32,
    )
    assert result.effective_alpha == 0.25
    assert result.recovery is False
    assert result.fallback_reasons == ()
    assert np.array_equal(result.blended_mean, expected_mean)
    assert np.array_equal(
        result.raw_action,
        np.add(expected_mean, reference.frozen_noise, dtype=np.float32),
    )


def test_support_exit_falls_back_to_same_step_tape_mean() -> None:
    reference = _reference()
    result = contract.select_tape_dagger_action(
        candidate_mean=np.asarray([0.8, 0.8], dtype=np.float32),
        candidate_std=reference.teacher_std,
        observation=_actor_observation(invariant_value=2.0),
        reference=reference,
        requested_alpha=0.75,
        latch_state=contract.v10s_blend.SafetyLatchState(),
        previous_penetration_m=0.0,
        active_v26_phase="WAIT_HS",
        support_envelope=_small_envelope(),
    )
    assert result.support_intervened is True
    assert result.safety_intervened is False
    assert result.effective_alpha == 0.0
    assert result.recovery is True
    assert result.fallback_reasons == ("support_p99_exit",)
    assert np.array_equal(result.blended_mean, reference.target_mean)
    assert np.array_equal(result.raw_action, reference.raw_action)


def test_causal_latch_enters_at_15mm_and_releases_only_in_swing() -> None:
    reference = _reference()
    entered = contract.select_tape_dagger_action(
        candidate_mean=np.asarray([0.8, 0.8], dtype=np.float32),
        candidate_std=reference.teacher_std,
        observation=_actor_observation(),
        reference=reference,
        requested_alpha=0.50,
        latch_state=contract.v10s_blend.SafetyLatchState(),
        previous_penetration_m=0.015,
        active_v26_phase="STANCE_AFTER_HS",
        support_envelope=_small_envelope(),
    )
    assert entered.latch_entered is True
    assert entered.safety_intervened is True
    assert entered.effective_alpha == 0.0
    held = contract.select_tape_dagger_action(
        candidate_mean=np.asarray([0.8, 0.8], dtype=np.float32),
        candidate_std=reference.teacher_std,
        observation=_actor_observation(),
        reference=reference,
        requested_alpha=0.50,
        latch_state=entered.latch_state,
        previous_penetration_m=0.010,
        active_v26_phase="STANCE_AFTER_HS",
        support_envelope=_small_envelope(),
    )
    assert held.latch_released is False
    assert held.effective_alpha == 0.0
    released = contract.select_tape_dagger_action(
        candidate_mean=np.asarray([0.8, 0.8], dtype=np.float32),
        candidate_std=reference.teacher_std,
        observation=_actor_observation(),
        reference=reference,
        requested_alpha=0.50,
        latch_state=held.latch_state,
        previous_penetration_m=0.010,
        active_v26_phase="SWING_AFTER_TO",
        support_envelope=_small_envelope(),
    )
    assert released.latch_released is True
    assert released.safety_intervened is False
    assert released.effective_alpha == 0.50


def test_action_selection_rejects_unregistered_alpha_and_logstd_drift() -> None:
    reference = _reference()
    kwargs = {
        "candidate_mean": np.zeros(2, dtype=np.float32),
        "candidate_std": reference.teacher_std,
        "observation": _actor_observation(),
        "reference": reference,
        "requested_alpha": 0.25,
        "latch_state": contract.v10s_blend.SafetyLatchState(),
        "previous_penetration_m": 0.0,
        "active_v26_phase": "WAIT_HS",
        "support_envelope": _small_envelope(),
    }
    with pytest.raises(contract.V12R14TapeDaggerError):
        contract.select_tape_dagger_action(**{**kwargs, "requested_alpha": 0.4})
    with pytest.raises(contract.V12R14TapeDaggerError):
        contract.select_tape_dagger_action(
            **{
                **kwargs,
                "candidate_std": np.full(2, 0.006, dtype=np.float32),
            }
        )


def test_collection_gate_accepts_shield_diagnostics_without_relaxing_physics() -> None:
    plan_row = dict(contract.collection_plan()[0])
    gate = contract.collection_gate(_passing_summary(plan_row), plan_row=plan_row)
    assert gate["passed"] is True
    assert gate["physical_gate_relaxed"] is False
    assert gate["shielded_collection_consumes_pure_trial"] is False


@pytest.mark.parametrize(
    ("field", "value", "failed_check"),
    (
        ("grf_penetration_max_m", 0.025, "penetration_semantics"),
        ("action_clipped_values", 1, "zero_anomalies"),
        ("candidate_mean_query_count", 499, "candidate_once_per_step"),
        ("teacher_model_query_count", 1, "no_teacher_model_query"),
        ("source_snapshot_after", {"sha": "drift"}, "source_stable"),
    ),
)
def test_collection_gate_fails_independent_blockers(
    field: str, value: object, failed_check: str
) -> None:
    plan_row = dict(contract.collection_plan()[0])
    summary = _passing_summary(plan_row)
    summary[field] = value
    gate = contract.collection_gate(summary, plan_row=plan_row)
    assert gate["passed"] is False
    assert gate["checks"][failed_check] is False


def test_transition_mask_is_per_trajectory_and_radius_two() -> None:
    rows = [
        {
            "step": step,
            "phase_fsm": {
                "accepted_transitions_this_step": (
                    [{"event": "HS"}] if step in {1, 250, 500} else []
                )
            },
        }
        for step in range(1, 501)
    ]
    mask, events, transition_steps = runner._transition_mask(rows)  # noqa: SLF001
    assert events == [1, 250, 500]
    assert transition_steps == [
        1,
        2,
        3,
        248,
        249,
        250,
        251,
        252,
        498,
        499,
        500,
    ]
    assert int(np.count_nonzero(mask)) == 11


def test_runtime_transitive_closure_remains_exact_94_files() -> None:
    closure = contract.verify_runtime_source_closure()
    assert closure["file_count"] == 94
    assert closure["basis_execution_lock"] == contract.V12R10_EXECUTION_LOCK_RECORD


def test_protocol_freeze_is_p0_independent_and_does_not_write() -> None:
    before = os.path.lexists(contract.P1_PROTOCOL_FREEZE)
    payload = runner.build_protocol_freeze()
    after = os.path.lexists(contract.P1_PROTOCOL_FREEZE)
    assert after == before
    assert payload["passed"] is True
    assert payload["p0_binding"] == "DEFERRED_TO_EXECUTION_LOCK_AFTER_PASS_P0"
    assert payload["protocol"]["rollout_count"] == 18


def test_npz_writer_is_exclusive(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    destination = tmp_path / "out" / "corpus.npz"
    arrays = {"values": np.asarray([1.0], dtype=np.float32)}
    runner._write_npz_exclusive(destination, arrays)  # noqa: SLF001
    with pytest.raises(runner.V12R14TapeDaggerExecutionError, match="no-clobber"):
        runner._write_npz_exclusive(destination, arrays)  # noqa: SLF001
    with np.load(destination, allow_pickle=False) as archive:
        assert np.array_equal(archive["values"], arrays["values"])


def test_execute_cli_requires_exact_one_shot_acknowledgement(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    called = False

    def stub_execute() -> dict:
        nonlocal called
        called = True
        return {"passed": True}

    monkeypatch.setattr(runner, "execute_p1", stub_execute)
    assert runner.main(["--execute"]) == 2
    error = json.loads(capsys.readouterr().err)
    assert error["passed"] is False
    assert error["status"] == "FAIL_H0_V12R14_P1_TAPE_DAGGER_CLI"
    assert "one-shot acknowledgement" in error["error"]
    assert called is False
    assert runner.main(["--execute", "--acknowledge-one-shot", "WRONG_VALUE"]) == 2
    capsys.readouterr()
    assert called is False
    assert (
        runner.main(
            ["--execute", "--acknowledge-one-shot", runner.EXECUTION_ACKNOWLEDGEMENT]
        )
        == 0
    )
    capsys.readouterr()
    assert called is True


def test_governance_cli_requires_exact_reviewed_source_acknowledgement(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    freeze_called = False
    lock_called = False

    def stub_freeze() -> dict:
        nonlocal freeze_called
        freeze_called = True
        return {"passed": True}

    def stub_lock() -> dict:
        nonlocal lock_called
        lock_called = True
        return {"passed": True}

    monkeypatch.setattr(runner, "write_protocol_freeze", stub_freeze)
    monkeypatch.setattr(runner, "write_execution_lock", stub_lock)
    assert runner.main(["--write-protocol-freeze"]) == 2
    assert runner.main(["--write-execution-lock"]) == 2
    assert (
        runner.main(
            ["--write-protocol-freeze", "--acknowledge-governance", "WRONG_VALUE"]
        )
        == 2
    )
    capsys.readouterr()
    assert freeze_called is False and lock_called is False
    assert (
        runner.main(
            [
                "--write-protocol-freeze",
                "--acknowledge-governance",
                runner.GOVERNANCE_ACKNOWLEDGEMENT,
            ]
        )
        == 0
    )
    capsys.readouterr()
    assert freeze_called is True
    assert (
        runner.main(
            [
                "--write-execution-lock",
                "--acknowledge-governance",
                runner.GOVERNANCE_ACKNOWLEDGEMENT,
            ]
        )
        == 0
    )
    capsys.readouterr()
    assert lock_called is True


def test_default_cli_is_description_only(capsys: pytest.CaptureFixture[str]) -> None:
    assert runner.main([]) == 0
    value = json.loads(capsys.readouterr().out)
    assert value["execution_mode"]["description_only"] is True
    assert value["rollout_count"] == 18


def test_execute_rejects_before_context_loader_when_lock_preflight_fails(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    context_called = False

    def fail_preflight(**_kwargs: object) -> runner.PreparedP1:
        raise runner.V12R14TapeDaggerExecutionError("execution lock stale")

    def forbidden_context(_path: Path) -> runner.ExecutionContext:
        nonlocal context_called
        context_called = True
        raise AssertionError("context must not load")

    monkeypatch.setattr(runner, "prepare_p1", fail_preflight)
    with pytest.raises(
        runner.V12R14TapeDaggerExecutionError, match="execution lock stale"
    ):
        runner.execute_p1(context_loader=forbidden_context)
    assert context_called is False


def test_manifest_public_schema_names_are_frozen_in_source() -> None:
    source = Path(runner.__file__).read_text(encoding="utf-8")
    for field in (
        '"array_keys"',
        '"array_shapes"',
        '"array_dtypes"',
        '"array_hashes"',
        '"trajectory_ids"',
        '"transition_window_provenance"',
    ):
        assert field in source
    assert contract.P1_LEDGER_STATUS == ("PASS_H0_V12R14_P1_CANDIDATE_EXPOSED_PIPELINE")
    assert contract.P1_RECEIPT.name == "receipt.json"


def test_passing_summary_fixture_has_no_hidden_mutation() -> None:
    plan_row = dict(contract.collection_plan()[0])
    summary = _passing_summary(plan_row)
    original = copy.deepcopy(summary)
    contract.collection_gate(summary, plan_row=plan_row)
    assert summary == original


def _truncated_summary(plan_row: dict, *, steps: int) -> dict:
    """A COHERENT physically-truncated shielded prefix (V12R13 lesson)."""

    summary = _passing_summary(plan_row)
    summary["steps"] = steps
    summary["sample_count"] = steps
    summary["persisted_label_count"] = steps
    summary["control_window_count"] = steps * 10
    summary["raw_sensor_sample_count"] = steps * 10
    summary["end_reason"] = "grf_penetration"
    summary["terminated"] = True
    summary["truncated"] = False
    summary["phase_valid_cycle_count"] = 0
    summary["grf_penetration_max_m"] = contract.PENETRATION_LIMIT_M + 5.7e-5
    summary["candidate_mean_query_count"] = steps
    summary["tape_reference_lookup_count"] = steps
    summary["tape_target_binding_count"] = steps
    summary["support_query_count"] = steps
    summary["mean_blend_count"] = steps
    summary["single_noise_application_count"] = steps
    summary["frozen_noise_lookup_count"] = steps
    return summary


def test_collection_gate_recovers_coherent_physically_truncated_prefix() -> None:
    """V12R13 lesson (57-micron fail): a coherent shielded prefix terminated
    by the physics is valid COLLECTION data, never a physical PASS."""

    plan_row = dict(contract.collection_plan()[0])
    summary = _truncated_summary(plan_row, steps=202)
    gate = contract.collection_gate(summary, plan_row=plan_row)
    assert gate["passed"] is True
    assert gate["full_horizon"] is False
    assert gate["physically_truncated"] is True
    assert gate["recoverable_for_data_collection"] is True
    assert gate["physical_full_horizon_pass"] is False
    assert gate["steps"] == 202


def test_collection_gate_rejects_incoherent_truncation() -> None:
    """Changing steps without the per-step counters is journal incoherence,
    not a physical truncation: it must still FAIL closed."""

    plan_row = dict(contract.collection_plan()[0])
    summary = _passing_summary(plan_row)
    summary["steps"] = 212
    gate = contract.collection_gate(summary, plan_row=plan_row)
    assert gate["passed"] is False
    assert gate["checks"]["horizon_semantics"] is False

    coherent = _truncated_summary(plan_row, steps=202)
    below_guard = dict(coherent)
    below_guard["grf_penetration_max_m"] = contract.PENETRATION_LIMIT_M - 1.0e-4
    gate = contract.collection_gate(below_guard, plan_row=plan_row)
    assert gate["passed"] is False
    assert gate["checks"]["penetration_semantics"] is False

    anomalous = _truncated_summary(plan_row, steps=202)
    anomalous["latch_rule_violation_count"] = 1
    gate = contract.collection_gate(anomalous, plan_row=plan_row)
    assert gate["passed"] is False
    assert gate["checks"]["zero_anomalies"] is False


def test_execution_lock_payload_is_occupancy_independent(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """V12R10 lesson: the P1 lock payload must not depend on namespace occupancy."""

    from types import SimpleNamespace

    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    freeze_path = tmp_path / "p1_freeze.json"
    freeze_path.write_text('{"stub": true}\n', encoding="utf-8")
    p1_root = tmp_path / "p1_candidate_exposed"
    monkeypatch.setattr(contract, "P1_PROTOCOL_FREEZE", freeze_path)
    monkeypatch.setattr(contract, "P1_ROOT", p1_root)
    monkeypatch.setattr(
        contract, "support_manifest", lambda support: {"stub_support": str(support)}
    )
    monkeypatch.setattr(
        contract,
        "collection_plan",
        lambda: [{"trajectory_id": "stub", "case_id": "stub", "requested_alpha": 0.25}],
    )
    prepared = SimpleNamespace(
        source_candidate_id="STUB_CANDIDATE",
        source_candidate_tree={
            "path": "p0",
            "tree_sha256": "a" * 64,
            "file_count": 1,
            "files": [],
        },
        source_snapshot={"stub": "snapshot"},
        tape=SimpleNamespace(array_hashes={"raw": "b" * 64}),
        support="STUB_SUPPORT",
    )
    before = runner._execution_lock_payload(prepared)  # noqa: SLF001
    assert "p1_destination_unoccupied" not in before
    assert before["p1_destination_unoccupied_at_lock_time"] is True
    p1_root.mkdir()
    (p1_root / "occupied.txt").write_text("occupied\n", encoding="utf-8")
    after = runner._execution_lock_payload(prepared)  # noqa: SLF001
    assert runner.forensic.canonical_json_bytes(
        before
    ) == runner.forensic.canonical_json_bytes(after)
