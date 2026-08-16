"""Pure tests for the one-round V6 residual/DAgger contract."""

from __future__ import annotations

import copy

import pytest

from validation import h0_primary_split_v6_qualification_contract as qualification
from validation import h0_primary_split_v6_residual_dagger_contract as contract


def _fit_summary(stage: str, *, dagger_samples: int) -> dict:
    fit = contract.P0_FIT if stage == "p0" else contract.P1_FIT
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "protocol_id": contract.PROTOCOL_ID,
        "fit_stage": stage,
        "fit": fit,
        "sample_count": 3000 + dagger_samples,
        "teacher_sample_count": 3000,
        "dagger_sample_count": dagger_samples,
        "reset_row_count": 6 if stage == "p0" else 8,
        "metrics": {
            "rmse": 0.001,
            "max_abs_error": 0.01,
            "reset_max_abs_error": 5.0e-6,
        },
        "all_finite": True,
        "base_h0_byte_exact": True,
        "critic_byte_exact": True,
        "logstd_byte_exact": True,
        "residual_limits": list(contract.RESIDUAL_LIMITS),
        "residual_input_indices": list(contract.RESIDUAL_INPUT_INDICES),
        "normalization_frozen": True,
        "optimizer_steps": fit["epochs"],
        "promotable": stage == "p1",
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }


def _dagger_summary(*, mode: str = "episode_time_limit", samples: int = 500):
    successful_steps = samples if mode == "episode_time_limit" else samples - 1
    rejection = (
        {
            "step": samples,
            "type": "ValueError",
            "message": (
                "Actor FSM rejected a V20 active event: "
                '{"adapted_events":[],"invalid_event_type":'
                '"hs_too_early_after_to","state_name":"SWING_AFTER_TO"}'
            ),
        }
        if mode == "fsm_event_rejected"
        else None
    )
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "protocol_id": contract.PROTOCOL_ID,
        "case_id": contract.DAGGER_CASE_IDS[0],
        "dagger_round": 1,
        "dagger_rounds_completed": 1,
        "behavior": "P0_CLOSED_LOOP_V25_BINARY_ACTIVE",
        "teacher": "H0_ONLINE_GRF_DETECTOR_LEGACY_EVENTS_SHADOW_FSM",
        "completion_mode": mode,
        "fsm_event_rejection": rejection,
        "sample_count": samples,
        "persisted_sample_count": samples,
        "control_window_count": successful_steps * 10,
        "v25_raw_sensor_sample_count": successful_steps * 10,
        "candidate_selected_before_teacher_count": samples,
        "served_action_teacher_dependency_count": 0,
        "action_clipped_values": 0,
        "nonfinite_count": 0,
        "grf_penetration_max_m": 0.0249,
        "timeout_count": 0,
        "invalid_event_count": 0,
        "hard_invalid_count": 0,
        "so_solver_unaccepted_count": 0,
        "sea_plugin_fallback_count": 0,
        "routing_failure_count": 0,
        "binary_phase_fsm_mode": "binary_active",
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }


def _development_summary() -> dict:
    case = contract.canonical_case(contract.DEVELOPMENT_CASE_IDS[0])
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "protocol_id": contract.PROTOCOL_ID,
        "case_id": case["case_id"],
        "candidate_id": "candidate-v6",
        "action_selection": case["action_selection"],
        "episode_start_offset_s": case["episode_start_offset_s"],
        "action_seed": case["action_seed"],
        "runtime_seed": case["runtime_seed"],
        "steps": 500,
        "control_window_count": 5000,
        "v25_raw_sensor_sample_count": 5000,
        "end_reason": "episode_time_limit",
        "terminated": False,
        "truncated": True,
        "phase_valid_cycle_count": 2,
        "grf_penetration_max_m": 0.0249,
        "action_clipped_values": 0,
        "fallback_count": 0,
        "timeout_count": 0,
        "safety_stop_count": 0,
        "sea_plugin_fallback_count": 0,
        "so_solver_unaccepted_count": 0,
        "hard_invalid_count": 0,
        "invalid_event_count": 0,
        "nonfinite_count": 0,
        "n_actor": 35,
        "n_observation": 84,
        "observation_dtype": "float32",
        "binary_phase_fsm_mode": "binary_active",
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "morphology_weight": 0.0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }


def test_single_round_stage_order_and_qualification_statuses_align() -> None:
    assert contract.STAGE_IDS[:5] == (
        "fit_p0",
        "collect_dagger__deterministic_offset_minus_0p20",
        "collect_dagger__stochastic_nominal_seed_126",
        "fit_p1",
        "freeze_p1",
    )
    assert contract.STAGE_IDS[-1] == "finalize_development"
    assert len(contract.STAGE_IDS) == 12
    assert contract.CANDIDATE_FREEZE_STATUS == (
        qualification.CANDIDATE_FREEZE_REQUIRED_STATUS
    )
    assert contract.DEVELOPMENT_PASS_STATUS == (
        qualification.DEVELOPMENT_PASS_REQUIRED_STATUS
    )
    assert contract.CANDIDATE_FREEZE_PATH.as_posix().endswith(
        "/adaptation/candidate_freeze.json"
    )
    assert contract.DEVELOPMENT_RECEIPT_PATH.as_posix().endswith(
        "/development/receipt.json"
    )
    assert not contract.AUTHORITY["retry_authorized"]
    assert not contract.AUTHORITY["second_dagger_round_authorized"]
    assert not contract.AUTHORITY["protected_trial_access_authorized"]


@pytest.mark.parametrize(
    ("stage", "epoch", "expected"),
    [
        ("p0", 1, 1.0e-3),
        ("p0", 2000, 1.0e-3),
        ("p0", 2001, 3.0e-4),
        ("p0", 5000, 3.0e-4),
        ("p0", 5001, 1.0e-4),
        ("p0", 8000, 1.0e-4),
        ("p1", 1, 3.0e-4),
        ("p1", 2000, 3.0e-4),
        ("p1", 2001, 1.0e-4),
        ("p1", 4000, 1.0e-4),
    ],
)
def test_learning_rate_schedule_is_exact(stage, epoch, expected) -> None:
    assert contract.learning_rate(stage, epoch) == expected


@pytest.mark.parametrize(
    ("stage", "epoch"),
    [("p0", 0), ("p0", 8001), ("p1", 4001), ("other", 1)],
)
def test_learning_rate_rejects_every_unregistered_epoch(stage, epoch) -> None:
    with pytest.raises(ValueError):
        contract.learning_rate(stage, epoch)


def test_fit_gate_accepts_p0_and_variable_length_one_round_p1_only() -> None:
    p0 = contract.fit_gate(_fit_summary("p0", dagger_samples=0), stage="p0")
    assert p0["passed"] and p0["status"] == contract.P0_PASS_STATUS

    # Both DAgger cases may stop at an accepted FSM rejection only after the
    # preregistered minimum useful trajectory prefix has been persisted.
    minimum = len(contract.DAGGER_CASE_IDS) * contract.MIN_DAGGER_SAMPLES_PER_CASE
    p1 = contract.fit_gate(_fit_summary("p1", dagger_samples=minimum), stage="p1")
    assert p1["passed"] and p1["status"] == contract.P1_PASS_STATUS
    malformed = _fit_summary("p1", dagger_samples=minimum - 1)
    assert not contract.fit_gate(malformed, stage="p1")["passed"]
    drifted = _fit_summary("p0", dagger_samples=0)
    drifted["promotable"] = True
    assert not contract.fit_gate(drifted, stage="p0")["passed"]


def test_dagger_gate_accepts_only_full_or_persisted_fsm_rejection() -> None:
    assert contract.dagger_gate(_dagger_summary())["passed"]
    early = _dagger_summary(
        mode="fsm_event_rejected",
        samples=contract.MIN_DAGGER_SAMPLES_PER_CASE,
    )
    assert contract.dagger_gate(early)["passed"]

    for mutation in (
        {"completion_mode": "runtime_terminal"},
        {"sample_count": 0, "persisted_sample_count": 0},
        {"dagger_rounds_completed": 2},
        {"served_action_teacher_dependency_count": 1},
        {"so_solver_unaccepted_count": 1},
        {"sea_plugin_fallback_count": 1},
        {"fsm_event_rejection": None},
    ):
        value = _dagger_summary(
            mode="fsm_event_rejected",
            samples=contract.MIN_DAGGER_SAMPLES_PER_CASE,
        )
        value.update(mutation)
        assert not contract.dagger_gate(value)["passed"]


def test_development_gate_is_strict_on_penetration_invalids_and_candidate() -> None:
    valid = _development_summary()
    assert contract.development_rollout_gate(valid)["passed"]
    for key, value in (
        ("grf_penetration_max_m", 0.025),
        ("invalid_event_count", 1),
        ("candidate_id", ""),
        ("steps", 499),
    ):
        drifted = copy.deepcopy(valid)
        drifted[key] = value
        assert not contract.development_rollout_gate(drifted)["passed"]


def test_stage_paths_are_unique_and_reject_unknown_ids() -> None:
    claims = [contract.worker_claim_path(stage) for stage in contract.STAGE_IDS]
    receipts = [contract.stage_receipt_path(stage) for stage in contract.STAGE_IDS]
    assert len(set(claims)) == len(claims)
    assert len(set(receipts)) == len(receipts)
    with pytest.raises(ValueError):
        contract.worker_claim_path("fit_p2")
    with pytest.raises(ValueError):
        contract.stage_receipt_path("fit_p2")
