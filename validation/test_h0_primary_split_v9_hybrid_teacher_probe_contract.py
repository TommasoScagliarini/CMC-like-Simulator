from __future__ import annotations

import copy

import h0_primary_split_v9_hybrid_teacher_probe_contract as contract


def _passing_summary() -> dict[str, object]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.ROLLOUT_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "probe_id": contract.PROBE_ID,
        "case_id": contract.CASE_ID,
        "action_selection": "deterministic",
        "episode_start_offset_s": contract.CASE["episode_start_offset_s"],
        "runtime_seed": contract.CASE["runtime_seed"],
        "behavior": "H0_HYBRID_TEACHER_MEAN_CLOSED_LOOP_V26_BINARY_ACTIVE",
        "teacher_id": contract.TEACHER_ID,
        "teacher_privileged_indices": [10, 11],
        "binary_phase_fsm_mode": "binary_active",
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "morphology_weight": 0.0,
        "steps": 500,
        "raw_sensor_sample_count": 5000,
        "control_window_count": 5000,
        "end_reason": "episode_time_limit",
        "terminated": False,
        "truncated": True,
        "phase_valid_cycle_count": 2,
        "grf_penetration_max_m": 0.024999,
        "causal_column_mismatch_count": 0,
        "teacher_query_mismatch_count": 0,
        "action_clipped_values": 0,
        "nonfinite_count": 0,
        "timeout_count": 0,
        "invalid_event_count": 0,
        "hard_invalid_count": 0,
        "so_solver_unaccepted_count": 0,
        "sea_plugin_fallback_count": 0,
        "routing_failure_count": 0,
        "step_contract_failure_count": 0,
        "binary_event_failure_count": 0,
        "safety_stop_count": 0,
        "n_actor": 35,
        "n_observation": 84,
        "observation_dtype": "float32",
        "actor_feature_names": list(contract.EXPECTED_ACTOR_FEATURE_NAMES),
        "observation_feature_names": list(
            contract.EXPECTED_OBSERVATION_FEATURE_NAMES
        ),
        "candidate_created": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def test_contract_is_single_case_zero_update_and_only_10_11_privileged() -> None:
    assert contract.CASE_IDS == ("deterministic_offset_minus_0p20",)
    assert contract.canonical_case(contract.CASE_ID)["action_selection"] == (
        "deterministic"
    )
    assert contract.PRIVILEGED_INDICES == (10, 11)
    assert contract.AUTHORITY["authority_text"] == (
        "ok allora procedi con la risoluzione"
    )
    assert contract.AUTHORITY["zero_update_probe"] is True
    assert contract.AUTHORITY["one_shot_no_retry"] is True
    assert contract.AUTHORITY["protected_trial_access_authorized"] is False


def test_rollout_gate_accepts_only_complete_strict_pass() -> None:
    gate = contract.rollout_gate(_passing_summary())
    assert gate["passed"] is True
    assert all(gate["checks"].values())


def test_rollout_gate_rejects_boundary_penetration_partial_or_updated() -> None:
    for field, value, check in (
        ("grf_penetration_max_m", 0.025, "penetration"),
        ("steps", 499, "steps"),
        ("phase_valid_cycle_count", 1, "cycles"),
        ("actor_updates", 1, "no_updates"),
        ("teacher_privileged_indices", [10, 11, 12], "only_10_11_privileged"),
    ):
        summary = copy.deepcopy(_passing_summary())
        summary[field] = value
        gate = contract.rollout_gate(summary)
        assert gate["passed"] is False
        assert gate["checks"][check] is False


def test_unknown_case_fails_closed() -> None:
    try:
        contract.canonical_case("deterministic_offset_nominal")
    except ValueError:
        pass
    else:  # pragma: no cover - explicit fail-closed assertion.
        raise AssertionError("unknown case accepted")
