from __future__ import annotations

import copy

import h0_primary_split_v8_residual_dagger_contract as contract


def _valid_dagger_summary() -> dict[str, object]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "protocol_id": contract.PROTOCOL_ID,
        "case_id": contract.DAGGER_CASE_IDS[0],
        "dagger_round": 1,
        "dagger_rounds_completed": 1,
        "behavior": "P0_CLOSED_LOOP_V25_BINARY_ACTIVE",
        "teacher": "H0_ONLINE_GRF_DETECTOR_LEGACY_EVENTS_SHADOW_FSM",
        "completion_mode": "episode_time_limit",
        "fsm_event_rejection": None,
        "sample_count": 500,
        "persisted_sample_count": 500,
        "candidate_selected_before_teacher_count": 500,
        "served_action_teacher_dependency_count": 0,
        "action_clipped_values": 0,
        "nonfinite_count": 0,
        "control_window_count": 5000,
        "v25_raw_sensor_sample_count": 5000,
        "timeout_count": 0,
        "invalid_event_count": 0,
        "hard_invalid_count": 0,
        "so_solver_unaccepted_count": 0,
        "sea_plugin_fallback_count": 0,
        "routing_failure_count": 0,
        "terminated": False,
        "truncated": True,
        "end_reason": "episode_time_limit",
        "phase_valid_cycle_count": 2,
        "grf_penetration_max_m": 0.024,
        "binary_phase_fsm_mode": "binary_active",
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "morphology_weight": 0.0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }


def test_v8r1p1_contract_is_fresh_v26_lineage() -> None:
    assert contract.SCHEMA_VERSION == 8
    assert "V8R1P1_V26" in contract.PROTOCOL_ID
    assert contract.EVENT_CONTRACT_ID == (
        "binary_point_v25+heel_qualified_fsm_v2"
    )
    assert contract.TARGET_CONTRACT_ID == (
        "primary_grf_split_v1+binary_point_v25+heel_qualified_fsm_v2"
    )
    assert "2026-08-07_h0_primary_split_v8r1p1_v26_residual" in str(
        contract.RUN_ROOT
    )
    assert "v6_v25_residual" not in str(contract.RUN_ROOT)
    assert contract.AUTHORITY["fresh_v8r1p1_lineage_authorized"] is True
    assert contract.AUTHORITY["partial_fsm_rejection_acceptance_authorized"] is False
    assert contract.MIN_DAGGER_SAMPLES_PER_CASE == contract.EXPECTED_STEPS == 500


def test_cases_and_worker_paths_are_fresh_copies() -> None:
    assert len(contract.DEVELOPMENT_CASE_IDS) == 6
    assert len(contract.DAGGER_CASE_IDS) == 2
    first = contract.canonical_case(contract.DEVELOPMENT_CASE_IDS[0])
    first["runtime_seed"] = -1
    assert contract.canonical_case(contract.DEVELOPMENT_CASE_IDS[0])["runtime_seed"] != -1
    assert len(contract.STAGE_IDS) == 12
    assert contract.worker_claim_path("fit_p0").name.startswith("01_")
    assert contract.stage_receipt_path("finalize_development") == (
        contract.DEVELOPMENT_RECEIPT_PATH
    )


def test_dagger_gate_requires_full_500_step_v26_episode() -> None:
    valid = _valid_dagger_summary()
    result = contract.dagger_gate(valid)
    assert result["passed"] is True
    assert result["status"] == contract.DAGGER_PASS_STATUS

    shortened = copy.deepcopy(valid)
    shortened.update(
        {
            "completion_mode": "fsm_event_rejected",
            "fsm_event_rejection": {
                "step": 300,
                "type": "ValueError",
                "message": "historical V20 partial path",
            },
            "sample_count": 300,
            "persisted_sample_count": 300,
            "candidate_selected_before_teacher_count": 300,
            "control_window_count": 2990,
            "v25_raw_sensor_sample_count": 2990,
            "terminated": False,
            "truncated": False,
            "end_reason": None,
        }
    )
    failed = contract.dagger_gate(shortened)
    assert failed["passed"] is False
    assert failed["checks"]["full_episode"] is False
    assert failed["checks"]["no_partial_rejection"] is False


def test_dagger_gate_rejects_old_event_contract() -> None:
    summary = _valid_dagger_summary()
    summary["event_contract_id"] = "binary_point_v25+functional_contact_fsm_v1"
    result = contract.dagger_gate(summary)
    assert result["passed"] is False
    assert result["checks"]["event_contract"] is False


def test_fit_p1_requires_exactly_two_full_dagger_rollouts() -> None:
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "protocol_id": contract.PROTOCOL_ID,
        "fit_stage": "p1",
        "fit": contract.P1_FIT,
        "sample_count": 4000,
        "teacher_sample_count": 3000,
        "dagger_sample_count": 1000,
        "reset_row_count": 8,
        "metrics": {
            "rmse": 0.001,
            "max_abs_error": 0.01,
            "reset_max_abs_error": 1.0e-6,
        },
        "all_finite": True,
        "base_h0_byte_exact": True,
        "critic_byte_exact": True,
        "logstd_byte_exact": True,
        "residual_limits": list(contract.RESIDUAL_LIMITS),
        "residual_input_indices": list(contract.RESIDUAL_INPUT_INDICES),
        "normalization_frozen": True,
        "optimizer_steps": contract.P1_FIT["epochs"],
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "promotable": True,
    }
    assert contract.fit_gate(summary, stage="p1")["passed"] is True
    summary["dagger_sample_count"] = 999
    summary["sample_count"] = 3999
    assert contract.fit_gate(summary, stage="p1")["passed"] is False
