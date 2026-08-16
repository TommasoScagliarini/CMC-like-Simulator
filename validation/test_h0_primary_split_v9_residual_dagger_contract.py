from __future__ import annotations

import copy

import h0_primary_split_v9_residual_dagger_contract as contract


def _fit_summary(stage: str) -> dict[str, object]:
    dagger = 0 if stage == "p0" else 1000
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "protocol_id": contract.PROTOCOL_ID,
        "fit_stage": stage,
        "fit": contract.P0_FIT if stage == "p0" else contract.P1_FIT,
        "sample_count": 3000 + dagger,
        "teacher_sample_count": 3000,
        "dagger_sample_count": dagger,
        "reset_row_count": 6 if stage == "p0" else 8,
        "metrics": {
            "rmse": 0.0014,
            "max_abs_error": 0.024,
            "reset_max_abs_error": 9.0e-5,
        },
        "all_finite": True,
        "base_h0_byte_exact": True,
        "critic_byte_exact": True,
        "logstd_byte_exact": True,
        "residual_limits": list(contract.RESIDUAL_LIMITS),
        "residual_input_indices": list(contract.RESIDUAL_INPUT_INDICES),
        "normalization_frozen": True,
        "optimizer_steps": (contract.P0_FIT if stage == "p0" else contract.P1_FIT)[
            "epochs"
        ],
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "promotable": stage == "p1",
    }


def test_v9_contract_freezes_causal_semantics_and_data_driven_limits() -> None:
    assert contract.SCHEMA_VERSION == 9
    assert contract.TEACHER_ID == "H0_ANALOG_LOAD_CONTACT_WITH_V26_EVENTS_FSM_V1"
    assert contract.EVENT_CONTRACT_ID.endswith("heel_qualified_fsm_v2")
    assert contract.RESIDUAL_LIMITS == (0.175, 0.15)
    assert contract.OFFLINE_THRESHOLDS == {
        "rmse_max": 0.0015,
        "max_abs_error_max": 0.025,
        "reset_max_abs_error_max": 1.0e-4,
    }
    assert contract.AUTHORITY["legacy_event_or_fsm_teacher_authorized"] is False


def test_fit_gate_uses_fresh_thresholds_without_reinterpreting_v8() -> None:
    summary = _fit_summary("p0")
    assert contract.fit_gate(summary, stage="p0")["passed"] is True
    for metric, value in (
        ("rmse", 0.0015001),
        ("max_abs_error", 0.025001),
        ("reset_max_abs_error", 0.0001001),
    ):
        failed = copy.deepcopy(summary)
        failed["metrics"][metric] = value
        assert contract.fit_gate(failed, stage="p0")["passed"] is False


def test_stage_order_requires_two_full_dagger_cases_before_p1() -> None:
    assert contract.STAGE_IDS[:4] == (
        "fit_p0",
        "collect_dagger__deterministic_offset_minus_0p20",
        "collect_dagger__stochastic_nominal_seed_126",
        "fit_p1",
    )
    assert contract.MIN_DAGGER_SAMPLES_PER_CASE == 500
    assert len(contract.DEVELOPMENT_CASE_IDS) == 6
