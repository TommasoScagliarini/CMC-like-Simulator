"""Unit tests for the inert V12R7 recovery contract."""

from __future__ import annotations

import copy
import os
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
if os.fspath(ROOT) not in sys.path:
    sys.path.insert(0, os.fspath(ROOT))

import h0_v12r7_recovery_contract as contract  # noqa: E402


def test_self_check_and_lineage_are_strict() -> None:
    gate = contract.contract_self_check()
    assert gate["passed"] is True
    assert all(gate["checks"].values())
    assert contract.AUTHORITY["retry_authorized"] is False
    assert contract.AUTHORITY["alpha_sweep_authorized"] is False
    assert contract.FIT["architecture"]["hidden_dims"] == [512, 512]
    assert contract.FIT["actor_fit_count"] == 1


def test_cases_are_copy_safe_and_critical_first() -> None:
    assert contract.COLLECTION_CASE_IDS[0] == "deterministic_offset_plus_0p20"
    case = contract.canonical_collection_case(contract.COLLECTION_CASE_IDS[0])
    case["case_id"] = "changed"
    assert (
        contract.canonical_collection_case(contract.COLLECTION_CASE_IDS[0])["case_id"]
        == contract.COLLECTION_CASE_IDS[0]
    )


def _collection_summary(*, recoverable: bool = True) -> dict[str, object]:
    steps = 179 if recoverable else 500
    return {
        "case_id": contract.COLLECTION_CASE_IDS[0],
        "candidate_tree_sha256": contract.LOCKED_INPUTS["r6_candidate"]["tree_sha256"],
        "steps": steps,
        "terminated": recoverable,
        "truncated": not recoverable,
        "end_reason": "grf_penetration" if recoverable else "episode_time_limit",
        "teacher_enabled": False,
        "teacher_query_count": 0,
        "blending_enabled": False,
        "safety_latch_enabled": False,
        "trace_step_count": steps,
        "replay_step_count": steps,
        "replay_boundary_count": steps + 1,
        "raw_sensor_sample_count": steps * 10,
        "fallback_count": 0,
        "hard_invalid_count": 0,
        "duplicate_event_count": 0,
        "out_of_order_event_count": 0,
        "left_non_v26_source_count": 0,
        "action_clipped_values": 0,
        "nonfinite_count": 0,
        "sea_plugin_fallback_count": 0,
        "so_solver_unaccepted_count": 0,
    }


def test_collection_accepts_integral_prefix_but_rejects_bool_counter() -> None:
    summary = _collection_summary()
    gate = contract.collection_integrity_gate(summary)
    assert gate["passed"] is True
    assert gate["recoverable_prefix"] is True

    tampered = copy.deepcopy(summary)
    tampered["steps"] = True
    assert contract.collection_integrity_gate(tampered)["passed"] is False


def test_collection_accepts_full_horizon_and_rejects_teacher() -> None:
    summary = _collection_summary(recoverable=False)
    assert contract.collection_integrity_gate(summary)["passed"] is True
    summary["teacher_query_count"] = 1
    assert contract.collection_integrity_gate(summary)["passed"] is False


def _metric() -> dict[str, float]:
    return {"rmse": 0.002, "max_abs_error": 0.02}


def test_fit_and_development_gates() -> None:
    fit = {
        "actor_fit_count": 1,
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "hidden_dims": [512, 512],
        "actor_feature_count": 35,
        "logstd_byte_exact": True,
        "disabled_clock_columns_bit_zero": True,
        "save_reload_exact": True,
        "global_metrics": _metric(),
        "reset_max_abs_error": 0.001,
        "per_case_metrics": {case: _metric() for case in contract.COLLECTION_CASE_IDS},
        "r4_failed_plus_metrics": _metric(),
        "observer_case_metrics": {
            case: _metric() for case in contract.COLLECTION_CASE_IDS
        },
        "observer_plus_late_metrics": _metric(),
    }
    assert contract.fit_gate(fit)["passed"] is True
    fit["actor_fit_count"] = True
    assert contract.fit_gate(fit)["passed"] is False

    case_id = contract.DEVELOPMENT_CASE_IDS[0]
    development = {
        "case_id": case_id,
        "steps": 500,
        "terminated": False,
        "truncated": True,
        "end_reason": "episode_time_limit",
        "phase_valid_cycle_count": 2,
        "grf_penetration_max_m": 0.024,
        "teacher_enabled": False,
        "teacher_query_count": 0,
        "blending_enabled": False,
        "safety_latch_enabled": False,
        "action_clipped_values": 0,
        "nonfinite_count": 0,
        "fallback_count": 0,
        "hard_invalid_count": 0,
        "duplicate_event_count": 0,
        "out_of_order_event_count": 0,
        "left_non_v26_source_count": 0,
        "sea_plugin_fallback_count": 0,
        "so_solver_unaccepted_count": 0,
    }
    assert contract.development_gate(development, case_id=case_id)["passed"] is True
    development["grf_penetration_max_m"] = 0.025
    assert contract.development_gate(development, case_id=case_id)["passed"] is False
