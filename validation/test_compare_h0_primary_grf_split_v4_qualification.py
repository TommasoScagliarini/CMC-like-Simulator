from __future__ import annotations

import copy
import unittest

from validation import compare_h0_primary_grf_split_v4_qualification as gates
from validation import h0_primary_grf_split_v4_qualification_contract as contract


def _metric(value: float, samples: int = 500) -> dict[str, float | int]:
    return {"sample_count": samples, "rms": value, "abs_max": value * 1.1}


def _summary(role: str, case_id: str) -> dict[str, object]:
    case = next(item for item in contract.canonical_cases() if item["case_id"] == case_id)
    sea = {}
    for joint in contract.JOINTS:
        sea[joint] = {
            signal: _metric(
                1.0,
                samples=5000 if signal != "tau_spring_rate_nm_s" else 4500,
            )
            for signal in contract.SEA_SIGNALS
        }
        sea[joint]["tau_input_saturated"] = {
            "sample_count": 5000,
            "count": 0,
            "fraction": 0.0,
        }
    result: dict[str, object] = {
        "schema_version": contract.SCHEMA_VERSION,
        "role": role,
        "case_id": case_id,
        "action_selection": case["action_selection"],
        "episode_start_offset_s": case["episode_start_offset_s"],
        "action_seed": case["action_seed"],
        "runtime_seed": case["runtime_seed"],
        "sigma": case["sigma"],
        "noise_tape_sha256": "a" * 64,
        "steps": 500,
        "end_reason": "episode_time_limit",
        "terminated": False,
        "truncated": True,
        "phase_valid_cycle_count": 3,
        "invalid_event_count": 1,
        "grf_penetration_max_m": 0.024,
        "action_clipped_values": 0,
        "fallback_count": 0,
        "raw_so_fallback_count": 3,
        "policy_step_terminal_so_fallback_count": 1,
        "timeout_count": 0,
        "safety_stop_count": 0,
        "sea_plugin_fallback_count": 0,
        "hard_invalid_count": 0,
        "nonfinite_count": 0,
        "so_policy_id": contract.SO_POLICY_ID,
        "so_solver_control_window_count": 5000,
        "so_solver_bounded_ls_invocation_count": 3,
        "so_solver_verified_bounded_ls_count": 3,
        "so_solver_verified_status0_max_iter_count": 2,
        "so_solver_unaccepted_hard_fallback_count": 0,
        "so_solver_unaccepted_bounded_ls_count": 0,
        "so_solver_hard_fallback_count": 2,
        "so_solver_reuse_previous_count": 0,
        "so_solver_bounded_ls_unsuccessful_count": 2,
        "so_solver_bounds_violation_count": 0,
        "so_solver_nonfinite_count": 0,
        "so_solver_selected_infeasible_count": 0,
        "so_solver_selected_solution_mismatch_count": 0,
        "so_solver_residual_contract_mismatch_count": 0,
        "n_actor": 35,
        "n_observation": 84,
        "observation_dtype": "float32",
        "action_shape": [2],
        "action_dtype": "float32",
        "actor_input_view": "historical_analog" if role == "baseline" else "primary_split",
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "phase_fsm_input_mode": contract.PHASE_FSM_INPUT_MODE,
        "binary_phase_fsm_mode": "disabled",
        "online_grf_applied_sides": ["left"],
        "morphology_weight": 0.0,
        "episode_metrics": {
            "reserve_norm_nm": _metric(100.0),
            "residual_norm_nm": _metric(1.0),
        },
        "sea_episode_metrics": sea,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    return result


class QualificationComparatorTests(unittest.TestCase):
    CASE = contract.CASE_IDS[1]

    def test_verified_status0_is_traced_and_accepted(self) -> None:
        result = gates.common_rollout_gate(
            _summary("baseline", self.CASE),
            role="baseline",
            case_id=self.CASE,
        )
        self.assertTrue(result["passed"])
        self.assertEqual(result["status"], contract.ROLLOUT_PASS_STATUS)
        checks = {item["name"]: item for item in result["checks"]}
        self.assertEqual(checks["raw_so_fallback_count_traced"]["status"], "PASS")
        self.assertEqual(checks["status0_accounts_for_hard"]["status"], "PASS")

    def test_unaccepted_bounded_ls_closes_common_gate(self) -> None:
        summary = _summary("candidate", self.CASE)
        summary["fallback_count"] = 1
        summary["so_solver_unaccepted_bounded_ls_count"] = 1
        result = gates.common_rollout_gate(
            summary,
            role="candidate",
            case_id=self.CASE,
        )
        self.assertFalse(result["passed"])
        self.assertEqual(result["status"], contract.FAIL_STATUS)

    def test_condition_matched_caps_are_applied(self) -> None:
        baseline = _summary("baseline", self.CASE)
        candidate = _summary("candidate", self.CASE)
        candidate["episode_metrics"]["reserve_norm_nm"]["rms"] = 104.9
        candidate["episode_metrics"]["reserve_norm_nm"]["abs_max"] = 110.0
        result = gates.condition_matched_gate(
            baseline,
            candidate,
            case_id=self.CASE,
        )
        self.assertTrue(result["passed"])
        self.assertEqual(result["status"], contract.CASE_PASS_STATUS)
        failing = copy.deepcopy(candidate)
        failing["episode_metrics"]["reserve_norm_nm"]["rms"] = 105.1
        failing["episode_metrics"]["reserve_norm_nm"]["abs_max"] = 110.0
        result = gates.condition_matched_gate(
            baseline,
            failing,
            case_id=self.CASE,
        )
        self.assertFalse(result["passed"])

    def test_error_counts_cannot_exceed_baseline(self) -> None:
        baseline = _summary("baseline", self.CASE)
        candidate = _summary("candidate", self.CASE)
        candidate["invalid_event_count"] = 2
        result = gates.condition_matched_gate(
            baseline,
            candidate,
            case_id=self.CASE,
        )
        self.assertFalse(result["passed"])

    def test_baseline_metric_schema_matches_frozen_order(self) -> None:
        metrics = gates.baseline_case_metrics(_summary("baseline", self.CASE))
        self.assertEqual(
            list(metrics["sea"]),
            [row[0] for row in contract.SEA_TOLERANCES],
        )
        self.assertEqual(
            list(metrics["reserve"]),
            [row[0] for row in contract.RESERVE_TOLERANCES],
        )


if __name__ == "__main__":
    unittest.main()
