from __future__ import annotations

import copy
import unittest

from validation import compare_h0_v3_v25_abc_post_zero_port as subject


SHA = "a" * 64


def _solution() -> dict:
    return {
        "output_shape_matches": True,
        "output_finite": True,
        "output_sha256": SHA,
        "bound_violation_max": 0.0,
        "equality_residual_finite": True,
        "equality_residual_norm": 1.0e-8,
        "equality_residual_max_abs": 8.0e-9,
        "equality_residual_relative_norm": 2.0e-10,
    }


def _window(index: int) -> dict:
    solution = _solution()
    return {
        "control_window_index": index,
        "control_window_time_s": index * 0.001,
        "selected_feasibility_attempt_index": 1,
        "served_solution_sha256": SHA,
        "selected_solver_solution_matches_served": True,
        "attempts": [
            {
                "attempt_index": 1,
                "feasibility_scale": 1.0,
                "feasibility_accepted": True,
                "residual_norm": 1.0e-8,
                "residual_relative_norm": 2.0e-10,
                "residual_max_abs": 8.0e-9,
                "solver_fallback_used": True,
                "selected": True,
                "solver_path": {
                    "schema": subject.so_recovery.SCHEMA,
                    "primary_solver": "slsqp",
                    "input_matrix_finite": True,
                    "input_target_finite": True,
                    "weights_finite": True,
                    "bounds_finite": True,
                    "warm_start_finite": True,
                    "input_matrix_sha256": SHA,
                    "input_target_sha256": SHA,
                    "weights_sha256": SHA,
                    "bounds_sha256": SHA,
                    "warm_start_sha256": SHA,
                    "slsqp_invoked": True,
                    "slsqp_success": False,
                    "slsqp_status": 8,
                    "slsqp_iterations": 10,
                    "slsqp_solution": solution,
                    "bounded_lsq_used": True,
                    "bounded_lsq_invoked": True,
                    "bounded_lsq_success": False,
                    "bounded_lsq_status": 0,
                    "bounded_lsq_iterations": 1000,
                    "bounded_lsq_message": subject.so_recovery.BOUNDED_LSQ_MAX_ITER_MESSAGE,
                    "bounded_lsq_cost": 1.0e-12,
                    "bounded_lsq_optimality": 2.0e-9,
                    "bounded_lsq_solution": solution,
                    "reuse_previous_solution": False,
                    "selected_solution": solution,
                    "hard_fallback": True,
                },
            }
        ],
    }


def _journal() -> list[dict]:
    windows = [_window(index) for index in range(1, 11)]
    return [
        {
            "step": step,
            "time_s": step * 0.01,
            "control_windows": copy.deepcopy(windows),
        }
        for step in range(1, 501)
    ]


def _summary() -> dict:
    summary = {
        "so_policy_id": subject.contract.SO_POLICY_ID,
        "fallback_count": 500,
        "so_fallback_count": 500,
        "sea_plugin_fallback_count": 0,
    }
    values = {
        "control_window_count": 5000,
        "solver_invocation_count": 5000,
        "primary_solver_nonconvergence_count": 5000,
        "bounded_ls_invocation_count": 5000,
        "selected_bounded_ls_count": 5000,
        "verified_bounded_ls_count": 5000,
        "verified_bounded_ls_success_count": 0,
        "verified_status0_max_iter_count": 5000,
        "unaccepted_hard_so_fallback_count": 0,
        "unaccepted_bounded_ls_count": 0,
        "hard_so_fallback_count": 5000,
        "reuse_previous_count": 0,
        "bounded_ls_unsuccessful_count": 5000,
        "bounds_violation_count": 0,
        "nonfinite_solver_count": 0,
        "selected_infeasible_count": 0,
        "selected_solution_mismatch_count": 0,
        "residual_contract_mismatch_count": 0,
    }
    summary.update({subject.SUMMARY_COUNTERS[key]: value for key, value in values.items()})
    return summary


class PostZeroPortComparatorTests(unittest.TestCase):
    def test_all_5000_verified_status0_windows_pass(self) -> None:
        result = subject.classify_solver_journal(_journal(), _summary())
        self.assertTrue(result["passed"])
        self.assertEqual(result["totals"]["control_window_count"], 5000)
        self.assertEqual(result["raw_fallback_count"], 500)

    def test_one_optimality_violation_fails_closed(self) -> None:
        journal = _journal()
        journal[0]["control_windows"][0] = copy.deepcopy(
            journal[0]["control_windows"][0]
        )
        journal[0]["control_windows"][0]["attempts"][0]["solver_path"][
            "bounded_lsq_optimality"
        ] = 2.0e-8
        summary = _summary()
        summary["so_solver_verified_bounded_ls_count"] = 4999
        summary["so_solver_verified_status0_max_iter_count"] = 4999
        summary["so_solver_unaccepted_hard_fallback_count"] = 1
        summary["so_solver_unaccepted_bounded_ls_count"] = 1
        result = subject.classify_solver_journal(journal, summary)
        self.assertFalse(result["passed"])
        self.assertEqual(result["totals"]["unaccepted_bounded_ls_count"], 1)

    def test_solver_journal_must_cover_exactly_500_steps(self) -> None:
        with self.assertRaisesRegex(subject.PostZeroPortGateError, "500 policy"):
            subject.classify_solver_journal(_journal()[:-1], _summary())


if __name__ == "__main__":
    unittest.main()
