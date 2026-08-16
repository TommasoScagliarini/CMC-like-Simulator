from __future__ import annotations

import copy
import unittest

from validation import h0_v3_so_recovery_contract as contract


SHA = "a" * 64


def _solution(*, bound_violation: float = 0.0) -> dict:
    return {
        "output_shape_matches": True,
        "output_finite": True,
        "output_sha256": SHA,
        "bound_violation_max": bound_violation,
        "equality_residual_finite": True,
        "equality_residual_norm": 1.0e-8,
        "equality_residual_max_abs": 8.0e-9,
        "equality_residual_relative_norm": 2.0e-10,
    }


def _bounded_path() -> dict:
    return {
        "schema": contract.SCHEMA,
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
        "slsqp_iterations": 17,
        "slsqp_solution": _solution(),
        "bounded_lsq_used": True,
        "bounded_lsq_invoked": True,
        "bounded_lsq_success": True,
        "bounded_lsq_status": 1,
        "bounded_lsq_iterations": 4,
        "bounded_lsq_message": "The first-order optimality measure is less than `tol`.",
        "bounded_lsq_cost": 1.0e-12,
        "bounded_lsq_optimality": 1.0e-9,
        "bounded_lsq_solution": _solution(),
        "reuse_previous_solution": False,
        "selected_solution": _solution(),
        "hard_fallback": False,
    }


def _entries() -> list[dict]:
    return [
        {
            "control_window_index": 1,
            "control_window_time_s": 13.95,
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
                    "solver_path": _bounded_path(),
                }
            ],
        }
    ]


def _status0_entries() -> list[dict]:
    entries = _entries()
    path = entries[0]["attempts"][0]["solver_path"]
    path.update(
        {
            "bounded_lsq_success": False,
            "bounded_lsq_status": contract.BOUNDED_LSQ_STATUS0,
            "bounded_lsq_iterations": contract.BOUNDED_LSQ_MAX_ITER,
            "bounded_lsq_message": contract.BOUNDED_LSQ_MAX_ITER_MESSAGE,
            "bounded_lsq_optimality": 2.9e-9,
            "hard_fallback": True,
        }
    )
    return entries


class SORecoveryContractTests(unittest.TestCase):
    def test_verified_bounded_recovery_is_not_a_hard_fallback(self) -> None:
        result = contract.classify_policy_step(_entries())
        counters = result["counters"]
        self.assertEqual(counters["bounded_ls_invocation_count"], 1)
        self.assertEqual(counters["selected_bounded_ls_count"], 1)
        self.assertEqual(counters["verified_bounded_ls_count"], 1)
        self.assertEqual(counters["hard_so_fallback_count"], 0)
        self.assertTrue(counters["all_bounded_ls_verified"])
        self.assertTrue(counters["hard_conditions_zero"])

    def test_status0_is_accepted_only_by_the_closed_status0_policy(self) -> None:
        strict = contract.classify_policy_step(_status0_entries())
        self.assertEqual(strict["counters"]["verified_bounded_ls_count"], 0)
        self.assertEqual(strict["counters"]["unaccepted_hard_so_fallback_count"], 1)
        self.assertFalse(strict["counters"]["hard_conditions_zero"])

        accepted = contract.classify_policy_step(
            _status0_entries(),
            policy_id=contract.VERIFIED_STATUS0_MAX_ITER_POLICY,
        )
        counters = accepted["counters"]
        self.assertEqual(counters["verified_status0_max_iter_count"], 1)
        self.assertEqual(counters["verified_bounded_ls_count"], 1)
        self.assertEqual(counters["unaccepted_hard_so_fallback_count"], 0)
        self.assertEqual(counters["unaccepted_bounded_ls_count"], 0)
        self.assertTrue(counters["all_bounded_ls_verified"])
        self.assertTrue(counters["hard_conditions_zero"])

    def test_status0_policy_rejects_any_signature_or_limit_drift(self) -> None:
        mutations = {
            "bounded_lsq_status": 1,
            "bounded_lsq_iterations": contract.BOUNDED_LSQ_MAX_ITER - 1,
            "bounded_lsq_message": "different",
            "bounded_lsq_optimality": 1.0001e-8,
            "reuse_previous_solution": True,
        }
        for field, value in mutations.items():
            with self.subTest(field=field):
                entries = _status0_entries()
                entries[0]["attempts"][0]["solver_path"][field] = value
                result = contract.classify_policy_step(
                    entries,
                    policy_id=contract.VERIFIED_STATUS0_MAX_ITER_POLICY,
                )
                self.assertEqual(
                    result["counters"]["verified_status0_max_iter_count"], 0
                )
                self.assertFalse(result["counters"]["hard_conditions_zero"])

    def test_unknown_policy_is_rejected(self) -> None:
        with self.assertRaisesRegex(
            contract.SORecoveryContractError, "unsupported SO recovery policy"
        ):
            contract.classify_policy_step(_entries(), policy_id="unknown")

    def test_boolean_integer_is_rejected(self) -> None:
        entries = _entries()
        entries[0]["attempts"][0]["solver_fallback_used"] = 1
        with self.assertRaisesRegex(contract.SORecoveryContractError, "Python bool"):
            contract.classify_policy_step(entries)

    def test_bounds_violation_fails_hard_conditions(self) -> None:
        entries = _entries()
        entries[0]["attempts"][0]["solver_path"]["selected_solution"] = _solution(
            bound_violation=1.0e-4
        )
        result = contract.classify_policy_step(entries)
        self.assertEqual(result["counters"]["bounds_violation_count"], 1)
        self.assertFalse(result["counters"]["hard_conditions_zero"])

    def test_selected_hash_mismatch_is_counted(self) -> None:
        entries = _entries()
        entries[0]["served_solution_sha256"] = "b" * 64
        result = contract.classify_policy_step(entries)
        self.assertEqual(result["counters"]["selected_solution_mismatch_count"], 1)
        self.assertFalse(result["counters"]["hard_conditions_zero"])

    def test_feasibility_flag_is_recomputed_from_frozen_tolerances(self) -> None:
        entries = _entries()
        entries[0]["attempts"][0]["feasibility_accepted"] = False
        result = contract.classify_policy_step(entries)
        self.assertEqual(result["counters"]["residual_contract_mismatch_count"], 1)
        self.assertFalse(result["counters"]["hard_conditions_zero"])

    def test_attempt_and_solver_residuals_must_match(self) -> None:
        entries = _entries()
        entries[0]["attempts"][0]["residual_norm"] = 5.0e-7
        result = contract.classify_policy_step(entries)
        self.assertEqual(result["counters"]["residual_contract_mismatch_count"], 1)
        self.assertFalse(result["counters"]["hard_conditions_zero"])

    def test_numeric_strings_are_rejected(self) -> None:
        entries = _entries()
        entries[0]["attempts"][0]["residual_norm"] = "1e-8"
        with self.assertRaisesRegex(contract.SORecoveryContractError, "must be finite"):
            contract.classify_policy_step(entries)

    def test_duplicate_or_wrong_selected_attempt_is_rejected(self) -> None:
        entries = _entries()
        second = copy.deepcopy(entries[0]["attempts"][0])
        second["attempt_index"] = 2
        entries[0]["attempts"].append(second)
        with self.assertRaisesRegex(
            contract.SORecoveryContractError, "exactly one selected"
        ):
            contract.classify_policy_step(entries)


if __name__ == "__main__":
    unittest.main(verbosity=2)
