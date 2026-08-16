"""Numerical-invariance tests for the static-optimization solver audit."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from scipy.optimize import lsq_linear, minimize


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from static_optimization import StaticOptimizer  # noqa: E402


def _optimizer() -> StaticOptimizer:
    optimizer = StaticOptimizer.__new__(StaticOptimizer)
    optimizer._weights = np.asarray([1.0, 4.0], dtype=float)
    optimizer._bounds = [(-1.0, 1.0), (-0.5, 0.5)]
    optimizer._x_prev = np.asarray([0.1, -0.1], dtype=float)
    optimizer._cfg = SimpleNamespace(qp_max_iter=100)
    optimizer._solver_path_diagnostics = {
        "input_matrix_finite": True,
        "input_target_finite": True,
        "weights_finite": True,
        "bounds_finite": True,
        "warm_start_finite": True,
        "bounded_lsq_used": False,
        "reuse_previous_solution": False,
    }
    return optimizer


class StaticOptimizationSolverAuditTests(unittest.TestCase):
    def test_bounded_lsq_audit_does_not_change_solution(self) -> None:
        optimizer = _optimizer()
        matrix = np.asarray([[1.0, 2.0], [0.5, -1.0]], dtype=float)
        target = np.asarray([0.4, 0.1], dtype=float)
        scale = np.sqrt(optimizer._weights)
        expected = lsq_linear(
            matrix / scale[np.newaxis, :],
            target,
            bounds=(
                np.asarray([-1.0, -0.5]) * scale,
                np.asarray([1.0, 0.5]) * scale,
            ),
            lsmr_tol="auto",
            max_iter=optimizer._cfg.qp_max_iter,
        ).x / scale

        actual = optimizer._solve_bounded_least_squares(matrix, target)

        np.testing.assert_array_equal(actual, expected)
        audit = optimizer._solver_path_diagnostics
        self.assertIs(audit["bounded_lsq_used"], True)
        self.assertIs(audit["bounded_lsq_invoked"], True)
        self.assertIs(audit["bounded_lsq_success"], True)
        self.assertIs(audit["reuse_previous_solution"], False)
        self.assertIs(audit["bounded_lsq_solution"]["output_finite"], True)
        self.assertLessEqual(
            audit["bounded_lsq_solution"]["bound_violation_max"], 1.0e-12
        )

    def test_nonfinite_input_reuses_previous_and_is_explicit(self) -> None:
        optimizer = _optimizer()
        previous = optimizer._x_prev.copy()
        with self.assertWarnsRegex(RuntimeWarning, "Non-finite SO inputs"):
            actual = optimizer._solve_bounded_least_squares(
                np.asarray([[np.nan, 0.0]], dtype=float),
                np.asarray([0.0], dtype=float),
            )

        np.testing.assert_array_equal(actual, previous)
        audit = optimizer._solver_path_diagnostics
        self.assertIs(audit["bounded_lsq_invoked"], False)
        self.assertIs(audit["bounded_lsq_success"], False)
        self.assertIs(audit["reuse_previous_solution"], True)

    def test_slsqp_audit_does_not_change_solution(self) -> None:
        optimizer = _optimizer()
        matrix = np.asarray([[1.0, 1.0]], dtype=float)
        target = np.asarray([0.25], dtype=float)
        weights = optimizer._weights

        expected = minimize(
            lambda x: 0.5 * float(weights @ (x * x)),
            x0=optimizer._x_prev,
            jac=lambda x: weights * x,
            method="SLSQP",
            bounds=optimizer._bounds,
            constraints=[
                {
                    "type": "eq",
                    "fun": lambda x: matrix @ x - target,
                    "jac": lambda x: matrix,
                }
            ],
            options={"maxiter": 100, "ftol": 1.0e-8, "disp": False},
        )

        actual, success = optimizer._solve_slsqp(matrix, target)

        np.testing.assert_array_equal(actual, expected.x)
        self.assertEqual(success, expected.success)
        audit = optimizer._solver_path_diagnostics
        self.assertIs(audit["slsqp_invoked"], True)
        self.assertEqual(audit["slsqp_status"], int(expected.status))
        self.assertEqual(audit["slsqp_iterations"], int(expected.nit))
        self.assertEqual(
            audit["slsqp_solution"]["output_sha256"],
            optimizer._array_sha256(expected.x),
        )

    def test_solution_audit_exposes_bound_violation_and_nonfinite(self) -> None:
        optimizer = _optimizer()
        matrix = np.eye(2)
        target = np.zeros(2)

        outside = optimizer._solution_audit(
            np.asarray([1.1, -0.6]), matrix, target
        )
        nonfinite = optimizer._solution_audit(
            np.asarray([np.nan, 0.0]), matrix, target
        )

        self.assertAlmostEqual(outside["bound_violation_max"], 0.1)
        self.assertIs(outside["output_finite"], True)
        self.assertIs(nonfinite["output_finite"], False)
        self.assertIsNone(nonfinite["output_sha256"])
        self.assertIsNone(nonfinite["equality_residual_norm"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
