"""Read-only diagnostic for SLSQP failures observed by H0 V3 replay.

This script monkey-patches the in-memory solver only to expose SciPy's status
message and residual.  It returns the exact same solution/success pair as the
production implementation and does not modify simulator artifacts.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import OptimizeResult, minimize


REPO_ROOT = Path(__file__).resolve().parents[1]
for root in (
    REPO_ROOT,
    REPO_ROOT / "validation",
    REPO_ROOT / "Trajectory Generator",
    REPO_ROOT / "Trajectory Generator" / "baseline_MLP",
):
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))

import run_h0_primary_grf_split_v3_semantic_replay as replay  # noqa: E402
from static_optimization import StaticOptimizer  # noqa: E402


def instrumented_slsqp(self, A: np.ndarray, b: np.ndarray):
    weights = self._weights

    def objective(x: np.ndarray) -> float:
        return 0.5 * float(weights @ (x * x))

    def gradient(x: np.ndarray) -> np.ndarray:
        return weights * x

    constraint = {
        "type": "eq",
        "fun": lambda x: A @ x - b,
        "jac": lambda x: A,
    }
    result: OptimizeResult = minimize(
        objective,
        x0=self._x_prev,
        jac=gradient,
        method="SLSQP",
        bounds=self._bounds,
        constraints=[constraint],
        options={"maxiter": self._cfg.qp_max_iter, "ftol": 1e-8, "disp": False},
    )
    if not result.success:
        residual = A @ result.x - b
        lower = np.asarray([item[0] for item in self._bounds], dtype=float)
        upper = np.asarray([item[1] for item in self._bounds], dtype=float)
        print(
            "[V3 SLSQP DIAGNOSTIC] "
            f"status={result.status} message={result.message!s} "
            f"iterations={getattr(result, 'nit', None)} "
            f"max_abs_residual={float(np.max(np.abs(residual))):.17g} "
            f"residual_norm={float(np.linalg.norm(residual)):.17g} "
            f"lower_violation={float(np.max(lower - result.x)):.17g} "
            f"upper_violation={float(np.max(result.x - upper)):.17g}",
            flush=True,
        )
    return result.x, bool(result.success)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, choices=replay.CANONICAL_SEEDS, default=123)
    parser.add_argument("--steps", type=int, default=100)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    StaticOptimizer._solve_slsqp = instrumented_slsqp
    replay.run_replay(
        seed=args.seed,
        output_dir=args.output_dir,
        diagnostic=True,
        max_steps=args.steps,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

