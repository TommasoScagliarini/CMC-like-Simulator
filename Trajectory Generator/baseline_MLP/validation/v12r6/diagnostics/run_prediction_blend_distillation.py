"""Distil one fixed P2/R5 action blend into a fresh-H0 standard actor.

This read-only design diagnostic writes no checkpoint or canonical receipt.
The P2/R5 blend is used only as a soft target; the resulting in-memory actor is
evaluated against the original locked teacher labels and every R5 offline
slice.  It cannot be promoted as an R6 result.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np


def _repo_root() -> Path:
    for candidate in Path(__file__).resolve().parents:
        if (candidate / "AGENTS.md").is_file():
            return candidate
    raise RuntimeError("repository root not found")


REPO_ROOT = _repo_root()
R5_ROOT = REPO_ROOT / "Trajectory Generator" / "baseline_MLP" / "validation" / "v12r5"
for source_root in (
    REPO_ROOT / "validation",
    REPO_ROOT / "Trajectory Generator" / "baseline_MLP",
    REPO_ROOT / "Trajectory Generator" / "baseline_MLP" / "validation" / "v12r3",
    R5_ROOT,
    Path(__file__).resolve().parent,
):
    if str(source_root) not in sys.path:
        sys.path.insert(0, str(source_root))

import analyze_r5_prediction_blend as blend  # noqa: E402
import h0_primary_split_v12r3_recovery_weighted_fitter as v12r3_fit  # noqa: E402
import h0_v12r5_case_balanced_contract as contract  # noqa: E402
import h0_v12r5_case_balanced_fitter as r5  # noqa: E402


def equal_case_reset_weights(
    cases: np.ndarray, resets: np.ndarray
) -> tuple[np.ndarray, dict[str, float]]:
    """Allocate mass 1000 per case and weight resets 100 within each case."""

    raw = np.where(resets, np.float64(100.0), np.float64(1.0))
    weights = np.empty(len(cases), dtype=np.float64)
    case_mass: dict[str, float] = {}
    for name in sorted(set(cases)):
        selection = np.flatnonzero(cases == name)
        denominator = math.fsum(float(raw[index]) for index in selection)
        weights[selection] = raw[selection] * (contract.CASE_TARGET_MASS / denominator)
        correction = contract.CASE_TARGET_MASS - math.fsum(
            float(weights[index]) for index in selection
        )
        weights[selection[-1]] += correction
        case_mass[name] = math.fsum(float(weights[index]) for index in selection)
    if not math.isclose(
        math.fsum(float(value) for value in weights),
        contract.NORMALIZED_TOTAL_MASS,
        rel_tol=0.0,
        abs_tol=1.0e-9,
    ):
        raise RuntimeError("distillation weights failed closure")
    return np.ascontiguousarray(weights), case_mass


def _evaluation_selections(
    cases: np.ndarray, tranches: np.ndarray, steps: np.ndarray
) -> tuple[dict[str, np.ndarray], np.ndarray]:
    rows = len(cases)
    selections = {
        "global": np.ones(rows, dtype=np.bool_),
        "p2_subset": np.arange(rows) < blend.P2_ROWS,
        "nominal_r4_pass": np.arange(rows) >= blend.P2_ROWS,
        "nominal_r4_student_exposed": blend._exposed_nominal_selection(rows),
    }
    selections.update({f"case::{name}": cases == name for name in sorted(set(cases))})
    critical = (
        (cases == "deterministic_offset_plus_0p20")
        & (tranches == "v8r1p1_base")
        & (steps >= 108)
        & (steps <= 230)
    )
    return selections, critical


def _teacher_evaluation(
    predictions: np.ndarray,
    teacher_targets: np.ndarray,
    resets: np.ndarray,
    selections: dict[str, np.ndarray],
    critical: np.ndarray,
    p2_predictions: np.ndarray,
) -> dict[str, Any]:
    metrics = {
        name: blend.metric_triplet(predictions, teacher_targets, resets, selection)
        for name, selection in selections.items()
    }
    critical_metrics = blend.metric_pair(predictions, teacher_targets, critical)
    p2_critical = blend.metric_pair(p2_predictions, teacher_targets, critical)
    failed = [
        name for name, value in metrics.items() if not blend._triplet_passes(value)
    ]
    if any(
        critical_metrics[name] > p2_critical[name] for name in ("rmse", "max_abs_error")
    ):
        failed.append("critical_window_non_regression")
    return {
        "passed": not failed,
        "failed_checks": failed,
        "metrics": metrics,
        "critical_window_metrics": critical_metrics,
        "p2_critical_window_metrics": p2_critical,
    }


def run(
    alpha_r5: float,
    *,
    weighting: str = "equal_case_reset",
    correction_passes: int = 0,
) -> dict[str, Any]:
    """Run one in-memory distillation with a preregistered blend coefficient."""

    if not math.isfinite(alpha_r5) or alpha_r5 < 0.0 or alpha_r5 > 1.0:
        raise ValueError("alpha_r5 must be finite and in [0, 1]")
    if weighting not in {"equal_case_reset", "r5_locked"}:
        raise ValueError("unsupported weighting")
    if type(correction_passes) is not int or correction_passes not in {0, 1}:
        raise ValueError("correction_passes must be 0 or 1")
    with np.load(blend.CORPUS_PATH, allow_pickle=False) as corpus:
        observations = np.ascontiguousarray(corpus["observations"], dtype=np.float32)
        teacher_targets = np.ascontiguousarray(corpus["actions"], dtype=np.float32)
        resets = np.ascontiguousarray(corpus["reset_mask"], dtype=np.bool_)
        cases = corpus["case_ids"].astype(str)
        tranches = corpus["tranche_ids"].astype(str)
        steps = np.ascontiguousarray(corpus["step_indices"], dtype=np.int64)
        r5_locked_weights = np.ascontiguousarray(
            corpus["normalized_sample_weights"], dtype=np.float64
        )
    p2_predictions = blend._predictions(
        blend._load_state(blend.P2_MODULE), observations
    )
    r5_predictions = blend._predictions(
        blend._load_state(blend.R5_MODULE), observations
    )
    soft_targets = np.ascontiguousarray(
        (1.0 - alpha_r5) * p2_predictions + alpha_r5 * r5_predictions,
        dtype=np.float32,
    )
    if weighting == "equal_case_reset":
        weights, case_mass = equal_case_reset_weights(cases, resets)
        weighting_id = "equal_case_mass_1000_reset_100_else_1"
    else:
        weights = r5_locked_weights
        case_mass = {
            name: math.fsum(float(value) for value in weights[cases == name])
            for name in sorted(set(cases))
        }
        weighting_id = "r5_locked_p2_hardness_equal_case_mass"
    normalization = v12r3_fit.frozen_base_normalization(observations[:3000])
    fit_targets = soft_targets
    pass_audit: list[dict[str, Any]] = []
    result = None
    for pass_index in range(correction_passes + 1):
        _, source_state = r5._load_source_module_and_state()
        result = r5.fit_case_balanced_full_mean_in_memory(
            source_state=source_state,
            observations=observations,
            targets=fit_targets,
            reset_mask=resets,
            sample_weights=weights,
            normalization=normalization,
        )
        residual = soft_targets.astype(np.float64) - result.predictions.astype(
            np.float64
        )
        pass_audit.append(
            {
                "pass_index": pass_index,
                "fit_target_max_abs_delta_from_soft": float(
                    np.max(
                        np.abs(
                            fit_targets.astype(np.float64)
                            - soft_targets.astype(np.float64)
                        )
                    )
                ),
                "soft_target_residual_rmse": float(
                    np.sqrt(np.mean(np.square(residual), dtype=np.float64))
                ),
                "soft_target_residual_max_abs": float(np.max(np.abs(residual))),
                "optimizer_audit": dict(result.optimizer_audit),
            }
        )
        if pass_index < correction_passes:
            fit_targets = np.ascontiguousarray(
                soft_targets.astype(np.float64) + residual,
                dtype=np.float32,
            )
    if result is None:
        raise AssertionError("distillation fit did not run")
    selections, critical = _evaluation_selections(cases, tranches, steps)
    teacher_evaluation = _teacher_evaluation(
        result.predictions,
        teacher_targets,
        resets,
        selections,
        critical,
        p2_predictions,
    )
    distillation_metrics = {
        name: blend.metric_triplet(result.predictions, soft_targets, resets, selection)
        for name, selection in selections.items()
    }
    return {
        "diagnostic_only": True,
        "promotable": False,
        "source": "fresh_H0",
        "alpha_r5": float(alpha_r5),
        "soft_target": "(1-alpha_r5)*P2_prediction+alpha_r5*R5_prediction",
        "weighting": weighting_id,
        "correction_passes": correction_passes,
        "pass_audit": pass_audit,
        "case_mass": case_mass,
        "teacher_evaluation": teacher_evaluation,
        "distillation_metrics": distillation_metrics,
        "optimizer_audit": dict(result.optimizer_audit),
        "preservation_audit": dict(result.preservation_audit),
        "normalization_audit": dict(result.normalization_audit),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--alpha-r5", type=float, required=True)
    parser.add_argument(
        "--weighting",
        choices=("equal_case_reset", "r5_locked"),
        default="equal_case_reset",
    )
    parser.add_argument("--correction-passes", type=int, choices=(0, 1), default=0)
    args = parser.parse_args()
    print(
        json.dumps(
            run(
                args.alpha_r5,
                weighting=args.weighting,
                correction_passes=args.correction_passes,
            ),
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
