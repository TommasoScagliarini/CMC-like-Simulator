"""Run one read-only fresh-H0 fit with tempered per-case mass.

The diagnostic keeps the exact R5 rows, labels, within-case P2-hardness shape,
optimizer, normalization, and preservation constraints.  It changes only the
mass allocated to each case:

``case_mass_i = 6000 * raw_case_mass_i**gamma / sum(raw_case_mass**gamma)``.

No checkpoint or receipt is written.  The result is design evidence only and
cannot be promoted or treated as a canonical R6 execution.
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


def tempered_weights(
    *,
    gamma: float,
    source_risk: np.ndarray,
    p2_predictions: np.ndarray,
    targets: np.ndarray,
    case_ids: np.ndarray,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Compute exact R5 raw weights with a tempered case-mass allocation."""

    if not math.isfinite(gamma) or gamma < 0.0 or gamma > 1.0:
        raise ValueError("gamma must be finite and in [0, 1]")
    errors = np.max(
        np.abs(p2_predictions.astype(np.float64) - targets.astype(np.float64)),
        axis=1,
    )
    hardness = 1.0 + 99.0 * np.square(
        np.minimum(1.0, errors / contract.HARDNESS_GATE_SCALE)
    )
    raw = np.maximum(source_risk.astype(np.float64), hardness)
    names = tuple(sorted(set(case_ids.astype(str))))
    if set(names) != set(contract.CASE_IDS):
        raise ValueError("case identity drifted")
    raw_mass = {
        name: math.fsum(float(value) for value in raw[case_ids == name])
        for name in names
    }
    powered = {name: raw_mass[name] ** gamma for name in names}
    denominator = math.fsum(powered.values())
    target_mass = {
        name: contract.NORMALIZED_TOTAL_MASS * powered[name] / denominator
        for name in names
    }
    weights = np.empty(len(raw), dtype=np.float64)
    observed_mass: dict[str, float] = {}
    for name in names:
        selection = np.flatnonzero(case_ids == name)
        weights[selection] = raw[selection] * (target_mass[name] / raw_mass[name])
        correction = target_mass[name] - math.fsum(
            float(weights[index]) for index in selection
        )
        weights[selection[-1]] += correction
        observed_mass[name] = math.fsum(float(weights[index]) for index in selection)
    total = math.fsum(float(value) for value in weights)
    if (
        not np.all(np.isfinite(weights))
        or np.any(weights <= 0.0)
        or not math.isclose(
            total, contract.NORMALIZED_TOTAL_MASS, rel_tol=0.0, abs_tol=1.0e-9
        )
    ):
        raise RuntimeError("tempered weights failed closure")
    return np.ascontiguousarray(weights), {
        "gamma": float(gamma),
        "raw_case_mass": raw_mass,
        "target_case_mass": target_mass,
        "observed_case_mass": observed_mass,
        "total_mass": total,
    }


def _evaluate(
    predictions: np.ndarray,
    observations: np.ndarray,
    targets: np.ndarray,
    resets: np.ndarray,
    cases: np.ndarray,
    tranches: np.ndarray,
    steps: np.ndarray,
) -> dict[str, Any]:
    rows = len(predictions)
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
    p2_state = blend._load_state(blend.P2_MODULE)
    p2_predictions = blend._predictions(p2_state, observations)
    p2_critical = blend.metric_pair(p2_predictions, targets, critical)
    metrics = {
        name: blend.metric_triplet(predictions, targets, resets, selection)
        for name, selection in selections.items()
    }
    critical_metrics = blend.metric_pair(predictions, targets, critical)
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


def run(gamma: float) -> dict[str, Any]:
    """Execute the sole requested diagnostic fit entirely in memory."""

    with np.load(blend.CORPUS_PATH, allow_pickle=False) as corpus:
        observations = np.ascontiguousarray(corpus["observations"], dtype=np.float32)
        targets = np.ascontiguousarray(corpus["actions"], dtype=np.float32)
        resets = np.ascontiguousarray(corpus["reset_mask"], dtype=np.bool_)
        cases = corpus["case_ids"].astype(str)
        tranches = corpus["tranche_ids"].astype(str)
        steps = np.ascontiguousarray(corpus["step_indices"], dtype=np.int64)
        source_risk = np.ascontiguousarray(
            corpus["raw_sample_weights"], dtype=np.float64
        )
    p2_state = blend._load_state(blend.P2_MODULE)
    p2_predictions = blend._predictions(p2_state, observations)
    weights, weight_audit = tempered_weights(
        gamma=gamma,
        source_risk=source_risk,
        p2_predictions=p2_predictions,
        targets=targets,
        case_ids=cases,
    )
    _, source_state = r5._load_source_module_and_state()
    normalization = v12r3_fit.frozen_base_normalization(observations[:3000])
    result = r5.fit_case_balanced_full_mean_in_memory(
        source_state=source_state,
        observations=observations,
        targets=targets,
        reset_mask=resets,
        sample_weights=weights,
        normalization=normalization,
    )
    evaluation = _evaluate(
        result.predictions,
        observations,
        targets,
        resets,
        cases,
        tranches,
        steps,
    )
    return {
        "diagnostic_only": True,
        "promotable": False,
        "source": "fresh_H0",
        "teacher_targets": "original_locked_labels",
        "weight_audit": weight_audit,
        "evaluation": evaluation,
        "optimizer_audit": dict(result.optimizer_audit),
        "preservation_audit": dict(result.preservation_audit),
        "normalization_audit": dict(result.normalization_audit),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gamma", type=float, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.gamma), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
