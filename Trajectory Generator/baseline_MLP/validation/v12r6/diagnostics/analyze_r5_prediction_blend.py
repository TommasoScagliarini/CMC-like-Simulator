"""Diagnose the offline feasibility region between the P2 and R5 actors.

This is a read-only design diagnostic.  A prediction-space blend is not a
deployable actor and must never be promoted.  It is used only to determine
whether a successor fit can plausibly trade P2 preservation against the R5
improvement in the critical ``+0.20`` case before any R6 protocol is frozen.
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
for source_root in (
    REPO_ROOT / "validation",
    REPO_ROOT / "Trajectory Generator" / "baseline_MLP",
):
    if str(source_root) not in sys.path:
        sys.path.insert(0, str(source_root))

import h0_primary_split_v11_weighted_fit as v11  # noqa: E402


CORPUS_PATH = (
    REPO_ROOT
    / "Trajectory Generator"
    / "baseline_MLP"
    / "validation"
    / "v12r5"
    / "h0_v12r5_run_20260809"
    / "fit"
    / "corpus.npz"
)
P2_MODULE = (
    REPO_ROOT
    / "Trajectory Generator"
    / "baseline_MLP"
    / "validation"
    / "v12r3"
    / "h0_v12r3_run_20260809"
    / "fit"
    / "p2"
    / "rl_module_target_adapted"
)
R5_MODULE = CORPUS_PATH.parent / "rl_module_target_adapted"
R4_TRACE = (
    REPO_ROOT
    / "Trajectory Generator"
    / "baseline_MLP"
    / "validation"
    / "v12r4"
    / "h0_v12r4_run_20260809"
    / "collect"
    / "deterministic_offset_nominal"
    / "trace.json"
)

RMSE_MAX = 0.006
MAX_ABS_ERROR_MAX = 0.060
RESET_MAX_ABS_ERROR_MAX = 0.003
P2_ROWS = 8_732


def _load_state(module_path: Path) -> dict[str, np.ndarray]:
    from ray.rllib.core.rl_module.rl_module import RLModule

    module = RLModule.from_checkpoint(module_path)
    module.eval()
    return {
        name: np.ascontiguousarray(value, dtype=np.float32)
        for name, value in module.get_state().items()
    }


def _predictions(state: dict[str, np.ndarray], observations: np.ndarray) -> np.ndarray:
    return np.ascontiguousarray(
        v11._state_logits(state, observations)[:, :2],
        dtype=np.float32,
    )


def metric_triplet(
    predictions: np.ndarray,
    targets: np.ndarray,
    reset_mask: np.ndarray,
    selection: np.ndarray,
) -> dict[str, float]:
    """Return the exact three offline metrics for one non-empty slice."""

    selected_reset = selection & reset_mask
    if not np.any(selection) or not np.any(selected_reset):
        raise ValueError("metric selection and reset subset must be non-empty")
    error = predictions[selection].astype(np.float64) - targets[selection].astype(
        np.float64
    )
    reset_error = predictions[selected_reset].astype(np.float64) - targets[
        selected_reset
    ].astype(np.float64)
    return {
        "rmse": float(np.sqrt(np.mean(np.square(error), dtype=np.float64))),
        "max_abs_error": float(np.max(np.abs(error))),
        "reset_max_abs_error": float(np.max(np.abs(reset_error))),
    }


def metric_pair(
    predictions: np.ndarray,
    targets: np.ndarray,
    selection: np.ndarray,
) -> dict[str, float]:
    """Return the two metrics used by the critical-window gate."""

    if not np.any(selection):
        raise ValueError("critical selection must be non-empty")
    error = predictions[selection].astype(np.float64) - targets[selection].astype(
        np.float64
    )
    return {
        "rmse": float(np.sqrt(np.mean(np.square(error), dtype=np.float64))),
        "max_abs_error": float(np.max(np.abs(error))),
    }


def _triplet_passes(value: dict[str, float]) -> bool:
    return (
        value["rmse"] <= RMSE_MAX
        and value["max_abs_error"] <= MAX_ABS_ERROR_MAX
        and value["reset_max_abs_error"] <= RESET_MAX_ABS_ERROR_MAX
    )


def _exposed_nominal_selection(rows: int) -> np.ndarray:
    trace = json.loads(R4_TRACE.read_text(encoding="utf-8"))
    if not isinstance(trace, list) or len(trace) != rows - P2_ROWS:
        raise RuntimeError("R4 nominal trace length drifted")
    exposed = np.asarray(
        [
            item.get("effective_alpha") == 0.5
            and item.get("safety_latch_active") is False
            for item in trace
        ],
        dtype=np.bool_,
    )
    selection = np.zeros(rows, dtype=np.bool_)
    selection[P2_ROWS:] = exposed
    return selection


def analyze(
    step: float, *, blend_space: str = "prediction", full_records: bool = False
) -> dict[str, Any]:
    """Evaluate every registered gate over a fixed alpha grid."""

    if not math.isfinite(step) or step <= 0.0 or step > 1.0:
        raise ValueError("step must be finite and in (0, 1]")
    if blend_space not in {"prediction", "parameter"}:
        raise ValueError("blend_space must be prediction or parameter")
    with np.load(CORPUS_PATH, allow_pickle=False) as corpus:
        observations = np.ascontiguousarray(corpus["observations"], dtype=np.float32)
        targets = np.ascontiguousarray(corpus["actions"], dtype=np.float32)
        resets = np.ascontiguousarray(corpus["reset_mask"], dtype=np.bool_)
        cases = corpus["case_ids"].astype(str)
        tranches = corpus["tranche_ids"].astype(str)
        steps = np.ascontiguousarray(corpus["step_indices"], dtype=np.int64)

    p2_state = _load_state(P2_MODULE)
    r5_state = _load_state(R5_MODULE)
    if p2_state.keys() != r5_state.keys() or any(
        p2_state[name].shape != r5_state[name].shape for name in p2_state
    ):
        raise RuntimeError("P2/R5 state topology drifted")
    p2 = _predictions(p2_state, observations)
    r5 = _predictions(r5_state, observations)
    rows = len(observations)
    selections = {
        "global": np.ones(rows, dtype=np.bool_),
        "p2_subset": np.arange(rows) < P2_ROWS,
        "nominal_r4_pass": np.arange(rows) >= P2_ROWS,
        "nominal_r4_student_exposed": _exposed_nominal_selection(rows),
    }
    selections.update(
        {f"case::{case_id}": cases == case_id for case_id in sorted(set(cases))}
    )
    critical = (
        (cases == "deterministic_offset_plus_0p20")
        & (tranches == "v8r1p1_base")
        & (steps >= 108)
        & (steps <= 230)
    )
    p2_critical = metric_pair(p2, targets, critical)

    count = int(math.floor(1.0 / step + 1.0e-12))
    alphas = sorted({0.0, 1.0, *(min(1.0, index * step) for index in range(count + 1))})
    feasible: list[dict[str, Any]] = []
    closest: list[dict[str, Any]] = []
    for alpha in alphas:
        if blend_space == "prediction":
            prediction = np.ascontiguousarray((1.0 - alpha) * p2 + alpha * r5)
        else:
            blended_state = {
                name: np.ascontiguousarray(
                    (1.0 - alpha) * p2_state[name] + alpha * r5_state[name],
                    dtype=np.float32,
                )
                for name in p2_state
            }
            prediction = _predictions(blended_state, observations)
        metrics = {
            name: metric_triplet(prediction, targets, resets, selection)
            for name, selection in selections.items()
        }
        critical_metrics = metric_pair(prediction, targets, critical)
        failed = [name for name, value in metrics.items() if not _triplet_passes(value)]
        if any(
            critical_metrics[name] > p2_critical[name]
            for name in ("rmse", "max_abs_error")
        ):
            failed.append("critical_window_non_regression")
        record = {
            "alpha_r5": float(alpha),
            "failed_checks": failed,
            "metrics": metrics,
            "critical_window_metrics": critical_metrics,
        }
        if not failed:
            feasible.append(record)
        score = sum(
            max(0.0, value["rmse"] / RMSE_MAX - 1.0)
            + max(0.0, value["max_abs_error"] / MAX_ABS_ERROR_MAX - 1.0)
            + max(
                0.0,
                value["reset_max_abs_error"] / RESET_MAX_ABS_ERROR_MAX - 1.0,
            )
            for value in metrics.values()
        )
        score += sum(
            max(0.0, critical_metrics[name] / p2_critical[name] - 1.0)
            for name in ("rmse", "max_abs_error")
        )
        closest.append({"score": float(score), **record})

    closest.sort(key=lambda item: (item["score"], item["alpha_r5"]))
    feasible_examples = []
    if feasible:
        example_indices = sorted({0, len(feasible) // 2, len(feasible) - 1})
        feasible_examples = [feasible[index] for index in example_indices]
    payload = {
        "diagnostic_only": True,
        "blend_space": blend_space,
        "blend_promotable": False,
        "alpha_definition": (f"{blend_space}=(1-alpha_r5)*P2+alpha_r5*R5"),
        "grid_step": float(step),
        "grid_count": len(alphas),
        "thresholds": {
            "rmse_max": RMSE_MAX,
            "max_abs_error_max": MAX_ABS_ERROR_MAX,
            "reset_max_abs_error_max": RESET_MAX_ABS_ERROR_MAX,
        },
        "p2_critical_window_metrics": p2_critical,
        "feasible_count": len(feasible),
        "feasible_alpha_min": feasible[0]["alpha_r5"] if feasible else None,
        "feasible_alpha_max": feasible[-1]["alpha_r5"] if feasible else None,
        "feasible_examples": feasible_examples,
        "closest": closest[:1],
    }
    if full_records:
        payload["feasible"] = feasible
        payload["all_grid_records"] = closest
    return payload


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--step", type=float, default=0.001)
    parser.add_argument(
        "--blend-space", choices=("prediction", "parameter"), default="prediction"
    )
    parser.add_argument("--full-records", action="store_true")
    args = parser.parse_args()
    print(
        json.dumps(
            analyze(
                args.step,
                blend_space=args.blend_space,
                full_records=args.full_records,
            ),
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
