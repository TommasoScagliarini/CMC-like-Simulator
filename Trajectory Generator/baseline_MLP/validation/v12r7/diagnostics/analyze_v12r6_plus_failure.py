"""Read-only forensics for the V12R6 ``+0.20`` penetration failure.

The diagnostic deliberately does not simulate, fit, save a checkpoint, or
write an artifact.  It compares the frozen P2/R5 actors on three already
recorded state distributions:

* the 500-step safe V8R1P1 teacher replay;
* the 212-step terminal V12R4 shielded trajectory (same-state teacher labels);
* the 179-step terminal V12R6 pure-composite trajectory.

The alpha results are counterfactual predictions on fixed observations.  They
must not be interpreted as closed-loop evidence or as authorization for an
alpha sweep.
"""

from __future__ import annotations

import json
import os
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
BASELINE_ROOT = REPO_ROOT / "Trajectory Generator" / "baseline_MLP"
for source_root in (BASELINE_ROOT, REPO_ROOT / "validation"):
    value = os.fspath(source_root)
    if value not in sys.path:
        sys.path.insert(0, value)

import h0_primary_split_v11_weighted_fit as v11  # noqa: E402


P2_CORPUS = BASELINE_ROOT / "validation/v12r3/h0_v12r3_run_20260809/fit/p2/corpus.npz"
P2_MODULE = P2_CORPUS.parent / "rl_module_target_adapted"
R5_CORPUS = BASELINE_ROOT / "validation/v12r5/h0_v12r5_run_20260809/fit/corpus.npz"
R5_MODULE = R5_CORPUS.parent / "rl_module_target_adapted"
R4_PLUS_LABELS = (
    BASELINE_ROOT
    / "validation/v12r4/h0_v12r4_run_20260809/collect"
    / "deterministic_offset_plus_0p20/labels.npz"
)
R4_PLUS_TRACE = R4_PLUS_LABELS.with_name("trace.json")
R6_PLUS_STEPS = (
    BASELINE_ROOT
    / "validation/v12r6/h0_v12r6_run_20260814/development"
    / "deterministic_offset_plus_0p20/steps"
)
SAFE_PLUS_TRACE = (
    REPO_ROOT
    / "validation/h0_primary_grf_split_adaptation_runs"
    / "2026-08-07_h0_primary_split_v8r1p1_v26_residual/teacher_replay"
    / "deterministic_offset_plus_0p20/trace.json"
)

CASE_ID = "deterministic_offset_plus_0p20"
SAFE_TRANCHE = "v8r1p1_base"
R4_TRANCHE = "v12r4_p3_coverage"
R6_ALPHA = 0.30
LOW_FEASIBLE_ALPHA = 0.268
LATE_STEP_START = 140
ACTION_NAMES = ("knee", "ankle")


class DiagnosticError(RuntimeError):
    """Raised when a frozen input does not match its expected structure."""


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
        v11._state_logits(state, observations)[:, :2], dtype=np.float32
    )


def _load_trace(path: Path) -> list[dict[str, Any]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, list) or not payload:
        raise DiagnosticError(f"trace is empty or malformed: {path}")
    if [row.get("step") for row in payload] != list(range(1, len(payload) + 1)):
        raise DiagnosticError(f"trace step sequence drifted: {path}")
    return payload


def _load_r6_steps(path: Path) -> list[dict[str, Any]]:
    files = sorted(path.glob("*.json"))
    rows = [json.loads(item.read_text(encoding="utf-8")) for item in files]
    if [row.get("step") for row in rows] != list(range(1, len(rows) + 1)):
        raise DiagnosticError("V12R6 step sequence drifted")
    return rows


def _metric(prediction: np.ndarray, target: np.ndarray) -> dict[str, Any]:
    error = prediction.astype(np.float64) - target.astype(np.float64)
    return {
        "rmse": float(np.sqrt(np.mean(np.square(error), dtype=np.float64))),
        "max_abs_error": float(np.max(np.abs(error))),
        "component_rmse": {
            name: float(np.sqrt(np.mean(np.square(error[:, index]), dtype=np.float64)))
            for index, name in enumerate(ACTION_NAMES)
        },
        "component_max_abs_error": {
            name: float(np.max(np.abs(error[:, index])))
            for index, name in enumerate(ACTION_NAMES)
        },
    }


def _percentiles(value: np.ndarray) -> dict[str, float]:
    data = np.asarray(value, dtype=np.float64)
    return {
        "min": float(np.min(data)),
        "p50": float(np.percentile(data, 50)),
        "p90": float(np.percentile(data, 90)),
        "p95": float(np.percentile(data, 95)),
        "max": float(np.max(data)),
    }


def _nearest_distances(
    query: np.ndarray, reference: np.ndarray, scale: np.ndarray
) -> np.ndarray:
    # At most 179 x 9232 x 35 float64 values are considered.  Chunking keeps
    # the helper portable on ordinary development machines.
    result = np.empty(len(query), dtype=np.float64)
    safe_scale = np.maximum(np.asarray(scale, dtype=np.float64), 1.0e-4)
    ref = reference.astype(np.float64) / safe_scale
    for start in range(0, len(query), 32):
        stop = min(start + 32, len(query))
        normalized = query[start:stop].astype(np.float64) / safe_scale
        squared = np.sum(np.square(normalized[:, None, :] - ref[None, :, :]), axis=2)
        result[start:stop] = np.sqrt(np.min(squared, axis=1))
    return result


def _matched_step_distance(
    query: np.ndarray, reference: np.ndarray, scale: np.ndarray
) -> np.ndarray:
    rows = min(len(query), len(reference))
    safe_scale = np.maximum(np.asarray(scale, dtype=np.float64), 1.0e-4)
    difference = (
        query[:rows].astype(np.float64) - reference[:rows].astype(np.float64)
    ) / safe_scale
    return np.sqrt(np.sum(np.square(difference), axis=1))


def _alpha_grid_metrics(
    p2: np.ndarray, r5: np.ndarray, target: np.ndarray
) -> dict[str, Any]:
    records: list[tuple[float, dict[str, Any]]] = []
    for alpha in np.linspace(0.0, 1.0, 1001, dtype=np.float64):
        prediction = np.ascontiguousarray((1.0 - alpha) * p2 + alpha * r5)
        records.append((float(alpha), _metric(prediction, target)))
    by_rmse = min(records, key=lambda item: (item[1]["rmse"], item[0]))
    by_max = min(records, key=lambda item: (item[1]["max_abs_error"], item[0]))
    selected = {}
    for alpha in (0.0, LOW_FEASIBLE_ALPHA, R6_ALPHA, 1.0):
        prediction = np.ascontiguousarray((1.0 - alpha) * p2 + alpha * r5)
        selected[f"alpha_{alpha:.3f}"] = _metric(prediction, target)
    return {
        "diagnostic_only_fixed_observations": True,
        "grid_step": 0.001,
        "best_rmse": {"alpha_r5": by_rmse[0], **by_rmse[1]},
        "best_max_abs_error": {"alpha_r5": by_max[0], **by_max[1]},
        "selected": selected,
    }


def _first_threshold_step(
    values: np.ndarray, threshold: float, *, after_step: int = 0
) -> int | None:
    indices = np.flatnonzero(
        (np.arange(1, len(values) + 1) > after_step)
        & (np.asarray(values, dtype=np.float64) >= threshold)
    )
    return int(indices[0] + 1) if len(indices) else None


def analyze() -> dict[str, Any]:
    """Return a JSON-serializable, read-only forensic summary."""

    with np.load(P2_CORPUS, allow_pickle=False) as source:
        p2_observations = np.ascontiguousarray(source["observations"], dtype=np.float32)
        p2_actions = np.ascontiguousarray(source["actions"], dtype=np.float32)
        p2_cases = source["case_ids"].astype(str)
        p2_tranches = source["tranche_ids"].astype(str)
        p2_steps = np.ascontiguousarray(source["step_indices"], dtype=np.int64)
        actor_feature_names = tuple(source["actor_feature_names"].astype(str))
    with np.load(R5_CORPUS, allow_pickle=False) as source:
        r5_cases = source["case_ids"].astype(str)
        r5_tranches = source["tranche_ids"].astype(str)
        r5_rows = len(source["observations"])
    with np.load(R4_PLUS_LABELS, allow_pickle=False) as source:
        r4_observations = np.ascontiguousarray(source["observations"], dtype=np.float32)
        r4_targets = np.ascontiguousarray(source["actions"], dtype=np.float32)
        r4_steps = np.ascontiguousarray(source["step_indices"], dtype=np.int64)

    safe_mask = (p2_cases == CASE_ID) & (p2_tranches == SAFE_TRANCHE)
    safe_order = np.argsort(p2_steps[safe_mask], kind="stable")
    safe_observations = p2_observations[safe_mask][safe_order]
    safe_targets = p2_actions[safe_mask][safe_order]
    safe_steps = p2_steps[safe_mask][safe_order]
    if not np.array_equal(safe_steps, np.arange(1, 501, dtype=np.int64)):
        raise DiagnosticError("safe +0.20 replay is not the expected 500-step tape")
    if not np.array_equal(r4_steps, np.arange(1, 213, dtype=np.int64)):
        raise DiagnosticError("R4 +0.20 label prefix is not 212 contiguous steps")

    r4_trace = _load_trace(R4_PLUS_TRACE)
    safe_trace = _load_trace(SAFE_PLUS_TRACE)
    r6_rows = _load_r6_steps(R6_PLUS_STEPS)
    r6_observations = np.asarray(
        [row["v26_observation"] for row in r6_rows], dtype=np.float32
    )
    r6_actions = np.asarray(
        [row["candidate_mean"] for row in r6_rows], dtype=np.float32
    )
    if r6_observations.shape != (179, 35) or r6_actions.shape != (179, 2):
        raise DiagnosticError("V12R6 terminal prefix shape drifted")

    p2_state = _load_state(P2_MODULE)
    r5_state = _load_state(R5_MODULE)
    predictions = {}
    for name, observations in {
        "safe_plus": safe_observations,
        "r4_failed_plus": r4_observations,
        "r6_terminal_plus": r6_observations,
    }.items():
        predictions[name] = {
            "p2": _predictions(p2_state, observations),
            "r5": _predictions(r5_state, observations),
        }

    reproduced_r6 = (1.0 - R6_ALPHA) * predictions["r6_terminal_plus"][
        "p2"
    ] + R6_ALPHA * predictions["r6_terminal_plus"]["r5"]
    reproduction_max = float(
        np.max(np.abs(reproduced_r6.astype(np.float64) - r6_actions))
    )
    if reproduction_max > 2.0e-6:
        raise DiagnosticError("the recorded V12R6 actor is not the 0.30 blend")

    robust_scale = np.percentile(p2_observations, 75, axis=0) - np.percentile(
        p2_observations, 25, axis=0
    )
    standard_scale = np.std(p2_observations, axis=0, dtype=np.float64)
    scale = np.maximum(robust_scale.astype(np.float64), standard_scale)
    constant_columns = np.flatnonzero(scale < 1.0e-4)

    plus_mask = p2_cases == CASE_ID
    r6_to_safe = _nearest_distances(r6_observations, safe_observations, scale)
    r6_to_p2_plus = _nearest_distances(
        r6_observations, p2_observations[plus_mask], scale
    )
    r6_to_all_p2 = _nearest_distances(r6_observations, p2_observations, scale)
    r6_to_r4 = _nearest_distances(r6_observations, r4_observations, scale)
    r6_matched_safe = _matched_step_distance(r6_observations, safe_observations, scale)
    r6_matched_r4 = _matched_step_distance(r6_observations, r4_observations, scale)

    late_mask = np.arange(1, len(r6_rows) + 1) >= LATE_STEP_START
    alpha_268 = (1.0 - LOW_FEASIBLE_ALPHA) * predictions["r6_terminal_plus"][
        "p2"
    ] + LOW_FEASIBLE_ALPHA * predictions["r6_terminal_plus"]["r5"]
    alpha_shift = alpha_268.astype(np.float64) - r6_actions.astype(np.float64)
    safe_matched_error = r6_actions.astype(np.float64) - safe_targets[:179].astype(
        np.float64
    )
    alpha_268_safe_error = alpha_268.astype(np.float64) - safe_targets[:179].astype(
        np.float64
    )

    r4_penetration = np.asarray(
        [row["grf_penetration_m"] for row in r4_trace], dtype=np.float64
    )
    safe_penetration = np.asarray(
        [row["reward_terms"]["grf_penetration_m"] for row in safe_trace],
        dtype=np.float64,
    )
    r6_penetration = np.asarray(
        [row["grf_penetration_m"] for row in r6_rows], dtype=np.float64
    )
    r4_latch = np.asarray(
        [row["safety_latch_active"] for row in r4_trace], dtype=np.bool_
    )

    return {
        "diagnostic_only": True,
        "closed_loop_claim_authorized": False,
        "inputs": {
            "p2_corpus_rows": int(len(p2_observations)),
            "r5_corpus_rows": int(r5_rows),
            "safe_plus_rows_in_p2": int(np.sum(safe_mask)),
            "r4_failed_plus_label_rows": int(len(r4_observations)),
            "r4_failed_plus_rows_in_r5": int(
                np.sum((r5_cases == CASE_ID) & (r5_tranches == R4_TRANCHE))
            ),
            "r6_terminal_plus_rows": int(len(r6_observations)),
        },
        "r6_composite_reproduction_max_abs": reproduction_max,
        "fixed_state_alpha_metrics": {
            "safe_plus": _alpha_grid_metrics(
                predictions["safe_plus"]["p2"],
                predictions["safe_plus"]["r5"],
                safe_targets,
            ),
            "r4_failed_plus": _alpha_grid_metrics(
                predictions["r4_failed_plus"]["p2"],
                predictions["r4_failed_plus"]["r5"],
                r4_targets,
            ),
        },
        "r6_distribution_distance": {
            "definition": "Euclidean distance after per-feature max(IQR,std,1e-4) scaling",
            "constant_feature_indices": [int(item) for item in constant_columns],
            "constant_feature_names": [
                actor_feature_names[index] for index in constant_columns
            ],
            "nearest_safe_plus": _percentiles(r6_to_safe),
            "nearest_any_p2_plus": _percentiles(r6_to_p2_plus),
            "nearest_any_p2": _percentiles(r6_to_all_p2),
            "nearest_r4_failed_plus": _percentiles(r6_to_r4),
            "matched_step_safe_plus": _percentiles(r6_matched_safe),
            "matched_step_r4_failed_plus": _percentiles(r6_matched_r4),
            "late_step_140_179": {
                "nearest_safe_plus": _percentiles(r6_to_safe[late_mask]),
                "nearest_any_p2_plus": _percentiles(r6_to_p2_plus[late_mask]),
                "nearest_r4_failed_plus": _percentiles(r6_to_r4[late_mask]),
                "matched_step_safe_plus": _percentiles(r6_matched_safe[late_mask]),
                "matched_step_r4_failed_plus": _percentiles(r6_matched_r4[late_mask]),
            },
        },
        "alpha_0p268_vs_0p300_on_r6_states": {
            "component_shift_abs_max": {
                name: float(np.max(np.abs(alpha_shift[:, index])))
                for index, name in enumerate(ACTION_NAMES)
            },
            "late_component_shift_abs_max": {
                name: float(np.max(np.abs(alpha_shift[late_mask, index])))
                for index, name in enumerate(ACTION_NAMES)
            },
            "late_component_shift_abs_mean": {
                name: float(np.mean(np.abs(alpha_shift[late_mask, index])))
                for index, name in enumerate(ACTION_NAMES)
            },
            "matched_safe_tape_component_rmse_alpha_0p300": {
                name: float(
                    np.sqrt(
                        np.mean(
                            np.square(safe_matched_error[late_mask, index]),
                            dtype=np.float64,
                        )
                    )
                )
                for index, name in enumerate(ACTION_NAMES)
            },
            "matched_safe_tape_component_rmse_alpha_0p268": {
                name: float(
                    np.sqrt(
                        np.mean(
                            np.square(alpha_268_safe_error[late_mask, index]),
                            dtype=np.float64,
                        )
                    )
                )
                for index, name in enumerate(ACTION_NAMES)
            },
            "matched_safe_tape_is_not_same_state_teacher_evidence": True,
        },
        "penetration_timing": {
            "late_excursion_definition": "first threshold crossing after step 130",
            "safe_teacher_tape": {
                "late_first_10mm_step": _first_threshold_step(
                    safe_penetration, 0.010, after_step=130
                ),
                "late_first_15mm_step": _first_threshold_step(
                    safe_penetration, 0.015, after_step=130
                ),
                "late_first_20mm_step": _first_threshold_step(
                    safe_penetration, 0.020, after_step=130
                ),
                "max_m": float(np.max(safe_penetration)),
            },
            "r6": {
                "late_first_10mm_step": _first_threshold_step(
                    r6_penetration, 0.010, after_step=130
                ),
                "late_first_15mm_step": _first_threshold_step(
                    r6_penetration, 0.015, after_step=130
                ),
                "late_first_20mm_step": _first_threshold_step(
                    r6_penetration, 0.020, after_step=130
                ),
                "terminal_step": int(len(r6_penetration)),
                "terminal_m": float(r6_penetration[-1]),
            },
            "r4": {
                "late_first_10mm_step": _first_threshold_step(
                    r4_penetration, 0.010, after_step=130
                ),
                "late_first_15mm_step": _first_threshold_step(
                    r4_penetration, 0.015, after_step=130
                ),
                "late_first_20mm_step": _first_threshold_step(
                    r4_penetration, 0.020, after_step=130
                ),
                "first_latch_step": int(np.flatnonzero(r4_latch)[0] + 1),
                "terminal_step": int(len(r4_penetration)),
                "terminal_m": float(r4_penetration[-1]),
                "teacher_takeover_steps": int(np.sum(r4_latch)),
            },
        },
        "selected_steps": {
            str(step): {
                "r6_action": r6_actions[step - 1].tolist(),
                "alpha_0p268_on_r6_state": alpha_268[step - 1].tolist(),
                "safe_tape_action": safe_targets[step - 1].tolist(),
                "r4_applied_action": r4_trace[step - 1]["applied_action"],
                "r4_same_state_teacher": r4_targets[step - 1].tolist(),
                "r6_penetration_m": float(r6_penetration[step - 1]),
                "r4_penetration_m": float(r4_penetration[step - 1]),
                "safe_tape_penetration_m": float(safe_penetration[step - 1]),
            }
            for step in (120, 140, 150, 170, 175, 179)
        },
    }


def main() -> int:
    print(json.dumps(analyze(), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
