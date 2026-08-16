"""Read-only forensic analysis of the terminal V12R9 offline fit.

This diagnostic deliberately imports the frozen V12R9 implementation only to
reconstruct predictions and source weights.  It never calls a V12R9 stage,
never writes below ``validation/v12r9``, and never opens an environment.

The optional fixed-hidden regressions are counterfactuals: they continue from
the frozen R9 actor by solving only its two mean-output rows.  No RLModule or
candidate checkpoint is published.
"""

from __future__ import annotations

import argparse
import collections
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any, Iterable, Mapping

import numpy as np


def _discover_repo_root(source: Path) -> Path:
    for candidate in source.resolve().parents:
        if (
            (candidate / "AGENTS.md").is_file()
            and (candidate / "Trajectory Generator").is_dir()
            and (candidate / "validation").is_dir()
        ):
            return candidate
    raise RuntimeError("repository root could not be discovered")


REPO_ROOT = _discover_repo_root(Path(__file__))
BASELINE_ROOT = REPO_ROOT / "Trajectory Generator" / "baseline_MLP"
R9_ROOT = BASELINE_ROOT / "validation" / "v12r9"
for _path in (BASELINE_ROOT, R9_ROOT):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import warm_start  # noqa: E402,F401  # registers the custom RLModule class
import h0_v12r9_recovery_fitter as r9  # noqa: E402


CORPUS_PATH = REPO_ROOT / r9.contract.CORPUS_PATH
SUMMARY_PATH = REPO_ROOT / r9.contract.FIT_SUMMARY_PATH
GATE_PATH = REPO_ROOT / r9.contract.FIT_GATE_PATH
LEDGER_PATH = REPO_ROOT / r9.contract.LEDGER_PATH
CANDIDATE_PATH = REPO_ROOT / r9.contract.CANDIDATE_MODULE_PATH
EXPECTED_TERMINAL_STATUS = "FAIL_H0_V12R9_RECOVERY_PIPELINE_TERMINAL"


def _strict_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _metrics(predictions: np.ndarray, targets: np.ndarray) -> dict[str, float]:
    error = predictions.astype(np.float64) - targets.astype(np.float64)
    return {
        "rmse": float(np.sqrt(np.mean(np.square(error)))),
        "max_abs_error": float(np.max(np.abs(error), initial=0.0)),
    }


def _indices_by_value(values: np.ndarray) -> dict[str, np.ndarray]:
    return {
        str(value): np.flatnonzero(values.astype(str) == str(value))
        for value in sorted(set(values.astype(str)))
    }


def _metric_payload(
    predictions: np.ndarray, arrays: Mapping[str, np.ndarray]
) -> dict[str, Any]:
    actions = arrays["actions"]
    strata = arrays["stratum_ids"].astype(str)
    cases = arrays["case_ids"].astype(str)
    steps = arrays["step_indices"]
    base_indices = {
        case_id: np.flatnonzero((strata == f"base::{case_id}") & (cases == case_id))
        for case_id in r9.contract.COLLECTION_CASE_IDS
    }
    observer_indices = {
        case_id: np.flatnonzero(strata == f"observer::{case_id}")
        for case_id in r9.contract.COLLECTION_CASE_IDS
    }
    r4_indices = np.flatnonzero(strata == "r4_failure::deterministic_offset_plus_0p20")
    plus_late = observer_indices["deterministic_offset_plus_0p20"]
    plus_late = plus_late[steps[plus_late] >= 140]
    error = np.abs(predictions.astype(np.float64) - actions.astype(np.float64))
    flat = int(np.argmax(error))
    row, action_dimension = np.unravel_index(flat, error.shape)
    return {
        "global_metrics": _metrics(predictions, actions),
        "reset_max_abs_error": float(np.max(error[arrays["reset_mask"]])),
        "per_case_metrics": {
            case_id: _metrics(predictions[index], actions[index])
            for case_id, index in base_indices.items()
        },
        "r4_failed_plus_metrics": _metrics(
            predictions[r4_indices], actions[r4_indices]
        ),
        "observer_case_metrics": {
            case_id: _metrics(predictions[index], actions[index])
            for case_id, index in observer_indices.items()
        },
        "observer_plus_late_metrics": _metrics(
            predictions[plus_late], actions[plus_late]
        ),
        "worst_row": {
            "absolute_error": float(error[row, action_dimension]),
            "row_index": int(row),
            "action_dimension": int(action_dimension),
            "stratum_id": str(strata[row]),
            "case_id": str(cases[row]),
            "step_index": int(steps[row]),
            "tranche_id": str(arrays["tranche_ids"][row]),
        },
    }


def _gate(metric_payload: Mapping[str, Any]) -> dict[str, Any]:
    summary = dict(_strict_json(SUMMARY_PATH))
    for key in (
        "global_metrics",
        "reset_max_abs_error",
        "per_case_metrics",
        "r4_failed_plus_metrics",
        "observer_case_metrics",
        "observer_plus_late_metrics",
        "worst_row",
    ):
        summary[key] = metric_payload[key]
    return r9.contract.fit_gate(summary)


def _candidate_state_and_predictions(
    observations: np.ndarray,
) -> tuple[dict[str, Any], np.ndarray]:
    from ray.rllib.core.rl_module.rl_module import RLModule

    module = RLModule.from_checkpoint(CANDIDATE_PATH.resolve())
    module.eval()
    state = r9.v11._clone_state(module.get_state())
    r9.validate_source_r6_state(state)
    predictions = np.ascontiguousarray(
        r9.v11._state_logits(state, observations)[:, :2], dtype=np.float32
    )
    return state, predictions


def _teacher_student_view_predictions(observations: np.ndarray) -> np.ndarray:
    from ray.rllib.core.rl_module.rl_module import RLModule

    teacher_path = (REPO_ROOT / r9.contract.SOURCE_H0_MODULE_PATH).resolve()
    teacher = RLModule.from_checkpoint(teacher_path)
    teacher.eval()
    teacher_state = r9.v11._clone_state(teacher.get_state())
    return np.ascontiguousarray(
        r9.v11._state_logits(teacher_state, observations)[:, :2], dtype=np.float32
    )


def _hidden_design(state: Mapping[str, Any], observations: np.ndarray) -> np.ndarray:
    w0 = np.asarray(state["pi_encoder.0.weight"], dtype=np.float64)
    b0 = np.asarray(state["pi_encoder.0.bias"], dtype=np.float64)
    w1 = np.asarray(state["pi_encoder.2.weight"], dtype=np.float64)
    b1 = np.asarray(state["pi_encoder.2.bias"], dtype=np.float64)
    hidden1 = np.tanh(observations.astype(np.float64) @ w0.T + b0)
    hidden2 = np.tanh(hidden1 @ w1.T + b1)
    return np.ascontiguousarray(
        np.concatenate([hidden2, np.ones((len(hidden2), 1), dtype=np.float64)], axis=1)
    )


def _fixed_hidden_weighted_least_squares(
    design: np.ndarray,
    targets: np.ndarray,
    weights: np.ndarray,
    *,
    ridge_fraction: float = 1.0e-12,
) -> tuple[np.ndarray, dict[str, float]]:
    if np.any(weights <= 0.0) or not np.all(np.isfinite(weights)):
        raise ValueError("least-squares weights must be finite and positive")
    weighted_design = design * weights[:, None]
    gram = design.T @ weighted_design
    rhs = design.T @ (weights[:, None] * targets.astype(np.float64))
    scale = float(np.trace(gram[:-1, :-1]) / (design.shape[1] - 1))
    ridge = ridge_fraction * scale
    regularizer = np.eye(design.shape[1], dtype=np.float64) * ridge
    regularizer[-1, -1] = 0.0
    beta = np.linalg.solve(gram + regularizer, rhs)
    predictions = np.ascontiguousarray(design @ beta, dtype=np.float32)
    return predictions, {
        "ridge_fraction": float(ridge_fraction),
        "ridge_absolute": float(ridge),
        "condition_number_regularized_gram": float(np.linalg.cond(gram + regularizer)),
    }


def _source_piece_weights(
    arrays: Mapping[str, np.ndarray],
    *,
    source_key: str,
) -> tuple[np.ndarray, dict[str, Any]]:
    if source_key not in {"raw_sample_weights", "normalized_sample_weights"}:
        raise ValueError(f"unsupported source-weight key: {source_key}")
    features = np.ascontiguousarray(arrays["actor_feature_names"], dtype="U64")
    base = r9._load_base_piece()
    r4 = r9._load_r4_piece(features)
    observers = {
        case_id: r9._load_observer_piece(case_id, features)
        for case_id in r9.contract.COLLECTION_CASE_IDS
    }
    source = np.concatenate(
        [
            base[source_key],
            np.ones(len(r4["observations"]), dtype=np.float64),
            *(
                observers[case_id][source_key]
                for case_id in r9.contract.COLLECTION_CASE_IDS
            ),
        ]
    ).astype(np.float64, copy=False)
    if source.shape != (len(arrays["observations"]),):
        raise RuntimeError("source-weight assembly drifted")
    strata = arrays["stratum_ids"].astype(str)
    normalized = np.empty_like(source)
    audit: dict[str, Any] = {}
    for stratum_id, index in _indices_by_value(strata).items():
        values = source[index]
        normalized[index] = 500.0 * values / math.fsum(values.astype(float))
        audit[stratum_id] = {
            "rows": int(len(index)),
            "source_key": source_key,
            "source_min": float(np.min(values)),
            "source_max": float(np.max(values)),
            "source_100_count": int(np.count_nonzero(values == 100.0)),
            "source_one_count": int(np.count_nonzero(values == 1.0)),
            "normalized_min": float(np.min(normalized[index])),
            "normalized_max": float(np.max(normalized[index])),
            "normalized_mass": float(math.fsum(normalized[index].astype(float))),
        }
    return np.ascontiguousarray(normalized), audit


def _reset_amplified_equal_stratum_weights(
    base_weights: np.ndarray,
    arrays: Mapping[str, np.ndarray],
    *,
    multiplier: float,
) -> np.ndarray:
    weights = np.asarray(base_weights, dtype=np.float64).copy()
    weights[arrays["reset_mask"]] *= float(multiplier)
    strata = arrays["stratum_ids"].astype(str)
    for index in _indices_by_value(strata).values():
        weights[index] *= 500.0 / math.fsum(weights[index].astype(float))
    return np.ascontiguousarray(weights)


def _stratum_error_audit(
    predictions: np.ndarray,
    arrays: Mapping[str, np.ndarray],
    objective_weights: np.ndarray,
) -> dict[str, Any]:
    target = arrays["actions"].astype(np.float64)
    squared_row_error = np.mean(
        np.square(predictions.astype(np.float64) - target), axis=1
    )
    unweighted_total = math.fsum(squared_row_error.astype(float))
    weighted_total = math.fsum((objective_weights * squared_row_error).astype(float))
    output: dict[str, Any] = {}
    for stratum_id, index in _indices_by_value(arrays["stratum_ids"]).items():
        selected_weights = objective_weights[index]
        selected_error = squared_row_error[index]
        output[stratum_id] = {
            "rows": int(len(index)),
            **_metrics(predictions[index], target[index]),
            "unweighted_sse_share": float(
                math.fsum(selected_error.astype(float)) / unweighted_total
            ),
            "objective_sse_share": float(
                math.fsum((selected_weights * selected_error).astype(float))
                / weighted_total
            ),
            "row_weight_min": float(np.min(selected_weights)),
            "row_weight_max": float(np.max(selected_weights)),
            "mass": float(math.fsum(selected_weights.astype(float))),
        }
    return output


def _exact_collision_audit(
    observations: np.ndarray,
    actions: np.ndarray,
    *,
    columns: Iterable[int],
) -> dict[str, Any]:
    projected = np.ascontiguousarray(observations[:, tuple(columns)])
    groups: dict[bytes, list[int]] = collections.defaultdict(list)
    for index, row in enumerate(projected):
        groups[row.tobytes()].append(index)
    duplicate = [group for group in groups.values() if len(group) > 1]
    conflicting = [
        group for group in duplicate if not np.all(actions[group] == actions[group[0]])
    ]
    # An unrestricted deterministic regressor can fit every singleton exactly.
    # For a duplicate input its squared-error optimum is the group label mean.
    lower_sse = 0.0
    lower_max = 0.0
    for group in conflicting:
        labels = actions[group].astype(np.float64)
        lower_sse += float(np.sum(np.square(labels - np.mean(labels, axis=0))))
        lower_max = max(
            lower_max,
            float(np.max(np.ptp(labels, axis=0) / 2.0)),
        )
    return {
        "projected_feature_count": int(projected.shape[1]),
        "unique_observation_count": int(len(groups)),
        "duplicate_group_count": int(len(duplicate)),
        "duplicate_row_count": int(sum(len(group) for group in duplicate)),
        "largest_duplicate_group": int(max(map(len, duplicate), default=1)),
        "conflicting_group_count": int(len(conflicting)),
        "conflicting_row_count": int(sum(len(group) for group in conflicting)),
        "collision_rmse_lower_bound": float(math.sqrt(lower_sse / actions.size)),
        "collision_max_abs_lower_bound": float(lower_max),
    }


def _nearest_distinct_audit(
    observations: np.ndarray,
    actions: np.ndarray,
    normalization: Any,
) -> dict[str, Any]:
    from scipy.spatial import cKDTree

    effective = np.ascontiguousarray(
        r9.v11.normalized_observations(observations, normalization)[:, 2:],
        dtype=np.float64,
    )
    unique_rows: dict[bytes, int] = {}
    for index, row in enumerate(effective):
        unique_rows.setdefault(row.tobytes(), index)
    representatives = np.asarray(list(unique_rows.values()), dtype=np.int64)
    values = effective[representatives]
    distances, neighbors = cKDTree(values).query(values, k=2, workers=1)
    nearest_distance = distances[:, 1]
    nearest_index = representatives[neighbors[:, 1]]
    label_delta = np.max(
        np.abs(actions[representatives].astype(np.float64) - actions[nearest_index]),
        axis=1,
    )
    audit: dict[str, Any] = {
        "normalization": "frozen R9 base-v8; disabled clock columns excluded",
        "unique_effective_rows": int(len(representatives)),
        "minimum_nonzero_distance": float(np.min(nearest_distance)),
        "distance_quantiles": {
            str(q): float(np.quantile(nearest_distance, q))
            for q in (0.0, 0.001, 0.01, 0.05, 0.5)
        },
        "nearest_pair_max_label_delta": float(np.max(label_delta)),
    }
    for limit in (1.0e-6, 1.0e-5, 1.0e-4, 1.0e-3, 1.0e-2):
        selected = nearest_distance <= limit
        audit[f"distance_le_{limit:g}"] = {
            "directed_row_count": int(np.count_nonzero(selected)),
            "max_label_delta": float(np.max(label_delta[selected], initial=0.0)),
        }
    ratio = label_delta / np.maximum(nearest_distance, np.finfo(np.float64).tiny)
    worst = int(np.argmax(ratio))
    audit["largest_local_label_slope"] = {
        "max_action_delta_over_normalized_l2": float(ratio[worst]),
        "distance": float(nearest_distance[worst]),
        "max_action_delta": float(label_delta[worst]),
        "row_index": int(representatives[worst]),
        "neighbor_row_index": int(nearest_index[worst]),
    }
    return audit


def _reset_audit(
    predictions: np.ndarray,
    arrays: Mapping[str, np.ndarray],
    source_weights: np.ndarray,
) -> dict[str, Any]:
    reset = np.flatnonzero(arrays["reset_mask"])
    groups: dict[bytes, list[int]] = collections.defaultdict(list)
    effective = np.ascontiguousarray(arrays["observations"][:, 2:])
    for index in reset:
        groups[effective[index].tobytes()].append(int(index))
    output = []
    for group_id, indices in enumerate(groups.values(), start=1):
        labels = arrays["actions"][indices]
        estimate = predictions[indices]
        output.append(
            {
                "group_id": group_id,
                "row_count": len(indices),
                "row_indices": indices,
                "stratum_ids": sorted(set(arrays["stratum_ids"][indices].astype(str))),
                "label": labels[0].astype(float).tolist(),
                "labels_byte_exact": bool(np.all(labels == labels[0])),
                "prediction": estimate[0].astype(float).tolist(),
                "predictions_byte_exact": bool(np.all(estimate == estimate[0])),
                "max_abs_error": float(
                    np.max(np.abs(estimate.astype(np.float64) - labels))
                ),
                "source_weight_min": float(np.min(source_weights[indices])),
                "source_weight_max": float(np.max(source_weights[indices])),
            }
        )
    return {
        "row_count": int(len(reset)),
        "unique_effective_state_count": int(len(groups)),
        "groups": output,
    }


def _top_error_rows(
    predictions: np.ndarray,
    arrays: Mapping[str, np.ndarray],
    source_weights: np.ndarray,
    *,
    limit: int = 20,
) -> list[dict[str, Any]]:
    absolute = np.abs(
        predictions.astype(np.float64) - arrays["actions"].astype(np.float64)
    )
    flat_order = np.argsort(absolute, axis=None)[::-1][:limit]
    output = []
    for flat in flat_order:
        row, action = np.unravel_index(int(flat), absolute.shape)
        output.append(
            {
                "row_index": int(row),
                "action_dimension": int(action),
                "absolute_error": float(absolute[row, action]),
                "prediction": float(predictions[row, action]),
                "target": float(arrays["actions"][row, action]),
                "stratum_id": str(arrays["stratum_ids"][row]),
                "case_id": str(arrays["case_ids"][row]),
                "step_index": int(arrays["step_indices"][row]),
                "tranche_id": str(arrays["tranche_ids"][row]),
                "reset": bool(arrays["reset_mask"][row]),
                "source_equal_stratum_weight": float(source_weights[row]),
            }
        )
    return output


def analyze() -> dict[str, Any]:
    summary = _strict_json(SUMMARY_PATH)
    persisted_gate = _strict_json(GATE_PATH)
    ledger = _strict_json(LEDGER_PATH)
    if (
        ledger.get("status") != EXPECTED_TERMINAL_STATUS
        or ledger.get("passed") is not False
        or ledger.get("attempted_stage") != "fit_recovery_actor"
    ):
        raise RuntimeError("R9 is not the expected immutable terminal fit failure")
    with np.load(CORPUS_PATH, allow_pickle=False) as loaded:
        arrays = {name: np.ascontiguousarray(loaded[name]) for name in loaded.files}
    state, predictions = _candidate_state_and_predictions(arrays["observations"])
    current_metrics = _metric_payload(predictions, arrays)
    if current_metrics["global_metrics"] != summary["global_metrics"]:
        raise RuntimeError("candidate predictions do not close persisted R9 metrics")
    current_gate = _gate(current_metrics)
    if current_gate != persisted_gate:
        raise RuntimeError("recomputed R9 fit gate drifted")
    source_weights, source_weight_audit = _source_piece_weights(
        arrays, source_key="normalized_sample_weights"
    )
    risk_weights, risk_weight_audit = _source_piece_weights(
        arrays, source_key="raw_sample_weights"
    )
    base_rows = int(r9.contract.LOCKED_INPUTS["base_corpus"]["rows"])
    base_v8 = np.flatnonzero(
        arrays["tranche_ids"][:base_rows].astype(str) == "v8r1p1_base"
    )
    normalization = r9.v11.frozen_base_normalization(arrays["observations"][base_v8])
    design = _hidden_design(state, arrays["observations"])
    experiments: dict[str, Any] = {}
    weight_variants = {
        "source_normalized_equal_stratum": source_weights,
        "source_normalized_equal_stratum_reset_x100": (
            _reset_amplified_equal_stratum_weights(
                source_weights, arrays, multiplier=100.0
            )
        ),
        "source_raw_risk_equal_stratum": risk_weights,
        "uniform_equal_stratum_reset_x100": (
            _reset_amplified_equal_stratum_weights(
                arrays["normalized_sample_weights"], arrays, multiplier=100.0
            )
        ),
    }
    for name, weights in weight_variants.items():
        estimate, solver_audit = _fixed_hidden_weighted_least_squares(
            design, arrays["actions"], weights
        )
        metrics = _metric_payload(estimate, arrays)
        experiments[name] = {
            "method": "fixed_R9_hidden_weighted_ridge_normal_equations",
            "solver": solver_audit,
            "metrics": metrics,
            "gate": _gate(metrics),
            "stratum_errors": _stratum_error_audit(estimate, arrays, weights),
            "reset": _reset_audit(estimate, arrays, source_weights),
            "top_error_rows": _top_error_rows(
                estimate, arrays, source_weights, limit=10
            ),
        }
    teacher_predictions = _teacher_student_view_predictions(arrays["observations"])
    teacher_metrics = _metric_payload(teacher_predictions, arrays)
    all_columns = tuple(range(arrays["observations"].shape[1]))
    effective_columns = tuple(
        index
        for index in all_columns
        if index not in r9.contract.DISABLED_CLOCK_COLUMNS
    )
    return {
        "status": "COMPLETE_H0_V12R10_READ_ONLY_R9_FIT_FORENSICS",
        "passed": True,
        "scope": "FROZEN_R9_CORPUS_AND_CANDIDATE_NO_ENVIRONMENT_NO_PUBLICATION",
        "frozen_inputs": {
            "corpus_path": CORPUS_PATH.relative_to(REPO_ROOT).as_posix(),
            "corpus_sha256": _sha256(CORPUS_PATH),
            "candidate_path": CANDIDATE_PATH.relative_to(REPO_ROOT).as_posix(),
            "candidate_tree_sha256": summary["candidate_module"]["tree_sha256"],
            "summary_sha256": _sha256(SUMMARY_PATH),
            "gate_sha256": _sha256(GATE_PATH),
            "ledger_sha256": _sha256(LEDGER_PATH),
            "terminal_status": ledger["status"],
        },
        "thresholds": r9.contract.OFFLINE_THRESHOLDS,
        "current_r9": {
            "metrics": current_metrics,
            "gate": current_gate,
            "stratum_errors_under_r9_uniform_weights": _stratum_error_audit(
                predictions, arrays, arrays["normalized_sample_weights"]
            ),
            "reset": _reset_audit(predictions, arrays, source_weights),
            "top_error_rows": _top_error_rows(
                predictions, arrays, source_weights, limit=20
            ),
        },
        "weight_provenance": {
            "r9_persisted_uniform_within_stratum": bool(
                np.all(arrays["raw_sample_weights"] == 1.0)
            ),
            "r9_persisted_weight_sha256": r9.v10s_fit.array_sha256(
                arrays["normalized_sample_weights"]
            ),
            "source_normalized_equal_stratum_weight_sha256": r9.v10s_fit.array_sha256(
                source_weights
            ),
            "source_raw_risk_equal_stratum_weight_sha256": r9.v10s_fit.array_sha256(
                risk_weights
            ),
            "source_normalized_strata": source_weight_audit,
            "source_raw_risk_strata": risk_weight_audit,
        },
        "collision_lower_bounds": {
            "full_35_features": _exact_collision_audit(
                arrays["observations"], arrays["actions"], columns=all_columns
            ),
            "effective_33_features_clock_zero": _exact_collision_audit(
                arrays["observations"],
                arrays["actions"],
                columns=effective_columns,
            ),
            "reset_effective_states": _exact_collision_audit(
                arrays["observations"][arrays["reset_mask"]],
                arrays["actions"][arrays["reset_mask"]],
                columns=effective_columns,
            ),
            "interpretation": (
                "These are collision-derived lower bounds for an unrestricted "
                "deterministic function; they are not a finite-width tanh "
                "approximation bound."
            ),
        },
        "near_distinct": _nearest_distinct_audit(
            arrays["observations"], arrays["actions"], normalization
        ),
        "teacher_on_student_view": {
            "meaning": (
                "Direct frozen H0 inference on persisted student observations; "
                "labels were queried on causally reconstructed teacher views."
            ),
            "metrics": teacher_metrics,
            "gate": _gate(teacher_metrics),
        },
        "fixed_hidden_counterfactuals": experiments,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        help="Optional JSON destination; must not be inside validation/v12r9.",
    )
    args = parser.parse_args()
    payload = analyze()
    text = json.dumps(payload, indent=2, sort_keys=True) + "\n"
    if args.output is None:
        print(text, end="")
        return
    destination = args.output.expanduser().resolve()
    try:
        destination.relative_to(R9_ROOT.resolve())
    except ValueError:
        pass
    else:
        raise SystemExit("refusing to write a diagnostic below validation/v12r9")
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(text, encoding="utf-8")
    print(destination)


if __name__ == "__main__":
    main()
