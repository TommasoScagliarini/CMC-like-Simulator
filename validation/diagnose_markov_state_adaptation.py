"""Diagnose a selective controller-state adaptation against its zero-column source."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
BASELINE_DIR = ROOT / "Trajectory Generator" / "baseline_MLP"
if str(BASELINE_DIR) not in sys.path:
    sys.path.insert(0, str(BASELINE_DIR))

import target_domain_markov_adaptation as markov
import target_domain_noise_adaptation as noise
import warm_start


def _metrics(prediction: np.ndarray, target: np.ndarray) -> dict[str, Any]:
    error = np.asarray(prediction, float) - np.asarray(target, float)
    absolute = np.abs(error)
    max_flat_index = int(np.argmax(absolute))
    max_sample, max_action = np.unravel_index(max_flat_index, absolute.shape)
    return {
        "rmse": float(np.sqrt(np.mean(np.square(error)))),
        "max_abs": float(absolute[max_sample, max_action]),
        "p95_abs": float(np.percentile(absolute, 95.0)),
        "p99_abs": float(np.percentile(absolute, 99.0)),
        "max_abs_sample": int(max_sample + 1),
        "max_abs_action": int(max_action),
        "per_action_rmse": np.sqrt(np.mean(np.square(error), axis=0)).tolist(),
    }


def _feature_stats(
    observations: np.ndarray,
    feature_names: list[str],
    selected_names: tuple[str, ...],
) -> dict[str, Any]:
    result = {}
    for name in selected_names:
        values = np.asarray(observations[:, feature_names.index(name)], float)
        result[name] = {
            "mean": float(np.mean(values)),
            "std": float(np.std(values)),
            "min": float(np.min(values)),
            "max": float(np.max(values)),
            "abs_max": float(np.max(np.abs(values))),
        }
    return result


def _dataset_report(
    *,
    observations: np.ndarray,
    targets: np.ndarray,
    feature_names: list[str],
    selected_names: tuple[str, ...],
    source_state: dict[str, Any],
    candidate_state: dict[str, Any],
) -> dict[str, Any]:
    source_means = noise._forward(source_state, observations)[:, : targets.shape[1]]
    candidate_means = noise._forward(candidate_state, observations)[:, : targets.shape[1]]
    shift = candidate_means - source_means
    absolute_shift = np.abs(shift)
    shift_flat_index = int(np.argmax(absolute_shift))
    shift_sample, shift_action = np.unravel_index(
        shift_flat_index, absolute_shift.shape
    )
    source_weight = warm_start._as_numpy(source_state["pi_encoder.0.weight"])
    candidate_weight = warm_start._as_numpy(
        candidate_state["pi_encoder.0.weight"]
    )
    weight_delta = candidate_weight - source_weight
    selected_indices = [feature_names.index(name) for name in selected_names]
    preactivation_delta = observations[:, selected_indices] @ weight_delta[
        :, selected_indices
    ].T
    per_feature_contribution = {}
    for name, index in zip(selected_names, selected_indices):
        contribution = np.outer(observations[:, index], weight_delta[:, index])
        per_feature_contribution[name] = {
            "column_norm": float(np.linalg.norm(weight_delta[:, index])),
            "preactivation_rms": float(np.sqrt(np.mean(np.square(contribution)))),
            "preactivation_abs_max": float(np.max(np.abs(contribution))),
        }
    return {
        "samples": int(len(observations)),
        "feature_stats": _feature_stats(
            observations, feature_names, selected_names
        ),
        "source_vs_target": _metrics(source_means, targets),
        "candidate_vs_target": _metrics(candidate_means, targets),
        "candidate_minus_source_action": {
            "rms": float(np.sqrt(np.mean(np.square(shift)))),
            "max_abs": float(absolute_shift[shift_sample, shift_action]),
            "p95_abs": float(np.percentile(absolute_shift, 95.0)),
            "p99_abs": float(np.percentile(absolute_shift, 99.0)),
            "max_abs_sample": int(shift_sample + 1),
            "max_abs_action": int(shift_action),
            "first": shift[0].astype(float).tolist(),
        },
        "new_columns_hidden_preactivation": {
            "rms": float(np.sqrt(np.mean(np.square(preactivation_delta)))),
            "max_abs": float(np.max(np.abs(preactivation_delta))),
            "first_l2": float(np.linalg.norm(preactivation_delta[0])),
            "max_l2": float(
                np.max(np.linalg.norm(preactivation_delta, axis=1))
            ),
            "per_feature": per_feature_contribution,
        },
        "first_actions": {
            "target": targets[0].astype(float).tolist(),
            "source": source_means[0].astype(float).tolist(),
            "candidate": candidate_means[0].astype(float).tolist(),
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-checkpoint", required=True)
    parser.add_argument("--candidate-checkpoint", required=True)
    parser.add_argument("--nominal-trace", required=True)
    parser.add_argument("--recovery-trace", action="append", default=[])
    parser.add_argument("--teacher-dataset", action="append", default=[])
    parser.add_argument("--output")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    nominal_rows = json.loads(Path(args.nominal_trace).read_text(encoding="utf-8"))
    feature_names = list(nominal_rows[0]["actor_observation_before"])
    selected_names = markov.markov_feature_names(feature_names)
    nominal_observations = np.asarray(
        [row["actor_observation_vector_before"] for row in nominal_rows],
        dtype=np.float32,
    )
    nominal_targets = np.asarray(
        [row["raw_policy_action"] for row in nominal_rows], dtype=np.float32
    )
    source_state = warm_start.load_module_state(Path(args.source_checkpoint))
    candidate_state = warm_start.load_module_state(Path(args.candidate_checkpoint))

    report: dict[str, Any] = {
        "source_checkpoint": str(Path(args.source_checkpoint).resolve()),
        "candidate_checkpoint": str(Path(args.candidate_checkpoint).resolve()),
        "selected_features": list(selected_names),
        "nominal": _dataset_report(
            observations=nominal_observations,
            targets=nominal_targets,
            feature_names=feature_names,
            selected_names=selected_names,
            source_state=source_state,
            candidate_state=candidate_state,
        ),
        "teacher_datasets": [],
        "recovery_phase_alignment": [],
    }
    for value in args.teacher_dataset:
        path = Path(value)
        with np.load(path) as archive:
            observations = np.asarray(archive["observations"], np.float32)
            targets = np.asarray(archive["actions"], np.float32)
            names = [str(name) for name in archive["actor_feature_names"].tolist()]
        if names != feature_names:
            raise ValueError(f"actor schema mismatch: {path}")
        report["teacher_datasets"].append(
            {
                "path": str(path.resolve()),
                **_dataset_report(
                    observations=observations,
                    targets=targets,
                    feature_names=feature_names,
                    selected_names=selected_names,
                    source_state=source_state,
                    candidate_state=candidate_state,
                ),
            }
        )
    for value in args.recovery_trace:
        path = Path(value)
        rows = json.loads(path.read_text(encoding="utf-8"))
        _, alignment = noise.truncate_before_discrete_mismatch(
            nominal_rows, rows, feature_names
        )
        report["recovery_phase_alignment"].append(
            {"path": str(path.resolve()), **alignment}
        )
    original = sum(
        item["original_steps"] for item in report["recovery_phase_alignment"]
    )
    retained = sum(
        item["retained_steps"] for item in report["recovery_phase_alignment"]
    )
    report["recovery_label_audit"] = {
        "original_samples": original,
        "phase_aligned_samples": retained,
        "phase_misaligned_samples": original - retained,
        "phase_misaligned_fraction": (
            float((original - retained) / original) if original else 0.0
        ),
    }

    text = json.dumps(report, indent=2)
    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
