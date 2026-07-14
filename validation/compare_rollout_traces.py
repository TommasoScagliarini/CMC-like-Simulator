"""Compare closed-loop behavior from two aligned deterministic rollout traces."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


_JOINTS = ("pros_knee_angle", "pros_ankle_angle")
_SUMMARY_KEYS = (
    "steps",
    "episode_return",
    "reward_mean",
    "phase_valid_cycle_count",
    "grf_penetration_max_m",
    "action_clipped_fraction",
    "end_reason",
)


def _load(path: str | Path) -> Any:
    resolved = Path(path).expanduser().resolve()
    return json.loads(resolved.read_text(encoding="utf-8"))


def _vector_stats(delta: np.ndarray) -> dict[str, Any]:
    values = np.asarray(delta, dtype=np.float64)
    return {
        "rmse": float(np.sqrt(np.mean(np.square(values)))),
        "rmse_per_channel": np.sqrt(np.mean(np.square(values), axis=0)).tolist(),
        "abs_max": float(np.max(np.abs(values))),
        "signed_mean_per_channel": np.mean(values, axis=0).tolist(),
    }


def _rows_to_array(
    rows: Sequence[Mapping[str, Any]], *keys: str
) -> np.ndarray:
    values: list[list[float]] = []
    for row in rows:
        value: Any = row
        for key in keys:
            value = value[key]
        values.append([float(item) for item in value])
    return np.asarray(values, dtype=np.float64)


def _prosthetic_array(
    rows: Sequence[Mapping[str, Any]], suffix: str = ""
) -> np.ndarray:
    return np.asarray(
        [
            [float(row["prosthetic_state"][f"{joint}{suffix}"]) for joint in _JOINTS]
            for row in rows
        ],
        dtype=np.float64,
    )


def _imitation_target_array(rows: Sequence[Mapping[str, Any]]) -> np.ndarray:
    return np.asarray(
        [
            [float(row["imitation_target_q"][joint]) for joint in _JOINTS]
            for row in rows
        ],
        dtype=np.float64,
    )


def _prescribed_distance(rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    target = _imitation_target_array(rows)
    actual = _prosthetic_array(rows)
    served = _prosthetic_array(rows, "_served_ref")
    return {
        "actual_q_minus_prescribed": _vector_stats(actual - target),
        "served_ref_minus_prescribed": _vector_stats(served - target),
    }


def _summary_subset(summary: Mapping[str, Any]) -> dict[str, Any]:
    return {key: summary.get(key) for key in _SUMMARY_KEYS}


def compare(
    reference_trace_path: str | Path,
    candidate_trace_path: str | Path,
    reference_summary_path: str | Path,
    candidate_summary_path: str | Path,
) -> dict[str, Any]:
    reference_rows = _load(reference_trace_path)
    candidate_rows = _load(candidate_trace_path)
    if not isinstance(reference_rows, list) or not isinstance(candidate_rows, list):
        raise ValueError("rollout traces must be JSON lists")
    aligned_steps = min(len(reference_rows), len(candidate_rows))
    if aligned_steps <= 0:
        raise ValueError("rollout traces have no aligned rows")
    reference_rows = reference_rows[:aligned_steps]
    candidate_rows = candidate_rows[:aligned_steps]

    reference_summary = _load(reference_summary_path)
    candidate_summary = _load(candidate_summary_path)
    reference_action = _rows_to_array(reference_rows, "raw_policy_action")
    candidate_action = _rows_to_array(candidate_rows, "raw_policy_action")
    reference_applied = _rows_to_array(reference_rows, "applied_policy_action")
    candidate_applied = _rows_to_array(candidate_rows, "applied_policy_action")
    reference_q = _prosthetic_array(reference_rows)
    candidate_q = _prosthetic_array(candidate_rows)
    reference_ref = _prosthetic_array(reference_rows, "_served_ref")
    candidate_ref = _prosthetic_array(candidate_rows, "_served_ref")

    return {
        "reference_trace": str(Path(reference_trace_path).expanduser().resolve()),
        "candidate_trace": str(Path(candidate_trace_path).expanduser().resolve()),
        "reference_steps": len(_load(reference_trace_path)),
        "candidate_steps": len(_load(candidate_trace_path)),
        "aligned_steps": aligned_steps,
        "channel_order": list(_JOINTS),
        "candidate_minus_reference": {
            "raw_policy_action": _vector_stats(candidate_action - reference_action),
            "applied_policy_action": _vector_stats(
                candidate_applied - reference_applied
            ),
            "prosthetic_q_rad": _vector_stats(candidate_q - reference_q),
            "served_reference_rad": _vector_stats(candidate_ref - reference_ref),
        },
        "reference_distance_from_prescribed": _prescribed_distance(reference_rows),
        "candidate_distance_from_prescribed": _prescribed_distance(candidate_rows),
        "reference_summary": _summary_subset(reference_summary),
        "candidate_summary": _summary_subset(candidate_summary),
        "summary_delta": {
            "episode_return": float(candidate_summary["episode_return"])
            - float(reference_summary["episode_return"]),
            "valid_cycles": int(candidate_summary["phase_valid_cycle_count"])
            - int(reference_summary["phase_valid_cycle_count"]),
            "grf_penetration_max_m": float(
                candidate_summary["grf_penetration_max_m"]
            )
            - float(reference_summary["grf_penetration_max_m"]),
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference-trace", required=True)
    parser.add_argument("--candidate-trace", required=True)
    parser.add_argument("--reference-summary", required=True)
    parser.add_argument("--candidate-summary", required=True)
    parser.add_argument("--output", required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result = compare(
        args.reference_trace,
        args.candidate_trace,
        args.reference_summary,
        args.candidate_summary,
    )
    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
