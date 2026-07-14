"""Compare a stochastic actor rollout with its deterministic baseline trace."""

from __future__ import annotations

import argparse
import json
import pickle
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


ACTION_NAMES = ("knee", "ankle")
STATE_KEYS = {
    "joint_q": ("pros_knee_angle", "pros_ankle_angle"),
    "served_reference": (
        "pros_knee_angle_served_ref",
        "pros_ankle_angle_served_ref",
    ),
}


def _array(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    return np.asarray(value)


def _load_trace(path: str | Path) -> list[dict[str, Any]]:
    resolved = Path(path).expanduser().resolve()
    rows = json.loads(resolved.read_text(encoding="utf-8"))
    if not isinstance(rows, list) or not rows:
        raise ValueError(f"expected a non-empty JSON trace list: {resolved}")
    if not all(isinstance(row, Mapping) for row in rows):
        raise ValueError(f"trace contains non-object rows: {resolved}")
    return [dict(row) for row in rows]


def _actor_logits(
    state: Mapping[str, Any], observations: np.ndarray
) -> np.ndarray:
    hidden = np.tanh(
        observations @ _array(state["pi.0.0.weight"]).T
        + _array(state["pi.0.0.bias"])
    )
    hidden = np.tanh(
        hidden @ _array(state["pi.0.2.weight"]).T
        + _array(state["pi.0.2.bias"])
    )
    return hidden @ _array(state["pi.1.weight"]).T + _array(state["pi.1.bias"])


def _vectors(rows: Sequence[Mapping[str, Any]], key: str) -> np.ndarray:
    return np.asarray([row[key] for row in rows], dtype=float)


def _prosthetic_vectors(
    rows: Sequence[Mapping[str, Any]], keys: Sequence[str]
) -> np.ndarray:
    return np.asarray(
        [[row["prosthetic_state"][key] for key in keys] for row in rows],
        dtype=float,
    )


def _reward_values(
    rows: Sequence[Mapping[str, Any]], key: str, default: float = 0.0
) -> np.ndarray:
    return np.asarray(
        [float(row["reward_terms"].get(key, default)) for row in rows],
        dtype=float,
    )


def _first_step_over(
    rows: Sequence[Mapping[str, Any]], values: np.ndarray, threshold: float
) -> int | None:
    indices = np.flatnonzero(values > threshold)
    return int(rows[int(indices[0])]["step"]) if indices.size else None


def _first_step_at_least(
    rows: Sequence[Mapping[str, Any]], values: np.ndarray, threshold: float
) -> int | None:
    indices = np.flatnonzero(values >= threshold)
    return int(rows[int(indices[0])]["step"]) if indices.size else None


def _event_steps(
    rows: Sequence[Mapping[str, Any]], cumulative_key: str
) -> list[int]:
    events: list[int] = []
    previous = 0
    for row in rows:
        current = int(round(float(row["reward_terms"].get(cumulative_key, 0.0))))
        if current > previous:
            events.extend([int(row["step"])] * (current - previous))
        previous = current
    return events


def _component_statistics(values: np.ndarray) -> dict[str, list[float]]:
    return {
        "mean": np.mean(values, axis=0).astype(float).tolist(),
        "std": np.std(values, axis=0).astype(float).tolist(),
        "rms": np.sqrt(np.mean(np.square(values), axis=0)).astype(float).tolist(),
        "abs_max": np.max(np.abs(values), axis=0).astype(float).tolist(),
    }


def analyze(
    deterministic_rows: Sequence[Mapping[str, Any]],
    stochastic_rows: Sequence[Mapping[str, Any]],
    actor_state: Mapping[str, Any],
) -> dict[str, Any]:
    aligned_count = min(len(deterministic_rows), len(stochastic_rows))
    deterministic = deterministic_rows[:aligned_count]
    stochastic = stochastic_rows[:aligned_count]
    deterministic_steps = [int(row["step"]) for row in deterministic]
    stochastic_steps = [int(row["step"]) for row in stochastic]
    if deterministic_steps != stochastic_steps:
        raise ValueError("deterministic and stochastic trace steps are not aligned")

    stochastic_observations = _vectors(
        stochastic_rows, "actor_observation_vector_before"
    ).astype(np.float32)
    stochastic_actions_full = _vectors(stochastic_rows, "raw_policy_action")
    logits = _actor_logits(actor_state, stochastic_observations)
    action_dim = stochastic_actions_full.shape[1]
    if logits.shape[1] != 2 * action_dim:
        raise ValueError("actor logits do not contain mean and log-std per action")
    policy_means = logits[:, :action_dim]
    log_std = logits[:, action_dim:]
    exploration_noise = stochastic_actions_full - policy_means

    deterministic_actions = _vectors(deterministic, "raw_policy_action")
    stochastic_actions = _vectors(stochastic, "raw_policy_action")
    raw_action_diff = stochastic_actions - deterministic_actions
    state_divergence: dict[str, Any] = {}
    for label, keys in STATE_KEYS.items():
        deterministic_values = _prosthetic_vectors(deterministic, keys)
        stochastic_values = _prosthetic_vectors(stochastic, keys)
        difference = stochastic_values - deterministic_values
        state_divergence[label] = {
            "first_step_abs_diff_over_0_01_rad": _first_step_over(
                stochastic, np.max(np.abs(difference), axis=1), 0.01
            ),
            "abs_max_diff_rad": float(np.max(np.abs(difference))),
            "component_abs_max_diff_rad": np.max(np.abs(difference), axis=0)
            .astype(float)
            .tolist(),
        }

    deterministic_penetration = _reward_values(deterministic_rows, "grf_penetration_m")
    stochastic_penetration = _reward_values(stochastic_rows, "grf_penetration_m")
    aligned_penetration_diff = (
        stochastic_penetration[:aligned_count]
        - deterministic_penetration[:aligned_count]
    )
    penetration_thresholds = {}
    for threshold_m in (0.012, 0.015, 0.020, 0.025):
        label = f"{int(round(threshold_m * 1000))}_mm"
        penetration_thresholds[label] = {
            "deterministic_step": _first_step_at_least(
                deterministic_rows, deterministic_penetration, threshold_m
            ),
            "stochastic_step": _first_step_at_least(
                stochastic_rows, stochastic_penetration, threshold_m
            ),
        }

    slew_limited = _reward_values(
        stochastic_rows, "target_slew_limited_fraction"
    )
    event_keys = {
        "valid_hs": "phase_valid_hs_count",
        "valid_to": "phase_valid_to_count",
        "valid_cycle": "phase_valid_cycle_count",
    }
    return {
        "deterministic_samples": len(deterministic_rows),
        "stochastic_samples": len(stochastic_rows),
        "aligned_samples": aligned_count,
        "action_names": list(ACTION_NAMES[:action_dim]),
        "exploration_noise": {
            **_component_statistics(exploration_noise),
            "configured_std_min": np.exp(np.min(log_std, axis=0))
            .astype(float)
            .tolist(),
            "configured_std_max": np.exp(np.max(log_std, axis=0))
            .astype(float)
            .tolist(),
        },
        "raw_action_divergence": {
            "first_step_abs_diff_over_0_05": _first_step_over(
                stochastic, np.max(np.abs(raw_action_diff), axis=1), 0.05
            ),
            "abs_max_diff": float(np.max(np.abs(raw_action_diff))),
            "component_abs_max_diff": np.max(np.abs(raw_action_diff), axis=0)
            .astype(float)
            .tolist(),
            "consecutive_delta_rms": np.sqrt(
                np.mean(np.square(np.diff(stochastic_actions_full, axis=0)), axis=0)
            )
            .astype(float)
            .tolist(),
            "policy_mean_consecutive_delta_rms": np.sqrt(
                np.mean(np.square(np.diff(policy_means, axis=0)), axis=0)
            )
            .astype(float)
            .tolist(),
        },
        "state_divergence": state_divergence,
        "penetration_divergence": {
            "first_step_abs_diff_over_2_mm": _first_step_over(
                stochastic, np.abs(aligned_penetration_diff), 0.002
            ),
            "abs_max_diff_mm": float(np.max(np.abs(aligned_penetration_diff)) * 1000),
            "final_aligned_stochastic_mm": float(
                stochastic_penetration[aligned_count - 1] * 1000
            ),
            "final_aligned_deterministic_mm": float(
                deterministic_penetration[aligned_count - 1] * 1000
            ),
            "threshold_steps": penetration_thresholds,
        },
        "target_slew_limiter": {
            "active_step_fraction": float(np.mean(slew_limited > 0.0)),
            "mean_limited_fraction": float(np.mean(slew_limited)),
        },
        "event_steps": {
            label: {
                "deterministic": _event_steps(deterministic_rows, key),
                "stochastic": _event_steps(stochastic_rows, key),
            }
            for label, key in event_keys.items()
        },
    }


def run(args: argparse.Namespace) -> dict[str, Any]:
    with Path(args.module_state).expanduser().resolve().open("rb") as handle:
        actor_state = pickle.load(handle)
    report = analyze(
        _load_trace(args.deterministic_trace),
        _load_trace(args.stochastic_trace),
        actor_state,
    )
    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--deterministic-trace", required=True)
    parser.add_argument("--stochastic-trace", required=True)
    parser.add_argument("--module-state", required=True)
    parser.add_argument("--output", required=True)
    return parser.parse_args()


if __name__ == "__main__":
    print(json.dumps(run(parse_args()), indent=2))
