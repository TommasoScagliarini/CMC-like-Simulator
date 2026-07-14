"""Attribute closed-loop actor drift to deployable observation features.

The diagnostic compares one deterministic nominal rollout with disturbed
rollouts from the same fixed start.  It separates the sampled action noise from
the policy-mean drift caused by the resulting state deviation, then evaluates
single-feature and feature-group counterfactuals through the frozen actor.
"""

from __future__ import annotations

import argparse
import json
import pickle
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


ACTION_NAMES = ("knee", "ankle")


def _array(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    return np.asarray(value, dtype=np.float64)


def _load_json(path: str | Path) -> Any:
    return json.loads(Path(path).expanduser().resolve().read_text(encoding="utf-8"))


def _load_state(path: str | Path) -> Mapping[str, Any]:
    resolved = Path(path).expanduser().resolve()
    state_path = resolved / "module_state.pkl" if resolved.is_dir() else resolved
    with state_path.open("rb") as handle:
        state = pickle.load(handle)
    if not isinstance(state, Mapping):
        raise ValueError(f"module state is not a mapping: {state_path}")
    return state


def _actor_parameters(state: Mapping[str, Any]) -> tuple[np.ndarray, ...]:
    return tuple(
        _array(state[name])
        for name in (
            "pi.0.0.weight",
            "pi.0.0.bias",
            "pi.0.2.weight",
            "pi.0.2.bias",
            "pi.1.weight",
            "pi.1.bias",
        )
    )


def actor_logits(state: Mapping[str, Any], observations: np.ndarray) -> np.ndarray:
    w1, b1, w2, b2, w3, b3 = _actor_parameters(state)
    x = np.asarray(observations, dtype=np.float64)
    h1 = np.tanh(x @ w1.T + b1)
    h2 = np.tanh(h1 @ w2.T + b2)
    return h2 @ w3.T + b3


def actor_mean_jacobian(
    state: Mapping[str, Any], observations: np.ndarray, action_dim: int
) -> np.ndarray:
    """Return d(mean_action)/d(observation) for every observation row."""
    w1, b1, w2, b2, w3, _ = _actor_parameters(state)
    x = np.asarray(observations, dtype=np.float64)
    h1 = np.tanh(x @ w1.T + b1)
    h2 = np.tanh(h1 @ w2.T + b2)
    left = w3[:action_dim][None, :, :] * (1.0 - h2[:, None, :] ** 2)
    middle = np.einsum("nah,hk->nak", left, w2)
    middle *= 1.0 - h1[:, None, :] ** 2
    return np.einsum("nak,kf->naf", middle, w1)


def _observations(rows: Sequence[Mapping[str, Any]]) -> np.ndarray:
    return np.asarray(
        [row["actor_observation_vector_before"] for row in rows], dtype=np.float64
    )


def _actions(rows: Sequence[Mapping[str, Any]]) -> np.ndarray:
    return np.asarray([row["raw_policy_action"] for row in rows], dtype=np.float64)


def _stats(values: np.ndarray) -> dict[str, Any]:
    array = np.asarray(values, dtype=np.float64)
    return {
        "rms": np.sqrt(np.mean(np.square(array), axis=0)).astype(float).tolist(),
        "abs_max": np.max(np.abs(array), axis=0).astype(float).tolist(),
        "mean": np.mean(array, axis=0).astype(float).tolist(),
    }


def _first_step_over(values: np.ndarray, threshold: float) -> int | None:
    magnitude = np.max(np.abs(np.asarray(values, dtype=float)), axis=1)
    indices = np.flatnonzero(magnitude > threshold)
    return int(indices[0] + 1) if indices.size else None


def _feature_groups(feature_names: Sequence[str]) -> dict[str, list[int]]:
    groups: dict[str, list[int]] = {
        "prosthetic_joint_state": [],
        "sea_motor_state": [],
        "ipsilateral_load_and_events": [],
        "deployable_phase_fsm": [],
        "controller_reference_memory": [],
        "sea_command_memory": [],
        "disabled_clock": [],
    }
    for index, name in enumerate(feature_names):
        if name in {"gait_phase_sin", "gait_phase_cos"}:
            groups["disabled_clock"].append(index)
        elif name.startswith("SEA_"):
            groups["sea_motor_state"].append(index)
        elif name.startswith("online_left_"):
            groups["ipsilateral_load_and_events"].append(index)
        elif name.startswith(("phase_fsm_", "phase_expected_", "phase_stance_", "phase_swing_", "phase_cycle_")):
            groups["deployable_phase_fsm"].append(index)
        elif name.endswith(("_previous_endpoint", "_served_ref", "_served_ref_vel", "_served_ref_accel")):
            groups["controller_reference_memory"].append(index)
        elif name.endswith(("_sea_u", "_sea_u_abs", "_sea_u_saturated")):
            groups["sea_command_memory"].append(index)
        elif name.startswith(("pros_knee_angle", "pros_ankle_angle")):
            groups["prosthetic_joint_state"].append(index)
    return {name: indices for name, indices in groups.items() if indices}


def _counterfactual_effect(
    state: Mapping[str, Any],
    nominal_obs: np.ndarray,
    disturbed_obs: np.ndarray,
    nominal_mean: np.ndarray,
    disturbed_mean: np.ndarray,
    action_dim: int,
    indices: Sequence[int],
) -> dict[str, Any]:
    counterfactual = disturbed_obs.copy()
    counterfactual[:, list(indices)] = nominal_obs[:, list(indices)]
    counter_mean = actor_logits(state, counterfactual)[:, :action_dim]
    baseline_error = np.sqrt(np.mean(np.square(disturbed_mean - nominal_mean)))
    counter_error = np.sqrt(np.mean(np.square(counter_mean - nominal_mean)))
    return {
        "mean_drift_rmse_before": float(baseline_error),
        "mean_drift_rmse_after_replacement": float(counter_error),
        "fraction_removed": float(
            (baseline_error - counter_error) / max(baseline_error, 1e-12)
        ),
        "counterfactual_action_change": _stats(counter_mean - disturbed_mean),
    }


def analyze_trace(
    nominal_rows: Sequence[Mapping[str, Any]],
    disturbed_rows: Sequence[Mapping[str, Any]],
    state: Mapping[str, Any],
    feature_names: Sequence[str],
) -> dict[str, Any]:
    count = min(len(nominal_rows), len(disturbed_rows))
    nominal_obs = _observations(nominal_rows[:count])
    disturbed_obs = _observations(disturbed_rows[:count])
    if nominal_obs.shape != disturbed_obs.shape:
        raise ValueError("nominal and disturbed actor observations are not aligned")
    if nominal_obs.shape[1] != len(feature_names):
        raise ValueError("feature manifest width does not match trace observations")

    sampled_actions = _actions(disturbed_rows[:count])
    action_dim = sampled_actions.shape[1]
    nominal_mean = actor_logits(state, nominal_obs)[:, :action_dim]
    disturbed_logits = actor_logits(state, disturbed_obs)
    disturbed_mean = disturbed_logits[:, :action_dim]
    exploration_noise = sampled_actions - disturbed_mean
    configured_sigma = np.exp(disturbed_logits[:, action_dim:])
    empirical_noise_rms = np.sqrt(np.mean(np.square(exploration_noise), axis=0))
    configured_sigma_mean = np.mean(configured_sigma, axis=0)
    noise_sigma_ratio = empirical_noise_rms / np.maximum(configured_sigma_mean, 1e-12)
    mean_drift = disturbed_mean - nominal_mean
    observation_delta = disturbed_obs - nominal_obs
    jacobian = actor_mean_jacobian(state, nominal_obs, action_dim)
    linearized_effect = jacobian * observation_delta[:, None, :]

    feature_rows: list[dict[str, Any]] = []
    for index, name in enumerate(feature_names):
        effect = _counterfactual_effect(
            state,
            nominal_obs,
            disturbed_obs,
            nominal_mean,
            disturbed_mean,
            action_dim,
            [index],
        )
        feature_rows.append(
            {
                "feature": name,
                "index": index,
                "delta_rms": float(np.sqrt(np.mean(observation_delta[:, index] ** 2))),
                "delta_abs_max": float(np.max(np.abs(observation_delta[:, index]))),
                "nominal_abs_max": float(np.max(np.abs(nominal_obs[:, index]))),
                "linearized_action_effect_rms": float(
                    np.sqrt(np.mean(linearized_effect[:, :, index] ** 2))
                ),
                **effect,
            }
        )
    feature_rows.sort(
        key=lambda row: abs(float(row["fraction_removed"])), reverse=True
    )

    group_rows = {
        name: {
            "features": [feature_names[index] for index in indices],
            **_counterfactual_effect(
                state,
                nominal_obs,
                disturbed_obs,
                nominal_mean,
                disturbed_mean,
                action_dim,
                indices,
            ),
        }
        for name, indices in _feature_groups(feature_names).items()
    }
    group_rows = dict(
        sorted(
            group_rows.items(),
            key=lambda item: abs(float(item[1]["fraction_removed"])),
            reverse=True,
        )
    )

    discrete_names = {
        name
        for name in feature_names
        if name.endswith(("_in_contact", "_heel_strike", "_toe_off", "_saturated"))
        or name.startswith(("phase_fsm_", "phase_expected_"))
    }
    discrete_indices = [
        index for index, name in enumerate(feature_names) if name in discrete_names
    ]
    mismatch = (
        np.any(nominal_obs[:, discrete_indices] != disturbed_obs[:, discrete_indices], axis=1)
        if discrete_indices
        else np.zeros(count, dtype=bool)
    )
    mismatch_steps = np.flatnonzero(mismatch)

    return {
        "nominal_steps": len(nominal_rows),
        "disturbed_steps": len(disturbed_rows),
        "aligned_steps": count,
        "action_names": list(ACTION_NAMES[:action_dim]),
        "configured_sigma_min": np.exp(np.min(disturbed_logits[:, action_dim:], axis=0))
        .astype(float)
        .tolist(),
        "trace_noise_contract": {
            "empirical_rms_to_configured_sigma_ratio": noise_sigma_ratio
            .astype(float)
            .tolist(),
            "compatible": bool(np.all((noise_sigma_ratio >= 0.5) & (noise_sigma_ratio <= 1.5))),
        },
        "configured_sigma_max": np.exp(np.max(disturbed_logits[:, action_dim:], axis=0))
        .astype(float)
        .tolist(),
        "sampled_exploration_noise": _stats(exploration_noise),
        "closed_loop_policy_mean_drift": {
            **_stats(mean_drift),
            "first_step_abs_over_0_003": _first_step_over(mean_drift, 0.003),
            "first_step_abs_over_0_005": _first_step_over(mean_drift, 0.005),
            "first_step_abs_over_0_01": _first_step_over(mean_drift, 0.01),
            "first_step_abs_over_0_05": _first_step_over(mean_drift, 0.05),
        },
        "observation_discrete_phase": {
            "first_mismatch_step": (
                int(mismatch_steps[0] + 1) if mismatch_steps.size else None
            ),
            "mismatch_fraction": float(np.mean(mismatch)),
        },
        "feature_counterfactual_ranking": feature_rows,
        "group_counterfactual_ranking": group_rows,
    }


def run(args: argparse.Namespace) -> dict[str, Any]:
    nominal = _load_json(args.nominal_trace)
    manifest = _load_json(args.feature_manifest)
    feature_names = [str(name) for name in manifest["actor_feature_names"]]
    state = _load_state(args.checkpoint)
    reports = []
    for trace_path in args.disturbed_trace:
        report = analyze_trace(nominal, _load_json(trace_path), state, feature_names)
        report["trace"] = str(Path(trace_path).expanduser().resolve())
        reports.append(report)
    payload = {
        "checkpoint": str(Path(args.checkpoint).expanduser().resolve()),
        "nominal_trace": str(Path(args.nominal_trace).expanduser().resolve()),
        "feature_manifest": str(Path(args.feature_manifest).expanduser().resolve()),
        "traces": reports,
    }
    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return payload


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--feature-manifest", required=True)
    parser.add_argument("--nominal-trace", required=True)
    parser.add_argument("--disturbed-trace", action="append", required=True)
    parser.add_argument("--output", required=True)
    return parser.parse_args()


if __name__ == "__main__":
    print(json.dumps(run(parse_args()), indent=2))
