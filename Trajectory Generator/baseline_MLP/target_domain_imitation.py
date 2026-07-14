"""Actor-only imitation adaptation under the frozen ex-novo target contract.

The teacher follows the prescribed prosthetic IK through the same absolute
action mapping, target slew limiter, reference governor, online contact, and
FSM used by ex-novo PPO. The student starts from an already-ported target
RLModule and sees only the target actor observation prefix.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import shutil
import sys
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Mapping, Sequence

import numpy as np

THIS_DIR = Path(__file__).resolve().parent
TRAJ_GEN_DIR = THIS_DIR.parent
REPO_ROOT = TRAJ_GEN_DIR.parent
TRAINING_RUNS_ROOT = TRAJ_GEN_DIR / "runs" / "training"
for path in (THIS_DIR, TRAJ_GEN_DIR, REPO_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import _bootstrap  # noqa: E402

_bootstrap.ensure_sim_paths()

import env_factory  # noqa: E402
import exploration_noise  # noqa: E402
import training_config  # noqa: E402
import warm_start  # noqa: E402


DEFAULT_CONFIG = THIS_DIR / "training_exnovo_cfg.yaml"


def _cli_path(value: str | Path) -> Path:
    text = os.fspath(value)
    if os.name != "nt":
        text = text.replace("\\", "/")
    return Path(text).expanduser()


def _resolve_input(value: str | Path) -> Path:
    path = _cli_path(value)
    if not path.is_absolute():
        path = Path.cwd() / path
    path = path.resolve()
    if not path.exists():
        raise FileNotFoundError(path)
    return path


def _resolve_output(value: str | None) -> Path:
    if value:
        path = _cli_path(value)
        if not path.is_absolute():
            path = Path.cwd() / path
        return path.resolve()
    return (
        TRAINING_RUNS_ROOT
        / f"target_domain_imitation_{datetime.now():%Y%m%d_%H%M%S}"
    ).resolve()


def _resolved_flat_config(config_path: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    nested = training_config.load(config_path)
    flat, reward = training_config.to_argparse_defaults(nested)
    defaults: dict[str, Any] = {
        "action_mode": "absolute",
        "max_delta_rad": 0.35,
        "segment_duration": 0.01,
        "episode_duration": 5.0,
        "episode_start_offset_s": 0.0,
        "policy_knots": 1,
        "random_init": False,
        "step_wall_timeout_s": 60.0,
        "pros_knee_target_slew_rate_limit_rad_s": 0.0,
        "pros_ankle_target_slew_rate_limit_rad_s": 0.0,
        "pros_ref_governor": True,
        "pros_ref_model": "second_order",
        "pros_ref_cutoff_hz": 6.0,
        "pros_knee_ref_velocity_limit_rad_s": 6.0,
        "pros_ankle_ref_velocity_limit_rad_s": 3.5,
        "pros_knee_ref_acceleration_limit_rad_s2": 60.0,
        "pros_ankle_ref_acceleration_limit_rad_s2": 55.0,
        "pros_knee_ref_jerk_limit_rad_s3": 3000.0,
        "pros_ankle_ref_jerk_limit_rad_s3": 2750.0,
        "gait_clock_enable": True,
        "actor_cyclic_phase_only": False,
        "include_reference_state_observation": False,
        "include_controller_state_observation": True,
        "include_controller_diagnostic_observation": True,
        "deployable_minimal_observation": False,
        "imitation_initialize_to_target": True,
        "reward_reference_range_floor": 0.05,
        "reward_reference_velocity_range_floor": 0.1,
        "grf_mode": "online_sensor",
        "online_grf_profile": env_factory.DEFAULT_NETWORK_ONLINE_GRF_PROFILE,
        "online_grf_detector_profile": None,
        "online_grf_observation": True,
        "online_grf_applied_side": [],
        "disable_prescribed_grf_side": [],
        "grf_penetration_penalty_threshold_m": 0.012,
        "grf_penetration_termination_m": 0.017,
        "asymmetric_actor_critic": False,
        "seed": 123,
    }
    defaults.update(flat)
    return defaults, reward


def _validate_target_contract(flat: Mapping[str, Any]) -> None:
    errors = []
    if str(flat["action_mode"]).lower() != "absolute":
        errors.append("action_mode must be absolute")
    if int(flat["policy_knots"]) != 1:
        errors.append("policy_knots must be 1")
    if bool(flat["random_init"]):
        errors.append("random_init must be false")
    if bool(flat["gait_clock_enable"]):
        errors.append("gait_clock_enable must be false")
    if not bool(flat["asymmetric_actor_critic"]):
        errors.append("asymmetric_actor_critic must be true")
    if float(flat["pros_knee_target_slew_rate_limit_rad_s"]) <= 0.0:
        errors.append("knee target slew limiter must be enabled")
    if float(flat["pros_ankle_target_slew_rate_limit_rad_s"]) <= 0.0:
        errors.append("ankle target slew limiter must be enabled")
    if errors:
        raise ValueError("invalid target-domain imitation contract: " + "; ".join(errors))


def build_target_env_config(
    flat: Mapping[str, Any],
    reward: Mapping[str, Any],
    *,
    output_dir: Path,
    record_outputs: bool,
) -> dict[str, Any]:
    return {
        "setup_xml_path": flat["setup_xml"],
        "segment_duration": float(flat["segment_duration"]),
        "episode_duration": float(flat["episode_duration"]),
        "episode_start_offset_s": float(flat["episode_start_offset_s"]),
        "policy_knots": int(flat["policy_knots"]),
        "action_mode": str(flat["action_mode"]),
        "max_delta_rad": float(flat["max_delta_rad"]),
        "target_slew_rate_limit_rad_s": {
            "pros_knee_angle": float(
                flat["pros_knee_target_slew_rate_limit_rad_s"]
            ),
            "pros_ankle_angle": float(
                flat["pros_ankle_target_slew_rate_limit_rad_s"]
            ),
        },
        "enable_pros_ref_governor": bool(flat["pros_ref_governor"]),
        "pros_ref_model": str(flat["pros_ref_model"]),
        "pros_ref_lpf_cutoff_hz": float(flat["pros_ref_cutoff_hz"]),
        "pros_ref_velocity_limit_rad_s": {
            "pros_knee_angle": float(flat["pros_knee_ref_velocity_limit_rad_s"]),
            "pros_ankle_angle": float(flat["pros_ankle_ref_velocity_limit_rad_s"]),
        },
        "pros_ref_acceleration_limit_rad_s2": {
            "pros_knee_angle": float(
                flat["pros_knee_ref_acceleration_limit_rad_s2"]
            ),
            "pros_ankle_angle": float(
                flat["pros_ankle_ref_acceleration_limit_rad_s2"]
            ),
        },
        "pros_ref_jerk_limit_rad_s3": {
            "pros_knee_angle": float(flat["pros_knee_ref_jerk_limit_rad_s3"]),
            "pros_ankle_angle": float(flat["pros_ankle_ref_jerk_limit_rad_s3"]),
        },
        "gait_clock_enable": bool(flat["gait_clock_enable"]),
        "actor_cyclic_phase_only": bool(flat["actor_cyclic_phase_only"]),
        "include_reference_state_observation": bool(
            flat["include_reference_state_observation"]
        ),
        "include_controller_state_observation": bool(
            flat["include_controller_state_observation"]
        ),
        "include_controller_diagnostic_observation": bool(
            flat["include_controller_diagnostic_observation"]
        ),
        "deployable_minimal_observation": bool(
            flat["deployable_minimal_observation"]
        ),
        "imitation_initialize_to_target": bool(
            flat["imitation_initialize_to_target"]
        ),
        "reward_reference_range_floor": float(flat["reward_reference_range_floor"]),
        "reward_reference_velocity_range_floor": float(
            flat["reward_reference_velocity_range_floor"]
        ),
        "random_init": False,
        "rebuild_model_on_reset": False,
        "fail_fast": True,
        "record_outputs": bool(record_outputs),
        "save_outputs_on_close": bool(record_outputs),
        "output_dir": str(output_dir / "teacher_sim_outputs"),
        "output_prefix": "target_teacher",
        "grf_mode": str(flat["grf_mode"]),
        "online_grf_profile_file": flat["online_grf_profile"],
        "online_grf_detector_profile_file": flat["online_grf_detector_profile"],
        "include_online_grf_observation": bool(flat["online_grf_observation"]),
        "critic_privileged_observation": bool(flat["asymmetric_actor_critic"]),
        "prescribed_grf_disabled_sides": list(flat["disable_prescribed_grf_side"]),
        "online_grf_applied_sides": list(flat["online_grf_applied_side"]),
        "step_wall_timeout_s": float(flat["step_wall_timeout_s"]),
        "grf_penetration_penalty_threshold_m": float(
            flat["grf_penetration_penalty_threshold_m"]
        ),
        "grf_penetration_termination_m": float(
            flat["grf_penetration_termination_m"]
        ),
        "reward": dict(reward),
    }


def encode_absolute_action(
    values: np.ndarray,
    coord_names: Sequence[str],
    bounds: Mapping[str, Sequence[float]],
) -> np.ndarray:
    values = np.asarray(values, dtype=float)
    if values.ndim != 2 or values.shape[1] != len(coord_names):
        raise ValueError("teacher values must have shape (policy_knots, n_coords)")
    action = np.empty_like(values)
    for index, name in enumerate(coord_names):
        low, high = (float(value) for value in bounds[name])
        if not high > low:
            raise ValueError(f"invalid absolute bounds for {name}: {(low, high)}")
        action[:, index] = 2.0 * (values[:, index] - low) / (high - low) - 1.0
    return np.clip(action, -1.0, 1.0)


def prescribed_teacher_action(
    base,
    target_t: float,
    *,
    lookahead_s: float | Mapping[str, float] = 0.0,
) -> np.ndarray:
    knots = int(base.env_cfg.policy_knots)
    future_times = base.t + (
        np.arange(1, knots + 1, dtype=float) / float(knots)
    ) * max(float(target_t) - float(base.t), float(base.cfg.dt))
    values = np.empty((knots, len(base.cfg.pros_coords)), dtype=float)
    for row, time_value in enumerate(future_times):
        for column, name in enumerate(base.cfg.pros_coords):
            lead = (
                float(lookahead_s.get(name, 0.0))
                if isinstance(lookahead_s, Mapping)
                else float(lookahead_s)
            )
            teacher_time = min(
                float(time_value) + max(0.0, lead),
                float(base.cfg.t_end),
            )
            q_base, _, _ = base.base_kin.get(teacher_time)
            values[row, column] = float(q_base[name])
    bounds = base.env_cfg.absolute_bounds_rad
    if bounds is None:
        raise ValueError("absolute_bounds_rad is required for the prescribed teacher")
    return encode_absolute_action(values, base.cfg.pros_coords, bounds).reshape(-1)


def _term(info: Mapping[str, Any], name: str, default: float = 0.0) -> float:
    terms = info.get("reward_terms", {}) if isinstance(info, Mapping) else {}
    try:
        value = float(terms.get(name, default))
    except (TypeError, ValueError):
        return float(default)
    return value if math.isfinite(value) else float(default)


def collect_teacher_dataset(
    env_config: Mapping[str, Any],
    output_dir: Path,
    *,
    seed: int,
    teacher_lookahead_s: float | Mapping[str, float],
    action_noise_sigma: float | Sequence[float] = 0.0,
    action_noise_hold_steps: int = 1,
) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    env = env_factory.make_cmc_env(env_config)
    base = env.unwrapped
    observations: list[np.ndarray] = []
    actions: list[np.ndarray] = []
    executed_actions: list[np.ndarray] = []
    action_noises: list[np.ndarray] = []
    times: list[float] = []
    trace: list[dict[str, Any]] = []
    rewards: list[float] = []
    penetrations: list[float] = []
    reserve_norms: list[float] = []
    slew_fractions: list[float] = []
    terminated = truncated = False
    final_info: dict[str, Any] = {}
    try:
        obs, final_info = env.reset(seed=seed)
        actor_names = tuple(str(name) for name in base.actor_feature_names)
        action_dim = int(np.prod(env.action_space.shape))
        noise_sigma = exploration_noise.broadcast_sigma(
            action_noise_sigma, action_dim
        ).reshape(env.action_space.shape)
        noise_process = exploration_noise.HeldStandardNormal(
            np.random.default_rng(seed),
            env.action_space.shape,
            action_noise_hold_steps,
        )
        expected_steps = int(
            math.ceil(
                float(base.env_cfg.episode_duration)
                / float(base.env_cfg.segment_duration)
            )
        )
        for step in range(1, expected_steps + 3):
            actor_obs = np.asarray(obs, dtype=np.float32).reshape(-1)[: base.n_actor]
            target_t = min(
                float(base.t) + float(base.env_cfg.segment_duration),
                float(base._episode_end),
            )
            teacher_action = prescribed_teacher_action(
                base,
                target_t,
                lookahead_s=teacher_lookahead_s,
            ).astype(np.float32)
            action_noise = (
                noise_process.next() * noise_sigma
            ).astype(np.float32)
            executed_action = teacher_action + action_noise
            observations.append(actor_obs.copy())
            actions.append(teacher_action.copy())
            executed_actions.append(executed_action.copy())
            action_noises.append(action_noise.copy())
            times.append(float(base.t))
            obs, reward, terminated, truncated, final_info = env.step(
                executed_action
            )
            rewards.append(float(reward))
            penetration = _term(final_info, "grf_penetration_m")
            reserve_norm = _term(final_info, "reserve_norm_nm")
            slew_fraction = _term(final_info, "target_slew_limited_fraction")
            penetrations.append(penetration)
            reserve_norms.append(reserve_norm)
            slew_fractions.append(slew_fraction)
            trace.append(
                {
                    "step": step,
                    "time_before": times[-1],
                    "time_after": float(final_info.get("time", float("nan"))),
                    "teacher_action": teacher_action.astype(float).tolist(),
                    "action_noise": action_noise.astype(float).tolist(),
                    "executed_action": executed_action.astype(float).tolist(),
                    "reward": float(reward),
                    "penetration_m": penetration,
                    "reserve_norm_nm": reserve_norm,
                    "target_slew_limited_fraction": slew_fraction,
                    "valid_hs_count": _term(final_info, "phase_valid_hs_count"),
                    "valid_to_count": _term(final_info, "phase_valid_to_count"),
                    "valid_cycle_count": _term(
                        final_info, "phase_valid_cycle_count"
                    ),
                    "terminated": bool(terminated),
                    "truncated": bool(truncated),
                    "end_reason": final_info.get("end_reason"),
                }
            )
            if terminated or truncated:
                break
    finally:
        env.close()

    dataset = {
        "observations": np.asarray(observations, dtype=np.float32),
        "actions": np.asarray(actions, dtype=np.float32),
        "executed_actions": np.asarray(executed_actions, dtype=np.float32),
        "action_noises": np.asarray(action_noises, dtype=np.float32),
        "times": np.asarray(times, dtype=np.float64),
        "actor_feature_names": np.asarray(actor_names, dtype=str),
    }
    expected_steps = int(
        math.ceil(
            float(env_config["episode_duration"])
            / float(env_config["segment_duration"])
        )
    )
    max_penetration = max(penetrations, default=0.0)
    hard_penetration = float(env_config["grf_penetration_termination_m"])
    final_hs = _term(final_info, "phase_valid_hs_count")
    final_to = _term(final_info, "phase_valid_to_count")
    final_cycles = _term(final_info, "phase_valid_cycle_count")
    gate = {
        "full_episode": len(observations) >= expected_steps,
        "not_safety_terminated": not bool(terminated),
        "at_least_one_valid_cycle": final_cycles >= 1.0,
        "penetration_below_hard_guard": max_penetration < hard_penetration,
        "finite_dataset": bool(
            np.all(np.isfinite(dataset["observations"]))
            and np.all(np.isfinite(dataset["actions"]))
        ),
    }
    summary = {
        "steps": len(observations),
        "expected_steps": expected_steps,
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "end_reason": final_info.get("end_reason"),
        "episode_return": float(np.sum(rewards)),
        "reward_mean": float(np.mean(rewards)) if rewards else None,
        "max_penetration_m": float(max_penetration),
        "hard_penetration_m": hard_penetration,
        "valid_hs_count": final_hs,
        "valid_to_count": final_to,
        "valid_cycle_count": final_cycles,
        "max_reserve_norm_nm": max(reserve_norms, default=0.0),
        "mean_reserve_norm_nm": (
            float(np.mean(reserve_norms)) if reserve_norms else None
        ),
        "slew_limited_steps": sum(value > 0.0 for value in slew_fractions),
        "mean_slew_limited_fraction": (
            float(np.mean(slew_fractions)) if slew_fractions else None
        ),
        "action_abs_max": (
            float(np.max(np.abs(dataset["actions"])))
            if dataset["actions"].size
            else 0.0
        ),
        "executed_action_abs_max": (
            float(np.max(np.abs(dataset["executed_actions"])))
            if dataset["executed_actions"].size
            else 0.0
        ),
        "action_noise_sigma": noise_sigma.reshape(-1).astype(float).tolist(),
        "action_noise_hold_steps": int(action_noise_hold_steps),
        "action_noise_hold_duration_s": float(
            action_noise_hold_steps * float(env_config["segment_duration"])
        ),
        "action_noise_realized_rms": (
            np.sqrt(np.mean(np.square(dataset["action_noises"]), axis=0))
            .astype(float)
            .tolist()
            if dataset["action_noises"].size
            else []
        ),
        "teacher_lookahead_s": (
            {str(key): float(value) for key, value in teacher_lookahead_s.items()}
            if isinstance(teacher_lookahead_s, Mapping)
            else float(teacher_lookahead_s)
        ),
        "n_actor": len(actor_names),
        "actor_feature_names": list(actor_names),
        "gate": gate,
        "gate_pass": all(gate.values()),
    }
    np.savez_compressed(output_dir / "teacher_dataset.npz", **dataset)
    (output_dir / "teacher_trace.json").write_text(
        json.dumps(trace, indent=2), encoding="utf-8"
    )
    (output_dir / "teacher_summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )
    return dataset, summary


def _zero_disabled_clock_columns(module, feature_names: Sequence[str]) -> list[int]:
    import torch

    indices = [
        feature_names.index(name)
        for name in warm_start.DISABLED_GAIT_CLOCK_FEATURES
        if name in feature_names
    ]
    with torch.no_grad():
        for index in indices:
            module.pi_encoder[0].weight[:, index].zero_()
    return indices


def _prediction_metrics(prediction: np.ndarray, target: np.ndarray) -> dict[str, Any]:
    error = np.asarray(prediction, dtype=float) - np.asarray(target, dtype=float)
    return {
        "mse": float(np.mean(np.square(error))),
        "rmse": float(np.sqrt(np.mean(np.square(error)))),
        "max_abs_error": float(np.max(np.abs(error))),
        "out_of_bounds_fraction": float(np.mean(np.abs(prediction) > 1.0)),
        "per_action_rmse": np.sqrt(np.mean(np.square(error), axis=0)).tolist(),
    }


def aggregate_dagger_traces(
    teacher_dataset: Mapping[str, np.ndarray],
    traces: Sequence[Sequence[Mapping[str, Any]]],
    *,
    trace_repeat: int = 1,
    interpolation_steps: int = 0,
) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    """Label on-policy observations with the time-aligned prescribed teacher."""
    if trace_repeat < 1:
        raise ValueError("trace_repeat must be >= 1")
    if interpolation_steps < 0:
        raise ValueError("interpolation_steps must be >= 0")
    teacher_observations = np.asarray(
        teacher_dataset["observations"], dtype=np.float32
    )
    teacher_actions = np.asarray(teacher_dataset["actions"], dtype=np.float32)
    teacher_times = np.asarray(teacher_dataset["times"], dtype=np.float64)
    feature_names = np.asarray(teacher_dataset["actor_feature_names"], dtype=str)
    discrete_indices = np.asarray(
        [
            index
            for index, name in enumerate(feature_names.tolist())
            if name.endswith(
                ("_in_contact", "_heel_strike", "_toe_off", "_saturated")
            )
            or name.startswith(("phase_fsm_", "phase_expected_"))
        ],
        dtype=int,
    )
    if teacher_observations.ndim != 2 or teacher_actions.ndim != 2:
        raise ValueError("teacher dataset must contain 2D observations and actions")
    if len(teacher_observations) != len(teacher_actions):
        raise ValueError("teacher observation/action lengths differ")

    visited_observations: list[np.ndarray] = []
    visited_actions: list[np.ndarray] = []
    visited_times: list[float] = []
    trace_summaries: list[dict[str, Any]] = []
    for trace_index, rows in enumerate(traces, start=1):
        policy_actions: list[np.ndarray] = []
        labels: list[np.ndarray] = []
        for expected_step, row in enumerate(rows, start=1):
            step = int(row.get("step", expected_step))
            if step != expected_step:
                raise ValueError(
                    f"trace {trace_index} step sequence is not contiguous at {step}"
                )
            teacher_index = step - 1
            if teacher_index >= len(teacher_actions):
                raise ValueError(
                    f"trace {trace_index} step {step} exceeds teacher horizon"
                )
            observation = np.asarray(
                row["actor_observation_vector_before"], dtype=np.float32
            ).reshape(-1)
            if observation.shape != (teacher_observations.shape[1],):
                raise ValueError(
                    f"trace {trace_index} step {step} observation width "
                    f"{observation.shape} != {(teacher_observations.shape[1],)}"
                )
            if not np.all(np.isfinite(observation)):
                raise ValueError(
                    f"trace {trace_index} step {step} observation is non-finite"
                )
            label = teacher_actions[teacher_index].copy()
            local_observations = [observation]
            for interpolation_index in range(1, interpolation_steps + 1):
                alpha = interpolation_index / float(interpolation_steps + 1)
                interpolated = teacher_observations[teacher_index] + alpha * (
                    observation - teacher_observations[teacher_index]
                )
                if discrete_indices.size:
                    interpolated[discrete_indices] = observation[discrete_indices]
                local_observations.append(interpolated.astype(np.float32))
            for local_observation in local_observations:
                visited_observations.append(local_observation)
                visited_actions.append(label)
                visited_times.append(float(teacher_times[teacher_index]))
            if "raw_policy_action" in row:
                policy_action = np.asarray(
                    row["raw_policy_action"], dtype=np.float32
                ).reshape(-1)
                if policy_action.shape != label.shape:
                    raise ValueError(
                        f"trace {trace_index} step {step} action width mismatch"
                    )
                policy_actions.append(policy_action)
                labels.append(label)
        summary: dict[str, Any] = {
            "trace_index": trace_index,
            "steps": len(rows),
        }
        if policy_actions:
            policy_array = np.asarray(policy_actions, dtype=np.float32)
            label_array = np.asarray(labels, dtype=np.float32)
            summary["policy_vs_teacher"] = _prediction_metrics(
                policy_array, label_array
            )
        trace_summaries.append(summary)

    if not visited_observations:
        raise ValueError("at least one non-empty DAgger trace is required")
    visited_obs_array = np.asarray(visited_observations, dtype=np.float32)
    visited_action_array = np.asarray(visited_actions, dtype=np.float32)
    visited_time_array = np.asarray(visited_times, dtype=np.float64)
    aggregate = {
        "observations": np.concatenate(
            [teacher_observations, np.tile(visited_obs_array, (trace_repeat, 1))],
            axis=0,
        ),
        "actions": np.concatenate(
            [teacher_actions, np.tile(visited_action_array, (trace_repeat, 1))],
            axis=0,
        ),
        "times": np.concatenate(
            [teacher_times, np.tile(visited_time_array, trace_repeat)],
            axis=0,
        ),
        "actor_feature_names": feature_names.copy(),
    }
    summary = {
        "teacher_samples": len(teacher_observations),
        "visited_trace_samples": sum(len(rows) for rows in traces),
        "interpolation_steps": interpolation_steps,
        "interpolated_samples": (
            sum(len(rows) for rows in traces) * interpolation_steps
        ),
        "unique_dagger_samples": len(visited_obs_array),
        "trace_repeat": trace_repeat,
        "dagger_training_samples": len(visited_obs_array) * trace_repeat,
        "aggregate_samples": len(aggregate["observations"]),
        "traces": trace_summaries,
    }
    return aggregate, summary


def adapt_actor(
    checkpoint: Path,
    dataset: Mapping[str, np.ndarray],
    output_dir: Path,
    *,
    seed: int,
    epochs: int,
    batch_size: int,
    learning_rate: float,
    validation_fraction: float,
    patience: int,
    clip_weight: float,
    logstd_weight: float,
    anchor_weight: float,
    freeze_logstd_head: bool = False,
    trainable_first_layer_features: Sequence[str] | None = None,
    first_layer_feature_scales: Mapping[str, float] | None = None,
) -> dict[str, Any]:
    import torch
    import torch.nn.functional as functional
    from ray.rllib.core.rl_module.rl_module import RLModule

    torch.manual_seed(seed)
    np.random.seed(seed)
    module = RLModule.from_checkpoint(checkpoint)
    module.pi.train()
    observations = np.asarray(dataset["observations"], dtype=np.float32)
    targets = np.asarray(dataset["actions"], dtype=np.float32)
    feature_names = [str(name) for name in dataset["actor_feature_names"].tolist()]
    if observations.ndim != 2 or observations.shape[1] != int(module._n_actor):
        raise ValueError(
            f"dataset actor width {observations.shape} != module n_actor {module._n_actor}"
        )
    action_dim = targets.shape[1]
    if action_dim * 2 != int(module.pi[-1].out_features):
        raise ValueError("teacher action width does not match Gaussian actor output")

    output_layer = module.pi[-1]
    source_logstd_weight = output_layer.weight[action_dim:].detach().clone()
    source_logstd_bias = output_layer.bias[action_dim:].detach().clone()
    if freeze_logstd_head and bool(torch.any(source_logstd_weight != 0.0)):
        raise ValueError(
            "freeze_logstd_head requires a constant log-std source "
            "(zero log-std output weights)"
        )

    def restore_logstd_head() -> None:
        if not freeze_logstd_head:
            return
        with torch.no_grad():
            output_layer.weight[action_dim:].copy_(source_logstd_weight)
            output_layer.bias[action_dim:].copy_(source_logstd_bias)

    clock_indices = _zero_disabled_clock_columns(module, feature_names)
    raw_tensor_obs = torch.as_tensor(observations, dtype=torch.float32)
    input_scales = np.ones(len(feature_names), dtype=np.float32)
    feature_scale_report: dict[str, float] = {}
    if first_layer_feature_scales:
        unknown = [name for name in first_layer_feature_scales if name not in feature_names]
        if unknown:
            raise ValueError(f"first-layer feature scales reference unknown inputs: {unknown}")
        first_layer_weight = module.pi_encoder[0].weight.detach()
        for name, raw_scale in first_layer_feature_scales.items():
            scale = float(raw_scale)
            if not np.isfinite(scale) or scale <= 0.0:
                raise ValueError(f"invalid first-layer feature scale for {name}: {scale}")
            index = feature_names.index(name)
            if bool(torch.any(first_layer_weight[:, index] != 0.0)):
                raise ValueError(
                    "feature scaling requires a zero source first-layer column: "
                    f"{name}"
                )
            input_scales[index] = scale
            feature_scale_report[name] = scale
    tensor_obs = raw_tensor_obs / torch.as_tensor(input_scales)
    tensor_targets = torch.as_tensor(targets, dtype=torch.float32)
    with torch.no_grad():
        initial_logits = module.pi(tensor_obs).detach().clone()
    source_logstd = initial_logits[:, action_dim:].clone()
    initial_predictions = initial_logits[:, :action_dim].cpu().numpy()
    anchor = {
        name: parameter.detach().clone()
        for name, parameter in module.pi.named_parameters()
    }

    trainable_feature_names: list[str] | None = None
    first_layer_gradient_hook = None
    if trainable_first_layer_features is not None:
        trainable_feature_names = list(
            dict.fromkeys(str(name) for name in trainable_first_layer_features)
        )
        unknown = [name for name in trainable_feature_names if name not in feature_names]
        if unknown:
            raise ValueError(
                "trainable first-layer features are absent from the actor schema: "
                f"{unknown}"
            )
        if not trainable_feature_names:
            raise ValueError("trainable_first_layer_features must not be empty")
        for parameter in module.pi.parameters():
            parameter.requires_grad_(False)
        first_layer_weight = module.pi_encoder[0].weight
        first_layer_weight.requires_grad_(True)
        gradient_mask = torch.zeros_like(first_layer_weight)
        for name in trainable_feature_names:
            gradient_mask[:, feature_names.index(name)] = 1.0
        first_layer_gradient_hook = first_layer_weight.register_hook(
            lambda gradient: gradient * gradient_mask
        )

    rng = np.random.default_rng(seed)
    indices = rng.permutation(len(observations))
    validation_count = max(1, int(round(len(indices) * validation_fraction)))
    validation_indices = np.sort(indices[:validation_count])
    training_indices = np.asarray(indices[validation_count:], dtype=int)
    if not len(training_indices):
        raise ValueError("validation split left no training samples")

    trainable_parameters = [
        parameter for parameter in module.pi.parameters() if parameter.requires_grad
    ]
    if not trainable_parameters:
        raise ValueError("actor adaptation has no trainable parameters")
    optimizer = torch.optim.Adam(trainable_parameters, lr=learning_rate)
    best_state = copy.deepcopy(module.pi.state_dict())
    best_validation_mse = float("inf")
    best_epoch = 0
    stale_epochs = 0
    history: list[dict[str, float]] = []

    def validation_mse() -> float:
        module.pi.eval()
        with torch.no_grad():
            logits = module.pi(tensor_obs[validation_indices])
            value = functional.mse_loss(
                logits[:, :action_dim], tensor_targets[validation_indices]
            )
        module.pi.train()
        return float(value.item())

    for epoch in range(1, epochs + 1):
        shuffled = rng.permutation(training_indices)
        epoch_losses: list[float] = []
        for start in range(0, len(shuffled), batch_size):
            batch_indices = shuffled[start : start + batch_size]
            logits = module.pi(tensor_obs[batch_indices])
            means = logits[:, :action_dim]
            mean_loss = functional.mse_loss(means, tensor_targets[batch_indices])
            clip_loss = torch.relu(torch.abs(means) - 1.0).square().mean()
            logstd_loss = functional.mse_loss(
                logits[:, action_dim:], source_logstd[batch_indices]
            )
            anchor_terms = [
                (parameter - anchor[name]).square().mean()
                for name, parameter in module.pi.named_parameters()
                if parameter.requires_grad
            ]
            anchor_loss = torch.stack(anchor_terms).mean()
            loss = (
                mean_loss
                + clip_weight * clip_loss
                + logstd_weight * logstd_loss
                + anchor_weight * anchor_loss
            )
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            optimizer.step()
            restore_logstd_head()
            _zero_disabled_clock_columns(module, feature_names)
            epoch_losses.append(float(loss.item()))

        val_mse = validation_mse()
        history.append(
            {
                "epoch": float(epoch),
                "train_loss": float(np.mean(epoch_losses)),
                "validation_mse": val_mse,
            }
        )
        if val_mse < best_validation_mse - 1e-9:
            best_validation_mse = val_mse
            best_epoch = epoch
            best_state = copy.deepcopy(module.pi.state_dict())
            stale_epochs = 0
        else:
            stale_epochs += 1
        if epoch == 1 or epoch % 25 == 0:
            print(
                f"[adapt] epoch={epoch} train={history[-1]['train_loss']:.8f} "
                f"val_mse={val_mse:.8f}",
                flush=True,
            )
        if stale_epochs >= patience:
            break

    module.pi.load_state_dict(best_state)
    if first_layer_gradient_hook is not None:
        first_layer_gradient_hook.remove()
    restore_logstd_head()
    _zero_disabled_clock_columns(module, feature_names)
    if feature_scale_report:
        with torch.no_grad():
            first_layer_weight = module.pi_encoder[0].weight
            for name, scale in feature_scale_report.items():
                first_layer_weight[:, feature_names.index(name)].div_(scale)
    module.pi.eval()
    with torch.no_grad():
        final_logits = module.pi(raw_tensor_obs)
    final_predictions = final_logits[:, :action_dim].cpu().numpy()
    final_logstd_error = float(
        torch.max(torch.abs(final_logits[:, action_dim:] - source_logstd)).item()
    )
    logstd_head_parameter_change = float(
        max(
            torch.max(
                torch.abs(output_layer.weight[action_dim:] - source_logstd_weight)
            ).item(),
            torch.max(
                torch.abs(output_layer.bias[action_dim:] - source_logstd_bias)
            ).item(),
        )
    )
    if freeze_logstd_head and logstd_head_parameter_change != 0.0:
        raise RuntimeError("frozen log-std output parameters changed")

    first_layer = module.pi_encoder[0].weight.detach().cpu().numpy()
    clock_norms = {
        feature_names[index]: float(np.linalg.norm(first_layer[:, index]))
        for index in clock_indices
    }
    fsm_norms = {
        name: float(np.linalg.norm(first_layer[:, feature_names.index(name)]))
        for name in feature_names
        if name.startswith("phase_")
    }
    module_dir = output_dir / "rl_module_target_adapted"
    module.save_to_path(module_dir)
    saved_state = warm_start.load_module_state(module_dir)
    live_state = module.get_state()
    save_comparison = warm_start.compare_actor_states(live_state, saved_state)
    if not save_comparison["exact"]:
        raise RuntimeError("saved adapted actor differs from the trained actor")
    source_state = warm_start.load_module_state(checkpoint)
    non_actor_comparison = warm_start.compare_non_actor_states(
        source_state, saved_state
    )
    non_actor_present = bool(non_actor_comparison.get("keys"))
    if non_actor_present and not non_actor_comparison["exact"]:
        raise RuntimeError("adaptation modified critic or other non-actor tensors")

    history_path = output_dir / "adaptation_history.json"
    history_path.write_text(json.dumps(history, indent=2), encoding="utf-8")
    report = {
        "source_checkpoint": str(checkpoint),
        "output_module": str(module_dir),
        "samples": len(observations),
        "training_samples": len(training_indices),
        "validation_samples": len(validation_indices),
        "epochs_requested": epochs,
        "epochs_run": len(history),
        "best_epoch": best_epoch,
        "best_validation_mse": best_validation_mse,
        "hyperparameters": {
            "seed": seed,
            "batch_size": batch_size,
            "learning_rate": learning_rate,
            "validation_fraction": validation_fraction,
            "patience": patience,
            "clip_weight": clip_weight,
            "logstd_weight": logstd_weight,
            "anchor_weight": anchor_weight,
            "freeze_logstd_head": bool(freeze_logstd_head),
            "trainable_first_layer_features": trainable_feature_names,
            "first_layer_feature_scales": feature_scale_report,
        },
        "initial_prediction": _prediction_metrics(initial_predictions, targets),
        "adapted_prediction": _prediction_metrics(final_predictions, targets),
        "logstd_max_abs_change_on_dataset": final_logstd_error,
        "logstd_head_max_abs_parameter_change": logstd_head_parameter_change,
        "disabled_clock_column_norms": clock_norms,
        "fsm_first_layer_column_norms": fsm_norms,
        "source_actor_digest": warm_start.actor_state_digest(source_state),
        "adapted_actor_digest": warm_start.actor_state_digest(saved_state),
        "save_reload": save_comparison,
        "non_actor_unchanged": non_actor_comparison,
        "non_actor_verification": (
            "exact"
            if non_actor_present
            else "not_available_in_inference_only_rl_module"
        ),
        "actor_feature_names": feature_names,
        "critic_trained": False,
        "ppo_updates": 0,
    }
    (output_dir / "adaptation_report.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--output-dir")
    parser.add_argument("--epochs", type=int, default=400)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--validation-fraction", type=float, default=0.20)
    parser.add_argument("--patience", type=int, default=60)
    parser.add_argument("--clip-weight", type=float, default=1.0)
    parser.add_argument("--logstd-weight", type=float, default=0.1)
    parser.add_argument("--anchor-weight", type=float, default=1e-5)
    parser.add_argument("--seed", type=int, default=123)
    parser.add_argument(
        "--episode-start-offset-s",
        type=float,
        default=None,
        help="Optional diagnostic override used to collect warm-start data at another phase.",
    )
    parser.add_argument("--teacher-lookahead-s", type=float, default=0.0)
    parser.add_argument("--teacher-knee-lookahead-s", type=float, default=None)
    parser.add_argument("--teacher-ankle-lookahead-s", type=float, default=None)
    parser.add_argument(
        "--teacher-action-noise-sigma",
        type=float,
        nargs="+",
        default=[0.0],
        help="Diagnostic Gaussian action sigma: shared or knee/ankle values.",
    )
    parser.add_argument(
        "--teacher-action-noise-hold-steps",
        type=int,
        default=1,
        help="Hold each diagnostic noise draw for this many policy steps.",
    )
    parser.add_argument("--teacher-only", action="store_true")
    parser.add_argument(
        "--record-teacher-outputs",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config_path = _resolve_input(args.config)
    checkpoint = _resolve_input(args.checkpoint)
    output_dir = _resolve_output(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    flat, reward = _resolved_flat_config(config_path)
    if args.episode_start_offset_s is not None:
        flat["episode_start_offset_s"] = float(args.episode_start_offset_s)
    _validate_target_contract(flat)
    env_config = build_target_env_config(
        flat,
        reward,
        output_dir=output_dir,
        record_outputs=args.record_teacher_outputs,
    )
    teacher_lookahead: float | dict[str, float]
    if (
        args.teacher_knee_lookahead_s is not None
        or args.teacher_ankle_lookahead_s is not None
    ):
        teacher_lookahead = {
            "pros_knee_angle": float(
                args.teacher_lookahead_s
                if args.teacher_knee_lookahead_s is None
                else args.teacher_knee_lookahead_s
            ),
            "pros_ankle_angle": float(
                args.teacher_lookahead_s
                if args.teacher_ankle_lookahead_s is None
                else args.teacher_ankle_lookahead_s
            ),
        }
    else:
        teacher_lookahead = float(args.teacher_lookahead_s)
    dataset, teacher_summary = collect_teacher_dataset(
        env_config,
        output_dir,
        seed=args.seed,
        teacher_lookahead_s=teacher_lookahead,
        action_noise_sigma=args.teacher_action_noise_sigma,
        action_noise_hold_steps=args.teacher_action_noise_hold_steps,
    )
    print(json.dumps({"teacher": teacher_summary}, indent=2), flush=True)
    if not teacher_summary["gate_pass"]:
        (output_dir / "run_summary.json").write_text(
            json.dumps(
                {
                    "ok": False,
                    "stage": "teacher_gate",
                    "teacher": teacher_summary,
                },
                indent=2,
            ),
            encoding="utf-8",
        )
        return 2

    if args.teacher_only:
        training_config.dump_resolved(
            SimpleNamespace(**flat),
            reward,
            output_dir / training_config.RESOLVED_CONFIG_NAME,
        )
        (output_dir / "run_summary.json").write_text(
            json.dumps(
                {
                    "ok": True,
                    "stage": "teacher_gate",
                    "config": str(config_path),
                    "teacher": teacher_summary,
                },
                indent=2,
            ),
            encoding="utf-8",
        )
        return 0

    adaptation = adapt_actor(
        checkpoint,
        dataset,
        output_dir,
        seed=args.seed,
        epochs=args.epochs,
        batch_size=args.batch_size,
        learning_rate=args.learning_rate,
        validation_fraction=args.validation_fraction,
        patience=args.patience,
        clip_weight=args.clip_weight,
        logstd_weight=args.logstd_weight,
        anchor_weight=args.anchor_weight,
    )
    snapshot_args = SimpleNamespace(**flat)
    training_config.dump_resolved(
        snapshot_args,
        reward,
        output_dir / training_config.RESOLVED_CONFIG_NAME,
    )
    shutil.copy2(config_path, output_dir / "source_training_config.yaml")
    summary = {
        "ok": True,
        "stage": "complete",
        "config": str(config_path),
        "checkpoint": str(checkpoint),
        "output_dir": str(output_dir),
        "teacher": teacher_summary,
        "adaptation": adaptation,
    }
    (output_dir / "run_summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
