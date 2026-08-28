"""Factory + RLlib registration for the CMC-like prosthetic trajectory env.

Adapts our single-agent ``CMCLikeProsthesisTrajectoryEnv`` (gymnasium.Env) to
the interface Ray RLlib expects: a callable that takes a single ``env_config``
dict and returns a gymnasium env. The simulator, the SEA C++ plugin and the
Static Optimization biological path are consumed unchanged through the env's
public gymnasium interface.

Run-as-script and ``python -m`` both work: we make ``baseline_MLP`` importable
and bootstrap the simulator roots before importing the env.
"""

from __future__ import annotations

import os
import math
import sys
from dataclasses import fields, replace
from pathlib import Path
from typing import Any, Mapping

# Make sibling modules importable no matter how we are launched.
sys.path.insert(0, str(Path(__file__).resolve().parent))

import _bootstrap  # noqa: E402

_bootstrap.ensure_sim_paths()

import win_runtime  # noqa: E402,F401  (apply Windows torch/OpenSim shim first)

import numpy as np  # noqa: E402
import gymnasium as gym  # noqa: E402
from gymnasium import spaces  # noqa: E402

from osim_trj_cmc_like import (  # noqa: E402
    CMCEnvConfig,
    CMCLikeProsthesisTrajectoryEnv,
)

import reward_function  # noqa: E402

ENV_NAME = "cmc_traj_env"
DEFAULT_NETWORK_GRF_MODE = "online_sensor"
DEFAULT_NETWORK_ONLINE_GRF_PROFILE = (
    "online_grf_profiles/"
    "AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json"
)

_CMC_ENV_FIELDS = {f.name for f in fields(CMCEnvConfig)}


def _episode_start_offset_for_runner(
    env_config: Mapping[str, Any] | None,
) -> float | None:
    """Select one configured start offset deterministically for this EnvRunner."""
    if env_config is None:
        return None
    choices = env_config.get("episode_start_offset_choices_s")
    if choices is None or choices == [] or choices == ():
        return None
    if isinstance(choices, (str, bytes)):
        choices = [choices]
    try:
        offsets = [float(value) for value in choices]
    except (TypeError, ValueError) as exc:
        raise ValueError(
            "episode_start_offset_choices_s must contain numeric offsets"
        ) from exc
    if not offsets or any(not math.isfinite(value) or value < 0.0 for value in offsets):
        raise ValueError(
            "episode_start_offset_choices_s must contain finite, non-negative offsets"
        )

    worker_index = int(
        getattr(env_config, "worker_index", env_config.get("worker_index", 0)) or 0
    )
    vector_index = int(
        getattr(env_config, "vector_index", env_config.get("vector_index", 0)) or 0
    )
    assignment_index = max(0, worker_index - 1) + max(0, vector_index)
    return offsets[assignment_index % len(offsets)]


def assign_episode_start_offset_for_runner(
    env: gym.Env,
    env_context: Mapping[str, Any] | None,
) -> float | None:
    """Apply the worker-specific offset after RLlib creates its vector env.

    RLlib 2.55 converts ``EnvContext`` to a plain dict while invoking a custom
    vector entry point, which drops ``worker_index`` before ``make_cmc_env``.
    Its environment-created callback still receives the original context, so
    this hook applies the assignment before the first reset.
    """
    assigned_start = _episode_start_offset_for_runner(env_context)
    if assigned_start is None:
        return None

    vector_envs = getattr(env, "envs", None)
    candidates = list(vector_envs) if vector_envs is not None else [env]
    applied = 0
    for candidate in candidates:
        base_env = getattr(candidate, "unwrapped", candidate)
        cfg = getattr(base_env, "env_cfg", None)
        if isinstance(cfg, CMCEnvConfig):
            base_env.env_cfg = replace(cfg, episode_start_offset_s=assigned_start)
            applied += 1
    if applied == 0:
        raise RuntimeError(
            "could not apply the EnvRunner-specific episode start offset"
        )
    return assigned_start


class FlattenClipAction(gym.ActionWrapper):
    """Expose a flat 1D Box action to RLlib; reshape + clip for the env.

    The env action is a 2D Box ``(policy_knots, 2)``, which RLlib's default
    action distribution does not support. This wrapper advertises the flattened
    ``(policy_knots * 2,)`` Box, then clips the (unbounded) Gaussian-policy
    action to the original bounds and reshapes it back to ``(policy_knots, 2)``
    before stepping the env (the env validates but does not clip; finding F1).
    """

    def __init__(self, env: gym.Env) -> None:
        super().__init__(env)
        self._orig_space: spaces.Box = env.action_space
        self._orig_shape = self._orig_space.shape
        self.action_space = spaces.Box(
            low=np.asarray(self._orig_space.low, dtype=np.float32).reshape(-1),
            high=np.asarray(self._orig_space.high, dtype=np.float32).reshape(-1),
            dtype=np.float32,
        )

    def action(self, action):
        arr = np.asarray(action, dtype=np.float32).reshape(self._orig_shape)
        return np.clip(arr, self._orig_space.low, self._orig_space.high)


def _resolve_setup_xml(value: Any) -> Any:
    """Resolve a (possibly relative) setup-xml path to an absolute path.

    Relative paths are resolved against the repository root so that parallel Ray
    workers (which may run with a different CWD) still find the setup file.
    """
    if not value:
        return value
    text = os.fspath(value)
    if os.name != "nt":
        text = text.replace("\\", "/")
    path = Path(text)
    if not path.is_absolute():
        path = (_bootstrap.REPO_ROOT / path).resolve()
    return str(path)


def build_env_config(env_config: Mapping[str, Any] | None) -> CMCEnvConfig:
    """Build a CMCEnvConfig from an RLlib-style dict (unknown keys ignored)."""
    raw = dict(env_config or {})
    assigned_start = _episode_start_offset_for_runner(env_config)
    if assigned_start is not None:
        raw["episode_start_offset_s"] = assigned_start
    raw.setdefault("grf_mode", DEFAULT_NETWORK_GRF_MODE)
    raw.setdefault(
        "online_grf_profile_file",
        DEFAULT_NETWORK_ONLINE_GRF_PROFILE,
    )
    raw.setdefault(
        "include_online_grf_observation",
        raw["grf_mode"] != "prescribed",
    )
    # RLlib injects bookkeeping keys (worker_index, vector_index, ...): drop them.
    kwargs = {k: v for k, v in raw.items() if k in _CMC_ENV_FIELDS}
    if "setup_xml_path" in kwargs:
        kwargs["setup_xml_path"] = _resolve_setup_xml(kwargs["setup_xml_path"])
    return CMCEnvConfig(**kwargs)


def make_cmc_env(env_config: Mapping[str, Any] | None = None) -> gym.Env:
    """RLlib env creator: dict -> wrapped CMCLikeProsthesisTrajectoryEnv.

    Wrapping order (inner -> outer):
      1. ``FlattenClipAction`` so RLlib sees a 1D Box action and the simulator
         receives reshaped, in-bounds actions;
      2. ``RewardShapingWrapper`` so the reward seen by the agent comes from
         ``reward_function.py`` (the single source of truth), not the env's
         internal reward. Reward overrides are read from ``env_config["reward"]``
         (a dict of ``RewardConfig`` fields); unknown keys are ignored.
    """
    raw = dict(env_config or {})
    reward_cfg = reward_function.reward_config_from_mapping(raw.get("reward"))
    raw["include_imitation_target_observation"] = (
        reward_cfg.reward_mode == "imitation"
    )
    raw["imitation_position_weights"] = {
        "pros_knee_angle": reward_cfg.imitation_knee_position_weight,
        "pros_ankle_angle": reward_cfg.imitation_ankle_position_weight,
    }
    raw["imitation_velocity_weights"] = {
        "pros_knee_angle": reward_cfg.imitation_knee_velocity_weight,
        "pros_ankle_angle": reward_cfg.imitation_ankle_velocity_weight,
    }
    raw["grf_ankle_moment_flip_tau_tol_nm"] = (
        reward_cfg.grf_ankle_moment_flip_tau_tol_nm
    )
    raw["grf_ankle_moment_flip_force_threshold_n"] = (
        reward_cfg.grf_ankle_moment_flip_force_threshold_n
    )
    raw["phase_min_stance_duration_s"] = reward_cfg.phase_min_stance_duration_s
    raw["phase_min_swing_duration_s"] = reward_cfg.phase_min_swing_duration_s
    raw["phase_landing_window_start_s"] = reward_cfg.phase_landing_window_start_s
    raw["phase_landing_window_end_s"] = reward_cfg.phase_landing_window_end_s
    raw["phase_stance_hard_timeout_s"] = reward_cfg.phase_stance_hard_timeout_s
    raw["phase_swing_hard_timeout_s"] = reward_cfg.phase_swing_hard_timeout_s
    raw["phase_landing_force_full_credit_bw"] = reward_cfg.contact_load_target_bw
    raw["phase_min_stance_contact_fraction"] = (
        reward_cfg.phase_min_stance_contact_fraction
    )
    raw["phase_min_stance_load_bw_s"] = reward_cfg.phase_min_stance_load_bw_s
    raw["phase_min_cycle_knee_excursion_rad"] = (
        reward_cfg.phase_min_cycle_knee_excursion_rad
    )
    raw["phase_hs_event_credit"] = reward_cfg.phase_hs_event_credit
    raw["phase_to_event_credit"] = reward_cfg.phase_to_event_credit
    raw["phase_cycle_complete_bonus"] = reward_cfg.phase_cycle_complete_bonus
    raw["phase_failure_extra_penalty"] = reward_cfg.phase_failure_extra_penalty
    if reward_cfg.reward_mode == "imitation":
        raw.setdefault("imitation_initialize_to_target", True)
    cfg = build_env_config(raw)  # "reward" is not a CMCEnvConfig field -> ignored
    env = CMCLikeProsthesisTrajectoryEnv(cfg)
    env = FlattenClipAction(env)
    env = reward_function.RewardShapingWrapper(env, reward_cfg)
    return env


def register_cmc_env() -> str:
    """Register the env creator with RLlib/Tune. Returns the env name."""
    from ray.tune.registry import register_env

    register_env(ENV_NAME, make_cmc_env)
    return ENV_NAME
