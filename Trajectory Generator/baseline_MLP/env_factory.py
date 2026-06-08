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
import sys
from dataclasses import fields
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
    "AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json"
)

_CMC_ENV_FIELDS = {f.name for f in fields(CMCEnvConfig)}


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
    path = Path(value)
    if not path.is_absolute():
        path = (_bootstrap.REPO_ROOT / path).resolve()
    return str(path)


def build_env_config(env_config: Mapping[str, Any] | None) -> CMCEnvConfig:
    """Build a CMCEnvConfig from an RLlib-style dict (unknown keys ignored)."""
    raw = dict(env_config or {})
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
