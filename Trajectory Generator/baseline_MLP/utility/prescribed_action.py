"""Absolute-radian reference playback for simulator diagnostics only."""

from __future__ import annotations

import numpy as np


def prescribed_action(env) -> np.ndarray:
    """Sample IK at the next segment's knots, including a shortened final step.

    This oracle requires diagnostic ``raw`` actions. It does not feed reference
    data to a learned policy or change the production absolute action mapping.
    """
    if env.env_cfg.action_mode.lower() != "raw":
        raise ValueError("prescribed playback requires raw absolute-radian actions")
    target_t = min(env.t + env.env_cfg.segment_duration, env._episode_end)
    count = env.env_cfg.policy_knots
    future = env.t + np.arange(1, count + 1, dtype=float) / count * max(
        float(target_t) - env.t, env.cfg.dt
    )
    values = []
    for t in future:
        q, _, _ = env.base_kin.get(float(t))
        values.append([q[name] for name in env.cfg.pros_coords])
    return np.asarray(values, dtype=float)
