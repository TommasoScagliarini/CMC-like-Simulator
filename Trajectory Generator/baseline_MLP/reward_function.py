"""Single source of truth for the baseline-MLP reward.

The CMC-like env (``osim_trj_cmc_like.py``) computes the **physics-derived loss
terms** of each step (tracking, reference, biological, effort, smoothness, SEA
saturation, safety) and exposes them all in ``info["reward_terms"]``. The scalar
reward is just a *tunable combination* of those losses.

This module owns that combination. ``RewardShapingWrapper`` recomputes the scalar
reward from ``info["reward_terms"]`` and replaces the env's reward, so the whole
``baseline_MLP`` pipeline (training and rollout) takes its reward from here. The
env, the ``SimulationRunner``, the Static Optimization path and the SEA C++ plugin
are **not modified**: we only re-shape the reward downstream of the env.

Boundary:
  * **env** -> raw per-step *losses* (what physically happened). Unchanged.
  * **reward_function.py** -> the scalar *reward* (the shaping you tune here).

Because the reward is re-derived from the losses, the env's own
``reward_*_weight`` config fields **no longer affect the agent's reward**; they
only feed the env's now-discarded internal reward. Tune the reward here instead.

Defaults reproduce the env's original reward **exactly**, so behaviour is
unchanged until you pass overrides (via ``env_config["reward"]`` / ``--reward-json``).
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, fields
from pathlib import Path
from typing import Any, Mapping

import gymnasium as gym
import numpy as np

# Loss keys read from ``info["reward_terms"]`` (produced by the env).
TRACKING_LOSS = "tracking_loss"
REFERENCE_LOSS = "reference_loss"
BIO_LOSS = "bio_loss"
EFFORT_LOSS = "effort_loss"
SMOOTHNESS_LOSS = "smoothness_loss"
SATURATION_LOSS = "saturation_loss"
SAFETY_LOSS = "safety_loss"


@dataclass
class RewardConfig:
    """Weights and blend coefficients of the scalar reward.

    Defaults match the env's original reward
    (``osim_trj_cmc_like.CMCLikeProsthesisTrajectoryEnv._get_reward`` + the safety
    term applied in ``step``):

        tracking_score  = 1 / (1 + tracking_weight  * tracking_loss)
        reference_score = 1 / (1 + reference_weight * reference_loss)
        bio_score       = 1 / (1 + bio_weight       * bio_loss)
        penalty = effort_weight*effort_loss + smoothness_weight*smoothness_loss
                  + saturation_weight*saturation_loss
        base   = clip(blend_tracking*tracking_score + blend_reference*reference_score
                      + blend_bio*bio_score - penalty, clip_low, clip_high)
        reward = base - safety_weight * safety_loss

    The ``safety_loss`` is non-zero only on an unsafe task termination (fall or
    joint divergence) and is already provided by the env in ``reward_terms``.
    """

    # Loss -> score sharpness (higher = reward decays faster with the loss).
    tracking_weight: float = 8.0
    reference_weight: float = 6.0
    bio_weight: float = 2.0

    # Additive penalties (subtracted before clipping).
    effort_weight: float = 0.05
    smoothness_weight: float = 0.1
    saturation_weight: float = 0.1

    # Safety penalty (subtracted after clipping; lets the reward go negative).
    safety_weight: float = 2.0

    # Convex-ish blend of the three positive scores.
    blend_tracking: float = 0.25
    blend_reference: float = 0.55
    blend_bio: float = 0.20

    # Clip applied to the positive part (before the safety penalty).
    clip_low: float = 0.0
    clip_high: float = 1.0

    # Out-of-band penalty on the *commanded reference* (the policy output, i.e.
    # the trajectory the SEA must track), per prosthetic coordinate in
    # cfg.pros_coords order [knee, ankle]. Mean squared excursion outside
    # [oob_q_min, oob_q_max], weighted and subtracted AFTER the clip (like safety)
    # so it keeps a gradient even when the positive reward is already 0. These are
    # the physiological gait-band limits — tighter than the env's anti-divergence
    # truncation bounds. Set oob_weight = 0.0 to disable.
    # pros_knee_angle is flexion-NEGATIVE in this model (IK range ~[-1.06,-0.13]);
    # pros_ankle_angle ~[-0.13, 0.40]. Bands contain the IK range so pure tracking
    # is not penalised.
    oob_weight: float = 2.0
    oob_q_min: tuple[float, ...] = (-1.35, -0.5)
    oob_q_max: tuple[float, ...] = (0.0, 0.5)

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any] | None) -> "RewardConfig":
        """Build from a dict; unknown keys are ignored, missing keys defaulted.

        Scalar fields are coerced to float; sequence fields (the oob bounds) to a
        tuple of floats.
        """
        if not data:
            return cls()
        known = {f.name for f in fields(cls)}
        kwargs: dict[str, Any] = {}
        for key, value in data.items():
            if key not in known:
                continue
            if isinstance(value, (list, tuple)):
                kwargs[key] = tuple(float(x) for x in value)
            else:
                kwargs[key] = float(value)
        return cls(**kwargs)

    def to_dict(self) -> dict[str, Any]:
        out: dict[str, Any] = {}
        for key, value in asdict(self).items():
            if isinstance(value, (list, tuple)):
                out[key] = [float(x) for x in value]
            else:
                out[key] = float(value)
        return out


def _score(loss: float, weight: float) -> float:
    return 1.0 / (1.0 + weight * float(loss))


def out_of_band_loss(reference, cfg: RewardConfig) -> float:
    """Mean squared excursion of the commanded reference outside the gait band.

    ``reference``: array ``(n_points, n_pros)`` of commanded reference q [rad] in
    ``cfg.pros_coords`` order (knee, ankle) — i.e. ``info["policy_segment_values"]`` —
    or a single ``(n_pros,)`` row. Per coordinate, only the part of q outside
    ``[oob_q_min, oob_q_max]`` is penalised (zero while in band). Columns without a
    matching bound are ignored.
    """
    ref = np.asarray(reference, dtype=float)
    if ref.ndim == 1:
        ref = ref.reshape(1, -1)
    n = min(ref.shape[1], len(cfg.oob_q_min), len(cfg.oob_q_max))
    if n == 0:
        return 0.0
    ref = ref[:, :n]
    low = np.asarray(cfg.oob_q_min[:n], dtype=float)
    high = np.asarray(cfg.oob_q_max[:n], dtype=float)
    over = np.clip(ref - high, 0.0, None)
    under = np.clip(low - ref, 0.0, None)
    return float(np.mean(over ** 2 + under ** 2))


def compute_reward(
    reward_terms: Mapping[str, float], cfg: RewardConfig, *, reference=None
) -> tuple[float, dict[str, float]]:
    """Combine the env's per-step losses into the scalar reward.

    ``reference`` (optional): the commanded reference trajectory of the step
    (``info["policy_segment_values"]``), used for the out-of-band penalty. When
    ``None`` or ``oob_weight == 0`` the penalty is 0.

    Returns ``(reward, components)`` where ``components`` carries the intermediate
    scores/penalties for logging (TensorBoard, summaries).
    """
    tracking_score = _score(reward_terms.get(TRACKING_LOSS, 0.0), cfg.tracking_weight)
    reference_score = _score(reward_terms.get(REFERENCE_LOSS, 0.0), cfg.reference_weight)
    bio_score = _score(reward_terms.get(BIO_LOSS, 0.0), cfg.bio_weight)

    penalty = (
        cfg.effort_weight * float(reward_terms.get(EFFORT_LOSS, 0.0))
        + cfg.smoothness_weight * float(reward_terms.get(SMOOTHNESS_LOSS, 0.0))
        + cfg.saturation_weight * float(reward_terms.get(SATURATION_LOSS, 0.0))
    )

    base = (
        cfg.blend_tracking * tracking_score
        + cfg.blend_reference * reference_score
        + cfg.blend_bio * bio_score
        - penalty
    )
    base = min(cfg.clip_high, max(cfg.clip_low, base))

    safety_term = cfg.safety_weight * float(reward_terms.get(SAFETY_LOSS, 0.0))

    if reference is not None and cfg.oob_weight:
        oob_loss = out_of_band_loss(reference, cfg)
    else:
        oob_loss = 0.0
    oob_term = cfg.oob_weight * oob_loss

    # Safety and out-of-band are subtracted AFTER the clip: they stay active (with
    # gradient) even once the positive reward has saturated to clip_low.
    reward = base - safety_term - oob_term

    components = {
        "reward": float(reward),
        "reward_base": float(base),
        "tracking_score": float(tracking_score),
        "reference_score": float(reference_score),
        "bio_score": float(bio_score),
        "penalty": float(penalty),
        "safety_term": float(safety_term),
        "oob_loss": float(oob_loss),
        "oob_term": float(oob_term),
    }
    return float(reward), components


def reward_config_from_mapping(data: Mapping[str, Any] | None) -> RewardConfig:
    """Public helper mirroring ``RewardConfig.from_mapping`` (import convenience)."""
    return RewardConfig.from_mapping(data)


def load_reward_overrides(spec: str | None) -> dict | None:
    """Parse a reward-override spec: a JSON file path or an inline JSON object.

    Shared by the training and rollout entrypoints (``--reward-json``). Returns a
    plain dict of ``RewardConfig`` field overrides, or ``None`` if ``spec`` is empty.
    """
    if not spec:
        return None
    path = Path(spec)
    text = path.read_text(encoding="utf-8") if path.exists() else spec
    data = json.loads(text)
    if not isinstance(data, dict):
        raise ValueError(
            "reward override must be a JSON object of RewardConfig fields"
        )
    return data


class RewardShapingWrapper(gym.Wrapper):
    """Replace the env reward with ``compute_reward`` over ``info['reward_terms']``.

    The action is forwarded unchanged (this is a pure reward transform). The
    out-of-band penalty uses the commanded reference of the step
    (``info['policy_segment_values']``). The recomputed components are added to
    ``info['reward_components']`` and mirrored into ``info['log']`` (prefixed
    ``RewardShaped/``) for logging; the env's original scalar is preserved as
    ``info['reward_env_original']``.

    If a step yields no ``reward_terms`` (e.g. an early native failure path), the
    env's own reward is returned untouched.
    """

    def __init__(self, env: gym.Env, reward_config: RewardConfig | None = None) -> None:
        super().__init__(env)
        self.reward_config = reward_config or RewardConfig()

    def step(self, action):
        obs, env_reward, terminated, truncated, info = self.env.step(action)
        terms = info.get("reward_terms") if isinstance(info, dict) else None
        if not terms:
            return obs, env_reward, terminated, truncated, info

        reference = info.get("policy_segment_values")
        reward, components = compute_reward(
            terms, self.reward_config, reference=reference
        )

        info = dict(info)
        info["reward_components"] = components
        info["reward_env_original"] = float(env_reward)
        log = dict(info.get("log") or {})
        for key, value in components.items():
            log[f"RewardShaped/{key}"] = float(value)
        info["log"] = log
        return obs, float(reward), terminated, truncated, info
