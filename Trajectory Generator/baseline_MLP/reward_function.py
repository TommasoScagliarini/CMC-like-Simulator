"""Single source of truth for the baseline-MLP reward.

The CMC-like env (``osim_trj_cmc_like.py``) computes the dimensionless
**physics-derived loss terms** of each step (tracking, reference, biological,
effort, smoothness, SEA saturation, safety) and exposes them all in
``info["reward_terms"]``. The scalar reward is just a *tunable combination* of
those losses.

This module owns that combination. ``RewardShapingWrapper`` recomputes the scalar
reward from ``info["reward_terms"]`` and replaces the env's reward, so the whole
``baseline_MLP`` pipeline (training and rollout) takes its reward from here. The
env, the ``SimulationRunner``, the Static Optimization path and the SEA C++ plugin
are **not modified**: we only re-shape the reward downstream of the env.

Boundary:
  * **env** -> dimensionless per-step *losses* plus raw diagnostics with units.
  * **reward_function.py** -> the scalar *reward* (the shaping you tune here).

Because the reward is re-derived from the losses, the env's own
``reward_*_weight`` config fields **no longer affect the agent's reward**; they
only feed the env's now-discarded internal reward. Tune the reward here instead.

Defaults preserve the legacy baseline shaping. New physical terms remain
opt-in until enabled by a training YAML or reward override, so old checkpoint
evaluation does not silently change when the env gains a new diagnostic loss.
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
COMMAND_RATE_LOSS = "command_rate_loss"
SEGMENT_DELTA_LOSS = "segment_delta_loss"
QDOT_REF_LOSS = "qdot_ref_loss"
QDDOT_REF_LOSS = "qddot_ref_loss"
JERK_REF_LOSS = "jerk_ref_loss"
REFERENCE_GOVERNOR_LOSS = "reference_governor_loss"
U_RATE_LOSS = "u_rate_loss"
SATURATION_LOSS = "saturation_loss"
SEA_SATURATION_LOSS = "sea_saturation_loss"
SEA_TORQUE_ERROR_LOSS = "sea_torque_error_loss"
SEA_MOTOR_SPEED_LOSS = "sea_motor_speed_loss"
SEA_MOTOR_ACCEL_LOSS = "sea_motor_accel_loss"
SEA_MOTOR_POWER_LOSS = "sea_motor_power_loss"
SAFETY_LOSS = "safety_loss"
GRF_PENETRATION_LOSS = "grf_penetration_loss"
SOUND_IMITATION_LOSS = "sound_imitation_loss"
SERVED_IMITATION_LOSS = "served_imitation_loss"


@dataclass
class RewardConfig:
    """Weights and blend coefficients of the scalar reward.

    Defaults preserve the original baseline reward structure:

        tracking_score  = 1 / (1 + tracking_weight  * tracking_loss)
        reference_score = 1 / (1 + reference_weight * reference_loss)
        bio_score       = 1 / (1 + bio_weight       * bio_loss)
        penalty = effort_weight*effort_loss + smoothness_weight*smoothness_loss
                  + saturation_weight*saturation_loss + optional physical terms
        base   = clip(blend_tracking*tracking_score + blend_reference*reference_score
                      + blend_bio*bio_score - penalty, clip_low, clip_high)
        reward = base - safety_weight * safety_loss

    ``safety_loss`` and ``grf_penetration_loss`` are provided by the env in
    ``reward_terms`` and applied after clipping when their central weights are
    enabled.
    """

    # Loss -> score sharpness (higher = reward decays faster with the loss).
    tracking_weight: float = 8.0
    reference_weight: float = 6.0
    bio_weight: float = 2.0

    # Additive penalties (subtracted before clipping).
    effort_weight: float = 0.05
    smoothness_weight: float = 0.1
    saturation_weight: float = 0.1
    command_rate_weight: float = 0.0
    # Optional component-level command/reference penalties. These are additive
    # to command_rate_weight, which preserves the legacy aggregate loss.
    segment_delta_weight: float = 0.0
    qdot_ref_weight: float = 0.0
    qddot_ref_weight: float = 0.0
    jerk_ref_weight: float = 0.0
    reference_governor_weight: float = 0.0
    u_rate_weight: float = 0.0
    sea_saturation_weight: float = 0.0
    sea_torque_error_weight: float = 0.0
    sea_motor_speed_weight: float = 0.0
    sea_motor_accel_weight: float = 0.0
    sea_motor_power_weight: float = 0.0

    # Safety/contact penalties subtracted after clipping. Penetration defaults
    # to zero for checkpoint compatibility; production configs enable it
    # explicitly when the online prosthetic contact is applied.
    safety_weight: float = 2.0
    grf_penetration_weight: float = 0.0

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

    # Reward objective selector. "ex_novo" (default) = the blend above, UNCHANGED.
    # "imitation" = pre-training reward where the prosthetic joints mirror the
    # sound (contralateral) leg anti-phase (uses ``sound_imitation_loss`` from the
    # env). See the 2026-06-10 imitation-reward report.
    reward_mode: str = "ex_novo"
    imitation_weight: float = 8.0          # loss -> score sharpness (imitation)
    served_imitation_weight: float = 8.0   # target -> served-reference quality
    imitation_knee_position_weight: float = 1.0
    imitation_ankle_position_weight: float = 1.0
    imitation_knee_velocity_weight: float = 0.02
    imitation_ankle_velocity_weight: float = 0.02
    blend_served_imitation: float = 0.0    # legacy imitation remains unchanged
    blend_imitation: float = 0.8           # sound-leg imitation score weight
    blend_imitation_tracking: float = 0.2  # SEA execution-quality weight

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any] | None) -> "RewardConfig":
        """Build from a dict; unknown keys are ignored, missing keys defaulted.

        String fields (e.g. ``reward_mode``) are kept as strings; sequence fields
        (the oob bounds) become tuples of floats; everything else is coerced to
        float.
        """
        if not data:
            return cls()
        known = {f.name for f in fields(cls)}
        kwargs: dict[str, Any] = {}
        for key, value in data.items():
            if key not in known:
                continue
            if isinstance(value, str):
                kwargs[key] = value
            elif isinstance(value, (list, tuple)):
                kwargs[key] = tuple(float(x) for x in value)
            else:
                kwargs[key] = float(value)
        return cls(**kwargs)

    def to_dict(self) -> dict[str, Any]:
        out: dict[str, Any] = {}
        for key, value in asdict(self).items():
            if isinstance(value, str):
                out[key] = value
            elif isinstance(value, (list, tuple)):
                out[key] = [float(x) for x in value]
            else:
                out[key] = float(value)
        return out


def _score(loss: float, weight: float) -> float:
    return 1.0 / (1.0 + weight * float(loss))


def _out_of_band_losses(reference, cfg: RewardConfig) -> tuple[float, float]:
    """Dimensionless and raw excursion of the commanded reference outside the gait band.

    ``reference``: array ``(n_points, n_pros)`` of commanded reference q [rad] in
    ``cfg.pros_coords`` order (knee, ankle) — i.e. ``info["policy_segment_values"]`` —
    or a single ``(n_pros,)`` row. Per coordinate, only the part of q outside
    ``[oob_q_min, oob_q_max]`` is penalised (zero while in band). The first
    returned loss is normalized by the band width; the second is raw q^2 for
    diagnostics only.
    """
    ref = np.asarray(reference, dtype=float)
    if ref.ndim == 1:
        ref = ref.reshape(1, -1)
    n = min(ref.shape[1], len(cfg.oob_q_min), len(cfg.oob_q_max))
    if n == 0:
        return 0.0, 0.0
    ref = ref[:, :n]
    low = np.asarray(cfg.oob_q_min[:n], dtype=float)
    high = np.asarray(cfg.oob_q_max[:n], dtype=float)
    over = np.clip(ref - high, 0.0, None)
    under = np.clip(low - ref, 0.0, None)
    excursion = over + under
    band = np.maximum(1e-9, high - low)
    normalized = excursion / band
    return (
        float(np.mean(np.clip(np.square(normalized), 0.0, 25.0))),
        float(np.mean(np.square(excursion))),
    )


def out_of_band_loss(reference, cfg: RewardConfig) -> float:
    """Dimensionless mean squared excursion outside the gait band."""
    return _out_of_band_losses(reference, cfg)[0]


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
    imitation_score = _score(
        reward_terms.get(SOUND_IMITATION_LOSS, 0.0), cfg.imitation_weight
    )
    served_imitation_score = _score(
        reward_terms.get(SERVED_IMITATION_LOSS, 0.0),
        cfg.served_imitation_weight,
    )

    # Shared penalty/safety/out-of-band terms (identical in both reward modes).
    penalty = (
        cfg.effort_weight * float(reward_terms.get(EFFORT_LOSS, 0.0))
        + cfg.smoothness_weight * float(reward_terms.get(SMOOTHNESS_LOSS, 0.0))
        + cfg.saturation_weight * float(reward_terms.get(SATURATION_LOSS, 0.0))
        + cfg.command_rate_weight * float(reward_terms.get(COMMAND_RATE_LOSS, 0.0))
        + cfg.segment_delta_weight
        * float(reward_terms.get(SEGMENT_DELTA_LOSS, 0.0))
        + cfg.qdot_ref_weight * float(reward_terms.get(QDOT_REF_LOSS, 0.0))
        + cfg.qddot_ref_weight * float(reward_terms.get(QDDOT_REF_LOSS, 0.0))
        + cfg.jerk_ref_weight * float(reward_terms.get(JERK_REF_LOSS, 0.0))
        + cfg.reference_governor_weight
        * float(reward_terms.get(REFERENCE_GOVERNOR_LOSS, 0.0))
        + cfg.u_rate_weight * float(reward_terms.get(U_RATE_LOSS, 0.0))
        + cfg.sea_saturation_weight
        * float(reward_terms.get(SEA_SATURATION_LOSS, 0.0))
        + cfg.sea_torque_error_weight
        * float(reward_terms.get(SEA_TORQUE_ERROR_LOSS, 0.0))
        + cfg.sea_motor_speed_weight
        * float(reward_terms.get(SEA_MOTOR_SPEED_LOSS, 0.0))
        + cfg.sea_motor_accel_weight
        * float(reward_terms.get(SEA_MOTOR_ACCEL_LOSS, 0.0))
        + cfg.sea_motor_power_weight
        * float(reward_terms.get(SEA_MOTOR_POWER_LOSS, 0.0))
    )
    safety_term = cfg.safety_weight * float(reward_terms.get(SAFETY_LOSS, 0.0))
    grf_penetration_term = cfg.grf_penetration_weight * float(
        reward_terms.get(GRF_PENETRATION_LOSS, 0.0)
    )
    if reference is not None and cfg.oob_weight:
        oob_loss, oob_raw_loss = _out_of_band_losses(reference, cfg)
    else:
        oob_loss = 0.0
        oob_raw_loss = 0.0
    oob_term = cfg.oob_weight * oob_loss

    # Positive base differs by mode. ex_novo (default) is UNCHANGED; imitation
    # rewards mirroring the sound leg (anti-phase) instead of the prosthetic IK
    # reference (no reference/bio terms: reference is the abandoned IK target, bio
    # is uncontrollable by the policy).
    if cfg.reward_mode == "imitation":
        base = (
            cfg.blend_served_imitation * served_imitation_score
            + cfg.blend_imitation * imitation_score
            + cfg.blend_imitation_tracking * tracking_score
            - penalty
        )
    else:
        base = (
            cfg.blend_tracking * tracking_score
            + cfg.blend_reference * reference_score
            + cfg.blend_bio * bio_score
            - penalty
        )
    base = min(cfg.clip_high, max(cfg.clip_low, base))

    # Safety, contact feasibility and out-of-band are subtracted AFTER the clip:
    # they stay active even once the positive reward has saturated to clip_low.
    reward = base - safety_term - grf_penetration_term - oob_term

    components = {
        "reward": float(reward),
        "reward_base": float(base),
        "tracking_score": float(tracking_score),
        "reference_score": float(reference_score),
        "bio_score": float(bio_score),
        "imitation_score": float(imitation_score),
        "served_imitation_score": float(served_imitation_score),
        "tracking_loss": float(reward_terms.get(TRACKING_LOSS, 0.0)),
        "tracking_position_loss": float(
            reward_terms.get("tracking_position_loss", 0.0)
        ),
        "tracking_velocity_loss": float(
            reward_terms.get("tracking_velocity_loss", 0.0)
        ),
        "reference_loss": float(reward_terms.get(REFERENCE_LOSS, 0.0)),
        "reference_position_loss": float(
            reward_terms.get("reference_position_loss", 0.0)
        ),
        "bio_loss": float(reward_terms.get(BIO_LOSS, 0.0)),
        "bio_position_loss": float(reward_terms.get("bio_position_loss", 0.0)),
        "sound_imitation_loss": float(reward_terms.get(SOUND_IMITATION_LOSS, 0.0)),
        "served_imitation_loss": float(
            reward_terms.get(SERVED_IMITATION_LOSS, 0.0)
        ),
        "command_rate_loss": float(reward_terms.get(COMMAND_RATE_LOSS, 0.0)),
        "segment_delta_loss": float(reward_terms.get(SEGMENT_DELTA_LOSS, 0.0)),
        "segment_knot_delta_loss": float(
            reward_terms.get("segment_knot_delta_loss", 0.0)
        ),
        "qdot_ref_loss": float(reward_terms.get(QDOT_REF_LOSS, 0.0)),
        "qddot_ref_loss": float(reward_terms.get(QDDOT_REF_LOSS, 0.0)),
        "jerk_ref_loss": float(reward_terms.get(JERK_REF_LOSS, 0.0)),
        "reference_governor_loss": float(
            reward_terms.get(REFERENCE_GOVERNOR_LOSS, 0.0)
        ),
        "u_rate_loss": float(reward_terms.get(U_RATE_LOSS, 0.0)),
        "sea_saturation_loss": float(reward_terms.get(SEA_SATURATION_LOSS, 0.0)),
        "sea_torque_error_loss": float(
            reward_terms.get(SEA_TORQUE_ERROR_LOSS, 0.0)
        ),
        "sea_motor_speed_loss": float(reward_terms.get(SEA_MOTOR_SPEED_LOSS, 0.0)),
        "sea_motor_accel_loss": float(reward_terms.get(SEA_MOTOR_ACCEL_LOSS, 0.0)),
        "sea_motor_power_loss": float(reward_terms.get(SEA_MOTOR_POWER_LOSS, 0.0)),
        "penalty": float(penalty),
        "safety_term": float(safety_term),
        "grf_penetration_loss": float(
            reward_terms.get(GRF_PENETRATION_LOSS, 0.0)
        ),
        "grf_penetration_term": float(grf_penetration_term),
        "oob_loss": float(oob_loss),
        "oob_raw_loss": float(oob_raw_loss),
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
