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
import os
from collections.abc import Mapping
from dataclasses import asdict, dataclass, fields
from pathlib import Path
from typing import Any

import gymnasium as gym
import numpy as np

from experimental_morphology_corridor import (
    CAUSAL_DELAYED_PHASE_MODE,
    EXPERIMENTAL_PHASE_MODE,
    V26_EVENT_CONTRACT_ID,
    CausalDelayedMorphologyBuffer,
    CompletedSegmentMorphologyLedger,
    extract_v26_morphology_runtime,
)

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
SEA_TAU_SPRING_EFFORT_LOSS = "sea_tau_spring_effort_loss"
SEA_TAU_SPRING_RATE_LOSS = "sea_tau_spring_rate_loss"
SEA_MOTOR_SPEED_LOSS = "sea_motor_speed_loss"
SEA_MOTOR_ACCEL_LOSS = "sea_motor_accel_loss"
SEA_MOTOR_POWER_LOSS = "sea_motor_power_loss"
POLICY_ACTION_CLIP_LOSS = "policy_action_clip_loss"
SAFETY_LOSS = "safety_loss"
GRF_PENETRATION_LOSS = "grf_penetration_loss"
GRF_ANKLE_MOMENT_FLIP_LOSS = "grf_ankle_moment_flip_loss"
SOUND_IMITATION_LOSS = "sound_imitation_loss"
SERVED_IMITATION_LOSS = "served_imitation_loss"

_REWARD_MODULE_DIR = Path(__file__).resolve().parent
_MORPHOLOGY_COORDS = ("pros_knee_angle", "pros_ankle_angle")


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
    sea_tau_spring_effort_weight: float = 0.0
    sea_tau_spring_rate_weight: float = 0.0
    sea_motor_speed_weight: float = 0.0
    sea_motor_accel_weight: float = 0.0
    sea_motor_power_weight: float = 0.0
    policy_action_clip_weight: float = 0.0

    # Safety/contact penalties subtracted after clipping. Penetration defaults
    # to zero for checkpoint compatibility; production configs enable it
    # explicitly when the online prosthetic contact is applied.
    safety_weight: float = 2.0
    grf_penetration_weight: float = 0.0
    grf_ankle_moment_flip_weight: float = 0.0
    grf_ankle_moment_flip_tau_tol_nm: float = 8.0
    grf_ankle_moment_flip_force_threshold_n: float = 50.0

    # Prescribed-free ex-novo shaping. Defaults are off so old imitation runs and
    # checkpoint evaluation keep their reward contract unless a YAML enables
    # these terms explicitly.
    blend_contact_load: float = 0.0
    blend_contact_support_to: float = 0.0
    blend_phase_regular: float = 0.0
    blend_phase_event_progress: float = 0.0
    blend_landing_window_contact: float = 0.0
    contact_load_target_bw: float = 0.65
    contact_load_max_bw: float = 1.35
    # Optional ex-novo contact-confidence/ledger mode. Zero defaults preserve
    # the legacy per-step load-magnitude reward for old checkpoint configs.
    contact_load_confidence_full_bw: float = 0.0
    contact_load_dense_evidence_limit_bw_s: float = 0.0
    contact_load_penetration_full_reward_m: float = 0.010
    contact_load_penetration_zero_reward_m: float = 0.012
    contact_support_to_window_start_s: float = 0.0
    contact_support_to_window_end_s: float = 0.0
    contact_support_failure_clawback_weight: float = 0.0
    prosthetic_stance_phase_end: float = 0.62
    swing_unloading_weight: float = 0.0
    swing_unloading_force_tol_bw: float = 0.08
    contact_overload_weight: float = 0.0
    grf_slip_weight: float = 0.0
    grf_slip_speed_scale_m_s: float = 0.5
    phase_regularity_weight: float = 4.0
    phase_event_order_weight: float = 1.0
    phase_period_weight: float = 1.0
    phase_period_nominal_s: float = 1.58
    phase_period_soft_margin_s: float = 0.25
    phase_period_hard_min_s: float = 0.90
    phase_period_hard_max_s: float = 2.20
    phase_periodicity_weight: float = 0.5
    phase_periodicity_scale_s: float = 0.20
    phase_period_min_s: float = 0.60
    phase_period_max_s: float = 1.60
    phase_stance_fraction_weight: float = 0.5
    phase_stance_fraction_min: float = 0.35
    phase_stance_fraction_max: float = 0.80
    phase_timeout_weight: float = 0.5
    phase_stance_timeout_s: float = 1.45
    phase_swing_timeout_s: float = 0.90
    phase_timeout_scale_s: float = 0.20
    phase_timeout_penalty_weight: float = 0.0
    phase_stance_hard_timeout_s: float = 2.20
    phase_swing_hard_timeout_s: float = 1.30
    phase_min_stance_duration_s: float = 0.05
    phase_min_swing_duration_s: float = 0.20
    phase_landing_window_start_s: float = 0.55
    phase_landing_window_end_s: float = 1.10
    phase_invalid_event_weight: float = 0.0
    phase_contact_validity_weight: float = 0.0
    phase_min_stance_contact_fraction: float = 0.0
    phase_min_stance_load_bw_s: float = 0.0
    phase_min_cycle_knee_excursion_rad: float = 0.0
    phase_hs_event_credit: float = 0.10
    phase_to_event_credit: float = 0.20
    phase_cycle_complete_bonus: float = 0.70
    phase_failure_extra_penalty: float = 0.05
    phase_clawback_penalty_weight: float = 0.0
    reserve_residual_weight: float = 0.0
    reserve_norm_scale_nm: float = 500.0
    residual_norm_scale_nm: float = 100.0
    pelvis_height_weight: float = 0.0
    pelvis_height_min_m: float = 0.75
    pelvis_height_scale_m: float = 0.10

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
    prosthetic_joint_range_weight: float = 0.0
    prosthetic_joint_q_min: tuple[float, ...] = (-1.40, -0.60)
    prosthetic_joint_q_max: tuple[float, ...] = (0.0, 0.60)
    morphology_profile: str = ""
    # ``legacy_cycle_fraction`` reproduces historical checkpoints exactly.
    # ``event_anchored`` fixes HS at phase 0 and TO at the canonical profile
    # phase while using only past valid cycle durations at runtime.
    morphology_phase_mode: str = "legacy_cycle_fraction"
    morphology_weight: float = 0.0
    morphology_std_multiplier_knee: float = 1.0
    morphology_std_multiplier_ankle: float = 1.0
    morphology_margin_knee_deg: float = 0.0
    morphology_margin_ankle_deg: float = 0.0
    # Experimental completed-segment ledger.  All effectful controls default
    # off: selecting the experimental phase mode with these defaults is a
    # diagnostic shadow evaluation only.
    morphology_completed_segment_max_samples: float = 4096.0
    morphology_hard_q_min: tuple[float, ...] = (-1.40, -0.60)
    morphology_hard_q_max: tuple[float, ...] = (0.0, 0.60)
    morphology_hard_termination_enabled: float = 0.0
    morphology_experimental_allow_effects: float = 0.0
    # Fixed-delay causal V26 corridor.  It stays explicitly opt-in and its
    # defaults do not alter historical/active training configurations.
    morphology_reward_delay_s: float = 0.04
    morphology_max_delivery_latency_s: float = 0.01
    morphology_causal_max_samples: float = 4096.0
    morphology_causal_event_contract_id: str = V26_EVENT_CONTRACT_ID
    morphology_causal_allow_effects: float = 0.0

    # Reward objective selector. "ex_novo" (default) = the blend above, UNCHANGED.
    # "imitation" = pre-training reward where the prosthetic joints mirror the
    # sound (contralateral) leg anti-phase (uses ``sound_imitation_loss`` from the
    # env). See the 2026-06-10 imitation-reward report.
    reward_mode: str = "ex_novo"
    imitation_weight: float = 8.0  # loss -> score sharpness (imitation)
    served_imitation_weight: float = 8.0  # target -> served-reference quality
    imitation_knee_position_weight: float = 1.0
    imitation_ankle_position_weight: float = 1.0
    imitation_knee_velocity_weight: float = 0.02
    imitation_ankle_velocity_weight: float = 0.02
    blend_served_imitation: float = 0.0  # legacy imitation remains unchanged
    blend_imitation: float = 0.8  # sound-leg imitation score weight
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


def _bounded_square_ratio(value: float, scale: float, cap: float = 25.0) -> float:
    scale = max(1e-9, abs(float(scale)))
    ratio = float(value) / scale
    return float(min(cap, ratio * ratio))


def _huber_loss(value: float, delta: float = 1.0, cap: float = 25.0) -> float:
    """Robust dimensionless loss: quadratic near zero, linear for large errors."""
    delta = max(1e-9, abs(float(delta)))
    x = abs(float(value))
    if x <= delta:
        loss = 0.5 * x * x
    else:
        loss = delta * (x - 0.5 * delta)
    return float(min(cap, loss))


def _huber_interval_loss(
    value: float,
    low: float,
    high: float,
    scale: float,
    *,
    cap: float = 25.0,
) -> float:
    """Huber loss for excursions outside a soft interval."""
    low_f = float(min(low, high))
    high_f = float(max(low, high))
    value_f = float(value)
    scale_f = max(1e-9, abs(float(scale)))
    if value_f < low_f:
        return _huber_loss((low_f - value_f) / scale_f, cap=cap)
    if value_f > high_f:
        return _huber_loss((value_f - high_f) / scale_f, cap=cap)
    return 0.0


def _interval_loss(value: float, low: float, high: float) -> float:
    low_f = float(min(low, high))
    high_f = float(max(low, high))
    width = max(1e-9, high_f - low_f)
    value_f = float(value)
    if value_f < low_f:
        return _bounded_square_ratio(low_f - value_f, width)
    if value_f > high_f:
        return _bounded_square_ratio(value_f - high_f, width)
    return 0.0


def _soft_window_score(
    value: float,
    full_start: float,
    full_end: float,
    outer_start: float,
    outer_end: float,
) -> float:
    """Unit score in a full-credit window with linear outer ramps."""
    full_low = float(min(full_start, full_end))
    full_high = float(max(full_start, full_end))
    outer_low = float(min(outer_start, full_low))
    outer_high = float(max(outer_end, full_high))
    value_f = float(value)
    if full_low <= value_f <= full_high:
        return 1.0
    if outer_low < value_f < full_low:
        return float((value_f - outer_low) / max(1e-9, full_low - outer_low))
    if full_high < value_f < outer_high:
        return float((outer_high - value_f) / max(1e-9, outer_high - full_high))
    return 0.0


def _resolve_reward_asset_path(spec: str | os.PathLike[str]) -> Path:
    text = os.fspath(spec)
    if os.name != "nt":
        text = text.replace("\\", "/")
    if not text:
        raise FileNotFoundError("empty reward asset path")
    path = Path(text).expanduser()
    candidates = (
        [path] if path.is_absolute() else [Path.cwd() / path, _REWARD_MODULE_DIR / path]
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    searched = ", ".join(str(candidate) for candidate in candidates)
    raise FileNotFoundError(f"reward asset not found: {text!r}; searched: {searched}")


def _load_morphology_profile(
    spec: str | os.PathLike[str] | None,
) -> dict[str, Any] | None:
    """Load the AB06 mean/std prosthetic morphology corridor profile.

    Empty ``spec`` disables the term. A non-empty but unreadable profile fails
    explicitly so a misspelled training YAML does not silently remove the guardrail.
    """
    if spec is None or str(spec).strip() == "":
        return None
    path = _resolve_reward_asset_path(spec)
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, Mapping):
        raise ValueError(f"morphology profile must be a JSON object: {path}")
    if str(data.get("units", "")).lower() not in {"radian", "radians", "rad"}:
        raise ValueError(f"morphology profile must use radians: {path}")

    phase_grid = np.asarray(data.get("phase_grid"), dtype=float)
    if phase_grid.ndim != 1 or phase_grid.size < 2:
        raise ValueError(f"morphology phase_grid must be a 1D array: {path}")
    if not np.all(np.isfinite(phase_grid)):
        raise ValueError(f"morphology phase_grid contains non-finite values: {path}")
    if not np.all(np.diff(phase_grid) > 0.0):
        raise ValueError(f"morphology phase_grid must be strictly increasing: {path}")

    coordinates = data.get("coordinates")
    if not isinstance(coordinates, Mapping):
        raise ValueError(f"morphology profile missing coordinates: {path}")

    parsed_coords: dict[str, dict[str, np.ndarray]] = {}
    for coord_name in _MORPHOLOGY_COORDS:
        coord = coordinates.get(coord_name)
        if not isinstance(coord, Mapping):
            raise ValueError(f"morphology profile missing {coord_name}: {path}")
        mean = np.asarray(coord.get("mean_rad"), dtype=float)
        std = np.asarray(coord.get("std_rad"), dtype=float)
        if mean.shape != phase_grid.shape or std.shape != phase_grid.shape:
            raise ValueError(
                f"morphology {coord_name} arrays must match phase_grid shape: {path}"
            )
        if not np.all(np.isfinite(mean)) or not np.all(np.isfinite(std)):
            raise ValueError(
                f"morphology {coord_name} contains non-finite values: {path}"
            )
        if np.any(std < 0.0):
            raise ValueError(
                f"morphology {coord_name} std_rad must be non-negative: {path}"
            )
        parsed_coords[coord_name] = {"mean_rad": mean, "std_rad": std}

    metadata = data.get("metadata") if isinstance(data.get("metadata"), Mapping) else {}
    return {
        "path": str(path),
        "version": data.get("version"),
        "name": data.get("name"),
        "phase_grid": phase_grid,
        "coordinates": parsed_coords,
        "metadata": dict(metadata),
    }


def _normalize_morphology_phase(phase: float) -> float | None:
    try:
        phase_f = float(phase)
    except (TypeError, ValueError):
        return None
    if not np.isfinite(phase_f):
        return None
    if phase_f < 0.0 or phase_f > 1.0:
        phase_f = phase_f % 1.0
    return float(np.clip(phase_f, 0.0, 1.0))


def _morphology_joint_params(cfg: RewardConfig, coord_name: str) -> tuple[float, float]:
    if coord_name == "pros_knee_angle":
        return (
            float(cfg.morphology_std_multiplier_knee),
            float(np.deg2rad(cfg.morphology_margin_knee_deg)),
        )
    if coord_name == "pros_ankle_angle":
        return (
            float(cfg.morphology_std_multiplier_ankle),
            float(np.deg2rad(cfg.morphology_margin_ankle_deg)),
        )
    raise KeyError(coord_name)


def _morphology_corridor_at(
    profile: Mapping[str, Any],
    phase: float,
    cfg: RewardConfig,
) -> dict[str, dict[str, float]]:
    phase_f = _normalize_morphology_phase(phase)
    if phase_f is None:
        raise ValueError(f"invalid morphology phase: {phase!r}")
    phase_grid = np.asarray(profile["phase_grid"], dtype=float)
    out: dict[str, dict[str, float]] = {}
    for coord_name in _MORPHOLOGY_COORDS:
        coord = profile["coordinates"][coord_name]
        mean = float(np.interp(phase_f, phase_grid, coord["mean_rad"]))
        std = float(np.interp(phase_f, phase_grid, coord["std_rad"]))
        k, margin_rad = _morphology_joint_params(cfg, coord_name)
        low = mean - float(k) * std - margin_rad
        high = mean + float(k) * std + margin_rad
        out[coord_name] = {
            "mean_rad": float(mean),
            "std_rad": float(std),
            "min_rad": float(min(low, high)),
            "max_rad": float(max(low, high)),
        }
    return out


def _morphology_interval_losses(
    value: float,
    low: float,
    high: float,
) -> tuple[float, float, float]:
    low_f = float(min(low, high))
    high_f = float(max(low, high))
    value_f = float(value)
    if value_f < low_f:
        excursion = low_f - value_f
    elif value_f > high_f:
        excursion = value_f - high_f
    else:
        excursion = 0.0
    width = max(1e-9, high_f - low_f)
    loss = min(25.0, (excursion / width) ** 2)
    return float(loss), float(excursion * excursion), float(excursion)


def _morphology_hard_excursions(
    knee_value: float,
    ankle_value: float,
    cfg: RewardConfig,
) -> tuple[float, float]:
    """Return raw excursions beyond provisional, phase-independent hard bounds.

    Equality with a hard boundary is valid.  The bounds are diagnostic unless
    ``morphology_hard_termination_enabled`` is explicitly enabled.
    """
    values = (float(knee_value), float(ankle_value))
    lows = tuple(float(value) for value in cfg.morphology_hard_q_min)
    highs = tuple(float(value) for value in cfg.morphology_hard_q_max)
    if len(lows) < 2 or len(highs) < 2:
        raise ValueError(
            "morphology_hard_q_min/max must each contain knee and ankle bounds"
        )
    excursions: list[float] = []
    for value, low, high in zip(values, lows[:2], highs[:2]):
        low_f = min(low, high)
        high_f = max(low, high)
        excursions.append(max(low_f - value, value - high_f, 0.0))
    return float(excursions[0]), float(excursions[1])


def _normalize_morphology_phase_mode(value: str) -> str:
    mode = str(value or "legacy_cycle_fraction").strip().lower().replace("-", "_")
    aliases = {
        "legacy": "legacy_cycle_fraction",
        "legacy_cycle_fraction": "legacy_cycle_fraction",
        "legacy_fsm": "legacy_cycle_fraction",
        "fsm_legacy": "legacy_cycle_fraction",
        "event": "event_anchored",
        "event_anchored": "event_anchored",
        "profile_event_anchored": "event_anchored",
        "hs_to_anchored": "event_anchored",
        "event_anchored_completed_segment_experimental": EXPERIMENTAL_PHASE_MODE,
        "completed_segment_experimental": EXPERIMENTAL_PHASE_MODE,
        "event_retrospective_experimental": EXPERIMENTAL_PHASE_MODE,
        "event_anchored_causal_delayed_experimental": CAUSAL_DELAYED_PHASE_MODE,
        "causal_delayed_experimental": CAUSAL_DELAYED_PHASE_MODE,
        "v26_causal_delayed": CAUSAL_DELAYED_PHASE_MODE,
    }
    try:
        return aliases[mode]
    except KeyError as exc:
        supported = ", ".join(sorted(set(aliases.values())))
        raise ValueError(
            f"unsupported morphology_phase_mode={value!r}; expected {supported}"
        ) from exc


_EVENT_WARPED_PHASE_PARAMETERIZATION = "event_warped_hs_to_to_to_hs_v1"


def _morphology_canonical_to_phase(
    profile: Mapping[str, Any],
    *,
    require_event_contract: bool = False,
) -> float:
    metadata = profile.get("metadata")
    if not isinstance(metadata, Mapping):
        metadata = {}
    if require_event_contract:
        parameterization = metadata.get("phase_parameterization")
        if parameterization != _EVENT_WARPED_PHASE_PARAMETERIZATION:
            raise ValueError(
                "event-anchored morphology profile metadata must declare "
                "phase_parameterization="
                f"{_EVENT_WARPED_PHASE_PARAMETERIZATION!r}; got "
                f"{parameterization!r}"
            )
        candidates = (metadata.get("canonical_to_phase"),)
    else:
        candidates = (
            metadata.get("canonical_to_phase"),
            metadata.get("mean_to_phase"),
        )
    for candidate in candidates:
        try:
            value = float(candidate)
        except (TypeError, ValueError):
            continue
        if np.isfinite(value) and 0.0 < value < 1.0:
            return value
    raise ValueError(
        "event-anchored morphology profile metadata must define a finite "
        "canonical_to_phase (or mean_to_phase) strictly inside (0, 1)"
    )


def _fsm_morphology_phase(
    info: Mapping[str, Any],
    cfg: RewardConfig,
    *,
    canonical_to_phase: float | None = None,
) -> tuple[float | None, float, float]:
    """Return morphology phase from the prosthetic HS-TO-HS FSM.

    The returned tuple is ``(phase, available, source_id)`` where source id is:
    0 = unavailable, 1 = nominal bootstrap timing, 2 = measured prosthetic
    period/stance fraction from the last completed cycle, 4 = robust median of
    recent valid stance and swing durations. Source id 3 remains reserved for
    the non-FSM online-gait fallback used by ``_morphology_terms``.
    """
    fsm = info.get("phase_fsm")
    if not isinstance(fsm, Mapping):
        return None, 0.0, 0.0

    state_id = int(float(fsm.get("state_id", 0.0) or 0.0))
    stance_elapsed_s = max(0.0, float(fsm.get("stance_elapsed_s", 0.0) or 0.0))
    swing_elapsed_s = max(0.0, float(fsm.get("swing_elapsed_s", 0.0) or 0.0))
    valid_cycle_count = float(fsm.get("valid_cycle_count", 0.0) or 0.0)
    last_period_s = float(fsm.get("last_period_s", 0.0) or 0.0)
    last_stance_fraction = float(fsm.get("last_stance_fraction", 0.0) or 0.0)
    robust_stance_duration_s = float(fsm.get("robust_stance_duration_s", 0.0) or 0.0)
    robust_swing_duration_s = float(fsm.get("robust_swing_duration_s", 0.0) or 0.0)
    duration_history_count = float(fsm.get("duration_history_count", 0.0) or 0.0)

    period_s = max(1e-9, float(cfg.phase_period_nominal_s))
    stance_fraction = float(
        np.clip(float(cfg.prosthetic_stance_phase_end), 1e-6, 1.0 - 1e-6)
    )
    source_id = 1.0
    if (
        valid_cycle_count > 0.0
        and last_period_s > 1e-9
        and 1e-6 < last_stance_fraction < 1.0 - 1e-6
    ):
        period_s = last_period_s
        stance_fraction = float(np.clip(last_stance_fraction, 1e-6, 1.0 - 1e-6))
        source_id = 2.0

    stance_duration_s = max(1e-9, period_s * stance_fraction)
    swing_duration_s = max(1e-9, period_s * (1.0 - stance_fraction))
    mode = _normalize_morphology_phase_mode(cfg.morphology_phase_mode)
    phase_boundary = stance_fraction
    if mode == "event_anchored":
        if (
            duration_history_count > 0.0
            and robust_stance_duration_s > 1e-9
            and robust_swing_duration_s > 1e-9
        ):
            stance_duration_s = robust_stance_duration_s
            swing_duration_s = robust_swing_duration_s
            source_id = 4.0
        if canonical_to_phase is None:
            canonical_to_phase = float(cfg.prosthetic_stance_phase_end)
        phase_boundary = float(np.clip(canonical_to_phase, 1e-6, 1.0 - 1e-6))

    stance_progress = stance_elapsed_s / stance_duration_s
    swing_progress = swing_elapsed_s / swing_duration_s
    if mode == "event_anchored":
        # A late event may hold at its anchor, but can never cross into the next
        # segment before the FSM actually detects that event.
        stance_progress = float(np.clip(stance_progress, 0.0, 1.0))
        swing_progress = float(np.clip(swing_progress, 0.0, 1.0))

    if state_id == 1:  # STANCE_AFTER_HS
        phase = phase_boundary * stance_progress
    elif state_id == 2:  # SWING_AFTER_TO
        phase = phase_boundary + (1.0 - phase_boundary) * swing_progress
    elif state_id == 4:  # TIMEOUT; preserve terminal phase diagnostics.
        timeout_side = float(fsm.get("timeout_side", 0.0) or 0.0)
        if swing_elapsed_s > 0.0 or timeout_side == 2.0:
            phase = phase_boundary + (1.0 - phase_boundary) * swing_progress
        elif stance_elapsed_s > 0.0 or timeout_side == 1.0:
            phase = phase_boundary * stance_progress
        else:
            return None, 0.0, 0.0
    else:
        return None, 0.0, 0.0

    if not np.isfinite(phase):
        return None, 0.0, 0.0
    return float(np.clip(phase, 0.0, 1.0)), 1.0, source_id


def _out_of_band_losses(
    reference,
    cfg: RewardConfig,
    *,
    bounds: tuple[tuple[float, ...], tuple[float, ...]] | None = None,
) -> tuple[float, float]:
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
    q_min, q_max = bounds or (cfg.oob_q_min, cfg.oob_q_max)
    n = min(ref.shape[1], len(q_min), len(q_max))
    if n == 0:
        return 0.0, 0.0
    ref = ref[:, :n]
    low = np.asarray(q_min[:n], dtype=float)
    high = np.asarray(q_max[:n], dtype=float)
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
    reference_score = _score(
        reward_terms.get(REFERENCE_LOSS, 0.0), cfg.reference_weight
    )
    bio_score = _score(reward_terms.get(BIO_LOSS, 0.0), cfg.bio_weight)
    contact_load_score = float(reward_terms.get("contact_load_score", 0.0))
    contact_support_to_score = float(reward_terms.get("contact_support_to_score", 0.0))
    phase_regular_score = float(reward_terms.get("phase_regular_score", 0.0))
    phase_event_progress_score = float(
        reward_terms.get("phase_event_progress_score", 0.0)
    )
    landing_window_contact_score = float(
        reward_terms.get("landing_window_contact_score", 0.0)
    )
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
        + cfg.segment_delta_weight * float(reward_terms.get(SEGMENT_DELTA_LOSS, 0.0))
        + cfg.qdot_ref_weight * float(reward_terms.get(QDOT_REF_LOSS, 0.0))
        + cfg.qddot_ref_weight * float(reward_terms.get(QDDOT_REF_LOSS, 0.0))
        + cfg.jerk_ref_weight * float(reward_terms.get(JERK_REF_LOSS, 0.0))
        + cfg.reference_governor_weight
        * float(reward_terms.get(REFERENCE_GOVERNOR_LOSS, 0.0))
        + cfg.u_rate_weight * float(reward_terms.get(U_RATE_LOSS, 0.0))
        + cfg.sea_saturation_weight * float(reward_terms.get(SEA_SATURATION_LOSS, 0.0))
        + cfg.sea_torque_error_weight
        * float(reward_terms.get(SEA_TORQUE_ERROR_LOSS, 0.0))
        + cfg.sea_tau_spring_effort_weight
        * float(reward_terms.get(SEA_TAU_SPRING_EFFORT_LOSS, 0.0))
        + cfg.sea_tau_spring_rate_weight
        * float(reward_terms.get(SEA_TAU_SPRING_RATE_LOSS, 0.0))
        + cfg.sea_motor_speed_weight
        * float(reward_terms.get(SEA_MOTOR_SPEED_LOSS, 0.0))
        + cfg.sea_motor_accel_weight
        * float(reward_terms.get(SEA_MOTOR_ACCEL_LOSS, 0.0))
        + cfg.sea_motor_power_weight
        * float(reward_terms.get(SEA_MOTOR_POWER_LOSS, 0.0))
        + cfg.policy_action_clip_weight
        * float(reward_terms.get(POLICY_ACTION_CLIP_LOSS, 0.0))
        + cfg.phase_invalid_event_weight
        * float(reward_terms.get("invalid_event_loss", 0.0))
        + cfg.phase_contact_validity_weight
        * float(reward_terms.get("contact_validity_loss", 0.0))
    )
    safety_term = cfg.safety_weight * float(reward_terms.get(SAFETY_LOSS, 0.0))
    grf_penetration_term = cfg.grf_penetration_weight * float(
        reward_terms.get(GRF_PENETRATION_LOSS, 0.0)
    )
    grf_ankle_moment_flip_term = cfg.grf_ankle_moment_flip_weight * float(
        reward_terms.get(GRF_ANKLE_MOMENT_FLIP_LOSS, 0.0)
    )
    phase_timeout_penalty_term = cfg.phase_timeout_penalty_weight * float(
        reward_terms.get("phase_timeout_loss", 0.0)
    )
    phase_clawback_penalty_term = cfg.phase_clawback_penalty_weight * (
        float(reward_terms.get("phase_clawback_penalty", 0.0))
        + float(reward_terms.get("phase_failure_extra_penalty", 0.0))
    )
    contact_support_clawback_term = cfg.contact_support_failure_clawback_weight * float(
        reward_terms.get("contact_support_clawback_penalty", 0.0)
    )
    exnovo_task_term = (
        cfg.swing_unloading_weight
        * float(reward_terms.get("swing_unloading_loss", 0.0))
        + cfg.contact_overload_weight
        * float(reward_terms.get("contact_overload_loss", 0.0))
        + cfg.grf_slip_weight * float(reward_terms.get("grf_slip_loss", 0.0))
        + cfg.reserve_residual_weight
        * float(reward_terms.get("reserve_residual_loss", 0.0))
        + cfg.pelvis_height_weight * float(reward_terms.get("pelvis_height_loss", 0.0))
    )
    prosthetic_joint_range_term = cfg.prosthetic_joint_range_weight * float(
        reward_terms.get("prosthetic_joint_range_loss", 0.0)
    )
    morphology_weight = float(cfg.morphology_weight)
    if not np.isfinite(morphology_weight) or morphology_weight < 0.0:
        raise ValueError("morphology_weight must be finite and non-negative")
    morphology_loss_raw = float(reward_terms.get("morphology_loss", 0.0))
    morphology_loss_nonfinite = not np.isfinite(morphology_loss_raw)
    if morphology_weight == 0.0:
        # Strict no-effect branch: never evaluate ``0 * morphology_loss``.
        # This preserves reward parity and prevents ``0 * NaN`` propagation.
        morphology_term = 0.0
    else:
        if morphology_loss_nonfinite:
            raise ValueError(
                "positive morphology_weight requires a finite morphology_loss"
            )
        morphology_term = morphology_weight * morphology_loss_raw
    morphology_loss_component = (
        0.0 if morphology_loss_nonfinite else morphology_loss_raw
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
            + cfg.blend_contact_load * contact_load_score
            + cfg.blend_contact_support_to * contact_support_to_score
            + cfg.blend_phase_regular * phase_regular_score
            + cfg.blend_phase_event_progress * phase_event_progress_score
            + cfg.blend_landing_window_contact * landing_window_contact_score
            - penalty
        )
    base = min(cfg.clip_high, max(cfg.clip_low, base))

    # Safety, contact feasibility and out-of-band are subtracted AFTER the clip:
    # they stay active even once the positive reward has saturated to clip_low.
    reward = (
        base
        - safety_term
        - grf_penetration_term
        - grf_ankle_moment_flip_term
        - phase_timeout_penalty_term
        - phase_clawback_penalty_term
        - contact_support_clawback_term
        - exnovo_task_term
        - prosthetic_joint_range_term
        - morphology_term
        - oob_term
    )

    components = {
        "reward": float(reward),
        "reward_base": float(base),
        "tracking_score": float(tracking_score),
        "reference_score": float(reference_score),
        "bio_score": float(bio_score),
        "contact_load_score": float(contact_load_score),
        "contact_support_to_score": float(contact_support_to_score),
        "phase_regular_score": float(phase_regular_score),
        "phase_event_progress_score": float(phase_event_progress_score),
        "landing_window_contact_score": float(landing_window_contact_score),
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
        "served_imitation_loss": float(reward_terms.get(SERVED_IMITATION_LOSS, 0.0)),
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
        "sea_torque_error_loss": float(reward_terms.get(SEA_TORQUE_ERROR_LOSS, 0.0)),
        "sea_tau_spring_effort_loss": float(
            reward_terms.get(SEA_TAU_SPRING_EFFORT_LOSS, 0.0)
        ),
        "sea_tau_spring_rate_loss": float(
            reward_terms.get(SEA_TAU_SPRING_RATE_LOSS, 0.0)
        ),
        "sea_motor_speed_loss": float(reward_terms.get(SEA_MOTOR_SPEED_LOSS, 0.0)),
        "sea_motor_accel_loss": float(reward_terms.get(SEA_MOTOR_ACCEL_LOSS, 0.0)),
        "sea_motor_power_loss": float(reward_terms.get(SEA_MOTOR_POWER_LOSS, 0.0)),
        "policy_action_clip_loss": float(
            reward_terms.get(POLICY_ACTION_CLIP_LOSS, 0.0)
        ),
        "policy_action_clip_fraction": float(
            reward_terms.get("policy_action_clip_fraction", 0.0)
        ),
        "policy_action_clip_abs_max": float(
            reward_terms.get("policy_action_clip_abs_max", 0.0)
        ),
        "invalid_event_loss": float(reward_terms.get("invalid_event_loss", 0.0)),
        "contact_validity_loss": float(reward_terms.get("contact_validity_loss", 0.0)),
        "invalid_event_count": float(reward_terms.get("invalid_event_count", 0.0)),
        "penalty": float(penalty),
        "safety_term": float(safety_term),
        "grf_penetration_loss": float(reward_terms.get(GRF_PENETRATION_LOSS, 0.0)),
        "grf_penetration_term": float(grf_penetration_term),
        "grf_ankle_moment_flip_loss": float(
            reward_terms.get(GRF_ANKLE_MOMENT_FLIP_LOSS, 0.0)
        ),
        "grf_ankle_moment_flip_term": float(grf_ankle_moment_flip_term),
        "phase_timeout_penalty_term": float(phase_timeout_penalty_term),
        "phase_clawback_penalty": float(
            reward_terms.get("phase_clawback_penalty", 0.0)
        ),
        "phase_failure_extra_penalty": float(
            reward_terms.get("phase_failure_extra_penalty", 0.0)
        ),
        "phase_clawback_penalty_term": float(phase_clawback_penalty_term),
        "contact_support_clawback_penalty": float(
            reward_terms.get("contact_support_clawback_penalty", 0.0)
        ),
        "contact_support_clawback_term": float(contact_support_clawback_term),
        "phase_cycle_complete_bonus": float(
            reward_terms.get("phase_cycle_complete_bonus", 0.0)
        ),
        "phase_cycle_failed_this_step": float(
            reward_terms.get("phase_cycle_failed_this_step", 0.0)
        ),
        "phase_pending_cycle_credit": float(
            reward_terms.get("phase_pending_cycle_credit", 0.0)
        ),
        "grf_ankle_moment_flip_tau_nm": float(
            reward_terms.get("grf_ankle_moment_flip_tau_nm", 0.0)
        ),
        "grf_ankle_moment_flip_excess_nm": float(
            reward_terms.get("grf_ankle_moment_flip_excess_nm", 0.0)
        ),
        "grf_ankle_moment_flip_raw_loss_nm2": float(
            reward_terms.get("grf_ankle_moment_flip_raw_loss_nm2", 0.0)
        ),
        "contact_load_loss": float(reward_terms.get("contact_load_loss", 0.0)),
        "contact_load_raw_score": float(
            reward_terms.get("contact_load_raw_score", 0.0)
        ),
        "contact_load_dense_active": float(
            reward_terms.get("contact_load_dense_active", 0.0)
        ),
        "contact_load_evidence_complete": float(
            reward_terms.get("contact_load_evidence_complete", 0.0)
        ),
        "contact_load_penetration_quality": float(
            reward_terms.get("contact_load_penetration_quality", 0.0)
        ),
        "contact_support_to_timing_score": float(
            reward_terms.get("contact_support_to_timing_score", 0.0)
        ),
        "contact_support_stance_duration_s": float(
            reward_terms.get("contact_support_stance_duration_s", 0.0)
        ),
        "contact_support_min_penetration_quality": float(
            reward_terms.get("contact_support_min_penetration_quality", 0.0)
        ),
        "contact_support_mean_penetration_quality": float(
            reward_terms.get("contact_support_mean_penetration_quality", 0.0)
        ),
        "contact_support_pending_dense_reward": float(
            reward_terms.get("contact_support_pending_dense_reward", 0.0)
        ),
        "contact_support_dense_confirmed_reward": float(
            reward_terms.get("contact_support_dense_confirmed_reward", 0.0)
        ),
        "contact_overload_loss": float(reward_terms.get("contact_overload_loss", 0.0)),
        "swing_unloading_loss": float(reward_terms.get("swing_unloading_loss", 0.0)),
        "grf_slip_loss": float(reward_terms.get("grf_slip_loss", 0.0)),
        "phase_regularity_loss": float(reward_terms.get("phase_regularity_loss", 0.0)),
        "phase_event_order_loss": float(
            reward_terms.get("phase_event_order_loss", 0.0)
        ),
        "phase_period_loss": float(reward_terms.get("phase_period_loss", 0.0)),
        "phase_period_hard_loss": float(
            reward_terms.get("phase_period_hard_loss", 0.0)
        ),
        "phase_periodicity_loss": float(
            reward_terms.get("phase_periodicity_loss", 0.0)
        ),
        "phase_stance_fraction_loss": float(
            reward_terms.get("phase_stance_fraction_loss", 0.0)
        ),
        "phase_timeout_loss": float(reward_terms.get("phase_timeout_loss", 0.0)),
        "phase_stance_timeout_loss": float(
            reward_terms.get("phase_stance_timeout_loss", 0.0)
        ),
        "phase_swing_timeout_loss": float(
            reward_terms.get("phase_swing_timeout_loss", 0.0)
        ),
        "reserve_residual_loss": float(reward_terms.get("reserve_residual_loss", 0.0)),
        "reserve_norm_loss": float(reward_terms.get("reserve_norm_loss", 0.0)),
        "residual_norm_loss": float(reward_terms.get("residual_norm_loss", 0.0)),
        "pelvis_height_loss": float(reward_terms.get("pelvis_height_loss", 0.0)),
        "exnovo_task_term": float(exnovo_task_term),
        "prosthetic_joint_range_loss": float(
            reward_terms.get("prosthetic_joint_range_loss", 0.0)
        ),
        "prosthetic_joint_range_raw_loss": float(
            reward_terms.get("prosthetic_joint_range_raw_loss", 0.0)
        ),
        "prosthetic_joint_range_term": float(prosthetic_joint_range_term),
        "morphology_loss": float(morphology_loss_component),
        "morphology_loss_input_nonfinite": float(morphology_loss_nonfinite),
        "morphology_loss_mean": float(reward_terms.get("morphology_loss_mean", 0.0)),
        "morphology_inside_score": float(
            reward_terms.get("morphology_inside_score", 0.0)
        ),
        "morphology_inside_fraction": float(
            reward_terms.get("morphology_inside_fraction", 0.0)
        ),
        "morphology_knee_loss": float(reward_terms.get("morphology_knee_loss", 0.0)),
        "morphology_ankle_loss": float(reward_terms.get("morphology_ankle_loss", 0.0)),
        "morphology_knee_raw_loss_rad2": float(
            reward_terms.get("morphology_knee_raw_loss_rad2", 0.0)
        ),
        "morphology_ankle_raw_loss_rad2": float(
            reward_terms.get("morphology_ankle_raw_loss_rad2", 0.0)
        ),
        "morphology_knee_excursion_rad": float(
            reward_terms.get("morphology_knee_excursion_rad", 0.0)
        ),
        "morphology_ankle_excursion_rad": float(
            reward_terms.get("morphology_ankle_excursion_rad", 0.0)
        ),
        "morphology_available": float(reward_terms.get("morphology_available", 0.0)),
        "morphology_phase": float(reward_terms.get("morphology_phase", 0.0)),
        "fsm_morphology_phase": float(reward_terms.get("fsm_morphology_phase", 0.0)),
        "morphology_phase_source_id": float(
            reward_terms.get("morphology_phase_source_id", 0.0)
        ),
        "morphology_phase_fsm_available": float(
            reward_terms.get("morphology_phase_fsm_available", 0.0)
        ),
        "morphology_phase_mode_id": float(
            reward_terms.get("morphology_phase_mode_id", 0.0)
        ),
        "morphology_canonical_to_phase": float(
            reward_terms.get("morphology_canonical_to_phase", 0.0)
        ),
        "morphology_settled_this_step": float(
            reward_terms.get("morphology_settled_this_step", 0.0)
        ),
        "morphology_settled_sample_count": float(
            reward_terms.get("morphology_settled_sample_count", 0.0)
        ),
        "morphology_pending_sample_count": float(
            reward_terms.get("morphology_pending_sample_count", 0.0)
        ),
        "morphology_active_segment_id": float(
            reward_terms.get("morphology_active_segment_id", 0.0)
        ),
        "morphology_discarded_segment_count": float(
            reward_terms.get("morphology_discarded_segment_count", 0.0)
        ),
        "morphology_discarded_sample_count": float(
            reward_terms.get("morphology_discarded_sample_count", 0.0)
        ),
        "morphology_ledger_overflow": float(
            reward_terms.get("morphology_ledger_overflow", 0.0)
        ),
        "morphology_ledger_nonmonotonic_sample": float(
            reward_terms.get("morphology_ledger_nonmonotonic_sample", 0.0)
        ),
        "morphology_segment_duration_s": float(
            reward_terms.get("morphology_segment_duration_s", 0.0)
        ),
        "morphology_segment_type_id": float(
            reward_terms.get("morphology_segment_type_id", 0.0)
        ),
        "morphology_hard_violation": float(
            reward_terms.get("morphology_hard_violation", 0.0)
        ),
        "morphology_hard_knee_excursion_rad": float(
            reward_terms.get("morphology_hard_knee_excursion_rad", 0.0)
        ),
        "morphology_hard_ankle_excursion_rad": float(
            reward_terms.get("morphology_hard_ankle_excursion_rad", 0.0)
        ),
        "morphology_hard_max_excursion_rad": float(
            reward_terms.get("morphology_hard_max_excursion_rad", 0.0)
        ),
        "morphology_causal_delay_s": float(
            reward_terms.get("morphology_causal_delay_s", 0.0)
        ),
        "morphology_causal_terminal_flushed": float(
            reward_terms.get("morphology_causal_terminal_flushed", 0.0)
        ),
        "morphology_causal_dropped_pending_count": float(
            reward_terms.get("morphology_causal_dropped_pending_count", 0.0)
        ),
        "morphology_causal_dropped_wait_hs_count": float(
            reward_terms.get("morphology_causal_dropped_wait_hs_count", 0.0)
        ),
        "morphology_causal_cancelled_transition_count": float(
            reward_terms.get(
                "morphology_causal_cancelled_transition_count",
                0.0,
            )
        ),
        "morphology_causal_timeout_transition_count": float(
            reward_terms.get("morphology_causal_timeout_transition_count", 0.0)
        ),
        "morphology_causal_failed_closed": float(
            reward_terms.get("morphology_causal_failed_closed", 0.0)
        ),
        "morphology_knee_value_rad": float(
            reward_terms.get("morphology_knee_value_rad", 0.0)
        ),
        "morphology_ankle_value_rad": float(
            reward_terms.get("morphology_ankle_value_rad", 0.0)
        ),
        "morphology_knee_min_rad": float(
            reward_terms.get("morphology_knee_min_rad", 0.0)
        ),
        "morphology_knee_max_rad": float(
            reward_terms.get("morphology_knee_max_rad", 0.0)
        ),
        "morphology_ankle_min_rad": float(
            reward_terms.get("morphology_ankle_min_rad", 0.0)
        ),
        "morphology_ankle_max_rad": float(
            reward_terms.get("morphology_ankle_max_rad", 0.0)
        ),
        "morphology_term": float(morphology_term),
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
    text = str(spec)
    path_text = text.replace("\\", "/") if os.name != "nt" else text
    path = Path(path_text)
    text = path.read_text(encoding="utf-8") if path.exists() else spec
    data = json.loads(text)
    if not isinstance(data, dict):
        raise ValueError("reward override must be a JSON object of RewardConfig fields")
    return data


class RewardShapingWrapper(gym.Wrapper):
    """Replace the env reward with ``compute_reward`` over ``info['reward_terms']``.

    The action is forwarded unchanged to the wrapped env; if an inner
    ``ActionWrapper`` clips it, this wrapper logs the raw-vs-bound excursion
    against the advertised action space. The out-of-band penalty uses the
    commanded reference of the step
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
        self._morphology_profile = _load_morphology_profile(
            self.reward_config.morphology_profile
        )
        self._morphology_phase_mode = _normalize_morphology_phase_mode(
            self.reward_config.morphology_phase_mode
        )
        self._morphology_canonical_to_phase: float | None = None
        if self._morphology_profile is not None and self._morphology_phase_mode in {
            "event_anchored",
            EXPERIMENTAL_PHASE_MODE,
            CAUSAL_DELAYED_PHASE_MODE,
        }:
            self._morphology_canonical_to_phase = _morphology_canonical_to_phase(
                self._morphology_profile,
                require_event_contract=True,
            )
        self._experimental_morphology_ledger: (
            CompletedSegmentMorphologyLedger | None
        ) = None
        if self._morphology_phase_mode == EXPERIMENTAL_PHASE_MODE:
            if self._morphology_profile is None:
                raise ValueError(
                    "experimental completed-segment morphology requires a "
                    "non-empty event-warped morphology_profile"
                )
            effects_requested = (
                abs(float(self.reward_config.morphology_weight)) > 0.0
                or float(self.reward_config.morphology_hard_termination_enabled) > 0.0
            )
            if (
                effects_requested
                and float(self.reward_config.morphology_experimental_allow_effects)
                <= 0.0
            ):
                raise ValueError(
                    "experimental completed-segment morphology is shadow-only: "
                    "keep morphology_weight=0 and hard termination disabled; "
                    "set morphology_experimental_allow_effects=1 only in an "
                    "explicit synthetic/research validation"
                )
            self._experimental_morphology_ledger = CompletedSegmentMorphologyLedger(
                max_samples=int(
                    self.reward_config.morphology_completed_segment_max_samples
                )
            )
        self._causal_morphology_buffer: CausalDelayedMorphologyBuffer | None = None
        if self._morphology_phase_mode == CAUSAL_DELAYED_PHASE_MODE:
            if self._morphology_profile is None:
                raise ValueError(
                    "causal delayed morphology requires a non-empty "
                    "event-warped morphology_profile"
                )
            if (
                str(self.reward_config.morphology_causal_event_contract_id)
                != V26_EVENT_CONTRACT_ID
            ):
                raise ValueError(
                    "causal delayed morphology requires the frozen V26 event "
                    "contract"
                )
            if float(self.reward_config.morphology_hard_termination_enabled) > 0.0:
                raise ValueError(
                    "causal delayed morphology does not authorize the "
                    "retrospective hard-termination hook"
                )
            if (
                float(self.reward_config.morphology_weight) > 0.0
                and float(self.reward_config.morphology_causal_allow_effects) <= 0.0
            ):
                raise ValueError(
                    "causal delayed morphology positive reward is gated; keep "
                    "morphology_weight=0 until Q2 authorizes an explicit A/B"
                )
            period_s = float(self.reward_config.phase_period_nominal_s)
            stance_fraction = float(
                np.clip(
                    self.reward_config.prosthetic_stance_phase_end,
                    1e-6,
                    1.0 - 1e-6,
                )
            )
            self._causal_morphology_buffer = CausalDelayedMorphologyBuffer(
                delay_s=float(self.reward_config.morphology_reward_delay_s),
                canonical_to_phase=float(self._morphology_canonical_to_phase),
                nominal_stance_duration_s=period_s * stance_fraction,
                nominal_swing_duration_s=period_s * (1.0 - stance_fraction),
                max_delivery_latency_s=float(
                    self.reward_config.morphology_max_delivery_latency_s
                ),
                max_samples=int(self.reward_config.morphology_causal_max_samples),
                event_contract_id=str(
                    self.reward_config.morphology_causal_event_contract_id
                ),
            )
        self._morphology_completed_segments_payload: list[dict[str, Any]] = []
        self._morphology_causal_samples_payload: list[dict[str, Any]] = []
        self._morphology_ledger_diagnostics_payload: dict[str, Any] = {}
        self._morphology_causal_diagnostics_payload: dict[str, Any] = {}
        self._reset_contact_support_state()

    def reset(self, **kwargs):
        self._reset_contact_support_state()
        if self._experimental_morphology_ledger is not None:
            self._experimental_morphology_ledger.reset()
        if self._causal_morphology_buffer is not None:
            self._causal_morphology_buffer.reset()
        self._morphology_completed_segments_payload = []
        self._morphology_causal_samples_payload = []
        self._morphology_ledger_diagnostics_payload = {}
        self._morphology_causal_diagnostics_payload = {}
        return self.env.reset(**kwargs)

    def _reset_contact_support_state(self) -> None:
        self._contact_support_last_valid_hs_count = 0.0
        self._contact_support_last_valid_to_count = 0.0
        self._contact_support_pending_dense_reward = 0.0
        self._contact_support_min_penetration_quality = 1.0
        self._contact_support_penetration_quality_sum = 0.0
        self._contact_support_penetration_quality_samples = 0
        self._contact_support_stance_seen = False

    def _action_clip_terms(self, action) -> dict[str, float]:
        space = getattr(self.env, "action_space", None)
        low = getattr(space, "low", None)
        high = getattr(space, "high", None)
        if low is None or high is None:
            return {
                POLICY_ACTION_CLIP_LOSS: 0.0,
                "policy_action_clip_fraction": 0.0,
                "policy_action_clip_abs_max": 0.0,
            }
        raw = np.asarray(action, dtype=float)
        try:
            low_arr = np.broadcast_to(np.asarray(low, dtype=float), raw.shape)
            high_arr = np.broadcast_to(np.asarray(high, dtype=float), raw.shape)
        except ValueError:
            return {
                POLICY_ACTION_CLIP_LOSS: 0.0,
                "policy_action_clip_fraction": 0.0,
                "policy_action_clip_abs_max": 0.0,
            }

        finite_bounds = np.isfinite(low_arr) | np.isfinite(high_arr)
        if not np.any(finite_bounds):
            return {
                POLICY_ACTION_CLIP_LOSS: 0.0,
                "policy_action_clip_fraction": 0.0,
                "policy_action_clip_abs_max": 0.0,
            }

        clipped = np.clip(raw, low_arr, high_arr)
        delta = np.abs(raw - clipped)
        valid = finite_bounds & np.isfinite(delta)
        if not np.any(valid):
            return {
                POLICY_ACTION_CLIP_LOSS: 0.0,
                "policy_action_clip_fraction": 0.0,
                "policy_action_clip_abs_max": 0.0,
            }

        clipped_delta = delta[valid]
        squared = np.square(clipped_delta)
        bounded = squared / (1.0 + squared)
        return {
            POLICY_ACTION_CLIP_LOSS: float(np.mean(bounded)),
            "policy_action_clip_fraction": float(np.mean(clipped_delta > 1e-9)),
            "policy_action_clip_abs_max": float(np.max(clipped_delta)),
        }

    def _contact_support_terms(
        self,
        info: Mapping[str, Any],
        *,
        stance_expected: bool,
        candidate_contact_score: float,
        penetration_quality: float,
    ) -> dict[str, float]:
        """Gate dense support credit and settle its provisional cycle ledger."""
        cfg = self.reward_config
        fsm = info.get("phase_fsm")
        if not isinstance(fsm, Mapping):
            fsm = {}

        ledger_enabled = any(
            (
                float(cfg.contact_load_confidence_full_bw) > 0.0,
                float(cfg.contact_load_dense_evidence_limit_bw_s) > 0.0,
                float(cfg.blend_contact_support_to) > 0.0,
                float(cfg.contact_support_failure_clawback_weight) > 0.0,
            )
        )
        if not ledger_enabled:
            return {
                "contact_load_score": float(candidate_contact_score),
                "contact_load_dense_active": float(stance_expected),
                "contact_load_evidence_complete": 0.0,
                "contact_support_to_score": 0.0,
                "contact_support_to_timing_score": 0.0,
                "contact_support_stance_duration_s": 0.0,
                "contact_support_min_penetration_quality": float(penetration_quality),
                "contact_support_mean_penetration_quality": float(penetration_quality),
                "contact_support_pending_dense_reward": 0.0,
                "contact_support_dense_confirmed_reward": 0.0,
                "contact_support_clawback_penalty": 0.0,
            }

        valid_hs_count = float(fsm.get("valid_hs_count", 0.0) or 0.0)
        valid_to_count = float(fsm.get("valid_to_count", 0.0) or 0.0)
        new_hs = valid_hs_count > self._contact_support_last_valid_hs_count
        new_to = valid_to_count > self._contact_support_last_valid_to_count
        state_id = int(float(fsm.get("state_id", 0.0) or 0.0))
        stance_load_integral = max(
            0.0,
            float(fsm.get("stance_load_integral_bw_s", 0.0) or 0.0),
        )

        reward_terms = info.get("reward_terms")
        if not isinstance(reward_terms, Mapping):
            reward_terms = {}
        episode_ended = bool(
            float(reward_terms.get("terminated", 0.0) or 0.0) > 0.0
            or float(reward_terms.get("truncated", 0.0) or 0.0) > 0.0
            or info.get("end_reason")
        )
        cycle_failed = bool(
            float(fsm.get("phase_cycle_failed_this_step", 0.0) or 0.0) > 0.0
            or float(fsm.get("timeout_exceeded", 0.0) or 0.0) > 0.0
            or (episode_ended and not new_to)
        )

        clawback = 0.0
        if cycle_failed:
            clawback = float(self._contact_support_pending_dense_reward)
            self._contact_support_pending_dense_reward = 0.0

        if new_hs:
            self._contact_support_pending_dense_reward = 0.0
            self._contact_support_min_penetration_quality = 1.0
            self._contact_support_penetration_quality_sum = 0.0
            self._contact_support_penetration_quality_samples = 0
            self._contact_support_stance_seen = False

        dense_active = False
        contact_load_score = 0.0
        evidence_limit = max(
            0.0,
            float(cfg.contact_load_dense_evidence_limit_bw_s),
        )
        evidence_complete = bool(
            evidence_limit > 0.0 and stance_load_integral >= evidence_limit
        )
        if state_id == 1 and stance_expected and not episode_ended:
            self._contact_support_stance_seen = True
            self._contact_support_min_penetration_quality = min(
                self._contact_support_min_penetration_quality,
                float(penetration_quality),
            )
            self._contact_support_penetration_quality_sum += float(penetration_quality)
            self._contact_support_penetration_quality_samples += 1
            dense_active = evidence_limit <= 0.0 or not evidence_complete
            if dense_active:
                contact_load_score = float(candidate_contact_score)
                self._contact_support_pending_dense_reward += (
                    float(cfg.blend_contact_load) * contact_load_score
                )

        to_score = 0.0
        timing_score = 0.0
        stance_duration_s = 0.0
        dense_confirmed = 0.0
        if new_to:
            gait = info.get("online_gait")
            sides = gait.get("sides") if isinstance(gait, Mapping) else None
            left = sides.get("left") if isinstance(sides, Mapping) else None
            if isinstance(left, Mapping):
                last_hs = left.get("last_heel_strike_time")
                last_to = left.get("last_toe_off_time")
                if last_hs is not None and last_to is not None:
                    stance_duration_s = max(0.0, float(last_to) - float(last_hs))
            window_start = float(cfg.contact_support_to_window_start_s)
            window_end = float(cfg.contact_support_to_window_end_s)
            if window_end > window_start > 0.0:
                timing_score = _soft_window_score(
                    stance_duration_s,
                    window_start,
                    window_end,
                    float(cfg.phase_min_stance_duration_s),
                    float(cfg.phase_stance_timeout_s),
                )
            else:
                timing_score = 1.0
            if self._contact_support_stance_seen:
                mean_penetration_quality = (
                    self._contact_support_penetration_quality_sum
                    / max(1, self._contact_support_penetration_quality_samples)
                )
                to_score = float(timing_score * mean_penetration_quality)
            dense_confirmed = float(self._contact_support_pending_dense_reward)
            self._contact_support_pending_dense_reward = 0.0

        self._contact_support_last_valid_hs_count = valid_hs_count
        self._contact_support_last_valid_to_count = valid_to_count
        return {
            "contact_load_score": float(contact_load_score),
            "contact_load_dense_active": float(dense_active),
            "contact_load_evidence_complete": float(evidence_complete),
            "contact_support_to_score": float(to_score),
            "contact_support_to_timing_score": float(timing_score),
            "contact_support_stance_duration_s": float(stance_duration_s),
            "contact_support_min_penetration_quality": float(
                self._contact_support_min_penetration_quality
            ),
            "contact_support_mean_penetration_quality": float(
                self._contact_support_penetration_quality_sum
                / max(1, self._contact_support_penetration_quality_samples)
            ),
            "contact_support_pending_dense_reward": float(
                self._contact_support_pending_dense_reward
            ),
            "contact_support_dense_confirmed_reward": float(dense_confirmed),
            "contact_support_clawback_penalty": float(clawback),
        }

    def _task_reward_terms(self, info: Mapping[str, Any]) -> dict[str, float]:
        cfg = self.reward_config
        gait = info.get("online_gait")
        sides = gait.get("sides") if isinstance(gait, Mapping) else None
        left_gait = sides.get("left") if isinstance(sides, Mapping) else None
        if not isinstance(left_gait, Mapping):
            left_gait = {}
        online_grf = info.get("online_grf")
        grf_sides = online_grf.get("left") if isinstance(online_grf, Mapping) else None
        if not isinstance(grf_sides, Mapping):
            grf_sides = {}

        normal_bw = max(0.0, float(left_gait.get("normal_force_bw", 0.0) or 0.0))
        penetration_m = max(0.0, float(grf_sides.get("penetration", 0.0) or 0.0))
        in_contact = bool(left_gait.get("in_contact", False))
        phase = float(left_gait.get("gait_phase", 0.0) or 0.0)
        cycle_duration = float(left_gait.get("cycle_duration_s", 0.0) or 0.0)
        last_hs = left_gait.get("last_heel_strike_time")
        last_to = left_gait.get("last_toe_off_time")
        phase_known = last_hs is not None and cycle_duration > 1e-9
        swing_since_to = last_to is not None and (
            last_hs is None or float(last_to) > float(last_hs)
        )
        phase_fsm = info.get("phase_fsm")
        if not isinstance(phase_fsm, Mapping):
            phase_fsm = {}
        try:
            phase_fsm_state_id = int(float(phase_fsm.get("state_id", 0.0) or 0.0))
        except (TypeError, ValueError, OverflowError):
            phase_fsm_state_id = 0
        phase_fsm_state_name = str(phase_fsm.get("state_name", "") or "")
        fsm_expects_stance = (
            phase_fsm_state_id == 1 or phase_fsm_state_name == "STANCE_AFTER_HS"
        )
        fsm_expects_swing = (
            phase_fsm_state_id == 2 or phase_fsm_state_name == "SWING_AFTER_TO"
        )
        phase_expectation_source_id = 0.0
        if fsm_expects_stance:
            stance_expected = True
            swing_expected = False
            phase_expectation_source_id = 1.0
        elif fsm_expects_swing:
            stance_expected = False
            swing_expected = True
            phase_expectation_source_id = 1.0
        elif swing_since_to:
            stance_expected = False
            swing_expected = True
        elif phase_known:
            stance_expected = phase <= float(cfg.prosthetic_stance_phase_end)
            swing_expected = not stance_expected
        else:
            stance_expected = in_contact
            swing_expected = False

        load_target = max(1e-9, float(cfg.contact_load_target_bw))
        confidence_full_bw = float(cfg.contact_load_confidence_full_bw)
        if confidence_full_bw <= 0.0:
            confidence_full_bw = load_target
        pen_full = max(0.0, float(cfg.contact_load_penetration_full_reward_m))
        pen_zero = max(pen_full, float(cfg.contact_load_penetration_zero_reward_m))
        if pen_zero <= pen_full + 1e-12:
            contact_penetration_quality = 1.0 if penetration_m <= pen_zero else 0.0
        elif penetration_m <= pen_full:
            contact_penetration_quality = 1.0
        elif penetration_m >= pen_zero:
            contact_penetration_quality = 0.0
        else:
            contact_penetration_quality = float(
                (pen_zero - penetration_m) / (pen_zero - pen_full)
            )
        if stance_expected:
            contact_load_raw_score = float(
                np.clip(normal_bw / max(1e-9, confidence_full_bw), 0.0, 1.0)
            )
            candidate_contact_score = float(
                contact_load_raw_score * contact_penetration_quality
            )
        else:
            contact_load_raw_score = 0.0
            candidate_contact_score = 0.0
        contact_support_terms = self._contact_support_terms(
            info,
            stance_expected=stance_expected,
            candidate_contact_score=candidate_contact_score,
            penetration_quality=contact_penetration_quality,
        )
        contact_load_score = float(contact_support_terms["contact_load_score"])
        contact_load_loss = (
            float((1.0 - contact_load_score) ** 2) if stance_expected else 0.0
        )
        contact_overload_loss = _bounded_square_ratio(
            max(0.0, normal_bw - float(cfg.contact_load_max_bw)),
            max(load_target, float(cfg.contact_load_max_bw)),
        )
        swing_unloading_loss = (
            _bounded_square_ratio(normal_bw, cfg.swing_unloading_force_tol_bw)
            if swing_expected
            else 0.0
        )
        slip_speed = max(0.0, float(grf_sides.get("slip_speed", 0.0) or 0.0))
        grf_slip_loss = (
            _bounded_square_ratio(slip_speed, cfg.grf_slip_speed_scale_m_s)
            if normal_bw > 0.05
            else 0.0
        )

        phase_terms = self._phase_regular_terms(info)
        reserve_terms = self._reserve_terms(info)
        pelvis_terms = self._pelvis_terms(info)
        joint_terms = self._prosthetic_joint_terms(info)
        morphology_terms = self._morphology_terms(info)
        return {
            "contact_load_raw_score": contact_load_raw_score,
            "contact_load_loss": contact_load_loss,
            "contact_load_confidence_full_bw": confidence_full_bw,
            "contact_load_penetration_quality": contact_penetration_quality,
            "contact_load_penetration_m": penetration_m,
            "contact_load_penetration_full_reward_m": pen_full,
            "contact_load_penetration_zero_reward_m": pen_zero,
            "contact_overload_loss": contact_overload_loss,
            "swing_unloading_loss": swing_unloading_loss,
            "grf_slip_loss": grf_slip_loss,
            "prosthetic_stance_expected": float(stance_expected),
            "prosthetic_swing_expected": float(swing_expected),
            "prosthetic_phase_expectation_source_id": float(
                phase_expectation_source_id
            ),
            "prosthetic_normal_force_bw": float(normal_bw),
            "prosthetic_slip_speed_m_s": float(slip_speed),
            **contact_support_terms,
            **phase_terms,
            **reserve_terms,
            **pelvis_terms,
            **joint_terms,
            **morphology_terms,
        }

    def _phase_regular_terms(self, info: Mapping[str, Any]) -> dict[str, float]:
        cfg = self.reward_config
        fsm = info.get("phase_fsm")
        if not isinstance(fsm, Mapping):
            fsm = {}
        period_soft_low = float(cfg.phase_period_nominal_s) - float(
            cfg.phase_period_soft_margin_s
        )
        period_soft_high = float(cfg.phase_period_nominal_s) + float(
            cfg.phase_period_soft_margin_s
        )
        event_order_loss = float(fsm.get("invalid_event_loss", 0.0) or 0.0)
        last_period_s = float(fsm.get("last_period_s", 0.0) or 0.0)
        previous_period_s = float(fsm.get("previous_period_s", 0.0) or 0.0)
        last_stance_fraction = float(fsm.get("last_stance_fraction", 0.0) or 0.0)
        phase_period_loss = (
            _huber_interval_loss(
                last_period_s,
                period_soft_low,
                period_soft_high,
                cfg.phase_period_soft_margin_s,
            )
            if last_period_s > 1e-9
            else 0.0
        )
        phase_period_hard_loss = (
            _huber_interval_loss(
                last_period_s,
                cfg.phase_period_hard_min_s,
                cfg.phase_period_hard_max_s,
                cfg.phase_period_soft_margin_s,
            )
            if last_period_s > 1e-9
            else 0.0
        )
        phase_periodicity_loss = (
            _huber_loss(
                (last_period_s - previous_period_s)
                / max(1e-9, float(cfg.phase_periodicity_scale_s))
            )
            if last_period_s > 1e-9 and previous_period_s > 1e-9
            else 0.0
        )
        stance_fraction_loss = (
            _huber_interval_loss(
                last_stance_fraction,
                cfg.phase_stance_fraction_min,
                cfg.phase_stance_fraction_max,
                max(
                    1e-9,
                    0.5
                    * (
                        float(cfg.phase_stance_fraction_max)
                        - float(cfg.phase_stance_fraction_min)
                    ),
                ),
            )
            if last_stance_fraction > 1e-9
            else 0.0
        )
        stance_elapsed_s = float(fsm.get("stance_elapsed_s", 0.0) or 0.0)
        swing_elapsed_s = float(fsm.get("swing_elapsed_s", 0.0) or 0.0)
        stance_timeout_loss = _huber_interval_loss(
            stance_elapsed_s,
            0.0,
            cfg.phase_stance_timeout_s,
            cfg.phase_timeout_scale_s,
        )
        swing_timeout_loss = _huber_interval_loss(
            swing_elapsed_s,
            0.0,
            cfg.phase_swing_timeout_s,
            cfg.phase_timeout_scale_s,
        )
        phase_timeout_exceeded = float(fsm.get("timeout_exceeded", 0.0) or 0.0)
        phase_timeout_side = float(fsm.get("timeout_side", 0.0) or 0.0)
        timeout_loss = float(stance_timeout_loss + swing_timeout_loss)
        phase_regularity_loss = float(
            float(cfg.phase_event_order_weight) * event_order_loss
            + float(cfg.phase_period_weight)
            * (phase_period_loss + phase_period_hard_loss)
            + float(cfg.phase_periodicity_weight) * phase_periodicity_loss
            + float(cfg.phase_stance_fraction_weight) * stance_fraction_loss
            + float(cfg.phase_timeout_weight) * timeout_loss
        )
        valid_cycle_count = float(fsm.get("valid_cycle_count", 0.0) or 0.0)
        phase_event_progress_score = float(
            fsm.get("phase_event_progress_score", 0.0) or 0.0
        )
        phase_regular_score = (
            _score(phase_regularity_loss, cfg.phase_regularity_weight)
            if valid_cycle_count > 0
            else 0.0
        )
        return {
            "phase_regular_score": float(phase_regular_score),
            "phase_event_progress_score": float(phase_event_progress_score),
            "landing_window_contact_score": float(
                fsm.get("landing_window_contact_score", 0.0) or 0.0
            ),
            "landing_window_active": float(
                fsm.get("landing_window_active", 0.0) or 0.0
            ),
            "invalid_event_loss": float(fsm.get("invalid_event_loss", 0.0) or 0.0),
            "contact_validity_loss": float(
                fsm.get("contact_validity_loss", 0.0) or 0.0
            ),
            "invalid_event_count": float(fsm.get("invalid_event_count", 0.0) or 0.0),
            "phase_fsm_state_id": float(fsm.get("state_id", 0.0) or 0.0),
            "phase_regularity_loss": float(phase_regularity_loss),
            "phase_event_order_loss": float(event_order_loss),
            "phase_period_loss": float(phase_period_loss),
            "phase_period_hard_loss": float(phase_period_hard_loss),
            "phase_periodicity_loss": float(phase_periodicity_loss),
            "phase_stance_fraction_loss": float(stance_fraction_loss),
            "phase_timeout_loss": float(timeout_loss),
            "phase_stance_timeout_loss": float(stance_timeout_loss),
            "phase_swing_timeout_loss": float(swing_timeout_loss),
            "phase_stance_elapsed_s": float(stance_elapsed_s),
            "phase_swing_elapsed_s": float(swing_elapsed_s),
            "phase_stance_hard_timeout_s": float(cfg.phase_stance_hard_timeout_s),
            "phase_swing_hard_timeout_s": float(cfg.phase_swing_hard_timeout_s),
            "phase_timeout_exceeded": float(phase_timeout_exceeded),
            "phase_timeout_side": float(phase_timeout_side),
            "phase_valid_hs_count": float(fsm.get("valid_hs_count", 0.0) or 0.0),
            "phase_valid_to_count": float(fsm.get("valid_to_count", 0.0) or 0.0),
            "phase_valid_cycle_count": float(valid_cycle_count),
            "phase_cycle_progress_credit": float(
                fsm.get("cycle_progress_credit", 0.0) or 0.0
            ),
            "phase_pending_cycle_credit": float(
                fsm.get("pending_cycle_credit", 0.0) or 0.0
            ),
            "phase_cycle_complete_bonus": float(
                fsm.get("phase_cycle_complete_bonus", 0.0) or 0.0
            ),
            "phase_clawback_penalty": float(
                fsm.get("phase_clawback_penalty", 0.0) or 0.0
            ),
            "phase_failure_extra_penalty": float(
                fsm.get("phase_failure_extra_penalty", 0.0) or 0.0
            ),
            "phase_cycle_failed_this_step": float(
                fsm.get("phase_cycle_failed_this_step", 0.0) or 0.0
            ),
            "phase_last_period_s": float(last_period_s),
            "phase_previous_period_s": float(previous_period_s),
            "phase_last_stance_fraction": float(last_stance_fraction),
            "phase_robust_stance_duration_s": float(
                fsm.get("robust_stance_duration_s", 0.0) or 0.0
            ),
            "phase_robust_swing_duration_s": float(
                fsm.get("robust_swing_duration_s", 0.0) or 0.0
            ),
            "phase_duration_history_count": float(
                fsm.get("duration_history_count", 0.0) or 0.0
            ),
            "phase_stance_contact_time_s": float(
                fsm.get("stance_contact_time_s", 0.0) or 0.0
            ),
            "phase_stance_load_integral_bw_s": float(
                fsm.get("stance_load_integral_bw_s", 0.0) or 0.0
            ),
            "phase_stance_contact_fraction": float(
                fsm.get("stance_contact_fraction", 0.0) or 0.0
            ),
            "phase_stance_mean_load_bw": float(
                fsm.get("stance_mean_load_bw", 0.0) or 0.0
            ),
            "phase_cycle_knee_excursion_rad": float(
                fsm.get("cycle_knee_excursion_rad", 0.0) or 0.0
            ),
            "phase_cycle_ankle_excursion_rad": float(
                fsm.get("cycle_ankle_excursion_rad", 0.0) or 0.0
            ),
            "phase_cycle_rejected_this_step": float(
                fsm.get("cycle_rejected_this_step", 0.0) or 0.0
            ),
        }

    def _prosthetic_joint_terms(self, info: Mapping[str, Any]) -> dict[str, float]:
        obs = info.get("observation")
        if not isinstance(obs, Mapping):
            return {
                "prosthetic_joint_range_loss": 0.0,
                "prosthetic_joint_range_raw_loss": 0.0,
            }
        values = []
        for coord_name in ("pros_knee_angle", "pros_ankle_angle"):
            if coord_name in obs:
                values.append(float(obs.get(coord_name, 0.0) or 0.0))
        if not values:
            return {
                "prosthetic_joint_range_loss": 0.0,
                "prosthetic_joint_range_raw_loss": 0.0,
            }
        loss, raw_loss = _out_of_band_losses(
            np.asarray(values, dtype=float),
            self.reward_config,
            bounds=(
                self.reward_config.prosthetic_joint_q_min,
                self.reward_config.prosthetic_joint_q_max,
            ),
        )
        return {
            "prosthetic_joint_range_loss": float(loss),
            "prosthetic_joint_range_raw_loss": float(raw_loss),
            "prosthetic_knee_q_rad": float(values[0]) if len(values) > 0 else 0.0,
            "prosthetic_ankle_q_rad": float(values[1]) if len(values) > 1 else 0.0,
        }

    @staticmethod
    def _empty_morphology_terms() -> dict[str, float]:
        return {
            "morphology_available": 0.0,
            "morphology_phase": 0.0,
            "morphology_loss": 0.0,
            "morphology_knee_loss": 0.0,
            "morphology_ankle_loss": 0.0,
            "morphology_knee_raw_loss_rad2": 0.0,
            "morphology_ankle_raw_loss_rad2": 0.0,
            "morphology_knee_excursion_rad": 0.0,
            "morphology_ankle_excursion_rad": 0.0,
            "morphology_knee_value_rad": 0.0,
            "morphology_ankle_value_rad": 0.0,
            "morphology_knee_min_rad": 0.0,
            "morphology_knee_max_rad": 0.0,
            "morphology_ankle_min_rad": 0.0,
            "morphology_ankle_max_rad": 0.0,
            "fsm_morphology_phase": 0.0,
            "morphology_phase_source_id": 0.0,
            "morphology_phase_fsm_available": 0.0,
            "morphology_phase_mode_id": 0.0,
            "morphology_canonical_to_phase": 0.0,
            "morphology_loss_mean": 0.0,
            "morphology_inside_score": 0.0,
            "morphology_inside_fraction": 0.0,
            "morphology_settled_this_step": 0.0,
            "morphology_settled_sample_count": 0.0,
            "morphology_pending_sample_count": 0.0,
            "morphology_active_segment_id": 0.0,
            "morphology_discarded_segment_count": 0.0,
            "morphology_discarded_sample_count": 0.0,
            "morphology_ledger_overflow": 0.0,
            "morphology_ledger_nonmonotonic_sample": 0.0,
            "morphology_segment_duration_s": 0.0,
            "morphology_segment_type_id": 0.0,
            "morphology_hard_violation": 0.0,
            "morphology_hard_knee_excursion_rad": 0.0,
            "morphology_hard_ankle_excursion_rad": 0.0,
            "morphology_hard_max_excursion_rad": 0.0,
            "morphology_causal_delay_s": 0.0,
            "morphology_causal_terminal_flushed": 0.0,
            "morphology_causal_dropped_pending_count": 0.0,
            "morphology_causal_dropped_wait_hs_count": 0.0,
            "morphology_causal_cancelled_transition_count": 0.0,
            "morphology_causal_timeout_transition_count": 0.0,
            "morphology_causal_failed_closed": 0.0,
        }

    def _experimental_morphology_terms(
        self,
        info: Mapping[str, Any],
    ) -> dict[str, float]:
        """Settle exact event-warped losses for completed FSM segments.

        The Gym API cannot rewrite rewards already returned for earlier steps.
        Consequently ``morphology_loss`` is the *sum* of the completed
        segment's per-sample losses and is emitted sparsely on TO/HS.  The
        shipped experimental override keeps its weight at zero; positive-weight
        PPO use requires a separate complete-episode protocol and validation.
        """
        terms = self._empty_morphology_terms()
        terms["morphology_phase_mode_id"] = 3.0
        ledger = self._experimental_morphology_ledger
        profile = self._morphology_profile
        alpha = self._morphology_canonical_to_phase
        if ledger is None or profile is None or alpha is None:
            return terms

        obs = info.get("observation")
        if not isinstance(obs, Mapping):
            obs = {}
        try:
            time_s = float(info.get("time", float("nan")))
            knee_value = float(obs.get("pros_knee_angle_served_ref", float("nan")))
            ankle_value = float(obs.get("pros_ankle_angle_served_ref", float("nan")))
        except (TypeError, ValueError):
            time_s = knee_value = ankle_value = float("nan")

        fsm = info.get("phase_fsm")
        if not isinstance(fsm, Mapping):
            fsm = {}
        transitions = fsm.get("accepted_transitions_this_step")
        if not isinstance(transitions, (list, tuple)):
            transitions = ()
        raw_terms = info.get("reward_terms")
        if not isinstance(raw_terms, Mapping):
            raw_terms = {}
        episode_ended = bool(
            float(raw_terms.get("terminated", 0.0) or 0.0) > 0.0
            or float(raw_terms.get("truncated", 0.0) or 0.0) > 0.0
            or float(fsm.get("timeout_exceeded", 0.0) or 0.0) > 0.0
            or info.get("end_reason")
        )
        update = ledger.update(
            time_s=time_s,
            knee_rad=knee_value,
            ankle_rad=ankle_value,
            accepted_transitions=transitions,
            episode_ended=episode_ended,
        )
        self._morphology_ledger_diagnostics_payload = {
            "discard_reason": str(update.discard_reason),
            "discarded_segment_count": int(update.discarded_segment_count),
            "discarded_sample_count": int(update.discarded_sample_count),
            "overflowed": bool(update.overflowed),
            "nonmonotonic_sample": bool(update.nonmonotonic_sample),
            "pending_sample_count": int(update.pending_sample_count),
            "active_segment_type": str(update.active_segment_type),
            "active_segment_start_time_s": float(update.active_segment_start_time_s),
            "completed_segment_count": len(update.completed_segments),
        }

        active_id = {"": 0.0, "stance": 1.0, "swing": 2.0}.get(
            update.active_segment_type,
            0.0,
        )
        terms.update(
            {
                "morphology_canonical_to_phase": float(alpha),
                "morphology_phase_source_id": 5.0,
                "morphology_phase_fsm_available": float(
                    bool(update.active_segment_type)
                ),
                "morphology_pending_sample_count": float(update.pending_sample_count),
                "morphology_active_segment_id": float(active_id),
                "morphology_discarded_segment_count": float(
                    update.discarded_segment_count
                ),
                "morphology_discarded_sample_count": float(
                    update.discarded_sample_count
                ),
                "morphology_ledger_overflow": float(update.overflowed),
                "morphology_ledger_nonmonotonic_sample": float(
                    update.nonmonotonic_sample
                ),
            }
        )

        payloads: list[dict[str, Any]] = []
        knee_loss_sum = 0.0
        ankle_loss_sum = 0.0
        knee_raw_sum = 0.0
        ankle_raw_sum = 0.0
        knee_excursion_max = 0.0
        ankle_excursion_max = 0.0
        knee_hard_max = 0.0
        ankle_hard_max = 0.0
        inside_score_sum = 0.0
        sample_count = 0
        last_sample_payload: dict[str, float] | None = None
        last_segment_type = ""
        last_segment_duration_s = 0.0

        for segment in update.completed_segments:
            phases = segment.phases(alpha)
            segment_samples: list[dict[str, float]] = []
            for sample, phase in zip(segment.samples, phases):
                corridor = _morphology_corridor_at(
                    profile,
                    phase,
                    self.reward_config,
                )
                knee_corridor = corridor["pros_knee_angle"]
                ankle_corridor = corridor["pros_ankle_angle"]
                knee_loss, knee_raw, knee_excursion = _morphology_interval_losses(
                    sample.knee_rad,
                    knee_corridor["min_rad"],
                    knee_corridor["max_rad"],
                )
                ankle_loss, ankle_raw, ankle_excursion = _morphology_interval_losses(
                    sample.ankle_rad,
                    ankle_corridor["min_rad"],
                    ankle_corridor["max_rad"],
                )
                knee_hard, ankle_hard = _morphology_hard_excursions(
                    sample.knee_rad,
                    sample.ankle_rad,
                    self.reward_config,
                )
                inside_score = 0.5 * (
                    float(knee_excursion == 0.0) + float(ankle_excursion == 0.0)
                )
                item = {
                    "time_s": float(sample.time_s),
                    "phase": float(phase),
                    "knee_served_ref_rad": float(sample.knee_rad),
                    "ankle_served_ref_rad": float(sample.ankle_rad),
                    "knee_min_rad": float(knee_corridor["min_rad"]),
                    "knee_max_rad": float(knee_corridor["max_rad"]),
                    "ankle_min_rad": float(ankle_corridor["min_rad"]),
                    "ankle_max_rad": float(ankle_corridor["max_rad"]),
                    "knee_loss": float(knee_loss),
                    "ankle_loss": float(ankle_loss),
                    "knee_excursion_rad": float(knee_excursion),
                    "ankle_excursion_rad": float(ankle_excursion),
                    "inside_score": float(inside_score),
                    "knee_hard_excursion_rad": float(knee_hard),
                    "ankle_hard_excursion_rad": float(ankle_hard),
                }
                segment_samples.append(item)
                last_sample_payload = item
                sample_count += 1
                knee_loss_sum += knee_loss
                ankle_loss_sum += ankle_loss
                knee_raw_sum += knee_raw
                ankle_raw_sum += ankle_raw
                knee_excursion_max = max(knee_excursion_max, knee_excursion)
                ankle_excursion_max = max(ankle_excursion_max, ankle_excursion)
                knee_hard_max = max(knee_hard_max, knee_hard)
                ankle_hard_max = max(ankle_hard_max, ankle_hard)
                inside_score_sum += inside_score

            last_segment_type = segment.segment_type
            last_segment_duration_s = segment.duration_s
            payloads.append(
                {
                    "segment_type": segment.segment_type,
                    "start_time_s": float(segment.start_time_s),
                    "end_time_s": float(segment.end_time_s),
                    "duration_s": float(segment.duration_s),
                    "sample_count": len(segment_samples),
                    "samples": segment_samples,
                }
            )

        self._morphology_completed_segments_payload = payloads
        if sample_count <= 0:
            return terms

        total_loss_sum = 0.5 * (knee_loss_sum + ankle_loss_sum)
        last = last_sample_payload or {}
        hard_max = max(knee_hard_max, ankle_hard_max)
        segment_type_id = 1.0 if last_segment_type == "stance" else 2.0
        terms.update(
            {
                "morphology_available": 1.0,
                "morphology_phase": float(last.get("phase", 0.0)),
                "fsm_morphology_phase": float(last.get("phase", 0.0)),
                "morphology_loss": float(total_loss_sum),
                "morphology_loss_mean": float(total_loss_sum / sample_count),
                "morphology_knee_loss": float(knee_loss_sum),
                "morphology_ankle_loss": float(ankle_loss_sum),
                "morphology_knee_raw_loss_rad2": float(knee_raw_sum),
                "morphology_ankle_raw_loss_rad2": float(ankle_raw_sum),
                "morphology_knee_excursion_rad": float(knee_excursion_max),
                "morphology_ankle_excursion_rad": float(ankle_excursion_max),
                "morphology_knee_value_rad": float(
                    last.get("knee_served_ref_rad", 0.0)
                ),
                "morphology_ankle_value_rad": float(
                    last.get("ankle_served_ref_rad", 0.0)
                ),
                "morphology_knee_min_rad": float(last.get("knee_min_rad", 0.0)),
                "morphology_knee_max_rad": float(last.get("knee_max_rad", 0.0)),
                "morphology_ankle_min_rad": float(last.get("ankle_min_rad", 0.0)),
                "morphology_ankle_max_rad": float(last.get("ankle_max_rad", 0.0)),
                "morphology_inside_score": float(inside_score_sum / sample_count),
                "morphology_inside_fraction": float(inside_score_sum / sample_count),
                "morphology_settled_this_step": float(len(update.completed_segments)),
                "morphology_settled_sample_count": float(sample_count),
                "morphology_segment_duration_s": float(last_segment_duration_s),
                "morphology_segment_type_id": float(segment_type_id),
                "morphology_hard_violation": float(hard_max > 0.0),
                "morphology_hard_knee_excursion_rad": float(knee_hard_max),
                "morphology_hard_ankle_excursion_rad": float(ankle_hard_max),
                "morphology_hard_max_excursion_rad": float(hard_max),
            }
        )
        return terms

    def _causal_morphology_terms(
        self,
        info: Mapping[str, Any],
    ) -> dict[str, float]:
        """Settle fixed-delay morphology losses from exact V26 journals."""
        terms = self._empty_morphology_terms()
        terms["morphology_phase_mode_id"] = 4.0
        terms["morphology_phase_source_id"] = 6.0
        buffer = self._causal_morphology_buffer
        profile = self._morphology_profile
        alpha = self._morphology_canonical_to_phase
        if buffer is None or profile is None or alpha is None:
            return terms

        obs = info.get("observation")
        if not isinstance(obs, Mapping):
            obs = {}
        try:
            time_s = float(info.get("time", float("nan")))
            knee_value = float(obs.get("pros_knee_angle_served_ref", float("nan")))
            ankle_value = float(obs.get("pros_ankle_angle_served_ref", float("nan")))
        except (TypeError, ValueError):
            time_s = knee_value = ankle_value = float("nan")

        raw_terms = info.get("reward_terms")
        if not isinstance(raw_terms, Mapping):
            raw_terms = {}
        phase = info.get("phase_fsm")
        if not isinstance(phase, Mapping):
            phase = {}
        episode_ended = bool(
            float(raw_terms.get("terminated", 0.0) or 0.0) > 0.0
            or float(raw_terms.get("truncated", 0.0) or 0.0) > 0.0
            or float(phase.get("timeout_exceeded", 0.0) or 0.0) > 0.0
            or info.get("end_reason")
        )

        try:
            runtime = extract_v26_morphology_runtime(info)
        except (TypeError, ValueError) as exc:
            update = buffer.fail_closed(
                f"v26_runtime:{exc}",
                rejected_samples=1,
            )
            runtime = None
        else:
            history_count = float(phase.get("duration_history_count", 0.0) or 0.0)
            robust_stance = float(phase.get("robust_stance_duration_s", 0.0) or 0.0)
            robust_swing = float(phase.get("robust_swing_duration_s", 0.0) or 0.0)
            use_robust = bool(
                np.isfinite(history_count)
                and history_count > 0.0
                and np.isfinite(robust_stance)
                and robust_stance > 0.0
                and np.isfinite(robust_swing)
                and robust_swing > 0.0
            )
            update = buffer.update(
                time_s=time_s,
                knee_rad=knee_value,
                ankle_rad=ankle_value,
                accepted_transitions=runtime.accepted_transitions,
                confirmed_detector_transitions=runtime.detector_transitions,
                pending_transition=runtime.pending_transition,
                cancelled_transitions=runtime.cancelled_transitions,
                episode_ended=episode_ended,
                actor_state_name=runtime.actor_state_name,
                partial_stance_active=runtime.partial_stance_active,
                stance_duration_s=robust_stance if use_robust else None,
                swing_duration_s=robust_swing if use_robust else None,
            )

        self._morphology_causal_diagnostics_payload = {
            "event_contract_id": str(buffer.event_contract_id),
            "delay_s": float(buffer.delay_s),
            "failed_closed": bool(update.failed_closed),
            "failure_reason": str(update.failure_reason),
            "drop_reason": str(update.drop_reason),
            "dropped_sample_count": int(update.dropped_sample_count),
            "dropped_pending_sample_count": int(update.dropped_pending_sample_count),
            "dropped_wait_hs_sample_count": int(update.dropped_wait_hs_sample_count),
            "pending_sample_count": int(update.pending_sample_count),
            "resolved_sample_count": len(update.resolved_samples),
            "total_resolved_sample_count": int(update.total_resolved_sample_count),
            "total_dropped_sample_count": int(update.total_dropped_sample_count),
            "cancelled_transition_count": int(update.cancelled_transition_count),
            "total_cancelled_transition_count": int(
                update.total_cancelled_transition_count
            ),
            "timeout_transition_count": int(update.timeout_transition_count),
            "terminal_flushed": bool(update.terminal_flushed),
            "actor_state_name": (
                runtime.actor_state_name if runtime is not None else ""
            ),
            "partial_stance_active": bool(
                runtime.partial_stance_active if runtime is not None else False
            ),
        }
        terms.update(
            {
                "morphology_canonical_to_phase": float(alpha),
                "morphology_pending_sample_count": float(update.pending_sample_count),
                "morphology_discarded_sample_count": float(update.dropped_sample_count),
                "morphology_ledger_overflow": float(
                    update.failure_reason == "causal_buffer_overflow"
                ),
                "morphology_causal_delay_s": float(buffer.delay_s),
                "morphology_causal_terminal_flushed": float(update.terminal_flushed),
                "morphology_causal_dropped_pending_count": float(
                    update.dropped_pending_sample_count
                ),
                "morphology_causal_dropped_wait_hs_count": float(
                    update.dropped_wait_hs_sample_count
                ),
                "morphology_causal_cancelled_transition_count": float(
                    update.cancelled_transition_count
                ),
                "morphology_causal_timeout_transition_count": float(
                    update.timeout_transition_count
                ),
                "morphology_causal_failed_closed": float(update.failed_closed),
            }
        )

        payloads: list[dict[str, Any]] = []
        knee_loss_sum = 0.0
        ankle_loss_sum = 0.0
        knee_raw_sum = 0.0
        ankle_raw_sum = 0.0
        knee_excursion_max = 0.0
        ankle_excursion_max = 0.0
        knee_hard_max = 0.0
        ankle_hard_max = 0.0
        inside_score_sum = 0.0
        last_payload: dict[str, Any] | None = None

        for resolved in update.resolved_samples:
            corridor = _morphology_corridor_at(
                profile,
                resolved.phase,
                self.reward_config,
            )
            knee_corridor = corridor["pros_knee_angle"]
            ankle_corridor = corridor["pros_ankle_angle"]
            knee_loss, knee_raw, knee_excursion = _morphology_interval_losses(
                resolved.sample.knee_rad,
                knee_corridor["min_rad"],
                knee_corridor["max_rad"],
            )
            ankle_loss, ankle_raw, ankle_excursion = _morphology_interval_losses(
                resolved.sample.ankle_rad,
                ankle_corridor["min_rad"],
                ankle_corridor["max_rad"],
            )
            knee_hard, ankle_hard = _morphology_hard_excursions(
                resolved.sample.knee_rad,
                resolved.sample.ankle_rad,
                self.reward_config,
            )
            inside_score = 0.5 * (
                float(knee_excursion == 0.0) + float(ankle_excursion == 0.0)
            )
            item: dict[str, Any] = {
                "time_s": float(resolved.sample.time_s),
                "emitted_time_s": float(resolved.emitted_time_s),
                "delay_s": float(resolved.delay_s),
                "terminal_flush": bool(resolved.terminal_flush),
                "segment_type": str(resolved.segment_type),
                "segment_start_time_s": float(resolved.segment_start_time_s),
                "anchor_confirmed_time_s": float(resolved.anchor_confirmed_time_s),
                "anchor_delivered_time_s": float(resolved.anchor_delivered_time_s),
                "duration_basis_s": float(resolved.duration_basis_s),
                "phase": float(resolved.phase),
                "knee_served_ref_rad": float(resolved.sample.knee_rad),
                "ankle_served_ref_rad": float(resolved.sample.ankle_rad),
                "knee_min_rad": float(knee_corridor["min_rad"]),
                "knee_max_rad": float(knee_corridor["max_rad"]),
                "ankle_min_rad": float(ankle_corridor["min_rad"]),
                "ankle_max_rad": float(ankle_corridor["max_rad"]),
                "knee_loss": float(knee_loss),
                "ankle_loss": float(ankle_loss),
                "knee_excursion_rad": float(knee_excursion),
                "ankle_excursion_rad": float(ankle_excursion),
                "inside_score": float(inside_score),
                "knee_hard_excursion_rad": float(knee_hard),
                "ankle_hard_excursion_rad": float(ankle_hard),
            }
            payloads.append(item)
            last_payload = item
            knee_loss_sum += knee_loss
            ankle_loss_sum += ankle_loss
            knee_raw_sum += knee_raw
            ankle_raw_sum += ankle_raw
            knee_excursion_max = max(knee_excursion_max, knee_excursion)
            ankle_excursion_max = max(ankle_excursion_max, ankle_excursion)
            knee_hard_max = max(knee_hard_max, knee_hard)
            ankle_hard_max = max(ankle_hard_max, ankle_hard)
            inside_score_sum += inside_score

        self._morphology_causal_samples_payload = payloads
        sample_count = len(payloads)
        if sample_count <= 0:
            return terms

        total_loss = 0.5 * (knee_loss_sum + ankle_loss_sum)
        hard_max = max(knee_hard_max, ankle_hard_max)
        last = last_payload or {}
        segment_type_id = 1.0 if last.get("segment_type") == "stance" else 2.0
        terms.update(
            {
                "morphology_available": 1.0,
                "morphology_phase": float(last.get("phase", 0.0)),
                "fsm_morphology_phase": float(last.get("phase", 0.0)),
                "morphology_phase_fsm_available": 1.0,
                "morphology_loss": float(total_loss),
                "morphology_loss_mean": float(total_loss / sample_count),
                "morphology_knee_loss": float(knee_loss_sum),
                "morphology_ankle_loss": float(ankle_loss_sum),
                "morphology_knee_raw_loss_rad2": float(knee_raw_sum),
                "morphology_ankle_raw_loss_rad2": float(ankle_raw_sum),
                "morphology_knee_excursion_rad": float(knee_excursion_max),
                "morphology_ankle_excursion_rad": float(ankle_excursion_max),
                "morphology_knee_value_rad": float(
                    last.get("knee_served_ref_rad", 0.0)
                ),
                "morphology_ankle_value_rad": float(
                    last.get("ankle_served_ref_rad", 0.0)
                ),
                "morphology_knee_min_rad": float(last.get("knee_min_rad", 0.0)),
                "morphology_knee_max_rad": float(last.get("knee_max_rad", 0.0)),
                "morphology_ankle_min_rad": float(last.get("ankle_min_rad", 0.0)),
                "morphology_ankle_max_rad": float(last.get("ankle_max_rad", 0.0)),
                "morphology_inside_score": float(inside_score_sum / sample_count),
                "morphology_inside_fraction": float(inside_score_sum / sample_count),
                "morphology_settled_this_step": 1.0,
                "morphology_settled_sample_count": float(sample_count),
                "morphology_segment_duration_s": float(
                    last.get("duration_basis_s", 0.0)
                ),
                "morphology_segment_type_id": float(segment_type_id),
                "morphology_hard_violation": float(hard_max > 0.0),
                "morphology_hard_knee_excursion_rad": float(knee_hard_max),
                "morphology_hard_ankle_excursion_rad": float(ankle_hard_max),
                "morphology_hard_max_excursion_rad": float(hard_max),
            }
        )
        return terms

    def _morphology_terms(self, info: Mapping[str, Any]) -> dict[str, float]:
        profile = self._morphology_profile
        if profile is None:
            return self._empty_morphology_terms()

        phase_mode = self._morphology_phase_mode
        if phase_mode == EXPERIMENTAL_PHASE_MODE:
            return self._experimental_morphology_terms(info)
        if phase_mode == CAUSAL_DELAYED_PHASE_MODE:
            return self._causal_morphology_terms(info)
        if phase_mode == "event_anchored":
            canonical_to_phase = self._morphology_canonical_to_phase
            if canonical_to_phase is None:
                raise RuntimeError("event-anchored morphology TO phase was not loaded")
        else:
            try:
                canonical_to_phase = _morphology_canonical_to_phase(profile)
            except ValueError:
                canonical_to_phase = 0.0
        fsm_phase, fsm_available, phase_source_id = _fsm_morphology_phase(
            info,
            self.reward_config,
            canonical_to_phase=(
                canonical_to_phase if phase_mode == "event_anchored" else None
            ),
        )
        if "phase_fsm" in info:
            if fsm_phase is None:
                return self._empty_morphology_terms()
            phase = fsm_phase
        else:
            phase = None
            phase_source_id = 3.0
            fsm_available = 0.0

        if phase is None:
            gait = info.get("online_gait")
            sides = gait.get("sides") if isinstance(gait, Mapping) else None
            left_gait = sides.get("left") if isinstance(sides, Mapping) else None
            phase = (
                _normalize_morphology_phase(left_gait.get("gait_phase"))
                if isinstance(left_gait, Mapping)
                else None
            )
        if phase is None:
            return self._empty_morphology_terms()

        obs = info.get("observation")
        if not isinstance(obs, Mapping):
            return self._empty_morphology_terms()

        try:
            knee_value = float(obs["pros_knee_angle_served_ref"])
            ankle_value = float(obs["pros_ankle_angle_served_ref"])
        except (KeyError, TypeError, ValueError):
            return self._empty_morphology_terms()
        if not (np.isfinite(knee_value) and np.isfinite(ankle_value)):
            return self._empty_morphology_terms()

        corridor = _morphology_corridor_at(profile, phase, self.reward_config)
        knee_corridor = corridor["pros_knee_angle"]
        ankle_corridor = corridor["pros_ankle_angle"]
        knee_loss, knee_raw_loss, knee_excursion = _morphology_interval_losses(
            knee_value,
            knee_corridor["min_rad"],
            knee_corridor["max_rad"],
        )
        ankle_loss, ankle_raw_loss, ankle_excursion = _morphology_interval_losses(
            ankle_value,
            ankle_corridor["min_rad"],
            ankle_corridor["max_rad"],
        )
        return {
            "morphology_available": 1.0,
            "morphology_phase": float(phase),
            "fsm_morphology_phase": float(fsm_phase or 0.0),
            "morphology_phase_source_id": float(phase_source_id),
            "morphology_phase_fsm_available": float(fsm_available),
            "morphology_phase_mode_id": float(
                2.0 if phase_mode == "event_anchored" else 1.0
            ),
            "morphology_canonical_to_phase": float(canonical_to_phase),
            "morphology_loss": float(0.5 * (knee_loss + ankle_loss)),
            "morphology_knee_loss": float(knee_loss),
            "morphology_ankle_loss": float(ankle_loss),
            "morphology_knee_raw_loss_rad2": float(knee_raw_loss),
            "morphology_ankle_raw_loss_rad2": float(ankle_raw_loss),
            "morphology_knee_excursion_rad": float(knee_excursion),
            "morphology_ankle_excursion_rad": float(ankle_excursion),
            "morphology_knee_value_rad": float(knee_value),
            "morphology_ankle_value_rad": float(ankle_value),
            "morphology_knee_min_rad": float(knee_corridor["min_rad"]),
            "morphology_knee_max_rad": float(knee_corridor["max_rad"]),
            "morphology_ankle_min_rad": float(ankle_corridor["min_rad"]),
            "morphology_ankle_max_rad": float(ankle_corridor["max_rad"]),
        }

    def _reserve_terms(self, info: Mapping[str, Any]) -> dict[str, float]:
        diag = info.get("so_diagnostics")
        if not isinstance(diag, Mapping):
            diag = {}
        reserve_norm = float(diag.get("tau_reserve_norm", 0.0) or 0.0)
        residual_norm = float(diag.get("residual_norm", 0.0) or 0.0)
        reserve_loss = _bounded_square_ratio(
            reserve_norm,
            self.reward_config.reserve_norm_scale_nm,
        )
        residual_loss = _bounded_square_ratio(
            residual_norm,
            self.reward_config.residual_norm_scale_nm,
        )
        return {
            "reserve_norm_nm": float(reserve_norm),
            "residual_norm_nm": float(residual_norm),
            "reserve_norm_loss": float(reserve_loss),
            "residual_norm_loss": float(residual_loss),
            "reserve_residual_loss": float(0.5 * (reserve_loss + residual_loss)),
        }

    def _pelvis_terms(self, info: Mapping[str, Any]) -> dict[str, float]:
        obs = info.get("observation")
        pelvis_ty = None
        if isinstance(obs, Mapping) and "pelvis_ty" in obs:
            pelvis_ty = float(obs.get("pelvis_ty", 0.0) or 0.0)
        if pelvis_ty is None:
            return {"pelvis_height_loss": 0.0}
        deficit = max(0.0, float(self.reward_config.pelvis_height_min_m) - pelvis_ty)
        return {
            "pelvis_ty_m": float(pelvis_ty),
            "pelvis_height_loss": _bounded_square_ratio(
                deficit,
                self.reward_config.pelvis_height_scale_m,
            ),
        }

    def step(self, action):
        action_clip_terms = self._action_clip_terms(action)
        obs, env_reward, terminated, truncated, info = self.env.step(action)
        terms = info.get("reward_terms") if isinstance(info, dict) else None
        if not terms:
            return obs, env_reward, terminated, truncated, info

        terms = dict(terms)
        terms.update(action_clip_terms)
        terms.update(self._task_reward_terms(info))
        reference = info.get("policy_segment_values")
        reward, components = compute_reward(
            terms, self.reward_config, reference=reference
        )

        info = dict(info)
        if float(terms.get("phase_timeout_exceeded", 0.0) or 0.0) > 0.0:
            side_code = float(terms.get("phase_timeout_side", 0.0) or 0.0)
            side_name = "stance" if side_code == 1.0 else "swing"
            current_reason = info.get("end_reason")
            if not terminated or current_reason in {
                None,
                "episode_time_limit",
                "dataset_end",
            }:
                terminated = True
                truncated = False
                info["end_reason"] = f"phase_timeout:{side_name}"
                terms["terminated"] = 1.0
                terms["truncated"] = 0.0
        if (
            self._morphology_phase_mode == CAUSAL_DELAYED_PHASE_MODE
            and float(self.reward_config.morphology_weight) > 0.0
            and float(terms.get("morphology_causal_failed_closed", 0.0) or 0.0) > 0.0
        ):
            terminated = True
            truncated = False
            info["end_reason"] = "morphology_causal_contract_failure"
            terms["terminated"] = 1.0
            terms["truncated"] = 0.0
        if (
            float(self.reward_config.morphology_hard_termination_enabled) > 0.0
            and float(terms.get("morphology_hard_violation", 0.0) or 0.0) > 0.0
        ):
            current_reason = info.get("end_reason")
            if current_reason in {
                None,
                "episode_time_limit",
                "dataset_end",
            }:
                terminated = True
                truncated = False
                info["end_reason"] = "morphology_hard_violation:completed_segment"
                terms["terminated"] = 1.0
                terms["truncated"] = 0.0
        info["reward_terms"] = terms
        info["reward_components"] = components
        info["reward_env_original"] = float(env_reward)
        if self._morphology_phase_mode == EXPERIMENTAL_PHASE_MODE:
            info["morphology_completed_segments"] = [
                dict(item) for item in self._morphology_completed_segments_payload
            ]
            info["morphology_ledger_diagnostics"] = dict(
                self._morphology_ledger_diagnostics_payload
            )
        elif self._morphology_phase_mode == CAUSAL_DELAYED_PHASE_MODE:
            info["morphology_causal_samples"] = [
                dict(item) for item in self._morphology_causal_samples_payload
            ]
            info["morphology_causal_diagnostics"] = dict(
                self._morphology_causal_diagnostics_payload
            )
        log = dict(info.get("log") or {})
        for key, value in components.items():
            log[f"RewardShaped/{key}"] = float(value)
        info["log"] = log
        return obs, float(reward), terminated, truncated, info
