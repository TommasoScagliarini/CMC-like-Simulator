"""Shared physical reward diagnostics for online and saved-trajectory tuning.

Jerk is the derivative of interval-average acceleration inferred from actual
joint velocities. It is sampled at policy boundaries, unlike the substep knee
torque-rate diagnostic. Callers must preserve this distinction in reports.
"""
from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
import math
from typing import Any


TUNING_PREFIXES = (
    "ankle_positive_torque", "knee_torque_rate",
    "knee_kinematic_jerk", "ankle_kinematic_jerk",
)
_SCALES = (
    "ankle_positive_torque_scale_nm", "knee_torque_rate_scale_nm_s",
    "kinematic_jerk_scale_rad_s3", "kinematic_jerk_scale_rad_s3",
)


def _get(config: Any, key: str, default: Any = None) -> Any:
    return config.get(key, default) if isinstance(config, Mapping) else getattr(config, key, default)


def _finite(value: Any, name: str, *, nonnegative: bool = False) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number")
    try:
        result = float(value)
    except (ValueError, TypeError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(result) or (nonnegative and result < 0.0):
        raise ValueError(f"{name} must be finite" + (" and non-negative" if nonnegative else ""))
    return result


def tuning_weights(config: Any) -> dict[str, float]:
    """Validate opt-in weights; all tuning parameters are inert in ex-novo."""
    if _get(config, "reward_mode", "ex_novo") != "imitation":
        return dict.fromkeys(TUNING_PREFIXES, 0.0)
    result = {}
    for prefix, scale_name in zip(TUNING_PREFIXES, _SCALES):
        weight = _finite(_get(config, prefix + "_weight", 0.0), prefix + "_weight", nonnegative=True)
        if weight:
            scale = _finite(_get(config, scale_name), scale_name)
            if scale <= 0.0:
                raise ValueError(f"{scale_name} must be positive")
        result[prefix] = weight
    return result


@dataclass
class TuningHistory:
    """Per-episode velocity/acceleration history, optionally primed at reset.

    ``kinematics`` uses trace ``prosthetic_state``/online ``observation`` keys:
    ``pros_knee_angle_vel`` and ``pros_ankle_angle_vel``. Without a reset sample,
    two completed rows warm up before jerk is available; with one, only one.
    """

    _samples: dict[str, tuple[float, float]] = field(default_factory=dict)
    _accelerations: dict[str, tuple[float, float]] = field(default_factory=dict)

    def reset(self, time_s: float | None = None, kinematics: Mapping[str, Any] | None = None) -> None:
        self._samples.clear()
        self._accelerations.clear()
        if time_s is None or not isinstance(kinematics, Mapping):
            return
        time = _finite(time_s, "time_s")
        for joint in ("knee", "ankle"):
            key = f"pros_{joint}_angle_vel"
            if key in kinematics:
                self._samples[joint] = (time, _finite(kinematics[key], key))

    def update(self, time_s: Any, kinematics: Mapping[str, Any], *, required: set[str]) -> dict[str, tuple[float, bool]]:
        time = _finite(time_s, "time_s")
        result = {}
        # Validate the entire observation before advancing either joint.
        velocities = {}
        for joint in ("knee", "ankle"):
            key = f"pros_{joint}_angle_vel"
            try:
                velocities[joint] = _finite(kinematics[key], key)
                previous = self._samples.get(joint)
                if previous is not None and time <= previous[0]:
                    raise ValueError("kinematic jerk requires strictly increasing time_s")
            except (KeyError, ValueError) as exc:
                if joint in required:
                    raise ValueError(f"required {joint} kinematic jerk input unavailable: {exc}") from exc
                velocities.pop(joint, None)
        for joint in ("knee", "ankle"):
            if joint not in velocities:
                self._samples.pop(joint, None)
                self._accelerations.pop(joint, None)
                result[joint] = (0.0, False)
                continue
            velocity = velocities[joint]
            previous = self._samples.get(joint)
            acceleration_previous = self._accelerations.get(joint)
            jerk, available = 0.0, False
            if previous is not None:
                acceleration = (velocity - previous[1]) / (time - previous[0])
                midpoint = (time + previous[0]) * 0.5
                if acceleration_previous is not None:
                    jerk = (acceleration - acceleration_previous[1]) / (midpoint - acceleration_previous[0])
                    jerk = _finite(jerk, f"{joint}_kinematic_jerk_rad_s3")
                    available = True
                self._accelerations[joint] = (midpoint, acceleration)
            self._samples[joint] = (time, velocity)
            result[joint] = (jerk, available)
        return result


def enrich_tuning_terms(
    terms: Mapping[str, Any], config: Any, *, time_s: float | None = None,
    kinematics: Mapping[str, Any] | None = None,
    history: TuningHistory | None = None, segment_completed: bool = True,
) -> dict[str, float]:
    """Return new losses/diagnostics to merge into existing reward terms.

    Torque inputs use the env's flattened ``pros_*_angle_sea_*`` names. Ankle
    positive mean is mean(max(tau_spring, 0)) over *all* segment samples. The
    knee rate input is its existing substep RMS. Enabled missing inputs raise;
    incomplete segments yield explicit unavailable terms and clear history.
    """
    weights = tuning_weights(config)
    result = {prefix + suffix: 0.0 for prefix in TUNING_PREFIXES for suffix in ("_loss", "_available")}
    result.update(knee_kinematic_jerk_warmup=0.0, ankle_kinematic_jerk_warmup=0.0)
    if _get(config, "reward_mode", "ex_novo") != "imitation":
        return result
    if not segment_completed:
        if history is not None:
            history.reset()
        return result

    specifications = (
        ("ankle_positive_torque", "pros_ankle_angle_sea_tau_spring_positive_mean_nm", "ankle_positive_torque_mean_nm", _SCALES[0], 30.0, False),
        ("knee_torque_rate", "pros_knee_angle_sea_tau_spring_rate_rms_nm_s", "knee_torque_rate_rms_nm_s", _SCALES[1], 1000.0, True),
    )
    for prefix, source, raw_key, scale_key, default_scale, squared in specifications:
        try:
            raw = _finite(terms[source], source, nonnegative=True)
            scale = _finite(_get(config, scale_key, default_scale), scale_key)
            if scale <= 0.0:
                raise ValueError(f"{scale_key} must be positive")
            loss = raw / scale
            if squared:
                loss *= loss
            loss = _finite(loss, prefix + "_loss", nonnegative=True)
        except (KeyError, ValueError) as exc:
            if weights[prefix]:
                raise ValueError(f"required {prefix} input unavailable: {exc}") from exc
            continue
        result.update({raw_key: raw, prefix + "_loss": loss, prefix + "_available": 1.0})

    required = {joint for joint in ("knee", "ankle") if weights[joint + "_kinematic_jerk"]}
    if history is None or time_s is None or not isinstance(kinematics, Mapping):
        if required:
            raise ValueError("kinematic jerk requires history, time_s and actual velocity kinematics")
        return result
    try:
        jerk_values = history.update(time_s, kinematics, required=required)
    except ValueError:
        if required:
            raise
        history.reset()
        return result
    for joint, (jerk, available) in jerk_values.items():
        prefix = joint + "_kinematic_jerk"
        result[prefix + "_warmup"] = float(not available and joint in history._samples)
        if not available:
            continue
        scale = _finite(_get(config, "kinematic_jerk_scale_rad_s3", 1000.0), "kinematic_jerk_scale_rad_s3")
        if scale <= 0.0:
            if weights[prefix]:
                raise ValueError("kinematic_jerk_scale_rad_s3 must be positive")
            continue
        ratio = jerk / scale
        loss = _finite(ratio * ratio, prefix + "_loss", nonnegative=True)
        result.update({prefix + "_rad_s3": jerk, prefix + "_loss": loss, prefix + "_available": 1.0})
    return result


def tuning_penalties(terms: Mapping[str, Any], config: Any) -> dict[str, float]:
    """Validate enabled losses and return their weighted, post-clip costs."""
    result = {}
    for prefix, weight in tuning_weights(config).items():
        if not weight:
            result[prefix + "_term"] = 0.0
            continue
        key = prefix + "_loss"
        if key not in terms:
            raise ValueError(f"required enabled reward term missing: {key}")
        result[prefix + "_term"] = _finite(weight * _finite(terms[key], key, nonnegative=True), prefix + "_term", nonnegative=True)
    return result
