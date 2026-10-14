"""Prescribed phase templates for LEFT and the historical RIGHT V26 target.

The V26 template algorithm is reused with identity coordinate mapping. This
clock supplies supervision and the maskable sin/cos actor slots. The shared
simulator's right-leg gait clock remains disabled.
"""
from __future__ import annotations

import numpy as np

from unified_MLP_39._bootstrap import ensure_sim_paths

ensure_sim_paths()
from baseline_MLP import win_runtime  # noqa: E402,F401 (Torch before OpenSim)
from osim_trj_cmc_like import GaitPhaseClock, PhaseBasedImitationTarget  # noqa: E402


COORDINATES = ("pros_knee_angle", "pros_ankle_angle")
TARGET_SOURCE = "prescribed_left_phase_template"
SUPERVISION_CONTRACT = {
    "source": TARGET_SOURCE,
    "clock": "prescribed_left_grf_heel_strikes",
    "phase_origin": "left_heel_strike",
    "phase_shifts": dict.fromkeys(COORDINATES, 0.0),
    "template": "V26_phase_resampled_cycle_mean_periodic_cubic_spline",
    "velocity": "template_phase_derivative_divided_by_local_prescribed_period",
    "clock_actor_observation": True,
    "clock_actor_mask": "same episode mask as the four prescribed q/qdot inputs",
}

RIGHT_TARGET_SOURCE = "prescribed_right_phase_template"
RIGHT_PHASE_SHIFTS = {"pros_knee_angle": 0.465, "pros_ankle_angle": 0.452}
RIGHT_SUPERVISION_CONTRACT = {
    "source": RIGHT_TARGET_SOURCE,
    "clock": "prescribed_right_grf_heel_strikes",
    "phase_origin": "right_heel_strike",
    "phase_shifts": RIGHT_PHASE_SHIFTS,
    "template": "V26_phase_resampled_cycle_mean_periodic_cubic_spline",
    "velocity": "template_phase_derivative_divided_by_local_prescribed_period",
    "clock_actor_observation": True,
    "clock_actor_phase": "unshifted_right_heel_strike_phase",
    "clock_actor_mask": "same episode mask as the four prescribed q/qdot inputs",
}


class PrescribedLeftPhaseTarget:
    def __init__(self, base_kin, heel_strikes, *, phase_samples=200, time_window=None):
        self.time_bounds = tuple(map(float, base_kin.time_bounds))
        strikes = np.asarray(heel_strikes, dtype=float)
        if (strikes.ndim != 1 or len(strikes) < 2
                or not np.all(np.isfinite(strikes)) or np.any(np.diff(strikes) <= 0)):
            raise ValueError("prescribed-left phase teacher requires increasing finite left heel strikes")
        self.clock = GaitPhaseClock(strikes, phase_offset=0.0)
        self.template = PhaseBasedImitationTarget(
            base_kin, self.clock, dict(zip(COORDINATES, COORDINATES)),
            dict.fromkeys(COORDINATES, 0.0), fallback_phase_shift=0.0,
            phase_samples=int(phase_samples), time_window=time_window,
        )
        summary = self.template.summary()
        if not self.template.available or set(summary["coordinates"]) != set(COORDINATES):
            raise ValueError("prescribed-left phase teacher requires complete finite left-coordinate cycles")
        self.phase_samples = max(16, int(phase_samples))
        self.time_window = None if time_window is None else list(map(float, time_window))

    def get(self, t):
        if isinstance(t, (bool, np.bool_)):
            raise ValueError("prescribed-left target time must be numeric")
        t = float(t)
        lower, upper = self.time_bounds
        if not np.isfinite(t) or t < lower - 1e-9 or t > upper + 1e-9:
            raise ValueError("prescribed-left target time is outside the finite IK domain")
        q, qdot, phases = self.template.get(t)
        if any(not np.isfinite(values[name]) for values in (q, qdot, phases) for name in COORDINATES):
            raise ValueError("prescribed-left phase teacher produced a nonfinite target")
        return q, qdot, phases

    def summary(self):
        return {
            **self.template.summary(), **SUPERVISION_CONTRACT,
            "phase_samples": self.phase_samples,
            "template_time_window": self.time_window,
            "heel_strike_times_s": self.clock.heel_strike_times.tolist(),
            "clock_edge_behavior": "nearest prescribed period extrapolation within IK domain",
        }


class PrescribedRightV26PhaseTarget:
    """Historical V26 supervision, with an unshifted RIGHT actor clock.

    The calibrated shifts apply to the two target splines, not to the clock
    exposed to the actor. Reusing the knee target phase as the actor clock
    would silently shift its sin/cos channels by 0.465 cycles.
    """

    def __init__(self, base_kin, heel_strikes, *, sound_coordinates,
                 phase_samples=200, time_window=None):
        self.time_bounds = tuple(map(float, base_kin.time_bounds))
        strikes = np.asarray(heel_strikes, dtype=float)
        if (strikes.ndim != 1 or len(strikes) < 2
                or not np.all(np.isfinite(strikes)) or np.any(np.diff(strikes) <= 0)):
            raise ValueError("prescribed-right V26 target requires increasing finite right heel strikes")
        self.sound_coordinates = dict(sound_coordinates)
        if set(self.sound_coordinates) != set(COORDINATES):
            raise ValueError("prescribed-right V26 target requires both prosthetic coordinate mappings")
        self.clock = GaitPhaseClock(strikes, phase_offset=0.0)
        self.template = PhaseBasedImitationTarget(
            base_kin, self.clock, self.sound_coordinates, RIGHT_PHASE_SHIFTS,
            fallback_phase_shift=0.5, phase_samples=int(phase_samples),
            time_window=time_window,
        )
        summary = self.template.summary()
        if not self.template.available or set(summary["coordinates"]) != set(COORDINATES):
            raise ValueError("prescribed-right V26 target requires complete finite right-coordinate cycles")
        self.phase_samples = max(16, int(phase_samples))
        self.time_window = None if time_window is None else list(map(float, time_window))

    def get(self, t):
        if isinstance(t, (bool, np.bool_)):
            raise ValueError("prescribed-right V26 target time must be numeric")
        t = float(t)
        lower, upper = self.time_bounds
        if not np.isfinite(t) or t < lower - 1e-9 or t > upper + 1e-9:
            raise ValueError("prescribed-right V26 target time is outside the finite IK domain")
        q, qdot, phases = self.template.get(t)
        if any(not np.isfinite(values[name]) for values in (q, qdot, phases) for name in COORDINATES):
            raise ValueError("prescribed-right V26 target produced a nonfinite target")
        return q, qdot, phases

    def summary(self):
        return {
            **self.template.summary(), **RIGHT_SUPERVISION_CONTRACT,
            "phase_shifts": dict(RIGHT_PHASE_SHIFTS),
            "source_coordinates": dict(self.sound_coordinates),
            "phase_samples": self.phase_samples,
            "template_time_window": self.time_window,
            "heel_strike_times_s": self.clock.heel_strike_times.tolist(),
            "clock_edge_behavior": "nearest prescribed period extrapolation within IK domain",
        }
