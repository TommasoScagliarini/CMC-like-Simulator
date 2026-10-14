"""Streaming kinematic TNT diagnostic: original and explicitly modified rate laws.

No force/contact input, future samples, neural policy, or online target fitting.
The stance timer/template are a separate extension to the paper's swing method.
"""
from __future__ import annotations

from dataclasses import asdict
import json
from pathlib import Path
import sys

import numpy as np
from scipy.interpolate import PchipInterpolator

HERE = Path(__file__).resolve().parent
PREVIOUS = HERE.parent / "tnt_preliminary_2026_10_13"
sys.path.insert(0, str(PREVIOUS))
from tnt_replay import Parameters, TNT, STANCE, INVALID


class RateEstimator(TNT):
    def __init__(self, parameters, *, method="literal", alpha=.25):
        super().__init__(parameters)
        if method not in {"literal", "blend", "duration"}:
            raise ValueError(method)
        if not 0 < alpha <= 1:
            raise ValueError("alpha must be in (0,1]")
        self.method, self.alpha = method, alpha
        self.swing_start = None
        self.raw_rate = self.rate

    def step(self, t, theta, dtheta, dt):
        previous_rate = self.rate
        event = super().step(t, theta, dtheta, dt)
        if event == "stance_to_early":
            self.swing_start = t
        if event == "middle_to_late":
            self.raw_rate = self.rate
            if self.method == "blend":
                # Explicit modification, not Equation 7 of the original paper.
                self.rate = (1 - self.alpha) * previous_rate + self.alpha * self.raw_rate
            elif self.method == "duration":
                # Alternative diagnostic, also not the original Equation 7.
                self.rate = self.p.t_em / (t - self.swing_start)
        return event


class StreamingPhase:
    def __init__(self, parameters, *, initial_time, initial_thigh_deg,
                 initial_stance_duration, method="literal", alpha=.25,
                 cutoff_hz=6.0, pfd_duration=None):
        self.estimator = RateEstimator(parameters, method=method, alpha=alpha)
        self.t = float(initial_time)
        self.theta = float(initial_thigh_deg)
        self.stance_start = self.t
        self.stance_duration = float(initial_stance_duration)
        self.cutoff_hz = cutoff_hz
        self.pfd_duration = pfd_duration
        self.pfd_start = None
        self.last = {}

    def step(self, t, theta):
        t, theta = float(t), float(theta)
        dt = t - self.t
        if dt < -1e-9 or not np.isfinite([t, theta]).all():
            raise ValueError("Nonfinite input or reversed time")
        dt = max(0., dt)
        a = np.exp(-2 * np.pi * self.cutoff_hz * dt)
        filtered = a * self.theta + (1 - a) * theta
        velocity = (filtered - self.theta) / dt if dt > 0 else 0.
        est = self.estimator
        if self.pfd_duration is None:
            event = est.step(t, filtered, velocity, dt)
        else:
            event = ""
            if est.state == STANCE and filtered < est.p.sigma_deg:
                est.state, est.T = 1, 0.
                est.swing_id += 1
                self.pfd_start = t
                event = "stance_to_early"
            elif est.state != STANCE:
                est.T = (t - self.pfd_start) / self.pfd_duration
                if est.T >= 1:
                    est.state, est.T = STANCE, 0.
                    est.completed += 1
                    event = "late_to_stance"
        if event == "stance_to_early" and est.completed > 0:
            self.stance_duration = max(t - self.stance_start, .01)
        if event == "late_to_stance":
            self.stance_start = t
        self.t, self.theta = t, filtered
        self.last = {
            "time": t, "theta": filtered, "dtheta": velocity, "state": est.state,
            "T": float(est.T), "rate": float(est.rate), "raw_rate": float(est.raw_rate),
            "stance_progress": float(np.clip((t - self.stance_start) / self.stance_duration, 0, 1)),
            "stance_duration": self.stance_duration, "cycle": est.swing_id,
            "completed": est.completed, "event": event,
        }
        return self.last


def stream_replay(t, theta, parameters, *, method="literal", alpha=.25,
                  initial_stance_duration=.6, pfd_duration=None):
    phase = StreamingPhase(parameters, initial_time=t[0], initial_thigh_deg=theta[0],
                           initial_stance_duration=initial_stance_duration,
                           method=method, alpha=alpha, pfd_duration=pfd_duration)
    records = [phase.step(now, angle) for now, angle in zip(t, theta)]
    keys = ["theta", "dtheta", "state", "T", "rate", "raw_rate", "stance_progress",
            "stance_duration", "cycle", "completed"]
    out = {key: np.array([r[key] for r in records]) for key in keys}
    out["t"] = np.asarray(t)
    out["events"] = [{"i": i, "t": r["time"], "event": r["event"],
                      "swing_id": r["cycle"], "T": r["T"]}
                     for i, r in enumerate(records) if r["event"]]
    return out


class KinematicTNTController:
    """Two absolute-angle references [knee,ankle] in degrees from a frozen bundle.

    The caller provides the actual measured thigh. The caller also preserves
    its existing target slew/governor/controller; those do not belong to TNT.
    INVALID raises before emitting a command, with no unreported fallback.
    """
    def __init__(self, bundle_json, initial_time, initial_thigh_deg):
        bundle = (json.loads(Path(bundle_json).read_text())
                  if isinstance(bundle_json, (str, Path)) else bundle_json)
        self.bundle = bundle
        self.phase = StreamingPhase(
            Parameters(**bundle["parameters"]), initial_time=initial_time,
            initial_thigh_deg=initial_thigh_deg,
            initial_stance_duration=bundle["initial_stance_duration_s"],
            method=bundle["rate_method"], alpha=bundle["rate_alpha"],
            cutoff_hz=bundle["filter_cutoff_hz"])
        template = bundle["template"]
        self.curves = {state: PchipInterpolator(template["grid"], np.asarray(template[state]), axis=0)
                       for state in ["stance", "swing"]}
        self.last = {}

    def step(self, t, theta_deg):
        self.last = self.phase.step(t, theta_deg)
        if self.last["state"] == INVALID:
            raise RuntimeError("TNT estimator INVALID; no diagnostic reference emitted")
        if self.last["state"] == STANCE:
            return self.curves["stance"](self.last["stance_progress"])
        return self.curves["swing"](np.clip(self.last["T"], 0, 1))
