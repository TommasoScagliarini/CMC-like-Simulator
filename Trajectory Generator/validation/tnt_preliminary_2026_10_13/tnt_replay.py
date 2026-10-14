"""Training-free, kinematic-only TNT replay; deliberately not a runtime controller.

Run extract_prescribed.py first, then this file with numpy/scipy/matplotlib.
All parameter selection and curve construction use the first 40 seconds only.
The held-out target joints are used exclusively for scoring and plotting.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.interpolate import PchipInterpolator
from scipy.signal import find_peaks

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
STANCE, EARLY, MIDDLE, LATE, INVALID = 0, 1, 2, 3, -1
STATE_NAMES = {0: "stance", 1: "early", 2: "middle", 3: "late", -1: "invalid"}


def json_safe(value):
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [json_safe(v) for v in value]
    if isinstance(value, np.generic):
        return json_safe(value.item())
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def write_json(path, value):
    path.write_text(json.dumps(json_safe(value), indent=2, allow_nan=False) + "\n")


@dataclass(frozen=True)
class Parameters:
    sigma_deg: float
    rho_deg: float
    t_em: float
    initial_rate: float


def inverse_quintic(y: float) -> float:
    """Bisection inverse of 10*x**3-15*x**4+6*x**5 on [0,1]."""
    if y <= 0.0:
        return 0.0
    if y >= 1.0:
        return 1.0
    lo, hi = 0.0, 1.0
    for _ in range(25):
        x = (lo + hi) * 0.5
        if x**3 * (10.0 + x * (-15.0 + 6.0 * x)) < y:
            lo = x
        else:
            hi = x
    return (lo + hi) * 0.5


class TNT:
    """Literal four-state estimator, no force input or future samples.

    One transition per sample. At late->stance, output becomes T=0 immediately.
    No monotonic ratchet, event detector, reset-on-stall, or timeout fallback.
    Invalid T_bm/rate is exposed as INVALID and never silently repaired.
    """
    def __init__(self, p: Parameters):
        if p.rho_deg <= 0 or not 0 < p.t_em < 1 or p.initial_rate <= 0:
            raise ValueError("Invalid TNT parameters")
        self.p = p
        self.state = STANCE
        self.T = 0.0
        self.rate = p.initial_rate
        self.theta_bm = 0.0
        self.T_bm = 0.0
        self.middle_start = 0.0
        self.entered = None
        self.swing_id = 0
        self.completed = 0

    def step(self, t: float, theta: float, dtheta: float, dt: float):
        if self.entered is None:
            self.entered = t
        transition = ""
        if self.state == STANCE:
            self.T = 0.0
            if theta < self.p.sigma_deg:
                self.state, self.entered = EARLY, t
                self.swing_id += 1
                transition = "stance_to_early"
        elif self.state == EARLY:
            self.T += self.rate * dt
            if dtheta > 0:
                self.theta_bm, self.T_bm = theta, self.T
                self.middle_start = t
                self.state, self.entered = MIDDLE, t
                transition = "early_to_middle"
                if not 0 <= self.T_bm < self.p.t_em:
                    self.state = INVALID
                    transition = "invalid_T_bm"
        elif self.state == MIDDLE:
            x = inverse_quintic((theta - self.theta_bm) / self.p.rho_deg)
            self.T = self.T_bm + x * (self.p.t_em - self.T_bm)
            if theta - self.theta_bm > self.p.rho_deg:
                duration = t - self.middle_start
                self.rate = (self.p.t_em - self.T_bm) / duration if duration > 0 else -1
                self.T = self.p.t_em
                self.state, self.entered = LATE, t
                transition = "middle_to_late"
                if not np.isfinite(self.rate) or self.rate <= 0:
                    self.state = INVALID
                    transition = "invalid_rate"
        elif self.state == LATE:
            self.T += self.rate * dt
            if self.T >= 1:
                self.T = 0.0
                self.state, self.entered = STANCE, t
                self.completed += 1
                transition = "late_to_stance"
        return transition


def causal_filter(t, theta, cutoff_hz=6.0):
    out = np.empty_like(theta)
    out[0] = theta[0]
    for i in range(1, len(t)):
        a = np.exp(-2 * np.pi * cutoff_hz * (t[i] - t[i - 1]))
        out[i] = a * out[i - 1] + (1 - a) * theta[i]
    rate = np.r_[0.0, np.diff(out) / np.diff(t)]
    return out, rate


def replay(t, theta, p, *, pfd_duration=None, initial_stance_duration=0.6):
    """PFD uses same sigma trigger, fixed swing duration, no dtheta dependence."""
    angle, angular_rate = causal_filter(t, theta)
    est = TNT(p)
    state = np.zeros(len(t), dtype=int)
    phase = np.zeros(len(t))
    stance_progress = np.zeros(len(t))
    cycle = np.zeros(len(t), dtype=int)
    completed = np.zeros(len(t), dtype=int)
    rate = np.zeros(len(t))
    events = []
    stance_start = float(t[0])
    stance_duration = initial_stance_duration
    swing_start = None
    for i, now in enumerate(t):
        dt = 0.0 if i == 0 else float(now - t[i - 1])
        if pfd_duration is None:
            event = est.step(float(now), float(angle[i]), float(angular_rate[i]), dt)
        else:
            event = ""
            if est.state == STANCE and angle[i] < p.sigma_deg:
                est.state, est.T = EARLY, 0.0
                est.swing_id += 1
                swing_start = float(now)
                event = "stance_to_early"
            elif est.state != STANCE:
                est.T = float((now - swing_start) / pfd_duration)
                if est.T >= 1:
                    est.state, est.T = STANCE, 0.0
                    est.completed += 1
                    event = "late_to_stance"
        if event:
            events.append({"i": i, "t": float(now), "event": event,
                           "swing_id": est.swing_id, "T": float(est.T)})
        if event == "stance_to_early" and est.completed > 0:
            # Previous completed stance only; never look at the current future end.
            stance_duration = max(float(now - stance_start), 0.01)
        if event == "late_to_stance":
            stance_start = float(now)
        state[i], phase[i], cycle[i] = est.state, est.T, est.swing_id
        completed[i], rate[i] = est.completed, est.rate
        stance_progress[i] = np.clip((now - stance_start) / stance_duration, 0, 1)
    return dict(t=t, theta=angle, dtheta=angular_rate, state=state, T=phase,
                stance_progress=stance_progress, cycle=cycle, completed=completed,
                rate=rate, events=events)


def complete_blocks(r, mask, mode):
    selected = ((r["state"] == STANCE) if mode == STANCE else (r["state"] > 0)) & mask
    # Require genuine start/end boundaries inside this selection, not split fragments.
    edges = np.diff(np.r_[False, selected, False].astype(int))
    blocks = []
    for a, b in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)):
        if a == 0 or b == len(selected) or not mask[a - 1] or not mask[b]:
            continue
        if r["completed"][a] < 1 or b - a < 5:
            continue
        blocks.append((a, b))
    return blocks


def make_template(r, q, mask):
    grid = np.linspace(0, 1, 101)
    curves, counts, spreads = {}, {}, {}
    for mode, key, coordinate in [(STANCE, "stance", "stance_progress"),
                                  (EARLY, "swing", "T")]:
        samples = []
        for a, b in complete_blocks(r, mask, mode):
            u = r[coordinate][a:b]
            # Repeated phase values represent a held command: average all samples,
            # rather than choosing a joint target from one arbitrary timestamp.
            unique, inv = np.unique(u, return_inverse=True)
            if len(unique) < 5 or np.ptp(unique) < 0.7:
                continue
            means = np.stack([np.bincount(inv, weights=q[a:b, j]) /
                              np.bincount(inv) for j in range(2)], axis=1)
            samples.append(np.column_stack([np.interp(grid, unique, means[:, j])
                                             for j in range(2)]))
        if len(samples) < 3:
            raise ValueError(f"Only {len(samples)} complete {key} calibration blocks")
        samples = np.asarray(samples)
        curves[key] = samples.mean(axis=0)
        counts[key] = len(samples)
        spreads[key] = np.sqrt(np.mean(np.var(samples, axis=0), axis=0)).tolist()
    # Shared endpoint values enforce position continuity for an ideal full interval.
    # A premature stance exit can still cause a jump; measured explicitly below.
    boundary = (curves["stance"][-1] + curves["swing"][0]) / 2
    curves["stance"][-1] = curves["swing"][0] = boundary
    boundary = (curves["swing"][-1] + curves["stance"][0]) / 2
    curves["swing"][-1] = curves["stance"][0] = boundary
    return {"grid": grid, **curves, "block_counts": counts,
            "intercycle_rms_std_deg": spreads}


def predict(r, template, *, constant_stance=False):
    stance = r["state"] == STANCE
    prediction = np.empty((len(r["t"]), 2))
    for j in range(2):
        f = PchipInterpolator(template["grid"], template["swing"][:, j])
        prediction[:, j] = f(np.clip(r["T"], 0, 1))
        if constant_stance:
            prediction[stance, j] = template["stance"][:, j].mean()
        else:
            f = PchipInterpolator(template["grid"], template["stance"][:, j])
            prediction[stance, j] = f(r["stance_progress"][stance])
    prediction[r["state"] == INVALID] = np.nan
    return prediction


def duration_stats(r, mask):
    out = {}
    for mode, name in [(STANCE, "stance"), (EARLY, "swing")]:
        blocks = complete_blocks(r, mask, mode)
        out[name] = [float(r["t"][b] - r["t"][a]) for a, b in blocks]
    return out


def kinematic_minima(r):
    # Offline scoring only. These future-dependent peaks NEVER enter replay.
    dt = float(np.median(np.diff(r["t"])))
    return find_peaks(-r["theta"], prominence=5, distance=int(.5 / dt))[0]


def rate_diagnostics(r):
    cycles = {}
    for e in r["events"]:
        cycles.setdefault(e["swing_id"], {})[e["event"]] = e
    rows = []
    for cycle, events in cycles.items():
        a = events.get("stance_to_early")
        b = events.get("early_to_middle", events.get("invalid_T_bm"))
        c = events.get("middle_to_late")
        if a is None or b is None:
            continue
        de = b["t"] - a["t"]
        row = {"swing_id": cycle, "start_s": a["t"], "middle_start_s": b["t"],
               "early_duration_s": de, "T_bm": b["T"],
               "previous_rate": b["T"] / de if de > 0 else None,
               "invalid_T_bm": b["event"] == "invalid_T_bm"}
        if c is not None:
            dm = c["t"] - b["t"]
            row.update(middle_duration_s=dm, early_to_middle_duration_ratio=de / dm,
                       updated_rate=float(r["rate"][c["i"]]))
        rows.append(row)
    return rows


def score(r, q, prediction, mask):
    out = {}
    for region, m in [("all", mask), ("stance", mask & (r["state"] == STANCE)),
                      ("swing", mask & (r["state"] > 0))]:
        out[region] = {"samples": int(m.sum())}
        if not m.any():
            continue
        for j, joint in enumerate(["knee", "ankle"]):
            err = prediction[m, j] - q[m, j]
            out[region][joint] = dict(rmse_deg=float(np.sqrt(np.mean(err**2))),
                                     bias_deg=float(np.mean(err)),
                                     p95_abs_deg=float(np.quantile(abs(err), .95)))
    indices = np.flatnonzero(mask)
    state_switch = np.r_[False, np.diff(r["state"]) != 0] & mask
    reference_diff = np.vstack([np.zeros((1, 2)), np.diff(prediction, axis=0)])
    same_swing = (r["state"][1:] > 0) & (r["state"][:-1] > 0) & mask[1:]
    out["diagnostics"] = {
        "completed_swings": sum(e["event"] == "late_to_stance" and mask[e["i"]]
                                for e in r["events"]),
        "swing_starts": sum(e["event"] == "stance_to_early" and mask[e["i"]]
                            for e in r["events"]),
        "invalid_samples": int(((r["state"] == INVALID) & mask).sum()),
        "phase_regressions": int(((np.diff(r["T"]) < -1e-6) & same_swing).sum()),
        "max_phase_backstep": float(max(0, -np.min(np.diff(r["T"])[same_swing])))
            if same_swing.any() else 0.0,
        "state_samples": {STATE_NAMES[k]: int(((r["state"] == k) & mask).sum())
                          for k in STATE_NAMES},
        "max_state_boundary_step_deg": np.max(abs(reference_diff[state_switch]), axis=0).tolist()
            if state_switch.any() else [0, 0],
        "max_reference_step_deg": np.max(abs(reference_diff[mask]), axis=0).tolist(),
        "p99_abs_reference_velocity_deg_s": np.quantile(
            abs(np.diff(prediction, axis=0) / np.diff(r["t"])[:, None])[mask[1:]],
            .99, axis=0).tolist(),
        "window_s": [float(r["t"][indices[0]]), float(r["t"][indices[-1]])],
        "durations_s": duration_stats(r, mask),
    }
    minima = kinematic_minima(r)
    out["diagnostics"]["independent_thigh_minima"] = int(mask[minima].sum())
    invalid = np.flatnonzero((r["state"] == INVALID) & mask)
    out["status"] = "invalid_estimator" if len(invalid) else "completed_without_invalid_state"
    out["diagnostics"]["first_invalid_time_s"] = float(r["t"][invalid[0]]) if len(invalid) else None
    if len(invalid):
        # Partial performance is explicitly separate, never an all-window success.
        valid_prefix = mask & (np.arange(len(mask)) < invalid[0])
        out["valid_prefix_only"] = {"samples": int(valid_prefix.sum())}
        if valid_prefix.any():
            for j, joint in enumerate(["knee", "ankle"]):
                out["valid_prefix_only"][joint] = {"rmse_deg": float(np.sqrt(np.mean(
                    (prediction[valid_prefix, j] - q[valid_prefix, j])**2)))}
    return out


def candidate_parameters(t, theta, q):
    filtered, _ = causal_filter(t, theta)
    dt = np.median(np.diff(t))
    minima = find_peaks(-filtered, prominence=5, distance=int(.5 / dt))[0]
    excursions = [float(filtered[a:b].max() - filtered[a])
                  for a, b in zip(minima[:-1], minima[1:])]
    if len(excursions) < 5:
        raise ValueError("Too few calibration thigh cycles")
    period = float(np.median(np.diff(t[minima])))
    base_min = float(np.quantile(filtered[minima], .9))
    rom = float(np.median(excursions))
    low_rom = float(np.quantile(excursions, .1))
    # First-cycle-only rate, based on a transparent fraction of calibration cadence.
    initial_rate = 1.0 / (.45 * period)
    candidates = [Parameters(base_min + f * rom, r * low_rom, e, initial_rate)
                  for f in [.05, .15, .25]
                  for r in [.55, .70, .85]
                  for e in [.65, .75, .85]]
    return candidates, {"median_thigh_period_s": period,
                        "median_excursion_deg": rom, "p10_excursion_deg": low_rom,
                        "p90_thigh_minimum_deg": base_min,
                        "initial_stance_duration_s": .55 * period}


def calibrate(t, theta, q, calibration_seconds=40):
    stop = int(np.searchsorted(t, t[0] + calibration_seconds))
    inner = t[:stop] < t[0] + 25
    candidates, geometry = candidate_parameters(t[:stop][inner], theta[:stop][inner], q[:stop][inner])
    records = []
    for p in candidates:
        r = replay(t[:stop], theta[:stop], p,
                   initial_stance_duration=geometry["initial_stance_duration_s"])
        try:
            template = make_template(r, q[:stop], inner)
            pred = predict(r, template)
            result = score(r, q[:stop], pred, ~inner)
            error = sum(result["all"][j]["rmse_deg"] / max(np.ptp(q[:stop][inner, k]), 1)
                        for k, j in enumerate(["knee", "ankle"]))
            # Independent minima in the inner validation region, so an actual
            # cadence change is not penalised for differing from first25s cadence.
            expected = max(int((~inner)[kinematic_minima(r)].sum()), 1)
            count = result["diagnostics"]["completed_swings"]
            error += abs(count - expected) / expected
            if result["diagnostics"]["invalid_samples"] or not np.isfinite(error):
                error = 1e6
            records.append({"parameters": asdict(p), "selection_score": float(error),
                            "inner_validation_metrics": result})
        except ValueError as exc:
            records.append({"parameters": asdict(p), "selection_score": 1e6,
                            "error": str(exc)})
    best = min(records, key=lambda x: x["selection_score"])
    if best["selection_score"] >= 1e6:
        raise ValueError("All calibration candidates failed")
    return Parameters(**best["parameters"]), geometry, records


def save_trace(path, r, q, predictions):
    names = ["time_s", "thigh_filtered_deg", "thigh_rate_deg_s", "tnt_state", "T",
             "stance_progress", "prescribed_knee_deg", "prescribed_ankle_deg"]
    columns = [r["t"], r["theta"], r["dtheta"], r["state"], r["T"],
               r["stance_progress"], q[:, 0], q[:, 1]]
    for name, prediction in predictions.items():
        names += [f"{name}_knee_deg", f"{name}_ankle_deg"]
        columns += [prediction[:, 0], prediction[:, 1]]
    np.savetxt(path, np.column_stack(columns), delimiter=",", header=",".join(names),
               comments="", fmt="%.9g")


def plot_replay(path, subject, r, q, pred, pfd, cutoff):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    invalid = np.flatnonzero(r["state"] == INVALID)
    center = float(r["t"][invalid[0]]) if len(invalid) else cutoff + 8
    m = (r["t"] >= max(center - 4, cutoff)) & (r["t"] < center + 2)
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True, constrained_layout=True)
    axes[0].plot(r["t"][m], r["theta"][m], color="C2")
    axes[0].set_ylabel("Thigh [deg]")
    axes[1].plot(r["t"][m], r["T"][m], label="TNT T")
    axes[1].plot(r["t"][m], r["stance_progress"][m] * (r["state"][m] == STANCE),
                 label="Stance timer extension", alpha=.7)
    axes[1].set_ylabel("Progress")
    axes[1].legend(loc="upper right")
    for j, name in enumerate(["Knee", "Ankle"]):
        ax = axes[j + 2]
        ax.plot(r["t"][m], q[m, j], color="black", label="Prescribed (held out)")
        ax.plot(r["t"][m], pred[m, j], label="TNT + stance timer + fixed curves")
        ax.plot(r["t"][m], pfd[m, j], "--", alpha=.8, label="Fixed-duration swing")
        ax.set_ylabel(f"{name} [deg]")
        ax.legend(loc="upper right", fontsize=8)
    axes[-1].set_xlabel("Recorded time [s]")
    fig.suptitle(f"{subject}: training-free offline replay | no GRF | no dynamics")
    for ax in axes:
        ax.grid(alpha=.25)
        if len(invalid):
            ax.axvspan(center, center + 2, color="red", alpha=.12)
            ax.axvline(center, color="red", ls=":")
    if len(invalid):
        axes[0].set_title("Red region: estimator INVALID; no generated TNT reference", color="darkred")
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=160)
    plt.close(fig)


def run_subject(subject, calibration_seconds):
    path = HERE / f"{subject.lower()}_prescribed.csv"
    data = np.genfromtxt(path, delimiter=",", names=True)
    t = data["time_s"]
    theta = data["thigh_deg"]
    q = np.column_stack([data["knee_deg"], data["ankle_deg"]])
    cutoff = float(t[0] + calibration_seconds)
    calibration, heldout = t < cutoff, t >= cutoff
    p, geometry, grid = calibrate(t, theta, q, calibration_seconds)
    r = replay(t, theta, p, initial_stance_duration=geometry["initial_stance_duration_s"])
    template = make_template(r, q, calibration)
    prediction = predict(r, template)
    const_prediction = predict(r, template, constant_stance=True)
    durations = duration_stats(r, calibration)
    pfd_duration = float(np.median(durations["swing"]))
    pfd = replay(t, theta, p, pfd_duration=pfd_duration,
                 initial_stance_duration=geometry["initial_stance_duration_s"])
    pfd_template = make_template(pfd, q, calibration)
    pfd_prediction = predict(pfd, pfd_template)
    result = {"subject": subject, "source": str(path.relative_to(ROOT)),
              "source_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
              "parameters": asdict(p), "calibration_geometry": geometry,
              "calibration_window_s": [float(t[0]), cutoff],
              "inner_split_s": float(t[0] + 25),
              "heldout_samples": int(heldout.sum()),
              "pfd_swing_duration_s": pfd_duration,
              "template_block_counts": template["block_counts"],
              "template_intercycle_rms_std_deg": template["intercycle_rms_std_deg"],
              "tnt": score(r, q, prediction, heldout),
              "tnt_constant_stance_ablation": score(r, q, const_prediction, heldout),
              "fixed_duration": score(pfd, q, pfd_prediction, heldout),
              "synthetic_time_scaling": {}}
    # Speed up / slow down the recorded signals, never reinterpret as measured speeds.
    # Parameters, curves, and initial rates remain frozen. Same causal warmup for all.
    for scale in [.75, 1.25]:
        physical_time = np.arange(t[0], t[0] + (t[-1] - t[0]) / scale, .01)
        source_time = t[0] + (physical_time - t[0]) * scale
        th = np.interp(source_time, t, theta)
        target = np.column_stack([np.interp(source_time, t, q[:, j]) for j in range(2)])
        eval_mask = source_time >= cutoff
        a = replay(physical_time, th, p,
                   initial_stance_duration=geometry["initial_stance_duration_s"])
        b = replay(physical_time, th, p, pfd_duration=pfd_duration,
                   initial_stance_duration=geometry["initial_stance_duration_s"])
        result["synthetic_time_scaling"][str(scale)] = {
            "tnt": score(a, target, predict(a, template), eval_mask),
            "fixed_duration": score(b, target, predict(b, pfd_template), eval_mask)}
    # Paper thresholds are a separate estimator diagnostic, not a template comparison.
    raw = replay(t, theta, Parameters(-6, 32, .8, p.initial_rate),
                 initial_stance_duration=geometry["initial_stance_duration_s"])
    raw_durations = {}
    for state_id, state_name in STATE_NAMES.items():
        selected = raw["state"] == state_id
        edges = np.diff(np.r_[False, selected, False].astype(int))
        spans = [float(t[b - 1] - t[a] + .01) for a, b in
                 zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1))]
        raw_durations[state_name] = max(spans, default=0.0)
    result["paper_thresholds_estimator_only"] = {
        "parameters": {"sigma_deg": -6, "rho_deg": 32, "t_em": .8,
                       "initial_rate": p.initial_rate},
        "completed_swings_heldout": sum(e["event"] == "late_to_stance" and heldout[e["i"]]
                                         for e in raw["events"]),
        "first_swing_start_s": next((e["t"] for e in raw["events"] if e["event"] == "stance_to_early"), None),
        "longest_state_run_whole_recording_s": raw_durations,
        "state_samples_heldout": {STATE_NAMES[k]: int(((raw["state"] == k) & heldout).sum())
                                   for k in STATE_NAMES}}
    minima = kinematic_minima(r)
    heldout_minima = minima[heldout[minima]]
    result["heldout_kinematics_description_only"] = {
        "median_thigh_minimum_period_s": float(np.median(np.diff(t[heldout_minima]))),
        "thigh_range_deg": [float(r["theta"][heldout].min()), float(r["theta"][heldout].max())]}
    plot_path = ROOT / "plot" / "tnt_preliminary_2026_10_13" / f"{subject}_offline_replay.png"
    plot_replay(plot_path, subject, r, q, prediction, pfd_prediction, cutoff)
    result["plot"] = str(plot_path.relative_to(ROOT))
    save_trace(HERE / f"{subject.lower()}_replay.csv", r, q,
               {"tnt": prediction, "pfd": pfd_prediction, "constant_stance": const_prediction})
    write_json(HERE / f"{subject.lower()}_calibration_grid.json", grid)
    template_json = {k: v.tolist() if isinstance(v, np.ndarray) else v for k, v in template.items()}
    write_json(HERE / f"{subject.lower()}_template.json", template_json)
    write_json(HERE / f"{subject.lower()}_events.json", r["events"])
    write_json(HERE / f"{subject.lower()}_rate_diagnostics.json", rate_diagnostics(r))
    print(json.dumps(json_safe({"subject": subject, "parameters": asdict(p),
                      "tnt": result["tnt"]["all"], "fixed_duration": result["fixed_duration"]["all"],
                      "completed": result["tnt"]["diagnostics"]["completed_swings"]})), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--subjects", nargs="+", default=["AB06", "AB07"])
    parser.add_argument("--calibration-seconds", type=float, default=40)
    args = parser.parse_args()
    result = {
        "instruction_check_token": "CMC_AGENT_OK_2026",
        "scope": "Offline kinematic replay; no NN/RL training, no dynamic integration, no GRF read",
        "method": "Paper TNT + separately labelled causal previous-stance timer + fixed PCHIP templates",
        "filter": "First-order causal lowpass 6Hz; backward difference of filtered thigh",
        "calibration": "27 parameter candidates; geometry/template from first25s, select next15s; final curves first40s",
        "test": "All subsequent recorded samples, no parameter/template selection from heldout",
        "revision_history": "attempt00 archived: expected inner-validation count used initial cadence; corrected to independent thigh minima. Selected parameters and failures unchanged.",
        "baseline": "PFD shares sigma and calibration data; its own phase-indexed template is fitted. It is not the identical waveform under a different clock.",
        "initialization": "Starts in stance; first swing is bootstrap, not a verified whole physical gait cycle",
        "continuity": "Template endpoints share positions; premature stance exit and variable phase rate can still cause jumps",
        "limitations": ["TNT internal states do not label physical HS/TO or load", "Stance timer is an extension to paper TNT",
                        "Same-trial temporal holdout, subject-specific calibration; not cross-subject qualification",
                        "Synthetic time scaling is not real different-speed or transient-speed validation",
                        "Offline angle reconstruction error is not dynamic tracking or physical stability"],
        "results": [run_subject(s, args.calibration_seconds) for s in args.subjects]}
    write_json(HERE / "results.json", result)


if __name__ == "__main__":
    main()
