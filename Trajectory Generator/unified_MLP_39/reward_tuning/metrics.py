"""Physical metrics; incomplete traces never become successful short rollouts."""
from __future__ import annotations

import io
import json
from pathlib import Path

import numpy as np
from scipy.signal import butter, sosfiltfilt

from .io import relative, sha256

JOINTS = ("knee", "ankle")


def _finite_array(values, name):
    try:
        values = np.asarray(values, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Invalid numeric {name}") from exc
    if values.ndim != 1 or not np.all(np.isfinite(values)):
        raise ValueError(f"Missing or nonfinite {name}")
    return values


def _signal(values, times, name):
    values, times = _finite_array(values, name), _finite_array(times, "time")
    if values.shape != times.shape or np.any(np.diff(times) <= 0):
        raise ValueError(f"Invalid length or time ordering for {name}")
    return values, times


def _trace_values(rows, container, key):
    try:
        return _finite_array([r[container][key] for r in rows], key)
    except (KeyError, TypeError) as exc:
        raise ValueError(f"Missing required trace signal: {container}.{key}") from exc


def _sample_interval(times, name):
    times = _finite_array(times, name + " time")
    if len(times) < 2 or np.any(np.diff(times) <= 0):
        raise ValueError(f"Insufficient or unordered {name} samples")
    dt = float(np.median(np.diff(times)))
    if not np.allclose(np.diff(times), dt, atol=1e-7, rtol=1e-5):
        raise ValueError(f"{name} samples are irregular")
    return dt


def _require_window_coverage(times, dt, lo, hi, name):
    # Stored substeps represent [sample_time, sample_time + dt), so the last
    # valid pre-integration sample normally ends one dt before the boundary.
    if times[0] > lo + 1e-7 or times[-1] + dt < hi - 1e-7:
        raise ValueError(f"{name} does not cover the requested physical window")


def rms(values):
    values = _finite_array(values, "RMS values")
    return float(np.sqrt(np.mean(values ** 2))) if values.size else None


def read_table(path):
    raw = Path(path).read_text(encoding="utf-8")
    body = raw.split("endheader", 1)[1].strip()
    columns = body.splitlines()[0].split()
    values = np.loadtxt(io.StringIO(body), skiprows=1, ndmin=2)
    if values.shape[1] != len(columns) or len(set(columns)) != len(columns) or "time" not in columns:
        raise ValueError(f"Invalid STO columns: {path}")
    result = dict(zip(columns, values.T))
    t = result["time"]
    if len(t) < 2 or np.any(np.diff(t) <= 0) or not np.all(np.isfinite(values)):
        raise ValueError(f"Invalid or nonfinite STO: {path}")
    return result


def completed_row(row):
    """Legacy hard failures repeat logical time despite a requested next segment."""
    time = _finite_array([row.get("time")], "trace time")[0]
    times = _finite_array(row.get("policy_segment_times", []), "policy segment times")
    if len(times) < 2 or np.any(np.diff(times) <= 0):
        raise ValueError("Missing or unordered policy segment times")
    if row.get("segment_completed") is False:
        return False
    return bool(time >= times[-1] - 1e-7)


def load_dataset(path, *, identity=None, group="left", scenario="nominal"):
    path = Path(path)
    hashes = {}
    def read_json(name):
        file = path / name
        hashes[relative(file)] = sha256(file)
        return json.loads(file.read_text(encoding="utf-8"))
    summary = read_json("rollout_summary.json")
    trace = read_json("rollout_policy_trace.json")
    if not trace:
        raise ValueError(f"Empty policy trace: {path}")
    rows = [row for row in trace if completed_row(row)]
    times = np.asarray([row["time"] for row in rows], float)
    if not len(rows) or np.any(np.diff(times) <= 0):
        raise ValueError(f"No strictly ordered complete segments: {path}")
    tables, missing = {}, []
    for name in ("sea_torques", "kinematics", "kinematics_reference", "online_grf"):
        file = path / "sim_outputs" / f"rollout_episode_{name}.sto"
        if file.is_file():
            hashes[relative(file)] = sha256(file)
            tables[name] = read_table(file)
        else:
            missing.append(relative(file))
    return dict(id=identity or path.name, group=group, scenario=scenario,
                path=relative(path), summary=summary, trace=trace, rows=rows,
                tables=tables, input_hashes=hashes, missing_files=missing)


def highpass(values, times, cutoff):
    values, times = _signal(values, times, "high-pass input")
    if not np.isfinite(cutoff) or cutoff <= 0:
        raise ValueError("High-pass cutoff must be finite and positive")
    if len(times) < 20:
        return None
    dt = float(np.median(np.diff(times)))
    if not np.allclose(np.diff(times), dt, rtol=1e-4, atol=1e-8):
        raise ValueError("Spectral metrics require uniformly sampled signals")
    if cutoff >= .5 / dt:
        return None
    return sosfiltfilt(butter(4, cutoff, fs=1 / dt, btype="highpass", output="sos"), values)


def causal_jerk(times, velocities):
    """Second divided difference of velocity; values belong to interval midpoints."""
    velocities, times = _signal(velocities, times, "actual velocities")
    output = np.full(len(times), np.nan)
    if len(times) >= 3:
        dt = np.diff(times)
        if np.any(dt <= 0):
            raise ValueError("Derivative time must increase")
        acceleration = np.diff(velocities) / dt
        output[2:] = np.diff(acceleration) / ((dt[1:] + dt[:-1]) / 2)
    return output


def measure(dataset, window=None, *, filter_stop=None):
    """Measure full-filter-then-crop signals, optionally excluding a future tail.

    Pair comparisons pass their shared end as ``filter_stop``: filtering keeps
    all earlier history for warmup but never consumes samples after that end.
    The same zero-phase filter and terminal edge treatment apply to both arms.
    Full-episode completion metadata always describes the original rollout.
    """
    full_rows, tables = dataset["rows"], dataset["tables"]
    full_times = _finite_array([r["time"] for r in full_rows], "trace time")
    if not len(full_times) or np.any(np.diff(full_times) <= 0):
        raise ValueError("No strictly ordered complete trace samples")
    rows = full_rows
    if filter_stop is not None:
        filter_stop = float(_finite_array([filter_stop], "filter stop")[0])
        rows = [r for r in full_rows if r["time"] <= filter_stop + 1e-8]
        tables = {name: {key: values[table["time"] <= filter_stop + 1e-8]
                         for key, values in table.items()} for name, table in tables.items()}
    if not rows:
        raise ValueError("No samples before filter stop")
    times = _finite_array([r["time"] for r in rows], "trace time")
    start = float(rows[0]["policy_segment_times"][0])
    stop = float(times[-1])
    lo, hi = (start, stop) if window is None else (max(start, window[0]), min(stop, window[1]))
    if hi <= lo:
        raise ValueError("No common physical time window")
    mask = (times >= lo - 1e-8) & (times <= hi + 1e-8)
    selected = [r for r, keep in zip(rows, mask) if keep]
    if not selected:
        raise ValueError("No complete trace samples inside metric window")
    tracking, smoothness = {}, {}
    for joint in JOINTS:
        coord = f"pros_{joint}_angle"
        q = _trace_values(rows, "prosthetic_state", coord)
        served = _trace_values(rows, "prosthetic_state", coord + "_served_ref")
        target = _trace_values(rows, "imitation_target_q", coord)
        phase = np.asarray([r["reward_terms"].get(coord + "_imitation_target_phase", np.nan) for r in rows], dtype=float)
        actual_error = np.rad2deg(q - target)
        served_error = np.rad2deg(served - target)
        bins = {}
        for index in range(10):
            pm = mask & (phase >= index / 10) & (phase < (index + 1) / 10)
            bins[str(index)] = {"samples": int(pm.sum()), "actual_rmse_deg": rms(actual_error[pm]),
                                "served_rmse_deg": rms(served_error[pm])}
        tracking[joint] = {"actual_rmse_deg": rms(actual_error[mask]),
                           "served_rmse_deg": rms(served_error[mask]),
                           "excursion_deg": float(np.rad2deg(np.ptp(q[mask]))),
                           "phase_available_samples": int(np.count_nonzero(mask & np.isfinite(phase))),
                           "phase_bins": bins}
        for label, values, velocity_key in (
            ("actual", q, coord + "_vel"), ("served", served, coord + "_served_ref_vel")
        ):
            # Reset velocity primes the same causal derivative used by online shaping.
            initial_key = coord + ("_vel" if label == "actual" else "_served_ref_vel")
            before = rows[0].get("actor_observation_before", {})
            vel = _trace_values(rows, "prosthetic_state", velocity_key)
            if initial_key in before:
                jerk = causal_jerk(np.r_[start, times], np.r_[before[initial_key], vel])[1:]
            else:
                jerk = causal_jerk(times, vel)
            hp = highpass(values, times, 10)
            smoothness[f"{joint}_{label}"] = {
                "jerk_rms_rad_s3": rms(jerk[mask & np.isfinite(jerk)]),
                "jerk_samples": int(np.count_nonzero(mask & np.isfinite(jerk))),
                "position_hp10_rms_deg": None if hp is None else float(np.rad2deg(rms(hp[mask]))),
                "sample_hz": float(1 / np.median(np.diff(times))) if len(times) > 1 else None,
            }
    result = {"window_s": [lo, hi], "complete_segments": len(selected),
              "tracking": tracking, "kinematic_smoothness": smoothness,
              "ankle": None, "knee": None, "contact": None}
    torque = tables.get("sea_torques")
    grf = tables.get("online_grf")
    if torque is not None:
        t = torque["time"]
        dt = _sample_interval(t, "Torque")
        _require_window_coverage(t, dt, lo, hi, "Torque")
        tm = (t >= lo - 1e-8) & (t <= hi + 1e-8)
        ankle = torque["SEA_Ankle_tau_spring"]
        positive = np.maximum(ankle, 0)
        am = {"positive_area_nm_s": float(positive[tm].sum() * dt),
              "positive_peak_nm": float(positive[tm].max()),
              "positive_duration_s": float(np.count_nonzero(tm & (ankle > 0)) * dt),
              "sample_hz": 1 / dt, "loaded_positive_area_nm_s": None,
              "swing_positive_area_nm_s": None, "unloaded_positive_area_nm_s": None}
        if grf is not None:
            gdt = _sample_interval(grf["time"], "GRF")
            _require_window_coverage(grf["time"], gdt, lo, hi, "GRF")
            if t[tm][0] < grf["time"][0] - 1e-7 or t[tm][-1] > grf["time"][-1] + 1e-7:
                raise ValueError("GRF coverage cannot classify torque samples without extrapolation")
            force = np.full(len(t), np.nan)
            force[tm] = np.interp(t[tm], grf["time"], grf["left_normal_force"])
            for name, subset in (("loaded", force >= 50), ("unloaded", force < 50)):
                am[f"{name}_positive_area_nm_s"] = float(positive[tm & subset].sum() * dt)
        # The trace FSM is end-of-segment; classify its preceding substeps consistently.
        indices = np.searchsorted(times, t + 1e-8, side="left")
        indices = np.clip(indices, 0, len(rows) - 1)
        states = np.asarray([r.get("phase_fsm", {}).get("state_id", np.nan) for r in rows], dtype=float)
        swing = states[indices] == 2
        if np.all(np.isfinite(states[indices][tm])):
            am["swing_positive_area_nm_s"] = float(positive[tm & swing].sum() * dt)
        result["ankle"] = am
        knee = torque["SEA_Knee_tau_spring"]
        km = {"rate_rms_nm_s": rms((np.diff(knee) / np.diff(t))[tm[1:] & tm[:-1]])}
        for hz in (10, 20):
            hp = highpass(knee, t, hz)
            km[f"hp{hz}_rms_nm"] = None if hp is None else rms(hp[tm])
        result["knee"] = km
    if grf is not None:
        t = grf["time"]
        dt = _sample_interval(t, "GRF")
        _require_window_coverage(t, dt, lo, hi, "GRF")
        gm = (t >= lo - 1e-8) & (t <= hi + 1e-8)
        result["contact"] = {
            "vertical_impulse_n_s": float(grf["left_normal_force"][gm].sum() * dt),
            "loaded_time_s": float(np.count_nonzero(gm & (grf["left_normal_force"] >= 50)) * dt),
            "penetration_max_mm": float(grf["left_penetration"][gm].max() * 1000),
            "sample_hz": 1 / dt,
        }
    summary = dataset["summary"]
    result["episode"] = {
        "completed": summary.get("end_reason") == "episode_time_limit",
        "duration_s": float(full_times[-1]) - start, "start_s": start, "stop_s": float(full_times[-1]),
        "steps": summary["steps"], "incomplete_rows": len(dataset["trace"]) - len(full_rows),
        "valid_cycles": summary.get("phase_valid_cycle_count"),
        "end_reason": summary.get("end_reason"),
        "penetration_max_mm": None if summary.get("grf_penetration_max_m") is None else 1000 * summary["grf_penetration_max_m"],
    }
    return result
