"""Gate logic of the F2R bridge (S0 tooling): read-only record extraction and
pure gate functions A / B / C / V / R (protocol rev 3, ``f2r_protocol.json``).

Two layers, both deterministic and numpy-only (no torch, no simulator):

* ``extract_record`` reduces one finished rollout_eval job directory to a flat
  record (``RECORD_KEYS``).  Scalar F0-consistent metrics (kinematics, penetration,
  reserve, actions, FSM counters) come from ``f1_analysis.f1_job_metrics`` when
  the receipt carries the F1 provenance fields, otherwise from the same
  definitions re-applied to the job outputs (``scalar_metrics_from_outputs``).
  The F2R-specific quantities (first heel-strike delay, online clock onset and
  periods, stance fraction, phase-aligned knee/ankle metrics against the pinned
  corridor mean, corridor coverage, online-vs-prescribed drift) are computed here
  from the trace, the reset diagnostics and the kinematics STO, with the F0
  overlay library for the FSM cycle chain and the event-warped phase mapping.
  Nothing is written; ``runs/`` is only read.

* ``gate_A`` / ``gate_B`` / ``gate_C`` (3 deterministic seed-123 records keyed by
  start), ``gate_V`` (3 held-out seed-125 records), ``gate_R`` (9 stochastic
  records) and ``evaluate_round`` are pure functions of records + thresholds.
  Every numeric threshold is read from ``protocol["gates"]`` (single source); a
  missing or ``None`` value (e.g. a start without a valid cycle) FAILS the check
  it feeds; structural violations (wrong seeds, missing starts, unknown rule)
  raise ``F2RContractError`` instead of silently passing or failing.
"""

from __future__ import annotations

import math
import re
import sys
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np

import f2r_common as R  # noqa: E402  (puts F0/F1 on sys.path)
import f0_common as C  # noqa: E402
import f0_matrix_analysis as A  # noqa: E402  (F0 library: tracking, counters, validators)
import f0_overlays as OV  # noqa: E402  (F0 library: FSM cycle chain, phase mapping)
import f0_replay_analysis as RA  # noqa: E402  (F0 library: read_sto / column)
import f1_analysis as FA  # noqa: E402  (F1 library: f1_job_metrics)
import f1_dataset as DS  # noqa: E402  (F1 library: trajectory_from_job + t_pre contract, file names)

F2RContractError = R.F2RContractError

RECORD_SCHEMA = 1
RECORD_KEYS: tuple[str, ...] = (
    "job_id", "start", "seed", "action_selection", "steps", "end_reason", "horizon_completed", "phase_timeout",
    "valid_cycles", "valid_hs", "valid_to", "invalid_events", "hs_cancelled_count", "resync_count",
    "first_hs_delay_s", "online_clock_informative_s", "online_periods_s", "drift_cycle_per_s",
    "knee_rom", "knee_min", "knee_max", "ankle_min", "ankle_max",
    "penetration_max_m", "reserve_norm_max_nm", "action_abs_max_raw", "action_clipped_steps",
    "stance_fraction", "knee_rmse", "knee_r", "ankle_rmse", "ankle_min_window",
    "settled_fraction", "dropped_fraction",
)
PHASE_ALIGNED_KEYS: tuple[str, ...] = ("stance_fraction", "knee_rmse", "knee_r", "ankle_rmse", "ankle_min_window")
F1_RECEIPT_FIELDS: tuple[str, ...] = ("config", "config_sha256", "rollout_eval_sha256", "command", "env_manifest", "env_manifest_sha256", "runtime", "candidate", "role", "family", "comparison_class", "start", "action_selection", "seed", "repeat")
PHASE_GRID_POINTS = 101
PHASE_GRID = np.linspace(0.0, 1.0, PHASE_GRID_POINTS)
DEAD_ONLINE_CLOCK = (0.0, 1.0, 0.0)  # (sin, cos, duration) emitted before the online clock is informative
ONLINE_CLOCK_NAMES = ("online_left_gait_phase_sin", "online_left_gait_phase_cos", "online_left_cycle_duration_s")
PERIOD_ROUND_DECIMALS = 6
HORIZON_END_REASON = A.HORIZON_END_REASON
GRF_PENETRATION_END_REASON = "grf_penetration"
PHASE_TIMEOUT_PREFIX = "phase_timeout"
RULES: tuple[str, ...] = ("all3", "2of3", "all3_and_2of3")
A_CHECKS: tuple[str, ...] = ("A1_first_hs_delay_s", "A2_cycles", "A3_hs_cancelled_max", "A4_horizon", "A5_online_clock_informative_within_s", "A6_online_period_s", "A7_stance_fraction_min")
B_CHECKS: tuple[str, ...] = ("B1_knee_rom_min_rad", "B2_knee_vs_corridor", "B3_ankle_negative_window", "B4_ankle_rmse_max_rad", "B5_stance_fraction", "B6_corridor_coverage")
C_CHECKS: tuple[str, ...] = ("C1_penetration_max_m", "C2_no_grf_penetration_termination", "C3_reserve_norm_max_nm_per_start", "C4_clipped_steps_max", "C5_action_abs_max")
T3_TRIGGER_CHECKS: tuple[str, ...] = ("B3_ankle_negative_window", "B2_knee_vs_corridor", "B5_stance_fraction")
NEXT_PROMOTE = "promote_to_R_without_aggregation"
NEXT_NOT_PROMOTED = "not_promoted"
_FRACTION_RE = re.compile(r"(\d+)\s*/\s*(\d+)")


# --- thresholds ---------------------------------------------------------------------


def gate_thresholds(protocol: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """``protocol["gates"]`` validated against the check names this module implements
    (fail-closed on any drift between the frozen JSON and the code)."""
    payload = R.load_protocol() if protocol is None else protocol
    gates = payload.get("gates") if isinstance(payload, Mapping) else None
    if not isinstance(gates, Mapping):
        raise F2RContractError("protocol without a 'gates' mapping")
    for gate, names in (("A", A_CHECKS), ("B", B_CHECKS), ("C", C_CHECKS)):
        block = gates.get(gate)
        if not isinstance(block, Mapping) or tuple(block.keys()) != names:
            raise F2RContractError(f"protocol gate {gate} checks {list(block.keys()) if isinstance(block, Mapping) else block!r} != {list(names)}")
        for name in names:
            if block[name].get("rule") not in RULES:
                raise F2RContractError(f"protocol check {name} without a known rule")
    for gate in ("V", "R"):
        block = gates.get(gate)
        if not isinstance(block, Mapping) or not isinstance(block.get("rules"), list) or not isinstance(block.get("jobs"), int):
            raise F2RContractError(f"protocol gate {gate} malformed")
    if int(gates["V"]["seed"]) != R.VALIDATION_SEED or int(gates["V"]["jobs"]) != len(R.STARTS):
        raise F2RContractError("protocol gate V must use the held-out seed on the 3 starts")
    if [int(s) for s in gates["R"]["seeds"]] != sorted({*R.COLLECTION_SEEDS, R.VALIDATION_SEED}) or int(gates["R"]["jobs"]) != len(R.STARTS) * 3:
        raise F2RContractError("protocol gate R must cover seeds 123/124/125 on the 3 starts")
    return dict(gates)


def rule_fraction(rules: Sequence[str], keyword: str, *, jobs: int) -> int:
    """Minimum count ``k`` parsed from the rule string containing ``keyword`` as ``k/n``
    (``n`` must equal ``jobs``).  Keeps the protocol JSON the single numeric source."""
    hits = [r for r in rules if isinstance(r, str) and keyword in r]
    if len(hits) != 1:
        raise F2RContractError(f"expected exactly one rule containing {keyword!r}, got {hits}")
    match = _FRACTION_RE.search(hits[0])
    if match is None:
        raise F2RContractError(f"rule {hits[0]!r} carries no k/n fraction")
    k, n = int(match.group(1)), int(match.group(2))
    if n != int(jobs) or not (0 < k <= n):
        raise F2RContractError(f"rule {hits[0]!r}: fraction {k}/{n} inconsistent with {jobs} jobs")
    return k


# --- small numeric helpers ----------------------------------------------------------------


def _finite(value: Any) -> bool:
    return A.is_finite_float(value)


def _num(value: Any) -> float | None:
    """Finite number -> float; anything else (None, bool, NaN, str) -> None."""
    return float(value) if _finite(value) else None


def _le(value: Any, limit: float) -> bool:
    v = _num(value)
    return v is not None and v <= float(limit)


def _ge(value: Any, limit: float) -> bool:
    v = _num(value)
    return v is not None and v >= float(limit)


def _within(value: Any, lo: float, hi: float) -> bool:
    v = _num(value)
    return v is not None and float(lo) <= v <= float(hi)


def _median_or_none(values: Sequence[float | None]) -> float | None:
    vals = [float(v) for v in values if v is not None]
    if not vals or len(vals) != len(values):
        return None
    return float(np.median(np.asarray(vals, dtype=np.float64)))


# --- prescribed / online phase ---------------------------------------------------------


def prescribed_phase(times: Sequence[float] | np.ndarray, heel_strikes: Sequence[float] | np.ndarray, *, offset: float = 0.0) -> np.ndarray:
    """Sound-side prescribed gait phase in [0, 1) at ``times`` from the heel-strike
    list (same algebra as ``osim_trj_cmc_like.GaitPhaseClock.phase``: fraction of
    the bracketing cycle, extrapolated with the nearest period outside the list,
    minus ``offset`` wrapped into [0, 1))."""
    hs = np.asarray(sorted(float(t) for t in np.asarray(heel_strikes, dtype=np.float64).reshape(-1)), dtype=np.float64)
    hs = hs[np.isfinite(hs)]
    if hs.size > 1:
        hs = hs[np.concatenate(([True], np.diff(hs) > 1e-9))]
    if hs.size < 2:
        raise F2RContractError("prescribed clock needs >= 2 heel strikes")
    t = np.asarray(times, dtype=np.float64).reshape(-1)
    if not np.all(np.isfinite(t)):
        raise F2RContractError("non-finite times for the prescribed phase")
    k = np.clip(np.searchsorted(hs, t, side="right") - 1, 0, hs.size - 2)
    raw = (t - hs[k]) / (hs[k + 1] - hs[k])  # bracketing cycle (also the first cycle for t < hs[0])
    after = t >= hs[-1]
    raw = np.where(after, (t - hs[-1]) / (hs[-1] - hs[-2]), raw)  # env formula past the last strike
    x = raw - (float(offset) % 1.0)
    return x - np.floor(x)


def online_phase(sin: np.ndarray, cos: np.ndarray) -> np.ndarray:
    """Phase in [0, 1) recovered from the online clock pair (atan2 / 2 pi, wrapped)."""
    ph = np.arctan2(np.asarray(sin, dtype=np.float64), np.asarray(cos, dtype=np.float64)) / (2.0 * math.pi)
    return ph - np.floor(ph)


def online_clock_columns(names35: Sequence[str]) -> tuple[int, int, int]:
    names = [str(n) for n in names35]
    if names != list(R.FEATURE_NAMES_35):
        raise F2RContractError("names35 differs from the pinned 35D manifest")
    cols = tuple(names.index(n) for n in ONLINE_CLOCK_NAMES)
    if cols != tuple(R.ONLINE_CLOCK_TRIPLET):
        raise F2RContractError(f"online clock triplet at {cols}, registry says {R.ONLINE_CLOCK_TRIPLET}")
    return cols  # type: ignore[return-value]


def online_clock_diagnostics(obs35: np.ndarray, t_pre: np.ndarray, reset_time: float, *, columns: tuple[int, int, int] = tuple(R.ONLINE_CLOCK_TRIPLET)) -> dict[str, Any]:
    """Onset (seconds after reset, at the pre-step time of the first row whose
    triplet != (0, 1, 0)), distinct positive cycle durations (rounded to
    ``PERIOD_ROUND_DECIMALS``, first-appearance order) and the dead-clock mask."""
    obs = np.asarray(obs35, dtype=np.float64)
    t = np.asarray(t_pre, dtype=np.float64).reshape(-1)
    if obs.ndim != 2 or obs.shape[0] != t.shape[0] or obs.shape[1] <= max(columns):
        raise F2RContractError("obs35 / t_pre shape mismatch for the online clock diagnostics")
    trip = obs[:, list(columns)]
    dead = (trip[:, 0] == DEAD_ONLINE_CLOCK[0]) & (trip[:, 1] == DEAD_ONLINE_CLOCK[1]) & (trip[:, 2] == DEAD_ONLINE_CLOCK[2])
    saturated = (trip[:, 0] == DEAD_ONLINE_CLOCK[0]) & (trip[:, 1] == DEAD_ONLINE_CLOCK[1])
    informative = np.flatnonzero(~dead)
    first = int(informative[0]) if informative.size else None
    periods: list[float] = []
    for d in trip[:, 2]:
        if d > 0.0:
            v = float(round(float(d), PERIOD_ROUND_DECIMALS))
            if v not in periods:
                periods.append(v)
    return {
        "informative_index": first,
        "informative_s": float(t[first] - float(reset_time)) if first is not None else None,
        "periods_s": periods,
        "dead_mask": dead,
        "saturated_mask": saturated,
    }


DRIFT_ESTIMATOR = "unwrapped_circular_difference"  # amendment A6-unwrap (2026-08-23): see f2r_protocol.json gates.A.A6_online_period_s


def drift_cycle_per_s(obs35: np.ndarray, t_pre: np.ndarray, heel_strikes: Sequence[float] | np.ndarray, *, offset: float = 0.0, columns: tuple[int, int, int] = tuple(R.ONLINE_CLOCK_TRIPLET), min_rows: int = 2) -> dict[str, Any]:
    """Linear slope vs time (cycle/s) of the **unwrapped** circular difference between the
    prescribed phase and the online phase over the rows after the online clock became
    informative, excluding rows where the online pair saturates at (0, 1).

    Amendment A6-unwrap (2026-08-23, threshold 0.055 unchanged): the centred difference
    ``((prescribed - online + 0.5) mod 1) - 0.5`` is unwrapped (``numpy.unwrap`` on 2*pi*diff,
    divided by 2*pi) before the fit, so an offset that crosses +-0.5 cycle — the physiological
    half-cycle lag between the prosthetic (online) and the sound-side (prescribed) heel-strike
    clocks — no longer flips sign mid-series and inflates the slope.  ``slope_wrapped_legacy``
    (the pre-amendment estimator) is kept for audit only.  Fail-closed: ``slope`` is None when
    fewer than ``min_rows`` rows qualify or when any value is non-finite (A6 then FAILS)."""
    obs = np.asarray(obs35, dtype=np.float64)
    t = np.asarray(t_pre, dtype=np.float64).reshape(-1)
    diag = online_clock_diagnostics(obs, t, float(t[0]) if t.size else 0.0, columns=columns)
    first = diag["informative_index"]
    if first is None:
        return {"slope": None, "rows": 0, "reason": "online clock never informative"}
    sel = np.arange(t.shape[0]) >= first
    sel &= ~diag["saturated_mask"]
    n = int(sel.sum())
    if n < int(min_rows):
        return {"slope": None, "rows": n, "reason": "too few informative rows"}
    on = online_phase(obs[sel, columns[0]], obs[sel, columns[1]])
    pre = prescribed_phase(t[sel], heel_strikes, offset=offset)
    diff = ((pre - on + 0.5) % 1.0) - 0.5
    tt = t[sel]
    if float(np.ptp(tt)) <= 0.0:
        return {"slope": None, "rows": n, "reason": "degenerate time span"}
    if not (np.all(np.isfinite(diff)) and np.all(np.isfinite(tt))):
        return {"slope": None, "rows": n, "reason": "non-finite phase difference or time"}
    unwrapped = np.unwrap(diff * (2.0 * math.pi)) / (2.0 * math.pi)
    slope, intercept = np.polyfit(tt - tt[0], unwrapped, 1)
    slope_legacy = float(np.polyfit(tt - tt[0], diff, 1)[0])
    if not (math.isfinite(float(slope)) and math.isfinite(float(intercept))):
        return {"slope": None, "rows": n, "reason": "non-finite drift fit"}
    return {"slope": float(slope), "intercept": float(intercept), "rows": n, "first_index": int(first), "estimator": DRIFT_ESTIMATOR, "slope_wrapped_legacy": slope_legacy, "wrap_crossings": int(np.sum(np.abs(np.diff(diff)) > 0.5)), "diff_min": float(diff.min()), "diff_max": float(diff.max()), "unwrapped_min": float(unwrapped.min()), "unwrapped_max": float(unwrapped.max())}


# --- FSM cycles / phase-aligned metrics -----------------------------------------------------


def first_accepted_event_time(rows: Sequence[Mapping[str, Any]], event: str) -> float | None:
    """``event_time_s`` of the first accepted FSM transition named ``event`` (None if absent)."""
    for r in rows:
        fsm = r.get("phase_fsm")
        for tr in (fsm.get("accepted_transitions_this_step") if isinstance(fsm, Mapping) else None) or []:
            if tr.get("event") == event:
                et = tr.get("event_time_s")
                if not _finite(et):
                    raise F2RContractError(f"accepted {event} without finite event_time_s at step {r.get('step')}")
                return float(et)
    return None


def stance_fraction_of(cycles: Sequence[Mapping[str, Any]]) -> float | None:
    """Median over accepted FSM cycles of (toe_off - hs_start) / duration (None without cycles)."""
    vals = []
    for c in cycles:
        hs0, to, dur = float(c["hs_start_s"]), float(c["toe_off_s"]), float(c["duration_s"])
        if not (dur > 0.0 and hs0 < to < hs0 + dur + 1e-12):
            raise F2RContractError(f"malformed cycle for the stance fraction: {c}")
        vals.append((to - hs0) / dur)
    return float(np.median(vals)) if vals else None


def corridor_reference(profile_path: Path = C.MORPH_PROFILE_EVENT_WARPED, *, grid: np.ndarray = PHASE_GRID, expected_sha256: str | None = R.CORRIDOR_PROFILE["sha256"]) -> dict[str, Any]:
    """Event-warped corridor mean of both prosthetic coordinates resampled on ``grid``
    plus the canonical toe-off phase alpha; the profile digest is pinned."""
    info = OV.phase_profile_alpha(Path(profile_path))
    if expected_sha256 is not None and info["sha256"] != expected_sha256:
        raise F2RContractError(f"corridor profile digest {info['sha256']} != pinned {expected_sha256}")
    prof = C.read_json(Path(profile_path))
    pg = np.asarray(prof["phase_grid"], dtype=np.float64)
    if pg.ndim != 1 or pg.size < 2 or np.any(np.diff(pg) < 0) or pg[0] != 0.0 or pg[-1] != 1.0:
        raise F2RContractError("corridor phase_grid must be non-decreasing on [0, 1]")
    g = np.asarray(grid, dtype=np.float64)
    ref = {}
    for coord in R.PROS_COORDS:
        mean = np.asarray(prof["coordinates"][coord]["mean_rad"], dtype=np.float64)
        if mean.shape != pg.shape or not np.all(np.isfinite(mean)):
            raise F2RContractError(f"corridor mean_rad malformed for {coord}")
        ref[coord] = np.interp(g, pg, mean)
    return {"alpha": float(info["alpha"]), "grid": g, "reference": ref, "profile": {"path": info["path"], "sha256": info["sha256"]}}


def phase_aligned_metrics(cycles: Sequence[Mapping[str, Any]], sto_time: np.ndarray, series: Mapping[str, np.ndarray], reference: Mapping[str, np.ndarray], *, alpha: float, grid: np.ndarray = PHASE_GRID, window: tuple[float, float]) -> dict[str, Any]:
    """Per valid cycle: resample each coordinate on the event-warped phase grid
    (``f0_overlays.phase_times``) and compare with the corridor mean through
    ``f0_matrix_analysis.tracking``; ``ankle_min_window`` = min of the resampled
    ankle over ``window``.  Per-job value = median over cycles; None without cycles
    (or when a Pearson r is undefined)."""
    t = np.asarray(sto_time, dtype=np.float64).reshape(-1)
    if t.size < 2 or np.any(np.diff(t) <= 0.0):
        raise F2RContractError("STO time grid must be strictly increasing")
    g = np.asarray(grid, dtype=np.float64)
    lo, hi = float(window[0]), float(window[1])
    if not (0.0 <= lo < hi <= 1.0):
        raise F2RContractError(f"invalid phase window {window}")
    win = (g >= lo) & (g <= hi)
    per_cycle: list[dict[str, Any]] = []
    for c in cycles:
        times = OV.phase_times(dict(c), g, float(alpha))
        if times[0] < t[0] - 1e-9 or times[-1] > t[-1] + 1e-9:
            raise F2RContractError(f"cycle {c} extends beyond the recorded kinematics [{t[0]}, {t[-1]}]")
        entry: dict[str, Any] = {"step": c.get("step")}
        for coord, key in zip(R.PROS_COORDS, ("knee", "ankle")):
            y = np.asarray(series[coord], dtype=np.float64)
            if y.shape != t.shape or not np.all(np.isfinite(y)):
                raise F2RContractError(f"{coord} series malformed")
            ys = np.interp(times, t, y)
            tr = A.tracking(np.asarray(reference[coord], dtype=np.float64), ys)
            entry[f"{key}_rmse"] = float(tr["rmse"])
            entry[f"{key}_r"] = tr["pearson_r"]
            if key == "ankle":
                entry["ankle_min_window"] = float(np.min(ys[win]))
        per_cycle.append(entry)
    return {
        "knee_rmse": _median_or_none([e["knee_rmse"] for e in per_cycle]),
        "knee_r": _median_or_none([e["knee_r"] for e in per_cycle]),
        "ankle_rmse": _median_or_none([e["ankle_rmse"] for e in per_cycle]),
        "ankle_min_window": _median_or_none([e["ankle_min_window"] for e in per_cycle]),
        "cycles_used": len(per_cycle),
        "window": [lo, hi],
        "per_cycle": per_cycle,
    }


# --- scalar metrics ---------------------------------------------------------------------------


def has_f1_receipt_fields(receipt: Mapping[str, Any]) -> bool:
    return all(k in receipt for k in F1_RECEIPT_FIELDS)


def scalar_metrics_from_outputs(job_dir: Path, summary: Mapping[str, Any], rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    """F0 scalar definitions re-applied to the job outputs (fallback when the receipt
    lacks the F1 provenance fields): kinematics min/max/rom from the kinematics STO,
    penetration series max from the online GRF STO, reserve norm max over every
    ``*_reserve_torque`` column, raw/applied action abs max from the trace."""
    job_dir = Path(job_dir)
    k_names, k_data = RA.read_sto(job_dir / "sim_outputs" / A.KINEMATICS_FILE)
    g_names, g_data = RA.read_sto(job_dir / "sim_outputs" / A.GRF_FILE)
    r_names, r_data = RA.read_sto(job_dir / "sim_outputs" / A.RESERVE_FILE)
    out: dict[str, Any] = {"route": "sto_fallback"}
    for coord, key in zip(R.PROS_COORDS, ("knee", "ankle")):
        y = RA.column(k_names, k_data, coord)
        if y is None or y.size == 0 or not np.all(np.isfinite(y)):
            raise F2RContractError(f"kinematics column {coord} missing or non-finite")
        out[f"{key}_min"], out[f"{key}_max"] = float(np.min(y)), float(np.max(y))
        out[f"{key}_rom"] = float(np.max(y) - np.min(y))
    pen = RA.column(g_names, g_data, "left_penetration")
    if pen is None or not np.all(np.isfinite(pen)):
        raise F2RContractError("left_penetration column missing or non-finite")
    out["penetration_max_m"] = float(np.max(pen))
    cols = [n for n in r_names if n.endswith(A.RESERVE_COLUMN_SUFFIX)]
    if not cols:
        raise F2RContractError("no *_reserve_torque column in the reserve file")
    block = r_data[:, [r_names.index(n) for n in cols]]
    if not np.all(np.isfinite(block)):
        raise F2RContractError("non-finite reserve torques")
    out["reserve_norm_max_nm"] = float(np.max(np.sqrt(np.sum(np.square(block), axis=1))))
    raw = np.asarray([r["raw_policy_action"] for r in rows], dtype=np.float64)
    applied = np.asarray([r["applied_policy_action"] for r in rows], dtype=np.float64)
    if not (np.all(np.isfinite(raw)) and np.all(np.isfinite(applied))):
        raise F2RContractError("non-finite actions in the trace")
    out["action_abs_max_raw"] = float(np.max(np.abs(raw)))
    out["action_abs_max_applied"] = float(np.max(np.abs(applied)))
    out["action_clipped_steps"] = A.as_int(summary["action_clipped_steps"])
    fsm = rows[-1].get("phase_fsm") if isinstance(rows[-1].get("phase_fsm"), Mapping) else None
    out["resync_count"] = A.counter(fsm, "resync_count")["value"]
    out["hs_cancelled_count"] = A.counter(fsm, "hs_cancelled_count")["value"]
    out["horizon_completed_f0"] = None
    return out


def scalar_metrics_f1(job_dir: Path, receipt: Mapping[str, Any], *, names35: Sequence[str]) -> dict[str, Any]:
    """Scalar metrics through the F0 analyser (``f1_analysis.f1_job_metrics``)."""
    m = FA.f1_job_metrics(Path(job_dir), dict(receipt), names35=list(names35))
    ka, fc = m["kinematics_actual"], m.get("fsm_counters", {})
    return {
        "route": "f1_job_metrics",
        "knee_min": float(ka["knee"]["min"]), "knee_max": float(ka["knee"]["max"]), "knee_rom": float(ka["knee"]["rom"]),
        "ankle_min": float(ka["ankle"]["min"]), "ankle_max": float(ka["ankle"]["max"]), "ankle_rom": float(ka["ankle"]["rom"]),
        "penetration_max_m": float(m["penetration"]["series_max_m"]),
        "reserve_norm_max_nm": float(m["reserve"]["series_norm_max_nm"]),
        "action_abs_max_raw": float(m["actions"]["raw_abs_max"]),
        "action_abs_max_applied": float(m["actions"]["applied_abs_max"]),
        "action_clipped_steps": int(m["actions"]["clipped_steps_summary"]),
        "resync_count": fc.get("resync_count", {}).get("value"),
        "hs_cancelled_count": fc.get("hs_cancelled_count", {}).get("value"),
        "horizon_completed_f0": bool(m["horizon"]["horizon_completed"]),
    }


# --- record extraction -----------------------------------------------------------------------


def start_of(summary: Mapping[str, Any], receipt: Mapping[str, Any]) -> str:
    """Start key from the exact episode start offset (receipt start, when present, must agree)."""
    offset = summary.get("episode_start_offset_s")
    if not _finite(offset):
        raise F2RContractError("summary without a numeric episode_start_offset_s")
    matches = [s for s, v in R.EXACT_STARTS.items() if repr(float(v)) == repr(float(offset))]
    if len(matches) != 1:
        raise F2RContractError(f"episode_start_offset_s {offset!r} is not one of the exact starts")
    start = matches[0]
    if receipt.get("start") is not None and receipt["start"] != start:
        raise F2RContractError(f"receipt start {receipt['start']!r} != summary start {start!r}")
    return start


def extract_record(job_dir: Path, receipt: Mapping[str, Any], *, names35: Sequence[str], heel_strikes: Sequence[float] | np.ndarray | None = None, offset: float = 0.0, protocol: Mapping[str, Any] | None = None, corridor: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """Flat gate record of one finished job directory (read-only; see module docstring).

    ``heel_strikes`` (prescribed right-side heel-strike times, e.g. the adapter
    summary ``reconstructor.clock_heel_strike_times_s``) enables ``drift_cycle_per_s``;
    without them the drift is None (which FAILS A6).  ``corridor`` may carry a
    cached ``corridor_reference()`` result."""
    job_dir = Path(job_dir)
    if job_dir.is_symlink() or not job_dir.is_dir():
        raise F2RContractError(f"job directory missing or symlink: {job_dir}")
    if not isinstance(receipt, Mapping):
        raise F2RContractError("receipt must be a mapping")
    columns = online_clock_columns(names35)
    thr = gate_thresholds(protocol)
    horizon_steps = int(thr["A"]["A4_horizon"]["steps"])
    window = tuple(float(x) for x in thr["B"]["B3_ankle_negative_window"]["phase_window"])
    ref = corridor if corridor is not None else corridor_reference()
    try:
        traj = DS.trajectory_from_job(job_dir, expected_width=R.ENV_ACTOR_WIDTH)
    except DS.DatasetError as exc:
        raise F2RContractError(f"job outputs malformed: {exc}") from exc
    summary = C.read_json(job_dir / DS.SUMMARY_FILE)
    rows = C.read_json(job_dir / DS.TRACE_FILE)
    steps = int(traj["steps"])
    start = start_of(summary, receipt)
    seed = int(traj["seed"])
    if receipt.get("seed") is not None and int(receipt["seed"]) != seed:
        raise F2RContractError(f"receipt seed {receipt['seed']} != summary action_seed {seed}")
    end_reason = str(traj["end_reason"])
    reset_time = float(traj["reset_time"])
    t_pre = np.asarray(traj["t_pre"], dtype=np.float64)
    obs = np.asarray(traj["obs35"], dtype=np.float64)
    last_fsm = rows[-1].get("phase_fsm") if isinstance(rows[-1].get("phase_fsm"), Mapping) else None
    if last_fsm is None or not A.is_nonneg_int(last_fsm.get("valid_cycle_count")):
        raise F2RContractError("last trace row without a phase_fsm.valid_cycle_count counter")
    for key in ("phase_valid_cycle_count", "phase_valid_hs_count", "phase_valid_to_count", "invalid_event_count"):
        if not A.is_nonneg_int(summary.get(key)):
            raise F2RContractError(f"summary counter {key} missing or not a non-negative integer")
    scalars = scalar_metrics_f1(job_dir, receipt, names35=names35) if has_f1_receipt_fields(receipt) else scalar_metrics_from_outputs(job_dir, summary, rows)
    horizon_completed = bool(end_reason == HORIZON_END_REASON and steps == horizon_steps)
    if scalars["horizon_completed_f0"] is not None and bool(scalars["horizon_completed_f0"]) != horizon_completed:
        raise F2RContractError(f"F0 horizon_completed {scalars['horizon_completed_f0']} != protocol horizon ({horizon_steps} steps)")
    # FSM chain and phase-aligned metrics
    cyc = OV.cycles_for_overlay(rows, summary_cycle_count=A.as_int(summary["phase_valid_cycle_count"]), trace_final_count=A.as_int(last_fsm["valid_cycle_count"]))
    cycles = list(cyc["cycles"])
    k_names, k_data = RA.read_sto(job_dir / "sim_outputs" / A.KINEMATICS_FILE)
    sto_time = RA.column(k_names, k_data, "time")
    series = {coord: RA.column(k_names, k_data, coord) for coord in R.PROS_COORDS}
    if sto_time is None or any(v is None for v in series.values()):
        raise F2RContractError("kinematics STO without time / prosthetic columns")
    phase = phase_aligned_metrics(cycles, sto_time, series, ref["reference"], alpha=float(ref["alpha"]), grid=ref["grid"], window=window) if cycles else {"knee_rmse": None, "knee_r": None, "ankle_rmse": None, "ankle_min_window": None, "cycles_used": 0, "window": list(window), "per_cycle": []}
    # online clock, first heel strike, drift
    clock = online_clock_diagnostics(obs, t_pre, reset_time, columns=columns)
    first_hs = first_accepted_event_time(rows, "heel_strike")
    drift = drift_cycle_per_s(obs, t_pre, heel_strikes, offset=offset, columns=columns) if heel_strikes is not None else {"slope": None, "rows": 0, "reason": "heel strikes not given"}
    causal = rows[-1].get("morphology_causal_diagnostics") if isinstance(rows[-1].get("morphology_causal_diagnostics"), Mapping) else None
    resolved = A.counter(causal, "total_resolved_sample_count")["value"]
    dropped = A.counter(causal, "total_dropped_sample_count")["value"]
    record: dict[str, Any] = {
        "record_schema": RECORD_SCHEMA,
        "job_id": str(receipt.get("job_id") or job_dir.name),
        "start": start,
        "seed": seed,
        "action_selection": str(traj["action_selection"]),
        "steps": steps,
        "end_reason": end_reason,
        "horizon_completed": horizon_completed,
        "phase_timeout": end_reason.startswith(PHASE_TIMEOUT_PREFIX),
        "valid_cycles": A.as_int(summary["phase_valid_cycle_count"]),
        "valid_hs": A.as_int(summary["phase_valid_hs_count"]),
        "valid_to": A.as_int(summary["phase_valid_to_count"]),
        "invalid_events": A.as_int(summary["invalid_event_count"]),
        "hs_cancelled_count": scalars["hs_cancelled_count"],
        "resync_count": scalars["resync_count"],
        "first_hs_delay_s": (first_hs - reset_time) if first_hs is not None else None,
        "online_clock_informative_s": clock["informative_s"],
        "online_periods_s": clock["periods_s"],
        "drift_cycle_per_s": drift["slope"],
        "knee_rom": scalars["knee_rom"], "knee_min": scalars["knee_min"], "knee_max": scalars["knee_max"],
        "ankle_min": scalars["ankle_min"], "ankle_max": scalars["ankle_max"],
        "penetration_max_m": scalars["penetration_max_m"],
        "reserve_norm_max_nm": scalars["reserve_norm_max_nm"],
        "action_abs_max_raw": scalars["action_abs_max_raw"],
        "action_clipped_steps": scalars["action_clipped_steps"],
        "stance_fraction": stance_fraction_of(cycles),
        "knee_rmse": phase["knee_rmse"], "knee_r": phase["knee_r"], "ankle_rmse": phase["ankle_rmse"], "ankle_min_window": phase["ankle_min_window"],
        "settled_fraction": (float(resolved) / steps) if resolved is not None else None,
        "dropped_fraction": (float(dropped) / steps) if dropped is not None else None,
        "source": {
            "job_dir": traj["job_dir"], "trace_sha256": traj["trace_sha256"], "summary_sha256": traj["summary_sha256"], "reset_sha256": traj["reset_sha256"],
            "reset_time_s": reset_time, "metrics_route": scalars["route"], "corridor": ref["profile"], "alpha": float(ref["alpha"]), "phase_grid_points": int(np.asarray(ref["grid"]).size),
            "fsm_cycles_extracted": len(cycles), "phase_aligned": cyc["phase_aligned"], "phase_contract_error": cyc.get("phase_contract_error"), "cycles": cycles, "per_cycle": phase["per_cycle"],
            "online_clock_first_index": clock["informative_index"], "drift": {k: v for k, v in drift.items() if k != "slope"}, "heel_strikes_given": heel_strikes is not None, "phase_offset": float(offset),
            "action_abs_max_applied": scalars["action_abs_max_applied"],
        },
    }
    missing = [k for k in RECORD_KEYS if k not in record]
    if missing:
        raise F2RContractError(f"record missing keys {missing}")
    return record


# --- gate primitives --------------------------------------------------------------------------


def validate_record(record: Any) -> dict[str, Any]:
    if not isinstance(record, Mapping):
        raise F2RContractError(f"record is {type(record).__name__}, expected mapping")
    missing = [k for k in RECORD_KEYS if k not in record]
    if missing:
        raise F2RContractError(f"record {record.get('job_id')!r} missing keys {missing}")
    if record["start"] not in R.STARTS:
        raise F2RContractError(f"record start {record['start']!r} not in {R.STARTS}")
    return dict(record)


def _by_start(records: Mapping[str, Any], *, seed: int, action_selection: str | None, label: str) -> dict[str, dict[str, Any]]:
    if not isinstance(records, Mapping):
        raise F2RContractError(f"{label}: records must be a mapping keyed by start")
    if set(records) != set(R.STARTS):
        raise F2RContractError(f"{label}: records keyed {sorted(records)} must be exactly the 3 starts {list(R.STARTS)}")
    out = {}
    for start in R.STARTS:
        rec = validate_record(records[start])
        if rec["start"] != start:
            raise F2RContractError(f"{label}: record at key {start!r} carries start {rec['start']!r}")
        if int(rec["seed"]) != int(seed):
            raise F2RContractError(f"{label}: record {rec['job_id']!r} has seed {rec['seed']}, required {seed}")
        if action_selection is not None and rec.get("action_selection") not in (None, action_selection):
            raise F2RContractError(f"{label}: record {rec['job_id']!r} action_selection {rec.get('action_selection')!r} != {action_selection!r}")
        out[start] = rec
    return out


def apply_rule(flags_by_start: Mapping[str, Any], rule: str) -> bool:
    """Aggregate per-start pass flags.  ``all3``: every start True; ``2of3``: at least
    two True; ``all3_and_2of3``: values are ``(flag_every_start, flag_two_starts)``
    pairs, pass iff every first flag is True AND >= 2 second flags are True (A2)."""
    if rule not in RULES:
        raise F2RContractError(f"unknown aggregation rule {rule!r}")
    if not isinstance(flags_by_start, Mapping) or set(flags_by_start) != set(R.STARTS):
        raise F2RContractError("flags must be keyed by exactly the 3 starts")
    if rule == "all3_and_2of3":
        pairs = []
        for s in R.STARTS:
            p = flags_by_start[s]
            if not (isinstance(p, (tuple, list)) and len(p) == 2 and all(isinstance(x, bool) for x in p)):
                raise F2RContractError(f"rule all3_and_2of3 needs (bool, bool) per start, got {p!r} at {s}")
            pairs.append((bool(p[0]), bool(p[1])))
        return all(p[0] for p in pairs) and sum(p[1] for p in pairs) >= 2
    flags = []
    for s in R.STARTS:
        f = flags_by_start[s]
        if not isinstance(f, bool):
            raise F2RContractError(f"flag at {s} must be a bool, got {f!r}")
        flags.append(f)
    return all(flags) if rule == "all3" else sum(flags) >= 2


def _check(rule: str, values_by_start: Mapping[str, Any], flags_by_start: Mapping[str, Any], threshold: Mapping[str, Any]) -> dict[str, Any]:
    return {"pass": apply_rule(flags_by_start, rule), "rule": rule, "values_by_start": dict(values_by_start), "flags_by_start": dict(flags_by_start), "threshold": dict(threshold)}


def _gate(name: str, checks: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    return {"gate": name, "pass": all(c["pass"] is True for c in checks.values()), "checks": dict(checks), "failed_checks": [k for k, c in checks.items() if c["pass"] is not True]}


# --- gates A / B / C (3 deterministic seed-123 records) ------------------------------------------


def gate_A(records: Mapping[str, Any], thr: Mapping[str, Any]) -> dict[str, Any]:
    """Structural gate: heel-strike onset, cycles, cancellations/resyncs, horizon,
    online clock onset / periods / drift, stance fraction (thresholds ``thr["A"]``)."""
    recs = _by_start(records, seed=R.DET_SEED, action_selection="deterministic", label="gate_A")
    a = thr["A"]
    checks: dict[str, dict[str, Any]] = {}
    t = a["A1_first_hs_delay_s"]
    checks["A1_first_hs_delay_s"] = _check(t["rule"], {s: r["first_hs_delay_s"] for s, r in recs.items()}, {s: _within(r["first_hs_delay_s"], t["min"], t["max"]) for s, r in recs.items()}, t)
    t = a["A2_cycles"]
    checks["A2_cycles"] = _check(t["rule"], {s: r["valid_cycles"] for s, r in recs.items()}, {s: (_ge(r["valid_cycles"], t["min_per_start"]), _ge(r["valid_cycles"], t["min_two_starts"])) for s, r in recs.items()}, t)
    t = a["A3_hs_cancelled_max"]
    checks["A3_hs_cancelled_max"] = _check(t["rule"], {s: {"hs_cancelled_count": r["hs_cancelled_count"], "resync_count": r["resync_count"]} for s, r in recs.items()}, {s: _le(r["hs_cancelled_count"], t["value"]) and _le(r["resync_count"], t["resync_max"]) for s, r in recs.items()}, t)
    t = a["A4_horizon"]
    checks["A4_horizon"] = _check(t["rule"], {s: {"steps": r["steps"], "end_reason": r["end_reason"], "horizon_completed": r["horizon_completed"], "phase_timeout": r["phase_timeout"]} for s, r in recs.items()}, {s: bool(r["horizon_completed"] is True and r["steps"] == int(t["steps"]) and r["end_reason"] == HORIZON_END_REASON and (not t.get("no_phase_timeout", True) or r["phase_timeout"] is False)) for s, r in recs.items()}, t)
    t = a["A5_online_clock_informative_within_s"]
    checks["A5_online_clock_informative_within_s"] = _check(t["rule"], {s: r["online_clock_informative_s"] for s, r in recs.items()}, {s: _le(r["online_clock_informative_s"], t["value"]) for s, r in recs.items()}, t)
    t = a["A6_online_period_s"]
    flags = {}
    for s, r in recs.items():
        periods = r["online_periods_s"]
        ok_periods = isinstance(periods, (list, tuple)) and len(periods) > 0 and all(_within(p, t["min"], t["max"]) for p in periods)
        d = _num(r["drift_cycle_per_s"])
        flags[s] = bool(ok_periods and d is not None and abs(d) <= float(t["drift_max_cycle_per_s"]))
    checks["A6_online_period_s"] = _check(t["rule"], {s: {"online_periods_s": r["online_periods_s"], "drift_cycle_per_s": r["drift_cycle_per_s"]} for s, r in recs.items()}, flags, t)
    t = a["A7_stance_fraction_min"]
    checks["A7_stance_fraction_min"] = _check(t["rule"], {s: r["stance_fraction"] for s, r in recs.items()}, {s: _ge(r["stance_fraction"], t["value"]) for s, r in recs.items()}, t)
    return _gate("A", checks)


def gate_B(records: Mapping[str, Any], thr: Mapping[str, Any]) -> dict[str, Any]:
    """Morphology gate vs the pinned corridor (thresholds ``thr["B"]``); a start without
    valid cycles carries None in the phase-aligned metrics and FAILS them."""
    recs = _by_start(records, seed=R.DET_SEED, action_selection="deterministic", label="gate_B")
    b = thr["B"]
    checks: dict[str, dict[str, Any]] = {}
    t = b["B1_knee_rom_min_rad"]
    checks["B1_knee_rom_min_rad"] = _check(t["rule"], {s: r["knee_rom"] for s, r in recs.items()}, {s: _ge(r["knee_rom"], t["value"]) for s, r in recs.items()}, t)
    t = b["B2_knee_vs_corridor"]
    checks["B2_knee_vs_corridor"] = _check(t["rule"], {s: {"knee_rmse": r["knee_rmse"], "knee_r": r["knee_r"]} for s, r in recs.items()}, {s: _le(r["knee_rmse"], t["rmse_max_rad"]) and _ge(r["knee_r"], t["pearson_min"]) for s, r in recs.items()}, t)
    t = b["B3_ankle_negative_window"]
    checks["B3_ankle_negative_window"] = _check(t["rule"], {s: r["ankle_min_window"] for s, r in recs.items()}, {s: _le(r["ankle_min_window"], t["min_max_rad"]) for s, r in recs.items()}, t)
    t = b["B4_ankle_rmse_max_rad"]
    checks["B4_ankle_rmse_max_rad"] = _check(t["rule"], {s: r["ankle_rmse"] for s, r in recs.items()}, {s: _le(r["ankle_rmse"], t["value"]) for s, r in recs.items()}, t)
    t = b["B5_stance_fraction"]
    checks["B5_stance_fraction"] = _check(t["rule"], {s: r["stance_fraction"] for s, r in recs.items()}, {s: _within(r["stance_fraction"], t["min"], t["max"]) for s, r in recs.items()}, t)
    t = b["B6_corridor_coverage"]
    checks["B6_corridor_coverage"] = _check(t["rule"], {s: {"settled_fraction": r["settled_fraction"], "dropped_fraction": r["dropped_fraction"]} for s, r in recs.items()}, {s: _ge(r["settled_fraction"], t["settled_fraction_min"]) and _le(r["dropped_fraction"], t["dropped_fraction_max"]) for s, r in recs.items()}, t)
    return _gate("B", checks)


def c123_flags(record: Mapping[str, Any], thr: Mapping[str, Any]) -> dict[str, bool]:
    """Per-job safety predicates C1 (penetration), C2 (no grf_penetration termination), C3 (reserve norm)."""
    c = thr["C"]
    return {
        "C1_penetration_max_m": _le(record["penetration_max_m"], c["C1_penetration_max_m"]["value"]),
        "C2_no_grf_penetration_termination": bool(isinstance(record["end_reason"], str) and record["end_reason"] != GRF_PENETRATION_END_REASON),
        "C3_reserve_norm_max_nm_per_start": _le(record["reserve_norm_max_nm"], c["C3_reserve_norm_max_nm_per_start"]["value"]),
    }


def gate_C(records: Mapping[str, Any], thr: Mapping[str, Any]) -> dict[str, Any]:
    """Safety gate (thresholds ``thr["C"]``; C3 per start, never a global aggregate)."""
    recs = _by_start(records, seed=R.DET_SEED, action_selection="deterministic", label="gate_C")
    c = thr["C"]
    checks: dict[str, dict[str, Any]] = {}
    t = c["C1_penetration_max_m"]
    checks["C1_penetration_max_m"] = _check(t["rule"], {s: r["penetration_max_m"] for s, r in recs.items()}, {s: c123_flags(r, thr)["C1_penetration_max_m"] for s, r in recs.items()}, t)
    t = c["C2_no_grf_penetration_termination"]
    checks["C2_no_grf_penetration_termination"] = _check(t["rule"], {s: r["end_reason"] for s, r in recs.items()}, {s: c123_flags(r, thr)["C2_no_grf_penetration_termination"] for s, r in recs.items()}, t)
    t = c["C3_reserve_norm_max_nm_per_start"]
    checks["C3_reserve_norm_max_nm_per_start"] = _check(t["rule"], {s: r["reserve_norm_max_nm"] for s, r in recs.items()}, {s: c123_flags(r, thr)["C3_reserve_norm_max_nm_per_start"] for s, r in recs.items()}, t)
    t = c["C4_clipped_steps_max"]
    checks["C4_clipped_steps_max"] = _check(t["rule"], {s: r["action_clipped_steps"] for s, r in recs.items()}, {s: _le(r["action_clipped_steps"], t["value"]) for s, r in recs.items()}, t)
    t = c["C5_action_abs_max"]
    checks["C5_action_abs_max"] = _check(t["rule"], {s: r["action_abs_max_raw"] for s, r in recs.items()}, {s: _le(r["action_abs_max_raw"], t["value"]) for s, r in recs.items()}, t)
    return _gate("C", checks)


# --- gate V (held-out seed 125) and gate R (9 stochastic jobs) --------------------------------------


def gate_V(records_125: Mapping[str, Any], thr: Mapping[str, Any]) -> dict[str, Any]:
    """Held-out promotion gate on the 3 seed-125 jobs: horizon at every start, >= 1 valid
    cycle at every start, no grf_penetration termination, C1-C3 on each job.  A record
    with another seed is a contract violation (125 is the only validation seed)."""
    v = thr["V"]
    recs = _by_start(records_125, seed=int(v["seed"]), action_selection="stochastic", label="gate_V")
    need = rule_fraction(v["rules"], "horizon", jobs=int(v["jobs"]))
    horizon = {s: bool(r["horizon_completed"] is True and r["end_reason"] == HORIZON_END_REASON) for s, r in recs.items()}
    cycles = {s: _ge(r["valid_cycles"], 1) for s, r in recs.items()}
    c123 = {s: c123_flags(r, thr) for s, r in recs.items()}
    checks = {
        "V_horizon": {"pass": sum(horizon.values()) >= need, "rule": f"{need}/{int(v['jobs'])}", "values_by_start": {s: {"steps": r["steps"], "end_reason": r["end_reason"]} for s, r in recs.items()}, "flags_by_start": horizon, "threshold": {"rule": v["rules"][0]}},
        "V_valid_cycle_each_start": {"pass": all(cycles.values()), "rule": "all3", "values_by_start": {s: r["valid_cycles"] for s, r in recs.items()}, "flags_by_start": cycles, "threshold": {"min_per_start": 1}},
        "V_no_grf_penetration": {"pass": all(c["C2_no_grf_penetration_termination"] for c in c123.values()), "rule": "all3", "values_by_start": {s: r["end_reason"] for s, r in recs.items()}, "flags_by_start": {s: c["C2_no_grf_penetration_termination"] for s, c in c123.items()}, "threshold": {"forbidden_end_reason": GRF_PENETRATION_END_REASON}},
        "V_C1_C3_each_job": {"pass": all(all(c.values()) for c in c123.values()), "rule": "all3", "values_by_start": {s: {"penetration_max_m": r["penetration_max_m"], "reserve_norm_max_nm": r["reserve_norm_max_nm"], "end_reason": r["end_reason"]} for s, r in recs.items()}, "flags_by_start": {s: all(c.values()) for s, c in c123.items()}, "threshold": {k: thr["C"][k].get("value") for k in ("C1_penetration_max_m", "C3_reserve_norm_max_nm_per_start")}},
    }
    out = _gate("V", checks)
    out["seed"] = int(v["seed"])
    out["semantics"] = v.get("semantics")
    return out


def gate_R(records: Iterable[Mapping[str, Any]], thr: Mapping[str, Any]) -> dict[str, Any]:
    """Robustness gate on the 9 stochastic jobs (seeds 123/124/125 x 3 starts, the 125
    jobs being those of V): horizon in >= k/9, >= 1 valid cycle in >= k/9, C1-C3 on each."""
    r_thr = thr["R"]
    recs = [validate_record(r) for r in (records.values() if isinstance(records, Mapping) else records)]
    jobs = int(r_thr["jobs"])
    if len(recs) != jobs:
        raise F2RContractError(f"gate_R needs exactly {jobs} records, got {len(recs)}")
    allowed = {int(s) for s in r_thr["seeds"]}
    pairs = [(r["start"], int(r["seed"])) for r in recs]
    if any(seed not in allowed for _, seed in pairs):
        raise F2RContractError(f"gate_R seeds must be in {sorted(allowed)}")
    if any(r.get("action_selection") not in (None, "stochastic") for r in recs):
        raise F2RContractError("gate_R records must be stochastic rollouts")
    expected = {(s, seed) for s in R.STARTS for seed in allowed}
    if set(pairs) != expected or len(set(pairs)) != len(pairs):
        raise F2RContractError("gate_R needs every (start, seed) pair exactly once")
    need_h = rule_fraction(r_thr["rules"], "horizon", jobs=jobs)
    need_c = rule_fraction(r_thr["rules"], "valid cycle", jobs=jobs)
    key = lambda r: f"{r['start']}_seed{r['seed']}"  # noqa: E731
    horizon = {key(r): bool(r["horizon_completed"] is True and r["end_reason"] == HORIZON_END_REASON) for r in recs}
    cycles = {key(r): _ge(r["valid_cycles"], 1) for r in recs}
    c123 = {key(r): all(c123_flags(r, thr).values()) for r in recs}
    checks = {
        "R_horizon": {"pass": sum(horizon.values()) >= need_h, "rule": f"{need_h}/{jobs}", "count": int(sum(horizon.values())), "flags_by_job": horizon, "threshold": {"min_jobs": need_h}},
        "R_valid_cycle": {"pass": sum(cycles.values()) >= need_c, "rule": f"{need_c}/{jobs}", "count": int(sum(cycles.values())), "flags_by_job": cycles, "threshold": {"min_jobs": need_c}},
        "R_C1_C3_each_job": {"pass": all(c123.values()), "rule": "all", "count": int(sum(c123.values())), "flags_by_job": c123, "threshold": {k: thr["C"][k].get("value") for k in ("C1_penetration_max_m", "C3_reserve_norm_max_nm_per_start")}},
    }
    out = _gate("R", checks)
    out["jobs"] = jobs
    out["seeds"] = sorted(allowed)
    return out


# --- round evaluation ---------------------------------------------------------------------------


COMMISSIONING_PHASES = ("T1", "T1R")


def evaluate_round(det_records: Mapping[str, Any], v_records: Mapping[str, Any] | None, protocol: Mapping[str, Any] | None = None, *, phase: str | None = None) -> dict[str, Any]:
    """Every gate is computed (no short circuit) for reporting; promotion to R requires
    A, B, C and V all PASS.  ``v_records=None`` (no seed-125 rollouts, e.g. T1) leaves V
    NOT_EVALUATED and blocks promotion.

    ``phase="T1R"`` (corrective commissioning, architect decision 2026-08-23): A is the only hard
    gate, B and C are computed as **informative**, V/R are NOT_EVALUATED, ``promotion`` is always
    False and ``next`` is a STOP; the explicit ``unlock_t2 = A.pass`` is the only product of the
    round; ``t3_trigger`` is never raised from T1R."""
    payload = R.load_protocol() if protocol is None else protocol
    thr = gate_thresholds(payload)
    result: dict[str, Any] = {"protocol_id": payload.get("protocol_id"), "A": gate_A(det_records, thr), "B": gate_B(det_records, thr), "C": gate_C(det_records, thr)}
    if v_records is None:
        result["V"] = {"gate": "V", "pass": False, "status": "NOT_EVALUATED", "reason": "no seed-125 validation records supplied", "checks": {}, "failed_checks": []}
    else:
        result["V"] = gate_V(v_records, thr)
        result["V"]["status"] = "EVALUATED"
    failed = [g for g in ("A", "B", "C", "V") if result[g]["pass"] is not True]
    result["failed_gates"] = failed
    if phase == "T1R":
        if v_records is not None:
            raise F2RContractError("T1R never has seed-125 validation records")
        result["B"]["informative"] = True; result["C"]["informative"] = True
        result["V"]["status"] = "NOT_EVALUATED"
        result["promotion"] = False
        result["unlock_t2"] = bool(result["A"]["pass"] is True)
        result["next"] = "STOP_audit_T2_unlocked_on_A_pass" if result["unlock_t2"] else "STOP_audit_T2_locked_A_failed"
        result["t3_trigger"] = False
        result["commissioning"] = "T1R: one corrective refit + 3 det seed-123 rollouts; STOP always; only A PASS unlocks T2; no aggregation; no T3 trigger"
        return result
    result["promotion"] = not failed
    result["next"] = NEXT_PROMOTE if not failed else NEXT_NOT_PROMOTED
    result["t3_trigger"] = t3_trigger(result)
    return result


def verify_t1r_unlock(*, gate_dir: Path | None = None, rollouts_dir: Path | None = None, refit_dir: Path | None = None) -> dict[str, Any]:
    """Fail-closed verification that T2 round 1 may start from the T1R student: exactly ONE
    ``gate_T1R_round_1_*.json`` in ``gate_dir`` with ``phase == "T1R"``, ``unlock_t2 is True`` and
    ``A.pass is True``; its three det records must reference, by trace/summary digest, the T1R job
    directories on disk (``STUDENT_T1R_r1__v3_canonical__<start>__det``), whose ``f2r.1`` receipts
    are ok, seed 123, deterministic and bound to the T1R student module (``module_state_sha256`` =
    digest of ``refit/T1R/round_1/rl_module_student/module_state.pkl``).  Returns
    ``{"unlocked": bool, "reason": str, ...}``; never raises on a locked state."""
    gate_dir = R.OUT_GATE if gate_dir is None else Path(gate_dir)
    rollouts_dir = R.OUT_ROLLOUTS if rollouts_dir is None else Path(rollouts_dir)
    refit_dir = R.OUT_REFIT if refit_dir is None else Path(refit_dir)
    out: dict[str, Any] = {"unlocked": False}
    gates = sorted(gate_dir.glob("gate_T1R_round_1_*.json")) if gate_dir.is_dir() else []
    if len(gates) != 1:
        out["reason"] = f"expected exactly one T1R gate artefact, found {len(gates)}"; out["gate_files"] = [C.rel(g) for g in gates]; return out
    gate_path = gates[0]
    try:
        g = C.read_json(gate_path)
    except Exception as exc:  # noqa: BLE001
        out["reason"] = f"T1R gate artefact unreadable: {type(exc).__name__}"; return out
    out["gate_file"] = C.rel(gate_path); out["gate_sha256"] = C.sha256_file(gate_path)
    if not (isinstance(g, Mapping) and g.get("phase") == "T1R" and g.get("round") == 1):
        out["reason"] = "gate artefact is not a T1R round-1 result"; return out
    if g.get("unlock_t2") is not True or not (isinstance(g.get("A"), Mapping) and g["A"].get("pass") is True):
        out["reason"] = f"T1R gate A did not pass (unlock_t2={g.get('unlock_t2')!r})"; return out
    module = refit_dir / "T1R" / "round_1" / "rl_module_student"; state = module / "module_state.pkl"
    if state.is_symlink() or not state.is_file():
        out["reason"] = "T1R student module missing"; return out
    module_sha = C.sha256_file(state)
    import f2r_matrix as MX

    records = g.get("records", {}).get("det", {}) if isinstance(g.get("records"), Mapping) else {}
    for start in R.STARTS:
        rec = records.get(start)
        job_dir = rollouts_dir / "T1R" / "round_1" / f"STUDENT_T1R_r1__v3_canonical__{start}__det"
        if not isinstance(rec, Mapping):
            out["reason"] = f"gate record missing for {start}"; return out
        trace, summary, receipt = job_dir / DS.TRACE_FILE, job_dir / DS.SUMMARY_FILE, job_dir / MX.RECEIPT_FILE
        if any(q.is_symlink() or not q.is_file() for q in (trace, summary, receipt)):
            out["reason"] = f"T1R job files missing for {start}"; return out
        src = rec.get("source", {}) if isinstance(rec.get("source"), Mapping) else {}
        if src.get("trace_sha256") != C.sha256_file(trace) or src.get("summary_sha256") != C.sha256_file(summary):
            out["reason"] = f"gate record digests differ from the T1R job on disk for {start}"; return out
        try:
            r = C.read_json(receipt)
        except Exception as exc:  # noqa: BLE001
            out["reason"] = f"T1R receipt unreadable for {start}: {type(exc).__name__}"; return out
        if not (r.get("status") == "ok" and r.get("returncode") == 0 and int(r.get("seed", -1)) == R.DET_SEED and r.get("action_selection") == "deterministic" and r.get("phase") == "T1R" and r.get("module_state_sha256") == module_sha and r.get("trace_sha256") == src.get("trace_sha256")):
            out["reason"] = f"T1R receipt inconsistent for {start}"; return out
    sys.path.insert(0, str(R.BASELINE_DIR)) if str(R.BASELINE_DIR) not in sys.path else None
    import warm_start as W

    out.update({"unlocked": True, "reason": "verified T1R gate with A PASS; records bound to the T1R jobs and student module", "student_module": str(module), "student_module_state_sha256": module_sha, "student_actor_digest": W.actor_state_digest(W.load_module_state(module))})
    return out


def t3_trigger(round_result: Mapping[str, Any]) -> bool:
    """Preregistered T3 trigger: gate A PASS and at least one of B3 / B2 / B5 FAIL."""
    a = round_result.get("A")
    b = round_result.get("B")
    if not isinstance(a, Mapping) or not isinstance(b, Mapping) or not isinstance(b.get("checks"), Mapping):
        raise F2RContractError("round result without gate A / B results")
    checks = b["checks"]
    missing = [k for k in T3_TRIGGER_CHECKS if k not in checks]
    if missing:
        raise F2RContractError(f"gate B result without checks {missing}")
    return bool(a.get("pass") is True and any(checks[k].get("pass") is not True for k in T3_TRIGGER_CHECKS))


# --- S1 round evaluation CLI (S0: plan only) ------------------------------------------------------


def prescribed_heel_strikes() -> list[float]:
    """Prescribed sound-side heel strikes from the pinned nominal anchor adapter summary (read-only, pin re-verified)."""
    anchor = R.ANCHORS["nominal"]
    path = Path(anchor["job_dir"]) / "f1_adapter_summary.json"
    if path.is_symlink() or not path.is_file() or C.sha256_file(path) != anchor["adapter_summary_sha256"]:
        raise F2RContractError(f"anchor adapter summary missing or not matching its pin: {path}")
    hs = C.read_json(path)["reconstructor"]["clock_heel_strike_times_s"]
    arr = np.asarray(hs, dtype=np.float64)
    if arr.ndim != 1 or arr.size < 2 or np.any(np.diff(arr) <= 0):
        raise F2RContractError("prescribed heel strikes malformed")
    return [float(x) for x in arr]


def evaluate_phase_round_s1(phase: str, round_index: int, *, out_dir: Path, rollouts_dir: Path | None = None) -> dict[str, Any]:
    """S1: extract the gate records of the det (123) and validation (125) jobs of ``phase``/``round``
    (receipts must be status ok), run ``evaluate_round`` and write a no-clobber JSON.  Stochastic
    collection jobs are reported for R only when all 9 stochastic jobs exist."""
    import f2r_matrix as MX

    rollouts_dir = R.OUT_ROLLOUTS if rollouts_dir is None else Path(rollouts_dir)
    hs = prescribed_heel_strikes()
    ref = corridor_reference()
    protocol = R.load_protocol()
    det: dict[str, Any] = {}
    val: dict[str, Any] = {}
    stoch: list[dict[str, Any]] = []
    for spec in MX.build_jobs():
        if spec["phase"] != phase or spec["round"] != round_index:
            continue
        job_dir = rollouts_dir / spec["phase"] / f"round_{spec['round']}" / spec["job_id"]
        receipt_path = job_dir / MX.RECEIPT_FILE
        if not receipt_path.is_file():
            if spec["purpose"] == "validation_gate" and phase in COMMISSIONING_PHASES:
                continue
            raise F2RContractError(f"missing receipt for {spec['job_id']}")
        receipt = C.read_json(receipt_path)
        if receipt.get("status") != "ok":
            raise F2RContractError(f"job {spec['job_id']} status {receipt.get('status')!r} != ok")
        rec = extract_record(job_dir, receipt, names35=list(R.FEATURE_NAMES_35), heel_strikes=hs, protocol=protocol, corridor=ref)
        if spec["purpose"] == "det":
            det[spec["start"]] = rec
        elif spec["purpose"] == "validation_gate":
            val[spec["start"]] = rec
            stoch.append(rec)
        else:
            stoch.append(rec)
    result = evaluate_round(det, val if val else None, protocol=protocol, phase=phase if phase == "T1R" else None)
    thr = gate_thresholds(protocol)
    result["R"] = gate_R(stoch, thr) if len(stoch) == int(thr["R"]["jobs"]) else {"gate": "R", "pass": False, "status": "NOT_EVALUATED", "reason": f"{len(stoch)} stochastic records (need {thr['R']['jobs']})"}
    result["phase"] = phase
    result["round"] = round_index
    result["records"] = {"det": det, "validation_gate": val, "stochastic_count": len(stoch)}
    result["heel_strikes_prescribed"] = hs
    out_dir = Path(out_dir)
    import time as _time

    stamp = _time.strftime("%Y%m%d_%H%M%S")
    path = R.unique_artifact_path(out_dir, f"gate_{phase}_round_{round_index}_{stamp}", ".json")  # exclusive, never reused
    R._atomic_fill(path, __import__("json").dumps(result, indent=2, default=str).encode("utf-8"))
    return {"result": C.rel(path), "sha256": C.sha256_file(path), "failed_gates": result["failed_gates"], "promotion": result["promotion"], "next": result["next"], "t3_trigger": result["t3_trigger"]}


def main(argv: Sequence[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description="F2R gate evaluation of one phase/round (S1 only; S0 prints the thresholds).")
    parser.add_argument("--phase", choices=("T1", "T1R", "T2", "T3"), default=None)
    parser.add_argument("--round", type=int, default=None)
    parser.add_argument("--out-dir", default=None)
    parser.add_argument("--authorized-stage", default=None)
    args = parser.parse_args(list(argv) if argv is not None else None)
    if args.phase is None:
        import json as _json

        print(_json.dumps(gate_thresholds(), indent=2))
        return 0
    if args.authorized_stage != "S1":
        raise SystemExit("gate evaluation reads student rollouts of S1+: not authorised in S0; pass --authorized-stage S1 after the architect's go")
    if args.round is None or not args.out_dir:
        raise SystemExit("--round and --out-dir are required")
    import json as _json

    print(_json.dumps(evaluate_phase_round_s1(args.phase, int(args.round), out_dir=Path(args.out_dir)), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
