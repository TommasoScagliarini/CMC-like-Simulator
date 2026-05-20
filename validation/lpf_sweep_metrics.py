"""Compute summary metrics for the LPF-on-u sweep.

Loads each sweep run, the IK reference, and reports per-fc metrics on the
2.5 s screening window (kinematic tracking, motor dynamics, saturations,
reserves). Pure read-only.

Usage:
    python validation/lpf_sweep_metrics.py
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

# Add repo root to sys.path so we can import output.read_sto without cwd
# assumptions.
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from output import read_sto  # noqa: E402

SWEEP_TAGS = [
    ("fcoff", "off"),
    ("fc200", "200"),
    ("fc100", "100"),
    ("fc50", "50"),
    ("fc25", "25"),
    ("fc15", "15"),
    ("fc10", "10"),
]
RESULTS_ROOT = ROOT / "results"
IK_PATH = ROOT / "models" / "AB06_SEASEA_Threadmill" / "data" / "IK_results_AB06_SEASEA.mot"


def _interp_ref_deg(t_query: np.ndarray, col: str) -> np.ndarray:
    times, cols, data, in_deg = read_sto(str(IK_PATH))
    if col not in cols:
        raise RuntimeError(f"Reference column {col!r} missing from IK file")
    idx = cols.index(col)
    series = data[:, idx]
    if not in_deg:
        series = np.degrees(series)
    return np.interp(t_query, times, series)


def _column(data: np.ndarray, cols: list[str], name: str) -> np.ndarray:
    if name not in cols:
        raise RuntimeError(f"Missing column {name!r}")
    return data[:, cols.index(name)]


def _rms(x: np.ndarray) -> float:
    finite = np.isfinite(x)
    if not finite.any():
        return float("nan")
    return float(np.sqrt(np.mean(x[finite] ** 2)))


def _max_abs(x: np.ndarray) -> float:
    finite = np.isfinite(x)
    if not finite.any():
        return float("nan")
    return float(np.max(np.abs(x[finite])))


def collect_one(out_dir: Path) -> dict | None:
    status_path = out_dir / "sim_output_run_status.txt"
    if not status_path.exists():
        return {"out_dir": out_dir.name, "status": "missing"}
    status = {}
    for line in status_path.read_text().splitlines():
        if "=" in line:
            k, _, v = line.partition("=")
            status[k.strip()] = v.strip()
    if status.get("status") != "complete":
        return {
            "out_dir": out_dir.name,
            "status": status.get("status", "unknown"),
            "t_end": float(status.get("t", "nan")),
            "error": status.get("error", ""),
        }

    kin_t, kin_cols, kin_data, _ = read_sto(str(out_dir / "sim_output_kinematics.sto"))
    diag_t, diag_cols, diag_data, _ = read_sto(str(out_dir / "sim_output_sea_diagnostics.sto"))
    res_t, res_cols, res_data, _ = read_sto(str(out_dir / "sim_output_reserve_torques.sto"))

    knee_q_sim = np.degrees(_column(kin_data, kin_cols, "pros_knee_angle"))
    ankle_q_sim = np.degrees(_column(kin_data, kin_cols, "pros_ankle_angle"))
    knee_q_ref = _interp_ref_deg(kin_t, "pros_knee_angle")
    ankle_q_ref = _interp_ref_deg(kin_t, "pros_ankle_angle")

    knee_err = knee_q_sim - knee_q_ref
    ankle_err = ankle_q_sim - ankle_q_ref

    knee_tau_err = _column(diag_data, diag_cols, "SEA_Knee_tau_error")
    ankle_tau_err = _column(diag_data, diag_cols, "SEA_Ankle_tau_error")
    knee_mdot = _column(diag_data, diag_cols, "SEA_Knee_motor_speed_dot_plugin")
    ankle_mdot = _column(diag_data, diag_cols, "SEA_Ankle_motor_speed_dot_plugin")
    knee_sat = _column(diag_data, diag_cols, "SEA_Knee_tau_input_saturated")
    ankle_sat = _column(diag_data, diag_cols, "SEA_Ankle_tau_input_saturated")
    knee_tau_input = _column(diag_data, diag_cols, "SEA_Knee_tau_input_plugin")
    ankle_tau_input = _column(diag_data, diag_cols, "SEA_Ankle_tau_input_plugin")

    reserve_norm = np.linalg.norm(res_data, axis=1)

    return {
        "out_dir": out_dir.name,
        "status": "complete",
        "samples": int(len(kin_t)),
        "knee_rmse_deg": _rms(knee_err),
        "ankle_rmse_deg": _rms(ankle_err),
        "mean_rmse_deg": 0.5 * (_rms(knee_err) + _rms(ankle_err)),
        "knee_max_err_deg": _max_abs(knee_err),
        "ankle_max_err_deg": _max_abs(ankle_err),
        "knee_tau_err_rms": _rms(knee_tau_err),
        "ankle_tau_err_rms": _rms(ankle_tau_err),
        "knee_motor_speed_dot_rms": _rms(knee_mdot),
        "ankle_motor_speed_dot_rms": _rms(ankle_mdot),
        "knee_motor_speed_dot_max": _max_abs(knee_mdot),
        "ankle_motor_speed_dot_max": _max_abs(ankle_mdot),
        "knee_sat_count": int(np.nansum(knee_sat)),
        "ankle_sat_count": int(np.nansum(ankle_sat)),
        "knee_tau_input_max": _max_abs(knee_tau_input),
        "ankle_tau_input_max": _max_abs(ankle_tau_input),
        "reserve_norm_rms": _rms(reserve_norm),
        "reserve_norm_max": _max_abs(reserve_norm),
    }


def main() -> int:
    rows = []
    for tag, fc_label in SWEEP_TAGS:
        out_dir = RESULTS_ROOT / f"_lpf_sweep_25s_{tag}_20260515"
        row = collect_one(out_dir)
        if row is None:
            print(f"{fc_label:>4}: NOT_FOUND")
            continue
        row["fc_hz"] = fc_label
        rows.append(row)

    if not rows:
        print("No runs found.")
        return 1

    completed = [r for r in rows if r.get("status") == "complete"]
    failed = [r for r in rows if r.get("status") != "complete"]

    fmt_header = (
        "fc   | knee RMSE | ankle RMSE | mean RMSE | knee tau_err | ankle tau_err |"
        " knee mdot RMS | ankle mdot RMS | knee_sat | ankle_sat | res RMS"
    )
    print(fmt_header)
    print("-" * len(fmt_header))
    for r in completed:
        print(
            f"{r['fc_hz']:>4} | {r['knee_rmse_deg']:>9.4f} | {r['ankle_rmse_deg']:>10.4f} |"
            f" {r['mean_rmse_deg']:>9.4f} | {r['knee_tau_err_rms']:>12.4f} |"
            f" {r['ankle_tau_err_rms']:>13.4f} | {r['knee_motor_speed_dot_rms']:>13.2f} |"
            f" {r['ankle_motor_speed_dot_rms']:>14.2f} | {r['knee_sat_count']:>8d} |"
            f" {r['ankle_sat_count']:>9d} | {r['reserve_norm_rms']:>7.2f}"
        )
    if failed:
        print()
        print("Failed/missing:")
        for r in failed:
            print(f"  {r.get('fc_hz', '?'):>4}: {r}")

    out_json = RESULTS_ROOT / "_lpf_sweep_log" / "metrics_20260515.json"
    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(json.dumps(rows, indent=2))
    print(f"\nSummary written to {out_json.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
