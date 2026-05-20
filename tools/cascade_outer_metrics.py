#!/usr/bin/env python3
"""Compare PD, PI, and cascade prosthetic-controller runs."""

from __future__ import annotations

import csv
import math
import os
import sys
import tempfile
from pathlib import Path
from typing import Dict, List, Optional, Tuple

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

os.environ.setdefault(
    "MPLCONFIGDIR",
    str(Path(tempfile.gettempdir()) / "cmc_like_matplotlib"),
)
os.environ.setdefault(
    "XDG_CACHE_HOME",
    str(Path(tempfile.gettempdir()) / "cmc_like_cache"),
)

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import butter, sosfiltfilt, welch

from config import SimulatorConfig
from kinematics_interpolator import KinematicsInterpolator
from output import read_sto


OUT_DIR = REPO_ROOT / "plot" / "05_17_2026_cascade_outer_comparison"
RUNS = [
    ("pd", "PD baseline", REPO_ROOT / "results" / "_fast_inner_pid_20260516_rerun_20260517"),
    ("pi", "PI inner", REPO_ROOT / "results" / "_pi_inner_bandmatched_full_20260517"),
    ("cascade_conservative", "Cascade conservative", REPO_ROOT / "results" / "_cascade_conservative_full_20260517"),
    ("cascade_balanced", "Cascade balanced", REPO_ROOT / "results" / "_cascade_balanced_full_20260517"),
    ("cascade_aggressive", "Cascade aggressive", REPO_ROOT / "results" / "_cascade_aggressive_full_20260517"),
]
COORDS = [
    ("pros_knee_angle", "SEA_Knee", "knee"),
    ("pros_ankle_angle", "SEA_Ankle", "ankle"),
]


class Table:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.time, self.columns, self.data, self.in_degrees = read_sto(str(path))
        self.index = {name: i for i, name in enumerate(self.columns)}

    def col(self, *names: str) -> Optional[np.ndarray]:
        for name in names:
            idx = self.index.get(name)
            if idx is not None:
                return self.data[:, idx]
        return None


def load_table(run_dir: Path, suffix: str) -> Optional[Table]:
    path = run_dir / f"sim_output_{suffix}.sto"
    if not path.is_file():
        return None
    return Table(path)


def status_text(run_dir: Path) -> str:
    path = run_dir / "sim_output_run_status.txt"
    if not path.is_file():
        return "missing"
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if line.startswith("status="):
            return line.split("=", 1)[1].strip()
    return "unknown"


def rms(values: np.ndarray) -> float:
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if values.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(values * values)))


def fraction(mask: np.ndarray) -> float:
    mask = np.asarray(mask)
    if mask.size == 0:
        return float("nan")
    return float(np.mean(mask))


def highpass_rms(time: np.ndarray, values: np.ndarray, cutoff_hz: float = 50.0) -> float:
    valid = np.isfinite(time) & np.isfinite(values)
    time = time[valid]
    values = values[valid]
    if values.size < 20:
        return float("nan")
    dt = float(np.median(np.diff(time)))
    if not math.isfinite(dt) or dt <= 0.0:
        return float("nan")
    fs = 1.0 / dt
    if cutoff_hz >= 0.45 * fs:
        return float("nan")
    sos = butter(4, cutoff_hz, btype="highpass", fs=fs, output="sos")
    filtered = sosfiltfilt(sos, values)
    return rms(filtered)


def reference_values(times: np.ndarray) -> Dict[str, np.ndarray]:
    cfg = SimulatorConfig()
    cfg.model_bundle_dir = str(REPO_ROOT / "models" / "AB06_SEASEA_Threadmill")
    cfg.model_file = str(
        REPO_ROOT
        / "models"
        / "AB06_SEASEA_Threadmill"
        / "AB06_SEASEA_stiff321_500_pi.osim"
    )
    cfg.kinematics_file = str(
        REPO_ROOT
        / "models"
        / "AB06_SEASEA_Threadmill"
        / "data"
        / "IK_results_AB06_SEASEA.mot"
    )
    cfg.t_start = float(np.nanmin(times))
    cfg.t_end = float(np.nanmax(times))
    kin = KinematicsInterpolator(cfg)
    refs: Dict[str, List[float]] = {coord: [] for coord, _sea, _key in COORDS}
    for t in times:
        q_ref, _qdot_ref, _qddot_ref = kin.get(float(t))
        for coord, _sea, _key in COORDS:
            refs[coord].append(float(q_ref[coord]))
    return {coord: np.asarray(values) for coord, values in refs.items()}


def run_metrics(run_id: str, label: str, run_dir: Path, refs: Dict[str, np.ndarray]) -> Dict[str, float | str]:
    states = load_table(run_dir, "states")
    controls = load_table(run_dir, "sea_controls")
    diag = load_table(run_dir, "sea_diagnostics")
    reserves = load_table(run_dir, "reserve_torques")
    if states is None or controls is None or diag is None:
        return {"run_id": run_id, "label": label, "status": "missing_outputs"}

    metrics: Dict[str, float | str] = {
        "run_id": run_id,
        "label": label,
        "status": status_text(run_dir),
        "n_samples": float(states.time.size),
    }

    rmse_values = []
    tau_error_rms_values = []
    motor_speed_dot_rms_values = []
    motor_speed_dot_hpf_values = []
    saturation_count = 0.0
    u_high_count = 0.0
    u_total_count = 0.0
    cascade_clamp_values = []
    cascade_aw_values = []
    cascade_i_abs_max_values = []
    cascade_xi_abs_max_values = []

    for coord, sea, key in COORDS:
        q = states.col(f"{coord}_q")
        ref = refs[coord]
        if q is not None and q.size == ref.size:
            err = q - ref
            metrics[f"rmse_{key}_rad"] = rms(err)
            rmse_values.append(metrics[f"rmse_{key}_rad"])

        tau_error = diag.col(f"{sea}_tau_error")
        if tau_error is not None:
            value = rms(tau_error)
            metrics[f"tau_error_{key}_rms_nm"] = value
            tau_error_rms_values.append(value)

        motor_speed_dot = diag.col(f"{sea}_motor_speed_dot_plugin")
        if motor_speed_dot is not None:
            value = rms(motor_speed_dot)
            metrics[f"motor_speed_dot_{key}_rms"] = value
            motor_speed_dot_rms_values.append(value)
            hpf_value = highpass_rms(diag.time, motor_speed_dot)
            metrics[f"motor_speed_dot_{key}_hpf50_rms"] = hpf_value
            motor_speed_dot_hpf_values.append(hpf_value)

        saturation = diag.col(f"{sea}_tau_input_saturated")
        if saturation is not None:
            saturation_count += float(np.nansum(saturation > 0.5))

        u = controls.col(coord)
        if u is not None:
            u_high_count += float(np.nansum(np.abs(u) > 0.95))
            u_total_count += float(np.sum(np.isfinite(u)))
            metrics[f"u_{key}_gt_095_fraction"] = fraction(np.abs(u[np.isfinite(u)]) > 0.95)

        cascade_i = diag.col(f"{sea}_cascade_inner_i_cmd")
        if cascade_i is not None and np.isfinite(cascade_i).any():
            cascade_i_abs_max_values.append(float(np.nanmax(np.abs(cascade_i))))
        cascade_xi = diag.col(f"{sea}_cascade_xi_v")
        if cascade_xi is not None and np.isfinite(cascade_xi).any():
            cascade_xi_abs_max_values.append(float(np.nanmax(np.abs(cascade_xi))))
        cascade_clamp = diag.col(f"{sea}_cascade_i_clamped")
        if cascade_clamp is not None and np.isfinite(cascade_clamp).any():
            cascade_clamp_values.append(fraction(cascade_clamp > 0.5))
        cascade_aw = diag.col(f"{sea}_cascade_anti_windup_active")
        if cascade_aw is not None and np.isfinite(cascade_aw).any():
            cascade_aw_values.append(fraction(cascade_aw > 0.5))

    if rmse_values:
        metrics["rmse_mean_rad"] = float(np.nanmean(rmse_values))
    if tau_error_rms_values:
        metrics["tau_error_mean_rms_nm"] = float(np.nanmean(tau_error_rms_values))
    if motor_speed_dot_rms_values:
        metrics["motor_speed_dot_mean_rms"] = float(np.nanmean(motor_speed_dot_rms_values))
    if motor_speed_dot_hpf_values:
        metrics["motor_speed_dot_hpf50_mean_rms"] = float(np.nanmean(motor_speed_dot_hpf_values))
    metrics["tau_input_saturation_count"] = saturation_count
    metrics["u_gt_095_fraction"] = u_high_count / u_total_count if u_total_count else float("nan")

    if reserves is not None and reserves.data.size:
        reserve_norm = np.linalg.norm(reserves.data, axis=1)
        metrics["reserve_norm_rms_nm"] = rms(reserve_norm)
        metrics["reserve_norm_max_nm"] = float(np.nanmax(reserve_norm))

    if cascade_i_abs_max_values:
        metrics["cascade_i_abs_max_nm"] = float(np.nanmax(cascade_i_abs_max_values))
    if cascade_xi_abs_max_values:
        metrics["cascade_xi_abs_max"] = float(np.nanmax(cascade_xi_abs_max_values))
    if cascade_clamp_values:
        metrics["cascade_clamp_fraction"] = float(np.nanmean(cascade_clamp_values))
    if cascade_aw_values:
        metrics["cascade_anti_windup_fraction"] = float(np.nanmean(cascade_aw_values))
    metrics["finite_outputs"] = str(
        all(np.isfinite(table.data).all() for table in [states, controls, diag])
    )
    return metrics


def write_csv(rows: List[Dict[str, float | str]], path: Path) -> None:
    keys = sorted({key for row in rows for key in row.keys()})
    preferred = [
        "run_id",
        "label",
        "status",
        "finite_outputs",
        "rmse_knee_rad",
        "rmse_ankle_rad",
        "rmse_mean_rad",
        "tau_error_mean_rms_nm",
        "motor_speed_dot_mean_rms",
        "motor_speed_dot_hpf50_mean_rms",
        "tau_input_saturation_count",
        "u_gt_095_fraction",
        "reserve_norm_rms_nm",
        "reserve_norm_max_nm",
        "cascade_i_abs_max_nm",
        "cascade_xi_abs_max",
        "cascade_clamp_fraction",
        "cascade_anti_windup_fraction",
    ]
    fieldnames = preferred + [key for key in keys if key not in preferred]
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def bar_plot(rows: List[Dict[str, float | str]], path: Path) -> None:
    labels = [str(row["run_id"]) for row in rows]
    metrics = [
        ("rmse_mean_rad", "Mean prosthetic RMSE [rad]"),
        ("tau_error_mean_rms_nm", "Tau error RMS [Nm]"),
        ("motor_speed_dot_hpf50_mean_rms", "Motor speed dot HPF >50 Hz RMS"),
        ("u_gt_095_fraction", "|u| > 0.95 fraction"),
        ("reserve_norm_rms_nm", "Reserve norm RMS [Nm]"),
        ("tau_input_saturation_count", "Tau input saturation count"),
    ]
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    for ax, (key, title) in zip(axes.ravel(), metrics):
        values = [float(row.get(key, np.nan)) for row in rows]
        ax.bar(labels, values, color=["0.45", "tab:blue", "tab:green", "tab:orange", "tab:red"])
        ax.set_title(title)
        ax.tick_params(axis="x", labelrotation=25)
        ax.grid(True, axis="y", alpha=0.25)
    fig.suptitle("PD vs PI vs Cascade Outer Controller Metrics")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def comparison_time_plot(path: Path) -> None:
    fig, axes = plt.subplots(5, 1, figsize=(14, 13), sharex=True)
    for run_id, label, run_dir in RUNS:
        states = load_table(run_dir, "states")
        controls = load_table(run_dir, "sea_controls")
        diag = load_table(run_dir, "sea_diagnostics")
        reserves = load_table(run_dir, "reserve_torques")
        if states is None or controls is None or diag is None:
            continue
        refs = reference_values(states.time)
        errs = []
        for coord, _sea, _key in COORDS:
            q = states.col(f"{coord}_q")
            if q is not None:
                errs.append(q - refs[coord])
        if errs:
            axes[0].plot(states.time, np.sqrt(np.mean(np.vstack(errs) ** 2, axis=0)), label=label)
        tau_errors = []
        motor_speed_dots = []
        for _coord, sea, _key in COORDS:
            tau = diag.col(f"{sea}_tau_error")
            msd = diag.col(f"{sea}_motor_speed_dot_plugin")
            if tau is not None:
                tau_errors.append(tau)
            if msd is not None:
                motor_speed_dots.append(msd)
        if tau_errors:
            axes[1].plot(diag.time, np.sqrt(np.mean(np.vstack(tau_errors) ** 2, axis=0)), label=label)
        if motor_speed_dots:
            axes[2].plot(diag.time, np.sqrt(np.mean(np.vstack(motor_speed_dots) ** 2, axis=0)), label=label)
        us = []
        for coord, _sea, _key in COORDS:
            u = controls.col(coord)
            if u is not None:
                us.append(np.abs(u))
        if us:
            axes[3].plot(controls.time, np.max(np.vstack(us), axis=0), label=label)
        if reserves is not None:
            axes[4].plot(reserves.time, np.linalg.norm(reserves.data, axis=1), label=label)
    ylabels = [
        "prosthetic RMS error [rad]",
        "tau_error RMS across SEA [Nm]",
        "motor_speed_dot RMS across SEA",
        "max |u|",
        "reserve torque norm [Nm]",
    ]
    for ax, ylabel in zip(axes, ylabels):
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.25)
    axes[-1].set_xlabel("time [s]")
    axes[0].legend(loc="best", fontsize=8)
    fig.suptitle("PD vs PI vs Cascade Time-Domain Comparison")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def cascade_integral_plot(path: Path) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    for run_id, label, run_dir in RUNS:
        if not run_id.startswith("cascade"):
            continue
        diag = load_table(run_dir, "sea_diagnostics")
        if diag is None:
            continue
        for ax, (_coord, sea, key) in zip(axes, COORDS):
            value = diag.col(f"{sea}_cascade_inner_i_cmd")
            if value is not None:
                ax.plot(diag.time, value, label=label)
            ax.set_title(key)
            ax.set_ylabel("Ki_inner * xi_v [Nm]")
            ax.grid(True, alpha=0.25)
    axes[-1].set_xlabel("time [s]")
    axes[0].legend(loc="best", fontsize=8)
    fig.suptitle("Cascade Integral Torque")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def spectrum_plot(path: Path) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    for run_id, label, run_dir in RUNS:
        diag = load_table(run_dir, "sea_diagnostics")
        if diag is None:
            continue
        dt = float(np.median(np.diff(diag.time)))
        fs = 1.0 / dt
        for ax, (_coord, sea, key) in zip(axes, COORDS):
            value = diag.col(f"{sea}_motor_speed_dot_plugin")
            if value is None:
                continue
            valid = np.isfinite(value)
            if np.sum(valid) < 20:
                continue
            freq, psd = welch(value[valid], fs=fs, nperseg=min(2048, np.sum(valid)))
            keep = freq >= 10.0
            ax.semilogy(freq[keep], psd[keep], label=label)
            ax.axvline(50.0, color="0.2", linestyle="--", linewidth=0.8)
            ax.set_title(key)
            ax.set_ylabel("PSD motor_speed_dot")
            ax.grid(True, alpha=0.25)
    axes[-1].set_xlabel("frequency [Hz]")
    axes[0].legend(loc="best", fontsize=8)
    fig.suptitle("Motor Speed Dot Spectrum")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def write_summary(rows: List[Dict[str, float | str]], path: Path) -> None:
    pi = next(row for row in rows if row["run_id"] == "pi")
    pi_hpf = float(pi.get("motor_speed_dot_hpf50_mean_rms", np.nan))
    candidates = []
    for row in rows:
        run_id = str(row["run_id"])
        if not run_id.startswith("cascade"):
            continue
        complete = row.get("status") == "complete"
        finite = row.get("finite_outputs") == "True"
        u_ok = float(row.get("u_gt_095_fraction", np.nan)) <= 0.05
        hpf = float(row.get("motor_speed_dot_hpf50_mean_rms", np.nan))
        hpf_ok = bool(np.isfinite(hpf) and np.isfinite(pi_hpf) and hpf <= 1.1 * pi_hpf)
        rmse = float(row.get("rmse_mean_rad", np.nan))
        accepted = complete and finite and u_ok and hpf_ok and np.isfinite(rmse)
        candidates.append((accepted, rmse, run_id, row))

    accepted = [item for item in candidates if item[0]]
    selected = "none"
    if accepted:
        accepted.sort(key=lambda item: item[1])
        best = accepted[0]
        close = [
            item for item in accepted
            if item[1] <= best[1] * 1.02
        ]
        order = {
            "cascade_conservative": 0,
            "cascade_balanced": 1,
            "cascade_aggressive": 2,
        }
        close.sort(key=lambda item: order.get(item[2], 99))
        selected = close[0][2]

    lines = [
        "# Cascade Outer Comparison Summary",
        "",
        f"Selected candidate: {selected}",
        "",
        "Selection filters: complete, finite, HPF chattering <= 110% of PI, |u|>0.95 fraction <= 5%.",
        "",
        "| run | status | RMSE mean rad | tau_error RMS Nm | HPF50 RMS | |u|>0.95 | reserve RMS Nm | sat count | cascade clamp |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in rows:
        lines.append(
            "| {run_id} | {status} | {rmse:.6g} | {tau:.6g} | {hpf:.6g} | {u:.4%} | {reserve:.6g} | {sat:.0f} | {clamp:.4%} |".format(
                run_id=row.get("run_id", ""),
                status=row.get("status", ""),
                rmse=float(row.get("rmse_mean_rad", np.nan)),
                tau=float(row.get("tau_error_mean_rms_nm", np.nan)),
                hpf=float(row.get("motor_speed_dot_hpf50_mean_rms", np.nan)),
                u=float(row.get("u_gt_095_fraction", np.nan)),
                reserve=float(row.get("reserve_norm_rms_nm", np.nan)),
                sat=float(row.get("tau_input_saturation_count", np.nan)),
                clamp=float(row.get("cascade_clamp_fraction", np.nan)),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    first_states = load_table(RUNS[0][2], "states")
    if first_states is None:
        raise FileNotFoundError("Could not load first run states table")
    refs = reference_values(first_states.time)
    rows = [run_metrics(run_id, label, run_dir, refs) for run_id, label, run_dir in RUNS]
    write_csv(rows, OUT_DIR / "metrics.csv")
    write_summary(rows, OUT_DIR / "summary.md")
    bar_plot(rows, OUT_DIR / "01_metric_bars.png")
    comparison_time_plot(OUT_DIR / "02_time_comparison.png")
    cascade_integral_plot(OUT_DIR / "03_cascade_integral_torque.png")
    spectrum_plot(OUT_DIR / "04_motor_speed_dot_spectrum.png")
    print(OUT_DIR)
    print((OUT_DIR / "summary.md").read_text(encoding="utf-8"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
