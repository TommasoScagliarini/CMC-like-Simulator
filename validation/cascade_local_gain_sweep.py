#!/usr/bin/env python3
"""
cascade_local_gain_sweep.py
===========================

Local gain sweep for the AB06 prosthetic cascade controller.

The sweep is centered on the current cascade gains and ranks candidates by:

1. prosthetic kinematic tracking, ankle-weighted;
2. high-frequency chatter in joint/motor velocity and tau_input;
3. motor power.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import platform
import shlex
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Sequence

import numpy as np
from scipy.signal import butter, sosfiltfilt

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from output import read_sto  # noqa: E402


SEA_KNEE = "SEA_Knee"
SEA_ANKLE = "SEA_Ankle"
COORD_KNEE = "pros_knee_angle"
COORD_ANKLE = "pros_ankle_angle"

BASE_KNEE_KP_OUTER = 18.85
BASE_KNEE_KP_INNER = 29.2
BASE_KNEE_KI_INNER = 1377.0
BASE_KNEE_I_LIMIT = 50.0

BASE_ANKLE_KP_OUTER = 37.7
BASE_ANKLE_KP_INNER = 3.77
BASE_ANKLE_KI_INNER = 355.0
BASE_ANKLE_I_LIMIT = 200.0

KP_OUTER_MULTIPLIERS = [0.75, 0.90, 1.00, 1.10, 1.25]
KP_INNER_MULTIPLIERS = [0.75, 0.90, 1.00, 1.10, 1.25]
KI_INNER_MULTIPLIERS = [0.60, 0.80, 1.00, 1.20, 1.50]

SCREEN_WINDOWS = (
    (13.1638, 14.7799),
    (17.9528, 19.5259),
)
QUICK_SMOKE_WINDOW = (13.1638, 13.1938)
FULL_WINDOW = (11.99, 21.0)

DEFAULT_SETUP = (
    "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml"
)
DEFAULT_MODEL = "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim"
DEFAULT_REFERENCE = "models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot"
DEFAULT_BASELINE_DIR = "results"

CHATTER_FIELDS = [
    "knee_qdot_hpf50_rms",
    "ankle_qdot_hpf50_rms",
    "knee_motor_speed_hpf50_rms",
    "ankle_motor_speed_hpf50_rms",
    "knee_motor_speed_dot_hpf50_rms",
    "ankle_motor_speed_dot_hpf50_rms",
    "knee_tau_input_plugin_hpf50_rms",
    "ankle_tau_input_plugin_hpf50_rms",
]
POWER_FIELDS = [
    "knee_motor_power_rms",
    "ankle_motor_power_rms",
    "knee_motor_power_mean_abs",
    "ankle_motor_power_mean_abs",
    "knee_motor_power_peak_abs",
    "ankle_motor_power_peak_abs",
]


@dataclass(frozen=True)
class Candidate:
    run_id: str
    knee_kp_outer: float
    knee_kp_inner: float
    knee_ki_inner: float
    knee_i_limit: float
    ankle_kp_outer: float
    ankle_kp_inner: float
    ankle_ki_inner: float
    ankle_i_limit: float


BASE_CANDIDATE = Candidate(
    run_id="baseline_cascade_current",
    knee_kp_outer=BASE_KNEE_KP_OUTER,
    knee_kp_inner=BASE_KNEE_KP_INNER,
    knee_ki_inner=BASE_KNEE_KI_INNER,
    knee_i_limit=BASE_KNEE_I_LIMIT,
    ankle_kp_outer=BASE_ANKLE_KP_OUTER,
    ankle_kp_inner=BASE_ANKLE_KP_INNER,
    ankle_ki_inner=BASE_ANKLE_KI_INNER,
    ankle_i_limit=BASE_ANKLE_I_LIMIT,
)


def default_workers() -> int:
    return 12 if platform.system().lower().startswith("win") else 6


def fmt_num(value: float) -> str:
    return f"{value:g}".replace(".", "p").replace("-", "m")


def fmt_xml(value: float) -> str:
    return f"{value:.12g}"


def resolve(path: str | Path) -> Path:
    p = Path(path)
    return p if p.is_absolute() else REPO_ROOT / p


def relpath(path: Path) -> str:
    try:
        return str(path.relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def parse_grid(raw: str | None, default: Sequence[float]) -> List[float]:
    if raw is None or not raw.strip():
        return [float(v) for v in default]
    return [float(item.strip()) for item in raw.split(",") if item.strip()]


def scaled_grid(base: float, multipliers: Sequence[float]) -> List[float]:
    values = [float(base) * float(mult) for mult in multipliers]
    return sorted(set(round(value, 12) for value in values))


def subprocess_env() -> Dict[str, str]:
    env = os.environ.copy()
    env.update({
        "OMP_NUM_THREADS": "1",
        "MKL_NUM_THREADS": "1",
        "OPENBLAS_NUM_THREADS": "1",
        "NUMEXPR_NUM_THREADS": "1",
        "PYTHONIOENCODING": "utf-8",
    })
    return env


def format_command(cmd: Sequence[str]) -> str:
    if os.name == "nt":
        return subprocess.list2cmdline(list(cmd))
    return shlex.join(cmd)


def format_duration(seconds: float) -> str:
    seconds = max(0.0, float(seconds))
    hours = int(seconds // 3600)
    minutes = int((seconds % 3600) // 60)
    secs = seconds - hours * 3600 - minutes * 60
    if hours:
        return f"{hours:d}h{minutes:02d}m{secs:04.1f}s"
    if minutes:
        return f"{minutes:d}m{secs:04.1f}s"
    return f"{secs:.1f}s"


class ProgressTracker:
    def __init__(self, total_jobs: int, workers: int) -> None:
        self.total_jobs = max(1, int(total_jobs))
        self.workers = max(1, int(workers))
        self.started_at = time.monotonic()
        self._elapsed_samples: List[float] = []

    def update(self, job_elapsed_s: float) -> None:
        if math.isfinite(job_elapsed_s) and job_elapsed_s > 0.0:
            self._elapsed_samples.append(float(job_elapsed_s))
            keep = max(24, self.workers * 4)
            if len(self._elapsed_samples) > keep:
                self._elapsed_samples = self._elapsed_samples[-keep:]

    def estimate_eta_s(self, done_jobs: int) -> float:
        done = max(0, min(int(done_jobs), self.total_jobs))
        remaining = self.total_jobs - done
        if remaining <= 0:
            return 0.0
        if not self._elapsed_samples:
            return math.inf
        avg = sum(self._elapsed_samples) / len(self._elapsed_samples)
        return avg * remaining / max(1, self.workers)

    def overall_text(self, done_jobs: int) -> str:
        done = max(0, min(int(done_jobs), self.total_jobs))
        pct = 100.0 * done / self.total_jobs
        elapsed = time.monotonic() - self.started_at
        eta_s = self.estimate_eta_s(done)
        eta = "n/a" if not math.isfinite(eta_s) else format_duration(eta_s)
        return (
            f"overall={done}/{self.total_jobs} ({pct:.2f}%) "
            f"total_elapsed={format_duration(elapsed)} total_eta={eta}"
        )

    def line(self, done_jobs: int, label: str, run_id: str, status: str) -> str:
        done = max(0, min(int(done_jobs), self.total_jobs))
        pct = 100.0 * done / self.total_jobs
        elapsed = time.monotonic() - self.started_at
        eta_s = self.estimate_eta_s(done)
        eta = "n/a" if not math.isfinite(eta_s) else format_duration(eta_s)
        return (
            f"[{label}] {pct:6.2f}% jobs={done}/{self.total_jobs} "
            f"elapsed={format_duration(elapsed)} eta={eta} "
            f"last={status} {run_id}"
        )


@dataclass
class OverallProgress:
    tracker: ProgressTracker
    done_jobs: int = 0

    def update(self, job_elapsed_s: float) -> None:
        self.done_jobs += 1
        self.tracker.update(job_elapsed_s)

    def suffix(self) -> str:
        return self.tracker.overall_text(self.done_jobs)


def candidate_base_row(candidate: Candidate) -> Dict[str, object]:
    return {
        "run_id": candidate.run_id,
        "knee_kp_outer": candidate.knee_kp_outer,
        "knee_kp_inner": candidate.knee_kp_inner,
        "knee_ki_inner": candidate.knee_ki_inner,
        "knee_i_limit": candidate.knee_i_limit,
        "ankle_kp_outer": candidate.ankle_kp_outer,
        "ankle_kp_inner": candidate.ankle_kp_inner,
        "ankle_ki_inner": candidate.ankle_ki_inner,
        "ankle_i_limit": candidate.ankle_i_limit,
    }


def make_run_id(prefix: str, candidate: Candidate) -> str:
    return (
        f"{prefix}_kpo{fmt_num(candidate.knee_kp_outer)}"
        f"_kpi{fmt_num(candidate.knee_kp_inner)}"
        f"_kii{fmt_num(candidate.knee_ki_inner)}"
        f"_kil{fmt_num(candidate.knee_i_limit)}"
        f"_apo{fmt_num(candidate.ankle_kp_outer)}"
        f"_api{fmt_num(candidate.ankle_kp_inner)}"
        f"_aii{fmt_num(candidate.ankle_ki_inner)}"
        f"_ail{fmt_num(candidate.ankle_i_limit)}"
    )


def candidate_with_id(prefix: str, candidate: Candidate) -> Candidate:
    return Candidate(
        run_id=make_run_id(prefix, candidate),
        knee_kp_outer=candidate.knee_kp_outer,
        knee_kp_inner=candidate.knee_kp_inner,
        knee_ki_inner=candidate.knee_ki_inner,
        knee_i_limit=candidate.knee_i_limit,
        ankle_kp_outer=candidate.ankle_kp_outer,
        ankle_kp_inner=candidate.ankle_kp_inner,
        ankle_ki_inner=candidate.ankle_ki_inner,
        ankle_i_limit=candidate.ankle_i_limit,
    )


def generate_knee_candidates(
    kp_outer_grid: Sequence[float],
    kp_inner_grid: Sequence[float],
    ki_inner_grid: Sequence[float],
    i_limit_grid: Sequence[float],
) -> List[Candidate]:
    candidates: List[Candidate] = []
    for kp_outer in kp_outer_grid:
        for kp_inner in kp_inner_grid:
            for ki_inner in ki_inner_grid:
                for i_limit in i_limit_grid:
                    candidates.append(candidate_with_id(
                        "knee",
                        Candidate(
                            run_id="",
                            knee_kp_outer=float(kp_outer),
                            knee_kp_inner=float(kp_inner),
                            knee_ki_inner=float(ki_inner),
                            knee_i_limit=float(i_limit),
                            ankle_kp_outer=BASE_ANKLE_KP_OUTER,
                            ankle_kp_inner=BASE_ANKLE_KP_INNER,
                            ankle_ki_inner=BASE_ANKLE_KI_INNER,
                            ankle_i_limit=BASE_ANKLE_I_LIMIT,
                        ),
                    ))
    return candidates


def generate_ankle_candidates(
    kp_outer_grid: Sequence[float],
    kp_inner_grid: Sequence[float],
    ki_inner_grid: Sequence[float],
    i_limit_grid: Sequence[float],
) -> List[Candidate]:
    candidates: List[Candidate] = []
    for kp_outer in kp_outer_grid:
        for kp_inner in kp_inner_grid:
            for ki_inner in ki_inner_grid:
                for i_limit in i_limit_grid:
                    candidates.append(candidate_with_id(
                        "ankle",
                        Candidate(
                            run_id="",
                            knee_kp_outer=BASE_KNEE_KP_OUTER,
                            knee_kp_inner=BASE_KNEE_KP_INNER,
                            knee_ki_inner=BASE_KNEE_KI_INNER,
                            knee_i_limit=BASE_KNEE_I_LIMIT,
                            ankle_kp_outer=float(kp_outer),
                            ankle_kp_inner=float(kp_inner),
                            ankle_ki_inner=float(ki_inner),
                            ankle_i_limit=float(i_limit),
                        ),
                    ))
    return candidates


def combine_candidates(
    knee_rows: Sequence[Dict[str, object]],
    ankle_rows: Sequence[Dict[str, object]],
) -> List[Candidate]:
    candidates: List[Candidate] = []
    for kr in knee_rows:
        for ar in ankle_rows:
            base = Candidate(
                run_id="",
                knee_kp_outer=float(kr["knee_kp_outer"]),
                knee_kp_inner=float(kr["knee_kp_inner"]),
                knee_ki_inner=float(kr["knee_ki_inner"]),
                knee_i_limit=float(kr["knee_i_limit"]),
                ankle_kp_outer=float(ar["ankle_kp_outer"]),
                ankle_kp_inner=float(ar["ankle_kp_inner"]),
                ankle_ki_inner=float(ar["ankle_ki_inner"]),
                ankle_i_limit=float(ar["ankle_i_limit"]),
            )
            candidates.append(candidate_with_id("combo", base))
    return candidates


def command_for_run(
    python_exe: str,
    setup_path: Path,
    model_path: Path,
    results_dir: Path,
    candidate: Candidate,
    t_start: float,
    t_end: float,
) -> List[str]:
    return [
        python_exe,
        str(REPO_ROOT / "main.py"),
        "--setup", str(setup_path),
        "--model", str(model_path),
        "--t-start", fmt_xml(t_start),
        "--t-end", fmt_xml(t_end),
        "--output-dir", str(results_dir),
        "--filter-grf",
        "--sea-outer-controller", "cascade",
        "--sea-cascade-kp-outer-knee", fmt_xml(candidate.knee_kp_outer),
        "--sea-cascade-kp-inner-knee", fmt_xml(candidate.knee_kp_inner),
        "--sea-cascade-ki-inner-knee", fmt_xml(candidate.knee_ki_inner),
        "--sea-cascade-inner-i-torque-limit-knee", fmt_xml(candidate.knee_i_limit),
        "--sea-cascade-kp-outer-ankle", fmt_xml(candidate.ankle_kp_outer),
        "--sea-cascade-kp-inner-ankle", fmt_xml(candidate.ankle_kp_inner),
        "--sea-cascade-ki-inner-ankle", fmt_xml(candidate.ankle_ki_inner),
        "--sea-cascade-inner-i-torque-limit-ankle", fmt_xml(candidate.ankle_i_limit),
    ]


def timeout_for_window(t_start: float, t_end: float, minimum: float) -> float:
    return max(float(minimum), 180.0 * max(0.01, float(t_end) - float(t_start)))


def run_simulation(
    python_exe: str,
    setup_path: Path,
    model_path: Path,
    results_dir: Path,
    candidate: Candidate,
    t_start: float,
    t_end: float,
    timeout_s: float,
) -> int:
    results_dir.mkdir(parents=True, exist_ok=True)
    cmd = command_for_run(
        python_exe, setup_path, model_path, results_dir, candidate, t_start, t_end
    )
    console = results_dir / "console.txt"
    with console.open("w", encoding="utf-8", errors="replace") as fh:
        fh.write("Command:\n" + format_command(cmd) + "\n\n")
        fh.write(f"Timeout: {timeout_s:.1f}s\n\n")
        fh.flush()
        try:
            completed = subprocess.run(
                cmd,
                cwd=str(REPO_ROOT),
                stdout=fh,
                stderr=subprocess.STDOUT,
                env=subprocess_env(),
                check=False,
                timeout=timeout_s,
            )
            return int(completed.returncode)
        except subprocess.TimeoutExpired:
            fh.write(f"\n[Sweep] TIMEOUT after {timeout_s:.1f}s\n")
            return 124


def load_run_status(results_dir: Path) -> Dict[str, str]:
    path = results_dir / "sim_output_run_status.txt"
    if not path.is_file():
        return {}
    values: Dict[str, str] = {}
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if "=" in raw:
            key, value = raw.split("=", 1)
            values[key.strip()] = value.strip()
    return values


def finite_sto_outputs(results_dir: Path) -> bool:
    paths = list(results_dir.glob("sim_output_*.sto"))
    if not paths:
        return False
    for path in paths:
        try:
            _time, _cols, data, _in_degrees = read_sto(str(path))
        except Exception:
            return False
        if not np.all(np.isfinite(data)):
            return False
    return True


def load_column(path: Path, column: str) -> tuple[np.ndarray, np.ndarray, bool]:
    times, cols, data, in_degrees = read_sto(str(path))
    if column not in cols:
        raise KeyError(f"Missing column {column} in {path}")
    return (
        np.asarray(times, dtype=float),
        np.asarray(data[:, cols.index(column)], dtype=float),
        bool(in_degrees),
    )


def unique_time_series(
    time_values: np.ndarray,
    values: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    unique_time, indices = np.unique(time_values, return_index=True)
    order = np.argsort(indices)
    selected = indices[order]
    return time_values[selected], values[selected]


def tracking_stats(results_dir: Path, reference_path: Path, coord: str) -> Dict[str, float]:
    sim_time, sim, sim_deg = load_column(results_dir / "sim_output_kinematics.sto", coord)
    ref_time, ref, ref_deg = load_column(reference_path, coord)
    if sim_deg:
        sim = np.deg2rad(sim)
    if ref_deg:
        ref = np.deg2rad(ref)
    ref_time, ref = unique_time_series(ref_time, ref)
    ref_i = np.interp(sim_time, ref_time, ref)
    err_deg = np.rad2deg(sim - ref_i)
    finite = np.isfinite(err_deg)
    if not np.any(finite):
        raise ValueError(f"No finite tracking samples for {coord}")
    err_deg = err_deg[finite]
    return {
        "n_samples": int(err_deg.size),
        "sse_deg2": float(np.sum(err_deg * err_deg)),
        "rms_deg": float(np.sqrt(np.mean(err_deg * err_deg))),
        "mean_abs_deg": float(np.mean(np.abs(err_deg))),
        "max_abs_deg": float(np.max(np.abs(err_deg))),
    }


def signal_stats(values: np.ndarray) -> Dict[str, float]:
    finite = np.asarray(values, dtype=float)
    finite = finite[np.isfinite(finite)]
    if finite.size == 0:
        return {"rms": math.inf, "mean_abs": math.inf, "max_abs": math.inf}
    return {
        "rms": float(np.sqrt(np.mean(finite * finite))),
        "mean_abs": float(np.mean(np.abs(finite))),
        "max_abs": float(np.max(np.abs(finite))),
    }


def hpf_rms(time_values: np.ndarray, values: np.ndarray, cutoff_hz: float = 50.0) -> float:
    valid = np.isfinite(time_values) & np.isfinite(values)
    t = np.asarray(time_values, dtype=float)[valid]
    y = np.asarray(values, dtype=float)[valid]
    if y.size < 20:
        return float("nan")
    dt = float(np.median(np.diff(t)))
    if not math.isfinite(dt) or dt <= 0.0:
        return float("nan")
    fs = 1.0 / dt
    if cutoff_hz >= 0.45 * fs:
        return float("nan")
    try:
        sos = butter(4, cutoff_hz, btype="highpass", fs=fs, output="sos")
        filtered = sosfiltfilt(sos, y)
    except ValueError:
        return float("nan")
    return float(np.sqrt(np.mean(filtered * filtered)))


def integrate_positive_negative(time_values: np.ndarray, power: np.ndarray) -> Dict[str, float]:
    valid = np.isfinite(time_values) & np.isfinite(power)
    t = np.asarray(time_values, dtype=float)[valid]
    p = np.asarray(power, dtype=float)[valid]
    if t.size < 2:
        return {
            "positive_energy_j": float("nan"),
            "negative_energy_j": float("nan"),
            "abs_energy_j": float("nan"),
        }
    positive = np.maximum(p, 0.0)
    negative = np.minimum(p, 0.0)
    return {
        "positive_energy_j": float(np.trapezoid(positive, t)),
        "negative_energy_j": float(np.trapezoid(negative, t)),
        "abs_energy_j": float(np.trapezoid(np.abs(p), t)),
    }


def collect_metrics(
    candidate: Candidate,
    stage: str,
    results_dir: Path,
    reference_path: Path,
    return_code: int,
    elapsed_s: float,
    timeout_s: float,
    window_idx: int | None = None,
) -> Dict[str, object]:
    row = candidate_base_row(candidate)
    row.update({
        "stage": stage,
        "window_idx": "" if window_idx is None else window_idx,
        "return_code": return_code,
        "timeout_s": timeout_s,
        "elapsed_s": elapsed_s,
        "results_dir": relpath(results_dir),
    })

    status = load_run_status(results_dir)
    row["run_status"] = status.get("status", "missing")
    row["complete"] = status.get("status") == "complete"
    row["finite_outputs"] = finite_sto_outputs(results_dir) if row["complete"] else False
    row["timeout"] = return_code == 124

    if row["timeout"]:
        row["fail_reason"] = "timeout"
        row["acceptable"] = False
        row["unstable"] = True
        return row
    if not row["complete"]:
        row["fail_reason"] = "incomplete"
        row["acceptable"] = False
        row["unstable"] = True
        return row
    if not row["finite_outputs"]:
        row["fail_reason"] = "nonfinite_outputs"
        row["acceptable"] = False
        row["unstable"] = True
        return row

    try:
        for label, coord in (("knee", COORD_KNEE), ("ankle", COORD_ANKLE)):
            stats = tracking_stats(results_dir, reference_path, coord)
            row[f"{label}_tracking_n"] = stats["n_samples"]
            row[f"{label}_tracking_sse_deg2"] = stats["sse_deg2"]
            row[f"{label}_tracking_rms_deg"] = stats["rms_deg"]
            row[f"{label}_tracking_mean_deg"] = stats["mean_abs_deg"]
            row[f"{label}_tracking_max_deg"] = stats["max_abs_deg"]

        row["mean_rmse_deg"] = 0.5 * (
            float(row["knee_tracking_rms_deg"])
            + float(row["ankle_tracking_rms_deg"])
        )
        row["worst_rmse_deg"] = max(
            float(row["knee_tracking_rms_deg"]),
            float(row["ankle_tracking_rms_deg"]),
        )
        row["max_abs_error_deg"] = max(
            float(row["knee_tracking_max_deg"]),
            float(row["ankle_tracking_max_deg"]),
        )

        states_path = results_dir / "sim_output_states.sto"
        diagnostics_path = results_dir / "sim_output_sea_diagnostics.sto"
        controls_path = results_dir / "sim_output_sea_controls.sto"
        power_path = results_dir / "sim_output_power.sto"

        sat_count = 0
        max_u = 0.0
        max_tau_input_raw_abs = 0.0
        max_tau_input_plugin_abs = 0.0

        for label, sea_name, coord in (
            ("knee", SEA_KNEE, COORD_KNEE),
            ("ankle", SEA_ANKLE, COORD_ANKLE),
        ):
            qdot_t, qdot, _ = load_column(states_path, f"{coord}_qdot")
            row[f"{label}_qdot_hpf50_rms"] = hpf_rms(qdot_t, qdot)
            qdot_stats = signal_stats(qdot)
            row[f"{label}_qdot_rms"] = qdot_stats["rms"]
            row[f"{label}_qdot_max_abs"] = qdot_stats["max_abs"]

            diag_t, motor_speed, _ = load_column(
                diagnostics_path, f"{sea_name}_motor_speed"
            )
            motor_speed_stats = signal_stats(motor_speed)
            row[f"{label}_motor_speed_rms"] = motor_speed_stats["rms"]
            row[f"{label}_motor_speed_max_abs"] = motor_speed_stats["max_abs"]
            row[f"{label}_motor_speed_hpf50_rms"] = hpf_rms(diag_t, motor_speed)

            _diag_t, motor_speed_dot, _ = load_column(
                diagnostics_path, f"{sea_name}_motor_speed_dot_plugin"
            )
            motor_speed_dot_stats = signal_stats(motor_speed_dot)
            row[f"{label}_motor_speed_dot_rms"] = motor_speed_dot_stats["rms"]
            row[f"{label}_motor_speed_dot_max_abs"] = motor_speed_dot_stats["max_abs"]
            row[f"{label}_motor_speed_dot_hpf50_rms"] = hpf_rms(
                diag_t, motor_speed_dot
            )

            _diag_t, tau_input_raw, _ = load_column(
                diagnostics_path, f"{sea_name}_tau_input_raw"
            )
            raw_stats = signal_stats(tau_input_raw)
            row[f"{label}_tau_input_raw_rms"] = raw_stats["rms"]
            row[f"{label}_tau_input_raw_mean_abs"] = raw_stats["mean_abs"]
            row[f"{label}_tau_input_raw_max_abs"] = raw_stats["max_abs"]
            max_tau_input_raw_abs = max(max_tau_input_raw_abs, raw_stats["max_abs"])

            _diag_t, tau_input_plugin, _ = load_column(
                diagnostics_path, f"{sea_name}_tau_input_plugin"
            )
            plugin_stats = signal_stats(tau_input_plugin)
            row[f"{label}_tau_input_plugin_rms"] = plugin_stats["rms"]
            row[f"{label}_tau_input_plugin_mean_abs"] = plugin_stats["mean_abs"]
            row[f"{label}_tau_input_plugin_max_abs"] = plugin_stats["max_abs"]
            row[f"{label}_tau_input_plugin_hpf50_rms"] = hpf_rms(
                diag_t, tau_input_plugin
            )
            max_tau_input_plugin_abs = max(
                max_tau_input_plugin_abs, plugin_stats["max_abs"]
            )

            _diag_t, tau_error, _ = load_column(diagnostics_path, f"{sea_name}_tau_error")
            tau_error_stats = signal_stats(tau_error)
            row[f"{label}_tau_error_rms"] = tau_error_stats["rms"]
            row[f"{label}_tau_error_max_abs"] = tau_error_stats["max_abs"]

            _diag_t, saturated, _ = load_column(
                diagnostics_path, f"{sea_name}_tau_input_saturated"
            )
            saturated_count = int(np.nansum(saturated > 0.5))
            row[f"{label}_tau_input_saturation_count"] = saturated_count
            sat_count += saturated_count

            control_t, u, _ = load_column(controls_path, coord)
            u_abs = np.abs(u[np.isfinite(u)])
            row[f"{label}_max_u"] = float(np.max(u_abs)) if u_abs.size else math.inf
            row[f"{label}_frac_u_gt_095"] = (
                float(np.mean(u_abs > 0.95)) if u_abs.size else math.inf
            )
            max_u = max(max_u, float(row[f"{label}_max_u"]))
            del control_t

            power_t, motor_power, _ = load_column(power_path, f"{sea_name}_motor_power")
            power_stats = signal_stats(motor_power)
            energy = integrate_positive_negative(power_t, motor_power)
            row[f"{label}_motor_power_rms"] = power_stats["rms"]
            row[f"{label}_motor_power_mean_abs"] = power_stats["mean_abs"]
            row[f"{label}_motor_power_peak_abs"] = power_stats["max_abs"]
            row[f"{label}_motor_positive_energy_j"] = energy["positive_energy_j"]
            row[f"{label}_motor_negative_energy_j"] = energy["negative_energy_j"]
            row[f"{label}_motor_abs_energy_j"] = energy["abs_energy_j"]

        row["sat_count"] = sat_count
        row["max_u"] = max_u
        row["max_tau_input_raw_abs"] = max_tau_input_raw_abs
        row["max_tau_input_plugin_abs"] = max_tau_input_plugin_abs

        fail_reasons: List[str] = []
        if return_code != 0:
            fail_reasons.append(f"return_code_{return_code}")
        if sat_count > 0:
            fail_reasons.append("tau_input_saturated")
        if max_u > 0.99:
            fail_reasons.append("max_u_gt_0p99")
        if max_tau_input_raw_abs > 500.0:
            fail_reasons.append("tau_input_raw_gt_500")
        if max_tau_input_plugin_abs > 500.0:
            fail_reasons.append("tau_input_plugin_gt_500")

        row["fail_reason"] = ";".join(fail_reasons)
        row["unstable"] = bool(fail_reasons)
        row["acceptable"] = not fail_reasons
    except Exception as exc:
        row["fail_reason"] = f"metric_error:{type(exc).__name__}:{exc}"
        row["acceptable"] = False
        row["unstable"] = True
    return row


def finite_float(value: object) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number


def normalized_mean(
    row: Dict[str, object],
    fields: Sequence[str],
    baseline: Dict[str, object] | None,
) -> float:
    values: List[float] = []
    for field in fields:
        value = finite_float(row.get(field))
        if value is None:
            continue
        if baseline is not None:
            base = finite_float(baseline.get(field))
            if base is None or abs(base) <= 1e-12:
                continue
            values.append(value / abs(base))
        else:
            values.append(value)
    return float(np.mean(values)) if values else math.inf


def add_scores(row: Dict[str, object], baseline: Dict[str, object] | None) -> None:
    knee = finite_float(row.get("knee_tracking_rms_deg"))
    ankle = finite_float(row.get("ankle_tracking_rms_deg"))
    if knee is not None and ankle is not None:
        row["score_kinematic_deg"] = 0.3 * knee + 0.7 * ankle
    row["score_chattering_norm"] = normalized_mean(row, CHATTER_FIELDS, baseline)
    row["score_power_norm"] = normalized_mean(row, POWER_FIELDS, baseline)


def aggregate_window_rows(
    candidate: Candidate,
    stage: str,
    rows: Sequence[Dict[str, object]],
    baseline: Dict[str, object] | None,
) -> Dict[str, object]:
    combined = candidate_base_row(candidate)
    combined["stage"] = stage
    combined["window_idx"] = "combined"
    combined["results_dir"] = ";".join(str(row.get("results_dir", "")) for row in rows)
    combined["return_code"] = max(int(row.get("return_code", 0) or 0) for row in rows)
    combined["timeout_s"] = max(float(row.get("timeout_s", 0.0) or 0.0) for row in rows)
    combined["elapsed_s"] = sum(float(row.get("elapsed_s", 0.0) or 0.0) for row in rows)
    combined["complete"] = all(bool(row.get("complete")) for row in rows)
    combined["finite_outputs"] = all(bool(row.get("finite_outputs")) for row in rows)
    combined["timeout"] = any(bool(row.get("timeout")) for row in rows)
    combined["run_status"] = "complete" if combined["complete"] else "incomplete"
    combined["fail_reason"] = ";".join(
        str(row.get("fail_reason", "")) for row in rows if row.get("fail_reason")
    )

    if not combined["complete"] or not combined["finite_outputs"] or combined["timeout"]:
        combined["acceptable"] = False
        combined["unstable"] = True
        add_scores(combined, baseline)
        return combined

    for prefix in ("knee", "ankle"):
        n = sum(int(row.get(f"{prefix}_tracking_n", 0) or 0) for row in rows)
        sse = sum(float(row.get(f"{prefix}_tracking_sse_deg2", 0.0) or 0.0) for row in rows)
        max_err = max(float(row.get(f"{prefix}_tracking_max_deg", 0.0) or 0.0) for row in rows)
        mean_values = [
            float(row.get(f"{prefix}_tracking_mean_deg", math.nan))
            for row in rows
            if row.get(f"{prefix}_tracking_mean_deg", "") != ""
        ]
        combined[f"{prefix}_tracking_n"] = n
        combined[f"{prefix}_tracking_sse_deg2"] = sse
        combined[f"{prefix}_tracking_rms_deg"] = math.sqrt(sse / n) if n else math.inf
        combined[f"{prefix}_tracking_mean_deg"] = (
            float(np.nanmean(mean_values)) if mean_values else math.inf
        )
        combined[f"{prefix}_tracking_max_deg"] = max_err

    combined["mean_rmse_deg"] = 0.5 * (
        float(combined["knee_tracking_rms_deg"])
        + float(combined["ankle_tracking_rms_deg"])
    )
    combined["worst_rmse_deg"] = max(
        float(combined["knee_tracking_rms_deg"]),
        float(combined["ankle_tracking_rms_deg"]),
    )
    combined["max_abs_error_deg"] = max(
        float(combined["knee_tracking_max_deg"]),
        float(combined["ankle_tracking_max_deg"]),
    )

    for field in (
        "sat_count",
        "knee_tau_input_saturation_count",
        "ankle_tau_input_saturation_count",
    ):
        combined[field] = sum(int(row.get(field, 0) or 0) for row in rows)

    max_fields = [
        "knee_qdot_max_abs",
        "ankle_qdot_max_abs",
        "knee_motor_speed_max_abs",
        "ankle_motor_speed_max_abs",
        "knee_motor_speed_dot_max_abs",
        "ankle_motor_speed_dot_max_abs",
        "knee_tau_input_raw_max_abs",
        "ankle_tau_input_raw_max_abs",
        "knee_tau_input_plugin_max_abs",
        "ankle_tau_input_plugin_max_abs",
        "knee_tau_error_max_abs",
        "ankle_tau_error_max_abs",
        "knee_max_u",
        "ankle_max_u",
        "max_u",
        "max_tau_input_raw_abs",
        "max_tau_input_plugin_abs",
        "knee_motor_power_peak_abs",
        "ankle_motor_power_peak_abs",
    ]
    for field in max_fields:
        combined[field] = max(float(row.get(field, 0.0) or 0.0) for row in rows)

    mean_fields = [
        "knee_qdot_rms",
        "ankle_qdot_rms",
        "knee_qdot_hpf50_rms",
        "ankle_qdot_hpf50_rms",
        "knee_motor_speed_rms",
        "ankle_motor_speed_rms",
        "knee_motor_speed_hpf50_rms",
        "ankle_motor_speed_hpf50_rms",
        "knee_motor_speed_dot_rms",
        "ankle_motor_speed_dot_rms",
        "knee_motor_speed_dot_hpf50_rms",
        "ankle_motor_speed_dot_hpf50_rms",
        "knee_tau_input_raw_rms",
        "ankle_tau_input_raw_rms",
        "knee_tau_input_raw_mean_abs",
        "ankle_tau_input_raw_mean_abs",
        "knee_tau_input_plugin_rms",
        "ankle_tau_input_plugin_rms",
        "knee_tau_input_plugin_mean_abs",
        "ankle_tau_input_plugin_mean_abs",
        "knee_tau_input_plugin_hpf50_rms",
        "ankle_tau_input_plugin_hpf50_rms",
        "knee_tau_error_rms",
        "ankle_tau_error_rms",
        "knee_frac_u_gt_095",
        "ankle_frac_u_gt_095",
        "knee_motor_power_rms",
        "ankle_motor_power_rms",
        "knee_motor_power_mean_abs",
        "ankle_motor_power_mean_abs",
        "knee_motor_positive_energy_j",
        "ankle_motor_positive_energy_j",
        "knee_motor_negative_energy_j",
        "ankle_motor_negative_energy_j",
        "knee_motor_abs_energy_j",
        "ankle_motor_abs_energy_j",
    ]
    for field in mean_fields:
        values = [
            float(row.get(field, math.nan))
            for row in rows
            if row.get(field, "") != ""
        ]
        finite_values = [value for value in values if math.isfinite(value)]
        combined[field] = float(np.mean(finite_values)) if finite_values else math.nan

    combined["unstable"] = any(bool(row.get("unstable")) for row in rows)
    combined["acceptable"] = all(bool(row.get("acceptable")) for row in rows)
    add_scores(combined, baseline)
    return combined


CSV_FIELDS = [
    "run_id", "stage", "window_idx",
    "knee_kp_outer", "knee_kp_inner", "knee_ki_inner", "knee_i_limit",
    "ankle_kp_outer", "ankle_kp_inner", "ankle_ki_inner", "ankle_i_limit",
    "return_code", "timeout", "timeout_s", "elapsed_s",
    "run_status", "complete", "finite_outputs", "acceptable", "unstable",
    "fail_reason", "results_dir",
    "knee_tracking_rms_deg", "ankle_tracking_rms_deg", "mean_rmse_deg",
    "worst_rmse_deg", "max_abs_error_deg",
    "score_kinematic_deg", "score_chattering_norm", "score_power_norm",
    "knee_qdot_hpf50_rms", "ankle_qdot_hpf50_rms",
    "knee_motor_speed_hpf50_rms", "ankle_motor_speed_hpf50_rms",
    "knee_motor_speed_dot_hpf50_rms", "ankle_motor_speed_dot_hpf50_rms",
    "knee_tau_input_plugin_hpf50_rms", "ankle_tau_input_plugin_hpf50_rms",
    "knee_tau_input_raw_rms", "ankle_tau_input_raw_rms",
    "knee_tau_input_plugin_rms", "ankle_tau_input_plugin_rms",
    "knee_tau_input_raw_max_abs", "ankle_tau_input_raw_max_abs",
    "knee_tau_input_plugin_max_abs", "ankle_tau_input_plugin_max_abs",
    "sat_count", "knee_tau_input_saturation_count",
    "ankle_tau_input_saturation_count",
    "knee_max_u", "ankle_max_u", "max_u",
    "knee_frac_u_gt_095", "ankle_frac_u_gt_095",
    "knee_motor_power_rms", "ankle_motor_power_rms",
    "knee_motor_power_mean_abs", "ankle_motor_power_mean_abs",
    "knee_motor_power_peak_abs", "ankle_motor_power_peak_abs",
    "knee_motor_positive_energy_j", "ankle_motor_positive_energy_j",
    "knee_motor_negative_energy_j", "ankle_motor_negative_energy_j",
    "knee_motor_abs_energy_j", "ankle_motor_abs_energy_j",
]


def write_csv(path: Path, rows: Sequence[Dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    extra = sorted({key for row in rows for key in row.keys()} - set(CSV_FIELDS))
    fields = CSV_FIELDS + extra
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fields})


def write_json(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def status_label(row: Dict[str, object]) -> str:
    if bool(row.get("timeout")):
        return "TIMEOUT"
    if bool(row.get("acceptable")):
        return "OK"
    if bool(row.get("complete")) and bool(row.get("finite_outputs")):
        return "REJECT"
    return "FAIL"


def ranking_tuple(row: Dict[str, object]) -> tuple[float, float, float, float, float]:
    return (
        float(row.get("score_chattering_norm", math.inf)),
        float(row.get("score_power_norm", math.inf)),
        float(row.get("score_kinematic_deg", math.inf)),
        float(row.get("worst_rmse_deg", math.inf)),
        float(row.get("max_abs_error_deg", math.inf)),
    )


def rank_rows(rows: Sequence[Dict[str, object]]) -> List[Dict[str, object]]:
    acceptable = [
        row for row in rows
        if bool(row.get("acceptable"))
        and math.isfinite(float(row.get("score_kinematic_deg", math.inf)))
    ]
    remaining = sorted(
        acceptable,
        key=lambda row: float(row.get("score_kinematic_deg", math.inf)),
    )
    ranked: List[Dict[str, object]] = []
    while remaining:
        best = remaining[0]
        best_score = float(best.get("score_kinematic_deg", math.inf))
        if best_score <= 0.0 or not math.isfinite(best_score):
            threshold = best_score
        else:
            threshold = best_score * 1.02
        group = [
            row for row in remaining
            if float(row.get("score_kinematic_deg", math.inf)) <= threshold
        ]
        group_ids = {id(row) for row in group}
        group.sort(key=ranking_tuple)
        ranked.extend(group)
        remaining = [row for row in remaining if id(row) not in group_ids]
    return ranked


def write_failures(path: Path, row_groups: Iterable[Sequence[Dict[str, object]]]) -> None:
    failures: List[Dict[str, object]] = []
    for rows in row_groups:
        failures.extend(row for row in rows if not bool(row.get("acceptable")))
    write_csv(path, failures)


def run_one_window(
    candidate: Candidate,
    stage: str,
    window_idx: int,
    run_dir: Path,
    setup_path: Path,
    model_path: Path,
    reference_path: Path,
    python_exe: str,
    t_start: float,
    t_end: float,
    timeout_s: float,
) -> Dict[str, object]:
    start = time.monotonic()
    rc = run_simulation(
        python_exe, setup_path, model_path, run_dir, candidate, t_start, t_end, timeout_s
    )
    elapsed = time.monotonic() - start
    return collect_metrics(
        candidate, stage, run_dir, reference_path, rc, elapsed, timeout_s, window_idx
    )


def run_stage(
    stage: str,
    candidates: Sequence[Candidate],
    windows: Sequence[tuple[float, float]],
    sweep_root: Path,
    setup_path: Path,
    model_path: Path,
    reference_path: Path,
    python_exe: str,
    workers: int,
    csv_path: Path,
    timeout_minimum: float,
    baseline: Dict[str, object] | None,
    overall: OverallProgress | None = None,
) -> List[Dict[str, object]]:
    total_jobs = len(candidates) * len(windows)
    print(
        f"[Sweep] {stage}: candidates={len(candidates)} "
        f"windows={len(windows)} workers={workers}",
        flush=True,
    )
    tracker = ProgressTracker(total_jobs, workers)
    rows_by_candidate: Dict[str, List[Dict[str, object]]] = {
        candidate.run_id: [] for candidate in candidates
    }
    combined_rows: List[Dict[str, object]] = []

    with ThreadPoolExecutor(max_workers=max(1, min(workers, total_jobs))) as pool:
        future_map = {}
        for candidate in candidates:
            for window_idx, (t_start, t_end) in enumerate(windows, start=1):
                timeout_s = timeout_for_window(t_start, t_end, timeout_minimum)
                run_dir = sweep_root / "runs" / stage / f"{candidate.run_id}_w{window_idx}"
                future = pool.submit(
                    run_one_window,
                    candidate,
                    stage,
                    window_idx,
                    run_dir,
                    setup_path,
                    model_path,
                    reference_path,
                    python_exe,
                    t_start,
                    t_end,
                    timeout_s,
                )
                future_map[future] = candidate

        done_jobs = 0
        for future in as_completed(future_map):
            candidate = future_map[future]
            row = future.result()
            job_elapsed = float(row.get("elapsed_s", 0.0) or 0.0)
            tracker.update(job_elapsed)
            if overall is not None:
                overall.update(job_elapsed)
            rows_by_candidate[candidate.run_id].append(row)
            done_jobs += 1
            line = tracker.line(done_jobs, stage, candidate.run_id, status_label(row))
            if overall is not None:
                line = f"{line} | {overall.suffix()}"
            print(line, flush=True)
            if len(rows_by_candidate[candidate.run_id]) == len(windows):
                combined = aggregate_window_rows(
                    candidate, stage, rows_by_candidate[candidate.run_id], baseline
                )
                combined_rows.append(combined)
                write_csv(csv_path, combined_rows)

    write_csv(csv_path, combined_rows)
    return combined_rows


def run_full_candidates(
    candidates: Sequence[Candidate],
    sweep_root: Path,
    setup_path: Path,
    model_path: Path,
    reference_path: Path,
    python_exe: str,
    workers: int,
    csv_path: Path,
    timeout_minimum: float,
    baseline: Dict[str, object] | None,
    overall: OverallProgress | None = None,
) -> List[Dict[str, object]]:
    stage = "full"
    t_start, t_end = FULL_WINDOW
    timeout_s = timeout_for_window(t_start, t_end, timeout_minimum)
    print(
        f"[Sweep] full: candidates={len(candidates)} workers={workers} "
        f"timeout={timeout_s:.1f}s",
        flush=True,
    )
    tracker = ProgressTracker(len(candidates), workers)
    rows: List[Dict[str, object]] = []
    with ThreadPoolExecutor(max_workers=max(1, min(workers, len(candidates)))) as pool:
        future_map = {}
        for candidate in candidates:
            run_dir = sweep_root / "full_runs" / candidate.run_id
            future = pool.submit(
                run_one_window,
                candidate,
                stage,
                1,
                run_dir,
                setup_path,
                model_path,
                reference_path,
                python_exe,
                t_start,
                t_end,
                timeout_s,
            )
            future_map[future] = candidate

        done = 0
        for future in as_completed(future_map):
            row = future.result()
            add_scores(row, baseline)
            rows.append(row)
            job_elapsed = float(row.get("elapsed_s", 0.0) or 0.0)
            tracker.update(job_elapsed)
            if overall is not None:
                overall.update(job_elapsed)
            done += 1
            write_csv(csv_path, rows)
            line = tracker.line(done, stage, str(row.get("run_id", "")), status_label(row))
            if overall is not None:
                line = f"{line} | {overall.suffix()}"
            print(line, flush=True)

    write_csv(csv_path, rows)
    return rows


def collect_baseline(
    baseline_dir: Path,
    reference_path: Path,
    baseline_csv: Path,
) -> Dict[str, object] | None:
    row = collect_metrics(
        BASE_CANDIDATE,
        "baseline",
        baseline_dir,
        reference_path,
        return_code=0,
        elapsed_s=0.0,
        timeout_s=0.0,
        window_idx=None,
    )
    if not bool(row.get("complete")) or not bool(row.get("finite_outputs")):
        row["fail_reason"] = row.get("fail_reason") or "baseline_unavailable"
        write_csv(baseline_csv, [row])
        return None
    add_scores(row, row)
    write_csv(baseline_csv, [row])
    return row


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Sweep local AB06 cascade gains for knee and ankle tracking."
    )
    parser.add_argument("--workers", type=int, default=default_workers())
    parser.add_argument("--python", default=sys.executable)
    parser.add_argument("--setup", default=DEFAULT_SETUP)
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--reference", default=DEFAULT_REFERENCE)
    parser.add_argument("--baseline-dir", default=DEFAULT_BASELINE_DIR)
    parser.add_argument("--sweep-root", default=None)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--quick-smoke", action="store_true")
    parser.add_argument("--top-n-per-joint", type=int, default=5)
    parser.add_argument("--timeout-minimum", type=float, default=300.0)
    parser.add_argument("--kp-outer-multipliers", default=None)
    parser.add_argument("--kp-inner-multipliers", default=None)
    parser.add_argument("--ki-inner-multipliers", default=None)
    parser.add_argument("--knee-i-limit-grid", default=None)
    parser.add_argument("--ankle-i-limit-grid", default=None)
    return parser


def main() -> int:
    args = build_parser().parse_args()

    setup_path = resolve(args.setup)
    model_path = resolve(args.model)
    reference_path = resolve(args.reference)
    baseline_dir = resolve(args.baseline_dir)

    kp_outer_mult = parse_grid(args.kp_outer_multipliers, KP_OUTER_MULTIPLIERS)
    kp_inner_mult = parse_grid(args.kp_inner_multipliers, KP_INNER_MULTIPLIERS)
    ki_inner_mult = parse_grid(args.ki_inner_multipliers, KI_INNER_MULTIPLIERS)
    knee_i_limit_grid = parse_grid(args.knee_i_limit_grid, [BASE_KNEE_I_LIMIT])
    ankle_i_limit_grid = parse_grid(args.ankle_i_limit_grid, [BASE_ANKLE_I_LIMIT])

    knee_candidates = generate_knee_candidates(
        scaled_grid(BASE_KNEE_KP_OUTER, kp_outer_mult),
        scaled_grid(BASE_KNEE_KP_INNER, kp_inner_mult),
        scaled_grid(BASE_KNEE_KI_INNER, ki_inner_mult),
        knee_i_limit_grid,
    )
    ankle_candidates = generate_ankle_candidates(
        scaled_grid(BASE_ANKLE_KP_OUTER, kp_outer_mult),
        scaled_grid(BASE_ANKLE_KP_INNER, kp_inner_mult),
        scaled_grid(BASE_ANKLE_KI_INNER, ki_inner_mult),
        ankle_i_limit_grid,
    )

    windows = SCREEN_WINDOWS
    full_candidates_count = min(args.top_n_per_joint, len(knee_candidates)) * min(
        args.top_n_per_joint, len(ankle_candidates)
    )
    total_real_jobs = (
        len(knee_candidates) * len(windows)
        + len(ankle_candidates) * len(windows)
        + full_candidates_count
    )

    if args.quick_smoke:
        knee_candidates = [candidate_with_id("knee", BASE_CANDIDATE)]
        ankle_candidates = [candidate_with_id("ankle", BASE_CANDIDATE)]
        windows = (QUICK_SMOKE_WINDOW,)
        full_candidates_count = 0
        total_jobs = len(knee_candidates) + len(ankle_candidates)
    else:
        total_jobs = total_real_jobs

    full_timeout = timeout_for_window(FULL_WINDOW[0], FULL_WINDOW[1], args.timeout_minimum)

    if args.dry_run:
        print("[Sweep] dry run")
        print(f"  setup={setup_path}")
        print(f"  model={model_path}")
        print(f"  reference={reference_path}")
        print(f"  baseline_dir={baseline_dir}")
        print(f"  workers={args.workers}")
        print(f"  kp_outer_multipliers={kp_outer_mult}")
        print(f"  kp_inner_multipliers={kp_inner_mult}")
        print(f"  ki_inner_multipliers={ki_inner_mult}")
        print(f"  knee_i_limit_grid={knee_i_limit_grid}")
        print(f"  ankle_i_limit_grid={ankle_i_limit_grid}")
        print(f"  screen_windows={windows}")
        print(f"  knee_candidates={len(knee_candidates)}")
        print(f"  ankle_candidates={len(ankle_candidates)}")
        print(f"  full_candidates={full_candidates_count}")
        print(f"  total_jobs={total_jobs}")
        print(f"  full_timeout_s={full_timeout:.1f}")
        sample = knee_candidates[0] if knee_candidates else BASE_CANDIDATE
        sample_cmd = command_for_run(
            args.python,
            setup_path,
            model_path,
            Path("results/_cascade_local_gain_sweep_example"),
            sample,
            windows[0][0],
            windows[0][1],
        )
        print("  sample_command=" + format_command(sample_cmd))
        print(
            "  ranking=acceptable first; 2% kinematic tie bands sorted by "
            "chattering score then motor power score"
        )
        return 0

    for path, label in (
        (setup_path, "setup"),
        (model_path, "model"),
        (reference_path, "reference"),
        (baseline_dir, "baseline dir"),
    ):
        if label.endswith("dir"):
            if not path.is_dir():
                raise FileNotFoundError(f"Missing {label}: {path}")
        elif not path.is_file():
            raise FileNotFoundError(f"Missing {label}: {path}")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    sweep_root = (
        resolve(args.sweep_root)
        if args.sweep_root
        else REPO_ROOT / "results" / f"_cascade_local_gain_sweep_{stamp}"
    )
    sweep_root.mkdir(parents=True, exist_ok=True)

    print(f"[Sweep] root={sweep_root}")
    print(f"[Sweep] workers={args.workers}")
    print(f"[Sweep] total_jobs={total_jobs}")
    print(f"[Sweep] python={args.python}")
    print(f"[Sweep] full_timeout_s={full_timeout:.1f}")

    baseline = collect_baseline(
        baseline_dir, reference_path, sweep_root / "baseline_metrics.csv"
    )
    if baseline is None:
        print(
            "[Sweep] baseline metrics unavailable; chattering/power scores "
            "will use raw unnormalized values.",
            flush=True,
        )
    else:
        print(
            "[Sweep] baseline score_kinematic_deg="
            f"{float(baseline.get('score_kinematic_deg', math.nan)):.6f}",
            flush=True,
        )

    overall = OverallProgress(ProgressTracker(total_jobs, args.workers))

    knee_rows = run_stage(
        "stage1_knee",
        knee_candidates,
        windows,
        sweep_root,
        setup_path,
        model_path,
        reference_path,
        args.python,
        args.workers,
        sweep_root / "stage1_knee_screen.csv",
        args.timeout_minimum,
        baseline,
        overall,
    )
    ankle_rows = run_stage(
        "stage1_ankle",
        ankle_candidates,
        windows,
        sweep_root,
        setup_path,
        model_path,
        reference_path,
        args.python,
        args.workers,
        sweep_root / "stage1_ankle_screen.csv",
        args.timeout_minimum,
        baseline,
        overall,
    )

    if args.quick_smoke:
        smoke_rows = knee_rows + ankle_rows
        smoke_pass = (
            len(smoke_rows) == 2
            and all(bool(row.get("complete")) for row in smoke_rows)
            and all(bool(row.get("finite_outputs")) for row in smoke_rows)
        )
        write_failures(sweep_root / "failures.csv", [knee_rows, ankle_rows])
        write_json(sweep_root / "sweep_summary.json", {
            "mode": "quick_smoke",
            "total_jobs": total_jobs,
            "smoke_pass": smoke_pass,
            "baseline_available": baseline is not None,
            "knee_rows": len(knee_rows),
            "ankle_rows": len(ankle_rows),
            "acceptable_rows": sum(
                1 for row in smoke_rows if bool(row.get("acceptable"))
            ),
            "full_rows": 0,
        })
        if smoke_pass:
            print(
                "[Sweep] quick smoke complete/finite; full sweep not launched.",
                flush=True,
            )
            return 0
        print("[Sweep] quick smoke failed completeness/finite-output checks.", flush=True)
        return 1

    top_n = max(1, int(args.top_n_per_joint))
    top_knee = rank_rows(knee_rows)[:top_n]
    top_ankle = rank_rows(ankle_rows)[:top_n]
    if not top_knee or not top_ankle:
        write_failures(sweep_root / "failures.csv", [knee_rows, ankle_rows])
        write_json(sweep_root / "sweep_summary.json", {
            "mode": "full",
            "total_jobs": total_jobs,
            "baseline_available": baseline is not None,
            "error": "no acceptable stage1 candidates",
        })
        print("[Sweep] no acceptable stage1 candidates for full combo.", flush=True)
        return 1

    full_candidates = combine_candidates(top_knee, top_ankle)
    full_rows = run_full_candidates(
        full_candidates,
        sweep_root,
        setup_path,
        model_path,
        reference_path,
        args.python,
        args.workers,
        sweep_root / "full_results.csv",
        args.timeout_minimum,
        baseline,
        overall,
    )
    ranking = rank_rows(full_rows)
    write_csv(sweep_root / "ranking.csv", ranking)
    write_failures(sweep_root / "failures.csv", [knee_rows, ankle_rows, full_rows])

    best = ranking[0] if ranking else None
    if best is not None:
        write_json(sweep_root / "best_candidate.json", best)
    write_json(sweep_root / "sweep_summary.json", {
        "mode": "full",
        "total_jobs": total_jobs,
        "baseline_available": baseline is not None,
        "ranking_rows": len(ranking),
        "best": best,
        "ranking_policy": (
            "acceptable first; 2% kinematic tie bands sorted by chattering "
            "score, then motor power score"
        ),
    })

    if best is None:
        print("[Sweep] no acceptable full candidates.", flush=True)
        return 1

    print("[Sweep] best full candidate:", flush=True)
    print(
        f"  {best['run_id']} kin={float(best['score_kinematic_deg']):.6g} "
        f"chatter={float(best['score_chattering_norm']):.6g} "
        f"power={float(best['score_power_norm']):.6g}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
