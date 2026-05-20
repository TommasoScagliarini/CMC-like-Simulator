"""
outer_pid_gain_sweep.py
=======================

PID gain sweep for the prosthetic SEA outer loop on AB06 with the
paper-equivalent 321/500 Nm/rad stiffness model.

This script is intentionally separate from ``outer_gain_sweep.py`` so the
historical PD sweep remains unchanged. It ranks candidates primarily by
prosthetic kinematic tracking RMSE against the IK reference.
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

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from output import read_sto  # noqa: E402


SEA_KNEE = "SEA_Knee"
SEA_ANKLE = "SEA_Ankle"
COORD_KNEE = "pros_knee_angle"
COORD_ANKLE = "pros_ankle_angle"

BASE_KNEE_KP = 160.0
BASE_KNEE_KD = 12.0
BASE_ANKLE_KP = 420.0
BASE_ANKLE_KD = 1.0

KNEE_KP_GRID = [80, 120, 160, 220, 300]
KNEE_KD_GRID = [6, 12, 18, 26]
KNEE_KI_GRID = [0, 10, 20, 40, 80]
ANKLE_KP_GRID = [250, 350, 420, 560, 750]
ANKLE_KD_GRID = [0.5, 1, 2, 4]
ANKLE_KI_GRID = [0, 30, 60, 120, 240]

SCREEN_WINDOWS = (
    (13.1638, 14.7799),
    (17.9528, 19.5259),
)
QUICK_SMOKE_WINDOW = (13.1638, 13.1938)
FULL_WINDOW = (11.99, 21.0)

DEFAULT_SETUP = "models/AB06_SEASEA_Threadmill/AB06_SEASEA_setup.xml"
DEFAULT_MODEL = "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500.osim"
DEFAULT_REFERENCE = "models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot"
DEFAULT_BASELINE_DIR = "results/_stiff321_500_ab06_pd_full"

BASELINE_MEAN_PROS_RMSE_DEG = 4.270496991108777


@dataclass(frozen=True)
class Candidate:
    run_id: str
    knee_kp: float
    knee_kd: float
    knee_ki: float
    ankle_kp: float
    ankle_kd: float
    ankle_ki: float


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
        "knee_kp": candidate.knee_kp,
        "knee_kd": candidate.knee_kd,
        "knee_ki": candidate.knee_ki,
        "ankle_kp": candidate.ankle_kp,
        "ankle_kd": candidate.ankle_kd,
        "ankle_ki": candidate.ankle_ki,
    }


def make_run_id(prefix: str, knee_kp: float, knee_kd: float, knee_ki: float,
                ankle_kp: float, ankle_kd: float, ankle_ki: float) -> str:
    return (
        f"{prefix}_kkp{fmt_num(knee_kp)}_kkd{fmt_num(knee_kd)}"
        f"_kki{fmt_num(knee_ki)}_akp{fmt_num(ankle_kp)}"
        f"_akd{fmt_num(ankle_kd)}_aki{fmt_num(ankle_ki)}"
    )


def generate_knee_candidates(kp_grid: Sequence[float], kd_grid: Sequence[float],
                             ki_grid: Sequence[float]) -> List[Candidate]:
    candidates: List[Candidate] = []
    for kp in kp_grid:
        for kd in kd_grid:
            for ki in ki_grid:
                candidates.append(Candidate(
                    run_id=make_run_id(
                        "knee", kp, kd, ki,
                        BASE_ANKLE_KP, BASE_ANKLE_KD, 0.0,
                    ),
                    knee_kp=float(kp),
                    knee_kd=float(kd),
                    knee_ki=float(ki),
                    ankle_kp=BASE_ANKLE_KP,
                    ankle_kd=BASE_ANKLE_KD,
                    ankle_ki=0.0,
                ))
    return candidates


def generate_ankle_candidates(kp_grid: Sequence[float], kd_grid: Sequence[float],
                              ki_grid: Sequence[float]) -> List[Candidate]:
    candidates: List[Candidate] = []
    for kp in kp_grid:
        for kd in kd_grid:
            for ki in ki_grid:
                candidates.append(Candidate(
                    run_id=make_run_id(
                        "ankle", BASE_KNEE_KP, BASE_KNEE_KD, 0.0,
                        kp, kd, ki,
                    ),
                    knee_kp=BASE_KNEE_KP,
                    knee_kd=BASE_KNEE_KD,
                    knee_ki=0.0,
                    ankle_kp=float(kp),
                    ankle_kd=float(kd),
                    ankle_ki=float(ki),
                ))
    return candidates


def candidate_from_row(row: Dict[str, object], prefix: str) -> Candidate:
    knee_kp = float(row["knee_kp"])
    knee_kd = float(row["knee_kd"])
    knee_ki = float(row["knee_ki"])
    ankle_kp = float(row["ankle_kp"])
    ankle_kd = float(row["ankle_kd"])
    ankle_ki = float(row["ankle_ki"])
    return Candidate(
        run_id=make_run_id(prefix, knee_kp, knee_kd, knee_ki, ankle_kp, ankle_kd, ankle_ki),
        knee_kp=knee_kp,
        knee_kd=knee_kd,
        knee_ki=knee_ki,
        ankle_kp=ankle_kp,
        ankle_kd=ankle_kd,
        ankle_ki=ankle_ki,
    )


def combine_candidates(knee_rows: Sequence[Dict[str, object]],
                       ankle_rows: Sequence[Dict[str, object]]) -> List[Candidate]:
    candidates: List[Candidate] = []
    for kr in knee_rows:
        for ar in ankle_rows:
            knee_kp = float(kr["knee_kp"])
            knee_kd = float(kr["knee_kd"])
            knee_ki = float(kr["knee_ki"])
            ankle_kp = float(ar["ankle_kp"])
            ankle_kd = float(ar["ankle_kd"])
            ankle_ki = float(ar["ankle_ki"])
            candidates.append(Candidate(
                run_id=make_run_id(
                    "combo", knee_kp, knee_kd, knee_ki,
                    ankle_kp, ankle_kd, ankle_ki,
                ),
                knee_kp=knee_kp,
                knee_kd=knee_kd,
                knee_ki=knee_ki,
                ankle_kp=ankle_kp,
                ankle_kd=ankle_kd,
                ankle_ki=ankle_ki,
            ))
    return candidates


def command_for_run(python_exe: str, setup_path: Path, model_path: Path,
                    results_dir: Path, candidate: Candidate,
                    t_start: float, t_end: float) -> List[str]:
    return [
        python_exe,
        str(REPO_ROOT / "main.py"),
        "--setup", str(setup_path),
        "--model", str(model_path),
        "--t-start", fmt_xml(t_start),
        "--t-end", fmt_xml(t_end),
        "--output-dir", str(results_dir),
        "--filter-grf",
        "--sea-outer-controller", "pid",
        "--sea-kp-knee", fmt_xml(candidate.knee_kp),
        "--sea-kd-knee", fmt_xml(candidate.knee_kd),
        "--sea-ki-knee", fmt_xml(candidate.knee_ki),
        "--sea-kp-ankle", fmt_xml(candidate.ankle_kp),
        "--sea-kd-ankle", fmt_xml(candidate.ankle_kd),
        "--sea-ki-ankle", fmt_xml(candidate.ankle_ki),
    ]


def timeout_for_window(t_start: float, t_end: float, minimum: float) -> float:
    return max(float(minimum), 180.0 * max(0.01, float(t_end) - float(t_start)))


def run_simulation(python_exe: str, setup_path: Path, model_path: Path,
                   results_dir: Path, candidate: Candidate,
                   t_start: float, t_end: float, timeout_s: float) -> int:
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


def unique_time_series(time_values: np.ndarray, values: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    unique_time, indices = np.unique(time_values, return_index=True)
    order = np.argsort(indices)
    selected = indices[order]
    return time_values[selected], values[selected]


def tracking_stats(results_dir: Path, reference_path: Path,
                   coord: str) -> Dict[str, float]:
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


def collect_metrics(candidate: Candidate, stage: str, results_dir: Path,
                    reference_path: Path, return_code: int, elapsed_s: float,
                    timeout_s: float, window_idx: int | None = None) -> Dict[str, object]:
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
        knee = tracking_stats(results_dir, reference_path, COORD_KNEE)
        ankle = tracking_stats(results_dir, reference_path, COORD_ANKLE)
        for prefix, stats in (("knee", knee), ("ankle", ankle)):
            row[f"{prefix}_tracking_n"] = stats["n_samples"]
            row[f"{prefix}_tracking_sse_deg2"] = stats["sse_deg2"]
            row[f"{prefix}_tracking_rms_deg"] = stats["rms_deg"]
            row[f"{prefix}_tracking_mean_deg"] = stats["mean_abs_deg"]
            row[f"{prefix}_tracking_max_deg"] = stats["max_abs_deg"]

        row["mean_pros_rmse_deg"] = 0.5 * (
            float(row["knee_tracking_rms_deg"])
            + float(row["ankle_tracking_rms_deg"])
        )
        row["worst_pros_rmse_deg"] = max(
            float(row["knee_tracking_rms_deg"]),
            float(row["ankle_tracking_rms_deg"]),
        )
        row["max_pros_error_deg"] = max(
            float(row["knee_tracking_max_deg"]),
            float(row["ankle_tracking_max_deg"]),
        )

        diag = results_dir / "sim_output_sea_diagnostics.sto"
        controls = results_dir / "sim_output_sea_controls.sto"
        sat_count = 0
        max_u = 0.0
        max_tau_raw = 0.0
        max_speed_dot = 0.0
        for label, sea_name, coord in (
            ("knee", SEA_KNEE, COORD_KNEE),
            ("ankle", SEA_ANKLE, COORD_ANKLE),
        ):
            _t, u, _deg = load_column(controls, coord)
            u_abs = np.abs(u)
            row[f"{label}_max_u"] = float(np.nanmax(u_abs))
            row[f"{label}_frac_u_gt_095"] = float(np.nanmean(u_abs > 0.95))
            max_u = max(max_u, float(row[f"{label}_max_u"]))

            _t, sat, _deg = load_column(diag, f"{sea_name}_tau_input_saturated")
            sat_i = int(np.nansum(sat > 0.5))
            row[f"{label}_tau_input_saturation_count"] = sat_i
            sat_count += sat_i

            _t, tau_error, _deg = load_column(diag, f"{sea_name}_tau_error")
            tau_error_stats = signal_stats(tau_error)
            row[f"{label}_tau_error_rms"] = tau_error_stats["rms"]
            row[f"{label}_tau_error_mean_abs"] = tau_error_stats["mean_abs"]
            row[f"{label}_tau_error_max_abs"] = tau_error_stats["max_abs"]

            _t, tau_raw, _deg = load_column(diag, f"{sea_name}_tau_input_raw")
            tau_raw_stats = signal_stats(tau_raw)
            row[f"{label}_tau_input_raw_rms"] = tau_raw_stats["rms"]
            row[f"{label}_tau_input_raw_mean_abs"] = tau_raw_stats["mean_abs"]
            row[f"{label}_max_tau_input_raw_abs"] = tau_raw_stats["max_abs"]
            max_tau_raw = max(max_tau_raw, float(row[f"{label}_max_tau_input_raw_abs"]))

            _t, speed_dot, _deg = load_column(
                diag, f"{sea_name}_motor_speed_dot_plugin"
            )
            speed_dot_stats = signal_stats(speed_dot)
            row[f"{label}_motor_speed_dot_rms"] = speed_dot_stats["rms"]
            row[f"{label}_motor_speed_dot_mean_abs"] = speed_dot_stats["mean_abs"]
            row[f"{label}_max_motor_speed_dot_abs"] = speed_dot_stats["max_abs"]
            max_speed_dot = max(max_speed_dot, float(row[f"{label}_max_motor_speed_dot_abs"]))

            power = results_dir / "sim_output_power.sto"
            _t, joint_power, _deg = load_column(power, f"{sea_name}_joint_power")
            _t, motor_power, _deg = load_column(power, f"{sea_name}_motor_power")
            joint_power_stats = signal_stats(joint_power)
            motor_power_stats = signal_stats(motor_power)
            diff_power_stats = signal_stats(motor_power - joint_power)
            row[f"{label}_joint_power_rms"] = joint_power_stats["rms"]
            row[f"{label}_motor_power_rms"] = motor_power_stats["rms"]
            row[f"{label}_motor_joint_power_diff_rms"] = diff_power_stats["rms"]
            if np.std(joint_power) > 0.0 and np.std(motor_power) > 0.0:
                row[f"{label}_motor_joint_power_corr"] = float(
                    np.corrcoef(joint_power, motor_power)[0, 1]
                )
            else:
                row[f"{label}_motor_joint_power_corr"] = math.nan

        row["sat_count"] = sat_count
        row["max_u"] = max_u
        row["max_tau_input_raw_abs"] = max_tau_raw
        row["max_motor_speed_dot_abs"] = max_speed_dot

        fail_reasons: List[str] = []
        if return_code != 0:
            fail_reasons.append(f"return_code_{return_code}")
        if sat_count > 0:
            fail_reasons.append("tau_input_saturated")
        if max_u > 0.99:
            fail_reasons.append("max_u_gt_0p99")
        if max_tau_raw > 500.0:
            fail_reasons.append("tau_input_raw_gt_500")

        row["fail_reason"] = ";".join(fail_reasons)
        row["unstable"] = bool(fail_reasons)
        row["acceptable"] = not fail_reasons
    except Exception as exc:
        row["fail_reason"] = f"metric_error:{type(exc).__name__}:{exc}"
        row["acceptable"] = False
        row["unstable"] = True
    return row


def aggregate_window_rows(candidate: Candidate, stage: str,
                          rows: Sequence[Dict[str, object]]) -> Dict[str, object]:
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
        return combined

    for prefix in ("knee", "ankle"):
        n = sum(int(row.get(f"{prefix}_tracking_n", 0) or 0) for row in rows)
        sse = sum(float(row.get(f"{prefix}_tracking_sse_deg2", 0.0) or 0.0) for row in rows)
        max_err = max(float(row.get(f"{prefix}_tracking_max_deg", 0.0) or 0.0) for row in rows)
        mean_abs_values = [
            float(row.get(f"{prefix}_tracking_mean_deg", math.nan))
            for row in rows
            if row.get(f"{prefix}_tracking_mean_deg", "") != ""
        ]
        combined[f"{prefix}_tracking_n"] = n
        combined[f"{prefix}_tracking_sse_deg2"] = sse
        combined[f"{prefix}_tracking_rms_deg"] = math.sqrt(sse / n) if n else math.inf
        combined[f"{prefix}_tracking_mean_deg"] = (
            float(np.nanmean(mean_abs_values)) if mean_abs_values else math.inf
        )
        combined[f"{prefix}_tracking_max_deg"] = max_err

    combined["mean_pros_rmse_deg"] = 0.5 * (
        float(combined["knee_tracking_rms_deg"])
        + float(combined["ankle_tracking_rms_deg"])
    )
    combined["worst_pros_rmse_deg"] = max(
        float(combined["knee_tracking_rms_deg"]),
        float(combined["ankle_tracking_rms_deg"]),
    )
    combined["max_pros_error_deg"] = max(
        float(combined["knee_tracking_max_deg"]),
        float(combined["ankle_tracking_max_deg"]),
    )

    for field in (
        "sat_count",
        "knee_tau_input_saturation_count",
        "ankle_tau_input_saturation_count",
    ):
        combined[field] = sum(int(row.get(field, 0) or 0) for row in rows)
    for field in (
        "knee_tau_error_max_abs",
        "ankle_tau_error_max_abs",
        "knee_max_u",
        "ankle_max_u",
        "max_u",
        "knee_max_tau_input_raw_abs",
        "ankle_max_tau_input_raw_abs",
        "max_tau_input_raw_abs",
        "knee_max_motor_speed_dot_abs",
        "ankle_max_motor_speed_dot_abs",
        "max_motor_speed_dot_abs",
    ):
        combined[field] = max(float(row.get(field, 0.0) or 0.0) for row in rows)

    for field in (
        "knee_tau_error_rms",
        "ankle_tau_error_rms",
        "knee_tau_error_mean_abs",
        "ankle_tau_error_mean_abs",
        "knee_tau_input_raw_rms",
        "ankle_tau_input_raw_rms",
        "knee_tau_input_raw_mean_abs",
        "ankle_tau_input_raw_mean_abs",
        "knee_motor_speed_dot_rms",
        "ankle_motor_speed_dot_rms",
        "knee_motor_speed_dot_mean_abs",
        "ankle_motor_speed_dot_mean_abs",
        "knee_joint_power_rms",
        "ankle_joint_power_rms",
        "knee_motor_power_rms",
        "ankle_motor_power_rms",
        "knee_motor_joint_power_diff_rms",
        "ankle_motor_joint_power_diff_rms",
        "knee_motor_joint_power_corr",
        "ankle_motor_joint_power_corr",
    ):
        values = [
            float(row.get(field, math.nan))
            for row in rows
            if row.get(field, "") != ""
        ]
        finite_values = [value for value in values if math.isfinite(value)]
        combined[field] = (
            float(np.mean(finite_values)) if finite_values else math.nan
        )

    combined["unstable"] = any(bool(row.get("unstable")) for row in rows)
    combined["acceptable"] = all(bool(row.get("acceptable")) for row in rows)
    return combined


CSV_FIELDS = [
    "run_id", "stage", "window_idx",
    "knee_kp", "knee_kd", "knee_ki",
    "ankle_kp", "ankle_kd", "ankle_ki",
    "return_code", "timeout", "timeout_s", "elapsed_s",
    "run_status", "complete", "finite_outputs", "acceptable", "unstable",
    "fail_reason", "results_dir",
    "knee_tracking_rms_deg", "knee_tracking_mean_deg", "knee_tracking_max_deg",
    "ankle_tracking_rms_deg", "ankle_tracking_mean_deg", "ankle_tracking_max_deg",
    "mean_pros_rmse_deg", "worst_pros_rmse_deg", "max_pros_error_deg",
    "delta_mean_rmse_deg", "delta_mean_rmse_pct",
    "knee_max_u", "ankle_max_u", "max_u",
    "knee_frac_u_gt_095", "ankle_frac_u_gt_095",
    "sat_count", "knee_tau_input_saturation_count",
    "ankle_tau_input_saturation_count",
    "knee_tau_error_rms", "ankle_tau_error_rms",
    "knee_tau_error_mean_abs", "ankle_tau_error_mean_abs",
    "knee_tau_error_max_abs", "ankle_tau_error_max_abs",
    "knee_tau_input_raw_rms", "ankle_tau_input_raw_rms",
    "knee_tau_input_raw_mean_abs", "ankle_tau_input_raw_mean_abs",
    "knee_max_tau_input_raw_abs", "ankle_max_tau_input_raw_abs",
    "max_tau_input_raw_abs",
    "knee_motor_speed_dot_rms", "ankle_motor_speed_dot_rms",
    "knee_motor_speed_dot_mean_abs", "ankle_motor_speed_dot_mean_abs",
    "knee_max_motor_speed_dot_abs", "ankle_max_motor_speed_dot_abs",
    "max_motor_speed_dot_abs",
    "knee_joint_power_rms", "ankle_joint_power_rms",
    "knee_motor_power_rms", "ankle_motor_power_rms",
    "knee_motor_joint_power_diff_rms", "ankle_motor_joint_power_diff_rms",
    "knee_motor_joint_power_corr", "ankle_motor_joint_power_corr",
]


def add_baseline_deltas(row: Dict[str, object], baseline_mean: float) -> None:
    try:
        mean = float(row["mean_pros_rmse_deg"])
    except (KeyError, TypeError, ValueError):
        return
    row["delta_mean_rmse_deg"] = mean - baseline_mean
    row["delta_mean_rmse_pct"] = 100.0 * (mean - baseline_mean) / baseline_mean


def write_csv(path: Path, rows: Sequence[Dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in CSV_FIELDS})


def row_sort_key(row: Dict[str, object]) -> tuple[float, float, float]:
    if not bool(row.get("acceptable")):
        return (math.inf, math.inf, math.inf)
    return (
        float(row.get("mean_pros_rmse_deg", math.inf)),
        float(row.get("worst_pros_rmse_deg", math.inf)),
        float(row.get("max_pros_error_deg", math.inf)),
    )


def status_label(row: Dict[str, object]) -> str:
    if bool(row.get("timeout")):
        return "TIMEOUT"
    if bool(row.get("acceptable")):
        return "OK"
    return "FAIL"


def run_stage(stage: str, candidates: Sequence[Candidate],
              windows: Sequence[tuple[float, float]], sweep_root: Path,
              setup_path: Path, model_path: Path, reference_path: Path,
              python_exe: str, workers: int, csv_path: Path,
              timeout_minimum: float, baseline_mean: float,
              overall: OverallProgress | None = None) -> List[Dict[str, object]]:
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
                    candidate, stage, rows_by_candidate[candidate.run_id]
                )
                add_baseline_deltas(combined, baseline_mean)
                combined_rows.append(combined)
                write_csv(csv_path, combined_rows)

    write_csv(csv_path, combined_rows)
    return combined_rows


def run_one_window(candidate: Candidate, stage: str, window_idx: int,
                   run_dir: Path, setup_path: Path, model_path: Path,
                   reference_path: Path, python_exe: str,
                   t_start: float, t_end: float, timeout_s: float) -> Dict[str, object]:
    start = time.monotonic()
    rc = run_simulation(
        python_exe, setup_path, model_path, run_dir, candidate, t_start, t_end, timeout_s
    )
    elapsed = time.monotonic() - start
    return collect_metrics(
        candidate, stage, run_dir, reference_path, rc, elapsed, timeout_s, window_idx
    )


def run_full_candidates(candidates: Sequence[Candidate], sweep_root: Path,
                        setup_path: Path, model_path: Path, reference_path: Path,
                        python_exe: str, workers: int, csv_path: Path,
                        timeout_minimum: float, baseline_mean: float,
                        overall: OverallProgress | None = None) -> List[Dict[str, object]]:
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
            candidate = future_map[future]
            row = future.result()
            add_baseline_deltas(row, baseline_mean)
            rows.append(row)
            job_elapsed = float(row.get("elapsed_s", 0.0) or 0.0)
            tracker.update(job_elapsed)
            if overall is not None:
                overall.update(job_elapsed)
            done += 1
            write_csv(csv_path, rows)
            line = tracker.line(done, stage, candidate.run_id, status_label(row))
            if overall is not None:
                line = f"{line} | {overall.suffix()}"
            print(line, flush=True)

    write_csv(csv_path, rows)
    return rows


def collect_baseline(baseline_dir: Path, reference_path: Path,
                     baseline_csv: Path) -> Dict[str, object]:
    candidate = Candidate(
        run_id="baseline_pd_321_500_filtered",
        knee_kp=BASE_KNEE_KP,
        knee_kd=BASE_KNEE_KD,
        knee_ki=0.0,
        ankle_kp=BASE_ANKLE_KP,
        ankle_kd=BASE_ANKLE_KD,
        ankle_ki=0.0,
    )
    row = collect_metrics(
        candidate,
        "baseline",
        baseline_dir,
        reference_path,
        return_code=0,
        elapsed_s=0.0,
        timeout_s=0.0,
        window_idx=None,
    )
    if "mean_pros_rmse_deg" in row:
        row["delta_mean_rmse_deg"] = 0.0
        row["delta_mean_rmse_pct"] = 0.0
    write_csv(baseline_csv, [row])
    return row


def acceptable_sorted(rows: Sequence[Dict[str, object]]) -> List[Dict[str, object]]:
    return [
        row for row in sorted(rows, key=row_sort_key)
        if bool(row.get("acceptable"))
    ]


def write_failures(path: Path, row_groups: Iterable[Sequence[Dict[str, object]]]) -> None:
    failures: List[Dict[str, object]] = []
    for rows in row_groups:
        failures.extend(row for row in rows if not bool(row.get("acceptable")))
    write_csv(path, failures)


def write_ranking(path: Path, rows: Sequence[Dict[str, object]]) -> List[Dict[str, object]]:
    ranking = acceptable_sorted(rows)
    write_csv(path, ranking)
    return ranking


def write_summary_json(path: Path, payload: Dict[str, object]) -> None:
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Sweep AB06 prosthetic outer PID gains for kinematic tracking."
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
    parser.add_argument("--knee-kp-grid", default=None)
    parser.add_argument("--knee-kd-grid", default=None)
    parser.add_argument("--knee-ki-grid", default=None)
    parser.add_argument("--ankle-kp-grid", default=None)
    parser.add_argument("--ankle-kd-grid", default=None)
    parser.add_argument("--ankle-ki-grid", default=None)
    return parser


def main() -> int:
    args = build_parser().parse_args()

    setup_path = resolve(args.setup)
    model_path = resolve(args.model)
    reference_path = resolve(args.reference)
    baseline_dir = resolve(args.baseline_dir)

    knee_kp = parse_grid(args.knee_kp_grid, KNEE_KP_GRID)
    knee_kd = parse_grid(args.knee_kd_grid, KNEE_KD_GRID)
    knee_ki = parse_grid(args.knee_ki_grid, KNEE_KI_GRID)
    ankle_kp = parse_grid(args.ankle_kp_grid, ANKLE_KP_GRID)
    ankle_kd = parse_grid(args.ankle_kd_grid, ANKLE_KD_GRID)
    ankle_ki = parse_grid(args.ankle_ki_grid, ANKLE_KI_GRID)

    knee_candidates = generate_knee_candidates(knee_kp, knee_kd, knee_ki)
    ankle_candidates = generate_ankle_candidates(ankle_kp, ankle_kd, ankle_ki)

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
        knee_candidates = knee_candidates[:1]
        ankle_candidates = ankle_candidates[:1]
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
        print(f"  knee_kp_grid={knee_kp}")
        print(f"  knee_kd_grid={knee_kd}")
        print(f"  knee_ki_grid={knee_ki}")
        print(f"  ankle_kp_grid={ankle_kp}")
        print(f"  ankle_kd_grid={ankle_kd}")
        print(f"  ankle_ki_grid={ankle_ki}")
        print(f"  screen_windows={windows}")
        print(f"  knee_candidates={len(knee_candidates)}")
        print(f"  ankle_candidates={len(ankle_candidates)}")
        print(f"  full_candidates={full_candidates_count}")
        print(f"  total_jobs={total_jobs}")
        print(f"  full_timeout_s={full_timeout:.1f}")
        print(
            "  progress_example="
            "[stage]  50.00% jobs=10/20 elapsed=2m00.0s "
            "eta=2m00.0s last=OK candidate_id | "
            "overall=42/133 (31.58%) total_elapsed=10m00.0s total_eta=25m00.0s"
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
        else REPO_ROOT / "results" / f"_outer_pid_gain_sweep_{stamp}"
    )
    sweep_root.mkdir(parents=True, exist_ok=True)

    print(f"[Sweep] root={sweep_root}")
    print(f"[Sweep] workers={args.workers}")
    print(f"[Sweep] total_jobs={total_jobs}")
    print(f"[Sweep] python={args.python}")
    print(f"[Sweep] full_timeout_s={full_timeout:.1f}")

    baseline_row = collect_baseline(
        baseline_dir, reference_path, sweep_root / "baseline_metrics.csv"
    )
    if "mean_pros_rmse_deg" not in baseline_row:
        raise RuntimeError(
            "Baseline metrics unavailable; cannot compute ranking deltas. "
            f"Reason: {baseline_row.get('fail_reason')}"
        )
    baseline_mean = float(baseline_row["mean_pros_rmse_deg"])
    print(
        "[Sweep] baseline mean_pros_rmse_deg="
        f"{baseline_mean:.6f} (expected {BASELINE_MEAN_PROS_RMSE_DEG:.6f})",
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
        baseline_mean,
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
        baseline_mean,
        overall,
    )

    if args.quick_smoke:
        write_failures(sweep_root / "failures.csv", [knee_rows, ankle_rows])
        write_summary_json(sweep_root / "sweep_summary.json", {
            "mode": "quick_smoke",
            "total_jobs": total_jobs,
            "baseline_mean_pros_rmse_deg": baseline_mean,
            "knee_rows": len(knee_rows),
            "ankle_rows": len(ankle_rows),
            "full_rows": 0,
        })
        print("[Sweep] quick smoke complete; full sweep not launched.", flush=True)
        return 0

    top_n = max(1, int(args.top_n_per_joint))
    top_knee = acceptable_sorted(knee_rows)[:top_n]
    top_ankle = acceptable_sorted(ankle_rows)[:top_n]
    if not top_knee or not top_ankle:
        write_failures(sweep_root / "failures.csv", [knee_rows, ankle_rows])
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
        baseline_mean,
        overall,
    )
    ranking = write_ranking(sweep_root / "ranking.csv", full_rows)
    write_failures(sweep_root / "failures.csv", [knee_rows, ankle_rows, full_rows])

    best = ranking[0] if ranking else None
    improved = (
        best is not None
        and float(best.get("mean_pros_rmse_deg", math.inf)) < baseline_mean
    )
    write_summary_json(sweep_root / "sweep_summary.json", {
        "mode": "full",
        "total_jobs": total_jobs,
        "baseline_mean_pros_rmse_deg": baseline_mean,
        "ranking_rows": len(ranking),
        "improved": improved,
        "best": best,
    })

    if best is None:
        print("[Sweep] no acceptable full candidates.", flush=True)
        return 1
    print("[Sweep] best full candidate:", flush=True)
    print(
        f"  {best['run_id']} mean={best['mean_pros_rmse_deg']:.6g} "
        f"delta={best.get('delta_mean_rmse_pct', math.nan):.3g}%",
        flush=True,
    )
    if not improved:
        print("[Sweep] best candidate did not improve the baseline.", flush=True)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
