"""
outer_gain_sweep.py
===================
Outer-loop PD gain sweep for the prosthetic SEA controller.

This script keeps the SEA plugin, model, and inner-loop properties fixed. It
only varies the Python-level outer gains:

    tau_cmd = Kp * (q_ref - q) + Kd * (qdot_ref - qdot)

No inverse-dynamics feed-forward term is reintroduced. The tau_ff diagnostic
channel may still be read from simulator outputs as an oracle comparison, but
it is not part of the SEA command.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import subprocess
import sys
import time
from collections import deque
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
from validation.hpf_noise_metric import analyze_sto, load_f_opt_from_model  # noqa: E402


SEA_KNEE = "SEA_Knee"
SEA_ANKLE = "SEA_Ankle"
COORD_KNEE = "pros_knee_angle"
COORD_ANKLE = "pros_ankle_angle"
SIDE_EFFECT_COORDS = (
    ("ankle_r", "ankle_angle_r"),
    ("mtp_r", "mtp_angle_r"),
)

KNEE_KP_GRID = [10, 20, 40, 60, 80, 120, 160, 220]
KNEE_KD_GRID = [0.5, 1, 2, 4, 6, 8, 12, 16]
ANKLE_KP_GRID = [20, 40, 80, 120, 180, 250, 350, 500, 700]
ANKLE_KD_GRID = [1, 2, 4, 6, 8, 12, 16, 24]

STAGE1_WINDOW = (4.26, 5.30)
STAGE2_WINDOWS = ((6.80, 7.20), (8.70, 9.10))
FULL_WINDOW = (4.26, 11.06)


@dataclass(frozen=True)
class Candidate:
    run_id: str
    knee_kp: float
    knee_kd: float
    ankle_kp: float
    ankle_kd: float


def fmt_num(value: float) -> str:
    return f"{value:g}".replace(".", "p").replace("-", "m")


def fmt_xml(value: float) -> str:
    return f"{value:.12g}"


def resolve(path: str | Path) -> Path:
    p = Path(path)
    return p if p.is_absolute() else REPO_ROOT / p


def parse_grid(raw: str | None, default: Sequence[float]) -> List[float]:
    if raw is None or not raw.strip():
        return [float(v) for v in default]
    return [float(item.strip()) for item in raw.split(",") if item.strip()]


def generate_candidates(
    knee_kp_grid: Sequence[float],
    knee_kd_grid: Sequence[float],
    ankle_kp_grid: Sequence[float],
    ankle_kd_grid: Sequence[float],
) -> List[Candidate]:
    candidates: List[Candidate] = []
    for knee_kp in knee_kp_grid:
        for knee_kd in knee_kd_grid:
            for ankle_kp in ankle_kp_grid:
                for ankle_kd in ankle_kd_grid:
                    run_id = (
                        f"kkp{fmt_num(knee_kp)}_kkd{fmt_num(knee_kd)}"
                        f"_akp{fmt_num(ankle_kp)}_akd{fmt_num(ankle_kd)}"
                    )
                    candidates.append(Candidate(
                        run_id=run_id,
                        knee_kp=float(knee_kp),
                        knee_kd=float(knee_kd),
                        ankle_kp=float(ankle_kp),
                        ankle_kd=float(ankle_kd),
                    ))
    return candidates


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


def candidate_base_row(candidate: Candidate) -> Dict[str, object]:
    return {
        "run_id": candidate.run_id,
        "knee_kp": candidate.knee_kp,
        "knee_kd": candidate.knee_kd,
        "ankle_kp": candidate.ankle_kp,
        "ankle_kd": candidate.ankle_kd,
    }


def command_for_run(
    python_exe: str,
    model_path: Path,
    results_dir: Path,
    candidate: Candidate,
    t_start: float,
    t_end: float,
) -> List[str]:
    return [
        python_exe,
        str(REPO_ROOT / "main.py"),
        "--model", str(model_path),
        "--t-start", fmt_xml(t_start),
        "--t-end", fmt_xml(t_end),
        "--output-dir", str(results_dir),
        "--sea-kp-knee", fmt_xml(candidate.knee_kp),
        "--sea-kd-knee", fmt_xml(candidate.knee_kd),
        "--sea-kp-ankle", fmt_xml(candidate.ankle_kp),
        "--sea-kd-ankle", fmt_xml(candidate.ankle_kd),
    ]


def run_simulation(
    python_exe: str,
    model_path: Path,
    results_dir: Path,
    candidate: Candidate,
    t_start: float,
    t_end: float,
    timeout_s: float,
) -> int:
    results_dir.mkdir(parents=True, exist_ok=True)
    cmd = command_for_run(
        python_exe, model_path, results_dir, candidate, t_start, t_end
    )
    console = results_dir / "console.txt"
    with console.open("w", encoding="utf-8", errors="replace") as fh:
        fh.write("Command:\n" + " ".join(cmd) + "\n\n")
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
    for path in results_dir.glob("sim_output_*.sto"):
        try:
            _time, _cols, data, _in_degrees = read_sto(str(path))
        except Exception:
            return False
        if not np.all(np.isfinite(data)):
            return False
    return True


def unique_time_series(time_values: np.ndarray, values: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    unique_time, indices = np.unique(time_values, return_index=True)
    order = np.argsort(indices)
    selected = indices[order]
    return time_values[selected], values[selected]


def load_column(path: Path, column: str) -> tuple[np.ndarray, np.ndarray, bool]:
    time_values, cols, data, in_degrees = read_sto(str(path))
    if column not in cols:
        raise KeyError(f"Missing column {column} in {path}")
    return (
        np.asarray(time_values, dtype=float),
        np.asarray(data[:, cols.index(column)], dtype=float),
        bool(in_degrees),
    )


def series_stats(values: np.ndarray) -> Dict[str, float]:
    return {
        "min": float(np.nanmin(values)),
        "max": float(np.nanmax(values)),
        "rms": float(np.sqrt(np.nanmean(values * values))),
        "mean_abs": float(np.nanmean(np.abs(values))),
        "max_abs": float(np.nanmax(np.abs(values))),
    }


def tracking_stats(
    results_dir: Path,
    reference_path: Path,
    coord: str,
) -> Dict[str, float]:
    sim_time, sim, sim_deg = load_column(
        results_dir / "sim_output_kinematics.sto", coord
    )
    ref_time, ref, ref_deg = load_column(reference_path, coord)
    if sim_deg:
        sim = np.deg2rad(sim)
    if ref_deg:
        ref = np.deg2rad(ref)
    ref_time, ref = unique_time_series(ref_time, ref)
    ref_i = np.interp(sim_time, ref_time, ref)
    err_deg = np.rad2deg(sim - ref_i)
    stats = series_stats(err_deg)
    return {
        "rms_deg": stats["rms"],
        "max_abs_deg": stats["max_abs"],
        "mean_abs_deg": stats["mean_abs"],
    }


def optional_stat(
    row: Dict[str, object],
    prefix: str,
    values: np.ndarray,
) -> None:
    stats = series_stats(values)
    for key, value in stats.items():
        row[f"{prefix}_{key}"] = value


def cleanup_screen_sto(results_dir: Path) -> None:
    for path in results_dir.glob("sim_output_*.sto"):
        try:
            path.unlink()
        except OSError:
            pass


def collect_metrics(
    candidate: Candidate,
    stage: str,
    results_dir: Path,
    model_path: Path,
    reference_path: Path,
    return_code: int,
    elapsed_s: float,
    cleanup_sto: bool,
) -> Dict[str, object]:
    row = candidate_base_row(candidate)
    row.update({
        "stage": stage,
        "return_code": return_code,
        "elapsed_s": elapsed_s,
        "results_dir": str(results_dir.relative_to(REPO_ROOT))
        if results_dir.is_relative_to(REPO_ROOT) else str(results_dir),
    })

    status = load_run_status(results_dir)
    row["run_status"] = status.get("status", "missing")
    row["complete"] = status.get("status") == "complete"
    row["finite_outputs"] = finite_sto_outputs(results_dir) if row["complete"] else False

    if return_code == 124:
        row["fail_reason"] = "timeout"
        row["acceptable"] = False
        row["score_total"] = math.inf
        return row
    if not row["complete"] or not row["finite_outputs"]:
        row["fail_reason"] = "incomplete_or_nonfinite"
        row["acceptable"] = False
        row["score_total"] = math.inf
        return row

    try:
        for label, coord in (
            ("knee", COORD_KNEE),
            ("ankle", COORD_ANKLE),
        ):
            stats = tracking_stats(results_dir, reference_path, coord)
            row[f"{label}_tracking_rms_deg"] = stats["rms_deg"]
            row[f"{label}_tracking_max_deg"] = stats["max_abs_deg"]
            row[f"{label}_tracking_mean_deg"] = stats["mean_abs_deg"]

        for slug, coord in SIDE_EFFECT_COORDS:
            stats = tracking_stats(results_dir, reference_path, coord)
            row[f"{slug}_tracking_rms_deg"] = stats["rms_deg"]
            row[f"{slug}_tracking_max_deg"] = stats["max_abs_deg"]
            row[f"{slug}_tracking_mean_deg"] = stats["mean_abs_deg"]

        diag_path = results_dir / "sim_output_sea_diagnostics.sto"
        control_path = results_dir / "sim_output_sea_controls.sto"
        rec_path = results_dir / "sim_output_recruitment.sto"

        max_u = 0.0
        frac_u_gt_095 = 0.0
        sat_count = 0
        max_tau_input_raw_abs = 0.0
        max_motor_speed_abs = 0.0
        max_motor_speed_dot_abs = 0.0
        max_tau_cmd_minus_pd_abs = 0.0

        for label, sea_name, coord in (
            ("knee", SEA_KNEE, COORD_KNEE),
            ("ankle", SEA_ANKLE, COORD_ANKLE),
        ):
            _t, u, _deg = load_column(control_path, coord)
            u_abs = np.abs(u)
            max_u = max(max_u, float(np.nanmax(u_abs)))
            frac = float(np.nanmean(u_abs > 0.95))
            frac_u_gt_095 = max(frac_u_gt_095, frac)
            row[f"{label}_max_u"] = float(np.nanmax(u_abs))
            row[f"{label}_frac_u_gt_095"] = frac

            _t, sat, _deg = load_column(
                diag_path, f"{sea_name}_tau_input_saturated"
            )
            sat_count += int(np.nansum(sat > 0.5))
            row[f"{label}_tau_input_saturation_count"] = int(np.nansum(sat > 0.5))

            _t, tau_raw, _deg = load_column(diag_path, f"{sea_name}_tau_input_raw")
            raw_abs = float(np.nanmax(np.abs(tau_raw)))
            max_tau_input_raw_abs = max(max_tau_input_raw_abs, raw_abs)
            row[f"{label}_max_tau_input_raw_abs"] = raw_abs

            _t, motor_speed, _deg = load_column(diag_path, f"{sea_name}_motor_speed")
            speed_abs = float(np.nanmax(np.abs(motor_speed)))
            max_motor_speed_abs = max(max_motor_speed_abs, speed_abs)
            row[f"{label}_max_motor_speed_abs"] = speed_abs

            _t, speed_dot, _deg = load_column(
                diag_path, f"{sea_name}_motor_speed_dot_plugin"
            )
            speed_dot_abs = float(np.nanmax(np.abs(speed_dot)))
            max_motor_speed_dot_abs = max(max_motor_speed_dot_abs, speed_dot_abs)
            row[f"{label}_max_motor_speed_dot_abs"] = speed_dot_abs

            _t, tau_cmd_raw, _deg = load_column(diag_path, f"{sea_name}_tau_cmd_raw")
            _t, outer_pd, _deg = load_column(diag_path, f"{sea_name}_outer_pd_cmd")
            cmd_diff = float(np.nanmax(np.abs(tau_cmd_raw - outer_pd)))
            max_tau_cmd_minus_pd_abs = max(max_tau_cmd_minus_pd_abs, cmd_diff)
            row[f"{label}_max_tau_cmd_minus_pd_abs"] = cmd_diff

            _t, tau_ff, _deg = load_column(diag_path, f"{sea_name}_tau_ff_cmd")
            optional_stat(row, f"{label}_tau_ff_diagnostic", tau_ff)

        row["sat_count"] = sat_count
        row["max_u"] = max_u
        row["frac_u_gt_095"] = frac_u_gt_095
        row["max_tau_input_raw_abs"] = max_tau_input_raw_abs
        row["max_motor_speed_abs"] = max_motor_speed_abs
        row["max_motor_speed_dot_abs"] = max_motor_speed_dot_abs
        row["max_tau_cmd_minus_pd_abs"] = max_tau_cmd_minus_pd_abs

        f_opt = load_f_opt_from_model(model_path)
        hpf = analyze_sto(
            results_dir / "sim_output_sea_torques.sto",
            control_path,
            cutoff_hz=50.0,
            skip_s=0.1,
            f_opt=f_opt,
        )
        knee_hpf = float(hpf.get("knee_hpf_noise_tau_input", math.inf))
        ankle_hpf = float(hpf.get("ankle_hpf_noise_tau_input", math.inf))
        row["knee_hpf_tau_input"] = knee_hpf
        row["ankle_hpf_tau_input"] = ankle_hpf
        row["worst_hpf_tau_input"] = max(knee_hpf, ankle_hpf)

        for name in (
            "muscle_share",
            "reserve_control_norm",
            "residual_norm",
            "muscle_capable_share",
        ):
            _t, values, _deg = load_column(rec_path, name)
            row[f"{name}_mean"] = float(np.nanmean(values))
            row[f"{name}_max"] = float(np.nanmax(values))
            row[f"{name}_min"] = float(np.nanmin(values))

        acceptable = (
            sat_count == 0
            and max_u <= 0.99
            and max_tau_input_raw_abs <= 450.0
            and max_tau_cmd_minus_pd_abs <= 1e-6
        )
        row["acceptable"] = acceptable

        fail_reasons: List[str] = []
        if sat_count:
            fail_reasons.append("tau_input_saturated")
        if max_u > 0.99:
            fail_reasons.append("max_u_gt_0p99")
        if max_tau_input_raw_abs > 450.0:
            fail_reasons.append("tau_input_raw_gt_450")
        if max_tau_cmd_minus_pd_abs > 1e-6:
            fail_reasons.append("tau_cmd_not_pd_only")
        row["fail_reason"] = ";".join(fail_reasons)

        knee_rms = float(row["knee_tracking_rms_deg"])
        ankle_rms = float(row["ankle_tracking_rms_deg"])
        knee_max = float(row["knee_tracking_max_deg"])
        ankle_max = float(row["ankle_tracking_max_deg"])
        ankle_r_rms = float(row["ankle_r_tracking_rms_deg"])
        mtp_r_rms = float(row["mtp_r_tracking_rms_deg"])

        row["score_tracking"] = knee_rms + ankle_rms
        row["score_max_error"] = 0.05 * (knee_max + ankle_max)
        row["score_noise"] = 25.0 * float(row["worst_hpf_tau_input"])
        row["score_control"] = 2.0 * max_u
        row["score_bio_side_effect"] = 0.25 * ankle_r_rms + 0.05 * mtp_r_rms
        row["score_total"] = (
            float(row["score_tracking"])
            + float(row["score_max_error"])
            + float(row["score_noise"])
            + float(row["score_control"])
            + float(row["score_bio_side_effect"])
        )
        if not acceptable:
            row["score_total"] = math.inf

    except Exception as exc:
        row["acceptable"] = False
        row["score_total"] = math.inf
        row["fail_reason"] = f"metric_error:{type(exc).__name__}:{exc}"

    if cleanup_sto and row.get("complete") and row.get("finite_outputs"):
        cleanup_screen_sto(results_dir)

    return row


CSV_FIELDS = [
    "run_id", "stage",
    "knee_kp", "knee_kd", "ankle_kp", "ankle_kd",
    "return_code", "elapsed_s", "run_status", "complete", "finite_outputs",
    "acceptable", "fail_reason", "results_dir",
    "knee_tracking_rms_deg", "knee_tracking_max_deg", "knee_tracking_mean_deg",
    "ankle_tracking_rms_deg", "ankle_tracking_max_deg", "ankle_tracking_mean_deg",
    "ankle_r_tracking_rms_deg", "ankle_r_tracking_max_deg", "ankle_r_tracking_mean_deg",
    "mtp_r_tracking_rms_deg", "mtp_r_tracking_max_deg", "mtp_r_tracking_mean_deg",
    "knee_max_u", "ankle_max_u", "max_u", "frac_u_gt_095",
    "knee_frac_u_gt_095", "ankle_frac_u_gt_095",
    "sat_count", "knee_tau_input_saturation_count", "ankle_tau_input_saturation_count",
    "knee_max_tau_input_raw_abs", "ankle_max_tau_input_raw_abs",
    "max_tau_input_raw_abs",
    "knee_hpf_tau_input", "ankle_hpf_tau_input", "worst_hpf_tau_input",
    "knee_max_motor_speed_abs", "ankle_max_motor_speed_abs", "max_motor_speed_abs",
    "knee_max_motor_speed_dot_abs", "ankle_max_motor_speed_dot_abs",
    "max_motor_speed_dot_abs",
    "knee_max_tau_cmd_minus_pd_abs", "ankle_max_tau_cmd_minus_pd_abs",
    "max_tau_cmd_minus_pd_abs",
    "muscle_share_mean", "muscle_share_min", "muscle_share_max",
    "reserve_control_norm_mean", "reserve_control_norm_min", "reserve_control_norm_max",
    "residual_norm_mean", "residual_norm_max",
    "muscle_capable_share_mean", "muscle_capable_share_min",
    "score_tracking", "score_max_error", "score_noise", "score_control",
    "score_bio_side_effect", "score_total",
    "knee_tau_ff_diagnostic_rms", "ankle_tau_ff_diagnostic_rms",
]


def write_csv(path: Path, rows: Sequence[Dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in CSV_FIELDS})


def row_sort_key(row: Dict[str, object]) -> float:
    try:
        return float(row.get("score_total", math.inf))
    except (TypeError, ValueError):
        return math.inf


def timeout_for_window(t_start: float, t_end: float, minimum: float) -> float:
    return max(minimum, 180.0 * max(0.01, t_end - t_start))


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
        self.workers = max(1, min(int(workers), self.total_jobs))
        self.started_at = time.monotonic()
        self._job_elapsed_s = deque(maxlen=max(24, self.workers * 4))

    def update(self, job_elapsed_s: float) -> None:
        if math.isfinite(job_elapsed_s) and job_elapsed_s > 0:
            self._job_elapsed_s.append(float(job_elapsed_s))

    def text(self, done_jobs: int) -> str:
        done = max(0, min(int(done_jobs), self.total_jobs))
        elapsed = time.monotonic() - self.started_at
        pct = 100.0 * done / self.total_jobs
        remaining = self.total_jobs - done
        if remaining <= 0:
            eta = 0.0
        elif not self._job_elapsed_s:
            eta = math.inf
        else:
            avg_job_s = sum(self._job_elapsed_s) / len(self._job_elapsed_s)
            eta = avg_job_s * remaining / self.workers
        eta_text = "n/a" if not math.isfinite(eta) else format_duration(eta)
        return (
            f"{pct:6.2f}% elapsed={format_duration(elapsed)} "
            f"eta={eta_text}"
        )


def screen_one_window(
    candidate: Candidate,
    stage: str,
    window_idx: int,
    t_start: float,
    t_end: float,
    sweep_root: Path,
    model_path: Path,
    reference_path: Path,
    python_exe: str,
    timeout_s: float,
    cleanup_sto: bool,
) -> Dict[str, object]:
    run_dir = sweep_root / "runs" / stage / f"{candidate.run_id}_w{window_idx}"
    start = time.monotonic()
    rc = run_simulation(
        python_exe, model_path, run_dir, candidate, t_start, t_end, timeout_s
    )
    elapsed = time.monotonic() - start
    return collect_metrics(
        candidate, stage, run_dir, model_path, reference_path,
        rc, elapsed, cleanup_sto,
    )


def combine_stage_rows(
    candidate: Candidate,
    stage: str,
    rows: Sequence[Dict[str, object]],
) -> Dict[str, object]:
    combined = candidate_base_row(candidate)
    combined["stage"] = stage
    combined["results_dir"] = ";".join(str(row.get("results_dir", "")) for row in rows)
    combined["return_code"] = max(int(row.get("return_code", 0) or 0) for row in rows)
    combined["elapsed_s"] = sum(float(row.get("elapsed_s", 0.0) or 0.0) for row in rows)
    combined["complete"] = all(bool(row.get("complete")) for row in rows)
    combined["finite_outputs"] = all(bool(row.get("finite_outputs")) for row in rows)
    combined["acceptable"] = all(bool(row.get("acceptable")) for row in rows)
    combined["fail_reason"] = ";".join(
        str(row.get("fail_reason", ""))
        for row in rows if row.get("fail_reason")
    )
    combined["run_status"] = "complete" if combined["complete"] else "incomplete"

    for field in CSV_FIELDS:
        if field in combined or field in {
            "run_id", "stage", "knee_kp", "knee_kd", "ankle_kp", "ankle_kd",
            "return_code", "elapsed_s", "run_status", "complete",
            "finite_outputs", "acceptable", "fail_reason", "results_dir",
        }:
            continue
        values = []
        for row in rows:
            value = row.get(field)
            if isinstance(value, (int, float)):
                values.append(float(value))
        if not values:
            continue
        if field == "sat_count" or field.endswith("_count"):
            combined[field] = sum(values)
        elif field.startswith("score_") or field.endswith("_rms_deg"):
            combined[field] = max(values)
        elif field.endswith("_min"):
            combined[field] = min(values)
        else:
            combined[field] = max(values)

    if not combined["acceptable"]:
        combined["score_total"] = math.inf
    return combined


def run_stage(
    stage: str,
    candidates: Sequence[Candidate],
    windows: Sequence[tuple[float, float]],
    sweep_root: Path,
    model_path: Path,
    reference_path: Path,
    python_exe: str,
    workers: int,
    csv_path: Path,
    cleanup_sto: bool,
    timeout_minimum: float,
) -> List[Dict[str, object]]:
    rows: List[Dict[str, object]] = []
    total_jobs = len(candidates) * len(windows)
    print(
        f"[Sweep] {stage}: {len(candidates)} candidate(s), "
        f"{len(windows)} window(s), {workers} worker(s)",
        flush=True,
    )
    progress = ProgressTracker(total_jobs, workers)

    with ThreadPoolExecutor(max_workers=max(1, min(workers, total_jobs))) as pool:
        future_map = {}
        for candidate in candidates:
            for window_idx, (t_start, t_end) in enumerate(windows, start=1):
                timeout_s = timeout_for_window(t_start, t_end, timeout_minimum)
                future = pool.submit(
                    screen_one_window,
                    candidate,
                    stage,
                    window_idx,
                    t_start,
                    t_end,
                    sweep_root,
                    model_path,
                    reference_path,
                    python_exe,
                    timeout_s,
                    cleanup_sto,
                )
                future_map[future] = (candidate, window_idx)

        per_candidate: Dict[str, List[Dict[str, object]]] = {
            candidate.run_id: [] for candidate in candidates
        }
        done_jobs = 0
        for future in as_completed(future_map):
            candidate, _window_idx = future_map[future]
            row = future.result()
            progress.update(float(row.get("elapsed_s", 0.0) or 0.0))
            per_candidate[candidate.run_id].append(row)
            done_jobs += 1
            if len(per_candidate[candidate.run_id]) == len(windows):
                combined = combine_stage_rows(
                    candidate, stage, per_candidate[candidate.run_id]
                )
                rows.append(combined)
                write_csv(csv_path, rows)
                status = "OK" if combined.get("acceptable") else "NO"
                print(
                    f"[{stage} {len(rows):04d}/{len(candidates):04d} "
                    f"jobs={done_jobs}/{total_jobs} "
                    f"{progress.text(done_jobs)}] "
                    f"{status} "
                    f"{combined['run_id']} score={combined.get('score_total')}",
                    flush=True,
                )

    write_csv(csv_path, rows)
    return rows


def run_full(
    candidates: Sequence[Candidate],
    sweep_root: Path,
    model_path: Path,
    reference_path: Path,
    python_exe: str,
    workers: int,
    csv_path: Path,
    timeout_minimum: float,
) -> List[Dict[str, object]]:
    rows: List[Dict[str, object]] = []
    t_start, t_end = FULL_WINDOW
    print(
        f"[Sweep] full: {len(candidates)} candidate(s), {workers} worker(s)",
        flush=True,
    )
    progress = ProgressTracker(len(candidates), workers)
    with ThreadPoolExecutor(max_workers=max(1, min(workers, len(candidates)))) as pool:
        future_map = {}
        for candidate in candidates:
            run_dir = sweep_root / "full_runs" / candidate.run_id
            timeout_s = timeout_for_window(t_start, t_end, timeout_minimum)
            future = pool.submit(
                _run_full_candidate,
                candidate,
                run_dir,
                model_path,
                reference_path,
                python_exe,
                timeout_s,
            )
            future_map[future] = candidate

        for future in as_completed(future_map):
            row = future.result()
            progress.update(float(row.get("elapsed_s", 0.0) or 0.0))
            rows.append(row)
            write_csv(csv_path, rows)
            status = "OK" if row.get("acceptable") else "NO"
            print(
                f"[full {len(rows):03d}/{len(candidates):03d} "
                f"{progress.text(len(rows))}] "
                f"{status} "
                f"{row.get('run_id')} score={row.get('score_total')}",
                flush=True,
            )
    write_csv(csv_path, rows)
    return rows


def _run_full_candidate(
    candidate: Candidate,
    run_dir: Path,
    model_path: Path,
    reference_path: Path,
    python_exe: str,
    timeout_s: float,
) -> Dict[str, object]:
    t_start, t_end = FULL_WINDOW
    start = time.monotonic()
    rc = run_simulation(
        python_exe, model_path, run_dir, candidate, t_start, t_end, timeout_s
    )
    elapsed = time.monotonic() - start
    return collect_metrics(
        candidate, "full", run_dir, model_path, reference_path,
        rc, elapsed, cleanup_sto=False,
    )


def write_best_json(
    path: Path,
    rows: Sequence[Dict[str, object]],
    python_exe: str,
    model_path: Path,
) -> None:
    best = sorted(rows, key=row_sort_key)
    payload = []
    for row in best:
        candidate = Candidate(
            run_id=str(row["run_id"]),
            knee_kp=float(row["knee_kp"]),
            knee_kd=float(row["knee_kd"]),
            ankle_kp=float(row["ankle_kp"]),
            ankle_kd=float(row["ankle_kd"]),
        )
        rerun_dir = f"results/_outer_best_{candidate.run_id}"
        cmd = command_for_run(
            python_exe,
            model_path,
            Path(rerun_dir),
            candidate,
            FULL_WINDOW[0],
            FULL_WINDOW[1],
        )
        item = {key: row.get(key) for key in CSV_FIELDS if key in row}
        item["rerun_command"] = " ".join(cmd)
        payload.append(item)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Sweep prosthetic outer PD gains with fixed SEA inner loop."
    )
    parser.add_argument("--workers", type=int, default=12)
    parser.add_argument("--python", default=sys.executable)
    parser.add_argument("--template", default="models/Adjusted_SEASEA - Copia_tuned.osim")
    parser.add_argument("--reference", default="data/3DGaitModel2392_Kinematics_q.sto")
    parser.add_argument("--sweep-root", default=None)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--quick-smoke", action="store_true")
    parser.add_argument("--top-n-stage2", type=int, default=96)
    parser.add_argument("--top-n-full", type=int, default=12)
    parser.add_argument("--timeout-minimum", type=float, default=300.0)
    parser.add_argument("--keep-screen-sto", action="store_true")
    parser.add_argument("--knee-kp-grid", default=None)
    parser.add_argument("--knee-kd-grid", default=None)
    parser.add_argument("--ankle-kp-grid", default=None)
    parser.add_argument("--ankle-kd-grid", default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    model_path = resolve(args.template)
    reference_path = resolve(args.reference)

    knee_kp_grid = parse_grid(args.knee_kp_grid, KNEE_KP_GRID)
    knee_kd_grid = parse_grid(args.knee_kd_grid, KNEE_KD_GRID)
    ankle_kp_grid = parse_grid(args.ankle_kp_grid, ANKLE_KP_GRID)
    ankle_kd_grid = parse_grid(args.ankle_kd_grid, ANKLE_KD_GRID)
    candidates = generate_candidates(
        knee_kp_grid, knee_kd_grid, ankle_kp_grid, ankle_kd_grid
    )

    if args.quick_smoke:
        candidates = candidates[:3]
        stage1_windows = ((4.26, 4.29),)
        stage2_windows: Sequence[tuple[float, float]] = ()
    else:
        stage1_windows = (STAGE1_WINDOW,)
        stage2_windows = STAGE2_WINDOWS

    if args.dry_run:
        print("[Sweep] dry run")
        print(f"  knee_kp_grid={knee_kp_grid}")
        print(f"  knee_kd_grid={knee_kd_grid}")
        print(f"  ankle_kp_grid={ankle_kp_grid}")
        print(f"  ankle_kd_grid={ankle_kd_grid}")
        print(f"  candidates={len(candidates)}")
        print(f"  stage1_windows={stage1_windows}")
        print(f"  stage2_windows={stage2_windows}")
        print(f"  top_n_stage2={args.top_n_stage2}")
        print(f"  top_n_full={args.top_n_full}")
        return 0

    if not model_path.is_file():
        raise FileNotFoundError(f"Model not found: {model_path}")
    if not reference_path.is_file():
        raise FileNotFoundError(f"Reference not found: {reference_path}")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    sweep_root = (
        resolve(args.sweep_root)
        if args.sweep_root
        else REPO_ROOT / "results" / f"_outer_gain_sweep_{stamp}"
    )
    sweep_root.mkdir(parents=True, exist_ok=True)
    cleanup_sto = not args.keep_screen_sto

    print(f"[Sweep] root={sweep_root}")
    print(f"[Sweep] candidates={len(candidates)}")
    print(f"[Sweep] python={args.python}")
    print(f"[Sweep] model={model_path}")
    print(f"[Sweep] cleanup_screen_sto={cleanup_sto}")

    stage1_rows = run_stage(
        "stage1",
        candidates,
        stage1_windows,
        sweep_root,
        model_path,
        reference_path,
        args.python,
        args.workers,
        sweep_root / "screen_stage1.csv",
        cleanup_sto,
        args.timeout_minimum,
    )

    ordered_stage1 = [
        row for row in sorted(stage1_rows, key=row_sort_key)
        if bool(row.get("acceptable"))
    ]
    if args.quick_smoke:
        print(f"[Sweep] quick smoke completed rows={len(stage1_rows)}")
        return 0
    if not ordered_stage1:
        print("[Sweep] no acceptable stage1 candidates")
        return 1

    stage2_candidates = [
        Candidate(
            run_id=str(row["run_id"]),
            knee_kp=float(row["knee_kp"]),
            knee_kd=float(row["knee_kd"]),
            ankle_kp=float(row["ankle_kp"]),
            ankle_kd=float(row["ankle_kd"]),
        )
        for row in ordered_stage1[:max(0, args.top_n_stage2)]
    ]
    stage2_rows = run_stage(
        "stage2",
        stage2_candidates,
        stage2_windows,
        sweep_root,
        model_path,
        reference_path,
        args.python,
        args.workers,
        sweep_root / "screen_stage2.csv",
        cleanup_sto,
        args.timeout_minimum,
    )

    ordered_stage2 = [
        row for row in sorted(stage2_rows, key=row_sort_key)
        if bool(row.get("acceptable"))
    ]
    if args.top_n_full <= 0:
        print("[Sweep] full runs disabled")
        write_best_json(
            sweep_root / "best_candidates.json",
            ordered_stage2[: max(0, args.top_n_stage2)],
            args.python,
            model_path,
        )
        return 0 if ordered_stage2 else 1
    if not ordered_stage2:
        print("[Sweep] no acceptable stage2 candidates")
        return 1

    full_candidates = [
        Candidate(
            run_id=str(row["run_id"]),
            knee_kp=float(row["knee_kp"]),
            knee_kd=float(row["knee_kd"]),
            ankle_kp=float(row["ankle_kp"]),
            ankle_kd=float(row["ankle_kd"]),
        )
        for row in ordered_stage2[:max(0, args.top_n_full)]
    ]
    full_rows = run_full(
        full_candidates,
        sweep_root,
        model_path,
        reference_path,
        args.python,
        args.workers,
        sweep_root / "full_results.csv",
        args.timeout_minimum,
    )
    ordered_full = sorted(full_rows, key=row_sort_key)
    write_best_json(
        sweep_root / "best_candidates.json",
        ordered_full,
        args.python,
        model_path,
    )

    print("[Sweep] best full candidates:")
    for row in ordered_full[:10]:
        print(
            f"  {row.get('run_id')} score={row.get('score_total')} "
            f"knee_rms={row.get('knee_tracking_rms_deg')} "
            f"ankle_rms={row.get('ankle_tracking_rms_deg')} "
            f"max_u={row.get('max_u')}",
            flush=True,
        )
    return 0 if ordered_full else 1


if __name__ == "__main__":
    raise SystemExit(main())
