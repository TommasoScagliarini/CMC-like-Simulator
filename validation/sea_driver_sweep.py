"""
SEA Motor-Driver Parameter Sweep
================================
Finds optimal K / Kp / Kd for both SEA actuators so the inner PD loop
behaves as a transparent motor driver across the widest operating range.

Mode: non-impedance (Impedence=false).
The new ``max_motor_torque`` property in the plugin is set to a high value
(default 5000 Nm) so the inner PD can use higher gains without clamping.

Formulas (linearised non-impedance dynamics):
    Kp = Jm * omega_n^2 / K - 1
    Kd = 2 * zeta * Jm * omega_n - Bm

The grid sweeps K_knee x K_ankle x omega_n x zeta.  Candidates that
violate the clamp pre-filter are discarded before simulation.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from output import read_sto  # noqa: E402

# ── Constants ────────────────────────────────────────────────────────────────
SEA_KNEE = "SEA_Knee"
SEA_ANKLE = "SEA_Ankle"
COORD_KNEE = "pros_knee_angle"
COORD_ANKLE = "pros_ankle_angle"
SEA_NAMES = (SEA_KNEE, SEA_ANKLE)

# Default grids
K_KNEE_GRID  = [250, 500, 750, 1000]
K_ANKLE_GRID = [500, 750, 1000, 1500]
OMEGA_N_GRID = [300, 500, 700, 900, 1100, 1400]
ZETA_GRID    = [0.7, 0.85, 1.0]

MAX_MOTOR_TORQUE_SWEEP = 5000.0    # set in each candidate .osim
CLAMP_MARGIN_FRACTION  = 0.90      # pre-filter accepts Kp*F_opt < margin
CONTROL_ACCEPT_MAX     = 0.95

# Python executable for running simulations (resolved at startup)
PYTHON_EXE: str = sys.executable

# Scoring weights
W_WORST_TRACKING = 0.40
W_MEAN_TRACKING  = 0.20
W_NOISE_FRAC     = 0.20
W_TAU_RAW        = 0.10
W_SPEED_DOT      = 0.10


# ── Data classes ─────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class SeaParams:
    stiffness: float
    kp: float
    kd: float


@dataclass(frozen=True)
class Candidate:
    run_id: str
    stage: str
    knee: SeaParams
    ankle: SeaParams
    omega_n: float
    zeta: float
    t_start: float
    t_end: float
    max_motor_torque: float


# ── Helpers ──────────────────────────────────────────────────────────────────
def fmt_num(value: float) -> str:
    return f"{value:g}".replace(".", "p").replace("-", "m")


def fmt_xml(value: float) -> str:
    return f"{value:.12g}"


def resolve(path: str | Path) -> Path:
    p = Path(path)
    return p if p.is_absolute() else REPO_ROOT / p


def xml_local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


# ── CLI ──────────────────────────────────────────────────────────────────────
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="SEA motor-driver parameter sweep")
    p.add_argument("--workers", type=int, default=12)
    p.add_argument("--dry-run", action="store_true",
                   help="Print candidate grid and exit (no simulation)")
    p.add_argument("--quick-smoke", action="store_true",
                   help="Run 2-3 candidates on a tiny window for build check")
    p.add_argument("--template",
                   default="models/Adjusted_SEASEA - Copia_tuned.osim")
    p.add_argument("--reference",
                   default="data/3DGaitModel2392_Kinematics_q.sto")
    p.add_argument("--screen-t-start", type=float, default=4.26)
    p.add_argument("--screen-t-end",   type=float, default=6.55)
    p.add_argument("--full-t-start",   type=float, default=4.26)
    p.add_argument("--full-t-end",     type=float, default=11.06)
    p.add_argument("--max-motor-torque", type=float,
                   default=MAX_MOTOR_TORQUE_SWEEP)
    p.add_argument("--top-n-full", type=int, default=5,
                   help="Number of top screening candidates for full run")
    p.add_argument("--python", default=None,
                   help="Python executable (must have opensim). "
                        "Auto-detects envCMC-like conda env if omitted.")
    return p.parse_args()


# ── Template parsing ─────────────────────────────────────────────────────────
def parse_template_props(
    model_path: Path,
) -> Dict[str, Dict[str, float | bool]]:
    from xml.etree import ElementTree as ET
    tree = ET.parse(model_path)
    props: Dict[str, Dict[str, float | bool]] = {}
    for element in tree.getroot().iter():
        if xml_local_name(element.tag) != "SeriesElasticActuator":
            continue
        name = element.attrib.get("name", "")
        if name not in SEA_NAMES:
            continue
        values = {
            xml_local_name(child.tag): (child.text or "").strip()
            for child in element
        }
        props[name] = {
            "K":    float(values["stiffness"]),
            "Kp":   float(values["Kp"]),
            "Kd":   float(values["Kd"]),
            "Jm":   float(values["motor_inertia"]),
            "Bm":   float(values["motor_damping"]),
            "F_opt": float(values["optimal_force"]),
            "impedance": values.get("Impedence", "false").lower() == "true",
        }
    missing = set(SEA_NAMES) - set(props)
    if missing:
        raise ValueError(f"Missing SEA actuator(s) in template: {sorted(missing)}")
    return props


# ── Gain derivation ──────────────────────────────────────────────────────────
def derive_params(
    stiffness: float, jm: float, bm: float,
    omega_n: float, zeta: float,
) -> Optional[SeaParams]:
    """Return SeaParams or None if the combination is physically invalid."""
    if stiffness <= 0 or omega_n <= 0 or zeta <= 0:
        return None
    kp = jm * omega_n * omega_n / stiffness - 1.0
    kd = 2.0 * zeta * jm * omega_n - bm
    if kp <= 0.0 or kd < 0.0:
        return None
    return SeaParams(float(stiffness), float(kp), float(kd))


def passes_clamp_prefilter(
    kp: float, f_opt: float, max_torque: float, margin: float,
) -> bool:
    """True if worst-case initial tau_input stays within margin."""
    return kp * f_opt <= max_torque * margin


# ── XML editing ──────────────────────────────────────────────────────────────
def replace_tag(block: str, tag: str, value: str) -> str:
    pattern = rf"(<{tag}>)(.*?)(</{tag}>)"
    replacement = rf"\g<1>{value}\g<3>"
    new_block, count = re.subn(pattern, replacement, block, count=1, flags=re.S)
    if count == 1:
        return new_block
    # Tag not found: insert before closing </SeriesElasticActuator>
    insert_line = f"\t\t\t\t\t<{tag}>{value}</{tag}>\n"
    close_tag = "</SeriesElasticActuator>"
    idx = block.rfind(close_tag)
    if idx == -1:
        raise ValueError(f"Cannot insert <{tag}>: no closing tag found")
    return block[:idx] + insert_line + block[idx:]


def replace_sea_block(text: str, sea_name: str, params: SeaParams,
                      max_motor_torque: float) -> str:
    pattern = (
        rf'(<SeriesElasticActuator name="{re.escape(sea_name)}">.*?'
        r'</SeriesElasticActuator>)'
    )

    def repl(match: re.Match[str]) -> str:
        block = match.group(1)
        block = replace_tag(block, "stiffness", fmt_xml(params.stiffness))
        block = replace_tag(block, "Kp", fmt_xml(params.kp))
        block = replace_tag(block, "Kd", fmt_xml(params.kd))
        block = replace_tag(block, "max_motor_torque",
                            fmt_xml(max_motor_torque))
        return block

    new_text, count = re.subn(pattern, repl, text, count=1, flags=re.S)
    if count != 1:
        raise ValueError(f"Could not find SEA block for {sea_name}")
    return new_text


def write_candidate_model(
    template: Path, out_path: Path, candidate: Candidate,
) -> None:
    text = template.read_text(encoding="utf-8", errors="replace")
    text = replace_sea_block(text, SEA_KNEE, candidate.knee,
                             candidate.max_motor_torque)
    text = replace_sea_block(text, SEA_ANKLE, candidate.ankle,
                             candidate.max_motor_torque)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(text, encoding="utf-8")


# ── Candidate generation ─────────────────────────────────────────────────────
def generate_candidates(
    props: Dict[str, Dict[str, float | bool]],
    k_knee_grid: List[float],
    k_ankle_grid: List[float],
    omega_n_grid: List[float],
    zeta_grid: List[float],
    max_motor_torque: float,
    margin: float,
    stage: str,
    t_start: float,
    t_end: float,
) -> List[Candidate]:
    jm_knee  = float(props[SEA_KNEE]["Jm"])
    bm_knee  = float(props[SEA_KNEE]["Bm"])
    jm_ankle = float(props[SEA_ANKLE]["Jm"])
    bm_ankle = float(props[SEA_ANKLE]["Bm"])
    f_opt_knee  = float(props[SEA_KNEE]["F_opt"])
    f_opt_ankle = float(props[SEA_ANKLE]["F_opt"])

    candidates: List[Candidate] = []
    rejected = 0

    for omega_n in omega_n_grid:
        for zeta in zeta_grid:
            for k_knee in k_knee_grid:
                knee = derive_params(k_knee, jm_knee, bm_knee, omega_n, zeta)
                if knee is None:
                    rejected += 1
                    continue
                if not passes_clamp_prefilter(
                    knee.kp, f_opt_knee, max_motor_torque, margin,
                ):
                    rejected += 1
                    continue

                for k_ankle in k_ankle_grid:
                    ankle = derive_params(
                        k_ankle, jm_ankle, bm_ankle, omega_n, zeta,
                    )
                    if ankle is None:
                        rejected += 1
                        continue
                    if not passes_clamp_prefilter(
                        ankle.kp, f_opt_ankle, max_motor_torque, margin,
                    ):
                        rejected += 1
                        continue

                    run_id = (
                        f"{stage}_kk{fmt_num(k_knee)}_ka{fmt_num(k_ankle)}"
                        f"_wn{fmt_num(omega_n)}_z{fmt_num(zeta)}"
                    )
                    candidates.append(Candidate(
                        run_id=run_id,
                        stage=stage,
                        knee=knee,
                        ankle=ankle,
                        omega_n=omega_n,
                        zeta=zeta,
                        t_start=t_start,
                        t_end=t_end,
                        max_motor_torque=max_motor_torque,
                    ))

    return candidates


# ── Metrics ──────────────────────────────────────────────────────────────────
def sto_series(path: Path, col: str) -> Tuple[np.ndarray, np.ndarray]:
    t, cols, data, _ = read_sto(str(path))
    idx = cols.index(col)
    return t, data[:, idx]


def rms(v: np.ndarray) -> float:
    return float(np.sqrt(np.nanmean(v * v)))


def max_abs(v: np.ndarray) -> float:
    return float(np.nanmax(np.abs(v)))


def tracking_rms_deg(
    results_dir: Path, reference_path: Path, coord: str,
) -> float:
    sim_t, sim_cols, sim_data, _ = read_sto(
        str(results_dir / "sim_output_kinematics.sto"))
    ref_t, ref_cols, ref_data, ref_deg = read_sto(str(reference_path))
    sim = sim_data[:, sim_cols.index(coord)]
    ref = ref_data[:, ref_cols.index(coord)]
    if ref_deg:
        ref = np.deg2rad(ref)
    err_rad = sim - np.interp(sim_t, ref_t, ref)
    return rms(np.rad2deg(err_rad))


def noise_fraction(tau_input: np.ndarray, tau_ref: np.ndarray) -> float:
    """Return 1 - R^2 between tau_input and tau_ref (0 = perfect, 1 = noise)."""
    if len(tau_input) < 10:
        return 1.0
    corr = np.corrcoef(tau_input, tau_ref)[0, 1]
    if not np.isfinite(corr):
        return 1.0
    return float(1.0 - corr * corr)


def read_run_status(results_dir: Path) -> Dict[str, str]:
    path = results_dir / "sim_output_run_status.txt"
    if not path.is_file():
        return {}
    values: Dict[str, str] = {}
    for raw_line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if "=" not in raw_line:
            continue
        key, value = raw_line.split("=", 1)
        values[key.strip()] = value.strip()
    return values


def table_has_only_finite(path: Path) -> bool:
    if not path.is_file():
        return False
    _, _, data, _ = read_sto(str(path))
    return bool(np.all(np.isfinite(data)))


def collect_metrics(
    candidate: Candidate,
    results_dir: Path,
    reference_path: Path,
    return_code: int,
    elapsed_s: float,
) -> Dict[str, object]:
    row: Dict[str, object] = {
        "run_id":    candidate.run_id,
        "stage":     candidate.stage,
        "knee_K":    candidate.knee.stiffness,
        "knee_Kp":   candidate.knee.kp,
        "knee_Kd":   candidate.knee.kd,
        "ankle_K":   candidate.ankle.stiffness,
        "ankle_Kp":  candidate.ankle.kp,
        "ankle_Kd":  candidate.ankle.kd,
        "omega_n":   candidate.omega_n,
        "zeta":      candidate.zeta,
        "max_motor_torque": candidate.max_motor_torque,
        "t_start":   candidate.t_start,
        "t_end":     candidate.t_end,
        "return_code": return_code,
        "elapsed_s": elapsed_s,
        "results_dir": str(results_dir.relative_to(REPO_ROOT)),
    }
    status = read_run_status(results_dir)
    row["run_status"] = status.get("status", "missing")
    row["complete"]   = status.get("status") == "complete"
    row["validator_ok"] = return_code == 0

    required_files = [
        "sim_output_sea_diagnostics.sto",
        "sim_output_sea_controls.sto",
        "sim_output_power.sto",
        "sim_output_sea_states.sto",
        "sim_output_sea_derivatives.sto",
        "sim_output_kinematics.sto",
    ]
    missing = [n for n in required_files if not (results_dir / n).is_file()]
    if missing:
        row["acceptable"] = False
        row["score"] = math.inf
        row["fail_reason"] = f"missing files: {', '.join(missing)}"
        return row

    finite_files = [
        "sim_output_sea_diagnostics.sto",
        "sim_output_power.sto",
        "sim_output_sea_states.sto",
        "sim_output_sea_derivatives.sto",
    ]
    row["finite_outputs"] = all(
        table_has_only_finite(results_dir / n) for n in finite_files
    )

    try:
        diag_path = results_dir / "sim_output_sea_diagnostics.sto"
        control_path = results_dir / "sim_output_sea_controls.sto"
        power_path = results_dir / "sim_output_power.sto"

        max_raw = 0.0
        sat_count = 0
        max_speed_dot = 0.0
        noise_fracs: List[float] = []
        motor_power_rms_values: List[float] = []

        for sea_name in SEA_NAMES:
            _, tau_input = sto_series(diag_path, f"{sea_name}_tau_input_plugin")
            _, tau_ref   = sto_series(diag_path, f"{sea_name}_tau_ref")
            _, sat       = sto_series(diag_path, f"{sea_name}_tau_input_saturated")
            _, speed_dot = sto_series(
                diag_path, f"{sea_name}_motor_speed_dot_plugin")
            _, motor_pwr = sto_series(power_path, f"{sea_name}_motor_power")

            max_raw = max(max_raw, max_abs(tau_input))
            sat_count += int(np.sum(sat > 0.5))
            max_speed_dot = max(max_speed_dot, max_abs(speed_dot))
            # Skip initial transient (first 100 samples) for noise calc
            skip = min(100, len(tau_input) // 4)
            noise_fracs.append(
                noise_fraction(tau_input[skip:], tau_ref[skip:]))
            motor_power_rms_values.append(rms(motor_pwr))

        max_u = 0.0
        for coord in (COORD_KNEE, COORD_ANKLE):
            _, control = sto_series(control_path, coord)
            max_u = max(max_u, max_abs(control))

        knee_tracking  = tracking_rms_deg(results_dir, reference_path, COORD_KNEE)
        ankle_tracking = tracking_rms_deg(results_dir, reference_path, COORD_ANKLE)

        worst_tracking = max(knee_tracking, ankle_tracking)
        mean_tracking  = 0.5 * (knee_tracking + ankle_tracking)
        worst_noise    = max(noise_fracs)
        max_motor_power_rms = max(motor_power_rms_values)

        score = (
            W_WORST_TRACKING * worst_tracking
            + W_MEAN_TRACKING * mean_tracking
            + W_NOISE_FRAC * worst_noise * 10.0
            + W_TAU_RAW * (max_raw / 1000.0)
            + W_SPEED_DOT * (max_speed_dot / 10000.0)
        )

        clamp_limit = candidate.max_motor_torque * CLAMP_MARGIN_FRACTION
        reasons: List[str] = []
        if not row["complete"]:
            reasons.append("simulation incomplete")
        if not row["validator_ok"]:
            reasons.append("validator FAIL or process error")
        if sat_count != 0:
            reasons.append(f"tau_input saturated in {sat_count} samples")
        if max_raw > clamp_limit:
            reasons.append(
                f"tau_input_raw max {max_raw:.1f} > {clamp_limit:.0f}")
        if max_u >= CONTROL_ACCEPT_MAX:
            reasons.append(f"max |u| {max_u:.3f} >= {CONTROL_ACCEPT_MAX:g}")
        if not row["finite_outputs"]:
            reasons.append("non-finite outputs")

        row.update({
            "sat_count":              sat_count,
            "max_tau_input_raw_abs":  max_raw,
            "max_u":                  max_u,
            "max_motor_speed_dot_abs": max_speed_dot,
            "max_motor_power_rms_abs": max_motor_power_rms,
            "knee_tracking_rms_deg":  knee_tracking,
            "ankle_tracking_rms_deg": ankle_tracking,
            "knee_noise_frac":        noise_fracs[0],
            "ankle_noise_frac":       noise_fracs[1],
            "worst_noise_frac":       worst_noise,
            "score":                  score,
            "acceptable":             not reasons,
            "fail_reason":            "; ".join(reasons),
        })
        return row
    except Exception as exc:
        row["acceptable"] = False
        row["score"] = math.inf
        row["fail_reason"] = f"metric error: {exc}"
        return row


# ── CSV ──────────────────────────────────────────────────────────────────────
CSV_FIELDS = [
    "run_id", "stage",
    "knee_K", "knee_Kp", "knee_Kd",
    "ankle_K", "ankle_Kp", "ankle_Kd",
    "omega_n", "zeta", "max_motor_torque",
    "t_start", "t_end",
    "return_code", "elapsed_s", "run_status",
    "complete", "validator_ok", "finite_outputs",
    "sat_count", "max_tau_input_raw_abs", "max_u",
    "max_motor_speed_dot_abs", "max_motor_power_rms_abs",
    "knee_tracking_rms_deg", "ankle_tracking_rms_deg",
    "knee_noise_frac", "ankle_noise_frac", "worst_noise_frac",
    "score", "acceptable", "fail_reason", "results_dir",
]


def write_csv(path: Path, rows: List[Dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({f: row.get(f, "") for f in CSV_FIELDS})


# ── Subprocess execution ─────────────────────────────────────────────────────
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


def run_candidate(
    candidate: Candidate,
    template: Path,
    sweep_root: Path,
    reference_path: Path,
) -> Dict[str, object]:
    model_path  = sweep_root / "models" / f"{candidate.run_id}.osim"
    results_dir = sweep_root / "runs" / candidate.run_id
    results_dir.mkdir(parents=True, exist_ok=True)
    write_candidate_model(template, model_path, candidate)

    cmd = [
        PYTHON_EXE,
        str(REPO_ROOT / "main.py"),
        "--model", str(model_path),
        "--output-dir", str(results_dir),
        "--t-start", fmt_xml(candidate.t_start),
        "--t-end", fmt_xml(candidate.t_end),
        "--validate",
    ]
    console_path = results_dir / "console.txt"
    start = time.monotonic()
    with console_path.open("w", encoding="utf-8", errors="replace") as log_fh:
        log_fh.write("Command:\n")
        log_fh.write(" ".join(cmd) + "\n\n")
        log_fh.flush()
        subprocess.run(
            cmd, cwd=str(REPO_ROOT),
            stdout=log_fh, stderr=subprocess.STDOUT,
            env=subprocess_env(), check=False,
        )
    elapsed = time.monotonic() - start
    return collect_metrics(
        candidate, results_dir, reference_path,
        0,  # validator exit code captured via run_status file
        elapsed,
    )


def run_batch(
    candidates: List[Candidate],
    template: Path,
    sweep_root: Path,
    reference_path: Path,
    workers: int,
    csv_path: Path,
    all_rows: List[Dict[str, object]],
    label: str = "",
) -> List[Dict[str, object]]:
    if not candidates:
        return []

    n = len(candidates)
    tag = f" [{label}]" if label else ""
    print(
        f"\n[Sweep]{tag} Running {n} candidate(s) with {workers} worker(s) ...",
        flush=True,
    )
    batch_t0 = time.monotonic()
    done = 0
    batch_rows: List[Dict[str, object]] = []
    effective_workers = max(1, min(workers, n))

    with ThreadPoolExecutor(max_workers=effective_workers) as pool:
        future_map = {
            pool.submit(
                run_candidate, c, template, sweep_root, reference_path
            ): c
            for c in candidates
        }
        for future in as_completed(future_map):
            candidate = future_map[future]
            try:
                row = future.result()
            except Exception as exc:
                row = {
                    "run_id": candidate.run_id,
                    "stage": candidate.stage,
                    "knee_K": candidate.knee.stiffness,
                    "knee_Kp": candidate.knee.kp,
                    "knee_Kd": candidate.knee.kd,
                    "ankle_K": candidate.ankle.stiffness,
                    "ankle_Kp": candidate.ankle.kp,
                    "ankle_Kd": candidate.ankle.kd,
                    "omega_n": candidate.omega_n,
                    "zeta": candidate.zeta,
                    "acceptable": False,
                    "score": math.inf,
                    "fail_reason": f"runner exception: {exc}",
                }

            all_rows.append(row)
            batch_rows.append(row)
            done += 1
            write_csv(csv_path, all_rows)

            # ── Progress log ─────────────────────────────────────────────
            elapsed = time.monotonic() - batch_t0
            pct = done / n * 100
            if done < n:
                eta = elapsed / done * (n - done)
                eta_str = f"ETA={eta:.0f}s"
            else:
                eta_str = "done"

            score_val = row.get("score", math.inf)
            score_txt = (
                f"{float(score_val):.3f}"
                if np.isfinite(float(score_val)) else "inf"
            )
            ok = row.get("acceptable", False)
            print(
                f"  [{done}/{n}] ({pct:.0f}%) elapsed={elapsed:.0f}s {eta_str}"
                f"  {row['run_id']}  score={score_txt} ok={ok}"
                f"  {row.get('fail_reason', '')}",
                flush=True,
            )

    total = time.monotonic() - batch_t0
    n_ok = sum(1 for r in batch_rows if r.get("acceptable"))
    print(
        f"[Sweep]{tag} Batch done: {n} runs, {n_ok} acceptable, "
        f"{total:.0f}s total",
        flush=True,
    )
    return batch_rows


# ── Sorting / selection ──────────────────────────────────────────────────────
def score_key(row: Dict[str, object]) -> Tuple[int, float]:
    acceptable = bool(row.get("acceptable"))
    score = float(row.get("score", math.inf))
    complete = bool(row.get("complete"))
    return (0 if acceptable else 1 if complete else 2, score)


# ── Report generation ────────────────────────────────────────────────────────
def generate_report(
    report_path: Path,
    sweep_root: Path,
    rows: List[Dict[str, object]],
    best: Optional[Dict[str, object]],
) -> None:
    ranked = sorted(rows, key=score_key)

    def cf(row: Dict[str, object], key: str, d: int = 3) -> str:
        v = row.get(key)
        if v in (None, ""):
            return ""
        try:
            return f"{float(v):.{d}f}"
        except (TypeError, ValueError):
            return ""

    lines = [
        "# SEA Motor-Driver Parameter Sweep",
        "",
        f"Data: {datetime.now().strftime('%Y-%m-%d %H:%M')}",
        "",
        "## Formula (non-impedance)",
        "",
        "```",
        "Kp = Jm * omega_n^2 / K - 1",
        "Kd = 2 * zeta * Jm * omega_n - Bm",
        "```",
        "",
    ]

    if best is None:
        lines.append("**Nessuna soluzione accettabile trovata.**")
        lines.append("")
    else:
        lines.extend([
            "## Best Candidate",
            "",
            f"- Run: `{best['run_id']}`",
            f"- Knee:  K={best['knee_K']}, Kp={float(best['knee_Kp']):.4g}, Kd={float(best['knee_Kd']):.4g}",
            f"- Ankle: K={best['ankle_K']}, Kp={float(best['ankle_Kp']):.4g}, Kd={float(best['ankle_Kd']):.4g}",
            f"- omega_n={best.get('omega_n')}, zeta={best.get('zeta')}",
            f"- Score: {float(best['score']):.4f}",
            f"- Tracking: knee={cf(best, 'knee_tracking_rms_deg')} deg, ankle={cf(best, 'ankle_tracking_rms_deg')} deg",
            f"- Noise fraction: knee={cf(best, 'knee_noise_frac')}, ankle={cf(best, 'ankle_noise_frac')}",
            f"- Max |tau_input|: {cf(best, 'max_tau_input_raw_abs')} Nm",
            f"- Max |u|: {cf(best, 'max_u')}",
            "",
        ])

    lines.extend([
        "## Top Candidates",
        "",
        "| # | Run | OK | Score | Kk | Ka | wn | z | Knee RMS | Ankle RMS | Noise | Max tau | Reason |",
        "|--:|-----|:--:|------:|---:|---:|---:|--:|---------:|----------:|------:|--------:|--------|",
    ])
    for i, row in enumerate(ranked[:20], 1):
        s = float(row.get("score", math.inf))
        st = f"{s:.3f}" if np.isfinite(s) else "inf"
        lines.append(
            f"| {i} | `{row.get('run_id','')}` "
            f"| {'Y' if row.get('acceptable') else 'N'} "
            f"| {st} "
            f"| {row.get('knee_K','')} | {row.get('ankle_K','')} "
            f"| {row.get('omega_n','')} | {row.get('zeta','')} "
            f"| {cf(row,'knee_tracking_rms_deg')} "
            f"| {cf(row,'ankle_tracking_rms_deg')} "
            f"| {cf(row,'worst_noise_frac')} "
            f"| {cf(row,'max_tau_input_raw_abs',0)} "
            f"| {str(row.get('fail_reason','')).replace('|','/')} |"
        )

    lines.extend([
        "",
        "## Files",
        "",
        f"- CSV: `{sweep_root / 'sweep_results.csv'}`",
        f"- JSON: `{sweep_root / 'best_candidate.json'}`",
    ])
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


# ── Dry run ──────────────────────────────────────────────────────────────────
def print_dry_run(
    props: Dict[str, Dict[str, float | bool]],
    max_motor_torque: float,
) -> None:
    candidates = generate_candidates(
        props,
        K_KNEE_GRID, K_ANKLE_GRID, OMEGA_N_GRID, ZETA_GRID,
        max_motor_torque, CLAMP_MARGIN_FRACTION,
        "dry", 0, 0,
    )
    total_grid = (
        len(K_KNEE_GRID) * len(K_ANKLE_GRID)
        * len(OMEGA_N_GRID) * len(ZETA_GRID)
    )
    print(f"Grid total: {total_grid}")
    print(f"After pre-filter: {len(candidates)}")
    print(f"max_motor_torque: {max_motor_torque}")
    print()
    print(
        f"{'omega_n':>8} {'zeta':>5} {'K_knee':>7} {'K_ankle':>8} "
        f"{'Kp_knee':>8} {'Kd_knee':>8} {'Kp_ankle':>9} {'Kd_ankle':>9} "
        f"{'peak_knee':>10} {'peak_ankle':>11}"
    )
    for c in sorted(candidates, key=lambda c: (c.omega_n, c.zeta,
                                                c.knee.stiffness,
                                                c.ankle.stiffness)):
        pk = c.knee.kp * float(props[SEA_KNEE]["F_opt"])
        pa = c.ankle.kp * float(props[SEA_ANKLE]["F_opt"])
        print(
            f"{c.omega_n:8g} {c.zeta:5.2f} {c.knee.stiffness:7g} "
            f"{c.ankle.stiffness:8g} "
            f"{c.knee.kp:8.3f} {c.knee.kd:8.3f} "
            f"{c.ankle.kp:9.3f} {c.ankle.kd:9.3f} "
            f"{pk:10.1f} {pa:11.1f}"
        )


# ── Main ─────────────────────────────────────────────────────────────────────
def _resolve_python(cli_python: Optional[str]) -> str:
    """Return a Python executable that can import opensim."""
    global PYTHON_EXE
    if cli_python:
        PYTHON_EXE = cli_python
        return PYTHON_EXE
    # Auto-detect: try envCMC-like conda env first, then opensim env
    # Conda envs live under {conda_prefix}/envs/
    conda_prefix = Path(os.environ.get("CONDA_PREFIX", sys.prefix))
    # If we're inside an env, go up to the root conda install
    envs_dir = conda_prefix / "envs"
    if not envs_dir.is_dir():
        envs_dir = conda_prefix.parent  # e.g. /opt/anaconda3/envs/foo -> parent
        if envs_dir.name == "envs":
            pass  # already correct
        else:
            envs_dir = conda_prefix.parent / "envs"
    for env_name in ("envCMC-like", "opensim"):
        candidate = envs_dir / env_name / "bin" / "python"
        if not candidate.exists():
            # Windows layout
            candidate = envs_dir / env_name / "python.exe"
        if candidate.exists():
            PYTHON_EXE = str(candidate)
            return PYTHON_EXE
    # Fallback: current interpreter
    PYTHON_EXE = sys.executable
    return PYTHON_EXE


def main() -> int:
    wall_t0 = time.monotonic()
    args = parse_args()

    py = _resolve_python(args.python)
    print(f"[Sweep] Python: {py}", flush=True)

    template = resolve(args.template)
    reference_path = resolve(args.reference)
    props = parse_template_props(template)
    max_torque = args.max_motor_torque

    if args.dry_run:
        print_dry_run(props, max_torque)
        return 0

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    suffix = "quick_smoke" if args.quick_smoke else stamp
    sweep_root = REPO_ROOT / "results" / f"_sea_driver_sweep_{suffix}"
    csv_path   = sweep_root / "sweep_results.csv"
    report_path = (
        REPO_ROOT / "reports" / "user"
        / f"{datetime.now().strftime('%Y-%m-%d')}_sea_driver_sweep.md"
    )
    sweep_root.mkdir(parents=True, exist_ok=True)
    all_rows: List[Dict[str, object]] = []

    # ── Quick smoke ──────────────────────────────────────────────────────
    if args.quick_smoke:
        smoke = []
        for omega_n in [500, 700]:
            p_knee = derive_params(250, 0.01, 0.1, omega_n, 0.85)
            p_ankle = derive_params(500, 0.01, 0.1, omega_n, 0.85)
            if p_knee and p_ankle:
                smoke.append(Candidate(
                    run_id=f"smoke_wn{omega_n}",
                    stage="smoke",
                    knee=p_knee, ankle=p_ankle,
                    omega_n=omega_n, zeta=0.85,
                    t_start=4.26, t_end=4.30,
                    max_motor_torque=max_torque,
                ))
        if not smoke:
            print("[Sweep] No valid smoke candidates", flush=True)
            return 1
        run_batch(
            smoke, template, sweep_root, reference_path,
            args.workers, csv_path, all_rows, label="smoke",
        )
        ok = all(r.get("complete") for r in all_rows)
        print(f"\n[Sweep] Smoke {'PASS' if ok else 'FAIL'}", flush=True)
        return 0 if ok else 1

    # ── Screening ────────────────────────────────────────────────────────
    screen_candidates = generate_candidates(
        props,
        K_KNEE_GRID, K_ANKLE_GRID, OMEGA_N_GRID, ZETA_GRID,
        max_torque, CLAMP_MARGIN_FRACTION,
        "screen",
        args.screen_t_start, args.screen_t_end,
    )
    print(
        f"[Sweep] Screening: {len(screen_candidates)} candidates "
        f"(from {len(K_KNEE_GRID)*len(K_ANKLE_GRID)*len(OMEGA_N_GRID)*len(ZETA_GRID)} grid points)",
        flush=True,
    )
    screen_rows = run_batch(
        screen_candidates, template, sweep_root, reference_path,
        args.workers, csv_path, all_rows, label="screen",
    )

    # ── Select top N for full run ────────────────────────────────────────
    ranked = sorted(screen_rows, key=score_key)
    top_n = args.top_n_full
    full_candidates: List[Candidate] = []
    for row in ranked[:top_n]:
        if float(row.get("score", math.inf)) == math.inf:
            continue
        knee = SeaParams(
            float(row["knee_K"]), float(row["knee_Kp"]),
            float(row["knee_Kd"]),
        )
        ankle = SeaParams(
            float(row["ankle_K"]), float(row["ankle_Kp"]),
            float(row["ankle_Kd"]),
        )
        run_id = f"full_{row['run_id'].removeprefix('screen_')}"
        full_candidates.append(Candidate(
            run_id=run_id,
            stage="full",
            knee=knee, ankle=ankle,
            omega_n=float(row.get("omega_n", 0)),
            zeta=float(row.get("zeta", 0)),
            t_start=args.full_t_start,
            t_end=args.full_t_end,
            max_motor_torque=max_torque,
        ))

    if not full_candidates:
        print("[Sweep] No candidates passed screening.", flush=True)
        generate_report(report_path, sweep_root, all_rows, None)
        return 1

    print(
        f"\n[Sweep] Full validation: {len(full_candidates)} candidate(s)",
        flush=True,
    )
    full_rows = run_batch(
        full_candidates, template, sweep_root, reference_path,
        min(args.workers, len(full_candidates)),
        csv_path, all_rows, label="full",
    )

    # ── Find best ────────────────────────────────────────────────────────
    acceptable = [r for r in full_rows if r.get("acceptable")]
    best: Optional[Dict[str, object]] = None
    if acceptable:
        best = sorted(acceptable, key=score_key)[0]

    best_json = sweep_root / "best_candidate.json"
    if best is None:
        best_json.write_text(
            json.dumps({"found": False}, indent=2), encoding="utf-8")
        generate_report(report_path, sweep_root, all_rows, None)
        print("\n[Sweep] No acceptable solution found.", flush=True)
    else:
        best_json.write_text(
            json.dumps({"found": True, "best": best}, indent=2,
                       default=str),
            encoding="utf-8",
        )
        generate_report(report_path, sweep_root, all_rows, best)
        print(f"\n[Sweep] Best: {best['run_id']}", flush=True)
        print(
            f"  Knee:  K={best['knee_K']}, Kp={float(best['knee_Kp']):.4g}, "
            f"Kd={float(best['knee_Kd']):.4g}",
            flush=True,
        )
        print(
            f"  Ankle: K={best['ankle_K']}, Kp={float(best['ankle_Kp']):.4g}, "
            f"Kd={float(best['ankle_Kd']):.4g}",
            flush=True,
        )

    total_elapsed = time.monotonic() - wall_t0
    print(f"\n[Sweep] Total elapsed: {total_elapsed:.0f}s", flush=True)
    print(f"[Sweep] Report: {report_path}", flush=True)
    print(f"[Sweep] CSV: {csv_path}", flush=True)
    return 0 if best is not None else 1


if __name__ == "__main__":
    raise SystemExit(main())
