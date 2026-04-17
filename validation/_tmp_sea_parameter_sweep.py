"""
Temporary SEA parameter sweep.

This script is intentionally self-deleting after a successful real sweep.
It varies SEA stiffness values and derives Kp/Kd so that the inner SEA loop has
fixed natural frequency and damping ratio.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import shutil
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple
from xml.etree import ElementTree as ET

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from output import read_sto  # noqa: E402


SEA_KNEE = "SEA_Knee"
SEA_ANKLE = "SEA_Ankle"
COORD_KNEE = "pros_knee_angle"
COORD_ANKLE = "pros_ankle_angle"
SEA_NAMES = (SEA_KNEE, SEA_ANKLE)
GRID = [75, 100, 125, 150, 175, 200, 250, 300, 350, 500, 750, 1000]
OMEGA_N = 500.0
ZETA = 0.7
TAU_INPUT_LIMIT = 500.0
TAU_INPUT_RAW_ACCEPT_MAX = 480.0
CONTROL_ACCEPT_MAX = 0.95


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
    t_start: float
    t_end: float

    @property
    def pair(self) -> Tuple[float, float]:
        return (self.knee.stiffness, self.ankle.stiffness)


def fmt_num(value: float) -> str:
    text = f"{value:g}"
    return text.replace(".", "p").replace("-", "m")


def fmt_xml(value: float) -> str:
    return f"{value:.12g}"


def resolve(path: str | Path) -> Path:
    p = Path(path)
    return p if p.is_absolute() else REPO_ROOT / p


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Temporary SEA K/Kp/Kd sweep")
    parser.add_argument("--workers", type=int, default=12)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--quick-smoke", action="store_true")
    parser.add_argument(
        "--template",
        default="models/Adjusted_SEASEA - Copia_tuned.osim",
    )
    parser.add_argument("--reference", default="data/3DGaitModel2392_Kinematics_q.sto")
    parser.add_argument("--screen-t-start", type=float, default=4.26)
    parser.add_argument("--screen-t-end", type=float, default=6.55)
    parser.add_argument("--full-t-start", type=float, default=4.26)
    parser.add_argument("--full-t-end", type=float, default=11.06)
    parser.add_argument("--omega-n", type=float, default=OMEGA_N)
    parser.add_argument("--zeta", type=float, default=ZETA)
    return parser.parse_args()


def xml_local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def parse_template_props(model_path: Path) -> Dict[str, Dict[str, float | bool]]:
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
            "K": float(values["stiffness"]),
            "Kp": float(values["Kp"]),
            "Kd": float(values["Kd"]),
            "Jm": float(values["motor_inertia"]),
            "Bm": float(values["motor_damping"]),
            "F_opt": float(values["optimal_force"]),
            "impedance": values["Impedence"].lower() == "true",
        }
    missing = set(SEA_NAMES) - set(props)
    if missing:
        raise ValueError(f"Missing SEA actuator(s) in template: {sorted(missing)}")
    for sea_name, sea_props in props.items():
        if sea_props["impedance"]:
            raise ValueError(f"{sea_name} has Impedence=true; this sweep expects false")
    return props


def derive_params(stiffness: float, jm: float, bm: float, omega_n: float, zeta: float) -> SeaParams:
    kp = jm * omega_n * omega_n / stiffness - 1.0
    kd = 2.0 * zeta * jm * omega_n - bm
    if stiffness <= 0.0 or kp <= 0.0 or kd < 0.0 or stiffness >= jm * omega_n * omega_n:
        raise ValueError(
            f"Invalid candidate: K={stiffness:g}, Kp={kp:g}, Kd={kd:g}, "
            f"Jm={jm:g}, Bm={bm:g}, omega_n={omega_n:g}, zeta={zeta:g}"
        )
    return SeaParams(float(stiffness), float(kp), float(kd))


def replace_tag(block: str, tag: str, value: float) -> str:
    pattern = rf"(<{tag}>)(.*?)(</{tag}>)"
    replacement = rf"\g<1>{fmt_xml(value)}\g<3>"
    new_block, count = re.subn(pattern, replacement, block, count=1, flags=re.S)
    if count != 1:
        raise ValueError(f"Could not replace <{tag}> inside SEA block")
    return new_block


def replace_sea_block(text: str, sea_name: str, params: SeaParams) -> str:
    pattern = rf'(<SeriesElasticActuator name="{re.escape(sea_name)}">.*?</SeriesElasticActuator>)'

    def repl(match: re.Match[str]) -> str:
        block = match.group(1)
        block = replace_tag(block, "stiffness", params.stiffness)
        block = replace_tag(block, "Kp", params.kp)
        block = replace_tag(block, "Kd", params.kd)
        return block

    new_text, count = re.subn(pattern, repl, text, count=1, flags=re.S)
    if count != 1:
        raise ValueError(f"Could not find SEA block for {sea_name}")
    return new_text


def write_candidate_model(template: Path, out_path: Path, candidate: Candidate) -> None:
    text = template.read_text(encoding="utf-8", errors="replace")
    text = replace_sea_block(text, SEA_KNEE, candidate.knee)
    text = replace_sea_block(text, SEA_ANKLE, candidate.ankle)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(text, encoding="utf-8")


def update_config_stiffness(config_path: Path, knee_k: float, ankle_k: float) -> None:
    text = config_path.read_text(encoding="utf-8")
    text, knee_count = re.subn(
        r'("SEA_Knee"\s*:\s*)[-+0-9.eE]+(,)',
        rf"\g<1>{fmt_xml(knee_k)}\g<2>",
        text,
        count=1,
    )
    text, ankle_count = re.subn(
        r'("SEA_Ankle"\s*:\s*)[-+0-9.eE]+(,)',
        rf"\g<1>{fmt_xml(ankle_k)}\g<2>",
        text,
        count=1,
    )
    if knee_count != 1 or ankle_count != 1:
        raise ValueError("Could not update config.py sea_stiffness values")
    config_path.write_text(text, encoding="utf-8")


def make_candidate(
    stage: str,
    knee_k: float,
    ankle_k: float,
    props: Dict[str, Dict[str, float | bool]],
    t_start: float,
    t_end: float,
    omega_n: float,
    zeta: float,
) -> Candidate:
    knee = derive_params(
        knee_k,
        float(props[SEA_KNEE]["Jm"]),
        float(props[SEA_KNEE]["Bm"]),
        omega_n,
        zeta,
    )
    ankle = derive_params(
        ankle_k,
        float(props[SEA_ANKLE]["Jm"]),
        float(props[SEA_ANKLE]["Bm"]),
        omega_n,
        zeta,
    )
    run_id = f"{stage}_k{fmt_num(knee_k)}_a{fmt_num(ankle_k)}"
    return Candidate(run_id, stage, knee, ankle, t_start, t_end)


def sto_series(path: Path, col: str) -> Tuple[np.ndarray, np.ndarray]:
    time, cols, data, _in_degrees = read_sto(str(path))
    try:
        idx = cols.index(col)
    except ValueError as exc:
        raise KeyError(f"Missing column {col} in {path}") from exc
    return time, data[:, idx]


def table_has_only_finite(path: Path) -> bool:
    if not path.is_file():
        return False
    _time, _cols, data, _in_degrees = read_sto(str(path))
    return bool(np.all(np.isfinite(data)))


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


def rms(values: np.ndarray) -> float:
    return float(np.sqrt(np.nanmean(values * values)))


def max_abs(values: np.ndarray) -> float:
    return float(np.nanmax(np.abs(values)))


def tracking_rms_deg(results_dir: Path, reference_path: Path, coord: str) -> float:
    sim_time, sim_cols, sim_data, _sim_deg = read_sto(str(results_dir / "sim_output_kinematics.sto"))
    ref_time, ref_cols, ref_data, ref_deg = read_sto(str(reference_path))
    sim = sim_data[:, sim_cols.index(coord)]
    ref = ref_data[:, ref_cols.index(coord)]
    if ref_deg:
        ref = np.deg2rad(ref)
    err_rad = sim - np.interp(sim_time, ref_time, ref)
    return rms(np.rad2deg(err_rad))


def collect_metrics(
    candidate: Candidate,
    results_dir: Path,
    reference_path: Path,
    return_code: int,
    elapsed_s: float,
) -> Dict[str, object]:
    row: Dict[str, object] = {
        "run_id": candidate.run_id,
        "stage": candidate.stage,
        "knee_K": candidate.knee.stiffness,
        "knee_Kp": candidate.knee.kp,
        "knee_Kd": candidate.knee.kd,
        "ankle_K": candidate.ankle.stiffness,
        "ankle_Kp": candidate.ankle.kp,
        "ankle_Kd": candidate.ankle.kd,
        "t_start": candidate.t_start,
        "t_end": candidate.t_end,
        "return_code": return_code,
        "elapsed_s": elapsed_s,
        "results_dir": str(results_dir.relative_to(REPO_ROOT)),
    }
    status = read_run_status(results_dir)
    row["run_status"] = status.get("status", "missing")
    row["complete"] = status.get("status") == "complete"
    row["validator_ok"] = return_code == 0

    required_files = [
        "sim_output_sea_diagnostics.sto",
        "sim_output_sea_controls.sto",
        "sim_output_power.sto",
        "sim_output_sea_states.sto",
        "sim_output_sea_derivatives.sto",
        "sim_output_kinematics.sto",
    ]
    missing = [name for name in required_files if not (results_dir / name).is_file()]
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
    row["finite_outputs"] = all(table_has_only_finite(results_dir / name) for name in finite_files)

    try:
        diag_path = results_dir / "sim_output_sea_diagnostics.sto"
        control_path = results_dir / "sim_output_sea_controls.sto"
        power_path = results_dir / "sim_output_power.sto"

        max_raw = 0.0
        sat_count = 0
        max_speed_dot = 0.0
        tau_error_rms_values: List[float] = []
        motor_power_rms_values: List[float] = []
        for sea_name in SEA_NAMES:
            _t, raw = sto_series(diag_path, f"{sea_name}_tau_input_raw")
            _t, sat = sto_series(diag_path, f"{sea_name}_tau_input_saturated")
            _t, speed_dot = sto_series(diag_path, f"{sea_name}_motor_speed_dot_plugin")
            _t, tau_error = sto_series(diag_path, f"{sea_name}_tau_error")
            _t, motor_power = sto_series(power_path, f"{sea_name}_motor_power")
            max_raw = max(max_raw, max_abs(raw))
            sat_count += int(np.sum(sat > 0.5))
            max_speed_dot = max(max_speed_dot, max_abs(speed_dot))
            tau_error_rms_values.append(rms(tau_error))
            motor_power_rms_values.append(rms(motor_power))

        max_u = 0.0
        for coord in (COORD_KNEE, COORD_ANKLE):
            _t, control = sto_series(control_path, coord)
            max_u = max(max_u, max_abs(control))

        knee_tracking = tracking_rms_deg(results_dir, reference_path, COORD_KNEE)
        ankle_tracking = tracking_rms_deg(results_dir, reference_path, COORD_ANKLE)

        worst_tracking = max(knee_tracking, ankle_tracking)
        mean_tracking = 0.5 * (knee_tracking + ankle_tracking)
        max_motor_power_rms = max(motor_power_rms_values)

        score = (
            0.55 * worst_tracking
            + 0.25 * mean_tracking
            + 0.10 * (max_speed_dot / 10000.0)
            + 0.05 * (max_raw / 100.0)
            + 0.05 * (max_motor_power_rms / 1000.0)
        )

        row.update(
            {
                "sat_count": sat_count,
                "max_tau_input_raw_abs": max_raw,
                "max_u": max_u,
                "max_motor_speed_dot_abs": max_speed_dot,
                "max_motor_power_rms_abs": max_motor_power_rms,
                "knee_tracking_rms_deg": knee_tracking,
                "ankle_tracking_rms_deg": ankle_tracking,
                "mean_tau_error_rms": float(np.mean(tau_error_rms_values)),
                "score": score,
            }
        )

        reasons: List[str] = []
        if not row["complete"]:
            reasons.append("simulation incomplete")
        if not row["validator_ok"]:
            reasons.append("validator FAIL or process error")
        if sat_count != 0:
            reasons.append(f"tau_input saturated in {sat_count} samples")
        if max_raw > TAU_INPUT_RAW_ACCEPT_MAX:
            reasons.append(f"tau_input_raw max {max_raw:.3f} > {TAU_INPUT_RAW_ACCEPT_MAX:g}")
        if max_u >= CONTROL_ACCEPT_MAX:
            reasons.append(f"max |u| {max_u:.3f} >= {CONTROL_ACCEPT_MAX:g}")
        if not row["finite_outputs"]:
            reasons.append("non-finite outputs")

        row["acceptable"] = not reasons
        row["fail_reason"] = "; ".join(reasons)
        return row
    except Exception as exc:
        row["acceptable"] = False
        row["score"] = math.inf
        row["fail_reason"] = f"metric error: {exc}"
        return row


CSV_FIELDS = [
    "run_id",
    "stage",
    "knee_K",
    "knee_Kp",
    "knee_Kd",
    "ankle_K",
    "ankle_Kp",
    "ankle_Kd",
    "t_start",
    "t_end",
    "return_code",
    "elapsed_s",
    "run_status",
    "complete",
    "validator_ok",
    "finite_outputs",
    "sat_count",
    "max_tau_input_raw_abs",
    "max_u",
    "max_motor_speed_dot_abs",
    "max_motor_power_rms_abs",
    "knee_tracking_rms_deg",
    "ankle_tracking_rms_deg",
    "mean_tau_error_rms",
    "score",
    "acceptable",
    "fail_reason",
    "results_dir",
]


def write_csv(path: Path, rows: List[Dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in CSV_FIELDS})


def subprocess_env() -> Dict[str, str]:
    env = os.environ.copy()
    env.update(
        {
            "OMP_NUM_THREADS": "1",
            "MKL_NUM_THREADS": "1",
            "OPENBLAS_NUM_THREADS": "1",
            "NUMEXPR_NUM_THREADS": "1",
            "PYTHONIOENCODING": "utf-8",
        }
    )
    return env


def run_candidate(
    candidate: Candidate,
    template: Path,
    sweep_root: Path,
    reference_path: Path,
) -> Dict[str, object]:
    model_path = sweep_root / "models" / f"{candidate.run_id}.osim"
    results_dir = sweep_root / "runs" / candidate.run_id
    results_dir.mkdir(parents=True, exist_ok=True)
    write_candidate_model(template, model_path, candidate)

    cmd = [
        sys.executable,
        str(REPO_ROOT / "main.py"),
        "--model",
        str(model_path),
        "--output-dir",
        str(results_dir),
        "--t-start",
        fmt_xml(candidate.t_start),
        "--t-end",
        fmt_xml(candidate.t_end),
        "--validate",
    ]
    console_path = results_dir / "console.txt"
    start = time.monotonic()
    with console_path.open("w", encoding="utf-8", errors="replace") as log_fh:
        log_fh.write("Command:\n")
        log_fh.write(" ".join(cmd) + "\n\n")
        log_fh.flush()
        completed = subprocess.run(
            cmd,
            cwd=str(REPO_ROOT),
            stdout=log_fh,
            stderr=subprocess.STDOUT,
            env=subprocess_env(),
            check=False,
        )
    elapsed = time.monotonic() - start
    row = collect_metrics(candidate, results_dir, reference_path, completed.returncode, elapsed)
    return row


def run_batch(
    candidates: List[Candidate],
    template: Path,
    sweep_root: Path,
    reference_path: Path,
    workers: int,
    csv_path: Path,
    rows: List[Dict[str, object]],
) -> List[Dict[str, object]]:
    if not candidates:
        return []
    print(f"[Sweep] Running {len(candidates)} candidate(s) with workers={workers} ...", flush=True)
    completed_rows: List[Dict[str, object]] = []
    with ThreadPoolExecutor(max_workers=max(1, min(workers, len(candidates)))) as pool:
        future_map = {
            pool.submit(run_candidate, candidate, template, sweep_root, reference_path): candidate
            for candidate in candidates
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
                    "t_start": candidate.t_start,
                    "t_end": candidate.t_end,
                    "acceptable": False,
                    "score": math.inf,
                    "fail_reason": f"runner exception: {exc}",
                }
            rows.append(row)
            completed_rows.append(row)
            write_csv(csv_path, rows)
            score = row.get("score", math.inf)
            score_text = f"{float(score):.4f}" if np.isfinite(float(score)) else "inf"
            print(
                f"[Sweep] {row['run_id']} done: acceptable={row.get('acceptable')} "
                f"score={score_text} reason={row.get('fail_reason', '')}",
                flush=True,
            )
    return completed_rows


def score_key(row: Dict[str, object]) -> Tuple[int, float]:
    acceptable = bool(row.get("acceptable"))
    score = float(row.get("score", math.inf))
    complete = bool(row.get("complete"))
    return (0 if acceptable else 1 if complete else 2, score)


def rows_to_candidates(
    rows: Iterable[Dict[str, object]],
    props: Dict[str, Dict[str, float | bool]],
    stage: str,
    t_start: float,
    t_end: float,
    omega_n: float,
    zeta: float,
) -> List[Candidate]:
    candidates: List[Candidate] = []
    seen: set[Tuple[float, float]] = set()
    for row in sorted(rows, key=score_key):
        pair = (float(row["knee_K"]), float(row["ankle_K"]))
        if pair in seen:
            continue
        seen.add(pair)
        candidates.append(
            make_candidate(stage, pair[0], pair[1], props, t_start, t_end, omega_n, zeta)
        )
    return candidates


def top_unique_values(rows: Iterable[Dict[str, object]], key: str, count: int) -> List[float]:
    values: List[float] = []
    for row in sorted(rows, key=score_key):
        value = float(row[key])
        if value not in values:
            values.append(value)
        if len(values) >= count:
            break
    return values


def generate_report(
    report_path: Path,
    sweep_root: Path,
    rows: List[Dict[str, object]],
    best: Optional[Dict[str, object]],
    applied: bool,
) -> None:
    ranked = sorted(rows, key=score_key)
    lines = [
        "# SEA Parameter Sweep",
        "",
        f"Data: {datetime.now().strftime('%Y-%m-%d %H:%M')}",
        "",
        "## Formula",
        "",
        "`Kp = Jm * omega_n^2 / K - 1`",
        "",
        "`Kd = 2 * zeta * Jm * omega_n - Bm`",
        "",
        "con `omega_n = 500 rad/s` e `zeta = 0.7`.",
        "",
        "## Risultato",
        "",
    ]
    if best is None:
        lines.extend([
            "Nessuna soluzione accettabile trovata.",
            "",
            f"Sweep root: `{sweep_root}`",
            "",
        ])
    else:
        lines.extend([
            "Soluzione accettabile trovata." if applied else "Soluzione candidata trovata, non applicata.",
            "",
            f"- Run: `{best['run_id']}`",
            f"- Results: `{best['results_dir']}`",
            f"- Knee: `K={best['knee_K']}`, `Kp={float(best['knee_Kp']):.6g}`, `Kd={float(best['knee_Kd']):.6g}`",
            f"- Ankle: `K={best['ankle_K']}`, `Kp={float(best['ankle_Kp']):.6g}`, `Kd={float(best['ankle_Kd']):.6g}`",
            f"- Score: `{float(best['score']):.6g}`",
            f"- Saturazioni tau_input: `{best.get('sat_count')}`",
            f"- Max tau_input_raw: `{float(best['max_tau_input_raw_abs']):.3f} Nm`",
            f"- Max |u|: `{float(best['max_u']):.3f}`",
            "",
        ])
    lines.extend([
        "## Top Candidates",
        "",
        "| Rank | Run | Acceptable | Score | Knee K | Ankle K | Knee RMS deg | Ankle RMS deg | Max raw Nm | Reason |",
        "|---:|---|---:|---:|---:|---:|---:|---:|---:|---|",
    ])

    def cell_float(row: Dict[str, object], key: str, digits: int = 3) -> str:
        value = row.get(key)
        if value in (None, ""):
            return ""
        try:
            return f"{float(value):.{digits}f}"
        except (TypeError, ValueError):
            return ""

    for i, row in enumerate(ranked[:12], start=1):
        score = float(row.get("score", math.inf))
        score_text = f"{score:.4f}" if np.isfinite(score) else "inf"
        lines.append(
            "| {rank} | `{run}` | {acc} | {score} | {kk} | {ak} | {krms} | {arms} | {raw} | {reason} |".format(
                rank=i,
                run=row.get("run_id", ""),
                acc=row.get("acceptable", ""),
                score=score_text,
                kk=row.get("knee_K", ""),
                ak=row.get("ankle_K", ""),
                krms=cell_float(row, "knee_tracking_rms_deg"),
                arms=cell_float(row, "ankle_tracking_rms_deg"),
                raw=cell_float(row, "max_tau_input_raw_abs"),
                reason=str(row.get("fail_reason", "")).replace("|", "/"),
            )
        )
    lines.extend([
        "",
        "## Files",
        "",
        f"- Sweep CSV: `{sweep_root / 'sweep_results.csv'}`",
        f"- Best JSON: `{sweep_root / 'best_candidate.json'}`",
    ])
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def cleanup_after_success(sweep_root: Path, winning_results_dir: Path) -> None:
    runs_dir = sweep_root / "runs"
    if runs_dir.is_dir():
        for child in runs_dir.iterdir():
            if child.resolve() == winning_results_dir.resolve():
                continue
            if child.is_dir() and sweep_root.resolve() in child.resolve().parents:
                shutil.rmtree(child)
    models_dir = sweep_root / "models"
    if models_dir.is_dir() and sweep_root.resolve() in models_dir.resolve().parents:
        shutil.rmtree(models_dir)


def self_delete() -> None:
    script_path = Path(__file__).resolve()
    pycache = script_path.parent / "__pycache__"
    if pycache.is_dir():
        for child in pycache.glob(f"{script_path.stem}*.pyc"):
            try:
                child.unlink()
            except OSError:
                pass
    try:
        script_path.unlink()
    except OSError as exc:
        print(f"[Sweep] WARNING: could not delete temporary script: {exc}", flush=True)


def print_dry_run(props: Dict[str, Dict[str, float | bool]], omega_n: float, zeta: float) -> None:
    print("K,Kp_Knee,Kd_Knee,Kp_Ankle,Kd_Ankle")
    for stiffness in GRID:
        knee = derive_params(stiffness, float(props[SEA_KNEE]["Jm"]), float(props[SEA_KNEE]["Bm"]), omega_n, zeta)
        ankle = derive_params(stiffness, float(props[SEA_ANKLE]["Jm"]), float(props[SEA_ANKLE]["Bm"]), omega_n, zeta)
        print(
            f"{stiffness:g},{knee.kp:.12g},{knee.kd:.12g},"
            f"{ankle.kp:.12g},{ankle.kd:.12g}"
        )


def main() -> int:
    args = parse_args()
    template = resolve(args.template)
    reference_path = resolve(args.reference)
    props = parse_template_props(template)

    if args.dry_run:
        print_dry_run(props, args.omega_n, args.zeta)
        return 0

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    suffix = "quick_smoke" if args.quick_smoke else stamp
    sweep_root = REPO_ROOT / "results" / f"_sea_parameter_sweep_{suffix}"
    csv_path = sweep_root / "sweep_results.csv"
    report_path = REPO_ROOT / "reports" / "user" / f"{datetime.now().strftime('%Y-%m-%d')}_sea_parameter_sweep.md"
    sweep_root.mkdir(parents=True, exist_ok=True)

    rows: List[Dict[str, object]] = []

    if args.quick_smoke:
        smoke_grid = [250, 500]
        smoke_pairs = []
        for k in smoke_grid:
            smoke_pairs.append((k, 500))
            smoke_pairs.append((250, k))
        smoke_pairs = list(dict.fromkeys(smoke_pairs))
        smoke_candidates = [
            make_candidate("smoke", knee_k, ankle_k, props, 4.26, 4.30, args.omega_n, args.zeta)
            for knee_k, ankle_k in smoke_pairs
        ]
        run_batch(smoke_candidates, template, sweep_root, reference_path, args.workers, csv_path, rows)
        generate_report(report_path, sweep_root, rows, None, applied=False)
        print(f"[Sweep] Quick smoke complete. CSV: {csv_path}")
        return 0 if all(row.get("complete") for row in rows) else 1

    screen_t_start = args.screen_t_start
    screen_t_end = args.screen_t_end
    full_t_start = args.full_t_start
    full_t_end = args.full_t_end

    knee_candidates = [
        make_candidate("screen_knee", k, 500, props, screen_t_start, screen_t_end, args.omega_n, args.zeta)
        for k in GRID
    ]
    ankle_candidates = [
        make_candidate("screen_ankle", 250, k, props, screen_t_start, screen_t_end, args.omega_n, args.zeta)
        for k in GRID
    ]

    stage_rows = run_batch(knee_candidates + ankle_candidates, template, sweep_root, reference_path, args.workers, csv_path, rows)

    knee_values = top_unique_values(
        [row for row in stage_rows if str(row.get("stage")) == "screen_knee"],
        "knee_K",
        4,
    )
    ankle_values = top_unique_values(
        [row for row in stage_rows if str(row.get("stage")) == "screen_ankle"],
        "ankle_K",
        4,
    )
    if not knee_values:
        knee_values = [250]
    if not ankle_values:
        ankle_values = [500]
    print(f"[Sweep] Selected knee K values: {knee_values}", flush=True)
    print(f"[Sweep] Selected ankle K values: {ankle_values}", flush=True)

    by_pair: Dict[Tuple[float, float], Dict[str, object]] = {
        (float(row["knee_K"]), float(row["ankle_K"])): row
        for row in rows
        if "knee_K" in row and "ankle_K" in row
    }
    combo_candidates: List[Candidate] = []
    for knee_k in knee_values:
        for ankle_k in ankle_values:
            pair = (float(knee_k), float(ankle_k))
            if pair in by_pair:
                continue
            combo_candidates.append(
                make_candidate("screen_combo", knee_k, ankle_k, props, screen_t_start, screen_t_end, args.omega_n, args.zeta)
            )
    combo_rows = run_batch(combo_candidates, template, sweep_root, reference_path, args.workers, csv_path, rows)
    for row in combo_rows:
        by_pair[(float(row["knee_K"]), float(row["ankle_K"]))] = row

    ranked_screen_rows = sorted(by_pair.values(), key=score_key)
    full_candidates = rows_to_candidates(
        ranked_screen_rows,
        props,
        "full",
        full_t_start,
        full_t_end,
        args.omega_n,
        args.zeta,
    )

    full_rows: List[Dict[str, object]] = []
    best: Optional[Dict[str, object]] = None
    idx = 0
    while idx < len(full_candidates):
        batch_size = 3 if idx == 0 else 1
        batch = full_candidates[idx: idx + batch_size]
        new_rows = run_batch(batch, template, sweep_root, reference_path, min(args.workers, len(batch)), csv_path, rows)
        full_rows.extend(new_rows)
        acceptable = [row for row in full_rows if row.get("acceptable")]
        if acceptable:
            best = sorted(acceptable, key=score_key)[0]
            break
        idx += batch_size

    best_json_path = sweep_root / "best_candidate.json"
    if best is None:
        best_json_path.write_text(json.dumps({"found": False}, indent=2), encoding="utf-8")
        generate_report(report_path, sweep_root, rows, None, applied=False)
        print("[Sweep] No acceptable solution found.", flush=True)
        print(f"[Sweep] Report: {report_path}", flush=True)
        return 1

    best_payload = {
        "found": True,
        "best": best,
        "omega_n": args.omega_n,
        "zeta": args.zeta,
    }
    best_json_path.write_text(json.dumps(best_payload, indent=2), encoding="utf-8")

    final_candidate = make_candidate(
        "final",
        float(best["knee_K"]),
        float(best["ankle_K"]),
        props,
        full_t_start,
        full_t_end,
        args.omega_n,
        args.zeta,
    )
    write_candidate_model(template, template, final_candidate)
    update_config_stiffness(REPO_ROOT / "config.py", final_candidate.knee.stiffness, final_candidate.ankle.stiffness)

    winning_results_dir = REPO_ROOT / str(best["results_dir"])
    cleanup_after_success(sweep_root, winning_results_dir)
    generate_report(report_path, sweep_root, rows, best, applied=True)

    print("[Sweep] Acceptable solution applied.", flush=True)
    print(f"[Sweep] Best: {best['run_id']}", flush=True)
    print(f"[Sweep] Report: {report_path}", flush=True)
    self_delete()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
