"""
sea_inner_10020_sweep.py
========================
Deterministic SEA inner-loop sweep for the fixed high-level 100/20 gains.

This script does not touch the C++ plugin and does not add new plugin
properties to generated models.  It only rewrites stiffness/Kp/Kd inside copied
.osim files, runs short screening windows, and optionally runs full acceptance
tests on the best candidates.
"""
from __future__ import annotations

import argparse
import csv
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
from typing import Dict, List, Sequence, Tuple
from xml.etree import ElementTree as ET

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
SEA_COORDS = (
    ("knee", SEA_KNEE, COORD_KNEE),
    ("ankle", SEA_ANKLE, COORD_ANKLE),
)

K_KNEE_GRID = [250, 500, 750, 1000]
K_ANKLE_GRID = [500, 750, 1000, 1250]
OMEGA_N_GRID = [700, 900, 1100, 1300, 1500]
ZETA_GRID = [0.7, 0.85, 1.0, 1.2]
SCREEN_WINDOWS = ((4.26, 5.30), (6.80, 7.20))


@dataclass(frozen=True)
class SeaParams:
    stiffness: float
    kp: float
    kd: float


@dataclass(frozen=True)
class Candidate:
    run_id: str
    knee: SeaParams
    ankle: SeaParams
    omega_n: float
    zeta: float


def fmt_num(value: float) -> str:
    return f"{value:g}".replace(".", "p").replace("-", "m")


def fmt_xml(value: float) -> str:
    return f"{value:.12g}"


def resolve(path: str | Path) -> Path:
    p = Path(path)
    return p if p.is_absolute() else REPO_ROOT / p


def xml_local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def parse_sea_props(model_path: Path) -> Dict[str, Dict[str, float]]:
    root = ET.parse(model_path).getroot()
    props: Dict[str, Dict[str, float]] = {}
    for element in root.iter():
        if xml_local_name(element.tag) != "SeriesElasticActuator":
            continue
        name = element.attrib.get("name", "")
        if name not in {SEA_KNEE, SEA_ANKLE}:
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
        }
    missing = {SEA_KNEE, SEA_ANKLE} - set(props)
    if missing:
        raise ValueError(f"Missing SEA actuator(s): {sorted(missing)}")
    return props


def derive_params(stiffness: float, jm: float, bm: float,
                  omega_n: float, zeta: float) -> SeaParams | None:
    kp = jm * omega_n * omega_n / stiffness - 1.0
    kd = 2.0 * zeta * jm * omega_n - bm
    if stiffness <= 0.0 or kp <= 0.0 or kd < 0.0:
        return None
    return SeaParams(float(stiffness), float(kp), float(kd))


def generate_candidates(props: Dict[str, Dict[str, float]]) -> List[Candidate]:
    candidates: List[Candidate] = []
    for omega_n in OMEGA_N_GRID:
        for zeta in ZETA_GRID:
            for knee_k in K_KNEE_GRID:
                knee = derive_params(
                    knee_k, props[SEA_KNEE]["Jm"], props[SEA_KNEE]["Bm"],
                    omega_n, zeta,
                )
                if knee is None:
                    continue
                for ankle_k in K_ANKLE_GRID:
                    ankle = derive_params(
                        ankle_k, props[SEA_ANKLE]["Jm"], props[SEA_ANKLE]["Bm"],
                        omega_n, zeta,
                    )
                    if ankle is None:
                        continue
                    run_id = (
                        f"kk{fmt_num(knee.stiffness)}_ka{fmt_num(ankle.stiffness)}"
                        f"_wn{fmt_num(omega_n)}_z{fmt_num(zeta)}"
                    )
                    candidates.append(Candidate(run_id, knee, ankle, omega_n, zeta))
    return candidates


def replace_tag(block: str, tag: str, value: float) -> str:
    pattern = rf"(<{tag}>)(.*?)(</{tag}>)"
    replacement = rf"\g<1>{fmt_xml(value)}\g<3>"
    new_block, count = re.subn(pattern, replacement, block, count=1, flags=re.S)
    if count != 1:
        raise ValueError(f"Missing <{tag}> in SEA block")
    return new_block


def replace_sea_block(text: str, sea_name: str, params: SeaParams) -> str:
    pattern = (
        rf'(<SeriesElasticActuator name="{re.escape(sea_name)}">.*?'
        r'</SeriesElasticActuator>)'
    )

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
            _time, _cols, data, _deg = read_sto(str(path))
        except Exception:
            return False
        if not np.all(np.isfinite(data)):
            return False
    return True


def sto_col(path: Path, column: str) -> np.ndarray:
    _time, cols, data, _deg = read_sto(str(path))
    return np.asarray(data[:, cols.index(column)], dtype=float)


def tracking_stats(results_dir: Path, reference: Path, coord: str) -> Tuple[float, float]:
    sim_t, sim_cols, sim_data, sim_deg = read_sto(
        str(results_dir / "sim_output_kinematics.sto")
    )
    ref_t, ref_cols, ref_data, ref_deg = read_sto(str(reference))
    sim = np.asarray(sim_data[:, sim_cols.index(coord)], dtype=float)
    ref = np.asarray(ref_data[:, ref_cols.index(coord)], dtype=float)
    if sim_deg:
        sim = np.deg2rad(sim)
    if ref_deg:
        ref = np.deg2rad(ref)
    err = np.rad2deg(sim - np.interp(sim_t, ref_t, ref))
    return (
        float(np.sqrt(np.nanmean(err * err))),
        float(np.nanmax(np.abs(err))),
    )


def collect_metrics(results_dir: Path, model_path: Path,
                    reference: Path) -> Dict[str, float | int | bool | str]:
    row: Dict[str, float | int | bool | str] = {}
    status = load_run_status(results_dir)
    row["run_status"] = status.get("status", "missing")
    row["complete"] = status.get("status") == "complete"
    row["finite_outputs"] = finite_sto_outputs(results_dir)
    if not row["complete"] or not row["finite_outputs"]:
        row["acceptable_screen"] = False
        row["score"] = math.inf
        return row

    diag = results_dir / "sim_output_sea_diagnostics.sto"
    controls = results_dir / "sim_output_sea_controls.sto"
    f_opt = load_f_opt_from_model(model_path)
    hpf = analyze_sto(
        results_dir / "sim_output_sea_torques.sto",
        controls,
        cutoff_hz=50.0,
        skip_s=0.1,
        f_opt=f_opt,
    )

    sat_count = 0
    max_raw = 0.0
    max_u = 0.0
    worst_hpf = 0.0
    worst_rms = 0.0
    worst_max = 0.0
    for label, sea_name, coord in SEA_COORDS:
        sat = sto_col(diag, f"{sea_name}_tau_input_saturated")
        sat_count += int(np.sum(sat > 0.5))
        raw = sto_col(diag, f"{sea_name}_tau_input_raw")
        max_raw = max(max_raw, float(np.nanmax(np.abs(raw))))
        u = sto_col(controls, coord)
        max_u = max(max_u, float(np.nanmax(np.abs(u))))
        worst_hpf = max(
            worst_hpf,
            float(hpf.get(f"{label}_hpf_noise_tau_input", math.inf)),
        )
        rms_deg, max_deg = tracking_stats(results_dir, reference, coord)
        worst_rms = max(worst_rms, rms_deg)
        worst_max = max(worst_max, max_deg)

    row.update({
        "sat_count": sat_count,
        "max_tau_input_raw_abs": max_raw,
        "max_u": max_u,
        "worst_hpf_tau_input": worst_hpf,
        "worst_tracking_rms_deg": worst_rms,
        "worst_tracking_max_deg": worst_max,
        "acceptable_screen": sat_count == 0,
        "score": (
            worst_rms
            + 0.05 * worst_max
            + 10.0 * worst_hpf
            + 0.001 * max_raw
            + max_u
        ),
    })
    return row


def run_simulation(python_exe: str, model_path: Path, results_dir: Path,
                   t_start: float, t_end: float, timeout_s: float) -> int:
    results_dir.mkdir(parents=True, exist_ok=True)
    cmd = [
        python_exe,
        str(REPO_ROOT / "main.py"),
        "--model", str(model_path),
        "--t-start", fmt_xml(t_start),
        "--t-end", fmt_xml(t_end),
        "--output-dir", str(results_dir),
    ]
    console = results_dir / "console.txt"
    with console.open("w", encoding="utf-8", errors="replace") as fh:
        fh.write("Command:\n" + " ".join(cmd) + "\n\n")
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
            fh.write(f"\nTIMEOUT after {timeout_s:.1f} s\n")
            return 124


def screen_candidate(candidate: Candidate, template: Path, sweep_root: Path,
                     reference: Path, python_exe: str,
                     timeout_s: float) -> Dict[str, object]:
    model_path = sweep_root / "models" / f"{candidate.run_id}.osim"
    write_candidate_model(template, model_path, candidate)
    combined: Dict[str, object] = candidate_row(candidate)
    combined["stage"] = "screen"
    combined["model_path"] = str(model_path.relative_to(REPO_ROOT))

    worst: Dict[str, float | int | bool | str] = {
        "sat_count": 0,
        "max_tau_input_raw_abs": 0.0,
        "max_u": 0.0,
        "worst_hpf_tau_input": 0.0,
        "worst_tracking_rms_deg": 0.0,
        "worst_tracking_max_deg": 0.0,
        "score": 0.0,
    }
    for idx, (t_start, t_end) in enumerate(SCREEN_WINDOWS, start=1):
        run_dir = sweep_root / "runs" / f"{candidate.run_id}_w{idx}"
        rc = run_simulation(
            python_exe, model_path, run_dir, t_start, t_end, timeout_s
        )
        metrics = collect_metrics(run_dir, model_path, reference)
        if rc != 0 or not metrics.get("complete", False):
            combined.update(metrics)
            combined["acceptable_screen"] = False
            combined["score"] = math.inf
            combined["fail_reason"] = f"window {idx} incomplete rc={rc}"
            combined["results_dir"] = str(run_dir.relative_to(REPO_ROOT))
            return combined
        worst["sat_count"] = int(worst["sat_count"]) + int(metrics["sat_count"])
        for key in [
            "max_tau_input_raw_abs",
            "max_u",
            "worst_hpf_tau_input",
            "worst_tracking_rms_deg",
            "worst_tracking_max_deg",
        ]:
            worst[key] = max(float(worst[key]), float(metrics[key]))

    worst["acceptable_screen"] = int(worst["sat_count"]) == 0
    worst["score"] = (
        float(worst["worst_tracking_rms_deg"])
        + 0.05 * float(worst["worst_tracking_max_deg"])
        + 10.0 * float(worst["worst_hpf_tau_input"])
        + 0.001 * float(worst["max_tau_input_raw_abs"])
        + float(worst["max_u"])
    )
    combined.update(worst)
    combined["results_dir"] = str((sweep_root / "runs").relative_to(REPO_ROOT))
    combined["fail_reason"] = "" if worst["acceptable_screen"] else "tau_input saturated"
    return combined


def candidate_row(candidate: Candidate) -> Dict[str, object]:
    return {
        "run_id": candidate.run_id,
        "knee_K": candidate.knee.stiffness,
        "knee_Kp": candidate.knee.kp,
        "knee_Kd": candidate.knee.kd,
        "ankle_K": candidate.ankle.stiffness,
        "ankle_Kp": candidate.ankle.kp,
        "ankle_Kd": candidate.ankle.kd,
        "omega_n": candidate.omega_n,
        "zeta": candidate.zeta,
    }


CSV_FIELDS = [
    "run_id", "stage",
    "knee_K", "knee_Kp", "knee_Kd",
    "ankle_K", "ankle_Kp", "ankle_Kd",
    "omega_n", "zeta",
    "run_status", "complete", "finite_outputs", "acceptable_screen",
    "sat_count", "max_tau_input_raw_abs", "max_u",
    "worst_hpf_tau_input", "worst_tracking_rms_deg",
    "worst_tracking_max_deg", "score", "fail_reason",
    "model_path", "results_dir",
]


def write_csv(path: Path, rows: Sequence[Dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in CSV_FIELDS})


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Sweep SEA inner params with fixed high-level 100/20 gains."
    )
    parser.add_argument("--template", default="models/Adjusted_SEASEA - Copia_tuned.osim")
    parser.add_argument("--reference", default="data/3DGaitModel2392_Kinematics_q.sto")
    parser.add_argument("--python", default=sys.executable)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--top-n-full", type=int, default=8)
    parser.add_argument("--screen-only", action="store_true")
    parser.add_argument("--timeout-s", type=float, default=180.0)
    parser.add_argument("--sweep-root", default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    template = resolve(args.template)
    reference = resolve(args.reference)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    sweep_root = (
        resolve(args.sweep_root)
        if args.sweep_root else REPO_ROOT / "results" / f"_inner10020_sweep_{stamp}"
    )
    props = parse_sea_props(template)
    candidates = generate_candidates(props)
    print(f"[Sweep] candidates={len(candidates)} root={sweep_root}", flush=True)

    screen_rows: List[Dict[str, object]] = []
    with ThreadPoolExecutor(max_workers=max(1, args.workers)) as pool:
        future_map = {
            pool.submit(
                screen_candidate,
                candidate,
                template,
                sweep_root,
                reference,
                args.python,
                args.timeout_s,
            ): candidate
            for candidate in candidates
        }
        for future in as_completed(future_map):
            row = future.result()
            screen_rows.append(row)
            write_csv(sweep_root / "screen_results.csv", screen_rows)
            status = "OK" if row.get("acceptable_screen") else "NO"
            print(
                f"[Screen {len(screen_rows):03d}/{len(candidates):03d}] "
                f"{status} {row.get('run_id')} score={row.get('score')}",
                flush=True,
            )

    ordered = sorted(
        [row for row in screen_rows if row.get("acceptable_screen")],
        key=lambda row: float(row.get("score", math.inf)),
    )
    write_csv(sweep_root / "screen_results.csv", screen_rows)
    write_csv(sweep_root / "screen_top.csv", ordered)
    print(f"[Sweep] acceptable screen candidates={len(ordered)}", flush=True)
    for row in ordered[:10]:
        print(
            f"  {row['run_id']} score={float(row['score']):.6g} "
            f"rms={float(row['worst_tracking_rms_deg']):.3f} "
            f"hpf={float(row['worst_hpf_tau_input']):.4f}",
            flush=True,
        )

    if args.screen_only or args.top_n_full <= 0 or not ordered:
        return 0 if ordered else 1

    full_rows: List[Dict[str, object]] = []
    for row in ordered[:args.top_n_full]:
        run_id = str(row["run_id"])
        model_path = resolve(str(row["model_path"]))
        full_dir = sweep_root / "full_runs" / run_id
        print(f"[Full] {run_id}", flush=True)
        start = time.monotonic()
        rc = run_simulation(
            args.python, model_path, full_dir, 4.26, 11.06,
            max(args.timeout_s, 1200.0),
        )
        metrics = collect_metrics(full_dir, model_path, reference)
        full_row = dict(row)
        full_row["stage"] = "full"
        full_row.update(metrics)
        full_row["elapsed_s"] = time.monotonic() - start
        full_row["results_dir"] = str(full_dir.relative_to(REPO_ROOT))
        full_row["return_code"] = rc
        full_rows.append(full_row)
        write_csv(sweep_root / "full_results.csv", full_rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
