#!/usr/bin/env python3
"""
Run the J_eff cascade redesign TODO validations.

This script keeps the active configuration untouched.  It creates dedicated
model variants, runs selected cascade designs, and reuses the cascade sweep
metric collector so the output is directly comparable with the 2026-05-18
morning best.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shutil
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, Iterable, List
from xml.etree import ElementTree as ET

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from validation.cascade_local_gain_sweep import (  # noqa: E402
    Candidate,
    collect_metrics,
    format_command,
    subprocess_env,
)


SETUP = REPO_ROOT / "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml"
REFERENCE = REPO_ROOT / "models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot"
BASE_MODEL = REPO_ROOT / "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim"
ZETA07_MODEL = REPO_ROOT / "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_zeta07.osim"
V3CAP_MODEL = REPO_ROOT / "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_jeff_v3cap.osim"
V3CAP_P01_MODEL = REPO_ROOT / "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_jeff_v3cap_p01.osim"
OPT_B_MODEL = REPO_ROOT / "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_optB_ankle_wn500.osim"
OPT_C_MODEL = REPO_ROOT / "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_optC_ankle_kd15.osim"

MORNING_BEST_DIR = (
    REPO_ROOT
    / "results/_cascade_full_ankle5_20260518_105345/full_runs/"
    "full_ankle5_kpo18p85_kpi29p2_kii1377_kil50_apo47p125_api2p8275_aii213_ail200"
)

FULL_WINDOW = (11.99, 21.0)
PUSHOFF_WINDOWS = (
    (13.1638, 14.7799),
    (17.9528, 19.5259),
)


@dataclass(frozen=True)
class Variant:
    run_id: str
    label: str
    model: Path
    candidate: Candidate


def fmt_float(value: float) -> str:
    return f"{float(value):.12g}"


def update_sea_properties(source: Path, destination: Path, updates: Dict[str, Dict[str, float]]) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(source)
    root = tree.getroot()

    found: set[str] = set()
    for element in root.iter():
        if element.tag.rsplit("}", 1)[-1] != "SeriesElasticActuator":
            continue
        sea_name = element.attrib.get("name", "")
        if sea_name not in updates:
            continue
        found.add(sea_name)
        child_by_name = {child.tag.rsplit("}", 1)[-1]: child for child in list(element)}
        for tag, value in updates[sea_name].items():
            child = child_by_name.get(tag)
            if child is None:
                child = ET.SubElement(element, tag)
            child.text = fmt_float(value)

    missing = sorted(set(updates) - found)
    if missing:
        raise RuntimeError(f"Missing SeriesElasticActuator blocks in {source}: {missing}")

    ET.indent(tree, space="\t")
    tree.write(destination, encoding="utf-8", xml_declaration=True)


def prepare_models() -> None:
    if not ZETA07_MODEL.is_file():
        update_sea_properties(
            BASE_MODEL,
            ZETA07_MODEL,
            {
                "SEA_Knee": {"Kp": 4.17, "Kd": 5.66, "Ki": 290.7},
                "SEA_Ankle": {"Kp": 1.01, "Kd": 4.38, "Ki": 87.81},
            },
        )

    update_sea_properties(
        BASE_MODEL,
        V3CAP_MODEL,
        {
            "SEA_Knee": {"Kp": 4.17, "Kd": 5.66, "Ki": 290.7},
            "SEA_Ankle": {"Kp": 1.556, "Kd": 4.956, "Ki": 126.2},
        },
    )

    update_sea_properties(
        BASE_MODEL,
        V3CAP_P01_MODEL,
        {
            "SEA_Knee": {"Kp": 3.603, "Kd": 5.30, "Ki": 145.35},
            "SEA_Ankle": {"Kp": 1.277, "Kd": 4.64, "Ki": 63.1},
        },
    )

    # Option B: knee invariato (morning best 18/11/190),
    # ankle motor driver ricomputato con omega_n=500 rad/s, p=0.2*omega_n=100, zeta=0.7:
    #   Ki = Jm*p*omega_n^2/Ks = 0.01*100*250000/500 = 500
    #   Kp = Jm*(omega_n^2 + 2*zeta*omega_n*p)/Ks - 1 = 0.01*320000/500 - 1 = 5.4
    #   Kd = Jm*(p + 2*zeta*omega_n) - Bm = 0.01*800 - 0.1 = 7.9
    update_sea_properties(
        BASE_MODEL,
        OPT_B_MODEL,
        {
            "SEA_Knee": {"Kp": 18.0, "Kd": 11.0, "Ki": 190.0},
            "SEA_Ankle": {"Kp": 5.4, "Kd": 7.9, "Ki": 500.0},
        },
    )

    # Option C: knee invariato, ankle banda omega_n=777 mantenuta, solo Kd alzato
    # per portare zeta da 0.71 a ~0.97. Kp=11.3 e Ki=123 invariati.
    # Caratteristica nuova: s^3 + 1510 s^2 + 615000 s + 6,150,000 con polo reale ~-10
    # e coppia complessa con omega_n ~774, zeta ~0.97 (oversmorzato).
    update_sea_properties(
        BASE_MODEL,
        OPT_C_MODEL,
        {
            "SEA_Knee": {"Kp": 18.0, "Kd": 11.0, "Ki": 190.0},
            "SEA_Ankle": {"Kp": 11.3, "Kd": 15.0, "Ki": 123.0},
        },
    )


def variants() -> Dict[str, Variant]:
    return {
        "morning_best": Variant(
            run_id="morning_best",
            label="Morning best 2026-05-18",
            model=BASE_MODEL,
            candidate=Candidate(
                run_id="morning_best",
                knee_kp_outer=18.85,
                knee_kp_inner=29.2,
                knee_ki_inner=1377.0,
                knee_i_limit=50.0,
                ankle_kp_outer=47.125,
                ankle_kp_inner=2.8275,
                ankle_ki_inner=213.0,
                ankle_i_limit=200.0,
            ),
        ),
        "v3a": Variant(
            run_id="v3a_jeff_free",
            label="V3 J_eff knee free",
            model=ZETA07_MODEL,
            candidate=Candidate(
                run_id="v3a_jeff_free",
                knee_kp_outer=40.0,
                knee_kp_inner=26.88,
                knee_ki_inner=2304.0,
                knee_i_limit=50.0,
                ankle_kp_outer=31.1,
                ankle_kp_inner=1.306,
                ankle_ki_inner=87.05,
                ankle_i_limit=80.0,
            ),
        ),
        "v3m": Variant(
            run_id="v3m_jeff_mid",
            label="V3 J_eff knee mid",
            model=ZETA07_MODEL,
            candidate=Candidate(
                run_id="v3m_jeff_mid",
                knee_kp_outer=40.0,
                knee_kp_inner=45.36,
                knee_ki_inner=3888.0,
                knee_i_limit=50.0,
                ankle_kp_outer=31.1,
                ankle_kp_inner=1.306,
                ankle_ki_inner=87.05,
                ankle_i_limit=80.0,
            ),
        ),
        "v3cap": Variant(
            run_id="v3cap_jeff_mid",
            label="V3' J_eff knee mid, ankle motor cap",
            model=V3CAP_MODEL,
            candidate=Candidate(
                run_id="v3cap_jeff_mid",
                knee_kp_outer=40.0,
                knee_kp_inner=45.36,
                knee_ki_inner=3888.0,
                knee_i_limit=50.0,
                ankle_kp_outer=35.1,
                ankle_kp_inner=1.474,
                ankle_ki_inner=110.88,
                ankle_i_limit=80.0,
            ),
        ),
        "v3cap_p01": Variant(
            run_id="v3cap_p01_jeff_mid",
            label="V3' p/on=0.1 zero-pole check",
            model=V3CAP_P01_MODEL,
            candidate=Candidate(
                run_id="v3cap_p01_jeff_mid",
                knee_kp_outer=40.0,
                knee_kp_inner=45.36,
                knee_ki_inner=3888.0,
                knee_i_limit=50.0,
                ankle_kp_outer=35.1,
                ankle_kp_inner=1.474,
                ankle_ki_inner=110.88,
                ankle_i_limit=80.0,
            ),
        ),
        "opt_b": Variant(
            run_id="opt_b_ankle_wn500",
            label="Option B: knee morning best, ankle motor wn=500",
            model=OPT_B_MODEL,
            candidate=Candidate(
                run_id="opt_b_ankle_wn500",
                knee_kp_outer=18.85,
                knee_kp_inner=29.2,
                knee_ki_inner=1377.0,
                knee_i_limit=50.0,
                ankle_kp_outer=47.125,
                ankle_kp_inner=2.8275,
                ankle_ki_inner=213.0,
                ankle_i_limit=200.0,
            ),
        ),
        "morning_best_run": Variant(
            run_id="morning_best_run",
            label="Morning best rerun on requested window",
            model=BASE_MODEL,
            candidate=Candidate(
                run_id="morning_best_run",
                knee_kp_outer=18.85,
                knee_kp_inner=29.2,
                knee_ki_inner=1377.0,
                knee_i_limit=50.0,
                ankle_kp_outer=47.125,
                ankle_kp_inner=2.8275,
                ankle_ki_inner=213.0,
                ankle_i_limit=200.0,
            ),
        ),
        "opt_c": Variant(
            run_id="opt_c_ankle_kd15",
            label="Option C: ankle Kd=15 (zeta~0.97, banda invariata)",
            model=OPT_C_MODEL,
            candidate=Candidate(
                run_id="opt_c_ankle_kd15",
                knee_kp_outer=18.85,
                knee_kp_inner=29.2,
                knee_ki_inner=1377.0,
                knee_i_limit=50.0,
                ankle_kp_outer=47.125,
                ankle_kp_inner=2.8275,
                ankle_ki_inner=213.0,
                ankle_i_limit=200.0,
            ),
        ),
    }


def command_for_variant(
    python_exe: str,
    variant: Variant,
    results_dir: Path,
    t_start: float,
    t_end: float,
) -> List[str]:
    c = variant.candidate
    return [
        python_exe,
        str(REPO_ROOT / "main.py"),
        "--setup", str(SETUP),
        "--model", str(variant.model),
        "--t-start", fmt_float(t_start),
        "--t-end", fmt_float(t_end),
        "--output-dir", str(results_dir),
        "--filter-grf",
        "--sea-outer-controller", "cascade",
        "--sea-cascade-kp-outer-knee", fmt_float(c.knee_kp_outer),
        "--sea-cascade-kp-inner-knee", fmt_float(c.knee_kp_inner),
        "--sea-cascade-ki-inner-knee", fmt_float(c.knee_ki_inner),
        "--sea-cascade-inner-i-torque-limit-knee", fmt_float(c.knee_i_limit),
        "--sea-cascade-kp-outer-ankle", fmt_float(c.ankle_kp_outer),
        "--sea-cascade-kp-inner-ankle", fmt_float(c.ankle_kp_inner),
        "--sea-cascade-ki-inner-ankle", fmt_float(c.ankle_ki_inner),
        "--sea-cascade-inner-i-torque-limit-ankle", fmt_float(c.ankle_i_limit),
    ]


def run_command(cmd: List[str], results_dir: Path, timeout_s: float) -> int:
    results_dir.mkdir(parents=True, exist_ok=True)
    console_path = results_dir / "console.txt"
    with console_path.open("w", encoding="utf-8", errors="replace") as fh:
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
            fh.write(f"\n[J_eff] TIMEOUT after {timeout_s:.1f}s\n")
            return 124


def run_variant(
    python_exe: str,
    out_root: Path,
    variant: Variant,
    t_start: float,
    t_end: float,
    timeout_s: float,
) -> Dict[str, object]:
    results_dir = out_root / "runs" / variant.run_id
    if results_dir.exists():
        shutil.rmtree(results_dir)
    cmd = command_for_variant(python_exe, variant, results_dir, t_start, t_end)
    started = time.monotonic()
    return_code = run_command(cmd, results_dir, timeout_s)
    elapsed = time.monotonic() - started
    row = collect_metrics(
        variant.candidate,
        "full",
        results_dir,
        REFERENCE,
        return_code,
        elapsed,
        timeout_s,
    )
    row["label"] = variant.label
    row["model"] = str(variant.model.relative_to(REPO_ROOT))
    return row


def collect_existing_morning_best(timeout_s: float = 0.0) -> Dict[str, object]:
    variant = variants()["morning_best"]
    row = collect_metrics(
        variant.candidate,
        "existing",
        MORNING_BEST_DIR,
        REFERENCE,
        0,
        0.0,
        timeout_s,
    )
    row["label"] = variant.label
    row["model"] = str(variant.model.relative_to(REPO_ROOT))
    return row


def write_rows(path: Path, rows: Iterable[Dict[str, object]]) -> None:
    rows = list(rows)
    fields = [
        "run_id",
        "label",
        "model",
        "run_status",
        "complete",
        "acceptable",
        "fail_reason",
        "knee_tracking_rms_deg",
        "ankle_tracking_rms_deg",
        "mean_rmse_deg",
        "score_kinematic_deg",
        "sat_count",
        "knee_tau_input_saturation_count",
        "ankle_tau_input_saturation_count",
        "max_u",
        "max_tau_input_plugin_abs",
        "knee_tau_input_plugin_hpf50_rms",
        "ankle_tau_input_plugin_hpf50_rms",
        "knee_motor_speed_dot_hpf50_rms",
        "ankle_motor_speed_dot_hpf50_rms",
        "knee_motor_speed_dot_rms",
        "ankle_motor_speed_dot_rms",
        "knee_tau_error_rms",
        "ankle_tau_error_rms",
        "results_dir",
        "elapsed_s",
    ]
    extra = sorted({key for row in rows for key in row.keys()} - set(fields))
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields + extra, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fields + extra})


def write_summary(path: Path, rows: List[Dict[str, object]]) -> None:
    lines = [
        "# Cascade J_eff TODO validation",
        "",
        "| run | status | acceptable | knee RMSE deg | ankle RMSE deg | sat | max |u| | ankle tau HPF50 | ankle mdot HPF50 | fail |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for row in rows:
        lines.append(
            "| {run} | {status} | {ok} | {kr:.4g} | {ar:.4g} | {sat} | {u:.4g} | {ath:.4g} | {amh:.4g} | {fail} |".format(
                run=row.get("run_id", ""),
                status=row.get("run_status", ""),
                ok=row.get("acceptable", ""),
                kr=float(row.get("knee_tracking_rms_deg", math.nan)),
                ar=float(row.get("ankle_tracking_rms_deg", math.nan)),
                sat=row.get("sat_count", ""),
                u=float(row.get("max_u", math.nan)),
                ath=float(row.get("ankle_tau_input_plugin_hpf50_rms", math.nan)),
                amh=float(row.get("ankle_motor_speed_dot_hpf50_rms", math.nan)),
                fail=str(row.get("fail_reason", "")),
            )
        )
    lines.append("")
    lines.append("Full CSV: `summary.csv`.")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--prepare-only",
        action="store_true",
        help="Only create/update model variants.",
    )
    parser.add_argument(
        "--python",
        default="/opt/anaconda3/envs/envCMC-like/bin/python",
        help="Python executable with OpenSim installed.",
    )
    parser.add_argument(
        "--out-root",
        default="results/_cascade_jeff_todo_20260519",
        help="Output root for runs and summaries.",
    )
    parser.add_argument(
        "--variants",
        default="v3a,v3m,v3cap",
        help="Comma-separated variants to run: v3a,v3m,v3cap,v3cap_p01.",
    )
    parser.add_argument(
        "--include-p01",
        action="store_true",
        help="Also run the p/omega=0.1 zero-pole variant.",
    )
    parser.add_argument("--t-start", type=float, default=FULL_WINDOW[0])
    parser.add_argument("--t-end", type=float, default=FULL_WINDOW[1])
    parser.add_argument("--timeout", type=float, default=2400.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    prepare_models()
    print(f"[J_eff] prepared model variants:\n  {V3CAP_MODEL}\n  {V3CAP_P01_MODEL}")
    if args.prepare_only:
        return 0

    out_root = Path(args.out_root)
    if not out_root.is_absolute():
        out_root = REPO_ROOT / out_root
    out_root.mkdir(parents=True, exist_ok=True)

    all_variants = variants()
    names = [name.strip() for name in args.variants.split(",") if name.strip()]
    if args.include_p01 and "v3cap_p01" not in names:
        names.append("v3cap_p01")
    unknown = sorted(set(names) - set(all_variants))
    if unknown:
        raise ValueError(f"Unknown variant names: {unknown}")

    rows = [collect_existing_morning_best(args.timeout)]
    for name in names:
        variant = all_variants[name]
        print(f"[J_eff] running {variant.run_id}: {variant.label}")
        row = run_variant(
            args.python,
            out_root,
            variant,
            float(args.t_start),
            float(args.t_end),
            float(args.timeout),
        )
        rows.append(row)
        print(
            "[J_eff] {run}: status={status} acceptable={ok} "
            "knee={knee:.4g}deg ankle={ankle:.4g}deg sat={sat} fail={fail}".format(
                run=row.get("run_id", variant.run_id),
                status=row.get("run_status", ""),
                ok=row.get("acceptable", ""),
                knee=float(row.get("knee_tracking_rms_deg", math.nan)),
                ankle=float(row.get("ankle_tracking_rms_deg", math.nan)),
                sat=row.get("sat_count", ""),
                fail=row.get("fail_reason", ""),
            )
        )

    write_rows(out_root / "summary.csv", rows)
    write_summary(out_root / "summary.md", rows)
    with (out_root / "variants.json").open("w", encoding="utf-8") as fh:
        json.dump(
            {
                name: {
                    "run_id": variant.run_id,
                    "label": variant.label,
                    "model": str(variant.model.relative_to(REPO_ROOT)),
                    "candidate": asdict(variant.candidate),
                }
                for name, variant in all_variants.items()
            },
            fh,
            indent=2,
            sort_keys=True,
        )
        fh.write("\n")

    print(f"[J_eff] wrote {out_root / 'summary.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
