"""
cmc_strict_metrics.py
=====================
Strict acceptance gate for CMC-like SEA tracking runs.

The normal validator checks broad simulator integrity.  This script adds the
stricter target for the high-gain 100/20 workflow: excellent prosthetic
kinematic tracking, low high-frequency motor torque energy, no motor-torque
saturation, and bounded control effort.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from config import SimulatorConfig  # noqa: E402
from output import read_sto  # noqa: E402
from validation.hpf_noise_metric import analyze_sto, load_f_opt_from_model  # noqa: E402
from validation.validate_sim_results import (  # noqa: E402
    Check,
    load_required_tables,
    load_run_status,
    load_table,
    parse_sea_props,
    validate_power_and_controls,
    validate_reserves,
    validate_run_status,
    validate_sea_derivatives,
    validate_sea_interface_diagnostics,
    validate_sea_tautology,
    validate_tracking,
)


SEA_COORDS = [
    ("knee", "SEA_Knee", "pros_knee_angle"),
    ("ankle", "SEA_Ankle", "pros_ankle_angle"),
]


def resolve_path(raw: str | Path) -> Path:
    path = Path(raw)
    return path if path.is_absolute() else REPO_ROOT / path


def load_sto_matrix(path: Path) -> Tuple[np.ndarray, List[str], np.ndarray, bool]:
    return read_sto(str(path))


def all_sto_outputs_finite(results_dir: Path, prefix: str) -> Tuple[bool, List[str]]:
    bad: List[str] = []
    for path in sorted(results_dir.glob(f"{prefix}_*.sto")):
        try:
            _time, _cols, data, _in_degrees = load_sto_matrix(path)
        except Exception as exc:
            bad.append(f"{path.name}: parse error {exc}")
            continue
        if not np.all(np.isfinite(data)):
            count = int(data.size - np.isfinite(data).sum())
            bad.append(f"{path.name}: {count} non-finite values")
    return not bad, bad


def tracking_stats_deg(
    results_dir: Path,
    prefix: str,
    reference_path: Path,
    coord: str,
) -> Dict[str, float]:
    sim_t, sim_cols, sim_data, sim_in_degrees = load_sto_matrix(
        results_dir / f"{prefix}_kinematics.sto"
    )
    ref_t, ref_cols, ref_data, ref_in_degrees = load_sto_matrix(reference_path)

    if coord not in sim_cols:
        raise KeyError(f"Missing simulated coordinate: {coord}")
    if coord not in ref_cols:
        raise KeyError(f"Missing reference coordinate: {coord}")

    sim = np.asarray(sim_data[:, sim_cols.index(coord)], dtype=float)
    ref = np.asarray(ref_data[:, ref_cols.index(coord)], dtype=float)
    if sim_in_degrees:
        sim = np.deg2rad(sim)
    if ref_in_degrees:
        ref = np.deg2rad(ref)

    ref_i = np.interp(sim_t, ref_t, ref)
    err_deg = np.rad2deg(sim - ref_i)
    return {
        "rms_deg": float(np.sqrt(np.nanmean(err_deg * err_deg))),
        "max_abs_deg": float(np.nanmax(np.abs(err_deg))),
        "mean_abs_deg": float(np.nanmean(np.abs(err_deg))),
    }


def load_column(path: Path, column: str) -> np.ndarray:
    _time, cols, data, _in_degrees = load_sto_matrix(path)
    if column not in cols:
        raise KeyError(f"Missing column {column} in {path}")
    return np.asarray(data[:, cols.index(column)], dtype=float)


def load_optional_column(path: Path, column: str) -> np.ndarray | None:
    _time, cols, data, _in_degrees = load_sto_matrix(path)
    if column not in cols:
        return None
    return np.asarray(data[:, cols.index(column)], dtype=float)


def run_validator_checks(
    cfg: SimulatorConfig,
    results_dir: Path,
    prefix: str,
    model_path: Path,
    reference_path: Path,
) -> List[Check]:
    tables = load_required_tables(results_dir, prefix)
    reference = load_table(reference_path) if reference_path.is_file() else None
    sea_props = parse_sea_props(
        model_path,
        [cfg.sea_knee_name, cfg.sea_ankle_name],
    )

    checks: List[Check] = []
    validate_run_status(results_dir, prefix, checks)
    validate_sea_tautology(cfg, tables, sea_props, checks)
    validate_sea_derivatives(cfg, tables, checks)
    validate_sea_interface_diagnostics(cfg, tables, checks)
    validate_reserves(cfg, tables, checks)
    validate_tracking(cfg, tables, reference, checks)
    validate_power_and_controls(cfg, tables, checks)
    return checks


def print_check(status: str, name: str, detail: str) -> bool:
    print(f"[{status}] {name}: {detail}")
    return status == "PASS"


def parse_args() -> argparse.Namespace:
    cfg = SimulatorConfig()
    parser = argparse.ArgumentParser(
        description="Strict CMC-like acceptance metrics for SEA runs."
    )
    parser.add_argument("--results-dir", default=cfg.output_dir)
    parser.add_argument("--prefix", default=cfg.output_prefix)
    parser.add_argument("--model", default=cfg.model_file)
    parser.add_argument("--reference", default=cfg.kinematics_file)
    parser.add_argument("--tracking-rms-deg", type=float, default=3.0)
    parser.add_argument("--tracking-max-deg", type=float, default=10.0)
    parser.add_argument("--hpf-max", type=float, default=0.05)
    parser.add_argument("--max-u", type=float, default=0.95)
    parser.add_argument("--max-tau-input-raw", type=float, default=450.0)
    parser.add_argument("--min-feasibility-scale-ones", type=float, default=0.95)
    parser.add_argument("--cutoff-hz", type=float, default=50.0)
    parser.add_argument("--skip-s", type=float, default=0.1)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    cfg = SimulatorConfig()
    results_dir = resolve_path(args.results_dir)
    model_path = resolve_path(args.model)
    reference_path = resolve_path(args.reference)
    prefix = args.prefix

    failures = 0
    if not results_dir.is_dir():
        raise FileNotFoundError(f"Results directory not found: {results_dir}")
    if not model_path.is_file():
        raise FileNotFoundError(f"Model file not found: {model_path}")
    if not reference_path.is_file():
        raise FileNotFoundError(f"Reference file not found: {reference_path}")

    status = load_run_status(results_dir, prefix)
    complete = status.get("status") == "complete"
    failures += not print_check(
        "PASS" if complete else "FAIL",
        "run status",
        f"status={status.get('status', 'missing')} t={status.get('t', '?')}",
    )

    finite_ok, finite_bad = all_sto_outputs_finite(results_dir, prefix)
    failures += not print_check(
        "PASS" if finite_ok else "FAIL",
        "finite STO outputs",
        "all finite" if finite_ok else "; ".join(finite_bad[:5]),
    )

    validator_checks = run_validator_checks(
        cfg, results_dir, prefix, model_path, reference_path
    )
    validator_failures = [check for check in validator_checks if check.status == "FAIL"]
    failures += not print_check(
        "PASS" if not validator_failures else "FAIL",
        "validator FAIL checks",
        "none" if not validator_failures else "; ".join(
            f"{check.name}: {check.detail}" for check in validator_failures[:5]
        ),
    )

    for label, _sea_name, coord in SEA_COORDS:
        s = tracking_stats_deg(results_dir, prefix, reference_path, coord)
        ok = (
            s["rms_deg"] <= args.tracking_rms_deg
            and s["max_abs_deg"] <= args.tracking_max_deg
        )
        failures += not print_check(
            "PASS" if ok else "FAIL",
            f"{label} tracking",
            (
                f"rms={s['rms_deg']:.3f} deg, "
                f"max={s['max_abs_deg']:.3f} deg, "
                f"mean={s['mean_abs_deg']:.3f} deg"
            ),
        )

    f_opt = load_f_opt_from_model(model_path)
    hpf = analyze_sto(
        results_dir / f"{prefix}_sea_torques.sto",
        results_dir / f"{prefix}_sea_controls.sto",
        cutoff_hz=args.cutoff_hz,
        skip_s=args.skip_s,
        f_opt=f_opt,
    )
    for label, _sea_name, _coord in SEA_COORDS:
        key = f"{label}_hpf_noise_tau_input"
        value = hpf.get(key, float("nan"))
        ok = math.isfinite(value) and value <= args.hpf_max
        failures += not print_check(
            "PASS" if ok else "FAIL",
            f"{label} HPF tau_input",
            f"{value:.6f} <= {args.hpf_max:.6f}" if math.isfinite(value) else "missing/NaN",
        )

    diag_path = results_dir / f"{prefix}_sea_diagnostics.sto"
    controls_path = results_dir / f"{prefix}_sea_controls.sto"
    for label, sea_name, coord in SEA_COORDS:
        sat = load_column(diag_path, f"{sea_name}_tau_input_saturated")
        sat_count = int(np.sum(sat > 0.5))
        failures += not print_check(
            "PASS" if sat_count == 0 else "FAIL",
            f"{label} tau_input saturation",
            f"count={sat_count}",
        )

        raw = load_column(diag_path, f"{sea_name}_tau_input_raw")
        max_raw = float(np.nanmax(np.abs(raw)))
        failures += not print_check(
            "PASS" if max_raw < args.max_tau_input_raw else "FAIL",
            f"{label} max |tau_input_raw|",
            f"{max_raw:.3f} Nm < {args.max_tau_input_raw:.3f} Nm",
        )

        u = load_column(controls_path, coord)
        max_u = float(np.nanmax(np.abs(u)))
        failures += not print_check(
            "PASS" if max_u < args.max_u else "FAIL",
            f"{label} max |u|",
            f"{max_u:.6f} < {args.max_u:.6f}",
        )

        scale = load_optional_column(diag_path, f"{sea_name}_sea_feasibility_scale")
        if scale is not None:
            one_fraction = float(np.mean(scale >= 0.999999))
            min_scale = float(np.nanmin(scale))
            ok = one_fraction >= args.min_feasibility_scale_ones
            failures += not print_check(
                "PASS" if ok else "FAIL",
                f"{label} SEA feasibility scale",
                (
                    f"scale==1 fraction={one_fraction:.3f} >= "
                    f"{args.min_feasibility_scale_ones:.3f}; "
                    f"min={min_scale:.6f}"
                ),
            )

    print(f"Summary: {'PASS' if failures == 0 else 'FAIL'} ({failures} failing checks)")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
