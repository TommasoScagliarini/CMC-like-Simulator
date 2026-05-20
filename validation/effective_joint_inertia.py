#!/usr/bin/env python3
"""
Estimate the effective rotational inertia seen by the prosthetic joints.

The cascade velocity loop was designed with a scalar plant approximation:

    qdot_dot = tau / J_joint

This script samples the OpenSim mass matrix along the IK trajectory and reports
several local inertia estimates for pros_knee_angle and pros_ankle_angle.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List

import numpy as np
import opensim

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from config import SimulatorConfig  # noqa: E402
from inverse_dynamics import build_mass_matrix  # noqa: E402
from kinematics_interpolator import KinematicsInterpolator  # noqa: E402
from model_loader import SimulationContext, setup_model  # noqa: E402
from setup_io import read_setup_xml  # noqa: E402


DEFAULT_SETUP = (
    "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml"
)
DEFAULT_BACKCALC = {
    "pros_knee_angle": 0.316,
    "pros_ankle_angle": 0.0193,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Estimate effective inertia for prosthetic knee and ankle."
    )
    parser.add_argument(
        "--setup",
        default=DEFAULT_SETUP,
        help="Simulator setup XML to sample.",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=181,
        help="Number of evenly-spaced IK states to sample.",
    )
    parser.add_argument(
        "--t-start",
        type=float,
        default=None,
        help="Override sampling start time [s].",
    )
    parser.add_argument(
        "--t-end",
        type=float,
        default=None,
        help="Override sampling end time [s].",
    )
    parser.add_argument(
        "--out-dir",
        default=None,
        help="Output directory. Defaults to results/_effective_joint_inertia_<timestamp>.",
    )
    parser.add_argument(
        "--coords",
        default="pros_knee_angle,pros_ankle_angle",
        help="Comma-separated coordinates to report.",
    )
    parser.add_argument(
        "--backcalc-knee",
        type=float,
        default=DEFAULT_BACKCALC["pros_knee_angle"],
        help="Back-calculated knee inertia to compare [kg*m^2].",
    )
    parser.add_argument(
        "--backcalc-ankle",
        type=float,
        default=DEFAULT_BACKCALC["pros_ankle_angle"],
        help="Back-calculated ankle inertia to compare [kg*m^2].",
    )
    return parser.parse_args()


def apply_setup(cfg: SimulatorConfig, setup_path: str) -> None:
    setup = read_setup_xml(setup_path)
    cfg.model_file = str(setup.model_file)
    cfg.kinematics_file = str(setup.kinematics_file)
    cfg.external_loads_xml = str(setup.external_loads_xml)
    cfg.reserve_actuators_xml = str(setup.reserve_actuators_xml)
    cfg.t_start = setup.t_start
    cfg.t_end = setup.t_end
    cfg.model_bundle_dir = str(setup.model_file.parent)


def parse_coord_list(raw: str) -> List[str]:
    coords = [item.strip() for item in raw.split(",") if item.strip()]
    if not coords:
        raise ValueError("At least one coordinate is required.")
    return coords


def set_state_from_reference(
    ctx: SimulationContext,
    state: opensim.State,
    q: Dict[str, float],
    qdot: Dict[str, float],
    time_value: float,
) -> None:
    state.setTime(float(time_value))
    sv = ctx.model.getStateVariableValues(state)

    for name, value in q.items():
        idx = ctx.q_sv_idx.get(name)
        if idx is not None:
            sv.set(idx, float(value))

    for name, value in qdot.items():
        idx = ctx.qdot_sv_idx.get(name)
        if idx is not None:
            sv.set(idx, float(value))

    ctx.model.setStateVariableValues(state, sv)
    ctx.model.realizePosition(state)


def finite_stats(values: Iterable[float]) -> Dict[str, float]:
    arr = np.array([float(v) for v in values if math.isfinite(float(v))])
    if arr.size == 0:
        return {
            "min": math.nan,
            "p05": math.nan,
            "median": math.nan,
            "mean": math.nan,
            "p95": math.nan,
            "max": math.nan,
            "std": math.nan,
        }
    return {
        "min": float(np.min(arr)),
        "p05": float(np.percentile(arr, 5.0)),
        "median": float(np.median(arr)),
        "mean": float(np.mean(arr)),
        "p95": float(np.percentile(arr, 95.0)),
        "max": float(np.max(arr)),
        "std": float(np.std(arr)),
    }


def summarize_rows(
    rows: List[Dict[str, float | str]],
    coords: List[str],
    backcalc: Dict[str, float],
) -> Dict[str, Dict[str, float]]:
    summary: Dict[str, Dict[str, float]] = {}
    fields = ("j_free_all", "j_locked", "j_free_pros_pair")

    for coord in coords:
        coord_rows = [row for row in rows if row["coord"] == coord]
        coord_summary: Dict[str, float] = {
            "samples": float(len(coord_rows)),
            "backcalc": float(backcalc.get(coord, math.nan)),
        }
        for field in fields:
            stats = finite_stats(float(row[field]) for row in coord_rows)
            for name, value in stats.items():
                coord_summary[f"{field}_{name}"] = value
            bc = coord_summary["backcalc"]
            median = coord_summary[f"{field}_median"]
            coord_summary[f"{field}_median_over_backcalc"] = (
                median / bc if math.isfinite(bc) and abs(bc) > 1e-12 else math.nan
            )
        summary[coord] = coord_summary

    return summary


def write_csv(path: Path, rows: List[Dict[str, float | str]]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_summary_markdown(
    path: Path,
    setup_path: str,
    cfg: SimulatorConfig,
    coords: List[str],
    summary: Dict[str, Dict[str, float]],
) -> None:
    lines = [
        "# Effective joint inertia validation",
        "",
        f"setup: `{setup_path}`",
        f"model: `{cfg.model_file}`",
        f"time window: `{cfg.t_start:.6g} -> {cfg.t_end:.6g} s`",
        "",
        "Definitions:",
        "",
        "- `J_free_all = 1 / (M^-1)_ii`: response to a unit torque with all model DOFs free.",
        "- `J_locked = M_ii`: inertia if all other DOFs are held at zero acceleration.",
        "- `J_free_pros_pair`: response with only the two prosthetic DOFs free.",
        "",
        "| coord | backcalc | J_free_all median | ratio | J_locked median | ratio | J_free_pros_pair median | ratio |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]

    for coord in coords:
        item = summary[coord]
        lines.append(
            "| {coord} | {bc:.6g} | {free:.6g} | {free_r:.3g} | "
            "{locked:.6g} | {locked_r:.3g} | {pair:.6g} | {pair_r:.3g} |".format(
                coord=coord,
                bc=item["backcalc"],
                free=item["j_free_all_median"],
                free_r=item["j_free_all_median_over_backcalc"],
                locked=item["j_locked_median"],
                locked_r=item["j_locked_median_over_backcalc"],
                pair=item["j_free_pros_pair_median"],
                pair_r=item["j_free_pros_pair_median_over_backcalc"],
            )
        )

    lines.extend([
        "",
        "Full percentile statistics are in `summary.json`; per-sample values are in `samples.csv`.",
        "",
    ])
    path.write_text("\n".join(lines), encoding="utf-8")


def estimate(
    cfg: SimulatorConfig,
    ctx: SimulationContext,
    coords: List[str],
    samples: int,
) -> List[Dict[str, float | str]]:
    kin = KinematicsInterpolator(cfg)
    matter = ctx.model.getMatterSubsystem()
    state = ctx.state
    n_mob = ctx.n_mob
    e_vec = opensim.Vector(n_mob, 0.0)
    me_vec = opensim.Vector(n_mob, 0.0)

    times = np.linspace(float(cfg.t_start), float(cfg.t_end), int(samples))
    pros_indices = [ctx.coord_mob_idx[name] for name in coords]
    rows: List[Dict[str, float | str]] = []

    for sample_idx, time_value in enumerate(times):
        q, qdot, _qddot = kin.get(float(time_value))
        set_state_from_reference(ctx, state, q, qdot, float(time_value))
        mass = build_mass_matrix(matter, state, n_mob, e_vec, me_vec)
        mass_inv = np.linalg.inv(mass)
        pros_mass = mass[np.ix_(pros_indices, pros_indices)]
        pros_mass_inv = np.linalg.inv(pros_mass)

        for local_idx, coord in enumerate(coords):
            mob_idx = ctx.coord_mob_idx[coord]
            inv_diag = float(mass_inv[mob_idx, mob_idx])
            pros_inv_diag = float(pros_mass_inv[local_idx, local_idx])
            rows.append({
                "sample": sample_idx,
                "time": float(time_value),
                "coord": coord,
                "q": float(q.get(coord, math.nan)),
                "qdot": float(qdot.get(coord, math.nan)),
                "mobility_index": mob_idx,
                "j_free_all": 1.0 / inv_diag,
                "j_locked": float(mass[mob_idx, mob_idx]),
                "j_free_pros_pair": 1.0 / pros_inv_diag,
                "minv_diag": inv_diag,
                "pros_minv_diag": pros_inv_diag,
            })

        if (sample_idx + 1) % max(1, len(times) // 10) == 0:
            print(f"[Inertia] sampled {sample_idx + 1}/{len(times)}")

    return rows


def main() -> int:
    args = parse_args()
    coords = parse_coord_list(args.coords)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = (
        Path(args.out_dir)
        if args.out_dir is not None
        else REPO_ROOT / "results" / f"_effective_joint_inertia_{timestamp}"
    )
    if not out_dir.is_absolute():
        out_dir = REPO_ROOT / out_dir

    cfg = SimulatorConfig()
    apply_setup(cfg, args.setup)
    if args.t_start is not None:
        cfg.t_start = args.t_start
    if args.t_end is not None:
        cfg.t_end = args.t_end
    if args.samples < 2:
        raise ValueError("--samples must be >= 2")

    backcalc = {
        "pros_knee_angle": float(args.backcalc_knee),
        "pros_ankle_angle": float(args.backcalc_ankle),
    }

    print(f"[Inertia] setup    : {args.setup}")
    print(f"[Inertia] model    : {cfg.model_file}")
    print(f"[Inertia] window   : {cfg.t_start:.6g} -> {cfg.t_end:.6g} s")
    print(f"[Inertia] samples  : {args.samples}")
    print(f"[Inertia] out_dir  : {out_dir}")

    ctx = setup_model(cfg)
    missing = [coord for coord in coords if coord not in ctx.coord_mob_idx]
    if missing:
        raise RuntimeError(f"Coordinates not found in model: {missing}")

    rows = estimate(cfg, ctx, coords, int(args.samples))
    summary = summarize_rows(rows, coords, backcalc)

    out_dir.mkdir(parents=True, exist_ok=True)
    write_csv(out_dir / "samples.csv", rows)
    with (out_dir / "summary.json").open("w", encoding="utf-8") as fh:
        json.dump(summary, fh, indent=2, sort_keys=True)
        fh.write("\n")
    write_summary_markdown(out_dir / "summary.md", args.setup, cfg, coords, summary)

    print("[Inertia] summary:")
    for coord in coords:
        item = summary[coord]
        print(
            "  {coord}: backcalc={bc:.6g}, "
            "J_free_all_median={free:.6g} ({free_r:.3g}x), "
            "J_locked_median={locked:.6g} ({locked_r:.3g}x), "
            "J_free_pros_pair_median={pair:.6g} ({pair_r:.3g}x)".format(
                coord=coord,
                bc=item["backcalc"],
                free=item["j_free_all_median"],
                free_r=item["j_free_all_median_over_backcalc"],
                locked=item["j_locked_median"],
                locked_r=item["j_locked_median_over_backcalc"],
                pair=item["j_free_pros_pair_median"],
                pair_r=item["j_free_pros_pair_median_over_backcalc"],
            )
        )
    print(f"[Inertia] wrote: {out_dir / 'summary.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
