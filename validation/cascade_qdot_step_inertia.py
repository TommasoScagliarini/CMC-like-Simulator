#!/usr/bin/env python3
"""
Check the knee J_eff choice with a qdot_ref step.

For the cascade controller, with zero position error and zero integral memory,
a velocity-reference step produces the immediate high-level command

    tau_cmd = Kp_inner * delta_qdot_ref

This script applies that torque to the sampled OpenSim mass matrix and reports
the initial acceleration predicted by the free-all, prosthetic-pair, and locked
inertia assumptions.  It is intentionally an input-output plant check; it does
not change the runtime controller or the C++ SEA plugin.
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
from model_loader import setup_model  # noqa: E402
from validation.effective_joint_inertia import (  # noqa: E402
    DEFAULT_SETUP,
    apply_setup,
    finite_stats,
    set_state_from_reference,
)


DEFAULT_DESIGNS = (
    ("morning_best", 29.2, 1377.0),
    ("v3a_j_free", 26.88, 2304.0),
    ("v3m_j_mid", 45.36, 3888.0),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--setup", default=DEFAULT_SETUP)
    parser.add_argument("--coord", default="pros_knee_angle")
    parser.add_argument("--samples", type=int, default=181)
    parser.add_argument("--t-start", type=float, default=None)
    parser.add_argument("--t-end", type=float, default=None)
    parser.add_argument(
        "--step-qdot",
        type=float,
        default=1.0,
        help="Velocity-reference step [rad/s].",
    )
    parser.add_argument(
        "--f-opt",
        type=float,
        default=100.0,
        help="SEA optimal_force torque limit used for command clipping [N*m].",
    )
    parser.add_argument(
        "--out-dir",
        default=None,
        help="Output directory. Defaults to results/_cascade_qdot_step_<timestamp>.",
    )
    return parser.parse_args()


def write_csv(path: Path, rows: List[Dict[str, float | str]]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def summarize_rows(rows: List[Dict[str, float | str]]) -> Dict[str, Dict[str, float]]:
    summary: Dict[str, Dict[str, float]] = {}
    for design in sorted({str(row["design"]) for row in rows}):
        design_rows = [row for row in rows if row["design"] == design]
        item: Dict[str, float] = {
            "samples": float(len(design_rows)),
            "kp_inner": float(design_rows[0]["kp_inner"]),
            "ki_inner": float(design_rows[0]["ki_inner"]),
            "tau_cmd": float(design_rows[0]["tau_cmd"]),
            "tau_cmd_clipped": float(design_rows[0]["tau_cmd_clipped"]),
        }
        for field in (
            "j_free_all",
            "j_free_pros_pair",
            "j_locked",
            "qddot_free_all",
            "qddot_free_pros_pair",
            "qddot_locked",
            "zeta_if_free_all",
            "zeta_if_free_pros_pair",
            "zeta_if_locked",
        ):
            stats = finite_stats(float(row[field]) for row in design_rows)
            for stat_name, value in stats.items():
                item[f"{field}_{stat_name}"] = value
        summary[design] = item
    return summary


def estimate_step_response(
    cfg: SimulatorConfig,
    coord: str,
    samples: int,
    step_qdot: float,
    f_opt: float,
) -> List[Dict[str, float | str]]:
    ctx = setup_model(cfg)
    if coord not in ctx.coord_mob_idx:
        raise RuntimeError(f"Coordinate not found in model: {coord}")

    pros_coords = [name for name in cfg.pros_coords if name in ctx.coord_mob_idx]
    if coord not in pros_coords:
        raise RuntimeError(
            f"{coord!r} is not in cfg.pros_coords; cannot compute pros-pair inertia."
        )
    pros_indices = [ctx.coord_mob_idx[name] for name in pros_coords]
    local_idx = pros_coords.index(coord)
    mob_idx = ctx.coord_mob_idx[coord]

    kin = KinematicsInterpolator(cfg)
    matter = ctx.model.getMatterSubsystem()
    state = ctx.state
    e_vec = opensim.Vector(ctx.n_mob, 0.0)
    me_vec = opensim.Vector(ctx.n_mob, 0.0)

    rows: List[Dict[str, float | str]] = []
    times = np.linspace(float(cfg.t_start), float(cfg.t_end), int(samples))
    for sample_idx, time_value in enumerate(times):
        q, qdot, _qddot = kin.get(float(time_value))
        set_state_from_reference(ctx, state, q, qdot, float(time_value))
        mass = build_mass_matrix(matter, state, ctx.n_mob, e_vec, me_vec)
        mass_inv = np.linalg.inv(mass)
        pros_mass = mass[np.ix_(pros_indices, pros_indices)]
        pros_mass_inv = np.linalg.inv(pros_mass)

        inv_diag = float(mass_inv[mob_idx, mob_idx])
        pair_inv_diag = float(pros_mass_inv[local_idx, local_idx])
        j_free_all = 1.0 / inv_diag
        j_free_pair = 1.0 / pair_inv_diag
        j_locked = float(mass[mob_idx, mob_idx])

        for design, kp_inner, ki_inner in DEFAULT_DESIGNS:
            tau_cmd = float(kp_inner * step_qdot)
            tau_cmd_clipped = float(np.clip(tau_cmd, -abs(f_opt), abs(f_opt)))
            rows.append({
                "sample": sample_idx,
                "time": float(time_value),
                "coord": coord,
                "design": design,
                "step_qdot": float(step_qdot),
                "kp_inner": float(kp_inner),
                "ki_inner": float(ki_inner),
                "tau_cmd": tau_cmd,
                "tau_cmd_clipped": tau_cmd_clipped,
                "j_free_all": j_free_all,
                "j_free_pros_pair": j_free_pair,
                "j_locked": j_locked,
                "qddot_free_all": tau_cmd_clipped * inv_diag,
                "qddot_free_pros_pair": tau_cmd_clipped * pair_inv_diag,
                "qddot_locked": tau_cmd_clipped / j_locked,
                "zeta_if_free_all": kp_inner / (2.0 * math.sqrt(j_free_all * ki_inner)),
                "zeta_if_free_pros_pair": kp_inner / (2.0 * math.sqrt(j_free_pair * ki_inner)),
                "zeta_if_locked": kp_inner / (2.0 * math.sqrt(j_locked * ki_inner)),
            })

        if (sample_idx + 1) % max(1, len(times) // 10) == 0:
            print(f"[QdotStep] sampled {sample_idx + 1}/{len(times)}")

    return rows


def markdown_table_row(design: str, item: Dict[str, float]) -> str:
    return (
        "| {design} | {kp:.4g} | {ki:.4g} | {tau:.4g} | "
        "{jf:.4g} | {qf:.4g} | {jp:.4g} | {qp:.4g} | "
        "{jl:.4g} | {ql:.4g} | {zf:.3g} | {zp:.3g} | {zl:.3g} |"
    ).format(
        design=design,
        kp=item["kp_inner"],
        ki=item["ki_inner"],
        tau=item["tau_cmd_clipped"],
        jf=item["j_free_all_median"],
        qf=item["qddot_free_all_median"],
        jp=item["j_free_pros_pair_median"],
        qp=item["qddot_free_pros_pair_median"],
        jl=item["j_locked_median"],
        ql=item["qddot_locked_median"],
        zf=item["zeta_if_free_all_median"],
        zp=item["zeta_if_free_pros_pair_median"],
        zl=item["zeta_if_locked_median"],
    )


def write_summary_markdown(
    path: Path,
    setup_path: str,
    cfg: SimulatorConfig,
    coord: str,
    step_qdot: float,
    f_opt: float,
    summary: Dict[str, Dict[str, float]],
) -> None:
    lines = [
        "# Cascade qdot_ref step inertia check",
        "",
        f"setup: `{setup_path}`",
        f"model: `{cfg.model_file}`",
        f"coord: `{coord}`",
        f"time window: `{cfg.t_start:.6g} -> {cfg.t_end:.6g} s`",
        f"step: `delta_qdot_ref = {step_qdot:.6g} rad/s`",
        f"command clip: `+/-{abs(f_opt):.6g} N*m`",
        "",
        "At zero position error and zero cascade integral memory, the immediate command is:",
        "",
        "`tau_cmd = Kp_inner * delta_qdot_ref`",
        "",
        "| design | Kp_inner | Ki_inner | tau step | J_free | qddot free | J_pair | qddot pair | J_locked | qddot locked | zeta free | zeta pair | zeta locked |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for design in DEFAULT_DESIGNS:
        name = design[0]
        lines.append(markdown_table_row(name, summary[name]))

    ref = summary["v3m_j_mid"]
    free_to_pair = (
        ref["qddot_free_all_median"] / ref["qddot_free_pros_pair_median"]
        if abs(ref["qddot_free_pros_pair_median"]) > 1e-12
        else math.nan
    )
    pair_to_locked = (
        ref["qddot_free_pros_pair_median"] / ref["qddot_locked_median"]
        if abs(ref["qddot_locked_median"]) > 1e-12
        else math.nan
    )
    lines.extend([
        "",
        "Interpretation:",
        "",
        (
            "- The same knee qdot_ref step gives "
            f"{free_to_pair:.3g}x more acceleration with `J_free_all` than "
            "with `J_free_pros_pair`."
        ),
        (
            "- `J_free_pros_pair` and `J_locked` are close for this step "
            f"({pair_to_locked:.3g}x acceleration ratio), so they form the "
            "conservative knee bracket."
        ),
        "- Full per-sample values are in `samples.csv`; percentile statistics are in `summary.json`.",
        "",
    ])
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    args = parse_args()
    if args.samples < 2:
        raise ValueError("--samples must be >= 2")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = (
        Path(args.out_dir)
        if args.out_dir is not None
        else REPO_ROOT / "results" / f"_cascade_qdot_step_{timestamp}"
    )
    if not out_dir.is_absolute():
        out_dir = REPO_ROOT / out_dir

    cfg = SimulatorConfig()
    apply_setup(cfg, args.setup)
    if args.t_start is not None:
        cfg.t_start = args.t_start
    if args.t_end is not None:
        cfg.t_end = args.t_end

    print(f"[QdotStep] setup    : {args.setup}")
    print(f"[QdotStep] model    : {cfg.model_file}")
    print(f"[QdotStep] coord    : {args.coord}")
    print(f"[QdotStep] window   : {cfg.t_start:.6g} -> {cfg.t_end:.6g} s")
    print(f"[QdotStep] samples  : {args.samples}")
    print(f"[QdotStep] out_dir  : {out_dir}")

    rows = estimate_step_response(
        cfg,
        str(args.coord),
        int(args.samples),
        float(args.step_qdot),
        float(args.f_opt),
    )
    summary = summarize_rows(rows)

    out_dir.mkdir(parents=True, exist_ok=True)
    write_csv(out_dir / "samples.csv", rows)
    with (out_dir / "summary.json").open("w", encoding="utf-8") as fh:
        json.dump(summary, fh, indent=2, sort_keys=True)
        fh.write("\n")
    write_summary_markdown(
        out_dir / "summary.md",
        args.setup,
        cfg,
        str(args.coord),
        float(args.step_qdot),
        float(args.f_opt),
        summary,
    )

    for name, item in summary.items():
        print(
            "[QdotStep] {name}: tau={tau:.4g} Nm, qddot medians "
            "free={free:.4g}, pair={pair:.4g}, locked={locked:.4g} rad/s^2".format(
                name=name,
                tau=item["tau_cmd_clipped"],
                free=item["qddot_free_all_median"],
                pair=item["qddot_free_pros_pair_median"],
                locked=item["qddot_locked_median"],
            )
        )
    print(f"[QdotStep] wrote: {out_dir / 'summary.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
