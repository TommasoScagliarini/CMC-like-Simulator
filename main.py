"""
main.py
=======
Entry point for the prosthetic gait simulator.

Usage
-----
    python main.py [--config config_override.yaml]

In the absence of a config file, SimulatorConfig defaults are used.
Edit config.py directly to change file paths and parameters.

Pipeline overview
-----------------
  1. Load configuration
  2. Load model + plugin + GRF + reserves  →  SimulationContext
  3. Build kinematic interpolator          →  KinematicsInterpolator
  4. Construct SimulationRunner (instantiates all sub-components)
  5. Run simulation loop
  6. Results are saved automatically at end of run

Each simulation component lives in its own module:

  model_loader.py            – OpenSim model setup & index caching
  kinematics_interpolator.py – IK data loading & cubic-spline interpolation
  prosthesis_controller.py   – SEA outer PD loop (high-level)
  outer_loop.py              – Biological kinematic tracking PD
  inverse_dynamics.py        – Partial ID via M · Δq̈
  static_optimization.py     – QP for muscle activations & reserves
  simulation_runner.py       – Main while loop

See each module's docstring for detailed explanations.
"""

from __future__ import annotations

import argparse
import sys
import traceback

from config import SimulatorConfig
from model_loader import setup_model
from kinematics_interpolator import KinematicsInterpolator
from simulation_runner import SimulationRunner


def main(cfg: SimulatorConfig) -> int:
    """
    Run the full simulation pipeline.

    Returns
    -------
    0 on success, 1 on error.
    """
    print("=" * 65)
    print("  Prosthetic Gait Simulator – Custom CMC Replacement")
    print("=" * 65)

    # ── Step 1: Load model, plugin, GRF, reserve actuators ───────────────────
    print("\n[Main] Step 1/3 – Loading model …")
    try:
        ctx = setup_model(cfg)
    except Exception as exc:
        print(f"\n[Main] ERROR during model setup:\n  {exc}")
        traceback.print_exc()
        return 1

    # ── Step 2: Build kinematic interpolator ─────────────────────────────────
    print("\n[Main] Step 2/3 – Loading kinematics …")
    try:
        kin = KinematicsInterpolator(cfg)
    except Exception as exc:
        print(f"\n[Main] ERROR loading kinematics:\n  {exc}")
        traceback.print_exc()
        return 1

    # Validate: every coordinate in the model must be present in the IK file.
    missing_coords = [
        name for name in ctx.coord_names
        if name not in kin.coord_names
    ]
    if missing_coords:
        print(
            f"\n[Main] WARNING: The following model coordinates are not in "
            f"the kinematics file and will use q=0, q̇=0 as reference:\n"
            f"  {missing_coords}"
        )

    # ── Step 3: Build runner and simulate ────────────────────────────────────
    print("\n[Main] Step 3/3 – Starting simulation …")
    try:
        runner = SimulationRunner(cfg, ctx, kin)
        runner.run()
    except Exception as exc:
        print(f"\n[Main] ERROR during simulation:\n  {exc}")
        traceback.print_exc()
        return 1

    print("\n[Main] Done.")
    return 0


# ─────────────────────────────────────────────────────────────────────────────
#  CLI
# ─────────────────────────────────────────────────────────────────────────────
def _parse_args() -> SimulatorConfig:
    """
    Parse command-line arguments and return a (possibly modified) config.

    Currently supports --config for loading a YAML override file.
    You can extend this with argparse options for individual parameters.
    """
    parser = argparse.ArgumentParser(
        description="Prosthetic gait simulator (custom CMC replacement)"
    )
    parser.add_argument(
        "--model",
        default=None,
        help="Override model file path from config.py",
    )
    parser.add_argument(
        "--kinematics",
        default=None,
        help="Override kinematics (.sto) file path",
    )
    parser.add_argument(
        "--t-start",
        type=float,
        default=None,
        help="Override simulation start time [s]",
    )
    parser.add_argument(
        "--t-end",
        type=float,
        default=None,
        help="Override simulation end time [s]",
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=None,
        help="Override integration time step [s]",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Override output directory",
    )
    parser.add_argument(
        "--solver",
        choices=["slsqp", "osqp"],
        default=None,
        help="QP solver backend for static optimisation",
    )

    args = parser.parse_args()

    # Start from defaults and apply CLI overrides
    cfg = SimulatorConfig()
    if args.model         is not None: cfg.model_file   = args.model
    if args.kinematics    is not None: cfg.kinematics_file = args.kinematics
    if args.t_start       is not None: cfg.t_start      = args.t_start
    if args.t_end         is not None: cfg.t_end        = args.t_end
    if args.dt            is not None: cfg.dt           = args.dt
    if args.output_dir    is not None: cfg.output_dir   = args.output_dir
    if args.solver        is not None: cfg.qp_solver    = args.solver

    return cfg


if __name__ == "__main__":
    cfg = _parse_args()

    # Print active configuration for reproducibility
    print("\nActive configuration:")
    for field_name, value in cfg.__dict__.items():
        print(f"  {field_name:<30} = {value}")
    print()

    sys.exit(main(cfg))
