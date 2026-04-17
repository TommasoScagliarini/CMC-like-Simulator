"""
main.py
=======
Entry point for the prosthetic gait simulator.

Usage
-----
    python main.py [--config config_override.yaml]
    python main.py --plot
    python main.py --validate
    python main.py --log

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
import contextlib
import os
import subprocess
import sys
import time as _time
import traceback
from datetime import datetime

from config import SimulatorConfig
from model_loader import setup_model
from kinematics_interpolator import KinematicsInterpolator
from simulation_runner import SimulationRunner


class _TeeStream:
    """Text stream that mirrors writes to multiple streams."""

    def __init__(self, *streams) -> None:
        self._streams = streams

    def write(self, text: str) -> int:
        for stream in self._streams:
            stream.write(text)
        return len(text)

    def flush(self) -> None:
        for stream in self._streams:
            stream.flush()


def _phase3_log_path(cfg: SimulatorConfig) -> str:
    """Return a timestamped log path inside the active output directory."""
    os.makedirs(cfg.output_dir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{cfg.output_prefix}_phase3_log_{stamp}.txt"
    return os.path.join(cfg.output_dir, filename)


def _run_phase3(cfg: SimulatorConfig, ctx, kin) -> int:
    """Construct the runner and execute only the simulation phase."""
    # ── Step 3: Build runner and simulate ────────────────────────────────────
    print("\n[Main] Step 3/3 - Starting simulation ...")
    try:
        runner = SimulationRunner(cfg, ctx, kin)
        runner.run()
    except Exception as exc:
        print(f"\n[Main] ERROR during simulation:\n  {exc}")
        traceback.print_exc()
        return 1

    print("\n[Main] Done.")
    return 0


def _run_phase3_with_log(cfg: SimulatorConfig, ctx, kin) -> int:
    """Run phase 3 while teeing stdout/stderr to a log file."""
    log_path = _phase3_log_path(cfg)
    print(f"\n[Main] Phase 3 log: {log_path}")

    with open(log_path, "w", encoding="utf-8", errors="replace") as log_fh:
        log_fh.write("Simulation phase 3 log\n")
        log_fh.write(f"created_at={datetime.now().isoformat(timespec='seconds')}\n")
        log_fh.write(f"output_dir={cfg.output_dir}\n")
        log_fh.write(f"output_prefix={cfg.output_prefix}\n")
        log_fh.write(f"t_start={cfg.t_start}\n")
        log_fh.write(f"t_end={cfg.t_end}\n")
        log_fh.write(f"dt={cfg.dt}\n")
        log_fh.write(f"sea_forward_mode={cfg.sea_forward_mode}\n")
        log_fh.write("\n")
        log_fh.flush()

        tee_stdout = _TeeStream(sys.stdout, log_fh)
        tee_stderr = _TeeStream(sys.stderr, log_fh)
        with contextlib.redirect_stdout(tee_stdout), contextlib.redirect_stderr(tee_stderr):
            exit_code = _run_phase3(cfg, ctx, kin)

    print(f"[Main] Simulation log saved: {log_path}")
    return exit_code


def main(cfg: SimulatorConfig, log_simulation: bool = False) -> int:
    """
    Run the full simulation pipeline.

    Returns
    -------
    0 on success, 1 on error.
    """
    print("=" * 65)
    print("  Prosthetic Gait Simulator - Custom CMC Replacement")
    print("=" * 65)

    # ── Step 1: Load model, plugin, GRF, reserve actuators ───────────────────
    print("\n[Main] Step 1/3 - Loading model ...")
    try:
        ctx = setup_model(cfg)
    except Exception as exc:
        print(f"\n[Main] ERROR during model setup:\n  {exc}")
        traceback.print_exc()
        return 1

    # ── Step 2: Build kinematic interpolator ─────────────────────────────────
    print("\n[Main] Step 2/3 - Loading kinematics ...")
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

    if log_simulation:
        return _run_phase3_with_log(cfg, ctx, kin)
    return _run_phase3(cfg, ctx, kin)


# ─────────────────────────────────────────────────────────────────────────────
#  CLI
# ─────────────────────────────────────────────────────────────────────────────
def _parse_args():
    """
    Parse command-line arguments and return a config plus raw CLI args.

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
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Generate plot PNGs after a successful simulation run.",
    )
    parser.add_argument(
        "--validate",
        action="store_true",
        help="Validate saved results after a successful simulation run.",
    )
    parser.add_argument(
        "--log",
        action="store_true",
        help="Save the phase 3 simulation console log to a txt in output-dir.",
    )
    parser.add_argument(
        "--sea-forward-mode",
        choices=["plugin", "ideal_torque"],
        default=None,
        help="SEA forward mode: plugin dynamics or legacy ideal torque baseline.",
    )
    parser.add_argument(
        "--sea-motor-substeps",
        type=int,
        default=None,
        help="Override numerical substeps used for SEA motor state integration.",
    )
    parser.add_argument(
        "--sea-motor-max-substeps",
        type=int,
        default=None,
        help="Override maximum retry substeps for SEA motor state integration.",
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
    if args.sea_forward_mode is not None:
        cfg.sea_forward_mode = args.sea_forward_mode
    if args.sea_motor_substeps is not None:
        cfg.sea_motor_substeps = args.sea_motor_substeps
    if args.sea_motor_max_substeps is not None:
        cfg.sea_motor_max_substeps = args.sea_motor_max_substeps

    return cfg, args


def _run_plotter(cfg: SimulatorConfig) -> int:
    """Run plot/plotter.py with paths from the active simulator config."""
    plotter_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "plot",
        "plotter.py",
    )
    cmd = [
        sys.executable,
        plotter_path,
        "--results-dir",
        cfg.output_dir,
        "--prefix",
        cfg.output_prefix,
        "--gait-side",
        cfg.plot_gait_side,
    ]

    print("\n[Main] Plotting results ...")
    try:
        completed = subprocess.run(cmd, check=False)
    except FileNotFoundError as exc:
        print(f"[Main] ERROR launching plotter:\n  {exc}")
        return 1

    if completed.returncode != 0:
        print(f"[Main] Plotter failed with exit code {completed.returncode}.")
    return completed.returncode


def _run_validator(cfg: SimulatorConfig) -> int:
    """Run validation/validate_sim_results.py with the active config paths."""
    validator_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "validation",
        "validate_sim_results.py",
    )
    report_name = f"{datetime.now().strftime('%Y-%m-%d')}_validazione_simulatore.md"
    report_path = os.path.join("reports", "user", report_name)
    cmd = [
        sys.executable,
        validator_path,
        "--results-dir",
        cfg.output_dir,
        "--prefix",
        cfg.output_prefix,
        "--model",
        cfg.model_file,
        "--reference",
        cfg.kinematics_file,
        "--out",
        report_path,
    ]

    print("\n[Main] Validating results ...")
    try:
        completed = subprocess.run(cmd, check=False)
    except FileNotFoundError as exc:
        print(f"[Main] ERROR launching validator:\n  {exc}")
        return 1

    if completed.returncode != 0:
        print(f"[Main] Validator reported FAIL with exit code {completed.returncode}.")
    return completed.returncode


if __name__ == "__main__":
    _wall_t0 = _time.perf_counter()

    cfg, args = _parse_args()

    # Print key configuration parameters
    _KEY_FIELDS = {
        "model_file", "kinematics_file",
        "t_start", "t_end", "dt",
        "output_dir", "output_prefix",
        "sea_forward_mode", "qp_solver",
    }
    print("\nActive configuration:")
    for field_name, value in cfg.__dict__.items():
        if field_name in _KEY_FIELDS:
            print(f"  {field_name:<30} = {value}")
    print()

    exit_code = main(cfg, log_simulation=args.log)
    if exit_code == 0 and args.plot:
        exit_code = _run_plotter(cfg)
    if exit_code == 0 and args.validate:
        exit_code = _run_validator(cfg)

    _total_elapsed = _time.perf_counter() - _wall_t0
    print(f"\n[Main] Total elapsed: {_total_elapsed:.1f} s")

    sys.exit(exit_code)
