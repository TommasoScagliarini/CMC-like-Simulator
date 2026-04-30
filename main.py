"""
main.py
=======
Entry point for the prosthetic gait simulator.

Usage
-----
    python main.py
    python main.py --setup
    python main.py --setup path/to/setup.xml
    python main.py --plot
    python main.py --validate
    python main.py --log

By default the simulator uses the last valid setup XML that was loaded.
If no valid last setup is available, a file picker opens.
Edit config.py directly to change non-file parameters.

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
from kinematics_interpolator import KinematicsInterpolator
from model_loader import setup_model
from path_resolver import normalize_cli_existing_path, resolve_simulator_paths
from simulation_runner import SimulationRunner
from setup_io import (
    ask_open_setup_xml_path,
    read_last_setup_path,
    read_setup_xml,
    write_last_setup_state,
)


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


def _apply_setup_to_config(cfg: SimulatorConfig, setup) -> None:
    cfg.model_file = str(setup.model_file)
    cfg.kinematics_file = str(setup.kinematics_file)
    cfg.external_loads_xml = str(setup.external_loads_xml)
    cfg.reserve_actuators_xml = str(setup.reserve_actuators_xml)
    cfg.t_start = setup.t_start
    cfg.t_end = setup.t_end
    cfg.model_bundle_dir = str(setup.model_file.parent)


def _has_direct_path_overrides(args: argparse.Namespace) -> bool:
    return any(
        getattr(args, field_name) is not None
        for field_name in (
            "model_bundle",
            "model",
            "kinematics",
            "external_loads",
            "reserve_actuators",
        )
    )


def _prompt_for_valid_setup(initial_path: str | None = None):
    prompt_path = initial_path
    while True:
        selected = ask_open_setup_xml_path(prompt_path)
        if selected is None:
            raise RuntimeError("Setup XML selection cancelled.")
        try:
            setup = read_setup_xml(selected)
        except Exception as exc:
            print(f"[Main] Selected setup XML is invalid:\n  {exc}")
            prompt_path = str(selected)
            continue
        return str(selected), setup


def _load_setup_from_args(args: argparse.Namespace):
    if args.setup is not None:
        if args.setup == "":
            selected_path, setup = _prompt_for_valid_setup()
        else:
            selected_path = str(args.setup)
            setup = read_setup_xml(selected_path)
        write_last_setup_state(selected_path)
        return selected_path, setup

    if _has_direct_path_overrides(args):
        print("[Main] Direct file overrides detected; skipping setup XML auto-load.")
        return None, None

    remembered_path = read_last_setup_path()
    if remembered_path is not None:
        try:
            setup = read_setup_xml(remembered_path)
        except Exception as exc:
            print(f"[Main] Last setup XML is unavailable:\n  {exc}")
        else:
            selected_path = str(remembered_path)
            write_last_setup_state(selected_path)
            return selected_path, setup

    selected_path, setup = _prompt_for_valid_setup(
        str(remembered_path) if remembered_path is not None else None
    )
    write_last_setup_state(selected_path)
    return selected_path, setup


# ─────────────────────────────────────────────────────────────────────────────
#  CLI
# ─────────────────────────────────────────────────────────────────────────────
def _parse_args():
    """
    Parse command-line arguments and return a config plus raw CLI args.

    File inputs can come from a setup XML, from config.py, or from explicit
    CLI overrides. CLI overrides always win.
    """
    parser = argparse.ArgumentParser(
        description="Prosthetic gait simulator (custom CMC replacement)"
    )
    parser.add_argument(
        "--setup",
        nargs="?",
        const="",
        default=None,
        help="Load a simulator setup XML. Without a value, opens a file picker.",
    )
    parser.add_argument(
        "--model-bundle",
        default=None,
        help="Override model bundle directory from config.py",
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
        "--external-loads",
        default=None,
        help="Override ExternalLoads (.xml) file path",
    )
    parser.add_argument(
        "--reserve-actuators",
        default=None,
        help="Override reserve actuators (.xml) file path",
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
    parser.add_argument(
        "--sea-feasibility-scaling",
        action="store_true",
        help="Enable prosthetic PD feasibility scaling for high-gain SEA runs.",
    )
    parser.add_argument(
        "--sea-kp-knee",
        type=float,
        default=None,
        help="Override prosthetic knee outer Kp [N*m/rad].",
    )
    parser.add_argument(
        "--sea-kd-knee",
        type=float,
        default=None,
        help="Override prosthetic knee outer Kd [N*m*s/rad].",
    )
    parser.add_argument(
        "--sea-kp-ankle",
        type=float,
        default=None,
        help="Override prosthetic ankle outer Kp [N*m/rad].",
    )
    parser.add_argument(
        "--sea-kd-ankle",
        type=float,
        default=None,
        help="Override prosthetic ankle outer Kd [N*m*s/rad].",
    )
    parser.add_argument(
        "--disable-kinematics-lowpass",
        action="store_true",
        help="Disable IK low-pass preprocessing before spline construction.",
    )
    parser.add_argument(
        "--kinematics-lowpass-cutoff",
        type=float,
        default=None,
        help="Override IK low-pass cutoff frequency [Hz].",
    )

    args = parser.parse_args()

    # Start from defaults and apply CLI overrides
    cfg = SimulatorConfig()
    try:
        setup_path, setup = _load_setup_from_args(args)
    except RuntimeError as exc:
        parser.exit(1, f"[Main] {exc}\n")
    except Exception as exc:
        parser.exit(1, f"[Main] ERROR loading setup XML:\n  {exc}\n")
    if setup is not None:
        _apply_setup_to_config(cfg, setup)
        print(f"[Main] Using setup XML: {setup_path}")

    if args.model_bundle  is not None:
        cfg.model_bundle_dir = args.model_bundle
        if args.model is None:
            cfg.model_file = ""
    if args.model         is not None: cfg.model_file   = normalize_cli_existing_path(args.model)
    if args.kinematics    is not None: cfg.kinematics_file = normalize_cli_existing_path(args.kinematics)
    if args.external_loads is not None:
        cfg.external_loads_xml = normalize_cli_existing_path(args.external_loads)
    if args.reserve_actuators is not None:
        cfg.reserve_actuators_xml = normalize_cli_existing_path(args.reserve_actuators)
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
    if args.sea_feasibility_scaling:
        cfg.enable_sea_feasibility_scaling = True
    if args.sea_kp_knee is not None:
        cfg.sea_kp[cfg.pros_coords[0]] = args.sea_kp_knee
    if args.sea_kd_knee is not None:
        cfg.sea_kd[cfg.pros_coords[0]] = args.sea_kd_knee
    if args.sea_kp_ankle is not None:
        cfg.sea_kp[cfg.pros_coords[1]] = args.sea_kp_ankle
    if args.sea_kd_ankle is not None:
        cfg.sea_kd[cfg.pros_coords[1]] = args.sea_kd_ankle
    if args.disable_kinematics_lowpass:
        cfg.enable_kinematics_lowpass_filter = False
    if args.kinematics_lowpass_cutoff is not None:
        cfg.kinematics_lowpass_cutoff_hz = args.kinematics_lowpass_cutoff

    return cfg, args


def _run_plotter(cfg: SimulatorConfig) -> int:
    """Run plot/plotter.py with paths from the active simulator config."""
    resolved_paths = resolve_simulator_paths(cfg)
    plotter_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "plot",
        "plotter.py",
    )
    cmd = [
        sys.executable,
        plotter_path,
        "--model-bundle",
        cfg.model_bundle_dir,
        "--results-dir",
        cfg.output_dir,
        "--prefix",
        cfg.output_prefix,
        "--gait-side",
        cfg.plot_gait_side,
        "--reference",
        str(resolved_paths.kinematics_path),
    ]
    if cfg.model_file:
        cmd.extend(["--model", str(resolved_paths.model_path)])

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
    resolved_paths = resolve_simulator_paths(cfg)
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
        "--model-bundle",
        cfg.model_bundle_dir,
        "--results-dir",
        cfg.output_dir,
        "--prefix",
        cfg.output_prefix,
        "--reference",
        str(resolved_paths.kinematics_path),
        "--out",
        report_path,
    ]
    if cfg.model_file:
        cmd.extend(["--model", str(resolved_paths.model_path)])

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
        "model_bundle_dir", "model_file", "kinematics_file",
        "external_loads_xml", "reserve_actuators_xml",
        "t_start", "t_end", "dt",
        "output_dir", "output_prefix",
        "sea_forward_mode", "qp_solver",
        "sea_kp", "sea_kd",
        "enable_kinematics_lowpass_filter",
        "kinematics_lowpass_cutoff_hz",
        "kinematics_lowpass_order",
        "kinematics_resample_dt",
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
