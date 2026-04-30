"""
Run measured-GRF window tests before attempting any GRF reconstruction.

This runner reuses the OpenSim pipeline helpers but keeps each measured window
in its own output tree. It is intended to compare force-plate and treadmill
windows with the same SEA-preserving model preparation flow:

  1. IK on the requested analysis window.
  2. ExternalLoads XML for the requested measured force profile.
  3. ID diagnostic.
  4. RRA candidate sweep.
  5. SEA-preserving model promotion.
  6. CMC-like simulations on predefined subwindows.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import shutil
import subprocess
import sys
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from xml.etree import ElementTree as ET

from evaluate_cmc_like_candidates import CandidateResult, analyze_output
from run_opensim_sea_pipeline import (
    DEFAULT_GRF,
    DEFAULT_LEGACY_RRA_TASKS,
    DEFAULT_MARKER_SET,
    DEFAULT_OPENSIM_CMD,
    DEFAULT_PIPELINE_ROOT,
    DEFAULT_PLUGIN,
    DEFAULT_SUPPORT_DATA,
    DEFAULT_TRC,
    MAX_COM_SHIFT_M,
    OPEN_SIM_XML_VERSION,
    RraCandidate,
    ToolConfig,
    build_id_setup,
    build_ik_setup,
    build_rra_actuators,
    build_rra_setup,
    build_simulator_setup,
    child,
    compute_com_shift_m,
    copy_mass_center,
    create_paths,
    ensure_absolute,
    find_generated_file,
    fmt,
    force_norm,
    mkdirs,
    moment_norm,
    parse_ik_marker_error_files,
    parse_marker_names,
    parse_residuals_from_logs,
    require_dir,
    require_file,
    rra_selection_key,
    run_opensim_tool,
    summarize_rra_candidates,
    verify_sea_actuators,
    write_xml,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TEST_ROOT = DEFAULT_PIPELINE_ROOT / "window_tests"
DEFAULT_SCALED_MODEL = DEFAULT_PIPELINE_ROOT / "01_scaling" / "scaled_SEASEA.osim"

FORCE_PROFILES: dict[str, list[tuple[str, str, str, str, str]]] = {
    "fp12": [
        ("right_FP2", "calcn_r", "FP2_v", "FP2_p", "FP2_moment_"),
        ("left_FP1", "foot_l", "FP1_v", "FP1_p", "FP1_moment_"),
    ],
    "treadmill_lr": [
        (
            "right_Treadmill_L",
            "calcn_r",
            "Treadmill_L_v",
            "Treadmill_L_p",
            "Treadmill_L_moment_",
        ),
        (
            "left_Treadmill_R",
            "foot_l",
            "Treadmill_R_v",
            "Treadmill_R_p",
            "Treadmill_R_moment_",
        ),
    ],
}


@dataclass(frozen=True)
class WindowSpec:
    name: str
    force_profile: str
    analysis_window: tuple[float, float]
    rra_window: tuple[float, float]
    cmc_windows: list[tuple[str, float, float]]


@dataclass
class CmcRunSummary:
    window_name: str
    run_name: str
    t_start: float
    t_end: float
    output_dir: str
    returncode: int
    status: str
    elapsed_s: float
    warnings_total: int = 0
    so_feasibility_warnings: int = 0
    nonfinite_total: int = 0
    qp_fallback_total: int = 0
    tau_reserve_norm_mean: float = math.nan
    tau_reserve_norm_max: float = math.nan
    pelvis_reserve_torque_max_abs: float = math.nan
    joint_reserve_torque_max_abs: float = math.nan
    so_residual_norm_max: float = math.nan
    sea_knee_saturation_frames: int = 0
    sea_ankle_saturation_frames: int = 0


@dataclass
class WindowRunSummary:
    name: str
    force_profile: str
    analysis_start: float
    analysis_end: float
    rra_start: float
    rra_end: float
    output_root: str
    ik_rms_marker_error_max_m: float | None = None
    ik_marker_error_max_m: float | None = None
    id_returncode: int | None = None
    selected_rra: str | None = None
    selected_adjusted_body: str | None = None
    rra_force_norm: float | None = None
    rra_moment_norm: float | None = None
    rra_com_shift_m: float | None = None
    final_model: str | None = None
    simulator_setup: str | None = None
    cmc_runs: list[CmcRunSummary] = field(default_factory=list)


def default_window_specs() -> list[WindowSpec]:
    return [
        WindowSpec(
            name="fp12_15889_19839",
            force_profile="fp12",
            analysis_window=(15.889, 19.839),
            rra_window=(15.889, 19.839),
            cmc_windows=[
                ("full_15889_19839", 15.889, 19.839),
                ("early_15889_17500", 15.889, 17.500),
                ("middle_17500_19000", 17.500, 19.000),
                ("late_19000_19839", 19.000, 19.839),
            ],
        ),
        WindowSpec(
            name="treadmill_30065_34275",
            force_profile="treadmill_lr",
            analysis_window=(30.065, 34.275),
            rra_window=(30.065, 34.275),
            cmc_windows=[
                ("full_30065_34275", 30.065, 34.275),
                ("single_to_overlap_30065_31166", 30.065, 31.166),
                ("both_active_31166_34275", 31.166, 34.275),
            ],
        ),
    ]


def build_external_forces_profile(
    destination: Path,
    grf_file: Path,
    kinematics_file: Path | None,
    force_profile: str,
) -> Path:
    if force_profile not in FORCE_PROFILES:
        raise ValueError(f"Unknown force profile: {force_profile}")

    root = ET.Element("OpenSimDocument", Version=OPEN_SIM_XML_VERSION)
    loads = child(root, "ExternalLoads", name=f"externalloads_{force_profile}")
    objects = child(loads, "objects")

    for name, body, force_id, point_id, torque_id in FORCE_PROFILES[force_profile]:
        force = child(objects, "ExternalForce", name=name)
        child(force, "applied_to_body", body)
        child(force, "force_expressed_in_body", "ground")
        child(force, "point_expressed_in_body", "ground")
        child(force, "force_identifier", force_id)
        child(force, "point_identifier", point_id)
        child(force, "torque_identifier", torque_id)
    child(loads, "groups")
    child(loads, "datafile", str(grf_file))
    child(
        loads,
        "external_loads_model_kinematics_file",
        "" if kinematics_file is None else str(kinematics_file),
    )
    child(loads, "lowpass_cutoff_frequency_for_load_kinematics", "6")
    return write_xml(root, destination)


def storage_time_range(path: Path) -> tuple[float, float]:
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    try:
        header_index = lines.index("endheader") + 1
    except ValueError as exc:
        raise ValueError(f"Not an OpenSim storage file with endheader: {path}") from exc
    rows = [line for line in lines[header_index + 1 :] if line.strip()]
    if not rows:
        raise ValueError(f"No data rows found in storage file: {path}")
    first = float(rows[0].split()[0])
    last = float(rows[-1].split()[0])
    return first, last


def clamp_window_to_range(
    start: float,
    end: float,
    valid_start: float,
    valid_end: float,
) -> tuple[float, float]:
    clamped_start = max(start, valid_start)
    clamped_end = min(end, valid_end)
    if clamped_end <= clamped_start:
        raise ValueError(
            f"Window {start}-{end} does not overlap valid range {valid_start}-{valid_end}."
        )
    return clamped_start, clamped_end


def copy_support_inputs(
    cfg: ToolConfig,
    root: Path,
    scaled_model: Path,
) -> dict[str, Path]:
    paths = create_paths(root)
    mkdirs(paths)

    copied = {
        "scaled_model": paths.scaling_dir / scaled_model.name,
        "marker_set": paths.inputs_dir / cfg.marker_set.name,
        "trc": paths.inputs_dir / cfg.trc.name,
        "grf": paths.inputs_dir / cfg.grf.name,
    }
    shutil.copy2(scaled_model, copied["scaled_model"])
    shutil.copy2(cfg.marker_set, copied["marker_set"])
    shutil.copy2(cfg.trc, copied["trc"])
    shutil.copy2(cfg.grf, copied["grf"])

    support_dir = paths.inputs_dir / "SEASEA_support"
    support_dir.mkdir(parents=True, exist_ok=True)
    for source in cfg.support_data.iterdir():
        if source.is_file() and source.suffix.lower() == ".xml":
            shutil.copy2(source, support_dir / source.name)
    copied["cmc_actuators"] = support_dir / "CMC_Actuators.xml"
    copied["cmc_tasks"] = support_dir / "CMC_Tasks - modified Kp_Kv.xml"
    return copied


def run_cmc_window(
    setup_xml: Path,
    final_model: Path,
    window_name: str,
    run_name: str,
    t_start: float,
    t_end: float,
    output_dir: Path,
    dry_run: bool,
    plot: bool,
    timeout_s: float | None,
) -> CmcRunSummary:
    if output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    stdout_log = output_dir / "stdout.log"
    cmd = [
        sys.executable,
        str(REPO_ROOT / "main.py"),
        "--setup",
        str(setup_xml),
        "--t-start",
        fmt(t_start),
        "--t-end",
        fmt(t_end),
        "--solver",
        "osqp",
        "--sea-feasibility-scaling",
        "--output-dir",
        str(output_dir),
        "--log",
    ]
    if plot:
        cmd.append("--plot")

    started = time.perf_counter()
    if dry_run:
        stdout_log.write_text("DRY RUN: " + " ".join(cmd) + "\n", encoding="utf-8")
        elapsed = 0.0
        returncode = 0
        timed_out = False
    else:
        completed_stdout, returncode, timed_out = run_command_with_timeout(
            cmd,
            REPO_ROOT,
            timeout_s,
        )
        elapsed = time.perf_counter() - started
        stdout_log.write_text(completed_stdout, encoding="utf-8", errors="replace")

    result = CandidateResult(
        name=run_name,
        model=str(final_model),
        output_dir=str(output_dir),
        returncode=returncode,
        status="dry_run" if dry_run else ("timeout" if timed_out else "missing_status"),
        elapsed_s=elapsed,
    )
    if not dry_run:
        result = analyze_output(result)
        if timed_out and result.status == "missing_status":
            result.status = "timeout"

    return CmcRunSummary(
        window_name=window_name,
        run_name=run_name,
        t_start=t_start,
        t_end=t_end,
        output_dir=str(output_dir),
        returncode=result.returncode,
        status=result.status,
        elapsed_s=result.elapsed_s,
        warnings_total=result.warnings_total,
        so_feasibility_warnings=result.so_feasibility_warnings,
        nonfinite_total=result.nonfinite_total,
        qp_fallback_total=result.qp_fallback_total,
        tau_reserve_norm_mean=result.tau_reserve_norm_mean,
        tau_reserve_norm_max=result.tau_reserve_norm_max,
        pelvis_reserve_torque_max_abs=result.pelvis_reserve_torque_max_abs,
        joint_reserve_torque_max_abs=result.joint_reserve_torque_max_abs,
        so_residual_norm_max=result.so_residual_norm_max,
        sea_knee_saturation_frames=result.sea_knee_saturation_frames,
        sea_ankle_saturation_frames=result.sea_ankle_saturation_frames,
    )


def terminate_process_tree(proc: subprocess.Popen[str]) -> None:
    if proc.poll() is not None:
        return
    if sys.platform.startswith("win"):
        subprocess.run(
            ["taskkill", "/PID", str(proc.pid), "/T", "/F"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )
    else:
        proc.kill()


def run_command_with_timeout(
    cmd: list[str],
    cwd: Path,
    timeout_s: float | None,
) -> tuple[str, int, bool]:
    creationflags = 0
    if sys.platform.startswith("win"):
        creationflags = getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)
    proc = subprocess.Popen(
        cmd,
        cwd=cwd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        creationflags=creationflags,
    )
    try:
        stdout, _ = proc.communicate(timeout=timeout_s if timeout_s and timeout_s > 0 else None)
        return stdout or "", proc.returncode, False
    except subprocess.TimeoutExpired as exc:
        terminate_process_tree(proc)
        stdout, _ = proc.communicate()
        partial = exc.stdout or ""
        if isinstance(partial, bytes):
            partial = partial.decode(errors="replace")
        message = (
            "\n[TIMEOUT] CMC-like run exceeded "
            f"{timeout_s:.1f} s and was terminated.\n"
        )
        return (partial + (stdout or "") + message), -9, True


def run_rra_candidate_window_worker(
    args: tuple[
        ToolConfig,
        Path,
        Path,
        Path,
        str,
        Path,
        Path,
        Path,
        str | None,
        float,
        float,
    ],
) -> RraCandidate:
    (
        cfg,
        scaled_model,
        actuators,
        tasks,
        task_profile,
        external_forces,
        desired_kinematics,
        workers_dir,
        adjusted_body,
        rra_start,
        rra_end,
    ) = args
    body_name = adjusted_body or "no_com"
    name = f"{task_profile}_{body_name}"
    worker_dir = workers_dir / f"rra_{name}"
    results_dir = worker_dir / "results"
    output_model = worker_dir / f"Adjusted_newmarkers_{name}.osim"
    setup_xml = worker_dir / "RRA_setup.xml"
    candidate = RraCandidate(
        name=name,
        task_profile=task_profile,
        adjusted_body=adjusted_body,
        worker_dir=worker_dir,
        setup_xml=setup_xml,
        output_model=output_model,
    )
    results_dir.mkdir(parents=True, exist_ok=True)

    local_model = worker_dir / "scaled_SEASEA.osim"
    local_actuators = worker_dir / "RRA_Actuators.xml"
    local_tasks = worker_dir / "RRA_Tasks.xml"
    local_external_forces = worker_dir / "ExternalForces.xml"
    local_desired_kinematics = worker_dir / "IK_rra_window.mot"
    shutil.copy2(scaled_model, local_model)
    shutil.copy2(actuators, local_actuators)
    shutil.copy2(tasks, local_tasks)
    shutil.copy2(external_forces, local_external_forces)
    shutil.copy2(desired_kinematics, local_desired_kinematics)

    build_rra_setup(
        setup_xml,
        Path(local_model.name),
        Path(local_actuators.name),
        Path(local_tasks.name),
        Path(local_external_forces.name),
        Path(local_desired_kinematics.name),
        Path("results"),
        Path(output_model.name),
        adjusted_body,
        rra_start,
        rra_end,
    )
    result = run_opensim_tool(setup_xml, worker_dir, cfg, f"rra_{name}")
    candidate.command_returncode = result.returncode
    if result.returncode != 0:
        candidate.status = "failed"
        candidate.notes.append(f"RRATool failed with code {result.returncode}")
        return candidate

    residuals = parse_residuals_from_logs(worker_dir) | parse_residuals_from_logs(results_dir)
    candidate.residuals = residuals
    candidate.force_norm = force_norm(residuals)
    candidate.moment_norm = moment_norm(residuals)
    candidate.com_shift_m = compute_com_shift_m(local_model, output_model, adjusted_body)
    candidate.status = "ok"
    if not residuals:
        candidate.notes.append("Average residuals unavailable; selection will use fallback ordering.")
    if adjusted_body and not output_model.is_file():
        candidate.status = "failed"
        candidate.notes.append("RRA did not produce adjusted model.")
    return candidate


def run_window_test(
    spec: WindowSpec,
    cfg: ToolConfig,
    scaled_model: Path,
    test_root: Path,
    max_workers: int,
    skip_cmc: bool,
    plot: bool,
    cmc_timeout_s: float | None,
) -> WindowRunSummary:
    root = test_root / spec.name
    paths = create_paths(root)
    copied = copy_support_inputs(cfg, root, scaled_model)
    marker_names = parse_marker_names(copied["marker_set"])

    summary = WindowRunSummary(
        name=spec.name,
        force_profile=spec.force_profile,
        analysis_start=spec.analysis_window[0],
        analysis_end=spec.analysis_window[1],
        rra_start=spec.rra_window[0],
        rra_end=spec.rra_window[1],
        output_root=str(root),
    )

    print(f"[window:{spec.name}] Running IK {spec.analysis_window[0]}-{spec.analysis_window[1]}...")
    ik_results = paths.ik_dir / "analysis_window"
    ik_motion_name = f"IK_{spec.name}.mot"
    ik_setup = paths.ik_dir / f"IK_{spec.name}_setup.xml"
    build_ik_setup(
        ik_setup,
        copied["scaled_model"],
        copied["trc"],
        marker_names,
        ik_results,
        ik_motion_name,
        spec.analysis_window[0],
        spec.analysis_window[1],
    )
    ik_result = run_opensim_tool(ik_setup, paths.ik_dir, cfg, f"ik_{spec.name}")
    if ik_result.returncode != 0:
        raise RuntimeError(f"IK failed for {spec.name}; inspect {paths.ik_dir}")
    if cfg.dry_run:
        kinematics = paths.ik_dir / ik_motion_name
    else:
        kinematics = find_generated_file(paths.ik_dir, ik_results, ik_motion_name)
        ik_destination = paths.ik_dir / kinematics.name
        if kinematics.resolve() != ik_destination.resolve():
            shutil.copy2(kinematics, ik_destination)
        rms, max_error = parse_ik_marker_error_files(ik_results)
        summary.ik_rms_marker_error_max_m = rms
        summary.ik_marker_error_max_m = max_error
    kinematics_start, kinematics_end = (
        spec.analysis_window if cfg.dry_run else storage_time_range(kinematics)
    )

    print(f"[window:{spec.name}] Running ID diagnostic with {spec.force_profile}...")
    id_external = build_external_forces_profile(
        paths.id_dir / "ExternalForces.xml",
        copied["grf"],
        kinematics,
        spec.force_profile,
    )
    id_setup = paths.id_dir / "ID_setup.xml"
    build_id_setup(
        id_setup,
        copied["scaled_model"],
        id_external,
        kinematics,
        paths.id_dir,
        spec.rra_window[0],
        spec.rra_window[1],
    )
    id_result = run_opensim_tool(id_setup, paths.id_dir, cfg, "id")
    summary.id_returncode = id_result.returncode
    if id_result.returncode != 0:
        print(f"[window:{spec.name}] WARNING: ID failed; continuing to RRA diagnostics.")
    if cfg.dry_run:
        write_window_report(summary, paths.pipeline_root / "window_report.md")
        print(
            f"[window:{spec.name}] Dry run prepared IK/ID scaffolding. "
            f"Report: {paths.pipeline_root / 'window_report.md'}"
        )
        return summary

    print(f"[window:{spec.name}] Running RRA candidates...")
    rra_actuators = build_rra_actuators(
        paths.rra_dir / "RRA_Actuators.xml",
        copied["scaled_model"],
        copied["cmc_actuators"],
    )
    rra_tasks = paths.rra_dir / "RRA_Tasks_repo_cmc.xml"
    shutil.copy2(copied["cmc_tasks"], rra_tasks)
    task_profiles: list[tuple[str, Path]] = [("repo_cmc_tasks", rra_tasks)]
    if cfg.legacy_rra_tasks is not None:
        legacy_tasks = paths.rra_dir / "RRA_Tasks_legacy.xml"
        shutil.copy2(cfg.legacy_rra_tasks, legacy_tasks)
        task_profiles.append(("legacy_rra_tasks", legacy_tasks))

    rra_external = build_external_forces_profile(
        paths.rra_dir / "ExternalForces.xml",
        copied["grf"],
        kinematics,
        spec.force_profile,
    )
    rra_kinematics = paths.rra_dir / kinematics.name
    shutil.copy2(kinematics, rra_kinematics)

    rra_args = []
    for task_profile, task_file in task_profiles:
        for body in ("pelvis", "torso", None):
            rra_args.append(
                (
                    cfg,
                    copied["scaled_model"],
                    rra_actuators,
                    task_file,
                    task_profile,
                    rra_external,
                    rra_kinematics,
                    paths.workers_dir,
                    body,
                    spec.rra_window[0],
                    spec.rra_window[1],
                )
            )
    with subprocess_context(max_workers) as executor:
        rra_candidates = list(executor.map(run_rra_candidate_window_worker, rra_args))

    summarize_rra_candidates(rra_candidates, paths.rra_dir / "rra_candidates.csv")
    ok_rra_candidates = [candidate for candidate in rra_candidates if candidate.status == "ok"]
    selected_rra = min(ok_rra_candidates, key=rra_selection_key) if ok_rra_candidates else None
    if selected_rra is None:
        raise RuntimeError(f"No RRA candidate completed successfully for {spec.name}.")
    if selected_rra.com_shift_m is None or selected_rra.com_shift_m > MAX_COM_SHIFT_M + 1e-9:
        print(
            f"[window:{spec.name}] WARNING: selected RRA exceeds COM-shift criterion; "
            "promoting diagnostic output only."
        )

    summary.selected_rra = selected_rra.name
    summary.selected_adjusted_body = selected_rra.adjusted_body
    summary.rra_force_norm = selected_rra.force_norm
    summary.rra_moment_norm = selected_rra.moment_norm
    summary.rra_com_shift_m = selected_rra.com_shift_m

    final_model = paths.final_dir / f"Adjusted_newmarkers_{spec.name}_ready.osim"
    source_adjusted = (
        selected_rra.output_model if selected_rra.adjusted_body else copied["scaled_model"]
    )
    copy_mass_center(copied["scaled_model"], source_adjusted, final_model, selected_rra.adjusted_body)
    if not verify_sea_actuators(final_model):
        raise RuntimeError(f"Final model does not contain both SEA actuators: {final_model}")
    summary.final_model = str(final_model)

    final_data_dir = paths.final_dir / "data"
    final_data_dir.mkdir(parents=True, exist_ok=True)
    final_kinematics = final_data_dir / "IK_analysis_window.mot"
    final_external = final_data_dir / "ExternalForces.xml"
    final_actuators = final_data_dir / "CMC_Actuators.xml"
    shutil.copy2(kinematics, final_kinematics)
    shutil.copy2(rra_external, final_external)
    shutil.copy2(copied["cmc_actuators"], final_actuators)
    shutil.copy2(copied["cmc_tasks"], final_data_dir / "CMC_Tasks - modified Kp_Kv.xml")

    simulator_setup = paths.final_dir / f"Adjusted_newmarkers_{spec.name}_setup.xml"
    build_simulator_setup(
        simulator_setup,
        final_model,
        final_kinematics,
        final_external,
        final_actuators,
        kinematics_start,
        kinematics_end,
    )
    summary.simulator_setup = str(simulator_setup)

    if not skip_cmc:
        for run_name, start, end in spec.cmc_windows:
            start, end = clamp_window_to_range(start, end, kinematics_start, kinematics_end)
            print(f"[window:{spec.name}] Running CMC-like {run_name}...")
            output_dir = paths.pipeline_root / "06_cmc_like_evaluation" / run_name
            summary.cmc_runs.append(
                run_cmc_window(
                    simulator_setup,
                    final_model,
                    spec.name,
                    run_name,
                    start,
                    end,
                    output_dir,
                    cfg.dry_run,
                    plot,
                    cmc_timeout_s,
                )
            )

    write_window_report(summary, paths.pipeline_root / "window_report.md")
    print(f"[window:{spec.name}] Done. Report: {paths.pipeline_root / 'window_report.md'}")
    return summary


def subprocess_context(max_workers: int):
    # A tiny wrapper keeps the callsite readable while preserving process
    # isolation for OpenSim-heavy RRA candidates.
    from concurrent.futures import ProcessPoolExecutor

    return ProcessPoolExecutor(max_workers=max(1, max_workers))


def metric(value: float | int | None) -> str:
    if value is None:
        return ""
    if isinstance(value, float) and not math.isfinite(value):
        return ""
    if isinstance(value, float):
        return f"{value:.6g}"
    return str(value)


def write_window_report(summary: WindowRunSummary, destination: Path) -> None:
    lines = [
        f"# Measured GRF Window Test - {summary.name}",
        "",
        f"Force profile: `{summary.force_profile}`",
        f"Analysis window: `{summary.analysis_start:.3f} - {summary.analysis_end:.3f} s`",
        f"RRA window: `{summary.rra_start:.3f} - {summary.rra_end:.3f} s`",
        "",
        "## IK",
        "",
        f"- RMS marker error max: `{metric(summary.ik_rms_marker_error_max_m)} m`",
        f"- Marker error max: `{metric(summary.ik_marker_error_max_m)} m`",
        "",
        "## RRA",
        "",
        f"- ID return code: `{summary.id_returncode}`",
        f"- Selected RRA: `{summary.selected_rra}`",
        f"- Adjusted body: `{summary.selected_adjusted_body or ''}`",
        f"- Force norm: `{metric(summary.rra_force_norm)}`",
        f"- Moment norm: `{metric(summary.rra_moment_norm)}`",
        f"- COM shift max component: `{metric(summary.rra_com_shift_m)} m`",
        f"- Final model: `{summary.final_model}`",
        f"- Simulator setup: `{summary.simulator_setup}`",
        "",
        "## CMC-like Runs",
        "",
        "| run | status | warnings | SO warn | non-finite | reserve mean | reserve max | pelvis max | joint max | SEA sat | elapsed s |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for run in summary.cmc_runs:
        sea_sat = run.sea_knee_saturation_frames + run.sea_ankle_saturation_frames
        lines.append(
            "| "
            + " | ".join(
                [
                    f"`{run.run_name}`",
                    run.status,
                    str(run.warnings_total),
                    str(run.so_feasibility_warnings),
                    str(run.nonfinite_total),
                    metric(run.tau_reserve_norm_mean),
                    metric(run.tau_reserve_norm_max),
                    metric(run.pelvis_reserve_torque_max_abs),
                    metric(run.joint_reserve_torque_max_abs),
                    str(sea_sat),
                    metric(run.elapsed_s),
                ]
            )
            + " |"
        )
    destination.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_combined_summary(summaries: list[WindowRunSummary], destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, str]] = []
    for summary in summaries:
        if not summary.cmc_runs:
            rows.append(
                {
                    "window": summary.name,
                    "run": "",
                    "force_profile": summary.force_profile,
                    "status": "",
                    "ik_rms_marker_error_max_m": metric(summary.ik_rms_marker_error_max_m),
                    "rra_force_norm": metric(summary.rra_force_norm),
                    "rra_moment_norm": metric(summary.rra_moment_norm),
                    "rra_com_shift_m": metric(summary.rra_com_shift_m),
                    "tau_reserve_norm_mean": "",
                    "tau_reserve_norm_max": "",
                    "pelvis_reserve_torque_max_abs": "",
                    "sea_saturation_frames": "",
                    "output_dir": summary.output_root,
                }
            )
            continue
        for run in summary.cmc_runs:
            rows.append(
                {
                    "window": summary.name,
                    "run": run.run_name,
                    "force_profile": summary.force_profile,
                    "status": run.status,
                    "ik_rms_marker_error_max_m": metric(summary.ik_rms_marker_error_max_m),
                    "rra_force_norm": metric(summary.rra_force_norm),
                    "rra_moment_norm": metric(summary.rra_moment_norm),
                    "rra_com_shift_m": metric(summary.rra_com_shift_m),
                    "tau_reserve_norm_mean": metric(run.tau_reserve_norm_mean),
                    "tau_reserve_norm_max": metric(run.tau_reserve_norm_max),
                    "pelvis_reserve_torque_max_abs": metric(run.pelvis_reserve_torque_max_abs),
                    "sea_saturation_frames": str(
                        run.sea_knee_saturation_frames + run.sea_ankle_saturation_frames
                    ),
                    "output_dir": run.output_dir,
                }
            )

    fields = list(rows[0].keys()) if rows else []
    with destination.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    md = destination.with_suffix(".md")
    lines = [
        "# Measured GRF Window Test Summary",
        "",
        "| window | run | status | IK RMS max | RRA force | RRA moment | COM shift | reserve mean | reserve max | pelvis max | SEA sat |",
        "| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in rows:
        lines.append(
            "| "
            + " | ".join(
                [
                    f"`{row['window']}`",
                    f"`{row['run']}`" if row["run"] else "",
                    row["status"],
                    row["ik_rms_marker_error_max_m"],
                    row["rra_force_norm"],
                    row["rra_moment_norm"],
                    row["rra_com_shift_m"],
                    row["tau_reserve_norm_mean"],
                    row["tau_reserve_norm_max"],
                    row["pelvis_reserve_torque_max_abs"],
                    row["sea_saturation_frames"],
                ]
            )
            + " |"
        )
    lines.extend(
        [
            "",
            "Notes:",
            "- Lower reserve and fewer SEA saturation frames are better.",
            "- Visual walking quality still needs GUI inspection for each generated setup.",
        ]
    )
    md.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_window(values: list[float] | None, label: str) -> tuple[float, float] | None:
    if values is None:
        return None
    start, end = values
    if end <= start:
        raise ValueError(f"{label} end must be greater than start.")
    return (start, end)


def slug_window(start: float, end: float) -> str:
    def clean(value: float) -> str:
        return re.sub(r"[^0-9]", "", f"{value:.3f}")

    return f"{clean(start)}_{clean(end)}"


def custom_specs_from_args(args: argparse.Namespace) -> list[WindowSpec] | None:
    analysis = parse_window(args.analysis_window, "analysis window")
    rra = parse_window(args.rra_window, "RRA window") if args.rra_window else analysis
    if analysis is None and args.force_profile is None and args.name is None and not args.cmc_window:
        return None
    if analysis is None or rra is None or args.force_profile is None:
        raise ValueError(
            "Custom mode requires --analysis-window START END and --force-profile."
        )
    name = args.name or f"{args.force_profile}_{slug_window(*analysis)}"
    cmc_windows = [
        (f"custom_{slug_window(start, end)}", start, end)
        for start, end in (parse_window(values, "CMC window") for values in args.cmc_window)
    ]
    if not cmc_windows:
        cmc_windows = [(f"full_{slug_window(*analysis)}", analysis[0], analysis[1])]
    return [
        WindowSpec(
            name=name,
            force_profile=args.force_profile,
            analysis_window=analysis,
            rra_window=rra,
            cmc_windows=cmc_windows,
        )
    ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--test-root", type=Path, default=DEFAULT_TEST_ROOT)
    parser.add_argument("--scaled-model", type=Path, default=DEFAULT_SCALED_MODEL)
    parser.add_argument("--marker-set", type=Path, default=DEFAULT_MARKER_SET)
    parser.add_argument("--trc", type=Path, default=DEFAULT_TRC)
    parser.add_argument("--grf", type=Path, default=DEFAULT_GRF)
    parser.add_argument("--support-data", type=Path, default=DEFAULT_SUPPORT_DATA)
    parser.add_argument("--opensim-cmd", type=Path, default=DEFAULT_OPENSIM_CMD)
    parser.add_argument("--plugin", type=Path, default=DEFAULT_PLUGIN)
    parser.add_argument("--legacy-rra-tasks", type=Path, default=DEFAULT_LEGACY_RRA_TASKS)
    parser.add_argument(
        "--only",
        action="append",
        choices=["fp12_15889_19839", "treadmill_30065_34275"],
        default=[],
        help="Run only one predefined measured-window test. Can be passed twice.",
    )
    parser.add_argument("--name", help="Custom test name.")
    parser.add_argument("--analysis-window", nargs=2, type=float, metavar=("START", "END"))
    parser.add_argument("--rra-window", nargs=2, type=float, metavar=("START", "END"))
    parser.add_argument("--force-profile", choices=sorted(FORCE_PROFILES))
    parser.add_argument(
        "--cmc-window",
        nargs=2,
        type=float,
        action="append",
        default=[],
        metavar=("START", "END"),
        help="Custom CMC-like test window. Can be passed multiple times.",
    )
    parser.add_argument("--max-workers", type=int, default=4)
    parser.add_argument(
        "--cmc-timeout-s",
        type=float,
        default=900.0,
        help="Terminate a single CMC-like window after this many seconds. Use 0 to disable.",
    )
    parser.add_argument("--skip-cmc", action="store_true")
    parser.add_argument("--plot", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    custom_specs = custom_specs_from_args(args)
    specs = custom_specs or default_window_specs()
    if args.only and custom_specs is not None:
        raise ValueError("--only cannot be combined with custom window arguments.")
    if args.only:
        wanted = set(args.only)
        specs = [spec for spec in specs if spec.name in wanted]
    if not specs:
        raise RuntimeError("No window tests selected.")

    cfg = ToolConfig(
        opensim_cmd=require_file(args.opensim_cmd, "opensim-cmd"),
        plugin=require_file(args.plugin, "SEA plugin"),
        model=require_file(args.scaled_model, "Scaled model"),
        marker_set=require_file(args.marker_set, "MarkerSet"),
        trc=require_file(args.trc, "TRC"),
        grf=require_file(args.grf, "GRF MOT"),
        support_data=require_dir(args.support_data, "SEASEA support data"),
        legacy_rra_tasks=ensure_absolute(args.legacy_rra_tasks)
        if args.legacy_rra_tasks and ensure_absolute(args.legacy_rra_tasks).is_file()
        else None,
        dry_run=args.dry_run,
    )
    scaled_model = require_file(args.scaled_model, "Scaled model")
    test_root = ensure_absolute(args.test_root)
    test_root.mkdir(parents=True, exist_ok=True)

    summaries = [
        run_window_test(
            spec,
            cfg,
            scaled_model,
            test_root,
            args.max_workers,
            args.skip_cmc,
            args.plot,
            None if args.cmc_timeout_s <= 0 else args.cmc_timeout_s,
        )
        for spec in specs
    ]
    write_combined_summary(summaries, test_root / "measured_window_summary.csv")
    print(f"[summary] {test_root / 'measured_window_summary.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
