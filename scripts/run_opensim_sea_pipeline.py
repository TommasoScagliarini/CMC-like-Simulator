"""
Run the SEA OpenSim preparation pipeline in an isolated model_pipeline_dir.

The runner intentionally keeps OpenSim-native steps in separate processes and
folders. This avoids shared Simbody/OpenSim/plugin state and makes failed
candidates easy to inspect.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import csv
import json
import math
import os
import re
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Iterable
from xml.etree import ElementTree as ET


REPO_ROOT = Path(__file__).resolve().parents[1]

DEFAULT_PIPELINE_ROOT = (
    Path.home() / "Desktop" / "Opensim OMNIBUS" / "model_pipeline_dir"
)
DEFAULT_MODEL = (
    Path.home()
    / "Downloads"
    / "Archive"
    / "TODO"
    / "gait2176_L_TFP_ORIGINALE_newmarkers - SEASEA.osim"
)
DEFAULT_MARKER_SET = Path.home() / "Downloads" / "Archive" / "markers_remapped.xml"
DEFAULT_TRC = Path.home() / "Downloads" / "Archive" / "ground_walking_cw.trc"
DEFAULT_GRF = Path.home() / "Downloads" / "Archive" / "ground_walking_cw_fp.mot"
DEFAULT_SUPPORT_DATA = REPO_ROOT / "models" / "SEASEA - whealthy data" / "data"
DEFAULT_OPENSIM_CMD = Path("C:/OpenSim-mCMC/bin/opensim-cmd.exe")
DEFAULT_PLUGIN = REPO_ROOT / "plugins" / "SEA_Plugin_BlackBox_mCMC_impedence_ff.dll"
DEFAULT_LEGACY_RRA_TASKS = (
    Path.home()
    / "Desktop"
    / "Opensim OMNIBUS"
    / "3D_Model_Leg_and_Prosthesis_Completo_21_76"
    / "RRA"
    / "RRA_Tasks.xml"
)

WORKER_COUNT = 4
FULL_TIME_RANGE = (17.0, 23.0)
RRA_TIME_RANGE = (18.965, 19.839)
SCALE_WINDOW_SECONDS = 0.05
SCALE_WINDOW_STRIDE_SECONDS = 0.25
SCALE_CANDIDATE_COUNT = 4
RMS_TARGET_M = 0.02
MAX_COM_SHIFT_M = 0.10

OPEN_SIM_XML_VERSION = "40000"


@dataclass(frozen=True)
class Paths:
    pipeline_root: Path
    inputs_dir: Path
    scaling_dir: Path
    ik_dir: Path
    id_dir: Path
    rra_dir: Path
    final_dir: Path
    workers_dir: Path
    logs_dir: Path


@dataclass(frozen=True)
class ToolConfig:
    opensim_cmd: Path
    plugin: Path
    model: Path
    marker_set: Path
    trc: Path
    grf: Path
    support_data: Path
    legacy_rra_tasks: Path | None = None
    dry_run: bool = False


@dataclass
class CommandResult:
    returncode: int
    stdout: str
    elapsed_s: float


@dataclass
class ScaleCandidate:
    index: int
    mode: str
    start: float
    end: float
    stability_mm_per_frame: float
    worker_dir: Path
    setup_xml: Path
    output_model: Path
    output_scale_file: Path
    output_motion_file: Path
    status: str = "pending"
    rms_marker_error_m: float | None = None
    max_marker_error_m: float | None = None
    max_scale_delta: float | None = None
    command_returncode: int | None = None
    notes: list[str] = field(default_factory=list)


@dataclass
class RraCandidate:
    name: str
    task_profile: str
    adjusted_body: str | None
    worker_dir: Path
    setup_xml: Path
    output_model: Path
    status: str = "pending"
    residuals: dict[str, float] = field(default_factory=dict)
    force_norm: float | None = None
    moment_norm: float | None = None
    com_shift_m: float | None = None
    command_returncode: int | None = None
    notes: list[str] = field(default_factory=list)


def fmt(value: float) -> str:
    return f"{value:.15g}"


def ensure_absolute(path: Path) -> Path:
    return path.expanduser().resolve()


def relpath(path: Path, base: Path) -> str:
    try:
        return path.resolve().relative_to(base.resolve()).as_posix()
    except ValueError:
        return str(path.resolve())


def require_file(path: Path, label: str) -> Path:
    resolved = ensure_absolute(path)
    if not resolved.is_file():
        raise FileNotFoundError(f"{label} not found: {resolved}")
    return resolved


def require_dir(path: Path, label: str) -> Path:
    resolved = ensure_absolute(path)
    if not resolved.is_dir():
        raise FileNotFoundError(f"{label} not found: {resolved}")
    return resolved


def create_paths(root: Path) -> Paths:
    root = ensure_absolute(root)
    return Paths(
        pipeline_root=root,
        inputs_dir=root / "00_inputs",
        scaling_dir=root / "01_scaling",
        ik_dir=root / "02_ik",
        id_dir=root / "03_id",
        rra_dir=root / "04_rra",
        final_dir=root / "05_cmc_like_ready",
        workers_dir=root / "workers",
        logs_dir=root / "logs",
    )


def mkdirs(paths: Paths) -> None:
    for directory in (
        paths.pipeline_root,
        paths.inputs_dir,
        paths.scaling_dir,
        paths.ik_dir,
        paths.id_dir,
        paths.rra_dir,
        paths.final_dir,
        paths.workers_dir,
        paths.logs_dir,
    ):
        directory.mkdir(parents=True, exist_ok=True)


def copy_inputs(cfg: ToolConfig, paths: Paths) -> dict[str, Path]:
    copied = {
        "model": paths.inputs_dir / cfg.model.name,
        "marker_set": paths.inputs_dir / cfg.marker_set.name,
        "trc": paths.inputs_dir / cfg.trc.name,
        "grf": paths.inputs_dir / cfg.grf.name,
    }
    for source_name, destination in copied.items():
        source = getattr(cfg, source_name)
        shutil.copy2(source, destination)

    support_dir = paths.inputs_dir / "SEASEA_support"
    support_dir.mkdir(parents=True, exist_ok=True)
    for source in cfg.support_data.iterdir():
        if source.is_file() and source.suffix.lower() == ".xml":
            shutil.copy2(source, support_dir / source.name)

    copied["support_dir"] = support_dir
    copied["cmc_actuators"] = support_dir / "CMC_Actuators.xml"
    copied["cmc_tasks"] = support_dir / "CMC_Tasks - modified Kp_Kv.xml"
    return copied


def write_xml(root: ET.Element, destination: Path) -> Path:
    destination.parent.mkdir(parents=True, exist_ok=True)
    tree = ET.ElementTree(root)
    ET.indent(tree, space="\t")
    tree.write(destination, encoding="utf-8", xml_declaration=True)
    return destination


def child(parent: ET.Element, tag: str, text: str | None = None, **attrib: str) -> ET.Element:
    node = ET.SubElement(parent, tag, attrib)
    if text is not None:
        node.text = text
    return node


def parse_marker_names(marker_set_xml: Path) -> list[str]:
    root = ET.fromstring(marker_set_xml.read_text(encoding="utf-8", errors="replace").lstrip())
    markers = []
    for node in root.iter():
        if node.tag == "Marker" and "name" in node.attrib:
            markers.append(node.attrib["name"])
    if not markers:
        raise ValueError(f"No markers found in {marker_set_xml}")
    return markers


def parse_trc(trc_path: Path) -> tuple[list[str], list[float], list[list[float]]]:
    lines = trc_path.read_text(encoding="utf-8", errors="replace").splitlines()
    header_idx = None
    for idx, line in enumerate(lines):
        if line.startswith("Frame#"):
            header_idx = idx
            break
    if header_idx is None:
        raise ValueError(f"Unable to find TRC Frame# header in {trc_path}")

    header = lines[header_idx].split("\t")
    marker_names = [name for name in header[2:] if name.strip()]
    if not marker_names:
        raise ValueError(f"No marker names found in TRC header: {trc_path}")

    times: list[float] = []
    rows: list[list[float]] = []
    expected_values = 2 + 3 * len(marker_names)
    for raw_line in lines[header_idx + 2 :]:
        if not raw_line.strip():
            continue
        parts = raw_line.split("\t")
        if len(parts) < expected_values:
            continue
        try:
            times.append(float(parts[1]))
            rows.append([float(value) for value in parts[2 : expected_values]])
        except ValueError:
            continue

    if not times:
        raise ValueError(f"No numeric TRC frames parsed from {trc_path}")
    return marker_names, times, rows


def stability_for_window(
    times: list[float],
    rows: list[list[float]],
    start: float,
    end: float,
) -> float:
    indices = [idx for idx, t in enumerate(times) if start <= t <= end]
    if len(indices) < 2:
        return math.inf
    total = 0.0
    count = 0
    for previous, current in zip(indices, indices[1:]):
        prev_row = rows[previous]
        curr_row = rows[current]
        marker_count = len(prev_row) // 3
        for marker_idx in range(marker_count):
            j = marker_idx * 3
            dx = curr_row[j] - prev_row[j]
            dy = curr_row[j + 1] - prev_row[j + 1]
            dz = curr_row[j + 2] - prev_row[j + 2]
            if all(math.isfinite(v) for v in (dx, dy, dz)):
                total += math.sqrt(dx * dx + dy * dy + dz * dz)
                count += 1
    return total / count if count else math.inf


def select_scaling_windows(trc_path: Path) -> list[tuple[int, float, float, float]]:
    _, times, rows = parse_trc(trc_path)
    start, stop = FULL_TIME_RANGE
    candidates: list[tuple[float, float, float]] = []
    current = start
    while current + SCALE_WINDOW_SECONDS <= stop + 1e-9:
        end = current + SCALE_WINDOW_SECONDS
        score = stability_for_window(times, rows, current, end)
        if math.isfinite(score):
            candidates.append((score, current, end))
        current += SCALE_WINDOW_STRIDE_SECONDS
    if not candidates:
        raise ValueError("No valid scaling windows found in the TRC.")
    candidates.sort(key=lambda item: item[0])
    return [
        (idx + 1, round(start, 6), round(end, 6), score)
        for idx, (score, start, end) in enumerate(candidates[:SCALE_CANDIDATE_COUNT])
    ]


def marker_tasks_xml(parent: ET.Element, marker_names: Iterable[str], weight: float = 1.0) -> None:
    task_set = child(parent, "IKTaskSet")
    objects = child(task_set, "objects")
    for name in marker_names:
        task = child(objects, "IKMarkerTask", name=name)
        child(task, "apply", "true")
        child(task, "weight", fmt(weight))
    child(task_set, "groups")


def measurement_set_xml(parent: ET.Element) -> None:
    measurement_set = child(parent, "MeasurementSet")
    objects = child(measurement_set, "objects")

    measurements = [
        ("pelvis_width", ["R_ASIS", "L_ASIS"], ["pelvis"], "X Z"),
        ("pelvis_depth", ["R_ASIS", "R_PSIS"], ["pelvis"], "X Y Z"),
        ("right_thigh", ["R_Thigh_Upper", "R_Knee_Lat"], ["femur_r"], "X Y Z"),
        ("right_shank", ["R_Knee_Lat", "R_Ankle_Lat"], ["tibia_r"], "X Y Z"),
        ("right_foot_length", ["R_Heel", "R_Toe_Tip"], ["calcn_r", "toes_r"], "X Z"),
        ("right_foot_width", ["R_Toe_Lat", "R_Toe_Med"], ["calcn_r", "toes_r"], "Z"),
        ("left_thigh", ["L_Thigh_Upper", "L_Knee_Lat"], ["transfemur"], "X Y Z"),
        ("left_shank", ["L_Knee_Lat", "L_Ankle_Lat"], ["osseo_pylon", "tibia_pylon"], "X Y Z"),
        ("left_foot_length", ["L_Heel", "L_Toe_Tip"], ["foot_l"], "X Z"),
        ("left_foot_width", ["L_Toe_Lat", "L_Toe_Med"], ["foot_l"], "Z"),
    ]

    for name, marker_pair, body_names, axes in measurements:
        measurement = child(objects, "Measurement", name=name)
        child(measurement, "apply", "true")
        pair_set = child(measurement, "MarkerPairSet")
        pair_objects = child(pair_set, "objects")
        marker_pair_node = child(pair_objects, "MarkerPair", name=f"{name}_pair")
        child(marker_pair_node, "markers", " ".join(marker_pair))
        child(pair_set, "groups")

        scale_set = child(measurement, "BodyScaleSet")
        scale_objects = child(scale_set, "objects")
        for body_name in body_names:
            scale = child(scale_objects, "BodyScale", name=body_name)
            child(scale, "axes", axes)
        child(scale_set, "groups")
    child(measurement_set, "groups")


def empty_measurement_set_xml(parent: ET.Element) -> None:
    measurement_set = child(parent, "MeasurementSet")
    child(measurement_set, "objects")
    child(measurement_set, "groups")


def build_scale_setup(
    destination: Path,
    model_file: Path,
    marker_set_file: Path,
    trc_file: Path,
    marker_names: list[str],
    start: float,
    end: float,
    output_model: Path,
    output_scale_file: Path,
    output_motion_file: Path,
    output_marker_file: Path,
    mode: str,
    mass_kg: float = 67.389928810000015,
    task_weight: float = 1.0,
) -> Path:
    root = ET.Element("OpenSimDocument", Version=OPEN_SIM_XML_VERSION)
    tool = child(root, "ScaleTool", name="sea_pipeline_scaled")
    child(tool, "mass", fmt(mass_kg))
    child(tool, "height", "-1")
    child(tool, "age", "-1")
    child(tool, "notes", "Generated by run_opensim_sea_pipeline.py")

    maker = child(tool, "GenericModelMaker")
    child(maker, "model_file", str(model_file))
    child(maker, "marker_set_file", str(marker_set_file))

    scaler = child(tool, "ModelScaler")
    child(scaler, "apply", "true")
    if mode == "measurements":
        child(scaler, "scaling_order", "measurements")
        measurement_set_xml(scaler)
    elif mode == "marker_only":
        child(scaler, "scaling_order", "")
        empty_measurement_set_xml(scaler)
    else:
        raise ValueError(f"Unknown scaling mode: {mode}")
    scale_set = child(scaler, "ScaleSet")
    child(scale_set, "objects")
    child(scale_set, "groups")
    child(scaler, "marker_file", str(trc_file))
    child(scaler, "time_range", f"{fmt(start)} {fmt(end)}")
    child(scaler, "preserve_mass_distribution", "true")
    child(scaler, "output_model_file", str(output_model))
    child(scaler, "output_scale_file", str(output_scale_file))

    placer = child(tool, "MarkerPlacer")
    child(placer, "apply", "true")
    marker_tasks_xml(placer, marker_names, task_weight)
    child(placer, "marker_file", str(trc_file))
    child(placer, "coordinate_file", "")
    child(placer, "time_range", f"{fmt(start)} {fmt(end)}")
    child(placer, "output_motion_file", str(output_motion_file))
    child(placer, "output_model_file", str(output_model))
    child(placer, "output_marker_file", str(output_marker_file))
    child(placer, "max_marker_movement", "-1")
    return write_xml(root, destination)


def build_ik_setup(
    destination: Path,
    model_file: Path,
    trc_file: Path,
    marker_names: list[str],
    results_dir: Path,
    output_motion_file: str,
    start: float,
    end: float,
    task_weight: float = 1.0,
) -> Path:
    root = ET.Element("OpenSimDocument", Version=OPEN_SIM_XML_VERSION)
    tool = child(root, "InverseKinematicsTool", name=Path(output_motion_file).stem)
    child(tool, "results_directory", str(results_dir))
    child(tool, "model_file", str(model_file))
    child(tool, "constraint_weight", "Inf")
    child(tool, "accuracy", "1e-05")
    child(tool, "time_range", f"{fmt(start)} {fmt(end)}")
    child(tool, "output_motion_file", output_motion_file)
    child(tool, "report_errors", "true")
    marker_tasks_xml(tool, marker_names, task_weight)
    child(tool, "marker_file", str(trc_file))
    child(tool, "coordinate_file", "")
    child(tool, "report_marker_locations", "false")
    return write_xml(root, destination)


def build_external_forces(destination: Path, grf_file: Path, kinematics_file: Path | None = None) -> Path:
    root = ET.Element("OpenSimDocument", Version=OPEN_SIM_XML_VERSION)
    loads = child(root, "ExternalLoads", name="externalloads")
    objects = child(loads, "objects")

    force_specs = [
        ("right_FP2", "calcn_r", "FP2_v", "FP2_p", "FP2_moment_"),
        ("left_FP1", "foot_l", "FP1_v", "FP1_p", "FP1_moment_"),
    ]
    for name, body, force_id, point_id, torque_id in force_specs:
        force = child(objects, "ExternalForce", name=name)
        child(force, "applied_to_body", body)
        child(force, "force_expressed_in_body", "ground")
        child(force, "point_expressed_in_body", "ground")
        child(force, "force_identifier", force_id)
        child(force, "point_identifier", point_id)
        child(force, "torque_identifier", torque_id)
    child(loads, "groups")
    child(loads, "datafile", str(grf_file))
    child(loads, "external_loads_model_kinematics_file", "" if kinematics_file is None else str(kinematics_file))
    child(loads, "lowpass_cutoff_frequency_for_load_kinematics", "6")
    return write_xml(root, destination)


def build_id_setup(
    destination: Path,
    model_file: Path,
    external_loads: Path,
    coordinates_file: Path,
    results_dir: Path,
    start: float,
    end: float,
) -> Path:
    root = ET.Element("OpenSimDocument", Version=OPEN_SIM_XML_VERSION)
    tool = child(root, "InverseDynamicsTool", name="sea_pipeline_id")
    child(tool, "results_directory", str(results_dir))
    child(tool, "model_file", str(model_file))
    child(tool, "time_range", f"{fmt(start)} {fmt(end)}")
    child(tool, "forces_to_exclude", "muscles")
    child(tool, "external_loads_file", str(external_loads))
    child(tool, "coordinates_file", str(coordinates_file))
    child(tool, "lowpass_cutoff_frequency_for_coordinates", "6")
    child(tool, "output_gen_force_file", "inverse_dynamics.sto")
    child(tool, "joints_to_report_body_forces", "")
    child(tool, "output_body_forces_file", "body_forces_at_joints.sto")
    return write_xml(root, destination)


def build_rra_setup(
    destination: Path,
    model_file: Path,
    actuators_file: Path,
    task_file: Path,
    external_loads: Path,
    desired_kinematics: Path,
    results_dir: Path,
    output_model: Path,
    adjusted_body: str | None,
    start: float,
    end: float,
) -> Path:
    root = ET.Element("OpenSimDocument", Version=OPEN_SIM_XML_VERSION)
    tool = child(root, "RRATool", name=f"sea_pipeline_rra_{adjusted_body or 'no_com'}")
    child(tool, "model_file", str(model_file))
    child(tool, "replace_force_set", "true")
    child(tool, "force_set_files", str(actuators_file))
    child(tool, "results_directory", str(results_dir))
    child(tool, "output_precision", "8")
    child(tool, "initial_time", fmt(start))
    child(tool, "final_time", fmt(end))
    child(tool, "solve_for_equilibrium_for_auxiliary_states", "false")
    child(tool, "maximum_number_of_integrator_steps", "20000")
    child(tool, "maximum_integrator_step_size", "1")
    child(tool, "minimum_integrator_step_size", "1e-08")
    child(tool, "integrator_error_tolerance", "1e-05")
    analyses = child(tool, "AnalysisSet", name="Analyses")
    child(analyses, "objects")
    child(analyses, "groups")
    controllers = child(tool, "ControllerSet", name="Controllers")
    child(controllers, "objects")
    child(controllers, "groups")
    child(tool, "external_loads_file", str(external_loads))
    child(tool, "desired_points_file", "")
    child(tool, "desired_kinematics_file", str(desired_kinematics))
    child(tool, "task_set_file", str(task_file))
    child(tool, "constraints_file", "")
    child(tool, "lowpass_cutoff_frequency", "6")
    child(tool, "optimizer_algorithm", "ipopt")
    child(tool, "numerical_derivative_step_size", "0.0001")
    child(tool, "optimization_convergence_tolerance", "1e-05")
    child(tool, "adjust_com_to_reduce_residuals", "true" if adjusted_body else "false")
    child(tool, "initial_time_for_com_adjustment", "-1")
    child(tool, "final_time_for_com_adjustment", "-1")
    child(tool, "adjusted_com_body", adjusted_body or "")
    child(tool, "output_model_file", str(output_model))
    child(tool, "use_verbose_printing", "false")
    return write_xml(root, destination)


def build_rra_actuators(destination: Path, scaled_model: Path, cmc_actuators: Path) -> Path:
    pelvis_com = body_mass_center(scaled_model, "pelvis") or [-0.07, 0.0, 0.0]
    pelvis_point = " ".join(fmt(value) for value in pelvis_com)

    root = ET.Element("OpenSimDocument", Version=OPEN_SIM_XML_VERSION)
    force_set = child(root, "ForceSet", name="sea_pipeline_RRA")
    child(force_set, "defaults")
    objects = child(force_set, "objects")

    residual_forces = [
        ("FX", (1.0, 0.0, 0.0), 4.0),
        ("FY", (0.0, 1.0, 0.0), 5000.0),
        ("FZ", (0.0, 0.0, 1.0), 3000.0),
    ]
    for name, direction, optimal_force in residual_forces:
        actuator = child(objects, "PointActuator", name=name)
        child(actuator, "isDisabled", "false")
        child(actuator, "min_control", "-infinity")
        child(actuator, "max_control", "infinity")
        child(actuator, "body", "pelvis")
        child(actuator, "point", pelvis_point)
        child(actuator, "point_is_global", "false")
        child(actuator, "direction", " ".join(fmt(value) for value in direction))
        child(actuator, "force_is_global", "true")
        child(actuator, "optimal_force", fmt(optimal_force))

    residual_moments = [
        ("MX", (1.0, 0.0, 0.0), 1000.0),
        ("MY", (0.0, 1.0, 0.0), 1000.0),
        ("MZ", (0.0, 0.0, 1.0), 1000.0),
    ]
    for name, axis, optimal_force in residual_moments:
        actuator = child(objects, "TorqueActuator", name=name)
        child(actuator, "isDisabled", "false")
        child(actuator, "min_control", "-infinity")
        child(actuator, "max_control", "infinity")
        child(actuator, "bodyA", "pelvis")
        child(actuator, "bodyB", "ground")
        child(actuator, "torque_is_global", "true")
        child(actuator, "axis", " ".join(fmt(value) for value in axis))
        child(actuator, "optimal_force", fmt(optimal_force))

    for coordinate in parse_coordinate_actuators(cmc_actuators):
        if coordinate.startswith("pelvis_"):
            continue
        actuator = child(objects, "CoordinateActuator", name=coordinate)
        child(actuator, "isDisabled", "false")
        child(actuator, "min_control", "-infinity")
        child(actuator, "max_control", "infinity")
        child(actuator, "coordinate", coordinate)
        child(actuator, "optimal_force", fmt(rra_coordinate_optimal_force(coordinate)))

    child(force_set, "groups")
    return write_xml(root, destination)


def parse_coordinate_actuators(actuator_file: Path) -> list[str]:
    root = ET.fromstring(actuator_file.read_text(encoding="utf-8", errors="replace").lstrip())
    coordinates: list[str] = []
    for node in root.iter("CoordinateActuator"):
        coord = node.findtext("coordinate")
        if coord:
            coord = coord.strip()
            if coord and coord not in coordinates:
                coordinates.append(coord)
    if not coordinates:
        raise ValueError(f"No CoordinateActuator coordinates found in {actuator_file}")
    return coordinates


def rra_coordinate_optimal_force(coordinate: str) -> float:
    if "hip_flexion" in coordinate or "knee_angle" in coordinate or "ankle_angle" in coordinate:
        return 300.0
    if "hip_adduction" in coordinate:
        return 200.0
    if coordinate.startswith("lumbar_"):
        return 200.0
    if coordinate.startswith("pros_"):
        return 300.0
    return 100.0


def build_simulator_setup(
    destination: Path,
    model_file: Path,
    kinematics_file: Path,
    external_loads: Path,
    reserve_actuators: Path,
    start: float,
    end: float,
) -> Path:
    root = ET.Element("OpenSimDocument", Version=OPEN_SIM_XML_VERSION)
    setup = child(root, "CMC_Simulator_Setup", name=destination.stem)
    child(setup, "model_file", str(model_file))
    child(setup, "kinematics_file", str(kinematics_file))
    child(setup, "external_loads_xml", str(external_loads))
    child(setup, "reserve_actuators_xml", str(reserve_actuators))
    child(setup, "t_start", fmt(start))
    child(setup, "t_end", fmt(end))
    return write_xml(root, destination)


def run_opensim_tool(setup_xml: Path, worker_dir: Path, cfg: ToolConfig, label: str) -> CommandResult:
    worker_dir.mkdir(parents=True, exist_ok=True)
    log_path = worker_dir / f"{label}.log"
    cmd = [
        str(cfg.opensim_cmd),
        "--library",
        str(cfg.plugin),
        "--log=info",
        "run-tool",
        str(setup_xml),
    ]
    started = time.perf_counter()
    if cfg.dry_run:
        text = "DRY RUN: " + " ".join(cmd)
        log_path.write_text(text + "\n", encoding="utf-8")
        return CommandResult(0, text, 0.0)

    completed = subprocess.run(
        cmd,
        cwd=worker_dir,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    elapsed = time.perf_counter() - started
    log_path.write_text(completed.stdout, encoding="utf-8", errors="replace")
    return CommandResult(completed.returncode, completed.stdout, elapsed)


def parse_ik_marker_error_files(directory: Path) -> tuple[float | None, float | None]:
    candidates = list(directory.glob("*marker_errors*.sto")) + list(directory.glob("*_ik_marker_errors.sto"))
    if not candidates:
        return None, None
    return parse_marker_error_sto(candidates[0])


def find_generated_file(primary_dir: Path, fallback_dir: Path, filename: str) -> Path:
    candidates = [
        primary_dir / filename,
        fallback_dir / filename,
    ]
    candidates.extend(primary_dir.rglob(filename))
    candidates.extend(fallback_dir.rglob(filename))
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    raise FileNotFoundError(
        f"Unable to locate generated file {filename!r} under {primary_dir} or {fallback_dir}"
    )


def parse_marker_error_sto(path: Path) -> tuple[float | None, float | None]:
    header: list[str] | None = None
    rows: list[list[float]] = []
    with path.open("r", encoding="utf-8", errors="replace") as fh:
        for raw_line in fh:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if parts and parts[0].lower() == "time":
                header = parts
                continue
            if header is None:
                continue
            try:
                rows.append([float(value) for value in parts])
            except ValueError:
                continue
    if header is None or not rows:
        return None, None
    labels = {name.lower(): idx for idx, name in enumerate(header)}
    rms_idx = next((idx for name, idx in labels.items() if name in {"rms", "rms_error"}), None)
    max_idx = next((idx for name, idx in labels.items() if name in {"max", "max_error"}), None)
    if rms_idx is None:
        rms_idx = len(header) - 2 if len(header) >= 3 else None
    if max_idx is None:
        max_idx = len(header) - 1 if len(header) >= 3 else None
    rms_values = [row[rms_idx] for row in rows if rms_idx is not None and len(row) > rms_idx]
    max_values = [row[max_idx] for row in rows if max_idx is not None and len(row) > max_idx]
    return (
        max(rms_values) if rms_values else None,
        max(max_values) if max_values else None,
    )


def parse_max_scale_delta(scale_file: Path) -> float | None:
    if not scale_file.is_file():
        return None
    root = ET.fromstring(scale_file.read_text(encoding="utf-8", errors="replace").lstrip())
    deltas: list[float] = []
    for scale in root.iter("Scale"):
        values = scale.findtext("scales")
        if not values:
            continue
        for raw in values.split():
            try:
                deltas.append(abs(float(raw) - 1.0))
            except ValueError:
                continue
    return max(deltas) if deltas else None


def parse_residual_file(path: Path) -> dict[str, float]:
    residuals: dict[str, float] = {}
    if not path.is_file():
        return residuals
    pattern = re.compile(r"\b(FX|FY|FZ|MX|MY|MZ)\s+average\s*=\s*([-+0-9.eE]+)")
    for match in pattern.finditer(path.read_text(encoding="utf-8", errors="replace")):
        residuals[match.group(1)] = float(match.group(2))
    return residuals


def parse_residuals_from_logs(directory: Path) -> dict[str, float]:
    residuals: dict[str, float] = {}
    pattern = re.compile(r"\b(FX|FY|FZ|MX|MY|MZ)\s*=\s*([-+0-9.eE]+)")
    for path in list(directory.glob("*avgResiduals.txt")) + list(directory.glob("*.log")):
        text = path.read_text(encoding="utf-8", errors="replace")
        for match in pattern.finditer(text):
            residuals[match.group(1)] = float(match.group(2))
    return residuals


def body_mass_center(model_xml: Path, body_name: str) -> list[float] | None:
    tree = ET.parse(model_xml)
    for node in tree.iter("Body"):
        if node.attrib.get("name") == body_name:
            mass_center = node.find("mass_center")
            if mass_center is None or not mass_center.text:
                return None
            return [float(value) for value in mass_center.text.split()]
    return None


def compute_com_shift_m(base_model: Path, adjusted_model: Path, body_name: str | None) -> float | None:
    if body_name is None or not adjusted_model.is_file():
        return 0.0 if body_name is None else None
    base = body_mass_center(base_model, body_name)
    adjusted = body_mass_center(adjusted_model, body_name)
    if base is None or adjusted is None or len(base) != 3 or len(adjusted) != 3:
        return None
    return max(abs(a - b) for a, b in zip(adjusted, base))


def force_norm(residuals: dict[str, float]) -> float | None:
    keys = ("FX", "FY", "FZ")
    if not all(key in residuals for key in keys):
        return None
    return math.sqrt(sum(residuals[key] ** 2 for key in keys))


def moment_norm(residuals: dict[str, float]) -> float | None:
    keys = ("MX", "MY", "MZ")
    if not all(key in residuals for key in keys):
        return None
    return math.sqrt(sum(residuals[key] ** 2 for key in keys))


def promote_scale_candidate(
    candidate: ScaleCandidate,
    paths: Paths,
) -> tuple[Path, Path]:
    final_model = paths.scaling_dir / "scaled_SEASEA.osim"
    final_scale = paths.scaling_dir / "scale_factors.xml"
    shutil.copy2(candidate.output_model, final_model)
    if candidate.output_scale_file.is_file():
        shutil.copy2(candidate.output_scale_file, final_scale)
    shutil.copy2(candidate.setup_xml, paths.scaling_dir / "Scale_setup.xml")
    return final_model, final_scale


def promote_ik_outputs(paths: Paths, full_output: Path, rra_output: Path, full_setup: Path, rra_setup: Path) -> None:
    for source in (full_output, rra_output, full_setup, rra_setup):
        if source.is_file():
            destination = paths.ik_dir / source.name
            if source.resolve() != destination.resolve():
                shutil.copy2(source, destination)


def copy_mass_center(
    base_model: Path,
    adjusted_model: Path,
    output_model: Path,
    body_name: str | None,
) -> None:
    if body_name is None:
        shutil.copy2(base_model, output_model)
        return

    base_tree = ET.parse(base_model)
    adjusted_center = body_mass_center(adjusted_model, body_name)
    if adjusted_center is None:
        raise ValueError(f"Unable to read adjusted mass_center for {body_name} from {adjusted_model}")

    updated = False
    for node in base_tree.iter("Body"):
        if node.attrib.get("name") == body_name:
            mass_center = node.find("mass_center")
            if mass_center is None:
                mass_center = child(node, "mass_center")
            mass_center.text = " ".join(fmt(value) for value in adjusted_center)
            updated = True
            break
    if not updated:
        raise ValueError(f"Body {body_name} not found in base model {base_model}")

    ET.indent(base_tree, space="\t")
    base_tree.write(output_model, encoding="utf-8", xml_declaration=True)


def verify_sea_actuators(model_file: Path) -> bool:
    text = model_file.read_text(encoding="utf-8", errors="replace")
    return "SeriesElasticActuator" in text and "SEA_Knee" in text and "SEA_Ankle" in text


def run_scale_candidate_worker(args: tuple[ToolConfig, Path, Path, Path, Path, list[str], int, str, float, float, float]) -> ScaleCandidate:
    cfg, model, marker_set, trc, workers_dir, marker_names, index, mode, start, end, stability = args
    worker_dir = workers_dir / f"scaling_{mode}_{index:03d}_{start:.3f}_{end:.3f}"
    output_model = worker_dir / "scaled_SEASEA.osim"
    output_scale_file = worker_dir / "scale_factors.xml"
    output_motion_file = worker_dir / "static_pose.mot"
    output_marker_file = worker_dir / "placed_markers.xml"
    setup_xml = worker_dir / "Scale_setup.xml"

    candidate = ScaleCandidate(
        index=index,
        mode=mode,
        start=start,
        end=end,
        stability_mm_per_frame=stability,
        worker_dir=worker_dir,
        setup_xml=setup_xml,
        output_model=output_model,
        output_scale_file=output_scale_file,
        output_motion_file=output_motion_file,
    )

    build_scale_setup(
        setup_xml,
        model,
        marker_set,
        trc,
        marker_names,
        start,
        end,
        output_model,
        output_scale_file,
        output_motion_file,
        output_marker_file,
        mode,
    )
    result = run_opensim_tool(setup_xml, worker_dir, cfg, "scale")
    candidate.command_returncode = result.returncode
    if result.returncode != 0 or not output_model.is_file():
        candidate.status = "failed"
        candidate.notes.append(f"ScaleTool failed with code {result.returncode}")
        return candidate
    candidate.max_scale_delta = parse_max_scale_delta(output_scale_file)

    ik_dir = worker_dir / "scale_quality_ik"
    ik_setup = worker_dir / "Scale_quality_IK_setup.xml"
    build_ik_setup(
        ik_setup,
        output_model,
        trc,
        marker_names,
        ik_dir,
        "scale_quality_ik.mot",
        start,
        end,
    )
    ik_result = run_opensim_tool(ik_setup, worker_dir, cfg, "scale_quality_ik")
    if ik_result.returncode != 0:
        candidate.status = "failed"
        candidate.notes.append(f"Scale quality IK failed with code {ik_result.returncode}")
        return candidate

    rms, max_error = parse_ik_marker_error_files(ik_dir)
    candidate.rms_marker_error_m = rms
    candidate.max_marker_error_m = max_error
    candidate.status = "ok"
    if rms is None:
        candidate.notes.append("Marker RMS unavailable; selection will use stability only.")
    return candidate


def run_rra_candidate_worker(args: tuple[ToolConfig, Path, Path, Path, str, Path, Path, Path, str | None]) -> RraCandidate:
    cfg, scaled_model, actuators, tasks, task_profile, external_forces, desired_kinematics, workers_dir, adjusted_body = args
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
        RRA_TIME_RANGE[0],
        RRA_TIME_RANGE[1],
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
        candidate.notes.append("Average residuals unavailable in logs/results.")
    if adjusted_body and not output_model.is_file():
        candidate.status = "failed"
        candidate.notes.append("RRA did not produce adjusted model.")
    return candidate


def scale_selection_key(candidate: ScaleCandidate) -> tuple[float, float, float, float]:
    rms = candidate.rms_marker_error_m
    if rms is None:
        rms = math.inf
    rms_bad = 1.0 if rms > RMS_TARGET_M else 0.0
    scale_delta = candidate.max_scale_delta
    if scale_delta is None:
        scale_delta = math.inf
    return (rms_bad, scale_delta, rms, candidate.stability_mm_per_frame)


def rra_selection_key(candidate: RraCandidate) -> tuple[int, float, float, float, float]:
    if candidate.status != "ok":
        return (1, math.inf, math.inf, math.inf, math.inf)
    com_bad = 1 if candidate.com_shift_m is None or candidate.com_shift_m > MAX_COM_SHIFT_M + 1e-9 else 0
    force = candidate.force_norm if candidate.force_norm is not None else math.inf
    moment = candidate.moment_norm if candidate.moment_norm is not None else math.inf
    com = candidate.com_shift_m if candidate.com_shift_m is not None else math.inf
    score = (force / 30.0) + (moment / 65.0)
    return (com_bad, score, force, moment, com)


def summarize_scale_candidates(candidates: list[ScaleCandidate], destination: Path) -> None:
    with destination.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(
            [
                "index",
                "mode",
                "start",
                "end",
                "stability_mm_per_frame",
                "rms_marker_error_m",
                "max_marker_error_m",
                "max_scale_delta",
                "status",
                "worker_dir",
                "notes",
            ]
        )
        for c in candidates:
            writer.writerow(
                [
                    c.index,
                    c.mode,
                    c.start,
                    c.end,
                    c.stability_mm_per_frame,
                    c.rms_marker_error_m,
                    c.max_marker_error_m,
                    c.max_scale_delta,
                    c.status,
                    c.worker_dir,
                    "; ".join(c.notes),
                ]
            )


def summarize_rra_candidates(candidates: list[RraCandidate], destination: Path) -> None:
    with destination.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(
            [
                "name",
                "task_profile",
                "adjusted_body",
                "force_norm",
                "moment_norm",
                "com_shift_max_component_m",
                "FX",
                "FY",
                "FZ",
                "MX",
                "MY",
                "MZ",
                "status",
                "worker_dir",
                "notes",
            ]
        )
        for c in candidates:
            writer.writerow(
                [
                    c.name,
                    c.task_profile,
                    c.adjusted_body or "",
                    c.force_norm,
                    c.moment_norm,
                    c.com_shift_m,
                    c.residuals.get("FX"),
                    c.residuals.get("FY"),
                    c.residuals.get("FZ"),
                    c.residuals.get("MX"),
                    c.residuals.get("MY"),
                    c.residuals.get("MZ"),
                    c.status,
                    c.worker_dir,
                    "; ".join(c.notes),
                ]
            )


def write_report(
    destination: Path,
    scale_candidates: list[ScaleCandidate],
    selected_scale: ScaleCandidate,
    rra_candidates: list[RraCandidate],
    selected_rra: RraCandidate | None,
    final_model: Path | None,
    smoke_returncode: int | None,
) -> None:
    lines = [
        "# OpenSim SEA Pipeline Report",
        "",
        f"Generated: {datetime.now().isoformat(timespec='seconds')}",
        "",
        "## Scaling",
        "",
        f"Selected mode: {selected_scale.mode}",
        f"Selected window: {selected_scale.start:.6f} - {selected_scale.end:.6f} s",
        f"Selected RMS marker error: {selected_scale.rms_marker_error_m}",
        f"Selected max scale delta: {selected_scale.max_scale_delta}",
        f"RMS target: {RMS_TARGET_M} m",
        "",
        "## RRA",
        "",
    ]
    if selected_rra is None:
        lines.append("No valid RRA candidate was selected.")
    else:
        lines.extend(
            [
                f"Selected candidate: {selected_rra.name}",
                f"Adjusted body: {selected_rra.adjusted_body}",
                f"Residuals: {json.dumps(selected_rra.residuals, sort_keys=True)}",
                f"Force norm: {selected_rra.force_norm}",
                f"Moment norm: {selected_rra.moment_norm}",
                f"COM shift max component: {selected_rra.com_shift_m}",
            ]
        )
    lines.extend(
        [
            "",
            "## Final",
            "",
            f"Final model: {final_model if final_model else 'not produced'}",
            f"Smoke test return code: {smoke_returncode}",
            "",
            "## Candidate Files",
            "",
            "- Scaling summary: `01_scaling/scaling_candidates.csv`",
            "- RRA summary: `04_rra/rra_candidates.csv`",
        ]
    )
    destination.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_smoke_test(setup_xml: Path, paths: Paths, dry_run: bool) -> int | None:
    smoke_dir = paths.final_dir / "smoke_results"
    smoke_dir.mkdir(parents=True, exist_ok=True)
    cmd = [
        sys.executable,
        str(REPO_ROOT / "main.py"),
        "--setup",
        str(setup_xml),
        "--t-start",
        fmt(RRA_TIME_RANGE[0]),
        "--t-end",
        fmt(min(RRA_TIME_RANGE[0] + 0.03, RRA_TIME_RANGE[1])),
        "--output-dir",
        str(smoke_dir),
    ]
    log_path = paths.logs_dir / "smoke_test.log"
    if dry_run:
        log_path.write_text("DRY RUN: " + " ".join(cmd) + "\n", encoding="utf-8")
        return 0
    completed = subprocess.run(
        cmd,
        cwd=REPO_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    log_path.write_text(completed.stdout, encoding="utf-8", errors="replace")
    return completed.returncode


def run_pipeline(cfg: ToolConfig, paths: Paths, run_smoke: bool) -> int:
    started = time.perf_counter()
    mkdirs(paths)

    copied = copy_inputs(cfg, paths)
    marker_names = parse_marker_names(copied["marker_set"])
    windows = select_scaling_windows(copied["trc"])
    (paths.logs_dir / "selected_scaling_windows.json").write_text(
        json.dumps(
            [
                {"index": idx, "start": start, "end": end, "stability_mm_per_frame": score}
                for idx, start, end, score in windows
            ],
            indent=2,
        ),
        encoding="utf-8",
    )

    print(f"[pipeline] Running scaling candidates with {WORKER_COUNT} workers...")
    scale_args = []
    for idx, start, end, stability in windows:
        for mode in ("marker_only", "measurements"):
            scale_args.append(
                (
                    cfg,
                    copied["model"],
                    copied["marker_set"],
                    copied["trc"],
                    paths.workers_dir,
                    marker_names,
                    idx,
                    mode,
                    start,
                    end,
                    stability,
                )
            )
    with concurrent.futures.ProcessPoolExecutor(max_workers=WORKER_COUNT) as pool:
        scale_candidates = list(pool.map(run_scale_candidate_worker, scale_args))

    summarize_scale_candidates(scale_candidates, paths.scaling_dir / "scaling_candidates.csv")
    ok_scale_candidates = [c for c in scale_candidates if c.status == "ok" and c.output_model.is_file()]
    if not ok_scale_candidates:
        raise RuntimeError("No scaling candidate completed successfully.")

    selected_scale = min(ok_scale_candidates, key=scale_selection_key)
    if selected_scale.rms_marker_error_m is not None and selected_scale.rms_marker_error_m > RMS_TARGET_M:
        print(
            "[pipeline] WARNING: selected scaling RMS is above target: "
            f"{selected_scale.rms_marker_error_m:.6f} m > {RMS_TARGET_M:.6f} m"
        )
    scaled_model, _ = promote_scale_candidate(selected_scale, paths)

    print("[pipeline] Running IK...")
    full_ik_setup = paths.ik_dir / "IK_full_17_23_setup.xml"
    full_ik_results = paths.ik_dir / "full"
    full_ik_motion = "IK_full_17_23.mot"
    build_ik_setup(
        full_ik_setup,
        scaled_model,
        copied["trc"],
        marker_names,
        full_ik_results,
        full_ik_motion,
        FULL_TIME_RANGE[0],
        FULL_TIME_RANGE[1],
    )
    full_ik_result = run_opensim_tool(full_ik_setup, paths.ik_dir, cfg, "ik_full")
    if full_ik_result.returncode != 0:
        raise RuntimeError("Full IK failed; inspect 02_ik/ik_full.log")

    rra_ik_setup = paths.ik_dir / "IK_rra_window_setup.xml"
    rra_ik_results = paths.ik_dir / "rra_window"
    rra_ik_motion = "IK_rra_window.mot"
    build_ik_setup(
        rra_ik_setup,
        scaled_model,
        copied["trc"],
        marker_names,
        rra_ik_results,
        rra_ik_motion,
        RRA_TIME_RANGE[0],
        RRA_TIME_RANGE[1],
    )
    rra_ik_result = run_opensim_tool(rra_ik_setup, paths.ik_dir, cfg, "ik_rra_window")
    if rra_ik_result.returncode != 0:
        raise RuntimeError("RRA-window IK failed; inspect 02_ik/ik_rra_window.log")
    full_kinematics = find_generated_file(paths.ik_dir, full_ik_results, full_ik_motion)
    rra_kinematics = find_generated_file(paths.ik_dir, rra_ik_results, rra_ik_motion)
    promote_ik_outputs(paths, full_kinematics, rra_kinematics, full_ik_setup, rra_ik_setup)

    print("[pipeline] Running ID diagnostic...")
    external_forces = paths.id_dir / "ExternalForces.xml"
    build_external_forces(external_forces, copied["grf"], rra_kinematics)
    id_setup = paths.id_dir / "ID_setup.xml"
    build_id_setup(
        id_setup,
        scaled_model,
        external_forces,
        rra_kinematics,
        paths.id_dir,
        RRA_TIME_RANGE[0],
        RRA_TIME_RANGE[1],
    )
    id_result = run_opensim_tool(id_setup, paths.id_dir, cfg, "id")
    if id_result.returncode != 0:
        print("[pipeline] WARNING: ID failed; continuing to RRA for diagnostics.")

    print(f"[pipeline] Running RRA candidates with {WORKER_COUNT} workers...")
    rra_actuators = paths.rra_dir / "RRA_Actuators.xml"
    rra_tasks = paths.rra_dir / "RRA_Tasks_repo_cmc.xml"
    build_rra_actuators(rra_actuators, scaled_model, copied["cmc_actuators"])
    shutil.copy2(copied["cmc_tasks"], rra_tasks)
    task_profiles: list[tuple[str, Path]] = [("repo_cmc_tasks", rra_tasks)]
    if cfg.legacy_rra_tasks is not None:
        legacy_tasks = paths.rra_dir / "RRA_Tasks_legacy.xml"
        shutil.copy2(cfg.legacy_rra_tasks, legacy_tasks)
        task_profiles.append(("legacy_rra_tasks", legacy_tasks))

    rra_external = paths.rra_dir / "ExternalForces.xml"
    build_external_forces(rra_external, copied["grf"], rra_kinematics)
    rra_kin_copy = paths.rra_dir / rra_kinematics.name
    shutil.copy2(rra_kinematics, rra_kin_copy)

    rra_args = []
    for task_profile, task_file in task_profiles:
        for body in ("pelvis", "torso", None):
            rra_args.append(
                (
                    cfg,
                    scaled_model,
                    rra_actuators,
                    task_file,
                    task_profile,
                    rra_external,
                    rra_kin_copy,
                    paths.workers_dir,
                    body,
                )
            )
    with concurrent.futures.ProcessPoolExecutor(max_workers=WORKER_COUNT) as pool:
        rra_candidates = list(pool.map(run_rra_candidate_worker, rra_args))

    summarize_rra_candidates(rra_candidates, paths.rra_dir / "rra_candidates.csv")
    ok_rra_candidates = [c for c in rra_candidates if c.status == "ok"]
    selected_rra = min(ok_rra_candidates, key=rra_selection_key) if ok_rra_candidates else None
    if selected_rra is None:
        raise RuntimeError("No RRA candidate completed successfully.")
    if selected_rra.com_shift_m is None or selected_rra.com_shift_m > MAX_COM_SHIFT_M + 1e-9:
        print(
            "[pipeline] WARNING: selected RRA candidate exceeds COM-shift criterion; "
            "promoting diagnostic output only."
        )

    final_model = paths.final_dir / "Adjusted_newmarkers_pipeline_ready.osim"
    source_adjusted = selected_rra.output_model if selected_rra.adjusted_body else scaled_model
    copy_mass_center(scaled_model, source_adjusted, final_model, selected_rra.adjusted_body)
    if not verify_sea_actuators(final_model):
        raise RuntimeError(f"Final model does not contain both SEA actuators: {final_model}")

    final_data_dir = paths.final_dir / "data"
    final_data_dir.mkdir(parents=True, exist_ok=True)
    final_kinematics = final_data_dir / "IK_rra_window.mot"
    final_external = final_data_dir / "ExternalForces.xml"
    final_actuators = final_data_dir / "CMC_Actuators.xml"
    shutil.copy2(rra_kinematics, final_kinematics)
    shutil.copy2(rra_external, final_external)
    shutil.copy2(copied["cmc_actuators"], final_actuators)
    shutil.copy2(copied["cmc_tasks"], final_data_dir / "CMC_Tasks - modified Kp_Kv.xml")

    simulator_setup = paths.final_dir / "Adjusted_newmarkers_pipeline_ready_setup.xml"
    build_simulator_setup(
        simulator_setup,
        final_model,
        final_kinematics,
        final_external,
        final_actuators,
        RRA_TIME_RANGE[0],
        RRA_TIME_RANGE[1],
    )

    smoke_returncode = run_smoke_test(simulator_setup, paths, cfg.dry_run) if run_smoke else None
    write_report(
        paths.pipeline_root / "pipeline_report.md",
        scale_candidates,
        selected_scale,
        rra_candidates,
        selected_rra,
        final_model,
        smoke_returncode,
    )
    elapsed = time.perf_counter() - started
    print(f"[pipeline] Done in {elapsed:.1f} s")
    print(f"[pipeline] Final model: {final_model}")
    print(f"[pipeline] Report: {paths.pipeline_root / 'pipeline_report.md'}")
    return 0 if smoke_returncode in (0, None) else smoke_returncode


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pipeline-root", type=Path, default=DEFAULT_PIPELINE_ROOT)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--marker-set", type=Path, default=DEFAULT_MARKER_SET)
    parser.add_argument("--trc", type=Path, default=DEFAULT_TRC)
    parser.add_argument("--grf", type=Path, default=DEFAULT_GRF)
    parser.add_argument("--support-data", type=Path, default=DEFAULT_SUPPORT_DATA)
    parser.add_argument("--opensim-cmd", type=Path, default=DEFAULT_OPENSIM_CMD)
    parser.add_argument("--plugin", type=Path, default=DEFAULT_PLUGIN)
    parser.add_argument("--legacy-rra-tasks", type=Path, default=DEFAULT_LEGACY_RRA_TASKS)
    parser.add_argument("--skip-smoke-test", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    cfg = ToolConfig(
        opensim_cmd=require_file(args.opensim_cmd, "opensim-cmd"),
        plugin=require_file(args.plugin, "SEA plugin"),
        model=require_file(args.model, "Initial model"),
        marker_set=require_file(args.marker_set, "MarkerSet"),
        trc=require_file(args.trc, "TRC"),
        grf=require_file(args.grf, "GRF MOT"),
        support_data=require_dir(args.support_data, "SEASEA support data"),
        legacy_rra_tasks=ensure_absolute(args.legacy_rra_tasks)
        if args.legacy_rra_tasks and ensure_absolute(args.legacy_rra_tasks).is_file()
        else None,
        dry_run=args.dry_run,
    )
    paths = create_paths(args.pipeline_root)
    return run_pipeline(cfg, paths, run_smoke=not args.skip_smoke_test)


if __name__ == "__main__":
    raise SystemExit(main())
