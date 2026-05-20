#!/usr/bin/env python3
"""Build a subject-specific EPIC ABx -> ABx_SEASEA treadmill bundle.

This is the single public entrypoint for reconstructing the workflow that was
first developed for AB06. It keeps the heavy domain-specific readers in MATLAB
because EPIC .mat files are MATLAB table/MCOS files, and it reuses the existing
OpenSim preparation pipeline when an OpenSim IK/RRA diagnostic pass is needed.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import re
import shlex
import shutil
import subprocess
import sys
from copy import deepcopy
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Iterable
from xml.etree import ElementTree as ET


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODELS_DIR = REPO_ROOT / "models"
DEFAULT_RESULTS_DIR = REPO_ROOT / "results"
TASKS_WITH_GRF = {"treadmill", "levelground", "ramp", "stair"}
TASK_CHOICES = ("treadmill", "levelground", "ramp", "stair")

RAW_LEFT_BODIES = ("femur_l", "tibia_l", "talus_l", "calcn_l", "toes_l")
PROSTHETIC_BODIES = ("transfemur", "osseo_pylon", "tibia_pylon", "foot_l")
RAW_LEFT_JOINTS = ("hip_l", "knee_l", "ankle_l", "subtalar_l", "mtp_l")
PROSTHETIC_JOINTS = ("osseo", "pros_knee", "pros_ankle")
REMOVED_BODY_REFS = tuple(f"/bodyset/{name}" for name in RAW_LEFT_BODIES)
REMOVED_COORD_REFS = (
    "knee_angle_l",
    "ankle_angle_l",
    "subtalar_angle_l",
    "mtp_angle_l",
)
PROSTHETIC_REFS = tuple(f"/bodyset/{name}" for name in PROSTHETIC_BODIES) + (
    "pros_knee_angle",
    "pros_ankle_angle",
)

LEFT_MARKER_PARENT_MAP = {
    "L_Thigh_Upper": "/bodyset/transfemur",
    "L_Thigh_Front": "/bodyset/transfemur",
    "L_Thigh_Rear": "/bodyset/transfemur",
    "L_Knee_Lat": "/bodyset/osseo_pylon",
    "L_Shank_Upper": "/bodyset/tibia_pylon",
    "L_Shank_Front": "/bodyset/tibia_pylon",
    "L_Shank_Rear": "/bodyset/tibia_pylon",
    "L_Ankle_Lat": "/bodyset/foot_l",
    "L_Heel": "/bodyset/foot_l",
    "L_Toe_Lat": "/bodyset/foot_l",
    "L_Toe_Med": "/bodyset/foot_l",
    "L_Toe_Tip": "/bodyset/foot_l",
}


@dataclass(frozen=True)
class TrialSelection:
    task: str
    trial: str
    marker_file: Path
    fp_file: Path | None
    ik_file: Path | None
    id_file: Path | None


@dataclass(frozen=True)
class SubjectDiscovery:
    subject: str
    raw_dir: Path
    session_dir: Path
    healthy_model: Path
    static_trial: TrialSelection
    operating_trial: TrialSelection


@dataclass
class BuildReport:
    subject: str
    task: str
    trial: str
    static_trial: str
    status: str = "pending"
    raw_dir: str = ""
    session_dir: str = ""
    healthy_model: str = ""
    graft_model: str = ""
    calibrated_model: str = ""
    bundle_dir: str = ""
    setup_xml: str = ""
    converted_static_dir: str = ""
    converted_trial_dir: str = ""
    ik_source: str = ""
    grf_source: str = ""
    healthy_overlay_dir: str = ""
    pipeline_root: str = ""
    rra_pipeline_root: str = ""
    smoke_output_dir: str = ""
    smoke_returncode: int | None = None
    mass_before_kg: float | None = None
    mass_after_kg: float | None = None
    removed_left_mass_kg: float | None = None
    marker_old_mean_error_m: float | None = None
    marker_new_fit_rms_m: float | None = None
    kinematics_time_range: list[float] | None = None
    grf_time_range: list[float] | None = None
    notes: list[str] = field(default_factory=list)
    generated_at: str = field(default_factory=lambda: datetime.now().isoformat(timespec="seconds"))


def repo_relative(path: Path) -> str:
    try:
        return path.resolve().relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return str(path.resolve())


def normalize_subject(raw: str) -> str:
    text = raw.strip().upper()
    match = re.fullmatch(r"AB0*(\d{1,2})", text)
    if not match:
        raise ValueError(f"Invalid subject {raw!r}; expected AB01 .. AB30 style.")
    return f"AB{int(match.group(1)):02d}"


def subject_sort_key(path: Path) -> tuple[int, str]:
    match = re.search(r"AB(\d+)-raw$", path.name, re.IGNORECASE)
    return (int(match.group(1)) if match else 999, path.name)


def discover_subjects(models_dir: Path) -> list[str]:
    subjects: list[str] = []
    for raw_dir in sorted(models_dir.glob("AB*-raw"), key=subject_sort_key):
        match = re.fullmatch(r"(AB\d+)-raw", raw_dir.name, re.IGNORECASE)
        if match:
            subjects.append(normalize_subject(match.group(1)))
    return subjects


def choose_first_existing(paths: Iterable[Path]) -> Path | None:
    for path in paths:
        if path.exists():
            return path
    return None


def select_session_dir(raw_dir: Path) -> Path:
    candidates = [
        child
        for child in raw_dir.iterdir()
        if child.is_dir() and (child / "static").is_dir()
    ]
    if not candidates:
        raise FileNotFoundError(f"No EPIC session directory found under {raw_dir}")
    return sorted(candidates, key=lambda p: p.name)[0]


def select_healthy_model(raw_dir: Path, subject: str) -> Path:
    models = sorted((raw_dir / "osimxml").glob("*.osim"))
    if not models:
        raise FileNotFoundError(f"No healthy .osim model found in {raw_dir / 'osimxml'}")
    preferred = [path for path in models if path.stem.upper() == subject]
    return preferred[0] if preferred else models[0]


def select_trial(session_dir: Path, task: str, requested: str, preferred: tuple[str, ...]) -> TrialSelection:
    marker_dir = session_dir / task / "markers"
    if not marker_dir.is_dir():
        raise FileNotFoundError(f"Missing marker directory: {marker_dir}")

    marker_files = sorted(marker_dir.glob("*.mat"))
    if not marker_files:
        raise FileNotFoundError(f"No marker .mat files in {marker_dir}")

    by_stem = {path.stem: path for path in marker_files}
    if requested != "auto":
        if requested not in by_stem:
            raise FileNotFoundError(f"Requested trial {requested!r} not found in {marker_dir}")
        selected = by_stem[requested]
    else:
        selected = None
        for name in preferred:
            if name in by_stem:
                selected = by_stem[name]
                break
        if selected is None:
            selected = marker_files[0]

    trial = selected.stem
    task_dir = session_dir / task
    return TrialSelection(
        task=task,
        trial=trial,
        marker_file=selected,
        fp_file=existing_or_none(task_dir / "fp" / f"{trial}.mat"),
        ik_file=existing_or_none(task_dir / "ik" / f"{trial}.mat"),
        id_file=existing_or_none(task_dir / "id" / f"{trial}.mat"),
    )


def existing_or_none(path: Path) -> Path | None:
    return path if path.is_file() else None


def discover_subject(
    models_dir: Path,
    subject: str,
    task: str,
    trial: str,
    static_trial: str,
) -> SubjectDiscovery:
    normalized = normalize_subject(subject)
    raw_dir = models_dir / f"{normalized}-raw"
    if not raw_dir.is_dir():
        raise FileNotFoundError(f"Subject raw directory not found: {raw_dir}")
    session_dir = select_session_dir(raw_dir)
    healthy_model = select_healthy_model(raw_dir, normalized)
    static_selection = select_trial(
        session_dir,
        "static",
        static_trial,
        ("static_01", "static01"),
    )
    operating_selection = select_trial(
        session_dir,
        task,
        trial,
        (f"{task}_01_01",),
    )
    return SubjectDiscovery(
        subject=normalized,
        raw_dir=raw_dir,
        session_dir=session_dir,
        healthy_model=healthy_model,
        static_trial=static_selection,
        operating_trial=operating_selection,
    )


def require_file(path: Path, label: str) -> Path:
    if not path.is_file():
        raise FileNotFoundError(f"{label} not found: {path}")
    return path.resolve()


def default_donor_model(models_dir: Path) -> Path:
    candidates = [
        models_dir / "AB06_SEASEA-raw" / "osimxml" / "AB06_SEASEA.osim",
        models_dir / "AB06_SEASEA-raw" / "osimxml" / "AB06_SEASEA_marker_calibrated.osim",
        models_dir / "AB06_SEASEA_Threadmill" / "AB06_SEASEA.osim",
        models_dir / "SEASEA" / "Adjusted_SEASEA - Copia_tuned.osim",
    ]
    found = choose_first_existing(candidates)
    if found is None:
        raise FileNotFoundError("Could not find a donor SEASEA model.")
    return found


def default_plugin() -> Path:
    if sys.platform == "darwin":
        candidates = [
            REPO_ROOT / "plugins" / "libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib",
            REPO_ROOT / "plugins" / "SEA_Plugin_BlackBox_mCMC_impedence.dylib",
        ]
    elif os.name == "nt":
        candidates = [
            REPO_ROOT / "plugins" / "SEA_Plugin_BlackBox_mCMC_impedence_ff.dll",
            REPO_ROOT / "plugins" / "SEA_Plugin_BlackBox_mCMC_impedence.dll",
        ]
    else:
        candidates = [
            REPO_ROOT / "plugins" / "libSEA_Plugin_BlackBox_mCMC_impedence_ff.so",
            REPO_ROOT / "plugins" / "SEA_Plugin_BlackBox_mCMC_impedence_ff.so",
        ]
    return choose_first_existing(candidates) or candidates[0]


def default_matlab() -> str:
    mac_matlab = Path("/Applications/MATLAB_R2024a.app/bin/matlab")
    if mac_matlab.is_file():
        return str(mac_matlab)
    return shutil.which("matlab") or "matlab"


def set_child_text(node: ET.Element, tag: str, value: str) -> None:
    child = node.find(tag)
    if child is None:
        child = ET.SubElement(node, tag)
    child.text = value


def get_float(node: ET.Element, tag: str) -> float:
    text = node.findtext(tag)
    if text is None:
        raise ValueError(f"Missing <{tag}> in {node.tag} {node.attrib.get('name', '')}")
    return float(text.strip())


def set_float(node: ET.Element, tag: str, value: float) -> None:
    set_child_text(node, tag, f"{value:.16g}")


def get_vec(node: ET.Element, tag: str) -> list[float]:
    text = node.findtext(tag)
    if text is None:
        raise ValueError(f"Missing <{tag}> in {node.tag} {node.attrib.get('name', '')}")
    return [float(part) for part in text.split()]


def set_vec(node: ET.Element, tag: str, values: Iterable[float]) -> None:
    set_child_text(node, tag, " ".join(f"{value:.16g}" for value in values))


def find_model(root: ET.Element) -> ET.Element:
    model = root.find("Model")
    if model is None:
        raise ValueError("OpenSim XML has no <Model> element.")
    return model


def set_objects(model: ET.Element, set_tag: str) -> ET.Element:
    set_node = model.find(set_tag)
    if set_node is None:
        raise ValueError(f"Model has no <{set_tag}>.")
    objects = set_node.find("objects")
    if objects is None:
        raise ValueError(f"<{set_tag}> has no <objects> child.")
    return objects


def find_named(objects: ET.Element, name: str) -> ET.Element:
    for child in list(objects):
        if child.attrib.get("name") == name:
            return child
    raise KeyError(name)


def find_index(objects: ET.Element, names: Iterable[str]) -> int:
    wanted = set(names)
    for index, child in enumerate(list(objects)):
        if child.attrib.get("name") in wanted:
            return index
    return len(list(objects))


def remove_named(objects: ET.Element, names: Iterable[str]) -> None:
    wanted = set(names)
    for child in list(objects):
        if child.attrib.get("name") in wanted:
            objects.remove(child)


def element_contains_any(node: ET.Element, needles: Iterable[str]) -> bool:
    text = ET.tostring(node, encoding="unicode")
    return any(needle in text for needle in needles)


def replace_text_refs(node: ET.Element, replacements: dict[str, str]) -> None:
    for elem in node.iter():
        if elem.text:
            for old, new in replacements.items():
                elem.text = elem.text.replace(old, new)


def total_model_mass(model: ET.Element) -> float:
    body_objects = set_objects(model, "BodySet")
    return sum(get_float(body, "mass") for body in body_objects if body.tag == "Body")


def scale_body_mass(body: ET.Element, new_mass: float) -> None:
    old_mass = get_float(body, "mass")
    set_float(body, "mass", new_mass)
    if old_mass <= 0:
        return
    ratio = new_mass / old_mass
    try:
        inertia = get_vec(body, "inertia")
    except ValueError:
        return
    set_vec(body, "inertia", [value * ratio for value in inertia])


def mass_preserving_targets(base_model: ET.Element, donor_model: ET.Element) -> tuple[dict[str, float], float]:
    base_bodies = set_objects(base_model, "BodySet")
    donor_bodies = set_objects(donor_model, "BodySet")
    removed = {name: get_float(find_named(base_bodies, name), "mass") for name in RAW_LEFT_BODIES}
    donor = {name: get_float(find_named(donor_bodies, name), "mass") for name in PROSTHETIC_BODIES}

    removed_total = sum(removed.values())
    femur_mass = removed["femur_l"]
    remaining = max(0.0, removed_total - femur_mass)
    donor_remaining = donor["osseo_pylon"] + donor["tibia_pylon"] + donor["foot_l"]

    targets = {"transfemur": femur_mass}
    if donor_remaining > 0:
        for name in ("osseo_pylon", "tibia_pylon", "foot_l"):
            targets[name] = remaining * donor[name] / donor_remaining
    else:
        equal = remaining / 3.0
        targets.update({"osseo_pylon": equal, "tibia_pylon": equal, "foot_l": equal})
    return targets, removed_total


def graft_bodyset(base_model: ET.Element, donor_model: ET.Element) -> float:
    base_objects = set_objects(base_model, "BodySet")
    donor_objects = set_objects(donor_model, "BodySet")
    targets, removed_total = mass_preserving_targets(base_model, donor_model)
    insert_at = find_index(base_objects, RAW_LEFT_BODIES)
    remove_named(base_objects, RAW_LEFT_BODIES)

    for offset, name in enumerate(PROSTHETIC_BODIES):
        body = deepcopy(find_named(donor_objects, name))
        scale_body_mass(body, targets[name])
        base_objects.insert(insert_at + offset, body)
    return removed_total


def subject_specific_hip_joint(base_model: ET.Element) -> ET.Element:
    joint_objects = set_objects(base_model, "JointSet")
    hip = deepcopy(find_named(joint_objects, "hip_l"))
    set_child_text(hip, "socket_child_frame", "transfemur_offset")
    frames = hip.find("frames")
    if frames is None:
        raise ValueError("hip_l has no <frames> block.")
    child_frame = None
    for frame in frames:
        if frame.attrib.get("name") == "femur_l_offset":
            child_frame = frame
            break
    if child_frame is None:
        raise ValueError("hip_l has no femur_l_offset frame.")
    child_frame.attrib["name"] = "transfemur_offset"
    set_child_text(child_frame, "socket_parent", "/bodyset/transfemur")
    replace_text_refs(
        hip,
        {
            "femur_l_offset": "transfemur_offset",
            "/bodyset/femur_l": "/bodyset/transfemur",
        },
    )
    return hip


def graft_jointset(base_model: ET.Element, donor_model: ET.Element) -> None:
    base_objects = set_objects(base_model, "JointSet")
    donor_objects = set_objects(donor_model, "JointSet")
    insert_at = find_index(base_objects, RAW_LEFT_JOINTS)
    hip = subject_specific_hip_joint(base_model)
    remove_named(base_objects, RAW_LEFT_JOINTS)
    base_objects.insert(insert_at, hip)
    for offset, name in enumerate(PROSTHETIC_JOINTS, start=1):
        base_objects.insert(insert_at + offset, deepcopy(find_named(donor_objects, name)))


def graft_forceset(base_model: ET.Element, donor_model: ET.Element) -> list[str]:
    base_objects = set_objects(base_model, "ForceSet")
    donor_objects = set_objects(donor_model, "ForceSet")
    removed: list[str] = []
    for force in list(base_objects):
        name = force.attrib.get("name", "")
        if (
            name.startswith("SEA_")
            or element_contains_any(force, REMOVED_BODY_REFS)
            or element_contains_any(force, REMOVED_COORD_REFS)
        ):
            removed.append(name)
            base_objects.remove(force)

    donor_additions = []
    for force in list(donor_objects):
        name = force.attrib.get("name", "")
        if name.startswith("SEA_") or element_contains_any(force, PROSTHETIC_REFS):
            donor_additions.append(deepcopy(force))
    for force in donor_additions:
        base_objects.append(force)
    return removed


def graft_markers(base_model: ET.Element) -> None:
    marker_objects = set_objects(base_model, "MarkerSet")
    for marker in marker_objects:
        name = marker.attrib.get("name")
        parent = LEFT_MARKER_PARENT_MAP.get(name or "")
        if parent is not None:
            set_child_text(marker, "socket_parent_frame", parent)


def assert_no_removed_refs(output_path: Path) -> None:
    text = output_path.read_text(encoding="utf-8", errors="replace")
    leftovers = [ref for ref in REMOVED_BODY_REFS if ref in text]
    leftovers += [coord for coord in REMOVED_COORD_REFS if coord in text]
    if leftovers:
        joined = ", ".join(sorted(set(leftovers)))
        raise RuntimeError(f"Grafted model still contains removed references: {joined}")


def graft_seasea_model(
    subject: str,
    healthy_model: Path,
    donor_model: Path,
    output_model: Path,
) -> tuple[float, float, float, list[str]]:
    base_tree = ET.parse(healthy_model)
    donor_tree = ET.parse(donor_model)
    base_model = find_model(base_tree.getroot())
    donor = find_model(donor_tree.getroot())

    mass_before = total_model_mass(base_model)
    base_model.attrib["name"] = f"{subject}_SEASEA"
    removed_mass = graft_bodyset(base_model, donor)
    graft_jointset(base_model, donor)
    removed_forces = graft_forceset(base_model, donor)
    graft_markers(base_model)
    mass_after = total_model_mass(base_model)

    output_model.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(base_tree, space="\t")
    base_tree.write(output_model, encoding="utf-8", xml_declaration=True)
    assert_no_removed_refs(output_model)
    return mass_before, mass_after, removed_mass, removed_forces


def matlab_quote(value: str | Path) -> str:
    return "'" + str(value).replace("'", "''") + "'"


def format_command(cmd: list[str]) -> str:
    parts = [str(part) for part in cmd]
    if os.name == "nt":
        return subprocess.list2cmdline(parts)
    return shlex.join(parts)


def run_logged(cmd: list[str], log_path: Path, cwd: Path = REPO_ROOT) -> int:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8", errors="replace") as handle:
        handle.write("$ " + format_command(cmd) + "\n\n")
        handle.flush()
        proc = subprocess.run(cmd, cwd=cwd, stdout=handle, stderr=subprocess.STDOUT, text=True)
    return proc.returncode


def run_matlab_convert(
    matlab: str,
    subject_root: Path,
    task: str,
    trial: str,
    output_dir: Path,
    model_file: Path,
    target_model: str,
    log_path: Path,
) -> None:
    batch = (
        f"cd({matlab_quote(REPO_ROOT)}); "
        "addpath('tools'); "
        "summary = convert_epic_ab06_to_opensim("
        f"'SubjectDir',{matlab_quote(subject_root)},"
        f"'Task',{matlab_quote(task)},"
        f"'Trial',{matlab_quote(trial)},"
        f"'OutputDir',{matlab_quote(output_dir)},"
        f"'TargetModel',{matlab_quote(target_model)},"
        f"'ModelFile',{matlab_quote(model_file)},"
        f"'IkTemplate',{matlab_quote(REPO_ROOT / 'models' / 'AB06_SEASEA-raw' / 'osimxml' / 'iksetup.xml')},"
        f"'IdTemplate',{matlab_quote(REPO_ROOT / 'models' / 'AB06_SEASEA-raw' / 'osimxml' / 'idsetup.xml')}"
        "); disp(summary)"
    )
    code = run_logged([matlab, "-batch", batch], log_path)
    if code != 0:
        raise RuntimeError(f"MATLAB conversion failed for {task}/{trial}; see {log_path}")


def run_matlab_healthy_overlay(
    matlab: str,
    subject: str,
    subject_root: Path,
    task: str,
    trial: str,
    output_dir: Path,
    log_path: Path,
) -> bool:
    if not (subject_root / task / "id" / f"{trial}.mat").is_file():
        return False
    batch = (
        f"cd({matlab_quote(REPO_ROOT)}); "
        "addpath('tools'); "
        "summary = export_ab06_healthy_overlay("
        f"'Subject',{matlab_quote(subject)},"
        f"'SourceRoot',{matlab_quote(subject_root)},"
        f"'Task',{matlab_quote(task)},"
        f"'Trial',{matlab_quote(trial)},"
        f"'OutputDir',{matlab_quote(output_dir)}"
        "); disp(summary)"
    )
    code = run_logged([matlab, "-batch", batch], log_path)
    if code != 0:
        raise RuntimeError(f"Healthy overlay export failed for {task}/{trial}; see {log_path}")
    return True


def converted_trial_dir(converted_root: Path, task: str, trial: str) -> Path:
    return converted_root / task / trial


def converted_trial_files(converted_root: Path, task: str, trial: str) -> dict[str, Path]:
    folder = converted_trial_dir(converted_root, task, trial)
    return {
        "dir": folder,
        "trc": folder / f"{trial}.trc",
        "grf": folder / f"{trial}_grf.mot",
        "external": folder / f"{trial}_ExternalLoads.xml",
        "ik_dataset": folder / f"{trial}_ik_dataset_ab06_seasea.mot",
    }


def run_marker_calibration(
    python_exe: str,
    model: Path,
    static_trc: Path,
    static_ik: Path,
    output_model: Path,
    report_csv: Path,
    plugin: Path,
    log_path: Path,
) -> None:
    cmd = [
        python_exe,
        str(REPO_ROOT / "tools" / "calibrate_ab06_seasea_markers.py"),
        "--model",
        str(model),
        "--trc",
        str(static_trc),
        "--ik",
        str(static_ik),
        "--output-model",
        str(output_model),
        "--report",
        str(report_csv),
        "--plugin",
        str(plugin),
    ]
    code = run_logged(cmd, log_path)
    if code != 0:
        raise RuntimeError(f"Marker calibration failed; see {log_path}")


def mean_marker_errors(report_csv: Path) -> tuple[float | None, float | None]:
    if not report_csv.is_file():
        return None, None
    old_values: list[float] = []
    new_values: list[float] = []
    with report_csv.open(newline="") as handle:
        for row in csv.DictReader(handle):
            if row.get("old_mean_error_m"):
                old_values.append(float(row["old_mean_error_m"]))
            if row.get("new_fit_rms_m"):
                new_values.append(float(row["new_fit_rms_m"]))
    old_mean = sum(old_values) / len(old_values) if old_values else None
    new_mean = sum(new_values) / len(new_values) if new_values else None
    return old_mean, new_mean


def range_label(start: float, end: float) -> str:
    return f"{start:.3f}_{end:.3f}".replace(".", "p").replace("-", "m")


def read_time_range(path: Path) -> tuple[float, float]:
    first: float | None = None
    last: float | None = None
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for raw in handle:
            stripped = raw.strip()
            if not stripped:
                continue
            try:
                value = float(stripped.split()[0])
            except ValueError:
                continue
            if first is None:
                first = value
            last = value
    if first is None or last is None:
        raise ValueError(f"Unable to read time range from {path}")
    return first, last


def run_ik_pipeline(
    python_exe: str,
    pipeline_root: Path,
    model: Path,
    trc: Path,
    grf: Path,
    support_data: Path,
    plugin: Path,
    full_range: tuple[float, float],
    log_path: Path,
) -> Path | None:
    start, end = full_range
    rra_end = min(end, start + 0.5)
    cmd = [
        python_exe,
        str(REPO_ROOT / "scripts" / "run_opensim_sea_pipeline.py"),
        "--pipeline-root",
        str(pipeline_root),
        "--model",
        str(model),
        "--marker-set",
        str(model),
        "--trc",
        str(trc),
        "--grf",
        str(grf),
        "--support-data",
        str(support_data),
        "--plugin",
        str(plugin),
        "--full-time-range",
        f"{start:.10g}",
        f"{end:.10g}",
        "--rra-time-range",
        f"{start:.10g}",
        f"{rra_end:.10g}",
        "--skip-smoke-test",
        "--dry-run",
    ]
    code = run_logged(cmd, log_path)
    if code != 0:
        return None
    label = range_label(start, end)
    candidates = sorted((pipeline_root / "02_ik").glob(f"**/IK_full_{label}.mot"))
    candidates += sorted((pipeline_root / "02_ik").glob("**/IK_full*.mot"))
    return candidates[0] if candidates else None


def run_rra_pipeline(
    python_exe: str,
    pipeline_root: Path,
    model: Path,
    trc: Path,
    grf: Path,
    support_data: Path,
    plugin: Path,
    full_range: tuple[float, float],
    log_path: Path,
) -> bool:
    start, end = full_range
    rra_end = min(end, start + 6.0)
    cmd = [
        python_exe,
        str(REPO_ROOT / "scripts" / "run_opensim_sea_pipeline.py"),
        "--pipeline-root",
        str(pipeline_root),
        "--model",
        str(model),
        "--marker-set",
        str(model),
        "--trc",
        str(trc),
        "--grf",
        str(grf),
        "--support-data",
        str(support_data),
        "--plugin",
        str(plugin),
        "--full-time-range",
        f"{start:.10g}",
        f"{end:.10g}",
        "--rra-time-range",
        f"{start:.10g}",
        f"{rra_end:.10g}",
        "--skip-smoke-test",
    ]
    return run_logged(cmd, log_path) == 0


def copy_required(source: Path, destination: Path) -> None:
    require_file(source, source.name)
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)


def write_external_forces_xml(path: Path, grf_filename: str) -> None:
    root = ET.Element("OpenSimDocument", Version="40000")
    external_loads = ET.SubElement(root, "ExternalLoads", name="externalloads")
    objects = ET.SubElement(external_loads, "objects")

    specs = [
        ("left_ground_force1", "foot_l", "ground_force1_v", "ground_force1_p", "ground_torque1_"),
        ("right_ground_force2", "calcn_r", "ground_force2_v", "ground_force2_p", "ground_torque2_"),
    ]
    for name, body, force_id, point_id, torque_id in specs:
        force = ET.SubElement(objects, "ExternalForce", name=name)
        for tag, text in [
            ("applied_to_body", body),
            ("force_expressed_in_body", "ground"),
            ("point_expressed_in_body", "ground"),
            ("force_identifier", force_id),
            ("point_identifier", point_id),
            ("torque_identifier", torque_id),
            ("data_source_name", grf_filename),
        ]:
            ET.SubElement(force, tag).text = text
    ET.SubElement(external_loads, "groups")
    ET.SubElement(external_loads, "datafile").text = grf_filename
    tree = ET.ElementTree(root)
    ET.indent(tree, space="\t")
    path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(path, encoding="utf-8", xml_declaration=True)


def write_setup_xml(path: Path, model: Path, kinematics: Path, external: Path, reserves: Path, start: float, end: float) -> None:
    root = ET.Element("OpenSimDocument", Version="40000")
    setup = ET.SubElement(root, "CMC_Simulator_Setup", name=path.stem)
    for tag, value in [
        ("model_file", repo_relative(model)),
        ("kinematics_file", repo_relative(kinematics)),
        ("external_loads_xml", repo_relative(external)),
        ("reserve_actuators_xml", repo_relative(reserves)),
        ("t_start", f"{start:.10g}"),
        ("t_end", f"{end:.10g}"),
    ]:
        ET.SubElement(setup, tag).text = value
    tree = ET.ElementTree(root)
    ET.indent(tree, space="\t")
    path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(path, encoding="utf-8", xml_declaration=True)


def build_bundle(
    subject: str,
    task: str,
    calibrated_model: Path,
    ik_source: Path,
    grf_source: Path,
    support_data: Path,
    bundle_dir: Path,
    setup_start: float,
    setup_end: float,
) -> dict[str, Path]:
    data_dir = bundle_dir / "data"
    model_out = bundle_dir / f"{subject}_SEASEA.osim"
    ik_out = data_dir / f"IK_results_{subject}_SEASEA.mot"
    grf_out = data_dir / f"{subject}_SEASEA_GRF_FullSpan.mot"
    external_out = data_dir / "ExternalForces.xml"
    actuators_out = data_dir / "CMC_Actuators.xml"
    tasks_out = data_dir / "CMC_Tasks - modified Kp_Kv.xml"
    setup_out = bundle_dir / f"{subject}_SEASEA_setup.xml"

    bundle_dir.mkdir(parents=True, exist_ok=True)
    copy_required(calibrated_model, model_out)
    copy_required(ik_source, ik_out)
    copy_required(grf_source, grf_out)
    write_external_forces_xml(external_out, grf_out.name)
    copy_required(support_data / "CMC_Actuators.xml", actuators_out)
    copy_required(support_data / "CMC_Tasks - modified Kp_Kv.xml", tasks_out)
    write_setup_xml(setup_out, model_out, ik_out, external_out, actuators_out, setup_start, setup_end)
    return {
        "bundle": bundle_dir,
        "model": model_out,
        "kinematics": ik_out,
        "grf": grf_out,
        "external": external_out,
        "actuators": actuators_out,
        "tasks": tasks_out,
        "setup": setup_out,
    }


def run_smoke(python_exe: str, setup_xml: Path, output_dir: Path, start: float, end: float, log_path: Path) -> int:
    cmd = [
        python_exe,
        str(REPO_ROOT / "main.py"),
        "--setup",
        str(setup_xml),
        "--output-dir",
        str(output_dir),
        "--t-start",
        f"{start:.10g}",
        "--t-end",
        f"{end:.10g}",
        "--log",
    ]
    return run_logged(cmd, log_path)


def write_report_files(report: BuildReport, bundle_dir: Path) -> None:
    json_path = bundle_dir / "pipeline_report.json"
    md_path = bundle_dir / "pipeline_report.md"
    json_path.write_text(json.dumps(asdict(report), indent=2), encoding="utf-8")

    lines = [
        f"# {report.subject}_SEASEA Pipeline Report",
        "",
        f"Generated: {report.generated_at}",
        f"Status: {report.status}",
        "",
        "## Inputs",
        "",
        f"- Subject: `{report.subject}`",
        f"- Task/trial: `{report.task}/{report.trial}`",
        f"- Static trial: `{report.static_trial}`",
        f"- Healthy model: `{report.healthy_model}`",
        "",
        "## Outputs",
        "",
        f"- Bundle: `{report.bundle_dir}`",
        f"- Setup: `{report.setup_xml}`",
        f"- Kinematics: `{report.ik_source}`",
        f"- GRF: `{report.grf_source}`",
        "",
        "## Diagnostics",
        "",
        f"- Mass before: `{report.mass_before_kg}` kg",
        f"- Mass after: `{report.mass_after_kg}` kg",
        f"- Removed left mass: `{report.removed_left_mass_kg}` kg",
        f"- Marker old mean error: `{report.marker_old_mean_error_m}` m",
        f"- Marker calibrated RMS: `{report.marker_new_fit_rms_m}` m",
        f"- Kinematics range: `{report.kinematics_time_range}` s",
        f"- GRF range: `{report.grf_time_range}` s",
        f"- Smoke return code: `{report.smoke_returncode}`",
    ]
    if report.notes:
        lines += ["", "## Notes", ""]
        lines += [f"- {note}" for note in report.notes]
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def validate_discovery(
    discovery: SubjectDiscovery,
    donor_model: Path,
    plugin: Path,
    matlab: str,
    python_exe: str,
    support_data: Path,
) -> dict[str, object]:
    return {
        "subject": discovery.subject,
        "raw_dir": repo_relative(discovery.raw_dir),
        "session_dir": repo_relative(discovery.session_dir),
        "healthy_model": repo_relative(discovery.healthy_model),
        "static_trial": discovery.static_trial.trial,
        "operating_trial": f"{discovery.operating_trial.task}/{discovery.operating_trial.trial}",
        "static_has_ik": discovery.static_trial.ik_file is not None,
        "operating_has_fp": discovery.operating_trial.fp_file is not None,
        "operating_has_ik": discovery.operating_trial.ik_file is not None,
        "operating_has_id": discovery.operating_trial.id_file is not None,
        "donor_model": repo_relative(donor_model),
        "plugin": repo_relative(plugin) if plugin.exists() else str(plugin),
        "plugin_exists": plugin.exists(),
        "matlab": matlab,
        "python": python_exe,
        "python_has_opensim": python_has_opensim(python_exe),
        "support_data": repo_relative(support_data),
        "support_data_exists": support_data.is_dir(),
    }


def python_has_opensim(python_exe: str) -> bool:
    proc = subprocess.run(
        [python_exe, "-c", "import opensim"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    return proc.returncode == 0


def require_python_opensim(python_exe: str) -> None:
    if not python_has_opensim(python_exe):
        raise RuntimeError(
            "The selected Python cannot import opensim. "
            "Pass --python pointing to the OpenSim Python environment."
        )


def ensure_overwrite_policy(paths: Iterable[Path], force: bool) -> None:
    existing = [path for path in paths if path.exists()]
    if existing and not force:
        formatted = "\n".join(f"  - {path}" for path in existing[:8])
        raise FileExistsError(
            "Output paths already exist. Re-run with --force to overwrite/update:\n"
            f"{formatted}"
        )


def build_subject(discovery: SubjectDiscovery, args: argparse.Namespace, donor_model: Path, plugin: Path, support_data: Path) -> BuildReport:
    subject = discovery.subject
    task = discovery.operating_trial.task
    trial = discovery.operating_trial.trial
    task_title = task[:1].upper() + task[1:]
    raw_out = args.output_models_dir / f"{subject}_SEASEA-raw"
    bundle_dir = args.output_models_dir / f"{subject}_SEASEA_{task_title}"
    results_prefix = f"{subject.lower()}_seasea_{task}_{trial}"
    logs_dir = args.results_dir / results_prefix / "logs"

    report = BuildReport(
        subject=subject,
        task=task,
        trial=trial,
        static_trial=discovery.static_trial.trial,
        raw_dir=repo_relative(discovery.raw_dir),
        session_dir=repo_relative(discovery.session_dir),
        healthy_model=repo_relative(discovery.healthy_model),
        bundle_dir=repo_relative(bundle_dir),
    )

    ensure_overwrite_policy(
        [
            raw_out / "osimxml" / f"{subject}_SEASEA.osim",
            raw_out / "osimxml" / f"{subject}_SEASEA_marker_calibrated.osim",
            bundle_dir,
        ],
        args.force,
    )

    graft_model = raw_out / "osimxml" / f"{subject}_SEASEA.osim"
    calibrated_model = raw_out / "osimxml" / f"{subject}_SEASEA_marker_calibrated.osim"
    marker_report = calibrated_model.with_suffix(".marker_calibration.csv")
    converted_root = raw_out / "data" / "converted"

    mass_before, mass_after, removed_mass, removed_forces = graft_seasea_model(
        subject,
        discovery.healthy_model,
        donor_model,
        graft_model,
    )
    report.graft_model = repo_relative(graft_model)
    report.mass_before_kg = mass_before
    report.mass_after_kg = mass_after
    report.removed_left_mass_kg = removed_mass
    report.notes.append(f"Removed/replaced {len(removed_forces)} healthy left-side force elements during graft.")

    run_matlab_convert(
        args.matlab,
        discovery.session_dir,
        "static",
        discovery.static_trial.trial,
        converted_root,
        graft_model,
        f"{subject}_SEASEA",
        logs_dir / "convert_static.log",
    )
    run_matlab_convert(
        args.matlab,
        discovery.session_dir,
        task,
        trial,
        converted_root,
        graft_model,
        f"{subject}_SEASEA",
        logs_dir / "convert_trial.log",
    )

    static_files = converted_trial_files(converted_root, "static", discovery.static_trial.trial)
    trial_files = converted_trial_files(converted_root, task, trial)
    report.converted_static_dir = repo_relative(static_files["dir"])
    report.converted_trial_dir = repo_relative(trial_files["dir"])

    run_marker_calibration(
        args.python,
        graft_model,
        static_files["trc"],
        static_files["ik_dataset"],
        calibrated_model,
        marker_report,
        plugin,
        logs_dir / "marker_calibration.log",
    )
    report.calibrated_model = repo_relative(calibrated_model)
    old_marker, new_marker = mean_marker_errors(marker_report)
    report.marker_old_mean_error_m = old_marker
    report.marker_new_fit_rms_m = new_marker

    kinematics_range = read_time_range(trial_files["ik_dataset"])
    grf_range = read_time_range(trial_files["grf"])
    report.kinematics_time_range = [kinematics_range[0], kinematics_range[1]]
    report.grf_time_range = [grf_range[0], grf_range[1]]

    ik_source = trial_files["ik_dataset"]
    if not args.skip_opensim_ik:
        ik_pipeline_root = args.results_dir / f"{results_prefix}_ik"
        report.pipeline_root = repo_relative(ik_pipeline_root)
        open_ik = run_ik_pipeline(
            args.python,
            ik_pipeline_root,
            calibrated_model,
            trial_files["trc"],
            trial_files["grf"],
            support_data,
            plugin,
            kinematics_range,
            logs_dir / "opensim_ik_pipeline.log",
        )
        if open_ik is not None:
            ik_source = open_ik
        else:
            report.notes.append("OpenSim IK dry-run failed or did not produce IK_full*.mot; using converted dataset IK.")

    if args.run_rra:
        rra_root = args.results_dir / f"{results_prefix}_rra"
        report.rra_pipeline_root = repo_relative(rra_root)
        ok = run_rra_pipeline(
            args.python,
            rra_root,
            calibrated_model,
            trial_files["trc"],
            trial_files["grf"],
            support_data,
            plugin,
            kinematics_range,
            logs_dir / "rra_pipeline.log",
        )
        report.notes.append("RRA diagnostic completed." if ok else "RRA diagnostic failed; see logs.")

    setup_start = kinematics_range[0]
    setup_end = min(kinematics_range[1], setup_start + args.default_duration)
    bundle_files = build_bundle(
        subject,
        task,
        calibrated_model,
        ik_source,
        trial_files["grf"],
        support_data,
        bundle_dir,
        setup_start,
        setup_end,
    )
    report.setup_xml = repo_relative(bundle_files["setup"])
    report.ik_source = repo_relative(bundle_files["kinematics"])
    report.grf_source = repo_relative(bundle_files["grf"])

    healthy_dir = bundle_dir / "data" / "healthy"
    exported_healthy = run_matlab_healthy_overlay(
        args.matlab,
        subject,
        discovery.session_dir,
        task,
        trial,
        healthy_dir,
        logs_dir / "healthy_overlay.log",
    )
    if exported_healthy:
        report.healthy_overlay_dir = repo_relative(healthy_dir)
    else:
        report.notes.append("Healthy overlay skipped because the selected trial has no ID .mat file.")

    if args.skip_smoke_test:
        report.notes.append("Smoke test skipped by user flag.")
    else:
        smoke_start = setup_start
        smoke_end = min(setup_end, smoke_start + args.smoke_duration)
        smoke_output = args.results_dir / f"{results_prefix}_smoke"
        report.smoke_output_dir = repo_relative(smoke_output)
        report.smoke_returncode = run_smoke(
            args.python,
            bundle_files["setup"],
            smoke_output,
            smoke_start,
            smoke_end,
            logs_dir / "smoke_test.log",
        )
        if report.smoke_returncode != 0:
            report.notes.append("Smoke test failed; inspect smoke_test.log and simulator output.")

    report.status = "complete" if report.smoke_returncode in (None, 0) else "complete_with_smoke_failure"
    write_report_files(report, bundle_dir)
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subject_group = parser.add_mutually_exclusive_group(required=True)
    subject_group.add_argument("--subject", help="Subject id, e.g. AB07.")
    subject_group.add_argument("--all-subjects", action="store_true", help="Process every local models/ABxx-raw subject.")
    parser.add_argument("--task", choices=TASK_CHOICES, default="treadmill")
    parser.add_argument("--trial", default="auto")
    parser.add_argument("--static-trial", default="auto")
    parser.add_argument("--models-dir", type=Path, default=DEFAULT_MODELS_DIR)
    parser.add_argument("--output-models-dir", type=Path, default=DEFAULT_MODELS_DIR)
    parser.add_argument("--results-dir", type=Path, default=DEFAULT_RESULTS_DIR)
    parser.add_argument("--donor-model", type=Path)
    parser.add_argument("--support-data", type=Path, default=DEFAULT_MODELS_DIR / "SEASEA" / "data")
    parser.add_argument("--plugin", type=Path, default=default_plugin())
    parser.add_argument("--matlab", default=default_matlab())
    parser.add_argument("--python", default=sys.executable, help="Python executable with OpenSim available for calibration/pipeline/smoke.")
    parser.add_argument("--side", choices=("left",), default="left")
    parser.add_argument("--run-rra", action="store_true")
    parser.add_argument("--skip-opensim-ik", action="store_true", help="Use converted dataset IK instead of an OpenSim IK dry-run.")
    parser.add_argument("--skip-smoke-test", action="store_true")
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument("--default-duration", type=float, default=9.01, help="Default setup duration from trial start.")
    parser.add_argument("--smoke-duration", type=float, default=0.03, help="Short simulator smoke-test duration.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.models_dir = args.models_dir.resolve()
    args.output_models_dir = args.output_models_dir.resolve()
    args.results_dir = args.results_dir.resolve()
    args.support_data = args.support_data.resolve()
    args.plugin = args.plugin.resolve()
    donor_model = (args.donor_model.resolve() if args.donor_model else default_donor_model(args.models_dir).resolve())

    subjects = discover_subjects(args.models_dir) if args.all_subjects else [normalize_subject(args.subject)]
    if not subjects:
        raise SystemExit("No local ABxx-raw subjects found.")

    discoveries = [
        discover_subject(args.models_dir, subject, args.task, args.trial, args.static_trial)
        for subject in subjects
    ]

    if args.validate_only:
        payload = [
            validate_discovery(discovery, donor_model, args.plugin, args.matlab, args.python, args.support_data)
            for discovery in discoveries
        ]
        print(json.dumps(payload[0] if len(payload) == 1 else payload, indent=2))
        return 0

    require_file(donor_model, "Donor SEASEA model")
    require_file(args.plugin, "SEA plugin")
    require_python_opensim(args.python)
    if not args.support_data.is_dir():
        raise FileNotFoundError(f"Support data directory not found: {args.support_data}")

    failures = 0
    reports: list[BuildReport] = []
    for discovery in discoveries:
        try:
            report = build_subject(discovery, args, donor_model, args.plugin, args.support_data)
            reports.append(report)
            print(f"[build_abx] Complete: {report.subject} -> {report.bundle_dir}")
        except Exception as exc:
            failures += 1
            print(f"[build_abx] ERROR for {discovery.subject}: {exc}", file=sys.stderr)
            if not args.all_subjects:
                raise

    if reports:
        summary_path = args.results_dir / "abx_seasea_last_build_summary.json"
        summary_path.parent.mkdir(parents=True, exist_ok=True)
        summary_path.write_text(json.dumps([asdict(report) for report in reports], indent=2), encoding="utf-8")
        print(f"[build_abx] Summary: {repo_relative(summary_path)}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
