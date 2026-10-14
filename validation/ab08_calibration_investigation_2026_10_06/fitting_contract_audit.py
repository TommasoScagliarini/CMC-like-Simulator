"""Read-only XML/IK-task audit of the prepared AB08 bundle; no solver calls."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
from xml.etree import ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from fit_seasea_subject import _canonical
from build_abx_seasea_pipeline import subject_specific_hip_joint


def objects(model, set_name):
    return {node.get("name"): node for node in model.findall(set_name + "/objects/*")}


def physical_force(node):
    node = deepcopy(node)
    for parent in node.iter():
        for child in list(parent):
            if child.tag == "Appearance":
                parent.remove(child)
        if parent.tag == "GeometryPath":
            parent.set("name", "path")
    return _canonical(node)


def error_stats(path):
    lines = path.read_text().splitlines()
    end = next(i for i, line in enumerate(lines) if line.strip().lower() == "endheader")
    labels = lines[end + 1].split()
    data = np.loadtxt(lines[end + 2:])
    rms = data[:, labels.index("marker_error_RMS")]
    maximum = data[:, labels.index("marker_error_max")]
    return {
        "file": str(path.relative_to(ROOT)), "rows": len(data),
        "mean_frame_rms_mm": float(np.mean(rms) * 1000),
        "median_frame_rms_mm": float(np.median(rms) * 1000),
        "p95_frame_rms_mm": float(np.quantile(rms, .95) * 1000),
        "max_frame_rms_mm": float(np.max(rms) * 1000),
        "max_single_marker_error_mm": float(np.max(maximum) * 1000),
        "max_single_marker_error_time_s": float(data[np.argmax(maximum), 0]),
    }


def main():
    paths = {
        "healthy": ROOT / "models/AB08-raw/osimxml/AB08.osim",
        "anatomical": ROOT / "models/AB08_SEASEA-raw/osimxml/AB08_SEASEA_anatomical.osim",
        "registered": ROOT / "models/AB08_SEASEA-raw/osimxml/AB08_SEASEA.osim",
        "calibrated": ROOT / "models/AB08_SEASEA_Threadmill/AB08_SEASEA_stiff321_500_pi.osim",
        "donor": ROOT / "results/abx_reference_rrmf81lr/donor_model/AB06_SEASEA_stiff321_500_pi.osim",
    }
    models = {name: ET.parse(path).getroot().find("Model") for name, path in paths.items()}
    healthy, final = models["healthy"], models["calibrated"]
    checks = {}
    old_body, new_body = objects(healthy, "BodySet"), objects(final, "BodySet")
    common_body = sorted(set(old_body) & set(new_body))
    checks["biological_bodies_exactly_preserved"] = all(
        _canonical(old_body[name]) == _canonical(new_body[name]) for name in common_body)
    old_joint, new_joint = objects(healthy, "JointSet"), objects(final, "JointSet")
    common_joint = sorted((set(old_joint) & set(new_joint)) - {"hip_l"})
    checks["biological_joints_exactly_preserved"] = all(
        _canonical(old_joint[name]) == _canonical(new_joint[name]) for name in common_joint)
    checks["subject_hip_preserved_except_transfemur_rename"] = (
        _canonical(subject_specific_hip_joint(healthy)) == _canonical(new_joint["hip_l"]))
    old_marker, new_marker = objects(healthy, "MarkerSet"), objects(final, "MarkerSet")
    calibrated_names = [
        "L_Thigh_Upper", "L_Thigh_Front", "L_Thigh_Rear", "L_Knee_Lat",
        "L_Shank_Upper", "L_Shank_Front", "L_Shank_Rear", "L_Ankle_Lat",
        "L_Heel", "L_Toe_Lat", "L_Toe_Med", "L_Toe_Tip",
    ]
    untouched_names = sorted(set(old_marker) - set(calibrated_names))
    checks["all_16_nonprosthetic_markers_preserved"] = len(untouched_names) == 16 and all(
        _canonical(old_marker[name]) == _canonical(new_marker[name]) for name in untouched_names)
    old_force, new_force = objects(healthy, "ForceSet"), objects(final, "ForceSet")
    retained_forces = sorted(name for name in old_force if name in new_force
                             and (name.endswith("_r") or name.startswith(("ercspn", "intobl", "extobl"))))
    checks["right_and_torso_muscle_physics_preserved"] = all(
        physical_force(old_force[name]) == physical_force(new_force[name]) for name in retained_forces)
    donor_force = objects(models["donor"], "ForceSet")
    sea_names = ["SEA_Knee", "SEA_Ankle"]
    checks["sea_exactly_matches_AB06"] = all(
        _canonical(new_force[name]) == _canonical(donor_force[name]) for name in sea_names)
    donor_joint = objects(models["donor"], "JointSet")
    for name in ("osseo", "pros_knee", "pros_ankle"):
        actual, expected = deepcopy(new_joint[name]), deepcopy(donor_joint[name])
        if name in ("osseo", "pros_ankle"):
            for node in (actual, expected):
                frame_name = node.findtext("socket_parent_frame")
                node.find(f"frames/PhysicalOffsetFrame[@name='{frame_name}']/translation").text = "0 0 0"
        checks[name + "_hardware_axes_and_ranges_preserved"] = _canonical(actual) == _canonical(expected)
    setup_path = ROOT / "results/ab08_seasea_treadmill_treadmill_01_01_ik/02_ik/IK_full_8p630_151p525_setup.xml"
    tool = ET.parse(setup_path).getroot().find("InverseKinematicsTool")
    tasks = tool.findall("IKTaskSet/objects/*")
    checks["28_marker_tasks_match_model"] = len(tasks) == 28 and set(t.get("name") for t in tasks) == set(new_marker)
    checks["all_tasks_active_unit_weight"] = all(t.tag == "IKMarkerTask" and
        t.findtext("apply") == "true" and float(t.findtext("weight")) == 1.0 for t in tasks)
    checks["no_coordinate_tracking_constraints"] = not tool.findtext("coordinate_file", "").strip()
    report = json.loads((ROOT / "models/AB08_SEASEA_Threadmill/pipeline_report.json").read_text())
    checks["AB08_static_used_for_marker_calibration"] = (
        "models/AB08_SEASEA-raw/data/converted/static/static01" == report["converted_static_dir"] and
        "/models/AB08_SEASEA-raw/" in report["ground_registration"]["sources"]["input_static_motion"]["path"])
    stats = {
        "AB08": error_stats(ROOT / "results/ab08_seasea_treadmill_treadmill_01_01_ik/02_ik/full/IK_full_8p630_151p525_ik_marker_errors.sto"),
        "AB07_ground_registered": error_stats(ROOT / "results/ab07_ground_registered_2026_10_06/ab07_seasea_treadmill_treadmill_01_01_ik/02_ik/full/IK_full_10p360_153p415_ik_marker_errors.sto"),
    }
    payload = {
        "instruction_check_token": "CMC_AGENT_OK_2026", "status": "PASS" if all(checks.values()) else "FAIL",
        "scope": "XML fitting and marker-IK setup contracts, saved native marker-error statistics only; no solver/dynamics/policy",
        "sources": {name: {"path": str(path.relative_to(ROOT)), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
                    for name, path in paths.items()},
        "checks": checks, "biological_bodies_preserved": common_body,
        "biological_joints_preserved": common_joint,
        "nonprosthetic_markers_preserved": untouched_names,
        "right_and_torso_muscle_physics_preserved": retained_forces,
        "static_calibrated_markers": calibrated_names,
        "IK": {"task_count": len(tasks), "weight": 1.0, "accuracy": tool.findtext("accuracy"),
               "constraint_weight": tool.findtext("constraint_weight"), "coordinate_file": tool.findtext("coordinate_file", "")},
        "full_trial_comparison": stats,
        "static_calibration_mean_local_fit_rms_mm": report["marker_new_fit_rms_m"] * 1000,
        "interpretation": [
            "AB08 static marker fitting and registered hardware use subject-specific AB08 data, with AB06 used only as shared geometric/hardware reference.",
            "R_Thigh_Upper belongs to the unchanged healthy right femur and is outside the 12-marker static prosthetic calibration.",
            "AB08 mean frame RMS is about 1.62 mm above corrected AB07, but its maximum single-marker error is lower than corrected AB07; global mean and isolated peak measure different things.",
            "Static local-fit RMS does not certify gait marker fit: the same calibration samples estimate the marker locations, and prosthetic PinJoint topology differs from healthy knee/ankle/foot kinematics.",
        ],
        "limitations": [
            "No full-trial AB06 native marker-error companion exists in the local repository; no direct AB06 full-trial numerical comparison inferred.",
            "This audit proves preparation/setup contracts, not clinical acceptability of 12 mm mean or 115 mm isolated peak.",
            "Left proximal muscles are grafted from AB06 donor by the existing builder convention; this does not determine marker IK residuals because IK does not recruit muscles.",
        ],
    }
    destination = Path(__file__).with_suffix(".json")
    destination.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"status": payload["status"], "checks": checks, "output": str(destination)}))


if __name__ == "__main__":
    main()
