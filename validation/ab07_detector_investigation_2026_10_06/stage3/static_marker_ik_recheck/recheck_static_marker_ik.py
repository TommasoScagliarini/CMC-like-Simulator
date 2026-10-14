"""Short native marker IK and frozen production detector check on staged AB07.

Adapted from TG/validation/AB07/replay_static_marker_ik.py. All outputs stay
beside this script or in the adjacent receipt; no forward dynamics or policy.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import sys
from xml.etree import ElementTree as ET

import numpy as np
import opensim as osim

OUTPUT = Path(__file__).resolve().parent
REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO))
from scripts.run_opensim_sea_pipeline import build_ik_setup, default_plugin
from tools.calibrate_ab06_seasea_markers import (
    from_vec3, interpolate_motion, load_plugin, parse_motion, parse_trc, set_state_from_motion, vec3,
)
from tools.register_seasea_ground import _vertices
from binary_phase_detector import BinaryPhaseDetectorSampler, load_binary_phase_detector_profile


def fp(path: Path) -> dict:
    return {"path": str(path.relative_to(REPO)), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def table(path: Path):
    lines = path.read_text().splitlines()
    end = next(i for i, line in enumerate(lines) if line.strip().lower() == "endheader")
    rows = [line for line in lines[end+1:] if line.strip()]
    return rows[0].split(), np.loadtxt(rows[1:], ndmin=2)


def evaluate(model, motion, times, profile, vertices, target):
    state = model.initSystem()
    foot = model.getBodySet().get("foot_l")
    sampler = BinaryPhaseDetectorSampler(model, load_binary_phase_detector_profile(PROFILE))
    origin = np.asarray(profile["ground"]["origin"])
    normal = np.asarray(profile["ground"]["normal"])
    clearances, contacts, heights, pitches, values = [], [], [], [], []
    parity_max = 0.0
    for time in times:
        state.setTime(float(time))
        q = interpolate_motion(motion, float(time))
        set_state_from_motion(model, state, q, motion.in_degrees)
        reading = sampler.sample(state, float(time))
        clearances.append([reading.signed_clearance_m[name] for name in ("left_heel", "left_toe")])
        contacts.append([reading.contacts[name] for name in ("left_heel", "left_toe")])
        # Independent body-station transform of the actual staged hardware.
        zero = from_vec3(foot.findStationLocationInGround(state, osim.Vec3(0, 0, 0)))
        columns = [from_vec3(foot.findStationLocationInGround(state, vec3(axis))) - zero for axis in np.eye(3)]
        rotation = np.column_stack(columns)
        heights.append(float(np.min((vertices @ rotation.T + zero - origin) @ normal)))
        pitches.append(math.atan2(float(normal @ rotation[:, 0]), float(normal @ rotation[:, 1])))
        for point in profile["points"]:
            frame = osim.PhysicalFrame.safeDownCast(model.getComponent(point["frame"]))
            station = from_vec3(frame.findStationLocationInGround(state, vec3(point["location"])))
            manual = float((station-origin) @ normal)
            parity_max = max(parity_max, abs(manual-reading.signed_clearance_m[point["name"]]))
            if bool(manual <= 0) != reading.contacts[point["name"]]:
                raise AssertionError("Production contact bit differs from independent station projection")
        values.append([q[name] for name in motion.labels[1:]])
    clearance, bits = np.asarray(clearances), np.asarray(contacts)
    height = float(np.median(heights))
    pitch = float(np.median(np.unwrap(pitches)))
    summary = {
        "samples": len(times), "time_range_s": [float(times[0]), float(times[-1])],
        "binary_points": {name: {"mean_clearance_m": float(clearance[:, i].mean()),
            "min_clearance_m": float(clearance[:, i].min()), "max_clearance_m": float(clearance[:, i].max()),
            "contact_fraction": float(bits[:, i].mean())} for i, name in enumerate(("left_heel", "left_toe"))},
        "median_min_stl_clearance_m": height,
        "stl_target_residual_m": height-target["median_min_stl_clearance_m"],
        "median_sagittal_pitch_rad": pitch,
        "median_sagittal_pitch_deg": math.degrees(pitch),
        "pitch_target_residual_rad": pitch-target["median_sagittal_pitch_rad"],
        "pitch_target_residual_deg": math.degrees(pitch-target["median_sagittal_pitch_rad"]),
        "production_sampler_parity": {"poses": len(times), "points_per_pose": 2,
            "clearance_max_error_m": parity_max, "contacts_identical": True}
    }
    if parity_max > 1e-12:
        raise AssertionError("Production sampler clearance parity failed")
    return summary, np.asarray(values), clearance


STAGE = REPO / "validation/ab07_detector_investigation_2026_10_06/stage3"
STAGED_MODELS = STAGE / "build/models"
MODEL = STAGED_MODELS / "AB07_SEASEA_Threadmill/AB07_SEASEA_stiff321_500_pi.osim"
TRC = STAGED_MODELS / "AB07_SEASEA-raw/data/converted/static/static01/static01.trc"
REGISTERED = MODEL.parent / "data/fitting/AB07_SEASEA_static_ground_registered.mot"
REGISTRATION_MANIFEST = MODEL.parent / "data/fitting/ground_registration.json"
PROFILE = REPO / "validation/binary_phase_detector_v25_geometry_runs/2026-08-04_local_reach_sweep_dev02_04_08/selected_candidate_profile.json"
MESH = REPO / "Geometry/AM_foot_l.STL"


def main():
    target_receipt = STAGE / "static_marker_ik_recheck.json"
    motion_path = OUTPUT / "static_marker_ik_0p04s.mot"
    if target_receipt.exists() or motion_path.exists():
        raise FileExistsError("Static recheck output exists; this diagnostic never overwrites it")
    plugin = default_plugin()
    sources = {"script": Path(__file__), "model": MODEL, "static_trc": TRC, "registered_static_motion": REGISTERED,
               "registration_manifest": REGISTRATION_MANIFEST, "binary_profile": PROFILE, "plugin": plugin,
               "foot_mesh": MESH, "ik_setup_writer": REPO / "scripts/run_opensim_sea_pipeline.py",
               "production_binary_sampler": REPO / "binary_phase_detector.py",
               "calibration_motion_reader": REPO / "tools/calibrate_ab06_seasea_markers.py",
               "stl_reader": REPO / "tools/register_seasea_ground.py",
               "previous_static_ik_helper": REPO / "Trajectory Generator/validation/AB07/replay_static_marker_ik.py"}
    before = {name: fp(path) for name, path in sources.items()}
    # Native default logs and any unexpected relative diagnostic output also
    # remain in the explicitly authorized directory.
    os.chdir(OUTPUT)
    osim.Logger.removeFileSink()
    osim.Logger.addFileSink(str(OUTPUT / "opensim.log"))
    osim.Logger.setLevelString("error")
    load_plugin(plugin)
    trc = parse_trc(TRC)
    if len(trc.markers) != 28:
        raise AssertionError("Expected the same 28 static marker tasks")
    setup = build_ik_setup(OUTPUT / "ik_setup.xml", MODEL, TRC, list(trc.markers), OUTPUT, str(motion_path), 0.0, 0.04, 1.0)
    task_xml = ET.parse(setup)
    tasks = task_xml.findall(".//IKMarkerTask")
    if len(tasks) != 28 or any(float(task.findtext("weight")) != 1 for task in tasks):
        raise AssertionError("Marker tasks differ from the earlier static replay")
    tool = osim.InverseKinematicsTool(str(setup))
    if tool.run() is False:
        raise RuntimeError("Short static marker IK failed")
    motion, registered = parse_motion(motion_path), parse_motion(REGISTERED)
    np.testing.assert_allclose(motion.times, np.arange(9)*0.005, rtol=0, atol=1e-12)
    profile = json.loads(PROFILE.read_text())
    registration = json.loads(REGISTRATION_MANIFEST.read_text())
    target = registration["reference"]
    vertices = _vertices(MESH)
    model = osim.Model(str(MODEL))
    observed, solved_q, solved_clearance = evaluate(model, motion, motion.times, profile, vertices, target)
    baseline, baseline_q, baseline_clearance = evaluate(model, registered, motion.times, profile, vertices, target)
    if set(motion.labels[1:]) != set(registered.labels[1:]) or motion.in_degrees != registered.in_degrees:
        raise AssertionError("Solved and registered coordinate schemas or units differ")
    baseline_q = baseline_q[:, [registered.labels[1:].index(name) for name in motion.labels[1:]]]
    full_baseline, _, _ = evaluate(model, registered, registered.times, profile, vertices, target)
    error_path = OUTPUT / "static_marker_ik_0p04s_ik_marker_errors.sto"
    labels, errors = table(error_path)
    statistics = dict(zip(labels[1:], errors[:, 1:].mean(axis=0).tolist()))
    if any(fp(path)["sha256"] != before[name]["sha256"] for name, path in sources.items()):
        raise RuntimeError("A source changed during the diagnostic")
    if observed["binary_points"]["left_heel"]["max_clearance_m"] >= 0:
        raise AssertionError("Registered negative heel clearance was not preserved after marker IK")
    receipt = {
        "schema_version": 1,
        "scope": "Native inverse kinematics over 0-0.04 s, then position-only production sampling; no forward integration, policy or operational writes",
        "sources": before, "source_files_unchanged": True,
        "ik_settings": {"markers": list(trc.markers), "marker_count": 28, "weight": 1.0,
            "constraint_weight": "Inf", "accuracy": 1e-5, "samples": 9, "time_range_s": [0.0, 0.04]},
        "ground": profile["ground"], "frozen_points": profile["points"],
        "registration_target": target,
        "recomputed_static_ik": observed,
        "registered_static_same_short_window": baseline,
        "registered_static_full": full_baseline,
        "marker_error_columns": labels,
        "marker_error_column_means": statistics,
        "marker_error_column_maxima": dict(zip(labels[1:], errors[:, 1:].max(axis=0).tolist())),
        "post_solver_vs_registered_short": {
            "heel_mean_clearance_delta_m": float(np.mean(solved_clearance[:, 0]-baseline_clearance[:, 0])),
            "toe_mean_clearance_delta_m": float(np.mean(solved_clearance[:, 1]-baseline_clearance[:, 1])),
            "median_stl_clearance_delta_m": observed["median_min_stl_clearance_m"]-baseline["median_min_stl_clearance_m"],
            "median_pitch_delta_deg": observed["median_sagittal_pitch_deg"]-baseline["median_sagittal_pitch_deg"],
            "coordinate_max_absolute_change": {name: {"value": float(np.max(np.abs(solved_q[:, i]-baseline_q[:, i]))),
                "unit": "m" if name in {"pelvis_tx", "pelvis_ty", "pelvis_tz"} else ("deg" if motion.in_degrees else "rad")}
                for i, name in enumerate(motion.labels[1:])}},
        "outputs": {"setup": fp(setup), "motion": fp(motion_path), "marker_errors": fp(error_path)},
        "checks": {"nine_static_frames": True, "unchanged_28_marker_weight1_setup": True,
            "production_sampler_parity": True, "heel_clearance_negative_all_solved_frames": True,
            "heel_contact_all_solved_frames": observed["binary_points"]["left_heel"]["contact_fraction"] == 1.0},
        "limitations": ["The 9 frames are inside the marker-calibration static trial; this corroborates registration consistency, not independent dynamic qualification.",
            "STL target is an AB06-relative benchmark median over full static windows; a short-window median and marker-IK adjustments can leave small target residuals.",
            "The frozen binary points/plane are virtual calibrated profiles, not a measured AB07 physical floor."]
    }
    target_receipt.write_text(json.dumps(receipt, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    print(json.dumps({"receipt": str(target_receipt.relative_to(REPO)), "recomputed": observed,
                      "registered_short": baseline, "marker_error_means": statistics, "checks": receipt["checks"]}, indent=2))


if __name__ == "__main__":
    main()
