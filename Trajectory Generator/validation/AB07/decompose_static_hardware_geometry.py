"""Decompose AB06/AB07 static geometry using saved poses, without integration.

Hybrid coordinate/mount cases partition the observed difference; they are not
candidate anatomical fits. The STL is visual hardware, not a contact model.
"""
from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
from copy import deepcopy
from xml.etree import ElementTree as ET

import numpy as np
import opensim as osim

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(REPO / "scripts"))
from run_opensim_sea_pipeline import default_plugin  # noqa: E402
from calibrate_ab06_seasea_markers import (  # noqa: E402
    from_vec3, load_plugin, parse_motion, set_state_from_motion, vec3,
)

OUT = REPO / "validation/ab07_detector_investigation_2026_10_06/stage2/static_hardware_decomposition.json"


def fingerprint(path: Path) -> dict:
    return {"path": str(path.relative_to(REPO)),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def stats(values) -> dict:
    v = np.asarray(values, dtype=float)
    return {"count": v.size, "min": float(v.min()), "median": float(np.median(v)),
            "mean": float(v.mean()), "p95": float(np.percentile(v, 95)),
            "max": float(v.max())}


def stl_vertices(path: Path) -> np.ndarray:
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + 50 * count:
        raise ValueError("Expected binary STL with exact triangle count")
    dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)),
                      ("attribute", "<u2")])
    triangles = np.ndarray((count,), dtype=dtype, buffer=data, offset=84)
    return np.unique(triangles["vertices"].reshape(-1, 3).astype(float), axis=0)


def frame_translation(tree, joint: str, frame: str):
    return tree.find(f".//JointSet/objects/*[@name='{joint}']/frames/"
                     f"PhysicalOffsetFrame[@name='{frame}']/translation")


def geometry(model, state, points, vertices, spheres, binary, primary) -> dict:
    transform = model.getBodySet().get("foot_l").getTransformInGround(state)
    rotation = transform.R()
    r = np.array([[rotation.get(i, j) for j in range(3)] for i in range(3)])
    origin = from_vec3(transform.p())
    xyz = points @ r.T + origin
    mesh_world = vertices @ r.T + origin
    sphere_world = spheres[:, :3] @ r.T + origin
    clearance = (xyz - np.array(binary["ground"]["origin"])) @ np.array(binary["ground"]["normal"])
    surface_distance = ((sphere_world - np.array(primary["ground"]["origin"]))
                        @ np.array(primary["ground"]["normal"]) - spheres[:, 3])
    return {"heel_world_y_m": float(xyz[0, 1]), "toe_world_y_m": float(xyz[1, 1]),
            "heel_clearance_m": float(clearance[0]), "toe_clearance_m": float(clearance[1]),
            "foot_origin_y_m": float(origin[1]), "visual_mesh_min_ground_y_m": float(mesh_world[:, 1].min()),
            "primary_sphere_max_penetration_m": float(max(0, -surface_distance.min())),
            "primary_sphere_min_surface_clearance_m": float(surface_distance.min())}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--force", action="store_true", help="Replace only this script's diagnostic JSON")
    args = parser.parse_args()
    if OUT.exists() and not args.force:
        raise FileExistsError(OUT)
    paths = {"AB06": REPO / "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim",
             "AB07": REPO / "models/AB07_SEASEA_Threadmill/AB07_SEASEA_stiff321_500_pi.osim",
             "AB06_static": REPO / "models/AB06_SEASEA-raw/data/converted/static/static_01/static_01_ik_dataset_ab06_seasea.mot",
             "AB07_static": REPO / "models/AB07_SEASEA-raw/data/converted/static/static01/static01_ik_dataset_ab06_seasea.mot",
             "binary": REPO / "validation/binary_phase_detector_v25_geometry_runs/2026-08-04_local_reach_sweep_dev02_04_08/selected_candidate_profile.json",
             "primary": REPO / "online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json",
             "mesh": REPO / "Geometry/AM_foot_l.STL",
             "plugin": default_plugin(),
             "script": Path(__file__).resolve()}
    for subject in ("AB06", "AB07"):
        paths[subject + "_ik"] = paths[subject].parent / f"data/IK_results_{subject}_SEASEA.mot"
    before = {name: fingerprint(path) for name, path in paths.items()}
    load_plugin(paths["plugin"])
    osim.Logger.setLevelString("error")
    binary, primary = (json.loads(paths[name].read_text()) for name in ("binary", "primary"))
    points = np.array([p["location"] for p in binary["points"]])
    spheres = np.array([p["location"] + [p["radius"]] for p in primary["spheres"]
                        if p["frame"] == "/bodyset/foot_l"])
    vertices = stl_vertices(paths["mesh"])
    models = {subject: osim.Model(str(paths[subject])) for subject in ("AB06", "AB07")}
    states = {subject: model.initSystem() for subject, model in models.items()}
    static = {subject: parse_motion(paths[subject + "_static"]) for subject in models}
    mean_q = {subject: dict(zip(motion.labels[1:], motion.values[:, 1:].mean(axis=0)))
              for subject, motion in static.items()}
    if not all(motion.in_degrees for motion in static.values()):
        raise ValueError("Expected static remaps in degrees")
    groups = {"pelvis_translation": ["pelvis_tx", "pelvis_ty", "pelvis_tz"],
              "pelvis_orientation": ["pelvis_tilt", "pelvis_list", "pelvis_rotation"],
              "hip_angles": ["hip_flexion_l", "hip_adduction_l", "hip_rotation_l"],
              "knee_angle": ["pros_knee_angle"], "ankle_angle": ["pros_ankle_angle"],
              "proximal_mount": [], "distal_mount": [], "anatomical_hip_frame": []}
    frame_groups = {"proximal_mount": ("osseo", "transfemur_offset"),
                    "distal_mount": ("pros_ankle", "tibia_pylon_offset"),
                    "anatomical_hip_frame": ("hip_l", "pelvis_offset")}
    trees = {subject: ET.parse(paths[subject]) for subject in models}
    native_means = {}
    for subject, model in models.items():
        set_state_from_motion(model, states[subject], mean_q[subject], True)
        native_means[subject] = geometry(model, states[subject], points, vertices, spheres, binary, primary)
    group_names = list(groups)
    values = {}
    with tempfile.TemporaryDirectory(prefix="ab07_static_factors_") as temporary:
        for geometry_bits in itertools.product((False, True), repeat=3):
            tree = deepcopy(trees["AB06"])
            active_geometry = dict(zip(frame_groups, geometry_bits))
            for name, enabled in active_geometry.items():
                if enabled:
                    pair = frame_groups[name]
                    frame_translation(tree, *pair).text = frame_translation(trees["AB07"], *pair).text
            clone = Path(temporary) / "geometry.osim"
            tree.write(clone, encoding="utf-8", xml_declaration=True)
            model = osim.Model(str(clone)); state = model.initSystem()
            for pose_bits in itertools.product((False, True), repeat=5):
                enabled = dict(zip(group_names[:5], pose_bits)) | active_geometry
                q = mean_q["AB06"].copy()
                for name in group_names[:5]:
                    if enabled[name]:
                        for coordinate in groups[name]:
                            q[coordinate] = mean_q["AB07"][coordinate]
                set_state_from_motion(model, state, q, True)
                mask = sum(1 << i for i, name in enumerate(group_names) if enabled[name])
                values[mask] = geometry(model, state, points, vertices, spheres, binary, primary)
    last = (1 << len(groups)) - 1
    for subject, mask in (("AB06", 0), ("AB07", last)):
        for metric in native_means[subject]:
            if abs(native_means[subject][metric] - values[mask][metric]) > 1e-12:
                raise AssertionError(f"Full hybrid does not reconstruct {subject}: {metric}")
    shapley = {}
    n = len(groups)
    for metric in ("heel_world_y_m", "toe_world_y_m", "foot_origin_y_m"):
        contributions = {}
        for i, name in enumerate(group_names):
            total = 0.0
            for mask in values:
                if mask & (1 << i):
                    continue
                k = mask.bit_count()
                weight = math.factorial(k) * math.factorial(n-k-1) / math.factorial(n)
                total += weight * (values[mask | (1 << i)][metric] - values[mask][metric])
            contributions[name] = total
        delta = values[last][metric] - values[0][metric]
        error = sum(contributions.values()) - delta
        if abs(error) > 1e-12:
            raise AssertionError("Shapley contributions do not sum to the observed difference")
        shapley[metric] = {"AB07_minus_AB06_m": delta, "contributions_m": contributions,
                           "identity_error_m": error}
    sampled_dynamic = {}
    for subject, model in models.items():
        motion = parse_motion(paths[subject + "_ik"])
        force_path = paths[subject].parent / f"data/{subject}_SEASEA_GRF_FullSpan.mot"
        paths[subject + "_grf"] = force_path
        before[subject + "_grf"] = fingerprint(force_path)
        force_lines = force_path.read_text().splitlines()
        end = next(i for i, line in enumerate(force_lines) if line.strip().lower() == "endheader")
        force_body = [line for line in force_lines[end+1:] if line.strip()]
        force_labels = force_body[0].split()
        force_values = np.loadtxt(force_body[1:], ndmin=2)
        rows = motion.values[::20]
        loaded = np.interp(rows[:, 0], force_values[:, 0], force_values[:, force_labels.index("ground_force1_vy")]) > 20
        metrics = []
        for row in rows:
            set_state_from_motion(model, states[subject], dict(zip(motion.labels[1:], row[1:])), motion.in_degrees)
            metrics.append(geometry(model, states[subject], points, vertices, spheres, binary, primary))
        sampled_dynamic[subject] = {"native_stride": 20, "samples": len(rows),
            "metrics_m": {key: {"all": stats([r[key] for r in metrics]),
                                  "nominal_loaded": stats(np.array([r[key] for r in metrics])[loaded])}
                          for key in metrics[0]},
            "note": "No contact/event inference at this coarse grid; measured force defines nominal healthy support."}
    assert all(fingerprint(paths[name])["sha256"] == f["sha256"] for name, f in before.items())
    result = {"schema_version": 1, "scope": "Offline static FK factor decomposition and visual/primary contact geometry; no integration or operative changes",
        "sources": before, "source_files_unchanged": True,
        "mean_static_coordinates_degrees_or_metres": mean_q,
        "native_mean_static_geometry": native_means,
        "hybrid_pose_count": len(values), "factor_groups": groups,
        "exact_order_averaged_decomposition": shapley,
        "one_factor_swaps_from_AB06": {name: values[1 << i] for i, name in enumerate(group_names)},
        "all_AB07_except_one_factor": {name: values[last ^ (1 << i)] for i, name in enumerate(group_names)},
        "sampled_dynamic_geometry": sampled_dynamic,
        "limitations": ["Shapley partitions subject differences; it does not establish whether a fitting convention is physically correct.",
            "Static poses use dataset-healthy coordinate remaps employed during marker calibration, not new static IK.",
            "Visual STL bounds do not define the online sphere contact law or a measured sole/surface.",
            "Positive sphere penetration is geometric only; no force is calculated.",
            "Different subjects, static postures and marker mounting; no hypothesis of identical foot posture."]}
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(OUT), "heel_shapley": shapley["heel_world_y_m"], "checks": "PASS"}))


if __name__ == "__main__":
    main()
