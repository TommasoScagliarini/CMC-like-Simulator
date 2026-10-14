"""Subject-local static geometry and donor-registration identifiability audit.

Only source reads, Position FK and scalar/algebraic geometry. No model write,
IK tool, dynamics/integration, marker calibration, profile/controller change.
Healthy VTP visual assets may be supplied with --healthy-geometry-dir; these
are explicitly generic/scaled model surfaces, not measured shoes/floor.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import glob
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
from xml.etree import ElementTree as ET

import numpy as np
from scipy.optimize import brentq

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
COMMON = ROOT / "validation/ab06_ab07_independent_attribution_2026_10_07/rebuild"
sys.path.insert(0, str(ROOT))
import opensim
from model_loader import _configure_geometry_search_paths
from tools import register_seasea_ground as registration
from tools.calibrate_ab06_seasea_markers import MotionData, parse_motion, parse_trc, set_state_from_motion
from tools.convert_epic_ab06_tables import extract_marker_data, IK_SOURCE_COLUMNS
STATIC_DECODER = ROOT / "validation/ab06_ab07_preparation_vs_model_2026_10_07/data_preparation_audit.py"
spec = importlib.util.spec_from_file_location("frozen_static12_table_decoder", STATIC_DECODER)
static_decoder = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = static_decoder
spec.loader.exec_module(static_decoder)


def fp(path):
    path = Path(path).resolve()
    try:
        label = path.relative_to(ROOT).as_posix()
        external = False
    except ValueError:
        label, external = str(path), True
    return {"path": label, "external_asset": external, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def vec(value):
    return np.array([value.get(i) for i in range(3)])


def rot(value):
    return np.array([[value.get(i, j) for j in range(3)] for i in range(3)])


def stats(values):
    values = np.asarray(values)
    if not np.all(np.isfinite(values)):
        raise ValueError("Nonfinite geometry statistic")
    return {"mean": np.mean(values, axis=0).tolist(), "median": np.median(values, axis=0).tolist(),
            "minimum": np.min(values, axis=0).tolist(), "maximum": np.max(values, axis=0).tolist(),
            "std": np.std(values, axis=0).tolist()}


def geometry_directories(supplied):
    dirs = [supplied] if supplied else []
    dirs += [ROOT / "Geometry", ROOT / "geometry"]
    for key in ("OPENSIM_HOME", "OPENSIM_ROOT", "OPENSIM_INSTALL_DIR"):
        if os.environ.get(key):
            p = Path(os.environ[key]); dirs += [p / "Geometry", p / "Resources/opensim/Geometry", p / "share/opensim/Geometry"]
    dirs += list(map(Path, glob.glob("/Applications/OpenSim*/OpenSim*.app/Contents/Resources/opensim/Geometry")))
    dirs += list(map(Path, glob.glob("C:/OpenSim*/Geometry") + glob.glob("C:/Program Files/OpenSim*/Geometry")))
    return [Path(d).resolve() for d in dirs if d and Path(d).is_dir()]


def read_vtp(path):
    tree = ET.parse(path)
    piece = tree.find(".//Piece")
    points = piece.find("Points/DataArray")
    if points.get("format") != "ascii" or points.get("NumberOfComponents") != "3":
        raise ValueError("Only declared ASCII Vec3 VTP points are supported")
    if points.get("type") not in ("Float32", "Float64"):
        raise ValueError("VTP coordinate scalar type must be explicitly floating")
    dtype = np.float32 if points.get("type") == "Float32" else np.float64
    values = np.fromstring(points.text, sep=" ").astype(dtype).astype(float).reshape(-1, 3)
    if len(values) != int(piece.get("NumberOfPoints")) or not np.all(np.isfinite(values)):
        raise ValueError("VTP point count/finite contract differs")
    return values


def foot_assets(model_path, dirs):
    tree = ET.parse(model_path)
    if tree.findtext("Model/length_units") != "meters":
        raise ValueError("Model length units must be explicitly meters")
    result = {}
    for name in ("talus_l", "calcn_l", "toes_l", "talus_r", "calcn_r", "toes_r"):
        body = tree.find(f".//BodySet/objects/Body[@name='{name}']")
        if body is None:
            continue
        entries = body.findall("attached_geometry/Mesh")
        if len(entries) != 1 or entries[0].findtext("socket_frame").strip() != "..":
            raise ValueError("Healthy visual surface must be attached directly to one body")
        entry = entries[0]
        filename = entry.findtext("mesh_file")
        resolved = [p / filename for p in dirs if (p / filename).is_file()]
        if not resolved:
            raise FileNotFoundError(f"Healthy visual asset unavailable: {filename}; supply --healthy-geometry-dir")
        scale = np.array(list(map(float, entry.findtext("scale_factors").split())))
        if scale.shape != (3,) or np.any(scale <= 0):
            raise ValueError("Invalid visual scale")
        vertices = read_vtp(resolved[0])*scale
        result[name] = {"vertices": vertices, "source": fp(resolved[0]), "model_body": name,
                        "mesh_file": filename, "scale_factors": scale.tolist(), "vertices_count": len(vertices),
                        "local_bounds_m": {"minimum": vertices.min(axis=0).tolist(), "maximum": vertices.max(axis=0).tolist()},
                        "surface_contract": "Scaled generic visual model surface, not an acquisition-specific physical sole/shoe measurement"}
    return result


def sample_model(model_path, motion, assets, vertices, trc=None):
    _configure_geometry_search_paths(str(model_path))
    model = opensim.Model(str(model_path))
    state = model.initSystem()
    if model.getConstraintSet().getSize():
        raise ValueError("Explicit unconstrained FK required")
    coords = model.getCoordinateSet()
    if {coords.get(i).getName() for i in range(coords.getSize())} != set(motion.labels[1:]):
        raise ValueError("Full coordinate names required; no default fill")
    prosthetic = model.getJointSet().contains("pros_ankle")
    joint_names = {"hip_l": ("hip_l", "child"), "knee_l": ("pros_knee" if prosthetic else "knee_l", "child"),
                   "ankle_l": ("pros_ankle" if prosthetic else "ankle_l", "parent"),
                   "hip_r": ("hip_r", "child"), "knee_r": ("knee_r", "child"), "ankle_r": ("ankle_r", "parent")}
    centers = {"pelvis_origin": []}; centers.update({n: [] for n in joint_names})
    visual_minima = {side: [] for side in ("left", "right") if ("calcn_l" if side == "left" else "calcn_r") in assets}
    mesh_minima = {name: [] for name in assets}
    foot_p, foot_r, tibia_r = [], [], []
    errors = {}
    if trc:
        if len(trc.times) != len(motion.times) or np.max(np.abs(trc.times-motion.times)) > 1e-10:
            raise ValueError("Static TRC/IK clocks must row-match; no interpolation")
        marker_names = [side+name for side in ("L_", "R_") for name in ("Ankle_Lat", "Heel", "Toe_Lat", "Toe_Med", "Toe_Tip")]
        errors = {n: [] for n in marker_names}
    assignment = 0.0
    for i, row in enumerate(motion.values):
        state.setTime(float(row[0]))
        q = dict(zip(motion.labels[1:], row[1:]))
        set_state_from_motion(model, state, q, motion.in_degrees)
        for name, value in q.items():
            expected = math.radians(value) if motion.in_degrees and name not in ("pelvis_tx", "pelvis_ty", "pelvis_tz") else value
            assignment = max(assignment, abs(coords.get(name).getValue(state)-expected))
        centers["pelvis_origin"].append(vec(model.getBodySet().get("pelvis").getTransformInGround(state).p()))
        for name, (joint_name, role) in joint_names.items():
            joint = model.getJointSet().get(joint_name)
            frame = joint.getParentFrame() if role == "parent" else joint.getChildFrame()
            centers[name].append(vec(frame.getTransformInGround(state).p()))
        for name, asset in assets.items():
            transform = model.getBodySet().get(name).getTransformInGround(state)
            world_y = asset["vertices"]@rot(transform.R())[1]+vec(transform.p())[1]
            mesh_minima[name].append(float(world_y.min()))
        for side in visual_minima:
            suffix = "_l" if side == "left" else "_r"
            visual_minima[side].append(min(mesh_minima[n][-1] for n in assets if n.endswith(suffix)))
        if prosthetic:
            transform = model.getBodySet().get("foot_l").getTransformInGround(state)
            foot_p.append(vec(transform.p())); foot_r.append(rot(transform.R()))
            tibia_r.append(rot(model.getBodySet().get("tibia_pylon").getTransformInGround(state).R()))
        for name in errors:
            errors[name].append(vec(model.getMarkerSet().get(name).getLocationInGround(state))-trc.markers[name][i])
    if assignment != 0.0:
        raise AssertionError("Requested static FK coordinate value was clamped/changed")
    result = {"centers": {n: np.array(v) for n, v in centers.items()},
              "visual_minima": {n: np.array(v) for n, v in visual_minima.items()},
              "mesh_minima": {n: np.array(v) for n, v in mesh_minima.items()},
              "marker_errors": {n: np.array(v) for n, v in errors.items()},
              "assignment_max_abs_error": assignment, "model": fp(model_path)}
    if prosthetic:
        result.update(foot_p=np.array(foot_p), foot_r=np.array(foot_r), tibia_r=np.array(tibia_r))
    return result


def summarized_fk(data, binary_y, primary_y):
    centers = data["centers"]
    lengths = {}
    for side in ("l", "r"):
        for label, a, b in (("upper", "hip_", "knee_"), ("shank", "knee_", "ankle_"), ("hip_ankle", "hip_", "ankle_")):
            lengths[side+"_"+label+"_center_distance_m"] = stats(np.linalg.norm(centers[a+side]-centers[b+side], axis=1))
    visual = {side: {"minimum_surface_ground_Y_m": stats(v), "relative_binary_plane_m": stats(v-binary_y),
                     "relative_primary_plane_m": stats(v-primary_y), "ankle_center_to_minimum_vertical_m": stats(centers["ankle_l" if side == "left" else "ankle_r"][:, 1]-v)} for side, v in data["visual_minima"].items()}
    return {"model": data["model"], "poses": len(centers["pelvis_origin"]), "coordinate_assignment_max_abs_error": data["assignment_max_abs_error"],
            "joint_center_ground_XYZ_m": {n: stats(v) for n, v in centers.items()}, "center_lengths": lengths,
            "visual_foot_surface_minima": visual,
            "mesh_surface_ground_Y_m": {n: stats(v) for n, v in data["mesh_minima"].items()},
            "static_healthy_marker_residuals": {n: {"signed_Y_m": stats(v[:, 1]), "euclidean_RMS_m": float(np.sqrt(np.mean(np.sum(v*v, axis=1))))} for n, v in data["marker_errors"].items()}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--healthy-geometry-dir", type=Path)
    args = parser.parse_args()
    output = HERE / "static_geometry_constraints.json"
    if output.exists():
        raise FileExistsError("Existing audit output is immutable")
    dirs = geometry_directories(args.healthy_geometry_dir)
    sources = {Path(__file__), STATIC_DECODER, ROOT / "AGENTS.md", ROOT / "tools/build_abx_seasea_pipeline.py", ROOT / "tools/fit_seasea_subject.py",
               ROOT / "tools/register_seasea_ground.py", ROOT / "tools/calibrate_ab06_seasea_markers.py", ROOT / "tools/convert_epic_ab06_tables.py", ROOT / "model_loader.py",
               ROOT / "Geometry/AM_foot_l.STL", ROOT / "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim",
               ROOT / "models/AB06_SEASEA-raw/data/converted/static/static_01/static_01_ik_dataset_ab06_seasea.mot",
               ROOT / "validation/binary_phase_detector_v25_geometry_runs/2026-08-04_local_reach_sweep_dev02_04_08/selected_candidate_profile.json",
               ROOT / "online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json",
               ROOT / "plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib" if sys.platform == "darwin" else ROOT / "plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff.dll" if os.name == "nt" else ROOT / "plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.so"}
    inputs = {}
    for subject, day, trial, filename in (("AB06", "10_09_18", "static_01", "AB06.osim"), ("AB07", "10_14_18", "static01", "ab07.osim")):
        healthy = ROOT / f"models/{subject}-raw/osimxml" / filename
        original = ROOT / f"models/{subject}-raw/{day}/static/ik/{trial}.mat"
        raw_marker = ROOT / f"models/{subject}-raw/{day}/static/markers/{trial}.mat"
        folder = COMMON / "models" / f"{subject}_SEASEA-raw"
        anatomical = folder / "osimxml" / f"{subject}_SEASEA_anatomical.osim"
        registered = folder / "osimxml" / f"{subject}_SEASEA.osim"
        static_motion = folder / f"data/converted/static/{trial}/{trial}_ik_dataset_ab06_seasea.mot"
        trc_path = folder / f"data/converted/static/{trial}/{trial}.trc"
        registered_motion = folder / "osimxml" / f"{subject}_SEASEA_static_ground_registered.mot"
        manifest_path = folder / "osimxml" / f"{subject}_SEASEA_ground_registration.json"
        assets = foot_assets(healthy, dirs)
        sources.update([healthy, original, raw_marker, anatomical, registered, static_motion, trc_path, registered_motion, manifest_path])
        sources.update(Path(a["source"]["path"]) if a["source"]["external_asset"] else ROOT / a["source"]["path"] for a in assets.values())
        inputs[subject] = dict(healthy=healthy, original=original, raw_marker=raw_marker, anatomical=anatomical, registered=registered,
                              static_motion=static_motion, trc_path=trc_path, registered_motion=registered_motion, manifest_path=manifest_path, assets=assets)
    before = {fp(p)["path"]: fp(p)["sha256"] for p in sorted(sources)}
    plugin = next(p for p in sources if p.suffix in (".dylib", ".dll", ".so"))
    registration.load_plugin(plugin)
    binary_profile = json.loads((ROOT / "validation/binary_phase_detector_v25_geometry_runs/2026-08-04_local_reach_sweep_dev02_04_08/selected_candidate_profile.json").read_text())
    primary_profile = json.loads((ROOT / "online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json").read_text())
    origin, normal = np.array(binary_profile["ground"]["origin"]), np.array(binary_profile["ground"]["normal"])
    primary_y = float(primary_profile["ground"]["origin"][1])
    if not np.array_equal(normal, [0., 1., 0.]):
        raise ValueError("This height decomposition requires the declared vertical common normal")
    vertices = registration._vertices(ROOT / "Geometry/AM_foot_l.STL")
    _configure_geometry_search_paths(str(ROOT / "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim"))
    donor = opensim.Model(str(ROOT / "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim"))
    donor_motion = parse_motion(ROOT / "models/AB06_SEASEA-raw/data/converted/static/static_01/static_01_ik_dataset_ab06_seasea.mot")
    donor_h, donor_pitch, _ = registration._pose_geometry(donor, donor_motion, vertices, origin, normal)
    target_h, target_pitch = float(np.median(donor_h)), float(np.median(donor_pitch))
    subjects = {}
    geometry_cache = {}
    for subject, p in inputs.items():
        print(subject, "static source/FK constraints", flush=True)
        table, static_ik_meta = static_decoder.generic_table(p["original"])
        if table.names != tuple(IK_SOURCE_COLUMNS):
            raise ValueError("Original static healthy coordinate schema differs")
        values = np.column_stack([table.column(n) for n in table.names])
        motion = MotionData(values[:, 0], ["time", *table.names[1:]], values, True)
        remap = parse_motion(p["static_motion"])
        trc = parse_trc(p["trc_path"])
        marker_table, static_marker_meta = static_decoder.generic_table(p["raw_marker"])
        raw_markers = extract_marker_data(marker_table)
        marker_scale = .001 if raw_markers.units == "mm" else 1.
        marker_error = float(np.max(np.abs(np.stack([trc.markers[n] for n in raw_markers.names], axis=1)-raw_markers.values*marker_scale)))
        if marker_error > 1e-10 or len(trc.times) != len(raw_markers.time) or np.max(np.abs(trc.times-raw_markers.time)) > 1e-10:
            raise AssertionError("Converted marker values/units/Header clock differ from own raw static acquisition")
        if not np.all(np.isfinite(values)) or np.max(np.abs(motion.times-remap.times)) > 1e-10:
            raise ValueError("Current raw/remapped static clocks differ")
        mapped_names = {"pros_knee_angle": "knee_angle_l", "pros_ankle_angle": "ankle_angle_l"}
        remap_error = max(float(np.max(np.abs(remap.values[:, i]-values[:, motion.labels.index(mapped_names.get(n, n))]))) for i, n in enumerate(remap.labels))
        if remap_error > 1e-8:
            raise AssertionError("Converted static remap differs from own original source")
        hdata = sample_model(p["healthy"], motion, p["assets"], vertices, trc)
        right_assets = {n: a for n, a in p["assets"].items() if n.endswith("_r")}
        anatomical = sample_model(p["anatomical"], remap, right_assets, vertices)
        registered_motion = parse_motion(p["registered_motion"])
        registered = sample_model(p["registered"], registered_motion, right_assets, vertices)
        protected_equal = registration._protected(ET.parse(p["anatomical"])) == registration._protected(ET.parse(p["registered"]))
        if not protected_equal:
            raise AssertionError("Registered graft differs beyond distal mountY")
        projected = np.einsum("i,nij->nj", normal, anatomical["foot_r"])
        base = (anatomical["foot_p"]-origin)@normal
        slopes = np.einsum("i,ni->n", normal, anatomical["tibia_r"][:, :, 1])
        def analytic_angle(shift, projected=projected, base=base):
            c, s = math.cos(shift), math.sin(shift)
            row = np.column_stack((projected[:, 0]*c+projected[:, 1]*s, -projected[:, 0]*s+projected[:, 1]*c, projected[:, 2]))
            heights = np.min(np.einsum("ni,vi->nv", row, vertices), axis=1)+base
            return heights, np.unwrap(np.arctan2(row[:, 0], row[:, 1]))
        model = opensim.Model(str(p["anatomical"]))
        h0, pitch0, native_slopes = registration._pose_geometry(model, remap, vertices, origin, normal)
        manifest = json.loads(p["manifest_path"].read_text())
        shift = manifest["adjustments"]["static_ankle_pose_offset_rad"]
        mount = float(registration._mount_y(ET.parse(p["anatomical"])).text.split()[1])
        hrot, pitchrot = analytic_angle(shift)
        delta_y = registration._solve_height_shift(hrot, slopes, target_h, mount)
        native_rot_h, native_rot_pitch, _ = registration._pose_geometry(model, remap, vertices, origin, normal, shift)
        angle_error = max(float(np.max(np.abs(analytic_angle(0)[0]-h0))), float(np.max(np.abs(hrot-native_rot_h))), float(np.max(np.abs(pitchrot-native_rot_pitch))), float(np.max(np.abs(slopes-native_slopes))))
        registered_y = np.min(np.einsum("ni,vi->nv", np.einsum("i,nij->nj", normal, registered["foot_r"]), vertices), axis=1)+(registered["foot_p"]-origin)@normal
        length_parity = float(np.max(np.abs(registered_y-(hrot+slopes*delta_y))))
        if angle_error > 1e-12 or length_parity > 1e-12 or abs(delta_y-manifest["adjustments"]["distal_mount_delta_y_m"]) > 1e-12:
            raise AssertionError("Analytic rigid ankle/mount law differs from native registered model")
        foot_measured = {n: stats(trc.markers[n][:, 1]) for n in trc.markers if n.startswith(("L_", "R_")) and ("Ankle" in n or "Heel" in n or "Toe" in n)}
        subjects[subject] = {"healthy_source": fp(p["healthy"]), "original_static_IK": fp(p["original"]), "raw_static_markers": fp(p["raw_marker"]), "TRC": fp(p["trc_path"]),
            "clock_s": [float(motion.times[0]), float(motion.times[-1])], "rows": len(motion.times), "raw_to_converted_remap_max_abs_numeric_error": remap_error,
            "raw_to_converted_marker_max_abs_error_m": marker_error, "static_MCOS_meta": {"IK": static_ik_meta, "markers": static_marker_meta},
            "foot_visual_assets": {n: {k: v for k, v in a.items() if k != "vertices"} for n, a in p["assets"].items()},
            "healthy_original_static": summarized_fk(hdata, origin[1], primary_y), "anatomical_hardware_original_static": summarized_fk(anatomical, origin[1], primary_y),
            "registered_hardware_static_calibration": summarized_fk(registered, origin[1], primary_y),
            "measured_static_foot_marker_ground_Y_m": foot_measured,
            "hardware_minimum_surface_m": {"anatomical_raw": stats(h0), "anatomical_same_pylon_pitch_registered": stats(hrot), "registered_final": stats(registered_y)},
            "own_healthy_bilateral_minimum_surface_difference_left_minus_right_m": stats(hdata["visual_minima"]["left"]-hdata["visual_minima"]["right"]),
            "registered_hardware_minus_own_right_visual_minimum_ground_height_m": stats(registered_y+origin[1]-registered["visual_minima"]["right"]),
            "anatomical_hardware_minus_own_right_visual_minimum_ground_height_m": stats(h0+origin[1]-anatomical["visual_minima"]["right"]),
            "native_analytic_validation": {"analytic_angle_height_pitch_slope_max_abs_error": angle_error, "registered_length_height_max_abs_error_m": length_parity, "protected_graft_except_mountY_equal": protected_equal},
            "own_right_visual_relative_target_candidate": {"target_definition": "Median STL left minimum equals median own right generic visual-foot minimum on same raw biological static pose; model-internal alternative, not physical floor/shoe truth",
                "at_donor_pitch_delta_mount_y_m": registration._solve_height_shift(hrot, slopes, float(np.median(anatomical["visual_minima"]["right"]-origin[1])), mount)}}
        geometry_cache[subject] = (p, model, remap, analytic_angle, slopes, mount, shift, h0, pitch0, anatomical)
    # Explicit AB07 donor-target sensitivity. Ground origin translation affects
    # subject and donor clearances equally and cannot identify a physical floor.
    p, model, remap, angle, slopes, mount, shift, h0, pitch0, anatomical = geometry_cache["AB07"]
    sensitivities = []
    for label, dh, dp in (("baseline", 0., 0.), ("donor_pitch_minus2deg", 0., -2.), ("donor_pitch_plus2deg", 0., 2.),
                          ("donor_height_minus5mm", -.005, 0.), ("donor_height_plus5mm", .005, 0.)):
        ankle_shift = (target_pitch+math.radians(dp)-float(np.median(pitch0))+math.pi)%(2*math.pi)-math.pi
        heights, pitches = angle(ankle_shift)
        dy = registration._solve_height_shift(heights, slopes, target_h+dh, mount)
        sensitivities.append({"case": label, "target_height_m": target_h+dh, "target_pitch_deg": math.degrees(target_pitch)+dp,
                              "static_ankle_shift_deg": math.degrees(ankle_shift), "delta_mount_y_m": dy,
                              "extra_distal_length_m": -dy, "resulting_mount_y_m": mount+dy,
                              "height_residual_m": float(np.median(heights+slopes*dy)-(target_h+dh)),
                              "pitch_residual_rad": float(np.median(pitches)-(target_pitch+math.radians(dp)))})
    baseline = sensitivities[0]["delta_mount_y_m"]
    plane_shifts = []
    heights, _ = angle(shift)
    for offset in (-.010, .010):
        dy = registration._solve_height_shift(heights-offset, slopes, target_h-offset, mount)
        plane_shifts.append({"common_plane_y_shift_m": offset, "both_clearances_shift_m": -offset,
                             "delta_mount_y_m": dy, "difference_from_baseline_m": dy-baseline})
    ankle = model.getCoordinateSet().get("pros_ankle_angle")
    raw_ankle = remap.values[:, remap.labels.index("pros_ankle_angle")]*np.pi/180
    lo, hi = ankle.getRangeMin()-float(raw_ankle.min()), ankle.getRangeMax()-float(raw_ankle.max())
    knots = np.linspace(lo+1e-10, hi-1e-10, 41)
    residuals = [float(np.median(angle(x)[0])-target_h) for x in knots]
    roots = []
    for i in range(len(knots)-1):
        if residuals[i]*residuals[i+1] < 0:
            root = float(brentq(lambda x: float(np.median(angle(x)[0])-target_h), knots[i], knots[i+1], xtol=1e-13))
            if not any(abs(root-r["static_ankle_shift_rad"]) < 1e-8 for r in roots):
                h, pitch = angle(root)
                native_h, native_pitch, _ = registration._pose_geometry(model, remap, vertices, origin, normal, root)
                parity = max(float(np.max(np.abs(h-native_h))), float(np.max(np.abs(pitch-native_pitch))))
                if parity > 1e-12:
                    raise AssertionError("Fixed-pylon alternative differs from native FK")
                roots.append({"static_ankle_shift_rad": root, "static_ankle_shift_deg": math.degrees(root), "delta_mount_y_m": 0.,
                              "height_residual_m": float(np.median(h)-target_h), "resulting_median_pitch_deg": math.degrees(float(np.median(pitch))),
                              "difference_from_donor_pitch_deg": math.degrees(float(np.median(pitch))-target_pitch), "native_algebra_parity": parity})
    after = {fp(p)["path"]: fp(p)["sha256"] for p in sorted(sources)}
    if before != after:
        raise AssertionError("An input changed during static FK audit")
    sensitivity_values = [row["extra_distal_length_m"] for row in sensitivities]
    result = {"schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(), "status": "PASS", "helper": fp(Path(__file__)),
        "instruction_check_token": "CMC_AGENT_OK_2026", "subjects": subjects,
        "benchmark_target": {"donor": fp(ROOT / "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim"), "static_pose_count": len(donor_motion.times),
                             "minimum_STL_clearance_median_m": target_h, "sagittal_pitch_median_deg": math.degrees(target_pitch)},
        "plane_contracts": {"binary_registration": {"origin_m": origin.tolist(), "normal": normal.tolist()}, "primary_contact_origin_Y_m": primary_y,
                            "physical_floor_identified": False, "visual_minima_are_force_contact_points": False},
        "AB07_target_sensitivity": sensitivities, "extra_length_span_over_declared_target_perturbations_m": max(sensitivity_values)-min(sensitivity_values),
        "AB07_common_plane_shift_invariance": plane_shifts,
        "AB07_same_pylon_ankle_only_clearance_alternatives": {"scope": "Height-only roots in unchanged full-motion ankle range; may fail donor pitch and are not physical assembly recommendations",
            "search_grid_size": len(knots), "allowed_static_shift_range_deg": [math.degrees(lo), math.degrees(hi)], "roots": roots,
            "search_limit": "Sign-changing branches bracketed on41 grid; root list is not a proof no other tangential root exists."},
        "equations": {"surface": "h_i(delta_theta,delta_y) = min_v[n dot R_foot_i Rz(delta_theta) v] + n dot (p_foot_i-origin) + (n dot R_tibia_i e_y)*delta_y",
                      "solver": "median_i h_i = donor_target_clearance; median projected sagittal pitch = donor_target_pitch",
                      "plane_translation_invariance": "Changing originY by c changes both subject heights and donor target by -c; all median residuals and fitted delta_y remain equal.",
                      "pylon_identifiability": "Given fixed biological pose, fixed hardware, fixed ankle pitch target and fixed height target, positive pylon slopes produce a unique scalar fit. This uniqueness is conditional on chosen benchmark targets."},
        "point_definitions": {"pelvis": "Pelvis body origin", "hip": "Hip child joint frame origin", "healthy_knee": "Knee child frame/tibia origin after coupled CustomJoint transform; parent frame origin is not the distal knee center",
                              "ankle": "Ankle parent joint frame origin, coincident child origin for zero child offset", "visual_surface": "Minimum over all scaled talus/calcn/toes visual vertices, not measured soft tissue/shoe sole or online contact spheres"},
        "identification_limits": ["Static GRF is absent for both raw subjects; source marker/IK/ik_offset tables do not establish force support or physical floor/shoe dimensions.",
            "Healthy marker agreement validates subject pose/marker geometry on the fitting data, not virtual prosthetic hardware alignment or measured prosthetic fiducials.",
            "The12 freely recalibrated marker local coordinates absorb changes in assembly/static pose; marker RMS alone does not independently identify pylon length.",
            "The pre-ground anatomical shank length uses own/reference healthy joint geometry. The additional17.579mm is a solved response to donor-relative static height/pitch targets, not a separately measured hardware length.",
            "Own right generic visual-foot target is a subject-local model convention; it is not promoted to physical floor truth or an optimal assembly decision.",
            "A fixed-pylon ankle-only solution can satisfy height while changing pitch. It demonstrates conditional non-uniqueness if pitch is unconstrained; it does not establish that the registered solution is wrong.",
            "No requirement that healthy/prepared subject height ordering be preserved is imposed; hardware and native joints differ.",
            "This diagnostic makes no GRF/recruitment/tracking/dynamic-causality claim and edits no model/controller/profile."],
        "inputs_integrity": {"count": len(before), "all_unchanged": True, "before": before, "after": after}}
    with output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, allow_nan=False); stream.write("\n")
    print("PASS", fp(output))
    print("AB07 target sensitivities", sensitivities)
    print("AB07 same-pylon roots", roots)


if __name__ == "__main__":
    main()
