"""Register fixed SEA foot hardware before freely calibrating anatomical markers.

The existing donor static pose defines a benchmark-relative ground convention.
Match median STL sole clearance and sagittal inclination by adjusting only the
distal pylon length and the ankle pose used for STATIC marker calibration.
No detector points, gains, joint axes/defaults/ranges or dynamic targets change.
The convention is not a measurement of an acquisition's physical floor.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import struct
from xml.etree import ElementTree as ET

import numpy as np
import opensim as osim
from scipy.optimize import brentq

try:
    from .calibrate_ab06_seasea_markers import (
        MotionData, from_vec3, load_plugin, parse_motion, set_state_from_motion,
    )
    from .fit_seasea_subject import _canonical
except ImportError:
    from calibrate_ab06_seasea_markers import (
        MotionData, from_vec3, load_plugin, parse_motion, set_state_from_motion,
    )
    from fit_seasea_subject import _canonical


def _fp(path: Path) -> dict:
    return {"path": str(path.resolve()), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def _mount_y(tree: ET.ElementTree) -> ET.Element:
    joint = tree.find(".//JointSet/objects/PinJoint[@name='pros_ankle']")
    if joint is None:
        raise ValueError("Ground registration requires pros_ankle PinJoint.")
    for role in ("parent", "child"):
        name = joint.findtext(f"socket_{role}_frame")
        frame = joint.find(f"frames/PhysicalOffsetFrame[@name='{name}']")
        orientation = [] if frame is None else [float(v) for v in frame.findtext("orientation", "").split()]
        if len(orientation) != 3 or any(not math.isfinite(v) or abs(v) > 1e-12 for v in orientation):
            raise ValueError("Ground registration requires aligned ankle offset frames.")
        values = [float(v) for v in frame.findtext("translation", "").split()]
        if len(values) != 3 or not all(math.isfinite(v) for v in values):
            raise ValueError("Invalid ankle mounting translation.")
        if role == "child" and any(abs(v) > 1e-12 for v in values):
            raise ValueError("Ground registration requires zero ankle child translation.")
        if role == "parent":
            if abs(values[0]) > 1e-12 or abs(values[2]) > 1e-12 or values[1] >= 0:
                raise ValueError("Expected a longitudinal, distal pylon mount.")
            result = frame.find("translation")
    return result


def _protected(tree: ET.ElementTree) -> tuple:
    copy = deepcopy(tree)
    value = _mount_y(copy)
    parts = value.text.split()
    parts[1] = "0"
    value.text = " ".join(parts)
    return _canonical(copy.getroot())


def _vertices(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    if len(raw) < 84:
        raise ValueError("Foot mesh must be a binary STL.")
    count = struct.unpack_from("<I", raw, 80)[0]
    if not count or len(raw) != 84 + 50 * count:
        raise ValueError("Foot mesh must be a binary STL with exact triangle count.")
    dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    triangles = np.ndarray((count,), dtype=dtype, buffer=raw, offset=84)
    vertices = np.unique(triangles["vertices"].reshape(-1, 3).astype(float), axis=0)
    if not np.all(np.isfinite(vertices)):
        raise ValueError("Nonfinite STL vertices.")
    return vertices


def _foot_geometry(tree: ET.ElementTree, mesh: Path) -> tuple:
    body = tree.find(".//BodySet/objects/Body[@name='foot_l']")
    if body is None:
        raise ValueError("Missing foot_l hardware.")
    geometries = body.findall("attached_geometry/*")
    if len(geometries) != 1 or geometries[0].tag != "Mesh":
        raise ValueError("Expected exactly one attached foot hardware mesh.")
    geometry = geometries[0]
    if geometry.findtext("socket_frame", "..").strip() != "..":
        raise ValueError("Foot STL must be attached directly to foot_l.")
    name = Path(geometry.findtext("mesh_file", "")).name
    if name.casefold() != mesh.name.casefold():
        raise ValueError("The provided STL does not match foot_l mesh_file.")
    if [float(v) for v in geometry.findtext("scale_factors", "1 1 1").split()] != [1, 1, 1]:
        raise ValueError("Registration requires the same unscaled fixed foot hardware.")
    return _canonical(geometry)


def _pose_geometry(model, motion: MotionData, vertices, origin, normal, ankle_shift=0.0):
    coordinates = model.getCoordinateSet()
    model_names = {coordinates.get(i).getName() for i in range(coordinates.getSize())}
    if set(motion.labels[1:]) != model_names or len(set(motion.labels)) != len(motion.labels):
        raise ValueError("Static motion must explicitly contain every model coordinate once.")
    if not np.all(np.isfinite(motion.values)) or np.any(np.diff(motion.times) <= 0):
        raise ValueError("Static motion contains invalid values or timestamps.")
    ankle = coordinates.get("pros_ankle_angle")
    state = model.initSystem()
    heights, pitches, slopes = [], [], []
    for row in motion.values:
        q = dict(zip(motion.labels[1:], row[1:]))
        q["pros_ankle_angle"] += math.degrees(ankle_shift) if motion.in_degrees else ankle_shift
        ankle_rad = math.radians(q["pros_ankle_angle"]) if motion.in_degrees else q["pros_ankle_angle"]
        if ankle_rad < ankle.getRangeMin() - 1e-12 or ankle_rad > ankle.getRangeMax() + 1e-12:
            raise ValueError("Registered static ankle pose exceeds the unchanged joint range.")
        set_state_from_motion(model, state, q, motion.in_degrees)
        foot = model.getBodySet().get("foot_l").getTransformInGround(state)
        rotation = foot.R()
        r = np.array([[rotation.get(i, j) for j in range(3)] for i in range(3)])
        projected = normal @ r
        if np.linalg.norm(projected[:2]) < 1e-10:
            raise ValueError("Foot sagittal inclination is undefined for this plane/pose.")
        heights.append(float(np.min(vertices @ projected + (from_vec3(foot.p()) - origin) @ normal)))
        pitches.append(math.atan2(projected[0], projected[1]))
        tibia = model.getBodySet().get("tibia_pylon").getTransformInGround(state).R()
        slopes.append(float(normal @ np.array([tibia.get(i, 1) for i in range(3)])))
    return np.array(heights), np.unwrap(pitches), np.array(slopes)


def _solve_height_shift(heights: np.ndarray, slopes: np.ndarray, target: float, initial_y: float) -> float:
    heights, slopes = np.asarray(heights), np.asarray(slopes)
    if heights.shape != slopes.shape or heights.size == 0 or not np.all(np.isfinite([*heights, *slopes, target, initial_y])):
        raise ValueError("Invalid mounting solve inputs.")
    if initial_y >= 0 or np.min(slopes) <= 1e-10:
        raise ValueError("Pylon height sensitivity must be positive and mount distal.")
    def residual(delta):
        return float(np.median(heights + slopes * delta) - target)
    r0 = residual(0.0)
    if abs(r0) <= 1e-12:
        return 0.0
    # Positive slopes give a unique monotone root. Bracket from its mean-slope
    # estimate, expanding as needed; never shorten the mount through zero.
    estimate = -r0 / float(np.median(slopes))
    edge = 2.0 * estimate
    upper_bound = -initial_y - 1e-12
    if edge > 0:
        edge = min(edge, upper_bound)
    for _ in range(50):
        if residual(edge) * r0 <= 0:
            break
        next_edge = 2.0 * edge
        if next_edge > 0:
            next_edge = min(next_edge, upper_bound)
        if next_edge == edge:
            raise ValueError("Ground target would invert or collapse the distal mount.")
        edge = next_edge
    else:
        raise ValueError("Cannot bracket the ground-registration mounting solve.")
    lo, hi = sorted((0.0, edge))
    return float(brentq(residual, lo, hi, xtol=1e-14))


def compute_registration(*, model_path: Path, static_motion: Path,
                         reference_model: Path, reference_motion: Path,
                         foot_mesh: Path, plane_profile: Path, plugin: Path | None):
    """Return a registered XML tree, static-only motion and manifest; no writes."""
    paths = {"input_model": model_path, "input_static_motion": static_motion,
             "reference_model": reference_model, "reference_static_motion": reference_motion,
             "foot_mesh": foot_mesh, "plane_profile": plane_profile, "script": Path(__file__)}
    if plugin:
        paths["plugin"] = plugin
    before = {name: _fp(path) for name, path in paths.items()}
    tree, ref_tree = ET.parse(model_path), ET.parse(reference_model)
    if _foot_geometry(tree, foot_mesh) != _foot_geometry(ref_tree, foot_mesh):
        raise ValueError("Subject/reference foot hardware geometry must be identical.")
    mounting = _mount_y(tree)
    _mount_y(ref_tree)
    original_y = float(mounting.text.split()[1])
    profile = json.loads(plane_profile.read_text())
    origin = np.asarray(profile["ground"]["origin"], dtype=float)
    normal = np.asarray(profile["ground"]["normal"], dtype=float)
    if origin.shape != (3,) or normal.shape != (3,) or not np.all(np.isfinite([*origin, *normal])) or abs(np.linalg.norm(normal)-1.0) > 1e-10:
        raise ValueError("Ground origin must be finite and normal must have unit length.")
    vertices = _vertices(foot_mesh)
    motion, reference = parse_motion(static_motion), parse_motion(reference_motion)
    load_plugin(plugin)
    subject_model, donor_model = osim.Model(str(model_path)), osim.Model(str(reference_model))
    target_h, target_pitch, _ = _pose_geometry(donor_model, reference, vertices, origin, normal)
    baseline_h, baseline_pitch, _ = _pose_geometry(subject_model, motion, vertices, origin, normal)
    h_target, pitch_target = float(np.median(target_h)), float(np.median(target_pitch))
    shift_ankle = (pitch_target - float(np.median(baseline_pitch)) + math.pi) % (2*math.pi) - math.pi
    if abs(shift_ankle) < 1e-12:
        shift_ankle = 0.0
    h_rotated, rotated_pitch, slopes = _pose_geometry(subject_model, motion, vertices, origin, normal, shift_ankle)
    if abs(float(np.median(rotated_pitch))-pitch_target) > 1e-10:
        raise ValueError("Ankle rotation does not reproduce the reference sagittal inclination.")
    shift_y = _solve_height_shift(h_rotated, slopes, h_target, original_y)
    registered = deepcopy(tree)
    if shift_y:
        parts = _mount_y(registered).text.split()
        parts[1] = format(original_y + shift_y, ".17g")
        _mount_y(registered).text = " ".join(parts)
    if _protected(registered) != _protected(tree):
        raise RuntimeError("Ground registration changed protected model XML.")
    q = np.array(motion.values, copy=True)
    q[:, motion.labels.index("pros_ankle_angle")] += math.degrees(shift_ankle) if motion.in_degrees else shift_ankle
    registered_motion = MotionData(motion.times.copy(), list(motion.labels), q, motion.in_degrees)
    fitted_h = h_rotated + slopes * shift_y
    if abs(float(np.median(fitted_h))-h_target) > 1e-10:
        raise RuntimeError("Ground registration failed the median hardware-clearance target.")
    if not all(_fp(paths[name])["sha256"] == receipt["sha256"] for name, receipt in before.items()):
        raise RuntimeError("An input changed during ground registration.")
    manifest = {"schema_version": 1, "method": "donor_relative_static_hardware_ground_v1",
        "sources": before, "ground": {"origin_m": origin.tolist(), "normal": normal.tolist()},
        "reference": {"pose_count": len(reference.times), "median_min_stl_clearance_m": h_target,
                      "median_sagittal_pitch_rad": pitch_target},
        "subject_before": {"pose_count": len(motion.times), "median_min_stl_clearance_m": float(np.median(baseline_h)),
                           "median_sagittal_pitch_rad": float(np.median(baseline_pitch)), "distal_mount_y_m": original_y},
        "subject_after": {"median_min_stl_clearance_m": float(np.median(fitted_h)),
                          "median_sagittal_pitch_rad": float(np.median(rotated_pitch)), "distal_mount_y_m": original_y+shift_y},
        "adjustments": {"distal_mount_delta_y_m": shift_y, "static_ankle_pose_offset_rad": shift_ankle,
                        "static_ankle_pose_offset_deg": math.degrees(shift_ankle)},
        "preserved": ["proximal anatomical mounting", "hip geometry and biological poses", "static knee pose",
            "fixed foot hardware", "SEA properties", "joint axes/defaults/ranges", "mass/COM/inertia", "markers",
            "source healthy motion", "detector and force profiles"],
        "checks": {"protected_model_xml_unchanged": True, "source_files_unchanged": True,
                   "height_residual_m": float(np.median(fitted_h))-h_target,
                   "pitch_residual_rad": float(np.median(rotated_pitch))-pitch_target},
        "limitations": ["Benchmark-relative registration, not a measured physical floor or shoe model.",
            "STL visual geometry defines the common registration convention, not the online sphere contact law.",
            "Ankle offset applies only to the static marker-calibration motion, not to runtime commands or joint zero.",
            "New marker IK and contact/event qualification remain required; matching static geometry alone is insufficient."]}
    return registered, registered_motion, manifest


def _write_motion(path: Path, motion: MotionData) -> None:
    header = (f"{path.stem}\nnRows={len(motion.times)}\nnColumns={len(motion.labels)}\n"
              f"inDegrees={'yes' if motion.in_degrees else 'no'}\nendheader\n")
    with path.open("w", encoding="utf-8") as stream:
        stream.write(header + "\t".join(motion.labels) + "\n")
        np.savetxt(stream, motion.values, delimiter="\t", fmt="%.17g")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("model", "static-motion", "reference-model", "reference-motion", "foot-mesh", "plane-profile",
                 "output-model", "output-static-motion", "report"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--plugin", type=Path)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    outputs = [args.output_model, args.output_static_motion, args.report]
    inputs = [args.model, args.static_motion, args.reference_model, args.reference_motion, args.foot_mesh, args.plane_profile]
    inputs.append(Path(__file__))
    if args.plugin:
        inputs.append(args.plugin)
    if len({p.resolve() for p in outputs}) != len(outputs) or {p.resolve() for p in outputs} & {p.resolve() for p in inputs}:
        raise ValueError("Registration outputs must be distinct from each other and all inputs.")
    if not args.force and any(p.exists() for p in outputs):
        raise FileExistsError("Registration output exists; use --force only for the requested outputs.")
    registered, motion, manifest = compute_registration(model_path=args.model, static_motion=args.static_motion,
        reference_model=args.reference_model, reference_motion=args.reference_motion, foot_mesh=args.foot_mesh,
        plane_profile=args.plane_profile, plugin=args.plugin)
    for output in outputs:
        output.parent.mkdir(parents=True, exist_ok=True)
    if not any(manifest["adjustments"].values()):
        # Preserve exact donor/reference XML bytes for the identity case.
        args.output_model.write_bytes(args.model.read_bytes())
    else:
        registered.write(args.output_model, encoding="utf-8", xml_declaration=True)
    _write_motion(args.output_static_motion, motion)
    manifest["outputs"] = {"model": _fp(args.output_model), "static_motion": _fp(args.output_static_motion)}
    args.report.write_text(json.dumps(manifest, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"method": manifest["method"], "adjustments": manifest["adjustments"], "checks": manifest["checks"]}))


if __name__ == "__main__":
    main()
