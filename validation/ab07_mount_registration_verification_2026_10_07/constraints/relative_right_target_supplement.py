"""Scalar registration supplement with explicit XML matrix algebra.

No OpenSim import, native FK, model write, IK or dynamic simulation. Recreates
the already validated left-chain rigid algebra only to recover per-pose scalar
height/slopes, validates its aggregate against the frozen native audit, then
changes only the conditional height target in the scalar mounting solve.
"""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import struct
from xml.etree import ElementTree as ET

import numpy as np
from scipy.optimize import brentq

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
BASE = HERE / "static_geometry_constraints.json"
OUTPUT = HERE / "relative_right_target_supplement.json"
EXPECTED_BASE_SHA = "64f745a0a5b755d55ed65135a1346f960a5f18292d51bbd48391b345e4ae2e6d"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1., 0., 0.], [0., c, -s], [0., s, c]])


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0., s], [0., 1., 0.], [-s, 0., c]])


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def frame(joint, role):
    name = joint.findtext(f"socket_{role}_frame")
    elem = joint.find(f"frames/PhysicalOffsetFrame[@name='{name}']")
    if not np.array_equal(np.fromstring(elem.findtext("orientation"), sep=" "), [0., 0., 0.]):
        raise ValueError("Closed algebra requires the validated zero frame orientations")
    t = np.fromstring(elem.findtext("translation"), sep=" ")
    if t.shape != (3,) or not np.all(np.isfinite(t)):
        raise ValueError("Invalid frame translation")
    return t


def active_axes(joint, names, axes, translations):
    transform = joint.find("SpatialTransform")
    if transform is None or joint.tag != "CustomJoint":
        raise ValueError("Expected the frozen CustomJoint contract")
    for i, (name, axis) in enumerate(zip(names, axes), 1):
        e = transform.find(f"TransformAxis[@name='rotation{i}']")
        if e.findtext("coordinates").strip() != name or not np.array_equal(np.fromstring(e.findtext("axis"), sep=" "), axis):
            raise ValueError("Active rotation axis differs")
        if not np.array_equal(np.fromstring(e.findtext("LinearFunction/coefficients"), sep=" "), [1., 0.]):
            raise ValueError("Active rotation is not unit linear")
    for i in range(1, 4):
        e = transform.find(f"TransformAxis[@name='translation{i}']")
        if translations:
            if e.findtext("coordinates").strip() != ("pelvis_tx", "pelvis_ty", "pelvis_tz")[i-1]:
                raise ValueError("Ground translation coordinate differs")
            if not np.array_equal(np.fromstring(e.findtext("axis"), sep=" "), np.eye(3)[i-1]):
                raise ValueError("Ground translation axis differs")
            if not np.array_equal(np.fromstring(e.findtext("LinearFunction/coefficients"), sep=" "), [1., 0.]):
                raise ValueError("Ground translation function differs")
        else:
            constant = e.find("Constant")
            multiplier = e.find("MultiplierFunction")
            if constant is not None:
                value = float(constant.findtext("value"))
            elif multiplier is not None:
                value = float(multiplier.findtext("scale"))*float(multiplier.findtext("function/Constant/value"))
            else:
                raise ValueError("Hip translation must be exactly a scaled zero constant")
            if value != 0.:
                raise ValueError("Hip translation is not identically zero")


def motion(path):
    lines = Path(path).read_text().splitlines()
    end = next(i for i, line in enumerate(lines) if line.strip().lower() == "endheader")
    if "inDegrees=yes" not in lines[:end]:
        raise ValueError("Expected explicit inDegrees=yes")
    labels = lines[end+1].split()
    values = np.array([list(map(float, line.split())) for line in lines[end+2:] if line.strip()])
    if len(labels) != values.shape[1] or labels[0] != "time" or len(set(labels)) != len(labels) or not np.all(np.isfinite(values)):
        raise ValueError("Invalid complete static motion")
    return labels, values


def vertices(path):
    raw = Path(path).read_bytes()
    n = struct.unpack_from("<I", raw, 80)[0]
    if len(raw) != 84+50*n or not n:
        raise ValueError("Exact binary STL triangle count required")
    dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    return np.unique(np.ndarray((n,), dtype=dtype, buffer=raw, offset=84)["vertices"].reshape(-1, 3).astype(float), axis=0)


def solve(heights, slopes, target, initial_y):
    # Same scalar monotone median/Brent rule as hash-bound production helper.
    if np.min(slopes) <= 1e-10 or initial_y >= 0:
        raise ValueError("Positive slope/distal mounting required")
    residual = lambda delta: float(np.median(heights+slopes*delta)-target)
    r0 = residual(0.)
    if abs(r0) <= 1e-12:
        return 0.
    edge = -2*r0/float(np.median(slopes))
    upper = -initial_y-1e-12
    for _ in range(50):
        if edge > 0:
            edge = min(edge, upper)
        if residual(edge)*r0 <= 0:
            lo, hi = sorted((0., edge))
            return float(brentq(residual, lo, hi, xtol=1e-14))
        next_edge = 2*edge
        if next_edge > 0:
            next_edge = min(next_edge, upper)
        if next_edge == edge:
            raise ValueError("Target would collapse/invert mount")
        edge = next_edge
    raise ValueError("Unable to bracket scalar target")


def main():
    if OUTPUT.exists():
        raise FileExistsError("Existing supplement is immutable")
    if sha(BASE) != EXPECTED_BASE_SHA:
        raise ValueError("Frozen native audit binding differs")
    base = json.loads(BASE.read_text())
    s = base["subjects"]["AB07"]
    model_path = ROOT/s["anatomical_hardware_original_static"]["model"]["path"]
    static_path = model_path.parents[1]/"data/converted/static/static01/static01_ik_dataset_ab06_seasea.mot"
    manifest_path = model_path.parent/"AB07_SEASEA_ground_registration.json"
    mesh_path = ROOT/"Geometry/AM_foot_l.STL"
    protected = {ROOT/k if not Path(k).is_absolute() else Path(k): v for k, v in base["inputs_integrity"]["after"].items()}
    for path, digest in protected.items():
        if sha(path) != digest:
            raise ValueError(f"Protected input differs: {path}")
    protected[BASE] = EXPECTED_BASE_SHA
    protected[Path(__file__).resolve()] = sha(__file__)
    before = {str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p): sha(p) for p in protected}
    tree = ET.parse(model_path)
    joints = {e.get("name"): e for e in tree.findall(".//JointSet/objects/*")}
    gp, hip, osseo, knee, ankle = [joints[n] for n in ("ground_pelvis", "hip_l", "osseo", "pros_knee", "pros_ankle")]
    active_axes(gp, ("pelvis_tilt", "pelvis_list", "pelvis_rotation"), ([0,0,1], [1,0,0], [0,1,0]), True)
    active_axes(hip, ("hip_flexion_l", "hip_adduction_l", "hip_rotation_l"), ([0,0,1], [-1,0,0], [0,-1,0]), False)
    if osseo.tag != "WeldJoint" or knee.tag != "PinJoint" or ankle.tag != "PinJoint":
        raise ValueError("Expected weld+positiveZ pin distal chain")
    for j in (gp, hip, osseo, knee, ankle):
        if np.any(frame(j, "child") != 0.):
            raise ValueError("This validated chain has zero child offsets")
    if np.any(frame(gp, "parent") != 0.):
        raise ValueError("Expected zero ground offset")
    labels, values = motion(static_path)
    model_names = {c.get("name") for c in tree.findall(".//JointSet/objects/*/coordinates/Coordinate")}
    if model_names != set(labels[1:]):
        raise ValueError("Motion must contain the complete 21-coordinate model schema")
    v = vertices(mesh_path)
    origin = np.array(base["plane_contracts"]["binary_registration"]["origin_m"])
    if base["plane_contracts"]["binary_registration"]["normal"] != [0., 1., 0.]:
        raise ValueError("Vertical normal required")
    center = {k: [] for k in ("pelvis_origin", "hip_l", "knee_l", "ankle_l")}
    foot_r, foot_p, slopes = [], [], []
    for row in values:
        q = dict(zip(labels[1:], row[1:]))
        q = {n: x if n in ("pelvis_tx", "pelvis_ty", "pelvis_tz") else math.radians(x) for n, x in q.items()}
        rpelvis = rz(q["pelvis_tilt"])@rx(q["pelvis_list"])@ry(q["pelvis_rotation"])
        pelvis = np.array([q[n] for n in ("pelvis_tx", "pelvis_ty", "pelvis_tz")])
        hip_p = pelvis+rpelvis@frame(hip, "parent")
        rupper = rpelvis@rz(q["hip_flexion_l"])@rx(-q["hip_adduction_l"])@ry(-q["hip_rotation_l"])
        knee_p = hip_p+rupper@(frame(osseo, "parent")+frame(knee, "parent"))
        rtibia = rupper@rz(q["pros_knee_angle"])
        ankle_p = knee_p+rtibia@frame(ankle, "parent")
        rfoot = rtibia@rz(q["pros_ankle_angle"])
        for name, val in zip(center, (pelvis, hip_p, knee_p, ankle_p)):
            center[name].append(val)
        foot_r.append(rfoot); foot_p.append(ankle_p); slopes.append(rtibia[1, 1])
    foot_r, foot_p, slopes = np.array(foot_r), np.array(foot_p), np.array(slopes)
    def h(shift):
        projected = np.einsum("nij,jk->nik", foot_r, rz(shift))[:, 1, :]
        return np.min(np.einsum("ni,vi->nv", projected, v), axis=1)+(foot_p-origin)[:, 1]
    manifest = json.loads(manifest_path.read_text())
    shift = manifest["adjustments"]["static_ankle_pose_offset_rad"]
    mount = frame(ankle, "parent")[1]
    stat_funcs = {"mean": np.mean, "median": np.median, "minimum": np.min, "maximum": np.max, "std": np.std}
    parity = {"joint_center_statistic_max_abs_error_m": 0., "height_statistic_max_abs_error_m": 0., "native_baseline_fit_difference_m": 0.}
    for n, samples in center.items():
        for stat, func in stat_funcs.items():
            err = np.max(np.abs(func(samples, axis=0)-np.array(s["anatomical_hardware_original_static"]["joint_center_ground_XYZ_m"][n][stat])))
            parity["joint_center_statistic_max_abs_error_m"] = max(parity["joint_center_statistic_max_abs_error_m"], float(err))
    for height_samples, name in ((h(0.), "anatomical_raw"), (h(shift), "anatomical_same_pylon_pitch_registered")):
        for stat, func in stat_funcs.items():
            parity["height_statistic_max_abs_error_m"] = max(parity["height_statistic_max_abs_error_m"], abs(float(func(height_samples))-s["hardware_minimum_surface_m"][name][stat]))
    donor_h = base["benchmark_target"]["minimum_STL_clearance_median_m"]
    baseline_dy = solve(h(shift), slopes, donor_h, mount)
    parity["native_baseline_fit_difference_m"] = abs(baseline_dy-base["AB07_target_sensitivity"][0]["delta_mount_y_m"])
    if max(parity.values()) > 1e-12:
        raise AssertionError("Closed scalar algebra differs from frozen native audit")
    right = {subject: base["subjects"][subject]["healthy_original_static"]["visual_foot_surface_minima"]["right"]["minimum_surface_ground_Y_m"]["median"] for subject in ("AB06", "AB07")}
    gap = base["subjects"]["AB06"]["registered_hardware_minus_own_right_visual_minimum_ground_height_m"]["median"]
    target = right["AB07"]-origin[1]+gap
    cases = []
    for label, delta_theta in (("same_static_ankle_offset0", 0.), ("donor_pitch_static_ankle_offset", shift)):
        heights = h(delta_theta)
        dy = solve(heights, slopes, target, mount)
        cases.append({"case": label, "static_ankle_offset_deg": math.degrees(delta_theta), "target_hardware_clearance_m": target,
                      "baseline_same_pylon_clearance_median_m": float(np.median(heights)), "baseline_target_residual_m": float(np.median(heights)-target),
                      "delta_mount_y_m": dy, "extra_distal_length_m": -dy, "resulting_mount_y_m": mount+dy,
                      "height_residual_m": float(np.median(heights+slopes*dy)-target)})
    after = {str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p): sha(p) for p in protected}
    if before != after:
        raise AssertionError("Protected input changed")
    result = {"status": "PASS", "created_utc": datetime.now(timezone.utc).isoformat(), "scope": "Scalar/matrix algebra only; no native FK/IK/model writes or integration",
              "helper_sha256": sha(__file__), "frozen_native_audit_sha256": EXPECTED_BASE_SHA, "native_aggregate_parity": parity,
              "own_right_visual_minimum_groundY_median_m": right, "own_right_groundY_AB07_minus_AB06_m": right["AB07"]-right["AB06"],
              "transported_donor_left_minus_own_right_gap_m": gap, "target_definition": "Median own-right AB07 visual groundY + AB06 median per-pose hardware-left-minus-own-right gap; converted to clearance on the same binary plane",
              "target_hardware_clearance_m": target, "target_minus_absolute_donor_clearance_m": target-donor_h,
              "median_gap_vs_difference_of_medians_AB06_m": gap-(donor_h+origin[1]-right["AB06"]), "cases": cases,
              "pylon_projection_slope": {"minimum": float(slopes.min()), "median": float(np.median(slopes)), "maximum": float(slopes.max())},
              "identification_limits": ["This scalar target transports a model-internal bilateral visual-surface benchmark, not physical floor, shoe thickness or prosthetic fitting measurements.",
                  "AB06 per-pose gap median is added to AB07 right-surface median; it is not an exact per-pose AB07 gap equation. Median-of-differences and difference-of-medians are explicitly distinguished.",
                  "Both own-right visual minima use healthy raw static IK and generic scaled talus/calcn/toes meshes. No acquisition-specific sole/hardware fiducials or static GRF independently identify assembly.",
                  "Changing the target changes conditional mounting length; neither this surrogate nor the absolute donor target is independently established as the correct physical target.",
                  "No clinical assembly recommendation, healthy-height-order constraint or GRF/dynamic causality claim follows."],
              "inputs_integrity": {"count": len(before), "all_unchanged": True, "before": before, "after": after}}
    with OUTPUT.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, allow_nan=False); stream.write("\n")
    print(json.dumps({"status": result["status"], "cases": cases, "parity": parity, "output_sha256": sha(OUTPUT)}, indent=2))


if __name__ == "__main__":
    main()
