"""Explore static hardware registration constraints without operational edits.

The fixed STL, two mounting translations and static prosthetic coordinate
offsets define counterfactual poses. The common AB06 static hardware reference
is a declared relative-registration convention, not a surveyed treadmill or
an anatomical ground truth. No detector bits, forces or events are fitted.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
from scipy.optimize import least_squares, minimize

from audit_registration_identifiability import (
    REPO, SUBJECTS, MARKERS, set_pose, transform, default_plugin,
    load_plugin, source, parse_trc,
)
from audit_profile_ground_contract import BINARY, PRIMARY, MESH, mesh_triangles
from audit_marker_geometry import stats
import opensim as osim


def rz(theta):
    theta = np.asarray(theta)
    r = np.zeros(theta.shape + (3, 3))
    r[..., 0, 0] = r[..., 1, 1] = np.cos(theta)
    r[..., 1, 0] = np.sin(theta)
    r[..., 0, 1] = -np.sin(theta)
    r[..., 2, 2] = 1
    return r


def parent_translation(tree, name):
    joint = tree.find(f".//JointSet/objects/*[@name='{name}']")
    socket = joint.findtext("socket_parent_frame")
    frame = joint.find(f"frames/PhysicalOffsetFrame[@name='{socket}']")
    if [float(x) for x in frame.findtext("orientation").split()] != [0, 0, 0]:
        raise ValueError("Expected aligned fixed hardware frames")
    return np.asarray([float(x) for x in frame.findtext("translation").split()])


def native_data(subject, raw):
    cfg = SUBJECTS[subject]
    model = osim.Model(str(cfg["model"]))
    state = model.initSystem()
    upper = model.getBodySet().get("transfemur")
    foot = model.getBodySet().get("foot_l")
    q = raw["coordinates_deg_or_m"]
    rs, ts, rf, tf = [], [], [], []
    for i in range(len(raw["time_s"])):
        row = {name: values[i] for name, values in q.items()}
        set_pose(model, state, row, prosthetic=True)
        r, t = transform(upper, state); rs.append(r); ts.append(t)
        r, t = transform(foot, state); rf.append(r); tf.append(t)
    tree = ET.parse(cfg["model"]).getroot()
    trc = parse_trc(cfg["static"])
    measured = np.stack([trc.markers[name] for name in MARKERS], axis=1)
    if len(measured) != len(rs):
        raise ValueError("Static coordinate and marker sample counts differ")
    return {"r_upper": np.stack(rs), "t_upper": np.stack(ts),
            "r_native": np.stack(rf), "t_native": np.stack(tf),
            "q_knee": np.deg2rad(q["knee_angle_l"]),
            "q_ankle": np.deg2rad(q["ankle_angle_l"]),
            "proximal": parent_translation(tree, "osseo"),
            "knee_fixed": parent_translation(tree, "pros_knee"),
            "distal": parent_translation(tree, "pros_ankle"),
            "measured": measured}


def forward(data, parameters):
    """[proximal Y delta m, distal Y delta m, knee/ankle offset rad]."""
    dp, dd, dk, da = parameters
    rp = data["r_upper"]
    rk = rp @ rz(data["q_knee"] + dk)
    rf = rk @ rz(data["q_ankle"] + da)
    upper = data["proximal"] + data["knee_fixed"] + [0, dp, 0]
    distal = data["distal"] + [0, dd, 0]
    tf = data["t_upper"] + np.einsum("nij,j->ni", rp, upper) + np.einsum("nij,j->ni", rk, distal)
    return rf, tf


def quantities(data, parameters, vertices, ground):
    r, t = forward(data, parameters)
    n, origin = np.asarray(ground["normal"]), np.asarray(ground["origin"])
    local_normal = np.einsum("j,njk->nk", n, r)
    sole = np.min(local_normal @ vertices.T, axis=1) + (t-origin) @ n
    pitch = np.arctan2(local_normal[:, 0], local_normal[:, 1])
    return r, t, sole, pitch


def describe(data, parameters, vertices, binary, primary):
    r, t, sole, pitch = quantities(data, parameters, vertices, binary["ground"])
    bn, bo = np.asarray(binary["ground"]["normal"]), np.asarray(binary["ground"]["origin"])
    points = np.asarray([p["location"] for p in binary["points"]])
    clearance = (np.einsum("nij,mj->nmi", r, points)+t[:, None, :]-bo) @ bn
    left = [s for s in primary["spheres"] if s["frame"] == "/bodyset/foot_l"]
    pn, po = np.asarray(primary["ground"]["normal"]), np.asarray(primary["ground"]["origin"])
    sphere_xyz = np.einsum("nij,mj->nmi", r, np.asarray([s["location"] for s in left]))+t[:, None, :]
    sphere_clearance = (sphere_xyz-po) @ pn-np.asarray([s["radius"] for s in left])
    measured_local = np.einsum("nmi,nij->nmj", data["measured"]-t[:, None, :], r)
    marker_local = measured_local.mean(axis=0)
    marker_pred = np.einsum("nij,mj->nmi", r, marker_local)+t[:, None, :]
    marker_distance = np.linalg.norm(marker_pred-data["measured"], axis=2)
    dp, dd, dk, da = parameters
    return {"parameters": {"proximal_y_delta_m": float(dp), "distal_y_delta_m": float(dd),
                            "static_knee_offset_deg": float(np.rad2deg(dk)), "static_ankle_offset_deg": float(np.rad2deg(da))},
            "candidate_mount_y_m": {"osseo": float(data["proximal"][1]+dp), "pros_ankle": float(data["distal"][1]+dd)},
            "visual_mesh_min_clearance_to_common_plane_m": stats(sole),
            "hardware_sagittal_pitch_deg": stats(np.rad2deg(pitch)),
            "binary_geometry_diagnostic": {p["name"]: {"clearance_m": stats(clearance[:, i]),
                "contact_fraction": float(np.mean(clearance[:, i] <= 0))} for i, p in enumerate(binary["points"])},
            "primary_geometry_diagnostic": {"max_penetration_m": stats(np.maximum(0, -sphere_clearance.min(axis=1))),
                "touching_spheres_count": stats(np.sum(sphere_clearance <= 0, axis=1)),
                "sphere_surface_clearance_m": {s["name"]: stats(sphere_clearance[:, i]) for i, s in enumerate(left)}},
            "counterfactual_recalibrated_five_marker_rms_m": stats(np.sqrt(np.mean(marker_distance**2, axis=1))),
            "counterfactual_marker_local_m": dict(zip(MARKERS, marker_local.tolist())),
            "scope": "Static geometric consequences only; no computed primary force, FSM or dynamic gait qualification"}


def fit_candidates(data, reference, vertices, binary, primary):
    target_height, target_pitch = reference
    def residual(p):
        _, _, sole, pitch = quantities(data, p, vertices, binary["ground"])
        return np.asarray([np.median(sole)-target_height, np.median(pitch)-target_pitch])
    results = {"baseline": describe(data, np.zeros(4), vertices, binary, primary)}
    # Preserve anthropometric upper mount and both remapped healthy angles.
    a = least_squares(lambda x: residual([0, x[0], 0, 0])[:1], [0.0],
                      bounds=([-.08], [.08]), xtol=1e-13, ftol=1e-13, gtol=1e-13)
    pa = np.asarray([0, a.x[0], 0, 0])
    # Hardware pitch and sole datum are constrained before marker calibration.
    b = least_squares(lambda x: residual([0, x[0], 0, x[1]]) * [1, .2], [0.0, 0.0],
                      bounds=([-.08, -.35], [.08, .35]), xtol=1e-13, ftol=1e-13, gtol=1e-13)
    pb = np.asarray([0, b.x[0], 0, b.x[1]])
    # Both mounts otherwise underdetermined: explicitly select minimum change.
    c = minimize(lambda x: float(x@x), [0.0, 0.0], method="SLSQP",
                 constraints={"type": "eq", "fun": lambda x: residual([x[0], x[1], 0, 0])[0]},
                 bounds=[(-.08, .08), (-.08, .08)], options={"ftol": 1e-14, "maxiter": 100})
    pc = np.asarray([*c.x, 0, 0])
    d = minimize(lambda x: float(x@x), [0.0, b.x[0]], method="SLSQP",
                 constraints={"type": "eq", "fun": lambda x: residual([x[0], x[1], 0, b.x[1]])[0]},
                 bounds=[(-.08, .08), (-.08, .08)], options={"ftol": 1e-14, "maxiter": 100})
    pd = np.asarray([*d.x, 0, b.x[1]])
    for name, p, solver in (("distal_only_sole_height", pa, a),
                            ("distal_plus_static_ankle_sole_height_and_pitch", pb, b),
                            ("two_mounts_minimum_change_sole_height", pc, c),
                            ("two_mounts_minimum_change_plus_static_ankle", pd, d)):
        if not solver.success:
            raise RuntimeError(f"Candidate solver failed: {name}: {solver.message}")
        results[name] = {**describe(data, p, vertices, binary, primary),
                         "target_residual": {"median_sole_height_m": float(residual(p)[0]),
                                             "median_pitch_deg": float(np.rad2deg(residual(p)[1]))},
                         "solver_success": True, "function_evaluations": int(solver.nfev)}
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=REPO/"validation/ab07_detector_investigation_2026_10_06/stage3/static_registration_candidates.json")
    parser.add_argument("--plugin", type=Path, default=default_plugin())
    args = parser.parse_args()
    if args.output.exists():
        parser.error("New receipt path required; previous artifacts are never overwritten")
    raw_path = REPO/"validation/ab07_detector_investigation_2026_10_06/stage2/static_healthy_coordinates.json"
    paths = {"script": Path(__file__).resolve(), "raw_coordinates": raw_path,
             "binary": BINARY, "primary": PRIMARY, "mesh": MESH, "plugin": args.plugin}
    for subject, cfg in SUBJECTS.items():
        paths.update({subject+"_model": cfg["model"], subject+"_static_trc": cfg["static"]})
    before = {name: source(path) for name, path in paths.items()}
    raw = json.loads(raw_path.read_text())
    binary, primary = json.loads(BINARY.read_text()), json.loads(PRIMARY.read_text())
    vertices = np.unique(mesh_triangles(MESH).reshape(-1, 3), axis=0)
    load_plugin(args.plugin); osim.Logger.setLevelString("error")
    data = {subject: native_data(subject, raw[subject]) for subject in SUBJECTS}
    parity = {}
    for subject, d in data.items():
        r, t = forward(d, np.zeros(4))
        parity[subject] = {"max_rotation_error": float(np.max(np.abs(r-d["r_native"]))),
                           "max_translation_error_m": float(np.max(np.abs(t-d["t_native"]))) }
        if max(parity[subject].values()) > 1e-12:
            raise AssertionError("Analytic chain does not reconstruct native fixed-hardware kinematics")
    _, _, sole, pitch = quantities(data["AB06"], np.zeros(4), vertices, binary["ground"])
    reference = (float(np.median(sole)), float(np.median(pitch)))
    results = {subject: fit_candidates(d, reference, vertices, binary, primary) for subject, d in data.items()}
    if max(abs(v) for case in results["AB06"].values() for v in case["parameters"].values()) > 1e-9:
        raise AssertionError("Reference-subject fitting must be identity")
    if before != {name: source(path) for name, path in paths.items()}:
        raise RuntimeError("An operational source changed during the diagnostic")
    receipt = {"schema_version": 1, "scope": "Exploratory static native-pose sampling and SciPy geometry fit; no operational edits, IK, dynamics or policy",
               "sources": before, "sources_unchanged": True, "analytic_native_parity": parity,
               "registration_reference": {"subject": "AB06", "sole_metric": "median over all static frames of minimum visual-STL vertex clearance",
                    "sole_clearance_m": reference[0], "sagittal_pitch_rad": reference[1],
                    "common_ground": binary["ground"],
                    "why_relative": "Same hardware, same virtual plane and AB06 exact-identity requirement; no independently surveyed physical sole/ground datum"},
               "subjects": results, "ab06_identity_pass": True,
               "limitations": ["These candidates are unselected exploratory registrations, not fixes validated by new IK or dynamic contact.",
                    "AB06 static hardware pitch transfer assumes comparable standing-foot registration; different measured anatomical foot placement can violate this assumption.",
                    "Visual STL is not a measured anatomical sole or the primary force-generating envelope.",
                    "One height constraint cannot identify two mounting lengths; minimum change is an explicit prior, not an independent anatomical measurement.",
                    "Static coordinate offsets belong to registration/calibration; they do not justify changing joint axes, controller gains, FSM thresholds or runtime policy.",
                    "Primary and binary geometry are evaluated, never included in fitting residuals; no force computed."]}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(receipt, indent=2, allow_nan=False)+"\n")
    print(f"Saved exploratory registration receipt: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
