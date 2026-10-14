"""Prospective FOURROI geometry-only supplement; no force/event/IK input.

One added osseo parent roll rotates the complete hardware assembly relative
to the biological upper body. Internal SEA/foot joint frames remain fixed.
--prepare binds criteria and source bytes before any optimization/native FK.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
from xml.etree import ElementTree as ET

import numpy as np
from scipy.optimize import minimize
from scipy.spatial import ConvexHull

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
BASE = HERE.parent
PROTOCOL = HERE / "protocol.json"
OUTPUT = HERE / "fit_receipt.json"
MODEL = HERE / "AB07_SEASEA_four_roi_grounded.osim"
PURE = ROOT / "validation/ab07_mount_registration_verification_2026_10_07/constraints/relative_right_target_supplement.py"
sys.path.insert(0, str(ROOT))


def now():
    return datetime.now(timezone.utc).isoformat()


def path(raw):
    p = Path(raw); return p if p.is_absolute() else ROOT / p


def fp(p):
    p = Path(p).resolve()
    return {"path": str(p.relative_to(ROOT)), "sha256": hashlib.sha256(p.read_bytes()).hexdigest(), "size_bytes": p.stat().st_size}


def write_new(p, obj):
    # Serialize before exclusive file creation, so no partial NumPy-bool sink.
    payload = json.dumps(obj, indent=2, allow_nan=False) + "\n"
    with p.open("x", encoding="utf-8") as stream:
        stream.write(payload)


def source_roles():
    base = json.loads((BASE / "fit_geometry_receipt_completed.json").read_text())
    result = {"helper": Path(__file__), "base_completed_receipt": BASE / "fit_geometry_receipt_completed.json",
              "base_fitted_model": path(base["output_model"]["path"]), "base_sole_diagnostic": BASE / "saved_sole_support_diagnostic.json",
              "base_geometry_protocol": BASE / "geometry_protocol_v2.json", "root_scope_protocol": BASE.parent / "protocol.json",
              "pure_geometry": PURE, "canonical_XML_helper": ROOT / "tools/fit_seasea_subject.py",
              "model_loader": ROOT / "model_loader.py", "plugin_loader": ROOT / "tools/calibrate_ab06_seasea_markers.py"}
    for role in ("fixed_foot_STL", "SEA_plugin_current_platform", "fixed_primary_contact_profile", "fixed_binary_detector_profile"):
        result[role] = path(base["sources_before"][role]["path"])
    return result


def prepare():
    roles = source_roles(); before = {k: fp(v) for k, v in roles.items()}
    base = json.loads(roles["base_completed_receipt"].read_text())
    diag = json.loads(roles["base_sole_diagnostic"].read_text())
    if base["status"] != "completed" or diag["status"] != "PASS":
        raise ValueError("Completed base geometry and diagnostic required")
    roi = {}
    for name in ("heel", "forefoot"):
        bounds = diag["edge_heights"][name]["local_Z_bounds_m"]
        span = bounds[1] - bounds[0]
        roi[name + "_negative_Z_lateral"] = {"region": name, "Z_max_m": bounds[0] + .2 * span}
        roi[name + "_positive_Z_medial"] = {"region": name, "Z_min_m": bounds[1] - .2 * span}
    protocol = {
        "created_utc": now(), "status": "supplement_contract_and_sources_frozen_before_first_FOURROI_evaluation",
        "instruction_check_token": "CMC_AGENT_OK_2026",
        "authorization": "Root authorized reversible geometry-only FOURROI extension after completed two-region fit; no additional IK/dynamic case is authorized here.",
        "reference": {"coordinate_maps": base["reference_coordinate_maps"], "healthy_knee_XYZ_m": base["geometry"]["healthy_knee_target_XYZ_m"],
                      "time_s": base["reference_time_s"], "biological_pose_change": "None; observed raw-static median kept exact."},
        "ground_Y_m": 0., "ground_meaning": "Explicit benchmark, not independently measured acquisition floor",
        "four_ROI": roi, "heel_X_max_m": base["support_regions"]["heel_X_max_m"],
        "forefoot_X_min_m": base["support_regions"]["forefoot_X_min_m"],
        "variables": {"osseo_parent_orientation_X_rad": [-math.pi / 12, math.pi / 12],
                      "static_pros_ankle_rad": [-math.pi / 2, math.pi / 2],
                      "dependent_mounts": "osseo parentY solves healthy kneeY; pros_ankle parentY solves whole-STL minY=0; both remain distal/negative"},
        "XML_whitelist": ["osseo parent translation.Y", "osseo parent orientation.X", "pros_ankle parent translation.Y"],
        "preserved": ["all biological reference values", "all SEA INTERNAL frame orientations and Pin axes", "coordinate defaults/ranges", "hardware housings/STL/scale", "SEA/muscle properties", "benchmark mass/COM/inertia", "detector/contact profile bytes", "all old audits and base fitted model"],
        "objective": "Minimize maximum over four ROI of [ROI minimum projected height - whole-STL minimum projected height]; no force/load/event/reserve enters selection.",
        "search": {"alpha_grid_count": 61, "ankle_grid_count": 181,
                   "hull_seeds": "All finite lower-convex-hull facet normals with local-up Y>0, mapped analytically into allowed alpha/ankle branches",
                   "local_refinement": "Bounded Powell from best8 distinct seeds/grid candidates, maxiter200, maxfev2000, xtol1e-11, ftol1e-12",
                   "choice": "Smallest observed objective among candidates meeting mounts/bounds and actual noncollinear contact guard; ties use smallest alpha absolute value then smallest ankle departure from healthy reference",
                   "limit": "Finite search proves a passing witness exists; FAIL is best observed strict failure, not certified global unreachability/clinical infeasibility."},
        "strict": {"four_ROI_max_Y_m": 1e-5, "whole_STL_nonpenetration_m": 1e-10, "knee_Y_m": 1e-10,
                   "actual_contact_tolerance_m": 1e-10, "XZ_contact_second_singular_value_min_m": 1e-10,
                   "native_position_parity_m": 1e-11, "native_rotation_parity_dimensionless": 1e-12},
        "optional_diagnostic_only": {"four_ROI_max_Y_m": .001, "meaning": "1mm descriptive classification only; never replaces strictPASS or retunes selection"},
        "interpretation": ["Four-region minima within10micrometers are a CAD geometry support convention, not a measured noise/shoe/clinical tolerance.",
                           "At least3 actual-plane vertices with noncollinear XZ are required separately; near-contact ROI points are not silently counted as exact contact.",
                           "All curved sole vertices need not be coplanar; no body-weight support, finite-area pressure or functional/anatomical necessity claim."],
        "sources_before": before,
        "outputs": {"model": str(MODEL.relative_to(ROOT)), "receipt": str(OUTPUT.relative_to(ROOT))},
        "new_force_or_event_data_read": False, "new_Claude_directory_read": False,
    }
    if before != {k: fp(v) for k, v in roles.items()}:
        raise ValueError("Input changed during prospective preparation")
    write_new(PROTOCOL, protocol); print("PREPARED", fp(PROTOCOL))


def frame_element(tree, name, role="parent"):
    joint = tree.find(f".//JointSet/objects/*[@name='{name}']")
    socket = joint.findtext(f"socket_{role}_frame")
    return joint.find(f"frames/PhysicalOffsetFrame[@name='{socket}']")


def masked(tree, canonical):
    copy = deepcopy(tree)
    for name in ("osseo", "pros_ankle"):
        element = frame_element(copy, name).find("translation")
        tokens = element.text.split(); tokens[1] = "0"; element.text = " ".join(tokens)
    element = frame_element(copy, "osseo").find("orientation")
    tokens = element.text.split(); tokens[0] = "0"; element.text = " ".join(tokens)
    return canonical(copy.getroot())


def fit():
    if OUTPUT.exists() or MODEL.exists():
        raise FileExistsError("Existing FOURROI outputs are immutable")
    protocol_fp = fp(PROTOCOL); protocol = json.loads(PROTOCOL.read_text())
    roles = source_roles(); before = {k: fp(v) for k, v in roles.items()}
    if before != protocol["sources_before"]:
        raise ValueError("Frozen geometry-only supplement inputs changed")
    spec = importlib.util.spec_from_file_location("four_roi_pure_frozen", PURE)
    pure = importlib.util.module_from_spec(spec); spec.loader.exec_module(pure)
    from tools.fit_seasea_subject import _canonical
    base = json.loads(roles["base_completed_receipt"].read_text())
    tree = ET.parse(roles["base_fitted_model"])
    joints = {e.get("name"): e for e in tree.findall(".//JointSet/objects/*")}
    for name, tag in (("osseo", "WeldJoint"), ("pros_knee", "PinJoint"), ("pros_ankle", "PinJoint")):
        if joints[name].tag != tag or np.any(pure.frame(joints[name], "child") != 0.):
            raise ValueError("Unchanged internal assembly topology/zero child frames required")
        pure.frame(joints[name], "parent")
    osseo = pure.frame(joints["osseo"], "parent")
    knee_fixed = pure.frame(joints["pros_knee"], "parent")
    ankle = pure.frame(joints["pros_ankle"], "parent")
    if np.any(ankle[[0, 2]] != 0.):
        raise ValueError("Longitudinal distal connector required")
    q = dict(protocol["reference"]["coordinate_maps"]["prepared_deg_m"])
    healthy_q = protocol["reference"]["coordinate_maps"]["healthy_deg_m"]
    qk, qa_base = math.radians(q["pros_knee_angle"]), math.radians(q["pros_ankle_angle"])
    rupper = np.array(base["geometry"]["foot_rotation_ground"]) @ pure.rz(-qa_base - qk)
    base_knee = np.array(base["geometry"]["fitted_knee_XYZ_m"])
    hip = base_knee - rupper @ (osseo + knee_fixed)
    target = np.array(protocol["reference"]["healthy_knee_XYZ_m"])
    vertices = pure.vertices(roles["fixed_foot_STL"])
    region = {"heel": vertices[:, 0] <= protocol["heel_X_max_m"], "forefoot": vertices[:, 0] >= protocol["forefoot_X_min_m"]}
    masks = {}
    for name, settings in protocol["four_ROI"].items():
        mask = region[settings["region"]].copy()
        mask &= vertices[:, 2] <= settings["Z_max_m"] if "Z_max_m" in settings else vertices[:, 2] >= settings["Z_min_m"]
        if not mask.any():
            raise ValueError("Empty frozen FOURROI")
        masks[name] = mask
    bounds = [protocol["variables"]["osseo_parent_orientation_X_rad"], protocol["variables"]["static_pros_ankle_rad"]]
    strict = protocol["strict"]

    def evaluate(x, origin):
        alpha, qa = map(float, x)
        rtibia = rupper @ pure.rx(alpha) @ pure.rz(qk)
        rfoot = rtibia @ pure.rz(qa)
        projected = vertices @ rfoot[1]
        minimum = float(projected.min())
        heights = {name: float(projected[mask].min() - minimum) for name, mask in masks.items()}
        upper_t = osseo.copy(); upper_t[1] += (target[1] - (hip + rupper @ (upper_t + pure.rx(alpha) @ knee_fixed))[1]) / rupper[1, 1]
        knee = hip + rupper @ (upper_t + pure.rx(alpha) @ knee_fixed)
        distal_t = ankle.copy(); distal_t[1] = (protocol["ground_Y_m"] - knee[1] - minimum) / rtibia[1, 1]
        pfoot = knee + rtibia @ distal_t
        world = vertices @ rfoot.T + pfoot
        contact = np.flatnonzero(np.abs(world[:, 1] - protocol["ground_Y_m"]) <= strict["actual_contact_tolerance_m"])
        sv = np.linalg.svd(world[contact][:, [0, 2]] - world[contact][:, [0, 2]].mean(axis=0), compute_uv=False) if len(contact) else np.array([])
        noncollinear = bool(len(contact) >= 3 and len(sv) >= 2 and sv[1] > strict["XZ_contact_second_singular_value_min_m"])
        return {"alpha_rad": alpha, "ankle_rad": qa, "objective_m": max(heights.values()), "ROI_min_Y_m": heights,
                "origin": origin, "actual_contact_count": int(len(contact)), "XZ_contact_singular_values_m": sv.tolist(),
                "noncollinear_actual_contact": noncollinear,
                "mounts_distal": bool(upper_t[1] < 0 and distal_t[1] < 0 and rtibia[1, 1] > 1e-10),
                "osseo_translation_m": upper_t.tolist(), "ankle_translation_m": distal_t.tolist(),
                "knee_XYZ_m": knee.tolist(), "foot_origin_XYZ_m": pfoot.tolist(), "foot_rotation_ground": rfoot.tolist()}

    def objective(x):
        alpha, qa = x
        row = (rupper @ pure.rx(float(alpha)) @ pure.rz(qk + float(qa)))[1]
        h = vertices @ row
        return max(float(h[mask].min() - h.min()) for mask in masks.values())

    candidates = []
    # Coarse fixed grid is a search aid, never an inference from dynamic data.
    grid_best = []
    for alpha in np.linspace(*bounds[0], protocol["search"]["alpha_grid_count"]):
        for qa in np.linspace(*bounds[1], protocol["search"]["ankle_grid_count"]):
            grid_best.append((objective([alpha, qa]), float(alpha), float(qa)))
    for _, alpha, qa in sorted(grid_best)[:8]:
        candidates.append(evaluate([alpha, qa], "fixed_grid_seed"))
    hull = ConvexHull(vertices)
    a, b, c = rupper[1]; amplitude = math.hypot(float(b), float(c)); phase = math.atan2(float(c), float(b))
    for index, equation in enumerate(hull.equations):
        up = -equation[:3]
        if up[1] <= 0 or abs(up[2]) > amplitude:
            continue
        angle = math.asin(float(up[2]) / amplitude)
        for alpha0 in (phase - angle, phase - (math.pi - angle)):
            for turn in (-1, 0, 1):
                alpha = alpha0 + 2 * math.pi * turn
                if not bounds[0][0] <= alpha <= bounds[0][1]:
                    continue
                by = float(b * math.cos(alpha) + c * math.sin(alpha))
                beta = math.atan2(float(up[0]), float(up[1])) - math.atan2(float(a), by)
                for angular_turn in (-1, 0, 1):
                    qa = beta - qk + 2 * math.pi * angular_turn
                    if bounds[1][0] <= qa <= bounds[1][1]:
                        candidates.append(evaluate([alpha, qa], f"lower_hull_facet_{index}"))
    seeds = []
    for cand in sorted(candidates, key=lambda v: v["objective_m"]):
        if all(np.linalg.norm(np.array([cand["alpha_rad"], cand["ankle_rad"]]) - seed) > 1e-8 for seed in seeds):
            seeds.append(np.array([cand["alpha_rad"], cand["ankle_rad"]]))
        if len(seeds) == 8:
            break
    refinements = []
    for index, seed in enumerate(seeds):
        result = minimize(objective, seed, method="Powell", bounds=bounds,
                          options={"maxiter": 200, "maxfev": 2000, "xtol": 1e-11, "ftol": 1e-12})
        cand = evaluate(result.x, f"bounded_Powell_{index}")
        candidates.append(cand)
        refinements.append({"origin": cand["origin"], "success": bool(result.success), "message": str(result.message),
                            "evaluations": int(result.nfev), "objective_m": float(result.fun)})
    eligible = [v for v in candidates if v["mounts_distal"] and v["noncollinear_actual_contact"]]
    best_unrestricted = min(candidates, key=lambda v: v["objective_m"])
    if not eligible:
        write_new(OUTPUT, {"status": "failed", "stage": "no_noncollinear_support_witness", "protocol": protocol_fp,
                           "best_observed": best_unrestricted, "output_model": None,
                           "failure_is_global_impossibility_proof": False, "sources_before": before,
                           "sources_after": {k: fp(v) for k, v in roles.items()}})
        raise ValueError("No eligible geometric support witness")
    chosen = min(eligible, key=lambda v: (v["objective_m"], abs(v["alpha_rad"]), abs(v["ankle_rad"] - math.radians(healthy_q["ankle_angle_l"]))))
    fitted = deepcopy(tree)
    for name, values in (("osseo", chosen["osseo_translation_m"]), ("pros_ankle", chosen["ankle_translation_m"])):
        frame_element(fitted, name).find("translation").text = " ".join(format(v, ".17g") for v in values)
    frame_element(fitted, "osseo").find("orientation").text = format(chosen["alpha_rad"], ".17g") + " 0 0"
    protected = masked(tree, _canonical) == masked(fitted, _canonical)
    if not protected:
        raise ValueError("XML changed outside authorized two Y + osseo parent X orientation")
    with MODEL.open("xb") as stream:
        fitted.write(stream, encoding="utf-8", xml_declaration=True)
    from tools.calibrate_ab06_seasea_markers import load_plugin
    from model_loader import _configure_geometry_search_paths
    import opensim
    load_plugin(roles["SEA_plugin_current_platform"]); _configure_geometry_search_paths(str(MODEL))
    model = opensim.Model(str(MODEL)); state = model.initSystem(); state.setTime(protocol["reference"]["time_s"])
    q["pros_ankle_angle"] = math.degrees(chosen["ankle_rad"])
    coords = model.getCoordinateSet()
    if model.getConstraintSet().getSize() or {coords.get(i).getName() for i in range(coords.getSize())} != set(q):
        raise ValueError("Full explicit unconstrained reference required")
    assignment = 0.
    for name, value in q.items():
        expected = value if name in ("pelvis_tx", "pelvis_ty", "pelvis_tz") else math.radians(value)
        coords.get(name).setValue(state, expected, False); assignment = max(assignment, abs(coords.get(name).getValue(state) - expected))
    model.realizePosition(state)
    transform = model.getBodySet().get("foot_l").getTransformInGround(state)
    native_r = np.array([[transform.R().get(i, j) for j in range(3)] for i in range(3)])
    native_p = np.array([transform.p().get(i) for i in range(3)])
    native_knee_frame = model.getJointSet().get("pros_knee").getChildFrame().getTransformInGround(state)
    native_knee = np.array([native_knee_frame.p().get(i) for i in range(3)])
    world = vertices @ native_r.T + native_p
    roi_native = {name: float(world[mask, 1].min() - protocol["ground_Y_m"]) for name, mask in masks.items()}
    contacts = np.flatnonzero(np.abs(world[:, 1] - protocol["ground_Y_m"]) <= strict["actual_contact_tolerance_m"])
    sv = np.linalg.svd(world[contacts][:, [0, 2]] - world[contacts][:, [0, 2]].mean(axis=0), compute_uv=False)
    parity = {"foot_origin_max_error_m": float(np.max(np.abs(native_p - chosen["foot_origin_XYZ_m"]))),
              "foot_rotation_max_error_dimensionless": float(np.max(np.abs(native_r - chosen["foot_rotation_ground"]))),
              "knee_XYZ_max_error_m": float(np.max(np.abs(native_knee - chosen["knee_XYZ_m"]))),
              "coordinate_assignment_max_error_rad_or_m": float(assignment)}
    after = {k: fp(v) for k, v in roles.items()}
    checks = {"four_ROI_strict_0p01mm": bool(max(roi_native.values()) <= strict["four_ROI_max_Y_m"]),
              "whole_STL_nonpenetration": bool(world[:, 1].min() - protocol["ground_Y_m"] >= -strict["whole_STL_nonpenetration_m"]),
              "healthy_knee_Y": bool(abs(native_knee[1] - target[1]) <= strict["knee_Y_m"]),
              "actual_XZ_contacts_noncollinear": bool(len(contacts) >= 3 and len(sv) >= 2 and sv[1] > strict["XZ_contact_second_singular_value_min_m"]),
              "native_position_parity": bool(max(parity["foot_origin_max_error_m"], parity["knee_XYZ_max_error_m"]) <= strict["native_position_parity_m"] and parity["foot_rotation_max_error_dimensionless"] <= strict["native_rotation_parity_dimensionless"] and assignment <= 1e-14),
              "XML_three_fields_only": bool(protected), "sources_unchanged": bool(before == after and fp(PROTOCOL) == protocol_fp)}
    result = {"created_utc": now(), "status": "completed" if all(checks.values()) else "failed", "strict_verdict": "PASS" if all(checks.values()) else "FAIL",
              "protocol": protocol_fp, "output_model": fp(MODEL), "OpenSim_version": opensim.GetVersionAndDate(),
              "reference_coordinate_maps": {"healthy_deg_m": healthy_q, "prepared_deg_m": q},
              "static_angular_offsets_deg": {"pros_knee_angle": q["pros_knee_angle"] - healthy_q["knee_angle_l"], "pros_ankle_angle": q["pros_ankle_angle"] - healthy_q["ankle_angle_l"]},
              "mounts": {"osseo_parent_translation_m": chosen["osseo_translation_m"], "osseo_parent_orientation_rad": [chosen["alpha_rad"], 0., 0.],
                         "osseo_parent_orientation_X_deg": math.degrees(chosen["alpha_rad"]), "pros_ankle_parent_translation_m": chosen["ankle_translation_m"], "pylon_length_m": -chosen["ankle_translation_m"][1]},
              "geometry": {"healthy_knee_target_XYZ_m": target.tolist(), "fitted_knee_XYZ_m": native_knee.tolist(),
                           "foot_origin_XYZ_m": native_p.tolist(), "foot_rotation_ground": native_r.tolist(),
                           "ROI_min_Y_m": roi_native, "whole_STL_min_Y_m": float(world[:, 1].min()),
                           "exact_contact_tolerance_m": strict["actual_contact_tolerance_m"], "actual_contact_count": len(contacts),
                           "XZ_contact_singular_values_m": sv.tolist(),
                           "actual_contacts": [{"index": int(i), "local_XYZ_m": vertices[i].tolist(), "ground_XYZ_m": world[i].tolist()} for i in contacts]},
              "checks": checks, "native_parity": parity, "chosen": chosen, "best_observed_unrestricted": best_unrestricted,
              "search_candidates": candidates, "Powell_refinements": refinements, "grid_evaluations": len(grid_best),
              "optional_1mm_classification_diagnostic_only": bool(max(roi_native.values()) <= .001),
              "sources_before": before, "sources_after": after, "new_IK_solves": 0, "new_integrations": 0,
              "new_force_event_reserve_outputs_read": False, "new_Claude_directory_read": False,
              "limits": protocol["interpretation"] + [protocol["search"]["limit"], "Assembly roll is a geometric adapter convention, not measured/clinically qualified hardware alignment; local SEA internal axes/properties remain unchanged."]}
    write_new(OUTPUT, result)
    print(json.dumps({"status": result["status"], "receipt": fp(OUTPUT), "mounts": result["mounts"], "ROI_min_Y_m": roi_native,
                      "actual_contact_count": len(contacts), "checks": checks, "parity": parity}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument("--prepare", action="store_true"); args = parser.parse_args()
    if args.prepare:
        prepare()
    else:
        try:
            fit()
        except Exception as exc:
            if not OUTPUT.exists():
                write_new(OUTPUT, {"created_utc": now(), "status": "failed", "strict_verdict": "FAIL", "stage": "helper_exception",
                                   "error": f"{type(exc).__name__}: {exc}", "protocol": fp(PROTOCOL) if PROTOCOL.exists() else None,
                                   "output_model": fp(MODEL) if MODEL.exists() else None, "no_fallback_or_relabel": True})
            raise
