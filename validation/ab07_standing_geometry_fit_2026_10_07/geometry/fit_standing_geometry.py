"""Isolated subject standing geometry fit; no IK, force fit, or dynamics.

--prepare writes the prospective source-bound protocol without native FK.
The fit modifies only two mounting Y translations in a NEW XML model. A
static ankle reference supplies the third geometric degree of freedom.
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
from scipy.optimize import brentq

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
PROTOCOL = HERE / "geometry_protocol.json"
SCOPE = HERE.parent / "protocol.json"
RECEIPT = HERE / "fit_geometry_receipt.json"
MODEL_OUTPUT = HERE / "AB07_SEASEA_standing_fit.osim"
PURE = ROOT / "validation/ab07_mount_registration_verification_2026_10_07/constraints/relative_right_target_supplement.py"
DECODER = ROOT / "validation/ab06_ab07_preparation_vs_model_2026_10_07/data_preparation_audit.py"
CACHE = ROOT / "validation/ab06_ab07_independent_attribution_2026_10_07/rebuild/models/AB07_SEASEA-raw"
sys.path.insert(0, str(ROOT))
TRANS = {"pelvis_tx", "pelvis_ty", "pelvis_tz"}
RENAME = {"pros_knee_angle": "knee_angle_l", "pros_ankle_angle": "ankle_angle_l"}
ALLOWED_JOINTS = ("osseo", "pros_ankle")


def now():
    return datetime.now(timezone.utc).isoformat()


def fp(path):
    p = Path(path).resolve()
    return {"path": str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p),
            "sha256": hashlib.sha256(p.read_bytes()).hexdigest(), "size_bytes": p.stat().st_size}


def write_new(path, obj):
    with path.open("x", encoding="utf-8") as stream:
        json.dump(obj, stream, indent=2, allow_nan=False); stream.write("\n")


def module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    obj = importlib.util.module_from_spec(spec); sys.modules[name] = obj
    spec.loader.exec_module(obj)
    return obj


def plugin_path():
    name = "SEA_Plugin_BlackBox_mCMC_impedence_ff" if os.name == "nt" else "libSEA_Plugin_BlackBox_mCMC_impedence_ff"
    suffix = ".dll" if os.name == "nt" else ".dylib" if sys.platform == "darwin" else ".so"
    return ROOT / "plugins" / (name + suffix)


def roles():
    return {
        "helper": Path(__file__), "root_scope_protocol": SCOPE,
        "AGENTS": ROOT / "AGENTS.md", "pure_geometry": PURE,
        "raw_static_decoder": DECODER, "converter": ROOT / "tools/convert_epic_ab06_tables.py",
        "motion_reader_and_marker_calibrator": ROOT / "tools/calibrate_ab06_seasea_markers.py",
        "model_loader": ROOT / "model_loader.py",
        "healthy_model": ROOT / "models/AB07-raw/osimxml/ab07.osim",
        "raw_static_IK": ROOT / "models/AB07-raw/10_14_18/static/ik/static01.mat",
        "raw_static_markers": ROOT / "models/AB07-raw/10_14_18/static/markers/static01.mat",
        "raw_static_TRC": CACHE / "data/converted/static/static01/static01.trc",
        "raw_remapped_static": CACHE / "data/converted/static/static01/static01_ik_dataset_ab06_seasea.mot",
        "anatomical_model_before_donor_ground_registration": CACHE / "osimxml/AB07_SEASEA_anatomical.osim",
        "fixed_foot_STL": ROOT / "Geometry/AM_foot_l.STL",
        "SEA_plugin_current_platform": plugin_path(),
        "fixed_primary_contact_profile": ROOT / "online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json",
        "fixed_binary_detector_profile": ROOT / "validation/binary_phase_detector_v25_geometry_runs/2026-08-04_local_reach_sweep_dev02_04_08/selected_candidate_profile.json",
    }


def prepare():
    sources = {name: fp(path) for name, path in roles().items()}
    decoder = module(DECODER, "standing_prepare_frozen_static_decoder")
    pure = module(PURE, "standing_prepare_pure_frozen_geometry")
    table, metadata = decoder.generic_table(roles()["raw_static_IK"])
    healthy_q = {name: float(np.median(table.column(name))) for name in table.names[1:]}
    labels, _ = pure.motion(roles()["raw_remapped_static"])
    initial_prepared_q = {name: healthy_q[RENAME.get(name, name)] for name in labels[1:]}
    if sources != {name: fp(path) for name, path in roles().items()}:
        raise ValueError("Source changed during protocol preparation")
    protocol = {
        "created_utc": now(), "instruction_check_token": "CMC_AGENT_OK_2026",
        "status": "prospective_contract_and_sources_bound_before_geometry_fit",
        "method": "own_healthy_raw_static_median_knee_height_and_STL_heel_forefoot_support_v1",
        "reference": {"selection": "Coordinate-by-coordinate median of all436 own healthy static rows; FK the resulting full23-coordinate pose, not median of centerXYZ from different poses.",
                      "units": "degrees for rotations; meters for pelvis translations",
                      "time": "Median original Header timestamp; no physical integration",
                      "biological_preservation": "All19 shared biological reference coordinates remain exact; knee maps by name, ankle is the sole static angular fit variable; only left subtalar/MTP are absent from graft.",
                      "standing_definition": "Observed acquisition quiet-standing representative, not knee-locked/zero-coordinate construction."},
        "ground": {"origin_m": [0., 0., 0.], "normal": [0., 1., 0.],
                   "meaning": "Explicit model/physical benchmark Y=0 assumption; raw COPY=0 is a convention, not independent measured floor elevation.",
                   "healthy_right_or_pelvis_translation": "None. No generic-bone visual minimum shifts source pose.",
                   "actual_primary_and_binary_planes": "Read and disclose separately; both profiles stay byte-frozen."},
        "support_regions": {"coordinates": "Unscaled fixed foot_l local STL coordinates; X fore-aft, Y up, Z lateral",
                            "heel": "x <= xmin + 0.25*(xmax-xmin)", "forefoot": "x >= xmax - 0.35*(xmax-xmin)",
                            "heel_fraction": .25, "forefoot_fraction": .35,
                            "fully_supported_definition": "One minimum in each region on Y=0 and no vertex of the entire rigid STL below plane; curved mesh need not have every vertex coplanar.",
                            "limitation": "Two-region support does not certify lateral full-sole flatness, clinical alignment, finite contact patch or supported body weight."},
        "fit": {"free_XML_scalars": ["osseo parent PhysicalOffsetFrame translation.Y", "pros_ankle parent PhysicalOffsetFrame translation.Y"],
                "free_static_scalar": "pros_ankle_angle at reference pose",
                "allowed_orientation_changes": [], "target_knee": "Healthy knee_l CHILD frame origin Y at full reference pose; XYZ disclosed, X/Z not constrained.",
                "ankle_target": "No healthy ankle-height constraint; ankle height follows grounded identical prosthetic foot.",
                "angular_search_grid_count": 4097, "range": "Unchanged pros_ankle coordinate range",
                "root_choice": "Enumerate endpoint/grid zero and sign-changing Brent roots; reject roots with whole-STL penetration or non-distal mounts, then choose root nearest healthy ankle reference. No force/dynamic outcome enters choice.",
                "root_search_limit": "Dense brackets do not certify absence of unsampled tangential roots; failure is preserved without silently introducing adapters.",
                "positive_mount_slopes_required": True, "mounts_must_remain_negative_Y": True},
        "tolerances": {"geometry_m": 1e-10, "native_parity_m": 1e-11, "rotation_dimensionless": 1e-12,
                       "coordinate_assignment_rad_or_m": 1e-14, "Brent_xtol_rad": 1e-13},
        "static_calibration_mapping": "For all original rows preserve biological values and Header clock, map knee/ankle by name and add preparedRef - healthyRef; drop only left subtalar/MTP. No donor registration or automatic old+4.73065 degree offset.",
        "reference_coordinate_maps": {"healthy_deg_m": healthy_q,
                                      "prepared_before_fit_deg_m": initial_prepared_q},
        "raw_static_metadata": metadata,
        "preserved": ["All XML except two parent-frame Y scalars", "all orientations/Pin axes", "SEA/controller semantics", "joint defaults/ranges", "hardware mesh/scale", "biological anatomy/muscles", "benchmark masses/COM/inertia", "raw marker/TRC data", "detector/contact profiles"],
        "prohibited": ["IK/build in this helper", "donor registration", "native integration", "force fit", "plugin modification", "production or old-audit writes", "new Claude directory reads"],
        "sources_before": sources,
        "outputs": {"model": str(MODEL_OUTPUT.relative_to(ROOT)), "receipt": str(RECEIPT.relative_to(ROOT))},
    }
    write_new(PROTOCOL, protocol)
    print("PREPARED", fp(PROTOCOL))


def vector(value):
    return np.array([value.get(i) for i in range(3)])


def rotation(value):
    return np.array([[value.get(i, j) for j in range(3)] for i in range(3)])


def set_pose(model, q, time_s, tol):
    state = model.initSystem(); state.setTime(time_s)
    coords = model.getCoordinateSet()
    if {coords.get(i).getName() for i in range(coords.getSize())} != set(q):
        raise ValueError("Every model coordinate must be provided exactly once")
    error = 0.
    for name, value in q.items():
        requested = value if name in TRANS else math.radians(value)
        coord = coords.get(name)
        if requested < coord.getRangeMin() - tol or requested > coord.getRangeMax() + tol:
            raise ValueError(f"Reference coordinate outside unchanged range: {name}")
        coord.setValue(state, requested, False)
        error = max(error, abs(coord.getValue(state) - requested))
    if error > tol:
        raise ValueError("Native coordinate assignment changed/clamped a value")
    model.realizePosition(state)
    return state, error


def parent_element(tree, name):
    joint = tree.find(f".//JointSet/objects/*[@name='{name}']")
    socket = joint.findtext("socket_parent_frame")
    return joint.find(f"frames/PhysicalOffsetFrame[@name='{socket}']/translation")


def normalized_except_mounts(tree, canonical):
    copy = deepcopy(tree)
    for name in ALLOWED_JOINTS:
        element = parent_element(copy, name)
        parts = element.text.split(); parts[1] = "0"; element.text = " ".join(parts)
    return canonical(copy.getroot())


def fit():
    if RECEIPT.exists() or MODEL_OUTPUT.exists():
        raise FileExistsError("Existing new-audit fit outputs are immutable")
    protocol_fp = fp(PROTOCOL)
    protocol = json.loads(PROTOCOL.read_text())
    paths = roles()
    before = {name: fp(path) for name, path in paths.items()}
    if before != protocol["sources_before"]:
        raise ValueError("A frozen fit input/helper changed")
    pure = module(PURE, "standing_fit_pure_frozen_geometry")
    decoder = module(DECODER, "standing_fit_frozen_static_decoder")
    from tools.convert_epic_ab06_tables import IK_SOURCE_COLUMNS
    from tools.calibrate_ab06_seasea_markers import load_plugin
    from tools.fit_seasea_subject import _canonical
    from model_loader import _configure_geometry_search_paths
    import opensim
    tol = protocol["tolerances"]
    table, metadata = decoder.generic_table(paths["raw_static_IK"])
    if table.names != tuple(IK_SOURCE_COLUMNS) or table.nrows != 436:
        raise ValueError("Frozen own static schema/count differs")
    time_values = table.column(table.names[0])
    if not np.all(np.diff(time_values) > 0):
        raise ValueError("Original static Header clock is not increasing")
    healthy_q = {name: float(np.median(table.column(name))) for name in table.names[1:]}
    if healthy_q != protocol["reference_coordinate_maps"]["healthy_deg_m"]:
        raise ValueError("Frozen healthy reference coordinate map differs")
    time_s = float(np.median(time_values))
    labels, remapped = pure.motion(paths["raw_remapped_static"])
    remap_error = max(float(np.max(np.abs(remapped[:, i] - table.column("Header" if name == "time" else RENAME.get(name, name)))))
                      for i, name in enumerate(labels))
    if remap_error > 1e-8:
        raise ValueError("Converted static remap differs from raw subject data")
    prepared_q = {name: healthy_q[RENAME.get(name, name)] for name in labels[1:]}
    if prepared_q != protocol["reference_coordinate_maps"]["prepared_before_fit_deg_m"]:
        raise ValueError("Frozen complete initial prosthetic reference map differs")
    load_plugin(paths["SEA_plugin_current_platform"])
    _configure_geometry_search_paths(str(paths["anatomical_model_before_donor_ground_registration"]))
    healthy = opensim.Model(str(paths["healthy_model"]))
    original = opensim.Model(str(paths["anatomical_model_before_donor_ground_registration"]))
    if healthy.getConstraintSet().getSize() or original.getConstraintSet().getSize():
        raise ValueError("No assemble or hidden kinematic constraints allowed")
    healthy_state, healthy_assign = set_pose(healthy, healthy_q, time_s, tol["coordinate_assignment_rad_or_m"])
    original_state, original_assign = set_pose(original, prepared_q, time_s, tol["coordinate_assignment_rad_or_m"])
    target = vector(healthy.getJointSet().get("knee_l").getChildFrame().getTransformInGround(healthy_state).p())
    source_tree = ET.parse(paths["anatomical_model_before_donor_ground_registration"])
    if source_tree.findtext("Model/length_units") != "meters":
        raise ValueError("Explicit model meters required")
    joints = {e.get("name"): e for e in source_tree.findall(".//JointSet/objects/*")}
    for name, tag in (("osseo", "WeldJoint"), ("pros_knee", "PinJoint"), ("pros_ankle", "PinJoint")):
        if joints[name].tag != tag or np.any(pure.frame(joints[name], "child") != 0):
            raise ValueError("Frozen SEA weld/Pin/zero-child hardware topology differs")
        pure.frame(joints[name], "parent")  # validates all orientations remain zero
    mesh = source_tree.find(".//BodySet/objects/Body[@name='foot_l']/attached_geometry/Mesh")
    if mesh.findtext("socket_frame", "..").strip() != ".." or Path(mesh.findtext("mesh_file")).name.casefold() != "am_foot_l.stl" or np.fromstring(mesh.findtext("scale_factors"), sep=" ").tolist() != [1., 1., 1.]:
        raise ValueError("Unscaled identical directly-attached foot required")
    vertices = pure.vertices(paths["fixed_foot_STL"])
    span = float(np.ptp(vertices[:, 0])); xmin, xmax = float(vertices[:, 0].min()), float(vertices[:, 0].max())
    heel_mask = vertices[:, 0] <= xmin + protocol["support_regions"]["heel_fraction"] * span
    fore_mask = vertices[:, 0] >= xmax - protocol["support_regions"]["forefoot_fraction"] * span
    if not heel_mask.any() or not fore_mask.any():
        raise ValueError("Empty declared support region")
    upper_r = rotation(original.getBodySet().get("transfemur").getTransformInGround(original_state).R())
    tibia_r = rotation(original.getBodySet().get("tibia_pylon").getTransformInGround(original_state).R())
    old_knee = vector(original.getJointSet().get("pros_knee").getChildFrame().getTransformInGround(original_state).p())
    osseo_old = pure.frame(joints["osseo"], "parent")
    ankle_old = pure.frame(joints["pros_ankle"], "parent")
    if upper_r[1, 1] <= 1e-10 or tibia_r[1, 1] <= 1e-10 or np.any(ankle_old[[0, 2]] != 0.):
        raise ValueError("Positive longitudinal mounting sensitivity required")
    osseo_new = osseo_old.copy(); osseo_new[1] += (target[1] - old_knee[1]) / upper_r[1, 1]
    knee_new = old_knee + upper_r[:, 1] * (osseo_new[1] - osseo_old[1])
    ankle_coordinate = original.getCoordinateSet().get("pros_ankle_angle")
    lo, hi = ankle_coordinate.getRangeMin(), ankle_coordinate.getRangeMax()
    if not np.array_equal([lo, hi], [-math.pi / 2, math.pi / 2]):
        raise ValueError("Unexpected frozen ankle range")

    def projected(a):
        r = tibia_r @ pure.rz(a)
        heights = vertices @ r[1]
        return r, heights, float(heights[heel_mask].min()), float(heights[fore_mask].min())

    def gap(a):
        _, _, heel, fore = projected(a)
        return heel - fore

    grid = np.linspace(lo, hi, protocol["fit"]["angular_search_grid_count"])
    values = np.array([gap(a) for a in grid])
    roots = [float(grid[i]) for i in np.flatnonzero(np.abs(values) <= tol["geometry_m"])]
    for i in np.flatnonzero(values[:-1] * values[1:] < 0):
        roots.append(float(brentq(gap, grid[i], grid[i + 1], xtol=tol["Brent_xtol_rad"])))
    unique = []
    for root in sorted(roots):
        if not unique or abs(root - unique[-1]) > 1e-9:
            unique.append(root)
    candidates = []
    for root in unique:
        r, heights, heel, fore = projected(root)
        ankle_y = (protocol["ground"]["origin_m"][1] - knee_new[1] - heel) / tibia_r[1, 1]
        world_y = knee_new[1] + tibia_r[1, 1] * ankle_y + heights
        feasible = bool(osseo_new[1] < 0 and ankle_y < 0 and world_y.min() >= -tol["geometry_m"]
                        and abs(world_y[heel_mask].min()) <= tol["geometry_m"]
                        and abs(world_y[fore_mask].min()) <= tol["geometry_m"])
        candidates.append({"ankle_rad": root, "ankle_deg": math.degrees(root), "ankle_mount_y_m": ankle_y,
                           "whole_STL_min_y_m": float(world_y.min()), "heel_min_y_m": float(world_y[heel_mask].min()),
                           "forefoot_min_y_m": float(world_y[fore_mask].min()), "feasible": feasible})
    valid = [c for c in candidates if c["feasible"]]
    if not valid:
        write_new(RECEIPT, {"status": "failed", "stage": "geometry_support_root", "protocol": protocol_fp,
                           "target_healthy_knee_XYZ_m": target.tolist(), "candidates": candidates,
                           "output_model": None, "failure": "No declared ankle root provides two-region support without penetration and distal mounts; no alignment fallback used.",
                           "sources_before": before, "sources_after": {name: fp(path) for name, path in paths.items()}})
        raise ValueError("No feasible declared standing support root")
    chosen = min(valid, key=lambda c: abs(c["ankle_deg"] - healthy_q["ankle_angle_l"]))
    prepared_q["pros_ankle_angle"] = chosen["ankle_deg"]
    ankle_new = ankle_old.copy(); ankle_new[1] = chosen["ankle_mount_y_m"]
    fitted_tree = deepcopy(source_tree)
    for name, value in (("osseo", osseo_new), ("pros_ankle", ankle_new)):
        parent_element(fitted_tree, name).text = " ".join(format(float(v), ".17g") for v in value)
    if normalized_except_mounts(source_tree, _canonical) != normalized_except_mounts(fitted_tree, _canonical):
        raise ValueError("Unexpected XML change outside two parent Y scalars")
    # New isolated XML is retained if a subsequent native check fails.
    with MODEL_OUTPUT.open("xb") as stream:
        fitted_tree.write(stream, encoding="utf-8", xml_declaration=True)
    fitted = opensim.Model(str(MODEL_OUTPUT))
    fitted_state, fitted_assign = set_pose(fitted, prepared_q, time_s, tol["coordinate_assignment_rad_or_m"])
    native_knee = vector(fitted.getJointSet().get("pros_knee").getChildFrame().getTransformInGround(fitted_state).p())
    native_foot = fitted.getBodySet().get("foot_l").getTransformInGround(fitted_state)
    native_r, native_p = rotation(native_foot.R()), vector(native_foot.p())
    analytic_r, _, _, _ = projected(chosen["ankle_rad"])
    analytic_p = knee_new + tibia_r @ ankle_new
    world = vertices @ native_r.T + native_p
    parity = {"knee_XYZ_max_error_m": float(np.max(np.abs(native_knee - knee_new))),
              "foot_origin_XYZ_max_error_m": float(np.max(np.abs(native_p - analytic_p))),
              "foot_rotation_max_error_dimensionless": float(np.max(np.abs(native_r - analytic_r))),
              "coordinate_assignment_max_error_rad_or_m": max(healthy_assign, original_assign, fitted_assign)}
    geometry = {"healthy_knee_target_XYZ_m": target.tolist(), "fitted_knee_XYZ_m": native_knee.tolist(),
                "knee_target_residual_XYZ_m": (native_knee - target).tolist(),
                "healthy_ankle_XYZ_m_diagnostic": vector(healthy.getJointSet().get("ankle_l").getParentFrame().getTransformInGround(healthy_state).p()).tolist(),
                "healthy_right_ankle_XYZ_m_diagnostic": vector(healthy.getJointSet().get("ankle_r").getParentFrame().getTransformInGround(healthy_state).p()).tolist(),
                "fitted_ankle_XYZ_m": native_p.tolist(), "foot_rotation_ground": native_r.tolist(),
                "whole_STL_min_Y_m": float(world[:, 1].min()), "heel_min_Y_m": float(world[heel_mask, 1].min()),
                "forefoot_min_Y_m": float(world[fore_mask, 1].min()),
                "heel_witness_XYZ_m": world[np.flatnonzero(heel_mask)[np.argmin(world[heel_mask, 1])]].tolist(),
                "forefoot_witness_XYZ_m": world[np.flatnonzero(fore_mask)[np.argmin(world[fore_mask, 1])]].tolist(),
                "foot_sagittal_pitch_deg": math.degrees(math.atan2(native_r[1, 0], native_r[1, 1])),
                "foot_lateral_roll_diagnostic_deg": math.degrees(math.asin(-native_r[1, 2])),
                "support_vertices_at_declared_tolerance": int(np.count_nonzero(np.abs(world[:, 1]) <= tol["geometry_m"]))}
    checks = {"knee_height_target": abs(native_knee[1] - target[1]) <= tol["geometry_m"],
              "heel_and_forefoot_grounded": max(abs(geometry["heel_min_Y_m"]), abs(geometry["forefoot_min_Y_m"])) <= tol["geometry_m"],
              "whole_STL_nonpenetration": geometry["whole_STL_min_Y_m"] >= -tol["geometry_m"],
              "native_position_parity": max(parity["knee_XYZ_max_error_m"], parity["foot_origin_XYZ_max_error_m"]) <= tol["native_parity_m"] and parity["foot_rotation_max_error_dimensionless"] <= tol["rotation_dimensionless"],
              "protected_XML_exact": True}
    primary = json.loads(paths["fixed_primary_contact_profile"].read_text())
    binary = json.loads(paths["fixed_binary_detector_profile"].read_text())
    after = {name: fp(path) for name, path in paths.items()}
    checks["frozen_inputs_unchanged"] = before == after and fp(PROTOCOL) == protocol_fp
    receipt = {
        "schema_version": 1, "created_utc": now(), "status": "completed" if all(checks.values()) else "failed",
        "instruction_check_token": "CMC_AGENT_OK_2026", "scope_protocol": before["root_scope_protocol"],
        "protocol": protocol_fp, "output_model": fp(MODEL_OUTPUT),
        "method": protocol["method"], "OpenSim_version": opensim.GetVersionAndDate(),
        "reference_coordinate_maps": {"healthy_deg_m": healthy_q, "prepared_deg_m": prepared_q},
        "reference_time_s": time_s, "raw_static_source_metadata": metadata,
        "source_static_rows": table.nrows, "source_static_time_s": time_values[[0, -1]].tolist(),
        "raw_to_remapped_max_abs_error": remap_error,
        "static_angular_offsets_deg": {name: prepared_q[name] - healthy_q[own] for name, own in RENAME.items()},
        "mounts": {"osseo_parent_translation_before_m": osseo_old.tolist(), "osseo_parent_translation_after_m": osseo_new.tolist(),
                   "pros_ankle_parent_translation_before_m": ankle_old.tolist(), "pros_ankle_parent_translation_after_m": ankle_new.tolist(),
                   "distal_pylon_length_m": -float(ankle_new[1]), "all_orientation_fields_unchanged": True},
        "ankle_root_candidates": candidates, "chosen_root": chosen,
        "support_regions": {"heel_X_max_m": xmin + .25 * span, "forefoot_X_min_m": xmax - .35 * span,
                            "heel_vertex_count": int(heel_mask.sum()), "forefoot_vertex_count": int(fore_mask.sum()), "all_vertex_count": len(vertices)},
        "geometry": geometry, "native_parity": parity, "checks": checks,
        "profile_planes_unchanged": {"fit_benchmark": protocol["ground"], "primary_force_ground": primary["ground"], "binary_ground": binary["ground"]},
        "geometry_allowed_XML_changes": protocol["fit"]["free_XML_scalars"],
        "sources_before": before, "sources_after": after, "new_integrations": 0, "new_IK_solves": 0,
        "donor_ground_registration_called": False, "new_Claude_directory_read": False,
        "equations": {"knee": "Knew=Kold+Rupper*ey*(osseoYnew-osseoYold); solve KnewY=healthyKneeY",
                      "foot": "Rfoot=Rtibia*Rz(ankle); pfoot=Knew+Rtibia*[0,ankleMountY,0]",
                      "support": "minheel (Rfoot*v)Y=minforefoot (Rfoot*v)Y; ankleMountY=(planeY-KnewY-minheel(Rfoot*v)Y)/RtibiaYY"},
        "limits": ["Geometry fit alone does not prove load support, clinical assembly, physical floor identification or biological/detector qualification.",
                   "Knee target constrains Y only; preserved fixed lateral hardware offsets can leave X/Z residuals.",
                   "Curved rigid STL supports heel and forefoot minima; lateral flatness, finite support area and all-static-row grounding are not imposed.",
                   "Mass/COM/inertia, hardware housings, local SEA axes, coordinate defaults/ranges and force/detector profiles retain the benchmark convention.",
                   "The reference is observed quiet standing with nonzero knee flexion, not a zero-coordinate/knee-locked construction."]}
    write_new(RECEIPT, receipt)
    print(json.dumps({"status": receipt["status"], "receipt": fp(RECEIPT), "mounts": receipt["mounts"],
                      "geometry": geometry, "offsets": receipt["static_angular_offsets_deg"]}, indent=2))
    if receipt["status"] != "completed":
        raise ValueError("Fitted native geometry checks failed; retained outputs must not feed preparation")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prepare", action="store_true")
    args = parser.parse_args()
    if args.prepare:
        prepare()
    else:
        try:
            fit()
        except Exception as exc:
            if not RECEIPT.exists():
                write_new(RECEIPT, {"created_utc": now(), "status": "failed", "stage": "geometry_helper_exception",
                                   "failure": f"{type(exc).__name__}: {exc}",
                                   "scope_protocol": fp(SCOPE) if SCOPE.exists() else None,
                                   "protocol": fp(PROTOCOL) if PROTOCOL.exists() else None,
                                   "output_model": fp(MODEL_OUTPUT) if MODEL_OUTPUT.exists() else None,
                                   "no_silent_alignment_fallback": True})
            raise
