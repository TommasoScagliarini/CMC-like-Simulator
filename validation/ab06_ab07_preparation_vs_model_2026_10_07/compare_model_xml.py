#!/usr/bin/env python3
"""Read saved XML only; distinguish subject inputs from graft conventions.

No OpenSim import, model construction, IK, detector or dynamics execution.
Only the requested new JSON audit is written. All consumed inputs are hashed
before and after. Run with Python 3.10+ on macOS or Windows.
"""
from __future__ import annotations

import argparse
import ast
from collections import Counter
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
from xml.etree import ElementTree as ET


RAW_LEFT = ("femur_l", "tibia_l", "talus_l", "calcn_l", "toes_l")
PROS = ("transfemur", "osseo_pylon", "tibia_pylon", "foot_l")
PARAMS = ("max_isometric_force", "optimal_fiber_length", "tendon_slack_length",
          "pennation_angle_at_optimal", "max_contraction_velocity",
          "activation_time_constant", "deactivation_time_constant")


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def tokens(text: str | None):
    values = (text or "").split()
    try:
        numbers = [float(value) for value in values]
    except ValueError:
        return values
    if not all(math.isfinite(value) for value in numbers):
        raise ValueError("Nonfinite numeric XML value")
    return numbers


def flat(element: ET.Element, path="", *, physical=False, omit_markers=False):
    """Compare named XML fields, preserving all physical numeric values.

    Physical comparisons omit Appearance subtrees and the GeometryPath
    component's own name only. Native marker calibration serializes that name
    from geometrypath to path; path-point names/order and every recorded
    physical property remain separately captured. XML property order and
    empty-container spelling are not dynamics parameters.
    """
    if physical and element.tag == "Appearance":
        return {}
    if omit_markers and element.tag == "MarkerSet":
        return {}
    result = {}
    value = tokens(element.text)
    if value:
        result[path + "/#text"] = value
    for key, value in element.attrib.items():
        if physical and element.tag == "GeometryPath" and key == "name":
            continue
        result[path + "/@" + key] = value
    counts = Counter()
    for child in element:
        if physical and child.tag == "Appearance":
            continue
        if omit_markers and child.tag == "MarkerSet":
            continue
        name = child.get("name")
        label = child.tag
        if name and not (physical and child.tag == "GeometryPath"):
            label += "[@name=" + name + "]"
        counts[label] += 1
        if counts[label] > 1:
            label += "[occurrence=" + str(counts[label]) + "]"
        result.update(flat(child, path + "/" + label,
                           physical=physical, omit_markers=omit_markers))
    return result


def differences(a, b):
    return [{"field": key, "before": a.get(key), "after": b.get(key)}
            for key in sorted(a.keys() | b.keys()) if a.get(key) != b.get(key)]


def objects(model, set_name):
    elements = list(model.find(set_name + "/objects"))
    result = {element.get("name"): element for element in elements}
    if len(result) != len(elements):
        raise ValueError("Duplicate or absent object names in " + set_name)
    return result


def vector(element, name):
    return [float(value) for value in element.findtext(name, "").split()]


def number(element, name):
    return float(element.findtext(name))


def fingerprint(root, path):
    return {"path": path.relative_to(root).as_posix(), "sha256": digest(path),
            "bytes": path.stat().st_size}


def compare_sets(first, second, set_name, physical=False):
    a, b = objects(first, set_name), objects(second, set_name)
    changes = {name: differences(flat(a[name], physical=physical),
                                 flat(b[name], physical=physical))
               for name in sorted(a.keys() & b.keys())}
    return {"preserved": [name for name, change in changes.items() if not change],
            "changed": {name: change for name, change in changes.items() if change},
            "removed": sorted(a.keys() - b.keys()),
            "added": sorted(b.keys() - a.keys())}


def coordinates(model):
    return {element.get("name"): flat(element)
            for element in model.findall(".//Coordinate")}


def joint_frames(joint):
    return {frame.get("name"): {
        "socket_parent": frame.findtext("socket_parent"),
        "translation_m": vector(frame, "translation"),
        "orientation_rad": vector(frame, "orientation")}
        for frame in joint.findall("frames/PhysicalOffsetFrame")}


def muscle_record(muscle):
    path = muscle.find("GeometryPath")
    points = path.find("PathPointSet/objects")
    return {"class": muscle.tag,
            "parameters": {field: tokens(muscle.findtext(field)) for field in PARAMS},
            "path_point_order": [(point.tag, point.get("name")) for point in points],
            "physical_fields": flat(muscle, physical=True)}


def numeric_summary(values):
    ordered = sorted(values)
    count = len(ordered)
    if not count:
        return {"count": 0}
    return {"count": count, "min": ordered[0], "max": ordered[-1],
            "mean": sum(ordered) / count,
            "median": (ordered[count // 2] if count % 2 else
                       (ordered[count // 2 - 1] + ordered[count // 2]) / 2)}


def changed_parameter_fields(a, b):
    return [key for key in PARAMS if tokens(a.findtext(key)) != tokens(b.findtext(key))]


def adjusted_hip(healthy):
    hip = deepcopy(objects(healthy, "JointSet")["hip_l"])
    hip.find("socket_child_frame").text = "transfemur_offset"
    for element in hip.iter():
        if element.get("name") == "femur_l_offset":
            element.set("name", "transfemur_offset")
        if element.text:
            element.text = element.text.replace("femur_l_offset", "transfemur_offset")
            element.text = element.text.replace("/bodyset/femur_l", "/bodyset/transfemur")
    return hip


def source_function(root, rel, function):
    path = root / rel
    tree = ast.parse(path.read_text(encoding="utf-8"))
    node = next(node for node in ast.walk(tree)
                if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
                and node.name == function)
    return {"path": rel, "sha256": digest(path), "function": function,
            "line": node.lineno, "end_line": node.end_lineno}


def run(root: Path, output: Path):
    stage = "validation/ab07_detector_investigation_2026_10_06/stage3"
    files = {
        "healthy_AB06": "models/AB06-raw/osimxml/AB06.osim",
        "healthy_AB07": "models/AB07-raw/osimxml/ab07.osim",
        "final_AB06_and_donor": "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim",
        "final_AB07": "models/AB07_SEASEA_Threadmill/AB07_SEASEA_stiff321_500_pi.osim",
        "AB06_raw_graft": "models/AB06_SEASEA-raw/osimxml/AB06_SEASEA.osim",
        "AB06_marker_calibrated": "models/AB06_SEASEA-raw/osimxml/AB06_SEASEA_marker_calibrated.osim",
        "AB07_anatomical": stage + "/build/models/AB07_SEASEA-raw/osimxml/AB07_SEASEA_anatomical.osim",
        "AB07_ground_registered": stage + "/build/models/AB07_SEASEA-raw/osimxml/AB07_SEASEA.osim",
        "AB07_marker_calibrated": stage + "/build/models/AB07_SEASEA-raw/osimxml/AB07_SEASEA_marker_calibrated.osim",
        "AB07_archived_pre_registration": stage + "/baseline/models/AB07_SEASEA_Threadmill/AB07_SEASEA_stiff321_500_pi.osim",
        "AB07_pipeline_report": "models/AB07_SEASEA_Threadmill/pipeline_report.json",
        "AB07_ground_registration_manifest": "models/AB07_SEASEA_Threadmill/data/fitting/ground_registration.json",
        "AB07_preparation_report": "reports/user/2026-10-06_preparazione_ab07_seasea_pipeline_e_qualifica_v26.md",
        "AB07_registration_report": "reports/user/2026-10-06_correzione_fitting_ab07_registrazione_piede_terreno.md",
        "builder_source": "tools/build_abx_seasea_pipeline.py",
        "mount_fitting_source": "tools/fit_seasea_subject.py",
        "ground_registration_source": "tools/register_seasea_ground.py",
        "marker_calibration_source": "tools/calibrate_ab06_seasea_markers.py",
        "paired_AB06_numerics": "Trajectory Generator/validation/paired_prescribed_AB06_AB07_2026_10_07/AB06/numerical_verification.json",
        "paired_AB07_numerics": "Trajectory Generator/validation/paired_prescribed_AB06_AB07_2026_10_07/AB07/numerical_verification.json",
    }
    paths = {role: root / rel for role, rel in files.items()}
    before = {role: fingerprint(root, path) for role, path in paths.items()}
    models = {role: ET.parse(path).find("Model")
              for role, path in paths.items() if path.suffix == ".osim"}
    h6, h7, s6, s7 = (models[role] for role in
        ("healthy_AB06", "healthy_AB07", "final_AB06_and_donor", "final_AB07"))
    source = {
        function: source_function(root, "tools/build_abx_seasea_pipeline.py", function)
        for function in ("scale_body_mass", "mass_preserving_targets", "graft_bodyset",
                         "subject_specific_hip_joint", "graft_jointset", "graft_forceset",
                         "graft_markers", "build_bundle")}
    source["fit_seasea_subject"] = source_function(root, "tools/fit_seasea_subject.py", "fit_seasea_subject")
    source["compute_registration"] = source_function(root, "tools/register_seasea_ground.py", "compute_registration")
    source["calibrate_markers"] = source_function(root, "tools/calibrate_ab06_seasea_markers.py", "calibrate_markers")

    body_data = {}
    for role, model in models.items():
        body_data[role] = {name: {"mass_kg": number(body, "mass"),
            "mass_center_m": vector(body, "mass_center"),
            "inertia_kg_m2_Ixx_Iyy_Izz_Ixy_Ixz_Iyz": vector(body, "inertia"),
            "attached_geometry": flat(body.find("attached_geometry"))}
            for name, body in objects(model, "BodySet").items()}
    subjects = {}
    donor_bodies = objects(s6, "BodySet")
    donor_forces = objects(s6, "ForceSet")
    donor_remaining = sum(number(donor_bodies[name], "mass") for name in PROS[1:])
    for subject, healthy, final in (("AB06", h6, s6), ("AB07", h7, s7)):
        hb, fb = objects(healthy, "BodySet"), objects(final, "BodySet")
        hf, ff = objects(healthy, "ForceSet"), objects(final, "ForceSet")
        removed_total = sum(number(hb[name], "mass") for name in RAW_LEFT)
        remaining = removed_total - number(hb["femur_l"], "mass")
        expected_masses = {"transfemur": number(hb["femur_l"], "mass"),
            **{name: remaining * number(donor_bodies[name], "mass") / donor_remaining
               for name in PROS[1:]}}
        prosthesis = {}
        for name in PROS:
            body, donor = fb[name], donor_bodies[name]
            ratio = expected_masses[name] / number(donor, "mass")
            expected_inertia = [value * ratio for value in vector(donor, "inertia")]
            expected = deepcopy(donor)
            expected.find("mass").text = repr(expected_masses[name])
            expected.find("inertia").text = " ".join(map(repr, expected_inertia))
            protected_actual, protected_donor = deepcopy(body), deepcopy(donor)
            for element in (protected_actual, protected_donor):
                element.find("mass").text = "0"
                element.find("inertia").text = "0 0 0 0 0 0"
            prosthesis[name] = {"mass_kg": number(body, "mass"),
                "expected_mass_kg": expected_masses[name], "ratio_to_AB06_donor_mass": ratio,
                "mass_matches_builder": math.isclose(number(body, "mass"), expected_masses[name], rel_tol=1e-12, abs_tol=1e-14),
                "inertia_matches_donor_mass_scaling": all(math.isclose(a, b, rel_tol=1e-12, abs_tol=1e-14)
                    for a, b in zip(vector(body, "inertia"), expected_inertia)),
                "COM_meshes_and_other_body_fields_exactly_donor": flat(protected_actual) == flat(protected_donor),
                "COM_m": vector(body, "mass_center"),
                "inertia_kg_m2": vector(body, "inertia")}
        muscle_origin = {}
        for name, force in ff.items():
            if not force.tag.endswith("Muscle"):
                continue
            actual = muscle_record(force)
            own = muscle_record(hf[name])
            donor = muscle_record(donor_forces[name])
            own_match = actual == own
            donor_match = actual == donor
            muscle_origin[name] = {
                "matches_own_healthy_all_recorded_physical_fields_and_path_order": own_match,
                "matches_AB06_donor_all_recorded_physical_fields_and_path_order": donor_match,
                "origin_class": "own_healthy_preserved" if own_match else "AB06_donor_imported" if donor_match else "unexplained",
                "changed_parameters_from_own_healthy": changed_parameter_fields(hf[name], force),
                "healthy_parameters": own["parameters"], "final_parameters": actual["parameters"],
                "healthy_to_final_physical_differences": differences(own["physical_fields"], actual["physical_fields"])}
        donor_names = [name for name, record in muscle_origin.items()
                       if record["origin_class"] == "AB06_donor_imported"]
        donor_mismatch = {}
        for field in ("optimal_fiber_length", "tendon_slack_length"):
            rows = [{"muscle": name, "healthy_m": number(hf[name], field),
                     "final_donor_m": number(ff[name], field),
                     "difference_percent": 100 * (number(ff[name], field) / number(hf[name], field) - 1)}
                    for name in donor_names]
            donor_mismatch[field] = {"per_muscle": rows,
                "signed_difference_percent": numeric_summary([row["difference_percent"] for row in rows]),
                "absolute_difference_percent": numeric_summary([abs(row["difference_percent"]) for row in rows])}
        path_displacements = []
        for name in donor_names:
            for change in muscle_origin[name]["healthy_to_final_physical_differences"]:
                if change["field"].endswith("/location/#text") and change["before"] is not None and change["after"] is not None:
                    if len(change["before"]) == len(change["after"]) == 3:
                        distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(change["before"], change["after"])))
                        path_displacements.append({"muscle": name, "field": change["field"],
                            "distance_m": distance, "healthy_location_m": change["before"],
                            "final_donor_location_m": change["after"]})
        healthy_joints, final_joints = objects(healthy, "JointSet"), objects(final, "JointSet")
        hardware_checks = {}
        for name in ("osseo", "pros_knee", "pros_ankle"):
            actual, donor = deepcopy(final_joints[name]), deepcopy(objects(s6, "JointSet")[name])
            if name in ("osseo", "pros_ankle"):
                for joint in (actual, donor):
                    parent = joint.findtext("socket_parent_frame")
                    frame = next(frame for frame in joint.findall("frames/PhysicalOffsetFrame") if frame.get("name") == parent)
                    text = frame.find("translation")
                    values = text.text.split()
                    values[1] = "0"
                    text.text = " ".join(values)
            hardware_checks[name] = flat(actual) == flat(donor)
        hc, fc = coordinates(healthy), coordinates(final)
        subjects[subject] = {
            "healthy_mass_kg": sum(number(body, "mass") for body in hb.values()),
            "final_mass_kg": sum(number(body, "mass") for body in fb.values()),
            "removed_left_mass_kg": removed_total,
            "biological_bodies_healthy_to_final": compare_sets(healthy, final, "BodySet"),
            "prosthetic_body_convention": prosthesis,
            "hip_subject_geometry_preserved_after_socket_rename": flat(adjusted_hip(healthy)) == flat(final_joints["hip_l"]),
            "joints_healthy_to_final": compare_sets(healthy, final, "JointSet"),
            "prosthetic_joint_hardware_equal_to_donor_excluding_two_allowed_mount_Y_values": hardware_checks,
            "all_SEA_properties_exact_float_tokens_equal_to_donor": {
                name: flat(force) == flat(donor_forces[name])
                for name, force in ff.items() if force.tag == "SeriesElasticActuator"},
            "markers_healthy_to_final": compare_sets(healthy, final, "MarkerSet"),
            "common_coordinates_identical": {name: hc[name] == fc[name] for name in sorted(hc.keys() & fc.keys())},
            "removed_coordinates": sorted(hc.keys() - fc.keys()),
            "new_coordinates": sorted(fc.keys() - hc.keys()),
            "muscle_count": len(muscle_origin),
            "muscle_origin_counts": dict(Counter(record["origin_class"] for record in muscle_origin.values())),
            "muscle_origins": muscle_origin,
            "donor_characteristic_length_mismatch": donor_mismatch,
            "donor_path_point_local_location_mismatch": {
                "note": "Local frame component distance only; not a ground-frame distance, muscle path length or moment arm. Femur/transfemur rebinding and changed joint frames must also be considered.",
                "per_changed_static_location": path_displacements,
                "distance_m_summary": numeric_summary([row["distance_m"] for row in path_displacements])},
            "removed_healthy_muscles": sorted(set(hf) - set(ff)),
            "all_final_joint_frames": {name: joint_frames(joint) for name, joint in final_joints.items()},
        }

    def stage_comparison(a, b):
        first, second = models[a], models[b]
        all_fields = differences(flat(first, physical=True, omit_markers=True), flat(second, physical=True, omit_markers=True))
        group_changes = [change for change in all_fields if "/groups/" in change["field"]]
        dynamic_fields = [change for change in all_fields if "/groups/" not in change["field"]]
        final_force_names = set(objects(second, "ForceSet"))
        return {"before": a, "after": b,
            "all_nonmarker_recorded_field_changes": all_fields,
            "physical_model_object_fields_excluding_markers_and_set_groups": dynamic_fields,
            "set_group_metadata_changes": group_changes,
            "group_cleanup_only_removes_names_absent_from_final_forces": all(
                [name for name in (change["before"] or []) if name in final_force_names] == (change["after"] or [])
                for change in group_changes),
            "marker_changes": compare_sets(first, second, "MarkerSet"),
            "muscle_parameter_and_path_changes": compare_sets(first, second, "ForceSet", physical=True)}
    stages = {
        "AB07_registration": stage_comparison("AB07_anatomical", "AB07_ground_registered"),
        "AB07_marker_calibration": stage_comparison("AB07_ground_registered", "AB07_marker_calibrated"),
        "AB07_bundle_copy": stage_comparison("AB07_marker_calibrated", "final_AB07"),
        "AB07_archived_to_final": stage_comparison("AB07_archived_pre_registration", "final_AB07"),
        "AB06_raw_to_marker_calibrated": stage_comparison("AB06_raw_graft", "AB06_marker_calibrated"),
        "AB06_marker_calibrated_to_final": stage_comparison("AB06_marker_calibrated", "final_AB06_and_donor"),
    }
    healthy_forces6, healthy_forces7 = objects(h6, "ForceSet"), objects(h7, "ForceSet")
    intrinsic = {
        "meaning": "Differences already present in supplied healthy source models; not verified patient physiology or a causal attribution.",
        "body_differences": compare_sets(h6, h7, "BodySet"),
        "joint_differences": compare_sets(h6, h7, "JointSet"),
        "muscle_differences": compare_sets(h6, h7, "ForceSet", physical=True),
        "muscle_parameter_change_counts": {field: sum(tokens(healthy_forces6[name].findtext(field)) != tokens(healthy_forces7[name].findtext(field)) for name in healthy_forces6) for field in PARAMS},
        "mass_ratio_AB07_over_AB06": subjects["AB07"]["healthy_mass_kg"] / subjects["AB06"]["healthy_mass_kg"],
    }
    pipeline = json.loads(paths["AB07_pipeline_report"].read_text())
    paired_binding = {}
    for subject, role in (("AB06", "final_AB06_and_donor"), ("AB07", "final_AB07")):
        saved = json.loads(paths["paired_" + subject + "_numerics"].read_text())["source_fingerprints"]["model"]
        paired_binding[subject] = {"captured": saved,
            "current_model_matches_paired_receipt_hash": before[role]["sha256"] == saved["expected_sha256"]}
    after = {role: fingerprint(root, path) for role, path in paths.items()}
    checks = {
        "all_read_inputs_unchanged": before == after,
        "both_models_match_paired_receipts": all(item["current_model_matches_paired_receipt_hash"] for item in paired_binding.values()),
        "all_seven_retained_bodies_exactly_own_healthy": all(len(record["biological_bodies_healthy_to_final"]["preserved"]) == 7 and not record["biological_bodies_healthy_to_final"]["changed"] for record in subjects.values()),
        "both_hips_preserve_subject_geometry": all(record["hip_subject_geometry_preserved_after_socket_rename"] for record in subjects.values()),
        "all_common_coordinate_definitions_exactly_own_healthy": all(all(record["common_coordinates_identical"].values()) for record in subjects.values()),
        "all_prosthetic_joint_hardware_fields_preserved_except_allowed_mount_Y_values": all(all(record["prosthetic_joint_hardware_equal_to_donor_excluding_two_allowed_mount_Y_values"].values()) for record in subjects.values()),
        "all_SEA_properties_equal_to_donor": all(all(record["all_SEA_properties_exact_float_tokens_equal_to_donor"].values()) for record in subjects.values()),
        "49_muscles_preserved_and_27_donor_in_both": all(record["muscle_origin_counts"] == {"AB06_donor_imported": 27, "own_healthy_preserved": 49} for record in subjects.values()),
        "no_unexplained_final_muscle_changes": all("unexplained" not in record["muscle_origin_counts"] for record in subjects.values()),
        "all_prosthetic_body_mass_and_inertia_follow_builder_convention": all(all(x["mass_matches_builder"] and x["inertia_matches_donor_mass_scaling"] and x["COM_meshes_and_other_body_fields_exactly_donor"] for x in record["prosthetic_body_convention"].values()) for record in subjects.values()),
        "AB07_ground_registration_exactly_one_physical_field_changed": len(stages["AB07_registration"]["physical_model_object_fields_excluding_markers_and_set_groups"]) == 1,
        "AB07_marker_calibration_preserves_all_body_joint_force_object_fields": not stages["AB07_marker_calibration"]["physical_model_object_fields_excluding_markers_and_set_groups"],
        "AB07_calibration_group_cleanup_only_removes_missing_muscle_names": stages["AB07_marker_calibration"]["group_cleanup_only_removes_names_absent_from_final_forces"],
        "AB07_final_is_byte_identical_to_calibrated_model": before["AB07_marker_calibrated"]["sha256"] == before["final_AB07"]["sha256"],
    }
    result = {
        "schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
        "instruction_check_token": "CMC_AGENT_OK_2026",
        "script": fingerprint(root, Path(__file__).resolve()),
        "scope": "Saved XML/source analysis only. No model loading, native integrations, IK solvers, policy, detector or tests executed.",
        "comparison_method": {"numeric_text": "Exact float-token equality except mass/inertia reconstruction, where rel1e-12/abs1e-14 covers XML rounding.",
            "physical_normalization": "Omit Appearance subtrees and GeometryPath's own name geometrypath/path. Do not omit physical parameters, path-point names/ordered paths, body frames or spatial-transform functions.",
            "group_metadata": "ForceSet ObjectGroup membership changes are retained separately: native model serialization removes names of muscles absent after grafting. Physical-object preservation is reported independently of group metadata, without discarding either evidence.",
            "limits": "XML properties and path order are observed; generalized mass matrix, muscle moment arms/forces and dynamic ground forces are not recomputed."},
        "inputs_before": before, "inputs_after": after, "source_evidence": source,
        "inventory": {role: {set_name: dict(Counter(element.tag for element in model.find(set_name + "/objects"))) for set_name in ("BodySet", "JointSet", "ForceSet", "MarkerSet")} for role, model in models.items()},
        "paired_model_binding": paired_binding,
        "preservation_contract": [
            "Retain seven healthy biological bodies, their mass/COM/inertia/geometry, seven unaffected joints, and subject hip with only socket/body rename.",
            "Replace five healthy left bodies with four fixed-geometry donor bodies. Preserve healthy total mass; transfemur takes healthy femur mass, remaining removed mass is split by donor proportions, inertias multiply by mass ratio; donor COM/meshes remain fixed.",
            "Remove muscles depending on removed distal anatomy and import 27 proximal-left donor muscles unchanged. Retain 49 muscles whose paths do not depend on removed anatomy. This is a pipeline convention, not preservation of all patient-side hip muscles.",
            "Fit only two longitudinal mounting translations to healthy femur/shank ratios. Ground registration subsequently changes only distal mount Y and ankle angle in static calibration motion, not coordinate default/range/runtime target.",
            "Calibrate selected left marker locations; final bundle copies calibrated model. SEA numeric properties/command semantics remain donor properties."],
        "healthy_source_differences": intrinsic, "body_numeric_records": body_data,
        "subjects": subjects, "preparation_stage_differences": stages,
        "AB07_recorded_fitting": pipeline["prosthesis_fit"],
        "AB07_recorded_registration": pipeline["ground_registration"],
        "checks": checks,
        "scientific_interpretation": [
            "Healthy source differences and introduced prosthetic conventions coexist. Matched controller settings do not make mechanical anatomy, body inertias or reference trajectories identical.",
            "Observed donor grafting, mass redistribution, reference-relative mounting and static registration are expected by source code. Expected-by-code does not establish physical correctness.",
            "No unexpected retained biological physical-field mutation is established by these comparisons. This is a preservation result, not proof that preparation causes no contact error.",
            "The 27 donor proximal-left muscles bypass subject-specific healthy scaling, including pelvic path locations and characteristic lengths. The common convention can interact differently with each subject's pelvis/hip geometry and prescribed motion; interaction is a hypothesis, not a measured causal result.",
            "AB06 has the same 27-muscle donor-versus-healthy mismatch. Per-muscle signed/absolute changes quantify differential mismatch; donor import itself is not an AB07-only accident or a violation of muscle actuation being muscle-driven.",
            "A static donor-relative STL ground convention does not identify the subject's physical floor, shoe compliance or online sphere contact law.",
            "No percentage of the AB06/AB07 impulse or timing difference is assigned to anatomy versus preparation; the saved matched experiments change these together."],
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--output", type=Path, default=Path(__file__).with_name("model_preservation_audit.json"))
    args = parser.parse_args()
    result = run(args.repo_root.resolve(), args.output.resolve())
    print(json.dumps({"output": str(args.output), "checks": result["checks"],
        "muscle_origin_counts": {key: record["muscle_origin_counts"] for key, record in result["subjects"].items()},
        "healthy_masses_kg": {key: record["healthy_mass_kg"] for key, record in result["subjects"].items()},
        "healthy_muscle_parameter_change_counts": result["healthy_source_differences"]["muscle_parameter_change_counts"]}, indent=2))


if __name__ == "__main__":
    main()
