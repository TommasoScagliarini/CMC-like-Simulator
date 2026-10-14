"""Freeze and prepare two coherent AB07 standing-fit/reference cases.

Consumes the geometry team's frozen model and complete reference coordinate
maps. Preserves each raw static biological pose and applies only the two
declared angular reference differences. Uses the original 12-marker calibration
and full-trial marker IK, without automatic ground registration, scaling, RRA,
ID, simulator smoke, dynamic rollout, profile fitting or operational edits.
Byte identity is preferred for the fresh 400 mm control; any textual-only
nonidentity is retained and quantified against exactly equal named payloads.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import traceback
import xml.etree.ElementTree as ET

import numpy as np

PREPARATION = Path(__file__).resolve().parent
AUDIT = PREPARATION.parent
ROOT = AUDIT.parents[1]
OLD = ROOT / "validation/ab07_pylon_necessity_screening_2026_10_07"
PROTOCOL = PREPARATION / "preparation_protocol.json"
sys.path.insert(0, str(ROOT))


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    sys.modules[name] = result
    spec.loader.exec_module(result)
    return result


original = module(
    "standing_original_preparation",
    ROOT / "validation/ab07_mount_registration_verification_2026_10_07/variants/prepare_mount_variants.py",
)
calibration, runner, xml = original.calibration, original.runner, original.xml


def read(path):
    return json.loads(Path(path).read_text(), parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))


def source(raw):
    path = Path(str(raw).replace("\\", "/"))
    return (path if path.is_absolute() else ROOT / path).resolve()


def fp(path):
    path = source(path)
    try:
        name = path.relative_to(ROOT).as_posix()
    except ValueError:
        name = str(path)
    return {"path": name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def matches_fp(item):
    actual = fp(item["path"])
    return (source(actual["path"]) == source(item["path"]) and actual["sha256"] == item["sha256"] and
            ("size_bytes" not in item or source(item["path"]).stat().st_size == item["size_bytes"]))


def recheck(manifest):
    after = {name: fp(name)["sha256"] for name in manifest}
    if after != manifest:
        raise AssertionError("Frozen sources changed: " + repr([name for name in manifest if after[name] != manifest[name]]))
    return after


def write_new(path, value):
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write("\n")


def add_protected(manifest, item):
    if item["path"] in manifest and manifest[item["path"]] != item["sha256"]:
        raise AssertionError("Conflicting protected source: " + item["path"])
    manifest[item["path"]] = item["sha256"]


def checked_motion(path, model_path, expected_range=None):
    motion = calibration.parse_motion(path)
    names = set(xml.coordinates(ET.parse(model_path).find("Model")))
    if len(names) != 21 or len(motion.labels) != 22 or len(set(motion.labels)) != 22:
        raise AssertionError("Require exactly 21 unique coordinates plus time")
    if motion.labels[0] != "time" or set(motion.labels[1:]) != names:
        raise AssertionError("Motion names differ from the complete model coordinate schema")
    if not motion.in_degrees or not np.isfinite(motion.values).all() or len(motion.times) < 2 or np.any(np.diff(motion.times) <= 0):
        raise AssertionError("Motion units, finiteness or monotonic clock invalid")
    if expected_range is not None and [float(motion.times[0]), float(motion.times[-1])] != expected_range:
        raise AssertionError("Motion does not span the frozen full trial")
    return motion


def geometry_isolation(before_path, after_path):
    before, after = ET.parse(before_path), ET.parse(after_path)
    changes = []
    for joint_name in ("osseo", "pros_ankle"):
        fields = []
        for tree in (before, after):
            joints = [joint for joint in tree.findall(".//JointSet/objects/*") if joint.get("name") == joint_name]
            if len(joints) != 1:
                raise AssertionError("Expected one named mounting joint")
            joint = joints[0]
            frame = joint.find("frames/PhysicalOffsetFrame[@name='" + joint.findtext("socket_parent_frame") + "']")
            field = frame.find("translation")
            values = [float(value) for value in field.text.split()]
            if len(values) != 3 or not np.isfinite(values).all():
                raise AssertionError("Mount translation is not a finite Vec3")
            fields.append((field, values))
        changes.append({"joint": joint_name, "property": "parent translation.Y", "before_m": fields[0][1][1], "after_m": fields[1][1][1]})
        for field, values in fields:
            values[1] = 0.0
            field.text = " ".join(format(value, ".17g") for value in values)
    differences = xml.differences(xml.flat(before.getroot()), xml.flat(after.getroot()))
    if differences:
        raise AssertionError("Fit changed XML beyond the two declared mounting Y components: " + repr(differences[:4]))
    return {"status": "PASS", "allowed_fields": changes, "all_other_XML_numeric_tokens_and_fields_exactly_preserved": True,
            "includes": "MarkerSet, masses/COM/inertias, meshes, joint axes/frames/defaults/ranges, forces, SEA/plugin parameters, metadata"}


def compare_reference_model(current, expected):
    differences = xml.differences(xml.flat(ET.parse(expected).getroot()), xml.flat(ET.parse(current).getroot()))
    return {"actual": fp(current), "historical": fp(expected), "byte_equal": Path(current).read_bytes() == Path(expected).read_bytes(),
            "exact_semantic_or_numeric_equal": not differences, "XML_numeric_token_differences": differences,
            "tolerance": "None; numerical XML tokens and every XML field must be equal"}


def compare_reference_motion(current, expected):
    a, b = calibration.parse_motion(expected), calibration.parse_motion(current)
    schema_equal = (a.in_degrees == b.in_degrees and len(a.labels) == len(b.labels) and
                    len(set(a.labels)) == len(a.labels) and len(set(b.labels)) == len(b.labels) and set(a.labels) == set(b.labels))
    row_count_equal = a.values.shape[0] == b.values.shape[0]
    differences = {}
    if schema_equal and row_count_equal:
        for name in a.labels:
            av, bv = a.values[:, a.labels.index(name)], b.values[:, b.labels.index(name)]
            differences[name] = {"exact_equal": bool(np.array_equal(av, bv)), "max_abs_difference": float(np.max(np.abs(av - bv)))}
    numeric_equal = schema_equal and row_count_equal and all(value["exact_equal"] for value in differences.values())
    return {"actual": fp(current), "historical": fp(expected), "byte_equal": Path(current).read_bytes() == Path(expected).read_bytes(),
            "exact_semantic_or_numeric_equal": numeric_equal, "same_named_schema_and_units": schema_equal,
            "same_row_count": row_count_equal, "columns": differences,
            "tolerance": "None; exact time and values compared by coordinate name; only textual metadata/header/order may differ"}


def freeze(fit_receipt_path):
    if PROTOCOL.exists():
        raise FileExistsError("Preserve frozen preparation protocol")
    fit_path = source(fit_receipt_path)
    fit = read(fit_path)
    if fit["status"] != "completed":
        raise AssertionError("Require a completed, source-bound geometry fit")
    if not matches_fp(fit["output_model"]):
        raise AssertionError("Geometry fit model differs from its receipt")
    scope = read(AUDIT / "protocol.json")
    if not matches_fp(fit["scope_protocol"]) or source(fit["scope_protocol"]["path"]) != AUDIT / "protocol.json" or not matches_fp(fit["protocol"]):
        raise AssertionError("Geometry fit is not bound to the frozen root scope and geometry protocol")
    geometry_protocol = read(source(fit["protocol"]["path"]))
    if not matches_fp(geometry_protocol["sources_before"]["root_scope_protocol"]) or source(geometry_protocol["sources_before"]["root_scope_protocol"]["path"]) != AUDIT / "protocol.json":
        raise AssertionError("Geometry protocol is bound to a different root scope")
    previous = read(OLD / "preparation/preparation_protocol.json")
    protected = dict(scope["original_and_previous_audit_sources_before"])
    for name, sha in previous["protected_sources_before"].items():
        add_protected(protected, {"path": name, "sha256": sha})
    roles = dict(previous["source_roles"])
    roles["geometry_fit_receipt"] = fp(fit_path)
    roles["fitted_geometry_model"] = fit["output_model"]
    roles["geometry_fit_protocol"] = fit["protocol"]
    isolation = geometry_isolation(source(roles["anatomical_model"]["path"]), source(fit["output_model"]["path"]))
    static = checked_motion(source(roles["unshifted_static_motion"]["path"]), source(roles["anatomical_model"]["path"]))
    prepared_names = set(static.labels[1:])
    healthy_names = prepared_names - {"pros_knee_angle", "pros_ankle_angle"}
    healthy_names |= {"knee_angle_l", "ankle_angle_l", "subtalar_angle_l", "mtp_angle_l"}
    refs = fit["reference_coordinate_maps"]
    healthy, prepared = refs["healthy_deg_m"], refs["prepared_deg_m"]
    if set(healthy) != healthy_names or set(prepared) != prepared_names:
        raise AssertionError("Geometry receipt requires complete healthy23/prepared21 named reference maps")
    if not np.isfinite(list(healthy.values()) + list(prepared.values())).all():
        raise AssertionError("Reference coordinate maps contain nonfinite values")
    biological = prepared_names - {"pros_knee_angle", "pros_ankle_angle"}
    if any(healthy[name] != prepared[name] for name in biological):
        raise AssertionError("Reference biological pose must be shared by healthy and prepared models")
    if healthy != geometry_protocol["reference_coordinate_maps"]["healthy_deg_m"]:
        raise AssertionError("Healthy reference differs from the pose frozen before geometry fitting")
    prepared_before = geometry_protocol["reference_coordinate_maps"]["prepared_before_fit_deg_m"]
    if any(prepared[name] != prepared_before[name] for name in prepared_names - {"pros_ankle_angle"}):
        raise AssertionError("Geometry fitting changed a biological/knee reference coordinate")
    if set(xml.coordinates(ET.parse(source(fit["output_model"]["path"])).find("Model"))) != prepared_names:
        raise AssertionError("Fitted geometry changed the 21-coordinate schema")
    offsets = {"pros_knee_angle": prepared["pros_knee_angle"] - healthy["knee_angle_l"],
               "pros_ankle_angle": prepared["pros_ankle_angle"] - healthy["ankle_angle_l"]}
    ref_path = OLD / "preparation/cases/P3_A1/receipt.json"
    ref = read(ref_path)
    if ref["status"] != "completed" or ref["case"]["distal_mount_length_m"] != 0.4:
        raise AssertionError("Require the completed exact 400 mm P3_A1 reference")
    for key in ("output_model", "full_IK", "static_motion"):
        if fp(ref[key]["path"]) != ref[key]:
            raise AssertionError("Frozen 400 mm reference artifact changed")
    roles["reference_400_receipt"] = fp(ref_path)
    mount = original.mount_element(ET.parse(source(fit["output_model"]["path"])))
    fitted_length = -float(mount.text.split()[1])
    cases = {
        "STANDING_AB07": {"id": "STANDING_AB07", "role": "new geometry fit with its own fresh marker calibration/full IK",
                         "constructed_model_source": fit["output_model"], "mount_override_Y_m": None,
                         "distal_mount_length_m": fitted_length, "distal_mount_length_mm": fitted_length * 1000,
                         "static_offsets_deg": offsets, "expected_byte_reference": None},
        "PYLON400_REFERENCE": {"id": "PYLON400_REFERENCE", "role": "fresh exact reproduction of the existing P3_A1 control",
                          "constructed_model_source": roles["anatomical_model"], "mount_override_Y_m": -0.4,
                          "distal_mount_length_m": 0.4, "distal_mount_length_mm": 400.0,
                          "static_offsets_deg": {"pros_knee_angle": 0.0, "pros_ankle_angle": ref["case"]["static_ankle_offset_deg"]},
                          "expected_byte_reference": {key: ref[key] for key in ("output_model", "full_IK", "static_motion")}},
    }
    for name, case in cases.items():
        case["case_dir"] = (PREPARATION / "cases" / name).relative_to(ROOT).as_posix()
    for item in roles.values():
        add_protected(protected, item)
    for item in geometry_protocol["sources_before"].values():
        if not matches_fp(item):
            raise AssertionError("Geometry source changed after its prospective freeze")
        add_protected(protected, item)
    for path in (Path(__file__), AUDIT / "protocol.json", OLD / "preparation/preparation_protocol.json", ref_path):
        add_protected(protected, fp(path))
    for item in previous["support_paths"].values():
        add_protected(protected, item)
    recheck(protected)
    value = {"created_utc": datetime.now(timezone.utc).isoformat(), "instruction_check_token": "CMC_AGENT_OK_2026",
             "status": "frozen_before_new_marker_calibration_and_full_IK", "scope_protocol": fp(AUDIT / "protocol.json"),
             "geometry_fit_receipt": fp(fit_path), "reference_coordinate_maps": refs,
             "geometry_model_isolation": isolation,
             "static_mapping": {"source": roles["unshifted_static_motion"], "source_samples": len(static.times),
                                "source_time_range_s": [float(static.times[0]), float(static.times[-1])],
                                "formula": "q_prepared_i(t)=q_healthy_i(t)-q_healthy_reference_i+q_prepared_reference_i for knee/ankle only; all19 biological columns and original clock exactly preserved",
                                "healthy_source_remapping": {"pros_knee_angle": "knee_angle_l", "pros_ankle_angle": "ankle_angle_l"},
                                "dropped_healthy_coordinates": ["subtalar_angle_l", "mtp_angle_l"],
                                "meaning": "A declared reference-coordinate convention, not equality of healthy CustomJoint and prosthetic PinJoint axes or a physical marker/prosthesis correspondence",
                                "automatic_ground_registration": False, "old_registered_angle_inherited_by_candidate": False},
             "cases": cases, "execution_order": ["PYLON400_REFERENCE", "STANDING_AB07"], "source_roles": roles,
             "support_data_dir": previous["support_data_dir"], "support_paths": previous["support_paths"],
             "full_time_range_s": previous["full_time_range_s"], "calibration_markers": previous["calibration_markers"],
             "calibration_time_range": None, "calibration_stride": 1, "backend": previous["backend"],
             "full_IK_contract": {"opensim_cmd": None, "skip_scaling": True, "ik_only": True, "run_smoke": False,
                                  "ID": False, "RRA": False, "full_named_schema": "21 unique model coordinates plus time; degree rotations/metre translations",
                                  "actual_solver_copies": "calibrated model and original fulltrial TRC byte matches required"},
             "reference_reproduction_contract": {"byte_identity_preferred": True,
                                                  "if_textual_nonidentity": "Preserve files and record both hashes; accept only exact XML semantic fields and exact named time/coordinate payload/units, without numerical tolerances or rerun",
                                                  "if_physical_or_numeric_nonidentity": "Preserve failed receipt and outputs; do not normalize, retune or retry"},
             "prospective_dynamic_plan": {"owner": "root", "cases": "both fresh cases; no outcome selection",
                                           "reset_HS_s": 12.591740519774012, "duration_s": 6.11, "steps": 611,
                                           "primary": "same three frozen nominal cycles; runtime and profiles to be frozen separately before native integration"},
             "limitations": ["New marker calibration/full IK is part of the preparation intervention; geometry alone is not isolated by the eventual dynamic comparison.",
                             "Geometry fitting to a declared standing pose does not by itself establish physical prosthesis dimensions or physiological gait validity.",
                             "Virtual prosthetic contact and measured healthy GRF are distinct systems; timing/force differences remain diagnostic."],
             "protected_count": len(protected), "protected_sources_before": protected}
    write_new(PROTOCOL, value)
    print(json.dumps({"protocol": fp(PROTOCOL), "cases": list(cases), "static_offsets_deg": offsets,
                      "protected_count": len(protected)}, allow_nan=False), flush=True)


def build(name):
    protocol = read(PROTOCOL)
    recheck(protocol["protected_sources_before"])
    if sys.executable != protocol["backend"]["python"] or str(calibration.osim.GetVersionAndDate()) != protocol["backend"]["opensim_version"]:
        raise AssertionError("Frozen Python/OpenSim backend mismatch")
    case = protocol["cases"][name]
    out = source(case["case_dir"])
    if out.exists():
        raise FileExistsError("No overwrite or reuse of a preparation attempt")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.mkdir()
    receipt = {"created_utc": datetime.now(timezone.utc).isoformat(), "status": "prepared", "case": case,
               "protocol": fp(PROTOCOL), "helper": fp(Path(__file__)), "backend": protocol["backend"],
               "support_paths": protocol["support_paths"], "geometry_fit_receipt": protocol["geometry_fit_receipt"]}
    write_new(out / "receipt_prepared.json", receipt)
    def role(key):
        return source(protocol["source_roles"][key]["path"])
    try:
        input_model = out / "AB07_SEASEA.osim"
        original_model = source(case["constructed_model_source"]["path"])
        if not matches_fp(case["constructed_model_source"]):
            raise AssertionError("Source geometry changed")
        if case["mount_override_Y_m"] is None:
            receipt["geometry_model_isolation"] = geometry_isolation(role("anatomical_model"), original_model)
            with input_model.open("xb") as stream:
                stream.write(original_model.read_bytes())
        else:
            tree = ET.parse(original_model)
            field = original.mount_element(tree)
            values = field.text.split()
            values[1] = format(case["mount_override_Y_m"], ".17g")
            field.text = " ".join(values)
            tree.write(input_model, encoding="utf-8", xml_declaration=True)
            before, after = ET.parse(original_model), ET.parse(input_model)
            original.mount_element(before).text = "0 0 0"
            original.mount_element(after).text = "0 0 0"
            if xml.flat(before.getroot()) != xml.flat(after.getroot()):
                raise AssertionError("Reference construction changed more than one mount Y")
        raw_static = checked_motion(role("unshifted_static_motion"), input_model)
        values = raw_static.values.copy()
        for coordinate, offset in case["static_offsets_deg"].items():
            values[:, raw_static.labels.index(coordinate)] += offset
        static = out / ("AB07_SEASEA_static_ground_registered.mot" if case["expected_byte_reference"] else "AB07_SEASEA_static_fitted.mot")
        with static.open("x", encoding="utf-8") as stream:
            stream.write(f"{static.stem}\nnRows={len(values)}\nnColumns=22\ninDegrees=yes\nendheader\n" + "\t".join(raw_static.labels) + "\n")
            np.savetxt(stream, values, delimiter="\t", fmt="%.17g")
        saved = checked_motion(static, input_model)
        unchanged = [i for i, label in enumerate(saved.labels) if label not in case["static_offsets_deg"]]
        if not np.array_equal(saved.values[:, unchanged], raw_static.values[:, unchanged]) or not np.array_equal(saved.values, values):
            raise AssertionError("Static mapping changed the clock or an undeclared coordinate")
        calibrated = out / "AB07_SEASEA_marker_calibrated.osim"
        marker_report = out / "AB07_SEASEA_marker_calibrated.marker_calibration.csv"
        calibration.calibrate_markers(input_model, role("static_TRC"), static, calibrated, marker_report, role("plugin"),
                                      list(protocol["calibration_markers"]), None, 1)
        preservation = original.marker_preservation(input_model, calibrated)
        expected = case["expected_byte_reference"]
        reference_comparison = None
        if expected:
            reference_comparison = {"output_model": compare_reference_model(calibrated, source(expected["output_model"]["path"])),
                                    "static_motion": compare_reference_motion(static, source(expected["static_motion"]["path"]))}
            receipt["reference_reproduction"] = reference_comparison
            if any(not value["exact_semantic_or_numeric_equal"] for value in reference_comparison.values()):
                raise AssertionError("Fresh reference model/static has substantive XML or numeric differences")
        cfg = runner.ToolConfig(opensim_cmd=None, plugin=role("plugin"), model=calibrated, marker_set=calibrated,
                                trc=role("full_trial_TRC"), grf=role("full_trial_GRF"), support_data=source(protocol["support_data_dir"]),
                                dry_run=False, skip_scaling=True, ik_only=True, full_time_range=tuple(protocol["full_time_range_s"]))
        paths = runner.create_paths(out / "pipeline")
        if runner.run_pipeline(cfg, paths, run_smoke=False) != 0:
            raise RuntimeError("Original full-trial marker IK failed")
        ik = paths.ik_dir / ("IK_full_" + runner.range_label(cfg.full_time_range) + ".mot")
        full = checked_motion(ik, calibrated, protocol["full_time_range_s"])
        if expected:
            reference_comparison["full_IK"] = compare_reference_motion(ik, source(expected["full_IK"]["path"]))
            if not reference_comparison["full_IK"]["exact_semantic_or_numeric_equal"]:
                raise AssertionError("Fresh reference full IK has numeric differences from frozen P3_A1")
        setup = ik.with_name(ik.stem + "_setup.xml")
        setup_xml = ET.parse(setup)
        actual_model = source(setup_xml.findtext(".//model_file"))
        actual_markers = source(setup_xml.findtext(".//marker_file"))
        if actual_model.read_bytes() != calibrated.read_bytes() or actual_markers.read_bytes() != role("full_trial_TRC").read_bytes():
            raise AssertionError("Actual solver model/TRC copies differ from the frozen inputs")
        recheck(protocol["protected_sources_before"])
        receipt.update(status="completed", completed_utc=datetime.now(timezone.utc).isoformat(),
                       constructed_model=fp(input_model), output_model=fp(calibrated), static_motion=fp(static),
                       full_IK=fp(ik), full_IK_time_range_s=[float(full.times[0]), float(full.times[-1])], full_IK_samples=len(full.times),
                       actual_solver_setup=fp(setup), actual_solver_model_copy=fp(actual_model), actual_solver_marker_copy=fp(actual_markers),
                       actual_solver_log=fp(paths.ik_dir / "ik_full.log"), marker_calibration_report=fp(marker_report),
                       marker_calibration_preservation=preservation, raw_biological_static_and_clock_preserved=True,
                       backend_contract=protocol["full_IK_contract"], protected_inputs_unchanged=True,
                       protected_input_count=protocol["protected_count"],
                       reference_reproduction=reference_comparison,
                       reference_byte_identity={"preferred": expected is not None,
                                                "model": reference_comparison["output_model"]["byte_equal"] if expected else None,
                                                "static_motion": reference_comparison["static_motion"]["byte_equal"] if expected else None,
                                                "full_IK": reference_comparison["full_IK"]["byte_equal"] if expected else None},
                       model_bundle_dir=out.relative_to(ROOT).as_posix())
        write_new(out / "receipt.json", receipt)
        print(json.dumps({"case": name, "status": "completed", "receipt": fp(out / "receipt.json"),
                          "output_model": receipt["output_model"], "full_IK": receipt["full_IK"]}, allow_nan=False), flush=True)
    except Exception as exc:
        receipt.update(status="failed", error_type=type(exc).__name__, error=str(exc), traceback=traceback.format_exc(),
                       protected_inputs_unchanged=(recheck(protocol["protected_sources_before"]) == protocol["protected_sources_before"]))
        write_new(out / "receipt.json", receipt)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_mutually_exclusive_group(required=True)
    commands.add_argument("--freeze", action="store_true")
    commands.add_argument("--case", choices=("PYLON400_REFERENCE", "STANDING_AB07"))
    parser.add_argument("--fit-receipt", type=Path)
    args = parser.parse_args()
    if args.freeze:
        if args.fit_receipt is None:
            parser.error("--freeze requires --fit-receipt")
        freeze(args.fit_receipt)
    else:
        if args.fit_receipt is not None:
            parser.error("--case reads only the frozen preparation protocol")
        build(args.case)


if __name__ == "__main__":
    main()
