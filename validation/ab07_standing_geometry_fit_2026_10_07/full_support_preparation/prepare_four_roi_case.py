"""Freeze/build the authorized FOURROI case using the existing preparation.

The original build function is delegated unchanged. Only its protocol path,
helper provenance path and explicitly enlarged geometry whitelist are redirected.
No automatic ground registration, profile fitting or forward simulation occurs.
"""
from __future__ import annotations

import argparse
import copy
from datetime import datetime, timezone
import importlib.util
from pathlib import Path
import sys
import types
import xml.etree.ElementTree as ET

import numpy as np

PREPARATION = Path(__file__).resolve().parent
AUDIT = PREPARATION.parent
ROOT = AUDIT.parents[1]
PROTOCOL = PREPARATION / "preparation_protocol.json"
CASE = "STANDING_FOURROI_AB07"
BASE_HELPER = AUDIT / "preparation/prepare_standing_cases.py"
spec = importlib.util.spec_from_file_location("four_roi_preparation_base", BASE_HELPER)
base = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = base
spec.loader.exec_module(base)
read, source, fp = base.read, base.source, base.fp
recheck, write_new, matches_fp = base.recheck, base.write_new, base.matches_fp
WHITELIST = {"osseo parent translation.Y", "osseo parent orientation.X", "pros_ankle parent translation.Y"}


def geometry_isolation(before_path, after_path):
    before, after = ET.parse(before_path), ET.parse(after_path)
    changes = []
    for joint_name, property_name, component in (("osseo", "translation", 1), ("osseo", "orientation", 0),
                                                 ("pros_ankle", "translation", 1)):
        entries = []
        for tree in (before, after):
            joints = [j for j in tree.findall(".//JointSet/objects/*") if j.get("name") == joint_name]
            if len(joints) != 1:
                raise AssertionError("Expected one named mounting joint")
            joint = joints[0]
            frame = joint.find("frames/PhysicalOffsetFrame[@name='" + joint.findtext("socket_parent_frame") + "']")
            field = frame.find(property_name)
            values = [float(value) for value in field.text.split()]
            if len(values) != 3 or not np.isfinite(values).all():
                raise AssertionError("Mount field must be a finite Vec3")
            entries.append((field, values))
        label = joint_name + " parent " + property_name + "." + "XYZ"[component]
        changes.append({"field": label, "units": "m" if property_name == "translation" else "rad",
                        "before": entries[0][1][component], "after": entries[1][1][component]})
        for field, values in entries:
            values[component] = 0.0
            field.text = " ".join(format(value, ".17g") for value in values)
    differences = base.xml.differences(base.xml.flat(before.getroot()), base.xml.flat(after.getroot()))
    if differences:
        raise AssertionError("Geometry changed outside the declared three scalar XML fields: " + repr(differences[:5]))
    return {"status": "PASS", "allowed_fields": changes,
            "all_other_XML_numeric_tokens_and_fields_exactly_preserved": True,
            "includes": "MarkerSet, masses/COM/inertias, meshes, internal SEA joint axes/defaults/ranges and all force/plugin properties"}


def freeze():
    if PROTOCOL.exists():
        raise FileExistsError("Preserve the frozen FOURROI preparation protocol")
    fit_path = AUDIT / "geometry/four_roi_support/fit_receipt.json"
    fit = read(fit_path)
    if fit["status"] != "completed" or fit["strict_verdict"] != "PASS" or not all(fit["checks"].values()):
        raise AssertionError("Require the completed strict FOURROI geometry fit")
    for item in (fit["protocol"], fit["output_model"]):
        if not matches_fp(item):
            raise AssertionError("FOURROI model/protocol differs from its receipt")
    geometry = read(source(fit["protocol"]["path"]))
    scope_fp = geometry["sources_before"]["root_scope_protocol"]
    if source(scope_fp["path"]) != AUDIT / "protocol.json" or not matches_fp(scope_fp):
        raise AssertionError("FOURROI protocol is not bound to the frozen root scope")
    if set(geometry["XML_whitelist"]) != WHITELIST or fit["sources_before"] != fit["sources_after"]:
        raise AssertionError("FOURROI whitelist/source preservation differs")
    qualification_path = AUDIT / "geometry/four_roi_support/saved_geometry_qualification.json"
    qualification = read(qualification_path)
    if qualification["status"] != "PASS" or not all(qualification["checks"].values()):
        raise AssertionError("Require the saved strict geometry qualification")
    if not matches_fp(qualification["input_native_receipt"]) or source(qualification["input_native_receipt"]["path"]) != fit_path:
        raise AssertionError("Saved geometry qualification is bound to another native fit")
    previous_path = AUDIT / "preparation/preparation_protocol.json"
    previous = read(previous_path)
    value = copy.deepcopy(previous)
    protected = dict(previous["protected_sources_before"])
    original_dynamic_path = AUDIT / "dynamic_protocol.json"
    original_dynamic = read(original_dynamic_path)
    for name, sha in original_dynamic["protected_sources_before"].items():
        base.add_protected(protected, {"path": name, "sha256": sha})
    roles = dict(previous["source_roles"])
    roles.update(geometry_fit_receipt=fp(fit_path), fitted_geometry_model=fit["output_model"], geometry_fit_protocol=fit["protocol"])
    static = base.checked_motion(source(roles["unshifted_static_motion"]["path"]), source(roles["anatomical_model"]["path"]))
    prepared_names = set(static.labels[1:])
    healthy_names = prepared_names - {"pros_knee_angle", "pros_ankle_angle"}
    healthy_names |= {"knee_angle_l", "ankle_angle_l", "subtalar_angle_l", "mtp_angle_l"}
    refs = fit["reference_coordinate_maps"]
    healthy, prepared = refs["healthy_deg_m"], refs["prepared_deg_m"]
    if set(healthy) != healthy_names or set(prepared) != prepared_names or not np.isfinite(list(healthy.values()) + list(prepared.values())).all():
        raise AssertionError("Require the complete finite healthy23/prepared21 reference maps")
    frozen_refs = geometry["reference"]["coordinate_maps"]
    if healthy != frozen_refs["healthy_deg_m"]:
        raise AssertionError("FOURROI changed the frozen healthy standing reference")
    if any(prepared[name] != frozen_refs["prepared_deg_m"][name] for name in prepared_names - {"pros_ankle_angle"}):
        raise AssertionError("FOURROI changed a biological/knee reference coordinate")
    if any(prepared[name] != healthy[name] for name in prepared_names - {"pros_knee_angle", "pros_ankle_angle"}):
        raise AssertionError("Biological reference poses differ")
    if set(base.xml.coordinates(ET.parse(source(fit["output_model"]["path"])).find("Model"))) != prepared_names:
        raise AssertionError("FOURROI geometry changed the 21-coordinate schema")
    isolation = geometry_isolation(source(roles["anatomical_model"]["path"]), source(fit["output_model"]["path"]))
    offsets = {"pros_knee_angle": prepared["pros_knee_angle"] - healthy["knee_angle_l"],
               "pros_ankle_angle": prepared["pros_ankle_angle"] - healthy["ankle_angle_l"]}
    length = -float(base.original.mount_element(ET.parse(source(fit["output_model"]["path"]))).text.split()[1])
    case = {"id": CASE, "role": "strict four-region standing geometry with its own fresh marker calibration/full IK",
            "constructed_model_source": fit["output_model"], "mount_override_Y_m": None,
            "distal_mount_length_m": length, "distal_mount_length_mm": length * 1000,
            "static_offsets_deg": offsets, "expected_byte_reference": None,
            "case_dir": (PREPARATION / "cases" / CASE).relative_to(ROOT).as_posix()}
    control_path = AUDIT / "dynamics/PYLON400_REFERENCE/run/receipt.json"
    control = read(control_path)
    if control["status"] != "completed" or not control["completed_requested_duration"] or control["steps"] != 611:
        raise AssertionError("Require finalized fresh400 for immutable reuse; no control rerun")
    # Protect every completed control output without consuming its numerical payload.
    control_files = {fp(path)["path"]: fp(path)["sha256"] for path in sorted(control_path.parent.rglob("*")) if path.is_file()}
    for name, sha in control_files.items():
        base.add_protected(protected, {"path": name, "sha256": sha})
    for manifest in (fit["sources_before"], geometry["sources_before"], qualification["sources_before"]):
        for item in manifest.values():
            if not matches_fp(item):
                raise AssertionError("Prospective geometry source changed")
            base.add_protected(protected, item)
    for item in roles.values():
        base.add_protected(protected, item)
    for path in (Path(__file__), BASE_HELPER, previous_path, original_dynamic_path, fit_path, qualification_path,
                 AUDIT / "geometry/four_roi_support/qualify_saved_geometry.py", control_path):
        base.add_protected(protected, fp(path))
    recheck(protected)
    value.update(created_utc=datetime.now(timezone.utc).isoformat(),
                 status="frozen_before_FOURROI_marker_calibration_and_full_IK",
                 supplement_authorization="Root authorized STANDING_FOURROI_AB07, one coherent fresh IK/native attempt; old pair remains immutable",
                 geometry_fit_receipt=fp(fit_path), geometry_fit_protocol=fit["protocol"], strict_geometry_qualification=fp(qualification_path),
                 scope_binding="FOURROI receipt.protocol -> FOURROI protocol.sources_before.root_scope_protocol -> original root scope; original root two-field whitelist explicitly supplemented by FOURROI three-field whitelist",
                 reference_coordinate_maps=refs, geometry_model_isolation=isolation, source_roles=roles,
                 cases={CASE: case}, execution_order=[CASE], original_preparation_protocol=fp(previous_path),
                 original_dynamic_protocol=fp(original_dynamic_path), finalized_fresh400_receipt=fp(control_path),
                 finalized_fresh400_outputs_before=control_files, reference_reproduction_contract={"control": "Reuse finalized fresh400; no new control solve or native run"},
                 protected_count=len(protected), protected_sources_before=protected)
    value["prospective_dynamic_plan"].update(cases="single FOURROI candidate; reuse finalized fresh400", owner="preparation agent delegated original runtime")
    value["limitations"] += ["Four-region vertex contact and a noncollinear support polygon do not establish distributed load-bearing contact or physiological validity.",
                             "The extra osseo mounting roll is explicit; internal prosthetic axes and all other XML properties remain unchanged."]
    write_new(PROTOCOL, value)
    print({"protocol": fp(PROTOCOL), "case": CASE, "static_offsets_deg": offsets, "protected_count": len(protected)}, flush=True)


def build():
    copied = dict(base.build.__globals__)
    copied.update(PROTOCOL=PROTOCOL, geometry_isolation=geometry_isolation, __file__=str(Path(__file__).resolve()))
    delegated = types.FunctionType(base.build.__code__, copied, base.build.__name__, base.build.__defaults__, base.build.__closure__)
    delegated.__kwdefaults__ = base.build.__kwdefaults__
    if delegated.__code__ is not base.build.__code__:
        raise AssertionError("Original preparation bytecode changed")
    return delegated(CASE)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_mutually_exclusive_group(required=True)
    commands.add_argument("--freeze", action="store_true")
    commands.add_argument("--build", action="store_true")
    args = parser.parse_args()
    freeze() if args.freeze else build()


if __name__ == "__main__":
    main()
