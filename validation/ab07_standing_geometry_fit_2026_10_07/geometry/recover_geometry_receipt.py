"""Complete a preserved partial JSON receipt without rerunning geometry/FK.

V2 serialized its completed reference/fit/native evidence, then json.dump
stopped on the first NumPy bool in checks. Parse only those complete fields;
verify saved native evidence algebraically, XML whitelist and source hashes;
write a separate receipt with built-in booleans and disclosed recovery timing.
"""
from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import struct
import sys
from xml.etree import ElementTree as ET

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT))
INITIAL = HERE / "fit_geometry_receipt_v2.json"
PROTOCOL = HERE / "geometry_protocol_v2.json"
OUTPUT = HERE / "fit_geometry_receipt_completed.json"
EXPECTED_PARTIAL_SHA = "b6424527fe614f26310c896e3ca7eb72fb85b942f13da2b29ee0bb079e7cc90e"
EXPECTED_PROTOCOL_SHA = "05195374231976098fc3a7066fcfef812ae508aa56aa31d31b8eded373870e13"
EXPECTED_CANONICAL_SHA = "d942f045f7eae11ccab0e9a1f4ba2a1fed090bc6c114145431079a51d0fa86e8"


def fp(path):
    p = Path(path).resolve()
    return {"path": str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p),
            "sha256": hashlib.sha256(p.read_bytes()).hexdigest(), "size_bytes": p.stat().st_size}


def resolve(raw):
    path = Path(raw)
    return path if path.is_absolute() else ROOT / path


def mount(tree, name):
    joint = tree.find(f".//JointSet/objects/*[@name='{name}']")
    frame = joint.findtext("socket_parent_frame")
    return joint.find(f"frames/PhysicalOffsetFrame[@name='{frame}']/translation")


def masked(tree, canonical):
    result = deepcopy(tree)
    for name in ("osseo", "pros_ankle"):
        value = mount(result, name); tokens = value.text.split(); tokens[1] = "0"; value.text = " ".join(tokens)
    return canonical(result.getroot())


def main():
    if OUTPUT.exists():
        raise FileExistsError("Existing completed receipt is immutable")
    started = datetime.now(timezone.utc).isoformat()
    initial_before, protocol_before, helper_before = fp(INITIAL), fp(PROTOCOL), fp(Path(__file__))
    if initial_before["sha256"] != EXPECTED_PARTIAL_SHA or protocol_before["sha256"] != EXPECTED_PROTOCOL_SHA:
        raise ValueError("Partial/native evidence or frozen protocol changed")
    protocol = json.loads(PROTOCOL.read_text())
    before = {name: fp(resolve(v["path"])) for name, v in protocol["sources_before"].items()}
    if before != protocol["sources_before"]:
        raise ValueError("Frozen geometry source changed before recovery")
    canonical_path = ROOT / "tools/fit_seasea_subject.py"
    canonical_before = fp(canonical_path)
    scope = json.loads(resolve(before["root_scope_protocol"]["path"]).read_text())
    if canonical_before["sha256"] != EXPECTED_CANONICAL_SHA or scope["original_and_previous_audit_sources_before"]["tools/fit_seasea_subject.py"] != EXPECTED_CANONICAL_SHA:
        raise ValueError("Pure canonical comparator is not current/root-source bound")
    from tools.fit_seasea_subject import _canonical
    text = INITIAL.read_text()
    marker = '  "checks":'
    if text.count(marker) != 1 or text.split(marker)[1].strip() != '{\n    "knee_height_target":':
        raise ValueError("Unexpected JSON truncation point; no guessed recovery allowed")
    prefix = text.split(marker)[0].rstrip().rstrip(",")
    receipt = json.loads(prefix + "\n}")
    if receipt["status"] != "completed" or receipt["protocol"] != protocol_before:
        raise ValueError("Preserved native computation did not declare completed")
    if receipt["reference_coordinate_maps"]["healthy_deg_m"] != protocol["reference_coordinate_maps"]["healthy_deg_m"]:
        raise ValueError("Healthy reference no longer matches frozen raw median")
    healthy = receipt["reference_coordinate_maps"]["healthy_deg_m"]
    prepared = receipt["reference_coordinate_maps"]["prepared_deg_m"]
    initial_prepared = protocol["reference_coordinate_maps"]["prepared_before_fit_deg_m"]
    if set(prepared) != set(initial_prepared) or any(prepared[n] != initial_prepared[n] for n in prepared if n != "pros_ankle_angle"):
        raise ValueError("Native fit changed a reference coordinate outside declared static ankle")
    model = resolve(receipt["output_model"]["path"])
    model_before = fp(model)
    if model_before != receipt["output_model"]:
        raise ValueError("Saved native-verified fitted model changed")
    original_tree = ET.parse(resolve(before["anatomical_model_before_donor_ground_registration"]["path"]))
    fitted_tree = ET.parse(model)
    xml_whitelist = masked(original_tree, _canonical) == masked(fitted_tree, _canonical)
    exact_mounts = True
    for joint, stem in (("osseo", "osseo"), ("pros_ankle", "pros_ankle")):
        actual = list(map(float, mount(fitted_tree, joint).text.split()))
        exact_mounts &= actual == receipt["mounts"][f"{stem}_parent_translation_after_m"]
        original = list(map(float, mount(original_tree, joint).text.split()))
        exact_mounts &= original == receipt["mounts"][f"{stem}_parent_translation_before_m"]
    raw = resolve(before["fixed_foot_STL"]["path"]).read_bytes()
    n = struct.unpack_from("<I", raw, 80)[0]
    if len(raw) != 84 + 50 * n or n == 0:
        raise ValueError("Unexpected fixed binary STL layout")
    dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    vertices = np.unique(np.ndarray((n,), dtype=dtype, buffer=raw, offset=84)["vertices"].reshape(-1, 3).astype(float), axis=0)
    geometry, native = receipt["geometry"], receipt["native_parity"]
    r = np.array(geometry["foot_rotation_ground"]); p = np.array(geometry["fitted_ankle_XYZ_m"])
    world = vertices @ r.T + p
    span = float(np.ptp(vertices[:, 0]))
    heel = vertices[:, 0] <= float(vertices[:, 0].min()) + protocol["support_regions"]["heel_fraction"] * span
    fore = vertices[:, 0] >= float(vertices[:, 0].max()) - protocol["support_regions"]["forefoot_fraction"] * span
    measured = {"whole_STL_min_Y_m": float(world[:, 1].min()), "heel_min_Y_m": float(world[heel, 1].min()),
                "forefoot_min_Y_m": float(world[fore, 1].min()),
                "support_vertices_at_declared_tolerance": int(np.count_nonzero(np.abs(world[:, 1]) <= protocol["tolerances"]["geometry_m"]))}
    saved_geometry_matches = all(measured[k] == geometry[k] for k in measured)
    tol = protocol["tolerances"]
    knee_y_residual = float(geometry["fitted_knee_XYZ_m"][1] - geometry["healthy_knee_target_XYZ_m"][1])
    checks = {
        "knee_height_target": bool(abs(knee_y_residual) <= tol["geometry_m"]),
        "heel_and_forefoot_grounded": bool(max(abs(measured["heel_min_Y_m"]), abs(measured["forefoot_min_Y_m"])) <= tol["geometry_m"]),
        "whole_STL_nonpenetration": bool(measured["whole_STL_min_Y_m"] >= -tol["geometry_m"]),
        "native_position_parity": bool(max(native["knee_XYZ_max_error_m"], native["foot_origin_XYZ_max_error_m"]) <= tol["native_parity_m"] and native["foot_rotation_max_error_dimensionless"] <= tol["rotation_dimensionless"]),
        "protected_XML_exact": bool(xml_whitelist and exact_mounts),
        "saved_native_geometry_matches_pure_recomputation": bool(saved_geometry_matches),
        "reference_maps_and_static_knee_preserved": True,
    }
    primary = json.loads(resolve(before["fixed_primary_contact_profile"]["path"]).read_text())
    binary = json.loads(resolve(before["fixed_binary_detector_profile"]["path"]).read_text())
    after = {name: fp(resolve(v["path"])) for name, v in protocol["sources_before"].items()}
    checks["frozen_inputs_unchanged"] = bool(before == after == protocol["sources_before"] and fp(PROTOCOL) == protocol_before
                                            and fp(INITIAL) == initial_before and fp(model) == model_before
                                            and fp(canonical_path) == canonical_before and fp(Path(__file__)) == helper_before)
    receipt.update(
        checks=checks, status="completed" if all(checks.values()) else "failed",
        profile_planes_unchanged={"fit_benchmark": protocol["ground"], "primary_force_ground": primary["ground"], "binary_ground": binary["ground"]},
        geometry_allowed_XML_changes=protocol["fit"]["free_XML_scalars"],
        sources_before=before, sources_after=after, new_integrations=0, new_IK_solves=0,
        donor_ground_registration_called=False, new_Claude_directory_read=False,
        equations={"knee": "Knew=Kold+Rupper*ey*(osseoYnew-osseoYold); solve KnewY=healthyKneeY",
                   "foot": "Rfoot=Rtibia*Rz(ankle); pfoot=Knew+Rtibia*[0,ankleMountY,0]",
                   "support": "minheel (Rfoot*v)Y=minforefoot (Rfoot*v)Y; ankleMountY=(planeY-KnewY-minheel(Rfoot*v)Y)/RtibiaYY"},
        limits=["Geometry-only result: no force support, clinical assembly or anatomy/function qualification claim.",
                "Two heel/forefoot support vertices with roll retained do not mean every sole vertex is coplanar or a finite support patch is established.",
                "Only knee Y was fitted; X/Z residuals and benchmark masses/hardware/internal axes remain disclosed.",
                "Y=0 is an explicit benchmark convention; actual force/detector profile planes are unchanged.",
                "Recovery source after hashes refer to the current recovery audit, not an invented original-run observation time."])
    receipt["receipt_recovery"] = {
        "started_utc": started, "completed_utc": datetime.now(timezone.utc).isoformat(),
        "preserved_partial_receipt": initial_before, "recovery_helper": helper_before,
        "pure_canonical_source_root_bound": canonical_before,
        "cause": "json.dump stopped at first NumPy bool check value after model/fit/native evidence had been serialized; partial status was completed.",
        "method": "Parse only the complete JSON fields preceding checks; reconstruct built-in bool guards from saved native numeric evidence, exact new/original XML whitelist, STL algebra and current frozen source hashes.",
        "new_native_FK_or_fit_calls": 0, "new_model_writes": 0,
        "original_helper_protocol_partial_and_fitted_model_unchanged": True,
        "saved_native_recomputation": measured,
    }
    payload = json.dumps(receipt, indent=2, allow_nan=False) + "\n"
    with OUTPUT.open("x", encoding="utf-8") as stream:
        stream.write(payload)
    print(json.dumps({"status": receipt["status"], "receipt": fp(OUTPUT), "checks": checks,
                      "pylon_length_mm": receipt["mounts"]["distal_pylon_length_m"] * 1000,
                      "ankle_offset_deg": receipt["static_angular_offsets_deg"]["pros_ankle_angle"]}, indent=2))
    if receipt["status"] != "completed":
        raise ValueError("Recovered receipt failed independent guards")


if __name__ == "__main__":
    main()
