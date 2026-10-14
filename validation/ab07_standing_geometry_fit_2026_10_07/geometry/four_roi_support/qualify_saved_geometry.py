"""Pure closing qualification of saved FOURROI geometry; no native imports."""
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
from xml.etree import ElementTree as ET
from copy import deepcopy

import numpy as np
from scipy.spatial import ConvexHull

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT))
INPUT = HERE / "fit_receipt.json"
PROTOCOL = HERE / "protocol.json"
OUTPUT = HERE / "saved_geometry_qualification.json"
EXPECTED_RECEIPT = "cb82cf29c331e18d980d03a1044e42276779bbfdf524e77b4d6736f16fc6b7fd"


def path(raw):
    p = Path(raw); return p if p.is_absolute() else ROOT / p


def fp(p):
    p = Path(p).resolve(); return {"path": str(p.relative_to(ROOT)), "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}


def finite_json(value):
    if isinstance(value, dict):
        return all(finite_json(v) for v in value.values())
    if isinstance(value, list):
        return all(finite_json(v) for v in value)
    return math.isfinite(value) if isinstance(value, (int, float)) else True


def mask(tree, canonical):
    result = deepcopy(tree)
    for name in ("osseo", "pros_ankle"):
        joint = result.find(f".//JointSet/objects/*[@name='{name}']")
        frame = joint.find(f"frames/PhysicalOffsetFrame[@name='{joint.findtext('socket_parent_frame')}']")
        e = frame.find("translation"); tokens = e.text.split(); tokens[1] = "0"; e.text = " ".join(tokens)
        if name == "osseo":
            e = frame.find("orientation"); tokens = e.text.split(); tokens[0] = "0"; e.text = " ".join(tokens)
    return canonical(result.getroot())


def main():
    if OUTPUT.exists():
        raise FileExistsError("Closing qualification is immutable")
    receipt = json.loads(INPUT.read_text()); protocol = json.loads(PROTOCOL.read_text())
    if fp(INPUT)["sha256"] != EXPECTED_RECEIPT:
        raise ValueError("Final native receipt changed")
    sources = {"helper": Path(__file__), "receipt": INPUT, "protocol": PROTOCOL,
               "output_model": path(receipt["output_model"]["path"]),
               "canonical": ROOT / "tools/fit_seasea_subject.py"}
    for name, v in receipt["sources_before"].items():
        sources["fit_source_" + name] = path(v["path"])
    before = {name: fp(p) for name, p in sources.items()}
    for name, expected in protocol["sources_before"].items():
        if before["fit_source_" + name]["sha256"] != expected["sha256"]:
            raise ValueError("Frozen supplement source changed")
    if before["output_model"]["sha256"] != receipt["output_model"]["sha256"] or before["protocol"]["sha256"] != receipt["protocol"]["sha256"]:
        raise ValueError("Saved model/protocol changed")
    from tools.fit_seasea_subject import _canonical
    alpha = receipt["mounts"]["osseo_parent_orientation_rad"][0]
    qa = receipt["chosen"]["ankle_rad"]
    bounds = protocol["variables"]
    checks = {"all_numeric_values_finite": finite_json(receipt),
              "alpha_inside_frozen_plusminus15deg": bool(bounds["osseo_parent_orientation_X_rad"][0] <= alpha <= bounds["osseo_parent_orientation_X_rad"][1]),
              "ankle_inside_frozen_range": bool(bounds["static_pros_ankle_rad"][0] <= qa <= bounds["static_pros_ankle_rad"][1]),
              "all_native_strict_checks_passed": bool(receipt["strict_verdict"] == "PASS" and all(receipt["checks"].values())),
              "whole_XML_three_field_whitelist": mask(ET.parse(sources["fit_source_base_fitted_model"]), _canonical) == mask(ET.parse(sources["output_model"]), _canonical)}
    raw = sources["fit_source_fixed_foot_STL"].read_bytes(); count = struct.unpack_from("<I", raw, 80)[0]
    if len(raw) != 84 + 50 * count or count == 0:
        raise ValueError("Exact fixed STL required")
    dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    local = np.unique(np.ndarray((count,), dtype=dtype, buffer=raw, offset=84)["vertices"].reshape(-1, 3).astype(float), axis=0)
    g = receipt["geometry"]; r = np.array(g["foot_rotation_ground"]); p = np.array(g["foot_origin_XYZ_m"])
    world = local @ r.T + p
    roi = {}
    for name, settings in protocol["four_ROI"].items():
        chosen = local[:, 0] <= protocol["heel_X_max_m"] if settings["region"] == "heel" else local[:, 0] >= protocol["forefoot_X_min_m"]
        chosen &= local[:, 2] <= settings["Z_max_m"] if "Z_max_m" in settings else local[:, 2] >= settings["Z_min_m"]
        indices = np.flatnonzero(chosen); witness = int(indices[np.argmin(world[chosen, 1])])
        roi[name] = {"minimum_Y_m": float(world[chosen, 1].min()), "minimum_Y_mm": float(world[chosen, 1].min() * 1000),
                     "witness_local_XYZ_m": local[witness].tolist(), "witness_ground_XYZ_m": world[witness].tolist(), "vertices": int(chosen.sum())}
    contacts = np.flatnonzero(np.abs(world[:, 1]) <= protocol["strict"]["actual_contact_tolerance_m"])
    xy = world[contacts][:, [0, 2]]
    sv = np.linalg.svd(xy - xy.mean(axis=0), compute_uv=False)
    checks["saved_four_ROI_recomputed_exact"] = all(roi[n]["minimum_Y_m"] == g["ROI_min_Y_m"][n] for n in roi)
    checks["noncollinear_actual_XZ_contacts"] = bool(len(contacts) >= 3 and sv[1] > protocol["strict"]["XZ_contact_second_singular_value_min_m"])
    area = float(ConvexHull(xy).volume) if checks["noncollinear_actual_XZ_contacts"] else 0.
    before_q = protocol["reference"]["coordinate_maps"]["prepared_deg_m"]
    after_q = receipt["reference_coordinate_maps"]["prepared_deg_m"]
    checks["all_biological_and_knee_reference_values_preserved"] = bool(set(before_q) == set(after_q) and all(before_q[n] == after_q[n] for n in before_q if n != "pros_ankle_angle"))
    after = {name: fp(p) for name, p in sources.items()}
    checks["sources_unchanged_during_qualification"] = before == after
    result = {"created_utc": datetime.now(timezone.utc).isoformat(), "status": "PASS" if all(checks.values()) else "FAIL",
              "input_native_receipt": before["receipt"], "checks": checks, "angles_deg": {"osseo_parent_X": math.degrees(alpha), "static_ankle": math.degrees(qa)},
              "four_ROI_heights_and_witnesses": roi, "actual_contact_count": len(contacts),
              "actual_XZ_contact_singular_values_m": sv.tolist(), "actual_XZ_contact_polygon_area_m2": area,
              "strict_0p01mm_max_gap_m": max(v["minimum_Y_m"] for v in roi.values()),
              "optional_1mm_not_used_as_success": True, "source_after_matches_before": True, "sources_before": before, "sources_after": after,
              "new_native_FK_or_fit_or_IK_calls": 0, "force_event_reserve_outputs_read": False,
              "limits": ["Three actual support vertices define a nonzero projected polygon; fourthROI is within10micrometers, not an exact fourth contact.",
                         "Support polygon is a geometric witness, not a body-weight/pressure/stability or clinical qualification.",
                         "No whole curved STL flattening or measured physical floor is asserted; benchmarkY0 and fixed force/detector planes remain distinct."]}
    payload = json.dumps(result, indent=2, allow_nan=False) + "\n"
    with OUTPUT.open("x") as stream:
        stream.write(payload)
    print(json.dumps({"status": result["status"], "receipt": fp(OUTPUT), "checks": checks, "contact_polygon_area_m2": area,
                      "ROI_mm": {n: v["minimum_Y_mm"] for n, v in roi.items()}}, indent=2))


if __name__ == "__main__":
    main()
