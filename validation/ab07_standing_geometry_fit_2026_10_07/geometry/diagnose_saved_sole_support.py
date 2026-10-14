"""Pure saved-transform sole margin diagnostic, with no FK/fit/solver.

Medial/lateral names follow source marker-label local-Z conventions only;
uncalibrated anatomical marker locations are not prosthetic measurements.
Outer strips are fixed to20% of each declared region's own local-Z span.
"""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import struct
from xml.etree import ElementTree as ET

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
INPUT = HERE / "fit_geometry_receipt_completed.json"
OUTPUT = HERE / "saved_sole_support_diagnostic.json"
EXPECTED_RECEIPT_SHA = "230ea4bf5b985537fe5f5fd2a1ec277743db92dee9f59935b5b3a32a53d236cc"


def fp(path):
    p = Path(path).resolve()
    return {"path": str(p.relative_to(ROOT)), "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}


def main():
    if OUTPUT.exists():
        raise FileExistsError("Existing diagnostic is immutable")
    receipt_fp = fp(INPUT)
    if receipt_fp["sha256"] != EXPECTED_RECEIPT_SHA:
        raise ValueError("Completed native geometry receipt changed")
    receipt = json.loads(INPUT.read_text())
    model = ROOT / receipt["output_model"]["path"]
    mesh = ROOT / receipt["sources_before"]["fixed_foot_STL"]["path"]
    sources = [Path(__file__), INPUT, model, mesh]
    before = {fp(p)["path"]: fp(p)["sha256"] for p in sources}
    if fp(model)["sha256"] != receipt["output_model"]["sha256"] or fp(mesh)["sha256"] != receipt["sources_before"]["fixed_foot_STL"]["sha256"]:
        raise ValueError("Saved model/STL changed")
    raw = mesh.read_bytes(); count = struct.unpack_from("<I", raw, 80)[0]
    if len(raw) != 84 + 50 * count or not count:
        raise ValueError("Exact binary STL expected")
    dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    local = np.unique(np.ndarray((count,), dtype=dtype, buffer=raw, offset=84)["vertices"].reshape(-1, 3).astype(float), axis=0)
    r = np.array(receipt["geometry"]["foot_rotation_ground"])
    p = np.array(receipt["geometry"]["fitted_ankle_XYZ_m"])
    if not np.isfinite(local).all() or np.max(np.abs(r.T @ r - np.eye(3))) > 1e-12 or abs(np.linalg.det(r) - 1.) > 1e-12:
        raise ValueError("Saved proper rotation/finite points expected")
    world = local @ r.T + p
    heel = local[:, 0] <= receipt["support_regions"]["heel_X_max_m"]
    fore = local[:, 0] >= receipt["support_regions"]["forefoot_X_min_m"]
    tol = 1e-10
    expected_mins = {"all": receipt["geometry"]["whole_STL_min_Y_m"],
                     "heel": receipt["geometry"]["heel_min_Y_m"], "forefoot": receipt["geometry"]["forefoot_min_Y_m"]}
    bands = {}
    for name, region in (("all", np.ones(len(local), dtype=bool)), ("heel", heel), ("forefoot", fore)):
        if float(world[region, 1].min()) != expected_mins[name]:
            raise ValueError("Saved transformed geometry differs from completed fit")
        zlo, zhi = float(local[region, 2].min()), float(local[region, 2].max())
        masks = {"negative_Z_lateral_outer20pct": region & (local[:, 2] <= zlo + .2 * (zhi - zlo)),
                 "positive_Z_medial_outer20pct": region & (local[:, 2] >= zhi - .2 * (zhi - zlo))}
        result = {"local_Z_bounds_m": [zlo, zhi], "edge_fraction": .2}
        for side, mask in masks.items():
            indices = np.flatnonzero(mask)
            witness = int(indices[np.argmin(world[mask, 1])])
            result[side] = {"vertices": int(mask.sum()), "minimum_Y_m": float(world[mask, 1].min()),
                            "minimum_Y_mm": float(world[mask, 1].min() * 1000),
                            "maximum_Y_m": float(world[mask, 1].max()),
                            "witness_local_XYZ_m": local[witness].tolist(), "witness_ground_XYZ_m": world[witness].tolist(),
                            "support_vertices_at_tolerance": int(np.count_nonzero(np.abs(world[mask, 1]) <= tol))}
        bands[name] = result
    tree = ET.parse(model)
    marker_conventions = {}
    for name in ("L_Toe_Med", "L_Toe_Lat"):
        marker = tree.find(f".//MarkerSet/objects/Marker[@name='{name}']")
        marker_conventions[name] = {"parent_frame": marker.findtext("socket_parent_frame"),
                                    "location_m": list(map(float, marker.findtext("location").split()))}
    if not (marker_conventions["L_Toe_Med"]["location_m"][2] > 0 > marker_conventions["L_Toe_Lat"]["location_m"][2]):
        raise ValueError("Declared anatomical marker-label sign convention differs")
    support_indices = np.flatnonzero(np.abs(world[:, 1]) <= tol)
    after = {fp(p)["path"]: fp(p)["sha256"] for p in sources}
    if before != after:
        raise ValueError("A diagnostic input changed")
    result = {"created_utc": datetime.now(timezone.utc).isoformat(), "status": "PASS", "input_receipt": receipt_fp,
              "method": "Direct algebra local vertices @ saved native R.T + saved native p; no new FK or geometric fit",
              "benchmark_plane_Y_m": 0., "edge_band_rule": "Outer20% of each declared region's own local-Z span; diagnostic only, not new fit criterion",
              "anatomical_marker_label_local_Z_convention": marker_conventions,
              "edge_heights": bands, "roll_diagnostic_deg": receipt["geometry"]["foot_lateral_roll_diagnostic_deg"],
              "support_count_at_1e_10m": len(support_indices),
              "support_vertices": [{"index": int(i), "local_XYZ_m": local[i].tolist(), "ground_XYZ_m": world[i].tolist(),
                                    "heel": bool(heel[i]), "forefoot": bool(fore[i])} for i in support_indices],
              "new_native_FK_calls": 0, "new_fit_calls": 0, "new_IK_calls": 0, "parameter_changes": [],
              "sources_unchanged": True, "sources_before": before, "sources_after": after,
              "limits": ["Heel/forefoot two-region contact and nonpenetration are verified; this result does not assert full medial-lateral sole flattening or finite-area support.",
                         "STL is identical fixed visual hardware geometry; planeY0 is a benchmark, not measured floor/shoe truth.",
                         "Medial/lateral signs use anatomical marker-label conventions inherited by the uncalibrated source. These markers are not independent measured prosthetic fiducials.",
                         "No force support, load distribution, clinical alignment, anatomy causality or controller qualification follows from these heights."]}
    payload = json.dumps(result, indent=2, allow_nan=False) + "\n"
    with OUTPUT.open("x") as stream:
        stream.write(payload)
    print(json.dumps({"status": result["status"], "receipt": fp(OUTPUT), "edge_heights": bands,
                      "support_vertices": result["support_vertices"]}, indent=2))


if __name__ == "__main__":
    main()
