"""Audit frozen profile geometry and existing receipts without OpenSim execution.

Only the requested JSON receipt is written. No calibration, model mutation,
simulation, FSM replay, training or policy evaluation is performed.
"""

from __future__ import annotations

import argparse
import ast
import csv
import hashlib
import json
from pathlib import Path
import re
import struct
from typing import Sequence
import xml.etree.ElementTree as ET

import numpy as np


REPO = Path(__file__).resolve().parents[3]
INVESTIGATION = REPO / "validation/ab07_detector_investigation_2026_10_06"
SELECTION = REPO / "validation/binary_phase_detector_v25_geometry_runs/2026-08-04_local_reach_sweep_dev02_04_08"
BINARY = SELECTION / "selected_candidate_profile.json"
PRIMARY = REPO / "online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json"
LEGACY = REPO / "online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json"
MESH = REPO / "Geometry/AM_foot_l.STL"


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def fingerprint(path: Path) -> dict:
    return {"path": path.relative_to(REPO).as_posix(),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def mesh_triangles(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    if len(raw) >= 84:
        count = struct.unpack_from("<I", raw, 80)[0]
        if count and len(raw) == 84 + count * 50:
            dtype = np.dtype([("normal", "<f4", (3,)),
                              ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
            return np.frombuffer(raw, dtype=dtype, offset=84, count=count)["vertices"].astype(float)
    values = re.findall(r"\bvertex\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)", raw.decode("ascii"))
    return np.asarray(values, dtype=float).reshape(-1, 3, 3)


def plantar_surface_y(triangles: np.ndarray, x: float, z: float) -> float:
    # Independent vertical-line reconstruction in local hardware coordinates.
    projected = triangles[:, :, (0, 2)]
    matrix = np.stack((projected[:, 1] - projected[:, 0],
                       projected[:, 2] - projected[:, 0]), axis=2)
    valid = np.abs(np.linalg.det(matrix)) > 1e-15
    weights12 = np.linalg.solve(matrix[valid],
                                (np.array([x, z]) - projected[valid, 0])[..., None])[..., 0]
    weights = np.column_stack((1 - weights12.sum(axis=1), weights12))
    inside = np.all(weights >= -1e-10, axis=1) & np.all(weights <= 1 + 1e-10, axis=1)
    hits = np.sum(weights[inside] * triangles[valid][inside, :, 1], axis=1)
    if len(hits) < 2:
        raise ValueError("Vertical line does not intersect both mesh surfaces")
    return float(np.min(hits))


def mesh_distance(point: np.ndarray, triangles: np.ndarray) -> float:
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    ab, ac, ap = b - a, c - a, point - a
    d00, d01, d11 = np.sum(ab * ab, axis=1), np.sum(ab * ac, axis=1), np.sum(ac * ac, axis=1)
    d20, d21 = np.sum(ap * ab, axis=1), np.sum(ap * ac, axis=1)
    determinant = d00 * d11 - d01 * d01
    valid = determinant > 1e-30
    u, v = np.zeros(len(a)), np.zeros(len(a))
    u[valid] = (d11[valid] * d20[valid] - d01[valid] * d21[valid]) / determinant[valid]
    v[valid] = (d00[valid] * d21[valid] - d01[valid] * d20[valid]) / determinant[valid]
    inside = valid & (u >= 0) & (v >= 0) & (u + v <= 1)
    closest = a + u[:, None] * ab + v[:, None] * ac
    distances = np.where(inside, np.linalg.norm(point - closest, axis=1), np.inf)
    for start, end in ((a, b), (b, c), (c, a)):
        edge = end - start
        length2 = np.sum(edge * edge, axis=1)
        fraction = np.divide(np.sum((point - start) * edge, axis=1), length2,
                             out=np.zeros(len(a)), where=length2 > 0)
        nearest = start + np.clip(fraction, 0, 1)[:, None] * edge
        distances = np.minimum(distances, np.linalg.norm(point - nearest, axis=1))
    return float(np.min(distances))


def trusted_geometry_crosscheck(triangles: np.ndarray, points: dict,
                               spheres: list[dict]) -> dict:
    # Execute only the two pure geometry functions; the source module also has
    # OpenSim entrypoints, so importing/running that module is unnecessary.
    source = REPO / "validation/audit_two_sensor_prescribed_geometry.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    functions = [node for node in tree.body if isinstance(node, ast.FunctionDef)
                 and node.name in {"_closest_point_on_triangle", "_minimum_mesh_distance"}]
    if len(functions) != 2:
        raise ValueError("Trusted point/triangle functions are unavailable")
    namespace = {"np": np, "Sequence": Sequence}
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(source), "exec"), namespace)
    vectors = [(p["point_m"], p["point_to_mesh_nearest_distance_m"]) for p in points.values()]
    vectors += [(s["center_m"], s["center_to_mesh_nearest_m"]) for s in spheres]
    errors = [abs(namespace["_minimum_mesh_distance"](point, triangles) - distance)
              for point, distance in vectors]
    if max(errors) > 1e-12:
        raise AssertionError("Independent point/triangle distance crosscheck failed")
    return {"source": fingerprint(source), "points_checked": len(vectors),
            "maximum_absolute_error_m": max(errors), "status": "PASS",
            "scope": "Pure STL distance arithmetic; no OpenSim import or execution"}


def vec(node: ET.Element, tag: str, default: str = "0 0 0") -> list[float]:
    return [float(x) for x in node.findtext(tag, default).split()]


def model_contract(subject: str) -> dict:
    path = REPO / f"models/{subject}_SEASEA_Threadmill/{subject}_SEASEA_stiff321_500_pi.osim"
    root = ET.parse(path)
    foot = root.find(".//BodySet/objects/Body[@name='foot_l']")
    mesh = foot.find("attached_geometry/Mesh")
    ankle = root.find(".//JointSet/objects/*[@name='pros_ankle']")
    child = ankle.find("frames/PhysicalOffsetFrame[@name='foot_offset']")
    parent = ankle.find("frames/PhysicalOffsetFrame[@name='tibia_pylon_offset']")
    mount = root.find(".//JointSet/objects/*[@name='osseo']/frames/PhysicalOffsetFrame[@name='transfemur_offset']")
    return {"source": fingerprint(path), "ankle_type": ankle.tag,
            "foot_mesh": {"filename": mesh.findtext("mesh_file"),
                          "socket_frame": mesh.findtext("socket_frame"),
                          "scale_factors": vec(mesh, "scale_factors", "1 1 1")},
            "foot_child_offset_m": vec(child, "translation"),
            "foot_child_orientation_rad": vec(child, "orientation"),
            "ankle_parent_offset_m": vec(parent, "translation"),
            "ankle_parent_orientation_rad": vec(parent, "orientation"),
            "proximal_mount_offset_m": vec(mount, "translation"),
            "stored_contact_geometry_count": len(root.findall(".//ContactGeometrySet/objects/*")),
            "markers": {m.attrib["name"]: vec(m, "location")
                        for m in root.findall(".//MarkerSet/objects/Marker")
                        if m.findtext("socket_parent_frame") == "/bodyset/foot_l"}}


def read_storage(path: Path) -> tuple[list[str], np.ndarray]:
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            if line.strip().lower() == "endheader":
                break
        labels = next(line.split() for line in stream if line.strip())
        return labels, np.loadtxt(stream)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path,
                        default=INVESTIGATION / "stage2/profile_ground_contract_audit.json")
    args = parser.parse_args()
    profile, primary, legacy = load_json(BINARY), load_json(PRIMARY), load_json(LEGACY)
    manifest = load_json(SELECTION / "manifest.json")
    selected = manifest["selected_verification"]
    v19 = load_json(REPO / "validation/binary_phase_detector_v19_geometry_receipt.json")
    v26 = load_json(REPO / "validation/binary_phase_fsm_v26_development_receipt.json")
    triangles = mesh_triangles(MESH)
    if not np.all(np.isfinite(triangles)):
        raise ValueError("Mesh is non-finite")
    vertices = triangles.reshape(-1, 3)
    points = {}
    for point in profile["points"]:
        role = point["name"]
        recorded = selected["heel" if role == "left_heel" else "toe"]
        location = np.asarray(point["location"])
        y = plantar_surface_y(triangles, float(location[0]), float(location[2]))
        surface = np.array([location[0], y, location[2]])
        points[role] = {"frame": point["frame"], "point_m": location.tolist(),
                        "reconstructed_mesh_anchor_m": surface.tolist(),
                        "local_minus_y_reach_m": float(y - location[1]),
                        "recorded_reach_m": recorded["reach_m"],
                        "anchor_reconstruction_error_m": float(np.max(np.abs(surface - recorded["surface_location_m"]))),
                        "point_to_mesh_nearest_distance_m": mesh_distance(location, triangles),
                        "anchor_to_mesh_nearest_distance_m": mesh_distance(surface, triangles)}
    models = {subject: model_contract(subject) for subject in ("AB06", "AB07")}
    legacy_marker_matches = {
        sphere["name"]: [name for name, position in models["AB06"]["markers"].items()
                         if np.linalg.norm(np.asarray(position) - sphere["location"]) < 1e-9]
        for sphere in legacy["spheres"] if sphere["side"] == "left"}
    binary_y, primary_y = profile["ground"]["origin"][1], primary["ground"]["origin"][1]
    delta = binary_y - primary_y
    threshold = next(ast.literal_eval(node.value) for node in ast.walk(ast.parse((REPO / "config.py").read_text()))
                     if isinstance(node, ast.AnnAssign) and getattr(node.target, "id", None) == "grf_contact_threshold_n")
    pose_comparison = {}
    with (INVESTIGATION / "sensor_pose_comparison.csv").open(encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            if row["case"] not in {"AB06_new_ik_filtered", "AB07_new_ik_filtered", "AB07_actual_prescribed", "AB07_actual_v26"}:
                continue
            pose_comparison.setdefault(row["case"], []).append([float(row[k]) for k in ("heel_clearance_m", "toe_clearance_m", "nominal_fz_n")])
    plane_counts = {}
    for case, values in pose_comparison.items():
        data = np.asarray(values)
        loaded = data[:, 2] > threshold
        plane_counts[case] = {"poses": len(data), "nominal_loaded_samples": int(np.sum(loaded)),
                              "scope": "Same frozen binary points and saved poses; alternate planes only in arithmetic, not runtime",
                              "planes": {name: {"y_m": y,
                                                 "heel_on": int(np.sum(data[:, 0] + binary_y - y <= 0)),
                                                 "toe_on": int(np.sum(data[:, 1] + binary_y - y <= 0)),
                                                 "any_on": int(np.sum(np.any(data[:, :2] + binary_y - y <= 0, axis=1)))}
                                         for name, y in (("frozen_binary", binary_y), ("force_plate_cop_convention", 0.0), ("primary", primary_y))}}
    left_spheres = [sphere for sphere in primary["spheres"] if sphere["side"] == "left"]
    sphere_mesh = []
    for sphere in left_spheres:
        distance = mesh_distance(np.asarray(sphere["location"]), triangles)
        sphere_mesh.append({"name": sphere["name"], "frame": sphere["frame"],
                            "center_m": sphere["location"], "radius_m": sphere["radius"],
                            "center_to_mesh_nearest_m": distance,
                            "sphere_surface_to_mesh_gap_m": max(0, distance - sphere["radius"])})
    force_plate_planes = {}
    for subject in models:
        path = REPO / f"models/{subject}_SEASEA_Threadmill/data/{subject}_SEASEA_GRF_FullSpan.mot"
        labels, data = read_storage(path)
        force_plate_planes[subject] = {"source": fingerprint(path), "exported_cop_y": {}}
        for plate in (1, 2):
            y = data[:, labels.index(f"ground_force{plate}_py")]
            loaded = data[:, labels.index(f"ground_force{plate}_vy")] > threshold
            force_plate_planes[subject]["exported_cop_y"][str(plate)] = {
                "loaded_samples": int(np.sum(loaded)), "min_m": float(np.min(y[loaded])),
                "median_m": float(np.median(y[loaded])), "max_m": float(np.max(y[loaded]))}
    units = selected["units"]
    inputs = [BINARY, PRIMARY, LEGACY, MESH, SELECTION / "manifest.json", SELECTION / "trace_manifest.json",
              REPO / "validation/binary_phase_detector_v19_geometry_receipt.json",
              REPO / "validation/binary_phase_fsm_v26_development_receipt.json",
              REPO / "validation/sweep_binary_phase_detector_v21_geometry.py",
              REPO / "validation/sweep_binary_phase_detector_v25_geometry.py",
              REPO / "validation/validate_online_grf.py", REPO / "validation/calibrate_online_grf_basis.py",
              REPO / "binary_phase_detector.py", REPO / "config.py",
              INVESTIGATION / "sensor_pose_comparison.csv", Path(__file__)]
    result = {
        "schema_version": 1, "status": "geometry_contract_audited_not_ab07_contact_qualified",
        "instruction_check_token": "CMC_AGENT_OK_2026",
        "scope": "JSON/XML/STL and saved-data arithmetic; no OpenSim load, integration, FSM replay, calibration or operational mutation",
        "sources": [fingerprint(path) for path in inputs],
        "common_grf_contact_threshold_n": threshold,
        "binary_contract": {"points": points, "ground": profile["ground"],
                            "contact_rule": profile["contact_rule"],
                            "equation": "c = n·(t_GF + R_GF (a_mesh - reach*e_y_local) - o_ground)",
                            "reach_projection": "dc/dreach = -n·R_GF e_y_local; local reach is not a world-Y subtraction or a sphere radius",
                            "runtime_marker_lookup": False,
                            "mesh_bounds_m": {"min": np.min(vertices, axis=0).tolist(), "max": np.max(vertices, axis=0).tolist()}},
        "hardware_frame_contract": {"models": models,
                                    "foot_mesh_contract_identical": models["AB06"]["foot_mesh"] == models["AB07"]["foot_mesh"],
                                    "foot_child_frame_contract_identical": all(models["AB06"][key] == models["AB07"][key] for key in ("foot_child_offset_m", "foot_child_orientation_rad")),
                                    "finding": "Selected sensors attach to fixed foot hardware frame, whose origin coincides with the PinJoint child/ankle center. Upstream fitting moves hardware and sensors together; marker recalibration cannot directly move the frozen stations. FrameGeometry scale0.2 only draws frame axes; attached foot mesh defaults to scale1."},
        "ground_registration": {"binary_y_m": binary_y, "legacy_detector_y_m": legacy["ground"]["origin"][1],
                                "primary_y_m": primary_y, "binary_minus_primary_y_m": delta,
                                "primary_metadata_offset_m": primary["metadata"]["ground_offset_along_normal_m"],
                                "legacy_source": legacy["source"], "legacy_status": legacy["metadata"]["status"],
                                "origin_lineage": "Binary V19→V21→V25 freezes legacy calibrated detector plane. validate_online_grf.py optimizes ground offset jointly with sphere radius/material against prescribed GRF; primary later lowers its effective plane to bound penetration.",
                                "physical_ground_survey_attested": False,
                                "ground_offset_code": "validation/validate_online_grf.py:537; validation/calibrate_online_grf_basis.py:261",
                                "force_plate_cop_y": force_plate_planes,
                                "cop_limit": "Exported COPs lie at y=0; this establishes the dataset application-point convention, not a surveyed shoe sole or independent treadmill-height measurement."},
        "same_points_alternate_plane_arithmetic": plane_counts,
        "primary_contact_envelope": {"total_spheres": len(primary["spheres"]), "left_spheres": sphere_mesh,
                                     "ground": primary["ground"],
                                     "left_sphere_to_mesh_gap_range_m": [min(s["sphere_surface_to_mesh_gap_m"] for s in sphere_mesh), max(s["sphere_surface_to_mesh_gap_m"] for s in sphere_mesh)],
                                     "finding": "Calibrated contact basis is a separate effective force-generating geometry, not the binary station envelope or the visible STL sole."},
        "independent_mesh_distance_crosscheck": trusted_geometry_crosscheck(triangles, points, sphere_mesh),
        "selection_lineage": {"v19_direct_mesh_rejection": v19["geometry"]["direct_mesh_surface_candidate"],
                              "older_analog_detector_marker_matches": legacy_marker_matches,
                              "marker_lineage_limit": "Legacy analog sphere centers coincide with AB06 calibrated markers. V19 kept earlier x/z ancestry but rebuilt plantar mesh anchors; V21 moved x and selected reach using AB06 development events. Final binary locations are constants on hardware frame, not runtime marker readings or an independently measured hardware installation.",
                              "v19_virtual_sole_choice": "25mm first checked common extension with sustained raw ON/OFF in DEV02/04; no event oracle in that step",
                              "v21": "Coarse/fine mesh x/reach sweep against fixed DEV02/04 canonical events; selected heel25mm/toe27mm",
                              "v25_selected_candidate_id": manifest["selected_candidate_id"],
                              "v25_parent_candidate_id": selected["parent_candidate_id"],
                              "v25_change": "Only toe reach27→27.5mm; heel25mm, x/z, mesh and plane frozen",
                              "v25_scope": manifest["data_scope"], "v25_unit_count": selected["unit_count"],
                              "v25_unit_pass_count": selected["unit_pass_count"],
                              "v25_zero_internal_gaps_cycles": sum(trial["cycle_count_checked"] for trial in selected["oracle_stance_continuity"]["trials"].values()),
                              "v25_max_confirmed_error_s": {"HS": max(unit["heel_strike"]["maximum_absolute_confirmed_error_s"] for unit in units), "TO": max(unit["toe_off"]["maximum_absolute_confirmed_error_s"] for unit in units)},
                              "v26_same_geometry_development_status": v26["status"], "v26_gate": v26["gate"],
                              "qualification_limit": "Development timing/continuity qualification on AB06 02/04/08. Total vertical GRF initial_contact is not localized heel load; no AB07 or surveyed physical sole/ground qualification follows."},
        "distinguishing_findings": [
            "No double-applied reach, meter/mm error or incorrect marker frame is present in the selected runtime profile. Independent STL intersection reconstructs both mesh anchors.",
            "Same hardware-local stations do not imply subject-invariant world clearance. The fitted mounts and newly solved pose determine foot-world transform while the plane remains fixed.",
            "Binary plane is higher, not lower, than primary by 24.3247778604 mm; switching those same points to the lower plane can only remove contacts. It cannot explain sparse AB07 heel episodes as a binary plane placed too low relative to primary.",
            "The selected points protrude 25/27.5 mm from the display mesh, and primary spheres have their own envelopes. Neither bit ON nor 20-sphere support alone certifies visible-mesh or anatomical heel contact.",
            "No stored model ContactGeometry establishes a single shared physical treadmill plane. The data export COP convention, calibrated detector plane and effective primary plane are distinct contracts."
        ],
        "limitations": ["The original full optimizer receipt that produced +14.8208231mm is not linked in the preliminary legacy profile; its source metadata and existing optimizer implementation establish calibrated origin but not an exact historical optimizer reconstruction.",
                        "Healthy-model foot comparison is a separate anatomical audit; healthy-remapped coordinates on the prosthesis cannot substitute for it.",
                        "Alternate plane counts are fixed-pose diagnostics, not proposed parameter changes or valid alternative dynamic rollouts."]}
    if max(point["anchor_reconstruction_error_m"] for point in points.values()) > 1e-10:
        raise AssertionError("Selected mesh anchor reconstruction failed")
    if abs(delta + primary["metadata"]["ground_offset_along_normal_m"]) > 1e-12:
        raise AssertionError("Primary ground-offset metadata disagrees with planes")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(args.output), "mesh_anchor_max_error_m": max(p["anchor_reconstruction_error_m"] for p in points.values()), "plane_delta_mm": delta * 1000, "scope": result["scope"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
