"""Independent artifact/native-pose audit after one authorized ABx preparation.

Writes only the requested audit JSON. No builder, converter, IK solver,
integrator, policy, controller or operational artifact is run or modified.
Expected dimensions and clocks derive from this subject's inputs/reference.
"""
from __future__ import annotations

import argparse
from contextlib import redirect_stdout
import csv
import hashlib
import io
import json
from pathlib import Path, PureWindowsPath
import struct
import sys
from xml.etree import ElementTree as ET

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO/"validation/ab07_pipeline_2026_10_06"))
from validate_bundle import model_info, read_storage, resolve_reference
from tools.calibrate_ab06_seasea_markers import parse_trc, set_state_from_motion, load_plugin

HERE = Path(__file__).resolve().parent
LEFT_BODIES = {"femur_l", "tibia_l", "talus_l", "calcn_l", "toes_l"}
LEFT_COORDS = {"knee_angle_l", "ankle_angle_l", "subtalar_angle_l", "mtp_angle_l"}
PROSTHETIC_BODIES = {"transfemur", "osseo_pylon", "tibia_pylon", "foot_l"}
TRANSLATIONS = {"pelvis_tx", "pelvis_ty", "pelvis_tz"}


def fp(path):
    p = Path(path).resolve()
    return {"path": str(p), "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}


def path(raw):
    p = Path(str(raw).replace("\\", "/"))
    return p.resolve() if p.is_absolute() else (REPO/p).resolve()


def metric(v):
    v = np.asarray(v, dtype=float)
    if not v.size or not np.all(np.isfinite(v)): raise ValueError("Invalid diagnostic values")
    return {"count": int(v.size), "mean": float(v.mean()), "median": float(np.median(v)),
            "min": float(v.min()), "p95": float(np.percentile(v, 95)), "max": float(v.max())}


def check(condition, label):
    if not condition: raise AssertionError(label)


def parent_xyz(tree, joint):
    node = tree.find(f".//JointSet/objects/*[@name='{joint}']")
    name = node.findtext("socket_parent_frame")
    return np.asarray([float(v) for v in node.findtext(f"frames/PhysicalOffsetFrame[@name='{name}']/translation").split()])


def anatomy(tree):
    scale = float(tree.findtext(".//JointSet/objects/CustomJoint[@name='knee_l']/SpatialTransform/TransformAxis[@name='translation1']/MultiplierFunction/scale"))
    return scale, -parent_xyz(tree, "ankle_l")[1]


def stl_vertices(mesh):
    raw = mesh.read_bytes(); count = struct.unpack_from("<I", raw, 80)[0]
    check(len(raw) == 84+50*count and count > 0, "Fixed foot must be complete binary STL")
    dtype = np.dtype([("n", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    return np.unique(np.frombuffer(raw, dtype=dtype, offset=84, count=count)["vertices"].reshape(-1, 3).astype(float), axis=0)


def native_info(model):
    state = model.initSystem()
    check(np.all(np.isfinite([state.getY().get(i) for i in range(state.getY().size())])), "Native initial state must be finite")
    names = model.getStateVariableNames()
    sea_states = sorted(str(names.get(i)) for i in range(names.getSize()) if "/forceset/SEA_" in str(names.get(i)))
    return {"coordinates": [model.getCoordinateSet().get(i).getName() for i in range(model.getCoordinateSet().getSize())],
            "muscles": [model.getMuscles().get(i).getName() for i in range(model.getMuscles().getSize())],
            "bodies": [model.getBodySet().get(i).getName() for i in range(model.getBodySet().getSize())],
            "mass_kg": sum(model.getBodySet().get(i).getMass() for i in range(model.getBodySet().getSize())),
            "state_count": names.getSize(), "sea_state_names": sea_states,
            "native_init_system": "PASS; no time integration"}


def static_metrics(model, motion_path, vertices, profile):
    import opensim as osim
    labels, rows, degrees = read_storage(motion_path)
    state = model.initSystem(); foot = model.getBodySet().get("foot_l")
    check(set(labels[1:]) == {model.getCoordinateSet().get(i).getName() for i in range(model.getCoordinateSet().getSize())}, "Static motion must provide all native coordinates")
    origin, normal = np.asarray(profile["ground"]["origin"]), np.asarray(profile["ground"]["normal"])
    points = profile.get("points", [])
    heights, pitches, clearances = [], [], []
    for row in rows:
        set_state_from_motion(model, state, dict(zip(labels[1:], row[1:])), degrees)
        x = foot.getTransformInGround(state)
        r = np.asarray([[x.R().get(i, j) for j in range(3)] for i in range(3)])
        t = np.asarray([x.p().get(i) for i in range(3)]); n_local = normal @ r
        heights.append(float(np.min(vertices @ n_local)+(t-origin) @ normal))
        pitches.append(float(np.arctan2(n_local[0], n_local[1])))
        clearances.append([float((t+r @ np.asarray(p["location"])-origin) @ normal) for p in points])
    clearances = np.asarray(clearances)
    return {"poses": len(rows), "time_range_s": rows[[0, -1], 0].tolist(),
            "median_min_stl_clearance_m": float(np.median(heights)),
            "median_sagittal_pitch_rad": float(np.median(np.unwrap(pitches))),
            "stl_clearance_m": metric(heights), "sagittal_pitch_deg": metric(np.rad2deg(pitches)),
            "binary_points": {p["name"]: {"clearance_m": metric(clearances[:, i]),
                 "contact_fraction": float(np.mean(clearances[:, i] <= 0))} for i, p in enumerate(points)},
            "scope": "Static geometry only; no measured prosthetic force or dynamic qualification"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--subject", default="AB08")
    parser.add_argument("--bundle", type=Path, default=REPO/"models/AB08_SEASEA_Threadmill")
    parser.add_argument("--before", type=Path, default=HERE/"execution_before.json")
    parser.add_argument("--output", type=Path, default=HERE/"post_build_audit.json")
    args = parser.parse_args()
    if args.output.exists(): parser.error("New audit path required; previous artifacts are never overwritten")
    bundle, subject = args.bundle.resolve(), args.subject.upper()
    report_path = bundle/"pipeline_report.json"
    report = json.loads(report_path.read_text()); before = json.loads(args.before.read_text())
    check(report["status"] == "complete" and report["subject"] == subject, "Completed requested subject build required")
    check(report["ik_method"] == "opensim_marker_ik" and report["smoke_returncode"] is None, "Real marker IK and disabled smoke test required")
    source_paths = {"audit_script": Path(__file__).resolve(), "execution_before": args.before,
                    "pipeline_report": report_path, "validator_io_helper": REPO/"validation/ab07_pipeline_2026_10_06/validate_bundle.py",
                    "calibration_pose_helper": REPO/"tools/calibrate_ab06_seasea_markers.py",
                    "semantic_xml_helper": REPO/"tools/fit_seasea_subject.py"}
    def record(name, p):
        p = Path(p).resolve(); source_paths[name] = p; return p
    model_path = record("model", bundle/(subject+"_SEASEA_stiff321_500_pi.osim"))
    healthy_path = record("subject_healthy_model", path(report["healthy_model"]))
    donor_path = record("donor_snapshot", path(report["donor_model"]))
    plugin = record("plugin", path(report["plugin"]))
    snapshots = report["preparation_references"]
    manifest_path = record("reference_manifest", path(snapshots["manifest"]))
    manifest = json.loads(manifest_path.read_text())
    check({k:v for k,v in snapshots.items() if k != "manifest"} == manifest, "Reference snapshot report and manifest must agree")
    snapshot_checks = {}
    for role, item in manifest["inputs"].items():
        frozen = record(role+"_snapshot", path(item["snapshot_file"]))
        original = record(role+"_reference_original", path(item["source_file"]))
        check(fp(frozen)["sha256"] == item["sha256"] == fp(original)["sha256"], "AB08 must preserve snapshot and original common reference bytes: "+role)
        snapshot_checks[role] = {"captured_sha256": item["sha256"], "snapshot_and_original_identical": True}
    check(donor_path == source_paths["donor_model_snapshot"], "Build must use frozen donor")
    check(fp(path("tools/build_abx_seasea_pipeline.py"))["sha256"] == before["builder_sha256"], "Builder source changed since the authorized execution receipt")
    protected = {**before["shared_parameters_and_reference_hashes"], **before["AB07_bundle_hashes"]}
    check(all(fp(path(name))["sha256"] == sha for name, sha in protected.items()), "Shared control/reference files or AB07 changed during AB08 preparation")
    current, healthy, donor = map(model_info, (model_path, healthy_path, donor_path))
    ct, ht, dt = (ET.parse(p) for p in (model_path, healthy_path, donor_path))
    check(current["sea"] == donor["sea"], "All SEA XML properties must exactly match frozen AB06 PI donor")
    expected_coord = (set(healthy["coordinates"])-LEFT_COORDS) | {"pros_knee_angle", "pros_ankle_angle"}
    expected_body = (set(healthy["bodies"])-LEFT_BODIES) | PROSTHETIC_BODIES
    removed_refs = ["/bodyset/"+v for v in LEFT_BODIES]+list(LEFT_COORDS)
    expected_muscles = sorted(f.get("name") for f in ht.findall(".//ForceSet/objects/*")
                              if "Muscle" in f.tag and not any(ref in ET.tostring(f, encoding="unicode") for ref in removed_refs))
    prosthetic_refs = ["/bodyset/"+v for v in PROSTHETIC_BODIES]+["pros_knee_angle", "pros_ankle_angle"]
    expected_muscles += sorted(f.get("name") for f in dt.findall(".//ForceSet/objects/*")
                               if "Muscle" in f.tag and any(ref in ET.tostring(f, encoding="unicode") for ref in prosthetic_refs))
    expected_muscles.sort()
    check(len(expected_muscles) == len(set(expected_muscles)), "Healthy/donor muscle graft must have unique names")
    check(set(current["coordinates"]) == expected_coord and len(current["coordinates"]) == len(expected_coord), "Coordinate contract must derive from this healthy subject")
    check(set(current["bodies"]) == expected_body, "Body graft must replace only the requested healthy left chain")
    check(sorted(current["muscles"]) == expected_muscles, "Biological muscles must match retained subject and grafted donor forces")
    check(abs(current["mass_kg"]-healthy["mass_kg"]) <= 1e-8, "Subject healthy total mass must be preserved")
    anatomical = record("anatomical_fitted_model", path(report["anatomical_model"]))
    at = ET.parse(anatomical)
    hs, hl = anatomy(ht); rs, rl = anatomy(ET.parse(source_paths["healthy_model_snapshot"]))
    mount, fixed, distal = (parent_xyz(dt, n) for n in ("osseo", "pros_knee", "pros_ankle"))
    upper_length = np.linalg.norm(mount+fixed)
    expected_prox = -np.sqrt((upper_length*hs/rs)**2-mount[0]**2-mount[2]**2)-fixed[1]
    expected_distal = distal[1]*hl/rl
    actual_prox, actual_distal = parent_xyz(at, "osseo")[1], parent_xyz(at, "pros_ankle")[1]
    check(abs(actual_prox-expected_prox) <= 1e-12 and abs(actual_distal-expected_distal) <= 1e-12, "Anatomical mounting lengths must reproduce independent reference-ratio arithmetic")
    registration = report["ground_registration"]
    reg_sources = registration["sources"]
    for key, item in reg_sources.items():
        p = record("registration_source_"+key, Path(item["path"]))
        check(fp(p)["sha256"] == item["sha256"], "Ground registration provenance source changed: "+key)
    adjustment = registration["adjustments"]
    check(abs(parent_xyz(ct, "osseo")[1]-expected_prox) <= 1e-12, "Ground registration must preserve upper mounting")
    check(abs(parent_xyz(ct, "pros_ankle")[1]-(expected_distal+adjustment["distal_mount_delta_y_m"])) <= 1e-12, "Final pylon length must include exactly the recorded ground correction")
    registered_static = record("registered_static_motion", path(report["static_calibration_motion"]))
    fitting_copy = record("bundle_static_motion", bundle/"data/fitting"/registered_static.name)
    ground_copy = record("bundle_ground_manifest", bundle/"data/fitting/ground_registration.json")
    check(fp(fitting_copy)["sha256"] == fp(registered_static)["sha256"], "Bundle static calibration motion copy differs")
    check(json.loads(ground_copy.read_text()) == registration, "Bundle registration manifest differs from pipeline report")
    ik = record("adopted_ik", path(report["ik_source"])); computed = record("computed_ik", path(report["ik_computed_source"]))
    check(fp(ik)["sha256"] == fp(computed)["sha256"], "Adopted motion must be the exact computed full marker IK")
    provenance = report["ik_provenance"]
    for key in ("source", "model", "marker"):
        p = record("ik_provenance_"+key, path(provenance[key+"_file"]))
        check(fp(p)["sha256"] == provenance[key+"_sha256"], "IK provenance hash invalid: "+key)
    check(fp(source_paths["ik_provenance_model"])["sha256"] == fp(model_path)["sha256"], "Operational model differs from IK model")
    labels, q, degrees = read_storage(ik)
    trc = parse_trc(source_paths["ik_provenance_marker"])
    check(tuple(current["coordinates"]) == labels[1:] and degrees, "IK names/order and angular units must match the model")
    check(len(q) == len(trc.times) and np.allclose(q[:, 0], trc.times, atol=1e-8, rtol=0), "Full IK must cover every TRC sample on the exact subject clock")
    check(np.allclose(q[[0, -1], 0], report["kinematics_time_range"], atol=1e-8, rtol=0), "Reported IK bounds differ from actual coverage")
    candidates = [directory/(computed.stem+suffix) for directory in (computed.parent, computed.parent/"full")
                  for suffix in ("_ik_marker_errors.sto", "_marker_errors.sto")]
    marker_error = record("ik_marker_errors", next(p for p in candidates if p.is_file()))
    elabels, errors, _ = read_storage(marker_error)
    check(np.array_equal(errors[:, 0], q[:, 0]), "Exact-stem marker error companion must have the adopted IK clock")
    marker_metrics = {n: metric(errors[:, elabels.index(n)]) for n in ("marker_error_RMS", "marker_error_max")}
    marker_csv = record("static_marker_calibration", source_paths["ik_provenance_model"].with_suffix(".marker_calibration.csv"))
    with marker_csv.open(newline="") as stream: calibration = list(csv.DictReader(stream))
    check(calibration and all(int(row["samples"]) > 0 for row in calibration), "Static marker fit must contain real samples")
    setup_path = record("setup", bundle/(subject+"_SEASEA_stiff321_500_pi_setup.xml")); setup = ET.parse(setup_path)
    check(len(list(bundle.glob("*.osim"))) == 1 and len(list(bundle.glob("*_setup.xml"))) == 1, "One operational model/setup without alias variants required")
    links = {tag: resolve_reference(setup.findtext(".//"+tag), setup_path.parent)
             for tag in ("model_file", "kinematics_file", "external_loads_xml", "reserve_actuators_xml")}
    check(links["model_file"] == model_path and links["kinematics_file"] == ik, "Setup must select the audited model and IK")
    start, end = float(setup.findtext(".//t_start")), float(setup.findtext(".//t_end"))
    check(q[0, 0]-1e-8 <= start < end <= q[-1, 0]+1e-8 and abs(start-q[0, 0]) <= 1e-8, "Setup window must start with and lie within the full trial")
    external_path = record("external_forces", links["external_loads_xml"]); external = ET.parse(external_path)
    grf = record("grf", resolve_reference(external.findtext(".//datafile"), external_path.parent))
    glabels, gv, _ = read_storage(grf)
    check(gv[0, 0] <= q[0, 0]+1e-8 and gv[-1, 0] >= q[-1, 0]-1e-8, "Measured GRF must cover the whole IK")
    loads = external.findall(".//ExternalForce")
    check({f.findtext("applied_to_body") for f in loads} == {"foot_l", "calcn_r"} and len(loads) == 2, "GRF application frames must match prosthetic/sound feet")
    for force in loads:
        check(all(force.findtext(tag).strip()+axis in glabels for tag in ("force_identifier", "point_identifier", "torque_identifier") for axis in "xyz"), "All GRF force/COP/torque columns must exist")
        check(all(force.findtext(tag).strip() == "ground" for tag in ("force_expressed_in_body", "point_expressed_in_body")), "GRF vectors/COP must use ground frame")
    for name in ("CMC_Actuators.xml", "CMC_Tasks - modified Kp_Kv.xml"):
        p = record(name, bundle/"data"/name); support = ET.parse(p)
        check({n.text.strip() for n in support.findall(".//coordinate")} <= expected_coord, "Support XML refers to absent coordinates")
        check(fp(p)["sha256"] == fp(REPO/"models/SEASEA/data"/name)["sha256"], "Support settings differ from the shared baseline source")
    overlay_metrics = {}
    for pattern in ("*Kinematics_q.sto", "*Actuation_force.sto"):
        files = list((bundle/"data/healthy").glob(pattern)); check(len(files) == 1, "Exactly one explicit healthy overlay required: "+pattern)
        p = record("healthy_overlay_"+pattern, files[0]); ol, ov, od = read_storage(p)
        check(np.array_equal(ov[:, 0], q[:, 0]), "Healthy overlay must use the complete adopted IK clock")
        overlay_metrics[pattern] = {"file": str(p), "rows": len(ov), "columns": len(ol)}
        if "Kinematics" in pattern:
            check(ol == labels and od == degrees, "Healthy kinematic overlay must match coordinate names and units")
            check(not np.allclose(ov[:, 1:], q[:, 1:], atol=1e-7, rtol=0), "Real marker IK must differ from the remapped healthy dataset")
    import opensim as osim
    from model_loader import _configure_geometry_search_paths
    record("runtime_geometry_helper", REPO/"model_loader.py")
    load_plugin(plugin); osim.Logger.setLevelString("error")
    osim.Logger.removeFileSink()
    geometry_log = io.StringIO()
    with redirect_stdout(geometry_log):
        _configure_geometry_search_paths(str(model_path))
    geometry_dirs = tuple(Path(line.split("Geometry path : ", 1)[1])
                          for line in geometry_log.getvalue().splitlines() if "Geometry path : " in line)
    check(geometry_dirs, "Runtime loader must register existing geometry search directories")
    mesh_files, mesh_casing = {}, {}
    for name in current["meshes"]:
        requested = PureWindowsPath(name).name
        directories = geometry_dirs
        literal_exists = any((directory/requested).is_file() for directory in directories)
        mesh = next((p for directory in directories if directory.is_dir()
                     for p in directory.iterdir() if p.is_file() and p.name.casefold() == requested.casefold()), None)
        check(mesh is not None, "Visual mesh asset unresolved: "+name)
        mesh_files[name] = str(record("mesh_"+name, mesh))
        mesh_casing[name] = {"requested_name": requested, "actual_name": mesh.name,
                             "exact_basename_case_match": requested == mesh.name,
                             "requested_filename_exists_on_this_filesystem": literal_exists}
    sources_before = {name: fp(p) for name, p in source_paths.items()}
    native = osim.Model(str(model_path)); ref_native = osim.Model(str(donor_path))
    info, ref_info = native_info(native), native_info(ref_native)
    check(info["coordinates"] == current["coordinates"] and sorted(info["muscles"]) == expected_muscles, "Native counts/names differ from audited XML")
    check(info["sea_state_names"] == ref_info["sea_state_names"] and info["sea_state_names"], "Native SEA internal states must match the same AB06 PI plugin/model contract")
    check(abs(info["mass_kg"]-healthy["mass_kg"]) <= 1e-8, "Native model does not preserve subject mass")
    foot_mesh = source_paths["registration_source_foot_mesh"]
    vertices = stl_vertices(foot_mesh); profile = json.loads(source_paths["registration_source_plane_profile"].read_text())
    ref_static = source_paths["static_motion_snapshot"]
    registered_geometry = static_metrics(native, registered_static, vertices, profile)
    reference_geometry = static_metrics(ref_native, ref_static, vertices, profile)
    height_error = registered_geometry["median_min_stl_clearance_m"]-reference_geometry["median_min_stl_clearance_m"]
    pitch_error = registered_geometry["median_sagittal_pitch_rad"]-reference_geometry["median_sagittal_pitch_rad"]
    check(abs(height_error) <= 1e-10 and abs(pitch_error) <= 1e-10, "Independent native median hardware height/pitch differs from common reference")
    check(all(fp(p) == sources_before[name] for name, p in source_paths.items()), "Source changed during independent audit")
    result = {"schema_version": 1, "status": "PASS_artifact_and_native_static_audit", "instruction_check_token": "CMC_AGENT_OK_2026",
              "subject": subject, "scope": "Completed bundle/provenance and native init/static pose verification; no IK solver, dynamics or policy",
              "command": before["command"], "sources": sources_before, "sources_unchanged": True,
              "shared_fingerprints_unchanged": True, "AB07_unchanged": True,
              "reference_snapshot_checks": snapshot_checks, "native_model": info, "native_reference": ref_info,
              "subject_mass_kg": healthy["mass_kg"], "SEA_matches_frozen_AB06_PI": True,
              "anatomical_fitting": {"upper_ratio": hs/rs, "shank_ratio": hl/rl,
                  "expected_proximal_y_m": float(expected_prox), "actual_proximal_y_m": float(actual_prox),
                  "expected_anatomical_distal_y_m": float(expected_distal), "actual_anatomical_distal_y_m": float(actual_distal)},
              "ground_registration": {"adjustments": adjustment, "native_subject_static": registered_geometry,
                  "native_reference_static": reference_geometry, "height_residual_m": height_error, "pitch_residual_rad": pitch_error},
              "IK": {"rows": len(q), "columns": len(labels), "time_range_s": q[[0,-1], 0].tolist(),
                  "dt_s": metric(np.diff(q[:, 0])), "finite": True, "matches_all_TRC_samples": True,
                  "marker_errors": marker_metrics, "real_marker_IK_distinct_from_healthy_remap": True},
              "static_marker_fit": {"markers": len(calibration), "mean_fit_rms_m": float(np.mean([float(r["new_fit_rms_m"]) for r in calibration])),
                                    "max_fit_error_m": max(float(r["new_fit_max_m"]) for r in calibration)},
              "setup": {"start_s": start, "end_s": end, "duration_s": end-start, "resolved_paths": {k:str(v) for k,v in links.items()}},
              "GRF": {"rows": len(gv), "columns": len(glabels), "time_range_s": gv[[0,-1],0].tolist(), "finite": True},
              "healthy_overlays": overlay_metrics, "resolved_meshes": mesh_files,
              "visual_mesh_filename_casing": mesh_casing,
              "runtime_geometry_search_paths_configured": True,
              "runtime_geometry_search_directories": [str(p) for p in geometry_dirs],
              "limitations": ["Smoke/forward simulation was disabled in the authorized preparation command.",
                  "Native init and static hardware registration do not qualify full-trial contact events, primary loads, tracking dynamics or learned-policy transfer.",
                  "Static ground convention is reference-relative to AB06 fixed hardware and virtual plane; no physical-floor measurement is inferred."]}
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
    print("Independent post-build audit saved:", args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
