"""Verify a rebuilt AB07 bundle without forward dynamics or a policy.

The archived baseline is immutable. Only distal mount Y and the calibrated
left MarkerSet may change; all native schema, SEA and shared code are checked.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
from xml.etree import ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.calibrate_ab06_seasea_markers import load_plugin, MotionData
from tools.fit_seasea_subject import _canonical
from scripts.run_opensim_sea_pipeline import default_plugin
import opensim as osim

BASE = ROOT / "validation/ab07_detector_investigation_2026_10_06/stage3/baseline/models/AB07_SEASEA_Threadmill"
STEM = "AB07_SEASEA_stiff321_500_pi"


def parse_motion(path):
    lines = path.read_text().splitlines()
    end = next(i for i, line in enumerate(lines) if line.strip().lower() == "endheader")
    data_lines = [line for line in lines[end+1:] if line.strip()]
    values = np.loadtxt(data_lines[1:], ndmin=2)
    degrees = any("indegrees=yes" in line.replace(" ", "").lower() for line in lines[:end])
    return MotionData(values[:, 0], data_lines[0].split(), values, degrees)


def fp(path):
    return {"path": str(path.relative_to(ROOT)), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def protected(tree):
    tree = deepcopy(tree)
    # Explicitly mask only allowed marker locations and distal mounting length.
    for marker in tree.findall(".//MarkerSet/objects/Marker"):
        if marker.get("name", "").startswith("L_"):
            marker.find("location").text = "0 0 0"
    joint = tree.find(".//JointSet/objects/PinJoint[@name='pros_ankle']")
    mount = joint.find("frames/PhysicalOffsetFrame[@name='tibia_pylon_offset']/translation")
    values = mount.text.split()
    values[1] = "0"
    mount.text = " ".join(values)
    return _canonical(tree.getroot())


def verify(bundle: Path):
    bundle = bundle.resolve()
    model_path = bundle / (STEM + ".osim")
    old_tree = ET.parse(BASE / model_path.name)
    tree = ET.parse(model_path)
    report = json.loads((bundle / "pipeline_report.json").read_text())
    checks = {"protected_model_semantics_unchanged": protected(tree) == protected(old_tree)}
    donor = ET.parse(ROOT / "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim")
    checks["sea_matches_AB06"] = [_canonical(e) for e in tree.findall(".//SeriesElasticActuator")] == [_canonical(e) for e in donor.findall(".//SeriesElasticActuator")]
    common = json.loads((ROOT / "validation/ab07_pipeline_2026_10_06/control_fingerprint_before.json").read_text())
    checks["twenty_control_plugin_reference_fingerprints_unchanged"] = all(fp(ROOT / p)["sha256"] == sha for p, sha in common.items())
    association = json.loads((BASE / "data/runtime_profile_association.json").read_text())
    checks["three_frozen_profiles_unchanged"] = all(fp(ROOT / v["path"])["sha256"] == v["sha256"] for v in association["profiles"].values())
    checks["shared_tasks_and_reserves_unchanged"] = all((bundle / "data" / f).read_bytes() == (BASE / "data" / f).read_bytes() for f in ("CMC_Actuators.xml", "CMC_Tasks - modified Kp_Kv.xml"))
    setup = ET.parse(bundle / (STEM + "_setup.xml"))
    targets = {"model_file": model_path, "kinematics_file": bundle / "data/IK_results_AB07_SEASEA.mot", "external_loads_xml": bundle / "data/ExternalForces.xml", "reserve_actuators_xml": bundle / "data/CMC_Actuators.xml"}
    checks["setup_binds_current_bundle"] = all((ROOT / setup.findtext(".//" + key)).resolve() == path for key, path in targets.items())
    external = ET.parse(bundle / "data/ExternalForces.xml")
    checks["external_loads_binds_current_grf"] = (bundle / "data" / external.findtext(".//datafile")).resolve() == bundle / "data/AB07_SEASEA_GRF_FullSpan.mot"
    checks["healthy_overlay_unchanged"] = all((bundle / "data/healthy" / f.name).read_bytes() == f.read_bytes() for f in (BASE / "data/healthy").iterdir() if f.is_file())
    for label, filename in (("grf", "AB07_SEASEA_GRF_FullSpan.mot"),):
        before, after = parse_motion(BASE / "data" / filename), parse_motion(bundle / "data" / filename)
        checks[label + "_numerical_data_unchanged"] = before.labels == after.labels and np.array_equal(before.values, after.values)
    ik_path = bundle / "data/IK_results_AB07_SEASEA.mot"
    ik = parse_motion(ik_path)
    grf = parse_motion(bundle / "data/AB07_SEASEA_GRF_FullSpan.mot")
    checks["new_native_marker_ik_provenance"] = report["ik_method"] == "opensim_marker_ik" and fp(ik_path)["sha256"] == report["ik_provenance"]["source_sha256"] and fp(model_path)["sha256"] == report["ik_provenance"]["model_sha256"]
    checks["full_ik_finite_schema_and_coverage"] = ik.values.shape == (28612, 22) and np.all(np.isfinite(ik.values)) and np.allclose(np.diff(ik.times), .005, rtol=0, atol=1e-9) and np.array_equal(ik.times[[0,-1]], [10.36,153.415]) and grf.times[0] <= ik.times[0] and grf.times[-1] >= ik.times[-1]
    original_static = parse_motion(ROOT / report["converted_static_dir"] / "static01_ik_dataset_ab06_seasea.mot")
    static_path = ROOT / report["static_calibration_motion"]
    static = parse_motion(static_path)
    ankle = static.labels.index("pros_ankle_angle")
    untouched = [i for i in range(len(static.labels)) if i != ankle]
    checks["static_pose_only_ankle_offset"] = static.labels == original_static.labels and np.array_equal(static.values[:, untouched], original_static.values[:, untouched]) and np.allclose(static.values[:, ankle] - original_static.values[:, ankle], report["ground_registration"]["adjustments"]["static_ankle_pose_offset_deg"], rtol=0, atol=1e-12)
    osim.Logger.setLevelString("error")
    load_plugin(default_plugin())
    model = osim.Model(str(model_path))
    state = model.initSystem()
    names = model.getStateVariableNames()
    sea_names = [names.get(i) for i in range(names.getSize()) if "/SEA_" in names.get(i)]
    schema = {"coordinate_count": model.getCoordinateSet().getSize(), "muscle_count": model.getMuscles().getSize(), "body_count": model.getBodySet().getSize(), "sea_state_count": len(sea_names), "mass_kg": model.getTotalMass(state), "sea_states": sea_names}
    checks["native_model_schema_and_mass"] = schema["coordinate_count"] == 21 and schema["muscle_count"] == 76 and schema["body_count"] == 11 and schema["sea_state_count"] == 6 and abs(schema["mass_kg"] - 58.80765053000002) < 1e-10
    errors_path = ROOT / report["pipeline_root"] / "02_ik/full/IK_full_10p360_153p415_ik_marker_errors.sto"
    errors = parse_motion(errors_path)
    rms = errors.values[:, errors.labels.index("marker_error_RMS")]
    maximum = errors.values[:, errors.labels.index("marker_error_max")]
    checks["marker_errors_follow_exact_ik"] = np.array_equal(errors.times, ik.times) and np.all(np.isfinite(errors.values))
    checks = {key: bool(value) for key, value in checks.items()}
    sources = {"model": fp(model_path), "ik": fp(ik_path), "grf": fp(bundle / "data/AB07_SEASEA_GRF_FullSpan.mot"), "static_registered": fp(static_path), "marker_error": fp(errors_path), "script": fp(Path(__file__)), "builder": fp(ROOT / "tools/build_abx_seasea_pipeline.py"), "registration": fp(ROOT / "tools/register_seasea_ground.py")}
    result = {"schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(), "instruction_check_token": "CMC_AGENT_OK_2026", "status": "PASS" if all(checks.values()) else "FAIL", "scope": "Artifact/model/provenance verification only; no dynamics, network or force qualification", "bundle": str(bundle.relative_to(ROOT)), "checks": checks, "native_schema": schema, "ik": {"rows": len(ik.times), "time_range_s": ik.times[[0,-1]].tolist(), "dynamic_marker_rms_mean_mm": float(np.mean(rms)*1000), "dynamic_marker_rms_max_mm": float(np.max(rms)*1000), "dynamic_marker_max_error_mm": float(np.max(maximum)*1000)}, "sources": sources}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    receipt = verify(args.bundle)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(receipt, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"status": receipt["status"], "checks": receipt["checks"], "ik": receipt["ik"]}))
    return 0 if receipt["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
