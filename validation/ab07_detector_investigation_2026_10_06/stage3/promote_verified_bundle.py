"""Promote the verified ground-registered AB07 preparation; never run dynamics.

Refuse to replace anything other than the exact, archived old baseline. Keep
all prior runs and their receipts, and bind new metadata to the new hashes.
"""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from tools.build_abx_seasea_pipeline import build_bundle, BuildReport, write_report_files
from validation.verify_ground_registered_bundle import verify


def read(path):
    return json.loads(path.read_text())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def main():
    stage = Path(__file__).resolve().parent
    new = stage / "build/models/AB07_SEASEA_Threadmill"
    canonical = ROOT / "models/AB07_SEASEA_Threadmill"
    baseline = stage / "baseline/models/AB07_SEASEA_Threadmill"
    archive = read(stage / "baseline/archive_manifest.json")
    assert archive["status"] == "PASS"
    for item in archive["original_to_archive"]:
        assert sha(ROOT / item["original_path"]) == item["sha256"], item["original_path"]
        assert sha(ROOT / item["archive_path"]) == item["sha256"], item["archive_path"]
    staged_check = verify(new)
    assert staged_check["status"] == "PASS", staged_check["checks"]
    contact = read(stage / "ik_contact_verification.json")
    assert contact["sources_unchanged"]
    assert contact["sources"]["AB07_ground_registered_model"]["sha256"] == sha(new / "AB07_SEASEA_stiff321_500_pi.osim")
    assert contact["sources"]["AB07_ground_registered_ik"]["sha256"] == sha(new / "data/IK_results_AB07_SEASEA.mot")
    static = read(stage / "static_marker_ik_recheck.json")
    assert all(static["checks"].values())
    assert static["sources"]["model"]["sha256"] == sha(new / "AB07_SEASEA_stiff321_500_pi.osim")
    report = read(new / "pipeline_report.json")
    files = build_bundle("AB07", "treadmill", ROOT / report["calibrated_model"], ROOT / report["ik_computed_source"],
        new / "data/AB07_SEASEA_GRF_FullSpan.mot", ROOT / "models/SEASEA/data", canonical,
        10.36, 19.37, model_stem="AB07_SEASEA_stiff321_500_pi")
    for folder in ("healthy", "fitting"):
        shutil.copytree(new / "data" / folder, canonical / "data" / folder, dirs_exist_ok=True)
    relative = lambda p: str(p.relative_to(ROOT))
    report.update(bundle_dir=relative(canonical), setup_xml=relative(files["setup"]), ik_source=relative(files["kinematics"]),
        grf_source=relative(files["grf"]), healthy_overlay_dir=relative(canonical / "data/healthy"),
        static_calibration_motion=relative(canonical / "data/fitting/AB07_SEASEA_static_ground_registered.mot"))
    report["notes"].extend(["Ground registration and marker IK verified before operational promotion.",
        "Preparation complete; new forward dynamics, actor FSM and force qualification remain pending.",
        "Prior prescribed runs concern the archived pre-correction model and IK only."])
    write_report_files(BuildReport(**report), canonical)
    final = verify(canonical)
    assert final["status"] == "PASS", final["checks"]
    final_path = stage / "artifact_verification_canonical.json"
    write(final_path, final)
    association = read(baseline / "data/runtime_profile_association.json")
    for key in ("qualification_receipt", "qualification_analysis", "completed_diagnostic_duration_s"):
        association.pop(key, None)
    association.update(model_sha256=sha(files["model"]), ik_sha256=sha(files["kinematics"]),
        status="ground_registration_and_saved_pose_contacts_verified_dynamic_qualification_pending",
        scope="Common frozen profiles bound to the new prepared model; saved-pose evidence only, no policy readiness claim",
        updated_utc=datetime.now(timezone.utc).isoformat(),
        saved_pose_verification=relative(stage / "ik_contact_verification.json"),
        native_static_ik_verification=relative(stage / "static_marker_ik_recheck.json"),
        baseline_history={"association": relative(baseline / "data/runtime_profile_association.json"),
            "verification": relative(baseline / "pipeline_verification.json"), "archive_manifest": relative(stage / "baseline/archive_manifest.json")})
    write(canonical / "data/runtime_profile_association.json", association)
    verification = {"schema_version": 2, "subject": "AB07", "trial": "treadmill_01_01", "created_utc": datetime.now(timezone.utc).isoformat(),
        "instruction_check_token": "CMC_AGENT_OK_2026",
        "status": "ground_registered_model_and_native_marker_ik_verified; V26_dynamic_qualification_pending",
        "scope": "New geometric preparation, actual marker IK and saved-pose detector replay; no new forward integration or network",
        "bundle": relative(canonical), "model": final["sources"]["model"], "kinematics": final["sources"]["ik"],
        "native_schema": final["native_schema"], "preparation": {"status": "complete", "method": "anatomical_fitting_then_donor_relative_static_hardware_ground_v1_then_marker_calibration_then_opensim_marker_ik",
            "ik": final["ik"], "ground_registration": relative(canonical / "data/fitting/ground_registration.json"),
            "marker_fit_rms_m": report["marker_new_fit_rms_m"]},
        "shared_parameters": final["checks"],
        "checks": {"artifact": relative(final_path), "regression_tests": relative(stage / "final_contract_tests.json"),
            "native_static_marker_ik": relative(stage / "static_marker_ik_recheck.json"), "saved_pose_contact_events": relative(stage / "ik_contact_verification.json")},
        "historical_pre_correction": {"archive_manifest": relative(stage / "baseline/archive_manifest.json"), "verification": relative(baseline / "pipeline_verification.json"),
            "scope": "All previously saved dynamics and plots use the old model/IK; they do not validate this new preparation"},
        "remaining_qualification": ["Fresh prescribed V26 dynamics: actor FSM, loads, tracking and reserves with the new model/IK",
            "One confirmed HS fails the inherited 50ms development timing reference (60.705ms early); saved-pose cycle recovery does not close timing qualification",
            "Known biological MTP tracking limit; Windows live verification"]}
    write(canonical / "pipeline_verification.json", verification)
    write(stage / "promotion_receipt.json", {"status": "PASS", "created_utc": datetime.now(timezone.utc).isoformat(),
        "script_sha256": sha(Path(__file__)), "archive_manifest": relative(stage / "baseline/archive_manifest.json"),
        "canonical_artifact_verification": relative(final_path), "model": final["sources"]["model"], "ik": final["sources"]["ik"],
        "dynamic_or_policy_run_started": False})
    print(json.dumps({"status": "PASS", "bundle": relative(canonical), "model_sha256": sha(files["model"]), "ik_sha256": sha(files["kinematics"])}))


if __name__ == "__main__":
    main()
