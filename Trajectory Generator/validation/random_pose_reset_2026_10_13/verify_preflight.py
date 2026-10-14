"""Independent, stdlib-only review of saved native reset and selection evidence."""
from pathlib import Path
import hashlib
import json
import math
import random
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
AUDIT = Path(__file__).resolve().parent
JOINTS = ("pros_knee_angle", "pros_ankle_angle")


def read(path):
    return json.loads(path.read_text(encoding="utf-8"))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def finite(value):
    if isinstance(value, dict):
        return all(finite(v) for v in value.values())
    if isinstance(value, list):
        return all(finite(v) for v in value)
    return not isinstance(value, float) or math.isfinite(value)


def main():
    plan = read(AUDIT / "frozen_manifest.json")
    selection = read(AUDIT / "preflight/selected_cases.json")
    assert len(plan["cases"]) == 401
    assert selection["ready_for_preregistered_rollouts"]
    rng = random.Random(20261013)
    expected_poses = [{JOINTS[0]: rng.uniform(-1.20, -.03),
                       JOINTS[1]: rng.uniform(-.55, .55)} for _ in range(400)]
    assert [case["pose"] for case in plan["cases"][1:]] == expected_poses
    protected = plan["protected_sha256"]
    for name, expected in protected.items():
        assert digest(ROOT / name) == expected, name
    old = read(ROOT / "Trajectory Generator/validation/reset_ik_audit_2026_10_12/frozen_manifest.json")
    assert all(protected.get(name) == sha for name, sha in old["protected_sha256"].items())
    model_name = "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim"
    model = ET.parse(ROOT / model_name)
    bounds = {}
    for name in JOINTS:
        node = model.find(f".//Coordinate[@name='{name}']")
        bounds[name] = [float(x) for x in node.findtext("range").split()]
    baseline = None
    accepted = []
    contacts = []
    outcomes = []
    for case in plan["cases"]:
        path = AUDIT / "preflight" / case["id"] / "receipt.json"
        receipt = read(path)
        assert digest(path) == selection["preflight_receipt_hashes"][str(path.relative_to(ROOT))]
        assert receipt["case"] == case and receipt["integration_steps"] == 0
        assert receipt["policy_loaded"] is False and finite(receipt)
        initial = receipt["initial_state"]
        if baseline is None:
            baseline = initial
        assert abs(initial["time_s"] - 13.946870983805102) < 1e-9
        for field in ("biological_ik_q", "biological_ik_qdot"):
            assert initial[field] == baseline[field]
        for name in initial["biological_ik_q"]:
            assert abs(initial["actual_q"][name] - initial["biological_ik_q"][name]) < 1e-10
            assert abs(initial["actual_qdot"][name] - initial["biological_ik_qdot"][name]) < 1e-10
        pose = receipt["resolved_prosthetic_pose"]
        for index, (name, sea) in enumerate(zip(JOINTS, ("SEA_Knee", "SEA_Ankle"))):
            assert bounds[name][0] <= pose[name] <= bounds[name][1]
            assert abs(initial["actual_q"][name] - pose[name]) < 1e-10
            assert initial["actual_qdot"][name] == 0
            assert abs(initial["sea_motor_angles"][sea] - pose[name]) < 1e-10
            assert initial["sea_motor_speeds"][sea] == 0
            assert abs(initial["previous_endpoint"][index] - pose[name]) < 1e-10
            assert initial["served_prosthetic_reference"][name] == {"q": pose[name], "v": 0.0, "a": 0.0}
        gate = receipt["static_contact_gate"]
        left = gate["primary_grf"]["sides"]["left"]
        force = math.sqrt(sum(float(x)**2 for x in left["force"]))
        assert abs(force - gate["peak_applied_force_norm_n"]) < 1e-7
        eligible = (gate["max_applied_penetration_m"] <= .027 and force <= gate["force_guard_n"])
        assert eligible == gate["eligible_to_integrate"]
        assert receipt["status"] == ("static_eligible" if eligible else "static_rejected")
        outcomes.append({"id": case["id"], "status": receipt["status"], "gate": gate})
        if eligible:
            accepted.append(case)
        contacts.append({"id": case["id"], "pose": pose, "eligible": eligible,
                         "penetration_m": left["penetration"], "force_norm_n": force})
    assert outcomes == selection["preflight_outcomes"]
    assert selection["cases"] == [accepted[0]] + [c for c in accepted if c["id"] != "coherent_nominal"][:20]
    result = {"passed": True, "python_version": sys.version,
              "receipts_reviewed": 401, "integration_steps": 0,
              "protected_inputs_verified": len(protected),
              "previous_protected_inputs_unchanged": len(old["protected_sha256"]),
              "model_coordinate_bounds_rad": bounds,
              "eligible_random_pool": len(accepted)-1,
              "selected_random_count": 20, "selected_ids": [c["id"] for c in selection["cases"]],
              "manifest_sha256": digest(AUDIT / "frozen_manifest.json"),
              "selection_sha256": digest(AUDIT / "preflight/selected_cases.json")}
    (AUDIT / "preflight_verification.json").write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
