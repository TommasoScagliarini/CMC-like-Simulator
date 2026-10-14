"""Independent saved-data review of static geometry and four bootstrap probes.

Reads no dynamic rollout results or other agents' conclusions; imports no
OpenSim, policy or frozen harness. It recomputes sphere-plane penetration and
force aggregation using the saved native points and the original profile.
"""
import hashlib
import json
import math
import struct
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path

AUDIT = Path(__file__).resolve().parent
REPO = AUDIT.parents[2]
JOINTS = ("pros_knee_angle", "pros_ankle_angle")
PROBES = ("coherent_nominal", "random_003", "random_006", "random_025")


def read(path):
    return json.loads(path.read_text(), parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def f32(value):
    return struct.unpack("f", struct.pack("f", float(value)))[0]


def close(left, right, tolerance=1e-10):
    if not math.isfinite(float(left)) or not math.isfinite(float(right)) or abs(float(left) - float(right)) > tolerance:
        raise ValueError(f"Numerical disagreement: {left}, {right}")


def check_initial(receipt, biological_reference):
    initial = receipt["initial_state"]
    pose = receipt["resolved_prosthetic_pose"]
    if len(initial["full_state"]) != len(biological_reference["full_state_after_reset"]):
        raise ValueError("Initial full state schema changed")
    close(initial["time_s"], 13.946870983805102)
    for name, value in initial["biological_ik_q"].items():
        close(initial["actual_q"][name], value)
        close(initial["actual_q"][name], biological_reference["actual_q"][name])
        close(initial["actual_qdot"][name], initial["biological_ik_qdot"][name])
        close(initial["actual_qdot"][name], biological_reference["actual_qdot"][name])
    for index, (name, motor) in enumerate(zip(JOINTS, ("SEA_Knee", "SEA_Ankle"))):
        target = pose[name]
        for actual in (initial["actual_q"][name], initial["served_prosthetic_reference"][name]["q"],
                       initial["sea_motor_angles"][motor], initial["previous_endpoint"][index]):
            close(actual, target)
        for actual in (initial["actual_qdot"][name], initial["sea_motor_speeds"][motor],
                       initial["served_prosthetic_reference"][name]["v"], initial["served_prosthetic_reference"][name]["a"]):
            close(actual, 0.0)
    for key in ("outer_pi_integral", "cascade_pi_integral"):
        for value in initial[key].values():
            close(value, 0.0)
    for states in initial["sea_auxiliary_states"].values():
        for value in states.values():
            close(value, 0.0)
    phase = initial["reset_info"]["phase_fsm"]
    for key in ("valid_hs_count", "valid_to_count", "valid_cycle_count", "invalid_event_count"):
        close(phase[key], 0.0)
    return {"biological_q_qdot_match_original_nominal_ik": True, "pose_reference_endpoint_motor_angles_match": True,
            "joint_motor_reference_v_a_and_pi_zero": True, "accepted_hs_to_cycles_at_reset": 0,
            "fsm_state_name": phase["state_name"], "partial_stance_active": phase.get("sensor_partial_stance_active")}


def main():
    json_path, md_path = AUDIT / "independent_bootstrap_preflight_review.json", AUDIT / "independent_bootstrap_preflight_review.md"
    if json_path.exists() or md_path.exists():
        raise FileExistsError("Preserve earlier review")
    inputs = {}

    def load(path):
        inputs[str(path.relative_to(REPO))] = sha(path)
        return read(path)

    plan = load(AUDIT / "frozen_manifest.json")
    selection = load(AUDIT / "preflight/selected_cases.json")
    profile = load(REPO / "online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json")
    detector_profile = load(REPO / "validation/binary_phase_detector_v25_geometry_runs/2026-08-04_local_reach_sweep_dev02_04_08/selected_candidate_profile.json")
    reference = load(REPO / "Trajectory Generator/validation/reset_ik_audit_2026_10_12/rollouts/nominal/receipt.json")["reset_reads"][0]
    model = REPO / "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim"
    inputs[str(model.relative_to(REPO))] = sha(model)
    mass = math.fsum(float(node.text) for node in ET.parse(model).getroot().findall(".//BodySet/objects/Body/mass"))
    force_limit = 5 * mass * 9.80665
    origin = profile["ground"]["origin"]
    normal = profile["ground"]["normal"]
    norm = math.sqrt(math.fsum(value * value for value in normal))
    normal = [value / norm for value in normal]
    results, eligible, geometry_max_error = {}, [], 0.0
    force_max_error, aggregation_max_error = 0.0, 0.0
    air_eligible, mismatch_primary_air_detector_contact = [], []
    reasons_count = {}
    for case in plan["cases"]:
        path = AUDIT / "preflight" / case["id"] / "receipt.json"
        receipt = load(path)
        if sha(path) != selection["preflight_receipt_hashes"][str(path.relative_to(REPO))] or receipt["case"] != case:
            raise ValueError("Preflight receipt hash/case mismatch")
        initial_checks = check_initial(receipt, reference)
        gate = receipt["static_contact_gate"]
        primary = gate["primary_grf"]
        spheres = {name: item for name, item in primary["spheres"].items() if item["side"] == "left"}
        if set(spheres) != {"online_grf_" + sphere["name"] for sphere in profile["spheres"] if sphere["side"] == "left"}:
            raise ValueError("Saved primary sphere set differs from original profile")
        clearances = {}
        for name, item in spheres.items():
            clearance = math.fsum((item["point"][i] - origin[i]) * normal[i] for i in range(3))
            expected_penetration = max(0.0, -clearance)
            delta = abs(expected_penetration - item["penetration_m"])
            geometry_max_error = max(geometry_max_error, delta)
            close(expected_penetration, item["penetration_m"])
            close(expected_penetration, item["penetration"])
            if item["in_contact"] != (expected_penetration > 0):
                raise ValueError("Primary sphere geometric contact flag differs")
            for index in range(3):
                close(item["cop_point"][index], item["point"][index] + expected_penetration * normal[index])
            clearances[name] = clearance
        aggregate = [math.fsum(item["force"][i] for item in spheres.values()) for i in range(3)]
        for index in range(3):
            error = abs(aggregate[index] - primary["sides"]["left"]["force"][index])
            aggregation_max_error = max(aggregation_max_error, error)
            close(aggregate[index], primary["sides"]["left"]["force"][index], 1e-8)
        force = math.sqrt(math.fsum(component * component for component in aggregate))
        force_max_error = max(force_max_error, abs(force - gate["peak_applied_force_norm_n"]))
        close(force, gate["peak_applied_force_norm_n"], 1e-8)
        close(force_limit, gate["force_guard_n"], 1e-8)
        max_penetration = max(max(0, -value) for value in clearances.values())
        close(max_penetration, gate["max_applied_penetration_m"])
        expected_eligible = max_penetration <= 0.027 and force <= force_limit
        if expected_eligible != gate["eligible_to_integrate"] or (receipt["status"] == "static_eligible") != expected_eligible:
            raise ValueError("Independent geometric eligibility disagrees")
        outcome = {"id": case["id"], "status": receipt["status"], "gate": gate}
        if outcome != selection["preflight_outcomes"][len(results)]:
            raise ValueError("Selection mutable outcome differs from hashed receipt")
        detector = gate["binary_detector_baseline"]
        for role, value in detector["signed_clearance_m"].items():
            if detector["contacts"][role] != (value <= 0.0):
                raise ValueError("Point detector bit does not match signed-clearance rule")
        primary_air = min(clearances.values()) >= 0.0
        detector_contact = any(detector["contacts"].values())
        if expected_eligible:
            eligible.append(case["id"])
            if primary_air:
                air_eligible.append(case["id"])
            if primary_air and detector_contact:
                mismatch_primary_air_detector_contact.append(case["id"])
        for reason in gate["rejection_reasons"]:
            reasons_count[reason] = reasons_count.get(reason, 0) + 1
        results[case["id"]] = {"eligible": expected_eligible, "max_primary_penetration_mm": max_penetration * 1000,
                                "primary_force_norm_n": force, "min_primary_surface_clearance_mm": min(clearances.values()) * 1000,
                                "all_primary_spheres_air": primary_air, "binary_detector_contacts": detector["contacts"],
                                "binary_detector_signed_clearance_m": detector["signed_clearance_m"],
                                "initial_checks": initial_checks, "rejection_reasons": gate["rejection_reasons"]}
    first20 = [name for name in eligible if name != "coherent_nominal"][:20]
    if [case["id"] for case in selection["cases"]] != ["coherent_nominal", *first20]:
        raise ValueError("Selection is not control plus first 20 eligible pool entries")
    probes = {}
    for name in PROBES:
        path = AUDIT / "probes" / name / "receipt.json"
        receipt = load(path)
        if receipt["status"] != "bootstrap_probe_complete_without_integration" or receipt["integration_steps"] != 0:
            raise ValueError("Bootstrap native probe incomplete or physically integrated")
        initial_checks = check_initial(receipt, reference)
        initial, segment = receipt["initial_state"], receipt["first_governor_segment"]
        pose = receipt["resolved_prosthetic_pose"]
        actor = dict(zip(receipt["actor_feature_names"], initial["observation"][:35]))
        for index, (joint, motor) in enumerate(zip(JOINTS, ("SEA_Knee", "SEA_Ankle"))):
            for value in (segment["raw_anchor_q"][index], segment["filter_state_before"]["q"][index], segment["served_initial_q"][joint]):
                close(value, pose[joint])
            for value in (segment["raw_anchor_v"][index], segment["filter_state_before"]["v"][index], segment["filter_state_before"]["a"][index],
                          segment["served_initial_v"][joint], segment["served_initial_a"][joint]):
                close(value, 0.0)
            for field in (joint, joint + "_previous_endpoint", joint + "_served_ref", motor + "_motor_angle"):
                close(actor[field], f32(pose[joint]), 0.0)
            for field in (joint + "_vel", joint + "_served_ref_vel", joint + "_served_ref_accel", joint + "_sea_u", motor + "_motor_speed"):
                close(actor[field], 0.0, 0.0)
        raw, applied = receipt["first_raw_policy_action"], receipt["first_applied_policy_action"]
        if len(raw) != 2 or len(applied) != 1 or len(applied[0]) != 2:
            raise ValueError("Probe action shape mismatch")
        for old, new in zip(raw, applied[0]):
            close(new, min(max(f32(old), -1.0), 1.0), 0.0)
        probes[name] = {"initial_checks": initial_checks, "first_segment_q_v_a_and_filter_match_initial_conditions": True,
                        "actor_position_and_endpoint_exact_float32_pose": True, "actor_velocities_accelerations_control_zero": True,
                        "raw_action": raw, "applied_action": applied,
                        "geometry": results[name], "integration_steps": 0}
    unchanged = all(sha(REPO / name) == expected for name, expected in inputs.items())
    output = {"schema_version": 1, "instruction_check_token": "CMC_AGENT_OK_2026", "created_utc": datetime.now(timezone.utc).isoformat(),
              "review_status": "saved_geometry_and_bootstrap_checks_confirmed", "native_agent_execution": False,
              "dynamic_rollout_results_read": False, "scientific_qualification": "not_assessed",
              "model_mass_kg_from_xml": mass, "five_bw_force_guard_n": force_limit,
              "candidate_count": 400, "random_eligible_count": len(eligible) - 1, "random_rejected_count": 401 - len(eligible),
              "first20_selected_random_ids": first20, "selection_verified_against_401_hashed_receipts": True,
              "sphere_plane_penetration_max_abs_error_m": geometry_max_error,
              "force_aggregation_max_abs_error_n": aggregation_max_error, "force_norm_max_abs_error_n": force_max_error,
              "rejection_reason_counts": reasons_count,
              "eligible_all_primary_spheres_air_ids": air_eligible,
              "eligible_primary_air_but_point_detector_contact_ids": mismatch_primary_air_detector_contact,
              "all_candidates": results, "native_bootstrap_probes": probes,
              "contact_contract": {"primary_ground": profile["ground"], "binary_point_detector_ground": detector_profile["ground"],
                                   "note": "Primary sphere-plane force/penetration and geometric point detector use distinct points and plane origins. Partial stance initialization with zero accepted HS/TO/cycles is not a fabricated heel strike. No air-clearance cap was applied."},
              "input_sha256": inputs, "read_inputs_unchanged": unchanged,
              "limitations": ["Sphere-plane consistency is checked from saved native FK contact points, not a fresh independent reconstruction of full OpenSim FK.",
                              "Four probes plan one policy segment without physical integration. They verify bootstrap continuity, not recovery or five-second walking.",
                              "Initial biological motion and prescribed right GRF remain externally supported; this is not a full-body standing-start test."]}
    if not unchanged:
        raise ValueError("Read input changed during verification")
    json_path.write_text(json.dumps(output, indent=2, allow_nan=False) + "\n")
    lines = ["# Review indipendente del preflight e bootstrap — 2026-10-13", "",
             f"Confermati in sola lettura 401 reset statici: controllo coerente + 400 candidati, **{output['random_eligible_count']} ammissibili e {output['random_rejected_count']} respinti**. La selezione coincide con i primi 20 ammissibili nell'ordine congelato; nessun risultato di rollout dinamico è stato letto.", "",
             f"Penetrazione ricalcolata dai punti nativi delle otto sfere sinistre rispetto al piano primario: errore massimo {geometry_max_error:.3g} m. Somma delle forze e norma confrontate col guard 5 BW ricavato dalla massa XML {mass:.11f} kg ({force_limit:.9f} N). Gate 27 mm verificato; nessun cap di clearance in aria.", "",
             f"Fra gli ammissibili, {len(air_eligible)} reset (controllo incluso) hanno tutte le sfere primarie in aria; {len(mismatch_primary_air_detector_contact)} hanno contemporaneamente almeno un point-detector attivo. I profili usano piani diversi: primario y={origin[1]:.9f} m, detector y={detector_profile['ground']['origin'][1]:.9f} m, oltre a punti diversi. È una distinzione di contratto sensoriale, non un heel strike valido inventato.", "",
             "Confermati i quattro probe nativi senza integrazione:", "",
             "| Caso | Pose q/ref/SEA/endpoint | v/a/PI iniziali | Primo segmento | Stato FSM | HS/TO/cicli |",
             "|---|---|---|---|---|---|"]
    for name, probe in probes.items():
        checks = probe["initial_checks"]
        lines.append(f"| {name} | coerenti | zero | q iniziale coerente, v=a=0 | {checks['fsm_state_name']} | 0/0/0 |")
    lines += ["", "Gli ingressi actor per q e previous endpoint coincidono con la pose dopo conversione float32; velocità, accelerazioni della reference e comando SEA sono zero. Gli anchor e lo stato interno del governor coincidono con le condizioni iniziali, senza ritorno alla IK protesica nella prima pianificazione. La biologia q/qdot coincide con la IK nominale del controllo precedente; raw action viene trasformata in (1,2) e clippata con la stessa regola del wrapper originale.", "",
              "Nessun verdetto di recupero, cammino autonomo o qualificazione scientifica. L'analisi usa punti FK già registrati; non ricalcola una nuova FK OpenSim. Tutti gli input letti e gli SHA sono nel JSON e risultano invariati. Nessun file congelato o output nativo è stato modificato.", ""]
    md_path.write_text("\n".join(lines))
    print(json.dumps({"status": output["review_status"], "eligible": output["random_eligible_count"], "rejected": output["random_rejected_count"],
                      "primary_air_eligible_including_control": len(air_eligible), "primary_air_point_contact_including_control": len(mismatch_primary_air_detector_contact),
                      "rejection_reasons": reasons_count, "probe_count": len(probes), "inputs_unchanged": unchanged}))


if __name__ == "__main__":
    main()
