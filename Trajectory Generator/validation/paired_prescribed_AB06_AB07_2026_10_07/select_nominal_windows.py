"""Freeze prescribed comparison windows from measured GRF, without simulation."""
from __future__ import annotations

import ast
import csv
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import List, Optional
import xml.etree.ElementTree as ET

import numpy as np

REPO = Path(__file__).resolve().parents[3]
AUDIT = Path(__file__).resolve().parent
OUTPUT = REPO / "output.py"
ANALYZER = REPO / "Trajectory Generator/validation/AB07/analyze_prescribed_v26.py"
DURATION_S = 9.0
MIN_RESET_AFTER_SETUP_START_S = 2.0
PRIMARY_CYCLE_COUNT = 3
THRESHOLD_N = 20.0
MIN_CONTACT_S = 0.05
MIN_CYCLE_S = 0.3


def fingerprint(path: Path) -> dict:
    return {"path": str(path.relative_to(REPO)), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def helper(path: Path, name: str):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    node = next(item for item in tree.body if isinstance(item, ast.FunctionDef) and item.name == name)
    namespace = {"np": np, "Path": Path, "List": List, "Optional": Optional}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[])), str(path), "exec"), namespace)
    return namespace[name]


def cycle_dict(row: tuple) -> dict:
    hs, next_hs, contact = row
    return {"hs_s": float(hs), "to_s": float(hs + contact), "next_hs_s": float(next_hs), "period_s": float(next_hs - hs), "stance_s": float(contact), "stance_fraction": float(contact / (next_hs - hs))}


def select_subject(subject: str, read_storage, extract_cycles) -> dict:
    setup = REPO / f"models/{subject}_SEASEA_Threadmill/{subject}_SEASEA_stiff321_500_pi_setup.xml"
    root = ET.parse(setup).getroot().find("CMC_Simulator_Setup")
    source_start, source_end = float(root.findtext("t_start")), float(root.findtext("t_end"))
    model = REPO / root.findtext("model_file")
    ik = REPO / root.findtext("kinematics_file")
    loads = REPO / root.findtext("external_loads_xml")
    reserve = REPO / root.findtext("reserve_actuators_xml")
    external = ET.parse(loads).getroot().find("ExternalLoads")
    grf = loads.parent / external.findtext("datafile")
    force = next(item for item in external.findall("objects/ExternalForce") if item.findtext("applied_to_body") == "foot_l")
    force_column = force.findtext("force_identifier") + "y"
    gt, labels, data, _ = read_storage(grf)
    kt, _klabels, _kdata, _degrees = read_storage(ik)
    assert np.all(np.diff(gt) > 0) and np.all(np.diff(kt) > 0)
    data_start, data_end = max(float(gt[0]), float(kt[0])), min(float(gt[-1]), float(kt[-1]))
    cycles = extract_cycles(gt, data[:, labels.index(force_column)], THRESHOLD_N, max(data_start, source_start), data_end, MIN_CONTACT_S, MIN_CYCLE_S)
    earliest_allowed = source_start + MIN_RESET_AFTER_SETUP_START_S
    candidates = []
    chosen = None
    for index, row in enumerate(cycles):
        hs = float(row[0])
        reason = None
        if hs < earliest_allowed:
            reason = "heel_strike_precedes_setup_start_plus_2s"
        elif hs + DURATION_S > data_end:
            reason = "requested_9s_window_exceeds_common_IK_and_GRF_coverage"
        elif index + PRIMARY_CYCLE_COUNT >= len(cycles):
            reason = "fewer_than_one_startup_plus_three_primary_cycles"
        else:
            selected = cycles[index:index + PRIMARY_CYCLE_COUNT + 1]
            if any(abs(selected[pos][1] - selected[pos + 1][0]) > 1e-10 for pos in range(len(selected) - 1)):
                reason = "filtered_cycles_are_not_contiguous"
            elif selected[-1][1] > hs + DURATION_S:
                reason = "three_primary_cycles_do_not_close_within_requested_window"
            else:
                chosen = selected
        candidates.append({"hs_s": hs, "eligible": reason is None, "rejection_reason": reason})
        if chosen is not None:
            break
    if chosen is None:
        raise ValueError(f"No predeclared eligible window for {subject}")
    reset = float(chosen[0][0])
    primary = [cycle_dict(row) for row in chosen[1:]]
    model_root = ET.parse(model).getroot()
    body_masses = {item.attrib["name"]: float(item.findtext("mass")) for item in model_root.findall(".//BodySet/objects/Body")}
    total_mass = sum(body_masses.values())
    gravity_text = model_root.findtext(".//Model/gravity") or model_root.findtext(".//gravity")
    gravity = [float(value) for value in gravity_text.split()] if gravity_text else None
    sea_properties = {}
    for item in model_root.findall(".//ForceSet/objects/SeriesElasticActuator"):
        sea_properties[item.attrib["name"]] = {key: item.findtext(key) for key in ["coordinate", "optimal_force", "motor_inertia", "motor_damping", "stiffness", "Kp", "Kd", "Impedence"]}
    csv_path = AUDIT / f"primary_gait_events_{subject}.csv"
    with csv_path.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=["side", "cycle_start", "cycle_end", "cycle_duration_s", "contact_duration_s", "source_force", "threshold_n", "min_contact_duration_s", "min_cycle_duration_s"])
        writer.writeheader()
        for row in primary:
            writer.writerow({"side": "left", "cycle_start": f"{row['hs_s']:.8f}", "cycle_end": f"{row['next_hs_s']:.8f}", "cycle_duration_s": f"{row['period_s']:.8f}", "contact_duration_s": f"{row['stance_s']:.8f}", "source_force": force_column, "threshold_n": f"{THRESHOLD_N:.8f}", "min_contact_duration_s": f"{MIN_CONTACT_S:.8f}", "min_cycle_duration_s": f"{MIN_CYCLE_S:.8f}"})
    return {"setup_original": str(setup.relative_to(REPO)), "setup_original_time_range_s": [source_start, source_end], "reset_hs_s": reset, "start_offset_s": reset - source_start, "requested_end_s": reset + DURATION_S, "duration_s": DURATION_S, "minimum_allowed_reset_hs_s": earliest_allowed, "common_source_data_time_range_s": [data_start, data_end], "IK_time_range_s": [float(kt[0]), float(kt[-1])], "GRF_time_range_s": [float(gt[0]), float(gt[-1])], "startup_cycle": cycle_dict(chosen[0]), "primary_cycles": primary, "primary_window_s": [primary[0]["hs_s"], primary[-1]["next_hs_s"]], "screened_candidates": candidates, "model_mass_kg_from_XML": total_mass, "model_body_masses_kg_from_XML": body_masses, "model_gravity_m_s2": gravity, "BW_normalization_definition": "runtime model total mass times9.80665; native runtime mass will be checked against XML sum", "BW_normalization_n": total_mass * 9.80665, "SEA_properties_from_XML": sea_properties, "left_force_column": force_column, "requires_derived_setup_t_end_extension": reset + DURATION_S > source_end, "derived_setup_rule": "Preserve all source paths and t_start; set only t_end=reset_hs_s+9.0 in a new audit-local setup copy. Apply start_offset_s=reset_hs_s-original_t_start; canonical setup is unchanged.", "primary_reference_csv": fingerprint(csv_path), "source_fingerprints": {key: fingerprint(path) for key, path in [("setup", setup), ("model", model), ("IK", ik), ("external_loads", loads), ("measured_GRF", grf), ("reserve_actuators", reserve)]}}


def main() -> None:
    if (AUDIT / "selection.json").exists():
        raise FileExistsError("Frozen selection already exists; refuse to rewrite")
    read_storage = helper(ANALYZER, "read_storage")
    extract_cycles = helper(OUTPUT, "_cycles_from_vertical_grf")
    subjects = {subject: select_subject(subject, read_storage, extract_cycles) for subject in ["AB06", "AB07"]}
    result = {"schema_version": 1, "instruction_check_token": "CMC_AGENT_OK_2026", "created_utc": datetime.now(timezone.utc).isoformat(), "status": "frozen_before_paired_rollouts", "criterion": {"reset_selection": "First measured-GRF left HS at least2.0s after original setup t_start, with9.0s fully covered by both original IK and measured GRF, and one contiguous startup plus3 subsequent complete nominal cycles available in that window. Reject earlier candidates in temporal order; never select after observing simulation outputs.", "duration_s": DURATION_S, "minimum_delay_from_original_setup_t_start_s": MIN_RESET_AFTER_SETUP_START_S, "startup_cycles_excluded_from_primary_metrics": 1, "primary_cycle_count_each_subject": PRIMARY_CYCLE_COUNT, "measured_GRF_event_threshold_n": THRESHOLD_N, "min_contact_duration_s": MIN_CONTACT_S, "min_cycle_duration_s": MIN_CYCLE_S, "cycle_source": "Existing production output._cycles_from_vertical_grf, AST-selected as a pure helper", "event_precision": "selection retains original full floating precision; optional primary CSV follows production8-decimal convention"}, "subjects": subjects, "helper_sources": {"selector": fingerprint(Path(__file__)), "cycle_extractor": fingerprint(OUTPUT), "storage_reader": fingerprint(ANALYZER)}, "scope": "Measured-data selection and new audit artifacts only. No simulator, policy, environment or detector runtime constructed, no rollout, no original model/IK/GRF/setup modification."}
    output = AUDIT / "selection.json"
    serialized = json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    with output.open("x", encoding="utf-8") as stream:
        stream.write(serialized)
    print(json.dumps({"selection": fingerprint(output), "subjects": {key: {field: value[field] for field in ["reset_hs_s", "start_offset_s", "requested_end_s", "startup_cycle", "primary_cycles", "primary_window_s", "model_mass_kg_from_XML"]} for key, value in subjects.items()}}, indent=2))


if __name__ == "__main__":
    main()
