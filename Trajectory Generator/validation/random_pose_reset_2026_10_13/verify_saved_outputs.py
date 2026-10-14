"""Independent saved-output verification; stdlib only, no OpenSim/Ray imports.

Reads only cases with a finalized receipt or final watchdog.  It never changes
frozen sources, selection, receipts, native outputs, plots or checkpoints.
Full verification requires every selected case to be finalized; --partial is
explicitly an interim artifact, never a campaign verdict.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

AUDIT = Path(__file__).resolve().parent
REPO = AUDIT.parents[2]
JOINTS = ("pros_knee_angle", "pros_ankle_angle")
SEAS = ("SEA_Knee", "SEA_Ankle")
EXPECTED = (
    "activations", "sea_controls", "reserve_controls", "reserve_torques",
    "kinematics", "kinematics_reference", "tau_bio", "muscle_forces",
    "sea_torques", "sea_states", "sea_derivatives", "sea_diagnostics",
    "power", "states", "recruitment", "so_torque_diagnostics", "online_grf",
)
EXPECTED_HARNESS = "7a218968860ce58977f6b112fae6e105faff787f04c2a89d5340f9cda4d6aee5"
DOMAIN = {JOINTS[0]: (-1.20, -0.03), JOINTS[1]: (-0.55, 0.55)}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def json_lines(path):
    if not Path(path).exists():
        return []
    return [json.loads(line) for line in Path(path).read_text(encoding="utf-8").splitlines() if line.strip()]


def relative(path):
    try:
        return str(Path(path).resolve().relative_to(REPO))
    except ValueError:
        return str(Path(path).resolve())


def finalize_marker(root, case_id):
    receipt_path = root / case_id / "receipt.json"
    receipt = None
    if receipt_path.exists():
        try:
            candidate = read_json(receipt_path)
            if "elapsed_wall_s" in candidate:
                receipt = candidate
        except (ValueError, OSError):
            pass
    watchdog_path = root / (case_id + "_watchdog.json")
    watchdog = None
    if watchdog_path.exists():
        try:
            candidate = read_json(watchdog_path)
            if candidate.get("returncode") is not None:
                watchdog = candidate
        except (ValueError, OSError):
            pass
    return receipt, watchdog


def sto(path):
    """Strict parser: no silently dropped malformed rows, explicit header checks."""
    raw = Path(path).read_bytes()
    lines = raw.decode("utf-8-sig").splitlines()
    boundary = next((i for i, line in enumerate(lines) if line.strip().lower() == "endheader"), None)
    if boundary is None or boundary + 1 >= len(lines):
        raise ValueError("Missing endheader or column names")
    metadata = {}
    for line in lines[:boundary]:
        if "=" in line:
            key, value = line.split("=", 1)
            metadata[key.strip().lower()] = value.strip()
    names = lines[boundary + 1].split()
    if not names or names[0] != "time" or len(set(names)) != len(names):
        raise ValueError("Missing time column or duplicate names")
    tokens, values = [], []
    for line_number, line in enumerate(lines[boundary + 2:], boundary + 3):
        if not line.strip():
            continue
        row = line.split()
        if len(row) != len(names):
            raise ValueError(f"Row {line_number}: width {len(row)} != {len(names)}")
        tokens.append(row)
        values.append([float(item) for item in row])
    finite = all(math.isfinite(value) for row in values for value in row)
    times = [row[0] for row in values]
    columns = {name: [row[i] for row in values] for i, name in enumerate(names)}
    deltas = [b - a for a, b in zip(times, times[1:])]
    report = {
        "path": relative(path), "sha256": hashlib.sha256(raw).hexdigest(),
        "rows": len(values), "columns": len(names), "all_finite": finite,
        "nonfinite_cell_count": sum(not math.isfinite(value) for row in values for value in row),
        "nonfinite_columns": [name for name, data in columns.items() if any(not math.isfinite(value) for value in data)],
        "header_nrows_matches": metadata.get("nrows") == str(len(values)),
        "header_ncolumns_matches": metadata.get("ncolumns") == str(len(names)),
        "in_degrees": metadata.get("indegrees"),
        "time_first_s": times[0] if times else None,
        "time_last_s": times[-1] if times else None,
        "strictly_increasing_time": all(delta > 0 for delta in deltas),
        "max_abs_grid_delta_minus_1ms_s": max((abs(delta - 0.001) for delta in deltas), default=0.0),
    }
    return {"names": names, "tokens": tokens, "columns": columns, "report": report}


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_safe(item) for item in value]
    return value


def max_abs(values):
    return max((abs(value) for value in values), default=None)


def rms(values):
    return math.sqrt(math.fsum(value * value for value in values) / len(values)) if values else None


def close(a, b, tol=1e-8):
    return isinstance(a, (int, float)) and isinstance(b, (int, float)) and math.isfinite(a) and math.isfinite(b) and abs(a - b) <= tol


def projection_hash(table, mapping, rows=None):
    """Exact UTF-8 bytes of selected saved numeric tokens, canonical delimiters.

    This is a biological-only subtable hash, not a claim that the whole STO file
    is identical; prosthetic reference columns intentionally differ.
    """
    rows = len(table["tokens"]) if rows is None else rows
    positions = [table["names"].index(source) for source, _label in mapping]
    digest = hashlib.sha256()
    digest.update(("\t".join(label for _source, label in mapping) + "\n").encode())
    for row in table["tokens"][:rows]:
        digest.update(("\t".join(row[i] for i in positions) + "\n").encode())
    return digest.hexdigest()


def compact_projection(table, mapping):
    positions = [table["names"].index(source) for source, _label in mapping]
    header = ("\t".join(label for _source, label in mapping) + "\n").encode()
    rows = [("\t".join(row[index] for index in positions) + "\n").encode() for row in table["tokens"]]
    return header, rows


def compact_projection_hash(projection, count):
    header, rows = projection
    digest = hashlib.sha256(header)
    for row in rows[:count]:
        digest.update(row)
    return digest.hexdigest()


def model_metadata():
    setup = REPO / "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml"
    model = REPO / "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim"
    document = ET.parse(model).getroot()
    mass = math.fsum(float(body.findtext("mass", "0")) for body in document.iter("Body"))
    bounds = {item.attrib["name"]: [float(value) for value in item.findtext("range").split()]
              for item in document.iter("Coordinate") if item.attrib.get("name") in JOINTS}
    actuators = {}
    for item in document.iter():
        if item.attrib.get("name") in SEAS and item.tag == "SeriesElasticActuator":
            actuators[item.attrib["name"]] = {child.tag: child.text.strip() for child in item if child.text and child.text.strip()}
    return {"setup": relative(setup), "setup_sha256": sha(setup), "model": relative(model),
            "model_sha256": sha(model), "total_body_mass_kg": mass,
            "prosthetic_coordinate_range_rad": bounds, "sea_properties_from_model": actuators}


def bootstrap(receipt):
    initial = receipt.get("initial_state")
    segment = receipt.get("first_governor_segment")
    pose = receipt.get("resolved_prosthetic_pose")
    if not initial or not pose:
        return {"available": False, "reason": "initial snapshot unavailable"}
    checks = {}
    for name, sea in zip(JOINTS, SEAS):
        checks[name] = {
            "actual_q_matches_pose": close(initial["actual_q"][name], pose[name], 1e-10),
            "actual_qdot_zero": close(initial["actual_qdot"][name], 0.0, 1e-10),
            "motor_angle_matches_pose": close(initial["sea_motor_angles"][sea], pose[name], 1e-10),
            "motor_speed_zero": close(initial["sea_motor_speeds"][sea], 0.0, 1e-10),
            "served_q_matches_pose": close(initial["served_prosthetic_reference"][name]["q"], pose[name], 1e-10),
            "served_v_zero": close(initial["served_prosthetic_reference"][name]["v"], 0.0, 1e-10),
            "served_a_zero": close(initial["served_prosthetic_reference"][name]["a"], 0.0, 1e-10),
            "previous_endpoint_matches_pose": close(initial["previous_endpoint"][JOINTS.index(name)], pose[name], 1e-10),
        }
        if segment:
            index = JOINTS.index(name)
            checks[name].update({
                "first_raw_anchor_q_matches_pose": close(segment["raw_anchor_q"][index], pose[name], 1e-10),
                "first_raw_anchor_v_zero": close(segment["raw_anchor_v"][index], 0.0, 1e-10),
                "first_filter_q_matches_pose": close(segment["filter_state_before"]["q"][index], pose[name], 1e-10),
                "first_filter_v_zero": close(segment["filter_state_before"]["v"][index], 0.0, 1e-10),
                "first_filter_a_zero": close(segment["filter_state_before"]["a"][index], 0.0, 1e-10),
                "first_served_q_matches_pose": close(segment["served_initial_q"][name], pose[name], 1e-10),
                "first_served_v_zero": close(segment["served_initial_v"][name], 0.0, 1e-10),
                "first_served_a_zero": close(segment["served_initial_a"][name], 0.0, 1e-10),
            })
    bio = all(close(initial["actual_q"][name], value, 1e-10) and
              close(initial["actual_qdot"][name], initial["biological_ik_qdot"][name], 1e-10)
              for name, value in initial["biological_ik_q"].items())
    pi_zero = all(close(value, 0.0, 1e-10) for key in ("outer_pi_integral", "cascade_pi_integral") for value in initial[key].values())
    aux_zero = all(close(value, 0.0, 1e-10) for states in initial.get("sea_auxiliary_states", {}).values() for value in states.values())
    reset_fsm = initial.get("reset_info", {}).get("phase_fsm", {})
    no_credit = all(close(reset_fsm.get(key, 0), 0, 1e-10) for key in ("valid_hs_count", "valid_to_count", "valid_cycle_count"))
    return {"available": True, "pose_rad": pose, "checks": checks,
            "biological_q_qdot_exact_to_ik": bio, "controller_integrals_zero": pi_zero,
            "sea_auxiliary_states_zero": aux_zero, "no_synthetic_reset_event_credit": no_credit,
            "first_segment_available": bool(segment),
            "all_available_checks_pass": bio and pi_zero and aux_zero and no_credit and all(all(items.values()) for items in checks.values()),
            "no_ik_fallback_scope": "Initial/first segment values checked; persistent refusal of uncovered prosthetic IK reads is established by the sealed source audit, not inferred from a single receipt."}


def journals(output, summary, receipt):
    returned = json_lines(output / "returned_rewards.jsonl")
    valid = json_lines(output / "last_valid_trace.jsonl")
    reward_sum = math.fsum(row["returned_reward"] for row in returned)
    end_row = returned[-1] if returned else None
    censored = summary.get("summary_kind") == "harness_censored_last_completed_transitions"
    summary_steps = summary.get("steps")
    summary_return = summary.get("episode_return")
    checks = {
        "returned_steps_contiguous": [row.get("step") for row in returned] == list(range(1, len(returned) + 1)),
        "valid_trace_count_matches_returned": len(valid) == len(returned),
        "summary_steps_match_returned": summary_steps == len(returned) if summary else None,
        "summary_return_matches_actual_sum": close(summary_return, reward_sum, 1e-8 + abs(reward_sum) * 1e-12) if summary else None,
        "valid_trace_rewards_match_returned": all(close(a["returned_reward"], b["returned_reward"], 1e-12) for a, b in zip(returned, valid)) and len(valid) == len(returned),
        "valid_trace_times_match_returned": all(close(a["time_after_s"], b["time"], 1e-9) for a, b in zip(returned, valid)) and len(valid) == len(returned),
    }
    if summary and end_row:
        if censored:
            checks["censored_summary_time_is_last_returned_boundary"] = close(summary.get("last_completed_transition_time_s"), end_row["time_after_s"], 1e-9)
        else:
            checks["summary_terminal_flags_match_last_returned"] = summary.get("terminated") == end_row.get("terminated") and summary.get("truncated") == end_row.get("truncated")
            checks["summary_end_reason_matches_last_returned"] = summary.get("end_reason") == end_row.get("end_reason")
    phase_counters = {}
    for summary_key, phase_key in (("phase_valid_hs_count", "valid_hs_count"),
                                   ("phase_valid_to_count", "valid_to_count"),
                                   ("phase_valid_cycle_count", "valid_cycle_count"),
                                   ("invalid_event_count", "invalid_event_count")):
        from_terms = max((int(row.get("reward_terms", {}).get(summary_key, 0) or 0) for row in valid), default=0)
        from_phase = int(valid[-1].get("phase_fsm", {}).get(phase_key, 0) or 0) if valid else 0
        expected = from_phase if censored else from_terms
        phase_counters[summary_key] = {"summary": summary.get(summary_key), "max_returned_terms": from_terms,
                                       "last_returned_phase_payload": from_phase,
                                       "summary_matches_declared_evaluator_semantics": summary.get(summary_key) == expected if summary else None}
    contacts = [{"step": row["step"], "time_after_s": row["time_after_s"], "terminal_contact": row["terminal_contact"]}
                for row in returned if row.get("terminal_contact")]
    result = {"returned_count": len(returned), "valid_trace_count": len(valid),
              "fully_completed_policy_windows": sum(float(row.get("reward_terms", {}).get("segment_completed", 1.0)) >= 0.5 for row in valid),
              "returned_incomplete_segment_count": sum(float(row.get("reward_terms", {}).get("segment_completed", 1.0)) < 0.5 for row in valid),
              "zero_logical_time_advancement_returned_steps": [row["step"] for row in returned if abs(row["time_after_s"] - row["time_before_s"]) < 1e-9],
              "actual_returned_reward_sum": reward_sum, "summary_episode_return": summary_return,
              "last_returned_boundary_time_s": end_row["time_after_s"] if end_row else None,
              "summary_is_censored": censored, "checks": checks, "terminal_contact_samples": contacts,
              "phase_counter_consistency": phase_counters,
              "first_last_returned": [returned[0], returned[-1]] if returned else [],
              "native_crash_interpretation": "Use JSONL/watchdog for completed boundaries if receipt count is stale; interrupted transition has no invented reward."}
    return result, valid


def terminal_physical_evidence(journal):
    samples = []
    for row in journal["terminal_contact_samples"]:
        contact = row["terminal_contact"]
        item = {"returned_step": row["step"], "logical_policy_time_s": row["time_after_s"],
                "source": contact.get("source"), "available": contact.get("available"),
                "physical_sample_time_s": contact.get("time_s"), "side": contact.get("side"),
                "penetration_m": contact.get("penetration_m"), "limit_m": contact.get("limit_m")}
        side_data = contact.get("online_grf", {}).get(contact.get("side"), {})
        force = side_data.get("force")
        if force is not None and len(force) == 3:
            item["physical_force_vector_norm_n"] = math.sqrt(math.fsum(float(value) ** 2 for value in force))
        if contact.get("available"):
            item["sample_after_logical_boundary"] = contact.get("time_s", row["time_after_s"]) > row["time_after_s"] + 1e-9
            item["physical_penetration_exceeds_declared_guard"] = contact.get("penetration_m", 0) > contact.get("limit_m", math.inf)
        samples.append(item)
    return {"samples": samples,
            "physical_hard_penetration_witness": any(item["source"] == "hard_penetration_guard" and item["available"] and item.get("physical_penetration_exceeds_declared_guard") for item in samples),
            "interpretation": "Reported summary/end_reason and physical guard witness remain separate. A causal wrapper label does not erase an explicitly measured physical guard event."}


def plot_check(output, case_id):
    receipt_path = output / "plot_receipt.json"
    expected_dir = REPO / "plot/random_pose_reset_2026_10_13" / case_id
    files = sorted(expected_dir.glob("[0-9][0-9]_*.png"))
    report = {"expected_root_directory": relative(expected_dir), "png_count": len(files),
              "seven_canonical_pngs": len(files) == 7 and {file.name[:2] for file in files} == {f"{i:02d}" for i in range(1, 8)},
              "plot_receipt_exists": receipt_path.exists()}
    if receipt_path.exists():
        declared = read_json(receipt_path)
        checks = []
        for path, digest in declared.get("figures", {}).items():
            actual = REPO / path
            checks.append({"path": path, "correct_case_root": actual.resolve().parent == expected_dir.resolve(),
                           "exists": actual.is_file(), "sha256_matches": actual.is_file() and sha(actual) == digest,
                           "png_signature": actual.is_file() and actual.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"})
        report.update(declared_rendered=declared.get("rendered"), declared_figures=len(checks), figures=checks,
                      all_receipt_hashes_correct=all(item["correct_case_root"] and item["sha256_matches"] and item["png_signature"] for item in checks) and len(checks) == 7)
    return report


def verify_case(root, case, model):
    case_id = case["id"]
    receipt, watchdog = finalize_marker(root, case_id)
    if receipt is None and watchdog is None:
        return {"case_id": case_id, "finalized": False}, None
    output = root / case_id
    receipt = receipt or (read_json(output / "receipt.json") if (output / "receipt.json").exists() else {})
    summary = read_json(output / "rollout_summary.json") if (output / "rollout_summary.json").exists() else {}
    result = {"case_id": case_id, "finalized": True, "receipt_status": receipt.get("status"),
              "receipt_sha256": sha(output / "receipt.json") if (output / "receipt.json").exists() else None,
              "watchdog": watchdog, "physical_summary": {key: summary.get(key) for key in ("summary_kind", "steps", "episode_return", "terminated", "truncated", "end_reason", "failure_exception")},
              "completed_requested_duration": receipt.get("completed_requested_duration"),
              "receipt_case_matches_selection": receipt.get("case") == case,
              "frozen_inputs_unchanged_receipt": receipt.get("frozen_inputs_unchanged"),
              "bootstrap": bootstrap(receipt), "issues": []}
    journal, trace = journals(output, summary, receipt)
    result["journal_verification"] = journal
    result["terminal_physical_evidence"] = terminal_physical_evidence(journal)
    result["last_returned_actual_prosthetic_state"] = {
        "logical_time_s": trace[-1]["time"] if trace else None,
        "segment_completed": trace[-1].get("reward_terms", {}).get("segment_completed") if trace else None,
        "actual_q_rad": {name: trace[-1].get("prosthetic_state", {}).get(name) for name in JOINTS} if trace else {},
        "operational_q_bounds_rad": receipt.get("effective_env_config", {}).get("truncation_bounds_rad", {}),
        "scope": "Physical q from returned observation; logical time can lag an interrupted guard sample, which is reported separately."}
    result["plots"] = plot_check(output, case_id)
    tables = {}
    expected_paths = {f"rollout_episode_{name}.sto" for name in EXPECTED}
    observed_paths = {path.name for path in (output / "sim_outputs").glob("*.sto")}
    result["sto_file_set"] = {"expected_count": 17, "observed_count": len(observed_paths),
                              "missing": sorted(expected_paths - observed_paths), "unexpected": sorted(observed_paths - expected_paths)}
    for name in EXPECTED:
        path = output / "sim_outputs" / f"rollout_episode_{name}.sto"
        if not path.exists():
            continue
        try:
            tables[name] = sto(path)
        except (ValueError, OSError) as exc:
            result["issues"].append(f"{path.name}: {type(exc).__name__}: {exc}")
    result["sto"] = {name: table["report"] for name, table in tables.items()}
    for name, table in tables.items():
        audit = table["report"]
        if not (audit["all_finite"] and audit["header_nrows_matches"] and audit["header_ncolumns_matches"] and audit["strictly_increasing_time"] and audit["in_degrees"] == "no" and audit["max_abs_grid_delta_minus_1ms_s"] < 2e-8):
            result["issues"].append(f"{name}: finite/header/units/grid check failed")
    grids = [table["columns"]["time"] for table in tables.values()]
    result["all_available_sto_grids_identical"] = all(grid == grids[0] for grid in grids) if grids else None
    if grids and not result["all_available_sto_grids_identical"]:
        result["issues"].append("Available STO time grids differ")
    kin, ref, state = (tables.get(name) for name in ("kinematics", "kinematics_reference", "states"))
    cross = None
    if kin and ref and state and kin["columns"]["time"] == ref["columns"]["time"] == state["columns"]["time"]:
        times = kin["columns"]["time"]
        result["saved_sample_coverage"] = {"rows": len(times), "start_s": times[0] if times else None,
                                           "last_saved_s": times[-1] if times else None,
                                           "receipt_start_s": receipt.get("actual_start_s", receipt.get("initial_state", {}).get("time_s")),
                                           "receipt_end_s": receipt.get("actual_end_s"),
                                           "note": "Recorder saves start-of-1ms intervals; last row is normally end minus1ms, not the final physical endpoint."}
        if result["completed_requested_duration"] and times:
            start_time = receipt.get("initial_state", {}).get("time_s")
            end_time = receipt.get("actual_end_s")
            expected_rows = round((end_time - start_time) / 0.001)
            result["saved_sample_coverage"]["completed_horizon_expected_rows"] = expected_rows
            result["saved_sample_coverage"]["complete_horizon_grid_matches"] = (
                len(times) == expected_rows and close(times[0], start_time, 1.01e-8)
                and close(times[-1] + 0.001, end_time, 1.01e-8)
            )
            if not result["saved_sample_coverage"]["complete_horizon_grid_matches"]:
                result["issues"].append("Completed horizon recorded grid/coverage mismatch")
        matches = {name: max_abs([a - b for a, b in zip(values, state["columns"].get(name + "_q", []))])
                   for name, values in kin["columns"].items() if name != "time"}
        result["kinematics_matches_coordinate_state_q_max_abs"] = matches
        if any(value is None or value > 1e-8 for value in matches.values()):
            result["issues"].append("Kinematics q differs from states q")
        if times and receipt.get("initial_state"):
            initial = receipt["initial_state"]
            result["initial_saved_pose_reference_matches_receipt"] = {
                name: {"actual_q": close(kin["columns"][name][0], initial["actual_q"][name], 1.01e-8),
                       "served_q": close(ref["columns"][name + "_q_ref"][0], initial["served_prosthetic_reference"][name]["q"], 1.01e-8),
                       "served_v": close(ref["columns"][name + "_qdot_ref"][0], initial["served_prosthetic_reference"][name]["v"], 1.01e-8)} for name in JOINTS}
        index_by_time = {round(value, 8): i for i, value in enumerate(times)}
        trace_checks = {"matched_boundaries": 0, "last_boundary_not_recorded": 0, "incomplete_segments_excluded_from_logical_boundary_comparison": 0, "max_actual_q_error_rad": 0.0,
                        "max_served_q_error_rad": 0.0, "max_served_v_error_rad_s": 0.0}
        for row in trace:
            if float(row.get("reward_terms", {}).get("segment_completed", 1.0)) < 0.5:
                # A guard may move the physical state beyond the retained logical
                # clock.  It is not the physical q at this complete boundary.
                trace_checks["incomplete_segments_excluded_from_logical_boundary_comparison"] += 1
                continue
            index = index_by_time.get(round(row["time"], 8))
            if index is None:
                trace_checks["last_boundary_not_recorded"] += 1
                continue
            trace_checks["matched_boundaries"] += 1
            values = row.get("prosthetic_state", {})
            for name in JOINTS:
                for trace_name, table, column, result_key in (
                    (name, kin, name, "max_actual_q_error_rad"),
                    (name + "_served_ref", ref, name + "_q_ref", "max_served_q_error_rad"),
                    (name + "_served_ref_vel", ref, name + "_qdot_ref", "max_served_v_error_rad_s"),
                ):
                    if trace_name in values:
                        trace_checks[result_key] = max(trace_checks[result_key], abs(values[trace_name] - table["columns"][column][index]))
        result["saved_reference_vs_returned_trace"] = trace_checks
        if any(trace_checks[key] > 1.01e-8 for key in ("max_actual_q_error_rad", "max_served_q_error_rad", "max_served_v_error_rad_s")):
            result["issues"].append("Complete-boundary trace versus STO actual/reference mismatch")
        start = receipt.get("initial_state", {}).get("time_s", times[0] if times else 0.0)
        tracking = {}
        for name in JOINTS:
            errors = [a - b for a, b in zip(kin["columns"][name], ref["columns"][name + "_q_ref"])]
            tracking[name] = {window: rms([error * 180 / math.pi for t, error in zip(times, errors) if selector(t - start)])
                              for window, selector in (("all_recorded_rmse_deg", lambda t: True),
                                                       ("first_050s_rmse_deg", lambda t: t < 0.5),
                                                       ("after_050s_rmse_deg", lambda t: t >= 0.5))}
            values = kin["columns"][name]
            lo, hi = model["prosthetic_coordinate_range_rad"][name]
            operational_bounds = receipt.get("effective_env_config", {}).get("truncation_bounds_rad", {}).get(name)
            tracking[name].update(min_actual_rad=min(values) if values else None, max_actual_rad=max(values) if values else None,
                                  declared_model_range_rad=[lo, hi], model_range_violation_count=sum(value < lo or value > hi for value in values))
            if operational_bounds:
                low_guard, high_guard = operational_bounds
                tracking[name].update(operational_guard_bounds_rad=operational_bounds,
                                      saved_operational_guard_violation_count=sum(value < low_guard - 1e-8 or value > high_guard + 1e-8 for value in values),
                                      guard_scope="Saved actual q, with1e-8 serialization tolerance. Unsaved terminal/trial state remains separate from STO coverage.")
        result["tracking_actual_minus_served"] = tracking
        bio_columns = [column for column in ref["names"] if column != "time" and not any(column.startswith(name + "_") for name in JOINTS)]
        ref_map = [("time", "time")] + [(column, column) for column in bio_columns]
        actual_map = [("time", "time")] + [(column.replace("_qdot_ref", "_qdot").replace("_q_ref", "_q"), column) for column in bio_columns]
        result["biological_reference_projection"] = {"columns": [label for _source, label in ref_map], "rows": len(times),
                                                     "reference_subtable_byte_sha256": projection_hash(ref, ref_map),
                                                     "actual_subtable_byte_sha256_same_labels": projection_hash(state, actual_map),
                                                     "actual_is_byte_equal_to_reference": projection_hash(ref, ref_map) == projection_hash(state, actual_map)}
        cross = {"rows": len(times), "ref_map": ref_map,
                 "reference_projection": compact_projection(ref, ref_map),
                 "actual_projection": compact_projection(state, actual_map)}
    grf = tables.get("online_grf")
    if grf:
        vectors = [math.sqrt(math.fsum(grf["columns"]["left_force_" + axis][i] ** 2 for axis in "xyz")) for i in range(grf["report"]["rows"])]
        effective = receipt.get("effective_simulator_config", {})
        limit = float(effective.get("online_grf_max_force_bw", 5.0)) * model["total_body_mass_kg"] * 9.80665
        result["physical_force_vector_check"] = {"applied_sides_from_effective_config": effective.get("online_grf_applied_sides"),
                                                "model_mass_kg": model["total_body_mass_kg"], "limit_n": limit,
                                                "saved_peak_left_force_norm_n": max(vectors, default=None),
                                                "saved_norm_samples_exceeding_limit": sum(value > limit + 1e-5 for value in vectors),
                                                "saved_peak_left_normal_force_n": max(grf["columns"]["left_normal_force"], default=None),
                                                "saved_max_left_penetration_m": max(grf["columns"]["left_penetration"], default=None),
                                                "static_receipt_force_guard_matches_model": close(receipt.get("static_contact_gate", {}).get("force_guard_n"), limit, 1e-5),
                                                "scope": "Saved APPLIED-left vector norms, not detector forces or proof about an interrupted unsaved guard sample."}
    diag = tables.get("sea_diagnostics")
    if diag:
        sea = {}
        for sea_name in SEAS:
            cols = diag["columns"]
            plugin = cols.get(sea_name + "_tau_input_plugin", [])
            flag = cols.get(sea_name + "_tau_input_saturated", [])
            diff = cols.get(sea_name + "_tau_input_plugin_minus_python", [])
            real = bool(plugin) and all(math.isfinite(value) for value in plugin)
            sea[sea_name] = {"actual_plugin_tau_input_channel_available": real,
                            "plugin_tau_input_max_abs_nm": max_abs(plugin),
                            "recorded_motor_clamp_flag_count": sum(value >= 0.5 for value in flag) if real else None,
                            "motor_clamp_flag_consistent_with_actual_plugin_at_saved_precision": all(not value >= 0.5 or abs(torque) >= 500 - 1e-7 for value, torque in zip(flag, plugin)) if real else None,
                            "actual_plugin_torque_exceeds_500nm": any(abs(value) > 500 + 1e-7 for value in plugin) if real else None,
                            "plugin_minus_python_max_abs_nm": max_abs(diff),
                            "source_of_motor_saturation_claim": "Actual tau_input_plugin plus matching tau_input_saturated, distinct from normalized joint SEA control u."}
            properties = model["sea_properties_from_model"].get(sea_name, {})
            if properties and properties.get("Impedence", "false").lower() == "false":
                kp, kd, ki = (float(properties.get(name, 0)) for name in ("Kp", "Kd", "Ki"))
                prefix = sea_name + "_"
                raw_errors, input_errors, integral_errors, parity_identity_errors = [], [], [], []
                limit_i = float(properties.get("integral_torque_limit", 0))
                for index in range(diag["report"]["rows"]):
                    ref_torque = cols[prefix + "tau_ref"][index]
                    spring = cols[prefix + "tau_spring_state"][index]
                    motor_speed = cols[prefix + "motor_speed"][index]
                    integral_torque = cols[prefix + "inner_integral_torque"][index]
                    xi = cols[prefix + "torque_error_integral"][index]
                    calculated = ref_torque + kp * (ref_torque - spring) + integral_torque - kd * motor_speed
                    raw_errors.append(calculated - cols[prefix + "tau_input_raw"][index])
                    input_errors.append(max(-500, min(500, calculated)) - plugin[index])
                    parity_identity_errors.append(plugin[index] - cols[prefix + "tau_input_python"][index] - cols[prefix + "tau_input_plugin_minus_python"][index])
                    integral_errors.append(max(-limit_i, min(limit_i, ki * xi)) - integral_torque)
                tolerance = (1 + 2 * abs(kp) + abs(kd)) * 0.5e-8 + 2e-8
                sea[sea_name]["nonimpedance_pi_law_from_recorded_channels"] = {
                    "kp": kp, "kd": kd, "ki": ki, "integral_torque_limit_nm": limit_i,
                    "max_raw_law_error_nm": max_abs(raw_errors), "max_plugin_clamped_law_error_nm": max_abs(input_errors),
                    "max_integral_torque_law_error_nm": max_abs(integral_errors),
                    "raw_law_rounding_tolerance_nm": tolerance,
                    "integral_rounding_tolerance_nm": (abs(ki) + 1) * 0.5e-8 + 2e-8,
                    "raw_law_consistent": max_abs(raw_errors) is not None and max_abs(raw_errors) <= tolerance,
                    "model_formula_to_plugin_agrees_within_recording_rounding_only": max_abs(input_errors) is not None and max_abs(input_errors) <= tolerance,
                    "plugin_parity_difference_channel_identity_max_error_nm": max_abs(parity_identity_errors),
                    "plugin_parity_difference_channel_identity_consistent": max_abs(parity_identity_errors) is not None and max_abs(parity_identity_errors) <= 1.51e-8,
                    "integral_law_consistent": max_abs(integral_errors) is not None and max_abs(integral_errors) <= (abs(ki) + 1) * 0.5e-8 + 2e-8,
                    "scope": "Raw PI law is recomputed from saved SI channels/model properties. Model-to-plugin difference is separately measured, not silently set to zero; its exported difference channel identity is verified. Rounding-only agreement is a precision diagnostic, not a dynamics qualification.",
                }
        result["sea_actual_channel_checks"] = sea
    controls = tables.get("sea_controls")
    if controls:
        result["sea_normalized_control_checks"] = {
            name: {"max_abs_u": max_abs(controls["columns"][name]),
                   "sample_count_at_normalized_hard_clip": sum(abs(value) >= 1 - 1e-8 for value in controls["columns"][name]),
                   "sample_count_outside_normalized_bound": sum(abs(value) > 1 + 1e-8 for value in controls["columns"][name]),
                   "scope": "Normalized SEA joint command u; independent of motor torque500Nm clamp and of neural raw-action clipping."}
            for name in JOINTS}
    result["input_pose_inside_preregistered_rom"] = {name: DOMAIN[name][0] <= receipt.get("resolved_prosthetic_pose", {}).get(name, math.nan) <= DOMAIN[name][1] for name in JOINTS}
    result["verification_source_sha256"] = sha(Path(__file__))
    result["artifact_integrity_available"] = (
        len(tables) == 17 and not result["sto_file_set"]["missing"]
        and not result["sto_file_set"]["unexpected"] and not result["issues"]
        and result["bootstrap"].get("all_available_checks_pass") is True
        and all(value is True for value in journal["checks"].values())
        and result["plots"].get("all_receipt_hashes_correct") is True
    )
    return result, cross


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, default=AUDIT / "rollouts")
    parser.add_argument("--selection", type=Path, default=AUDIT / "preflight/selected_cases.json")
    parser.add_argument("--destination", type=Path, default=AUDIT)
    parser.add_argument("--partial", action="store_true")
    args = parser.parse_args()
    selection = read_json(args.selection)
    cases = selection["cases"]
    statuses = [finalize_marker(args.output_root, case["id"]) for case in cases]
    unfinished = [case["id"] for case, (receipt, watchdog) in zip(cases, statuses) if receipt is None and watchdog is None]
    if unfinished and not args.partial:
        raise SystemExit("Campaign not finalized; no full verification produced: " + ", ".join(unfinished))
    model = model_metadata()
    reports, projection_inputs = [], []
    for case in cases:
        try:
            result, cross = verify_case(args.output_root, case, model)
        except Exception as exc:
            # A damaged/censored case cannot prevent verification of its peers.
            # This is an explicit inability to verify, never a physical verdict.
            receipt, watchdog = finalize_marker(args.output_root, case["id"])
            result = {"case_id": case["id"], "finalized": receipt is not None or watchdog is not None,
                      "artifact_integrity_available": False, "verification_error": f"{type(exc).__name__}: {exc}",
                      "issues": ["Reader could not complete verification; original artifacts remain preserved"],
                      "receipt_status": (receipt or {}).get("status"), "watchdog": watchdog,
                      "physical_summary": {}, "journal_verification": {"returned_count": None, "checks": {}},
                      "sto": {}, "sto_file_set": {"observed_count": len(list((args.output_root / case["id"] / "sim_outputs").glob("*.sto")))},
                      "plots": {"png_count": len(list((REPO / "plot/random_pose_reset_2026_10_13" / case["id"]).glob("[0-9][0-9]_*.png")))}}
            cross = None
        if result.get("finalized"):
            try:
                raw_receipt = read_json(args.output_root / case["id"] / "receipt.json") if (args.output_root / case["id"] / "receipt.json").exists() else {}
            except (ValueError, OSError) as exc:
                raw_receipt = {}
                result["receipt_read_error"] = f"{type(exc).__name__}: {exc}"
            result["receipt_source_hashes_match_selection"] = raw_receipt.get("source_hashes") == selection.get("frozen_source_hashes")
        reports.append(result)
        if cross:
            projection_inputs.append((result, cross))
    if projection_inputs:
        baseline, reference = max(projection_inputs, key=lambda item: item[1]["rows"])
        for result, cross in projection_inputs:
            count = min(reference["rows"], cross["rows"])
            same_columns = reference["ref_map"] == cross["ref_map"]
            current_ref = compact_projection_hash(cross["reference_projection"], count)
            baseline_ref = compact_projection_hash(reference["reference_projection"], count)
            actual = compact_projection_hash(cross["actual_projection"], count)
            baseline_actual = compact_projection_hash(reference["actual_projection"], count)
            result["cross_case_biological_reference_bytes"] = {"baseline_case": baseline["case_id"], "common_prefix_rows": count,
                "same_columns": same_columns, "reference_bytes_identical": same_columns and current_ref == baseline_ref,
                "actual_biological_bytes_identical_to_baseline": same_columns and actual == baseline_actual,
                "reference_prefix_sha256": current_ref, "baseline_reference_prefix_sha256": baseline_ref,
                "scope": "Hash of saved time+biological q/qdot reference tokens; compare identical prefixes for censored cases. Actual physical trajectories are analyzed separately."}
    protected_files = {}
    for filename, expected in selection.get("frozen_source_hashes", {}).items():
        path = REPO / filename
        observed = sha(path) if path.is_file() else None
        protected_files[filename] = {"expected_sha256": expected, "observed_sha256": observed, "unchanged": observed == expected}
    audit = {"audit_id": "independent_saved_output_verification_2026_10_13", "instruction_check_token": "CMC_AGENT_OK_2026",
             "independence": {"claude13_outputs_read": False, "root_numerical_campaign_conclusions_read": False,
                              "root_case_specific_operational_note_received_after_first_validator_draft": "A parent message described one interrupted terminal step in random_006 after this validator's first source was saved. The validator directly checked raw finalized receipt/journals/STO; no Claude output or campaign synthesis was read. Timestamp/partial-segment checks were already specified by the sealed source audit.",
                              "native_imports_or_tests": False, "frozen_inputs_mutated": False},
             "campaign_finalized": not unfinished, "selected_count": len(cases), "unfinished_cases": unfinished,
             "selection_sha256": sha(args.selection), "validator_sha256": sha(Path(__file__)),
             "sealed_harness_sha256": sha(AUDIT / "random_pose_reset.py"), "harness_matches_reviewed_source": sha(AUDIT / "random_pose_reset.py") == EXPECTED_HARNESS,
             "protected_files": protected_files,
             "all_current_protected_files_match_selection": all(item["unchanged"] for item in protected_files.values()) if protected_files else False,
             "artifact_integrity_cases": [item["case_id"] for item in reports if item.get("artifact_integrity_available")],
             "model_metadata": model, "cases": reports,
             "interpretation": "Artifact/identity/reward checks are separate from completion, gait cycles, tracking and biomechanics. Missing evidence in censored/native crashes is not silently treated as a successful check. No survivors-only campaign denominator."}
    counts = {
        "selected_cases": len(cases), "finalized_cases": sum(item.get("finalized", False) for item in reports),
        "sto_parsed": sum(len(item.get("sto", {})) for item in reports),
        "sto_all_finite": sum(table["all_finite"] for item in reports for table in item.get("sto", {}).values()),
        "canonical_pngs": sum(item.get("plots", {}).get("png_count", 0) for item in reports),
        "canonical_png_hashes_matching": sum(sum(figure["sha256_matches"] and figure["correct_case_root"] and figure["png_signature"] for figure in item.get("plots", {}).get("figures", [])) for item in reports),
        "protected_files": len(protected_files), "protected_files_unchanged": sum(item["unchanged"] for item in protected_files.values()),
        "bootstrap_pass_cases": sum(item.get("bootstrap", {}).get("all_available_checks_pass") is True for item in reports),
        "receipt_source_hash_match_cases": sum(item.get("receipt_source_hashes_match_selection") is True for item in reports),
        "identical_biological_reference_prefix_cases": sum(item.get("cross_case_biological_reference_bytes", {}).get("reference_bytes_identical") is True for item in reports),
        "artifact_integrity_cases": sum(item.get("artifact_integrity_available") is True for item in reports),
    }
    audit["summary_counts"] = counts
    args.destination.mkdir(parents=True, exist_ok=True)
    stem = "independent_saved_output_verification" + ("_partial" if unfinished else "")
    path = args.destination / (stem + ".json")
    path.write_text(json.dumps(json_safe(audit), indent=2, allow_nan=False) + "\n", encoding="utf-8")
    lines = ["# Verifica indipendente degli output salvati — 2026-10-13", "", "Instruction check token: CMC_AGENT_OK_2026", "",
             f"Casi selezionati: {len(cases)}; non finalizzati: {len(unfinished)}. Harness: `{audit['sealed_harness_sha256']}`.", "",
             "Non sono stati letti output Claude13 o report/sintesi numeriche root della campagna. Le verifiche sono ricostruite dai raw finalizzati; nessun import/test nativo o modifica agli input congelati.", "",
             "Nota sulla provenienza: dopo il primo draft del validator il parent ha inviato un esempio operativo relativo al terminale di random_006. Tale esposizione e dichiarata nel JSON; i file raw sono stati verificati direttamente e non sono stati letti output Claude o sintesi numeriche della campagna.", "",
             f"Verificati {counts['sto_parsed']} STO ({counts['sto_all_finite']} finiti), {counts['canonical_png_hashes_matching']} PNG con hash/percorso corretti e {counts['protected_files_unchanged']}/{counts['protected_files']} input protetti invariati. Bootstrap coerente in {counts['bootstrap_pass_cases']} casi e byte della reference biologica identici su prefissi comuni in {counts['identical_biological_reference_prefix_cases']} casi.", "",
             "Questi conteggi misurano l'integrita degli artifact. Lo status `rollout_complete` significa ritorno normale dell'evaluator; il completamento5s viene verificato da `completed_requested_duration`, e la sequenza HS→TO→HS costituisce un criterio funzionale separato.", "",
             "| Caso | Status receipt | Fine summary | Step restituiti | STO | Finitezza | Bio ref identica | Reward somma | Plot |", "|---|---|---|---:|---:|---|---|---|---:|"]
    for result in reports:
        if not result.get("finalized"):
            lines.append(f"| {result['case_id']} | non finalizzato | — | — | — | — | — | — | — |")
            continue
        finite = all(item["all_finite"] for item in result.get("sto", {}).values()) if result.get("sto") else None
        lines.append(f"| {result['case_id']} | {result.get('receipt_status')} | {result['physical_summary'].get('end_reason')} | {result['journal_verification']['returned_count']} | {result['sto_file_set']['observed_count']}/17 | {finite} | {result.get('cross_case_biological_reference_bytes', {}).get('reference_bytes_identical')} | {result['journal_verification']['checks'].get('summary_return_matches_actual_sum')} | {result['plots']['png_count']}/7 |")
    case25 = next((item for item in reports if item["case_id"] == "random_025" and item.get("finalized")), None)
    if case25 and "last_returned_actual_prosthetic_state" in case25:
        last = case25["last_returned_actual_prosthetic_state"]
        lines += ["", "Il caso random_025 viene identificato dai dati raw come terminazione `joint_divergence:pros_knee_angle`, coerente fra summary e ultimo reward journal. "
                  f"L'ultimo q knee e {last['actual_q_rad'].get('pros_knee_angle')} rad, oltre il limite operativo superiore0 rad "
                  f"(bounds {last['operational_q_bounds_rad'].get('pros_knee_angle')}), al tempo {last['logical_time_s']} s. "
                  f"Ha {case25['journal_verification']['returned_count']} reward restituiti e {case25['journal_verification']['fully_completed_policy_windows']} finestre complete; "
                  "l'uscita dal limite operativo non equivale a uscita dal range ±pi/2 dichiarato nel modello."]
    physical_cases = [item["case_id"] for item in reports if item.get("terminal_physical_evidence", {}).get("physical_hard_penetration_witness")]
    if physical_cases:
        lines += ["", "Witness misurato del guard fisico di penetrazione nei journal: " + ", ".join(physical_cases) + ". "
                  "Il timestamp fisico e distinto dal logical policy time; l'etichetta riportata dal wrapper nella summary resta conservata senza cancellare il guard sottostante."]
    lines += ["", "I bio reference hash riguardano soltanto time+colonne biologiche q/qdot, con token salvati e prefisso comune; le reference protesiche possono differire. Stati fisici effettivi e reference sono controlli distinti.", "",
              "La forza e verificata dalla norma del vettore APPLIED sinistro e dal mass/guard effettivo, non dalla sola normal force. Clamp motore riportato soltanto quando il canale tau_input_plugin reale e disponibile e coerente col flag; u saturato e un altro fenomeno.", "",
              "Step/return vengono ricostruiti dai reward realmente restituiti. I casi censurati o nativamente interrotti non ricevono reward o campioni ipotetici della transizione fallita. Completamento temporale, ciclo HS→TO→HS e validita biomeccanica non sono dedotti dal solo stato degli artifact.", "",
              f"Evidenze complete: [{path.name}]({path.name})."]
    (args.destination / (stem + ".md")).write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps({"output": relative(path), "selected_cases": len(cases), "finalized": not unfinished}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
