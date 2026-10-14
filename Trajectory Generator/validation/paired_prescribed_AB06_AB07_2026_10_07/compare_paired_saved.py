"""Analyze finalized paired prescribed runs using frozen nominal cycles only.

Read-only native inputs; imports no OpenSim/Ray/env/policy/detector runtime.
The primary three nominal cycles are never replaced after an interrupted run.
"""
from __future__ import annotations

import argparse
import ast
import csv
import hashlib
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

import numpy as np

REPO = Path(__file__).resolve().parents[3]
AUDIT = Path(__file__).resolve().parent
ANALYZER = REPO / "Trajectory Generator/validation/AB07/analyze_prescribed_v26.py"
MATCHER = REPO / "validation/validate_online_grf_events.py"
CROSSING = REPO / "validation/validate_online_grf.py"
PHASE_HELPER = REPO / "Trajectory Generator/validation/AB07/v26c_best47_transfer_2026_10_06/compare_transfer.py"
TOLERANCE_RECEIPT = REPO / "results/ab07_v26_ground_registered_prescribed_9p01s_2026_10_06_r1/qualification_analysis.json"
JOINTS = ("pros_knee_angle", "pros_ankle_angle")


def resolve(path):
    value = Path(path)
    return value if value.is_absolute() else REPO / value


def fingerprint(path):
    path = Path(path)
    try:
        label = str(path.relative_to(REPO))
    except ValueError:
        label = str(path)
    return {"path": label, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def load(path):
    def reject(value):
        raise ValueError(f"Nonfinite JSON source constant {value}: {path}")
    return json.loads(Path(path).read_text(encoding="utf-8"), parse_constant=reject)


def jsonl(path):
    def reject(value):
        raise ValueError(f"Nonfinite JSONL source constant {value}: {path}")
    return [json.loads(line, parse_constant=reject) for line in Path(path).read_text(encoding="utf-8").splitlines() if line.strip()]


def metrics(values):
    array = np.asarray(list(values), dtype=float)
    if not len(array):
        return None
    if not np.all(np.isfinite(array)):
        raise ValueError("Nonfinite saved metric values")
    return {"count": len(array), "min": float(np.min(array)), "max": float(np.max(array)), "mean": float(np.mean(array)), "rms": float(np.sqrt(np.mean(array ** 2)))}


def pure(path, name):
    tree = ast.parse(Path(path).read_text(encoding="utf-8"))
    function = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == name)
    namespace = {"np": np, "Path": Path, "Any": Any, "Iterable": Iterable, "csv": csv, "metrics": metrics}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[])), str(path), "exec"), namespace)
    return namespace[name]


def strict_statistics(value):
    if isinstance(value, dict):
        empty = value.get("matched_count") == 0
        return {key: None if empty and key in {"timing_bias_s", "timing_mae_s", "timing_max_abs_s"} else strict_statistics(item) for key, item in value.items()}
    if isinstance(value, list):
        return [strict_statistics(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError("Nonfinite analysis outside empty event-matching statistics")
    return value


def require_grid(times, label):
    if len(times) < 2 or not np.all(np.isfinite(times)) or np.any(np.diff(times) <= 0):
        raise ValueError(f"Invalid saved grid: {label}")


def validate_trace(rows, label):
    times = np.asarray([row["time"] for row in rows], dtype=float)
    if not np.all(np.isfinite(times)) or (len(times) > 1 and np.any(np.diff(times) < 0)):
        raise ValueError(f"Nonfinite or decreasing saved trace clocks: {label}")
    for index in range(1, len(times)):
        if times[index] == times[index - 1] and not (index == len(times) - 1 and (rows[index].get("segment_completed", True) in (False, 0.0) or rows[index].get("failure"))):
            raise ValueError(f"Duplicate completed trace clock: {label}/{index}")


def table(path, reader):
    times, labels, values, degrees = reader(path)
    require_grid(times, path)
    return {"time": times, "columns": {name: values[:, index] for index, name in enumerate(labels)}, "degrees": degrees}


def supported(interval, *tables):
    start, end = interval
    return all(float(item["time"][0]) <= start + 1e-8 and float(item["time"][-1]) >= end - 1e-8 for item in tables)


def interpolate(times, source, column):
    if not supported([float(times[0]), float(times[-1])], source):
        raise ValueError(f"Interpolation would extrapolate or clamp: {column}")
    return np.interp(times, source["time"], source["columns"][column])


def exact_interval_grid(source_times, start, end):
    inner = source_times[(source_times > start + 1e-10) & (source_times < end - 1e-10)]
    return np.concatenate(([start], inner, [end]))


def nominal(cycles):
    return {"heel_strike": sorted({cycle[key] for cycle in cycles for key in ("hs_s", "next_hs_s")}), "toe_off": [cycle["to_s"] for cycle in cycles]}


def compare_events(events, reference, window, matcher, tolerances, holdout):
    result = {}
    buffer_s = max(float(value) for value in holdout.values())
    for name, code in (("heel_strike", "HS"), ("toe_off", "TO")):
        all_events = [event for event in events if event.get("event") == name]
        selected = [event for event in all_events if window[0] - buffer_s <= float(event["event_time_s"]) <= window[1] + buffer_s]
        outside = [event for event in all_events if event not in selected]
        channels = {}
        for channel, field, tolerance in (("onset", "event_time_s", holdout[name]), ("confirmed", "confirmed_time_s", tolerances[f"{code}_TOLERANCE_S"]), ("delivered", "delivered_time_s", tolerances[f"DELIVERED_{code}_TOLERANCE_S"])):
            values = [float(event[field]) for event in selected]
            channels[channel] = {"observed_times_s": values, "tolerance_s": float(tolerance), "matching": matcher(reference[name], values, float(tolerance))}
        result[name] = {"reference_times_s": reference[name], "observed_candidate_window_s": [window[0] - buffer_s, window[1] + buffer_s], "candidate_window_note": "Inherited maximum onset reference tolerance is used only as a boundary buffer, never as a new acceptance gate.", "channels": channels, "outside_analysis_boundary_buffer_events": outside}
    return result


def physical_segment(cycle, force, measured, kinematics, raw_ik, weight_n, threshold):
    start, end = cycle["hs_s"], cycle["next_hs_s"]
    if not supported([start, end], force, measured, kinematics, raw_ik):
        return {"status": "not_fully_observed", "frozen_nominal_cycle": cycle, "reason": "Frozen boundaries are outside saved force/kinematics or measured-reference coverage; no replacement or extrapolation."}
    times = exact_interval_grid(force["time"], start, end)
    fy = interpolate(times, force, "left_force_y")
    ref = interpolate(times, measured, "left_vertical_force")
    pen = interpolate(times, force, "left_penetration")
    integrate = np.trapezoid if hasattr(np, "trapezoid") else np.trapz
    duration = end - start
    average = lambda values: float(integrate(values, times) / duration)
    impulse, reference_impulse = float(integrate(fy, times)), float(integrate(ref, times))
    error = fy - ref
    mean_y, mean_r = average(fy), average(ref)
    variance_y, variance_r = average((fy - mean_y) ** 2), average((ref - mean_r) ** 2)
    correlation = average((fy - mean_y) * (ref - mean_r)) / math.sqrt(variance_y * variance_r) if variance_y > 0 and variance_r > 0 else None
    raw_mask = (force["time"] >= start) & (force["time"] < end)
    raw_times = force["time"][raw_mask]
    truth = interpolate(raw_times, measured, "left_vertical_force") > threshold
    prediction = force["columns"]["left_force_y"][raw_mask] > threshold
    tp, fp, fn = [int(np.count_nonzero(mask)) for mask in (prediction & truth, prediction & ~truth, ~prediction & truth)]
    tracking = {}
    for joint in JOINTS:
        actual = interpolate(times, kinematics, joint) * (1 if kinematics["degrees"] else 180 / math.pi)
        reference = interpolate(times, raw_ik, joint) * (1 if raw_ik["degrees"] else 180 / math.pi)
        joint_error = actual - reference
        tracking[joint] = {"error_rms_deg": math.sqrt(average(joint_error ** 2)), "error_mean_deg": average(joint_error), "error_max_abs_deg": float(np.max(np.abs(joint_error))), "actual_excursion_deg": float(np.ptp(actual)), "raw_IK_excursion_deg": float(np.ptp(reference))}
    return {"status": "fully_observed", "frozen_nominal_cycle": cycle, "integration_grid": {"native_interval_samples": len(raw_times), "augmented_time_points": len(times), "timestamp_serialization_tolerance_s": 1e-8, "method": "Saved 1 ms force grid plus linear interpolation at exact frozen nominal boundaries. Up to 10 ns coincidence tolerance accounts for STO 8-decimal timestamp serialization; np.interp may use the coincident endpoint within that rounding interval only. No extrapolation beyond this declared source-rounding tolerance. RMS/correlation use time integrals to avoid phase-dependent sample-count weighting."}, "vertical_impulse_ns": impulse, "measured_vertical_impulse_ns": reference_impulse, "vertical_impulse_ratio": impulse / reference_impulse if reference_impulse else None, "vertical_impulse_percent_error": (impulse / reference_impulse - 1) * 100 if reference_impulse else None, "vertical_impulse_bw_s": impulse / weight_n, "measured_vertical_impulse_bw_s": reference_impulse / weight_n, "mean_vertical_force_bw": mean_y / weight_n, "measured_mean_vertical_force_bw": mean_r / weight_n, "peak_vertical_force_n": float(np.max(fy)), "peak_vertical_force_bw": float(np.max(fy)) / weight_n, "measured_peak_vertical_force_bw": float(np.max(ref)) / weight_n, "vertical_error_rms_n": math.sqrt(average(error ** 2)), "vertical_error_rms_bw": math.sqrt(average(error ** 2)) / weight_n, "vertical_correlation": correlation, "contact_20n_native_samples": {"threshold_n": threshold, "tp": tp, "fp": fp, "fn": fn, "f1": 2 * tp / (2 * tp + fp + fn) if 2 * tp + fp + fn else None}, "penetration_max_m": float(np.max(pen)), "raw_IK_tracking": tracking}


def analyze_subject(subject, directory, selection, reader, matcher, crossing, phase_helper, tolerances, holdout, sources, phase_rows):
    chosen = selection["subjects"][subject]
    receipt_path, trace_path = directory / "receipt.json", directory / "trace.jsonl"
    receipt = load(receipt_path)
    if receipt.get("status") not in {"completed", "interrupted", "failed"}:
        raise ValueError(f"Final process receipt required for {subject}; status={receipt.get('status')}")
    if receipt.get("selection", {}).get("sha256") != fingerprint(resolve(receipt["selection"]["path"]))["sha256"]:
        raise ValueError(f"Frozen selection receipt binding changed for {subject}")
    captured_selection = load(resolve(receipt["selection"]["path"]))
    if captured_selection != selection:
        raise ValueError(f"Analysis selection differs from run selection for {subject}")
    rows = jsonl(trace_path) if trace_path.exists() else []
    validate_trace(rows, subject)
    sources.extend([receipt_path, trace_path] if trace_path.exists() else [receipt_path])
    for field, receipt_key in (("model", "model"), ("IK", "ik"), ("measured_GRF", "prescribed_grf"), ("external_loads", "external_loads_xml"), ("reserve_actuators", "reserve_actuators")):
        expected = chosen["source_fingerprints"][field]
        captured = receipt["source_files"][receipt_key]
        if captured["sha256"] != expected["sha256"] or fingerprint(resolve(expected["path"]))["sha256"] != expected["sha256"]:
            raise ValueError(f"Frozen subject source mismatch: {subject}/{field}")
        sources.append(resolve(expected["path"]))
    for value, key in ((chosen["reset_hs_s"], "actual_episode_start_s"), (chosen["requested_end_s"], "actual_episode_end_s")):
        captured = receipt.get(key, receipt.get("requested_time_range", [None, None])[0 if "start" in key else 1])
        if captured is not None and abs(float(captured) - value) > 1e-8:
            raise ValueError(f"Actual requested episode deviates from frozen selection: {subject}/{key}")
    prefix = receipt.get("effective_simulator_config", {}).get("output_prefix", "sim_output")
    force_path = directory / "sim_outputs" / f"{prefix}_online_grf.sto"
    kin_path = directory / "sim_outputs" / f"{prefix}_kinematics.sto"
    raw_path = resolve(chosen["source_fingerprints"]["IK"]["path"])
    measured_path = resolve(chosen["source_fingerprints"]["measured_GRF"]["path"])
    result = {"receipt": fingerprint(receipt_path), "receipt_status": receipt["status"], "completed_requested_duration": receipt.get("completed_requested_duration", False), "requested_window_s_frozen": [chosen["reset_hs_s"], chosen["requested_end_s"]], "recorded_last_trace_boundary_s": float(rows[-1]["time"]) if rows else None, "normalization": {"mass_kg": chosen["model_mass_kg_from_XML"], "BW_n": chosen["BW_normalization_n"], "source": "Frozen XML body-mass sum; captured and current model hash match selection; g=9.80665."}, "primary_cycle_count_required_by_design": 3, "primary_cycles_frozen": chosen["primary_cycles"], "startup_cycle_frozen": chosen["startup_cycle"]}
    if not force_path.exists() or not kin_path.exists():
        result.update(status="no_recorded_physical_tables", primary_cycle_metrics=[{"status": "not_fully_observed", "frozen_nominal_cycle": cycle} for cycle in chosen["primary_cycles"]], fully_observed_primary_cycle_count=0)
        return result, receipt
    sources.extend([force_path, kin_path])
    force, kinematics = table(force_path, reader), table(kin_path, reader)
    raw_ik, measured = table(raw_path, reader), table(measured_path, reader)
    measured["columns"]["left_vertical_force"] = measured["columns"][chosen["left_force_column"]]
    dt = float(receipt["effective_simulator_config"]["dt"])
    if not np.allclose(np.diff(force["time"]), dt, rtol=0, atol=1e-8):
        raise ValueError(f"Saved force acquisition does not match captured integration dt for {subject}")
    result["physical_table_coverage_s"] = {"force": [float(force["time"][0]), float(force["time"][-1])], "kinematics": [float(kinematics["time"][0]), float(kinematics["time"][-1])]}
    result["integration_dt_s"] = dt
    weight, threshold = chosen["BW_normalization_n"], selection["criterion"]["measured_GRF_event_threshold_n"]
    segments = [("startup", chosen["startup_cycle"])] + [(f"primary_{index}", cycle) for index, cycle in enumerate(chosen["primary_cycles"], 1)]
    calculated = {name: physical_segment(cycle, force, measured, kinematics, raw_ik, weight, threshold) for name, cycle in segments}
    result["startup"] = calculated["startup"]
    result["primary_cycle_metrics"] = [calculated[f"primary_{index}"] for index in range(1, 4)]
    full_count = sum(item["status"] == "fully_observed" for item in result["primary_cycle_metrics"])
    result["fully_observed_primary_cycle_count"] = full_count
    result["status"] = "three_frozen_primary_cycles_observed" if full_count == 3 else "incomplete_frozen_primary_cycles_no_replacement"
    if full_count == 3:
        combined = {"hs_s": chosen["primary_window_s"][0], "next_hs_s": chosen["primary_window_s"][1], "label": "three_contiguous_frozen_nominal_cycles"}
        result["primary_aggregate"] = physical_segment(combined, force, measured, kinematics, raw_ik, weight, threshold)
    else:
        result["primary_aggregate"] = {"status": "not_available_for_all_three_frozen_cycles"}
    for name, cycle in segments:
        if calculated[name]["status"] != "fully_observed":
            continue
        phase = np.linspace(0, 1, 1001)
        times = cycle["hs_s"] + phase * (cycle["next_hs_s"] - cycle["hs_s"])
        fy, ref = interpolate(times, force, "left_force_y"), interpolate(times, measured, "left_vertical_force")
        joint_values = {joint: (interpolate(times, kinematics, joint) * (1 if kinematics["degrees"] else 180 / math.pi), interpolate(times, raw_ik, joint) * (1 if raw_ik["degrees"] else 180 / math.pi)) for joint in JOINTS}
        for index in range(len(phase)):
            phase_rows[subject].append({"segment": name, "phase_fraction": float(phase[index]), "time_s": float(times[index]), "online_Fy_n": float(fy[index]), "measured_Fy_n": float(ref[index]), "online_Fy_bw": float(fy[index] / weight), "measured_Fy_bw": float(ref[index] / weight), **{f"{joint}_{kind}_deg": float(values[part][index]) for joint, values in joint_values.items() for kind, part in (("actual", 0), ("raw_IK", 1))}})
    binary = [event for row in rows for event in row.get("binary_phase_fsm", {}).get("events_this_step", [])]
    transitions = [event for row in rows for event in row.get("phase_fsm", {}).get("accepted_transitions_this_step", [])]
    cancellations = [event for event in transitions if event.get("event") == "heel_strike_cancelled"]
    retained = [event for event in transitions if event.get("event") in {"heel_strike", "toe_off"} and not (event.get("event") == "heel_strike" and any(abs(float(event["event_time_s"]) - float(cancel["event_time_s"])) <= 1e-8 for cancel in cancellations))]
    force_events = [{"event": name, "event_time_s": float(time), "confirmed_time_s": float(time), "delivered_time_s": float(time), "source": "raw 20 N_vertical_force_crossing_no_debounce_no_delivery"} for name in ("heel_strike", "toe_off") for time in crossing(force["time"], force["columns"]["left_force_y"], threshold, rising=name == "heel_strike")]
    result["event_channels"] = {}
    for scope, cycles in (("primary", chosen["primary_cycles"]), ("startup", [chosen["startup_cycle"]])):
        reference, window = nominal(cycles), [cycles[0]["hs_s"], cycles[-1]["next_hs_s"]]
        result["event_channels"][scope] = {"binary_geometry_FSM": compare_events(binary, reference, window, matcher, tolerances, holdout), "actor_retained_after_HS_repairs": compare_events(retained, reference, window, matcher, tolerances, holdout), "primary_force_20N_crossings": compare_events(force_events, reference, window, matcher, tolerances, holdout)}
        for event_result in result["event_channels"][scope]["primary_force_20N_crossings"].values():
            event_result["channels"] = {"onset": event_result["channels"]["onset"]}
            event_result["note"] = "Raw force crossings have no binary confirmation/delivery semantics. Their interpolated crossing timing is a separate diagnostic channel."
    primary_start, primary_end = chosen["primary_window_s"]
    primary_rows = [row for row in rows if primary_start <= float(row["time"]) <= primary_end]
    result["actual_minus_served_reference_deg_10ms_primary"] = {}
    for joint in JOINTS:
        errors = []
        for row in primary_rows:
            state = row.get("prosthetic_state", row.get("observation", {}))
            if joint in state and f"{joint}_served_ref" in state:
                errors.append(math.degrees(float(state[joint]) - float(state[f"{joint}_served_ref"])))
        result["actual_minus_served_reference_deg_10ms_primary"][joint] = metrics(errors)
    samples = [sample for row in rows for sample in row.get("binary_phase_sensor_samples", []) if primary_start <= float(sample["time_s"]) < primary_end]
    result["binary_boolean_contact_1ms_primary"] = {"sample_count": len(samples), "heel_contact_fraction": float(np.mean([sample["left_heel_contact"] for sample in samples])) if samples else None, "toe_contact_fraction": float(np.mean([sample["left_toe_contact"] for sample in samples])) if samples else None, "any_point_contact_fraction": float(np.mean([sample["left_heel_contact"] or sample["left_toe_contact"] for sample in samples])) if samples else None, "coverage_note": "Only actually saved boolean samples inside the frozen primary window; absent/partial recordings are not reconstructed."}
    secondary = {}
    for label, mask in (("entire_saved_episode", np.ones(len(force["time"]), dtype=bool)), ("after_primary", force["time"] >= primary_end)):
        times = force["time"][mask]
        fy = force["columns"]["left_force_y"][mask]
        penetration = force["columns"]["left_penetration"][mask]
        secondary[label] = {"recorded_sample_count": len(times), "recorded_sample_window_s": [float(times[0]), float(times[-1])] if len(times) else None, "vertical_force_n": metrics(fy), "penetration_m": metrics(penetration), "fraction_above_captured_soft_penetration": float(np.mean(penetration > receipt["effective_env_config"]["grf_penetration_penalty_threshold_m"])) if len(times) else None, "fraction_above_captured_termination_penetration": float(np.mean(penetration > receipt["effective_env_config"]["grf_penetration_termination_m"])) if len(times) else None, "scope": "Secondary statistics of saved samples only. A recorded last sample is never substituted for the frozen requested episode end or a nominal cycle boundary."}
    result["remaining_episode_secondary_saved_sample_statistics"] = secondary
    result["full_episode_secondary_journal"] = phase_helper(rows, nominal([chosen["startup_cycle"]] + chosen["primary_cycles"]), tolerances, {"sensor_holdout_heel_strike_max_error_s": holdout["heel_strike"], "sensor_holdout_toe_off_max_error_s": holdout["toe_off"]}, matcher, chosen["reset_hs_s"])
    result["full_episode_secondary_journal"]["interpretation"] += " Its event matching is against the frozen startup+primary events, so full 9 s tail events are explicitly extras; only event_channels.primary supplies the primary event comparison. Swing duration journal can include repaired/previously revoked entries and is not a count of independent physiological swings."
    result["full_episode_secondary_journal"]["all_primary_force_20N_crossings"] = force_events
    return result, receipt


def differences(left, right, allowed):
    return {key: {"AB06": left.get(key), "AB07": right.get(key)} for key in sorted(set(left) | set(right)) if left.get(key) != right.get(key) and key not in allowed}


def compare_selected_sea(left, right):
    records = []
    for actuator in sorted(set(left) | set(right)):
        for key in sorted(set(left.get(actuator, {})) | set(right.get(actuator, {}))):
            before, after = left.get(actuator, {}).get(key), right.get(actuator, {}).get(key)
            if before == after:
                continue
            try:
                semantic_equal = math.isclose(float(before), float(after), rel_tol=1e-14, abs_tol=1e-14)
                absolute_difference = abs(float(before) - float(after))
            except (TypeError, ValueError):
                semantic_equal = before == after
                absolute_difference = None
            records.append({"actuator": actuator, "property": key, "AB06_raw_XML": before, "AB07_raw_XML": after, "absolute_numeric_difference": absolute_difference, "semantic_equal_within_declared_tolerance": semantic_equal})
    return {"relative_tolerance": 1e-14, "absolute_tolerance": 1e-14, "raw_text_differences": records, "semantic_equal": all(record["semantic_equal_within_declared_tolerance"] for record in records), "scope": "Only SEA properties explicitly captured in the frozen selection. This numerical text-serialization comparison is not a subject gait gate; complete runtime SEA/PI equations are checked by the separate native numerical audit."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ab06", type=Path, default=REPO / "results/AB06_paired_prescribed_HS_9s_2026_10_07")
    parser.add_argument("--ab07", type=Path, default=REPO / "results/AB07_paired_prescribed_HS_9s_2026_10_07")
    parser.add_argument("--selection", type=Path, default=AUDIT / "selection.json")
    parser.add_argument("--out", type=Path, default=AUDIT / "comparison_metrics.json")
    args = parser.parse_args()
    selection_path, out = resolve(args.selection), resolve(args.out)
    selection = load(selection_path)
    if selection["criterion"]["primary_cycle_count_each_subject"] != 3:
        raise ValueError("Frozen design requires three primary cycles")
    inherited = load(TOLERANCE_RECEIPT)["methodology"]
    tolerances, holdout = inherited["v26_inherited_confirmed_and_delivered_tolerances_s"], inherited["sensor_holdout_tolerances_s"]
    reader, matcher = pure(ANALYZER, "read_storage"), pure(MATCHER, "match_events")
    crossing, phase_helper = pure(CROSSING, "_crossing_times"), pure(PHASE_HELPER, "phase_analysis")
    sources = [Path(__file__).resolve(), selection_path, ANALYZER, MATCHER, CROSSING, PHASE_HELPER, TOLERANCE_RECEIPT]
    phase_rows = {"AB06": [], "AB07": []}
    results, receipts = {}, {}
    for subject, directory in (("AB06", resolve(args.ab06)), ("AB07", resolve(args.ab07))):
        results[subject], receipts[subject] = analyze_subject(subject, directory, selection, reader, matcher, crossing, phase_helper, tolerances, holdout, sources, phase_rows)
    env_allowed = {"setup_xml_path", "output_dir", "episode_start_offset_s"}
    physics_allowed = {"model_file", "model_bundle_dir", "kinematics_file", "external_loads_xml", "reserve_actuators_xml", "t_start", "t_end", "output_dir"}
    configuration = {"effective_env_unexpected_differences": differences(receipts["AB06"].get("effective_env_config", {}), receipts["AB07"].get("effective_env_config", {}), env_allowed), "effective_simulator_unexpected_differences": differences(receipts["AB06"].get("effective_simulator_config", {}), receipts["AB07"].get("effective_simulator_config", {}), physics_allowed), "profile_association_exactly_equal": receipts["AB06"]["profile_association"] == receipts["AB07"]["profile_association"], "SEA_XML_text_equal": selection["subjects"]["AB06"]["SEA_properties_from_XML"] == selection["subjects"]["AB07"]["SEA_properties_from_XML"], "SEA_XML_semantic_comparison": compare_selected_sea(selection["subjects"]["AB06"]["SEA_properties_from_XML"], selection["subjects"]["AB07"]["SEA_properties_from_XML"]), "allowed_subject_changes": "subject model/anatomy/mass, IK, measured GRF including right applied GRF, subject reserve definitions, source time ranges, setup/output paths and reset offsets chosen by frozen rule"}
    result = {"schema_version": 1, "instruction_check_token": "CMC_AGENT_OK_2026", "created_utc": datetime.now(timezone.utc).isoformat(), "scope": "Pure postprocessing of saved finalized paired prescribed outputs. No simulator/policy/inference/detector runtime, no original artifact writes or frozen-window replacement.", "selection": fingerprint(selection_path), "configuration_comparability": configuration, "results": results, "methodology": {"primary": "Exactly three preselected nominal complete cycles after one excluded nominal startup cycle for each subject; cycle boundaries come from measured GRF, never simulated FSM event counts.", "force_threshold_n": selection["criterion"]["measured_GRF_event_threshold_n"], "acquisition": "Saved native 1 ms STO grids, checked against captured integration_dt. Exact nominal endpoints are interpolated inside observed support, allowing 10 ns timestamp coincidence for STO 8-decimal serialization only; no extrapolation beyond that declared rounding allowance.", "force_normalization": "Fy/BW with frozen subject model mass times9.80665. Compare simulation/measurement impulse ratio per-cycle and ratio of summed impulses; BW normalization is postprocessing only.", "phase_waveform_csv": "1001 points per nominal cycle, phase0..1, derived from native samples for shape visualization only; physical impulse/RMS metrics retain native 1 ms grids.", "event_semantics": "Saved binary geometryFSM onset, confirmed and delivered versus measured GRF events are separate from retained actor events and raw vertical-force 20 N crossings. Missing and extra events are retained.", "inherited_tolerances_s": {"confirmed_and_delivered": tolerances, "onset_reference": holdout}, "tolerance_scope": "Inherited diagnostic references, not newly adopted subject acceptance gates.", "interruptions": "Frozen requested duration and cycle boundaries remain unchanged. Any unavailable full primary cycle is reported unavailable and the3-cycle aggregate is withheld; observed last sample never becomes a replacement nominal end."}, "limitations": ["This is an IK-driven bank-control comparison, not a learned-policy transfer qualification.", "Subject changes include anatomical model, IK and right measured loading, so deviations do not isolate one contact-profile parameter or mechanism.", "Startup and remaining 9 s tail are secondary; full episode duration completion is distinct from three-cycle observation and physiological validity.", "An HS at the initial reset is initialization, not an expected new streamed transition. Startup matching retains this fact as an explicit missing/reference distinction rather than silently changing primary windows.", "Nominal GRF threshold events are production-derived references rather than manual physiological annotations.", "Actor-cycle validity and generic IK tracking criteria are diagnostics and do not create new AB06/AB07 acceptance gates."], "source_files": [fingerprint(path) for path in dict.fromkeys(sources)]}
    result = strict_statistics(result)
    serialized = json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    if out.exists():
        raise FileExistsError(f"Refuse to overwrite audit artifact {out}")
    csv_outputs = {}
    for subject, rows in phase_rows.items():
        if rows:
            path = out.parent / f"{out.stem}_phase_waveforms_{subject}.csv"
            if path.exists():
                raise FileExistsError(path)
            with path.open("x", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
            csv_outputs[subject] = fingerprint(path)
    result["phase_waveform_outputs"] = csv_outputs
    serialized = json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    with out.open("x", encoding="utf-8") as stream:
        stream.write(serialized)
    load(out)
    print(json.dumps({"output": fingerprint(out), "subjects": {subject: {"status": value["status"], "fully_observed_primary_cycles": value["fully_observed_primary_cycle_count"], "completed_requested_duration": value["completed_requested_duration"]} for subject, value in results.items()}}, indent=2))


if __name__ == "__main__":
    main()
