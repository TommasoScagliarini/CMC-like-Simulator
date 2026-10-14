"""Saved-data comparison of AB07 best47 transfer with matched prescribed data.

No policy, OpenSim model, environment, detector or controller is constructed.
Inherited event tolerances remain diagnostic references, never new gates.
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


REPO = Path(__file__).resolve().parents[4]
AUDIT = Path(__file__).resolve().parent
PRESCRIBED = REPO / "results/ab07_v26_ground_registered_prescribed_9p01s_2026_10_06_r1"
AB06 = REPO / "Trajectory Generator/runs/rollout/MLP_V26C_best47_AB06_control_2026_10_06_5s"
ANALYZER = REPO / "Trajectory Generator/validation/AB07/analyze_prescribed_v26.py"
MATCHER = REPO / "validation/validate_online_grf_events.py"
CROSSING = REPO / "validation/validate_online_grf.py"
TOLERANCES = REPO / "validation/validate_binary_phase_fsm_v20_development.py"
HOLDOUT = REPO / "validation/online_grf_acceptance_thresholds.json"
JOINTS = ("pros_knee_angle", "pros_ankle_angle")


def resolve(raw):
    path = Path(str(raw).replace("\\", "/")).expanduser()
    return path.resolve() if path.is_absolute() else (REPO / path).resolve()


def rel(path):
    try:
        return path.resolve().relative_to(REPO).as_posix()
    except ValueError:
        return path.resolve().as_posix()


def fingerprint(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return {"path": rel(path), "sha256": digest.hexdigest()}


def load(path):
    def reject(value):
        raise ValueError(f"Non-finite JSON constant {value}: {path}")
    return json.loads(path.read_text(encoding="utf-8"), parse_constant=reject)


def jsonl(path):
    def reject(value):
        raise ValueError(f"Non-finite JSONL constant {value}: {path}")
    with path.open(encoding="utf-8") as handle:
        return [json.loads(line, parse_constant=reject) for line in handle if line.strip()]


def compliant_statistics(value, empty_paths, path=""):
    if isinstance(value, dict):
        return {key: compliant_statistics(item, empty_paths, f"{path}.{key}".lstrip(".")) for key, item in value.items()}
    if isinstance(value, list):
        return [compliant_statistics(item, empty_paths, f"{path}[{index}]") for index, item in enumerate(value)]
    if isinstance(value, np.generic):
        return compliant_statistics(value.item(), empty_paths, path)
    if isinstance(value, float) and not math.isfinite(value):
        if "matching" not in path:
            raise ValueError(f"Unexpected non-finite analysis value: {path}")
        empty_paths.append(path)
        return None
    return value


def require_coverage(times, source_times, label):
    if not len(source_times) or np.any(np.diff(source_times) <= 0):
        raise ValueError(f"Invalid saved source time grid: {label}")
    if times[0] < source_times[0] - 1e-8 or times[-1] > source_times[-1] + 1e-8:
        raise ValueError(f"Matched timestamps outside saved source coverage: {label}")


def pure_function(source, name):
    tree = ast.parse(source.read_text(encoding="utf-8"))
    function = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == name)
    namespace = {"np": np, "Path": Path, "Any": Any, "Iterable": Iterable, "csv": csv}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), "exec"), namespace)
    return namespace[name]


def inherited_tolerances():
    wanted = {"HS_TOLERANCE_S", "TO_TOLERANCE_S", "DELIVERED_HS_TOLERANCE_S", "DELIVERED_TO_TOLERANCE_S"}
    found = {}
    for node in ast.parse(TOLERANCES.read_text()).body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id in wanted:
                    found[target.id] = float(ast.literal_eval(node.value))
    if set(found) != wanted:
        raise ValueError("Inherited timing constants are incomplete")
    return found


def metrics(values):
    values = np.asarray(list(values), dtype=float)
    if not values.size:
        return None
    if not np.all(np.isfinite(values)):
        raise ValueError("Non-finite saved numeric values")
    return {"count": int(values.size), "min": float(np.min(values)), "max": float(np.max(values)),
            "mean": float(np.mean(values)), "median": float(np.median(values)),
            "rms": float(np.sqrt(np.mean(values * values)))}


def table(root, suffix, reader, sources):
    files = list((root / "sim_outputs").glob(f"*_{suffix}.sto"))
    if len(files) != 1:
        raise ValueError(f"Expected one {suffix} STO in {root}, found {len(files)}")
    sources.append(files[0])
    times, labels, values, degrees = reader(files[0])
    return {"path": files[0], "time": times, "columns": {name: values[:, index] for index, name in enumerate(labels)}, "in_degrees": degrees}


def selection(times, start, end):
    return (times >= start - 1e-8) & (times <= end + 1e-8)


def phase_analysis(rows, nominal, tolerances, holdout, matcher, reset_start):
    transitions = [event for row in rows for event in row.get("phase_fsm", {}).get("accepted_transitions_this_step", [])]
    cancellations = [event for event in transitions if event.get("event") == "heel_strike_cancelled"]
    revoked = [event for event in transitions if event.get("event") == "heel_strike" and any(abs(float(event["event_time_s"]) - float(cancel["event_time_s"])) <= 1e-8 for cancel in cancellations)]
    retained = [event for event in transitions if event.get("event") in ("heel_strike", "toe_off") and event not in revoked]
    binary_observed = bool(rows) and all("binary_phase_fsm" in row for row in rows)
    binary = [event for row in rows for event in row.get("binary_phase_fsm", {}).get("events_this_step", [])]
    matching = {}
    for channel, events in (("actor_historically_accepted", transitions), ("actor_retained_after_repairs", retained), ("binary", binary if binary_observed else None)):
        if events is None:
            matching[channel] = {"status": "not_recorded"}
            continue
        matching[channel] = {}
        for name, code, holdout_key in (("heel_strike", "HS", "sensor_holdout_heel_strike_max_error_s"), ("toe_off", "TO", "sensor_holdout_toe_off_max_error_s")):
            selected = [event for event in events if event.get("event") == name]
            onset = [float(event["event_time_s"]) for event in selected]
            confirmed = [float(event["confirmed_time_s"]) for event in selected]
            delivered = [float(event["delivered_time_s"]) for event in selected]
            matching[channel][name] = {
                "onset_times_s": onset, "confirmed_times_s": confirmed, "delivered_times_s": delivered,
                "onset_holdout_reference_matching": matcher(nominal[name], onset, float(holdout[holdout_key])),
                "confirmed_v26_reference_matching": matcher(nominal[name], confirmed, tolerances[f"{code}_TOLERANCE_S"]),
                "delivered_v26_reference_matching": matcher(nominal[name], delivered, tolerances[f"DELIVERED_{code}_TOLERANCE_S"]),
            }
    segments = {"startup_partial_stance": [], "anchored_valid_stance": [], "other_stance": [], "swing": [], "cancelled_hs_repairs": []}
    for event in transitions:
        if event.get("event") == "heel_strike_cancelled":
            segments["cancelled_hs_repairs"].append(event)
            continue
        kind = event.get("closed_segment_type")
        if kind not in ("stance", "swing"):
            continue
        entry = dict(event, duration_s=float(event["segment_end_time_s"]) - float(event["segment_start_time_s"]))
        if kind == "swing":
            segments["swing"].append(entry)
        elif abs(float(event["segment_start_time_s"]) - reset_start) <= 1e-8 and float(event.get("anchor_geometry_valid", 0)) != 1:
            segments["startup_partial_stance"].append(entry)
        elif float(event.get("segment_valid", 0)) == 1 and float(event.get("anchor_geometry_valid", 0)) == 1:
            segments["anchored_valid_stance"].append(entry)
        else:
            segments["other_stance"].append(entry)
    samples = [sample for row in rows for sample in row.get("binary_phase_sensor_samples", [])]
    binary_contact = {"status": "recorded", "sample_count": len(samples)} if samples else {"status": "not_recorded"}
    if samples:
        binary_contact.update({
            "heel_contact_fraction": float(np.mean([sample["left_heel_contact"] for sample in samples])),
            "toe_contact_fraction": float(np.mean([sample["left_toe_contact"] for sample in samples])),
            "any_point_contact_fraction": float(np.mean([sample["left_heel_contact"] or sample["left_toe_contact"] for sample in samples])),
            "first_sample_time_s": float(samples[0]["time_s"]), "last_sample_time_s": float(samples[-1]["time_s"]),
        })
    last = rows[-1].get("phase_fsm", {}) if rows else {}
    return {
        "last_counters": {key: last.get(key) for key in ("valid_hs_count", "valid_to_count", "valid_cycle_count", "invalid_event_count", "resync_count", "hs_cancelled_count", "last_period_s", "last_stance_fraction")},
        "timeout_flag_nonzero_saved_steps": [float(row["time"]) for row in rows if float(row.get("phase_fsm", {}).get("timeout_exceeded", 0)) != 0],
        "accepted_transitions": transitions,
        "heel_strike_cancellations": cancellations,
        "previously_accepted_heel_strikes_revoked": revoked,
        "ordinary_actor_events_retained_after_repairs": retained,
        "invalid_resync_or_rejection_journal": [{"time": float(row["time"]), **{key: row["phase_fsm"].get(key) for key in ("invalid_event_this_step", "invalid_event_type", "invalid_event_count", "resync_event_this_step", "resync_count", "hs_cancelled_count", "cycle_rejected_this_step", "cycle_reject_reason", "cycle_degraded")}}
            for row in rows if any(float(row.get("phase_fsm", {}).get(key, 0)) != 0 for key in ("invalid_event_this_step", "resync_event_this_step", "cycle_rejected_this_step"))],
        "historically_marked_valid_cycle_closures": [event for event in transitions if float(event.get("cycle_valid", -1)) == 1],
        "closed_segments": segments,
        "segment_duration_s": {name: metrics(event["duration_s"] for event in events) for name, events in segments.items() if name != "cancelled_hs_repairs"},
        "event_matching": matching,
        "binary_boolean_contact_1ms": binary_contact,
        "interpretation": "The historical acceptance journal preserves subsequently revoked HS; the retained-after-repairs matching and final counters are separate. HS cancellation records are repairs rather than physiological zero-duration stances. Actor-valid cycles remain distinct from binary events and agreement with measured contact; an unanchored startup TO remains an explicit extra against the complete-cycle oracle.",
    }


def load_analysis(force, times, oracle_force, threshold, crossing, nominal, holdout, matcher):
    columns = force["columns"]
    vertical = np.interp(times, force["time"], columns["left_force_y"])
    penetration = np.interp(times, force["time"], columns["left_penetration"])
    reference = np.interp(times, oracle_force[0], oracle_force[1])
    trapezoid = np.trapezoid if hasattr(np, "trapezoid") else np.trapz
    online_impulse = float(trapezoid(vertical, times))
    reference_impulse = float(trapezoid(reference, times))
    prediction, truth = vertical > threshold, reference > threshold
    tp, fp, fn = (int(np.count_nonzero(values)) for values in (prediction & truth, prediction & ~truth, ~prediction & truth))
    events = {name: crossing(times, vertical, threshold, rising=name == "heel_strike").tolist() for name in ("heel_strike", "toe_off")}
    return {
        "time_basis_s": [float(times[0]), float(times[-1])], "sample_count": int(len(times)),
        "method": "Trapezoid on matched saved 1 ms STO timestamps; reference GRF interpolated from measured data. No extrapolation or missing terminal sample is invented.",
        "vertical_force_n": metrics(vertical), "reference_vertical_force_n": metrics(reference),
        "vertical_error_n": metrics(vertical - reference),
        "vertical_correlation": float(np.corrcoef(vertical, reference)[0, 1]) if np.std(vertical) and np.std(reference) else None,
        "vertical_impulse_ns": online_impulse, "reference_vertical_impulse_ns": reference_impulse,
        "vertical_impulse_ratio_to_oracle": online_impulse / reference_impulse if reference_impulse else None,
        "vertical_peak_ratio_to_oracle": float(np.max(vertical) / np.max(reference)) if np.max(reference) else None,
        "penetration_m": metrics(penetration),
        "fraction_above_existing_soft_20mm": float(np.mean(penetration > .020)),
        "fraction_above_existing_termination_28mm": float(np.mean(penetration > .028)),
        "fraction_above_existing_runner_hard_30mm": float(np.mean(penetration > .030)),
        "online_contact_fraction": float(np.mean(prediction)), "oracle_contact_fraction": float(np.mean(truth)),
        "contact_at_oracle_threshold": {"threshold_n": threshold, "tp": tp, "fp": fp, "fn": fn, "f1": 2 * tp / max(1, 2 * tp + fp + fn)},
        "raw_primary_force_crossings_s": events,
        "raw_primary_force_crossing_holdout_reference_matching": {name: matcher(nominal[name], events[name], float(holdout[key])) for name, key in (("heel_strike", "sensor_holdout_heel_strike_max_error_s"), ("toe_off", "sensor_holdout_toe_off_max_error_s"))},
        "raw_force_crossings_are_binary_events": False,
    }


def waveforms(kinematics, raw_ik, times, cycles):
    result = {}
    for joint in JOINTS:
        actual = np.interp(times, kinematics["time"], kinematics["columns"][joint])
        if kinematics["in_degrees"]:
            actual = np.radians(actual)
        raw = np.interp(times, raw_ik["time"], raw_ik["columns"][joint])
        if raw_ik["in_degrees"]:
            raw = np.radians(raw)
        records = []
        for cycle in cycles:
            parts = {}
            for name, left, right in (("full_cycle", cycle["hs_s"], cycle["next_hs_s"]), ("stance", cycle["hs_s"], cycle["to_s"]), ("swing", cycle["to_s"], cycle["next_hs_s"])):
                use = selection(times, left, right)
                parts[name] = {"actual_deg": metrics(np.degrees(actual[use])), "raw_ik_deg": metrics(np.degrees(raw[use])),
                               "actual_minus_raw_ik_deg": metrics(np.degrees(actual[use] - raw[use])),
                               "actual_excursion_deg": float(np.degrees(np.ptp(actual[use]))) if np.any(use) else None,
                               "raw_ik_excursion_deg": float(np.degrees(np.ptp(raw[use]))) if np.any(use) else None}
            records.append({"nominal_cycle": cycle, "phases": parts})
        result[joint] = {"actual_deg": metrics(np.degrees(actual)), "raw_ik_deg": metrics(np.degrees(raw)),
                         "actual_minus_raw_ik_deg": metrics(np.degrees(actual - raw)),
                         "nominal_complete_cycles": records}
    return result


def boundary_diagnostics(rows):
    result = {}
    for joint in JOINTS:
        errors = []
        for row in rows:
            state = row.get("prosthetic_state", row.get("observation", {}))
            errors.append(math.degrees(float(state[joint]) - float(state[f"{joint}_served_ref"])))
        result[joint] = metrics(errors)
    morphology_keys = ("morphology_loss", "morphology_knee_loss", "morphology_ankle_loss", "morphology_inside_fraction", "morphology_settled_sample_count", "morphology_pending_sample_count", "morphology_discarded_sample_count", "morphology_causal_failed_closed")
    return {"actual_minus_served_deg_10ms": result,
            "morphology_terms_10ms": {key: metrics(row["reward_terms"][key] for row in rows if key in row.get("reward_terms", {})) for key in morphology_keys},
            "last_morphology_causal_diagnostics": rows[-1].get("morphology_causal_diagnostics"),
            "morphology_interpretation": "Saved reward diagnostics use their own configurations; the learned policy retains its frozen AB06 corridor. They are not an independent AB07 morphology acceptance measure."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--current", type=Path, required=True)
    parser.add_argument("--prescribed", type=Path, default=PRESCRIBED)
    parser.add_argument("--step-info", type=Path, help="Optional saved step-info JSONL with binary payload/samples; no reconstruction if absent")
    parser.add_argument("--out", type=Path, default=AUDIT / "comparison_metrics.json")
    args = parser.parse_args()
    current, prescribed = resolve(args.current), resolve(args.prescribed)
    receipt_path, prescribed_receipt_path = current / "receipt.json", prescribed / "receipt.json"
    receipt, old_receipt = load(receipt_path), load(prescribed_receipt_path)
    if receipt.get("status") != "completed" or old_receipt.get("status") != "completed":
        raise ValueError("Final completed process receipts are required; episode truncation/termination are reported separately")
    current_trace_path, prescribed_trace_path = current / "rollout_policy_trace.json", prescribed / "trace.jsonl"
    current_rows, prescribed_rows = load(current_trace_path), jsonl(prescribed_trace_path)
    sources = [Path(__file__).resolve(), receipt_path, prescribed_receipt_path, current_trace_path, prescribed_trace_path, ANALYZER, MATCHER, CROSSING, TOLERANCES, HOLDOUT]
    sidecar_path = resolve(args.step_info) if args.step_info else resolve(receipt["runtime_diagnostics_path"]) if receipt.get("runtime_diagnostics_path") else None
    sidecar = []
    if sidecar_path:
        sidecar = jsonl(sidecar_path)
        if len(sidecar) != len(current_rows):
            raise ValueError("Sidecar/policy trace row counts differ")
        for row, extra in zip(current_rows, sidecar):
            payload = extra.get("info", extra)
            if abs(float(payload["time"]) - float(row["time"])) > 1e-8:
                raise ValueError("Sidecar/policy trace timestamps differ")
            for key in ("binary_phase_fsm", "binary_phase_sensor_samples", "online_grf", "so_diagnostics", "segment_completed", "target_time", "terminal_contact", "recorded_returned_reward", "recorded_terminated", "recorded_truncated", "failure"):
                if key in payload:
                    row[key] = payload[key]
        sources.append(sidecar_path)
    start, planned_end = float(receipt["actual_episode_start_s"]), float(receipt["actual_episode_end_s"])
    logical_end = float(receipt["final_time_s"])
    terminal_contact = receipt.get("terminal_contact")
    physical_clocks = [float(receipt[key]) for key in ("final_runner_time_s", "final_state_time_s") if receipt.get(key) is not None]
    if terminal_contact and terminal_contact.get("available"):
        physical_clocks.append(float(terminal_contact["time_s"]))
    observed_end = float(receipt["final_state_time_s"]) if receipt.get("final_state_time_s") is not None else float(receipt["final_runner_time_s"]) if receipt.get("final_runner_time_s") is not None else logical_end
    completed_rows = [row for row in current_rows if row.get("segment_completed", True)]
    complete_boundary_end = max((float(row["time"]) for row in completed_rows), default=start)
    if abs(start - float(old_receipt["requested_time_range"][0])) > 1e-8:
        raise ValueError("Current and prescribed initial times differ; no same-reset-window claim is possible")
    current_rows = [row for row in current_rows if start - 1e-8 <= float(row["time"]) <= observed_end + 1e-8]
    prescribed_rows = [row for row in prescribed_rows if start - 1e-8 <= float(row["time"]) <= observed_end + 1e-8]
    reader = pure_function(ANALYZER, "read_storage")
    oracle_cycles = pure_function(ANALYZER, "oracle_cycles")
    matcher = pure_function(MATCHER, "match_events")
    crossing = pure_function(CROSSING, "_crossing_times")
    tolerance_values, holdout = inherited_tolerances(), load(HOLDOUT)
    oracle_path = prescribed / "sim_outputs/sim_output_gait_events.csv"
    sources.append(oracle_path)
    force_path = resolve(old_receipt["source_files"]["prescribed_grf"]["path"])
    ft, flabels, fvalues, _ = reader(force_path)
    sources.append(force_path)
    with oracle_path.open(newline="", encoding="utf-8") as handle:
        oracle_row = next(row for row in csv.DictReader(handle) if row["side"] == "left")
    threshold = float(oracle_row["threshold_n"])
    oracle_force = (ft, fvalues[:, flabels.index(oracle_row["source_force"])])
    planned_cycles, planned_events = oracle_cycles(oracle_path, start, planned_end)
    observed_cycles, nominal = oracle_cycles(oracle_path, start, observed_end)
    force_tables = {"prescribed": table(prescribed, "online_grf", reader, sources), "learned": table(current, "online_grf", reader, sources)}
    current_times = force_tables["learned"]["time"]
    times = current_times[selection(current_times, start, observed_end)]
    if not len(times) or times[0] < force_tables["prescribed"]["time"][0] - 1e-8 or times[-1] > force_tables["prescribed"]["time"][-1] + 1e-8:
        raise ValueError("Current physical sample times fall outside prescribed coverage")
    if times[0] < ft[0] or times[-1] > ft[-1]:
        raise ValueError("Measured force does not cover the matched sample window")
    for name, force_table in force_tables.items():
        require_coverage(times, force_table["time"], f"{name} force")
    require_coverage(times, ft, "measured reference force")
    ik_path = resolve(old_receipt["source_files"]["ik"]["path"])
    it, ilabels, ivalues, idegrees = reader(ik_path)
    sources.append(ik_path)
    raw_ik = {"time": it, "columns": {name: ivalues[:, index] for index, name in enumerate(ilabels)}, "in_degrees": idegrees}
    require_coverage(times, it, "raw IK")
    results = {}
    for name, root, rows in (("prescribed", prescribed, prescribed_rows), ("learned", current, current_rows)):
        kinematics = table(root, "kinematics", reader, sources)
        recruitment = table(root, "recruitment", reader, sources)
        require_coverage(times, kinematics["time"], f"{name} kinematics")
        require_coverage(times, recruitment["time"], f"{name} recruitment")
        results[name] = {
            "force_1ms_matched_window": load_analysis(force_tables[name], times, oracle_force, threshold, crossing, nominal, holdout, matcher),
            "phase_and_events": phase_analysis(rows, nominal, tolerance_values, holdout, matcher, start),
            "waveforms_vs_raw_ik": waveforms(kinematics, raw_ik, times, observed_cycles),
            "saved_policy_boundaries": boundary_diagnostics(rows),
            "recruitment_1ms": {key: metrics(np.interp(times, recruitment["time"], recruitment["columns"][key])) for key in ("tau_reserve_norm", "muscle_capable_reserve_norm", "unactuated_reserve_norm", "muscle_share", "muscle_capable_share") if key in recruitment["columns"]},
        }
    summary_path = current / "rollout_summary.json"
    sources.append(summary_path)
    summary = load(summary_path)
    reset_path = current / "rollout_reset_diagnostics.json"
    sources.append(reset_path)
    reset = load(reset_path)
    prescribed_reset = old_receipt["reset"].get("observation", {})
    reset_comparison = {}
    for joint in JOINTS:
        for current_key, prescribed_key in (("actual_q", joint), ("actual_qdot", f"{joint}_vel"), ("served_q", f"{joint}_served_ref"), ("served_qdot", f"{joint}_served_ref_vel")):
            value = reset["joints"][joint][current_key]
            old_value = prescribed_reset[prescribed_key]
            reset_comparison[f"{joint}.{current_key}"] = {"prescribed": old_value, "learned": value, "equal": old_value == value}
    returned_rewards = [row["recorded_returned_reward"] for row in current_rows if "recorded_returned_reward" in row]
    reward_evidence = {"saved_returned_rewards": len(returned_rewards), "per_step_returned_reward": metrics(returned_rewards),
                       "sum_returned_rewards": float(np.sum(returned_rewards)) if returned_rewards else None,
                       "summary_episode_return": summary["episode_return"],
                       "sum_matches_summary_exactly": float(np.sum(returned_rewards)) == summary["episode_return"] if len(returned_rewards) == len(current_rows) else None}
    current_config, prescribed_config = receipt["effective_simulator_config"], old_receipt["effective_simulator_config"]
    physical_config_differences = {key: {"prescribed": prescribed_config.get(key), "learned": current_config.get(key)}
        for key in sorted(set(current_config) | set(prescribed_config))
        if key not in ("output_dir", "output_prefix") and current_config.get(key) != prescribed_config.get(key)}
    current_env, prescribed_env = receipt["effective_env_config"], old_receipt["effective_env_config"]
    env_differences = {key: {"prescribed": prescribed_env.get(key), "learned": current_env.get(key)}
        for key in sorted(set(current_env) | set(prescribed_env)) if current_env.get(key) != prescribed_env.get(key)}
    shared_source_bindings = {}
    for key in ("model", "ik", "setup", "prescribed_grf", "external_loads_xml", "reserve_actuators"):
        old_source, new_source = old_receipt["source_files"].get(key), receipt["source_fingerprints"].get(key)
        shared_source_bindings[key] = {"prescribed": old_source, "learned": new_source,
                                      "sha256_equal": old_source["sha256"] == new_source["sha256"] if old_source and new_source else None}
    ab06_summary_path = AB06 / "rollout_summary.json"
    ab06_summary = load(ab06_summary_path)
    sources.append(ab06_summary_path)
    payload = {
        "schema_version": 1, "instruction_check_token": "CMC_AGENT_OK_2026", "created_utc": datetime.now(timezone.utc).isoformat(),
        "scope": "Observed AB07 V26C best47 exploratory zero-shot transfer compared with the same-reset prescribed AB07 prefix; AB06 is descriptive only",
        "status": "posthoc_comparison_requires_scientific_interpretation",
        "current": rel(current), "prescribed": rel(prescribed),
        "requested_episode_window_s": [start, planned_end], "observed_episode_window_s": [start, observed_end],
        "observed_clocks_s": {"logical_environment_end": logical_end, "native_runner_end": receipt.get("final_runner_time_s"), "native_state_end": receipt.get("final_state_time_s"), "last_completed_policy_boundary": complete_boundary_end},
        "physical_horizon_authority": "Native OpenSim state time when captured, otherwise native runner time, otherwise explicitly limited to the logical environment time. A terminal guard sample is separate evidence.",
        "other_physical_clock_differences_to_authoritative_end_s": [clock - observed_end for clock in physical_clocks],
        "terminal_contact": terminal_contact,
        "terminal_segment_rows": [{key: row.get(key) for key in ("step", "time", "target_time", "segment_completed", "terminal_contact", "recorded_returned_reward", "failure")} for row in current_rows if not row.get("segment_completed", True)],
        "common_physical_sample_window_s": [float(times[0]), float(times[-1])],
        "completed_requested_duration": receipt.get("completed_requested_duration"),
        "learned_rollout_summary": summary,
        "reset_physical_state_comparison": reset_comparison,
        "shared_subject_source_bindings": shared_source_bindings,
        "simulator_config_differences_excluding_output_location": physical_config_differences,
        "effective_env_config_differences": env_differences,
        "clock_reset_features": {"prescribed": {key: prescribed_reset.get(key) for key in ("gait_phase_sin", "gait_phase_cos")},
                                  "learned": {key: current_rows[0].get("actor_observation_before", {}).get(key) for key in ("gait_phase_sin", "gait_phase_cos")},
                                  "comparison_note": "Learned clock-off convention remains (0,1); prescribed phase clock is an explicit control-contract difference."},
        "returned_reward_observation": reward_evidence,
        "nominal_requested_window": {"complete_cycles": planned_cycles, "events_s": planned_events},
        "nominal_observed_window": {"complete_cycles": observed_cycles, "events_s": nominal},
        "methodology": {"oracle": fingerprint(oracle_path), "force_source": fingerprint(force_path), "raw_ik_source": fingerprint(ik_path),
            "inherited_confirmed_and_delivered_tolerances_s": tolerance_values,
            "inherited_onset_holdout_reference_s": {"heel_strike": holdout["sensor_holdout_heel_strike_max_error_s"], "toe_off": holdout["sensor_holdout_toe_off_max_error_s"]},
            "tolerance_scope": "Historical AB06 binary development and analog holdout diagnostic references; not new AB07 acceptance gates",
            "startup": "Unanchored partial actor stance is separate from anchored valid stance; complete-cycle oracle excludes unanchored initial TO; force-threshold crossings are a separate channel",
            "comparison_window": "Prescribed is cropped to the actual learned observation horizon. If learned ends early, no future prescribed data are counted as the matched window. Planned nominal coverage is retained separately."},
        "results": results,
        "AB06_descriptive_reference": {"path": rel(AB06), "summary": ab06_summary,
            "comparable_subject_or_start": False, "note": "Different subject, trial and event-relative reset; exact reproduction of historical AB06 does not establish AB07 transfer quality."},
        "limitations": [
            "No policy inference, simulator integration, new detector run, saved-pose replay or reward reconstruction is performed.",
            "The prescribed run uses raw IK actions; best47 uses normalized absolute actions with clock disabled and no privileged target inputs in the actor. The intervention changes this control contract; a difference cannot be uniquely attributed to network weights.",
            "The exploratory AB07 start offset 0 differs from the training/AB06 central offset 1.956870983805102; this is not a matched-phase cross-subject test.",
            "1 ms STO physical statistics use pre-integration samples through episode_end minus integration_dt. Earlier 9.01 s qualification impulse used post-step 10 ms boundaries; values across these horizons/time bases are not interchangeable.",
            "Saved binary data are reported only if present. Analog 10 ms heel/toe contacts are not used to invent binary events or 1 ms boolean occupancy.",
            "Reference cycle timing includes measured nominal events; internal valid cycle count alone does not validate gait physiology or autonomy.",
            "Raw IK, served reference and actual trajectories are separate quantities. The frozen AB06 morphology corridor is a reward prior, not a new subject-specific target or acceptance criterion.",
            "Recruitment norms contain mixed generalized force units; they are comparative diagnostics, not direct scalar joint torque or muscle adequacy certification.",
            "Returned reward of a prescribed imitation configuration is not compared to the ex-novo learned return, and cross-subject return is not an autonomous qualification score.",
        ],
        "source_files": [fingerprint(path) for path in dict.fromkeys(sources)],
    }
    empty_paths = []
    payload = compliant_statistics(payload, empty_paths)
    payload["empty_inherited_matching_statistics_written_as_null"] = empty_paths
    failure_provenance = AUDIT / "first_comparison_write_failure.json"
    payload["analysis_write_provenance"] = fingerprint(failure_provenance) if failure_provenance.exists() else None
    serialized = json.dumps(payload, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    out = resolve(args.out)
    with out.open("x", encoding="utf-8") as handle:
        handle.write(serialized)
    print(json.dumps({"output": rel(out), "observed_window_s": payload["observed_episode_window_s"], "nominal_complete_cycles": len(observed_cycles), "steps": summary["steps"]}))


if __name__ == "__main__":
    main()
