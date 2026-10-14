"""Read-only force/timing audit of historical AB06 J1 prescribed teacher.

Uses saved traces and established pure analysis helpers only. It never imports
OpenSim, Ray, the policy, environment, historical collector or detector runtime.
Requires numpy and PyYAML; works with pathlib on macOS and Windows.
"""
from __future__ import annotations

import ast
import csv
import hashlib
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[5]
AUDIT = Path(__file__).resolve().parent
J1 = REPO / "Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/j1_runs/j1_nominal_v26c_2026-08-26_r1"
AB07 = REPO / "results/ab07_v26_ground_registered_prescribed_9p01s_2026_10_06_r1"
ANALYZER = REPO / "Trajectory Generator/validation/AB07/analyze_prescribed_v26.py"
MATCHER = REPO / "validation/validate_online_grf_events.py"


def fingerprint(path: Path) -> dict:
    return {"path": str(path.relative_to(REPO)), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def read_json(path: Path) -> Any:
    def reject(value):
        raise ValueError(f"Source contains nonfinite JSON constant {value}: {path}")
    return json.loads(path.read_text(encoding="utf-8"), parse_constant=reject)


def helper(path: Path, name: str):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    function = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == name)
    namespace = {"np": np, "csv": csv, "Path": Path, "Any": Any}
    module = ast.Module(body=[function], type_ignores=[])
    exec(compile(ast.fix_missing_locations(module), str(path), "exec"), namespace)
    return namespace[name]


def sanitize_empty_matching(value):
    if isinstance(value, dict):
        empty = value.get("matched_count") == 0
        return {key: None if empty and key in {"timing_bias_s", "timing_mae_s", "timing_max_abs_s"} else sanitize_empty_matching(item) for key, item in value.items()}
    if isinstance(value, list):
        return [sanitize_empty_matching(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError("Unexpected nonfinite derived metric")
    return value


def main() -> None:
    receipt = read_json(J1 / "v26c_j1_collection_receipt.json")
    original_trace = read_json(J1 / "teacher_trace.json")
    j1_trace = [{"time": row["time_after"], "phase_fsm": row["phase_fsm"], "online_grf": row["online_grf"], "binary_phase_fsm": row["info_scalars"]["binary_phase_fsm"]} for row in original_trace]
    ab07_trace = [json.loads(line) for line in (AB07 / "trace.jsonl").read_text(encoding="utf-8").splitlines() if line.strip()]
    ab07_analysis = read_json(AB07 / "qualification_analysis.json")
    ab07_receipt = read_json(AB07 / "receipt.json")
    config_path = REPO / receipt["pinned_config"]["path"]
    assert fingerprint(config_path)["sha256"] == receipt["pinned_config"]["sha256"]
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    assert fingerprint(J1 / "teacher_trace.json")["sha256"] == receipt["outputs_sha256"]["teacher_trace.json"]
    read_storage = helper(ANALYZER, "read_storage")
    oracle_cycles = helper(ANALYZER, "oracle_cycles")
    match_events = helper(MATCHER, "match_events")
    tolerances = ab07_analysis["methodology"]["v26_inherited_confirmed_and_delivered_tolerances_s"]
    holdout = ab07_analysis["methodology"]["sensor_holdout_tolerances_s"]

    def analyze(rows, oracle, force_file, episode_start):
        times = np.asarray([row["time"] for row in rows], dtype=float)
        force = np.asarray([row["online_grf"]["left"]["force"][1] for row in rows], dtype=float)
        assert np.all(np.isfinite(force)) and np.all(np.isfinite(times)) and np.all(np.diff(times) > 0)
        ft, labels, data, _ = read_storage(force_file)
        assert ft[0] <= times[0] and ft[-1] >= times[-1]
        with oracle.open(encoding="utf-8") as stream:
            left = next(row for row in csv.DictReader(stream) if row["side"] == "left")
        reference = np.interp(times, ft, data[:, labels.index(left["source_force"])])
        cycles, nominal = oracle_cycles(oracle, episode_start, float(times[-1]))
        events = [event for row in rows for event in row["binary_phase_fsm"]["events_this_step"]]
        comparisons = {}
        for name, code in [("heel_strike", "HS"), ("toe_off", "TO")]:
            selected = [event for event in events if event["event"] == name]
            comparisons[name] = {}
            for channel, field, tolerance in [("onset", "event_time_s", holdout[name]), ("confirmed", "confirmed_time_s", tolerances[f"{code}_TOLERANCE_S"]), ("delivered", "delivered_time_s", tolerances[f"DELIVERED_{code}_TOLERANCE_S"])]:
                values = [float(event[field]) for event in selected]
                comparisons[name][channel] = {"observed_times_s": values, "tolerance_s": tolerance, "matching": sanitize_empty_matching(match_events(nominal[name], values, tolerance))}
        trapz = np.trapezoid if hasattr(np, "trapezoid") else np.trapz
        impulse, ref_impulse = float(trapz(force, times)), float(trapz(reference, times))
        threshold = float(left["threshold_n"])
        pred, ref = force > threshold, reference > threshold
        tp, fp, fn = int(np.count_nonzero(pred & ref)), int(np.count_nonzero(pred & ~ref)), int(np.count_nonzero(~pred & ref))
        return {"episode_window_s": [episode_start, float(times[-1])], "saved_boundary_window_s": [float(times[0]), float(times[-1])], "saved_boundary_count": len(rows), "integration_method": "trapezoid on saved 10 ms post-step boundaries, identical to AB07 test1 qualification", "vertical_impulse_ns": impulse, "measured_vertical_impulse_ns": ref_impulse, "vertical_impulse_ratio": impulse / ref_impulse, "vertical_impulse_percent_difference": (impulse / ref_impulse - 1) * 100, "vertical_correlation": float(np.corrcoef(force, reference)[0, 1]), "vertical_error_rms_n": float(np.sqrt(np.mean((force - reference) ** 2))), "contact_f1": 2 * tp / (2 * tp + fp + fn), "oracle": {"events": fingerprint(oracle), "measured_grf": fingerprint(force_file), "source_force": left["source_force"], "threshold_n": threshold, "min_contact_duration_s": float(left["min_contact_duration_s"]), "min_cycle_duration_s": float(left["min_cycle_duration_s"]), "nominal_events_s": nominal, "nominal_complete_cycles": cycles}, "binary_event_matching": comparisons, "actor_final_counters": {key: rows[-1]["phase_fsm"].get(key) for key in ["valid_hs_count", "valid_to_count", "valid_cycle_count", "invalid_event_count", "resync_count", "hs_cancelled_count"]}}

    j1 = analyze(j1_trace, J1 / "sim_outputs/j1_teacher_episode_gait_events.csv", REPO / "models/AB06_SEASEA_Threadmill/data/AB06_SEASEA_GRF_FullSpan.mot", float(original_trace[0]["time_before"]))
    ab07_args = (AB07 / "sim_outputs/sim_output_gait_events.csv", REPO / "models/AB07_SEASEA_Threadmill/data/AB07_SEASEA_GRF_FullSpan.mot", float(ab07_receipt["requested_time_range"][0]))
    ab07_full = analyze(ab07_trace, *ab07_args)
    assert ab07_full["vertical_impulse_ratio"] == ab07_analysis["primary_left_contact"]["relative_load_diagnostics"]["vertical_impulse_ratio_to_oracle"]
    ab07_prefix = analyze([row for row in ab07_trace if float(row["time"]) <= ab07_args[2] + 5.0 + 1e-8], *ab07_args)
    for name in ["heel_strike", "toe_off"]:
        assert ab07_full["binary_event_matching"][name]["onset"]["matching"] == ab07_analysis["binary_event_comparison"][name]["onset_holdout_reference_matching"]
    sim, grf, reward = config["simulation"], config["grf"], config["reward"]
    shared_profiles = {}
    for old_key, new_key in [("online_grf_profile", "online_grf_profile_file"), ("online_grf_detector_profile", "online_grf_detector_profile_file"), ("binary_phase_detector_profile", "binary_phase_detector_profile_file")]:
        q = REPO / grf[old_key]
        current = fingerprint(q)
        captured = ab07_receipt["profile_association"][new_key]
        shared_profiles[old_key] = {"historical_config_path": grf[old_key], "current_fingerprint": current, "AB07_captured_fingerprint": captured, "current_sha_matches_AB07_captured": current["sha256"] == captured["sha256"], "historical_per_profile_sha_attested_by_J1_receipt": False}
    result = {"schema_version": 1, "instruction_check_token": "CMC_AGENT_OK_2026", "created_utc": datetime.now(timezone.utc).isoformat(), "scope": "Saved AB06 J1 prescribed IK teacher compared to measured AB06 GRF using the same analysis definitions as AB07 test1. No simulation, inference, detector execution or historical-data modification.", "results": {"AB06_J1_nominal_5s": j1, "AB07_test1_full_9p01s": ab07_full, "AB07_test1_first_5s": ab07_prefix}, "J1_receipt": fingerprint(J1 / "v26c_j1_collection_receipt.json"), "J1_collection_semantics": receipt["collection_semantics"], "J1_runtime_identity": receipt["runtime_identity"], "J1_pinned_config": fingerprint(config_path), "J1_teacher_generator": {"function": "target_domain_imitation.prescribed_teacher_action", "semantics": "IK sampled at next knot, encoded as normalized absolute action, sigma0 and lookahead0; no learned actor as action source", "source": fingerprint(REPO / "Trajectory Generator/baseline_MLP/target_domain_imitation.py")}, "profile_comparability": shared_profiles, "protocol_differences": {"J1": {"action_mode": sim["action_mode"], "gait_clock_enable": sim["gait_clock_enable"], "slew_rate_rad_s": [sim["pros_knee_target_slew_rate_limit_rad_s"], sim["pros_ankle_target_slew_rate_limit_rad_s"]], "governor_model": sim["pros_ref_model"], "governor_cutoff_hz": sim["pros_ref_cutoff_hz"], "phase_min_stance_duration_s": reward["phase_min_stance_duration_s"], "phase_min_swing_duration_s": reward["phase_min_swing_duration_s"], "phase_min_stance_contact_fraction": reward["phase_min_stance_contact_fraction"], "phase_min_stance_load_bw_s": reward["phase_min_stance_load_bw_s"], "phase_min_cycle_knee_excursion_rad": reward["phase_min_cycle_knee_excursion_rad"]}, "AB07_test1": {key: ab07_receipt["effective_env_config"].get(key) for key in ["action_mode", "gait_clock_enable", "target_slew_rate_limit_rad_s", "pros_ref_model", "pros_ref_cutoff_hz", "phase_min_stance_duration_s", "phase_min_swing_duration_s", "phase_min_stance_contact_fraction", "phase_min_stance_load_bw_s", "phase_min_cycle_knee_excursion_rad"]}}, "limitations": ["Subjects, reset phases and full duration differ (AB06 5s versus AB07 9.01s). The AB07 prefix has equal duration, not equal gait phase or identical control settings.", "Both action sources derive from IK and both use left applied online GRF with measured GRF retained as reference. J1 normalized absolute teacher uses slew2.5/2 and exnovo-style FSM thresholds; AB07 test1 uses raw playback with disabled slew and imitation-style FSM thresholds.", "The historical pinned YAML references the same three profile paths. Current profile hashes match AB07 captured inputs, but J1 receipt lacks individual historical profile hash attestation; this is provenance compatibility rather than a claim of bit-identical historical runtime.", "The established binary-FSM onset channel is compared with offline measured-GRF cycle events. Onset, confirmation, delivery, actor acceptance and primary-force crossings remain distinct quantities.", "Oracle GRF events use20N/50ms/300ms production extraction; startup partial events are excluded from nominal complete-cycle reference and unmatched observed events remain explicit.", "All timing tolerances are inherited diagnostic references, not new AB06 or AB07 qualification gates.", "The AB06 learned best47 control is deliberately excluded from this prescribed comparison."], "sources": [fingerprint(Path(__file__)), fingerprint(ANALYZER), fingerprint(MATCHER), fingerprint(J1 / "teacher_trace.json"), fingerprint(AB07 / "trace.jsonl"), fingerprint(AB07 / "qualification_analysis.json"), fingerprint(AB07 / "receipt.json")]}
    result = sanitize_empty_matching(result)
    serialized = json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    output = AUDIT / "comparison_metrics.json"
    with output.open("x", encoding="utf-8") as stream:
        stream.write(serialized)
    read_json(output)
    print(json.dumps({"output": fingerprint(output), "summary": {name: {"impulse_percent_difference": case["vertical_impulse_percent_difference"], "HS_onset_bias_ms": case["binary_event_matching"]["heel_strike"]["onset"]["matching"]["timing_bias_s"] * 1000, "TO_onset_bias_ms": case["binary_event_matching"]["toe_off"]["onset"]["matching"]["timing_bias_s"] * 1000} for name, case in result["results"].items()}}, indent=2))


if __name__ == "__main__":
    main()
