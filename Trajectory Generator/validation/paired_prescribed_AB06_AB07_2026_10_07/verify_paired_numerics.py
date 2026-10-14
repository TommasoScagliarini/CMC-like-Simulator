"""Audit saved paired IK-driven rollouts without constructing a runtime.

Preserves original numerical helpers and operational files. Requested duration,
saved pre-integration prefix and logical/physical clocks are separate facts.
"""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict, fields, replace
from datetime import datetime, timezone
import importlib.util
import json
from pathlib import Path
import sys
from xml.etree import ElementTree as ET

import numpy as np

REPO = Path(__file__).resolve().parents[3]
AUDIT = Path(__file__).resolve().parent
TRANSFER_HELPER = REPO / "Trajectory Generator/validation/AB07/v26c_best47_transfer_2026_10_06/verify_transfer_rollout_numerics.py"
TRANSFER_HELPER_SHA = "4dfb1aff3ff82ac92cafc99cbdc75893bb1e8dbd832368bb325b7f620c69bf4c"


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def trace_clock_and_bw_audit(rows, episode, env_cfg, cfg, mass_kg, h):
    times = np.asarray([float(row["time"]) for row in rows])
    completed = np.asarray([bool(row.get("segment_completed", False)) for row in rows])
    segment_dt = float(env_cfg["segment_duration"])
    expected_steps = int(round((episode["end_s"] - episode["start_s"]) / segment_dt))
    expected_times = episode["start_s"] + np.arange(1, expected_steps + 1) * segment_dt
    expected_completed_times = episode["start_s"] + np.arange(1, int(np.count_nonzero(completed)) + 1) * segment_dt
    sensor_rows, sensor_failures = [], []
    expected_sensor_count = int(round(segment_dt / float(env_cfg["detector_sample_dt_s"])))
    identities = {}
    for prefix in ("gait_phase", "online_left_gait_phase"):
        names = (f"{prefix}_sin", f"{prefix}_cos")
        if rows and all(all(name in row.get("observation", {}) for name in names) for row in rows):
            sine, cosine = [np.asarray([float(row["observation"][name]) for row in rows]) for name in names]
            identities[prefix + "_unit_circle"] = h.identity(sine*sine+cosine*cosine, np.ones(len(rows)), 2e-7, "dimensionless")
    bw = mass_kg * 9.80665
    normal_n, normal_bw, implied_bw = [], [], []
    for index, row in enumerate(rows):
        samples = row.get("binary_phase_sensor_samples", [])
        sample_times = np.asarray([float(sample["time_s"]) for sample in samples])
        expected_grid = float(row["time"]) - segment_dt + np.arange(1, expected_sensor_count + 1) * float(env_cfg["detector_sample_dt_s"])
        grid_matches = bool(len(sample_times) == expected_sensor_count and np.all(np.isfinite(sample_times)) and np.allclose(sample_times, expected_grid, atol=1e-8, rtol=0)) if completed[index] else None
        if completed[index] and not grid_matches:
            sensor_failures.append(index + 1)
        sensor_rows.append({"step": index+1, "logical_time_s": float(row["time"]), "target_time_s": row.get("target_time"), "segment_completed": bool(completed[index]), "samples": len(samples), "sensor_time_first_s": float(sample_times[0]) if len(sample_times) else None, "sensor_time_last_s": float(sample_times[-1]) if len(sample_times) else None, "completed_grid_matches": grid_matches})
        left = row.get("online_gait", {}).get("sides", {}).get("left", {})
        if "normal_force_n" in left and "normal_force_bw" in left:
            force, normalized = float(left["normal_force_n"]), float(left["normal_force_bw"])
            normal_n.append(force)
            normal_bw.append(normalized)
            if abs(normalized) > 1e-10:
                implied_bw.append(force/normalized)
    identities["left_load_normalization_matches_XML_mass"] = h.identity(np.asarray(normal_bw), np.asarray(normal_n)/bw, 1e-10, "dimensionless") if normal_n else {"passed": None, "available": False}
    return {"trace_steps": len(rows), "expected_policy_segments": expected_steps, "completed_policy_segments": int(np.count_nonzero(completed)), "incomplete_attempt_steps": (np.flatnonzero(~completed)+1).tolist(), "covers_requested_segment_end_times": bool(len(times)==expected_steps and np.allclose(times, expected_times, atol=1e-7, rtol=0)), "covers_completed_segment_end_times": bool(len(times) and np.allclose(times[completed], expected_completed_times, atol=1e-7, rtol=0)), "sensor_batches": sensor_rows, "completed_binary_sensor_grid_failure_steps": sensor_failures, "completed_binary_sensor_grids_all_match": not sensor_failures if rows else None, "boundary_observation_identities": identities, "XML_mass_kg": mass_kg, "XML_BW_N": bw, "runtime_implied_BW_N": h.metrics(implied_bw) if implied_bw else None, "BW_source": "Runtime getTotalMass(state)*9.80665 at osim_trj_cmc_like.py:2105, indirectly checked using original online_gait.sides.left.normal_force_n/normal_force_bw exported at :3919. No native model instantiated by this audit.", "clock_note": "Trace observations are after prescribed segment boundaries; they are not learned actor-before observations. Ten 1 ms binary samples are required only for completed 10 ms segments."}


def final_boundary_diagnosis(receipt, rows, env_cfg, h):
    message = "Policy boundaries must span a positive integer number of samples."
    if not rows or message not in receipt.get("error", ""):
        return {"applicable": False}
    previous = float(rows[-1]["time"])
    boundary = float(receipt["final_time_s"])
    sample_dt = float(env_cfg["detector_sample_dt_s"])
    duration = boundary-previous
    count_float = duration/sample_dt
    count = int(round(count_float))
    residual = count_float-count
    sources = ("Trajectory Generator/binary_phase_fsm.py", "Trajectory Generator/binary_phase_adapter.py", "Trajectory Generator/osim_trj_cmc_like.py", "simulation_runner.py")
    return {"applicable": True, "receipt_status": receipt["status"], "receipt_error": receipt["error"], "last_returned_policy_boundary_s": previous, "attempted_final_boundary_s": boundary, "last_committed_binary_sample_time_s": rows[-1]["binary_phase_fsm"]["last_sample_time_s"], "last_segment_duration_s": duration, "sensor_sample_dt_s": sample_dt, "expected_count_float": count_float, "nearest_integer_sample_count": count, "sample_count_residual": residual, "absolute_sample_count_residual": abs(residual), "existing_integer_count_guard_tolerance": 1e-9, "existing_guard_would_reject": count <= 0 or abs(residual)>1e-9, "duration_minus_integer_samples_s": duration-count*sample_dt, "duration_minus_integer_samples_picoseconds": (duration-count*sample_dt)*1e12, "physical_vs_returned_policy_status": "Runner/state/environment getters reach the requested physical target. The last env.step raises during the subsequent atomic binary-FSM transport validation, before observation/reward/termination return and before the harness increments its returned-step counter or writes the final trace row. The receipt remains failed with 899 returned boundaries; the physical 9 s prefix is complete.", "missing_transport_data": "The failed final callback does not return a final observation/reward/event/FSM payload; the final binary batch is not archived in the 899-row trace and is not reconstructed.", "guard_source": "binary_phase_fsm.py:197-205 tests abs((boundary-previous)/sample_dt-round(...))>1e-9; adapter.py:195 validates candidate copies before commit; env.py:1854-1889 advances runner and sets env.t before transport; runner.py:501-547 records before RK4 and updates native/runtime time after integration.", "sources": [{"path": path, "sha256": h.sha256(REPO/path)} for path in sources]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--subject", required=True, choices=("AB06", "AB07"))
    parser.add_argument("--run-dir", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    for path in (REPO, REPO / "Trajectory Generator/baseline_MLP"):
        sys.path.insert(0, str(path))
    import win_runtime  # Existing Windows native-library load-order shim.
    from config import SimulatorConfig
    from validation import validate_sim_results as validator
    import opensim

    transfer = load_module(TRANSFER_HELPER, "preserved_transfer_helpers")
    h = transfer.load_helpers()
    if h.sha256(TRANSFER_HELPER) != TRANSFER_HELPER_SHA:
        raise ValueError("Preserved transfer helper no longer matches its frozen source.")
    prepared_path = AUDIT / args.subject / "prepared_receipt.json"
    prepared = read_json(prepared_path)
    provenance_path = AUDIT / "numerical_review_provenance.json"
    provenance = read_json(provenance_path)
    if h.sha256(prepared_path) != provenance["prepared_receipts"][args.subject]["sha256"]:
        raise ValueError("Prepared receipt changed after the independent numerical review.")
    for source in provenance["read_sources"]:
        if h.sha256(REPO/source["path"]) != source["sha256"]:
            raise ValueError("A frozen review source changed: " + source["path"])
    run = (args.run_dir or Path(prepared["output_directory"])).resolve()
    output = args.output or AUDIT / args.subject / "numerical_verification.json"
    receipt_path = run / "receipt.json"
    receipt = read_json(receipt_path)
    if receipt.get("status") not in {"completed", "interrupted", "failed"}:
        raise ValueError("Run is not finalized; numerical audit will not consume active outputs.")
    snapshot = receipt["effective_simulator_config"]
    expected_keys = {field.name for field in fields(SimulatorConfig)}
    if set(snapshot) != expected_keys:
        raise ValueError("Effective simulator snapshot differs from the complete current config schema.")
    cfg = SimulatorConfig(**snapshot)
    env_cfg = receipt["effective_env_config"]
    kwargs_match = all(env_cfg.get(key) == value for key, value in prepared["env_kwargs"].items())
    if not kwargs_match:
        raise ValueError("Effective environment differs from the prepared contract.")
    episode = transfer.episode_bounds(receipt, receipt.get("reset", {}), cfg, env_cfg)
    coverage_cfg = replace(cfg, t_start=episode["start_s"], t_end=episode["end_s"])
    trace_path = run / "trace.jsonl"
    rows = [json.loads(line) for line in trace_path.read_text(encoding="utf-8").splitlines() if line.strip()] if trace_path.is_file() else []
    outputs = run / "sim_outputs"
    required = ("sea_torques", "sea_controls", "sea_states", "sea_derivatives", "sea_diagnostics", "states", "kinematics", "reserve_torques", "power", "kinematics_reference", "reserve_controls", "recruitment", "so_torque_diagnostics")
    tables = {name: transfer.load_table_allow_empty(outputs/f"{cfg.output_prefix}_{name}.sto", validator) for name in required}
    for path in sorted(outputs.glob(f"{cfg.output_prefix}_*.sto")):
        name = path.stem.removeprefix(cfg.output_prefix+"_")
        if name not in tables:
            tables[name] = transfer.load_table_allow_empty(path, validator)
    anchor = tables["kinematics"].time if tables["kinematics"] is not None else np.empty(0)
    missing = [name for name, table in tables.items() if table is None]
    numeric_available = bool(len(anchor) and not missing)
    table_audit = transfer.partial_table_audit(tables, coverage_cfg, anchor, h)
    fingerprints = {}
    entries = {**receipt["source_files"], **receipt["profile_association"]}
    for name, record in entries.items():
        path = h.resolve_input(record["path"])
        observed = h.sha256(path)
        fingerprints[name] = {"path": str(path.relative_to(REPO)), "expected_sha256": record["sha256"], "current_sha256": observed, "matches_receipt": observed == record["sha256"]}
    model = h.resolve_input(snapshot["model_file"])
    raw = validator.load_table(h.resolve_input(snapshot["kinematics_file"]))
    props = h.xml_sea_properties(model, [cfg.sea_knee_name, cfg.sea_ankle_name], validator)
    precision = int(opensim.CoordinateActuator().getOutput("actuation").getNumberOfSignificantDigits())
    if precision != 8:
        raise ValueError("OpenSim output reporting precision changed from the established 8 significant digits.")
    checks = []
    if numeric_available:
        for check_fn in (validator.validate_sea_tautology, validator.validate_reserves, validator.validate_tracking, validator.validate_power_and_controls, validator.validate_sea_derivatives, validator.validate_sea_interface_diagnostics):
            if check_fn is validator.validate_sea_tautology:
                check_fn(cfg, tables, props, checks)
            elif check_fn is validator.validate_tracking:
                check_fn(cfg, tables, raw, checks)
            else:
                check_fn(cfg, tables, checks)
        tracking = transfer.rename_raw_tracking(h.tracking_audit(cfg, tables, raw, validator))
        sea = h.sea_audit(cfg, tables, props, precision)
        transfer.command_limit_audit(cfg, tables, sea, h)
        reserves = h.reserves_and_recruitment(cfg, tables)
    else:
        tracking, sea, reserves = {"coordinates": {}, "available": False}, {}, {"available": False}
    model_xml = ET.parse(model).getroot()
    mass = sum(float(body.findtext("mass")) for body in model_xml.iter("Body"))
    mtp_metadata = {element.attrib["name"]: {child.tag: (child.text or "").strip() for child in element if child.tag in {"default_value", "range", "clamped", "locked"}} for element in model_xml.iter("Coordinate") if "mtp" in element.attrib.get("name", "").lower()}
    trace_audit = trace_clock_and_bw_audit(rows, episode, env_cfg, cfg, mass, h)
    events = transfer.event_time_audit(run, cfg.output_prefix, rows, episode, cfg)
    final = receipt.get("final_time_s")
    physical = receipt.get("final_runner_time_s")
    native = receipt.get("final_state_time_s")
    saved_end = float(anchor[-1]+cfg.integration_dt) if len(anchor) else None
    non_tautology = [check for check in checks if check.name.endswith(("tau_ref - tau_spring", "algebraic motor state", "motor_speed - qdot"))]
    actual_reset_matches_requested = bool(np.allclose([episode["start_s"], episode["end_s"]], receipt["requested_time_range"], atol=1e-8, rtol=0))
    clocks_at_requested_end = bool(all(value is not None and abs(float(value)-episode["end_s"]) <= 1e-7 for value in (final, physical, native)))
    native_summary = {"grf_penetration_max_m": max((float(row.get("online_grf", {}).get("left", {}).get("penetration", 0.0)) for row in rows), default=None), "grf_penetration_samples": len(rows)}
    summary = {"receipt_status": receipt.get("status"), "completed_requested_duration": receipt.get("completed_requested_duration", False), "clocks_at_requested_end": clocks_at_requested_end, "actual_reset_matches_requested": actual_reset_matches_requested, "end_reason": receipt.get("final_info", {}).get("end_reason"), "termination": {"terminated": receipt.get("terminated"), "truncated": receipt.get("truncated"), "failure": receipt.get("final_failure"), "terminal_contact": receipt.get("terminal_contact"), "close_error": receipt.get("close_error")}, "requested_output_rows": int(round((episode["end_s"]-episode["start_s"])/cfg.integration_dt)), "recorded_output_rows": len(anchor), "missing_tables": missing, "recorded_numerical_data_available": numeric_available, "all_sto_finite": all(table.get("all_values_finite", False) for table in table_audit.values()) if numeric_available else None, "all_sto_same_time_grid": all(table.get("same_time_grid_as_kinematics", False) for table in table_audit.values()) if numeric_available else None, "all_sto_cover_requested_episode": all(table.get("covers_requested_preintegration_samples", False) for table in table_audit.values()), "all_sto_match_contiguous_recorded_prefix": all(table.get("matches_contiguous_recorded_prefix", False) for table in table_audit.values()) if numeric_available else None, "all_SEA_identity_checks_pass": all(entry["all_identity_checks_pass"] for entry in sea.values()) if numeric_available else None, "SEA_identity_count": sum(len(entry["identities"]) for entry in sea.values()), "SEA_non_tautology_checks_pass": all(check.status=="PASS" for check in non_tautology) if len(non_tautology)==6 and len(anchor)>2 else None, "source_fingerprints_all_match_receipt": all(entry["matches_receipt"] for entry in fingerprints.values()), "effective_env_matches_prepared_kwargs": kwargs_match, "trace_steps_match_receipt": len(rows)==receipt.get("steps"), "completed_binary_sensor_grids_all_match": trace_audit["completed_binary_sensor_grids_all_match"], "trace_boundary_identities_all_pass": all(entry.get("passed") for entry in trace_audit["boundary_observation_identities"].values()) if rows else None, "legacy_validator_counts": dict(Counter(check.status for check in checks)), "scientific_note": "Finite output, coherent SEA identities and completed duration are numerical/runtime results. MTP drift, raw and served tracking, biological reserve demand, contact morphology and diagnostic equilibrium fallbacks remain distinct scientific limitations."}
    preservation = {"AB07_original_helper": {"path": str(transfer.AB07_HELPER.relative_to(REPO)), "sha256": h.sha256(transfer.AB07_HELPER)}, "AB07_transfer_helper": {"path": str(TRANSFER_HELPER.relative_to(REPO)), "sha256": h.sha256(TRANSFER_HELPER), "matches_frozen_sha256": True}}
    result = {"created_utc": datetime.now(timezone.utc).isoformat(), "subject": args.subject, "run": str(run.relative_to(REPO)), "script_sha256": h.sha256(Path(__file__)), "receipt": {"path": str(receipt_path.relative_to(REPO)), "sha256": h.sha256(receipt_path)}, "prepared_receipt": {"path": str(prepared_path.relative_to(REPO)), "sha256": h.sha256(prepared_path)}, "summary": summary, "episode": episode, "recording_coverage": {"requested_duration_s": episode["end_s"]-episode["start_s"], "first_saved_preintegration_time_s": float(anchor[0]) if len(anchor) else None, "last_saved_preintegration_time_s": float(anchor[-1]) if len(anchor) else None, "saved_grid_end_exclusive_s": saved_end, "logical_completed_boundary_s": final, "physical_runner_time_s": physical, "native_state_time_s": native, "saved_end_minus_runner_s": saved_end-physical if saved_end is not None and physical is not None else None, "note": "A saved pre-integration sample does not prove its following RK4 step completed; an early terminal segment can extend the recorded physical prefix beyond the last completed policy boundary."}, "effective_simulator_config": snapshot, "effective_env_config": env_cfg, "source_fingerprints": fingerprints, "output_table_audit": table_audit, "tracking": tracking, "SEA": sea, "MTP_separate": {"model_coordinate_metadata": mtp_metadata, "tracking": {name: value for name, value in tracking["coordinates"].items() if "mtp" in name.lower()}}, "reserves_and_recruitment": reserves, "high_rate_contact": transfer.high_rate_contact_audit(tables, cfg, env_cfg, native_summary, h), "trace_clocks_and_BW": trace_audit, "event_timestamps": events, "legacy_validator": {"path": "validation/validate_sim_results.py", "sha256": h.sha256(REPO/"validation/validate_sim_results.py"), "checks": [asdict(check) for check in checks]}, "reused_helpers": preservation, "reporting_precision": {"output_default_significant_digits": precision, "runner_read_method": "simulation_runner.py:1871 float(output.getValueAsString(state))", "STO_writer": "output.py:86 .8f", "angle_derivative_bound": "Original helper pointwise 8-significant-digit serialization bound plus two STO half quanta; no tolerance change."}, "scope": "Saved-data audit only. No simulation, policy inference, environment, model or detector constructed. No frozen input or previous audit modifications."}
    summary["all_sto_dt_matches_captured_config"] = all(table.get("sample_dt_matches_receipt", False) for table in table_audit.values()) if len(anchor)>=2 else None
    summary["receipt_error"] = receipt.get("error")
    summary["last_returned_policy_boundary_s"] = float(rows[-1]["time"]) if rows else None
    summary["returned_policy_trace_covers_requested_episode"] = trace_audit["covers_requested_segment_end_times"]
    result["final_boundary_diagnosis"] = final_boundary_diagnosis(receipt, rows, env_cfg, h)
    result["high_rate_contact"]["native_segment_end_summary_source"] = "Maximum of available saved trace left.penetration values; the prescribed qualifier does not emit a rollout_eval summary. Failed callbacks can leave fewer saved boundary samples than physical STO samples."
    result["numerical_review_provenance"] = {"path": str(provenance_path.relative_to(REPO)), "sha256": h.sha256(provenance_path), "prepared_receipt_matches_review_sha256": True, "all_review_sources_unchanged": True}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    print(json.dumps({"output": str(output), "summary": summary}, indent=2))


if __name__ == "__main__":
    main()
