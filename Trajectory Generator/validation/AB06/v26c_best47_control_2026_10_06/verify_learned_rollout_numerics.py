"""Read-only audit of one finalized AB06 learned deterministic rollout.

Consumes the observational receipt from the rollout harness and the native
rollout_eval artifacts. No environment, policy, or simulator is constructed.
The established AB07 numerical helpers are imported without changing them.
"""

from __future__ import annotations

import argparse
from collections import Counter
import csv
from dataclasses import asdict, fields, replace
from datetime import datetime, timezone
import importlib.util
import json
import math
from pathlib import Path
import sys
from xml.etree import ElementTree as ET

import numpy as np


REPO = Path(__file__).resolve().parents[4]
ARTIFACT_DIR = Path(__file__).resolve().parent
AB07_HELPER = REPO / "Trajectory Generator" / "validation" / "AB07" / "prescribed_ground_registered_2026_10_06" / "verify_sto_numerics.py"


def read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def load_helpers():
    spec = importlib.util.spec_from_file_location("ab07_numerical_helpers", AB07_HELPER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    reference = read_json(AB07_HELPER.parent / "numerical_verification.json")
    current = module.sha256(AB07_HELPER)
    if current != reference["diagnostic_script_sha256"]:
        raise ValueError("AB07 helper no longer matches its finalized audited source hash.")
    return module


def finite_numeric_paths(value, prefix="") -> list[str]:
    bad = []
    if isinstance(value, dict):
        for key, item in value.items():
            bad.extend(finite_numeric_paths(item, f"{prefix}.{key}" if prefix else str(key)))
    elif isinstance(value, list):
        for index, item in enumerate(value):
            bad.extend(finite_numeric_paths(item, f"{prefix}[{index}]"))
    elif isinstance(value, (float, int)) and not isinstance(value, bool) and not math.isfinite(value):
        bad.append(prefix)
    return bad


def receipt_time(receipt: dict, names: tuple[str, ...]):
    for name in names:
        if name in receipt and receipt[name] is not None:
            return float(receipt[name]), f"receipt.{name}"
    return None, None


def episode_bounds(receipt: dict, reset: dict, cfg, env_cfg: dict) -> dict:
    start, start_source = receipt_time(receipt, ("actual_episode_start_s", "episode_start_s", "episode_start_time_s", "episode_start"))
    if start is None:
        start = float(reset["time"])
        start_source = "rollout_reset_diagnostics.time"
    end, end_source = receipt_time(receipt, ("actual_episode_end_s", "episode_end_s", "episode_end_time_s", "episode_end"))
    if end is None:
        duration = env_cfg["episode_duration"]
        end = float(cfg.t_end) if duration is None else min(float(cfg.t_end), start + float(duration))
        end_source = "min(effective_simulator_config.t_end, recorded_reset_time + effective_env_config.episode_duration), as in _initialise_episode"
    final, final_source = receipt_time(receipt, ("final_time_s", "final_time", "actual_final_time_s"))
    return {"start_s": start, "end_s": end, "start_source": start_source, "end_source": end_source, "final_time_s": final, "final_time_source": final_source}


def rename_raw_tracking(tracking: dict) -> dict:
    names = {
        "actual_minus_new_raw_ik": "actual_minus_raw_ik",
        "new_raw_ik_reference": "raw_ik_reference",
        "served_minus_new_raw_ik": "served_minus_raw_ik",
    }
    for entry in tracking["coordinates"].values():
        for old, new in names.items():
            if old in entry:
                entry[new] = entry.pop(old)
    return tracking


def event_time_audit(rollout: Path, prefix: str, trace: list, episode: dict, cfg) -> dict:
    result = {"csv": {}, "policy_trace_online_events": {}}
    tolerance = 1e-7
    for path in sorted((rollout / "sim_outputs").glob(f"{prefix}_gait_events*.csv")):
        with path.open(encoding="utf-8", newline="") as stream:
            rows = list(csv.DictReader(stream))
        reference_only = path.stem.endswith("_gait_events")
        start = cfg.t_start if reference_only else episode["start_s"]
        end = cfg.t_end if reference_only else episode["end_s"]
        violations = []
        nonfinite = []
        confirmed_before_event = []
        per_side_times = {}
        for index, row in enumerate(rows):
            for name in ("time", "confirmed_time", "cycle_start", "cycle_end"):
                if row.get(name) in (None, ""):
                    continue
                value = float(row[name])
                if not math.isfinite(value):
                    nonfinite.append({"row": index, "field": name})
                elif value < start - tolerance or value > end + tolerance:
                    violations.append({"row": index, "field": name, "time_s": value})
            if row.get("time") and row.get("confirmed_time"):
                if float(row["confirmed_time"]) < float(row["time"]) - tolerance:
                    confirmed_before_event.append(index)
            if row.get("time"):
                per_side_times.setdefault(row.get("side", "unknown"), []).append(float(row["time"]))
        result["csv"][path.name] = {
            "rows": len(rows), "role": "prescribed GRF reference cycles over full setup window" if reference_only else "recorded online GRF threshold events, independent from the learned active phase-FSM contract",
            "checked_time_domain_s": [start, end],
            "out_of_time_domain": violations, "nonfinite_timestamp_fields": nonfinite,
            "confirmation_before_event_rows": confirmed_before_event,
            "time_nondecreasing_per_side": {side: bool(np.all(np.diff(times) >= -tolerance)) for side, times in per_side_times.items()},
        }
    rows = []
    nonfinite = []
    outside = []
    confirmation_before = []
    trace_time = np.asarray([float(item["time"]) for item in trace])
    for item in trace:
        for event in item.get("online_events", []):
            if not isinstance(event, dict):
                continue
            row = {"policy_step": item.get("step"), "segment_end_s": float(item["time"]), "event": event}
            rows.append(row)
            for key in ("time", "time_s", "confirmed_time", "confirmed_time_s", "event_time_s"):
                if event.get(key) is None:
                    continue
                value = float(event[key])
                if not math.isfinite(value):
                    nonfinite.append({"policy_step": item.get("step"), "field": key})
                elif value < episode["start_s"] - tolerance or value > episode["end_s"] + tolerance:
                    outside.append({"policy_step": item.get("step"), "field": key, "time_s": value})
            physical = event.get("time", event.get("time_s", event.get("event_time_s")))
            confirmed = event.get("confirmed_time", event.get("confirmed_time_s"))
            if physical is not None and confirmed is not None and float(confirmed) < float(physical) - tolerance:
                confirmation_before.append(item.get("step"))
    result["policy_trace_online_events"] = {
        "role": "Events delivered by the active environment/FSM in each recorded policy segment; kept separate from threshold CSV events and prescribed reference cycles.",
        "rows": rows, "event_count": len(rows), "nonfinite_timestamp_fields": nonfinite,
        "out_of_episode_domain": outside, "confirmation_before_event_policy_steps": confirmation_before,
        "trace_rows": len(trace), "trace_times_strictly_increasing": bool(np.all(np.diff(trace_time) > 0)),
        "trace_time_start_s": float(trace_time[0]), "trace_time_end_s": float(trace_time[-1]),
    }
    return result


def policy_trace_audit(trace: list, summary: dict, helpers, receipt: dict, episode: dict, env_cfg: dict) -> dict:
    raw = np.asarray([item["raw_policy_action"] for item in trace], dtype=float)
    applied = np.asarray([item["applied_policy_action"] for item in trace], dtype=float)
    clipped = np.abs(raw - applied) > 1e-7
    fields_to_check = ("raw_policy_action", "applied_policy_action", "actor_observation_vector_before", "policy_segment_times", "policy_segment_values", "policy_segment_derivatives", "prosthetic_state", "imitation_target_q", "imitation_target_qdot", "reward_terms")
    bad = []
    for index, item in enumerate(trace):
        for name in fields_to_check:
            bad.extend(finite_numeric_paths(item.get(name), f"trace[{index}].{name}"))
    segment_dt = float(env_cfg["segment_duration"])
    expected_steps = int(round((episode["end_s"] - episode["start_s"]) / segment_dt))
    times = np.asarray([float(item["time"]) for item in trace])
    expected_times = episode["start_s"] + np.arange(1, expected_steps + 1) * segment_dt
    actor_names = receipt["actor_feature_names"]
    return {
        "rows": len(trace), "numeric_nonfinite_paths": bad,
        "checked_numeric_fields": list(fields_to_check), "raw_action": helpers.metrics(raw),
        "applied_action": helpers.metrics(applied),
        "raw_minus_applied": helpers.metrics(raw - applied),
        "clipped_policy_steps": int(np.count_nonzero(np.any(clipped, axis=1))),
        "clipped_action_values": int(np.count_nonzero(clipped)),
        "clipped_action_fraction": float(np.mean(clipped)),
        "rollout_reported_clipped_steps": summary.get("action_clipped_steps"),
        "rollout_reported_clipped_fraction": summary.get("action_clipped_fraction"),
        "segment_dt_s": segment_dt, "expected_policy_segments": expected_steps,
        "covers_requested_segment_end_times": bool(len(times) == expected_steps and np.allclose(times, expected_times, atol=1e-7, rtol=0)),
        "actor_observation_vectors_have_captured_width": all(len(item["actor_observation_vector_before"]) == len(actor_names) for item in trace),
        "actor_observation_maps_preserve_captured_feature_order": all(list(item["actor_observation_before"]) == actor_names for item in trace),
        "clipping_note": "Offline raw-minus-applied comparison uses 1e-7 to exclude float32 wrapper rounding; native rollout_summary clipping counters retain the rollout_eval implementation's own criterion.",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rollout-dir", type=Path, required=True)
    parser.add_argument("--receipt", type=Path)
    parser.add_argument("--output", type=Path, default=ARTIFACT_DIR / "numerical_verification.json")
    args = parser.parse_args()
    rollout = args.rollout_dir.resolve()
    receipt_path = (args.receipt or rollout / "receipt.json").resolve()
    receipt = read_json(receipt_path)
    summary_path = rollout / "rollout_summary.json"
    if receipt.get("status") in {"prepared", "running", "started"} or not summary_path.is_file():
        parser.error("Rollout/receipt not finalized; wait for the authorized run to close its outputs.")
    summary = read_json(summary_path)
    trace = read_json(rollout / "rollout_policy_trace.json")
    reset = read_json(rollout / "rollout_reset_diagnostics.json")
    if not trace:
        raise ValueError("Policy trace is empty; actual coverage cannot be verified.")
    for path in (REPO, REPO / "Trajectory Generator" / "baseline_MLP"):
        sys.path.insert(0, str(path))
    import win_runtime  # noqa: F401: existing Windows OpenSim load-order shim
    from config import SimulatorConfig
    from validation import validate_sim_results as validator
    import opensim

    helpers = load_helpers()
    provenance_path = ARTIFACT_DIR / "audit_provenance.json"
    provenance = read_json(provenance_path)
    ab07_preservation = {
        name: {"expected_sha256": expected, "current_sha256": helpers.sha256(AB07_HELPER.parent / name), "unchanged": helpers.sha256(AB07_HELPER.parent / name) == expected}
        for name, expected in provenance["reused_AB07_files"].items()
    }
    if not all(item["unchanged"] for item in ab07_preservation.values()):
        raise ValueError("A preserved AB07 audit artifact changed before this analysis.")
    snapshot = receipt["effective_simulator_config"]
    expected_keys = {field.name for field in fields(SimulatorConfig)}
    if set(snapshot) != expected_keys:
        raise ValueError(f"Captured simulator snapshot incomplete: missing={sorted(expected_keys - set(snapshot))}, unknown={sorted(set(snapshot) - expected_keys)}")
    cfg = SimulatorConfig(**snapshot)
    env_cfg = receipt["effective_env_config"]
    episode = episode_bounds(receipt, reset, cfg, env_cfg)
    coverage_cfg = replace(cfg, t_start=episode["start_s"], t_end=episode["end_s"])
    prefix = cfg.output_prefix
    outputs = rollout / "sim_outputs"
    tables = validator.load_required_tables(outputs, prefix)
    for suffix in ("kinematics_reference", "reserve_controls", "recruitment", "so_torque_diagnostics"):
        tables[suffix] = validator.load_table(outputs / f"{prefix}_{suffix}.sto")
    for path in sorted(outputs.glob(f"{prefix}_*.sto")):
        suffix = path.stem.removeprefix(f"{prefix}_")
        if suffix not in tables:
            tables[suffix] = validator.load_table(path)
    missing = [name for name, table in tables.items() if table is None]
    if missing:
        raise ValueError(f"Required output tables missing: {missing}")
    fingerprints = {}
    source_entries = receipt.get("source_fingerprints", receipt.get("source_files"))
    if not source_entries:
        raise ValueError("The observational receipt does not contain source fingerprints.")
    for name, entry in source_entries.items():
        path = helpers.resolve_input(entry["path"])
        actual_hash = helpers.sha256(path)
        fingerprints[name] = {"path": str(path.relative_to(REPO)), "receipt_sha256": entry["sha256"], "current_sha256": actual_hash, "matches_receipt": actual_hash == entry["sha256"]}
    model_path = helpers.resolve_input(snapshot["model_file"])
    raw_path = helpers.resolve_input(snapshot["kinematics_file"])
    raw = validator.load_table(raw_path)
    props = helpers.xml_sea_properties(model_path, [cfg.sea_knee_name, cfg.sea_ankle_name], validator)
    precision = int(opensim.CoordinateActuator().getOutput("actuation").getNumberOfSignificantDigits())
    if precision != 8:
        raise ValueError("Observed output default differs from the validated 8-digit reporting method.")
    checks = []
    validator.validate_run_status(outputs, prefix, checks)
    validator.validate_sea_tautology(cfg, tables, props, checks)
    validator.validate_reserves(cfg, tables, checks)
    validator.validate_tracking(cfg, tables, raw, checks)
    validator.validate_power_and_controls(cfg, tables, checks)
    validator.validate_sea_derivatives(cfg, tables, checks)
    validator.validate_sea_interface_diagnostics(cfg, tables, checks)
    table_audit = helpers.audit_tables(tables, coverage_cfg, tables["kinematics"].time)
    tracking = rename_raw_tracking(helpers.tracking_audit(cfg, tables, raw, validator))
    sea = helpers.sea_audit(cfg, tables, props, precision)
    reserves = helpers.reserves_and_recruitment(cfg, tables)
    policy = policy_trace_audit(trace, summary, helpers, receipt, episode, env_cfg)
    events = event_time_audit(rollout, prefix, trace, episode, cfg)
    mtp_metadata = {}
    for element in ET.parse(model_path).getroot().iter():
        name = element.attrib.get("name", "")
        if validator.xml_local_name(element.tag) == "Coordinate" and "mtp" in name.lower():
            mtp_metadata[name] = {validator.xml_local_name(child.tag): (child.text or "").strip() for child in element if validator.xml_local_name(child.tag) in {"locked", "clamped", "range", "default_value"}}
    online_event_timestamp_issues = sum(len(item["out_of_time_domain"]) + len(item["nonfinite_timestamp_fields"]) + len(item["confirmation_before_event_rows"]) for item in events["csv"].values() if not item["role"].startswith("prescribed"))
    active_event_audit = events["policy_trace_online_events"]
    online_event_timestamp_issues += len(active_event_audit["nonfinite_timestamp_fields"]) + len(active_event_audit["out_of_episode_domain"]) + len(active_event_audit["confirmation_before_event_policy_steps"])
    final_time = float(trace[-1]["time"])
    completed = bool(summary.get("ok") and not summary.get("terminated") and summary.get("end_reason") in {"episode_time_limit", "dataset_end"} and abs(final_time - episode["end_s"]) < 1e-7)
    numeric_summary = {
        "rollout_script_ok": summary.get("ok"), "completed_requested_episode": completed,
        "end_reason": summary.get("end_reason"), "terminated": summary.get("terminated"), "truncated": summary.get("truncated"),
        "output_rows": len(tables["kinematics"].time), "expected_episode_duration_s": episode["end_s"] - episode["start_s"],
        "all_sto_finite": all(item["all_values_finite"] for item in table_audit.values()),
        "all_sto_same_time_grid": all(item["same_time_grid_as_kinematics"] for item in table_audit.values()),
        "all_sto_dt_matches_captured_config": all(item["sample_dt_matches_receipt"] for item in table_audit.values()),
        "all_sto_cover_requested_episode_samples": all(item["covers_requested_preintegration_samples"] for item in table_audit.values()),
        "source_fingerprints_all_match_receipt": all(item["matches_receipt"] for item in fingerprints.values()),
        "all_sea_identity_checks_pass": all(item["all_identity_checks_pass"] for item in sea.values()),
        "policy_trace_numeric_values_finite": not policy["numeric_nonfinite_paths"],
        "rollout_summary_nonfinite_paths": finite_numeric_paths(summary),
        "policy_steps_match_summary": len(trace) == int(summary["steps"]),
        "feature_order_matches_historical": receipt.get("feature_order_matches_historical"),
        "summary_actor_feature_order_matches_receipt": summary.get("actor_feature_names") == receipt.get("actor_feature_names"),
        "summary_full_feature_order_matches_receipt": summary.get("observation_feature_names") == receipt.get("observation_feature_names"),
        "policy_trace_covers_requested_segment_end_times": policy["covers_requested_segment_end_times"],
        "active_and_threshold_event_timestamp_issue_count": online_event_timestamp_issues,
        "trace_final_time_matches_receipt": episode["final_time_s"] is None or abs(final_time - episode["final_time_s"]) < 1e-7,
        "legacy_validator_counts": dict(Counter(check.status for check in checks)),
        "scientific_note": "Finiteness, consistent SEA dynamics and completed duration are runtime checks. Raw IK tracking, MTP drift, biological reserve demand and gait-event quality remain distinct scientific questions; generic validator tracking thresholds are not a subject-specific acceptance criterion.",
    }
    result = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "rollout": str(rollout.relative_to(REPO)), "diagnostic_script_sha256": helpers.sha256(Path(__file__)),
        "receipt_path": str(receipt_path.relative_to(REPO)), "receipt_sha256": helpers.sha256(receipt_path), "receipt_status": receipt.get("status"),
        "episode": episode, "trace_final_time_s": final_time, "summary": numeric_summary,
        "effective_simulator_config": snapshot, "effective_env_config": env_cfg,
        "captured_policy_contract": {"checkpoint": receipt.get("checkpoint"), "checkpoint_logical_iteration": receipt.get("checkpoint_logical_iteration"), "action_selection": receipt.get("action_selection"), "seed": receipt.get("seed"), "actor_feature_names": receipt.get("actor_feature_names"), "observation_feature_names": receipt.get("observation_feature_names"), "feature_order_matches_historical": receipt.get("feature_order_matches_historical")},
        "source_fingerprints": fingerprints, "output_table_audit": table_audit,
        "raw_ik": {"path": str(raw_path.relative_to(REPO)), "sha256": helpers.sha256(raw_path), "in_degrees": raw.in_degrees},
        "tracking": tracking, "SEA": sea,
        "MTP_separate": {"model_coordinate_metadata": mtp_metadata, "tracking": {name: entry for name, entry in tracking["coordinates"].items() if "mtp" in name.lower()}},
        "reserves_and_recruitment": reserves, "policy_trace": policy, "event_timestamps": events,
        "rollout_summary": summary, "reset_diagnostics": reset,
        "reused_helpers": {"path": str(AB07_HELPER.relative_to(REPO)), "sha256": helpers.sha256(AB07_HELPER), "AB07_audit_unchanged": True, "preserved_AB07_artifacts": ab07_preservation},
        "audit_provenance": {"path": str(provenance_path.relative_to(REPO)), "sha256": helpers.sha256(provenance_path)},
        "reused_validator": {"path": "validation/validate_sim_results.py", "sha256": helpers.sha256(REPO / "validation" / "validate_sim_results.py"), "checks": [asdict(check) for check in checks]},
        "reporting_precision": {"opensim_version": opensim.GetVersionAndDate(), "output_default_significant_digits": precision, "runner_read_method": "simulation_runner.py:1871: float(output.getValueAsString(state))", "sto_writer": "output.py:86: .8f", "angle_derivative_bound": "per sample: string half quantum at 8 significant digits plus two STO half quanta, 1e-8 total, and floating-point margin"},
        "scope": "Read-only analysis of a finalized learned deterministic AB06 rollout. No environment or policy constructed, no integration or training, no operational file modifications, no AB07 audit modifications.",
        "coverage_note": "The captured SimulatorConfig retains the full dataset bounds. Coverage uses actual episode start/end and pre-integration recording, so a completed 5 s episode has 5000 samples with the final sample at episode_end - integration_dt.",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + ".tmp")
    temporary.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(args.output)
    print(json.dumps({"output": str(args.output), "summary": numeric_summary}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
