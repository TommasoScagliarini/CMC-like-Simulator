"""Read-only audit of one finalized AB07 learned deterministic transfer rollout.

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


def load_table_allow_empty(path: Path, validator):
    if not path.is_file():
        return None
    rows = None
    in_degrees = False
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            stripped = line.strip()
            if stripped.lower().startswith("nrows="):
                rows = int(stripped.split("=", 1)[1])
            elif stripped.lower().startswith("indegrees="):
                in_degrees = stripped.split("=", 1)[1].lower() == "yes"
            elif stripped.lower() == "endheader":
                break
        columns = stream.readline().strip().split("\t")[1:]
    if rows == 0:
        return validator.Table(path, np.empty(0), columns, np.empty((0, len(columns))), in_degrees)
    return validator.load_table(path)


def partial_table_audit(tables: dict, cfg, anchor_time: np.ndarray, helpers) -> dict:
    result = {}
    for suffix, table in tables.items():
        if table is None:
            result[suffix] = {"present": False, "has_recorded_data": False}
            continue
        if len(table.time) >= 2:
            result[suffix] = helpers.audit_tables({suffix: table}, cfg, anchor_time)[suffix]
        else:
            result[suffix] = {
                "present": True, "file": str(table.path.relative_to(REPO)), "sha256": helpers.sha256(table.path),
                "rows": len(table.time), "columns": len(table.columns), "in_degrees": table.in_degrees,
                "all_values_finite": bool(np.all(np.isfinite(table.data))),
                "time_finite": bool(np.all(np.isfinite(table.time))),
                "time_start_s": float(table.time[0]) if len(table.time) else None,
                "time_end_s": float(table.time[-1]) if len(table.time) else None,
                "same_time_grid_as_kinematics": bool(np.array_equal(table.time, anchor_time)),
                "sample_dt_matches_receipt": None, "covers_requested_preintegration_samples": False,
                "nonfinite_columns": {name: {"count": int(np.count_nonzero(~np.isfinite(table.data[:, table.index[name]])))} for name in table.columns if np.any(~np.isfinite(table.data[:, table.index[name]]))},
            }
        expected_prefix = cfg.t_start + np.arange(len(table.time)) * cfg.integration_dt
        result[suffix]["has_recorded_data"] = len(table.time) > 0
        result[suffix]["matches_contiguous_recorded_prefix"] = bool(len(table.time) and np.allclose(table.time, expected_prefix, atol=1e-8, rtol=0))
    return result


def actor_clock_audit(trace: list, env_cfg: dict, helpers) -> dict:
    if not trace:
        return {"available": False, "all_clock_identity_checks_pass": None}
    slots = ("gait_phase_sin", "gait_phase_cos", "online_left_gait_phase_sin", "online_left_gait_phase_cos", "online_left_cycle_duration_s", "phase_fsm_wait_hs", "phase_fsm_stance_after_hs", "phase_fsm_swing_after_to", "phase_expected_hs", "phase_expected_to", "phase_stance_elapsed_norm", "phase_swing_elapsed_norm", "phase_cycle_progress_credit")
    values = {name: np.asarray([float(row["actor_observation_before"][name]) for row in trace]) for name in slots}
    identities = {}
    if not env_cfg["gait_clock_enable"]:
        identities["disabled_reference_clock_sin_zero"] = helpers.identity(values["gait_phase_sin"], np.zeros(len(trace)), 1e-7, "dimensionless")
        identities["disabled_reference_clock_cos_one"] = helpers.identity(values["gait_phase_cos"], np.ones(len(trace)), 1e-7, "dimensionless")
    for prefix in ("gait_phase", "online_left_gait_phase"):
        identities[f"{prefix}_unit_circle"] = helpers.identity(values[f"{prefix}_sin"] ** 2 + values[f"{prefix}_cos"] ** 2, np.ones(len(trace)), 2e-7, "dimensionless")
    if len(trace) > 1:
        previous = [row["phase_fsm"] for row in trace[:-1]]
        stance = np.asarray([float(payload["stance_elapsed_s"]) for payload in previous])
        swing = np.asarray([float(payload["swing_elapsed_s"]) for payload in previous])
        identities["stance_age_slot_matches_previous_fsm_boundary"] = helpers.identity(values["phase_stance_elapsed_norm"][1:], np.clip(stance / max(1e-9, env_cfg["phase_stance_hard_timeout_s"]), 0, 1.5), 1e-7, "dimensionless")
        identities["swing_age_slot_matches_previous_fsm_boundary"] = helpers.identity(values["phase_swing_elapsed_norm"][1:], np.clip(swing / max(1e-9, env_cfg["phase_swing_hard_timeout_s"]), 0, 1.5), 1e-7, "dimensionless")
        for name, state_id in (("phase_fsm_wait_hs", 0), ("phase_fsm_stance_after_hs", 1), ("phase_fsm_swing_after_to", 2)):
            expected = np.asarray([float(int(payload["state_id"]) == state_id) for payload in previous])
            identities[f"{name}_matches_previous_fsm_boundary"] = helpers.identity(values[name][1:], expected, 0.0, "boolean")
        for name, expected_event in (("phase_expected_hs", "heel_strike"), ("phase_expected_to", "toe_off")):
            expected = np.asarray([float(payload["expected_next_event"] == expected_event) for payload in previous])
            identities[f"{name}_matches_previous_fsm_boundary"] = helpers.identity(values[name][1:], expected, 0.0, "boolean")
    return {
        "slots": {name: helpers.metrics(value) for name, value in values.items()},
        "identities": identities, "all_clock_identity_checks_pass": all(item["passed"] for item in identities.values()),
        "units": "Elapsed actor slots are dimensionless seconds divided by configured stance/swing hard timeouts and clipped to [0,1.5]; cycle duration is seconds; sine/cosine and flags are dimensionless.",
        "causality": "actor_observation_before at policy step i is checked against phase_fsm after step i-1. The first actor observation has no prior phase payload in this trace and is not assigned an invented reset state.",
        "source": "prosthetic_phase_fsm.py:1872 observation; osim_trj_cmc_like.py:2981 reference clock and :3048 phase observation",
    }


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
    physical, physical_source = receipt_time(receipt, ("final_runner_time_s", "physical_runner_current_time_s", "runner_final_time_s", "runner_current_time_s", "physical_final_time_s", "actual_physical_final_time_s"))
    return {"start_s": start, "end_s": end, "start_source": start_source, "end_source": end_source, "final_time_s": final, "final_time_source": final_source, "physical_runner_time_s": physical, "physical_runner_time_source": physical_source, "final_native_state_time_s": receipt.get("final_state_time_s"), "terminal_contact": receipt.get("terminal_contact"), "final_failure": receipt.get("final_failure")}


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
        "trace_time_start_s": float(trace_time[0]) if len(trace_time) else None, "trace_time_end_s": float(trace_time[-1]) if len(trace_time) else None,
    }
    return result


def policy_trace_audit(trace: list, summary: dict, helpers, receipt: dict, episode: dict, env_cfg: dict) -> dict:
    if not trace:
        return {"rows": 0, "available": False, "numeric_nonfinite_paths": [], "covers_requested_segment_end_times": False, "covers_completed_segment_end_times": None, "completed_policy_segments": 0, "incomplete_policy_attempts": []}
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
    completed_mask = np.asarray([float(row.get("reward_terms", {}).get("segment_completed", 1.0)) > 0.5 for row in trace])
    completed_times = times[completed_mask]
    expected_completed_times = episode["start_s"] + np.arange(1, len(completed_times) + 1) * segment_dt
    return {
        "rows": len(trace), "numeric_nonfinite_paths": bad,
        "checked_numeric_fields": list(fields_to_check), "raw_action": helpers.metrics(raw),
        "applied_action": helpers.metrics(applied),
        "raw_minus_applied": helpers.metrics(raw - applied),
        "clipped_policy_steps": int(np.count_nonzero(np.any(clipped, axis=1))),
        "clipped_action_values": int(np.count_nonzero(clipped)),
        "clipped_action_fraction": float(np.mean(clipped)),
        "clipped_records": [{"policy_step": int(row["step"]), "segment_end_s": float(row["time"]), "raw_policy_action": raw[index].tolist(), "applied_policy_action": applied[index].tolist(), "max_abs_raw_minus_applied": float(np.max(np.abs(raw[index] - applied[index])))} for index, row in enumerate(trace) if np.any(clipped[index])],
        "rollout_reported_clipped_steps": summary.get("action_clipped_steps"),
        "rollout_reported_clipped_fraction": summary.get("action_clipped_fraction"),
        "segment_dt_s": segment_dt, "expected_policy_segments": expected_steps,
        "covers_requested_segment_end_times": bool(len(times) == expected_steps and np.allclose(times, expected_times, atol=1e-7, rtol=0)),
        "completed_policy_segments": int(np.count_nonzero(completed_mask)),
        "incomplete_policy_attempts": [int(row["step"]) for row, completed in zip(trace, completed_mask) if not completed],
        "covers_completed_segment_end_times": bool(np.allclose(completed_times, expected_completed_times, atol=1e-7, rtol=0)),
        "terminal_logical_time_repeats_previous_boundary": bool(len(times) > 1 and not completed_mask[-1] and abs(times[-1] - times[-2]) < 1e-7),
        "actor_observation_vectors_have_captured_width": all(len(item["actor_observation_vector_before"]) == len(actor_names) for item in trace),
        "actor_observation_maps_preserve_captured_feature_order": all(list(item["actor_observation_before"]) == actor_names for item in trace),
        "clipping_note": "Offline raw-minus-applied comparison uses 1e-7 to exclude float32 wrapper rounding; native rollout_summary clipping counters retain the rollout_eval implementation's own criterion.",
    }


def command_limit_audit(cfg, tables: dict, sea: dict, helpers) -> None:
    """Add command-limit evidence separately from plugin motor/PI clamps."""
    time = tables["kinematics"].time
    for name, entry in sea.items():
        coord = entry["coordinate"]
        u = helpers.series_on(tables["sea_controls"], coord, time)
        tau_raw = helpers.series_on(tables["sea_diagnostics"], f"{name}_tau_cmd_raw", time)
        u_unclipped = tau_raw / entry["model_properties"]["F_opt"]
        excess = np.maximum(np.abs(u_unclipped) - 1.0, 0.0)
        clamp = np.abs(u_unclipped) > 1.0 + 1e-8
        entry["command_limit_evidence"] = {
            "near_saturation_definition": "abs(recorded applied u) >= 0.999; a command threshold, not a motor torque clamp",
            "exactly_at_plus_or_minus_one": helpers.mask_stats(np.abs(u) == 1.0, cfg.integration_dt),
            "u_unclipped": helpers.metrics(u_unclipped),
            "unclipped_command_above_unit_bound": helpers.mask_stats(clamp, cfg.integration_dt),
            "u_unclipped_minus_applied": helpers.metrics(u_unclipped - u),
            "unclipped_abs_excess_above_one": helpers.metrics(excess),
            "first_command_clipping_sample_s": float(time[np.flatnonzero(clamp)[0]]) if np.any(clamp) else None,
            "last_command_clipping_sample_s": float(time[np.flatnonzero(clamp)[-1]]) if np.any(clamp) else None,
            "source": "u_unclipped reconstructed from recorded tau_cmd_raw / model F_opt; prosthesis_controller.py:193-198 computes this ratio, clips to [-1,1], then applies the captured u LPF (disabled here). This differs from clipping the neural policy action.",
        }
        for key in ("cascade_i_clamped", "cascade_anti_windup_active"):
            flag = helpers.series_on(tables["sea_diagnostics"], f"{name}_{key}", time)
            entry["command_limit_evidence"][key] = helpers.mask_stats(flag > 0.5, cfg.integration_dt)


def high_rate_contact_audit(tables: dict, cfg, env_cfg: dict, summary: dict, helpers) -> dict:
    table = tables.get("online_grf")
    if table is None or not len(table.time):
        return {"available": False}
    threshold = float(env_cfg["grf_penetration_termination_m"])
    result = {"available": True, "sample_dt_s": cfg.integration_dt, "samples": len(table.time), "applied_online_grf_sides": list(cfg.online_grf_applied_sides), "env_termination_threshold_m": threshold, "per_side": {}, "native_segment_end_summary_max_penetration_m": summary.get("grf_penetration_max_m"), "native_segment_end_summary_samples": summary.get("grf_penetration_samples"), "source": "1 ms pre-integration online_grf STO. The environment guard and native rollout summary use applied online-GRF sides (osim_trj_cmc_like.py:4406); channels for other sides are kept separate. Sampled maxima do not certify continuous or RK4-stage maxima."}
    for side in ("left", "right"):
        values = table.data[:, table.index[f"{side}_penetration"]]
        maximum_index = int(np.argmax(values))
        result["per_side"][side] = {"applied_online_grf": side in cfg.online_grf_applied_sides, "penetration_m": helpers.metrics(values), "maximum_sample_time_s": float(table.time[maximum_index]), "above_env_threshold": helpers.mask_stats(values > threshold, cfg.integration_dt)}
    return result


def runtime_diagnostics_audit(path: Path, env_cfg: dict, summary: dict, helpers) -> dict:
    if not path.is_file():
        return {"available": False}
    dt = float(env_cfg["detector_sample_dt_s"])
    segment_dt = float(env_cfg["segment_duration"])
    expected_samples = int(round(segment_dt / dt))
    rows = []
    rewards = []
    bad = []
    completed_sensor_grid_failures = []
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            row = json.loads(line)
            step = int(row["diagnostic_step"])
            samples = row.get("binary_phase_sensor_samples", [])
            times = np.asarray([float(sample["time_s"]) for sample in samples])
            completed = bool(row["segment_completed"])
            expected = float(row["time"]) - segment_dt + np.arange(1, expected_samples + 1) * dt
            grid_matches = bool(len(times) == expected_samples and np.allclose(times, expected, atol=1e-8, rtol=0)) if completed else None
            if completed and not grid_matches:
                completed_sensor_grid_failures.append(step)
            bad.extend(finite_numeric_paths(samples, f"runtime_step[{step}].binary_phase_sensor_samples"))
            rewards.append(float(row["recorded_returned_reward"]))
            rows.append({"step": step, "logical_time_s": row["time"], "target_time_s": row["target_time"], "segment_completed": completed, "binary_sensor_samples": len(times), "binary_sensor_first_time_s": float(times[0]) if len(times) else None, "binary_sensor_last_time_s": float(times[-1]) if len(times) else None, "completed_sensor_grid_matches": grid_matches, "returned_reward": row["recorded_returned_reward"], "terminated": row["recorded_terminated"], "truncated": row["recorded_truncated"], "end_reason": row.get("end_reason"), "failure": row.get("failure"), "terminal_contact_time_s": (row.get("terminal_contact") or {}).get("time_s")})
    reward_sum = float(np.sum(rewards))
    return {"available": True, "path": str(path.relative_to(REPO)), "sha256": helpers.sha256(path), "steps": len(rows), "per_step": rows, "sensor_nonfinite_paths": bad, "completed_binary_sensor_grid_failure_steps": completed_sensor_grid_failures, "completed_binary_sensor_grids_all_match": not completed_sensor_grid_failures and not bad, "recorded_returned_reward_sum": reward_sum, "reward_sum_matches_native_summary": helpers.identity(np.asarray([reward_sum]), np.asarray([summary["episode_return"]]), 1e-10, "dimensionless") if "episode_return" in summary else None, "sample_count_note": "Ten 1 ms samples are required only for completed 10 ms policy segments. An interrupted segment's recorded arrays are inventoried and are not treated as a completed sensor transport batch."}


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
    if receipt.get("status") in {"prepared", "running", "started"} or (not summary_path.is_file() and receipt.get("status") != "failed"):
        parser.error("Rollout/receipt not finalized; wait for the authorized run to close its outputs.")
    summary_available = summary_path.is_file()
    summary = read_json(summary_path) if summary_available else {"ok": False, "end_reason": receipt.get("end_reason", "worker_failure"), "terminated": receipt.get("terminated"), "truncated": receipt.get("truncated"), "error": receipt.get("error")}
    trace_path = rollout / "rollout_policy_trace.json"
    reset_path = rollout / "rollout_reset_diagnostics.json"
    trace = read_json(trace_path) if trace_path.is_file() else []
    reset = read_json(reset_path) if reset_path.is_file() else {}
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
    ab06_preservation = {
        path: {"expected_sha256": expected, "current_sha256": helpers.sha256(REPO / path), "unchanged": helpers.sha256(REPO / path) == expected}
        for path, expected in provenance["preserved_AB06_files"].items()
    }
    if not all(item["unchanged"] for item in ab06_preservation.values()):
        raise ValueError("A preserved AB06 audit artifact changed before this analysis.")
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
    tables = {suffix: load_table_allow_empty(outputs / f"{prefix}_{suffix}.sto", validator) for suffix in ("sea_torques", "sea_controls", "sea_states", "sea_derivatives", "sea_diagnostics", "states", "kinematics", "reserve_torques", "power")}
    for suffix in ("kinematics_reference", "reserve_controls", "recruitment", "so_torque_diagnostics"):
        tables[suffix] = load_table_allow_empty(outputs / f"{prefix}_{suffix}.sto", validator)
    for path in sorted(outputs.glob(f"{prefix}_*.sto")):
        suffix = path.stem.removeprefix(f"{prefix}_")
        if suffix not in tables:
            tables[suffix] = load_table_allow_empty(path, validator)
    missing = [name for name, table in tables.items() if table is None]
    if missing and receipt.get("status") != "failed":
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
    anchor_time = tables["kinematics"].time if tables["kinematics"] is not None else np.empty(0)
    numeric_data_available = bool(len(anchor_time) and not missing)
    if numeric_data_available:
        validator.validate_sea_tautology(cfg, tables, props, checks)
        validator.validate_reserves(cfg, tables, checks)
        validator.validate_tracking(cfg, tables, raw, checks)
        validator.validate_power_and_controls(cfg, tables, checks)
        validator.validate_sea_derivatives(cfg, tables, checks)
        validator.validate_sea_interface_diagnostics(cfg, tables, checks)
        tracking = rename_raw_tracking(helpers.tracking_audit(cfg, tables, raw, validator))
        sea = helpers.sea_audit(cfg, tables, props, precision)
        command_limit_audit(cfg, tables, sea, helpers)
        reserves = helpers.reserves_and_recruitment(cfg, tables)
    else:
        validator.add_check(checks, "WARN", "recorded numerical data", "No complete nonempty output table set; SEA/tracking/recruitment identities were not attempted.")
        tracking = {"coordinates": {}, "available": False}
        sea = {}
        reserves = {"available": False}
    table_audit = partial_table_audit(tables, coverage_cfg, anchor_time, helpers)
    policy = policy_trace_audit(trace, summary, helpers, receipt, episode, env_cfg)
    events = event_time_audit(rollout, prefix, trace, episode, cfg)
    actor_clocks = actor_clock_audit(trace, env_cfg, helpers)
    runtime_diagnostics = runtime_diagnostics_audit(rollout / "runtime_diagnostics.jsonl", env_cfg, summary, helpers)
    mtp_metadata = {}
    for element in ET.parse(model_path).getroot().iter():
        name = element.attrib.get("name", "")
        if validator.xml_local_name(element.tag) == "Coordinate" and "mtp" in name.lower():
            mtp_metadata[name] = {validator.xml_local_name(child.tag): (child.text or "").strip() for child in element if validator.xml_local_name(child.tag) in {"locked", "clamped", "range", "default_value"}}
    online_event_timestamp_issues = sum(len(item["out_of_time_domain"]) + len(item["nonfinite_timestamp_fields"]) + len(item["confirmation_before_event_rows"]) for item in events["csv"].values() if not item["role"].startswith("prescribed"))
    active_event_audit = events["policy_trace_online_events"]
    online_event_timestamp_issues += len(active_event_audit["nonfinite_timestamp_fields"]) + len(active_event_audit["out_of_episode_domain"]) + len(active_event_audit["confirmation_before_event_policy_steps"])
    final_time = float(trace[-1]["time"]) if trace else episode["final_time_s"]
    completed = bool(summary.get("ok") and not summary.get("terminated") and summary.get("end_reason") in {"episode_time_limit", "dataset_end"} and final_time is not None and abs(final_time - episode["end_s"]) < 1e-7)
    non_tautology_checks = [check for check in checks if check.name.endswith(("tau_ref - tau_spring", "algebraic motor state", "motor_speed - qdot"))]
    recorded_end = float(anchor_time[-1] + cfg.integration_dt) if len(anchor_time) else None
    requested_rows = int(round((episode["end_s"] - episode["start_s"]) / cfg.integration_dt))
    physical_runner_time = episode["physical_runner_time_s"]
    recording_coverage = {"requested_rows": requested_rows, "recorded_rows": len(anchor_time), "start_s": float(anchor_time[0]) if len(anchor_time) else None, "last_preintegration_sample_s": float(anchor_time[-1]) if len(anchor_time) else None, "recorded_preintegration_grid_end_exclusive_s": recorded_end, "logical_completed_policy_boundary_s": final_time, "physical_runner_time_s": physical_runner_time, "native_state_time_s": episode["final_native_state_time_s"], "recorded_grid_end_minus_logical_boundary_s": recorded_end - final_time if recorded_end is not None and final_time is not None else None, "recorded_grid_end_minus_physical_runner_time_s": recorded_end - physical_runner_time if recorded_end is not None and physical_runner_time is not None else None, "recorded_rows_match_physical_runner_prefix": bool(physical_runner_time is not None and abs((physical_runner_time - episode["start_s"]) / cfg.integration_dt - len(anchor_time)) < 1e-4), "samples_at_or_after_final_logical_boundary": int(np.count_nonzero(anchor_time >= final_time - 1e-8)) if final_time is not None else None, "note": "A recorded pre-integration sample does not prove completion of its following integration step. On an interrupted RK4 segment, the logical policy boundary can remain earlier than runner/native state/terminal-contact timestamps. Requested-duration completion and recorded-prefix continuity are separate checks."}
    numeric_summary = {
        "rollout_script_ok": summary.get("ok"), "completed_requested_episode": completed,
        "end_reason": summary.get("end_reason"), "terminated": summary.get("terminated"), "truncated": summary.get("truncated"),
        "native_rollout_summary_available": summary_available, "recorded_numerical_data_available": numeric_data_available,
        "output_rows": len(anchor_time), "requested_output_rows": requested_rows, "expected_episode_duration_s": episode["end_s"] - episode["start_s"],
        "all_sto_finite": all(item.get("all_values_finite", False) for item in table_audit.values()) if numeric_data_available else None,
        "all_sto_same_time_grid": all(item.get("same_time_grid_as_kinematics", False) for item in table_audit.values()) if numeric_data_available else None,
        "all_sto_dt_matches_captured_config": all(item.get("sample_dt_matches_receipt", False) for item in table_audit.values()) if len(anchor_time) >= 2 else None,
        "all_sto_cover_requested_episode_samples": all(item.get("covers_requested_preintegration_samples", False) for item in table_audit.values()),
        "all_sto_match_contiguous_recorded_prefix": all(item.get("matches_contiguous_recorded_prefix", False) for item in table_audit.values()) if numeric_data_available else None,
        "source_fingerprints_all_match_receipt": all(item["matches_receipt"] for item in fingerprints.values()),
        "all_sea_identity_checks_pass": all(item["all_identity_checks_pass"] for item in sea.values()) if numeric_data_available else None,
        "sea_non_tautology_checks_pass": all(check.status == "PASS" for check in non_tautology_checks) if len(anchor_time) > 2 and len(non_tautology_checks) == 6 else None,
        "policy_trace_numeric_values_finite": not policy["numeric_nonfinite_paths"] if trace else None,
        "rollout_summary_nonfinite_paths": finite_numeric_paths(summary),
        "policy_steps_match_summary": len(trace) == int(summary["steps"]) if "steps" in summary and trace else None,
        "feature_order_matches_historical": receipt.get("feature_order_matches_historical"),
        "summary_actor_feature_order_matches_receipt": summary.get("actor_feature_names") == receipt.get("actor_feature_names") if summary_available else None,
        "summary_full_feature_order_matches_receipt": summary.get("observation_feature_names") == receipt.get("observation_feature_names") if summary_available else None,
        "policy_trace_covers_requested_segment_end_times": policy["covers_requested_segment_end_times"],
        "policy_trace_covers_completed_segment_end_times": policy["covers_completed_segment_end_times"],
        "actor_clock_identity_checks_pass": actor_clocks["all_clock_identity_checks_pass"],
        "completed_binary_sensor_grids_all_match": runtime_diagnostics.get("completed_binary_sensor_grids_all_match"),
        "active_and_threshold_event_timestamp_issue_count": online_event_timestamp_issues,
        "trace_final_time_matches_receipt": abs(final_time - episode["final_time_s"]) < 1e-7 if episode["final_time_s"] is not None and final_time is not None else None,
        "legacy_validator_counts": dict(Counter(check.status for check in checks)),
        "scientific_note": "Finiteness, consistent SEA dynamics and completed duration are runtime checks. Raw IK tracking, MTP drift, biological reserve demand and gait-event quality remain distinct scientific questions; generic validator tracking thresholds are not a subject-specific acceptance criterion.",
    }
    result = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "rollout": str(rollout.relative_to(REPO)), "diagnostic_script_sha256": helpers.sha256(Path(__file__)),
        "receipt_path": str(receipt_path.relative_to(REPO)), "receipt_sha256": helpers.sha256(receipt_path), "receipt_status": receipt.get("status"),
        "episode": episode, "recording_coverage": recording_coverage, "trace_final_time_s": final_time, "summary": numeric_summary,
        "effective_simulator_config": snapshot, "effective_env_config": env_cfg,
        "captured_policy_contract": {"checkpoint": receipt.get("checkpoint"), "checkpoint_logical_iteration": receipt.get("checkpoint_logical_iteration"), "action_selection": receipt.get("action_selection"), "seed": receipt.get("seed"), "actor_feature_names": receipt.get("actor_feature_names"), "observation_feature_names": receipt.get("observation_feature_names"), "feature_order_matches_historical": receipt.get("feature_order_matches_historical")},
        "source_fingerprints": fingerprints, "output_table_audit": table_audit,
        "raw_ik": {"path": str(raw_path.relative_to(REPO)), "sha256": helpers.sha256(raw_path), "in_degrees": raw.in_degrees},
        "tracking": tracking, "SEA": sea,
        "MTP_separate": {"model_coordinate_metadata": mtp_metadata, "tracking": {name: entry for name, entry in tracking["coordinates"].items() if "mtp" in name.lower()}},
        "reserves_and_recruitment": reserves, "policy_trace": policy, "event_timestamps": events, "actor_clocks": actor_clocks, "runtime_diagnostics": runtime_diagnostics,
        "high_rate_contact": high_rate_contact_audit(tables, cfg, env_cfg, summary, helpers),
        "rollout_summary": summary, "reset_diagnostics": reset,
        "reused_helpers": {"path": str(AB07_HELPER.relative_to(REPO)), "sha256": helpers.sha256(AB07_HELPER), "AB07_audit_unchanged": True, "preserved_AB07_artifacts": ab07_preservation},
        "preserved_AB06_artifacts": ab06_preservation,
        "audit_provenance": {"path": str(provenance_path.relative_to(REPO)), "sha256": helpers.sha256(provenance_path)},
        "reused_validator": {"path": "validation/validate_sim_results.py", "sha256": helpers.sha256(REPO / "validation" / "validate_sim_results.py"), "checks": [asdict(check) for check in checks]},
        "reporting_precision": {"opensim_version": opensim.GetVersionAndDate(), "output_default_significant_digits": precision, "runner_read_method": "simulation_runner.py:1871: float(output.getValueAsString(state))", "sto_writer": "output.py:86: .8f", "angle_derivative_bound": "per sample: string half quantum at 8 significant digits plus two STO half quanta, 1e-8 total, and floating-point margin"},
        "recorded_sea_non_tautology_checks": [asdict(check) for check in non_tautology_checks],
        "scope": "Read-only analysis of a finalized learned deterministic AB07 transfer rollout. No environment or policy constructed, no integration or training, no operational file modifications, no AB07 audit modifications.",
        "coverage_note": "The captured SimulatorConfig retains the full dataset bounds. Requested coverage uses actual episode start/end; observed coverage separately checks a contiguous pre-integration prefix. A completed 5 s episode has 5000 samples, while an early terminal rollout can have a valid shorter prefix. Recorded, completed policy, physical runner and native RK4 state clocks are kept distinct.",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + ".tmp")
    temporary.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(args.output)
    print(json.dumps({"output": str(args.output), "summary": numeric_summary}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
