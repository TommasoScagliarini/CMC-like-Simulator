"""Summarize frozen-policy perturbations; completion is not inferred from ok:true.

Usage: python analyze_results.py [--case control stop_restart_aligned speed_changes]
Missing/running cases are retained as such. Rerun after each experiment.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT / "Trajectory Generator/baseline_MLP"))
import _bootstrap

_bootstrap.ensure_sim_paths()
import win_runtime  # noqa: F401,E402
import numpy as np  # noqa: E402

PLOT = ROOT / "plot/speed_transitions_2026_10_26"
os.environ.setdefault("MPLCONFIGDIR", str(PLOT / ".matplotlib"))
import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

JOINTS = ("pros_knee_angle", "pros_ankle_angle")


def read_json(path, default):
    return json.loads(path.read_text(encoding="utf-8")) if path.exists() else default


def read_journal(path):
    records = []
    if path.exists():
        lines = path.read_text(encoding="utf-8").splitlines()
        for index, line in enumerate(lines):
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError:
                if index != len(lines) - 1:
                    raise
                # A running worker can be midway through its final write.
    return records


def numbers(values):
    array = np.asarray(values, dtype=float)
    return array[np.isfinite(array)]


def maximum(values):
    array = numbers(values)
    return float(array.max()) if len(array) else None


def nonfinite_count(value):
    if isinstance(value, dict):
        return sum(nonfinite_count(v) for v in value.values())
    if isinstance(value, list):
        return sum(nonfinite_count(v) for v in value)
    return int(isinstance(value, float) and not math.isfinite(value))


def storage(path):
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            if line.strip().lower() == "endheader":
                break
        columns = next(stream).split()
        data = np.loadtxt(stream, ndmin=2)
    return columns, data


def motion_statistics(rows, start, end, origin):
    selected = [r for r in rows if start - 1e-8 <= r["time"] - origin <= end + 1e-8]
    result = {"sample_count": len(selected), "source": "policy-boundary trace, 10 ms"}
    for joint in JOINTS:
        stats = {}
        for label, position_key, velocity_key in (
            ("actual", joint, joint + "_vel"),
            ("served", joint + "_served_ref", joint + "_served_ref_vel"),
        ):
            q = numbers([r["prosthetic_state"].get(position_key, np.nan) for r in selected])
            v = numbers([r["prosthetic_state"].get(velocity_key, np.nan) for r in selected])
            stats[label] = {
                "range_rad": float(np.ptp(q)) if len(q) else None,
                "velocity_rms_rad_s": float(np.sqrt(np.mean(v*v))) if len(v) else None,
                "velocity_abs_max_rad_s": float(np.max(np.abs(v))) if len(v) else None,
            }
        result[joint] = stats
    return result


def cycle_intervals(directory, protocol, journal, observed):
    """Separate valid sensor cycles from the prescribed GRF timing oracle."""
    origin = protocol["start_s"]
    schedule = np.loadtxt(directory / "schedule.csv", delimiter=",", skiprows=1)

    def interval(start, end, **extra):
        elapsed_start, elapsed_end = start-origin, end-origin
        grid = schedule[(schedule[:, 0] > start) & (schedule[:, 0] < end), 0]
        grid = np.concatenate(([start], grid, [end]))
        factor = np.interp(grid, schedule[:, 0], schedule[:, 2])
        mean = np.sum(.5*(factor[1:]+factor[:-1])*np.diff(grid)) / (end-start)
        overlaps = [label for label, a, b, _ in protocol["stages"]
                    if min(elapsed_end, b)-max(elapsed_start, a) > 1e-8]
        ramp_overlap = sum(max(0., min(elapsed_end, b)-max(elapsed_start, a))
                           for _, a, b, rate in protocol["stages"] if rate is None)
        contained = next((label for label, a, b, _ in protocol["stages"]
                          if elapsed_start >= a-1e-8 and elapsed_end <= b+1e-8), None)
        return {"start_elapsed_s": elapsed_start, "end_elapsed_s": elapsed_end,
                "duration_s": end-start, "speed_factor_min": float(factor.min()),
                "speed_factor_max": float(factor.max()), "speed_factor_time_mean": float(mean),
                "stages_overlapped": overlaps, "contained_in_stage": contained,
                "spans_ramp": ramp_overlap > 1e-8, "ramp_overlap_s": ramp_overlap,
                "fully_within_observed_rollout": elapsed_start >= -1e-8 and elapsed_end <= observed+1e-8,
                **extra}

    actual = []
    for row in journal:
        phase = row["phase_fsm"]
        for event in phase.get("accepted_transitions_this_step", []):
            if event["event"] == "heel_strike" and event.get("cycle_valid") == 1:
                end = event["event_time_s"]
                period = phase["last_period_s"]
                if period <= 0:
                    raise ValueError("Valid cycle has a nonpositive measured period")
                actual.append(interval(end-period, end, side="left"))
    prescribed = []
    path = directory / "sim_outputs/rollout_episode_gait_events.csv"
    if path.exists():
        with path.open(encoding="utf-8", newline="") as stream:
            for row in csv.DictReader(stream):
                start, end = float(row["cycle_start"]), float(row["cycle_end"])
                if end > origin and start < origin+protocol["duration_s"]:
                    prescribed.append(interval(start, end, side=row["side"],
                        source_force=row["source_force"],
                        contact_duration_s=float(row["contact_duration_s"])))
    return {"actual_valid_cycles": actual, "prescribed_grf_cycles": prescribed,
            "prescribed_note": "Diagnostic cycles from prescribed GRF (warped in perturbed cases); not actual prosthetic cycles. The CSV can include future unexecuted intervals."}


def analyze(case):
    directory = HERE / case
    protocol = read_json(directory / "protocol.json", None)
    if protocol is None:
        return {"status": "missing", "directory": str(directory.relative_to(ROOT))}, None
    summary = read_json(directory / "rollout_summary.json", {})
    failure = read_json(directory / "technical_failure.json", None)
    trace = read_json(directory / "rollout_policy_trace.json", [])
    journal = read_journal(directory / "journal.jsonl")
    origin, duration = protocol["start_s"], protocol["duration_s"]
    elapsed = np.asarray([row["time"] - origin for row in journal])
    observed = float(elapsed[-1]) if len(elapsed) else 0.0
    coverage_complete = observed >= duration - 1e-6
    ended = bool(summary.get("terminated") or summary.get("truncated"))
    if failure is not None:
        status = "technical_failure"
    elif not summary:
        status = "running_or_incomplete"
    elif coverage_complete and not summary.get("terminated"):
        status = "completed_schedule"
    elif ended:
        status = "terminated_before_schedule_end"
    else:
        status = "incomplete_limit_or_interrupted"
    result = {"status": status, "summary_exists": bool(summary),
              "technical_failure": failure,
              "requested_duration_s": duration, "observed_logical_duration_s": observed,
              "observed_physical_duration_s": journal[-1]["physical_time"] - origin if journal else None,
              "covered_entire_schedule": coverage_complete,
              "end_reason": summary.get("end_reason"),
              "terminated": summary.get("terminated"), "truncated": summary.get("truncated"),
              "integrity": read_json(directory / "integrity.json", None),
              "stages": [], "events": [], "source_checks": protocol.get("native_checks", {})}
    for label, start, end, factor in protocol["stages"]:
        selected = [r for r in journal if start + 1e-8 < r["time"] - origin <= end + 1e-8]
        overlap = max(0.0, min(observed, end) - start)
        result["stages"].append({
            "name": label, "planned_start_s": start, "planned_end_s": end,
            "speed_factor": factor, "covered_duration_s": overlap,
            "coverage_fraction": min(1.0, overlap / (end-start)),
            "policy_boundaries": len(selected),
            "motion": motion_statistics(trace, start, min(end, observed), origin),
            "cycle_completions": sum(r["phase_fsm"].get("cycle_completed_this_step", 0) for r in selected),
            "max_penetration_m": maximum([r["reward_terms"].get("grf_penetration_m", np.nan) for r in selected]),
        })
        if label == "hold":
            result["hold_after_first_half_second"] = motion_statistics(trace, start + .5, min(end, observed), origin)
    result["cycle_timing"] = cycle_intervals(directory, protocol, journal, observed)
    for stage in result["stages"]:
        stage["stage_contained_actual_valid_cycles"] = sum(
            row["contained_in_stage"] == stage["name"]
            for row in result["cycle_timing"]["actual_valid_cycles"])
        stage["stage_contained_executed_prescribed_cycles"] = {
            side: sum(row["contained_in_stage"] == stage["name"]
                      and row["fully_within_observed_rollout"] and row["side"] == side
                      for row in result["cycle_timing"]["prescribed_grf_cycles"])
            for side in ("left", "right")}
    for row in journal:
        for event in row["phase_fsm"].get("accepted_transitions_this_step", []):
            result["events"].append({
                "boundary_elapsed_s": row["time"]-origin,
                "event_elapsed_s": event.get("event_time_s", row["time"])-origin,
                "event": event["event"], "cycle_valid": event.get("cycle_valid"),
                "closed_segment_type": event.get("closed_segment_type"),
                "last_period_s": row["phase_fsm"].get("last_period_s"),
            })
    last_fsm = journal[-1]["phase_fsm"] if journal else {}
    result["fsm"] = {key: last_fsm.get(key) for key in (
        "state_name", "valid_hs_count", "valid_to_count", "valid_cycle_count",
        "invalid_event_count", "resync_count", "last_period_s")}
    result["first_timeout_elapsed_s"] = next((r["time"]-origin for r in journal
        if r["phase_fsm"].get("timeout_exceeded", 0) > 0), None)
    result["max_penetration_m"] = maximum([r["reward_terms"].get("grf_penetration_m", np.nan) for r in journal])
    result["reserve_norm_max_nm"] = maximum([r["reward_terms"].get("reserve_norm_nm", np.nan) for r in journal])
    if trace:
        raw = np.asarray([r["raw_policy_action"] for r in trace])
        applied = np.asarray([r["applied_policy_action"] for r in trace])
        result["raw_action"] = {"max_abs": maximum(np.abs(raw).ravel()),
            "clipped_value_fraction": float(np.mean(raw != applied)),
            "clipped_step_count": int(np.count_nonzero(np.any(raw != applied, axis=1)))}
    tables = {}
    for path in sorted((directory / "sim_outputs").glob("*.sto")):
        cols, data = storage(path)
        tables[path.name] = {"rows": len(data), "columns": len(cols),
            "nonfinite_values": int(np.count_nonzero(~np.isfinite(data))),
            "nonfinite_columns": {cols[i]: int(np.count_nonzero(~np.isfinite(data[:, i])))
                                  for i in range(len(cols)) if not np.all(np.isfinite(data[:, i]))},
            "strictly_increasing_time": bool(np.all(np.diff(data[:, 0]) > 0))}
    required = ["rollout_episode_" + suffix + ".sto" for suffix in
                ("states", "kinematics", "kinematics_reference", "sea_controls", "sea_torques", "power")]
    required_trace = [{key: row.get(key) for key in (
        "raw_policy_action", "applied_policy_action", "actor_observation_vector_before", "prosthetic_state")}
        for row in trace]
    result["finite_checks"] = {"journal_nonfinite_values": nonfinite_count(journal),
        "trace_nonfinite_values": nonfinite_count(trace), "sto_files": tables,
        "required_missing_sto_files": [name for name in required if name not in tables],
        "required_numeric_streams_finite": bool(trace) and bool(journal)
            and nonfinite_count(required_trace) == 0
            and all(name in tables and tables[name]["nonfinite_values"] == 0 for name in required),
        "note": "NaN in optional diagnostic channels is inventoried by column, not automatically classified as physical failure.",
        "all_available_outputs_finite": bool(journal) and nonfinite_count(journal) == 0
            and nonfinite_count(trace) == 0 and all(t["nonfinite_values"] == 0 for t in tables.values())}
    return result, (protocol, journal, trace)


def pre_transition_comparisons(cases):
    control = cases.get("control", (None, None))[1]
    comparisons = {}
    names = [name for name, (_, payload) in cases.items()
             if name in {"stop_restart", "stop_restart_aligned", "speed_changes"}
             or (payload and payload[0].get("case") in {"stop_restart", "speed_changes"})]
    for name in names:
        candidate = cases.get(name, (None, None))[1]
        if control is None or candidate is None or not control[2] or not candidate[2]:
            comparisons[name] = {"status": "waiting_for_both_policy_traces"}
            continue
        origin = candidate[0]["start_s"]
        limit = candidate[0]["transitions"][0][0]
        baseline = {round(row["time"]-control[0]["start_s"], 7): row for row in control[2]}
        pairs = [(baseline[round(row["time"]-origin, 7)], row) for row in candidate[2]
                 if row["time"]-origin <= limit+1e-8 and round(row["time"]-origin, 7) in baseline]
        if not pairs:
            comparisons[name] = {"status": "no_common_policy_boundaries"}
            continue
        last = pairs[-1][1]["time"]-origin
        comparison = {"status": "measured", "samples": len(pairs), "requested_end_elapsed_s": limit,
                      "common_end_elapsed_s": last, "prefix_fully_covered": last >= limit-.010001}
        metrics = {
            "raw_actions": lambda r: r["raw_policy_action"],
            "applied_actions": lambda r: r["applied_policy_action"],
            "actor_observation": lambda r: r["actor_observation_vector_before"],
            "actual_joint_positions_rad": lambda r: [r["prosthetic_state"][j] for j in JOINTS],
            "actual_joint_velocities_rad_s": lambda r: [r["prosthetic_state"][j+"_vel"] for j in JOINTS],
        }
        for key, extract in metrics.items():
            difference = np.asarray([extract(a) for a, _ in pairs])-np.asarray([extract(b) for _, b in pairs])
            finite = bool(np.all(np.isfinite(difference)))
            comparison[key] = {"all_finite": finite,
                "max_abs_difference": float(np.max(np.abs(difference))) if finite else None,
                "rms_difference": float(np.sqrt(np.mean(difference*difference))) if finite else None}
        comparisons[name] = comparison
    return comparisons


def overview(cases):
    fig, axes = plt.subplots(5, len(cases), figsize=(5.2*len(cases), 12), squeeze=False)
    labels = ["Velocità relativa", "Ginocchio [rad]", "Caviglia [rad]", "Stato FSM", "Penetrazione [mm]"]
    for col, (name, (result, payload)) in enumerate(cases.items()):
        axes[0, col].set_title(name + "\n" + result["status"], fontsize=10)
        if payload is None:
            for ax in axes[:, col]:
                ax.text(.5, .5, "Non ancora disponibile", ha="center", va="center", transform=ax.transAxes)
            continue
        protocol, journal, trace = payload
        origin = protocol["start_s"]
        schedule = np.loadtxt(HERE / name / "schedule.csv", delimiter=",", skiprows=1)
        planned = (schedule[:, 0] >= origin) & (schedule[:, 0] <= origin+protocol["duration_s"])
        axes[0, col].plot(schedule[planned, 0]-origin, schedule[planned, 2], "--", color=".65", label="programma")
        times = np.asarray([r["time"]-origin for r in journal])
        axes[0, col].plot(times, [r["speed_factor"] for r in journal], color="tab:blue", label="eseguito")
        trace_times = np.asarray([r["time"]-origin for r in trace])
        for row, joint in enumerate(JOINTS, start=1):
            axes[row, col].plot(trace_times, [r["prosthetic_state"].get(joint, np.nan) for r in trace], label="reale")
            axes[row, col].plot(trace_times, [r["prosthetic_state"].get(joint+"_served_ref", np.nan) for r in trace], "--", label="riferimento servito")
        axes[3, col].step(times, [r["phase_fsm"]["state_id"] for r in journal], where="post")
        axes[3, col].set_yticks([0, 1, 2, 4], ["WAIT", "STANCE", "SWING", "TIMEOUT"])
        axes[4, col].plot(times, [1000*r["reward_terms"].get("grf_penetration_m", np.nan) for r in journal])
        axes[4, col].axhline(28, color="tab:red", linestyle=":", label="soglia episodio")
        for ax in axes[:, col]:
            ax.set_xlim(0, protocol["duration_s"])
            ax.grid(alpha=.2)
            ax.axvline(result["observed_logical_duration_s"], color=".3", linestyle=":", linewidth=.8)
            for label, start, end, factor in protocol["stages"]:
                if label in {"hold", "slow", "fast"}:
                    ax.axvspan(start, end, color={"hold":"tab:gray", "slow":"tab:orange", "fast":"tab:green"}[label], alpha=.1)
        for row in (0, 1, 2, 4):
            axes[row, col].legend(fontsize=7, loc="best")
        axes[-1, col].set_xlabel("Tempo dall’avvio [s]")
    for row, label in enumerate(labels):
        axes[row, 0].set_ylabel(label)
    fig.suptitle("Best47 — transizioni sintetiche; le aree vuote non sono state simulate", fontsize=12)
    fig.tight_layout()
    PLOT.mkdir(parents=True, exist_ok=True)
    fig.savefig(PLOT / "transition_overview.png", dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", nargs="+", default=["control", "stop_restart_aligned", "speed_changes"])
    args = parser.parse_args()
    if any(Path(name).name != name for name in args.case):
        parser.error("case names must be directory basenames")
    cases = {name: analyze(name) for name in args.case}
    data = {"interpretation": "Descriptive stress test; completed schedule alone does not establish standing or speed adaptation.",
            "cases": {name: value[0] for name, value in cases.items()},
            "pre_transition_equivalence": pre_transition_comparisons(cases)}
    (HERE / "analysis.json").write_text(json.dumps(data, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    overview(cases)
    for name, (result, _) in cases.items():
        print(name, result["status"], "elapsed", result.get("observed_logical_duration_s"), "end", result.get("end_reason"))


if __name__ == "__main__":
    main()
