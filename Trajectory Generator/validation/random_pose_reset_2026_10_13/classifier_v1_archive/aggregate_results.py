"""Preregistered saved-output classifier. Stdlib only; no simulator imports.

--seal writes the code/criteria seal without reading any rollout outcomes.
Aggregate refuses source drift and never redraws or replaces selected poses.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
CRITERIA_PATH = HERE / "classification_preregistration.json"
CRITERIA = json.loads(CRITERIA_PATH.read_text())
JOINTS = tuple(CRITERIA["tracking"]["joints"])


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"),
                      parse_constant=lambda v: (_ for _ in ()).throw(ValueError("Nonfinite JSON " + v)))


def write_new(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as f:
        json.dump(data, f, indent=2, allow_nan=False)
        f.write("\n")


def finite(value):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("Nonfinite numeric measurement")
    return value


def storage(path):
    lines = Path(path).read_text(encoding="utf-8-sig").splitlines()
    end = next(i for i, l in enumerate(lines) if l.strip().lower() == "endheader")
    meta = dict(l.split("=", 1) for l in lines[:end] if "=" in l)
    body = [l.split() for l in lines[end + 1:] if l.strip()]
    names, raw = body[0], body[1:]
    if names[0].lower() != "time" or not raw:
        raise ValueError("Empty or malformed Storage")
    rows = [[finite(v) for v in row] for row in raw]
    if any(len(row) != len(names) for row in rows):
        raise ValueError("Storage column width mismatch")
    if int(meta["nRows"]) != len(rows) or int(meta["nColumns"]) != len(names):
        raise ValueError("Storage header shape mismatch")
    table = {name: [r[i] for r in rows] for i, name in enumerate(names)}
    dt = CRITERIA["saved_integration_dt_s"]
    tol = CRITERIA["saved_time_tolerance_s"]
    if any(abs(b - a - dt) > tol for a, b in zip(table["time"], table["time"][1:])):
        raise ValueError("Saved integration grid is not 1ms")
    return table, meta


def cp_interval(k, n, alpha=0.05):
    """Exact Clopper-Pearson via inversion of finite binomial sums."""
    if isinstance(k, bool) or isinstance(n, bool) or int(k) != k or int(n) != n or not 0 <= k <= n or n <= 0 or not 0 < alpha < 1:
        raise ValueError("Require integer 0<=successes<=N and N>0")
    k, n = int(k), int(n)

    def tail(p, lo, hi):
        return math.fsum(math.comb(n, i) * p ** i * (1 - p) ** (n - i) for i in range(lo, hi + 1))

    def invert(lo_index, hi_index, target, increasing):
        lo, hi = 0.0, 1.0
        for _ in range(90):
            mid = (lo + hi) / 2
            if (tail(mid, lo_index, hi_index) < target) == increasing:
                lo = mid
            else:
                hi = mid
        return (lo + hi) / 2

    lower = 0.0 if k == 0 else invert(k, n, alpha / 2, True)
    upper = 1.0 if k == n else invert(0, k, alpha / 2, False)
    one_lower = 0.0 if k == 0 else invert(k, n, alpha, True)
    return {"successes": k, "n": n, "estimate": k / n,
            "two_sided95": [lower, upper], "one_sided95_lower": one_lower,
            "method": "Clopper-Pearson exact binomial finite-sum inversion", "alpha": alpha}


def interpolate(times, values, t):
    import bisect
    if t < times[0] - 1e-8 or t > times[-1] + 1e-8:
        raise ValueError("Cycle boundary outside saved actual kinematics")
    i = bisect.bisect_left(times, t)
    if i == 0:
        return values[0]
    if i == len(times):
        return values[-1]
    fraction = (t - times[i - 1]) / (times[i] - times[i - 1])
    return values[i - 1] + fraction * (values[i] - values[i - 1])


def cycle_measurements(trace, actual):
    """Measure EVERY complete valid cycle; each must pass both joints to count."""
    events = [(row, e) for row in trace for e in row["phase_fsm"]["accepted_transitions_this_step"]]
    previous_hs, toe_off = None, None
    hs_confirmed, cycles = [], []
    last_time = -math.inf
    for row, event in events:
        kind = event["event"]
        onset = finite(event["event_time_s"])
        confirmed = finite(event["confirmed_time_s"])
        if onset < last_time or confirmed < onset or confirmed > finite(row["time"]) + 1e-8:
            raise ValueError("Accepted-event journal time/order inconsistent")
        last_time = onset
        if kind == "toe_off":
            toe_off = onset
        elif kind == "heel_strike":
            hs_confirmed.append(confirmed)
            if finite(event["cycle_valid"]) == 1.0:
                if previous_hs is None or toe_off is None or not previous_hs < toe_off < onset:
                    raise ValueError("Valid cycle close lacks HS-TO-HS evidence")
                indices = [i for i, t in enumerate(actual["time"]) if previous_hs <= t < onset]
                if not indices:
                    raise ValueError("Complete cycle has no saved samples")
                excursion = {}
                for joint in JOINTS:
                    values = [actual[joint][i] for i in indices]
                    values += [interpolate(actual["time"], actual[joint], previous_hs),
                               interpolate(actual["time"], actual[joint], onset)]
                    excursion[joint] = max(values) - min(values)
                c = CRITERIA["cyclic_shape"]
                qualifies = excursion[JOINTS[0]] >= c["knee_min_excursion_rad"] and excursion[JOINTS[1]] >= c["ankle_min_excursion_rad"]
                cycles.append({"hs_start_s": previous_hs, "toe_off_s": toe_off, "hs_end_s": onset,
                               "sample_count": len(indices), "excursion_rad": excursion, "both_joints_qualify": qualifies})
            previous_hs, toe_off = onset, None
        else:
            raise ValueError("Unexpected accepted transition: " + kind)
    return hs_confirmed, cycles


def measurements(receipt, summary, trace, actual, reference, grf):
    if not trace:
        raise ValueError("No returned policy transitions")
    t0 = finite(receipt.get("actual_start_s", receipt.get("initial_state", {}).get("time_s")))
    t1 = finite(receipt["actual_end_s"])
    if actual["time"] != reference["time"] or actual["time"] != grf["time"]:
        raise ValueError("Saved actual/reference/GRF grids differ")
    if len(trace) != int(summary["steps"]) or len(trace) != int(receipt["integration_steps"]):
        raise ValueError("Returned transition counts disagree")
    times = [finite(r["time"]) for r in trace]
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError("Policy trace time not monotone")
    tol = CRITERIA["saved_time_tolerance_s"]
    if abs(actual["time"][0] - t0) > tol:
        raise ValueError("Recorded integration time inconsistent with receipt")
    if summary["end_reason"] == "episode_time_limit":
        if len(actual["time"]) != 5000 or abs(actual["time"][-1] - t0 - 4.999) > tol:
            raise ValueError("Complete rollout missing full integration samples")
    raw_peak_pen = max(grf["left_penetration"])
    raw_last_time = actual["time"][-1]
    # A failed, unreturned policy segment can have recorded native substeps.
    # Preserve its physical measurements, but do not assign its samples to
    # returned transitions, cycle credit, reward, or returned-prefix RMSE.
    prefix_ids = [i for i, t in enumerate(actual["time"]) if t < t1 - tol]
    partial_sample_count = len(actual["time"]) - len(prefix_ids)
    actual = {k: [v[i] for i in prefix_ids] for k, v in actual.items()}
    reference = {k: [v[i] for i in prefix_ids] for k, v in reference.items()}
    grf = {k: [v[i] for i in prefix_ids] for k, v in grf.items()}
    if not prefix_ids:
        raise ValueError("No returned-prefix integration samples")
    windows = {"full": list(range(len(actual["time"]))),
               "post050": [i for i, t in enumerate(actual["time"]) if t - t0 >= 0.5 - CRITERIA["tracking"]["time_tolerance_s"]],
               "startup050": [i for i, t in enumerate(actual["time"]) if t - t0 < 0.5 - CRITERIA["tracking"]["time_tolerance_s"]]}
    tracking = {}
    for joint in JOINTS:
        tracking[joint] = {}
        for name, ids in windows.items():
            tracking[joint][name] = (math.sqrt(math.fsum((actual[joint][i] - reference[joint + "_q_ref"][i]) ** 2 for i in ids) / len(ids)) * 180 / math.pi) if ids else None
    flags = {name: sum(finite(r["phase_fsm"][name]) != 0 for r in trace) for name in CRITERIA["zero_phase_flags"]}
    counters = {name: max(finite(r["phase_fsm"][name]) for r in trace) for name in CRITERIA["zero_phase_counters"]}
    actor_cycles = finite(trace[-1]["phase_fsm"]["valid_cycle_count"])
    if actor_cycles != finite(summary["phase_valid_cycle_count"]):
        raise ValueError("Trace/summary valid-cycle counts disagree")
    hs, cycles = cycle_measurements(trace, actual)
    if actor_cycles != len(cycles):
        raise ValueError("Actor valid-cycle count lacks complete journal cycles")
    peak_pen = raw_peak_pen
    static_gate = receipt.get("static_contact_gate", {})
    if "max_applied_penetration_m" in static_gate:
        peak_pen = max(peak_pen, finite(static_gate["max_applied_penetration_m"]))
    first_hs = min(hs) - t0 if hs else None
    gates = {
        "completion": int(summary["steps"]) == 500 and summary["end_reason"] == "episode_time_limit" and not bool(summary["terminated"]) and abs(t1 - t0 - 5) <= 1e-8,
        "valid_cycles": actor_cycles >= 2,
        "first_accepted_hs": first_hs is not None and first_hs <= 3.0,
        "zero_flags": all(v == 0 for v in flags.values()),
        "zero_counters": all(v == 0 for v in counters.values()),
        "primary_penetration": peak_pen < 0.028,
        "post050_tracking": all(tracking[j]["post050"] is not None and tracking[j]["post050"] <= 2.0 for j in JOINTS),
        "two_cycles_each_both_excursions": sum(c["both_joints_qualify"] for c in cycles) >= 2,
    }
    return {"gates": gates, "completed_safely": gates["completion"] and gates["primary_penetration"],
            "duration_s": t1 - t0, "returned_policy_steps": len(trace), "valid_cycles": actor_cycles,
            "first_hs_confirmed_elapsed_s": first_hs, "phase_flag_record_counts": flags,
            "phase_counter_maxima": counters, "left_primary_peak_penetration_m": peak_pen,
            "tracking_rmse_deg": tracking, "window_samples": {k: len(v) for k, v in windows.items()},
            "complete_valid_cycles": cycles, "qualifying_cycle_count": sum(c["both_joints_qualify"] for c in cycles),
            "partial_unreturned_segment_saved_samples": partial_sample_count,
            "raw_saved_last_time_s": raw_last_time, "last_returned_policy_boundary_s": t1,
            "tracking_population": "returned-prefix native samples only; unreturned partial samples excluded"}


def exception_class(receipt, summary):
    text = " ".join(str(x) for x in (receipt.get("error", ""), summary.get("failure_exception", ""), summary.get("end_reason", ""))).lower()
    reason = str(summary.get("end_reason", ""))
    if any(s in text for s in ("segmentwallclocktimeout", "watchdog", "wallclocktimeout", "wall-clock timeout", "step_wall_timeout")):
        return "TECHNICAL_CENSORED"
    if "onlinegrfpenetrationlimitexceeded" in text or "online grf exceeds" in text or "grf_penetration" in text or receipt.get("status") == "physical_force_guard_censored" or reason in {"fall", "pelvis_height", "grf_force"} or reason.startswith("joint_divergence:"):
        return "PHYSICAL_FAILURE"
    if reason == "numerical_failure":
        return "TECHNICAL_CENSORED"
    if receipt.get("status") in {"numerical_exception_censored", "harness_or_native_failure", "static_reset_rejected", "started"}:
        return "TECHNICAL_CENSORED"
    if receipt.get("status") == "physical_guard_censored":
        return "TECHNICAL_CENSORED"  # ambiguous guard label alone is not causal proof
    return None


def recover_returned_prefix(folder, receipt):
    """SIGSEGV may bypass finally. Use paired saved RETURNED records only."""
    trace_file, reward_file = Path(folder) / "last_valid_trace.jsonl", Path(folder) / "returned_rewards.jsonl"
    if not trace_file.exists() or not reward_file.exists():
        return None

    def complete_lines(path):
        rows = []
        for line in path.read_text().splitlines():
            if not line.strip():
                continue
            try:
                rows.append(json.loads(line, parse_constant=lambda v: (_ for _ in ()).throw(ValueError(v))))
            except (ValueError, TypeError):
                break  # preserve a torn final line as unavailable, never invent it
        return rows

    traces, rewards = complete_lines(trace_file), complete_lines(reward_file)
    n = min(len(traces), len(rewards))
    if not n:
        return None
    for i, (tr, rw) in enumerate(zip(traces[:n], rewards[:n]), 1):
        if tr["step"] != i or rw["step"] != i or abs(finite(tr["time"]) - finite(rw["time_after_s"])) > 1e-8 or abs(finite(tr["returned_reward"]) - finite(rw["returned_reward"])) > 1e-12:
            raise ValueError("Saved returned journals disagree")
    last = traces[n - 1]["phase_fsm"]
    updated = dict(receipt)
    updated.update(actual_start_s=finite(rewards[0]["time_before_s"]), actual_end_s=finite(rewards[n - 1]["time_after_s"]), integration_steps=n)
    summary = {"steps": n, "episode_return": math.fsum(finite(r["returned_reward"]) for r in rewards[:n]),
               "phase_valid_cycle_count": finite(last["valid_cycle_count"]), "end_reason": "native_crash_technical_censored",
               "terminated": False, "truncated": True, "summary_kind": "offline_paired_returned_prefix_only"}
    evidence = {"source": "paired last_valid_trace.jsonl and returned_rewards.jsonl only", "paired_returned_records": n,
                "stale_receipt_integration_steps": receipt.get("integration_steps"), "unpaired_trace_records": len(traces) - n,
                "unpaired_reward_records": len(rewards) - n, "last_completed_time_s": updated["actual_end_s"],
                "paired_reward_sum": summary["episode_return"], "failed_transition_reward_invented": False}
    return updated, summary, traces[:n], evidence


def classify_folder(folder, case):
    folder = Path(folder)
    result = {"id": case["id"], "case": case, "classification": "POSTPROCESSING_UNRESOLVED",
              "strict_primary_pass": False, "completed_safely": False, "input_sha256": {}, "measurement_error": None}
    if not (folder / "receipt.json").exists():
        result.update(classification="TECHNICAL_CENSORED", measurement_error="Missing execution receipt")
        return result
    result["input_sha256"]["receipt.json"] = sha(folder / "receipt.json")
    try:
        receipt = read_json(folder / "receipt.json")
    except (ValueError, TypeError) as exc:
        result["measurement_error"] = f"Unreadable execution receipt: {exc}"
        return result
    if not isinstance(receipt, dict):
        result["measurement_error"] = "Execution receipt is not an object"
        return result
    if receipt.get("case", {}).get("id") != case["id"] or receipt.get("case", {}).get("pose") != case.get("pose"):
        raise ValueError("Execution receipt differs from selected pose")
    result["receipt_status"] = receipt.get("status")
    result["postprocessing"] = receipt.get("postprocessing")
    # A crash can prevent the final hash check; that stays technically uncertain.
    # Paired returned journals may still describe a completed prefix.
    integrity_ok = receipt.get("policy_updates") == 0 and receipt.get("frozen_inputs_unchanged") is True
    if receipt.get("policy_updates") != 0:
        result.update(classification="TECHNICAL_CENSORED", measurement_error="Frozen-input/update integrity not established")
        return result
    try:
        summary = read_json(folder / "rollout_summary.json") if (folder / "rollout_summary.json").exists() else {}
    except (ValueError, TypeError) as exc:
        result.update(classification=exception_class(receipt, {}) or "POSTPROCESSING_UNRESOLVED",
                      measurement_error=f"Unreadable numeric summary: {exc}")
        return result
    recovered_trace = None
    if not summary:
        try:
            recovered = recover_returned_prefix(folder, receipt)
        except (ValueError, KeyError, TypeError) as exc:
            result.update(classification="TECHNICAL_CENSORED", measurement_error=f"Returned-prefix recovery failed: {exc}")
            return result
        if recovered:
            receipt, summary, recovered_trace, result["returned_prefix_recovery"] = recovered
            for name in ("last_valid_trace.jsonl", "returned_rewards.jsonl"):
                result["input_sha256"][name] = sha(folder / name)
    if not isinstance(summary, dict):
        result["measurement_error"] = "Numeric summary is not an object"
        return result
    prior_class = exception_class(receipt, summary)
    if not integrity_ok:
        prior_class = "TECHNICAL_CENSORED"
        result["final_frozen_input_check_unavailable"] = True
    if prior_class:
        result["classification"] = prior_class
    result["end_reason"] = summary.get("end_reason")
    result["returned_reward_sum"] = summary.get("episode_return")
    try:
        trace_path = folder / "rollout_policy_trace.json"
        if recovered_trace is not None:
            trace_path = folder / "last_valid_trace.jsonl"
            trace = recovered_trace
        elif trace_path.exists():
            trace = read_json(trace_path)
        else:
            trace_path = folder / "last_valid_trace.jsonl"
            trace = [json.loads(l) for l in trace_path.read_text().splitlines() if l.strip()]
        paths = {"actual": folder / "sim_outputs/rollout_episode_kinematics.sto",
                 "reference": folder / "sim_outputs/rollout_episode_kinematics_reference.sto",
                 "grf": folder / "sim_outputs/rollout_episode_online_grf.sto"}
        tables = {}
        for name, path in paths.items():
            table, meta = storage(path)
            if name in {"actual", "reference"} and meta.get("inDegrees", "").strip().lower() != "no":
                raise ValueError("Unexpected angular units; require radiants inDegrees=no")
            tables[name] = table
        for path in [folder / "rollout_summary.json", trace_path, *paths.values()]:
            if path.exists():
                result["input_sha256"][str(path.relative_to(folder))] = sha(path)
        values = measurements(receipt, summary, trace, tables["actual"], tables["reference"], tables["grf"])
        result.update(values)
        if prior_class is None:
            result["classification"] = "GAIT_RECOVERY_PASS" if all(values["gates"].values()) else "GAIT_CRITERION_FAILURE"
        result["strict_primary_pass"] = result["classification"] == "GAIT_RECOVERY_PASS"
    except (ValueError, KeyError, TypeError, FileNotFoundError, IndexError, StopIteration, OverflowError) as exc:
        result["measurement_error"] = f"{type(exc).__name__}: {exc}"
        # A witnessed physical guard remains failure even when final data are incomplete.
        # A missing metric for a normal complete run remains unresolved postprocessing.
    result["failed_gates"] = [k for k, v in result.get("gates", {}).items() if not v]
    return result


def seal(path):
    data = {"sealed_utc": datetime.now(timezone.utc).isoformat(), "criterion_id": CRITERIA["criterion_id"],
            "new_dynamic_outcomes_read": False, "classifier_sha256": sha(__file__),
            "criteria_sha256": sha(CRITERIA_PATH),
            "offline_tests_sha256": sha(HERE / "test_aggregate_results.py")}
    write_new(path, data)
    return data


def aggregate(selection_path, rollouts, seal_path):
    locked = read_json(seal_path)
    if locked["classifier_sha256"] != sha(__file__) or locked["criteria_sha256"] != sha(CRITERIA_PATH) or locked["offline_tests_sha256"] != sha(HERE / "test_aggregate_results.py"):
        raise ValueError("Classifier/criteria/test seal drift")
    selected = read_json(selection_path)
    cases = selected["cases"]
    for c in cases:
        if not c["id"] or any(x not in "abcdefghijklmnopqrstuvwxyz0123456789_" for x in c["id"]):
            raise ValueError("Unsafe case id")
    if len({c["id"] for c in cases}) != len(cases):
        raise ValueError("Duplicated selected case")
    results = [classify_folder(Path(rollouts) / c["id"], c) for c in cases]
    random_results = [r for r in results if r["id"].startswith("random_")]
    controls = [r for r in results if not r["id"].startswith("random_")]
    n = len(random_results)
    successes = sum(r["strict_primary_pass"] for r in random_results)
    unresolved = sum(r["classification"] in {"TECHNICAL_CENSORED", "POSTPROCESSING_UNRESOLVED"} for r in random_results)
    counts = dict(Counter(r["classification"] for r in random_results))
    preflight = selected.get("preflight_outcomes", [])
    pool = [r for r in preflight if r["id"].startswith("random_")]
    if pool:
        expected_ids = [r["id"] for r in pool if (r.get("gate") or {}).get("eligible_to_integrate", False)][:20]
        if [r["id"] for r in random_results] != expected_ids:
            raise ValueError("Selected random IDs do not follow first-eligible frozen pool order")
    reasons = Counter(reason for r in pool for reason in (r.get("gate") or {}).get("rejection_reasons", []))
    primary_reasons = Counter(((r.get("gate") or {}).get("rejection_reasons") or ["unresolved_geometry"])[0]
                              for r in pool if not (r.get("gate") or {}).get("eligible_to_integrate", False))
    complete = [r for r in random_results if r.get("gates", {}).get("completion")]
    safe_complete = [r for r in random_results if r["completed_safely"]]
    frontier = {"run_completed": len(complete), "completed_safely": len(safe_complete), "strict_gait_recovery_pass": successes,
                "completed_safely_but_strict_fail_ids": [r["id"] for r in safe_complete if not r["strict_primary_pass"]],
                "completion_vs_strict_gate_failures": dict(Counter(g for r in complete for g in r["failed_gates"]))}
    return {"schema_version": 1, "criterion_id": CRITERIA["criterion_id"], "scope": "AB06 best47, fixed biology timing, qdot0, geometric-eligible uniform pose distribution",
            "selection_sha256": sha(selection_path), "classifier_seal": locked,
            "selected_random_n": n, "target_random_n": 20, "target_n_reached": n == 20,
            "successes": successes, "unresolved_n": unresolved, "classification_counts": counts,
            "success_probability_cp95": cp_interval(successes, n) if n and not unresolved else None,
            "all_selected_success_proportion_bounds": [successes / n, (successes + unresolved) / n] if n else None,
            "geometry_pool": {"proposals": len(pool), "eligible": sum(bool((r.get("gate") or {}).get("eligible_to_integrate")) for r in pool),
                              "all_rejection_reason_counts": dict(reasons), "exclusive_primary_reason_counts": dict(primary_reasons)},
            "completion_vs_strict_frontier": frontier, "controls": controls, "random_cases": random_results,
            "limitations": ["No physical failure is replaced. Unresolved technical/postprocessing cases stay in the selected denominator.",
                            "CP describes conditional Monte Carlo success probability, not universal capacity, standing-start validation, hardware robustness or cross-subject generalization.",
                            "Biology uses fixed IK references while its actual19DOF muscle/reserve-driven states remain dynamically free.",
                            "Rewards/events from unreturned transitions are never fabricated; interrupted physical state is not a completed policy boundary."]}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--seal", type=Path, help="Write a new source/criteria seal; reads no outcomes")
    p.add_argument("--selection", type=Path, default=HERE / "preflight/selected_cases.json")
    p.add_argument("--rollouts", type=Path, default=HERE / "rollouts")
    p.add_argument("--seal-file", type=Path, default=HERE / "classifier_seal.json")
    p.add_argument("--output", type=Path, default=HERE / "aggregate_results.json")
    args = p.parse_args()
    if args.seal:
        print(json.dumps(seal(args.seal), indent=2))
    else:
        result = aggregate(args.selection, args.rollouts, args.seal_file)
        write_new(args.output, result)
        print(json.dumps({k: result[k] for k in ["selected_random_n", "successes", "unresolved_n", "classification_counts", "completion_vs_strict_frontier"]}, indent=2))


if __name__ == "__main__":
    main()
