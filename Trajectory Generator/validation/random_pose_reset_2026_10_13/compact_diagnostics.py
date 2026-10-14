"""Exploratory saved-data diagnostics, separate from sealed success criteria.

Stdlib only. Reads completed receipts (or completed supervisor execution records),
skips running cases, never changes the classifier, harness, weights or criteria.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import importlib.util
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("sealed_pose_classifier_read_helpers", HERE / "aggregate_results_v2.py")
A = importlib.util.module_from_spec(spec)
spec.loader.exec_module(A)
JOINTS = ("pros_knee_angle", "pros_ankle_angle")
TRANSLATIONS = ("pelvis_tx", "pelvis_ty", "pelvis_tz")
COMPARE_WINDOWS = {"startup_000_050": (0, .5), "early_050_200": (.5, 2), "middle_200_400": (2, 4), "late_400_500": (4, 5)}
RESOURCE_WINDOWS = {"startup_000_050": (0, .5), "late_400_500": (4, 5)}


def vector2(raw):
    if len(raw) == 1 and isinstance(raw[0], list):
        raw = raw[0]
    if len(raw) != 2:
        raise ValueError("Expected two policy actions")
    return [A.finite(x) for x in raw]


def scalar_stats(values):
    if not values:
        return {"samples": 0, "mean": None, "rms": None, "peak_abs": None, "min": None, "max": None}
    return {"samples": len(values), "mean": math.fsum(values) / len(values),
            "rms": math.sqrt(math.fsum(x * x for x in values) / len(values)),
            "peak_abs": max(abs(x) for x in values), "min": min(values), "max": max(values)}


def window_ids(times, t0, lo, hi):
    return [i for i, t in enumerate(times) if lo - 1e-8 <= t - t0 < hi - 1e-8]


def clipping(trace):
    raw = [vector2(r["raw_policy_action"]) for r in trace]
    applied = [vector2(r["applied_policy_action"]) for r in trace]
    per_joint = {}
    for i, joint in enumerate(JOINTS):
        per_joint[joint] = {"raw_outside_normalized_bounds_steps": sum(abs(r[i]) > 1 for r in raw),
                            "raw_max_abs": max((abs(r[i]) for r in raw), default=None),
                            "applied_max_abs": max((abs(r[i]) for r in applied), default=None),
                            "raw_minus_applied_rms": scalar_stats([r[i] - a[i] for r, a in zip(raw, applied)])["rms"]}
    return {"returned_steps": len(trace), "any_joint_raw_clipped_steps": sum(any(abs(x) > 1 for x in r) for r in raw),
            "per_joint": per_joint, "population": "returned policy transitions only; normalized action [-1,1]"}


def event_times(trace, t0, zero_terminal_steps=()):
    events = [e for i, row in enumerate(trace, 1) if i not in zero_terminal_steps for e in row["phase_fsm"]["accepted_transitions_this_step"]]

    def timed(event):
        if event is None:
            return None
        return {"onset_elapsed_s": A.finite(event["event_time_s"]) - t0,
                "confirmed_elapsed_s": A.finite(event["confirmed_time_s"]) - t0,
                "delivered_elapsed_s": A.finite(event.get("delivered_time_s", event["confirmed_time_s"])) - t0}

    hs = [e for e in events if e["event"] == "heel_strike"]
    to = [e for e in events if e["event"] == "toe_off"]
    closes = [e for e in hs if A.finite(e["cycle_valid"]) == 1]
    return {"accepted_hs_count": len(hs), "accepted_to_count": len(to), "complete_valid_cycle_closes": len(closes),
            "first_hs": timed(hs[0] if hs else None), "first_to": timed(to[0] if to else None),
            "first_cycle_close": timed(closes[0] if closes else None), "second_cycle_close": timed(closes[1] if len(closes) > 1 else None),
            "note": "First TO may close a bootstrap partial stance. Missing events remain null; no event reconstructed from forces or kinematics."}


def initial_diagnostics(receipt, control_receipt):
    state = receipt.get("initial_state", {})
    actual = state.get("actual_q", {})
    if control_receipt:
        nominal = control_receipt.get("initial_state", {}).get("actual_q", {})
        nominal_source = "coherent_nominal receipt actual initial q"
    else:
        nominal = {j: state.get("original_prosthetic_ik", {}).get(j, {}).get("q") for j in JOINTS}
        nominal_source = "own recorded nominal IK q; coherent control unavailable"
    delta = {j: A.finite(actual[j]) - A.finite(nominal[j]) for j in JOINTS if j in actual and nominal.get(j) is not None}
    gate = receipt.get("static_contact_gate", {})
    left = gate.get("primary_grf", {}).get("sides", {}).get("left", {})
    binary = gate.get("binary_detector_baseline") or {}
    contacts = binary.get("contacts", {})
    heel, toe = contacts.get("left_heel"), contacts.get("left_toe")
    word = {(False, False): "AIR", (True, False): "HEEL", (True, True): "BOTH", (False, True): "TOE"}.get((heel, toe)) if heel is not None and toe is not None else None
    force_limit = gate.get("force_guard_n")
    bw = A.finite(force_limit) / 5 if force_limit is not None else None
    result = {"actual_q_rad": {j: actual.get(j) for j in JOINTS}, "delta_from_nominal_rad": delta,
              "euclidean_distance_from_nominal_rad": math.sqrt(math.fsum(x * x for x in delta.values())) if len(delta) == 2 else None,
              "nominal_source": nominal_source, "bodyweight_n_from_frozen5BW_guard": bw,
              "primary_contact": {"in_contact": left.get("in_contact"), "normal_force_n": left.get("normal_force"),
                                  "force_vector_n": left.get("force"), "max_penetration_m": gate.get("max_applied_penetration_m"),
                                  "force_norm_n": gate.get("peak_applied_force_norm_n")},
              "detector": {"word": word, "heel_contact": heel, "toe_contact": toe,
                           "signed_clearance_m": binary.get("signed_clearance_m")},
              "pelvis_height_m": gate.get("pelvis_height_m")}
    result["stratum"] = "primary_contact_" + str(left.get("in_contact")) + "__detector_" + str(word)
    return result


def reserve_diagnostics(table, t0):
    if table is None:
        return None
    columns = [name for name in table if name != "time" and not any(j in name for j in JOINTS)]
    by_coord = {name.removesuffix("_reserve_torque"): name for name in columns}
    torque_cols = [name for coord, name in by_coord.items() if coord not in TRANSLATIONS]
    force_cols = [by_coord[c] for c in TRANSLATIONS if c in by_coord]
    result = {}
    for window, (lo, hi) in RESOURCE_WINDOWS.items():
        ids = window_ids(table["time"], t0, lo, hi)
        metrics = {coord: {"rms": scalar_stats([table[col][i] for i in ids])["rms"],
                           "peak_abs": scalar_stats([table[col][i] for i in ids])["peak_abs"],
                           "unit": "N" if coord in TRANSLATIONS else "Nm"} for coord, col in by_coord.items()}
        result[window] = {"samples": len(ids), "per_coordinate": metrics,
                          "biological_rotational_reserve_norm_Nm": scalar_stats([math.sqrt(math.fsum(table[c][i] ** 2 for c in torque_cols)) for i in ids]),
                          "pelvis_translation_residual_force_norm_N": scalar_stats([math.sqrt(math.fsum(table[c][i] ** 2 for c in force_cols)) for i in ids]),
                          "pelvis_residuals": {c: v for c, v in metrics.items() if c.startswith("pelvis_")}}
    result["population"] = "all saved physical native samples in each requested window, including explicitly noted partial unreturned segment"
    return result


def force_diagnostics(table, t0, bw):
    if table is None:
        return None
    result = {}
    for window, (lo, hi) in RESOURCE_WINDOWS.items():
        ids = window_ids(table["time"], t0, lo, hi)
        force_norm = [math.sqrt(math.fsum(table["left_force_" + axis][i] ** 2 for axis in "xyz")) for i in ids]
        normal = [table["left_normal_force"][i] for i in ids]
        contact = [table["left_in_contact"][i] for i in ids]
        result[window] = {"samples": len(ids), "left_force_norm_n": scalar_stats(force_norm), "left_normal_force_n": scalar_stats(normal),
                          "left_normal_force_bw": scalar_stats([x / bw for x in normal]) if bw else None,
                          "geometric_contact_fraction": math.fsum(bool(x) for x in contact) / len(contact) if contact else None,
                          "left_penetration_m": scalar_stats([table["left_penetration"][i] for i in ids])}
    result["all_saved_peak_force_norm_n"] = max((math.sqrt(math.fsum(table["left_force_" + a][i] ** 2 for a in "xyz")) for i in range(len(table["time"]))), default=None)
    result["all_saved_peak_penetration_m"] = max(table["left_penetration"], default=None)
    result["population"] = "all saved physical native samples, not reconstructed reward/events"
    return result


def compare_same_clock(actual, control, t0):
    result = {}
    if actual is None or control is None:
        return None
    # Same nominal t0 and exact saved1ms clock. No optimized shift, interpolation,
    # phase warping, or substitution of sound-side kinematics.
    control_index = {round((t - t0) / .001): i for i, t in enumerate(control["time"])}
    for window, (lo, hi) in COMPARE_WINDOWS.items():
        pairs = []
        for i in window_ids(actual["time"], t0, lo, hi):
            j = control_index.get(round((actual["time"][i] - t0) / .001))
            if j is not None and abs(actual["time"][i] - control["time"][j]) <= 2e-8:
                pairs.append((i, j))
        rms = {}
        for coord in actual:
            if coord == "time" or coord not in control:
                continue
            scale = 1 if coord in TRANSLATIONS else 180 / math.pi
            rms[coord] = {"rmse": math.sqrt(math.fsum((actual[coord][i] - control[coord][j]) ** 2 for i, j in pairs) / len(pairs)) * scale if pairs else None,
                          "unit": "m" if coord in TRANSLATIONS else "deg"}
        result[window] = {"overlap_samples": len(pairs), "requested_samples": round((hi - lo) / .001),
                          "biological_q_rmse": {k: v for k, v in rms.items() if k not in JOINTS},
                          "prosthetic_q_rmse": {k: v for k, v in rms.items() if k in JOINTS}}
    return result


def completed(summary, receipt):
    return bool(summary.get("steps") == 500 and summary.get("end_reason") == "episode_time_limit" and not summary.get("terminated", True)
                and abs(A.finite(receipt["actual_end_s"]) - A.finite(receipt["actual_start_s"]) - 5) <= 1e-8)


def load_case(root, case):
    folder = Path(root) / case["id"]
    result = {"id": case["id"], "case": case, "exploratory_only": True, "diagnostic_errors": [], "input_sha256": {}}
    receipt_path = folder / "receipt.json"
    if not receipt_path.exists():
        result["status"] = "PENDING_NO_RECEIPT"
        return result, None, {}, {}
    receipt = A.read_json(receipt_path)
    execution = Path(root) / (case["id"] + "_execution.json")
    # The harness writes execution.json only after its supervisor returns.
    if "elapsed_wall_s" not in receipt and not execution.exists():
        result["status"] = "PENDING_RUNNING_RECEIPT"
        return result, None, {}, {}
    result["input_sha256"]["receipt.json"] = A.sha(receipt_path)
    result.update(status=receipt.get("status"), elapsed_wall_s=receipt.get("elapsed_wall_s"),
                  final_frozen_inputs_check=receipt.get("frozen_inputs_unchanged"))
    try:
        receipt, returned_evidence = A.attach_returned_journal_evidence(folder, receipt)
    except (ValueError, KeyError, TypeError) as exc:
        returned_evidence = None
        result["diagnostic_errors"].append(f"Returned journal: {type(exc).__name__}: {exc}")
    if returned_evidence:
        result["returned_journal_evidence"] = returned_evidence
        result["input_sha256"]["returned_rewards.jsonl"] = A.sha(folder / "returned_rewards.jsonl")
    summary_path = folder / "rollout_summary.json"
    summary = A.read_json(summary_path) if summary_path.exists() else {}
    trace = []
    trace_path = folder / "rollout_policy_trace.json"
    if trace_path.exists():
        trace = A.read_json(trace_path)
        result["input_sha256"]["rollout_policy_trace.json"] = A.sha(trace_path)
    elif not summary:
        recovered = A.recover_returned_prefix(folder, receipt)
        if recovered:
            receipt, summary, trace, evidence = recovered
            result["returned_prefix_recovery"] = evidence
    if summary_path.exists():
        result["input_sha256"]["rollout_summary.json"] = A.sha(summary_path)
    t0 = A.finite(receipt.get("actual_start_s", receipt.get("initial_state", {}).get("time_s")))
    boundary = A.finite(receipt.get("actual_end_s", trace[-1]["time"] if trace else t0))
    tables, checks = {}, {}
    for path in sorted((folder / "sim_outputs").glob("*.sto")):
        key = path.stem.removeprefix("rollout_episode_")
        try:
            table, meta = A.storage(path)
            tables[key] = table
            checks[path.name] = {"rows": len(table["time"]), "all_numeric_finite": True,
                                 "first_time_s": table["time"][0], "last_time_s": table["time"][-1], "inDegrees": meta.get("inDegrees"), "sha256": A.sha(path)}
        except (ValueError, KeyError, TypeError, StopIteration, IndexError) as exc:
            result["diagnostic_errors"].append(f"{path.name}: {type(exc).__name__}: {exc}")
    actual = tables.get("kinematics")
    for name, table in tables.items():
        checks["rollout_episode_" + name + ".sto"]["grid_matches_actual"] = table["time"] == actual["time"] if actual else None
    result.update(sto_file_count=len(checks), expected_full_sto_files=17, sto_checks=checks,
                  actual_start_s=t0, last_returned_boundary_s=boundary, returned_steps=len(trace),
                  recorded_policy_steps=summary.get("steps"), end_reason=summary.get("end_reason"),
                  requested_horizon_completed=completed(summary, receipt) if summary else False,
                  requested_duration_s=5, returned_duration_s=boundary - t0)
    if actual:
        result["saved_physical_samples"] = len(actual["time"])
        result["saved_physical_duration_s"] = actual["time"][-1] - t0 + .001
        result["partial_unreturned_saved_samples"] = sum(t >= boundary - 2e-8 for t in actual["time"])
    try:
        result["raw_action_clipping"] = clipping(trace)
        zeros = receipt.get("_witnessed_zero_time_terminal_steps", [])
        result["accepted_event_times"] = event_times(trace, t0, zeros)
        result["positive_duration_only_action_clipping"] = clipping([r for i, r in enumerate(trace, 1) if i not in zeros])
    except (ValueError, KeyError, TypeError, IndexError) as exc:
        result["diagnostic_errors"].append(f"Returned trace diagnostics: {type(exc).__name__}: {exc}")
    return result, receipt, summary, tables


def load_case_or_note(root, case):
    try:
        return load_case(root, case)
    except (ValueError, KeyError, TypeError, FileNotFoundError, IndexError) as exc:
        return {"id": case["id"], "case": case, "exploratory_only": True,
                "status": "DIAGNOSTIC_UNAVAILABLE", "diagnostic_errors": [f"{type(exc).__name__}: {exc}"]}, None, {}, {}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--selection", type=Path, default=HERE / "preflight/selected_cases.json")
    p.add_argument("--rollouts", type=Path, default=HERE / "rollouts")
    p.add_argument("--output", type=Path, default=HERE / "compact_diagnostics.json")
    args = p.parse_args()
    sealed_before = {name: A.sha(HERE / name) for name in ("aggregate_results.py", "classification_preregistration.json", "classifier_seal.json", "test_aggregate_results.py", "aggregate_results_v2.py", "classifier_seal_v2.json", "test_aggregate_results_v2.py")}
    selection = A.read_json(args.selection)
    loaded = {case["id"]: load_case_or_note(args.rollouts, case) for case in selection["cases"]}
    control = loaded.get("coherent_nominal")
    control_receipt = control[1] if control else None
    control_table = control[3].get("kinematics") if control and control[0].get("requested_horizon_completed") else None
    cases = []
    for result, receipt, summary, tables in loaded.values():
        if receipt is not None:
            try:
                result["initial"] = initial_diagnostics(receipt, control_receipt)
                t0 = result["actual_start_s"]
                result["biological_reserves"] = reserve_diagnostics(tables.get("reserve_torques"), t0)
                result["left_physical_force"] = force_diagnostics(tables.get("online_grf"), t0, result["initial"]["bodyweight_n_from_frozen5BW_guard"])
                contacts = result.get("returned_journal_evidence", {}).get("measured_terminal_contacts", [])
                result["terminal_primary_contact_samples"] = contacts
                peak = (result["left_physical_force"] or {}).get("all_saved_peak_penetration_m")
                result["max_witnessed_physical_penetration_m"] = max([p for p in [peak, *[c["penetration_m"] for c in contacts]] if p is not None], default=None)
                if result["requested_horizon_completed"]:
                    result["same_clock_actual_vs_coherent_nominal"] = compare_same_clock(tables.get("kinematics"), control_table, t0)
                    if control_table is None:
                        result["same_clock_comparison_unavailable_reason"] = "Coherent control did not complete full horizon with saved actual kinematics"
            except (ValueError, KeyError, TypeError, IndexError) as exc:
                result["diagnostic_errors"].append(f"Physical diagnostics: {type(exc).__name__}: {exc}")
        cases.append(result)
    output = {"schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(), "exploratory_descriptive_only": True,
              "changes_to_pass_criteria": False, "native_executions": 0,
              "selection_sha256": A.sha(args.selection), "sealed_classifier_inputs_before": sealed_before,
              "diagnostics_code_sha256": A.sha(__file__), "cases": cases,
              "initial_contact_detector_strata_counts": dict(Counter(r["initial"]["stratum"] for r in cases if "initial" in r)),
              "limitations": ["Only finalized receipts or completed supervisor executions are read; running cases remain pending.",
                              "Clipping/events describe returned policy boundaries only. Physical STO windows can include explicitly counted partial unreturned samples, never fabricated reward or events.",
                              "Rotational reserve norms have unitsNm; pelvis translational residuals have unitsN and are not merged into a physically misleading norm.",
                              "Comparison includes all full-horizon completers, irrespective of strict primary PASS; incomplete cases are not zero-filled.",
                              "Same-clock actual differences are descriptive, without time shifts, phase alignment or new pass criteria.",
                              "Biology has fixed IK references but its actual muscle/reserve-driven states remain dynamically free."]}
    sealed_after = {name: A.sha(HERE / name) for name in sealed_before}
    if sealed_before != sealed_after:
        raise RuntimeError("Sealed classifier input changed during descriptive analysis")
    output["sealed_classifier_preserved"] = True
    A.write_new(args.output, output)
    print({"cases": len(cases), "finalized": sum("initial" in r for r in cases), "sealed_classifier_preserved": True})


if __name__ == "__main__":
    main()
