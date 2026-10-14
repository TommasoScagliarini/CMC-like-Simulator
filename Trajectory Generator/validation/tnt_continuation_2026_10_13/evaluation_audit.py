"""Separate retrospective evaluation corrections; never changes estimators/bundles.

All configurations were previously selected. Targets enter scoring or the
already established first-40-second template fit only. No new parameter search.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys

sys.dont_write_bytecode = True
import numpy as np

HERE = Path(__file__).resolve().parent
PREVIOUS = HERE.parent / "tnt_preliminary_2026_10_13"
sys.path.insert(0, str(PREVIOUS))
from tnt_replay import (Parameters, STANCE, EARLY, INVALID, STATE_NAMES,
                       complete_blocks, kinematic_minima, make_template, predict, write_json)
from kinematic_generator import stream_replay


def fingerprint(path):
    return {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def strict_blocks(r, mask, mode):
    events = {event["i"]: event["event"] for event in r["events"]}
    opening, closing = (("late_to_stance", "stance_to_early") if mode == STANCE
                        else ("stance_to_early", "late_to_stance"))
    return [(a, b) for a, b in complete_blocks(r, mask, mode)
            if events.get(a) == opening and events.get(b) == closing]


def block_audit(r, mask):
    out = {}
    for mode, name in [(STANCE, "stance"), (EARLY, "swing")]:
        legacy = complete_blocks(r, mask, mode)
        strict = strict_blocks(r, mask, mode)
        removed = sorted(set(legacy) - set(strict))
        event_at = {e["i"]: e["event"] for e in r["events"]}
        out[name] = {"legacy_count": len(legacy), "strict_count": len(strict),
                     "identical_blocks": legacy == strict,
                     "strict_durations_s": [float(r["t"][b] - r["t"][a]) for a, b in strict],
                     "excluded_fragments": [{"start_s": float(r["t"][a]),
                                             "end_s": float(r["t"][b]),
                                             "duration_s": float(r["t"][b]-r["t"][a]),
                                             "opening_event": event_at.get(a),
                                             "closing_event": event_at.get(b)} for a, b in removed]}
    return out


def numerical_summary(values):
    values = np.asarray(values)
    if not len(values):
        return {"count": 0, "max_abs": None, "p99_9_abs": None}
    assert np.isfinite(values).all()
    return {"count": len(values), "max_abs": np.max(np.abs(values), axis=0).tolist(),
            "p99_9_abs": np.quantile(np.abs(values), .999, axis=0).tolist()}


def continuity(r, qref, mask):
    """Pair/triple masks prevent either invalid values or cross-boundary artifacts."""
    valid = (r["state"] != INVALID) & np.isfinite(r["T"])
    if qref is not None:
        valid &= np.isfinite(qref).all(axis=1)
    selected = np.asarray(mask) & valid
    pair = selected[:-1] & selected[1:]
    delta_t = np.diff(r["t"])
    delta_T = np.diff(r["T"])
    q_delta = np.diff(qref, axis=0) if qref is not None else None
    velocity = q_delta / delta_t[:, None] if qref is not None else None
    acceleration = (np.diff(velocity, axis=0) / ((delta_t[1:]+delta_t[:-1])/2)[:, None]
                    if qref is not None else None)
    triple = pair[:-1] & pair[1:]

    def entry(m):
        item = {"delta_T": numerical_summary(delta_T[m])}
        if m.any():
            item["delta_T"].update(min_signed=float(delta_T[m].min()), max_signed=float(delta_T[m].max()))
        if qref is not None:
            item["delta_q_deg"] = numerical_summary(q_delta[m])
            item["raw_q_velocity_deg_s"] = numerical_summary(velocity[m])
        return item

    transitions = {}
    for before, after in sorted(set(zip(r["state"][:-1][pair], r["state"][1:][pair]))):
        m = pair & (r["state"][:-1] == before) & (r["state"][1:] == after)
        key = f"{STATE_NAMES[int(before)]}_to_{STATE_NAMES[int(after)]}"
        transitions[key] = entry(m)
        if qref is not None:
            # Acceleration assigned to the transition ending its second interval.
            transitions[key]["raw_q_acceleration_deg_s2"] = numerical_summary(acceleration[triple & m[1:]])
    same_swing = pair & (r["state"][:-1] > 0) & (r["state"][1:] > 0)
    out = {"selected_valid_samples": int(selected.sum()), "all_valid_pairs": entry(pair),
           "within_swing_pairs_excluding_expected_reset": entry(same_swing),
           "by_transition": transitions,
           "phase_regressions_within_swing": int(np.sum(delta_T[same_swing] < -1e-6)),
           "note": "The large negative late_to_stance deltaT is the defined phase reset; raw q derivatives are before any governor."}
    if qref is not None:
        out["raw_q_acceleration_deg_s2"] = numerical_summary(acceleration[triple])
    return out


def trajectory_metrics(r, target, qref, mask):
    valid = mask & (r["state"] != INVALID) & np.isfinite(qref).all(axis=1)
    out = {"requested_samples": int(mask.sum()), "valid_samples": int(valid.sum()),
           "invalid_or_nonfinite_samples": int((mask & ~valid).sum()),
           "valid_fraction": float(valid.sum()/mask.sum()) if mask.any() else None,
           "window_s": [float(r["t"][mask][0]), float(r["t"][mask][-1])] if mask.any() else None,
           "all_window_success": bool(mask.any() and np.all(valid[mask])), "regions": {}}
    for name, selected in [("all_valid", valid), ("stance_valid", valid & (r["state"] == STANCE)),
                           ("swing_valid", valid & (r["state"] > 0))]:
        error = qref[selected] - target[selected]
        out["regions"][name] = {"samples": int(selected.sum()),
            "rmse_deg": np.sqrt(np.mean(error**2, axis=0)).tolist() if len(error) else None,
            "bias_deg": np.mean(error, axis=0).tolist() if len(error) else None}
    out["continuity"] = continuity(r, qref, mask)
    return out


def dwell_and_coverage(r, mask, tolerance=.10, stall_seconds=2.5):
    """Independent minima are offline diagnostic labels, not estimator input."""
    t, states = r["t"], r["state"]
    boundaries = np.r_[0, np.flatnonzero(np.diff(states)) + 1, len(states)]
    selected_indices = np.flatnonzero(mask)
    lo, hi = int(selected_indices[0]), int(selected_indices[-1]) + 1
    rows = []
    for a, b in zip(boundaries[:-1], boundaries[1:]):
        left, right = max(int(a), lo), min(int(b), hi)
        if left >= right:
            continue
        end = float(t[right]) if right < len(t) else float(t[-1])
        rows.append({"state": STATE_NAMES[int(states[a])], "start_s": float(t[left]),
                     "end_s": end, "observed_duration_s": end-float(t[left]),
                     "left_censored": left != a or a == 0,
                     "right_censored": right != b or b == len(t)})
    minima_all = kinematic_minima(r)
    minima = minima_all[mask[minima_all]]
    middle = [e for e in r["events"] if e["event"] == "early_to_middle"]
    finished = {e["swing_id"]: e for e in r["events"] if e["event"] == "late_to_stance"}
    matched = {}
    spurious = []
    for event in middle:
        if not mask[event["i"]]:
            continue
        closest = int(np.argmin(abs(t[minima] - event["t"]))) if len(minima) else None
        if closest is None or abs(t[minima[closest]] - event["t"]) > tolerance + 1e-9:
            spurious.append(event)
        else:
            matched.setdefault(int(minima[closest]), []).append(event)
    missed = [int(i) for i in minima if int(i) not in matched]
    completed_matches = [i for i, events in matched.items()
                         if any(e["swing_id"] in finished for e in events)]
    intervals = []
    for i, following in zip(minima_all[:-1], minima_all[1:]):
        if not mask[i]:
            continue
        events = matched.get(int(i), [])
        closes = [finished[e["swing_id"]]["t"] for e in events if e["swing_id"] in finished]
        intervals.append({"minimum_s": float(t[i]), "next_minimum_s": float(t[following]),
                          "minimum_interval_s": float(t[following]-t[i]),
                          "matched_middle_count": len(events),
                          "completed_before_next_minimum": any(end <= t[following] for end in closes),
                          "swing_completion_times_s": closes})
    return {"stall_threshold_s": stall_seconds,
            "stall_threshold_status": "Exploratory diagnostic threshold, not a clinical bound or runtime guard",
            "longest_observed_state_s": {name: max((row["observed_duration_s"] for row in rows
                                                     if row["state"] == name), default=0.)
                                          for name in STATE_NAMES.values()},
            "stalls": [row for row in rows if row["state"] != "invalid" and row["observed_duration_s"] > stall_seconds],
            "invalid_samples": int(np.count_nonzero(mask & (states == INVALID))),
            "invalid_intervals": [row for row in rows if row["state"] == "invalid"],
            "all_state_intervals": rows,
            "raw_event_counts": {name: sum(e["event"] == name and bool(mask[e["i"]])
                                            for e in r["events"])
                                 for name in ["stance_to_early", "early_to_middle",
                                              "middle_to_late", "late_to_stance", "invalid_T_bm"]},
            "independent_minima": len(minima), "matched_middle_minima": len(matched),
            "matched_completed_minima": len(completed_matches),
            "missed_minimum_times_s": [float(t[i]) for i in missed],
            "duplicate_middle_events": sum(max(0, len(events)-1) for events in matched.values()),
            "spurious_middle_events": spurious, "match_tolerance_s": tolerance,
            "minimum_intervals": intervals,
            "note": "Middle matching is meaningful for TNT; PFD has no middle event. The +/-100ms association tolerance is a diagnostic convention, not proof that an unmatched event is physiologically absent; raw event counts are reported separately. Final incomplete intervals are censored."}


def load_template(value):
    return {key: np.asarray(value[key]) for key in ["grid", "stance", "swing"]}


def main():
    old_path, continuation_path = PREVIOUS / "results.json", HERE / "continuation_results.json"
    old = json.loads(old_path.read_text())["results"]
    new = {x["subject"]: x for x in json.loads(continuation_path.read_text())["results"]}
    protected = [PREVIOUS / "tnt_replay.py", HERE / "kinematic_generator.py", old_path,
                 HERE / "ab06_bundle.json", HERE / "ab07_bundle.json"]
    before_hashes = [fingerprint(path) for path in protected]
    subjects = []
    for original in old:
        subject = original["subject"]
        prefix = subject.lower()
        data = np.genfromtxt(PREVIOUS / f"{prefix}_prescribed.csv", delimiter=",", names=True)
        t, theta = data["time_s"], data["thigh_deg"]
        target = np.c_[data["knee_deg"], data["ankle_deg"]]
        full, cal, later = np.ones(len(t), dtype=bool), t < t[0]+40, t >= t[0]+40
        p = Parameters(**original["parameters"])
        stance_duration = original["calibration_geometry"]["initial_stance_duration_s"]
        bundle = json.loads((HERE / f"{prefix}_bundle.json").read_text())
        cases = {}
        templates = {}
        for method in ["literal", "blend", "duration"]:
            r = stream_replay(t, theta, p, method=method, initial_stance_duration=stance_duration)
            # Retain frozen curves where saved; duration uses the already-established
            # first-40-second fitting procedure, after proving block eligibility.
            audit = block_audit(r, cal)
            assert all(row["identical_blocks"] for row in audit.values()), "Calibration block correction changes this template"
            if method == "literal":
                template = load_template(json.loads((PREVIOUS / f"{prefix}_template.json").read_text()))
            elif method == "blend":
                template = load_template(bundle["template"])
            else:
                template = make_template(r, target, cal)
            cases[method] = r
            templates[method] = template
        if prefix == "ab07":
            fixed = json.loads((HERE / "fixed_scan/ab07_literal_generator.json").read_text())
            r = stream_replay(t, theta, Parameters(**fixed["parameters"]), method="literal",
                              initial_stance_duration=fixed["initial_stance_duration_s"])
            cases["literal_fixed_scan_selected"] = r
            templates["literal_fixed_scan_selected"] = load_template(fixed["template"])
        frozen_pfd_duration = original["pfd_swing_duration_s"]
        pfd = stream_replay(t, theta, p, pfd_duration=frozen_pfd_duration,
                            initial_stance_duration=stance_duration)
        pfd_calibration_blocks = block_audit(pfd, cal)
        assert all(row["identical_blocks"] for row in pfd_calibration_blocks.values())
        pfd_template = make_template(pfd, target, cal)
        pfd_prediction = predict(pfd, pfd_template)
        saved_old_trace = np.genfromtxt(PREVIOUS / f"{prefix}_replay.csv", delimiter=",", names=True)
        original_pfd_prediction = np.c_[saved_old_trace["pfd_knee_deg"], saved_old_trace["pfd_ankle_deg"]]
        pfd_reproduction_error = float(np.max(abs(original_pfd_prediction-pfd_prediction)))
        assert pfd_reproduction_error < 1e-5, "Frozen PFD baseline does not reproduce original trace"
        summaries = {}
        for method, r in cases.items():
            prediction = predict(r, templates[method])
            invalid = np.flatnonzero((r["state"] == INVALID) | ~np.isfinite(prediction).all(axis=1))
            stop = int(invalid[0]) if len(invalid) else len(t)
            matched = later & (np.arange(len(t)) < stop)
            summaries[method] = {
                "heldout_status": "reused_post_hoc",
                "calibration_blocks": block_audit(r, cal),
                "later_blocks": block_audit(r, later),
                "later_trajectory": trajectory_metrics(r, target, prediction, later),
                "full_valid_continuity": continuity(r, prediction, full),
                "full_dwell_and_coverage": dwell_and_coverage(r, full),
                "later_dwell_and_coverage": dwell_and_coverage(r, later),
                "matched_valid_prefix_comparison": {
                    "interpretation": "Conditional error before the first invalid sample; it does not excuse the remaining failure",
                    "tnt": trajectory_metrics(r, target, prediction, matched),
                    "frozen_original_pfd": trajectory_metrics(pfd, target, pfd_prediction, matched)}}
        raw = stream_replay(t, theta, Parameters(-6., 32., .8, p.initial_rate),
                            initial_stance_duration=stance_duration)
        subjects.append({"subject": subject, "input": fingerprint(PREVIOUS / f"{prefix}_prescribed.csv"),
                         "original_parameters": original["parameters"],
                         "pfd_baseline": {"frozen_original_duration_s": frozen_pfd_duration,
                             "continuation_blend_derived_duration_s": new[subject]["pfd_duration_s"],
                             "duration_difference_s": new[subject]["pfd_duration_s"]-frozen_pfd_duration,
                             "reproduction_max_error_deg_vs_original_saved_csv": pfd_reproduction_error,
                             "calibration_blocks": pfd_calibration_blocks,
                             "later_trajectory": trajectory_metrics(pfd, target, pfd_prediction, later)},
                         "cases": summaries,
                         "paper_thresholds_estimator_only": {
                             "qualification": "Untranslated paper thresholds in our angle convention; not a calibrated test of the published device",
                             "full_dwell_and_coverage": dwell_and_coverage(raw, full),
                             "later_dwell_and_coverage": dwell_and_coverage(raw, later),
                             "full_valid_phase_continuity": continuity(raw, None, full)}})
    after_hashes = [fingerprint(path) for path in protected]
    assert before_hashes == after_hashes
    result = {"scope": "Retrospective evaluation-only correction; no estimator, parameter, saved template, bundle or dynamics changes",
              "heldout_status": "reused_post_hoc",
              "corrections": ["Only genuine boundary events complete a block",
                              "Finite adjacent pairs/triples preserve prefix continuity metrics",
                              "PFD compared on exactly the same valid-prefix mask",
                              "Fixed original PFD duration, independent of later TNT rate variants",
                              "State dwell and independent-minimum misses exposed separately from INVALID"],
              "joint_order": ["knee", "ankle"], "protected_inputs_unchanged": True,
              "protected_inputs": before_hashes, "audit_source": fingerprint(Path(__file__)),
              "results": subjects}
    write_json(HERE / "evaluation_audit.json", result)
    lines = ["Retrospective evaluation audit: no operational change, no parameter search.",
             "TNT/PFD matched-prefix errors are conditional diagnostics, not whole-record success."]
    for subject in subjects:
        lines.append("\n" + subject["subject"])
        pfd = subject["pfd_baseline"]
        lines.append(f"PFD duration fixed at original {pfd['frozen_original_duration_s']:.6f}s; continuation used {pfd['continuation_blend_derived_duration_s']:.6f}s.")
        for method, case in subject["cases"].items():
            blocks = case["calibration_blocks"]
            coverage = case["full_dwell_and_coverage"]
            matches = case["matched_valid_prefix_comparison"]
            c = case["full_valid_continuity"]
            lines.append(f"{method}: calibration blocks unchanged={all(x['identical_blocks'] for x in blocks.values())}; "
                         f"middle matched {coverage['matched_middle_minima']}/{coverage['independent_minima']}, "
                         f"completed matched {coverage['matched_completed_minima']}; stalls={len(coverage['stalls'])}.")
            lines.append(f"  Matched-window RMSE knee/ankle deg: TNT={matches['tnt']['regions']['all_valid']['rmse_deg']}; "
                         f"PFD={matches['frozen_original_pfd']['regions']['all_valid']['rmse_deg']}.")
            lines.append(f"  Max raw deltaq deg={c['all_valid_pairs']['delta_q_deg']['max_abs']}; "
                         f"max raw velocity deg/s={c['all_valid_pairs']['raw_q_velocity_deg_s']['max_abs']}; "
                         f"max raw acceleration deg/s2={c['raw_q_acceleration_deg_s2']['max_abs']}.")
            removed = case["later_blocks"]["swing"]["excluded_fragments"]
            if removed:
                lines.append(f"  Excluded INVALID-truncated swing durations: {[x['duration_s'] for x in removed]}.")
    (HERE / "evaluation_audit_findings.txt").write_text("\n".join(lines)+"\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
