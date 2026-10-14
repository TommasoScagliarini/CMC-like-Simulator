"""Offline event/cycle audit of deterministic rollouts; no FSM replay or edits."""
import hashlib
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[2]
TG = PACKAGE.parent
REPO = TG.parent
FOLDERS = {
    "right_best35": PACKAGE / "runs/rollout/MLP39_right_v26_40iter_2026-09-23_best35",
    "right_last40": PACKAGE / "runs/rollout/MLP39_right_v26_40iter_2026-09-23_last40",
    "left_last40": PACKAGE / "runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_40iter_2026-09-21_last40",
    "left_best72": PACKAGE / "runs/rollout/MLP39_completion_2026-09-22_best072/deterministic",
    "v26_best87": TG / "runs/rollout/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter_rollout",
}
HASHES = {}


def read(path):
    data = path.read_bytes()
    HASHES[str(path.relative_to(REPO))] = hashlib.sha256(data).hexdigest()
    return json.loads(data)


def grf(folder):
    path = folder / "sim_outputs/rollout_episode_online_grf.sto"
    data = path.read_bytes()
    HASHES[str(path.relative_to(REPO))] = hashlib.sha256(data).hexdigest()
    lines = data.decode().splitlines()
    header = lines.index("endheader") + 1
    names = lines[header].split()
    values = np.array([[float(x) for x in line.split()] for line in lines[header + 1:] if line.strip()])
    return {k: values[:, i] for i, k in enumerate(names)}


def contact_window(table, start, end):
    mask = (table["time"] >= start - 1e-8) & (table["time"] <= end + 1e-8)
    force = table["left_normal_force"][mask]
    return {
        "time_range_s": [start, end], "samples": len(force),
        "normal_force_min_n": float(force.min()), "normal_force_max_n": float(force.max()),
        "normal_force_mean_n": float(force.mean()),
        "fraction_force_over_5n": float(np.mean(force > 5)),
        "fraction_force_over_50n": float(np.mean(force > 50)),
        "penetration_max_mm": float(table["left_penetration"][mask].max() * 1000),
        "scope": "Applied primary GRF, distinct from detector functional stance latch used for resync.",
    }


def audit(folder):
    rows = read(folder / "rollout_policy_trace.json")
    summary = read(folder / "rollout_summary.json")
    table = grf(folder)
    events, transitions, valid_cycles = [], [], []
    for r in rows:
        f = r["phase_fsm"]
        for event in f.get("accepted_transitions_this_step", []):
            transitions.append({**event, "closed_segment_duration_s": event["segment_end_time_s"] - event["segment_start_time_s"]})
        interesting = any(f.get(k) for k in (
            "accepted_transitions_this_step", "invalid_event_this_step", "resync_event_this_step",
            "cycle_rejected_this_step", "cycle_completed_this_step"))
        if not interesting:
            continue
        item = {"step": r["step"], "recorded_boundary_time_s": r["time"],
                "state_name": f["state_name"], "accepted_transitions": f.get("accepted_transitions_this_step", []),
                "invalid_event_type": f.get("invalid_event_type"),
                "cycle_reject_reason": f.get("cycle_reject_reason"),
                "valid_cycle_count": f.get("valid_cycle_count"),
                "cycle_degraded": f.get("cycle_degraded"),
                "resync_count": f.get("resync_count"),
                "applied_normal_force_n_at_reward_sample": r["reward_terms"].get("grf_ankle_moment_flip_force_n"),
                "detector_sensors": r.get("detector_sensors")}
        if f.get("invalid_event_this_step"):
            item["previous_accepted_to_time_s"] = f["last_valid_to_time_s"]
            item["boundary_minus_previous_to_s"] = r["time"] - f["last_valid_to_time_s"]
        if f.get("resync_event_this_step"):
            ev = f["accepted_transitions_this_step"][-1]
            item["primary_contact_over_resync_dwell"] = contact_window(table, ev["event_time_s"], ev["delivered_time_s"])
        if f.get("cycle_completed_this_step"):
            valid_cycles.append({"recorded_boundary_time_s": r["time"], "end_hs_time_s": f["last_valid_hs_time_s"],
                "period_s": f["last_period_s"], "start_hs_time_s": f["last_valid_hs_time_s"] - f["last_period_s"],
                "stance_fraction": f["last_stance_fraction"],
                "stance_duration_s": f["last_period_s"] * f["last_stance_fraction"],
                "swing_duration_s": f["last_period_s"] * (1 - f["last_stance_fraction"])})
        events.append(item)
    last = rows[-1]["phase_fsm"]
    cfg = summary["reward_config"]
    result = {
        "rollout": str(folder.relative_to(REPO)), "steps": len(rows), "end_reason": summary["end_reason"],
        "final_fsm": {k: last.get(k) for k in ("valid_hs_count", "valid_to_count", "valid_cycle_count",
            "invalid_event_count", "resync_count", "hs_cancelled_count", "cycle_degraded", "fsm_behaviour_version")},
        "valid_cycles": valid_cycles, "events": events, "accepted_transitions": transitions,
        "cycle_rejection_flags_by_reason": {reason: sum(r["phase_fsm"].get("cycle_reject_reason") == reason for r in rows)
            for reason in sorted({r["phase_fsm"].get("cycle_reject_reason") for r in rows} - {"", None})},
        "event_and_phase_reward_configuration": {k: cfg.get(k) for k in (
            "blend_phase_regular", "blend_phase_event_progress", "blend_landing_window_contact",
            "phase_invalid_event_weight", "phase_contact_validity_weight", "phase_clawback_penalty_weight",
            "phase_timeout_penalty_weight", "phase_min_swing_duration_s", "phase_stance_hard_timeout_s", "phase_swing_hard_timeout_s")},
    }
    # Metadata absence is preserved for the historical rollout rather than filled with zero.
    assert len(valid_cycles) == int(last["valid_cycle_count"])
    return result


def main():
    HERE.mkdir(parents=True, exist_ok=True)
    result = {"rollouts": {name: audit(folder) for name, folder in FOLDERS.items()},
        "method": "Trace event timing, accepted segment/cycle records and stored primary GRF only. No physics simulation and no FSM replay.",
        "cautions": [
            "Invalid-event times are recorded policy boundaries; original detector event timestamps are not retained for rejected events.",
            "Resync anchor is onset of detector/FSM disagreement, not necessarily the physical heel-strike timestamp.",
            "Primary force and legacy heel/toe sensors are different signals from the binary detector functional stance latch.",
            "resync and following cycle_degraded flags are related rejections, not independent physical cycles.",
            "One valid cycle means one accepted non-degraded HS-to-HS cycle; the limb can continue moving through rejected cycles.",
            "No counterfactual V2 replay is used; not evidence that the V3 recovery caused the physical irregularity or saved a failing episode.",
        ]}
    for path in (TG / "prosthetic_phase_fsm.py", TG / "binary_phase_adapter.py"):
        HASHES[str(path.relative_to(REPO))] = hashlib.sha256(path.read_bytes()).hexdigest()
    (HERE / "metrics.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    (HERE / "input_hashes.json").write_text(json.dumps(HASHES, indent=2) + "\n")
    print(json.dumps({name: {"final": r["final_fsm"], "valid_cycles": r["valid_cycles"],
                            "rejection_flags": r["cycle_rejection_flags_by_reason"]}
                      for name, r in result["rollouts"].items()}, indent=2))


if __name__ == "__main__":
    main()
