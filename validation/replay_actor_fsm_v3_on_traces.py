#!/usr/bin/env python3
"""L1 — replay the actor FSM (v2 vs v3) on recorded V26 rollout traces.

The detector is unchanged by v3, so the faithful replay level is the actor
FSM: inputs are the adapter-delivered left events (accepted ones recorded in
``online_events``; rejected ones reconstructed from the recorded payload's
``invalid_event_type`` at the rejection step), the detector heel/toe loads as
stable-contact truth, the primary GRF for load evidence and the prosthetic
angles. The recorded ``phase_fsm`` payload is the ground truth for the v2
replay fidelity check; v3 is then measured against v2 on the same inputs.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from fsm_v3_scenarios import make_fsm  # noqa: E402
from prosthetic_phase_fsm import (  # noqa: E402
    BINARY_ACTIVE_V26_ADAPTER_SOURCE,
    BINARY_ACTIVE_V26_EVENT_CONTRACT_ID,
    STANCE_AFTER_HS,
    SWING_AFTER_TO,
    WAIT_HS,
)

REPO = HERE.parent
ROLLOUT_ROOT = REPO / "Trajectory Generator" / "runs" / "rollout"
BODY_WEIGHT_N = 735.0  # AB06 approx; only scales load evidence (gates are 0)
DT = 0.010
REJECTED_TO_TYPES = {
    "to_too_early_after_hs", "to_before_hs", "double_to_before_hs",
    "stance_contact_too_low", "stance_load_too_low",
}
REJECTED_HS_TYPES = {"hs_too_early_after_to", "double_hs_before_to"}
FSM_KEYS = {
    "min_stance_duration_s": "phase_min_stance_duration_s",
    "min_swing_duration_s": "phase_min_swing_duration_s",
    "landing_window_start_s": "phase_landing_window_start_s",
    "landing_window_end_s": "phase_landing_window_end_s",
    "stance_hard_timeout_s": "phase_stance_hard_timeout_s",
    "swing_hard_timeout_s": "phase_swing_hard_timeout_s",
    "hs_event_credit": "phase_hs_event_credit",
    "toe_off_event_credit": "phase_to_event_credit",
    "cycle_complete_bonus": "phase_cycle_complete_bonus",
    "failure_extra_penalty": "phase_failure_extra_penalty",
    "min_stance_load_bw_s": "phase_min_stance_load_bw_s",
    "min_stance_contact_fraction": "phase_min_stance_contact_fraction",
    "min_cycle_knee_excursion_rad": "phase_min_cycle_knee_excursion_rad",
    "landing_force_full_credit_bw": "contact_load_target_bw",
}


def load_sto(path: Path):
    lines = path.read_text().splitlines()
    for i, line in enumerate(lines):
        if line.startswith("time"):
            labels = line.split("\t")
            data = np.array([[float(x) for x in r.split("\t")] for r in lines[i + 1 :] if r])
            return labels, data
    raise ValueError(path)


def fsm_overrides(summary: dict) -> dict:
    def find(key):
        stack = [summary]
        while stack:
            node = stack.pop()
            if isinstance(node, dict):
                if key in node and isinstance(node[key], (int, float)):
                    return float(node[key])
                stack.extend(node.values())
        return None
    out = {}
    for cfg_key, sum_key in FSM_KEYS.items():
        value = find(sum_key)
        if value is not None:
            out[cfg_key] = value
    return out


def adapter_event(name: str, onset: float, boundary: float) -> dict:
    return {
        "side": "left", "event": name,
        "event_time_s": onset, "confirmed_time_s": onset + 0.005,
        "delivered_time_s": boundary,
        "source": BINARY_ACTIVE_V26_ADAPTER_SOURCE,
        "event_contract_id": BINARY_ACTIVE_V26_EVENT_CONTRACT_ID,
    }


def step_inputs(trace: list, grf_labels, grf_data, *, initial_latch: bool):
    """Yield per-step (boundary, events, binary_contact, in_contact, force_bw, knee, ankle, recorded)."""
    i_t, i_ic, i_nf = 0, grf_labels.index("left_in_contact"), grf_labels.index("left_normal_force")
    state = {"latch": bool(initial_latch)}
    times = grf_data[:, i_t]
    for row in trace:
        boundary = round(float(row["time"]), 6)
        events = []
        for e in row.get("online_events") or []:
            if e.get("side") == "left" and "delivered_time_s" in e:
                events.append(adapter_event(str(e["event"]), round(float(e["event_time_s"]), 6), boundary))
        recorded = row["phase_fsm"]
        itype = str(recorded.get("invalid_event_type", ""))
        if float(recorded.get("invalid_event_this_step", 0.0)) > 0.0:
            name = "toe_off" if itype in REJECTED_TO_TYPES else ("heel_strike" if itype in REJECTED_HS_TYPES else None)
            if name is not None:
                onset = round(boundary - 0.009, 6)
                while any(abs(onset - ev["event_time_s"]) < 1e-6 for ev in events):
                    onset = round(onset + 0.0005, 6)
                events.append(adapter_event(name, onset, boundary))
        events.sort(key=lambda ev: ev["event_time_s"])
        # Detector functional latch reconstructed from every detector event
        # delivered so far (accepted + rejected): HS -> stance, TO -> swing.
        for ev in events:
            state["latch"] = ev["event"] == "heel_strike"
        binary_contact = bool(state["latch"])
        k = int(np.clip(np.searchsorted(times, boundary), 0, len(times) - 1))
        in_contact = bool(grf_data[k, i_ic] > 0.5)
        recorded_bw = (row.get("reward_terms") or {}).get("prosthetic_normal_force_bw")
        force_bw = (
            max(0.0, float(recorded_bw))
            if recorded_bw is not None
            else max(0.0, float(grf_data[k, i_nf])) / BODY_WEIGHT_N
        )
        ps = row.get("prosthetic_state") or {}
        yield (boundary, tuple(events), binary_contact, in_contact, force_bw,
               ps.get("pros_knee_angle"), ps.get("pros_ankle_angle"), recorded)


def run_version(trace, grf, overrides, *, v3: bool):
    labels, data = grf
    flags = dict(resync_enabled=v3, hs_cancel_enabled=v3)
    fsm = make_fsm(**overrides, **flags)
    first_boundary = round(float(trace[0]["time"]), 6)
    reset_time = round(first_boundary - DT, 6)
    rec0 = trace[0]["phase_fsm"]
    primed_in_stance = bool(float(rec0.get("sensor_partial_stance_active", 0.0)) > 0.0) or (
        rec0.get("state_name") == "STANCE_AFTER_HS" and float(rec0.get("valid_hs_count", 0.0)) == 0.0
    )
    fsm.reset_from_binary_baseline(time_s=reset_time, in_contact=primed_in_stance)
    desync_steps, state_match, payloads = 0, 0, []
    for boundary, events, bcontact, icontact, fbw, knee, ankle, recorded in step_inputs(
        trace, labels, data, initial_latch=primed_in_stance
    ):
        try:
            p = fsm.update_from_binary_events(
                time_s=boundary, previous_time_s=round(boundary - DT, 6), events=events,
                normal_force_bw=fbw, in_contact=icontact,
                prosthetic_knee_angle_rad=knee, prosthetic_ankle_angle_rad=ankle,
                binary_contact=bcontact,
            )
        except ValueError as exc:
            return {"error": f"{type(exc).__name__}: {exc}", "steps": len(payloads)}
        payloads.append(p)
        s = fsm.state_id
        if (s == STANCE_AFTER_HS and not bcontact) or (s in (SWING_AFTER_TO, WAIT_HS) and bcontact):
            desync_steps += 1
        state_match += int(p["state_name"] == recorded.get("state_name"))
        if p["state_name"] == "TIMEOUT":
            break
    last = payloads[-1]
    return {
        "steps": len(payloads),
        "final_state": last["state_name"],
        "valid_cycles": last["valid_cycle_count"],
        "valid_hs": last["valid_hs_count"],
        "invalid_events": last["invalid_event_count"],
        "resync": last.get("resync_count", 0.0),
        "hs_cancelled": last.get("hs_cancelled_count", 0.0),
        "desync_s": round(desync_steps * DT, 3),
        "state_match_vs_recorded": round(state_match / max(1, len(payloads)), 4),
        "cycle_bonus_total": round(sum(p["phase_cycle_complete_bonus"] for p in payloads), 3),
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--rollouts", nargs="*", default=None)
    ap.add_argument("--out", default=str(HERE / "actor_fsm_v3_replay_receipt.json"))
    args = ap.parse_args()
    dirs = [ROLLOUT_ROOT / d for d in args.rollouts] if args.rollouts else sorted(
        d for d in ROLLOUT_ROOT.iterdir() if (d / "rollout_policy_trace.json").is_file()
    )
    receipt = {}
    for d in dirs:
        trace = json.loads((d / "rollout_policy_trace.json").read_text())
        if not trace or trace[min(10, len(trace) - 1)].get("phase_fsm", {}).get("event_source") != "binary_active_v26":
            continue
        summary = json.loads((d / "rollout_summary.json").read_text())
        overrides = fsm_overrides(summary)
        grf = load_sto(d / "sim_outputs" / "rollout_episode_online_grf.sto")
        res = {
            "fsm_overrides": overrides,
            "recorded": {
                "steps": len(trace),
                "final_state": trace[-1]["phase_fsm"].get("state_name"),
                "valid_cycles": trace[-1]["phase_fsm"].get("valid_cycle_count"),
                "invalid_events": trace[-1]["phase_fsm"].get("invalid_event_count"),
            },
            "v2": run_version(trace, grf, overrides, v3=False),
            "v3": run_version(trace, grf, overrides, v3=True),
        }
        receipt[d.name] = res
        r, v2, v3 = res["recorded"], res["v2"], res["v3"]
        print(f"\n== {d.name} (swing_to={overrides.get('swing_hard_timeout_s')}) ==")
        print(f"  recorded : steps {r['steps']:4d} end {r['final_state']:16s} cycles {r['valid_cycles']} invalid {r['invalid_events']}")
        for tag, v in (("v2 replay", v2), ("v3 replay", v3)):
            if "error" in v:
                print(f"  {tag}: ERROR {v['error']}")
                continue
            print(f"  {tag}: steps {v['steps']:4d} end {v['final_state']:16s} cycles {v['valid_cycles']} invalid {v['invalid_events']} "
                  f"resync {v['resync']} cancel {v['hs_cancelled']} desync {v['desync_s']}s match {v['state_match_vs_recorded']}")
    Path(args.out).write_text(json.dumps(receipt, indent=1, allow_nan=False))
    print("\nreceipt:", args.out)


if __name__ == "__main__":
    main()
