"""One-variable diagnostics of production binary V26 + actor v3 stop behavior.

This is a synthetic sensor replay, not a dynamical standing test. Production
sources and the previous test's evidence are read only. The longer-timeout case
is an isolated diagnostic: it also changes elapsed-time observation scaling,
and is not a proposed safe policy configuration.
"""
from __future__ import annotations

from dataclasses import asdict, replace
import hashlib
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import fsm_probe as probe
import numpy as np


def compact(case):
    first = case["first_timeout"]
    payload = case["final_actor_payload"]
    return {
        "first_timeout": None if first is None else {
            "time_s": first["time_s"], "stage": first["stage"],
            "stance_elapsed_s": first["payload"]["stance_elapsed_s"],
            "last_valid_hs_time_s": first["payload"].get("last_valid_hs_time_s"),
        },
        "recovery_after_timeout_diagnostic_only": case["first_recovery_after_timeout_diagnostic_only"],
        "transfer_failure": case["transfer_failure"],
        "stage_end_snapshots": case["stage_end_snapshots"],
        "transitions": case["transitions"],
        "final_actor_state": payload["state_name"],
        "valid_cycles": payload["valid_cycle_count"],
        "invalid_events": payload["invalid_event_count"],
        "final_binary_contact": case["final_binary_payload"]["stable_contact_state"],
        "final_binary_gait_phase": case["final_binary_payload"]["gait_phase"],
        "would_real_episode_terminate": case["would_real_episode_terminate"],
    }


def fixed_baseline(actor_cfg, binary_cfg, contact, duration=3.0):
    adapter = probe.BinaryPhaseActiveAdapterV26()
    candidate = adapter.prime(
        binary_fsm=probe.HeelQualifiedBinaryPhaseFSM(binary_cfg),
        phase_fsm=probe.ProstheticPhaseFSM(actor_cfg), time_s=0.0,
        heel_contact=contact, toe_contact=contact,
    )
    initial = candidate.phase_payload["state_name"]
    first_timeout = None
    events = []
    snapshots = {}
    for step in range(1, round(duration / .01)+1):
        candidate = adapter.advance(
            binary_fsm=candidate.binary_fsm, phase_fsm=candidate.phase_fsm,
            time_s=step/100., previous_time_s=(step-1)/100.,
            sensor_samples=[{"time_s": i/1000., "left_heel_contact": contact,
                             "left_toe_contact": contact}
                            for i in range((step-1)*10+1, step*10+1)],
            normal_force_bw=.8 if contact else 0.0, in_contact=contact,
            prosthetic_knee_angle_rad=-.1, prosthetic_ankle_angle_rad=0.0,
            invalid_event_mode="drop",
        )
        events.extend(candidate.binary_payload["events_this_step"])
        if candidate.phase_payload["timeout_exceeded"] and first_timeout is None:
            first_timeout = step/100.
        if step in (100, 150, 200, 300):
            snapshots[str(step/100.)] = candidate.phase_payload
    return {"initial_actor_state": initial, "first_timeout_s": first_timeout,
            "final_actor_state": candidate.phase_payload["state_name"],
            "final_binary_contact": candidate.binary_payload["stable_contact_state"],
            "binary_event_count": len(events),
            "diagnostic_payloads": snapshots,
            "meaning": "constant loaded stance" if contact else "constant airborne WAIT_HS, not standing"}


def trace_clocks():
    folder = HERE.parent / "stop_restart_aligned"
    spec = json.loads((folder / "protocol.json").read_text())
    trace = json.loads((folder / "rollout_policy_trace.json").read_text())
    hold = next(x for x in spec["stages"] if x[0] == "hold")
    # actor_observation_before belongs to segment start, not row's end time.
    rows = [r for r in trace if hold[1] <= r["policy_segment_times"][0]-spec["start_s"] <= hold[2]]
    obs = [r["actor_observation_before"] for r in rows]
    result = {"samples": len(rows), "elapsed_start_s": rows[0]["policy_segment_times"][0]-spec["start_s"],
              "elapsed_end_s": rows[-1]["policy_segment_times"][0]-spec["start_s"]}
    for prefix in ("gait_phase", "online_left_gait_phase"):
        phase = np.unwrap(np.arctan2([r[prefix+"_sin"] for r in obs], [r[prefix+"_cos"] for r in obs])) / (2*np.pi)
        result[prefix] = {"unwrapped_advance_cycles": float(phase[-1]-phase[0]),
                          "sin_range": float(np.ptp([r[prefix+"_sin"] for r in obs])),
                          "cos_range": float(np.ptp([r[prefix+"_cos"] for r in obs]))}
    result["fsm_state_counts"] = {name: sum(bool(r[name]) for r in obs) for name in (
        "phase_fsm_wait_hs", "phase_fsm_stance_after_hs", "phase_fsm_swing_after_to")}
    result["phase_stance_elapsed_norm_range"] = [min(r["phase_stance_elapsed_norm"] for r in obs), max(r["phase_stance_elapsed_norm"] for r in obs)]
    return result


def main():
    actor, binary = probe.configuration()
    schedule = [("walk_before", probe.PERIOD*2+.4, 1.), ("hold", 3., 0.),
                ("walk_after", probe.PERIOD*2, 1.)]
    relaxed = replace(actor, stance_hard_timeout_s=10.)
    cases = {
        "constant_default": compact(probe.run_case("constant", [("walk", 9.72, 1.)], actor, binary)),
        "stop_default": compact(probe.run_case("stop", schedule, actor, binary)),
        "stop_stance_timeout_10s": compact(probe.run_case("stop", schedule, relaxed, binary)),
        "constant_loaded_baseline": fixed_baseline(actor, binary, True),
        "constant_loaded_baseline_timeout_10s": fixed_baseline(relaxed, binary, True),
        "constant_airborne_baseline": fixed_baseline(actor, binary, False),
    }
    assert cases["constant_default"]["first_timeout"] is None
    assert cases["stop_default"]["first_timeout"] is not None
    assert cases["stop_stance_timeout_10s"]["first_timeout"] is None
    assert cases["stop_stance_timeout_10s"]["transfer_failure"] is None
    assert cases["constant_loaded_baseline"]["binary_event_count"] == 0
    assert cases["constant_loaded_baseline"]["first_timeout_s"] is not None
    assert cases["constant_loaded_baseline_timeout_10s"]["first_timeout_s"] is None
    sources = [Path(__file__), Path(probe.__file__), probe.TRAJECTORY / "phase_tracking/binary_phase.py",
               probe.TRAJECTORY / "phase_tracking/prosthetic_phase_fsm.py", probe.PRESET]
    result = {"scope": "synthetic contacts through unchanged production FSMs; no policy or dynamics",
              "long_timeout_caveat": "diagnostic only; changes elapsed-time normalization too; does not prove network can stop",
              "actor_config": asdict(actor), "changed_variable": {"stance_hard_timeout_s": [2.2, 10.0]},
              "source_sha256": {str(p.relative_to(probe.TRAJECTORY)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
              "cases": cases, "actual_rollout_hold_actor_clocks": trace_clocks()}
    (HERE / "fsm_diagnosis.json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
    for name, case in cases.items():
        print(name, json.dumps({k: v for k, v in case.items() if k not in ("transitions", "stage_end_snapshots", "diagnostic_payloads")}))
    print("actual_rollout_hold_actor_clocks", json.dumps(result["actual_rollout_hold_actor_clocks"]))


if __name__ == "__main__":
    main()
