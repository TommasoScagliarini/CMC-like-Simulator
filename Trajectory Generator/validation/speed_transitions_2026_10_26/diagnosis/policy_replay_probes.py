"""One-variable offline probes; recorded feedback is deliberately not recomputed.

Clamp separately the prescribed clock, online phase, or actor FSM timers.
A final serving-only probe holds the target constant to check filter settling.
None of these is an intervention on the physical closed-loop system.
"""
from __future__ import annotations

import json

from replay_stop_policy import (
    BASELINE, CHECKPOINT, COORDS, HERE, RUN, build_reference_env,
    np, rollout_eval, yaml,
)


def main():
    trace = json.loads((RUN / "rollout_policy_trace.json").read_text())
    protocol = json.loads((RUN / "protocol.json").read_text())
    summary = json.loads((RUN / "rollout_summary.json").read_text())
    sim = yaml.safe_load((BASELINE / "training_exnovo_cfg.yaml").read_text())["simulation"]
    rollout_eval._load_inference_stack()
    rollout_eval.torch.set_num_threads(1)
    module = rollout_eval.RLModule.from_checkpoint(str(CHECKPOINT))
    module.eval()
    hold = next(stage for stage in protocol["stages"] if stage[0] == "hold")
    hold_trace = [row for row in trace if hold[1] <= row["policy_segment_times"][0] - protocol["start_s"] < hold[2]]
    anchor = hold_trace[0]["actor_observation_before"]
    names = summary["actor_feature_names"]
    groups = {
        "baseline": [],
        "freeze_prescribed_clock": ["gait_phase_sin", "gait_phase_cos"],
        "freeze_online_phase": ["online_left_gait_phase_sin", "online_left_gait_phase_cos"],
        "freeze_fsm_timers": ["phase_stance_elapsed_norm", "phase_swing_elapsed_norm", "phase_cycle_progress_credit"],
        "constant_target_serving_only": [],
    }
    cases = {}
    for label, clamped in groups.items():
        env = build_reference_env(trace[0], sim)
        held_action = None
        rows = []
        max_action_delta = 0.0
        for row in trace:
            env.t = row["policy_segment_times"][0]
            end = row["policy_segment_times"][-1]
            elapsed = env.t - protocol["start_s"]
            in_hold = hold[1] <= elapsed < hold[2]
            obs = np.asarray(row["actor_observation_vector_before"]).copy()
            if in_hold:
                for name in clamped:
                    obs[names.index(name)] = anchor[name]
            action = rollout_eval._policy_action(module, obs, (1, 2))
            if in_hold and label == "constant_target_serving_only":
                if held_action is None:
                    q, _, _ = env.kin.get(env.t)
                    held_action = np.array([[
                        2.0 * (q[name] - env.env_cfg.absolute_bounds_rad[name][0])
                        / (env.env_cfg.absolute_bounds_rad[name][1] - env.env_cfg.absolute_bounds_rad[name][0]) - 1.0
                        for name in COORDS
                    ]])
                action = held_action.copy()
            if in_hold:
                max_action_delta = max(max_action_delta, float(np.max(np.abs(action.ravel() - row["raw_policy_action"]))))
            times, values, derivatives = env._action_to_segment(env._validate_action(action), end)
            env.kin.set_segment(times, values, derivatives)
            env._last_policy_endpoint = values[-1].copy()
            q, v, _ = env.kin.get(end)
            raw_targets = [env.env_cfg.absolute_bounds_rad[name][0] + .5 * (action[0, j] + 1.0) * (env.env_cfg.absolute_bounds_rad[name][1] - env.env_cfg.absolute_bounds_rad[name][0]) for j, name in enumerate(COORDS)]
            rows.append([end-protocol["start_s"], *[q[name] for name in COORDS], *[v[name] for name in COORDS], *raw_targets])
        data = np.asarray(rows)
        mask = (data[:, 0] >= hold[1] + .5) & (data[:, 0] < hold[2])
        cases[label] = {
            "clamped_features": clamped,
            "clamped_values": {name: anchor[name] for name in clamped},
            "hold_action_max_difference_from_baseline": max_action_delta,
            "hold_raw_target_rom_rad": dict(zip(COORDS, np.ptp(data[mask, 5:7], axis=0).tolist())),
            "hold_served_reference_rom_rad": dict(zip(COORDS, np.ptp(data[mask, 1:3], axis=0).tolist())),
            "hold_served_velocity_rms_rad_s": dict(zip(COORDS, np.sqrt(np.mean(data[mask, 3:5]**2, axis=0)).tolist())),
        }
        np.savetxt(HERE / f"policy_replay_{label}.csv", data, delimiter=",", header="elapsed_s,knee_ref_rad,ankle_ref_rad,knee_ref_vel_rad_s,ankle_ref_vel_rad_s,knee_raw_target_rad,ankle_raw_target_rad", comments="")
    result = {
        "limitations": "Offline sensitivity and governor checks on recorded actor feedback. No recomputation of mechanics, contact, FSM events or observations; no claim that feature clamping makes a physical prosthesis stand.",
        "hold_metrics_discard_initial_s": .5,
        "actor_feature_names": names,
        "n_actor_loaded_checkpoint": module._n_actor,
        "hold_temporal_feature_ranges": {
            name: [min(row["actor_observation_before"][name] for row in hold_trace), max(row["actor_observation_before"][name] for row in hold_trace)]
            for name in sorted(set(name for features in groups.values() for name in features))
        },
        "cases": cases,
    }
    (HERE / "policy_replay_probes.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    assert cases["freeze_prescribed_clock"]["hold_action_max_difference_from_baseline"] == 0.0
    assert max(cases["constant_target_serving_only"]["hold_served_reference_rom_rad"].values()) < .02
    assert max(cases["constant_target_serving_only"]["hold_served_velocity_rms_rad_s"].values()) < .05


if __name__ == "__main__":
    main()
