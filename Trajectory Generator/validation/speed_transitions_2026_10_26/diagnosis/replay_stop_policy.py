"""Replay captured actor inputs through the real policy and reference serving.

Diagnostic only: feedback observations are frozen from the physical rollout.
This is a reference-generation reproduction, not a new closed-loop simulation.
The optional --assert-still checks the requested stopped behaviour and currently
fails; it is intentionally kept outside the production test suite.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from pathlib import Path
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
BASELINE = ROOT / "Trajectory Generator/baseline_MLP"
sys.path.insert(0, str(BASELINE))
import _bootstrap

_bootstrap.ensure_sim_paths()
import win_runtime  # noqa: F401
import numpy as np
import yaml
import rollout_eval
from osim_trj_cmc_like import (
    CMCEnvConfig,
    CMCLikeProsthesisTrajectoryEnv,
    ProstheticSegmentKinematics,
)

COORDS = ("pros_knee_angle", "pros_ankle_angle")
CHECKPOINT = ROOT / "Trajectory Generator/runs/training/MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-01/rl_module_best"
RUN = HERE.parent / "stop_restart_aligned"


def build_reference_env(initial, sim):
    """Minimal object using the production mapping and reference governor."""
    class InitialKinematics:
        coord_names = list(COORDS)

        def get(self, _time):
            return (
                dict(zip(COORDS, initial["policy_segment_values"][0])),
                dict(zip(COORDS, initial["policy_segment_derivatives"][0])),
                dict.fromkeys(COORDS, 0.0),
            )

    env = CMCLikeProsthesisTrajectoryEnv.__new__(CMCLikeProsthesisTrajectoryEnv)
    env.env_cfg = CMCEnvConfig(policy_knots=1, action_mode="absolute")
    env.cfg = SimpleNamespace(pros_coords=COORDS, dt=0.001)
    env._target_slew_rate_limit = np.array([
        sim["pros_knee_target_slew_rate_limit_rad_s"],
        sim["pros_ankle_target_slew_rate_limit_rad_s"],
    ])
    env._last_policy_endpoint = np.asarray(initial["policy_segment_values"][0])
    env.kin = ProstheticSegmentKinematics(
        InitialKinematics(), COORDS,
        ref_model=sim["pros_ref_model"],
        ref_lpf_cutoff_hz=sim["pros_ref_cutoff_hz"],
        ref_governor_enable=sim["pros_ref_governor"],
        ref_velocity_limits=[sim[f"pros_{joint}_ref_velocity_limit_rad_s"] for joint in ("knee", "ankle")],
        ref_acceleration_limits=[sim[f"pros_{joint}_ref_acceleration_limit_rad_s2"] for joint in ("knee", "ankle")],
        ref_jerk_limits=[sim[f"pros_{joint}_ref_jerk_limit_rad_s3"] for joint in ("knee", "ankle")],
    )
    return env


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--assert-still", action="store_true")
    args = parser.parse_args()
    started = time.perf_counter()
    trace = json.loads((RUN / "rollout_policy_trace.json").read_text())
    protocol = json.loads((RUN / "protocol.json").read_text())
    sim = yaml.safe_load((BASELINE / "training_exnovo_cfg.yaml").read_text())["simulation"]
    rollout_eval._load_inference_stack()
    rollout_eval.torch.set_num_threads(1)
    module = rollout_eval.RLModule.from_checkpoint(str(CHECKPOINT))
    module.eval()
    env = build_reference_env(trace[0], sim)
    action_error, segment_error, served_error = [], [], []
    rows = []
    for item in trace:
        action = rollout_eval._policy_action(
            module, np.asarray(item["actor_observation_vector_before"]), (1, 2)
        )
        action_error.append(float(np.max(np.abs(action.ravel() - item["raw_policy_action"]))))
        env.t = item["policy_segment_times"][0]
        end = item["policy_segment_times"][-1]
        times, values, derivatives = env._action_to_segment(env._validate_action(action), end)
        segment_error.append(float(np.max(np.abs(values - item["policy_segment_values"]))))
        env.kin.set_segment(times, values, derivatives)
        env._last_policy_endpoint = values[-1].copy()
        q, v, _ = env.kin.get(end)
        served = [q[name] for name in COORDS]
        served_error.append(float(np.max(np.abs(np.array(served) - [item["prosthetic_state"][name + "_served_ref"] for name in COORDS]))))
        rows.append([end - protocol["start_s"], *served, *[v[name] for name in COORDS]])

    data = np.asarray(rows)
    hold = next(stage for stage in protocol["stages"] if stage[0] == "hold")
    mask = (data[:, 0] >= hold[1] + 0.5) & (data[:, 0] < hold[2])
    rom = np.ptp(data[mask, 1:3], axis=0)
    rms = np.sqrt(np.mean(data[mask, 3:5] ** 2, axis=0))
    result = {
        "kind": "frozen-observation replay of actual inference, action mapping and reference governor",
        "limitations": "Does not re-integrate mechanics or recompute observations; not a counterfactual closed-loop rollout.",
        "samples": len(trace), "hold_samples_after_0p5s": int(mask.sum()),
        "max_action_difference_from_rollout": max(action_error),
        "max_segment_difference_rad": max(segment_error),
        "max_served_reference_difference_rad": max(served_error),
        "hold_served_reference_rom_rad": dict(zip(COORDS, rom.tolist())),
        "hold_served_reference_velocity_rms_rad_s": dict(zip(COORDS, rms.tolist())),
        "diagnostic_still_thresholds": {"rom_rad": 0.02, "velocity_rms_rad_s": 0.05},
        "still": bool(np.all(rom <= 0.02) and np.all(rms <= 0.05)),
        "elapsed_s_excluding_imports": time.perf_counter() - started,
        "checkpoint_state_sha256": hashlib.sha256((CHECKPOINT / "module_state.pkl").read_bytes()).hexdigest(),
    }
    (HERE / "policy_replay.json").write_text(json.dumps(result, indent=2) + "\n")
    np.savetxt(HERE / "policy_replay.csv", data, delimiter=",", header="elapsed_s,knee_ref_rad,ankle_ref_rad,knee_ref_vel_rad_s,ankle_ref_vel_rad_s", comments="")
    print(json.dumps(result, indent=2))
    assert max(action_error) < 1e-6, "Replay does not match saved policy actions"
    assert max(segment_error) < 1e-5, "Replay does not match saved action segments"
    assert max(served_error) < 1e-5, "Replay does not match saved served references"
    if args.assert_still:
        assert result["still"], "STOP FAILED: re-inferred policy and real reference governor keep moving during the stopped interval"


if __name__ == "__main__":
    main()
