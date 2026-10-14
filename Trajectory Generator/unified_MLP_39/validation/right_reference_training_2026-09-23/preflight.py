"""Validate the native RIGHT training environment against archived V26 data.

No policy checkpoint is loaded and no training is started. Twenty archived
actions exercise dynamics twice, with prescribed inputs visible and masked.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
TG = PACKAGE.parent
REPO = TG.parent
for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ[name] = "1"
sys.path.insert(0, str(TG))
from unified_MLP_39 import train_ppo_mlp as trainer


def read(path):
    return json.loads(path.read_text())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    cli = parser.parse_args()
    if cli.output.exists():
        raise FileExistsError(cli.output)
    sys.argv = ["preflight", "--config", str(cli.config)]
    args = trainer.parse_args()
    trainer._load_training_stack()
    import numpy as np
    import yaml
    from unified_MLP_39 import env_factory, reward_function, actor_transition
    config = trainer.build_config(args, args._cfg_reward)
    assert args.target_reference == "right_v26"
    assert args.target_velocity_scale == args.target_availability_probability == 1.0
    assert not args.target_curriculum_probabilities and not args.resume_from and not args.init_actor_from
    assert args.binary_phase_actor_fsm_version == "v3"
    assert args.train_batch_size == 4096 and args.num_epochs == 10 and args.minibatch_size == 512
    assert args.num_env_runners == 13 and args.seed == 123
    assert args.episode_start_offset_s == 1.0 and not args.episode_start_offset_choices_s
    historical = TG / "runs/rollout/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter_rollout"
    trace = read(historical / "rollout_policy_trace.json")
    original = read(historical / "rollout_summary.json")
    source_cfg = yaml.safe_load((TG / "runs/training/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter/training_cfg.resolved.yaml").read_text())
    current_reward = reward_function.RewardConfig.from_mapping(args._cfg_reward)
    old_reward = reward_function.RewardConfig.from_mapping(source_cfg["reward"])
    assert old_reward.to_dict() == current_reward.to_dict()
    reconstructed = sum(reward_function.compute_reward(row["reward_terms"], current_reward,
        reference=row["policy_segment_values"])[0] for row in trace)
    assert abs(reconstructed - original["episode_return"]) < 1e-9
    env = env_factory.make_cmc_env(config.env_config)
    base = env.unwrapped
    joints = ("pros_knee_angle", "pros_ankle_angle")
    perm = [0, 1, *range(6, 39), 2, 3, 4, 5]
    max_target_error = max_clock_error = 0.0
    trials = []
    try:
        assert base.n_actor == 39 and base.n_obs == 88
        assert base.env_cfg.binary_phase_actor_fsm_version == "v3"
        for row in trace:
            q, qd, _ = base.imitation_target(row["time"])
            for joint in joints:
                for actual, expected in ((q[joint], row["imitation_target_q"][joint]),
                                          (qd[joint], row["imitation_target_qdot"][joint])):
                    max_target_error = max(max_target_error, abs(actual - expected))
                    np.testing.assert_allclose(actual, expected, atol=1e-10, rtol=0)
            phase = base._prescribed_phase_teacher.clock.phase(row["time"] - .01)
            clock = [np.sin(2*np.pi*phase), np.cos(2*np.pi*phase)]
            expected = [row["actor_observation_before"][k] for k in ("gait_phase_sin", "gait_phase_cos")]
            max_clock_error = max(max_clock_error, float(np.max(abs(np.asarray(clock) - expected))))
            np.testing.assert_allclose(clock, expected, atol=2e-6, rtol=0)
        ranges = {}
        for joint in joints:
            for label, mapping, suffix in (("position", base._imitation_position_ranges_rad, "_imitation_position_range_rad"),
                                            ("velocity", base._imitation_velocity_ranges_rad_s, "_imitation_velocity_range_rad_s")):
                expected = trace[0]["reward_terms"][joint + suffix]
                np.testing.assert_allclose(mapping[joint], expected, atol=1e-12, rtol=0)
                ranges[f"{joint}_{label}"] = mapping[joint]
        for visible in (True, False):
            base._target_availability.set_probability(float(visible))
            obs, _ = env.reset(seed=123)
            np.testing.assert_allclose(base.t, 12.99, atol=1e-10, rtol=0)
            rewards, losses, states = [], [], []
            for row in trace[:20]:
                expected = np.asarray(row["actor_observation_vector_before"], dtype=np.float32)[perm]
                if not visible:
                    expected[[0, 1, 35, 36, 37, 38]] = 0
                np.testing.assert_array_equal(obs[:39], expected)
                obs, reward, terminated, truncated, info = env.step(np.asarray(row["raw_policy_action"], dtype=np.float32))
                assert not terminated and not truncated
                assert info["imitation_target"]["source"] == "prescribed_right_phase_template"
                assert info["target_availability"]["visible"] == visible
                rewards.append(float(reward))
                losses.append([info["reward_terms"][k] for k in ("sound_imitation_loss", "served_imitation_loss")])
                states.append([float(info["observation"][j]) for j in joints])
            trials.append({"visible": visible, "steps": len(rewards), "rewards": rewards,
                           "losses": losses, "states": states})
        for field in ("rewards", "losses", "states"):
            np.testing.assert_array_equal(trials[0][field], trials[1][field])
        contract = actor_transition.build_contract(base.actor_feature_names, base.observation_feature_names,
            env.action_space.low, env.action_space.high, target_velocity_scale=1.0, target_reference="right_v26")
        assert contract == args._unified_contract
    finally:
        env.close()
    receipt = {"pass": True, "training_started": False, "reference": "right_v26", "fsm": "v3",
        "historical_template_samples": len(trace), "max_target_q_qdot_error": max_target_error,
        "max_actor_clock_error": max_clock_error, "reward_configuration_equal": True,
        "historical_return_reconstruction_error": abs(reconstructed - original["episode_return"]),
        "normalization_ranges": ranges, "dynamic_steps_per_visibility_condition": 20,
        "actor_observations_equal_historical_after_permutation": True,
        "visible_vs_masked_rewards_losses_states_exact": True, "trials": trials,
        "contract": contract, "training_env_config": config.env_config,
        "config_sha256": hashlib.sha256(cli.config.read_bytes()).hexdigest()}
    cli.output.parent.mkdir(parents=True, exist_ok=True)
    cli.output.write_text(json.dumps(receipt, indent=2, allow_nan=False) + "\n")
    print(json.dumps({k:v for k,v in receipt.items() if k not in ("trials", "contract", "training_env_config")}, indent=2))


if __name__ == "__main__":
    main()
