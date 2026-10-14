"""Small OpenSim check: prescribed-left supervision survives target masking.

No optimizer or Ray workers are started. Writes a receipt only when all three
starts pass the observation, target, loss and clock assertions.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE.parent))
from unified_MLP_39._bootstrap import ensure_sim_paths

ensure_sim_paths()
from baseline_MLP import win_runtime  # noqa: E402,F401
import numpy as np  # noqa: E402
from unified_MLP_39 import env_factory, train_ppo_mlp as trainer  # noqa: E402
from unified_MLP_39.phase_target import COORDINATES, TARGET_SOURCE  # noqa: E402
from unified_MLP_39.target_availability import velocity_scale  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=PACKAGE / "configs/smoke_pretraining_visible_targets.yaml")
    parser.add_argument("--target-velocity-scale", type=velocity_scale)
    cli = parser.parse_args()
    if cli.output.exists():
        raise FileExistsError(cli.output)
    sys.argv = ["verify", "--config", str(cli.config)]
    if cli.target_velocity_scale is not None:
        sys.argv.extend(["--target-velocity-scale", str(cli.target_velocity_scale)])
    args = trainer.parse_args()
    trainer._load_training_stack()
    config = trainer.build_config(args, args._cfg_reward)
    scale = args.target_velocity_scale
    results = []
    for offset in args.episode_start_offset_choices_s:
        options = {**config.env_config, "episode_start_offset_choices_s": [],
                   "episode_start_offset_s": offset}
        env = env_factory.make_cmc_env(options)
        try:
            base = env.unwrapped
            samples = []
            for probability in (1.0, 0.0):
                base._target_availability.set_probability(probability)
                obs, _ = env.reset(seed=123)
                q, qdot, phase = base.imitation_target(base.t)
                reference = np.array([q[COORDINATES[0]], qdot[COORDINATES[0]] / scale,
                                      q[COORDINATES[1]], qdot[COORDINATES[1]] / scale])
                np.testing.assert_allclose(obs[35:39], reference if probability else np.zeros(4), atol=1e-7)
                angle = 2 * np.pi * phase[COORDINATES[0]]
                np.testing.assert_allclose(obs[:2], [np.sin(angle), np.cos(angle)] if probability else [0, 0], atol=1e-7)
                assert not base._gait_clock.available
                assert base._prescribed_left_teacher.clock.available
                assert np.isfinite(obs).all()
                post_obs, reward, terminated, truncated, info = env.step(np.zeros(2, dtype=np.float32))
                payload = info["imitation_target"]
                assert payload["source"] == TARGET_SOURCE
                post_target = base.imitation_target(base.t)
                assert payload["q"] == post_target[0] and payload["qdot"] == post_target[1]
                assert payload["phase"] == post_target[2]
                angle = 2 * np.pi * post_target[2][COORDINATES[0]]
                np.testing.assert_allclose(post_obs[:2], [np.sin(angle), np.cos(angle)] if probability else [0, 0], atol=1e-7)
                expected = [post_target[0][COORDINATES[0]], post_target[1][COORDINATES[0]] / scale,
                            post_target[0][COORDINATES[1]], post_target[1][COORDINATES[1]] / scale]
                np.testing.assert_allclose(post_obs[35:39], expected if probability else np.zeros(4), atol=1e-7)
                assert np.isfinite(reward) and not terminated and not truncated
                samples.append({"probability": probability, "time": base.t,
                                "target": payload, "reward": float(reward),
                                "prescribed_inputs": post_obs[[0, 1, 35, 36, 37, 38]].tolist(),
                                "losses": {key: info["reward_terms"][key] for key in
                                           ("sound_imitation_loss", "served_imitation_loss")}})
            assert samples[0]["target"] == samples[1]["target"]
            np.testing.assert_allclose(samples[0]["reward"], samples[1]["reward"], atol=1e-10, rtol=0)
            for key in samples[0]["losses"]:
                np.testing.assert_allclose(samples[0]["losses"][key], samples[1]["losses"][key], atol=1e-10, rtol=0)
            results.append({"offset_s": offset, "teacher": base._imitation_target_summary(), "samples": samples})
        finally:
            env.close()
    cli.output.parent.mkdir(parents=True, exist_ok=True)
    cli.output.write_text(json.dumps({"pass": True, "starts": results,
                                     "schema_version": args._unified_contract["schema_version"],
                                     "target_velocity_scale": scale,
                                     "training_started": False}, indent=2, allow_nan=False) + "\n")
    print(f"PASS: three starts; six jointly visible/masked prescribed inputs; scale={scale}; identical reward; {cli.output}")


if __name__ == "__main__":
    main()
