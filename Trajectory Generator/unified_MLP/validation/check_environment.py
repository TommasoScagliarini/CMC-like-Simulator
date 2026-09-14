"""Live, short environment parity audit; no PPO or checkpoint updates."""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from unified_MLP import _bootstrap
_bootstrap.ensure_sim_paths()
from baseline_MLP import win_runtime  # noqa: F401
import numpy as np
from unified_MLP import env_factory, train_ppo_mlp as trainer, training_config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="environment_contract.json")
    audit_args = parser.parse_args()
    destination = (_bootstrap.UNIFIED_DIR / "runs" / "validation" / audit_args.output).resolve()
    destination.relative_to((_bootstrap.UNIFIED_DIR / "runs" / "validation").resolve())
    if destination.exists():
        raise FileExistsError(destination)
    sys.argv = ["train", "--config", str(_bootstrap.UNIFIED_DIR / "configs" / "smoke_pretraining.yaml")]
    args = trainer.parse_args()
    trainer._validate_start_sampling_args(args)
    trainer._load_training_stack()
    ppo = trainer.build_config(args, args._cfg_reward)
    raw_imitation = copy.deepcopy(ppo.env_config)
    raw_exnovo = copy.deepcopy(raw_imitation)
    raw_exnovo["reward"] = training_config.load(
        _bootstrap.UNIFIED_DIR / "configs" / "ex_novo.yaml"
    )["reward"]
    rows = []
    for offset in args.episode_start_offset_choices_s:
        environments = []
        try:
            for raw in (raw_imitation, raw_exnovo):
                cfg = {**raw, "episode_start_offset_s": offset, "episode_start_offset_choices_s": []}
                environments.append(env_factory.make_cmc_env(cfg))
            observations = [env.reset(seed=123)[0] for env in environments]
            first, second = [env.unwrapped for env in environments]
            assert first.n_actor == second.n_actor == 35
            assert first.actor_feature_names == second.actor_feature_names
            assert first.observation_feature_names == second.observation_feature_names
            assert np.array_equal(observations[0], observations[1])
            assert np.array_equal(observations[0][:2], np.zeros(2))
            q, qdot, _ = first.base_kin.get(first.t)
            target, target_velocity, _ = first.imitation_target(first.t)
            for name in ("pros_knee_angle", "pros_ankle_angle"):
                assert target[name] == q[name] and target_velocity[name] == qdot[name]
            action = np.asarray([q[name] for name in ("pros_knee_angle", "pros_ankle_angle")], dtype=np.float32)
            action = np.clip(action, environments[0].action_space.low, environments[0].action_space.high)
            steps = [env.step(action) for env in environments]
            assert np.array_equal(steps[0][0], steps[1][0])
            assert steps[0][2:4] == steps[1][2:4]
            assert all(np.isfinite(step[1]) for step in steps)
            rows.append({
                "offset_s": offset, "n_actor": first.n_actor, "n_obs": first.n_obs,
                "reset_observations_exact": True, "step_observations_exact": True,
                "clock_slots_zero": True, "prescribed_left_target_exact": True,
                "imitation_reward": float(steps[0][1]), "ex_novo_reward": float(steps[1][1]),
            })
        finally:
            for env in environments:
                env.close()
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps({"ok": True, "starts": rows}, indent=2))
    print(json.dumps({"ok": True, "output": str(destination), "starts": len(rows)}))


if __name__ == "__main__":
    main()
