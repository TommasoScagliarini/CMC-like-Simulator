"""Minimal PPO/SNN training smoke for a CMC-like trajectory environment.

This entrypoint is deliberately model-agnostic. It receives a simulator setup
XML through ``--setup-xml`` or falls back to the simulator's persisted last
setup. The current trajectory environment action is a short sequence of
prosthetic q-reference knots with shape ``(policy_knots, n_prosthetic_coords)``.
The SNN output layout mirrors that flattened action contract for this smoke.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np
import torch

_THIS_FILE = Path(__file__).resolve()
_PROSTHESIS_ROOT = _THIS_FILE.parents[2]
_TRAJ_GEN_DIR = _THIS_FILE.parents[3]
_REPO_ROOT = _THIS_FILE.parents[4]
for _path in (_PROSTHESIS_ROOT, _TRAJ_GEN_DIR, _REPO_ROOT):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from osim_trj_cmc_like import CMCEnvConfig, CMCLikeProsthesisTrajectoryEnv
from prosthesis_snn.config import SNNConfig
from prosthesis_snn.generator import ReferenceGenerator
from prosthesis_snn.training.actor_critic import ProsthesisSNNActorCritic
from prosthesis_snn.training.checkpoint import save_reference_checkpoint
from prosthesis_snn.training.ppo_snn import PPO_SNN
from skrl.memories.torch import RandomMemory


@dataclass(frozen=True)
class CMCPPOSmokeConfig:
    setup_xml_path: str | None = None
    output_dir: str = "runs/prosthesis_snn_cmc_ppo_smoke"
    segment_duration: float = 0.002
    episode_duration: float = 0.004
    policy_knots: int = 3
    max_delta_rad: float = 0.05
    timesteps: int = 2
    rollouts: int = 2
    learning_epochs: int = 1
    mini_batches: int = 1
    sequence_length: int = 1
    hidden_size: int = 16
    num_layers: int = 1
    learning_rate: float = 1e-4
    log_std_init: float = -3.0
    seed: int = 123
    device: str = "cpu"
    resume_agent_path: str | None = None
    load_reference_path: str | None = None


def run_cmc_ppo_smoke(config: CMCPPOSmokeConfig) -> dict[str, Any]:
    """Run a tiny PPO update and export an inference checkpoint."""

    np.random.seed(config.seed)
    torch.manual_seed(config.seed)
    device = torch.device(config.device)

    output_dir = Path(config.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    agent_checkpoint_path = output_dir / "agent.pt"
    reference_checkpoint_path = output_dir / "reference.pt"
    summary_path = output_dir / "summary.json"

    env = CMCLikeProsthesisTrajectoryEnv(
        CMCEnvConfig(
            setup_xml_path=config.setup_xml_path,
            segment_duration=config.segment_duration,
            episode_duration=config.episode_duration,
            policy_knots=config.policy_knots,
            action_mode="delta",
            max_delta_rad=config.max_delta_rad,
            random_init=False,
            rebuild_model_on_reset=False,
            record_outputs=False,
            fail_fast=True,
        )
    )

    try:
        obs, info = env.reset(seed=config.seed)
        obs_dim = int(np.prod(env.observation_space.shape))
        action_shape = tuple(int(item) for item in env.action_space.shape)
        action_size = int(np.prod(action_shape))
        pros_coords = tuple(env.cfg.pros_coords)
        if len(action_shape) != 2 or action_shape[1] != len(pros_coords):
            raise ValueError(
                "Expected trajectory action shape "
                "(policy_knots, n_prosthetic_coords), got "
                f"{action_shape} for {len(pros_coords)} prosthetic coords"
            )

        feature_names = tuple(f"obs_{index:03d}" for index in range(obs_dim))
        snn_cfg = SNNConfig(
            input_size=obs_dim,
            hidden_size=config.hidden_size,
            num_layers=config.num_layers,
            # This mirrors numpy C-order flattening of env.action_space:
            # [knot_1_coord_1, knot_1_coord_2, knot_2_coord_1, ...].
            output_coords=tuple(f"knot_{index + 1}" for index in range(action_shape[0])),
            output_channels=pros_coords,
            feature_names=feature_names,
        )
        if snn_cfg.output_size != action_size:
            raise ValueError(
                f"SNN output size {snn_cfg.output_size} != action size {action_size}"
            )

        model = ProsthesisSNNActorCritic(
            observation_space=env.observation_space,
            action_space=env.action_space,
            device=device,
            cfg=snn_cfg,
            num_envs=1,
            sequence_length=config.sequence_length,
            log_std_init=config.log_std_init,
        )

        if config.load_reference_path:
            generator = ReferenceGenerator.from_checkpoint(
                config.load_reference_path,
                device=device,
            )
            model.load_reference_state_dict(generator.model.state_dict())

        memory = RandomMemory(
            memory_size=config.rollouts,
            num_envs=1,
            device=device,
        )
        agent_cfg = {
            "rollouts": config.rollouts,
            "learning_epochs": config.learning_epochs,
            "mini_batches": config.mini_batches,
            "learning_rate": config.learning_rate,
            "learning_starts": 0,
            "experiment": {
                "directory": str(output_dir / "skrl"),
                "experiment_name": "cmc_ppo_smoke",
                "write_interval": 0,
                "checkpoint_interval": 0,
                "store_separately": False,
                "wandb": False,
                "wandb_kwargs": {},
            },
        }
        agent = PPO_SNN(
            models={"policy": model, "value": model},
            memory=memory,
            observation_space=env.observation_space,
            action_space=env.action_space,
            device=device,
            cfg=agent_cfg,
            num_envs=1,
        )
        agent.init(trainer_cfg={"timesteps": config.timesteps})
        if config.resume_agent_path:
            agent.load(config.resume_agent_path)

        rewards: list[float] = []
        times: list[float] = []
        truncated_count = 0
        terminated_count = 0
        obs_tensor = _as_batch(obs, device)
        last_info: dict[str, Any] = dict(info)

        for timestep in range(config.timesteps):
            agent.pre_interaction(timestep=timestep, timesteps=config.timesteps)
            actions, _outputs = agent.act(
                obs_tensor,
                timestep=timestep,
                timesteps=config.timesteps,
            )
            env_action = (
                actions.detach()
                .cpu()
                .numpy()
                .reshape(action_shape)
                .astype(np.float32)
            )
            next_obs, reward, terminated, truncated, step_info = env.step(env_action)
            if not np.all(np.isfinite(next_obs)) or not np.isfinite(reward):
                raise AssertionError("non-finite observation or reward during smoke")

            next_obs_tensor = _as_batch(next_obs, device)
            agent.record_transition(
                observations=obs_tensor,
                states=None,
                actions=actions,
                rewards=torch.as_tensor([[reward]], dtype=torch.float32, device=device),
                next_observations=next_obs_tensor,
                next_states=None,
                terminated=torch.as_tensor(
                    [[terminated]], dtype=torch.bool, device=device
                ),
                truncated=torch.as_tensor([[truncated]], dtype=torch.bool, device=device),
                infos=step_info,
                timestep=timestep,
                timesteps=config.timesteps,
            )
            agent.post_interaction(timestep=timestep, timesteps=config.timesteps)

            rewards.append(float(reward))
            times.append(float(step_info["time"]))
            truncated_count += int(bool(truncated))
            terminated_count += int(bool(terminated))
            last_info = dict(step_info)

            if truncated:
                obs, _ = env.reset(seed=config.seed + timestep + 1)
                obs_tensor = _as_batch(obs, device)
            elif terminated:
                if timestep < config.timesteps - 1:
                    obs, _ = env.reset(seed=config.seed + timestep + 1)
                    obs_tensor = _as_batch(obs, device)
            else:
                obs_tensor = next_obs_tensor

        agent.save(str(agent_checkpoint_path))
        save_reference_checkpoint(
            reference_checkpoint_path,
            model,
            metadata={
                "entrypoint": "cmc_ppo_smoke",
                "action_contract": "trajectory_knots",
                "action_shape": list(action_shape),
                "prosthetic_coords": list(pros_coords),
                "config": asdict(config),
            },
        )

        reloaded = ReferenceGenerator.from_checkpoint(
            reference_checkpoint_path,
            device=device,
        )
        predicted = reloaded.predict(obs_tensor.detach().cpu().numpy().reshape(-1))
        flat_pred = np.array(
            [
                value
                for channel_values in predicted.values()
                for value in channel_values.values()
            ],
            dtype=float,
        )
        if flat_pred.shape != (action_size,) or not np.all(np.isfinite(flat_pred)):
            raise AssertionError("reloaded reference checkpoint produced bad outputs")

        summary: dict[str, Any] = {
            "ok": True,
            "setup_xml_path": config.setup_xml_path,
            "timesteps": config.timesteps,
            "rollouts": config.rollouts,
            "reward_min": float(np.min(rewards)),
            "reward_max": float(np.max(rewards)),
            "reward_last": float(rewards[-1]),
            "times": times,
            "terminated_count": terminated_count,
            "truncated_count": truncated_count,
            "agent_checkpoint_path": str(agent_checkpoint_path),
            "reference_checkpoint_path": str(reference_checkpoint_path),
            "summary_path": str(summary_path),
            "observation_size": obs_dim,
            "action_shape": list(action_shape),
            "prosthetic_coords": list(pros_coords),
            "output_coords": list(snn_cfg.output_coords),
            "output_channels": list(snn_cfg.output_channels),
            "reload_output_finite": bool(np.all(np.isfinite(flat_pred))),
            "last_reward_terms": _jsonable(last_info.get("reward_terms", {})),
        }
        summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        return summary
    finally:
        env.close()


def _as_batch(obs: np.ndarray, device: torch.device) -> torch.Tensor:
    return torch.as_tensor(obs, dtype=torch.float32, device=device).reshape(1, -1)


def _jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    return value


def parse_args() -> CMCPPOSmokeConfig:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--setup-xml")
    parser.add_argument("--output-dir", default=CMCPPOSmokeConfig.output_dir)
    parser.add_argument("--segment-duration", type=float, default=0.002)
    parser.add_argument("--episode-duration", type=float, default=0.004)
    parser.add_argument("--policy-knots", type=int, default=3)
    parser.add_argument("--max-delta-rad", type=float, default=0.05)
    parser.add_argument("--timesteps", type=int, default=2)
    parser.add_argument("--rollouts", type=int, default=2)
    parser.add_argument("--hidden-size", type=int, default=16)
    parser.add_argument("--learning-rate", type=float, default=1e-4)
    parser.add_argument("--seed", type=int, default=123)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--resume-agent-path")
    parser.add_argument("--load-reference-path")
    args = parser.parse_args()
    return CMCPPOSmokeConfig(
        setup_xml_path=args.setup_xml,
        output_dir=args.output_dir,
        segment_duration=args.segment_duration,
        episode_duration=args.episode_duration,
        policy_knots=args.policy_knots,
        max_delta_rad=args.max_delta_rad,
        timesteps=args.timesteps,
        rollouts=args.rollouts,
        hidden_size=args.hidden_size,
        learning_rate=args.learning_rate,
        seed=args.seed,
        device=args.device,
        resume_agent_path=args.resume_agent_path,
        load_reference_path=args.load_reference_path,
    )


def main() -> None:
    summary = run_cmc_ppo_smoke(parse_args())
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
