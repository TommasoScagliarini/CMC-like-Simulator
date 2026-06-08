"""Production-minimal PPO/SNN training entrypoint for the CMC-like env."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Mapping

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
class CMCPPOTrainConfig:
    setup_xml_path: str | None = None
    output_dir: str | None = None
    segment_duration: float = 0.01
    episode_duration: float = 0.5
    policy_knots: int = 3
    max_delta_rad: float = 0.35
    timesteps: int = 100
    rollouts: int = 16
    learning_epochs: int = 4
    mini_batches: int = 2
    sequence_length: int = 1
    hidden_size: int = 64
    num_layers: int = 1
    learning_rate: float = 1e-4
    log_std_init: float = -2.0
    seed: int = 123
    device: str = "cpu"
    random_init: bool = False
    record_outputs: bool = False
    fail_fast: bool = True
    resume_agent_path: str | None = None
    load_reference_path: str | None = None


def run_cmc_ppo_training(config: CMCPPOTrainConfig) -> dict[str, Any]:
    np.random.seed(config.seed)
    torch.manual_seed(config.seed)
    device = torch.device(config.device)

    output_dir = _resolve_output_dir(config.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    agent_checkpoint_path = output_dir / "agent.pt"
    reference_checkpoint_path = output_dir / "reference.pt"
    last_agent_checkpoint_path = output_dir / "last_agent.pt"
    last_reference_checkpoint_path = output_dir / "last_reference.pt"
    best_agent_checkpoint_path = output_dir / "best_agent.pt"
    best_reference_checkpoint_path = output_dir / "best_reference.pt"
    summary_path = output_dir / "summary.json"

    env_cfg = CMCEnvConfig(
        setup_xml_path=config.setup_xml_path,
        segment_duration=config.segment_duration,
        episode_duration=config.episode_duration,
        policy_knots=config.policy_knots,
        action_mode="delta",
        max_delta_rad=config.max_delta_rad,
        random_init=config.random_init,
        rebuild_model_on_reset=False,
        record_outputs=config.record_outputs,
        save_outputs_on_close=config.record_outputs,
        output_dir=str(output_dir / "sim_outputs"),
        output_prefix="train_episode",
        fail_fast=config.fail_fast,
    )
    env = CMCLikeProsthesisTrajectoryEnv(env_cfg)

    try:
        obs, info = env.reset(seed=config.seed)
        feature_names = tuple(env.observation_feature_names)
        obs_dim = int(np.prod(env.observation_space.shape))
        action_shape = tuple(int(item) for item in env.action_space.shape)
        action_size = int(np.prod(action_shape))
        pros_coords = tuple(env.cfg.pros_coords)

        snn_cfg = SNNConfig.for_env_action(
            input_size=obs_dim,
            hidden_size=config.hidden_size,
            num_layers=config.num_layers,
            policy_knots=action_shape[0],
            pros_coords=pros_coords,
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
            _validate_checkpoint_contract(generator, feature_names, action_shape)
            model.load_reference_state_dict(generator.model.state_dict())

        memory = RandomMemory(
            memory_size=config.rollouts,
            num_envs=1,
            device=device,
        )
        agent = PPO_SNN(
            models={"policy": model, "value": model},
            memory=memory,
            observation_space=env.observation_space,
            action_space=env.action_space,
            device=device,
            cfg={
                "rollouts": config.rollouts,
                "learning_epochs": config.learning_epochs,
                "mini_batches": config.mini_batches,
                "learning_rate": config.learning_rate,
                "learning_starts": 0,
                "timesteps": config.timesteps,
                "experiment": {
                    "directory": str(output_dir / "skrl"),
                    "experiment_name": "cmc_ppo_train",
                    "write_interval": 0,
                    "checkpoint_interval": 0,
                    "store_separately": False,
                    "wandb": False,
                    "wandb_kwargs": {},
                },
            },
            num_envs=1,
        )
        agent.init(trainer_cfg={"timesteps": config.timesteps})
        if config.resume_agent_path:
            agent.load(config.resume_agent_path)

        rewards: list[float] = []
        times: list[float] = []
        action_abs_max: list[float] = []
        episode_returns: list[float] = []
        episode_lengths: list[int] = []
        episode_end_reasons: list[str] = []
        episode_return = 0.0
        episode_length = 0
        episode_index = 0
        completed_timesteps = 0
        interrupted = False
        best_score = -float("inf")
        best_timestep: int | None = None
        best_episode_index: int | None = None
        best_metric_kind = "episode_return"
        terminated_count = 0
        truncated_count = 0
        last_info: dict[str, Any] = dict(info)
        last_step_info: dict[str, Any] | None = None
        obs_tensor = _as_batch(obs, device)

        try:
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
                if not np.all(np.isfinite(env_action)):
                    raise AssertionError("non-finite action during training")

                next_obs, reward, terminated, truncated, step_info = env.step(env_action)
                if not np.all(np.isfinite(next_obs)) or not np.isfinite(reward):
                    raise AssertionError(
                        "non-finite observation or reward during training"
                    )
                _validate_feature_names(env, step_info)

                next_obs_tensor = _as_batch(next_obs, device)
                agent.record_transition(
                    observations=obs_tensor,
                    states=None,
                    actions=actions,
                    rewards=torch.as_tensor(
                        [[reward]],
                        dtype=torch.float32,
                        device=device,
                    ),
                    next_observations=next_obs_tensor,
                    next_states=None,
                    terminated=torch.as_tensor(
                        [[terminated]], dtype=torch.bool, device=device
                    ),
                    truncated=torch.as_tensor(
                        [[truncated]], dtype=torch.bool, device=device
                    ),
                    infos=step_info,
                    timestep=timestep,
                    timesteps=config.timesteps,
                )
                agent.post_interaction(timestep=timestep, timesteps=config.timesteps)

                reward_value = float(reward)
                rewards.append(reward_value)
                times.append(float(step_info["time"]))
                action_abs_max.append(float(np.max(np.abs(env_action))))
                episode_return += reward_value
                episode_length += 1
                completed_timesteps += 1
                terminated_count += int(bool(terminated))
                truncated_count += int(bool(truncated))
                last_info = dict(step_info)
                last_step_info = dict(step_info)

                if terminated or truncated:
                    reason = str(
                        step_info.get("end_reason")
                        or ("terminated" if terminated else "truncated")
                    )
                    episode_returns.append(float(episode_return))
                    episode_lengths.append(int(episode_length))
                    episode_end_reasons.append(reason)
                    if episode_return > best_score:
                        best_score = float(episode_return)
                        best_timestep = int(timestep)
                        best_episode_index = int(episode_index)
                        _save_training_checkpoint_pair(
                            agent=agent,
                            model=model,
                            agent_path=best_agent_checkpoint_path,
                            reference_path=best_reference_checkpoint_path,
                            metadata=_checkpoint_metadata(
                                role="best",
                                metric_name=best_metric_kind,
                                metric_value=best_score,
                                timestep=timestep,
                                episode_index=episode_index,
                                action_shape=action_shape,
                                pros_coords=pros_coords,
                                feature_names=feature_names,
                                setup_xml_path=config.setup_xml_path,
                                env_cfg=env_cfg,
                                train_cfg=config,
                            ),
                        )

                    episode_return = 0.0
                    episode_length = 0
                    episode_index += 1
                    obs, info = env.reset(seed=config.seed + timestep + 1)
                    obs_tensor = _as_batch(obs, device)
                    last_info = dict(info)
                else:
                    obs_tensor = next_obs_tensor
        except KeyboardInterrupt:
            interrupted = True

        if episode_length > 0 and not episode_returns:
            best_metric_kind = "partial_episode_return"
            best_score = float(episode_return)
            best_timestep = max(0, completed_timesteps - 1)
            best_episode_index = int(episode_index)
            _save_training_checkpoint_pair(
                agent=agent,
                model=model,
                agent_path=best_agent_checkpoint_path,
                reference_path=best_reference_checkpoint_path,
                metadata=_checkpoint_metadata(
                    role="best",
                    metric_name=best_metric_kind,
                    metric_value=best_score,
                    timestep=best_timestep,
                    episode_index=episode_index,
                    action_shape=action_shape,
                    pros_coords=pros_coords,
                    feature_names=feature_names,
                    setup_xml_path=config.setup_xml_path,
                    env_cfg=env_cfg,
                    train_cfg=config,
                ),
            )
        elif best_score == -float("inf") and rewards:
            best_metric_kind = "reward_mean_fallback"
            best_score = float(np.mean(rewards))
            best_timestep = max(0, completed_timesteps - 1)
            best_episode_index = None
            _save_training_checkpoint_pair(
                agent=agent,
                model=model,
                agent_path=best_agent_checkpoint_path,
                reference_path=best_reference_checkpoint_path,
                metadata=_checkpoint_metadata(
                    role="best",
                    metric_name=best_metric_kind,
                    metric_value=best_score,
                    timestep=best_timestep,
                    episode_index=None,
                    action_shape=action_shape,
                    pros_coords=pros_coords,
                    feature_names=feature_names,
                    setup_xml_path=config.setup_xml_path,
                    env_cfg=env_cfg,
                    train_cfg=config,
                ),
            )

        last_metric_value = float(rewards[-1]) if rewards else float("nan")
        last_timestep = max(0, completed_timesteps - 1)
        last_metadata = _checkpoint_metadata(
            role="last",
            metric_name="last_reward",
            metric_value=last_metric_value,
            timestep=last_timestep,
            episode_index=episode_index,
            action_shape=action_shape,
            pros_coords=pros_coords,
            feature_names=feature_names,
            setup_xml_path=config.setup_xml_path,
            env_cfg=env_cfg,
            train_cfg=config,
        )
        _save_training_checkpoint_pair(
            agent=agent,
            model=model,
            agent_path=last_agent_checkpoint_path,
            reference_path=last_reference_checkpoint_path,
            metadata=last_metadata,
        )
        _save_training_checkpoint_pair(
            agent=agent,
            model=model,
            agent_path=agent_checkpoint_path,
            reference_path=reference_checkpoint_path,
            metadata={**last_metadata, "checkpoint_role": "last_alias"},
        )

        reloaded = ReferenceGenerator.from_checkpoint(
            reference_checkpoint_path,
            device=device,
        )
        _validate_checkpoint_contract(reloaded, feature_names, action_shape)
        reloaded_action = reloaded.predict_action(
            last_info.get("observation", {}),
            action_shape=action_shape,
        )
        if not np.all(np.isfinite(reloaded_action)):
            raise AssertionError("reloaded checkpoint produced non-finite action")

        summary: dict[str, Any] = {
            "ok": True,
            "output_dir": str(output_dir),
            "setup_xml_path": config.setup_xml_path,
            "timesteps": config.timesteps,
            "timesteps_completed": completed_timesteps,
            "interrupted": interrupted,
            "rollouts": config.rollouts,
            "reward_min": float(np.min(rewards)) if rewards else float("nan"),
            "reward_max": float(np.max(rewards)) if rewards else float("nan"),
            "reward_mean": float(np.mean(rewards)) if rewards else float("nan"),
            "reward_last": float(rewards[-1]) if rewards else float("nan"),
            "episode_returns": episode_returns,
            "episode_lengths": episode_lengths,
            "episode_end_reasons": episode_end_reasons,
            "best_metric_name": best_metric_kind,
            "best_score": float(best_score) if np.isfinite(best_score) else None,
            "best_timestep": best_timestep,
            "best_episode_index": best_episode_index,
            "action_abs_max": float(np.max(action_abs_max)) if action_abs_max else 0.0,
            "times": times,
            "terminated_count": terminated_count,
            "truncated_count": truncated_count,
            "agent_checkpoint_path": str(agent_checkpoint_path),
            "reference_checkpoint_path": str(reference_checkpoint_path),
            "last_agent_checkpoint_path": str(last_agent_checkpoint_path),
            "last_reference_checkpoint_path": str(last_reference_checkpoint_path),
            "best_agent_checkpoint_path": str(best_agent_checkpoint_path),
            "best_reference_checkpoint_path": str(best_reference_checkpoint_path),
            "summary_path": str(summary_path),
            "observation_size": obs_dim,
            "observation_feature_names": list(feature_names),
            "action_shape": list(action_shape),
            "prosthetic_coords": list(pros_coords),
            "last_reward_terms": _jsonable(
                (last_step_info or last_info).get("reward_terms", {})
            ),
        }
        summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        return summary
    finally:
        env.close()


def _resolve_output_dir(raw: str | None) -> Path:
    if raw:
        return Path(raw)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("runs") / f"cmc_ppo_train_{stamp}"


def _save_training_checkpoint_pair(
    *,
    agent: PPO_SNN,
    model: ProsthesisSNNActorCritic,
    agent_path: Path,
    reference_path: Path,
    metadata: Mapping[str, Any],
) -> None:
    agent.save(str(agent_path))
    save_reference_checkpoint(
        reference_path,
        model,
        metadata=metadata,
    )


def _checkpoint_metadata(
    *,
    role: str,
    metric_name: str,
    metric_value: float,
    timestep: int | None,
    episode_index: int | None,
    action_shape: tuple[int, int],
    pros_coords: tuple[str, ...],
    feature_names: tuple[str, ...],
    setup_xml_path: str | None,
    env_cfg: CMCEnvConfig,
    train_cfg: CMCPPOTrainConfig,
) -> dict[str, Any]:
    return {
        "entrypoint": "cmc_ppo_train",
        "checkpoint_role": role,
        "selection_metric": metric_name,
        "selection_metric_value": float(metric_value),
        "selection_timestep": timestep,
        "selection_episode_index": episode_index,
        "action_contract": "env_action",
        "action_shape": list(action_shape),
        "prosthetic_coords": list(pros_coords),
        "feature_names": list(feature_names),
        "setup_xml_path": setup_xml_path,
        "env_config": _jsonable(asdict(env_cfg)),
        "training_config": _jsonable(asdict(train_cfg)),
    }


def _as_batch(obs: np.ndarray, device: torch.device) -> torch.Tensor:
    return torch.as_tensor(obs, dtype=torch.float32, device=device).reshape(1, -1)


def _validate_feature_names(env: CMCLikeProsthesisTrajectoryEnv, info: Mapping[str, Any]) -> None:
    names = tuple(info.get("observation_feature_names", ()))
    if names != tuple(env.observation_feature_names):
        raise RuntimeError("Environment observation feature names changed.")


def _validate_checkpoint_contract(
    generator: ReferenceGenerator,
    feature_names: tuple[str, ...],
    action_shape: tuple[int, int],
) -> None:
    if generator.cfg.output_contract != "env_action":
        raise ValueError(
            "checkpoint must use output_contract='env_action' for CMC training"
        )
    if tuple(generator.cfg.feature_names) != feature_names:
        raise ValueError(
            "checkpoint feature names do not match env feature names: "
            f"{tuple(generator.cfg.feature_names)} != {feature_names}"
        )
    if generator.cfg.action_shape != action_shape:
        raise ValueError(
            f"checkpoint action shape {generator.cfg.action_shape} != env {action_shape}"
        )


def _jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    return value


def parse_args() -> CMCPPOTrainConfig:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--setup-xml")
    parser.add_argument("--output-dir")
    parser.add_argument("--segment-duration", type=float, default=0.01)
    parser.add_argument("--episode-duration", type=float, default=0.5)
    parser.add_argument("--policy-knots", type=int, default=3)
    parser.add_argument("--max-delta-rad", type=float, default=0.35)
    parser.add_argument("--timesteps", type=int, default=100)
    parser.add_argument("--rollouts", type=int, default=16)
    parser.add_argument("--learning-epochs", type=int, default=4)
    parser.add_argument("--mini-batches", type=int, default=2)
    parser.add_argument("--sequence-length", type=int, default=1)
    parser.add_argument("--hidden-size", type=int, default=64)
    parser.add_argument("--num-layers", type=int, default=1)
    parser.add_argument("--learning-rate", type=float, default=1e-4)
    parser.add_argument("--log-std-init", type=float, default=-2.0)
    parser.add_argument("--seed", type=int, default=123)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--random-init", action="store_true")
    parser.add_argument("--record-outputs", action="store_true")
    parser.add_argument("--no-fail-fast", action="store_true")
    parser.add_argument("--resume-agent-path")
    parser.add_argument("--load-reference-path")
    args = parser.parse_args()
    return CMCPPOTrainConfig(
        setup_xml_path=args.setup_xml,
        output_dir=args.output_dir,
        segment_duration=args.segment_duration,
        episode_duration=args.episode_duration,
        policy_knots=args.policy_knots,
        max_delta_rad=args.max_delta_rad,
        timesteps=args.timesteps,
        rollouts=args.rollouts,
        learning_epochs=args.learning_epochs,
        mini_batches=args.mini_batches,
        sequence_length=args.sequence_length,
        hidden_size=args.hidden_size,
        num_layers=args.num_layers,
        learning_rate=args.learning_rate,
        log_std_init=args.log_std_init,
        seed=args.seed,
        device=args.device,
        random_init=args.random_init,
        record_outputs=args.record_outputs,
        fail_fast=not args.no_fail_fast,
        resume_agent_path=args.resume_agent_path,
        load_reference_path=args.load_reference_path,
    )


def main() -> None:
    summary = run_cmc_ppo_training(parse_args())
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
