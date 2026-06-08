"""Run a CMC-like trajectory env rollout from an exported SNN checkpoint."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

_THIS_FILE = Path(__file__).resolve()
_PROSTHESIS_ROOT = _THIS_FILE.parents[1]
_TRAJ_GEN_DIR = _THIS_FILE.parents[2]
_REPO_ROOT = _THIS_FILE.parents[3]
for _path in (_PROSTHESIS_ROOT, _TRAJ_GEN_DIR, _REPO_ROOT):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from osim_trj_cmc_like import CMCEnvConfig, CMCLikeProsthesisTrajectoryEnv
from prosthesis_snn.generator import ReferenceGenerator


@dataclass(frozen=True)
class CMCPolicyRolloutConfig:
    checkpoint_path: str
    setup_xml_path: str | None = None
    output_dir: str | None = None
    segment_duration: float = 0.01
    episode_duration: float = 0.5
    max_delta_rad: float = 0.35
    timesteps: int = 100
    seed: int = 123
    device: str = "cpu"
    record_outputs: bool = False
    fail_fast: bool = True
    reset_on_done: bool = False


def run_policy_rollout(config: CMCPolicyRolloutConfig) -> dict[str, Any]:
    output_dir = _resolve_output_dir(config.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    summary_path = output_dir / "rollout_summary.json"

    generator = ReferenceGenerator.from_checkpoint(
        config.checkpoint_path,
        device=config.device,
    )
    if generator.cfg.output_contract != "env_action":
        raise ValueError(
            "checkpoint must use output_contract='env_action' for env rollout"
        )
    action_shape = generator.cfg.action_shape

    env_cfg = CMCEnvConfig(
        setup_xml_path=config.setup_xml_path,
        segment_duration=config.segment_duration,
        episode_duration=config.episode_duration,
        policy_knots=action_shape[0],
        action_mode="delta",
        max_delta_rad=config.max_delta_rad,
        random_init=False,
        rebuild_model_on_reset=False,
        record_outputs=config.record_outputs,
        save_outputs_on_close=config.record_outputs,
        output_dir=str(output_dir / "sim_outputs"),
        output_prefix="policy_rollout",
        fail_fast=config.fail_fast,
    )
    env = CMCLikeProsthesisTrajectoryEnv(env_cfg)

    try:
        obs, info = env.reset(seed=config.seed)
        _validate_contract(generator, env, action_shape)

        rewards: list[float] = []
        times: list[float] = []
        action_abs_max: list[float] = []
        terminated_count = 0
        truncated_count = 0
        last_info: dict[str, Any] = dict(info)
        last_step_info: dict[str, Any] | None = None

        for timestep in range(config.timesteps):
            features = last_info.get("observation", {})
            action = generator.predict_action(features, action_shape=action_shape)
            if not np.all(np.isfinite(action)):
                raise AssertionError("checkpoint produced non-finite action")

            obs, reward, terminated, truncated, step_info = env.step(action)
            if not np.all(np.isfinite(obs)) or not np.isfinite(reward):
                raise AssertionError("non-finite observation or reward during rollout")
            _validate_contract(generator, env, action_shape)

            rewards.append(float(reward))
            times.append(float(step_info["time"]))
            action_abs_max.append(float(np.max(np.abs(action))))
            terminated_count += int(bool(terminated))
            truncated_count += int(bool(truncated))
            last_info = dict(step_info)
            last_step_info = dict(step_info)

            if terminated or truncated:
                if not config.reset_on_done:
                    break
                obs, info = env.reset(seed=config.seed + timestep + 1)
                last_info = dict(info)

        summary: dict[str, Any] = {
            "ok": True,
            "checkpoint_path": config.checkpoint_path,
            "output_dir": str(output_dir),
            "setup_xml_path": config.setup_xml_path,
            "timesteps_requested": config.timesteps,
            "timesteps_completed": len(rewards),
            "reward_min": float(np.min(rewards)) if rewards else float("nan"),
            "reward_max": float(np.max(rewards)) if rewards else float("nan"),
            "reward_mean": float(np.mean(rewards)) if rewards else float("nan"),
            "reward_last": float(rewards[-1]) if rewards else float("nan"),
            "action_abs_max": float(np.max(action_abs_max)) if action_abs_max else 0.0,
            "times": times,
            "terminated_count": terminated_count,
            "truncated_count": truncated_count,
            "summary_path": str(summary_path),
            "observation_feature_names": list(env.observation_feature_names),
            "checkpoint_feature_names": list(generator.cfg.feature_names),
            "action_shape": list(action_shape),
            "last_reward_terms": _jsonable(
                (last_step_info or last_info).get("reward_terms", {})
            ),
        }
        summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        return summary
    finally:
        env.close()


def _validate_contract(
    generator: ReferenceGenerator,
    env: CMCLikeProsthesisTrajectoryEnv,
    action_shape: tuple[int, int],
) -> None:
    if tuple(env.action_space.shape) != action_shape:
        raise ValueError(
            f"env action shape {env.action_space.shape} != checkpoint {action_shape}"
        )
    feature_names = tuple(env.observation_feature_names)
    if tuple(generator.cfg.feature_names) != feature_names:
        raise ValueError(
            "checkpoint feature names do not match env feature names: "
            f"{tuple(generator.cfg.feature_names)} != {feature_names}"
        )


def _resolve_output_dir(raw: str | None) -> Path:
    if raw:
        return Path(raw)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("runs") / f"cmc_policy_rollout_{stamp}"


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


def parse_args() -> CMCPolicyRolloutConfig:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--setup-xml")
    parser.add_argument("--output-dir")
    parser.add_argument("--segment-duration", type=float, default=0.01)
    parser.add_argument("--episode-duration", type=float, default=0.5)
    parser.add_argument("--max-delta-rad", type=float, default=0.35)
    parser.add_argument("--timesteps", type=int, default=100)
    parser.add_argument("--seed", type=int, default=123)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--record-outputs", action="store_true")
    parser.add_argument("--no-fail-fast", action="store_true")
    parser.add_argument("--reset-on-done", action="store_true")
    args = parser.parse_args()
    return CMCPolicyRolloutConfig(
        checkpoint_path=args.checkpoint,
        setup_xml_path=args.setup_xml,
        output_dir=args.output_dir,
        segment_duration=args.segment_duration,
        episode_duration=args.episode_duration,
        max_delta_rad=args.max_delta_rad,
        timesteps=args.timesteps,
        seed=args.seed,
        device=args.device,
        record_outputs=args.record_outputs,
        fail_fast=not args.no_fail_fast,
        reset_on_done=args.reset_on_done,
    )


def main() -> None:
    summary = run_policy_rollout(parse_args())
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
