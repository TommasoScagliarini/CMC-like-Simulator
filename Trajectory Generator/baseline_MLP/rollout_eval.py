"""Deterministic rollout / evaluation from a trained baseline-MLP checkpoint.

Loads the exported inference RLModule (``rl_module_best`` / ``rl_module_last``
directory written by ``train_ppo_mlp.py``), runs a greedy rollout on the
CMC-like env, and reports quality metrics. With ``--record-outputs`` the env
writes .sto files that the simulator's existing ``visualize.py`` can replay.

Loading only the RLModule (not the full Algorithm) keeps eval lightweight and
needs no Ray cluster / env registration.

Run from the repository root, e.g.:
  python "Trajectory Generator\\baseline_MLP\\rollout_eval.py" --checkpoint runs\\...\\rl_module_best --setup-xml ...
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import win_runtime  # noqa: E402,F401  (apply OMP/OpenSim shim before torch/opensim)
import _bootstrap  # noqa: E402

_bootstrap.ensure_sim_paths()

import argparse  # noqa: E402
import json  # noqa: E402

import numpy as np  # noqa: E402
import torch  # noqa: E402
from ray.rllib.core.rl_module.rl_module import RLModule  # noqa: E402

import env_factory  # noqa: E402
import reward_function  # noqa: E402

_DEFAULT_SETUP_XML = r"models\AB06_SEASEA_Threadmill\AB06_SEASEA_stiff321_500_pi_setup.xml"


def _deterministic_action(module: RLModule, obs: np.ndarray, action_shape) -> np.ndarray:
    """Greedy (mean) action from the inference RLModule for a single obs."""
    obs_t = torch.as_tensor(np.asarray(obs), dtype=torch.float32).reshape(1, -1)
    with torch.no_grad():
        fwd = module.forward_inference({"obs": obs_t})
    logits = fwd["action_dist_inputs"]
    try:
        dist_cls = module.get_inference_action_dist_cls()
        action_t = dist_cls.from_logits(logits).to_deterministic().sample()
    except Exception:
        # Gaussian fallback: deterministic action = mean (first half of logits).
        mean = logits[..., : logits.shape[-1] // 2]
        action_t = mean
    action = action_t.detach().cpu().numpy().reshape(action_shape).astype(np.float32)
    return action


def run(args: argparse.Namespace) -> dict:
    # Absolute path: pyarrow's from_uri (used by RLlib) rejects relative paths.
    checkpoint = Path(args.checkpoint).resolve()
    module = RLModule.from_checkpoint(checkpoint)

    env_config = {
        "setup_xml_path": args.setup_xml,
        "segment_duration": args.segment_duration,
        "episode_duration": args.episode_duration,
        "policy_knots": args.policy_knots,
        "action_mode": "delta",
        "max_delta_rad": args.max_delta_rad,
        "random_init": False,
        "fail_fast": True,
        "record_outputs": args.record_outputs,
        "save_outputs_on_close": args.record_outputs,
        "grf_mode": args.grf_mode,
        "online_grf_profile_file": args.online_grf_profile,
        "include_online_grf_observation": args.online_grf_observation,
    }
    # Same reward shaping as training (reward_function.py), so the evaluated
    # return is comparable to the trained objective.
    reward_overrides = reward_function.load_reward_overrides(args.reward_json)
    if reward_overrides:
        env_config["reward"] = reward_overrides
    if args.output_dir:
        env_config["output_dir"] = str(Path(args.output_dir) / "sim_outputs")
        env_config["output_prefix"] = "rollout_episode"

    env = env_factory.make_cmc_env(env_config)
    action_shape = tuple(int(d) for d in env.action_space.shape)

    rewards: list[float] = []
    action_abs_max = 0.0
    pelvis_ty_min = float("inf")
    terminated = truncated = False
    steps = 0
    try:
        obs, info = env.reset(seed=args.seed)
        for _ in range(args.max_steps):
            action = _deterministic_action(module, obs, action_shape)
            action_abs_max = max(action_abs_max, float(np.max(np.abs(action))))
            obs, reward, terminated, truncated, info = env.step(action)
            rewards.append(float(reward))
            steps += 1
            obs_map = info.get("observation", {}) if isinstance(info, dict) else {}
            if "pelvis_ty" in obs_map:
                pelvis_ty_min = min(pelvis_ty_min, float(obs_map["pelvis_ty"]))
            if terminated or truncated:
                break
    finally:
        env.close()

    summary = {
        "ok": True,
        "checkpoint": str(args.checkpoint),
        "setup_xml_path": args.setup_xml,
        "steps": steps,
        "episode_return": float(np.sum(rewards)) if rewards else 0.0,
        "reward_mean": float(np.mean(rewards)) if rewards else None,
        "reward_min": float(np.min(rewards)) if rewards else None,
        "reward_max": float(np.max(rewards)) if rewards else None,
        "action_abs_max": action_abs_max,
        "pelvis_ty_min": None if pelvis_ty_min == float("inf") else pelvis_ty_min,
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "action_shape": list(action_shape),
        "record_outputs": bool(args.record_outputs),
        "grf_mode": args.grf_mode,
        "online_grf_profile_file": args.online_grf_profile,
        "online_grf_observation": bool(args.online_grf_observation),
        "reward_config": reward_function.RewardConfig.from_mapping(
            reward_overrides
        ).to_dict(),
    }
    if args.output_dir:
        out = Path(args.output_dir)
        out.mkdir(parents=True, exist_ok=True)
        (out / "rollout_summary.json").write_text(
            json.dumps(summary, indent=2), encoding="utf-8"
        )
        summary["summary_path"] = str(out / "rollout_summary.json")
    return summary


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--checkpoint", required=True, help="rl_module_* directory")
    p.add_argument("--setup-xml", default=_DEFAULT_SETUP_XML)
    p.add_argument("--output-dir")
    p.add_argument("--segment-duration", type=float, default=0.01)
    p.add_argument("--episode-duration", type=float, default=0.5)
    p.add_argument("--policy-knots", type=int, default=3)
    p.add_argument("--max-delta-rad", type=float, default=0.35)
    p.add_argument("--max-steps", type=int, default=10000)
    p.add_argument("--seed", type=int, default=123)
    p.add_argument("--record-outputs", action="store_true")
    p.add_argument(
        "--grf-mode",
        choices=("prescribed", "online_sensor", "online"),
        default=env_factory.DEFAULT_NETWORK_GRF_MODE,
        help="GRF mode used during inference (default: online_sensor).",
    )
    p.add_argument(
        "--online-grf-profile",
        default=env_factory.DEFAULT_NETWORK_ONLINE_GRF_PROFILE,
        help="onlineGRF profile JSON used by online_sensor/online.",
    )
    p.add_argument(
        "--online-grf-observation",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Expose onlineGRF gait features (default: on); must match training.",
    )
    p.add_argument(
        "--reward-json",
        default=None,
        help="Reward overrides (JSON file path or inline JSON object of "
        "reward_function.RewardConfig fields); should match the training run.",
    )
    return p.parse_args()


def main() -> None:
    summary = run(parse_args())
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
