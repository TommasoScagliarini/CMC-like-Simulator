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

import argparse  # noqa: E402
import json  # noqa: E402
from datetime import datetime  # noqa: E402

import process_watchdog  # noqa: E402

_DEFAULT_SETUP_XML = r"models\AB06_SEASEA_Threadmill\AB06_SEASEA_stiff321_500_pi_setup.xml"
_DEFAULT_GRF_MODE = "online_sensor"
_DEFAULT_ONLINE_GRF_PROFILE = (
    "online_grf_profiles/"
    "AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json"
)
_SCRIPT_PATH = Path(__file__).resolve()
_WATCHDOG_FILENAME = "watchdog_state.json"
_INFERENCE_STACK_LOADED = False


def _load_inference_stack() -> None:
    """Import Torch/RLlib/OpenSim only inside the supervised worker process."""
    global _INFERENCE_STACK_LOADED
    global RLModule, env_factory, np, reward_function, torch

    if _INFERENCE_STACK_LOADED:
        return
    import win_runtime  # noqa: F401
    import _bootstrap

    _bootstrap.ensure_sim_paths()

    import numpy as _np
    import torch as _torch
    from ray.rllib.core.rl_module.rl_module import RLModule as _RLModule

    import env_factory as _env_factory
    import reward_function as _reward_function

    np = _np
    torch = _torch
    RLModule = _RLModule
    env_factory = _env_factory
    reward_function = _reward_function
    _INFERENCE_STACK_LOADED = True


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
    output_dir = Path(args.output_dir).resolve()
    heartbeat_path = output_dir / _WATCHDOG_FILENAME
    process_watchdog.write_heartbeat(
        heartbeat_path,
        "inference stack imports",
        timeout_s=args.startup_timeout_s,
    )
    _load_inference_stack()
    process_watchdog.write_heartbeat(
        heartbeat_path,
        "load inference checkpoint",
        timeout_s=args.startup_timeout_s,
    )
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
        "prescribed_grf_disabled_sides": args.disable_prescribed_grf_side,
        "online_grf_applied_sides": args.online_grf_applied_side,
    }
    # Same reward shaping as training (reward_function.py), so the evaluated
    # return is comparable to the trained objective.
    reward_overrides = reward_function.load_reward_overrides(args.reward_json)
    if reward_overrides:
        env_config["reward"] = reward_overrides
    if args.output_dir:
        env_config["output_dir"] = str(output_dir / "sim_outputs")
        env_config["output_prefix"] = "rollout_episode"

    process_watchdog.write_heartbeat(
        heartbeat_path,
        "build inference environment",
        timeout_s=args.startup_timeout_s,
    )
    env = env_factory.make_cmc_env(env_config)
    action_shape = tuple(int(d) for d in env.action_space.shape)

    rewards: list[float] = []
    action_abs_max = 0.0
    pelvis_ty_min = float("inf")
    terminated = truncated = False
    steps = 0
    try:
        process_watchdog.write_heartbeat(
            heartbeat_path,
            "reset inference environment",
            timeout_s=args.step_timeout_s,
        )
        obs, info = env.reset(seed=args.seed)
        for _ in range(args.max_steps):
            action = _deterministic_action(module, obs, action_shape)
            action_abs_max = max(action_abs_max, float(np.max(np.abs(action))))
            process_watchdog.write_heartbeat(
                heartbeat_path,
                f"inference simulator step {steps + 1}",
                progress=steps,
                timeout_s=args.step_timeout_s,
            )
            obs, reward, terminated, truncated, info = env.step(action)
            rewards.append(float(reward))
            steps += 1
            process_watchdog.write_heartbeat(
                heartbeat_path,
                f"completed inference simulator step {steps}",
                progress=steps,
                timeout_s=args.step_timeout_s,
            )
            obs_map = info.get("observation", {}) if isinstance(info, dict) else {}
            if "pelvis_ty" in obs_map:
                pelvis_ty_min = min(pelvis_ty_min, float(obs_map["pelvis_ty"]))
            if terminated or truncated:
                break
    finally:
        process_watchdog.write_heartbeat(
            heartbeat_path,
            "close inference environment",
            progress=steps,
            timeout_s=args.step_timeout_s,
        )
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
        "prescribed_grf_disabled_sides": list(args.disable_prescribed_grf_side),
        "online_grf_applied_sides": list(args.online_grf_applied_side),
        "reward_config": reward_function.RewardConfig.from_mapping(
            reward_overrides
        ).to_dict(),
    }
    if args.output_dir:
        out = output_dir
        out.mkdir(parents=True, exist_ok=True)
        (out / "rollout_summary.json").write_text(
            json.dumps(summary, indent=2), encoding="utf-8"
        )
        summary["summary_path"] = str(out / "rollout_summary.json")
    process_watchdog.write_heartbeat(heartbeat_path, "complete", progress=steps)
    return summary


def run_supervised(args: argparse.Namespace) -> int:
    output_dir = Path(
        args.output_dir
        or (Path("runs") / f"rollout_eval_{datetime.now():%Y%m%d_%H%M%S}")
    ).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    child_args = list(sys.argv[1:])
    if args.output_dir is None:
        child_args.extend(["--output-dir", str(output_dir)])
    child_args.append("--worker-process")
    result = process_watchdog.supervise_process(
        [sys.executable, str(_SCRIPT_PATH), *child_args],
        heartbeat_file=output_dir / _WATCHDOG_FILENAME,
        summary_file=output_dir / "watchdog_summary.json",
        startup_timeout_s=args.startup_timeout_s,
        stall_timeout_s=args.stall_timeout_s,
        run_timeout_s=args.run_timeout_s,
        label="inference rollout",
    )
    if result["timeout_reason"] is not None:
        return 124 if result["timeout_reason"] != "user_interrupt" else 130
    return int(result["returncode"])


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
        default=_DEFAULT_GRF_MODE,
        help="GRF mode used during inference (default: online_sensor).",
    )
    p.add_argument(
        "--online-grf-profile",
        default=_DEFAULT_ONLINE_GRF_PROFILE,
        help="onlineGRF profile JSON used by online_sensor/online.",
    )
    p.add_argument(
        "--online-grf-observation",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Expose onlineGRF gait features (default: on); must match training.",
    )
    p.add_argument(
        "--disable-prescribed-grf-side",
        action="append",
        choices=("left", "right"),
        default=[],
        help="Diagnostic opt-in: remove the selected prescribed ExternalForce "
        "from dynamics while retaining its data as an oracle.",
    )
    p.add_argument(
        "--online-grf-applied-side",
        action="append",
        choices=("left", "right"),
        default=[],
        help="Hybrid GRF: APPLY the online contact on the given side (prescribed "
        "auto-disabled there). Must match training. Use 'left' (prosthetic).",
    )
    p.add_argument(
        "--reward-json",
        default=None,
        help="Reward overrides (JSON file path or inline JSON object of "
        "reward_function.RewardConfig fields); should match the training run.",
    )
    p.add_argument(
        "--startup-timeout-s",
        type=float,
        default=300.0,
        help="Timeout waiting for checkpoint/env startup and first heartbeat.",
    )
    p.add_argument(
        "--step-timeout-s",
        type=float,
        default=120.0,
        help="Hard timeout for each reset, simulator step, and close phase.",
    )
    p.add_argument(
        "--stall-timeout-s",
        type=float,
        default=180.0,
        help="Terminate inference if the heartbeat stops advancing.",
    )
    p.add_argument(
        "--run-timeout-s",
        type=float,
        default=1800.0,
        help="Total inference wall-clock timeout; <=0 disables.",
    )
    p.add_argument("--worker-process", action="store_true", help=argparse.SUPPRESS)
    return p.parse_args()


def main() -> None:
    args = parse_args()
    if not args.worker_process:
        raise SystemExit(run_supervised(args))
    summary = run(args)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
