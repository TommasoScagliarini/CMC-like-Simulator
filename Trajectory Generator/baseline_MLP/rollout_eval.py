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
import math  # noqa: E402
from datetime import datetime  # noqa: E402

import process_watchdog  # noqa: E402
import progress_display  # noqa: E402
import training_config  # noqa: E402

_DEFAULT_SETUP_XML = r"models\AB06_SEASEA_Threadmill\AB06_SEASEA_stiff321_500_pi_setup.xml"
_DEFAULT_GRF_MODE = "online_sensor"
_DEFAULT_ONLINE_GRF_PROFILE = (
    "online_grf_profiles/"
    "AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json"
)
_SCRIPT_PATH = Path(__file__).resolve()
_TRAJ_GEN_DIR = _SCRIPT_PATH.parents[1]   # .../Trajectory Generator
_RUNS_ROOT = _TRAJ_GEN_DIR / "runs"       # run artifacts live here (moved 2026-06-09)
_WATCHDOG_FILENAME = "watchdog_state.json"


def _resolve_output_dir(output_dir, default_stem):
    """Resolve ``--output-dir`` so artifacts always land under
    ``Trajectory Generator/runs`` regardless of the current working directory.

    A relative path is taken relative to the Trajectory Generator directory (so
    the historical ``runs\\foo`` arguments keep pointing at the moved folder); an
    absolute path is honored as-is. CWD-independent so the supervisor and the
    worker child always agree on the same directory.
    """
    if output_dir:
        path = Path(output_dir)
        if not path.is_absolute():
            path = _TRAJ_GEN_DIR / path
    else:
        path = _RUNS_ROOT / f"{default_stem}_{datetime.now():%Y%m%d_%H%M%S}"
    return path.resolve()


def _resolve_input_path(value: str) -> Path:
    """Resolve an input path (e.g. ``--checkpoint``) accepting both conventions.

    Relative paths are tried against the CWD first (``Trajectory
    Generator\\runs\\...`` from the simulator root) and then against the
    Trajectory Generator directory, so the historical ``runs\\foo`` form keeps
    working after the 2026-06-09 move of ``runs/`` — mirroring how
    ``--output-dir`` is resolved. Absolute paths are honored as-is.
    """
    path = Path(value).expanduser()
    if path.is_absolute():
        return path.resolve()
    cwd_candidate = (Path.cwd() / path).resolve()
    if cwd_candidate.exists():
        return cwd_candidate
    traj_candidate = (_TRAJ_GEN_DIR / path).resolve()
    if traj_candidate.exists():
        return traj_candidate
    raise FileNotFoundError(
        f"Path {value!r} not found; tried {cwd_candidate} and {traj_candidate}."
    )
_INFERENCE_STACK_LOADED = False


def progress_display_fmt(value: float) -> str:
    """Compact float for the live rollout suffix."""
    try:
        fv = float(value)
    except (TypeError, ValueError):
        return "n/a"
    return f"{fv:.4g}" if math.isfinite(fv) else "n/a"


def _jsonable(value):
    """Convert nested NumPy/scalar diagnostic payloads to JSON-compatible data."""
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if hasattr(value, "tolist"):
        return _jsonable(value.tolist())
    if hasattr(value, "item"):
        return value.item()
    return value


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
    # Clean inference log by default (mirrors training): silence Python warnings
    # (e.g. the Static Optimization QP bounded-least-squares fallback
    # RuntimeWarning) AND all logging below ERROR (the RLModule deprecation is
    # emitted via the ray logger, not the warnings module). logging.disable is a
    # GLOBAL threshold that survives ray's import-time logging reconfiguration
    # (a per-logger setLevel would be reset by the rllib import). Result: the
    # console shows just the single live progress line. Use --verbose to restore.
    if not getattr(args, "verbose", False):
        import logging
        import warnings

        warnings.filterwarnings("ignore")
        logging.disable(logging.WARNING)
    output_dir = _resolve_output_dir(args.output_dir, "rollout_eval")
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
    checkpoint = _resolve_input_path(args.checkpoint)
    module = RLModule.from_checkpoint(checkpoint)

    env_config = {
        "setup_xml_path": args.setup_xml,
        "segment_duration": args.segment_duration,
        "episode_duration": args.episode_duration,
        "episode_start_offset_s": args.episode_start_offset_s,
        "policy_knots": args.policy_knots,
        "action_mode": args.action_mode,
        "max_delta_rad": args.max_delta_rad,
        "enable_pros_ref_governor": args.pros_ref_governor,
        "pros_ref_model": args.pros_ref_model,
        "pros_ref_lpf_cutoff_hz": args.pros_ref_cutoff_hz,
        "pros_ref_velocity_limit_rad_s": {
            "pros_knee_angle": args.pros_knee_ref_velocity_limit_rad_s,
            "pros_ankle_angle": args.pros_ankle_ref_velocity_limit_rad_s,
        },
        "pros_ref_acceleration_limit_rad_s2": {
            "pros_knee_angle": args.pros_knee_ref_acceleration_limit_rad_s2,
            "pros_ankle_angle": args.pros_ankle_ref_acceleration_limit_rad_s2,
        },
        "pros_ref_jerk_limit_rad_s3": {
            "pros_knee_angle": args.pros_knee_ref_jerk_limit_rad_s3,
            "pros_ankle_angle": args.pros_ankle_ref_jerk_limit_rad_s3,
        },
        "actor_cyclic_phase_only": args.actor_cyclic_phase_only,
        "include_reference_state_observation": args.include_reference_state_observation,
        "imitation_initialize_to_target": args.imitation_initialize_to_target,
        "random_init": False,
        "fail_fast": True,
        "record_outputs": args.record_outputs,
        "save_outputs_on_close": args.record_outputs,
        "grf_mode": args.grf_mode,
        "online_grf_profile_file": args.online_grf_profile,
        "include_online_grf_observation": args.online_grf_observation,
        "critic_privileged_observation": args.asymmetric_actor_critic,
        "prescribed_grf_disabled_sides": args.disable_prescribed_grf_side,
        "online_grf_applied_sides": args.online_grf_applied_side,
        "step_wall_timeout_s": args.step_wall_timeout_s,
    }
    # Same reward shaping as training (reward_function.py), so the evaluated
    # return is comparable to the trained objective.
    # Reward precedence (weak -> strong): snapshot/--config reward, then
    # --reward-json, then --reward-mode. Same shaping as training for comparability.
    reward_overrides = dict(getattr(args, "_cfg_reward", None) or {})
    json_overrides = reward_function.load_reward_overrides(args.reward_json)
    if json_overrides:
        reward_overrides.update(json_overrides)
    if args.reward_mode is not None:
        reward_overrides["reward_mode"] = args.reward_mode
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

    # Upper bound on the number of steps for the progress bar (an episode may end
    # earlier via termination/truncation); capped by --max-steps.
    if args.segment_duration > 0.0 and args.episode_duration:
        total_steps = min(
            args.max_steps,
            int(math.ceil(args.episode_duration / args.segment_duration)),
        )
    else:
        total_steps = args.max_steps
    progress = (
        progress_display.LiveProgress(total=total_steps, label="rollout")
        if args.progress
        else None
    )

    rewards: list[float] = []
    action_abs_max = 0.0
    applied_action_abs_max = 0.0
    action_clipped_steps = 0
    action_clipped_values = 0
    action_values_total = 0
    pelvis_ty_min = float("inf")
    policy_trace: list[dict] = []
    reset_diagnostics: dict = {}
    terminated = truncated = False
    steps = 0
    # Pre-bind for the finally block: if env.reset() itself raises, the progress
    # epilogue must not shadow the real error with a NameError on `info`.
    info: dict = {}
    try:
        process_watchdog.write_heartbeat(
            heartbeat_path,
            "reset inference environment",
            timeout_s=args.step_timeout_s,
        )
        if progress is not None:
            progress.update(phase="reset", phase_reset=True)
            progress.render()
        obs, info = env.reset(seed=args.seed)
        if isinstance(info, dict):
            reset_diagnostics = dict(info.get("reset_diagnostics", {}) or {})
        for _ in range(args.max_steps):
            raw_action = _deterministic_action(module, obs, action_shape)
            # Mirror the env wrapper's clipping so the applied action is traceable.
            action = np.clip(
                raw_action,
                env.action_space.low,
                env.action_space.high,
            ).astype(np.float32)
            clipped_mask = action != raw_action
            action_abs_max = max(
                action_abs_max, float(np.max(np.abs(raw_action)))
            )
            applied_action_abs_max = max(
                applied_action_abs_max, float(np.max(np.abs(action)))
            )
            action_clipped_steps += int(np.any(clipped_mask))
            action_clipped_values += int(np.count_nonzero(clipped_mask))
            action_values_total += int(action.size)
            process_watchdog.write_heartbeat(
                heartbeat_path,
                f"inference simulator step {steps + 1}",
                progress=steps,
                timeout_s=args.step_timeout_s,
            )
            if progress is not None:
                progress.update(phase=f"step {steps + 1}", phase_reset=True)
                progress.render()
            obs, reward, terminated, truncated, info = env.step(action)
            rewards.append(float(reward))
            steps += 1
            if args.record_outputs and isinstance(info, dict):
                imitation_target = info.get("imitation_target", {})
                observation = info.get("observation", {})
                policy_trace.append(
                    {
                        "step": int(steps),
                        "time": float(info.get("time", float("nan"))),
                        "raw_policy_action": np.asarray(
                            raw_action, dtype=float
                        ).tolist(),
                        "applied_policy_action": np.asarray(
                            action, dtype=float
                        ).tolist(),
                        "policy_segment_times": np.asarray(
                            info.get("policy_segment_times", []),
                            dtype=float,
                        ).tolist(),
                        "policy_segment_values": np.asarray(
                            info.get("policy_segment_values", []),
                            dtype=float,
                        ).tolist(),
                        "policy_segment_derivatives": np.asarray(
                            info.get("policy_segment_derivatives", []),
                            dtype=float,
                        ).tolist(),
                        "imitation_target_q": dict(
                            imitation_target.get("q", {})
                            if isinstance(imitation_target, dict)
                            else {}
                        ),
                        "imitation_target_qdot": dict(
                            imitation_target.get("qdot", {})
                            if isinstance(imitation_target, dict)
                            else {}
                        ),
                        "prosthetic_state": {
                            key: float(observation[key])
                            for key in (
                                "pros_knee_angle",
                                "pros_knee_angle_vel",
                                "pros_knee_angle_served_ref",
                                "pros_knee_angle_served_ref_vel",
                                "pros_knee_angle_served_ref_accel",
                                "pros_ankle_angle",
                                "pros_ankle_angle_vel",
                                "pros_ankle_angle_served_ref",
                                "pros_ankle_angle_served_ref_vel",
                                "pros_ankle_angle_served_ref_accel",
                            )
                            if isinstance(observation, dict) and key in observation
                        },
                        "reference_governor_diagnostics": _jsonable(
                            info.get("reference_governor_diagnostics", {})
                        ),
                        "reward_terms": dict(info.get("reward_terms", {}) or {}),
                    }
                )
            process_watchdog.write_heartbeat(
                heartbeat_path,
                f"completed inference simulator step {steps}",
                progress=steps,
                timeout_s=args.step_timeout_s,
            )
            if progress is not None:
                progress.update(
                    completed=steps, suffix=f"reward {progress_display_fmt(reward)}"
                )
                progress.render()
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
        if progress is not None:
            end_reason = (
                info.get("end_reason") if isinstance(info, dict) else None
            )
            progress.finish(
                f"Rollout done: {steps} steps, "
                f"return {progress_display_fmt(float(np.sum(rewards)) if rewards else 0.0)}, "
                f"terminated={terminated} truncated={truncated}"
                + (f" ({end_reason})" if end_reason else "")
            )
        env.close()

    summary = {
        "ok": True,
        "checkpoint": str(args.checkpoint),
        "setup_xml_path": args.setup_xml,
        "episode_start_offset_s": float(args.episode_start_offset_s),
        "imitation_initialize_to_target": bool(args.imitation_initialize_to_target),
        "steps": steps,
        "episode_return": float(np.sum(rewards)) if rewards else 0.0,
        "reward_mean": float(np.mean(rewards)) if rewards else None,
        "reward_min": float(np.min(rewards)) if rewards else None,
        "reward_max": float(np.max(rewards)) if rewards else None,
        "action_abs_max": action_abs_max,
        "applied_action_abs_max": applied_action_abs_max,
        "action_clipped_steps": action_clipped_steps,
        "action_clipped_fraction": (
            float(action_clipped_values / action_values_total)
            if action_values_total
            else 0.0
        ),
        "pelvis_ty_min": None if pelvis_ty_min == float("inf") else pelvis_ty_min,
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "action_shape": list(action_shape),
        "record_outputs": bool(args.record_outputs),
        "grf_mode": args.grf_mode,
        "online_grf_profile_file": args.online_grf_profile,
        "online_grf_observation": bool(args.online_grf_observation),
        "critic_privileged_observation": bool(args.asymmetric_actor_critic),
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
        if args.record_outputs:
            (out / "rollout_policy_trace.json").write_text(
                json.dumps(policy_trace, indent=2), encoding="utf-8"
            )
            (out / "rollout_reset_diagnostics.json").write_text(
                json.dumps(_jsonable(reset_diagnostics), indent=2), encoding="utf-8"
            )
        summary["summary_path"] = str(out / "rollout_summary.json")
    process_watchdog.write_heartbeat(heartbeat_path, "complete", progress=steps)
    return summary


def run_supervised(args: argparse.Namespace) -> int:
    output_dir = _resolve_output_dir(args.output_dir, "rollout_eval")
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
    # First pass: read --checkpoint/--config/--no-auto-config so the trained run's
    # resolved snapshot (and/or a --config file) can seed argparse defaults for the
    # env/model/reward parameters. An explicit CLI flag still overrides them, so the
    # evaluated env matches training without re-typing ~14 flags.
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--checkpoint")
    pre.add_argument("--config", default=None)
    pre.add_argument("--no-auto-config", action="store_true")
    pre_args, _ = pre.parse_known_args()

    layered_defaults: dict = {}
    cfg_reward: dict = {}
    if pre_args.checkpoint and not pre_args.no_auto_config:
        # Resolve the checkpoint the same way --checkpoint is resolved later (the
        # "runs\..." convention maps to "Trajectory Generator\runs\..."), so the
        # sibling training_cfg.resolved.yaml is actually found.
        try:
            ckpt_path = _resolve_input_path(pre_args.checkpoint)
        except (FileNotFoundError, OSError):
            ckpt_path = pre_args.checkpoint
        snapshot = training_config.load_resolved_for_checkpoint(ckpt_path)
        if snapshot:
            flat, reward = training_config.to_argparse_defaults(snapshot)
            layered_defaults.update(flat)
            cfg_reward.update(reward)
    if pre_args.config:
        cfg = training_config.load(training_config.resolve_config_path(pre_args.config))
        flat, reward = training_config.to_argparse_defaults(cfg)
        layered_defaults.update(flat)
        cfg_reward.update(reward)
    # The rollout's output dir is run-identity, never inherited from training.
    layered_defaults.pop("output_dir", None)

    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--checkpoint", required=True, help="rl_module_* directory")
    p.add_argument(
        "--config",
        default=None,
        help="Optional YAML config read as defaults (env/model/reward). Normally "
        "unnecessary: a checkpoint's training_cfg.resolved.yaml is auto-loaded so "
        "the rollout matches training.",
    )
    p.add_argument(
        "--no-auto-config",
        action="store_true",
        help="Do not auto-load the checkpoint's resolved training config snapshot.",
    )
    p.add_argument("--setup-xml", default=_DEFAULT_SETUP_XML)
    p.add_argument("--output-dir")
    p.add_argument("--segment-duration", type=float, default=0.01)
    p.add_argument("--episode-duration", type=float, default=0.5)
    p.add_argument(
        "--episode-start-offset-s",
        type=float,
        default=0.0,
        help="Deterministic offset from setup t_start; must match training.",
    )
    p.add_argument("--policy-knots", type=int, default=3)
    p.add_argument(
        "--action-mode",
        choices=["absolute", "delta", "raw"],
        default="absolute",
        help=(
            "Must match the checkpoint's training action mode. 'absolute' "
            "(default): absolute prosthetic trajectory; 'delta': offset from "
            "prescribed IK; 'raw': raw radians."
        ),
    )
    p.add_argument("--max-delta-rad", type=float, default=0.35)
    p.add_argument(
        "--pros-ref-governor",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Must match the checkpoint's served-reference governor setting.",
    )
    p.add_argument(
        "--pros-ref-model",
        choices=("second_order", "butterworth3_jerk_limited"),
        default="second_order",
        help="Served-reference model; must match training.",
    )
    p.add_argument("--pros-ref-cutoff-hz", type=float, default=6.0)
    p.add_argument("--pros-knee-ref-velocity-limit-rad-s", type=float, default=6.0)
    p.add_argument("--pros-ankle-ref-velocity-limit-rad-s", type=float, default=3.5)
    p.add_argument(
        "--pros-knee-ref-acceleration-limit-rad-s2", type=float, default=60.0
    )
    p.add_argument(
        "--pros-ankle-ref-acceleration-limit-rad-s2", type=float, default=55.0
    )
    p.add_argument("--pros-knee-ref-jerk-limit-rad-s3", type=float, default=3000.0)
    p.add_argument("--pros-ankle-ref-jerk-limit-rad-s3", type=float, default=2750.0)
    p.add_argument(
        "--actor-cyclic-phase-only",
        action=argparse.BooleanOptionalAction,
        default=False,
    )
    p.add_argument(
        "--include-reference-state-observation",
        action=argparse.BooleanOptionalAction,
        default=False,
    )
    p.add_argument(
        "--imitation-initialize-to-target",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Must match training.",
    )
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
        "--asymmetric-actor-critic",
        "--critic-privileged-observation",  # deprecated alias
        dest="asymmetric_actor_critic",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Asymmetric actor-critic: env emits the FULL [actor | privileged] obs "
        "instead of the realistic actor-only obs. Custom RLModule; MUST match "
        "training (normally auto-set from the checkpoint snapshot).",
    )
    p.add_argument(
        "--disable-prescribed-grf-side",
        nargs="*",
        choices=("left", "right"),
        default=[],
        help="Diagnostic opt-in: remove the selected prescribed ExternalForce(s) "
        "from dynamics while retaining the data as an oracle. Space-separated.",
    )
    p.add_argument(
        "--online-grf-applied-side",
        nargs="*",
        choices=("left", "right"),
        default=[],
        help="Hybrid GRF: APPLY the online contact on the given side(s) (prescribed "
        "auto-disabled there). Must match training. Space-separated; use 'left'.",
    )
    p.add_argument(
        "--reward-json",
        default=None,
        help="Reward overrides (JSON file path or inline JSON object of "
        "reward_function.RewardConfig fields); should match the training run.",
    )
    p.add_argument(
        "--reward-mode",
        choices=("ex_novo", "imitation"),
        default=None,
        help="Reward objective ('ex_novo' or 'imitation'); overrides reward_mode "
        "in --reward-json and should match the training run.",
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
        "--step-wall-timeout-s",
        type=float,
        default=30.0,
        help="Per-env-step wall-clock budget [s] inside the env: a degenerate, "
        "pathologically slow segment truncates the rollout gracefully "
        "(end_reason='step_wall_timeout'). 0 disables.",
    )
    p.add_argument(
        "--progress",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Live in-place progress bar with percent, step counter, elapsed and "
        "ETA (default: on). Use --no-progress for plain logging.",
    )
    p.add_argument(
        "--verbose",
        action="store_true",
        help="Restore full Python warnings and Ray/RLlib logging (default: off). "
        "Off keeps a clean log with a single live progress line; turn on to debug.",
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

    # Apply snapshot/--config values as defaults (only for args this script has),
    # then parse so explicit CLI flags win.
    valid_dests = {action.dest for action in p._actions}
    p.set_defaults(**{k: v for k, v in layered_defaults.items() if k in valid_dests})
    args = p.parse_args()
    args._cfg_reward = cfg_reward
    return args


def main() -> None:
    args = parse_args()
    if not args.worker_process:
        raise SystemExit(run_supervised(args))
    summary = run(args)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
