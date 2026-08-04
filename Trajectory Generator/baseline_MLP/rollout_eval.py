"""Rollout / evaluation from a trained baseline-MLP checkpoint.

Loads the exported inference RLModule (``rl_module_best`` / ``rl_module_last``
directory written by ``train_ppo_mlp.py``), runs a greedy rollout on the
CMC-like env, and reports quality metrics. By default the env writes .sto files
that the simulator's existing ``visualize.py`` can replay; use
``--no-record-outputs`` for metric-only evaluation.

Loading only the RLModule (not the full Algorithm) keeps eval lightweight and
needs no Ray cluster / env registration.

Run from the repository root, e.g.:
  python "Trajectory Generator/baseline_MLP/rollout_eval.py"
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
import exploration_noise  # noqa: E402
import training_config  # noqa: E402

_DEFAULT_SETUP_XML = r"models\AB06_SEASEA_Threadmill\AB06_SEASEA_stiff321_500_pi_setup.xml"
_DEFAULT_GRF_MODE = "online_sensor"
_DEFAULT_ONLINE_GRF_PROFILE = (
    "online_grf_profiles/"
    "AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json"
)
_DEFAULT_ONLINE_GRF_DETECTOR_PROFILE = None
_SCRIPT_PATH = Path(__file__).resolve()
_TRAJ_GEN_DIR = _SCRIPT_PATH.parents[1]   # .../Trajectory Generator
_RUNS_ROOT = _TRAJ_GEN_DIR / "runs"
_TRAINING_RUNS_ROOT = _RUNS_ROOT / "training"
_ROLLOUT_RUNS_ROOT = _RUNS_ROOT / "rollout"
_WATCHDOG_FILENAME = "watchdog_state.json"


def _validated_nonnegative_rollout_metric(value, *, field: str) -> float:
    """Return one finite, non-negative rollout metric or fail explicitly.

    Checking before updating a running maximum is important: ``max(0.0, NaN)``
    can evaluate to ``0.0`` and would otherwise hide a corrupted simulator
    diagnostic in an apparently safe rollout summary.
    """
    if isinstance(value, bool):
        raise RuntimeError(f"{field} must be numeric, finite, and non-negative")
    try:
        converted = float(value)
    except (TypeError, ValueError) as exc:
        raise RuntimeError(
            f"{field} must be numeric, finite, and non-negative"
        ) from exc
    if not math.isfinite(converted) or converted < 0.0:
        raise RuntimeError(f"{field} must be finite and non-negative")
    return converted


def _validate_module_observation_contract(
    module,
    actor_feature_names: tuple[str, ...],
    observation_feature_names: tuple[str, ...],
) -> None:
    """Reject silent prefix slicing when an old checkpoint schema has drifted."""
    expected_actor = getattr(module, "_n_actor", None)
    expected_full = getattr(module, "_n_full", None)
    mismatches = []
    if expected_actor is not None and int(expected_actor) != len(actor_feature_names):
        mismatches.append(
            f"actor checkpoint={int(expected_actor)} runtime={len(actor_feature_names)}"
        )
    if expected_full is not None and int(expected_full) != len(observation_feature_names):
        mismatches.append(
            "full observation "
            f"checkpoint={int(expected_full)} runtime={len(observation_feature_names)}"
        )
    if mismatches:
        raise RuntimeError(
            "Checkpoint observation schema does not match the current environment "
            f"({'; '.join(mismatches)}). Prefix slicing would map different feature "
            "semantics; use a compatible historical environment or an explicit "
            "feature adapter."
        )


def _cli_path(value: str | Path) -> Path:
    text = str(value)
    if sys.platform != "win32":
        text = text.replace("\\", "/")
    return Path(text).expanduser()


def _strip_runs_prefix(path: Path, category: str) -> Path:
    parts = path.parts
    if parts and parts[0].lower() == "trajectory generator":
        parts = parts[1:]
    if parts and parts[0].lower() == "runs":
        parts = parts[1:]
    if parts and parts[0].lower() == category.lower():
        parts = parts[1:]
    return Path(*parts) if parts else Path()


def _under(path: Path, root: Path) -> bool:
    try:
        path.resolve().relative_to(root.resolve())
    except ValueError:
        return False
    return True


def _read_json_dict(path: Path) -> dict:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return data if isinstance(data, dict) else {}


def _sanitize_name_suffix(value: str | None) -> str:
    if value is None:
        return ""
    suffix = str(value).strip()
    if not suffix:
        return ""
    invalid = '<>:"/\\|?*'
    suffix = "".join(
        "_" if char in invalid or ord(char) < 32 else char for char in suffix
    ).strip(" .")
    if not suffix:
        return ""
    return suffix if suffix.startswith("_") else f"_{suffix}"


def _unique_path(path: Path) -> Path:
    if not path.exists():
        return path
    for index in range(2, 1000):
        candidate = Path(f"{path}_{index:02d}")
        if not candidate.exists():
            return candidate
    raise RuntimeError(f"Could not find a free output directory name for {path}")


def _training_run_timestamp(run_dir: Path) -> float:
    summary_path = run_dir / "summary.json"
    if summary_path.exists():
        try:
            return summary_path.stat().st_mtime
        except OSError:
            pass
    try:
        return run_dir.stat().st_mtime
    except OSError:
        return 0.0


def _latest_training_checkpoint() -> Path:
    if not _TRAINING_RUNS_ROOT.is_dir():
        raise FileNotFoundError(
            f"Training runs directory not found: {_TRAINING_RUNS_ROOT}. "
            "Pass --checkpoint explicitly."
        )
    candidates: list[tuple[float, str, Path]] = []
    for run_dir in _TRAINING_RUNS_ROOT.iterdir():
        if not run_dir.is_dir():
            continue
        checkpoint = run_dir / "rl_module_best"
        if not checkpoint.is_dir():
            continue
        summary_path = run_dir / "summary.json"
        if summary_path.exists() and _read_json_dict(summary_path).get("ok") is False:
            continue
        candidates.append(
            (_training_run_timestamp(run_dir), run_dir.name.lower(), checkpoint)
        )
    if not candidates:
        raise FileNotFoundError(
            f"No valid training run with rl_module_best found in "
            f"{_TRAINING_RUNS_ROOT}. Pass --checkpoint explicitly."
        )
    candidates.sort(key=lambda item: (item[0], item[1]))
    return candidates[-1][2].resolve()


def _resolve_category_output_dir(output_dir, default_stem, category_root, category):
    if output_dir:
        path = _cli_path(output_dir)
        if path.is_absolute():
            resolved = path.resolve()
            if _under(resolved, _RUNS_ROOT) and not _under(resolved, category_root):
                rel = _strip_runs_prefix(resolved.relative_to(_RUNS_ROOT), category)
                return (category_root / rel).resolve()
            return resolved
        rel = _strip_runs_prefix(path, category)
        path = category_root / rel
    else:
        path = category_root / f"{default_stem}_{datetime.now():%Y%m%d_%H%M%S}"
    return path.resolve()


def _resolve_output_dir(output_dir, default_stem):
    """Resolve ``--output-dir`` so artifacts always land under
    ``Trajectory Generator/runs/rollout`` regardless of the current working
    directory.

    A relative path is taken relative to the rollout runs directory. Historical
    ``runs\\foo`` arguments are treated as ``runs\\rollout\\foo``; absolute paths
    outside ``Trajectory Generator/runs`` are honored as-is. CWD-independent so
    the supervisor and the worker child always agree on the same directory.
    """
    return _resolve_category_output_dir(
        output_dir, default_stem, _ROLLOUT_RUNS_ROOT, "rollout"
    )


def _checkpoint_run_dir(checkpoint: Path) -> Path:
    checkpoint = checkpoint.resolve()
    if checkpoint.name.startswith(("rl_module_", "checkpoint_")):
        return checkpoint.parent
    return checkpoint


def _rollout_name_from_training_run(run_name: str, name_suffix: str | None) -> str:
    lower = run_name.lower()
    index = lower.find("training")
    if index >= 0:
        base = f"{run_name[:index]}rollout{run_name[index + len('training'):]}"
    else:
        base = f"{run_name}_rollout"
    return f"{base}{_sanitize_name_suffix(name_suffix)}"


def _default_rollout_output_dir(
    checkpoint: Path,
    name_suffix: str | None = None,
) -> Path:
    run_dir = _checkpoint_run_dir(checkpoint)
    folder = _rollout_name_from_training_run(run_dir.name, name_suffix)
    return _unique_path((_ROLLOUT_RUNS_ROOT / folder).resolve())


def _resolve_input_path(value: str) -> Path:
    """Resolve an input path (e.g. ``--checkpoint``) accepting both conventions.

    Relative paths are tried against the CWD first (``Trajectory
    Generator\\runs\\...`` from the simulator root) and then against the
    Trajectory Generator directory, so the historical ``runs\\foo`` form keeps
    working after the 2026-06-09 move of ``runs/`` — mirroring how
    ``--output-dir`` is resolved. Absolute paths are honored as-is.
    """
    path = _cli_path(value)
    if path.is_absolute():
        return path.resolve()
    cwd_candidate = (Path.cwd() / path).resolve()
    if cwd_candidate.exists():
        return cwd_candidate
    traj_candidate = (_TRAJ_GEN_DIR / path).resolve()
    if traj_candidate.exists():
        return traj_candidate
    training_candidate = (
        _TRAINING_RUNS_ROOT / _strip_runs_prefix(path, "training")
    ).resolve()
    if training_candidate.exists():
        return training_candidate
    raise FileNotFoundError(
        f"Path {value!r} not found; tried {cwd_candidate}, {traj_candidate}, "
        f"and {training_candidate}."
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


def _policy_action_with_diagnostics(
    module: RLModule,
    obs: np.ndarray,
    action_shape,
    *,
    action_selection: str = "deterministic",
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Select one action and expose the Gaussian mean/std/noise used."""
    obs_t = torch.as_tensor(np.asarray(obs), dtype=torch.float32).reshape(1, -1)
    with torch.no_grad():
        if action_selection == "stochastic":
            fwd = module.forward_exploration({"obs": obs_t})
        else:
            fwd = module.forward_inference({"obs": obs_t})
    logits = fwd["action_dist_inputs"]
    try:
        if action_selection == "stochastic":
            dist_cls = module.get_exploration_action_dist_cls()
            action_t = dist_cls.from_logits(logits).sample()
        else:
            dist_cls = module.get_inference_action_dist_cls()
            action_t = dist_cls.from_logits(logits).to_deterministic().sample()
    except Exception:
        # Flat Box fallback: logits contain Gaussian mean followed by log-std.
        mean = logits[..., : logits.shape[-1] // 2]
        if action_selection == "stochastic":
            log_std = logits[..., logits.shape[-1] // 2 :]
            action_t = mean + torch.exp(log_std) * torch.randn_like(mean)
        else:
            action_t = mean

    action_dim = logits.shape[-1] // 2
    mean_t = logits[..., :action_dim]
    std_t = torch.exp(logits[..., action_dim:])
    noise_t = action_t - mean_t

    def array(value) -> np.ndarray:
        return value.detach().cpu().numpy().reshape(action_shape).astype(np.float32)

    return array(action_t), array(mean_t), array(std_t), array(noise_t)


def _policy_action(
    module: RLModule,
    obs: np.ndarray,
    action_shape,
    *,
    action_selection: str = "deterministic",
) -> np.ndarray:
    """Select one action using the deployment or PPO exploration path."""
    action, _, _, _ = _policy_action_with_diagnostics(
        module,
        obs,
        action_shape,
        action_selection=action_selection,
    )
    return action


def _held_stochastic_action(
    module: RLModule,
    obs: np.ndarray,
    action_shape,
    standard_normal_noise: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Apply held standard-normal noise to the current Gaussian parameters."""
    obs_t = torch.as_tensor(np.asarray(obs), dtype=torch.float32).reshape(1, -1)
    with torch.no_grad():
        fwd = module.forward_exploration({"obs": obs_t})
    logits = fwd["action_dist_inputs"]
    action_dim = logits.shape[-1] // 2
    mean_t = logits[..., :action_dim]
    std_t = torch.exp(logits[..., action_dim:])
    noise_t = torch.as_tensor(
        np.asarray(standard_normal_noise),
        dtype=mean_t.dtype,
        device=mean_t.device,
    ).reshape(mean_t.shape)
    applied_noise_t = std_t * noise_t
    action_t = mean_t + applied_noise_t

    def array(value) -> np.ndarray:
        return value.detach().cpu().numpy().reshape(action_shape).astype(np.float32)

    return (
        array(action_t),
        array(mean_t),
        array(std_t),
        array(applied_noise_t),
    )


def _deterministic_action(module: RLModule, obs: np.ndarray, action_shape) -> np.ndarray:
    """Backward-compatible greedy action helper."""
    return _policy_action(
        module,
        obs,
        action_shape,
        action_selection="deterministic",
    )


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
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
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
        "target_slew_rate_limit_rad_s": {
            "pros_knee_angle": args.pros_knee_target_slew_rate_limit_rad_s,
            "pros_ankle_angle": args.pros_ankle_target_slew_rate_limit_rad_s,
        },
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
        "gait_clock_enable": args.gait_clock_enable,
        "actor_cyclic_phase_only": args.actor_cyclic_phase_only,
        "include_reference_state_observation": args.include_reference_state_observation,
        "include_controller_state_observation": (
            args.include_controller_state_observation
        ),
        "include_controller_diagnostic_observation": (
            args.include_controller_diagnostic_observation
        ),
        "deployable_minimal_observation": args.deployable_minimal_observation,
        "imitation_initialize_to_target": args.imitation_initialize_to_target,
        "reward_reference_range_floor": args.reward_reference_range_floor,
        "reward_reference_velocity_range_floor": (
            args.reward_reference_velocity_range_floor
        ),
        "random_init": False,
        "fail_fast": True,
        "record_outputs": args.record_outputs,
        "save_outputs_on_close": args.record_outputs,
        "grf_mode": args.grf_mode,
        "online_grf_profile_file": args.online_grf_profile,
        "online_grf_detector_profile_file": args.online_grf_detector_profile,
        "phase_fsm_input_mode": args.phase_fsm_input_mode,
        "phase_sensor_on_threshold_n": args.phase_sensor_on_threshold_n,
        "phase_sensor_off_threshold_n": args.phase_sensor_off_threshold_n,
        "phase_sensor_dwell_s": args.phase_sensor_dwell_s,
        "detector_sample_dt_s": args.detector_sample_dt_s,
        "event_contract_id": args.event_contract_id,
        "include_online_grf_observation": args.online_grf_observation,
        "critic_privileged_observation": args.asymmetric_actor_critic,
        "prescribed_grf_disabled_sides": args.disable_prescribed_grf_side,
        "online_grf_applied_sides": args.online_grf_applied_side,
        "step_wall_timeout_s": args.step_wall_timeout_s,
        "grf_penetration_penalty_threshold_m": (
            args.grf_penetration_penalty_threshold_m
        ),
        "grf_penetration_termination_m": args.grf_penetration_termination_m,
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
    base_env = env.unwrapped
    actor_feature_names = tuple(
        str(name) for name in getattr(base_env, "actor_feature_names", ())
    )
    observation_feature_names = tuple(
        str(name) for name in getattr(base_env, "observation_feature_names", ())
    )
    _validate_module_observation_contract(
        module,
        actor_feature_names,
        observation_feature_names,
    )

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
    exploration_noise_samples: list[np.ndarray] = []
    exploration_std_samples: list[np.ndarray] = []
    pelvis_ty_min = float("inf")
    grf_penetration_max_m = 0.0
    grf_penetration_samples = 0
    reserve_norm_max_nm = 0.0
    residual_norm_max_nm = 0.0
    reserve_norm_samples = 0
    residual_norm_samples = 0
    phase_valid_hs_count = 0
    phase_valid_to_count = 0
    phase_valid_cycle_count = 0
    invalid_event_count = 0
    morphology_settled_segments = 0
    morphology_settled_samples = 0
    morphology_discarded_segments = 0
    morphology_discarded_samples = 0
    policy_trace: list[dict] = []
    reset_diagnostics: dict = {}
    terminated = truncated = False
    steps = 0
    held_noise_process = exploration_noise.HeldStandardNormal(
        np.random.default_rng(args.seed),
        action_shape,
        args.exploration_noise_hold_steps,
    )
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
            obs_before = np.asarray(obs, dtype=float).reshape(-1).copy()
            actor_obs_before = obs_before[: len(actor_feature_names)]
            policy_mean = None
            action_noise = None
            if args.action_selection == "stochastic_held":
                raw_action, policy_mean, policy_std, action_noise = (
                    _held_stochastic_action(
                        module,
                        obs,
                        action_shape,
                        held_noise_process.next(),
                    )
                )
                exploration_noise_samples.append(action_noise.copy())
                exploration_std_samples.append(policy_std.copy())
            else:
                if args.action_selection == "stochastic":
                    raw_action, policy_mean, policy_std, action_noise = (
                        _policy_action_with_diagnostics(
                            module,
                            obs,
                            action_shape,
                            action_selection=args.action_selection,
                        )
                    )
                    exploration_noise_samples.append(action_noise.copy())
                    exploration_std_samples.append(policy_std.copy())
                else:
                    raw_action = _policy_action(
                        module,
                        obs,
                        action_shape,
                        action_selection=args.action_selection,
                    )
            # Mirror the env wrapper's clipping for trace/summary only. Step the
            # env with the raw action so RewardShapingWrapper can penalize the
            # raw-vs-applied excursion before FlattenClipAction protects the sim.
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
            obs, reward, terminated, truncated, info = env.step(raw_action)
            rewards.append(float(reward))
            steps += 1
            reward_terms = (
                info.get("reward_terms", {}) if isinstance(info, dict) else {}
            )
            if isinstance(reward_terms, dict):
                if "grf_penetration_m" not in reward_terms:
                    raise RuntimeError(
                        "rollout reward_terms is missing grf_penetration_m"
                    )
                penetration_value = _validated_nonnegative_rollout_metric(
                    reward_terms["grf_penetration_m"],
                    field="grf_penetration_m",
                )
                grf_penetration_max_m = max(
                    grf_penetration_max_m,
                    penetration_value,
                )
                grf_penetration_samples += 1
                if "reserve_norm_nm" in reward_terms:
                    reserve_value = _validated_nonnegative_rollout_metric(
                        reward_terms["reserve_norm_nm"],
                        field="reserve_norm_nm",
                    )
                    reserve_norm_max_nm = max(reserve_norm_max_nm, reserve_value)
                    reserve_norm_samples += 1
                if "residual_norm_nm" in reward_terms:
                    residual_value = float(reward_terms["residual_norm_nm"])
                    if not math.isfinite(residual_value):
                        raise RuntimeError("non-finite residual_norm_nm in rollout")
                    residual_norm_max_nm = max(residual_norm_max_nm, residual_value)
                    residual_norm_samples += 1
                phase_valid_hs_count = max(
                    phase_valid_hs_count,
                    int(reward_terms.get("phase_valid_hs_count", 0) or 0),
                )
                phase_valid_to_count = max(
                    phase_valid_to_count,
                    int(reward_terms.get("phase_valid_to_count", 0) or 0),
                )
                phase_valid_cycle_count = max(
                    phase_valid_cycle_count,
                    int(reward_terms.get("phase_valid_cycle_count", 0) or 0),
                )
                invalid_event_count = max(
                    invalid_event_count,
                    int(reward_terms.get("invalid_event_count", 0) or 0),
                )
                morphology_settled_segments += int(
                    reward_terms.get("morphology_settled_this_step", 0) or 0
                )
                morphology_settled_samples += int(
                    reward_terms.get("morphology_settled_sample_count", 0) or 0
                )
                morphology_discarded_segments += int(
                    reward_terms.get("morphology_discarded_segment_count", 0)
                    or 0
                )
                morphology_discarded_samples += int(
                    reward_terms.get("morphology_discarded_sample_count", 0)
                    or 0
                )
            if (args.record_outputs or args.record_policy_trace) and isinstance(info, dict):
                imitation_target = info.get("imitation_target", {})
                observation = info.get("observation", {})
                policy_trace.append(
                    {
                        "step": int(steps),
                        "time": float(info.get("time", float("nan"))),
                        "raw_policy_action": np.asarray(
                            raw_action, dtype=float
                        ).tolist(),
                        "policy_action_mean": (
                            np.asarray(policy_mean, dtype=float).tolist()
                            if policy_mean is not None
                            else None
                        ),
                        "exploration_action_noise": (
                            np.asarray(action_noise, dtype=float).tolist()
                            if action_noise is not None
                            else None
                        ),
                        "applied_policy_action": np.asarray(
                            action, dtype=float
                        ).tolist(),
                        "actor_observation_before": {
                            name: float(actor_obs_before[index])
                            for index, name in enumerate(actor_feature_names)
                        },
                        "actor_observation_vector_before": (
                            actor_obs_before.astype(float).tolist()
                        ),
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
                        "phase_fsm": _jsonable(info.get("phase_fsm", {})),
                        "detector_sensors": _jsonable(
                            (
                                info.get("online_grf_detector", {}).get(
                                    "sensors", {}
                                )
                                if isinstance(
                                    info.get("online_grf_detector", {}),
                                    dict,
                                )
                                else {}
                            )
                        ),
                        "legacy_online_events": _jsonable(
                            info.get("legacy_online_events", [])
                        ),
                        "online_events": _jsonable(
                            info.get("online_events", [])
                        ),
                        "morphology_completed_segments": _jsonable(
                            info.get("morphology_completed_segments", [])
                        ),
                        "morphology_ledger_diagnostics": _jsonable(
                            info.get("morphology_ledger_diagnostics", {})
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
        "action_mode": args.action_mode,
        "action_selection": args.action_selection,
        "action_seed": int(args.seed),
        "exploration_noise_hold_steps": int(
            args.exploration_noise_hold_steps
        ),
        "exploration_noise_hold_duration_s": float(
            args.exploration_noise_hold_steps * args.segment_duration
        ),
        "exploration_noise_realized_rms": (
            np.sqrt(
                np.mean(
                    np.square(np.asarray(exploration_noise_samples, dtype=float)),
                    axis=0,
                )
            )
            .reshape(-1)
            .astype(float)
            .tolist()
            if exploration_noise_samples
            else None
        ),
        "exploration_std_mean": (
            np.mean(np.asarray(exploration_std_samples, dtype=float), axis=0)
            .reshape(-1)
            .astype(float)
            .tolist()
            if exploration_std_samples
            else None
        ),
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
        "grf_penetration_max_m": float(grf_penetration_max_m),
        "grf_penetration_samples": int(grf_penetration_samples),
        "reserve_norm_max_nm": (
            float(reserve_norm_max_nm) if reserve_norm_samples else None
        ),
        "reserve_norm_samples": int(reserve_norm_samples),
        "residual_norm_max_nm": (
            float(residual_norm_max_nm) if residual_norm_samples else None
        ),
        "residual_norm_samples": int(residual_norm_samples),
        "phase_valid_hs_count": int(phase_valid_hs_count),
        "phase_valid_to_count": int(phase_valid_to_count),
        "phase_valid_cycle_count": int(phase_valid_cycle_count),
        "invalid_event_count": int(invalid_event_count),
        "morphology_settled_segments": int(morphology_settled_segments),
        "morphology_settled_samples": int(morphology_settled_samples),
        "morphology_discarded_segments": int(morphology_discarded_segments),
        "morphology_discarded_samples": int(morphology_discarded_samples),
        "end_reason": info.get("end_reason") if isinstance(info, dict) else None,
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "action_shape": list(action_shape),
        "actor_feature_names": list(actor_feature_names),
        "observation_feature_names": list(observation_feature_names),
        "n_actor": len(actor_feature_names),
        "n_observation": len(observation_feature_names),
        "record_outputs": bool(args.record_outputs),
        "record_policy_trace": bool(args.record_policy_trace),
        "grf_mode": args.grf_mode,
        "online_grf_profile_file": args.online_grf_profile,
        "online_grf_detector_profile_file": args.online_grf_detector_profile,
        "phase_fsm_input_mode": args.phase_fsm_input_mode,
        "phase_sensor_on_threshold_n": float(args.phase_sensor_on_threshold_n),
        "phase_sensor_off_threshold_n": float(args.phase_sensor_off_threshold_n),
        "phase_sensor_dwell_s": float(args.phase_sensor_dwell_s),
        "detector_sample_dt_s": float(args.detector_sample_dt_s),
        "event_contract_id": str(args.event_contract_id),
        "online_grf_observation": bool(args.online_grf_observation),
        "gait_clock_enable": bool(args.gait_clock_enable),
        "deployable_minimal_observation": bool(args.deployable_minimal_observation),
        "include_controller_state_observation": bool(
            args.include_controller_state_observation
        ),
        "include_controller_diagnostic_observation": bool(
            args.include_controller_diagnostic_observation
        ),
        "critic_privileged_observation": bool(args.asymmetric_actor_critic),
        "prescribed_grf_disabled_sides": list(args.disable_prescribed_grf_side),
        "online_grf_applied_sides": list(args.online_grf_applied_side),
        "grf_penetration_penalty_threshold_m": float(
            args.grf_penetration_penalty_threshold_m
        ),
        "grf_penetration_termination_m": float(
            args.grf_penetration_termination_m
        ),
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
        if args.record_outputs or args.record_policy_trace:
            (out / "rollout_policy_trace.json").write_text(
                json.dumps(policy_trace, indent=2), encoding="utf-8"
            )
        if args.record_outputs:
            (out / "rollout_reset_diagnostics.json").write_text(
                json.dumps(_jsonable(reset_diagnostics), indent=2), encoding="utf-8"
            )
        summary["summary_path"] = str(out / "rollout_summary.json")
    process_watchdog.write_heartbeat(heartbeat_path, "complete", progress=steps)
    return summary


def run_supervised(args: argparse.Namespace) -> int:
    output_dir = _resolve_output_dir(args.output_dir, "rollout_eval")
    output_dir.mkdir(parents=True, exist_ok=True)
    child_args = [value for value in sys.argv[1:] if value != "--worker-process"]
    child_args.extend(["--checkpoint", str(args.checkpoint)])
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

    help_requested = any(value in ("-h", "--help") for value in sys.argv[1:])
    default_checkpoint: Path | None = None
    checkpoint_for_snapshot = pre_args.checkpoint
    default_checkpoint_error: Exception | None = None
    if checkpoint_for_snapshot is None and not help_requested:
        try:
            default_checkpoint = _latest_training_checkpoint()
            checkpoint_for_snapshot = str(default_checkpoint)
        except (FileNotFoundError, OSError) as exc:
            default_checkpoint_error = exc

    layered_defaults: dict = {}
    cfg_reward: dict = {}
    if checkpoint_for_snapshot and not pre_args.no_auto_config:
        # Resolve the checkpoint the same way --checkpoint is resolved later (the
        # "runs\..." convention maps to "Trajectory Generator\runs\..."), so the
        # sibling training_cfg.resolved.yaml is actually found.
        try:
            ckpt_path = _resolve_input_path(checkpoint_for_snapshot)
        except (FileNotFoundError, OSError):
            ckpt_path = checkpoint_for_snapshot
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
    p.add_argument(
        "--checkpoint",
        default=str(default_checkpoint) if default_checkpoint is not None else None,
        help=(
            "rl_module_* directory. If omitted, the latest valid run in "
            "Trajectory Generator/runs/training is used."
        ),
    )
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
    p.add_argument(
        "--name",
        default=None,
        help=(
            "Optional suffix for the auto-generated rollout folder name "
            "(e.g. --name _example). Ignored when --output-dir is provided."
        ),
    )
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
        "--pros-knee-target-slew-rate-limit-rad-s",
        type=float,
        default=0.0,
        help=(
            "Max target-to-target rate for generated prosthetic knee references. "
            "Non-positive disables the limiter."
        ),
    )
    p.add_argument(
        "--pros-ankle-target-slew-rate-limit-rad-s",
        type=float,
        default=0.0,
        help=(
            "Max target-to-target rate for generated prosthetic ankle references. "
            "Non-positive disables the limiter."
        ),
    )
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
        "--gait-clock-enable",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Expose the prescribed sound-leg gait clock. Disable for clean "
            "prosthesis-phase ex-novo evaluation."
        ),
    )
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
        "--include-controller-state-observation",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Must match training; core controller state otherwise remains critic-only.",
    )
    p.add_argument(
        "--include-controller-diagnostic-observation",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Must match training; derived command diagnostics otherwise remain critic-only.",
    )
    p.add_argument(
        "--deployable-minimal-observation",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Restrict actor obs to prosthetic angles, online GRF detector, and FSM.",
    )
    p.add_argument(
        "--imitation-initialize-to-target",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Must match training.",
    )
    p.add_argument(
        "--reward-reference-range-floor",
        type=float,
        default=0.05,
        help="Must match training.",
    )
    p.add_argument(
        "--reward-reference-velocity-range-floor",
        type=float,
        default=0.1,
        help="Must match training.",
    )
    p.add_argument("--max-steps", type=int, default=10000)
    p.add_argument("--seed", type=int, default=123)
    p.add_argument(
        "--action-selection",
        choices=("deterministic", "stochastic", "stochastic_held"),
        default="deterministic",
        help=(
            "deterministic uses the Gaussian mean; stochastic uses RLlib's "
            "forward_exploration distribution exactly as PPO sampling does; "
            "stochastic_held applies the same Gaussian parameters while holding "
            "each standard-normal draw for multiple policy steps."
        ),
    )
    p.add_argument(
        "--exploration-noise-hold-steps",
        type=int,
        default=1,
        help="For stochastic_held, retain each noise draw for this many steps.",
    )
    p.add_argument(
        "--record-outputs",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Write detailed simulator outputs under <output-dir>/sim_outputs "
            "(default: on). Use --no-record-outputs for metric-only rollout."
        ),
    )
    p.add_argument(
        "--record-policy-trace",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Write rollout_policy_trace.json without enabling the simulator's "
            "detailed .sto outputs. Diagnostic only; does not alter the policy."
        ),
    )
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
        "--online-grf-detector-profile",
        default=_DEFAULT_ONLINE_GRF_DETECTOR_PROFILE,
        help=(
            "Optional second onlineGRF profile used only for HS/TO detection. "
            "It is added as sensor-only contact and never applied to dynamics."
        ),
    )
    p.add_argument(
        "--phase-fsm-input-mode",
        choices=("legacy_events", "shadow", "two_sensor"),
        default="legacy_events",
        help=(
            "Input source for the existing prosthetic phase FSM. 'shadow' "
            "records heel/toe candidates while legacy events remain active."
        ),
    )
    p.add_argument("--phase-sensor-on-threshold-n", type=float, default=5.0)
    p.add_argument("--phase-sensor-off-threshold-n", type=float, default=2.0)
    p.add_argument("--phase-sensor-dwell-s", type=float, default=0.03)
    p.add_argument("--detector-sample-dt-s", type=float, default=0.001)
    p.add_argument("--event-contract-id", default="legacy_events_v1")
    p.add_argument(
        "--grf-penetration-penalty-threshold-m",
        type=float,
        default=0.012,
        help="Soft online-contact penetration threshold [m]; must match training.",
    )
    p.add_argument(
        "--grf-penetration-termination-m",
        type=float,
        default=0.017,
        help="Hard online-contact penetration termination [m]; must match training.",
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
    if args.checkpoint is None:
        message = (
            str(default_checkpoint_error)
            if default_checkpoint_error is not None
            else "No checkpoint was provided and no default checkpoint was found."
        )
        p.error(f"{message} Pass --checkpoint explicitly.")
    if args.output_dir is None:
        try:
            checkpoint_path = _resolve_input_path(args.checkpoint)
        except (FileNotFoundError, OSError) as exc:
            p.error(
                "Cannot resolve checkpoint for automatic rollout output naming: "
                f"{exc}"
            )
        args.checkpoint = str(checkpoint_path)
        args.output_dir = str(_default_rollout_output_dir(checkpoint_path, args.name))
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
