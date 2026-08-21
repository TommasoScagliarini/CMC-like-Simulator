"""Single source of truth for baseline_MLP training/rollout parameters.

A YAML file (``training_cfg.yaml`` next to this module) holds every network and
simulation parameter. It is consumed by ``train_ppo_mlp.py`` and
``rollout_eval.py`` as argparse *defaults*: an explicitly-passed CLI flag still
wins over the YAML (``argparse.set_defaults`` semantics).

Training writes the fully-resolved config (YAML + CLI overrides) to
``<output_dir>/training_cfg.resolved.yaml``. ``rollout_eval.py`` auto-loads that
snapshot from a checkpoint's run directory so the evaluated env/model/reward
always match what was actually trained (no silent mismatch).

Only stdlib + PyYAML here (no numpy/torch/OpenSim): this module is imported in
the early argparse phase, before the heavy training stack and the Windows
torch/OpenSim shim are loaded.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Any, Callable

import yaml

_THIS_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG_PATH = _THIS_DIR / "training_cfg.yaml"
RESOLVED_CONFIG_NAME = "training_cfg.resolved.yaml"

# Sections written to the resolved snapshot are everything EXCEPT run-identity
# (output_dir/resume_from): the snapshot describes the reproducible config, not
# the invocation, and the rollout must not inherit the training run's output dir.
_SNAPSHOT_SKIP_SECTIONS = {"run"}


def _cli_path(value: str | Path) -> Path:
    text = os.fspath(value)
    if os.name != "nt":
        text = text.replace("\\", "/")
    return Path(text).expanduser()


def _warn(message: str) -> None:
    """Emit a config warning on stderr (bypasses the warnings filter that the
    train/rollout scripts disable for a clean console)."""
    print(f"[training_config] WARNING: {message}", file=sys.stderr)


# --- value coercers (yaml.safe_load already yields proper types for well-formed
# YAML; these guard against e.g. lr written as "1e-4" — a string under YAML 1.1 —
# and normalise list/optional fields). -------------------------------------------

def _to_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def _opt(coerce: Callable[[Any], Any]) -> Callable[[Any], Any]:
    def _coerce(value: Any) -> Any:
        return None if value is None else coerce(value)

    return _coerce


def _list_str(value: Any) -> list[str]:
    if value is None:
        return []
    if isinstance(value, (list, tuple)):
        return [str(item) for item in value]
    return [str(value)]


def _list_float(value: Any) -> list[float]:
    if value is None:
        return []
    if isinstance(value, (list, tuple)):
        return [float(item) for item in value]
    return [float(value)]


# Section -> {argparse dest: coercer}. The YAML key IS the argparse dest. This is
# the single source for both flattening (YAML -> defaults) and dumping the
# resolved snapshot, so train and rollout stay aligned.
SECTION_MAP: dict[str, dict[str, Callable[[Any], Any]]] = {
    "model": {
        "num_hidden_layers": int,
        "dim_hidden_layers": int,
        "fcnet_activation": str,
        "asymmetric_actor_critic": _to_bool,
        "rl_module_kind": str,
        "primary_split_v25_residual_input_mean": _opt(_list_float),
        "primary_split_v25_residual_input_std": _opt(_list_float),
        "primary_split_v25_residual_limits": _opt(_list_float),
        "primary_split_v25_residual_init_seed": _opt(int),
        "freeze_logstd": _to_bool,
        "freeze_actor": _to_bool,
        "seed": int,
    },
    "ppo": {
        "train_batch_size": int,
        "minibatch_size": int,
        "num_epochs": int,
        "lr": float,
        "gamma": float,
        "lam": float,
        "clip_param": float,
        "kl_coeff": float,
        "kl_target": float,
        "vf_clip_param": _opt(float),
        "vf_loss_coeff": _opt(float),
    },
    "parallelism": {
        "num_env_runners": int,
        "ray_num_cpus": _opt(int),
        "exact_start_sampling": _to_bool,
    },
    "simulation": {
        "setup_xml": str,
        "segment_duration": float,
        "episode_duration": _opt(float),
        "iterations": int,
        "policy_knots": int,
        "random_init": _to_bool,
        "episode_start_offset_s": float,
        "episode_start_offset_choices_s": _list_float,
        "step_wall_timeout_s": float,
        "grf_penetration_penalty_threshold_m": float,
        "grf_penetration_termination_m": float,
        # Diagnostic-only: "absolute" is the sole production action mode, so these
        # are intentionally omitted from the shipped training_cfg.yaml. They stay
        # valid keys so the resolved snapshot can round-trip a diagnostic run's
        # action_mode/max_delta_rad back into the rollout.
        "action_mode": str,
        "max_delta_rad": float,
        "pros_knee_target_slew_rate_limit_rad_s": float,
        "pros_ankle_target_slew_rate_limit_rad_s": float,
        "pros_ref_governor": _to_bool,
        "pros_ref_model": str,
        "pros_ref_cutoff_hz": float,
        "pros_knee_ref_velocity_limit_rad_s": float,
        "pros_ankle_ref_velocity_limit_rad_s": float,
        "pros_knee_ref_acceleration_limit_rad_s2": float,
        "pros_ankle_ref_acceleration_limit_rad_s2": float,
        "pros_knee_ref_jerk_limit_rad_s3": float,
        "pros_ankle_ref_jerk_limit_rad_s3": float,
        "gait_clock_enable": _to_bool,
        "actor_cyclic_phase_only": _to_bool,
        "include_reference_state_observation": _to_bool,
        "include_controller_state_observation": _to_bool,
        "include_controller_diagnostic_observation": _to_bool,
        "deployable_minimal_observation": _to_bool,
        "imitation_initialize_to_target": _to_bool,
        "reward_reference_range_floor": float,
        "reward_reference_velocity_range_floor": float,
    },
    "grf": {
        "grf_mode": str,
        "online_grf_profile": str,
        "online_grf_detector_profile": _opt(str),
        "binary_phase_detector_profile": _opt(str),
        "phase_fsm_input_mode": str,
        "phase_sensor_on_threshold_n": float,
        "phase_sensor_off_threshold_n": float,
        "phase_sensor_dwell_s": float,
        "detector_sample_dt_s": float,
        "event_contract_id": str,
        "binary_phase_fsm_mode": str,
        "binary_phase_debounce_s": float,
        "binary_phase_invalid_event_policy": str,
        "binary_phase_actor_fsm_version": str,
        "binary_phase_event_contract_id": str,
        "online_grf_observation": _to_bool,
        "online_grf_applied_side": _list_str,
        "disable_prescribed_grf_side": _list_str,
    },
    "supervision": {
        "iteration_timeout_s": float,
        "sample_timeout_s": float,
        "startup_timeout_s": float,
        "checkpoint_timeout_s": float,
        "cleanup_timeout_s": float,
        "max_consecutive_skips": int,
        "max_consecutive_crash_restarts": int,
        "child_self_timeout": _to_bool,
        "checkpoint_every": int,
        "retain_iteration_checkpoints": _to_bool,
        "max_minibatch_mean_kl_loss": _opt(float),
    },
    "logging": {
        "tensorboard": _to_bool,
        "progress": _to_bool,
        "verbose_workers": _to_bool,
    },
    "run": {
        "output_dir": _opt(str),
        "resume_from": _opt(str),
    },
}


def resolve_config_path(value: str | Path | None) -> Path:
    """Resolve a ``--config`` value: absolute honored; relative tried against the
    CWD first, then this module's directory; ``None`` -> the default path."""
    if value is None:
        return DEFAULT_CONFIG_PATH
    path = _cli_path(value)
    if path.is_absolute():
        return path
    cwd_candidate = Path.cwd() / path
    if cwd_candidate.is_file():
        return cwd_candidate.resolve()
    return (_THIS_DIR / path).resolve()


def load(path: str | Path) -> dict[str, Any]:
    """Load a YAML config into a dict. A missing *default* file yields ``{}`` (the
    built-in argparse defaults then apply); a missing *explicit* file is an error."""
    path = _cli_path(path)
    if not path.is_file():
        if path.resolve() == DEFAULT_CONFIG_PATH.resolve():
            return {}
        raise FileNotFoundError(f"config file not found: {path}")
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError(f"config file {path} must be a YAML mapping at the top level")
    return data


def to_argparse_defaults(
    cfg: dict[str, Any] | None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Flatten a nested config dict to ``(flat_defaults, reward_overrides)``.

    ``flat_defaults`` maps argparse dest -> coerced value (feed to
    ``parser.set_defaults``). ``reward_overrides`` is the raw ``reward:`` mapping
    (not an argparse dest). Unknown sections/keys are warned and ignored.
    """
    flat: dict[str, Any] = {}
    reward: dict[str, Any] = {}
    for section, content in (cfg or {}).items():
        if section == "reward":
            if isinstance(content, dict):
                reward = dict(content)
            elif content is not None:
                _warn(f"'reward' must be a mapping, got {type(content).__name__} (ignored)")
            continue
        spec = SECTION_MAP.get(section)
        if spec is None:
            _warn(f"unknown config section '{section}' (ignored)")
            continue
        if content is None:
            continue
        if not isinstance(content, dict):
            _warn(f"config section '{section}' must be a mapping (ignored)")
            continue
        for key, value in content.items():
            # Backward compatibility for configs written before iterations
            # became a simulation-level parameter.
            if section == "supervision" and key == "iterations":
                try:
                    flat[key] = int(value)
                except (TypeError, ValueError) as exc:
                    _warn(f"config '{section}.{key}'={value!r} coercion failed: {exc}")
                continue
            if key not in spec:
                _warn(f"unknown config key '{section}.{key}' (ignored)")
                continue
            try:
                flat[key] = spec[key](value)
            except (TypeError, ValueError) as exc:
                _warn(f"config '{section}.{key}'={value!r} coercion failed: {exc}")
    return flat, reward


def _yaml_safe(value: Any) -> Any:
    if isinstance(value, (list, tuple)):
        return [_yaml_safe(item) for item in value]
    if isinstance(value, Path):
        return str(value)
    return value


def _resolve_reward(reward_overrides: dict[str, Any] | None) -> dict[str, Any]:
    """Full resolved RewardConfig dict (defaults filled) for the snapshot; falls
    back to the raw overrides if reward_function is unavailable."""
    try:
        import reward_function

        return reward_function.RewardConfig.from_mapping(reward_overrides or {}).to_dict()
    except Exception:
        return dict(reward_overrides or {})


def dump_resolved(
    args: Any, reward_overrides: dict[str, Any] | None, path: str | Path
) -> Path:
    """Write the fully-resolved config (YAML + CLI overrides) to ``path`` as the
    rollout-facing snapshot. Reads resolved values off the parsed ``args``."""
    nested: dict[str, Any] = {}
    for section, spec in SECTION_MAP.items():
        if section in _SNAPSHOT_SKIP_SECTIONS:
            continue
        section_dict: dict[str, Any] = {}
        for dest in spec:
            if hasattr(args, dest):
                section_dict[dest] = _yaml_safe(getattr(args, dest))
        if section_dict:
            nested[section] = section_dict
    nested["reward"] = _resolve_reward(reward_overrides)

    path = _cli_path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        handle.write("# Resolved training config (auto-generated): YAML + CLI overrides.\n")
        handle.write("# Read by rollout_eval.py to reconstruct the trained env/model/reward.\n")
        yaml.safe_dump(
            nested, handle, sort_keys=False, default_flow_style=False, allow_unicode=True
        )
    return path


def load_resolved_for_checkpoint(checkpoint_dir: str | Path | None) -> dict[str, Any] | None:
    """Find and load ``training_cfg.resolved.yaml`` for a checkpoint, by walking up
    from the checkpoint dir (e.g. ``<run>/rl_module_best``) to the run dir.
    Returns ``None`` if no snapshot is found (legacy run)."""
    if not checkpoint_dir:
        return None
    base = _cli_path(checkpoint_dir)
    try:
        base = base.resolve()
    except OSError:
        pass
    seen: set[Path] = set()
    for candidate in (base, base.parent, base.parent.parent):
        if candidate in seen:
            continue
        seen.add(candidate)
        snapshot = candidate / RESOLVED_CONFIG_NAME
        if snapshot.is_file():
            try:
                return load(snapshot)
            except (ValueError, OSError) as exc:
                _warn(f"failed to read snapshot {snapshot}: {exc}")
                return None
    return None
