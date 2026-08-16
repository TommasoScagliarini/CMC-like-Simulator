#!/usr/bin/env python3
"""Fail-closed V12R9 production-topology preflight and post-run validator.

The restore preflight deliberately builds the exact final 12-remote-EnvRunner
PPO configuration and restores ``checkpoint_zero`` without invoking an update
or collecting an environment batch.  The post-run action is separate and
accepts only the completed 50-update artifact set.

Importing this module, ``--source-check``, and ``--print-plan`` are source-only.
Only the explicit ``--restore-preflight`` action starts Ray.  Only the explicit
``--post-run-audit`` action writes the final run audit.
"""

from __future__ import annotations

import argparse
import ast
import copy
import hashlib
import json
import math
import os
import sys
import tempfile
import time
from collections.abc import Mapping, Sequence
from pathlib import Path, PurePath, PurePosixPath
from typing import Any, Callable

import numpy as np

try:
    from . import h0_v12r9_training_contract as contract
except ImportError:  # Direct execution with this directory on ``sys.path``.
    import h0_v12r9_training_contract as contract


class TrainingReadinessError(RuntimeError):
    """Raised when a readiness or post-run invariant drifts."""


REPO_ROOT = Path(__file__).absolute().parents[4]
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
BASELINE_ROOT = TRAJECTORY_ROOT / "baseline_MLP"
LOCAL_VALIDATION_ROOT = BASELINE_ROOT / "validation"
for _root in (
    BASELINE_ROOT,
    TRAJECTORY_ROOT,
    REPO_ROOT,
    REPO_ROOT / "validation",
    LOCAL_VALIDATION_ROOT / "v12r9zero",
    LOCAL_VALIDATION_ROOT / "v12r9morph",
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import run_h0_v12r9_morphology as morph_runner  # noqa: E402
import run_h0_v12r9_zero_checkpoint as zero_runner  # noqa: E402


# Stable literal ABI consumed read-only by the upstream morphology handoff.
# Keep these strings whole: they are intentionally auditable without importing
# this module (and therefore without exposing any live execution surface).
VALIDATION_ABI_LITERALS = {
    "protocol_id": "AB06_H0_V12R9_MORPH_TRAINING_50_UPDATE_READINESS",
    "preflight_receipt_path": "Trajectory Generator/baseline_MLP/validation/v12r9training/h0_v12r9_training_preflight_20260814/receipt.json",
    "preflight_required_status": "PASS_H0_V12R9_TRAINING_PREFLIGHT_12_RUNNER_RESTORE",
    "postrun_audit_path": "Trajectory Generator/runs/training/v12r9_morphology_0025_50update/v12r9_training_integrity_audit.json",
    "postrun_required_status": "PASS_H0_V12R9_TRAINING_50_UPDATE_INTEGRITY",
}


def require_repository_root_cwd(
    cwd: str | Path | None = None,
) -> dict[str, str | bool]:
    observed = Path.cwd() if cwd is None else Path(cwd)
    observed_real = os.path.normcase(os.path.realpath(os.path.abspath(observed)))
    expected_real = os.path.normcase(os.path.realpath(os.path.abspath(REPO_ROOT)))
    if observed_real != expected_real:
        raise TrainingReadinessError(
            "V12R9 training validation requires working "
            f"directory=repository_root; observed {observed}"
        )
    return {
        "passed": True,
        "required_working_directory": contract.REQUIRED_WORKING_DIRECTORY,
        "observed_working_directory": "repository_root",
    }


def resolve_relative(value: str | PurePath) -> Path:
    text = str(value)
    pure = PurePosixPath(text)
    if (
        not text
        or text == "."
        or pure.is_absolute()
        or ".." in pure.parts
        or "\\" in text
        or pure.as_posix() != text
    ):
        raise TrainingReadinessError(
            f"non-canonical repository-relative path: {value!r}"
        )
    return REPO_ROOT.joinpath(*pure.parts)


PREFLIGHT_RECEIPT = resolve_relative(contract.PREFLIGHT_RECEIPT_PATH)
FINAL_OUTPUT_DIR = resolve_relative(contract.FINAL_OUTPUT_DIR)
POSTRUN_AUDIT = resolve_relative(contract.POSTRUN_AUDIT_PATH)
CHECKPOINT_ZERO = resolve_relative(contract.CHECKPOINT_ZERO_PATH)
ZERO_AUDIT = resolve_relative(contract.ZERO_AUDIT_PATH)
MORPH_HANDOFF = resolve_relative(contract.MORPH_TERMINAL_ENDPOINT["handoff_path"])
SOURCE_PATHS = {
    name: resolve_relative(path)
    for name, path in contract.SOURCE_RELATIVE_PATHS.items()
}


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise TrainingReadinessError(f"duplicate JSON key: {key!r}")
        result[key] = value
    return result


def _reject_nonfinite_constant(value: str) -> None:
    raise TrainingReadinessError(f"non-finite JSON constant: {value}")


def _finite_json(value: Any, label: str) -> None:
    if value is None or type(value) in {bool, str, int}:
        return
    if isinstance(value, float):
        if not math.isfinite(value):
            raise TrainingReadinessError(f"{label} contains a non-finite value")
        return
    if isinstance(value, Mapping):
        for key, child in value.items():
            if not isinstance(key, str) or not key:
                raise TrainingReadinessError(f"{label} has an invalid key")
            _finite_json(child, f"{label}.{key}")
        return
    if isinstance(value, Sequence) and not isinstance(
        value, (str, bytes, bytearray)
    ):
        for index, child in enumerate(value):
            _finite_json(child, f"{label}[{index}]")
        return
    raise TrainingReadinessError(
        f"{label} is not strict JSON: {type(value).__name__}"
    )


def strict_json_any(path: str | Path) -> Any:
    file_path = Path(path)
    try:
        value = json.loads(
            file_path.read_text(encoding="utf-8"),
            object_pairs_hook=_reject_duplicate_keys,
            parse_constant=_reject_nonfinite_constant,
        )
    except TrainingReadinessError:
        raise
    except (FileNotFoundError, OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise TrainingReadinessError(f"could not read strict JSON {file_path}") from exc
    _finite_json(value, str(file_path))
    return value


def strict_mapping(path: str | Path) -> dict[str, Any]:
    value = strict_json_any(path)
    if not isinstance(value, Mapping):
        raise TrainingReadinessError(f"expected JSON mapping: {path}")
    return copy.deepcopy(dict(value))


def strict_jsonl(path: str | Path) -> list[dict[str, Any]]:
    file_path = Path(path)
    try:
        lines = file_path.read_text(encoding="utf-8").splitlines()
    except (FileNotFoundError, OSError, UnicodeError) as exc:
        raise TrainingReadinessError(f"could not read JSONL {file_path}") from exc
    rows: list[dict[str, Any]] = []
    for line_number, line in enumerate(lines, 1):
        if not line.strip():
            raise TrainingReadinessError(
                f"blank line in canonical JSONL {file_path}:{line_number}"
            )
        try:
            value = json.loads(
                line,
                object_pairs_hook=_reject_duplicate_keys,
                parse_constant=_reject_nonfinite_constant,
            )
        except TrainingReadinessError:
            raise
        except json.JSONDecodeError as exc:
            raise TrainingReadinessError(
                f"invalid JSONL row {file_path}:{line_number}"
            ) from exc
        if not isinstance(value, Mapping):
            raise TrainingReadinessError(
                f"non-mapping JSONL row {file_path}:{line_number}"
            )
        row = copy.deepcopy(dict(value))
        _finite_json(row, f"{file_path}:{line_number}")
        rows.append(row)
    return rows


def _same_json(left: Any, right: Any) -> bool:
    try:
        return json.dumps(
            left, sort_keys=True, separators=(",", ":"), allow_nan=False
        ) == json.dumps(
            right, sort_keys=True, separators=(",", ":"), allow_nan=False
        )
    except (TypeError, ValueError):
        return False


def _same_config_value(left: Any, right: Any) -> bool:
    if (
        not isinstance(left, bool)
        and not isinstance(right, bool)
        and isinstance(left, (int, float))
        and isinstance(right, (int, float))
    ):
        return math.isfinite(float(left)) and math.isfinite(float(right)) and float(
            left
        ) == float(right)
    return _same_json(left, right)


def _same_path(value: Any, expected: Path) -> bool:
    if not isinstance(value, (str, os.PathLike)):
        return False
    try:
        candidate = Path(value)
        if not candidate.is_absolute():
            candidate = REPO_ROOT / candidate
        return candidate.resolve(strict=False) == expected.resolve(strict=False)
    except (OSError, TypeError, ValueError):
        return False


def _artifact_record(path: str | Path) -> dict[str, Any]:
    return zero_runner.artifact_record(Path(path))


def source_closure_snapshot() -> dict[str, Any]:
    return {
        name: _artifact_record(path)
        for name, path in sorted(SOURCE_PATHS.items())
    }


def _load_train_module() -> Any:
    import train_ppo_mlp

    return train_ppo_mlp


def parse_final_training_args(platform_id: str = "macos_arm64") -> tuple[Any, Any]:
    """Parse and validate the immutable handoff without importing the live stack."""

    train = _load_train_module()
    argv = contract.final_training_argv(platform_id)
    previous = list(sys.argv)
    try:
        # argv[0] is the command interpreter and argv[1] is the launcher script.
        sys.argv = [argv[1], *argv[2:]]
        args = train.parse_args()
    finally:
        sys.argv = previous
    train._validate_rl_module_args(args)  # noqa: SLF001
    train._validate_warm_start_args(args)  # noqa: SLF001
    train._validate_start_sampling_args(args)  # noqa: SLF001
    train._validate_kl_guard_args(args)  # noqa: SLF001
    return train, args


def _inline_reward_overrides(args: Any) -> dict[str, Any]:
    raw = getattr(args, "reward_json", None)
    if not isinstance(raw, str) or not raw.lstrip().startswith("{"):
        raise TrainingReadinessError("final reward override must be inline JSON")
    try:
        value = json.loads(
            raw,
            object_pairs_hook=_reject_duplicate_keys,
            parse_constant=_reject_nonfinite_constant,
        )
    except (json.JSONDecodeError, TrainingReadinessError) as exc:
        raise TrainingReadinessError("final reward JSON is invalid") from exc
    if not isinstance(value, Mapping):
        raise TrainingReadinessError("final reward override is not a mapping")
    result = copy.deepcopy(dict(value))
    if not _same_json(result, contract.EXPECTED_REWARD_OVERRIDES):
        raise TrainingReadinessError("final positive reward overrides drifted")
    return result


def target_args_snapshot(args: Any) -> dict[str, Any]:
    sampling = getattr(args, "_start_sampling_contract", None)
    if sampling is None:
        raise TrainingReadinessError("exact-start sampling contract is absent")
    observed = {
        "iterations": int(args.iterations),
        "num_env_runners": int(args.num_env_runners),
        "ray_num_cpus": int(args.ray_num_cpus),
        "train_batch_size": int(args.train_batch_size),
        "minibatch_size": int(args.minibatch_size),
        "num_epochs": int(args.num_epochs),
        "lr": float(args.lr),
        "gamma": float(args.gamma),
        "lambda": float(args.lam),
        "clip_param": float(args.clip_param),
        "kl_coeff": float(args.kl_coeff),
        "kl_target": float(args.kl_target),
        "max_minibatch_mean_kl_loss": float(args.max_minibatch_mean_kl_loss),
        "max_consecutive_skips": int(args.max_consecutive_skips),
        "max_consecutive_crash_restarts": int(args.max_consecutive_crash_restarts),
        "retain_iteration_checkpoints": bool(args.retain_iteration_checkpoints),
        "checkpoint_every": int(args.checkpoint_every),
        "exact_start_sampling": bool(args.exact_start_sampling),
        "start_offsets_s": [float(value) for value in sampling.offsets_s],
        "rollout_fragment_length": int(sampling.rollout_fragment_length),
        "steps_per_start": int(sampling.expected_steps_per_start),
        "runners_per_start": int(sampling.runners_per_start),
        "rl_module_kind": str(args.rl_module_kind),
        "num_hidden_layers": int(args.num_hidden_layers),
        "dim_hidden_layers": int(args.dim_hidden_layers),
        "asymmetric_actor_critic": bool(args.asymmetric_actor_critic),
        "freeze_logstd": bool(args.freeze_logstd),
        "resume_from": str(args.resume_from),
        "output_dir": str(args.output_dir),
        "warm_start": bool(args.warm_start),
        "warm_start_raw": bool(args.warm_start_raw),
        "phase_fsm_input_mode": str(args.phase_fsm_input_mode),
        "event_contract_id": str(args.event_contract_id),
        "binary_phase_fsm_mode": str(args.binary_phase_fsm_mode),
        "binary_phase_detector_profile": str(args.binary_phase_detector_profile),
        "detector_sample_dt_s": float(args.detector_sample_dt_s),
        "binary_phase_debounce_s": float(args.binary_phase_debounce_s),
        "binary_phase_event_contract_id": str(
            args.binary_phase_event_contract_id
        ),
        "reward_overrides": _inline_reward_overrides(args),
    }
    expected = {
        "iterations": contract.FINAL_TRAINING_ITERATIONS,
        "num_env_runners": contract.EXPECTED_REMOTE_ENV_RUNNERS,
        "ray_num_cpus": contract.EXPECTED_RAY_NUM_CPUS,
        "train_batch_size": contract.EXPECTED_TRAIN_BATCH_SIZE,
        "minibatch_size": contract.EXPECTED_MINIBATCH_SIZE,
        "num_epochs": contract.EXPECTED_NUM_EPOCHS,
        "lr": contract.EXPECTED_LR,
        "gamma": contract.EXPECTED_GAMMA,
        "lambda": contract.EXPECTED_LAMBDA,
        "clip_param": contract.EXPECTED_CLIP_PARAM,
        "kl_coeff": contract.EXPECTED_KL_COEFF,
        "kl_target": contract.EXPECTED_KL_TARGET,
        "max_minibatch_mean_kl_loss": (
            contract.EXPECTED_MAX_MINIBATCH_MEAN_KL_LOSS
        ),
        "max_consecutive_skips": 1,
        "max_consecutive_crash_restarts": 1,
        "retain_iteration_checkpoints": True,
        "checkpoint_every": 1,
        "exact_start_sampling": True,
        "start_offsets_s": list(contract.EXPECTED_START_OFFSETS_S),
        "rollout_fragment_length": contract.EXPECTED_ROLLOUT_FRAGMENT_LENGTH,
        "steps_per_start": contract.EXPECTED_STEPS_PER_START,
        "runners_per_start": contract.EXPECTED_RUNNERS_PER_START,
        "rl_module_kind": contract.EXPECTED_MODEL["rl_module_kind"],
        "num_hidden_layers": contract.EXPECTED_MODEL["num_hidden_layers"],
        "dim_hidden_layers": contract.EXPECTED_MODEL["dim_hidden_layers"],
        "asymmetric_actor_critic": True,
        "freeze_logstd": True,
        "resume_from": contract.CHECKPOINT_ZERO_PATH.as_posix(),
        "output_dir": contract.FINAL_OUTPUT_DIR.as_posix(),
        "warm_start": False,
        "warm_start_raw": False,
        "phase_fsm_input_mode": "legacy_events",
        "event_contract_id": contract.morph.q3.LEGACY_EVENT_CONTRACT_ID,
        "binary_phase_fsm_mode": contract.morph.q3.V26_BINARY_MODE,
        "binary_phase_detector_profile": (
            contract.morph.q3.DETECTOR_PROFILE_PATH.as_posix()
        ),
        "detector_sample_dt_s": 0.001,
        "binary_phase_debounce_s": 0.005,
        "binary_phase_event_contract_id": contract.morph.q3.EVENT_CONTRACT_ID,
        "reward_overrides": copy.deepcopy(contract.EXPECTED_REWARD_OVERRIDES),
    }
    if not _same_json(observed, expected):
        drift = {
            key: {"observed": observed.get(key), "expected": expected.get(key)}
            for key in expected
            if not _same_json(observed.get(key), expected.get(key))
        }
        raise TrainingReadinessError(f"final target args drifted: {drift}")
    return observed


def build_preflight_plan() -> dict[str, Any]:
    argv = contract.final_training_argv("macos_arm64")
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "PLAN_H0_V12R9_TRAINING_PREFLIGHT_12_RUNNER_RESTORE",
        "protocol_id": contract.PROTOCOL_ID,
        "source_only": True,
        "required_working_directory": contract.REQUIRED_WORKING_DIRECTORY,
        "upstream": {
            "morph": copy.deepcopy(contract.MORPH_TERMINAL_ENDPOINT),
            "zero": copy.deepcopy(contract.ZERO_TERMINAL_ENDPOINT),
        },
        "training_argv": list(argv),
        "training_command": contract.render_training_command("macos_arm64"),
        "preflight": {
            "remote_env_runners": contract.EXPECTED_REMOTE_ENV_RUNNERS,
            "local_env_runners": contract.EXPECTED_LOCAL_ENV_RUNNERS,
            "env_runner_surfaces": contract.EXPECTED_ENV_RUNNER_SURFACES,
            "ray_num_cpus": contract.EXPECTED_RAY_NUM_CPUS,
            "algorithm_builds": 1,
            "checkpoint_restores": 1,
            "algorithm_update_calls": 0,
            "environment_sampling_calls": 0,
            "receipt": contract.PREFLIGHT_RECEIPT_PATH.as_posix(),
            "required_status": contract.PREFLIGHT_PASS_STATUS,
        },
        "postrun": {
            "expected_updates": contract.FINAL_TRAINING_ITERATIONS,
            "expected_env_steps": contract.EXPECTED_LIFETIME_ENV_STEPS,
            "audit": contract.POSTRUN_AUDIT_PATH.as_posix(),
            "required_status": contract.POSTRUN_PASS_STATUS,
        },
    }


def source_check() -> dict[str, Any]:
    contract_result = contract.contract_self_check()
    _, args = parse_final_training_args()
    target = target_args_snapshot(args)
    validator_tree = ast.parse(Path(__file__).read_text(encoding="utf-8"))
    forbidden_attributes = sorted(
        {
            node.attr
            for node in ast.walk(validator_tree)
            if isinstance(node, ast.Attribute) and node.attr in {"train", "sample"}
        }
    )
    launcher_source = SOURCE_PATHS["morph_training_launcher"].read_text(
        encoding="utf-8"
    )
    causal_source = SOURCE_PATHS["morph_causal_runtime"].read_text(
        encoding="utf-8"
    )
    site_source = SOURCE_PATHS["morph_site_hook"].read_text(encoding="utf-8")
    train_source = SOURCE_PATHS["training_entrypoint"].read_text(encoding="utf-8")
    configure_at = launcher_source.find("configure_process()")
    main_at = launcher_source.find("train_ppo_mlp.main()")
    checks = {
        "contract": contract_result["passed"] is True,
        "literal_abi": VALIDATION_ABI_LITERALS
        == {
            "protocol_id": contract.PROTOCOL_ID,
            "preflight_receipt_path": contract.PREFLIGHT_RECEIPT_PATH.as_posix(),
            "preflight_required_status": contract.PREFLIGHT_PASS_STATUS,
            "postrun_audit_path": contract.POSTRUN_AUDIT_PATH.as_posix(),
            "postrun_required_status": contract.POSTRUN_PASS_STATUS,
        },
        "all_sources_exist": all(path.is_file() for path in SOURCE_PATHS.values()),
        "final_args": bool(target),
        "validator_has_no_update_or_sampling_attribute_call": not forbidden_attributes,
        "launcher_before_training_main": 0 <= configure_at < main_at,
        "launcher_sets_marker": "SITE_MARKER_ENV" in launcher_source
        and "PYTHONPATH" in launcher_source,
        "site_hook_is_opt_in": "os.environ.get(_MARKER) == _RUNTIME_ID" in site_source,
        "runtime_installs_reward_and_corridor": (
            "corridor.CausalDelayedMorphologyBuffer" in causal_source
            and "reward.CausalDelayedMorphologyBuffer" in causal_source
        ),
        "supervisor_child_inherits_environment": "env = os.environ.copy()"
        in train_source,
        "ray_worker_setup_hook_present": 'runtime_env["worker_process_setup_hook"]'
        in train_source,
    }
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            "PASS_H0_V12R9_TRAINING_SOURCE_CHECK"
            if all(checks.values())
            else "FAIL_H0_V12R9_TRAINING_SOURCE_CHECK"
        ),
        "passed": all(checks.values()),
        "source_only": True,
        "training_ready": False,
        "reason": "live restore preflight and morphology terminal PASS still required",
        "checks": checks,
        "forbidden_attributes": forbidden_attributes,
        "target_args": target,
        "plan": build_preflight_plan(),
    }


def _array(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    return np.ascontiguousarray(np.asarray(value))


def _tensor_digest(value: Any) -> str:
    array = _array(value)
    digest = hashlib.sha256()
    digest.update(str(array.dtype).encode("ascii"))
    digest.update(repr(tuple(int(item) for item in array.shape)).encode("ascii"))
    digest.update(array.tobytes(order="C"))
    return digest.hexdigest()


def _partition_surface(state: Mapping[str, Any], *, actor: bool) -> dict[str, Any]:
    actor_names = set(zero_runner.ACTOR_STATE_SHAPES)
    names = sorted(name for name in state if (name in actor_names) is actor)
    if actor and names != sorted(actor_names):
        raise TrainingReadinessError("actor state surface drifted")
    if not actor and not names:
        raise TrainingReadinessError("critic state surface is empty")
    digest = hashlib.sha256()
    byte_count = 0
    for name in names:
        array = _array(state[name])
        byte_count += int(array.nbytes)
        digest.update(name.encode("utf-8"))
        digest.update(_tensor_digest(array).encode("ascii"))
    if actor:
        return {
            "actor_digest": zero_runner.actor_state_digest(state),
            "actor_state_sha256": digest.hexdigest(),
            "actor_key_count": len(names),
            "actor_byte_count": byte_count,
        }
    return {
        "critic_state_sha256": digest.hexdigest(),
        "critic_key_count": len(names),
        "critic_byte_count": byte_count,
    }


def _env_runner_restore_probe(env_runner: Any) -> dict[str, Any]:
    """Executed on each local/remote EnvRunner; it does not touch an env batch."""

    from ray.rllib.algorithms.algorithm import COMPONENT_RL_MODULE
    import h0_v12r9_morphology_causal_runtime as causal_runtime

    state = env_runner.get_state(
        components=[COMPONENT_RL_MODULE], inference_only=True
    )
    runtime = causal_runtime.assert_installed()
    return {
        "worker_index": int(env_runner.worker_index or 0),
        "pid": os.getpid(),
        "module_state": state,
        "runtime": runtime,
        "marker": os.environ.get(causal_runtime.SITE_MARKER_ENV),
        "pythonpath_has_hook": str(causal_runtime.HERE)
        in os.environ.get("PYTHONPATH", "").split(os.pathsep),
    }


def _env_runner_surfaces(
    *, algo: Any, expected_actor: Mapping[str, Any], warm_start: Any, timeout_s: float
) -> list[dict[str, Any]]:
    raw = algo.env_runner_group.foreach_env_runner(
        func=_env_runner_restore_probe,
        local_env_runner=True,
        timeout_seconds=float(timeout_s),
    )
    if not isinstance(raw, Sequence) or isinstance(raw, (str, bytes, bytearray)):
        raise TrainingReadinessError("EnvRunner probe did not return a sequence")
    rows: list[dict[str, Any]] = []
    for item in raw:
        if not isinstance(item, Mapping):
            raise TrainingReadinessError("EnvRunner probe returned a non-mapping")
        actor = warm_start.find_actor_state(item.get("module_state"))
        if actor is None:
            raise TrainingReadinessError("EnvRunner probe has no actor state")
        surface = _partition_surface(actor, actor=True)
        if not _same_json(surface, expected_actor):
            raise TrainingReadinessError(
                f"EnvRunner {item.get('worker_index')} actor differs from checkpoint-zero"
            )
        runtime = item.get("runtime")
        if (
            not isinstance(runtime, Mapping)
            or runtime.get("passed") is not True
            or runtime.get("runtime_id") != contract.EXPECTED_CAUSAL_RUNTIME_ID
            or item.get("marker") != contract.EXPECTED_CAUSAL_RUNTIME_ID
            or item.get("pythonpath_has_hook") is not True
        ):
            raise TrainingReadinessError(
                f"EnvRunner {item.get('worker_index')} lacks causal runtime"
            )
        rows.append(
            {
                "worker_index": item.get("worker_index"),
                "pid": item.get("pid"),
                "actor": surface,
                "causal_runtime": copy.deepcopy(dict(runtime)),
                "marker": item.get("marker"),
                "pythonpath_has_hook": item.get("pythonpath_has_hook"),
            }
        )
    rows.sort(key=lambda row: int(row["worker_index"]))
    indices = tuple(int(row["worker_index"]) for row in rows)
    if (
        len(rows) != contract.EXPECTED_ENV_RUNNER_SURFACES
        or indices != contract.EXPECTED_RUNNER_INDICES
    ):
        raise TrainingReadinessError(
            f"expected EnvRunner indices {contract.EXPECTED_RUNNER_INDICES}, got {indices}"
        )
    return rows


def _live_config_snapshot(algo: Any, args: Any) -> dict[str, Any]:
    zero_live = zero_runner.live_config_snapshot(
        algo, expected_reward=contract.EXPECTED_REWARD_OVERRIDES
    )
    config = getattr(algo, "config", None)
    observed = {
        "num_env_runners": getattr(config, "num_env_runners", None),
        "train_batch_size": getattr(config, "train_batch_size", None),
        "minibatch_size": getattr(config, "minibatch_size", None),
        "num_epochs": getattr(config, "num_epochs", None),
        "lr": getattr(config, "lr", None),
        "gamma": getattr(config, "gamma", None),
        "lambda": getattr(config, "lambda_", None),
        "clip_param": getattr(config, "clip_param", None),
        "kl_coeff": getattr(config, "kl_coeff", None),
        "kl_target": getattr(config, "kl_target", None),
        "rollout_fragment_length": getattr(config, "rollout_fragment_length", None),
    }
    expected = {
        "num_env_runners": contract.EXPECTED_REMOTE_ENV_RUNNERS,
        "train_batch_size": contract.EXPECTED_TRAIN_BATCH_SIZE,
        "minibatch_size": contract.EXPECTED_MINIBATCH_SIZE,
        "num_epochs": contract.EXPECTED_NUM_EPOCHS,
        "lr": contract.EXPECTED_LR,
        "gamma": contract.EXPECTED_GAMMA,
        "lambda": contract.EXPECTED_LAMBDA,
        "clip_param": contract.EXPECTED_CLIP_PARAM,
        "kl_coeff": contract.EXPECTED_KL_COEFF,
        "kl_target": contract.EXPECTED_KL_TARGET,
        "rollout_fragment_length": contract.EXPECTED_ROLLOUT_FRAGMENT_LENGTH,
    }
    if not _same_json(observed, expected):
        raise TrainingReadinessError(
            f"live Algorithm production configuration drifted: {observed}"
        )
    sampling = getattr(args, "_start_sampling_contract", None)
    return {
        "algorithm": observed,
        "environment": zero_live,
        "exact_start_sampling": {
            "offsets_s": list(sampling.offsets_s),
            "runners_per_start": int(sampling.runners_per_start),
            "steps_per_start": int(sampling.expected_steps_per_start),
            "rollout_fragment_length": int(sampling.rollout_fragment_length),
        },
    }


def preflight_receipt_gate(payload: Mapping[str, Any]) -> dict[str, Any]:
    checks = payload.get("checks")
    activity = payload.get("activity")
    runners = payload.get("env_runners")
    progress = payload.get("progress")
    checks_mapping = dict(checks) if isinstance(checks, Mapping) else {}
    activity_mapping = dict(activity) if isinstance(activity, Mapping) else {}
    progress_mapping = dict(progress) if isinstance(progress, Mapping) else {}
    runner_rows = list(runners) if isinstance(runners, list) else []
    actor_surfaces = payload.get("actor_surfaces")
    actor_surfaces = (
        dict(actor_surfaces) if isinstance(actor_surfaces, Mapping) else {}
    )
    critic_surface = payload.get("critic_surface")
    critic_surface = (
        dict(critic_surface) if isinstance(critic_surface, Mapping) else {}
    )
    optimizer = payload.get("optimizer")
    optimizer = dict(optimizer) if isinstance(optimizer, Mapping) else {}
    gate = {
        "identity": payload.get("schema_version") == contract.SCHEMA_VERSION
        and payload.get("status") == contract.PREFLIGHT_PASS_STATUS
        and payload.get("passed") is True
        and payload.get("protocol_id") == contract.PROTOCOL_ID,
        "checks": set(checks_mapping) == set(contract.REQUIRED_PREFLIGHT_CHECKS)
        and all(value is True for value in checks_mapping.values()),
        "upstream": payload.get("morph_terminal_status")
        == contract.morph.PIPELINE_TERMINAL_PASS_STATUS
        and payload.get("zero_terminal_status") == contract.zero.PASS_STATUS,
        "exact_argv": payload.get("training_argv")
        == list(contract.final_training_argv("macos_arm64")),
        "source_closure": _same_json(
            payload.get("source_closure_before"), payload.get("source_closure_after")
        )
        and bool(payload.get("source_closure_before")),
        "actor_local_learner": bool(actor_surfaces)
        and _same_json(
            actor_surfaces.get("expected_checkpoint_zero"),
            actor_surfaces.get("local"),
        )
        and _same_json(
            actor_surfaces.get("expected_checkpoint_zero"),
            actor_surfaces.get("learner"),
        ),
        "critic": bool(critic_surface)
        and _same_json(
            critic_surface, payload.get("expected_checkpoint_zero_critic")
        ),
        "optimizer": bool(optimizer)
        and _same_json(
            optimizer, payload.get("expected_checkpoint_zero_optimizer")
        ),
        "activity": activity_mapping
        == {
            "algorithm_builds": 1,
            "checkpoint_restores": 1,
            "algorithm_update_calls": 0,
            "environment_sampling_calls": 0,
        },
        "runner_count": len(runner_rows) == contract.EXPECTED_ENV_RUNNER_SURFACES
        and tuple(row.get("worker_index") for row in runner_rows)
        == contract.EXPECTED_RUNNER_INDICES,
        "runner_actor_runtime": bool(runner_rows)
        and all(
            isinstance(row.get("actor"), Mapping)
            and _same_json(
                row.get("actor"), actor_surfaces.get("expected_checkpoint_zero")
            )
            and row.get("causal_runtime", {}).get("passed") is True
            and row.get("causal_runtime", {}).get("runtime_id")
            == contract.EXPECTED_CAUSAL_RUNTIME_ID
            for row in runner_rows
        ),
        "progress": progress_mapping.get("before_restore")
        == progress_mapping.get("after_restore")
        == progress_mapping.get("after_introspection")
        and progress_mapping.get("after_restore")
        == {name: 0 for name in contract.zero.ZERO_COUNTER_NAMES},
        "ray_shutdown": payload.get("ray_shutdown") is True,
        "driver_causal_runtime": payload.get("driver_causal_runtime", {}).get(
            "runtime_id"
        )
        == contract.EXPECTED_CAUSAL_RUNTIME_ID
        and payload.get("driver_causal_runtime", {}).get("corridor_installed")
        is True
        and payload.get("driver_causal_runtime", {}).get("reward_installed") is True,
        "training_not_executed": payload.get("training_executed") is False,
        "final_output_absent": payload.get("final_output_absent") is True,
    }
    return {
        "passed": all(gate.values()),
        "checks": gate,
        "failed_checks": sorted(name for name, passed in gate.items() if not passed),
    }


def execute_restore_preflight() -> dict[str, Any]:
    """Perform the one authorized exact-topology restore and publish its receipt."""

    working_directory = require_repository_root_cwd()
    if os.path.lexists(PREFLIGHT_RECEIPT):
        raise TrainingReadinessError(
            f"refusing to overwrite preflight receipt: {PREFLIGHT_RECEIPT}"
        )
    if os.path.lexists(FINAL_OUTPUT_DIR):
        raise TrainingReadinessError(
            f"final training output must be absent before preflight: {FINAL_OUTPUT_DIR}"
        )
    started = time.time()
    closure_before = source_closure_snapshot()
    morph_ledger = morph_runner.verify_terminal_ledger()
    zero_ledger = zero_runner.verify_terminal_pass()
    handoff = morph_runner.strict_mapping(MORPH_HANDOFF)
    if (
        handoff.get("status") != contract.morph.HANDOFF_PASS_STATUS
        or handoff.get("training_validation_endpoint")
        != contract.morph.TRAINING_VALIDATION_ENDPOINT
        or handoff.get("required_training_launcher")
        != contract.TRAINING_LAUNCHER.as_posix()
        or handoff.get("required_working_directory")
        != contract.REQUIRED_WORKING_DIRECTORY
    ):
        raise TrainingReadinessError("morphology training handoff drifted")
    zero_audit = zero_runner.strict_json(ZERO_AUDIT)
    expected_actor = copy.deepcopy(
        zero_audit["actor_surfaces"]["positive_restore_local"]
    )
    expected_critic = copy.deepcopy(
        zero_audit["critic_surfaces"]["after_positive_restore"]
    )
    expected_optimizer = copy.deepcopy(
        zero_audit["optimizer_surfaces"]["after_positive_restore"]
    )
    expected_progress = copy.deepcopy(
        zero_audit["progress_surfaces"]["after_positive_restore"]
    )
    train, args = parse_final_training_args()
    target_args = target_args_snapshot(args)

    import run_h0_v12r9_morphology_training as training_launcher
    import h0_v12r9_morphology_causal_runtime as causal_runtime
    import warm_start

    driver_runtime = training_launcher.configure_process()
    causal_runtime.assert_installed()
    train._load_training_stack()  # noqa: SLF001
    ray = train.ray
    if ray.is_initialized():
        raise TrainingReadinessError("Ray must be stopped before restore preflight")

    algo = None
    stop_completed = False
    with tempfile.TemporaryDirectory(prefix="h0_v12r9_restore_preflight_") as temp:
        temp_root = Path(temp)
        runtime_env = {
            "env_vars": {
                "KMP_DUPLICATE_LIB_OK": "TRUE",
                "PYTHONWARNINGS": "ignore",
                causal_runtime.SITE_MARKER_ENV: causal_runtime.RUNTIME_ID,
                "PYTHONPATH": os.environ.get("PYTHONPATH", ""),
            },
            "worker_process_setup_hook": train._worker_setup,  # noqa: SLF001
        }
        ray.init(
            include_dashboard=False,
            ignore_reinit_error=False,
            runtime_env=runtime_env,
            log_to_driver=False,
            num_cpus=contract.EXPECTED_RAY_NUM_CPUS,
            num_gpus=0,
        )
        try:
            from ray.tune.trainable import trainable as ray_trainable

            ray_trainable.DEFAULT_STORAGE_PATH = os.fspath(temp_root / "rllib")
            reward = dict(getattr(args, "_cfg_reward", None) or {})
            reward.update(_inline_reward_overrides(args))
            config = train.build_config(args, reward)
            algo = config.build_algo()
            progress_before = zero_runner.zero_progress_audit(algo)
            algo.restore_from_path(CHECKPOINT_ZERO)
            lr_reapply = train._reapply_optimizer_learning_rate(  # noqa: SLF001
                algo, contract.EXPECTED_LR
            )
            progress_after_restore = zero_runner.zero_progress_audit(algo)
            live_config = _live_config_snapshot(algo, args)

            local_state = algo.get_module(contract.zero.DEFAULT_POLICY_ID).get_state()
            learner_state = train._learner_module_state(algo)  # noqa: SLF001
            local_actor = _partition_surface(local_state, actor=True)
            learner_actor = _partition_surface(learner_state, actor=True)
            critic = _partition_surface(learner_state, actor=False)
            optimizer = zero_runner.optimizer_audit(train, algo)
            if not (
                _same_json(local_actor, expected_actor)
                and _same_json(learner_actor, expected_actor)
                and _same_json(critic, expected_critic)
                and _same_json(optimizer, expected_optimizer)
                and _same_json(progress_before, expected_progress)
                and _same_json(progress_after_restore, expected_progress)
            ):
                raise TrainingReadinessError(
                    "checkpoint-zero state differs on local/learner/critic/optimizer/progress"
                )
            env_runners = _env_runner_surfaces(
                algo=algo,
                expected_actor=expected_actor,
                warm_start=warm_start,
                timeout_s=float(args.startup_timeout_s),
            )
            progress_after_introspection = zero_runner.zero_progress_audit(algo)
            if not _same_json(progress_after_introspection, expected_progress):
                raise TrainingReadinessError(
                    "EnvRunner introspection changed zero progress counters"
                )
            if os.path.lexists(FINAL_OUTPUT_DIR):
                raise TrainingReadinessError(
                    "restore preflight unexpectedly created final training output"
                )
            algo.stop()
            algo = None
            stop_completed = True
        finally:
            if algo is not None:
                try:
                    algo.stop()
                finally:
                    algo = None
            if ray.is_initialized():
                ray.shutdown()
    ray_shutdown = not ray.is_initialized()
    if not stop_completed or not ray_shutdown:
        raise TrainingReadinessError("Algorithm/Ray shutdown did not complete")
    closure_after = source_closure_snapshot()
    if not _same_json(closure_after, closure_before):
        raise TrainingReadinessError("source closure changed during restore preflight")

    checks = {name: True for name in contract.REQUIRED_PREFLIGHT_CHECKS}
    payload = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PREFLIGHT_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "revision": contract.REVISION,
        "started_unix_s": started,
        "completed_unix_s": time.time(),
        "working_directory": working_directory,
        "checks": checks,
        "activity": {
            "algorithm_builds": 1,
            "checkpoint_restores": 1,
            "algorithm_update_calls": 0,
            "environment_sampling_calls": 0,
        },
        "morph_terminal_ledger": _artifact_record(
            resolve_relative(contract.MORPH_TERMINAL_ENDPOINT["path"])
        ),
        "zero_terminal_ledger": _artifact_record(
            resolve_relative(contract.ZERO_TERMINAL_ENDPOINT["path"])
        ),
        "morph_terminal_status": morph_ledger.get("status"),
        "zero_terminal_status": zero_ledger.get("status"),
        "morph_handoff": _artifact_record(MORPH_HANDOFF),
        "checkpoint_zero": zero_runner.tree_record(CHECKPOINT_ZERO),
        "zero_audit": _artifact_record(ZERO_AUDIT),
        "training_argv": list(contract.final_training_argv("macos_arm64")),
        "target_args": target_args,
        "live_config": live_config,
        "actor_surfaces": {
            "expected_checkpoint_zero": expected_actor,
            "local": local_actor,
            "learner": learner_actor,
        },
        "critic_surface": critic,
        "expected_checkpoint_zero_critic": expected_critic,
        "optimizer": optimizer,
        "expected_checkpoint_zero_optimizer": expected_optimizer,
        "optimizer_lr_reapply": lr_reapply,
        "progress": {
            "before_restore": progress_before,
            "after_restore": progress_after_restore,
            "after_introspection": progress_after_introspection,
        },
        "driver_causal_runtime": driver_runtime,
        "env_runners": env_runners,
        "source_closure_before": closure_before,
        "source_closure_after": closure_after,
        "final_output_absent": not os.path.lexists(FINAL_OUTPUT_DIR),
        "ray_shutdown": ray_shutdown,
        "training_executed": False,
        "next_stage": "AUTHORIZED_EXACT_50_UPDATE_TRAINING_COMMAND",
    }
    gate = preflight_receipt_gate(payload)
    if gate["passed"] is not True:
        raise TrainingReadinessError(
            f"preflight receipt self-gate failed: {gate['failed_checks']}"
        )
    try:
        PREFLIGHT_RECEIPT.parent.mkdir(parents=True, exist_ok=False)
    except OSError as exc:
        raise TrainingReadinessError(
            f"refusing existing/unsafe preflight publication root: "
            f"{PREFLIGHT_RECEIPT.parent}"
        ) from exc
    zero_runner.write_json_exclusive(PREFLIGHT_RECEIPT, payload)
    return payload


def _is_finite_number(value: Any) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def _optimizer_lrs_exact(value: Any) -> bool:
    rows: list[Mapping[str, Any]] = []

    def visit(node: Any) -> None:
        if isinstance(node, Mapping):
            if "learning_rate" in node:
                rows.append(node)
            for child in node.values():
                visit(child)
        elif isinstance(node, Sequence) and not isinstance(
            node, (str, bytes, bytearray)
        ):
            for child in node:
                visit(child)

    visit(value)
    return len(rows) == 1 and (
        rows[0].get("optimizer_name") == "default_optimizer"
        and rows[0].get("optimizer_type") == "Adam"
        and _is_finite_number(rows[0].get("learning_rate"))
        and float(rows[0]["learning_rate"]) == contract.EXPECTED_LR
    )


def _optimizer_lr_reapply_exact(value: Any) -> bool:
    rows: list[Mapping[str, Any]] = []

    def visit(node: Any) -> None:
        if isinstance(node, Mapping):
            if {"optimizer_name", "optimizer_type", "requested", "after"}.issubset(
                node
            ):
                rows.append(node)
            for child in node.values():
                visit(child)
        elif isinstance(node, Sequence) and not isinstance(
            node, (str, bytes, bytearray)
        ):
            for child in node:
                visit(child)

    visit(value)
    return len(rows) == 1 and (
        rows[0].get("optimizer_name") == "default_optimizer"
        and rows[0].get("optimizer_type") == "Adam"
        and rows[0].get("requested") == contract.EXPECTED_LR
        and rows[0].get("after") == contract.EXPECTED_LR
    )


def history_integrity_gate(rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    expected_iterations = list(range(1, contract.FINAL_TRAINING_ITERATIONS + 1))
    observed_iterations = [row.get("iteration") for row in rows]
    row_checks: list[dict[str, Any]] = []
    for expected_iteration, row in zip(expected_iterations, rows):
        balance = row.get("exact_start_balance")
        kl = row.get("kl_update_guard")
        balance = dict(balance) if isinstance(balance, Mapping) else {}
        kl = dict(kl) if isinstance(kl, Mapping) else {}
        learner_checks = balance.get("learner_checks")
        learner_checks = (
            dict(learner_checks) if isinstance(learner_checks, Mapping) else {}
        )
        expected_steps = {
            label: float(contract.EXPECTED_STEPS_PER_START)
            for label in contract.EXPECTED_START_LABELS
        }
        row_checks.append(
            {
                "iteration": row.get("iteration") == expected_iteration,
                "lifetime_steps": row.get("num_env_steps_sampled_lifetime")
                == float(expected_iteration * contract.EXPECTED_TRAIN_BATCH_SIZE),
                "critical_metrics_finite": all(
                    _is_finite_number(row.get(name))
                    for name in (
                        "episode_return_mean",
                        "episode_len_mean",
                        "policy_loss",
                        "vf_loss",
                        "entropy",
                        "mean_kl_loss",
                        "max_minibatch_mean_kl_loss",
                        "min_minibatch_mean_kl_loss",
                        "kl_minibatch_count",
                        "kl_nonfinite_count",
                        "current_kl_coeff",
                    )
                ),
                "exact_balance": balance.get("pass") is True
                and balance.get("learner_batch_pass") is True
                and balance.get("expected_steps") == expected_steps
                and balance.get("actual_steps") == expected_steps
                and balance.get("missing") == []
                and balance.get("unexpected") == []
                and balance.get("mismatched") == {}
                and balance.get("expected_real_steps")
                == float(contract.EXPECTED_TRAIN_BATCH_SIZE)
                and balance.get("learner_connector_steps_in")
                == float(contract.EXPECTED_TRAIN_BATCH_SIZE)
                and _is_finite_number(balance.get("learner_connector_steps_out"))
                and _is_finite_number(balance.get("pre_compaction_rows"))
                and _is_finite_number(balance.get("removed_compaction_rows"))
                and balance.get("learner_connector_steps_out")
                == balance.get("pre_compaction_rows")
                and balance.get("pre_compaction_rows")
                - balance.get("removed_compaction_rows")
                == float(contract.EXPECTED_TRAIN_BATCH_SIZE)
                and balance.get("removed_compaction_rows") > 0.0
                and balance.get("compacted_rows")
                == float(contract.EXPECTED_TRAIN_BATCH_SIZE)
                and balance.get("interleaved_rows")
                == float(contract.EXPECTED_TRAIN_BATCH_SIZE)
                and balance.get("interleaved_start_conditions") == 3.0
                and balance.get("interleaved_rows_per_start")
                == float(contract.EXPECTED_STEPS_PER_START)
                and balance.get("max_start_run_length")
                == float(contract.EXPECTED_MAX_START_RUN_LENGTH)
                and balance.get("expected_module_steps_trained")
                == float(contract.EXPECTED_TRAIN_BATCH_SIZE)
                and balance.get("module_steps_trained")
                == float(contract.EXPECTED_TRAIN_BATCH_SIZE)
                and balance.get("expected_kl_minibatches")
                == float(contract.EXPECTED_KL_MINIBATCH_COUNT)
                and balance.get("kl_minibatch_count")
                == float(contract.EXPECTED_KL_MINIBATCH_COUNT)
                and balance.get("kl_nonfinite_count") == 0.0
                and balance.get("advantage_counts") == expected_steps
                and balance.get("rollout_fragment_length")
                == contract.EXPECTED_ROLLOUT_FRAGMENT_LENGTH
                and balance.get("runners_per_start")
                == contract.EXPECTED_RUNNERS_PER_START
                and bool(learner_checks)
                and all(value is True for value in learner_checks.values()),
                "kl_guard": kl.get("enabled") is True
                and kl.get("pass") is True
                and kl.get("logical_iteration") == expected_iteration
                and kl.get("max_minibatch_mean_kl_loss_limit")
                == contract.EXPECTED_MAX_MINIBATCH_MEAN_KL_LOSS
                and kl.get("required_kl_nonfinite_count") == 0.0
                and kl.get("failed_checks") == []
                and isinstance(kl.get("metrics"), Mapping)
                and kl.get("metrics", {}).get("max_minibatch_mean_kl_loss")
                == row.get("max_minibatch_mean_kl_loss")
                and kl.get("metrics", {}).get("min_minibatch_mean_kl_loss")
                == row.get("min_minibatch_mean_kl_loss")
                and kl.get("metrics", {}).get("kl_minibatch_count")
                == float(contract.EXPECTED_KL_MINIBATCH_COUNT)
                and kl.get("metrics", {}).get("kl_nonfinite_count") == 0.0
                and row.get("kl_minibatch_count")
                == float(contract.EXPECTED_KL_MINIBATCH_COUNT)
                and row.get("kl_nonfinite_count") == 0.0
                and bool(kl.get("checks"))
                and all(
                    value is True
                    for value in (
                        kl.get("checks") or {}
                    ).values()
                ),
                "optimizer_lr": _optimizer_lrs_exact(
                    row.get("optimizer_learning_rates")
                ),
                "milestone_declared": isinstance(
                    row.get("iteration_milestone"), str
                ),
            }
        )
    aggregate = {
        "row_count": len(rows) == contract.FINAL_TRAINING_ITERATIONS,
        "iterations": observed_iterations == expected_iterations,
        "rows": len(row_checks) == contract.FINAL_TRAINING_ITERATIONS
        and all(all(check.values()) for check in row_checks),
    }
    return {
        "passed": all(aggregate.values()),
        "checks": aggregate,
        "failed_iterations": [
            index + 1
            for index, check in enumerate(row_checks)
            if not all(check.values())
        ],
        "row_checks": row_checks,
    }


def _resolved_config_gate(config: Mapping[str, Any]) -> dict[str, Any]:
    model = config.get("model") if isinstance(config.get("model"), Mapping) else {}
    ppo = config.get("ppo") if isinstance(config.get("ppo"), Mapping) else {}
    parallel = (
        config.get("parallelism")
        if isinstance(config.get("parallelism"), Mapping)
        else {}
    )
    simulation = (
        config.get("simulation")
        if isinstance(config.get("simulation"), Mapping)
        else {}
    )
    grf = config.get("grf") if isinstance(config.get("grf"), Mapping) else {}
    supervision = (
        config.get("supervision")
        if isinstance(config.get("supervision"), Mapping)
        else {}
    )
    reward = config.get("reward") if isinstance(config.get("reward"), Mapping) else {}
    checks = {
        "model": model.get("num_hidden_layers") == 2
        and model.get("dim_hidden_layers") == 512
        and model.get("rl_module_kind") == "standard"
        and model.get("asymmetric_actor_critic") is True
        and model.get("freeze_logstd") is True
        and model.get("freeze_actor") is False,
        "ppo": ppo.get("train_batch_size") == contract.EXPECTED_TRAIN_BATCH_SIZE
        and ppo.get("minibatch_size") == contract.EXPECTED_MINIBATCH_SIZE
        and ppo.get("num_epochs") == contract.EXPECTED_NUM_EPOCHS
        and ppo.get("lr") == contract.EXPECTED_LR
        and ppo.get("gamma") == contract.EXPECTED_GAMMA
        and ppo.get("lam") == contract.EXPECTED_LAMBDA
        and ppo.get("clip_param") == contract.EXPECTED_CLIP_PARAM
        and ppo.get("kl_coeff") == contract.EXPECTED_KL_COEFF
        and ppo.get("kl_target") == contract.EXPECTED_KL_TARGET,
        "parallelism": parallel.get("num_env_runners")
        == contract.EXPECTED_REMOTE_ENV_RUNNERS
        and parallel.get("ray_num_cpus") == contract.EXPECTED_RAY_NUM_CPUS
        and parallel.get("exact_start_sampling") is True,
        "simulation": simulation.get("iterations")
        == contract.FINAL_TRAINING_ITERATIONS
        and simulation.get("episode_start_offset_choices_s")
        == list(contract.EXPECTED_START_OFFSETS_S)
        and simulation.get("episode_start_offset_s")
        == contract.EXPECTED_START_OFFSETS_S[1]
        and simulation.get("random_init") is False,
        "detector": grf.get("phase_fsm_input_mode") == "legacy_events"
        and grf.get("event_contract_id") == contract.morph.q3.LEGACY_EVENT_CONTRACT_ID
        and grf.get("binary_phase_fsm_mode") == contract.morph.q3.V26_BINARY_MODE
        and grf.get("binary_phase_detector_profile")
        == contract.morph.q3.DETECTOR_PROFILE_PATH.as_posix()
        and grf.get("detector_sample_dt_s") == 0.001
        and grf.get("binary_phase_debounce_s") == 0.005
        and grf.get("binary_phase_event_contract_id")
        == contract.morph.q3.EVENT_CONTRACT_ID,
        "supervision": supervision.get("max_consecutive_skips") == 1
        and supervision.get("max_consecutive_crash_restarts") == 1
        and supervision.get("checkpoint_every") == 1
        and supervision.get("retain_iteration_checkpoints") is True
        and supervision.get("max_minibatch_mean_kl_loss")
        == contract.EXPECTED_MAX_MINIBATCH_MEAN_KL_LOSS,
        "reward": all(
            _same_config_value(reward.get(key), value)
            for key, value in contract.EXPECTED_REWARD_OVERRIDES.items()
        ),
    }
    return {"passed": all(checks.values()), "checks": checks}


def _expected_resolved_config() -> dict[str, Any]:
    """Reconstruct the complete snapshot from the frozen final argv."""

    import training_config

    _, args = parse_final_training_args()
    reward = dict(getattr(args, "_cfg_reward", None) or {})
    reward.update(_inline_reward_overrides(args))
    nested: dict[str, Any] = {}
    for section, spec in training_config.SECTION_MAP.items():
        if section in training_config._SNAPSHOT_SKIP_SECTIONS:  # noqa: SLF001
            continue
        section_values = {
            destination: training_config._yaml_safe(  # noqa: SLF001
                getattr(args, destination)
            )
            for destination in spec
            if hasattr(args, destination)
        }
        if section_values:
            nested[section] = section_values
    nested["reward"] = training_config._resolve_reward(reward)  # noqa: SLF001
    return nested


def _milestone_gate(run_dir: Path, rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    expected_names = [
        f"milestone_iteration_{iteration:06d}"
        for iteration in range(1, contract.FINAL_TRAINING_ITERATIONS + 1)
    ]
    observed_names = sorted(
        path.name
        for path in run_dir.glob("milestone_iteration_*")
        if path.is_dir()
    )
    invalid: list[int] = []
    for iteration, name in enumerate(expected_names, 1):
        root = run_dir / name
        checkpoint = root / "checkpoint_last"
        module = root / "rl_module_last"
        try:
            checkpoint_meta = strict_mapping(root / "checkpoint_last_meta.json")
            module_meta = strict_mapping(root / "rl_module_last_meta.json")
        except TrainingReadinessError:
            invalid.append(iteration)
            continue
        required_checkpoint = all(
            (checkpoint / suffix).is_file()
            for suffix in contract.zero.CHECKPOINT_REQUIRED_SUFFIXES
        )
        required_module = all(
            (module / suffix).is_file()
            for suffix in (
                "module_state.pkl",
                "metadata.json",
                "class_and_ctor_args.pkl",
            )
        )
        declared = (
            rows[iteration - 1].get("iteration_milestone")
            if iteration <= len(rows)
            else None
        )
        valid = (
            required_checkpoint
            and required_module
            and checkpoint_meta.get("logical_iteration") == iteration
            and checkpoint_meta.get("rllib_training_iteration") == iteration
            and _same_path(checkpoint_meta.get("checkpoint"), checkpoint)
            and module_meta.get("logical_iteration") == iteration
            and module_meta.get("rllib_training_iteration") == iteration
            and _same_path(module_meta.get("rl_module"), module)
            and _same_path(declared, root)
        )
        if not valid:
            invalid.append(iteration)
    return {
        "passed": observed_names == expected_names and not invalid,
        "observed_names": observed_names,
        "expected_names": expected_names,
        "invalid_iterations": invalid,
    }


def _logstd_gate(
    run_dir: Path,
    *,
    state_loader: Callable[[Path], Mapping[str, Any]] | None = None,
) -> dict[str, Any]:
    if state_loader is None:
        import warm_start

        state_loader = warm_start.load_module_state
    reference_state = state_loader(resolve_relative(contract.morph.MODULE_EXPORT_PATH))
    keys = ("pi.1.weight", "pi.1.bias")
    reference = {
        "pi.1.weight": _array(reference_state["pi.1.weight"])[2:],
        "pi.1.bias": _array(reference_state["pi.1.bias"])[2:],
    }
    invalid: list[int] = []
    digests: list[dict[str, Any]] = []
    for iteration in range(1, contract.FINAL_TRAINING_ITERATIONS + 1):
        state = state_loader(
            run_dir
            / f"milestone_iteration_{iteration:06d}"
            / "rl_module_last"
        )
        exact = all(
            key in state
            and _array(state[key])[2:].dtype == reference[key].dtype
            and _array(state[key])[2:].shape == reference[key].shape
            and _array(state[key])[2:].tobytes(order="C")
            == reference[key].tobytes(order="C")
            for key in keys
        )
        if not exact:
            invalid.append(iteration)
        digests.append(
            {
                "iteration": iteration,
                "logstd_weight_sha256": _tensor_digest(
                    _array(state["pi.1.weight"])[2:]
                ),
                "logstd_bias_sha256": _tensor_digest(
                    _array(state["pi.1.bias"])[2:]
                ),
            }
        )
    return {
        "passed": not invalid,
        "checked_iterations": contract.FINAL_TRAINING_ITERATIONS,
        "invalid_iterations": invalid,
        "digests": digests,
    }


def _critic_summary_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    rows = summary.get("critic_state_audit")
    if not isinstance(rows, list):
        rows = []
    expected_stages = ["before_training"] + [
        "after_iteration" for _ in range(contract.FINAL_TRAINING_ITERATIONS)
    ]
    expected_iterations = [None] + list(
        range(1, contract.FINAL_TRAINING_ITERATIONS + 1)
    )
    stages = [row.get("stage") if isinstance(row, Mapping) else None for row in rows]
    iterations = [
        row.get("iteration") if isinstance(row, Mapping) else None for row in rows
    ]
    digests = [
        row.get("critic_digest") if isinstance(row, Mapping) else None for row in rows
    ]
    valid_digests = all(
        isinstance(value, str)
        and len(value) == 64
        and all(char in "0123456789abcdef" for char in value)
        for value in digests
    )
    return {
        "passed": len(rows) == 51
        and stages == expected_stages
        and iterations == expected_iterations
        and valid_digests
        and len(set(digests)) > 1,
        "row_count": len(rows),
        "distinct_digest_count": len(set(digests)),
    }


def _summary_gate(
    summary: Mapping[str, Any],
    rows: Sequence[Mapping[str, Any]],
    run_dir: Path,
) -> dict[str, Any]:
    sampling = summary.get("exact_start_sampling_contract")
    sampling = dict(sampling) if isinstance(sampling, Mapping) else {}
    retention = summary.get("iteration_checkpoint_retention")
    retention = dict(retention) if isinstance(retention, Mapping) else {}
    expected_milestones = [
        str(run_dir / f"milestone_iteration_{iteration:06d}")
        for iteration in range(1, contract.FINAL_TRAINING_ITERATIONS + 1)
    ]
    reward = summary.get("reward_config")
    reward = dict(reward) if isinstance(reward, Mapping) else {}
    checks = {
        "completed": summary.get("ok") is True
        and summary.get("stop_reason") == "completed"
        and summary.get("interrupted") is False
        and summary.get("timed_out") is False
        and summary.get("error") is None,
        "updates": summary.get("iterations_run") == 50
        and summary.get("iterations_completed") == 50
        and summary.get("iterations_completed_this_process") == 50
        and summary.get("iteration_start") == 1
        and summary.get("next_iteration") == 51,
        "restore": summary.get("initialization_mode") == "resume_from"
        and _same_path(summary.get("resume_from"), CHECKPOINT_ZERO)
        and _same_path(summary.get("supervisor_resume_from"), CHECKPOINT_ZERO)
        and summary.get("restored_training_iteration") == 0
        and summary.get("restored_logical_iteration") == 0,
        "no_warm_start": summary.get("warm_start_requested") is False
        and summary.get("warm_start_applied") is False
        and summary.get("warm_start_h0_requested") is False
        and summary.get("warm_start_h0_applied") is False
        and summary.get("warm_start_raw_requested") is False
        and summary.get("warm_start_raw_applied") is False
        and summary.get("warm_start_raw_transplant_applied_this_process") is False
        and summary.get("warm_start_report") is None
        and summary.get("warm_start") is None
        and summary.get("warm_start_mode") is None,
        "model": summary.get("freeze_logstd") is True
        and summary.get("freeze_actor") is False
        and summary.get("rl_module_kind") == "standard",
        "topology": summary.get("num_env_runners") == 12
        and summary.get("ray_num_cpus") == 13
        and summary.get("exact_start_sampling") is True
        and sampling
        == {
            "offsets_s": list(contract.EXPECTED_START_OFFSETS_S),
            "rollout_fragment_length": contract.EXPECTED_ROLLOUT_FRAGMENT_LENGTH,
            "expected_steps_per_start": contract.EXPECTED_STEPS_PER_START,
            "runners_per_start": contract.EXPECTED_RUNNERS_PER_START,
        },
        "ppo": summary.get("num_epochs") == contract.EXPECTED_NUM_EPOCHS
        and summary.get("lr") == contract.EXPECTED_LR
        and summary.get("clip_param") == contract.EXPECTED_CLIP_PARAM
        and summary.get("kl_coeff") == contract.EXPECTED_KL_COEFF
        and summary.get("kl_target") == contract.EXPECTED_KL_TARGET,
        "detector": summary.get("phase_fsm_input_mode") == "legacy_events"
        and summary.get("event_contract_id")
        == contract.morph.q3.LEGACY_EVENT_CONTRACT_ID
        and summary.get("binary_phase_fsm_mode")
        == contract.morph.q3.V26_BINARY_MODE
        and summary.get("binary_phase_detector_profile_file")
        == contract.morph.q3.DETECTOR_PROFILE_PATH.as_posix()
        and summary.get("detector_sample_dt_s") == 0.001
        and summary.get("binary_phase_debounce_s") == 0.005
        and summary.get("binary_phase_event_contract_id")
        == contract.morph.q3.EVENT_CONTRACT_ID,
        "reward": all(
            _same_config_value(reward.get(key), value)
            for key, value in contract.EXPECTED_REWARD_OVERRIDES.items()
        ),
        "supervision": summary.get("max_consecutive_skips") == 1
        and summary.get("max_consecutive_crash_restarts") == 1,
        "no_restart": summary.get("restart_count") == 0
        and summary.get("crash_restart_count") == 0
        and summary.get("crash_restarts") == []
        and summary.get("skipped_iterations") == [],
        "history_exact": _same_json(summary.get("history"), list(rows)),
        "retention": retention.get("enabled") is True
        and retention.get("milestones") == expected_milestones,
        "optimizer_restore_lr": (
            isinstance(summary.get("optimizer_lr_audit"), list)
            and len(summary.get("optimizer_lr_audit")) == 1
            and summary["optimizer_lr_audit"][0].get("stage") == "after_restore"
            and _optimizer_lr_reapply_exact(
                summary["optimizer_lr_audit"][0].get("learners")
            )
        ),
    }
    return {"passed": all(checks.values()), "checks": checks}


def _final_checkpoint_gate(run_dir: Path) -> dict[str, Any]:
    checkpoint = run_dir / "checkpoint_last"
    module = run_dir / "rl_module_last"
    try:
        checkpoint_meta = strict_mapping(run_dir / "checkpoint_last_meta.json")
    except TrainingReadinessError:
        return {"passed": False}
    def same_tree_content(left: Path, right: Path) -> bool:
        first = zero_runner.tree_record(left)
        second = zero_runner.tree_record(right)
        return (
            first.get("tree_sha256") == second.get("tree_sha256")
            and first.get("file_count") == second.get("file_count")
            and first.get("files") == second.get("files")
        )

    return {
        "passed": all(
            (checkpoint / suffix).is_file()
            for suffix in contract.zero.CHECKPOINT_REQUIRED_SUFFIXES
        )
        and all(
            (module / suffix).is_file()
            for suffix in (
                "module_state.pkl",
                "metadata.json",
                "class_and_ctor_args.pkl",
            )
        )
        and checkpoint_meta.get("logical_iteration") == 50
        and checkpoint_meta.get("rllib_training_iteration") == 50
        and _same_path(checkpoint_meta.get("checkpoint"), checkpoint)
        and same_tree_content(
            checkpoint,
            run_dir / "milestone_iteration_000050" / "checkpoint_last",
        )
        and same_tree_content(
            module,
            run_dir / "milestone_iteration_000050" / "rl_module_last",
        )
    }


def _profile_attestations_gate() -> dict[str, Any]:
    observed: dict[str, Any] = {}
    checks: dict[str, bool] = {}
    for name, expected in sorted(contract.morph.PROFILE_ATTESTATIONS.items()):
        record = _artifact_record(resolve_relative(expected["path"]))
        observed[name] = record
        checks[name] = (
            record.get("path") == expected["path"]
            and record.get("sha256") == expected["sha256"]
        )
    return {"passed": all(checks.values()), "checks": checks, "observed": observed}


def build_postrun_audit(
    run_dir: str | Path,
    ray_log_dir: str | Path,
    *,
    restart_auditor: Callable[..., Mapping[str, Any]] | None = None,
    state_loader: Callable[[Path], Mapping[str, Any]] | None = None,
) -> dict[str, Any]:
    require_repository_root_cwd()
    run_root = Path(run_dir).expanduser().resolve(strict=False)
    ray_logs = Path(ray_log_dir).expanduser().resolve(strict=False)
    preflight = strict_mapping(PREFLIGHT_RECEIPT)
    preflight_gate = preflight_receipt_gate(preflight)
    summary = strict_mapping(run_root / "summary.json")
    supervisor = strict_mapping(run_root / "supervisor_state.json")
    watchdog = strict_mapping(run_root / "watchdog_state.json")
    rows = strict_jsonl(run_root / "train_iterations.jsonl")
    import training_config

    resolved = training_config.load(run_root / training_config.RESOLVED_CONFIG_NAME)
    expected_resolved = _expected_resolved_config()
    history_gate = history_integrity_gate(rows)
    summary_gate = _summary_gate(summary, rows, run_root)
    config_gate = _resolved_config_gate(resolved)
    milestone_gate = _milestone_gate(run_root, rows)
    logstd_gate = _logstd_gate(run_root, state_loader=state_loader)
    critic_gate = _critic_summary_gate(summary)
    final_checkpoint_gate = _final_checkpoint_gate(run_root)
    profiles_gate = _profile_attestations_gate()
    current_closure = source_closure_snapshot()
    current_checkpoint_zero = zero_runner.tree_record(CHECKPOINT_ZERO)
    closure_gate = {
        "passed": _same_json(
            current_closure, preflight.get("source_closure_after")
        )
        and preflight.get("morph_handoff") == _artifact_record(MORPH_HANDOFF)
        and preflight.get("zero_audit") == _artifact_record(ZERO_AUDIT)
        and _same_json(
            preflight.get("checkpoint_zero"), current_checkpoint_zero
        )
    }
    supervisor_gate = {
        "passed": supervisor.get("status") == "completed"
        and supervisor.get("restart_count") == 0
        and supervisor.get("skipped_iterations") == []
        and supervisor.get("consecutive_skips") == 0
        and supervisor.get("crash_restart_count") == 0
        and supervisor.get("consecutive_crash_restarts") == 0
        and watchdog.get("phase") == "complete"
        and watchdog.get("timeout_s") == 0.0
    }
    if restart_auditor is None:
        from validation import audit_training_restarts

        restart_auditor = audit_training_restarts.audit_training_restarts
    driver_pid = watchdog.get("pid")
    restart = dict(restart_auditor(run_root, ray_logs, driver_pid=driver_pid))
    restart_gate = {
        "passed": restart.get("ok") is True
        and restart.get("status") == "PASS"
        and restart.get("failed_checks") == []
        and restart.get("contract", {}).get("expected_num_env_runners") == 12
        and restart.get("iteration_history", {}).get("rows") == 50
        and restart.get("ray_driver_logs", {}).get("resolution") == "pid"
        and restart.get("ray_driver_logs", {}).get("findings") == []
        and restart.get("ray_driver_logs", {}).get("read_errors") == []
        and restart.get("ray_driver_logs", {}).get("env_runner_lifecycle_lines", 0)
        > 0
    }
    checks = {
        "preflight_receipt_pass": preflight_gate["passed"] is True,
        "canonical_run_directory": run_root == FINAL_OUTPUT_DIR.resolve(strict=False),
        "summary_completed_exact_50": summary_gate["checks"].get("completed")
        is True
        and summary_gate["checks"].get("updates") is True,
        "resume_from_checkpoint_zero": summary_gate["checks"].get("restore") is True,
        "no_warm_start_interface": summary_gate["checks"].get("no_warm_start")
        is True,
        "model_and_runtime_exact": summary_gate["checks"].get("model") is True
        and summary_gate["checks"].get("topology") is True
        and summary_gate["checks"].get("ppo") is True
        and summary_gate["checks"].get("detector") is True
        and profiles_gate["passed"] is True
        and closure_gate["passed"] is True,
        "positive_reward_exact": summary_gate["checks"].get("reward") is True
        and _same_json(summary.get("reward_config"), resolved.get("reward")),
        "resolved_config_exact": config_gate["passed"] is True
        and _same_json(resolved, expected_resolved),
        "history_exact_50_unique_updates": history_gate["checks"].get("row_count")
        is True
        and history_gate["checks"].get("iterations") is True
        and summary_gate["checks"].get("history_exact") is True,
        "lifetime_steps_exact": history_gate["passed"] is True,
        "exact_start_balance_every_update": history_gate["passed"] is True,
        "kl_guard_every_update": history_gate["passed"] is True,
        "optimizer_lr_every_update": history_gate["passed"] is True
        and summary_gate["checks"].get("optimizer_restore_lr") is True,
        "critic_audit_complete": critic_gate["passed"] is True,
        "logstd_frozen_across_all_milestones": logstd_gate["passed"] is True,
        "all_50_milestones_complete": milestone_gate["passed"] is True
        and summary_gate["checks"].get("retention") is True,
        "final_checkpoint_complete": final_checkpoint_gate["passed"] is True,
        "no_supervisor_restart_or_skip": supervisor_gate["passed"] is True
        and summary_gate["checks"].get("no_restart") is True
        and summary_gate["checks"].get("supervision") is True,
        "no_hidden_env_runner_restart": restart_gate["passed"] is True,
    }
    passed = set(checks) == set(contract.REQUIRED_POSTRUN_CHECKS) and all(
        checks.values()
    )
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.POSTRUN_PASS_STATUS if passed else "FAIL_H0_V12R9_TRAINING_50_UPDATE_INTEGRITY",
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "run_dir": str(run_root),
        "ray_log_dir": str(ray_logs),
        "checks": checks,
        "failed_checks": sorted(name for name, ok in checks.items() if not ok),
        "preflight_receipt": _artifact_record(PREFLIGHT_RECEIPT),
        "summary": summary_gate,
        "resolved_config": config_gate,
        "expected_resolved_config_sha256": hashlib.sha256(
            json.dumps(
                expected_resolved,
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            ).encode("utf-8")
        ).hexdigest(),
        "history": history_gate,
        "critic": critic_gate,
        "logstd": logstd_gate,
        "milestones": milestone_gate,
        "final_checkpoint": final_checkpoint_gate,
        "supervisor": supervisor_gate,
        "restart_audit": restart,
        "watchdog": watchdog,
        "profile_attestations": profiles_gate,
        "source_closure": closure_gate,
        "training_updates": contract.FINAL_TRAINING_ITERATIONS,
        "environment_steps": contract.EXPECTED_LIFETIME_ENV_STEPS,
        "next_stage": (
            "AUTHORIZED_DOWNSTREAM_POLICY_QUALIFICATION"
            if passed
            else contract.POSTRUN_FAILURE_MODE
        ),
    }


def postrun_integrity_gate(payload: Mapping[str, Any]) -> dict[str, Any]:
    checks = payload.get("checks")
    checks = dict(checks) if isinstance(checks, Mapping) else {}
    gate = {
        "identity": payload.get("schema_version") == contract.SCHEMA_VERSION
        and payload.get("status") == contract.POSTRUN_PASS_STATUS
        and payload.get("passed") is True
        and payload.get("protocol_id") == contract.PROTOCOL_ID,
        "checks": set(checks) == set(contract.REQUIRED_POSTRUN_CHECKS)
        and all(value is True for value in checks.values()),
        "updates": payload.get("training_updates")
        == contract.FINAL_TRAINING_ITERATIONS,
        "steps": payload.get("environment_steps")
        == contract.EXPECTED_LIFETIME_ENV_STEPS,
        "no_failures": payload.get("failed_checks") == [],
    }
    return {
        "passed": all(gate.values()),
        "checks": gate,
        "failed_checks": sorted(name for name, value in gate.items() if not value),
    }


def execute_postrun_audit(run_dir: str | Path, ray_log_dir: str | Path) -> dict[str, Any]:
    require_repository_root_cwd()
    if os.path.lexists(POSTRUN_AUDIT):
        raise TrainingReadinessError(
            f"refusing to overwrite post-run audit: {POSTRUN_AUDIT}"
        )
    payload = build_postrun_audit(run_dir, ray_log_dir)
    zero_runner.write_json_exclusive(POSTRUN_AUDIT, payload)
    return payload


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_mutually_exclusive_group(required=True)
    actions.add_argument("--print-plan", action="store_true")
    actions.add_argument("--source-check", action="store_true")
    actions.add_argument("--restore-preflight", action="store_true")
    actions.add_argument("--post-run-audit", action="store_true")
    parser.add_argument(
        "--run-dir",
        default=contract.FINAL_OUTPUT_DIR.as_posix(),
        help="Completed run directory; canonical default is immutable.",
    )
    parser.add_argument(
        "--ray-log-dir",
        default=None,
        help="Exact Ray session logs directory for the training worker.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.print_plan:
        payload = build_preflight_plan()
    elif args.source_check:
        payload = source_check()
    elif args.restore_preflight:
        payload = execute_restore_preflight()
    else:
        if not args.ray_log_dir:
            raise SystemExit("--post-run-audit requires --ray-log-dir")
        payload = execute_postrun_audit(args.run_dir, args.ray_log_dir)
    print(json.dumps(payload, indent=2, sort_keys=False, allow_nan=False))
    return 0 if payload.get("passed", True) is True else 1


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "TrainingReadinessError",
    "build_postrun_audit",
    "build_preflight_plan",
    "execute_postrun_audit",
    "execute_restore_preflight",
    "history_integrity_gate",
    "main",
    "parse_args",
    "parse_final_training_args",
    "postrun_integrity_gate",
    "preflight_receipt_gate",
    "require_repository_root_cwd",
    "source_check",
    "strict_json_any",
    "strict_jsonl",
    "strict_mapping",
    "target_args_snapshot",
]
