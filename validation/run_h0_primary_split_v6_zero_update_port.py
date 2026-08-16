"""Fail-closed zero-update trainer port for the qualified V6 residual P1.

No call to ``Algorithm.train`` exists in this driver.  After a separately
frozen execution lock binds one P1 across development and qualification, the
runtime builds a fresh PPO Algorithm with the explicit V25 residual RLModule,
copies the complete inference policy (base actor, residual buffers/network and
Gaussian logstd), retains a fresh critic and empty optimizer, saves at progress
zero, restores into another fresh Algorithm, and repeats every audit.
"""

from __future__ import annotations

import hashlib
import json
import math
import numbers
import os
import pickle
import platform
import sys
import tempfile
import time
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
BASELINE_ROOT = TRAJECTORY_ROOT / "baseline_MLP"
for import_root in (VALIDATION_ROOT, BASELINE_ROOT, TRAJECTORY_ROOT, REPO_ROOT):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

import h0_primary_split_v6_zero_update_contract as contract  # noqa: E402


def _resolve_relative(relative: str) -> Path:
    pure = PurePosixPath(relative)
    if not relative or pure.is_absolute() or ".." in pure.parts:
        raise RuntimeError(f"non-canonical contract path: {relative!r}")
    return REPO_ROOT.joinpath(*pure.parts)


LOCK = _resolve_relative(contract.LOCK_PATH.as_posix())
OUTPUT_ROOT = _resolve_relative(contract.OUTPUT_ROOT.as_posix())
INPUT_PATHS = {
    key: _resolve_relative(value) for key, value in contract.INPUT_PATHS.items()
}
SOURCE_PATHS = {
    key: _resolve_relative(value) for key, value in contract.SOURCE_PATHS.items()
}
CANDIDATE_DIR = INPUT_PATHS["candidate_module_state"].parent


class ZeroUpdatePortError(RuntimeError):
    """Raised when any prerequisite or zero-update invariant is unproven."""


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).expanduser().resolve().open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def repo_relative(path: str | Path) -> str:
    resolved = Path(path).expanduser().resolve()
    try:
        return resolved.relative_to(REPO_ROOT.resolve()).as_posix()
    except ValueError as exc:
        raise ZeroUpdatePortError(f"path is outside repository: {resolved}") from exc


def source_record(path: str | Path) -> dict[str, Any]:
    resolved = Path(path).expanduser().resolve()
    if not resolved.is_file() or resolved.is_symlink():
        raise ZeroUpdatePortError(f"required regular file is missing: {resolved}")
    return {
        "path": repo_relative(resolved),
        "sha256": sha256_file(resolved),
        "size_bytes": resolved.stat().st_size,
    }


def _require_finite_json(value: Any, label: str = "payload") -> None:
    if value is None or type(value) in {bool, str, int}:
        return
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ZeroUpdatePortError(f"{label} contains a non-finite number")
        return
    if isinstance(value, Mapping):
        for key, child in value.items():
            if not isinstance(key, str) or not key:
                raise ZeroUpdatePortError(f"{label} contains an invalid key")
            _require_finite_json(child, f"{label}.{key}")
        return
    if isinstance(value, list):
        for index, child in enumerate(value):
            _require_finite_json(child, f"{label}[{index}]")
        return
    raise ZeroUpdatePortError(f"{label} contains a non-JSON value")


def canonical_json(value: Any) -> bytes:
    _require_finite_json(value)
    try:
        return json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise ZeroUpdatePortError("payload is not strict JSON") from exc


def strict_json(path: str | Path) -> dict[str, Any]:
    resolved = Path(path).expanduser().resolve()

    def reject_duplicates(pairs: Sequence[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"duplicate JSON key: {key}")
            result[key] = value
        return result

    try:
        value = json.loads(
            resolved.read_text(encoding="utf-8"),
            parse_constant=lambda token: (_ for _ in ()).throw(
                ValueError(f"non-finite JSON token: {token}")
            ),
            object_pairs_hook=reject_duplicates,
        )
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        raise ZeroUpdatePortError(f"invalid strict JSON: {resolved}") from exc
    if not isinstance(value, Mapping):
        raise ZeroUpdatePortError(f"expected JSON object: {resolved}")
    _require_finite_json(value, str(resolved))
    return dict(value)


def write_json_exclusive(path: str | Path, payload: Mapping[str, Any]) -> Path:
    destination = Path(path).expanduser().resolve()
    canonical_json(payload)
    destination.parent.mkdir(parents=True, exist_ok=True)
    descriptor, raw_temporary = tempfile.mkstemp(
        prefix=f".{destination.name}.", suffix=".tmp", dir=destination.parent
    )
    temporary = Path(raw_temporary)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            json.dump(
                dict(payload),
                stream,
                indent=2,
                sort_keys=True,
                ensure_ascii=False,
                allow_nan=False,
            )
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        flags = os.O_CREAT | os.O_EXCL | os.O_WRONLY | getattr(os, "O_BINARY", 0)
        try:
            claimed = os.open(str(destination), flags, 0o600)
        except FileExistsError as exc:
            raise ZeroUpdatePortError(f"refusing to clobber: {destination}") from exc
        os.close(claimed)
        os.replace(temporary, destination)
    finally:
        if temporary.exists():
            temporary.unlink()
    return destination


def tree_record(root: str | Path) -> dict[str, Any]:
    directory = Path(root).expanduser().resolve()
    if not directory.is_dir() or directory.is_symlink():
        raise ZeroUpdatePortError(f"artifact tree is missing: {directory}")
    files = {}
    for path in sorted(directory.rglob("*")):
        if path.is_symlink():
            raise ZeroUpdatePortError(f"artifact tree contains a symlink: {path}")
        if path.is_file():
            files[path.relative_to(directory).as_posix()] = source_record(path)
    if not files:
        raise ZeroUpdatePortError(f"artifact tree is empty: {directory}")
    return {"root": repo_relative(directory), "files": files}


def validate_residual_config(value: Any) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise ZeroUpdatePortError("residual_model_config must be an object")
    expected_keys = {
        "input_mean",
        "input_std",
        "limits",
        "init_seed",
        "input_indices",
        "architecture",
    }
    if set(value) != expected_keys:
        raise ZeroUpdatePortError("residual_model_config schema drifted")

    def vector(name: str, length: int, *, positive: bool = False) -> list[float]:
        raw = value[name]
        if not isinstance(raw, list) or len(raw) != length:
            raise ZeroUpdatePortError(f"{name} must contain {length} values")
        result = []
        for item in raw:
            if isinstance(item, bool) or not isinstance(item, numbers.Real):
                raise ZeroUpdatePortError(f"{name} contains a non-number")
            number = float(item)
            if not math.isfinite(number) or (positive and number <= 0.0):
                raise ZeroUpdatePortError(f"{name} contains an invalid value")
            result.append(number)
        return result

    mean = vector("input_mean", contract.RESIDUAL_INPUT_COUNT)
    std = vector("input_std", contract.RESIDUAL_INPUT_COUNT, positive=True)
    limits = vector("limits", contract.RESIDUAL_LIMIT_COUNT, positive=True)
    seed = value["init_seed"]
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise ZeroUpdatePortError("init_seed must be a non-negative integer")
    indices = value["input_indices"]
    architecture = value["architecture"]
    if indices != list(contract.RESIDUAL_INPUT_INDICES):
        raise ZeroUpdatePortError("residual input indices drifted")
    if architecture != list(contract.RESIDUAL_ARCHITECTURE):
        raise ZeroUpdatePortError("residual architecture drifted")
    if limits != list(contract.RESIDUAL_LIMITS):
        raise ZeroUpdatePortError("residual limits drifted")
    if seed != contract.RESIDUAL_INIT_SEED:
        raise ZeroUpdatePortError("residual initialization seed drifted")
    return {
        "input_mean": mean,
        "input_std": std,
        "limits": limits,
        "init_seed": seed,
        "input_indices": list(indices),
        "architecture": list(architecture),
    }


def verify_lock() -> dict[str, Any]:
    lock = strict_json(LOCK)
    expected_scalars = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.LOCK_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "source_protocol_id": contract.SOURCE_PROTOCOL_ID,
        "qualification_protocol_id": contract.QUALIFICATION_PROTOCOL_ID,
        "revision": contract.REVISION,
        "so_policy_id": contract.SO_POLICY_ID,
        "target_bundle_contract_id": contract.TARGET_BUNDLE_CONTRACT_ID,
        "output_root": repo_relative(OUTPUT_ROOT),
        "target_fixed_config": contract.TARGET_FIXED_CONFIG,
        "authority": contract.AUTHORITY,
        "required_checks": list(contract.REQUIRED_CHECKS),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "environment_samples": 0,
        "protected_trials_opened": [],
    }
    for key, expected in expected_scalars.items():
        if canonical_json(lock.get(key)) != canonical_json(expected):
            raise ZeroUpdatePortError(f"execution lock drifted at {key}")
    candidate_id = lock.get("candidate_id")
    if not isinstance(candidate_id, str) or not candidate_id:
        raise ZeroUpdatePortError("execution lock has no candidate_id")
    validate_residual_config(lock.get("residual_model_config"))
    for field, paths in (("sources", SOURCE_PATHS), ("inputs", INPUT_PATHS)):
        records = lock.get(field)
        if not isinstance(records, Mapping) or set(records) != set(paths):
            raise ZeroUpdatePortError(f"execution lock {field} schema drifted")
        for name, path in paths.items():
            if canonical_json(records[name]) != canonical_json(source_record(path)):
                raise ZeroUpdatePortError(f"execution lock {field}.{name} drifted")
    return lock


def load_module_state(module_dir: str | Path) -> dict[str, Any]:
    path = Path(module_dir).expanduser().resolve() / "module_state.pkl"
    with path.open("rb") as stream:
        value = pickle.load(stream)
    if not isinstance(value, Mapping):
        raise ZeroUpdatePortError(f"module state is not a mapping: {path}")
    return dict(value)


def _critic_key(key: str) -> bool:
    return key.startswith("vf.") or key.startswith("vf_encoder.")


def policy_state(state: Mapping[str, Any]) -> dict[str, Any]:
    return {key: value for key, value in state.items() if not _critic_key(key)}


def critic_state(state: Mapping[str, Any]) -> dict[str, Any]:
    return {key: value for key, value in state.items() if _critic_key(key)}


def _value_bytes(value: Any) -> bytes:
    if hasattr(value, "detach"):
        array = value.detach().cpu().contiguous().numpy()
        return (
            str(array.dtype).encode()
            + repr(tuple(array.shape)).encode()
            + array.tobytes(order="C")
        )
    if hasattr(value, "tobytes") and hasattr(value, "shape"):
        return (
            str(value.dtype).encode()
            + repr(tuple(value.shape)).encode()
            + value.tobytes(order="C")
        )
    return pickle.dumps(value, protocol=4)


def state_digest(state: Mapping[str, Any]) -> str:
    digest = hashlib.sha256()
    for key in sorted(state):
        digest.update(key.encode("utf-8"))
        digest.update(b"\0")
        digest.update(_value_bytes(state[key]))
        digest.update(b"\0")
    return digest.hexdigest()


def compare_states(
    expected: Mapping[str, Any], actual: Mapping[str, Any]
) -> dict[str, Any]:
    expected_keys = set(expected)
    actual_keys = set(actual)
    shared = sorted(expected_keys & actual_keys)
    mismatched = [
        key
        for key in shared
        if _value_bytes(expected[key]) != _value_bytes(actual[key])
    ]
    missing = sorted(expected_keys - actual_keys)
    unexpected = sorted(actual_keys - expected_keys)
    exact = not mismatched and not missing and not unexpected
    return {
        "exact": exact,
        "expected_digest": state_digest(expected),
        "actual_digest": state_digest(actual),
        "key_count": len(expected_keys),
        "mismatched_keys": mismatched,
        "missing_keys": missing,
        "unexpected_keys": unexpected,
    }


def transplant_policy_state(
    *, target_state: Mapping[str, Any], candidate_state: Mapping[str, Any]
) -> tuple[dict[str, Any], dict[str, Any]]:
    candidate_policy = policy_state(candidate_state)
    target_policy = policy_state(target_state)
    candidate_critic = critic_state(candidate_state)
    if candidate_critic:
        raise ZeroUpdatePortError("qualified P1 export unexpectedly contains a critic")
    if set(candidate_policy) != set(target_policy):
        report = compare_states(candidate_policy, target_policy)
        raise ZeroUpdatePortError(f"candidate/target policy schema differs: {report}")
    residual_keys = sorted(
        key for key in candidate_policy if key.startswith("primary_split_v25_residual")
    )
    base_actor_keys = sorted(key for key in candidate_policy if key.startswith("pi"))
    if not residual_keys or not base_actor_keys:
        raise ZeroUpdatePortError("candidate lacks base actor or residual policy state")
    merged = dict(target_state)
    merged.update(candidate_policy)
    copied = compare_states(candidate_policy, policy_state(merged))
    critic_preserved = compare_states(critic_state(target_state), critic_state(merged))
    if not copied["exact"] or not critic_preserved["exact"]:
        raise ZeroUpdatePortError(
            "policy transplant was not exact and critic-preserving"
        )
    return merged, {
        "policy": copied,
        "critic_preserved": critic_preserved,
        "base_actor_key_count": len(base_actor_keys),
        "residual_key_count": len(residual_keys),
        "logstd_container": "pi final-layer state copied byte-exact",
    }


def _as_zero(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise ZeroUpdatePortError(f"{label} is not numeric")
    number = float(value)
    if not math.isfinite(number) or number != 0.0:
        raise ZeroUpdatePortError(f"{label} is not zero")
    return 0


def zero_progress_audit(algo: Any) -> dict[str, int]:
    from ray.rllib.utils import metrics as ray_metrics

    names = {
        "num_env_steps_sampled_lifetime": ray_metrics.NUM_ENV_STEPS_SAMPLED_LIFETIME,
        "num_agent_steps_sampled_lifetime": ray_metrics.NUM_AGENT_STEPS_SAMPLED_LIFETIME,
        "num_env_steps_trained_lifetime": ray_metrics.NUM_ENV_STEPS_TRAINED_LIFETIME,
        "num_agent_steps_trained_lifetime": ray_metrics.NUM_AGENT_STEPS_TRAINED_LIFETIME,
        "num_grad_updates_lifetime": ray_metrics.NUM_GRAD_UPDATES_LIFETIME,
    }
    logger = getattr(algo, "metrics", None)
    if logger is None or not callable(getattr(logger, "peek", None)):
        raise ZeroUpdatePortError("Algorithm metrics logger is unavailable")
    result = {
        "training_iteration": _as_zero(getattr(algo, "iteration", None), "iteration")
    }
    result.update(
        {
            label: _as_zero(logger.peek(metric, default=0.0), label)
            for label, metric in names.items()
        }
    )
    if tuple(result) != contract.ZERO_COUNTER_NAMES:
        raise ZeroUpdatePortError("zero progress schema drifted")
    return result


def _learner_module(learner: Any) -> Any:
    collection = getattr(learner, "module", None)
    if collection is not None:
        try:
            return collection[contract.DEFAULT_POLICY_ID]
        except (KeyError, TypeError):
            pass
    getter = getattr(learner, "get_module", None)
    if callable(getter):
        return getter(contract.DEFAULT_POLICY_ID)
    raise ZeroUpdatePortError("learner module is unavailable")


def optimizer_empty_and_residual_registered_on_learner(learner: Any) -> dict[str, Any]:
    module = _learner_module(learner)
    residual = {
        name: parameter
        for name, parameter in module.named_parameters()
        if name.startswith("primary_split_v25_residual")
    }
    if not residual:
        raise ZeroUpdatePortError("learner has no residual parameters")
    optimizer_rows = []
    optimized_ids: set[int] = set()
    for name, optimizer in learner.get_optimizers_for_module(
        contract.DEFAULT_POLICY_ID
    ):
        state = optimizer.state_dict()
        entries = state.get("state") if isinstance(state, Mapping) else None
        if not isinstance(entries, Mapping) or entries:
            raise ZeroUpdatePortError(f"optimizer {name} is unavailable or non-empty")
        groups = getattr(optimizer, "param_groups", None)
        if not isinstance(groups, list):
            raise ZeroUpdatePortError(f"optimizer {name} param groups are unavailable")
        for group in groups:
            for parameter in group.get("params", []):
                optimized_ids.add(id(parameter))
        optimizer_rows.append(
            {
                "name": str(name),
                "type": type(optimizer).__name__,
                "state_entry_count": 0,
                "param_group_count": len(groups),
            }
        )
    if not optimizer_rows:
        raise ZeroUpdatePortError("target learner has no optimizer")
    missing = sorted(
        name for name, value in residual.items() if id(value) not in optimized_ids
    )
    if missing:
        raise ZeroUpdatePortError(
            f"residual parameters missing from optimizer: {missing}"
        )
    return {
        "optimizers": optimizer_rows,
        "residual_parameter_names": sorted(residual),
        "residual_parameter_count": len(residual),
        "all_residual_parameters_registered": True,
        "optimizer_state_empty": True,
    }


def optimizer_audit(train: Any, algo: Any) -> list[dict[str, Any]]:
    reports = train._learner_call_results(  # noqa: SLF001
        algo, optimizer_empty_and_residual_registered_on_learner
    )
    if not reports:
        raise ZeroUpdatePortError("optimizer audit produced no learner evidence")
    return reports


def _target_training_args(train: Any, lock: Mapping[str, Any], output: Path) -> Any:
    residual = validate_residual_config(lock["residual_model_config"])
    argv = [
        str(train.__file__),
        "--config",
        str(INPUT_PATHS["source_training_config"]),
        "--output-dir",
        str(output),
        "--iterations",
        "0",
        "--num-env-runners",
        "0",
        "--ray-num-cpus",
        "1",
        "--no-tensorboard",
        "--no-progress",
        "--no-update-history",
        "--no-exact-start-sampling",
        "--asymmetric-actor-critic",
        "--rl-module-kind",
        contract.RL_MODULE_KIND,
        "--freeze-logstd",
        "--no-freeze-actor",
        "--phase-fsm-input-mode",
        "legacy_events",
        "--event-contract-id",
        contract.EVENT_CONTRACT_ID,
        "--binary-phase-fsm-mode",
        "binary_active",
        "--binary-phase-event-contract-id",
        contract.BINARY_EVENT_CONTRACT_ID,
        "--binary-phase-detector-profile",
        str(INPUT_PATHS["v25_binary_profile"]),
        "--online-grf-detector-profile",
        str(INPUT_PATHS["legacy_analog_detector_profile"]),
        "--primary-split-v25-residual-input-mean",
        *(format(value, ".17g") for value in residual["input_mean"]),
        "--primary-split-v25-residual-input-std",
        *(format(value, ".17g") for value in residual["input_std"]),
        "--primary-split-v25-residual-limits",
        *(format(value, ".17g") for value in residual["limits"]),
        "--primary-split-v25-residual-init-seed",
        str(residual["init_seed"]),
    ]
    previous = list(sys.argv)
    try:
        sys.argv = argv
        args = train.parse_args()
    finally:
        sys.argv = previous
    train._validate_rl_module_args(args)  # noqa: SLF001
    observed = {key: getattr(args, key) for key in contract.TARGET_FIXED_CONFIG}
    if canonical_json(observed) != canonical_json(contract.TARGET_FIXED_CONFIG):
        raise ZeroUpdatePortError(f"target fixed config drifted: {observed}")
    if args.primary_split_v25_residual_input_mean != residual["input_mean"]:
        raise ZeroUpdatePortError("serialized residual mean drifted")
    if args.primary_split_v25_residual_input_std != residual["input_std"]:
        raise ZeroUpdatePortError("serialized residual std drifted")
    if args.primary_split_v25_residual_limits != residual["limits"]:
        raise ZeroUpdatePortError("serialized residual limits drifted")
    if args.primary_split_v25_residual_init_seed != residual["init_seed"]:
        raise ZeroUpdatePortError("serialized residual seed drifted")
    reward = dict(getattr(args, "_cfg_reward", None) or {})
    if float(reward.get("morphology_weight", math.nan)) != contract.MORPHOLOGY_WEIGHT:
        raise ZeroUpdatePortError("zero-update target morphology_weight is not zero")
    return args


def audit_resolved_config(
    path: str | Path, residual: Mapping[str, Any]
) -> dict[str, Any]:
    """Re-read the emitted snapshot and prove all P1 constructor data survived."""

    import training_config

    loaded = training_config.load(path)
    flat, reward = training_config.to_argparse_defaults(loaded)
    expected = {
        "rl_module_kind": contract.RL_MODULE_KIND,
        "primary_split_v25_residual_input_mean": list(residual["input_mean"]),
        "primary_split_v25_residual_input_std": list(residual["input_std"]),
        "primary_split_v25_residual_limits": list(residual["limits"]),
        "primary_split_v25_residual_init_seed": int(residual["init_seed"]),
    }
    observed = {key: flat.get(key) for key in expected}
    if canonical_json(observed) != canonical_json(expected):
        raise ZeroUpdatePortError(f"resolved residual constructor drifted: {observed}")
    morphology = reward.get("morphology_weight")
    if (
        isinstance(morphology, bool)
        or not isinstance(morphology, numbers.Real)
        or float(morphology) != contract.MORPHOLOGY_WEIGHT
    ):
        raise ZeroUpdatePortError("resolved morphology_weight is not zero")
    return {
        "residual_constructor": observed,
        "morphology_weight": float(morphology),
        "snapshot": source_record(path),
    }


def _policy_surfaces(
    algo: Any, expected: Mapping[str, Any], train: Any, timeout: float
) -> dict[str, Any]:
    from ray.rllib.algorithms.algorithm import COMPONENT_RL_MODULE

    local = compare_states(
        expected, policy_state(algo.get_module(contract.DEFAULT_POLICY_ID).get_state())
    )
    learner = compare_states(expected, policy_state(train._learner_module_state(algo)))  # noqa: SLF001
    if not local["exact"] or not learner["exact"]:
        raise ZeroUpdatePortError("local or learner policy differs from qualified P1")
    runner_states = algo.env_runner_group.foreach_env_runner(
        func=lambda runner: runner.get_state(
            components=[COMPONENT_RL_MODULE], inference_only=True
        ),
        local_env_runner=True,
        timeout_seconds=float(timeout),
    )
    runners = []
    for index, wrapped in enumerate(runner_states):
        state = _find_policy_state(wrapped)
        report = compare_states(expected, policy_state(state))
        if not report["exact"]:
            raise ZeroUpdatePortError(f"EnvRunner {index} policy differs from P1")
        runners.append(report)
    if not runners:
        raise ZeroUpdatePortError("no EnvRunner policy was audited")
    return {"local": local, "learner": learner, "env_runners": runners}


def _find_policy_state(value: Any) -> Mapping[str, Any]:
    if isinstance(value, Mapping):
        if any(str(key).startswith("pi") for key in value):
            return value
        for child in value.values():
            try:
                return _find_policy_state(child)
            except ZeroUpdatePortError:
                pass
    raise ZeroUpdatePortError("nested state contains no policy tensors")


def _export_policy(
    algo: Any, path: Path, expected: Mapping[str, Any]
) -> dict[str, Any]:
    if path.exists():
        raise ZeroUpdatePortError(f"refusing to clobber export: {path}")
    algo.get_module(contract.DEFAULT_POLICY_ID).save_to_path(path.resolve())
    exported = policy_state(load_module_state(path))
    report = compare_states(expected, exported)
    if not report["exact"]:
        raise ZeroUpdatePortError("exported policy differs from qualified P1")
    return {"comparison": report, "tree": tree_record(path)}


_CHECKPOINT_FILES = {
    "algorithm_state.pkl",
    "class_and_ctor_args.pkl",
    "rllib_checkpoint.json",
    "learner_group/learner/state.pkl",
    "learner_group/learner/rl_module/module_state.pkl",
    "env_runner/state.pkl",
}


def _full_checkpoint(path: Path) -> dict[str, Any]:
    tree = tree_record(path)
    missing = sorted(_CHECKPOINT_FILES - set(tree["files"]))
    if missing:
        raise ZeroUpdatePortError(f"full checkpoint is incomplete: {missing}")
    return tree


def _runtime_port(lock: Mapping[str, Any]) -> dict[str, Any]:
    import training_config
    import train_ppo_mlp as train

    train._load_training_stack()  # noqa: SLF001
    ray = train.ray
    args = _target_training_args(train, lock, OUTPUT_ROOT / "rllib")
    reward = dict(getattr(args, "_cfg_reward", None) or {})
    resolved_path = training_config.dump_resolved(
        args, reward, OUTPUT_ROOT / contract.OUTPUT_NAMES["resolved_config"]
    )
    resolved_config = audit_resolved_config(
        resolved_path,
        validate_residual_config(lock["residual_model_config"]),
    )
    ray.init(
        include_dashboard=False,
        ignore_reinit_error=False,
        runtime_env={
            "env_vars": {"KMP_DUPLICATE_LIB_OK": "TRUE", "PYTHONWARNINGS": "ignore"}
        },
        log_to_driver=False,
        num_cpus=1,
        num_gpus=0,
    )
    first = None
    restored = None
    try:
        from primary_split_v25_residual import PrimarySplitV25ResidualTorchRLModule
        from ray.rllib.algorithms.algorithm import (
            COMPONENT_LEARNER,
            COMPONENT_LEARNER_GROUP,
            COMPONENT_RL_MODULE,
        )
        from ray.tune.trainable import trainable as ray_trainable

        rllib_root = OUTPUT_ROOT / "rllib"
        rllib_root.mkdir(parents=True, exist_ok=False)
        ray_trainable.DEFAULT_STORAGE_PATH = str(rllib_root)
        candidate = load_module_state(CANDIDATE_DIR)
        candidate_policy = policy_state(candidate)

        first = train.build_config(args, reward).build_algo()
        if not isinstance(
            first.get_module(contract.DEFAULT_POLICY_ID),
            PrimarySplitV25ResidualTorchRLModule,
        ):
            raise ZeroUpdatePortError(
                "fresh Algorithm did not build the residual module"
            )
        progress_before = zero_progress_audit(first)
        optimizer_before = optimizer_audit(train, first)
        if (
            len(tuple(getattr(args, "_target_actor_feature_names", ())))
            != contract.EXPECTED_ACTOR_FEATURES
        ):
            raise ZeroUpdatePortError("target actor layout is not 35")
        if int(getattr(args, "_target_n_full", -1)) != contract.EXPECTED_FULL_FEATURES:
            raise ZeroUpdatePortError("target observation layout is not 84")

        local_before = first.get_module(contract.DEFAULT_POLICY_ID).get_state()
        learner_before = train._learner_module_state(first)  # noqa: SLF001
        if not critic_state(learner_before):
            raise ZeroUpdatePortError("fresh learner critic is missing")
        local_merged, local_copy = transplant_policy_state(
            target_state=local_before, candidate_state=candidate
        )
        learner_merged, learner_copy = transplant_policy_state(
            target_state=learner_before, candidate_state=candidate
        )
        first.get_module(contract.DEFAULT_POLICY_ID).set_state(local_merged)
        first.set_state(
            {
                COMPONENT_LEARNER_GROUP: {
                    COMPONENT_LEARNER: {
                        COMPONENT_RL_MODULE: {
                            contract.DEFAULT_POLICY_ID: learner_merged
                        }
                    }
                }
            }
        )
        first.env_runner_group.sync_weights(
            from_worker_or_learner_group=first.learner_group,
            timeout_seconds=float(args.startup_timeout_s),
            inference_only=True,
        )
        learner_after = train._learner_module_state(first)  # noqa: SLF001
        fresh_critic = compare_states(
            critic_state(learner_before), critic_state(learner_after)
        )
        if not fresh_critic["exact"]:
            raise ZeroUpdatePortError("policy transplant changed the fresh critic")
        surfaces_before = _policy_surfaces(
            first, candidate_policy, train, args.startup_timeout_s
        )
        optimizer_after = optimizer_audit(train, first)
        progress_after = zero_progress_audit(first)
        initial_export = _export_policy(
            first,
            OUTPUT_ROOT / contract.OUTPUT_NAMES["initial_export"],
            candidate_policy,
        )

        checkpoint = OUTPUT_ROOT / contract.OUTPUT_NAMES["checkpoint"]
        if checkpoint.exists():
            raise ZeroUpdatePortError(f"refusing to clobber checkpoint: {checkpoint}")
        first.save_to_path(checkpoint)
        checkpoint_tree = _full_checkpoint(checkpoint)
        if zero_progress_audit(first) != progress_before:
            raise ZeroUpdatePortError("checkpoint save changed progress")
        first.stop()
        first = None

        restored = train.build_config(args, reward).build_algo()
        if any(zero_progress_audit(restored).values()):
            raise ZeroUpdatePortError("second fresh Algorithm has non-zero progress")
        restored.restore_from_path(checkpoint)
        restored_progress = zero_progress_audit(restored)
        restored_optimizer = optimizer_audit(train, restored)
        restored_learner = train._learner_module_state(restored)  # noqa: SLF001
        restored_critic = compare_states(
            critic_state(learner_after), critic_state(restored_learner)
        )
        if not restored_critic["exact"]:
            raise ZeroUpdatePortError("restored fresh critic differs")
        restored_surfaces = _policy_surfaces(
            restored, candidate_policy, train, args.startup_timeout_s
        )
        restored_export = _export_policy(
            restored,
            OUTPUT_ROOT / contract.OUTPUT_NAMES["restored_export"],
            candidate_policy,
        )
        if any(restored_progress.values()):
            raise ZeroUpdatePortError("restored Algorithm has non-zero progress")

        return {
            "candidate_id": lock["candidate_id"],
            "candidate_policy_digest": state_digest(candidate_policy),
            "checks": {name: True for name in contract.REQUIRED_CHECKS},
            "target_actor_feature_names": list(args._target_actor_feature_names),
            "target_observation_feature_names": list(
                args._target_observation_feature_names
            ),
            "resolved_config": resolved_config,
            "local_policy_transplant": local_copy,
            "learner_policy_transplant": learner_copy,
            "fresh_critic": fresh_critic,
            "surfaces_before_save": surfaces_before,
            "surfaces_after_restore": restored_surfaces,
            "optimizer": {
                "before_transplant": optimizer_before,
                "after_transplant": optimizer_after,
                "after_restore": restored_optimizer,
            },
            "progress": {
                "before_transplant": progress_before,
                "after_transplant": progress_after,
                "after_restore": restored_progress,
            },
            "checkpoint": checkpoint_tree,
            "initial_export": initial_export,
            "restored_export": restored_export,
        }
    finally:
        for algo in (restored, first):
            if algo is not None:
                try:
                    algo.stop()
                except Exception:
                    pass
        if ray.is_initialized():
            ray.shutdown()


def execute() -> dict[str, Any]:
    lock = verify_lock()
    if os.path.lexists(OUTPUT_ROOT):
        raise ZeroUpdatePortError(
            f"zero-update output is already claimed: {OUTPUT_ROOT}"
        )
    OUTPUT_ROOT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_ROOT.mkdir()
    claim = write_json_exclusive(
        OUTPUT_ROOT / contract.OUTPUT_NAMES["attempt_claim"],
        {
            "schema_version": contract.SCHEMA_VERSION,
            "status": "H0_PRIMARY_SPLIT_V6_ZERO_UPDATE_ATTEMPT_CLAIMED",
            "protocol_id": contract.PROTOCOL_ID,
            "candidate_id": lock["candidate_id"],
            "execution_lock": source_record(LOCK),
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
            "environment_samples": 0,
            "protected_trials_opened": [],
        },
    )
    started = time.time()
    passed = False
    error = None
    runtime = None
    try:
        runtime = _runtime_port(lock)
        if set(runtime["checks"]) != set(contract.REQUIRED_CHECKS) or any(
            value is not True for value in runtime["checks"].values()
        ):
            raise ZeroUpdatePortError("runtime checks are incomplete")
        audit_path = write_json_exclusive(
            OUTPUT_ROOT / contract.OUTPUT_NAMES["audit"],
            {
                "schema_version": contract.SCHEMA_VERSION,
                "status": contract.PASS_STATUS,
                "passed": True,
                "protocol_id": contract.PROTOCOL_ID,
                **runtime,
                "actor_updates": 0,
                "critic_updates": 0,
                "ppo_updates": 0,
                "environment_samples": 0,
                "protected_trials_opened": [],
            },
        )
        receipt = {
            "schema_version": contract.SCHEMA_VERSION,
            "status": contract.PASS_STATUS,
            "passed": True,
            "protocol_id": contract.PROTOCOL_ID,
            "candidate_id": lock["candidate_id"],
            "candidate_policy_digest": runtime["candidate_policy_digest"],
            "execution_lock": source_record(LOCK),
            "attempt_claim": source_record(claim),
            "qualification_receipt": source_record(
                INPUT_PATHS["qualification_receipt"]
            ),
            "audit": source_record(audit_path),
            "checkpoint": tree_record(
                OUTPUT_ROOT / contract.OUTPUT_NAMES["checkpoint"]
            ),
            "initial_export": tree_record(
                OUTPUT_ROOT / contract.OUTPUT_NAMES["initial_export"]
            ),
            "restored_export": tree_record(
                OUTPUT_ROOT / contract.OUTPUT_NAMES["restored_export"]
            ),
            "platform": {
                "system": platform.system(),
                "machine": platform.machine(),
                "python": platform.python_version(),
            },
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
            "environment_samples": 0,
            "protected_trials_opened": [],
        }
        write_json_exclusive(OUTPUT_ROOT / contract.OUTPUT_NAMES["receipt"], receipt)
        passed = True
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
        receipt = {}
    ledger = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PASS_STATUS if passed else contract.FAIL_STATUS,
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": lock["candidate_id"],
        "started_unix_s": started,
        "completed_unix_s": time.time(),
        "error": error,
        "execution_lock": source_record(LOCK),
        "attempt_claim": source_record(claim),
        "receipt_created": (OUTPUT_ROOT / contract.OUTPUT_NAMES["receipt"]).is_file(),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "environment_samples": 0,
        "protected_trials_opened": [],
        "next_stage": "H0_TRAINING_READY_GATES" if passed else "STOP_WITHOUT_RETRY",
    }
    write_json_exclusive(OUTPUT_ROOT / contract.OUTPUT_NAMES["ledger"], ledger)
    if not passed:
        raise ZeroUpdatePortError(error or contract.FAIL_STATUS)
    return receipt


if __name__ == "__main__":
    try:
        print(json.dumps(execute(), indent=2, sort_keys=True, allow_nan=False))
    except Exception as exc:
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
        raise SystemExit(2) from exc
