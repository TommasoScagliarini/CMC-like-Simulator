"""One-shot, zero-update trainer port for the qualified H0 V5 actor.

The driver builds a fresh target Algorithm from the frozen V5 configuration,
copies only the qualified actor, and never calls ``Algorithm.train``.  It then
saves a complete iteration-zero checkpoint, restores that checkpoint into a
second fresh Algorithm, and re-audits actor, critic, optimizer, and progress
counters.  Execution is fail-closed and no-clobber.
"""

from __future__ import annotations

import hashlib
import json
import math
import numbers
import os
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

import h0_primary_grf_split_v5_zero_update_contract as contract  # noqa: E402


def _resolve_relative(relative: str) -> Path:
    pure = PurePosixPath(relative)
    if (
        not relative
        or pure.is_absolute()
        or ".." in pure.parts
        or pure.as_posix() != relative
    ):
        raise RuntimeError(f"non-canonical contract path: {relative!r}")
    return REPO_ROOT.joinpath(*pure.parts)


LOCK = _resolve_relative(contract.LOCK_RELATIVE_PATH)
OUTPUT_ROOT = _resolve_relative(contract.OUTPUT_ROOT_RELATIVE_PATH)
INPUT_PATHS = {
    key: _resolve_relative(value)
    for key, value in contract.INPUT_RELATIVE_PATHS.items()
}
SOURCE_PATHS = {
    key: _resolve_relative(value)
    for key, value in contract.SOURCE_RELATIVE_PATHS.items()
}
CANDIDATE_DIR = INPUT_PATHS["candidate_module_state"].parent

LOCK_KEYS = frozenset(
    {
        "schema_version",
        "status",
        "protocol_id",
        "source_protocol_id",
        "revision",
        "candidate_id",
        "so_policy_id",
        "event_contract_id",
        "output_root",
        "target_config_overrides",
        "authority",
        "sources",
        "inputs",
        "required_checks",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
        "environment_samples",
        "protected_trials_opened",
    }
)

REQUIRED_CHECKS = (
    "qualified_candidate_frozen",
    "fresh_target_algorithm",
    "candidate_actor_exact_before_save",
    "learner_actor_exact_before_save",
    "env_runner_actor_exact_before_save",
    "export_actor_exact_before_save",
    "export_manifest_exact_before_save",
    "fresh_critic_created_without_source_restore",
    "critic_unchanged_by_actor_transplant",
    "optimizer_empty_before_save",
    "zero_progress_before_save",
    "full_checkpoint_saved_at_zero",
    "restored_learner_actor_exact",
    "restored_env_runner_actor_exact",
    "restored_export_actor_exact",
    "restored_export_manifest_exact",
    "restored_critic_exact",
    "restored_optimizer_empty",
    "zero_progress_after_restore",
    "no_train_call",
    "no_actor_update",
    "no_critic_update",
    "no_ppo_update",
    "no_environment_sample",
    "no_protected_access",
)

_CHECKPOINT_REQUIRED_SUFFIXES = frozenset(
    {
        "algorithm_state.pkl",
        "class_and_ctor_args.pkl",
        "rllib_checkpoint.json",
        "learner_group/learner/state.pkl",
        "learner_group/learner/rl_module/module_state.pkl",
        "env_runner/state.pkl",
    }
)


class ZeroUpdatePortError(RuntimeError):
    """Raised whenever the zero-update contract cannot be proven exactly."""


def _sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).expanduser().resolve().open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _repo_relative(path: str | Path) -> str:
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
        "path": _repo_relative(resolved),
        "sha256": _sha256_file(resolved),
        "size_bytes": resolved.stat().st_size,
    }


def tree_record(root: str | Path) -> dict[str, Any]:
    directory = Path(root).expanduser().resolve()
    if not directory.is_dir() or directory.is_symlink():
        raise ZeroUpdatePortError(f"artifact tree is missing: {directory}")
    files: dict[str, Any] = {}
    for path in sorted(directory.rglob("*")):
        if path.is_symlink():
            raise ZeroUpdatePortError(f"artifact tree contains a symlink: {path}")
        if path.is_file():
            relative = path.relative_to(directory).as_posix()
            files[relative] = source_record(path)
    if not files:
        raise ZeroUpdatePortError(f"artifact tree is empty: {directory}")
    return {"root": _repo_relative(directory), "files": files}


def _strict_json(path: str | Path) -> dict[str, Any]:
    resolved = Path(path).expanduser().resolve()

    def reject_duplicate_keys(pairs: Sequence[tuple[str, Any]]) -> dict[str, Any]:
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
                ValueError(f"non-finite JSON token {token}")
            ),
            object_pairs_hook=reject_duplicate_keys,
        )
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        raise ZeroUpdatePortError(f"invalid strict JSON: {resolved}") from exc
    if not isinstance(value, Mapping):
        raise ZeroUpdatePortError(f"expected JSON object: {resolved}")
    _require_finite_json(value, str(resolved))
    return dict(value)


def _require_finite_json(value: Any, label: str) -> None:
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


def _canonical_json_bytes(value: Any) -> bytes:
    try:
        return json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise ZeroUpdatePortError("payload is not strict finite JSON") from exc


def _publish_temporary_exclusive(temporary: Path, destination: Path) -> None:
    flags = os.O_CREAT | os.O_EXCL | os.O_WRONLY | getattr(os, "O_BINARY", 0)
    try:
        descriptor = os.open(str(destination), flags, 0o600)
    except FileExistsError as exc:
        raise ZeroUpdatePortError(f"refusing to clobber: {destination}") from exc
    else:
        os.close(descriptor)
    os.replace(str(temporary), str(destination))


def write_json_exclusive(path: str | Path, payload: Mapping[str, Any]) -> Path:
    destination = Path(path).expanduser().resolve()
    _require_finite_json(payload, "JSON payload")
    _canonical_json_bytes(payload)
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
        _publish_temporary_exclusive(temporary, destination)
    finally:
        if temporary.exists():
            temporary.unlink()
    return destination


def _verified_record(record: Any, expected: Path, label: str) -> Path:
    if not isinstance(record, Mapping) or set(record) != {
        "path",
        "sha256",
        "size_bytes",
    }:
        raise ZeroUpdatePortError(f"{label} is not an exact record")
    observed = source_record(expected)
    if _canonical_json_bytes(record) != _canonical_json_bytes(observed):
        raise ZeroUpdatePortError(f"{label} identity drifted")
    return expected.resolve()


def verify_lock() -> dict[str, Any]:
    lock = _strict_json(LOCK)
    if set(lock) != LOCK_KEYS:
        raise ZeroUpdatePortError("zero-update lock schema drifted")
    exact = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.LOCK_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "source_protocol_id": contract.SOURCE_PROTOCOL_ID,
        "revision": contract.REVISION,
        "candidate_id": contract.CANDIDATE_ID,
        "so_policy_id": contract.SO_POLICY_ID,
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "output_root": _repo_relative(OUTPUT_ROOT),
        "target_config_overrides": contract.TARGET_CONFIG_OVERRIDES,
        "authority": contract.AUTHORITY,
        "required_checks": list(REQUIRED_CHECKS),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "environment_samples": 0,
        "protected_trials_opened": [],
    }
    for key, expected in exact.items():
        if _canonical_json_bytes(lock.get(key)) != _canonical_json_bytes(expected):
            raise ZeroUpdatePortError(f"zero-update lock drifted at {key}")
    for field, paths in (("sources", SOURCE_PATHS), ("inputs", INPUT_PATHS)):
        records = lock.get(field)
        if not isinstance(records, Mapping) or set(records) != set(paths):
            raise ZeroUpdatePortError(f"zero-update lock {field} schema drifted")
        for key, path in paths.items():
            _verified_record(records[key], path, f"lock.{field}.{key}")
    return lock


def _target_training_args(train_module: Any, output_dir: Path) -> Any:
    argv = [
        str(train_module.__file__),
        "--config",
        str(INPUT_PATHS["v5_training_config"]),
        "--output-dir",
        str(output_dir),
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
        "--phase-fsm-input-mode",
        "legacy_events",
        "--event-contract-id",
        contract.EVENT_CONTRACT_ID,
        "--binary-phase-fsm-mode",
        "disabled",
    ]
    previous = list(sys.argv)
    try:
        sys.argv = argv
        args = train_module.parse_args()
    finally:
        sys.argv = previous
    observed = {
        key: getattr(args, key) for key in contract.TARGET_CONFIG_OVERRIDES
    }
    if _canonical_json_bytes(observed) != _canonical_json_bytes(
        contract.TARGET_CONFIG_OVERRIDES
    ):
        raise ZeroUpdatePortError(f"target config overrides drifted: {observed}")
    if not bool(args.asymmetric_actor_critic):
        raise ZeroUpdatePortError("V5 target must use asymmetric actor/critic")
    return args


def _as_zero_integer(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise ZeroUpdatePortError(f"{label} is not numeric")
    number = float(value)
    if not math.isfinite(number) or not number.is_integer() or number != 0.0:
        raise ZeroUpdatePortError(f"{label} is not zero: {value!r}")
    return 0


def zero_progress_audit(algo: Any) -> dict[str, int]:
    from ray.rllib.utils import metrics as ray_metrics

    metric_names = {
        "num_env_steps_sampled_lifetime": ray_metrics.NUM_ENV_STEPS_SAMPLED_LIFETIME,
        "num_agent_steps_sampled_lifetime": (
            ray_metrics.NUM_AGENT_STEPS_SAMPLED_LIFETIME
        ),
        "num_env_steps_trained_lifetime": ray_metrics.NUM_ENV_STEPS_TRAINED_LIFETIME,
        "num_agent_steps_trained_lifetime": (
            ray_metrics.NUM_AGENT_STEPS_TRAINED_LIFETIME
        ),
        "num_grad_updates_lifetime": ray_metrics.NUM_GRAD_UPDATES_LIFETIME,
    }
    values = {
        "training_iteration": _as_zero_integer(
            getattr(algo, "iteration", None), "training_iteration"
        )
    }
    logger = getattr(algo, "metrics", None)
    if logger is None or not callable(getattr(logger, "peek", None)):
        raise ZeroUpdatePortError("Algorithm metrics logger is unavailable")
    for label, metric_key in metric_names.items():
        values[label] = _as_zero_integer(
            logger.peek(metric_key, default=0.0), label
        )
    if tuple(values) != contract.ZERO_COUNTER_NAMES:
        raise ZeroUpdatePortError("zero progress counter schema drifted")
    return values


def _optimizer_empty_on_learner(learner: Any) -> list[dict[str, Any]]:
    reports: list[dict[str, Any]] = []
    for optimizer_name, optimizer in learner.get_optimizers_for_module(
        contract.DEFAULT_POLICY_ID
    ):
        state = optimizer.state_dict()
        if not isinstance(state, Mapping) or not isinstance(
            state.get("state"), Mapping
        ):
            raise ZeroUpdatePortError("optimizer state_dict schema is unavailable")
        state_entries = len(state["state"])
        report = {
            "optimizer_name": str(optimizer_name),
            "optimizer_type": type(optimizer).__name__,
            "state_entry_count": state_entries,
            "param_group_count": len(state.get("param_groups", [])),
            "empty": state_entries == 0,
        }
        if not report["empty"]:
            raise ZeroUpdatePortError(
                f"optimizer {optimizer_name} contains restored/updated state"
            )
        reports.append(report)
    if not reports:
        raise ZeroUpdatePortError("no optimizer registered for target learner")
    return reports


def optimizer_empty_audit(train_module: Any, algo: Any) -> list[Any]:
    reports = train_module._learner_call_results(  # noqa: SLF001
        algo, _optimizer_empty_on_learner
    )
    if not reports or any(not group for group in reports):
        raise ZeroUpdatePortError("optimizer audit returned no learner evidence")
    return reports


def fresh_critic_audit(
    *,
    candidate_state: Mapping[str, Any],
    learner_state: Mapping[str, Any],
    warm_start: Any,
) -> dict[str, Any]:
    """Prove that an actor-only source did not contribute a target critic."""

    candidate_non_actor = warm_start.compare_non_actor_states(
        candidate_state, candidate_state
    )
    fresh_learner_critic = warm_start.compare_non_actor_states(
        learner_state, learner_state
    )
    source_vs_fresh_boundary = warm_start.compare_non_actor_states(
        candidate_state, learner_state
    )
    if (
        candidate_non_actor["keys"]
        or not fresh_learner_critic["keys"]
        or not fresh_learner_critic["exact"]
        or source_vs_fresh_boundary["keys"]
        or source_vs_fresh_boundary["missing_keys"]
        or source_vs_fresh_boundary["unexpected_keys"]
        != fresh_learner_critic["keys"]
        or source_vs_fresh_boundary["exact"]
    ):
        raise ZeroUpdatePortError(
            "target critic was not freshly created outside the actor-only source"
        )
    return {
        "candidate_actor_only": candidate_non_actor,
        "fresh_learner": fresh_learner_critic,
        "source_vs_fresh_boundary": source_vs_fresh_boundary,
    }


def _actor_surfaces(
    *,
    algo: Any,
    expected_state: Mapping[str, Any],
    train_module: Any,
    warm_start: Any,
    timeout_s: float,
) -> dict[str, Any]:
    from ray.rllib.algorithms.algorithm import COMPONENT_RL_MODULE

    local_state = algo.get_module(contract.DEFAULT_POLICY_ID).get_state()
    learner_state = train_module._learner_module_state(algo)  # noqa: SLF001
    local = warm_start.compare_actor_states(expected_state, local_state)
    learner = warm_start.compare_actor_states(expected_state, learner_state)
    if not local["exact"] or not learner["exact"]:
        raise ZeroUpdatePortError("candidate actor differs on local module or learner")
    runner_states = algo.env_runner_group.foreach_env_runner(
        func=lambda runner: runner.get_state(
            components=[COMPONENT_RL_MODULE], inference_only=True
        ),
        local_env_runner=True,
        timeout_seconds=float(timeout_s),
    )
    runner_reports = []
    for index, state in enumerate(runner_states):
        actor = warm_start.find_actor_state(state)
        if actor is None:
            raise ZeroUpdatePortError(f"EnvRunner {index} has no actor state")
        comparison = warm_start.compare_actor_states(expected_state, actor)
        if not comparison["exact"]:
            raise ZeroUpdatePortError(f"EnvRunner {index} actor differs")
        runner_reports.append(comparison)
    if not runner_reports:
        raise ZeroUpdatePortError("no EnvRunner actor was audited")
    return {"local_module": local, "learner": learner, "env_runners": runner_reports}


def _export_actor(
    *,
    algo: Any,
    destination: Path,
    expected_state: Mapping[str, Any],
    warm_start: Any,
) -> dict[str, Any]:
    if destination.exists():
        raise ZeroUpdatePortError(f"refusing to clobber export: {destination}")
    algo.get_module(contract.DEFAULT_POLICY_ID).save_to_path(destination.resolve())
    exported = warm_start.load_module_state(destination)
    comparison = warm_start.compare_actor_states(expected_state, exported)
    if not comparison["exact"]:
        raise ZeroUpdatePortError("exported actor differs from qualified candidate")
    source_manifest = _strict_json(INPUT_PATHS["candidate_actor_manifest"])
    feature_names = source_manifest.get("actor_feature_names")
    actor_digest = warm_start.actor_state_digest(exported)
    if (
        not isinstance(feature_names, list)
        or len(feature_names) != contract.EXPECTED_ACTOR_FEATURES
        or any(not isinstance(name, str) or not name for name in feature_names)
        or len(set(feature_names)) != len(feature_names)
        or source_manifest.get("actor_feature_count") != len(feature_names)
        or source_manifest.get("actor_digest") != actor_digest
        or source_manifest.get("candidate_id") != contract.CANDIDATE_ID
        or source_manifest.get("observation_contract_id")
        != "primary_grf_split_v1"
        or source_manifest.get("event_contract_id") != "legacy_events_v1"
        or source_manifest.get("trainable_scope") != "full_mean_network"
        or source_manifest.get("logstd_policy") != "frozen_bit_exact"
    ):
        raise ZeroUpdatePortError("qualified actor feature manifest drifted")
    manifest = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_PRIMARY_SPLIT_V5_ZERO_UPDATE_ACTOR_MANIFEST",
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": contract.CANDIDATE_ID,
        "observation_contract_id": "primary_grf_split_v1",
        "event_contract_id": "legacy_events_v1",
        "trainable_scope": "full_mean_network",
        "logstd_policy": "frozen_bit_exact",
        "actor_feature_count": len(feature_names),
        "actor_feature_names": list(feature_names),
        "actor_digest": actor_digest,
        "module_state_sha256": _sha256_file(destination / "module_state.pkl"),
        "source_actor_manifest": source_record(
            INPUT_PATHS["candidate_actor_manifest"]
        ),
        "zero_update_execution_lock": source_record(LOCK),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "environment_samples": 0,
        "protected_trials_opened": [],
    }
    manifest_path = write_json_exclusive(
        destination / "actor_feature_manifest.json", manifest
    )
    return {
        "comparison": comparison,
        "manifest": source_record(manifest_path),
        "tree": tree_record(destination),
    }


def _assert_full_checkpoint(path: Path) -> dict[str, Any]:
    tree = tree_record(path)
    relative_files = set(tree["files"])
    missing = sorted(_CHECKPOINT_REQUIRED_SUFFIXES - relative_files)
    if missing:
        raise ZeroUpdatePortError(f"full checkpoint is incomplete: {missing}")
    return tree


def _build_algorithm(train_module: Any, args: Any, reward: dict[str, Any]) -> Any:
    config = train_module.build_config(args, reward)
    return config.build_algo()


def _runtime_port() -> dict[str, Any]:
    import training_config
    import train_ppo_mlp as train
    import warm_start

    train._load_training_stack()  # noqa: SLF001
    ray = train.ray
    args = _target_training_args(train, OUTPUT_ROOT / "rllib")
    reward = dict(getattr(args, "_cfg_reward", None) or {})
    training_config.dump_resolved(
        args,
        reward,
        OUTPUT_ROOT / contract.OUTPUT_NAMES["resolved_config"],
    )

    runtime_env = {
        "env_vars": {
            "KMP_DUPLICATE_LIB_OK": "TRUE",
            "PYTHONWARNINGS": "ignore",
        }
    }
    ray.init(
        include_dashboard=False,
        ignore_reinit_error=False,
        runtime_env=runtime_env,
        log_to_driver=False,
        num_cpus=1,
        num_gpus=0,
    )
    first = None
    restored = None
    try:
        from ray.rllib.algorithms.algorithm import (
            COMPONENT_LEARNER,
            COMPONENT_LEARNER_GROUP,
            COMPONENT_RL_MODULE,
        )
        from ray.tune.trainable import trainable as ray_trainable

        rllib_root = OUTPUT_ROOT / "rllib"
        rllib_root.mkdir(parents=True, exist_ok=False)
        ray_trainable.DEFAULT_STORAGE_PATH = str(rllib_root)
        candidate_state = warm_start.load_module_state(CANDIDATE_DIR)
        candidate_digest = warm_start.actor_state_digest(candidate_state)

        first = _build_algorithm(train, args, reward)
        progress_before = zero_progress_audit(first)
        optimizer_before = optimizer_empty_audit(train, first)
        target_names = tuple(getattr(args, "_target_actor_feature_names", ()))
        if len(target_names) != contract.EXPECTED_ACTOR_FEATURES:
            raise ZeroUpdatePortError("target actor feature layout is not 35")
        if int(getattr(args, "_target_n_full", -1)) != contract.EXPECTED_FULL_FEATURES:
            raise ZeroUpdatePortError("target full observation layout is not 84")

        local_before = first.get_module(contract.DEFAULT_POLICY_ID).get_state()
        learner_before = train._learner_module_state(first)  # noqa: SLF001
        fresh_critic = fresh_critic_audit(
            candidate_state=candidate_state,
            learner_state=learner_before,
            warm_start=warm_start,
        )
        transplanted, transplant_report = warm_start.transplant_actor_state(
            target_state=local_before,
            target_actor_feature_names=target_names,
            source_checkpoint=CANDIDATE_DIR,
            source_config=INPUT_PATHS["v5_training_config"],
            source_actor_feature_manifest=INPUT_PATHS["candidate_actor_manifest"],
            mode="drop",
            zero_target_features=(),
        )
        actor_copy = warm_start.compare_actor_states(candidate_state, transplanted)
        if not actor_copy["exact"]:
            raise ZeroUpdatePortError("actor transplant is not bit-exact")
        first.get_module(contract.DEFAULT_POLICY_ID).set_state(transplanted)
        first.set_state(
            {
                COMPONENT_LEARNER_GROUP: {
                    COMPONENT_LEARNER: {
                        COMPONENT_RL_MODULE: {
                            contract.DEFAULT_POLICY_ID: transplanted
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
        critic_preserved = warm_start.compare_non_actor_states(
            learner_before, learner_after
        )
        if not critic_preserved["exact"]:
            raise ZeroUpdatePortError("actor transplant changed the fresh critic")
        surfaces_before = _actor_surfaces(
            algo=first,
            expected_state=candidate_state,
            train_module=train,
            warm_start=warm_start,
            timeout_s=args.startup_timeout_s,
        )
        optimizer_after_transplant = optimizer_empty_audit(train, first)
        progress_after_transplant = zero_progress_audit(first)

        initial_export_path = OUTPUT_ROOT / contract.OUTPUT_NAMES["initial_export"]
        initial_export = _export_actor(
            algo=first,
            destination=initial_export_path,
            expected_state=candidate_state,
            warm_start=warm_start,
        )
        checkpoint_path = OUTPUT_ROOT / contract.OUTPUT_NAMES["checkpoint"]
        if checkpoint_path.exists():
            raise ZeroUpdatePortError(
                f"refusing to clobber checkpoint: {checkpoint_path}"
            )
        first.save_to_path(checkpoint_path)
        checkpoint_tree = _assert_full_checkpoint(checkpoint_path)
        if zero_progress_audit(first) != progress_before:
            raise ZeroUpdatePortError("checkpoint save changed progress counters")
        first.stop()
        first = None

        restored = _build_algorithm(train, args, reward)
        if any(zero_progress_audit(restored).values()):
            raise ZeroUpdatePortError("second fresh Algorithm did work before restore")
        restored.restore_from_path(checkpoint_path)
        restored_progress = zero_progress_audit(restored)
        restored_optimizer = optimizer_empty_audit(train, restored)
        restored_learner = train._learner_module_state(restored)  # noqa: SLF001
        restored_critic = warm_start.compare_non_actor_states(
            learner_after, restored_learner
        )
        if not restored_critic["exact"]:
            raise ZeroUpdatePortError("restored critic differs from saved fresh critic")
        restored_surfaces = _actor_surfaces(
            algo=restored,
            expected_state=candidate_state,
            train_module=train,
            warm_start=warm_start,
            timeout_s=args.startup_timeout_s,
        )
        restored_export = _export_actor(
            algo=restored,
            destination=OUTPUT_ROOT / contract.OUTPUT_NAMES["restored_export"],
            expected_state=candidate_state,
            warm_start=warm_start,
        )
        if any(restored_progress.values()):
            raise ZeroUpdatePortError("restored Algorithm has non-zero progress")

        checks = {name: True for name in REQUIRED_CHECKS}
        return {
            "candidate_actor_digest": candidate_digest,
            "checks": checks,
            "target_actor_feature_names": list(target_names),
            "target_observation_feature_names": list(
                getattr(args, "_target_observation_feature_names", ())
            ),
            "transplant_report": transplant_report,
            "actor_copy": actor_copy,
            "surfaces_before_save": surfaces_before,
            "surfaces_after_restore": restored_surfaces,
            "critic": {
                **fresh_critic,
                "preserved_by_transplant": critic_preserved,
                "restored_exact": restored_critic,
            },
            "optimizer": {
                "before_transplant": optimizer_before,
                "after_transplant": optimizer_after_transplant,
                "after_restore": restored_optimizer,
            },
            "progress": {
                "before_transplant": progress_before,
                "after_transplant": progress_after_transplant,
                "after_restore": restored_progress,
            },
            "checkpoint_tree": checkpoint_tree,
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


def _attempt_claim_payload() -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_PRIMARY_SPLIT_V5_ZERO_UPDATE_PORT_ATTEMPT_CLAIMED",
        "protocol_id": contract.PROTOCOL_ID,
        "execution_lock": source_record(LOCK),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "environment_samples": 0,
        "protected_trials_opened": [],
    }


def _receipt_payload(audit_path: Path, audit: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "source_protocol_id": contract.SOURCE_PROTOCOL_ID,
        "candidate_id": contract.CANDIDATE_ID,
        "candidate_actor_digest": audit["candidate_actor_digest"],
        "execution_lock": source_record(LOCK),
        "attempt_claim": source_record(
            OUTPUT_ROOT / contract.OUTPUT_NAMES["attempt_claim"]
        ),
        "qualification_execution_ledger": source_record(
            INPUT_PATHS["qualification_execution_ledger"]
        ),
        "candidate_freeze": source_record(INPUT_PATHS["candidate_freeze"]),
        "audit": source_record(audit_path),
        "resolved_config": source_record(
            OUTPUT_ROOT / contract.OUTPUT_NAMES["resolved_config"]
        ),
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
            "python_version": platform.python_version(),
            "python_executable": Path(sys.executable).resolve().as_posix(),
        },
        "actor_transplants": 1,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "environment_samples": 0,
        "protected_trials_opened": [],
    }


def execute() -> dict[str, Any]:
    verify_lock()
    if os.path.lexists(OUTPUT_ROOT):
        raise ZeroUpdatePortError(
            f"zero-update output is already claimed: {OUTPUT_ROOT}"
        )
    OUTPUT_ROOT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_ROOT.mkdir()
    write_json_exclusive(
        OUTPUT_ROOT / contract.OUTPUT_NAMES["attempt_claim"],
        _attempt_claim_payload(),
    )
    started = time.time()
    passed = False
    error: str | None = None
    audit: dict[str, Any] | None = None
    try:
        audit = _runtime_port()
        if (
            not isinstance(audit.get("checks"), Mapping)
            or set(audit["checks"]) != set(REQUIRED_CHECKS)
            or any(value is not True for value in audit["checks"].values())
        ):
            raise ZeroUpdatePortError("zero-update runtime checks did not all pass")
        audit_payload = {
            "schema_version": contract.SCHEMA_VERSION,
            "status": contract.PASS_STATUS,
            "passed": True,
            "protocol_id": contract.PROTOCOL_ID,
            **audit,
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
            "environment_samples": 0,
            "protected_trials_opened": [],
        }
        audit_path = write_json_exclusive(
            OUTPUT_ROOT / contract.OUTPUT_NAMES["audit"], audit_payload
        )
        receipt = _receipt_payload(audit_path, audit_payload)
        write_json_exclusive(
            OUTPUT_ROOT / contract.OUTPUT_NAMES["receipt"], receipt
        )
        passed = True
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
        receipt = {}
    ledger = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PASS_STATUS if passed else contract.FAIL_STATUS,
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "started_unix_s": started,
        "completed_unix_s": time.time(),
        "error": error,
        "execution_lock": source_record(LOCK),
        "attempt_claim": source_record(
            OUTPUT_ROOT / contract.OUTPUT_NAMES["attempt_claim"]
        ),
        "receipt_created": (
            OUTPUT_ROOT / contract.OUTPUT_NAMES["receipt"]
        ).is_file(),
        "actor_transplants": int(audit is not None),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "environment_samples": 0,
        "protected_trials_opened": [],
        "next_stage": "V25_ABC_PREFLIGHT" if passed else "STOP_WITHOUT_RETRY",
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
