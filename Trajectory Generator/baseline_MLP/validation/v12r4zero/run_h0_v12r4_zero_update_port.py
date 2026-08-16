"""Fail-closed post-Q2 V12R4 zero-update RLlib checkpoint port.

The runner is inert on import.  A future invocation requires an exclusively
published execution lock that binds the same exact actor-only V12R4 P3 tree in
R4 development, Q2 protocol, Q2 receipt, and both terminal ledgers.  It builds
fresh standard asymmetric PPO Algorithms, copies only the actor, never calls
``Algorithm.train``, samples zero environment steps, and verifies a full RLlib
save/reload at progress zero.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
import numbers
import os
import platform
import stat
import sys
import time
from pathlib import Path, PurePath, PurePosixPath
from typing import Any, Mapping, Sequence

try:
    from . import h0_v12r4_zero_update_contract as contract
except ImportError:  # Direct ``python validation/v12r4zero/run_*.py``.
    import h0_v12r4_zero_update_contract as contract


REPO_ROOT = Path(__file__).absolute().parents[4]
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
BASELINE_ROOT = TRAJECTORY_ROOT / "baseline_MLP"
for import_root in (BASELINE_ROOT, TRAJECTORY_ROOT, REPO_ROOT):
    text_root = os.fspath(import_root)
    if text_root not in sys.path:
        sys.path.insert(0, text_root)


class ZeroUpdatePortError(RuntimeError):
    """Raised when any lineage, safety, or zero-update invariant fails."""


def resolve_relative(value: str | PurePath) -> Path:
    """Resolve one canonical POSIX repository path without following links."""

    text = PurePosixPath(value).as_posix()
    pure = PurePosixPath(text)
    if not text or pure.is_absolute() or ".." in pure.parts or text != str(value):
        raise ZeroUpdatePortError(f"non-canonical repository path: {value!r}")
    return REPO_ROOT.joinpath(*pure.parts)


LOCK = resolve_relative(contract.LOCK_PATH)
OUTPUT_ROOT = resolve_relative(contract.OUTPUT_ROOT)
CANDIDATE_DIR = resolve_relative(contract.CANDIDATE_MODULE_PATH)
INPUT_PATHS = {
    name: resolve_relative(path) for name, path in contract.INPUT_RELATIVE_PATHS.items()
}
SOURCE_PATHS = {
    name: resolve_relative(path)
    for name, path in contract.SOURCE_RELATIVE_PATHS.items()
}

LOCK_KEYS = frozenset(
    {
        "schema_version",
        "status",
        "protocol_id",
        "qualification_protocol_id",
        "revision",
        "candidate_binding_state",
        "candidate_id",
        "candidate_module",
        "candidate_selection_rule",
        "source_topology_id",
        "target_contract_id",
        "target_fixed_config",
        "target_reward_config",
        "output_root",
        "authority",
        "required_checks",
        "sources",
        "inputs",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
        "environment_samples",
        "protected_trials_opened",
    }
)


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
        raise ZeroUpdatePortError("value is not strict finite JSON") from exc


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
    if isinstance(value, (list, tuple)):
        for index, child in enumerate(value):
            _require_finite_json(child, f"{label}[{index}]")
        return
    raise ZeroUpdatePortError(f"{label} contains a non-JSON value")


def _is_link_or_reparse(path: Path) -> bool:
    """Recognize POSIX links and Windows junction/reparse points."""

    try:
        status = os.lstat(path)
    except OSError:
        return False
    attributes = int(getattr(status, "st_file_attributes", 0) or 0)
    reparse_flag = int(getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400))
    return stat.S_ISLNK(status.st_mode) or bool(attributes & reparse_flag)


def _relative_parts(path: Path) -> tuple[str, ...]:
    root = Path(os.path.abspath(REPO_ROOT))
    absolute = Path(os.path.abspath(path))
    try:
        return absolute.relative_to(root).parts
    except ValueError as exc:
        raise ZeroUpdatePortError(f"path is outside repository: {path}") from exc


def reject_link_or_reparse_ancestors(path: Path, *, include_leaf: bool) -> None:
    """Reject every existing link/reparse component below the repository."""

    parts = _relative_parts(path)
    limit = len(parts) if include_leaf else max(0, len(parts) - 1)
    current = Path(os.path.abspath(REPO_ROOT))
    for part in parts[:limit]:
        current /= part
        if os.path.lexists(current) and _is_link_or_reparse(current):
            raise ZeroUpdatePortError(f"unsafe symlink/junction component: {current}")


def repo_relative(path: str | Path) -> str:
    parts = _relative_parts(Path(path))
    return PurePosixPath(*parts).as_posix()


def _regular_file_status(path: Path) -> os.stat_result:
    reject_link_or_reparse_ancestors(path, include_leaf=True)
    try:
        status = os.lstat(path)
    except OSError as exc:
        raise ZeroUpdatePortError(f"required file is missing: {path}") from exc
    if not stat.S_ISREG(status.st_mode):
        raise ZeroUpdatePortError(f"required path is not a regular file: {path}")
    return status


def _sha256_file(path: str | Path) -> str:
    """Hash one stable regular file without following a POSIX leaf symlink."""

    target = Path(path)
    before = _regular_file_status(target)
    flags = os.O_RDONLY | getattr(os, "O_BINARY", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(target, flags)
    except OSError as exc:
        raise ZeroUpdatePortError(f"cannot safely open file: {target}") from exc
    digest = hashlib.sha256()
    try:
        opened = os.fstat(descriptor)
        if not stat.S_ISREG(opened.st_mode):
            raise ZeroUpdatePortError(f"opened path is not regular: {target}")
        with os.fdopen(descriptor, "rb") as stream:
            descriptor = -1
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    finally:
        if descriptor >= 0:
            os.close(descriptor)
    after = _regular_file_status(target)
    stable_fields = ("st_dev", "st_ino", "st_size", "st_mtime_ns")
    if any(
        getattr(before, key, None) != getattr(after, key, None) for key in stable_fields
    ):
        raise ZeroUpdatePortError(f"file changed while hashing: {target}")
    return digest.hexdigest()


def source_record(path: str | Path) -> dict[str, Any]:
    target = Path(path)
    before = _regular_file_status(target)
    digest = _sha256_file(target)
    after = _regular_file_status(target)
    if before.st_size != after.st_size:
        raise ZeroUpdatePortError(f"file size changed while recording: {target}")
    return {
        "path": repo_relative(target),
        "sha256": digest,
        "size_bytes": int(after.st_size),
    }


def tree_record(root: str | Path) -> dict[str, Any]:
    """Hash a regular-file tree using the exact R4 candidate tree ABI."""

    directory = Path(root)
    reject_link_or_reparse_ancestors(directory, include_leaf=True)
    try:
        status = os.lstat(directory)
    except OSError as exc:
        raise ZeroUpdatePortError(f"candidate tree is missing: {directory}") from exc
    if not stat.S_ISDIR(status.st_mode):
        raise ZeroUpdatePortError(f"candidate tree is not a directory: {directory}")

    file_paths: list[Path] = []
    for current_text, directory_names, file_names in os.walk(
        directory, topdown=True, followlinks=False
    ):
        current = Path(current_text)
        for name in list(directory_names):
            child = current / name
            if _is_link_or_reparse(child):
                raise ZeroUpdatePortError(
                    f"candidate tree contains a symlink/junction: {child}"
                )
            child_status = os.lstat(child)
            if not stat.S_ISDIR(child_status.st_mode):
                raise ZeroUpdatePortError(
                    f"candidate tree contains a non-directory entry: {child}"
                )
        for name in file_names:
            child = current / name
            if _is_link_or_reparse(child):
                raise ZeroUpdatePortError(
                    f"candidate tree contains a symlink/junction: {child}"
                )
            child_status = os.lstat(child)
            if not stat.S_ISREG(child_status.st_mode):
                raise ZeroUpdatePortError(
                    f"candidate tree contains a non-regular file: {child}"
                )
            file_paths.append(child)
    if not file_paths:
        raise ZeroUpdatePortError(f"candidate tree is empty: {directory}")

    digest = hashlib.sha256()
    rows: list[dict[str, Any]] = []
    for path in sorted(
        file_paths, key=lambda item: item.relative_to(directory).as_posix()
    ):
        relative = path.relative_to(directory).as_posix()
        record = source_record(path)
        row = {
            "path": relative,
            "sha256": record["sha256"],
            "size_bytes": record["size_bytes"],
        }
        rows.append(row)
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(str(record["sha256"]).encode("ascii"))
        digest.update(b"\0")
        digest.update(str(record["size_bytes"]).encode("ascii"))
        digest.update(b"\n")
    return {
        "path": repo_relative(directory),
        "tree_sha256": digest.hexdigest(),
        "file_count": len(rows),
        "files": rows,
    }


def closure_snapshot() -> dict[str, Any]:
    """Hash all future lock sources/inputs and the candidate tree."""

    return {
        "sources": {name: source_record(path) for name, path in SOURCE_PATHS.items()},
        "inputs": {name: source_record(path) for name, path in INPUT_PATHS.items()},
        "candidate_module": tree_record(CANDIDATE_DIR),
    }


def verify_closure(snapshot: Mapping[str, Any]) -> dict[str, Any]:
    """Perform a fresh TOCTOU rehash and require byte-identical closure."""

    observed = closure_snapshot()
    if _canonical_json_bytes(observed) != _canonical_json_bytes(snapshot):
        raise ZeroUpdatePortError("locked source/input/candidate closure drifted")
    return observed


def strict_json(path: str | Path) -> dict[str, Any]:
    target = Path(path)
    _regular_file_status(target)

    def reject_duplicates(pairs: Sequence[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"duplicate JSON key: {key}")
            result[key] = value
        return result

    try:
        with target.open("r", encoding="utf-8") as stream:
            value = json.load(
                stream,
                parse_constant=lambda token: (_ for _ in ()).throw(
                    ValueError(f"non-finite JSON token: {token}")
                ),
                object_pairs_hook=reject_duplicates,
            )
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        raise ZeroUpdatePortError(f"invalid strict JSON: {target}") from exc
    if not isinstance(value, Mapping):
        raise ZeroUpdatePortError(f"JSON root is not an object: {target}")
    _require_finite_json(value, os.fspath(target))
    return dict(value)


def write_json_exclusive(path: str | Path, payload: Mapping[str, Any]) -> Path:
    """Publish one strict JSON file without following or replacing a target."""

    destination = Path(path)
    _require_finite_json(payload, "JSON payload")
    encoded = (
        json.dumps(
            dict(payload),
            indent=2,
            sort_keys=True,
            ensure_ascii=False,
            allow_nan=False,
        ).encode("utf-8")
        + b"\n"
    )
    reject_link_or_reparse_ancestors(destination, include_leaf=False)
    flags = (
        os.O_CREAT
        | os.O_EXCL
        | os.O_WRONLY
        | getattr(os, "O_BINARY", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    try:
        descriptor = os.open(destination, flags, 0o600)
    except OSError as exc:
        raise ZeroUpdatePortError(
            f"refusing unsafe/clobber write: {destination}"
        ) from exc
    try:
        with os.fdopen(descriptor, "wb") as stream:
            descriptor = -1
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
    except Exception:
        try:
            destination.unlink()
        except OSError:
            pass
        raise
    finally:
        if descriptor >= 0:
            os.close(descriptor)
    return destination


def validate_v26_target_config() -> dict[str, Any]:
    """Validate the additive Q2-blocked V26 morphology target YAML exactly."""

    try:
        import yaml

        class UniqueKeyLoader(yaml.SafeLoader):
            pass

        def construct_mapping(loader: Any, node: Any, deep: bool = False) -> Any:
            pairs = loader.construct_pairs(node, deep=deep)
            result: dict[Any, Any] = {}
            for key, value in pairs:
                if key in result:
                    raise ValueError(f"duplicate YAML key: {key}")
                result[key] = value
            return result

        UniqueKeyLoader.add_constructor(
            yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, construct_mapping
        )
        target = INPUT_PATHS["v26_target_config"]
        _regular_file_status(target)
        with target.open("r", encoding="utf-8") as stream:
            value = yaml.load(stream, Loader=UniqueKeyLoader)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        raise ZeroUpdatePortError("invalid V26 target config") from exc
    if not isinstance(value, Mapping):
        raise ZeroUpdatePortError("V26 target config root is not a mapping")
    candidate = value.get("candidate")
    grf = value.get("grf")
    reward = value.get("reward")
    if not all(isinstance(item, Mapping) for item in (candidate, grf, reward)):
        raise ZeroUpdatePortError("V26 target config sections are missing")
    if (
        candidate.get("status") != "v26_runtime_ready_q2_blocked"
        or candidate.get("positive_ab_gate") != "Q2"
        or candidate.get("ppo_updates_authorized") is not False
        or candidate.get("qualifying_rollouts_authorized") is not False
        or candidate.get("active_training_config_replaced") is not False
    ):
        raise ZeroUpdatePortError("V26 target candidate gate drifted")
    expected_grf = {
        "phase_fsm_input_mode": "legacy_events",
        "event_contract_id": "legacy_events_v1",
        "binary_phase_fsm_mode": "binary_active",
        "binary_phase_detector_profile": contract.V26_DETECTOR_PROFILE_PATH.as_posix(),
        "binary_phase_event_contract_id": contract.BINARY_EVENT_CONTRACT_ID,
        "binary_phase_debounce_s": 0.005,
        "detector_sample_dt_s": 0.001,
        "policy_step_s": 0.01,
    }
    if any(grf.get(key) != expected for key, expected in expected_grf.items()):
        raise ZeroUpdatePortError("V26 target GRF/FSM config drifted")
    if any(
        reward.get(key) != expected
        for key, expected in contract.TARGET_REWARD_CONFIG.items()
    ):
        raise ZeroUpdatePortError("V26 target morphology config drifted")
    if _sha256_file(INPUT_PATHS["v26_detector_profile"]) != grf.get(
        "binary_phase_detector_profile_sha256"
    ):
        raise ZeroUpdatePortError("V26 detector profile hash drifted")
    if _sha256_file(INPUT_PATHS["morphology_profile"]) != reward.get(
        "morphology_profile_sha256"
    ):
        raise ZeroUpdatePortError("morphology profile hash drifted")
    return {"grf": dict(grf), "reward": dict(reward)}


def verify_lock() -> dict[str, Any]:
    lock = strict_json(LOCK)
    if set(lock) != LOCK_KEYS:
        raise ZeroUpdatePortError("execution lock schema drifted")
    candidate = lock.get("candidate_module")
    tree_hash = candidate.get("tree_sha256") if isinstance(candidate, Mapping) else None
    expected_id = contract.candidate_id_for_tree(tree_hash)
    exact = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.LOCK_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "qualification_protocol_id": contract.QUALIFICATION_PROTOCOL_ID,
        "revision": contract.REVISION,
        "candidate_binding_state": "BOUND_AFTER_Q2_TERMINAL_PASS",
        "candidate_id": expected_id,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "source_topology_id": contract.SOURCE_TOPOLOGY_ID,
        "target_contract_id": contract.TARGET_CONTRACT_ID,
        "target_fixed_config": contract.TARGET_FIXED_CONFIG,
        "target_reward_config": contract.TARGET_REWARD_CONFIG,
        "output_root": repo_relative(OUTPUT_ROOT),
        "authority": contract.AUTHORITY,
        "required_checks": list(contract.REQUIRED_CHECKS),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "environment_samples": 0,
        "protected_trials_opened": [],
    }
    for key, expected in exact.items():
        if _canonical_json_bytes(lock.get(key)) != _canonical_json_bytes(expected):
            raise ZeroUpdatePortError(f"execution lock drifted at {key}")
    closure = {
        "sources": lock.get("sources"),
        "inputs": lock.get("inputs"),
        "candidate_module": candidate,
    }
    verify_closure(closure)
    validate_v26_target_config()
    return lock


def _as_zero(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise ZeroUpdatePortError(f"{label} is not numeric")
    numeric = float(value)
    if not math.isfinite(numeric) or numeric != 0.0:
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
    logger = getattr(algo, "metrics", None)
    if logger is None or not callable(getattr(logger, "peek", None)):
        raise ZeroUpdatePortError("Algorithm metrics logger is unavailable")
    result = {
        "training_iteration": _as_zero(
            getattr(algo, "iteration", None), "training_iteration"
        )
    }
    result.update(
        {
            label: _as_zero(logger.peek(metric, default=0.0), label)
            for label, metric in metric_names.items()
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


def _json_option(value: Any, label: str) -> Any:
    if value is None or type(value) in {bool, str, int}:
        return value
    if isinstance(value, numbers.Real):
        numeric = float(value)
        if not math.isfinite(numeric):
            raise ZeroUpdatePortError(f"non-finite optimizer option: {label}")
        return numeric
    if isinstance(value, (list, tuple)):
        return [_json_option(item, label) for item in value]
    raise ZeroUpdatePortError(f"unsupported optimizer option {label}: {type(value)}")


def optimizer_snapshot_on_learner(learner: Any) -> dict[str, Any]:
    """Prove empty optimizer state and exact one-to-one parameter groups."""

    module = _learner_module(learner)
    named = list(module.named_parameters())
    if not named or len({name for name, _ in named}) != len(named):
        raise ZeroUpdatePortError("learner named-parameter surface is invalid")
    trainable = {
        id(parameter): name for name, parameter in named if parameter.requires_grad
    }
    if not trainable:
        raise ZeroUpdatePortError("target learner exposes no trainable parameters")
    seen: dict[int, str] = {}
    optimizers: list[dict[str, Any]] = []
    optimizer_names: set[str] = set()
    for optimizer_name, optimizer in learner.get_optimizers_for_module(
        contract.DEFAULT_POLICY_ID
    ):
        name = str(optimizer_name)
        if not name or name in optimizer_names:
            raise ZeroUpdatePortError("optimizer names are empty or duplicated")
        optimizer_names.add(name)
        state = optimizer.state_dict()
        state_entries = state.get("state") if isinstance(state, Mapping) else None
        groups = getattr(optimizer, "param_groups", None)
        if not isinstance(state_entries, Mapping) or state_entries:
            raise ZeroUpdatePortError(f"optimizer {name} state is not empty")
        if not isinstance(groups, list) or not groups:
            raise ZeroUpdatePortError(f"optimizer {name} has no parameter groups")
        group_rows: list[dict[str, Any]] = []
        for group_index, group in enumerate(groups):
            if not isinstance(group, Mapping) or not isinstance(
                group.get("params"), list
            ):
                raise ZeroUpdatePortError(f"optimizer {name} group is malformed")
            parameter_names: list[str] = []
            for parameter in group["params"]:
                identity = id(parameter)
                parameter_name = trainable.get(identity)
                if parameter_name is None:
                    raise ZeroUpdatePortError(
                        f"optimizer {name} contains unknown/frozen parameter"
                    )
                if identity in seen:
                    raise ZeroUpdatePortError(
                        f"optimizer parameter duplicated across groups: {parameter_name}"
                    )
                seen[identity] = f"{name}:{group_index}"
                parameter_names.append(parameter_name)
            options = {
                key: _json_option(value, f"{name}.{key}")
                for key, value in sorted(group.items())
                if key != "params"
            }
            group_rows.append(
                {
                    "group_index": group_index,
                    "parameter_names": parameter_names,
                    "options": options,
                }
            )
        optimizers.append(
            {
                "optimizer_name": name,
                "optimizer_type": type(optimizer).__name__,
                "state_entry_count": 0,
                "param_groups": group_rows,
            }
        )
    if not optimizers:
        raise ZeroUpdatePortError("target learner has no optimizer")
    missing = sorted(
        name for identity, name in trainable.items() if identity not in seen
    )
    if missing:
        raise ZeroUpdatePortError(
            f"trainable parameters missing from optimizer: {missing}"
        )
    return {
        "optimizer_state_empty": True,
        "trainable_parameter_count": len(trainable),
        "trainable_parameter_names": sorted(trainable.values()),
        "all_trainable_parameters_registered_once": True,
        "optimizers": optimizers,
    }


def optimizer_audit(train_module: Any, algo: Any) -> list[dict[str, Any]]:
    reports = train_module._learner_call_results(  # noqa: SLF001
        algo, optimizer_snapshot_on_learner
    )
    if not reports:
        raise ZeroUpdatePortError("optimizer audit returned no learner evidence")
    return reports


def fresh_critic_audit(
    *,
    candidate_state: Mapping[str, Any],
    learner_state: Mapping[str, Any],
    warm_start: Any,
) -> dict[str, Any]:
    candidate_non_actor = warm_start.compare_non_actor_states(
        candidate_state, candidate_state
    )
    fresh_critic = warm_start.compare_non_actor_states(learner_state, learner_state)
    boundary = warm_start.compare_non_actor_states(candidate_state, learner_state)
    if (
        candidate_non_actor["keys"]
        or not fresh_critic["keys"]
        or not fresh_critic["exact"]
        or boundary["keys"]
        or boundary["missing_keys"]
        or boundary["unexpected_keys"] != fresh_critic["keys"]
        or boundary["exact"]
    ):
        raise ZeroUpdatePortError("target critic is not fresh and source-independent")
    return {
        "candidate_actor_only": candidate_non_actor,
        "fresh_target_critic": fresh_critic,
        "source_target_boundary": boundary,
    }


def transplant_standard_actor(
    *,
    target_state: Mapping[str, Any],
    candidate_state: Mapping[str, Any],
    warm_start: Any,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Copy the exact standard full-mean actor and retain the fresh critic."""

    candidate_digest = warm_start.actor_state_digest(candidate_state)
    candidate_non_actor = warm_start.compare_non_actor_states(
        candidate_state, candidate_state
    )
    if candidate_non_actor["keys"] or any(
        "residual" in key.lower() for key in candidate_state
    ):
        raise ZeroUpdatePortError("candidate is not the actor-only standard topology")
    merged = {
        key: value.clone() if hasattr(value, "clone") else copy.deepcopy(value)
        for key, value in target_state.items()
    }
    actor_keys = tuple(candidate_state)
    for key in actor_keys:
        if key not in merged:
            raise ZeroUpdatePortError(f"target actor key is missing: {key}")
        value = candidate_state[key]
        merged[key] = value.clone() if hasattr(value, "clone") else copy.deepcopy(value)
    actor = warm_start.compare_actor_states(candidate_state, merged)
    critic = warm_start.compare_non_actor_states(target_state, merged)
    if not actor["exact"] or not critic["exact"]:
        raise ZeroUpdatePortError("actor transplant was not exact/critic preserving")
    if warm_start.actor_state_digest(merged) != candidate_digest:
        raise ZeroUpdatePortError("transplanted actor digest drifted")
    return merged, {
        "topology": contract.SOURCE_TOPOLOGY_ID,
        "candidate_actor_digest": candidate_digest,
        "actor": actor,
        "fresh_critic_preserved": critic,
        "source_actor_key_count": len(actor_keys),
        "source_non_actor_key_count": 0,
        "residual_parameter_count": 0,
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
        raise ZeroUpdatePortError("local/learner actor surface drifted")
    runner_states = algo.env_runner_group.foreach_env_runner(
        func=lambda runner: runner.get_state(
            components=[COMPONENT_RL_MODULE], inference_only=True
        ),
        local_env_runner=True,
        timeout_seconds=float(timeout_s),
    )
    runner_reports: list[dict[str, Any]] = []
    for index, state in enumerate(runner_states):
        actor_state = warm_start.find_actor_state(state)
        if actor_state is None:
            raise ZeroUpdatePortError(f"EnvRunner {index} has no actor state")
        comparison = warm_start.compare_actor_states(expected_state, actor_state)
        if not comparison["exact"]:
            raise ZeroUpdatePortError(f"EnvRunner {index} actor surface drifted")
        runner_reports.append(comparison)
    if not runner_reports:
        raise ZeroUpdatePortError("no local EnvRunner actor surface was audited")
    return {"local": local, "learner": learner, "env_runners": runner_reports}


def _export_actor(
    *,
    algo: Any,
    destination: Path,
    expected_state: Mapping[str, Any],
    feature_names: Sequence[str],
    lock: Mapping[str, Any],
    warm_start: Any,
) -> dict[str, Any]:
    if os.path.lexists(destination):
        raise ZeroUpdatePortError(f"refusing to clobber export: {destination}")
    reject_link_or_reparse_ancestors(destination, include_leaf=False)
    algo.get_module(contract.DEFAULT_POLICY_ID).save_to_path(destination)
    exported = warm_start.load_module_state(destination)
    comparison = warm_start.compare_actor_states(expected_state, exported)
    if not comparison["exact"]:
        raise ZeroUpdatePortError("exported actor differs from qualified candidate")
    manifest = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_V12R4_Q2_V26_ZERO_UPDATE_ACTOR_EXPORT",
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": lock["candidate_id"],
        "candidate_module": lock["candidate_module"],
        "source_topology_id": contract.SOURCE_TOPOLOGY_ID,
        "rl_module_kind": contract.STANDARD_RL_MODULE_KIND,
        "target_contract_id": contract.TARGET_CONTRACT_ID,
        "actor_feature_count": len(feature_names),
        "actor_feature_names": list(feature_names),
        "actor_digest": warm_start.actor_state_digest(exported),
        "module_state_sha256": _sha256_file(destination / "module_state.pkl"),
        "morphology_weight": 0.0,
        "execution_lock": source_record(LOCK),
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
    relative_files = {row["path"] for row in tree["files"]}
    missing = sorted(contract.CHECKPOINT_REQUIRED_SUFFIXES - relative_files)
    if missing:
        raise ZeroUpdatePortError(f"full RLlib checkpoint is incomplete: {missing}")
    return tree


def _target_training_args(
    train_module: Any, output_dir: Path
) -> tuple[Any, dict[str, Any]]:
    target = validate_v26_target_config()
    argv = [
        os.fspath(train_module.__file__),
        "--config",
        os.fspath(INPUT_PATHS["source_training_config"]),
        "--output-dir",
        os.fspath(output_dir),
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
        contract.STANDARD_RL_MODULE_KIND,
        "--phase-fsm-input-mode",
        "legacy_events",
        "--event-contract-id",
        contract.LEGACY_EVENT_CONTRACT_ID,
        "--binary-phase-fsm-mode",
        "binary_active",
        "--binary-phase-detector-profile",
        contract.V26_DETECTOR_PROFILE_PATH.as_posix(),
        "--binary-phase-debounce-s",
        "0.005",
        "--binary-phase-event-contract-id",
        contract.BINARY_EVENT_CONTRACT_ID,
    ]
    previous = list(sys.argv)
    try:
        sys.argv = argv
        args = train_module.parse_args()
    finally:
        sys.argv = previous
    observed = {key: getattr(args, key) for key in contract.TARGET_FIXED_CONFIG}
    if _canonical_json_bytes(observed) != _canonical_json_bytes(
        contract.TARGET_FIXED_CONFIG
    ):
        raise ZeroUpdatePortError(f"resolved V26 fixed config drifted: {observed}")
    reward = dict(getattr(args, "_cfg_reward", None) or {})
    reward.update(target["reward"])
    for key, expected in contract.TARGET_REWARD_CONFIG.items():
        if reward.get(key) != expected:
            raise ZeroUpdatePortError(f"resolved V26 reward drifted at {key}")
    if reward.get("morphology_weight") != 0.0:
        raise ZeroUpdatePortError("morphology weight is not exactly zero")
    args._cfg_reward = reward
    return args, reward


def _build_algorithm(train_module: Any, args: Any, reward: Mapping[str, Any]) -> Any:
    config = train_module.build_config(args, dict(reward))
    return config.build_algo()


def _runtime_port(lock: Mapping[str, Any]) -> dict[str, Any]:
    import training_config
    import train_ppo_mlp as train
    import warm_start

    locked_closure = {
        "sources": lock["sources"],
        "inputs": lock["inputs"],
        "candidate_module": lock["candidate_module"],
    }
    verify_closure(locked_closure)
    train._load_training_stack()  # noqa: SLF001
    ray = train.ray
    args, reward = _target_training_args(train, OUTPUT_ROOT / "rllib")
    training_config.dump_resolved(
        args,
        reward,
        OUTPUT_ROOT / contract.OUTPUT_NAMES["resolved_config"],
    )
    ray.init(
        include_dashboard=False,
        ignore_reinit_error=False,
        runtime_env={
            "env_vars": {
                "KMP_DUPLICATE_LIB_OK": "TRUE",
                "PYTHONWARNINGS": "ignore",
            }
        },
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
        ray_trainable.DEFAULT_STORAGE_PATH = os.fspath(rllib_root)
        candidate_state = warm_start.load_module_state(CANDIDATE_DIR)
        candidate_digest = warm_start.actor_state_digest(candidate_state)
        if warm_start.compare_non_actor_states(candidate_state, candidate_state)[
            "keys"
        ]:
            raise ZeroUpdatePortError("R4 candidate unexpectedly contains a critic")

        verify_closure(locked_closure)
        first = _build_algorithm(train, args, reward)
        progress_before = zero_progress_audit(first)
        optimizer_before = optimizer_audit(train, first)
        feature_names = tuple(getattr(args, "_target_actor_feature_names", ()))
        full_names = tuple(getattr(args, "_target_observation_feature_names", ()))
        if len(feature_names) != contract.EXPECTED_ACTOR_FEATURES:
            raise ZeroUpdatePortError("target actor feature layout is not exactly 35")
        if len(full_names) != contract.EXPECTED_FULL_FEATURES:
            raise ZeroUpdatePortError(
                "target full observation layout is not exactly 84"
            )

        local_before = first.get_module(contract.DEFAULT_POLICY_ID).get_state()
        learner_before = train._learner_module_state(first)  # noqa: SLF001
        fresh_critic = fresh_critic_audit(
            candidate_state=candidate_state,
            learner_state=learner_before,
            warm_start=warm_start,
        )
        transplanted, transplant_report = transplant_standard_actor(
            target_state=local_before,
            candidate_state=candidate_state,
            warm_start=warm_start,
        )
        first.get_module(contract.DEFAULT_POLICY_ID).set_state(transplanted)
        first.set_state(
            {
                COMPONENT_LEARNER_GROUP: {
                    COMPONENT_LEARNER: {
                        COMPONENT_RL_MODULE: {contract.DEFAULT_POLICY_ID: transplanted}
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
            raise ZeroUpdatePortError("actor transplant changed fresh critic")
        optimizer_after = optimizer_audit(train, first)
        if _canonical_json_bytes(optimizer_after) != _canonical_json_bytes(
            optimizer_before
        ):
            raise ZeroUpdatePortError("optimizer param groups changed by transplant")
        surfaces_before = _actor_surfaces(
            algo=first,
            expected_state=candidate_state,
            train_module=train,
            warm_start=warm_start,
            timeout_s=args.startup_timeout_s,
        )
        progress_after = zero_progress_audit(first)
        if progress_after != progress_before:
            raise ZeroUpdatePortError("actor transplant changed progress counters")
        initial_export = _export_actor(
            algo=first,
            destination=OUTPUT_ROOT / contract.OUTPUT_NAMES["initial_export"],
            expected_state=candidate_state,
            feature_names=feature_names,
            lock=lock,
            warm_start=warm_start,
        )

        verify_closure(locked_closure)
        checkpoint = OUTPUT_ROOT / contract.OUTPUT_NAMES["checkpoint"]
        if os.path.lexists(checkpoint):
            raise ZeroUpdatePortError(f"refusing to clobber checkpoint: {checkpoint}")
        first.save_to_path(checkpoint)
        checkpoint_tree = _assert_full_checkpoint(checkpoint)
        if zero_progress_audit(first) != progress_before:
            raise ZeroUpdatePortError("checkpoint save changed progress counters")
        first.stop()
        first = None

        verify_closure(locked_closure)
        restored = _build_algorithm(train, args, reward)
        if any(zero_progress_audit(restored).values()):
            raise ZeroUpdatePortError("fresh restore target already has progress")
        restored.restore_from_path(checkpoint)
        restored_progress = zero_progress_audit(restored)
        restored_optimizer = optimizer_audit(train, restored)
        if _canonical_json_bytes(restored_optimizer) != _canonical_json_bytes(
            optimizer_before
        ):
            raise ZeroUpdatePortError("restored optimizer param groups/state drifted")
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
            feature_names=feature_names,
            lock=lock,
            warm_start=warm_start,
        )
        if any(restored_progress.values()):
            raise ZeroUpdatePortError("restored Algorithm has non-zero progress")
        verify_closure(locked_closure)
        return {
            "candidate_id": lock["candidate_id"],
            "candidate_actor_digest": candidate_digest,
            "checks": {name: True for name in contract.REQUIRED_CHECKS},
            "target_actor_feature_names": list(feature_names),
            "target_observation_feature_names": list(full_names),
            "transplant": transplant_report,
            "actor_surfaces_before_save": surfaces_before,
            "actor_surfaces_after_restore": restored_surfaces,
            "critic": {
                **fresh_critic,
                "preserved_by_transplant": critic_preserved,
                "restored_exact": restored_critic,
            },
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


def _attempt_claim_payload(lock: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.ATTEMPT_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": lock["candidate_id"],
        "execution_lock": source_record(LOCK),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "environment_samples": 0,
        "protected_trials_opened": [],
    }


def execute() -> dict[str, Any]:
    """Execute the future one-shot port; never called during source preparation."""

    lock = verify_lock()
    if os.path.lexists(OUTPUT_ROOT):
        raise ZeroUpdatePortError(
            f"zero-update output is already claimed: {OUTPUT_ROOT}"
        )
    reject_link_or_reparse_ancestors(OUTPUT_ROOT, include_leaf=False)
    OUTPUT_ROOT.mkdir(parents=False, exist_ok=False)
    claim_path = write_json_exclusive(
        OUTPUT_ROOT / contract.OUTPUT_NAMES["attempt_claim"],
        _attempt_claim_payload(lock),
    )
    started = time.time()
    runtime: dict[str, Any] | None = None
    passed = False
    error: str | None = None
    receipt: dict[str, Any] = {}
    try:
        runtime = _runtime_port(lock)
        checks = runtime.get("checks")
        if (
            not isinstance(checks, Mapping)
            or set(checks) != set(contract.REQUIRED_CHECKS)
            or any(value is not True for value in checks.values())
        ):
            raise ZeroUpdatePortError("zero-update runtime checks are incomplete")
        audit = {
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
        }
        audit_path = write_json_exclusive(
            OUTPUT_ROOT / contract.OUTPUT_NAMES["audit"], audit
        )
        receipt = {
            "schema_version": contract.SCHEMA_VERSION,
            "status": contract.PASS_STATUS,
            "passed": True,
            "protocol_id": contract.PROTOCOL_ID,
            "candidate_id": lock["candidate_id"],
            "candidate_module": lock["candidate_module"],
            "execution_lock": source_record(LOCK),
            "attempt_claim": source_record(claim_path),
            "q2_final_receipt": source_record(INPUT_PATHS["q2_final_receipt"]),
            "q2_pipeline_ledger": source_record(INPUT_PATHS["q2_pipeline_ledger"]),
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
                "python_executable": os.fspath(Path(sys.executable).absolute()),
            },
            "actor_transplants": 1,
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
            "environment_samples": 0,
            "protected_trials_opened": [],
            "runtime_promoted": False,
            "warm_start_command_published": False,
        }
        write_json_exclusive(OUTPUT_ROOT / contract.OUTPUT_NAMES["receipt"], receipt)
        passed = True
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
    ledger = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PASS_STATUS if passed else contract.FAIL_STATUS,
        "passed": passed,
        "terminal": True,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": lock["candidate_id"],
        "started_unix_s": started,
        "completed_unix_s": time.time(),
        "error": error,
        "execution_lock": source_record(LOCK),
        "attempt_claim": source_record(claim_path),
        "receipt_created": (OUTPUT_ROOT / contract.OUTPUT_NAMES["receipt"]).is_file(),
        "actor_transplants": int(runtime is not None),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "environment_samples": 0,
        "protected_trials_opened": [],
        "runtime_promoted": False,
        "warm_start_command_published": False,
        "next_stage": "WAIT_SEPARATE_TRAINING_AUTHORITY" if passed else "STOP",
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
