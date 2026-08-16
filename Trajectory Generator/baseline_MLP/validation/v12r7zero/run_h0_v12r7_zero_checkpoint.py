"""Historical fail-closed V12R7/Q3 W512 checkpoint-zero scaffold.

The module is inert on import.  V12R7 closed with a terminal FAIL, therefore
its public execution and terminal-verifier entrypoints are permanently blocked.
The internal implementation remains a tested source template for a new,
separately bound V12R8 namespace: fresh standard asymmetric PPO module, exact
actor-only transplant, no train/sample/update call, and byte-exact full RLlib
checkpoint save/reload at progress zero.
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

import numpy as np

try:
    from . import h0_v12r7_zero_checkpoint_contract as contract
except ImportError:  # Direct ``python validation/v12r7zero/run_*.py``.
    import h0_v12r7_zero_checkpoint_contract as contract


REPO_ROOT = Path(__file__).absolute().parents[4]
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
BASELINE_ROOT = TRAJECTORY_ROOT / "baseline_MLP"
for _root in (
    BASELINE_ROOT,
    TRAJECTORY_ROOT,
    REPO_ROOT,
    REPO_ROOT / "validation",
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))


class ZeroCheckpointError(RuntimeError):
    """Raised when lineage, zero-state, or publication invariants drift."""


def resolve_relative(value: str | PurePath) -> Path:
    """Resolve one canonical repository-relative POSIX path without links."""

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
        raise ZeroCheckpointError(f"non-canonical repository path: {value!r}")
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
        "pipeline_id",
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
        "prerequisite_audit",
        "sources",
        "inputs",
        "actor_transplants",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
        "environment_samples",
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
        raise ZeroCheckpointError("value is not strict finite JSON") from exc


def _require_finite_json(value: Any, label: str) -> None:
    if value is None or type(value) in {bool, str, int}:
        return
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ZeroCheckpointError(f"{label} contains a non-finite number")
        return
    if isinstance(value, Mapping):
        for key, child in value.items():
            if not isinstance(key, str) or not key:
                raise ZeroCheckpointError(f"{label} contains an invalid key")
            _require_finite_json(child, f"{label}.{key}")
        return
    if isinstance(value, (list, tuple)):
        for index, child in enumerate(value):
            _require_finite_json(child, f"{label}[{index}]")
        return
    raise ZeroCheckpointError(f"{label} contains a non-JSON value")


def _is_link_or_reparse(path: Path) -> bool:
    try:
        status = os.lstat(path)
    except OSError:
        return False
    attributes = int(getattr(status, "st_file_attributes", 0) or 0)
    reparse = int(getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400))
    return stat.S_ISLNK(status.st_mode) or bool(attributes & reparse)


def _relative_parts(path: Path) -> tuple[str, ...]:
    root = Path(os.path.abspath(REPO_ROOT))
    absolute = Path(os.path.abspath(path))
    try:
        return absolute.relative_to(root).parts
    except ValueError as exc:
        raise ZeroCheckpointError(f"path escapes repository root: {path}") from exc


def reject_link_or_reparse_ancestors(path: Path, *, include_leaf: bool) -> None:
    parts = _relative_parts(path)
    limit = len(parts) if include_leaf else max(0, len(parts) - 1)
    current = Path(os.path.abspath(REPO_ROOT))
    for part in parts[:limit]:
        current /= part
        if os.path.lexists(current) and _is_link_or_reparse(current):
            raise ZeroCheckpointError(f"unsafe symlink/junction component: {current}")


def repo_relative(path: str | Path) -> str:
    return PurePosixPath(*_relative_parts(Path(path))).as_posix()


def _regular_file_status(path: Path) -> os.stat_result:
    reject_link_or_reparse_ancestors(path, include_leaf=True)
    try:
        status = os.lstat(path)
    except OSError as exc:
        raise ZeroCheckpointError(f"required file is missing: {path}") from exc
    if not stat.S_ISREG(status.st_mode):
        raise ZeroCheckpointError(f"required path is not a regular file: {path}")
    return status


def _sha256_file(path: str | Path) -> str:
    target = Path(path)
    before = _regular_file_status(target)
    flags = os.O_RDONLY | getattr(os, "O_BINARY", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(target, flags)
    except OSError as exc:
        raise ZeroCheckpointError(f"cannot safely open file: {target}") from exc
    digest = hashlib.sha256()
    try:
        opened = os.fstat(descriptor)
        if not stat.S_ISREG(opened.st_mode):
            raise ZeroCheckpointError(f"opened path is not regular: {target}")
        with os.fdopen(descriptor, "rb") as stream:
            descriptor = -1
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    finally:
        if descriptor >= 0:
            os.close(descriptor)
    after = _regular_file_status(target)
    stable = ("st_dev", "st_ino", "st_size", "st_mtime_ns")
    if any(
        getattr(before, name, None) != getattr(after, name, None) for name in stable
    ):
        raise ZeroCheckpointError(f"file changed while hashing: {target}")
    return digest.hexdigest()


def artifact_record(path: str | Path) -> dict[str, Any]:
    target = Path(path)
    before = _regular_file_status(target)
    digest = _sha256_file(target)
    after = _regular_file_status(target)
    if before.st_size != after.st_size:
        raise ZeroCheckpointError(f"file changed while recording: {target}")
    return {
        "path": repo_relative(target),
        "sha256": digest,
        "size_bytes": int(after.st_size),
    }


def tree_record(root: str | Path) -> dict[str, Any]:
    directory = Path(root)
    reject_link_or_reparse_ancestors(directory, include_leaf=True)
    try:
        status = os.lstat(directory)
    except OSError as exc:
        raise ZeroCheckpointError(f"artifact tree is missing: {directory}") from exc
    if not stat.S_ISDIR(status.st_mode) or _is_link_or_reparse(directory):
        raise ZeroCheckpointError(f"artifact tree is unsafe: {directory}")
    paths: list[Path] = []
    for current_text, directory_names, file_names in os.walk(
        directory, topdown=True, followlinks=False
    ):
        current = Path(current_text)
        directory_names.sort()
        file_names.sort()
        for name in directory_names:
            child = current / name
            if _is_link_or_reparse(child) or not stat.S_ISDIR(os.lstat(child).st_mode):
                raise ZeroCheckpointError(f"unsafe tree directory: {child}")
        for name in file_names:
            child = current / name
            if _is_link_or_reparse(child) or not stat.S_ISREG(os.lstat(child).st_mode):
                raise ZeroCheckpointError(f"unsafe tree file: {child}")
            paths.append(child)
    if not paths:
        raise ZeroCheckpointError(f"artifact tree is empty: {directory}")
    digest = hashlib.sha256()
    rows: list[dict[str, Any]] = []
    for path in sorted(paths, key=lambda item: item.relative_to(directory).as_posix()):
        relative = path.relative_to(directory).as_posix()
        record = artifact_record(path)
        row = {
            "path": relative,
            "sha256": record["sha256"],
            "size_bytes": record["size_bytes"],
        }
        rows.append(row)
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(str(row["sha256"]).encode("ascii"))
        digest.update(b"\0")
        digest.update(str(row["size_bytes"]).encode("ascii"))
        digest.update(b"\n")
    return {
        "path": repo_relative(directory),
        "tree_sha256": digest.hexdigest(),
        "file_count": len(rows),
        "files": rows,
    }


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
        raise ZeroCheckpointError(f"invalid strict JSON: {target}") from exc
    if not isinstance(value, Mapping):
        raise ZeroCheckpointError(f"JSON root is not an object: {target}")
    _require_finite_json(value, os.fspath(target))
    return dict(value)


def write_json_exclusive(path: str | Path, payload: Mapping[str, Any]) -> Path:
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
        raise ZeroCheckpointError(
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


def closure_snapshot() -> dict[str, Any]:
    return {
        "sources": {name: artifact_record(path) for name, path in SOURCE_PATHS.items()},
        "inputs": {name: artifact_record(path) for name, path in INPUT_PATHS.items()},
        "candidate_module": tree_record(CANDIDATE_DIR),
    }


def verify_closure(snapshot: Mapping[str, Any]) -> dict[str, Any]:
    observed = closure_snapshot()
    if _canonical_json_bytes(observed) != _canonical_json_bytes(snapshot):
        raise ZeroCheckpointError("locked source/input/candidate closure drifted")
    return observed


def _zero_int(value: Any) -> bool:
    return type(value) is int and value == 0


def _one_int(value: Any) -> bool:
    return type(value) is int and value == 1


def _candidate_binding(payload: Mapping[str, Any], label: str) -> tuple[str, Any]:
    candidate_id = payload.get("candidate_id")
    candidate_module = payload.get("candidate_module")
    if not isinstance(candidate_id, str) or not isinstance(candidate_module, Mapping):
        raise ZeroCheckpointError(f"{label} does not bind a candidate")
    return candidate_id, dict(candidate_module)


def _validate_candidate_tree() -> tuple[str, dict[str, Any]]:
    module = tree_record(CANDIDATE_DIR)
    if (
        module["path"] != contract.CANDIDATE_MODULE_PATH.as_posix()
        or module["file_count"] != len(contract.CANDIDATE_REQUIRED_FILES)
        or {row["path"] for row in module["files"]}
        != set(contract.CANDIDATE_REQUIRED_FILES)
    ):
        raise ZeroCheckpointError("candidate is not the exact five-file R7 tree")
    return contract.candidate_id_for_tree(module["tree_sha256"]), module


def _validate_actor_manifest(
    candidate_id: str, candidate_module: Mapping[str, Any]
) -> dict[str, Any]:
    path = CANDIDATE_DIR / "actor_feature_manifest.json"
    manifest = strict_json(path)
    rows = {row["path"]: row for row in candidate_module["files"]}
    state_row = rows["module_state.pkl"]
    expected_keys = set(contract.q3.ACTOR_FEATURE_MANIFEST_KEYS)
    if (
        set(manifest) != expected_keys
        or manifest.get("schema_version") != 1
        or manifest.get("status") != contract.r7.ACTOR_FEATURE_MANIFEST_STATUS
        or manifest.get("topology_id") != contract.r7.TOPOLOGY_ID
        or manifest.get("fit_contract_id") != contract.r7.FIT_CONTRACT_ID
        or manifest.get("actor_feature_count") != contract.EXPECTED_ACTOR_FEATURES
        or manifest.get("actor_feature_names") != list(contract.q3.ACTOR_FEATURE_NAMES)
        or manifest.get("fcnet_hiddens") != list(contract.EXPECTED_HIDDENS)
        or manifest.get("disabled_clock_columns")
        != list(contract.DISABLED_CLOCK_COLUMNS)
        or manifest.get("module_state_sha256") != state_row["sha256"]
        or not isinstance(manifest.get("actor_digest"), str)
        or len(manifest["actor_digest"]) != 64
        or candidate_id
        != contract.candidate_id_for_tree(candidate_module["tree_sha256"])
    ):
        raise ZeroCheckpointError("candidate actor manifest drifted from W512 ABI")
    return manifest


def _validate_r7_terminal(
    candidate_id: str, candidate_module: Mapping[str, Any]
) -> dict[str, Any]:
    freeze = strict_json(INPUT_PATHS["r7_candidate_freeze"])
    final = strict_json(INPUT_PATHS["r7_final_development"])
    ledger = strict_json(INPUT_PATHS["r7_terminal_ledger"])
    for payload, status, label in (
        (freeze, contract.r7.CANDIDATE_FREEZE_PASS_STATUS, "R7 candidate freeze"),
        (final, contract.r7.DEVELOPMENT_PASS_STATUS, "R7 final development"),
        (ledger, contract.r7.PIPELINE_TERMINAL_PASS_STATUS, "R7 terminal ledger"),
    ):
        bound_id, bound_module = _candidate_binding(payload, label)
        if (
            payload.get("status") != status
            or payload.get("passed") is not True
            or payload.get("protocol_id") != contract.r7.PROTOCOL_ID
            or bound_id != candidate_id
            or _canonical_json_bytes(bound_module)
            != _canonical_json_bytes(candidate_module)
        ):
            raise ZeroCheckpointError(f"{label} is not the exact terminal PASS")
    if (
        freeze.get("candidate_selection_rule") != contract.CANDIDATE_SELECTION_RULE
        or freeze.get("candidate_frozen") is not True
        or freeze.get("fit_gate_passed") is not True
        or freeze.get("standard_actor") is not True
        or freeze.get("warm_start_target_512_compatible") is not True
        or not _one_int(freeze.get("actor_fit_count"))
        or not _one_int(freeze.get("actor_updates"))
        or not _zero_int(freeze.get("critic_updates"))
        or not _zero_int(freeze.get("ppo_updates"))
        or freeze.get("q3_paths_opened") != []
        or freeze.get("runtime_promoted") is not False
    ):
        raise ZeroCheckpointError("R7 candidate-freeze semantics drifted")
    if (
        not _one_int(final.get("actor_fit_count"))
        or not _one_int(final.get("actor_updates"))
        or final.get("development_rollout_count")
        != len(contract.r7.DEVELOPMENT_CASE_IDS)
        or not _zero_int(final.get("critic_updates"))
        or not _zero_int(final.get("ppo_updates"))
        or final.get("qualification_executed") is not False
        or final.get("runtime_promoted") is not False
        or final.get("checkpoint_zero_created") is not False
        or final.get("positive_morphology_enabled") is not False
    ):
        raise ZeroCheckpointError("R7 final-development semantics drifted")
    if (
        ledger.get("terminal") is not True
        or ledger.get("error") is not None
        or ledger.get("candidate_selection_rule") != contract.CANDIDATE_SELECTION_RULE
        or ledger.get("candidate_freeze")
        != artifact_record(INPUT_PATHS["r7_candidate_freeze"])
        or ledger.get("final_development_receipt")
        != artifact_record(INPUT_PATHS["r7_final_development"])
        or not _one_int(ledger.get("actor_fit_count"))
        or not _one_int(ledger.get("actor_updates"))
        or not _zero_int(ledger.get("critic_updates"))
        or not _zero_int(ledger.get("ppo_updates"))
        or ledger.get("qualification_executed") is not False
        or ledger.get("runtime_promoted") is not False
        or ledger.get("checkpoint_zero_created") is not False
        or ledger.get("positive_morphology_enabled") is not False
        or ledger.get("next_stage") != "WAIT_SEPARATE_V12R7Q3_PROTOCOL"
    ):
        raise ZeroCheckpointError("R7 terminal-ledger semantics drifted")
    return {
        "status": contract.r7.PIPELINE_TERMINAL_PASS_STATUS,
        "passed": True,
        "candidate_id": candidate_id,
        "candidate_module": copy.deepcopy(dict(candidate_module)),
        "candidate_freeze": artifact_record(INPUT_PATHS["r7_candidate_freeze"]),
        "final_development": artifact_record(INPUT_PATHS["r7_final_development"]),
        "terminal_ledger": artifact_record(INPUT_PATHS["r7_terminal_ledger"]),
    }


def _validate_q3_terminal(
    candidate_id: str, candidate_module: Mapping[str, Any]
) -> dict[str, Any]:
    receipt = strict_json(INPUT_PATHS["q3_final_receipt"])
    ledger = strict_json(INPUT_PATHS["q3_terminal_ledger"])
    for payload, status, label in (
        (receipt, contract.q3.AGGREGATE_PASS_STATUS, "Q3 final receipt"),
        (ledger, contract.q3.PIPELINE_TERMINAL_PASS_STATUS, "Q3 terminal ledger"),
    ):
        bound_id, bound_module = _candidate_binding(payload, label)
        if (
            payload.get("status") != status
            or payload.get("passed") is not True
            or payload.get("protocol_id") != contract.q3.PROTOCOL_ID
            or bound_id != candidate_id
            or _canonical_json_bytes(bound_module)
            != _canonical_json_bytes(candidate_module)
            or not _zero_int(payload.get("actor_updates"))
            or not _zero_int(payload.get("critic_updates"))
            or not _zero_int(payload.get("ppo_updates"))
            or payload.get("checkpoint_zero_created") is not False
            or payload.get("morphology_weight") != 0.0
            or payload.get("positive_morphology_enabled") is not False
            or payload.get("runtime_promoted") is not False
            or payload.get("next_stage") != contract.q3.NEXT_STAGE_AFTER_Q3_PASS
        ):
            raise ZeroCheckpointError(f"{label} semantic PASS drifted")
    if ledger.get("terminal") is not True or ledger.get("error") is not None:
        raise ZeroCheckpointError("Q3 terminal ledger is not a clean terminal PASS")
    return {
        "status": contract.q3.PIPELINE_TERMINAL_PASS_STATUS,
        "passed": True,
        "candidate_id": candidate_id,
        "candidate_module": copy.deepcopy(dict(candidate_module)),
        "protocol_freeze": artifact_record(INPUT_PATHS["q3_protocol_freeze"]),
        "execution_lock": artifact_record(INPUT_PATHS["q3_execution_lock"]),
        "final_receipt": artifact_record(INPUT_PATHS["q3_final_receipt"]),
        "terminal_ledger": artifact_record(INPUT_PATHS["q3_terminal_ledger"]),
    }


def validate_runtime_target_config() -> dict[str, Any]:
    """Validate the V26 binary-active, causal corridor weight-zero YAML."""

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
        target = INPUT_PATHS["morphology_config"]
        _regular_file_status(target)
        with target.open("r", encoding="utf-8") as stream:
            value = yaml.load(stream, Loader=UniqueKeyLoader)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        raise ZeroCheckpointError("invalid V26 morphology target config") from exc
    if not isinstance(value, Mapping):
        raise ZeroCheckpointError("V26 morphology target root is not a mapping")
    candidate = value.get("candidate")
    grf = value.get("grf")
    reward = value.get("reward")
    if not all(isinstance(item, Mapping) for item in (candidate, grf, reward)):
        raise ZeroCheckpointError("V26 target sections are missing")
    expected_grf = {
        "phase_fsm_input_mode": "legacy_events",
        "event_contract_id": contract.LEGACY_EVENT_CONTRACT_ID,
        "binary_phase_fsm_mode": "binary_active",
        "binary_phase_detector_profile": contract.DETECTOR_PROFILE_PATH.as_posix(),
        "binary_phase_detector_profile_sha256": contract.q3.DETECTOR_PROFILE_SHA256,
        "binary_phase_event_contract_id": contract.BINARY_EVENT_CONTRACT_ID,
        "binary_phase_debounce_s": 0.005,
        "detector_sample_dt_s": 0.001,
        "policy_step_s": 0.01,
    }
    yaml_reward = {
        key: value
        for key, value in contract.TARGET_REWARD_CONFIG.items()
        if key != "morphology_experimental_allow_effects"
    }
    if any(grf.get(key) != expected for key, expected in expected_grf.items()):
        raise ZeroCheckpointError("V26 detector/FSM config drifted")
    if any(reward.get(key) != expected for key, expected in yaml_reward.items()):
        raise ZeroCheckpointError("V26 morphology reward config drifted")
    if (
        _sha256_file(INPUT_PATHS["detector_profile"])
        != contract.q3.DETECTOR_PROFILE_SHA256
        or _sha256_file(INPUT_PATHS["morphology_profile"])
        != contract.q3.MORPHOLOGY_PROFILE_SHA256
        or _sha256_file(INPUT_PATHS["morphology_config"])
        != contract.q3.MORPHOLOGY_CONFIG_SHA256
    ):
        raise ZeroCheckpointError("V26 detector/morphology artifact hash drifted")
    _regular_file_status(INPUT_PATHS["training_config"])
    return {
        "grf": dict(grf),
        "reward": copy.deepcopy(contract.TARGET_REWARD_CONFIG),
        "positive_structure": {
            "compatible": True,
            "authorized_now": False,
            "future_weights": list(contract.POSITIVE_MORPHOLOGY_WEIGHTS),
            "future_causal_allow_effects": 1.0,
        },
    }


def validate_prerequisites() -> dict[str, Any]:
    """Semantically bind clean terminal R7 and Q3 PASS artifacts."""

    if contract.CANDIDATE_ID is not None or contract.CANDIDATE_MODULE is not None:
        raise ZeroCheckpointError("source contract candidate must remain deferred")
    candidate_id, candidate_module = _validate_candidate_tree()
    actor_manifest = _validate_actor_manifest(candidate_id, candidate_module)
    r7_audit = _validate_r7_terminal(candidate_id, candidate_module)
    q3_audit = _validate_q3_terminal(candidate_id, candidate_module)
    runtime = validate_runtime_target_config()
    return {
        "passed": True,
        "candidate_id": candidate_id,
        "candidate_module": candidate_module,
        "actor_manifest": actor_manifest,
        "r7": r7_audit,
        "q3": q3_audit,
        "runtime_target": runtime,
    }


def verify_lock() -> dict[str, Any]:
    lock = strict_json(LOCK)
    if set(lock) != LOCK_KEYS:
        raise ZeroCheckpointError("execution lock schema drifted")
    candidate = lock.get("candidate_module")
    tree_hash = candidate.get("tree_sha256") if isinstance(candidate, Mapping) else ""
    exact = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.LOCK_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "qualification_protocol_id": contract.QUALIFICATION_PROTOCOL_ID,
        "revision": contract.REVISION,
        "candidate_binding_state": "BOUND_AFTER_R7_AND_Q3_TERMINAL_PASS",
        "candidate_id": contract.candidate_id_for_tree(tree_hash),
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "source_topology_id": contract.SOURCE_TOPOLOGY_ID,
        "target_contract_id": contract.TARGET_CONTRACT_ID,
        "target_fixed_config": contract.TARGET_FIXED_CONFIG,
        "target_reward_config": contract.TARGET_REWARD_CONFIG,
        "output_root": repo_relative(OUTPUT_ROOT),
        "authority": contract.AUTHORITY,
        "required_checks": list(contract.REQUIRED_CHECKS),
        "actor_transplants": 1,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "environment_samples": 0,
    }
    for key, expected in exact.items():
        if _canonical_json_bytes(lock.get(key)) != _canonical_json_bytes(expected):
            raise ZeroCheckpointError(f"execution lock drifted at {key}")
    closure = {
        "sources": lock.get("sources"),
        "inputs": lock.get("inputs"),
        "candidate_module": candidate,
    }
    verify_closure(closure)
    prerequisites = validate_prerequisites()
    if _canonical_json_bytes(lock.get("prerequisite_audit")) != _canonical_json_bytes(
        prerequisites
    ):
        raise ZeroCheckpointError("execution lock prerequisite audit drifted")
    return lock


def _as_zero(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise ZeroCheckpointError(f"{label} is not numeric")
    numeric = float(value)
    if not math.isfinite(numeric) or numeric != 0.0:
        raise ZeroCheckpointError(f"{label} is not zero: {value!r}")
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
        raise ZeroCheckpointError("Algorithm metrics logger is unavailable")
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
        raise ZeroCheckpointError("zero progress schema drifted")
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
    raise ZeroCheckpointError("learner module is unavailable")


def _json_option(value: Any, label: str) -> Any:
    if value is None or type(value) in {bool, str, int}:
        return value
    if isinstance(value, numbers.Real):
        numeric = float(value)
        if not math.isfinite(numeric):
            raise ZeroCheckpointError(f"non-finite optimizer option: {label}")
        return numeric
    if isinstance(value, (list, tuple)):
        return [_json_option(item, label) for item in value]
    raise ZeroCheckpointError(f"unsupported optimizer option {label}: {type(value)}")


def optimizer_snapshot_on_learner(learner: Any) -> dict[str, Any]:
    """Prove empty optimizer state and one-to-one parameter registration."""

    module = _learner_module(learner)
    named = list(module.named_parameters())
    if not named or len({name for name, _ in named}) != len(named):
        raise ZeroCheckpointError("learner named-parameter surface is invalid")
    trainable = {
        id(parameter): name for name, parameter in named if parameter.requires_grad
    }
    if not trainable:
        raise ZeroCheckpointError("target learner exposes no trainable parameters")
    seen: dict[int, str] = {}
    optimizers: list[dict[str, Any]] = []
    optimizer_names: set[str] = set()
    for optimizer_name, optimizer in learner.get_optimizers_for_module(
        contract.DEFAULT_POLICY_ID
    ):
        name = str(optimizer_name)
        if not name or name in optimizer_names:
            raise ZeroCheckpointError("optimizer names are empty or duplicated")
        optimizer_names.add(name)
        state = optimizer.state_dict()
        state_entries = state.get("state") if isinstance(state, Mapping) else None
        groups = getattr(optimizer, "param_groups", None)
        if not isinstance(state_entries, Mapping) or state_entries:
            raise ZeroCheckpointError(f"optimizer {name} state is not empty")
        if not isinstance(groups, list) or not groups:
            raise ZeroCheckpointError(f"optimizer {name} has no parameter groups")
        group_rows: list[dict[str, Any]] = []
        for group_index, group in enumerate(groups):
            if not isinstance(group, Mapping) or not isinstance(
                group.get("params"), list
            ):
                raise ZeroCheckpointError(f"optimizer {name} group is malformed")
            parameter_names: list[str] = []
            for parameter in group["params"]:
                identity = id(parameter)
                parameter_name = trainable.get(identity)
                if parameter_name is None:
                    raise ZeroCheckpointError(
                        f"optimizer {name} contains unknown/frozen parameter"
                    )
                if identity in seen:
                    raise ZeroCheckpointError(
                        f"optimizer parameter duplicated across groups: {parameter_name}"
                    )
                seen[identity] = f"{name}:{group_index}"
                parameter_names.append(parameter_name)
            group_rows.append(
                {
                    "group_index": group_index,
                    "parameter_names": parameter_names,
                    "options": {
                        key: _json_option(value, f"{name}.{key}")
                        for key, value in sorted(group.items())
                        if key != "params"
                    },
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
        raise ZeroCheckpointError("target learner has no optimizer")
    missing = sorted(
        name for identity, name in trainable.items() if identity not in seen
    )
    if missing:
        raise ZeroCheckpointError(
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
        raise ZeroCheckpointError("optimizer audit returned no learner evidence")
    return reports


ACTOR_STATE_SHAPES = {
    "pi_encoder.0.weight": (512, 35),
    "pi_encoder.0.bias": (512,),
    "pi_encoder.2.weight": (512, 512),
    "pi_encoder.2.bias": (512,),
    "pi.0.0.weight": (512, 35),
    "pi.0.0.bias": (512,),
    "pi.0.2.weight": (512, 512),
    "pi.0.2.bias": (512,),
    "pi.1.weight": (4, 512),
    "pi.1.bias": (4,),
}


def _array(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    return np.ascontiguousarray(np.asarray(value))


def _bytes_equal(left: Any, right: Any) -> bool:
    a = _array(left)
    b = _array(right)
    return a.dtype == b.dtype and a.shape == b.shape and a.tobytes() == b.tobytes()


def _positive_zero(value: Any) -> bool:
    array = _array(value)
    if array.dtype == np.dtype(np.float32):
        return bool(np.all(array.view(np.uint32) == 0))
    if array.dtype == np.dtype(np.float64):
        return bool(np.all(array.view(np.uint64) == 0))
    return False


def validate_candidate_actor_state(
    state: Mapping[str, Any], warm_start: Any
) -> dict[str, Any]:
    if set(state) != set(ACTOR_STATE_SHAPES):
        raise ZeroCheckpointError("candidate is not an actor-only standard state")
    arrays = {name: _array(state[name]) for name in ACTOR_STATE_SHAPES}
    if any(
        arrays[name].dtype != np.dtype(np.float32)
        or arrays[name].shape != expected
        or not np.all(np.isfinite(arrays[name]))
        for name, expected in ACTOR_STATE_SHAPES.items()
    ):
        raise ZeroCheckpointError("candidate actor dtype/shape/finiteness drifted")
    for left, right in (
        ("pi_encoder.0.weight", "pi.0.0.weight"),
        ("pi_encoder.0.bias", "pi.0.0.bias"),
        ("pi_encoder.2.weight", "pi.0.2.weight"),
        ("pi_encoder.2.bias", "pi.0.2.bias"),
    ):
        if not _bytes_equal(arrays[left], arrays[right]):
            raise ZeroCheckpointError("candidate encoder aliases drifted")
    if not _positive_zero(
        arrays["pi_encoder.0.weight"][:, contract.DISABLED_CLOCK_COLUMNS]
    ) or not _positive_zero(arrays["pi.1.weight"][2:]):
        raise ZeroCheckpointError("candidate clock/logstd zero contract drifted")
    sigma = np.exp(arrays["pi.1.bias"][2:].astype(np.float64))
    if not np.allclose(
        sigma,
        np.repeat(contract.EXPECTED_SIGMA, contract.EXPECTED_ACTION_DIM),
        rtol=0.0,
        atol=1.0e-9,
    ):
        raise ZeroCheckpointError("candidate exploration sigma drifted")
    return {
        "actor_only": True,
        "actor_feature_count": 35,
        "hidden_dims": [512, 512],
        "action_dim": 2,
        "disabled_clock_columns_bit_zero": True,
        "logstd_weight_bit_zero": True,
        "sigma": sigma.astype(float).tolist(),
        "actor_digest": warm_start.actor_state_digest(state),
    }


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
        raise ZeroCheckpointError("target critic is not fresh and source-independent")
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
    validate_candidate_actor_state(candidate_state, warm_start)
    merged = {
        key: value.clone() if hasattr(value, "clone") else copy.deepcopy(value)
        for key, value in target_state.items()
    }
    for key, expected_shape in ACTOR_STATE_SHAPES.items():
        if key not in merged or _array(merged[key]).shape != expected_shape:
            raise ZeroCheckpointError(f"target actor topology drifted at {key}")
        value = candidate_state[key]
        merged[key] = value.clone() if hasattr(value, "clone") else copy.deepcopy(value)
    actor = warm_start.compare_actor_states(candidate_state, merged)
    critic = warm_start.compare_non_actor_states(target_state, merged)
    if not actor["exact"] or not critic["exact"]:
        raise ZeroCheckpointError("actor transplant was not exact/critic preserving")
    return merged, {
        "topology": contract.SOURCE_TOPOLOGY_ID,
        "candidate_actor_digest": warm_start.actor_state_digest(candidate_state),
        "actor": actor,
        "fresh_critic_preserved_byte_exact": critic,
        "source_actor_key_count": len(ACTOR_STATE_SHAPES),
        "source_non_actor_key_count": 0,
        "actor_transplants": 1,
        "actor_updates": 0,
    }


def _actor_surfaces(
    *,
    algo: Any,
    expected_actor: Mapping[str, Any],
    expected_full_state: Mapping[str, Any],
    train_module: Any,
    warm_start: Any,
    timeout_s: float,
) -> dict[str, Any]:
    from ray.rllib.algorithms.algorithm import COMPONENT_RL_MODULE

    local_state = algo.get_module(contract.DEFAULT_POLICY_ID).get_state()
    learner_state = train_module._learner_module_state(algo)  # noqa: SLF001
    local_actor = warm_start.compare_actor_states(expected_actor, local_state)
    learner_actor = warm_start.compare_actor_states(expected_actor, learner_state)
    local_critic = warm_start.compare_non_actor_states(expected_full_state, local_state)
    learner_critic = warm_start.compare_non_actor_states(
        expected_full_state, learner_state
    )
    if not all(
        item["exact"]
        for item in (local_actor, learner_actor, local_critic, learner_critic)
    ):
        raise ZeroCheckpointError("local/learner actor or critic surface drifted")
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
            raise ZeroCheckpointError(f"EnvRunner {index} has no actor state")
        comparison = warm_start.compare_actor_states(expected_actor, actor_state)
        if not comparison["exact"]:
            raise ZeroCheckpointError(f"EnvRunner {index} actor surface drifted")
        runner_reports.append(comparison)
    if not runner_reports:
        raise ZeroCheckpointError("no local EnvRunner actor surface was audited")
    return {
        "local_actor": local_actor,
        "learner_actor": learner_actor,
        "local_critic": local_critic,
        "learner_critic": learner_critic,
        "env_runners": runner_reports,
    }


def _export_module(
    *,
    algo: Any,
    destination: Path,
    expected_actor: Mapping[str, Any],
    expected_full_state: Mapping[str, Any],
    feature_names: Sequence[str],
    lock: Mapping[str, Any],
    warm_start: Any,
) -> dict[str, Any]:
    if os.path.lexists(destination):
        raise ZeroCheckpointError(f"refusing to clobber export: {destination}")
    reject_link_or_reparse_ancestors(destination, include_leaf=False)
    algo.get_module(contract.DEFAULT_POLICY_ID).save_to_path(destination)
    exported = warm_start.load_module_state(destination)
    actor = warm_start.compare_actor_states(expected_actor, exported)
    critic = warm_start.compare_non_actor_states(expected_full_state, exported)
    if not actor["exact"] or not critic["exact"]:
        raise ZeroCheckpointError("exported actor/critic differs from live zero module")
    manifest = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_V12R7_Q3_ZERO_CHECKPOINT_MODULE_EXPORT",
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
        "fresh_critic_preserved_byte_exact": True,
        "morphology_weight": 0.0,
        "execution_lock": artifact_record(LOCK),
        "actor_transplants": 1,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "environment_samples": 0,
    }
    manifest_path = write_json_exclusive(
        destination / "actor_feature_manifest.json", manifest
    )
    return {
        "actor": actor,
        "critic": critic,
        "manifest": artifact_record(manifest_path),
        "tree": tree_record(destination),
    }


def _assert_full_checkpoint(path: Path) -> dict[str, Any]:
    tree = tree_record(path)
    relative_files = {row["path"] for row in tree["files"]}
    missing = sorted(contract.CHECKPOINT_REQUIRED_SUFFIXES - relative_files)
    if missing:
        raise ZeroCheckpointError(f"full RLlib checkpoint is incomplete: {missing}")
    return tree


def _target_training_args(
    train_module: Any,
    output_dir: Path,
    *,
    morphology_weight: float,
    positive_authorized: bool,
) -> tuple[Any, dict[str, Any]]:
    target = validate_runtime_target_config()
    argv = [
        os.fspath(train_module.__file__),
        "--config",
        os.fspath(INPUT_PATHS["training_config"]),
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
        "--num-hidden-layers",
        "2",
        "--dim-hidden-layers",
        "512",
        "--freeze-logstd",
        "--phase-fsm-input-mode",
        "legacy_events",
        "--event-contract-id",
        contract.LEGACY_EVENT_CONTRACT_ID,
        "--binary-phase-fsm-mode",
        "binary_active",
        "--binary-phase-detector-profile",
        contract.DETECTOR_PROFILE_PATH.as_posix(),
        "--detector-sample-dt-s",
        "0.001",
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
        raise ZeroCheckpointError(f"resolved fixed target config drifted: {observed}")
    if morphology_weight == 0.0:
        allow_effects = 0.0
    elif (
        positive_authorized is True
        and morphology_weight in contract.POSITIVE_MORPHOLOGY_WEIGHTS
    ):
        allow_effects = 1.0
    else:
        raise ZeroCheckpointError("positive morphology config lacks separate authority")
    reward = dict(getattr(args, "_cfg_reward", None) or {})
    reward.update(target["reward"])
    reward["morphology_weight"] = float(morphology_weight)
    reward["morphology_causal_allow_effects"] = allow_effects
    args._cfg_reward = reward
    return args, reward


def _build_algorithm(train_module: Any, args: Any, reward: Mapping[str, Any]) -> Any:
    config = train_module.build_config(args, dict(reward))
    return config.build_algo()


def training_resume_source_order_audit(train_module: Any) -> dict[str, Any]:
    """Bind the live-config-before-restore ordering in the training entrypoint."""

    source = Path(train_module.__file__).read_text(encoding="utf-8")
    yaml_reward_at = source.find(
        'reward_overrides = dict(getattr(args, "_cfg_reward", None) or {})'
    )
    json_reward_at = source.find(
        "json_overrides = reward_function.load_reward_overrides(args.reward_json)",
        yaml_reward_at,
    )
    merge_reward_at = source.find(
        "reward_overrides.update(json_overrides)", json_reward_at
    )
    build_config_at = source.find("config = build_config(args, reward_overrides)")
    build_algo_at = source.find("algo = config.build_algo()", build_config_at)
    restore_at = source.find("algo.restore_from_path(resume_path)", build_algo_at)
    passed = (
        0
        <= yaml_reward_at
        < json_reward_at
        < merge_reward_at
        < build_config_at
        < build_algo_at
        < restore_at
    )
    if not passed:
        raise ZeroCheckpointError(
            "training entrypoint no longer builds live config before restore"
        )
    return {
        "passed": True,
        "reward_json_merged_before_build_config": True,
        "build_config_before_build_algo": True,
        "build_algo_before_restore_from_path": True,
        "training_entrypoint": artifact_record(train_module.__file__),
    }


def live_config_snapshot(
    algo: Any,
    *,
    expected_reward: Mapping[str, Any],
) -> dict[str, Any]:
    config = getattr(algo, "config", None)
    env = getattr(config, "env_config", None)
    if not isinstance(env, Mapping):
        raise ZeroCheckpointError("Algorithm has no live env_config mapping")
    reward = env.get("reward")
    if not isinstance(reward, Mapping):
        raise ZeroCheckpointError("Algorithm live env_config has no reward mapping")
    env_expected = {
        "phase_fsm_input_mode": "legacy_events",
        "event_contract_id": contract.LEGACY_EVENT_CONTRACT_ID,
        "binary_phase_fsm_mode": "binary_active",
        "binary_phase_detector_profile_file": contract.DETECTOR_PROFILE_PATH.as_posix(),
        "detector_sample_dt_s": 0.001,
        "binary_phase_debounce_s": 0.005,
        "binary_phase_event_contract_id": contract.BINARY_EVENT_CONTRACT_ID,
    }
    observed_env = {key: env.get(key) for key in env_expected}
    expected_corridor = {
        key: expected_reward[key] for key in contract.TARGET_REWARD_CONFIG
    }
    observed_reward = {key: reward.get(key) for key in expected_corridor}
    if observed_env != env_expected or observed_reward != expected_corridor:
        raise ZeroCheckpointError("Algorithm live V26/morphology config drifted")
    return {
        "env": observed_env,
        "reward": observed_reward,
        "full_reward_config_sha256": hashlib.sha256(
            _canonical_json_bytes(dict(reward))
        ).hexdigest(),
        "morphology_weight": float(reward["morphology_weight"]),
        "morphology_causal_allow_effects": float(
            reward["morphology_causal_allow_effects"]
        ),
    }


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
    source_order = training_resume_source_order_audit(train)
    train._load_training_stack()  # noqa: SLF001
    ray = train.ray
    zero_args, zero_reward = _target_training_args(
        train,
        OUTPUT_ROOT / "rllib_zero",
        morphology_weight=0.0,
        positive_authorized=False,
    )
    positive_weight = contract.POSITIVE_MORPHOLOGY_WEIGHTS[0]
    positive_args, positive_reward = _target_training_args(
        train,
        OUTPUT_ROOT / "rllib_positive_restore_smoke",
        morphology_weight=positive_weight,
        positive_authorized=True,
    )
    training_config.dump_resolved(
        zero_args,
        zero_reward,
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
    positive = None
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
        candidate_audit = validate_candidate_actor_state(candidate_state, warm_start)
        if (
            candidate_audit["actor_digest"]
            != lock["prerequisite_audit"]["actor_manifest"]["actor_digest"]
        ):
            raise ZeroCheckpointError("candidate state/manifest actor digest drifted")

        verify_closure(locked_closure)
        first = _build_algorithm(train, zero_args, zero_reward)
        zero_live_config = live_config_snapshot(first, expected_reward=zero_reward)
        progress_before = zero_progress_audit(first)
        optimizer_before = optimizer_audit(train, first)
        feature_names = tuple(getattr(zero_args, "_target_actor_feature_names", ()))
        full_names = tuple(getattr(zero_args, "_target_observation_feature_names", ()))
        if feature_names != tuple(contract.q3.ACTOR_FEATURE_NAMES):
            raise ZeroCheckpointError("target actor feature order is not exact 35")
        if len(full_names) != contract.EXPECTED_FULL_FEATURES:
            raise ZeroCheckpointError("target full observation layout is not exact 84")

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
            timeout_seconds=float(zero_args.startup_timeout_s),
            inference_only=True,
        )
        learner_after = train._learner_module_state(first)  # noqa: SLF001
        critic_preserved = warm_start.compare_non_actor_states(
            learner_before, learner_after
        )
        if not critic_preserved["exact"]:
            raise ZeroCheckpointError("actor transplant changed fresh critic")
        optimizer_after = optimizer_audit(train, first)
        if _canonical_json_bytes(optimizer_after) != _canonical_json_bytes(
            optimizer_before
        ):
            raise ZeroCheckpointError("optimizer groups changed by actor transplant")
        surfaces_before = _actor_surfaces(
            algo=first,
            expected_actor=candidate_state,
            expected_full_state=learner_after,
            train_module=train,
            warm_start=warm_start,
            timeout_s=zero_args.startup_timeout_s,
        )
        progress_after = zero_progress_audit(first)
        if progress_after != progress_before:
            raise ZeroCheckpointError("actor transplant changed progress counters")
        initial_export = _export_module(
            algo=first,
            destination=OUTPUT_ROOT / contract.OUTPUT_NAMES["initial_export"],
            expected_actor=candidate_state,
            expected_full_state=learner_after,
            feature_names=feature_names,
            lock=lock,
            warm_start=warm_start,
        )

        verify_closure(locked_closure)
        checkpoint = OUTPUT_ROOT / contract.OUTPUT_NAMES["checkpoint"]
        if os.path.lexists(checkpoint):
            raise ZeroCheckpointError(f"refusing to clobber checkpoint: {checkpoint}")
        first.save_to_path(checkpoint)
        checkpoint_tree = _assert_full_checkpoint(checkpoint)
        if zero_progress_audit(first) != progress_before:
            raise ZeroCheckpointError("checkpoint save changed zero counters")
        first.stop()
        first = None

        verify_closure(locked_closure)
        restored = _build_algorithm(train, zero_args, zero_reward)
        restored_config_before = live_config_snapshot(
            restored, expected_reward=zero_reward
        )
        if any(zero_progress_audit(restored).values()):
            raise ZeroCheckpointError("fresh restore target already has progress")
        restored.restore_from_path(checkpoint)
        restored_config_after = live_config_snapshot(
            restored, expected_reward=zero_reward
        )
        if restored_config_after != restored_config_before:
            raise ZeroCheckpointError("zero live config changed during restore")
        restored_progress = zero_progress_audit(restored)
        restored_optimizer = optimizer_audit(train, restored)
        if _canonical_json_bytes(restored_optimizer) != _canonical_json_bytes(
            optimizer_before
        ):
            raise ZeroCheckpointError("restored optimizer groups/state drifted")
        restored_learner = train._learner_module_state(restored)  # noqa: SLF001
        restored_critic = warm_start.compare_non_actor_states(
            learner_after, restored_learner
        )
        if not restored_critic["exact"]:
            raise ZeroCheckpointError("restored critic differs from saved fresh critic")
        restored_surfaces = _actor_surfaces(
            algo=restored,
            expected_actor=candidate_state,
            expected_full_state=learner_after,
            train_module=train,
            warm_start=warm_start,
            timeout_s=zero_args.startup_timeout_s,
        )
        restored_export = _export_module(
            algo=restored,
            destination=OUTPUT_ROOT / contract.OUTPUT_NAMES["restored_export"],
            expected_actor=candidate_state,
            expected_full_state=learner_after,
            feature_names=feature_names,
            lock=lock,
            warm_start=warm_start,
        )
        if any(restored_progress.values()):
            raise ZeroCheckpointError("restored Algorithm has non-zero progress")
        restored.stop()
        restored = None

        # Critical handoff smoke: construct the future live positive corridor
        # config first, restore the weight-zero full checkpoint second, and prove
        # the restore does not replace reward/env configuration.
        verify_closure(locked_closure)
        positive = _build_algorithm(train, positive_args, positive_reward)
        positive_config_before = live_config_snapshot(
            positive, expected_reward=positive_reward
        )
        if (
            positive_config_before["morphology_weight"] != positive_weight
            or positive_config_before["morphology_causal_allow_effects"] != 1.0
            or any(zero_progress_audit(positive).values())
        ):
            raise ZeroCheckpointError("positive resume smoke did not start live/zero")
        positive.restore_from_path(checkpoint)
        positive_config_after = live_config_snapshot(
            positive, expected_reward=positive_reward
        )
        if positive_config_after != positive_config_before:
            raise ZeroCheckpointError(
                "checkpoint restore replaced the live positive reward/env config"
            )
        positive_progress = zero_progress_audit(positive)
        positive_optimizer = optimizer_audit(train, positive)
        positive_learner = train._learner_module_state(positive)  # noqa: SLF001
        positive_critic = warm_start.compare_non_actor_states(
            learner_after, positive_learner
        )
        positive_surfaces = _actor_surfaces(
            algo=positive,
            expected_actor=candidate_state,
            expected_full_state=learner_after,
            train_module=train,
            warm_start=warm_start,
            timeout_s=positive_args.startup_timeout_s,
        )
        if (
            any(positive_progress.values())
            or _canonical_json_bytes(positive_optimizer)
            != _canonical_json_bytes(optimizer_before)
            or not positive_critic["exact"]
        ):
            raise ZeroCheckpointError(
                "positive restore smoke changed zero checkpoint state"
            )
        verify_closure(locked_closure)
        return {
            "candidate_id": lock["candidate_id"],
            "candidate_actor_audit": candidate_audit,
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
                "positive_restore_exact": positive_critic,
            },
            "optimizer": {
                "before_transplant": optimizer_before,
                "after_transplant": optimizer_after,
                "after_restore": restored_optimizer,
                "after_positive_restore": positive_optimizer,
            },
            "progress": {
                "before_transplant": progress_before,
                "after_transplant": progress_after,
                "after_restore": restored_progress,
                "after_positive_restore": positive_progress,
            },
            "live_config": {
                "zero_before_save": zero_live_config,
                "zero_before_restore": restored_config_before,
                "zero_after_restore": restored_config_after,
                "positive_before_restore": positive_config_before,
                "positive_after_restore": positive_config_after,
                "positive_restore_preserved_live_config": True,
            },
            "training_source_order": source_order,
            "positive_resume_actor_surfaces": positive_surfaces,
            "checkpoint_tree": checkpoint_tree,
            "initial_export": initial_export,
            "restored_export": restored_export,
        }
    finally:
        for algo in (positive, restored, first):
            if algo is not None:
                try:
                    algo.stop()
                except Exception:
                    pass
        if ray.is_initialized():
            ray.shutdown()


def training_handoff_payload(
    *, checkpoint_tree: Mapping[str, Any], lock: Mapping[str, Any]
) -> dict[str, Any]:
    platforms: dict[str, Any] = {}
    for platform_id in ("macos_arm64", "windows_x86_64"):
        argv = contract.resume_training_argv(
            platform_id=platform_id,
            output_dir="Trajectory Generator/runs/training/v12r7_positive_corridor_0025",
            iterations=50,
            morphology_weight=0.0025,
            positive_morphology_authorized=True,
        )
        platforms[platform_id] = {
            "proposed_resume_argv": list(argv),
            "proposed_resume_command": contract.render_command(argv, platform_id),
        }
    if any(
        "--resume-from" not in row["proposed_resume_argv"]
        or "--warm-start" in row["proposed_resume_argv"]
        or "--warm-start-raw" in row["proposed_resume_argv"]
        for row in platforms.values()
    ):
        raise ZeroCheckpointError("training handoff is not resume-only")
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "DEFERRED_H0_V12R7_ZERO_CHECKPOINT_RESUME_INTERFACE",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": lock["candidate_id"],
        "checkpoint": copy.deepcopy(dict(checkpoint_tree)),
        "initialization_mode": "resume_from_full_checkpoint_zero",
        "required_flag": "--resume-from",
        "forbidden_flags": ["--warm-start", "--warm-start-raw"],
        "target_training_iterations": 50,
        "morphology_weight": 0.0025,
        "morphology_causal_allow_effects": 1.0,
        "positive_live_config_restore_smoke_passed": True,
        "platforms": platforms,
        "training_authorized": False,
        "training_command_published": False,
        "requires_terminal_positive_ab": True,
        "next_stage": contract.NEXT_STAGE_AFTER_ZERO_PASS,
        "training_execution_performed": False,
    }


def _attempt_claim_payload(lock: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.ATTEMPT_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": lock["candidate_id"],
        "execution_lock": artifact_record(LOCK),
        "actor_transplants": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "environment_samples": 0,
    }


def execute() -> dict[str, Any]:
    """Reject execution from the terminally failed historical R7 lineage."""

    if contract.CANONICAL_ENTRYPOINTS_BLOCKED:
        raise ZeroCheckpointError(
            f"{contract.RETIREMENT_REASON}; create and bind "
            f"{contract.SUCCESSOR_ZERO_NAMESPACE}"
        )

    lock = verify_lock()
    if os.path.lexists(OUTPUT_ROOT):
        raise ZeroCheckpointError(f"zero-checkpoint output is claimed: {OUTPUT_ROOT}")
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
            raise ZeroCheckpointError("zero-checkpoint runtime checks are incomplete")
        handoff = training_handoff_payload(
            checkpoint_tree=runtime["checkpoint_tree"], lock=lock
        )
        handoff_path = write_json_exclusive(
            OUTPUT_ROOT / contract.OUTPUT_NAMES["handoff"], handoff
        )
        audit = {
            "schema_version": contract.SCHEMA_VERSION,
            "status": contract.PASS_STATUS,
            "passed": True,
            "protocol_id": contract.PROTOCOL_ID,
            **runtime,
            "training_handoff": artifact_record(handoff_path),
            "actor_transplants": 1,
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
            "environment_samples": 0,
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
            "execution_lock": artifact_record(LOCK),
            "attempt_claim": artifact_record(claim_path),
            "r7_terminal_ledger": artifact_record(INPUT_PATHS["r7_terminal_ledger"]),
            "q3_final_receipt": artifact_record(INPUT_PATHS["q3_final_receipt"]),
            "q3_terminal_ledger": artifact_record(INPUT_PATHS["q3_terminal_ledger"]),
            "audit": artifact_record(audit_path),
            "resolved_config": artifact_record(
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
            "training_handoff": artifact_record(handoff_path),
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
            "runtime_promoted": False,
            "training_executed": False,
            "resume_interface_published": True,
            "training_command_published": False,
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
        "execution_lock": artifact_record(LOCK),
        "attempt_claim": artifact_record(claim_path),
        "receipt_created": (OUTPUT_ROOT / contract.OUTPUT_NAMES["receipt"]).is_file(),
        "actor_transplants": int(runtime is not None),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "environment_samples": 0,
        "runtime_promoted": False,
        "training_executed": False,
        "next_stage": (
            contract.NEXT_STAGE_AFTER_ZERO_PASS if passed else "STOP_TERMINAL_NO_RETRY"
        ),
    }
    write_json_exclusive(OUTPUT_ROOT / contract.OUTPUT_NAMES["ledger"], ledger)
    if not passed:
        raise ZeroCheckpointError(error or contract.FAIL_STATUS)
    return receipt


def verify_terminal_pass() -> dict[str, Any]:
    """Reject terminal PASS claims from the terminally failed R7 lineage."""

    if contract.CANONICAL_ENTRYPOINTS_BLOCKED:
        raise ZeroCheckpointError(
            f"{contract.RETIREMENT_REASON}; no R7 checkpoint-zero can be valid"
        )

    lock = verify_lock()
    receipt = strict_json(resolve_relative(contract.RECEIPT_PATH))
    ledger = strict_json(resolve_relative(contract.TERMINAL_LEDGER_PATH))
    handoff = strict_json(resolve_relative(contract.HANDOFF_PATH))
    audit = strict_json(resolve_relative(contract.AUDIT_PATH))
    checkpoint = _assert_full_checkpoint(resolve_relative(contract.CHECKPOINT_PATH))
    zero_counters = all(
        _zero_int(receipt.get(name)) and _zero_int(ledger.get(name))
        for name in (
            "actor_updates",
            "critic_updates",
            "ppo_updates",
            "environment_samples",
        )
    )
    platforms = handoff.get("platforms")
    platform_commands = (
        isinstance(platforms, Mapping)
        and set(platforms) == {"macos_arm64", "windows_x86_64"}
        and all(
            isinstance(row, Mapping)
            and isinstance(row.get("proposed_resume_argv"), list)
            and "--resume-from" in row["proposed_resume_argv"]
            and "--warm-start" not in row["proposed_resume_argv"]
            and "--warm-start-raw" not in row["proposed_resume_argv"]
            and isinstance(row.get("proposed_resume_command"), str)
            and bool(row["proposed_resume_command"])
            for row in platforms.values()
        )
    )
    checks = {
        "receipt_pass": receipt.get("status") == contract.PASS_STATUS
        and receipt.get("passed") is True
        and receipt.get("protocol_id") == contract.PROTOCOL_ID,
        "ledger_pass": ledger.get("status") == contract.PASS_STATUS
        and ledger.get("passed") is True
        and ledger.get("terminal") is True
        and ledger.get("error") is None
        and ledger.get("next_stage") == contract.NEXT_STAGE_AFTER_ZERO_PASS,
        "candidate": receipt.get("candidate_id") == lock.get("candidate_id")
        and receipt.get("candidate_module") == lock.get("candidate_module")
        and ledger.get("candidate_id") == lock.get("candidate_id"),
        "records": receipt.get("execution_lock") == artifact_record(LOCK)
        and receipt.get("audit")
        == artifact_record(resolve_relative(contract.AUDIT_PATH))
        and receipt.get("resolved_config")
        == artifact_record(resolve_relative(contract.RESOLVED_CONFIG_PATH))
        and receipt.get("training_handoff")
        == artifact_record(resolve_relative(contract.HANDOFF_PATH)),
        "checkpoint": receipt.get("checkpoint") == checkpoint
        and handoff.get("checkpoint") == checkpoint,
        "zero_state": receipt.get("actor_transplants") == 1
        and ledger.get("actor_transplants") == 1
        and zero_counters,
        "no_training": receipt.get("training_executed") is False
        and ledger.get("training_executed") is False
        and receipt.get("runtime_promoted") is False
        and receipt.get("training_command_published") is False,
        "resume_interface": handoff.get("status")
        == "DEFERRED_H0_V12R7_ZERO_CHECKPOINT_RESUME_INTERFACE"
        and handoff.get("passed") is True
        and handoff.get("initialization_mode") == "resume_from_full_checkpoint_zero"
        and handoff.get("required_flag") == "--resume-from"
        and handoff.get("forbidden_flags") == ["--warm-start", "--warm-start-raw"]
        and handoff.get("target_training_iterations") == 50
        and handoff.get("morphology_weight") == 0.0025
        and handoff.get("morphology_causal_allow_effects") == 1.0
        and handoff.get("positive_live_config_restore_smoke_passed") is True
        and handoff.get("training_authorized") is False
        and handoff.get("training_command_published") is False
        and handoff.get("requires_terminal_positive_ab") is True
        and handoff.get("next_stage") == contract.NEXT_STAGE_AFTER_ZERO_PASS
        and platform_commands,
        "positive_config_survival": audit.get("live_config", {}).get(
            "positive_restore_preserved_live_config"
        )
        is True,
    }
    if not all(checks.values()):
        failed = sorted(name for name, value in checks.items() if not value)
        raise ZeroCheckpointError(
            f"terminal checkpoint-zero verification failed: {failed}"
        )
    return receipt


if __name__ == "__main__":
    try:
        print(json.dumps(execute(), indent=2, sort_keys=True, allow_nan=False))
    except Exception as exc:
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
        raise SystemExit(2) from exc
