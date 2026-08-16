"""Latent one-shot runtime for the V12R8/Q3 checkpoint-zero port.

Importing this module is inert.  The live path remains fail-closed until the
stable R8 and Q3 semantic verifiers both return terminal PASS for the same
five-file candidate.  Once an exact no-clobber execution lock exists,
``execute`` builds a fresh W512 Algorithm, transplants only the actor, saves and
reloads a full RLlib checkpoint at progress zero, and performs the sole
positive restore smoke at morphology weight 0.0025.  It never calls train or
samples an environment.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
import numbers
import os
import stat
import sys
import time
from collections.abc import Mapping, Sequence
from pathlib import Path, PurePath, PurePosixPath
from typing import Any

import numpy as np

try:
    from . import h0_v12r8_zero_checkpoint_contract as contract
    from . import h0_v12r8_zero_checkpoint_gates as gates
except ImportError:  # Direct execution with this directory on ``sys.path``.
    import h0_v12r8_zero_checkpoint_contract as contract
    import h0_v12r8_zero_checkpoint_gates as gates


class ZeroCheckpointDeferredError(RuntimeError):
    """Raised when lineage, zero-state, or publication invariants drift."""


REPO_ROOT = Path(__file__).absolute().parents[4]
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
BASELINE_ROOT = TRAJECTORY_ROOT / "baseline_MLP"
LOCAL_VALIDATION_ROOT = BASELINE_ROOT / "validation"
for _root in (
    BASELINE_ROOT,
    TRAJECTORY_ROOT,
    REPO_ROOT,
    REPO_ROOT / "validation",
    LOCAL_VALIDATION_ROOT / "v12r8",
    LOCAL_VALIDATION_ROOT / "v12r8q3",
    LOCAL_VALIDATION_ROOT / "v12r8q3" / "runtime",
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))


def resolve_relative(value: str | PurePath) -> Path:
    """Resolve a canonical repository-relative POSIX path."""

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
        raise ZeroCheckpointDeferredError(f"non-canonical repository path: {value!r}")
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
        "revision",
        "source_state",
        "candidate_binding_state",
        "binding",
        "candidate_selection_rule",
        "source_topology_id",
        "upstream_endpoints",
        "upstream_payload",
        "upstream_gate",
        "actor_manifest",
        "actor_manifest_gate",
        "source_closure",
        "source_closure_gate",
        "target_fixed_config",
        "zero_reward_config",
        "positive_restore_reward_config",
        "profile_attestations",
        "required_audit_checks",
        "actor_transplants",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
        "environment_samples",
        "training_authorized",
        "training_command_published",
        "authority",
    }
)

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
        raise ZeroCheckpointDeferredError("value is not strict finite JSON") from exc


def _require_finite_json(value: Any, label: str) -> None:
    if value is None or type(value) in {bool, str, int}:
        return
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ZeroCheckpointDeferredError(f"{label} contains a non-finite number")
        return
    if isinstance(value, Mapping):
        for key, child in value.items():
            if not isinstance(key, str) or not key:
                raise ZeroCheckpointDeferredError(f"{label} contains an invalid key")
            _require_finite_json(child, f"{label}.{key}")
        return
    if isinstance(value, (list, tuple)):
        for index, child in enumerate(value):
            _require_finite_json(child, f"{label}[{index}]")
        return
    raise ZeroCheckpointDeferredError(f"{label} contains a non-JSON value")


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
        raise ZeroCheckpointDeferredError(
            f"path escapes repository root: {path}"
        ) from exc


def reject_link_or_reparse_ancestors(path: Path, *, include_leaf: bool) -> None:
    parts = _relative_parts(path)
    limit = len(parts) if include_leaf else max(0, len(parts) - 1)
    current = Path(os.path.abspath(REPO_ROOT))
    for part in parts[:limit]:
        current /= part
        if os.path.lexists(current) and _is_link_or_reparse(current):
            raise ZeroCheckpointDeferredError(
                f"unsafe symlink/junction component: {current}"
            )


def repo_relative(path: str | Path) -> str:
    return PurePosixPath(*_relative_parts(Path(path))).as_posix()


def _regular_file_status(path: Path) -> os.stat_result:
    reject_link_or_reparse_ancestors(path, include_leaf=True)
    try:
        status = os.lstat(path)
    except OSError as exc:
        raise ZeroCheckpointDeferredError(f"required file is missing: {path}") from exc
    if not stat.S_ISREG(status.st_mode):
        raise ZeroCheckpointDeferredError(
            f"required path is not a regular file: {path}"
        )
    return status


def _sha256_file(path: str | Path) -> str:
    target = Path(path)
    before = _regular_file_status(target)
    flags = os.O_RDONLY | getattr(os, "O_BINARY", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(target, flags)
    except OSError as exc:
        raise ZeroCheckpointDeferredError(f"cannot safely open file: {target}") from exc
    digest = hashlib.sha256()
    try:
        opened = os.fstat(descriptor)
        if not stat.S_ISREG(opened.st_mode):
            raise ZeroCheckpointDeferredError(f"opened path is not regular: {target}")
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
        raise ZeroCheckpointDeferredError(f"file changed while hashing: {target}")
    return digest.hexdigest()


def artifact_record(path: str | Path) -> dict[str, Any]:
    target = Path(path)
    before = _regular_file_status(target)
    digest = _sha256_file(target)
    after = _regular_file_status(target)
    if before.st_size != after.st_size:
        raise ZeroCheckpointDeferredError(f"file changed while recording: {target}")
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
        raise ZeroCheckpointDeferredError(
            f"artifact tree is missing: {directory}"
        ) from exc
    if not stat.S_ISDIR(status.st_mode) or _is_link_or_reparse(directory):
        raise ZeroCheckpointDeferredError(f"artifact tree is unsafe: {directory}")
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
                raise ZeroCheckpointDeferredError(f"unsafe tree directory: {child}")
        for name in file_names:
            child = current / name
            if _is_link_or_reparse(child) or not stat.S_ISREG(os.lstat(child).st_mode):
                raise ZeroCheckpointDeferredError(f"unsafe tree file: {child}")
            paths.append(child)
    if not paths:
        raise ZeroCheckpointDeferredError(f"artifact tree is empty: {directory}")
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
        digest.update(row["sha256"].encode("ascii"))
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
        raise ZeroCheckpointDeferredError(f"invalid strict JSON: {target}") from exc
    if not isinstance(value, Mapping):
        raise ZeroCheckpointDeferredError(f"JSON root is not an object: {target}")
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
        raise ZeroCheckpointDeferredError(
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
        raise ZeroCheckpointDeferredError(
            "locked source/input/candidate closure drifted"
        )
    return observed


def _mapping(value: Any) -> dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


def _same_json(left: Any, right: Any) -> bool:
    return gates.canonical_json_sha256(left) != "" and gates.canonical_json_sha256(
        left
    ) == gates.canonical_json_sha256(right)


def _zero_int(value: Any) -> bool:
    return type(value) is int and value == 0


def semantic_upstream_payload() -> dict[str, Any]:
    """Invoke the stable live R8 and Q3 terminal semantic verifiers."""

    try:
        import run_h0_v12r8_q3_qualification as q3_runner
        import run_h0_v12r8_recovery as r8_runner

        r8_result = r8_runner.verify_terminal_ledger()
        q3_result = q3_runner.verify_terminal_ledger()
    except Exception as exc:
        raise ZeroCheckpointDeferredError(
            f"R8/Q3 terminal semantic verification failed: {exc}"
        ) from exc
    if not isinstance(r8_result, Mapping) or not isinstance(q3_result, Mapping):
        raise ZeroCheckpointDeferredError(
            "R8/Q3 terminal verifier did not return mappings"
        )
    r8_payload = dict(r8_result)
    q3_payload = dict(q3_result)

    def attestation(
        endpoint: Mapping[str, Any], result: Mapping[str, Any]
    ) -> dict[str, Any]:
        return {
            "endpoint": copy.deepcopy(dict(endpoint)),
            "artifact": artifact_record(resolve_relative(str(endpoint["path"]))),
            "verifier_module": endpoint["verifier_module"],
            "verifier": endpoint["verifier"],
            "verified_result_sha256": gates.canonical_json_sha256(result),
            "verifier_returned_mapping": True,
        }

    payload = {
        "v12r8_terminal": r8_payload,
        "v12r8_q3_terminal": q3_payload,
        "semantic_attestations": {
            "v12r8_terminal": attestation(contract.R8_TERMINAL_ENDPOINT, r8_payload),
            "v12r8_q3_terminal": attestation(contract.Q3_TERMINAL_ENDPOINT, q3_payload),
        },
    }
    gate = gates.upstream_terminal_gate(payload)
    if gate.get("passed") is not True:
        failed = sorted(
            name for name, passed in _mapping(gate.get("checks")).items() if not passed
        )
        raise ZeroCheckpointDeferredError(
            f"R8/Q3 terminal semantic gate failed: {failed}"
        )
    return payload


def candidate_actor_manifest(candidate_module: Mapping[str, Any]) -> dict[str, Any]:
    """Read and validate the actor manifest bound by the candidate tree."""

    manifest = strict_json(CANDIDATE_DIR / "actor_feature_manifest.json")
    gate = gates.actor_manifest_gate(manifest, candidate_module=candidate_module)
    if gate.get("passed") is not True:
        raise ZeroCheckpointDeferredError(
            "candidate actor manifest drifted from the standard W512 ABI"
        )
    return manifest


def verify_lock_payload(
    value: Any,
    *,
    observed_source_closure: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Recompute every semantic lock gate and optional closure observation."""

    lock = _mapping(value)
    upstream = gates.upstream_terminal_gate(lock.get("upstream_payload"))
    binding = _mapping(lock.get("binding"))
    manifest = gates.actor_manifest_gate(
        lock.get("actor_manifest"),
        candidate_module=_mapping(binding.get("candidate_module")),
    )
    closure = gates.source_closure_gate(lock.get("source_closure"))
    closure_inputs = _mapping(_mapping(lock.get("source_closure")).get("inputs"))
    checks = {
        "schema": set(lock) == LOCK_KEYS
        and lock.get("schema_version") == contract.SCHEMA_VERSION
        and lock.get("status") == contract.LOCK_STATUS
        and lock.get("protocol_id") == contract.PROTOCOL_ID
        and lock.get("pipeline_id") == contract.PIPELINE_ID
        and lock.get("revision") == contract.REVISION,
        "source_state": lock.get("source_state") == contract.SOURCE_STATE
        and lock.get("candidate_binding_state")
        == "BOUND_AFTER_R8_AND_Q3_TERMINAL_PASS",
        "upstream": upstream.get("passed") is True
        and _same_json(lock.get("upstream_gate"), upstream)
        and _same_json(lock.get("upstream_endpoints"), contract.UPSTREAM_ENDPOINTS),
        "binding": gates.candidate_binding_valid(binding)
        and binding.get("candidate_id") == upstream.get("candidate_id")
        and _same_json(
            binding.get("candidate_module"), upstream.get("candidate_module")
        ),
        "actor": manifest.get("passed") is True
        and _same_json(lock.get("actor_manifest_gate"), manifest)
        and binding.get("actor_digest")
        == _mapping(lock.get("actor_manifest")).get("actor_digest"),
        "closure": closure.get("passed") is True
        and _same_json(lock.get("source_closure_gate"), closure)
        and _same_json(
            _mapping(lock.get("source_closure")).get("candidate_module"),
            binding.get("candidate_module"),
        ),
        "closure_rehash": observed_source_closure is None
        or _same_json(observed_source_closure, lock.get("source_closure")),
        "candidate_contract": lock.get("candidate_selection_rule")
        == contract.CANDIDATE_SELECTION_RULE
        and lock.get("source_topology_id") == contract.SOURCE_TOPOLOGY_ID,
        "configs": _same_json(
            lock.get("target_fixed_config"), contract.TARGET_FIXED_CONFIG
        )
        and _same_json(lock.get("zero_reward_config"), contract.ZERO_REWARD_CONFIG)
        and _same_json(
            lock.get("positive_restore_reward_config"),
            contract.POSITIVE_RESTORE_REWARD_CONFIG,
        )
        and _same_json(lock.get("profile_attestations"), contract.PROFILE_ATTESTATIONS),
        "profile_closure": all(
            _mapping(closure_inputs.get(name)).get("path") == expected["path"]
            and _mapping(closure_inputs.get(name)).get("sha256") == expected["sha256"]
            for name, expected in contract.PROFILE_ATTESTATIONS.items()
        ),
        "audit_contract": lock.get("required_audit_checks")
        == list(contract.REQUIRED_AUDIT_CHECKS),
        "zero_activity": lock.get("actor_transplants") == 1
        and _zero_int(lock.get("actor_updates"))
        and _zero_int(lock.get("critic_updates"))
        and _zero_int(lock.get("ppo_updates"))
        and _zero_int(lock.get("environment_samples")),
        "authority": lock.get("training_authorized") is False
        and lock.get("training_command_published") is False
        and _same_json(lock.get("authority"), contract.AUTHORITY),
    }
    if not all(checks.values()):
        failed = sorted(name for name, passed in checks.items() if passed is not True)
        raise ZeroCheckpointDeferredError(f"checkpoint-zero lock failed: {failed}")
    return lock


def verify_lock() -> dict[str, Any]:
    """Verify the published lock, live semantic upstreams, and source closure."""

    lock = strict_json(LOCK)
    observed_upstream = semantic_upstream_payload()
    if not _same_json(observed_upstream, lock.get("upstream_payload")):
        raise ZeroCheckpointDeferredError("locked R8/Q3 terminal evidence drifted")
    observed_closure = closure_snapshot()
    return verify_lock_payload(
        lock,
        observed_source_closure=observed_closure,
    )


def _array(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    return np.ascontiguousarray(np.asarray(value))


def _bytes_equal(left: Any, right: Any) -> bool:
    first = _array(left)
    second = _array(right)
    return (
        first.dtype == second.dtype
        and first.shape == second.shape
        and first.tobytes(order="C") == second.tobytes(order="C")
    )


def _positive_zero(value: Any) -> bool:
    array = _array(value)
    if array.dtype == np.dtype(np.float32):
        return bool(np.all(array.view(np.uint32) == 0))
    if array.dtype == np.dtype(np.float64):
        return bool(np.all(array.view(np.uint64) == 0))
    return False


def _tensor_digest(value: Any) -> str:
    array = _array(value)
    digest = hashlib.sha256()
    digest.update(str(array.dtype).encode("ascii"))
    digest.update(repr(tuple(int(item) for item in array.shape)).encode("ascii"))
    digest.update(array.tobytes(order="C"))
    return digest.hexdigest()


def actor_state_digest(state: Mapping[str, Any]) -> str:
    """Match ``warm_start.actor_state_digest`` for the ten actor tensors."""

    digest = hashlib.sha256()
    for name in sorted(ACTOR_STATE_SHAPES):
        if name not in state:
            raise ZeroCheckpointDeferredError(f"actor state is missing {name}")
        digest.update(name.encode("utf-8"))
        digest.update(_tensor_digest(state[name]).encode("ascii"))
    return digest.hexdigest()


def validate_candidate_actor_state(state: Mapping[str, Any]) -> dict[str, Any]:
    """Validate exact actor-only W512 shapes, aliases, clock zeros and sigma."""

    if set(state) != set(ACTOR_STATE_SHAPES):
        raise ZeroCheckpointDeferredError(
            "candidate is not an actor-only standard W512 state"
        )
    arrays = {name: _array(state[name]) for name in ACTOR_STATE_SHAPES}
    if any(
        arrays[name].dtype != np.dtype(np.float32)
        or arrays[name].shape != shape
        or not np.all(np.isfinite(arrays[name]))
        for name, shape in ACTOR_STATE_SHAPES.items()
    ):
        raise ZeroCheckpointDeferredError(
            "candidate actor dtype, shape, or finiteness drifted"
        )
    for left, right in (
        ("pi_encoder.0.weight", "pi.0.0.weight"),
        ("pi_encoder.0.bias", "pi.0.0.bias"),
        ("pi_encoder.2.weight", "pi.0.2.weight"),
        ("pi_encoder.2.bias", "pi.0.2.bias"),
    ):
        if not _bytes_equal(arrays[left], arrays[right]):
            raise ZeroCheckpointDeferredError("candidate encoder aliases drifted")
    if not _positive_zero(
        arrays["pi_encoder.0.weight"][:, contract.DISABLED_CLOCK_COLUMNS]
    ) or not _positive_zero(arrays["pi.1.weight"][2:]):
        raise ZeroCheckpointDeferredError("clock/logstd bit-zero contract drifted")
    sigma = np.exp(arrays["pi.1.bias"][2:].astype(np.float64))
    if not np.allclose(
        sigma,
        np.repeat(contract.EXPECTED_SIGMA, contract.EXPECTED_ACTION_DIM),
        rtol=0.0,
        atol=1.0e-9,
    ):
        raise ZeroCheckpointDeferredError("candidate exploration sigma drifted")
    return {
        "actor_only": True,
        "actor_feature_count": contract.EXPECTED_ACTOR_FEATURES,
        "hidden_dims": list(contract.EXPECTED_HIDDENS),
        "action_dim": contract.EXPECTED_ACTION_DIM,
        "actor_key_count": len(ACTOR_STATE_SHAPES),
        "disabled_clock_columns_bit_zero": True,
        "logstd_weight_bit_zero": True,
        "sigma": sigma.astype(float).tolist(),
        "actor_digest": actor_state_digest(state),
    }


def transplant_standard_actor(
    *,
    target_state: Mapping[str, Any],
    candidate_state: Mapping[str, Any],
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Replace only actor tensors and prove every non-actor byte is preserved."""

    candidate = validate_candidate_actor_state(candidate_state)
    if not set(ACTOR_STATE_SHAPES).issubset(target_state):
        raise ZeroCheckpointDeferredError("target standard actor surface is missing")
    merged = {
        name: value.clone() if hasattr(value, "clone") else copy.deepcopy(value)
        for name, value in target_state.items()
    }
    for name, shape in ACTOR_STATE_SHAPES.items():
        if _array(merged[name]).shape != shape:
            raise ZeroCheckpointDeferredError(
                f"target actor topology drifted at {name}"
            )
        value = candidate_state[name]
        merged[name] = (
            value.clone() if hasattr(value, "clone") else copy.deepcopy(value)
        )
    actor_exact = all(
        _bytes_equal(candidate_state[name], merged[name]) for name in ACTOR_STATE_SHAPES
    )
    non_actor = sorted(set(target_state) - set(ACTOR_STATE_SHAPES))
    critic_exact = bool(non_actor) and all(
        _bytes_equal(target_state[name], merged[name]) for name in non_actor
    )
    if not actor_exact or not critic_exact:
        raise ZeroCheckpointDeferredError(
            "actor transplant was not exact or changed the fresh critic"
        )
    return merged, {
        "candidate_actor_digest": candidate["actor_digest"],
        "actor_exact": True,
        "fresh_critic_preserved_byte_exact": True,
        "source_actor_key_count": len(ACTOR_STATE_SHAPES),
        "source_non_actor_key_count": 0,
        "target_non_actor_key_count": len(non_actor),
        "actor_transplants": 1,
        "actor_updates": 0,
    }


def _json_option(value: Any, label: str) -> Any:
    if value is None or type(value) in {bool, str, int}:
        return value
    if isinstance(value, numbers.Real):
        numeric = float(value)
        if not math.isfinite(numeric):
            raise ZeroCheckpointDeferredError(f"non-finite optimizer option: {label}")
        return numeric
    if isinstance(value, (list, tuple)):
        return [_json_option(item, label) for item in value]
    raise ZeroCheckpointDeferredError(
        f"unsupported optimizer option {label}: {type(value)}"
    )


def optimizer_snapshot_on_learner(learner: Any) -> dict[str, Any]:
    """Prove empty optimizer state and one registration per trainable parameter."""

    collection = getattr(learner, "module", None)
    module = None
    if collection is not None:
        try:
            module = collection[contract.DEFAULT_POLICY_ID]
        except (KeyError, TypeError):
            pass
    if module is None and callable(getattr(learner, "get_module", None)):
        module = learner.get_module(contract.DEFAULT_POLICY_ID)
    if module is None:
        raise ZeroCheckpointDeferredError("learner module is unavailable")
    named = list(module.named_parameters())
    if not named or len({name for name, _ in named}) != len(named):
        raise ZeroCheckpointDeferredError("learner parameter surface is invalid")
    trainable = {
        id(parameter): name for name, parameter in named if parameter.requires_grad
    }
    if not trainable:
        raise ZeroCheckpointDeferredError("target learner has no trainable parameters")
    seen: set[int] = set()
    rows: list[dict[str, Any]] = []
    optimizer_names: set[str] = set()
    for raw_name, optimizer in learner.get_optimizers_for_module(
        contract.DEFAULT_POLICY_ID
    ):
        name = str(raw_name)
        state = optimizer.state_dict()
        state_entries = state.get("state") if isinstance(state, Mapping) else None
        groups = getattr(optimizer, "param_groups", None)
        if not name or name in optimizer_names:
            raise ZeroCheckpointDeferredError("optimizer names are invalid")
        optimizer_names.add(name)
        if not isinstance(state_entries, Mapping) or state_entries:
            raise ZeroCheckpointDeferredError(f"optimizer {name} state is not empty")
        if not isinstance(groups, list) or not groups:
            raise ZeroCheckpointDeferredError(f"optimizer {name} has no groups")
        group_rows: list[dict[str, Any]] = []
        for index, group in enumerate(groups):
            if not isinstance(group, Mapping) or not isinstance(
                group.get("params"), list
            ):
                raise ZeroCheckpointDeferredError(f"optimizer {name} group malformed")
            parameter_names: list[str] = []
            for parameter in group["params"]:
                identity = id(parameter)
                parameter_name = trainable.get(identity)
                if parameter_name is None or identity in seen:
                    raise ZeroCheckpointDeferredError(
                        "optimizer contains an unknown, frozen, or duplicate parameter"
                    )
                seen.add(identity)
                parameter_names.append(parameter_name)
            group_rows.append(
                {
                    "group_index": index,
                    "parameter_names": parameter_names,
                    "options": {
                        key: _json_option(value, f"{name}.{key}")
                        for key, value in sorted(group.items())
                        if key != "params"
                    },
                }
            )
        rows.append(
            {
                "optimizer_name": name,
                "optimizer_type": type(optimizer).__name__,
                "state_entry_count": 0,
                "param_groups": group_rows,
            }
        )
    if not rows or set(trainable) != seen:
        raise ZeroCheckpointDeferredError(
            "not every trainable parameter is registered exactly once"
        )
    return {
        "optimizer_state_empty": True,
        "trainable_parameter_count": len(trainable),
        "trainable_parameter_names": sorted(trainable.values()),
        "all_trainable_parameters_registered_once": True,
        "optimizers": rows,
    }


def zero_progress_snapshot(values: Mapping[str, Any]) -> dict[str, int]:
    """Normalize the six RLlib progress counters only when all are exact zero."""

    if set(values) != set(contract.ZERO_COUNTER_NAMES):
        raise ZeroCheckpointDeferredError("zero-progress counter schema drifted")
    result: dict[str, int] = {}
    for name in contract.ZERO_COUNTER_NAMES:
        value = values[name]
        if isinstance(value, bool) or not isinstance(value, numbers.Real):
            raise ZeroCheckpointDeferredError(f"{name} is not numeric")
        numeric = float(value)
        if not math.isfinite(numeric) or numeric != 0.0:
            raise ZeroCheckpointDeferredError(f"{name} is not zero")
        result[name] = 0
    return result


def zero_progress_audit(algo: Any) -> dict[str, int]:
    """Read and require the six canonical RLlib progress counters at zero."""

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
        raise ZeroCheckpointDeferredError("Algorithm metrics logger is unavailable")
    values: dict[str, Any] = {"training_iteration": getattr(algo, "iteration", None)}
    values.update(
        {
            label: logger.peek(metric, default=0.0)
            for label, metric in metric_names.items()
        }
    )
    return zero_progress_snapshot(values)


def optimizer_audit(train_module: Any, algo: Any) -> dict[str, Any]:
    reports = train_module._learner_call_results(  # noqa: SLF001
        algo, optimizer_snapshot_on_learner
    )
    if (
        not isinstance(reports, Sequence)
        or isinstance(reports, (str, bytes, bytearray))
        or len(reports) != 1
        or not isinstance(reports[0], Mapping)
    ):
        raise ZeroCheckpointDeferredError(
            "optimizer audit requires exactly one learner report"
        )
    return copy.deepcopy(dict(reports[0]))


def _partition_surface(state: Mapping[str, Any], *, actor: bool) -> dict[str, Any]:
    names = sorted(name for name in state if (name in ACTOR_STATE_SHAPES) is actor)
    if actor and names != sorted(ACTOR_STATE_SHAPES):
        raise ZeroCheckpointDeferredError("actor surface keys drifted")
    if not actor and not names:
        raise ZeroCheckpointDeferredError("fresh critic surface is empty")
    digest = hashlib.sha256()
    byte_count = 0
    for name in names:
        array = _array(state[name])
        byte_count += int(array.nbytes)
        digest.update(name.encode("utf-8"))
        digest.update(_tensor_digest(array).encode("ascii"))
    if actor:
        return {
            "actor_digest": actor_state_digest(state),
            "actor_state_sha256": digest.hexdigest(),
            "actor_key_count": len(names),
            "actor_byte_count": byte_count,
        }
    return {
        "critic_state_sha256": digest.hexdigest(),
        "critic_key_count": len(names),
        "critic_byte_count": byte_count,
    }


def _actor_surface(state: Mapping[str, Any]) -> dict[str, Any]:
    return _partition_surface(state, actor=True)


def _critic_surface(state: Mapping[str, Any]) -> dict[str, Any]:
    return _partition_surface(state, actor=False)


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
        raise ZeroCheckpointDeferredError(
            "target critic is not fresh and source-independent"
        )
    return {
        "candidate_actor_only": candidate_non_actor,
        "fresh_target_critic": fresh_critic,
        "source_target_boundary": boundary,
        "surface": _critic_surface(learner_state),
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
    comparisons = {
        "local_actor": warm_start.compare_actor_states(expected_actor, local_state),
        "learner_actor": warm_start.compare_actor_states(expected_actor, learner_state),
        "local_critic": warm_start.compare_non_actor_states(
            expected_full_state, local_state
        ),
        "learner_critic": warm_start.compare_non_actor_states(
            expected_full_state, learner_state
        ),
    }
    if not all(item["exact"] for item in comparisons.values()):
        raise ZeroCheckpointDeferredError(
            "local/learner actor or critic surface drifted"
        )
    runner_states = algo.env_runner_group.foreach_env_runner(
        func=lambda runner: runner.get_state(
            components=[COMPONENT_RL_MODULE], inference_only=True
        ),
        local_env_runner=True,
        timeout_seconds=float(timeout_s),
    )
    runner_surfaces: list[dict[str, Any]] = []
    for index, state in enumerate(runner_states):
        actor_state = warm_start.find_actor_state(state)
        if actor_state is None:
            raise ZeroCheckpointDeferredError(f"EnvRunner {index} has no actor state")
        comparison = warm_start.compare_actor_states(expected_actor, actor_state)
        if not comparison["exact"]:
            raise ZeroCheckpointDeferredError(
                f"EnvRunner {index} actor surface drifted"
            )
        runner_surfaces.append(_actor_surface(actor_state))
    if not runner_surfaces or any(
        not _same_json(item, runner_surfaces[0]) for item in runner_surfaces[1:]
    ):
        raise ZeroCheckpointDeferredError(
            "EnvRunner actor surfaces are absent or inconsistent"
        )
    return {
        "local": _actor_surface(local_state),
        "learner": _actor_surface(learner_state),
        "env_runner": runner_surfaces[0],
        "critic": _critic_surface(learner_state),
        "comparisons": comparisons,
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
        raise ZeroCheckpointDeferredError(f"refusing to clobber export: {destination}")
    reject_link_or_reparse_ancestors(destination, include_leaf=False)
    algo.get_module(contract.DEFAULT_POLICY_ID).save_to_path(destination)
    exported = warm_start.load_module_state(destination)
    actor = warm_start.compare_actor_states(expected_actor, exported)
    critic = warm_start.compare_non_actor_states(expected_full_state, exported)
    if not actor["exact"] or not critic["exact"]:
        raise ZeroCheckpointDeferredError(
            "exported actor/critic differs from live zero module"
        )
    manifest = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_V12R8_Q3_ZERO_CHECKPOINT_MODULE_EXPORT",
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": lock["binding"]["candidate_id"],
        "candidate_module": lock["binding"]["candidate_module"],
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
        "actor": _actor_surface(exported),
        "critic": _critic_surface(exported),
        "manifest": artifact_record(manifest_path),
        "tree": tree_record(destination),
    }


def _assert_full_checkpoint(path: Path) -> dict[str, Any]:
    tree = tree_record(path)
    gate = gates.checkpoint_tree_gate(tree)
    if gate.get("passed") is not True:
        missing = sorted(
            contract.CHECKPOINT_REQUIRED_SUFFIXES
            - {row["path"] for row in tree["files"]}
        )
        raise ZeroCheckpointDeferredError(
            f"full RLlib checkpoint is incomplete: {missing}"
        )
    return tree


def validate_runtime_target_config() -> dict[str, Any]:
    """Attest the immutable V26/corridor input files and live reward fields."""

    for name, expected in contract.PROFILE_ATTESTATIONS.items():
        observed = artifact_record(INPUT_PATHS[name])
        if (
            observed["path"] != expected["path"]
            or observed["sha256"] != expected["sha256"]
        ):
            raise ZeroCheckpointDeferredError(f"{name} profile attestation drifted")
    return {
        "fixed": copy.deepcopy(contract.TARGET_FIXED_CONFIG),
        "zero_reward": copy.deepcopy(contract.ZERO_REWARD_CONFIG),
        "positive_reward": copy.deepcopy(contract.POSITIVE_RESTORE_REWARD_CONFIG),
        "runtime_attestations": copy.deepcopy(contract.TARGET_RUNTIME_ATTESTATIONS),
    }


def _target_training_args(
    train_module: Any,
    output_dir: Path,
    *,
    positive: bool,
) -> tuple[Any, dict[str, Any]]:
    validate_runtime_target_config()
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
        contract.q3.DETECTOR_PROFILE_PATH.as_posix(),
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
    if not _same_json(observed, contract.TARGET_FIXED_CONFIG):
        raise ZeroCheckpointDeferredError(
            f"resolved fixed target config drifted: {observed}"
        )
    selected = (
        contract.POSITIVE_RESTORE_REWARD_CONFIG
        if positive
        else contract.ZERO_REWARD_CONFIG
    )
    reward = dict(getattr(args, "_cfg_reward", None) or {})
    reward.update(copy.deepcopy(selected))
    args._cfg_reward = reward
    return args, reward


def _build_algorithm(train_module: Any, args: Any, reward: Mapping[str, Any]) -> Any:
    config = train_module.build_config(args, dict(reward))
    return config.build_algo()


def training_resume_source_order_audit(train_module: Any) -> dict[str, Any]:
    source = Path(train_module.__file__).read_text(encoding="utf-8")
    yaml_at = source.find(
        'reward_overrides = dict(getattr(args, "_cfg_reward", None) or {})'
    )
    json_at = source.find(
        "json_overrides = reward_function.load_reward_overrides(args.reward_json)",
        yaml_at,
    )
    merge_at = source.find("reward_overrides.update(json_overrides)", json_at)
    config_at = source.find("config = build_config(args, reward_overrides)")
    algo_at = source.find("algo = config.build_algo()", config_at)
    restore_at = source.find("algo.restore_from_path(resume_path)", algo_at)
    if not 0 <= yaml_at < json_at < merge_at < config_at < algo_at < restore_at:
        raise ZeroCheckpointDeferredError(
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
        raise ZeroCheckpointDeferredError("Algorithm has no live env_config mapping")
    reward = env.get("reward")
    if not isinstance(reward, Mapping):
        raise ZeroCheckpointDeferredError(
            "Algorithm live env_config has no reward mapping"
        )
    env_expected = {
        "phase_fsm_input_mode": "legacy_events",
        "event_contract_id": contract.LEGACY_EVENT_CONTRACT_ID,
        "binary_phase_fsm_mode": "binary_active",
        "binary_phase_detector_profile_file": contract.q3.DETECTOR_PROFILE_PATH.as_posix(),
        "detector_sample_dt_s": 0.001,
        "binary_phase_debounce_s": 0.005,
        "binary_phase_event_contract_id": contract.BINARY_EVENT_CONTRACT_ID,
    }
    observed_env = {key: env.get(key) for key in env_expected}
    observed_reward = {key: reward.get(key) for key in expected_reward}
    if observed_env != env_expected or not _same_json(observed_reward, expected_reward):
        raise ZeroCheckpointDeferredError(
            "Algorithm live V26/morphology config drifted"
        )
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
    """Build/save/reload the real zero checkpoint without train/sample calls."""

    import training_config
    import train_ppo_mlp as train
    import warm_start

    locked_closure = copy.deepcopy(dict(lock["source_closure"]))
    verify_closure(locked_closure)
    source_order = training_resume_source_order_audit(train)
    train._load_training_stack()  # noqa: SLF001
    ray = train.ray
    zero_args, zero_reward = _target_training_args(
        train, OUTPUT_ROOT / "rllib_zero", positive=False
    )
    positive_args, positive_reward = _target_training_args(
        train, OUTPUT_ROOT / "rllib_positive_restore_smoke", positive=True
    )
    training_config.dump_resolved(
        zero_args,
        zero_reward,
        resolve_relative(contract.RESOLVED_CONFIG_PATH),
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
        candidate_audit = validate_candidate_actor_state(candidate_state)
        if candidate_audit["actor_digest"] != lock["binding"]["actor_digest"]:
            raise ZeroCheckpointDeferredError(
                "candidate state/manifest actor digest drifted"
            )

        verify_closure(locked_closure)
        first = _build_algorithm(train, zero_args, zero_reward)
        zero_live = live_config_snapshot(
            first, expected_reward=contract.ZERO_REWARD_CONFIG
        )
        progress_before = zero_progress_audit(first)
        optimizer_before = optimizer_audit(train, first)
        feature_names = tuple(getattr(zero_args, "_target_actor_feature_names", ()))
        full_names = tuple(getattr(zero_args, "_target_observation_feature_names", ()))
        if feature_names != tuple(contract.q3.ACTOR_FEATURE_NAMES):
            raise ZeroCheckpointDeferredError(
                "target actor feature order is not exact 35"
            )
        if len(full_names) != contract.EXPECTED_FULL_FEATURES:
            raise ZeroCheckpointDeferredError(
                "target full observation layout is not exact 84"
            )

        local_before = first.get_module(contract.DEFAULT_POLICY_ID).get_state()
        learner_before = train._learner_module_state(first)  # noqa: SLF001
        fresh = fresh_critic_audit(
            candidate_state=candidate_state,
            learner_state=learner_before,
            warm_start=warm_start,
        )
        transplanted, transplant_report = transplant_standard_actor(
            target_state=local_before,
            candidate_state=candidate_state,
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
            raise ZeroCheckpointDeferredError("actor transplant changed fresh critic")
        optimizer_after = optimizer_audit(train, first)
        if not _same_json(optimizer_after, optimizer_before):
            raise ZeroCheckpointDeferredError(
                "optimizer groups changed by actor transplant"
            )
        before_surfaces = _actor_surfaces(
            algo=first,
            expected_actor=candidate_state,
            expected_full_state=learner_after,
            train_module=train,
            warm_start=warm_start,
            timeout_s=zero_args.startup_timeout_s,
        )
        progress_after = zero_progress_audit(first)
        if progress_after != progress_before:
            raise ZeroCheckpointDeferredError(
                "actor transplant changed progress counters"
            )
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
            raise ZeroCheckpointDeferredError(
                f"refusing to clobber checkpoint: {checkpoint}"
            )
        first.save_to_path(checkpoint)
        checkpoint_tree = _assert_full_checkpoint(checkpoint)
        learner_after_save = train._learner_module_state(first)  # noqa: SLF001
        optimizer_after_save = optimizer_audit(train, first)
        progress_after_save = zero_progress_audit(first)
        if not _same_json(_critic_surface(learner_after_save), fresh["surface"]):
            raise ZeroCheckpointDeferredError("checkpoint save changed fresh critic")
        if not _same_json(optimizer_after_save, optimizer_before):
            raise ZeroCheckpointDeferredError("checkpoint save changed optimizer")
        if progress_after_save != progress_before:
            raise ZeroCheckpointDeferredError("checkpoint save changed zero counters")
        first.stop()
        first = None

        verify_closure(locked_closure)
        restored = _build_algorithm(train, zero_args, zero_reward)
        restored_config_before = live_config_snapshot(
            restored, expected_reward=contract.ZERO_REWARD_CONFIG
        )
        zero_progress_audit(restored)
        restored.restore_from_path(checkpoint)
        restored_config_after = live_config_snapshot(
            restored, expected_reward=contract.ZERO_REWARD_CONFIG
        )
        if restored_config_after != restored_config_before:
            raise ZeroCheckpointDeferredError("zero live config changed during restore")
        restored_progress = zero_progress_audit(restored)
        restored_optimizer = optimizer_audit(train, restored)
        restored_learner = train._learner_module_state(restored)  # noqa: SLF001
        restored_critic = warm_start.compare_non_actor_states(
            learner_after, restored_learner
        )
        if not restored_critic["exact"]:
            raise ZeroCheckpointDeferredError(
                "restored critic differs from saved fresh critic"
            )
        if not _same_json(restored_optimizer, optimizer_before):
            raise ZeroCheckpointDeferredError("restored optimizer groups/state drifted")
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
        restored.stop()
        restored = None

        verify_closure(locked_closure)
        positive = _build_algorithm(train, positive_args, positive_reward)
        positive_before = live_config_snapshot(
            positive, expected_reward=contract.POSITIVE_RESTORE_REWARD_CONFIG
        )
        if (
            positive_before["morphology_weight"] != contract.POSITIVE_MORPHOLOGY_WEIGHT
            or positive_before["morphology_causal_allow_effects"] != 1.0
        ):
            raise ZeroCheckpointDeferredError(
                "positive restore smoke did not start with exact live config"
            )
        zero_progress_audit(positive)
        positive.restore_from_path(checkpoint)
        positive_after = live_config_snapshot(
            positive, expected_reward=contract.POSITIVE_RESTORE_REWARD_CONFIG
        )
        if positive_after != positive_before:
            raise ZeroCheckpointDeferredError(
                "checkpoint restore replaced the live positive reward/env config"
            )
        positive_progress = zero_progress_audit(positive)
        positive_optimizer = optimizer_audit(train, positive)
        positive_learner = train._learner_module_state(positive)  # noqa: SLF001
        positive_critic = warm_start.compare_non_actor_states(
            learner_after, positive_learner
        )
        if not positive_critic["exact"] or not _same_json(
            positive_optimizer, optimizer_before
        ):
            raise ZeroCheckpointDeferredError(
                "positive restore smoke changed zero checkpoint state"
            )
        positive_surfaces = _actor_surfaces(
            algo=positive,
            expected_actor=candidate_state,
            expected_full_state=learner_after,
            train_module=train,
            warm_start=warm_start,
            timeout_s=positive_args.startup_timeout_s,
        )
        verify_closure(locked_closure)

        actor_surfaces = {
            "local": before_surfaces["local"],
            "learner": before_surfaces["learner"],
            "env_runner": before_surfaces["env_runner"],
            "initial_export": initial_export["actor"],
            "restored_local": restored_surfaces["local"],
            "restored_learner": restored_surfaces["learner"],
            "restored_env_runner": restored_surfaces["env_runner"],
            "restored_export": restored_export["actor"],
            "positive_restore_local": positive_surfaces["local"],
            "positive_restore_learner": positive_surfaces["learner"],
            "positive_restore_env_runner": positive_surfaces["env_runner"],
        }
        critic_surfaces = {
            "fresh_before_transplant": fresh["surface"],
            "after_transplant": _critic_surface(learner_after),
            "after_save": _critic_surface(learner_after_save),
            "after_restore": _critic_surface(restored_learner),
            "after_positive_restore": _critic_surface(positive_learner),
        }
        optimizer_surfaces = {
            "before_transplant": optimizer_before,
            "after_transplant": optimizer_after,
            "after_save": optimizer_after_save,
            "after_restore": restored_optimizer,
            "after_positive_restore": positive_optimizer,
        }
        progress_surfaces = {
            "before_transplant": progress_before,
            "after_transplant": progress_after,
            "after_save": progress_after_save,
            "after_restore": restored_progress,
            "after_positive_restore": positive_progress,
        }
        audit = {
            "schema_version": contract.SCHEMA_VERSION,
            "status": "COMPLETE_H0_V12R8_ZERO_CHECKPOINT_AUDIT",
            "protocol_id": contract.PROTOCOL_ID,
            "binding": copy.deepcopy(lock["binding"]),
            "checks": {name: True for name in contract.REQUIRED_AUDIT_CHECKS},
            "target_fixed_config": copy.deepcopy(contract.TARGET_FIXED_CONFIG),
            "zero_reward_config": copy.deepcopy(contract.ZERO_REWARD_CONFIG),
            "actor_surfaces": actor_surfaces,
            "critic_surfaces": critic_surfaces,
            "optimizer_surfaces": optimizer_surfaces,
            "progress_surfaces": progress_surfaces,
            "candidate_source_non_actor_key_count": 0,
            "source_critic_restored": False,
            "source_optimizer_restored": False,
            "checkpoint_tree": checkpoint_tree,
            "positive_restore_smoke": {
                "target_fixed_config": copy.deepcopy(contract.TARGET_FIXED_CONFIG),
                "reward_config": copy.deepcopy(contract.POSITIVE_RESTORE_REWARD_CONFIG),
                "restore_from": contract.CHECKPOINT_PATH.as_posix(),
                "restore_completed": True,
                "actor_digest": lock["binding"]["actor_digest"],
                "critic_surface": _critic_surface(positive_learner),
                "optimizer_snapshot": positive_optimizer,
                "progress": positive_progress,
                "algorithm_train_calls": 0,
                "environment_samples": 0,
            },
            "closure_before_build": copy.deepcopy(locked_closure),
            "closure_before_restore": copy.deepcopy(locked_closure),
            "closure_after_restore": closure_snapshot(),
            "candidate_actor_audit": candidate_audit,
            "transplant": transplant_report,
            "fresh_critic_audit": fresh,
            "training_source_order": source_order,
            "zero_live_config": zero_live,
            "positive_live_config_before_restore": positive_before,
            "positive_live_config_after_restore": positive_after,
            "initial_export": initial_export,
            "restored_export": restored_export,
            "target_actor_feature_names": list(feature_names),
            "target_observation_feature_names": list(full_names),
            "actor_transplants": 1,
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
            "environment_samples": 0,
            "algorithm_train_calls": 0,
            "training_executed": False,
        }
        gate = gates.runtime_audit_gate(audit, expected_binding=lock["binding"])
        if gate.get("passed") is not True:
            failed = sorted(
                name
                for name, passed in _mapping(gate.get("checks")).items()
                if not passed
            )
            raise ZeroCheckpointDeferredError(
                f"live checkpoint-zero audit failed: {failed}"
            )
        return audit
    finally:
        for algo in (positive, restored, first):
            if algo is not None:
                try:
                    algo.stop()
                except Exception:
                    pass
        if ray.is_initialized():
            ray.shutdown()


def checkpoint_creation_plan(lock_payload: Mapping[str, Any]) -> dict[str, Any]:
    """Return the frozen no-sampling operation order without constructing RLlib."""

    lock = verify_lock_payload(lock_payload)
    stages = (
        "rehash_source_closure_before_build",
        "build_fresh_zero_progress_w512_algorithm",
        "audit_fresh_critic_and_empty_optimizer",
        "transplant_candidate_actor_only",
        "sync_and_audit_actor_surfaces",
        "save_full_checkpoint_zero",
        "stop_first_algorithm",
        "rehash_source_closure_before_restore",
        "build_fresh_zero_progress_restore_target",
        "restore_full_checkpoint_zero",
        "audit_restored_actor_critic_optimizer_progress",
        "build_positive_0025_live_config_before_restore",
        "restore_checkpoint_into_positive_live_config",
        "audit_positive_config_and_zero_state_survival",
        "rehash_source_closure_after_restore",
        "publish_zero_receipt_without_training_authority",
    )
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "PLANNED_H0_V12R8_ZERO_CHECKPOINT",
        "protocol_id": contract.PROTOCOL_ID,
        "binding": copy.deepcopy(lock["binding"]),
        "stages": list(stages),
        "target_fixed_config": copy.deepcopy(contract.TARGET_FIXED_CONFIG),
        "zero_reward_config": copy.deepcopy(contract.ZERO_REWARD_CONFIG),
        "positive_restore_reward_config": copy.deepcopy(
            contract.POSITIVE_RESTORE_REWARD_CONFIG
        ),
        "checkpoint_path": contract.CHECKPOINT_PATH.as_posix(),
        "actor_transplants": 1,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "environment_samples": 0,
        "training_executed": False,
    }


def build_terminal_receipt(
    *,
    lock_payload: Mapping[str, Any],
    runtime_audit: Mapping[str, Any],
) -> dict[str, Any]:
    """Produce a terminal PASS/FAIL decision from the complete runtime audit."""

    lock = verify_lock_payload(lock_payload)
    audit = gates.runtime_audit_gate(
        runtime_audit,
        expected_binding=lock["binding"],
    )
    passed = audit.get("passed") is True
    checkpoint = copy.deepcopy(_mapping(runtime_audit).get("checkpoint_tree"))
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PASS_STATUS if passed else contract.FAIL_STATUS,
        "passed": passed,
        "terminal": True,
        "error": None if passed else "zero_checkpoint_runtime_audit_failed",
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "binding": copy.deepcopy(lock["binding"]),
        "audit_gate": audit,
        "checkpoint": checkpoint,
        "actor_transplants": 1,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "environment_samples": 0,
        "training_executed": False,
        "positive_live_config_restore_smoke_passed": passed,
        "training_authorized": False,
        "training_command_published": False,
        "next_stage": (
            contract.NEXT_STAGE_AFTER_ZERO_PASS if passed else "STOP_TERMINAL_NO_RETRY"
        ),
    }


def build_training_handoff(
    terminal_receipt: Mapping[str, Any],
    *,
    terminal_receipt_artifact: Mapping[str, Any],
) -> dict[str, Any]:
    """Build a non-authorizing resume interface after exact zero PASS."""

    receipt_gate = gates.terminal_receipt_gate(terminal_receipt)
    if receipt_gate.get("passed") is not True:
        raise ZeroCheckpointDeferredError(
            "terminal checkpoint-zero PASS is required for the resume interface"
        )
    if not gates.artifact_record_valid(
        terminal_receipt_artifact,
        expected_path=contract.RECEIPT_PATH.as_posix(),
    ):
        raise ZeroCheckpointDeferredError("terminal zero receipt is not hash-bound")
    platforms: dict[str, Any] = {}
    for platform_id in ("macos_arm64", "windows_x86_64"):
        argv = contract.resume_training_argv(platform_id)
        platforms[platform_id] = {
            "proposed_resume_argv": list(argv),
            "proposed_resume_command": contract.render_command(argv, platform_id),
        }
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.HANDOFF_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "authorized_by_terminal_zero_receipt": copy.deepcopy(
            dict(terminal_receipt_artifact)
        ),
        "binding": copy.deepcopy(dict(terminal_receipt["binding"])),
        "checkpoint": copy.deepcopy(dict(terminal_receipt["checkpoint"])),
        "initialization_mode": "resume_from_full_checkpoint_zero",
        "required_flag": "--resume-from",
        "forbidden_flags": ["--warm-start", "--warm-start-raw"],
        "target_training_iterations": contract.FINAL_TRAINING_ITERATIONS,
        "morphology_weight": contract.POSITIVE_MORPHOLOGY_WEIGHT,
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
        "pipeline_id": contract.PIPELINE_ID,
        "binding": copy.deepcopy(dict(lock["binding"])),
        "execution_lock": artifact_record(LOCK),
        "actor_transplants": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "environment_samples": 0,
    }


def _terminal_ledger_payload(
    *,
    lock: Mapping[str, Any],
    claim_path: Path,
    started_unix_s: float,
    passed: bool,
    runtime_audit: Mapping[str, Any] | None,
    error: BaseException | None,
) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PASS_STATUS if passed else contract.FAIL_STATUS,
        "passed": passed,
        "terminal": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "binding": copy.deepcopy(dict(lock["binding"])),
        "started_unix_s": float(started_unix_s),
        "completed_unix_s": float(time.time()),
        "error_type": type(error).__name__ if error is not None else None,
        "error": str(error) if error is not None else None,
        "execution_lock": artifact_record(LOCK),
        "attempt_claim": artifact_record(claim_path),
        "audit": (
            artifact_record(resolve_relative(contract.AUDIT_PATH)) if passed else None
        ),
        "receipt": (
            artifact_record(resolve_relative(contract.RECEIPT_PATH)) if passed else None
        ),
        "training_handoff": (
            artifact_record(resolve_relative(contract.HANDOFF_PATH)) if passed else None
        ),
        "resolved_config": (
            artifact_record(resolve_relative(contract.RESOLVED_CONFIG_PATH))
            if runtime_audit is not None
            else None
        ),
        "checkpoint": (
            copy.deepcopy(dict(runtime_audit["checkpoint_tree"]))
            if runtime_audit is not None
            else None
        ),
        "actor_transplants": 1 if runtime_audit is not None else 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "environment_samples": 0,
        "runtime_promoted": False,
        "training_executed": False,
        "training_authorized": False,
        "training_command_published": False,
        "retry_authorized": False,
        "resume_authorized": False,
        "next_stage": (
            contract.NEXT_STAGE_AFTER_ZERO_PASS if passed else "STOP_TERMINAL_NO_RETRY"
        ),
    }


def execute() -> dict[str, Any]:
    """Claim and execute the canonical checkpoint-zero port exactly once."""

    lock = verify_lock()
    if os.path.lexists(OUTPUT_ROOT):
        raise ZeroCheckpointDeferredError(
            f"checkpoint-zero output is already claimed: {OUTPUT_ROOT}"
        )
    reject_link_or_reparse_ancestors(OUTPUT_ROOT, include_leaf=False)
    OUTPUT_ROOT.mkdir(parents=False, exist_ok=False)
    claim_path = write_json_exclusive(
        OUTPUT_ROOT / contract.OUTPUT_NAMES["attempt_claim"],
        _attempt_claim_payload(lock),
    )
    started = time.time()
    runtime: dict[str, Any] | None = None
    try:
        runtime = _runtime_port(lock)
        audit_gate = gates.runtime_audit_gate(runtime, expected_binding=lock["binding"])
        if audit_gate.get("passed") is not True:
            raise ZeroCheckpointDeferredError(
                "checkpoint-zero runtime audit did not close"
            )
        write_json_exclusive(resolve_relative(contract.AUDIT_PATH), runtime)
        receipt = build_terminal_receipt(
            lock_payload=lock,
            runtime_audit=runtime,
        )
        if receipt.get("passed") is not True:
            raise ZeroCheckpointDeferredError(
                "checkpoint-zero terminal receipt did not pass"
            )
        receipt_path = write_json_exclusive(
            resolve_relative(contract.RECEIPT_PATH), receipt
        )
        handoff = build_training_handoff(
            receipt,
            terminal_receipt_artifact=artifact_record(receipt_path),
        )
        write_json_exclusive(resolve_relative(contract.HANDOFF_PATH), handoff)
        ledger = _terminal_ledger_payload(
            lock=lock,
            claim_path=claim_path,
            started_unix_s=started,
            passed=True,
            runtime_audit=runtime,
            error=None,
        )
        write_json_exclusive(resolve_relative(contract.TERMINAL_LEDGER_PATH), ledger)
        return verify_terminal_pass()
    except BaseException as exc:
        ledger_path = resolve_relative(contract.TERMINAL_LEDGER_PATH)
        if not os.path.lexists(ledger_path):
            try:
                write_json_exclusive(
                    ledger_path,
                    _terminal_ledger_payload(
                        lock=lock,
                        claim_path=claim_path,
                        started_unix_s=started,
                        passed=False,
                        runtime_audit=runtime,
                        error=exc,
                    ),
                )
            except BaseException:
                pass
        raise


def verify_terminal_pass() -> dict[str, Any]:
    """Verify the immutable PASS closure and non-authorizing resume handoff."""

    lock = verify_lock()
    audit = strict_json(resolve_relative(contract.AUDIT_PATH))
    receipt = strict_json(resolve_relative(contract.RECEIPT_PATH))
    handoff = strict_json(resolve_relative(contract.HANDOFF_PATH))
    ledger = strict_json(resolve_relative(contract.TERMINAL_LEDGER_PATH))
    checkpoint = _assert_full_checkpoint(resolve_relative(contract.CHECKPOINT_PATH))
    audit_gate = gates.runtime_audit_gate(audit, expected_binding=lock["binding"])
    expected_receipt = build_terminal_receipt(
        lock_payload=lock,
        runtime_audit=audit,
    )
    receipt_record = artifact_record(resolve_relative(contract.RECEIPT_PATH))
    expected_handoff = build_training_handoff(
        receipt,
        terminal_receipt_artifact=receipt_record,
    )
    ledger_checks = {
        "identity": ledger.get("schema_version") == contract.SCHEMA_VERSION
        and ledger.get("status") == contract.PASS_STATUS
        and ledger.get("passed") is True
        and ledger.get("terminal") is True
        and ledger.get("protocol_id") == contract.PROTOCOL_ID
        and ledger.get("pipeline_id") == contract.PIPELINE_ID
        and ledger.get("error_type") is None
        and ledger.get("error") is None,
        "binding": _same_json(ledger.get("binding"), lock.get("binding")),
        "records": ledger.get("execution_lock") == artifact_record(LOCK)
        and ledger.get("audit")
        == artifact_record(resolve_relative(contract.AUDIT_PATH))
        and ledger.get("receipt") == receipt_record
        and ledger.get("training_handoff")
        == artifact_record(resolve_relative(contract.HANDOFF_PATH))
        and ledger.get("resolved_config")
        == artifact_record(resolve_relative(contract.RESOLVED_CONFIG_PATH)),
        "checkpoint": _same_json(ledger.get("checkpoint"), checkpoint)
        and _same_json(receipt.get("checkpoint"), checkpoint)
        and _same_json(audit.get("checkpoint_tree"), checkpoint)
        and _same_json(handoff.get("checkpoint"), checkpoint),
        "zero_activity": ledger.get("actor_transplants") == 1
        and _zero_int(ledger.get("actor_updates"))
        and _zero_int(ledger.get("critic_updates"))
        and _zero_int(ledger.get("ppo_updates"))
        and _zero_int(ledger.get("environment_samples")),
        "closed": ledger.get("runtime_promoted") is False
        and ledger.get("training_executed") is False
        and ledger.get("training_authorized") is False
        and ledger.get("training_command_published") is False
        and ledger.get("retry_authorized") is False
        and ledger.get("resume_authorized") is False
        and ledger.get("next_stage") == contract.NEXT_STAGE_AFTER_ZERO_PASS,
    }
    if (
        audit_gate.get("passed") is not True
        or not _same_json(receipt, expected_receipt)
        or gates.terminal_receipt_gate(receipt).get("passed") is not True
        or not _same_json(handoff, expected_handoff)
        or not all(ledger_checks.values())
    ):
        failed = sorted(
            name for name, passed in ledger_checks.items() if passed is not True
        )
        raise ZeroCheckpointDeferredError(
            f"terminal checkpoint-zero verification failed: {failed}"
        )
    return receipt


if __name__ == "__main__":
    try:
        print(json.dumps(execute(), indent=2, sort_keys=True, allow_nan=False))
    except ZeroCheckpointDeferredError as exc:
        raise SystemExit(str(exc)) from exc


__all__ = [
    "ACTOR_STATE_SHAPES",
    "LOCK_KEYS",
    "ZeroCheckpointDeferredError",
    "actor_state_digest",
    "build_terminal_receipt",
    "build_training_handoff",
    "candidate_actor_manifest",
    "checkpoint_creation_plan",
    "closure_snapshot",
    "execute",
    "fresh_critic_audit",
    "live_config_snapshot",
    "optimizer_audit",
    "optimizer_snapshot_on_learner",
    "semantic_upstream_payload",
    "strict_json",
    "transplant_standard_actor",
    "tree_record",
    "validate_candidate_actor_state",
    "validate_runtime_target_config",
    "verify_closure",
    "verify_lock",
    "verify_lock_payload",
    "verify_terminal_pass",
    "write_json_exclusive",
    "zero_progress_audit",
    "zero_progress_snapshot",
]
