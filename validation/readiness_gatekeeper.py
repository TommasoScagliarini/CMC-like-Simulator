"""Shared fail-closed primitives for readiness gates.

The module deliberately contains no scientific policy.  It provides the
mechanical contract shared by steps 0, 1, and 2:

* strict JSON decoding (including duplicate-key and non-finite rejection);
* schema-v1 validation for terminal gate payloads;
* repository-relative SHA-256 records and prerequisite-lock verification;
* immutable no-clobber JSON publication; and
* a uniform ``failure.json`` payload builder.

Callers remain responsible for deciding which scientific checks constitute a
PASS.  A PASS read through :func:`verify_gate_file` is accepted only after its
immutable prerequisite chain and any embedded protocol/source/candidate
bindings have been re-hashed.
"""

from __future__ import annotations

import hashlib
import hmac
import json
import math
import os
import uuid
from pathlib import Path, PurePosixPath, PureWindowsPath
from typing import Any, Mapping, Sequence


SCHEMA_VERSION = 1
STEP_IDS = frozenset({"0", "1", "2"})
TERMINAL_STATUSES = frozenset({"PASS", "FAIL", "BLOCKED", "ERROR"})
CANONICAL_DECISIONS = {
    "0": "PRIMARY_GRF_READY",
    "1": "H0_SEP_READY",
    "2": "DETECTOR_TRAINING_READY",
}
REQUIRED_GATE_FIELDS = frozenset(
    {
        "schema_version",
        "step_id",
        "status",
        "protocol_sha256",
        "source_hashes_match",
        "prerequisite_locks",
        "data_access_receipts",
        "checks",
        "failed_checks",
        "artifacts",
    }
)
_PROTECTED_SOURCE_FIELDS = ("source_hashes", "protected_sources")
_EXPECTED_DIRECT_PREREQUISITES = {
    "0": frozenset(),
    "1": frozenset({"0"}),
    "2": frozenset({"0", "1"}),
}


class GatekeeperError(ValueError):
    """Base class for a fail-closed gatekeeper refusal."""


class StrictJSONError(GatekeeperError):
    """Raised when input is not strict, finite JSON."""


class GateSchemaError(GatekeeperError):
    """Raised when a gate payload violates schema v1."""


class HashVerificationError(GatekeeperError):
    """Raised when a protected file is absent, unsafe, or hash-incoherent."""


class PrerequisiteLockError(GatekeeperError):
    """Raised when a prerequisite is not an intact PASS lock."""


class NoClobberError(GatekeeperError):
    """Raised when an immutable output destination is already occupied."""


def is_sha256(value: Any) -> bool:
    """Return whether *value* is a canonical lowercase SHA-256 digest."""

    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def _reject_nonfinite_constant(token: str) -> None:
    raise StrictJSONError(f"non-finite JSON number {token!r}")


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise StrictJSONError(f"duplicate JSON object key {key!r}")
        result[key] = value
    return result


def _validate_json_value(
    value: Any,
    *,
    location: str = "$",
    ancestors: set[int] | None = None,
) -> None:
    """Reject values which cannot be represented as finite strict JSON."""

    if value is None or isinstance(value, (str, bool)):
        return
    if isinstance(value, int):
        return
    if isinstance(value, float):
        if not math.isfinite(value):
            raise StrictJSONError(f"{location} is not finite")
        return

    active = ancestors if ancestors is not None else set()
    if isinstance(value, Mapping):
        identity = id(value)
        if identity in active:
            raise StrictJSONError(f"{location} contains a reference cycle")
        active.add(identity)
        try:
            for key, item in value.items():
                if not isinstance(key, str):
                    raise StrictJSONError(
                        f"{location} contains non-string object key {key!r}"
                    )
                _validate_json_value(
                    item,
                    location=f"{location}.{key}",
                    ancestors=active,
                )
        finally:
            active.remove(identity)
        return

    if isinstance(value, (list, tuple)):
        identity = id(value)
        if identity in active:
            raise StrictJSONError(f"{location} contains a reference cycle")
        active.add(identity)
        try:
            for index, item in enumerate(value):
                _validate_json_value(
                    item,
                    location=f"{location}[{index}]",
                    ancestors=active,
                )
        finally:
            active.remove(identity)
        return

    raise StrictJSONError(
        f"{location} has unsupported JSON type {type(value).__name__}"
    )


def loads_json_strict(text: str, *, source: str = "<string>") -> Any:
    """Decode one strict JSON document.

    Python's standard decoder normally accepts ``NaN``/``Infinity`` and keeps
    the last duplicate object key.  Both behaviours are forbidden here.
    Overflowing numeric literals such as ``1e999`` are also rejected by the
    post-decode finiteness walk.
    """

    if not isinstance(text, str):
        raise StrictJSONError(f"{source}: JSON input must be text")
    try:
        value = json.loads(
            text,
            object_pairs_hook=_reject_duplicate_keys,
            parse_constant=_reject_nonfinite_constant,
        )
        _validate_json_value(value)
    except StrictJSONError as exc:
        raise StrictJSONError(f"{source}: {exc}") from exc
    except (json.JSONDecodeError, RecursionError) as exc:
        raise StrictJSONError(f"{source}: invalid JSON: {exc}") from exc
    return value


def load_json_strict(path: str | Path) -> Any:
    """Read and strictly decode one UTF-8 JSON file."""

    source = Path(path)
    try:
        text = source.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as exc:
        raise StrictJSONError(f"cannot read strict JSON {source}: {exc}") from exc
    return loads_json_strict(text, source=source.as_posix())


def _require_json_array(payload: Mapping[str, Any], field: str) -> list[Any]:
    value = payload[field]
    if type(value) is not list:
        raise GateSchemaError(f"{field} must be a JSON array")
    return value


def _validate_file_record_shape(record: Any, *, label: str) -> Mapping[str, Any]:
    if not isinstance(record, Mapping):
        raise GateSchemaError(f"{label} must be an object")
    path = record.get("path")
    if not isinstance(path, str) or not path.strip():
        raise GateSchemaError(f"{label}.path must be a non-empty string")
    if not is_sha256(record.get("sha256")):
        raise GateSchemaError(f"{label}.sha256 must be a lowercase SHA-256")
    if "size_bytes" in record and (
        type(record["size_bytes"]) is not int or record["size_bytes"] < 0
    ):
        raise GateSchemaError(f"{label}.size_bytes must be a non-negative integer")
    return record


def _validate_checks(
    checks: Sequence[Any],
    failed_checks: Sequence[Any],
    *,
    gate_status: str,
) -> None:
    identifiers: list[str] = []
    failed_identifiers: list[str] = []
    for index, check in enumerate(checks):
        label = f"checks[{index}]"
        if not isinstance(check, Mapping):
            raise GateSchemaError(f"{label} must be an object")
        identifier = check.get("id")
        status = check.get("status")
        if not isinstance(identifier, str) or not identifier.strip():
            raise GateSchemaError(f"{label}.id must be a non-empty string")
        if not isinstance(status, str) or status not in TERMINAL_STATUSES:
            raise GateSchemaError(
                f"{label}.status must be PASS, FAIL, BLOCKED, or ERROR"
            )
        identifiers.append(identifier)
        if status != "PASS":
            failed_identifiers.append(identifier)
    if len(set(identifiers)) != len(identifiers):
        raise GateSchemaError("checks contains duplicate IDs")
    for index, identifier in enumerate(failed_checks):
        if not isinstance(identifier, str) or not identifier.strip():
            raise GateSchemaError(
                f"failed_checks[{index}] must be a non-empty check ID"
            )
    if len(set(failed_checks)) != len(failed_checks):
        raise GateSchemaError("failed_checks contains duplicate IDs")
    if list(failed_checks) != failed_identifiers:
        raise GateSchemaError(
            "failed_checks must exactly list non-PASS checks in checks order"
        )
    if gate_status == "PASS" and any(
        check_status != "PASS"
        for check_status in (
            check["status"] for check in checks if isinstance(check, Mapping)
        )
    ):
        raise GateSchemaError("PASS gate contains a non-PASS check")


def _records_from_collection(
    records: Any,
    *,
    label: str,
    allow_empty: bool,
) -> list[tuple[str, Mapping[str, Any]]]:
    if isinstance(records, Mapping):
        if "path" in records or "sha256" in records:
            values = [(label, _validate_file_record_shape(records, label=label))]
        else:
            values = []
            for name, record in records.items():
                if not isinstance(name, str) or not name:
                    raise GateSchemaError(f"{label} contains an invalid record name")
                record_label = f"{label}.{name}"
                values.append(
                    (
                        record_label,
                        _validate_file_record_shape(record, label=record_label),
                    )
                )
    elif isinstance(records, Sequence) and not isinstance(records, (str, bytes)):
        values = [
            (
                f"{label}[{index}]",
                _validate_file_record_shape(record, label=f"{label}[{index}]"),
            )
            for index, record in enumerate(records)
        ]
    else:
        raise GateSchemaError(f"{label} must be an object or array of file records")
    if not values and not allow_empty:
        raise GateSchemaError(f"{label} must contain at least one file record")
    return values


def validate_gate_payload(
    payload: Mapping[str, Any],
    *,
    expected_step_id: str | None = None,
    expected_protocol_sha256: str | None = None,
    require_pass: bool = False,
) -> dict[str, Any]:
    """Validate and return a shallow copy of one terminal schema-v1 gate."""

    if not isinstance(payload, Mapping):
        raise GateSchemaError("gate payload must be a JSON object")
    try:
        _validate_json_value(payload)
    except StrictJSONError as exc:
        raise GateSchemaError(str(exc)) from exc

    missing = sorted(REQUIRED_GATE_FIELDS.difference(payload))
    if missing:
        raise GateSchemaError(f"gate payload missing required fields: {missing}")
    if type(payload["schema_version"]) is not int or payload["schema_version"] != 1:
        raise GateSchemaError("schema_version must be integer 1")
    if not isinstance(payload["step_id"], str) or payload["step_id"] not in STEP_IDS:
        raise GateSchemaError("step_id must be one of '0', '1', or '2'")
    if expected_step_id is not None and payload["step_id"] != expected_step_id:
        raise GateSchemaError(
            f"step_id mismatch: expected {expected_step_id!r}, "
            f"observed {payload['step_id']!r}"
        )
    if (
        not isinstance(payload["status"], str)
        or payload["status"] not in TERMINAL_STATUSES
    ):
        raise GateSchemaError(
            "status must be PASS, FAIL, BLOCKED, or ERROR"
        )
    if not is_sha256(payload["protocol_sha256"]):
        raise GateSchemaError("protocol_sha256 must be a lowercase SHA-256")
    if (
        expected_protocol_sha256 is not None
        and payload["protocol_sha256"] != expected_protocol_sha256
    ):
        raise GateSchemaError("protocol_sha256 does not match the expected protocol")
    if type(payload["source_hashes_match"]) is not bool:
        raise GateSchemaError("source_hashes_match must be a boolean")

    prerequisite_locks = _require_json_array(payload, "prerequisite_locks")
    data_access_receipts = _require_json_array(payload, "data_access_receipts")
    checks = _require_json_array(payload, "checks")
    failed_checks = _require_json_array(payload, "failed_checks")
    artifacts = _require_json_array(payload, "artifacts")
    for index, record in enumerate(prerequisite_locks):
        _validate_file_record_shape(
            record,
            label=f"prerequisite_locks[{index}]",
        )
    for index, record in enumerate(data_access_receipts):
        _validate_file_record_shape(
            record,
            label=f"data_access_receipts[{index}]",
        )
    for index, record in enumerate(artifacts):
        _validate_file_record_shape(
            record,
            label=f"artifacts[{index}]",
        )
    _validate_checks(checks, failed_checks, gate_status=payload["status"])

    if payload["status"] == "PASS":
        if payload["source_hashes_match"] is not True:
            raise GateSchemaError("PASS requires source_hashes_match=true")
        if failed_checks:
            raise GateSchemaError("PASS requires failed_checks=[]")
        expected_decision = CANONICAL_DECISIONS[payload["step_id"]]
        if payload.get("decision") != expected_decision:
            raise GateSchemaError(
                f"PASS step {payload['step_id']} requires "
                f"decision={expected_decision!r}"
            )
        if not data_access_receipts:
            raise GateSchemaError(
                "PASS requires at least one data_access_receipts file record"
            )
        if not checks:
            raise GateSchemaError("PASS requires at least one check record")
        if not artifacts:
            raise GateSchemaError("PASS requires at least one artifact file record")
    elif not failed_checks:
        raise GateSchemaError(
            f"{payload['status']} requires at least one failed_checks entry"
        )
    if require_pass and payload["status"] != "PASS":
        raise GateSchemaError(
            f"required PASS gate has terminal status {payload['status']}"
        )

    protected_fields = [
        field for field in _PROTECTED_SOURCE_FIELDS if field in payload
    ]
    if len(protected_fields) > 1:
        raise GateSchemaError(
            "gate must not define both source_hashes and protected_sources"
        )
    if payload["status"] == "PASS" and not protected_fields:
        raise GateSchemaError(
            "PASS requires a non-empty source_hashes or protected_sources collection"
        )
    if protected_fields:
        _records_from_collection(
            payload[protected_fields[0]],
            label=protected_fields[0],
            allow_empty=False,
        )
    if payload["status"] == "PASS" and "protocol" not in payload:
        raise GateSchemaError("PASS requires a protocol file record")
    if "protocol" in payload:
        protocol = _validate_file_record_shape(payload["protocol"], label="protocol")
        if protocol["sha256"] != payload["protocol_sha256"]:
            raise GateSchemaError(
                "protocol record hash differs from protocol_sha256"
            )
    if payload["status"] == "PASS" and "candidate" not in payload:
        raise GateSchemaError("PASS requires a candidate file record")
    if "candidate" in payload:
        _validate_file_record_shape(payload["candidate"], label="candidate")

    return dict(payload)


def sha256_file(path: str | Path) -> str:
    """Hash a regular file in bounded memory."""

    source = Path(path)
    if not source.is_file():
        raise HashVerificationError(f"protected file is missing: {source}")
    digest = hashlib.sha256()
    try:
        with source.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
    except OSError as exc:
        raise HashVerificationError(f"cannot hash protected file {source}: {exc}") from exc
    return digest.hexdigest()


def resolve_repo_relative_path(path: str | Path, repo_root: str | Path) -> Path:
    """Resolve a portable repository-relative path without allowing escape."""

    root = Path(repo_root).resolve()
    raw = str(path).strip()
    if not raw:
        raise HashVerificationError("protected path must not be empty")
    windows_path = PureWindowsPath(raw)
    if (
        Path(raw).is_absolute()
        or windows_path.is_absolute()
        or bool(windows_path.drive)
    ):
        raise HashVerificationError(
            f"protected path must be repository-relative: {raw!r}"
        )
    portable = PurePosixPath(raw.replace("\\", "/"))
    if not portable.parts or any(part == ".." for part in portable.parts):
        raise HashVerificationError(
            f"protected path escapes repository root: {raw!r}"
        )
    try:
        resolved = root.joinpath(*portable.parts).resolve(strict=False)
        resolved.relative_to(root)
    except (OSError, RuntimeError, ValueError) as exc:
        raise HashVerificationError(
            f"protected path escapes repository root: {raw!r}"
        ) from exc
    return resolved


def source_record(path: str | Path, repo_root: str | Path) -> dict[str, Any]:
    """Create a canonical repository-relative path/hash/size record."""

    root = Path(repo_root).resolve()
    source = Path(path)
    if not source.is_absolute():
        source = resolve_repo_relative_path(source, root)
    else:
        source = source.resolve(strict=False)
        try:
            source.relative_to(root)
        except ValueError as exc:
            raise HashVerificationError(
                f"source lies outside repository root: {source}"
            ) from exc
    if not source.is_file():
        raise HashVerificationError(f"protected file is missing: {source}")
    return {
        "path": source.relative_to(root).as_posix(),
        "sha256": sha256_file(source),
        "size_bytes": source.stat().st_size,
    }


def verify_file_record(
    record: Mapping[str, Any],
    repo_root: str | Path,
    *,
    label: str = "protected_file",
) -> Path:
    """Verify one repository-relative ``{path, sha256}`` record."""

    validated = _validate_file_record_shape(record, label=label)
    source = resolve_repo_relative_path(validated["path"], repo_root)
    if not source.is_file():
        raise HashVerificationError(
            f"{label} is missing: {validated['path']}"
        )
    if "size_bytes" in validated and source.stat().st_size != validated["size_bytes"]:
        raise HashVerificationError(
            f"{label} size mismatch for {validated['path']}: "
            f"expected {validated['size_bytes']}, observed {source.stat().st_size}"
        )
    observed = sha256_file(source)
    if not hmac.compare_digest(observed, validated["sha256"]):
        raise HashVerificationError(
            f"{label} SHA-256 mismatch for {validated['path']}: "
            f"expected {validated['sha256']}, observed {observed}"
        )
    return source


def verify_source_hashes(
    records: Any,
    repo_root: str | Path,
    *,
    label: str = "source_hashes",
) -> bool:
    """Re-hash every protected source record, raising on the first mismatch."""

    normalized_paths: set[Path] = set()
    for record_label, record in _records_from_collection(
        records,
        label=label,
        allow_empty=False,
    ):
        resolved = verify_file_record(record, repo_root, label=record_label)
        if resolved in normalized_paths:
            raise HashVerificationError(
                f"{label} contains duplicate protected path {record['path']!r}"
            )
        normalized_paths.add(resolved)
    return True


def verify_gate_bindings(
    payload: Mapping[str, Any],
    repo_root: str | Path,
) -> bool:
    """Verify protocol, protected-source, and candidate bindings if embedded."""

    gate = validate_gate_payload(payload)
    if "protocol" in gate:
        verify_file_record(gate["protocol"], repo_root, label="protocol")
    protected_fields = [
        field for field in _PROTECTED_SOURCE_FIELDS if field in gate
    ]
    if protected_fields:
        verify_source_hashes(
            gate[protected_fields[0]],
            repo_root,
            label=protected_fields[0],
        )
    if "candidate" in gate:
        verify_file_record(gate["candidate"], repo_root, label="candidate")
    for index, receipt in enumerate(gate["data_access_receipts"]):
        verify_file_record(
            receipt,
            repo_root,
            label=f"data_access_receipts[{index}]",
        )
    for index, artifact in enumerate(gate["artifacts"]):
        verify_file_record(
            artifact,
            repo_root,
            label=f"artifacts[{index}]",
        )
    return True


def _expected_reference_value(
    reference: Mapping[str, Any],
    primary_name: str,
    alias_name: str,
) -> Any:
    primary = reference.get(primary_name)
    alias = reference.get(alias_name)
    if primary is not None and alias is not None and primary != alias:
        raise PrerequisiteLockError(
            f"conflicting {primary_name}/{alias_name} prerequisite constraints"
        )
    return primary if primary is not None else alias


def _verify_prerequisite_lock(
    reference: Mapping[str, Any],
    repo_root: str | Path,
    *,
    expected_step_id: str | None,
    expected_decision: str | None,
    active: set[Path],
    verified: dict[Path, dict[str, Any]],
) -> dict[str, Any]:
    try:
        lock_path = verify_file_record(
            reference,
            repo_root,
            label="prerequisite_lock",
        )
    except (GateSchemaError, HashVerificationError) as exc:
        raise PrerequisiteLockError(str(exc)) from exc

    if lock_path in active:
        raise PrerequisiteLockError(
            f"cyclic prerequisite lock chain at {lock_path}"
        )
    gate = verified.get(lock_path)
    if gate is None:
        active.add(lock_path)
        try:
            raw = load_json_strict(lock_path)
            if not isinstance(raw, Mapping):
                raise PrerequisiteLockError(
                    f"prerequisite lock root is not an object: {lock_path}"
                )
            try:
                gate = validate_gate_payload(raw, require_pass=True)
                verify_gate_bindings(gate, repo_root)
            except GatekeeperError as exc:
                raise PrerequisiteLockError(
                    f"invalid prerequisite lock {lock_path}: {exc}"
                ) from exc
            if gate["source_hashes_match"] is not True:
                raise PrerequisiteLockError(
                    f"prerequisite lock does not certify source hashes: {lock_path}"
                )
            _verify_prerequisite_collection(
                gate["prerequisite_locks"],
                repo_root,
                downstream_step_id=gate["step_id"],
                enforce_complete_chain=True,
                active=active,
                verified=verified,
            )
            verified[lock_path] = gate
        finally:
            active.remove(lock_path)

    pinned_step = _expected_reference_value(
        reference,
        "step_id",
        "expected_step_id",
    )
    required_step = expected_step_id if expected_step_id is not None else pinned_step
    if (
        expected_step_id is not None
        and pinned_step is not None
        and pinned_step != expected_step_id
    ):
        raise PrerequisiteLockError(
            "caller/reference prerequisite step constraints disagree"
        )
    if required_step is not None and gate["step_id"] != required_step:
        raise PrerequisiteLockError(
            f"prerequisite step mismatch: expected {required_step!r}, "
            f"observed {gate['step_id']!r}"
        )

    pinned_decision = _expected_reference_value(
        reference,
        "decision",
        "expected_decision",
    )
    required_decision = (
        expected_decision if expected_decision is not None else pinned_decision
    )
    if (
        expected_decision is not None
        and pinned_decision is not None
        and pinned_decision != expected_decision
    ):
        raise PrerequisiteLockError(
            "caller/reference prerequisite decision constraints disagree"
        )
    if required_decision is not None and gate.get("decision") != required_decision:
        raise PrerequisiteLockError(
            f"prerequisite decision mismatch: expected {required_decision!r}, "
            f"observed {gate.get('decision')!r}"
        )
    expected_protocol = reference.get("protocol_sha256")
    if (
        expected_protocol is not None
        and gate["protocol_sha256"] != expected_protocol
    ):
        raise PrerequisiteLockError("prerequisite protocol hash constraint failed")
    return gate


def verify_prerequisite_lock(
    reference: Mapping[str, Any],
    repo_root: str | Path,
    *,
    expected_step_id: str | None = None,
    expected_decision: str | None = None,
) -> dict[str, Any]:
    """Verify one immutable PASS lock and its full prerequisite chain."""

    return _verify_prerequisite_lock(
        reference,
        repo_root,
        expected_step_id=expected_step_id,
        expected_decision=expected_decision,
        active=set(),
        verified={},
    )


def _verify_prerequisite_collection(
    references: Any,
    repo_root: str | Path,
    *,
    downstream_step_id: str | None,
    enforce_complete_chain: bool,
    active: set[Path],
    verified: dict[Path, dict[str, Any]],
) -> list[dict[str, Any]]:
    if type(references) is not list:
        raise PrerequisiteLockError("prerequisite_locks must be a JSON array")
    gates: list[dict[str, Any]] = []
    paths: set[Path] = set()
    for index, reference in enumerate(references):
        if not isinstance(reference, Mapping):
            raise PrerequisiteLockError(
                f"prerequisite_locks[{index}] must be an object"
            )
        try:
            resolved = resolve_repo_relative_path(reference.get("path", ""), repo_root)
        except HashVerificationError as exc:
            raise PrerequisiteLockError(str(exc)) from exc
        if resolved in paths:
            raise PrerequisiteLockError(
                f"duplicate prerequisite lock path {reference.get('path')!r}"
            )
        paths.add(resolved)
        gates.append(
            _verify_prerequisite_lock(
                reference,
                repo_root,
                expected_step_id=None,
                expected_decision=None,
                active=active,
                verified=verified,
            )
        )

    observed_steps = [gate["step_id"] for gate in gates]
    if len(set(observed_steps)) != len(observed_steps):
        raise PrerequisiteLockError(
            f"duplicate prerequisite step IDs: {observed_steps}"
        )
    if downstream_step_id is not None:
        if downstream_step_id not in STEP_IDS:
            raise PrerequisiteLockError(
                f"invalid downstream step ID {downstream_step_id!r}"
            )
        if any(int(step_id) >= int(downstream_step_id) for step_id in observed_steps):
            raise PrerequisiteLockError(
                f"prerequisite is not upstream of step {downstream_step_id}: "
                f"{observed_steps}"
            )
        if enforce_complete_chain:
            expected = _EXPECTED_DIRECT_PREREQUISITES[downstream_step_id]
            if frozenset(observed_steps) != expected:
                raise PrerequisiteLockError(
                    f"step {downstream_step_id} requires direct prerequisite "
                    f"steps {sorted(expected)}, observed {sorted(observed_steps)}"
                )
    _verify_single_lineage(references, repo_root, verified)
    return gates


def _verify_single_lineage(
    references: Sequence[Mapping[str, Any]],
    repo_root: str | Path,
    verified: Mapping[Path, Mapping[str, Any]],
) -> None:
    """Reject mixing distinct immutable locks for the same prerequisite step."""

    identities: dict[str, set[tuple[Path, str]]] = {
        step_id: set() for step_id in STEP_IDS
    }
    visited: set[tuple[Path, str]] = set()

    def visit(reference: Mapping[str, Any]) -> None:
        try:
            path = resolve_repo_relative_path(reference["path"], repo_root)
            digest = reference["sha256"]
        except (KeyError, HashVerificationError) as exc:
            raise PrerequisiteLockError(
                "cannot resolve prerequisite lineage identity"
            ) from exc
        identity = (path, digest)
        if identity in visited:
            return
        visited.add(identity)
        gate = verified.get(path)
        if gate is None:
            raise PrerequisiteLockError(
                f"prerequisite lineage was not verified: {path}"
            )
        step_id = gate["step_id"]
        identities[step_id].add(identity)
        for nested in gate["prerequisite_locks"]:
            visit(nested)

    for reference in references:
        visit(reference)
    incoherent = {
        step_id: sorted(
            f"{path.as_posix()}@{digest}" for path, digest in values
        )
        for step_id, values in identities.items()
        if len(values) > 1
    }
    if incoherent:
        raise PrerequisiteLockError(
            f"incoherent prerequisite lineage: {incoherent}"
        )


def verify_prerequisite_locks(
    references: Sequence[Mapping[str, Any]],
    repo_root: str | Path,
    *,
    downstream_step_id: str | None = None,
    enforce_complete_chain: bool = False,
) -> list[dict[str, Any]]:
    """Verify prerequisite PASS locks, optionally enforcing the 0 -> 1 -> 2 chain."""

    return _verify_prerequisite_collection(
        references,
        repo_root,
        downstream_step_id=downstream_step_id,
        enforce_complete_chain=enforce_complete_chain,
        active=set(),
        verified={},
    )


def verify_gate_file(
    path: str | Path,
    repo_root: str | Path,
    *,
    expected_sha256: str | None = None,
    expected_step_id: str | None = None,
    expected_protocol_sha256: str | None = None,
    require_pass: bool = False,
    verify_prerequisites: bool = True,
) -> dict[str, Any]:
    """Load a gate strictly and verify all available immutable bindings."""

    gate_path = Path(path)
    if not gate_path.is_absolute():
        gate_path = resolve_repo_relative_path(gate_path, repo_root)
    else:
        root = Path(repo_root).resolve()
        gate_path = gate_path.resolve(strict=False)
        try:
            gate_path.relative_to(root)
        except ValueError as exc:
            raise HashVerificationError(
                f"gate lies outside repository root: {gate_path}"
            ) from exc
    if expected_sha256 is not None:
        if not is_sha256(expected_sha256):
            raise HashVerificationError("expected gate SHA-256 is not canonical")
        observed = sha256_file(gate_path)
        if not hmac.compare_digest(observed, expected_sha256):
            raise HashVerificationError(
                f"gate SHA-256 mismatch for {gate_path}: "
                f"expected {expected_sha256}, observed {observed}"
            )
    raw = load_json_strict(gate_path)
    if not isinstance(raw, Mapping):
        raise GateSchemaError(f"gate root is not an object: {gate_path}")
    gate = validate_gate_payload(
        raw,
        expected_step_id=expected_step_id,
        expected_protocol_sha256=expected_protocol_sha256,
        require_pass=require_pass,
    )
    verify_gate_bindings(gate, repo_root)
    if verify_prerequisites:
        verify_prerequisite_locks(
            gate["prerequisite_locks"],
            repo_root,
            downstream_step_id=gate["step_id"],
            enforce_complete_chain=True,
        )
    return gate


def _serialize_json(payload: Any) -> bytes:
    try:
        _validate_json_value(payload)
        text = json.dumps(
            payload,
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
    except (StrictJSONError, TypeError, ValueError) as exc:
        raise StrictJSONError(f"cannot serialize strict JSON: {exc}") from exc
    return f"{text}\n".encode("utf-8")


def write_json_no_clobber(path: str | Path, payload: Any) -> Path:
    """Atomically publish one immutable JSON file without replacing a path."""

    destination = Path(path)
    encoded = _serialize_json(payload)
    destination.parent.mkdir(parents=True, exist_ok=True)
    if os.path.lexists(destination):
        raise NoClobberError(f"refusing existing output: {destination}")

    staging = destination.parent / (
        f".{destination.name}.tmp-{os.getpid()}-{uuid.uuid4().hex}"
    )
    descriptor: int | None = None
    try:
        descriptor = os.open(
            staging,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL,
            0o644,
        )
        with os.fdopen(descriptor, "wb") as stream:
            descriptor = None
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.link(staging, destination)
        except FileExistsError as exc:
            raise NoClobberError(
                f"refusing existing output: {destination}"
            ) from exc
    finally:
        if descriptor is not None:
            os.close(descriptor)
        try:
            staging.unlink()
        except FileNotFoundError:
            pass
    return destination


def claim_output_directory(path: str | Path) -> Path:
    """Create a new output directory and refuse any pre-existing destination."""

    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        output.mkdir()
    except FileExistsError as exc:
        raise NoClobberError(f"refusing occupied output directory: {output}") from exc
    return output


def write_gate_status_no_clobber(
    path: str | Path,
    payload: Mapping[str, Any],
) -> Path:
    """Validate schema v1 and immutably publish a terminal gate status."""

    validated = validate_gate_payload(payload)
    return write_json_no_clobber(path, validated)


def _json_array_copy(value: Sequence[Any], *, label: str) -> list[Any]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise GateSchemaError(f"{label} must be an array")
    copied = list(value)
    try:
        _validate_json_value(copied, location=label)
    except StrictJSONError as exc:
        raise GateSchemaError(str(exc)) from exc
    return copied


def build_failure_payload(
    *,
    step_id: str,
    status: str,
    protocol_sha256: str,
    phase: str,
    last_data_access: Mapping[str, Any] | None,
    failed_checks: Sequence[Any],
    artifacts: Sequence[Any],
    source_hashes_match: bool,
    prerequisite_locks: Sequence[Mapping[str, Any]] = (),
    data_access_receipts: Sequence[Any] = (),
    checks: Sequence[Any] = (),
    error: str | None = None,
    extra: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Build a uniform terminal ``failure.json`` payload.

    ``status`` must be ``FAIL``, ``BLOCKED``, or ``ERROR``.  A missing
    ``last_data_access`` is represented explicitly as no semantic access,
    rather than being omitted.
    """

    if status not in TERMINAL_STATUSES.difference({"PASS"}):
        raise GateSchemaError(
            "failure payload status must be FAIL, BLOCKED, or ERROR"
        )
    if not isinstance(phase, str) or not phase.strip():
        raise GateSchemaError("failure phase must be a non-empty string")
    if last_data_access is None:
        access: Mapping[str, Any] = {"semantic_access_started": False}
    elif not isinstance(last_data_access, Mapping):
        raise GateSchemaError("last_data_access must be an object or null")
    else:
        access = dict(last_data_access)
    if type(access.get("semantic_access_started")) is not bool:
        raise GateSchemaError(
            "last_data_access.semantic_access_started must be a boolean"
        )

    preserved = _json_array_copy(artifacts, label="artifacts")
    failed = _json_array_copy(failed_checks, label="failed_checks")
    check_records = _json_array_copy(checks, label="checks")
    if not check_records:
        check_records = [
            {"id": identifier, "status": status}
            for identifier in failed
        ]
    payload: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "step_id": step_id,
        "status": status,
        "protocol_sha256": protocol_sha256,
        "source_hashes_match": source_hashes_match,
        "prerequisite_locks": _json_array_copy(
            prerequisite_locks,
            label="prerequisite_locks",
        ),
        "data_access_receipts": _json_array_copy(
            data_access_receipts,
            label="data_access_receipts",
        ),
        "checks": check_records,
        "failed_checks": failed,
        "artifacts": preserved,
        "phase": phase.strip(),
        "last_data_access": access,
        "artifacts_preserved": list(preserved),
    }
    if error is not None:
        if not isinstance(error, str) or not error:
            raise GateSchemaError("error must be a non-empty string when provided")
        payload["error"] = error
    if extra is not None:
        if not isinstance(extra, Mapping):
            raise GateSchemaError("failure extra fields must be an object")
        collisions = sorted(set(extra).intersection(payload))
        if collisions:
            raise GateSchemaError(
                f"failure extra fields collide with reserved fields: {collisions}"
            )
        payload.update(extra)
    return validate_gate_payload(payload)


def write_failure_json(
    output_directory: str | Path,
    **kwargs: Any,
) -> Path:
    """Build and immutably write ``failure.json`` below *output_directory*."""

    payload = build_failure_payload(**kwargs)
    return write_json_no_clobber(Path(output_directory) / "failure.json", payload)


__all__ = [
    "CANONICAL_DECISIONS",
    "GateSchemaError",
    "GatekeeperError",
    "HashVerificationError",
    "NoClobberError",
    "PrerequisiteLockError",
    "REQUIRED_GATE_FIELDS",
    "SCHEMA_VERSION",
    "STEP_IDS",
    "StrictJSONError",
    "TERMINAL_STATUSES",
    "build_failure_payload",
    "claim_output_directory",
    "is_sha256",
    "load_json_strict",
    "loads_json_strict",
    "resolve_repo_relative_path",
    "sha256_file",
    "source_record",
    "validate_gate_payload",
    "verify_file_record",
    "verify_gate_bindings",
    "verify_gate_file",
    "verify_prerequisite_lock",
    "verify_prerequisite_locks",
    "verify_source_hashes",
    "write_failure_json",
    "write_gate_status_no_clobber",
    "write_json_no_clobber",
]
