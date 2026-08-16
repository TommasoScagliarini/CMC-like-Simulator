"""Audit and exclusively publish the additive V12R5 protocol freeze.

This module is deliberately source-only: it imports neither RLlib nor OpenSim,
does not fit a model, and does not reset or step an environment.  The protocol
can be published only after both the historical Q2 design and the separate Q3
design freeze are present, while every Q2/Q3 execution output remains absent.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import stat
import sys
import tempfile
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence


def _discover_repo_root(source: Path) -> Path:
    for candidate in source.resolve().parents:
        if (
            (candidate / "AGENTS.md").is_file()
            and (candidate / "validation").is_dir()
            and (candidate / "Trajectory Generator").is_dir()
        ):
            return candidate
    raise RuntimeError("repository root could not be discovered")


REPO_ROOT = _discover_repo_root(Path(__file__))
REVISION_ROOT = Path(__file__).resolve().parent
if str(REVISION_ROOT) not in sys.path:
    sys.path.insert(0, str(REVISION_ROOT))

import h0_v12r5_case_balanced_contract as contract  # noqa: E402


PROTOCOL_FREEZE_FILE = REPO_ROOT.joinpath(*contract.PROTOCOL_FREEZE_PATH.parts)
DESIGN_AUDIT_FILE = REPO_ROOT.joinpath(*contract.DESIGN_AUDIT_PATH.parts)
PROTOCOL_PUBLICATION_FAILURE_FILE = REPO_ROOT.joinpath(
    *contract.PROTOCOL_PUBLICATION_FAILURE_PATH.parts
)

_DESIGN_AUDIT_FIELDS = frozenset(
    {
        "schema_version",
        "status",
        "passed",
        "protocol_id",
        "protocol_freeze",
        "contract_self_check",
        "source_precondition_gate",
        "environment_reset_calls",
        "environment_step_calls",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
    }
)
_PROTOCOL_FREEZE_FIELDS = frozenset(
    {
        "schema_version",
        "status",
        "passed",
        "protocol_id",
        "pipeline_id",
        "revision",
        "authority",
        "source_precondition_gate",
        "source_records",
        "locked_inputs",
        "stage_order",
        "new_collection_stages",
        "new_collection_count",
        "corpus_counts",
        "weighting",
        "critical_window",
        "fit",
        "candidate_selection",
        "development_cases",
        "q2_historical_design_freeze",
        "q2_unopened_paths",
        "q3_design_freeze",
        "q3_unopened_paths",
        "q3_prerequisites",
        "execution_lock_created",
        "pipeline_claim_created",
        "environment_reset_calls",
        "environment_step_calls",
        "raw_sensor_sample_count",
        "teacher_query_count",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
        "runtime_promoted",
        "checkpoint_zero_created",
        "positive_morphology_enabled",
    }
)
_PROTOCOL_PUBLICATION_FAILURE_FIELDS = frozenset(
    {
        "schema_version",
        "status",
        "passed",
        "terminal",
        "protocol_id",
        "pipeline_id",
        "phase",
        "expected_protocol_freeze",
        "expected_design_audit",
        "protocol_freeze_observation",
        "design_audit_observation",
        "error",
        "q2_paths_opened",
        "q3_paths_opened",
        "qualification_violation_detected",
        "new_collection_count",
        "environment_reset_calls",
        "environment_step_calls",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
        "retry_authorized",
        "resume_authorized",
        "rescue_authorized",
        "sweep_authorized",
        "qualification_executed",
        "runtime_promoted",
        "checkpoint_zero_created",
        "positive_morphology_enabled",
        "next_stage",
    }
)


class V12R5ProtocolFreezeError(RuntimeError):
    """Raised when V12R5 source evidence is unsafe or inconsistent."""


def resolve_relative(path: str | os.PathLike[str] | PurePosixPath) -> Path:
    raw = path.as_posix() if isinstance(path, PurePosixPath) else os.fspath(path)
    pure = PurePosixPath(raw)
    if not raw or pure.is_absolute() or ".." in pure.parts or pure.as_posix() != raw:
        raise V12R5ProtocolFreezeError(
            f"non-canonical repository-relative path: {raw!r}"
        )
    return REPO_ROOT.joinpath(*pure.parts)


def _regular_file_no_links(path: Path) -> bool:
    try:
        info = path.lstat()
    except OSError:
        return False
    return stat.S_ISREG(info.st_mode) and not path.is_symlink()


def sha256_file(path: Path) -> str:
    if not _regular_file_no_links(path):
        raise V12R5ProtocolFreezeError(f"unsafe or missing regular file: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def artifact_record(path: str | os.PathLike[str] | PurePosixPath) -> dict[str, Any]:
    resolved = resolve_relative(path)
    return {
        "path": resolved.relative_to(REPO_ROOT).as_posix(),
        "sha256": sha256_file(resolved),
        "size_bytes": resolved.stat().st_size,
    }


def _canonical_json_bytes(payload: Mapping[str, Any]) -> bytes:
    try:
        return (
            json.dumps(
                dict(payload),
                indent=2,
                sort_keys=True,
                ensure_ascii=False,
                allow_nan=False,
            )
            + "\n"
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise V12R5ProtocolFreezeError("payload is not canonical strict JSON") from exc


def _logical_path(path: str | os.PathLike[str] | PurePosixPath) -> str:
    return resolve_relative(path).relative_to(REPO_ROOT).as_posix()


def _expected_artifact_record(
    path: str | os.PathLike[str] | PurePosixPath,
    payload: Mapping[str, Any],
) -> dict[str, Any]:
    encoded = _canonical_json_bytes(payload)
    return {
        "path": _logical_path(path),
        "sha256": hashlib.sha256(encoded).hexdigest(),
        "size_bytes": len(encoded),
    }


def _raw_artifact_record(path: Path, *, logical_path: str) -> dict[str, Any]:
    if not _regular_file_no_links(path):
        raise V12R5ProtocolFreezeError(f"unsafe or missing regular file: {path}")
    data = path.read_bytes()
    return {
        "path": logical_path,
        "sha256": hashlib.sha256(data).hexdigest(),
        "size_bytes": len(data),
    }


def _publication_observation(
    path: Path, *, expected_record: Mapping[str, Any]
) -> dict[str, Any]:
    """Bind raw bytes without requiring a partially published file to parse."""

    if not os.path.lexists(path):
        return {"state": "ABSENT", "artifact": None}
    if path.is_symlink() or not _regular_file_no_links(path):
        return {"state": "UNSAFE_NONREGULAR", "artifact": None}
    try:
        record = _raw_artifact_record(
            path, logical_path=str(expected_record.get("path"))
        )
    except BaseException:
        return {"state": "UNSAFE_UNREADABLE_REGULAR", "artifact": None}
    return {
        "state": (
            "VALID_REGULAR"
            if record == dict(expected_record)
            else "PARTIAL_OR_INVALID_REGULAR"
        ),
        "artifact": record,
    }


def _fsync_directory(path: Path) -> None:
    """Best-effort durability barrier where directory descriptors are supported."""

    try:
        descriptor = os.open(path, os.O_RDONLY)
    except OSError:  # pragma: no cover - platform/filesystem dependent.
        return
    try:
        os.fsync(descriptor)
    except OSError:  # pragma: no cover - platform/filesystem dependent.
        pass
    finally:
        os.close(descriptor)


def _error_record(error: BaseException) -> dict[str, str]:
    message = str(error) or repr(error) or type(error).__name__
    return {"type": type(error).__name__, "message": message}


def _strict_equal(left: Any, right: Any) -> bool:
    """Compare JSON-like values without bool/int/float coercion."""

    if type(left) is not type(right):
        return False
    if isinstance(left, Mapping):
        return set(left) == set(right) and all(
            _strict_equal(left[key], right[key]) for key in left
        )
    if isinstance(left, (list, tuple)):
        return len(left) == len(right) and all(
            _strict_equal(a, b) for a, b in zip(left, right, strict=True)
        )
    return bool(left == right)


def tree_record(path: str | os.PathLike[str] | PurePosixPath) -> dict[str, Any]:
    root = resolve_relative(path)
    if not root.is_dir() or root.is_symlink():
        raise V12R5ProtocolFreezeError(f"unsafe or missing tree: {root}")
    files = sorted(item for item in root.rglob("*") if item.is_file())
    if not files or any(item.is_symlink() for item in files):
        raise V12R5ProtocolFreezeError(f"empty or linked artifact tree: {root}")
    digest = hashlib.sha256()
    rows: list[dict[str, Any]] = []
    for item in files:
        relative = item.relative_to(root).as_posix()
        sha256 = sha256_file(item)
        size_bytes = item.stat().st_size
        rows.append({"path": relative, "sha256": sha256, "size_bytes": size_bytes})
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(sha256.encode("ascii"))
        digest.update(b"\0")
        digest.update(str(size_bytes).encode("ascii"))
        digest.update(b"\n")
    return {
        "path": root.relative_to(REPO_ROOT).as_posix(),
        "tree_sha256": digest.hexdigest(),
        "file_count": len(rows),
        "files": rows,
    }


def _load_mapping(path: str | os.PathLike[str] | PurePosixPath) -> dict[str, Any]:
    resolved = resolve_relative(path)
    return _load_mapping_file(resolved)


def _load_mapping_file(resolved: Path) -> dict[str, Any]:
    if not _regular_file_no_links(resolved):
        raise V12R5ProtocolFreezeError(
            f"cannot read unsafe/nonregular JSON: {resolved}"
        )
    try:
        value = json.loads(resolved.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise V12R5ProtocolFreezeError(f"cannot read strict JSON: {resolved}") from exc
    if not isinstance(value, dict):
        raise V12R5ProtocolFreezeError(f"JSON root is not an object: {resolved}")
    return value


def _records_exact(records: Mapping[str, Mapping[str, Any]]) -> bool:
    return all(
        artifact_record(value["path"]) == dict(value) for value in records.values()
    )


def _safe_environment_source_closure() -> dict[str, dict[str, Any]]:
    if (
        artifact_record(contract.SAFE_V8R1P1_EXECUTION_LOCK_ARTIFACT["path"])
        != contract.SAFE_V8R1P1_EXECUTION_LOCK_ARTIFACT
    ):
        raise V12R5ProtocolFreezeError("safe V8R1P1 execution lock drifted")
    lock = _load_mapping(contract.SAFE_V8R1P1_EXECUTION_LOCK_ARTIFACT["path"])
    sources = lock.get("sources")
    if not isinstance(sources, Mapping) or len(sources) != 29:
        raise V12R5ProtocolFreezeError("safe V8R1P1 source closure is malformed")
    closure: dict[str, dict[str, Any]] = {}
    for name, value in sorted(sources.items()):
        if not isinstance(name, str) or not isinstance(value, Mapping):
            raise V12R5ProtocolFreezeError("safe environment source row is malformed")
        record = dict(value)
        if set(record) != {"path", "sha256", "size_bytes"}:
            raise V12R5ProtocolFreezeError("safe environment source schema drifted")
        if artifact_record(str(record["path"])) != record:
            raise V12R5ProtocolFreezeError(f"safe environment source drifted: {name}")
        closure[name] = record
    return closure


def _locked_record_checks() -> dict[str, bool]:
    return {
        "p2_corpus": artifact_record(contract.P2_CORPUS_ARTIFACT["path"])
        == contract.P2_CORPUS_ARTIFACT,
        "p2_report": artifact_record(contract.P2_ADAPTATION_REPORT_ARTIFACT["path"])
        == contract.P2_ADAPTATION_REPORT_ARTIFACT,
        "p2_history": artifact_record(contract.P2_ADAPTATION_HISTORY_ARTIFACT["path"])
        == contract.P2_ADAPTATION_HISTORY_ARTIFACT,
        "p2_module": tree_record(contract.P2_MODULE_TREE["path"])
        == contract.P2_MODULE_TREE,
        "source_h0": tree_record(contract.SOURCE_H0_MODULE_PATH).get("tree_sha256")
        == contract.SOURCE_H0_TREE_SHA256,
        "r4_terminal": _records_exact(contract.R4_TERMINAL_ARTIFACTS),
        "r4_nominal_pass": _records_exact(contract.R4_NOMINAL_REUSABLE_ARTIFACTS),
        "r4_plus_failure": _records_exact(contract.R4_PLUS_FAILURE_EVIDENCE),
        "safe_plus_reference": _records_exact(
            contract.SAFE_V8R1P1_PLUS_REPLAY_ARTIFACTS
        ),
        "safe_environment_execution_lock": artifact_record(
            contract.SAFE_V8R1P1_EXECUTION_LOCK_ARTIFACT["path"]
        )
        == contract.SAFE_V8R1P1_EXECUTION_LOCK_ARTIFACT,
        "safe_environment_29_source_closure": len(_safe_environment_source_closure())
        == 29,
        "external_runtime_sources": _records_exact(
            contract.FROZEN_EXTERNAL_RUNTIME_SOURCES
        ),
        "production_source_closure": _records_exact(_production_source_closure())
        and len(_production_source_closure())
        == contract.EXPECTED_PRODUCTION_SOURCE_COUNT,
        "q2_design": artifact_record(contract.Q2_DESIGN_FREEZE_ARTIFACT["path"])
        == contract.Q2_DESIGN_FREEZE_ARTIFACT,
        "q3_design": artifact_record(contract.Q3_DESIGN_FREEZE_ARTIFACT["path"])
        == contract.Q3_DESIGN_FREEZE_ARTIFACT,
    }


def _locked_semantics() -> dict[str, bool]:
    r4 = _load_mapping(contract.R4_TERMINAL_ARTIFACTS["terminal_ledger"]["path"])
    nominal_receipt = _load_mapping(
        contract.R4_NOMINAL_REUSABLE_ARTIFACTS["receipt"]["path"]
    )
    nominal_gate = _load_mapping(contract.R4_NOMINAL_REUSABLE_ARTIFACTS["gate"]["path"])
    nominal_summary = _load_mapping(
        contract.R4_NOMINAL_REUSABLE_ARTIFACTS["summary"]["path"]
    )
    plus_failure = _load_mapping(contract.R4_PLUS_FAILURE_EVIDENCE["failure"]["path"])
    plus_gate = _load_mapping(contract.R4_PLUS_FAILURE_EVIDENCE["gate"]["path"])
    plus_summary = _load_mapping(contract.R4_PLUS_FAILURE_EVIDENCE["summary"]["path"])
    safe_gate = _load_mapping(
        contract.SAFE_V8R1P1_PLUS_REPLAY_ARTIFACTS["gate"]["path"]
    )
    safe_summary = _load_mapping(
        contract.SAFE_V8R1P1_PLUS_REPLAY_ARTIFACTS["summary"]["path"]
    )
    return {
        "r4_terminal_failure": r4.get("status") == "FAIL_H0_V12R4_P3_COVERAGE_TERMINAL"
        and r4.get("passed") is False
        and r4.get("attempted_stage") == "collect_cov__deterministic_offset_plus_0p20"
        and r4.get("retry_authorized") is False
        and r4.get("resume_authorized") is False,
        "nominal_pass_reusable": nominal_receipt.get("passed") is True
        and nominal_receipt.get("sample_count") == contract.NOMINAL_PASS_ROWS
        and nominal_gate.get("passed") is True
        and nominal_gate.get("collection_data_reusable") is True
        and nominal_summary.get("steps") == contract.NOMINAL_PASS_ROWS,
        "failed_plus_forensic_only": plus_failure.get("passed") is False
        and plus_failure.get("last_completed_step") == 212
        and plus_gate.get("passed") is False
        and plus_gate.get("collection_data_reusable") is False
        and plus_summary.get("steps") == 212,
        "safe_plus_reference_pass": safe_gate.get("passed") is True
        and safe_summary.get("steps") == contract.EXPECTED_STEPS,
    }


def _q2_design_semantics() -> dict[str, bool]:
    payload = _load_mapping(contract.Q2_DESIGN_FREEZE_ARTIFACT["path"])
    snapshot = payload.get("design_snapshot")
    return {
        "artifact_exact": artifact_record(contract.Q2_DESIGN_FREEZE_ARTIFACT["path"])
        == contract.Q2_DESIGN_FREEZE_ARTIFACT,
        "historical_identity": payload.get("status")
        == "PASS_H0_V12R4_Q2_QUALIFICATION_DESIGN_FREEZE"
        and payload.get("passed") is True,
        "candidate_deferred": payload.get("candidate_binding_state") == "DEFERRED"
        and payload.get("candidate_id") is None
        and payload.get("candidate_module") is None
        and isinstance(snapshot, Mapping)
        and snapshot.get("candidate_binding_state") == "DEFERRED"
        and snapshot.get("candidate_id") is None
        and snapshot.get("candidate_module") is None,
    }


def _q3_design_semantics() -> dict[str, bool]:
    path = resolve_relative(contract.Q3_DESIGN_FREEZE_PATH)
    if not _regular_file_no_links(path):
        return {
            "present_regular": False,
            "identity": False,
            "candidate_deferred": False,
            "r5_prerequisites_exact": False,
        }
    payload = _load_mapping(contract.Q3_DESIGN_FREEZE_PATH)
    snapshot = payload.get("design_snapshot")
    prerequisites = payload.get("future_prerequisite_requirements")
    if prerequisites is None and isinstance(snapshot, Mapping):
        prerequisites = snapshot.get("future_prerequisite_requirements")
    observed = {
        (row.get("path"), row.get("required_status"))
        for row in prerequisites
        if isinstance(prerequisites, list) and isinstance(row, Mapping)
    }
    expected = {
        (value["path"], value["status"]) for value in contract.Q3_PREREQUISITES.values()
    }
    candidate_deferred = (
        payload.get("candidate_binding_state") == "DEFERRED"
        and payload.get("candidate_id") is None
        and payload.get("candidate_module") is None
    )
    if isinstance(snapshot, Mapping):
        candidate_deferred = candidate_deferred and (
            snapshot.get("candidate_binding_state") == "DEFERRED"
            and snapshot.get("candidate_id") is None
            and snapshot.get("candidate_module") is None
        )
    return {
        "present_regular": True,
        "artifact_exact": artifact_record(contract.Q3_DESIGN_FREEZE_PATH)
        == contract.Q3_DESIGN_FREEZE_ARTIFACT,
        "identity": payload.get("status") == contract.Q3_DESIGN_FREEZE_STATUS
        and payload.get("passed") is True,
        "candidate_deferred": candidate_deferred,
        "r5_prerequisites_exact": len(observed) == 5 and observed == expected,
    }


def _output_absence() -> dict[str, bool]:
    paths = {
        "protocol_freeze": PROTOCOL_FREEZE_FILE,
        "design_audit": DESIGN_AUDIT_FILE,
        "protocol_publication_failure": PROTOCOL_PUBLICATION_FAILURE_FILE,
        "execution_lock": resolve_relative(contract.EXECUTION_LOCK_PATH),
        "execution_lock_publication_failure": resolve_relative(
            contract.EXECUTION_LOCK_PUBLICATION_FAILURE_PATH
        ),
        "run_root": resolve_relative(contract.RUN_ROOT),
        "pipeline_claim_failure": resolve_relative(
            contract.PIPELINE_CLAIM_FAILURE_PATH
        ),
        **{
            f"q2_{name}": resolve_relative(path)
            for name, path in contract.Q2_UNOPENED_PATHS.items()
        },
        **{
            f"q3_{name}": resolve_relative(path)
            for name, path in contract.Q3_UNOPENED_PATHS.items()
        },
    }
    return {name: not os.path.lexists(path) for name, path in paths.items()}


def source_precondition_gate() -> dict[str, Any]:
    """Recompute immutable inputs, Q3 design readiness, and output absence."""

    self_check = contract.contract_self_check()
    records = _locked_record_checks()
    semantics = _locked_semantics()
    q2 = _q2_design_semantics()
    q3 = _q3_design_semantics()
    absence = _output_absence()
    checks = {
        "contract": self_check.get("passed") is True,
        "locked_records": all(records.values()),
        "locked_semantics": all(semantics.values()),
        "q2_historical_design": all(q2.values()),
        "q3_design_preregistered": all(q3.values()),
        "outputs_unopened": all(absence.values()),
        "one_shot_no_collection_authority": all(
            contract.AUTHORITY[name]
            for name in (
                "source_implementation_authorized",
                "protocol_freeze_publication_authorized",
                "execution_lock_authorized",
                "actor_fit_execution_authorized",
                "candidate_freeze_authorized",
                "development_execution_authorized",
            )
        )
        and contract.AUTHORITY["new_environment_collection_authorized"] is False
        and not any(
            contract.AUTHORITY[name]
            for name in (
                "qualification_execution_authorized",
                "retry_authorized",
                "resume_authorized",
                "rescue_authorized",
                "sweep_authorized",
                "runtime_promotion_authorized",
                "checkpoint_zero_authorized",
                "positive_morphology_authorized",
            )
        ),
    }
    q3_record = (
        copy.deepcopy(contract.Q3_DESIGN_FREEZE_ARTIFACT) if all(q3.values()) else None
    )
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            "PASS_H0_V12R5_CASE_BALANCED_SOURCE_PRECONDITIONS"
            if all(checks.values())
            else "FAIL_H0_V12R5_CASE_BALANCED_SOURCE_PRECONDITIONS"
        ),
        "passed": all(checks.values()),
        "protocol_id": contract.PROTOCOL_ID,
        "checks": checks,
        "record_checks": records,
        "semantic_checks": semantics,
        "q2_design_checks": q2,
        "q3_design_checks": q3,
        "q2_design_freeze": copy.deepcopy(contract.Q2_DESIGN_FREEZE_ARTIFACT),
        "q3_design_freeze": q3_record,
        "safe_environment_source_closure": _safe_environment_source_closure(),
        "production_source_closure": _production_source_closure(),
        "absence_checks": absence,
        "q2_paths_opened": [
            name for name in contract.Q2_UNOPENED_PATHS if not absence[f"q2_{name}"]
        ],
        "q3_paths_opened": [
            name for name in contract.Q3_UNOPENED_PATHS if not absence[f"q3_{name}"]
        ],
    }


def _prepublication_source_gate_snapshot() -> dict[str, Any]:
    """Recompute the frozen gate independently of every downstream output."""

    gate = source_precondition_gate()
    absence = {name: True for name in gate["absence_checks"]}
    checks = dict(gate["checks"])
    checks["outputs_unopened"] = True
    passed = all(value is True for value in checks.values())
    gate.update(
        {
            "status": (
                "PASS_H0_V12R5_CASE_BALANCED_SOURCE_PRECONDITIONS"
                if passed
                else "FAIL_H0_V12R5_CASE_BALANCED_SOURCE_PRECONDITIONS"
            ),
            "passed": passed,
            "checks": checks,
            "absence_checks": absence,
            "q2_paths_opened": [],
            "q3_paths_opened": [],
        }
    )
    return gate


def _source_records() -> dict[str, dict[str, Any]]:
    expected = (
        ".gitattributes",
        "__init__.py",
        "conftest.py",
        "h0_v12r5_case_balanced_contract.py",
        "freeze_h0_v12r5_case_balanced.py",
        "h0_v12r5_case_balanced_fitter.py",
        "run_h0_v12r5_case_balanced.py",
        "test_h0_v12r5_case_balanced_contract.py",
        "test_freeze_h0_v12r5_case_balanced.py",
        "test_h0_v12r5_case_balanced_fitter.py",
        "test_run_h0_v12r5_case_balanced.py",
    )
    return {name: artifact_record(contract.VALIDATION_ROOT / name) for name in expected}


def _production_source_closure() -> dict[str, dict[str, Any]]:
    """Build the exact 64-external plus four-local production closure."""

    closure = copy.deepcopy(contract.FROZEN_EXTERNAL_RUNTIME_SOURCES)
    for name in contract.LOCAL_R5_PRODUCTION_SOURCE_NAMES:
        record = artifact_record(contract.VALIDATION_ROOT / name)
        path = str(record["path"])
        if path in closure:
            raise V12R5ProtocolFreezeError(f"duplicate production source: {path}")
        closure[path] = record
    if (
        len(closure) != contract.EXPECTED_PRODUCTION_SOURCE_COUNT
        or any(path != record["path"] for path, record in closure.items())
        or any(PurePosixPath(path).name.startswith("test_") for path in closure)
    ):
        raise V12R5ProtocolFreezeError("production source closure is not exact")
    return closure


def _production_source_closure_exact(observed: Any) -> bool:
    return isinstance(observed, Mapping) and dict(observed) == (
        _production_source_closure()
    )


def _expected_locked_inputs() -> dict[str, Any]:
    return {
        "source_h0": tree_record(contract.SOURCE_H0_MODULE_PATH),
        "p2_corpus": copy.deepcopy(contract.P2_CORPUS_ARTIFACT),
        "p2_module": copy.deepcopy(contract.P2_MODULE_TREE),
        "p2_adaptation_report": copy.deepcopy(contract.P2_ADAPTATION_REPORT_ARTIFACT),
        "p2_adaptation_history": copy.deepcopy(contract.P2_ADAPTATION_HISTORY_ARTIFACT),
        "r4_terminal": copy.deepcopy(contract.R4_TERMINAL_ARTIFACTS),
        "r4_nominal_pass_reusable": copy.deepcopy(
            contract.R4_NOMINAL_REUSABLE_ARTIFACTS
        ),
        "r4_plus_failure_forensic_only": copy.deepcopy(
            contract.R4_PLUS_FAILURE_EVIDENCE
        ),
        "safe_plus_reference": copy.deepcopy(
            contract.SAFE_V8R1P1_PLUS_REPLAY_ARTIFACTS
        ),
        "safe_environment_execution_lock": copy.deepcopy(
            contract.SAFE_V8R1P1_EXECUTION_LOCK_ARTIFACT
        ),
        "safe_environment_source_closure": _safe_environment_source_closure(),
        "external_runtime_sources": copy.deepcopy(
            contract.FROZEN_EXTERNAL_RUNTIME_SOURCES
        ),
        "production_source_closure": _production_source_closure(),
    }


def build_protocol_freeze_payload() -> dict[str, Any]:
    """Build the canonical pre-execution freeze payload in memory."""

    gate = source_precondition_gate()
    if gate.get("passed") is not True:
        failed = [name for name, value in gate["checks"].items() if not value]
        raise V12R5ProtocolFreezeError(f"V12R5 source preconditions failed: {failed}")
    return _protocol_freeze_payload_from_gate(gate)


def _protocol_freeze_payload_from_gate(
    gate: Mapping[str, Any],
) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PROTOCOL_FREEZE_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "revision": contract.REVISION,
        "authority": copy.deepcopy(contract.AUTHORITY),
        "source_precondition_gate": gate,
        "source_records": _source_records(),
        "locked_inputs": _expected_locked_inputs(),
        "stage_order": list(contract.STAGE_IDS),
        "new_collection_stages": [],
        "new_collection_count": 0,
        "corpus_counts": contract.expected_corpus_counts(),
        "weighting": copy.deepcopy(contract.WEIGHTING),
        "critical_window": copy.deepcopy(contract.CRITICAL_WINDOW),
        "fit": copy.deepcopy(contract.FIT),
        "candidate_selection": {
            "rule": contract.CANDIDATE_SELECTION_RULE,
            "module_path": contract.CANDIDATE_MODULE_PATH.as_posix(),
            "candidate_id": "DEFERRED_UNTIL_CASE_BALANCED_FIT",
            "candidate_tree_sha256": "DEFERRED_UNTIL_CASE_BALANCED_FIT",
        },
        "development_cases": [
            contract.canonical_development_case(case_id)
            for case_id in contract.DEVELOPMENT_CASE_IDS
        ],
        "q2_historical_design_freeze": copy.deepcopy(
            contract.Q2_DESIGN_FREEZE_ARTIFACT
        ),
        "q2_unopened_paths": {
            name: path.as_posix() for name, path in contract.Q2_UNOPENED_PATHS.items()
        },
        "q3_design_freeze": gate["q3_design_freeze"],
        "q3_unopened_paths": {
            name: path.as_posix() for name, path in contract.Q3_UNOPENED_PATHS.items()
        },
        "q3_prerequisites": copy.deepcopy(contract.Q3_PREREQUISITES),
        "execution_lock_created": False,
        "pipeline_claim_created": False,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "raw_sensor_sample_count": 0,
        "teacher_query_count": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
    }


def _protocol_freeze_payload_checks(payload: Any) -> dict[str, bool]:
    source_gate = (
        payload.get("source_precondition_gate")
        if isinstance(payload, Mapping)
        else None
    )
    expected_source_gate = _prepublication_source_gate_snapshot()
    zero_fields = (
        "environment_reset_calls",
        "environment_step_calls",
        "raw_sensor_sample_count",
        "teacher_query_count",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
    )
    expected_candidate = {
        "rule": contract.CANDIDATE_SELECTION_RULE,
        "module_path": contract.CANDIDATE_MODULE_PATH.as_posix(),
        "candidate_id": "DEFERRED_UNTIL_CASE_BALANCED_FIT",
        "candidate_tree_sha256": "DEFERRED_UNTIL_CASE_BALANCED_FIT",
    }
    expected_payload = _protocol_freeze_payload_from_gate(expected_source_gate)
    checks = {
        "schema": isinstance(payload, Mapping)
        and set(payload) == _PROTOCOL_FREEZE_FIELDS,
        "canonical_payload": isinstance(payload, Mapping)
        and _strict_equal(dict(payload), expected_payload),
        "identity": isinstance(payload, Mapping)
        and payload.get("schema_version") == contract.SCHEMA_VERSION
        and payload.get("status") == contract.PROTOCOL_FREEZE_STATUS
        and payload.get("passed") is True
        and payload.get("protocol_id") == contract.PROTOCOL_ID
        and payload.get("pipeline_id") == contract.PIPELINE_ID
        and payload.get("revision") == contract.REVISION,
        "source_gate": isinstance(source_gate, Mapping)
        and source_gate == expected_source_gate
        and expected_source_gate.get("passed") is True,
        "authority": isinstance(payload, Mapping)
        and payload.get("authority") == contract.AUTHORITY,
        "source_closure": isinstance(payload, Mapping)
        and payload.get("source_records") == _source_records(),
        "locked_inputs": isinstance(payload, Mapping)
        and payload.get("locked_inputs") == _expected_locked_inputs(),
        "locked_records": all(_locked_record_checks().values()),
        "locked_semantics": all(_locked_semantics().values()),
        "stage_order": isinstance(payload, Mapping)
        and payload.get("stage_order") == list(contract.STAGE_IDS),
        "no_collection": isinstance(payload, Mapping)
        and payload.get("new_collection_stages") == []
        and payload.get("new_collection_count") == 0,
        "fit_contract": isinstance(payload, Mapping)
        and payload.get("fit") == contract.FIT
        and payload.get("weighting") == contract.WEIGHTING
        and payload.get("critical_window") == contract.CRITICAL_WINDOW
        and payload.get("corpus_counts") == contract.expected_corpus_counts(),
        "candidate_deferred": isinstance(payload, Mapping)
        and payload.get("candidate_selection") == expected_candidate,
        "development_cases": isinstance(payload, Mapping)
        and payload.get("development_cases")
        == [
            contract.canonical_development_case(case_id)
            for case_id in contract.DEVELOPMENT_CASE_IDS
        ],
        "q2_design": isinstance(payload, Mapping)
        and payload.get("q2_historical_design_freeze")
        == contract.Q2_DESIGN_FREEZE_ARTIFACT
        and payload.get("q2_unopened_paths")
        == {name: path.as_posix() for name, path in contract.Q2_UNOPENED_PATHS.items()}
        and all(_q2_design_semantics().values()),
        "q3_design": isinstance(payload, Mapping)
        and payload.get("q3_design_freeze") == contract.Q3_DESIGN_FREEZE_ARTIFACT
        and payload.get("q3_unopened_paths")
        == {name: path.as_posix() for name, path in contract.Q3_UNOPENED_PATHS.items()}
        and payload.get("q3_prerequisites") == contract.Q3_PREREQUISITES
        and all(_q3_design_semantics().values()),
        "unopened_authority": isinstance(payload, Mapping)
        and payload.get("execution_lock_created") is False
        and payload.get("pipeline_claim_created") is False,
        "zero_activity": isinstance(payload, Mapping)
        and all(payload.get(name) == 0 for name in zero_fields),
        "no_promotion": isinstance(payload, Mapping)
        and payload.get("runtime_promoted") is False
        and payload.get("checkpoint_zero_created") is False
        and payload.get("positive_morphology_enabled") is False,
    }
    return checks


def _validate_protocol_freeze_payload(payload: Any) -> None:
    _canonical_json_bytes(payload)
    checks = _protocol_freeze_payload_checks(payload)
    if not all(checks.values()):
        failed = [name for name, passed in checks.items() if not passed]
        raise V12R5ProtocolFreezeError(
            f"in-memory protocol freeze validation failed: {failed}"
        )


def build_design_audit_payload(
    protocol_payload: Mapping[str, Any],
    expected_protocol_record: Mapping[str, Any],
) -> dict[str, Any]:
    """Build the design-audit commit marker before either output is written."""

    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "PASS_H0_V12R5_CASE_BALANCED_DESIGN_AUDIT",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "protocol_freeze": dict(expected_protocol_record),
        "contract_self_check": contract.contract_self_check(),
        "source_precondition_gate": copy.deepcopy(
            protocol_payload["source_precondition_gate"]
        ),
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }


def _validate_design_audit_payload(
    audit: Any,
    *,
    protocol_payload: Mapping[str, Any],
    expected_protocol_record: Mapping[str, Any],
) -> None:
    _canonical_json_bytes(audit)
    expected = build_design_audit_payload(protocol_payload, expected_protocol_record)
    if (
        not isinstance(audit, Mapping)
        or set(audit) != _DESIGN_AUDIT_FIELDS
        or not _strict_equal(dict(audit), expected)
    ):
        raise V12R5ProtocolFreezeError("in-memory design audit validation failed")


def _write_json_exclusive(path: Path, payload: Mapping[str, Any]) -> None:
    """Durably publish canonical bytes without ever replacing the final path."""

    path.parent.mkdir(parents=True, exist_ok=True)
    if os.path.lexists(path):
        raise V12R5ProtocolFreezeError(f"refusing to clobber: {path}")
    encoded = _canonical_json_bytes(payload)
    descriptor, temporary_raw = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_raw)
    descriptor_open = True
    try:
        with os.fdopen(descriptor, "wb") as stream:
            descriptor_open = False
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.chmod(temporary, 0o644)
        except OSError:  # pragma: no cover - platform/filesystem dependent.
            pass
        os.link(temporary, path)
        _fsync_directory(path.parent)
    except FileExistsError as exc:
        raise V12R5ProtocolFreezeError(f"refusing to clobber: {path}") from exc
    finally:
        if descriptor_open:
            os.close(descriptor)
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def _record_schema_exact(record: Any, *, expected_path: PurePosixPath) -> bool:
    if not isinstance(record, Mapping) or set(record) != {
        "path",
        "sha256",
        "size_bytes",
    }:
        return False
    sha256 = record.get("sha256")
    sha_valid = (
        isinstance(sha256, str)
        and len(sha256) == 64
        and all(character in "0123456789abcdef" for character in sha256)
    )
    return (
        record.get("path") == expected_path.as_posix()
        and sha_valid
        and type(record.get("size_bytes")) is int
        and record["size_bytes"] > 0
    )


def _opened_qualification_paths() -> dict[str, list[str]]:
    return {
        "q2": [
            name
            for name, path in contract.Q2_UNOPENED_PATHS.items()
            if os.path.lexists(resolve_relative(path))
        ],
        "q3": [
            name
            for name, path in contract.Q3_UNOPENED_PATHS.items()
            if os.path.lexists(resolve_relative(path))
        ],
    }


def _qualification_snapshot() -> dict[str, Any]:
    opened = _opened_qualification_paths()
    q2_opened = list(opened["q2"])
    q3_opened = list(opened["q3"])
    return {
        "q2_paths_opened": q2_opened,
        "q3_paths_opened": q3_opened,
        "qualification_violation_detected": bool(q2_opened or q3_opened),
    }


def _qualification_snapshot_valid(payload: Mapping[str, Any]) -> bool:
    q2_opened = payload.get("q2_paths_opened")
    q3_opened = payload.get("q3_paths_opened")
    if (
        not isinstance(q2_opened, list)
        or not all(type(name) is str for name in q2_opened)
        or len(q2_opened) != len(set(q2_opened))
        or not set(q2_opened).issubset(contract.Q2_UNOPENED_PATHS)
        or not isinstance(q3_opened, list)
        or not all(type(name) is str for name in q3_opened)
        or len(q3_opened) != len(set(q3_opened))
        or not set(q3_opened).issubset(contract.Q3_UNOPENED_PATHS)
    ):
        return False
    canonical_q2 = [
        name for name in contract.Q2_UNOPENED_PATHS if name in set(q2_opened)
    ]
    canonical_q3 = [
        name for name in contract.Q3_UNOPENED_PATHS if name in set(q3_opened)
    ]
    return (
        q2_opened == canonical_q2
        and q3_opened == canonical_q3
        and payload.get("qualification_violation_detected")
        is bool(q2_opened or q3_opened)
    )


def _build_protocol_publication_failure(
    *,
    error: BaseException,
    expected_protocol_record: Mapping[str, Any],
    expected_audit_record: Mapping[str, Any],
) -> dict[str, Any]:
    qualification = _qualification_snapshot()
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "FAIL_H0_V12R5_PROTOCOL_PUBLICATION_TERMINAL",
        "passed": False,
        "terminal": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "phase": "PROTOCOL_FREEZE_THEN_DESIGN_AUDIT",
        "expected_protocol_freeze": dict(expected_protocol_record),
        "expected_design_audit": dict(expected_audit_record),
        "protocol_freeze_observation": _publication_observation(
            PROTOCOL_FREEZE_FILE, expected_record=expected_protocol_record
        ),
        "design_audit_observation": _publication_observation(
            DESIGN_AUDIT_FILE, expected_record=expected_audit_record
        ),
        "error": _error_record(error),
        **qualification,
        "new_collection_count": 0,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "qualification_executed": False,
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
        "next_stage": "STOP_TERMINAL",
    }


def verify_protocol_publication_failure() -> dict[str, Any]:
    """Verify the terminal, no-clobber receipt for a split publication."""

    failure = _load_mapping_file(PROTOCOL_PUBLICATION_FAILURE_FILE)
    recomputed_protocol_payload = _protocol_freeze_payload_from_gate(
        _prepublication_source_gate_snapshot()
    )
    _validate_protocol_freeze_payload(recomputed_protocol_payload)
    recomputed_protocol_record = _expected_artifact_record(
        contract.PROTOCOL_FREEZE_PATH, recomputed_protocol_payload
    )
    recomputed_audit_payload = build_design_audit_payload(
        recomputed_protocol_payload, recomputed_protocol_record
    )
    _validate_design_audit_payload(
        recomputed_audit_payload,
        protocol_payload=recomputed_protocol_payload,
        expected_protocol_record=recomputed_protocol_record,
    )
    recomputed_audit_record = _expected_artifact_record(
        contract.DESIGN_AUDIT_PATH, recomputed_audit_payload
    )
    protocol_observation = _publication_observation(
        PROTOCOL_FREEZE_FILE, expected_record=recomputed_protocol_record
    )
    audit_observation = _publication_observation(
        DESIGN_AUDIT_FILE, expected_record=recomputed_audit_record
    )
    error = failure.get("error")
    expected_error = {
        "type": error.get("type") if isinstance(error, Mapping) else None,
        "message": error.get("message") if isinstance(error, Mapping) else None,
    }
    q2_opened = failure.get("q2_paths_opened")
    q3_opened = failure.get("q3_paths_opened")
    qualification_violation = failure.get("qualification_violation_detected")
    expected_failure = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "FAIL_H0_V12R5_PROTOCOL_PUBLICATION_TERMINAL",
        "passed": False,
        "terminal": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "phase": "PROTOCOL_FREEZE_THEN_DESIGN_AUDIT",
        "expected_protocol_freeze": recomputed_protocol_record,
        "expected_design_audit": recomputed_audit_record,
        "protocol_freeze_observation": protocol_observation,
        "design_audit_observation": audit_observation,
        "error": expected_error,
        "q2_paths_opened": copy.deepcopy(q2_opened),
        "q3_paths_opened": copy.deepcopy(q3_opened),
        "qualification_violation_detected": qualification_violation,
        "new_collection_count": 0,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "qualification_executed": False,
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
        "next_stage": "STOP_TERMINAL",
    }
    allowed_states = {
        "ABSENT",
        "PARTIAL_OR_INVALID_REGULAR",
        "UNSAFE_UNREADABLE_REGULAR",
        "VALID_REGULAR",
    }
    expected_failure_record = _expected_artifact_record(
        contract.PROTOCOL_PUBLICATION_FAILURE_PATH, expected_failure
    )
    observed_failure_record = _raw_artifact_record(
        PROTOCOL_PUBLICATION_FAILURE_FILE,
        logical_path=contract.PROTOCOL_PUBLICATION_FAILURE_PATH.as_posix(),
    )
    checks = {
        "schema": set(failure) == _PROTOCOL_PUBLICATION_FAILURE_FIELDS,
        "canonical_payload": _strict_equal(failure, expected_failure),
        "failure_receipt_canonical": _strict_equal(
            observed_failure_record, expected_failure_record
        ),
        "expected_records": _record_schema_exact(
            failure.get("expected_protocol_freeze"),
            expected_path=contract.PROTOCOL_FREEZE_PATH,
        )
        and _record_schema_exact(
            failure.get("expected_design_audit"),
            expected_path=contract.DESIGN_AUDIT_PATH,
        )
        and _strict_equal(
            failure.get("expected_protocol_freeze"), recomputed_protocol_record
        )
        and _strict_equal(
            failure.get("expected_design_audit"), recomputed_audit_record
        ),
        "protocol_observation": _strict_equal(
            failure.get("protocol_freeze_observation"), protocol_observation
        )
        and isinstance(protocol_observation, Mapping)
        and protocol_observation.get("state") in allowed_states,
        "audit_observation": _strict_equal(
            failure.get("design_audit_observation"), audit_observation
        )
        and isinstance(audit_observation, Mapping)
        and audit_observation.get("state") in allowed_states,
        "error_exact": isinstance(error, Mapping)
        and set(error) == {"type", "message"}
        and type(error.get("type")) is str
        and bool(error["type"])
        and type(error.get("message")) is str
        and bool(error["message"]),
        "qualification_bound": _qualification_snapshot_valid(failure),
    }
    if not all(checks.values()):
        failed = [name for name, passed in checks.items() if not passed]
        raise V12R5ProtocolFreezeError(
            f"protocol publication failure receipt drifted: {failed}"
        )
    return failure


def _publish_protocol_publication_failure(
    *,
    error: BaseException,
    expected_protocol_record: Mapping[str, Any],
    expected_audit_record: Mapping[str, Any],
) -> None:
    failure = _build_protocol_publication_failure(
        error=error,
        expected_protocol_record=expected_protocol_record,
        expected_audit_record=expected_audit_record,
    )
    try:
        _write_json_exclusive(PROTOCOL_PUBLICATION_FAILURE_FILE, failure)
        verify_protocol_publication_failure()
    except BaseException as evidence_error:
        raise V12R5ProtocolFreezeError(
            "protocol publication failed and terminal evidence could not be verified"
        ) from evidence_error


def verify_protocol_freeze() -> dict[str, Any]:
    """Verify both transaction members; the design audit is the commit marker."""

    if os.path.lexists(PROTOCOL_PUBLICATION_FAILURE_FILE):
        raise V12R5ProtocolFreezeError(
            "protocol publication failure receipt is terminal and dominant"
        )
    payload = _load_mapping_file(PROTOCOL_FREEZE_FILE)
    _validate_protocol_freeze_payload(payload)
    expected_protocol_record = _expected_artifact_record(
        contract.PROTOCOL_FREEZE_PATH, payload
    )
    protocol_observation = _publication_observation(
        PROTOCOL_FREEZE_FILE, expected_record=expected_protocol_record
    )
    if protocol_observation.get("state") != "VALID_REGULAR":
        raise V12R5ProtocolFreezeError("protocol freeze is not canonical")
    audit = _load_mapping_file(DESIGN_AUDIT_FILE)
    _validate_design_audit_payload(
        audit,
        protocol_payload=payload,
        expected_protocol_record=expected_protocol_record,
    )
    expected_audit_record = _expected_artifact_record(contract.DESIGN_AUDIT_PATH, audit)
    audit_observation = _publication_observation(
        DESIGN_AUDIT_FILE, expected_record=expected_audit_record
    )
    if audit_observation.get("state") != "VALID_REGULAR":
        raise V12R5ProtocolFreezeError("design audit commit marker is not canonical")
    return payload


def publish_protocol_freeze() -> Path:
    """Publish freeze then commit marker, or terminally bind the visible prefix."""

    if not contract.AUTHORITY["protocol_freeze_publication_authorized"]:
        raise V12R5ProtocolFreezeError("V12R5 protocol publication is unauthorized")
    if any(
        os.path.lexists(path)
        for path in (
            PROTOCOL_FREEZE_FILE,
            DESIGN_AUDIT_FILE,
            PROTOCOL_PUBLICATION_FAILURE_FILE,
        )
    ):
        raise V12R5ProtocolFreezeError(
            "protocol freeze/design audit/publication failure exists"
        )

    payload = build_protocol_freeze_payload()
    _validate_protocol_freeze_payload(payload)
    expected_protocol_record = _expected_artifact_record(
        contract.PROTOCOL_FREEZE_PATH, payload
    )
    audit = build_design_audit_payload(payload, expected_protocol_record)
    _validate_design_audit_payload(
        audit,
        protocol_payload=payload,
        expected_protocol_record=expected_protocol_record,
    )
    expected_audit_record = _expected_artifact_record(contract.DESIGN_AUDIT_PATH, audit)

    try:
        _write_json_exclusive(PROTOCOL_FREEZE_FILE, payload)
        if (
            _publication_observation(
                PROTOCOL_FREEZE_FILE, expected_record=expected_protocol_record
            ).get("state")
            != "VALID_REGULAR"
        ):
            raise V12R5ProtocolFreezeError(
                "protocol freeze post-publication record mismatch"
            )
        _write_json_exclusive(DESIGN_AUDIT_FILE, audit)
        if (
            _publication_observation(
                DESIGN_AUDIT_FILE, expected_record=expected_audit_record
            ).get("state")
            != "VALID_REGULAR"
        ):
            raise V12R5ProtocolFreezeError(
                "design audit post-publication record mismatch"
            )
        verify_protocol_freeze()
    except BaseException as error:
        _publish_protocol_publication_failure(
            error=error,
            expected_protocol_record=expected_protocol_record,
            expected_audit_record=expected_audit_record,
        )
        raise
    return PROTOCOL_FREEZE_FILE


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--check", action="store_true", help="audit without writes")
    group.add_argument("--publish", action="store_true", help="publish no-clobber")
    group.add_argument(
        "--verify", action="store_true", help="verify a published freeze"
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.publish:
        path = publish_protocol_freeze()
        print(path.relative_to(REPO_ROOT).as_posix())
        return 0
    payload = verify_protocol_freeze() if args.verify else source_precondition_gate()
    print(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False))
    return 0 if payload.get("passed") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "V12R5ProtocolFreezeError",
    "artifact_record",
    "build_design_audit_payload",
    "build_protocol_freeze_payload",
    "main",
    "publish_protocol_freeze",
    "resolve_relative",
    "source_precondition_gate",
    "tree_record",
    "verify_protocol_freeze",
    "verify_protocol_publication_failure",
]
