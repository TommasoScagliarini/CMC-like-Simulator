"""Publish and verify the immutable V12R6 protocol and execution lock.

This module never builds a model and never opens an environment.  ``--prepare``
first freezes the complete source/input contract and then publishes a separate
one-shot execution lock only while the run and qualification namespaces are
still absent.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import stat
import sys
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
for _root in (
    REPO_ROOT,
    REPO_ROOT / "validation",
    REPO_ROOT / "Trajectory Generator" / "baseline_MLP",
    REVISION_ROOT.parent / "v12r5",
    REVISION_ROOT,
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_forensic_rollout as forensic  # noqa: E402
import h0_v12r6_functional_composite_contract as contract  # noqa: E402


class V12R6FreezeError(RuntimeError):
    """Raised when a freeze or lock invariant fails closed."""


def resolve_relative(path: str | os.PathLike[str] | PurePosixPath) -> Path:
    raw = path.as_posix() if isinstance(path, PurePosixPath) else os.fspath(path)
    pure = PurePosixPath(raw)
    if (
        not raw
        or "\\" in raw
        or pure.is_absolute()
        or ".." in pure.parts
        or pure.as_posix() != raw
    ):
        raise V12R6FreezeError(f"non-canonical repository path: {raw!r}")
    return REPO_ROOT.joinpath(*pure.parts)


def _is_link_or_reparse(path: Path) -> bool:
    try:
        status = os.lstat(path)
    except OSError:
        return False
    attributes = int(getattr(status, "st_file_attributes", 0) or 0)
    reparse = int(getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400))
    return stat.S_ISLNK(status.st_mode) or bool(attributes & reparse)


def _reject_link_or_reparse_ancestors(path: Path, *, include_leaf: bool) -> None:
    try:
        parts = path.relative_to(REPO_ROOT).parts
    except ValueError as exc:
        raise V12R6FreezeError(f"path escapes repository root: {path}") from exc
    limit = len(parts) if include_leaf else max(0, len(parts) - 1)
    current = REPO_ROOT
    for part in parts[:limit]:
        current /= part
        if os.path.lexists(current) and _is_link_or_reparse(current):
            raise V12R6FreezeError(f"unsafe symlink/junction path component: {current}")


def _is_regular_no_links(path: Path) -> bool:
    try:
        status = os.lstat(path)
    except OSError:
        return False
    attributes = int(getattr(status, "st_file_attributes", 0) or 0)
    reparse = int(getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400))
    return stat.S_ISREG(status.st_mode) and not bool(attributes & reparse)


def _artifact_record(path: str | os.PathLike[str] | PurePosixPath) -> dict[str, Any]:
    target = resolve_relative(path)
    _reject_link_or_reparse_ancestors(target, include_leaf=True)
    if not _is_regular_no_links(target):
        raise V12R6FreezeError(f"unsafe or missing artifact: {target}")
    data = target.read_bytes()
    return {
        "path": target.relative_to(REPO_ROOT).as_posix(),
        "sha256": hashlib.sha256(data).hexdigest(),
        "size_bytes": len(data),
    }


def tree_record(path: str | os.PathLike[str] | PurePosixPath) -> dict[str, Any]:
    root = resolve_relative(path)
    _reject_link_or_reparse_ancestors(root, include_leaf=True)
    try:
        root_status = os.lstat(root)
    except OSError as exc:
        raise V12R6FreezeError(f"unsafe or missing tree: {root}") from exc
    if not stat.S_ISDIR(root_status.st_mode) or _is_link_or_reparse(root):
        raise V12R6FreezeError(f"unsafe or missing tree: {root}")
    files: list[Path] = []
    for current_text, directory_names, file_names in os.walk(
        root, topdown=True, followlinks=False
    ):
        current = Path(current_text)
        directory_names.sort()
        file_names.sort()
        for name in directory_names:
            child = current / name
            try:
                child_status = os.lstat(child)
            except OSError as exc:
                raise V12R6FreezeError(f"unsafe tree entry: {child}") from exc
            if not stat.S_ISDIR(child_status.st_mode) or _is_link_or_reparse(child):
                raise V12R6FreezeError(
                    f"tree contains an unsafe directory entry: {child}"
                )
        for name in file_names:
            child = current / name
            if not _is_regular_no_links(child) or _is_link_or_reparse(child):
                raise V12R6FreezeError(f"tree contains an unsafe file: {child}")
            files.append(child)
    if not files:
        raise V12R6FreezeError(f"empty artifact tree: {root}")
    files.sort(key=lambda item: item.relative_to(root).as_posix())
    digest = hashlib.sha256()
    rows: list[dict[str, Any]] = []
    for item in files:
        relative = item.relative_to(root).as_posix()
        data = item.read_bytes()
        sha256 = hashlib.sha256(data).hexdigest()
        size_bytes = len(data)
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


LOCAL_SOURCE_NAMES = (
    "__init__.py",
    "build_h0_v12r6_composite_actor.py",
    "h0_v12r6_functional_composite_contract.py",
    "h0_v12r6_physical_development.py",
    "run_h0_v12r6_functional_composite.py",
    "freeze_h0_v12r6_functional_composite.py",
)
ADDITIONAL_SOURCE_PATHS = (
    "Trajectory Generator/baseline_MLP/validation/v12r5/"
    "h0_v12r5_case_balanced_contract.py",
    "Trajectory Generator/baseline_MLP/validation/v12r5/run_h0_v12r5_case_balanced.py",
)


def production_source_paths() -> tuple[str, ...]:
    inherited = tuple(contract.v12r5.FROZEN_EXTERNAL_RUNTIME_SOURCES)
    local = tuple(
        (contract.VALIDATION_ROOT / name).as_posix() for name in LOCAL_SOURCE_NAMES
    )
    return tuple(dict.fromkeys((*inherited, *ADDITIONAL_SOURCE_PATHS, *local)))


def production_source_closure() -> dict[str, dict[str, Any]]:
    return {path: _artifact_record(path) for path in production_source_paths()}


def _artifact_exact(expected: Mapping[str, Any]) -> bool:
    try:
        return _artifact_record(str(expected["path"])) == dict(expected)
    except (KeyError, TypeError, V12R6FreezeError):
        return False


def _tree_exact(expected: Mapping[str, Any]) -> bool:
    try:
        return tree_record(str(expected["path"])) == dict(expected)
    except (KeyError, TypeError, V12R6FreezeError):
        return False


def _mapping(path: Path) -> dict[str, Any]:
    try:
        value = forensic.strict_json_load(path)
    except Exception as exc:
        raise V12R6FreezeError(f"invalid strict JSON mapping: {path}") from exc
    if not isinstance(value, dict):
        raise V12R6FreezeError(f"expected JSON mapping: {path}")
    return value


def _closed_path_snapshot() -> dict[str, list[str]]:
    groups = {
        "historical_q2": contract.HISTORICAL_Q2_CLOSED_PATHS,
        "historical_q3": contract.HISTORICAL_Q3_CLOSED_PATHS,
        "future_v12r6q3": contract.Q3_CLOSED_PATHS,
    }
    return {
        name: [
            path.as_posix()
            for path in paths.values()
            if os.path.lexists(resolve_relative(path))
        ]
        for name, paths in groups.items()
    }


def _all_qualification_closed() -> bool:
    return all(not paths for paths in _closed_path_snapshot().values())


def expected_protocol_payload() -> dict[str, Any]:
    sources = production_source_closure()
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PROTOCOL_FREEZE_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "revision": contract.REVISION,
        "authority": copy.deepcopy(contract.AUTHORITY),
        "contract_self_check": contract.contract_self_check(),
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "synthesis": copy.deepcopy(contract.SYNTHESIS),
        "source_records": copy.deepcopy(contract.SOURCE_RECORDS),
        "development_cases": [
            contract.canonical_development_case(case_id)
            for case_id in contract.DEVELOPMENT_CASE_IDS
        ],
        "stage_order": list(contract.STAGE_IDS),
        "run_root": contract.RUN_ROOT.as_posix(),
        "candidate_module_path": contract.CANDIDATE_MODULE_PATH.as_posix(),
        "future_q3_closed_paths": {
            name: path.as_posix() for name, path in contract.Q3_CLOSED_PATHS.items()
        },
        "production_source_count": len(sources),
        "production_source_closure": sources,
        "one_shot": True,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "qualification_execution_authorized": False,
        "new_collection_count": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }


def verify_protocol_freeze() -> dict[str, Any]:
    path = resolve_relative(contract.PROTOCOL_FREEZE_PATH)
    observed = _mapping(path)
    expected = expected_protocol_payload()
    if observed != expected:
        raise V12R6FreezeError("V12R6 protocol freeze or source closure drifted")
    if not all(
        _artifact_exact(record)
        for record in contract.SOURCE_RECORDS.values()
        if "sha256" in record
    ):
        raise V12R6FreezeError("locked input artifact drifted")
    if not _tree_exact(contract.P2_MODULE_TREE) or not _tree_exact(
        contract.R5_MODULE_TREE
    ):
        raise V12R6FreezeError("locked source module tree drifted")
    return observed


def publish_protocol_freeze() -> dict[str, Any]:
    destination = resolve_relative(contract.PROTOCOL_FREEZE_PATH)
    if os.path.lexists(destination):
        return verify_protocol_freeze()
    if os.path.lexists(resolve_relative(contract.RUN_ROOT)):
        raise V12R6FreezeError("run root exists before protocol freeze")
    if not _all_qualification_closed():
        raise V12R6FreezeError("qualification output opened before protocol freeze")
    payload = expected_protocol_payload()
    forensic.write_json_exclusive(destination, payload)
    return verify_protocol_freeze()


def expected_execution_lock_payload() -> dict[str, Any]:
    protocol = verify_protocol_freeze()
    snapshot = _closed_path_snapshot()
    occupancy = {
        "run_root_absent": not os.path.lexists(resolve_relative(contract.RUN_ROOT)),
        "pipeline_claim_absent": not os.path.lexists(
            resolve_relative(contract.PIPELINE_CLAIM_PATH)
        ),
        "pipeline_ledger_absent": not os.path.lexists(
            resolve_relative(contract.PIPELINE_LEDGER_PATH)
        ),
        "qualification_unopened": all(not value for value in snapshot.values()),
    }
    checks = {
        "protocol": protocol == expected_protocol_payload(),
        "contract": contract.contract_self_check()["passed"] is True,
        "sources": protocol["production_source_closure"] == production_source_closure(),
        "locked_inputs": all(
            _artifact_exact(record)
            for record in contract.SOURCE_RECORDS.values()
            if "sha256" in record
        )
        and _tree_exact(contract.P2_MODULE_TREE)
        and _tree_exact(contract.R5_MODULE_TREE),
        "occupancy": all(occupancy.values()),
        "authority": contract.AUTHORITY["composite_synthesis_authorized"] is True
        and contract.AUTHORITY["development_execution_authorized"] is True
        and contract.AUTHORITY["qualification_execution_authorized"] is False,
    }
    passed = all(checks.values())
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.EXECUTION_LOCK_STATUS
        if passed
        else contract.TERMINAL_FAIL_STATUS,
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "revision": contract.REVISION,
        "authority": copy.deepcopy(contract.AUTHORITY),
        "checks": checks,
        "occupancy": occupancy,
        "closed_path_snapshot": snapshot,
        "protocol_freeze": _artifact_record(contract.PROTOCOL_FREEZE_PATH),
        "production_source_closure": production_source_closure(),
        "source_records": copy.deepcopy(contract.SOURCE_RECORDS),
        "stage_order": list(contract.STAGE_IDS),
        "run_root": contract.RUN_ROOT.as_posix(),
        "one_shot": True,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "qualification_execution_authorized": False,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }


def verify_execution_lock(
    *, require_pristine: bool = False, require_qualification_closed: bool = True
) -> dict[str, Any]:
    lock_path = resolve_relative(contract.EXECUTION_LOCK_PATH)
    lock = _mapping(lock_path)
    if (
        lock.get("status") != contract.EXECUTION_LOCK_STATUS
        or lock.get("passed") is not True
        or lock.get("protocol_id") != contract.PROTOCOL_ID
        or lock.get("pipeline_id") != contract.PIPELINE_ID
        or lock.get("protocol_freeze")
        != _artifact_record(contract.PROTOCOL_FREEZE_PATH)
        or lock.get("production_source_closure") != production_source_closure()
        or lock.get("source_records") != contract.SOURCE_RECORDS
        or lock.get("stage_order") != list(contract.STAGE_IDS)
    ):
        raise V12R6FreezeError("V12R6 execution lock or source closure drifted")
    if not all(
        _artifact_exact(record)
        for record in contract.SOURCE_RECORDS.values()
        if "sha256" in record
    ):
        raise V12R6FreezeError("locked input artifact drifted after execution lock")
    if not _tree_exact(contract.P2_MODULE_TREE) or not _tree_exact(
        contract.R5_MODULE_TREE
    ):
        raise V12R6FreezeError("source module tree drifted after execution lock")
    if require_qualification_closed and not _all_qualification_closed():
        raise V12R6FreezeError("qualification namespace opened during V12R6")
    if require_pristine and os.path.lexists(resolve_relative(contract.RUN_ROOT)):
        raise V12R6FreezeError("V12R6 run root is no longer pristine")
    return lock


def publish_execution_lock() -> dict[str, Any]:
    destination = resolve_relative(contract.EXECUTION_LOCK_PATH)
    if os.path.lexists(destination):
        return verify_execution_lock(require_pristine=True)
    payload = expected_execution_lock_payload()
    if payload["passed"] is not True:
        failed = [name for name, passed in payload["checks"].items() if not passed]
        raise V12R6FreezeError(f"execution lock preconditions failed: {failed}")
    forensic.write_json_exclusive(destination, payload)
    return verify_execution_lock(require_pristine=True)


def prepare() -> dict[str, Any]:
    protocol = publish_protocol_freeze()
    lock = publish_execution_lock()
    return {
        "passed": True,
        "protocol_freeze": _artifact_record(contract.PROTOCOL_FREEZE_PATH),
        "execution_lock": _artifact_record(contract.EXECUTION_LOCK_PATH),
        "protocol": protocol,
        "lock": lock,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--prepare", action="store_true")
    mode.add_argument("--verify", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.prepare:
        result = prepare()
    else:
        result = {
            "protocol": verify_protocol_freeze(),
            "lock": verify_execution_lock(require_pristine=False),
        }
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
