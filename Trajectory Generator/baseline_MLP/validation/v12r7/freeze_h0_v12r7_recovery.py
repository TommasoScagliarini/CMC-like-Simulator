"""Freeze and verify the V12R7 recovery protocol and one-shot lock.

Importing this module is read-only.  ``prepare()`` is the sole publication
surface and must be called explicitly after source review.  The protocol binds
the complete inherited V12R6 runtime closure plus the R7 contract, probe,
fitter, freezer and runner.  The execution lock additionally attests the exact
terminal R6 inputs and the current OpenSim/plugin runtime.
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
from typing import Any, Callable, Mapping, Sequence


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
LOCAL_VALIDATION = REVISION_ROOT.parent
for _root in (
    REPO_ROOT,
    REPO_ROOT / "validation",
    REPO_ROOT / "Trajectory Generator" / "baseline_MLP",
    LOCAL_VALIDATION,
    LOCAL_VALIDATION / "v12r3",
    LOCAL_VALIDATION / "v12r6",
    REVISION_ROOT,
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import freeze_h0_v12r6_functional_composite as r6_freezer  # noqa: E402
import h0_forensic_rollout as forensic  # noqa: E402
import h0_v12r7_recovery_contract as contract  # noqa: E402
import run_h0_primary_split_v12r3_autonomy_recovery as runtime_source  # noqa: E402


class V12R7FreezeError(RuntimeError):
    """Raised when protocol or lock publication cannot remain exact."""


LOCAL_PRODUCTION_SOURCES = (
    "Trajectory Generator/baseline_MLP/validation/v12r7/__init__.py",
    "Trajectory Generator/baseline_MLP/validation/v12r7/h0_v12r7_recovery_contract.py",
    "Trajectory Generator/baseline_MLP/validation/v12r7/h0_v12r7_recovery_probe.py",
    "Trajectory Generator/baseline_MLP/validation/v12r7/h0_v12r7_recovery_fitter.py",
    "Trajectory Generator/baseline_MLP/validation/v12r7/freeze_h0_v12r7_recovery.py",
    "Trajectory Generator/baseline_MLP/validation/v12r7/run_h0_v12r7_recovery.py",
)
ADDITIONAL_TRANSITIVE_SOURCES = (
    "Trajectory Generator/baseline_MLP/validation/v12r3/"
    "h0_primary_split_v12r3_pure_probe_observer_labeler.py",
    "Trajectory Generator/baseline_MLP/validation/v12r3/"
    "h0_primary_split_v12r3_recovery_weighted_fitter.py",
    "validation/h0_primary_split_v10_coherent_teacher.py",
    "validation/h0_primary_split_v10s_fit.py",
    "validation/h0_primary_split_v11_weighted_fit.py",
    "Trajectory Generator/baseline_MLP/warm_start.py",
)
EXPECTED_R6_CANDIDATE_FILES = {
    "actor_feature_manifest.json",
    "class_and_ctor_args.pkl",
    "composite_build_manifest.json",
    "metadata.json",
    "module_state.pkl",
}

RuntimeAttestor = Callable[[], Mapping[str, Any]]


def resolve_relative(path: str | os.PathLike[str] | PurePosixPath) -> Path:
    """Resolve one canonical repository-relative POSIX path without following it."""

    raw = path.as_posix() if isinstance(path, PurePosixPath) else os.fspath(path)
    pure = PurePosixPath(raw)
    if (
        not raw
        or "\\" in raw
        or pure.is_absolute()
        or ".." in pure.parts
        or any(":" in part for part in pure.parts)
        or pure.as_posix() != raw
    ):
        raise V12R7FreezeError(f"non-canonical repository path: {raw!r}")
    return REPO_ROOT.joinpath(*pure.parts)


def _is_link_or_reparse(path: Path) -> bool:
    try:
        status = os.lstat(path)
    except OSError:
        return False
    attributes = int(getattr(status, "st_file_attributes", 0) or 0)
    reparse = int(getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400))
    return stat.S_ISLNK(status.st_mode) or bool(attributes & reparse)


def _reject_link_ancestors(path: Path, *, include_leaf: bool) -> None:
    try:
        parts = path.relative_to(REPO_ROOT).parts
    except ValueError as exc:
        raise V12R7FreezeError(f"path escaped repository: {path}") from exc
    limit = len(parts) if include_leaf else max(0, len(parts) - 1)
    current = REPO_ROOT
    for part in parts[:limit]:
        current /= part
        if os.path.lexists(current) and _is_link_or_reparse(current):
            raise V12R7FreezeError(f"unsafe symlink/junction component: {current}")


def _regular_file(path: Path) -> bool:
    try:
        status = os.lstat(path)
    except OSError:
        return False
    attributes = int(getattr(status, "st_file_attributes", 0) or 0)
    reparse = int(getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400))
    return stat.S_ISREG(status.st_mode) and not bool(attributes & reparse)


def safe_repository_path(
    path: str | os.PathLike[str] | PurePosixPath,
    *,
    include_leaf: bool = True,
) -> Path:
    """Return a canonical path after rejecting every existing link/reparse part."""

    if type(include_leaf) is not bool:
        raise V12R7FreezeError("include_leaf must be strict bool")
    target = resolve_relative(path)
    _reject_link_ancestors(target, include_leaf=include_leaf)
    return target


def artifact_record(path: str | os.PathLike[str] | PurePosixPath) -> dict[str, Any]:
    target = safe_repository_path(path)
    if not _regular_file(target) or _is_link_or_reparse(target):
        raise V12R7FreezeError(f"unsafe or missing artifact: {target}")
    data = target.read_bytes()
    return {
        "path": target.relative_to(REPO_ROOT).as_posix(),
        "sha256": hashlib.sha256(data).hexdigest(),
        "size_bytes": len(data),
    }


def tree_record(path: str | os.PathLike[str] | PurePosixPath) -> dict[str, Any]:
    root = safe_repository_path(path)
    try:
        root_status = os.lstat(root)
    except OSError as exc:
        raise V12R7FreezeError(f"unsafe or missing tree: {root}") from exc
    if not stat.S_ISDIR(root_status.st_mode) or _is_link_or_reparse(root):
        raise V12R7FreezeError(f"unsafe or missing tree: {root}")
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
                raise V12R7FreezeError(f"unsafe tree entry: {child}") from exc
            if not stat.S_ISDIR(child_status.st_mode) or _is_link_or_reparse(child):
                raise V12R7FreezeError(f"unsafe tree directory: {child}")
        for name in file_names:
            child = current / name
            if not _regular_file(child) or _is_link_or_reparse(child):
                raise V12R7FreezeError(f"unsafe tree file: {child}")
            files.append(child)
    if not files:
        raise V12R7FreezeError(f"empty artifact tree: {root}")
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


def _mapping(path: str | os.PathLike[str] | PurePosixPath) -> dict[str, Any]:
    record = artifact_record(path)
    target = resolve_relative(record["path"])
    try:
        value = forensic.strict_json_load(target)
    except Exception as exc:
        raise V12R7FreezeError(f"invalid strict JSON mapping: {target}") from exc
    if not isinstance(value, dict):
        raise V12R7FreezeError(f"expected JSON mapping: {target}")
    return value


def _strict_equal(left: Any, right: Any) -> bool:
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


def production_source_paths() -> tuple[str, ...]:
    inherited = tuple(r6_freezer.production_source_paths())
    return tuple(
        dict.fromkeys(
            (*inherited, *ADDITIONAL_TRANSITIVE_SOURCES, *LOCAL_PRODUCTION_SOURCES)
        )
    )


def production_source_closure() -> dict[str, dict[str, Any]]:
    return {path: artifact_record(path) for path in production_source_paths()}


def _locked_artifact(name: str) -> dict[str, Any]:
    expected = contract.LOCKED_INPUTS[name]
    observed = artifact_record(str(expected["path"]))
    exact = {
        "path": expected["path"],
        "sha256": expected["sha256"],
        "size_bytes": expected["size_bytes"],
    }
    if observed != exact:
        raise V12R7FreezeError(f"locked artifact drifted: {name}")
    return observed


def attest_locked_inputs() -> dict[str, Any]:
    """Recompute exact R6 terminal lineage and all immutable fit inputs."""

    records = {
        name: _locked_artifact(name)
        for name in (
            "r6_terminal_ledger",
            "r6_candidate_freeze",
            "base_corpus",
            "r4_failed_plus_labels",
        )
    }
    module = tree_record(contract.LOCKED_INPUTS["r6_candidate"]["path"])
    expected_module = contract.LOCKED_INPUTS["r6_candidate"]
    if (
        module["tree_sha256"] != expected_module["tree_sha256"]
        or module["file_count"] != expected_module["file_count"]
        or {row["path"] for row in module["files"]} != EXPECTED_R6_CANDIDATE_FILES
    ):
        raise V12R7FreezeError("locked R6 candidate tree drifted")
    ledger = _mapping(contract.R6_TERMINAL_LEDGER)
    candidate_freeze = _mapping(contract.R6_CANDIDATE_FREEZE)
    semantic_checks = {
        "r6_terminal_failure": ledger.get("passed") is False
        and ledger.get("terminal") is True
        and ledger.get("attempted_stage")
        == "development__deterministic_offset_plus_0p20"
        and ledger.get("candidate_module", {}).get("tree_sha256")
        == expected_module["tree_sha256"]
        and ledger.get("retry_authorized") is False,
        "r6_candidate_freeze_pass": candidate_freeze.get("passed") is True
        and candidate_freeze.get("candidate_module", {}).get("tree_sha256")
        == expected_module["tree_sha256"]
        and candidate_freeze.get("actor_updates") == 0
        and type(candidate_freeze.get("actor_updates")) is int,
        "row_counts": contract.LOCKED_INPUTS["base_corpus"]["rows"] == 9_232
        and contract.LOCKED_INPUTS["r4_failed_plus_labels"]["rows"] == 212,
    }
    if not all(semantic_checks.values()):
        failed = sorted(name for name, value in semantic_checks.items() if not value)
        raise V12R7FreezeError(f"locked input semantic audit failed: {failed}")
    return {
        "passed": True,
        "records": records,
        "r6_candidate": module,
        "semantic_checks": semantic_checks,
    }


def _runtime_attestation() -> dict[str, Any]:
    try:
        value = dict(runtime_source._runtime_record())
        forensic.canonical_json_bytes(value)
    except Exception as exc:
        raise V12R7FreezeError("runtime/plugin attestation failed") from exc
    readiness = value.get("platform_plugin_readiness")
    if (
        value.get("inference_stack_ready") is not True
        or not isinstance(readiness, Mapping)
        or readiness.get("passed") is not True
    ):
        raise V12R7FreezeError("runtime/plugin stack is not ready")
    return value


def _strict_runtime(attestor: RuntimeAttestor | None) -> dict[str, Any]:
    source = _runtime_attestation if attestor is None else attestor
    try:
        value = dict(source())
        forensic.canonical_json_bytes(value)
    except Exception as exc:
        raise V12R7FreezeError("runtime attestor returned malformed evidence") from exc
    readiness = value.get("platform_plugin_readiness")
    if (
        value.get("inference_stack_ready") is not True
        or not isinstance(readiness, Mapping)
        or readiness.get("passed") is not True
    ):
        raise V12R7FreezeError("runtime/plugin readiness is not PASS")
    return value


def expected_protocol_payload() -> dict[str, Any]:
    sources = production_source_closure()
    locked = attest_locked_inputs()
    self_check = contract.contract_self_check()
    if self_check.get("passed") is not True:
        raise V12R7FreezeError("R7 contract self-check failed")
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PROTOCOL_FREEZE_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "revision": contract.REVISION,
        "authority": copy.deepcopy(contract.AUTHORITY),
        "contract_self_check": self_check,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_module_path": contract.CANDIDATE_MODULE_PATH.as_posix(),
        "fit_contract": copy.deepcopy(contract.FIT),
        "offline_thresholds": copy.deepcopy(contract.OFFLINE_THRESHOLDS),
        "collection_cases": [
            contract.canonical_collection_case(case_id)
            for case_id in contract.COLLECTION_CASE_IDS
        ],
        "development_cases": [
            contract.canonical_development_case(case_id)
            for case_id in contract.DEVELOPMENT_CASE_IDS
        ],
        "stage_order": list(contract.STAGE_IDS),
        "run_root": contract.RUN_ROOT.as_posix(),
        "locked_inputs": locked,
        "production_source_count": len(sources),
        "production_source_closure": sources,
        "transitive_source_basis": "V12R6_RUNTIME_CLOSURE_PLUS_R7_DIRECT_IMPORTS",
        "one_shot": True,
        "single_collection_round": True,
        "single_actor_fit": True,
        "retry_authorized": False,
        "resume_authorized": False,
        "alpha_sweep_authorized": False,
        "qualification_execution_authorized": False,
        "positive_morphology_authorized": False,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }


def verify_protocol_freeze() -> dict[str, Any]:
    observed = _mapping(contract.PROTOCOL_FREEZE_PATH)
    expected = expected_protocol_payload()
    if not _strict_equal(observed, expected):
        raise V12R7FreezeError("R7 protocol freeze or source closure drifted")
    return observed


def publish_protocol_freeze() -> dict[str, Any]:
    destination = safe_repository_path(contract.PROTOCOL_FREEZE_PATH)
    if os.path.lexists(destination):
        return verify_protocol_freeze()
    if os.path.lexists(resolve_relative(contract.RUN_ROOT)):
        raise V12R7FreezeError("run root exists before protocol freeze")
    forensic.write_json_exclusive(destination, expected_protocol_payload())
    return verify_protocol_freeze()


def expected_execution_lock_payload(
    *, runtime_attestor: RuntimeAttestor | None = None
) -> dict[str, Any]:
    protocol = verify_protocol_freeze()
    runtime = _strict_runtime(runtime_attestor)
    occupancy = {
        "run_root_absent": not os.path.lexists(resolve_relative(contract.RUN_ROOT)),
        "pipeline_claim_absent": not os.path.lexists(
            resolve_relative(contract.CLAIM_PATH)
        ),
        "pipeline_ledger_absent": not os.path.lexists(
            resolve_relative(contract.LEDGER_PATH)
        ),
    }
    checks = {
        "protocol_exact": protocol == expected_protocol_payload(),
        "sources_exact": protocol.get("production_source_closure")
        == production_source_closure(),
        "locked_inputs_exact": attest_locked_inputs().get("passed") is True,
        "runtime_ready": runtime.get("inference_stack_ready") is True
        and runtime.get("platform_plugin_readiness", {}).get("passed") is True,
        "occupancy_pristine": all(value is True for value in occupancy.values()),
        "authority_exact": contract.AUTHORITY.get("actor_fit_execution_authorized")
        is True
        and contract.AUTHORITY.get("development_execution_authorized") is True
        and contract.AUTHORITY.get("qualification_execution_authorized") is False,
    }
    passed = all(value is True for value in checks.values())
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.EXECUTION_LOCK_PASS_STATUS
            if passed
            else "FAIL_H0_V12R7_RECOVERY_EXECUTION_LOCK"
        ),
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "revision": contract.REVISION,
        "authority": copy.deepcopy(contract.AUTHORITY),
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_module_path": contract.CANDIDATE_MODULE_PATH.as_posix(),
        "checks": checks,
        "occupancy": occupancy,
        "runtime": runtime,
        "protocol_freeze": artifact_record(contract.PROTOCOL_FREEZE_PATH),
        "production_source_closure": production_source_closure(),
        "locked_inputs": attest_locked_inputs(),
        "stage_order": list(contract.STAGE_IDS),
        "run_root": contract.RUN_ROOT.as_posix(),
        "one_shot": True,
        "single_collection_round": True,
        "single_actor_fit": True,
        "retry_authorized": False,
        "resume_authorized": False,
        "alpha_sweep_authorized": False,
        "qualification_execution_authorized": False,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }


def verify_execution_lock(
    *,
    require_pristine: bool = False,
    runtime_attestor: RuntimeAttestor | None = None,
) -> dict[str, Any]:
    if type(require_pristine) is not bool:
        raise V12R7FreezeError("require_pristine must be strict bool")
    verify_protocol_freeze()
    lock = _mapping(contract.EXECUTION_LOCK_PATH)
    current_runtime = _strict_runtime(runtime_attestor)
    stored_checks = lock.get("checks")
    stored_occupancy = lock.get("occupancy")
    identity_checks = {
        "status": lock.get("status") == contract.EXECUTION_LOCK_PASS_STATUS,
        "passed": lock.get("passed") is True,
        "identity": lock.get("protocol_id") == contract.PROTOCOL_ID
        and lock.get("pipeline_id") == contract.PIPELINE_ID,
        "protocol": lock.get("protocol_freeze")
        == artifact_record(contract.PROTOCOL_FREEZE_PATH),
        "sources": lock.get("production_source_closure") == production_source_closure(),
        "locked_inputs": lock.get("locked_inputs") == attest_locked_inputs(),
        "runtime": lock.get("runtime") == current_runtime,
        "stage_order": lock.get("stage_order") == list(contract.STAGE_IDS),
        "candidate_selection": lock.get("candidate_selection_rule")
        == contract.CANDIDATE_SELECTION_RULE
        and lock.get("candidate_module_path")
        == contract.CANDIDATE_MODULE_PATH.as_posix(),
        "stored_gate_pass": isinstance(stored_checks, Mapping)
        and bool(stored_checks)
        and all(value is True for value in stored_checks.values()),
        "stored_pristine_occupancy": isinstance(stored_occupancy, Mapping)
        and set(stored_occupancy)
        == {"run_root_absent", "pipeline_claim_absent", "pipeline_ledger_absent"}
        and all(value is True for value in stored_occupancy.values()),
        "authority": lock.get("authority") == contract.AUTHORITY,
        "one_shot": lock.get("one_shot") is True
        and lock.get("retry_authorized") is False
        and lock.get("resume_authorized") is False
        and lock.get("alpha_sweep_authorized") is False,
        "zero_preexecution": all(
            type(lock.get(name)) is int and lock[name] == 0
            for name in (
                "environment_reset_calls",
                "environment_step_calls",
                "actor_updates",
                "critic_updates",
                "ppo_updates",
            )
        ),
    }
    if not all(value is True for value in identity_checks.values()):
        failed = sorted(name for name, value in identity_checks.items() if not value)
        raise V12R7FreezeError(f"R7 execution lock drifted: {failed}")
    if require_pristine and os.path.lexists(resolve_relative(contract.RUN_ROOT)):
        raise V12R7FreezeError("R7 run root is no longer pristine")
    return lock


def publish_execution_lock(
    *, runtime_attestor: RuntimeAttestor | None = None
) -> dict[str, Any]:
    destination = safe_repository_path(contract.EXECUTION_LOCK_PATH)
    if os.path.lexists(destination):
        return verify_execution_lock(
            require_pristine=True, runtime_attestor=runtime_attestor
        )
    payload = expected_execution_lock_payload(runtime_attestor=runtime_attestor)
    if payload.get("passed") is not True:
        failed = sorted(
            name for name, value in payload.get("checks", {}).items() if not value
        )
        raise V12R7FreezeError(f"execution lock preconditions failed: {failed}")
    forensic.write_json_exclusive(destination, payload)
    return verify_execution_lock(
        require_pristine=True, runtime_attestor=runtime_attestor
    )


def prepare(*, runtime_attestor: RuntimeAttestor | None = None) -> dict[str, Any]:
    protocol = publish_protocol_freeze()
    lock = publish_execution_lock(runtime_attestor=runtime_attestor)
    return {
        "passed": True,
        "protocol_freeze": artifact_record(contract.PROTOCOL_FREEZE_PATH),
        "execution_lock": artifact_record(contract.EXECUTION_LOCK_PATH),
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
    result = (
        prepare()
        if args.prepare
        else {
            "protocol": verify_protocol_freeze(),
            "lock": verify_execution_lock(require_pristine=False),
        }
    )
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "V12R7FreezeError",
    "artifact_record",
    "attest_locked_inputs",
    "expected_execution_lock_payload",
    "expected_protocol_payload",
    "prepare",
    "production_source_closure",
    "production_source_paths",
    "publish_execution_lock",
    "publish_protocol_freeze",
    "resolve_relative",
    "safe_repository_path",
    "tree_record",
    "verify_execution_lock",
    "verify_protocol_freeze",
]
