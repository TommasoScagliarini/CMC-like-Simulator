"""Run or verify the sole V12R2 in-memory P0 numerical design audit.

``--prepare`` first verifies the immutable V12R2 protocol freeze, binds all
seven execution sources, then calls the frozen fitter exactly once.  The fit is
in memory: no candidate checkpoint, run root, lock, or pipeline artifact may
be created.  If and only if the enriched payload passes the pure contract gate,
the canonical receipt is claimed with ``O_EXCL`` and written once.

``--verify`` is strictly read-only.  It never calls the fitter and requires the
receipt's source/freeze bindings and canonical JSON bytes to remain exact.
"""

from __future__ import annotations

import argparse
import copy
import importlib
import os
import platform
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
VALIDATION_ROOT = REPO_ROOT / "validation"
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
BASELINE_ROOT = TRAJECTORY_ROOT / "baseline_MLP"
LOCAL_VALIDATION_ROOT = BASELINE_ROOT / "validation"
R2_VALIDATION_ROOT = Path(__file__).resolve().parent
for _root in (
    REPO_ROOT,
    VALIDATION_ROOT,
    TRAJECTORY_ROOT,
    BASELINE_ROOT,
    LOCAL_VALIDATION_ROOT,
    R2_VALIDATION_ROOT,
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_forensic_rollout as forensic  # noqa: E402
import freeze_h0_primary_split_v12r2_autonomy_recovery as protocol_freezer  # noqa: E402
import h0_primary_split_v12r2_autonomy_recovery_contract as contract  # noqa: E402
import h0_primary_split_v12r2_recovery_weighted_fitter as fitter  # noqa: E402


class V12R2DesignAuditError(RuntimeError):
    """Raised when the single audit cannot be published or verified exactly."""


def resolve_relative(path: str | os.PathLike[str] | PurePosixPath) -> Path:
    raw = path.as_posix() if isinstance(path, PurePosixPath) else os.fspath(path)
    pure = PurePosixPath(raw)
    if (
        not raw
        or pure.is_absolute()
        or ".." in pure.parts
        or "\\" in raw
        or pure.as_posix() != raw
    ):
        raise V12R2DesignAuditError(f"non-canonical repository path: {raw!r}")
    return REPO_ROOT.joinpath(*pure.parts)


RECEIPT_PATH = resolve_relative(contract.DESIGN_AUDIT_RECEIPT_PATH)
PROTOCOL_FREEZE_PATH = resolve_relative(contract.PROTOCOL_FREEZE_PATH)
EXECUTION_LOCK_PATH = resolve_relative(contract.EXECUTION_LOCK_PATH)
RUN_ROOT = resolve_relative(contract.RUN_ROOT)
PIPELINE_CLAIM_PATH = resolve_relative(contract.PIPELINE_CLAIM_PATH)
PIPELINE_LEDGER_PATH = resolve_relative(contract.PIPELINE_LEDGER_PATH)


def _record(path: str | os.PathLike[str] | PurePosixPath) -> dict[str, Any]:
    return forensic.artifact_record(resolve_relative(path), artifact_root=REPO_ROOT)


def _execution_source_records() -> dict[str, dict[str, Any]]:
    expected_names = tuple(contract.FUTURE_EXECUTION_SOURCES_REQUIRED)
    paths = contract.FUTURE_EXECUTION_SOURCE_RELATIVE_PATHS
    if (
        len(expected_names) != 7
        or tuple(paths) != expected_names
        or set(paths) != set(expected_names)
    ):
        raise V12R2DesignAuditError("execution source declaration drifted")
    try:
        return {name: _record(paths[name]) for name in expected_names}
    except Exception as exc:
        raise V12R2DesignAuditError(
            "all seven execution sources must exist before the audit"
        ) from exc


def _v12r1_lineage_records() -> dict[str, dict[str, Any]]:
    records = {
        name: _record(artifact["path"])
        for name, artifact in contract.V12R1_LINEAGE_ARTIFACTS.items()
    }
    if records != contract.V12R1_LINEAGE_ARTIFACTS:
        raise V12R2DesignAuditError("terminal V12R1 lineage drifted")
    return records


def _runtime_record() -> dict[str, str]:
    versions: dict[str, str] = {
        "system": platform.system(),
        "machine": platform.machine(),
        "python": platform.python_version(),
    }
    for name in ("numpy", "scipy", "torch"):
        module = importlib.import_module(name)
        value = getattr(module, "__version__", None)
        if not isinstance(value, str) or not value:
            raise V12R2DesignAuditError(f"runtime version unavailable: {name}")
        versions[name] = value
    return versions


def _forbidden_execution_occupancy() -> dict[str, bool]:
    return {
        "execution_lock_absent": not os.path.lexists(EXECUTION_LOCK_PATH),
        "run_root_absent": not os.path.lexists(RUN_ROOT),
        "pipeline_claim_absent": not os.path.lexists(PIPELINE_CLAIM_PATH),
        "pipeline_ledger_absent": not os.path.lexists(PIPELINE_LEDGER_PATH),
    }


def _assert_prepare_occupancy() -> dict[str, bool]:
    occupancy = {
        "design_audit_receipt_absent": not os.path.lexists(RECEIPT_PATH),
        **_forbidden_execution_occupancy(),
    }
    failed = [name for name, value in occupancy.items() if value is not True]
    if failed:
        raise V12R2DesignAuditError(
            f"design audit no-clobber/execution occupancy failed: {failed}"
        )
    return occupancy


def _verify_protocol_freeze() -> dict[str, Any]:
    try:
        payload = protocol_freezer.verify_protocol_freeze()
    except Exception as exc:
        raise V12R2DesignAuditError(
            "canonical V12R2 protocol freeze is missing or drifted"
        ) from exc
    if (
        payload.get("passed") is not True
        or payload.get("schema_version") != contract.SCHEMA_VERSION
        or payload.get("protocol_id") != contract.PROTOCOL_ID
        or payload.get("execution_lock") is not None
        or payload.get("actor_fit_executions") != 0
        or payload.get("environment_reset_calls") != 0
        or payload.get("environment_step_calls") != 0
    ):
        raise V12R2DesignAuditError("protocol freeze is not design-only PASS")
    return dict(payload)


def _verify_protocol_freeze_snapshot() -> dict[str, Any]:
    """Recompute frozen bytes after later authorized occupancy has opened."""

    expected = protocol_freezer._assemble_protocol_freeze(
        {
            "protocol_freeze_unoccupied": True,
            "execution_lock_absent": True,
            "design_audit_absent": True,
            "run_root_absent": True,
        }
    )
    try:
        observed = forensic.strict_json_load(PROTOCOL_FREEZE_PATH)
    except Exception as exc:
        raise V12R2DesignAuditError(
            "canonical V12R2 protocol freeze is unreadable"
        ) from exc
    if (
        observed != expected
        or PROTOCOL_FREEZE_PATH.read_bytes() != forensic.canonical_json_bytes(expected)
    ):
        raise V12R2DesignAuditError(
            "V12R2 protocol freeze/current design sources drifted"
        )
    return dict(expected)


def build_design_audit() -> dict[str, Any]:
    """Execute exactly one in-memory fit and return its gated receipt payload."""

    occupancy_before = _assert_prepare_occupancy()
    _verify_protocol_freeze()
    protocol_record_before = _record(contract.PROTOCOL_FREEZE_PATH)
    v12_record_before = _record(contract.V12_PROTOCOL_FREEZE_PATH)
    expected_v12 = {
        "path": contract.V12_PROTOCOL_FREEZE_PATH.as_posix(),
        "sha256": contract.V12_PROTOCOL_FREEZE_SHA256,
        "size_bytes": contract.V12_PROTOCOL_FREEZE_SIZE_BYTES,
    }
    if v12_record_before != expected_v12:
        raise V12R2DesignAuditError("immutable V12 parent freeze drifted")
    sources_before = _execution_source_records()
    fit_contract = contract.fit_contract_self_check()
    if fit_contract.get("passed") is not True:
        raise V12R2DesignAuditError("fit contract self-check is not PASS")
    r1_lineage = _v12r1_lineage_records()
    runtime_before = _runtime_record()

    # This is the only authorized fit call in this module.  The returned audit
    # must itself attest exactly one fit/update and zero persisted checkpoints.
    raw = fitter.run_design_audit_in_memory()
    if not isinstance(raw, Mapping):
        raise V12R2DesignAuditError("in-memory fitter returned a non-object")

    occupancy_after = _forbidden_execution_occupancy()
    sources_after = _execution_source_records()
    runtime_after = _runtime_record()
    if occupancy_after != {name: True for name in occupancy_after}:
        raise V12R2DesignAuditError("in-memory audit opened execution scope")
    if os.path.lexists(RECEIPT_PATH):
        raise V12R2DesignAuditError("audit fitter unexpectedly created the receipt")
    if sources_after != sources_before:
        raise V12R2DesignAuditError("execution source changed during audit")
    if runtime_after != runtime_before:
        raise V12R2DesignAuditError("runtime identity changed during audit")
    if _record(contract.PROTOCOL_FREEZE_PATH) != protocol_record_before:
        raise V12R2DesignAuditError("V12R2 protocol freeze changed during audit")
    if _record(contract.V12_PROTOCOL_FREEZE_PATH) != v12_record_before:
        raise V12R2DesignAuditError("V12 protocol freeze changed during audit")

    payload = copy.deepcopy(dict(raw))
    payload.update(
        {
            "schema_version": contract.SCHEMA_VERSION,
            "status": contract.DESIGN_AUDIT_PASS_STATUS,
            "passed": True,
            "protocol_id": contract.PROTOCOL_ID,
            "contract_id": contract.FIT_CONTRACT_ID,
            "design_audit_id": contract.DESIGN_AUDIT_ID,
            "protocol_freeze": protocol_record_before,
            "v12_protocol_freeze": expected_v12,
            "execution_sources": sources_before,
            "fit_contract_self_check": fit_contract,
            "v12r1_terminal_failure_lineage": r1_lineage,
            "probe_replay_schema": copy.deepcopy(contract.PROBE_REPLAY_SCHEMA),
            "p0_reproduction_tolerance": copy.deepcopy(
                contract.P0_REPRODUCTION_TOLERANCE
            ),
            "runtime": runtime_before,
            "preflight_occupancy": occupancy_before,
            "postfit_execution_occupancy": occupancy_after,
            "artifacts_written": [contract.DESIGN_AUDIT_RECEIPT_PATH.as_posix()],
        }
    )
    gate = contract.design_audit_gate(payload)
    if gate.get("passed") is not True:
        failed = [
            name for name, value in gate.get("checks", {}).items() if value is not True
        ]
        raise V12R2DesignAuditError(f"design audit contract gate failed: {failed}")
    payload["contract_gate"] = copy.deepcopy(gate)
    if contract.design_audit_gate(payload) != gate:
        raise V12R2DesignAuditError("design audit gate is not payload-stable")
    return payload


def _symlink_ancestors(path: Path) -> list[Path]:
    result: list[Path] = []
    for ancestor in (path.parent, *path.parent.parents):
        try:
            if stat.S_ISLNK(os.lstat(ancestor).st_mode):
                result.append(ancestor)
        except FileNotFoundError:
            continue
    return result


def _write_json_exclusive(path: Path, payload: Mapping[str, Any]) -> None:
    destination = Path(os.path.abspath(os.fspath(path)))
    if destination != RECEIPT_PATH:
        raise V12R2DesignAuditError(f"non-canonical audit receipt: {destination}")
    if _symlink_ancestors(destination):
        raise V12R2DesignAuditError("audit receipt has a symlink ancestor")
    try:
        parent_status = os.lstat(destination.parent)
    except FileNotFoundError as exc:
        raise V12R2DesignAuditError("audit receipt parent is missing") from exc
    if not stat.S_ISDIR(parent_status.st_mode):
        raise V12R2DesignAuditError("audit receipt parent is not a directory")
    if os.path.lexists(destination):
        raise V12R2DesignAuditError("audit receipt exists/no-clobber")

    encoded = forensic.canonical_json_bytes(payload)
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_BINARY", 0)
    try:
        descriptor = os.open(destination, flags, 0o644)
    except FileExistsError as exc:
        raise V12R2DesignAuditError("audit receipt exists/no-clobber") from exc
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
    except Exception:
        # Retain the exclusive partial claim: reuse/retry is fail-closed.
        raise
    try:
        directory_descriptor = os.open(destination.parent, os.O_RDONLY)
    except OSError:
        directory_descriptor = None
    if directory_descriptor is not None:
        try:
            os.fsync(directory_descriptor)
        except OSError:
            pass
        finally:
            os.close(directory_descriptor)


def prepare_design_audit() -> dict[str, Any]:
    """Run the sole fit and publish only the canonical receipt, no-clobber."""

    payload = build_design_audit()
    _write_json_exclusive(RECEIPT_PATH, payload)
    return verify_design_audit()


def verify_design_audit() -> dict[str, Any]:
    """Read-only exact verification; never execute the fitter."""

    _verify_protocol_freeze_snapshot()
    try:
        status = os.lstat(RECEIPT_PATH)
    except FileNotFoundError as exc:
        raise V12R2DesignAuditError(
            "canonical design audit receipt is missing"
        ) from exc
    if not stat.S_ISREG(status.st_mode) or _symlink_ancestors(RECEIPT_PATH):
        raise V12R2DesignAuditError("design audit receipt is not a safe regular file")
    try:
        payload = forensic.strict_json_load(RECEIPT_PATH)
    except Exception as exc:
        raise V12R2DesignAuditError("design audit receipt is not strict JSON") from exc
    if not isinstance(payload, Mapping):
        raise V12R2DesignAuditError("design audit receipt is not an object")
    observed = dict(payload)
    if RECEIPT_PATH.read_bytes() != forensic.canonical_json_bytes(observed):
        raise V12R2DesignAuditError("design audit receipt bytes are not canonical")

    gate = contract.design_audit_gate(observed)
    if gate.get("passed") is not True or observed.get("contract_gate") != gate:
        raise V12R2DesignAuditError("design audit contract gate drifted")
    current_bindings = {
        "protocol_freeze": _record(contract.PROTOCOL_FREEZE_PATH),
        "v12_protocol_freeze": _record(contract.V12_PROTOCOL_FREEZE_PATH),
        "execution_sources": _execution_source_records(),
        "fit_contract_self_check": contract.fit_contract_self_check(),
        "v12r1_terminal_failure_lineage": _v12r1_lineage_records(),
        "probe_replay_schema": contract.PROBE_REPLAY_SCHEMA,
        "runtime": _runtime_record(),
        "artifacts_written": [contract.DESIGN_AUDIT_RECEIPT_PATH.as_posix()],
    }
    drifted = [
        name for name, value in current_bindings.items() if observed.get(name) != value
    ]
    if drifted:
        raise V12R2DesignAuditError(f"design audit current bindings drifted: {drifted}")
    return observed


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--prepare", action="store_true")
    action.add_argument("--verify", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    payload = prepare_design_audit() if args.prepare else verify_design_audit()
    print(f"{payload['status']}: {contract.DESIGN_AUDIT_RECEIPT_PATH.as_posix()}")
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
