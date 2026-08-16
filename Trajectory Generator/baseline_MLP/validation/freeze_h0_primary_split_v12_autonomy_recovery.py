"""Build and freeze the non-executable V12 autonomy/recovery protocol.

This command performs read-only audits of the terminal V11 evidence and writes
one strict no-clobber protocol-freeze JSON.  It cannot fit an actor, load a
policy for inference, reset/step an environment, mint a pipeline claim, or
create the future execution lock.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import math
import os
import stat
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
from scipy.spatial import cKDTree


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
V12_VALIDATION_ROOT = Path(__file__).resolve().parent
for _root in (REPO_ROOT, VALIDATION_ROOT, V12_VALIDATION_ROOT):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_forensic_rollout as forensic  # noqa: E402
import h0_primary_split_v12_autonomy_recovery_contract as contract  # noqa: E402


class V12ProtocolFreezeError(RuntimeError):
    """Raised when the design-only V12 protocol cannot be frozen exactly."""


def _array_sha256(value: Any) -> str:
    array = np.ascontiguousarray(np.asarray(value))
    digest = hashlib.sha256()
    digest.update(str(array.dtype).encode("ascii"))
    digest.update(repr(tuple(int(item) for item in array.shape)).encode("ascii"))
    digest.update(array.tobytes(order="C"))
    return digest.hexdigest()


def resolve_relative(path: str | os.PathLike[str]) -> Path:
    value = Path(path)
    return (value if value.is_absolute() else REPO_ROOT / value).resolve()


def _lexical_absolute(path: str | os.PathLike[str]) -> Path:
    value = Path(path).expanduser()
    lexical = value if value.is_absolute() else REPO_ROOT / value
    return Path(os.path.abspath(os.fspath(lexical)))


def _mapping(path: str | os.PathLike[str]) -> dict[str, Any]:
    value = forensic.strict_json_load(resolve_relative(path))
    if not isinstance(value, Mapping):
        raise V12ProtocolFreezeError(f"expected JSON object: {path}")
    return dict(value)


def _sequence(path: str | os.PathLike[str]) -> list[Any]:
    value = forensic.strict_json_load(resolve_relative(path))
    if not isinstance(value, list):
        raise V12ProtocolFreezeError(f"expected JSON array: {path}")
    return value


def _record(path: str | os.PathLike[str]) -> dict[str, Any]:
    return forensic.artifact_record(resolve_relative(path), artifact_root=REPO_ROOT)


def _tree_record(path: str | os.PathLike[str]) -> dict[str, Any]:
    root = resolve_relative(path)
    if not root.is_dir():
        raise V12ProtocolFreezeError(f"artifact tree is missing: {root}")
    files = sorted(item for item in root.rglob("*") if item.is_file())
    if not files or any(item.is_symlink() for item in files):
        raise V12ProtocolFreezeError(f"artifact tree is empty or symlinked: {root}")
    rows: list[dict[str, Any]] = []
    digest = hashlib.sha256()
    for item in files:
        relative = item.relative_to(root).as_posix()
        sha256 = forensic.sha256_file(item)
        size = item.stat().st_size
        rows.append({"path": relative, "sha256": sha256, "size_bytes": size})
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(sha256.encode("ascii"))
        digest.update(b"\0")
        digest.update(str(size).encode("ascii"))
        digest.update(b"\n")
    return {
        "path": root.relative_to(REPO_ROOT).as_posix(),
        "tree_sha256": digest.hexdigest(),
        "file_count": len(rows),
        "files": rows,
    }


def artifact_snapshot(path: str | os.PathLike[str]) -> dict[str, Any]:
    """Return an idempotency snapshot that also distinguishes symlinks."""

    resolved = _lexical_absolute(path)
    exists = os.path.lexists(resolved)
    if not exists:
        return {"path": str(resolved), "lexists": False}
    status = os.lstat(resolved)
    kind = (
        "regular"
        if stat.S_ISREG(status.st_mode)
        else "directory"
        if stat.S_ISDIR(status.st_mode)
        else "symlink"
        if stat.S_ISLNK(status.st_mode)
        else "other"
    )
    result: dict[str, Any] = {
        "path": str(resolved),
        "lexists": True,
        "kind": kind,
        "size_bytes": int(status.st_size),
    }
    if kind == "regular":
        result["sha256"] = hashlib.sha256(resolved.read_bytes()).hexdigest()
    elif kind == "symlink":
        result["target"] = os.readlink(resolved)
    return result


def repository_artifact_snapshot(path: str | os.PathLike[str]) -> dict[str, Any]:
    """Return a movable snapshot whose path is repository-relative."""

    result = artifact_snapshot(path)
    try:
        relative = _lexical_absolute(path).relative_to(REPO_ROOT)
    except ValueError as exc:
        raise V12ProtocolFreezeError(
            f"portable snapshot is outside repository: {path}"
        ) from exc
    result["path"] = relative.as_posix()
    return result


def _symlink_ancestors(path: Path) -> list[str]:
    result: list[str] = []
    for ancestor in (path.parent, *path.parent.parents):
        try:
            if stat.S_ISLNK(os.lstat(ancestor).st_mode):
                result.append(str(ancestor))
        except FileNotFoundError:
            continue
    return result


def _write_json_exclusive_lexical(path: Path, payload: Mapping[str, Any]) -> Path:
    """Publish once under an existing, lexically checked directory."""

    destination = _lexical_absolute(path)
    ancestors = _symlink_ancestors(destination)
    if ancestors:
        raise V12ProtocolFreezeError(
            f"protocol freeze parent contains symlink(s): {ancestors}"
        )
    try:
        parent_status = os.lstat(destination.parent)
    except FileNotFoundError as exc:
        raise V12ProtocolFreezeError(
            f"protocol freeze parent must already exist: {destination.parent}"
        ) from exc
    if not stat.S_ISDIR(parent_status.st_mode):
        raise V12ProtocolFreezeError(
            f"protocol freeze parent is not a directory: {destination.parent}"
        )
    if os.path.lexists(destination):
        raise V12ProtocolFreezeError(
            f"protocol freeze exists/no-clobber: {destination}"
        )
    encoded = forensic.canonical_json_bytes(payload)
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_BINARY", 0)
    try:
        descriptor = os.open(destination, flags, 0o644)
    except FileExistsError as exc:
        raise V12ProtocolFreezeError(
            f"protocol freeze exists/no-clobber: {destination}"
        ) from exc
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
    except Exception:
        # A partial exclusive claim is deliberately retained and consumes the
        # destination fail-closed; it must never be silently retried.
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
    return destination


def _source_records() -> dict[str, dict[str, Any]]:
    return {
        name: _record(path)
        for name, path in sorted(contract.SOURCE_RELATIVE_PATHS.items())
    }


def _input_records() -> dict[str, dict[str, Any]]:
    return {
        name: _record(path)
        for name, path in sorted(contract.INPUT_RELATIVE_PATHS.items())
    }


def _authoritative_hash_gate(
    sources: Mapping[str, Mapping[str, Any]],
    inputs: Mapping[str, Mapping[str, Any]],
) -> dict[str, Any]:
    observed: dict[str, str | None] = {}
    checks: dict[str, bool] = {}
    for name, expected in contract.V11_AUTHORITATIVE_SHA256.items():
        record = sources.get(name) if name in sources else inputs.get(name)
        value = record.get("sha256") if isinstance(record, Mapping) else None
        observed[name] = value if isinstance(value, str) else None
        checks[name] = value == expected
    return {
        "passed": bool(checks) and all(checks.values()),
        "checks": checks,
        "expected_sha256": copy.deepcopy(contract.V11_AUTHORITATIVE_SHA256),
        "observed_sha256": observed,
    }


def _authority_gate() -> dict[str, Any]:
    false_flags = (
        "pipeline_execution_authorized",
        "actor_fit_execution_authorized",
        "environment_reset_authorized",
        "environment_step_authorized",
        "design_audit_fit_authorized",
        "offline_teacher_labeling_authorized",
        "critic_updates_authorized",
        "ppo_updates_authorized",
        "protected_trial_access_authorized",
        "reserve_trial_access_authorized",
        "runtime_promotion_authorized",
        "physical_gate_relaxation_authorized",
        "primary_grf_modification_authorized",
        "detector_or_fsm_modification_authorized",
        "sea_semantic_modification_authorized",
        "retry_authorized",
        "sweep_authorized",
        "rescue_authorized",
    )
    expected_keys = {
        "authority_date",
        "authority_text",
        "authority_scope",
        "design_and_freeze_authorized",
        *false_flags,
    }
    checks = {
        "exact_schema": set(contract.AUTHORITY) == expected_keys,
        "identity": contract.AUTHORITY.get("authority_date") == "2026-08-09"
        and contract.AUTHORITY.get("authority_text") == "procedi"
        and contract.AUTHORITY.get("authority_scope")
        == "V12_DESIGN_AND_PROTOCOL_FREEZE_ONLY",
        "design_freeze_only_true_flag": contract.AUTHORITY.get(
            "design_and_freeze_authorized"
        )
        is True,
        "all_execution_and_relaxation_flags_false": all(
            contract.AUTHORITY.get(name) is False for name in false_flags
        ),
        "boolean_types_exact": type(
            contract.AUTHORITY.get("design_and_freeze_authorized")
        )
        is bool
        and all(type(contract.AUTHORITY.get(name)) is bool for name in false_flags),
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "false_flags": list(false_flags),
    }


def _v11_fit_and_freeze_provenance_audit() -> dict[str, Any]:
    """Dereference every V11 fit/freeze receipt, component, and claim."""

    candidate = _mapping(contract.V11_CANDIDATE_FREEZE_PATH)
    fit_bindings = candidate.get("fit_receipts")
    if not isinstance(fit_bindings, list) or len(fit_bindings) != len(
        contract.prior.FIT_STAGES
    ):
        raise V12ProtocolFreezeError("V11 candidate fit bindings drifted")
    pipeline_claim = _record(contract.prior.PIPELINE_CLAIM_PATH)
    design_audit = _record(contract.prior.DESIGN_AUDIT_RECEIPT_PATH)
    source_h0 = _tree_record(contract.prior.SOURCE_H0_MODULE_PATH)
    rows: list[dict[str, Any]] = []
    for binding, stage in zip(fit_bindings, contract.prior.FIT_STAGES, strict=True):
        receipt_path = contract.prior.FIT_RECEIPT_PATHS[stage]
        receipt_record = _record(receipt_path)
        if (
            not isinstance(binding, Mapping)
            or set(binding) != {"fit_stage", "receipt"}
            or binding.get("fit_stage") != stage
            or binding.get("receipt") != receipt_record
        ):
            raise V12ProtocolFreezeError(f"V11 fit binding drifted: {stage}")
        receipt = _mapping(receipt_path)
        root = resolve_relative(contract.prior.FIT_ROOTS[stage])
        expected_dagger_receipts = [
            _record(
                resolve_relative(
                    contract.prior.canonical_collection_case(case_id, round_index)[
                        "destination"
                    ]
                )
                / "receipt.json"
            )
            for round_index in contract.prior.FIT_COMPLETED_ROUNDS[stage]
            for case_id in contract.prior.COLLECTION_CASE_IDS
        ]
        actual_components = {
            "adaptation_history": _record(root / "adaptation_history.json"),
            "adaptation_report": _record(root / "adaptation_report.json"),
            "corpus": _record(root / "corpus.npz"),
            "gate": _record(root / "gate.json"),
            "summary": _record(root / "summary.json"),
            "pipeline_claim": pipeline_claim,
            "worker_claim": _record(contract.prior.worker_claim_path(f"fit_{stage}")),
            "design_audit": design_audit,
        }
        module = _tree_record(contract.prior.MODULE_PATHS[stage])
        component_binding_passed = all(
            receipt.get(name) == record for name, record in actual_components.items()
        )
        checks = {
            "receipt_passed": receipt.get("passed") is True
            and receipt.get("status") == contract.prior.FIT_PASS_STATUS
            and receipt.get("fit_stage") == stage,
            "candidate_binding": binding.get("receipt") == receipt_record,
            "components_current": component_binding_passed,
            "dagger_receipts_current": receipt.get("dagger_receipts")
            == expected_dagger_receipts,
            "module_current": receipt.get("module") == module,
            "source_h0_current": receipt.get("source_h0") == source_h0,
            "zero_forbidden_updates": receipt.get("actor_updates") == 1
            and receipt.get("critic_updates") == 0
            and receipt.get("ppo_updates") == 0,
        }
        if not all(checks.values()):
            raise V12ProtocolFreezeError(f"V11 fit provenance drifted: {stage}")
        rows.append(
            {
                "fit_stage": stage,
                "passed": True,
                "checks": checks,
                "receipt": receipt_record,
                "components": actual_components,
                "dagger_receipts": expected_dagger_receipts,
                "module": module,
            }
        )

    candidate_root = resolve_relative(contract.V11_CANDIDATE_FREEZE_PATH).parent
    candidate_components = {
        "pipeline_claim": pipeline_claim,
        "worker_claim": _record(contract.prior.worker_claim_path("freeze_p3")),
        "design_audit_receipt": design_audit,
        "p3_fit_receipt": _record(contract.prior.FIT_RECEIPT_PATHS["p3"]),
        "summary": _record(candidate_root / "candidate_freeze_summary.json"),
        "gate": _record(candidate_root / "candidate_freeze_gate.json"),
    }
    candidate_checks = {
        "passed": candidate.get("passed") is True
        and candidate.get("status") == contract.prior.FREEZE_PASS_STATUS,
        "components_current": all(
            candidate.get(name) == record
            for name, record in candidate_components.items()
        ),
        "fit_receipts_current": all(row["passed"] is True for row in rows),
        "candidate_module_current": candidate.get("candidate_module")
        == _tree_record(contract.prior.MODULE_PATHS["p3"]),
        "source_h0_current": candidate.get("source_h0") == source_h0,
        "source_h0_authoritative": source_h0.get("tree_sha256")
        == contract.SOURCE_H0_TREE_SHA256,
    }
    return {
        "passed": all(candidate_checks.values()),
        "checks": candidate_checks,
        "fit_receipts": rows,
        "candidate_freeze": _record(contract.V11_CANDIDATE_FREEZE_PATH),
        "candidate_components": candidate_components,
        "pipeline_claim": pipeline_claim,
        "source_h0": source_h0,
    }


def _v11_terminal_gate() -> dict[str, Any]:
    ledger = _mapping(contract.V11_TERMINAL_LEDGER_PATH)
    freeze = _mapping(contract.V11_CANDIDATE_FREEZE_PATH)
    summary = _mapping(contract.V11_FINAL_FAILURE_SUMMARY_PATH)
    gate = _mapping(contract.V11_FINAL_FAILURE_GATE_PATH)
    failure = _mapping(contract.V11_FINAL_FAILURE_PATH)
    failure_root = resolve_relative(contract.V11_FINAL_FAILURE_ROOT)
    failure_artifacts = failure.get("artifacts")
    run_start = _mapping(failure_root / "run_start.json")
    actual_claims = {
        "pipeline_claim": _record(contract.prior.PIPELINE_CLAIM_PATH),
        "worker_claim": _record(
            contract.prior.worker_claim_path(
                f"final__{contract.V11_FINAL_FAILURE_CASE_ID}"
            )
        ),
    }
    actual_artifacts = {
        "run_start": _record(failure_root / "run_start.json"),
        "trace": _record(failure_root / "trace.json"),
        "partial_summary": _record(failure_root / "partial_summary.json"),
        "summary": _record(failure_root / "summary.json"),
        "gate": _record(failure_root / "gate.json"),
        "steps": [
            _record(failure_root / "steps" / f"{step:06d}.json")
            for step in range(1, 260)
        ],
    }
    checks = {
        "ledger_identity": ledger.get("protocol_id") == contract.prior.PROTOCOL_ID
        and ledger.get("pipeline_id") == contract.prior.PIPELINE_ID,
        "ledger_terminal": ledger.get("status") == contract.prior.PIPELINE_FAIL_STATUS
        and ledger.get("passed") is False
        and ledger.get("next_stage") == "STOP_V11_TERMINAL_NO_RETRY_SWEEP_OR_RESCUE",
        "four_actor_fits": ledger.get("actor_updates") == 4
        and ledger.get("fit_actor_update_count") == 4
        and ledger.get("critic_updates") == 0
        and ledger.get("ppo_updates") == 0,
        "three_collection_rounds": ledger.get("safe_dagger_rounds") == 3,
        "protected_closed": ledger.get("protected_trials_opened") == []
        and ledger.get("reserve_trials_opened") == [],
        "candidate_frozen_not_promoted": freeze.get("passed") is True
        and freeze.get("candidate_frozen") is True
        and freeze.get("candidate_fit_stage") == "p3",
        "failure_case": summary.get("case_id") == contract.V11_FINAL_FAILURE_CASE_ID
        and summary.get("steps") == 259
        and summary.get("end_reason") == "grf_penetration"
        and summary.get("phase_valid_cycle_count") == 1,
        "physical_limit_preserved": summary.get("grf_penetration_max_m")
        == 0.025569911689665874
        and summary.get("grf_penetration_max_m") > contract.PENETRATION_LIMIT_M,
        "final_gate_failed": gate.get("passed") is False
        and gate.get("status") == contract.prior.FINAL_ROLLOUT_FAIL_STATUS,
        "failure_receipt": failure.get("passed") is False
        and failure.get("status") == contract.prior.FINAL_ROLLOUT_FAIL_STATUS,
        "failure_artifact_chain": isinstance(failure_artifacts, Mapping)
        and all(
            failure_artifacts.get(name) == record
            for name, record in actual_artifacts.items()
        ),
        "failure_claim_chain": all(
            run_start.get(name) == record for name, record in actual_claims.items()
        )
        and run_start.get("stage_id") == f"final__{contract.V11_FINAL_FAILURE_CASE_ID}",
        "no_retry": ledger.get("retry_authorized") is False
        and ledger.get("sweep_authorized") is False,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "ledger": _record(contract.V11_TERMINAL_LEDGER_PATH),
        "candidate_freeze": _record(contract.V11_CANDIDATE_FREEZE_PATH),
        "failure_summary": _record(contract.V11_FINAL_FAILURE_SUMMARY_PATH),
        "failure_gate": _record(contract.V11_FINAL_FAILURE_GATE_PATH),
        "failure": _record(contract.V11_FINAL_FAILURE_PATH),
        "failure_artifact_chain": actual_artifacts,
        "failure_claim_chain": actual_claims,
    }


def _streaks(rows: Sequence[Mapping[str, Any]]) -> list[tuple[int, int]]:
    result: list[tuple[int, int]] = []
    start: int | None = None
    previous = 0
    for offset, row in enumerate(rows, start=1):
        active = row.get("safety_latch_active") is True
        if active and start is None:
            start = offset
        if not active and start is not None:
            result.append((start, previous))
            start = None
        previous = offset
    if start is not None:
        result.append((start, previous))
    return result


def _v11_collection_recovery_audit() -> dict[str, Any]:
    candidate_freeze = _mapping(contract.V11_CANDIDATE_FREEZE_PATH)
    frozen_bindings = candidate_freeze.get("collection_receipts")
    if not isinstance(frozen_bindings, list) or len(frozen_bindings) != 6:
        raise V12ProtocolFreezeError("V11 candidate-freeze collection bindings drifted")
    frozen_receipts: dict[tuple[int, str], Mapping[str, Any]] = {}
    for binding in frozen_bindings:
        if not isinstance(binding, Mapping) or set(binding) != {
            "round_index",
            "case_id",
            "receipt",
        }:
            raise V12ProtocolFreezeError("malformed V11 frozen collection binding")
        key = (int(binding["round_index"]), str(binding["case_id"]))
        if key in frozen_receipts or not isinstance(binding["receipt"], Mapping):
            raise V12ProtocolFreezeError("duplicate V11 frozen collection binding")
        frozen_receipts[key] = binding["receipt"]
    cases: list[dict[str, Any]] = []
    aggregate = {
        "rows": 0,
        "latch_active_rows": 0,
        "near_latch_rows": 0,
        "at_or_above_activation_rows": 0,
        "at_or_above_20mm_rows": 0,
        "at_or_above_24mm_rows": 0,
        "current_at_or_above_24mm_rows": 0,
        "current_at_or_above_25mm_rows": 0,
        "teacher_dependent_rows": 0,
    }
    for binding in contract.V11_COLLECTION_INPUTS:
        round_index = int(binding["round_index"])
        case_id = str(binding["case_id"])
        root = resolve_relative(binding["root"])
        trace_path = root / "trace.json"
        partial_summary_path = root / "partial_summary.json"
        summary_path = root / "summary.json"
        gate_path = root / "gate.json"
        receipt_path = root / "receipt.json"
        rows = _sequence(trace_path)
        summary = _mapping(summary_path)
        gate = _mapping(gate_path)
        receipt = _mapping(receipt_path)
        artifacts = receipt.get("artifacts")
        if not isinstance(artifacts, Mapping):
            raise V12ProtocolFreezeError("V11 collection receipt artifacts drifted")
        actual_persisted = {
            "trace": _record(trace_path),
            "partial_summary": _record(partial_summary_path),
            "summary": _record(summary_path),
        }
        actual_terminal = {
            **actual_persisted,
            "gate": _record(gate_path),
            "run_start": _record(root / "run_start.json"),
        }
        actual_claims = {
            "pipeline_claim": _record(contract.prior.PIPELINE_CLAIM_PATH),
            "worker_claim": _record(
                contract.prior.worker_claim_path(f"collect_r{round_index}__{case_id}")
            ),
        }
        recorded_steps = artifacts.get("steps")
        actual_steps = [
            _record(root / "steps" / f"{step:06d}.json")
            for step in range(1, contract.EXPECTED_STEPS + 1)
        ]
        artifact_binding_passed = (
            all(
                artifacts.get(name) == record
                for name, record in actual_terminal.items()
            )
            and recorded_steps == actual_steps
        )
        claim_binding_passed = all(
            receipt.get(name) == record for name, record in actual_claims.items()
        )
        receipt_record = _record(receipt_path)
        candidate_freeze_binding_passed = (
            frozen_receipts.get((round_index, case_id)) == receipt_record
        )
        expected_gate = contract.prior.collection_gate(summary, round_index=round_index)
        expected_gate["persisted_before_gate"] = actual_persisted
        if (
            len(rows) != contract.EXPECTED_STEPS
            or receipt.get("passed") is not True
            or receipt.get("case_id") != case_id
            or receipt.get("round_index") != round_index
            or not artifact_binding_passed
            or not claim_binding_passed
            or not candidate_freeze_binding_passed
            or forensic.canonical_json_bytes(gate)
            != forensic.canonical_json_bytes(expected_gate)
        ):
            raise V12ProtocolFreezeError(
                f"V11 collection closure drifted: r{round_index}/{case_id}"
            )
        typed_rows: list[Mapping[str, Any]] = []
        near = 0
        above_15 = 0
        above_20 = 0
        above_24 = 0
        current_above_24 = 0
        current_above_25 = 0
        latch = 0
        dependent = 0
        for step, raw in enumerate(rows, start=1):
            if not isinstance(raw, Mapping) or raw.get("step") != step:
                raise V12ProtocolFreezeError("V11 collection trace order drifted")
            previous = raw.get("previous_penetration_m")
            current = raw.get("grf_penetration_m")
            if (
                isinstance(previous, bool)
                or not isinstance(previous, (int, float))
                or not np.isfinite(float(previous))
                or float(previous) < 0.0
            ):
                raise V12ProtocolFreezeError("V11 penetration metadata is malformed")
            if (
                isinstance(current, bool)
                or not isinstance(current, (int, float))
                or not np.isfinite(float(current))
                or float(current) < 0.0
            ):
                raise V12ProtocolFreezeError(
                    "V11 current penetration metadata is malformed"
                )
            active = raw.get("safety_latch_active") is True
            near += int(0.010 < float(previous) < 0.015 and not active)
            above_15 += int(float(previous) >= 0.015)
            above_20 += int(float(previous) >= 0.020)
            above_24 += int(float(previous) >= 0.024)
            current_above_24 += int(float(current) >= 0.024)
            current_above_25 += int(float(current) >= 0.025)
            latch += int(active)
            dependent += int(float(raw.get("effective_alpha", 1.0)) < 1.0)
            typed_rows.append(raw)
        intervals = _streaks(typed_rows)
        row = {
            "round_index": round_index,
            "case_id": case_id,
            "rows": len(rows),
            "latch_active_rows": latch,
            "latch_fraction": latch / len(rows),
            "latch_intervals_inclusive": [list(pair) for pair in intervals],
            "max_consecutive_latch_steps": max(
                (end - start + 1 for start, end in intervals), default=0
            ),
            "near_latch_rows": near,
            "at_or_above_activation_rows": above_15,
            "at_or_above_20mm_rows": above_20,
            "at_or_above_24mm_rows": above_24,
            "current_at_or_above_24mm_rows": current_above_24,
            "current_at_or_above_25mm_rows": current_above_25,
            "teacher_dependent_rows": dependent,
            "candidate_freeze_receipt_binding_passed": (
                candidate_freeze_binding_passed
            ),
            "receipt_artifact_binding_passed": artifact_binding_passed,
            "receipt_claim_binding_passed": claim_binding_passed,
            "run_start": actual_terminal["run_start"],
            "trace": actual_terminal["trace"],
            "partial_summary": actual_terminal["partial_summary"],
            "summary": actual_terminal["summary"],
            "gate": actual_terminal["gate"],
            "receipt": receipt_record,
            "claims": actual_claims,
        }
        cases.append(row)
        for name in aggregate:
            aggregate[name] += int(row[name])
    checks = {
        "six_cases": len(cases) == 6,
        "candidate_freeze_to_receipts_bound": all(
            row["candidate_freeze_receipt_binding_passed"] for row in cases
        ),
        "receipts_to_artifacts_bound": all(
            row["receipt_artifact_binding_passed"] for row in cases
        ),
        "receipts_to_claims_bound": all(
            row["receipt_claim_binding_passed"] for row in cases
        ),
        "three_thousand_rows": aggregate["rows"] == 3000,
        "all_teacher_dependent": aggregate["teacher_dependent_rows"] == 3000,
        "latch_majority": aggregate["latch_active_rows"] == 1597
        and aggregate["latch_active_rows"] / aggregate["rows"] > 0.5,
        "risk_rows_present": aggregate["near_latch_rows"] > 0
        and aggregate["at_or_above_activation_rows"] > 0,
        "shielded_24mm_tail_sparse": aggregate["current_at_or_above_24mm_rows"] == 8
        and aggregate["current_at_or_above_25mm_rows"] == 0,
        "no_v12_reweight_of_shielded_rows": contract.RECOVERY_WEIGHTING[
            "shielded_dagger_risk_modifier"
        ]
        == 1.0,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "aggregate": {
            **aggregate,
            "latch_fraction": aggregate["latch_active_rows"] / aggregate["rows"],
        },
        "cases": cases,
        "scientific_interpretation": (
            "V11 shielded rows are valid labels but do not contain the pure "
            "closed-loop branch; V12 must label each pure probe after rollout"
        ),
    }


def _v11_corpus_audit() -> dict[str, Any]:
    path = resolve_relative(contract.V11_P3_CORPUS_PATH)
    try:
        with np.load(path, allow_pickle=False) as archive:
            arrays = {name: archive[name] for name in archive.files}
    except Exception as exc:
        raise V12ProtocolFreezeError("V11 P3 corpus cannot be read safely") from exc
    expected_keys = {
        "observations",
        "actions",
        "reset_mask",
        "actor_feature_names",
        "case_ids",
        "step_indices",
        "tranche_ids",
        "origins",
        "training_indices",
    }
    observations = arrays["observations"]
    base = np.ascontiguousarray(observations[:3000], dtype=np.float32)
    mean = base.mean(axis=0, dtype=np.float64).astype(np.float32)
    raw_std = base.std(axis=0, dtype=np.float64, ddof=0).astype(np.float32)
    std = np.ascontiguousarray(
        np.maximum(raw_std, np.float32(1.0e-4)), dtype=np.float32
    )
    normalized = np.ascontiguousarray((observations - mean) / std, dtype=np.float32)
    included = np.asarray(
        contract.COVERAGE_WEIGHTING["included_feature_indices"], dtype=np.int64
    )
    features = np.ascontiguousarray(normalized[:, included], dtype=np.float32)
    tree_spec = contract.COVERAGE_WEIGHTING["loo_tree"]
    tree = cKDTree(
        features,
        leafsize=tree_spec["leafsize"],
        compact_nodes=tree_spec["compact_nodes"],
        balanced_tree=tree_spec["balanced_tree"],
    )
    _query_distance, query_indices = tree.query(
        features,
        k=tree_spec["query_k"],
        eps=tree_spec["eps"],
        p=tree_spec["p"],
        workers=tree_spec["workers"],
    )
    tie_spec = contract.COVERAGE_WEIGHTING["tie_audit"]
    _extended_distance, extended_indices = tree.query(
        features,
        k=tie_spec["extended_query_k"],
        eps=tree_spec["eps"],
        p=tree_spec["p"],
        workers=tree_spec["workers"],
    )

    def _nearest_from_candidates(
        candidates_matrix: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        selected = np.empty(len(features), dtype=np.int64)
        tie_counts = np.empty(len(features), dtype=np.int64)
        for row_index, candidates in enumerate(candidates_matrix):
            alternatives = [
                int(value) for value in candidates if int(value) != row_index
            ]
            if not alternatives:
                raise V12ProtocolFreezeError(
                    "coverage LOO query did not find a neighbour"
                )
            candidate_features = features[np.asarray(alternatives, dtype=np.int64)]
            delta = candidate_features.astype(np.float64) - features[row_index].astype(
                np.float64
            )
            distances = np.sqrt(np.sum(np.square(delta), axis=1) / features.shape[1])
            minimum = float(np.min(distances))
            ties = [
                candidate
                for candidate, distance in zip(alternatives, distances, strict=True)
                if float(distance) == minimum
            ]
            selected[row_index] = min(ties)
            tie_counts[row_index] = len(ties)
        return selected, tie_counts

    nearest, query_tie_counts = _nearest_from_candidates(query_indices)
    extended_nearest, extended_tie_counts = _nearest_from_candidates(extended_indices)
    unique_observation_count = int(np.unique(features, axis=0).shape[0])
    maximum_tie_count = int(np.max(extended_tie_counts))
    query_matches_extended = bool(np.array_equal(nearest, extended_nearest))
    delta = features.astype(np.float64) - features[nearest].astype(np.float64)
    loo_distances = np.sqrt(
        np.sum(np.square(delta), axis=1, dtype=np.float64) / features.shape[1]
    )
    p95 = float(np.percentile(loo_distances, 95, method="linear"))
    coverage_hashes = {
        "observations": _array_sha256(observations),
        "base_observations": _array_sha256(base),
        "mean": _array_sha256(mean),
        "std": _array_sha256(std),
        "normalized": _array_sha256(normalized),
        "features": _array_sha256(features),
        "nearest_indices": _array_sha256(nearest),
        "loo_distances": _array_sha256(loo_distances),
    }
    coverage_expected = contract.COVERAGE_WEIGHTING
    checks = {
        "keys": set(arrays) == expected_keys,
        "observations": arrays.get("observations", np.empty(0)).shape == (6000, 35)
        and arrays["observations"].dtype == np.dtype("float32")
        and bool(np.all(np.isfinite(arrays["observations"]))),
        "actions": arrays.get("actions", np.empty(0)).shape == (6000, 2)
        and arrays["actions"].dtype == np.dtype("float32")
        and bool(np.all(np.isfinite(arrays["actions"]))),
        "reset_rows": arrays.get("reset_mask", np.empty(0)).shape == (6000,)
        and int(np.count_nonzero(arrays["reset_mask"])) == 12,
        "clock_layout": arrays.get("observations", np.empty((0, 0))).shape == (6000, 35)
        and bool(np.all(arrays["observations"][:, 0] == 0.0))
        and bool(np.all(arrays["observations"][:, 1] == 1.0)),
        "training_indices": arrays.get("training_indices", np.empty(0)).shape == (6000,)
        and bool(np.array_equal(arrays["training_indices"], np.arange(6000))),
        "coverage_observations_hash": coverage_hashes["observations"]
        == coverage_expected["reference_observations_sha256"],
        "coverage_base_hash": coverage_hashes["base_observations"]
        == coverage_expected["base_observations_sha256"],
        "coverage_normalization_hashes": coverage_hashes["mean"]
        == coverage_expected["normalization_mean_sha256"]
        and coverage_hashes["std"] == coverage_expected["normalization_std_sha256"],
        "coverage_matrix_hashes": coverage_hashes["normalized"]
        == coverage_expected["normalized_observations_sha256"]
        and coverage_hashes["features"]
        == coverage_expected["normalized_feature_matrix_sha256"],
        "coverage_loo_hashes": coverage_hashes["nearest_indices"]
        == coverage_expected["loo_nearest_indices_sha256"]
        and coverage_hashes["loo_distances"]
        == coverage_expected["loo_distances_sha256"],
        "coverage_p95": abs(p95 - coverage_expected["loo_p95"]) <= 1.0e-12,
        "coverage_tie_audit": unique_observation_count
        == tie_spec["unique_observation_count"]
        and maximum_tie_count == tie_spec["maximum_minimum_distance_tie_count"]
        and query_matches_extended is tie_spec["query_k_matches_extended_query"]
        and bool(np.array_equal(query_tie_counts, extended_tie_counts)),
        "metadata_shapes": arrays.get("actor_feature_names", np.empty(0)).shape == (35,)
        and arrays.get("case_ids", np.empty(0)).shape == (6000,)
        and arrays.get("step_indices", np.empty(0)).shape == (6000,)
        and arrays.get("tranche_ids", np.empty(0)).shape == (6000,)
        and arrays.get("origins", np.empty(0)).shape == (6000,),
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "artifact": _record(path),
        "observations_sha256": _array_sha256(arrays["observations"]),
        "actions_sha256": _array_sha256(arrays["actions"]),
        "reset_mask_sha256": _array_sha256(arrays["reset_mask"]),
        "all_array_sha256": {
            name: _array_sha256(arrays[name]) for name in sorted(arrays)
        },
        "seed_rows": 6000,
        "risk_reweighted_seed_rows": 0,
        "coverage_audit": {
            "hashes": coverage_hashes,
            "loo_p95": p95,
            "reference_loo_p95": coverage_expected["loo_p95"],
            "absolute_tolerance": 1.0e-12,
            "included_feature_indices": included.tolist(),
            "feature_count": int(features.shape[1]),
            "tie_audit": {
                "extended_query_k": tie_spec["extended_query_k"],
                "unique_observation_count": unique_observation_count,
                "maximum_minimum_distance_tie_count": maximum_tie_count,
                "query_k_matches_extended_query": query_matches_extended,
                "query_and_extended_tie_counts_match": bool(
                    np.array_equal(query_tie_counts, extended_tie_counts)
                ),
            },
        },
    }


def _v11_pure_failure_coverage_audit() -> dict[str, Any]:
    trace_path = resolve_relative(contract.V11_FINAL_FAILURE_TRACE_PATH)
    rows = _sequence(trace_path)
    if any(
        not isinstance(row, Mapping) or row.get("step") != index
        for index, row in enumerate(rows, start=1)
    ):
        raise V12ProtocolFreezeError("V11 pure failure trace order drifted")
    try:
        observations = np.ascontiguousarray(
            np.asarray([row["v26_observation"] for row in rows]), dtype=np.float32
        )
    except Exception as exc:
        raise V12ProtocolFreezeError("V11 pure observations are malformed") from exc
    corpus_path = resolve_relative(contract.V11_P3_CORPUS_PATH)
    with np.load(corpus_path, allow_pickle=False) as archive:
        reference = np.ascontiguousarray(archive["observations"], dtype=np.float32)
    base = np.ascontiguousarray(reference[:3000], dtype=np.float32)
    mean = base.mean(axis=0, dtype=np.float64).astype(np.float32)
    raw_std = base.std(axis=0, dtype=np.float64, ddof=0).astype(np.float32)
    std = np.ascontiguousarray(
        np.maximum(raw_std, np.float32(1.0e-4)), dtype=np.float32
    )
    reference_features = np.ascontiguousarray(
        ((reference - mean) / std)[:, 2:35], dtype=np.float32
    )
    query_features = np.ascontiguousarray(
        ((observations - mean) / std)[:, 2:35], dtype=np.float32
    )
    reference_float64 = reference_features.astype(np.float64)
    nearest = np.empty(len(query_features), dtype=np.int64)
    distances = np.empty(len(query_features), dtype=np.float64)
    certificate_margins = np.empty(len(query_features), dtype=np.float64)
    certificate_thresholds = np.empty(len(query_features), dtype=np.float64)
    audit_k = int(
        contract.COVERAGE_WEIGHTING["new_row_query"]["global_minimum_candidate_audit_k"]
    )
    for row_index, query in enumerate(query_features):
        delta = reference_float64 - query.astype(np.float64)
        squared = np.square(delta)
        approximate_sums = np.sum(squared, axis=1, dtype=np.float64)
        partition = np.argpartition(approximate_sums, audit_k)
        candidate_indices = partition[:audit_k]
        exact_sums = np.asarray(
            [math.fsum(squared[index].tolist()) for index in candidate_indices],
            dtype=np.float64,
        )
        minimum_sum = float(np.min(exact_sums))
        ties = candidate_indices[exact_sums == minimum_sum]
        nearest[row_index] = int(np.min(ties))
        distances[row_index] = math.sqrt(minimum_sum / reference_features.shape[1])
        excluded_minimum = float(np.min(approximate_sums[partition[audit_k:]]))
        certificate_margins[row_index] = excluded_minimum - minimum_sum
        certificate_thresholds[row_index] = 1.0e-10 * max(1.0, excluded_minimum)
    expected = contract.V11_PURE_FAILURE_COVERAGE
    p95 = float(contract.COVERAGE_WEIGHTING["loo_p95"])
    ood = distances > p95
    hashes = {
        "trace_observations": _array_sha256(observations),
        "normalized_features": _array_sha256(query_features),
        "nearest_indices": _array_sha256(nearest),
        "nearest_distances": _array_sha256(distances),
    }
    checks = {
        "trace_artifact": _record(trace_path).get("sha256")
        == contract.V11_AUTHORITATIVE_SHA256["v11_final_failure_trace"],
        "row_count": len(rows) == expected["row_count"]
        and observations.shape == (expected["row_count"], 35),
        "finite_observations": bool(np.all(np.isfinite(observations))),
        "pure_unblended": all(
            row.get("teacher_enabled") is False
            and row.get("teacher_query_count") == 0
            and row.get("blending_enabled") is False
            and row.get("safety_latch_enabled") is False
            and row.get("served_action_teacher_dependency_count") == 0
            for row in rows
        ),
        "terminal_penetration": rows[-1].get("end_reason") == "grf_penetration"
        and rows[-1].get("grf_penetration_m") == 0.025569911689665874,
        "legacy_teacher_payload_absent": all(
            "counterfactual_teacher_mean" not in row
            and "counterfactual_teacher_observation" not in row
            for row in rows
        ),
        "hashes": hashes["trace_observations"] == expected["trace_observations_sha256"]
        and hashes["normalized_features"] == expected["normalized_features_sha256"]
        and hashes["nearest_indices"] == expected["nearest_indices_sha256"]
        and hashes["nearest_distances"] == expected["nearest_distances_sha256"],
        "ood_count": int(np.count_nonzero(ood)) == expected["ood_row_count"],
        "late_branch_all_ood": bool(np.all(ood[200:]))
        is expected["all_steps_201_through_259_ood"],
        "anchor_distances": abs(float(distances[200]) - expected["step_201_distance"])
        <= 1.0e-15
        and abs(float(distances[258]) - expected["step_259_distance"]) <= 1.0e-15,
        "global_minimum_certificate": audit_k == 64
        and bool(np.all(certificate_margins > certificate_thresholds)),
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "artifact": _record(trace_path),
        "hashes": hashes,
        "row_count": len(rows),
        "ood_row_count": int(np.count_nonzero(ood)),
        "all_steps_201_through_259_ood": bool(np.all(ood[200:])),
        "step_201_distance": float(distances[200]),
        "step_259_distance": float(distances[258]),
        "minimum_global_certificate_margin_sum": float(np.min(certificate_margins)),
        "scientific_interpretation": (
            "The terminal pure branch is outside the V11 seed coverage and the "
            "saved trace cannot reconstruct coherent same-state teacher labels."
        ),
    }


def _path_isolation_audit(
    declared_paths: Mapping[str, str | os.PathLike[str]] | None = None,
) -> dict[str, Any]:
    source = (
        contract.declared_mutation_paths()
        if declared_paths is None
        else dict(declared_paths)
    )
    raw_paths = {name: str(path) for name, path in source.items()}
    posix_paths = {
        name: contract.PurePosixPath(value) for name, value in raw_paths.items()
    }
    lexical_paths = {
        name: _lexical_absolute(value) for name, value in posix_paths.items()
    }
    resolved_paths = {name: path.resolve() for name, path in lexical_paths.items()}
    run_root = contract.RUN_ROOT
    run_root_resolved = resolve_relative(run_root)
    outside_run_root = {
        contract.PROTOCOL_FREEZE_PATH,
        contract.EXECUTION_LOCK_PATH,
        contract.DESIGN_AUDIT_RECEIPT_PATH,
    }
    historical_root = resolve_relative(
        "validation/h0_primary_grf_split_adaptation_runs"
    )
    protected_data_root = resolve_relative("models/AB06-raw")
    protected_existing = {
        resolve_relative(path) for path in contract.SOURCE_RELATIVE_PATHS.values()
    } | {resolve_relative(path) for path in contract.INPUT_RELATIVE_PATHS.values()}
    violations: list[dict[str, str]] = []
    for name, raw in posix_paths.items():
        resolved = resolved_paths[name]
        reasons: list[str] = []
        if raw.is_absolute() or ".." in raw.parts or "\\" in raw_paths[name]:
            reasons.append("non_canonical_relative_path")
        if raw != contract.V12_VALIDATION_ROOT and (
            contract.V12_VALIDATION_ROOT not in raw.parents
        ):
            reasons.append("outside_v12_validation_namespace")
        if raw not in outside_run_root and not (
            raw == run_root or run_root in raw.parents
        ):
            reasons.append("outside_v12_run_root")
        if resolved not in {
            resolve_relative(path) for path in outside_run_root
        } and not (
            resolved == run_root_resolved or run_root_resolved in resolved.parents
        ):
            reasons.append("resolved_outside_v12_allowlist")
        if historical_root == resolved or historical_root in resolved.parents:
            reasons.append("historical_run_alias")
        if protected_data_root == resolved or protected_data_root in resolved.parents:
            reasons.append("protected_data_alias")
        if resolved in protected_existing:
            reasons.append("source_or_input_alias")
        if len(raw.as_posix()) >= 160:
            reasons.append("relative_path_too_long")
        for reason in reasons:
            violations.append({"name": name, "path": raw.as_posix(), "reason": reason})

    values = [path.as_posix() for path in posix_paths.values()]
    duplicate_values = sorted({value for value in values if values.count(value) > 1})
    actual_outside = {
        path
        for path in posix_paths.values()
        if path != run_root and run_root not in path.parents
    }
    required_claims = {
        contract.worker_claim_path(stage_id) for stage_id in contract.STAGE_IDS
    }
    required_receipts = {
        contract.stage_receipt_path(stage_id) for stage_id in contract.STAGE_IDS
    }
    declared_values = set(posix_paths.values())
    checks = {
        "nonempty_manifest": bool(posix_paths),
        "unique_names_and_paths": len(posix_paths) == len(source)
        and not duplicate_values,
        "exact_three_top_level_outputs": actual_outside == outside_run_root,
        "all_other_paths_contained_in_run_root": all(
            path in outside_run_root or path == run_root or run_root in path.parents
            for path in posix_paths.values()
        ),
        "worker_claim_root_declared": contract.WORKER_CLAIMS_ROOT in declared_values,
        "all_worker_claims_declared": required_claims <= declared_values,
        "all_stage_receipts_declared": required_receipts <= declared_values,
        "no_forbidden_or_alias_violation": not violations,
        "relative_paths_short": all(len(value) < 160 for value in values),
    }
    maximum_relative_path_length = max(map(len, values))
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "violations": violations,
        "duplicate_values": duplicate_values,
        "declared_mutation_paths": {
            name: path.as_posix() for name, path in posix_paths.items()
        },
        "declared_path_count": len(posix_paths),
        "worker_claim_count": len(required_claims),
        "stage_receipt_count": len(required_receipts),
        "historical_root_forbidden": historical_root.relative_to(REPO_ROOT).as_posix(),
        "protected_data_root_forbidden": protected_data_root.relative_to(
            REPO_ROOT
        ).as_posix(),
        "maximum_relative_path_length": maximum_relative_path_length,
        "maximum_checkout_root_length_for_legacy_windows": (
            258 - maximum_relative_path_length
        ),
        "maximum_checkout_root_length_for_preferred_windows": (
            238 - maximum_relative_path_length
        ),
        "legacy_windows_max_path_including_nul": 260,
        "preferred_windows_absolute_limit": 240,
        "cross_platform_relative_limit": 160,
    }


def _stage_topology_gate() -> dict[str, Any]:
    expected = (
        "fit_p0",
        "probe_p0",
        "label_p0",
        "collect_r1__deterministic_offset_minus_0p20",
        "collect_r1__stochastic_nominal_seed_126",
        "fit_p1",
        "probe_p1",
        "label_p1",
        "collect_r2__deterministic_offset_minus_0p20",
        "collect_r2__stochastic_nominal_seed_126",
        "fit_p2",
        "probe_p2",
        "label_p2",
        "collect_r3__deterministic_offset_minus_0p20",
        "collect_r3__stochastic_nominal_seed_126",
        "fit_p3",
        "probe_p3",
        "label_p3",
        "freeze_p3",
        "final__deterministic_offset_minus_0p20",
        "final__deterministic_offset_nominal",
        "final__deterministic_offset_plus_0p20",
        "final__stochastic_nominal_seed_126",
        "final__stochastic_nominal_seed_127",
        "final__stochastic_nominal_seed_128",
        "finalize_development",
    )
    claim_paths = [contract.worker_claim_path(stage) for stage in expected]
    receipt_paths = [contract.stage_receipt_path(stage) for stage in expected]
    checks = {
        "exact_order": contract.STAGE_IDS == expected,
        "twenty_six_stages": len(expected) == 26,
        "unique_stage_ids": len(expected) == len(set(expected)),
        "unique_worker_claims": len(claim_paths) == len(set(claim_paths)),
        "unique_stage_receipts": len(receipt_paths) == len(set(receipt_paths)),
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "expected": list(expected),
    }


def _assemble_protocol_freeze(occupancy: Mapping[str, bool]) -> dict[str, Any]:
    expected_occupancy_keys = {
        "protocol_freeze_unoccupied",
        "execution_lock_absent",
        "design_audit_absent",
        "run_root_absent",
    }
    if set(occupancy) != expected_occupancy_keys or not all(
        type(value) is bool for value in occupancy.values()
    ):
        raise V12ProtocolFreezeError("occupancy evidence schema drifted")
    v11_receipt_before = repository_artifact_snapshot(
        contract.prior.DESIGN_AUDIT_RECEIPT_PATH
    )
    sources = _source_records()
    inputs = _input_records()
    hashes = _authoritative_hash_gate(sources, inputs)
    authority = _authority_gate()
    topology = _stage_topology_gate()
    v11_fit_provenance = _v11_fit_and_freeze_provenance_audit()
    terminal = _v11_terminal_gate()
    collection = _v11_collection_recovery_audit()
    corpus = _v11_corpus_audit()
    pure_failure = _v11_pure_failure_coverage_audit()
    isolation = _path_isolation_audit()
    v11_receipt_after = repository_artifact_snapshot(
        contract.prior.DESIGN_AUDIT_RECEIPT_PATH
    )
    expected_future_sources = (
        "v12_recovery_weighted_fitter",
        "v12_pure_probe_observer_labeler",
        "v12_design_audit_runner",
        "v12_pipeline_runner",
        "v12_execution_tests",
    )
    design_gate_names = (
        "fit_gate",
        "probe_integrity_gate",
        "pure_probe_gate",
        "observer_label_gate",
        "collection_data_gate",
        "latch_dependence_gate",
        "candidate_freeze_gate",
        "final_rollout_gate",
        "final_development_gate",
    )
    checks = {
        "authority_design_freeze_only": authority["passed"] is True,
        "stage_topology_exact": topology["passed"] is True,
        "pure_design_gates_present": all(
            callable(getattr(contract, name, None)) for name in design_gate_names
        ),
        "v11_hashes": hashes["passed"] is True,
        "v11_fit_and_freeze_provenance": v11_fit_provenance["passed"] is True,
        "v11_terminal": terminal["passed"] is True,
        "v11_collection_evidence": collection["passed"] is True,
        "v11_seed_corpus": corpus["passed"] is True,
        "v11_pure_failure_coverage": pure_failure["passed"] is True,
        "write_paths_isolated": isolation["passed"] is True,
        "v11_design_receipt_idempotent": v11_receipt_before == v11_receipt_after,
        "future_execution_sources_deferred_exact": (
            contract.FUTURE_EXECUTION_SOURCES_REQUIRED == expected_future_sources
        ),
        **dict(occupancy),
    }
    passed = all(checks.values())
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.PROTOCOL_FREEZE_PASS_STATUS
            if passed
            else contract.PROTOCOL_FREEZE_FAIL_STATUS
        ),
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "revision": contract.REVISION,
        "checks": checks,
        "authority": copy.deepcopy(contract.AUTHORITY),
        "authority_gate": authority,
        "stage_order": list(contract.STAGE_IDS),
        "stage_topology_gate": topology,
        "fit_design": copy.deepcopy(contract.FIT),
        "recovery_weighting": copy.deepcopy(contract.RECOVERY_WEIGHTING),
        "coverage_weighting": copy.deepcopy(contract.COVERAGE_WEIGHTING),
        "latch_independence": copy.deepcopy(contract.LATCH_INDEPENDENCE),
        "probe_behavior": contract.PROBE_BEHAVIOR,
        "v11_authoritative_hashes": hashes,
        "v11_fit_and_freeze_provenance_audit": v11_fit_provenance,
        "v11_terminal_evidence": terminal,
        "v11_collection_recovery_audit": collection,
        "v11_seed_corpus_audit": corpus,
        "v11_pure_failure_coverage_audit": pure_failure,
        "write_path_isolation": isolation,
        "sources": sources,
        "inputs": inputs,
        "v11_design_receipt_snapshot_before": v11_receipt_before,
        "v11_design_receipt_snapshot_after": v11_receipt_after,
        "future_execution_sources_required": list(
            contract.FUTURE_EXECUTION_SOURCES_REQUIRED
        ),
        "execution_lock": None,
        "pipeline_claim": None,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "actor_fit_executions": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "next_stage": "WAIT_EXPLICIT_V12_ONE_SHOT_EXECUTION_AUTHORIZATION",
    }


def build_protocol_freeze(
    *,
    require_unoccupied: bool = True,
    occupancy_paths: Mapping[str, str | os.PathLike[str]] | None = None,
) -> dict[str, Any]:
    """Build the complete freeze payload without writing or running the plant."""

    occupancy_targets = {
        "protocol_freeze": contract.PROTOCOL_FREEZE_PATH,
        "execution_lock": contract.EXECUTION_LOCK_PATH,
        "design_audit": contract.DESIGN_AUDIT_RECEIPT_PATH,
        "run_root": contract.RUN_ROOT,
    }
    if occupancy_paths is not None:
        if set(occupancy_paths) != set(occupancy_targets):
            raise V12ProtocolFreezeError("occupancy override schema drifted")
        occupancy_targets = dict(occupancy_paths)
    occupancy = {
        "protocol_freeze_unoccupied": not os.path.lexists(
            _lexical_absolute(occupancy_targets["protocol_freeze"])
        ),
        "execution_lock_absent": not os.path.lexists(
            _lexical_absolute(occupancy_targets["execution_lock"])
        ),
        "design_audit_absent": not os.path.lexists(
            _lexical_absolute(occupancy_targets["design_audit"])
        ),
        "run_root_absent": not os.path.lexists(
            _lexical_absolute(occupancy_targets["run_root"])
        ),
    }
    payload = _assemble_protocol_freeze(occupancy)
    if require_unoccupied and payload["passed"] is not True:
        failed = [
            name for name, value in payload["checks"].items() if value is not True
        ]
        raise V12ProtocolFreezeError(f"V12 protocol freeze preflight failed: {failed}")
    return payload


def prepare_protocol_freeze(
    *,
    output_path: str | os.PathLike[str] | None = None,
    enforce_canonical_destination: bool = True,
) -> dict[str, Any]:
    destination = (
        _lexical_absolute(contract.PROTOCOL_FREEZE_PATH)
        if output_path is None
        else _lexical_absolute(output_path)
    )
    canonical = _lexical_absolute(contract.PROTOCOL_FREEZE_PATH)
    if enforce_canonical_destination and destination != canonical:
        raise V12ProtocolFreezeError(f"non-canonical protocol freeze: {destination}")
    snapshot = artifact_snapshot(destination)
    if snapshot["lexists"]:
        raise V12ProtocolFreezeError(
            f"protocol freeze exists/no-clobber: {destination}"
        )
    if _symlink_ancestors(destination):
        raise V12ProtocolFreezeError("protocol freeze path has a symlink ancestor")
    occupancy_override = None
    if not enforce_canonical_destination:
        occupancy_override = {
            "protocol_freeze": destination,
            "execution_lock": destination.with_name("execution_lock.json"),
            "design_audit": destination.with_name("design_audit.json"),
            "run_root": destination.with_name("run_root"),
        }
    payload = build_protocol_freeze(
        require_unoccupied=True,
        occupancy_paths=occupancy_override,
    )
    receipt_path = _write_json_exclusive_lexical(destination, payload)
    reloaded = forensic.strict_json_load(receipt_path)
    expected_bytes = forensic.canonical_json_bytes(payload)
    if reloaded != payload or receipt_path.read_bytes() != expected_bytes:
        raise V12ProtocolFreezeError("protocol freeze failed exact strict-JSON reload")
    return dict(payload)


def verify_protocol_freeze(
    *,
    input_path: str | os.PathLike[str] | None = None,
    enforce_canonical_destination: bool = True,
) -> dict[str, Any]:
    destination = _lexical_absolute(
        contract.PROTOCOL_FREEZE_PATH if input_path is None else input_path
    )
    canonical = _lexical_absolute(contract.PROTOCOL_FREEZE_PATH)
    if enforce_canonical_destination and destination != canonical:
        raise V12ProtocolFreezeError(f"non-canonical protocol freeze: {destination}")
    snapshot = artifact_snapshot(destination)
    if snapshot.get("kind") != "regular" or _symlink_ancestors(destination):
        raise V12ProtocolFreezeError("V12 protocol freeze is not a regular safe path")
    forbidden_now = {
        "execution_lock": contract.EXECUTION_LOCK_PATH,
        "design_audit": contract.DESIGN_AUDIT_RECEIPT_PATH,
        "run_root": contract.RUN_ROOT,
        "pipeline_claim": contract.PIPELINE_CLAIM_PATH,
    }
    occupied = [
        name
        for name, path in forbidden_now.items()
        if os.path.lexists(_lexical_absolute(path))
    ]
    if occupied:
        raise V12ProtocolFreezeError(
            f"V12 execution scope opened after design freeze: {occupied}"
        )
    expected = _assemble_protocol_freeze(
        {
            "protocol_freeze_unoccupied": True,
            "execution_lock_absent": True,
            "design_audit_absent": True,
            "run_root_absent": True,
        }
    )
    try:
        payload = forensic.strict_json_load(destination)
    except Exception as exc:
        raise V12ProtocolFreezeError("V12 protocol freeze is not strict JSON") from exc
    if payload != expected or destination.read_bytes() != forensic.canonical_json_bytes(
        expected
    ):
        raise V12ProtocolFreezeError("V12 protocol freeze drifted or scope opened")
    return dict(payload)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--prepare", action="store_true")
    action.add_argument("--verify", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    payload = prepare_protocol_freeze() if args.prepare else verify_protocol_freeze()
    print(f"{payload['status']}: {contract.PROTOCOL_FREEZE_PATH.as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "V12ProtocolFreezeError",
    "artifact_snapshot",
    "build_protocol_freeze",
    "main",
    "prepare_protocol_freeze",
    "verify_protocol_freeze",
]
