"""Fail-closed full-mean fit engine for the V10S safe-DAgger lineage.

The base corpus is reconstructed only from the six immutable V8R1P1 replay
traces.  Their deployed input is the V26-compatible ``v25_observation`` and
their label is the byte-audited frozen H0 mean.  Optional DAgger tranches are
accepted only through passing receipts that prove a same-state
counterfactual teacher query.

This module contains the sole actor-update primitive for V10S.  Every call
loads the original frozen H0 checkpoint afresh and performs the preregistered
400-epoch full-mean fit.  It never restores an earlier V10S candidate and it
never updates a critic or runs PPO.
"""

from __future__ import annotations

import hashlib
import math
import os
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path, PurePath
from typing import Any, Mapping, Sequence

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
BASELINE_ROOT = TRAJECTORY_ROOT / "baseline_MLP"
for _root in (REPO_ROOT, VALIDATION_ROOT, TRAJECTORY_ROOT, BASELINE_ROOT):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_forensic_rollout as forensic  # noqa: E402
import h0_primary_grf_split_v8r1p1_teacher_replay_contract as v8  # noqa: E402
import h0_primary_split_v10_coherent_teacher as coherent  # noqa: E402
import h0_primary_split_v10s_safe_dagger_contract as contract  # noqa: E402
import warm_start  # noqa: E402


class V10SFitError(RuntimeError):
    """Raised when fit evidence cannot be consumed without fallback."""


def _imitation_engine() -> Any:
    # The numerical fit stack imports torch/RLlib/OpenSim bootstrap helpers.
    # Keep it outside the evidence-only import path so corpus and contract
    # audits remain portable to lightweight macOS/Windows Python processes.
    import target_domain_imitation

    return target_domain_imitation


@dataclass(frozen=True)
class FitCorpus:
    """Contiguous, provenance-carrying actor imitation corpus."""

    observations: np.ndarray
    actions: np.ndarray
    reset_mask: np.ndarray
    actor_feature_names: np.ndarray
    case_ids: np.ndarray
    step_indices: np.ndarray
    tranche_ids: np.ndarray
    origins: np.ndarray
    source_records: Mapping[str, Any]
    audit: Mapping[str, Any]

    def dataset(self) -> dict[str, np.ndarray]:
        return {
            "observations": self.observations,
            "actions": self.actions,
            "actor_feature_names": self.actor_feature_names,
        }

    def arrays(self) -> dict[str, np.ndarray]:
        return {
            "observations": self.observations,
            "actions": self.actions,
            "reset_mask": self.reset_mask,
            "actor_feature_names": self.actor_feature_names,
            "case_ids": self.case_ids,
            "step_indices": self.step_indices,
            "tranche_ids": self.tranche_ids,
            "origins": self.origins,
            "training_indices": np.arange(len(self.observations), dtype=np.int64),
        }


def _resolve(relative_or_absolute: str | PurePath | Path) -> Path:
    path = Path(relative_or_absolute)
    return (path if path.is_absolute() else REPO_ROOT / path).resolve()


def _mapping(path: str | Path) -> dict[str, Any]:
    try:
        value = forensic.strict_json_load(path)
    except Exception as exc:
        raise V10SFitError(f"invalid strict JSON object: {_resolve(path)}") from exc
    if not isinstance(value, Mapping):
        raise V10SFitError(f"expected strict JSON object: {_resolve(path)}")
    return dict(value)


def _sequence(path: str | Path) -> list[Any]:
    try:
        value = forensic.strict_json_load(path)
    except Exception as exc:
        raise V10SFitError(f"invalid strict JSON array: {_resolve(path)}") from exc
    if not isinstance(value, list):
        raise V10SFitError(f"expected strict JSON array: {_resolve(path)}")
    return value


def _record(path: str | Path) -> dict[str, Any]:
    try:
        return forensic.artifact_record(_resolve(path), artifact_root=REPO_ROOT)
    except Exception as exc:
        raise V10SFitError(f"cannot record artifact: {_resolve(path)}") from exc


def _record_matches(value: Any, path: str | Path) -> bool:
    return isinstance(value, Mapping) and dict(value) == _record(path)


def _tree_record(path: str | Path) -> dict[str, Any]:
    root = _resolve(path)
    if not root.is_dir():
        raise V10SFitError(f"artifact tree is missing: {root}")
    files = sorted(item for item in root.rglob("*") if item.is_file())
    if not files:
        raise V10SFitError(f"artifact tree is empty: {root}")
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


def array_sha256(value: Any) -> str:
    array = np.ascontiguousarray(np.asarray(value))
    digest = hashlib.sha256()
    digest.update(str(array.dtype).encode("ascii"))
    digest.update(repr(tuple(int(item) for item in array.shape)).encode("ascii"))
    digest.update(array.tobytes(order="C"))
    return digest.hexdigest()


def _float32_vector(value: Any, *, width: int, label: str) -> np.ndarray:
    raw = np.asarray(value)
    if raw.shape != (width,):
        raise V10SFitError(f"{label} must have shape ({width},)")
    try:
        result = np.ascontiguousarray(raw, dtype=np.float32)
    except (TypeError, ValueError, OverflowError) as exc:
        raise V10SFitError(f"{label} is not a float32 vector") from exc
    if not np.all(np.isfinite(result)):
        raise V10SFitError(f"{label} contains a non-finite value")
    return result


def _same_float32(left: np.ndarray, right: np.ndarray) -> bool:
    return (
        left.dtype == np.dtype("float32")
        and right.dtype == np.dtype("float32")
        and left.shape == right.shape
        and left.tobytes(order="C") == right.tobytes(order="C")
    )


def _ensure_not_failed_v9(path: str | Path, *, label: str) -> Path:
    resolved = _resolve(path)
    forbidden = tuple(
        _resolve(item) for item in getattr(contract, "FORBIDDEN_FAILED_INPUT_ROOTS", ())
    )
    if not forbidden:
        forbidden = (
            _resolve(
                "validation/h0_primary_grf_split_adaptation_runs/"
                "2026-08-07_h0_primary_split_v9_v26_causal_residual"
            ),
            _resolve(
                "validation/h0_primary_grf_split_adaptation_runs/"
                "2026-08-07_h0_primary_split_v9r1_v26_causal_residual"
            ),
        )
    for root in forbidden:
        try:
            resolved.relative_to(root)
        except ValueError:
            continue
        raise V10SFitError(f"{label} resolves inside a failed V9 lineage")
    return resolved


def _teacher_block_indices() -> tuple[int, ...]:
    indices = tuple(
        int(item) for item in getattr(contract, "TEACHER_BLOCK_INDICES", range(10, 25))
    )
    if indices != coherent.TEACHER_BLOCK_INDICES:
        raise V10SFitError("V10S teacher block drifted from coherent V10")
    return indices


def _view_pair(
    row: Mapping[str, Any],
    *,
    student_key: str,
    teacher_key: str,
    label: str,
) -> tuple[np.ndarray, np.ndarray, tuple[int, ...]]:
    width = int(getattr(contract, "EXPECTED_ACTOR_FEATURES", 35))
    student = _float32_vector(
        row.get(student_key), width=width, label=f"{label} student"
    )
    teacher = _float32_vector(
        row.get(teacher_key), width=width, label=f"{label} teacher"
    )
    block = frozenset(_teacher_block_indices())
    changed = tuple(
        index
        for index in range(width)
        if student[index : index + 1].tobytes() != teacher[index : index + 1].tobytes()
    )
    escaped = [index for index in changed if index not in block]
    if escaped:
        raise V10SFitError(
            f"{label} teacher view changed columns outside 10:24: {escaped}"
        )
    unchanged = [index for index in range(width) if index not in block]
    if student[unchanged].tobytes() != teacher[unchanged].tobytes():
        raise V10SFitError(f"{label} invariant columns are not byte exact")
    return student, teacher, changed


def _base_teacher_root() -> Path:
    relative = getattr(
        contract,
        "BASE_CORPUS_ROOT",
        getattr(contract, "V8R1P1_TEACHER_ROOT", v8.RUN_ROOT),
    )
    return _ensure_not_failed_v9(relative, label="V8R1P1 teacher root")


def _adjudication_path() -> Path:
    relative = getattr(
        contract,
        "TEACHER_EVIDENCE_RECEIPT_PATH",
        getattr(
            contract,
            "V10_TIME_ALIGNMENT_ADJUDICATION_PATH",
            "validation/h0_primary_split_v10_time_alignment_adjudication_receipt.json",
        ),
    )
    return _ensure_not_failed_v9(relative, label="V10 adjudication")


def _validate_adjudication() -> dict[str, Any]:
    path = _adjudication_path()
    payload = _mapping(path)
    gate = contract.teacher_evidence_gate(payload)
    if gate.get("passed") is not True or not all(gate.get("checks", {}).values()):
        raise V10SFitError("V10 coherent-teacher adjudication is not a closed PASS")
    return gate


def _validate_v8_case(
    *, root: Path, case_id: str, ledger_row: Mapping[str, Any]
) -> tuple[Path, dict[str, Any]]:
    case_root = root / case_id
    trace_path = case_root / "trace.json"
    receipt_path = case_root / "receipt.json"
    gate_path = case_root / "gate.json"
    summary_path = case_root / "summary.json"
    receipt = _mapping(receipt_path)
    gate = _mapping(gate_path)
    summary = _mapping(summary_path)
    artifacts = receipt.get("artifacts")
    if (
        receipt.get("status") != v8.ROLLOUT_PASS_STATUS
        or receipt.get("passed") is not True
        or receipt.get("case_id") != case_id
        or receipt.get("candidate_created") is not False
        or receipt.get("actor_updates") != 0
        or receipt.get("critic_updates") != 0
        or receipt.get("ppo_updates") != 0
        or receipt.get("protected_trials_opened") != []
        or not isinstance(artifacts, Mapping)
        or not _record_matches(artifacts.get("trace"), trace_path)
        or not _record_matches(artifacts.get("gate"), gate_path)
        or not _record_matches(artifacts.get("summary"), summary_path)
    ):
        raise V10SFitError(f"V8R1P1 receipt drifted: {case_id}")
    checks = gate.get("checks")
    if (
        gate.get("status") != v8.ROLLOUT_PASS_STATUS
        or gate.get("passed") is not True
        or gate.get("case_id") != case_id
        or not isinstance(checks, Mapping)
        or not checks
        or not all(value is True for value in checks.values())
        or summary.get("steps") != v8.EXPECTED_STEPS
        or summary.get("candidate_created") is not False
    ):
        raise V10SFitError(f"V8R1P1 gate/summary drifted: {case_id}")
    if (
        ledger_row.get("case_id") != case_id
        or not _record_matches(ledger_row.get("receipt"), receipt_path)
        or not _record_matches(ledger_row.get("gate"), gate_path)
        or not _record_matches(ledger_row.get("summary"), summary_path)
    ):
        raise V10SFitError(f"V8R1P1 ledger closure drifted: {case_id}")
    return trace_path, {
        "receipt": _record(receipt_path),
        "gate": _record(gate_path),
        "summary": _record(summary_path),
        "trace": _record(trace_path),
    }


def load_frozen_v8_corpus() -> FitCorpus:
    """Load exactly 3,000 gated V8R1P1 rows with byte-level audits."""

    teacher_evidence_gate = _validate_adjudication()
    root = _base_teacher_root()
    ledger_path = root / "execution_ledger.json"
    ledger = _mapping(ledger_path)
    case_ids = tuple(v8.CASE_IDS)
    completed = ledger.get("completed_receipts")
    if (
        ledger.get("status") != v8.PROTOCOL_PASS_STATUS
        or ledger.get("passed") is not True
        or ledger.get("completed_cases") != list(case_ids)
        or ledger.get("expected_cases") != list(case_ids)
        or ledger.get("candidate_created") is not False
        or ledger.get("actor_updates") != 0
        or ledger.get("critic_updates") != 0
        or ledger.get("ppo_updates") != 0
        or ledger.get("protected_trials_opened") != []
        or not isinstance(completed, list)
        or len(completed) != len(case_ids)
    ):
        raise V10SFitError("V8R1P1 teacher ledger is not a terminal PASS")
    ledger_rows = {
        str(item.get("case_id")): item
        for item in completed
        if isinstance(item, Mapping)
    }
    if tuple(ledger_rows) != case_ids:
        raise V10SFitError("V8R1P1 completed-receipt order/schema drifted")

    observations: list[np.ndarray] = []
    actions: list[np.ndarray] = []
    reset_mask: list[bool] = []
    row_cases: list[str] = []
    step_indices: list[int] = []
    tranche_ids: list[str] = []
    origins: list[str] = []
    case_records: dict[str, Any] = {}
    changed_histogram = np.zeros(35, dtype=np.int64)
    exact_target_count = 0

    for case_id in case_ids:
        trace_path, records = _validate_v8_case(
            root=root, case_id=case_id, ledger_row=ledger_rows[case_id]
        )
        case_records[case_id] = records
        rows = _sequence(trace_path)
        if len(rows) != v8.EXPECTED_STEPS:
            raise V10SFitError(f"V8R1P1 trace length drifted: {case_id}")
        for offset, raw in enumerate(rows):
            step = offset + 1
            label = f"V8R1P1/{case_id}/{step}"
            if (
                not isinstance(raw, Mapping)
                or raw.get("step") != step
                or raw.get("case_id") != case_id
            ):
                raise V10SFitError(f"{label} row identity/order drifted")
            student, _teacher, changed = _view_pair(
                raw,
                student_key="v25_observation",
                teacher_key="baseline_teacher_observation",
                label=label,
            )
            target = _float32_vector(
                raw.get("queried_teacher_mean"), width=2, label=f"{label} target"
            )
            frozen = _float32_vector(
                raw.get("frozen_teacher_mean"), width=2, label=f"{label} frozen target"
            )
            if not _same_float32(target, frozen):
                raise V10SFitError(f"{label} queried/frozen teacher means differ")
            if np.any(np.abs(target) > 1.0):
                raise V10SFitError(f"{label} teacher mean exceeds action bounds")
            checks = raw.get("checks")
            if (
                not isinstance(checks, Mapping)
                or checks.get("teacher_mean_byte_exact") is not True
                or checks.get("invariant_columns_byte_exact") is not True
            ):
                raise V10SFitError(f"{label} frozen row checks drifted")
            for index in changed:
                changed_histogram[index] += 1
            exact_target_count += 1
            observations.append(student)
            actions.append(target)
            reset_mask.append(offset == 0)
            row_cases.append(case_id)
            step_indices.append(step)
            tranche_ids.append("v8r1p1_base")
            origins.append(f"v8r1p1:{case_id}:{step}")

    expected = len(case_ids) * v8.EXPECTED_STEPS
    result = _make_corpus(
        observations=observations,
        actions=actions,
        reset_mask=reset_mask,
        case_ids=row_cases,
        step_indices=step_indices,
        tranche_ids=tranche_ids,
        origins=origins,
        source_records={
            "v10_time_alignment_adjudication": _record(_adjudication_path()),
            "v10_teacher_evidence_gate": teacher_evidence_gate,
            "v8r1p1_execution_ledger": _record(ledger_path),
            "v8r1p1_cases": case_records,
        },
        audit={
            "base_sample_count": expected,
            "queried_equals_frozen_teacher_count": exact_target_count,
            "teacher_view_changes_only_10_24_count": expected,
            "changed_column_histogram": changed_histogram.tolist(),
            "failed_v9_rows_used": 0,
            "dagger_sample_count": 0,
            "same_state_dagger_sample_count": 0,
            "teacher_evidence_id": teacher_evidence_gate["teacher_evidence_id"],
            "teacher_evidence_passed": teacher_evidence_gate["passed"],
        },
    )
    if (
        result.observations.shape != (3000, 35)
        or result.actions.shape != (3000, 2)
        or int(np.count_nonzero(result.reset_mask)) != 6
        or exact_target_count != expected
        or not np.any(changed_histogram[list(_teacher_block_indices())] > 0)
        or np.any(
            changed_histogram[
                [index for index in range(35) if index not in _teacher_block_indices()]
            ]
            != 0
        )
    ):
        raise V10SFitError("V8R1P1 aggregate corpus contract drifted")
    return result


def _make_corpus(
    *,
    observations: Sequence[Any],
    actions: Sequence[Any],
    reset_mask: Sequence[bool],
    case_ids: Sequence[str],
    step_indices: Sequence[int],
    tranche_ids: Sequence[str],
    origins: Sequence[str],
    source_records: Mapping[str, Any],
    audit: Mapping[str, Any],
) -> FitCorpus:
    count = len(observations)
    arrays = {
        "observations": np.ascontiguousarray(observations, dtype=np.float32),
        "actions": np.ascontiguousarray(actions, dtype=np.float32),
        "reset_mask": np.ascontiguousarray(reset_mask, dtype=np.bool_),
        "case_ids": np.asarray(case_ids, dtype="U64"),
        "step_indices": np.asarray(step_indices, dtype=np.int64),
        "tranche_ids": np.asarray(tranche_ids, dtype="U64"),
        "origins": np.asarray(origins, dtype="U160"),
    }
    if (
        arrays["observations"].shape != (count, 35)
        or arrays["actions"].shape != (count, 2)
        or arrays["reset_mask"].shape != (count,)
        or arrays["case_ids"].shape != (count,)
        or arrays["step_indices"].shape != (count,)
        or arrays["tranche_ids"].shape != (count,)
        or arrays["origins"].shape != (count,)
        or not np.all(np.isfinite(arrays["observations"]))
        or not np.all(np.isfinite(arrays["actions"]))
        or np.any(np.abs(arrays["actions"]) > 1.0)
    ):
        raise V10SFitError("fit corpus arrays are malformed")
    feature_names = np.asarray(v8.EXPECTED_ACTOR_FEATURE_NAMES, dtype="U64")
    if (
        feature_names.shape != (35,)
        or tuple(feature_names.astype(str).tolist())
        != coherent.EXPECTED_ACTOR_FEATURE_NAMES
    ):
        raise V10SFitError("actor feature layout drifted")
    return FitCorpus(
        observations=arrays["observations"],
        actions=arrays["actions"],
        reset_mask=arrays["reset_mask"],
        actor_feature_names=feature_names,
        case_ids=arrays["case_ids"],
        step_indices=arrays["step_indices"],
        tranche_ids=arrays["tranche_ids"],
        origins=arrays["origins"],
        source_records=dict(source_records),
        audit=dict(audit),
    )


def _trace_record_from_receipt(
    receipt: Mapping[str, Any],
) -> tuple[Path, Mapping[str, Any]]:
    artifacts = receipt.get("artifacts")
    record: Any = None
    if isinstance(artifacts, Mapping):
        record = artifacts.get("trace")
    if record is None:
        record = receipt.get("trace")
    if not isinstance(record, Mapping) or not isinstance(record.get("path"), str):
        raise V10SFitError("DAgger receipt does not contain a trace record")
    raw_path = Path(str(record["path"]))
    if raw_path.is_absolute() or ".." in raw_path.parts:
        raise V10SFitError("DAgger trace record path is not repository-relative")
    path = _ensure_not_failed_v9(raw_path, label="DAgger trace")
    if not _record_matches(record, path):
        raise V10SFitError("DAgger trace hash/size record drifted")
    return path, record


def load_dagger_tranche(receipt_path: str | Path) -> FitCorpus:
    """Load one passing, same-state V10S DAgger case receipt."""

    path = _ensure_not_failed_v9(receipt_path, label="DAgger receipt")
    receipt = _mapping(path)
    sample_count = receipt.get("sample_count")
    same_state_count = receipt.get("same_state_teacher_label_count")
    if same_state_count is None:
        same_state_count = receipt.get("same_state_label_count")
    if (
        receipt.get("schema_version") != contract.SCHEMA_VERSION
        or receipt.get("passed") is not True
        or receipt.get("protocol_id") != contract.PROTOCOL_ID
        or sample_count != contract.COLLECTION_SAMPLES_PER_CASE
        or same_state_count != sample_count
        or receipt.get("candidate_selected_before_teacher_count") != sample_count
        or receipt.get("teacher_query_count") != sample_count
        or receipt.get("persisted_label_count") != sample_count
        or receipt.get("candidate_mean_query_count") != sample_count
        or receipt.get("served_action_teacher_dependency_count") != sample_count
        or receipt.get("actor_updates") != 0
        or receipt.get("critic_updates") != 0
        or receipt.get("ppo_updates") != 0
        or receipt.get("protected_trials_opened") != []
        or receipt.get("reserve_trials_opened") != []
        or receipt.get("retry_authorized") is not False
    ):
        raise V10SFitError("DAgger receipt is not a same-state zero-update PASS")
    expected_status = getattr(
        contract,
        "DAGGER_PASS_STATUS",
        getattr(contract, "COLLECTION_PASS_STATUS", None),
    )
    if expected_status is not None and receipt.get("status") != expected_status:
        raise V10SFitError("DAgger receipt status drifted")
    case_id = receipt.get("case_id")
    dagger_round = receipt.get("round_index")
    if (
        not isinstance(case_id, str)
        or not case_id
        or type(dagger_round) is not int
        or dagger_round not in {1, 2, 3}
        or receipt.get("dagger_round") != dagger_round
    ):
        raise V10SFitError("DAgger case/round identity is malformed")
    try:
        canonical_case = contract.canonical_collection_case(case_id, dagger_round)
    except ValueError as exc:
        raise V10SFitError("DAgger case/round is not canonical") from exc
    canonical_root = _resolve(canonical_case["destination"])
    canonical_receipt = _resolve(
        contract.stage_receipt_path(f"collect_r{dagger_round}__{case_id}")
    )
    stage_id = f"collect_r{dagger_round}__{case_id}"
    if (
        path != canonical_receipt
        or receipt.get("stage_id") != stage_id
        or receipt.get("candidate_fit_stage") != canonical_case["candidate_fit_stage"]
        or receipt.get("requested_alpha") != canonical_case["requested_alpha"]
        or not _record_matches(
            receipt.get("pipeline_claim"), contract.PIPELINE_CLAIM_PATH
        )
        or not _record_matches(
            receipt.get("worker_claim"), contract.worker_claim_path(stage_id)
        )
    ):
        raise V10SFitError("DAgger receipt path is not canonical for round/case")
    trace_path, _trace_record = _trace_record_from_receipt(receipt)
    if trace_path != canonical_root / "trace.json":
        raise V10SFitError("DAgger trace path is not canonical for round/case")
    artifacts = receipt.get("artifacts")
    if not isinstance(artifacts, Mapping):
        raise V10SFitError("DAgger receipt artifacts are malformed")
    required_artifacts = ("trace", "partial_summary", "summary", "gate")
    if any(not isinstance(artifacts.get(name), Mapping) for name in required_artifacts):
        raise V10SFitError("DAgger receipt lacks trace/summary/gate closure")
    summary_record = artifacts["summary"]
    partial_record = artifacts["partial_summary"]
    gate_record = artifacts["gate"]
    summary_path = canonical_root / "summary.json"
    partial_path = canonical_root / "partial_summary.json"
    gate_path = canonical_root / "gate.json"
    if (
        not _record_matches(summary_record, summary_path)
        or not _record_matches(partial_record, partial_path)
        or not _record_matches(gate_record, gate_path)
    ):
        raise V10SFitError("DAgger summary/gate artifact records drifted")
    summary = _mapping(summary_path)
    gate_path = _ensure_not_failed_v9(gate_record["path"], label="DAgger gate")
    gate = _mapping(gate_path)
    persisted = {
        "trace": dict(artifacts["trace"]),
        "partial_summary": dict(partial_record),
        "summary": dict(summary_record),
    }
    expected_gate = contract.collection_gate(summary, round_index=dagger_round)
    expected_gate["persisted_before_gate"] = persisted
    if (
        gate_path != canonical_root / "gate.json"
        or forensic.canonical_json_bytes(gate)
        != forensic.canonical_json_bytes(expected_gate)
        or gate.get("passed") is not True
        or summary.get("same_state_teacher_label_count") != sample_count
        or summary.get("candidate_selected_before_teacher_count") != sample_count
        or summary.get("served_action_teacher_dependency_count") != sample_count
    ):
        raise V10SFitError("DAgger gate is not an intact canonical PASS")

    rows = _sequence(trace_path)
    if len(rows) != sample_count:
        raise V10SFitError("DAgger trace/receipt sample counts disagree")
    observations: list[np.ndarray] = []
    actions: list[np.ndarray] = []
    reset_mask: list[bool] = []
    row_cases: list[str] = []
    step_indices: list[int] = []
    tranche_ids: list[str] = []
    origins: list[str] = []
    tranche_id = f"dagger_r{dagger_round}_{case_id}"
    changed_histogram = np.zeros(35, dtype=np.int64)
    for offset, raw in enumerate(rows):
        step = offset + 1
        label = f"V10S/{tranche_id}/{step}"
        if (
            not isinstance(raw, Mapping)
            or raw.get("step") != step
            or raw.get("case_id") != case_id
            or raw.get("candidate_selected_before_teacher") is not True
        ):
            raise V10SFitError(f"{label} same-state row identity drifted")
        student, _teacher, changed = _view_pair(
            raw,
            student_key="v25_observation",
            teacher_key="counterfactual_teacher_observation",
            label=label,
        )
        target = _float32_vector(
            raw.get("counterfactual_teacher_mean"), width=2, label=f"{label} target"
        )
        if np.any(np.abs(target) > 1.0):
            raise V10SFitError(f"{label} target exceeds action bounds")
        if raw.get("teacher_queried_on_same_state") is not True:
            raise V10SFitError(f"{label} was not labelled on the current state")
        for index in changed:
            changed_histogram[index] += 1
        observations.append(student)
        actions.append(target)
        reset_mask.append(offset == 0)
        row_cases.append(case_id)
        step_indices.append(step)
        tranche_ids.append(tranche_id)
        origins.append(f"dagger:r{dagger_round}:{case_id}:{step}")
    return _make_corpus(
        observations=observations,
        actions=actions,
        reset_mask=reset_mask,
        case_ids=row_cases,
        step_indices=step_indices,
        tranche_ids=tranche_ids,
        origins=origins,
        source_records={
            "receipt": _record(path),
            "trace": _record(trace_path),
            "gate": dict(gate_record),
        },
        audit={
            "base_sample_count": 0,
            "queried_equals_frozen_teacher_count": 0,
            "teacher_view_changes_only_10_24_count": sample_count,
            "changed_column_histogram": changed_histogram.tolist(),
            "failed_v9_rows_used": 0,
            "dagger_sample_count": sample_count,
            "same_state_dagger_sample_count": sample_count,
            "dagger_round": dagger_round,
            "case_id": case_id,
        },
    )


def _expected_counts(stage: str) -> tuple[int, int, int]:
    if stage not in tuple(contract.FIT_STAGES):
        raise V10SFitError(f"unknown V10S fit stage: {stage!r}")
    index = tuple(contract.FIT_STAGES).index(stage)
    expected = contract.expected_fit_counts(stage)
    samples = expected["sample_count"]
    resets = expected["reset_row_count"]
    receipts = index * int(contract.COLLECTION_CASE_COUNT_PER_ROUND)
    return int(samples), int(resets), int(receipts)


def load_fit_corpus(
    dagger_receipt_paths: Sequence[str | Path] = (), *, stage: str = "p0"
) -> FitCorpus:
    """Build a cumulative V10S corpus for one preregistered fit stage."""

    expected_samples, expected_resets, expected_receipts = _expected_counts(stage)
    canonical_receipts = tuple(
        _ensure_not_failed_v9(item, label="DAgger receipt")
        for item in dagger_receipt_paths
    )
    if len(canonical_receipts) != expected_receipts or len(
        set(canonical_receipts)
    ) != len(canonical_receipts):
        raise V10SFitError(
            f"{stage} requires exactly {expected_receipts} distinct DAgger receipts"
        )
    base = load_frozen_v8_corpus()
    tranches = [load_dagger_tranche(path) for path in canonical_receipts]
    identities = [
        (int(item.audit["dagger_round"]), str(item.audit["case_id"]))
        for item in tranches
    ]
    if len(set(identities)) != len(identities):
        raise V10SFitError("duplicate DAgger round/case tranche")
    expected_identities = [
        (round_id, case_id)
        for round_id in contract.FIT_COMPLETED_ROUNDS[stage]
        for case_id in contract.COLLECTION_CASE_IDS
    ]
    if identities != expected_identities:
        raise V10SFitError(
            f"{stage} DAgger tranche order/identity {identities} != "
            f"{expected_identities}"
        )
    pieces = [base, *tranches]
    combined = _make_corpus(
        observations=np.concatenate([item.observations for item in pieces], axis=0),
        actions=np.concatenate([item.actions for item in pieces], axis=0),
        reset_mask=np.concatenate([item.reset_mask for item in pieces], axis=0),
        case_ids=np.concatenate([item.case_ids for item in pieces], axis=0),
        step_indices=np.concatenate([item.step_indices for item in pieces], axis=0),
        tranche_ids=np.concatenate([item.tranche_ids for item in pieces], axis=0),
        origins=np.concatenate([item.origins for item in pieces], axis=0),
        source_records={
            **base.source_records,
            "dagger_tranches": [dict(item.source_records) for item in tranches],
        },
        audit={
            "base_sample_count": len(base.observations),
            "queried_equals_frozen_teacher_count": base.audit[
                "queried_equals_frozen_teacher_count"
            ],
            "teacher_view_changes_only_10_24_count": sum(
                int(item.audit["teacher_view_changes_only_10_24_count"])
                for item in pieces
            ),
            "failed_v9_rows_used": 0,
            "dagger_sample_count": sum(len(item.observations) for item in tranches),
            "same_state_dagger_sample_count": sum(
                int(item.audit["same_state_dagger_sample_count"]) for item in tranches
            ),
            "dagger_receipt_count": len(tranches),
            "dagger_identities": [list(item) for item in identities],
            "teacher_evidence_id": base.audit["teacher_evidence_id"],
            "teacher_evidence_passed": base.audit["teacher_evidence_passed"],
        },
    )
    if (
        len(combined.observations) != expected_samples
        or int(np.count_nonzero(combined.reset_mask)) != expected_resets
        or combined.audit["failed_v9_rows_used"] != 0
        or combined.audit["same_state_dagger_sample_count"]
        != combined.audit["dagger_sample_count"]
    ):
        raise V10SFitError(f"{stage} cumulative corpus contract drifted")
    return combined


def _write_npz_exclusive(path: str | Path, arrays: Mapping[str, np.ndarray]) -> Path:
    destination = _resolve(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    if os.path.lexists(destination):
        raise V10SFitError(f"refusing to clobber: {destination}")
    descriptor, temporary_raw = tempfile.mkstemp(
        prefix=f".{destination.name}.", suffix=".tmp", dir=destination.parent
    )
    temporary = Path(temporary_raw)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            np.savez(stream, **arrays)
            stream.flush()
            os.fsync(stream.fileno())
        flags = os.O_CREAT | os.O_EXCL | os.O_WRONLY | getattr(os, "O_BINARY", 0)
        try:
            claim = os.open(str(destination), flags, 0o600)
        except FileExistsError as exc:
            raise V10SFitError(f"refusing to clobber: {destination}") from exc
        else:
            os.close(claim)
        os.replace(temporary, destination)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
    return destination


def prediction_metrics(predictions: Any, targets: Any) -> dict[str, float]:
    left = np.asarray(predictions, dtype=np.float64)
    right = np.asarray(targets, dtype=np.float64)
    if left.shape != right.shape or left.ndim != 2 or left.shape[1:] != (2,):
        raise V10SFitError("prediction/target shapes do not match")
    error = left - right
    if not np.all(np.isfinite(error)):
        raise V10SFitError("offline prediction error is non-finite")
    return {
        "rmse": float(np.sqrt(np.mean(np.square(error)))),
        "max_abs_error": float(np.max(np.abs(error), initial=0.0)),
        "knee_rmse": float(np.sqrt(np.mean(np.square(error[:, 0])))),
        "ankle_rmse": float(np.sqrt(np.mean(np.square(error[:, 1])))),
        "knee_max_abs_error": float(np.max(np.abs(error[:, 0]), initial=0.0)),
        "ankle_max_abs_error": float(np.max(np.abs(error[:, 1]), initial=0.0)),
    }


def full_mean_update_audit(
    source_state: Mapping[str, Any], candidate_state: Mapping[str, Any]
) -> dict[str, Any]:
    actor_keys = {
        "pi_encoder.0.weight",
        "pi_encoder.0.bias",
        "pi_encoder.2.weight",
        "pi_encoder.2.bias",
        "pi.0.0.weight",
        "pi.0.0.bias",
        "pi.0.2.weight",
        "pi.0.2.bias",
        "pi.1.weight",
        "pi.1.bias",
    }
    if not actor_keys.issubset(source_state) or not actor_keys.issubset(
        candidate_state
    ):
        raise V10SFitError("actor state schema drifted")
    changed: list[str] = []
    arrays: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    all_finite = True
    for key in sorted(actor_keys):
        source = np.ascontiguousarray(warm_start._as_numpy(source_state[key]))
        candidate = np.ascontiguousarray(warm_start._as_numpy(candidate_state[key]))
        if source.dtype != candidate.dtype or source.shape != candidate.shape:
            raise V10SFitError(f"actor tensor dtype/shape drifted: {key}")
        arrays[key] = (source, candidate)
        all_finite = all_finite and bool(np.all(np.isfinite(candidate)))
        if source.tobytes() != candidate.tobytes():
            changed.append(key)
    output_source_w, output_candidate_w = arrays["pi.1.weight"]
    output_source_b, output_candidate_b = arrays["pi.1.bias"]
    logstd_exact = bool(
        output_source_w[2:].tobytes() == output_candidate_w[2:].tobytes()
        and output_source_b[2:].tobytes() == output_candidate_b[2:].tobytes()
    )
    mean_output_changed = bool(
        output_source_w[:2].tobytes() != output_candidate_w[:2].tobytes()
        or output_source_b[:2].tobytes() != output_candidate_b[:2].tobytes()
    )
    hidden_changed = any(key not in {"pi.1.weight", "pi.1.bias"} for key in changed)
    aliases_exact = all(
        arrays[left][1].tobytes() == arrays[right][1].tobytes()
        for left, right in (
            ("pi_encoder.0.weight", "pi.0.0.weight"),
            ("pi_encoder.0.bias", "pi.0.0.bias"),
            ("pi_encoder.2.weight", "pi.0.2.weight"),
            ("pi_encoder.2.bias", "pi.0.2.bias"),
        )
    )
    clock_columns_zero = all(
        np.count_nonzero(arrays[key][1][:, :2]) == 0
        for key in ("pi_encoder.0.weight", "pi.0.0.weight")
    )
    non_actor = warm_start.compare_non_actor_states(source_state, candidate_state)
    non_actor_ok = non_actor.get("exact") is True or (
        non_actor.get("keys") == []
        and non_actor.get("missing_keys") == []
        and non_actor.get("unexpected_keys") == []
    )
    return {
        "trainable_scope": "full_mean_network",
        "changed_actor_keys": changed,
        "actor_digest_changed": warm_start.actor_state_digest(source_state)
        != warm_start.actor_state_digest(candidate_state),
        "hidden_mean_network_changed": hidden_changed,
        "mean_output_changed": mean_output_changed,
        "logstd_parameter_rows_bit_exact": logstd_exact,
        "encoder_aliases_bit_exact": aliases_exact,
        "disabled_clock_columns_zero": clock_columns_zero,
        "all_candidate_actor_tensors_finite": all_finite,
        "non_actor_exact_or_absent": bool(non_actor_ok),
        "non_actor_comparison": non_actor,
        "changes_confined_to_full_mean_network": bool(
            changed
            and hidden_changed
            and mean_output_changed
            and logstd_exact
            and aliases_exact
            and clock_columns_zero
            and non_actor_ok
        ),
    }


def _module_logits(module_path: str | Path, observations: np.ndarray) -> np.ndarray:
    import torch
    from ray.rllib.core.rl_module.rl_module import RLModule

    module = RLModule.from_checkpoint(_resolve(module_path))
    module.pi.eval()
    with torch.no_grad():
        logits = module.pi(torch.as_tensor(observations, dtype=torch.float32))
    result = np.ascontiguousarray(logits.detach().cpu().numpy(), dtype=np.float32)
    if result.shape != (len(observations), 4) or not np.all(np.isfinite(result)):
        raise V10SFitError("module logits are malformed")
    return result


def _source_h0_path() -> Path:
    relative = getattr(
        contract,
        "SOURCE_H0_MODULE_PATH",
        "validation/critic_warmup/"
        "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/rl_module_last",
    )
    return _ensure_not_failed_v9(relative, label="source H0")


def _fit_root(stage: str) -> Path:
    roots = getattr(contract, "FIT_ROOTS")
    return _resolve(roots[stage])


def _module_path(stage: str) -> Path:
    paths = getattr(contract, "MODULE_PATHS")
    return _resolve(paths[stage])


def _fit_receipt_path(stage: str) -> Path:
    paths = getattr(contract, "FIT_RECEIPT_PATHS")
    return _resolve(paths[stage])


def _validate_claim(
    path: str | Path,
    *,
    stage: str,
    worker: bool,
    enforce_canonical: bool,
) -> dict[str, Any]:
    resolved = _resolve(path)
    payload = _mapping(resolved)
    expected_status = (
        contract.WORKER_CLAIM_STATUS if worker else contract.PIPELINE_CLAIM_STATUS
    )
    if (
        payload.get("schema_version") != contract.SCHEMA_VERSION
        or payload.get("status") != expected_status
        or payload.get("protocol_id") != contract.PROTOCOL_ID
        or payload.get("pipeline_id") != contract.PIPELINE_ID
        or payload.get("retry_authorized") is not False
        or payload.get("critic_updates") != 0
        or payload.get("ppo_updates") != 0
        or payload.get("protected_trials_opened") != []
        or payload.get("reserve_trials_opened") != []
    ):
        raise V10SFitError("pipeline/worker claim authority drifted")
    if worker and payload.get("stage_id") != f"fit_{stage}":
        raise V10SFitError("worker claim does not authorize this fit stage")
    if enforce_canonical:
        expected_path = _resolve(
            contract.worker_claim_path(f"fit_{stage}")
            if worker
            else contract.PIPELINE_CLAIM_PATH
        )
        if resolved != expected_path:
            raise V10SFitError("fit claim path is not canonical")
        if worker:
            if not _record_matches(
                payload.get("pipeline_claim"), contract.PIPELINE_CLAIM_PATH
            ):
                raise V10SFitError("worker claim pipeline record drifted")
        elif not _record_matches(payload.get("execution_lock"), contract.LOCK_PATH):
            raise V10SFitError("pipeline claim execution-lock record drifted")
    return payload


def _fit_spec() -> dict[str, Any]:
    observed = dict(contract.FIT)
    expected = {
        "seed": 123,
        "epochs": 400,
        "batch_size": 128,
        "learning_rate": 5.0e-5,
        "validation_fraction": 0.0,
        "patience": 0,
        "clip_weight": 1.0,
        "logstd_weight": 0.0,
        "anchor_weight": 1.0e-2,
        "selection_mode": "fixed_final_epoch",
        "trainable_first_layer_features": None,
        "freeze_logstd_head": True,
    }
    if observed != expected:
        raise V10SFitError("V10S full-mean fit specification drifted")
    return observed


def _canonical_fit_destination(
    stage: str, output_dir: str | Path, *, enforce: bool
) -> Path:
    destination = _resolve(output_dir)
    expected = _fit_root(stage)
    if enforce and destination != expected:
        raise V10SFitError(f"non-canonical {stage} destination: {destination}")
    if os.path.lexists(destination):
        raise V10SFitError(f"fit stage already exists/no-clobber: {destination}")
    module_path = destination / "rl_module_target_adapted"
    if enforce and module_path.resolve() != _module_path(stage):
        raise V10SFitError("contract module path is not the upstream fit path")
    receipt_path = destination / "receipt.json"
    if enforce and receipt_path.resolve() != _fit_receipt_path(stage):
        raise V10SFitError("contract fit receipt path drifted")
    return destination


def run_fit_stage(
    *,
    stage: str,
    output_dir: str | Path,
    dagger_receipt_paths: Sequence[str | Path] = (),
    pipeline_claim_path: str | Path,
    worker_claim_path: str | Path,
    enforce_canonical_destination: bool = True,
) -> dict[str, Any]:
    """Execute one no-clobber full-mean fit from a fresh frozen H0.

    The caller must claim pipeline/stage authority before invoking this
    function.  A scientific gate failure is returned and persisted; malformed
    evidence or an occupied destination raises ``V10SFitError``.
    """

    _expected_counts(stage)
    pipeline_claim = _validate_claim(
        pipeline_claim_path,
        stage=stage,
        worker=False,
        enforce_canonical=enforce_canonical_destination,
    )
    worker_claim = _validate_claim(
        worker_claim_path,
        stage=stage,
        worker=True,
        enforce_canonical=enforce_canonical_destination,
    )
    if enforce_canonical_destination and (
        not isinstance(pipeline_claim.get("execution_token_sha256"), str)
        or len(pipeline_claim["execution_token_sha256"]) != 64
        or worker_claim.get("execution_token_sha256")
        != pipeline_claim["execution_token_sha256"]
    ):
        raise V10SFitError("pipeline/worker execution-token binding drifted")
    corpus = load_fit_corpus(dagger_receipt_paths, stage=stage)
    fit = _fit_spec()
    destination = _canonical_fit_destination(
        stage, output_dir, enforce=enforce_canonical_destination
    )
    source_h0 = _source_h0_path()
    source_before = _tree_record(source_h0)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.mkdir(exist_ok=False)
    corpus_path = _write_npz_exclusive(destination / "corpus.npz", corpus.arrays())

    report = _imitation_engine().adapt_actor(
        source_h0,
        corpus.dataset(),
        destination,
        seed=fit["seed"],
        epochs=fit["epochs"],
        batch_size=fit["batch_size"],
        learning_rate=fit["learning_rate"],
        validation_fraction=fit["validation_fraction"],
        patience=fit["patience"],
        clip_weight=fit["clip_weight"],
        logstd_weight=fit["logstd_weight"],
        anchor_weight=fit["anchor_weight"],
        freeze_logstd_head=True,
        trainable_first_layer_features=None,
        selection_mode="fixed_final_epoch",
    )
    module_path = destination / "rl_module_target_adapted"
    if source_before != _tree_record(source_h0):
        raise V10SFitError("frozen source H0 changed during adaptation")
    source_state = warm_start.load_module_state(source_h0)
    candidate_state = warm_start.load_module_state(module_path)
    update_audit = full_mean_update_audit(source_state, candidate_state)
    source_logits = _module_logits(source_h0, corpus.observations)
    candidate_logits = _module_logits(module_path, corpus.observations)
    metrics = prediction_metrics(candidate_logits[:, :2], corpus.actions)
    reset_error = np.abs(
        candidate_logits[corpus.reset_mask, :2].astype(np.float64)
        - corpus.actions[corpus.reset_mask].astype(np.float64)
    )
    metrics["reset_max_abs_error"] = float(np.max(reset_error, initial=0.0))
    logstd_outputs_exact = bool(
        source_logits[:, 2:].tobytes() == candidate_logits[:, 2:].tobytes()
    )
    report_hyperparameters = report.get("hyperparameters")
    report_checks = {
        "fresh_source_h0": _resolve(report.get("source_checkpoint", "")) == source_h0,
        "fixed_final_epoch": report.get("selection_mode") == "fixed_final_epoch",
        "epochs_exact": report.get("epochs_run") == fit["epochs"]
        and report.get("best_epoch") == fit["epochs"],
        "all_rows_used": report.get("training_samples") == len(corpus.observations)
        and report.get("validation_samples") == 0,
        "full_mean_scope": isinstance(report_hyperparameters, Mapping)
        and report_hyperparameters.get("trainable_first_layer_features") is None,
        "fit_spec_exact": isinstance(report_hyperparameters, Mapping)
        and report_hyperparameters.get("seed") == fit["seed"]
        and report_hyperparameters.get("batch_size") == fit["batch_size"]
        and report_hyperparameters.get("learning_rate") == fit["learning_rate"]
        and report_hyperparameters.get("anchor_weight") == fit["anchor_weight"]
        and report_hyperparameters.get("freeze_logstd_head") is True,
        "mean_only_update": update_audit["changes_confined_to_full_mean_network"],
        "disabled_clock_columns_zero": update_audit[
            "disabled_clock_columns_zero"
        ],
        "logstd_parameters_exact": update_audit["logstd_parameter_rows_bit_exact"],
        "logstd_outputs_exact": logstd_outputs_exact,
        "non_actor_exact_or_absent": update_audit["non_actor_exact_or_absent"],
        "save_reload_exact": isinstance(report.get("save_reload"), Mapping)
        and report["save_reload"].get("exact") is True,
        "all_finite": bool(
            np.all(np.isfinite(candidate_logits))
            and update_audit["all_candidate_actor_tensors_finite"]
        ),
        "failed_v9_rows_excluded": corpus.audit["failed_v9_rows_used"] == 0,
        "dagger_labels_same_state": corpus.audit["same_state_dagger_sample_count"]
        == corpus.audit["dagger_sample_count"],
    }
    sample_identities = list(
        zip(
            corpus.tranche_ids.astype(str).tolist(),
            corpus.case_ids.astype(str).tolist(),
            corpus.step_indices.tolist(),
        )
    )
    duplicate_sample_count = len(sample_identities) - len(set(sample_identities))
    expected_counts = contract.expected_fit_counts(stage)
    source_h0_byte_exact = source_before == _tree_record(source_h0)
    critic_byte_exact = bool(update_audit["non_actor_exact_or_absent"])
    logstd_byte_exact = bool(
        update_audit["logstd_parameter_rows_bit_exact"] and logstd_outputs_exact
    )
    all_finite = bool(
        all(report_checks.values())
        and np.all(np.isfinite(corpus.observations))
        and np.all(np.isfinite(corpus.actions))
        and all(math.isfinite(float(value)) for value in metrics.values())
    )
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_PRIMARY_SPLIT_V10S_FULL_MEAN_FIT_COMPLETE_UNGATED",
        "protocol_id": contract.PROTOCOL_ID,
        "fit_stage": stage,
        "fit": fit,
        "source_h0_id": contract.SOURCE_H0_ID,
        "initial_checkpoint_id": contract.SOURCE_H0_ID,
        "continued_from_previous_candidate": False,
        "trainable_scope": contract.TRAINABLE_SCOPE,
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "teacher_evidence_id": corpus.audit["teacher_evidence_id"],
        "teacher_evidence_passed": corpus.audit["teacher_evidence_passed"],
        "base_corpus_case_ids": list(contract.FINAL_CASE_IDS),
        "completed_collection_rounds": list(
            expected_counts["completed_collection_rounds"]
        ),
        "sample_count": len(corpus.observations),
        "reset_row_count": int(np.count_nonzero(corpus.reset_mask)),
        "base_sample_count": int(corpus.audit["base_sample_count"]),
        "dagger_sample_count": int(corpus.audit["dagger_sample_count"]),
        "dagger_receipt_count": len(dagger_receipt_paths),
        "duplicate_sample_count": duplicate_sample_count,
        "metrics": metrics,
        "all_finite": all_finite,
        "source_h0_byte_exact": source_h0_byte_exact,
        "critic_byte_exact": critic_byte_exact,
        "logstd_byte_exact": logstd_byte_exact,
        "report_checks": report_checks,
        "corpus_audit": dict(corpus.audit),
        "corpus_observations_sha256": array_sha256(corpus.observations),
        "corpus_actions_sha256": array_sha256(corpus.actions),
        "corpus_reset_mask_sha256": array_sha256(corpus.reset_mask),
        "corpus": _record(corpus_path),
        "corpus_sources": dict(corpus.source_records),
        "source_h0": source_before,
        "module": _tree_record(module_path),
        "full_mean_update_audit": update_audit,
        "adaptation_report": _record(destination / "adaptation_report.json"),
        "adaptation_history": _record(destination / "adaptation_history.json"),
        "pipeline_claim": _record(pipeline_claim_path),
        "worker_claim": _record(worker_claim_path),
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    summary_path = forensic.write_json_exclusive(destination / "summary.json", summary)
    gate = contract.fit_gate(summary, stage=stage)
    gate_path = forensic.write_json_exclusive(destination / "gate.json", gate)
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": gate["status"],
        "passed": gate["passed"],
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": f"fit_{stage}",
        "fit_stage": stage,
        "candidate_created": True,
        "candidate_scope": "FIT_ONLY_UNFROZEN",
        "retry_authorized": False,
        "fit": fit,
        "corpus": _record(corpus_path),
        "summary": _record(summary_path),
        "gate": _record(gate_path),
        "adaptation_report": _record(destination / "adaptation_report.json"),
        "adaptation_history": _record(destination / "adaptation_history.json"),
        "module": _tree_record(module_path),
        "source_h0": source_before,
        "pipeline_claim": _record(pipeline_claim_path),
        "worker_claim": _record(worker_claim_path),
        "dagger_receipts": [_record(item) for item in dagger_receipt_paths],
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    receipt_path = forensic.write_json_exclusive(destination / "receipt.json", receipt)
    receipt["receipt"] = _record(receipt_path)
    # Claims are read before any actor update; retain locals so static/runtime
    # audits can prove they were actually parsed rather than merely hashed.
    if not pipeline_claim or not worker_claim:
        raise V10SFitError("empty execution claim after completed fit")
    return receipt


__all__ = [
    "FitCorpus",
    "V10SFitError",
    "array_sha256",
    "full_mean_update_audit",
    "load_dagger_tranche",
    "load_fit_corpus",
    "load_frozen_v8_corpus",
    "prediction_metrics",
    "run_fit_stage",
]
