"""One-shot V12R9 additive recovery, fit and pure-development runner.

R8 remains immutable: plus labels are referenced directly, its valid minus
prefix is labelled offline, and only the four unstarted observer cases touch
an environment.  The remainder is one actor fit and six pure developments.
"""

from __future__ import annotations

import argparse
import copy
import json
import os
import sys
from dataclasses import dataclass
from pathlib import Path, PurePath
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
    LOCAL_VALIDATION / "v12r7",
    LOCAL_VALIDATION / "v12r8",
    REVISION_ROOT,
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import freeze_h0_v12r9_recovery as freezer  # noqa: E402
import h0_forensic_rollout as forensic  # noqa: E402
import h0_v12r6_physical_development as physical  # noqa: E402
import h0_v12r9_recovery_contract as contract  # noqa: E402
import h0_v12r9_recovery_fitter as fitter  # noqa: E402
import h0_v12r9_recovery_probe as probe  # noqa: E402


class V12R9ExecutionError(RuntimeError):
    """Raised after the one-shot R9 lineage has failed terminally."""


ACTIVITY_FIELDS = (
    "collection_rounds_completed",
    "environment_reset_calls",
    "environment_step_calls",
    "historical_prefix_environment_reset_calls",
    "historical_prefix_environment_step_calls",
    "new_collection_environment_reset_calls",
    "new_collection_environment_step_calls",
    "development_environment_reset_calls",
    "development_environment_step_calls",
    "raw_sensor_sample_count",
    "offline_teacher_label_calls",
    "imported_observer_label_rows",
    "adjudicated_prefix_label_rows",
    "new_collection_label_rows",
    "actor_fit_stage_calls",
    "actor_updates",
    "adamw_epochs_completed",
    "lbfgs_closure_calls",
    "development_rollouts_completed",
    "critic_updates",
    "ppo_updates",
)
EXPECTED_CANDIDATE_FILES = {
    "actor_feature_manifest.json",
    "candidate_build_manifest.json",
    "class_and_ctor_args.pkl",
    "metadata.json",
    "module_state.pkl",
}
TERMINAL_LEDGER_FIELDS = {
    "schema_version",
    "status",
    "passed",
    "terminal",
    "protocol_id",
    "pipeline_id",
    "candidate_selection_rule",
    "candidate_id",
    "candidate_module",
    "candidate_module_observation",
    "stage_order",
    "completed_stages",
    "completed_stage_count",
    "attempted_stage",
    "activity_totals",
    "collection_round_count",
    "actor_fit_count",
    "development_count",
    "protocol_freeze",
    "execution_lock",
    "pipeline_claim",
    "candidate_freeze",
    "final_development_receipt",
    "qualification_executed",
    "runtime_promoted",
    "checkpoint_zero_created",
    "positive_morphology_enabled",
    "q3_paths_opened",
    "actor_updates",
    "critic_updates",
    "ppo_updates",
    "environment_reset_calls",
    "environment_step_calls",
    "historical_prefix_environment_reset_calls",
    "historical_prefix_environment_step_calls",
    "new_collection_environment_reset_calls",
    "new_collection_environment_step_calls",
    "development_environment_reset_calls",
    "development_environment_step_calls",
    "retry_authorized",
    "resume_authorized",
    "alpha_sweep_authorized",
    "error",
    "next_stage",
}
R8_ADJUDICATION_STAGE_FIELDS = {
    "schema_version",
    "status",
    "passed",
    "protocol_id",
    "pipeline_id",
    "stage_id",
    "case_id",
    "adjudication",
    "r8_terminal_ledger",
    "r8_minus_trace",
    "r8_minus_replay",
    "r8_minus_summary",
    "r8_minus_gate",
    "r8_minus_receipt",
    "candidate_module",
    "pipeline_claim",
    "worker_claim",
    "offline_label_authorized",
    "r8_probe_rerun",
    "r8_artifacts_modified",
    "environment_reset_calls",
    "environment_step_calls",
    "actor_updates",
    "critic_updates",
    "ppo_updates",
}
PLUS_IMPORT_STAGE_FIELDS = {
    "schema_version",
    "status",
    "passed",
    "protocol_id",
    "pipeline_id",
    "stage_id",
    "case_id",
    "label_receipt",
    "labels",
    "label_summary",
    "label_gate",
    "source_stage_receipt",
    "pipeline_claim",
    "worker_claim",
    "labelled_row_count",
    "same_state_teacher_label_count",
    "teacher_query_count",
    "direct_immutable_reference",
    "labels_copied",
    "semantic_and_byte_exact_closed",
    "r8_artifacts_modified",
    "environment_reset_calls",
    "environment_step_calls",
    "actor_updates",
    "critic_updates",
    "ppo_updates",
}
MINUS_LABEL_STAGE_FIELDS = {
    "schema_version",
    "status",
    "passed",
    "protocol_id",
    "pipeline_id",
    "stage_id",
    "case_id",
    "adjudication_stage_receipt",
    "source_replay",
    "label_receipt",
    "labels",
    "label_summary",
    "label_gate",
    "pipeline_claim",
    "worker_claim",
    "labelled_row_count",
    "same_state_teacher_label_count",
    "teacher_query_count",
    "teacher_h0_id",
    "teacher_h0",
    "coverage_reference_corpus",
    "imported_r8_prefix",
    "environment_reset_calls",
    "environment_step_calls",
    "actor_updates",
    "critic_updates",
    "ppo_updates",
}
COLLECTION_LABEL_STAGE_FIELDS = {
    "schema_version",
    "status",
    "passed",
    "protocol_id",
    "pipeline_id",
    "stage_id",
    "case_id",
    "candidate_tree_sha256",
    "probe_step_count",
    "labelled_row_count",
    "same_state_teacher_label_count",
    "teacher_query_count",
    "teacher_h0_id",
    "teacher_h0",
    "coverage_reference_corpus",
    "probe_receipt",
    "probe_gate",
    "probe_replay",
    "labels",
    "label_summary",
    "label_gate",
    "label_receipt",
    "pipeline_claim",
    "worker_claim",
    "single_collection_round",
    "actor_updates",
    "critic_updates",
    "ppo_updates",
}
CANDIDATE_FREEZE_FIELDS = {
    "schema_version",
    "status",
    "passed",
    "protocol_id",
    "pipeline_id",
    "stage_id",
    "candidate_selection_rule",
    "candidate_id",
    "candidate_module",
    "fit_receipt",
    "actor_feature_manifest",
    "actor_digest",
    "candidate_build_manifest",
    "semantic_audit",
    "pipeline_claim",
    "worker_claim",
    "candidate_frozen",
    "fit_gate_passed",
    "standard_actor",
    "warm_start_target_512_compatible",
    "actor_fit_count",
    "runtime_promoted",
    "qualification_executed",
    "q3_paths_opened",
    "actor_updates",
    "critic_updates",
    "ppo_updates",
}


@dataclass(frozen=True)
class RunnerAdapters:
    """Small injectable boundary used only by source/unit orchestration tests."""

    run_adjudication: Callable[..., Mapping[str, Any]] = (
        probe.publish_r8_minus_adjudication
    )
    import_plus_labels: Callable[..., Mapping[str, Any]] = (
        probe.adjudicator.verify_r8_plus_label_import
    )
    run_imported_minus_labels: Callable[..., Mapping[str, Any]] = (
        probe.label_adjudicated_r8_minus
    )
    run_probe: Callable[..., Mapping[str, Any]] = probe.run_recovery_probe
    run_labels: Callable[..., Mapping[str, Any]] = probe.label_recovery_probe
    run_fit: Callable[..., Mapping[str, Any]] = fitter.run_fit_stage
    verify_fit: Callable[[], Mapping[str, Any]] = fitter.verify_fit_stage
    run_physical: Callable[..., Mapping[str, Any]] = physical.run_case


def _path(value: str | PurePath | Path) -> Path:
    if isinstance(value, Path) and value.is_absolute():
        target = Path(os.path.abspath(value))
        try:
            relative = target.relative_to(REPO_ROOT)
        except ValueError as exc:
            raise V12R9ExecutionError(f"path escaped repository: {target}") from exc
        raw = relative.as_posix()
    else:
        raw = value.as_posix() if isinstance(value, PurePath) else os.fspath(value)
    try:
        return freezer.safe_repository_path(raw)
    except Exception as exc:
        raise V12R9ExecutionError(f"unsafe repository path: {raw!r}") from exc


def _logical(path: str | PurePath | Path) -> str:
    return _path(path).relative_to(REPO_ROOT).as_posix()


def _record(path: str | PurePath | Path) -> dict[str, Any]:
    try:
        return freezer.artifact_record(_logical(path))
    except Exception as exc:
        raise V12R9ExecutionError(f"invalid artifact: {path}") from exc


def _tree(path: str | PurePath | Path) -> dict[str, Any]:
    try:
        return freezer.tree_record(_logical(path))
    except Exception as exc:
        raise V12R9ExecutionError(f"invalid artifact tree: {path}") from exc


def _mapping(path: str | PurePath | Path) -> dict[str, Any]:
    target = _path(path)
    try:
        _record(target)
        value = forensic.strict_json_load(target)
    except Exception as exc:
        raise V12R9ExecutionError(f"invalid strict JSON: {target}") from exc
    if not isinstance(value, Mapping):
        raise V12R9ExecutionError(f"expected JSON object: {target}")
    return dict(value)


def _sequence(path: str | PurePath | Path) -> list[Any]:
    target = _path(path)
    try:
        _record(target)
        value = forensic.strict_json_load(target)
    except Exception as exc:
        raise V12R9ExecutionError(f"invalid strict JSON: {target}") from exc
    if not isinstance(value, list):
        raise V12R9ExecutionError(f"expected JSON array: {target}")
    return value


def _write(
    path: str | PurePath | Path,
    payload: Mapping[str, Any] | Sequence[Any],
) -> dict[str, Any]:
    target = _path(path)
    mutation_root = _path(contract.ROOT)
    try:
        target.relative_to(mutation_root)
    except ValueError as exc:
        raise V12R9ExecutionError(
            "R9 runner mutation escaped the exclusive R9 namespace"
        ) from exc
    forensic.write_json_exclusive(target, payload)
    return _record(target)


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


def _strict_schema(value: Mapping[str, Any], fields: set[str]) -> bool:
    return bool(
        set(value) == fields
        and type(value.get("schema_version")) is int
        and value.get("schema_version") == contract.SCHEMA_VERSION
    )


def _artifact_exact(value: Any) -> bool:
    if not isinstance(value, Mapping):
        return False
    try:
        return _strict_equal(dict(value), _record(str(value["path"])))
    except (KeyError, TypeError, V12R9ExecutionError):
        return False


def _zero_int(value: Any) -> bool:
    return type(value) is int and value == 0


def _zero_activity() -> dict[str, int]:
    return {name: 0 for name in ACTIVITY_FIELDS}


def _increment(activity: dict[str, int], name: str, amount: int = 1) -> None:
    if name not in activity:
        raise V12R9ExecutionError(f"unknown activity counter: {name}")
    if type(amount) is not int or amount < 0:
        raise V12R9ExecutionError("activity increment must be non-negative int")
    current = activity[name]
    if type(current) is not int or current < 0:
        raise V12R9ExecutionError("activity counter drifted")
    activity[name] = current + amount


def _increment_scoped(
    activity: dict[str, int], scope: str, name: str, amount: int = 1
) -> None:
    _increment(activity, name, amount)
    if name in {"environment_reset_calls", "environment_step_calls"}:
        _increment(activity, f"{scope}_{name}", amount)


def _claims_root() -> Path:
    return _path(contract.RUN_ROOT) / "claims"


def _stage_claim_path(stage_id: str) -> Path:
    try:
        index = tuple(contract.STAGE_IDS).index(stage_id) + 1
    except ValueError as exc:
        raise V12R9ExecutionError(f"unknown stage id: {stage_id}") from exc
    if "/" in stage_id or "\\" in stage_id or ".." in stage_id:
        raise V12R9ExecutionError("unsafe stage identifier")
    return _claims_root() / f"{index:02d}_{stage_id}.json"


def _expected_pipeline_claim() -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "CLAIMED_H0_V12R9_RECOVERY_PIPELINE_ONCE",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "authority": copy.deepcopy(contract.AUTHORITY),
        "stage_order": list(contract.STAGE_IDS),
        "protocol_freeze": _record(contract.PROTOCOL_FREEZE_PATH),
        "execution_lock": _record(contract.EXECUTION_LOCK_PATH),
        "one_shot": True,
        "single_collection_round": True,
        "single_actor_fit": True,
        "retry_authorized": False,
        "resume_authorized": False,
        "alpha_sweep_authorized": False,
        "qualification_execution_authorized": False,
    }


def verify_pipeline_claim() -> dict[str, Any]:
    claim = _mapping(contract.CLAIM_PATH)
    if not _strict_equal(claim, _expected_pipeline_claim()):
        raise V12R9ExecutionError("pipeline claim semantic drifted")
    return claim


def _expected_worker_claim(stage_id: str) -> dict[str, Any]:
    if stage_id not in contract.STAGE_IDS:
        raise V12R9ExecutionError(f"unknown stage id: {stage_id}")
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "CLAIMED_H0_V12R9_STAGE_ONCE",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": stage_id,
        "pipeline_claim": _record(contract.CLAIM_PATH),
        "protocol_freeze": _record(contract.PROTOCOL_FREEZE_PATH),
        "execution_lock": _record(contract.EXECUTION_LOCK_PATH),
        "one_shot": True,
        "retry_authorized": False,
        "resume_authorized": False,
        "alpha_sweep_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }


def verify_worker_claim(stage_id: str) -> dict[str, Any]:
    verify_pipeline_claim()
    claim = _mapping(_stage_claim_path(stage_id))
    if not _strict_equal(claim, _expected_worker_claim(stage_id)):
        raise V12R9ExecutionError(f"worker claim semantic drifted: {stage_id}")
    return claim


def verify_claim_directory(stage_ids: Sequence[str]) -> dict[str, Any]:
    expected_ids = list(stage_ids)
    if len(expected_ids) != len(set(expected_ids)) or any(
        stage_id not in contract.STAGE_IDS for stage_id in expected_ids
    ):
        raise V12R9ExecutionError("claim directory stage set drifted")
    root = _claims_root()
    if not root.is_dir() or root.is_symlink():
        raise V12R9ExecutionError("claims root is absent or unsafe")
    expected_names = {_stage_claim_path(stage_id).name for stage_id in expected_ids}
    entries = list(root.iterdir())
    if {entry.name for entry in entries} != expected_names or any(
        not entry.is_file() or entry.is_symlink() for entry in entries
    ):
        raise V12R9ExecutionError("claims directory contains missing or extra entries")
    for stage_id in expected_ids:
        verify_worker_claim(stage_id)
    return {"passed": True, "stage_ids": expected_ids}


def _claim_stage(stage_id: str) -> dict[str, Any]:
    payload = _expected_worker_claim(stage_id)
    _write(_stage_claim_path(stage_id), payload)
    return verify_worker_claim(stage_id)


def _collection_stage_receipt_path(case_id: str) -> Path:
    contract.canonical_collection_case(case_id)
    return _path(contract.collection_case_root(case_id)) / "stage_receipt.json"


def _development_receipt_path(case_id: str) -> Path:
    contract.canonical_development_case(case_id)
    return _path(contract.DEVELOPMENT_ROOT) / case_id / "receipt.json"


def _historical_label_root() -> Path:
    return _path(contract.R8_PLUS_LABEL_ROOT)


def _imported_minus_label_root() -> Path:
    return (
        _path(contract.collection_case_root(contract.IMPORTED_MINUS_CASE_ID))
        / "observer_labels"
    )


def _run_r8_terminal_adjudication(adapters: RunnerAdapters) -> dict[str, Any]:
    result = adapters.run_adjudication(
        destination=_path(contract.R8_MINUS_ADJUDICATION_PATH),
        artifact_root=REPO_ROOT,
    )
    verified = probe.verify_r8_minus_adjudication(
        destination=_path(contract.R8_MINUS_ADJUDICATION_PATH),
        artifact_root=REPO_ROOT,
    )
    if not _strict_equal(result, verified) or verified.get("passed") is not True:
        raise V12R9ExecutionError("R8 terminal/minus adjudication verifier disagrees")
    if any(
        verified.get(name) != 0 or type(verified.get(name)) is not int
        for name in ("environment_reset_calls", "environment_step_calls")
    ):
        raise V12R9ExecutionError("R8 adjudication accessed an environment")
    stage_id = "adjudicate_r8_terminal_and_minus_prefix"
    stage_receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "PASS_H0_V12R9_R8_TERMINAL_ADJUDICATION_STAGE",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": stage_id,
        "case_id": contract.IMPORTED_MINUS_CASE_ID,
        "adjudication": _record(contract.R8_MINUS_ADJUDICATION_PATH),
        "r8_terminal_ledger": _record(contract.R8_TERMINAL_LEDGER_PATH),
        "r8_minus_trace": _record(contract.R8_MINUS_TRACE_PATH),
        "r8_minus_replay": _record(contract.R8_MINUS_REPLAY_PATH),
        "r8_minus_summary": _record(contract.R8_MINUS_SUMMARY_PATH),
        "r8_minus_gate": _record(contract.R8_MINUS_GATE_PATH),
        "r8_minus_receipt": _record(contract.R8_MINUS_RECEIPT_PATH),
        "candidate_module": copy.deepcopy(contract.FULL_R6_CANDIDATE_TREE),
        "pipeline_claim": _record(contract.CLAIM_PATH),
        "worker_claim": _record(_stage_claim_path(stage_id)),
        "offline_label_authorized": True,
        "r8_probe_rerun": False,
        "r8_artifacts_modified": False,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    _write(contract.R8_MINUS_ADJUDICATION_STAGE_RECEIPT_PATH, stage_receipt)
    return verify_r8_terminal_adjudication_stage_receipt()


def verify_r8_terminal_adjudication_stage_receipt() -> dict[str, Any]:
    semantic = probe.verify_r8_minus_adjudication(
        destination=_path(contract.R8_MINUS_ADJUDICATION_PATH),
        artifact_root=REPO_ROOT,
    )
    receipt = _mapping(contract.R8_MINUS_ADJUDICATION_STAGE_RECEIPT_PATH)
    stage_id = "adjudicate_r8_terminal_and_minus_prefix"
    verify_worker_claim(stage_id)
    expected_records = {
        "adjudication": contract.R8_MINUS_ADJUDICATION_PATH,
        "r8_terminal_ledger": contract.R8_TERMINAL_LEDGER_PATH,
        "r8_minus_trace": contract.R8_MINUS_TRACE_PATH,
        "r8_minus_replay": contract.R8_MINUS_REPLAY_PATH,
        "r8_minus_summary": contract.R8_MINUS_SUMMARY_PATH,
        "r8_minus_gate": contract.R8_MINUS_GATE_PATH,
        "r8_minus_receipt": contract.R8_MINUS_RECEIPT_PATH,
        "pipeline_claim": contract.CLAIM_PATH,
        "worker_claim": _stage_claim_path(stage_id),
    }
    checks = {
        "schema": _strict_schema(receipt, R8_ADJUDICATION_STAGE_FIELDS),
        "semantic": semantic.get("passed") is True
        and semantic.get("offline_label_authorized") is True
        and semantic.get("candidate_identity_adjudication", {}).get("mismatch_fields")
        == ["candidate_module"],
        "identity": receipt.get("status")
        == "PASS_H0_V12R9_R8_TERMINAL_ADJUDICATION_STAGE"
        and receipt.get("passed") is True
        and receipt.get("protocol_id") == contract.PROTOCOL_ID
        and receipt.get("pipeline_id") == contract.PIPELINE_ID
        and receipt.get("stage_id") == stage_id
        and receipt.get("case_id") == contract.IMPORTED_MINUS_CASE_ID,
        "records": all(
            receipt.get(name) == _record(path)
            for name, path in expected_records.items()
        ),
        "candidate_full_tree": receipt.get("candidate_module")
        == contract.FULL_R6_CANDIDATE_TREE,
        "immutable_no_rerun": receipt.get("offline_label_authorized") is True
        and receipt.get("r8_probe_rerun") is False
        and receipt.get("r8_artifacts_modified") is False,
        "zero_activity": all(
            type(receipt.get(name)) is int and receipt[name] == 0
            for name in (
                "environment_reset_calls",
                "environment_step_calls",
                "actor_updates",
                "critic_updates",
                "ppo_updates",
            )
        ),
    }
    if not all(value is True for value in checks.values()):
        failed = sorted(name for name, value in checks.items() if not value)
        raise V12R9ExecutionError(f"R8 adjudication stage drifted: {failed}")
    return receipt


def _run_plus_label_import(
    activity: dict[str, int], adapters: RunnerAdapters
) -> dict[str, Any]:
    imported = adapters.import_plus_labels(
        artifact_root=REPO_ROOT, semantic_verify=True
    )
    if (
        imported.get("passed") is not True
        or imported.get("direct_immutable_reference") is not True
        or imported.get("labels_copied") is not False
        or imported.get("semantic_and_byte_exact_closed") is not True
    ):
        raise V12R9ExecutionError("R8 plus label import failed closed")
    rows = imported.get("labelled_row_count")
    if type(rows) is not int or rows != 179:
        raise V12R9ExecutionError("R8 plus imported row count drifted")
    stage_id = "import_r8_plus_labels"
    stage_receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "PASS_H0_V12R9_R8_PLUS_LABEL_IMPORT_STAGE",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": stage_id,
        "case_id": contract.HISTORICAL_CASE_ID,
        "label_receipt": _record(contract.R8_PLUS_LABEL_RECEIPT_PATH),
        "labels": _record(contract.R8_PLUS_LABELS_PATH),
        "label_summary": _record(contract.R8_PLUS_LABEL_SUMMARY_PATH),
        "label_gate": _record(contract.R8_PLUS_LABEL_GATE_PATH),
        "source_stage_receipt": _record(contract.R8_PLUS_LABEL_STAGE_RECEIPT_PATH),
        "pipeline_claim": _record(contract.CLAIM_PATH),
        "worker_claim": _record(_stage_claim_path(stage_id)),
        "labelled_row_count": rows,
        "same_state_teacher_label_count": rows,
        "teacher_query_count": 0,
        "direct_immutable_reference": True,
        "labels_copied": False,
        "semantic_and_byte_exact_closed": True,
        "r8_artifacts_modified": False,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    _write(contract.R8_PLUS_IMPORT_STAGE_RECEIPT_PATH, stage_receipt)
    _increment(activity, "imported_observer_label_rows", rows)
    _increment(activity, "collection_rounds_completed")
    return verify_plus_label_import_receipt()


def verify_plus_label_import_receipt() -> dict[str, Any]:
    semantic = probe.verify_observer_label_closure(
        contract.HISTORICAL_CASE_ID, require_stage_receipt=False
    )
    receipt = _mapping(contract.R8_PLUS_IMPORT_STAGE_RECEIPT_PATH)
    stage_id = "import_r8_plus_labels"
    verify_worker_claim(stage_id)
    expected_records = {
        "label_receipt": contract.R8_PLUS_LABEL_RECEIPT_PATH,
        "labels": contract.R8_PLUS_LABELS_PATH,
        "label_summary": contract.R8_PLUS_LABEL_SUMMARY_PATH,
        "label_gate": contract.R8_PLUS_LABEL_GATE_PATH,
        "source_stage_receipt": contract.R8_PLUS_LABEL_STAGE_RECEIPT_PATH,
        "pipeline_claim": contract.CLAIM_PATH,
        "worker_claim": _stage_claim_path(stage_id),
    }
    checks = {
        "schema": _strict_schema(receipt, PLUS_IMPORT_STAGE_FIELDS),
        "identity": receipt.get("status") == "PASS_H0_V12R9_R8_PLUS_LABEL_IMPORT_STAGE"
        and receipt.get("passed") is True
        and receipt.get("protocol_id") == contract.PROTOCOL_ID
        and receipt.get("pipeline_id") == contract.PIPELINE_ID
        and receipt.get("stage_id") == stage_id
        and receipt.get("case_id") == contract.HISTORICAL_CASE_ID,
        "records": all(
            receipt.get(name) == _record(path)
            for name, path in expected_records.items()
        ),
        "semantic": semantic.get("passed") is True
        and semantic.get("semantic_and_byte_exact_closed") is True
        and semantic.get("direct_immutable_reference") is True,
        "rows": receipt.get("labelled_row_count")
        == receipt.get("same_state_teacher_label_count")
        == semantic.get("labelled_row_count")
        == 179,
        "direct_reference": receipt.get("direct_immutable_reference") is True
        and receipt.get("labels_copied") is False
        and receipt.get("semantic_and_byte_exact_closed") is True
        and receipt.get("r8_artifacts_modified") is False,
        "zero_activity": all(
            type(receipt.get(name)) is int and receipt[name] == 0
            for name in (
                "teacher_query_count",
                "environment_reset_calls",
                "environment_step_calls",
                "actor_updates",
                "critic_updates",
                "ppo_updates",
            )
        ),
    }
    if not all(value is True for value in checks.values()):
        failed = sorted(name for name, value in checks.items() if not value)
        raise V12R9ExecutionError(f"R8 plus import stage drifted: {failed}")
    return receipt


def _run_imported_minus_label(
    activity: dict[str, int], adapters: RunnerAdapters
) -> dict[str, Any]:
    label_root = _imported_minus_label_root()
    result = adapters.run_imported_minus_labels(
        adjudication_destination=_path(contract.R8_MINUS_ADJUDICATION_PATH),
        label_destination=label_root,
        source_h0_path=_path(contract.SOURCE_H0_MODULE_PATH),
        artifact_root=REPO_ROOT,
    )
    if result.get("passed") is not True:
        raise V12R9ExecutionError("R8 minus offline labels failed")
    semantic = probe.verify_observer_label_closure(
        contract.IMPORTED_MINUS_CASE_ID, require_stage_receipt=False
    )
    rows = semantic.get("labelled_row_count")
    if type(rows) is not int or rows != 252:
        raise V12R9ExecutionError("R8 minus label row count drifted")
    stage_id = "label_r8_minus_prefix"
    label_receipt = _mapping(label_root / "receipt.json")
    stage_receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "PASS_H0_V12R9_R8_MINUS_OFFLINE_LABEL_STAGE",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": stage_id,
        "case_id": contract.IMPORTED_MINUS_CASE_ID,
        "adjudication_stage_receipt": _record(
            contract.R8_MINUS_ADJUDICATION_STAGE_RECEIPT_PATH
        ),
        "source_replay": _record(contract.R8_MINUS_REPLAY_PATH),
        "label_receipt": _record(label_root / "receipt.json"),
        "labels": _record(label_root / "labels.npz"),
        "label_summary": _record(label_root / "summary.json"),
        "label_gate": _record(label_root / "gate.json"),
        "pipeline_claim": _record(contract.CLAIM_PATH),
        "worker_claim": _record(_stage_claim_path(stage_id)),
        "labelled_row_count": rows,
        "same_state_teacher_label_count": rows,
        "teacher_query_count": rows,
        "teacher_h0_id": label_receipt["teacher_h0_id"],
        "teacher_h0": copy.deepcopy(label_receipt["teacher_h0"]),
        "coverage_reference_corpus": copy.deepcopy(
            label_receipt["coverage_reference_corpus"]
        ),
        "imported_r8_prefix": True,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    _write(contract.R8_MINUS_LABEL_STAGE_RECEIPT_PATH, stage_receipt)
    _increment(activity, "offline_teacher_label_calls", rows)
    _increment(activity, "adjudicated_prefix_label_rows", rows)
    _increment(activity, "collection_rounds_completed")
    return verify_imported_minus_label_receipt()


def verify_imported_minus_label_receipt() -> dict[str, Any]:
    semantic = probe.verify_observer_label_closure(
        contract.IMPORTED_MINUS_CASE_ID, require_stage_receipt=False
    )
    receipt = _mapping(contract.R8_MINUS_LABEL_STAGE_RECEIPT_PATH)
    label_root = _imported_minus_label_root()
    stage_id = "label_r8_minus_prefix"
    verify_worker_claim(stage_id)
    expected_records = {
        "adjudication_stage_receipt": (
            contract.R8_MINUS_ADJUDICATION_STAGE_RECEIPT_PATH
        ),
        "source_replay": contract.R8_MINUS_REPLAY_PATH,
        "label_receipt": label_root / "receipt.json",
        "labels": label_root / "labels.npz",
        "label_summary": label_root / "summary.json",
        "label_gate": label_root / "gate.json",
        "pipeline_claim": contract.CLAIM_PATH,
        "worker_claim": _stage_claim_path(stage_id),
    }
    checks = {
        "schema": _strict_schema(receipt, MINUS_LABEL_STAGE_FIELDS),
        "identity": receipt.get("status")
        == "PASS_H0_V12R9_R8_MINUS_OFFLINE_LABEL_STAGE"
        and receipt.get("passed") is True
        and receipt.get("protocol_id") == contract.PROTOCOL_ID
        and receipt.get("pipeline_id") == contract.PIPELINE_ID
        and receipt.get("stage_id") == stage_id
        and receipt.get("case_id") == contract.IMPORTED_MINUS_CASE_ID,
        "records": all(
            receipt.get(name) == _record(path)
            for name, path in expected_records.items()
        ),
        "same_state": receipt.get("labelled_row_count")
        == receipt.get("same_state_teacher_label_count")
        == receipt.get("teacher_query_count")
        == semantic.get("labelled_row_count")
        == semantic.get("teacher_query_count")
        == 252,
        "semantic_recompute": semantic.get("passed") is True,
        "offline_label_inputs": receipt.get("teacher_h0_id")
        == semantic.get("teacher_h0_id")
        and receipt.get("teacher_h0") == semantic.get("teacher_h0")
        and receipt.get("coverage_reference_corpus")
        == semantic.get("coverage_reference_corpus"),
        "imported_no_environment": receipt.get("imported_r8_prefix") is True
        and all(
            _zero_int(receipt.get(name))
            for name in (
                "environment_reset_calls",
                "environment_step_calls",
                "actor_updates",
                "critic_updates",
                "ppo_updates",
            )
        ),
    }
    if not all(value is True for value in checks.values()):
        failed = sorted(name for name, value in checks.items() if not value)
        raise V12R9ExecutionError(f"R8 minus label stage drifted: {failed}")
    return receipt


def _run_collection_label(
    case_id: str,
    activity: dict[str, int],
    adapters: RunnerAdapters,
) -> dict[str, Any]:
    stage_id = f"collect_label__{case_id}"
    worker_claim = _record(_stage_claim_path(stage_id))
    case_root = _path(contract.collection_case_root(case_id))
    config = probe.RecoveryProbeConfig(case_id=case_id, artifact_root=REPO_ROOT)
    result = adapters.run_probe(
        config=config,
        destination=case_root,
        module_path=_path(contract.R6_CANDIDATE_MODULE_PATH),
        activity_callback=lambda name, amount: _increment_scoped(
            activity, "new_collection", name, amount
        ),
    )
    if result.get("passed") is not True:
        raise V12R9ExecutionError(f"R9 collection probe failed: {case_id}")
    closed = probe.verify_probe_closure(case_root, config=config)
    label_root = case_root / "observer_labels"
    labelled = adapters.run_labels(
        config=config,
        probe_destination=case_root,
        label_destination=label_root,
        source_h0_path=_path(contract.SOURCE_H0_MODULE_PATH),
    )
    if labelled.get("passed") is not True:
        raise V12R9ExecutionError(f"R9 observer labels failed: {case_id}")
    label_summary = _mapping(label_root / "summary.json")
    label_gate = _mapping(label_root / "gate.json")
    if (
        contract.label_gate(label_summary) != label_gate
        or label_gate.get("passed") is not True
    ):
        raise V12R9ExecutionError(f"R9 observer label gate drifted: {case_id}")
    rows = label_summary.get("labelled_row_count")
    if type(rows) is not int or rows < contract.MINIMUM_RECOVERABLE_PREFIX_STEPS:
        raise V12R9ExecutionError("observer label row count drifted")
    _increment(activity, "offline_teacher_label_calls", rows)
    _increment(activity, "new_collection_label_rows", rows)
    _increment(activity, "collection_rounds_completed")
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "PASS_H0_V12R9_COLLECTION_AND_LABEL_STAGE",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": stage_id,
        "case_id": case_id,
        "candidate_tree_sha256": contract.LOCKED_INPUTS["r6_candidate"]["tree_sha256"],
        "probe_step_count": closed["replay"].n_steps,
        "labelled_row_count": rows,
        "same_state_teacher_label_count": rows,
        "teacher_query_count": rows,
        "teacher_h0_id": labelled["receipt"]["teacher_h0_id"],
        "teacher_h0": copy.deepcopy(labelled["receipt"]["teacher_h0"]),
        "coverage_reference_corpus": copy.deepcopy(
            labelled["receipt"]["coverage_reference_corpus"]
        ),
        "probe_receipt": _record(case_root / "receipt.json"),
        "probe_gate": _record(case_root / "gate.json"),
        "probe_replay": _record(case_root / "replay_boundaries.npz"),
        "labels": _record(label_root / "labels.npz"),
        "label_summary": _record(label_root / "summary.json"),
        "label_gate": _record(label_root / "gate.json"),
        "label_receipt": _record(label_root / "receipt.json"),
        "pipeline_claim": _record(contract.CLAIM_PATH),
        "worker_claim": worker_claim,
        "single_collection_round": True,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    _write(_collection_stage_receipt_path(case_id), receipt)
    return verify_collection_label_receipt(case_id)


def verify_collection_label_receipt(case_id: str) -> dict[str, Any]:
    stage_id = f"collect_label__{case_id}"
    verify_worker_claim(stage_id)
    root = _path(contract.collection_case_root(case_id))
    config = probe.RecoveryProbeConfig(case_id=case_id, artifact_root=REPO_ROOT)
    closed = probe.verify_probe_closure(root, config=config)
    label_root = root / "observer_labels"
    summary = _mapping(label_root / "summary.json")
    gate = _mapping(label_root / "gate.json")
    label_receipt = _mapping(label_root / "receipt.json")
    receipt = _mapping(_collection_stage_receipt_path(case_id))
    semantic = probe.verify_observer_label_closure(case_id)
    records = {
        "probe_receipt": root / "receipt.json",
        "probe_gate": root / "gate.json",
        "probe_replay": root / "replay_boundaries.npz",
        "labels": label_root / "labels.npz",
        "label_summary": label_root / "summary.json",
        "label_gate": label_root / "gate.json",
        "label_receipt": label_root / "receipt.json",
        "pipeline_claim": contract.CLAIM_PATH,
        "worker_claim": _stage_claim_path(stage_id),
    }
    checks = {
        "schema": _strict_schema(receipt, COLLECTION_LABEL_STAGE_FIELDS),
        "identity": receipt.get("status") == "PASS_H0_V12R9_COLLECTION_AND_LABEL_STAGE"
        and receipt.get("passed") is True
        and receipt.get("protocol_id") == contract.PROTOCOL_ID
        and receipt.get("pipeline_id") == contract.PIPELINE_ID
        and receipt.get("stage_id") == stage_id
        and receipt.get("case_id") == case_id,
        "probe": closed.get("passed") is True
        and type(receipt.get("probe_step_count")) is int
        and receipt.get("probe_step_count")
        == semantic.get("labelled_row_count")
        == closed["replay"].n_steps,
        "label_gate": gate == contract.label_gate(summary)
        and gate.get("passed") is True
        and label_receipt.get("passed") is True,
        "same_state": type(receipt.get("labelled_row_count")) is int
        and receipt.get("labelled_row_count") == semantic.get("labelled_row_count")
        and receipt.get("same_state_teacher_label_count")
        == semantic.get("same_state_teacher_label_count")
        == receipt.get("labelled_row_count")
        and receipt.get("teacher_query_count")
        == semantic.get("teacher_query_count")
        == receipt.get("labelled_row_count"),
        "candidate_binding": receipt.get("candidate_tree_sha256")
        == contract.LOCKED_INPUTS["r6_candidate"]["tree_sha256"],
        "semantic_recompute": semantic.get("passed") is True,
        "offline_label_inputs": receipt.get("teacher_h0_id")
        == semantic.get("teacher_h0_id")
        and receipt.get("teacher_h0") == semantic.get("teacher_h0")
        and receipt.get("coverage_reference_corpus")
        == semantic.get("coverage_reference_corpus"),
        "records": all(
            receipt.get(name) == _record(path) for name, path in records.items()
        ),
        "one_round": receipt.get("single_collection_round") is True,
        "zero_updates": all(
            type(receipt.get(name)) is int and receipt[name] == 0
            for name in ("actor_updates", "critic_updates", "ppo_updates")
        ),
    }
    if not all(value is True for value in checks.values()):
        failed = sorted(name for name, value in checks.items() if not value)
        raise V12R9ExecutionError(f"collection/label receipt drifted: {failed}")
    return receipt


def _run_fit(activity: dict[str, int], adapters: RunnerAdapters) -> dict[str, Any]:
    stage_id = "fit_recovery_actor"
    _increment(activity, "actor_fit_stage_calls")
    receipt = adapters.run_fit(
        pipeline_claim_path=_path(contract.CLAIM_PATH),
        worker_claim_path=_stage_claim_path(stage_id),
        protocol_freeze_path=_path(contract.PROTOCOL_FREEZE_PATH),
        execution_lock_path=_path(contract.EXECUTION_LOCK_PATH),
        activity_callback=lambda name, amount: _increment(activity, name, amount),
    )
    if receipt.get("passed") is not True:
        raise V12R9ExecutionError("R9 actor fit receipt is not PASS")
    verified = dict(adapters.verify_fit())
    if not _strict_equal(receipt, verified):
        raise V12R9ExecutionError("R9 actor fit verifier disagrees")
    return verified


def _candidate_semantic_audit(
    module: Mapping[str, Any],
    actor_manifest: Mapping[str, Any],
    build_manifest: Mapping[str, Any],
) -> dict[str, Any]:
    file_names = {
        row.get("path") for row in module.get("files", []) if isinstance(row, Mapping)
    }
    checks = {
        "five_file_tree": module.get("file_count") == 5
        and file_names == EXPECTED_CANDIDATE_FILES,
        "actor_manifest": actor_manifest.get("status")
        == contract.ACTOR_FEATURE_MANIFEST_STATUS
        and actor_manifest.get("topology_id") == contract.TOPOLOGY_ID
        and actor_manifest.get("fit_contract_id") == contract.FIT_CONTRACT_ID
        and actor_manifest.get("actor_feature_count")
        == contract.EXPECTED_ACTOR_FEATURES
        and actor_manifest.get("fcnet_hiddens") == [512, 512]
        and actor_manifest.get("disabled_clock_columns") == [0, 1],
        "build_manifest": build_manifest.get("passed") is True
        and build_manifest.get("protocol_id") == contract.PROTOCOL_ID
        and build_manifest.get("fit_contract_id") == contract.FIT_CONTRACT_ID
        and build_manifest.get("topology_id") == contract.TOPOLOGY_ID
        and build_manifest.get("architecture") == contract.FIT["architecture"],
        "single_update": type(build_manifest.get("actor_fit_count")) is int
        and build_manifest.get("actor_fit_count") == 1
        and type(build_manifest.get("actor_updates")) is int
        and build_manifest.get("actor_updates") == 1
        and type(build_manifest.get("critic_updates")) is int
        and build_manifest.get("critic_updates") == 0
        and type(build_manifest.get("ppo_updates")) is int
        and build_manifest.get("ppo_updates") == 0,
        "preservation": build_manifest.get("logstd_byte_exact") is True
        and build_manifest.get("disabled_clock_columns_bit_zero") is True
        and build_manifest.get("save_reload_exact") is True,
        "digest_bound": actor_manifest.get("actor_digest")
        == build_manifest.get("actor_digest")
        and actor_manifest.get("module_state_sha256")
        == build_manifest.get("module_state_sha256"),
    }
    return {"passed": all(value is True for value in checks.values()), "checks": checks}


def _run_candidate_freeze(adapters: RunnerAdapters) -> dict[str, Any]:
    fit_receipt = dict(adapters.verify_fit())
    module = _tree(contract.CANDIDATE_MODULE_PATH)
    identity = contract.candidate_id(module["tree_sha256"])
    actor_manifest_path = (
        _path(contract.CANDIDATE_MODULE_PATH) / "actor_feature_manifest.json"
    )
    build_manifest_path = (
        _path(contract.CANDIDATE_MODULE_PATH) / "candidate_build_manifest.json"
    )
    actor_manifest = _mapping(actor_manifest_path)
    build_manifest = _mapping(build_manifest_path)
    semantic = _candidate_semantic_audit(module, actor_manifest, build_manifest)
    if semantic.get("passed") is not True:
        raise V12R9ExecutionError("R9 candidate semantic freeze failed")
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.CANDIDATE_FREEZE_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": "freeze_recovery_actor",
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_id": identity,
        "candidate_module": module,
        "fit_receipt": _record(contract.FIT_RECEIPT_PATH),
        "actor_feature_manifest": _record(actor_manifest_path),
        "actor_digest": actor_manifest["actor_digest"],
        "candidate_build_manifest": _record(build_manifest_path),
        "semantic_audit": semantic,
        "pipeline_claim": _record(contract.CLAIM_PATH),
        "worker_claim": _record(_stage_claim_path("freeze_recovery_actor")),
        "candidate_frozen": True,
        "fit_gate_passed": True,
        "standard_actor": True,
        "warm_start_target_512_compatible": True,
        "actor_fit_count": 1,
        "runtime_promoted": False,
        "qualification_executed": False,
        "q3_paths_opened": [],
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    if fit_receipt.get("candidate_id") != identity:
        raise V12R9ExecutionError("fit/freeze candidate identity drifted")
    _write(contract.CANDIDATE_FREEZE_PATH, receipt)
    return verify_candidate_freeze_receipt(fit_verifier=adapters.verify_fit)


def verify_candidate_freeze_receipt(
    *, fit_verifier: Callable[[], Mapping[str, Any]] | None = None
) -> dict[str, Any]:
    verify_fit = fitter.verify_fit_stage if fit_verifier is None else fit_verifier
    fit_receipt = dict(verify_fit())
    verify_worker_claim("fit_recovery_actor")
    verify_worker_claim("freeze_recovery_actor")
    receipt = _mapping(contract.CANDIDATE_FREEZE_PATH)
    module = _tree(contract.CANDIDATE_MODULE_PATH)
    identity = contract.candidate_id(module["tree_sha256"])
    actor_path = _path(contract.CANDIDATE_MODULE_PATH) / "actor_feature_manifest.json"
    build_path = _path(contract.CANDIDATE_MODULE_PATH) / "candidate_build_manifest.json"
    semantic = _candidate_semantic_audit(
        module, _mapping(actor_path), _mapping(build_path)
    )
    checks = {
        "schema": _strict_schema(receipt, CANDIDATE_FREEZE_FIELDS),
        "identity": receipt.get("status") == contract.CANDIDATE_FREEZE_PASS_STATUS
        and receipt.get("passed") is True
        and receipt.get("protocol_id") == contract.PROTOCOL_ID
        and receipt.get("pipeline_id") == contract.PIPELINE_ID
        and receipt.get("stage_id") == "freeze_recovery_actor"
        and receipt.get("candidate_selection_rule")
        == contract.CANDIDATE_SELECTION_RULE,
        "candidate": receipt.get("candidate_id") == identity
        and receipt.get("candidate_module") == module
        and fit_receipt.get("candidate_id") == identity,
        "fit": receipt.get("fit_receipt") == _record(contract.FIT_RECEIPT_PATH),
        "manifests": receipt.get("actor_feature_manifest") == _record(actor_path)
        and receipt.get("candidate_build_manifest") == _record(build_path)
        and receipt.get("actor_digest") == _mapping(actor_path).get("actor_digest"),
        "semantic": semantic.get("passed") is True
        and receipt.get("semantic_audit") == semantic,
        "claims": receipt.get("pipeline_claim") == _record(contract.CLAIM_PATH)
        and receipt.get("worker_claim")
        == _record(_stage_claim_path("freeze_recovery_actor")),
        "closed": receipt.get("candidate_frozen") is True
        and receipt.get("fit_gate_passed") is True
        and receipt.get("standard_actor") is True
        and receipt.get("warm_start_target_512_compatible") is True
        and type(receipt.get("actor_fit_count")) is int
        and receipt.get("actor_fit_count") == 1
        and receipt.get("q3_paths_opened") == []
        and receipt.get("runtime_promoted") is False
        and receipt.get("qualification_executed") is False,
        "counters": type(receipt.get("actor_updates")) is int
        and receipt.get("actor_updates") == 1
        and type(receipt.get("critic_updates")) is int
        and receipt.get("critic_updates") == 0
        and type(receipt.get("ppo_updates")) is int
        and receipt.get("ppo_updates") == 0,
    }
    if not all(value is True for value in checks.values()):
        failed = sorted(name for name, value in checks.items() if not value)
        raise V12R9ExecutionError(f"candidate freeze receipt drifted: {failed}")
    return receipt


def _physical_config() -> physical.PhysicalDevelopmentConfig:
    return physical.PhysicalDevelopmentConfig(
        protocol_id=contract.PROTOCOL_ID,
        start_status="STARTED_H0_V12R9_PURE_DEVELOPMENT",
        partial_status="PERSISTED_H0_V12R9_DEVELOPMENT_BEFORE_GATE",
        complete_status="COMPLETE_H0_V12R9_PURE_DEVELOPMENT",
        schema_version=contract.SCHEMA_VERSION,
        artifact_root=REPO_ROOT,
        progress_label="V12R9 pure development",
        progress_every=25,
    )


def _recompute_development_runtime_evidence(
    *, summary: Mapping[str, Any], trace: Sequence[Any], case_id: str
) -> tuple[dict[str, Any], dict[str, bool]]:
    audit = physical.pure_policy_trace_audit(
        trace, config=_physical_config(), case_id=case_id
    )
    checks = {
        "pure_trace": audit.get("passed") is True,
        "detector_active": summary.get("binary_phase_fsm_mode") == "binary_active"
        and summary.get("event_contract_id") == contract.EVENT_CONTRACT_ID
        and summary.get("target_contract_id") == contract.TARGET_CONTRACT_ID,
        "morphology_zero": isinstance(summary.get("morphology_weight"), (int, float))
        and not isinstance(summary.get("morphology_weight"), bool)
        and float(summary["morphology_weight"]) == 0.0,
        "zero_updates": all(
            type(summary.get(name)) is int and summary[name] == 0
            for name in ("actor_updates", "critic_updates", "ppo_updates")
        ),
    }
    return audit, checks


def _run_development(
    case_id: str,
    activity: dict[str, int],
    adapters: RunnerAdapters,
) -> dict[str, Any]:
    verify_candidate_freeze_receipt(fit_verifier=adapters.verify_fit)
    module = _tree(contract.CANDIDATE_MODULE_PATH)
    identity = contract.candidate_id(module["tree_sha256"])
    case = contract.canonical_development_case(case_id)
    destination = _path(contract.DEVELOPMENT_ROOT) / case_id
    result = adapters.run_physical(
        config=_physical_config(),
        case=case,
        destination=destination,
        module_path=_path(contract.CANDIDATE_MODULE_PATH),
        activity_callback=lambda name, amount: _increment_scoped(
            activity, "development", name, amount
        ),
        start_metadata={
            "pipeline_claim": _record(contract.CLAIM_PATH),
            "worker_claim": _record(_stage_claim_path(f"development__{case_id}")),
            "execution_lock": _record(contract.EXECUTION_LOCK_PATH),
            "candidate_id": identity,
            "candidate_module": module,
            "candidate_freeze": _record(contract.CANDIDATE_FREEZE_PATH),
        },
        summary_metadata={
            "candidate_id": identity,
            "candidate_module": module,
            "candidate_tree_sha256": module["tree_sha256"],
            "candidate_freeze": _record(contract.CANDIDATE_FREEZE_PATH),
            "target_contract_id": contract.TARGET_CONTRACT_ID,
            "event_contract_id": contract.EVENT_CONTRACT_ID,
        },
    )
    raw_summary = result.get("summary")
    if not isinstance(raw_summary, Mapping):
        raise V12R9ExecutionError("development summary is malformed")
    try:
        normalization = probe.adjudicator.normalize_v26_prefix_summary(
            raw_summary,
            expected_steps=int(raw_summary.get("steps", -1)),
        )
    except probe.adjudicator.V12R9AdjudicationError as exc:
        raise V12R9ExecutionError("development V26 normalization failed") from exc
    summary = {
        **normalization["summary"],
        "v26_summary_normalization": probe._normalization_receipt(normalization),
    }
    normalized_summary_path = destination / "normalized_summary.json"
    _write(normalized_summary_path, summary)
    runtime_checks = {
        "pure_trace": isinstance(summary.get("pure_policy_trace_audit"), Mapping)
        and summary["pure_policy_trace_audit"].get("passed") is True,
        "detector_active": summary.get("binary_phase_fsm_mode") == "binary_active"
        and summary.get("event_contract_id") == contract.EVENT_CONTRACT_ID
        and summary.get("target_contract_id") == contract.TARGET_CONTRACT_ID,
        "morphology_zero": isinstance(summary.get("morphology_weight"), (int, float))
        and not isinstance(summary.get("morphology_weight"), bool)
        and float(summary["morphology_weight"]) == 0.0,
        "zero_updates": all(
            type(summary.get(name)) is int and summary[name] == 0
            for name in ("actor_updates", "critic_updates", "ppo_updates")
        ),
    }
    if not all(value is True for value in runtime_checks.values()):
        failed = sorted(name for name, value in runtime_checks.items() if not value)
        raise V12R9ExecutionError(f"R9 development runtime integrity failed: {failed}")
    gate = contract.development_gate(summary, case_id=case_id)
    _write(destination / "gate.json", gate)
    if gate.get("passed") is not True:
        raise V12R9ExecutionError(f"R9 development gate failed: {case_id}")
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "PASS_H0_V12R9_PURE_DEVELOPMENT_RECEIPT",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": f"development__{case_id}",
        "case_id": case_id,
        "candidate_id": identity,
        "candidate_module": module,
        "candidate_freeze": _record(contract.CANDIDATE_FREEZE_PATH),
        "raw_summary": _record(destination / "summary.json"),
        "summary": _record(normalized_summary_path),
        "gate": _record(destination / "gate.json"),
        "trace": _record(destination / "trace.json"),
        "pure_policy_trace_audit": copy.deepcopy(
            summary.get("pure_policy_trace_audit")
        ),
        "runtime_integrity_checks": runtime_checks,
        "teacher_query_count": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    _write(destination / "receipt.json", receipt)
    _increment(activity, "development_rollouts_completed")
    return verify_development_receipt(case_id, fit_verifier=adapters.verify_fit)


def verify_development_receipt(
    case_id: str,
    *,
    fit_verifier: Callable[[], Mapping[str, Any]] | None = None,
) -> dict[str, Any]:
    verify_candidate_freeze_receipt(fit_verifier=fit_verifier)
    verify_worker_claim(f"development__{case_id}")
    root = _path(contract.DEVELOPMENT_ROOT) / case_id
    receipt = _mapping(root / "receipt.json")
    raw_summary = _mapping(root / "summary.json")
    summary = _mapping(root / "normalized_summary.json")
    gate = _mapping(root / "gate.json")
    trace = _sequence(root / "trace.json")
    module = _tree(contract.CANDIDATE_MODULE_PATH)
    identity = contract.candidate_id(module["tree_sha256"])
    expected_gate = contract.development_gate(summary, case_id=case_id)
    try:
        normalization = probe.adjudicator.normalize_v26_prefix_summary(
            raw_summary,
            expected_steps=int(raw_summary.get("steps", -1)),
        )
        expected_normalized = {
            **normalization["summary"],
            "v26_summary_normalization": probe._normalization_receipt(normalization),
        }
    except probe.adjudicator.V12R9AdjudicationError as exc:
        raise V12R9ExecutionError("development normalization closure failed") from exc
    expected_audit, expected_runtime_checks = _recompute_development_runtime_evidence(
        summary=summary, trace=trace, case_id=case_id
    )
    expected_receipt_fields = {
        "schema_version",
        "status",
        "passed",
        "protocol_id",
        "pipeline_id",
        "stage_id",
        "case_id",
        "candidate_id",
        "candidate_module",
        "candidate_freeze",
        "raw_summary",
        "summary",
        "gate",
        "trace",
        "pure_policy_trace_audit",
        "runtime_integrity_checks",
        "teacher_query_count",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
    }
    checks = {
        "schema": set(receipt) == expected_receipt_fields,
        "identity": type(receipt.get("schema_version")) is int
        and receipt.get("schema_version") == contract.SCHEMA_VERSION
        and receipt.get("status") == "PASS_H0_V12R9_PURE_DEVELOPMENT_RECEIPT"
        and receipt.get("passed") is True
        and receipt.get("protocol_id") == contract.PROTOCOL_ID
        and receipt.get("pipeline_id") == contract.PIPELINE_ID
        and receipt.get("stage_id") == f"development__{case_id}"
        and receipt.get("case_id") == case_id,
        "candidate": receipt.get("candidate_id") == identity
        and receipt.get("candidate_module") == module
        and receipt.get("candidate_freeze") == _record(contract.CANDIDATE_FREEZE_PATH),
        "gate": gate == expected_gate and gate.get("passed") is True,
        "trace_audit": expected_audit.get("passed") is True
        and _strict_equal(summary.get("pure_policy_trace_audit"), expected_audit)
        and _strict_equal(receipt.get("pure_policy_trace_audit"), expected_audit),
        "runtime_integrity": all(
            value is True for value in expected_runtime_checks.values()
        )
        and _strict_equal(
            receipt.get("runtime_integrity_checks"), expected_runtime_checks
        ),
        "normalization": summary == expected_normalized
        and summary.get("v26_summary_normalization", {}).get("passed") is True,
        "artifacts": receipt.get("raw_summary") == _record(root / "summary.json")
        and receipt.get("summary") == _record(root / "normalized_summary.json")
        and receipt.get("gate") == _record(root / "gate.json")
        and receipt.get("trace") == _record(root / "trace.json"),
        "pure_zero": all(
            type(receipt.get(name)) is int and receipt[name] == 0
            for name in (
                "teacher_query_count",
                "actor_updates",
                "critic_updates",
                "ppo_updates",
            )
        ),
    }
    if not all(value is True for value in checks.values()):
        failed = sorted(name for name, value in checks.items() if not value)
        raise V12R9ExecutionError(f"development receipt drifted: {failed}")
    return receipt


def _run_final_development(
    activity: Mapping[str, int],
    adapters: RunnerAdapters,
) -> dict[str, Any]:
    freeze = verify_candidate_freeze_receipt(fit_verifier=adapters.verify_fit)
    module = _tree(contract.CANDIDATE_MODULE_PATH)
    identity = contract.candidate_id(module["tree_sha256"])
    bindings = []
    for case_id in contract.DEVELOPMENT_CASE_IDS:
        receipt = verify_development_receipt(case_id, fit_verifier=adapters.verify_fit)
        root = _path(contract.DEVELOPMENT_ROOT) / case_id
        bindings.append(
            {
                "case_id": case_id,
                "passed": True,
                "receipt": _record(root / "receipt.json"),
                "summary": receipt["summary"],
                "gate": receipt["gate"],
            }
        )
    if activity.get("collection_rounds_completed") != len(contract.COLLECTION_CASE_IDS):
        raise V12R9ExecutionError("collection round count drifted")
    if activity.get("actor_fit_stage_calls") != 1 or activity.get("actor_updates") != 1:
        raise V12R9ExecutionError("sole actor fit count drifted")
    if activity.get("development_rollouts_completed") != len(
        contract.DEVELOPMENT_CASE_IDS
    ):
        raise V12R9ExecutionError("development rollout count drifted")
    expected_new_label_rows = sum(
        int(_mapping(_collection_stage_receipt_path(case_id))["labelled_row_count"])
        for case_id in contract.NEW_COLLECTION_CASE_IDS
    )
    expected_activity = {
        "historical_prefix_environment_reset_calls": 0,
        "historical_prefix_environment_step_calls": 0,
        "new_collection_environment_reset_calls": len(contract.NEW_COLLECTION_CASE_IDS),
        "development_environment_reset_calls": len(contract.DEVELOPMENT_CASE_IDS),
        "environment_reset_calls": len(contract.NEW_COLLECTION_CASE_IDS)
        + len(contract.DEVELOPMENT_CASE_IDS),
        "imported_observer_label_rows": 179,
        "adjudicated_prefix_label_rows": 252,
        "new_collection_label_rows": expected_new_label_rows,
        "offline_teacher_label_calls": 252 + expected_new_label_rows,
    }
    if any(activity.get(name) != value for name, value in expected_activity.items()):
        raise V12R9ExecutionError("scoped environment reset accounting drifted")
    if (
        type(activity.get("new_collection_environment_step_calls")) is not int
        or activity["new_collection_environment_step_calls"] <= 0
        or type(activity.get("development_environment_step_calls")) is not int
        or activity["development_environment_step_calls"] <= 0
        or activity.get("environment_step_calls")
        != activity["new_collection_environment_step_calls"]
        + activity["development_environment_step_calls"]
        or activity.get("raw_sensor_sample_count")
        != activity["environment_step_calls"] * contract.RAW_SAMPLES_PER_STEP
    ):
        raise V12R9ExecutionError("scoped environment step accounting drifted")
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.DEVELOPMENT_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": "finalize_development",
        "candidate_id": identity,
        "candidate_module": module,
        "candidate_freeze": _record(contract.CANDIDATE_FREEZE_PATH),
        "rollout_bindings": bindings,
        "case_gates": [
            {"case_id": case_id, "passed": True}
            for case_id in contract.DEVELOPMENT_CASE_IDS
        ],
        "collection_round_count": len(contract.COLLECTION_CASE_IDS),
        "actor_fit_count": 1,
        "actor_updates": 1,
        "development_rollout_count": len(contract.DEVELOPMENT_CASE_IDS),
        "development_count": len(contract.DEVELOPMENT_CASE_IDS),
        "passing_development_count": len(contract.DEVELOPMENT_CASE_IDS),
        "failed_development_count": 0,
        "environment_reset_calls": activity["environment_reset_calls"],
        "environment_step_calls": activity["environment_step_calls"],
        "historical_prefix_environment_reset_calls": activity[
            "historical_prefix_environment_reset_calls"
        ],
        "historical_prefix_environment_step_calls": activity[
            "historical_prefix_environment_step_calls"
        ],
        "new_collection_environment_reset_calls": activity[
            "new_collection_environment_reset_calls"
        ],
        "new_collection_environment_step_calls": activity[
            "new_collection_environment_step_calls"
        ],
        "development_environment_reset_calls": activity[
            "development_environment_reset_calls"
        ],
        "development_environment_step_calls": activity[
            "development_environment_step_calls"
        ],
        "development_actor_updates": 0,
        "development_critic_updates": 0,
        "development_ppo_updates": 0,
        "teacher_query_count": 0,
        "mean_blend_count": 0,
        "safety_intervention_count": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "qualification_executed": False,
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
        "q3_paths_opened": [],
        "pipeline_claim": _record(contract.CLAIM_PATH),
        "worker_claim": _record(_stage_claim_path("finalize_development")),
        "fit_receipt": _record(contract.FIT_RECEIPT_PATH),
        "fit_candidate_id": freeze["candidate_id"],
        "retry_authorized": False,
        "resume_authorized": False,
        "alpha_sweep_authorized": False,
    }
    _write(contract.FINAL_DEVELOPMENT_PATH, receipt)
    return verify_final_development_receipt(fit_verifier=adapters.verify_fit)


def verify_final_development_receipt(
    *, fit_verifier: Callable[[], Mapping[str, Any]] | None = None
) -> dict[str, Any]:
    freeze = verify_candidate_freeze_receipt(fit_verifier=fit_verifier)
    verify_worker_claim("finalize_development")
    receipt = _mapping(contract.FINAL_DEVELOPMENT_PATH)
    module = _tree(contract.CANDIDATE_MODULE_PATH)
    identity = contract.candidate_id(module["tree_sha256"])
    expected_bindings = []
    expected_development_steps = 0
    for case_id in contract.DEVELOPMENT_CASE_IDS:
        development = verify_development_receipt(case_id, fit_verifier=fit_verifier)
        root = _path(contract.DEVELOPMENT_ROOT) / case_id
        expected_bindings.append(
            {
                "case_id": case_id,
                "passed": True,
                "receipt": _record(root / "receipt.json"),
                "summary": development["summary"],
                "gate": development["gate"],
            }
        )
        steps = _mapping(root / "normalized_summary.json").get("steps")
        if type(steps) is not int or not 1 <= steps <= contract.EXPECTED_STEPS:
            raise V12R9ExecutionError("development step count drifted")
        expected_development_steps += steps
    expected_collection_steps = 0
    for case_id in contract.NEW_COLLECTION_CASE_IDS:
        steps = _mapping(_collection_stage_receipt_path(case_id)).get(
            "probe_step_count"
        )
        if type(steps) is not int or not 1 <= steps <= contract.EXPECTED_STEPS:
            raise V12R9ExecutionError("collection step count drifted")
        expected_collection_steps += steps
    expected_receipt_fields = {
        "schema_version",
        "status",
        "passed",
        "protocol_id",
        "pipeline_id",
        "stage_id",
        "candidate_id",
        "candidate_module",
        "candidate_freeze",
        "rollout_bindings",
        "case_gates",
        "collection_round_count",
        "actor_fit_count",
        "actor_updates",
        "development_rollout_count",
        "development_count",
        "passing_development_count",
        "failed_development_count",
        "environment_reset_calls",
        "environment_step_calls",
        "historical_prefix_environment_reset_calls",
        "historical_prefix_environment_step_calls",
        "new_collection_environment_reset_calls",
        "new_collection_environment_step_calls",
        "development_environment_reset_calls",
        "development_environment_step_calls",
        "development_actor_updates",
        "development_critic_updates",
        "development_ppo_updates",
        "teacher_query_count",
        "mean_blend_count",
        "safety_intervention_count",
        "critic_updates",
        "ppo_updates",
        "qualification_executed",
        "runtime_promoted",
        "checkpoint_zero_created",
        "positive_morphology_enabled",
        "q3_paths_opened",
        "pipeline_claim",
        "worker_claim",
        "fit_receipt",
        "fit_candidate_id",
        "retry_authorized",
        "resume_authorized",
        "alpha_sweep_authorized",
    }
    checks = {
        "schema": set(receipt) == expected_receipt_fields,
        "identity": type(receipt.get("schema_version")) is int
        and receipt.get("schema_version") == contract.SCHEMA_VERSION
        and receipt.get("status") == contract.DEVELOPMENT_PASS_STATUS
        and receipt.get("passed") is True
        and receipt.get("protocol_id") == contract.PROTOCOL_ID
        and receipt.get("pipeline_id") == contract.PIPELINE_ID
        and receipt.get("stage_id") == "finalize_development",
        "candidate": receipt.get("candidate_id") == identity
        and receipt.get("candidate_module") == module
        and receipt.get("candidate_freeze") == _record(contract.CANDIDATE_FREEZE_PATH)
        and receipt.get("fit_candidate_id") == freeze.get("candidate_id"),
        "rollouts": receipt.get("rollout_bindings") == expected_bindings
        and receipt.get("development_rollout_count")
        == len(contract.DEVELOPMENT_CASE_IDS),
        "case_gates": receipt.get("case_gates")
        == [
            {"case_id": case_id, "passed": True}
            for case_id in contract.DEVELOPMENT_CASE_IDS
        ]
        and type(receipt.get("development_count")) is int
        and receipt.get("development_count") == len(contract.DEVELOPMENT_CASE_IDS)
        and type(receipt.get("passing_development_count")) is int
        and receipt.get("passing_development_count")
        == len(contract.DEVELOPMENT_CASE_IDS)
        and type(receipt.get("failed_development_count")) is int
        and receipt.get("failed_development_count") == 0,
        "single_collection_fit": receipt.get("collection_round_count")
        == len(contract.COLLECTION_CASE_IDS)
        and type(receipt.get("collection_round_count")) is int
        and type(receipt.get("actor_fit_count")) is int
        and receipt.get("actor_fit_count") == 1
        and type(receipt.get("actor_updates")) is int
        and receipt.get("actor_updates") == 1,
        "environment_accounting": all(
            type(receipt.get(name)) is int
            for name in (
                "environment_reset_calls",
                "environment_step_calls",
                "historical_prefix_environment_reset_calls",
                "historical_prefix_environment_step_calls",
                "new_collection_environment_reset_calls",
                "new_collection_environment_step_calls",
                "development_environment_reset_calls",
                "development_environment_step_calls",
            )
        )
        and receipt.get("historical_prefix_environment_reset_calls") == 0
        and receipt.get("historical_prefix_environment_step_calls") == 0
        and receipt.get("new_collection_environment_reset_calls")
        == len(contract.NEW_COLLECTION_CASE_IDS)
        and receipt.get("development_environment_reset_calls")
        == len(contract.DEVELOPMENT_CASE_IDS)
        and receipt.get("new_collection_environment_step_calls")
        == expected_collection_steps
        and receipt.get("development_environment_step_calls")
        == expected_development_steps
        and receipt.get("environment_reset_calls")
        == len(contract.NEW_COLLECTION_CASE_IDS) + len(contract.DEVELOPMENT_CASE_IDS)
        and receipt.get("environment_step_calls")
        == receipt.get("new_collection_environment_step_calls")
        + receipt.get("development_environment_step_calls"),
        "zero_forbidden_updates": type(receipt.get("critic_updates")) is int
        and receipt.get("critic_updates") == 0
        and type(receipt.get("ppo_updates")) is int
        and receipt.get("ppo_updates") == 0,
        "pure_development_counters": all(
            type(receipt.get(name)) is int and receipt[name] == 0
            for name in (
                "development_actor_updates",
                "development_critic_updates",
                "development_ppo_updates",
                "teacher_query_count",
                "mean_blend_count",
                "safety_intervention_count",
            )
        ),
        "development_only": receipt.get("qualification_executed") is False
        and receipt.get("runtime_promoted") is False
        and receipt.get("checkpoint_zero_created") is False
        and receipt.get("positive_morphology_enabled") is False,
        "claims": receipt.get("pipeline_claim") == _record(contract.CLAIM_PATH)
        and receipt.get("worker_claim")
        == _record(_stage_claim_path("finalize_development"))
        and receipt.get("fit_receipt") == _record(contract.FIT_RECEIPT_PATH),
        "closed": receipt.get("retry_authorized") is False
        and receipt.get("resume_authorized") is False
        and receipt.get("alpha_sweep_authorized") is False
        and receipt.get("q3_paths_opened") == [],
    }
    if not all(value is True for value in checks.values()):
        failed = sorted(name for name, value in checks.items() if not value)
        raise V12R9ExecutionError(f"final development receipt drifted: {failed}")
    return receipt


def _candidate_module_snapshot() -> tuple[Any, Any, dict[str, Any]]:
    candidate = None
    candidate_id = None
    observation: dict[str, Any] = {"state": "ABSENT", "tree": None}
    candidate_path = _path(contract.CANDIDATE_MODULE_PATH)
    if candidate_path.is_dir() and not candidate_path.is_symlink():
        try:
            candidate = _tree(candidate_path)
            candidate_id = contract.candidate_id(candidate["tree_sha256"])
            observation = {"state": "VALID_NONEMPTY_TREE", "tree": candidate}
        except BaseException as exc:
            observation = {
                "state": "PARTIAL_OR_UNSAFE_TREE",
                "tree": None,
                "error": {"type": type(exc).__name__, "message": str(exc)},
            }
    elif os.path.lexists(candidate_path):
        observation = {"state": "UNSAFE_NONDIRECTORY", "tree": None}
    return candidate, candidate_id, observation


def _terminal_payload(
    *,
    passed: bool,
    completed: Sequence[Mapping[str, Any]],
    attempted_stage: str | None,
    activity: Mapping[str, int],
    error: BaseException | None,
) -> dict[str, Any]:
    candidate, candidate_id, candidate_observation = _candidate_module_snapshot()
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.PIPELINE_TERMINAL_PASS_STATUS
            if passed
            else "FAIL_H0_V12R9_RECOVERY_PIPELINE_TERMINAL"
        ),
        "passed": passed,
        "terminal": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_id": candidate_id,
        "candidate_module": candidate,
        "candidate_module_observation": candidate_observation,
        "stage_order": list(contract.STAGE_IDS),
        "completed_stages": list(completed),
        "completed_stage_count": len(completed),
        "attempted_stage": attempted_stage,
        "activity_totals": dict(activity),
        "collection_round_count": sum(
            item.get("stage_id") in {"import_r8_plus_labels", "label_r8_minus_prefix"}
            or item.get("stage_id", "").startswith("collect_label__")
            for item in completed
        ),
        "actor_fit_count": sum(
            item.get("stage_id") == "fit_recovery_actor" for item in completed
        ),
        "development_count": sum(
            item.get("stage_id", "").startswith("development__") for item in completed
        ),
        "protocol_freeze": _record(contract.PROTOCOL_FREEZE_PATH),
        "execution_lock": _record(contract.EXECUTION_LOCK_PATH),
        "pipeline_claim": _record(contract.CLAIM_PATH),
        "candidate_freeze": (
            _record(contract.CANDIDATE_FREEZE_PATH)
            if _path(contract.CANDIDATE_FREEZE_PATH).is_file()
            else None
        ),
        "final_development_receipt": (
            _record(contract.FINAL_DEVELOPMENT_PATH)
            if _path(contract.FINAL_DEVELOPMENT_PATH).is_file()
            else None
        ),
        "qualification_executed": False,
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
        "q3_paths_opened": [],
        "actor_updates": activity["actor_updates"],
        "critic_updates": activity["critic_updates"],
        "ppo_updates": activity["ppo_updates"],
        "environment_reset_calls": activity["environment_reset_calls"],
        "environment_step_calls": activity["environment_step_calls"],
        "historical_prefix_environment_reset_calls": activity[
            "historical_prefix_environment_reset_calls"
        ],
        "historical_prefix_environment_step_calls": activity[
            "historical_prefix_environment_step_calls"
        ],
        "new_collection_environment_reset_calls": activity[
            "new_collection_environment_reset_calls"
        ],
        "new_collection_environment_step_calls": activity[
            "new_collection_environment_step_calls"
        ],
        "development_environment_reset_calls": activity[
            "development_environment_reset_calls"
        ],
        "development_environment_step_calls": activity[
            "development_environment_step_calls"
        ],
        "retry_authorized": False,
        "resume_authorized": False,
        "alpha_sweep_authorized": False,
        "error": (
            None
            if error is None
            else {"type": type(error).__name__, "message": str(error) or repr(error)}
        ),
        "next_stage": (
            "WAIT_SEPARATE_V12R9Q3_PROTOCOL" if passed else "STOP_TERMINAL_NO_RETRY"
        ),
    }


def _verify_completed_stage_semantics(
    stage_id: str,
    *,
    fit_verifier: Callable[[], Mapping[str, Any]],
) -> dict[str, Any]:
    verify_worker_claim(stage_id)
    if stage_id == "adjudicate_r8_terminal_and_minus_prefix":
        return verify_r8_terminal_adjudication_stage_receipt()
    if stage_id == "import_r8_plus_labels":
        return verify_plus_label_import_receipt()
    if stage_id == "label_r8_minus_prefix":
        return verify_imported_minus_label_receipt()
    if stage_id.startswith("collect_label__"):
        return verify_collection_label_receipt(stage_id.removeprefix("collect_label__"))
    if stage_id == "fit_recovery_actor":
        return dict(fit_verifier())
    if stage_id == "freeze_recovery_actor":
        return verify_candidate_freeze_receipt(fit_verifier=fit_verifier)
    if stage_id.startswith("development__"):
        return verify_development_receipt(
            stage_id.removeprefix("development__"), fit_verifier=fit_verifier
        )
    if stage_id == "finalize_development":
        return verify_final_development_receipt(fit_verifier=fit_verifier)
    raise V12R9ExecutionError(f"unknown completed stage: {stage_id}")


def _prefix_activity_coherent(
    ledger: Mapping[str, Any],
    *,
    completed_ids: Sequence[str],
    attempted_stage: str | None,
) -> bool:
    activity = ledger.get("activity_totals")
    if not isinstance(activity, Mapping):
        return False
    completed_labels = sum(
        stage in {"import_r8_plus_labels", "label_r8_minus_prefix"}
        or stage.startswith("collect_label__")
        for stage in completed_ids
    )
    plus_import_complete = int("import_r8_plus_labels" in completed_ids)
    minus_label_complete = int("label_r8_minus_prefix" in completed_ids)
    completed_new = sum(stage.startswith("collect_label__") for stage in completed_ids)
    completed_fit = int("fit_recovery_actor" in completed_ids)
    completed_development = sum(
        stage.startswith("development__") for stage in completed_ids
    )
    attempted_plus_import = attempted_stage == "import_r8_plus_labels"
    attempted_minus_label = attempted_stage == "label_r8_minus_prefix"
    attempted_label = bool(
        attempted_plus_import
        or attempted_minus_label
        or (attempted_stage or "").startswith("collect_label__")
    )
    attempted_new = bool((attempted_stage or "").startswith("collect_label__"))
    attempted_fit = attempted_stage == "fit_recovery_actor"
    attempted_development = bool((attempted_stage or "").startswith("development__"))
    completed_new_steps = sum(
        int(
            _mapping(
                _collection_stage_receipt_path(stage.removeprefix("collect_label__"))
            )["probe_step_count"]
        )
        for stage in completed_ids
        if stage.startswith("collect_label__")
    )
    completed_new_label_rows = sum(
        int(_mapping(_stage_receipt_path(stage))["labelled_row_count"])
        for stage in completed_ids
        if stage.startswith("collect_label__")
    )
    completed_minus_label_rows = (
        int(
            _mapping(_stage_receipt_path("label_r8_minus_prefix"))["labelled_row_count"]
        )
        if minus_label_complete
        else 0
    )
    completed_imported_rows = 179 * plus_import_complete
    completed_offline_teacher_rows = (
        completed_minus_label_rows + completed_new_label_rows
    )
    completed_development_steps = sum(
        int(
            _mapping(
                _path(contract.DEVELOPMENT_ROOT)
                / stage.removeprefix("development__")
                / "normalized_summary.json"
            )["steps"]
        )
        for stage in completed_ids
        if stage.startswith("development__")
    )

    def in_closed_interval(name: str, lower: int, upper: int) -> bool:
        value = activity.get(name)
        return type(value) is int and lower <= value <= upper

    return bool(
        ledger.get("collection_round_count") == completed_labels
        and ledger.get("actor_fit_count") == completed_fit
        and ledger.get("development_count") == completed_development
        and in_closed_interval(
            "collection_rounds_completed",
            completed_labels,
            completed_labels + int(attempted_label),
        )
        and in_closed_interval(
            "new_collection_environment_reset_calls",
            completed_new,
            completed_new + int(attempted_new),
        )
        and in_closed_interval(
            "development_environment_reset_calls",
            completed_development,
            completed_development + int(attempted_development),
        )
        and activity.get("historical_prefix_environment_reset_calls") == 0
        and activity.get("historical_prefix_environment_step_calls") == 0
        and in_closed_interval(
            "new_collection_environment_step_calls",
            completed_new_steps,
            completed_new_steps + contract.EXPECTED_STEPS * int(attempted_new),
        )
        and in_closed_interval(
            "development_environment_step_calls",
            completed_development_steps,
            completed_development_steps
            + contract.EXPECTED_STEPS * int(attempted_development),
        )
        and in_closed_interval(
            "offline_teacher_label_calls",
            completed_offline_teacher_rows,
            completed_offline_teacher_rows
            + 252 * int(attempted_minus_label)
            + contract.EXPECTED_STEPS * int(attempted_new),
        )
        and in_closed_interval(
            "imported_observer_label_rows",
            completed_imported_rows,
            completed_imported_rows + 179 * int(attempted_plus_import),
        )
        and in_closed_interval(
            "adjudicated_prefix_label_rows",
            completed_minus_label_rows,
            completed_minus_label_rows + 252 * int(attempted_minus_label),
        )
        and in_closed_interval(
            "new_collection_label_rows",
            completed_new_label_rows,
            completed_new_label_rows + contract.EXPECTED_STEPS * int(attempted_new),
        )
        and activity.get("environment_reset_calls")
        == activity.get("new_collection_environment_reset_calls")
        + activity.get("development_environment_reset_calls")
        and activity.get("environment_step_calls")
        == activity.get("new_collection_environment_step_calls")
        + activity.get("development_environment_step_calls")
        and activity.get("raw_sensor_sample_count")
        == activity.get("environment_step_calls") * contract.RAW_SAMPLES_PER_STEP
        and in_closed_interval(
            "actor_fit_stage_calls",
            completed_fit,
            completed_fit + int(attempted_fit),
        )
        and in_closed_interval(
            "actor_updates", completed_fit, completed_fit + int(attempted_fit)
        )
        and activity.get("development_rollouts_completed")
        in {
            completed_development,
            completed_development + int(attempted_development),
        }
        and activity.get("critic_updates") == 0
        and activity.get("ppo_updates") == 0
        and ledger.get("actor_updates") == activity.get("actor_updates")
        and ledger.get("critic_updates") == activity.get("critic_updates")
        and ledger.get("ppo_updates") == activity.get("ppo_updates")
        and (
            (
                completed_fit == 1
                and activity.get("actor_fit_stage_calls") == 1
                and activity.get("actor_updates") == 1
                and activity.get("adamw_epochs_completed") == 2000
                and type(activity.get("lbfgs_closure_calls")) is int
                and 1 <= activity["lbfgs_closure_calls"] <= 600
            )
            or (
                completed_fit == 0
                and not attempted_fit
                and activity.get("actor_fit_stage_calls") == 0
                and activity.get("actor_updates") == 0
                and activity.get("adamw_epochs_completed") == 0
                and activity.get("lbfgs_closure_calls") == 0
            )
            or (
                completed_fit == 0
                and attempted_fit
                and type(activity.get("adamw_epochs_completed")) is int
                and 0 <= activity["adamw_epochs_completed"] <= 2000
                and type(activity.get("lbfgs_closure_calls")) is int
                and 0 <= activity["lbfgs_closure_calls"] <= 600
                and (
                    activity["lbfgs_closure_calls"] == 0
                    or activity["adamw_epochs_completed"] == 2000
                )
                and (
                    activity["actor_updates"] == 0
                    or activity["actor_fit_stage_calls"] == 1
                )
                and (
                    activity["adamw_epochs_completed"] == 0
                    or activity["actor_updates"] == 1
                )
            )
        )
    )


def verify_terminal_ledger(
    *, fit_verifier: Callable[[], Mapping[str, Any]] | None = None
) -> dict[str, Any]:
    freezer.verify_protocol_freeze()
    freezer.verify_execution_lock(require_pristine=False)
    locked_inputs_before = freezer.attest_locked_inputs()
    verify_pipeline_claim()
    ledger = _mapping(contract.LEDGER_PATH)
    if set(ledger) != TERMINAL_LEDGER_FIELDS:
        raise V12R9ExecutionError("terminal ledger schema drifted")
    if type(ledger.get("passed")) is not bool:
        raise V12R9ExecutionError("terminal ledger passed flag drifted")
    passed = ledger["passed"]
    expected_status = (
        contract.PIPELINE_TERMINAL_PASS_STATUS
        if passed
        else "FAIL_H0_V12R9_RECOVERY_PIPELINE_TERMINAL"
    )
    candidate, candidate_id, candidate_observation = _candidate_module_snapshot()
    expected_candidate_freeze = (
        _record(contract.CANDIDATE_FREEZE_PATH)
        if _path(contract.CANDIDATE_FREEZE_PATH).is_file()
        else None
    )
    expected_final_development = (
        _record(contract.FINAL_DEVELOPMENT_PATH)
        if _path(contract.FINAL_DEVELOPMENT_PATH).is_file()
        else None
    )
    checks = {
        "identity": type(ledger.get("schema_version")) is int
        and ledger.get("schema_version") == contract.SCHEMA_VERSION
        and ledger.get("status") == expected_status
        and ledger.get("terminal") is True
        and ledger.get("protocol_id") == contract.PROTOCOL_ID
        and ledger.get("pipeline_id") == contract.PIPELINE_ID,
        "selection_and_order": ledger.get("candidate_selection_rule")
        == contract.CANDIDATE_SELECTION_RULE
        and ledger.get("stage_order") == list(contract.STAGE_IDS),
        "candidate_snapshot": _strict_equal(ledger.get("candidate_id"), candidate_id)
        and _strict_equal(ledger.get("candidate_module"), candidate)
        and _strict_equal(
            ledger.get("candidate_module_observation"), candidate_observation
        ),
        "bindings": ledger.get("protocol_freeze")
        == _record(contract.PROTOCOL_FREEZE_PATH)
        and ledger.get("execution_lock") == _record(contract.EXECUTION_LOCK_PATH)
        and ledger.get("pipeline_claim") == _record(contract.CLAIM_PATH)
        and _strict_equal(ledger.get("candidate_freeze"), expected_candidate_freeze)
        and _strict_equal(
            ledger.get("final_development_receipt"), expected_final_development
        ),
        "closed": ledger.get("retry_authorized") is False
        and ledger.get("resume_authorized") is False
        and ledger.get("alpha_sweep_authorized") is False
        and ledger.get("runtime_promoted") is False
        and ledger.get("qualification_executed") is False
        and ledger.get("checkpoint_zero_created") is False
        and ledger.get("positive_morphology_enabled") is False
        and ledger.get("q3_paths_opened") == [],
        "next_stage": ledger.get("next_stage")
        == ("WAIT_SEPARATE_V12R9Q3_PROTOCOL" if passed else "STOP_TERMINAL_NO_RETRY"),
        "counter_types": all(
            type(ledger.get(name)) is int and ledger[name] >= 0
            for name in (
                "collection_round_count",
                "actor_fit_count",
                "development_count",
                "actor_updates",
                "critic_updates",
                "ppo_updates",
            )
        ),
        "activity_schema": isinstance(ledger.get("activity_totals"), Mapping)
        and set(ledger["activity_totals"]) == set(ACTIVITY_FIELDS)
        and all(
            type(ledger["activity_totals"].get(name)) is int
            and ledger["activity_totals"][name] >= 0
            for name in ACTIVITY_FIELDS
        ),
        "activity_aliases": isinstance(ledger.get("activity_totals"), Mapping)
        and all(
            type(ledger.get(name)) is int
            and ledger.get(name) == ledger["activity_totals"].get(name)
            for name in (
                "environment_reset_calls",
                "environment_step_calls",
                "historical_prefix_environment_reset_calls",
                "historical_prefix_environment_step_calls",
                "new_collection_environment_reset_calls",
                "new_collection_environment_step_calls",
                "development_environment_reset_calls",
                "development_environment_step_calls",
            )
        ),
    }
    completed = ledger.get("completed_stages")
    count = ledger.get("completed_stage_count")
    if (
        not isinstance(completed, list)
        or type(count) is not int
        or count != len(completed)
    ):
        raise V12R9ExecutionError("terminal completed-stage prefix drifted")
    selected_fit_verifier = (
        fitter.verify_fit_stage if fit_verifier is None else fit_verifier
    )
    cached_fit: dict[str, Any] | None = None

    def verify_fit_once() -> dict[str, Any]:
        nonlocal cached_fit
        if cached_fit is None:
            cached_fit = dict(selected_fit_verifier())
        return copy.deepcopy(cached_fit)

    completed_ids: list[str] = []
    for index, item in enumerate(completed):
        expected_stage = contract.STAGE_IDS[index]
        if (
            not isinstance(item, Mapping)
            or item.get("stage_id") != expected_stage
            or item.get("receipt") != _record(_stage_receipt_path(expected_stage))
        ):
            raise V12R9ExecutionError("terminal stage receipt binding drifted")
        _verify_completed_stage_semantics(expected_stage, fit_verifier=verify_fit_once)
        completed_ids.append(expected_stage)
    if not all(value is True for value in checks.values()):
        failed = sorted(name for name, value in checks.items() if not value)
        raise V12R9ExecutionError(f"terminal ledger drifted: {failed}")
    if passed:
        if (
            len(completed) != len(contract.STAGE_IDS)
            or ledger.get("attempted_stage") is not None
            or ledger.get("error") is not None
            or type(ledger.get("collection_round_count")) is not int
            or ledger.get("collection_round_count") != len(contract.COLLECTION_CASE_IDS)
            or type(ledger.get("actor_fit_count")) is not int
            or ledger.get("actor_fit_count") != 1
            or type(ledger.get("development_count")) is not int
            or ledger.get("development_count") != len(contract.DEVELOPMENT_CASE_IDS)
            or ledger.get("actor_updates") != 1
            or ledger.get("critic_updates") != 0
            or ledger.get("ppo_updates") != 0
            or ledger.get("historical_prefix_environment_reset_calls") != 0
            or ledger.get("historical_prefix_environment_step_calls") != 0
            or ledger.get("new_collection_environment_reset_calls")
            != len(contract.NEW_COLLECTION_CASE_IDS)
            or ledger.get("development_environment_reset_calls")
            != len(contract.DEVELOPMENT_CASE_IDS)
            or ledger.get("environment_reset_calls")
            != len(contract.NEW_COLLECTION_CASE_IDS)
            + len(contract.DEVELOPMENT_CASE_IDS)
            or ledger.get("environment_step_calls")
            != ledger.get("new_collection_environment_step_calls")
            + ledger.get("development_environment_step_calls")
            or not _prefix_activity_coherent(
                ledger,
                completed_ids=completed_ids,
                attempted_stage=None,
            )
            or ledger.get("candidate_freeze") != _record(contract.CANDIDATE_FREEZE_PATH)
            or ledger.get("final_development_receipt")
            != _record(contract.FINAL_DEVELOPMENT_PATH)
        ):
            raise V12R9ExecutionError("terminal PASS closure drifted")
        verify_claim_directory(completed_ids)
        verify_candidate_freeze_receipt(fit_verifier=verify_fit_once)
        verify_final_development_receipt(fit_verifier=verify_fit_once)
    else:
        attempted = ledger.get("attempted_stage")
        error = ledger.get("error")
        proper_prefix = len(completed_ids) < len(contract.STAGE_IDS)
        expected_attempted = (
            contract.STAGE_IDS[len(completed_ids)] if proper_prefix else None
        )
        strict_error = (
            isinstance(error, Mapping)
            and set(error) == {"type", "message"}
            and isinstance(error.get("type"), str)
            and bool(error["type"])
            and isinstance(error.get("message"), str)
            and bool(error["message"])
        )
        if (
            not proper_prefix
            or attempted != expected_attempted
            or not strict_error
            or not _prefix_activity_coherent(
                ledger,
                completed_ids=completed_ids,
                attempted_stage=attempted,
            )
        ):
            raise V12R9ExecutionError("terminal FAIL forensic prefix drifted")
        assert expected_attempted is not None
        verify_claim_directory([*completed_ids, expected_attempted])
    if not _strict_equal(freezer.attest_locked_inputs(), locked_inputs_before):
        raise V12R9ExecutionError("locked inputs changed during terminal verification")
    return ledger


def _stage_receipt_path(stage_id: str) -> Path:
    if stage_id == "adjudicate_r8_terminal_and_minus_prefix":
        return _path(contract.R8_MINUS_ADJUDICATION_STAGE_RECEIPT_PATH)
    if stage_id == "import_r8_plus_labels":
        return _path(contract.R8_PLUS_IMPORT_STAGE_RECEIPT_PATH)
    if stage_id == "label_r8_minus_prefix":
        return _path(contract.R8_MINUS_LABEL_STAGE_RECEIPT_PATH)
    if stage_id.startswith("collect_label__"):
        return _collection_stage_receipt_path(stage_id.removeprefix("collect_label__"))
    if stage_id == "fit_recovery_actor":
        return _path(contract.FIT_RECEIPT_PATH)
    if stage_id == "freeze_recovery_actor":
        return _path(contract.CANDIDATE_FREEZE_PATH)
    if stage_id.startswith("development__"):
        return _development_receipt_path(stage_id.removeprefix("development__"))
    if stage_id == "finalize_development":
        return _path(contract.FINAL_DEVELOPMENT_PATH)
    raise V12R9ExecutionError(f"unknown stage: {stage_id}")


def execute(*, adapters: RunnerAdapters | None = None) -> dict[str, Any]:
    selected = adapters or RunnerAdapters()
    ledger_path = _path(contract.LEDGER_PATH)
    if os.path.lexists(ledger_path):
        return verify_terminal_ledger(fit_verifier=selected.verify_fit)
    freezer.verify_protocol_freeze()
    freezer.verify_execution_lock(require_pristine=True)
    run_root = _path(contract.RUN_ROOT)
    if os.path.lexists(run_root):
        raise V12R9ExecutionError("R9 run root occupied; resume is forbidden")
    os.mkdir(run_root, 0o700)
    os.mkdir(_claims_root(), 0o700)
    claim = _expected_pipeline_claim()
    _write(contract.CLAIM_PATH, claim)
    activity = _zero_activity()
    completed: list[dict[str, Any]] = []
    attempted_stage: str | None = None
    error: BaseException | None = None
    try:
        for stage_id in contract.STAGE_IDS:
            attempted_stage = stage_id
            _claim_stage(stage_id)
            if stage_id == "adjudicate_r8_terminal_and_minus_prefix":
                _run_r8_terminal_adjudication(selected)
            elif stage_id == "import_r8_plus_labels":
                _run_plus_label_import(activity, selected)
            elif stage_id == "label_r8_minus_prefix":
                _run_imported_minus_label(activity, selected)
            elif stage_id.startswith("collect_label__"):
                _run_collection_label(
                    stage_id.removeprefix("collect_label__"), activity, selected
                )
            elif stage_id == "fit_recovery_actor":
                _run_fit(activity, selected)
            elif stage_id == "freeze_recovery_actor":
                _run_candidate_freeze(selected)
            elif stage_id.startswith("development__"):
                _run_development(
                    stage_id.removeprefix("development__"), activity, selected
                )
            elif stage_id == "finalize_development":
                _run_final_development(activity, selected)
            else:  # pragma: no cover - contract self-check owns this branch.
                raise V12R9ExecutionError(f"unknown stage: {stage_id}")
            receipt_path = _stage_receipt_path(stage_id)
            completed.append({"stage_id": stage_id, "receipt": _record(receipt_path)})
            attempted_stage = None
    except BaseException as exc:
        error = exc
    passed = error is None and len(completed) == len(contract.STAGE_IDS)
    terminal = _terminal_payload(
        passed=passed,
        completed=completed,
        attempted_stage=attempted_stage,
        activity=activity,
        error=error,
    )
    _write(contract.LEDGER_PATH, terminal)
    verified = verify_terminal_ledger(fit_verifier=selected.verify_fit)
    if not passed:
        assert error is not None
        raise V12R9ExecutionError(
            f"V12R9 stopped terminally at {attempted_stage}: {error}"
        ) from error
    return verified


def preflight() -> dict[str, Any]:
    freezer.verify_protocol_freeze()
    freezer.verify_execution_lock(require_pristine=True)
    locked = freezer.attest_locked_inputs()
    label_paths_match = all(
        fitter.observer_labels_path(case_id) == contract.observer_label_path(case_id)
        for case_id in contract.COLLECTION_CASE_IDS
    )
    if not label_paths_match:
        raise V12R9ExecutionError("fitter/probe observer label path contract drifted")
    return {
        "passed": True,
        "status": "READY_H0_V12R9_RECOVERY_ONE_SHOT",
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_order": list(contract.STAGE_IDS),
        "locked_inputs": locked,
        "source_count": len(freezer.production_source_closure()),
        "runtime": freezer.verify_execution_lock(require_pristine=True)["runtime"],
        "run_root_absent": not os.path.lexists(_path(contract.RUN_ROOT)),
        "single_collection_round": True,
        "single_actor_fit": True,
        "r8_retry_or_resume_authorized": False,
        "imported_prefix_environment_calls": {
            "reset": 0,
            "step": 0,
        },
        "new_collection_rollout_count": len(contract.NEW_COLLECTION_CASE_IDS),
        "development_rollout_count": len(contract.DEVELOPMENT_CASE_IDS),
        "qualification_execution_authorized": False,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--preflight", action="store_true")
    mode.add_argument("--execute", action="store_true")
    mode.add_argument("--verify", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.preflight:
        result = preflight()
    elif args.execute:
        result = execute()
    else:
        result = verify_terminal_ledger()
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    # The Python API deliberately remains idempotent for an already-terminal
    # ledger.  The CLI, however, must never translate a verified terminal FAIL
    # into process success: automation relies on its exit status.
    return 0 if result.get("passed") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "RunnerAdapters",
    "V12R9ExecutionError",
    "execute",
    "preflight",
    "verify_candidate_freeze_receipt",
    "verify_collection_label_receipt",
    "verify_development_receipt",
    "verify_final_development_receipt",
    "verify_imported_minus_label_receipt",
    "verify_plus_label_import_receipt",
    "verify_r8_terminal_adjudication_stage_receipt",
    "verify_terminal_ledger",
]
