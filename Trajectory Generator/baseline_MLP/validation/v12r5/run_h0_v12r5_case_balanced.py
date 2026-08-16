"""Execute the additive V12R5 case-balanced pipeline exactly once.

There is no environment collection stage or collection execution surface.
The runner attests immutable inputs, assembles P2 plus the sole reusable R4
nominal labels, performs one fresh-H0 fit, freezes one candidate, and executes
six pure development rollouts in critical ``+0.20``-first order.  Historical
Q2 and future Q3 execution outputs remain unopened throughout.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import math
import os
import platform
import secrets
import stat
import sys
import time
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
LOCAL_VALIDATION = REPO_ROOT / "Trajectory Generator" / "baseline_MLP" / "validation"
REVISION_ROOT = Path(__file__).resolve().parent
for _root in (
    REPO_ROOT,
    REPO_ROOT / "validation",
    REPO_ROOT / "Trajectory Generator",
    REPO_ROOT / "Trajectory Generator" / "baseline_MLP",
    LOCAL_VALIDATION,
    LOCAL_VALIDATION / "v12r3",
    REVISION_ROOT,
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_forensic_rollout as forensic  # noqa: E402
import run_h0_primary_split_v9_causal_teacher as env_source  # noqa: E402
import run_h0_primary_split_v12r3_autonomy_recovery as runtime  # noqa: E402
import h0_v12r5_case_balanced_contract as contract  # noqa: E402
import h0_v12r5_case_balanced_fitter as fit_engine  # noqa: E402
import freeze_h0_v12r5_case_balanced as freezer  # noqa: E402


class V12R5ExecutionError(RuntimeError):
    """Raised when continuing would violate the frozen one-shot protocol."""


def resolve_relative(path: str | os.PathLike[str] | PurePosixPath) -> Path:
    raw = path.as_posix() if isinstance(path, PurePosixPath) else os.fspath(path)
    pure = PurePosixPath(raw)
    if not raw or pure.is_absolute() or ".." in pure.parts or pure.as_posix() != raw:
        raise V12R5ExecutionError(f"non-canonical repository path: {raw!r}")
    return REPO_ROOT.joinpath(*pure.parts)


RUN_ROOT = resolve_relative(contract.RUN_ROOT)
LOCK_PATH = resolve_relative(contract.EXECUTION_LOCK_PATH)
LOCK_PUBLICATION_FAILURE_PATH = resolve_relative(
    contract.EXECUTION_LOCK_PUBLICATION_FAILURE_PATH
)
PROTOCOL_FREEZE_PATH = resolve_relative(contract.PROTOCOL_FREEZE_PATH)
DESIGN_AUDIT_PATH = resolve_relative(contract.DESIGN_AUDIT_PATH)
PIPELINE_CLAIM_PATH = resolve_relative(contract.PIPELINE_CLAIM_PATH)
PIPELINE_CLAIM_FAILURE_PATH = resolve_relative(contract.PIPELINE_CLAIM_FAILURE_PATH)
PIPELINE_LEDGER_PATH = resolve_relative(contract.PIPELINE_LEDGER_PATH)
PIPELINE_TERMINAL_PUBLICATION_FAILURE_PATH = resolve_relative(
    contract.PIPELINE_TERMINAL_PUBLICATION_FAILURE_PATH
)
WORKER_CLAIMS_ROOT = resolve_relative(contract.WORKER_CLAIMS_ROOT)
CANDIDATE_FREEZE_PATH = resolve_relative(contract.CANDIDATE_FREEZE_PATH)
SOURCE_H0_MODULE = resolve_relative(contract.SOURCE_H0_MODULE_PATH).resolve()

_ACTIVITY_NAMES = (
    "environment_reset_calls",
    "environment_step_calls",
    "raw_sensor_sample_count",
    "teacher_query_count",
    "actor_fit_stage_calls_attempted",
    "actor_fit_executions_confirmed",
    "actor_updates_attempted",
    "actor_updates_confirmed",
    "adamw_epochs_completed",
    "lbfgs_closure_calls",
)
_EXECUTION_LOCK_FIELDS = frozenset(
    {
        "schema_version",
        "status",
        "passed",
        "protocol_id",
        "pipeline_id",
        "revision",
        "authority",
        "checks",
        "occupancy",
        "protocol_freeze",
        "design_audit",
        "q2_design_freeze",
        "q3_design_freeze",
        "locked_inputs",
        "source_h0",
        "candidate_selection",
        "stage_order",
        "run_root",
        "platform",
        "one_shot",
        "retry_authorized",
        "resume_authorized",
        "rescue_authorized",
        "sweep_authorized",
        "qualification_execution_authorized",
        "environment_reset_calls",
        "environment_step_calls",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
    }
)
_EXECUTION_LOCK_CHECK_FIELDS = frozenset(
    {
        "protocol",
        "design_audit",
        "q3_design",
        "contract",
        "source_h0",
        "external_runtime_sources",
        "production_source_closure",
        "occupancy",
        "authority",
    }
)
_EXECUTION_LOCK_OCCUPANCY_FIELDS = frozenset(
    {
        "lock_absent",
        "lock_publication_failure_absent",
        "run_root_absent",
        "pipeline_claim_absent",
        "pipeline_claim_failure_absent",
        "pipeline_ledger_absent",
        "terminal_publication_failure_absent",
        "qualification_unopened",
    }
)
_EXECUTION_LOCK_PUBLICATION_FAILURE_FIELDS = frozenset(
    {
        "schema_version",
        "status",
        "passed",
        "terminal",
        "protocol_id",
        "pipeline_id",
        "phase",
        "expected_execution_lock",
        "expected_execution_lock_record",
        "execution_lock_observation",
        "protocol_freeze",
        "design_audit",
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
_PLATFORM_FIELDS = frozenset({"system", "machine", "python", "executable"})
_PIPELINE_CLAIM_FIELDS = frozenset(
    {
        "schema_version",
        "status",
        "passed",
        "protocol_id",
        "pipeline_id",
        "authority",
        "one_shot",
        "execution_token_sha256",
        "execution_lock",
        "protocol_freeze",
        "design_audit",
        "stage_order",
        "retry_authorized",
        "resume_authorized",
        "rescue_authorized",
        "sweep_authorized",
        "qualification_execution_authorized",
    }
)
_WORKER_CLAIM_FIELDS = frozenset(
    {
        "schema_version",
        "status",
        "passed",
        "protocol_id",
        "pipeline_id",
        "authority",
        "one_shot",
        "stage_id",
        "stage_index",
        "stage_kind",
        "execution_token_sha256",
        "pipeline_claim",
        "previous_receipts",
        "retry_authorized",
        "resume_authorized",
        "rescue_authorized",
        "sweep_authorized",
        "qualification_execution_authorized",
    }
)
_ATTESTATION_RECEIPT_FIELDS = frozenset(
    {
        "schema_version",
        "status",
        "passed",
        "protocol_id",
        "stage_id",
        "locked_input_attestation",
        "failed_plus_prefix_rows_loaded",
        "new_collection_count",
        "pipeline_claim",
        "worker_claim",
        "q2_paths_opened",
        "q3_paths_opened",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
    }
)
_CORPUS_RECEIPT_FIELDS = frozenset(
    {
        "schema_version",
        "status",
        "passed",
        "protocol_id",
        "stage_id",
        "fit_counts",
        "corpus_audit",
        "source_records",
        "pipeline_claim",
        "worker_claim",
        "q2_paths_opened",
        "q3_paths_opened",
        "new_collection_count",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
    }
)
_FIT_RECEIPT_FIELDS = frozenset(
    {
        "schema_version",
        "status",
        "passed",
        "protocol_id",
        "candidate_selection_rule",
        "candidate_id",
        "candidate_module",
        "summary",
        "gate",
        "corpus",
        "adaptation_report",
        "adaptation_history",
        "pipeline_claim",
        "worker_claim",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
    }
)
_CANDIDATE_FREEZE_SUMMARY_FIELDS = frozenset(
    {
        "schema_version",
        "status",
        "protocol_id",
        "candidate_selection_rule",
        "candidate_id",
        "candidate_module",
        "fit_receipt",
        "fit_passed",
        "candidate_frozen",
        "source_h0_byte_exact",
        "logstd_byte_exact",
        "critic_present",
        "save_reload_exact",
        "pipeline_claim",
        "worker_claim",
        "q2_paths_opened",
        "q3_paths_opened",
        "runtime_promoted",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
    }
)
_CANDIDATE_FREEZE_RECEIPT_FIELDS = frozenset(
    {
        *_CANDIDATE_FREEZE_SUMMARY_FIELDS,
        "passed",
        "stage_id",
        "summary",
        "gate",
    }
)
_DEVELOPMENT_SUMMARY_FIELDS = frozenset(
    {
        "action_clipped_values",
        "action_seed",
        "action_selection",
        "actor_updates",
        "binary_event_failure_count",
        "binary_event_prefix_integrity",
        "binary_phase_event_gate",
        "binary_phase_fsm_mode",
        "blending_enabled",
        "candidate_freeze",
        "candidate_id",
        "candidate_module",
        "case",
        "case_id",
        "control_window_count",
        "critic_updates",
        "detector_or_fsm_modified",
        "end_reason",
        "episode_metrics",
        "episode_start_offset_s",
        "event_contract_id",
        "fallback_count",
        "grf_penetration_max_m",
        "hard_invalid_count",
        "invalid_event_count",
        "morphology_weight",
        "n_actor",
        "n_observation",
        "nonfinite_count",
        "observation_dtype",
        "phase_valid_cycle_count",
        "ppo_updates",
        "protected_trials_opened",
        "protocol_id",
        "pure_policy_trace_audit",
        "pure_policy_trace_row_count",
        "q2_paths_opened",
        "q3_paths_opened",
        "random_noise_draw_count",
        "raw_sensor_sample_count",
        "reserve_trials_opened",
        "routing_failure_count",
        "runtime_seed",
        "safety_latch_enabled",
        "safety_stop_count",
        "schema_version",
        "sea_episode_metrics",
        "sea_plugin_fallback_count",
        "sea_reserve_gate_passed",
        "sigma",
        "single_noise_application_count",
        "so_solver_unaccepted_count",
        "status",
        "step_contract_failure_count",
        "steps",
        "target_contract_id",
        "teacher_enabled",
        "terminated",
        "timeout_count",
        "truncated",
        *contract.PURE_POLICY_COUNTER_FIELDS,
    }
)
_FINAL_DEVELOPMENT_SUMMARY_FIELDS = frozenset(
    {
        "schema_version",
        "status",
        "protocol_id",
        "candidate_selection_rule",
        "candidate_id",
        "candidate_module",
        "candidate_freeze",
        "case_gates",
        "rollout_bindings",
        "candidate_tree_unique_count",
        "new_collection_count",
        "development_count",
        "environment_reset_calls",
        "environment_step_calls",
        "raw_sensor_sample_count",
        "teacher_query_count",
        "pure_policy_trace_row_count",
        *contract.PURE_POLICY_COUNTER_FIELDS,
        "actor_updates",
        "critic_updates",
        "ppo_updates",
        "retry_authorized",
        "resume_authorized",
        "rescue_authorized",
        "q2_paths_opened",
        "q3_paths_opened",
        "runtime_promoted",
        "checkpoint_zero_created",
        "positive_morphology_enabled",
        "pipeline_claim",
        "worker_claim",
    }
)
_DEVELOPMENT_RECEIPT_FIELDS = frozenset(
    {
        "schema_version",
        "status",
        "passed",
        "protocol_id",
        "stage_id",
        "case_id",
        "candidate_id",
        "candidate_module",
        "candidate_freeze",
        "summary",
        "gate",
        "trace",
        "pipeline_claim",
        "worker_claim",
        "pure_policy_trace_audit",
        "pure_policy_trace_row_count",
        *contract.PURE_POLICY_COUNTER_FIELDS,
        "actor_updates",
        "critic_updates",
        "ppo_updates",
    }
)
_FINAL_RECEIPT_FIELDS = frozenset(
    {
        "schema_version",
        "status",
        "passed",
        "protocol_id",
        "stage_id",
        "candidate_selection_rule",
        "candidate_id",
        "candidate_module",
        "candidate_freeze",
        "summary",
        "gate",
        "rollout_bindings",
        "pipeline_claim",
        "worker_claim",
        "development_only",
        "qualification_executed",
        "runtime_promoted",
        "checkpoint_zero_created",
        "positive_morphology_enabled",
        "retry_authorized",
        "resume_authorized",
        "rescue_authorized",
        "sweep_authorized",
        "pure_policy_trace_row_count",
        *contract.PURE_POLICY_COUNTER_FIELDS,
        "actor_updates",
        "critic_updates",
        "ppo_updates",
    }
)
_DEVELOPMENT_FAILURE_FIELDS = frozenset(
    {
        "schema_version",
        "status",
        "passed",
        "terminal",
        "protocol_id",
        "pipeline_id",
        "phase",
        "stage_id",
        "case_id",
        "end_reason",
        "error",
        "worker_claim_observation",
        "pipeline_claim",
        "execution_lock",
        "protocol_freeze",
        "visible_prefix",
        "q2_paths_opened",
        "q3_paths_opened",
        "qualification_violation_detected",
        "new_collection_count",
        "stage_actor_updates",
        "critic_updates",
        "ppo_updates",
        "retry_authorized",
        "resume_authorized",
        "rescue_authorized",
        "sweep_authorized",
        "qualification_execution_authorized",
        "runtime_promoted",
        "checkpoint_zero_created",
        "positive_morphology_enabled",
        "next_stage",
    }
)
_PIPELINE_CLAIM_FAILURE_FIELDS = frozenset(
    {
        "schema_version",
        "status",
        "passed",
        "terminal",
        "protocol_id",
        "pipeline_id",
        "phase",
        "run_root",
        "run_root_claimed",
        "pipeline_claim_observation",
        "worker_claims_root_state",
        "execution_lock",
        "protocol_freeze",
        "q2_paths_opened",
        "q3_paths_opened",
        "qualification_violation_detected",
        "error",
        "activity_totals",
        "stage_activity",
        "new_collection_count",
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
_TERMINAL_PUBLICATION_FAILURE_FIELDS = frozenset(
    {
        "schema_version",
        "status",
        "passed",
        "terminal",
        "protocol_id",
        "pipeline_id",
        "phase",
        "intended_outcome",
        "terminal_intent",
        "terminal_intent_valid",
        "intended_ledger",
        "intended_ledger_record",
        "pipeline_ledger_observation",
        "completed_stages",
        "completed_stage_count",
        "attempted_stage",
        "error",
        "execution_lock",
        "execution_lock_observation",
        "protocol_freeze",
        "protocol_freeze_observation",
        "pipeline_claim",
        "pipeline_claim_observation",
        "activity_totals",
        "stage_activity",
        "q2_paths_opened",
        "q3_paths_opened",
        "qualification_violation_detected",
        "preterminal_run_root_inventory",
        "new_collection_count",
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
_TERMINAL_INTENT_FIELDS = frozenset(
    {
        "schema_version",
        "protocol_id",
        "pipeline_id",
        "intended_outcome",
        "completed_stages",
        "completed_stage_count",
        "attempted_stage",
        "stage_error",
        "failure_publication_error",
        "activity_totals",
        "stage_activity",
        "next_stage",
    }
)
_TERMINAL_LEDGER_FIELDS = frozenset(
    {
        "schema_version",
        "status",
        "passed",
        "terminal",
        "protocol_id",
        "pipeline_id",
        "attempted_stage",
        "completed_stages",
        "completed_stage_count",
        "stage_order",
        "candidate_selection_rule",
        "candidate_id",
        "candidate_module",
        "candidate_freeze",
        "candidate_freeze_observation",
        "final_development_receipt",
        "protocol_freeze",
        "execution_lock",
        "pipeline_claim",
        "q2_paths_opened",
        "q3_paths_opened",
        "qualification_violation_detected",
        "preterminal_run_root_inventory",
        "error",
        "activity_totals",
        "stage_activity",
        "attempted_stage_output_observations",
        "attempted_stage_worker_claim_observation",
        "attempted_stage_worker_claim",
        "attempted_stage_worker_claim_created",
        "attempted_stage_receipt",
        "attempted_stage_receipt_created",
        "attempted_stage_failure_observation",
        "attempted_stage_failure_artifact",
        "attempted_stage_failure_publication_error",
        "new_collection_count",
        "development_count",
        "environment_reset_calls",
        "environment_step_calls",
        "raw_sensor_sample_count",
        "teacher_query_count",
        "actor_fit_stage_calls_attempted",
        "actor_fit_executions_confirmed",
        "actor_updates_attempted",
        "actor_updates",
        "adamw_epochs_completed",
        "lbfgs_closure_calls",
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
_PRETERMINAL_RUN_ROOT_INVENTORY_FIELDS = frozenset(
    {
        "run_root",
        "excluded_relative_paths",
        "semantic_allowlist_id",
        "root_state",
        "complete",
        "semantic_closed",
        "unexpected_relative_paths",
        "entries",
    }
)
_PRETERMINAL_RUN_ROOT_INVENTORY_ENTRY_FIELDS = frozenset(
    {"relative_path", "state", "artifact"}
)
_PRETERMINAL_RUN_ROOT_EXCLUSIONS = (
    PIPELINE_LEDGER_PATH.name,
    PIPELINE_TERMINAL_PUBLICATION_FAILURE_PATH.name,
)
_PRETERMINAL_RUN_ROOT_SEMANTIC_ALLOWLIST_ID = "H0_V12R5_PRETERMINAL_EXACT_PATHS_V1"
_PRETERMINAL_RUN_ROOT_ENTRY_STATES = frozenset(
    {
        "DIRECTORY",
        "REGULAR_FILE",
        "UNSAFE_SYMLINK",
        "UNSAFE_SPECIAL",
        "UNREADABLE_DIRECTORY",
        "UNREADABLE_REGULAR",
        "VANISHED_DURING_SCAN",
    }
)
_ACTIVITY_TOTALS = {name: 0 for name in _ACTIVITY_NAMES}
_STAGE_ACTIVITY: dict[str, dict[str, Any]] = {}
_ACTIVE_STAGE_ID: str | None = None


def _execution_lock_schema_exact(value: Any) -> bool:
    if not isinstance(value, Mapping) or set(value) != _EXECUTION_LOCK_FIELDS:
        return False
    checks = value.get("checks")
    occupancy = value.get("occupancy")
    platform_record = value.get("platform")
    return (
        isinstance(checks, Mapping)
        and set(checks) == _EXECUTION_LOCK_CHECK_FIELDS
        and isinstance(occupancy, Mapping)
        and set(occupancy) == _EXECUTION_LOCK_OCCUPANCY_FIELDS
        and isinstance(platform_record, Mapping)
        and set(platform_record) == _PLATFORM_FIELDS
    )


def _terminal_ledger_schema_exact(value: Any) -> bool:
    return isinstance(value, Mapping) and set(value) == _TERMINAL_LEDGER_FIELDS


def _reset_activity() -> None:
    global _ACTIVE_STAGE_ID
    _ACTIVE_STAGE_ID = None
    _STAGE_ACTIVITY.clear()
    for name in _ACTIVITY_NAMES:
        _ACTIVITY_TOTALS[name] = 0


def _begin_stage_activity(stage_id: str) -> None:
    global _ACTIVE_STAGE_ID
    if stage_id in _STAGE_ACTIVITY:
        raise V12R5ExecutionError(f"stage activity already opened: {stage_id}")
    _ACTIVE_STAGE_ID = stage_id
    _STAGE_ACTIVITY[stage_id] = {
        "stage_id": stage_id,
        "stage_kind": contract.stage_descriptor(stage_id)["kind"],
        **{name: 0 for name in _ACTIVITY_NAMES},
    }


def _activity_totals_snapshot() -> dict[str, int]:
    """Recompute totals from per-stage rows, the sole authoritative counters."""

    return {
        name: sum(int(row[name]) for row in _STAGE_ACTIVITY.values())
        for name in _ACTIVITY_NAMES
    }


def _activity_increment(name: str, amount: int = 1) -> None:
    if name not in _ACTIVITY_TOTALS:
        raise V12R5ExecutionError(f"unknown activity counter: {name}")
    if type(amount) is not int or amount < 0:
        raise V12R5ExecutionError("activity increment must be a nonnegative int")
    if _ACTIVE_STAGE_ID is None:
        raise V12R5ExecutionError("activity increment outside a claimed stage")
    _STAGE_ACTIVITY[_ACTIVE_STAGE_ID][name] += amount
    # Compatibility cache only.  Terminal evidence always recomputes from the
    # stage rows, so interruption between these assignments cannot forge or
    # invalidate the authoritative totals.
    _ACTIVITY_TOTALS[name] = sum(int(row[name]) for row in _STAGE_ACTIVITY.values())


def _mapping(path: str | Path) -> dict[str, Any]:
    try:
        value = forensic.strict_json_load(Path(path))
    except Exception as exc:
        raise V12R5ExecutionError(f"invalid strict JSON object: {path}") from exc
    if not isinstance(value, Mapping):
        raise V12R5ExecutionError(f"expected JSON object: {path}")
    return dict(value)


def _strict_equal(left: Any, right: Any) -> bool:
    """Compare JSON-like values without Python's bool/int/float coercions."""

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


def _exact_int(value: Any, expected: int | None = None) -> bool:
    return type(value) is int and (expected is None or value == expected)


def _error_mapping_valid(value: Any) -> bool:
    return (
        isinstance(value, Mapping)
        and set(value) == {"type", "message"}
        and type(value.get("type")) is str
        and bool(value["type"])
        and type(value.get("message")) is str
        and bool(value["message"])
    )


def _sequence(path: str | Path) -> list[Any]:
    try:
        value = forensic.strict_json_load(Path(path))
    except Exception as exc:
        raise V12R5ExecutionError(f"invalid strict JSON array: {path}") from exc
    if not isinstance(value, list):
        raise V12R5ExecutionError(f"expected JSON array: {path}")
    return value


def _record(path: str | Path) -> dict[str, Any]:
    return forensic.artifact_record(Path(path), artifact_root=REPO_ROOT)


def _logical_artifact_path(path: str | Path) -> str:
    return Path(path).resolve().relative_to(REPO_ROOT.resolve()).as_posix()


def _artifact_record_schema_valid(
    value: Any, *, expected_path: str | Path | None = None
) -> bool:
    if not isinstance(value, Mapping) or set(value) != {
        "path",
        "sha256",
        "size_bytes",
    }:
        return False
    digest = value.get("sha256")
    if (
        type(value.get("path")) is not str
        or not value["path"]
        or type(digest) is not str
        or len(digest) != 64
        or any(character not in "0123456789abcdef" for character in digest)
        or type(value.get("size_bytes")) is not int
        or value["size_bytes"] < 0
    ):
        return False
    if expected_path is None:
        return True
    try:
        return value["path"] == _logical_artifact_path(expected_path)
    except BaseException:
        return False


def _canonical_payload_record(path: Path, payload: Any) -> dict[str, Any]:
    encoded = forensic.canonical_json_bytes(payload)
    return {
        "path": path.resolve().relative_to(REPO_ROOT).as_posix(),
        "sha256": hashlib.sha256(encoded).hexdigest(),
        "size_bytes": len(encoded),
    }


def _expected_json_observation(
    path: Path, *, expected_record: Mapping[str, Any]
) -> dict[str, Any]:
    if not os.path.lexists(path):
        return {"state": "ABSENT", "artifact": None}
    if path.is_symlink():
        return {"state": "UNSAFE_SYMLINK", "artifact": None}
    if not path.is_file():
        return {"state": "UNSAFE_NONREGULAR", "artifact": None}
    try:
        artifact = _record(path)
    except BaseException:
        return {"state": "UNSAFE_UNREADABLE_REGULAR", "artifact": None}
    if artifact["size_bytes"] == 0:
        state = "ZERO_BYTE_REGULAR"
    else:
        try:
            _mapping(path)
        except V12R5ExecutionError:
            state = "INVALID_JSON_REGULAR"
        else:
            state = (
                "VALID"
                if _strict_equal(artifact, dict(expected_record))
                else "INVALID_SCHEMA_REGULAR"
            )
    return {"state": state, "artifact": artifact}


def _pipeline_claim_payload_valid(payload: Any) -> bool:
    if not isinstance(payload, Mapping) or set(payload) != _PIPELINE_CLAIM_FIELDS:
        return False
    token_hash = payload.get("execution_token_sha256")
    token_valid = (
        isinstance(token_hash, str)
        and len(token_hash) == 64
        and all(character in "0123456789abcdef" for character in token_hash)
    )
    return (
        token_valid
        and payload.get("schema_version") == contract.SCHEMA_VERSION
        and type(payload.get("schema_version")) is type(contract.SCHEMA_VERSION)
        and payload.get("status") == "CLAIMED_H0_V12R5_CASE_BALANCED_PIPELINE_ONCE"
        and payload.get("passed") is True
        and payload.get("protocol_id") == contract.PROTOCOL_ID
        and payload.get("pipeline_id") == contract.PIPELINE_ID
        and _strict_equal(payload.get("authority"), contract.AUTHORITY)
        and payload.get("one_shot") is True
        and _strict_equal(payload.get("execution_lock"), _record(LOCK_PATH))
        and _strict_equal(payload.get("protocol_freeze"), _record(PROTOCOL_FREEZE_PATH))
        and _strict_equal(payload.get("design_audit"), _record(DESIGN_AUDIT_PATH))
        and _strict_equal(payload.get("stage_order"), list(contract.STAGE_IDS))
        and all(
            payload.get(name) is False
            for name in (
                "retry_authorized",
                "resume_authorized",
                "rescue_authorized",
                "sweep_authorized",
                "qualification_execution_authorized",
            )
        )
    )


def _expected_previous_receipts(stage_id: str) -> list[dict[str, Any]]:
    index = contract.STAGE_IDS.index(stage_id)
    rows: list[dict[str, Any]] = []
    for prior in contract.STAGE_IDS[:index]:
        receipt = _stage_receipt_path(prior)
        if not receipt.is_file():
            raise V12R5ExecutionError(f"prior stage receipt missing: {prior}")
        rows.append({"stage_id": prior, "receipt": _record(receipt)})
    return rows


def _worker_claim_payload_valid(payload: Any, *, stage_id: str) -> bool:
    if not isinstance(payload, Mapping) or set(payload) != _WORKER_CLAIM_FIELDS:
        return False
    try:
        pipeline_claim = _mapping(PIPELINE_CLAIM_PATH)
    except V12R5ExecutionError:
        return False
    return (
        _pipeline_claim_payload_valid(pipeline_claim)
        and payload.get("schema_version") == contract.SCHEMA_VERSION
        and type(payload.get("schema_version")) is type(contract.SCHEMA_VERSION)
        and payload.get("status") == "CLAIMED_H0_V12R5_CASE_BALANCED_STAGE_ONCE"
        and payload.get("passed") is True
        and payload.get("protocol_id") == contract.PROTOCOL_ID
        and payload.get("pipeline_id") == contract.PIPELINE_ID
        and _strict_equal(payload.get("authority"), contract.AUTHORITY)
        and payload.get("one_shot") is True
        and payload.get("stage_id") == stage_id
        and type(payload.get("stage_index")) is int
        and payload.get("stage_index") == contract.STAGE_IDS.index(stage_id)
        and payload.get("stage_kind") == contract.stage_descriptor(stage_id)["kind"]
        and payload.get("execution_token_sha256")
        == pipeline_claim.get("execution_token_sha256")
        and _strict_equal(payload.get("pipeline_claim"), _record(PIPELINE_CLAIM_PATH))
        and _strict_equal(
            payload.get("previous_receipts"), _expected_previous_receipts(stage_id)
        )
        and all(
            payload.get(name) is False
            for name in (
                "retry_authorized",
                "resume_authorized",
                "rescue_authorized",
                "sweep_authorized",
                "qualification_execution_authorized",
            )
        )
    )


def _worker_claim_observation(stage_id: str) -> dict[str, Any]:
    path = _claim_path(stage_id)
    if not os.path.lexists(path):
        return {"state": "ABSENT", "artifact": None}
    if path.is_symlink() or not path.is_file():
        return {"state": "UNSAFE_NONREGULAR", "artifact": None}
    try:
        artifact = _record(path)
    except BaseException:
        return {"state": "UNSAFE_UNREADABLE_REGULAR", "artifact": None}
    try:
        payload = _mapping(path)
        valid = _worker_claim_payload_valid(payload, stage_id=stage_id)
    except BaseException:
        state = "PARTIAL_OR_INVALID_REGULAR"
    else:
        state = "VALID_REGULAR" if valid else "PARTIAL_OR_INVALID_REGULAR"
    return {"state": state, "artifact": artifact}


def _verify_worker_claims_root_exact(
    *, completed_stage_ids: Sequence[str], attempted_stage: str | None
) -> None:
    """Reject every unbound worker-root entry, including links and directories."""

    if not WORKER_CLAIMS_ROOT.is_dir() or WORKER_CLAIMS_ROOT.is_symlink():
        raise V12R5ExecutionError("worker claims root is not a safe directory")
    expected_ids = list(completed_stage_ids)
    if (
        attempted_stage is not None
        and attempted_stage not in expected_ids
        and os.path.lexists(_claim_path(attempted_stage))
    ):
        expected_ids.append(attempted_stage)
    expected_names = {f"{stage_id}.json" for stage_id in expected_ids}
    entries = list(WORKER_CLAIMS_ROOT.iterdir())
    if {entry.name for entry in entries} != expected_names or any(
        entry.is_symlink() or not entry.is_file() for entry in entries
    ):
        raise V12R5ExecutionError("worker claims root closure drifted")


def _pipeline_claim_observation() -> dict[str, Any]:
    """Bind an absent, valid, or partial regular claim without parsing assumptions."""

    if not os.path.lexists(PIPELINE_CLAIM_PATH):
        return {"state": "ABSENT", "artifact": None}
    if PIPELINE_CLAIM_PATH.is_symlink() or not PIPELINE_CLAIM_PATH.is_file():
        return {"state": "UNSAFE_NONREGULAR", "artifact": None}
    try:
        artifact = _record(PIPELINE_CLAIM_PATH)
    except BaseException:
        return {"state": "UNSAFE_UNREADABLE_REGULAR", "artifact": None}
    try:
        payload = _mapping(PIPELINE_CLAIM_PATH)
        valid = _pipeline_claim_payload_valid(payload)
    except BaseException:
        state = "PARTIAL_OR_INVALID_REGULAR"
    else:
        state = "VALID_REGULAR" if valid else "PARTIAL_OR_INVALID_REGULAR"
    return {"state": state, "artifact": artifact}


def _worker_claims_root_state() -> str:
    if not os.path.lexists(WORKER_CLAIMS_ROOT):
        return "ABSENT"
    if WORKER_CLAIMS_ROOT.is_dir() and not WORKER_CLAIMS_ROOT.is_symlink():
        return "DIRECTORY"
    return "UNSAFE_NONDIRECTORY"


def _pipeline_claim_failure_schema_exact(value: Any) -> bool:
    return isinstance(value, Mapping) and set(value) == (_PIPELINE_CLAIM_FAILURE_FIELDS)


def _error_record(error: BaseException) -> dict[str, str]:
    message = str(error) or repr(error) or type(error).__name__
    return {"type": type(error).__name__, "message": message}


def _tree(path: str | Path) -> dict[str, Any]:
    target = Path(path)
    relative = target.relative_to(REPO_ROOT) if target.is_absolute() else target
    return freezer.tree_record(PurePosixPath(relative.as_posix()))


def _external_runtime_sources_exact() -> bool:
    direct = all(
        _record(record["path"]) == dict(record)
        for record in contract.FROZEN_EXTERNAL_RUNTIME_SOURCES.values()
    )
    safe_lock = (
        _record(contract.SAFE_V8R1P1_EXECUTION_LOCK_ARTIFACT["path"])
        == contract.SAFE_V8R1P1_EXECUTION_LOCK_ARTIFACT
    )
    production = freezer._production_source_closure()
    return (
        direct
        and safe_lock
        and len(freezer._safe_environment_source_closure()) == 29
        and len(production) == contract.EXPECTED_PRODUCTION_SOURCE_COUNT
        and freezer._production_source_closure_exact(production)
    )


def _locked_input_snapshot() -> dict[str, Any]:
    return {
        "p2_corpus": copy.deepcopy(contract.P2_CORPUS_ARTIFACT),
        "p2_module": copy.deepcopy(contract.P2_MODULE_TREE),
        "r4_terminal": copy.deepcopy(contract.R4_TERMINAL_ARTIFACTS),
        "r4_nominal_pass": copy.deepcopy(contract.R4_NOMINAL_REUSABLE_ARTIFACTS),
        "r4_plus_failure_forensic_only": copy.deepcopy(
            contract.R4_PLUS_FAILURE_EVIDENCE
        ),
        "safe_plus_reference": copy.deepcopy(
            contract.SAFE_V8R1P1_PLUS_REPLAY_ARTIFACTS
        ),
        "q2_historical_design": copy.deepcopy(contract.Q2_DESIGN_FREEZE_ARTIFACT),
        "q3_design": copy.deepcopy(contract.Q3_DESIGN_FREEZE_ARTIFACT),
        "external_runtime_sources": copy.deepcopy(
            contract.FROZEN_EXTERNAL_RUNTIME_SOURCES
        ),
        "production_source_closure": freezer._production_source_closure(),
        "safe_environment_execution_lock": copy.deepcopy(
            contract.SAFE_V8R1P1_EXECUTION_LOCK_ARTIFACT
        ),
        "safe_environment_source_closure": freezer._safe_environment_source_closure(),
    }


def _qualification_unopened() -> bool:
    paths = (*contract.Q2_UNOPENED_PATHS.values(), *contract.Q3_UNOPENED_PATHS.values())
    return all(not os.path.lexists(resolve_relative(path)) for path in paths)


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
    """Capture Q2/Q3 occupancy once and bind its fail-closed interpretation."""

    opened = _opened_qualification_paths()
    q2_opened = list(opened["q2"])
    q3_opened = list(opened["q3"])
    return {
        "q2_paths_opened": q2_opened,
        "q3_paths_opened": q3_opened,
        "qualification_violation_detected": bool(q2_opened or q3_opened),
    }


def _qualification_snapshot_valid(payload: Mapping[str, Any]) -> bool:
    """Validate stored qualification evidence without consulting live paths."""

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


def _preterminal_run_root_label() -> str:
    try:
        return RUN_ROOT.relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return str(RUN_ROOT)


def _inventory_regular_artifact(
    path: Path, *, relative_path: str, expected_stat: os.stat_result
) -> tuple[str, dict[str, Any] | None]:
    """Hash one lstat-confirmed file without intentionally following links."""

    flags = os.O_RDONLY
    flags |= getattr(os, "O_BINARY", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    descriptor: int | None = None
    try:
        descriptor = os.open(path, flags)
        opened_stat = os.fstat(descriptor)
        if not stat.S_ISREG(opened_stat.st_mode) or (
            expected_stat.st_dev,
            expected_stat.st_ino,
        ) != (opened_stat.st_dev, opened_stat.st_ino):
            return "VANISHED_DURING_SCAN", None
        digest = hashlib.sha256()
        size_bytes = 0
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
            size_bytes += len(chunk)
        final_stat = os.fstat(descriptor)
        if (opened_stat.st_dev, opened_stat.st_ino, opened_stat.st_size) != (
            final_stat.st_dev,
            final_stat.st_ino,
            final_stat.st_size,
        ) or size_bytes != final_stat.st_size:
            return "VANISHED_DURING_SCAN", None
        logical_path = PurePosixPath(
            _preterminal_run_root_label(), relative_path
        ).as_posix()
        return (
            "REGULAR_FILE",
            {
                "path": logical_path,
                "sha256": digest.hexdigest(),
                "size_bytes": size_bytes,
            },
        )
    except FileNotFoundError:
        return "VANISHED_DURING_SCAN", None
    except BaseException:
        return "UNREADABLE_REGULAR", None
    finally:
        if descriptor is not None:
            try:
                os.close(descriptor)
            except BaseException:
                pass


def _contract_run_relative(path: PurePosixPath) -> str:
    return path.relative_to(contract.RUN_ROOT).as_posix()


def _preterminal_run_root_allowed_relative_paths() -> frozenset[str]:
    """Enumerate every path the one-shot pipeline may own before terminalization."""

    allowed = {
        contract.PIPELINE_CLAIM_PATH.name,
        contract.PIPELINE_CLAIM_FAILURE_PATH.name,
        contract.WORKER_CLAIMS_ROOT.name,
        "locked_input_attestation_receipt.json",
        "case_balanced_corpus_assembly_receipt.json",
        "candidate_freeze_summary.json",
        "candidate_freeze_gate.json",
        contract.CANDIDATE_FREEZE_PATH.name,
        "final_development_summary.json",
        "final_development_gate.json",
        contract.FINAL_DEVELOPMENT_RECEIPT_PATH.name,
        _contract_run_relative(contract.FIT_ROOT),
        _contract_run_relative(contract.DEVELOPMENT_ROOT),
    }
    worker_root = _contract_run_relative(contract.WORKER_CLAIMS_ROOT)
    allowed.update(f"{worker_root}/{stage_id}.json" for stage_id in contract.STAGE_IDS)

    fit_root = _contract_run_relative(contract.FIT_ROOT)
    allowed.update(
        f"{fit_root}/{name}"
        for name in (
            "corpus.npz",
            "adaptation_history.json",
            "adaptation_report.json",
            "summary.json",
            "gate.json",
            "receipt.json",
        )
    )
    module_root = _contract_run_relative(contract.CANDIDATE_MODULE_PATH)
    allowed.add(module_root)
    allowed.update(
        f"{module_root}/{name}"
        for name in (
            "class_and_ctor_args.pkl",
            "metadata.json",
            "module_state.pkl",
        )
    )

    for case_id in contract.DEVELOPMENT_CASE_IDS:
        case_root = _contract_run_relative(contract.DEVELOPMENT_PATHS[case_id])
        steps_root = f"{case_root}/steps"
        allowed.update(
            {
                case_root,
                steps_root,
                f"{case_root}/run_start.json",
                f"{case_root}/trace.json",
                f"{case_root}/partial_summary.json",
                f"{case_root}/summary.json",
                f"{case_root}/gate.json",
                f"{case_root}/receipt.json",
                f"{case_root}/failure.json",
            }
        )
        allowed.update(
            f"{steps_root}/{step:06d}.json"
            for step in range(1, contract.EXPECTED_STEPS + 1)
        )
    return frozenset(allowed)


def _preterminal_run_root_inventory() -> dict[str, Any]:
    """Return a no-follow, no-throw inventory of all preterminal run output."""

    run_root_label = _preterminal_run_root_label()
    allowed_relative_paths = _preterminal_run_root_allowed_relative_paths()
    inventory: dict[str, Any] = {
        "run_root": run_root_label,
        "excluded_relative_paths": list(_PRETERMINAL_RUN_ROOT_EXCLUSIONS),
        "semantic_allowlist_id": _PRETERMINAL_RUN_ROOT_SEMANTIC_ALLOWLIST_ID,
        "root_state": "ABSENT",
        "complete": False,
        "semantic_closed": False,
        "unexpected_relative_paths": [],
        "entries": [],
    }
    try:
        try:
            root_stat = os.lstat(RUN_ROOT)
        except FileNotFoundError:
            return inventory
        except BaseException:
            inventory["root_state"] = "UNREADABLE_DIRECTORY"
            return inventory
        if stat.S_ISLNK(root_stat.st_mode):
            inventory["root_state"] = "UNSAFE_SYMLINK"
            return inventory
        if not stat.S_ISDIR(root_stat.st_mode):
            inventory["root_state"] = "UNSAFE_NONDIRECTORY"
            return inventory
        inventory["root_state"] = "DIRECTORY"
        rows: list[dict[str, Any]] = []

        def scan(directory: Path, prefix: PurePosixPath | None) -> str | None:
            try:
                children = sorted(os.scandir(directory), key=lambda entry: entry.name)
            except FileNotFoundError:
                return "VANISHED_DURING_SCAN"
            except BaseException:
                return "UNREADABLE_DIRECTORY"
            for child in children:
                relative = (
                    PurePosixPath(child.name) if prefix is None else prefix / child.name
                )
                relative_path = relative.as_posix()
                if relative_path in _PRETERMINAL_RUN_ROOT_EXCLUSIONS:
                    continue
                row: dict[str, Any] = {
                    "relative_path": relative_path,
                    "state": "VANISHED_DURING_SCAN",
                    "artifact": None,
                }
                try:
                    observed_stat = child.stat(follow_symlinks=False)
                except FileNotFoundError:
                    rows.append(row)
                    continue
                except BaseException:
                    row["state"] = "UNSAFE_SPECIAL"
                    rows.append(row)
                    continue
                if stat.S_ISLNK(observed_stat.st_mode):
                    row["state"] = "UNSAFE_SYMLINK"
                elif stat.S_ISDIR(observed_stat.st_mode):
                    row["state"] = "DIRECTORY"
                    rows.append(row)
                    directory_state = scan(Path(child.path), relative)
                    if directory_state is not None:
                        row["state"] = directory_state
                    continue
                elif stat.S_ISREG(observed_stat.st_mode):
                    row["state"], row["artifact"] = _inventory_regular_artifact(
                        Path(child.path),
                        relative_path=relative_path,
                        expected_stat=observed_stat,
                    )
                else:
                    row["state"] = "UNSAFE_SPECIAL"
                rows.append(row)
            return None

        root_scan_state = scan(RUN_ROOT, None)
        if root_scan_state is not None:
            inventory["root_state"] = root_scan_state
        rows.sort(key=lambda row: row["relative_path"])
        inventory["entries"] = rows
        inventory["complete"] = inventory["root_state"] == "DIRECTORY" and all(
            row["state"] in {"DIRECTORY", "REGULAR_FILE"} for row in rows
        )
        unexpected = sorted(
            row["relative_path"]
            for row in rows
            if row["relative_path"] not in allowed_relative_paths
        )
        inventory["unexpected_relative_paths"] = unexpected
        inventory["semantic_closed"] = inventory["complete"] and not unexpected
        return inventory
    except BaseException:
        inventory["root_state"] = "UNREADABLE_DIRECTORY"
        inventory["complete"] = False
        return inventory


def _preterminal_run_root_inventory_schema_valid(value: Any) -> bool:
    if (
        not isinstance(value, Mapping)
        or set(value) != _PRETERMINAL_RUN_ROOT_INVENTORY_FIELDS
        or type(value.get("run_root")) is not str
        or not _strict_equal(
            value.get("excluded_relative_paths"),
            list(_PRETERMINAL_RUN_ROOT_EXCLUSIONS),
        )
        or value.get("semantic_allowlist_id")
        != _PRETERMINAL_RUN_ROOT_SEMANTIC_ALLOWLIST_ID
        or value.get("root_state")
        not in {
            "DIRECTORY",
            "ABSENT",
            "UNSAFE_SYMLINK",
            "UNSAFE_NONDIRECTORY",
            "UNREADABLE_DIRECTORY",
            "VANISHED_DURING_SCAN",
        }
        or type(value.get("complete")) is not bool
        or type(value.get("semantic_closed")) is not bool
        or not isinstance(value.get("unexpected_relative_paths"), list)
        or not isinstance(value.get("entries"), list)
    ):
        return False
    entries = value["entries"]
    relative_paths: list[str] = []
    for row in entries:
        if (
            not isinstance(row, Mapping)
            or set(row) != _PRETERMINAL_RUN_ROOT_INVENTORY_ENTRY_FIELDS
            or type(row.get("relative_path")) is not str
            or not row["relative_path"]
            or row.get("state") not in _PRETERMINAL_RUN_ROOT_ENTRY_STATES
        ):
            return False
        relative = PurePosixPath(row["relative_path"])
        if (
            relative.is_absolute()
            or ".." in relative.parts
            or relative.as_posix() != row["relative_path"]
            or row["relative_path"] in _PRETERMINAL_RUN_ROOT_EXCLUSIONS
        ):
            return False
        relative_paths.append(row["relative_path"])
        artifact = row.get("artifact")
        if row.get("state") == "REGULAR_FILE":
            if (
                not isinstance(artifact, Mapping)
                or set(artifact) != {"path", "sha256", "size_bytes"}
                or type(artifact.get("path")) is not str
                or type(artifact.get("sha256")) is not str
                or len(artifact["sha256"]) != 64
                or any(
                    character not in "0123456789abcdef"
                    for character in artifact["sha256"]
                )
                or type(artifact.get("size_bytes")) is not int
                or artifact["size_bytes"] < 0
            ):
                return False
        elif artifact is not None:
            return False
    safe_complete = value.get("root_state") == "DIRECTORY" and all(
        row.get("state") in {"DIRECTORY", "REGULAR_FILE"} for row in entries
    )
    unexpected = [
        relative_path
        for relative_path in relative_paths
        if relative_path not in _preterminal_run_root_allowed_relative_paths()
    ]
    return (
        relative_paths == sorted(relative_paths)
        and len(relative_paths) == len(set(relative_paths))
        and value.get("complete") is safe_complete
        and _strict_equal(value.get("unexpected_relative_paths"), unexpected)
        and value.get("semantic_closed") is (safe_complete and not unexpected)
    )


def _load_stack() -> tuple[Any, Any, Any, Any, Any, Any, Any]:
    try:
        return runtime._load_rollout_stack()
    except Exception as exc:
        raise V12R5ExecutionError("V26 inference stack is not ready") from exc


def _module_preflight(
    path: Path, *, expected_tree: Mapping[str, Any]
) -> dict[str, Any]:
    import numpy as np
    import torch
    from ray.rllib.core.rl_module.rl_module import RLModule

    observed_tree = _tree(path)
    if observed_tree != dict(expected_tree):
        raise V12R5ExecutionError(f"checkpoint tree drifted: {path}")
    module = RLModule.from_checkpoint(path.resolve())
    module.eval()
    actor = np.zeros(contract.EXPECTED_ACTOR_FEATURES, dtype=np.float32)
    actor[1] = np.float32(1.0)
    mean, std = runtime._query_mean_std(module, actor, np=np, torch=torch)
    logits = np.concatenate((mean, np.log(std)), dtype=np.float32)
    if logits.shape != (4,) or not np.all(np.isfinite(logits)):
        raise V12R5ExecutionError(f"checkpoint inference is malformed: {path}")
    return {
        "module": observed_tree,
        "checkpoint_path_absolute": str(path.resolve()),
        "action_dist_inputs_shape": [1, 4],
        "action_dist_inputs_finite": True,
        "eval_mode": True,
    }


def build_execution_lock(*, require_unoccupied: bool = True) -> dict[str, Any]:
    if os.path.lexists(LOCK_PUBLICATION_FAILURE_PATH):
        verify_execution_lock_publication_failure()
        raise V12R5ExecutionError(
            "execution lock publication failure evidence is terminal and dominant"
        )
    protocol = freezer.verify_protocol_freeze()
    design = _mapping(DESIGN_AUDIT_PATH)
    q3_design_path = resolve_relative(contract.Q3_DESIGN_FREEZE_PATH)
    q3_design = _mapping(q3_design_path)
    q3_snapshot = q3_design.get("design_snapshot")
    q3_access = q3_design.get("qualification_access")
    q3_policy = (
        q3_access.get("candidate_binding_policy")
        if isinstance(q3_access, Mapping)
        else None
    )
    occupancy = {
        "lock_absent": not os.path.lexists(LOCK_PATH),
        "lock_publication_failure_absent": not os.path.lexists(
            LOCK_PUBLICATION_FAILURE_PATH
        ),
        "run_root_absent": not os.path.lexists(RUN_ROOT),
        "pipeline_claim_absent": not os.path.lexists(PIPELINE_CLAIM_PATH),
        "pipeline_claim_failure_absent": not os.path.lexists(
            PIPELINE_CLAIM_FAILURE_PATH
        ),
        "pipeline_ledger_absent": not os.path.lexists(PIPELINE_LEDGER_PATH),
        "terminal_publication_failure_absent": not os.path.lexists(
            PIPELINE_TERMINAL_PUBLICATION_FAILURE_PATH
        ),
        "qualification_unopened": _qualification_unopened(),
    }
    source_tree = _tree(SOURCE_H0_MODULE)
    checks = {
        "protocol": protocol.get("status") == contract.PROTOCOL_FREEZE_STATUS
        and protocol.get("passed") is True,
        "design_audit": design.get("status")
        == "PASS_H0_V12R5_CASE_BALANCED_DESIGN_AUDIT"
        and design.get("passed") is True,
        "q3_design": _record(q3_design_path) == contract.Q3_DESIGN_FREEZE_ARTIFACT
        and q3_design.get("status") == contract.Q3_DESIGN_FREEZE_STATUS
        and q3_design.get("passed") is True
        and q3_design.get("candidate_binding_state") == "DEFERRED"
        and q3_design.get("candidate_id") is None
        and q3_design.get("candidate_module") is None
        and isinstance(q3_snapshot, Mapping)
        and q3_snapshot.get("candidate_binding_state") == "DEFERRED"
        and isinstance(q3_policy, Mapping)
        and q3_policy.get("selection_rule") == contract.CANDIDATE_SELECTION_RULE
        and q3_policy.get("canonical_candidate_module_path")
        == contract.CANDIDATE_MODULE_PATH.as_posix(),
        "contract": contract.contract_self_check().get("passed") is True,
        "source_h0": source_tree.get("tree_sha256") == contract.SOURCE_H0_TREE_SHA256,
        "external_runtime_sources": _external_runtime_sources_exact(),
        "production_source_closure": freezer._production_source_closure_exact(
            protocol.get("locked_inputs", {}).get("production_source_closure")
        ),
        "occupancy": all(occupancy.values()),
        "authority": all(
            contract.AUTHORITY[name]
            for name in (
                "execution_lock_authorized",
                "actor_fit_execution_authorized",
                "candidate_freeze_authorized",
                "development_execution_authorized",
            )
        ),
    }
    passed = all(checks.values())
    payload = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.EXECUTION_LOCK_STATUS if passed else contract.TERMINAL_FAIL_STATUS
        ),
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "revision": contract.REVISION,
        "authority": copy.deepcopy(contract.AUTHORITY),
        "checks": checks,
        "occupancy": occupancy,
        "protocol_freeze": _record(PROTOCOL_FREEZE_PATH),
        "design_audit": _record(DESIGN_AUDIT_PATH),
        "q2_design_freeze": _record(
            resolve_relative(contract.Q2_DESIGN_FREEZE_ARTIFACT["path"])
        ),
        "q3_design_freeze": _record(q3_design_path),
        "locked_inputs": _locked_input_snapshot(),
        "source_h0": source_tree,
        "candidate_selection": {
            "rule": contract.CANDIDATE_SELECTION_RULE,
            "module_path": contract.CANDIDATE_MODULE_PATH.as_posix(),
            "candidate_id": "DEFERRED_UNTIL_CASE_BALANCED_FIT",
            "candidate_tree_sha256": "DEFERRED_UNTIL_CASE_BALANCED_FIT",
        },
        "stage_order": list(contract.STAGE_IDS),
        "run_root": contract.RUN_ROOT.as_posix(),
        "platform": {
            "system": platform.system(),
            "machine": platform.machine(),
            "python": platform.python_version(),
            "executable": str(Path(sys.executable).resolve()),
        },
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
    if require_unoccupied and not passed:
        failed = [name for name, value in checks.items() if not value]
        raise V12R5ExecutionError(f"execution lock preflight failed: {failed}")
    return payload


def _execution_lock_payload_valid(observed: Any) -> bool:
    if not _execution_lock_schema_exact(observed):
        return False
    try:
        protocol = freezer.verify_protocol_freeze()
        q2_design_record = _record(
            resolve_relative(contract.Q2_DESIGN_FREEZE_ARTIFACT["path"])
        )
        q3_design_record = _record(resolve_relative(contract.Q3_DESIGN_FREEZE_PATH))
        expected_platform = {
            "system": platform.system(),
            "machine": platform.machine(),
            "python": platform.python_version(),
            "executable": str(Path(sys.executable).resolve()),
        }
    except BaseException:
        return False
    checks = observed.get("checks")
    occupancy = observed.get("occupancy")
    return (
        observed.get("schema_version") == contract.SCHEMA_VERSION
        and type(observed.get("schema_version")) is type(contract.SCHEMA_VERSION)
        and observed.get("status") == contract.EXECUTION_LOCK_STATUS
        and observed.get("passed") is True
        and observed.get("protocol_id") == contract.PROTOCOL_ID
        and observed.get("pipeline_id") == contract.PIPELINE_ID
        and observed.get("revision") == contract.REVISION
        and _strict_equal(observed.get("authority"), contract.AUTHORITY)
        and _strict_equal(observed.get("stage_order"), list(contract.STAGE_IDS))
        and all(value is True for value in checks.values())
        and all(value is True for value in occupancy.values())
        and _strict_equal(observed.get("platform"), expected_platform)
        and _strict_equal(
            observed.get("protocol_freeze"), _record(PROTOCOL_FREEZE_PATH)
        )
        and _strict_equal(observed.get("design_audit"), _record(DESIGN_AUDIT_PATH))
        and _strict_equal(observed.get("q2_design_freeze"), q2_design_record)
        and _strict_equal(protocol.get("q2_historical_design_freeze"), q2_design_record)
        and _strict_equal(observed.get("q3_design_freeze"), q3_design_record)
        and _strict_equal(protocol.get("q3_design_freeze"), q3_design_record)
        and _strict_equal(q3_design_record, contract.Q3_DESIGN_FREEZE_ARTIFACT)
        and _strict_equal(observed.get("locked_inputs"), _locked_input_snapshot())
        and _strict_equal(observed.get("source_h0"), _tree(SOURCE_H0_MODULE))
        and _strict_equal(
            protocol.get("locked_inputs", {}).get("external_runtime_sources"),
            contract.FROZEN_EXTERNAL_RUNTIME_SOURCES,
        )
        and freezer._production_source_closure_exact(
            protocol.get("locked_inputs", {}).get("production_source_closure")
        )
        and _external_runtime_sources_exact()
        and _strict_equal(
            observed.get("candidate_selection"),
            {
                "rule": contract.CANDIDATE_SELECTION_RULE,
                "module_path": contract.CANDIDATE_MODULE_PATH.as_posix(),
                "candidate_id": "DEFERRED_UNTIL_CASE_BALANCED_FIT",
                "candidate_tree_sha256": "DEFERRED_UNTIL_CASE_BALANCED_FIT",
            },
        )
        and observed.get("run_root") == contract.RUN_ROOT.as_posix()
        and observed.get("one_shot") is True
        and all(
            observed.get(name) is False
            for name in (
                "retry_authorized",
                "resume_authorized",
                "rescue_authorized",
                "sweep_authorized",
                "qualification_execution_authorized",
            )
        )
        and all(
            type(observed.get(name)) is int and observed[name] == 0
            for name in (
                "environment_reset_calls",
                "environment_step_calls",
                "actor_updates",
                "critic_updates",
                "ppo_updates",
            )
        )
    )


def _execution_lock_publication_failure_payload(
    expected_lock: Mapping[str, Any], error: BaseException
) -> dict[str, Any]:
    expected = copy.deepcopy(dict(expected_lock))
    expected_record = _canonical_payload_record(LOCK_PATH, expected)
    qualification = _qualification_snapshot()
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.TERMINAL_FAIL_STATUS,
        "passed": False,
        "terminal": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "phase": "EXECUTION_LOCK_PUBLICATION",
        "expected_execution_lock": expected,
        "expected_execution_lock_record": expected_record,
        "execution_lock_observation": _expected_json_observation(
            LOCK_PATH, expected_record=expected_record
        ),
        "protocol_freeze": _record(PROTOCOL_FREEZE_PATH),
        "design_audit": _record(DESIGN_AUDIT_PATH),
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


def _execution_lock_publication_failure_payload_valid(payload: Any) -> bool:
    if (
        not isinstance(payload, Mapping)
        or set(payload) != _EXECUTION_LOCK_PUBLICATION_FAILURE_FIELDS
    ):
        return False
    expected = payload.get("expected_execution_lock")
    if not isinstance(expected, Mapping) or not _execution_lock_payload_valid(expected):
        return False
    expected = dict(expected)
    expected_record = _canonical_payload_record(LOCK_PATH, expected)
    observation = _expected_json_observation(LOCK_PATH, expected_record=expected_record)
    return (
        payload.get("schema_version") == contract.SCHEMA_VERSION
        and type(payload.get("schema_version")) is type(contract.SCHEMA_VERSION)
        and payload.get("status") == contract.TERMINAL_FAIL_STATUS
        and payload.get("passed") is False
        and payload.get("terminal") is True
        and payload.get("protocol_id") == contract.PROTOCOL_ID
        and payload.get("pipeline_id") == contract.PIPELINE_ID
        and payload.get("phase") == "EXECUTION_LOCK_PUBLICATION"
        and _strict_equal(
            payload.get("expected_execution_lock_record"), expected_record
        )
        and _strict_equal(payload.get("execution_lock_observation"), observation)
        and observation.get("state")
        in {
            "ABSENT",
            "VALID",
            "ZERO_BYTE_REGULAR",
            "INVALID_JSON_REGULAR",
            "INVALID_SCHEMA_REGULAR",
            "UNSAFE_SYMLINK",
            "UNSAFE_NONREGULAR",
            "UNSAFE_UNREADABLE_REGULAR",
        }
        and _strict_equal(payload.get("protocol_freeze"), _record(PROTOCOL_FREEZE_PATH))
        and _strict_equal(payload.get("design_audit"), _record(DESIGN_AUDIT_PATH))
        and _error_mapping_valid(payload.get("error"))
        and _qualification_snapshot_valid(payload)
        and all(
            type(payload.get(name)) is int and payload[name] == 0
            for name in (
                "new_collection_count",
                "environment_reset_calls",
                "environment_step_calls",
                "actor_updates",
                "critic_updates",
                "ppo_updates",
            )
        )
        and all(
            payload.get(name) is False
            for name in (
                "retry_authorized",
                "resume_authorized",
                "rescue_authorized",
                "sweep_authorized",
                "qualification_executed",
                "runtime_promoted",
                "checkpoint_zero_created",
                "positive_morphology_enabled",
            )
        )
        and payload.get("next_stage") == "STOP_TERMINAL"
    )


def verify_execution_lock_publication_failure() -> dict[str, Any]:
    payload = _mapping(LOCK_PUBLICATION_FAILURE_PATH)
    if not _execution_lock_publication_failure_payload_valid(payload):
        raise V12R5ExecutionError("execution lock publication failure evidence drifted")
    return payload


def _publish_execution_lock_publication_failure(
    expected_lock: Mapping[str, Any], error: BaseException
) -> None:
    failure = _execution_lock_publication_failure_payload(expected_lock, error)
    if not _execution_lock_publication_failure_payload_valid(failure):
        raise V12R5ExecutionError(
            "execution lock publication failure payload failed in-memory gate"
        ) from error
    freezer._write_json_exclusive(LOCK_PUBLICATION_FAILURE_PATH, failure)
    verify_execution_lock_publication_failure()


def prepare_execution_lock() -> dict[str, Any]:
    if os.path.lexists(LOCK_PUBLICATION_FAILURE_PATH):
        verify_execution_lock_publication_failure()
        raise V12R5ExecutionError(
            "execution lock publication failure evidence is terminal and dominant"
        )
    if os.path.lexists(LOCK_PATH):
        raise V12R5ExecutionError("execution lock exists/no-clobber")
    payload = build_execution_lock(require_unoccupied=True)
    if not _execution_lock_payload_valid(payload):
        raise V12R5ExecutionError("execution lock failed in-memory gate")
    try:
        freezer._write_json_exclusive(LOCK_PATH, payload)
        return verify_execution_lock(require_run_root_absent=True)
    except BaseException as exc:
        _publish_execution_lock_publication_failure(payload, exc)
        raise


def verify_execution_lock(*, require_run_root_absent: bool = False) -> dict[str, Any]:
    if os.path.lexists(LOCK_PUBLICATION_FAILURE_PATH):
        verify_execution_lock_publication_failure()
        raise V12R5ExecutionError(
            "execution lock publication failure evidence is terminal and dominant"
        )
    observed = _mapping(LOCK_PATH)
    if not _execution_lock_payload_valid(observed):
        raise V12R5ExecutionError("execution lock content drifted")
    if require_run_root_absent and os.path.lexists(RUN_ROOT):
        raise V12R5ExecutionError("R5 run root already claimed")
    if require_run_root_absent and not _qualification_unopened():
        raise V12R5ExecutionError("Q2/Q3 execution output already opened")
    return observed


def _claim_path(stage_id: str) -> Path:
    return WORKER_CLAIMS_ROOT / f"{stage_id}.json"


def _stage_receipt_path(stage_id: str) -> Path:
    descriptor = contract.stage_descriptor(stage_id)
    kind = descriptor["kind"]
    if kind == "attestation":
        return RUN_ROOT / "locked_input_attestation_receipt.json"
    if kind == "corpus":
        return RUN_ROOT / "case_balanced_corpus_assembly_receipt.json"
    if kind == "fit":
        return resolve_relative(contract.FIT_ROOT) / "receipt.json"
    if kind == "candidate_freeze":
        return CANDIDATE_FREEZE_PATH
    if kind == "development":
        return resolve_relative(descriptor["case"]["destination"]) / "receipt.json"
    return resolve_relative(contract.FINAL_DEVELOPMENT_RECEIPT_PATH)


def _historical_fit_call(callback: Any) -> Any:
    """Recompute immutable evidence while normalizing only later Q2/Q3 occupancy."""

    original = fit_engine._qualification_unopened
    fit_engine._qualification_unopened = lambda: True
    try:
        return callback()
    finally:
        fit_engine._qualification_unopened = original


def _historical_locked_input_attestation() -> dict[str, Any]:
    value = _historical_fit_call(fit_engine._attest_locked_inputs)
    if not isinstance(value, Mapping):
        raise V12R5ExecutionError("historical locked-input attestation is malformed")
    return dict(value)


def _historical_case_balanced_corpus() -> Any:
    return _historical_fit_call(fit_engine.load_case_balanced_corpus)


def _candidate_freeze_summary_payload_valid(
    summary: Any, *, module: Mapping[str, Any], identity: str, stage_id: str
) -> bool:
    if (
        not isinstance(summary, Mapping)
        or set(summary) != _CANDIDATE_FREEZE_SUMMARY_FIELDS
    ):
        return False
    return (
        _exact_int(summary.get("schema_version"), contract.SCHEMA_VERSION)
        and summary.get("status") == contract.CANDIDATE_FREEZE_COMPLETE_STATUS
        and summary.get("protocol_id") == contract.PROTOCOL_ID
        and summary.get("candidate_selection_rule") == contract.CANDIDATE_SELECTION_RULE
        and summary.get("candidate_id") == identity
        and _strict_equal(summary.get("candidate_module"), module)
        and _strict_equal(
            summary.get("fit_receipt"),
            _record(resolve_relative(contract.FIT_ROOT) / "receipt.json"),
        )
        and summary.get("fit_passed") is True
        and summary.get("candidate_frozen") is True
        and summary.get("source_h0_byte_exact") is True
        and summary.get("logstd_byte_exact") is True
        and summary.get("critic_present") is False
        and summary.get("save_reload_exact") is True
        and _strict_equal(summary.get("pipeline_claim"), _record(PIPELINE_CLAIM_PATH))
        and _strict_equal(summary.get("worker_claim"), _record(_claim_path(stage_id)))
        and summary.get("q2_paths_opened") == []
        and summary.get("q3_paths_opened") == []
        and summary.get("runtime_promoted") is False
        and all(
            _exact_int(summary.get(name), 0)
            for name in ("actor_updates", "critic_updates", "ppo_updates")
        )
    )


def _candidate_freeze_payload_valid(payload: Any) -> bool:
    stage_id = "freeze_case_balanced_candidate"
    try:
        if (
            not isinstance(payload, Mapping)
            or set(payload) != _CANDIDATE_FREEZE_RECEIPT_FIELDS
        ):
            return False
        summary_path = RUN_ROOT / "candidate_freeze_summary.json"
        gate_path = RUN_ROOT / "candidate_freeze_gate.json"
        summary = _mapping(summary_path)
        gate = _mapping(gate_path)
        module = _tree(resolve_relative(contract.CANDIDATE_MODULE_PATH))
        identity = contract.candidate_id(module["tree_sha256"])
        worker = _mapping(_claim_path(stage_id))
    except BaseException:
        return False
    expected_gate = contract.candidate_freeze_gate(summary)
    expected_receipt = {
        **copy.deepcopy(dict(summary)),
        "status": contract.CANDIDATE_FREEZE_STATUS,
        "passed": True,
        "stage_id": stage_id,
        "summary": _record(summary_path),
        "gate": _record(gate_path),
    }
    return (
        _candidate_freeze_summary_payload_valid(
            summary, module=module, identity=identity, stage_id=stage_id
        )
        and _strict_equal(gate, expected_gate)
        and gate.get("passed") is True
        and _strict_equal(dict(payload), expected_receipt)
        and _worker_claim_payload_valid(worker, stage_id=stage_id)
    )


def _episode_metric_valid(value: Any, *, expected_samples: int) -> bool:
    return (
        isinstance(value, Mapping)
        and set(value) == {"abs_max", "rms", "sample_count"}
        and type(value.get("abs_max")) is float
        and math.isfinite(value["abs_max"])
        and type(value.get("rms")) is float
        and math.isfinite(value["rms"])
        and _exact_int(value.get("sample_count"), expected_samples)
    )


def _episode_metrics_valid(value: Any) -> bool:
    return (
        isinstance(value, Mapping)
        and set(value) == {"reserve_norm_nm", "residual_norm_nm"}
        and all(
            _episode_metric_valid(
                value.get(name), expected_samples=contract.EXPECTED_STEPS
            )
            for name in ("reserve_norm_nm", "residual_norm_nm")
        )
    )


def _sea_episode_metrics_valid(value: Any) -> bool:
    if not isinstance(value, Mapping) or set(value) != {
        "pros_ankle_angle",
        "pros_knee_angle",
    }:
        return False
    ordinary = {
        "motor_accel_rad_s2": contract.EXPECTED_RAW_SENSOR_SAMPLES,
        "motor_power_w": contract.EXPECTED_RAW_SENSOR_SAMPLES,
        "motor_speed_rad_s": contract.EXPECTED_RAW_SENSOR_SAMPLES,
        "tau_spring_nm": contract.EXPECTED_RAW_SENSOR_SAMPLES,
        "tau_spring_rate_nm_s": contract.EXPECTED_RAW_SENSOR_SAMPLES
        - contract.EXPECTED_STEPS,
        "torque_error_nm": contract.EXPECTED_RAW_SENSOR_SAMPLES,
    }
    for actuator in value.values():
        if not isinstance(actuator, Mapping) or set(actuator) != {
            *ordinary,
            "tau_input_saturated",
        }:
            return False
        if any(
            not _episode_metric_valid(
                actuator.get(name), expected_samples=expected_samples
            )
            for name, expected_samples in ordinary.items()
        ):
            return False
        saturation = actuator.get("tau_input_saturated")
        if not (
            isinstance(saturation, Mapping)
            and set(saturation) == {"count", "fraction", "sample_count"}
            and _exact_int(saturation.get("count"), 0)
            and type(saturation.get("fraction")) is float
            and saturation["fraction"] == 0.0
            and _exact_int(
                saturation.get("sample_count"), contract.EXPECTED_RAW_SENSOR_SAMPLES
            )
        ):
            return False
    return True


def _binary_event_gate_valid(value: Any) -> bool:
    if not isinstance(value, Mapping) or set(value) != {
        "duplicate_event_count",
        "event_count",
        "events",
        "fallback_count",
        "hard_invalid_count",
        "left_non_v26_source_count",
        "out_of_order_event_count",
        "passed",
        "sample_count",
    }:
        return False
    events = value.get("events")
    return (
        value.get("passed") is True
        and _exact_int(value.get("sample_count"), contract.EXPECTED_RAW_SENSOR_SAMPLES)
        and all(
            _exact_int(value.get(name), 0)
            for name in (
                "duplicate_event_count",
                "fallback_count",
                "hard_invalid_count",
                "left_non_v26_source_count",
                "out_of_order_event_count",
            )
        )
        and isinstance(events, list)
        and _exact_int(value.get("event_count"), len(events))
        and all(isinstance(event, Mapping) for event in events)
    )


def _binary_event_prefix_valid(value: Any) -> bool:
    expected_checks = {
        "sample_count_positive",
        "sample_count_matches_raw_sensor",
        "raw_sensor_matches_control_windows",
        "ten_samples_per_control_step",
        "zero_duplicate_event_count",
        "zero_out_of_order_event_count",
        "zero_left_non_v26_source_count",
        "zero_fallback_count",
        "zero_hard_invalid_count",
    }
    if not isinstance(value, Mapping) or set(value) != {
        "passed",
        "sample_count",
        "raw_sensor_sample_count",
        "control_window_count",
        "expected_sample_count",
        "checks",
    }:
        return False
    checks = value.get("checks")
    return (
        value.get("passed") is True
        and all(
            _exact_int(value.get(name), contract.EXPECTED_RAW_SENSOR_SAMPLES)
            for name in (
                "sample_count",
                "raw_sensor_sample_count",
                "control_window_count",
                "expected_sample_count",
            )
        )
        and isinstance(checks, Mapping)
        and set(checks) == expected_checks
        and all(item is True for item in checks.values())
    )


def _development_summary_payload_valid(
    summary: Any,
    *,
    case_id: str,
    module: Mapping[str, Any],
    identity: str,
    freeze_record: Mapping[str, Any],
    trace_audit: Mapping[str, Any],
) -> bool:
    if not isinstance(summary, Mapping) or set(summary) != _DEVELOPMENT_SUMMARY_FIELDS:
        return False
    case = contract.canonical_development_case(case_id)
    zero_counts = (
        "action_clipped_values",
        "fallback_count",
        "timeout_count",
        "safety_stop_count",
        "sea_plugin_fallback_count",
        "so_solver_unaccepted_count",
        "hard_invalid_count",
        "invalid_event_count",
        "nonfinite_count",
        "routing_failure_count",
        "step_contract_failure_count",
        "binary_event_failure_count",
    )
    return (
        _exact_int(summary.get("schema_version"), contract.SCHEMA_VERSION)
        and summary.get("status") == contract.DEVELOPMENT_COMPLETE_STATUS
        and summary.get("protocol_id") == contract.PROTOCOL_ID
        and summary.get("case_id") == case_id
        and _strict_equal(summary.get("case"), case)
        and summary.get("action_selection") == case["action_selection"]
        and _strict_equal(
            summary.get("episode_start_offset_s"), case["episode_start_offset_s"]
        )
        and _strict_equal(summary.get("action_seed"), case["action_seed"])
        and _strict_equal(summary.get("runtime_seed"), case["runtime_seed"])
        and _strict_equal(summary.get("sigma"), case["sigma"])
        and summary.get("candidate_id") == identity
        and _strict_equal(summary.get("candidate_module"), module)
        and _strict_equal(summary.get("candidate_freeze"), freeze_record)
        and summary.get("teacher_enabled") is False
        and summary.get("blending_enabled") is False
        and summary.get("safety_latch_enabled") is False
        and _strict_equal(summary.get("pure_policy_trace_audit"), trace_audit)
        and _exact_int(
            summary.get("pure_policy_trace_row_count"), contract.EXPECTED_STEPS
        )
        and all(_exact_int(summary.get(name), 0) for name in zero_counts)
        and all(
            _exact_int(summary.get(name), 0)
            for name in contract.PURE_POLICY_COUNTER_FIELDS
        )
        and all(
            _exact_int(summary.get(name), 0)
            for name in ("actor_updates", "critic_updates", "ppo_updates")
        )
        and _exact_int(summary.get("steps"), contract.EXPECTED_STEPS)
        and _exact_int(
            summary.get("control_window_count"), contract.EXPECTED_CONTROL_WINDOWS
        )
        and _exact_int(
            summary.get("raw_sensor_sample_count"),
            contract.EXPECTED_RAW_SENSOR_SAMPLES,
        )
        and _exact_int(summary.get("phase_valid_cycle_count"))
        and type(summary.get("grf_penetration_max_m")) is float
        and math.isfinite(summary["grf_penetration_max_m"])
        and summary.get("end_reason") == "episode_time_limit"
        and summary.get("terminated") is False
        and summary.get("truncated") is True
        and _exact_int(summary.get("n_actor"), contract.EXPECTED_ACTOR_FEATURES)
        and _exact_int(summary.get("n_observation"), contract.EXPECTED_FULL_FEATURES)
        and summary.get("observation_dtype") == contract.EXPECTED_DTYPE
        and summary.get("binary_phase_fsm_mode") == "binary_active"
        and summary.get("event_contract_id") == contract.EVENT_CONTRACT_ID
        and summary.get("target_contract_id") == contract.TARGET_CONTRACT_ID
        and summary.get("detector_or_fsm_modified") is False
        and _strict_equal(summary.get("morphology_weight"), contract.MORPHOLOGY_WEIGHT)
        and _exact_int(
            summary.get("random_noise_draw_count"),
            contract.EXPECTED_STEPS if case["action_selection"] == "stochastic" else 0,
        )
        and _exact_int(
            summary.get("single_noise_application_count"), contract.EXPECTED_STEPS
        )
        and summary.get("protected_trials_opened") == []
        and summary.get("reserve_trials_opened") == []
        and summary.get("q2_paths_opened") == []
        and summary.get("q3_paths_opened") == []
        and summary.get("sea_reserve_gate_passed") is True
        and _episode_metrics_valid(summary.get("episode_metrics"))
        and _sea_episode_metrics_valid(summary.get("sea_episode_metrics"))
        and _binary_event_gate_valid(summary.get("binary_phase_event_gate"))
        and _binary_event_prefix_valid(summary.get("binary_event_prefix_integrity"))
    )


def _expected_development_bindings() -> list[dict[str, Any]]:
    bindings: list[dict[str, Any]] = []
    for case_id in contract.DEVELOPMENT_CASE_IDS:
        stage_id = f"development__{case_id}"
        root = resolve_relative(contract.DEVELOPMENT_PATHS[case_id])
        receipt = _mapping(root / "receipt.json")
        if not _stage_receipt_payload_valid(receipt, stage_id=stage_id):
            raise V12R5ExecutionError(f"development receipt drifted: {case_id}")
        trace = _sequence(root / "trace.json")
        bindings.append(
            {
                "case_id": case_id,
                "passed": True,
                "receipt": _record(root / "receipt.json"),
                "gate": _record(root / "gate.json"),
                "summary": _record(root / "summary.json"),
                "trace": _record(root / "trace.json"),
                "pure_policy_trace_audit": contract.pure_policy_trace_audit(
                    trace, case_id=case_id
                ),
            }
        )
    return bindings


def _expected_final_development_summary(
    *,
    module: Mapping[str, Any],
    identity: str,
    bindings: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    stage_id = "finalize_development"
    binding_rows = [copy.deepcopy(dict(row)) for row in bindings]
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "COMPLETE_H0_V12R5_CASE_BALANCED_DEVELOPMENT_AGGREGATE",
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_id": identity,
        "candidate_module": copy.deepcopy(dict(module)),
        "candidate_freeze": _record(CANDIDATE_FREEZE_PATH),
        "case_gates": [
            {"case_id": row["case_id"], "passed": True} for row in binding_rows
        ],
        "rollout_bindings": binding_rows,
        "candidate_tree_unique_count": 1,
        "new_collection_count": 0,
        "development_count": len(contract.DEVELOPMENT_CASE_IDS),
        "environment_reset_calls": len(contract.DEVELOPMENT_CASE_IDS),
        "environment_step_calls": len(contract.DEVELOPMENT_CASE_IDS)
        * contract.EXPECTED_STEPS,
        "raw_sensor_sample_count": len(contract.DEVELOPMENT_CASE_IDS)
        * contract.EXPECTED_RAW_SENSOR_SAMPLES,
        "teacher_query_count": 0,
        "pure_policy_trace_row_count": len(contract.DEVELOPMENT_CASE_IDS)
        * contract.EXPECTED_STEPS,
        **{name: 0 for name in contract.PURE_POLICY_COUNTER_FIELDS},
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "q2_paths_opened": [],
        "q3_paths_opened": [],
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "worker_claim": _record(_claim_path(stage_id)),
    }


def _final_development_summary_payload_valid(
    summary: Any,
    *,
    module: Mapping[str, Any],
    identity: str,
    bindings: Sequence[Mapping[str, Any]],
) -> bool:
    return (
        isinstance(summary, Mapping)
        and set(summary) == _FINAL_DEVELOPMENT_SUMMARY_FIELDS
        and _strict_equal(
            dict(summary),
            _expected_final_development_summary(
                module=module,
                identity=identity,
                bindings=bindings,
            ),
        )
    )


def _development_forensic_artifacts_valid(case_id: str) -> bool:
    stage_id = f"development__{case_id}"
    try:
        descriptor = contract.stage_descriptor(stage_id)
        destination = resolve_relative(descriptor["case"]["destination"])
        if destination.is_symlink() or not destination.is_dir():
            return False
        expected_children = {
            "run_start.json",
            "steps",
            "trace.json",
            "partial_summary.json",
            "summary.json",
            "gate.json",
            "receipt.json",
        }
        children = {child.name: child for child in destination.iterdir()}
        if set(children) != expected_children:
            return False
        if any(child.is_symlink() for child in children.values()):
            return False
        if not children["steps"].is_dir() or any(
            not child.is_file() for name, child in children.items() if name != "steps"
        ):
            return False

        writer = forensic.ForensicRolloutWriter(
            destination,
            artifact_root=REPO_ROOT,
        )
        finalized = writer.finalized_artifact_records()
        if (
            set(finalized) != {"trace", "partial_summary", "summary"}
            or writer.last_completed_step != contract.EXPECTED_STEPS
        ):
            return False

        module = _tree(resolve_relative(contract.CANDIDATE_MODULE_PATH))
        identity = contract.candidate_id(module["tree_sha256"])
        run_start = _mapping(destination / "run_start.json")
        partial = _mapping(destination / "partial_summary.json")
        expected_start = {
            "schema_version": contract.SCHEMA_VERSION,
            "status": "STARTED_H0_V12R5_CASE_BALANCED_DEVELOPMENT",
            "protocol_id": contract.PROTOCOL_ID,
            "stage_id": stage_id,
            "case": contract.canonical_development_case(case_id),
            "candidate_id": identity,
            "candidate_module": module,
            "candidate_freeze": _record(CANDIDATE_FREEZE_PATH),
            "teacher_enabled": False,
            "blending_enabled": False,
            "safety_latch_enabled": False,
            "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
            "worker_claim": _record(_claim_path(stage_id)),
        }
        expected_partial = {
            "schema_version": contract.SCHEMA_VERSION,
            "status": "PERSISTED_H0_V12R5_DEVELOPMENT_BEFORE_GATE",
            "protocol_id": contract.PROTOCOL_ID,
            "stage_id": stage_id,
            "steps": contract.EXPECTED_STEPS,
            "gate_evaluated": False,
        }
        return _strict_equal(run_start, expected_start) and _strict_equal(
            partial, expected_partial
        )
    except BaseException:
        return False


def _stage_receipt_payload_valid(payload: Any, *, stage_id: str) -> bool:
    descriptor = contract.stage_descriptor(stage_id)
    kind = descriptor["kind"]
    expected_fields = {
        "attestation": _ATTESTATION_RECEIPT_FIELDS,
        "corpus": _CORPUS_RECEIPT_FIELDS,
        "fit": _FIT_RECEIPT_FIELDS,
        "candidate_freeze": _CANDIDATE_FREEZE_RECEIPT_FIELDS,
        "development": _DEVELOPMENT_RECEIPT_FIELDS,
        "finalize": _FINAL_RECEIPT_FIELDS,
    }[kind]
    if not isinstance(payload, Mapping) or set(payload) != expected_fields:
        return False
    if kind == "candidate_freeze":
        return _candidate_freeze_payload_valid(payload)
    try:
        worker = _mapping(_claim_path(stage_id))
    except BaseException:
        return False
    common = (
        payload.get("schema_version") == contract.SCHEMA_VERSION
        and type(payload.get("schema_version")) is type(contract.SCHEMA_VERSION)
        and payload.get("passed") is True
        and payload.get("protocol_id") == contract.PROTOCOL_ID
        and payload.get("stage_id") == stage_id
        and _strict_equal(payload.get("pipeline_claim"), _record(PIPELINE_CLAIM_PATH))
        and _strict_equal(payload.get("worker_claim"), _record(_claim_path(stage_id)))
        and _worker_claim_payload_valid(worker, stage_id=stage_id)
    )
    if not common:
        return False
    if kind == "attestation":
        try:
            expected_attestation = _historical_locked_input_attestation()
        except BaseException:
            return False
        return (
            payload.get("status") == contract.SOURCE_ATTEST_PASS_STATUS
            and _strict_equal(
                payload.get("locked_input_attestation"), expected_attestation
            )
            and _exact_int(payload.get("failed_plus_prefix_rows_loaded"), 0)
            and _exact_int(payload.get("new_collection_count"), 0)
            and payload.get("q2_paths_opened") == []
            and payload.get("q3_paths_opened") == []
            and all(
                _exact_int(payload.get(name), 0)
                for name in ("actor_updates", "critic_updates", "ppo_updates")
            )
        )
    if kind == "corpus":
        try:
            expected_corpus = _historical_case_balanced_corpus()
        except BaseException:
            return False
        return (
            payload.get("status") == "PASS_H0_V12R5_CASE_BALANCED_CORPUS_ASSEMBLY"
            and _strict_equal(
                payload.get("fit_counts"), contract.expected_corpus_counts()
            )
            and _strict_equal(payload.get("corpus_audit"), expected_corpus.audit)
            and _strict_equal(
                payload.get("source_records"), expected_corpus.source_records
            )
            and _exact_int(payload.get("new_collection_count"), 0)
            and payload.get("q2_paths_opened") == []
            and payload.get("q3_paths_opened") == []
            and all(
                _exact_int(payload.get(name), 0)
                for name in ("actor_updates", "critic_updates", "ppo_updates")
            )
        )
    if kind == "fit":
        try:
            verified = fit_engine.verify_fit_stage()
        except BaseException:
            return False
        return (
            _strict_equal(dict(payload), verified)
            and payload.get("status") == contract.FIT_PASS_STATUS
            and payload.get("candidate_selection_rule")
            == contract.CANDIDATE_SELECTION_RULE
            and _exact_int(payload.get("actor_updates"), 1)
            and _exact_int(payload.get("critic_updates"), 0)
            and _exact_int(payload.get("ppo_updates"), 0)
        )
    if kind == "development":
        case_id = str(descriptor["case"]["case_id"])
        destination = resolve_relative(descriptor["case"]["destination"])
        try:
            summary = _mapping(destination / "summary.json")
            gate = _mapping(destination / "gate.json")
            trace = _sequence(destination / "trace.json")
            trace_audit = contract.pure_policy_trace_audit(trace, case_id=case_id)
            module = _tree(resolve_relative(contract.CANDIDATE_MODULE_PATH))
            identity = contract.candidate_id(module["tree_sha256"])
            freeze_record = _record(CANDIDATE_FREEZE_PATH)
            expected_gate = contract.development_gate(
                summary, case_id=case_id, trace=trace
            )
        except BaseException:
            return False
        expected_receipt = {
            "schema_version": contract.SCHEMA_VERSION,
            "status": contract.DEVELOPMENT_PASS_STATUS,
            "passed": True,
            "protocol_id": contract.PROTOCOL_ID,
            "stage_id": stage_id,
            "case_id": case_id,
            "candidate_id": identity,
            "candidate_module": module,
            "candidate_freeze": freeze_record,
            "summary": _record(destination / "summary.json"),
            "gate": _record(destination / "gate.json"),
            "trace": _record(destination / "trace.json"),
            "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
            "worker_claim": _record(_claim_path(stage_id)),
            "pure_policy_trace_audit": trace_audit,
            "pure_policy_trace_row_count": contract.EXPECTED_STEPS,
            **{name: 0 for name in contract.PURE_POLICY_COUNTER_FIELDS},
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
        }
        return (
            _development_summary_payload_valid(
                summary,
                case_id=case_id,
                module=module,
                identity=identity,
                freeze_record=freeze_record,
                trace_audit=trace_audit,
            )
            and _development_forensic_artifacts_valid(case_id)
            and _strict_equal(gate, expected_gate)
            and gate.get("passed") is True
            and _strict_equal(dict(payload), expected_receipt)
        )
    try:
        summary_path = RUN_ROOT / "final_development_summary.json"
        gate_path = RUN_ROOT / "final_development_gate.json"
        summary = _mapping(summary_path)
        gate = _mapping(gate_path)
        module = _tree(resolve_relative(contract.CANDIDATE_MODULE_PATH))
        identity = contract.candidate_id(module["tree_sha256"])
        bindings = _expected_development_bindings()
        expected_gate = contract.aggregate_development_gate(summary)
    except BaseException:
        return False
    expected_receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.FINAL_DEVELOPMENT_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_id": identity,
        "candidate_module": module,
        "candidate_freeze": _record(CANDIDATE_FREEZE_PATH),
        "summary": _record(summary_path),
        "gate": _record(gate_path),
        "rollout_bindings": bindings,
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "worker_claim": _record(_claim_path(stage_id)),
        "development_only": True,
        "qualification_executed": False,
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "pure_policy_trace_row_count": len(contract.DEVELOPMENT_CASE_IDS)
        * contract.EXPECTED_STEPS,
        **{name: 0 for name in contract.PURE_POLICY_COUNTER_FIELDS},
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    return (
        _final_development_summary_payload_valid(
            summary, module=module, identity=identity, bindings=bindings
        )
        and _strict_equal(gate, expected_gate)
        and gate.get("passed") is True
        and _strict_equal(dict(payload), expected_receipt)
    )


def _verify_stage_receipt_claim_bindings(
    stage_id: str, receipt: Mapping[str, Any] | None = None
) -> dict[str, Any]:
    """Verify exact stage semantics plus pipeline/worker claim closure."""

    observed = _mapping(_stage_receipt_path(stage_id))
    if (
        receipt is not None and not _strict_equal(observed, dict(receipt))
    ) or not _stage_receipt_payload_valid(observed, stage_id=stage_id):
        raise V12R5ExecutionError(f"stage receipt claim binding drifted: {stage_id}")
    return observed


def _raw_file_observation(
    path: Path,
    *,
    json_payload: bool,
    validator: Any = None,
) -> dict[str, Any]:
    if not os.path.lexists(path):
        return {"state": "ABSENT", "artifact": None}
    if path.is_symlink():
        return {"state": "UNSAFE_SYMLINK", "artifact": None}
    if not path.is_file():
        return {"state": "UNSAFE_NONREGULAR", "artifact": None}
    try:
        artifact = _record(path)
    except BaseException:
        return {"state": "UNSAFE_UNREADABLE_REGULAR", "artifact": None}
    if artifact.get("size_bytes") == 0:
        return {"state": "ZERO_BYTE_REGULAR", "artifact": artifact}
    if not json_payload:
        return {"state": "VALID_REGULAR", "artifact": artifact}
    try:
        payload = forensic.strict_json_load(path)
    except BaseException:
        state = "INVALID_JSON_REGULAR"
    else:
        if validator is None:
            state = "VALID_JSON_REGULAR"
        else:
            try:
                valid = validator(payload)
            except BaseException:
                valid = False
            state = "VALID_REGULAR" if valid else "INVALID_SCHEMA_REGULAR"
    return {"state": state, "artifact": artifact}


def _raw_tree_observation(path: Path) -> dict[str, Any]:
    if not os.path.lexists(path):
        return {"state": "ABSENT", "artifact": None}
    if path.is_symlink() or not path.is_dir():
        return {"state": "UNSAFE_NONDIRECTORY", "artifact": None}
    try:
        artifact = _tree(path)
    except BaseException:
        return {"state": "PARTIAL_OR_INVALID_DIRECTORY", "artifact": None}
    return {"state": "VALID_DIRECTORY", "artifact": artifact}


def _candidate_freeze_observation() -> dict[str, Any]:
    return _raw_file_observation(
        CANDIDATE_FREEZE_PATH,
        json_payload=True,
        validator=_candidate_freeze_payload_valid,
    )


def _stage_output_observations(stage_id: str) -> dict[str, Any]:
    descriptor = contract.stage_descriptor(stage_id)
    kind = descriptor["kind"]
    if kind in {"attestation", "corpus"}:
        return {
            "receipt": _raw_file_observation(
                _stage_receipt_path(stage_id), json_payload=True
            )
        }
    if kind == "fit":
        root = resolve_relative(contract.FIT_ROOT)
        return {
            "adaptation_history": _raw_file_observation(
                root / "adaptation_history.json", json_payload=True
            ),
            "adaptation_report": _raw_file_observation(
                root / "adaptation_report.json", json_payload=True
            ),
            "candidate_module": _raw_tree_observation(
                resolve_relative(contract.CANDIDATE_MODULE_PATH)
            ),
            "corpus": _raw_file_observation(root / "corpus.npz", json_payload=False),
            "gate": _raw_file_observation(root / "gate.json", json_payload=True),
            "receipt": _raw_file_observation(root / "receipt.json", json_payload=True),
            "summary": _raw_file_observation(root / "summary.json", json_payload=True),
        }
    if kind == "candidate_freeze":
        return {
            "gate": _raw_file_observation(
                RUN_ROOT / "candidate_freeze_gate.json", json_payload=True
            ),
            "receipt": _candidate_freeze_observation(),
            "summary": _raw_file_observation(
                RUN_ROOT / "candidate_freeze_summary.json", json_payload=True
            ),
        }
    if kind == "finalize":
        return {
            "gate": _raw_file_observation(
                RUN_ROOT / "final_development_gate.json", json_payload=True
            ),
            "receipt": _raw_file_observation(
                _stage_receipt_path(stage_id), json_payload=True
            ),
            "summary": _raw_file_observation(
                RUN_ROOT / "final_development_summary.json", json_payload=True
            ),
        }
    destination = resolve_relative(descriptor["case"]["destination"])
    return {"destination": _raw_tree_observation(destination)}


def _create_worker_claims_root() -> None:
    WORKER_CLAIMS_ROOT.mkdir(parents=False, exist_ok=False)


def _claim_run_root() -> tuple[str, dict[str, Any]]:
    """Persistently claim the one-shot run root; this performs filesystem I/O."""

    if os.path.lexists(RUN_ROOT):
        raise V12R5ExecutionError("R5 run root exists/no retry")
    verify_execution_lock(require_run_root_absent=True)
    token = secrets.token_hex(32)
    token_hash = hashlib.sha256(token.encode("utf-8")).hexdigest()
    execution_lock_record = _record(LOCK_PATH)
    protocol_freeze_record = _record(PROTOCOL_FREEZE_PATH)
    design_audit_record = _record(DESIGN_AUDIT_PATH)
    claim = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "CLAIMED_H0_V12R5_CASE_BALANCED_PIPELINE_ONCE",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "authority": copy.deepcopy(contract.AUTHORITY),
        "one_shot": True,
        "execution_token_sha256": token_hash,
        "execution_lock": execution_lock_record,
        "protocol_freeze": protocol_freeze_record,
        "design_audit": design_audit_record,
        "stage_order": list(contract.STAGE_IDS),
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "qualification_execution_authorized": False,
    }
    RUN_ROOT.mkdir(parents=True, exist_ok=False)
    try:
        forensic.write_json_exclusive(PIPELINE_CLAIM_PATH, claim)
        if _mapping(PIPELINE_CLAIM_PATH) != claim:
            raise V12R5ExecutionError("pipeline claim write did not round-trip")
        _create_worker_claims_root()
        return token_hash, claim
    except BaseException as exc:
        qualification = _qualification_snapshot()
        failure = {
            "schema_version": contract.SCHEMA_VERSION,
            "status": contract.TERMINAL_FAIL_STATUS,
            "passed": False,
            "terminal": True,
            "protocol_id": contract.PROTOCOL_ID,
            "pipeline_id": contract.PIPELINE_ID,
            "phase": "PIPELINE_CLAIM",
            "run_root": contract.RUN_ROOT.as_posix(),
            "run_root_claimed": True,
            "pipeline_claim_observation": _pipeline_claim_observation(),
            "worker_claims_root_state": _worker_claims_root_state(),
            "execution_lock": execution_lock_record,
            "protocol_freeze": protocol_freeze_record,
            **qualification,
            "error": _error_record(exc),
            "activity_totals": _activity_totals_snapshot(),
            "stage_activity": [],
            "new_collection_count": 0,
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
        try:
            forensic.write_json_exclusive(PIPELINE_CLAIM_FAILURE_PATH, failure)
            verify_pipeline_claim_failure()
        except BaseException as evidence_error:
            raise V12R5ExecutionError(
                "run root was claimed but claim-failure evidence could not be verified"
            ) from evidence_error
        raise


def verify_pipeline_claim_failure() -> dict[str, Any]:
    """Verify fail-closed evidence for an interrupted one-shot root claim."""

    failure = _mapping(PIPELINE_CLAIM_FAILURE_PATH)
    observation = _pipeline_claim_observation()
    worker_state = _worker_claims_root_state()
    error = failure.get("error")
    activity = {name: 0 for name in _ACTIVITY_NAMES}
    checks = {
        "schema": _pipeline_claim_failure_schema_exact(failure),
        "identity": failure.get("schema_version") == contract.SCHEMA_VERSION
        and type(failure.get("schema_version")) is type(contract.SCHEMA_VERSION)
        and failure.get("status") == contract.TERMINAL_FAIL_STATUS
        and failure.get("passed") is False
        and failure.get("terminal") is True
        and failure.get("protocol_id") == contract.PROTOCOL_ID
        and failure.get("pipeline_id") == contract.PIPELINE_ID
        and failure.get("phase") == "PIPELINE_CLAIM",
        "root_consumed": failure.get("run_root") == contract.RUN_ROOT.as_posix()
        and failure.get("run_root_claimed") is True
        and RUN_ROOT.is_dir()
        and not RUN_ROOT.is_symlink(),
        "claim_bound": _strict_equal(
            failure.get("pipeline_claim_observation"), observation
        )
        and observation.get("state")
        in {
            "ABSENT",
            "PARTIAL_OR_INVALID_REGULAR",
            "UNSAFE_UNREADABLE_REGULAR",
            "VALID_REGULAR",
        },
        "worker_root_bound": failure.get("worker_claims_root_state") == worker_state
        and worker_state in {"ABSENT", "DIRECTORY"},
        "run_root_closed": _pipeline_claim_failure_root_closed(),
        "sources_current": _strict_equal(
            failure.get("execution_lock"), _record(LOCK_PATH)
        )
        and _strict_equal(
            failure.get("protocol_freeze"), _record(PROTOCOL_FREEZE_PATH)
        ),
        "error_nonempty": _error_mapping_valid(error),
        "zero_activity": _strict_equal(failure.get("activity_totals"), activity)
        and failure.get("stage_activity") == [],
        "qualification_snapshot": _qualification_snapshot_valid(failure),
        "prohibitions": _exact_int(failure.get("new_collection_count"), 0)
        and _exact_int(failure.get("critic_updates"), 0)
        and _exact_int(failure.get("ppo_updates"), 0)
        and all(
            failure.get(name) is False
            for name in (
                "retry_authorized",
                "resume_authorized",
                "rescue_authorized",
                "sweep_authorized",
                "qualification_executed",
                "runtime_promoted",
                "checkpoint_zero_created",
                "positive_morphology_enabled",
            )
        ),
        "stop_terminal": failure.get("next_stage") == "STOP_TERMINAL",
    }
    if not all(checks.values()):
        failed = [name for name, value in checks.items() if not value]
        raise V12R5ExecutionError(f"pipeline claim failure evidence drifted: {failed}")
    return failure


def _pipeline_claim_failure_root_closed() -> bool:
    try:
        if RUN_ROOT.is_symlink() or not RUN_ROOT.is_dir():
            return False
        allowed = {PIPELINE_CLAIM_FAILURE_PATH.name}
        if os.path.lexists(PIPELINE_CLAIM_PATH):
            allowed.add(PIPELINE_CLAIM_PATH.name)
        if os.path.lexists(WORKER_CLAIMS_ROOT):
            allowed.add(WORKER_CLAIMS_ROOT.name)
        entries = {entry.name: entry for entry in RUN_ROOT.iterdir()}
        if set(entries) != allowed or any(
            entry.is_symlink() for entry in entries.values()
        ):
            return False
        if not PIPELINE_CLAIM_FAILURE_PATH.is_file():
            return False
        if os.path.lexists(PIPELINE_CLAIM_PATH) and not PIPELINE_CLAIM_PATH.is_file():
            return False
        if os.path.lexists(WORKER_CLAIMS_ROOT):
            if not WORKER_CLAIMS_ROOT.is_dir():
                return False
            if any(WORKER_CLAIMS_ROOT.iterdir()):
                return False
        return True
    except BaseException:
        return False


def _write_worker_claim(stage_id: str, token_hash: str) -> dict[str, Any]:
    index = contract.STAGE_IDS.index(stage_id)
    previous = _expected_previous_receipts(stage_id)
    path = _claim_path(stage_id)
    if os.path.lexists(path) or os.path.lexists(_stage_receipt_path(stage_id)):
        raise V12R5ExecutionError(f"stage already consumed: {stage_id}")
    payload = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "CLAIMED_H0_V12R5_CASE_BALANCED_STAGE_ONCE",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "authority": copy.deepcopy(contract.AUTHORITY),
        "one_shot": True,
        "stage_id": stage_id,
        "stage_index": index,
        "stage_kind": contract.stage_descriptor(stage_id)["kind"],
        "execution_token_sha256": token_hash,
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "previous_receipts": previous,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "qualification_execution_authorized": False,
    }
    forensic.write_json_exclusive(path, payload)
    observed = _mapping(path)
    if observed != payload or not _worker_claim_payload_valid(
        observed, stage_id=stage_id
    ):
        raise V12R5ExecutionError(f"worker claim semantic drift: {stage_id}")
    return payload


def _diagnostic_raw_journal(info: Mapping[str, Any], *, step: int) -> dict[str, Any]:
    """Capture only observer diagnostics; never consumed by actions or gates."""

    raw_samples = info.get("binary_phase_sensor_samples")
    phase_samples = info.get("phase_sensor_samples")
    samples: list[dict[str, Any]] = []
    if isinstance(raw_samples, Sequence) and not isinstance(raw_samples, (str, bytes)):
        phase_rows = (
            list(phase_samples)
            if isinstance(phase_samples, Sequence)
            and not isinstance(phase_samples, (str, bytes))
            else []
        )
        for index, raw in enumerate(raw_samples):
            raw_map = raw if isinstance(raw, Mapping) else {}
            phase_map = (
                phase_rows[index]
                if index < len(phase_rows) and isinstance(phase_rows[index], Mapping)
                else {}
            )
            samples.append(
                {
                    "sensor_index": index + 1,
                    "time_s": raw_map.get("time_s", phase_map.get("time_s")),
                    "left_heel_contact": raw_map.get("left_heel_contact"),
                    "left_toe_contact": raw_map.get("left_toe_contact"),
                    "left_heel_clearance_m": phase_map.get("left_heel_clearance_m"),
                    "left_toe_clearance_m": phase_map.get("left_toe_clearance_m"),
                    "left_heel_normal_n": phase_map.get("left_heel_normal_n"),
                    "left_toe_normal_n": phase_map.get("left_toe_normal_n"),
                }
            )
    binary = info.get("binary_phase_fsm")
    binary_map = binary if isinstance(binary, Mapping) else {}
    return {
        "observer_only": True,
        "control_dependency": False,
        "gate_dependency": False,
        "blocker_if_field_unavailable": False,
        "step": step,
        "samples": samples,
        "online_grf": copy.deepcopy(info.get("online_grf")),
        "online_grf_detector": copy.deepcopy(info.get("online_grf_detector")),
        "accepted_events": copy.deepcopy(binary_map.get("events_this_step", [])),
        "pending_event": copy.deepcopy(binary_map.get("pending_event")),
        "availability": {
            "binary_contact_samples": bool(samples),
            "clearance": any(
                row["left_heel_clearance_m"] is not None
                or row["left_toe_clearance_m"] is not None
                for row in samples
            ),
            "per_sensor_analog_grf": any(
                row["left_heel_normal_n"] is not None
                or row["left_toe_normal_n"] is not None
                for row in samples
            ),
            "step_online_grf": isinstance(info.get("online_grf"), Mapping),
        },
    }


def _rollout_stack() -> tuple[Any, Any, Any, Any, Any, Any, Any]:
    return _load_stack()


def _run_attestation() -> dict[str, Any]:
    stage_id = "attest_locked_inputs"
    attestation = fit_engine._attest_locked_inputs()
    qualification = _opened_qualification_paths()
    if qualification != {"q2": [], "q3": []}:
        raise V12R5ExecutionError("qualification output opened during attestation")
    payload = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.SOURCE_ATTEST_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "locked_input_attestation": copy.deepcopy(attestation),
        "failed_plus_prefix_rows_loaded": 0,
        "new_collection_count": 0,
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "worker_claim": _record(_claim_path(stage_id)),
        "q2_paths_opened": [],
        "q3_paths_opened": [],
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    forensic.write_json_exclusive(_stage_receipt_path(stage_id), payload)
    return payload


def _run_corpus_assembly() -> dict[str, Any]:
    stage_id = "assemble_case_balanced_corpus"
    corpus = fit_engine.load_case_balanced_corpus()
    payload = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "PASS_H0_V12R5_CASE_BALANCED_CORPUS_ASSEMBLY",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "fit_counts": contract.expected_corpus_counts(),
        "corpus_audit": copy.deepcopy(corpus.audit),
        "source_records": copy.deepcopy(corpus.source_records),
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "worker_claim": _record(_claim_path(stage_id)),
        "q2_paths_opened": [],
        "q3_paths_opened": [],
        "new_collection_count": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    forensic.write_json_exclusive(_stage_receipt_path(stage_id), payload)
    return payload


def _run_fit() -> dict[str, Any]:
    _activity_increment("actor_fit_stage_calls_attempted")
    _activity_increment("actor_updates_attempted")
    receipt = fit_engine.run_fit_stage(
        pipeline_claim_path=PIPELINE_CLAIM_PATH,
        worker_claim_path=_claim_path("fit_case_balanced_candidate"),
        protocol_freeze_path=PROTOCOL_FREEZE_PATH,
        execution_lock_path=LOCK_PATH,
        activity_callback=_activity_increment,
    )
    _activity_increment("actor_fit_executions_confirmed")
    _activity_increment("actor_updates_confirmed")
    return receipt


def _run_candidate_freeze() -> dict[str, Any]:
    stage_id = "freeze_case_balanced_candidate"
    fit_receipt = fit_engine.verify_fit_stage()
    fit_summary = _mapping(resolve_relative(contract.FIT_ROOT) / "summary.json")
    module = _tree(resolve_relative(contract.CANDIDATE_MODULE_PATH))
    identity = contract.candidate_id(module["tree_sha256"])
    if (
        fit_receipt.get("candidate_id") != identity
        or fit_receipt.get("candidate_module") != module
        or fit_summary.get("candidate_id") != identity
        or fit_summary.get("candidate_module") != module
    ):
        raise V12R5ExecutionError("fit candidate identity drifted before freeze")
    summary_path = RUN_ROOT / "candidate_freeze_summary.json"
    gate_path = RUN_ROOT / "candidate_freeze_gate.json"
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.CANDIDATE_FREEZE_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_id": identity,
        "candidate_module": module,
        "fit_receipt": _record(resolve_relative(contract.FIT_ROOT) / "receipt.json"),
        "fit_passed": True,
        "candidate_frozen": True,
        "source_h0_byte_exact": fit_summary.get("source_h0_byte_exact") is True,
        "logstd_byte_exact": fit_summary.get("logstd_byte_exact") is True,
        "critic_present": fit_summary.get("critic_present"),
        "save_reload_exact": fit_summary.get("save_reload_exact") is True,
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "worker_claim": _record(_claim_path(stage_id)),
        "q2_paths_opened": [],
        "q3_paths_opened": [],
        "runtime_promoted": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    forensic.write_json_exclusive(summary_path, summary)
    gate = contract.candidate_freeze_gate(summary)
    forensic.write_json_exclusive(gate_path, gate)
    if gate.get("passed") is not True:
        raise V12R5ExecutionError("candidate freeze gate failed")
    receipt = {
        **summary,
        "status": contract.CANDIDATE_FREEZE_STATUS,
        "passed": True,
        "stage_id": stage_id,
        "summary": _record(summary_path),
        "gate": _record(gate_path),
    }
    forensic.write_json_exclusive(CANDIDATE_FREEZE_PATH, receipt)
    return receipt


def _run_development(case_id: str) -> dict[str, Any]:
    (
        rollout_eval,
        np,
        torch,
        RLModule,
        env_factory,
        legacy,
        v26_collector,
    ) = _rollout_stack()
    stage_id = f"development__{case_id}"
    case = contract.canonical_development_case(case_id)
    destination = resolve_relative(case["destination"])
    if os.path.lexists(destination):
        raise V12R5ExecutionError(f"development destination exists: {case_id}")
    freeze = _mapping(CANDIDATE_FREEZE_PATH)
    module_path = resolve_relative(contract.CANDIDATE_MODULE_PATH).resolve()
    module_record = _tree(module_path)
    candidate_identity = contract.candidate_id(module_record["tree_sha256"])
    if (
        freeze.get("status") != contract.CANDIDATE_FREEZE_STATUS
        or freeze.get("passed") is not True
        or freeze.get("candidate_id") != candidate_identity
        or freeze.get("candidate_module") != module_record
    ):
        raise V12R5ExecutionError("candidate freeze identity drifted")
    candidate = RLModule.from_checkpoint(module_path.resolve())
    candidate.eval()
    innovations = runtime._frozen_innovations(
        case_id, action_selection=str(case["action_selection"]), np=np
    )
    env = env_factory.make_cmc_env(env_source.build_env_config(case))
    writer = forensic.ForensicRolloutWriter(destination, artifact_root=REPO_ROOT)
    writer.start(
        {
            "schema_version": contract.SCHEMA_VERSION,
            "status": "STARTED_H0_V12R5_CASE_BALANCED_DEVELOPMENT",
            "protocol_id": contract.PROTOCOL_ID,
            "stage_id": stage_id,
            "case": copy.deepcopy(case),
            "candidate_id": candidate_identity,
            "candidate_module": module_record,
            "candidate_freeze": _record(CANDIDATE_FREEZE_PATH),
            "teacher_enabled": False,
            "blending_enabled": False,
            "safety_latch_enabled": False,
            "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
            "worker_claim": _record(_claim_path(stage_id)),
        }
    )
    rows: list[dict[str, Any]] = []
    actor_names: tuple[str, ...] = ()
    full_names: tuple[str, ...] = ()
    audit: dict[str, Any] | None = None
    info: Mapping[str, Any] = {}
    terminated = False
    truncated = False
    started = time.monotonic()
    try:
        _activity_increment("environment_reset_calls")
        observation, reset_info = env.reset(seed=int(case["runtime_seed"]))
        observation = np.ascontiguousarray(observation, dtype=np.float32)
        actor_names, full_names = runtime._validate_runtime_layout(
            module=candidate,
            env=env,
            observation=observation,
            rollout_eval=rollout_eval,
            np=np,
        )
        audit = runtime._new_physical_audit(reset_info=reset_info, legacy=legacy, np=np)
        for index in range(contract.EXPECTED_STEPS):
            step = index + 1
            observation_before = observation.copy()
            actor = np.ascontiguousarray(
                observation_before[: contract.EXPECTED_ACTOR_FEATURES],
                dtype=np.float32,
            )
            mean, std = runtime._query_mean_std(candidate, actor, np=np, torch=torch)
            noise = np.ascontiguousarray(std * innovations[index], dtype=np.float32)
            raw_action = np.ascontiguousarray(
                np.add(mean, noise, dtype=np.float32), dtype=np.float32
            )
            if not np.all(np.isfinite(raw_action)):
                raise V12R5ExecutionError("development action is non-finite")
            applied = np.ascontiguousarray(
                np.clip(raw_action, env.action_space.low, env.action_space.high),
                dtype=np.float32,
            )
            _activity_increment("environment_step_calls")
            observation_after, reward, terminated, truncated, info = env.step(applied)
            observation_after = np.ascontiguousarray(
                observation_after, dtype=np.float32
            )
            if not isinstance(info, Mapping):
                raise V12R5ExecutionError("development info is malformed")
            raw_samples = info.get("binary_phase_sensor_samples")
            if (
                not isinstance(raw_samples, Sequence)
                or isinstance(raw_samples, (str, bytes))
                or len(raw_samples) != contract.EXPECTED_RAW_SENSOR_SAMPLES_PER_STEP
            ):
                raise V12R5ExecutionError(
                    "development step must expose exactly 10 raw sensor samples"
                )
            _activity_increment(
                "raw_sensor_sample_count",
                contract.EXPECTED_RAW_SENSOR_SAMPLES_PER_STEP,
            )
            physical = runtime._consume_physical_step(
                audit,
                step=step,
                info=info,
                observation_before=observation_before,
                observation_after=observation_after,
                reward=reward,
                action=raw_action,
                applied_action=applied,
                extra_vectors=(actor, mean, std, noise),
                legacy=legacy,
                v26_collector=v26_collector,
            )
            row = {
                "schema_version": contract.SCHEMA_VERSION,
                "protocol_id": contract.PROTOCOL_ID,
                "stage_id": stage_id,
                "case_id": case_id,
                "v26_observation": actor.tolist(),
                "candidate_mean": mean.tolist(),
                "candidate_std": std.tolist(),
                "standard_normal": innovations[index].tolist(),
                "single_noise": noise.tolist(),
                "raw_action": raw_action.tolist(),
                "applied_action": applied.tolist(),
                "teacher_enabled": False,
                "blending_enabled": False,
                "safety_latch_enabled": False,
                "teacher_query_count": 0,
                "served_action_teacher_dependency_count": 0,
                "mean_blend_count": 0,
                "safety_intervention_count": 0,
                "safety_latch_activation_count": 0,
                "safety_latch_release_count": 0,
                "raw_sensor_sample_count": len(raw_samples),
                "reward": float(reward),
                "time_s": float(info.get("time")),
                "grf_penetration_m": physical["penetration_m"],
                "reserve_norm_nm": physical["reserve_norm_nm"],
                "residual_norm_nm": physical["residual_norm_nm"],
                "phase_fsm": legacy._jsonable(physical["phase"]),
                "observer_raw_sensor_journal": legacy._jsonable(
                    _diagnostic_raw_journal(info, step=step)
                ),
                "checks": physical["checks"],
                "terminated": bool(terminated),
                "truncated": bool(truncated),
                "end_reason": info.get("end_reason"),
            }
            writer.write_step(step, row)
            rows.append({"step": step, **row})
            observation = observation_after
            if step == 1 or step % 25 == 0:
                elapsed = time.monotonic() - started
                eta = elapsed / step * (contract.EXPECTED_STEPS - step)
                print(
                    f"[V12R5 development/{case_id}] {step:3d}/"
                    f"{contract.EXPECTED_STEPS} elapsed={elapsed:7.1f}s "
                    f"eta={eta:7.1f}s",
                    flush=True,
                )
            if terminated or truncated:
                break
    finally:
        env.close()
    if audit is None:
        raise V12R5ExecutionError("development audit was not initialized")
    pure_policy_audit = contract.pure_policy_trace_audit(rows, case_id=case_id)
    if pure_policy_audit.get("passed") is not True:
        raise V12R5ExecutionError("pure-policy trace audit failed")
    partial = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "PERSISTED_H0_V12R5_DEVELOPMENT_BEFORE_GATE",
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "steps": len(rows),
        "gate_evaluated": False,
    }
    summary = {
        **runtime._physical_summary(
            audit,
            case=case,
            rows=rows,
            info=info,
            terminated=terminated,
            truncated=truncated,
            actor_names=actor_names,
            full_names=full_names,
            legacy=legacy,
            v26_collector=v26_collector,
        ),
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.DEVELOPMENT_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "case": copy.deepcopy(case),
        "candidate_id": candidate_identity,
        "candidate_module": module_record,
        "candidate_freeze": _record(CANDIDATE_FREEZE_PATH),
        "teacher_enabled": False,
        "blending_enabled": False,
        "safety_latch_enabled": False,
        "pure_policy_trace_audit": pure_policy_audit,
        "pure_policy_trace_row_count": len(rows),
        **{name: 0 for name in contract.PURE_POLICY_COUNTER_FIELDS},
        "sea_reserve_gate_passed": audit["sea_plugin_fallback_count"] == 0
        and audit["so_solver_unaccepted_count"] == 0
        and audit["nonfinite_count"] == 0,
        "target_contract_id": contract.TARGET_CONTRACT_ID,
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "detector_or_fsm_modified": False,
        "q2_paths_opened": [],
        "q3_paths_opened": [],
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    writer.finalize_before_gate(trace=rows, partial_summary=partial, summary=summary)
    gate = contract.development_gate(summary, case_id=case_id, trace=rows)
    writer.publish_gate(gate)
    if gate.get("passed") is not True:
        raise V12R5ExecutionError(f"development gate failed: {case_id}")
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.DEVELOPMENT_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "case_id": case_id,
        "candidate_id": candidate_identity,
        "candidate_module": module_record,
        "candidate_freeze": _record(CANDIDATE_FREEZE_PATH),
        "summary": _record(destination / "summary.json"),
        "gate": _record(destination / "gate.json"),
        "trace": _record(destination / "trace.json"),
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "worker_claim": _record(_claim_path(stage_id)),
        "pure_policy_trace_audit": pure_policy_audit,
        "pure_policy_trace_row_count": len(rows),
        **{name: 0 for name in contract.PURE_POLICY_COUNTER_FIELDS},
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    forensic.write_json_exclusive(destination / "receipt.json", receipt)
    return receipt


def _run_finalize_development() -> dict[str, Any]:
    stage_id = "finalize_development"
    freeze = _mapping(CANDIDATE_FREEZE_PATH)
    module = _tree(resolve_relative(contract.CANDIDATE_MODULE_PATH))
    identity = contract.candidate_id(module["tree_sha256"])
    bindings: list[dict[str, Any]] = []
    for case_id in contract.DEVELOPMENT_CASE_IDS:
        root = resolve_relative(contract.DEVELOPMENT_PATHS[case_id])
        receipt = _mapping(root / "receipt.json")
        summary = _mapping(root / "summary.json")
        gate = _mapping(root / "gate.json")
        trace = _sequence(root / "trace.json")
        trace_audit = contract.pure_policy_trace_audit(trace, case_id=case_id)
        expected_gate = contract.development_gate(summary, case_id=case_id, trace=trace)
        zero_counters = all(
            summary.get(name) == 0 and receipt.get(name) == 0
            for name in contract.PURE_POLICY_COUNTER_FIELDS
        )
        if (
            receipt.get("schema_version") != contract.SCHEMA_VERSION
            or receipt.get("status") != contract.DEVELOPMENT_PASS_STATUS
            or receipt.get("passed") is not True
            or receipt.get("protocol_id") != contract.PROTOCOL_ID
            or receipt.get("stage_id") != f"development__{case_id}"
            or receipt.get("case_id") != case_id
            or receipt.get("candidate_id") != identity
            or receipt.get("candidate_module") != module
            or receipt.get("candidate_freeze") != _record(CANDIDATE_FREEZE_PATH)
            or receipt.get("summary") != _record(root / "summary.json")
            or receipt.get("gate") != _record(root / "gate.json")
            or receipt.get("trace") != _record(root / "trace.json")
            or receipt.get("pipeline_claim") != _record(PIPELINE_CLAIM_PATH)
            or receipt.get("worker_claim")
            != _record(_claim_path(f"development__{case_id}"))
            or summary.get("candidate_id") != identity
            or summary.get("candidate_module") != module
            or summary.get("pure_policy_trace_audit") != trace_audit
            or summary.get("pure_policy_trace_row_count") != contract.EXPECTED_STEPS
            or receipt.get("pure_policy_trace_audit") != trace_audit
            or receipt.get("pure_policy_trace_row_count") != contract.EXPECTED_STEPS
            or trace_audit.get("passed") is not True
            or gate != expected_gate
            or gate.get("passed") is not True
            or not zero_counters
        ):
            raise V12R5ExecutionError(f"development binding drifted: {case_id}")
        bindings.append(
            {
                "case_id": case_id,
                "passed": True,
                "receipt": _record(root / "receipt.json"),
                "gate": _record(root / "gate.json"),
                "summary": _record(root / "summary.json"),
                "trace": _record(root / "trace.json"),
                "pure_policy_trace_audit": trace_audit,
            }
        )
    if (
        freeze.get("candidate_id") != identity
        or freeze.get("candidate_module") != module
    ):
        raise V12R5ExecutionError("candidate identity drifted at finalization")
    expected_activity = {
        "environment_reset_calls": 6,
        "environment_step_calls": 3000,
        "raw_sensor_sample_count": 30_000,
        "teacher_query_count": 0,
        "actor_fit_stage_calls_attempted": 1,
        "actor_fit_executions_confirmed": 1,
        "actor_updates_attempted": 1,
        "actor_updates_confirmed": 1,
        "adamw_epochs_completed": 3000,
    }
    activity_totals = _activity_totals_snapshot()
    observed_activity = {name: activity_totals[name] for name in expected_activity}
    if (
        observed_activity != expected_activity
        or activity_totals["lbfgs_closure_calls"] < 1
    ):
        raise V12R5ExecutionError(
            f"live pipeline activity drifted before finalization: {observed_activity}"
        )
    summary_path = RUN_ROOT / "final_development_summary.json"
    gate_path = RUN_ROOT / "final_development_gate.json"
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "COMPLETE_H0_V12R5_CASE_BALANCED_DEVELOPMENT_AGGREGATE",
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_id": identity,
        "candidate_module": module,
        "candidate_freeze": _record(CANDIDATE_FREEZE_PATH),
        "case_gates": [
            {"case_id": item["case_id"], "passed": item["passed"]} for item in bindings
        ],
        "rollout_bindings": bindings,
        "candidate_tree_unique_count": 1,
        "new_collection_count": 0,
        "development_count": 6,
        "environment_reset_calls": activity_totals["environment_reset_calls"],
        "environment_step_calls": activity_totals["environment_step_calls"],
        "raw_sensor_sample_count": activity_totals["raw_sensor_sample_count"],
        "teacher_query_count": activity_totals["teacher_query_count"],
        "pure_policy_trace_row_count": sum(
            int(item["pure_policy_trace_audit"]["row_count"]) for item in bindings
        ),
        **{name: 0 for name in contract.PURE_POLICY_COUNTER_FIELDS},
        "actor_updates": activity_totals["actor_updates_confirmed"],
        "critic_updates": 0,
        "ppo_updates": 0,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "q2_paths_opened": [],
        "q3_paths_opened": [],
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "worker_claim": _record(_claim_path(stage_id)),
    }
    forensic.write_json_exclusive(summary_path, summary)
    gate = contract.aggregate_development_gate(summary)
    forensic.write_json_exclusive(gate_path, gate)
    if gate.get("passed") is not True:
        raise V12R5ExecutionError("final development aggregate gate failed")
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.FINAL_DEVELOPMENT_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_id": identity,
        "candidate_module": module,
        "candidate_freeze": _record(CANDIDATE_FREEZE_PATH),
        "summary": _record(summary_path),
        "gate": _record(gate_path),
        "rollout_bindings": bindings,
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "worker_claim": _record(_claim_path(stage_id)),
        "development_only": True,
        "qualification_executed": False,
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "pure_policy_trace_row_count": summary["pure_policy_trace_row_count"],
        **{name: 0 for name in contract.PURE_POLICY_COUNTER_FIELDS},
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    forensic.write_json_exclusive(
        resolve_relative(contract.FINAL_DEVELOPMENT_RECEIPT_PATH), receipt
    )
    return receipt


def _run_stage(stage_id: str) -> dict[str, Any]:
    descriptor = contract.stage_descriptor(stage_id)
    kind = descriptor["kind"]
    if kind == "attestation":
        return _run_attestation()
    if kind == "corpus":
        return _run_corpus_assembly()
    if kind == "fit":
        return _run_fit()
    if kind == "candidate_freeze":
        return _run_candidate_freeze()
    if kind == "development":
        return _run_development(str(descriptor["case"]["case_id"]))
    if kind == "finalize":
        return _run_finalize_development()
    raise V12R5ExecutionError(f"unknown stage kind: {kind!r}")


def _visible_development_prefix(destination: Path) -> list[dict[str, Any]]:
    if not destination.exists():
        return []
    if not destination.is_dir() or destination.is_symlink():
        raise V12R5ExecutionError("development destination is unsafe")
    rows: list[dict[str, Any]] = []
    for path in sorted(destination.rglob("*")):
        if path == destination / "failure.json":
            continue
        if path.is_symlink():
            raise V12R5ExecutionError(f"linked development prefix path: {path}")
        if path.is_file():
            rows.append(_record(path))
    return rows


def _development_failure_payload_valid(payload: Any, *, stage_id: str) -> bool:
    if not isinstance(payload, Mapping) or set(payload) != _DEVELOPMENT_FAILURE_FIELDS:
        return False
    descriptor = contract.stage_descriptor(stage_id)
    if descriptor["kind"] != "development":
        return False
    destination = resolve_relative(descriptor["case"]["destination"])
    error = payload.get("error")
    worker_observation = _worker_claim_observation(stage_id)
    return (
        payload.get("schema_version") == contract.SCHEMA_VERSION
        and type(payload.get("schema_version")) is type(contract.SCHEMA_VERSION)
        and payload.get("status") == contract.TERMINAL_FAIL_STATUS
        and payload.get("passed") is False
        and payload.get("terminal") is True
        and payload.get("protocol_id") == contract.PROTOCOL_ID
        and payload.get("pipeline_id") == contract.PIPELINE_ID
        and payload.get("phase") == "DEVELOPMENT_STAGE"
        and payload.get("stage_id") == stage_id
        and payload.get("case_id") == descriptor["case"]["case_id"]
        and payload.get("end_reason") == "v12r5_development_failed_terminal_no_retry"
        and _error_mapping_valid(error)
        and _strict_equal(payload.get("worker_claim_observation"), worker_observation)
        and worker_observation.get("state")
        in {
            "ABSENT",
            "PARTIAL_OR_INVALID_REGULAR",
            "UNSAFE_UNREADABLE_REGULAR",
            "VALID_REGULAR",
        }
        and _strict_equal(payload.get("pipeline_claim"), _record(PIPELINE_CLAIM_PATH))
        and _pipeline_claim_payload_valid(_mapping(PIPELINE_CLAIM_PATH))
        and _strict_equal(payload.get("execution_lock"), _record(LOCK_PATH))
        and _strict_equal(payload.get("protocol_freeze"), _record(PROTOCOL_FREEZE_PATH))
        and _strict_equal(
            payload.get("visible_prefix"), _visible_development_prefix(destination)
        )
        and _qualification_snapshot_valid(payload)
        and _exact_int(payload.get("new_collection_count"), 0)
        and _exact_int(payload.get("stage_actor_updates"), 0)
        and _exact_int(payload.get("critic_updates"), 0)
        and _exact_int(payload.get("ppo_updates"), 0)
        and all(
            payload.get(name) is False
            for name in (
                "retry_authorized",
                "resume_authorized",
                "rescue_authorized",
                "sweep_authorized",
                "qualification_execution_authorized",
                "runtime_promoted",
                "checkpoint_zero_created",
                "positive_morphology_enabled",
            )
        )
        and payload.get("next_stage") == "STOP_TERMINAL"
    )


def _development_failure_observation(stage_id: str) -> dict[str, Any] | None:
    descriptor = contract.stage_descriptor(stage_id)
    if descriptor["kind"] != "development":
        return None
    failure_path = resolve_relative(descriptor["case"]["destination"]) / "failure.json"
    if not os.path.lexists(failure_path):
        return {"state": "ABSENT", "artifact": None}
    if failure_path.is_symlink():
        return {"state": "UNSAFE_SYMLINK", "artifact": None}
    if not failure_path.is_file():
        return {"state": "UNSAFE_NONREGULAR", "artifact": None}
    try:
        artifact = _record(failure_path)
    except BaseException:
        return {"state": "UNSAFE_UNREADABLE_REGULAR", "artifact": None}
    if artifact["size_bytes"] == 0:
        return {"state": "ZERO_BYTE_REGULAR", "artifact": artifact}
    try:
        payload = _mapping(failure_path)
    except V12R5ExecutionError:
        state = "INVALID_JSON_REGULAR"
    else:
        try:
            valid = _development_failure_payload_valid(payload, stage_id=stage_id)
        except BaseException:
            valid = False
        state = "VALID" if valid else "INVALID_SCHEMA_REGULAR"
    return {"state": state, "artifact": artifact}


def _ensure_development_failure_artifact(
    stage_id: str, error: BaseException
) -> dict[str, Any] | None:
    descriptor = contract.stage_descriptor(stage_id)
    if descriptor["kind"] != "development":
        return None
    destination = resolve_relative(descriptor["case"]["destination"])
    failure_path = destination / "failure.json"
    if os.path.lexists(failure_path):
        observation = _development_failure_observation(stage_id)
        if (
            isinstance(observation, Mapping)
            and observation.get("state") == "VALID"
            and isinstance(observation.get("artifact"), Mapping)
        ):
            return dict(observation["artifact"])
        raise V12R5ExecutionError(
            f"existing development failure evidence is invalid: {observation}"
        )
    destination.mkdir(parents=True, exist_ok=True)
    worker_observation = _worker_claim_observation(stage_id)
    pipeline_claim_record = _record(PIPELINE_CLAIM_PATH)
    execution_lock_record = _record(LOCK_PATH)
    protocol_freeze_record = _record(PROTOCOL_FREEZE_PATH)
    visible_prefix = _visible_development_prefix(destination)
    qualification = _qualification_snapshot()
    payload = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.TERMINAL_FAIL_STATUS,
        "passed": False,
        "terminal": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "phase": "DEVELOPMENT_STAGE",
        "stage_id": stage_id,
        "case_id": descriptor["case"]["case_id"],
        "end_reason": "v12r5_development_failed_terminal_no_retry",
        "error": _error_record(error),
        "worker_claim_observation": worker_observation,
        "pipeline_claim": pipeline_claim_record,
        "execution_lock": execution_lock_record,
        "protocol_freeze": protocol_freeze_record,
        "visible_prefix": visible_prefix,
        **qualification,
        "new_collection_count": 0,
        "stage_actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "qualification_execution_authorized": False,
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
        "next_stage": "STOP_TERMINAL",
    }
    if not _development_failure_payload_valid(payload, stage_id=stage_id):
        raise V12R5ExecutionError("development failure payload failed in-memory gate")
    forensic.write_json_exclusive(failure_path, payload)
    observation = _development_failure_observation(stage_id)
    if not isinstance(observation, Mapping) or observation.get("state") != "VALID":
        raise V12R5ExecutionError(
            f"development failure publication did not verify: {observation}"
        )
    return dict(observation["artifact"])


def _attempt_development_failure_artifact(
    stage_id: str, error: BaseException
) -> dict[str, str] | None:
    """Attempt stage evidence without preventing the terminal-ledger write."""

    try:
        _ensure_development_failure_artifact(stage_id, error)
    except BaseException as publication_error:
        return _error_record(publication_error)
    return None


def _terminal_intent(
    *,
    passed: bool,
    attempted_stage: str | None,
    completed: Sequence[Mapping[str, Any]],
    error: BaseException | Mapping[str, Any] | None,
    failure_publication_error: Mapping[str, Any] | None,
) -> dict[str, Any]:
    """Build the filesystem-independent terminalization intent.

    This object is deliberately constructed before any terminal artifact is
    inspected.  It therefore remains available even when ledger construction
    itself fails on a partial or unsafe stage output.
    """

    completed_rows = [copy.deepcopy(dict(row)) for row in completed]
    completed_count = len(completed_rows)
    terminal_attempted = (
        None if completed_count == len(contract.STAGE_IDS) else attempted_stage
    )
    if isinstance(error, BaseException):
        stage_error: Mapping[str, Any] | None = _error_record(error)
    elif error is None:
        stage_error = None
    else:
        stage_error = copy.deepcopy(dict(error))
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "intended_outcome": "PASS" if passed else "FAIL",
        "completed_stages": completed_rows,
        "completed_stage_count": completed_count,
        "attempted_stage": terminal_attempted,
        "stage_error": stage_error,
        "failure_publication_error": (
            None
            if failure_publication_error is None
            else copy.deepcopy(dict(failure_publication_error))
        ),
        "activity_totals": _activity_totals_snapshot(),
        "stage_activity": [
            copy.deepcopy(_STAGE_ACTIVITY[stage_id])
            for stage_id in contract.STAGE_IDS
            if stage_id in _STAGE_ACTIVITY
        ],
        "next_stage": "WAIT_SEPARATE_Q3_PROTOCOL" if passed else "STOP_TERMINAL",
    }


def _terminal_intent_from_ledger(ledger: Mapping[str, Any]) -> dict[str, Any]:
    completed = ledger.get("completed_stages")
    completed_rows = completed if isinstance(completed, list) else []
    completed_count = ledger.get("completed_stage_count")
    normalized_attempted = (
        None
        if type(completed_count) is int and completed_count == len(contract.STAGE_IDS)
        else ledger.get("attempted_stage")
    )
    passed = ledger.get("passed") is True
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "intended_outcome": "PASS" if passed else "FAIL",
        "completed_stages": copy.deepcopy(completed_rows),
        "completed_stage_count": completed_count,
        "attempted_stage": normalized_attempted,
        "stage_error": copy.deepcopy(ledger.get("error")),
        "failure_publication_error": copy.deepcopy(
            ledger.get("attempted_stage_failure_publication_error")
        ),
        "activity_totals": copy.deepcopy(ledger.get("activity_totals")),
        "stage_activity": copy.deepcopy(ledger.get("stage_activity")),
        "next_stage": ledger.get(
            "next_stage",
            "WAIT_SEPARATE_Q3_PROTOCOL" if passed else "STOP_TERMINAL",
        ),
    }


def _completed_stage_prefix_current(
    completed: Sequence[Mapping[str, Any]], *, outcome: str
) -> bool:
    try:
        for row in completed:
            stage_id = str(row["stage_id"])
            receipt_path = _stage_receipt_path(stage_id)
            if not receipt_path.is_file() or receipt_path.is_symlink():
                return False
            if not _strict_equal(row["receipt"], _record(receipt_path)):
                return False
            _verify_stage_receipt_claim_bindings(stage_id)
        completed_count = len(completed)
        if outcome == "PASS":
            return completed_count == len(contract.STAGE_IDS)

        attempted_stage = contract.STAGE_IDS[completed_count]
        attempted_path = _stage_receipt_path(attempted_stage)
        if attempted_path.is_file() and not attempted_path.is_symlink():
            try:
                _verify_stage_receipt_claim_bindings(attempted_stage)
            except BaseException:
                pass
            else:
                return False
        return _attempted_stage_ownership_coherent(attempted_stage) and all(
            _stage_ownership_absent(stage_id)
            for stage_id in contract.STAGE_IDS[completed_count + 1 :]
        )
    except BaseException:
        return False


def _stage_ownership_absent(stage_id: str) -> bool:
    if os.path.lexists(_claim_path(stage_id)):
        return False
    try:
        observations = _stage_output_observations(stage_id)
    except BaseException:
        return False
    return bool(observations) and all(
        isinstance(observation, Mapping)
        and observation.get("state") == "ABSENT"
        and observation.get("artifact") is None
        for observation in observations.values()
    )


def _attempted_stage_ownership_coherent(stage_id: str) -> bool:
    """A stage cannot emit output before its exclusive worker claim succeeds."""

    try:
        worker = _worker_claim_observation(stage_id)
        outputs = _stage_output_observations(stage_id)
    except BaseException:
        return False
    if (
        not isinstance(worker, Mapping)
        or not isinstance(outputs, Mapping)
        or not outputs
    ):
        return False
    state = worker.get("state")
    if state == "VALID_REGULAR":
        return _artifact_record_schema_valid(
            worker.get("artifact"), expected_path=_claim_path(stage_id)
        )
    if state not in {
        "ABSENT",
        "PARTIAL_OR_INVALID_REGULAR",
        "UNSAFE_NONREGULAR",
        "UNSAFE_UNREADABLE_REGULAR",
    }:
        return False
    return all(
        isinstance(observation, Mapping)
        and observation.get("state") == "ABSENT"
        and observation.get("artifact") is None
        for observation in outputs.values()
    )


def _completed_fit_lbfgs_closure_calls_current() -> int | None:
    try:
        summary = _mapping(resolve_relative(contract.FIT_ROOT) / "summary.json")
        value = summary.get("lbfgs_closure_calls")
    except BaseException:
        return None
    maximum = int(contract.FIT["lbfgs"]["max_eval"])
    return value if type(value) is int and 1 <= value <= maximum else None


def _completed_stage_activity_valid(row: Mapping[str, Any], *, stage_id: str) -> bool:
    expected = {name: 0 for name in _ACTIVITY_NAMES}
    kind = contract.stage_descriptor(stage_id)["kind"]
    if kind == "fit":
        lbfgs_closure_calls = _completed_fit_lbfgs_closure_calls_current()
        expected.update(
            {
                "actor_fit_stage_calls_attempted": 1,
                "actor_fit_executions_confirmed": 1,
                "actor_updates_attempted": 1,
                "actor_updates_confirmed": 1,
                "adamw_epochs_completed": 3000,
            }
        )
        return (
            all(
                row[name] == value
                for name, value in expected.items()
                if name != "lbfgs_closure_calls"
            )
            and row["lbfgs_closure_calls"] == lbfgs_closure_calls
        )
    if kind == "development":
        expected.update(
            {
                "environment_reset_calls": 1,
                "environment_step_calls": contract.EXPECTED_STEPS,
                "raw_sensor_sample_count": contract.EXPECTED_RAW_SENSOR_SAMPLES,
            }
        )
    return all(row[name] == value for name, value in expected.items())


def _attempted_stage_activity_valid(row: Mapping[str, Any], *, stage_id: str) -> bool:
    kind = contract.stage_descriptor(stage_id)["kind"]
    if row["teacher_query_count"] != 0:
        return False
    fit_fields = {
        "actor_fit_stage_calls_attempted",
        "actor_fit_executions_confirmed",
        "actor_updates_attempted",
        "actor_updates_confirmed",
        "adamw_epochs_completed",
        "lbfgs_closure_calls",
    }
    development_fields = {
        "environment_reset_calls",
        "environment_step_calls",
        "raw_sensor_sample_count",
    }
    permitted = (
        fit_fields
        if kind == "fit"
        else development_fields
        if kind == "development"
        else set()
    )
    if any(row[name] != 0 for name in set(_ACTIVITY_NAMES) - permitted):
        return False
    if kind == "fit":
        return (
            row["actor_fit_stage_calls_attempted"] <= 1
            and row["actor_fit_executions_confirmed"] <= 1
            and row["actor_updates_attempted"] <= 1
            and row["actor_updates_confirmed"] <= 1
            and row["actor_updates_attempted"] <= row["actor_fit_stage_calls_attempted"]
            and row["actor_updates_confirmed"] <= row["actor_fit_executions_confirmed"]
            and row["actor_fit_executions_confirmed"]
            <= row["actor_fit_stage_calls_attempted"]
            and row["actor_updates_confirmed"] <= row["actor_updates_attempted"]
            and row["adamw_epochs_completed"] <= 3000
            and row["lbfgs_closure_calls"] <= int(contract.FIT["lbfgs"]["max_eval"])
            and (
                row["adamw_epochs_completed"] == 0
                or row["actor_fit_stage_calls_attempted"] == 1
            )
            and (
                row["lbfgs_closure_calls"] == 0 or row["adamw_epochs_completed"] == 3000
            )
            and (
                row["actor_fit_executions_confirmed"] == 0
                or (
                    row["adamw_epochs_completed"] == 3000
                    and row["lbfgs_closure_calls"] >= 1
                )
            )
            and row["actor_updates_confirmed"] <= row["actor_fit_executions_confirmed"]
        )
    if kind == "development":
        steps = row["environment_step_calls"]
        raw_samples = row["raw_sensor_sample_count"]
        return (
            row["environment_reset_calls"] <= 1
            and steps <= contract.EXPECTED_STEPS
            and raw_samples <= contract.EXPECTED_RAW_SENSOR_SAMPLES
            and (steps == 0 or row["environment_reset_calls"] == 1)
            and raw_samples
            in {
                steps * contract.EXPECTED_RAW_SENSOR_SAMPLES_PER_STEP,
                max(0, steps - 1) * contract.EXPECTED_RAW_SENSOR_SAMPLES_PER_STEP,
            }
        )
    return True


def _terminal_intent_activity_valid(
    activity: Sequence[Any],
    totals: Mapping[str, Any],
    *,
    completed_count: int,
    outcome: str,
) -> bool:
    expected_ids = (
        list(contract.STAGE_IDS)
        if outcome == "PASS"
        else list(contract.STAGE_IDS[: completed_count + 1])
    )
    if len(activity) != len(expected_ids):
        return False
    for index, (row, stage_id) in enumerate(zip(activity, expected_ids, strict=True)):
        if (
            not isinstance(row, Mapping)
            or set(row) != {"stage_id", "stage_kind", *_ACTIVITY_NAMES}
            or row.get("stage_id") != stage_id
            or row.get("stage_kind") != contract.stage_descriptor(stage_id)["kind"]
            or any(
                type(row.get(name)) is not int or row[name] < 0
                for name in _ACTIVITY_NAMES
            )
        ):
            return False
        if index < completed_count:
            if not _completed_stage_activity_valid(row, stage_id=stage_id):
                return False
        elif not _attempted_stage_activity_valid(row, stage_id=stage_id):
            return False
    recomputed = {
        name: sum(int(row[name]) for row in activity) for name in _ACTIVITY_NAMES
    }
    return _strict_equal(dict(totals), recomputed)


def _terminal_intent_snapshot_schema_valid(intent: Any) -> bool:
    """Validate that an intent is safe to preserve as terminal failure evidence.

    This deliberately checks structure, identity, and canonical ordering without
    requiring live stage semantics.  The stricter `_terminal_intent_valid` gate
    may itself be the operation that failed; retaining its controlled snapshot
    is what prevents that failure from creating a terminal-evidence gap.
    """

    if not isinstance(intent, Mapping) or set(intent) != _TERMINAL_INTENT_FIELDS:
        return False
    completed = intent.get("completed_stages")
    completed_count = intent.get("completed_stage_count")
    totals = intent.get("activity_totals")
    activity = intent.get("stage_activity")
    outcome = intent.get("intended_outcome")
    if (
        intent.get("schema_version") != contract.SCHEMA_VERSION
        or type(intent.get("schema_version")) is not type(contract.SCHEMA_VERSION)
        or intent.get("protocol_id") != contract.PROTOCOL_ID
        or intent.get("pipeline_id") != contract.PIPELINE_ID
        or outcome not in {"PASS", "FAIL"}
        or not isinstance(completed, list)
        or type(completed_count) is not int
        or completed_count != len(completed)
        or completed_count < 0
        or completed_count > len(contract.STAGE_IDS)
        or not isinstance(totals, Mapping)
        or set(totals) != set(_ACTIVITY_NAMES)
        or any(
            type(totals.get(name)) is not int or totals[name] < 0
            for name in _ACTIVITY_NAMES
        )
        or not isinstance(activity, list)
    ):
        return False
    completed_ids = [
        row.get("stage_id") if isinstance(row, Mapping) else None for row in completed
    ]
    if completed_ids != list(contract.STAGE_IDS[:completed_count]) or any(
        not isinstance(row, Mapping)
        or set(row) != {"stage_id", "receipt"}
        or not _artifact_record_schema_valid(
            row.get("receipt"),
            expected_path=_stage_receipt_path(str(row.get("stage_id"))),
        )
        for row in completed
    ):
        return False
    expected_attempted = (
        None
        if completed_count == len(contract.STAGE_IDS)
        else contract.STAGE_IDS[completed_count]
    )
    if intent.get("attempted_stage") != expected_attempted:
        return False
    activity_ids = [
        row.get("stage_id") if isinstance(row, Mapping) else None for row in activity
    ]
    if (
        activity_ids != list(contract.STAGE_IDS[: len(activity)])
        or len(activity) < completed_count
        or len(activity) > min(completed_count + 1, len(contract.STAGE_IDS))
        or any(
            not isinstance(row, Mapping)
            or set(row) != {"stage_id", "stage_kind", *_ACTIVITY_NAMES}
            or row.get("stage_kind")
            != contract.stage_descriptor(str(row.get("stage_id")))["kind"]
            or any(
                type(row.get(name)) is not int or row[name] < 0
                for name in _ACTIVITY_NAMES
            )
            for row in activity
        )
    ):
        return False
    stage_error = intent.get("stage_error")
    publication_error = intent.get("failure_publication_error")
    return (
        (stage_error is None or _error_mapping_valid(stage_error))
        and (publication_error is None or _error_mapping_valid(publication_error))
        and intent.get("next_stage")
        == ("WAIT_SEPARATE_Q3_PROTOCOL" if outcome == "PASS" else "STOP_TERMINAL")
    )


def _terminal_intent_valid(intent: Any) -> bool:
    if not _terminal_intent_snapshot_schema_valid(intent):
        return False
    if _pipeline_claim_observation().get("state") != "VALID_REGULAR":
        return False
    completed = intent.get("completed_stages")
    completed_count = intent.get("completed_stage_count")
    totals = intent.get("activity_totals")
    activity = intent.get("stage_activity")
    outcome = intent.get("intended_outcome")
    if (
        intent.get("schema_version") != contract.SCHEMA_VERSION
        or type(intent.get("schema_version")) is not type(contract.SCHEMA_VERSION)
        or intent.get("protocol_id") != contract.PROTOCOL_ID
        or intent.get("pipeline_id") != contract.PIPELINE_ID
        or outcome not in {"PASS", "FAIL"}
        or not isinstance(completed, list)
        or type(completed_count) is not int
        or completed_count != len(completed)
        or completed_count < 0
        or completed_count > len(contract.STAGE_IDS)
        or not isinstance(totals, Mapping)
        or set(totals) != set(_ACTIVITY_NAMES)
        or any(
            type(totals.get(name)) is not int or totals[name] < 0
            for name in _ACTIVITY_NAMES
        )
        or not isinstance(activity, list)
    ):
        return False
    completed_ids = [
        row.get("stage_id") if isinstance(row, Mapping) else None for row in completed
    ]
    if completed_ids != list(contract.STAGE_IDS[:completed_count]):
        return False
    if any(
        not isinstance(row, Mapping) or set(row) != {"stage_id", "receipt"}
        for row in completed
    ):
        return False
    if any(
        not _artifact_record_schema_valid(
            row["receipt"], expected_path=_stage_receipt_path(str(row["stage_id"]))
        )
        for row in completed
    ):
        return False
    if not _completed_stage_prefix_current(completed, outcome=outcome):
        return False
    expected_attempted = (
        None
        if completed_count == len(contract.STAGE_IDS)
        else contract.STAGE_IDS[completed_count]
    )
    if intent.get("attempted_stage") != expected_attempted:
        return False
    if not _terminal_intent_activity_valid(
        activity,
        totals,
        completed_count=completed_count,
        outcome=outcome,
    ):
        return False
    stage_error = intent.get("stage_error")
    failure_error = intent.get("failure_publication_error")
    if outcome == "PASS":
        return (
            completed_count == len(contract.STAGE_IDS)
            and stage_error is None
            and failure_error is None
            and intent.get("next_stage") == "WAIT_SEPARATE_Q3_PROTOCOL"
        )
    return (
        completed_count < len(contract.STAGE_IDS)
        and _error_mapping_valid(stage_error)
        and (failure_error is None or _error_mapping_valid(failure_error))
        and intent.get("next_stage") == "STOP_TERMINAL"
    )


def _terminal_ledger(
    *,
    passed: bool,
    attempted_stage: str | None,
    completed: Sequence[Mapping[str, Any]],
    error: BaseException | None,
    failure_publication_error: Mapping[str, str] | None,
) -> dict[str, Any]:
    candidate_id: str | None = None
    candidate_module: Any = None
    candidate_freeze: Any = None
    final_receipt: Any = None
    candidate_freeze_observation = _candidate_freeze_observation()
    if candidate_freeze_observation.get("state") == "VALID_REGULAR":
        freeze = _mapping(CANDIDATE_FREEZE_PATH)
        candidate_id = freeze.get("candidate_id")
        candidate_module = freeze.get("candidate_module")
        candidate_freeze = candidate_freeze_observation.get("artifact")
    final_path = resolve_relative(contract.FINAL_DEVELOPMENT_RECEIPT_PATH)
    final_observation = _raw_file_observation(final_path, json_payload=True)
    if final_observation.get("state") == "VALID_JSON_REGULAR":
        final_receipt = final_observation.get("artifact")
    attempted_worker_observation = (
        _worker_claim_observation(attempted_stage)
        if attempted_stage is not None
        else None
    )
    failure_observation = None
    failure_artifact = None
    attempted_receipt = None
    attempted_outputs = None
    if attempted_stage is not None:
        descriptor = contract.stage_descriptor(attempted_stage)
        attempted_outputs = _stage_output_observations(attempted_stage)
        receipt_observation = attempted_outputs.get("receipt")
        if isinstance(receipt_observation, Mapping):
            attempted_receipt = receipt_observation.get("artifact")
        if descriptor["kind"] == "development":
            failure_observation = _development_failure_observation(attempted_stage)
            if isinstance(failure_observation, Mapping):
                failure_artifact = failure_observation.get("artifact")
    protocol_freeze_record = _record(PROTOCOL_FREEZE_PATH)
    execution_lock_record = _record(LOCK_PATH)
    pipeline_claim_record = (
        _record(PIPELINE_CLAIM_PATH) if PIPELINE_CLAIM_PATH.is_file() else None
    )
    qualification = _qualification_snapshot()
    preterminal_inventory = _preterminal_run_root_inventory()
    activity_totals = _activity_totals_snapshot()
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.PIPELINE_PASS_STATUS if passed else contract.TERMINAL_FAIL_STATUS
        ),
        "passed": passed,
        "terminal": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "attempted_stage": attempted_stage,
        "completed_stages": [dict(row) for row in completed],
        "completed_stage_count": len(completed),
        "stage_order": list(contract.STAGE_IDS),
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_id": candidate_id,
        "candidate_module": candidate_module,
        "candidate_freeze": candidate_freeze,
        "candidate_freeze_observation": candidate_freeze_observation,
        "final_development_receipt": final_receipt,
        "protocol_freeze": protocol_freeze_record,
        "execution_lock": execution_lock_record,
        "pipeline_claim": pipeline_claim_record,
        **qualification,
        "preterminal_run_root_inventory": preterminal_inventory,
        "error": None if error is None else _error_record(error),
        "activity_totals": activity_totals,
        "stage_activity": [
            copy.deepcopy(_STAGE_ACTIVITY[stage_id])
            for stage_id in contract.STAGE_IDS
            if stage_id in _STAGE_ACTIVITY
        ],
        "attempted_stage_output_observations": attempted_outputs,
        "attempted_stage_worker_claim_observation": attempted_worker_observation,
        "attempted_stage_worker_claim": (
            attempted_worker_observation.get("artifact")
            if isinstance(attempted_worker_observation, Mapping)
            else None
        ),
        "attempted_stage_worker_claim_created": (
            isinstance(attempted_worker_observation, Mapping)
            and attempted_worker_observation.get("artifact") is not None
        ),
        "attempted_stage_receipt": attempted_receipt,
        "attempted_stage_receipt_created": attempted_receipt is not None,
        "attempted_stage_failure_observation": failure_observation,
        "attempted_stage_failure_artifact": failure_artifact,
        "attempted_stage_failure_publication_error": (
            None
            if failure_publication_error is None
            else dict(failure_publication_error)
        ),
        "new_collection_count": 0,
        "development_count": sum(
            contract.stage_descriptor(row["stage_id"])["kind"] == "development"
            for row in completed
        ),
        "environment_reset_calls": activity_totals["environment_reset_calls"],
        "environment_step_calls": activity_totals["environment_step_calls"],
        "raw_sensor_sample_count": activity_totals["raw_sensor_sample_count"],
        "teacher_query_count": activity_totals["teacher_query_count"],
        "actor_fit_stage_calls_attempted": activity_totals[
            "actor_fit_stage_calls_attempted"
        ],
        "actor_fit_executions_confirmed": activity_totals[
            "actor_fit_executions_confirmed"
        ],
        "actor_updates_attempted": activity_totals["actor_updates_attempted"],
        "actor_updates": activity_totals["actor_updates_confirmed"],
        "adamw_epochs_completed": activity_totals["adamw_epochs_completed"],
        "lbfgs_closure_calls": activity_totals["lbfgs_closure_calls"],
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
        "next_stage": "WAIT_SEPARATE_Q3_PROTOCOL" if passed else "STOP_TERMINAL",
    }


def _terminal_failure_structure_gate(ledger: Mapping[str, Any]) -> dict[str, Any]:
    """Validate terminal-FAIL ordering and closed negative authority in memory."""

    completed = ledger.get("completed_stages")
    activity = ledger.get("stage_activity")
    completed_rows = completed if isinstance(completed, list) else []
    activity_rows = activity if isinstance(activity, list) else []
    completed_ids = [
        row.get("stage_id") for row in completed_rows if isinstance(row, Mapping)
    ]
    attempted = ledger.get("attempted_stage")
    expected_attempted = (
        contract.STAGE_IDS[len(completed_rows)]
        if len(completed_rows) < len(contract.STAGE_IDS)
        else None
    )
    activity_ids = [
        row.get("stage_id") for row in activity_rows if isinstance(row, Mapping)
    ]
    descriptor = (
        contract.stage_descriptor(attempted)
        if isinstance(attempted, str) and attempted in contract.STAGE_IDS
        else None
    )
    error = ledger.get("error")
    failure_publication_error = ledger.get("attempted_stage_failure_publication_error")
    publication_error_valid = _error_mapping_valid(failure_publication_error)
    failure_observation = ledger.get("attempted_stage_failure_observation")
    failure_observation_state = (
        failure_observation.get("state")
        if isinstance(failure_observation, Mapping)
        else None
    )
    failure_observation_artifact = (
        failure_observation.get("artifact")
        if isinstance(failure_observation, Mapping)
        else None
    )
    checks = {
        "terminal_identity": ledger.get("status") == contract.TERMINAL_FAIL_STATUS
        and ledger.get("passed") is False
        and ledger.get("terminal") is True,
        "completed_prefix": len(completed_ids) == len(completed_rows)
        and completed_ids == list(contract.STAGE_IDS[: len(completed_rows)]),
        "attempted_immediate": attempted == expected_attempted,
        "activity_prefix": len(activity_ids) == len(activity_rows)
        and activity_ids == list(contract.STAGE_IDS[: len(completed_rows) + 1]),
        "activity_schema": all(
            isinstance(row, Mapping)
            and set(row) == {"stage_id", "stage_kind", *_ACTIVITY_NAMES}
            and all(
                type(row.get(name)) is int and row[name] >= 0
                for name in _ACTIVITY_NAMES
            )
            for row in activity_rows
        ),
        "error_nonempty": _error_mapping_valid(error),
        "stop_terminal": ledger.get("next_stage") == "STOP_TERMINAL",
        "qualification_snapshot": _qualification_snapshot_valid(ledger),
        "prohibitions": _exact_int(ledger.get("new_collection_count"), 0)
        and _exact_int(ledger.get("critic_updates"), 0)
        and _exact_int(ledger.get("ppo_updates"), 0)
        and all(
            ledger.get(name) is False
            for name in (
                "retry_authorized",
                "resume_authorized",
                "rescue_authorized",
                "sweep_authorized",
                "qualification_executed",
                "runtime_promoted",
                "checkpoint_zero_created",
                "positive_morphology_enabled",
            )
        ),
        "failure_evidence": (
            isinstance(descriptor, Mapping)
            and descriptor.get("kind") == "development"
            and ledger.get("attempted_stage_failure_artifact")
            == failure_observation_artifact
            and (
                (
                    failure_observation_state == "VALID"
                    and isinstance(failure_observation_artifact, Mapping)
                    and failure_publication_error is None
                )
                or (
                    failure_observation_state
                    in {
                        "ABSENT",
                        "ZERO_BYTE_REGULAR",
                        "INVALID_JSON_REGULAR",
                        "INVALID_SCHEMA_REGULAR",
                        "UNSAFE_SYMLINK",
                        "UNSAFE_NONREGULAR",
                        "UNSAFE_UNREADABLE_REGULAR",
                    }
                    and publication_error_valid
                )
            )
        )
        or (
            isinstance(descriptor, Mapping)
            and descriptor.get("kind") != "development"
            and failure_observation is None
            and ledger.get("attempted_stage_failure_artifact") is None
            and failure_publication_error is None
        ),
    }
    return {"passed": all(checks.values()), "checks": checks}


def _terminal_publication_failure_payload(
    *,
    terminal_intent: Mapping[str, Any],
    intended_ledger: Mapping[str, Any] | None,
    error: BaseException,
) -> dict[str, Any]:
    intent = copy.deepcopy(dict(terminal_intent))
    intent_valid = _terminal_intent_valid(intent)
    intended = None if intended_ledger is None else copy.deepcopy(dict(intended_ledger))
    intended_record = (
        None
        if intended is None
        else _canonical_payload_record(PIPELINE_LEDGER_PATH, intended)
    )
    ledger_observation = (
        _raw_file_observation(PIPELINE_LEDGER_PATH, json_payload=True)
        if intended_record is None
        else _expected_json_observation(
            PIPELINE_LEDGER_PATH, expected_record=intended_record
        )
    )
    execution_lock_observation = _raw_file_observation(LOCK_PATH, json_payload=True)
    protocol_freeze_observation = _raw_file_observation(
        PROTOCOL_FREEZE_PATH, json_payload=True
    )
    pipeline_claim_observation = _pipeline_claim_observation()
    qualification = _qualification_snapshot()
    preterminal_inventory = _preterminal_run_root_inventory()
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.TERMINAL_FAIL_STATUS,
        "passed": False,
        "terminal": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "phase": "TERMINALIZATION",
        "intended_outcome": intent.get("intended_outcome"),
        "terminal_intent": intent,
        "terminal_intent_valid": intent_valid,
        "intended_ledger": intended,
        "intended_ledger_record": intended_record,
        "pipeline_ledger_observation": ledger_observation,
        "completed_stages": copy.deepcopy(intent.get("completed_stages")),
        "completed_stage_count": intent.get("completed_stage_count"),
        "attempted_stage": intent.get("attempted_stage"),
        "error": _error_record(error),
        "execution_lock": execution_lock_observation.get("artifact"),
        "execution_lock_observation": execution_lock_observation,
        "protocol_freeze": protocol_freeze_observation.get("artifact"),
        "protocol_freeze_observation": protocol_freeze_observation,
        "pipeline_claim": pipeline_claim_observation.get("artifact"),
        "pipeline_claim_observation": pipeline_claim_observation,
        "activity_totals": copy.deepcopy(intent.get("activity_totals")),
        "stage_activity": copy.deepcopy(intent.get("stage_activity")),
        **qualification,
        "preterminal_run_root_inventory": preterminal_inventory,
        "new_collection_count": 0,
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


def _terminal_publication_failure_payload_valid(payload: Any) -> bool:
    if (
        not isinstance(payload, Mapping)
        or set(payload) != _TERMINAL_PUBLICATION_FAILURE_FIELDS
    ):
        return False
    intent = payload.get("terminal_intent")
    if not _terminal_intent_snapshot_schema_valid(intent):
        return False
    intent = dict(intent)
    intent_valid = _terminal_intent_valid(intent)
    if payload.get("terminal_intent_valid") is not intent_valid:
        return False
    intended_value = payload.get("intended_ledger")
    if intended_value is None:
        intended = None
        expected_record = None
        observation = _raw_file_observation(PIPELINE_LEDGER_PATH, json_payload=True)
    elif isinstance(intended_value, Mapping):
        if not intent_valid:
            return False
        intended = dict(intended_value)
        if not _terminal_ledger_schema_exact(intended):
            return False
        expected_record = _canonical_payload_record(PIPELINE_LEDGER_PATH, intended)
        observation = _expected_json_observation(
            PIPELINE_LEDGER_PATH, expected_record=expected_record
        )
        ledger_intent = _terminal_intent_from_ledger(intended)
        if not _strict_equal(ledger_intent, intent):
            return False
        try:
            intended_current = _strict_equal(
                _verify_terminal_ledger_payload(
                    intended,
                    require_publication_failure_absent=False,
                ),
                intended,
            )
        except BaseException:
            intended_current = False
    else:
        return False
    if intended_value is None:
        intended_current = True
    error = payload.get("error")
    stored_inventory = payload.get("preterminal_run_root_inventory")
    current_inventory = _preterminal_run_root_inventory()
    execution_lock_observation = _raw_file_observation(LOCK_PATH, json_payload=True)
    protocol_freeze_observation = _raw_file_observation(
        PROTOCOL_FREEZE_PATH, json_payload=True
    )
    pipeline_claim_observation = _pipeline_claim_observation()
    source_file_states = {
        "ABSENT",
        "VALID_JSON_REGULAR",
        "ZERO_BYTE_REGULAR",
        "INVALID_JSON_REGULAR",
        "INVALID_SCHEMA_REGULAR",
        "UNSAFE_SYMLINK",
        "UNSAFE_NONREGULAR",
        "UNSAFE_UNREADABLE_REGULAR",
    }
    claim_states = {
        "ABSENT",
        "PARTIAL_OR_INVALID_REGULAR",
        "UNSAFE_NONREGULAR",
        "UNSAFE_UNREADABLE_REGULAR",
        "VALID_REGULAR",
    }
    try:
        source_observations_current = (
            _strict_equal(
                payload.get("execution_lock_observation"),
                execution_lock_observation,
            )
            and _strict_equal(
                payload.get("protocol_freeze_observation"),
                protocol_freeze_observation,
            )
            and _strict_equal(
                payload.get("pipeline_claim_observation"),
                pipeline_claim_observation,
            )
            and execution_lock_observation.get("state") in source_file_states
            and protocol_freeze_observation.get("state") in source_file_states
            and pipeline_claim_observation.get("state") in claim_states
            and not os.path.lexists(PIPELINE_CLAIM_FAILURE_PATH)
        )
        if intent_valid:
            pipeline_claim_current = source_observations_current and (
                execution_lock_observation.get("state") == "VALID_JSON_REGULAR"
                and protocol_freeze_observation.get("state") == "VALID_JSON_REGULAR"
                and pipeline_claim_observation.get("state") == "VALID_REGULAR"
            )
            _verify_worker_claims_root_exact(
                completed_stage_ids=[
                    str(row["stage_id"]) for row in intent["completed_stages"]
                ],
                attempted_stage=intent.get("attempted_stage"),
            )
            worker_claims_current = True
        else:
            pipeline_claim_current = source_observations_current
            worker_claims_current = _worker_claims_root_state() == "DIRECTORY"
    except BaseException:
        pipeline_claim_current = False
        worker_claims_current = False
    return (
        payload.get("schema_version") == contract.SCHEMA_VERSION
        and type(payload.get("schema_version")) is type(contract.SCHEMA_VERSION)
        and payload.get("status") == contract.TERMINAL_FAIL_STATUS
        and payload.get("passed") is False
        and payload.get("terminal") is True
        and payload.get("protocol_id") == contract.PROTOCOL_ID
        and payload.get("pipeline_id") == contract.PIPELINE_ID
        and payload.get("phase") == "TERMINALIZATION"
        and payload.get("intended_outcome") == intent.get("intended_outcome")
        and payload.get("terminal_intent_valid") is intent_valid
        and intended_current
        and pipeline_claim_current
        and worker_claims_current
        and _strict_equal(payload.get("intended_ledger_record"), expected_record)
        and _strict_equal(payload.get("pipeline_ledger_observation"), observation)
        and observation.get("state")
        in {
            "ABSENT",
            "VALID",
            "VALID_JSON_REGULAR",
            "ZERO_BYTE_REGULAR",
            "INVALID_JSON_REGULAR",
            "INVALID_SCHEMA_REGULAR",
            "UNSAFE_SYMLINK",
            "UNSAFE_NONREGULAR",
            "UNSAFE_UNREADABLE_REGULAR",
        }
        and _strict_equal(
            payload.get("completed_stages"), intent.get("completed_stages")
        )
        and payload.get("completed_stage_count") == intent.get("completed_stage_count")
        and type(payload.get("completed_stage_count")) is int
        and payload.get("attempted_stage") == intent.get("attempted_stage")
        and _error_mapping_valid(error)
        and _strict_equal(
            payload.get("execution_lock"),
            execution_lock_observation.get("artifact"),
        )
        and _strict_equal(
            payload.get("protocol_freeze"),
            protocol_freeze_observation.get("artifact"),
        )
        and _strict_equal(
            payload.get("pipeline_claim"),
            pipeline_claim_observation.get("artifact"),
        )
        and _strict_equal(payload.get("activity_totals"), intent.get("activity_totals"))
        and _strict_equal(payload.get("stage_activity"), intent.get("stage_activity"))
        and _qualification_snapshot_valid(payload)
        and _preterminal_run_root_inventory_schema_valid(stored_inventory)
        and _strict_equal(stored_inventory, current_inventory)
        and type(payload.get("new_collection_count")) is int
        and payload.get("new_collection_count") == 0
        and type(payload.get("critic_updates")) is int
        and payload.get("critic_updates") == 0
        and type(payload.get("ppo_updates")) is int
        and payload.get("ppo_updates") == 0
        and all(
            payload.get(name) is False
            for name in (
                "retry_authorized",
                "resume_authorized",
                "rescue_authorized",
                "sweep_authorized",
                "qualification_executed",
                "runtime_promoted",
                "checkpoint_zero_created",
                "positive_morphology_enabled",
            )
        )
        and payload.get("next_stage") == "STOP_TERMINAL"
    )


def verify_terminal_publication_failure() -> dict[str, Any]:
    payload = _mapping(PIPELINE_TERMINAL_PUBLICATION_FAILURE_PATH)
    if not _terminal_publication_failure_payload_valid(payload):
        raise V12R5ExecutionError("terminal publication failure evidence drifted")
    return payload


def _publish_terminal_publication_failure(
    *,
    terminal_intent: Mapping[str, Any],
    intended_ledger: Mapping[str, Any] | None,
    error: BaseException,
) -> None:
    failure = _terminal_publication_failure_payload(
        terminal_intent=terminal_intent,
        intended_ledger=intended_ledger,
        error=error,
    )
    if not _terminal_publication_failure_payload_valid(failure):
        raise V12R5ExecutionError(
            "terminal publication failure payload failed in-memory gate"
        ) from error
    freezer._write_json_exclusive(PIPELINE_TERMINAL_PUBLICATION_FAILURE_PATH, failure)
    verify_terminal_publication_failure()


def _commit_terminal_ledger(
    intended_ledger: Mapping[str, Any], *, terminal_intent: Mapping[str, Any]
) -> dict[str, Any]:
    intended: dict[str, Any] | None = None
    try:
        intended = _verify_terminal_ledger_payload(
            intended_ledger,
            require_publication_failure_absent=True,
        )
        freezer._write_json_exclusive(PIPELINE_LEDGER_PATH, intended)
        return verify_terminal_ledger()
    except BaseException as exc:
        _publish_terminal_publication_failure(
            terminal_intent=terminal_intent,
            intended_ledger=intended,
            error=exc,
        )
        raise


def _publish_terminal_ledger(intended_ledger: Mapping[str, Any]) -> dict[str, Any]:
    """Compatibility entry point with preverification inside the failure boundary."""

    intent = _terminal_intent_from_ledger(intended_ledger)
    if not _terminal_intent_valid(intent):
        error = V12R5ExecutionError("terminal intent derived from ledger is invalid")
        _publish_terminal_publication_failure(
            terminal_intent=intent,
            intended_ledger=None,
            error=error,
        )
        raise error
    return _commit_terminal_ledger(intended_ledger, terminal_intent=intent)


def _terminalize(
    *,
    passed: bool,
    attempted_stage: str | None,
    completed: Sequence[Mapping[str, Any]],
    error: BaseException | None,
    failure_publication_error: Mapping[str, Any] | None,
) -> dict[str, Any]:
    intent = _terminal_intent(
        passed=passed,
        attempted_stage=attempted_stage,
        completed=completed,
        error=error,
        failure_publication_error=failure_publication_error,
    )
    if not _terminal_intent_valid(intent):
        intent_error = V12R5ExecutionError("terminal intent failed in-memory gate")
        _publish_terminal_publication_failure(
            terminal_intent=intent,
            intended_ledger=None,
            error=intent_error,
        )
        raise intent_error
    intended: dict[str, Any] | None = None
    try:
        intended = _terminal_ledger(
            passed=passed,
            attempted_stage=attempted_stage,
            completed=completed,
            error=error,
            failure_publication_error=failure_publication_error,
        )
    except BaseException as exc:
        _publish_terminal_publication_failure(
            terminal_intent=intent,
            intended_ledger=None,
            error=exc,
        )
        raise
    return _commit_terminal_ledger(intended, terminal_intent=intent)


def execute_pipeline_once() -> dict[str, Any]:
    _reset_activity()
    verify_execution_lock(require_run_root_absent=True)
    token_hash, _claim = _claim_run_root()
    completed: list[dict[str, Any]] = []
    attempted: str | None = None
    try:
        for stage_id in contract.STAGE_IDS:
            attempted = stage_id
            _begin_stage_activity(stage_id)
            _write_worker_claim(stage_id, token_hash)
            if not _qualification_unopened():
                raise V12R5ExecutionError(
                    "Q2/Q3 execution output opened before R5 stage"
                )
            receipt = _run_stage(stage_id)
            if receipt.get("passed") is not True:
                raise V12R5ExecutionError(f"stage returned non-PASS: {stage_id}")
            _verify_stage_receipt_claim_bindings(stage_id, receipt)
            completed.append(
                {
                    "stage_id": stage_id,
                    "receipt": _record(_stage_receipt_path(stage_id)),
                }
            )
    except BaseException as exc:
        failure_publication_error = None
        if attempted is not None:
            failure_publication_error = _attempt_development_failure_artifact(
                attempted, exc
            )
        _terminalize(
            passed=False,
            attempted_stage=attempted,
            completed=completed,
            error=exc,
            failure_publication_error=failure_publication_error,
        )
        raise
    return _terminalize(
        passed=True,
        attempted_stage=attempted,
        completed=completed,
        error=None,
        failure_publication_error=None,
    )


def _verify_terminal_ledger_payload(
    ledger: Mapping[str, Any], *, require_publication_failure_absent: bool
) -> dict[str, Any]:
    """Validate a terminal payload against current artifact closure."""

    verify_execution_lock(require_run_root_absent=False)
    ledger = dict(ledger)
    passed = ledger.get("passed") is True
    expected_status = (
        contract.PIPELINE_PASS_STATUS if passed else contract.TERMINAL_FAIL_STATUS
    )
    completed = ledger.get("completed_stages")
    stage_activity = ledger.get("stage_activity")
    totals = ledger.get("activity_totals")
    if (
        not _terminal_ledger_schema_exact(ledger)
        or ledger.get("schema_version") != contract.SCHEMA_VERSION
        or type(ledger.get("schema_version")) is not type(contract.SCHEMA_VERSION)
        or type(ledger.get("passed")) is not bool
        or ledger.get("status") != expected_status
        or ledger.get("terminal") is not True
        or ledger.get("protocol_id") != contract.PROTOCOL_ID
        or ledger.get("pipeline_id") != contract.PIPELINE_ID
        or ledger.get("stage_order") != list(contract.STAGE_IDS)
        or ledger.get("candidate_selection_rule") != contract.CANDIDATE_SELECTION_RULE
        or not isinstance(completed, list)
        or not _exact_int(ledger.get("completed_stage_count"), len(completed))
        or not isinstance(stage_activity, list)
        or not isinstance(totals, Mapping)
        or set(totals) != set(_ACTIVITY_NAMES)
        or any(
            type(totals.get(name)) is not int or totals[name] < 0
            for name in _ACTIVITY_NAMES
        )
    ):
        raise V12R5ExecutionError("terminal ledger schema/identity drifted")
    if not _terminal_intent_valid(_terminal_intent_from_ledger(ledger)):
        raise V12R5ExecutionError("terminal ledger intent closure drifted")
    expected_completed: list[dict[str, Any]] = []
    for stage_id in contract.STAGE_IDS[: len(completed)]:
        receipt_path = _stage_receipt_path(stage_id)
        if not receipt_path.is_file():
            raise V12R5ExecutionError(f"completed receipt missing: {stage_id}")
        _verify_stage_receipt_claim_bindings(stage_id)
        expected_completed.append(
            {"stage_id": stage_id, "receipt": _record(receipt_path)}
        )
    if not _strict_equal(completed, expected_completed):
        raise V12R5ExecutionError("terminal ledger completed-stage closure drifted")
    attempted = ledger.get("attempted_stage")
    if passed:
        expected_activity_ids = list(contract.STAGE_IDS)
    else:
        if len(completed) >= len(contract.STAGE_IDS):
            raise V12R5ExecutionError(
                "terminal FAIL cannot follow all completed stages"
            )
        expected_attempted = contract.STAGE_IDS[len(completed)]
        if attempted != expected_attempted:
            raise V12R5ExecutionError(
                "terminal FAIL attempted stage is not immediately after completed prefix"
            )
        expected_activity_ids = list(contract.STAGE_IDS[: len(completed) + 1])
    activity_ids = [
        row.get("stage_id") for row in stage_activity if isinstance(row, Mapping)
    ]
    if (
        len(activity_ids) != len(stage_activity)
        or activity_ids != expected_activity_ids
        or any(
            set(row) != {"stage_id", "stage_kind", *_ACTIVITY_NAMES}
            for row in stage_activity
        )
        or any(
            row.get("stage_kind")
            != contract.stage_descriptor(str(row.get("stage_id")))["kind"]
            for row in stage_activity
        )
        or any(
            type(row.get(name)) is not int or row[name] < 0
            for row in stage_activity
            for name in _ACTIVITY_NAMES
        )
    ):
        raise V12R5ExecutionError("terminal ledger stage activity drifted")
    recomputed_totals = {
        name: sum(int(row[name]) for row in stage_activity) for name in _ACTIVITY_NAMES
    }
    top_level_counters = {
        "environment_reset_calls": ledger.get("environment_reset_calls"),
        "environment_step_calls": ledger.get("environment_step_calls"),
        "raw_sensor_sample_count": ledger.get("raw_sensor_sample_count"),
        "teacher_query_count": ledger.get("teacher_query_count"),
        "actor_fit_stage_calls_attempted": ledger.get(
            "actor_fit_stage_calls_attempted"
        ),
        "actor_fit_executions_confirmed": ledger.get("actor_fit_executions_confirmed"),
        "actor_updates_attempted": ledger.get("actor_updates_attempted"),
        "actor_updates_confirmed": ledger.get("actor_updates"),
        "adamw_epochs_completed": ledger.get("adamw_epochs_completed"),
        "lbfgs_closure_calls": ledger.get("lbfgs_closure_calls"),
    }
    if not _strict_equal(dict(totals), recomputed_totals) or not _strict_equal(
        top_level_counters, recomputed_totals
    ):
        raise V12R5ExecutionError("terminal ledger live activity closure drifted")
    completed_worker_observations = {
        row["stage_id"]: _worker_claim_observation(row["stage_id"]) for row in completed
    }
    if any(
        observation.get("state") != "VALID_REGULAR"
        for observation in completed_worker_observations.values()
    ):
        raise V12R5ExecutionError("completed worker claim semantics drifted")
    expected_worker_observation = (
        _worker_claim_observation(attempted) if isinstance(attempted, str) else None
    )
    expected_worker = (
        expected_worker_observation.get("artifact")
        if isinstance(expected_worker_observation, Mapping)
        else None
    )
    if (
        not _strict_equal(
            ledger.get("attempted_stage_worker_claim_observation"),
            expected_worker_observation,
        )
        or (
            isinstance(expected_worker_observation, Mapping)
            and expected_worker_observation.get("state")
            not in {
                "ABSENT",
                "PARTIAL_OR_INVALID_REGULAR",
                "UNSAFE_NONREGULAR",
                "UNSAFE_UNREADABLE_REGULAR",
                "VALID_REGULAR",
            }
        )
        or not _strict_equal(
            ledger.get("attempted_stage_worker_claim"), expected_worker
        )
        or ledger.get("attempted_stage_worker_claim_created")
        is not (expected_worker is not None)
    ):
        raise V12R5ExecutionError("terminal ledger attempted worker binding drifted")
    _verify_worker_claims_root_exact(
        completed_stage_ids=[row["stage_id"] for row in completed],
        attempted_stage=attempted if isinstance(attempted, str) else None,
    )
    expected_outputs = (
        _stage_output_observations(attempted) if isinstance(attempted, str) else None
    )
    expected_receipt_observation = (
        expected_outputs.get("receipt")
        if isinstance(expected_outputs, Mapping)
        else None
    )
    expected_attempted_receipt = (
        expected_receipt_observation.get("artifact")
        if isinstance(expected_receipt_observation, Mapping)
        else None
    )
    if (
        not _strict_equal(
            ledger.get("attempted_stage_output_observations"), expected_outputs
        )
        or not _strict_equal(
            ledger.get("attempted_stage_receipt"), expected_attempted_receipt
        )
        or ledger.get("attempted_stage_receipt_created")
        is not (expected_attempted_receipt is not None)
    ):
        raise V12R5ExecutionError("terminal ledger attempted output binding drifted")
    if isinstance(attempted, str) and not _attempted_stage_ownership_coherent(
        attempted
    ):
        raise V12R5ExecutionError("terminal ledger attempted ownership drifted")
    current_candidate_observation = _candidate_freeze_observation()
    current_candidate_freeze = (
        current_candidate_observation.get("artifact")
        if current_candidate_observation.get("state") == "VALID_REGULAR"
        else None
    )
    current_freeze_payload = (
        _mapping(CANDIDATE_FREEZE_PATH)
        if current_candidate_observation.get("state") == "VALID_REGULAR"
        else None
    )
    candidate_stage_index = contract.STAGE_IDS.index("freeze_case_balanced_candidate")
    candidate_state = current_candidate_observation.get("state")
    if passed or len(completed) > candidate_stage_index:
        candidate_phase_valid = candidate_state == "VALID_REGULAR"
    elif attempted == "freeze_case_balanced_candidate":
        candidate_phase_valid = candidate_state in {
            "ABSENT",
            "ZERO_BYTE_REGULAR",
            "INVALID_JSON_REGULAR",
            "INVALID_SCHEMA_REGULAR",
            "UNSAFE_SYMLINK",
            "UNSAFE_NONREGULAR",
            "UNSAFE_UNREADABLE_REGULAR",
            "VALID_REGULAR",
        }
    else:
        candidate_phase_valid = candidate_state == "ABSENT"
    final_path = resolve_relative(contract.FINAL_DEVELOPMENT_RECEIPT_PATH)
    current_final_observation = _raw_file_observation(final_path, json_payload=True)
    current_final = (
        current_final_observation.get("artifact")
        if current_final_observation.get("state") == "VALID_JSON_REGULAR"
        else None
    )
    pipeline_claim = _mapping(PIPELINE_CLAIM_PATH)
    stored_inventory = ledger.get("preterminal_run_root_inventory")
    current_inventory = _preterminal_run_root_inventory()
    prohibitions = {
        "new_collection_count": _exact_int(ledger.get("new_collection_count"), 0),
        "critic_updates": _exact_int(ledger.get("critic_updates"), 0),
        "ppo_updates": _exact_int(ledger.get("ppo_updates"), 0),
        "retry_authorized": ledger.get("retry_authorized") is False,
        "resume_authorized": ledger.get("resume_authorized") is False,
        "rescue_authorized": ledger.get("rescue_authorized") is False,
        "sweep_authorized": ledger.get("sweep_authorized") is False,
        "qualification_executed": ledger.get("qualification_executed") is False,
        "runtime_promoted": ledger.get("runtime_promoted") is False,
        "checkpoint_zero_created": ledger.get("checkpoint_zero_created") is False,
        "positive_morphology_enabled": ledger.get("positive_morphology_enabled")
        is False,
    }
    common_checks = {
        "protocol_current": _strict_equal(
            ledger.get("protocol_freeze"), _record(PROTOCOL_FREEZE_PATH)
        ),
        "lock_current": _strict_equal(ledger.get("execution_lock"), _record(LOCK_PATH)),
        "pipeline_claim_current": _strict_equal(
            ledger.get("pipeline_claim"), _record(PIPELINE_CLAIM_PATH)
        )
        and _pipeline_claim_payload_valid(pipeline_claim),
        "claim_failure_absent": not os.path.lexists(PIPELINE_CLAIM_FAILURE_PATH),
        "terminal_publication_failure_absent": (
            not require_publication_failure_absent
            or not os.path.lexists(PIPELINE_TERMINAL_PUBLICATION_FAILURE_PATH)
        ),
        "candidate_freeze_current": _strict_equal(
            ledger.get("candidate_freeze_observation"),
            current_candidate_observation,
        )
        and _strict_equal(ledger.get("candidate_freeze"), current_candidate_freeze)
        and ledger.get("candidate_id")
        == (
            current_freeze_payload.get("candidate_id")
            if isinstance(current_freeze_payload, Mapping)
            else None
        )
        and _strict_equal(
            ledger.get("candidate_module"),
            current_freeze_payload.get("candidate_module")
            if isinstance(current_freeze_payload, Mapping)
            else None,
        ),
        "candidate_phase": candidate_phase_valid,
        "final_current": _strict_equal(
            ledger.get("final_development_receipt"), current_final
        ),
        "qualification_snapshot": _qualification_snapshot_valid(ledger),
        "preterminal_inventory_current": (
            _preterminal_run_root_inventory_schema_valid(stored_inventory)
            and _strict_equal(stored_inventory, current_inventory)
            and (
                not passed
                or (
                    stored_inventory.get("complete") is True
                    and stored_inventory.get("semantic_closed") is True
                )
            )
        ),
        "prohibitions": all(prohibitions.values()),
        "next_stage": ledger.get("next_stage")
        == ("WAIT_SEPARATE_Q3_PROTOCOL" if passed else "STOP_TERMINAL"),
        "development_count": _exact_int(
            ledger.get("development_count"),
            sum(
                contract.stage_descriptor(row["stage_id"])["kind"] == "development"
                for row in completed
            ),
        ),
        "runtime_sources": _external_runtime_sources_exact(),
    }
    if not all(common_checks.values()):
        failed = [name for name, value in common_checks.items() if not value]
        raise V12R5ExecutionError(f"terminal ledger common closure drifted: {failed}")
    if passed:
        module = _tree(resolve_relative(contract.CANDIDATE_MODULE_PATH))
        identity = contract.candidate_id(module["tree_sha256"])
        freeze = _mapping(CANDIDATE_FREEZE_PATH)
        final = _mapping(final_path)
        final_summary_path = RUN_ROOT / "final_development_summary.json"
        final_gate_path = RUN_ROOT / "final_development_gate.json"
        final_summary = _mapping(final_summary_path)
        final_gate = _mapping(final_gate_path)
        expected_final_gate = contract.aggregate_development_gate(final_summary)
        pass_checks = {
            "all_stages": len(completed) == len(contract.STAGE_IDS),
            "attempted_final": attempted == contract.STAGE_IDS[-1],
            "error_absent": ledger.get("error") is None,
            "failure_artifact_absent": ledger.get("attempted_stage_failure_artifact")
            is None,
            "failure_observation_absent": ledger.get(
                "attempted_stage_failure_observation"
            )
            is None,
            "failure_publication_error_absent": ledger.get(
                "attempted_stage_failure_publication_error"
            )
            is None,
            "development_failure_artifacts_absent": all(
                not os.path.lexists(
                    resolve_relative(contract.DEVELOPMENT_PATHS[case_id])
                    / "failure.json"
                )
                for case_id in contract.DEVELOPMENT_CASE_IDS
            ),
            "freeze_status": freeze.get("status") == contract.CANDIDATE_FREEZE_STATUS
            and freeze.get("passed") is True,
            "final_status": final.get("status")
            == contract.FINAL_DEVELOPMENT_PASS_STATUS
            and final.get("passed") is True,
            "candidate_identity": ledger.get("candidate_id") == identity
            and freeze.get("candidate_id") == identity
            and final.get("candidate_id") == identity,
            "candidate_tree": _strict_equal(ledger.get("candidate_module"), module)
            and _strict_equal(freeze.get("candidate_module"), module)
            and _strict_equal(final.get("candidate_module"), module),
            "candidate_freeze_record": _strict_equal(
                ledger.get("candidate_freeze"), _record(CANDIDATE_FREEZE_PATH)
            ),
            "final_record": _strict_equal(
                ledger.get("final_development_receipt"), _record(final_path)
            ),
            "final_candidate_freeze": _strict_equal(
                final.get("candidate_freeze"), _record(CANDIDATE_FREEZE_PATH)
            ),
            "final_gate_recomputed": _strict_equal(final_gate, expected_final_gate)
            and final_gate.get("passed") is True,
            "final_receipt_bindings": _strict_equal(
                final.get("summary"), _record(final_summary_path)
            )
            and _strict_equal(final.get("gate"), _record(final_gate_path))
            and _strict_equal(
                final.get("rollout_bindings"), final_summary.get("rollout_bindings")
            )
            and _strict_equal(final.get("pipeline_claim"), _record(PIPELINE_CLAIM_PATH))
            and _strict_equal(
                final.get("worker_claim"),
                _record(_claim_path("finalize_development")),
            )
            and _exact_int(final.get("pure_policy_trace_row_count"), 3000)
            and all(
                _exact_int(final.get(name), 0)
                and _exact_int(final_summary.get(name), 0)
                for name in contract.PURE_POLICY_COUNTER_FIELDS
            ),
            "activity": recomputed_totals["environment_reset_calls"] == 6
            and recomputed_totals["environment_step_calls"] == 3000
            and recomputed_totals["raw_sensor_sample_count"] == 30_000
            and recomputed_totals["teacher_query_count"] == 0
            and recomputed_totals["actor_fit_stage_calls_attempted"] == 1
            and recomputed_totals["actor_fit_executions_confirmed"] == 1
            and recomputed_totals["actor_updates_attempted"] == 1
            and recomputed_totals["actor_updates_confirmed"] == 1
            and recomputed_totals["adamw_epochs_completed"] == 3000
            and recomputed_totals["lbfgs_closure_calls"] >= 1,
            "q2_unopened_at_terminal": ledger.get("q2_paths_opened") == [],
            "q3_unopened_at_terminal": ledger.get("q3_paths_opened") == [],
            "qualification_violation_absent": ledger.get(
                "qualification_violation_detected"
            )
            is False,
            "no_collection": _exact_int(ledger.get("new_collection_count"), 0),
        }
        if not all(pass_checks.values()):
            failed = [name for name, value in pass_checks.items() if not value]
            raise V12R5ExecutionError(f"terminal PASS ledger drifted: {failed}")
    else:
        structure = _terminal_failure_structure_gate(ledger)
        if structure.get("passed") is not True:
            failed = [name for name, value in structure["checks"].items() if not value]
            raise V12R5ExecutionError(f"terminal FAIL structure drifted: {failed}")
        error = ledger.get("error")
        descriptor = contract.stage_descriptor(str(attempted))
        expected_failure_observation = (
            _development_failure_observation(str(attempted))
            if descriptor["kind"] == "development"
            else None
        )
        expected_failure = (
            expected_failure_observation.get("artifact")
            if isinstance(expected_failure_observation, Mapping)
            else None
        )
        failure_state = (
            expected_failure_observation.get("state")
            if isinstance(expected_failure_observation, Mapping)
            else None
        )
        publication_error = ledger.get("attempted_stage_failure_publication_error")
        publication_error_valid = _error_mapping_valid(publication_error)
        fail_checks = {
            "error": _error_mapping_valid(error),
            "attempted_failure": (
                descriptor["kind"] == "development"
                and (
                    (
                        failure_state == "VALID"
                        and expected_failure is not None
                        and publication_error is None
                    )
                    or (
                        failure_state
                        in {
                            "ABSENT",
                            "ZERO_BYTE_REGULAR",
                            "INVALID_JSON_REGULAR",
                            "INVALID_SCHEMA_REGULAR",
                            "UNSAFE_SYMLINK",
                            "UNSAFE_NONREGULAR",
                            "UNSAFE_UNREADABLE_REGULAR",
                        }
                        and publication_error_valid
                    )
                )
            )
            or (
                descriptor["kind"] != "development"
                and expected_failure is None
                and expected_failure_observation is None
                and publication_error is None
            ),
            "failure_observation_binding": _strict_equal(
                ledger.get("attempted_stage_failure_observation"),
                expected_failure_observation,
            ),
            "failure_binding": _strict_equal(
                ledger.get("attempted_stage_failure_artifact"), expected_failure
            ),
            "publication_error_schema": publication_error is None
            or publication_error_valid,
        }
        if not all(fail_checks.values()):
            failed = [name for name, value in fail_checks.items() if not value]
            raise V12R5ExecutionError(f"terminal FAIL ledger drifted: {failed}")
    return ledger


def verify_terminal_ledger() -> dict[str, Any]:
    """Verify the canonical terminal outcome; failure evidence dominates."""

    if os.path.lexists(PIPELINE_TERMINAL_PUBLICATION_FAILURE_PATH):
        verify_terminal_publication_failure()
        raise V12R5ExecutionError(
            "terminal publication failure evidence dominates pipeline ledger"
        )
    return _verify_terminal_ledger_payload(
        _mapping(PIPELINE_LEDGER_PATH),
        require_publication_failure_absent=True,
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--prepare-lock", action="store_true")
    group.add_argument("--verify-lock", action="store_true")
    group.add_argument("--execute", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.prepare_lock:
        payload = prepare_execution_lock()
    elif args.verify_lock:
        payload = verify_execution_lock()
    else:
        payload = execute_pipeline_once()
    print(forensic.canonical_json_bytes(payload).decode("utf-8"))
    return 0 if payload.get("passed") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "V12R5ExecutionError",
    "build_execution_lock",
    "execute_pipeline_once",
    "main",
    "prepare_execution_lock",
    "resolve_relative",
    "verify_execution_lock",
    "verify_pipeline_claim_failure",
    "verify_terminal_ledger",
    "verify_terminal_publication_failure",
]
