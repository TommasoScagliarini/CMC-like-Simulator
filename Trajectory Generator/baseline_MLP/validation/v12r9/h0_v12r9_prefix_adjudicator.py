"""Read-only adjudication of the immutable terminal V12R8 attempt.

The V12R8 minus rollout is never resumed or retried.  This module proves that
its 252-step prefix is closed and that the terminal error has exactly one
cause: the producer persisted a truthful full candidate tree while the legacy
verifier expected the three-field projection inherited from V12R7.
"""

from __future__ import annotations

import copy
import sys
from pathlib import Path
from typing import Any, Mapping


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
LOCAL_VALIDATION_ROOT = (
    REPO_ROOT / "Trajectory Generator" / "baseline_MLP" / "validation"
)
for _root in (
    REPO_ROOT,
    REPO_ROOT / "validation",
    REPO_ROOT / "Trajectory Generator" / "baseline_MLP",
    LOCAL_VALIDATION_ROOT,
    LOCAL_VALIDATION_ROOT / "v12r3",
    LOCAL_VALIDATION_ROOT / "v12r6",
    LOCAL_VALIDATION_ROOT / "v12r7",
    LOCAL_VALIDATION_ROOT / "v12r8",
    Path(__file__).resolve().parent,
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_forensic_rollout as forensic  # noqa: E402
import h0_primary_split_v12r3_autonomy_recovery_contract as replay_contract  # noqa: E402
import h0_primary_split_v12r3_pure_probe_observer_labeler as observer  # noqa: E402
import h0_v12r8_prefix_adjudicator as _r8_plus_adjudicator  # noqa: E402
import h0_v12r8_recovery_contract as r8_contract  # noqa: E402
import h0_v12r8_recovery_probe as r8_probe  # noqa: E402
import h0_v12r9_recovery_contract as contract  # noqa: E402


DETECTOR_ANOMALY_FIELDS = _r8_plus_adjudicator.DETECTOR_ANOMALY_FIELDS
RUNTIME_ANOMALY_FIELDS = _r8_plus_adjudicator.RUNTIME_ANOMALY_FIELDS
PROJECTED_ANOMALY_FIELDS = _r8_plus_adjudicator.PROJECTED_ANOMALY_FIELDS
ADJUDICATION_STATUS = "PASS_H0_V12R9_R8_MINUS_PREFIX_ADJUDICATION"
PLUS_IMPORT_STATUS = "PASS_H0_V12R9_R8_PLUS_LABEL_IMPORT"
ROOT_CAUSE = "STRICT_FULL_TREE_VS_LOCKED_PROJECTION_EQUALITY"


class V12R9AdjudicationError(RuntimeError):
    """Raised when immutable R8 evidence cannot be admitted fail closed."""


def _inside(path: str | Path, root: Path, *, label: str) -> Path:
    target = Path(path).expanduser()
    if not target.is_absolute():
        target = root / target
    target = target.resolve()
    try:
        target.relative_to(root)
    except ValueError as exc:
        raise V12R9AdjudicationError(f"{label} escaped artifact_root") from exc
    return target


def _strict_mapping(path: Path) -> dict[str, Any]:
    try:
        value = forensic.strict_json_load(path)
    except Exception as exc:
        raise V12R9AdjudicationError(f"unreadable JSON object: {path}") from exc
    if not isinstance(value, Mapping):
        raise V12R9AdjudicationError(f"expected JSON object: {path}")
    return dict(value)


def _strict_sequence(path: Path) -> list[Any]:
    try:
        value = forensic.strict_json_load(path)
    except Exception as exc:
        raise V12R9AdjudicationError(f"unreadable JSON sequence: {path}") from exc
    if not isinstance(value, list):
        raise V12R9AdjudicationError(f"expected JSON sequence: {path}")
    return list(value)


def _record(path: Path, root: Path) -> dict[str, Any]:
    try:
        return forensic.artifact_record(path, artifact_root=root)
    except Exception as exc:
        raise V12R9AdjudicationError(f"invalid locked artifact: {path}") from exc


def _assert_locked_record(
    name: str, expected: Mapping[str, Any], root: Path
) -> dict[str, Any]:
    observed = _record(_inside(str(expected["path"]), root, label=name), root)
    if observed != dict(expected):
        raise V12R9AdjudicationError(f"locked R8 artifact drifted: {name}")
    return observed


def _tree_record(path: Path, root: Path) -> dict[str, Any]:
    try:
        return r8_probe._tree_artifact(path, root)
    except Exception as exc:
        raise V12R9AdjudicationError("candidate tree attestation failed") from exc


def _zero_int(value: Any) -> bool:
    return type(value) is int and value == 0


def normalize_v26_prefix_summary(
    summary: Mapping[str, Any], *, expected_steps: int
) -> dict[str, Any]:
    """Reuse the frozen projection rules, but publish an R9 identity."""

    try:
        if (
            type(expected_steps) is not int
            or not isinstance(summary, Mapping)
            or type(summary.get("steps")) is not int
            or summary.get("steps") != expected_steps
        ):
            raise V12R9AdjudicationError("expected prefix step count drifted")
        value = _r8_plus_adjudicator.normalize_v26_prefix_summary(
            summary,
            frozen_target_contract_id=contract.TARGET_CONTRACT_ID,
            attested_contract_source=r8_contract.R7_CONTRACT_SOURCE_RECORD,
        )
    except Exception as exc:
        raise V12R9AdjudicationError(str(exc)) from exc
    result = copy.deepcopy(value)
    result["status"] = (
        "PASS_H0_V12R9_V26_PREFIX_SUMMARY_NORMALIZATION"
        if result.get("passed") is True
        else "FAIL_H0_V12R9_V26_PREFIX_SUMMARY_NORMALIZATION"
    )
    return result


def _attest_r8_terminal(root: Path) -> dict[str, Any]:
    ledger_path = _inside(
        contract.R8_TERMINAL_LEDGER_PATH.as_posix(), root, label="R8 ledger"
    )
    ledger = _strict_mapping(ledger_path)
    completed = ledger.get("completed_stages")
    activity = ledger.get("activity_totals")
    checks = {
        "terminal_fail_identity": ledger.get("schema_version")
        == r8_contract.SCHEMA_VERSION
        and ledger.get("protocol_id") == r8_contract.PROTOCOL_ID
        and ledger.get("pipeline_id") == r8_contract.PIPELINE_ID
        and ledger.get("status") == contract.R8_TERMINAL_FAIL_STATUS
        and ledger.get("passed") is False
        and ledger.get("terminal") is True,
        "proper_prefix": isinstance(completed, list)
        and [row.get("stage_id") for row in completed if isinstance(row, Mapping)]
        == list(r8_contract.STAGE_IDS[:2])
        and ledger.get("completed_stage_count") == 2
        and ledger.get("attempted_stage") == contract.R8_TERMINAL_STAGE,
        "error_exact": ledger.get("error")
        == {
            "type": contract.R8_TERMINAL_ERROR_TYPE,
            "message": contract.R8_TERMINAL_ERROR_MESSAGE,
        },
        "terminal_stop": ledger.get("next_stage") == "STOP_TERMINAL_NO_RETRY"
        and ledger.get("retry_authorized") is False
        and ledger.get("resume_authorized") is False
        and ledger.get("alpha_sweep_authorized") is False,
        "no_candidate_or_training": ledger.get("candidate_module") is None
        and ledger.get("candidate_id") is None
        and ledger.get("candidate_freeze") is None
        and ledger.get("actor_fit_count") == 0
        and all(
            _zero_int(ledger.get(name))
            for name in ("actor_updates", "critic_updates", "ppo_updates")
        ),
        "activity_exact": isinstance(activity, Mapping)
        and activity.get("collection_rounds_completed") == 1
        and activity.get("new_collection_environment_reset_calls") == 1
        and activity.get("new_collection_environment_step_calls") == 252
        and activity.get("raw_sensor_sample_count") == 2_520
        and activity.get("offline_teacher_label_calls") == 179
        and activity.get("actor_fit_stage_calls") == 0
        and activity.get("development_environment_reset_calls") == 0
        and activity.get("development_environment_step_calls") == 0,
        "claim_binding": ledger.get("pipeline_claim")
        == contract.LOCKED_R8_EVIDENCE["pipeline_claim"]
        and ledger.get("protocol_freeze")
        == contract.LOCKED_R8_EVIDENCE["protocol_freeze"]
        and ledger.get("execution_lock")
        == contract.LOCKED_R8_EVIDENCE["execution_lock"],
    }
    if not all(checks.values()):
        failed = sorted(name for name, value in checks.items() if value is not True)
        raise V12R9AdjudicationError(f"terminal R8 closure failed: {failed}")
    return {"checks": checks, "ledger": ledger}


def _summary_identity_map(
    *,
    summary: Mapping[str, Any],
    trace: list[Any],
    replay: observer.LoadedReplay,
    candidate_tree: Mapping[str, Any],
) -> dict[str, Any]:
    return {
        "schema_version": r8_contract.SCHEMA_VERSION,
        "status": r8_probe.PROBE_COMPLETE_STATUS,
        "protocol_id": r8_contract.PROTOCOL_ID,
        "stage_id": contract.R8_TERMINAL_STAGE,
        "case_id": contract.IMPORTED_MINUS_CASE_ID,
        "behavior": r8_probe.PROBE_BEHAVIOR,
        "candidate_id": r8_probe.EXPECTED_CANDIDATE_ID,
        "candidate_module": copy.deepcopy(dict(candidate_tree)),
        "candidate_tree_sha256": contract.FULL_R6_CANDIDATE_TREE["tree_sha256"],
        "trace_step_count": len(trace),
        "replay_step_count": replay.n_steps,
        "replay_boundary_count": replay.boundary_count,
        "replay_event_count": replay.event_count,
        "historical_plus_rerun": False,
        "teacher_enabled": False,
        "teacher_loaded_during_rollout": False,
        "blending_enabled": False,
        "safety_latch_enabled": False,
    }


def _attest_r8_minus_prefix(root: Path) -> dict[str, Any]:
    prefix_root = _inside(
        contract.R8_MINUS_ROOT.as_posix(), root, label="R8 minus root"
    )
    summary = _strict_mapping(prefix_root / "summary.json")
    trace = _strict_sequence(prefix_root / "trace.json")
    gate = _strict_mapping(prefix_root / "gate.json")
    receipt = _strict_mapping(prefix_root / "receipt.json")
    replay = observer.load_probe_replay_strict(
        prefix_root / "replay_boundaries.npz", contract_module=replay_contract
    )
    config = r8_probe.RecoveryProbeConfig(
        case_id=contract.IMPORTED_MINUS_CASE_ID, artifact_root=root
    )
    recomputed_gate = r8_probe.recovery_prefix_gate(
        summary, trace=trace, replay=replay, config=config
    )
    if gate != recomputed_gate or recomputed_gate.get("passed") is not True:
        raise V12R9AdjudicationError("R8 minus gate does not recompute exact PASS")

    expected_records = {
        "run_start": _record(prefix_root / "run_start.json", root),
        "trace": _record(prefix_root / "trace.json", root),
        "partial_summary": _record(prefix_root / "partial_summary.json", root),
        "summary": _record(prefix_root / "summary.json", root),
        "gate": _record(prefix_root / "gate.json", root),
        "replay_payload": _record(prefix_root / "replay_boundaries.npz", root),
    }
    if any(receipt.get(name) != value for name, value in expected_records.items()):
        raise V12R9AdjudicationError("R8 minus receipt artifact binding drifted")

    candidate_path = _inside(
        contract.R6_CANDIDATE_MODULE_PATH.as_posix(), root, label="R6 candidate"
    )
    actual_tree = _tree_record(candidate_path, root)
    if actual_tree != contract.FULL_R6_CANDIDATE_TREE:
        raise V12R9AdjudicationError("full R6 candidate tree drifted")
    if summary.get("candidate_module") != actual_tree:
        raise V12R9AdjudicationError(
            "R8 summary does not bind the actual candidate tree"
        )

    legacy_expected = _summary_identity_map(
        summary=summary,
        trace=trace,
        replay=replay,
        candidate_tree=r8_contract.LOCKED_INPUTS["r6_candidate"],
    )
    corrected_expected = _summary_identity_map(
        summary=summary,
        trace=trace,
        replay=replay,
        candidate_tree=actual_tree,
    )
    legacy_mismatches = sorted(
        name
        for name, expected in legacy_expected.items()
        if summary.get(name) != expected
    )
    corrected_mismatches = sorted(
        name
        for name, expected in corrected_expected.items()
        if summary.get(name) != expected
    )
    projection_fields = ("path", "tree_sha256", "file_count")
    projected_actual = {
        name: summary["candidate_module"].get(name) for name in projection_fields
    }
    actual_extra_fields = sorted(
        set(summary["candidate_module"])
        - set(r8_contract.LOCKED_INPUTS["r6_candidate"])
    )
    identity = {
        "mismatch_fields": legacy_mismatches,
        "actual_extra_fields": actual_extra_fields,
        "locked_projection_exact": projected_actual
        == r8_contract.LOCKED_INPUTS["r6_candidate"],
        "full_tree_files_exact": summary["candidate_module"].get("files")
        == contract.FULL_R6_CANDIDATE_TREE["files"],
        "tree_sha256_exact": summary.get("candidate_tree_sha256")
        == contract.FULL_R6_CANDIDATE_TREE["tree_sha256"],
        "file_count_exact": summary["candidate_module"].get("file_count") == 5,
        "semantic_identity_equivalent": corrected_mismatches == [],
        "root_cause": ROOT_CAUSE,
    }
    if identity != {
        "mismatch_fields": ["candidate_module"],
        "actual_extra_fields": ["files"],
        "locked_projection_exact": True,
        "full_tree_files_exact": True,
        "tree_sha256_exact": True,
        "file_count_exact": True,
        "semantic_identity_equivalent": True,
        "root_cause": ROOT_CAUSE,
    }:
        raise V12R9AdjudicationError("R8 root cause is not uniquely established")

    scalar_checks = {
        "schema_version": r8_contract.SCHEMA_VERSION,
        "status": r8_probe.RECEIPT_STATUS,
        "passed": True,
        "integrity_passed": True,
        "recoverable_for_observer_label": True,
        "autonomy_passed": False,
        "protocol_id": r8_contract.PROTOCOL_ID,
        "stage_id": contract.R8_TERMINAL_STAGE,
        "case_id": contract.IMPORTED_MINUS_CASE_ID,
        "probe_step_count": replay.n_steps,
        "candidate_id": r8_probe.EXPECTED_CANDIDATE_ID,
        "candidate_tree_sha256": contract.FULL_R6_CANDIDATE_TREE["tree_sha256"],
        "historical_plus_rerun": False,
    }
    if (
        set(receipt) != r8_probe.PROBE_RECEIPT_FIELDS
        or any(receipt.get(name) != value for name, value in scalar_checks.items())
        or receipt.get("v26_summary_normalization")
        != summary.get("v26_summary_normalization")
        or any(
            not _zero_int(receipt.get(name))
            for name in (
                "teacher_query_count",
                "actor_updates",
                "critic_updates",
                "ppo_updates",
            )
        )
    ):
        raise V12R9AdjudicationError("R8 minus receipt semantic binding drifted")

    writer = forensic.ForensicRolloutWriter(prefix_root, artifact_root=root)
    try:
        journal_records = writer.artifact_records()
        final_records = writer.finalized_artifact_records()
    except Exception as exc:
        raise V12R9AdjudicationError("R8 per-step forensic journal drifted") from exc
    journal_ok = (
        len(journal_records.get("steps", [])) == replay.n_steps == len(trace) == 252
        and all(
            final_records.get(name) == expected_records[name] for name in final_records
        )
        and journal_records.get("gate") == expected_records["gate"]
    )
    if not journal_ok:
        raise V12R9AdjudicationError("R8 journal aggregate binding drifted")

    v26 = summary.get("v26_summary_normalization")
    v26_audit = {
        "passed": isinstance(v26, Mapping) and v26.get("passed") is True,
        "raw_sensor_sample_count": summary.get("raw_sensor_sample_count"),
        "replay_event_count": replay.event_count,
        "event_contract_id": replay.event_contract_id,
        "detector_anomaly_counters_zero": all(
            _zero_int(summary.get(name)) for name in DETECTOR_ANOMALY_FIELDS
        ),
        "runtime_anomaly_counters_zero": all(
            _zero_int(summary.get(name)) for name in RUNTIME_ANOMALY_FIELDS
        ),
    }
    if v26_audit != {
        "passed": True,
        "raw_sensor_sample_count": 2_520,
        "replay_event_count": 3,
        "event_contract_id": r8_contract.EVENT_CONTRACT_ID,
        "detector_anomaly_counters_zero": True,
        "runtime_anomaly_counters_zero": True,
    }:
        raise V12R9AdjudicationError("R8 V26 prefix audit failed")

    try:
        r8_probe.verify_probe_closure(prefix_root, config=config)
    except r8_probe.V12R8RecoveryProbeError as exc:
        legacy_failure = {"type": type(exc).__name__, "message": str(exc)}
    else:
        raise V12R9AdjudicationError("legacy R8 verifier unexpectedly passed")
    if legacy_failure != {
        "type": contract.R8_TERMINAL_ERROR_TYPE,
        "message": contract.R8_TERMINAL_ERROR_MESSAGE,
    }:
        raise V12R9AdjudicationError("legacy R8 verifier failure changed")

    return {
        "summary": summary,
        "trace": trace,
        "gate": gate,
        "receipt": receipt,
        "replay": replay,
        "artifact_records": expected_records,
        "candidate_identity_adjudication": identity,
        "journal_attestation": {
            "passed": True,
            "step_count": len(journal_records["steps"]),
            "aggregate_records": final_records,
        },
        "trace_audit": copy.deepcopy(gate["trace_audit"]),
        "replay_audit": {
            "passed": True,
            "step_count": replay.n_steps,
            "boundary_count": replay.boundary_count,
            "event_count": replay.event_count,
            "actor_observations_trace_byte_exact": gate["checks"][
                "replay_trace_actor_byte_exact"
            ],
            "previous_penetration_trace_byte_exact": gate["checks"][
                "replay_trace_penetration_byte_exact"
            ],
        },
        "v26_audit": v26_audit,
        "legacy_verifier_failure": legacy_failure,
        "corrected_probe_closure": {
            "passed": True,
            "identity_mismatch_fields": corrected_mismatches,
            "gate_exact": True,
            "receipt_exact": True,
        },
    }


def load_and_adjudicate_r8_terminal(
    *, artifact_root: str | Path = REPO_ROOT
) -> dict[str, Any]:
    """Recompute the complete R8 terminal/prefix admission without writes."""

    root = Path(artifact_root).expanduser().resolve()
    locked_records = {
        name: _assert_locked_record(name, expected, root)
        for name, expected in contract.LOCKED_R8_EVIDENCE.items()
    }
    source_records = {
        "contract": _assert_locked_record(
            "R8 contract source", contract.R8_CONTRACT_SOURCE_RECORD, root
        ),
        "probe": _assert_locked_record(
            "R8 probe source", contract.R8_PROBE_SOURCE_RECORD, root
        ),
        "runner": _assert_locked_record(
            "R8 runner source", contract.R8_RUNNER_SOURCE_RECORD, root
        ),
    }
    terminal = _attest_r8_terminal(root)
    minus = _attest_r8_minus_prefix(root)
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": ADJUDICATION_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": "adjudicate_r8_terminal_and_minus_prefix",
        "case_id": contract.IMPORTED_MINUS_CASE_ID,
        "source_protocol_id": r8_contract.PROTOCOL_ID,
        "source_pipeline_id": r8_contract.PIPELINE_ID,
        "source_terminal_status": contract.R8_TERMINAL_FAIL_STATUS,
        "source_attempted_stage": contract.R8_TERMINAL_STAGE,
        "artifact_records": locked_records,
        "source_code_records": source_records,
        "terminal_checks": terminal["checks"],
        "candidate_identity_adjudication": minus["candidate_identity_adjudication"],
        "journal_attestation": minus["journal_attestation"],
        "trace_audit": minus["trace_audit"],
        "replay_audit": minus["replay_audit"],
        "v26_audit": minus["v26_audit"],
        "legacy_verifier_failure": minus["legacy_verifier_failure"],
        "counterfactual_probe_closure": minus["corrected_probe_closure"],
        "offline_label_authorized": True,
        "autonomy_passed": False,
        "r8_artifacts_modified": False,
        "r8_probe_rerun": False,
        "r8_retry_authorized": False,
        "r8_resume_authorized": False,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "teacher_query_count": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "next_stage": "import_r8_plus_labels",
    }


def publish_r8_minus_adjudication(
    *, destination: str | Path, artifact_root: str | Path = REPO_ROOT
) -> dict[str, Any]:
    """Publish the canonical R9 adjudication receipt exactly once."""

    root = Path(artifact_root).expanduser().resolve()
    target = _inside(destination, root, label="R9 adjudication destination")
    expected = _inside(
        contract.R8_MINUS_ADJUDICATION_PATH.as_posix(),
        root,
        label="canonical adjudication",
    )
    if target != expected:
        raise V12R9AdjudicationError("adjudication destination is not canonical R9")
    payload = load_and_adjudicate_r8_terminal(artifact_root=root)
    forensic.write_json_exclusive(target, payload)
    return verify_r8_minus_adjudication(destination=target, artifact_root=root)


def verify_r8_minus_adjudication(
    *, destination: str | Path, artifact_root: str | Path = REPO_ROOT
) -> dict[str, Any]:
    root = Path(artifact_root).expanduser().resolve()
    target = _inside(destination, root, label="R9 adjudication destination")
    observed = _strict_mapping(target)
    expected = load_and_adjudicate_r8_terminal(artifact_root=root)
    if observed != expected or observed.get("passed") is not True:
        raise V12R9AdjudicationError("R9 adjudication receipt drifted")
    return observed


def verify_r8_plus_label_import(
    *, artifact_root: str | Path = REPO_ROOT, semantic_verify: bool = True
) -> dict[str, Any]:
    """Close the R8 plus labels in place; no copy and no R9 teacher query."""

    if type(semantic_verify) is not bool:
        raise V12R9AdjudicationError("semantic_verify must be strict bool")
    root = Path(artifact_root).expanduser().resolve()
    records = {
        name: _assert_locked_record(name, contract.LOCKED_R8_EVIDENCE[name], root)
        for name in (
            "plus_labels",
            "plus_label_summary",
            "plus_label_gate",
            "plus_label_receipt",
            "plus_label_stage_receipt",
        )
    }
    semantic = None
    if semantic_verify:
        try:
            semantic = r8_probe.verify_observer_label_closure(
                r8_contract.HISTORICAL_CASE_ID,
                artifact_root=root,
                require_stage_receipt=True,
            )
        except Exception as exc:
            raise V12R9AdjudicationError(
                "R8 plus label semantic/byte-exact closure failed"
            ) from exc
        if semantic.get("passed") is not True:
            raise V12R9AdjudicationError("R8 plus label closure is not PASS")
    summary = _strict_mapping(
        _inside(
            contract.R8_PLUS_LABEL_SUMMARY_PATH.as_posix(),
            root,
            label="R8 plus label summary",
        )
    )
    rows = summary.get("labelled_row_count")
    if type(rows) is not int or rows != 179:
        raise V12R9AdjudicationError("R8 plus imported row count drifted")
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": PLUS_IMPORT_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": "import_r8_plus_labels",
        "case_id": contract.HISTORICAL_CASE_ID,
        "source_protocol_id": r8_contract.PROTOCOL_ID,
        "source_label_stage": "label_r7_plus_prefix",
        "source_artifacts": records,
        "semantic_and_byte_exact_closed": semantic_verify,
        "semantic_closure_status": None if semantic is None else semantic.get("status"),
        "direct_immutable_reference": True,
        "labels_copied": False,
        "r8_artifacts_modified": False,
        "labelled_row_count": rows,
        "same_state_teacher_label_count": rows,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "teacher_query_count": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }


__all__ = [
    "ADJUDICATION_STATUS",
    "DETECTOR_ANOMALY_FIELDS",
    "PLUS_IMPORT_STATUS",
    "PROJECTED_ANOMALY_FIELDS",
    "ROOT_CAUSE",
    "RUNTIME_ANOMALY_FIELDS",
    "V12R9AdjudicationError",
    "load_and_adjudicate_r8_terminal",
    "normalize_v26_prefix_summary",
    "publish_r8_minus_adjudication",
    "verify_r8_minus_adjudication",
    "verify_r8_plus_label_import",
]
