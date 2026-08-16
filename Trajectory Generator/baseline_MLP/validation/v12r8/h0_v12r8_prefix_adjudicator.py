"""Read-only adjudication of the immutable terminal V12R7 critical prefix.

The pure core consumes already-loaded evidence.  The I/O adapter only reads
and hash-attests the frozen R7 files; it never writes into, repairs, or reruns
R7.  In particular, ``target_contract_id`` is not inferred from the historical
summary (where it is absent).  It is projected only after the R7 protocol
freeze, execution lock, and their exact contract-source closure have passed.
"""

from __future__ import annotations

import copy
import math
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


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
REVISION_ROOT = Path(__file__).resolve().parent
for _root in (
    REPO_ROOT,
    REPO_ROOT / "validation",
    REPO_ROOT / "Trajectory Generator" / "baseline_MLP",
    LOCAL_VALIDATION_ROOT,
    LOCAL_VALIDATION_ROOT / "v12r3",
    LOCAL_VALIDATION_ROOT / "v12r6",
    LOCAL_VALIDATION_ROOT / "v12r7",
    REVISION_ROOT,
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import freeze_h0_v12r7_recovery as r7_freezer  # noqa: E402
import h0_forensic_rollout as forensic  # noqa: E402
import h0_primary_split_v12r3_autonomy_recovery_contract as replay_contract  # noqa: E402
import h0_primary_split_v12r3_pure_probe_observer_labeler as observer  # noqa: E402
import h0_v12r7_recovery_contract as r7_contract  # noqa: E402
import h0_v12r7_recovery_probe as r7_probe  # noqa: E402
import h0_v12r8_recovery_contract as contract  # noqa: E402


ADJUDICATION_PASS_STATUS = "PASS_H0_V12R8_R7_PLUS_PREFIX_ADJUDICATION"
ADJUDICATION_FAIL_STATUS = "FAIL_H0_V12R8_R7_PLUS_PREFIX_ADJUDICATION"
NORMALIZATION_STATUS = "PASS_H0_V12R8_V26_PREFIX_SUMMARY_NORMALIZATION"
PROJECTED_ANOMALY_FIELDS = (
    "duplicate_event_count",
    "out_of_order_event_count",
    "left_non_v26_source_count",
)
NESTED_V26_COUNTER_FIELDS = (
    "fallback_count",
    "hard_invalid_count",
    *PROJECTED_ANOMALY_FIELDS,
)
DETECTOR_ANOMALY_FIELDS = (
    *NESTED_V26_COUNTER_FIELDS,
    "invalid_event_count",
    "routing_failure_count",
    "step_contract_failure_count",
    "binary_event_failure_count",
)
RUNTIME_ANOMALY_FIELDS = (
    "action_clipped_values",
    "nonfinite_count",
    "sea_plugin_fallback_count",
    "so_solver_unaccepted_count",
)


class V12R8AdjudicationError(RuntimeError):
    """Raised when immutable evidence cannot be read without ambiguity."""


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


def _zero_int(value: Any) -> bool:
    return type(value) is int and value == 0


def _finite(value: Any) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def _all_true_mapping(value: Any) -> bool:
    return (
        isinstance(value, Mapping)
        and bool(value)
        and all(item is True for item in value.values())
    )


def normalize_v26_prefix_summary(
    summary: Mapping[str, Any],
    *,
    frozen_target_contract_id: str,
    attested_contract_source: Mapping[str, Any],
) -> dict[str, Any]:
    """Purely project nested V26 evidence without changing thresholds.

    The caller must supply the exact R7 contract-source record attested by both
    the immutable protocol freeze and execution lock.  The historical prefix
    is allowed to retain ``binary_phase_event_gate.passed == false`` because
    that full-horizon gate expects 5000 samples; its 1790-sample prefix is
    instead required to pass the dedicated nested prefix-integrity object.
    """

    if not isinstance(summary, Mapping):
        raise V12R8AdjudicationError("summary must be a mapping")
    if not _strict_equal(attested_contract_source, contract.R7_CONTRACT_SOURCE_RECORD):
        raise V12R8AdjudicationError("R7 contract source is not exactly attested")
    if frozen_target_contract_id != contract.TARGET_CONTRACT_ID:
        raise V12R8AdjudicationError("frozen target contract identity drifted")
    source = copy.deepcopy(dict(summary))
    steps = source.get("steps")
    if type(steps) is not int or not (
        contract.MINIMUM_RECOVERABLE_PREFIX_STEPS <= steps <= contract.EXPECTED_STEPS
    ):
        raise V12R8AdjudicationError("prefix step count is outside the frozen range")
    nested = source.get("binary_phase_event_gate")
    prefix = source.get("binary_event_prefix_integrity")
    if not isinstance(nested, Mapping) or not isinstance(prefix, Mapping):
        raise V12R8AdjudicationError("nested V26 evidence is absent")
    expected_samples = steps * contract.RAW_SAMPLES_PER_STEP
    if (
        nested.get("sample_count") != expected_samples
        or prefix.get("sample_count") != expected_samples
        or prefix.get("expected_sample_count") != expected_samples
        or prefix.get("raw_sensor_sample_count") != expected_samples
        or prefix.get("passed") is not True
        or not _all_true_mapping(prefix.get("checks"))
    ):
        raise V12R8AdjudicationError("nested V26 prefix sample evidence drifted")
    full_horizon = steps == contract.EXPECTED_STEPS
    if (full_horizon and nested.get("passed") is not True) or (
        not full_horizon and nested.get("passed") is not False
    ):
        raise V12R8AdjudicationError("nested V26 gate horizon semantics drifted")
    if any(not _zero_int(nested.get(name)) for name in NESTED_V26_COUNTER_FIELDS):
        raise V12R8AdjudicationError("nested V26 anomaly counter is nonzero or invalid")
    for name in ("fallback_count", "hard_invalid_count"):
        if source.get(name) != nested.get(name) or type(source.get(name)) is not int:
            raise V12R8AdjudicationError(f"top-level {name} disagrees with nested V26")
    events = nested.get("events")
    if (
        not isinstance(events, list)
        or nested.get("event_count") != len(events)
        or any(
            not isinstance(event, Mapping)
            or event.get("event_contract_id") != contract.EVENT_CONTRACT_ID
            or event.get("source") != "binary_phase_fsm_v26"
            for event in events
        )
    ):
        raise V12R8AdjudicationError("nested V26 event identity drifted")
    if (
        source.get("binary_phase_fsm_mode") != "binary_active"
        or source.get("event_contract_id") != contract.EVENT_CONTRACT_ID
    ):
        raise V12R8AdjudicationError("top-level V26 activation evidence drifted")

    projected: list[str] = []
    for name in PROJECTED_ANOMALY_FIELDS:
        nested_value = nested[name]
        if name in source:
            if source[name] != nested_value or type(source[name]) is not int:
                raise V12R8AdjudicationError(
                    f"existing top-level {name} disagrees with nested V26"
                )
        else:
            source[name] = nested_value
            projected.append(name)
    if "target_contract_id" in source:
        if source["target_contract_id"] != frozen_target_contract_id:
            raise V12R8AdjudicationError("existing target contract identity drifted")
    else:
        source["target_contract_id"] = frozen_target_contract_id
        projected.append("target_contract_id")

    # Prove this transform is additive only: no persisted value or nested gate
    # is modified, and the derived values remain exact zero/V26 identities.
    for key, value in summary.items():
        if not _strict_equal(source.get(key), value):
            raise V12R8AdjudicationError(f"normalization changed persisted field {key}")
    if source["binary_phase_event_gate"].get("passed") is not nested.get("passed"):
        raise V12R8AdjudicationError("normalization changed nested gate result")
    return {
        "status": NORMALIZATION_STATUS,
        "passed": True,
        "summary": source,
        "projected_fields": projected,
        "projected_values": {name: source[name] for name in projected},
        "nested_gate_passed_preserved": nested.get("passed"),
        "nested_gate_full_horizon_applicable": full_horizon,
        "prefix_integrity_passed": True,
        "sample_count": expected_samples,
        "target_contract_derivation": "FROZEN_R7_CONTRACT_SOURCE_CLOSURE",
    }


def _r7_terminal_ledger_checks(ledger: Mapping[str, Any]) -> dict[str, bool]:
    activity = ledger.get("activity_totals")
    expected_activity = {
        "actor_fit_stage_calls": 0,
        "actor_updates": 0,
        "adamw_epochs_completed": 0,
        "collection_rounds_completed": 0,
        "critic_updates": 0,
        "development_rollouts_completed": 0,
        "environment_reset_calls": 1,
        "environment_step_calls": 179,
        "lbfgs_closure_calls": 0,
        "offline_teacher_label_calls": 0,
        "ppo_updates": 0,
        "raw_sensor_sample_count": 1790,
    }
    return {
        "mapping": isinstance(ledger, Mapping),
        "terminal_fail_status": ledger.get("status") == contract.R7_TERMINAL_FAIL_STATUS
        and ledger.get("passed") is False
        and ledger.get("terminal") is True,
        "r7_identity": ledger.get("schema_version") == r7_contract.SCHEMA_VERSION
        and ledger.get("protocol_id") == r7_contract.PROTOCOL_ID
        and ledger.get("pipeline_id") == r7_contract.PIPELINE_ID,
        "failed_first_stage": ledger.get("attempted_stage")
        == contract.R7_TERMINAL_STAGE
        and ledger.get("completed_stage_count") == 0
        and ledger.get("completed_stages") == [],
        "known_projection_failure": ledger.get("error")
        == {
            "message": "recovery prefix gate is not closed PASS",
            "type": "V12R7RecoveryProbeError",
        },
        "terminal_no_retry": ledger.get("next_stage") == "STOP_TERMINAL_NO_RETRY"
        and ledger.get("retry_authorized") is False
        and ledger.get("resume_authorized") is False
        and ledger.get("alpha_sweep_authorized") is False,
        "no_fit_or_development": ledger.get("actor_fit_count") == 0
        and ledger.get("development_count") == 0
        and ledger.get("candidate_id") is None
        and ledger.get("candidate_module") is None,
        "activity_exact": _strict_equal(activity, expected_activity),
        "zero_update_counters": all(
            _zero_int(ledger.get(name))
            for name in ("actor_updates", "critic_updates", "ppo_updates")
        ),
    }


def _freeze_lock_checks(
    protocol_freeze: Mapping[str, Any],
    execution_lock: Mapping[str, Any],
) -> dict[str, bool]:
    source_key = contract.R7_CONTRACT_SOURCE_RECORD["path"]
    freeze_source = (
        protocol_freeze.get("production_source_closure", {}).get(source_key)
        if isinstance(protocol_freeze.get("production_source_closure"), Mapping)
        else None
    )
    lock_source = (
        execution_lock.get("production_source_closure", {}).get(source_key)
        if isinstance(execution_lock.get("production_source_closure"), Mapping)
        else None
    )
    freeze_record = contract.LOCKED_R7_EVIDENCE["protocol_freeze"]
    return {
        "freeze_pass": protocol_freeze.get("passed") is True
        and protocol_freeze.get("status") == r7_contract.PROTOCOL_FREEZE_PASS_STATUS,
        "lock_pass": execution_lock.get("passed") is True
        and execution_lock.get("status") == r7_contract.EXECUTION_LOCK_PASS_STATUS,
        "identity_exact": all(
            value.get("schema_version") == r7_contract.SCHEMA_VERSION
            and value.get("protocol_id") == r7_contract.PROTOCOL_ID
            and value.get("pipeline_id") == r7_contract.PIPELINE_ID
            for value in (protocol_freeze, execution_lock)
        ),
        "contract_self_check": protocol_freeze.get("contract_self_check", {}).get(
            "passed"
        )
        is True,
        "source_closed_twice": _strict_equal(
            freeze_source, contract.R7_CONTRACT_SOURCE_RECORD
        )
        and _strict_equal(lock_source, contract.R7_CONTRACT_SOURCE_RECORD),
        "lock_binds_freeze": _strict_equal(
            execution_lock.get("protocol_freeze"), freeze_record
        ),
        "one_shot_no_retry": all(
            value.get("one_shot") is True
            and value.get("retry_authorized") is False
            and value.get("resume_authorized") is False
            and value.get("alpha_sweep_authorized") is False
            for value in (protocol_freeze, execution_lock)
        ),
    }


def _persisted_failure_checks(
    summary: Mapping[str, Any],
    persisted_gate: Mapping[str, Any],
    receipt: Mapping[str, Any],
) -> dict[str, bool]:
    checks = persisted_gate.get("checks")
    failed_checks = (
        {name for name, value in checks.items() if value is False}
        if isinstance(checks, Mapping)
        else set()
    )
    return {
        "summary_complete": summary.get("status") == r7_probe.PROBE_COMPLETE_STATUS
        and summary.get("steps") == 179
        and summary.get("trace_step_count") == 179
        and summary.get("replay_step_count") == 179
        and summary.get("replay_boundary_count") == 180,
        "summary_terminal_shape": summary.get("terminated") is True
        and summary.get("truncated") is False
        and summary.get("end_reason") == "grf_penetration",
        "summary_identity": summary.get("protocol_id") == r7_contract.PROTOCOL_ID
        and summary.get("case_id") == contract.HISTORICAL_CASE_ID
        and summary.get("candidate_tree_sha256")
        == r7_contract.LOCKED_INPUTS["r6_candidate"]["tree_sha256"],
        "target_absent_as_diagnosed": "target_contract_id" not in summary,
        "projection_fields_absent_as_diagnosed": all(
            name not in summary for name in PROJECTED_ANOMALY_FIELDS
        ),
        "persisted_gate_terminal_fail": persisted_gate.get("passed") is False
        and persisted_gate.get("integrity_passed") is False
        and persisted_gate.get("recoverable_for_observer_label") is False
        and persisted_gate.get("status") == "FAIL_H0_V12R7_R6_PREFIX_INTEGRITY",
        "only_projection_checks_failed": failed_checks
        == {"contract_integrity", "detector_active", "zero_detector_anomalies"},
        "receipt_failure_exact": receipt.get("passed") is False
        and receipt.get("integrity_passed") is False
        and receipt.get("recoverable_for_observer_label") is False
        and receipt.get("autonomy_passed") is False
        and receipt.get("gate") == contract.LOCKED_R7_EVIDENCE["plus_gate"]
        and receipt.get("summary") == contract.LOCKED_R7_EVIDENCE["plus_summary"]
        and receipt.get("trace") == contract.LOCKED_R7_EVIDENCE["plus_trace"]
        and receipt.get("replay_payload") == contract.LOCKED_R7_EVIDENCE["plus_replay"],
    }


def _counterfactual_r7_gate(
    normalized: Mapping[str, Any],
    *,
    trace: Sequence[Any],
    replay: observer.LoadedReplay,
    reproduction_audit: Mapping[str, Any],
) -> dict[str, Any]:
    """Pure equivalent of the R7 prefix gate with supplied reproduction proof."""

    config = r7_probe.RecoveryProbeConfig(
        case_id=contract.HISTORICAL_CASE_ID,
        artifact_root=REPO_ROOT,
        progress_every=0,
    )
    trace_audit = r7_probe.pure_prefix_trace_audit(trace, config=config)
    contract_gate = r7_contract.collection_integrity_gate(normalized)
    steps = normalized.get("steps")
    rows = list(trace)
    replay_trace_exact = False
    penetration_exact = False
    if type(steps) is int and steps == len(rows) == replay.n_steps:
        try:
            trace_actor = np.ascontiguousarray(
                [row["v26_observation"] for row in rows], dtype=np.float32
            )
            trace_penetration = np.asarray(
                [row["previous_penetration_m"] for row in rows], dtype=np.float64
            )
            replay_trace_exact = trace_actor.shape == (
                steps,
                contract.EXPECTED_ACTOR_FEATURES,
            ) and trace_actor.tobytes(order="C") == replay.arrays[
                "actor_observations"
            ].tobytes(order="C")
            penetration_exact = trace_penetration.shape == (
                steps,
            ) and trace_penetration.tobytes(order="C") == replay.arrays[
                "previous_penetration_m"
            ].tobytes(order="C")
        except (KeyError, TypeError, ValueError):
            pass
    binary_prefix = normalized.get("binary_event_prefix_integrity")
    physical_prefix = (
        type(steps) is int
        and contract.MINIMUM_RECOVERABLE_PREFIX_STEPS <= steps < contract.EXPECTED_STEPS
        and normalized.get("end_reason") == "grf_penetration"
        and normalized.get("terminated") is True
        and normalized.get("truncated") is False
        and normalized.get("safety_stop_count") == 1
        and _finite(normalized.get("grf_penetration_max_m"))
        and float(normalized["grf_penetration_max_m"]) >= contract.PENETRATION_LIMIT_M
    )
    checks = {
        "contract_integrity": contract_gate.get("passed") is True,
        "contract_closed_trajectory": contract_gate.get("recoverable_prefix") is True,
        "trace_pure_and_closed": trace_audit.get("passed") is True,
        "r6_plus_reproduced_when_applicable": reproduction_audit.get("passed") is True
        and normalized.get("r6_plus_reproduction_audit") == reproduction_audit,
        "physical_prefix_or_complete_episode": physical_prefix,
        "candidate_exact": normalized.get("candidate_tree_sha256")
        == config.expected_candidate_tree_sha256
        and normalized.get("candidate_id") == config.expected_candidate_id,
        "detector_active": normalized.get("binary_phase_fsm_mode") == "binary_active"
        and normalized.get("event_contract_id") == config.event_contract_id
        and normalized.get("target_contract_id") == config.target_contract_id,
        "detector_prefix_integrity": isinstance(binary_prefix, Mapping)
        and binary_prefix.get("passed") is True
        and binary_prefix.get("sample_count") == steps * contract.RAW_SAMPLES_PER_STEP,
        "zero_detector_anomalies": all(
            _zero_int(normalized.get(name)) for name in DETECTOR_ANOMALY_FIELDS
        ),
        "zero_runtime_anomalies": all(
            _zero_int(normalized.get(name)) for name in RUNTIME_ANOMALY_FIELDS
        ),
        "teacher_free_summary": normalized.get("teacher_enabled") is False
        and normalized.get("teacher_loaded_during_rollout") is False
        and normalized.get("blending_enabled") is False
        and normalized.get("safety_latch_enabled") is False
        and all(
            _zero_int(normalized.get(name)) for name in r7_probe.ZERO_ONLINE_COUNTERS
        ),
        "replay_shape": replay.n_steps == steps and replay.boundary_count == steps + 1,
        "replay_event_contract": replay.event_contract_id == config.event_contract_id,
        "replay_trace_actor_byte_exact": replay_trace_exact,
        "replay_trace_penetration_byte_exact": penetration_exact,
        "zero_updates": all(
            _zero_int(normalized.get(name))
            for name in ("actor_updates", "critic_updates", "ppo_updates")
        ),
        "morphology_disabled": _finite(normalized.get("morphology_weight"))
        and float(normalized["morphology_weight"]) == 0.0,
    }
    passed = all(value is True for value in checks.values())
    return {
        "schema_version": r7_contract.SCHEMA_VERSION,
        "status": (
            "PASS_H0_V12R7_RECOVERABLE_R6_PREFIX"
            if passed
            else "FAIL_H0_V12R7_R6_PREFIX_INTEGRITY"
        ),
        "passed": passed,
        "integrity_passed": passed,
        "recoverable_for_observer_label": passed,
        "recoverable_prefix": bool(passed and physical_prefix),
        "complete_episode": False,
        "autonomy_passed": False,
        "protocol_id": r7_contract.PROTOCOL_ID,
        "case_id": contract.HISTORICAL_CASE_ID,
        "probe_step_count": steps if type(steps) is int else None,
        "candidate_id": config.expected_candidate_id,
        "candidate_tree_sha256": config.expected_candidate_tree_sha256,
        "checks": checks,
        "trace_audit": trace_audit,
        "r6_plus_reproduction_audit": copy.deepcopy(dict(reproduction_audit)),
        "contract_gate": contract_gate,
        "next_stage": (
            "OFFLINE_COHERENT_H0_LABEL_REQUIRED" if passed else "STOP_INTEGRITY_FAILURE"
        ),
    }


def adjudicate_r7_plus_prefix(
    *,
    artifact_records: Mapping[str, Any],
    protocol_freeze: Mapping[str, Any],
    execution_lock: Mapping[str, Any],
    terminal_ledger: Mapping[str, Any],
    summary: Mapping[str, Any],
    persisted_gate: Mapping[str, Any],
    receipt: Mapping[str, Any],
    trace: Sequence[Any],
    replay: observer.LoadedReplay,
    journal_attestation: Mapping[str, Any],
    reproduction_audit: Mapping[str, Any],
) -> dict[str, Any]:
    """Pure fail-closed adjudicator over already-loaded immutable evidence."""

    if (
        not all(
            isinstance(value, Mapping)
            for value in (
                artifact_records,
                protocol_freeze,
                execution_lock,
                terminal_ledger,
                summary,
                persisted_gate,
                receipt,
                journal_attestation,
                reproduction_audit,
            )
        )
        or not isinstance(trace, Sequence)
        or isinstance(trace, (str, bytes))
    ):
        raise V12R8AdjudicationError("adjudicator evidence has invalid types")
    expected_record_names = set(contract.LOCKED_R7_EVIDENCE)
    record_checks = {
        "record_names_exact": set(artifact_records) == expected_record_names,
        "records_exact": set(artifact_records) == expected_record_names
        and all(
            _strict_equal(artifact_records[name], expected)
            for name, expected in contract.LOCKED_R7_EVIDENCE.items()
        ),
    }
    freeze_checks = _freeze_lock_checks(protocol_freeze, execution_lock)
    ledger_checks = _r7_terminal_ledger_checks(terminal_ledger)
    failure_checks = _persisted_failure_checks(summary, persisted_gate, receipt)
    journal_checks = {
        "trace_matches_step_journal": journal_attestation.get(
            "trace_matches_step_journal"
        )
        is True,
        "trace_record_exact": _strict_equal(
            journal_attestation.get("trace"),
            contract.LOCKED_R7_EVIDENCE["plus_trace"],
        ),
        "summary_record_exact": _strict_equal(
            journal_attestation.get("summary"),
            contract.LOCKED_R7_EVIDENCE["plus_summary"],
        ),
    }
    reproduction_checks = {
        "passed": reproduction_audit.get("passed") is True,
        "179_of_179": reproduction_audit.get("historical_step_count") == 179
        and reproduction_audit.get("current_step_count") == 179
        and reproduction_audit.get("row_count_exact") is True,
        "source_and_fields_exact": reproduction_audit.get("historical_source_valid")
        is True
        and reproduction_audit.get("shared_runtime_fields_exact") is True
        and reproduction_audit.get("first_mismatch_step") is None
        and reproduction_audit.get("first_mismatch_field") is None,
        "summary_bound": _strict_equal(
            summary.get("r6_plus_reproduction_audit"), reproduction_audit
        ),
    }
    replay_checks = {
        "strict_shape": replay.n_steps == 179 and replay.boundary_count == 180,
        "event_contract": replay.event_contract_id == contract.EVENT_CONTRACT_ID,
        "summary_count_binding": summary.get("replay_step_count") == replay.n_steps
        and summary.get("replay_boundary_count") == replay.boundary_count,
    }
    prechecks = {
        **{f"artifact__{key}": value for key, value in record_checks.items()},
        **{f"freeze_lock__{key}": value for key, value in freeze_checks.items()},
        **{f"ledger__{key}": value for key, value in ledger_checks.items()},
        **{f"r7_failure__{key}": value for key, value in failure_checks.items()},
        **{f"journal__{key}": value for key, value in journal_checks.items()},
        **{f"reproduction__{key}": value for key, value in reproduction_checks.items()},
        **{f"replay__{key}": value for key, value in replay_checks.items()},
    }
    if not all(value is True for value in prechecks.values()):
        return {
            "schema_version": contract.SCHEMA_VERSION,
            "status": ADJUDICATION_FAIL_STATUS,
            "passed": False,
            "protocol_id": contract.PROTOCOL_ID,
            "case_id": contract.HISTORICAL_CASE_ID,
            "checks": prechecks,
            "target_contract_derivation": None,
            "normalization": None,
            "counterfactual_r7_gate": None,
            "offline_label_authorized": False,
            "r7_artifacts_modified": False,
            "r7_probe_rerun": False,
            "next_stage": "STOP_ADJUDICATION_FAILURE",
        }

    target_derivation = {
        "method": "FROZEN_R7_CONTRACT_SOURCE_CLOSURE",
        "target_contract_id": r7_contract.TARGET_CONTRACT_ID,
        "event_contract_id": r7_contract.EVENT_CONTRACT_ID,
        "protocol_freeze": copy.deepcopy(
            dict(contract.LOCKED_R7_EVIDENCE["protocol_freeze"])
        ),
        "execution_lock": copy.deepcopy(
            dict(contract.LOCKED_R7_EVIDENCE["execution_lock"])
        ),
        "contract_source": copy.deepcopy(dict(contract.R7_CONTRACT_SOURCE_RECORD)),
        "attested_after_all_prechecks": True,
    }
    try:
        normalization = normalize_v26_prefix_summary(
            summary,
            frozen_target_contract_id=target_derivation["target_contract_id"],
            attested_contract_source=target_derivation["contract_source"],
        )
        counterfactual_gate = _counterfactual_r7_gate(
            normalization["summary"],
            trace=trace,
            replay=replay,
            reproduction_audit=reproduction_audit,
        )
    except (V12R8AdjudicationError, r7_probe.V12R7RecoveryProbeError):
        return {
            "schema_version": contract.SCHEMA_VERSION,
            "status": ADJUDICATION_FAIL_STATUS,
            "passed": False,
            "protocol_id": contract.PROTOCOL_ID,
            "case_id": contract.HISTORICAL_CASE_ID,
            "checks": {**prechecks, "normalization_and_gate": False},
            "target_contract_derivation": target_derivation,
            "normalization": None,
            "counterfactual_r7_gate": None,
            "offline_label_authorized": False,
            "r7_artifacts_modified": False,
            "r7_probe_rerun": False,
            "next_stage": "STOP_ADJUDICATION_FAILURE",
        }
    final_checks = {
        **prechecks,
        "normalization_passed": normalization.get("passed") is True,
        "historical_projection_exact": normalization.get("projected_fields")
        == [*PROJECTED_ANOMALY_FIELDS, "target_contract_id"],
        "nested_full_horizon_gate_preserved_false": normalization.get(
            "nested_gate_passed_preserved"
        )
        is False
        and normalization.get("nested_gate_full_horizon_applicable") is False,
        "counterfactual_r7_gate_passed": counterfactual_gate.get("passed") is True
        and counterfactual_gate.get("recoverable_for_observer_label") is True
        and counterfactual_gate.get("autonomy_passed") is False,
    }
    passed = all(value is True for value in final_checks.values())
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": ADJUDICATION_PASS_STATUS if passed else ADJUDICATION_FAIL_STATUS,
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "case_id": contract.HISTORICAL_CASE_ID,
        "checks": final_checks,
        "artifact_records": copy.deepcopy(dict(artifact_records)),
        "target_contract_derivation": target_derivation,
        "normalization": normalization,
        "counterfactual_r7_gate": counterfactual_gate,
        "offline_label_authorized": passed,
        "autonomy_passed": False,
        "r7_artifacts_modified": False,
        "r7_probe_rerun": False,
        "next_stage": "OFFLINE_LABEL_IN_R8_NAMESPACE"
        if passed
        else "STOP_ADJUDICATION_FAILURE",
    }


def _strict_mapping(path: Path) -> dict[str, Any]:
    try:
        value = forensic.strict_json_load(path)
    except Exception as exc:
        raise V12R8AdjudicationError(f"invalid strict JSON: {path}") from exc
    if not isinstance(value, Mapping):
        raise V12R8AdjudicationError(f"expected JSON mapping: {path}")
    return dict(value)


def _strict_trace(path: Path) -> list[Any]:
    try:
        value = forensic.strict_json_load(path)
    except Exception as exc:
        raise V12R8AdjudicationError(f"invalid strict JSON trace: {path}") from exc
    if not isinstance(value, list):
        raise V12R8AdjudicationError("R7 plus trace is not a JSON array")
    return value


def load_r7_plus_evidence(*, artifact_root: str | Path = REPO_ROOT) -> dict[str, Any]:
    """Load and hash-attest the canonical R7 evidence without adjudicating it."""

    root = Path(artifact_root).expanduser().resolve()
    if root != REPO_ROOT:
        raise V12R8AdjudicationError("canonical R7 evidence requires repository root")
    records: dict[str, Any] = {}
    for name, expected in contract.LOCKED_R7_EVIDENCE.items():
        try:
            records[name] = r7_freezer.artifact_record(expected["path"])
        except Exception as exc:
            raise V12R8AdjudicationError(f"cannot attest R7 artifact {name}") from exc
    protocol_freeze = _strict_mapping(
        REPO_ROOT.joinpath(*contract.R7_PROTOCOL_FREEZE_PATH.parts)
    )
    execution_lock = _strict_mapping(
        REPO_ROOT.joinpath(*contract.R7_EXECUTION_LOCK_PATH.parts)
    )
    terminal_ledger = _strict_mapping(
        REPO_ROOT.joinpath(*contract.R7_TERMINAL_LEDGER_PATH.parts)
    )
    plus_root = REPO_ROOT.joinpath(*contract.R7_PLUS_ROOT.parts)
    summary = _strict_mapping(plus_root / "summary.json")
    persisted_gate = _strict_mapping(plus_root / "gate.json")
    receipt = _strict_mapping(plus_root / "receipt.json")
    trace = _strict_trace(plus_root / "trace.json")
    try:
        replay = observer.load_probe_replay_strict(
            plus_root / "replay_boundaries.npz", contract_module=replay_contract
        )
        finalized = forensic.ForensicRolloutWriter(
            plus_root, artifact_root=REPO_ROOT
        ).finalized_artifact_records()
    except Exception as exc:
        raise V12R8AdjudicationError(
            "R7 plus replay or per-step forensic journal is invalid"
        ) from exc
    reproduction = r7_probe.r6_plus_reproduction_audit(trace)
    journal_attestation = {
        "trace_matches_step_journal": True,
        "trace": finalized["trace"],
        "summary": finalized["summary"],
        "partial_summary": finalized["partial_summary"],
    }
    return {
        "artifact_records": records,
        "protocol_freeze": protocol_freeze,
        "execution_lock": execution_lock,
        "terminal_ledger": terminal_ledger,
        "summary": summary,
        "persisted_gate": persisted_gate,
        "receipt": receipt,
        "trace": trace,
        "replay": replay,
        "journal_attestation": journal_attestation,
        "reproduction_audit": reproduction,
    }


def load_and_adjudicate_r7_plus(
    *, artifact_root: str | Path = REPO_ROOT
) -> dict[str, Any]:
    """Read, hash-attest, and adjudicate the canonical R7 plus prefix."""

    return adjudicate_r7_plus_prefix(
        **load_r7_plus_evidence(artifact_root=artifact_root)
    )


__all__ = [
    "ADJUDICATION_FAIL_STATUS",
    "ADJUDICATION_PASS_STATUS",
    "NORMALIZATION_STATUS",
    "PROJECTED_ANOMALY_FIELDS",
    "V12R8AdjudicationError",
    "adjudicate_r7_plus_prefix",
    "load_and_adjudicate_r7_plus",
    "load_r7_plus_evidence",
    "normalize_v26_prefix_summary",
]
