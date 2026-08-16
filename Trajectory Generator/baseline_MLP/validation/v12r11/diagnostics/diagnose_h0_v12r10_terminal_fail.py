#!/usr/bin/env python3
"""Deterministic offline diagnosis of the frozen V12R10 terminal failure.

This script only reads persisted JSON/source artifacts.  It deliberately does
not import the rollout runtime, OpenSim, Ray, Torch, a teacher, or any project
module.  The only write it can perform is the requested V12R11 result JSON.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[5]
VALIDATION_ROOT = REPO_ROOT / "Trajectory Generator/baseline_MLP/validation"
CASE_ID = "deterministic_offset_plus_0p20"
EXPECTED_STEPS = 500
EXPECTED_ACTION_DIM = 2
EXPECTED_RAW_SAMPLES_PER_STEP = 10
PENETRATION_LIMIT_M = 0.025
MINIMUM_VALID_CYCLES = 2
COUNTER_FIELDS = (
    "teacher_query_count",
    "served_action_teacher_dependency_count",
    "mean_blend_count",
    "safety_intervention_count",
    "safety_latch_activation_count",
    "safety_latch_release_count",
)
FORBIDDEN_TEACHER_FIELDS = {
    "teacher_mean",
    "teacher_action",
    "blended_mean",
    "requested_alpha",
    "effective_alpha",
    "safety_latch_active",
}

R10_RUN = VALIDATION_ROOT / "v12r10/h0_v12r10_run_20260815"
R10_CASE = R10_RUN / "development" / CASE_ID
R6_RUN = VALIDATION_ROOT / "v12r6/h0_v12r6_run_20260814"
R6_CASE = R6_RUN / "development" / CASE_ID
R7_CASE = VALIDATION_ROOT / "v12r7/h0_v12r7_run_20260814/observer_collection" / CASE_ID
R8_CASE = VALIDATION_ROOT / "v12r8/h0_v12r8_run_20260814/observer_collection" / CASE_ID
R9_CASE = VALIDATION_ROOT / "v12r9/h0_v12r9_run_20260814/observer_collection" / CASE_ID
DEFAULT_OUTPUT = (
    Path(__file__).resolve().parent
    / "results"
    / ("h0_v12r10_terminal_fail_diagnosis.json")
)

# Immutable evidence and exact production source used by the failed run.
LOCKED_SHA256 = {
    R10_RUN / "pipeline_ledger.json": (
        "cf50e9450e29abbb8ef9ce759b825a6d5e09905fb6b62d6f2161feed3f6f1cb1"
    ),
    R10_CASE / "run_start.json": (
        "eaf3ca566d799ae5b637fb82e8cef2f3aa7f4895309d002134a212fa2763cd83"
    ),
    R10_CASE / "steps/000211.json": (
        "d3680696bec239cd1dcc84cef59a1e64fd8338e175bf382c836b95b22a964796"
    ),
    R10_CASE / "steps/000212.json": (
        "13e06e9f6bd2a9c80eda58c6fbc64d109a33dc17de25b577a600a4e2f5eb72e1"
    ),
    R10_RUN / "fit/summary.json": (
        "57baa4ec7d1ef6d8f65603078b851d85fcfdd5424fa9529cc43bdfe097c9dd50"
    ),
    R6_RUN / "pipeline_ledger.json": (
        "ce67aea83b1f98aa251ad130af1da25982435381c0aeddeafa0a56bd3274e340"
    ),
    R6_CASE / "run_start.json": (
        "0e9753dc04d25674ba901147e56325e546129489ef5da006d8f5e5d2f3deca65"
    ),
    R6_CASE / "steps/000178.json": (
        "33bb45e11e58f6e4b3911c203c6dfd05367bbc707c3af4eda3affe5ccfe883d1"
    ),
    R6_CASE / "steps/000179.json": (
        "9b29b991ad7764e044e67bda8c381285fc63d5ecc13c74103d22c9997c34c2ad"
    ),
    VALIDATION_ROOT / "v12r6/h0_v12r6_physical_development.py": (
        "453917785a5e6c8ebed13ac813f7c226a8e8a4529ade3992fba9433f52ad4554"
    ),
    VALIDATION_ROOT / "v12r10/h0_v12r10_recovery_contract.py": (
        "a625e7d22ab8f99d6648d077b96308ff7af2d6017b2b2032d5d653aee6b4eaa4"
    ),
    VALIDATION_ROOT / "v12r7/h0_v12r7_recovery_contract.py": (
        "3333d65cf20b238ef80637808a91c103b084edc889fb6acf890f5a754ad0974a"
    ),
    R7_CASE / "summary.json": (
        "b07d25996e144f1102dd98e483d5912903e06f400dc52ccb4db5ce5e4bfd399b"
    ),
    R7_CASE / "gate.json": (
        "d6147af3404b1d8386718f2956de51fbb3efde5a5754d63ffe430607916a03f0"
    ),
    R8_CASE / "adjudication.json": (
        "8482de8672fe5c8fd8381e2fb4be0aec43556b19d7fb9988cf90c6ba764b7f97"
    ),
    R8_CASE / "historical_label_stage_receipt.json": (
        "995064c4fa2627a44740de1d806ef02a8c3fe155d3565e8bbea9f6c8a6b38376"
    ),
    R9_CASE / "import_stage_receipt.json": (
        "a3d58f0f759a15a9af3b45b4be8a0a2bb9ad6067d9673f7e64ad626efed4f9dd"
    ),
}
LOCKED_JOURNALS = {
    "r10": {
        "count": 212,
        "size_bytes": 2_275_738,
        "manifest_sha256": (
            "a311bb924f12127ad4f068aa5653085ffeb21a320f0178335987cf9253408a9e"
        ),
    },
    "r6": {
        "count": 179,
        "size_bytes": 1_924_100,
        "manifest_sha256": (
            "fcf085130223e4c53ba785afe0478da59f2fe6219dcc9658b62cd8b942750c4f"
        ),
    },
}


class DiagnosticError(RuntimeError):
    """Raised when frozen evidence no longer matches the diagnostic lock."""


def _relative(path: Path) -> str:
    return path.resolve().relative_to(REPO_ROOT).as_posix()


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _canonical_bytes(value: Any) -> bytes:
    return json.dumps(
        value,
        allow_nan=False,
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")


def _record(path: Path) -> dict[str, Any]:
    payload = path.read_bytes()
    return {
        "path": _relative(path),
        "sha256": _sha256_bytes(payload),
        "size_bytes": len(payload),
    }


def _load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _finite_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _verify_locked_sources() -> list[dict[str, Any]]:
    records = []
    for path, expected_sha256 in LOCKED_SHA256.items():
        record = _record(path)
        if record["sha256"] != expected_sha256:
            raise DiagnosticError(f"locked artifact drifted: {record['path']}")
        records.append(record)

    physical_source = (
        VALIDATION_ROOT / "v12r6/h0_v12r6_physical_development.py"
    ).read_text(encoding="utf-8")
    r10_contract = (
        VALIDATION_ROOT / "v12r10/h0_v12r10_recovery_contract.py"
    ).read_text(encoding="utf-8")
    r7_contract = (VALIDATION_ROOT / "v12r7/h0_v12r7_recovery_contract.py").read_text(
        encoding="utf-8"
    )
    required_literals = {
        "physical_expected_steps": "EXPECTED_STEPS = 500" in physical_source,
        "physical_sensor_samples": (
            "EXPECTED_RAW_SENSOR_SAMPLES_PER_STEP = 10" in physical_source
        ),
        "physical_audit_failure": (
            'raise V12R6PhysicalDevelopmentError("pure-policy trace audit failed")'
            in physical_source
        ),
        "r10_inherits_penetration": (
            "PENETRATION_LIMIT_M = v12r9.PENETRATION_LIMIT_M" in r10_contract
        ),
        "r7_penetration_value": "PENETRATION_LIMIT_M = 0.025" in r7_contract,
        "r7_minimum_cycles": "MINIMUM_VALID_CYCLES = 2" in r7_contract,
    }
    if not all(required_literals.values()):
        failed = sorted(key for key, value in required_literals.items() if not value)
        raise DiagnosticError(f"production constants drifted: {failed}")
    return records


def _load_step_journal(
    case_root: Path,
    *,
    label: str,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    files = sorted((case_root / "steps").glob("*.json"))
    expected_names = [f"{index:06d}.json" for index in range(1, len(files) + 1)]
    if [path.name for path in files] != expected_names:
        raise DiagnosticError(f"{label} step filenames are not contiguous")

    manifest = [_record(path) for path in files]
    compact_manifest = [
        {
            "path": Path(record["path"]).name,
            "sha256": record["sha256"],
            "size_bytes": record["size_bytes"],
        }
        for record in manifest
    ]
    journal = {
        "algorithm": "sha256(canonical_json([{path,sha256,size_bytes},...]))",
        "count": len(files),
        "size_bytes": sum(record["size_bytes"] for record in manifest),
        "manifest_sha256": _sha256_bytes(_canonical_bytes(compact_manifest)),
        "first_step": manifest[0] if manifest else None,
        "penultimate_step": manifest[-2] if len(manifest) > 1 else None,
        "final_step": manifest[-1] if manifest else None,
    }
    expected = LOCKED_JOURNALS[label]
    if any(journal[key] != expected[key] for key in expected):
        raise DiagnosticError(f"{label} frozen step journal drifted")

    rows = [_load_json(path) for path in files]
    if any(row.get("step") != index for index, row in enumerate(rows, start=1)):
        raise DiagnosticError(f"{label} row steps are not contiguous")
    return rows, journal


def _trace_audit(
    rows: Sequence[Any],
    *,
    schema_version: int,
    protocol_id: str,
    stage_id: str,
    require_full_horizon: bool,
) -> dict[str, Any]:
    """Mirror the production audit, or audit the persisted prefix itself."""

    counters = {name: 0 for name in COUNTER_FIELDS}
    initial = len(rows) == EXPECTED_STEPS if require_full_horizon else True
    schema_exact = initial
    identity_exact = initial
    action_path_exact = initial
    raw_sensor_exact = initial
    forbidden_teacher_payload_absent = initial
    per_row_zero_counters = initial

    for expected_step, row in enumerate(rows, start=1):
        if not isinstance(row, Mapping):
            schema_exact = False
            identity_exact = False
            action_path_exact = False
            raw_sensor_exact = False
            forbidden_teacher_payload_absent = False
            per_row_zero_counters = False
            continue
        identity_exact = identity_exact and (
            row.get("step") == expected_step
            and row.get("schema_version") == schema_version
            and row.get("protocol_id") == protocol_id
            and row.get("stage_id") == stage_id
            and row.get("case_id") == CASE_ID
        )
        schema_exact = schema_exact and all(
            type(row.get(name)) is int for name in COUNTER_FIELDS
        )
        for name in COUNTER_FIELDS:
            value = row.get(name)
            if type(value) is int:
                counters[name] += value
            per_row_zero_counters = per_row_zero_counters and value == 0
        schema_exact = schema_exact and (
            row.get("teacher_enabled") is False
            and row.get("blending_enabled") is False
            and row.get("safety_latch_enabled") is False
        )
        forbidden_teacher_payload_absent = (
            forbidden_teacher_payload_absent
            and FORBIDDEN_TEACHER_FIELDS.isdisjoint(row)
        )
        mean = row.get("candidate_mean")
        noise = row.get("single_noise")
        action = row.get("raw_action")
        vectors = (mean, noise, action)
        action_path_exact = action_path_exact and all(
            isinstance(value, list)
            and len(value) == EXPECTED_ACTION_DIM
            and all(_finite_number(item) for item in value)
            for value in vectors
        )
        if action_path_exact:
            action_path_exact = all(
                math.isclose(
                    float(action[index]),
                    float(mean[index]) + float(noise[index]),
                    rel_tol=0.0,
                    abs_tol=1.0e-7,
                )
                for index in range(EXPECTED_ACTION_DIM)
            )
        journal = row.get("observer_raw_sensor_journal")
        samples = journal.get("samples") if isinstance(journal, Mapping) else None
        raw_sensor_exact = raw_sensor_exact and (
            row.get("raw_sensor_sample_count") == EXPECTED_RAW_SAMPLES_PER_STEP
            and isinstance(samples, list)
            and len(samples) == EXPECTED_RAW_SAMPLES_PER_STEP
        )

    zero_counters = all(value == 0 for value in counters.values())
    passed = all(
        (
            schema_exact,
            identity_exact,
            action_path_exact,
            raw_sensor_exact,
            forbidden_teacher_payload_absent,
            per_row_zero_counters,
            zero_counters,
        )
    )
    return {
        "passed": passed,
        "row_count": len(rows),
        "schema_exact": schema_exact,
        "identity_exact": identity_exact,
        "candidate_mean_plus_noise_exact": action_path_exact,
        "raw_sensor_samples_exact": raw_sensor_exact,
        "forbidden_teacher_payload_absent": forbidden_teacher_payload_absent,
        "per_row_zero_counters": per_row_zero_counters,
        "zero_counters": zero_counters,
        "counters": counters,
    }


def _first_crossing(
    rows: Sequence[Mapping[str, Any]], threshold: float
) -> dict[str, Any] | None:
    for index, row in enumerate(rows):
        penetration = float(row["grf_penetration_m"])
        if penetration >= threshold:
            previous = rows[index - 1] if index else None
            return {
                "threshold_m": threshold,
                "step": row["step"],
                "time_s": row["time_s"],
                "penetration_m": penetration,
                "margin_m": penetration - threshold,
                "previous_step": None if previous is None else previous["step"],
                "previous_penetration_m": (
                    None if previous is None else previous["grf_penetration_m"]
                ),
            }
    return None


def _terminal_increasing_suffix(
    rows: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    start = len(rows) - 1
    while start > 0 and (
        float(rows[start]["grf_penetration_m"])
        > float(rows[start - 1]["grf_penetration_m"])
    ):
        start -= 1
    selected = rows[start:]
    first = selected[0]
    last = selected[-1]
    delta_m = float(last["grf_penetration_m"]) - float(first["grf_penetration_m"])
    duration_s = float(last["time_s"]) - float(first["time_s"])
    return {
        "strictly_increasing": True,
        "row_count": len(selected),
        "start_step": first["step"],
        "end_step": last["step"],
        "start_time_s": first["time_s"],
        "end_time_s": last["time_s"],
        "start_penetration_m": first["grf_penetration_m"],
        "end_penetration_m": last["grf_penetration_m"],
        "delta_m": delta_m,
        "duration_s": duration_s,
        "mean_finite_difference_m_per_s": delta_m / duration_s,
        "rows": [
            {
                "step": row["step"],
                "time_s": row["time_s"],
                "penetration_m": row["grf_penetration_m"],
                "raw_action": row["raw_action"],
                "applied_action": row["applied_action"],
                "state_name": row["phase_fsm"]["state_name"],
                "expected_next_event": row["phase_fsm"]["expected_next_event"],
                "end_reason": row["end_reason"],
            }
            for row in selected
        ],
    }


def _state_segments(rows: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    segments: list[dict[str, Any]] = []
    for row in rows:
        state = row["phase_fsm"]["state_name"]
        if not segments or segments[-1]["state_name"] != state:
            segments.append(
                {
                    "state_name": state,
                    "start_step": row["step"],
                    "end_step": row["step"],
                    "start_time_s": row["time_s"],
                    "end_time_s": row["time_s"],
                }
            )
        else:
            segments[-1]["end_step"] = row["step"]
            segments[-1]["end_time_s"] = row["time_s"]
    return segments


def _event_timeline(rows: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    events = []
    for row in rows:
        phase = row["phase_fsm"]
        for event in phase["accepted_transitions_this_step"]:
            events.append(
                {
                    "delivered_step": row["step"],
                    "delivered_time_s": event["delivered_time_s"],
                    "event": event["event"],
                    "event_time_s": event["event_time_s"],
                    "confirmed_time_s": event["confirmed_time_s"],
                    "from_state_id": event["from_state_id"],
                    "to_state_id": event["to_state_id"],
                    "closed_segment_type": event["closed_segment_type"],
                    "segment_valid": event["segment_valid"],
                    "anchor_geometry_valid": event["anchor_geometry_valid"],
                }
            )
    return events


def _false_check_counts(rows: Sequence[Mapping[str, Any]]) -> dict[str, int]:
    names = sorted({name for row in rows for name in row["checks"]})
    return {
        name: sum(row["checks"].get(name) is not True for row in rows) for name in names
    }


def _max_field(rows: Sequence[Mapping[str, Any]], field: str) -> dict[str, int | float]:
    row = max(rows, key=lambda item: float(item[field]))
    return {"step": row["step"], "value": row[field]}


def _run_diagnosis(
    *,
    label: str,
    run_root: Path,
    case_root: Path,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    ledger = _load_json(run_root / "pipeline_ledger.json")
    run_start = _load_json(case_root / "run_start.json")
    rows, journal = _load_step_journal(case_root, label=label)
    final = rows[-1]
    previous = rows[-2]
    stage_id = f"development__{CASE_ID}"
    production_audit = _trace_audit(
        rows,
        schema_version=run_start["schema_version"],
        protocol_id=run_start["protocol_id"],
        stage_id=stage_id,
        require_full_horizon=True,
    )
    prefix_audit = _trace_audit(
        rows,
        schema_version=run_start["schema_version"],
        protocol_id=run_start["protocol_id"],
        stage_id=stage_id,
        require_full_horizon=False,
    )
    false_checks = _false_check_counts(rows)
    clipped = [
        {
            "step": row["step"],
            "raw_action": row["raw_action"],
            "applied_action": row["applied_action"],
            "maximum_bound_overshoot": max(
                max(0.0, abs(float(value)) - 1.0) for value in row["raw_action"]
            ),
        }
        for row in rows
        if row["raw_action"] != row["applied_action"]
    ]
    terminal_rows = [
        row["step"] for row in rows if row["terminated"] or row["truncated"]
    ]
    terminal_binding = {
        "ledger_status_terminal_fail": ledger["status"].endswith("PIPELINE_TERMINAL"),
        "ledger_passed_false": ledger["passed"] is False,
        "ledger_attempted_stage_exact": ledger["attempted_stage"] == stage_id,
        "ledger_error_exact": ledger["error"]
        == {
            "message": "pure-policy trace audit failed",
            "type": "V12R6PhysicalDevelopmentError",
        },
        "run_start_stage_exact": run_start["stage_id"] == stage_id,
        "run_start_case_exact": run_start["case"]["case_id"] == CASE_ID,
        "candidate_id_exact": ledger["candidate_id"] == run_start["candidate_id"],
        "activity_step_count_exact": (
            ledger["activity_totals"].get(
                "development_environment_step_calls",
                ledger["activity_totals"]["environment_step_calls"],
            )
            == len(rows)
        ),
        "activity_sensor_count_exact": (
            ledger["activity_totals"]["raw_sensor_sample_count"]
            == len(rows) * EXPECTED_RAW_SAMPLES_PER_STEP
        ),
        "single_terminal_row_is_final": terminal_rows == [final["step"]],
        "final_end_reason_penetration": final["end_reason"] == "grf_penetration",
        "final_terminated_not_truncated": (
            final["terminated"] is True and final["truncated"] is False
        ),
        "first_limit_crossing_is_final": (
            float(previous["grf_penetration_m"]) < PENETRATION_LIMIT_M
            and float(final["grf_penetration_m"]) >= PENETRATION_LIMIT_M
        ),
    }
    if not all(terminal_binding.values()):
        failed = sorted(key for key, value in terminal_binding.items() if not value)
        raise DiagnosticError(f"{label} terminal binding failed: {failed}")

    phase_final = final["phase_fsm"]
    diagnosis = {
        "ledger": {
            "record": _record(run_root / "pipeline_ledger.json"),
            "status": ledger["status"],
            "passed": ledger["passed"],
            "attempted_stage": ledger["attempted_stage"],
            "error": ledger["error"],
            "candidate_id": ledger["candidate_id"],
            "activity_totals": ledger["activity_totals"],
        },
        "run_start": {
            "record": _record(case_root / "run_start.json"),
            "status": run_start["status"],
            "protocol_id": run_start["protocol_id"],
            "schema_version": run_start["schema_version"],
            "candidate_id": run_start["candidate_id"],
            "case": run_start["case"],
            "teacher_enabled": run_start["teacher_enabled"],
            "blending_enabled": run_start["blending_enabled"],
            "safety_latch_enabled": run_start["safety_latch_enabled"],
            "binary_detector_required": run_start.get("binary_detector_required"),
            "morphology_weight_required": run_start.get("morphology_weight_required"),
        },
        "journal_binding": journal,
        "terminal_binding": terminal_binding,
        "production_pure_policy_trace_audit_reconstruction": production_audit,
        "prefix_semantic_audit": prefix_audit,
        "proximal_audit_failure": {
            "control": "pure_policy_trace_audit.full_horizon_initialization",
            "expected_rows": EXPECTED_STEPS,
            "actual_rows": len(rows),
            "missing_rows": EXPECTED_STEPS - len(rows),
            "explanation": (
                "The production audit initializes six semantic booleans from "
                "len(trace) == 500. Early physical termination therefore makes "
                "those booleans false even though every persisted prefix row "
                "passes the same row-level semantic checks."
            ),
        },
        "physical_trigger": {
            "threshold_m": PENETRATION_LIMIT_M,
            "first_crossing": _first_crossing(rows, PENETRATION_LIMIT_M),
            "terminal_step": final["step"],
            "terminal_time_s": final["time_s"],
            "terminal_penetration_m": final["grf_penetration_m"],
            "terminal_margin_m": (
                float(final["grf_penetration_m"]) - PENETRATION_LIMIT_M
            ),
            "terminal_margin_fraction_of_limit": (
                float(final["grf_penetration_m"]) - PENETRATION_LIMIT_M
            )
            / PENETRATION_LIMIT_M,
            "terminal_increment_from_previous_m": (
                float(final["grf_penetration_m"]) - float(previous["grf_penetration_m"])
            ),
            "previous_step": previous["step"],
            "previous_penetration_m": previous["grf_penetration_m"],
            "terminal_end_reason": final["end_reason"],
            "terminal_reward": final["reward"],
        },
        "penetration_threshold_crossings": {
            str(threshold): _first_crossing(rows, threshold)
            for threshold in (0.015, 0.020, 0.024, PENETRATION_LIMIT_M)
        },
        "terminal_penetration_ramp": _terminal_increasing_suffix(rows),
        "actions": {
            "clipped_step_count": len(clipped),
            "clipped_scalar_count": sum(
                sum(
                    raw != applied
                    for raw, applied in zip(
                        row["raw_action"], row["applied_action"], strict=True
                    )
                )
                for row in rows
            ),
            "clipped_steps": clipped,
            "raw_min_by_dimension": [
                min(float(row["raw_action"][index]) for row in rows)
                for index in range(EXPECTED_ACTION_DIM)
            ],
            "raw_max_by_dimension": [
                max(float(row["raw_action"][index]) for row in rows)
                for index in range(EXPECTED_ACTION_DIM)
            ],
            "terminal_raw_action": final["raw_action"],
            "terminal_applied_action": final["applied_action"],
            "last_clipped_step_to_terminal_gap_steps": (
                None if not clipped else final["step"] - clipped[-1]["step"]
            ),
        },
        "fsm_and_events": {
            "event_source_values": sorted(
                {row["phase_fsm"]["event_source"] for row in rows}
            ),
            "state_segments": _state_segments(rows),
            "accepted_event_timeline": _event_timeline(rows),
            "accepted_event_count": sum(
                len(row["phase_fsm"]["accepted_transitions_this_step"]) for row in rows
            ),
            "final_state": phase_final["state_name"],
            "final_expected_next_event": phase_final["expected_next_event"],
            "valid_hs_count": phase_final["valid_hs_count"],
            "valid_to_count": phase_final["valid_to_count"],
            "valid_cycle_count": phase_final["valid_cycle_count"],
            "minimum_valid_cycles": MINIMUM_VALID_CYCLES,
            "invalid_event_count": phase_final["invalid_event_count"],
            "timeout_exceeded": phase_final["timeout_exceeded"],
        },
        "runtime_checks": {
            "false_row_check_counts": false_checks,
            "all_rows_finite": false_checks["finite"] == 0,
            "binary_active_routing_exact_all_rows": (
                false_checks["binary_active_routing_exact"] == 0
            ),
            "hard_invalid_failures": false_checks["no_hard_invalid"],
            "so_unaccepted_failures": false_checks["no_unaccepted_so"],
            "sea_fallback_failures": false_checks["no_sea_fallback"],
            "timeout_failures": false_checks["no_timeout"],
            "control_window_failures": false_checks["ten_control_windows"],
            "sensor_sample_failures": false_checks["ten_sensor_samples"],
            "raw_sensor_sample_count": sum(
                int(row["raw_sensor_sample_count"]) for row in rows
            ),
            "max_reserve_norm_nm": _max_field(rows, "reserve_norm_nm"),
            "max_residual_norm_nm": _max_field(rows, "residual_norm_nm"),
            "note": (
                "A GRF penetration termination is separate from the persisted "
                "no_hard_invalid check, so zero hard-invalid failures do not "
                "contradict the terminal end_reason."
            ),
        },
        "counterfactual_development_gate_blockers": {
            "official_gate_emitted": False,
            "blocked_before_summary_and_gate": True,
            "full_horizon": len(rows) == EXPECTED_STEPS,
            "pure_trace": production_audit["passed"],
            "penetration_below_limit": (
                max(float(row["grf_penetration_m"]) for row in rows)
                < PENETRATION_LIMIT_M
            ),
            "minimum_valid_cycles": (
                phase_final["valid_cycle_count"] >= MINIMUM_VALID_CYCLES
            ),
            "zero_action_clipping": not clipped,
            "zero_hard_invalid_so_fallback": (
                false_checks["no_hard_invalid"]
                == false_checks["no_unaccepted_so"]
                == false_checks["no_sea_fallback"]
                == 0
            ),
        },
        "maxima": {
            "penetration_m": _max_field(rows, "grf_penetration_m"),
            "reserve_norm_nm": _max_field(rows, "reserve_norm_nm"),
            "residual_norm_nm": _max_field(rows, "residual_norm_nm"),
        },
    }
    return diagnosis, rows


def _mean_absolute(values: Sequence[float]) -> float:
    return sum(abs(value) for value in values) / len(values)


def _root_mean_square(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values))


def _historical_lineage() -> dict[str, Any]:
    r7_summary_path = R7_CASE / "summary.json"
    r7_gate_path = R7_CASE / "gate.json"
    r8_adjudication_path = R8_CASE / "adjudication.json"
    r8_labels_path = R8_CASE / "historical_label_stage_receipt.json"
    r9_import_path = R9_CASE / "import_stage_receipt.json"
    r7_summary = _load_json(r7_summary_path)
    r7_gate = _load_json(r7_gate_path)
    r8_adjudication = _load_json(r8_adjudication_path)
    r8_labels = _load_json(r8_labels_path)
    r9_import = _load_json(r9_import_path)
    return {
        "r7_reproduced_r6_prefix": {
            "summary_record": _record(r7_summary_path),
            "gate_record": _record(r7_gate_path),
            "steps": r7_summary["steps"],
            "end_reason": r7_summary["end_reason"],
            "terminal_penetration_m": r7_summary["grf_penetration_max_m"],
            "reproduction_audit": r7_summary["r6_plus_reproduction_audit"],
            "gate_status": r7_gate["status"],
            "gate_passed": r7_gate["passed"],
        },
        "r8_adjudicated_and_labelled_prefix": {
            "adjudication_record": _record(r8_adjudication_path),
            "label_stage_record": _record(r8_labels_path),
            "adjudication_status": r8_adjudication["status"],
            "adjudication_passed": r8_adjudication["passed"],
            "label_stage_status": r8_labels["status"],
            "labelled_row_count": r8_labels["labelled_row_count"],
            "same_state_teacher_label_count": (
                r8_labels["same_state_teacher_label_count"]
            ),
        },
        "r9_imported_same_179_labels": {
            "import_record": _record(r9_import_path),
            "status": r9_import["status"],
            "passed": r9_import["passed"],
            "labelled_row_count": r9_import["labelled_row_count"],
            "teacher_query_count": r9_import["teacher_query_count"],
        },
    }


def _compare_runs(
    r10: Mapping[str, Any],
    r10_rows: Sequence[Mapping[str, Any]],
    r6: Mapping[str, Any],
    r6_rows: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    shared = min(len(r10_rows), len(r6_rows))
    action_deltas = [
        [
            float(r10_rows[index]["raw_action"][dimension])
            - float(r6_rows[index]["raw_action"][dimension])
            for index in range(shared)
        ]
        for dimension in range(EXPECTED_ACTION_DIM)
    ]
    penetration_deltas = [
        float(r10_rows[index]["grf_penetration_m"])
        - float(r6_rows[index]["grf_penetration_m"])
        for index in range(shared)
    ]
    r6_terminal_step = int(r6["physical_trigger"]["terminal_step"])
    r10_at_r6_terminal = r10_rows[r6_terminal_step - 1]
    initial_observation_max_abs_difference = max(
        abs(float(left) - float(right))
        for left, right in zip(
            r10_rows[0]["v26_observation"],
            r6_rows[0]["v26_observation"],
            strict=True,
        )
    )
    return {
        "shared_prefix_steps": shared,
        "same_initial_v26_observation": (initial_observation_max_abs_difference == 0.0),
        "initial_observation_max_abs_difference": (
            initial_observation_max_abs_difference
        ),
        "r6_terminal": r6["physical_trigger"],
        "r10_terminal": r10["physical_trigger"],
        "r10_at_r6_terminal_step": {
            "step": r10_at_r6_terminal["step"],
            "time_s": r10_at_r6_terminal["time_s"],
            "penetration_m": r10_at_r6_terminal["grf_penetration_m"],
            "margin_below_limit_m": (
                PENETRATION_LIMIT_M - float(r10_at_r6_terminal["grf_penetration_m"])
            ),
            "terminated": r10_at_r6_terminal["terminated"],
            "raw_action": r10_at_r6_terminal["raw_action"],
        },
        "horizon_extension": {
            "steps": len(r10_rows) - len(r6_rows),
            "simulated_time_s": (
                float(r10_rows[-1]["time_s"]) - float(r6_rows[-1]["time_s"])
            ),
            "relative_to_r6": (len(r10_rows) / len(r6_rows)) - 1.0,
            "r10_fraction_of_required_horizon": len(r10_rows) / EXPECTED_STEPS,
            "r6_fraction_of_required_horizon": len(r6_rows) / EXPECTED_STEPS,
        },
        "shared_prefix_action_difference": {
            f"dimension_{dimension}": {
                "mean_absolute": _mean_absolute(action_deltas[dimension]),
                "root_mean_square": _root_mean_square(action_deltas[dimension]),
                "maximum_absolute": max(abs(x) for x in action_deltas[dimension]),
            }
            for dimension in range(EXPECTED_ACTION_DIM)
        },
        "shared_prefix_penetration_difference_m": {
            "mean_absolute": _mean_absolute(penetration_deltas),
            "root_mean_square": _root_mean_square(penetration_deltas),
            "maximum_absolute": max(abs(value) for value in penetration_deltas),
        },
        "fsm_event_timing": {
            "r6": r6["fsm_and_events"]["accepted_event_timeline"],
            "r10": r10["fsm_and_events"]["accepted_event_timeline"],
        },
        "clipping_comparison": {
            "r6_clipped_steps": r6["actions"]["clipped_step_count"],
            "r10_clipped_steps": r10["actions"]["clipped_step_count"],
        },
        "assessment": {
            "known_terminal_failure_mode_persists": True,
            "runtime_or_detector_regression_supported": False,
            "candidate_extended_horizon_but_did_not_close_risk": True,
            "new_candidate_specific_clipping_vs_r6": True,
            "basis": (
                "R6 and its exact 179-row R7 runtime-field reproduction already "
                "terminated in "
                "the same hardest case with end_reason=grf_penetration. R10 "
                "survived 33 additional steps and was below the limit at the "
                "R6 boundary, but later crossed the same unchanged 0.025 m "
                "limit and introduced four action-0 clipping steps absent in R6."
            ),
        },
    }


def build_result() -> dict[str, Any]:
    locked_records = _verify_locked_sources()
    r10, r10_rows = _run_diagnosis(label="r10", run_root=R10_RUN, case_root=R10_CASE)
    r6, r6_rows = _run_diagnosis(label="r6", run_root=R6_RUN, case_root=R6_CASE)
    fit_summary_path = R10_RUN / "fit/summary.json"
    fit_summary = _load_json(fit_summary_path)
    comparison = _compare_runs(r10, r10_rows, r6, r6_rows)
    result = {
        "schema_version": 1,
        "status": "PASS_H0_V12R11_R10_TERMINAL_DIAGNOSIS",
        "diagnostic_passed": True,
        "execution_mode": {
            "persisted_artifacts_only": True,
            "environment_executed": False,
            "fit_executed": False,
            "teacher_queried": False,
            "retry_or_resume_executed": False,
        },
        "diagnostic_script": _record(Path(__file__).resolve()),
        "locked_input_records": locked_records,
        "production_contract": {
            "expected_steps": EXPECTED_STEPS,
            "raw_sensor_samples_per_step": EXPECTED_RAW_SAMPLES_PER_STEP,
            "penetration_limit_m": PENETRATION_LIMIT_M,
            "minimum_valid_cycles": MINIMUM_VALID_CYCLES,
            "hardest_first_case_id": CASE_ID,
        },
        "r10": r10,
        "r6_reference": r6,
        "historical_lineage": _historical_lineage(),
        "r10_offline_fit_context": {
            "record": _record(fit_summary_path),
            "status": fit_summary["status"],
            "passed": fit_summary["passed"],
            "sample_count": fit_summary["sample_count"],
            "observer_plus_case_metrics": fit_summary["observer_case_metrics"][CASE_ID],
            "observer_plus_late_metrics": fit_summary["observer_plus_late_metrics"],
            "limitation_ids": [
                limitation["id"] for limitation in fit_summary["limitations"]
            ],
        },
        "comparison": comparison,
        "fail_closed_disposition": {
            "candidate_promotable": False,
            "training_ready_claim_allowed": False,
            "qualification_allowed": False,
            "same_r10_retry_or_resume_allowed": False,
            "same_r10_artifact_mutation_allowed": False,
            "decision": "KEEP_R10_TERMINAL_FAIL_IMMUTABLE",
            "successor_requirements": [
                "Open a new protocol/candidate identity; do not retry R10.",
                (
                    "Treat full 500-step completion, penetration < 0.025 m, "
                    "at least two valid cycles, and zero action clipping as "
                    "independent mandatory gates."
                ),
                (
                    "Target the post-heel-strike stance recovery tail and bind "
                    "a saturation-aware action check before any new one-shot."
                ),
                (
                    "Run the unchanged hardest +0.20 case first and stop on "
                    "the first failed gate."
                ),
            ],
        },
    }
    result["canonical_payload_sha256_excluding_this_field"] = _sha256_bytes(
        _canonical_bytes(result)
    )
    return result


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help="Result JSON path (default: diagnostics/results).",
    )
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="Recompute and validate without writing a result file.",
    )
    parser.add_argument(
        "--stdout",
        action="store_true",
        help="Also print the complete result JSON.",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    result = build_result()
    payload = (
        json.dumps(
            result,
            allow_nan=False,
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        )
        + "\n"
    )
    if not args.check_only:
        output = args.output.expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        temporary = output.with_suffix(output.suffix + ".tmp")
        temporary.write_text(payload, encoding="utf-8")
        temporary.replace(output)
        print(f"wrote {_relative(output)} sha256={_sha256_bytes(payload.encode())}")
    else:
        print(f"check-only payload_sha256={_sha256_bytes(payload.encode())}")
    if args.stdout:
        print(payload, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
