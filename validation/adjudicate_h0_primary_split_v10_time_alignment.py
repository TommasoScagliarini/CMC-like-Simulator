"""Adjudicate the V10 pre-step/post-step time-alignment bug offline.

The frozen V10 protocol and its FAIL ledger are never rewritten.  This script
only checks whether the persisted evidence would pass when the historical
post-step timestamp is compared with the corresponding post-step boundary.
It performs no simulation and no model update.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
import re
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))

import h0_primary_split_v10_coherent_teacher_probe_contract as contract  # noqa: E402


SCHEMA_VERSION = 94
STATUS = "PASS_H0_PRIMARY_SPLIT_V10_TIME_ALIGNMENT_ADJUDICATION"
RECEIPT = VALIDATION_ROOT / "h0_primary_split_v10_time_alignment_adjudication_receipt.json"
RUN_ROOT = (
    VALIDATION_ROOT
    / "h0_primary_grf_split_adaptation_runs"
    / "2026-08-07_h0_primary_split_v10_coherent_teacher_probe"
)
ROLLOUT_ROOT = RUN_ROOT / contract.CASE_ID
TRACE = ROLLOUT_ROOT / "trace.json"
SUMMARY = ROLLOUT_ROOT / "summary.json"
GATE = ROLLOUT_ROOT / "gate.json"
FAILURE = ROLLOUT_ROOT / "failure.json"
LEDGER = RUN_ROOT / "execution_ledger.json"
LOCK = VALIDATION_ROOT / "h0_primary_split_v10_coherent_teacher_probe_execution_lock.json"
V6_RUNNER = VALIDATION_ROOT / "run_h0_primary_grf_split_v6_teacher_replay.py"
TEST_SOURCE = VALIDATION_ROOT / "test_adjudicate_h0_primary_split_v10_time_alignment.py"
POLICY_DT_S = 0.01
HISTORICAL_TIME_TOLERANCE_S = 1.0e-12
RAW_SAMPLE_TIME_TOLERANCE_S = 1.0e-9


class V10TimeAdjudicationError(RuntimeError):
    """Raised if the frozen evidence cannot support the narrow adjudication."""


def _reject_constant(value: str) -> None:
    raise V10TimeAdjudicationError(f"non-finite JSON constant: {value}")


def _load(path: Path) -> Any:
    try:
        with path.open("r", encoding="utf-8") as handle:
            return json.load(handle, parse_constant=_reject_constant)
    except (OSError, ValueError, TypeError) as exc:
        raise V10TimeAdjudicationError(f"cannot read strict JSON {path}") from exc


def _mapping(path: Path) -> dict[str, Any]:
    value = _load(path)
    if not isinstance(value, Mapping):
        raise V10TimeAdjudicationError(f"expected JSON object: {path}")
    return dict(value)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _record(path: Path) -> dict[str, Any]:
    resolved = path.resolve()
    try:
        relative = resolved.relative_to(REPO_ROOT).as_posix()
    except ValueError as exc:
        raise V10TimeAdjudicationError(f"artifact is outside repository: {path}") from exc
    if not resolved.is_file():
        raise V10TimeAdjudicationError(f"artifact is missing: {path}")
    return {
        "path": relative,
        "sha256": _sha256(resolved),
        "size_bytes": resolved.stat().st_size,
    }


def _canonical_bytes(value: Any) -> bytes:
    return (
        json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")


def _write_exclusive(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = _canonical_bytes(value)
    try:
        descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    except FileExistsError as exc:
        raise V10TimeAdjudicationError(f"receipt already exists: {path}") from exc
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
    except Exception:
        try:
            path.unlink()
        except OSError:
            pass
        raise


def _finite(value: Any, *, label: str) -> float:
    if isinstance(value, bool):
        raise V10TimeAdjudicationError(f"{label} is not numeric")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise V10TimeAdjudicationError(f"{label} is not numeric") from exc
    if not math.isfinite(result):
        raise V10TimeAdjudicationError(f"{label} is not finite")
    return result


def _verify_frozen_sources(lock: Mapping[str, Any]) -> tuple[int, list[str]]:
    sources = lock.get("sources")
    if not isinstance(sources, Mapping):
        raise V10TimeAdjudicationError("V10 lock has no source manifest")
    drifted: list[str] = []
    for name, raw_record in sources.items():
        if not isinstance(raw_record, Mapping):
            raise V10TimeAdjudicationError(f"source record {name} is malformed")
        relative = raw_record.get("path")
        if not isinstance(relative, str):
            raise V10TimeAdjudicationError(f"source record {name} has no path")
        observed = _record(REPO_ROOT / relative)
        if observed != dict(raw_record):
            drifted.append(str(name))
    return len(sources), drifted


def _row_audit(rows: Sequence[Any]) -> dict[str, Any]:
    if len(rows) != contract.EXPECTED_STEPS:
        raise V10TimeAdjudicationError("V10 trace is not 500 rows")
    corrected_deltas: list[float] = []
    recorded_deltas: list[float] = []
    next_boundary_deltas: list[float] = []
    raw_sample_deltas: list[float] = []
    raw_fsm_exact_count = 0
    non_time_failed_checks: dict[str, int] = {}
    time_check_failure_count = 0
    for position, raw in enumerate(rows):
        step = position + 1
        if not isinstance(raw, Mapping) or raw.get("step") != step:
            raise V10TimeAdjudicationError(f"trace row {step} is malformed")
        checks = raw.get("checks")
        if not isinstance(checks, Mapping):
            raise V10TimeAdjudicationError(f"trace row {step} has no checks")
        for name, passed in checks.items():
            if name == "historical_time_float64_byte_exact":
                if passed is not False:
                    raise V10TimeAdjudicationError(
                        f"trace row {step} did not record the known time failure"
                    )
                time_check_failure_count += 1
            elif passed is not True:
                non_time_failed_checks[name] = non_time_failed_checks.get(name, 0) + 1

        before = _finite(raw.get("boundary_time_s"), label=f"row {step} boundary")
        historical_after = _finite(
            raw.get("historical_baseline_time_s"),
            label=f"row {step} historical time",
        )
        recorded_deltas.append(historical_after - before)
        corrected_deltas.append(abs((before + POLICY_DT_S) - historical_after))
        if position + 1 < len(rows):
            next_boundary = _finite(
                rows[position + 1].get("boundary_time_s"),
                label=f"row {step + 1} boundary",
            )
            next_boundary_deltas.append(abs(next_boundary - historical_after))

        samples = raw.get("binary_phase_sensor_samples")
        binary_fsm = raw.get("binary_phase_fsm")
        if (
            not isinstance(samples, Sequence)
            or isinstance(samples, (str, bytes, bytearray))
            or not samples
            or not isinstance(samples[-1], Mapping)
            or not isinstance(binary_fsm, Mapping)
        ):
            raise V10TimeAdjudicationError(f"row {step} lacks persisted timing evidence")
        raw_time = _finite(samples[-1].get("time_s"), label=f"row {step} raw time")
        fsm_time = _finite(
            binary_fsm.get("last_sample_time_s"),
            label=f"row {step} FSM sample time",
        )
        raw_fsm_exact_count += int(raw_time == fsm_time)
        raw_sample_deltas.append(abs(raw_time - historical_after))

    return {
        "row_count": len(rows),
        "time_check_failure_count": time_check_failure_count,
        "non_time_failed_checks": non_time_failed_checks,
        "recorded_offset_min_s": min(recorded_deltas),
        "recorded_offset_max_s": max(recorded_deltas),
        "recorded_offset_error_from_policy_dt_max_s": max(
            abs(value - POLICY_DT_S) for value in recorded_deltas
        ),
        "corrected_post_step_time_max_abs_error_s": max(corrected_deltas),
        "corrected_post_step_time_within_tolerance_count": sum(
            value <= HISTORICAL_TIME_TOLERANCE_S for value in corrected_deltas
        ),
        "next_boundary_post_step_within_tolerance_count": sum(
            value <= HISTORICAL_TIME_TOLERANCE_S for value in next_boundary_deltas
        ),
        "next_boundary_comparison_count": len(next_boundary_deltas),
        "raw_sample_fsm_time_exact_count": raw_fsm_exact_count,
        "raw_sample_time_max_abs_error_s": max(raw_sample_deltas),
        "raw_sample_time_within_runtime_tolerance_count": sum(
            value <= RAW_SAMPLE_TIME_TOLERANCE_S for value in raw_sample_deltas
        ),
    }


def build_adjudication() -> dict[str, Any]:
    ledger = _mapping(LEDGER)
    gate = _mapping(GATE)
    summary = _mapping(SUMMARY)
    failure = _mapping(FAILURE)
    lock = _mapping(LOCK)
    rows = _load(TRACE)
    if not isinstance(rows, list):
        raise V10TimeAdjudicationError("V10 trace is not an array")
    source_count, drifted_sources = _verify_frozen_sources(lock)
    row_audit = _row_audit(rows)

    v6_source = V6_RUNNER.read_text(encoding="utf-8")
    v6_tolerance_frozen = bool(
        re.search(r"^TIME_TOLERANCE_S\s*=\s*1\.0e-12\s*$", v6_source, re.MULTILINE)
    )
    failed_gate_checks = sorted(
        name for name, passed in gate.get("checks", {}).items() if passed is not True
    )
    corrected = copy.deepcopy(summary)
    corrected["historical_time_mismatch_count"] = 0
    corrected["step_contract_failure_count"] = 0
    corrected_gate = contract.rollout_gate(corrected)
    changed_fields = sorted(
        name
        for name in set(summary) | set(corrected)
        if summary.get(name) != corrected.get(name)
    )

    checks = {
        "original_protocol_fail_preserved": (
            ledger.get("status") == contract.PROTOCOL_FAIL_STATUS
            and ledger.get("passed") is False
            and ledger.get("retry_authorized") is False
        ),
        "original_rollout_fail_preserved": (
            gate.get("status") == contract.ROLLOUT_FAIL_STATUS
            and gate.get("passed") is False
            and failure.get("status") == contract.ROLLOUT_FAIL_STATUS
        ),
        "only_aggregate_gate_failure_is_zero_failures": failed_gate_checks
        == ["zero_failures"],
        "only_per_step_failure_is_misaligned_time": (
            row_audit["time_check_failure_count"] == contract.EXPECTED_STEPS
            and row_audit["non_time_failed_checks"] == {}
        ),
        "summary_failure_counts_are_only_time_derived": (
            summary.get("historical_time_mismatch_count") == contract.EXPECTED_STEPS
            and summary.get("step_contract_failure_count") == contract.EXPECTED_STEPS
        ),
        "recorded_offset_is_exactly_one_policy_step": (
            row_audit["recorded_offset_error_from_policy_dt_max_s"]
            <= HISTORICAL_TIME_TOLERANCE_S
        ),
        "corrected_post_step_time_passes_historical_tolerance": (
            row_audit["corrected_post_step_time_within_tolerance_count"]
            == contract.EXPECTED_STEPS
        ),
        "next_boundary_confirms_first_499_post_step_times": (
            row_audit["next_boundary_comparison_count"]
            == contract.EXPECTED_STEPS - 1
            and row_audit["next_boundary_post_step_within_tolerance_count"]
            == contract.EXPECTED_STEPS - 1
        ),
        "raw_sensor_and_fsm_times_are_consistent": (
            row_audit["raw_sample_fsm_time_exact_count"]
            == contract.EXPECTED_STEPS
            and row_audit["raw_sample_time_within_runtime_tolerance_count"]
            == contract.EXPECTED_STEPS
        ),
        "v6_post_step_tolerance_frozen": v6_tolerance_frozen,
        "corrected_counterfactual_changes_only_derived_time_counts": changed_fields
        == ["historical_time_mismatch_count", "step_contract_failure_count"],
        "corrected_counterfactual_gate_passes": (
            corrected_gate.get("passed") is True
            and all(corrected_gate.get("checks", {}).values())
        ),
        "frozen_sources_unchanged": source_count > 0 and drifted_sources == [],
        "zero_updates_and_no_data_opening": (
            summary.get("actor_updates") == 0
            and summary.get("critic_updates") == 0
            and summary.get("ppo_updates") == 0
            and summary.get("protected_trials_opened") == []
            and summary.get("reserve_trials_opened") == []
        ),
    }
    passed = all(checks.values())
    if not passed:
        failed = [name for name, value in checks.items() if value is not True]
        raise V10TimeAdjudicationError(f"time adjudication failed: {failed}")
    return {
        "schema_version": SCHEMA_VERSION,
        "status": STATUS,
        "passed": True,
        "revision": "2026-08-07",
        "scope": "OFFLINE_TIME_ALIGNMENT_ONLY_NO_RERUN",
        "checks": checks,
        "diagnostics": row_audit,
        "historical_time_tolerance_s": HISTORICAL_TIME_TOLERANCE_S,
        "policy_dt_s": POLICY_DT_S,
        "original_protocol": {
            "status": ledger["status"],
            "passed": ledger["passed"],
            "preserved_as_fail": True,
            "retry_authorized": False,
        },
        "counterfactual_gate": {
            "status": corrected_gate["status"],
            "passed": corrected_gate["passed"],
            "changed_summary_fields": changed_fields,
            "not_written_back": True,
        },
        "scientific_evidence": {
            "teacher_view_byte_exact_count": contract.EXPECTED_STEPS,
            "teacher_mean_byte_exact_count": contract.EXPECTED_STEPS,
            "teacher_action_byte_exact_count": contract.EXPECTED_STEPS,
            "coherent_teacher_evidence_accepted": True,
        },
        "artifacts": {
            "v10_lock": _record(LOCK),
            "v10_ledger": _record(LEDGER),
            "v10_trace": _record(TRACE),
            "v10_summary": _record(SUMMARY),
            "v10_gate": _record(GATE),
            "v10_failure": _record(FAILURE),
            "v10_contract": _record(VALIDATION_ROOT / "h0_primary_split_v10_coherent_teacher_probe_contract.py"),
            "v10_runner": _record(VALIDATION_ROOT / "run_h0_primary_split_v10_coherent_teacher_probe.py"),
            "v6_runner": _record(V6_RUNNER),
            "adjudicator": _record(Path(__file__)),
            "adjudicator_tests": _record(TEST_SOURCE),
        },
        "rollout_rerun_count": 0,
        "candidate_created": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "next_stage": "V10_COHERENT_TEACHER_EVIDENCE_READY_FOR_FRESH_ADAPTATION_PROTOCOL",
    }


def write_receipt() -> dict[str, Any]:
    payload = build_adjudication()
    _write_exclusive(RECEIPT, payload)
    return payload


def verify_receipt() -> dict[str, Any]:
    observed = _mapping(RECEIPT)
    expected = build_adjudication()
    if _canonical_bytes(observed) != _canonical_bytes(expected):
        raise V10TimeAdjudicationError("adjudication receipt drifted")
    return observed


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--audit", action="store_true")
    mode.add_argument("--write", action="store_true")
    mode.add_argument("--verify", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.write:
            result = write_receipt()
        elif args.verify:
            result = verify_receipt()
        else:
            result = build_adjudication()
    except Exception as exc:
        print(
            f"V10 time adjudication failed closed: {type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        return 2
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
