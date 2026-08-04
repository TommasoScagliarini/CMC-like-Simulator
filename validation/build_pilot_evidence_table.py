"""Build a fail-closed evidence table for the preregistered PPO pilot.

This tool is deliberately offline.  It only consolidates the immutable parent
protocol and selection-interpretation addendum, a completed pilot screen, a
completed cumulative policy-drift audit, and the immutable training history
referenced by those reports.  It never launches training or rollouts, never
reads held-out rollout data, and never selects, copies, or promotes a
checkpoint.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import sys
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence

ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from validation import ppo_pilot_screen as screen
from validation import robust_ppo_gate as gate


SCHEMA_VERSION = 1
TABLE_NAME = "ppo_pilot_candidate_evidence"
SELECTION_ADDENDUM_SCHEMA_VERSION = 1
SELECTION_ADDENDUM_STATUS = "preregistered_selection_interpretation_addendum"
EXPECTED_ALL_LOGICAL_ITERATIONS = tuple(range(2, 52))
EXPECTED_SCREENED_LOGICAL_ITERATIONS = screen.EXPECTED_SCREENED_LOGICAL_ITERATIONS
EXPECTED_CASES = (
    "deterministic_minus020",
    "deterministic_nominal",
    "deterministic_plus020",
    "stochastic_plus020_seed123",
)
EXPECTED_TRACE_ROLES = EXPECTED_CASES
FORBIDDEN_HELD_OUT_SEEDS = frozenset(screen.SEALED_HELD_OUT_SEEDS)
_HELDOUT_SEED_PATH_PATTERN = re.compile(
    r"(?:^|[^0-9])seed[_-]?(126|127|128)(?:[^0-9]|$)",
    re.IGNORECASE,
)


class EvidenceInputError(ValueError):
    """Raised when an input cannot support a fail-closed evidence table."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise EvidenceInputError(message)


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    _require(isinstance(value, Mapping), f"{name} must be a JSON mapping")
    return value


def _sequence(value: Any, name: str) -> list[Any]:
    _require(isinstance(value, list), f"{name} must be a JSON list")
    return value


def _read_json(path: Path, name: str) -> dict[str, Any]:
    value, _ = _read_json_snapshot(path, name)
    return value


def _reject_nonfinite_json(token: str) -> None:
    raise ValueError(f"non-finite JSON number {token!r}")


def _read_json_snapshot(path: Path, name: str) -> tuple[dict[str, Any], str]:
    try:
        raw = path.read_bytes()
        value = json.loads(
            raw.decode("utf-8"),
            parse_constant=_reject_nonfinite_json,
        )
    except (OSError, UnicodeDecodeError, ValueError) as exc:
        raise EvidenceInputError(f"could not read {name} as valid JSON: {path}") from exc
    _require(isinstance(value, dict), f"{name} must contain a JSON mapping: {path}")
    return value, hashlib.sha256(raw).hexdigest()


def _finite(value: Any, name: str, *, minimum: float | None = None) -> float:
    _require(not isinstance(value, bool), f"{name} must be a finite number")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise EvidenceInputError(f"{name} must be a finite number") from exc
    _require(math.isfinite(number), f"{name} must be a finite number")
    if minimum is not None:
        _require(number >= minimum, f"{name} must be >= {minimum}")
    return number


def _integer_number(value: Any, name: str, *, minimum: int = 0) -> int:
    number = _finite(value, name, minimum=float(minimum))
    _require(number.is_integer(), f"{name} must be an integer-valued number")
    return int(number)


def _same_path(actual: Any, expected: Path) -> bool:
    return isinstance(actual, (str, os.PathLike)) and gate._same_path(actual, expected)


def _path_within(path: Path, root: Path) -> bool:
    try:
        path.resolve(strict=False).relative_to(root.resolve(strict=False))
    except (OSError, RuntimeError, ValueError):
        return False
    return True


def _sha256_file(path: Path, name: str) -> str:
    _require(path.is_file(), f"{name} does not exist or is not a regular file: {path}")
    try:
        return gate._sha256(path)
    except OSError as exc:
        raise EvidenceInputError(f"could not hash {name}: {path}") from exc


def _is_sha256(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def _unique_by_integer(
    rows: Sequence[Any],
    *,
    key: str,
    expected: Sequence[int],
    name: str,
) -> dict[int, Mapping[str, Any]]:
    indexed: dict[int, Mapping[str, Any]] = {}
    order: list[int] = []
    for position, raw_row in enumerate(rows):
        row = _mapping(raw_row, f"{name}[{position}]")
        value = row.get(key)
        _require(type(value) is int, f"{name}[{position}].{key} must be an integer")
        _require(value not in indexed, f"{name} contains duplicate {key}={value}")
        indexed[value] = row
        order.append(value)
    _require(order == list(expected), f"{name} must contain exactly {list(expected)} in order")
    return indexed


def _check_map(classification: Mapping[str, Any], case_name: str) -> dict[str, Mapping[str, Any]]:
    checks = _sequence(classification.get("checks"), f"{case_name}.classification.checks")
    indexed: dict[str, Mapping[str, Any]] = {}
    for position, raw_check in enumerate(checks):
        check = _mapping(raw_check, f"{case_name}.classification.checks[{position}]")
        check_name = check.get("name")
        _require(
            isinstance(check_name, str) and check_name,
            f"{case_name}.classification.checks[{position}].name is invalid",
        )
        _require(check_name not in indexed, f"{case_name} has duplicate check {check_name!r}")
        _require(check.get("status") in {"PASS", "FAIL"}, f"{case_name}.{check_name} has invalid status")
        indexed[check_name] = check
    return indexed


def _command_option(command: Any, option: str, case_name: str) -> str:
    values = _sequence(command, f"{case_name}.invocation.command")
    positions = [index for index, value in enumerate(values) if value == option]
    _require(len(positions) == 1, f"{case_name} command must contain {option} exactly once")
    position = positions[0]
    _require(position + 1 < len(values), f"{case_name} command has no value after {option}")
    value = values[position + 1]
    _require(isinstance(value, str), f"{case_name} command value after {option} must be text")
    return value


def _validate_protocol(protocol: Mapping[str, Any]) -> Mapping[str, Any]:
    report, context = screen._protocol_contract(protocol)
    failed = report.get("failed_checks")
    _require(
        report.get("status") == "PASS" and context is not None,
        f"protocol contract failed: {failed}",
    )
    return context


def _recorded_path(value: Any, name: str) -> Path:
    _require(isinstance(value, str) and value.strip(), f"{name} must be a non-empty path")
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = ROOT_DIR / path
    return path.resolve(strict=False)


def _validate_selection_addendum(
    *,
    addendum: Mapping[str, Any],
    protocol: Mapping[str, Any],
    protocol_path: Path,
    protocol_sha256: str,
    drift_report_path: Path,
) -> Mapping[str, Any]:
    _require(
        addendum.get("schema_version") == SELECTION_ADDENDUM_SCHEMA_VERSION,
        "unsupported selection-addendum schema",
    )
    _require(
        addendum.get("status") == SELECTION_ADDENDUM_STATUS,
        "selection addendum has the wrong status",
    )
    _require(addendum.get("immutable") is True, "selection addendum is not immutable")
    created_at = addendum.get("created_at")
    _require(isinstance(created_at, str) and created_at, "selection addendum has no creation timestamp")
    try:
        created_timestamp = datetime.fromisoformat(created_at)
    except ValueError as exc:
        raise EvidenceInputError("selection addendum creation timestamp is invalid") from exc
    _require(
        created_timestamp.tzinfo is not None
        and created_timestamp.utcoffset() is not None,
        "selection addendum creation timestamp must include a UTC offset",
    )

    parent = _mapping(addendum.get("parent_protocol"), "selection_addendum.parent_protocol")
    _require(
        _recorded_path(parent.get("path"), "selection addendum parent protocol")
        == protocol_path,
        "selection addendum parent-protocol path mismatch",
    )
    _require(
        parent.get("sha256") == protocol_sha256,
        "selection addendum parent-protocol SHA-256 mismatch",
    )
    _require(
        addendum.get("scope")
        == "clarify_only_the_existing_cumulative_empirical_kl_tie_breaker",
        "selection addendum changes or exceeds its allowed scope",
    )

    interpretation = _mapping(
        addendum.get("selection_interpretation"),
        "selection_addendum.selection_interpretation",
    )
    expected_interpretation = {
        "cumulative_means": "direct_H0_to_current_milestone_drift_not_the_sum_of_incremental_update_KLs",
        "source_report": "policy_drift_from_h0_milestones.json",
        "source_field": "milestones[].kl_reference_to_candidate_mean",
        "statistic": "fixed_observation_aggregate.empirical_kl_reference_to_candidate_mean",
        "development_trace_set": "the_four_preregistered_fixed_H0_traces",
        "ordering": "ascending",
        "candidate_scope": "only_milestones_eligible_under_every_development_gate",
        "empirical_kl_max_role": "diagnostic_only_not_a_selection_tie_breaker",
    }
    _require(
        dict(interpretation) == expected_interpretation,
        "selection-addendum field/statistic/ordering contract mismatch",
    )
    _require(
        drift_report_path.name == interpretation["source_report"],
        "policy-drift input filename differs from the addendum source report",
    )

    unchanged = _mapping(
        addendum.get("unchanged_contract"),
        "selection_addendum.unchanged_contract",
    )
    expected_unchanged = {
        "primary_order": "minimum worst condition-matched reserve ratio versus H0",
        "first_tie_breaker": "minimum worst penetration ratio versus H0",
        "second_tie_breaker": "minimum cumulative empirical KL versus H0",
        "third_tie_breaker": "earlier pilot update",
        "automatic_selection_or_promotion": False,
        "held_out_seeds": list(screen.SEALED_HELD_OUT_SEEDS),
        "held_out_status": "sealed",
    }
    _require(
        dict(unchanged) == expected_unchanged,
        "selection addendum changes the preregistered selection or held-out contract",
    )
    selection = _mapping(protocol.get("candidate_selection"), "protocol.candidate_selection")
    _require(
        selection.get("primary_order") == unchanged["primary_order"],
        "selection addendum primary order differs from its parent protocol",
    )
    _require(
        selection.get("tie_breakers")
        == [
            unchanged["first_tie_breaker"],
            unchanged["second_tie_breaker"],
            unchanged["third_tie_breaker"],
        ],
        "selection addendum tie-breakers differ from its parent protocol",
    )

    timing = _mapping(
        addendum.get("timing_attestation"),
        "selection_addendum.timing_attestation",
    )
    expected_timing = {
        "training_status": "active",
        "latest_completed_logical_iteration": 9,
        "completed_new_actor_updates": 8,
        "development_screening_started": False,
        "held_out_opened": False,
    }
    _require(
        dict(timing) == expected_timing,
        "selection-addendum timing or held-out sealing attestation mismatch",
    )
    return interpretation


def _validate_training_sources(
    *,
    run_dir: Path,
    screen_report: Mapping[str, Any],
) -> tuple[Path, Path, list[dict[str, Any]], str, str]:
    run_validation = _mapping(screen_report.get("run_validation"), "screen.run_validation")
    _require(run_validation.get("status") == "PASS", "screen run validation did not pass")
    _require(run_validation.get("failed_checks") == [], "screen run validation contains failures")

    expected_summary = run_dir / "summary.json"
    expected_history = run_dir / "train_iterations.jsonl"
    _require(_same_path(run_validation.get("run_dir"), run_dir), "screen run directory differs from protocol")
    _require(
        _same_path(run_validation.get("summary_path"), expected_summary),
        "screen summary path differs from protocol run",
    )
    _require(
        _same_path(run_validation.get("train_iterations_path"), expected_history),
        "screen training-history path differs from protocol run",
    )

    summary, summary_digest = _read_json_snapshot(expected_summary, "training summary")
    _require(summary.get("ok") is True, "training summary is not successful")
    _require(summary.get("stop_reason") == "completed", "training summary is not complete")
    _require(summary.get("interrupted") is False, "training summary reports interruption")
    _require(summary.get("timed_out") is False, "training summary reports timeout")
    _require(summary.get("error") is None, "training summary reports an error")
    _require(summary.get("skipped_iterations") == [], "training summary reports skipped iterations")
    _require(summary.get("restart_count") == 0, "training summary reports supervisor restarts")
    _require(summary.get("crash_restart_count") == 0, "training summary reports crash restarts")

    try:
        history_digest_before = _sha256_file(expected_history, "training history")
        history = gate._jsonl_objects(expected_history)
    except (OSError, ValueError) as exc:
        raise EvidenceInputError(f"could not read complete training history: {expected_history}") from exc
    history_index = _unique_by_integer(
        history,
        key="iteration",
        expected=EXPECTED_ALL_LOGICAL_ITERATIONS,
        name="training history",
    )
    history_rows = [dict(history_index[iteration]) for iteration in EXPECTED_ALL_LOGICAL_ITERATIONS]
    _require(
        summary.get("history") == history_rows,
        "training summary history is not byte-content-equivalent to train_iterations.jsonl records",
    )
    reported_audits = _sequence(
        run_validation.get("training_iteration_audits"),
        "screen.run_validation.training_iteration_audits",
    )
    _require(
        len(reported_audits) == len(EXPECTED_ALL_LOGICAL_ITERATIONS),
        "screen run validation has an incomplete training-iteration audit set",
    )
    fresh_audits = [
        gate.classify_training_iterations(
            expected_history,
            start_offsets_s=gate.DEFAULT_START_OFFSETS_S,
            expected_training_iteration=iteration,
        )
        for iteration in EXPECTED_ALL_LOGICAL_ITERATIONS
    ]
    _require(
        reported_audits == fresh_audits,
        "screen training-iteration audits changed or are inconsistent",
    )
    _require(
        all(audit.get("status") == "PASS" for audit in fresh_audits),
        "one or more training iterations fail the update contract",
    )
    history_digest_after = _sha256_file(expected_history, "training history")
    _require(
        history_digest_after == history_digest_before,
        "training history changed while the evidence table was being built",
    )
    return (
        expected_summary,
        expected_history,
        history_rows,
        summary_digest,
        history_digest_before,
    )


def _validate_drift_report(
    *,
    report: Mapping[str, Any],
    protocol: Mapping[str, Any],
    run_dir: Path,
) -> dict[int, Mapping[str, Any]]:
    _require(report.get("schema_version") == 1, "unsupported policy-drift report schema")
    _require(report.get("audit") == "cumulative_policy_drift_from_h0", "wrong policy-drift audit type")
    _require(_same_path(report.get("training_run"), run_dir), "policy-drift run differs from protocol")

    development = _mapping(protocol.get("development_gate"), "protocol.development_gate")
    source = _mapping(protocol.get("source"), "protocol.source")
    reference = Path(str(development.get("reference_checkpoint"))).expanduser()
    if not reference.is_absolute():
        reference = ROOT_DIR / reference
    reference = reference.resolve(strict=False)
    _require(
        _same_path(report.get("reference_h0_rl_module"), reference),
        "policy-drift H0 reference differs from protocol",
    )
    _require(
        report.get("reference_h0_actor_digest") == source.get("actor_digest"),
        "policy-drift H0 actor digest differs from protocol",
    )
    _require(
        report.get("heldout_seeds_excluded") == list(screen.SEALED_HELD_OUT_SEEDS),
        "policy-drift report does not explicitly exclude the sealed seeds",
    )

    traces = _sequence(report.get("development_traces"), "policy-drift development_traces")
    _require(len(traces) == len(EXPECTED_TRACE_ROLES), "policy-drift audit must use exactly four development traces")
    trace_paths: list[Path] = []
    offsets = tuple(float(value) for value in _mapping(protocol.get("sampling"), "protocol.sampling")["start_offsets_s"])
    expected_trace_offsets = (offsets[0], offsets[1], offsets[2], offsets[2])
    expected_modes = ("deterministic", "deterministic", "deterministic", "stochastic")
    for index, raw_trace in enumerate(traces):
        trace = _mapping(raw_trace, f"policy-drift development_traces[{index}]")
        role = EXPECTED_TRACE_ROLES[index]
        _require(trace.get("protocol_role") == role, f"unexpected policy-drift trace role at index {index}")
        _require(trace.get("action_selection") == expected_modes[index], f"wrong action mode for trace {role}")
        offset = _finite(trace.get("episode_start_offset_s"), f"trace {role} offset")
        _require(math.isclose(offset, expected_trace_offsets[index], rel_tol=0.0, abs_tol=1.0e-12), f"wrong start offset for trace {role}")
        seed = trace.get("action_seed")
        _require(type(seed) is int, f"trace {role} must record an integer development seed")
        _require(seed not in FORBIDDEN_HELD_OUT_SEEDS, f"trace {role} uses a sealed held-out seed")
        if role == "stochastic_plus020_seed123":
            _require(seed == screen.EXPECTED_DEVELOPMENT_SEED, "stochastic drift trace must use development seed 123")
        trace_path = Path(str(trace.get("path"))).expanduser().resolve(strict=False)
        _require(
            _HELDOUT_SEED_PATH_PATTERN.search(trace_path.as_posix()) is None,
            f"trace {role} path names a held-out seed",
        )
        _require(_is_sha256(trace.get("sha256")), f"trace {role} has no valid SHA-256")
        trace_paths.append(trace_path)

    raw_rows = _sequence(report.get("milestones"), "policy-drift milestones")
    rows = _unique_by_integer(
        raw_rows,
        key="logical_iteration",
        expected=EXPECTED_ALL_LOGICAL_ITERATIONS,
        name="policy-drift milestones",
    )
    _require(report.get("milestone_count") == len(EXPECTED_ALL_LOGICAL_ITERATIONS), "wrong policy-drift milestone count")

    failed: list[int] = []
    for iteration in EXPECTED_ALL_LOGICAL_ITERATIONS:
        row = rows[iteration]
        expected_root = run_dir / f"{screen.MILESTONE_PREFIX}{iteration:06d}"
        expected_module = expected_root / "rl_module_last"
        _require(_same_path(row.get("milestone"), expected_root), f"drift milestone path mismatch at iteration {iteration}")
        _require(_same_path(row.get("rl_module"), expected_module), f"drift RLModule path mismatch at iteration {iteration}")
        _require(row.get("reference_actor_digest") == source.get("actor_digest"), f"drift reference digest mismatch at iteration {iteration}")
        _require(_is_sha256(row.get("candidate_actor_digest")), f"invalid candidate actor digest at iteration {iteration}")
        _finite(row.get("action_mean_rmse"), f"iteration {iteration} action_mean_rmse", minimum=0.0)
        _finite(row.get("action_mean_abs_max"), f"iteration {iteration} action_mean_abs_max", minimum=0.0)
        _finite(row.get("kl_reference_to_candidate_mean"), f"iteration {iteration} empirical KL mean", minimum=-1.0e-7)
        _finite(row.get("kl_reference_to_candidate_max"), f"iteration {iteration} empirical KL max", minimum=-1.0e-7)
        _require(row.get("metrics_finite") is True, f"drift metrics are incomplete at iteration {iteration}")
        _require(type(row.get("logstd_bit_exact")) is bool, f"missing logstd audit at iteration {iteration}")
        expected_status = "PASS" if row.get("logstd_bit_exact") is True else "FAIL"
        _require(row.get("status") == expected_status, f"inconsistent drift status at iteration {iteration}")

        per_trace = _sequence(row.get("per_development_trace"), f"iteration {iteration} per_development_trace")
        _require(len(per_trace) == len(trace_paths), f"iteration {iteration} has incomplete per-trace drift evidence")
        for trace_index, raw_metrics in enumerate(per_trace):
            metrics = _mapping(raw_metrics, f"iteration {iteration} trace {trace_index}")
            _require(_same_path(metrics.get("trace"), trace_paths[trace_index]), f"iteration {iteration} drift trace set mismatch")
        if row.get("status") != "PASS":
            failed.append(iteration)

    _require(report.get("failed_logical_iterations") == failed, "policy-drift failed-iteration list is inconsistent")
    _require(report.get("ok") is (not failed), "policy-drift top-level status is inconsistent")
    return rows


def _validate_case(
    *,
    raw_case: Any,
    name: str,
    logical_iteration: int,
    expected_mode: str,
    expected_offset: float,
    expected_checkpoint: Path,
    screen_root: Path,
    penetration_limit_m: float,
) -> dict[str, Any]:
    case = _mapping(raw_case, f"iteration {logical_iteration} case {name}")
    _require(case.get("status") in {"PASS", "FAIL"}, f"case {name} has invalid status")
    _require(case.get("operational_failure") is False, f"case {name} has an operational failure")

    invocation = _mapping(case.get("invocation"), f"case {name}.invocation")
    _require(invocation.get("name") == name, f"case {name} invocation name mismatch")
    _require(invocation.get("status") == "PASS", f"case {name} rollout process did not complete")
    _require(invocation.get("returncode") == 0, f"case {name} rollout process returned an error")
    seed = _integer_number(_command_option(invocation.get("command"), "--seed", name), f"case {name} command seed")
    _require(seed == screen.EXPECTED_DEVELOPMENT_SEED, f"case {name} did not use development seed 123")
    _require(seed not in FORBIDDEN_HELD_OUT_SEEDS, f"case {name} used a held-out seed")
    _require(_command_option(invocation.get("command"), "--action-selection", name) == expected_mode, f"case {name} command action mode mismatch")
    command_offset = _finite(_command_option(invocation.get("command"), "--episode-start-offset-s", name), f"case {name} command offset")
    _require(math.isclose(command_offset, expected_offset, rel_tol=0.0, abs_tol=1.0e-12), f"case {name} command offset mismatch")
    _require(_same_path(_command_option(invocation.get("command"), "--checkpoint", name), expected_checkpoint), f"case {name} command checkpoint mismatch")

    expected_case_dir = screen_root / f"{screen.MILESTONE_PREFIX}{logical_iteration:06d}" / "rollouts" / name
    _require(_same_path(invocation.get("output_dir"), expected_case_dir), f"case {name} invocation output path mismatch")
    _require(_same_path(_command_option(invocation.get("command"), "--output-dir", name), expected_case_dir), f"case {name} command output path mismatch")

    classification = _mapping(case.get("classification"), f"case {name}.classification")
    _require(classification.get("name") == name, f"case {name} classification name mismatch")
    _require(classification.get("status") == case.get("status"), f"case {name} status mismatch")
    expected = _mapping(classification.get("expected"), f"case {name}.classification.expected")
    _require(expected.get("action_selection") == expected_mode, f"case {name} expected action mode mismatch")
    _require(expected.get("seed") == screen.EXPECTED_DEVELOPMENT_SEED, f"case {name} classification seed mismatch")
    classified_offset = _finite(expected.get("offset_s"), f"case {name} classified offset")
    _require(math.isclose(classified_offset, expected_offset, rel_tol=0.0, abs_tol=1.0e-12), f"case {name} classified offset mismatch")

    summary_path = expected_case_dir / "rollout_summary.json"
    _require(_same_path(classification.get("summary_path"), summary_path), f"case {name} summary path mismatch")
    summary_digest = _sha256_file(summary_path, f"case {name} rollout summary")
    _require(classification.get("summary_sha256") == summary_digest, f"case {name} rollout summary changed after screening")

    checks = _check_map(classification, name)
    _require("steps" in checks, f"case {name} has no steps check")
    _require("phase_valid_cycle_count" in checks, f"case {name} has no cycle-count check")
    steps = _integer_number(checks["steps"].get("actual"), f"case {name} steps")
    cycles = _integer_number(checks["phase_valid_cycle_count"].get("actual"), f"case {name} valid cycles")

    observed = _mapping(classification.get("observed"), f"case {name}.classification.observed")
    reserve_nm = _finite(observed.get("reserve_norm_max_nm"), f"case {name} reserve", minimum=0.0)
    penetration_m = _finite(observed.get("grf_penetration_max_m"), f"case {name} penetration", minimum=0.0)
    reserve_ratio = _finite(observed.get("reserve_ratio_vs_h0"), f"case {name} reserve ratio", minimum=0.0)
    penetration_ratio = _finite(observed.get("penetration_ratio_vs_h0"), f"case {name} penetration ratio", minimum=0.0)

    reserve_contract = _mapping(classification.get("reserve_contract"), f"case {name}.reserve_contract")
    reference_reserve = _finite(reserve_contract.get("reference_cap_nm"), f"case {name} H0 reserve", minimum=0.0)
    numerical_tolerance = _finite(reserve_contract.get("numerical_tolerance_nm"), f"case {name} reserve tolerance", minimum=0.0)
    reserve_gate = _finite(reserve_contract.get("maximum_inclusive_nm"), f"case {name} reserve gate", minimum=0.0)
    _require(math.isclose(reserve_gate, reference_reserve + numerical_tolerance, rel_tol=0.0, abs_tol=1.0e-9), f"case {name} reserve gate is inconsistent")
    if reference_reserve > 0.0:
        _require(math.isclose(reserve_ratio, reserve_nm / reference_reserve, rel_tol=1.0e-12, abs_tol=1.0e-12), f"case {name} reserve ratio is inconsistent")

    failed_checks = _sequence(classification.get("failed_checks"), f"case {name}.failed_checks")
    expected_case_status = "PASS" if not failed_checks else "FAIL"
    _require(classification.get("status") == expected_case_status, f"case {name} failed-check list is inconsistent")

    return {
        "condition": name,
        "execution": "RUN",
        "gate_status": case.get("status"),
        "failed_checks": list(failed_checks),
        "steps": steps,
        "valid_cycles": cycles,
        "reserve": {
            "observed_nm": reserve_nm,
            "reference_h0_nm": reference_reserve,
            "ratio_vs_h0": reserve_ratio,
            "margin_vs_h0_nm": reference_reserve - reserve_nm,
            "numerical_tolerance_nm": numerical_tolerance,
            "gate_upper_bound_nm": reserve_gate,
            "margin_to_gate_nm": reserve_gate - reserve_nm,
        },
        "penetration": {
            "observed_m": penetration_m,
            "ratio_vs_h0": penetration_ratio,
            "strict_gate_m": penetration_limit_m,
            "margin_to_strict_gate_m": penetration_limit_m - penetration_m,
        },
        "source_summary": {
            "path": gate._portable_path(summary_path),
            "sha256": summary_digest,
        },
    }


def _not_run_case(name: str, reason: str) -> dict[str, Any]:
    return {
        "condition": name,
        "execution": "NOT_RUN",
        "reason": reason,
        "gate_status": "NOT_EVALUATED",
        "failed_checks": [],
        "steps": None,
        "valid_cycles": None,
        "reserve": None,
        "penetration": None,
        "source_summary": None,
    }


def _validate_screen_and_build_rows(
    *,
    report: Mapping[str, Any],
    report_path: Path,
    protocol: Mapping[str, Any],
    protocol_path: Path,
    protocol_sha256: str,
    run_dir: Path,
    history_path: Path,
    history_rows: Sequence[Mapping[str, Any]],
    drift_rows: Mapping[int, Mapping[str, Any]],
) -> list[dict[str, Any]]:
    _require(report.get("schema_version") == screen.REPORT_SCHEMA_VERSION, "unsupported pilot-screen schema")
    _require(report.get("screen") == "ppo_pilot_screen", "wrong pilot-screen report type")
    _require(report.get("screening_completed") is True, "pilot screening is incomplete")
    _require(report.get("operational_failures") == [], "pilot screen contains operational failures")
    _require(report.get("screened_logical_iterations") == list(EXPECTED_SCREENED_LOGICAL_ITERATIONS), "pilot screen milestone set differs from preregistration")
    _require(_same_path(report.get("output"), report_path), "pilot-screen report does not identify its own path")

    protocol_record = _mapping(report.get("protocol"), "screen.protocol")
    _require(_same_path(protocol_record.get("path"), protocol_path), "pilot screen used a different protocol")
    _require(protocol_record.get("sha256") == protocol_sha256, "pilot protocol changed after screening")
    protocol_contract = _mapping(protocol_record.get("contract"), "screen.protocol.contract")
    _require(protocol_contract.get("status") == "PASS", "pilot-screen protocol contract failed")
    _require(protocol_contract.get("failed_checks") == [], "pilot-screen protocol contract contains failures")

    reserve_reference = _mapping(report.get("reserve_reference"), "screen.reserve_reference")
    _require(reserve_reference.get("status") == "PASS", "pilot-screen reserve reference failed")
    reference_checkpoint = _mapping(
        reserve_reference.get("checkpoint"), "screen.reserve_reference.checkpoint"
    )
    _require(
        reference_checkpoint.get("status") == "PASS",
        "pilot-screen reserve-reference checkpoint failed",
    )
    reference_rollouts = _sequence(
        reserve_reference.get("rollouts"), "screen.reserve_reference.rollouts"
    )
    _require(
        len(reference_rollouts) == len(EXPECTED_CASES)
        and all(
            isinstance(item, Mapping)
            and item.get("status") == "PASS"
            and item.get("failed_checks") == []
            for item in reference_rollouts
        ),
        "pilot-screen reserve-reference rollouts contain failures",
    )
    restart_audit = _mapping(report.get("restart_audit"), "screen.restart_audit")
    _require(
        restart_audit.get("status") == "PASS",
        "pilot-screen restart audit did not pass",
    )
    _require(
        restart_audit.get("failed_checks") == [],
        "pilot-screen restart audit contains failures",
    )
    held_out = _mapping(report.get("held_out"), "screen.held_out")
    _require(held_out.get("status") == "SEALED", "held-out gate is not sealed")
    _require(held_out.get("opened") is False, "held-out gate was opened")
    _require(held_out.get("seeds_used") == [], "pilot screen used held-out seeds")
    _require(report.get("fallback_attempted") is False, "pilot screen attempted a fallback")
    _require(report.get("checkpoint_selected") is None, "pilot screen selected a checkpoint")
    _require(report.get("checkpoint_promoted") is False, "pilot screen promoted a checkpoint")
    _require(report.get("checkpoint_copied") is False, "pilot screen copied a checkpoint")

    milestones = _sequence(report.get("milestones"), "screen.milestones")
    indexed = _unique_by_integer(
        milestones,
        key="logical_iteration",
        expected=EXPECTED_SCREENED_LOGICAL_ITERATIONS,
        name="screen.milestones",
    )
    history_index = {int(row["iteration"]): row for row in history_rows}
    sampling = _mapping(protocol.get("sampling"), "protocol.sampling")
    offsets = tuple(float(value) for value in sampling["start_offsets_s"])
    penetration_limit = _finite(
        _mapping(protocol.get("development_gate"), "protocol.development_gate").get("penetration_limit_m_strict"),
        "protocol penetration limit",
        minimum=0.0,
    )
    screen_root = report_path.parent

    output_rows: list[dict[str, Any]] = []
    derived_eligible: list[int] = []
    derived_rejected: list[int] = []
    for logical_iteration in EXPECTED_SCREENED_LOGICAL_ITERATIONS:
        milestone = indexed[logical_iteration]
        _require(milestone.get("pilot_update_index") == logical_iteration - 1, f"wrong pilot update index at iteration {logical_iteration}")
        checkpoint = run_dir / f"{screen.MILESTONE_PREFIX}{logical_iteration:06d}" / "rl_module_last"
        checkpoint_report = _mapping(milestone.get("candidate_checkpoint"), f"iteration {logical_iteration} candidate checkpoint")
        _require(checkpoint_report.get("status") == "PASS", f"iteration {logical_iteration} checkpoint artifact failed validation")
        _require(_same_path(checkpoint_report.get("path"), checkpoint), f"iteration {logical_iteration} checkpoint path mismatch")

        training = _mapping(milestone.get("training"), f"iteration {logical_iteration} training audit")
        _require(_same_path(training.get("path"), history_path), f"iteration {logical_iteration} training history path mismatch")
        fresh_training = gate.classify_training_iterations(
            history_path,
            start_offsets_s=offsets,
            expected_training_iteration=logical_iteration,
        )
        _require(training == fresh_training, f"training audit changed or is inconsistent at iteration {logical_iteration}")
        training_status = training.get("status")
        _require(training_status in {"PASS", "FAIL"}, f"invalid training audit status at iteration {logical_iteration}")
        training_record = history_index[logical_iteration]
        training_return = _finite(training_record.get("episode_return_mean"), f"iteration {logical_iteration} training return")
        training_length = _finite(training_record.get("episode_len_mean"), f"iteration {logical_iteration} training episode length", minimum=0.0)

        critical = _mapping(milestone.get("critical"), f"iteration {logical_iteration} critical result")
        deterministic = _mapping(milestone.get("deterministic"), f"iteration {logical_iteration} deterministic result")
        cases_by_name: dict[str, dict[str, Any]] = {}
        if training_status != "PASS":
            _require(critical.get("status") == "SKIPPED", f"iteration {logical_iteration} critical case should be skipped")
            _require(critical.get("reason") == "training_iteration_audit_failed", f"iteration {logical_iteration} critical skip reason mismatch")
            _require(deterministic.get("status") == "SKIPPED", f"iteration {logical_iteration} deterministic cases should be skipped")
            _require(deterministic.get("reason") == "training_iteration_audit_failed", f"iteration {logical_iteration} deterministic skip reason mismatch")
            _require(deterministic.get("cases") == [], f"iteration {logical_iteration} has unexpected deterministic cases")
            for case_name in EXPECTED_CASES:
                cases_by_name[case_name] = _not_run_case(case_name, "training_iteration_audit_failed")
        else:
            critical_case = _validate_case(
                raw_case=critical,
                name=EXPECTED_CASES[3],
                logical_iteration=logical_iteration,
                expected_mode="stochastic",
                expected_offset=offsets[2],
                expected_checkpoint=checkpoint,
                screen_root=screen_root,
                penetration_limit_m=penetration_limit,
            )
            cases_by_name[EXPECTED_CASES[3]] = critical_case
            if critical_case["gate_status"] != "PASS":
                _require(deterministic.get("status") == "SKIPPED", f"iteration {logical_iteration} deterministic cases should be fail-fast skipped")
                _require(deterministic.get("reason") == "critical_stochastic_plus020_failed", f"iteration {logical_iteration} fail-fast reason mismatch")
                _require(deterministic.get("cases") == [], f"iteration {logical_iteration} has rollouts after critical failure")
                for case_name in EXPECTED_CASES[:3]:
                    cases_by_name[case_name] = _not_run_case(case_name, "critical_stochastic_plus020_failed")
            else:
                deterministic_cases = _sequence(deterministic.get("cases"), f"iteration {logical_iteration} deterministic cases")
                _require(len(deterministic_cases) == 3, f"iteration {logical_iteration} must contain three deterministic cases")
                expected_deterministic_status = "PASS"
                for case_index, case_name in enumerate(EXPECTED_CASES[:3]):
                    case_evidence = _validate_case(
                        raw_case=deterministic_cases[case_index],
                        name=case_name,
                        logical_iteration=logical_iteration,
                        expected_mode="deterministic",
                        expected_offset=offsets[case_index],
                        expected_checkpoint=checkpoint,
                        screen_root=screen_root,
                        penetration_limit_m=penetration_limit,
                    )
                    cases_by_name[case_name] = case_evidence
                    if case_evidence["gate_status"] != "PASS":
                        expected_deterministic_status = "FAIL"
                _require(deterministic.get("status") == expected_deterministic_status, f"iteration {logical_iteration} deterministic aggregate status mismatch")

        ordered_cases = [cases_by_name[name] for name in EXPECTED_CASES]
        run_cases = [case for case in ordered_cases if case["execution"] == "RUN"]
        all_four_observed = len(run_cases) == len(EXPECTED_CASES)
        all_four_pass = all_four_observed and all(case["gate_status"] == "PASS" for case in run_cases)
        any_gate_failure = any(case["gate_status"] == "FAIL" for case in run_cases)
        if all_four_pass:
            four_case_status = "PASS"
        elif any_gate_failure:
            four_case_status = "FAIL"
        else:
            four_case_status = "NOT_FULLY_EVALUATED"

        expected_eligibility = "ELIGIBLE" if training_status == "PASS" and all_four_pass else "REJECTED"
        _require(milestone.get("status") == expected_eligibility, f"iteration {logical_iteration} eligibility is inconsistent")
        _require(milestone.get("operational_failure") is False, f"iteration {logical_iteration} reports an operational failure")
        (derived_eligible if expected_eligibility == "ELIGIBLE" else derived_rejected).append(logical_iteration)

        drift = drift_rows[logical_iteration]
        output_rows.append(
            {
                "logical_iteration": logical_iteration,
                "pilot_update_index": logical_iteration - 1,
                "screen_eligibility": expected_eligibility,
                "training": {
                    "audit_status": training_status,
                    "failed_checks": list(training.get("failed_checks", [])),
                    "episode_return_mean": training_return,
                    "episode_len_mean": training_length,
                },
                "robustness": {
                    "four_case_status": four_case_status,
                    "all_four_observed": all_four_observed,
                    "all_four_pass": all_four_pass,
                    "cases": ordered_cases,
                    "worst_observed": {
                        "reserve_ratio_vs_h0_max": max(case["reserve"]["ratio_vs_h0"] for case in run_cases) if run_cases else None,
                        "reserve_margin_to_gate_nm_min": min(case["reserve"]["margin_to_gate_nm"] for case in run_cases) if run_cases else None,
                        "penetration_ratio_vs_h0_max": max(case["penetration"]["ratio_vs_h0"] for case in run_cases) if run_cases else None,
                        "penetration_margin_to_strict_gate_m_min": min(case["penetration"]["margin_to_strict_gate_m"] for case in run_cases) if run_cases else None,
                    },
                },
                "policy_drift_from_h0": {
                    "audit_status": drift.get("status"),
                    "logstd_bit_exact": drift.get("logstd_bit_exact"),
                    "action_mean_rmse": _finite(drift.get("action_mean_rmse"), f"iteration {logical_iteration} action mean RMSE", minimum=0.0),
                    "action_mean_abs_max": _finite(drift.get("action_mean_abs_max"), f"iteration {logical_iteration} action mean max", minimum=0.0),
                    "empirical_kl_reference_to_candidate_mean": _finite(drift.get("kl_reference_to_candidate_mean"), f"iteration {logical_iteration} empirical KL mean", minimum=-1.0e-7),
                    "empirical_kl_reference_to_candidate_max": _finite(drift.get("kl_reference_to_candidate_max"), f"iteration {logical_iteration} empirical KL max", minimum=-1.0e-7),
                    "candidate_actor_digest": drift.get("candidate_actor_digest"),
                },
                "preregistered_ordering_keys": {
                    "eligible_for_ordering": expected_eligibility == "ELIGIBLE",
                    "primary_worst_condition_matched_reserve_ratio_vs_h0": (
                        max(case["reserve"]["ratio_vs_h0"] for case in run_cases)
                        if expected_eligibility == "ELIGIBLE"
                        else None
                    ),
                    "tie_breaker_worst_penetration_ratio_vs_h0": (
                        max(case["penetration"]["ratio_vs_h0"] for case in run_cases)
                        if expected_eligibility == "ELIGIBLE"
                        else None
                    ),
                    "tie_breaker_cumulative_empirical_kl_from_h0_mean": (
                        _finite(
                            drift.get("kl_reference_to_candidate_mean"),
                            f"iteration {logical_iteration} empirical KL ordering key",
                            minimum=-1.0e-7,
                        )
                        if expected_eligibility == "ELIGIBLE"
                        else None
                    ),
                    "tie_breaker_earlier_pilot_update": (
                        logical_iteration - 1
                        if expected_eligibility == "ELIGIBLE"
                        else None
                    ),
                },
            }
        )

    _require(report.get("eligible_logical_iterations") == derived_eligible, "screen eligible-iteration list is inconsistent")
    _require(report.get("rejected_logical_iterations") == derived_rejected, "screen rejected-iteration list is inconsistent")
    expected_ok = bool(derived_eligible)
    _require(report.get("ok") is expected_ok, "pilot-screen top-level ok flag is inconsistent")
    _require(report.get("status") == ("PASS" if expected_ok else "FAIL"), "pilot-screen top-level status is inconsistent")
    return output_rows


def build_evidence_table(
    *,
    protocol_path: str | Path,
    selection_addendum_path: str | Path,
    screen_report_path: str | Path,
    drift_report_path: str | Path,
    output_path: str | Path,
) -> dict[str, Any]:
    """Validate complete inputs and return the eight-row evidence table."""

    protocol_file = Path(protocol_path).expanduser().resolve(strict=False)
    addendum_file = Path(selection_addendum_path).expanduser().resolve(strict=False)
    screen_file = Path(screen_report_path).expanduser().resolve(strict=False)
    drift_file = Path(drift_report_path).expanduser().resolve(strict=False)
    output_file = Path(output_path).expanduser().resolve(strict=False)
    _require(not os.path.lexists(output_file), f"refusing existing output: {output_file}")

    protocol, protocol_sha256 = _read_json_snapshot(protocol_file, "protocol")
    context = _validate_protocol(protocol)
    addendum, addendum_sha256 = _read_json_snapshot(
        addendum_file, "selection addendum"
    )
    selection_interpretation = _validate_selection_addendum(
        addendum=addendum,
        protocol=protocol,
        protocol_path=protocol_file,
        protocol_sha256=protocol_sha256,
        drift_report_path=drift_file,
    )
    screen_report, screen_sha256 = _read_json_snapshot(
        screen_file, "pilot-screen report"
    )
    drift_report, drift_sha256 = _read_json_snapshot(
        drift_file, "policy-drift report"
    )

    run_dir = Path(context["run_dir"])
    _require(
        not _path_within(output_file, run_dir),
        "refusing to write the evidence table inside the immutable training run",
    )
    summary_path, history_path, history_rows, summary_sha256, history_sha256 = _validate_training_sources(
        run_dir=run_dir,
        screen_report=screen_report,
    )
    drift_rows = _validate_drift_report(
        report=drift_report,
        protocol=protocol,
        run_dir=run_dir,
    )
    rows = _validate_screen_and_build_rows(
        report=screen_report,
        report_path=screen_file,
        protocol=protocol,
        protocol_path=protocol_file,
        protocol_sha256=protocol_sha256,
        run_dir=run_dir,
        history_path=history_path,
        history_rows=history_rows,
        drift_rows=drift_rows,
    )
    selection = _mapping(protocol.get("candidate_selection"), "protocol.candidate_selection")

    return {
        "schema_version": SCHEMA_VERSION,
        "table": TABLE_NAME,
        "status": "COMPLETE",
        "ok": True,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "output": gate._portable_path(output_file),
        "inputs": {
            "protocol": {"path": gate._portable_path(protocol_file), "sha256": protocol_sha256},
            "selection_addendum": {
                "path": gate._portable_path(addendum_file),
                "sha256": addendum_sha256,
            },
            "pilot_screen": {"path": gate._portable_path(screen_file), "sha256": screen_sha256},
            "policy_drift": {"path": gate._portable_path(drift_file), "sha256": drift_sha256},
            "training_summary": {"path": gate._portable_path(summary_path), "sha256": summary_sha256},
            "training_history": {"path": gate._portable_path(history_path), "sha256": history_sha256},
        },
        "contract": {
            "screening_complete": True,
            "operational_failures": [],
            "held_out_data_read": False,
            "rollouts_launched": False,
            "training_launched": False,
            "arbitrary_score_assigned": False,
            "ranking_performed": False,
            "checkpoint_selected": False,
            "checkpoint_copied": False,
            "checkpoint_promoted": False,
        },
        "preregistered_candidate_ordering": {
            "eligible_set": selection.get("eligible_set"),
            "primary_order": selection.get("primary_order"),
            "tie_breakers": list(selection.get("tie_breakers", [])),
            "cumulative_kl_interpretation": dict(selection_interpretation),
            "ordering_performed": False,
            "selected_logical_iteration": None,
        },
        "screened_logical_iterations": list(EXPECTED_SCREENED_LOGICAL_ITERATIONS),
        "eligible_logical_iterations": [row["logical_iteration"] for row in rows if row["screen_eligibility"] == "ELIGIBLE"],
        "rejected_logical_iterations": [row["logical_iteration"] for row in rows if row["screen_eligibility"] == "REJECTED"],
        "row_count": len(rows),
        "rows": rows,
    }


def _write_json_atomic_no_clobber(path: Path, payload: Mapping[str, Any]) -> None:
    """Publish JSON atomically while refusing to replace an existing path."""

    path.parent.mkdir(parents=True, exist_ok=True)
    if os.path.lexists(path):
        raise FileExistsError(f"refusing existing output: {path}")
    encoded = (json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n").encode("utf-8")
    staging = path.parent / f".{path.name}.tmp-{os.getpid()}-{uuid.uuid4().hex}"
    descriptor: int | None = None
    try:
        descriptor = os.open(staging, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
        with os.fdopen(descriptor, "wb") as handle:
            descriptor = None
            handle.write(encoded)
            handle.flush()
            os.fsync(handle.fileno())
        try:
            os.link(staging, path)
        except FileExistsError as exc:
            raise FileExistsError(f"refusing existing output: {path}") from exc
    finally:
        if descriptor is not None:
            os.close(descriptor)
        try:
            staging.unlink()
        except FileNotFoundError:
            pass


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol", type=Path, required=True)
    parser.add_argument("--selection-addendum", type=Path, required=True)
    parser.add_argument("--screen-report", type=Path, required=True)
    parser.add_argument("--drift-report", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        table = build_evidence_table(
            protocol_path=args.protocol,
            selection_addendum_path=args.selection_addendum,
            screen_report_path=args.screen_report,
            drift_report_path=args.drift_report,
            output_path=args.output,
        )
        _write_json_atomic_no_clobber(args.output.expanduser().resolve(strict=False), table)
    except (EvidenceInputError, FileExistsError, OSError, ValueError) as exc:
        print(json.dumps({"ok": False, "status": "REFUSED", "error": str(exc)}, indent=2), file=sys.stderr)
        return 2
    print(json.dumps(table, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
