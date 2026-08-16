"""Strict, read-only determinism check for two V3 diagnostic replays.

This comparator is deliberately outside the V3 execution protocol.  It may
only inspect development seeds 123 and 124, requires both source receipts to
reproduce the exact measured four-check failure, and never turns diagnostic
determinism or a finite solver output into a protocol PASS.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import numbers
import os
import re
import sys
import tempfile
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))

import h0_v3_so_recovery_contract as so_recovery  # noqa: E402


COMPARATOR = Path(__file__).resolve()
TESTS = REPO_ROOT / "validation" / "test_compare_h0_primary_grf_split_v3_diagnostics.py"
V3_RUNNER = VALIDATION_ROOT / "run_h0_primary_grf_split_v3_semantic_replay.py"
RECOVERY_CLASSIFIER = VALIDATION_ROOT / "h0_v3_so_recovery_contract.py"
STATIC_OPTIMIZATION = REPO_ROOT / "static_optimization.py"
SIMULATION_RUNNER = REPO_ROOT / "simulation_runner.py"
ENVIRONMENT = REPO_ROOT / "Trajectory Generator" / "osim_trj_cmc_like.py"
PROTOCOL_RUN_ROOT = (
    REPO_ROOT
    / "validation"
    / "h0_primary_grf_split_adaptation_runs"
    / "2026-08-06_h0_primary_split_v3_semantic_replay"
)

SCHEMA_VERSION = 1
ALLOWED_DIAGNOSTIC_SEEDS = (123, 124)
EXPECTED_STEPS = 500
EXPECTED_ACTOR_FEATURES = 35
EXPECTED_ACTIONS = 2
EVENT_CONTRACT = "primary_grf_split_v1+legacy_events_v1"
BOUNDED_LSQ_MAX_ITER = 1000
MEASURED_FAILED_CHECKS = frozenset(
    {
        "zero_fallbacks",
        "zero_hard_so_fallbacks",
        "zero_bounded_lsq_failures",
        "all_bounded_lsq_verified",
    }
)

JSON_ARTIFACTS = (
    "summary.json",
    "gate.json",
    "fallback_journal.json",
    "solver_audit_journal.json",
)
ALL_LOCAL_ARTIFACTS = ("paired_replay.npz", *JSON_ARTIFACTS, "receipt.json")
RECEIPT_ARTIFACT_NAMES = {
    "arrays": "paired_replay.npz",
    "summary": "summary.json",
    "gate": "gate.json",
    "fallback_journal": "fallback_journal.json",
    "solver_audit_journal": "solver_audit_journal.json",
}
EXPECTED_ARCHIVE = {
    "target_observations": (np.dtype("float32"), (EXPECTED_STEPS, 35)),
    "reconstructed_teacher_observations": (
        np.dtype("float32"),
        (EXPECTED_STEPS, 35),
    ),
    "historical_observations": (np.dtype("float32"), (EXPECTED_STEPS, 35)),
    "teacher_means": (np.dtype("float32"), (EXPECTED_STEPS, EXPECTED_ACTIONS)),
    "teacher_logstd": (np.dtype("float32"), (EXPECTED_STEPS, EXPECTED_ACTIONS)),
    "replay_actions": (np.dtype("float32"), (EXPECTED_STEPS, EXPECTED_ACTIONS)),
    "times": (np.dtype("float64"), (EXPECTED_STEPS,)),
    "rewards": (np.dtype("float64"), (EXPECTED_STEPS,)),
    "actor_feature_names": (
        np.dtype("U64"),
        (EXPECTED_ACTOR_FEATURES,),
    ),
}
EXPECTED_CHECK_NAMES = {
    "steps_500",
    "episode_time_limit",
    "not_terminated",
    "truncated",
    "cycles_at_least_two",
    "penetration_below_25mm",
    "zero_clipping",
    "zero_timeouts",
    "zero_safety_stops",
    "zero_fallbacks",
    "zero_hard_so_fallbacks",
    "zero_reuse_previous",
    "zero_bounded_lsq_failures",
    "zero_solver_bounds_violations",
    "zero_solver_nonfinite",
    "zero_selected_infeasible",
    "zero_selected_solution_mismatch",
    "zero_residual_contract_mismatch",
    "all_bounded_lsq_verified",
    "zero_hard_invalid",
    "zero_nonfinite",
    "legacy_invalid_matches_history",
    "fixed_features_bit_exact",
    "analog_teacher_view_bit_exact",
    "times_match_history",
    "primary_view_differs",
    "actor_layout_35",
    "observation_layout_84",
    "float32_observation",
    "primary_left_only",
    "v25_disabled",
    "event_contract",
    "morphology_zero",
    "h0_not_behavior",
    "no_ppo",
}
_SHA256_RE = re.compile(r"[0-9a-f]{64}\Z")


class V3DiagnosticComparisonError(RuntimeError):
    """Raised whenever diagnostic determinism cannot be proven exactly."""


def _reject_constant(token: str) -> None:
    raise ValueError(f"non-finite JSON token {token!r}")


def _reject_duplicate_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key {key!r}")
        result[key] = value
    return result


def canonical_json_bytes(value: Any) -> bytes:
    try:
        return json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise V3DiagnosticComparisonError(
            f"value is not finite strict JSON: {exc}"
        ) from exc


def strict_json_load(path: str | Path) -> Any:
    resolved = Path(path).expanduser().resolve()
    try:
        payload = json.loads(
            resolved.read_text(encoding="utf-8"),
            parse_constant=_reject_constant,
            object_pairs_hook=_reject_duplicate_pairs,
        )
        canonical_json_bytes(payload)
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as exc:
        raise V3DiagnosticComparisonError(
            f"cannot read strict JSON {resolved}: {exc}"
        ) from exc
    return payload


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    try:
        with Path(path).expanduser().resolve().open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise V3DiagnosticComparisonError(f"cannot hash {path}: {exc}") from exc
    return digest.hexdigest()


def _is_within(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
    except ValueError:
        return False
    return True


def source_record(path: str | Path) -> dict[str, Any]:
    resolved = Path(path).expanduser().resolve()
    if not resolved.is_file():
        raise V3DiagnosticComparisonError(f"source is not a file: {resolved}")
    if _is_within(resolved, REPO_ROOT):
        location = {
            "path": resolved.relative_to(REPO_ROOT).as_posix(),
        }
    else:
        location = {
            "path": resolved.as_posix(),
            "path_scope": "external_absolute",
        }
    return {
        **location,
        "sha256": sha256_file(resolved),
        "size_bytes": resolved.stat().st_size,
    }


def _mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise V3DiagnosticComparisonError(f"{label} must be a JSON object")
    return dict(value)


def _sequence(value: Any, label: str) -> list[Any]:
    if isinstance(value, (str, bytes, bytearray)) or not isinstance(value, Sequence):
        raise V3DiagnosticComparisonError(f"{label} must be a JSON array")
    return list(value)


def _boolean(value: Any, label: str) -> bool:
    if type(value) is not bool:
        raise V3DiagnosticComparisonError(f"{label} must be a JSON boolean")
    return value


def _counter(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, numbers.Integral):
        raise V3DiagnosticComparisonError(
            f"{label} must be a non-negative JSON integer"
        )
    result = int(value)
    if result < 0:
        raise V3DiagnosticComparisonError(
            f"{label} must be a non-negative JSON integer"
        )
    return result


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise V3DiagnosticComparisonError(f"{label} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise V3DiagnosticComparisonError(f"{label} must be a finite number")
    return result


def _record_path(record_value: Any, label: str) -> Path:
    record = _mapping(record_value, label)
    repo_schema = {"path", "sha256", "size_bytes"}
    external_schema = {"path", "path_scope", "sha256", "size_bytes"}
    if set(record) not in (repo_schema, external_schema):
        raise V3DiagnosticComparisonError(f"{label} source-record schema drifted")
    path_value = record.get("path")
    scope = record.get("path_scope", "repository_relative")
    if not isinstance(path_value, str) or not path_value:
        raise V3DiagnosticComparisonError(f"{label}.path must be a non-empty string")
    if scope == "repository_relative":
        pure = PurePosixPath(path_value)
        if pure.is_absolute() or ".." in pure.parts or "\\" in path_value:
            raise V3DiagnosticComparisonError(f"{label}.path is not canonical")
        path = (REPO_ROOT / Path(*pure.parts)).resolve()
        if not _is_within(path, REPO_ROOT):
            raise V3DiagnosticComparisonError(f"{label}.path escapes the repository")
    elif scope == "external_absolute":
        raw = Path(path_value)
        if not raw.is_absolute() or raw.as_posix() != path_value:
            raise V3DiagnosticComparisonError(f"{label}.path is not canonical")
        path = raw.expanduser().resolve()
    else:
        raise V3DiagnosticComparisonError(f"{label}.path_scope is invalid")
    digest = record.get("sha256")
    if not isinstance(digest, str) or _SHA256_RE.fullmatch(digest) is None:
        raise V3DiagnosticComparisonError(f"{label}.sha256 is invalid")
    size = _counter(record.get("size_bytes"), f"{label}.size_bytes")
    if not path.is_file() or path.stat().st_size != size or sha256_file(path) != digest:
        raise V3DiagnosticComparisonError(f"{label} does not match its source file")
    return path


def _expected_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    def counter(key: str) -> int | None:
        try:
            return _counter(summary.get(key), f"summary.{key}")
        except V3DiagnosticComparisonError:
            return None

    def finite(key: str) -> float | None:
        try:
            return _finite(summary.get(key), f"summary.{key}")
        except V3DiagnosticComparisonError:
            return None

    penetration = finite("grf_penetration_max_m")
    morphology_weight = finite("morphology_weight")
    cycles = counter("phase_valid_cycle_count")
    invalid_events = counter("invalid_event_count")
    historical_invalid_events = counter("historical_invalid_event_count")
    bounded_lsq = counter("so_solver_bounded_ls_invocation_count")
    verified_bounded_lsq = counter("so_solver_verified_bounded_ls_count")
    checks = {
        "steps_500": counter("steps") == EXPECTED_STEPS,
        "episode_time_limit": summary.get("end_reason") == "episode_time_limit",
        "not_terminated": type(summary.get("terminated")) is bool
        and summary.get("terminated") is False,
        "truncated": type(summary.get("truncated")) is bool
        and summary.get("truncated") is True,
        "cycles_at_least_two": cycles is not None and cycles >= 2,
        "penetration_below_25mm": penetration is not None
        and 0.0 <= penetration < 0.025,
        "zero_clipping": counter("action_clipped_values") == 0,
        "zero_timeouts": counter("timeout_count") == 0,
        "zero_safety_stops": counter("safety_stop_count") == 0,
        "zero_fallbacks": counter("fallback_count") == 0,
        "zero_hard_so_fallbacks": counter("so_solver_hard_fallback_count") == 0,
        "zero_reuse_previous": counter("so_solver_reuse_previous_count") == 0,
        "zero_bounded_lsq_failures": counter("so_solver_bounded_ls_unsuccessful_count")
        == 0,
        "zero_solver_bounds_violations": counter("so_solver_bounds_violation_count")
        == 0,
        "zero_solver_nonfinite": counter("so_solver_nonfinite_count") == 0,
        "zero_selected_infeasible": counter("so_solver_selected_infeasible_count") == 0,
        "zero_selected_solution_mismatch": counter(
            "so_solver_selected_solution_mismatch_count"
        )
        == 0,
        "zero_residual_contract_mismatch": counter(
            "so_solver_residual_contract_mismatch_count"
        )
        == 0,
        "all_bounded_lsq_verified": bounded_lsq is not None
        and verified_bounded_lsq is not None
        and bounded_lsq == verified_bounded_lsq,
        "zero_hard_invalid": counter("hard_invalid_count") == 0,
        "zero_nonfinite": counter("nonfinite_count") == 0,
        "legacy_invalid_matches_history": invalid_events is not None
        and invalid_events == historical_invalid_events,
        "fixed_features_bit_exact": counter("fixed_feature_mismatch_count") == 0,
        "analog_teacher_view_bit_exact": counter("teacher_view_mismatch_count") == 0,
        "times_match_history": counter("time_mismatch_count") == 0,
        "primary_view_differs": (counter("mutable_feature_difference_count") or 0) > 0,
        "actor_layout_35": counter("n_actor") == EXPECTED_ACTOR_FEATURES,
        "observation_layout_84": counter("n_observation") == 84,
        "float32_observation": summary.get("observation_dtype") == "float32",
        "primary_left_only": summary.get("online_grf_applied_sides") == ["left"],
        "v25_disabled": summary.get("binary_phase_fsm_mode") == "disabled",
        "event_contract": summary.get("event_contract_id") == EVENT_CONTRACT,
        "morphology_zero": morphology_weight == 0.0,
        "h0_not_behavior": type(summary.get("h0_used_for_behavior")) is bool
        and summary.get("h0_used_for_behavior") is False,
        "no_ppo": counter("ppo_updates") == 0,
    }
    if set(checks) != EXPECTED_CHECK_NAMES:
        raise AssertionError("internal V3 diagnostic check schema drifted")
    passed = all(checks.values())
    return {
        "schema_version": 3,
        "status": (
            "PASS_H0_PRIMARY_SPLIT_V3_REPLAY"
            if passed
            else "FAIL_H0_PRIMARY_SPLIT_V3_REPLAY"
        ),
        "passed": passed,
        "checks": checks,
    }


def _validate_known_failure(summary: dict[str, Any], gate: dict[str, Any]) -> None:
    expected_gate = _expected_gate(summary)
    if canonical_json_bytes(gate) != canonical_json_bytes(expected_gate):
        raise V3DiagnosticComparisonError(
            "diagnostic gate does not equal the independently recomputed V3 gate"
        )
    checks = _mapping(gate.get("checks"), "gate.checks")
    if set(checks) != EXPECTED_CHECK_NAMES:
        raise V3DiagnosticComparisonError("diagnostic gate check schema drifted")
    for name, result in checks.items():
        _boolean(result, f"gate.checks.{name}")
    failed = frozenset(name for name, result in checks.items() if not result)
    if (
        gate.get("schema_version") != 3
        or gate.get("status") != "FAIL_H0_PRIMARY_SPLIT_V3_REPLAY"
        or type(gate.get("passed")) is not bool
        or gate.get("passed") is not False
        or failed != MEASURED_FAILED_CHECKS
    ):
        raise V3DiagnosticComparisonError(
            "diagnostic does not reproduce the exact measured four-check failure"
        )
    fallback_count = _counter(summary.get("fallback_count"), "fallback_count")
    so_fallback_count = _counter(summary.get("so_fallback_count"), "so_fallback_count")
    sea_fallback_count = _counter(
        summary.get("sea_plugin_fallback_count"), "sea_plugin_fallback_count"
    )
    bounded_invocations = _counter(
        summary.get("so_solver_bounded_ls_invocation_count"),
        "so_solver_bounded_ls_invocation_count",
    )
    verified_bounded = _counter(
        summary.get("so_solver_verified_bounded_ls_count"),
        "so_solver_verified_bounded_ls_count",
    )
    hard_fallbacks = _counter(
        summary.get("so_solver_hard_fallback_count"),
        "so_solver_hard_fallback_count",
    )
    unsuccessful_bounded = _counter(
        summary.get("so_solver_bounded_ls_unsuccessful_count"),
        "so_solver_bounded_ls_unsuccessful_count",
    )
    residual_mismatches = _counter(
        summary.get("so_solver_residual_contract_mismatch_count"),
        "so_solver_residual_contract_mismatch_count",
    )
    if (
        fallback_count <= 0
        or so_fallback_count != fallback_count
        or sea_fallback_count != 0
        or bounded_invocations <= 0
        or hard_fallbacks <= 0
        or unsuccessful_bounded <= 0
        or verified_bounded >= bounded_invocations
        or residual_mismatches != 0
        or checks["zero_residual_contract_mismatch"] is not True
    ):
        raise V3DiagnosticComparisonError(
            "measured failure counters are internally inconsistent"
        )


def _validate_fallback_journal(
    journal_value: Any, summary: Mapping[str, Any]
) -> list[Any]:
    journal = _sequence(journal_value, "fallback journal")
    fallback_total = 0
    previous_step = 0
    for index, raw in enumerate(journal):
        row = _mapping(raw, f"fallback journal[{index}]")
        if set(row) != {
            "step",
            "time_s",
            "so_fallback",
            "sea_plugin_fallback_count",
        }:
            raise V3DiagnosticComparisonError(
                f"fallback journal[{index}] schema drifted"
            )
        step = _counter(row["step"], f"fallback journal[{index}].step")
        if step <= previous_step or step > EXPECTED_STEPS:
            raise V3DiagnosticComparisonError("fallback steps are not monotonic")
        previous_step = step
        _finite(row["time_s"], f"fallback journal[{index}].time_s")
        so_fallback = _counter(
            row["so_fallback"], f"fallback journal[{index}].so_fallback"
        )
        if so_fallback not in {0, 1}:
            raise V3DiagnosticComparisonError(
                "fallback journal so_fallback must be the canonical 0/1 counter"
            )
        sea_fallbacks = _counter(
            row["sea_plugin_fallback_count"],
            f"fallback journal[{index}].sea_plugin_fallback_count",
        )
        if so_fallback == 0 and sea_fallbacks == 0:
            raise V3DiagnosticComparisonError("fallback journal contains an empty row")
        fallback_total += so_fallback + sea_fallbacks
    if fallback_total != _counter(summary.get("fallback_count"), "fallback_count"):
        raise V3DiagnosticComparisonError(
            "fallback journal does not reconcile with summary.fallback_count"
        )
    return journal


_CLASSIFIER_TO_SUMMARY = {
    "control_window_count": "so_solver_control_window_count",
    "solver_invocation_count": "so_solver_attempt_count",
    "primary_solver_nonconvergence_count": ("so_solver_primary_nonconvergence_count"),
    "bounded_ls_invocation_count": "so_solver_bounded_ls_invocation_count",
    "selected_bounded_ls_count": "so_solver_selected_bounded_ls_count",
    "verified_bounded_ls_count": "so_solver_verified_bounded_ls_count",
    "hard_so_fallback_count": "so_solver_hard_fallback_count",
    "reuse_previous_count": "so_solver_reuse_previous_count",
    "bounded_ls_unsuccessful_count": "so_solver_bounded_ls_unsuccessful_count",
    "bounds_violation_count": "so_solver_bounds_violation_count",
    "nonfinite_solver_count": "so_solver_nonfinite_count",
    "selected_infeasible_count": "so_solver_selected_infeasible_count",
    "selected_solution_mismatch_count": ("so_solver_selected_solution_mismatch_count"),
    "residual_contract_mismatch_count": ("so_solver_residual_contract_mismatch_count"),
}


def _solution_contract(value: Any, label: str) -> dict[str, Any]:
    solution = _mapping(value, label)
    expected_keys = {
        "output_shape_matches",
        "output_finite",
        "output_sha256",
        "bound_violation_max",
        "equality_residual_finite",
        "equality_residual_norm",
        "equality_residual_max_abs",
        "equality_residual_relative_norm",
    }
    if set(solution) != expected_keys:
        raise V3DiagnosticComparisonError(f"{label} schema drifted")
    if (
        _boolean(solution["output_shape_matches"], f"{label}.output_shape_matches")
        is not True
        or _boolean(solution["output_finite"], f"{label}.output_finite") is not True
        or _boolean(
            solution["equality_residual_finite"],
            f"{label}.equality_residual_finite",
        )
        is not True
    ):
        raise V3DiagnosticComparisonError(f"{label} output is not finite/complete")
    digest = solution.get("output_sha256")
    if not isinstance(digest, str) or _SHA256_RE.fullmatch(digest) is None:
        raise V3DiagnosticComparisonError(f"{label}.output_sha256 is invalid")
    bound_violation = _finite(
        solution.get("bound_violation_max"), f"{label}.bound_violation_max"
    )
    if bound_violation < 0.0 or bound_violation > so_recovery.BOUND_TOLERANCE:
        raise V3DiagnosticComparisonError(f"{label} violates frozen bounds")
    for field in (
        "equality_residual_norm",
        "equality_residual_max_abs",
        "equality_residual_relative_norm",
    ):
        if _finite(solution.get(field), f"{label}.{field}") < 0.0:
            raise V3DiagnosticComparisonError(f"{label}.{field} must be non-negative")
    return solution


def _validate_status0_max_iter_case(
    *,
    window: Mapping[str, Any],
    attempt: Mapping[str, Any],
    normalized_attempt: Mapping[str, Any],
    label: str,
) -> None:
    path = _mapping(attempt.get("solver_path"), f"{label}.solver_path")
    if (
        _boolean(attempt.get("solver_fallback_used"), f"{label}.fallback_used")
        is not True
        or _boolean(attempt.get("selected"), f"{label}.selected") is not True
        or _boolean(
            attempt.get("feasibility_accepted"), f"{label}.feasibility_accepted"
        )
        is not True
        or _boolean(path.get("bounded_lsq_used"), f"{label}.bounded_lsq_used")
        is not True
        or _boolean(path.get("bounded_lsq_invoked"), f"{label}.bounded_lsq_invoked")
        is not True
        or _boolean(path.get("bounded_lsq_success"), f"{label}.bounded_lsq_success")
        is not False
        or _boolean(
            path.get("reuse_previous_solution"),
            f"{label}.reuse_previous_solution",
        )
        is not False
        or _boolean(path.get("hard_fallback"), f"{label}.hard_fallback") is not True
    ):
        raise V3DiagnosticComparisonError(
            f"{label} does not match the raw status=0/max-iter signature"
        )
    if (
        _counter(path.get("bounded_lsq_status"), f"{label}.bounded_lsq_status") != 0
        or _counter(
            path.get("bounded_lsq_iterations"),
            f"{label}.bounded_lsq_iterations",
        )
        != BOUNDED_LSQ_MAX_ITER
    ):
        raise V3DiagnosticComparisonError(
            f"{label} is not a bounded-LS status=0/max-iter case"
        )
    message = path.get("bounded_lsq_message")
    if (
        not isinstance(message, str)
        or "maximum number of iterations is exceeded" not in message.lower()
    ):
        raise V3DiagnosticComparisonError(f"{label} bounded-LS message is not max-iter")
    for field in (
        "input_matrix_finite",
        "input_target_finite",
        "weights_finite",
        "bounds_finite",
        "warm_start_finite",
    ):
        if _boolean(path.get(field), f"{label}.{field}") is not True:
            raise V3DiagnosticComparisonError(f"{label} has non-finite solver input")
    for field in ("bounded_lsq_cost", "bounded_lsq_optimality"):
        if _finite(path.get(field), f"{label}.{field}") < 0.0:
            raise V3DiagnosticComparisonError(f"{label}.{field} must be non-negative")
    _solution_contract(path.get("slsqp_solution"), f"{label}.slsqp_solution")
    bounded_solution = _solution_contract(
        path.get("bounded_lsq_solution"), f"{label}.bounded_lsq_solution"
    )
    selected_solution = _solution_contract(
        path.get("selected_solution"), f"{label}.selected_solution"
    )
    if bounded_solution["output_sha256"] != selected_solution["output_sha256"]:
        raise V3DiagnosticComparisonError(
            f"{label} selected output is not the bounded-LS output"
        )
    if (
        _boolean(
            normalized_attempt.get("residual_contract_matches"),
            f"{label}.residual_contract_matches",
        )
        is not True
    ):
        raise V3DiagnosticComparisonError(
            f"{label} residual/feasibility contract is inconsistent"
        )
    served_digest = window.get("served_solution_sha256")
    if (
        _boolean(
            window.get("selected_solver_solution_matches_served"),
            f"{label}.selected_solver_solution_matches_served",
        )
        is not True
        or served_digest != selected_solution["output_sha256"]
    ):
        raise V3DiagnosticComparisonError(f"{label} served output mismatch")


def _validate_solver_journal(
    value: Any, summary: Mapping[str, Any]
) -> tuple[list[Any], dict[str, Any]]:
    journal = _sequence(value, "solver audit journal")
    if len(journal) != EXPECTED_STEPS:
        raise V3DiagnosticComparisonError(
            f"solver audit journal must contain {EXPECTED_STEPS} policy steps"
        )
    totals = {key: 0 for key in _CLASSIFIER_TO_SUMMARY}
    status0_max_iter_cases = 0
    for offset, raw in enumerate(journal, start=1):
        row = _mapping(raw, f"solver audit journal[{offset - 1}]")
        if set(row) != {"step", "time_s", "control_windows"}:
            raise V3DiagnosticComparisonError(
                f"solver audit journal[{offset - 1}] schema drifted"
            )
        if _counter(row["step"], "solver journal step") != offset:
            raise V3DiagnosticComparisonError("solver journal steps are not contiguous")
        _finite(row["time_s"], "solver journal time_s")
        windows = _sequence(row["control_windows"], "solver journal control_windows")
        try:
            classification = so_recovery.classify_policy_step(windows)
        except so_recovery.SORecoveryContractError as exc:
            raise V3DiagnosticComparisonError(
                f"solver journal step {offset} violates the current classifier: {exc}"
            ) from exc
        counters = _mapping(
            classification.get("counters"), f"solver classification step {offset}"
        )
        normalized_windows = _sequence(
            classification.get("entries"), f"normalized solver step {offset}"
        )
        if len(normalized_windows) != len(windows):
            raise V3DiagnosticComparisonError(
                f"solver classifier dropped windows at step {offset}"
            )
        for key in totals:
            totals[key] += _counter(
                counters.get(key), f"solver classification step {offset}.{key}"
            )
        for window_index, (raw_window_value, normalized_window_value) in enumerate(
            zip(windows, normalized_windows), start=1
        ):
            raw_window = _mapping(
                raw_window_value, f"step {offset}.window {window_index}"
            )
            normalized_window = _mapping(
                normalized_window_value,
                f"step {offset}.normalized window {window_index}",
            )
            attempts = _sequence(
                raw_window.get("attempts"),
                f"step {offset}.window {window_index}.attempts",
            )
            normalized_attempts = _sequence(
                normalized_window.get("attempts"),
                f"step {offset}.normalized window {window_index}.attempts",
            )
            if len(attempts) != len(normalized_attempts):
                raise V3DiagnosticComparisonError(
                    f"solver classifier dropped attempts at step {offset}"
                )
            for attempt_index, (attempt_value, normalized_value) in enumerate(
                zip(attempts, normalized_attempts), start=1
            ):
                attempt = _mapping(
                    attempt_value,
                    f"step {offset}.window {window_index}.attempt {attempt_index}",
                )
                normalized_attempt = _mapping(
                    normalized_value,
                    (
                        f"step {offset}.normalized window {window_index}."
                        f"attempt {attempt_index}"
                    ),
                )
                path = _mapping(
                    attempt.get("solver_path"),
                    f"step {offset}.window {window_index}.attempt {attempt_index}.path",
                )
                hard = _boolean(
                    path.get("hard_fallback"),
                    f"step {offset}.window {window_index}.hard_fallback",
                )
                bounded_used = _boolean(
                    path.get("bounded_lsq_used"),
                    f"step {offset}.window {window_index}.bounded_lsq_used",
                )
                bounded_unsuccessful = bool(
                    bounded_used
                    and not _boolean(
                        path.get("bounded_lsq_success"),
                        f"step {offset}.window {window_index}.bounded_lsq_success",
                    )
                )
                if hard is not bounded_unsuccessful:
                    raise V3DiagnosticComparisonError(
                        "raw hard and unsuccessful bounded-LS cases diverge"
                    )
                if hard:
                    status0_max_iter_cases += 1
                    _validate_status0_max_iter_case(
                        window=raw_window,
                        attempt=attempt,
                        normalized_attempt=normalized_attempt,
                        label=(
                            f"step {offset}.window {window_index}."
                            f"attempt {attempt_index}"
                        ),
                    )
    for counter_key, summary_key in _CLASSIFIER_TO_SUMMARY.items():
        observed = _counter(summary.get(summary_key), f"summary.{summary_key}")
        if totals[counter_key] != observed:
            raise V3DiagnosticComparisonError(
                f"classifier aggregate {counter_key} does not match {summary_key}"
            )
    zero_required = (
        "reuse_previous_count",
        "bounds_violation_count",
        "nonfinite_solver_count",
        "selected_infeasible_count",
        "selected_solution_mismatch_count",
        "residual_contract_mismatch_count",
    )
    if any(totals[key] != 0 for key in zero_required):
        raise V3DiagnosticComparisonError(
            "classifier found reuse/nonfinite/bounds/infeasible/residual/served mismatch"
        )
    if (
        status0_max_iter_cases <= 0
        or status0_max_iter_cases != totals["hard_so_fallback_count"]
        or status0_max_iter_cases != totals["bounded_ls_unsuccessful_count"]
    ):
        raise V3DiagnosticComparisonError(
            "raw status=0/max-iter cases do not explain every hard/unsuccessful count"
        )
    return journal, {
        "classifier_totals": totals,
        "bounded_lsq_status0_max_iter_case_count": status0_max_iter_cases,
        "all_raw_hard_cases_match_status0_max_iter_signature": True,
        "scientific_disposition": "UNACCEPTED_DIAGNOSTIC_REQUIRES_PROTOCOL_DECISION",
    }


def _validate_receipt(receipt: dict[str, Any], *, seed: int, directory: Path) -> None:
    expected_keys = {
        "schema_version",
        "status",
        "passed",
        "seed",
        "diagnostic",
        "execution_lock",
        "attempt_claim",
        "artifacts",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
        "protected_trials_opened",
    }
    if set(receipt) != expected_keys:
        raise V3DiagnosticComparisonError("diagnostic receipt schema drifted")
    if (
        _counter(receipt.get("schema_version"), "receipt.schema_version") != 3
        or receipt.get("status") != "FAIL_H0_PRIMARY_SPLIT_V3_REPLAY"
        or _boolean(receipt.get("passed"), "receipt.passed") is not False
        or _counter(receipt.get("seed"), "receipt.seed") != seed
        or _boolean(receipt.get("diagnostic"), "receipt.diagnostic") is not True
        or receipt.get("execution_lock") is not None
        or receipt.get("attempt_claim") is not None
        or _counter(receipt.get("actor_updates"), "receipt.actor_updates") != 0
        or _counter(receipt.get("critic_updates"), "receipt.critic_updates") != 0
        or _counter(receipt.get("ppo_updates"), "receipt.ppo_updates") != 0
        or receipt.get("protected_trials_opened") != []
    ):
        raise V3DiagnosticComparisonError(
            "source receipt is not a failed, update-free V3 diagnostic"
        )
    artifacts = _mapping(receipt.get("artifacts"), "receipt.artifacts")
    if set(artifacts) != {
        *RECEIPT_ARTIFACT_NAMES,
        "historical_trace",
        "historical_summary",
    }:
        raise V3DiagnosticComparisonError("diagnostic receipt artifact schema drifted")
    for name, filename in RECEIPT_ARTIFACT_NAMES.items():
        resolved = _record_path(artifacts[name], f"receipt.artifacts.{name}")
        if resolved != (directory / filename).resolve():
            raise V3DiagnosticComparisonError(
                f"receipt artifact {name} does not identify {filename}"
            )
    for name in ("historical_trace", "historical_summary"):
        history_path = _record_path(artifacts[name], f"receipt.artifacts.{name}")
        if f"seed{seed}" not in history_path.as_posix():
            raise V3DiagnosticComparisonError(
                f"receipt artifact {name} does not identify seed {seed}"
            )


def _load_archive(path: Path) -> dict[str, np.ndarray]:
    try:
        with np.load(path, allow_pickle=False) as archive:
            if tuple(archive.files) != tuple(EXPECTED_ARCHIVE):
                raise V3DiagnosticComparisonError(
                    f"NPZ key/order drift in {path}: {archive.files!r}"
                )
            arrays = {key: np.asarray(archive[key]) for key in archive.files}
    except V3DiagnosticComparisonError:
        raise
    except (OSError, ValueError, KeyError) as exc:
        raise V3DiagnosticComparisonError(
            f"cannot load strict NPZ {path}: {exc}"
        ) from exc
    for key, values in arrays.items():
        expected_dtype, expected_shape = EXPECTED_ARCHIVE[key]
        if values.dtype != expected_dtype or values.shape != expected_shape:
            raise V3DiagnosticComparisonError(
                f"NPZ array {key} contract drifted: "
                f"{values.dtype}/{values.shape} != {expected_dtype}/{expected_shape}"
            )
        if values.dtype.kind in "fc" and not np.all(np.isfinite(values)):
            raise V3DiagnosticComparisonError(f"NPZ array {key} is non-finite")
        if values.dtype.kind == "U":
            names = values.tolist()
            if any(not isinstance(item, str) or not item for item in names):
                raise V3DiagnosticComparisonError(
                    "actor_feature_names contains an empty/non-string value"
                )
            if len(set(names)) != len(names):
                raise V3DiagnosticComparisonError(
                    "actor_feature_names contains duplicates"
                )
    return arrays


def _load_diagnostic(directory_value: str | Path, *, seed: int) -> dict[str, Any]:
    directory = Path(directory_value).expanduser().resolve()
    if not directory.is_dir():
        raise V3DiagnosticComparisonError(
            f"diagnostic directory does not exist: {directory}"
        )
    if _is_within(directory, PROTOCOL_RUN_ROOT.resolve()):
        raise V3DiagnosticComparisonError(
            "diagnostic inputs must remain outside the canonical V3 protocol run root"
        )
    paths = {name: directory / name for name in ALL_LOCAL_ARTIFACTS}
    missing = [name for name, path in paths.items() if not path.is_file()]
    if missing:
        raise V3DiagnosticComparisonError(
            f"diagnostic directory is incomplete; missing {missing!r}"
        )
    summary = _mapping(strict_json_load(paths["summary.json"]), "summary")
    gate = _mapping(strict_json_load(paths["gate.json"]), "gate")
    fallback = _validate_fallback_journal(
        strict_json_load(paths["fallback_journal.json"]), summary
    )
    solver, solver_classification = _validate_solver_journal(
        strict_json_load(paths["solver_audit_journal.json"]), summary
    )
    receipt = _mapping(strict_json_load(paths["receipt.json"]), "receipt")
    if _counter(summary.get("seed"), "summary.seed") != seed:
        raise V3DiagnosticComparisonError("diagnostic summary seed is incorrect")
    _validate_known_failure(summary, gate)
    _validate_receipt(receipt, seed=seed, directory=directory)
    archive = _load_archive(paths["paired_replay.npz"])
    return {
        "directory": directory,
        "paths": paths,
        "summary": summary,
        "gate": gate,
        "fallback": fallback,
        "solver": solver,
        "solver_classification": solver_classification,
        "archive": archive,
    }


def _compare_exact(left: Any, right: Any, label: str) -> None:
    if canonical_json_bytes(left) != canonical_json_bytes(right):
        raise V3DiagnosticComparisonError(f"{label} differs between replicas")


def _compare_archive(
    left: Mapping[str, np.ndarray], right: Mapping[str, np.ndarray]
) -> dict[str, Any]:
    if tuple(left) != tuple(right) or tuple(left) != tuple(EXPECTED_ARCHIVE):
        raise V3DiagnosticComparisonError("NPZ key/order differs between replicas")
    records: dict[str, Any] = {}
    for key in EXPECTED_ARCHIVE:
        a = left[key]
        b = right[key]
        if a.dtype != b.dtype or a.shape != b.shape:
            raise V3DiagnosticComparisonError(f"NPZ array {key} metadata differs")
        a_bytes = np.ascontiguousarray(a).tobytes(order="C")
        b_bytes = np.ascontiguousarray(b).tobytes(order="C")
        if a_bytes != b_bytes:
            raise V3DiagnosticComparisonError(f"NPZ array {key} is not bit-exact")
        records[key] = {
            "dtype": a.dtype.str,
            "shape": list(a.shape),
            "payload_sha256": hashlib.sha256(a_bytes).hexdigest(),
            "bit_exact": True,
        }
    return records


def _publish_temporary_exclusive(temporary: Path, destination: Path) -> None:
    flags = os.O_CREAT | os.O_EXCL | os.O_WRONLY | getattr(os, "O_BINARY", 0)
    try:
        descriptor = os.open(str(destination), flags, 0o600)
    except FileExistsError as exc:
        raise V3DiagnosticComparisonError(
            f"refusing to clobber: {destination}"
        ) from exc
    else:
        os.close(descriptor)
    os.replace(str(temporary), str(destination))


def write_json_exclusive(path: str | Path, payload: Any) -> Path:
    destination = Path(path).expanduser().resolve()
    canonical_json_bytes(payload)
    destination.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_raw = tempfile.mkstemp(
        prefix=f".{destination.name}.", suffix=".tmp", dir=destination.parent
    )
    temporary = Path(temporary_raw)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            json.dump(
                payload,
                stream,
                indent=2,
                sort_keys=True,
                ensure_ascii=False,
                allow_nan=False,
            )
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        _publish_temporary_exclusive(temporary, destination)
    finally:
        if temporary.exists():
            temporary.unlink()
    return destination


def compare_diagnostics(
    *, seed: int, replica_a_dir: str | Path, replica_b_dir: str | Path
) -> dict[str, Any]:
    if isinstance(seed, bool) or seed not in ALLOWED_DIAGNOSTIC_SEEDS:
        raise V3DiagnosticComparisonError(
            "comparison is restricted to development diagnostic seeds 123/124"
        )
    if Path(so_recovery.__file__).resolve() != RECOVERY_CLASSIFIER.resolve():
        raise V3DiagnosticComparisonError(
            "loaded solver classifier does not match the repository source"
        )
    a_path = Path(replica_a_dir).expanduser().resolve()
    b_path = Path(replica_b_dir).expanduser().resolve()
    if a_path == b_path:
        raise V3DiagnosticComparisonError("replicas must be distinct directories")
    replica_a = _load_diagnostic(a_path, seed=seed)
    replica_b = _load_diagnostic(b_path, seed=seed)
    for filename, key in (
        ("summary.json", "summary"),
        ("gate.json", "gate"),
        ("fallback_journal.json", "fallback"),
        ("solver_audit_journal.json", "solver"),
    ):
        _compare_exact(replica_a[key], replica_b[key], filename)
        if (
            replica_a["paths"][filename].read_bytes()
            != replica_b["paths"][filename].read_bytes()
        ):
            raise V3DiagnosticComparisonError(
                f"{filename} is semantically equal but not byte-exact"
            )
    archive_records = _compare_archive(replica_a["archive"], replica_b["archive"])
    _compare_exact(
        replica_a["solver_classification"],
        replica_b["solver_classification"],
        "independent solver classification",
    )
    input_records = {
        label: {
            name: source_record(replica["paths"][name]) for name in ALL_LOCAL_ARTIFACTS
        }
        for label, replica in (("replica_a", replica_a), ("replica_b", replica_b))
    }
    summary = replica_a["summary"]
    return {
        "schema_version": SCHEMA_VERSION,
        "status": "MATCH_H0_PRIMARY_SPLIT_V3_DIAGNOSTIC_FAILURE_REPRODUCED",
        "passed": False,
        "comparison_passed": True,
        "diagnostic": True,
        "seed": seed,
        "source_protocol_gate_passed": False,
        "source_protocol_gate_status": "FAIL_H0_PRIMARY_SPLIT_V3_REPLAY",
        "measured_source_gate_failures": sorted(MEASURED_FAILED_CHECKS),
        "zero_residual_contract_mismatch": True,
        "eligible_for_protocol_promotion": False,
        "exact_json_artifacts": list(JSON_ARTIFACTS),
        "npz_arrays": archive_records,
        "observed_fallback_count": _counter(
            summary.get("fallback_count"), "summary.fallback_count"
        ),
        "observed_verified_bounded_ls_count": _counter(
            summary.get("so_solver_verified_bounded_ls_count"),
            "summary.so_solver_verified_bounded_ls_count",
        ),
        "independent_solver_classification": replica_a["solver_classification"],
        "sources": {
            "comparator": source_record(COMPARATOR),
            "tests": source_record(TESTS),
            "v3_runner": source_record(V3_RUNNER),
            "solver_classifier": source_record(RECOVERY_CLASSIFIER),
            "static_optimization": source_record(STATIC_OPTIMIZATION),
            "simulation_runner": source_record(SIMULATION_RUNNER),
            "environment": source_record(ENVIRONMENT),
        },
        "inputs": input_records,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--replica-a-dir", type=Path, required=True)
    parser.add_argument("--replica-b-dir", type=Path, required=True)
    parser.add_argument(
        "--receipt",
        type=Path,
        required=True,
        help="New diagnostic receipt path; existing paths are never overwritten.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        receipt = compare_diagnostics(
            seed=args.seed,
            replica_a_dir=args.replica_a_dir,
            replica_b_dir=args.replica_b_dir,
        )
        write_json_exclusive(args.receipt, receipt)
    except V3DiagnosticComparisonError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(receipt, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
