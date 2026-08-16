"""Policy-aware A/B/C gates for the V3 zero-port V25 branch."""

from __future__ import annotations

import json
import os
import tempfile
from pathlib import Path
from typing import Any, Mapping, Sequence

from validation import compare_h0_v25_abc as legacy
from validation import h0_v3_so_recovery_contract as so_recovery
from validation import h0_v3_v25_abc_post_zero_port_contract as contract


class PostZeroPortGateError(RuntimeError):
    """Raised when an A/B/C artifact cannot be accepted fail-closed."""


def strict_json_load(path: str | Path) -> Any:
    return legacy.strict_json_load(path)


def canonical_json_bytes(value: Any) -> bytes:
    return legacy.canonical_json_bytes(value)


def payload_sha256(value: Any) -> str:
    return legacy.payload_sha256(value)


def write_json_exclusive(path: str | Path, payload: Any) -> Path:
    destination = Path(path).expanduser().resolve()
    canonical_json_bytes(payload)
    destination.parent.mkdir(parents=True, exist_ok=True)
    descriptor, raw_temporary = tempfile.mkstemp(
        prefix=f".{destination.name}.", suffix=".tmp", dir=destination.parent
    )
    temporary = Path(raw_temporary)
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
        flags = os.O_CREAT | os.O_EXCL | os.O_WRONLY | getattr(os, "O_BINARY", 0)
        try:
            claim = os.open(str(destination), flags, 0o600)
        except FileExistsError as exc:
            raise PostZeroPortGateError(f"refusing to clobber: {destination}") from exc
        else:
            os.close(claim)
        os.replace(temporary, destination)
    finally:
        if temporary.exists():
            temporary.unlink()
    return destination


def _counter(value: Any, label: str) -> int:
    if type(value) is not int or value < 0:
        raise PostZeroPortGateError(f"{label} must be a non-negative integer")
    return value


def _mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise PostZeroPortGateError(f"{label} must be an object")
    return dict(value)


SUMMARY_COUNTERS = {
    "control_window_count": "so_solver_control_window_count",
    "solver_invocation_count": "so_solver_attempt_count",
    "primary_solver_nonconvergence_count": "so_solver_primary_nonconvergence_count",
    "bounded_ls_invocation_count": "so_solver_bounded_ls_invocation_count",
    "selected_bounded_ls_count": "so_solver_selected_bounded_ls_count",
    "verified_bounded_ls_count": "so_solver_verified_bounded_ls_count",
    "verified_bounded_ls_success_count": (
        "so_solver_verified_bounded_ls_success_count"
    ),
    "verified_status0_max_iter_count": (
        "so_solver_verified_status0_max_iter_count"
    ),
    "unaccepted_hard_so_fallback_count": (
        "so_solver_unaccepted_hard_fallback_count"
    ),
    "unaccepted_bounded_ls_count": "so_solver_unaccepted_bounded_ls_count",
    "hard_so_fallback_count": "so_solver_hard_fallback_count",
    "reuse_previous_count": "so_solver_reuse_previous_count",
    "bounded_ls_unsuccessful_count": "so_solver_bounded_ls_unsuccessful_count",
    "bounds_violation_count": "so_solver_bounds_violation_count",
    "nonfinite_solver_count": "so_solver_nonfinite_count",
    "selected_infeasible_count": "so_solver_selected_infeasible_count",
    "selected_solution_mismatch_count": "so_solver_selected_solution_mismatch_count",
    "residual_contract_mismatch_count": "so_solver_residual_contract_mismatch_count",
}


def classify_solver_journal(
    journal: Any, summary: Mapping[str, Any]
) -> dict[str, Any]:
    if not isinstance(journal, Sequence) or isinstance(journal, (str, bytes)):
        raise PostZeroPortGateError("solver journal must be an array")
    if len(journal) != contract.EXPECTED_STEPS:
        raise PostZeroPortGateError("solver journal must contain 500 policy steps")
    totals = {key: 0 for key in SUMMARY_COUNTERS}
    for step, raw_row in enumerate(journal, start=1):
        row = _mapping(raw_row, f"solver journal[{step}]")
        if set(row) != {"step", "time_s", "control_windows"}:
            raise PostZeroPortGateError(f"solver journal row {step} schema drifted")
        if _counter(row.get("step"), "solver journal step") != step:
            raise PostZeroPortGateError("solver journal steps are not contiguous")
        try:
            classified = so_recovery.classify_policy_step(
                row.get("control_windows"), policy_id=contract.SO_POLICY_ID
            )
        except so_recovery.SORecoveryContractError as exc:
            raise PostZeroPortGateError(str(exc)) from exc
        for key in totals:
            totals[key] += _counter(classified["counters"].get(key), key)
    data = _mapping(summary, "summary")
    for key, summary_key in SUMMARY_COUNTERS.items():
        if totals[key] != _counter(data.get(summary_key), summary_key):
            raise PostZeroPortGateError(f"summary counter drifted: {summary_key}")
    raw_so = _counter(data.get("so_fallback_count"), "so_fallback_count")
    raw_sea = _counter(
        data.get("sea_plugin_fallback_count"), "sea_plugin_fallback_count"
    )
    raw_total = _counter(data.get("fallback_count"), "fallback_count")
    checks = {
        "policy_id_exact": data.get("so_policy_id") == contract.SO_POLICY_ID,
        "control_windows_5000": totals["control_window_count"]
        == contract.EXPECTED_CONTROL_WINDOWS,
        "all_bounded_lsq_verified": totals["bounded_ls_invocation_count"]
        == totals["verified_bounded_ls_count"],
        "verified_partition_exact": totals["verified_bounded_ls_count"]
        == totals["verified_bounded_ls_success_count"]
        + totals["verified_status0_max_iter_count"],
        "raw_status0_signature_exact": totals["hard_so_fallback_count"]
        == totals["bounded_ls_unsuccessful_count"]
        == totals["verified_status0_max_iter_count"],
        "zero_unaccepted_hard": totals["unaccepted_hard_so_fallback_count"] == 0,
        "zero_unaccepted_bounded": totals["unaccepted_bounded_ls_count"] == 0,
        "zero_previous_reuse": totals["reuse_previous_count"] == 0,
        "zero_bounds_violation": totals["bounds_violation_count"] == 0,
        "zero_nonfinite": totals["nonfinite_solver_count"] == 0,
        "zero_selected_infeasible": totals["selected_infeasible_count"] == 0,
        "zero_selected_served_mismatch": totals[
            "selected_solution_mismatch_count"
        ]
        == 0,
        "zero_residual_contract_mismatch": totals[
            "residual_contract_mismatch_count"
        ]
        == 0,
        "zero_sea_fallback": raw_sea == 0,
        "raw_fallback_reconciled": raw_total == raw_so + raw_sea,
        "raw_policy_step_fallback_bounded": raw_so
        <= totals["verified_bounded_ls_count"],
    }
    return {
        "schema_version": 1,
        "status": (
            "PASS_VERIFIED_STATUS0_MAX_ITER_POLICY"
            if all(checks.values())
            else "FAIL_UNACCEPTED_SO_OR_SEA_FALLBACK"
        ),
        "passed": all(checks.values()),
        "so_policy_id": contract.SO_POLICY_ID,
        "checks": checks,
        "raw_fallback_count": raw_total,
        "raw_so_policy_step_fallback_count": raw_so,
        "raw_sea_fallback_count": raw_sea,
        "totals": totals,
    }


def _normalized_summary(summary: Mapping[str, Any], policy: Mapping[str, Any]) -> dict:
    result = dict(summary)
    if policy.get("passed") is True:
        result["fallback_count"] = 0
    return result


def common_rollout_gate(
    *, summary: Mapping[str, Any], solver_journal: Any
) -> dict[str, Any]:
    policy = classify_solver_journal(solver_journal, summary)
    common = legacy.common_rollout_checks(_normalized_summary(summary, policy))
    passed = policy["passed"] is True and all(
        item.get("status") == "PASS" for item in common
    )
    return {
        "status": "PASS_POST_ZERO_PORT_COMMON" if passed else "FAIL_POST_ZERO_PORT_COMMON",
        "passed": passed,
        "common_checks": common,
        "solver_policy_gate": policy,
    }


def compare_ab(
    *,
    a_trace: Sequence[Mapping[str, Any]],
    b_trace: Sequence[Mapping[str, Any]],
    a_summary: Mapping[str, Any],
    b_summary: Mapping[str, Any],
    a_v25_journal: Mapping[str, Any],
    b_v25_journal: Mapping[str, Any],
    a_solver_journal: Any,
    b_solver_journal: Any,
) -> dict[str, Any]:
    a_policy = classify_solver_journal(a_solver_journal, a_summary)
    b_policy = classify_solver_journal(b_solver_journal, b_summary)
    base = legacy.compare_ab(
        a_trace=a_trace,
        b_trace=b_trace,
        a_summary=_normalized_summary(a_summary, a_policy),
        b_summary=_normalized_summary(b_summary, b_policy),
        a_journal=a_v25_journal,
        b_journal=b_v25_journal,
    )
    solver_exact = canonical_json_bytes(a_solver_journal) == canonical_json_bytes(
        b_solver_journal
    )
    passed = bool(
        base.get("passed") is True
        and a_policy["passed"] is True
        and b_policy["passed"] is True
        and solver_exact
    )
    return {
        "status": "PASS_AB_SHADOW_NONINTERFERENCE" if passed else "ERROR_SHADOW_NONINTERFERENCE",
        "passed": passed,
        "legacy_projected_gate": base,
        "a_solver_policy_gate": a_policy,
        "b_solver_policy_gate": b_policy,
        "solver_journal_bit_exact": solver_exact,
        "a_solver_journal_sha256": payload_sha256(a_solver_journal),
        "b_solver_journal_sha256": payload_sha256(b_solver_journal),
    }


def gate_c(
    *,
    a_summary: Mapping[str, Any],
    c_summary: Mapping[str, Any],
    c_solver_journal: Any,
) -> dict[str, Any]:
    c_policy = classify_solver_journal(c_solver_journal, c_summary)
    base = legacy.gate_c(
        a_summary=a_summary,
        c_summary=_normalized_summary(c_summary, c_policy),
    )
    condition_match = a_summary.get("condition_id") == c_summary.get("condition_id")
    passed = bool(
        base.get("passed") is True
        and c_policy["passed"] is True
        and condition_match
        and c_summary.get("binary_phase_fsm_mode") == "binary_active"
        and c_summary.get("binary_phase_event_contract_id")
        == contract.V25_ACTIVE_CONTRACT_ID
    )
    return {
        "status": "PASS_H0_V25_COMPATIBLE_CONDITION" if passed else "FAIL_H0_V25_COMPATIBILITY",
        "passed": passed,
        "condition_matched": condition_match,
        "legacy_event_sea_reserve_gate": base,
        "c_solver_policy_gate": c_policy,
    }
