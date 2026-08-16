"""Fail-closed classification for V3 static-optimization solver telemetry."""

from __future__ import annotations

import math
import numbers
from typing import Any, Mapping, Sequence


SCHEMA = "static_optimization_solver_audit_v1"
STRICT_ZERO_POLICY = "strict_zero_all_fallbacks_v1"
VERIFIED_SUCCESS_POLICY = "verified_historical_bounded_ls_success_v1"
VERIFIED_STATUS0_MAX_ITER_POLICY = "verified_status0_max_iter_v1"
SUPPORTED_POLICY_IDS = frozenset(
    {
        STRICT_ZERO_POLICY,
        VERIFIED_SUCCESS_POLICY,
        VERIFIED_STATUS0_MAX_ITER_POLICY,
    }
)
BOUND_TOLERANCE = 1.0e-9
FEASIBILITY_ABS_TOLERANCE = 1.0e-6
FEASIBILITY_REL_TOLERANCE = 1.0e-3
RESIDUAL_CONSISTENCY_ATOL = 1.0e-12
BOUNDED_LSQ_STATUS0 = 0
BOUNDED_LSQ_MAX_ITER = 1000
BOUNDED_LSQ_MAX_ITER_MESSAGE = "The maximum number of iterations is exceeded."
BOUNDED_LSQ_STATUS0_OPTIMALITY_MAX = 1.0e-8


class SORecoveryContractError(RuntimeError):
    """Raised when solver telemetry is incomplete, malformed, or unsafe."""


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise SORecoveryContractError(f"{label} must be an object")
    return value


def _bool(value: Any, label: str) -> bool:
    if type(value) is not bool:
        raise SORecoveryContractError(f"{label} must be a Python bool")
    return value


def _int(value: Any, label: str, *, minimum: int = 0) -> int:
    if type(value) is not int or value < minimum:
        raise SORecoveryContractError(f"{label} must be an integer >= {minimum}")
    return value


def _signed_int(value: Any, label: str) -> int:
    if type(value) is not int:
        raise SORecoveryContractError(f"{label} must be an integer")
    return value


def _finite(value: Any, label: str, *, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise SORecoveryContractError(f"{label} must be finite")
    number = float(value)
    if not math.isfinite(number) or (minimum is not None and number < minimum):
        raise SORecoveryContractError(f"{label} must be finite and >= {minimum}")
    return number


def _digest(value: Any, label: str, *, required: bool) -> str | None:
    if value is None and not required:
        return None
    if (
        not isinstance(value, str)
        or len(value) != 64
        or any(char not in "0123456789abcdef" for char in value)
    ):
        raise SORecoveryContractError(f"{label} must be a lowercase SHA-256")
    return value


def _solution(value: Any, label: str) -> dict[str, Any]:
    payload = _mapping(value, label)
    shape_matches = _bool(
        payload.get("output_shape_matches"), f"{label}.output_shape_matches"
    )
    output_finite = _bool(payload.get("output_finite"), f"{label}.output_finite")
    residual_finite = _bool(
        payload.get("equality_residual_finite"),
        f"{label}.equality_residual_finite",
    )
    digest = _digest(
        payload.get("output_sha256"),
        f"{label}.output_sha256",
        required=output_finite,
    )
    bound_violation = (
        _finite(
            payload.get("bound_violation_max"),
            f"{label}.bound_violation_max",
            minimum=0.0,
        )
        if output_finite and shape_matches
        else None
    )
    residual_norm = (
        _finite(
            payload.get("equality_residual_norm"),
            f"{label}.equality_residual_norm",
            minimum=0.0,
        )
        if residual_finite
        else None
    )
    residual_max_abs = (
        _finite(
            payload.get("equality_residual_max_abs"),
            f"{label}.equality_residual_max_abs",
            minimum=0.0,
        )
        if residual_finite
        else None
    )
    residual_relative = (
        _finite(
            payload.get("equality_residual_relative_norm"),
            f"{label}.equality_residual_relative_norm",
            minimum=0.0,
        )
        if residual_finite
        else None
    )
    return {
        "output_shape_matches": shape_matches,
        "output_finite": output_finite,
        "output_sha256": digest,
        "bound_violation_max": bound_violation,
        "equality_residual_finite": residual_finite,
        "equality_residual_norm": residual_norm,
        "equality_residual_max_abs": residual_max_abs,
        "equality_residual_relative_norm": residual_relative,
    }


def classify_policy_step(
    raw_entries: Any,
    *,
    policy_id: str = VERIFIED_SUCCESS_POLICY,
) -> dict[str, Any]:
    """Validate every control-window/attempt and return policy-aware counters.

    Raw solver flags and counters are never rewritten.  The status-0 policy
    only adds an independent acceptance classification for the exact frozen
    max-iteration signature; all other unsuccessful solver outcomes remain
    unaccepted.
    """
    if policy_id not in SUPPORTED_POLICY_IDS:
        raise SORecoveryContractError(f"unsupported SO recovery policy: {policy_id!r}")
    status0_policy = policy_id == VERIFIED_STATUS0_MAX_ITER_POLICY
    if isinstance(raw_entries, (str, bytes)) or not isinstance(raw_entries, Sequence):
        raise SORecoveryContractError("so_solver_audit_entries must be an array")
    if not raw_entries:
        raise SORecoveryContractError("solver audit must contain a control window")

    counters = {
        "control_window_count": 0,
        "solver_invocation_count": 0,
        "primary_solver_nonconvergence_count": 0,
        "bounded_ls_invocation_count": 0,
        "selected_bounded_ls_count": 0,
        "verified_bounded_ls_count": 0,
        "verified_bounded_ls_success_count": 0,
        "verified_status0_max_iter_count": 0,
        "unaccepted_hard_so_fallback_count": 0,
        "unaccepted_bounded_ls_count": 0,
        "hard_so_fallback_count": 0,
        "reuse_previous_count": 0,
        "bounded_ls_unsuccessful_count": 0,
        "bounds_violation_count": 0,
        "nonfinite_solver_count": 0,
        "selected_infeasible_count": 0,
        "selected_solution_mismatch_count": 0,
        "residual_contract_mismatch_count": 0,
    }
    normalized: list[dict[str, Any]] = []
    previous_time = -math.inf
    for window_position, raw_window in enumerate(raw_entries, start=1):
        label = f"window[{window_position}]"
        window = _mapping(raw_window, label)
        window_index = _int(
            window.get("control_window_index"),
            f"{label}.control_window_index",
            minimum=1,
        )
        if window_index != window_position:
            raise SORecoveryContractError("control-window indices are not contiguous")
        time_s = _finite(
            window.get("control_window_time_s"),
            f"{label}.control_window_time_s",
        )
        if time_s <= previous_time:
            raise SORecoveryContractError("control-window times are not monotonic")
        previous_time = time_s
        selected_index = _int(
            window.get("selected_feasibility_attempt_index"),
            f"{label}.selected_feasibility_attempt_index",
            minimum=1,
        )
        served_digest = _digest(
            window.get("served_solution_sha256"),
            f"{label}.served_solution_sha256",
            required=True,
        )
        solution_matches = _bool(
            window.get("selected_solver_solution_matches_served"),
            f"{label}.selected_solver_solution_matches_served",
        )
        if not solution_matches:
            counters["selected_solution_mismatch_count"] += 1
        attempts = window.get("attempts")
        if isinstance(attempts, (str, bytes)) or not isinstance(attempts, Sequence):
            raise SORecoveryContractError(f"{label}.attempts must be an array")
        if not attempts:
            raise SORecoveryContractError(f"{label}.attempts must not be empty")

        selected_seen = 0
        normalized_attempts: list[dict[str, Any]] = []
        for attempt_position, raw_attempt in enumerate(attempts, start=1):
            attempt_label = f"{label}.attempt[{attempt_position}]"
            attempt = _mapping(raw_attempt, attempt_label)
            attempt_index = _int(
                attempt.get("attempt_index"),
                f"{attempt_label}.attempt_index",
                minimum=1,
            )
            if attempt_index != attempt_position:
                raise SORecoveryContractError(
                    "solver-attempt indices are not contiguous"
                )
            selected = _bool(attempt.get("selected"), f"{attempt_label}.selected")
            fallback_used = _bool(
                attempt.get("solver_fallback_used"),
                f"{attempt_label}.solver_fallback_used",
            )
            feasibility_accepted = _bool(
                attempt.get("feasibility_accepted"),
                f"{attempt_label}.feasibility_accepted",
            )
            scale = _finite(
                attempt.get("feasibility_scale"),
                f"{attempt_label}.feasibility_scale",
                minimum=0.0,
            )
            residual_norm = _finite(
                attempt.get("residual_norm"),
                f"{attempt_label}.residual_norm",
                minimum=0.0,
            )
            residual_relative = _finite(
                attempt.get("residual_relative_norm"),
                f"{attempt_label}.residual_relative_norm",
                minimum=0.0,
            )
            residual_max_abs = _finite(
                attempt.get("residual_max_abs"),
                f"{attempt_label}.residual_max_abs",
                minimum=0.0,
            )
            path = _mapping(attempt.get("solver_path"), f"{attempt_label}.solver_path")
            if path.get("schema") != SCHEMA:
                raise SORecoveryContractError("solver audit schema drifted")
            primary_solver = path.get("primary_solver")
            if primary_solver != "slsqp":
                raise SORecoveryContractError("V3 requires the frozen SLSQP solver")
            finite_flags = [
                _bool(path.get(field), f"{attempt_label}.solver_path.{field}")
                for field in (
                    "input_matrix_finite",
                    "input_target_finite",
                    "weights_finite",
                    "bounds_finite",
                    "warm_start_finite",
                )
            ]
            for field, flag in zip(
                (
                    "input_matrix_sha256",
                    "input_target_sha256",
                    "weights_sha256",
                    "bounds_sha256",
                    "warm_start_sha256",
                ),
                finite_flags,
            ):
                _digest(
                    path.get(field),
                    f"{attempt_label}.solver_path.{field}",
                    required=flag,
                )
            slsqp_invoked = _bool(
                path.get("slsqp_invoked"),
                f"{attempt_label}.solver_path.slsqp_invoked",
            )
            slsqp_success = _bool(
                path.get("slsqp_success"),
                f"{attempt_label}.solver_path.slsqp_success",
            )
            slsqp_solution = None
            if slsqp_invoked:
                _signed_int(
                    path.get("slsqp_status"),
                    f"{attempt_label}.solver_path.slsqp_status",
                )
                _int(
                    path.get("slsqp_iterations"),
                    f"{attempt_label}.solver_path.slsqp_iterations",
                    minimum=0,
                )
                slsqp_solution = _solution(
                    path.get("slsqp_solution"),
                    f"{attempt_label}.solver_path.slsqp_solution",
                )
            bounded_used = _bool(
                path.get("bounded_lsq_used"),
                f"{attempt_label}.solver_path.bounded_lsq_used",
            )
            reuse_previous = _bool(
                path.get("reuse_previous_solution"),
                f"{attempt_label}.solver_path.reuse_previous_solution",
            )
            hard_fallback = _bool(
                path.get("hard_fallback"),
                f"{attempt_label}.solver_path.hard_fallback",
            )
            selected_solution = _solution(
                path.get("selected_solution"),
                f"{attempt_label}.solver_path.selected_solution",
            )
            residual_contract_matches = bool(
                selected_solution["equality_residual_finite"]
                and abs(residual_norm - selected_solution["equality_residual_norm"])
                <= RESIDUAL_CONSISTENCY_ATOL
                and abs(
                    residual_relative
                    - selected_solution["equality_residual_relative_norm"]
                )
                <= RESIDUAL_CONSISTENCY_ATOL
                and abs(
                    residual_max_abs - selected_solution["equality_residual_max_abs"]
                )
                <= RESIDUAL_CONSISTENCY_ATOL
            )
            recomputed_feasible = bool(
                residual_norm <= FEASIBILITY_ABS_TOLERANCE
                or residual_max_abs <= FEASIBILITY_ABS_TOLERANCE
                or residual_relative <= FEASIBILITY_REL_TOLERANCE
            )
            residual_contract_matches = bool(
                residual_contract_matches
                and feasibility_accepted is recomputed_feasible
            )
            if not residual_contract_matches:
                counters["residual_contract_mismatch_count"] += 1
            if selected and selected_solution["output_sha256"] != served_digest:
                counters["selected_solution_mismatch_count"] += 1

            bounded_invoked = False
            bounded_success = False
            bounded_status = None
            bounded_iterations = None
            bounded_message = None
            bounded_optimality = None
            bounded_solution = None
            if bounded_used:
                bounded_invoked = _bool(
                    path.get("bounded_lsq_invoked"),
                    f"{attempt_label}.solver_path.bounded_lsq_invoked",
                )
                bounded_success = _bool(
                    path.get("bounded_lsq_success"),
                    f"{attempt_label}.solver_path.bounded_lsq_success",
                )
                if bounded_invoked:
                    bounded_status = _signed_int(
                        path.get("bounded_lsq_status"),
                        f"{attempt_label}.solver_path.bounded_lsq_status",
                    )
                    bounded_iterations = _int(
                        path.get("bounded_lsq_iterations"),
                        f"{attempt_label}.solver_path.bounded_lsq_iterations",
                        minimum=0,
                    )
                    bounded_message = path.get("bounded_lsq_message")
                    if not isinstance(bounded_message, str) or not bounded_message:
                        raise SORecoveryContractError(
                            f"{attempt_label}.solver_path.bounded_lsq_message "
                            "must be a non-empty string"
                        )
                    _finite(
                        path.get("bounded_lsq_cost"),
                        f"{attempt_label}.solver_path.bounded_lsq_cost",
                        minimum=0.0,
                    )
                    bounded_optimality = _finite(
                        path.get("bounded_lsq_optimality"),
                        f"{attempt_label}.solver_path.bounded_lsq_optimality",
                        minimum=0.0,
                    )
                bounded_solution = _solution(
                    path.get("bounded_lsq_solution"),
                    f"{attempt_label}.solver_path.bounded_lsq_solution",
                )

            nonfinite = bool(
                not all(finite_flags)
                or slsqp_solution is None
                or not slsqp_solution["output_finite"]
                or not selected_solution["output_finite"]
                or not selected_solution["equality_residual_finite"]
                or (
                    bounded_solution is not None
                    and (
                        not bounded_solution["output_finite"]
                        or not bounded_solution["equality_residual_finite"]
                    )
                )
            )
            bounds_violation = bool(
                selected_solution["bound_violation_max"] is None
                or selected_solution["bound_violation_max"] > BOUND_TOLERANCE
                or (
                    bounded_solution is not None
                    and (
                        bounded_solution["bound_violation_max"] is None
                        or bounded_solution["bound_violation_max"] > BOUND_TOLERANCE
                    )
                )
            )
            common_bounded_contract = bool(
                bounded_used
                and fallback_used
                and bounded_invoked
                and not reuse_previous
                and not nonfinite
                and not bounds_violation
                and residual_contract_matches
                and feasibility_accepted
                and recomputed_feasible
                and selected
                and solution_matches
                and selected_solution["output_shape_matches"]
                and bounded_solution is not None
                and bounded_solution["output_shape_matches"]
                and bounded_solution["output_sha256"]
                == selected_solution["output_sha256"]
                == served_digest
            )
            verified_bounded_success = bool(
                common_bounded_contract
                and bounded_success
                and not hard_fallback
            )
            verified_status0_max_iter = bool(
                common_bounded_contract
                and not bounded_success
                and hard_fallback
                and bounded_status == BOUNDED_LSQ_STATUS0
                and bounded_iterations == BOUNDED_LSQ_MAX_ITER
                and bounded_message == BOUNDED_LSQ_MAX_ITER_MESSAGE
                and bounded_optimality is not None
                and bounded_optimality <= BOUNDED_LSQ_STATUS0_OPTIMALITY_MAX
            )
            verified_bounded = bool(
                (
                    policy_id != STRICT_ZERO_POLICY
                    and verified_bounded_success
                )
                or (status0_policy and verified_status0_max_iter)
            )
            unaccepted_hard = bool(hard_fallback and not verified_bounded)
            unaccepted_bounded = bool(bounded_invoked and not verified_bounded)

            counters["solver_invocation_count"] += 1
            counters["primary_solver_nonconvergence_count"] += int(
                slsqp_invoked and not slsqp_success
            )
            counters["bounded_ls_invocation_count"] += int(bounded_invoked)
            counters["selected_bounded_ls_count"] += int(selected and bounded_used)
            counters["verified_bounded_ls_count"] += int(verified_bounded)
            counters["verified_bounded_ls_success_count"] += int(
                verified_bounded_success
            )
            counters["verified_status0_max_iter_count"] += int(
                verified_status0_max_iter
            )
            counters["unaccepted_hard_so_fallback_count"] += int(unaccepted_hard)
            counters["unaccepted_bounded_ls_count"] += int(unaccepted_bounded)
            counters["hard_so_fallback_count"] += int(hard_fallback)
            counters["reuse_previous_count"] += int(reuse_previous)
            counters["bounded_ls_unsuccessful_count"] += int(
                bounded_used and not bounded_success
            )
            counters["bounds_violation_count"] += int(bounds_violation)
            counters["nonfinite_solver_count"] += int(nonfinite)
            counters["selected_infeasible_count"] += int(
                selected and not feasibility_accepted
            )
            selected_seen += int(selected)
            normalized_attempts.append(
                {
                    "attempt_index": attempt_index,
                    "selected": selected,
                    "feasibility_scale": scale,
                    "feasibility_accepted": feasibility_accepted,
                    "fallback_used": fallback_used,
                    "bounded_lsq_used": bounded_used,
                    "verified_bounded_lsq": verified_bounded,
                    "verified_bounded_lsq_success": verified_bounded_success,
                    "verified_status0_max_iter": verified_status0_max_iter,
                    "accepted_by_selected_policy": verified_bounded,
                    "hard_fallback": hard_fallback,
                    "reuse_previous_solution": reuse_previous,
                    "residual_contract_matches": residual_contract_matches,
                    "selected_solution_sha256": selected_solution["output_sha256"],
                }
            )

        if selected_index > len(normalized_attempts):
            raise SORecoveryContractError(
                "selected solver-attempt index is out of range"
            )
        if (
            selected_seen != 1
            or not normalized_attempts[selected_index - 1]["selected"]
        ):
            raise SORecoveryContractError(
                "each control window must identify exactly one selected attempt"
            )
        counters["control_window_count"] += 1
        normalized.append(
            {
                "control_window_index": window_index,
                "control_window_time_s": time_s,
                "selected_feasibility_attempt_index": selected_index,
                "served_solution_sha256": served_digest,
                "attempts": normalized_attempts,
            }
        )

    counters["all_bounded_ls_verified"] = bool(
        counters["bounded_ls_invocation_count"] == counters["verified_bounded_ls_count"]
    )
    counters["hard_conditions_zero"] = bool(
        counters["unaccepted_hard_so_fallback_count"] == 0
        and counters["unaccepted_bounded_ls_count"] == 0
        and counters["reuse_previous_count"] == 0
        and counters["bounds_violation_count"] == 0
        and counters["nonfinite_solver_count"] == 0
        and counters["selected_infeasible_count"] == 0
        and counters["selected_solution_mismatch_count"] == 0
        and counters["residual_contract_mismatch_count"] == 0
    )
    return {
        "policy_id": policy_id,
        "counters": counters,
        "entries": normalized,
    }
