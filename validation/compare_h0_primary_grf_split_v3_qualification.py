"""Pure fail-closed gates for autonomous H0 V3 qualification.

No OpenSim, Ray, Torch, SciPy, or write path is imported here.  The comparator
validates each rollout independently and then compares the candidate only with
the exact condition-matched analog-H0 baseline frozen before candidate access.
"""

from __future__ import annotations

import math
import numbers
from typing import Any, Mapping

try:  # package import under unittest; direct import in validation workers.
    from validation import h0_primary_grf_split_v3_qualification_contract as contract
except ImportError:  # pragma: no cover - exercised by direct script execution.
    import h0_primary_grf_split_v3_qualification_contract as contract


class QualificationGateError(RuntimeError):
    """Raised when an artifact is missing, malformed, or non-finite."""


def _mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise QualificationGateError(f"{label} must be an object")
    return dict(value)


def _finite(value: Any, label: str, *, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise QualificationGateError(f"{label} must be a finite number")
    result = float(value)
    if not math.isfinite(result) or (minimum is not None and result < minimum):
        raise QualificationGateError(f"{label} is outside its finite domain")
    return result


def _counter(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, numbers.Integral):
        raise QualificationGateError(f"{label} must be a non-negative integer")
    result = int(value)
    if result < 0:
        raise QualificationGateError(f"{label} must be a non-negative integer")
    return result


def _check(name: str, actual: Any, expected: Any, passed: bool) -> dict[str, Any]:
    return {
        "name": name,
        "status": "PASS" if passed else "FAIL",
        "actual": actual,
        "expected": expected,
    }


def _case(case_id: str) -> dict[str, Any]:
    matches = [row for row in contract.canonical_cases() if row["case_id"] == case_id]
    if len(matches) != 1:
        raise QualificationGateError(f"unknown canonical case {case_id!r}")
    return matches[0]


def _metric(value: Any, label: str, *, expected_samples: int | None) -> dict[str, Any]:
    data = _mapping(value, label)
    if set(data) != {"sample_count", "rms", "abs_max"}:
        raise QualificationGateError(f"{label} metric schema drifted")
    samples = _counter(data["sample_count"], f"{label}.sample_count")
    if expected_samples is not None and samples != expected_samples:
        raise QualificationGateError(
            f"{label}.sample_count={samples}, expected {expected_samples}"
        )
    rms = _finite(data["rms"], f"{label}.rms", minimum=0.0)
    maximum = _finite(data["abs_max"], f"{label}.abs_max", minimum=0.0)
    if maximum + 1.0e-15 < rms:
        raise QualificationGateError(f"{label}.abs_max is below rms")
    return {"sample_count": samples, "rms": rms, "abs_max": maximum}


def _nested_count(summary: Mapping[str, Any], path: str, label: str) -> int:
    if ".tau_input_saturated.count" in path:
        joint = path.split(".", 1)[0]
        sea = _mapping(summary.get("sea_episode_metrics"), f"{label}.sea")
        joint_data = _mapping(sea.get(joint), f"{label}.sea.{joint}")
        saturation = _mapping(
            joint_data.get("tau_input_saturated"), f"{label}.{joint}.saturation"
        )
        return _counter(saturation.get("count"), f"{label}.{path}")
    return _counter(summary.get(path), f"{label}.{path}")


def _validate_metrics(summary: Mapping[str, Any], label: str) -> None:
    episode = _mapping(summary.get("episode_metrics"), f"{label}.episode_metrics")
    for name in ("reserve_norm_nm", "residual_norm_nm"):
        _metric(episode.get(name), f"{label}.{name}", expected_samples=contract.EXPECTED_STEPS)

    sea = _mapping(summary.get("sea_episode_metrics"), f"{label}.sea_episode_metrics")
    if set(sea) != set(contract.JOINTS):
        raise QualificationGateError(f"{label} SEA joint schema drifted")
    for joint in contract.JOINTS:
        joint_data = _mapping(sea[joint], f"{label}.{joint}")
        for signal in contract.SEA_SIGNALS:
            metric = _metric(
                joint_data.get(signal),
                f"{label}.{joint}.{signal}",
                expected_samples=None,
            )
            if metric["sample_count"] <= 0:
                raise QualificationGateError(f"{label}.{joint}.{signal} is empty")
        saturation = _mapping(
            joint_data.get("tau_input_saturated"), f"{label}.{joint}.saturation"
        )
        if set(saturation) != {"sample_count", "count", "fraction"}:
            raise QualificationGateError(f"{label}.{joint} saturation schema drifted")
        denominator = _counter(
            saturation["sample_count"], f"{label}.{joint}.saturation.sample_count"
        )
        count = _counter(saturation["count"], f"{label}.{joint}.saturation.count")
        fraction = _finite(
            saturation["fraction"], f"{label}.{joint}.saturation.fraction", minimum=0.0
        )
        if (
            denominator <= 0
            or count > denominator
            or fraction > 1.0
            or not math.isclose(
                fraction, count / denominator, rel_tol=0.0, abs_tol=1.0e-15
            )
        ):
            raise QualificationGateError(f"{label}.{joint} saturation is inconsistent")


def common_rollout_gate(
    summary: Mapping[str, Any], *, role: str, case_id: str
) -> dict[str, Any]:
    """Apply fixed V3 physical/layout and full-window SO gates."""

    data = _mapping(summary, f"{role}/{case_id} summary")
    if role not in {"baseline", "candidate"}:
        raise QualificationGateError(f"unknown role {role!r}")
    case = _case(case_id)
    checks: list[dict[str, Any]] = []

    identity = {
        "case_id": case_id,
        "action_selection": case["action_selection"],
        "episode_start_offset_s": case["episode_start_offset_s"],
        "action_seed": case["action_seed"],
        "runtime_seed": case["runtime_seed"],
        "sigma": case["sigma"],
    }
    for name, expected in identity.items():
        actual = data.get(name)
        if isinstance(expected, float):
            actual_number = _finite(actual, f"{role}.{name}")
            passed = math.isclose(actual_number, expected, rel_tol=0.0, abs_tol=1.0e-12)
            actual = actual_number
        else:
            passed = actual == expected
        checks.append(_check(name, actual, expected, passed))

    fixed = (
        ("steps", _counter(data.get("steps"), f"{role}.steps"), contract.EXPECTED_STEPS),
        ("end_reason", data.get("end_reason"), "episode_time_limit"),
        ("terminated", data.get("terminated"), False),
        ("truncated", data.get("truncated"), True),
        ("n_actor", data.get("n_actor"), contract.EXPECTED_ACTOR_FEATURES),
        ("n_observation", data.get("n_observation"), contract.EXPECTED_FULL_FEATURES),
        ("observation_dtype", data.get("observation_dtype"), contract.EXPECTED_DTYPE),
        ("action_shape", data.get("action_shape"), list(contract.EXPECTED_ACTION_SHAPE)),
        ("action_dtype", data.get("action_dtype"), contract.EXPECTED_DTYPE),
        ("event_contract_id", data.get("event_contract_id"), contract.EVENT_CONTRACT_ID),
        ("phase_fsm_input_mode", data.get("phase_fsm_input_mode"), contract.PHASE_FSM_INPUT_MODE),
        ("binary_phase_fsm_mode", data.get("binary_phase_fsm_mode"), "disabled"),
        ("online_grf_applied_sides", data.get("online_grf_applied_sides"), ["left"]),
        ("morphology_weight", _finite(data.get("morphology_weight"), f"{role}.morphology_weight"), 0.0),
        ("so_policy_id", data.get("so_policy_id"), contract.SO_POLICY_ID),
        ("actor_updates", data.get("actor_updates"), 0),
        ("critic_updates", data.get("critic_updates"), 0),
        ("ppo_updates", data.get("ppo_updates"), 0),
        ("protected_trials_opened", data.get("protected_trials_opened"), []),
    )
    for name, actual, expected in fixed:
        checks.append(_check(name, actual, expected, actual == expected))

    expected_view = "historical_analog" if role == "baseline" else "primary_split"
    checks.append(
        _check("actor_input_view", data.get("actor_input_view"), expected_view,
               data.get("actor_input_view") == expected_view)
    )
    cycles = _counter(data.get("phase_valid_cycle_count"), f"{role}.cycles")
    checks.append(_check("phase_valid_cycle_count", cycles, ">=2", cycles >= 2))
    penetration = _finite(
        data.get("grf_penetration_max_m"), f"{role}.penetration", minimum=0.0
    )
    checks.append(
        _check("grf_penetration_max_m", penetration, "<0.025",
               penetration < contract.PENETRATION_LIMIT_M)
    )
    for name in contract.ZERO_REQUIRED_COUNTS:
        value = _counter(data.get(name), f"{role}.{name}")
        checks.append(_check(name, value, 0, value == 0))

    control_windows = _counter(
        data.get("so_solver_control_window_count"), f"{role}.control_windows"
    )
    invocations = _counter(
        data.get("so_solver_bounded_ls_invocation_count"), f"{role}.bounded_invocations"
    )
    verified = _counter(
        data.get("so_solver_verified_bounded_ls_count"), f"{role}.bounded_verified"
    )
    raw_fallbacks = _counter(data.get("raw_so_fallback_count"), f"{role}.raw_fallbacks")
    status0 = _counter(
        data.get("so_solver_verified_status0_max_iter_count"), f"{role}.status0"
    )
    unsuccessful = _counter(
        data.get("so_solver_bounded_ls_unsuccessful_count"), f"{role}.unsuccessful"
    )
    hard = _counter(data.get("so_solver_hard_fallback_count"), f"{role}.hard")
    checks.extend(
        (
            _check("so_solver_control_window_count", control_windows,
                   contract.EXPECTED_CONTROL_WINDOWS,
                   control_windows == contract.EXPECTED_CONTROL_WINDOWS),
            _check("all_bounded_lsq_verified", verified, invocations,
                   verified == invocations),
            _check("raw_so_fallback_count_traced", raw_fallbacks, invocations,
                   raw_fallbacks == invocations),
            _check("status0_accounts_for_unsuccessful", unsuccessful, status0,
                   unsuccessful == status0),
            _check("status0_accounts_for_hard", hard, status0, hard == status0),
        )
    )
    _counter(
        data.get("policy_step_terminal_so_fallback_count"),
        f"{role}.policy_step_terminal_so_fallback_count",
    )
    _validate_metrics(data, role)
    passed = all(check["status"] == "PASS" for check in checks)
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            "PASS_H0_PRIMARY_SPLIT_V3_QUALIFICATION_ROLLOUT"
            if passed
            else "FAIL_H0_PRIMARY_SPLIT_V3_QUALIFICATION_ROLLOUT"
        ),
        "passed": passed,
        "role": role,
        "case_id": case_id,
        "checks": checks,
    }


def _continuous_values(summary: Mapping[str, Any]) -> tuple[dict[str, float], dict[str, float]]:
    data = _mapping(summary, "summary")
    episode = _mapping(data.get("episode_metrics"), "episode_metrics")
    reserve: dict[str, float] = {}
    for metric in ("reserve_norm_nm", "residual_norm_nm"):
        values = _metric(episode.get(metric), metric, expected_samples=contract.EXPECTED_STEPS)
        for aggregation in contract.CONTINUOUS_AGGREGATIONS:
            reserve[f"{metric}.{aggregation}"] = float(values[aggregation])
    sea_values: dict[str, float] = {}
    sea = _mapping(data.get("sea_episode_metrics"), "sea_episode_metrics")
    for joint in contract.JOINTS:
        joint_data = _mapping(sea.get(joint), joint)
        for signal in contract.SEA_SIGNALS:
            metric = _metric(joint_data.get(signal), f"{joint}.{signal}", expected_samples=None)
            for aggregation in contract.CONTINUOUS_AGGREGATIONS:
                sea_values[f"{joint}.{signal}.{aggregation}"] = float(metric[aggregation])
    return sea_values, reserve


def baseline_case_metrics(summary: Mapping[str, Any]) -> dict[str, dict[str, float]]:
    """Return the exact ordered schema consumed by the scaffold receipt."""

    sea, reserve = _continuous_values(summary)
    expected_sea = [row[0] for row in contract.SEA_TOLERANCES]
    expected_reserve = [row[0] for row in contract.RESERVE_TOLERANCES]
    if list(sea) != expected_sea or list(reserve) != expected_reserve:
        raise QualificationGateError("baseline metric order drifted")
    return {"sea": sea, "reserve": reserve}


def condition_matched_gate(
    baseline: Mapping[str, Any], candidate: Mapping[str, Any], *, case_id: str
) -> dict[str, Any]:
    baseline_data = _mapping(baseline, "baseline")
    candidate_data = _mapping(candidate, "candidate")
    baseline_common = common_rollout_gate(baseline_data, role="baseline", case_id=case_id)
    candidate_common = common_rollout_gate(candidate_data, role="candidate", case_id=case_id)
    checks: list[dict[str, Any]] = []

    for name in (
        "case_id",
        "action_selection",
        "episode_start_offset_s",
        "action_seed",
        "runtime_seed",
        "sigma",
        "noise_tape_sha256",
    ):
        actual = candidate_data.get(name)
        expected = baseline_data.get(name)
        checks.append(_check(f"condition.{name}", actual, expected, actual == expected))

    baseline_sea, baseline_reserve = _continuous_values(baseline_data)
    candidate_sea, candidate_reserve = _continuous_values(candidate_data)
    for family, rows, reference, observed in (
        ("sea", contract.SEA_TOLERANCES, baseline_sea, candidate_sea),
        ("reserve", contract.RESERVE_TOLERANCES, baseline_reserve, candidate_reserve),
    ):
        for metric, absolute, relative in rows:
            baseline_value = reference[metric]
            candidate_value = observed[metric]
            tolerance = max(float(absolute), float(relative) * abs(baseline_value))
            cap = baseline_value + tolerance
            item = _check(
                f"{family}.{metric}", candidate_value, f"<= {cap}", candidate_value <= cap
            )
            item.update({"baseline": baseline_value, "tolerance": tolerance, "cap": cap})
            checks.append(item)

    for name in contract.NONINCREASING_COUNTS:
        baseline_count = _nested_count(baseline_data, name, "baseline")
        candidate_count = _nested_count(candidate_data, name, "candidate")
        checks.append(
            _check(f"count.{name}", candidate_count, f"<= {baseline_count}",
                   candidate_count <= baseline_count)
        )

    passed = (
        baseline_common["passed"] is True
        and candidate_common["passed"] is True
        and all(item["status"] == "PASS" for item in checks)
    )
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            "PASS_H0_PRIMARY_SPLIT_V3_QUALIFICATION_CASE"
            if passed
            else "FAIL_H0_PRIMARY_SPLIT_V3_AUTONOMOUS_QUALIFICATION"
        ),
        "passed": passed,
        "case_id": case_id,
        "baseline_common_gate": baseline_common,
        "candidate_common_gate": candidate_common,
        "condition_matched_checks": checks,
    }


__all__ = [
    "QualificationGateError",
    "baseline_case_metrics",
    "common_rollout_gate",
    "condition_matched_gate",
]
