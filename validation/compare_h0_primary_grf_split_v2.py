"""Fail-closed gates for the prescribed-teacher H0 primary-split V2 branch."""

from __future__ import annotations

import math
from typing import Any, Mapping

import compare_h0_primary_grf_split as v1


COUNTERFACTUAL_RMSE_LIMIT = 0.03
COUNTERFACTUAL_MAX_ABS_LIMIT = 0.15
RELATIVE_TOLERANCE = 0.15
RESERVE_ABSOLUTE_TOLERANCE = 25.0
RESIDUAL_ABSOLUTE_TOLERANCE = 1.0e-6
SEA_ABSOLUTE_TOLERANCE = 1.0e-6

JOINTS = v1.JOINTS
SEA_SIGNALS = v1.SEA_SIGNALS

canonical_json_bytes = v1.canonical_json_bytes
payload_sha256 = v1.payload_sha256
strict_json_load = v1.strict_json_load
write_json_exclusive = v1.write_json_exclusive
common_rollout_gate = v1.common_rollout_gate


class H0PrimarySplitV2GateError(RuntimeError):
    """Raised when a V2 gate payload is malformed."""


def _mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise H0PrimarySplitV2GateError(f"{label} must be an object")
    return dict(value)


def _finite(value: Any, label: str, *, minimum: float | None = None) -> float:
    if isinstance(value, bool):
        raise H0PrimarySplitV2GateError(f"{label} must be numeric")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise H0PrimarySplitV2GateError(f"{label} must be numeric") from exc
    if not math.isfinite(number) or (minimum is not None and number < minimum):
        raise H0PrimarySplitV2GateError(f"{label} is invalid")
    return number


def _integer(value: Any, label: str) -> int:
    number = _finite(value, label, minimum=0.0)
    if not number.is_integer():
        raise H0PrimarySplitV2GateError(f"{label} must be an integer")
    return int(number)


def _check(name: str, actual: Any, expected: Any, passed: bool) -> dict[str, Any]:
    return {
        "name": name,
        "actual": actual,
        "expected": expected,
        "status": "PASS" if passed else "FAIL",
    }


def prescribed_counterfactual_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    data = _mapping(
        summary.get("prescribed_counterfactual_error"),
        "candidate.prescribed_counterfactual_error",
    )
    samples = _integer(data.get("sample_count"), "counterfactual.sample_count")
    rmse = _finite(data.get("rmse"), "counterfactual.rmse", minimum=0.0)
    maximum = _finite(
        data.get("max_abs_error"), "counterfactual.max_abs_error", minimum=0.0
    )
    dependency = _integer(
        data.get("served_action_teacher_dependency_count"),
        "counterfactual.served_action_teacher_dependency_count",
    )
    selected_first = data.get("action_selected_before_teacher_diagnostic")
    checks = [
        _check(
            "sample_count", samples, v1.EXPECTED_STEPS, samples == v1.EXPECTED_STEPS
        ),
        _check(
            "rmse",
            rmse,
            f"<={COUNTERFACTUAL_RMSE_LIMIT}",
            rmse <= COUNTERFACTUAL_RMSE_LIMIT,
        ),
        _check(
            "max_abs_error",
            maximum,
            f"<={COUNTERFACTUAL_MAX_ABS_LIMIT}",
            maximum <= COUNTERFACTUAL_MAX_ABS_LIMIT,
        ),
        _check(
            "served_action_teacher_dependency_count", dependency, 0, dependency == 0
        ),
        _check(
            "action_selected_before_teacher_diagnostic",
            selected_first,
            True,
            selected_first is True,
        ),
    ]
    passed = all(item["status"] == "PASS" for item in checks)
    return {
        "status": "PASS_PRESCRIBED_COUNTERFACTUAL"
        if passed
        else "FAIL_PRESCRIBED_COUNTERFACTUAL",
        "passed": passed,
        "checks": checks,
    }


def _metric(payload: Any, label: str) -> dict[str, float | int]:
    data = _mapping(payload, label)
    return {
        "rms": _finite(data.get("rms"), f"{label}.rms", minimum=0.0),
        "abs_max": _finite(data.get("abs_max"), f"{label}.abs_max", minimum=0.0),
        "sample_count": _integer(data.get("sample_count"), f"{label}.sample_count"),
    }


def _cap(reference: float, absolute_tolerance: float) -> float:
    return reference + max(RELATIVE_TOLERANCE * reference, absolute_tolerance)


def _nonregression(
    reference: Mapping[str, Any], candidate: Mapping[str, Any]
) -> dict[str, Any]:
    checks: list[dict[str, Any]] = []
    ref_episode = _mapping(
        reference.get("episode_metrics"), "reference.episode_metrics"
    )
    cand_episode = _mapping(
        candidate.get("episode_metrics"), "candidate.episode_metrics"
    )
    for name, tolerance in (
        ("reserve_norm_nm", RESERVE_ABSOLUTE_TOLERANCE),
        ("residual_norm_nm", RESIDUAL_ABSOLUTE_TOLERANCE),
    ):
        ref_metric = _metric(ref_episode.get(name), f"reference.{name}")
        cand_metric = _metric(cand_episode.get(name), f"candidate.{name}")
        for field in ("rms", "abs_max"):
            limit = _cap(float(ref_metric[field]), tolerance)
            actual = float(cand_metric[field])
            checks.append(
                _check(
                    f"episode_metrics.{name}.{field}",
                    actual,
                    f"<={limit}",
                    actual <= limit,
                )
            )

    ref_sea = _mapping(
        reference.get("sea_episode_metrics"), "reference.sea_episode_metrics"
    )
    cand_sea = _mapping(
        candidate.get("sea_episode_metrics"), "candidate.sea_episode_metrics"
    )
    for joint in JOINTS:
        ref_joint = _mapping(ref_sea.get(joint), f"reference.sea.{joint}")
        cand_joint = _mapping(cand_sea.get(joint), f"candidate.sea.{joint}")
        for signal in SEA_SIGNALS:
            ref_metric = _metric(ref_joint.get(signal), f"reference.{joint}.{signal}")
            cand_metric = _metric(cand_joint.get(signal), f"candidate.{joint}.{signal}")
            for field in ("rms", "abs_max"):
                limit = _cap(float(ref_metric[field]), SEA_ABSOLUTE_TOLERANCE)
                actual = float(cand_metric[field])
                checks.append(
                    _check(
                        f"sea.{joint}.{signal}.{field}",
                        actual,
                        f"<={limit}",
                        actual <= limit,
                    )
                )
        ref_sat = _mapping(
            ref_joint.get("tau_input_saturated"), f"reference.{joint}.saturation"
        )
        cand_sat = _mapping(
            cand_joint.get("tau_input_saturated"), f"candidate.{joint}.saturation"
        )
        ref_fraction = _finite(
            ref_sat.get("fraction"),
            f"reference.{joint}.saturation.fraction",
            minimum=0.0,
        )
        cand_fraction = _finite(
            cand_sat.get("fraction"),
            f"candidate.{joint}.saturation.fraction",
            minimum=0.0,
        )
        checks.append(
            _check(
                f"sea.{joint}.saturation_fraction",
                cand_fraction,
                f"<={ref_fraction}",
                cand_fraction <= ref_fraction,
            )
        )

    for name in (
        "action_clipped_values",
        "timeout_count",
        "fallback_count",
        "hard_invalid_count",
        "invalid_event_count",
        "nonfinite_count",
        "safety_stop_count",
    ):
        ref_value = _integer(reference.get(name), f"reference.{name}")
        cand_value = _integer(candidate.get(name), f"candidate.{name}")
        checks.append(
            _check(name, cand_value, f"<={ref_value}", cand_value <= ref_value)
        )

    passed = all(item["status"] == "PASS" for item in checks)
    return {
        "status": "PASS_CONDITION_NONREGRESSION"
        if passed
        else "FAIL_CONDITION_NONREGRESSION",
        "passed": passed,
        "relative_tolerance": RELATIVE_TOLERANCE,
        "checks": checks,
    }


def condition_matched_gate(
    reference: Mapping[str, Any], candidate: Mapping[str, Any]
) -> dict[str, Any]:
    reference_common = common_rollout_gate(reference, label="reference")
    candidate_common = common_rollout_gate(candidate, label="candidate")
    identity_fields = (
        "trial_id",
        "plateau_id",
        "action_selection",
        "seed",
        "episode_start_time_s",
        "episode_start_offset_s",
        "sigma",
        "noise_tape_sha256",
        "event_contract_id",
    )
    identity_checks = [
        _check(
            name,
            candidate.get(name),
            reference.get(name),
            candidate.get(name) == reference.get(name),
        )
        for name in identity_fields
    ]
    identity_passed = all(item["status"] == "PASS" for item in identity_checks)
    counterfactual = prescribed_counterfactual_gate(candidate)
    nonregression = _nonregression(reference, candidate)
    passed = (
        reference_common["passed"]
        and candidate_common["passed"]
        and identity_passed
        and counterfactual["passed"]
        and nonregression["passed"]
    )
    return {
        "schema_version": 1,
        "gate_id": "H0_PRIMARY_GRF_SPLIT_V2_CONDITION_PAIR",
        "status": "PASS_H0_PRIMARY_SPLIT_V2_CONDITION"
        if passed
        else "FAIL_H0_PRIMARY_SPLIT_V2_CONDITION",
        "passed": passed,
        "reference_common": reference_common,
        "candidate_common": candidate_common,
        "condition_identity": {"passed": identity_passed, "checks": identity_checks},
        "prescribed_counterfactual": counterfactual,
        "nonregression": nonregression,
    }
