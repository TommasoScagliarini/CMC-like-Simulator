"""Pure closed-loop gate for ``H0_primary_split_v1`` rollouts.

The comparator has no OpenSim, Torch, or RLlib dependency.  It consumes one
reference-teacher summary and the condition-matched autonomous-candidate
summary, validates the common physical contract, applies the tolerances frozen
in the 2026-08-05 primary-GRF-split addendum, and publishes at most one strict
JSON result.

The condition identity is deliberately explicit.  A matching ``condition_id``
alone is not sufficient evidence that two runs used the same trial, window,
seed, action mode, sigma, and innovation tape.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import tempfile
from pathlib import Path
from typing import Any, Mapping, Sequence


SCHEMA_VERSION = 1
EXPECTED_STEPS = 500
EXPECTED_ACTOR_FEATURES = 35
EXPECTED_FULL_FEATURES = 84
EXPECTED_DTYPE = "float32"
EXPECTED_MORPHOLOGY_WEIGHT = 0.0
EXPECTED_EVENT_CONTRACT = "primary_grf_split_v1+legacy_events_v1"
EXPECTED_BINARY_PHASE_MODE = "disabled"
PENETRATION_LIMIT_M = 0.025

RELATIVE_NONREGRESSION_TOLERANCE = 0.05
RESERVE_ABSOLUTE_TOLERANCE_NM = 5.0
RESIDUAL_ABSOLUTE_TOLERANCE_NM = 1.0e-6
SEA_ABSOLUTE_TOLERANCE = 1.0e-6
COUNTERFACTUAL_RMSE_LIMIT = 0.015
COUNTERFACTUAL_MAX_ABS_LIMIT = 0.10

JOINTS = ("pros_knee_angle", "pros_ankle_angle")
SEA_SIGNALS = (
    "torque_error_nm",
    "tau_spring_nm",
    "tau_spring_rate_nm_s",
    "motor_speed_rad_s",
    "motor_accel_rad_s2",
    "motor_power_w",
)
CONTINUOUS_AGGREGATIONS = ("rms", "abs_max")
ZERO_COMMON_COUNTS = (
    "action_clipped_values",
    "timeout_count",
    "safety_stop_count",
    "fallback_count",
    "hard_invalid_count",
    "invalid_event_count",
    "nonfinite_count",
)
NO_TOLERANCE_COUNTS = (
    "action_clipped_values",
    "timeout_count",
    "safety_stop_count",
    "fallback_count",
    "hard_invalid_count",
    "invalid_event_count",
    "nonfinite_count",
)
CONDITION_FIELDS = (
    "condition_id",
    "trial_id",
    "plateau_id",
    "episode_start_time_s",
    "episode_start_offset_s",
    "seed",
    "action_selection",
    "sigma",
    "noise_tape_sha256",
)


class H0PrimaryGRFSplitGateError(RuntimeError):
    """Raised when an input or output cannot be classified safely."""


def _reject_constant(token: str) -> None:
    raise H0PrimaryGRFSplitGateError(f"non-finite JSON constant: {token}")


def _reject_duplicate_pairs(pairs: Sequence[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise H0PrimaryGRFSplitGateError(f"duplicate JSON key: {key!r}")
        result[key] = value
    return result


def _finite_tree(value: Any) -> bool:
    if value is None or isinstance(value, (bool, int, str)):
        return True
    if isinstance(value, float):
        return math.isfinite(value)
    if isinstance(value, Mapping):
        return all(
            isinstance(key, str) and _finite_tree(item) for key, item in value.items()
        )
    if isinstance(value, (list, tuple)):
        return all(_finite_tree(item) for item in value)
    return False


def canonical_json_bytes(payload: Any) -> bytes:
    if not _finite_tree(payload):
        raise H0PrimaryGRFSplitGateError("payload is not a finite strict-JSON value")
    try:
        return (
            json.dumps(
                payload,
                indent=2,
                sort_keys=True,
                ensure_ascii=False,
                allow_nan=False,
            )
            + "\n"
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise H0PrimaryGRFSplitGateError(f"payload is not strict JSON: {exc}") from exc


def payload_sha256(payload: Any) -> str:
    return hashlib.sha256(canonical_json_bytes(payload)).hexdigest()


def strict_json_load(path: str | Path) -> dict[str, Any]:
    source = Path(path).expanduser().resolve()
    try:
        payload = json.loads(
            source.read_text(encoding="utf-8"),
            object_pairs_hook=_reject_duplicate_pairs,
            parse_constant=_reject_constant,
        )
    except H0PrimaryGRFSplitGateError:
        raise
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise H0PrimaryGRFSplitGateError(
            f"cannot read strict JSON {source}: {exc}"
        ) from exc
    if not isinstance(payload, Mapping):
        raise H0PrimaryGRFSplitGateError(f"JSON root must be an object: {source}")
    canonical_json_bytes(payload)
    return dict(payload)


def _fsync_directory(path: Path) -> None:
    try:
        descriptor = os.open(path, os.O_RDONLY)
    except OSError:  # pragma: no cover - unsupported on some platforms.
        return
    try:
        os.fsync(descriptor)
    except OSError:  # pragma: no cover - unsupported on some platforms.
        pass
    finally:
        os.close(descriptor)


def write_json_exclusive(path: str | Path, payload: Any) -> Path:
    """Publish one strict JSON artifact atomically and without replacement."""

    target = Path(path).expanduser().resolve()
    encoded = canonical_json_bytes(payload)
    target.parent.mkdir(parents=True, exist_ok=True)
    if os.path.lexists(target):
        raise H0PrimaryGRFSplitGateError(f"refusing to clobber: {target}")

    descriptor, temporary_raw = tempfile.mkstemp(
        prefix=f".{target.name}.", suffix=".tmp", dir=str(target.parent)
    )
    temporary = Path(temporary_raw)
    descriptor_open = True
    try:
        with os.fdopen(descriptor, "wb") as stream:
            descriptor_open = False
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.link(temporary, target)
        except FileExistsError as exc:
            raise H0PrimaryGRFSplitGateError(f"refusing to clobber: {target}") from exc
        _fsync_directory(target.parent)
        return target
    finally:
        if descriptor_open:
            os.close(descriptor)
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def _mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise H0PrimaryGRFSplitGateError(f"{label} must be an object")
    return dict(value)


def _finite(value: Any, label: str, *, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise H0PrimaryGRFSplitGateError(f"{label} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise H0PrimaryGRFSplitGateError(f"{label} must be finite")
    if minimum is not None and result < minimum:
        raise H0PrimaryGRFSplitGateError(f"{label} must be >= {minimum}, got {result}")
    return result


def _integer(value: Any, label: str, *, minimum: int = 0) -> int:
    if type(value) is not int:
        raise H0PrimaryGRFSplitGateError(f"{label} must be an integer")
    if value < minimum:
        raise H0PrimaryGRFSplitGateError(f"{label} must be >= {minimum}, got {value}")
    return value


def _sha256(value: Any, label: str) -> str:
    if not (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    ):
        raise H0PrimaryGRFSplitGateError(f"{label} must be a lowercase SHA-256 digest")
    return value


def _check(name: str, actual: Any, expected: Any, passed: bool) -> dict[str, Any]:
    return {
        "name": name,
        "status": "PASS" if passed else "FAIL",
        "actual": actual,
        "expected": expected,
    }


def _metric(
    value: Any,
    label: str,
    *,
    expected_sample_count: int | None,
) -> dict[str, Any]:
    metric = _mapping(value, label)
    sample_count = _integer(metric.get("sample_count"), f"{label}.sample_count")
    if expected_sample_count is not None and sample_count != expected_sample_count:
        raise H0PrimaryGRFSplitGateError(
            f"{label}.sample_count must be {expected_sample_count}, got {sample_count}"
        )
    rms = _finite(metric.get("rms"), f"{label}.rms", minimum=0.0)
    abs_max = _finite(metric.get("abs_max"), f"{label}.abs_max", minimum=0.0)
    if abs_max + 1.0e-15 < rms:
        raise H0PrimaryGRFSplitGateError(f"{label}.abs_max cannot be below rms")
    return {"sample_count": sample_count, "rms": rms, "abs_max": abs_max}


def common_rollout_gate(
    summary: Mapping[str, Any], *, label: str = "rollout"
) -> dict[str, Any]:
    """Validate the physical/layout contract shared by reference and candidate."""

    data = _mapping(summary, f"{label} summary")
    checks: list[dict[str, Any]] = []

    steps = _integer(data.get("steps"), f"{label}.steps")
    checks.append(_check("steps", steps, EXPECTED_STEPS, steps == EXPECTED_STEPS))
    checks.append(
        _check(
            "end_reason",
            data.get("end_reason"),
            "episode_time_limit",
            data.get("end_reason") == "episode_time_limit",
        )
    )
    checks.append(
        _check(
            "terminated", data.get("terminated"), False, data.get("terminated") is False
        )
    )
    checks.append(
        _check("truncated", data.get("truncated"), True, data.get("truncated") is True)
    )
    cycles = _integer(
        data.get("phase_valid_cycle_count"), f"{label}.phase_valid_cycle_count"
    )
    checks.append(_check("phase_valid_cycle_count", cycles, ">=2", cycles >= 2))

    penetration = _finite(
        data.get("grf_penetration_max_m"),
        f"{label}.grf_penetration_max_m",
        minimum=0.0,
    )
    checks.append(
        _check(
            "grf_penetration_max_m",
            penetration,
            f"<{PENETRATION_LIMIT_M}",
            penetration < PENETRATION_LIMIT_M,
        )
    )
    for name in ZERO_COMMON_COUNTS:
        value = _integer(data.get(name), f"{label}.{name}")
        checks.append(_check(name, value, 0, value == 0))

    checks.extend(
        (
            _check(
                "n_actor",
                data.get("n_actor"),
                EXPECTED_ACTOR_FEATURES,
                data.get("n_actor") == EXPECTED_ACTOR_FEATURES,
            ),
            _check(
                "n_observation",
                data.get("n_observation"),
                EXPECTED_FULL_FEATURES,
                data.get("n_observation") == EXPECTED_FULL_FEATURES,
            ),
            _check(
                "observation_dtype",
                data.get("observation_dtype"),
                EXPECTED_DTYPE,
                data.get("observation_dtype") == EXPECTED_DTYPE,
            ),
            _check(
                "action_shape",
                data.get("action_shape"),
                [2],
                data.get("action_shape") == [2],
            ),
            _check(
                "action_dtype",
                data.get("action_dtype"),
                EXPECTED_DTYPE,
                data.get("action_dtype") == EXPECTED_DTYPE,
            ),
        )
    )
    morphology_weight = _finite(
        data.get("morphology_weight"), f"{label}.morphology_weight"
    )
    checks.append(
        _check(
            "morphology_weight",
            morphology_weight,
            EXPECTED_MORPHOLOGY_WEIGHT,
            morphology_weight == EXPECTED_MORPHOLOGY_WEIGHT,
        )
    )
    checks.extend(
        (
            _check(
                "event_contract_id",
                data.get("event_contract_id"),
                EXPECTED_EVENT_CONTRACT,
                data.get("event_contract_id") == EXPECTED_EVENT_CONTRACT,
            ),
            _check(
                "binary_phase_fsm_mode",
                data.get("binary_phase_fsm_mode"),
                EXPECTED_BINARY_PHASE_MODE,
                data.get("binary_phase_fsm_mode")
                == EXPECTED_BINARY_PHASE_MODE,
            ),
            _check(
                "ppo_updates",
                data.get("ppo_updates"),
                0,
                data.get("ppo_updates") == 0,
            ),
            _check(
                "protected_trials_opened",
                data.get("protected_trials_opened"),
                [],
                data.get("protected_trials_opened") == [],
            ),
            _check(
                "preregistered_provenance",
                data.get("preregistered_provenance", {}).get("passed")
                if isinstance(data.get("preregistered_provenance"), Mapping)
                else None,
                True,
                isinstance(data.get("preregistered_provenance"), Mapping)
                and data["preregistered_provenance"].get("passed") is True,
            ),
        )
    )
    actor_names = data.get("actor_feature_names")
    full_names = data.get("observation_feature_names")
    no_binary_features = (
        isinstance(actor_names, list)
        and isinstance(full_names, list)
        and not any(
            "binary_phase" in str(name) for name in [*actor_names, *full_names]
        )
    )
    checks.append(
        _check(
            "binary_features_absent",
            no_binary_features,
            True,
            no_binary_features,
        )
    )

    episode_metrics = _mapping(data.get("episode_metrics"), f"{label}.episode_metrics")
    for metric_name in ("reserve_norm_nm", "residual_norm_nm"):
        metric = _metric(
            episode_metrics.get(metric_name),
            f"{label}.episode_metrics.{metric_name}",
            expected_sample_count=EXPECTED_STEPS,
        )
        checks.append(
            _check(
                f"episode_metrics.{metric_name}.finite",
                metric,
                "500 finite nonnegative samples",
                True,
            )
        )

    sea = _mapping(data.get("sea_episode_metrics"), f"{label}.sea_episode_metrics")
    for joint in JOINTS:
        joint_metrics = _mapping(sea.get(joint), f"{label}.sea_episode_metrics.{joint}")
        signal_sample_count: int | None = None
        for signal in SEA_SIGNALS:
            metric = _metric(
                joint_metrics.get(signal),
                f"{label}.sea_episode_metrics.{joint}.{signal}",
                expected_sample_count=None,
            )
            if metric["sample_count"] <= 0:
                raise H0PrimaryGRFSplitGateError(
                    f"{label}.{joint}.{signal}.sample_count must be positive"
                )
            if signal_sample_count is None:
                signal_sample_count = metric["sample_count"]
            checks.append(
                _check(
                    f"sea_episode_metrics.{joint}.{signal}.finite",
                    metric,
                    "finite nonnegative metric",
                    True,
                )
            )

        saturation = _mapping(
            joint_metrics.get("tau_input_saturated"),
            f"{label}.sea_episode_metrics.{joint}.tau_input_saturated",
        )
        denominator = _integer(
            saturation.get("sample_count"), f"{label}.{joint}.saturation.sample_count"
        )
        count = _integer(saturation.get("count"), f"{label}.{joint}.saturation.count")
        fraction = _finite(
            saturation.get("fraction"),
            f"{label}.{joint}.saturation.fraction",
            minimum=0.0,
        )
        consistent = (
            denominator > 0
            and denominator == signal_sample_count
            and count <= denominator
            and fraction <= 1.0
            and math.isclose(
                fraction, count / denominator, rel_tol=0.0, abs_tol=1.0e-15
            )
        )
        checks.append(
            _check(
                f"sea_episode_metrics.{joint}.tau_input_saturated.consistent",
                {
                    "sample_count": denominator,
                    "count": count,
                    "fraction": fraction,
                },
                "same positive denominator and fraction=count/sample_count",
                consistent,
            )
        )

    passed = all(check["status"] == "PASS" for check in checks)
    return {
        "status": "PASS_COMMON_ROLLOUT" if passed else "FAIL_COMMON_ROLLOUT",
        "passed": passed,
        "checks": checks,
    }


def common_rollout_checks(summary: Mapping[str, Any]) -> list[dict[str, Any]]:
    """Compatibility helper returning only the common check ledger."""

    return common_rollout_gate(summary)["checks"]


def _condition_identity(summary: Mapping[str, Any], label: str) -> dict[str, Any]:
    data = _mapping(summary, f"{label} summary")
    identity: dict[str, Any] = {}
    for field in CONDITION_FIELDS:
        if field not in data:
            raise H0PrimaryGRFSplitGateError(
                f"{label} summary is missing condition field {field!r}"
            )
        identity[field] = data[field]

    for field in ("condition_id", "trial_id", "plateau_id", "action_selection"):
        if not isinstance(identity[field], str) or not identity[field]:
            raise H0PrimaryGRFSplitGateError(f"{label}.{field} must be non-empty text")
    _integer(identity["seed"], f"{label}.seed")
    _finite(identity["episode_start_time_s"], f"{label}.episode_start_time_s")
    _finite(identity["episode_start_offset_s"], f"{label}.episode_start_offset_s")
    sigma = _finite(identity["sigma"], f"{label}.sigma", minimum=0.0)
    tape = identity["noise_tape_sha256"]
    if sigma > 0.0:
        _sha256(tape, f"{label}.noise_tape_sha256")
    elif tape is not None:
        _sha256(tape, f"{label}.noise_tape_sha256")
    return identity


def _cap(reference: float, absolute_tolerance: float) -> tuple[float, float]:
    tolerance = max(
        RELATIVE_NONREGRESSION_TOLERANCE * abs(reference), absolute_tolerance
    )
    return reference + tolerance, tolerance


def _condition_matched_nonregression_gate(
    reference_summary: Mapping[str, Any],
    candidate_summary: Mapping[str, Any],
) -> dict[str, Any]:
    """Apply exact-condition and addendum metric non-regression gates."""

    reference = _mapping(reference_summary, "reference summary")
    candidate = _mapping(candidate_summary, "candidate summary")
    reference_identity = _condition_identity(reference, "reference")
    candidate_identity = _condition_identity(candidate, "candidate")
    checks: list[dict[str, Any]] = []
    for field in CONDITION_FIELDS:
        checks.append(
            _check(
                f"condition.{field}",
                candidate_identity[field],
                reference_identity[field],
                candidate_identity[field] == reference_identity[field],
            )
        )

    reference_episode = _mapping(
        reference.get("episode_metrics"), "reference.episode_metrics"
    )
    candidate_episode = _mapping(
        candidate.get("episode_metrics"), "candidate.episode_metrics"
    )

    def add_continuous(
        name: str, reference_value: Any, candidate_value: Any, absolute_tolerance: float
    ) -> None:
        reference_number = _finite(reference_value, f"reference.{name}", minimum=0.0)
        candidate_number = _finite(candidate_value, f"candidate.{name}", minimum=0.0)
        cap, tolerance = _cap(reference_number, absolute_tolerance)
        item = _check(name, candidate_number, f"<= {cap}", candidate_number <= cap)
        item.update(
            {
                "reference": reference_number,
                "tolerance": tolerance,
                "cap": cap,
            }
        )
        checks.append(item)

    for metric_name, absolute_tolerance in (
        ("reserve_norm_nm", RESERVE_ABSOLUTE_TOLERANCE_NM),
        ("residual_norm_nm", RESIDUAL_ABSOLUTE_TOLERANCE_NM),
    ):
        reference_metric = _mapping(
            reference_episode.get(metric_name), f"reference.{metric_name}"
        )
        candidate_metric = _mapping(
            candidate_episode.get(metric_name), f"candidate.{metric_name}"
        )
        reference_count = _integer(
            reference_metric.get("sample_count"),
            f"reference.{metric_name}.sample_count",
        )
        candidate_count = _integer(
            candidate_metric.get("sample_count"),
            f"candidate.{metric_name}.sample_count",
        )
        checks.append(
            _check(
                f"{metric_name}.sample_count",
                candidate_count,
                reference_count,
                candidate_count == reference_count,
            )
        )
        for aggregation in CONTINUOUS_AGGREGATIONS:
            add_continuous(
                f"{metric_name}.{aggregation}",
                reference_metric.get(aggregation),
                candidate_metric.get(aggregation),
                absolute_tolerance,
            )

    reference_sea = _mapping(
        reference.get("sea_episode_metrics"), "reference.sea_episode_metrics"
    )
    candidate_sea = _mapping(
        candidate.get("sea_episode_metrics"), "candidate.sea_episode_metrics"
    )
    for joint in JOINTS:
        reference_joint = _mapping(reference_sea.get(joint), f"reference.{joint}")
        candidate_joint = _mapping(candidate_sea.get(joint), f"candidate.{joint}")
        for signal in SEA_SIGNALS:
            reference_metric = _mapping(
                reference_joint.get(signal), f"reference.{joint}.{signal}"
            )
            candidate_metric = _mapping(
                candidate_joint.get(signal), f"candidate.{joint}.{signal}"
            )
            _integer(
                reference_metric.get("sample_count"),
                f"reference.{joint}.{signal}.sample_count",
            )
            _integer(
                candidate_metric.get("sample_count"),
                f"candidate.{joint}.{signal}.sample_count",
            )
            for aggregation in CONTINUOUS_AGGREGATIONS:
                add_continuous(
                    f"{joint}.{signal}.{aggregation}",
                    reference_metric.get(aggregation),
                    candidate_metric.get(aggregation),
                    SEA_ABSOLUTE_TOLERANCE,
                )

        reference_saturation = _mapping(
            reference_joint.get("tau_input_saturated"), f"reference.{joint}.saturation"
        )
        candidate_saturation = _mapping(
            candidate_joint.get("tau_input_saturated"), f"candidate.{joint}.saturation"
        )
        reference_denominator = _integer(
            reference_saturation.get("sample_count"),
            f"reference.{joint}.saturation.sample_count",
        )
        candidate_denominator = _integer(
            candidate_saturation.get("sample_count"),
            f"candidate.{joint}.saturation.sample_count",
        )
        reference_count = _integer(
            reference_saturation.get("count"),
            f"reference.{joint}.saturation.count",
        )
        candidate_count = _integer(
            candidate_saturation.get("count"),
            f"candidate.{joint}.saturation.count",
        )
        reference_fraction = _finite(
            reference_saturation.get("fraction"),
            f"reference.{joint}.saturation.fraction",
            minimum=0.0,
        )
        candidate_fraction = _finite(
            candidate_saturation.get("fraction"),
            f"candidate.{joint}.saturation.fraction",
            minimum=0.0,
        )
        checks.append(
            _check(
                f"{joint}.tau_input_saturated.fraction",
                candidate_fraction,
                f"<= {reference_fraction}",
                candidate_fraction <= reference_fraction,
            )
        )
        checks.append(
            {
                "name": f"{joint}.tau_input_saturated.adaptive_sample_diagnostic",
                "status": "PASS",
                "actual": {
                    "reference_sample_count": reference_denominator,
                    "candidate_sample_count": candidate_denominator,
                    "reference_saturated_count": reference_count,
                    "candidate_saturated_count": candidate_count,
                },
                "expected": "positive internally-consistent denominators; gate normalized fraction only",
            }
        )

    for name in NO_TOLERANCE_COUNTS:
        reference_value = _integer(reference.get(name), f"reference.{name}")
        candidate_value = _integer(candidate.get(name), f"candidate.{name}")
        checks.append(
            _check(
                name,
                candidate_value,
                f"<= {reference_value}",
                candidate_value <= reference_value,
            )
        )

    passed = all(check["status"] == "PASS" for check in checks)
    return {
        "status": (
            "PASS_CONDITION_MATCHED_NONREGRESSION"
            if passed
            else "FAIL_CONDITION_MATCHED_NONREGRESSION"
        ),
        "passed": passed,
        "reference_condition": reference_identity,
        "candidate_condition": candidate_identity,
        "checks": checks,
    }


def gate_counterfactual(metrics: Mapping[str, Any]) -> dict[str, Any]:
    """Gate candidate means against H0 teacher means on candidate states."""

    data = _mapping(metrics, "counterfactual_teacher_error")
    samples = _integer(
        data.get("sample_count"), "counterfactual_teacher_error.sample_count"
    )
    rmse = _finite(data.get("rmse"), "counterfactual_teacher_error.rmse", minimum=0.0)
    max_abs_error = _finite(
        data.get("max_abs_error"),
        "counterfactual_teacher_error.max_abs_error",
        minimum=0.0,
    )
    teacher_dependency_count = _integer(
        data.get("served_action_teacher_dependency_count"),
        "counterfactual_teacher_error.served_action_teacher_dependency_count",
    )
    action_selected_first = data.get("action_selected_before_teacher_diagnostic")
    checks = [
        _check("sample_count", samples, EXPECTED_STEPS, samples == EXPECTED_STEPS),
        _check(
            "rmse",
            rmse,
            f"<={COUNTERFACTUAL_RMSE_LIMIT}",
            rmse <= COUNTERFACTUAL_RMSE_LIMIT,
        ),
        _check(
            "max_abs_error",
            max_abs_error,
            f"<={COUNTERFACTUAL_MAX_ABS_LIMIT}",
            max_abs_error <= COUNTERFACTUAL_MAX_ABS_LIMIT,
        ),
        _check(
            "served_action_teacher_dependency_count",
            teacher_dependency_count,
            0,
            teacher_dependency_count == 0,
        ),
        _check(
            "action_selected_before_teacher_diagnostic",
            action_selected_first,
            True,
            action_selected_first is True,
        ),
    ]
    passed = all(check["status"] == "PASS" for check in checks)
    return {
        "status": "PASS_COUNTERFACTUAL_TEACHER"
        if passed
        else "FAIL_COUNTERFACTUAL_TEACHER",
        "passed": passed,
        "checks": checks,
    }


def condition_matched_gate(
    reference_summary: Mapping[str, Any],
    candidate_summary: Mapping[str, Any],
    counterfactual_teacher_error: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Run the complete gate for one condition-matched rollout pair.

    This is the public supervisor API.  It intentionally includes both common
    rollout contracts and the counterfactual gate, so a caller cannot promote a
    candidate after checking only the relative reserve/residual/SEA metrics.
    """

    reference = _mapping(reference_summary, "reference summary")
    candidate = _mapping(candidate_summary, "candidate summary")
    if counterfactual_teacher_error is None:
        counterfactual_teacher_error = _mapping(
            candidate.get("counterfactual_teacher_error"),
            "candidate.counterfactual_teacher_error",
        )
    reference_common = common_rollout_gate(reference, label="reference")
    candidate_common = common_rollout_gate(candidate, label="candidate")
    nonregression = _condition_matched_nonregression_gate(reference, candidate)
    counterfactual = gate_counterfactual(counterfactual_teacher_error)
    passed = all(
        result["passed"]
        for result in (
            reference_common,
            candidate_common,
            nonregression,
            counterfactual,
        )
    )
    return {
        "schema_version": SCHEMA_VERSION,
        "gate_id": "H0_PRIMARY_GRF_SPLIT_V1_CONDITION_PAIR",
        "status": (
            "PASS_H0_PRIMARY_GRF_SPLIT_CONDITION"
            if passed
            else "FAIL_H0_PRIMARY_GRF_SPLIT_CLOSED_LOOP"
        ),
        "passed": passed,
        "thresholds": {
            "steps_exact": EXPECTED_STEPS,
            "penetration_m_strictly_less_than": PENETRATION_LIMIT_M,
            "relative_nonregression_tolerance": RELATIVE_NONREGRESSION_TOLERANCE,
            "reserve_absolute_tolerance_nm": RESERVE_ABSOLUTE_TOLERANCE_NM,
            "residual_absolute_tolerance_nm": RESIDUAL_ABSOLUTE_TOLERANCE_NM,
            "sea_absolute_tolerance_native_unit": SEA_ABSOLUTE_TOLERANCE,
            "counterfactual_rmse_max": COUNTERFACTUAL_RMSE_LIMIT,
            "counterfactual_abs_error_max": COUNTERFACTUAL_MAX_ABS_LIMIT,
        },
        "reference_summary_sha256": payload_sha256(reference),
        "candidate_summary_sha256": payload_sha256(candidate),
        "reference_common": reference_common,
        "candidate_common": candidate_common,
        "condition_matched_nonregression": nonregression,
        "counterfactual_teacher_error": counterfactual,
        # Flat compatibility fields used by audit/report helpers.
        "reference_condition": nonregression["reference_condition"],
        "candidate_condition": nonregression["candidate_condition"],
        "checks": nonregression["checks"],
    }


def compare_h0_primary_grf_split(
    reference_summary: Mapping[str, Any],
    candidate_summary: Mapping[str, Any],
    counterfactual_teacher_error: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Run the complete pure comparator for one frozen condition pair."""

    return condition_matched_gate(
        reference_summary,
        candidate_summary,
        counterfactual_teacher_error,
    )


# Short aliases used by the qualification supervisor.
compare_rollouts = compare_h0_primary_grf_split
gate_candidate = condition_matched_gate


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--reference", "--reference-summary", dest="reference", required=True
    )
    parser.add_argument(
        "--candidate", "--candidate-summary", dest="candidate", required=True
    )
    parser.add_argument(
        "--counterfactual",
        help=(
            "Optional strict JSON metrics; defaults to "
            "candidate.counterfactual_teacher_error"
        ),
    )
    parser.add_argument("--output", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        reference = strict_json_load(args.reference)
        candidate = strict_json_load(args.candidate)
        counterfactual = (
            strict_json_load(args.counterfactual) if args.counterfactual else None
        )
        result = compare_h0_primary_grf_split(reference, candidate, counterfactual)
        write_json_exclusive(args.output, result)
    except H0PrimaryGRFSplitGateError as exc:
        print(
            f"H0 primary-GRF-split comparison failed closed: {exc}", file=os.sys.stderr
        )
        return 2
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0 if result["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
