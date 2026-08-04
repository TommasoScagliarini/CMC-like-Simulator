"""Pure warm-up certification helpers for the V14.1 detector protocol.

The functions in this module perform no file or OpenSim access.  They certify
that continuous detector and prescribed histories contain enough causal
context before the first scoreable heel strike, and provide small helpers for
folding that certificate into full-gate and diagnostic root-safety decisions.
"""

from __future__ import annotations

import copy
import math
from collections.abc import Mapping, Sequence
from typing import Any


CERTIFICATE_SCHEMA_VERSION = 1
WARMUP_GUARD_S = 0.090
NUMERIC_TOLERANCE_S = 1.0e-12

WAIT_HS = 0
STANCE_AFTER_HS = 1
SWING_AFTER_TO = 2
VALID_CYCLE_COMPLETED = 3
TIMEOUT = 4
INVALID_EVENT = 5

STATE_NAMES = {
    WAIT_HS: "WAIT_HS",
    STANCE_AFTER_HS: "STANCE_AFTER_HS",
    SWING_AFTER_TO: "SWING_AFTER_TO",
    VALID_CYCLE_COMPLETED: "VALID_CYCLE_COMPLETED",
    TIMEOUT: "TIMEOUT",
    INVALID_EVENT: "INVALID_EVENT",
}
FAILURE_STATE_IDS = frozenset({TIMEOUT, INVALID_EVENT})

REQUIRED_CERTIFICATE_CHECKS = (
    "detector_complete_hs_to_toe_off_to_hs_cycle",
    "prescribed_complete_hs_to_toe_off_to_hs_cycle",
    "zero_timeout_transitions",
    "zero_invalid_transitions",
    "cutoff_state_known",
    "cutoff_state_non_failure",
)


class WarmupValidationError(ValueError):
    """Raised when evidence cannot safely support a warm-up decision."""


def _finite_float(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise WarmupValidationError(f"{label} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise WarmupValidationError(f"{label} must be a finite number") from exc
    if not math.isfinite(result):
        raise WarmupValidationError(f"{label} must be a finite number")
    return result


def _sequence(value: Any, label: str) -> list[Any]:
    if isinstance(value, (str, bytes, Mapping)):
        raise WarmupValidationError(f"{label} must be a sequence")
    try:
        return list(value)
    except TypeError as exc:
        raise WarmupValidationError(f"{label} must be a sequence") from exc


def _finite_series(value: Any, label: str) -> list[float]:
    return [
        _finite_float(item, f"{label}[{index}]")
        for index, item in enumerate(_sequence(value, label))
    ]


def _strictly_increasing(values: Sequence[float], label: str) -> None:
    if any(right <= left for left, right in zip(values, values[1:])):
        raise WarmupValidationError(f"{label} must be strictly increasing")


def _state_series(value: Any, expected_size: int) -> list[int]:
    raw = _sequence(value, "state_ids")
    if len(raw) != expected_size:
        raise WarmupValidationError(
            "sample_times_s and state_ids must have identical lengths"
        )
    result: list[int] = []
    for index, item in enumerate(raw):
        numeric = _finite_float(item, f"state_ids[{index}]")
        rounded = int(round(numeric))
        if not math.isclose(
            numeric, float(rounded), rel_tol=0.0, abs_tol=NUMERIC_TOLERANCE_S
        ):
            raise WarmupValidationError("state_ids must contain integer-valued IDs")
        result.append(rounded)
    return result


def _event_timestamp(record: Mapping[str, Any], label: str) -> float:
    values = [
        (field, _finite_float(record[field], f"{label}.{field}"))
        for field in ("confirmed_time_s", "observed_at_s", "time", "event_time_s")
        if record.get(field) is not None
    ]
    if not values:
        raise WarmupValidationError(f"{label} has no event timestamp")
    timestamp = values[0][1]
    if any(
        not math.isclose(
            value,
            timestamp,
            rel_tol=0.0,
            abs_tol=NUMERIC_TOLERANCE_S,
        )
        for _field, value in values[1:]
    ):
        raise WarmupValidationError(f"{label} has inconsistent event timestamps")
    return timestamp


def _validated_detector_events(
    accepted_events: Any,
    *,
    analysis_start_s: float,
    sample_end_s: float,
) -> tuple[list[dict[str, Any]], list[float]]:
    records = _sequence(accepted_events, "detector_accepted_events")
    gait: list[dict[str, Any]] = []
    timeouts: list[float] = []
    previous_timestamp = -math.inf
    previous_gait_onset = -math.inf
    for index, raw in enumerate(records):
        label = f"detector_accepted_events[{index}]"
        if not isinstance(raw, Mapping):
            raise WarmupValidationError(f"{label} must be an object")
        event = raw.get("event")
        if event not in {"heel_strike", "toe_off", "timeout"}:
            raise WarmupValidationError(f"{label}.event is unknown")
        if event == "timeout":
            timestamp = _event_timestamp(raw, label)
            timeouts.append(timestamp)
        else:
            onset = _finite_float(raw.get("event_time_s"), f"{label}.event_time_s")
            confirmed = _finite_float(
                raw.get("confirmed_time_s"), f"{label}.confirmed_time_s"
            )
            if confirmed + NUMERIC_TOLERANCE_S < onset:
                raise WarmupValidationError(
                    f"{label} is confirmed before its detector onset"
                )
            if onset <= previous_gait_onset + NUMERIC_TOLERANCE_S:
                raise WarmupValidationError(
                    "detector gait-event onsets must be strictly chronological"
                )
            timestamp = confirmed
            gait.append(
                {
                    "event": event,
                    "event_time_s": onset,
                    "confirmed_time_s": confirmed,
                }
            )
            previous_gait_onset = onset
        if timestamp <= previous_timestamp + NUMERIC_TOLERANCE_S:
            raise WarmupValidationError(
                "detector accepted events must be strictly chronological"
            )
        if timestamp < analysis_start_s - NUMERIC_TOLERANCE_S:
            raise WarmupValidationError(
                "detector accepted event precedes analysis_start_s"
            )
        if timestamp > sample_end_s + NUMERIC_TOLERANCE_S:
            raise WarmupValidationError(
                "detector accepted event escapes the sampled interval"
            )
        previous_timestamp = timestamp
    return gait, timeouts


def _validated_invalid_times(
    invalid_steps: Any,
    *,
    analysis_start_s: float,
    sample_end_s: float,
) -> list[float]:
    result: list[float] = []
    for index, raw in enumerate(_sequence(invalid_steps, "invalid_steps")):
        label = f"invalid_steps[{index}]"
        if not isinstance(raw, Mapping):
            raise WarmupValidationError(f"{label} must be an object")
        timestamp = _event_timestamp(raw, label)
        if not (
            analysis_start_s - NUMERIC_TOLERANCE_S
            <= timestamp
            <= sample_end_s + NUMERIC_TOLERANCE_S
        ):
            raise WarmupValidationError(f"{label} escapes the sampled interval")
        result.append(timestamp)
    return result


def _detector_cycles(
    gait_events: Sequence[Mapping[str, Any]],
    *,
    analysis_start_s: float,
    cutoff_s: float,
) -> list[dict[str, float]]:
    scoped = [
        item
        for item in gait_events
        if (
            float(item["event_time_s"])
            >= analysis_start_s - NUMERIC_TOLERANCE_S
            and float(item["confirmed_time_s"])
            <= cutoff_s + NUMERIC_TOLERANCE_S
        )
    ]
    phase = "await_hs"
    opening_hs: Mapping[str, Any] | None = None
    toe_off: Mapping[str, Any] | None = None
    cycles: list[dict[str, float]] = []
    for item in scoped:
        event = str(item["event"])
        if phase == "await_hs":
            if event != "heel_strike":
                raise WarmupValidationError(
                    "detector warm-up gait sequence does not begin with HS"
                )
            opening_hs = item
            phase = "await_toe_off"
        elif phase == "await_toe_off":
            if event != "toe_off":
                raise WarmupValidationError(
                    "detector warm-up gait sequence is not HS--TO--HS"
                )
            toe_off = item
            phase = "await_closing_hs"
        else:
            if event != "heel_strike":
                raise WarmupValidationError(
                    "detector warm-up gait sequence is not HS--TO--HS"
                )
            assert opening_hs is not None and toe_off is not None
            cycles.append(
                {
                    "opening_hs_confirmed_s": float(
                        opening_hs["confirmed_time_s"]
                    ),
                    "toe_off_confirmed_s": float(toe_off["confirmed_time_s"]),
                    "closing_hs_confirmed_s": float(item["confirmed_time_s"]),
                }
            )
            opening_hs = item
            toe_off = None
            phase = "await_toe_off"
    return cycles


def _prescribed_cycles(
    prescribed_events: Any,
    *,
    analysis_start_s: float,
    cutoff_s: float,
    first_scoreable_hs_s: float,
) -> list[dict[str, float]]:
    if not isinstance(prescribed_events, Mapping):
        raise WarmupValidationError("prescribed_events must be an object")
    if set(prescribed_events) != {"heel_strike", "toe_off"}:
        raise WarmupValidationError(
            "prescribed_events must contain exactly heel_strike and toe_off"
        )
    heel = _finite_series(prescribed_events["heel_strike"], "prescribed heel_strike")
    toe = _finite_series(prescribed_events["toe_off"], "prescribed toe_off")
    if len(heel) != len(toe) + 1 or len(heel) < 2:
        raise WarmupValidationError(
            "prescribed events must contain complete HS--TO--HS cycles"
        )
    _strictly_increasing(heel, "prescribed heel_strike")
    _strictly_increasing(toe, "prescribed toe_off")
    for index, toe_time in enumerate(toe):
        if not (heel[index] < toe_time < heel[index + 1]):
            raise WarmupValidationError(
                "prescribed events must strictly alternate HS--TO--HS"
            )
    if not any(
        math.isclose(
            item,
            first_scoreable_hs_s,
            rel_tol=0.0,
            abs_tol=NUMERIC_TOLERANCE_S,
        )
        for item in heel
    ):
        raise WarmupValidationError(
            "first_scoreable_hs_s is absent from prescribed heel strikes"
        )
    return [
        {
            "opening_hs_s": heel[index],
            "toe_off_s": toe_time,
            "closing_hs_s": heel[index + 1],
        }
        for index, toe_time in enumerate(toe)
        if (
            heel[index] >= analysis_start_s - NUMERIC_TOLERANCE_S
            and heel[index + 1] <= cutoff_s + NUMERIC_TOLERANCE_S
        )
    ]


def certify_warmup(
    *,
    analysis_start_s: float,
    first_scoreable_hs_s: float,
    sample_dt_s: float,
    sample_times_s: Sequence[float],
    state_ids: Sequence[float],
    detector_accepted_events: Sequence[Mapping[str, Any]],
    detector_invalid_steps: Sequence[Mapping[str, Any]],
    prescribed_events: Mapping[str, Sequence[float]],
) -> dict[str, Any]:
    """Return a deterministic certificate for the pre-score warm-up window.

    Detector cycles are counted using confirmation times because an onset that
    has not yet completed its dwell at the cutoff cannot establish causal FSM
    state.  The cutoff state is the latest sampled state at or before the
    cutoff, and must be no more than one declared cadence old.
    """

    analysis_start = _finite_float(analysis_start_s, "analysis_start_s")
    first_scoreable_hs = _finite_float(
        first_scoreable_hs_s, "first_scoreable_hs_s"
    )
    sample_dt = _finite_float(sample_dt_s, "sample_dt_s")
    if sample_dt <= 0.0:
        raise WarmupValidationError("sample_dt_s must be positive")
    cutoff = first_scoreable_hs - WARMUP_GUARD_S
    if cutoff <= analysis_start + NUMERIC_TOLERANCE_S:
        raise WarmupValidationError(
            "warm-up cutoff must be strictly after analysis_start_s"
        )

    times = _finite_series(sample_times_s, "sample_times_s")
    if len(times) < 2:
        raise WarmupValidationError("sample_times_s requires at least two samples")
    _strictly_increasing(times, "sample_times_s")
    grid_tolerance = max(NUMERIC_TOLERANCE_S, sample_dt * 1.0e-9)
    if not math.isclose(
        times[0], analysis_start, rel_tol=0.0, abs_tol=grid_tolerance
    ):
        raise WarmupValidationError("sample grid does not begin at analysis_start_s")
    if times[-1] < cutoff - grid_tolerance:
        raise WarmupValidationError("sample grid does not cover the warm-up cutoff")
    if any(
        not math.isclose(
            right - left,
            sample_dt,
            rel_tol=0.0,
            abs_tol=grid_tolerance,
        )
        for left, right in zip(times, times[1:])
    ):
        raise WarmupValidationError("sample grid is inconsistent with sample_dt_s")
    states = _state_series(state_ids, len(times))

    cutoff_indices = [
        index
        for index, time_s in enumerate(times)
        if time_s <= cutoff + NUMERIC_TOLERANCE_S
    ]
    if not cutoff_indices:
        raise WarmupValidationError("no causal state sample exists at the cutoff")
    cutoff_index = cutoff_indices[-1]
    cutoff_sample_time = times[cutoff_index]
    cutoff_lag = max(0.0, cutoff - cutoff_sample_time)
    if cutoff_lag > sample_dt + grid_tolerance:
        raise WarmupValidationError("cutoff state sample is stale")
    cutoff_state_id = states[cutoff_index]
    cutoff_state_known = cutoff_state_id in STATE_NAMES
    cutoff_state_non_failure = (
        cutoff_state_known and cutoff_state_id not in FAILURE_STATE_IDS
    )

    gait_events, timeout_times = _validated_detector_events(
        detector_accepted_events,
        analysis_start_s=analysis_start,
        sample_end_s=times[-1],
    )
    invalid_times = _validated_invalid_times(
        detector_invalid_steps,
        analysis_start_s=analysis_start,
        sample_end_s=times[-1],
    )
    detector_cycles = _detector_cycles(
        gait_events,
        analysis_start_s=analysis_start,
        cutoff_s=cutoff,
    )
    prescribed_cycles = _prescribed_cycles(
        prescribed_events,
        analysis_start_s=analysis_start,
        cutoff_s=cutoff,
        first_scoreable_hs_s=first_scoreable_hs,
    )

    warmup_indices = [
        index
        for index, time_s in enumerate(times)
        if analysis_start - NUMERIC_TOLERANCE_S
        <= time_s
        <= cutoff + NUMERIC_TOLERANCE_S
    ]
    timeout_state_sample_count = sum(
        states[index] == TIMEOUT for index in warmup_indices
    )
    invalid_state_sample_count = sum(
        states[index] == INVALID_EVENT for index in warmup_indices
    )
    unknown_state_sample_count = sum(
        states[index] not in STATE_NAMES for index in warmup_indices
    )
    scoped_timeout_times = [
        item
        for item in timeout_times
        if analysis_start - NUMERIC_TOLERANCE_S
        <= item
        <= cutoff + NUMERIC_TOLERANCE_S
    ]
    scoped_invalid_times = [
        item
        for item in invalid_times
        if analysis_start - NUMERIC_TOLERANCE_S
        <= item
        <= cutoff + NUMERIC_TOLERANCE_S
    ]
    checks = {
        "detector_complete_hs_to_toe_off_to_hs_cycle": bool(detector_cycles),
        "prescribed_complete_hs_to_toe_off_to_hs_cycle": bool(
            prescribed_cycles
        ),
        "zero_timeout_transitions": not scoped_timeout_times
        and timeout_state_sample_count == 0,
        "zero_invalid_transitions": not scoped_invalid_times
        and invalid_state_sample_count == 0
        and unknown_state_sample_count == 0,
        "cutoff_state_known": cutoff_state_known,
        "cutoff_state_non_failure": cutoff_state_non_failure,
    }
    ok = bool(all(checks.values()))
    return {
        "schema_version": CERTIFICATE_SCHEMA_VERSION,
        "status": "PASS_WARMUP_CERTIFIED" if ok else "FAIL_WARMUP_NOT_CERTIFIED",
        "ok": ok,
        "analysis_start_s": analysis_start,
        "first_scoreable_hs_s": first_scoreable_hs,
        "warmup_guard_s": WARMUP_GUARD_S,
        "cutoff_s": cutoff,
        "sample_dt_s": sample_dt,
        "checks": checks,
        "detector": {
            "cycle_time_field": "confirmed_time_s",
            "complete_cycle_count": len(detector_cycles),
            "complete_cycles": detector_cycles,
            "timeout_transition_times_s": scoped_timeout_times,
            "timeout_state_sample_count": timeout_state_sample_count,
        },
        "prescribed": {
            "complete_cycle_count": len(prescribed_cycles),
            "complete_cycles": prescribed_cycles,
        },
        "invalid": {
            "transition_times_s": scoped_invalid_times,
            "invalid_state_sample_count": invalid_state_sample_count,
            "unknown_state_sample_count": unknown_state_sample_count,
        },
        "cutoff_state": {
            "sample_index": cutoff_index,
            "sample_time_s": cutoff_sample_time,
            "sample_lag_s": cutoff_lag,
            "state_id": cutoff_state_id,
            "state_name": STATE_NAMES.get(cutoff_state_id, "UNKNOWN"),
            "known": cutoff_state_known,
            "non_failure": cutoff_state_non_failure,
        },
    }


def _validated_certificate_ok(certificate: Any) -> bool:
    if not isinstance(certificate, Mapping):
        raise WarmupValidationError("warm-up certificate must be an object")
    if certificate.get("schema_version") != CERTIFICATE_SCHEMA_VERSION:
        raise WarmupValidationError("warm-up certificate schema drifted")
    if not isinstance(certificate.get("ok"), bool):
        raise WarmupValidationError("warm-up certificate ok flag is not boolean")
    checks = certificate.get("checks")
    if not isinstance(checks, Mapping):
        raise WarmupValidationError("warm-up certificate checks are missing")
    if any(not isinstance(checks.get(key), bool) for key in REQUIRED_CERTIFICATE_CHECKS):
        raise WarmupValidationError("warm-up certificate checks are incomplete")
    analysis_start = _finite_float(
        certificate.get("analysis_start_s"), "certificate.analysis_start_s"
    )
    first_scoreable = _finite_float(
        certificate.get("first_scoreable_hs_s"),
        "certificate.first_scoreable_hs_s",
    )
    guard = _finite_float(
        certificate.get("warmup_guard_s"), "certificate.warmup_guard_s"
    )
    cutoff = _finite_float(certificate.get("cutoff_s"), "certificate.cutoff_s")
    sample_dt = _finite_float(
        certificate.get("sample_dt_s"), "certificate.sample_dt_s"
    )
    if (
        not math.isclose(
            guard, WARMUP_GUARD_S, rel_tol=0.0, abs_tol=NUMERIC_TOLERANCE_S
        )
        or not math.isclose(
            cutoff,
            first_scoreable - guard,
            rel_tol=0.0,
            abs_tol=NUMERIC_TOLERANCE_S,
        )
        or cutoff <= analysis_start + NUMERIC_TOLERANCE_S
        or sample_dt <= 0.0
    ):
        raise WarmupValidationError("warm-up certificate interval is inconsistent")

    section_counts: dict[str, int] = {}
    section_cycles: dict[str, list[Any]] = {}
    for section in ("detector", "prescribed"):
        payload = certificate.get(section)
        if not isinstance(payload, Mapping):
            raise WarmupValidationError(
                f"warm-up certificate {section} evidence is missing"
            )
        count = payload.get("complete_cycle_count")
        cycles = payload.get("complete_cycles")
        if isinstance(count, bool) or not isinstance(count, int) or count < 0:
            raise WarmupValidationError(
                f"warm-up certificate {section} cycle count is invalid"
            )
        if not isinstance(cycles, list) or count != len(cycles):
            raise WarmupValidationError(
                f"warm-up certificate {section} cycle evidence is inconsistent"
            )
        section_counts[section] = count
        section_cycles[section] = cycles

    cycle_fields = {
        "detector": (
            "opening_hs_confirmed_s",
            "toe_off_confirmed_s",
            "closing_hs_confirmed_s",
        ),
        "prescribed": ("opening_hs_s", "toe_off_s", "closing_hs_s"),
    }
    for section, cycles in section_cycles.items():
        fields = cycle_fields[section]
        previous_closing: float | None = None
        for index, cycle in enumerate(cycles):
            if not isinstance(cycle, Mapping) or set(cycle) != set(fields):
                raise WarmupValidationError(
                    f"warm-up certificate {section} cycle schema is invalid"
                )
            opening, toe_off, closing = (
                _finite_float(
                    cycle[field], f"certificate.{section}.cycles[{index}].{field}"
                )
                for field in fields
            )
            if not (
                analysis_start - NUMERIC_TOLERANCE_S
                <= opening
                < toe_off
                < closing
                <= cutoff + NUMERIC_TOLERANCE_S
            ):
                raise WarmupValidationError(
                    f"warm-up certificate {section} cycle order/bounds are invalid"
                )
            if previous_closing is not None and not math.isclose(
                opening,
                previous_closing,
                rel_tol=0.0,
                abs_tol=NUMERIC_TOLERANCE_S,
            ):
                raise WarmupValidationError(
                    f"warm-up certificate {section} cycles are not contiguous"
                )
            previous_closing = closing

    detector = certificate["detector"]
    if detector.get("cycle_time_field") != "confirmed_time_s":
        raise WarmupValidationError("detector warm-up time field drifted")
    timeout_times = detector.get("timeout_transition_times_s")
    timeout_samples = detector.get("timeout_state_sample_count")
    if not isinstance(timeout_times, list):
        raise WarmupValidationError("detector timeout evidence is invalid")
    for index, item in enumerate(timeout_times):
        timestamp = _finite_float(
            item, f"certificate.detector.timeout_transition_times_s[{index}]"
        )
        if not (
            analysis_start - NUMERIC_TOLERANCE_S
            <= timestamp
            <= cutoff + NUMERIC_TOLERANCE_S
        ):
            raise WarmupValidationError("detector timeout evidence is invalid")
    if (
        isinstance(timeout_samples, bool)
        or not isinstance(timeout_samples, int)
        or timeout_samples < 0
    ):
        raise WarmupValidationError("detector timeout sample count is invalid")

    invalid = certificate.get("invalid")
    if not isinstance(invalid, Mapping):
        raise WarmupValidationError("warm-up invalid-transition evidence is missing")
    invalid_times = invalid.get("transition_times_s")
    invalid_samples = invalid.get("invalid_state_sample_count")
    unknown_samples = invalid.get("unknown_state_sample_count")
    if not isinstance(invalid_times, list):
        raise WarmupValidationError("invalid-transition evidence is invalid")
    for index, item in enumerate(invalid_times):
        timestamp = _finite_float(
            item, f"certificate.invalid.transition_times_s[{index}]"
        )
        if not (
            analysis_start - NUMERIC_TOLERANCE_S
            <= timestamp
            <= cutoff + NUMERIC_TOLERANCE_S
        ):
            raise WarmupValidationError("invalid-transition evidence is invalid")
    if (
        isinstance(invalid_samples, bool)
        or not isinstance(invalid_samples, int)
        or invalid_samples < 0
    ):
        raise WarmupValidationError("invalid-state sample count is invalid")
    if (
        isinstance(unknown_samples, bool)
        or not isinstance(unknown_samples, int)
        or unknown_samples < 0
    ):
        raise WarmupValidationError("unknown-state sample count is invalid")

    cutoff_state = certificate.get("cutoff_state")
    if not isinstance(cutoff_state, Mapping):
        raise WarmupValidationError("warm-up certificate cutoff state is missing")
    state_numeric = _finite_float(
        cutoff_state.get("state_id"), "certificate.cutoff_state.state_id"
    )
    state_id = int(round(state_numeric))
    if not math.isclose(
        state_numeric,
        float(state_id),
        rel_tol=0.0,
        abs_tol=NUMERIC_TOLERANCE_S,
    ):
        raise WarmupValidationError("certificate cutoff state ID is not integral")
    known = state_id in STATE_NAMES
    non_failure = known and state_id not in FAILURE_STATE_IDS
    if cutoff_state.get("known") is not known:
        raise WarmupValidationError("cutoff-state known flag is inconsistent")
    if cutoff_state.get("non_failure") is not non_failure:
        raise WarmupValidationError("cutoff-state failure flag is inconsistent")
    state_sample_time = _finite_float(
        cutoff_state.get("sample_time_s"),
        "certificate.cutoff_state.sample_time_s",
    )
    state_lag = _finite_float(
        cutoff_state.get("sample_lag_s"), "certificate.cutoff_state.sample_lag_s"
    )
    if (
        state_sample_time > cutoff + NUMERIC_TOLERANCE_S
        or state_lag < 0.0
        or state_lag > sample_dt + max(NUMERIC_TOLERANCE_S, sample_dt * 1.0e-9)
        or not math.isclose(
            state_lag,
            max(0.0, cutoff - state_sample_time),
            rel_tol=0.0,
            abs_tol=max(NUMERIC_TOLERANCE_S, sample_dt * 1.0e-9),
        )
    ):
        raise WarmupValidationError("certificate cutoff sample is inconsistent")

    derived_checks = {
        "detector_complete_hs_to_toe_off_to_hs_cycle": (
            section_counts["detector"] >= 1
        ),
        "prescribed_complete_hs_to_toe_off_to_hs_cycle": (
            section_counts["prescribed"] >= 1
        ),
        "zero_timeout_transitions": not timeout_times and timeout_samples == 0,
        "zero_invalid_transitions": (
            not invalid_times and invalid_samples == 0 and unknown_samples == 0
        ),
        "cutoff_state_known": known,
        "cutoff_state_non_failure": non_failure,
    }
    if any(checks[key] is not value for key, value in derived_checks.items()):
        raise WarmupValidationError("warm-up certificate checks are inconsistent")
    derived_ok = bool(all(derived_checks.values()))
    if certificate["ok"] != derived_ok:
        raise WarmupValidationError("warm-up certificate decision is inconsistent")
    expected_status = (
        "PASS_WARMUP_CERTIFIED" if derived_ok else "FAIL_WARMUP_NOT_CERTIFIED"
    )
    if certificate.get("status") != expected_status:
        raise WarmupValidationError("warm-up certificate status is inconsistent")
    return derived_ok


def apply_warmup_certificate(
    row: Mapping[str, Any],
    certificate: Mapping[str, Any],
    *,
    is_v13: bool,
    protocol_error_type: type[Exception] | None = None,
) -> dict[str, Any]:
    """Return a row with the warm-up decision applied fail-closed.

    A V13 failure invalidates the comparison itself and raises the supplied
    protocol exception (``RuntimeError`` by default).  A non-baseline failure
    remains a valid observation but cannot pass the unit full gate.
    """

    if not isinstance(row, Mapping):
        raise WarmupValidationError("unit row must be an object")
    if not isinstance(is_v13, bool):
        raise WarmupValidationError("is_v13 must be boolean")
    error_type = RuntimeError if protocol_error_type is None else protocol_error_type
    if not isinstance(error_type, type) or not issubclass(error_type, Exception):
        raise WarmupValidationError("protocol_error_type must be an exception type")
    warmup_ok = _validated_certificate_ok(certificate)
    if is_v13 and not warmup_ok:
        raise error_type("V13 warm-up certification failed")
    result = copy.deepcopy(dict(row))
    result["warmup_ok"] = warmup_ok
    if not warmup_ok:
        result["unit_full_gate_ok"] = False
    return result


def root_safe_warmup_ok(rows: Sequence[Mapping[str, Any]]) -> bool:
    """Return true only for a non-empty unit set with explicit warm-up PASS."""

    if isinstance(rows, (str, bytes)) or not isinstance(rows, Sequence):
        raise WarmupValidationError("root-safety rows must be a sequence")
    return bool(rows) and all(
        isinstance(row, Mapping) and row.get("warmup_ok") is True for row in rows
    )


def apply_root_safe_warmup(
    root_safety: Mapping[str, Any],
    rows: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    """Conjoin a V14 root-safety result with explicit warm-up evidence."""

    if not isinstance(root_safety, Mapping):
        raise WarmupValidationError("root_safety must be an object")
    if not isinstance(root_safety.get("ok"), bool):
        raise WarmupValidationError("root_safety ok flag must be boolean")
    row_list = _sequence(rows, "root-safety rows")
    units: list[str] = []
    for index, row in enumerate(row_list):
        if not isinstance(row, Mapping):
            raise WarmupValidationError(
                f"root-safety rows[{index}] must be an object"
            )
        unit = row.get("v14_stage_unit", f"index_{index}")
        unit_text = str(unit)
        if unit_text in units:
            raise WarmupValidationError("root-safety warm-up units are duplicated")
        units.append(unit_text)
    warmup_ok = root_safe_warmup_ok(row_list)
    result = copy.deepcopy(dict(root_safety))
    result["warmup"] = {
        "ok": warmup_ok,
        "required_for_root_safety": True,
        "unit_count": len(row_list),
        "failed_or_missing_units": [
            unit
            for unit, row in zip(units, row_list)
            if row.get("warmup_ok") is not True
        ],
    }
    result["ok"] = bool(result["ok"] and warmup_ok)
    return result
