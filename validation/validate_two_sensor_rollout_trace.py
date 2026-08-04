"""Validate and plot heel/toe detector evidence saved by a policy rollout.

The input is the ``rollout_policy_trace.json`` emitted by
``Trajectory Generator/baseline_MLP/rollout_eval.py`` with
``--record-policy-trace``.  The validator is independent from OpenSim and
RLlib: it uses matplotlib plus the repository's pure-Python
``ProstheticPhaseFSM`` (and its NumPy dependency), so a completed rollout can
be audited on macOS or Windows without loading a model or checkpoint.

For a ``shadow`` trace, recorded detector events are not treated as the true
two-sensor counterfactual because their FSM state was driven by legacy events.
The script replays the raw heel/toe loads through the same production FSM in
``two_sensor`` mode, with no legacy events, and uses that replay for the hard
checks and plot.  The original shadow and legacy streams remain in the JSON
for comparison.

Outputs
-------
``two_sensor_detector_metrics.json``
    Machine-readable event counts, timestamps, trace completeness, and every
    hard semantic check (including violation examples).
``01_two_sensor_detector_trace.png``
    Served prosthetic kinematics, regional normal loads and thresholds,
    debounced contact latches, and legacy/sensor/FSM-accepted events.

Exit status is zero only when every hard check passes.  Plots and metrics are
still written on failure so the reason can be inspected visually.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys
import tempfile
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence


os.environ.setdefault(
    "MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "cmc_like_matplotlib")
)
os.environ.setdefault(
    "XDG_CACHE_HOME", str(Path(tempfile.gettempdir()) / "cmc_like_cache")
)

import matplotlib


matplotlib.use("Agg")
import matplotlib.pyplot as plt


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
TRAJECTORY_GENERATOR_DIR = REPO_ROOT / "Trajectory Generator"
if str(TRAJECTORY_GENERATOR_DIR) not in sys.path:
    sys.path.insert(0, str(TRAJECTORY_GENERATOR_DIR))

from prosthetic_phase_fsm import (  # noqa: E402
    ProstheticPhaseFSM,
    ProstheticPhaseFSMConfig,
)


SCHEMA_VERSION = 2
DEFAULT_ON_THRESHOLD_N = 5.0
DEFAULT_OFF_THRESHOLD_N = 2.0
DEFAULT_DWELL_S = 0.03
ALLOWED_EVENT_SOURCES = {"legacy_events", "shadow", "two_sensor"}


@dataclass(frozen=True)
class EventRecord:
    stream: str
    name: str
    row_index: int
    row_time_s: float
    event_time_s: float
    payload: Mapping[str, Any]


@dataclass(frozen=True)
class TraceSample:
    index: int
    time_s: float
    knee_served_rad: float
    ankle_served_rad: float
    knee_actual_rad: float
    ankle_actual_rad: float
    heel_load_n: float
    toe_load_n: float
    heel_contact: bool
    toe_contact: bool
    event_source: str
    aggregate_normal_force_bw: float
    aggregate_in_contact: bool | None
    legacy_events: tuple[EventRecord, ...]
    sensor_events: tuple[EventRecord, ...]
    accepted_events: tuple[EventRecord, ...]
    recorded_sensor_events: tuple[EventRecord, ...]
    recorded_accepted_events: tuple[EventRecord, ...]


def _finite_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _mapping(value: Any) -> Mapping[str, Any] | None:
    return value if isinstance(value, Mapping) else None


def _sequence(value: Any) -> Sequence[Any] | None:
    if isinstance(value, Sequence) and not isinstance(
        value, (str, bytes, bytearray)
    ):
        return value
    return None


def _normalise_event_name(value: Any) -> str:
    token = str(value or "").strip().lower().replace("-", "_").replace(" ", "_")
    if token in {"hs", "heelstrike", "heel_strike"}:
        return "heel_strike"
    if token in {"to", "toeoff", "toe_off"}:
        return "toe_off"
    return token or "unknown"


def _left_event(payload: Mapping[str, Any]) -> bool:
    side = str(payload.get("side", "left")).strip().lower()
    return side in {"", "l", "left"}


def _event_time(
    payload: Mapping[str, Any],
    *,
    stream: str,
    row_time_s: float,
) -> float:
    keys = (
        ("event_time_s", "time", "event_time")
        if stream == "accepted"
        else ("time", "event_time_s", "event_time")
    )
    for key in keys:
        value = _finite_float(payload.get(key))
        if value is not None:
            return value
    return row_time_s


def _events(
    raw: Any,
    *,
    stream: str,
    row_index: int,
    row_time_s: float,
) -> tuple[EventRecord, ...]:
    values = _sequence(raw)
    if values is None:
        return ()
    result: list[EventRecord] = []
    for item in values:
        payload = _mapping(item)
        if payload is None or not _left_event(payload):
            continue
        result.append(
            EventRecord(
                stream=stream,
                name=_normalise_event_name(payload.get("event")),
                row_index=row_index,
                row_time_s=row_time_s,
                event_time_s=_event_time(
                    payload,
                    stream=stream,
                    row_time_s=row_time_s,
                ),
                payload=payload,
            )
        )
    return tuple(result)


def _sensor_mapping(
    sensors: Mapping[str, Any], aliases: Iterable[str]
) -> Mapping[str, Any] | None:
    for name in aliases:
        value = _mapping(sensors.get(name))
        if value is not None:
            return value
    return None


def _load_from_sensor(sensor: Mapping[str, Any] | None) -> float:
    if sensor is None:
        return math.nan
    for key in ("normal_load_n", "normal_force_n", "load_n", "force_n"):
        value = _finite_float(sensor.get(key))
        if value is not None:
            return value
    return math.nan


def _state_value(state: Mapping[str, Any], key: str) -> float:
    value = _finite_float(state.get(key))
    return value if value is not None else math.nan


def _actor_observation(raw_row: Any) -> Mapping[str, Any]:
    row = _mapping(raw_row)
    if row is None:
        return {}
    value = _mapping(row.get("actor_observation_before"))
    return value if value is not None else {}


def _aligned_aggregate_evidence(
    rows: Sequence[Any], index: int
) -> tuple[float, bool | None]:
    """Recover the aggregate detector evidence aligned with one trace row.

    ``actor_observation_before`` belongs to the state immediately before that
    row's action.  Therefore the actor observation in row ``i + 1`` is aligned
    with the post-step detector sample saved in row ``i``.  The last row has no
    successor and deliberately remains unavailable instead of silently using a
    one-step-old value.
    """
    if index + 1 >= len(rows):
        return math.nan, None
    actor = _actor_observation(rows[index + 1])
    normal_force_bw = _finite_float(actor.get("online_left_normal_grf_bw"))
    in_contact_raw = _finite_float(actor.get("online_left_in_contact"))
    return (
        normal_force_bw if normal_force_bw is not None else math.nan,
        None if in_contact_raw is None else bool(in_contact_raw >= 0.5),
    )


def _parse_trace(rows: Sequence[Any]) -> tuple[list[TraceSample], list[dict[str, Any]]]:
    samples: list[TraceSample] = []
    schema_violations: list[dict[str, Any]] = []
    for index, raw_row in enumerate(rows):
        row = _mapping(raw_row)
        if row is None:
            schema_violations.append(
                {"row_index": index, "reason": "trace row is not a JSON object"}
            )
            continue

        time_s = _finite_float(row.get("time"))
        state = _mapping(row.get("prosthetic_state"))
        sensors = _mapping(row.get("detector_sensors"))
        fsm = _mapping(row.get("phase_fsm"))
        legacy_raw = _sequence(row.get("legacy_online_events"))

        missing: list[str] = []
        if time_s is None:
            missing.append("time")
            time_s = float(index)
        if state is None:
            missing.append("prosthetic_state")
            state = {}
        if sensors is None:
            missing.append("detector_sensors")
            sensors = {}
        if fsm is None:
            missing.append("phase_fsm")
            fsm = {}
        if legacy_raw is None:
            missing.append("legacy_online_events")
            legacy_raw = ()
        if _sequence(fsm.get("sensor_events_this_step")) is None:
            missing.append("phase_fsm.sensor_events_this_step")
        if _sequence(fsm.get("accepted_transitions_this_step")) is None:
            missing.append("phase_fsm.accepted_transitions_this_step")

        heel_sensor = _sensor_mapping(
            sensors, ("left_heel", "heel_left", "heel", "prosthetic_heel")
        )
        toe_sensor = _sensor_mapping(
            sensors, ("left_toe", "toe_left", "toe", "prosthetic_toe")
        )
        heel_load_n = _load_from_sensor(heel_sensor)
        toe_load_n = _load_from_sensor(toe_sensor)
        if not math.isfinite(heel_load_n):
            missing.append("detector_sensors.left_heel.normal_load_n")
        if not math.isfinite(toe_load_n):
            missing.append("detector_sensors.left_toe.normal_load_n")

        heel_contact_raw = _finite_float(fsm.get("sensor_heel_contact"))
        toe_contact_raw = _finite_float(fsm.get("sensor_toe_contact"))
        if heel_contact_raw is None:
            missing.append("phase_fsm.sensor_heel_contact")
            heel_contact_raw = 0.0
        if toe_contact_raw is None:
            missing.append("phase_fsm.sensor_toe_contact")
            toe_contact_raw = 0.0

        event_source = str(fsm.get("event_source", "")).strip().lower()
        if event_source not in ALLOWED_EVENT_SOURCES:
            missing.append("phase_fsm.event_source")

        knee_served = _state_value(state, "pros_knee_angle_served_ref")
        ankle_served = _state_value(state, "pros_ankle_angle_served_ref")
        if not math.isfinite(knee_served):
            missing.append("prosthetic_state.pros_knee_angle_served_ref")
        if not math.isfinite(ankle_served):
            missing.append("prosthetic_state.pros_ankle_angle_served_ref")

        if missing:
            schema_violations.append(
                {
                    "row_index": index,
                    "time_s": time_s,
                    "reason": "missing or invalid required fields",
                    "fields": sorted(set(missing)),
                }
            )

        sensor_events = _events(
            fsm.get("sensor_events_this_step"),
            stream="sensor",
            row_index=index,
            row_time_s=time_s,
        )
        accepted_events = _events(
            fsm.get("accepted_transitions_this_step"),
            stream="accepted",
            row_index=index,
            row_time_s=time_s,
        )
        aggregate_force_bw, aggregate_in_contact = _aligned_aggregate_evidence(
            rows, index
        )

        samples.append(
            TraceSample(
                index=index,
                time_s=time_s,
                knee_served_rad=knee_served,
                ankle_served_rad=ankle_served,
                knee_actual_rad=_state_value(state, "pros_knee_angle"),
                ankle_actual_rad=_state_value(state, "pros_ankle_angle"),
                heel_load_n=heel_load_n,
                toe_load_n=toe_load_n,
                heel_contact=bool(heel_contact_raw >= 0.5),
                toe_contact=bool(toe_contact_raw >= 0.5),
                event_source=event_source,
                aggregate_normal_force_bw=aggregate_force_bw,
                aggregate_in_contact=aggregate_in_contact,
                legacy_events=_events(
                    legacy_raw,
                    stream="legacy",
                    row_index=index,
                    row_time_s=time_s,
                ),
                sensor_events=sensor_events,
                accepted_events=accepted_events,
                recorded_sensor_events=sensor_events,
                recorded_accepted_events=accepted_events,
            )
        )
    return samples, schema_violations


def _regional_body_weight_estimate(samples: Sequence[TraceSample]) -> float | None:
    estimates: list[float] = []
    for sample in samples:
        regional_load_n = max(0.0, sample.heel_load_n) + max(
            0.0, sample.toe_load_n
        )
        if (
            regional_load_n > 1e-6
            and math.isfinite(sample.aggregate_normal_force_bw)
            and sample.aggregate_normal_force_bw > 1e-6
        ):
            estimate = regional_load_n / sample.aggregate_normal_force_bw
            if math.isfinite(estimate) and estimate > 1e-6:
                estimates.append(estimate)
    return float(statistics.median(estimates)) if estimates else None


def _offline_two_sensor_replay(
    samples: Sequence[TraceSample],
    *,
    on_threshold_n: float,
    off_threshold_n: float,
    dwell_s: float,
    fsm_config_overrides: Mapping[str, float] | None = None,
) -> tuple[list[TraceSample], dict[str, Any]]:
    """Replay raw shadow loads through the production two-sensor FSM.

    The recorded shadow FSM is intentionally *not* used as the detector
    counterfactual: its state transitions are driven by legacy events.  This
    replay instantiates the exact same pure-Python FSM class with
    ``event_source='two_sensor'`` and gives it no legacy events.
    """
    config_values: dict[str, Any] = dict(fsm_config_overrides or {})
    config_values.update(
        {
            "event_source": "two_sensor",
            "sensor_on_threshold_n": float(on_threshold_n),
            "sensor_off_threshold_n": float(off_threshold_n),
            "sensor_dwell_s": float(dwell_s),
        }
    )
    config = ProstheticPhaseFSMConfig(**config_values)
    fsm = ProstheticPhaseFSM(config)
    body_weight_estimate_n = _regional_body_weight_estimate(samples)
    replayed: list[TraceSample] = []
    aligned_force_steps = 0
    fallback_force_steps = 0
    aligned_contact_steps = 0
    fallback_contact_steps = 0

    for sample in samples:
        if math.isfinite(sample.aggregate_normal_force_bw):
            normal_force_bw = max(0.0, sample.aggregate_normal_force_bw)
            aligned_force_steps += 1
        elif body_weight_estimate_n is not None:
            normal_force_bw = (
                max(0.0, sample.heel_load_n) + max(0.0, sample.toe_load_n)
            ) / body_weight_estimate_n
            fallback_force_steps += 1
        else:
            # Current production transition gates use zero minima for force
            # evidence.  Keeping a finite zero here therefore preserves event
            # transitions while explicitly marking the diagnostic fallback.
            normal_force_bw = 0.0
            fallback_force_steps += 1

        if sample.aggregate_in_contact is not None:
            in_contact = bool(sample.aggregate_in_contact)
            aligned_contact_steps += 1
        else:
            in_contact = bool(
                sample.heel_load_n > 0.0 or sample.toe_load_n > 0.0
            )
            fallback_contact_steps += 1

        knee = (
            sample.knee_actual_rad
            if math.isfinite(sample.knee_actual_rad)
            else sample.knee_served_rad
        )
        ankle = (
            sample.ankle_actual_rad
            if math.isfinite(sample.ankle_actual_rad)
            else sample.ankle_served_rad
        )
        payload = fsm.update(
            time_s=sample.time_s,
            events=(),
            normal_force_bw=normal_force_bw,
            in_contact=in_contact,
            prosthetic_knee_angle_rad=knee,
            prosthetic_ankle_angle_rad=ankle,
            heel_normal_force_n=sample.heel_load_n,
            toe_normal_force_n=sample.toe_load_n,
        )
        sensor_events = _events(
            payload.get("sensor_events_this_step"),
            stream="sensor",
            row_index=sample.index,
            row_time_s=sample.time_s,
        )
        accepted_events = _events(
            payload.get("accepted_transitions_this_step"),
            stream="accepted",
            row_index=sample.index,
            row_time_s=sample.time_s,
        )
        replayed.append(
            replace(
                sample,
                heel_contact=bool(
                    float(payload.get("sensor_heel_contact", 0.0) or 0.0)
                    >= 0.5
                ),
                toe_contact=bool(
                    float(payload.get("sensor_toe_contact", 0.0) or 0.0)
                    >= 0.5
                ),
                sensor_events=sensor_events,
                accepted_events=accepted_events,
            )
        )

    provenance = {
        "applied": True,
        "reason": (
            "recorded shadow sensor events share a state machine driven by "
            "legacy events; hard checks use an offline counterfactual replay"
        ),
        "authority_for_hard_checks_and_plot": "offline_two_sensor_fsm_replay",
        "production_class": "prosthetic_phase_fsm.ProstheticPhaseFSM",
        "production_config": asdict(config),
        "production_config_source": (
            "rollout_summary.reward_config mapped exactly as "
            "baseline_MLP/env_factory.py"
            if fsm_config_overrides
            else "ProstheticPhaseFSMConfig defaults"
        ),
        "legacy_events_supplied_to_replay": 0,
        "replay_step_count": len(replayed),
        "input_sampling": "one replay update per recorded policy-trace row",
        "raw_inputs": [
            "time",
            "detector_sensors.left_heel.normal_load_n",
            "detector_sensors.left_toe.normal_load_n",
            "prosthetic_state.pros_knee_angle",
            "prosthetic_state.pros_ankle_angle",
        ],
        "aggregate_force_evidence": {
            "aligned_steps": aligned_force_steps,
            "fallback_steps": fallback_force_steps,
            "alignment_method": (
                "actor_observation_before in row i+1 is the post-step state "
                "saved by detector loads in row i"
            ),
            "regional_body_weight_estimate_n": body_weight_estimate_n,
        },
        "aggregate_contact_evidence": {
            "aligned_steps": aligned_contact_steps,
            "fallback_steps": fallback_contact_steps,
            "fallback": "regional heel/toe load greater than zero",
        },
        "limitations": [
            "The reset-time regional loads are not present in the trace; replay "
            "starts at the first saved post-step sample.",
            "Only detector samples retained at policy-step boundaries can be "
            "replayed; sub-step excursions would require a denser trace.",
        ],
    }
    return replayed, provenance


def _violation(event: EventRecord, reason: str, **details: Any) -> dict[str, Any]:
    result = {
        "row_index": event.row_index,
        "row_time_s": event.row_time_s,
        "event_time_s": event.event_time_s,
        "event": event.name,
        "reason": reason,
    }
    result.update(details)
    return result


def _check_result(
    name: str,
    description: str,
    violations: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    return {
        "name": name,
        "description": description,
        "pass": not violations,
        "violation_count": len(violations),
        "violations": [dict(item) for item in violations[:25]],
        "violations_truncated": max(0, len(violations) - 25),
    }


def _semantic_event_sequence(
    events: Sequence[EventRecord], *, stream: str
) -> list[dict[str, Any]]:
    relevant = [
        event for event in events if event.name in {"heel_strike", "toe_off"}
    ]
    violations: list[dict[str, Any]] = []
    previous: EventRecord | None = None
    for event in relevant:
        if previous is not None:
            if event.event_time_s + 1e-12 < previous.event_time_s:
                violations.append(
                    _violation(
                        event,
                        f"{stream} event timestamps are not monotonic",
                        previous_event=previous.name,
                        previous_event_time_s=previous.event_time_s,
                    )
                )
            if event.name == previous.name:
                violations.append(
                    _violation(
                        event,
                        f"{stream} HS/TO events do not alternate",
                        previous_event=previous.name,
                        previous_event_time_s=previous.event_time_s,
                    )
                )
        previous = event
    return violations


def _nearest_heel_rise_distance(
    samples: Sequence[TraceSample], event_time_s: float
) -> float | None:
    rises: list[float] = []
    previous = False
    for sample in samples:
        if sample.heel_contact and not previous:
            rises.append(sample.time_s)
        previous = sample.heel_contact
    if not rises:
        return None
    return min(abs(value - event_time_s) for value in rises)


def _run_checks(
    samples: Sequence[TraceSample],
    schema_violations: Sequence[Mapping[str, Any]],
    *,
    on_threshold_n: float,
    alignment_tolerance_s: float,
) -> list[dict[str, Any]]:
    sensor_events = [event for sample in samples for event in sample.sensor_events]
    accepted_events = [
        event for sample in samples for event in sample.accepted_events
    ]
    by_index = {sample.index: sample for sample in samples}

    checks: list[dict[str, Any]] = [
        _check_result(
            "trace_schema_complete",
            "Every row contains time, served angles, both sensor loads, FSM "
            "contacts/source, and all three event streams.",
            schema_violations,
        )
    ]

    toe_only_violations: list[dict[str, Any]] = []
    sensor_hs_violations: list[dict[str, Any]] = []
    for event in sensor_events:
        if event.name != "heel_strike":
            continue
        sample = by_index[event.row_index]
        if sample.toe_contact and not sample.heel_contact:
            toe_only_violations.append(
                _violation(
                    event,
                    "toe-only contact emitted heel_strike",
                    heel_contact=sample.heel_contact,
                    toe_contact=sample.toe_contact,
                    heel_load_n=sample.heel_load_n,
                    toe_load_n=sample.toe_load_n,
                )
            )
        confirmed_time = _finite_float(event.payload.get("confirmed_time"))
        onset_time = _finite_float(event.payload.get("time"))
        causal_times = (
            confirmed_time is not None
            and onset_time is not None
            and onset_time <= confirmed_time + 1e-12
            and confirmed_time <= event.row_time_s + alignment_tolerance_s
        )
        if (
            not sample.heel_contact
            or sample.heel_load_n + 1e-12 < on_threshold_n
            or not causal_times
        ):
            sensor_hs_violations.append(
                _violation(
                    event,
                    "sensor heel_strike lacks stable heel-contact/onset evidence",
                    heel_contact=sample.heel_contact,
                    heel_load_n=sample.heel_load_n,
                    on_threshold_n=on_threshold_n,
                    onset_time_s=onset_time,
                    confirmed_time_s=confirmed_time,
                )
            )

    checks.extend(
        (
            _check_result(
                "toe_only_never_generates_hs",
                "A forefoot-only contact may be diagnostic but cannot create HS.",
                toe_only_violations,
            ),
            _check_result(
                "sensor_hs_requires_heel_contact_and_onset",
                "Every detector HS has a stable heel latch, heel load at the ON "
                "threshold, and causal onset/confirmation timestamps.",
                sensor_hs_violations,
            ),
        )
    )

    accepted_hs_violations: list[dict[str, Any]] = []
    for event in accepted_events:
        if event.name != "heel_strike":
            continue
        sample = by_index[event.row_index]
        same_row_sensor_hs = [
            candidate
            for candidate in sample.sensor_events
            if candidate.name == "heel_strike"
            and abs(candidate.event_time_s - event.event_time_s)
            <= alignment_tolerance_s
        ]
        rise_distance = _nearest_heel_rise_distance(samples, event.event_time_s)
        onset_evidence = bool(same_row_sensor_hs) or (
            rise_distance is not None
            and rise_distance <= alignment_tolerance_s
        )
        if not sample.heel_contact or not onset_evidence:
            accepted_hs_violations.append(
                _violation(
                    event,
                    "accepted HS lacks heel contact or a temporally aligned heel onset",
                    event_source=sample.event_source,
                    heel_contact=sample.heel_contact,
                    toe_contact=sample.toe_contact,
                    heel_load_n=sample.heel_load_n,
                    nearest_heel_rise_distance_s=rise_distance,
                    same_row_sensor_hs=bool(same_row_sensor_hs),
                    alignment_tolerance_s=alignment_tolerance_s,
                )
            )
    checks.append(
        _check_result(
            "accepted_hs_requires_heel_contact_and_onset",
            "An FSM-accepted HS must coincide with heel contact and a heel onset.",
            accepted_hs_violations,
        )
    )

    sensor_to_violations: list[dict[str, Any]] = []
    for event in sensor_events:
        if event.name != "toe_off":
            continue
        sample = by_index[event.row_index]
        if sample.heel_contact or sample.toe_contact:
            sensor_to_violations.append(
                _violation(
                    event,
                    "sensor toe_off emitted before both contacts were stably OFF",
                    heel_contact=sample.heel_contact,
                    toe_contact=sample.toe_contact,
                    heel_load_n=sample.heel_load_n,
                    toe_load_n=sample.toe_load_n,
                )
            )
    checks.append(
        _check_result(
            "sensor_to_requires_both_contacts_off",
            "A detector TO is emitted only after both debounced sensors are OFF.",
            sensor_to_violations,
        )
    )

    accepted_to_violations: list[dict[str, Any]] = []
    for event in accepted_events:
        if event.name != "toe_off":
            continue
        sample = by_index[event.row_index]
        if sample.heel_contact or sample.toe_contact:
            accepted_to_violations.append(
                _violation(
                    event,
                    "accepted TO occurred while a foot sensor remained active",
                    event_source=sample.event_source,
                    heel_contact=sample.heel_contact,
                    toe_contact=sample.toe_contact,
                    heel_load_n=sample.heel_load_n,
                    toe_load_n=sample.toe_load_n,
                )
            )
    checks.append(
        _check_result(
            "accepted_to_requires_both_contacts_off",
            "An FSM-accepted TO requires both debounced sensor contacts OFF.",
            accepted_to_violations,
        )
    )

    checks.extend(
        (
            _check_result(
                "sensor_hs_to_order_is_valid",
                "Sensor HS/TO timestamps are monotonic and alternate; the first "
                "event may be TO after a partial-stance reset.",
                _semantic_event_sequence(sensor_events, stream="sensor"),
            ),
            _check_result(
                "accepted_hs_to_order_is_valid",
                "Accepted HS/TO timestamps are monotonic and alternate; the first "
                "event may be TO after a partial-stance reset.",
                _semantic_event_sequence(accepted_events, stream="accepted"),
            ),
        )
    )
    return checks


def _event_summary(events: Sequence[EventRecord]) -> dict[str, Any]:
    names = sorted(set(event.name for event in events))
    return {
        "total": len(events),
        "counts": {
            name: sum(event.name == name for event in events) for name in names
        },
        "times_s": {
            name: [
                event.event_time_s for event in events if event.name == name
            ]
            for name in names
        },
    }


def _event_state_summary(
    events: Sequence[EventRecord],
    samples: Sequence[TraceSample],
) -> list[dict[str, Any]]:
    """Attach physical-onset and debounce-confirmation state to events."""
    by_index = {sample.index: sample for sample in samples}
    rows: list[dict[str, Any]] = []
    for event in events:
        if event.name not in {"heel_strike", "toe_off"}:
            continue
        nearest = min(
            samples,
            key=lambda sample: abs(sample.time_s - event.event_time_s),
        )
        emitted = by_index[event.row_index]
        rows.append(
            {
                "event": event.name,
                "event_time_s": event.event_time_s,
                "confirmed_time_s": _finite_float(
                    event.payload.get("confirmed_time")
                ),
                "emission_row_time_s": event.row_time_s,
                "onset_nearest_sample_time_s": nearest.time_s,
                "onset_sample_alignment_error_s": (
                    nearest.time_s - event.event_time_s
                ),
                "onset_knee_served_deg": math.degrees(nearest.knee_served_rad),
                "onset_ankle_served_deg": math.degrees(nearest.ankle_served_rad),
                "onset_knee_actual_deg": (
                    math.degrees(nearest.knee_actual_rad)
                    if math.isfinite(nearest.knee_actual_rad)
                    else None
                ),
                "onset_ankle_actual_deg": (
                    math.degrees(nearest.ankle_actual_rad)
                    if math.isfinite(nearest.ankle_actual_rad)
                    else None
                ),
                "onset_heel_load_n": nearest.heel_load_n,
                "onset_toe_load_n": nearest.toe_load_n,
                "confirmation_heel_load_n": emitted.heel_load_n,
                "confirmation_toe_load_n": emitted.toe_load_n,
                "confirmation_heel_contact": emitted.heel_contact,
                "confirmation_toe_contact": emitted.toe_contact,
            }
        )
    return rows


def _degrees(values: Sequence[float]) -> list[float]:
    return [math.degrees(value) if math.isfinite(value) else math.nan for value in values]


def _plot(
    samples: Sequence[TraceSample],
    output_path: Path,
    *,
    on_threshold_n: float,
    off_threshold_n: float,
    overall_pass: bool,
    audit_authority: str,
) -> None:
    times = [sample.time_s for sample in samples]
    figure, axes = plt.subplots(
        4,
        1,
        figsize=(15.0, 12.0),
        sharex=True,
        gridspec_kw={"height_ratios": (2.0, 1.7, 1.0, 1.8)},
        constrained_layout=True,
    )

    ax = axes[0]
    ax.plot(
        times,
        _degrees([sample.knee_served_rad for sample in samples]),
        color="#6a3d9a",
        linewidth=2.0,
        label="Knee served reference",
    )
    ax.plot(
        times,
        _degrees([sample.ankle_served_rad for sample in samples]),
        color="#ff7f00",
        linewidth=2.0,
        label="Ankle served reference",
    )
    if any(math.isfinite(sample.knee_actual_rad) for sample in samples):
        ax.plot(
            times,
            _degrees([sample.knee_actual_rad for sample in samples]),
            color="#6a3d9a",
            linestyle="--",
            alpha=0.45,
            label="Knee actual",
        )
    if any(math.isfinite(sample.ankle_actual_rad) for sample in samples):
        ax.plot(
            times,
            _degrees([sample.ankle_actual_rad for sample in samples]),
            color="#ff7f00",
            linestyle="--",
            alpha=0.45,
            label="Ankle actual",
        )
    for sample in samples:
        for event in sample.accepted_events:
            if event.name == "heel_strike":
                ax.axvline(event.event_time_s, color="#d62728", alpha=0.28, linewidth=1.0)
            elif event.name == "toe_off":
                ax.axvline(event.event_time_s, color="#1f77b4", alpha=0.28, linewidth=1.0)
    ax.set_ylabel("Angle [deg]")
    ax.set_title("Served prosthetic trajectories (accepted HS red, TO blue)")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best", ncol=2)

    ax = axes[1]
    ax.plot(
        times,
        [sample.heel_load_n for sample in samples],
        color="#d62728",
        linewidth=1.6,
        label="Heel sensor",
    )
    ax.plot(
        times,
        [sample.toe_load_n for sample in samples],
        color="#1f77b4",
        linewidth=1.6,
        label="Toe sensor",
    )
    ax.axhline(
        on_threshold_n,
        color="#2ca02c",
        linestyle="--",
        linewidth=1.4,
        label=f"ON threshold ({on_threshold_n:g} N)",
    )
    ax.axhline(
        off_threshold_n,
        color="#555555",
        linestyle=":",
        linewidth=1.4,
        label=f"OFF threshold ({off_threshold_n:g} N)",
    )
    ax.set_ylabel("Normal load [N]")
    ax.set_title("Virtual regional detector loads (detector-only, not GRF generation)")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best", ncol=2)

    ax = axes[2]
    ax.step(
        times,
        [float(sample.heel_contact) for sample in samples],
        where="post",
        color="#d62728",
        linewidth=1.8,
        label="Heel stable contact",
    )
    ax.step(
        times,
        [float(sample.toe_contact) for sample in samples],
        where="post",
        color="#1f77b4",
        linewidth=1.8,
        label="Toe stable contact",
    )
    ax.set_ylim(-0.15, 1.15)
    ax.set_yticks((0, 1), labels=("OFF", "ON"))
    ax.set_ylabel("Latch")
    ax.set_title("Debounced contacts used by the existing gait-phase FSM")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best", ncol=2)

    ax = axes[3]
    event_lanes = (
        ("legacy", "heel_strike", 5.0, "#d62728", "o", "Legacy HS"),
        ("legacy", "toe_off", 4.0, "#1f77b4", "o", "Legacy TO"),
        ("sensor", "heel_strike", 3.0, "#d62728", "^", "Sensor HS"),
        ("sensor", "toe_off", 2.0, "#1f77b4", "v", "Sensor TO"),
        ("accepted", "heel_strike", 1.0, "#8c1d18", "*", "Accepted HS"),
        ("accepted", "toe_off", 0.0, "#154360", "*", "Accepted TO"),
    )
    all_events = [
        event
        for sample in samples
        for event in (
            *sample.legacy_events,
            *sample.sensor_events,
            *sample.accepted_events,
        )
    ]
    for stream, name, lane, colour, marker, label in event_lanes:
        values = [
            event.event_time_s
            for event in all_events
            if event.stream == stream and event.name == name
        ]
        if values:
            ax.scatter(
                values,
                [lane] * len(values),
                color=colour,
                marker=marker,
                s=70 if stream == "accepted" else 48,
                label=label,
                zorder=3,
            )
    diagnostics = [
        event.event_time_s
        for event in all_events
        if event.stream == "sensor"
        and event.name not in {"heel_strike", "toe_off"}
    ]
    if diagnostics:
        ax.scatter(
            diagnostics,
            [-1.0] * len(diagnostics),
            color="#7f7f7f",
            marker="x",
            s=45,
            label="Sensor diagnostic",
        )
    ax.set_yticks(
        (-1, 0, 1, 2, 3, 4, 5),
        labels=(
            "Sensor other",
            "Accepted TO",
            "Accepted HS",
            "Sensor TO",
            "Sensor HS",
            "Legacy TO",
            "Legacy HS",
        ),
    )
    ax.set_ylim(-1.7, 5.7)
    ax.set_ylabel("Event stream")
    ax.set_xlabel("Simulation time [s]")
    ax.set_title("Legacy, two-sensor candidate, and FSM-accepted event timing")
    ax.grid(True, axis="x", alpha=0.25)
    if all_events:
        ax.legend(loc="upper center", ncol=4, fontsize=8)

    sources = sorted(set(sample.event_source for sample in samples))
    status = "PASS" if overall_pass else "FAIL"
    figure.suptitle(
        f"Heel/toe detector rollout audit — {status} — mode(s): "
        f"{', '.join(sources) or 'unknown'}\n"
        f"event authority: {audit_authority}",
        fontsize=14,
        fontweight="bold",
    )
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def _load_rollout_summary(trace_path: Path) -> Mapping[str, Any]:
    summary_path = trace_path.parent / "rollout_summary.json"
    if not summary_path.is_file():
        return {}
    try:
        raw = json.loads(summary_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return _mapping(raw) or {}


def _load_thresholds_from_summary(
    summary: Mapping[str, Any],
) -> dict[str, float]:
    result: dict[str, float] = {}
    for target, source in (
        ("on", "phase_sensor_on_threshold_n"),
        ("off", "phase_sensor_off_threshold_n"),
        ("dwell", "phase_sensor_dwell_s"),
    ):
        value = _finite_float(summary.get(source))
        if value is not None:
            result[target] = value
    return result


def _fsm_overrides_from_summary(
    summary: Mapping[str, Any],
) -> dict[str, float]:
    """Mirror the phase-FSM mapping in ``baseline_MLP/env_factory.py``.

    Detector thresholds are passed separately because they live at the rollout
    top level.  Returning an empty mapping is deliberate when no summary is
    available: the caller then records that production defaults were used.
    """
    reward = _mapping(summary.get("reward_config"))
    if reward is None:
        return {}
    field_map = {
        "min_stance_duration_s": "phase_min_stance_duration_s",
        "min_swing_duration_s": "phase_min_swing_duration_s",
        "landing_window_start_s": "phase_landing_window_start_s",
        "landing_window_end_s": "phase_landing_window_end_s",
        "stance_hard_timeout_s": "phase_stance_hard_timeout_s",
        "swing_hard_timeout_s": "phase_swing_hard_timeout_s",
        "landing_force_full_credit_bw": "contact_load_target_bw",
        "min_stance_contact_fraction": "phase_min_stance_contact_fraction",
        "min_stance_load_bw_s": "phase_min_stance_load_bw_s",
        "min_cycle_knee_excursion_rad": "phase_min_cycle_knee_excursion_rad",
        "hs_event_credit": "phase_hs_event_credit",
        "toe_off_event_credit": "phase_to_event_credit",
        "cycle_complete_bonus": "phase_cycle_complete_bonus",
        "failure_extra_penalty": "phase_failure_extra_penalty",
    }
    result: dict[str, float] = {}
    for target, source in field_map.items():
        value = _finite_float(reward.get(source))
        if value is not None:
            result[target] = value
    return result


def _synthetic_trace() -> list[dict[str, Any]]:
    def row(
        index: int,
        time_s: float,
        heel_load: float,
        toe_load: float,
        heel_contact: bool,
        toe_contact: bool,
        *,
        sensor_events: Sequence[Mapping[str, Any]] = (),
        accepted_events: Sequence[Mapping[str, Any]] = (),
        legacy_events: Sequence[Mapping[str, Any]] = (),
    ) -> dict[str, Any]:
        phase = 2.0 * math.pi * time_s
        return {
            "step": index,
            "time": time_s,
            "prosthetic_state": {
                "pros_knee_angle_served_ref": -0.35 + 0.25 * math.cos(phase),
                "pros_ankle_angle_served_ref": 0.08 * math.sin(phase),
                "pros_knee_angle": -0.34 + 0.23 * math.cos(phase),
                "pros_ankle_angle": 0.07 * math.sin(phase),
            },
            "detector_sensors": {
                "left_heel": {"normal_load_n": heel_load},
                "left_toe": {"normal_load_n": toe_load},
            },
            "legacy_online_events": list(legacy_events),
            "phase_fsm": {
                "event_source": "two_sensor",
                "sensor_heel_contact": float(heel_contact),
                "sensor_toe_contact": float(toe_contact),
                "sensor_events_this_step": list(sensor_events),
                "accepted_transitions_this_step": list(accepted_events),
            },
        }

    hs1 = {
        "side": "left",
        "event": "heel_strike",
        "time": 0.10,
        "confirmed_time": 0.13,
        "source": "two_sensor",
    }
    to1 = {
        "side": "left",
        "event": "toe_off",
        "time": 0.70,
        "confirmed_time": 0.73,
        "source": "two_sensor",
    }
    hs2 = {
        "side": "left",
        "event": "heel_strike",
        "time": 1.20,
        "confirmed_time": 1.23,
        "source": "two_sensor",
    }
    return [
        row(0, 0.00, 0.0, 0.0, False, False),
        row(
            1,
            0.13,
            10.0,
            0.0,
            True,
            False,
            sensor_events=(hs1,),
            accepted_events=(
                {"event": "heel_strike", "event_time_s": 0.10},
            ),
            legacy_events=(
                {"side": "left", "event": "heel_strike", "time": 0.10},
            ),
        ),
        row(2, 0.40, 9.0, 12.0, True, True),
        row(
            3,
            0.73,
            0.0,
            0.0,
            False,
            False,
            sensor_events=(to1,),
            accepted_events=(
                {"event": "toe_off", "event_time_s": 0.70},
            ),
            legacy_events=(
                {"side": "left", "event": "toe_off", "time": 0.70},
            ),
        ),
        row(4, 0.95, 0.0, 0.0, False, False),
        row(
            5,
            1.23,
            11.0,
            0.0,
            True,
            False,
            sensor_events=(hs2,),
            accepted_events=(
                {"event": "heel_strike", "event_time_s": 1.20},
            ),
            legacy_events=(
                {"side": "left", "event": "heel_strike", "time": 1.20},
            ),
        ),
    ]


def _synthetic_shadow_toe_first_trace() -> list[dict[str, Any]]:
    """Shadow fixture whose legacy FSM accepts a deliberately false early HS."""

    def row(
        index: int,
        time_s: float,
        heel_load: float,
        toe_load: float,
        *,
        legacy_events: Sequence[Mapping[str, Any]] = (),
        recorded_sensor_events: Sequence[Mapping[str, Any]] = (),
        recorded_accepted_events: Sequence[Mapping[str, Any]] = (),
    ) -> dict[str, Any]:
        phase = 2.0 * math.pi * time_s
        return {
            "step": index,
            "time": time_s,
            "prosthetic_state": {
                "pros_knee_angle_served_ref": -0.30 + 0.22 * math.cos(phase),
                "pros_ankle_angle_served_ref": 0.07 * math.sin(phase),
                "pros_knee_angle": -0.29 + 0.20 * math.cos(phase),
                "pros_ankle_angle": 0.06 * math.sin(phase),
            },
            "detector_sensors": {
                "left_heel": {"normal_load_n": heel_load},
                "left_toe": {"normal_load_n": toe_load},
            },
            "legacy_online_events": list(legacy_events),
            "phase_fsm": {
                "event_source": "shadow",
                # These recorded shadow latches/events are not the audit
                # authority.  The validator must replace them with replay.
                "sensor_heel_contact": 0.0,
                "sensor_toe_contact": float(toe_load >= 5.0),
                "sensor_events_this_step": list(recorded_sensor_events),
                "accepted_transitions_this_step": list(
                    recorded_accepted_events
                ),
            },
        }

    false_legacy_hs = {
        "side": "left",
        "event": "heel_strike",
        "time": 0.10,
    }
    return [
        row(0, 0.00, 0.0, 0.0),
        row(1, 0.03, 0.0, 0.0),
        row(2, 0.10, 0.0, 10.0),
        row(
            3,
            0.13,
            0.0,
            10.0,
            legacy_events=(false_legacy_hs,),
            recorded_sensor_events=(
                {
                    "side": "left",
                    "event": "forefoot_first",
                    "time": 0.10,
                    "confirmed_time": 0.13,
                },
            ),
            recorded_accepted_events=(
                {"event": "heel_strike", "event_time_s": 0.10},
            ),
        ),
        row(4, 0.20, 10.0, 10.0),
        row(5, 0.23, 10.0, 10.0),
        row(6, 0.40, 8.0, 11.0),
        row(7, 0.70, 0.0, 0.0),
        row(8, 0.73, 0.0, 0.0),
    ]


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--trace",
        type=Path,
        help="Path to rollout_policy_trace.json (not needed with --self-test).",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        required=True,
        help="Directory receiving the JSON metrics and PNG audit plot.",
    )
    parser.add_argument(
        "--on-threshold-n",
        type=float,
        help="Override the detector ON threshold; otherwise read rollout_summary.json.",
    )
    parser.add_argument(
        "--off-threshold-n",
        type=float,
        help="Override the detector OFF threshold; otherwise read rollout_summary.json.",
    )
    parser.add_argument(
        "--dwell-s",
        type=float,
        help="Override the detector dwell used by shadow replay.",
    )
    parser.add_argument(
        "--alignment-tolerance-s",
        type=float,
        default=0.06,
        help="Maximum accepted-HS to heel-onset alignment error (default: 0.06 s).",
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help=(
            "Replay a toe-first shadow fixture through the production FSM and "
            "run the full JSON/PNG pipeline."
        ),
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    if args.self_test and args.trace is not None:
        raise SystemExit("--self-test and --trace are mutually exclusive")
    if not args.self_test and args.trace is None:
        raise SystemExit("--trace is required unless --self-test is used")
    if args.alignment_tolerance_s < 0.0:
        raise SystemExit("--alignment-tolerance-s must be non-negative")

    source = "synthetic:self-test"
    summary_thresholds: dict[str, float] = {}
    fsm_config_overrides: dict[str, float] = {}
    if args.self_test:
        rows: Sequence[Any] = _synthetic_shadow_toe_first_trace()
    else:
        trace_path = args.trace.expanduser().resolve()
        if not trace_path.is_file():
            raise SystemExit(f"Trace not found: {trace_path}")
        try:
            raw = json.loads(trace_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            raise SystemExit(f"Invalid JSON in {trace_path}: {exc}") from exc
        rows_value = _sequence(raw)
        if rows_value is None:
            raise SystemExit("rollout_policy_trace.json must contain a JSON list")
        rows = rows_value
        source = str(trace_path)
        rollout_summary = _load_rollout_summary(trace_path)
        summary_thresholds = _load_thresholds_from_summary(rollout_summary)
        fsm_config_overrides = _fsm_overrides_from_summary(rollout_summary)

    on_threshold_n = (
        args.on_threshold_n
        if args.on_threshold_n is not None
        else summary_thresholds.get("on", DEFAULT_ON_THRESHOLD_N)
    )
    off_threshold_n = (
        args.off_threshold_n
        if args.off_threshold_n is not None
        else summary_thresholds.get("off", DEFAULT_OFF_THRESHOLD_N)
    )
    dwell_s = (
        args.dwell_s
        if args.dwell_s is not None
        else summary_thresholds.get("dwell", DEFAULT_DWELL_S)
    )
    if not (math.isfinite(on_threshold_n) and on_threshold_n > 0.0):
        raise SystemExit("ON threshold must be finite and positive")
    if not (
        math.isfinite(off_threshold_n)
        and 0.0 <= off_threshold_n < on_threshold_n
    ):
        raise SystemExit("OFF threshold must be finite, non-negative, and below ON")
    if not (math.isfinite(dwell_s) and dwell_s >= 0.0):
        raise SystemExit("dwell must be finite and non-negative")

    samples, schema_violations = _parse_trace(rows)
    if not samples:
        raise SystemExit("Trace contains no usable rows")
    event_sources = set(sample.event_source for sample in samples)
    if event_sources == {"shadow"}:
        samples, replay_provenance = _offline_two_sensor_replay(
            samples,
            on_threshold_n=on_threshold_n,
            off_threshold_n=off_threshold_n,
            dwell_s=dwell_s,
            fsm_config_overrides=fsm_config_overrides,
        )
    else:
        if "shadow" in event_sources:
            schema_violations.append(
                {
                    "reason": (
                        "mixed shadow/non-shadow trace cannot have one unambiguous "
                        "event authority"
                    ),
                    "event_sources": sorted(event_sources),
                }
            )
        replay_provenance = {
            "applied": False,
            "reason": (
                "offline replay is required only for a pure shadow trace; "
                "recorded FSM events remain authoritative"
            ),
            "authority_for_hard_checks_and_plot": "recorded_fsm_payload",
        }

    if args.self_test:
        replay_sensor_events = [
            event for sample in samples for event in sample.sensor_events
        ]
        replay_accepted_events = [
            event for sample in samples for event in sample.accepted_events
        ]
        hs_candidates = [
            event
            for event in replay_sensor_events
            if event.name == "heel_strike"
        ]
        forefoot_events = [
            event
            for event in replay_sensor_events
            if event.name == "forefoot_first"
        ]
        accepted_hs = [
            event
            for event in replay_accepted_events
            if event.name == "heel_strike"
        ]
        if not (
            len(forefoot_events) == 1
            and abs(forefoot_events[0].event_time_s - 0.10) <= 1e-12
            and len(hs_candidates) == 1
            and abs(hs_candidates[0].event_time_s - 0.20) <= 1e-12
            and len(accepted_hs) == 1
            and abs(accepted_hs[0].event_time_s - 0.20) <= 1e-12
        ):
            raise AssertionError(
                "toe-first replay contract failed: only the later debounced "
                "heel onset may create/accept HS"
            )

    checks = _run_checks(
        samples,
        schema_violations,
        on_threshold_n=on_threshold_n,
        alignment_tolerance_s=float(args.alignment_tolerance_s),
    )
    overall_pass = all(bool(item["pass"]) for item in checks)

    streams = {
        "legacy": [event for sample in samples for event in sample.legacy_events],
        "sensor": [event for sample in samples for event in sample.sensor_events],
        "accepted": [
            event for sample in samples for event in sample.accepted_events
        ],
    }
    recorded_streams = {
        "sensor": [
            event
            for sample in samples
            for event in sample.recorded_sensor_events
        ],
        "accepted": [
            event
            for sample in samples
            for event in sample.recorded_accepted_events
        ],
    }
    metrics = {
        "schema_version": SCHEMA_VERSION,
        "source_trace": source,
        "overall_pass": overall_pass,
        "sample_count": len(samples),
        "trace_row_count": len(rows),
        "time_start_s": min(sample.time_s for sample in samples),
        "time_end_s": max(sample.time_s for sample in samples),
        "event_sources": sorted(event_sources),
        "event_authority": replay_provenance[
            "authority_for_hard_checks_and_plot"
        ],
        "offline_replay": replay_provenance,
        "detector_config": {
            "on_threshold_n": on_threshold_n,
            "off_threshold_n": off_threshold_n,
            "dwell_s": dwell_s,
            "alignment_tolerance_s": float(args.alignment_tolerance_s),
        },
        "event_streams": {
            name: _event_summary(events) for name, events in streams.items()
        },
        "event_state_at_times": {
            name: _event_state_summary(events, samples)
            for name, events in streams.items()
        },
        "recorded_fsm_event_streams_before_replay": {
            name: _event_summary(events)
            for name, events in recorded_streams.items()
        },
        "hard_checks": checks,
        "hard_check_failures": [
            item["name"] for item in checks if not bool(item["pass"])
        ],
    }

    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    metrics_path = output_dir / "two_sensor_detector_metrics.json"
    plot_path = output_dir / "01_two_sensor_detector_trace.png"
    metrics_path.write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    _plot(
        samples,
        plot_path,
        on_threshold_n=on_threshold_n,
        off_threshold_n=off_threshold_n,
        overall_pass=overall_pass,
        audit_authority=str(
            replay_provenance["authority_for_hard_checks_and_plot"]
        ),
    )

    print(f"Detector trace audit: {'PASS' if overall_pass else 'FAIL'}")
    print(f"Samples: {len(samples)}")
    print(f"Metrics: {metrics_path}")
    print(f"Plot: {plot_path}")
    for item in checks:
        print(
            f"  [{'PASS' if item['pass'] else 'FAIL'}] {item['name']} "
            f"(violations={item['violation_count']})"
        )
    return 0 if overall_pass else 2


if __name__ == "__main__":
    raise SystemExit(main())
