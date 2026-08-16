"""Pure V10 helper for a coherent legacy-H0 teacher view.

The deployable observation remains the V26 actor view.  This module builds a
label-only counterfactual for the frozen H0 actor by replacing exactly actor
columns 10 through 24 (inclusive):

* analog detector load/contact;
* boundary-visible legacy left HS/TO pulses;
* the legacy left gait clock, anchored to physical event onsets; and
* an independent legacy-event ``ProstheticPhaseFSM`` observation.

No value outside that block may change.  ``LegacyGaitShadow`` validates and
advances candidate state transactionally, so malformed or non-monotonic input
cannot partially mutate the shadow used by the next label.
"""

from __future__ import annotations

import copy
import dataclasses
import math
from typing import Any, Mapping, Sequence

import numpy as np


ACTOR_FEATURE_COUNT = 35
LOAD_INDEX = 10
CONTACT_INDEX = 11
HEEL_STRIKE_INDEX = 12
TOE_OFF_INDEX = 13
GAIT_PHASE_SIN_INDEX = 14
GAIT_PHASE_COS_INDEX = 15
CYCLE_DURATION_INDEX = 16
PHASE_BLOCK_START = 17
PHASE_BLOCK_STOP = 25
TEACHER_BLOCK_START = LOAD_INDEX
TEACHER_BLOCK_STOP = PHASE_BLOCK_STOP
TEACHER_BLOCK_INDICES = tuple(range(TEACHER_BLOCK_START, TEACHER_BLOCK_STOP))
UNCHANGED_INDICES = tuple(
    index for index in range(ACTOR_FEATURE_COUNT) if index not in TEACHER_BLOCK_INDICES
)

EXPECTED_ACTOR_FEATURE_NAMES = (
    "gait_phase_sin",
    "gait_phase_cos",
    "pros_knee_angle",
    "pros_knee_angle_vel",
    "pros_ankle_angle",
    "pros_ankle_angle_vel",
    "SEA_Knee_motor_angle",
    "SEA_Knee_motor_speed",
    "SEA_Ankle_motor_angle",
    "SEA_Ankle_motor_speed",
    "online_left_normal_grf_bw",
    "online_left_in_contact",
    "online_left_heel_strike",
    "online_left_toe_off",
    "online_left_gait_phase_sin",
    "online_left_gait_phase_cos",
    "online_left_cycle_duration_s",
    "phase_fsm_wait_hs",
    "phase_fsm_stance_after_hs",
    "phase_fsm_swing_after_to",
    "phase_expected_hs",
    "phase_expected_to",
    "phase_stance_elapsed_norm",
    "phase_swing_elapsed_norm",
    "phase_cycle_progress_credit",
    "pros_knee_angle_previous_endpoint",
    "pros_knee_angle_served_ref",
    "pros_knee_angle_served_ref_vel",
    "pros_knee_angle_served_ref_accel",
    "pros_knee_angle_sea_u",
    "pros_ankle_angle_previous_endpoint",
    "pros_ankle_angle_served_ref",
    "pros_ankle_angle_served_ref_vel",
    "pros_ankle_angle_served_ref_accel",
    "pros_ankle_angle_sea_u",
)

PHASE_FEATURE_NAMES = EXPECTED_ACTOR_FEATURE_NAMES[
    PHASE_BLOCK_START:PHASE_BLOCK_STOP
]
_TIME_TOLERANCE_S = 1.0e-9
_RESET_PHASE_OBSERVATION = {
    "phase_fsm_wait_hs": 1.0,
    "phase_fsm_stance_after_hs": 0.0,
    "phase_fsm_swing_after_to": 0.0,
    "phase_expected_hs": 1.0,
    "phase_expected_to": 0.0,
    "phase_stance_elapsed_norm": 0.0,
    "phase_swing_elapsed_norm": 0.0,
    "phase_cycle_progress_credit": 0.0,
}


class V10CoherentTeacherError(RuntimeError):
    """Raised when a V10 label cannot be built without fallback."""


def _finite_number(value: Any, *, label: str) -> float:
    if isinstance(value, (bool, np.bool_)):
        raise V10CoherentTeacherError(f"{label} must be numeric")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise V10CoherentTeacherError(f"{label} must be numeric") from exc
    if not math.isfinite(result):
        raise V10CoherentTeacherError(f"{label} must be finite")
    return result


def _nonnegative_number(value: Any, *, label: str) -> float:
    result = _finite_number(value, label=label)
    if result < 0.0:
        raise V10CoherentTeacherError(f"{label} must be nonnegative")
    return result


def _actor(value: Any, *, label: str) -> np.ndarray:
    raw = np.asarray(value)
    if raw.shape != (ACTOR_FEATURE_COUNT,):
        raise V10CoherentTeacherError(
            f"{label} must have shape ({ACTOR_FEATURE_COUNT},)"
        )
    if raw.dtype != np.dtype("float32"):
        raise V10CoherentTeacherError(f"{label} must have dtype float32")
    actor = np.ascontiguousarray(raw.copy())
    if not np.all(np.isfinite(actor)):
        raise V10CoherentTeacherError(f"{label} must be finite")
    return actor


def validate_actor_feature_names(names: Sequence[str]) -> None:
    if isinstance(names, (str, bytes, bytearray)):
        raise V10CoherentTeacherError("actor feature names must be a sequence")
    ordered = tuple(str(name) for name in names)
    if ordered != EXPECTED_ACTOR_FEATURE_NAMES:
        raise V10CoherentTeacherError(
            "actor feature names do not match the frozen 35-field layout"
        )


def _phase_observation(phase_fsm: Any) -> dict[str, float]:
    try:
        raw = phase_fsm.observation()
    except Exception as exc:  # pragma: no cover - defensive adapter boundary
        raise V10CoherentTeacherError(
            "legacy phase FSM observation failed"
        ) from exc
    if not isinstance(raw, Mapping) or set(raw) != set(PHASE_FEATURE_NAMES):
        raise V10CoherentTeacherError("legacy phase FSM observation schema drifted")
    result = {
        name: _finite_number(raw[name], label=f"legacy phase FSM {name}")
        for name in PHASE_FEATURE_NAMES
    }
    return result


def _observation_joint_angles(info: Mapping[str, Any]) -> tuple[float, float]:
    observation = info.get("observation")
    if not isinstance(observation, Mapping):
        raise V10CoherentTeacherError("info.observation must be an object")
    knee = _finite_number(
        observation.get("pros_knee_angle"),
        label="info.observation.pros_knee_angle",
    )
    ankle = _finite_number(
        observation.get("pros_ankle_angle"),
        label="info.observation.pros_ankle_angle",
    )
    return knee, ankle


def _event_timestamp(
    event: Mapping[str, Any],
    *,
    primary: str,
    fallback: str | None,
    label: str,
) -> float | None:
    has_primary = primary in event and event.get(primary) is not None
    has_fallback = (
        fallback is not None and fallback in event and event.get(fallback) is not None
    )
    if not has_primary and not has_fallback:
        return None
    primary_value = (
        _finite_number(event[primary], label=f"{label}.{primary}")
        if has_primary
        else None
    )
    fallback_value = (
        _finite_number(event[fallback], label=f"{label}.{fallback}")
        if has_fallback and fallback is not None
        else None
    )
    if (
        primary_value is not None
        and fallback_value is not None
        and abs(primary_value - fallback_value) > _TIME_TOLERANCE_S
    ):
        raise V10CoherentTeacherError(
            f"{label}.{primary}/{fallback} timestamps disagree"
        )
    return primary_value if primary_value is not None else fallback_value


class LegacyGaitShadow:
    """Transactional legacy gait-clock and ``ProstheticPhaseFSM`` shadow."""

    def __init__(self, phase_fsm: Any) -> None:
        config = getattr(phase_fsm, "config", None)
        if getattr(config, "event_source", None) != "legacy_events":
            raise V10CoherentTeacherError(
                "phase_fsm must use event_source='legacy_events'"
            )
        if not all(callable(getattr(phase_fsm, name, None)) for name in (
            "reset",
            "update",
            "observation",
            "payload",
        )):
            raise V10CoherentTeacherError(
                "phase_fsm does not implement the required legacy interface"
            )
        try:
            self._phase_fsm = copy.deepcopy(phase_fsm)
            self._phase_fsm.reset()
        except Exception as exc:
            raise V10CoherentTeacherError(
                "legacy phase FSM could not be copied and reset"
            ) from exc
        if _phase_observation(self._phase_fsm) != _RESET_PHASE_OBSERVATION:
            raise V10CoherentTeacherError("legacy phase FSM reset contract drifted")
        self._clear_gait_state()

    @classmethod
    def from_runtime_phase_fsm(cls, runtime_phase_fsm: Any) -> "LegacyGaitShadow":
        """Clone runtime timing gates while restoring legacy event semantics."""

        config = getattr(runtime_phase_fsm, "config", None)
        if config is None or not dataclasses.is_dataclass(config):
            raise V10CoherentTeacherError("runtime phase FSM config is malformed")
        try:
            legacy_config = dataclasses.replace(config, event_source="legacy_events")
            legacy_fsm = type(runtime_phase_fsm)(legacy_config)
        except Exception as exc:
            raise V10CoherentTeacherError(
                "could not construct the legacy phase FSM shadow"
            ) from exc
        return cls(legacy_fsm)

    def _clear_gait_state(self) -> None:
        self._episode_start_time_s: float | None = None
        self._last_boundary_time_s: float | None = None
        self._last_left_event_time_s: float | None = None
        self._last_hs_time_s: float | None = None
        self._cycle_duration_s = 0.0

    def reset(self) -> None:
        """Clear all episode state without assigning a synthetic event."""

        try:
            candidate = copy.deepcopy(self._phase_fsm)
            candidate.reset()
        except Exception as exc:
            raise V10CoherentTeacherError("legacy phase FSM reset failed") from exc
        if _phase_observation(candidate) != _RESET_PHASE_OBSERVATION:
            raise V10CoherentTeacherError("legacy phase FSM reset contract drifted")
        self._phase_fsm = candidate
        self._clear_gait_state()

    def phase_observation(self) -> dict[str, float]:
        """Return a detached snapshot of the current legacy FSM observation."""

        return dict(_phase_observation(self._phase_fsm))

    def phase_payload(self) -> dict[str, Any]:
        """Return detached diagnostics without changing historical semantics."""

        try:
            raw = self._phase_fsm.payload()
        except Exception as exc:  # pragma: no cover - defensive adapter boundary
            raise V10CoherentTeacherError(
                "legacy phase FSM payload failed"
            ) from exc
        if not isinstance(raw, Mapping):
            raise V10CoherentTeacherError(
                "legacy phase FSM payload is malformed"
            )
        return copy.deepcopy(dict(raw))

    def _validate_info_common(
        self,
        info: Mapping[str, Any],
    ) -> tuple[float, float, float]:
        if not isinstance(info, Mapping):
            raise V10CoherentTeacherError("info must be an object")
        boundary = _finite_number(info.get("time"), label="info.time")
        knee, ankle = _observation_joint_angles(info)
        return boundary, knee, ankle

    @staticmethod
    def _detector_load_contact(
        info: Mapping[str, Any],
        *,
        body_weight_n: float,
        reset_boundary: bool,
    ) -> tuple[float, bool]:
        detector = info.get("online_grf_detector")
        if not isinstance(detector, Mapping):
            raise V10CoherentTeacherError(
                "info.online_grf_detector must be an object"
            )
        if reset_boundary:
            if detector:
                raise V10CoherentTeacherError(
                    "reset detector stream must be empty"
                )
            return 0.0, False
        left = detector.get("left")
        if not isinstance(left, Mapping):
            raise V10CoherentTeacherError(
                "info.online_grf_detector.left is missing"
            )
        normal_force_n = _nonnegative_number(
            left.get("normal_force"),
            label="info.online_grf_detector.left.normal_force",
        )
        in_contact = left.get("in_contact")
        if not isinstance(in_contact, (bool, np.bool_)):
            raise V10CoherentTeacherError(
                "info.online_grf_detector.left.in_contact must be boolean"
            )
        return normal_force_n / body_weight_n, bool(in_contact)

    def _legacy_left_events(
        self,
        info: Mapping[str, Any],
        *,
        boundary_time_s: float,
        reset_boundary: bool,
    ) -> list[dict[str, Any]]:
        raw_events = info.get("legacy_online_events")
        if not isinstance(raw_events, Sequence) or isinstance(
            raw_events, (str, bytes, bytearray)
        ):
            raise V10CoherentTeacherError(
                "info.legacy_online_events must be an ordered sequence"
            )
        if reset_boundary and raw_events:
            raise V10CoherentTeacherError("reset legacy event stream must be empty")

        left_events: list[dict[str, Any]] = []
        for index, raw in enumerate(raw_events):
            label = f"info.legacy_online_events[{index}]"
            if not isinstance(raw, Mapping):
                raise V10CoherentTeacherError(f"{label} must be an object")
            side = str(raw.get("side", "")).strip().lower()
            event_name = str(raw.get("event", "")).strip().lower()
            if side not in {"left", "right"}:
                raise V10CoherentTeacherError(f"{label}.side is unknown")
            if event_name not in {"heel_strike", "toe_off"}:
                raise V10CoherentTeacherError(f"{label}.event is unknown")

            onset = _event_timestamp(
                raw,
                primary="event_time_s",
                fallback="time",
                label=label,
            )
            if onset is None:
                raise V10CoherentTeacherError(f"{label} has no physical onset")
            confirmed = _event_timestamp(
                raw,
                primary="confirmed_time_s",
                fallback="confirmed_time",
                label=label,
            )
            if confirmed is None:
                confirmed = onset
            delivered = _event_timestamp(
                raw,
                primary="delivered_time_s",
                fallback=None,
                label=label,
            )
            if delivered is None:
                delivered = boundary_time_s
            elif abs(delivered - boundary_time_s) > _TIME_TOLERANCE_S:
                raise V10CoherentTeacherError(
                    f"{label}.delivered_time_s is not this policy boundary"
                )
            if not (
                onset <= confirmed + _TIME_TOLERANCE_S
                and confirmed <= delivered + _TIME_TOLERANCE_S
                and delivered <= boundary_time_s + _TIME_TOLERANCE_S
            ):
                raise V10CoherentTeacherError(
                    f"{label} timestamps are not causal"
                )
            episode_start = self._episode_start_time_s
            if (
                episode_start is not None
                and onset < episode_start - _TIME_TOLERANCE_S
            ):
                raise V10CoherentTeacherError(
                    f"{label} onset precedes the episode boundary"
                )

            canonical = dict(raw)
            canonical.update(
                {
                    "side": side,
                    "event": event_name,
                    "time": onset,
                    "event_time_s": onset,
                    "confirmed_time": confirmed,
                    "confirmed_time_s": confirmed,
                    "delivered_time_s": boundary_time_s,
                }
            )
            if "cycle_duration_s" in raw and raw.get("cycle_duration_s") is not None:
                canonical["cycle_duration_s"] = _nonnegative_number(
                    raw["cycle_duration_s"],
                    label=f"{label}.cycle_duration_s",
                )
            if "contact_duration_s" in raw and raw.get("contact_duration_s") is not None:
                canonical["contact_duration_s"] = _nonnegative_number(
                    raw["contact_duration_s"],
                    label=f"{label}.contact_duration_s",
                )
            if side == "left":
                left_events.append(canonical)

        previous = self._last_left_event_time_s
        for event in left_events:
            onset = float(event["event_time_s"])
            if previous is not None and onset <= previous + _TIME_TOLERANCE_S:
                raise V10CoherentTeacherError(
                    "legacy left event onsets must be strictly monotonic"
                )
            previous = onset
        return left_events

    def consume(
        self,
        info: Mapping[str, Any],
        *,
        body_weight_n: float,
        reset_boundary: bool = False,
    ) -> np.ndarray:
        """Consume one boundary and return the coherent float32 block 10:25."""

        weight = _finite_number(body_weight_n, label="body_weight_n")
        if weight <= 0.0:
            raise V10CoherentTeacherError("body_weight_n must be positive")
        boundary, knee, ankle = self._validate_info_common(info)
        load_bw, in_contact = self._detector_load_contact(
            info,
            body_weight_n=weight,
            reset_boundary=reset_boundary,
        )

        if reset_boundary:
            events = self._legacy_left_events(
                info,
                boundary_time_s=boundary,
                reset_boundary=True,
            )
            if events:
                raise V10CoherentTeacherError(
                    "reset must not attribute legacy left events"
                )
            try:
                candidate_fsm = copy.deepcopy(self._phase_fsm)
                candidate_fsm.reset()
            except Exception as exc:
                raise V10CoherentTeacherError(
                    "legacy phase FSM reset failed"
                ) from exc
            phase = _phase_observation(candidate_fsm)
            if phase != _RESET_PHASE_OBSERVATION:
                raise V10CoherentTeacherError(
                    "legacy phase FSM reset contract drifted"
                )
            episode_start = boundary
            last_event = None
            last_hs = None
            cycle_duration = 0.0
        else:
            if self._last_boundary_time_s is None:
                raise V10CoherentTeacherError(
                    "LegacyGaitShadow must consume a reset boundary first"
                )
            if boundary <= self._last_boundary_time_s + _TIME_TOLERANCE_S:
                raise V10CoherentTeacherError(
                    "policy boundaries must be strictly monotonic"
                )
            events = self._legacy_left_events(
                info,
                boundary_time_s=boundary,
                reset_boundary=False,
            )
            episode_start = self._episode_start_time_s
            last_event = self._last_left_event_time_s
            last_hs = self._last_hs_time_s
            cycle_duration = self._cycle_duration_s

            for event in events:
                onset = float(event["event_time_s"])
                last_event = onset
                if event["event"] != "heel_strike":
                    continue
                supplied_cycle = event.get("cycle_duration_s")
                if last_hs is None:
                    if supplied_cycle is not None:
                        raise V10CoherentTeacherError(
                            "the first legacy heel strike cannot claim a cycle"
                        )
                else:
                    if supplied_cycle is None:
                        raise V10CoherentTeacherError(
                            "a repeated legacy heel strike requires cycle_duration_s"
                        )
                    expected_cycle = onset - last_hs
                    tolerance = max(
                        _TIME_TOLERANCE_S,
                        abs(expected_cycle) * 1.0e-9,
                    )
                    if (
                        expected_cycle <= 0.0
                        or abs(float(supplied_cycle) - expected_cycle) > tolerance
                    ):
                        raise V10CoherentTeacherError(
                            "legacy cycle duration disagrees with HS onsets"
                        )
                    cycle_duration = float(supplied_cycle)
                last_hs = onset

            try:
                candidate_fsm = copy.deepcopy(self._phase_fsm)
                payload = candidate_fsm.update(
                    time_s=boundary,
                    events=events,
                    normal_force_bw=load_bw,
                    in_contact=in_contact,
                    prosthetic_knee_angle_rad=knee,
                    prosthetic_ankle_angle_rad=ankle,
                )
            except Exception as exc:
                raise V10CoherentTeacherError(
                    "legacy phase FSM update failed"
                ) from exc
            if not isinstance(payload, Mapping):
                raise V10CoherentTeacherError(
                    "legacy phase FSM payload is malformed"
                )
            # Validate the diagnostics, but preserve the historical FSM's own
            # handling of rejected/early events.  Stable H0 traces legitimately
            # contain such pulses: the gait clock exposes the pulse while the
            # FSM ignores the transition and records it diagnostically.  Turning
            # that established behaviour into a V10 exception would make the
            # teacher less faithful, not safer.
            _nonnegative_number(
                payload.get("invalid_event_this_step"),
                label="legacy phase FSM invalid_event_this_step",
            )
            _nonnegative_number(
                payload.get("timeout_exceeded"),
                label="legacy phase FSM timeout_exceeded",
            )
            phase = _phase_observation(candidate_fsm)

        heel_strike = any(event["event"] == "heel_strike" for event in events)
        toe_off = any(event["event"] == "toe_off" for event in events)
        if last_hs is None or cycle_duration <= 0.0:
            gait_phase = 0.0
        else:
            gait_phase = float(
                np.clip((boundary - last_hs) / cycle_duration, 0.0, 1.0)
            )
        angle = 2.0 * np.pi * gait_phase
        block = np.asarray(
            [
                load_bw,
                float(in_contact),
                float(heel_strike),
                float(toe_off),
                float(np.sin(angle)),
                float(np.cos(angle)),
                cycle_duration,
                *(phase[name] for name in PHASE_FEATURE_NAMES),
            ],
            dtype=np.float32,
        )
        if block.shape != (TEACHER_BLOCK_STOP - TEACHER_BLOCK_START,):
            raise V10CoherentTeacherError("coherent teacher block shape drifted")
        if not np.all(np.isfinite(block)):
            raise V10CoherentTeacherError("coherent teacher block is non-finite")

        # Commit only after the complete candidate and output passed validation.
        self._phase_fsm = candidate_fsm
        self._episode_start_time_s = episode_start
        self._last_boundary_time_s = boundary
        self._last_left_event_time_s = last_event
        self._last_hs_time_s = last_hs
        self._cycle_duration_s = cycle_duration
        return np.ascontiguousarray(block)


def assert_coherent_pair(student_v26: Any, teacher_h0: Any) -> None:
    """Fail unless only the inclusive actor block 10..24 can differ."""

    student = _actor(student_v26, label="student_v26")
    teacher = _actor(teacher_h0, label="teacher_h0")
    if (
        student[list(UNCHANGED_INDICES)].tobytes(order="C")
        != teacher[list(UNCHANGED_INDICES)].tobytes(order="C")
    ):
        raise V10CoherentTeacherError(
            "teacher changed a V26 field outside indices 10..24"
        )


def build_teacher_view(
    student_v26: Any,
    actor_feature_names: Sequence[str],
    info: Mapping[str, Any],
    *,
    body_weight_n: float,
    shadow: LegacyGaitShadow,
    reset_boundary: bool = False,
) -> np.ndarray:
    """Build one coherent H0 label view without mutating the V26 input."""

    validate_actor_feature_names(actor_feature_names)
    student = _actor(student_v26, label="student_v26")
    if not isinstance(shadow, LegacyGaitShadow):
        raise V10CoherentTeacherError("shadow must be a LegacyGaitShadow")
    block = shadow.consume(
        info,
        body_weight_n=body_weight_n,
        reset_boundary=reset_boundary,
    )
    teacher = student.copy()
    teacher[TEACHER_BLOCK_START:TEACHER_BLOCK_STOP] = block
    assert_coherent_pair(student, teacher)
    if (
        teacher[TEACHER_BLOCK_START:TEACHER_BLOCK_STOP].tobytes(order="C")
        != block.tobytes(order="C")
    ):
        raise V10CoherentTeacherError("coherent teacher block assignment drifted")
    return np.ascontiguousarray(teacher)


__all__ = [
    "ACTOR_FEATURE_COUNT",
    "CONTACT_INDEX",
    "CYCLE_DURATION_INDEX",
    "EXPECTED_ACTOR_FEATURE_NAMES",
    "GAIT_PHASE_COS_INDEX",
    "GAIT_PHASE_SIN_INDEX",
    "HEEL_STRIKE_INDEX",
    "LOAD_INDEX",
    "LegacyGaitShadow",
    "PHASE_BLOCK_START",
    "PHASE_BLOCK_STOP",
    "PHASE_FEATURE_NAMES",
    "TEACHER_BLOCK_INDICES",
    "TEACHER_BLOCK_START",
    "TEACHER_BLOCK_STOP",
    "TOE_OFF_INDEX",
    "UNCHANGED_INDICES",
    "V10CoherentTeacherError",
    "assert_coherent_pair",
    "build_teacher_view",
    "validate_actor_feature_names",
]
