"""Transactional V20-to-ProstheticPhaseFSM bridge for V25 active events.

The V25 detector and :class:`BinaryPhaseFSM` remain responsible only for the
force-free heel/toe bits and their debounced functional HS/TO events.  This
module transfers those accepted events to the existing actor-facing
``ProstheticPhaseFSM`` without changing its observation or reward contract.

Both FSMs are advanced on private copies.  Callers receive a complete candidate
result and commit the returned objects only after every validation succeeds;
there is therefore no partially-consumed detector batch on adapter failure.
"""

from __future__ import annotations

import copy
import json
import math
from dataclasses import dataclass
from typing import Any, Mapping, Sequence

from binary_phase_fsm import BinaryPhaseFSM
from prosthetic_phase_fsm import (
    BINARY_ACTIVE_ADAPTER_SOURCE,
    BINARY_ACTIVE_DEBOUNCE_S,
    BINARY_ACTIVE_EVENT_CONTRACT_ID,
    BINARY_ACTIVE_MAX_DELIVERY_DELAY_S,
    STATE_NAMES,
    STANCE_AFTER_HS,
    SWING_AFTER_TO,
    TIMEOUT,
    WAIT_HS,
    ProstheticPhaseFSM,
)


_TIME_TOLERANCE_S = 1e-9
V20_SOURCE = "binary_phase_fsm_v20"
_BINARY_STATE_IDS = {
    "AIR": 0,
    "TOE": 1,
    "HEEL": 2,
    "BOTH": 3,
}
_ACTIVE_PHASE_RUNTIME_STATE_IDS = {
    WAIT_HS,
    STANCE_AFTER_HS,
    SWING_AFTER_TO,
    TIMEOUT,
}


@dataclass(frozen=True)
class BinaryPhaseActiveResult:
    """A fully validated candidate commit for one reset or policy boundary."""

    binary_fsm: BinaryPhaseFSM
    phase_fsm: ProstheticPhaseFSM
    binary_payload: dict[str, Any]
    phase_payload: dict[str, Any]
    left_events: list[dict[str, Any]]
    adapter_payload: dict[str, Any]


class BinaryPhaseActiveAdapter:
    """Bridge V20 events into the existing actor-facing gait-cycle FSM."""

    def prime(
        self,
        *,
        binary_fsm: BinaryPhaseFSM,
        phase_fsm: ProstheticPhaseFSM,
        time_s: float,
        heel_contact: bool,
        toe_contact: bool,
    ) -> BinaryPhaseActiveResult:
        """Prime both FSMs from the same non-event V25 ``t0`` baseline."""

        self._validate_pair(binary_fsm, phase_fsm)
        baseline_time = self._finite_number(time_s, "baseline time_s")
        heel = self._native_bool(heel_contact, "heel_contact")
        toe = self._native_bool(toe_contact, "toe_contact")

        candidate_binary = copy.deepcopy(binary_fsm)
        candidate_phase = copy.deepcopy(phase_fsm)
        binary_payload = candidate_binary.reset(
            time_s=baseline_time,
            heel_contact=heel,
            toe_contact=toe,
        )
        phase_payload = candidate_phase.reset_from_binary_baseline(
            time_s=baseline_time,
            in_contact=bool(heel or toe),
        )
        self._validate_binary_runtime_state(binary_payload)
        self._validate_phase_runtime_state(phase_payload)
        phase_cursor = candidate_phase.binary_active_last_boundary_time_s
        if (
            phase_cursor is None
            or abs(float(phase_cursor) - baseline_time) > _TIME_TOLERANCE_S
        ):
            raise ValueError("Actor FSM baseline cursor mismatch.")
        self._validate_baseline_transfer(
            binary_payload,
            phase_payload,
            time_s=baseline_time,
            in_contact=bool(heel or toe),
        )
        adapter_payload = {
            "mode": "binary_active",
            "adapter_source": BINARY_ACTIVE_ADAPTER_SOURCE,
            "event_contract_id": BINARY_ACTIVE_EVENT_CONTRACT_ID,
            "baseline_time_s": baseline_time,
            "baseline_contact_state": binary_payload["stable_contact_state"],
            "baseline_in_contact": bool(heel or toe),
            "partial_stance_bootstrap": bool(heel or toe),
            "events_adapted_this_step": 0,
            "atomic_commit_ready": True,
        }
        self._validate_strict_json(
            {
                "binary_payload": binary_payload,
                "phase_payload": phase_payload,
                "left_events": [],
                "adapter_payload": adapter_payload,
            },
            "binary active reset candidate",
        )
        return BinaryPhaseActiveResult(
            binary_fsm=candidate_binary,
            phase_fsm=candidate_phase,
            binary_payload=copy.deepcopy(binary_payload),
            phase_payload=copy.deepcopy(phase_payload),
            left_events=[],
            adapter_payload=adapter_payload,
        )

    def advance(
        self,
        *,
        binary_fsm: BinaryPhaseFSM,
        phase_fsm: ProstheticPhaseFSM,
        time_s: float,
        previous_time_s: float,
        sensor_samples: Sequence[Mapping[str, object]],
        normal_force_bw: float,
        in_contact: bool,
        prosthetic_knee_angle_rad: float | None = None,
        prosthetic_ankle_angle_rad: float | None = None,
    ) -> BinaryPhaseActiveResult:
        """Advance V20 then the actor FSM and return one atomic candidate."""

        self._validate_pair(binary_fsm, phase_fsm)
        boundary = self._finite_number(time_s, "policy time_s")
        previous = self._finite_number(previous_time_s, "previous_time_s")
        if abs((boundary - previous) - 0.010) > _TIME_TOLERANCE_S:
            raise ValueError(
                "Binary active policy boundaries must span exactly 10 ms."
            )
        load_bw = self._finite_number(normal_force_bw, "normal_force_bw")
        if load_bw < 0.0:
            raise ValueError("normal_force_bw must be non-negative.")
        contact = self._native_bool(in_contact, "in_contact")
        knee = self._optional_finite(
            prosthetic_knee_angle_rad,
            "prosthetic_knee_angle_rad",
        )
        ankle = self._optional_finite(
            prosthetic_ankle_angle_rad,
            "prosthetic_ankle_angle_rad",
        )

        candidate_binary = copy.deepcopy(binary_fsm)
        candidate_phase = copy.deepcopy(phase_fsm)
        binary_payload = candidate_binary.update_policy_step(
            time_s=boundary,
            previous_time_s=previous,
            sensor_samples=sensor_samples,
        )
        self._validate_binary_runtime_state(binary_payload)
        adapted_events = self._adapt_events(
            binary_payload,
            phase_fsm=candidate_phase,
            boundary=boundary,
            previous=previous,
        )
        phase_payload = candidate_phase.update_from_binary_events(
            time_s=boundary,
            previous_time_s=previous,
            events=adapted_events,
            normal_force_bw=load_bw,
            in_contact=contact,
            prosthetic_knee_angle_rad=knee,
            prosthetic_ankle_angle_rad=ankle,
        )
        self._validate_transfer(adapted_events, phase_payload)
        left_events = self._accepted_left_events(
            phase_payload,
            adapted_events=adapted_events,
        )
        adapter_payload = {
            "mode": "binary_active",
            "adapter_source": BINARY_ACTIVE_ADAPTER_SOURCE,
            "event_contract_id": BINARY_ACTIVE_EVENT_CONTRACT_ID,
            "previous_time_s": previous,
            "delivered_time_s": boundary,
            "events_adapted_this_step": len(adapted_events),
            "event_order_this_step": [
                str(event["event"]) for event in adapted_events
            ],
            "atomic_commit_ready": True,
        }
        self._validate_strict_json(
            {
                "binary_payload": binary_payload,
                "phase_payload": phase_payload,
                "left_events": left_events,
                "adapter_payload": adapter_payload,
            },
            "binary active candidate",
        )
        return BinaryPhaseActiveResult(
            binary_fsm=candidate_binary,
            phase_fsm=candidate_phase,
            binary_payload=copy.deepcopy(binary_payload),
            phase_payload=copy.deepcopy(phase_payload),
            left_events=copy.deepcopy(left_events),
            adapter_payload=adapter_payload,
        )

    @staticmethod
    def _validate_pair(
        binary_fsm: BinaryPhaseFSM,
        phase_fsm: ProstheticPhaseFSM,
    ) -> None:
        if not isinstance(binary_fsm, BinaryPhaseFSM):
            raise TypeError("binary_fsm must be a BinaryPhaseFSM instance.")
        if not isinstance(phase_fsm, ProstheticPhaseFSM):
            raise TypeError("phase_fsm must be a ProstheticPhaseFSM instance.")
        binary_cfg = binary_fsm.config
        if str(binary_cfg.source) != V20_SOURCE:
            raise ValueError("Binary FSM source is not the frozen V20 source.")
        if (
            str(binary_cfg.event_contract_id)
            != BINARY_ACTIVE_EVENT_CONTRACT_ID
        ):
            raise ValueError("Binary FSM active event contract mismatch.")
        if abs(float(binary_cfg.sample_dt_s) - 0.001) > 1e-12:
            raise ValueError("Binary active sample_dt_s must be 0.001.")
        if abs(float(binary_cfg.debounce_s) - 0.005) > 1e-12:
            raise ValueError("Binary active debounce_s must be 0.005.")
        if abs(float(binary_cfg.max_delivery_delay_s) - 0.010) > 1e-12:
            raise ValueError(
                "Binary active max_delivery_delay_s must be 0.010."
            )
        if phase_fsm.config.event_source != "binary_active":
            raise ValueError(
                "Actor-facing FSM must use event_source='binary_active'."
            )
        BinaryPhaseActiveAdapter._validate_binary_runtime_state(
            binary_fsm.payload()
        )
        BinaryPhaseActiveAdapter._validate_phase_runtime_state(
            phase_fsm.payload()
        )
        BinaryPhaseActiveAdapter._validate_phase_configuration(phase_fsm)

    @classmethod
    def _validate_binary_runtime_state(
        cls,
        payload: Mapping[str, Any],
    ) -> None:
        if not isinstance(payload, Mapping):
            raise TypeError("V20 runtime payload must be a mapping.")
        for state_field, id_field in (
            ("raw_contact_state", "raw_contact_state_id"),
            ("stable_contact_state", "stable_contact_state_id"),
        ):
            state_name = str(payload.get(state_field, ""))
            expected_id = _BINARY_STATE_IDS.get(state_name)
            if expected_id is None:
                raise ValueError(f"V20 {state_field} is unknown.")
            observed_id = cls._finite_number(
                payload.get(id_field),
                f"V20 {id_field}",
            )
            if observed_id != float(expected_id):
                raise ValueError(
                    f"V20 {state_field}/{id_field} are inconsistent."
                )
        in_contact = payload.get("in_contact")
        if type(in_contact) is not bool:
            raise TypeError("V20 in_contact must be a native bool.")
        expected_contact = str(payload["stable_contact_state"]) != "AIR"
        if in_contact is not expected_contact:
            raise ValueError("V20 stable state and in_contact disagree.")
        cls._validate_strict_json(payload, "V20 payload")

    @classmethod
    def _validate_phase_runtime_state(
        cls,
        payload: Mapping[str, Any],
    ) -> None:
        if not isinstance(payload, Mapping):
            raise TypeError("Actor FSM runtime payload must be a mapping.")
        state_value = cls._finite_number(
            payload.get("state_id"),
            "actor FSM state_id",
        )
        state_id = int(round(state_value))
        if abs(state_value - state_id) > _TIME_TOLERANCE_S:
            raise ValueError("Actor FSM state_id must be integral.")
        if state_id not in _ACTIVE_PHASE_RUNTIME_STATE_IDS:
            raise ValueError(
                "Actor FSM state_id is not an allowed runtime state."
            )
        expected_name = STATE_NAMES.get(state_id)
        if expected_name is None:
            raise ValueError("Actor FSM state_id is unknown.")
        if payload.get("state_name") != expected_name:
            raise ValueError("Actor FSM state_id/state_name are inconsistent.")
        cls._validate_strict_json(payload, "actor FSM payload")

    @classmethod
    def _validate_phase_configuration(
        cls,
        phase_fsm: ProstheticPhaseFSM,
    ) -> None:
        for name, raw_value in vars(phase_fsm.config).items():
            if name == "event_source":
                continue
            if isinstance(raw_value, bool):
                raise TypeError(
                    f"Actor FSM config {name} must be a finite number."
                )
            try:
                value = float(raw_value)
            except (TypeError, ValueError) as exc:
                raise TypeError(
                    f"Actor FSM config {name} must be a finite number."
                ) from exc
            if not math.isfinite(value):
                raise ValueError(
                    f"Actor FSM config {name} must be finite."
                )
        history_window = phase_fsm.config.duration_history_window_cycles
        if (
            not isinstance(history_window, int)
            or isinstance(history_window, bool)
            or history_window < 1
        ):
            raise ValueError(
                "Actor FSM duration_history_window_cycles must be a positive "
                "integer."
            )

    @staticmethod
    def _validate_strict_json(value: object, label: str) -> None:
        try:
            json.dumps(value, allow_nan=False)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"{label} must be strict JSON without NaN or Inf."
            ) from exc

    @classmethod
    def _validate_baseline_transfer(
        cls,
        binary_payload: Mapping[str, Any],
        phase_payload: Mapping[str, Any],
        *,
        time_s: float,
        in_contact: bool,
    ) -> None:
        if binary_payload.get("events_this_step") != []:
            raise ValueError("V20 baseline must not emit an event.")
        observed_time = cls._finite_number(
            binary_payload.get("last_sample_time_s"),
            "V20 baseline last_sample_time_s",
        )
        if abs(observed_time - time_s) > _TIME_TOLERANCE_S:
            raise ValueError("V20 baseline timestamp mismatch.")
        if binary_payload.get("in_contact") is not in_contact:
            raise ValueError("V20 baseline contact mismatch.")
        if phase_payload.get("accepted_transitions_this_step") != []:
            raise ValueError("Actor FSM baseline must not accept a transition.")
        if float(phase_payload.get("valid_hs_count", -1.0)) != 0.0:
            raise ValueError("Actor FSM baseline invented a heel strike.")
        if float(phase_payload.get("valid_to_count", -1.0)) != 0.0:
            raise ValueError("Actor FSM baseline invented a toe off.")
        expected_state = STANCE_AFTER_HS if in_contact else WAIT_HS
        if float(phase_payload.get("state_id", -1.0)) != float(expected_state):
            raise ValueError("Actor FSM baseline state mismatch.")
        partial = float(
            phase_payload.get("sensor_partial_stance_active", -1.0)
        )
        if partial != float(in_contact):
            raise ValueError("Actor FSM partial-stance bootstrap mismatch.")

    @classmethod
    def _adapt_events(
        cls,
        binary_payload: Mapping[str, Any],
        *,
        phase_fsm: ProstheticPhaseFSM,
        boundary: float,
        previous: float,
    ) -> list[dict[str, Any]]:
        if not isinstance(binary_payload, Mapping):
            raise TypeError("V20 payload must be a mapping.")
        if binary_payload.get("source") != V20_SOURCE:
            raise ValueError("V20 payload source mismatch.")
        if (
            binary_payload.get("event_contract_id")
            != BINARY_ACTIVE_EVENT_CONTRACT_ID
        ):
            raise ValueError("V20 payload active contract mismatch.")
        last_sample = cls._finite_number(
            binary_payload.get("last_sample_time_s"),
            "V20 last_sample_time_s",
        )
        if abs(last_sample - boundary) > _TIME_TOLERANCE_S:
            raise ValueError("V20 payload was not delivered at this boundary.")
        raw_events = binary_payload.get("events_this_step")
        if isinstance(raw_events, (str, bytes, bytearray)) or not isinstance(
            raw_events,
            Sequence,
        ):
            raise TypeError("V20 events_this_step must be an ordered sequence.")

        phase_before = phase_fsm.payload()
        partial_stance = bool(
            float(phase_before.get("sensor_partial_stance_active", 0.0))
        )
        adapted: list[dict[str, Any]] = []
        previous_event_time: float | None = None
        identities: set[tuple[str, float, float, float]] = set()
        for index, raw_event in enumerate(raw_events):
            if not isinstance(raw_event, Mapping):
                raise TypeError(f"V20 event {index} must be a mapping.")
            if str(raw_event.get("side", "")).strip().lower() != "left":
                raise ValueError("V20 active events must be left-sided.")
            name = str(raw_event.get("event", "")).strip().lower()
            if name not in {"heel_strike", "toe_off"}:
                raise ValueError("V20 active event type is invalid.")
            if raw_event.get("source") != V20_SOURCE:
                raise ValueError("V20 active event source mismatch.")
            if (
                raw_event.get("event_contract_id")
                != BINARY_ACTIVE_EVENT_CONTRACT_ID
            ):
                raise ValueError("V20 active event contract mismatch.")
            event_time = cls._finite_number(
                raw_event.get("event_time_s"),
                f"V20 event {index} event_time_s",
            )
            confirmed = cls._finite_number(
                raw_event.get("confirmed_time_s"),
                f"V20 event {index} confirmed_time_s",
            )
            delivered = cls._finite_number(
                raw_event.get("delivered_time_s"),
                f"V20 event {index} delivered_time_s",
            )
            if not (
                event_time <= confirmed + _TIME_TOLERANCE_S
                and confirmed <= delivered + _TIME_TOLERANCE_S
                and abs(delivered - boundary) <= _TIME_TOLERANCE_S
            ):
                raise ValueError(
                    "V20 active event timestamps are not causal at the "
                    "current policy boundary."
                )
            if (
                abs(
                    (confirmed - event_time)
                    - BINARY_ACTIVE_DEBOUNCE_S
                )
                > _TIME_TOLERANCE_S
            ):
                raise ValueError(
                    "V20 active confirmation latency must equal 5 ms."
                )
            if not (
                confirmed > previous + _TIME_TOLERANCE_S
                and confirmed <= boundary + _TIME_TOLERANCE_S
            ):
                raise ValueError(
                    "V20 active confirmation is outside the current open-left "
                    "policy interval."
                )
            if (
                delivered - confirmed < -_TIME_TOLERANCE_S
                or delivered - confirmed
                > BINARY_ACTIVE_MAX_DELIVERY_DELAY_S + _TIME_TOLERANCE_S
            ):
                raise ValueError(
                    "V20 active delivery latency exceeds 10 ms."
                )
            if previous_event_time is not None and (
                event_time <= previous_event_time + _TIME_TOLERANCE_S
            ):
                raise ValueError("V20 active events are not strictly ordered.")
            identity = (name, event_time, confirmed, delivered)
            if identity in identities:
                raise ValueError("V20 active event is duplicated.")
            startup_partial = raw_event.get("startup_partial_stance")
            if startup_partial is not None and type(startup_partial) is not bool:
                raise TypeError(
                    "startup_partial_stance must be a native bool when present."
                )
            if name == "toe_off" and bool(startup_partial) != partial_stance:
                raise ValueError(
                    "V20 leading toe-off and actor partial-stance state disagree."
                )

            event = copy.deepcopy(dict(raw_event))
            event.update(
                {
                    "side": "left",
                    "event": name,
                    "time": event_time,
                    "confirmed_time": confirmed,
                    "event_time_s": event_time,
                    "confirmed_time_s": confirmed,
                    "delivered_time_s": delivered,
                    "source": BINARY_ACTIVE_ADAPTER_SOURCE,
                    "event_contract_id": BINARY_ACTIVE_EVENT_CONTRACT_ID,
                    "v20_source": V20_SOURCE,
                }
            )
            adapted.append(event)
            identities.add(identity)
            previous_event_time = event_time
        return adapted

    @classmethod
    def _validate_transfer(
        cls,
        adapted_events: Sequence[Mapping[str, Any]],
        phase_payload: Mapping[str, Any],
    ) -> None:
        cls._validate_phase_runtime_state(phase_payload)
        if phase_payload.get("event_source") != "binary_active":
            raise ValueError("Actor FSM active source was not preserved.")
        if float(phase_payload.get("invalid_event_this_step", 0.0)) != 0.0:
            rejected_events = []
            for index, event in enumerate(adapted_events):
                rejected_events.append(
                    {
                        "side": str(event.get("side", "")),
                        "event": str(event.get("event", "")),
                        "source": str(event.get("source", "")),
                        "event_contract_id": str(
                            event.get("event_contract_id", "")
                        ),
                        "event_time_s": cls._finite_number(
                            event.get("event_time_s"),
                            f"rejected actor event {index} event_time_s",
                        ),
                        "confirmed_time_s": cls._finite_number(
                            event.get("confirmed_time_s"),
                            f"rejected actor event {index} confirmed_time_s",
                        ),
                        "delivered_time_s": cls._finite_number(
                            event.get("delivered_time_s"),
                            f"rejected actor event {index} delivered_time_s",
                        ),
                    }
                )
            diagnostic = {
                "invalid_event_type": str(
                    phase_payload.get("invalid_event_type", "")
                ),
                "state_name": str(phase_payload.get("state_name", "")),
                "adapted_events": rejected_events,
            }
            encoded = json.dumps(
                diagnostic,
                allow_nan=False,
                sort_keys=True,
                separators=(",", ":"),
            )
            raise ValueError(
                "Actor FSM rejected a V20 active event: " + encoded
            )
        transitions_raw = phase_payload.get("accepted_transitions_this_step")
        if isinstance(transitions_raw, (str, bytes, bytearray)) or not isinstance(
            transitions_raw,
            Sequence,
        ):
            raise TypeError("Actor FSM transition journal must be a sequence.")
        transitions = [
            item
            for item in transitions_raw
            if isinstance(item, Mapping)
            and str(item.get("event", "")).strip().lower()
            in {"heel_strike", "toe_off"}
        ]
        if len(transitions) != len(adapted_events):
            raise ValueError(
                "V20-to-actor transfer was incomplete or duplicated."
            )
        for event, transition in zip(adapted_events, transitions):
            if str(transition.get("event", "")).lower() != str(event["event"]):
                raise ValueError("V20-to-actor event order mismatch.")
            for field in (
                "event_time_s",
                "confirmed_time_s",
                "delivered_time_s",
            ):
                observed = cls._finite_number(
                    transition.get(field),
                    f"actor transition {field}",
                )
                if abs(observed - float(event[field])) > _TIME_TOLERANCE_S:
                    raise ValueError(
                        f"V20-to-actor timestamp mismatch for {field}."
                    )

    @staticmethod
    def _accepted_left_events(
        phase_payload: Mapping[str, Any],
        *,
        adapted_events: Sequence[Mapping[str, Any]],
    ) -> list[dict[str, Any]]:
        result: list[dict[str, Any]] = []
        transitions = [
            transition
            for transition in phase_payload.get(
                "accepted_transitions_this_step",
                [],
            )
            if isinstance(transition, Mapping)
            and str(transition.get("event", "")).strip().lower()
            in {"heel_strike", "toe_off"}
        ]
        for transition, adapted in zip(transitions, adapted_events):
            if not isinstance(transition, Mapping):
                continue
            name = str(transition.get("event", "")).strip().lower()
            if name not in {"heel_strike", "toe_off"}:
                continue
            event_time = float(transition["event_time_s"])
            confirmed = float(transition["confirmed_time_s"])
            delivered = float(transition["delivered_time_s"])
            event: dict[str, Any] = {
                "side": "left",
                "event": name,
                "time": event_time,
                "confirmed_time": confirmed,
                "event_time_s": event_time,
                "confirmed_time_s": confirmed,
                "delivered_time_s": delivered,
                "source": BINARY_ACTIVE_ADAPTER_SOURCE,
                "event_contract_id": BINARY_ACTIVE_EVENT_CONTRACT_ID,
            }
            for diagnostic in (
                "startup_partial_stance",
                "landing_sensor",
                "contact_leader",
                "semantic",
                "onset_contact_state",
                "confirmed_contact_state",
            ):
                if diagnostic in adapted:
                    event[diagnostic] = copy.deepcopy(adapted[diagnostic])
            if name == "heel_strike":
                period = float(phase_payload.get("last_period_s", 0.0) or 0.0)
                event["cycle_duration_s"] = period if period > 0.0 else None
            result.append(event)
        return result

    @staticmethod
    def _native_bool(value: object, field_name: str) -> bool:
        if type(value) is not bool:
            raise TypeError(f"{field_name} must be a native bool.")
        return value

    @staticmethod
    def _finite_number(value: object, field_name: str) -> float:
        if isinstance(value, bool):
            raise TypeError(f"{field_name} must be a finite number.")
        try:
            result = float(value)
        except (TypeError, ValueError) as exc:
            raise TypeError(f"{field_name} must be a finite number.") from exc
        if not math.isfinite(result):
            raise ValueError(f"{field_name} must be finite.")
        return result

    @classmethod
    def _optional_finite(
        cls,
        value: float | None,
        field_name: str,
    ) -> float | None:
        return None if value is None else cls._finite_number(value, field_name)


__all__ = [
    "BinaryPhaseActiveAdapter",
    "BinaryPhaseActiveResult",
    "BINARY_ACTIVE_ADAPTER_SOURCE",
    "BINARY_ACTIVE_EVENT_CONTRACT_ID",
    "V20_SOURCE",
]
