"""Transactional V26-to-actor bridge for the frozen V25 binary detector.

The external runtime topology remains ``binary_active``.  Internally this
adapter pins the heel-qualified V26 detector source and the distinct
``binary_active_v26`` actor event source.  V20 remains importable and is never
silently accepted by this lineage.
"""

from __future__ import annotations

import copy
from typing import Any, Mapping, Sequence

from binary_phase_adapter import (
    BINARY_ACTIVE_ADAPTER_SOURCE,
    BINARY_ACTIVE_EVENT_CONTRACT_ID,
    BinaryPhaseActiveAdapter,
    BinaryPhaseActiveResult,
    V20_SOURCE,
    _BINARY_STATE_IDS,
    _TIME_TOLERANCE_S,
)
from binary_phase_fsm_v26 import (
    HeelQualifiedBinaryPhaseFSM,
    V26_EVENT_CONTRACT_ID,
    V26_SOURCE,
)
from prosthetic_phase_fsm import (
    BINARY_ACTIVE_V26_ADAPTER_SOURCE,
    BINARY_ACTIVE_V26_EVENT_CONTRACT_ID,
    ProstheticPhaseFSM,
)


V26_ACTOR_EVENT_SOURCE = "binary_active_v26"


class BinaryPhaseActiveAdapterV26(BinaryPhaseActiveAdapter):
    """Bridge heel-qualified V26 events into ``ProstheticPhaseFSM``."""

    def prime(
        self,
        *,
        binary_fsm: HeelQualifiedBinaryPhaseFSM,
        phase_fsm: ProstheticPhaseFSM,
        time_s: float,
        heel_contact: bool,
        toe_contact: bool,
    ) -> BinaryPhaseActiveResult:
        result = super().prime(
            binary_fsm=binary_fsm,
            phase_fsm=phase_fsm,
            time_s=time_s,
            heel_contact=heel_contact,
            toe_contact=toe_contact,
        )
        payload = {
            "mode": "binary_active",
            "actor_event_source": V26_ACTOR_EVENT_SOURCE,
            "adapter_source": BINARY_ACTIVE_V26_ADAPTER_SOURCE,
            "event_contract_id": BINARY_ACTIVE_V26_EVENT_CONTRACT_ID,
            "baseline_time_s": float(time_s),
            "baseline_contact_state": result.binary_payload[
                "stable_contact_state"
            ],
            "baseline_in_contact": bool(heel_contact or toe_contact),
            "partial_stance_bootstrap": bool(heel_contact or toe_contact),
            "events_adapted_this_step": 0,
            "atomic_commit_ready": True,
        }
        self._validate_strict_json(payload, "V26 binary active reset adapter")
        return BinaryPhaseActiveResult(
            binary_fsm=result.binary_fsm,
            phase_fsm=result.phase_fsm,
            binary_payload=result.binary_payload,
            phase_payload=result.phase_payload,
            left_events=result.left_events,
            adapter_payload=payload,
        )

    def advance(
        self,
        *,
        binary_fsm: HeelQualifiedBinaryPhaseFSM,
        phase_fsm: ProstheticPhaseFSM,
        time_s: float,
        previous_time_s: float,
        sensor_samples: Sequence[Mapping[str, object]],
        normal_force_bw: float,
        in_contact: bool,
        prosthetic_knee_angle_rad: float | None = None,
        prosthetic_ankle_angle_rad: float | None = None,
    ) -> BinaryPhaseActiveResult:
        result = super().advance(
            binary_fsm=binary_fsm,
            phase_fsm=phase_fsm,
            time_s=time_s,
            previous_time_s=previous_time_s,
            sensor_samples=sensor_samples,
            normal_force_bw=normal_force_bw,
            in_contact=in_contact,
            prosthetic_knee_angle_rad=prosthetic_knee_angle_rad,
            prosthetic_ankle_angle_rad=prosthetic_ankle_angle_rad,
        )
        payload = {
            "mode": "binary_active",
            "actor_event_source": V26_ACTOR_EVENT_SOURCE,
            "adapter_source": BINARY_ACTIVE_V26_ADAPTER_SOURCE,
            "event_contract_id": BINARY_ACTIVE_V26_EVENT_CONTRACT_ID,
            "previous_time_s": float(previous_time_s),
            "delivered_time_s": float(time_s),
            "events_adapted_this_step": len(result.left_events),
            "event_order_this_step": [
                str(event["event"]) for event in result.left_events
            ],
            "atomic_commit_ready": True,
        }
        self._validate_strict_json(payload, "V26 binary active adapter")
        return BinaryPhaseActiveResult(
            binary_fsm=result.binary_fsm,
            phase_fsm=result.phase_fsm,
            binary_payload=result.binary_payload,
            phase_payload=result.phase_payload,
            left_events=result.left_events,
            adapter_payload=payload,
        )

    @staticmethod
    def _validate_pair(
        binary_fsm: HeelQualifiedBinaryPhaseFSM,
        phase_fsm: ProstheticPhaseFSM,
    ) -> None:
        if not isinstance(binary_fsm, HeelQualifiedBinaryPhaseFSM):
            raise TypeError(
                "binary_fsm must be a HeelQualifiedBinaryPhaseFSM instance."
            )
        if not isinstance(phase_fsm, ProstheticPhaseFSM):
            raise TypeError("phase_fsm must be a ProstheticPhaseFSM instance.")
        cfg = binary_fsm.config
        if str(cfg.source) != V26_SOURCE:
            raise ValueError("Binary FSM source is not the frozen V26 source.")
        if str(cfg.event_contract_id) != BINARY_ACTIVE_V26_EVENT_CONTRACT_ID:
            raise ValueError("Binary FSM V26 active event contract mismatch.")
        if abs(float(cfg.sample_dt_s) - 0.001) > 1e-12:
            raise ValueError("Binary active V26 sample_dt_s must be 0.001.")
        if abs(float(cfg.debounce_s) - 0.005) > 1e-12:
            raise ValueError("Binary active V26 debounce_s must be 0.005.")
        if abs(float(cfg.max_delivery_delay_s) - 0.010) > 1e-12:
            raise ValueError(
                "Binary active V26 max_delivery_delay_s must be 0.010."
            )
        if phase_fsm.config.event_source != V26_ACTOR_EVENT_SOURCE:
            raise ValueError(
                "Actor-facing FSM must use event_source='binary_active_v26'."
            )
        BinaryPhaseActiveAdapterV26._validate_binary_runtime_state(
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
            raise TypeError("V26 runtime payload must be a mapping.")
        if payload.get("source") != V26_SOURCE:
            raise ValueError("V26 runtime payload source mismatch.")
        if payload.get("event_contract_id") != V26_EVENT_CONTRACT_ID:
            raise ValueError("V26 runtime payload contract mismatch.")
        for state_field, id_field in (
            ("raw_contact_state", "raw_contact_state_id"),
            ("stable_contact_state", "stable_contact_state_id"),
        ):
            state_name = str(payload.get(state_field, ""))
            expected_id = _BINARY_STATE_IDS.get(state_name)
            if expected_id is None:
                raise ValueError(f"V26 {state_field} is unknown.")
            observed_id = cls._finite_number(
                payload.get(id_field),
                f"V26 {id_field}",
            )
            if observed_id != float(expected_id):
                raise ValueError(
                    f"V26 {state_field}/{id_field} are inconsistent."
                )
        in_contact = payload.get("in_contact")
        if type(in_contact) is not bool:
            raise TypeError("V26 in_contact must be a native bool.")
        stable = str(payload["stable_contact_state"])
        if stable == "AIR" and in_contact:
            raise ValueError("V26 AIR cannot hold the functional stance latch.")
        if stable in {"HEEL", "BOTH"} and not in_contact:
            raise ValueError(
                "V26 heel-bearing stable contact requires the stance latch."
            )
        expected_phase = "STANCE" if in_contact else "SWING"
        if payload.get("gait_phase") != expected_phase:
            raise ValueError("V26 gait phase and functional latch disagree.")
        cls._validate_strict_json(payload, "V26 payload")

    @classmethod
    def _adapt_events(
        cls,
        binary_payload: Mapping[str, Any],
        *,
        phase_fsm: ProstheticPhaseFSM,
        boundary: float,
        previous: float,
    ) -> list[dict[str, Any]]:
        cls._validate_binary_runtime_state(binary_payload)
        raw_events = binary_payload.get("events_this_step")
        if isinstance(raw_events, (str, bytes, bytearray)) or not isinstance(
            raw_events,
            Sequence,
        ):
            raise TypeError("V26 events_this_step must be an ordered sequence.")
        for index, event in enumerate(raw_events):
            if not isinstance(event, Mapping):
                raise TypeError(f"V26 event {index} must be a mapping.")
            if event.get("source") != V26_SOURCE:
                raise ValueError("V26 active event source mismatch.")
            if event.get("event_contract_id") != V26_EVENT_CONTRACT_ID:
                raise ValueError("V26 active event contract mismatch.")

        # Reuse the already-audited timestamp/order validator after an explicit
        # provenance check.  The transformed copy never escapes this method.
        legacy_view = copy.deepcopy(dict(binary_payload))
        legacy_view["source"] = V20_SOURCE
        legacy_view["event_contract_id"] = BINARY_ACTIVE_EVENT_CONTRACT_ID
        for event in legacy_view["events_this_step"]:
            event["source"] = V20_SOURCE
            event["event_contract_id"] = BINARY_ACTIVE_EVENT_CONTRACT_ID
        adapted = BinaryPhaseActiveAdapter._adapt_events(
            legacy_view,
            phase_fsm=phase_fsm,
            boundary=boundary,
            previous=previous,
        )
        for event in adapted:
            event["source"] = BINARY_ACTIVE_V26_ADAPTER_SOURCE
            event["event_contract_id"] = BINARY_ACTIVE_V26_EVENT_CONTRACT_ID
            event.pop("v20_source", None)
            event["v26_source"] = V26_SOURCE
        return adapted

    @classmethod
    def _validate_transfer(
        cls,
        adapted_events: Sequence[Mapping[str, Any]],
        phase_payload: Mapping[str, Any],
    ) -> None:
        if phase_payload.get("event_source") != V26_ACTOR_EVENT_SOURCE:
            raise ValueError("Actor FSM V26 source was not preserved.")
        legacy_phase_view = copy.deepcopy(dict(phase_payload))
        legacy_phase_view["event_source"] = "binary_active"
        BinaryPhaseActiveAdapter._validate_transfer(
            adapted_events,
            legacy_phase_view,
        )

    @staticmethod
    def _accepted_left_events(
        phase_payload: Mapping[str, Any],
        *,
        adapted_events: Sequence[Mapping[str, Any]],
    ) -> list[dict[str, Any]]:
        result = BinaryPhaseActiveAdapter._accepted_left_events(
            phase_payload,
            adapted_events=adapted_events,
        )
        for output, adapted in zip(result, adapted_events):
            output["source"] = BINARY_ACTIVE_V26_ADAPTER_SOURCE
            output["event_contract_id"] = BINARY_ACTIVE_V26_EVENT_CONTRACT_ID
            for field in ("hs_semantics", "v26_source"):
                if field in adapted:
                    output[field] = copy.deepcopy(adapted[field])
        return result


__all__ = [
    "BinaryPhaseActiveAdapterV26",
    "BINARY_ACTIVE_V26_ADAPTER_SOURCE",
    "BINARY_ACTIVE_V26_EVENT_CONTRACT_ID",
    "V26_ACTOR_EVENT_SOURCE",
]
