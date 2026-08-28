"""Heel-qualified binary gait-event FSM for the frozen V25 detector.

V25 continues to expose the same two force-free Boolean geometry samples.
This module changes only their functional event interpretation relative to
V20: while the functional latch is in swing, toe-only contact remains a raw
diagnostic and cannot start a stance.  A heel sample (HEEL or BOTH) must stay
present for the frozen 5 ms debounce before a heel strike is confirmed.
Once stance is active, contact is retained through HEEL, BOTH, and TOE; toe
off still requires stable AIR.

The V20 implementation is intentionally inherited rather than edited so its
historical source and behaviour remain available byte-for-byte.
"""

from __future__ import annotations

from dataclasses import dataclass

from binary_phase_fsm import (
    CONTACT_BOTH,
    CONTACT_HEEL,
    CONTACT_TOE,
    BinaryPhaseFSM,
    BinaryPhaseFSMConfig,
    _TIME_TOLERANCE_S,
)


V26_SOURCE = "binary_phase_fsm_v26"
V26_EVENT_CONTRACT_ID = "binary_point_v25+heel_qualified_fsm_v2"


@dataclass(frozen=True)
class HeelQualifiedBinaryPhaseFSMConfig(BinaryPhaseFSMConfig):
    """Frozen V26 timing and provenance contract."""

    source: str = V26_SOURCE
    event_contract_id: str = V26_EVENT_CONTRACT_ID


class HeelQualifiedBinaryPhaseFSM(BinaryPhaseFSM):
    """Emit heel-qualified HS and stable-AIR TO from V25 Boolean samples."""

    def __init__(
        self,
        config: HeelQualifiedBinaryPhaseFSMConfig | None = None,
    ) -> None:
        super().__init__(config or HeelQualifiedBinaryPhaseFSMConfig())

    def _update_phase(
        self,
        raw_in_contact: bool,
        raw_state: str,
        time_s: float,
        delivered_time_s: float,
    ) -> None:
        # In stance, every non-AIR word retains contact and only AIR may start
        # a TO candidate.  In swing, only heel-bearing words may start an HS;
        # TOE is deliberately observable but functionally equivalent to AIR.
        if self._in_contact:
            functional_target = bool(raw_in_contact)
        else:
            functional_target = raw_state in {CONTACT_HEEL, CONTACT_BOTH}

        if functional_target == self._in_contact:
            if self._pending_phase_target is not None:
                self._candidate_cancellations_this_step.append(
                    {
                        "event": (
                            "heel_strike"
                            if self._pending_phase_target
                            else "toe_off"
                        ),
                        "event_time_s": self._pending_phase_since_s,
                        "cancelled_time_s": time_s,
                        "reason": "raw_phase_returned_to_stable_latch",
                    }
                )
                self._candidate_cancellation_count += 1
            self._pending_phase_target = None
            self._pending_phase_since_s = None
            self._pending_phase_onset_state = None
            return

        if functional_target != self._pending_phase_target:
            self._pending_phase_target = functional_target
            self._pending_phase_since_s = time_s
            self._pending_phase_onset_state = raw_state
            return

        onset = float(self._pending_phase_since_s)
        if time_s - onset + _TIME_TOLERANCE_S < self.config.debounce_s:
            return

        target = bool(self._pending_phase_target)
        onset_state = str(self._pending_phase_onset_state)
        self._in_contact = target
        self._pending_phase_target = None
        self._pending_phase_since_s = None
        self._pending_phase_onset_state = None
        event_name = "heel_strike" if target else "toe_off"
        event = {
            "side": "left",
            "event": event_name,
            "time": onset,
            "event_time_s": onset,
            "confirmed_time_s": time_s,
            "confirmed_time": time_s,
            "delivered_time_s": delivered_time_s,
            "source": str(self.config.source),
            "event_contract_id": str(self.config.event_contract_id),
            "semantic": (
                "heel_qualified_initial_contact"
                if target
                else "functional_final_contact"
            ),
            "onset_contact_state": onset_state,
            "confirmed_contact_state": raw_state,
        }
        if target:
            contact_leader = {
                CONTACT_HEEL: "heel",
                CONTACT_BOTH: "both",
            }.get(onset_state, "unknown")
            event["landing_sensor"] = contact_leader
            event["contact_leader"] = contact_leader
            event["hs_semantics"] = "first_stable_heel_contact"
            if (
                self._last_hs_event_time_s is not None
                and self._last_to_event_time_s is not None
                and self._last_hs_event_time_s < self._last_to_event_time_s < onset
            ):
                self._cycle_count += 1
            self._last_hs_event_time_s = onset
        else:
            event["startup_partial_stance"] = self._last_hs_event_time_s is None
            self._last_to_event_time_s = onset
        self._event_count[event_name] += 1
        self._last_event = event_name
        self._events_this_step.append(event)


__all__ = [
    "HeelQualifiedBinaryPhaseFSM",
    "HeelQualifiedBinaryPhaseFSMConfig",
    "V26_EVENT_CONTRACT_ID",
    "V26_SOURCE",
]
