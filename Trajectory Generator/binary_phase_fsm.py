"""Fail-closed FSM for the force-free V19 binary foot-contact detector.

The physical detector exposes two Boolean samples (heel and toe).  This
module deliberately keeps their interpretation separate from the historical
``ProstheticPhaseFSM``: the four contact words are diagnostic, while gait
events depend only on the debounced AIR/CONTACT class.
"""

from __future__ import annotations

import copy
import math
from dataclasses import dataclass
from typing import Mapping, Sequence


_TIME_TOLERANCE_S = 1e-9

CONTACT_AIR = "AIR"
CONTACT_HEEL = "HEEL"
CONTACT_BOTH = "BOTH"
CONTACT_TOE = "TOE"

_CONTACT_STATE_BY_BITS = {
    (False, False): CONTACT_AIR,
    (True, False): CONTACT_HEEL,
    (True, True): CONTACT_BOTH,
    (False, True): CONTACT_TOE,
}
_CONTACT_STATE_ID = {
    CONTACT_AIR: 0,
    CONTACT_TOE: 1,
    CONTACT_HEEL: 2,
    CONTACT_BOTH: 3,
}


@dataclass(frozen=True)
class BinaryPhaseFSMConfig:
    """Frozen timing contract for functional HS/TO extraction."""

    sample_dt_s: float = 0.001
    debounce_s: float = 0.005
    max_delivery_delay_s: float = 0.010
    source: str = "binary_phase_fsm_v20"
    event_contract_id: str = (
        "binary_point_v19+functional_contact_fsm_v1_shadow"
    )

    def __post_init__(self) -> None:
        sample_dt_s = float(self.sample_dt_s)
        debounce_s = float(self.debounce_s)
        max_delivery_delay_s = float(self.max_delivery_delay_s)
        if not math.isfinite(sample_dt_s) or sample_dt_s <= 0.0:
            raise ValueError("sample_dt_s must be finite and positive.")
        if not math.isfinite(debounce_s) or debounce_s <= 0.0:
            raise ValueError("debounce_s must be finite and positive.")
        if not math.isfinite(max_delivery_delay_s) or max_delivery_delay_s <= 0.0:
            raise ValueError("max_delivery_delay_s must be finite and positive.")
        ratio = debounce_s / sample_dt_s
        if abs(ratio - round(ratio)) > 1e-9:
            raise ValueError("debounce_s must be an integer multiple of sample_dt_s.")
        if not str(self.source).strip():
            raise ValueError("source must be non-empty.")
        if not str(self.event_contract_id).strip():
            raise ValueError("event_contract_id must be non-empty.")


class BinaryPhaseFSM:
    """Debounce the V19 heel/toe bits and emit functional HS/TO events.

    ``heel_strike`` means the first stable transition from AIR to any contact.
    ``toe_off`` means the first stable transition from any contact to AIR.
    Confirmation occurs exactly ``debounce_s`` after the raw onset and the
    event keeps that onset as ``event_time_s``.

    A policy batch is validated completely before any state is mutated.  This
    makes missing, duplicate, non-monotonic, off-grid, non-finite, or non-bool
    samples fail closed.
    """

    _SAMPLE_FIELDS = frozenset(
        {"time_s", "left_heel_contact", "left_toe_contact"}
    )

    def __init__(self, config: BinaryPhaseFSMConfig | None = None) -> None:
        self.config = config or BinaryPhaseFSMConfig()
        self.reset()

    def reset(
        self,
        *,
        time_s: float | None = None,
        heel_contact: bool = False,
        toe_contact: bool = False,
    ) -> dict:
        """Reset at a sampled boundary without attributing an event to it."""

        if time_s is None:
            if heel_contact is not False or toe_contact is not False:
                raise ValueError("Contact bits require an explicit reset time_s.")
            baseline_time = None
            heel = False
            toe = False
        else:
            baseline_time = self._finite_time(time_s, "time_s")
            heel = self._strict_bool(heel_contact, "heel_contact")
            toe = self._strict_bool(toe_contact, "toe_contact")

        contact_state = _CONTACT_STATE_BY_BITS[(heel, toe)]
        self._last_sample_time_s: float | None = baseline_time
        self._raw_contact_state = contact_state
        self._stable_contact_state = contact_state
        self._pending_contact_state: str | None = None
        self._pending_contact_since_s: float | None = None
        self._in_contact = bool(heel or toe)
        self._pending_phase_target: bool | None = None
        self._pending_phase_since_s: float | None = None
        self._pending_phase_onset_state: str | None = None
        self._events_this_step: list[dict] = []
        self._contact_state_transitions_this_step: list[dict] = []
        self._candidate_cancellations_this_step: list[dict] = []
        self._candidate_cancellation_count = 0
        self._event_count = {"heel_strike": 0, "toe_off": 0}
        self._cycle_count = 0
        self._last_event: str | None = None
        self._last_hs_event_time_s: float | None = None
        self._last_to_event_time_s: float | None = None
        self._samples_processed = 0
        return self.payload()

    def update(
        self,
        *,
        time_s: float,
        heel_contact: bool,
        toe_contact: bool,
        delivered_time_s: float | None = None,
    ) -> dict:
        """Process one 1 ms sample through the legacy-compatible scalar API."""

        self._require_initialised()
        sample = self._validate_sample(
            {
                "time_s": time_s,
                "left_heel_contact": heel_contact,
                "left_toe_contact": toe_contact,
            }
        )
        expected_time = float(self._last_sample_time_s) + self.config.sample_dt_s
        self._require_time_close(sample[0], expected_time, "scalar sample time")
        delivery = sample[0] if delivered_time_s is None else self._finite_time(
            delivered_time_s, "delivered_time_s"
        )
        if delivery + _TIME_TOLERANCE_S < sample[0]:
            raise ValueError("delivered_time_s cannot precede the sample time.")
        if delivery - sample[0] > self.config.max_delivery_delay_s + _TIME_TOLERANCE_S:
            raise ValueError("delivered_time_s exceeds the fixed policy-delay bound.")

        self._reset_transients()
        self._process_sample(*sample, delivered_time_s=delivery)
        return self.payload()

    # Explicit name retained alongside ``update`` for callers that want to
    # distinguish the scalar reference from the policy-batch API.
    update_sample = update

    def update_policy_step(
        self,
        *,
        time_s: float,
        previous_time_s: float,
        sensor_samples: Sequence[Mapping[str, object]],
    ) -> dict:
        """Validate and process one policy-step batch, delivering events once."""

        self._require_initialised()
        boundary = self._finite_time(time_s, "time_s")
        previous = self._finite_time(previous_time_s, "previous_time_s")
        if boundary <= previous:
            raise ValueError("time_s must be strictly greater than previous_time_s.")
        if (
            boundary - previous
            > self.config.max_delivery_delay_s + _TIME_TOLERANCE_S
        ):
            raise ValueError("Policy batch exceeds the fixed delivery-delay bound.")
        self._require_time_close(
            previous,
            float(self._last_sample_time_s),
            "previous_time_s",
        )
        if isinstance(sensor_samples, (str, bytes)) or not isinstance(
            sensor_samples, Sequence
        ):
            raise TypeError("sensor_samples must be a sequence of mappings.")

        expected_count_float = (boundary - previous) / self.config.sample_dt_s
        expected_count = int(round(expected_count_float))
        if (
            expected_count <= 0
            or abs(expected_count_float - expected_count) > 1e-9
        ):
            raise ValueError(
                "Policy boundaries must span a positive integer number of samples."
            )
        if len(sensor_samples) != expected_count:
            raise ValueError(
                f"Expected {expected_count} unique samples in "
                f"(previous_time_s, time_s], observed {len(sensor_samples)}."
            )

        # Validate the complete batch first.  No transient or persistent state
        # is touched until all entries satisfy the strict contract.
        validated: list[tuple[float, bool, bool]] = []
        for index, raw_sample in enumerate(sensor_samples, start=1):
            sample = self._validate_sample(raw_sample)
            expected_time = previous + index * self.config.sample_dt_s
            self._require_time_close(
                sample[0], expected_time, f"sensor_samples[{index - 1}].time_s"
            )
            validated.append(sample)
        self._require_time_close(validated[-1][0], boundary, "last sample time")

        self._reset_transients()
        for sample in validated:
            self._process_sample(*sample, delivered_time_s=boundary)
        return self.payload()

    def payload(self) -> dict:
        """Return a deep-copy-safe, strict-JSON-compatible diagnostic payload."""

        pending_event = None
        if self._pending_phase_target is not None:
            pending_event = {
                "event": (
                    "heel_strike" if self._pending_phase_target else "toe_off"
                ),
                "event_time_s": self._pending_phase_since_s,
                "onset_contact_state": self._pending_phase_onset_state,
            }
        return copy.deepcopy(
            {
                "source": str(self.config.source),
                "event_contract_id": str(self.config.event_contract_id),
                "sample_dt_s": float(self.config.sample_dt_s),
                "debounce_s": float(self.config.debounce_s),
                "max_delivery_delay_s": float(
                    self.config.max_delivery_delay_s
                ),
                "last_sample_time_s": self._last_sample_time_s,
                "samples_processed": int(self._samples_processed),
                "raw_contact_state": self._raw_contact_state,
                "raw_contact_state_id": _CONTACT_STATE_ID[self._raw_contact_state],
                "stable_contact_state": self._stable_contact_state,
                "stable_contact_state_id": _CONTACT_STATE_ID[
                    self._stable_contact_state
                ],
                "in_contact": bool(self._in_contact),
                "gait_phase": "STANCE" if self._in_contact else "SWING",
                "pending_event": pending_event,
                "pending_contact_state": self._pending_contact_state,
                "events_this_step": self._events_this_step,
                "contact_state_transitions_this_step": (
                    self._contact_state_transitions_this_step
                ),
                "candidate_cancellations_this_step": (
                    self._candidate_cancellations_this_step
                ),
                "candidate_cancellation_count": int(
                    self._candidate_cancellation_count
                ),
                "event_count": dict(self._event_count),
                "cycle_count": int(self._cycle_count),
                "last_event": self._last_event,
                "last_hs_event_time_s": self._last_hs_event_time_s,
                "last_to_event_time_s": self._last_to_event_time_s,
            }
        )

    def _process_sample(
        self,
        time_s: float,
        heel_contact: bool,
        toe_contact: bool,
        *,
        delivered_time_s: float,
    ) -> None:
        raw_state = _CONTACT_STATE_BY_BITS[(heel_contact, toe_contact)]
        self._raw_contact_state = raw_state
        self._update_contact_word(raw_state, time_s, delivered_time_s)
        self._update_phase(bool(heel_contact or toe_contact), raw_state, time_s, delivered_time_s)
        self._last_sample_time_s = time_s
        self._samples_processed += 1

    def _update_contact_word(
        self, raw_state: str, time_s: float, delivered_time_s: float
    ) -> None:
        if raw_state == self._stable_contact_state:
            self._pending_contact_state = None
            self._pending_contact_since_s = None
            return
        if raw_state != self._pending_contact_state:
            self._pending_contact_state = raw_state
            self._pending_contact_since_s = time_s
            return
        onset = float(self._pending_contact_since_s)
        if time_s - onset + _TIME_TOLERANCE_S < self.config.debounce_s:
            return
        previous = self._stable_contact_state
        self._stable_contact_state = raw_state
        self._pending_contact_state = None
        self._pending_contact_since_s = None
        self._contact_state_transitions_this_step.append(
            {
                "from": previous,
                "to": raw_state,
                "event_time_s": onset,
                "confirmed_time_s": time_s,
                "delivered_time_s": delivered_time_s,
            }
        )

    def _update_phase(
        self,
        raw_in_contact: bool,
        raw_state: str,
        time_s: float,
        delivered_time_s: float,
    ) -> None:
        if raw_in_contact == self._in_contact:
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
        if raw_in_contact != self._pending_phase_target:
            self._pending_phase_target = raw_in_contact
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
                "functional_initial_contact"
                if target
                else "functional_final_contact"
            ),
            "onset_contact_state": onset_state,
            "confirmed_contact_state": raw_state,
        }
        if target:
            contact_leader = {
                CONTACT_HEEL: "heel",
                CONTACT_TOE: "toe",
                CONTACT_BOTH: "both",
            }.get(onset_state, "unknown")
            event["landing_sensor"] = contact_leader
            event["contact_leader"] = contact_leader
            event["hs_semantics"] = "first_stable_any_contact"
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

    def _reset_transients(self) -> None:
        self._events_this_step = []
        self._contact_state_transitions_this_step = []
        self._candidate_cancellations_this_step = []

    def _validate_sample(
        self, raw_sample: Mapping[str, object]
    ) -> tuple[float, bool, bool]:
        if not isinstance(raw_sample, Mapping):
            raise TypeError("Each detector sample must be a mapping.")
        observed_fields = frozenset(raw_sample.keys())
        if observed_fields != self._SAMPLE_FIELDS:
            missing = sorted(self._SAMPLE_FIELDS - observed_fields)
            unexpected = sorted(observed_fields - self._SAMPLE_FIELDS)
            raise ValueError(
                f"Detector sample fields mismatch; missing={missing}, "
                f"unexpected={unexpected}."
            )
        return (
            self._finite_time(raw_sample["time_s"], "sample time_s"),
            self._strict_bool(
                raw_sample["left_heel_contact"], "left_heel_contact"
            ),
            self._strict_bool(
                raw_sample["left_toe_contact"], "left_toe_contact"
            ),
        )

    @staticmethod
    def _strict_bool(value: object, field_name: str) -> bool:
        # NumPy bool scalars are intentionally rejected: the runner contract
        # serialises native Python booleans and strict typing catches drift.
        if type(value) is not bool:
            raise TypeError(f"{field_name} must be a native bool.")
        return value

    @staticmethod
    def _finite_time(value: object, field_name: str) -> float:
        if isinstance(value, bool):
            raise TypeError(f"{field_name} must be a finite number.")
        try:
            result = float(value)
        except (TypeError, ValueError) as exc:
            raise TypeError(f"{field_name} must be a finite number.") from exc
        if not math.isfinite(result):
            raise ValueError(f"{field_name} must be finite.")
        return result

    @staticmethod
    def _require_time_close(observed: float, expected: float, label: str) -> None:
        if abs(observed - expected) > _TIME_TOLERANCE_S:
            raise ValueError(
                f"{label} is off-grid: expected {expected:.12g}, "
                f"observed {observed:.12g}."
            )

    def _require_initialised(self) -> None:
        if self._last_sample_time_s is None:
            raise RuntimeError(
                "BinaryPhaseFSM.reset(time_s=..., heel_contact=..., "
                "toe_contact=...) must establish the t0 baseline first."
            )


__all__ = [
    "BinaryPhaseFSM",
    "BinaryPhaseFSMConfig",
    "CONTACT_AIR",
    "CONTACT_HEEL",
    "CONTACT_BOTH",
    "CONTACT_TOE",
]
