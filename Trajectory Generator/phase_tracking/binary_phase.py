"""Binary heel/toe gait-event FSMs and transactional actor-phase adapters.

The base FSM debounces AIR/contact transitions. The heel-qualified V26 FSM
requires stable heel contact to start stance; toe-only contact in swing stays
diagnostic. Both adapters advance private copies of the detector and actor
FSMs and return a complete candidate for an atomic commit.

Class inheritance, timing and event provenance are independent of this module
layout: the V20/V26 source labels and event contracts remain unchanged.
"""

from __future__ import annotations

import copy
import json
import math
import sys
from dataclasses import dataclass
from typing import Any, Mapping, Sequence

from prosthetic_phase_fsm import (
    BINARY_ACTIVE_ADAPTER_SOURCE,
    BINARY_ACTIVE_DEBOUNCE_S,
    BINARY_ACTIVE_EVENT_CONTRACT_ID,
    BINARY_ACTIVE_MAX_DELIVERY_DELAY_S,
    BINARY_ACTIVE_V26_ADAPTER_SOURCE,
    BINARY_ACTIVE_V26_EVENT_CONTRACT_ID,
    STATE_NAMES,
    STANCE_AFTER_HS,
    SWING_AFTER_TO,
    TIMEOUT,
    WAIT_HS,
    ProstheticPhaseFSM,
)


# Base contact FSM (V20).

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


# Heel-qualified contact FSM (V26).

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


# Transactional actor-phase adapter (V20).

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


class BinaryPhaseTransferError(ValueError):
    """Behaviour-dependent actor-FSM rejection of a V20/V26 active event.

    Subclasses ``ValueError`` so every existing fail-closed consumer keeps
    its semantics; training runtimes may catch THIS type specifically to
    terminate the episode instead of killing the worker."""


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
        invalid_event_mode: str = "raise",
    ) -> BinaryPhaseActiveResult:
        """Advance V20 then the actor FSM and return one atomic candidate.

        ``invalid_event_mode="drop"`` returns the candidate even when the
        actor FSM rejects the transferred event: both candidate FSMs have
        already consumed the 10 ms window at that point, so committing them
        is the only way to keep the sensor cursor and the FSM clocks aligned.
        The rejected event is not published (``left_events`` is empty) and the
        drop is flagged in ``adapter_payload``. The default ``"raise"`` keeps
        the fail-closed qualification behaviour byte for byte.
        """

        mode_label = str(invalid_event_mode).strip().lower()
        if mode_label not in {"raise", "drop"}:
            raise ValueError("invalid_event_mode must be 'raise' or 'drop'.")
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
            # Truth for the v3 resync monitor: the detector's FUNCTIONAL stance
            # latch (gait phase), not the raw contact word — toe-only contact
            # is legitimate swing (drag) in the heel-qualified contract.
            binary_contact=bool(binary_payload.get("in_contact", False)),
        )
        invalid_event_dropped = False
        try:
            self._validate_transfer(adapted_events, phase_payload)
        except BinaryPhaseTransferError:
            if mode_label != "drop":
                raise
            invalid_event_dropped = True
        if invalid_event_dropped:
            left_events = []
        else:
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
            "invalid_event_dropped": invalid_event_dropped,
            "invalid_event_type": (
                str(phase_payload.get("invalid_event_type", ""))
                if invalid_event_dropped
                else ""
            ),
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
            # v3 recovery flags are typed (bool / enumerated str), not numbers.
            if name in {"resync_enabled", "hs_cancel_enabled"}:
                if type(raw_value) is not bool:
                    raise TypeError(
                        f"Actor FSM config {name} must be a native bool."
                    )
                continue
            if name == "resync_contact_source":
                if raw_value not in {"binary", "primary"}:
                    raise ValueError(
                        "Actor FSM config resync_contact_source must be "
                        "'binary' or 'primary'."
                    )
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
            raise BinaryPhaseTransferError(
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


# Heel-qualified actor-phase adapter (V26).

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
        invalid_event_mode: str = "raise",
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
            invalid_event_mode=invalid_event_mode,
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
            "invalid_event_dropped": bool(
                result.adapter_payload.get("invalid_event_dropped", False)
            ),
            "invalid_event_type": str(
                result.adapter_payload.get("invalid_event_type", "")
            ),
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
        # Latch/phase coherence is behaviour-dependent (chaotic training gaits
        # can produce these payloads): typed as BinaryPhaseTransferError so RL
        # callers terminate the episode instead of losing the worker. Schema
        # and type violations above stay fatal.
        if stable == "AIR" and in_contact:
            raise BinaryPhaseTransferError(
                "V26 AIR cannot hold the functional stance latch."
            )
        if stable in {"HEEL", "BOTH"} and not in_contact:
            raise BinaryPhaseTransferError(
                "V26 heel-bearing stable contact requires the stance latch."
            )
        expected_phase = "STANCE" if in_contact else "SWING"
        if payload.get("gait_phase") != expected_phase:
            raise BinaryPhaseTransferError(
                "V26 gait phase and functional latch disagree."
            )
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
    "BinaryPhaseFSM",
    "BinaryPhaseFSMConfig",
    "CONTACT_AIR",
    "CONTACT_HEEL",
    "CONTACT_BOTH",
    "CONTACT_TOE",
    "HeelQualifiedBinaryPhaseFSM",
    "HeelQualifiedBinaryPhaseFSMConfig",
    "V26_EVENT_CONTRACT_ID",
    "V26_SOURCE",
    "BinaryPhaseActiveAdapter",
    "BinaryPhaseActiveResult",
    "BINARY_ACTIVE_ADAPTER_SOURCE",
    "BINARY_ACTIVE_EVENT_CONTRACT_ID",
    "V20_SOURCE",
    "BinaryPhaseActiveAdapterV26",
    "BINARY_ACTIVE_V26_ADAPTER_SOURCE",
    "BINARY_ACTIVE_V26_EVENT_CONTRACT_ID",
    "V26_ACTOR_EVENT_SOURCE",
    "BinaryPhaseTransferError",
]


# Old pickles resolve these module names after the current module (or the
# environment) has been imported. Keep one class identity without shim files.
for _legacy_module_name in (
    "binary_phase_fsm",
    "binary_phase_fsm_v26",
    "binary_phase_adapter",
    "binary_phase_adapter_v26",
):
    sys.modules.setdefault(_legacy_module_name, sys.modules[__name__])
del _legacy_module_name
