"""Runtime finite-state machine for prosthetic HS-TO-HS phase tracking.

The FSM is intentionally sensor-level: it consumes only prosthetic-side online
GRF detector events and force/contact flags. It stores per-episode state in
memory and exposes a compact observation payload plus richer diagnostics.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any, Mapping, Sequence

import numpy as np


WAIT_HS = 0
STANCE_AFTER_HS = 1
SWING_AFTER_TO = 2
VALID_CYCLE_COMPLETED = 3
TIMEOUT = 4
INVALID_EVENT = 5

BINARY_ACTIVE_ADAPTER_SOURCE = "v25_fsm_v20"
BINARY_ACTIVE_EVENT_CONTRACT_ID = (
    "binary_point_v25+functional_contact_fsm_v1"
)
BINARY_ACTIVE_V26_ADAPTER_SOURCE = "v25_fsm_v26"
BINARY_ACTIVE_V26_EVENT_CONTRACT_ID = (
    "binary_point_v25+heel_qualified_fsm_v2"
)
BINARY_ACTIVE_DEBOUNCE_S = 0.005
BINARY_ACTIVE_MAX_DELIVERY_DELAY_S = 0.010

_BINARY_ACTIVE_CONTRACTS = {
    "binary_active": {
        "adapter_source": BINARY_ACTIVE_ADAPTER_SOURCE,
        "event_contract_id": BINARY_ACTIVE_EVENT_CONTRACT_ID,
    },
    "binary_active_v26": {
        "adapter_source": BINARY_ACTIVE_V26_ADAPTER_SOURCE,
        "event_contract_id": BINARY_ACTIVE_V26_EVENT_CONTRACT_ID,
    },
}

_EVENT_CAUSALITY_TOLERANCE_S = 1e-12
_BINARY_ACTIVE_EVENT_CAUSALITY_TOLERANCE_S = 1e-9

STATE_NAMES = {
    WAIT_HS: "WAIT_HS",
    STANCE_AFTER_HS: "STANCE_AFTER_HS",
    SWING_AFTER_TO: "SWING_AFTER_TO",
    VALID_CYCLE_COMPLETED: "VALID_CYCLE_COMPLETED",
    TIMEOUT: "TIMEOUT",
    INVALID_EVENT: "INVALID_EVENT",
}


def _event_time_or_fallback(event: Mapping[str, Any], fallback: float) -> float:
    """Read an event timestamp without treating the valid value ``0.0`` as absent."""
    raw = event.get("event_time_s", event.get("time"))
    if raw is None:
        return float(fallback)
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return float(fallback)
    return value if np.isfinite(value) else float(fallback)


@dataclass(frozen=True)
class ProstheticPhaseFSMConfig:
    event_source: str = "legacy_events"
    detector_sample_dt_s: float = 0.001
    sensor_on_threshold_n: float = 5.0
    sensor_off_threshold_n: float = 2.0
    sensor_dwell_s: float = 0.03
    min_stance_duration_s: float = 0.05
    min_swing_duration_s: float = 0.20
    landing_window_start_s: float = 0.55
    landing_window_end_s: float = 1.10
    stance_hard_timeout_s: float = 2.20
    swing_hard_timeout_s: float = 1.30
    landing_force_full_credit_bw: float = 0.65
    min_stance_contact_fraction: float = 0.0
    min_stance_load_bw_s: float = 0.0
    min_cycle_knee_excursion_rad: float = 0.0
    hs_event_credit: float = 0.10
    toe_off_event_credit: float = 0.20
    cycle_complete_bonus: float = 0.70
    failure_extra_penalty: float = 0.05
    duration_history_window_cycles: int = 5

    def __post_init__(self) -> None:
        if self.event_source not in {
            "legacy_events",
            "shadow",
            "two_sensor",
            "binary_active",
            "binary_active_v26",
        }:
            raise ValueError(
                "event_source must be 'legacy_events', 'shadow', "
                "'two_sensor', 'binary_active', or 'binary_active_v26'."
            )
        on_threshold = float(self.sensor_on_threshold_n)
        off_threshold = float(self.sensor_off_threshold_n)
        dwell = float(self.sensor_dwell_s)
        sample_dt = float(self.detector_sample_dt_s)
        if not np.isfinite(sample_dt) or sample_dt <= 0.0:
            raise ValueError("detector_sample_dt_s must be finite and positive.")
        if not np.isfinite(on_threshold) or on_threshold <= 0.0:
            raise ValueError("sensor_on_threshold_n must be finite and positive.")
        if not np.isfinite(off_threshold) or off_threshold < 0.0:
            raise ValueError("sensor_off_threshold_n must be finite and non-negative.")
        if off_threshold >= on_threshold:
            raise ValueError(
                "sensor_off_threshold_n must be lower than sensor_on_threshold_n."
            )
        if not np.isfinite(dwell) or dwell < 0.0:
            raise ValueError("sensor_dwell_s must be finite and non-negative.")


class ProstheticPhaseFSM:
    """Per-episode HS-TO-HS state machine for the prosthetic side."""

    def __init__(self, config: ProstheticPhaseFSMConfig | None = None) -> None:
        self.config = config or ProstheticPhaseFSMConfig()
        self.reset()

    def reset(self) -> None:
        self.state_id = WAIT_HS
        self.last_valid_hs_time: float | None = None
        self.last_valid_to_time: float | None = None
        self.last_period_s = 0.0
        self.previous_period_s = 0.0
        self.last_stance_fraction = 0.0
        history_window = max(
            1,
            int(self.config.duration_history_window_cycles),
        )
        self._valid_stance_durations_s: deque[float] = deque(
            maxlen=history_window
        )
        self._valid_swing_durations_s: deque[float] = deque(
            maxlen=history_window
        )
        self.valid_hs_count = 0
        self.valid_to_count = 0
        self.valid_cycle_count = 0
        self.invalid_event_count = 0
        self.invalid_event_this_step = 0.0
        self.invalid_event_type = ""
        self.cycle_completed_this_step = 0.0
        self.timeout_exceeded = 0.0
        self.timeout_side = 0.0
        self.stance_elapsed_s = 0.0
        self.swing_elapsed_s = 0.0
        self.cycle_progress_credit = 0.0
        self.pending_cycle_credit = 0.0
        self.phase_event_progress_score = 0.0
        self.phase_cycle_complete_bonus = 0.0
        self.phase_clawback_penalty = 0.0
        self.phase_failure_extra_penalty = 0.0
        self.phase_cycle_failed_this_step = 0.0
        self.landing_window_active = 0.0
        self.landing_window_contact_score = 0.0
        self.last_update_time_s: float | None = None
        self.stance_contact_time_s = 0.0
        self.stance_load_integral_bw_s = 0.0
        self.stance_contact_fraction = 0.0
        self.stance_mean_load_bw = 0.0
        self.cycle_knee_min_rad: float | None = None
        self.cycle_knee_max_rad: float | None = None
        self.cycle_ankle_min_rad: float | None = None
        self.cycle_ankle_max_rad: float | None = None
        self.cycle_knee_excursion_rad = 0.0
        self.cycle_ankle_excursion_rad = 0.0
        self.cycle_rejected_this_step = 0.0
        self.cycle_reject_reason = ""
        # Accepted-transition journal for downstream diagnostics/reward logic.
        # This is deliberately NOT part of ``observation()``: exposing the
        # exact event timestamp must not change the actor contract.
        self.accepted_transitions_this_step: list[dict[str, Any]] = []
        # The two sensor contacts are input guards of this FSM, not a second
        # gait-cycle state machine.  They are deliberately absent from
        # ``observation()`` so existing actors retain their exact schema.
        self._sensor_contact = {"heel": False, "toe": False}
        self._sensor_pending_target: dict[str, bool | None] = {
            "heel": None,
            "toe": None,
        }
        self._sensor_pending_since_s: dict[str, float | None] = {
            "heel": None,
            "toe": None,
        }
        self._sensor_last_on_time_s: dict[str, float | None] = {
            "heel": None,
            "toe": None,
        }
        self._sensor_last_off_time_s: dict[str, float | None] = {
            "heel": None,
            "toe": None,
        }
        self._sensor_clear_since_s: float | None = None
        self._sensor_hs_armed = False
        self._sensor_forefoot_first = False
        self._sensor_bootstrap_reported = False
        self._sensor_startup_heel_only_since_s: float | None = None
        self._partial_stance_start_time_s: float | None = None
        # Stable contact edges are state-independent detector diagnostics.  In
        # shadow mode the gait state is intentionally still driven by legacy
        # events, so these raw edges (or an offline replay from the loads) are
        # the only valid counterfactual input for the prospective two-sensor
        # detector.  They are diagnostics only and never enter observation().
        self.sensor_edges_this_step: list[dict[str, Any]] = []
        self.sensor_events_this_step: list[dict[str, Any]] = []
        self._sensor_batch_last_time_s: float | None = None
        # Independent cursor for the V25/V20 adapter. It is deliberately not
        # reused as continuous-evidence time: the actor-facing FSM keeps its
        # historical first-policy-step accumulation semantics.
        self._binary_active_last_boundary_time_s: float | None = None

    def reset_from_binary_baseline(
        self,
        *,
        time_s: float,
        in_contact: bool,
    ) -> dict[str, Any]:
        """Reset the actor-facing FSM from one non-event V20 baseline.

        ``AIR`` keeps the normal ``WAIT_HS`` state.  Any contact word enters an
        uncredited partial stance without inventing a heel strike.  The method
        is intentionally available only to the explicit V25/V20 active path;
        legacy and historical two-sensor reset semantics remain untouched.
        """

        if self.config.event_source not in _BINARY_ACTIVE_CONTRACTS:
            raise ValueError(
                "reset_from_binary_baseline requires event_source="
                "'binary_active' or 'binary_active_v26'."
            )
        if isinstance(time_s, (bool, np.bool_)):
            raise TypeError("Binary baseline time_s must be a finite number.")
        try:
            baseline_time = float(time_s)
        except (TypeError, ValueError) as exc:
            raise TypeError(
                "Binary baseline time_s must be a finite number."
            ) from exc
        if not np.isfinite(baseline_time):
            raise ValueError("Binary baseline time_s must be finite.")
        if type(in_contact) is not bool:
            raise TypeError("Binary baseline in_contact must be a native bool.")

        self.reset()
        self._binary_active_last_boundary_time_s = baseline_time
        if in_contact:
            self._bootstrap_partial_stance(baseline_time)
        return self.payload()

    @property
    def binary_active_last_boundary_time_s(self) -> float | None:
        """Return the private adapter cursor without changing actor payloads."""

        return self._binary_active_last_boundary_time_s

    def update_from_binary_events(
        self,
        *,
        time_s: float,
        previous_time_s: float,
        events: Sequence[Mapping[str, Any]],
        normal_force_bw: float,
        in_contact: bool,
        prosthetic_knee_angle_rad: float | None = None,
        prosthetic_ankle_angle_rad: float | None = None,
    ) -> dict[str, Any]:
        """Consume one already-debounced V20 event batch without re-detection."""

        binary_contract = _BINARY_ACTIVE_CONTRACTS.get(
            self.config.event_source
        )
        if binary_contract is None:
            raise ValueError(
                "update_from_binary_events requires event_source="
                "'binary_active' or 'binary_active_v26'."
            )
        if isinstance(events, (str, bytes, bytearray)) or not isinstance(
            events,
            Sequence,
        ):
            raise TypeError("Adapted binary events must be an ordered sequence.")

        if isinstance(time_s, (bool, np.bool_)):
            raise TypeError("Binary active policy boundary must be numeric.")
        boundary = float(time_s)
        if not np.isfinite(boundary):
            raise ValueError("Binary active policy boundary must be finite.")
        if isinstance(previous_time_s, (bool, np.bool_)):
            raise TypeError("Binary active previous boundary must be numeric.")
        previous = float(previous_time_s)
        if not np.isfinite(previous):
            raise ValueError("Binary active previous boundary must be finite.")
        cursor = self._binary_active_last_boundary_time_s
        if cursor is None or abs(float(cursor) - previous) > 1e-9:
            raise ValueError(
                "Binary active actor-FSM boundary is discontinuous."
            )
        if boundary <= previous:
            raise ValueError(
                "Binary active policy boundary must advance monotonically."
            )
        if isinstance(normal_force_bw, (bool, np.bool_)):
            raise TypeError("Binary active normal_force_bw must be numeric.")
        normal_force_f = float(normal_force_bw)
        if not np.isfinite(normal_force_f) or normal_force_f < 0.0:
            raise ValueError(
                "Binary active normal_force_bw must be finite and non-negative."
            )
        if type(in_contact) is not bool:
            raise TypeError("Binary active in_contact must be a native bool.")
        for field_name, value in (
            ("prosthetic_knee_angle_rad", prosthetic_knee_angle_rad),
            ("prosthetic_ankle_angle_rad", prosthetic_ankle_angle_rad),
        ):
            if value is None:
                continue
            if isinstance(value, (bool, np.bool_)):
                raise TypeError(f"Binary active {field_name} must be numeric.")
            try:
                finite_value = float(value)
            except (TypeError, ValueError) as exc:
                raise TypeError(
                    f"Binary active {field_name} must be numeric."
                ) from exc
            if not np.isfinite(finite_value):
                raise ValueError(
                    f"Binary active {field_name} must be finite."
                )
        validated: list[dict[str, Any]] = []
        previous_event_time: float | None = None
        for index, raw_event in enumerate(events):
            if not isinstance(raw_event, Mapping):
                raise TypeError(
                    f"Adapted binary event {index} must be a mapping."
                )
            event = dict(raw_event)
            if str(event.get("side", "")).strip().lower() != "left":
                raise ValueError("Adapted binary events must be left-sided.")
            name = str(event.get("event", "")).strip().lower()
            if name not in {"heel_strike", "toe_off"}:
                raise ValueError(
                    "Adapted binary events must be heel_strike or toe_off."
                )
            if event.get("source") != binary_contract["adapter_source"]:
                raise ValueError("Adapted binary event source mismatch.")
            if (
                event.get("event_contract_id")
                != binary_contract["event_contract_id"]
            ):
                raise ValueError("Adapted binary event contract mismatch.")
            required_times = (
                "event_time_s",
                "confirmed_time_s",
                "delivered_time_s",
            )
            if any(field not in event for field in required_times):
                raise ValueError(
                    "Adapted binary events require explicit event, confirmed, "
                    "and delivered timestamps."
                )
            if any(
                isinstance(event[field], (bool, np.bool_))
                for field in required_times
            ):
                raise TypeError(
                    "Adapted binary event timestamps must be finite numbers."
                )
            try:
                event_time = float(event["event_time_s"])
            except (TypeError, ValueError) as exc:
                raise TypeError(
                    "Adapted binary event timestamps must be finite numbers."
                ) from exc
            confirmed, delivered = self._event_confirmation_delivery(
                event,
                event_time,
            )
            if (
                abs(
                    (confirmed - event_time)
                    - BINARY_ACTIVE_DEBOUNCE_S
                )
                > 1e-9
            ):
                raise ValueError(
                    "Adapted binary confirmation latency must equal 5 ms."
                )
            if not (
                confirmed > previous + 1e-9
                and confirmed <= boundary + 1e-9
            ):
                raise ValueError(
                    "Adapted binary confirmation is outside the current "
                    "open-left policy interval."
                )
            if abs(delivered - boundary) > 1e-9:
                raise ValueError(
                    "Adapted binary event must be delivered at the current "
                    "policy boundary."
                )
            if (
                delivered - confirmed < -1e-9
                or delivered - confirmed
                > BINARY_ACTIVE_MAX_DELIVERY_DELAY_S + 1e-9
            ):
                raise ValueError(
                    "Adapted binary delivery latency exceeds 10 ms."
                )
            if previous_event_time is not None and (
                event_time <= previous_event_time + 1e-12
            ):
                raise ValueError(
                    "Adapted binary event timestamps must be strictly ordered."
                )
            event["event"] = name
            event["time"] = event_time
            event["confirmed_time"] = confirmed
            event["event_time_s"] = event_time
            event["confirmed_time_s"] = confirmed
            event["delivered_time_s"] = delivered
            validated.append(event)
            previous_event_time = event_time

        self._sensor_batch_last_time_s = None
        self._reset_policy_step_transients()
        self._prepare_policy_continuous_inputs(
            time_s=boundary,
            normal_force_bw=normal_force_f,
            in_contact=bool(in_contact),
            prosthetic_knee_angle_rad=prosthetic_knee_angle_rad,
            prosthetic_ankle_angle_rad=prosthetic_ankle_angle_rad,
        )
        self._process_events(validated, boundary)
        result = self._finish_policy_step(
            time_s=boundary,
            normal_force_bw=normal_force_f,
            in_contact=bool(in_contact),
        )
        self._binary_active_last_boundary_time_s = boundary
        return result

    @property
    def expected_next_event(self) -> str:
        if self.state_id in {WAIT_HS, SWING_AFTER_TO}:
            return "heel_strike"
        if self.state_id == STANCE_AFTER_HS:
            return "toe_off"
        return "none"

    def _reset_policy_step_transients(self) -> None:
        """Reset pulse/journal fields once per actor-visible policy step."""
        self.invalid_event_this_step = 0.0
        self.invalid_event_type = ""
        self.cycle_completed_this_step = 0.0
        self.timeout_exceeded = 0.0
        self.timeout_side = 0.0
        self.phase_event_progress_score = 0.0
        self.phase_cycle_complete_bonus = 0.0
        self.phase_clawback_penalty = 0.0
        self.phase_failure_extra_penalty = 0.0
        self.phase_cycle_failed_this_step = 0.0
        self.cycle_rejected_this_step = 0.0
        self.cycle_reject_reason = ""
        self.accepted_transitions_this_step = []
        self.sensor_edges_this_step = []
        self.sensor_events_this_step = []

    def _prepare_policy_continuous_inputs(
        self,
        *,
        time_s: float,
        normal_force_bw: float,
        in_contact: bool,
        prosthetic_knee_angle_rad: float | None,
        prosthetic_ankle_angle_rad: float | None,
    ) -> None:
        # Primary load/contact and kinematics intentionally remain at policy
        # cadence.  Only heel/toe debounce runs on the 1 ms sample stream.
        self._accumulate_stance_evidence(
            time_s,
            normal_force_bw=normal_force_bw,
            in_contact=bool(in_contact),
        )
        self._record_cycle_kinematics(
            prosthetic_knee_angle_rad,
            prosthetic_ankle_angle_rad,
        )
        self._current_knee_angle_rad = prosthetic_knee_angle_rad
        self._current_ankle_angle_rad = prosthetic_ankle_angle_rad

    def _finish_policy_step(
        self,
        *,
        time_s: float,
        normal_force_bw: float,
        in_contact: bool,
    ) -> dict[str, Any]:
        self._refresh_time_terms(time_s)
        self._refresh_stance_diagnostics()
        self._refresh_cycle_excursions()
        self._refresh_scores(normal_force_bw, bool(in_contact))
        self.last_update_time_s = time_s
        return self.payload()

    def update(
        self,
        *,
        time_s: float,
        events: Sequence[Mapping[str, Any]] = (),
        normal_force_bw: float,
        in_contact: bool,
        prosthetic_knee_angle_rad: float | None = None,
        prosthetic_ankle_angle_rad: float | None = None,
        heel_normal_force_n: float | None = None,
        toe_normal_force_n: float | None = None,
    ) -> dict[str, Any]:
        time_f = float(time_s)
        normal_force_f = float(normal_force_bw)
        event_source = self.config.event_source
        if event_source in _BINARY_ACTIVE_CONTRACTS:
            raise ValueError(
                f"{event_source} must use update_from_binary_events(); "
                "generic or legacy event injection is forbidden."
            )
        sensor_forces: tuple[float, float] | None = None
        if event_source in {"shadow", "two_sensor"}:
            sensor_forces = (
                self._validated_sensor_force(
                    heel_normal_force_n,
                    "heel_normal_force_n",
                ),
                self._validated_sensor_force(
                    toe_normal_force_n,
                    "toe_normal_force_n",
                ),
            )
        if event_source == "two_sensor" and self._has_left_gait_event(events):
            raise ValueError(
                "two_sensor mode does not accept legacy left HS/TO events."
            )
        self._sensor_batch_last_time_s = None
        self._reset_policy_step_transients()
        self._prepare_policy_continuous_inputs(
            time_s=time_f,
            normal_force_bw=normal_force_f,
            in_contact=bool(in_contact),
            prosthetic_knee_angle_rad=prosthetic_knee_angle_rad,
            prosthetic_ankle_angle_rad=prosthetic_ankle_angle_rad,
        )

        if sensor_forces is not None:
            sensor_events = self._update_two_sensor_guards(
                time_f,
                heel_normal_force_n=sensor_forces[0],
                toe_normal_force_n=sensor_forces[1],
                allow_partial_stance_bootstrap=(event_source == "two_sensor"),
                delivered_time_s=time_f,
            )
            self.sensor_events_this_step = [dict(event) for event in sensor_events]

        active_events: Sequence[Mapping[str, Any]]
        if event_source == "two_sensor":
            active_events = self.sensor_events_this_step
        else:
            active_events = events
        self._process_events(active_events, time_f)
        return self._finish_policy_step(
            time_s=time_f,
            normal_force_bw=normal_force_f,
            in_contact=bool(in_contact),
        )

    def update_policy_step(
        self,
        *,
        time_s: float,
        sensor_samples: Sequence[Mapping[str, Any]],
        previous_time_s: float | None = None,
        events: Sequence[Mapping[str, Any]] = (),
        normal_force_bw: float,
        in_contact: bool,
        prosthetic_knee_angle_rad: float | None = None,
        prosthetic_ankle_angle_rad: float | None = None,
    ) -> dict[str, Any]:
        """Consume one complete 1 ms detector batch at a 10 ms boundary.

        The sample interval is open on ``previous_time_s`` and closed on
        ``time_s``.  Validation completes before any FSM state is mutated.
        Pulses and journals are reset once, then all detector samples are
        processed causally in timestamp order and delivered together.
        """
        event_source = self.config.event_source
        if event_source not in {"shadow", "two_sensor"}:
            raise ValueError(
                "update_policy_step requires shadow or two_sensor event_source."
            )
        if event_source == "two_sensor" and self._has_left_gait_event(events):
            raise ValueError(
                "two_sensor mode does not accept legacy left HS/TO events."
            )

        time_f = float(time_s)
        normal_force_f = float(normal_force_bw)
        validated_samples, segment_start = self._validated_sensor_batch(
            sensor_samples,
            time_s=time_f,
            previous_time_s=previous_time_s,
        )

        self._reset_policy_step_transients()
        self._prepare_policy_continuous_inputs(
            time_s=time_f,
            normal_force_bw=normal_force_f,
            in_contact=bool(in_contact),
            prosthetic_knee_angle_rad=prosthetic_knee_angle_rad,
            prosthetic_ankle_angle_rad=prosthetic_ankle_angle_rad,
        )

        for sample_time, heel_force, toe_force in validated_samples:
            sensor_events = self._update_two_sensor_guards(
                sample_time,
                heel_normal_force_n=heel_force,
                toe_normal_force_n=toe_force,
                allow_partial_stance_bootstrap=(event_source == "two_sensor"),
                delivered_time_s=time_f,
            )
            self.sensor_events_this_step.extend(
                dict(event) for event in sensor_events
            )
            if event_source == "two_sensor":
                self._process_events(sensor_events, sample_time)
                # In the active detector mode, timeouts retain sequential 1 ms
                # semantics. Primary continuous evidence is still accumulated
                # only once above at policy cadence.
                self._refresh_time_terms(sample_time)

        if event_source == "shadow":
            # Shadow must remain behaviourally identical to the scalar legacy
            # path: apply boundary-visible legacy events before the one policy-
            # cadence timeout refresh performed by _finish_policy_step().  The
            # 1 ms stream above updates detector diagnostics only.
            self._process_events(events, time_f)

        result = self._finish_policy_step(
            time_s=time_f,
            normal_force_bw=normal_force_f,
            in_contact=bool(in_contact),
        )
        self._sensor_batch_last_time_s = time_f
        result["sensor_batch_previous_time_s"] = float(segment_start)
        result["sensor_batch_sample_count"] = float(len(validated_samples))
        return result

    def _validated_sensor_batch(
        self,
        samples: Sequence[Mapping[str, Any]],
        *,
        time_s: float,
        previous_time_s: float | None,
    ) -> tuple[list[tuple[float, float, float]], float]:
        if not np.isfinite(time_s):
            raise ValueError("time_s must be finite for detector batch updates.")
        if not isinstance(samples, Sequence) or isinstance(
            samples,
            (str, bytes, bytearray),
        ):
            raise ValueError("Detector samples must be an ordered sequence.")
        internal_previous = self._sensor_batch_last_time_s
        if previous_time_s is None:
            if internal_previous is None:
                raise ValueError(
                    "previous_time_s is required for the first detector batch."
                )
            previous = float(internal_previous)
        else:
            previous = float(previous_time_s)
            if not np.isfinite(previous):
                raise ValueError("previous_time_s must be finite.")
            if internal_previous is not None:
                tolerance = max(
                    1e-12,
                    float(self.config.detector_sample_dt_s) * 1e-7,
                )
                if abs(previous - float(internal_previous)) > tolerance:
                    raise ValueError(
                        "Detector batch boundary is discontinuous with the "
                        "previous policy step."
                    )

        dt = float(self.config.detector_sample_dt_s)
        tolerance = max(1e-12, dt * 1e-7)
        duration = time_s - previous
        if duration <= tolerance:
            raise ValueError("Detector batch interval must be positive.")
        count_float = duration / dt
        expected_count = int(round(count_float))
        if expected_count <= 0 or abs(count_float - expected_count) > 1e-7:
            raise ValueError(
                "Detector batch interval is not aligned to detector_sample_dt_s."
            )
        if len(samples) != expected_count:
            raise ValueError(
                "Detector batch is incomplete: expected "
                f"{expected_count} samples, got {len(samples)}."
            )

        validated: list[tuple[float, float, float]] = []
        for index, sample in enumerate(samples, start=1):
            if not isinstance(sample, Mapping):
                raise ValueError("Every detector sample must be a mapping.")
            try:
                sample_time = float(sample["time_s"])
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(
                    "Every detector sample requires a finite time_s."
                ) from exc
            expected_time = previous + index * dt
            if (
                not np.isfinite(sample_time)
                or abs(sample_time - expected_time) > tolerance
            ):
                relation = (
                    "duplicate/non-monotonic"
                    if validated and sample_time <= validated[-1][0] + tolerance
                    else "missing/off-grid"
                )
                raise ValueError(
                    f"Detector sample {index} is {relation}: expected "
                    f"{expected_time:.12g}, got {sample_time:.12g}."
                )
            heel = self._validated_sensor_force(
                sample.get("left_heel_normal_n"),
                "left_heel_normal_n",
            )
            toe = self._validated_sensor_force(
                sample.get("left_toe_normal_n"),
                "left_toe_normal_n",
            )
            validated.append((expected_time, heel, toe))
        return validated, previous

    @staticmethod
    def _has_left_gait_event(events: Sequence[Mapping[str, Any]]) -> bool:
        return any(
            isinstance(event, Mapping)
            and str(event.get("side", "")).lower() == "left"
            and str(event.get("event", "")).lower() in {"heel_strike", "toe_off"}
            for event in events
        )

    @staticmethod
    def _validated_sensor_force(value: float | None, name: str) -> float:
        if value is None:
            raise ValueError(f"{name} is required in shadow and two_sensor modes.")
        try:
            result = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must be a finite non-negative number.") from exc
        if not np.isfinite(result) or result < 0.0:
            raise ValueError(f"{name} must be a finite non-negative number.")
        return result

    def _process_events(
        self,
        events: Sequence[Mapping[str, Any]],
        fallback_time_s: float,
    ) -> None:
        for event in sorted(
            (event for event in events if isinstance(event, Mapping)),
            key=lambda item: _event_time_or_fallback(item, fallback_time_s),
        ):
            if str(event.get("side", "")).lower() != "left":
                continue
            name = str(event.get("event", "")).lower()
            event_time = _event_time_or_fallback(event, fallback_time_s)
            if name == "heel_strike":
                self._handle_heel_strike(event_time, event)
            elif name == "toe_off":
                self._handle_toe_off(event_time, event)

    def _update_sensor_contact(
        self,
        *,
        sensor: str,
        force_n: float,
        time_s: float,
    ) -> tuple[bool | None, float | None]:
        """Debounce one regional contact and return its stable edge.

        The returned timestamp is the causal candidate onset.  The edge is only
        returned after ``sensor_dwell_s`` has elapsed, so callers can retain the
        physical onset while exposing the pulse no earlier than confirmation.
        """
        active = bool(self._sensor_contact[sensor])
        requested: bool | None = None
        if active and force_n <= float(self.config.sensor_off_threshold_n):
            requested = False
        elif not active and force_n >= float(self.config.sensor_on_threshold_n):
            requested = True

        if requested is None:
            self._sensor_pending_target[sensor] = None
            self._sensor_pending_since_s[sensor] = None
            return None, None

        if self._sensor_pending_target[sensor] != requested:
            self._sensor_pending_target[sensor] = requested
            self._sensor_pending_since_s[sensor] = float(time_s)

        onset = self._sensor_pending_since_s[sensor]
        if onset is None:
            return None, None
        if (
            float(time_s) - float(onset) + 1e-12
            < float(self.config.sensor_dwell_s)
        ):
            return None, None

        self._sensor_contact[sensor] = bool(requested)
        self._sensor_pending_target[sensor] = None
        self._sensor_pending_since_s[sensor] = None
        if requested:
            self._sensor_last_on_time_s[sensor] = float(onset)
        else:
            self._sensor_last_off_time_s[sensor] = float(onset)
        return bool(requested), float(onset)

    def _refresh_sensor_clear_arm(
        self,
        *,
        time_s: float,
        heel_normal_force_n: float,
        toe_normal_force_n: float,
    ) -> None:
        both_raw_clear = (
            heel_normal_force_n <= float(self.config.sensor_off_threshold_n)
            and toe_normal_force_n <= float(self.config.sensor_off_threshold_n)
        )
        if not both_raw_clear:
            self._sensor_clear_since_s = None
            return
        if self._sensor_clear_since_s is None:
            self._sensor_clear_since_s = float(time_s)
        if (
            float(time_s) - float(self._sensor_clear_since_s) + 1e-12
            < float(self.config.sensor_dwell_s)
        ):
            return
        if not self._sensor_contact["heel"] and not self._sensor_contact["toe"]:
            self._sensor_hs_armed = True
            self._sensor_forefoot_first = False

    def _bootstrap_partial_stance(self, contact_time_s: float) -> None:
        """Enter the existing stance state without inventing an HS event.

        This covers an episode reset that occurs after physical initial contact.
        No accepted transition, event credit, valid-HS count, or synthetic
        ``last_valid_hs_time`` is created.  The partial segment can still end at
        the first real two-sensor toe-off.
        """
        self.state_id = STANCE_AFTER_HS
        self._partial_stance_start_time_s = float(contact_time_s)
        self._sensor_hs_armed = False
        self._sensor_forefoot_first = False
        self._sensor_bootstrap_reported = True
        self.cycle_progress_credit = 0.0
        self.pending_cycle_credit = 0.0
        self._reset_cycle_evidence()
        self._record_cycle_kinematics(
            getattr(self, "_current_knee_angle_rad", None),
            getattr(self, "_current_ankle_angle_rad", None),
        )

    def _sensor_event(
        self,
        *,
        event: str,
        event_time_s: float,
        confirmed_time_s: float,
        delivered_time_s: float,
        **extra: Any,
    ) -> dict[str, Any]:
        event_time = float(event_time_s)
        confirmed_time = float(confirmed_time_s)
        delivered_time = float(delivered_time_s)
        if not (
            np.isfinite(event_time)
            and np.isfinite(confirmed_time)
            and np.isfinite(delivered_time)
            and event_time <= confirmed_time + 1e-12
            and confirmed_time <= delivered_time + 1e-12
        ):
            raise ValueError(
                "Detector event timestamps must be finite and causal "
                "(event <= confirmed <= delivered)."
            )
        result: dict[str, Any] = {
            "side": "left",
            "event": str(event),
            # Legacy aliases remain available to existing event consumers.
            "time": event_time,
            "confirmed_time": confirmed_time,
            "event_time_s": event_time,
            "confirmed_time_s": confirmed_time,
            "delivered_time_s": delivered_time,
            "source": "two_sensor",
        }
        result.update(extra)
        return result

    def _update_two_sensor_guards(
        self,
        time_s: float,
        *,
        heel_normal_force_n: float,
        toe_normal_force_n: float,
        allow_partial_stance_bootstrap: bool,
        delivered_time_s: float,
    ) -> list[dict[str, Any]]:
        """Map two debounced regional contacts onto the existing gait states."""
        startup_unresolved = bool(
            self.state_id == WAIT_HS
            and not self._sensor_hs_armed
            and not self._sensor_bootstrap_reported
        )
        raw_startup_heel_only = bool(
            heel_normal_force_n >= float(self.config.sensor_on_threshold_n)
            and toe_normal_force_n <= float(self.config.sensor_off_threshold_n)
        )
        if startup_unresolved and raw_startup_heel_only:
            if self._sensor_startup_heel_only_since_s is None:
                self._sensor_startup_heel_only_since_s = float(time_s)
        else:
            self._sensor_startup_heel_only_since_s = None

        heel_edge, heel_edge_time = self._update_sensor_contact(
            sensor="heel",
            force_n=heel_normal_force_n,
            time_s=time_s,
        )
        toe_edge, toe_edge_time = self._update_sensor_contact(
            sensor="toe",
            force_n=toe_normal_force_n,
            time_s=time_s,
        )
        for sensor, edge, edge_time in (
            ("heel", heel_edge, heel_edge_time),
            ("toe", toe_edge, toe_edge_time),
        ):
            if edge is None or edge_time is None:
                continue
            self.sensor_edges_this_step.append(
                {
                    "sensor": sensor,
                    "edge": "contact_on" if edge else "contact_off",
                    "event_time_s": float(edge_time),
                    "confirmed_time_s": float(time_s),
                    "delivered_time_s": float(delivered_time_s),
                    "source": "two_sensor_guard",
                }
            )
        self._refresh_sensor_clear_arm(
            time_s=time_s,
            heel_normal_force_n=heel_normal_force_n,
            toe_normal_force_n=toe_normal_force_n,
        )

        events: list[dict[str, Any]] = []
        expecting_hs = self.state_id in {WAIT_HS, SWING_AFTER_TO}
        heel_rose = heel_edge is True and heel_edge_time is not None
        toe_rose = toe_edge is True and toe_edge_time is not None
        startup_heel_only_onset = self._sensor_startup_heel_only_since_s
        startup_heel_only_ready = bool(
            startup_unresolved
            and raw_startup_heel_only
            and startup_heel_only_onset is not None
            and float(time_s) - float(startup_heel_only_onset) + 1e-12
            >= float(self.config.sensor_dwell_s)
        )

        # Resolve a stable contact already present at detector startup.  A
        # heel-only pattern sustained for the normal debounce margin is the
        # episode-boundary HS requested by the detector contract.  Toe-only or
        # heel+toe contact remains a partial-stance bootstrap: with only two
        # contact sensors those patterns describe an already established
        # stance more plausibly than a new heel strike.
        if (
            self.state_id == WAIT_HS
            and not self._sensor_hs_armed
            and (self._sensor_contact["heel"] or self._sensor_contact["toe"])
            and not self._sensor_bootstrap_reported
        ):
            if (
                startup_heel_only_ready
                and heel_rose
                and self._sensor_contact["heel"]
                and not self._sensor_contact["toe"]
            ):
                event_time = float(startup_heel_only_onset)
                events.append(
                    self._sensor_event(
                        event="heel_strike",
                        event_time_s=event_time,
                        confirmed_time_s=time_s,
                        delivered_time_s=delivered_time_s,
                        confirmation_latency_s=float(time_s) - event_time,
                        forefoot_contact_preceded_heel=False,
                        startup_contact=True,
                        startup_pattern="heel_on_toe_off",
                    )
                )
                self._sensor_bootstrap_reported = True
                self._sensor_hs_armed = False
                self._sensor_startup_heel_only_since_s = None
            else:
                on_times = [
                    value
                    for value in self._sensor_last_on_time_s.values()
                    if value is not None
                ]
                bootstrap_time = min(on_times) if on_times else float(time_s)
                if allow_partial_stance_bootstrap:
                    self._bootstrap_partial_stance(bootstrap_time)
                else:
                    self._sensor_bootstrap_reported = True
                    events.append(
                        self._sensor_event(
                            event="partial_stance_bootstrap",
                            event_time_s=bootstrap_time,
                            confirmed_time_s=time_s,
                            delivered_time_s=delivered_time_s,
                        )
                    )
            expecting_hs = False

        if expecting_hs and self._sensor_hs_armed:
            toe_precedes_heel = bool(
                toe_rose
                and (
                    not heel_rose
                    or float(toe_edge_time) + 1e-12 < float(heel_edge_time)
                )
            )
            if toe_precedes_heel:
                self._sensor_forefoot_first = True
                events.append(
                    self._sensor_event(
                        event="forefoot_first",
                        event_time_s=float(toe_edge_time),
                        confirmed_time_s=time_s,
                        delivered_time_s=delivered_time_s,
                    )
                )

            if heel_rose:
                toe_was_already_active = bool(
                    self._sensor_contact["toe"]
                    and not toe_rose
                )
                if toe_was_already_active:
                    self._sensor_forefoot_first = True
                # Toe-only contact is never promoted to HS.  If the heel later
                # establishes a stable contact, however, that physical heel
                # onset is the HS; forefoot-first remains a diagnostic rather
                # than a hidden gate that can suppress the real heel sensor.
                events.append(
                    self._sensor_event(
                        event="heel_strike",
                        event_time_s=float(heel_edge_time),
                        confirmed_time_s=time_s,
                        delivered_time_s=delivered_time_s,
                        confirmation_latency_s=(
                            float(time_s) - float(heel_edge_time)
                        ),
                        forefoot_contact_preceded_heel=bool(
                            self._sensor_forefoot_first
                        ),
                    )
                )
                self._sensor_hs_armed = False

        heel_fell = heel_edge is False and heel_edge_time is not None
        toe_fell = toe_edge is False and toe_edge_time is not None
        both_stably_off = (
            not self._sensor_contact["heel"] and not self._sensor_contact["toe"]
        )
        if both_stably_off and (heel_fell or toe_fell):
            off_times = [
                value
                for value in self._sensor_last_off_time_s.values()
                if value is not None
            ]
            clear_time = max(off_times) if off_times else float(time_s)
            if self.state_id == STANCE_AFTER_HS:
                stance_anchor = self._stance_anchor_time()
                contact_duration = (
                    None
                    if stance_anchor is None
                    else max(0.0, float(clear_time) - float(stance_anchor))
                )
                extra: dict[str, Any] = {}
                if contact_duration is not None:
                    extra["contact_duration_s"] = float(contact_duration)
                events.append(
                    self._sensor_event(
                        event="toe_off",
                        event_time_s=clear_time,
                        confirmed_time_s=time_s,
                        delivered_time_s=delivered_time_s,
                        **extra,
                    )
                )
            self._sensor_hs_armed = True
            self._sensor_forefoot_first = False

        return events

    def _stance_anchor_time(self) -> float | None:
        if self.last_valid_hs_time is not None:
            return float(self.last_valid_hs_time)
        if self._partial_stance_start_time_s is not None:
            return float(self._partial_stance_start_time_s)
        return None

    def _handle_heel_strike(
        self,
        event_time: float,
        event: Mapping[str, Any] | None = None,
    ) -> None:
        cfg = self.config
        confirmed_time, delivered_time = self._event_confirmation_delivery(
            event,
            event_time,
        )
        startup_contact = bool(
            isinstance(event, Mapping) and event.get("startup_contact", False)
        )
        if self.state_id == WAIT_HS:
            self._accept_hs(
                event_time,
                progress=0.25,
                startup_contact=startup_contact,
                confirmed_time_s=confirmed_time,
                delivered_time_s=delivered_time,
            )
            return
        if self.state_id == STANCE_AFTER_HS:
            self._mark_invalid("double_hs_before_to")
            return
        if self.state_id == SWING_AFTER_TO:
            swing_elapsed = (
                event_time - float(self.last_valid_to_time)
                if self.last_valid_to_time is not None
                else 0.0
            )
            if swing_elapsed < float(cfg.min_swing_duration_s):
                self._mark_invalid("hs_too_early_after_to")
                return
            previous_hs = self.last_valid_hs_time
            previous_to = self.last_valid_to_time
            # The first HS after a mid-stance episode bootstrap starts the
            # first complete cycle; it cannot close an HS-to-HS cycle because
            # no preceding HS was observed.
            if previous_hs is None:
                self._accept_hs(
                    event_time,
                    progress=0.25,
                    confirmed_time_s=confirmed_time,
                    delivered_time_s=delivered_time,
                )
                return
            valid, reason = self._cycle_valid_for_completion(event_time)
            if not valid:
                self._reject_cycle(
                    reason,
                    accept_new_hs=True,
                    event_time=event_time,
                    confirmed_time_s=confirmed_time,
                    delivered_time_s=delivered_time,
                )
                return
            if previous_hs is not None and event_time > float(previous_hs):
                period = event_time - float(previous_hs)
                self.previous_period_s = float(self.last_period_s)
                self.last_period_s = float(period)
                if previous_to is not None and float(previous_hs) < float(previous_to) < event_time:
                    stance_duration = float(previous_to) - float(previous_hs)
                    swing_duration = event_time - float(previous_to)
                    self.last_stance_fraction = float(
                        stance_duration / period
                    )
                    self._valid_stance_durations_s.append(stance_duration)
                    self._valid_swing_durations_s.append(swing_duration)
            self.valid_cycle_count += 1
            self.cycle_completed_this_step = 1.0
            self._add_event_credit(cfg.cycle_complete_bonus)
            self.phase_cycle_complete_bonus = float(cfg.cycle_complete_bonus)
            self._accept_hs(
                event_time,
                progress=1.0,
                cycle_valid=True,
                confirmed_time_s=confirmed_time,
                delivered_time_s=delivered_time,
            )
            return
        if self.state_id == TIMEOUT:
            return

    def _handle_toe_off(self, event_time: float, event: Mapping[str, Any]) -> None:
        cfg = self.config
        confirmed_time, delivered_time = self._event_confirmation_delivery(
            event,
            event_time,
        )
        if self.state_id == WAIT_HS:
            self._mark_invalid("to_before_hs")
            return
        if self.state_id == STANCE_AFTER_HS:
            stance_anchor = self._stance_anchor_time()
            stance_elapsed = (
                event_time - float(stance_anchor)
                if stance_anchor is not None
                else 0.0
            )
            partial_stance = (
                self.last_valid_hs_time is None
                and self._partial_stance_start_time_s is not None
            )
            # A reset can occur near the end of a stance that began before the
            # episode.  Its observed duration/evidence is necessarily partial,
            # so the first real both-sensors-off transition must not be rejected
            # by full-stance gates and then lost forever.  It remains uncredited
            # and cannot complete a cycle.
            if not partial_stance:
                if stance_elapsed < float(cfg.min_stance_duration_s):
                    self._mark_invalid("to_too_early_after_hs")
                    return
                valid, reason = self._stance_valid_for_toe_off(
                    stance_elapsed,
                    event,
                )
                if not valid:
                    self._mark_invalid(reason)
                    return
            self.last_valid_to_time = float(event_time)
            self.valid_to_count += 1
            self._record_accepted_transition(
                event="toe_off",
                event_time_s=event_time,
                confirmed_time_s=confirmed_time,
                delivered_time_s=delivered_time,
                from_state_id=STANCE_AFTER_HS,
                to_state_id=SWING_AFTER_TO,
                closed_segment_type="stance",
                segment_start_time_s=stance_anchor,
                segment_valid=not partial_stance,
                opens_segment_type="swing",
                cycle_valid=None,
                cycle_reject_reason="",
            )
            self.state_id = SWING_AFTER_TO
            self.cycle_progress_credit = 0.50
            self._partial_stance_start_time_s = None
            # A real TO can terminate a partial bootstrap stance, but that
            # incomplete segment receives no event credit or pending credit.
            if not partial_stance:
                self.pending_cycle_credit += float(cfg.toe_off_event_credit)
                self._add_event_credit(cfg.toe_off_event_credit)
            return
        if self.state_id == SWING_AFTER_TO:
            self._mark_invalid("double_to_before_hs")
            return

    def _accept_hs(
        self,
        event_time: float,
        *,
        progress: float,
        closed_segment_valid: bool = True,
        cycle_valid: bool | None = None,
        cycle_reject_reason: str = "",
        startup_contact: bool = False,
        confirmed_time_s: float | None = None,
        delivered_time_s: float | None = None,
    ) -> None:
        cfg = self.config
        from_state_id = int(self.state_id)
        closes_swing = from_state_id == SWING_AFTER_TO
        self._record_accepted_transition(
            event="heel_strike",
            event_time_s=event_time,
            confirmed_time_s=confirmed_time_s,
            delivered_time_s=delivered_time_s,
            from_state_id=from_state_id,
            to_state_id=STANCE_AFTER_HS,
            closed_segment_type="swing" if closes_swing else "",
            segment_start_time_s=(
                self.last_valid_to_time if closes_swing else None
            ),
            segment_valid=bool(closed_segment_valid),
            opens_segment_type="stance",
            cycle_valid=cycle_valid,
            cycle_reject_reason=cycle_reject_reason,
            startup_contact=startup_contact,
        )
        self.last_valid_hs_time = float(event_time)
        self._partial_stance_start_time_s = None
        self.valid_hs_count += 1
        self.state_id = STANCE_AFTER_HS
        self.cycle_progress_credit = float(progress)
        self.pending_cycle_credit = float(cfg.hs_event_credit)
        self._reset_cycle_evidence()
        self._record_cycle_kinematics(
            getattr(self, "_current_knee_angle_rad", None),
            getattr(self, "_current_ankle_angle_rad", None),
        )
        self._add_event_credit(cfg.hs_event_credit)

    def _accumulate_stance_evidence(
        self,
        time_s: float,
        *,
        normal_force_bw: float,
        in_contact: bool,
    ) -> None:
        if self.last_update_time_s is None:
            self.last_update_time_s = float(time_s)
            return
        dt = max(0.0, float(time_s) - float(self.last_update_time_s))
        if dt <= 0.0 or self.state_id != STANCE_AFTER_HS:
            return
        if bool(in_contact):
            self.stance_contact_time_s += dt
        self.stance_load_integral_bw_s += max(0.0, float(normal_force_bw)) * dt

    def _reset_cycle_evidence(self) -> None:
        self.stance_contact_time_s = 0.0
        self.stance_load_integral_bw_s = 0.0
        self.stance_contact_fraction = 0.0
        self.stance_mean_load_bw = 0.0
        self.cycle_knee_min_rad = None
        self.cycle_knee_max_rad = None
        self.cycle_ankle_min_rad = None
        self.cycle_ankle_max_rad = None
        self.cycle_knee_excursion_rad = 0.0
        self.cycle_ankle_excursion_rad = 0.0

    def _record_cycle_kinematics(
        self,
        knee_angle_rad: float | None,
        ankle_angle_rad: float | None,
    ) -> None:
        if knee_angle_rad is not None:
            try:
                knee = float(knee_angle_rad)
            except (TypeError, ValueError):
                knee = float("nan")
            if np.isfinite(knee):
                self.cycle_knee_min_rad = (
                    knee if self.cycle_knee_min_rad is None else min(self.cycle_knee_min_rad, knee)
                )
                self.cycle_knee_max_rad = (
                    knee if self.cycle_knee_max_rad is None else max(self.cycle_knee_max_rad, knee)
                )
        if ankle_angle_rad is not None:
            try:
                ankle = float(ankle_angle_rad)
            except (TypeError, ValueError):
                ankle = float("nan")
            if np.isfinite(ankle):
                self.cycle_ankle_min_rad = (
                    ankle if self.cycle_ankle_min_rad is None else min(self.cycle_ankle_min_rad, ankle)
                )
                self.cycle_ankle_max_rad = (
                    ankle if self.cycle_ankle_max_rad is None else max(self.cycle_ankle_max_rad, ankle)
                )

    def _refresh_stance_diagnostics(self) -> None:
        stance_duration = max(0.0, float(self.stance_elapsed_s))
        if self.last_valid_to_time is not None and self.last_valid_hs_time is not None:
            if float(self.last_valid_to_time) >= float(self.last_valid_hs_time):
                stance_duration = max(
                    stance_duration,
                    float(self.last_valid_to_time) - float(self.last_valid_hs_time),
                )
        if stance_duration > 1e-9:
            self.stance_contact_fraction = float(
                np.clip(self.stance_contact_time_s / stance_duration, 0.0, 1.0)
            )
            self.stance_mean_load_bw = float(
                max(0.0, self.stance_load_integral_bw_s) / stance_duration
            )
        else:
            self.stance_contact_fraction = 0.0
            self.stance_mean_load_bw = 0.0

    def _refresh_cycle_excursions(self) -> None:
        if self.cycle_knee_min_rad is not None and self.cycle_knee_max_rad is not None:
            self.cycle_knee_excursion_rad = float(
                max(0.0, self.cycle_knee_max_rad - self.cycle_knee_min_rad)
            )
        else:
            self.cycle_knee_excursion_rad = 0.0
        if self.cycle_ankle_min_rad is not None and self.cycle_ankle_max_rad is not None:
            self.cycle_ankle_excursion_rad = float(
                max(0.0, self.cycle_ankle_max_rad - self.cycle_ankle_min_rad)
            )
        else:
            self.cycle_ankle_excursion_rad = 0.0

    def _stance_valid_for_toe_off(
        self,
        stance_elapsed_s: float,
        event: Mapping[str, Any],
    ) -> tuple[bool, str]:
        cfg = self.config
        min_contact_fraction = max(0.0, float(cfg.min_stance_contact_fraction))
        min_load = max(0.0, float(cfg.min_stance_load_bw_s))
        if min_contact_fraction > 0.0:
            event_contact_duration = event.get("contact_duration_s")
            contact_time = self.stance_contact_time_s
            if event_contact_duration is not None:
                try:
                    contact_time = max(contact_time, float(event_contact_duration))
                except (TypeError, ValueError):
                    pass
            contact_fraction = contact_time / max(1e-9, float(stance_elapsed_s))
            if contact_fraction < min_contact_fraction:
                return False, "stance_contact_too_low"
        if min_load > 0.0 and self.stance_load_integral_bw_s < min_load:
            return False, "stance_load_too_low"
        return True, ""

    def _cycle_valid_for_completion(self, event_time: float) -> tuple[bool, str]:
        cfg = self.config
        self._refresh_cycle_excursions()
        min_knee = max(0.0, float(cfg.min_cycle_knee_excursion_rad))
        if min_knee > 0.0 and self.cycle_knee_excursion_rad < min_knee:
            return False, "cycle_knee_excursion_too_low"
        return True, ""

    def _event_causality_tolerance_s(self) -> float:
        """Return the timestamp tolerance frozen for the active event source."""
        if self.config.event_source in _BINARY_ACTIVE_CONTRACTS:
            return _BINARY_ACTIVE_EVENT_CAUSALITY_TOLERANCE_S
        return _EVENT_CAUSALITY_TOLERANCE_S

    def _event_confirmation_delivery(
        self,
        event: Mapping[str, Any] | None,
        event_time_s: float,
    ) -> tuple[float, float]:
        """Return canonical confirmation/delivery timestamps for an event."""
        event_time = float(event_time_s)
        confirmed_raw: Any = event_time
        delivered_raw: Any = event_time
        if isinstance(event, Mapping):
            confirmed_raw = event.get(
                "confirmed_time_s",
                event.get("confirmed_time", event_time),
            )
            delivered_raw = event.get("delivered_time_s", confirmed_raw)
        try:
            confirmed = float(confirmed_raw)
            delivered = float(delivered_raw)
        except (TypeError, ValueError) as exc:
            raise ValueError("Event timestamps must be numeric and finite.") from exc
        tolerance = self._event_causality_tolerance_s()
        if not (
            np.isfinite(event_time)
            and np.isfinite(confirmed)
            and np.isfinite(delivered)
            and event_time <= confirmed + tolerance
            and confirmed <= delivered + tolerance
        ):
            raise ValueError(
                "Event timestamps must be finite and causal "
                "(event <= confirmed <= delivered)."
            )
        return confirmed, delivered

    def _reject_cycle(
        self,
        reason: str,
        *,
        accept_new_hs: bool,
        event_time: float,
        confirmed_time_s: float | None = None,
        delivered_time_s: float | None = None,
    ) -> None:
        self.cycle_rejected_this_step = 1.0
        self.cycle_reject_reason = str(reason)
        self._mark_invalid(str(reason))
        if accept_new_hs:
            self._accept_hs(
                event_time,
                progress=0.25,
                closed_segment_valid=True,
                cycle_valid=False,
                cycle_reject_reason=reason,
                confirmed_time_s=confirmed_time_s,
                delivered_time_s=delivered_time_s,
            )

    def _record_accepted_transition(
        self,
        *,
        event: str,
        event_time_s: float,
        confirmed_time_s: float | None = None,
        delivered_time_s: float | None = None,
        from_state_id: int,
        to_state_id: int,
        closed_segment_type: str,
        segment_start_time_s: float | None,
        segment_valid: bool,
        opens_segment_type: str,
        cycle_valid: bool | None,
        cycle_reject_reason: str,
        startup_contact: bool = False,
    ) -> None:
        """Append one JSON-safe FSM transition accepted by the state machine.

        Raw detector events that the FSM rejects never appear here.  Consumers
        can therefore anchor a retrospective segment to the exact timestamp
        that caused the accepted state transition, including detector events
        confirmed a few integration steps after their physical timestamp.
        """
        start = (
            float(segment_start_time_s)
            if segment_start_time_s is not None
            else -1.0
        )
        confirmed = (
            float(event_time_s)
            if confirmed_time_s is None
            else float(confirmed_time_s)
        )
        delivered = confirmed if delivered_time_s is None else float(delivered_time_s)
        tolerance = self._event_causality_tolerance_s()
        if not (
            np.isfinite(float(event_time_s))
            and np.isfinite(confirmed)
            and np.isfinite(delivered)
            and float(event_time_s) <= confirmed + tolerance
            and confirmed <= delivered + tolerance
        ):
            raise ValueError(
                "Accepted transition timestamps must be finite and causal."
            )
        transition = {
            "event": str(event),
            "event_time_s": float(event_time_s),
            "confirmed_time_s": confirmed,
            "delivered_time_s": delivered,
            "from_state_id": float(from_state_id),
            "to_state_id": float(to_state_id),
            "closed_segment_type": str(closed_segment_type),
            "segment_start_time_s": start,
            "segment_end_time_s": float(event_time_s),
            "segment_valid": float(bool(segment_valid)),
            "anchor_geometry_valid": float(bool(segment_valid)),
            "opens_segment_type": str(opens_segment_type),
            "cycle_valid": (
                -1.0 if cycle_valid is None else float(bool(cycle_valid))
            ),
            "cycle_reject_reason": str(cycle_reject_reason),
        }
        # Keep legacy transition journals byte-for-byte compatible: the new
        # diagnostic key exists only on the exceptional startup HS.
        if startup_contact:
            transition["startup_contact"] = 1.0
        self.accepted_transitions_this_step.append(transition)

    def _mark_invalid(self, event_type: str) -> None:
        self.invalid_event_this_step = 1.0
        self.invalid_event_count += 1
        self.invalid_event_type = str(event_type)
        self._apply_cycle_failure()

    def _add_event_credit(self, value: float) -> None:
        self.phase_event_progress_score += max(0.0, float(value))

    def _apply_cycle_failure(self) -> None:
        if self.phase_cycle_failed_this_step:
            return
        cfg = self.config
        self.phase_cycle_failed_this_step = 1.0
        self.phase_failure_extra_penalty = max(
            0.0,
            float(cfg.failure_extra_penalty),
        )
        self.phase_clawback_penalty = max(0.0, float(self.pending_cycle_credit))
        self.pending_cycle_credit = 0.0

    def _refresh_time_terms(self, time_s: float) -> None:
        cfg = self.config
        self.stance_elapsed_s = 0.0
        self.swing_elapsed_s = 0.0
        stance_anchor = self._stance_anchor_time()
        if self.state_id == STANCE_AFTER_HS and stance_anchor is not None:
            self.stance_elapsed_s = max(0.0, time_s - float(stance_anchor))
            if (
                float(cfg.stance_hard_timeout_s) > 0.0
                and self.stance_elapsed_s > float(cfg.stance_hard_timeout_s)
            ):
                self._record_accepted_transition(
                    event="timeout",
                    event_time_s=time_s,
                    from_state_id=STANCE_AFTER_HS,
                    to_state_id=TIMEOUT,
                    closed_segment_type="stance",
                    segment_start_time_s=stance_anchor,
                    segment_valid=False,
                    opens_segment_type="",
                    cycle_valid=False,
                    cycle_reject_reason="phase_timeout:stance",
                )
                self.state_id = TIMEOUT
                self.timeout_exceeded = 1.0
                self.timeout_side = 1.0
                self._apply_cycle_failure()
        elif self.state_id == SWING_AFTER_TO and self.last_valid_to_time is not None:
            self.swing_elapsed_s = max(0.0, time_s - float(self.last_valid_to_time))
            if (
                float(cfg.swing_hard_timeout_s) > 0.0
                and self.swing_elapsed_s > float(cfg.swing_hard_timeout_s)
            ):
                self._record_accepted_transition(
                    event="timeout",
                    event_time_s=time_s,
                    from_state_id=SWING_AFTER_TO,
                    to_state_id=TIMEOUT,
                    closed_segment_type="swing",
                    segment_start_time_s=self.last_valid_to_time,
                    segment_valid=False,
                    opens_segment_type="",
                    cycle_valid=False,
                    cycle_reject_reason="phase_timeout:swing",
                )
                self.state_id = TIMEOUT
                self.timeout_exceeded = 1.0
                self.timeout_side = 2.0
                self._apply_cycle_failure()

    def _refresh_scores(self, normal_force_bw: float, in_contact: bool) -> None:
        cfg = self.config
        if self.state_id == WAIT_HS:
            self.cycle_progress_credit = 0.0
        elif self.state_id == STANCE_AFTER_HS and not self.cycle_completed_this_step:
            self.cycle_progress_credit = (
                0.0 if self.last_valid_hs_time is None else 0.25
            )
        elif self.state_id == SWING_AFTER_TO:
            self.cycle_progress_credit = 0.50

        self.landing_window_active = 0.0
        self.landing_window_contact_score = 0.0
        if self.state_id == SWING_AFTER_TO:
            if (
                float(cfg.landing_window_start_s)
                <= self.swing_elapsed_s
                <= float(cfg.landing_window_end_s)
            ):
                self.landing_window_active = 1.0
                force_score = float(
                    np.clip(
                        max(0.0, float(normal_force_bw))
                        / max(1e-9, float(cfg.landing_force_full_credit_bw)),
                        0.0,
                        1.0,
                    )
                )
                self.landing_window_contact_score = force_score if in_contact else 0.0

    def observation(self) -> dict[str, float]:
        cfg = self.config
        return {
            "phase_fsm_wait_hs": float(self.state_id == WAIT_HS),
            "phase_fsm_stance_after_hs": float(self.state_id == STANCE_AFTER_HS),
            "phase_fsm_swing_after_to": float(self.state_id == SWING_AFTER_TO),
            "phase_expected_hs": float(self.expected_next_event == "heel_strike"),
            "phase_expected_to": float(self.expected_next_event == "toe_off"),
            "phase_stance_elapsed_norm": float(
                np.clip(
                    self.stance_elapsed_s
                    / max(1e-9, float(cfg.stance_hard_timeout_s)),
                    0.0,
                    1.5,
                )
            ),
            "phase_swing_elapsed_norm": float(
                np.clip(
                    self.swing_elapsed_s
                    / max(1e-9, float(cfg.swing_hard_timeout_s)),
                    0.0,
                    1.5,
                )
            ),
            "phase_cycle_progress_credit": float(self.cycle_progress_credit),
        }

    def payload(self) -> dict[str, Any]:
        robust_stance_duration_s = (
            float(np.median(tuple(self._valid_stance_durations_s)))
            if self._valid_stance_durations_s
            else 0.0
        )
        robust_swing_duration_s = (
            float(np.median(tuple(self._valid_swing_durations_s)))
            if self._valid_swing_durations_s
            else 0.0
        )
        return {
            "event_source": self.config.event_source,
            "state_id": float(self.state_id),
            "state_name": STATE_NAMES.get(self.state_id, "UNKNOWN"),
            "expected_next_event": self.expected_next_event,
            "stance_elapsed_s": float(self.stance_elapsed_s),
            "swing_elapsed_s": float(self.swing_elapsed_s),
            "cycle_progress_credit": float(self.cycle_progress_credit),
            "pending_cycle_credit": float(self.pending_cycle_credit),
            "phase_event_progress_score": float(self.phase_event_progress_score),
            "phase_cycle_complete_bonus": float(self.phase_cycle_complete_bonus),
            "phase_clawback_penalty": float(self.phase_clawback_penalty),
            "phase_failure_extra_penalty": float(self.phase_failure_extra_penalty),
            "phase_cycle_failed_this_step": float(self.phase_cycle_failed_this_step),
            "valid_hs_count": float(self.valid_hs_count),
            "valid_to_count": float(self.valid_to_count),
            "valid_cycle_count": float(self.valid_cycle_count),
            "invalid_event_this_step": float(self.invalid_event_this_step),
            "invalid_event_count": float(self.invalid_event_count),
            "invalid_event_type": self.invalid_event_type,
            "invalid_event_loss": float(self.invalid_event_this_step),
            "contact_validity_loss": 0.0,
            "landing_window_active": float(self.landing_window_active),
            "landing_window_contact_score": float(self.landing_window_contact_score),
            "cycle_completed_this_step": float(self.cycle_completed_this_step),
            "timeout_exceeded": float(self.timeout_exceeded),
            "timeout_side": float(self.timeout_side),
            "last_period_s": float(self.last_period_s),
            "previous_period_s": float(self.previous_period_s),
            "last_stance_fraction": float(self.last_stance_fraction),
            "last_valid_hs_time_s": (
                float(self.last_valid_hs_time)
                if self.last_valid_hs_time is not None
                else -1.0
            ),
            "last_valid_to_time_s": (
                float(self.last_valid_to_time)
                if self.last_valid_to_time is not None
                else -1.0
            ),
            "accepted_transitions_this_step": [
                dict(item) for item in self.accepted_transitions_this_step
            ],
            "sensor_events_this_step": [
                dict(item) for item in self.sensor_events_this_step
            ],
            "sensor_edges_this_step": [
                dict(item) for item in self.sensor_edges_this_step
            ],
            "sensor_heel_contact": float(self._sensor_contact["heel"]),
            "sensor_toe_contact": float(self._sensor_contact["toe"]),
            "sensor_hs_armed": float(self._sensor_hs_armed),
            "sensor_forefoot_first": float(self._sensor_forefoot_first),
            "sensor_partial_stance_active": float(
                self._partial_stance_start_time_s is not None
            ),
            "robust_stance_duration_s": robust_stance_duration_s,
            "robust_swing_duration_s": robust_swing_duration_s,
            "duration_history_count": float(
                min(
                    len(self._valid_stance_durations_s),
                    len(self._valid_swing_durations_s),
                )
            ),
            "stance_contact_time_s": float(self.stance_contact_time_s),
            "stance_load_integral_bw_s": float(self.stance_load_integral_bw_s),
            "stance_contact_fraction": float(self.stance_contact_fraction),
            "stance_mean_load_bw": float(self.stance_mean_load_bw),
            "cycle_knee_excursion_rad": float(self.cycle_knee_excursion_rad),
            "cycle_ankle_excursion_rad": float(self.cycle_ankle_excursion_rad),
            "cycle_rejected_this_step": float(self.cycle_rejected_this_step),
            "cycle_reject_reason": self.cycle_reject_reason,
        }
