"""Experimental retrospective HS-TO-HS morphology-segment ledger.

This module is intentionally isolated from the production corridor modes.  It
does not modify actions, served references, simulator state, or the actor
observation.  It only buffers timestamped served-reference samples until the
prosthetic FSM accepts the event that closes a stance or swing segment.

Accepted FSM events may be confirmed after their physical timestamp.  The
ledger therefore repartitions its timestamped buffer at the event timestamp:

* ``[HS, TO)`` belongs to stance;
* ``[TO, next_HS)`` belongs to swing;
* the sample exactly on an event belongs to the newly opened segment.

Only a completed segment has an exact event-normalized phase.  Incomplete,
invalid, timed-out, or overflowing segments are reported and discarded rather
than assigned a fabricated future endpoint.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping, Sequence


EXPERIMENTAL_PHASE_MODE = "event_anchored_completed_segment_experimental"
CAUSAL_DELAYED_PHASE_MODE = "event_anchored_causal_delayed_experimental"
TWO_SENSOR_EVENT_CONTRACT_ID = (
    "primary_grf_split_v1+two_sensor_highrate_v1"
)
_TIME_EPS = 1e-12


@dataclass(frozen=True)
class MorphologySample:
    """One policy-step served reference, in raw OpenSim sign convention."""

    time_s: float
    knee_rad: float
    ankle_rad: float


@dataclass(frozen=True)
class CompletedMorphologySegment:
    """One exact, half-open FSM segment and the samples owned by it."""

    segment_type: str
    start_time_s: float
    end_time_s: float
    samples: tuple[MorphologySample, ...]

    @property
    def duration_s(self) -> float:
        return float(self.end_time_s - self.start_time_s)

    def phases(self, canonical_to_phase: float) -> tuple[float, ...]:
        """Map timestamps exactly onto the canonical stance/swing interval."""
        alpha = float(canonical_to_phase)
        duration = self.duration_s
        if not (0.0 < alpha < 1.0):
            raise ValueError("canonical_to_phase must lie strictly inside (0, 1)")
        if not math.isfinite(duration) or duration <= _TIME_EPS:
            raise ValueError("completed morphology segment has invalid duration")

        phases: list[float] = []
        for sample in self.samples:
            progress = (float(sample.time_s) - self.start_time_s) / duration
            # Samples are owned by a half-open interval.  The tiny numerical
            # clamp only absorbs timestamp roundoff; it never predicts/clamps a
            # segment against an unknown future event.
            progress = min(1.0, max(0.0, float(progress)))
            if self.segment_type == "stance":
                phase = alpha * progress
            elif self.segment_type == "swing":
                phase = alpha + (1.0 - alpha) * progress
            else:
                raise ValueError(f"unsupported segment_type={self.segment_type!r}")
            phases.append(float(phase))
        return tuple(phases)


@dataclass(frozen=True)
class MorphologyLedgerUpdate:
    """Immutable result emitted after one policy step."""

    completed_segments: tuple[CompletedMorphologySegment, ...] = ()
    discarded_segment_count: int = 0
    discarded_sample_count: int = 0
    discard_reason: str = ""
    overflowed: bool = False
    nonmonotonic_sample: bool = False
    pending_sample_count: int = 0
    active_segment_type: str = ""
    active_segment_start_time_s: float = -1.0


class CompletedSegmentMorphologyLedger:
    """Per-environment buffer driven only by accepted FSM transition records."""

    def __init__(self, *, max_samples: int = 4096) -> None:
        max_samples_i = int(max_samples)
        if max_samples_i < 2:
            raise ValueError("max_samples must be at least 2")
        self.max_samples = max_samples_i
        self.reset()

    def reset(self) -> None:
        self._samples: list[MorphologySample] = []
        self._active_segment_type = ""
        self._active_segment_start_time_s = -1.0

    @property
    def pending_sample_count(self) -> int:
        return len(self._samples)

    @property
    def active_segment_type(self) -> str:
        return self._active_segment_type

    def update(
        self,
        *,
        time_s: float,
        knee_rad: float,
        ankle_rad: float,
        accepted_transitions: Sequence[Mapping[str, Any]] | None,
        episode_ended: bool = False,
    ) -> MorphologyLedgerUpdate:
        """Buffer one sample and settle every accepted closure in this step."""
        completed: list[CompletedMorphologySegment] = []
        discarded_segments = 0
        discarded_samples = 0
        discard_reasons: list[str] = []
        overflowed = False
        nonmonotonic = False

        sample = self._finite_sample(time_s, knee_rad, ankle_rad)
        if sample is not None:
            if self._samples and sample.time_s < self._samples[-1].time_s - _TIME_EPS:
                nonmonotonic = True
                discard_reasons.append("nonmonotonic_sample")
            elif (
                self._samples
                and abs(sample.time_s - self._samples[-1].time_s) <= _TIME_EPS
            ):
                # A repeated timestamp represents the same policy instant.
                self._samples[-1] = sample
            else:
                self._samples.append(sample)

        if len(self._samples) > self.max_samples:
            overflowed = True
            discarded_segments += int(bool(self._active_segment_type))
            discarded_samples += len(self._samples) - 1
            discard_reasons.append("buffer_overflow")
            self._samples = self._samples[-1:]
            self._active_segment_type = ""
            self._active_segment_start_time_s = -1.0

        transitions = [
            item for item in (accepted_transitions or ()) if isinstance(item, Mapping)
        ]
        transitions.sort(key=self._transition_sort_key)
        for transition in transitions:
            result = self._process_transition(transition)
            if result["completed"] is not None:
                completed.append(result["completed"])
            discarded_segments += int(result["discarded_segment_count"])
            discarded_samples += int(result["discarded_sample_count"])
            reason = str(result["discard_reason"] or "")
            if reason:
                discard_reasons.append(reason)

        if episode_ended and (self._samples or self._active_segment_type):
            discarded_segments += int(bool(self._active_segment_type))
            discarded_samples += len(self._samples)
            discard_reasons.append(
                "episode_end_incomplete_segment"
                if self._active_segment_type
                else "episode_end_before_first_hs"
            )
            self.reset()

        return MorphologyLedgerUpdate(
            completed_segments=tuple(completed),
            discarded_segment_count=discarded_segments,
            discarded_sample_count=discarded_samples,
            discard_reason="|".join(dict.fromkeys(discard_reasons)),
            overflowed=overflowed,
            nonmonotonic_sample=nonmonotonic,
            pending_sample_count=len(self._samples),
            active_segment_type=self._active_segment_type,
            active_segment_start_time_s=float(self._active_segment_start_time_s),
        )

    @staticmethod
    def _finite_sample(
        time_s: float,
        knee_rad: float,
        ankle_rad: float,
    ) -> MorphologySample | None:
        try:
            values = (float(time_s), float(knee_rad), float(ankle_rad))
        except (TypeError, ValueError):
            return None
        if not all(math.isfinite(value) for value in values):
            return None
        return MorphologySample(*values)

    @staticmethod
    def _transition_sort_key(transition: Mapping[str, Any]) -> tuple[float, int]:
        try:
            time_s = float(transition.get("event_time_s", float("inf")))
        except (TypeError, ValueError):
            time_s = float("inf")
        event = str(transition.get("event", "") or "").lower()
        order = {"toe_off": 0, "heel_strike": 1, "timeout": 2}.get(event, 3)
        return time_s, order

    def _process_transition(self, transition: Mapping[str, Any]) -> dict[str, Any]:
        event = str(transition.get("event", "") or "").lower()
        try:
            event_time_s = float(transition.get("event_time_s"))
        except (TypeError, ValueError):
            return self._discard_open("invalid_transition_time")
        if not math.isfinite(event_time_s):
            return self._discard_open("invalid_transition_time")

        if event == "timeout":
            return self._discard_open("fsm_timeout")
        if event not in {"heel_strike", "toe_off"}:
            return {
                "completed": None,
                "discarded_segment_count": 0,
                "discarded_sample_count": 0,
                "discard_reason": "",
            }

        expected_closed = "stance" if event == "toe_off" else "swing"
        opens = "swing" if event == "toe_off" else "stance"
        declared_closed = str(
            transition.get("closed_segment_type", expected_closed) or ""
        ).lower()
        segment_valid = bool(float(transition.get("segment_valid", 1.0) or 0.0))
        declared_start_raw = transition.get("segment_start_time_s")
        declared_start_matches = True
        if declared_start_raw is not None:
            try:
                declared_start = float(declared_start_raw)
            except (TypeError, ValueError):
                declared_start_matches = False
            else:
                if not math.isfinite(declared_start):
                    declared_start_matches = False
                elif self._active_segment_type:
                    declared_start_matches = (
                        declared_start >= 0.0
                        and abs(declared_start - self._active_segment_start_time_s)
                        <= _TIME_EPS
                    )

        before, carry = self._split_samples(event_time_s)
        completed = None
        discarded_segments = 0
        discarded_samples = 0
        reason = ""

        if self._active_segment_type:
            valid_geometry = (
                self._active_segment_type == expected_closed
                and declared_closed == expected_closed
                and declared_start_matches
                and event_time_s > self._active_segment_start_time_s + _TIME_EPS
            )
            if segment_valid and valid_geometry and before:
                completed = CompletedMorphologySegment(
                    segment_type=self._active_segment_type,
                    start_time_s=float(self._active_segment_start_time_s),
                    end_time_s=float(event_time_s),
                    samples=tuple(before),
                )
            else:
                discarded_segments = 1
                discarded_samples = len(before)
                if not segment_valid:
                    reason = "fsm_rejected_segment"
                elif not declared_start_matches:
                    reason = "transition_start_mismatch"
                elif not valid_geometry:
                    reason = "invalid_segment_geometry"
                else:
                    reason = "empty_completed_segment"
        else:
            # Samples before the first accepted HS are a lookback queue.  They
            # exist only so a backdated first HS can recover samples after the
            # true event; samples genuinely before the anchor are unavailable.
            discarded_samples = len(before)
            if before:
                reason = "before_first_anchor"

        self._samples = carry
        self._active_segment_type = opens
        self._active_segment_start_time_s = float(event_time_s)
        return {
            "completed": completed,
            "discarded_segment_count": discarded_segments,
            "discarded_sample_count": discarded_samples,
            "discard_reason": reason,
        }

    def _split_samples(
        self,
        event_time_s: float,
    ) -> tuple[list[MorphologySample], list[MorphologySample]]:
        before: list[MorphologySample] = []
        carry: list[MorphologySample] = []
        for sample in self._samples:
            if sample.time_s < event_time_s - _TIME_EPS:
                before.append(sample)
            else:
                carry.append(sample)
        return before, carry

    def _discard_open(self, reason: str) -> dict[str, Any]:
        discarded_samples = len(self._samples)
        discarded_segments = int(bool(self._active_segment_type))
        self.reset()
        return {
            "completed": None,
            "discarded_segment_count": discarded_segments,
            "discarded_sample_count": discarded_samples,
            "discard_reason": str(reason),
        }


@dataclass(frozen=True)
class CausalMorphologyAnchor:
    """One accepted detector transition, anchored to its physical onset."""

    event: str
    segment_type: str
    event_time_s: float
    confirmed_time_s: float
    delivered_time_s: float
    initial_partial_bootstrap: bool = False


@dataclass(frozen=True)
class ResolvedCausalMorphologySample:
    """A served-reference sample that is safe to score causally.

    ``emitted_time_s`` is when the loss may affect the reward.  ``phase`` is
    computed from the latest accepted physical event available by that time;
    it never uses the next event or a prescribed trajectory.
    """

    sample: MorphologySample
    segment_type: str
    segment_start_time_s: float
    anchor_confirmed_time_s: float
    anchor_delivered_time_s: float
    emitted_time_s: float
    duration_basis_s: float
    phase: float
    terminal_flush: bool = False

    @property
    def delay_s(self) -> float:
        return float(self.emitted_time_s - self.sample.time_s)


@dataclass(frozen=True)
class CausalMorphologyUpdate:
    """Immutable output of one causal delayed-buffer update."""

    resolved_samples: tuple[ResolvedCausalMorphologySample, ...] = ()
    dropped_sample_count: int = 0
    dropped_pending_sample_count: int = 0
    drop_reason: str = ""
    pending_sample_count: int = 0
    total_resolved_sample_count: int = 0
    total_dropped_sample_count: int = 0
    terminal_flushed: bool = False
    failed_closed: bool = False
    failure_reason: str = ""


class CausalDelayedMorphologyBuffer:
    """Fixed-delay, event-anchored morphology buffer.

    The caller records the knee/ankle reference actually served at every
    policy step and supplies the detector's accepted-transition journal.  A
    normal update emits only samples at least ``delay_s`` old.  Events are
    applied before eligible samples are resolved, so a transition delivered
    on the delay boundary owns the sample at its physical onset.

    A detector transition that is still being debounced must be supplied as
    ``pending_transition`` on every update.  Samples at or after that physical
    onset remain buffered.  On episode end, resolved samples are flushed and
    only samples at/after the pending onset are dropped.  This is the explicit
    terminal rule required to avoid inventing a future event.

    The class only returns morphology samples and phases.  It deliberately has
    no observation, pulse, action, simulator, or prescribed-trajectory API.
    """

    def __init__(
        self,
        *,
        delay_s: float = 0.04,
        canonical_to_phase: float,
        nominal_stance_duration_s: float,
        nominal_swing_duration_s: float,
        max_delivery_latency_s: float = 0.01,
        max_samples: int = 4096,
        event_contract_id: str = TWO_SENSOR_EVENT_CONTRACT_ID,
    ) -> None:
        self.delay_s = self._positive_finite(delay_s, "delay_s")
        alpha = float(canonical_to_phase)
        if not math.isfinite(alpha) or not 0.0 < alpha < 1.0:
            raise ValueError("canonical_to_phase must lie strictly inside (0, 1)")
        self.canonical_to_phase = alpha
        self.nominal_stance_duration_s = self._positive_finite(
            nominal_stance_duration_s,
            "nominal_stance_duration_s",
        )
        self.nominal_swing_duration_s = self._positive_finite(
            nominal_swing_duration_s,
            "nominal_swing_duration_s",
        )
        self.max_delivery_latency_s = self._positive_finite(
            max_delivery_latency_s,
            "max_delivery_latency_s",
        )
        if self.max_delivery_latency_s > self.delay_s + _TIME_EPS:
            raise ValueError("max_delivery_latency_s cannot exceed delay_s")
        max_samples_i = int(max_samples)
        if max_samples_i < 2:
            raise ValueError("max_samples must be at least 2")
        self.max_samples = max_samples_i
        contract = str(event_contract_id or "").strip()
        if contract != TWO_SENSOR_EVENT_CONTRACT_ID:
            raise ValueError(
                "event_contract_id must be "
                f"{TWO_SENSOR_EVENT_CONTRACT_ID!r}"
            )
        self.event_contract_id = contract
        self.reset()

    def reset(self) -> None:
        self._samples: list[MorphologySample] = []
        self._anchors: list[CausalMorphologyAnchor] = []
        self._last_update_time_s: float | None = None
        self._last_emitted_sample_time_s: float | None = None
        self._last_pending: tuple[str, float] | None = None
        self._total_resolved_sample_count = 0
        self._total_dropped_sample_count = 0
        self._failed_reason = ""
        self._episode_ended = False

    @property
    def failed_closed(self) -> bool:
        return bool(self._failed_reason)

    @property
    def failure_reason(self) -> str:
        return self._failed_reason

    @property
    def pending_sample_count(self) -> int:
        return len(self._samples)

    def update(
        self,
        *,
        time_s: float,
        knee_rad: float,
        ankle_rad: float,
        accepted_transitions: Sequence[Mapping[str, Any]] | None,
        pending_transition: Mapping[str, Any] | None = None,
        episode_ended: bool = False,
        stance_duration_s: float | None = None,
        swing_duration_s: float | None = None,
    ) -> CausalMorphologyUpdate:
        """Append one served sample and emit only causally resolved samples."""
        if self._failed_reason:
            return self._snapshot(failed_closed=True)
        if self._episode_ended:
            return self._fail_closed("update_after_episode_end", rejected_samples=1)

        sample = self._finite_sample(time_s, knee_rad, ankle_rad)
        if sample is None:
            return self._fail_closed("nonfinite_sample", rejected_samples=1)
        if (
            self._last_update_time_s is not None
            and sample.time_s <= self._last_update_time_s + _TIME_EPS
        ):
            return self._fail_closed("nonmonotonic_policy_time", rejected_samples=1)

        try:
            stance_duration = self._duration_or_default(
                stance_duration_s,
                self.nominal_stance_duration_s,
                "stance_duration_s",
            )
            swing_duration = self._duration_or_default(
                swing_duration_s,
                self.nominal_swing_duration_s,
                "swing_duration_s",
            )
            anchors = self._parse_anchors(
                accepted_transitions or (),
                delivered_by_s=sample.time_s,
            )
            pending = self._parse_pending(pending_transition, now_s=sample.time_s)
            self._validate_anchor_sequence(anchors)
            pending = self._validate_pending_continuity(pending, anchors)
        except (TypeError, ValueError) as exc:
            return self._fail_closed(str(exc), rejected_samples=1)

        self._last_update_time_s = sample.time_s
        self._samples.append(sample)
        if len(self._samples) > self.max_samples:
            return self._fail_closed("causal_buffer_overflow")

        dropped = 0
        drop_reasons: list[str] = []
        for anchor in anchors:
            if not self._anchors:
                before = [
                    item
                    for item in self._samples
                    if item.time_s < anchor.event_time_s - _TIME_EPS
                ]
                if before:
                    dropped += len(before)
                    drop_reasons.append(
                        "before_initial_partial_to"
                        if anchor.initial_partial_bootstrap
                        else "before_first_physical_hs"
                    )
                    self._samples = [
                        item
                        for item in self._samples
                        if item.time_s >= anchor.event_time_s - _TIME_EPS
                    ]
            self._anchors.append(anchor)
        self._last_pending = pending

        resolved: list[ResolvedCausalMorphologySample] = []
        cutoff_s = sample.time_s - self.delay_s
        carry: list[MorphologySample] = []
        for buffered in self._samples:
            if buffered.time_s > cutoff_s + _TIME_EPS:
                carry.append(buffered)
                continue
            if pending is not None and buffered.time_s >= pending[1] - _TIME_EPS:
                carry.append(buffered)
                continue
            item = self._resolve(
                buffered,
                emitted_time_s=sample.time_s,
                stance_duration_s=stance_duration,
                swing_duration_s=swing_duration,
                terminal_flush=False,
            )
            if item is None:
                carry.append(buffered)
            else:
                resolved.append(item)
        self._samples = carry

        dropped_pending = 0
        terminal_flushed = False
        if episode_ended:
            terminal_flushed = True
            terminal_resolved: list[ResolvedCausalMorphologySample] = []
            terminal_unresolved = 0
            for buffered in self._samples:
                if pending is not None and buffered.time_s >= pending[1] - _TIME_EPS:
                    dropped_pending += 1
                    continue
                item = self._resolve(
                    buffered,
                    emitted_time_s=sample.time_s,
                    stance_duration_s=stance_duration,
                    swing_duration_s=swing_duration,
                    terminal_flush=True,
                )
                if item is None:
                    terminal_unresolved += 1
                else:
                    terminal_resolved.append(item)
            resolved.extend(terminal_resolved)
            if dropped_pending:
                drop_reasons.append("episode_end_pending_transition")
            if terminal_unresolved:
                dropped += terminal_unresolved
                drop_reasons.append("episode_end_without_physical_anchor")
            dropped += dropped_pending
            self._samples = []
            self._episode_ended = True

        if resolved:
            self._last_emitted_sample_time_s = max(
                item.sample.time_s for item in resolved
            )
        self._total_resolved_sample_count += len(resolved)
        self._total_dropped_sample_count += dropped
        return CausalMorphologyUpdate(
            resolved_samples=tuple(resolved),
            dropped_sample_count=dropped,
            dropped_pending_sample_count=dropped_pending,
            drop_reason="|".join(dict.fromkeys(drop_reasons)),
            pending_sample_count=len(self._samples),
            total_resolved_sample_count=self._total_resolved_sample_count,
            total_dropped_sample_count=self._total_dropped_sample_count,
            terminal_flushed=terminal_flushed,
        )

    @staticmethod
    def _positive_finite(value: float, name: str) -> float:
        try:
            result = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must be finite and positive") from exc
        if not math.isfinite(result) or result <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return result

    @staticmethod
    def _finite_sample(
        time_s: float,
        knee_rad: float,
        ankle_rad: float,
    ) -> MorphologySample | None:
        try:
            values = (float(time_s), float(knee_rad), float(ankle_rad))
        except (TypeError, ValueError):
            return None
        if not all(math.isfinite(value) for value in values):
            return None
        return MorphologySample(*values)

    @classmethod
    def _duration_or_default(
        cls,
        candidate: float | None,
        default: float,
        name: str,
    ) -> float:
        return default if candidate is None else cls._positive_finite(candidate, name)

    def _parse_anchors(
        self,
        transitions: Sequence[Mapping[str, Any]],
        *,
        delivered_by_s: float,
    ) -> list[CausalMorphologyAnchor]:
        anchors: list[CausalMorphologyAnchor] = []
        for transition in transitions:
            if not isinstance(transition, Mapping):
                raise ValueError("invalid_transition_record")
            event = str(transition.get("event", "") or "").strip().lower()
            if event not in {"heel_strike", "toe_off"}:
                raise ValueError("unsupported_transition_event")
            try:
                event_time_s = float(transition["event_time_s"])
                confirmed_time_s = float(transition["confirmed_time_s"])
                delivered_time_s = float(transition["delivered_time_s"])
                segment_valid = float(transition.get("segment_valid", 1.0))
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError("invalid_transition_timestamps") from exc
            values = (
                event_time_s,
                confirmed_time_s,
                delivered_time_s,
                segment_valid,
            )
            if not all(math.isfinite(value) for value in values):
                raise ValueError("nonfinite_transition")
            initial_partial = bool(
                segment_valid == 0.0
                and event == "toe_off"
                and not self._anchors
                and not anchors
                and transition.get("from_state_id") == 1.0
                and transition.get("to_state_id") == 2.0
                and str(transition.get("closed_segment_type", "")).strip().lower()
                == "stance"
                and str(transition.get("opens_segment_type", "")).strip().lower()
                == "swing"
            )
            if segment_valid not in {0.0, 1.0} or (
                segment_valid != 1.0 and not initial_partial
            ):
                raise ValueError("invalid_or_rejected_transition_segment")
            if not (
                event_time_s <= confirmed_time_s + _TIME_EPS
                and confirmed_time_s <= delivered_time_s + _TIME_EPS
                and delivered_time_s <= delivered_by_s + _TIME_EPS
            ):
                raise ValueError("invalid_transition_time_order")
            if delivered_time_s - event_time_s > self.delay_s + _TIME_EPS:
                raise ValueError("transition_exceeds_morphology_delay")
            if (
                delivered_time_s - confirmed_time_s
                > self.max_delivery_latency_s + _TIME_EPS
            ):
                raise ValueError("transition_exceeds_delivery_latency")
            segment_type = "stance" if event == "heel_strike" else "swing"
            declared_opens = transition.get("opens_segment_type")
            if (
                declared_opens is not None
                and str(declared_opens).strip().lower() != segment_type
            ):
                raise ValueError("transition_open_segment_mismatch")
            anchors.append(
                CausalMorphologyAnchor(
                    event=event,
                    segment_type=segment_type,
                    event_time_s=event_time_s,
                    confirmed_time_s=confirmed_time_s,
                    delivered_time_s=delivered_time_s,
                    initial_partial_bootstrap=initial_partial,
                )
            )
        anchors.sort(key=lambda item: (item.delivered_time_s, item.event_time_s))
        return anchors

    def _validate_anchor_sequence(
        self,
        candidates: Sequence[CausalMorphologyAnchor],
    ) -> None:
        previous = self._anchors[-1] if self._anchors else None
        last_emitted = self._last_emitted_sample_time_s
        for anchor in candidates:
            if previous is None and anchor.initial_partial_bootstrap:
                expected = "toe_off"
            else:
                expected = (
                    "heel_strike"
                    if previous is None or previous.event == "toe_off"
                    else "toe_off"
                )
            if anchor.event != expected:
                raise ValueError("nonalternating_transition_sequence")
            if (
                previous is not None
                and anchor.event_time_s <= previous.event_time_s + _TIME_EPS
            ):
                raise ValueError("nonmonotonic_physical_event_time")
            if (
                last_emitted is not None
                and anchor.event_time_s <= last_emitted + _TIME_EPS
            ):
                raise ValueError("late_transition_overlaps_emitted_sample")
            previous = anchor

    def _parse_pending(
        self,
        transition: Mapping[str, Any] | None,
        *,
        now_s: float,
    ) -> tuple[str, float] | None:
        if transition is None:
            return None
        if not isinstance(transition, Mapping):
            raise ValueError("invalid_pending_transition")
        event = str(transition.get("event", "") or "").strip().lower()
        if event not in {"heel_strike", "toe_off"}:
            raise ValueError("invalid_pending_transition_event")
        try:
            onset_s = float(transition["event_time_s"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("invalid_pending_transition_time") from exc
        if not math.isfinite(onset_s) or onset_s > now_s + _TIME_EPS:
            raise ValueError("invalid_pending_transition_time")
        return event, onset_s

    def _validate_pending_continuity(
        self,
        pending: tuple[str, float] | None,
        new_anchors: Sequence[CausalMorphologyAnchor],
    ) -> tuple[str, float] | None:
        previous_pending = self._last_pending
        matched_previous = False
        for anchor in new_anchors:
            if (
                previous_pending is not None
                and anchor.event == previous_pending[0]
                and abs(anchor.event_time_s - previous_pending[1]) <= _TIME_EPS
            ):
                matched_previous = True
        if previous_pending is not None and not matched_previous:
            if pending is None:
                raise ValueError("pending_transition_disappeared")
            if (
                pending[0] != previous_pending[0]
                or abs(pending[1] - previous_pending[1]) > _TIME_EPS
            ):
                raise ValueError("pending_transition_changed")

        if pending is not None:
            if any(
                anchor.event == pending[0]
                and abs(anchor.event_time_s - pending[1]) <= _TIME_EPS
                for anchor in new_anchors
            ):
                return None
            previous = new_anchors[-1] if new_anchors else (
                self._anchors[-1] if self._anchors else None
            )
            expected = (
                {"heel_strike", "toe_off"}
                if previous is None
                else ({"heel_strike"} if previous.event == "toe_off" else {"toe_off"})
            )
            if pending[0] not in expected:
                raise ValueError("pending_transition_wrong_event")
            if previous is not None and pending[1] <= previous.event_time_s + _TIME_EPS:
                raise ValueError("pending_transition_before_active_anchor")
        return pending

    def _resolve(
        self,
        sample: MorphologySample,
        *,
        emitted_time_s: float,
        stance_duration_s: float,
        swing_duration_s: float,
        terminal_flush: bool,
    ) -> ResolvedCausalMorphologySample | None:
        anchor = next(
            (
                item
                for item in reversed(self._anchors)
                if item.event_time_s <= sample.time_s + _TIME_EPS
            ),
            None,
        )
        if anchor is None:
            return None
        duration = (
            stance_duration_s
            if anchor.segment_type == "stance"
            else swing_duration_s
        )
        progress = float(
            min(1.0, max(0.0, (sample.time_s - anchor.event_time_s) / duration))
        )
        if anchor.segment_type == "stance":
            phase = self.canonical_to_phase * progress
        else:
            phase = self.canonical_to_phase + (
                (1.0 - self.canonical_to_phase) * progress
            )
        return ResolvedCausalMorphologySample(
            sample=sample,
            segment_type=anchor.segment_type,
            segment_start_time_s=anchor.event_time_s,
            anchor_confirmed_time_s=anchor.confirmed_time_s,
            anchor_delivered_time_s=anchor.delivered_time_s,
            emitted_time_s=float(emitted_time_s),
            duration_basis_s=float(duration),
            phase=float(phase),
            terminal_flush=bool(terminal_flush),
        )

    def _fail_closed(
        self,
        reason: str,
        *,
        rejected_samples: int = 0,
    ) -> CausalMorphologyUpdate:
        dropped = len(self._samples) + max(0, int(rejected_samples))
        self._samples = []
        self._failed_reason = str(reason)
        self._total_dropped_sample_count += dropped
        return self._snapshot(
            dropped_sample_count=dropped,
            drop_reason=str(reason),
            failed_closed=True,
        )

    def _snapshot(
        self,
        *,
        dropped_sample_count: int = 0,
        drop_reason: str = "",
        failed_closed: bool = False,
    ) -> CausalMorphologyUpdate:
        return CausalMorphologyUpdate(
            dropped_sample_count=int(dropped_sample_count),
            drop_reason=str(drop_reason),
            pending_sample_count=len(self._samples),
            total_resolved_sample_count=self._total_resolved_sample_count,
            total_dropped_sample_count=self._total_dropped_sample_count,
            failed_closed=bool(failed_closed or self._failed_reason),
            failure_reason=str(self._failed_reason),
        )


def apply_causal_morphology_reward(
    base_reward: float,
    morphology_loss: float,
    morphology_weight: float,
) -> float:
    """Apply only the delayed morphology term to an already-computed reward.

    A zero weight returns the original Python float directly, which provides a
    strict weight-zero parity contract.  This helper has no access to actions,
    observations, detector pulses, or simulator state.
    """
    try:
        base = float(base_reward)
        loss = float(morphology_loss)
        weight = float(morphology_weight)
    except (TypeError, ValueError) as exc:
        raise ValueError("morphology reward inputs must be finite") from exc
    if not all(math.isfinite(value) for value in (base, loss, weight)):
        raise ValueError("morphology reward inputs must be finite")
    if weight < 0.0:
        raise ValueError("morphology_weight must be non-negative")
    if weight == 0.0:
        return base
    result = base - weight * loss
    if not math.isfinite(result):
        raise ValueError("delayed morphology reward is non-finite")
    return float(result)
