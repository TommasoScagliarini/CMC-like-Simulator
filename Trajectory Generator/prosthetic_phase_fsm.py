"""Runtime finite-state machine for prosthetic HS-TO-HS phase tracking.

The FSM is intentionally sensor-level: it consumes only prosthetic-side online
GRF detector events and force/contact flags. It stores per-episode state in
memory and exposes a compact observation payload plus richer diagnostics.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Sequence

import numpy as np


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


@dataclass(frozen=True)
class ProstheticPhaseFSMConfig:
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

    @property
    def expected_next_event(self) -> str:
        if self.state_id in {WAIT_HS, SWING_AFTER_TO}:
            return "heel_strike"
        if self.state_id == STANCE_AFTER_HS:
            return "toe_off"
        return "none"

    def update(
        self,
        *,
        time_s: float,
        events: Sequence[Mapping[str, Any]],
        normal_force_bw: float,
        in_contact: bool,
        prosthetic_knee_angle_rad: float | None = None,
        prosthetic_ankle_angle_rad: float | None = None,
    ) -> dict[str, Any]:
        time_f = float(time_s)
        normal_force_f = float(normal_force_bw)
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

        self._accumulate_stance_evidence(
            time_f,
            normal_force_bw=normal_force_f,
            in_contact=bool(in_contact),
        )
        self._record_cycle_kinematics(
            prosthetic_knee_angle_rad,
            prosthetic_ankle_angle_rad,
        )
        self._current_knee_angle_rad = prosthetic_knee_angle_rad
        self._current_ankle_angle_rad = prosthetic_ankle_angle_rad

        for event in sorted(
            (event for event in events if isinstance(event, Mapping)),
            key=lambda item: float(item.get("time", time_f) or time_f),
        ):
            if str(event.get("side", "")).lower() != "left":
                continue
            name = str(event.get("event", "")).lower()
            event_time = float(event.get("time", time_f) or time_f)
            if name == "heel_strike":
                self._handle_heel_strike(event_time)
            elif name == "toe_off":
                self._handle_toe_off(event_time, event)

        self._refresh_time_terms(time_f)
        self._refresh_stance_diagnostics()
        self._refresh_cycle_excursions()
        self._refresh_scores(normal_force_f, bool(in_contact))
        self.last_update_time_s = time_f
        return self.payload()

    def _handle_heel_strike(self, event_time: float) -> None:
        cfg = self.config
        if self.state_id == WAIT_HS:
            self._accept_hs(event_time, progress=0.25)
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
            valid, reason = self._cycle_valid_for_completion(event_time)
            if not valid:
                self._reject_cycle(reason, accept_new_hs=True, event_time=event_time)
                return
            if previous_hs is not None and event_time > float(previous_hs):
                period = event_time - float(previous_hs)
                self.previous_period_s = float(self.last_period_s)
                self.last_period_s = float(period)
                if previous_to is not None and float(previous_hs) < float(previous_to) < event_time:
                    self.last_stance_fraction = float(
                        (float(previous_to) - float(previous_hs)) / period
                    )
            self.valid_cycle_count += 1
            self.cycle_completed_this_step = 1.0
            self._add_event_credit(cfg.cycle_complete_bonus)
            self.phase_cycle_complete_bonus = float(cfg.cycle_complete_bonus)
            self._accept_hs(event_time, progress=1.0)
            return
        if self.state_id == TIMEOUT:
            return

    def _handle_toe_off(self, event_time: float, event: Mapping[str, Any]) -> None:
        cfg = self.config
        if self.state_id == WAIT_HS:
            self._mark_invalid("to_before_hs")
            return
        if self.state_id == STANCE_AFTER_HS:
            stance_elapsed = (
                event_time - float(self.last_valid_hs_time)
                if self.last_valid_hs_time is not None
                else 0.0
            )
            if stance_elapsed < float(cfg.min_stance_duration_s):
                self._mark_invalid("to_too_early_after_hs")
                return
            valid, reason = self._stance_valid_for_toe_off(stance_elapsed, event)
            if not valid:
                self._mark_invalid(reason)
                return
            self.last_valid_to_time = float(event_time)
            self.valid_to_count += 1
            self.state_id = SWING_AFTER_TO
            self.cycle_progress_credit = 0.50
            self.pending_cycle_credit += float(cfg.toe_off_event_credit)
            self._add_event_credit(cfg.toe_off_event_credit)
            return
        if self.state_id == SWING_AFTER_TO:
            self._mark_invalid("double_to_before_hs")
            return

    def _accept_hs(self, event_time: float, *, progress: float) -> None:
        cfg = self.config
        self.last_valid_hs_time = float(event_time)
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

    def _reject_cycle(
        self,
        reason: str,
        *,
        accept_new_hs: bool,
        event_time: float,
    ) -> None:
        self.cycle_rejected_this_step = 1.0
        self.cycle_reject_reason = str(reason)
        self._mark_invalid(str(reason))
        if accept_new_hs:
            self._accept_hs(event_time, progress=0.25)

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
        if self.state_id == STANCE_AFTER_HS and self.last_valid_hs_time is not None:
            self.stance_elapsed_s = max(0.0, time_s - float(self.last_valid_hs_time))
            if (
                float(cfg.stance_hard_timeout_s) > 0.0
                and self.stance_elapsed_s > float(cfg.stance_hard_timeout_s)
            ):
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
                self.state_id = TIMEOUT
                self.timeout_exceeded = 1.0
                self.timeout_side = 2.0
                self._apply_cycle_failure()

    def _refresh_scores(self, normal_force_bw: float, in_contact: bool) -> None:
        cfg = self.config
        if self.state_id == WAIT_HS:
            self.cycle_progress_credit = 0.0
        elif self.state_id == STANCE_AFTER_HS and not self.cycle_completed_this_step:
            self.cycle_progress_credit = 0.25
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
        return {
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
            "stance_contact_time_s": float(self.stance_contact_time_s),
            "stance_load_integral_bw_s": float(self.stance_load_integral_bw_s),
            "stance_contact_fraction": float(self.stance_contact_fraction),
            "stance_mean_load_bw": float(self.stance_mean_load_bw),
            "cycle_knee_excursion_rad": float(self.cycle_knee_excursion_rad),
            "cycle_ankle_excursion_rad": float(self.cycle_ankle_excursion_rad),
            "cycle_rejected_this_step": float(self.cycle_rejected_this_step),
            "cycle_reject_reason": self.cycle_reject_reason,
        }
