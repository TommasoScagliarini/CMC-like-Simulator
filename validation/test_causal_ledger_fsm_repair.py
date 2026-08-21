"""Causal morphology buffer × actor-FSM v3 repairs: re-arm, never fail closed."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
for entry in (str(REPO / "Trajectory Generator"), str(REPO / "Trajectory Generator" / "baseline_MLP")):
    if entry not in sys.path:
        sys.path.insert(0, entry)


from experimental_morphology_corridor import (  # noqa: E402
    CausalDelayedMorphologyBuffer,
    FSM_REPAIR_EVENTS,
)

CONTRACT = "binary_point_v25+heel_qualified_fsm_v2"
DT = 0.010


def make_buffer() -> CausalDelayedMorphologyBuffer:
    return CausalDelayedMorphologyBuffer(
        delay_s=0.04,
        canonical_to_phase=0.62,
        nominal_stance_duration_s=1.0,
        nominal_swing_duration_s=0.6,
        event_contract_id=CONTRACT,
    )


def transition(event: str, onset: float, delivered: float, *, from_id: float, to_id: float,
               closed: str, opens: str, valid: bool = True) -> dict:
    return {
        "event": event,
        "event_time_s": onset,
        "confirmed_time_s": onset + 0.005,
        "delivered_time_s": delivered,
        "from_state_id": from_id,
        "to_state_id": to_id,
        "closed_segment_type": closed,
        "segment_start_time_s": -1.0,
        "segment_end_time_s": onset,
        "segment_valid": 1.0 if valid else 0.0,
        "anchor_geometry_valid": 1.0 if valid else 0.0,
        "opens_segment_type": opens,
        "cycle_valid": -1.0,
        "cycle_reject_reason": "",
    }


def drive(buf, t, *, transitions=(), state="STANCE_AFTER_HS"):
    return buf.update(
        time_s=t, knee_rad=-0.3, ankle_rad=0.1,
        accepted_transitions=list(transitions), actor_state_name=state,
    )


def test_repair_events_are_the_expected_set() -> None:
    assert FSM_REPAIR_EVENTS == {"toe_off_resync", "heel_strike_resync", "heel_strike_cancelled"}


def test_toe_off_resync_rearms_instead_of_failing_closed() -> None:
    buf = make_buffer()
    t = 1.0
    # real HS anchor opens a stance
    upd = drive(buf, t, transitions=(transition("heel_strike", t - 0.009, t, from_id=0.0, to_id=1.0, closed="", opens="stance"),), state="STANCE_AFTER_HS")
    assert not upd.failed_closed
    for _ in range(30):
        t = round(t + DT, 6)
        upd = drive(buf, t, state="STANCE_AFTER_HS")
        assert not upd.failed_closed
    # v3 repair: late, invalid-segment transition, actor already in swing
    t = round(t + DT, 6)
    repair = transition("toe_off_resync", t - 0.08, t, from_id=1.0, to_id=2.0, closed="stance", opens="swing", valid=False)
    upd = drive(buf, t, transitions=(repair,), state="SWING_AFTER_TO")
    assert not upd.failed_closed, upd.failure_reason
    assert upd.total_repair_transition_count == 1
    # alignment stays suspended while re-arming (actor in swing, no anchors)
    for _ in range(20):
        t = round(t + DT, 6)
        upd = drive(buf, t, state="SWING_AFTER_TO")
        assert not upd.failed_closed, upd.failure_reason
    # first real anchor after the repair re-starts the chain and alignment resumes
    t = round(t + DT, 6)
    upd = drive(buf, t, transitions=(transition("heel_strike", t - 0.009, t, from_id=2.0, to_id=1.0, closed="swing", opens="stance"),), state="STANCE_AFTER_HS")
    assert not upd.failed_closed, upd.failure_reason
    for _ in range(10):
        t = round(t + DT, 6)
        assert not drive(buf, t, state="STANCE_AFTER_HS").failed_closed
    # and alignment is enforced again: a wrong actor state now fails closed
    t = round(t + DT, 6)
    assert drive(buf, t, state="SWING_AFTER_TO").failed_closed


def test_first_anchor_after_heel_strike_resync_may_be_a_toe_off() -> None:
    buf = make_buffer()
    t = 1.0
    upd = drive(buf, t, transitions=(transition("heel_strike_resync", t - 0.08, t, from_id=2.0, to_id=1.0, closed="swing", opens="stance", valid=False),), state="STANCE_AFTER_HS")
    assert not upd.failed_closed, upd.failure_reason
    for _ in range(40):
        t = round(t + DT, 6)
        assert not drive(buf, t, state="STANCE_AFTER_HS").failed_closed
    t = round(t + DT, 6)
    upd = drive(buf, t, transitions=(transition("toe_off", t - 0.009, t, from_id=1.0, to_id=2.0, closed="stance", opens="swing"),), state="SWING_AFTER_TO")
    assert not upd.failed_closed, upd.failure_reason


def test_unknown_transition_event_still_fails_closed() -> None:
    buf = make_buffer()
    upd = drive(buf, 1.0, transitions=(transition("bogus_event", 0.991, 1.0, from_id=0.0, to_id=1.0, closed="", opens="stance"),), state="STANCE_AFTER_HS")
    assert upd.failed_closed
    assert upd.failure_reason == "unsupported_transition_event"


@pytest.mark.parametrize("state", ["STANCE_AFTER_HS"])
def test_nominal_sequence_unchanged_without_repairs(state: str) -> None:
    buf = make_buffer()
    t = 1.0
    upd = drive(buf, t, transitions=(transition("heel_strike", t - 0.009, t, from_id=0.0, to_id=1.0, closed="", opens="stance"),), state=state)
    for _ in range(60):
        t = round(t + DT, 6)
        upd = drive(buf, t, state="STANCE_AFTER_HS")
    t = round(t + DT, 6)
    upd = drive(buf, t, transitions=(transition("toe_off", t - 0.009, t, from_id=1.0, to_id=2.0, closed="stance", opens="swing"),), state="SWING_AFTER_TO")
    assert not upd.failed_closed
    assert upd.total_repair_transition_count == 0


def test_pending_onset_within_clock_drift_is_accepted_but_real_future_is_not() -> None:
    """Regression for the 1e-12 tolerance bug: env clock vs binary sample grid
    drift by ~1e-12 s inside an episode; a pending candidate whose onset is the
    boundary sample must not be a contract failure. A genuinely future onset
    (1 us) still fails closed."""
    buf = make_buffer()
    t = 17.076870983804135
    pending_drift = {"event": "heel_strike", "event_time_s": 17.07687098380528}
    upd = buf.update(time_s=t, knee_rad=-0.3, ankle_rad=0.1, accepted_transitions=[],
                     pending_transition=pending_drift, actor_state_name="WAIT_HS")
    assert not upd.failed_closed, upd.failure_reason
    buf2 = make_buffer()
    pending_future = {"event": "heel_strike", "event_time_s": t + 1e-6}
    upd2 = buf2.update(time_s=t, knee_rad=-0.3, ankle_rad=0.1, accepted_transitions=[],
                       pending_transition=pending_future, actor_state_name="WAIT_HS")
    assert upd2.failed_closed
    assert upd2.failure_reason == "invalid_pending_transition_time"
