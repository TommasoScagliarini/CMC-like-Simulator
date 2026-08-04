#!/usr/bin/env python3
"""Tests for fail-closed reconstruction of recorded FSM transitions."""

from __future__ import annotations

import copy

import replay_completed_segment_morphology as replay


def _row(
    time_s: float,
    *,
    state: int,
    hs: int,
    toe_off: int,
    cycles: int,
    stance_elapsed: float = 0.0,
    swing_elapsed: float = 0.0,
    period: float = 0.0,
    stance_fraction: float = 0.0,
) -> dict:
    return {
        "step": 1,
        "time": time_s,
        "prosthetic_state": {
            "pros_knee_angle_served_ref": -0.2,
            "pros_ankle_angle_served_ref": 0.1,
        },
        "reward_terms": {
            "phase_fsm_state_id": float(state),
            "phase_valid_hs_count": float(hs),
            "phase_valid_to_count": float(toe_off),
            "phase_valid_cycle_count": float(cycles),
            "phase_stance_elapsed_s": stance_elapsed,
            "phase_swing_elapsed_s": swing_elapsed,
            "phase_last_period_s": period,
            "phase_last_stance_fraction": stance_fraction,
            "phase_timeout_exceeded": 0.0,
            "phase_timeout_side": 0.0,
        },
    }


def _valid_rows() -> list[dict]:
    # Physical anchors are HS=0.15, TO=0.35, next HS=0.55.  Confirmation is
    # delayed by 0.05 s in all three cases.
    rows = [
        _row(0.10, state=0, hs=0, toe_off=0, cycles=0),
        _row(0.20, state=1, hs=1, toe_off=0, cycles=0, stance_elapsed=0.05),
        _row(0.30, state=1, hs=1, toe_off=0, cycles=0, stance_elapsed=0.15),
        _row(0.40, state=2, hs=1, toe_off=1, cycles=0, swing_elapsed=0.05),
        _row(0.50, state=2, hs=1, toe_off=1, cycles=0, swing_elapsed=0.15),
        _row(
            0.60,
            state=1,
            hs=2,
            toe_off=1,
            cycles=1,
            stance_elapsed=0.05,
            period=0.40,
            stance_fraction=0.50,
        ),
    ]
    for index, row in enumerate(rows, start=1):
        row["step"] = index
    return rows


def test_reconstruction_uses_physical_event_timestamps() -> None:
    journals, summary = replay.recover_transition_journal(_valid_rows())
    events = [event for journal in journals for event in journal]
    assert [event["event"] for event in events] == [
        "heel_strike",
        "toe_off",
        "heel_strike",
    ]
    expected_times = (0.15, 0.35, 0.55)
    assert all(
        abs(event["event_time_s"] - expected) < 1.0e-12
        for event, expected in zip(events, expected_times)
    )
    assert abs(events[1]["segment_start_time_s"] - 0.15) < 1.0e-12
    assert abs(events[2]["segment_start_time_s"] - 0.35) < 1.0e-12
    assert summary["completed_cycle_count"] == 1


def test_counter_skip_is_rejected() -> None:
    rows = _valid_rows()
    rows[1]["reward_terms"]["phase_valid_hs_count"] = 2.0
    try:
        replay.recover_transition_journal(rows)
    except ValueError as exc:
        assert "counter skipped" in str(exc)
    else:
        raise AssertionError("counter skip was accepted")


def test_simultaneous_hs_to_increment_is_rejected() -> None:
    rows = _valid_rows()
    rows[1]["reward_terms"]["phase_valid_to_count"] = 1.0
    try:
        replay.recover_transition_journal(rows)
    except ValueError as exc:
        assert "both increment" in str(exc)
    else:
        raise AssertionError("ambiguous simultaneous counters were accepted")


def test_elapsed_clock_drift_is_rejected() -> None:
    rows = copy.deepcopy(_valid_rows())
    rows[2]["reward_terms"]["phase_stance_elapsed_s"] = 0.14
    try:
        replay.recover_transition_journal(rows)
    except ValueError as exc:
        assert "elapsed clock disagrees" in str(exc)
    else:
        raise AssertionError("inconsistent elapsed clock was accepted")


TESTS = (
    test_reconstruction_uses_physical_event_timestamps,
    test_counter_skip_is_rejected,
    test_simultaneous_hs_to_increment_is_rejected,
    test_elapsed_clock_drift_is_rejected,
)


if __name__ == "__main__":
    for test in TESTS:
        test()
        print(f"[PASS] {test.__name__}")
    print(f"ALL RECORDED FSM REPLAY TESTS PASSED ({len(TESTS)})")
