"""L0 tests for the actor-FSM v3 mechanics (design 2026-08-21).

A = resync on desynchronisation, B = bounced-HS cancellation,
C = mid-cycle bootstrap (verified), plus compatibility invariants.
"""

from __future__ import annotations

import json
import random
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fsm_v3_scenarios import (  # noqa: E402
    DT,
    GOLDEN_SCRIPT,
    event,
    make_fsm,
    run_script,
    step,
)

GOLDEN = Path(__file__).resolve().parent / "fixtures" / "fsm_v2_golden_scripted_sequence.json"
V3 = dict(resync_enabled=True, hs_cancel_enabled=True)
DWELL_STEPS = 8  # resync_dwell_s 0.08 / DT
OBS_KEYS_V2 = (
    "phase_fsm_wait_hs", "phase_fsm_stance_after_hs", "phase_fsm_swing_after_to",
    "phase_expected_hs", "phase_expected_to", "phase_stance_elapsed_norm",
    "phase_swing_elapsed_norm", "phase_cycle_progress_credit",
)


def _journal(payload, name):
    return [t for t in payload["accepted_transitions_this_step"] if t["event"] == name]


# ---------------------------------------------------------------- compat

def test_flags_off_matches_v2_golden_on_every_v2_field() -> None:
    golden = json.loads(GOLDEN.read_text())
    payloads = run_script(make_fsm(), GOLDEN_SCRIPT)
    assert len(payloads) == len(golden)
    for i, (got, want) in enumerate(zip(payloads, golden)):
        for key, value in want.items():
            assert got[key] == value, f"step {i} key {key}: {got[key]!r} != {value!r}"


def test_flags_off_ignores_binary_contact_kwarg() -> None:
    golden = json.loads(GOLDEN.read_text())
    payloads = run_script(make_fsm(), GOLDEN_SCRIPT, pass_binary_contact=True)
    for got, want in zip(payloads, golden):
        for key, value in want.items():
            assert got[key] == value


def test_observation_schema_is_unchanged() -> None:
    fsm = make_fsm(**V3)
    fsm.reset_from_binary_baseline(time_s=1.0, in_contact=False)
    assert tuple(fsm.observation().keys()) == OBS_KEYS_V2


# ---------------------------------------------------------------- B: cancel

@pytest.mark.parametrize("bounce_steps", [1, 2, 3])
def test_bounced_hs_is_cancelled_and_state_stays_coherent(bounce_steps) -> None:
    # A *young* HS (it did not close a valid cycle): episode starts in a
    # partial stance, the partial TO opens the swing, the first real HS lands
    # and bounces within the minimum stance (stance = (bounce_steps+1)*10 ms).
    fsm = make_fsm(**V3)
    script = [("ground", 20), ("to",), ("air", 45),
              ("hs",), ("ground", int(bounce_steps)), ("to",)]
    payloads = run_script(fsm, script, start_contact=True, pass_binary_contact=True)
    last = payloads[-1]
    assert last["state_name"] == "SWING_AFTER_TO"
    assert last["invalid_event_type"] == "hs_bounce_cancelled"
    assert last["hs_cancelled_count"] == 1.0
    assert last["valid_hs_count"] == 0.0  # the bounced HS was revoked
    assert _journal(last, "heel_strike_cancelled")
    assert last["pending_cycle_credit"] == 0.0
    assert last["phase_failure_extra_penalty"] > 0.0
    # The next real HS is accepted normally (no double_hs lock).
    t = 1.0 + DT * len(payloads)
    for k in range(1, 41):
        step(fsm, round(t + DT * k, 6), contact=False, binary_contact=False)
    t_hs = round(t + DT * 41, 6)
    p = step(fsm, t_hs, events=(event("heel_strike", t_hs),),
             contact=True, binary_contact=True)
    assert p["state_name"] == "STANCE_AFTER_HS"
    assert p["invalid_event_this_step"] == 0.0


def test_bounce_without_previous_to_returns_to_wait_hs() -> None:
    fsm = make_fsm(**V3)
    payloads = run_script(fsm, [("air", 20), ("hs",), ("ground", 2), ("to",)],
                          pass_binary_contact=True)
    assert payloads[-1]["state_name"] == "WAIT_HS"
    assert payloads[-1]["valid_hs_count"] == 0.0


def test_bounce_after_hs_that_closed_a_valid_cycle_keeps_v2_path_then_resyncs() -> None:
    fsm = make_fsm(**V3)
    script = [("air", 20), ("hs",), ("ground", 60), ("to",), ("air", 45),
              ("hs",),                                  # closes a valid cycle
              ("ground", 2), ("to",)]                  # bounce on a cycle-closing HS
    payloads = run_script(fsm, script, pass_binary_contact=True)
    assert payloads[-1]["invalid_event_type"] == "to_too_early_after_hs"
    assert payloads[-1]["state_name"] == "STANCE_AFTER_HS"
    t = 1.0 + DT * len(payloads)
    for k in range(1, DWELL_STEPS + 2):
        p = step(fsm, round(t + DT * k, 6), contact=False, binary_contact=False)
    assert p["state_name"] == "SWING_AFTER_TO"
    assert p["resync_count"] == 1.0


# ---------------------------------------------------------------- A: resync

def _stance_with_rejected_to(fsm, valid_cycles: int = 1):
    script = [("air", 20), ("hs",), ("ground", 60), ("to",), ("air", 45), ("hs",)]
    script += [("ground", 60), ("to",), ("air", 45), ("hs",)] * (valid_cycles - 1)
    script += [("ground", 3), ("to",)]  # TO 40 ms after an HS that closed a cycle
    return run_script(fsm, script, pass_binary_contact=True)


def test_early_real_to_is_resynced_within_dwell_without_credit() -> None:
    fsm = make_fsm(resync_enabled=True, hs_cancel_enabled=False)
    payloads = _stance_with_rejected_to(fsm)
    rejected = payloads[-1]
    assert rejected["invalid_event_type"] == "to_too_early_after_hs"
    score_before = rejected["phase_event_progress_score"]
    t = 1.0 + DT * len(payloads)
    states = []
    for k in range(1, DWELL_STEPS + 1):
        p = step(fsm, round(t + DT * k, 6), contact=False, binary_contact=False)
        states.append(p["state_name"])
    assert states[-2] == "STANCE_AFTER_HS"  # one step before the dwell elapses
    assert states[-1] == "SWING_AFTER_TO"
    assert p["resync_event_this_step"] == 1.0
    assert p["resync_count"] == 1.0
    assert p["cycle_degraded"] == 1.0
    assert _journal(p, "toe_off_resync")[0]["segment_valid"] == 0.0
    assert p["phase_event_progress_score"] == 0.0 or p["phase_event_progress_score"] <= score_before
    assert p["valid_to_count"] == rejected["valid_to_count"]  # no credit, no count
    assert p["phase_failure_extra_penalty"] == 0.0  # no second penalty


def test_brief_air_below_dwell_does_not_resync() -> None:
    fsm = make_fsm(resync_enabled=True, hs_cancel_enabled=False)
    payloads = _stance_with_rejected_to(fsm)
    t = 1.0 + DT * len(payloads)
    for k in range(1, DWELL_STEPS - 2):
        p = step(fsm, round(t + DT * k, 6), contact=False, binary_contact=False)
    p = step(fsm, round(t + DT * (DWELL_STEPS - 2), 6), contact=True, binary_contact=True)
    assert p["state_name"] == "STANCE_AFTER_HS"
    assert p["resync_count"] == 0.0


def test_rejected_hs_in_swing_is_resynced_as_degraded_heel_strike() -> None:
    fsm = make_fsm(resync_enabled=True, hs_cancel_enabled=False)
    script = [("air", 20), ("hs",), ("ground", 60), ("to",), ("air", 10), ("hs",)]
    payloads = run_script(fsm, script, pass_binary_contact=True)
    assert payloads[-1]["invalid_event_type"] == "hs_too_early_after_to"
    assert payloads[-1]["state_name"] == "SWING_AFTER_TO"
    hs_count = payloads[-1]["valid_hs_count"]
    t = 1.0 + DT * len(payloads)
    for k in range(1, DWELL_STEPS + 1):
        p = step(fsm, round(t + DT * k, 6), contact=True, binary_contact=True)
    assert p["state_name"] == "STANCE_AFTER_HS"
    assert _journal(p, "heel_strike_resync")
    assert p["valid_hs_count"] == hs_count  # degraded HS is not counted
    assert p["cycle_degraded"] == 1.0


def test_degraded_cycle_earns_no_bonus_and_next_clean_cycle_does() -> None:
    fsm = make_fsm(resync_enabled=True, hs_cancel_enabled=False)
    payloads = _stance_with_rejected_to(fsm)
    t = 1.0 + DT * len(payloads)
    for k in range(1, DWELL_STEPS + 1):
        step(fsm, round(t + DT * k, 6), contact=False, binary_contact=False)
    t = round(t + DT * DWELL_STEPS, 6)
    for k in range(1, 40):
        step(fsm, round(t + DT * k, 6), contact=False, binary_contact=False)
    t = round(t + DT * 40, 6)
    closing = step(fsm, t, events=(event("heel_strike", t),), contact=True, binary_contact=True)
    assert closing["invalid_event_this_step"] == 0.0  # accepted, published
    assert closing["cycle_completed_this_step"] == 0.0
    assert closing["phase_cycle_complete_bonus"] == 0.0
    assert closing["cycle_reject_reason"] == "cycle_degraded"
    assert closing["cycle_degraded"] == 0.0
    # clean follow-up cycle
    for k in range(1, 61):
        step(fsm, round(t + DT * k, 6), contact=True, binary_contact=True)
    t = round(t + DT * 61, 6)
    step(fsm, t, events=(event("toe_off", t),), contact=False, binary_contact=False)
    for k in range(1, 46):
        step(fsm, round(t + DT * k, 6), contact=False, binary_contact=False)
    t = round(t + DT * 46, 6)
    clean = step(fsm, t, events=(event("heel_strike", t),), contact=True, binary_contact=True)
    assert clean["cycle_completed_this_step"] == 1.0
    assert clean["phase_cycle_complete_bonus"] > 0.0


def test_timeouts_still_fire_when_contact_never_contradicts() -> None:
    fsm = make_fsm(**V3, stance_hard_timeout_s=0.5)
    payloads = run_script(fsm, [("air", 5), ("hs",), ("ground", 60)], pass_binary_contact=True)
    assert payloads[-1]["state_name"] == "TIMEOUT"
    assert payloads[-1]["resync_count"] == 0.0


def test_resync_never_runs_in_timeout_state() -> None:
    fsm = make_fsm(**V3, stance_hard_timeout_s=0.3)
    payloads = run_script(fsm, [("air", 5), ("hs",), ("ground", 40), ("air", 30)],
                          pass_binary_contact=True)
    assert payloads[-1]["state_name"] == "TIMEOUT"
    assert payloads[-1]["resync_count"] == 0.0


# ---------------------------------------------------------------- C: bootstrap

@pytest.mark.parametrize("start_contact,first_event,lead", [
    (True, "to", 1), (True, "to", 10), (True, "to", 40), (True, "to", 90),
    (False, "hs", 1), (False, "hs", 10), (False, "hs", 40), (False, "hs", 90),
])
def test_mid_cycle_start_accepts_first_physical_event(start_contact, first_event, lead) -> None:
    fsm = make_fsm(**V3)
    hold = "ground" if start_contact else "air"
    payloads = run_script(fsm, [(hold, lead), (first_event,)],
                          start_contact=start_contact, pass_binary_contact=True)
    last = payloads[-1]
    assert last["invalid_event_this_step"] == 0.0
    assert last["state_name"] == ("SWING_AFTER_TO" if first_event == "to" else "STANCE_AFTER_HS")
    assert last["resync_count"] == 0.0


# ---------------------------------------------------------------- fuzz

def test_fuzz_random_sequences_keep_invariants() -> None:
    rng = random.Random(20260821)
    for seed in range(300):
        fsm = make_fsm(**V3)
        start_contact = rng.random() < 0.5
        fsm.reset_from_binary_baseline(time_s=1.0, in_contact=start_contact)
        t = 1.0
        contact = start_contact
        last_counts = (0.0, 0.0)
        for _ in range(120):
            t = round(t + DT, 6)
            r = rng.random()
            events = ()
            if r < 0.08:
                events = (event("heel_strike", t),)
                contact = True
            elif r < 0.16:
                events = (event("toe_off", t),)
                contact = False
            elif r < 0.22:
                contact = not contact
            p = step(fsm, t, events=events, contact=contact, binary_contact=contact)
            json.dumps(p, allow_nan=False)
            assert p["state_name"] in {"WAIT_HS", "STANCE_AFTER_HS", "SWING_AFTER_TO", "TIMEOUT"}
            counts = (p["resync_count"], p["hs_cancelled_count"])
            assert counts >= last_counts
            last_counts = counts
            if p["state_name"] == "TIMEOUT":
                break
