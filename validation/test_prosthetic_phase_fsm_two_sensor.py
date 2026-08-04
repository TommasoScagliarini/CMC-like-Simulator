"""Focused tests for the two-sensor input guards of ProstheticPhaseFSM.

These tests exercise the pure-Python FSM only.  End-to-end sensor geometry,
OpenSim force isolation, prescribed replay, and frozen-policy robustness remain
separate validation gates.
"""

from __future__ import annotations

import math
import sys
from contextlib import contextmanager
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
TRAJECTORY_GENERATOR = REPO / "Trajectory Generator"
if str(TRAJECTORY_GENERATOR) not in sys.path:
    sys.path.insert(0, str(TRAJECTORY_GENERATOR))

from prosthetic_phase_fsm import (  # noqa: E402
    STANCE_AFTER_HS,
    SWING_AFTER_TO,
    WAIT_HS,
    ProstheticPhaseFSM,
    ProstheticPhaseFSMConfig,
)


ACTOR_OBSERVATION_KEYS = (
    "phase_fsm_wait_hs",
    "phase_fsm_stance_after_hs",
    "phase_fsm_swing_after_to",
    "phase_expected_hs",
    "phase_expected_to",
    "phase_stance_elapsed_norm",
    "phase_swing_elapsed_norm",
    "phase_cycle_progress_credit",
)

TESTS = []


def _test(func):
    TESTS.append(func)
    return func


@contextmanager
def _raises(exception_type, message_fragment: str):
    try:
        yield
    except exception_type as exc:
        assert message_fragment in str(exc), str(exc)
    else:
        raise AssertionError(f"Expected {exception_type.__name__}")


def _fsm(event_source: str = "two_sensor") -> ProstheticPhaseFSM:
    return ProstheticPhaseFSM(
        ProstheticPhaseFSMConfig(
            event_source=event_source,
            sensor_on_threshold_n=5.0,
            sensor_off_threshold_n=2.0,
            sensor_dwell_s=0.02,
            min_stance_duration_s=0.05,
            min_swing_duration_s=0.20,
        )
    )


def _tick(
    fsm: ProstheticPhaseFSM,
    time_s: float,
    heel_n: float,
    toe_n: float,
    *,
    events=(),
):
    return fsm.update(
        time_s=time_s,
        events=events,
        normal_force_bw=(heel_n + toe_n) / 100.0,
        in_contact=bool(heel_n >= 5.0 or toe_n >= 5.0),
        heel_normal_force_n=heel_n,
        toe_normal_force_n=toe_n,
    )


def _arm_from_clear(fsm: ProstheticPhaseFSM) -> None:
    _tick(fsm, 0.00, 0.0, 0.0)
    payload = _tick(fsm, 0.02, 0.0, 0.0)
    assert payload["sensor_hs_armed"] == 1.0


def _event_names(payload) -> list[str]:
    return [str(item["event"]) for item in payload["sensor_events_this_step"]]


def _edge_names(payload) -> list[tuple[str, str]]:
    return [
        (str(item["sensor"]), str(item["edge"]))
        for item in payload["sensor_edges_this_step"]
    ]


@_test
def test_config_and_two_sensor_inputs_fail_closed() -> None:
    with _raises(ValueError, "event_source"):
        ProstheticPhaseFSMConfig(event_source="implicit")
    with _raises(ValueError, "lower"):
        ProstheticPhaseFSMConfig(
            sensor_on_threshold_n=5.0,
            sensor_off_threshold_n=5.0,
        )

    fsm = _fsm()
    with _raises(ValueError, "heel_normal_force_n"):
        fsm.update(time_s=0.0, normal_force_bw=0.0, in_contact=False)
    with _raises(ValueError, "legacy left HS/TO"):
        _tick(
            fsm,
            0.0,
            0.0,
            0.0,
            events=({"side": "left", "event": "heel_strike", "time": 0.0},),
        )
    with _raises(ValueError, "finite non-negative"):
        _tick(fsm, 0.0, float("nan"), 0.0)


@_test
def test_actor_observation_contract_is_identical_in_all_modes() -> None:
    observations = {
        mode: _fsm(mode).observation()
        for mode in ("legacy_events", "shadow", "two_sensor")
    }
    for observation in observations.values():
        assert tuple(observation) == ACTOR_OBSERVATION_KEYS
    assert observations["legacy_events"] == observations["shadow"]
    assert observations["legacy_events"] == observations["two_sensor"]


@_test
def test_legacy_event_mode_remains_backward_compatible() -> None:
    fsm = _fsm("legacy_events")
    first = fsm.update(
        time_s=0.0,
        events=({"side": "left", "event": "heel_strike", "time": 0.0},),
        normal_force_bw=0.7,
        in_contact=True,
    )
    toe_off = fsm.update(
        time_s=0.6,
        events=({"side": "left", "event": "toe_off", "time": 0.6},),
        normal_force_bw=0.0,
        in_contact=False,
    )
    second = fsm.update(
        time_s=1.4,
        events=({"side": "left", "event": "heel_strike", "time": 1.4},),
        normal_force_bw=0.7,
        in_contact=True,
    )

    assert first["state_id"] == float(STANCE_AFTER_HS)
    assert toe_off["state_id"] == float(SWING_AFTER_TO)
    assert second["valid_hs_count"] == 2.0
    assert second["valid_to_count"] == 1.0
    assert second["valid_cycle_count"] == 1.0
    assert math.isclose(second["last_period_s"], 1.4, abs_tol=1e-12)
    assert math.isclose(second["phase_event_progress_score"], 0.8, abs_tol=1e-12)


@_test
def test_heel_opens_hs_and_both_sensors_clear_close_toe_off() -> None:
    fsm = _fsm()
    _arm_from_clear(fsm)

    _tick(fsm, 0.10, 10.0, 0.0)
    heel_strike = _tick(fsm, 0.12, 10.0, 0.0)
    assert _event_names(heel_strike) == ["heel_strike"]
    assert heel_strike["state_id"] == float(STANCE_AFTER_HS)
    assert heel_strike["valid_hs_count"] == 1.0
    accepted_hs = heel_strike["accepted_transitions_this_step"][0]
    assert accepted_hs["event"] == "heel_strike"
    assert "startup_contact" not in accepted_hs
    assert math.isclose(accepted_hs["event_time_s"], 0.10, abs_tol=1e-12)

    _tick(fsm, 0.15, 10.0, 10.0)
    _tick(fsm, 0.17, 10.0, 10.0)
    _tick(fsm, 0.40, 0.0, 10.0)
    heel_off_only = _tick(fsm, 0.42, 0.0, 10.0)
    assert heel_off_only["state_id"] == float(STANCE_AFTER_HS)
    assert heel_off_only["valid_to_count"] == 0.0

    _tick(fsm, 0.60, 0.0, 0.0)
    toe_off = _tick(fsm, 0.62, 0.0, 0.0)
    assert _event_names(toe_off) == ["toe_off"]
    assert toe_off["state_id"] == float(SWING_AFTER_TO)
    assert toe_off["valid_to_count"] == 1.0
    accepted_to = toe_off["accepted_transitions_this_step"][0]
    assert accepted_to["event"] == "toe_off"
    assert math.isclose(accepted_to["event_time_s"], 0.60, abs_tol=1e-12)

    _tick(fsm, 0.90, 10.0, 0.0)
    next_hs = _tick(fsm, 0.92, 10.0, 0.0)
    assert next_hs["valid_hs_count"] == 2.0
    assert next_hs["valid_cycle_count"] == 1.0
    assert math.isclose(next_hs["last_period_s"], 0.80, abs_tol=1e-12)


@_test
def test_toe_only_is_not_hs_but_later_heel_contact_is_hs() -> None:
    fsm = _fsm()
    _arm_from_clear(fsm)

    _tick(fsm, 0.10, 0.0, 10.0)
    toe_contact = _tick(fsm, 0.12, 0.0, 10.0)
    assert _event_names(toe_contact) == ["forefoot_first"]
    assert _edge_names(toe_contact) == [("toe", "contact_on")]
    assert toe_contact["state_id"] == float(WAIT_HS)
    assert toe_contact["valid_hs_count"] == 0.0

    _tick(fsm, 0.20, 10.0, 10.0)
    late_heel = _tick(fsm, 0.22, 10.0, 10.0)
    assert _event_names(late_heel) == ["heel_strike"]
    assert _edge_names(late_heel) == [("heel", "contact_on")]
    assert late_heel["state_id"] == float(STANCE_AFTER_HS)
    assert late_heel["valid_hs_count"] == 1.0
    sensor_event = late_heel["sensor_events_this_step"][0]
    assert sensor_event["forefoot_contact_preceded_heel"] is True


@_test
def test_simultaneous_contact_is_one_heel_strike() -> None:
    fsm = _fsm()
    _arm_from_clear(fsm)
    _tick(fsm, 0.10, 10.0, 10.0)
    payload = _tick(fsm, 0.12, 10.0, 10.0)
    assert _event_names(payload) == ["heel_strike"]
    assert payload["valid_hs_count"] == 1.0
    assert len(payload["accepted_transitions_this_step"]) == 1


@_test
def test_debounce_hysteresis_and_stance_recontact_do_not_duplicate_events() -> None:
    fsm = _fsm()
    _arm_from_clear(fsm)

    _tick(fsm, 0.10, 10.0, 0.0)
    short_bump = _tick(fsm, 0.11, 0.0, 0.0)
    assert short_bump["valid_hs_count"] == 0.0

    _tick(fsm, 0.20, 10.0, 0.0)
    hs = _tick(fsm, 0.22, 10.0, 0.0)
    assert hs["valid_hs_count"] == 1.0

    # Between OFF and ON thresholds, the active heel latch remains active.
    between_thresholds = _tick(fsm, 0.30, 3.0, 0.0)
    assert between_thresholds["sensor_heel_contact"] == 1.0
    assert between_thresholds["valid_to_count"] == 0.0

    _tick(fsm, 0.40, 0.0, 0.0)
    toe_off = _tick(fsm, 0.42, 0.0, 0.0)
    assert toe_off["valid_to_count"] == 1.0

    # Heel chatter shorter than dwell while in swing creates no new HS.
    _tick(fsm, 0.50, 10.0, 0.0)
    chatter = _tick(fsm, 0.51, 0.0, 0.0)
    assert chatter["valid_hs_count"] == 1.0


@_test
def test_startup_heel_only_is_one_debounced_hs() -> None:
    fsm = _fsm()

    first = _tick(fsm, 0.00, 10.0, 0.0)
    margin = _tick(fsm, 0.01, 10.0, 0.0)
    confirmed = _tick(fsm, 0.02, 10.0, 0.0)

    assert first["state_id"] == float(WAIT_HS)
    assert margin["state_id"] == float(WAIT_HS)
    assert first["sensor_events_this_step"] == []
    assert margin["sensor_events_this_step"] == []
    assert _event_names(confirmed) == ["heel_strike"]
    assert confirmed["state_id"] == float(STANCE_AFTER_HS)
    assert confirmed["valid_hs_count"] == 1.0
    assert confirmed["sensor_partial_stance_active"] == 0.0

    sensor_event = confirmed["sensor_events_this_step"][0]
    assert sensor_event["startup_contact"] is True
    assert sensor_event["startup_pattern"] == "heel_on_toe_off"
    assert math.isclose(sensor_event["time"], 0.0, abs_tol=1e-12)
    assert math.isclose(sensor_event["confirmed_time"], 0.02, abs_tol=1e-12)
    accepted = confirmed["accepted_transitions_this_step"]
    assert len(accepted) == 1
    assert accepted[0]["startup_contact"] == 1.0
    assert math.isclose(accepted[0]["event_time_s"], 0.0, abs_tol=1e-12)

    no_duplicate = _tick(fsm, 0.04, 10.0, 0.0)
    assert no_duplicate["sensor_events_this_step"] == []
    assert no_duplicate["accepted_transitions_this_step"] == []
    assert no_duplicate["valid_hs_count"] == 1.0


@_test
def test_startup_heel_only_margin_rejects_transient_contact() -> None:
    fsm = _fsm()

    _tick(fsm, 0.00, 10.0, 0.0)
    transient = _tick(fsm, 0.01, 0.0, 0.0)
    clear = _tick(fsm, 0.03, 0.0, 0.0)

    assert transient["valid_hs_count"] == 0.0
    assert transient["sensor_events_this_step"] == []
    assert clear["valid_hs_count"] == 0.0
    assert clear["sensor_hs_armed"] == 1.0


@_test
def test_startup_toe_on_before_margin_cancels_heel_only_hs() -> None:
    fsm = _fsm()

    _tick(fsm, 0.00, 10.0, 0.0)
    _tick(fsm, 0.01, 10.0, 10.0)
    bootstrap = _tick(fsm, 0.02, 10.0, 10.0)

    assert bootstrap["state_id"] == float(STANCE_AFTER_HS)
    assert bootstrap["sensor_partial_stance_active"] == 1.0
    assert bootstrap["valid_hs_count"] == 0.0
    assert bootstrap["accepted_transitions_this_step"] == []


@_test
def test_startup_toe_only_is_partial_stance_not_hs() -> None:
    fsm = _fsm()

    _tick(fsm, 0.00, 0.0, 10.0)
    bootstrap = _tick(fsm, 0.02, 0.0, 10.0)

    assert bootstrap["state_id"] == float(STANCE_AFTER_HS)
    assert bootstrap["sensor_partial_stance_active"] == 1.0
    assert bootstrap["valid_hs_count"] == 0.0
    assert bootstrap["accepted_transitions_this_step"] == []


@_test
def test_midstance_bootstrap_creates_no_synthetic_hs_or_credit() -> None:
    fsm = _fsm()
    _tick(fsm, 0.00, 10.0, 10.0)
    bootstrap = _tick(fsm, 0.02, 10.0, 10.0)

    assert bootstrap["state_id"] == float(STANCE_AFTER_HS)
    assert bootstrap["sensor_partial_stance_active"] == 1.0
    assert bootstrap["valid_hs_count"] == 0.0
    assert bootstrap["last_valid_hs_time_s"] == -1.0
    assert bootstrap["accepted_transitions_this_step"] == []
    assert bootstrap["phase_event_progress_score"] == 0.0
    assert bootstrap["pending_cycle_credit"] == 0.0
    assert bootstrap["cycle_progress_credit"] == 0.0

    # Even when reset occurs just before TO, the partial segment must close;
    # its unobserved pre-reset duration cannot be checked against the full
    # min-stance gate.
    _tick(fsm, 0.03, 0.0, 0.0)
    toe_off = _tick(fsm, 0.05, 0.0, 0.0)
    assert toe_off["state_id"] == float(SWING_AFTER_TO)
    assert toe_off["valid_to_count"] == 1.0
    assert toe_off["phase_event_progress_score"] == 0.0
    assert toe_off["pending_cycle_credit"] == 0.0
    assert toe_off["accepted_transitions_this_step"][0]["segment_valid"] == 0.0

    _tick(fsm, 0.30, 10.0, 0.0)
    first_complete_hs = _tick(fsm, 0.32, 10.0, 0.0)
    assert first_complete_hs["valid_hs_count"] == 1.0
    assert first_complete_hs["valid_cycle_count"] == 0.0
    assert first_complete_hs["phase_cycle_complete_bonus"] == 0.0
    assert first_complete_hs["phase_event_progress_score"] == 0.1


@_test
def test_shadow_updates_sensor_diagnostics_but_applies_only_legacy_event() -> None:
    fsm = _fsm("shadow")
    _arm_from_clear(fsm)
    _tick(fsm, 0.10, 10.0, 0.0)
    payload = _tick(
        fsm,
        0.12,
        10.0,
        0.0,
        events=({"side": "left", "event": "heel_strike", "time": 0.10},),
    )

    assert _event_names(payload) == ["heel_strike"]
    assert payload["valid_hs_count"] == 1.0
    assert len(payload["accepted_transitions_this_step"]) == 1
    assert payload["state_id"] == float(STANCE_AFTER_HS)


@_test
def test_shadow_raw_edges_remain_available_when_legacy_state_masks_counterfactual() -> None:
    fsm = _fsm("shadow")
    _arm_from_clear(fsm)

    # Legacy incorrectly calls the forefoot-only onset an HS and therefore
    # changes the active state.  The later physical heel edge must still be
    # present as a state-independent diagnostic for offline replay.
    _tick(fsm, 0.10, 0.0, 10.0)
    toe_only = _tick(
        fsm,
        0.12,
        0.0,
        10.0,
        events=({"side": "left", "event": "heel_strike", "time": 0.10},),
    )
    assert _edge_names(toe_only) == [("toe", "contact_on")]
    assert toe_only["state_id"] == float(STANCE_AFTER_HS)

    _tick(fsm, 0.20, 10.0, 10.0)
    heel = _tick(fsm, 0.22, 10.0, 10.0)
    assert _edge_names(heel) == [("heel", "contact_on")]
    assert _event_names(heel) == []


@_test
def test_shadow_bootstrap_is_diagnostic_only() -> None:
    fsm = _fsm("shadow")
    _tick(fsm, 0.00, 10.0, 10.0)
    payload = _tick(fsm, 0.02, 10.0, 10.0)
    assert _event_names(payload) == ["partial_stance_bootstrap"]
    assert payload["state_id"] == float(WAIT_HS)
    assert payload["accepted_transitions_this_step"] == []
    assert payload["valid_hs_count"] == 0.0


@_test
def test_shadow_startup_heel_only_is_diagnostic_only() -> None:
    fsm = _fsm("shadow")
    _tick(fsm, 0.00, 10.0, 0.0)
    payload = _tick(fsm, 0.02, 10.0, 0.0)

    assert _event_names(payload) == ["heel_strike"]
    assert payload["sensor_events_this_step"][0]["startup_contact"] is True
    assert payload["state_id"] == float(WAIT_HS)
    assert payload["accepted_transitions_this_step"] == []
    assert payload["valid_hs_count"] == 0.0


@_test
def test_shadow_startup_candidate_preserves_legacy_active_fsm() -> None:
    legacy = _fsm("legacy_events")
    shadow = _fsm("shadow")
    sequence = (
        (0.00, 10.0, 0.0, ()),
        (
            0.02,
            10.0,
            0.0,
            ({"side": "left", "event": "heel_strike", "time": 0.0},),
        ),
        (
            0.60,
            0.0,
            0.0,
            ({"side": "left", "event": "toe_off", "time": 0.60},),
        ),
        (0.62, 0.0, 0.0, ()),
        (0.90, 10.0, 0.0, ()),
        (
            0.92,
            10.0,
            0.0,
            ({"side": "left", "event": "heel_strike", "time": 0.90},),
        ),
    )
    active_keys = (
        "state_id",
        "expected_next_event",
        "valid_hs_count",
        "valid_to_count",
        "valid_cycle_count",
        "invalid_event_count",
        "last_valid_hs_time_s",
        "last_valid_to_time_s",
        "accepted_transitions_this_step",
    )
    saw_startup_candidate = False
    for time_s, heel_n, toe_n, events in sequence:
        legacy_payload = _tick(
            legacy,
            time_s,
            heel_n,
            toe_n,
            events=events,
        )
        shadow_payload = _tick(
            shadow,
            time_s,
            heel_n,
            toe_n,
            events=events,
        )
        saw_startup_candidate = saw_startup_candidate or any(
            bool(event.get("startup_contact", False))
            for event in shadow_payload["sensor_events_this_step"]
        )
        assert shadow.observation() == legacy.observation()
        for key in active_keys:
            assert shadow_payload[key] == legacy_payload[key], key

    assert saw_startup_candidate


if __name__ == "__main__":
    for test in TESTS:
        test()
        print(f"[PASS] {test.__name__}")
    print(f"ALL TWO-SENSOR FSM TESTS PASSED ({len(TESTS)})")
