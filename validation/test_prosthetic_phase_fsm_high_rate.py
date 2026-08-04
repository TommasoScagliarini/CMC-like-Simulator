"""Synthetic contract tests for 1 ms detector transport and FSM batching."""

from __future__ import annotations

import math
import sys
from contextlib import contextmanager
from pathlib import Path
from types import MethodType, SimpleNamespace


REPO = Path(__file__).resolve().parents[1]
TRAJECTORY_GENERATOR = REPO / "Trajectory Generator"
for path in (REPO, TRAJECTORY_GENERATOR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from prosthetic_phase_fsm import (  # noqa: E402
    ProstheticPhaseFSM,
    ProstheticPhaseFSMConfig,
)
from simulation_runner import (  # noqa: E402
    PhaseSensorSamplingError,
    SimulationRunner,
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


def _fsm() -> ProstheticPhaseFSM:
    return ProstheticPhaseFSM(
        ProstheticPhaseFSMConfig(
            event_source="two_sensor",
            detector_sample_dt_s=0.001,
            sensor_on_threshold_n=0.5,
            sensor_off_threshold_n=0.25,
            sensor_dwell_s=0.03,
            min_stance_duration_s=0.05,
            min_swing_duration_s=0.20,
            min_stance_contact_fraction=0.0,
            min_stance_load_bw_s=0.0,
            min_cycle_knee_excursion_rad=0.0,
        )
    )


def _sample(time_s: float, heel_n: float, toe_n: float) -> dict[str, float]:
    return {
        "time_s": float(time_s),
        "left_heel_normal_n": float(heel_n),
        "left_toe_normal_n": float(toe_n),
    }


def _force_sequence() -> list[dict[str, float]]:
    samples = []
    for index in range(1, 501):
        time_s = index * 0.001
        heel = 0.0
        toe = 0.0
        if 0.041 <= time_s <= 0.150:
            heel = 1.0
        if 0.082 <= time_s <= 0.190:
            toe = 1.0
        if 0.451 <= time_s <= 0.500:
            heel = 1.0
        samples.append(_sample(time_s, heel, toe))
    return samples


def _without_delivery(items):
    return [
        {
            key: value
            for key, value in item.items()
            if key != "delivered_time_s"
        }
        for item in items
    ]


@_test
def test_sequential_and_policy_batches_have_exact_detector_parity() -> None:
    samples = _force_sequence()
    sequential = _fsm()
    batch = _fsm()
    sequential_events = []
    sequential_edges = []
    sequential_transitions = []
    batch_events = []
    batch_edges = []
    batch_transitions = []

    for sample in samples:
        payload = sequential.update(
            time_s=sample["time_s"],
            normal_force_bw=(
                sample["left_heel_normal_n"]
                + sample["left_toe_normal_n"]
            ),
            in_contact=bool(
                sample["left_heel_normal_n"] >= 0.5
                or sample["left_toe_normal_n"] >= 0.5
            ),
            heel_normal_force_n=sample["left_heel_normal_n"],
            toe_normal_force_n=sample["left_toe_normal_n"],
        )
        sequential_events.extend(payload["sensor_events_this_step"])
        sequential_edges.extend(payload["sensor_edges_this_step"])
        sequential_transitions.extend(
            payload["accepted_transitions_this_step"]
        )

    for offset in range(0, len(samples), 10):
        policy_samples = samples[offset : offset + 10]
        policy_time = policy_samples[-1]["time_s"]
        payload = batch.update_policy_step(
            time_s=policy_time,
            previous_time_s=offset * 0.001,
            sensor_samples=policy_samples,
            normal_force_bw=(
                policy_samples[-1]["left_heel_normal_n"]
                + policy_samples[-1]["left_toe_normal_n"]
            ),
            in_contact=bool(
                policy_samples[-1]["left_heel_normal_n"] >= 0.5
                or policy_samples[-1]["left_toe_normal_n"] >= 0.5
            ),
        )
        assert payload["sensor_batch_sample_count"] == 10.0
        batch_events.extend(payload["sensor_events_this_step"])
        batch_edges.extend(payload["sensor_edges_this_step"])
        batch_transitions.extend(payload["accepted_transitions_this_step"])

    assert _without_delivery(batch_events) == _without_delivery(
        sequential_events
    )
    assert _without_delivery(batch_edges) == _without_delivery(
        sequential_edges
    )
    assert _without_delivery(batch_transitions) == _without_delivery(
        sequential_transitions
    )
    assert [item["event"] for item in batch_transitions] == [
        "heel_strike",
        "toe_off",
        "heel_strike",
    ]
    assert batch.valid_hs_count == sequential.valid_hs_count == 2
    assert batch.valid_to_count == sequential.valid_to_count == 1
    assert batch.valid_cycle_count == sequential.valid_cycle_count == 1
    for item in batch_events + batch_edges + batch_transitions:
        assert item["event_time_s"] <= item["confirmed_time_s"] + 1e-12
        assert item["confirmed_time_s"] <= item["delivered_time_s"] + 1e-12
        assert item["delivered_time_s"] - item["confirmed_time_s"] <= 0.01 + 1e-12


def _run_impulse(duration_s: float) -> list[dict]:
    fsm = _fsm()
    events = []
    onset = 0.041
    end = onset + duration_s
    samples = [
        _sample(
            index * 0.001,
            1.0 if onset - 1e-12 <= index * 0.001 <= end + 1e-12 else 0.0,
            0.0,
        )
        for index in range(1, 111)
    ]
    for offset in range(0, len(samples), 10):
        payload = fsm.update_policy_step(
            time_s=(offset + 10) * 0.001,
            previous_time_s=offset * 0.001,
            sensor_samples=samples[offset : offset + 10],
            normal_force_bw=0.0,
            in_contact=False,
        )
        events.extend(payload["sensor_events_this_step"])
    return [event for event in events if event["event"] == "heel_strike"]


@_test
def test_29_ms_is_rejected_and_30_to_36_ms_are_confirmed() -> None:
    assert _run_impulse(0.029) == []
    for duration_s in (0.030, 0.031, 0.035, 0.036):
        events = _run_impulse(duration_s)
        assert len(events) == 1, duration_s
        event = events[0]
        assert math.isclose(event["event_time_s"], 0.041, abs_tol=1e-12)
        assert math.isclose(event["confirmed_time_s"], 0.071, abs_tol=1e-12)


@_test
def test_batch_boundaries_and_bad_samples_fail_before_state_mutation() -> None:
    fsm = _fsm()
    first = [_sample(index * 0.001, 0.0, 0.0) for index in range(1, 11)]
    payload = fsm.update_policy_step(
        time_s=0.010,
        previous_time_s=0.0,
        sensor_samples=first,
        normal_force_bw=0.0,
        in_contact=False,
    )
    assert payload["sensor_batch_previous_time_s"] == 0.0
    before = fsm.payload()

    valid_second = [
        _sample(index * 0.001, 0.0, 0.0) for index in range(11, 21)
    ]
    malformed_batches = []
    malformed_batches.append((valid_second[:-1], "Detector"))
    duplicate = [dict(item) for item in valid_second]
    duplicate[4]["time_s"] = duplicate[3]["time_s"]
    malformed_batches.append((duplicate, "Detector"))
    nonmonotonic = [dict(item) for item in valid_second]
    nonmonotonic[4]["time_s"] = 0.012
    malformed_batches.append((nonmonotonic, "Detector"))
    nonfinite = [dict(item) for item in valid_second]
    nonfinite[4]["left_heel_normal_n"] = float("nan")
    malformed_batches.append((nonfinite, "finite non-negative"))

    for malformed, message_fragment in malformed_batches:
        with _raises(ValueError, message_fragment):
            fsm.update_policy_step(
                time_s=0.020,
                previous_time_s=0.010,
                sensor_samples=malformed,
                normal_force_bw=0.0,
                in_contact=False,
            )
        assert fsm.payload() == before

    accepted = fsm.update_policy_step(
        time_s=0.020,
        sensor_samples=valid_second,
        normal_force_bw=0.0,
        in_contact=False,
    )
    assert accepted["sensor_batch_sample_count"] == 10.0
    with _raises(ValueError, "discontinuous"):
        fsm.update_policy_step(
            time_s=0.030,
            previous_time_s=0.010,
            sensor_samples=[
                _sample(index * 0.001, 0.0, 0.0)
                for index in range(21, 31)
            ],
            normal_force_bw=0.0,
            in_contact=False,
        )


@_test
def test_shadow_timeout_order_is_identical_to_scalar_legacy() -> None:
    common = dict(
        detector_sample_dt_s=0.001,
        sensor_on_threshold_n=0.5,
        sensor_off_threshold_n=0.25,
        sensor_dwell_s=0.03,
        min_stance_duration_s=0.0,
        min_swing_duration_s=0.0,
        stance_hard_timeout_s=0.015,
        min_stance_contact_fraction=0.0,
        min_stance_load_bw_s=0.0,
        min_cycle_knee_excursion_rad=0.0,
    )
    legacy = ProstheticPhaseFSM(
        ProstheticPhaseFSMConfig(event_source="legacy_events", **common)
    )
    shadow = ProstheticPhaseFSM(
        ProstheticPhaseFSMConfig(event_source="shadow", **common)
    )

    hs = ({"side": "left", "event": "heel_strike", "time": 0.010},)
    legacy.update(
        time_s=0.010,
        events=hs,
        normal_force_bw=1.0,
        in_contact=True,
    )
    shadow.update_policy_step(
        time_s=0.010,
        previous_time_s=0.0,
        sensor_samples=[
            _sample(index * 0.001, 0.0, 0.0) for index in range(1, 11)
        ],
        events=hs,
        normal_force_bw=1.0,
        in_contact=True,
    )

    toe_off = ({"side": "left", "event": "toe_off", "time": 0.030},)
    legacy_payload = legacy.update(
        time_s=0.030,
        events=toe_off,
        normal_force_bw=0.0,
        in_contact=False,
    )
    shadow_payload = shadow.update_policy_step(
        time_s=0.030,
        previous_time_s=0.010,
        sensor_samples=[
            _sample(index * 0.001, 0.0, 0.0) for index in range(11, 31)
        ],
        events=toe_off,
        normal_force_bw=0.0,
        in_contact=False,
    )

    assert legacy_payload["state_name"] == "SWING_AFTER_TO"
    assert shadow_payload["state_name"] == "SWING_AFTER_TO"
    for key in (
        "state_id",
        "valid_hs_count",
        "valid_to_count",
        "invalid_event_count",
        "timeout_exceeded",
        "timeout_side",
        "cycle_progress_credit",
        "pending_cycle_credit",
    ):
        assert shadow_payload[key] == legacy_payload[key], key


def _fake_runner() -> SimulationRunner:
    runner = SimulationRunner.__new__(SimulationRunner)
    runner._ctx = SimpleNamespace(online_grf_detector_force_paths=["heel", "toe"])
    runner._phase_sensor_sample_dt_s = 0.001
    runner._phase_sensor_last_sample_time_s = 0.0
    runner._last_step_info = {}

    def fake_sample(self, _state, time_s):
        return _sample(time_s, 1.0, 2.0)

    runner._sample_phase_detector_channels = MethodType(fake_sample, runner)
    return runner


@_test
def test_runner_open_left_closed_right_dedup_and_error_contract() -> None:
    runner = _fake_runner()
    samples = []
    runner._append_phase_sensor_sample(None, 0.0005, samples)
    assert samples == []
    for index in range(1, 11):
        runner._append_phase_sensor_sample(None, index * 0.001, samples)
    assert len(samples) == 10
    for index, sample in enumerate(samples, start=1):
        assert math.isclose(
            sample["time_s"],
            index * 0.001,
            abs_tol=1e-12,
        )
    runner._finalize_phase_sensor_segment(
        segment_start_time_s=0.0,
        t_stop=0.010,
        samples=samples,
    )

    with _raises(PhaseSensorSamplingError, "Duplicate"):
        runner._append_phase_sensor_sample(None, 0.010, samples)
    with _raises(PhaseSensorSamplingError, "non-monotonic"):
        runner._append_phase_sensor_sample(None, -0.001, samples)

    missing = _fake_runner()
    with _raises(PhaseSensorSamplingError, "Missing"):
        missing._append_phase_sensor_sample(None, 0.002, [])
    nonfinite = _fake_runner()

    def nonfinite_sample(self, _state, time_s):
        return _sample(time_s, float("nan"), 0.0)

    nonfinite._sample_phase_detector_channels = MethodType(
        nonfinite_sample,
        nonfinite,
    )
    with _raises(PhaseSensorSamplingError, "non-finite"):
        nonfinite._append_phase_sensor_sample(None, 0.001, [])
    with _raises(PhaseSensorSamplingError, "not aligned"):
        missing._finalize_phase_sensor_segment(
            segment_start_time_s=0.0,
            t_stop=0.0015,
            samples=[],
        )
    with _raises(PhaseSensorSamplingError, "incomplete"):
        missing._finalize_phase_sensor_segment(
            segment_start_time_s=0.0,
            t_stop=0.001,
            samples=[],
        )


if __name__ == "__main__":
    for test in TESTS:
        test()
        print(f"[PASS] {test.__name__}")
    print(f"ALL HIGH-RATE DETECTOR TESTS PASSED ({len(TESTS)})")
