from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest


VALIDATION_ROOT = Path(__file__).resolve().parent
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))

import evaluate_two_sensor_v18_signal_semantics as v18


def _trace(
    heel_segments: list[tuple[int, int, float]],
    toe_segments: list[tuple[int, int, float]],
    samples: int = 1201,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    times = np.arange(samples, dtype=float) * 0.001
    heel = np.zeros(samples, dtype=float)
    toe = np.zeros(samples, dtype=float)
    for start, stop, value in heel_segments:
        heel[start:stop] = value
    for start, stop, value in toe_segments:
        toe[start:stop] = value
    return times, heel, toe


def test_contract_is_frozen_to_three_semantics_and_two_open_trials() -> None:
    contract = v18.load_and_validate_contract()
    assert contract["semantics"] == list(v18.SEMANTICS)
    assert contract["trials"] == ["02", "04"]
    assert contract["internal_holdout"]["opened"] is False


def test_t0_primes_without_emitting_an_event() -> None:
    times, heel, toe = _trace([(0, 100, 1.0)], [])
    result = v18.process_trace("heel_only", "sequential_1ms", times, heel, toe)
    assert all(event["event_time_s"] > 0.0 for event in result["events"])
    assert not any(event["event"] == "heel_strike" for event in result["events"])


def test_debounce_confirmation_is_exactly_30ms_after_onset() -> None:
    times, heel, toe = _trace([(100, 400, 1.0)], [], samples=501)
    result = v18.process_trace("heel_only", "sequential_1ms", times, heel, toe)
    hs = next(event for event in result["events"] if event["event"] == "heel_strike")
    assert hs["event_time_s"] == pytest.approx(0.100)
    assert hs["confirmed_time_s"] == pytest.approx(0.130)
    assert hs["confirmed_time_s"] - hs["event_time_s"] == pytest.approx(0.03)


def test_first_regional_and_combined_are_not_later_than_heel_only() -> None:
    times, heel, toe = _trace(
        [(200, 500, 1.0)],
        [(100, 500, 1.0)],
        samples=601,
    )
    outputs = {
        semantic: v18.process_trace(
            semantic, "sequential_1ms", times, heel, toe
        )
        for semantic in v18.SEMANTICS
    }
    confirmations = {
        semantic: next(
            event["confirmed_time_s"]
            for event in output["events"]
            if event["event"] == "heel_strike"
        )
        for semantic, output in outputs.items()
    }
    assert confirmations["first_stable_regional"] < confirmations["heel_only"]
    assert confirmations["combined_load"] <= confirmations["heel_only"]


def test_combined_load_uses_sum_before_debounce() -> None:
    times, heel, toe = _trace(
        [(100, 300, 0.3)],
        [(100, 300, 0.3)],
        samples=401,
    )
    combined = v18.process_trace(
        "combined_load", "sequential_1ms", times, heel, toe
    )
    regional = v18.process_trace(
        "first_stable_regional", "sequential_1ms", times, heel, toe
    )
    assert any(event["event"] == "heel_strike" for event in combined["events"])
    assert not any(event["event"] == "heel_strike" for event in regional["events"])


@pytest.mark.parametrize("semantic", v18.SEMANTICS)
def test_sequential_and_batch_are_exactly_equal(semantic: str) -> None:
    times, heel, toe = _trace(
        [(100, 350, 1.0), (700, 1000, 1.0)],
        [(150, 400, 1.0), (650, 1050, 1.0)],
    )
    sequential = v18.process_trace(
        semantic, "sequential_1ms", times, heel, toe
    )
    batched = v18.process_trace(
        semantic, "batched_10ms_same_samples", times, heel, toe
    )
    assert sequential == batched


def test_delivery_is_next_policy_boundary() -> None:
    times, heel, toe = _trace([(103, 400, 1.0)], [], samples=501)
    result = v18.process_trace("heel_only", "sequential_1ms", times, heel, toe)
    hs = next(event for event in result["events"] if event["event"] == "heel_strike")
    assert hs["confirmed_index"] == 133
    assert hs["delivered_index"] == 140
    assert hs["delivered_time_s"] - hs["confirmed_time_s"] == pytest.approx(0.007)


def test_ordered_matching_is_one_to_one() -> None:
    assert v18._ordered_match_count([1.0, 2.0], [0.96, 1.04, 2.01], 0.05) == 2
    assert v18._ordered_match_count([1.0, 2.0], [1.2, 2.2], 0.05) == 0


def test_unsupported_semantic_and_mode_fail_closed() -> None:
    times, heel, toe = _trace([], [], samples=50)
    with pytest.raises(v18.V18SelectionError, match="semantic"):
        v18.process_trace("invented", "sequential_1ms", times, heel, toe)
    with pytest.raises(v18.V18SelectionError, match="mode"):
        v18.process_trace("heel_only", "invented", times, heel, toe)
