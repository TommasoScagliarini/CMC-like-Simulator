"""Unit gates for the isolated delayed causal morphology corridor.

Run from the repository root with either::

    python validation/test_causal_delayed_morphology_corridor.py
    pytest -q validation/test_causal_delayed_morphology_corridor.py

The suite is OpenSim-free and never starts PPO.
"""

from __future__ import annotations

import copy
import hashlib
import math
import struct
import sys
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
BASELINE_DIR = REPO_ROOT / "Trajectory Generator" / "baseline_MLP"
sys.path.insert(0, str(BASELINE_DIR))

from experimental_morphology_corridor import (  # noqa: E402
    CAUSAL_DELAYED_PHASE_MODE,
    TWO_SENSOR_EVENT_CONTRACT_ID,
    CausalDelayedMorphologyBuffer,
    ResolvedCausalMorphologySample,
    apply_causal_morphology_reward,
)


def _buffer(*, max_samples: int = 4096) -> CausalDelayedMorphologyBuffer:
    return CausalDelayedMorphologyBuffer(
        delay_s=0.04,
        canonical_to_phase=0.60,
        nominal_stance_duration_s=0.10,
        nominal_swing_duration_s=0.10,
        max_samples=max_samples,
    )


def _transition(
    event: str,
    onset_s: float,
    delivered_s: float,
) -> dict[str, float | str]:
    segment_type = "stance" if event == "heel_strike" else "swing"
    return {
        "event": event,
        "event_time_s": float(onset_s),
        "confirmed_time_s": float(max(onset_s, delivered_s - 0.01)),
        "delivered_time_s": float(delivered_s),
        "segment_valid": 1.0,
        "opens_segment_type": segment_type,
    }


def _pending(event: str, onset_s: float) -> dict[str, float | str]:
    return {"event": event, "event_time_s": float(onset_s)}


def _step(
    buffer: CausalDelayedMorphologyBuffer,
    time_s: float,
    *,
    transitions=(),
    pending=None,
    ended: bool = False,
):
    time_f = float(time_s)
    return buffer.update(
        time_s=time_f,
        knee_rad=0.20 + time_f,
        ankle_rad=-0.10 - time_f,
        accepted_transitions=transitions,
        pending_transition=pending,
        episode_ended=ended,
    )


def _loss(item: ResolvedCausalMorphologySample) -> float:
    return float(
        item.phase**2
        + item.sample.knee_rad**2
        + item.sample.ankle_rad**2
    )


def test_weight_zero_is_bit_exact() -> None:
    base = math.nextafter(123.456789, math.inf)
    shaped = apply_causal_morphology_reward(base, 987654.321, 0.0)
    assert struct.pack("!d", shaped) == struct.pack("!d", base)


def test_causal_sum_matches_offline_physical_onset_oracle() -> None:
    buffer = _buffer()
    emitted: list[ResolvedCausalMorphologySample] = []
    dropped = 0

    for index in range(21):
        time_s = index / 100.0
        transitions = ()
        pending = None
        if 0.05 <= time_s < 0.09:
            pending = _pending("heel_strike", 0.05)
        elif math.isclose(time_s, 0.09, abs_tol=1e-12):
            transitions = (_transition("heel_strike", 0.05, 0.09),)
        elif 0.13 <= time_s < 0.17:
            pending = _pending("toe_off", 0.13)
        elif math.isclose(time_s, 0.17, abs_tol=1e-12):
            transitions = (_transition("toe_off", 0.13, 0.17),)

        update = _step(
            buffer,
            time_s,
            transitions=transitions,
            pending=pending,
            ended=index == 20,
        )
        assert not update.failed_closed, update.failure_reason
        emitted.extend(update.resolved_samples)
        dropped += update.dropped_sample_count

    # Samples before the first physical HS are neither prescribed nor assigned
    # a fabricated phase.  Every sample from HS onward is settled exactly once.
    assert dropped == 5
    assert [round(item.sample.time_s, 10) for item in emitted] == [
        round(index / 100.0, 10) for index in range(5, 21)
    ]

    expected_sum = 0.0
    for index in range(5, 21):
        time_s = index / 100.0
        if time_s < 0.13:
            phase = 0.60 * ((time_s - 0.05) / 0.10)
        else:
            phase = 0.60 + 0.40 * ((time_s - 0.13) / 0.10)
        phase = min(1.0, max(0.0, phase))
        expected_sum += phase**2 + (0.20 + time_s) ** 2 + (-0.10 - time_s) ** 2

    online_sum = sum(_loss(item) for item in emitted)
    assert math.isclose(online_sum, expected_sum, rel_tol=0.0, abs_tol=1e-12)

    for item in emitted:
        assert item.anchor_delivered_time_s <= item.emitted_time_s + 1e-12
        if not item.terminal_flush:
            assert item.delay_s >= 0.04 - 1e-12
        if item.sample.time_s < 0.13:
            assert item.segment_type == "stance"
            assert math.isclose(item.segment_start_time_s, 0.05, abs_tol=1e-12)
        else:
            assert item.segment_type == "swing"
            assert math.isclose(item.segment_start_time_s, 0.13, abs_tol=1e-12)


def test_positive_weight_changes_only_reward_for_frozen_actions() -> None:
    actions = [[0.25, -0.50], [0.10, 0.20], [-0.30, 0.40]]
    observations = [[1.0, 2.0], [3.0, 4.0], [5.0, 6.0]]
    pulses = [{"hs": 1.0, "to": 0.0}, {"hs": 0.0, "to": 0.0}]
    actions_before = copy.deepcopy(actions)
    observations_before = copy.deepcopy(observations)
    pulses_before = copy.deepcopy(pulses)

    buffer = _buffer()
    _step(
        buffer,
        0.00,
        transitions=(_transition("heel_strike", 0.00, 0.00),),
    )
    _step(buffer, 0.01)
    update = _step(buffer, 0.02, ended=True)
    loss = sum(_loss(item) for item in update.resolved_samples)
    baseline_reward = 7.25
    shaped_reward = apply_causal_morphology_reward(
        baseline_reward,
        loss,
        0.05,
    )

    assert shaped_reward == baseline_reward - 0.05 * loss
    assert actions == actions_before
    assert observations == observations_before
    assert pulses == pulses_before


def test_terminal_flushes_resolved_and_drops_only_pending_onset_tail() -> None:
    buffer = _buffer()
    terminal_update = None
    for index in range(5, 19):
        time_s = index / 100.0
        transitions = (
            (_transition("heel_strike", 0.05, 0.05),)
            if index == 5
            else ()
        )
        pending = _pending("toe_off", 0.16) if index >= 16 else None
        terminal_update = _step(
            buffer,
            time_s,
            transitions=transitions,
            pending=pending,
            ended=index == 18,
        )
        assert not terminal_update.failed_closed, terminal_update.failure_reason

    assert terminal_update is not None
    assert terminal_update.terminal_flushed
    assert terminal_update.pending_sample_count == 0
    assert terminal_update.dropped_pending_sample_count == 3
    assert terminal_update.dropped_sample_count == 3
    assert terminal_update.drop_reason == "episode_end_pending_transition"
    terminal_times = [
        round(item.sample.time_s, 10)
        for item in terminal_update.resolved_samples
        if item.terminal_flush
    ]
    assert terminal_times == [0.15]
    assert terminal_update.total_resolved_sample_count == 11  # 0.05 through 0.15
    assert terminal_update.total_dropped_sample_count == 3


def test_nonfinite_and_contract_violations_fail_closed_permanently() -> None:
    try:
        CausalDelayedMorphologyBuffer(
            delay_s=0.04,
            canonical_to_phase=0.60,
            nominal_stance_duration_s=0.10,
            nominal_swing_duration_s=0.10,
            event_contract_id="legacy_events_v1",
        )
    except ValueError as exc:
        assert "event_contract_id" in str(exc)
    else:
        raise AssertionError("legacy event contract was accepted")

    buffer = _buffer()
    first = _step(
        buffer,
        0.00,
        transitions=(_transition("heel_strike", 0.00, 0.00),),
    )
    assert not first.failed_closed
    failed = buffer.update(
        time_s=0.01,
        knee_rad=float("nan"),
        ankle_rad=0.0,
        accepted_transitions=(),
    )
    assert failed.failed_closed
    assert failed.failure_reason == "nonfinite_sample"
    assert failed.dropped_sample_count == 2
    after = _step(buffer, 0.02)
    assert after.failed_closed
    assert after.resolved_samples == ()

    late = _buffer()
    _step(late, 0.00)
    late_update = _step(
        late,
        0.05,
        transitions=(_transition("heel_strike", 0.00, 0.05),),
    )
    assert late_update.failed_closed
    assert late_update.failure_reason == "transition_exceeds_morphology_delay"

    slow_delivery = _buffer()
    slow_delivery_update = slow_delivery.update(
        time_s=0.02,
        knee_rad=0.0,
        ankle_rad=0.0,
        accepted_transitions=(
            {
                "event": "heel_strike",
                "event_time_s": 0.00,
                "confirmed_time_s": 0.00,
                "delivered_time_s": 0.02,
                "segment_valid": 1.0,
            },
        ),
    )
    assert slow_delivery_update.failed_closed
    assert slow_delivery_update.failure_reason == "transition_exceeds_delivery_latency"

    pending = _buffer()
    _step(pending, 0.00, pending=_pending("heel_strike", 0.00))
    disappeared = _step(pending, 0.01)
    assert disappeared.failed_closed
    assert disappeared.failure_reason == "pending_transition_disappeared"

    overflow = _buffer(max_samples=2)
    _step(overflow, 0.00)
    _step(overflow, 0.01)
    overflow_update = _step(overflow, 0.02)
    assert overflow_update.failed_closed
    assert overflow_update.failure_reason == "causal_buffer_overflow"

    corrupted_partial = _buffer()
    corrupted_update = _step(
        corrupted_partial,
        0.04,
        transitions=(
            {
                **_transition("toe_off", 0.00, 0.04),
                "segment_valid": 0.0,
                "from_state_id": 0.0,
                "to_state_id": 2.0,
                "closed_segment_type": "stance",
                "opens_segment_type": "swing",
            },
        ),
    )
    assert corrupted_update.failed_closed
    assert (
        corrupted_update.failure_reason
        == "invalid_or_rejected_transition_segment"
    )

    for args in (
        (1.0, float("nan"), 0.05),
        (1.0, 2.0, float("inf")),
        (1.0, 2.0, -0.1),
    ):
        try:
            apply_causal_morphology_reward(*args)
        except ValueError:
            pass
        else:  # pragma: no cover - clearer direct-run failure.
            raise AssertionError(f"non-finite/invalid reward input accepted: {args}")


def test_initial_partial_stance_to_discards_prefix_and_opens_swing() -> None:
    buffer = _buffer()
    updates = []
    for index in range(8):
        time_s = index / 100.0
        pending = (
            _pending("toe_off", 0.03)
            if 0.03 <= time_s < 0.07
            else None
        )
        transitions = ()
        if math.isclose(time_s, 0.07, abs_tol=1e-12):
            transitions = (
                {
                    **_transition("toe_off", 0.03, 0.07),
                    "segment_valid": 0.0,
                    "from_state_id": 1.0,
                    "to_state_id": 2.0,
                    "closed_segment_type": "stance",
                    "opens_segment_type": "swing",
                },
            )
        update = _step(
            buffer,
            time_s,
            transitions=transitions,
            pending=pending,
        )
        assert not update.failed_closed, update.failure_reason
        updates.append(update)

    final = updates[-1]
    assert final.dropped_sample_count == 3
    assert final.drop_reason == "before_initial_partial_to"
    assert [item.sample.time_s for item in final.resolved_samples] == [0.03]
    assert final.resolved_samples[0].segment_type == "swing"
    assert math.isclose(
        final.resolved_samples[0].phase,
        0.60,
        abs_tol=1e-12,
    )


def test_candidate_yaml_is_separate_preflight_only_contract() -> None:
    candidate_path = (
        BASELINE_DIR
        / "experimental_configs"
        / "morphology_event_anchored_causal_candidate.yaml"
    )
    candidate = yaml.safe_load(candidate_path.read_text(encoding="utf-8"))
    active = yaml.safe_load(
        (BASELINE_DIR / "training_exnovo_cfg.yaml").read_text(encoding="utf-8")
    )

    assert candidate["candidate"]["ppo_updates_authorized"] is False
    assert candidate["candidate"]["active_training_config_replaced"] is False
    assert candidate["simulation"]["segment_duration"] == 0.01
    assert candidate["grf"]["event_contract_id"] == TWO_SENSOR_EVENT_CONTRACT_ID
    assert candidate["grf"]["phase_fsm_input_mode"] == "two_sensor"
    assert candidate["grf"]["phase_sensor_on_threshold_n"] == 0.5
    assert candidate["grf"]["phase_sensor_off_threshold_n"] == 0.25
    assert candidate["grf"]["phase_sensor_dwell_s"] == 0.03
    assert candidate["reward"]["morphology_phase_mode"] == CAUSAL_DELAYED_PHASE_MODE
    assert candidate["reward"]["morphology_reward_delay_s"] == 0.04
    assert candidate["reward"]["morphology_max_delivery_latency_s"] == 0.01
    assert candidate["reward"]["morphology_weight"] == 0.05
    assert candidate["reward"]["morphology_hard_termination_enabled"] == 0.0
    assert active["reward"]["morphology_weight"] == 0.0

    profile_path = BASELINE_DIR / candidate["reward"]["morphology_profile"]
    profile_sha = hashlib.sha256(profile_path.read_bytes()).hexdigest()
    assert profile_sha == candidate["reward"]["morphology_profile_sha256"]

    detector_path = REPO_ROOT / candidate["grf"][
        "online_grf_detector_profile"
    ]
    detector_sha = hashlib.sha256(detector_path.read_bytes()).hexdigest()
    assert detector_sha == candidate["grf"][
        "online_grf_detector_profile_sha256"
    ]


TESTS = (
    test_weight_zero_is_bit_exact,
    test_causal_sum_matches_offline_physical_onset_oracle,
    test_positive_weight_changes_only_reward_for_frozen_actions,
    test_terminal_flushes_resolved_and_drops_only_pending_onset_tail,
    test_nonfinite_and_contract_violations_fail_closed_permanently,
    test_initial_partial_stance_to_discards_prefix_and_opens_swing,
    test_candidate_yaml_is_separate_preflight_only_contract,
)


if __name__ == "__main__":
    for test in TESTS:
        test()
        print(f"[PASS] {test.__name__}")
    print(f"ALL CAUSAL DELAYED MORPHOLOGY TESTS PASSED ({len(TESTS)})")
