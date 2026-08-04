"""Regression tests for the experimental completed-segment morphology ledger.

Run directly from the repository root::

    python validation/test_experimental_retrospective_morphology.py

The tests intentionally avoid OpenSim.  They exercise the production FSM
accepted-transition journal and the isolated retrospective ledger with
synthetic policy-step samples whose timestamps make ownership unambiguous.
No test enables the experimental mode in the production training YAML.
"""

from __future__ import annotations

import copy
import json
import math
import sys
import tempfile
from collections.abc import Mapping, Sequence
from dataclasses import replace
from pathlib import Path
from typing import Any

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
TRAJECTORY_DIR = REPO_ROOT / "Trajectory Generator"
BASELINE_DIR = TRAJECTORY_DIR / "baseline_MLP"
sys.path.insert(0, str(TRAJECTORY_DIR))
sys.path.insert(0, str(BASELINE_DIR))

from experimental_morphology_corridor import (  # noqa: E402
    EXPERIMENTAL_PHASE_MODE,
    CompletedMorphologySegment,
    CompletedSegmentMorphologyLedger,
    MorphologySample,
)
from prosthetic_phase_fsm import (  # noqa: E402
    STANCE_AFTER_HS,
    SWING_AFTER_TO,
    TIMEOUT,
    WAIT_HS,
    ProstheticPhaseFSM,
    ProstheticPhaseFSMConfig,
)
import reward_function  # noqa: E402


TESTS = []


def _test(func):
    TESTS.append(func)
    return func


def _assert_close(actual: float, expected: float, *, atol: float = 1e-12) -> None:
    assert math.isclose(
        float(actual),
        float(expected),
        rel_tol=0.0,
        abs_tol=atol,
    ), f"expected {expected!r}, got {actual!r}"


def _assert_sequence_close(
    actual: Sequence[float],
    expected: Sequence[float],
    *,
    atol: float = 1e-12,
) -> None:
    assert len(actual) == len(expected), (len(actual), len(expected))
    for actual_value, expected_value in zip(actual, expected):
        _assert_close(actual_value, expected_value, atol=atol)


def _assert_raises(error_type, callback, *, contains: str = "") -> None:
    try:
        callback()
    except error_type as exc:
        if contains:
            assert contains in str(exc), str(exc)
        return
    except Exception as exc:  # pragma: no cover - clearer standalone failure.
        raise AssertionError(
            f"expected {error_type.__name__}, got {type(exc).__name__}: {exc}"
        ) from exc
    raise AssertionError(f"expected {error_type.__name__} to be raised")


def _transition(
    event: str,
    event_time_s: float,
    *,
    segment_valid: bool = True,
    closed_segment_type: str | None = None,
    segment_start_time_s: float | None = None,
) -> dict[str, Any]:
    event_name = str(event)
    expected_closed = {
        "toe_off": "stance",
        "heel_strike": "swing",
        "timeout": "",
    }.get(event_name, "")
    payload = {
        "event": event_name,
        "event_time_s": float(event_time_s),
        "closed_segment_type": (
            expected_closed if closed_segment_type is None else str(closed_segment_type)
        ),
        "segment_valid": float(bool(segment_valid)),
    }
    if segment_start_time_s is not None:
        payload["segment_start_time_s"] = float(segment_start_time_s)
    return payload


def _ledger_update(
    ledger: CompletedSegmentMorphologyLedger,
    time_s: float,
    *,
    transitions: Sequence[Mapping[str, Any]] = (),
    episode_ended: bool = False,
    knee_rad: float | None = None,
    ankle_rad: float | None = None,
):
    time_f = float(time_s)
    return ledger.update(
        time_s=time_f,
        knee_rad=time_f if knee_rad is None else float(knee_rad),
        ankle_rad=-time_f if ankle_rad is None else float(ankle_rad),
        accepted_transitions=transitions,
        episode_ended=episode_ended,
    )


def _sample_times(segment: CompletedMorphologySegment) -> tuple[float, ...]:
    return tuple(float(sample.time_s) for sample in segment.samples)


def _left_event(name: str, event_time_s: float) -> dict[str, Any]:
    return {
        "side": "left",
        "event": str(name),
        "time": float(event_time_s),
    }


def _fsm_update(
    fsm: ProstheticPhaseFSM,
    time_s: float,
    events: Sequence[Mapping[str, Any]] = (),
    *,
    knee_rad: float = -0.2,
    ankle_rad: float = 0.1,
) -> dict[str, Any]:
    return fsm.update(
        time_s=float(time_s),
        events=events,
        normal_force_bw=1.0,
        in_contact=True,
        prosthetic_knee_angle_rad=float(knee_rad),
        prosthetic_ankle_angle_rad=float(ankle_rad),
    )


class _ScriptedEnv(reward_function.gym.Env):
    """Deterministic Gym env that replays precomputed reward-info payloads."""

    action_space = reward_function.gym.spaces.Box(
        low=-1.0,
        high=1.0,
        shape=(2,),
        dtype=np.float32,
    )
    observation_space = reward_function.gym.spaces.Box(
        low=-np.inf,
        high=np.inf,
        shape=(1,),
        dtype=np.float32,
    )

    def __init__(self, infos: Sequence[Mapping[str, Any]]) -> None:
        super().__init__()
        self._infos = [copy.deepcopy(dict(info)) for info in infos]
        self._index = 0
        self.received_actions: list[np.ndarray] = []

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._index = 0
        self.received_actions = []
        return np.zeros(1, dtype=np.float32), {}

    def step(self, action):
        if self._index >= len(self._infos):
            raise RuntimeError("scripted morphology env exhausted")
        self.received_actions.append(np.asarray(action, dtype=float).copy())
        info = copy.deepcopy(self._infos[self._index])
        terminated = bool(info.pop("_env_terminated", False))
        truncated = bool(info.pop("_env_truncated", False))
        self._index += 1
        return (
            np.asarray([float(self._index)], dtype=np.float32),
            -123.0,
            terminated,
            truncated,
            info,
        )


def _synthetic_profile(path: Path) -> Path:
    """Write a flat +/-0.1 rad corridor with a valid event contract."""
    payload = {
        "version": 1,
        "name": "synthetic_flat_event_corridor",
        "units": "radian",
        "phase_grid": [0.0, 0.625, 1.0],
        "coordinates": {
            "pros_knee_angle": {
                "mean_rad": [0.0, 0.0, 0.0],
                "std_rad": [0.1, 0.1, 0.1],
            },
            "pros_ankle_angle": {
                "mean_rad": [0.0, 0.0, 0.0],
                "std_rad": [0.1, 0.1, 0.1],
            },
        },
        "metadata": {
            "phase_parameterization": "event_warped_hs_to_to_to_hs_v1",
            "canonical_to_phase": 0.625,
        },
    }
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def _scripted_infos(
    samples: Sequence[tuple[float, float, float, Sequence[Mapping[str, Any]]]],
) -> list[dict[str, Any]]:
    """Build realistic wrapper infos with the production FSM journal."""
    fsm = ProstheticPhaseFSM(
        ProstheticPhaseFSMConfig(
            min_stance_duration_s=0.01,
            min_swing_duration_s=0.01,
            stance_hard_timeout_s=5.0,
            swing_hard_timeout_s=5.0,
            min_stance_contact_fraction=0.0,
            min_stance_load_bw_s=0.0,
            min_cycle_knee_excursion_rad=0.0,
        )
    )
    infos: list[dict[str, Any]] = []
    for time_s, knee_rad, ankle_rad, events in samples:
        payload = _fsm_update(
            fsm,
            time_s,
            events,
            knee_rad=knee_rad,
            ankle_rad=ankle_rad,
        )
        infos.append(
            {
                "time": float(time_s),
                "observation": {
                    "pros_knee_angle_served_ref": float(knee_rad),
                    "pros_ankle_angle_served_ref": float(ankle_rad),
                    "pros_knee_angle": float(knee_rad),
                    "pros_ankle_angle": float(ankle_rad),
                },
                "phase_fsm": payload,
                "reward_terms": {
                    "tracking_loss": 0.0,
                    "terminated": 0.0,
                    "truncated": 0.0,
                },
                "end_reason": None,
            }
        )
    return infos


def _wrapper_config(
    profile_path: Path,
    *,
    phase_mode: str = EXPERIMENTAL_PHASE_MODE,
    morphology_weight: float = 0.0,
    hard_termination_enabled: bool = False,
    allow_effects: bool = False,
    hard_q_min: tuple[float, float] = (-10.0, -10.0),
    hard_q_max: tuple[float, float] = (10.0, 10.0),
) -> reward_function.RewardConfig:
    return reward_function.RewardConfig(
        reward_mode="ex_novo",
        tracking_weight=1.0,
        blend_tracking=1.0,
        blend_reference=0.0,
        blend_bio=0.0,
        blend_contact_load=0.0,
        blend_contact_support_to=0.0,
        blend_phase_regular=0.0,
        blend_phase_event_progress=0.0,
        blend_landing_window_contact=0.0,
        safety_weight=0.0,
        grf_penetration_weight=0.0,
        phase_timeout_penalty_weight=0.0,
        phase_clawback_penalty_weight=0.0,
        contact_support_failure_clawback_weight=0.0,
        reserve_residual_weight=0.0,
        pelvis_height_weight=0.0,
        prosthetic_joint_range_weight=0.0,
        oob_weight=0.0,
        morphology_profile=str(profile_path),
        morphology_phase_mode=phase_mode,
        morphology_weight=float(morphology_weight),
        morphology_std_multiplier_knee=1.0,
        morphology_std_multiplier_ankle=1.0,
        morphology_margin_knee_deg=0.0,
        morphology_margin_ankle_deg=0.0,
        morphology_hard_q_min=hard_q_min,
        morphology_hard_q_max=hard_q_max,
        morphology_hard_termination_enabled=float(hard_termination_enabled),
        morphology_experimental_allow_effects=float(allow_effects),
    )


def _run_wrapper(
    infos: Sequence[Mapping[str, Any]],
    config: reward_function.RewardConfig,
):
    env = _ScriptedEnv(infos)
    wrapper = reward_function.RewardShapingWrapper(env, config)
    wrapper.reset(seed=123)
    outputs = []
    action = np.asarray([0.25, -0.25], dtype=np.float32)
    for _ in infos:
        outputs.append(wrapper.step(action))
    assert len(env.received_actions) == len(infos)
    assert all(np.array_equal(value, action) for value in env.received_actions)
    return outputs


@_test
def test_experimental_mode_is_explicit_and_opt_in() -> None:
    assert EXPERIMENTAL_PHASE_MODE == ("event_anchored_completed_segment_experimental")
    assert reward_function.RewardConfig().morphology_phase_mode == (
        "legacy_cycle_fraction"
    )
    assert (
        reward_function._normalize_morphology_phase_mode(
            "event_retrospective_experimental"
        )
        == EXPERIMENTAL_PHASE_MODE
    )
    assert (
        reward_function._normalize_morphology_phase_mode(
            "completed_segment_experimental"
        )
        == EXPERIMENTAL_PHASE_MODE
    )
    assert reward_function.RewardConfig().morphology_hard_termination_enabled == 0.0
    assert reward_function.RewardConfig().morphology_experimental_allow_effects == 0.0


@_test
def test_exact_phases_are_invariant_to_stretch_and_compression() -> None:
    alpha = 0.625
    normalized_progress = (0.0, 0.25, 0.50, 0.75)

    short_stance = CompletedMorphologySegment(
        segment_type="stance",
        start_time_s=0.0,
        end_time_s=0.4,
        samples=tuple(
            MorphologySample(0.4 * progress, -progress, progress)
            for progress in normalized_progress
        ),
    )
    long_stance = CompletedMorphologySegment(
        segment_type="stance",
        start_time_s=2.0,
        end_time_s=4.0,
        samples=tuple(
            MorphologySample(2.0 + 2.0 * progress, -progress, progress)
            for progress in normalized_progress
        ),
    )
    short_swing = CompletedMorphologySegment(
        segment_type="swing",
        start_time_s=5.0,
        end_time_s=5.4,
        samples=tuple(
            MorphologySample(5.0 + 0.4 * progress, -progress, progress)
            for progress in normalized_progress
        ),
    )
    long_swing = CompletedMorphologySegment(
        segment_type="swing",
        start_time_s=8.0,
        end_time_s=10.0,
        samples=tuple(
            MorphologySample(8.0 + 2.0 * progress, -progress, progress)
            for progress in normalized_progress
        ),
    )

    expected_stance = tuple(alpha * value for value in normalized_progress)
    expected_swing = tuple(
        alpha + (1.0 - alpha) * value for value in normalized_progress
    )
    for actual, expected in zip(short_stance.phases(alpha), expected_stance):
        _assert_close(actual, expected)
    for actual, expected in zip(long_stance.phases(alpha), expected_stance):
        _assert_close(actual, expected)
    for actual, expected in zip(short_swing.phases(alpha), expected_swing):
        _assert_close(actual, expected)
    for actual, expected in zip(long_swing.phases(alpha), expected_swing):
        _assert_close(actual, expected)

    _assert_sequence_close(short_stance.phases(alpha), long_stance.phases(alpha))
    _assert_sequence_close(short_swing.phases(alpha), long_swing.phases(alpha))
    assert all(
        left < right
        for left, right in zip(long_stance.phases(alpha), long_stance.phases(alpha)[1:])
    )
    assert all(
        left < right
        for left, right in zip(short_swing.phases(alpha), short_swing.phases(alpha)[1:])
    )


@_test
def test_backdated_hs_and_to_repartition_the_uncommitted_queue() -> None:
    ledger = CompletedSegmentMorphologyLedger(max_samples=64)

    _ledger_update(ledger, 9.98)
    _ledger_update(ledger, 10.02)
    _ledger_update(ledger, 10.04)
    hs_update = _ledger_update(
        ledger,
        10.06,
        transitions=(_transition("heel_strike", 10.0),),
    )

    assert hs_update.discarded_sample_count == 1
    assert hs_update.discard_reason == "before_first_anchor"
    assert hs_update.active_segment_type == "stance"
    assert hs_update.pending_sample_count == 3

    _ledger_update(ledger, 10.50)
    _ledger_update(ledger, 10.99)
    _ledger_update(ledger, 11.00)
    to_update = _ledger_update(
        ledger,
        11.01,
        transitions=(_transition("toe_off", 11.0),),
    )

    assert len(to_update.completed_segments) == 1
    stance = to_update.completed_segments[0]
    assert stance.segment_type == "stance"
    _assert_close(stance.start_time_s, 10.0)
    _assert_close(stance.end_time_s, 11.0)
    assert _sample_times(stance) == (10.02, 10.04, 10.06, 10.50, 10.99)
    assert to_update.active_segment_type == "swing"
    # The exact-TO sample and the confirmation-step sample belong to swing.
    assert to_update.pending_sample_count == 2

    _ledger_update(ledger, 11.50)
    _ledger_update(ledger, 12.00)
    hs2_update = _ledger_update(
        ledger,
        12.06,
        transitions=(_transition("heel_strike", 12.0),),
    )

    assert len(hs2_update.completed_segments) == 1
    swing = hs2_update.completed_segments[0]
    assert swing.segment_type == "swing"
    _assert_close(swing.start_time_s, 11.0)
    _assert_close(swing.end_time_s, 12.0)
    assert _sample_times(swing) == (11.00, 11.01, 11.50)
    assert hs2_update.active_segment_type == "stance"
    # The exact-HS sample is retained for the newly opened stance.
    assert hs2_update.pending_sample_count == 2


@_test
def test_half_open_ownership_has_no_lost_or_duplicate_anchored_samples() -> None:
    ledger = CompletedSegmentMorphologyLedger(max_samples=64)
    completed: list[CompletedMorphologySegment] = []

    updates = (
        (0.00, (_transition("heel_strike", 0.00),)),
        (0.25, ()),
        (0.50, ()),
        (1.00, ()),
        (1.10, (_transition("toe_off", 1.00),)),
        (1.50, ()),
        (2.00, ()),
        (2.10, (_transition("heel_strike", 2.00),)),
        (2.50, ()),
        (3.00, ()),
        (3.10, (_transition("toe_off", 3.00),)),
    )
    all_times: list[float] = []
    for time_s, transitions in updates:
        all_times.append(time_s)
        update = _ledger_update(ledger, time_s, transitions=transitions)
        completed.extend(update.completed_segments)

    assert [segment.segment_type for segment in completed] == [
        "stance",
        "swing",
        "stance",
    ]
    settled_times = [
        sample.time_s for segment in completed for sample in segment.samples
    ]
    expected = [value for value in all_times if 0.0 <= value < 3.0]
    assert settled_times == expected
    assert len(settled_times) == len(set(settled_times))
    # The sample exactly on each event starts the next segment.
    assert _sample_times(completed[0])[-1] == 0.50
    assert _sample_times(completed[1])[0] == 1.00
    assert _sample_times(completed[2])[0] == 2.00
    assert ledger.pending_sample_count == 2  # 3.00 and its confirmation 3.10.


@_test
def test_rejected_segment_is_discarded_but_opens_the_next_segment() -> None:
    ledger = CompletedSegmentMorphologyLedger(max_samples=32)
    _ledger_update(
        ledger,
        0.0,
        transitions=(_transition("heel_strike", 0.0),),
    )
    _ledger_update(ledger, 0.2)
    rejected = _ledger_update(
        ledger,
        0.5,
        transitions=(_transition("toe_off", 0.4, segment_valid=False),),
    )

    assert rejected.completed_segments == ()
    assert rejected.discarded_segment_count == 1
    assert rejected.discarded_sample_count == 2
    assert rejected.discard_reason == "fsm_rejected_segment"
    assert rejected.active_segment_type == "swing"
    # The confirmation sample lies after the backdated TO and is retained.
    assert rejected.pending_sample_count == 1


@_test
def test_transition_start_mismatch_discards_desynchronized_segment() -> None:
    ledger = CompletedSegmentMorphologyLedger(max_samples=32)
    _ledger_update(
        ledger,
        1.0,
        transitions=(_transition("heel_strike", 1.0),),
    )
    _ledger_update(ledger, 1.2)
    mismatch = _ledger_update(
        ledger,
        1.51,
        transitions=(
            _transition(
                "toe_off",
                1.5,
                segment_start_time_s=1.01,
            ),
        ),
    )

    assert mismatch.completed_segments == ()
    assert mismatch.discarded_segment_count == 1
    assert mismatch.discarded_sample_count == 2
    assert mismatch.discard_reason == "transition_start_mismatch"
    assert mismatch.active_segment_type == "swing"
    assert mismatch.active_segment_start_time_s == 1.5
    assert mismatch.pending_sample_count == 1

    nonfinite_ledger = CompletedSegmentMorphologyLedger(max_samples=32)
    _ledger_update(
        nonfinite_ledger,
        2.0,
        transitions=(_transition("heel_strike", 2.0),),
    )
    _ledger_update(nonfinite_ledger, 2.2)
    nonfinite = _ledger_update(
        nonfinite_ledger,
        2.51,
        transitions=(
            _transition(
                "toe_off",
                2.5,
                segment_start_time_s=float("nan"),
            ),
        ),
    )
    assert nonfinite.completed_segments == ()
    assert nonfinite.discarded_segment_count == 1
    assert nonfinite.discarded_sample_count == 2
    assert nonfinite.discard_reason == "transition_start_mismatch"
    assert nonfinite.active_segment_type == "swing"
    assert nonfinite.pending_sample_count == 1


@_test
def test_fsm_rejected_cycle_still_settles_exact_swing_once() -> None:
    fsm = ProstheticPhaseFSM(
        ProstheticPhaseFSMConfig(
            min_stance_duration_s=0.05,
            min_swing_duration_s=0.05,
            stance_hard_timeout_s=2.0,
            swing_hard_timeout_s=2.0,
            min_stance_contact_fraction=0.0,
            min_stance_load_bw_s=0.0,
            min_cycle_knee_excursion_rad=0.5,
        )
    )
    ledger = CompletedSegmentMorphologyLedger(max_samples=64)

    hs = _fsm_update(fsm, 0.06, (_left_event("heel_strike", 0.0),))
    _ledger_update(
        ledger,
        0.06,
        transitions=hs["accepted_transitions_this_step"],
    )
    middle = _fsm_update(fsm, 0.2)
    _ledger_update(
        ledger,
        0.2,
        transitions=middle["accepted_transitions_this_step"],
    )
    toe_off = _fsm_update(fsm, 0.41, (_left_event("toe_off", 0.4),))
    stance_update = _ledger_update(
        ledger,
        0.41,
        transitions=toe_off["accepted_transitions_this_step"],
    )
    assert len(stance_update.completed_segments) == 1
    assert stance_update.completed_segments[0].segment_type == "stance"

    swing_middle = _fsm_update(fsm, 0.6)
    _ledger_update(
        ledger,
        0.6,
        transitions=swing_middle["accepted_transitions_this_step"],
    )
    next_hs = _fsm_update(fsm, 0.86, (_left_event("heel_strike", 0.8),))
    journal = next_hs["accepted_transitions_this_step"]
    assert next_hs["cycle_rejected_this_step"] == 1.0
    assert next_hs["valid_cycle_count"] == 0.0
    assert next_hs["phase_cycle_failed_this_step"] == 1.0
    assert next_hs["cycle_reject_reason"] == "cycle_knee_excursion_too_low"
    assert len(journal) == 1
    assert journal[0]["event"] == "heel_strike"
    assert journal[0]["closed_segment_type"] == "swing"
    assert journal[0]["segment_valid"] == 1.0
    assert journal[0]["anchor_geometry_valid"] == 1.0
    assert journal[0]["cycle_valid"] == 0.0
    assert journal[0]["cycle_reject_reason"] == "cycle_knee_excursion_too_low"

    settled = _ledger_update(ledger, 0.86, transitions=journal)
    assert len(settled.completed_segments) == 1
    assert settled.completed_segments[0].segment_type == "swing"
    assert _sample_times(settled.completed_segments[0]) == (0.41, 0.6)
    assert settled.discarded_segment_count == 0
    assert settled.discard_reason == ""
    assert settled.active_segment_type == "stance"

    # The cycle rejection/failure remains in the FSM, but the geometrically
    # complete swing cannot be silently dropped or settled a second time.
    after = _fsm_update(fsm, 0.90)
    no_second_settlement = _ledger_update(
        ledger,
        0.90,
        transitions=after["accepted_transitions_this_step"],
    )
    assert no_second_settlement.completed_segments == ()
    assert after["valid_cycle_count"] == 0.0


@_test
def test_fsm_timeout_discards_the_open_ledger_segment_once() -> None:
    fsm = ProstheticPhaseFSM(
        ProstheticPhaseFSMConfig(
            min_stance_duration_s=0.01,
            min_swing_duration_s=0.01,
            stance_hard_timeout_s=0.30,
            swing_hard_timeout_s=0.30,
            min_stance_contact_fraction=0.0,
            min_stance_load_bw_s=0.0,
            min_cycle_knee_excursion_rad=0.0,
        )
    )
    ledger = CompletedSegmentMorphologyLedger(max_samples=32)

    hs = _fsm_update(fsm, 1.06, (_left_event("heel_strike", 1.0),))
    _ledger_update(
        ledger,
        1.06,
        transitions=hs["accepted_transitions_this_step"],
    )
    regular = _fsm_update(fsm, 1.20)
    _ledger_update(
        ledger,
        1.20,
        transitions=regular["accepted_transitions_this_step"],
    )
    timeout = _fsm_update(fsm, 1.31)

    assert timeout["state_id"] == float(TIMEOUT)
    assert timeout["timeout_exceeded"] == 1.0
    journal = timeout["accepted_transitions_this_step"]
    assert len(journal) == 1
    assert journal[0]["event"] == "timeout"
    assert journal[0]["closed_segment_type"] == "stance"
    assert journal[0]["segment_valid"] == 0.0
    assert journal[0]["anchor_geometry_valid"] == 0.0
    assert journal[0]["cycle_valid"] == 0.0

    discarded = _ledger_update(ledger, 1.31, transitions=journal)
    assert discarded.completed_segments == ()
    assert discarded.discarded_segment_count == 1
    assert discarded.discarded_sample_count == 3
    assert discarded.discard_reason == "fsm_timeout"
    assert discarded.pending_sample_count == 0
    assert discarded.active_segment_type == ""

    after_timeout = _fsm_update(fsm, 1.40)
    assert after_timeout["accepted_transitions_this_step"] == []


@_test
def test_reset_and_episode_end_clear_incomplete_state() -> None:
    ledger = CompletedSegmentMorphologyLedger(max_samples=32)
    _ledger_update(
        ledger,
        1.0,
        transitions=(_transition("heel_strike", 1.0),),
    )
    _ledger_update(ledger, 1.2)
    ended = _ledger_update(ledger, 1.4, episode_ended=True)

    assert ended.completed_segments == ()
    assert ended.discarded_segment_count == 1
    assert ended.discarded_sample_count == 3
    assert ended.discard_reason == "episode_end_incomplete_segment"
    assert ended.pending_sample_count == 0
    assert ended.active_segment_type == ""

    _ledger_update(
        ledger,
        2.0,
        transitions=(_transition("heel_strike", 2.0),),
    )
    _ledger_update(ledger, 2.1)
    ledger.reset()
    assert ledger.pending_sample_count == 0
    assert ledger.active_segment_type == ""

    before_hs = CompletedSegmentMorphologyLedger(max_samples=8)
    _ledger_update(before_hs, 3.0)
    wait_ended = _ledger_update(before_hs, 3.1, episode_ended=True)
    assert wait_ended.discarded_segment_count == 0
    assert wait_ended.discarded_sample_count == 2
    assert wait_ended.discard_reason == "episode_end_before_first_hs"


@_test
def test_episode_end_discards_active_segment_with_no_finite_samples() -> None:
    ledger = CompletedSegmentMorphologyLedger(max_samples=8)
    anchor = ledger.update(
        time_s=1.0,
        knee_rad=float("nan"),
        ankle_rad=float("nan"),
        accepted_transitions=(_transition("heel_strike", 1.0),),
    )
    assert anchor.active_segment_type == "stance"
    assert anchor.active_segment_start_time_s == 1.0
    assert anchor.pending_sample_count == 0

    ended = ledger.update(
        time_s=1.1,
        knee_rad=float("nan"),
        ankle_rad=float("nan"),
        accepted_transitions=(),
        episode_ended=True,
    )
    assert ended.completed_segments == ()
    assert ended.discarded_segment_count == 1
    assert ended.discarded_sample_count == 0
    assert ended.discard_reason == "episode_end_incomplete_segment"
    assert ended.pending_sample_count == 0
    assert ended.active_segment_type == ""
    assert ledger.pending_sample_count == 0
    assert ledger.active_segment_type == ""


@_test
def test_duplicate_timestamp_replaces_sample_and_nonmonotonic_is_ignored() -> None:
    ledger = CompletedSegmentMorphologyLedger(max_samples=16)
    _ledger_update(
        ledger,
        0.0,
        transitions=(_transition("heel_strike", 0.0),),
        knee_rad=0.0,
        ankle_rad=0.0,
    )
    _ledger_update(ledger, 0.1, knee_rad=1.0, ankle_rad=-1.0)
    duplicate = _ledger_update(
        ledger,
        0.1,
        knee_rad=2.0,
        ankle_rad=-2.0,
    )
    assert duplicate.pending_sample_count == 2
    assert not duplicate.nonmonotonic_sample

    nonmonotonic = _ledger_update(
        ledger,
        0.05,
        knee_rad=99.0,
        ankle_rad=-99.0,
    )
    assert nonmonotonic.nonmonotonic_sample
    assert nonmonotonic.discard_reason == "nonmonotonic_sample"
    assert nonmonotonic.pending_sample_count == 2

    closed = _ledger_update(
        ledger,
        0.21,
        transitions=(_transition("toe_off", 0.2),),
    )
    assert len(closed.completed_segments) == 1
    samples = closed.completed_segments[0].samples
    assert _sample_times(closed.completed_segments[0]) == (0.0, 0.1)
    assert samples[1].knee_rad == 2.0
    assert all(sample.knee_rad != 99.0 for sample in samples)


@_test
def test_buffer_overflow_fails_closed_and_does_not_complete_old_segment() -> None:
    ledger = CompletedSegmentMorphologyLedger(max_samples=2)
    _ledger_update(
        ledger,
        0.0,
        transitions=(_transition("heel_strike", 0.0),),
    )
    _ledger_update(ledger, 0.1)
    overflow = _ledger_update(ledger, 0.2)

    assert overflow.overflowed
    assert overflow.discarded_segment_count == 1
    assert overflow.discarded_sample_count == 2
    assert overflow.discard_reason == "buffer_overflow"
    assert overflow.pending_sample_count == 1
    assert overflow.active_segment_type == ""

    after = _ledger_update(
        ledger,
        0.5,
        transitions=(_transition("toe_off", 0.4),),
    )
    assert after.completed_segments == ()
    assert after.active_segment_type == "swing"
    assert "before_first_anchor" in after.discard_reason


@_test
def test_fsm_journal_contains_only_accepted_left_transitions() -> None:
    fsm = ProstheticPhaseFSM(
        ProstheticPhaseFSMConfig(
            min_stance_duration_s=0.20,
            min_swing_duration_s=0.20,
            stance_hard_timeout_s=2.0,
            swing_hard_timeout_s=2.0,
            min_stance_contact_fraction=0.0,
            min_stance_load_bw_s=0.0,
            min_cycle_knee_excursion_rad=0.0,
        )
    )

    to_before_hs = _fsm_update(fsm, 0.91, (_left_event("toe_off", 0.90),))
    assert to_before_hs["state_id"] == float(WAIT_HS)
    assert to_before_hs["invalid_event_this_step"] == 1.0
    assert to_before_hs["accepted_transitions_this_step"] == []

    right_hs = _fsm_update(
        fsm,
        0.92,
        ({"side": "right", "event": "heel_strike", "time": 0.90},),
    )
    assert right_hs["accepted_transitions_this_step"] == []

    hs = _fsm_update(fsm, 1.06, (_left_event("heel_strike", 1.0),))
    assert hs["state_id"] == float(STANCE_AFTER_HS)
    assert hs["last_valid_hs_time_s"] == 1.0
    assert len(hs["accepted_transitions_this_step"]) == 1
    assert hs["accepted_transitions_this_step"][0]["event"] == "heel_strike"
    assert hs["accepted_transitions_this_step"][0]["cycle_valid"] == -1.0

    double_hs = _fsm_update(
        fsm,
        1.11,
        (_left_event("heel_strike", 1.10),),
    )
    assert double_hs["invalid_event_this_step"] == 1.0
    assert double_hs["accepted_transitions_this_step"] == []

    early_to = _fsm_update(fsm, 1.16, (_left_event("toe_off", 1.15),))
    assert early_to["invalid_event_this_step"] == 1.0
    assert early_to["accepted_transitions_this_step"] == []

    toe_off = _fsm_update(fsm, 1.41, (_left_event("toe_off", 1.40),))
    assert toe_off["state_id"] == float(SWING_AFTER_TO)
    assert toe_off["last_valid_to_time_s"] == 1.40
    assert len(toe_off["accepted_transitions_this_step"]) == 1
    assert toe_off["accepted_transitions_this_step"][0]["event"] == "toe_off"
    assert toe_off["accepted_transitions_this_step"][0]["cycle_valid"] == -1.0

    double_to = _fsm_update(fsm, 1.46, (_left_event("toe_off", 1.45),))
    assert double_to["invalid_event_this_step"] == 1.0
    assert double_to["accepted_transitions_this_step"] == []

    early_hs = _fsm_update(fsm, 1.51, (_left_event("heel_strike", 1.50),))
    assert early_hs["invalid_event_this_step"] == 1.0
    assert early_hs["accepted_transitions_this_step"] == []

    next_hs = _fsm_update(fsm, 1.86, (_left_event("heel_strike", 1.80),))
    assert next_hs["state_id"] == float(STANCE_AFTER_HS)
    journal = next_hs["accepted_transitions_this_step"]
    assert len(journal) == 1
    assert journal[0]["event"] == "heel_strike"
    assert journal[0]["closed_segment_type"] == "swing"
    assert journal[0]["segment_valid"] == 1.0
    assert journal[0]["anchor_geometry_valid"] == 1.0
    assert journal[0]["cycle_valid"] == 1.0
    assert journal[0]["cycle_reject_reason"] == ""
    _assert_close(journal[0]["event_time_s"], 1.80)


@_test
def test_fsm_journal_preserves_backdated_zero_time_hs() -> None:
    """A physical event at t=0 must not be replaced by confirmation time."""
    fsm = ProstheticPhaseFSM(
        ProstheticPhaseFSMConfig(
            min_stance_duration_s=0.01,
            min_swing_duration_s=0.01,
            stance_hard_timeout_s=2.0,
            swing_hard_timeout_s=2.0,
            min_stance_contact_fraction=0.0,
            min_stance_load_bw_s=0.0,
            min_cycle_knee_excursion_rad=0.0,
        )
    )
    ledger = CompletedSegmentMorphologyLedger(max_samples=16)
    _ledger_update(ledger, 0.01)
    _ledger_update(ledger, 0.03)

    payload = _fsm_update(
        fsm,
        0.06,
        (_left_event("heel_strike", 0.0),),
    )
    assert payload["state_id"] == float(STANCE_AFTER_HS)
    assert payload["last_valid_hs_time_s"] == 0.0
    journal = payload["accepted_transitions_this_step"]
    assert len(journal) == 1
    assert journal[0]["event"] == "heel_strike"
    assert journal[0]["event_time_s"] == 0.0

    update = _ledger_update(ledger, 0.06, transitions=journal)
    assert update.discarded_sample_count == 0
    assert update.active_segment_type == "stance"
    assert update.active_segment_start_time_s == 0.0
    assert update.pending_sample_count == 3


@_test
def test_wrapper_rejects_effects_without_explicit_experimental_opt_in() -> None:
    _assert_raises(
        ValueError,
        lambda: reward_function.RewardShapingWrapper(
            _ScriptedEnv(()),
            reward_function.RewardConfig(
                morphology_phase_mode=EXPERIMENTAL_PHASE_MODE,
                morphology_profile="",
            ),
        ),
        contains=(
            "experimental completed-segment morphology requires a non-empty "
            "event-warped morphology_profile"
        ),
    )

    with tempfile.TemporaryDirectory(prefix="morphology-wrapper-guard-") as directory:
        profile = _synthetic_profile(Path(directory) / "profile.json")
        configs = (
            _wrapper_config(
                profile,
                morphology_weight=0.01,
                allow_effects=False,
            ),
            _wrapper_config(
                profile,
                hard_termination_enabled=True,
                allow_effects=False,
            ),
        )
        for config in configs:
            _assert_raises(
                ValueError,
                lambda config=config: reward_function.RewardShapingWrapper(
                    _ScriptedEnv(()),
                    config,
                ),
                contains="experimental completed-segment morphology is shadow-only",
            )

        # The diagnostic-only contract remains constructible without the opt-in.
        shadow = reward_function.RewardShapingWrapper(
            _ScriptedEnv(()),
            _wrapper_config(profile, allow_effects=False),
        )
        assert shadow._experimental_morphology_ledger is not None


@_test
def test_wrapper_weight_zero_matches_current_reward_exactly_on_settlement() -> None:
    samples = (
        (1.06, 0.30, 0.00, (_left_event("heel_strike", 1.00),)),
        (1.20, 0.20, 0.00, ()),
        (1.40, 0.10, 0.00, ()),
        (1.51, 0.90, 0.00, (_left_event("toe_off", 1.50),)),
        (1.70, 0.00, 0.00, ()),
    )
    infos = _scripted_infos(samples)
    with tempfile.TemporaryDirectory(prefix="morphology-wrapper-zero-") as directory:
        profile = _synthetic_profile(Path(directory) / "profile.json")
        common = _wrapper_config(profile, morphology_weight=0.0)
        current_outputs = _run_wrapper(
            infos,
            replace(common, morphology_phase_mode="event_anchored"),
        )
        experimental_outputs = _run_wrapper(infos, common)

    current_rewards = [output[1] for output in current_outputs]
    experimental_rewards = [output[1] for output in experimental_outputs]
    assert current_rewards == experimental_rewards
    assert current_rewards == [1.0] * len(samples)

    for current, experimental in zip(current_outputs, experimental_outputs):
        assert current[2:4] == experimental[2:4] == (False, False)
        assert current[4]["reward_components"]["morphology_term"] == 0.0
        assert experimental[4]["reward_components"]["morphology_term"] == 0.0

    settlement_info = experimental_outputs[3][4]
    assert settlement_info["reward_terms"]["morphology_settled_this_step"] == 1.0
    assert settlement_info["reward_terms"]["morphology_settled_sample_count"] == 3.0
    assert settlement_info["reward_terms"]["morphology_loss"] > 0.0
    assert settlement_info["reward_components"]["morphology_term"] == 0.0


@_test
def test_wrapper_positive_weight_applies_exact_loss_sum_only_at_closure() -> None:
    samples = (
        (1.06, 0.30, 0.00, (_left_event("heel_strike", 1.00),)),
        (1.20, 0.20, 0.00, ()),
        (1.40, 0.10, 0.00, ()),
        # This large value is after the physical TO and must remain pending.
        (1.51, 0.90, 0.00, (_left_event("toe_off", 1.50),)),
    )
    infos = _scripted_infos(samples)
    weight = 0.2
    with tempfile.TemporaryDirectory(prefix="morphology-wrapper-weight-") as directory:
        profile = _synthetic_profile(Path(directory) / "profile.json")
        outputs = _run_wrapper(
            infos,
            _wrapper_config(
                profile,
                morphology_weight=weight,
                allow_effects=True,
            ),
        )

    # Flat corridor [-0.1, +0.1], width 0.2.  Only knee is outside:
    # q=0.3 -> joint loss 1.0; q=0.2 -> 0.25; q=0.1 -> 0.0.
    # The combined per-sample morphology loss averages knee/ankle, hence 0.625.
    expected_loss_sum = 0.5 * (1.0 + 0.25 + 0.0)
    expected_term = weight * expected_loss_sum
    assert [output[1] for output in outputs[:3]] == [1.0, 1.0, 1.0]
    _assert_close(outputs[3][1], 1.0 - expected_term)

    closure_terms = outputs[3][4]["reward_terms"]
    closure_components = outputs[3][4]["reward_components"]
    _assert_close(closure_terms["morphology_loss"], expected_loss_sum)
    _assert_close(closure_terms["morphology_loss_mean"], expected_loss_sum / 3.0)
    _assert_close(closure_components["morphology_term"], expected_term)
    assert closure_terms["morphology_settled_sample_count"] == 3.0
    assert closure_terms["morphology_pending_sample_count"] == 1.0
    # The post-TO q=0.9 sample is not charged in the stance settlement.
    assert closure_terms["morphology_knee_excursion_rad"] < 0.21


@_test
def test_wrapper_hard_bounds_are_inclusive_and_termination_is_delayed() -> None:
    samples = (
        # Every stance value lies exactly on or inside an inclusive hard bound.
        (1.06, 0.30, 0.20, (_left_event("heel_strike", 1.00),)),
        (1.20, -0.30, -0.20, ()),
        (1.40, 0.00, 0.20, ()),
        # This violation occurs after the backdated TO and opens the swing.
        (1.51, 0.31, 0.00, (_left_event("toe_off", 1.50),)),
        (1.70, 0.00, -0.21, ()),
        (2.00, 0.00, 0.00, ()),
        # The hard violation is discovered only when this HS closes swing.
        (2.06, 0.00, 0.00, (_left_event("heel_strike", 2.00),)),
    )
    infos = _scripted_infos(samples)
    hard_min = (-0.30, -0.20)
    hard_max = (0.30, 0.20)

    with tempfile.TemporaryDirectory(prefix="morphology-wrapper-hard-") as directory:
        profile = _synthetic_profile(Path(directory) / "profile.json")
        disabled = _run_wrapper(
            infos,
            _wrapper_config(
                profile,
                hard_termination_enabled=False,
                hard_q_min=hard_min,
                hard_q_max=hard_max,
            ),
        )
        enabled = _run_wrapper(
            infos,
            _wrapper_config(
                profile,
                hard_termination_enabled=True,
                allow_effects=True,
                hard_q_min=hard_min,
                hard_q_max=hard_max,
            ),
        )

    # Closing stance proves equality with all four hard boundaries is valid.
    stance_close = enabled[3]
    assert stance_close[2] is False
    assert stance_close[3] is False
    assert stance_close[4]["reward_terms"]["morphology_hard_violation"] == 0.0
    assert stance_close[4]["end_reason"] is None

    # Out-of-hard samples are buffered during swing and cannot terminate early.
    for output in enabled[3:6]:
        assert output[2] is False
        assert output[4]["reward_terms"]["morphology_hard_violation"] == 0.0

    swing_close = enabled[6]
    assert swing_close[2] is True
    assert swing_close[3] is False
    assert swing_close[4]["end_reason"] == (
        "morphology_hard_violation:completed_segment"
    )
    hard_terms = swing_close[4]["reward_terms"]
    assert hard_terms["morphology_hard_violation"] == 1.0
    _assert_close(hard_terms["morphology_hard_knee_excursion_rad"], 0.01)
    _assert_close(hard_terms["morphology_hard_ankle_excursion_rad"], 0.01)
    _assert_close(hard_terms["morphology_hard_max_excursion_rad"], 0.01)

    # The disabled flag never changes episode control, while diagnostics remain.
    assert all(output[2] is False and output[3] is False for output in disabled)
    assert disabled[6][4]["end_reason"] is None
    assert disabled[6][4]["reward_terms"]["morphology_hard_violation"] == 1.0


@_test
def test_wrapper_hard_termination_preserves_infrastructure_failure_reason() -> None:
    samples = (
        (1.06, 0.30, 0.20, (_left_event("heel_strike", 1.00),)),
        (1.20, -0.30, -0.20, ()),
        (1.40, 0.00, 0.20, ()),
        (1.51, 0.31, 0.00, (_left_event("toe_off", 1.50),)),
        (1.70, 0.00, -0.21, ()),
        (2.00, 0.00, 0.00, ()),
        # This accepted HS closes the violating swing on the same step as an
        # infrastructure truncation.  Morphology may report the violation but
        # must not hide the causal end reason supplied by the environment.
        (2.06, 0.00, 0.00, (_left_event("heel_strike", 2.00),)),
    )
    base_infos = _scripted_infos(samples)
    hard_min = (-0.30, -0.20)
    hard_max = (0.30, 0.20)

    with tempfile.TemporaryDirectory(
        prefix="morphology-wrapper-precedence-"
    ) as directory:
        profile = _synthetic_profile(Path(directory) / "profile.json")
        config = _wrapper_config(
            profile,
            hard_termination_enabled=True,
            allow_effects=True,
            hard_q_min=hard_min,
            hard_q_max=hard_max,
        )
        for failure_reason in ("numerical_failure", "step_wall_timeout"):
            infos = copy.deepcopy(base_infos)
            infos[-1]["end_reason"] = failure_reason
            infos[-1]["reward_terms"]["truncated"] = 1.0
            infos[-1]["_env_truncated"] = True
            outputs = _run_wrapper(infos, config)

            closing_step = outputs[-1]
            assert closing_step[2] is False
            assert closing_step[3] is True
            assert closing_step[4]["end_reason"] == failure_reason
            terms = closing_step[4]["reward_terms"]
            assert terms["morphology_hard_violation"] == 1.0
            assert terms["terminated"] == 0.0
            assert terms["truncated"] == 1.0


if __name__ == "__main__":
    for test in TESTS:
        test()
        print(f"[PASS] {test.__name__}")
    print("ALL EXPERIMENTAL RETROSPECTIVE MORPHOLOGY TESTS PASSED " f"({len(TESTS)})")
