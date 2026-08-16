from __future__ import annotations

import copy
import sys
from pathlib import Path

import numpy as np
import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
if str(TRAJECTORY_ROOT) not in sys.path:
    sys.path.insert(0, str(TRAJECTORY_ROOT))

from prosthetic_phase_fsm import (  # noqa: E402
    ProstheticPhaseFSM,
    ProstheticPhaseFSMConfig,
)
from h0_primary_split_v10_coherent_teacher import (  # noqa: E402
    EXPECTED_ACTOR_FEATURE_NAMES,
    LegacyGaitShadow,
    TEACHER_BLOCK_STOP,
    UNCHANGED_INDICES,
    V10CoherentTeacherError,
    build_teacher_view,
)


BODY_WEIGHT_N = 800.0


def _student() -> np.ndarray:
    return np.linspace(-0.75, 0.75, 35, dtype=np.float32)


def _shadow() -> LegacyGaitShadow:
    return LegacyGaitShadow(
        ProstheticPhaseFSM(
            ProstheticPhaseFSMConfig(event_source="legacy_events")
        )
    )


def _info(
    time_s: float,
    *,
    force_n: float | None = None,
    contact: bool | None = None,
    events: list[dict] | None = None,
) -> dict:
    detector: dict = {}
    if force_n is not None or contact is not None:
        detector = {
            "left": {
                "normal_force": force_n,
                "in_contact": contact,
            }
        }
    return {
        "time": time_s,
        "online_grf_detector": detector,
        "legacy_online_events": [] if events is None else events,
        "observation": {
            "pros_knee_angle": 0.15,
            "pros_ankle_angle": -0.05,
        },
    }


def _build(
    shadow: LegacyGaitShadow,
    student: np.ndarray,
    info: dict,
    *,
    reset: bool = False,
) -> np.ndarray:
    return build_teacher_view(
        student,
        EXPECTED_ACTOR_FEATURE_NAMES,
        info,
        body_weight_n=BODY_WEIGHT_N,
        shadow=shadow,
        reset_boundary=reset,
    )


def _event(
    name: str,
    *,
    onset: float,
    confirmed: float,
    delivered: float,
    cycle: float | None = None,
) -> dict:
    event = {
        "side": "left",
        "event": name,
        "time": onset,
        "event_time_s": onset,
        "confirmed_time": confirmed,
        "confirmed_time_s": confirmed,
        "delivered_time_s": delivered,
    }
    if name == "heel_strike":
        event["cycle_duration_s"] = cycle
    if name == "toe_off":
        event["contact_duration_s"] = 0.49
    return event


def test_reset_replaces_exactly_the_inclusive_10_to_24_block() -> None:
    shadow = _shadow()
    student = _student()
    before = student.tobytes(order="C")
    teacher = _build(shadow, student, _info(0.0), reset=True)

    expected_block = np.asarray(
        [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            1.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ],
        dtype=np.float32,
    )
    assert student.tobytes(order="C") == before
    assert teacher[10:TEACHER_BLOCK_STOP].tobytes() == expected_block.tobytes()
    assert teacher[list(UNCHANGED_INDICES)].tobytes() == student[
        list(UNCHANGED_INDICES)
    ].tobytes()


def test_gait_phase_uses_event_onset_not_confirmation_or_delivery() -> None:
    shadow = _shadow()
    student = _student()
    _build(shadow, student, _info(0.0), reset=True)

    first_hs = _event(
        "heel_strike",
        onset=0.01,
        confirmed=0.06,
        delivered=0.06,
    )
    first = _build(
        shadow,
        student,
        _info(0.06, force_n=80.0, contact=True, events=[first_hs]),
    )
    assert first[10] == np.float32(0.1)
    assert first[11] == np.float32(1.0)
    assert first[12] == np.float32(1.0)
    assert first[13] == np.float32(0.0)

    toe_off = _event(
        "toe_off",
        onset=0.50,
        confirmed=0.50,
        delivered=0.51,
    )
    second = _build(
        shadow,
        student,
        _info(0.51, force_n=0.0, contact=False, events=[toe_off]),
    )
    assert second[12] == np.float32(0.0)
    assert second[13] == np.float32(1.0)

    next_hs = _event(
        "heel_strike",
        onset=1.01,
        confirmed=1.06,
        delivered=1.07,
        cycle=1.0,
    )
    third = _build(
        shadow,
        student,
        _info(1.07, force_n=160.0, contact=True, events=[next_hs]),
    )
    expected_phase = (1.07 - 1.01) / 1.0
    assert third[12] == np.float32(1.0)
    assert third[14] == pytest.approx(
        np.float32(np.sin(2.0 * np.pi * expected_phase)), abs=1.0e-7
    )
    assert third[15] == pytest.approx(
        np.float32(np.cos(2.0 * np.pi * expected_phase)), abs=1.0e-7
    )
    assert third[16] == np.float32(1.0)
    assert third[17:25].tobytes() == np.asarray(
        list(shadow.phase_observation().values()), dtype=np.float32
    ).tobytes()


def test_reset_starts_a_new_episode_without_synthetic_events() -> None:
    shadow = _shadow()
    student = _student()
    _build(shadow, student, _info(5.0), reset=True)
    _build(
        shadow,
        student,
        _info(
            5.06,
            force_n=100.0,
            contact=True,
            events=[
                _event(
                    "heel_strike",
                    onset=5.01,
                    confirmed=5.06,
                    delivered=5.06,
                )
            ],
        ),
    )

    reset = _build(shadow, student, _info(2.0), reset=True)
    assert reset[10:25].tobytes() == np.asarray(
        [0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0],
        dtype=np.float32,
    ).tobytes()


def test_malformed_boundary_is_transactional() -> None:
    shadow = _shadow()
    student = _student()
    _build(shadow, student, _info(0.0), reset=True)

    malformed = _info(0.10, force_n=np.nan, contact=True)
    with pytest.raises(V10CoherentTeacherError, match="must be finite"):
        _build(shadow, student, malformed)

    valid = _build(
        shadow,
        student,
        _info(0.10, force_n=0.0, contact=False),
    )
    assert valid[10] == np.float32(0.0)
    with pytest.raises(V10CoherentTeacherError, match="strictly monotonic"):
        _build(
            shadow,
            student,
            _info(0.10, force_n=0.0, contact=False),
        )


@pytest.mark.parametrize(
    "mutation",
    [
        lambda payload: payload.pop("online_grf_detector"),
        lambda payload: payload["online_grf_detector"].update(
            {"left": {"normal_force": 1.0, "in_contact": 1}}
        ),
        lambda payload: payload["observation"].update(
            {"pros_knee_angle": np.inf}
        ),
        lambda payload: payload.update({"legacy_online_events": {}}),
    ],
)
def test_malformed_info_fails_closed(mutation) -> None:
    shadow = _shadow()
    student = _student()
    _build(shadow, student, _info(0.0), reset=True)
    payload = _info(0.10, force_n=0.0, contact=False)
    mutation(payload)
    with pytest.raises(V10CoherentTeacherError):
        _build(shadow, student, payload)


def test_layout_dtype_and_nonfinite_actor_are_strict() -> None:
    reset = _info(0.0)
    with pytest.raises(V10CoherentTeacherError, match="dtype float32"):
        build_teacher_view(
            _student().astype(np.float64),
            EXPECTED_ACTOR_FEATURE_NAMES,
            reset,
            body_weight_n=BODY_WEIGHT_N,
            shadow=_shadow(),
            reset_boundary=True,
        )

    bad_names = list(EXPECTED_ACTOR_FEATURE_NAMES)
    bad_names[12] = "wrong_hs"
    with pytest.raises(V10CoherentTeacherError, match="frozen 35-field"):
        build_teacher_view(
            _student(),
            bad_names,
            reset,
            body_weight_n=BODY_WEIGHT_N,
            shadow=_shadow(),
            reset_boundary=True,
        )

    nonfinite = _student()
    nonfinite[34] = np.nan
    with pytest.raises(V10CoherentTeacherError, match="must be finite"):
        build_teacher_view(
            nonfinite,
            EXPECTED_ACTOR_FEATURE_NAMES,
            reset,
            body_weight_n=BODY_WEIGHT_N,
            shadow=_shadow(),
            reset_boundary=True,
        )


def test_event_causality_and_cycle_duration_mismatch_fail_closed() -> None:
    shadow = _shadow()
    student = _student()
    _build(shadow, student, _info(0.0), reset=True)
    impossible = _event(
        "heel_strike",
        onset=0.02,
        confirmed=0.06,
        delivered=0.05,
    )
    with pytest.raises(V10CoherentTeacherError):
        _build(
            shadow,
            student,
            _info(0.05, force_n=10.0, contact=True, events=[impossible]),
        )

    first_hs = _event(
        "heel_strike",
        onset=0.01,
        confirmed=0.06,
        delivered=0.06,
    )
    _build(
        shadow,
        student,
        _info(0.06, force_n=10.0, contact=True, events=[first_hs]),
    )
    toe_off = _event(
        "toe_off",
        onset=0.50,
        confirmed=0.50,
        delivered=0.51,
    )
    _build(
        shadow,
        student,
        _info(0.51, force_n=0.0, contact=False, events=[toe_off]),
    )
    wrong_cycle = _event(
        "heel_strike",
        onset=1.01,
        confirmed=1.06,
        delivered=1.07,
        cycle=0.9,
    )
    with pytest.raises(V10CoherentTeacherError, match="disagrees"):
        _build(
            shadow,
            student,
            _info(1.07, force_n=20.0, contact=True, events=[wrong_cycle]),
        )

    correct_cycle = copy.deepcopy(wrong_cycle)
    correct_cycle["cycle_duration_s"] = 1.0
    recovered = _build(
        shadow,
        student,
        _info(1.07, force_n=20.0, contact=True, events=[correct_cycle]),
    )
    assert recovered[16] == np.float32(1.0)


def test_historical_early_event_is_diagnostic_not_a_teacher_failure() -> None:
    shadow = _shadow()
    student = _student()
    _build(shadow, student, _info(0.0), reset=True)
    _build(
        shadow,
        student,
        _info(
            0.06,
            force_n=100.0,
            contact=True,
            events=[
                _event(
                    "heel_strike",
                    onset=0.01,
                    confirmed=0.06,
                    delivered=0.06,
                )
            ],
        ),
    )

    teacher = _build(
        shadow,
        student,
        _info(
            0.07,
            force_n=0.0,
            contact=False,
            events=[
                _event(
                    "toe_off",
                    onset=0.02,
                    confirmed=0.07,
                    delivered=0.07,
                )
            ],
        ),
    )

    payload = shadow.phase_payload()
    assert teacher[13] == np.float32(1.0)
    assert teacher[18] == np.float32(1.0)
    assert payload["invalid_event_this_step"] == 1.0
    assert payload["invalid_event_type"] == "to_too_early_after_hs"
