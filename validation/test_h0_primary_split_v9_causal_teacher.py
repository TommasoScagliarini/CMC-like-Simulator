from __future__ import annotations

import numpy as np
import pytest

from h0_primary_split_v9_causal_teacher import (
    CAUSAL_INDICES,
    V9CausalTeacherError,
    assert_causal_pair,
    from_current_shadow,
    from_replay_views,
)


def _student() -> np.ndarray:
    return np.arange(35, dtype=np.float32) / np.float32(35.0)


def test_replay_teacher_changes_only_load_and_contact() -> None:
    student = _student()
    legacy = student.copy()
    legacy[10:25] = np.arange(15, dtype=np.float32) + np.float32(10.0)
    teacher = from_replay_views(student, legacy)
    assert teacher[10:12].tobytes() == legacy[10:12].tobytes()
    assert teacher[list(CAUSAL_INDICES)].tobytes() == student[
        list(CAUSAL_INDICES)
    ].tobytes()


def test_replay_teacher_is_invariant_to_all_legacy_nonprimary_fields() -> None:
    student = _student()
    first = student.copy()
    second = student.copy()
    first[10:12] = (0.2, 1.0)
    second[10:12] = first[10:12]
    second[list(CAUSAL_INDICES)] *= np.float32(-100.0)
    assert from_replay_views(student, first).tobytes() == from_replay_views(
        student, second
    ).tobytes()


def test_current_shadow_uses_same_state_analog_load_contact_only() -> None:
    student = _student()
    info = {
        "online_grf_detector": {
            "left": {"normal_force": 80.0, "in_contact": True}
        }
    }
    teacher = from_current_shadow(student, info, body_weight_n=800.0)
    assert teacher[10] == np.float32(0.1)
    assert teacher[11] == np.float32(1.0)
    assert teacher[list(CAUSAL_INDICES)].tobytes() == student[
        list(CAUSAL_INDICES)
    ].tobytes()


def test_reset_requires_both_shadow_streams_empty() -> None:
    student = _student()
    teacher = from_current_shadow(
        student,
        {"online_grf_detector": {}},
        body_weight_n=800.0,
        reset_boundary=True,
    )
    assert teacher[10] == np.float32(0.0)
    assert teacher[11] == np.float32(0.0)
    with pytest.raises(V9CausalTeacherError):
        from_current_shadow(
            student,
            {"online_grf_detector": {}},
            body_weight_n=800.0,
            reset_boundary=False,
        )


def test_causal_pair_rejects_event_or_fsm_mutation() -> None:
    student = _student()
    for index in (12, 17, 24, 34):
        teacher = student.copy()
        teacher[index] += np.float32(1.0)
        with pytest.raises(V9CausalTeacherError):
            assert_causal_pair(student, teacher)


@pytest.mark.parametrize("bad", [np.nan, np.inf, -1.0])
def test_current_shadow_rejects_invalid_force(bad: float) -> None:
    with pytest.raises(V9CausalTeacherError):
        from_current_shadow(
            _student(),
            {
                "online_grf_detector": {
                    "left": {"normal_force": bad, "in_contact": True}
                }
            },
            body_weight_n=800.0,
        )
