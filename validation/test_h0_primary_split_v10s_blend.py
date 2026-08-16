from __future__ import annotations

import numpy as np
import pytest

import h0_primary_split_v10s_blend as blend


def _vector(first: float, second: float) -> np.ndarray:
    return np.asarray([first, second], dtype=np.float32)


def test_alpha_is_student_weight_and_noise_is_added_after_blending() -> None:
    student = _vector(1.0, -0.5)
    teacher = _vector(-1.0, 0.5)
    noise = _vector(0.1, -0.2)
    result = blend.select_safe_dagger_action(
        student,
        teacher,
        noise,
        requested_alpha=0.25,
        latch_state=blend.SafetyLatchState(),
        previous_penetration_m=0.0,
        active_v26_phase="WAIT_HS",
    )

    expected_mean = np.asarray([-0.5, 0.25], dtype=np.float32)
    expected_action = np.asarray([-0.4, 0.05], dtype=np.float32)
    assert result.requested_alpha == 0.25
    assert result.effective_alpha == 0.25
    assert result.blended_mean.tobytes() == expected_mean.tobytes()
    assert result.action == pytest.approx(expected_action, abs=1.0e-7)
    assert result.action.dtype == np.dtype("float32")
    assert result.safety_intervened is False


def test_latch_activates_at_boundary_and_forces_the_teacher_mean() -> None:
    teacher = _vector(-0.2, 0.3)
    result = blend.select_safe_dagger_action(
        _vector(0.9, -0.9),
        teacher,
        _vector(0.0, 0.0),
        requested_alpha=0.75,
        latch_state=blend.SafetyLatchState(),
        previous_penetration_m=0.015,
        active_v26_phase="STANCE_AFTER_HS",
    )

    assert result.latch_entered is True
    assert result.latch_released is False
    assert result.safety_intervened is True
    assert result.effective_alpha == 0.0
    assert result.blended_mean.tobytes() == teacher.tobytes()
    assert result.latch_state == blend.SafetyLatchState(
        active=True,
        activation_count=1,
        release_count=0,
        intervention_action_count=1,
    )


def test_latch_hysteresis_requires_low_penetration_and_active_v26_swing() -> None:
    active = blend.SafetyLatchState(
        active=True,
        activation_count=1,
        intervention_action_count=3,
    )
    not_low = blend.advance_safety_latch(
        active,
        previous_penetration_m=0.010001,
        active_v26_phase="SWING_AFTER_TO",
    )
    assert not_low.state.active is True
    assert not_low.state.intervention_action_count == 4

    wrong_phase = blend.advance_safety_latch(
        not_low.state,
        previous_penetration_m=0.010,
        active_v26_phase="STANCE_AFTER_HS",
    )
    assert wrong_phase.state.active is True
    assert wrong_phase.state.intervention_action_count == 5

    released = blend.advance_safety_latch(
        wrong_phase.state,
        previous_penetration_m=0.010,
        active_v26_phase="SWING_AFTER_TO",
    )
    assert released.released is True
    assert released.intervened is False
    assert released.state == blend.SafetyLatchState(
        active=False,
        activation_count=1,
        release_count=1,
        intervention_action_count=5,
    )


def test_activation_has_priority_over_swing_release() -> None:
    active = blend.SafetyLatchState(
        active=True,
        activation_count=1,
        intervention_action_count=1,
    )
    transition = blend.advance_safety_latch(
        active,
        previous_penetration_m=0.020,
        active_v26_phase="SWING_AFTER_TO",
    )
    assert transition.state.active is True
    assert transition.released is False
    assert transition.state.intervention_action_count == 2


@pytest.mark.parametrize(
    ("student", "teacher", "noise", "alpha", "penetration", "phase"),
    [
        (_vector(0, 0).astype(np.float64), _vector(0, 0), _vector(0, 0), 0.5, 0.0, "WAIT_HS"),
        (_vector(0, 0), _vector(np.nan, 0), _vector(0, 0), 0.5, 0.0, "WAIT_HS"),
        (_vector(0, 0), _vector(0, 0), _vector(0, 0), 1.1, 0.0, "WAIT_HS"),
        (_vector(0, 0), _vector(0, 0), _vector(0, 0), 0.5, np.inf, "WAIT_HS"),
        (_vector(0, 0), _vector(0, 0), _vector(0, 0), 0.5, 0.0, "legacy"),
    ],
)
def test_malformed_or_nonfinite_inputs_fail_closed(
    student,
    teacher,
    noise,
    alpha,
    penetration,
    phase,
) -> None:
    with pytest.raises(blend.V10SBlendError):
        blend.select_safe_dagger_action(
            student,
            teacher,
            noise,
            requested_alpha=alpha,
            latch_state=blend.SafetyLatchState(),
            previous_penetration_m=penetration,
            active_v26_phase=phase,
        )


def test_noise_shape_and_latch_state_are_strict() -> None:
    with pytest.raises(blend.V10SBlendError, match="shape"):
        blend.apply_single_noise(_vector(0, 0), np.zeros(3, dtype=np.float32))
    with pytest.raises(blend.V10SBlendError, match="inconsistent"):
        blend.SafetyLatchState(active=True)
    with pytest.raises(blend.V10SBlendError, match="releases exceed"):
        blend.SafetyLatchState(
            active=False,
            activation_count=0,
            release_count=1,
        )
