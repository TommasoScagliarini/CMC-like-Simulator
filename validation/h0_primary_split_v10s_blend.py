"""Pure action blending and safety-latch helpers for V10S safe DAgger.

``alpha`` is the student weight.  The behavior mean is therefore computed as
``(1 - alpha) * teacher + alpha * student``.  If the diagnostic safety latch is
active, the effective student weight is zero and the teacher mean is served.
Exactly one already-sampled noise vector is added *after* the means are mixed.

The helper has no simulator, model, random-number-generator, or file-system
dependency.  In particular it never clips an action and cannot relax the
independent physical rollout gates.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np


ACTION_DIM = 2
ACTION_DTYPE = np.dtype("float32")
SAFETY_LATCH_ACTIVATION_M = 0.015
SAFETY_LATCH_RELEASE_M = 0.010
SAFETY_LATCH_RELEASE_PHASE = "SWING_AFTER_TO"
V26_PHASE_NAMES = frozenset(
    {
        "WAIT_HS",
        "STANCE_AFTER_HS",
        "SWING_AFTER_TO",
        "VALID_CYCLE_COMPLETED",
        "TIMEOUT",
        "INVALID_EVENT",
    }
)


class V10SBlendError(ValueError):
    """Raised when a V10S action cannot be built without fallback."""


def _finite_number(value: Any, *, label: str) -> float:
    if isinstance(value, (bool, np.bool_)):
        raise V10SBlendError(f"{label} must be numeric")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise V10SBlendError(f"{label} must be numeric") from exc
    if not math.isfinite(result):
        raise V10SBlendError(f"{label} must be finite")
    return result


def _alpha(value: Any, *, label: str = "requested_alpha") -> float:
    result = _finite_number(value, label=label)
    if not 0.0 <= result <= 1.0:
        raise V10SBlendError(f"{label} must lie in [0, 1]")
    return result


def _action_vector(value: Any, *, label: str) -> np.ndarray:
    raw = np.asarray(value)
    if raw.shape != (ACTION_DIM,):
        raise V10SBlendError(f"{label} must have shape ({ACTION_DIM},)")
    if raw.dtype != ACTION_DTYPE:
        raise V10SBlendError(f"{label} must have dtype float32")
    result = np.ascontiguousarray(raw.copy())
    if not np.all(np.isfinite(result)):
        raise V10SBlendError(f"{label} must be finite")
    return result


def _phase_name(value: Any) -> str:
    if not isinstance(value, str) or value not in V26_PHASE_NAMES:
        raise V10SBlendError("active_v26_phase is not a recognized V26 state")
    return value


@dataclass(frozen=True)
class SafetyLatchState:
    """Immutable state carried between policy boundaries."""

    active: bool = False
    activation_count: int = 0
    release_count: int = 0
    intervention_action_count: int = 0

    def __post_init__(self) -> None:
        if type(self.active) is not bool:
            raise V10SBlendError("SafetyLatchState.active must be boolean")
        for name in (
            "activation_count",
            "release_count",
            "intervention_action_count",
        ):
            value = getattr(self, name)
            if type(value) is not int or value < 0:
                raise V10SBlendError(
                    f"SafetyLatchState.{name} must be a nonnegative integer"
                )
        if self.release_count > self.activation_count:
            raise V10SBlendError("safety latch releases exceed activations")
        expected_delta = 1 if self.active else 0
        if self.activation_count - self.release_count != expected_delta:
            raise V10SBlendError("safety latch counters are inconsistent")


@dataclass(frozen=True)
class SafetyLatchTransition:
    """One pure update driven by the preceding step's penetration."""

    state: SafetyLatchState
    entered: bool
    released: bool
    intervened: bool
    previous_penetration_m: float
    active_v26_phase: str


@dataclass(frozen=True)
class SafeDaggerAction:
    """Auditable result of mean blending followed by one noise application."""

    action: np.ndarray
    blended_mean: np.ndarray
    requested_alpha: float
    effective_alpha: float
    latch_state: SafetyLatchState
    latch_entered: bool
    latch_released: bool
    safety_intervened: bool


def advance_safety_latch(
    state: SafetyLatchState,
    *,
    previous_penetration_m: Any,
    active_v26_phase: Any,
) -> SafetyLatchTransition:
    """Advance the latch without inspecting the action about to be served.

    Activation has priority at ``>= 15 mm``.  Once active, the latch releases
    only at ``<= 10 mm`` while the *active V26* phase is ``SWING_AFTER_TO``.
    The input is explicitly the previous completed step's penetration, keeping
    the intervention causal.
    """

    if not isinstance(state, SafetyLatchState):
        raise V10SBlendError("state must be SafetyLatchState")
    penetration = _finite_number(
        previous_penetration_m,
        label="previous_penetration_m",
    )
    if penetration < 0.0:
        raise V10SBlendError("previous_penetration_m must be nonnegative")
    phase = _phase_name(active_v26_phase)

    entered = False
    released = False
    active = state.active
    activations = state.activation_count
    releases = state.release_count

    if penetration >= SAFETY_LATCH_ACTIVATION_M:
        if not active:
            active = True
            entered = True
            activations += 1
    elif (
        active
        and penetration <= SAFETY_LATCH_RELEASE_M
        and phase == SAFETY_LATCH_RELEASE_PHASE
    ):
        active = False
        released = True
        releases += 1

    interventions = state.intervention_action_count + int(active)
    next_state = SafetyLatchState(
        active=active,
        activation_count=activations,
        release_count=releases,
        intervention_action_count=interventions,
    )
    return SafetyLatchTransition(
        state=next_state,
        entered=entered,
        released=released,
        intervened=active,
        previous_penetration_m=penetration,
        active_v26_phase=phase,
    )


def effective_student_alpha(
    requested_alpha: Any,
    *,
    latch_state: SafetyLatchState,
) -> float:
    """Return zero during an intervention, otherwise the requested weight."""

    alpha = _alpha(requested_alpha)
    if not isinstance(latch_state, SafetyLatchState):
        raise V10SBlendError("latch_state must be SafetyLatchState")
    return 0.0 if latch_state.active else alpha


def blend_policy_means(
    student_mean: Any,
    teacher_mean: Any,
    *,
    requested_alpha: Any,
    latch_state: SafetyLatchState,
) -> tuple[np.ndarray, float]:
    """Blend two float32 means; ``alpha`` is always the student weight."""

    student = _action_vector(student_mean, label="student_mean")
    teacher = _action_vector(teacher_mean, label="teacher_mean")
    alpha = effective_student_alpha(
        requested_alpha,
        latch_state=latch_state,
    )
    alpha32 = np.float32(alpha)
    one_minus_alpha32 = np.float32(1.0) - alpha32
    blended = np.add(
        np.multiply(teacher, one_minus_alpha32, dtype=np.float32),
        np.multiply(student, alpha32, dtype=np.float32),
        dtype=np.float32,
    )
    if not np.all(np.isfinite(blended)):  # pragma: no cover - guarded inputs
        raise V10SBlendError("blended_mean is non-finite")
    return np.ascontiguousarray(blended), alpha


def apply_single_noise(mean: Any, noise: Any) -> np.ndarray:
    """Add one externally generated float32 noise vector after mean blending."""

    base = _action_vector(mean, label="blended_mean")
    one_noise = _action_vector(noise, label="single_noise")
    action = np.add(base, one_noise, dtype=np.float32)
    if not np.all(np.isfinite(action)):
        raise V10SBlendError("action is non-finite")
    return np.ascontiguousarray(action)


def select_safe_dagger_action(
    student_mean: Any,
    teacher_mean: Any,
    single_noise: Any,
    *,
    requested_alpha: Any,
    latch_state: SafetyLatchState,
    previous_penetration_m: Any,
    active_v26_phase: Any,
) -> SafeDaggerAction:
    """Advance the latch, blend means, then apply exactly one noise vector."""

    requested = _alpha(requested_alpha)
    transition = advance_safety_latch(
        latch_state,
        previous_penetration_m=previous_penetration_m,
        active_v26_phase=active_v26_phase,
    )
    blended, effective = blend_policy_means(
        student_mean,
        teacher_mean,
        requested_alpha=requested,
        latch_state=transition.state,
    )
    action = apply_single_noise(blended, single_noise)
    return SafeDaggerAction(
        action=action,
        blended_mean=blended,
        requested_alpha=requested,
        effective_alpha=effective,
        latch_state=transition.state,
        latch_entered=transition.entered,
        latch_released=transition.released,
        safety_intervened=transition.intervened,
    )


__all__ = [
    "ACTION_DIM",
    "ACTION_DTYPE",
    "SAFETY_LATCH_ACTIVATION_M",
    "SAFETY_LATCH_RELEASE_M",
    "SAFETY_LATCH_RELEASE_PHASE",
    "V26_PHASE_NAMES",
    "SafeDaggerAction",
    "SafetyLatchState",
    "SafetyLatchTransition",
    "V10SBlendError",
    "advance_safety_latch",
    "apply_single_noise",
    "blend_policy_means",
    "effective_student_alpha",
    "select_safe_dagger_action",
]
