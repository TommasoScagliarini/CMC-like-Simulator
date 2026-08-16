"""Pure construction helpers for the V9 event-causal privileged teacher.

The deployable actor always receives the current V26 actor observation.  The
teacher is allowed to replace only the continuous load/contact fields 10/11
with the force-free analog shadow values at the same physical state.  Every
event, gait-phase, FSM, kinematic, SEA, and served-controller field remains
byte-exact to the V26 student view.
"""

from __future__ import annotations

from typing import Any, Mapping, Sequence

import numpy as np


ACTOR_FEATURE_COUNT = 35
LOAD_INDEX = 10
CONTACT_INDEX = 11
PRIVILEGED_INDICES = (LOAD_INDEX, CONTACT_INDEX)
CAUSAL_INDICES = tuple(
    index for index in range(ACTOR_FEATURE_COUNT) if index not in PRIVILEGED_INDICES
)


class V9CausalTeacherError(RuntimeError):
    pass


def _actor(value: Any, *, label: str) -> np.ndarray:
    array = np.ascontiguousarray(np.asarray(value, dtype=np.float32))
    if array.shape != (ACTOR_FEATURE_COUNT,) or not np.all(np.isfinite(array)):
        raise V9CausalTeacherError(
            f"{label} must be finite float32 ({ACTOR_FEATURE_COUNT},)"
        )
    return array


def _finite_nonnegative(value: Any, *, label: str) -> float:
    if isinstance(value, (bool, np.bool_)):
        raise V9CausalTeacherError(f"{label} must be numeric")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise V9CausalTeacherError(f"{label} must be numeric") from exc
    if not np.isfinite(result) or result < 0.0:
        raise V9CausalTeacherError(f"{label} must be finite and nonnegative")
    return result


def from_replay_views(
    student_v26: Any,
    legacy_reference: Any,
) -> np.ndarray:
    """Relabel one gated replay state without importing legacy events/FSM."""

    student = _actor(student_v26, label="student_v26")
    reference = _actor(legacy_reference, label="legacy_reference")
    teacher = student.copy()
    teacher[list(PRIVILEGED_INDICES)] = reference[list(PRIVILEGED_INDICES)]
    assert_causal_pair(student, teacher)
    return teacher


def from_current_shadow(
    student_v26: Any,
    info: Mapping[str, Any],
    *,
    body_weight_n: float,
    reset_boundary: bool = False,
) -> np.ndarray:
    """Build a same-state DAgger label view from the force-free analog shadow."""

    student = _actor(student_v26, label="student_v26")
    weight = _finite_nonnegative(body_weight_n, label="body_weight_n")
    if weight <= 0.0:
        raise V9CausalTeacherError("body_weight_n must be positive")
    stream = info.get("online_grf_detector")
    if not isinstance(stream, Mapping):
        raise V9CausalTeacherError("online_grf_detector must be an object")
    left = stream.get("left")
    if left is None and reset_boundary and not stream:
        load_bw = 0.0
        contact = False
    else:
        if not isinstance(left, Mapping):
            raise V9CausalTeacherError("online_grf_detector.left is missing")
        load_bw = _finite_nonnegative(
            left.get("normal_force"), label="online_grf_detector.left.normal_force"
        ) / weight
        raw_contact = left.get("in_contact")
        if not isinstance(raw_contact, (bool, np.bool_)):
            raise V9CausalTeacherError(
                "online_grf_detector.left.in_contact must be boolean"
            )
        contact = bool(raw_contact)
    teacher = student.copy()
    teacher[LOAD_INDEX] = np.float32(load_bw)
    teacher[CONTACT_INDEX] = np.float32(contact)
    assert_causal_pair(student, teacher)
    return teacher


def assert_causal_pair(student_v26: Any, teacher: Any) -> None:
    """Fail closed unless only load/contact can differ from the V26 input."""

    student = _actor(student_v26, label="student_v26")
    target = _actor(teacher, label="teacher")
    if (
        target[list(CAUSAL_INDICES)].tobytes(order="C")
        != student[list(CAUSAL_INDICES)].tobytes(order="C")
    ):
        raise V9CausalTeacherError(
            "teacher changed a V26 causal field outside indices 10/11"
        )


def causal_columns_byte_exact(student_v26: Any, teacher: Any) -> bool:
    try:
        assert_causal_pair(student_v26, teacher)
    except V9CausalTeacherError:
        return False
    return True


def validate_actor_feature_names(names: Sequence[str]) -> None:
    ordered = tuple(str(name) for name in names)
    if len(ordered) != ACTOR_FEATURE_COUNT:
        raise V9CausalTeacherError("actor feature layout must contain 35 fields")
    if ordered[LOAD_INDEX] != "online_left_normal_grf_bw":
        raise V9CausalTeacherError("load feature is not frozen at index 10")
    if ordered[CONTACT_INDEX] != "online_left_in_contact":
        raise V9CausalTeacherError("contact feature is not frozen at index 11")


__all__ = [
    "ACTOR_FEATURE_COUNT",
    "CAUSAL_INDICES",
    "CONTACT_INDEX",
    "LOAD_INDEX",
    "PRIVILEGED_INDICES",
    "V9CausalTeacherError",
    "assert_causal_pair",
    "causal_columns_byte_exact",
    "from_current_shadow",
    "from_replay_views",
    "validate_actor_feature_names",
]
