"""Pure contracts for adapting H0 to the primary-GRF observation split.

This module deliberately contains no OpenSim or RLlib imports.  The validation
driver owns simulation and checkpoint I/O; these helpers make the paired-view
construction, group split, metrics, and fail-closed gates independently
testable on macOS and Windows.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from typing import Any, Mapping, Sequence

import numpy as np


ACTOR_FEATURE_COUNT = 35
ACTION_DIM = 2
PRIMARY_LOAD_FEATURE = "online_left_normal_grf_bw"
PRIMARY_CONTACT_FEATURE = "online_left_in_contact"
LOAD_FEATURE_INDEX = 10
CONTACT_FEATURE_INDEX = 11
PHASE_FEATURES = (
    "phase_fsm_wait_hs",
    "phase_fsm_stance_after_hs",
    "phase_fsm_swing_after_to",
    "phase_expected_hs",
    "phase_expected_to",
    "phase_stance_elapsed_norm",
    "phase_swing_elapsed_norm",
    "phase_cycle_progress_credit",
)
PHASE_FEATURE_INDICES = tuple(range(17, 25))
STUDENT_VIEW = "student_view"
TEACHER_VIEW = "teacher_view"


class PrimaryGRFSplitContractError(RuntimeError):
    """Raised when a paired sample cannot be constructed without fallback."""


@dataclass(frozen=True)
class PairedViews:
    student: np.ndarray
    teacher: np.ndarray
    primary_load_bw: float
    primary_contact: bool
    detector_load_bw: float
    detector_contact: bool


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, (bool, np.bool_)):
        raise PrimaryGRFSplitContractError(f"{label} must be a finite number")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise PrimaryGRFSplitContractError(
            f"{label} must be a finite number"
        ) from exc
    if not math.isfinite(number):
        raise PrimaryGRFSplitContractError(f"{label} must be finite")
    return number


def _left_aggregate(
    info: Mapping[str, Any], key: str, *, allow_empty_reset: bool
) -> Mapping[str, Any] | None:
    stream = info.get(key)
    if not isinstance(stream, Mapping):
        raise PrimaryGRFSplitContractError(f"{key} must be an object")
    left = stream.get("left")
    if left is None and allow_empty_reset and not stream:
        return None
    if not isinstance(left, Mapping):
        raise PrimaryGRFSplitContractError(f"{key}.left aggregate is missing")
    return left


def _load_contact(
    aggregate: Mapping[str, Any] | None,
    *,
    body_weight_n: float,
    label: str,
) -> tuple[float, bool]:
    if aggregate is None:
        return 0.0, False
    normal_force_n = _finite_number(
        aggregate.get("normal_force"), f"{label}.normal_force"
    )
    if normal_force_n < 0.0:
        raise PrimaryGRFSplitContractError(
            f"{label}.normal_force must be nonnegative"
        )
    contact = aggregate.get("in_contact")
    if not isinstance(contact, (bool, np.bool_)):
        raise PrimaryGRFSplitContractError(
            f"{label}.in_contact must be boolean"
        )
    return normal_force_n / body_weight_n, bool(contact)


def build_paired_views(
    observation: Any,
    actor_feature_names: Sequence[str],
    info: Mapping[str, Any],
    *,
    body_weight_n: float,
    reset_boundary: bool = False,
    teacher_phase_observation: Mapping[str, Any] | None = None,
) -> PairedViews:
    """Build student/teacher actor prefixes from one physical state.

    H0 was trained before the primary/detector split: both its load/contact
    fields and the continuous guards of its phase FSM used the analog detector.
    Consequently the counterfactual teacher replaces columns 10/11 and the
    eight phase-FSM observation columns 17:25.  At reset the environment has
    not attributed a contact sample yet, so both streams are empty and the two
    FSMs must expose the same reset state.
    """

    weight = _finite_number(body_weight_n, "body_weight_n")
    if weight <= 0.0:
        raise PrimaryGRFSplitContractError("body_weight_n must be positive")
    names = tuple(str(name) for name in actor_feature_names)
    if len(names) != ACTOR_FEATURE_COUNT:
        raise PrimaryGRFSplitContractError(
            f"actor layout must contain {ACTOR_FEATURE_COUNT} features"
        )
    if (
        names[LOAD_FEATURE_INDEX] != PRIMARY_LOAD_FEATURE
        or names[CONTACT_FEATURE_INDEX] != PRIMARY_CONTACT_FEATURE
    ):
        raise PrimaryGRFSplitContractError(
            "primary load/contact columns are not frozen at indices 10/11"
        )
    if tuple(names[index] for index in PHASE_FEATURE_INDICES) != PHASE_FEATURES:
        raise PrimaryGRFSplitContractError(
            "phase-FSM columns are not frozen at indices 17:25"
        )
    raw = np.asarray(observation)
    if raw.ndim != 1 or raw.shape[0] < ACTOR_FEATURE_COUNT:
        raise PrimaryGRFSplitContractError(
            "observation must contain the complete actor prefix"
        )
    if raw.dtype != np.dtype("float32"):
        raise PrimaryGRFSplitContractError("observation dtype must be float32")
    student = np.ascontiguousarray(raw[:ACTOR_FEATURE_COUNT].copy())
    if not np.all(np.isfinite(student)):
        raise PrimaryGRFSplitContractError("actor observation is non-finite")

    primary = _left_aggregate(
        info, "online_grf", allow_empty_reset=reset_boundary
    )
    detector = _left_aggregate(
        info, "online_grf_detector", allow_empty_reset=reset_boundary
    )
    if reset_boundary and ((primary is None) != (detector is None)):
        raise PrimaryGRFSplitContractError(
            "reset primary and detector streams must both be empty"
        )
    primary_load, primary_contact = _load_contact(
        primary, body_weight_n=weight, label="online_grf.left"
    )
    detector_load, detector_contact = _load_contact(
        detector, body_weight_n=weight, label="online_grf_detector.left"
    )

    expected_primary = np.asarray(
        [primary_load, float(primary_contact)], dtype=np.float32
    )
    actual_primary = student[[LOAD_FEATURE_INDEX, CONTACT_FEATURE_INDEX]]
    if actual_primary.tobytes(order="C") != expected_primary.tobytes(order="C"):
        raise PrimaryGRFSplitContractError(
            "student load/contact do not exactly match the primary GRF"
        )

    teacher = student.copy()
    teacher[LOAD_FEATURE_INDEX] = np.float32(detector_load)
    teacher[CONTACT_FEATURE_INDEX] = np.float32(detector_contact)
    if teacher_phase_observation is None:
        if not reset_boundary:
            raise PrimaryGRFSplitContractError(
                "post-step teacher view requires an independent analog-fed FSM"
            )
    else:
        if set(teacher_phase_observation) != set(PHASE_FEATURES):
            raise PrimaryGRFSplitContractError(
                "teacher phase observation schema mismatch"
            )
        for index, name in zip(PHASE_FEATURE_INDICES, PHASE_FEATURES):
            teacher[index] = np.float32(
                _finite_number(teacher_phase_observation[name], name)
            )
    other = np.ones(ACTOR_FEATURE_COUNT, dtype=bool)
    other[
        [LOAD_FEATURE_INDEX, CONTACT_FEATURE_INDEX, *PHASE_FEATURE_INDICES]
    ] = False
    if teacher[other].tobytes(order="C") != student[other].tobytes(order="C"):
        raise PrimaryGRFSplitContractError(
            "teacher construction changed a feature outside split/FSM columns"
        )
    if not np.all(np.isfinite(teacher)):
        raise PrimaryGRFSplitContractError("teacher observation is non-finite")
    return PairedViews(
        student=student,
        teacher=teacher,
        primary_load_bw=float(np.float32(primary_load)),
        primary_contact=primary_contact,
        detector_load_bw=float(np.float32(detector_load)),
        detector_contact=detector_contact,
    )


def array_sha256(array: Any) -> str:
    value = np.ascontiguousarray(np.asarray(array))
    digest = hashlib.sha256()
    digest.update(str(value.dtype).encode("ascii"))
    digest.update(json.dumps(list(value.shape), separators=(",", ":")).encode("ascii"))
    digest.update(value.tobytes(order="C"))
    return digest.hexdigest()


def assemble_grouped_dataset(
    groups: Sequence[Mapping[str, Any]],
    *,
    training_trials: Sequence[str],
    validation_trials: Sequence[str],
) -> tuple[dict[str, np.ndarray], np.ndarray, np.ndarray, dict[str, Any]]:
    """Create two records per physical state and a leakage-free group split."""

    train_set = {str(value) for value in training_trials}
    validation_set = {str(value) for value in validation_trials}
    if not train_set or not validation_set or train_set & validation_set:
        raise PrimaryGRFSplitContractError(
            "training and validation trials must be nonempty and disjoint"
        )
    observations: list[np.ndarray] = []
    actions: list[np.ndarray] = []
    trial_ids: list[str] = []
    view_roles: list[str] = []
    group_ids: list[str] = []
    seen_trials: set[str] = set()
    physical_state_count = 0
    for group in groups:
        trial = str(group.get("trial_id", ""))
        students = np.asarray(group.get("student_views"), dtype=np.float32)
        teachers = np.asarray(group.get("teacher_views"), dtype=np.float32)
        labels = np.asarray(group.get("teacher_means"), dtype=np.float32)
        if (
            students.ndim != 2
            or students.shape[1] != ACTOR_FEATURE_COUNT
            or teachers.shape != students.shape
            or labels.shape != (len(students), ACTION_DIM)
            or len(students) == 0
        ):
            raise PrimaryGRFSplitContractError(
                f"trial {trial!r} has malformed paired arrays"
            )
        if not all(
            np.all(np.isfinite(value)) for value in (students, teachers, labels)
        ):
            raise PrimaryGRFSplitContractError(
                f"trial {trial!r} contains non-finite values"
            )
        if np.any(np.abs(labels) > 1.0):
            raise PrimaryGRFSplitContractError(
                f"trial {trial!r} teacher means exceed action bounds"
            )
        seen_trials.add(trial)
        for index in range(len(students)):
            group_id = f"trial{trial}:state{index:03d}"
            for role, view in (
                (STUDENT_VIEW, students[index]),
                (TEACHER_VIEW, teachers[index]),
            ):
                observations.append(view.copy())
                actions.append(labels[index].copy())
                trial_ids.append(trial)
                view_roles.append(role)
                group_ids.append(group_id)
        physical_state_count += len(students)
    expected_trials = train_set | validation_set
    if seen_trials != expected_trials:
        raise PrimaryGRFSplitContractError(
            f"dataset trial groups {sorted(seen_trials)} != {sorted(expected_trials)}"
        )
    observations_array = np.ascontiguousarray(observations, dtype=np.float32)
    actions_array = np.ascontiguousarray(actions, dtype=np.float32)
    trial_array = np.asarray(trial_ids, dtype="U8")
    role_array = np.asarray(view_roles, dtype="U16")
    group_array = np.asarray(group_ids, dtype="U32")
    training_indices = np.flatnonzero(np.isin(trial_array, sorted(train_set))).astype(
        np.int64
    )
    validation_indices = np.flatnonzero(
        np.isin(trial_array, sorted(validation_set))
    ).astype(np.int64)
    if (
        len(training_indices) + len(validation_indices) != len(observations_array)
        or np.intersect1d(training_indices, validation_indices).size
    ):
        raise PrimaryGRFSplitContractError("group split is incomplete or overlapping")
    train_groups = set(group_array[training_indices].tolist())
    validation_groups = set(group_array[validation_indices].tolist())
    if train_groups & validation_groups:
        raise PrimaryGRFSplitContractError("physical-state group leakage detected")
    dataset = {
        "observations": observations_array,
        "actions": actions_array,
        "actor_feature_names": np.asarray(
            groups[0]["actor_feature_names"], dtype="U64"
        ),
        "trial_ids": trial_array,
        "view_roles": role_array,
        "group_ids": group_array,
    }
    summary = {
        "physical_states": physical_state_count,
        "records": len(observations_array),
        "training_records": len(training_indices),
        "validation_records": len(validation_indices),
        "training_trials": sorted(train_set),
        "validation_trials": sorted(validation_set),
        "training_group_count": len(train_groups),
        "validation_group_count": len(validation_groups),
        "observations_sha256": array_sha256(observations_array),
        "actions_sha256": array_sha256(actions_array),
        "training_indices_sha256": array_sha256(training_indices),
        "validation_indices_sha256": array_sha256(validation_indices),
    }
    return dataset, training_indices, validation_indices, summary


def assemble_train_only_dataset(
    groups: Sequence[Mapping[str, Any]],
    *,
    training_trials: Sequence[str],
) -> tuple[dict[str, np.ndarray], np.ndarray, np.ndarray, dict[str, Any]]:
    """Create a paired corpus whose every row is used by a fixed-epoch fit.

    At least two trial groups are required so the established grouped builder
    can validate schemas, finiteness, labels, names, and physical-state pairing
    without introducing a second implementation of those contracts.  Its
    temporary partition is discarded; the returned validation set is empty.
    """

    trials = tuple(dict.fromkeys(str(value) for value in training_trials))
    if len(trials) < 2:
        raise PrimaryGRFSplitContractError(
            "train-only assembly requires at least two distinct trials"
        )
    dataset, _temporary_train, _temporary_validation, base = (
        assemble_grouped_dataset(
            groups,
            training_trials=(trials[0],),
            validation_trials=trials[1:],
        )
    )
    training_indices = np.arange(len(dataset["observations"]), dtype=np.int64)
    validation_indices = np.empty(0, dtype=np.int64)
    group_ids = np.asarray(dataset["group_ids"]).astype(str)
    summary = {
        **base,
        "partition_mode": "fixed_final_epoch_train_only",
        "training_records": len(training_indices),
        "validation_records": 0,
        "training_trials": sorted(trials),
        "validation_trials": [],
        "training_group_count": len(set(group_ids.tolist())),
        "validation_group_count": 0,
        "training_indices_sha256": array_sha256(training_indices),
        "validation_indices_sha256": array_sha256(validation_indices),
    }
    return dataset, training_indices, validation_indices, summary


def prediction_metrics(prediction: Any, target: Any) -> dict[str, float | int]:
    actual = np.asarray(prediction, dtype=np.float32)
    expected = np.asarray(target, dtype=np.float32)
    if actual.shape != expected.shape or actual.ndim != 2:
        raise PrimaryGRFSplitContractError("prediction/target shape mismatch")
    if not np.all(np.isfinite(actual)) or not np.all(np.isfinite(expected)):
        raise PrimaryGRFSplitContractError("prediction metrics received non-finite data")
    error = actual.astype(np.float64) - expected.astype(np.float64)
    return {
        "samples": len(actual),
        "rmse": float(np.sqrt(np.mean(np.square(error)))),
        "max_abs_error": float(np.max(np.abs(error), initial=0.0)),
    }


def offline_adaptation_gate(
    *,
    source_predictions: Any,
    adapted_predictions: Any,
    targets: Any,
    validation_indices: Any,
    view_roles: Any,
    all_adapted_logits: Any,
    action_dim: int = ACTION_DIM,
) -> dict[str, Any]:
    """Apply the preregistered student/anchor validation thresholds."""

    source = np.asarray(source_predictions, dtype=np.float32)
    adapted = np.asarray(adapted_predictions, dtype=np.float32)
    expected = np.asarray(targets, dtype=np.float32)
    indices = np.asarray(validation_indices, dtype=np.int64)
    roles = np.asarray(view_roles).astype(str)
    logits = np.asarray(all_adapted_logits, dtype=np.float32)
    if source.shape != expected.shape or adapted.shape != expected.shape:
        raise PrimaryGRFSplitContractError("offline prediction shape mismatch")
    if logits.shape != (len(expected), action_dim * 2):
        raise PrimaryGRFSplitContractError("adapted logits shape mismatch")
    if len(roles) != len(expected) or np.any(indices < 0) or np.any(indices >= len(expected)):
        raise PrimaryGRFSplitContractError("offline validation metadata mismatch")
    student_indices = indices[roles[indices] == STUDENT_VIEW]
    teacher_indices = indices[roles[indices] == TEACHER_VIEW]
    if not len(student_indices) or not len(teacher_indices):
        raise PrimaryGRFSplitContractError("both validation views are required")
    source_student = prediction_metrics(source[student_indices], expected[student_indices])
    adapted_student = prediction_metrics(
        adapted[student_indices], expected[student_indices]
    )
    adapted_teacher = prediction_metrics(
        adapted[teacher_indices], expected[teacher_indices]
    )
    improvement = (
        1.0
        if float(source_student["rmse"]) == 0.0
        and float(adapted_student["rmse"]) == 0.0
        else (
            0.0
            if float(source_student["rmse"]) == 0.0
            else 1.0
            - float(adapted_student["rmse"]) / float(source_student["rmse"])
        )
    )
    checks = {
        "student_rmse": float(adapted_student["rmse"]) <= 0.01,
        "student_max_abs_error": float(adapted_student["max_abs_error"]) <= 0.10,
        "teacher_rmse": float(adapted_teacher["rmse"]) <= 0.005,
        "teacher_max_abs_error": float(adapted_teacher["max_abs_error"]) <= 0.05,
        "student_rmse_reduction": improvement >= 0.50,
        "finite_logits": bool(np.all(np.isfinite(logits))),
        "means_within_action_bounds": bool(
            np.all(np.abs(logits[:, :action_dim]) <= 1.0)
        ),
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "source_student": source_student,
        "adapted_student": adapted_student,
        "adapted_teacher": adapted_teacher,
        "student_rmse_reduction_fraction": float(improvement),
    }
