"""Read-only semantic analysis of the frozen R10 extended-fit outliers.

The terminal extended fit deliberately did not persist its model state.  This
diagnostic therefore consumes only the terminal top-20 rows frozen in the dry-
fit result; it never reruns or approximates the fit.  It closes those rows
against the immutable R9 corpus, replays the already persisted observer
boundaries without querying a policy, and compares V26/student and
legacy/teacher phase semantics around the dominant transition.

No environment is constructed or stepped and no candidate is published.
"""

from __future__ import annotations

import argparse
import collections
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np


def _discover_repo_root(source: Path) -> Path:
    for candidate in source.resolve().parents:
        if (
            (candidate / "AGENTS.md").is_file()
            and (candidate / "validation").is_dir()
            and (candidate / "Trajectory Generator").is_dir()
        ):
            return candidate
    raise RuntimeError("repository root could not be discovered")


REPO_ROOT = _discover_repo_root(Path(__file__))
BASELINE_ROOT = REPO_ROOT / "Trajectory Generator" / "baseline_MLP"
VALIDATION_ROOT = BASELINE_ROOT / "validation"
R9_RUN_ROOT = VALIDATION_ROOT / "v12r9" / "h0_v12r9_run_20260814"
DIAGNOSTIC_ROOT = Path(__file__).resolve().parent
for _path in (
    REPO_ROOT,
    REPO_ROOT / "validation",
    REPO_ROOT / "Trajectory Generator",
    BASELINE_ROOT,
    VALIDATION_ROOT,
    VALIDATION_ROOT / "v12r3",
):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import h0_primary_split_v10_coherent_teacher as coherent_teacher  # noqa: E402
import h0_primary_split_v12r3_autonomy_recovery_contract as replay_contract  # noqa: E402
import h0_primary_split_v12r3_pure_probe_observer_labeler as observer  # noqa: E402


CORPUS_PATH = R9_RUN_ROOT / "fit" / "corpus.npz"
DRY_FIT_RESULT_PATH = (
    DIAGNOSTIC_ROOT / "results" / "r9_extended_uniform_reset3_dry_fit.json"
)
DEFAULT_OUTPUT_PATH = DIAGNOSTIC_ROOT / "results" / "extended_outlier_analysis.json"
EXPECTED_CORPUS_SHA256 = (
    "1b35d0789d11a0f3bca3cae15c5877ceaf68845bf69ed7231d5a4ecc4d5b9dfe"
)
EXPECTED_DRY_FIT_SHA256 = (
    "1aa594a15b6d54a97bf374f1d78801ffa1fafa5050e2add33dd4637f7107c009"
)
EXPECTED_DRY_FIT_STATUS = "COMPLETE_H0_V12R10_EXTENDED_UNIFORM_RESET3_DRY_FIT"
STATUS = "COMPLETE_H0_V12R10_EXTENDED_OUTLIER_ANALYSIS"
ERROR_THRESHOLD = 0.06
TOP_LIMIT = 20
FOCUS_CENTER_STEP = 385
FOCUS_RADIUS = 10
NORMALIZATION_STD_FLOOR = np.float32(1.0e-4)
FOCUS_CASES = (
    "deterministic_offset_nominal",
    "stochastic_nominal_seed_127",
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _strict_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _float_list(values: Iterable[Any]) -> list[float]:
    return [float(value) for value in values]


def _load_inputs() -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    if _sha256(CORPUS_PATH) != EXPECTED_CORPUS_SHA256:
        raise RuntimeError("immutable R9 corpus digest drifted")
    if _sha256(DRY_FIT_RESULT_PATH) != EXPECTED_DRY_FIT_SHA256:
        raise RuntimeError("frozen extended dry-fit result digest drifted")
    result = _strict_json(DRY_FIT_RESULT_PATH)
    if (
        result.get("status") != EXPECTED_DRY_FIT_STATUS
        or result.get("passed") is not True
        or result.get("decision") != "REJECT_DRY_FIT_NO_RETRY"
        or result.get("terminal_state", {}).get("state_persisted") is not False
        or result.get("terminal_state", {}).get("candidate_published") is not False
    ):
        raise RuntimeError("dry-fit terminal/no-publication contract drifted")
    with np.load(CORPUS_PATH, allow_pickle=False) as loaded:
        arrays = {
            name: np.ascontiguousarray(loaded[name].copy()) for name in loaded.files
        }
    if (
        arrays["observations"].shape != (11875, 35)
        or arrays["observations"].dtype != np.dtype(np.float32)
        or arrays["actions"].shape != (11875, 2)
        or arrays["actions"].dtype != np.dtype(np.float32)
        or int(np.count_nonzero(arrays["reset_mask"])) != 26
        or len(result.get("terminal_top_error_rows", ())) != TOP_LIMIT
    ):
        raise RuntimeError("corpus or frozen top-20 shape drifted")
    return arrays, result


def _reset3_weights(arrays: Mapping[str, np.ndarray]) -> np.ndarray:
    weights = np.ascontiguousarray(
        arrays["normalized_sample_weights"], dtype=np.float64
    ).copy()
    weights[arrays["reset_mask"]] *= 3.0
    strata = arrays["stratum_ids"].astype(str)
    for stratum_id in sorted(set(strata)):
        selected = np.flatnonzero(strata == stratum_id)
        weights[selected] *= 500.0 / math.fsum(weights[selected].astype(float))
        if not math.isclose(
            math.fsum(weights[selected].astype(float)),
            500.0,
            rel_tol=0.0,
            abs_tol=1.0e-9,
        ):
            raise RuntimeError(f"reset3 mass drifted for {stratum_id}")
    if not math.isclose(
        math.fsum(weights.astype(float)), 6500.0, rel_tol=0.0, abs_tol=1.0e-8
    ):
        raise RuntimeError("reset3 total mass drifted")
    return np.ascontiguousarray(weights)


def _normalized_observations(
    arrays: Mapping[str, np.ndarray],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    base = np.flatnonzero(arrays["tranche_ids"].astype(str) == "v8r1p1_base")
    if len(base) != 3000:
        raise RuntimeError("frozen base-normalization rows drifted")
    source = arrays["observations"][base]
    mean = np.ascontiguousarray(
        source.mean(axis=0, dtype=np.float64).astype(np.float32)
    )
    std = np.ascontiguousarray(
        np.maximum(
            source.std(axis=0, dtype=np.float64).astype(np.float32),
            NORMALIZATION_STD_FLOOR,
        ),
        dtype=np.float32,
    )
    normalized = np.ascontiguousarray(
        (arrays["observations"] - mean) / std, dtype=np.float32
    )
    if (
        not np.all(np.isfinite(normalized))
        or normalized[:, :2].tobytes()
        != np.zeros((len(normalized), 2), dtype=np.float32).tobytes()
    ):
        raise RuntimeError("frozen normalization closure failed")
    return normalized, mean, std


def _phase_state(observation: np.ndarray) -> str:
    if float(observation[17]) > 0.5:
        return "WAIT_HS"
    if float(observation[18]) > 0.5:
        return "STANCE_AFTER_HS"
    if float(observation[19]) > 0.5:
        return "SWING_AFTER_TO"
    return "OTHER_OR_TIMEOUT"


def _episode_indices(arrays: Mapping[str, np.ndarray], episode_id: str) -> np.ndarray:
    selected = np.flatnonzero(arrays["episode_ids"].astype(str) == episode_id)
    order = np.argsort(arrays["step_indices"][selected], kind="stable")
    result = selected[order]
    if len(set(arrays["step_indices"][result].astype(int))) != len(result):
        raise RuntimeError(f"duplicate step in episode {episode_id}")
    return result


def _corpus_event_steps(
    arrays: Mapping[str, np.ndarray], episode_id: str
) -> list[dict[str, Any]]:
    indices = _episode_indices(arrays, episode_id)
    output: list[dict[str, Any]] = []
    previous_state: str | None = None
    for index in indices:
        observation = arrays["observations"][index]
        state = _phase_state(observation)
        state_changed = previous_state is not None and state != previous_state
        heel_strike = float(observation[12]) > 0.5
        toe_off = float(observation[13]) > 0.5
        if state_changed or heel_strike or toe_off:
            output.append(
                {
                    "row_index": int(index),
                    "step_index": int(arrays["step_indices"][index]),
                    "state": state,
                    "state_changed": bool(state_changed),
                    "heel_strike_pulse": bool(heel_strike),
                    "toe_off_pulse": bool(toe_off),
                }
            )
        previous_state = state
    return output


def _nearest_step(
    step: int, events: Sequence[Mapping[str, Any]], *, key: str = "step_index"
) -> dict[str, Any]:
    if not events:
        return {"previous": None, "next": None, "absolute_nearest": None}
    previous = [event for event in events if int(event[key]) <= step]
    following = [event for event in events if int(event[key]) >= step]
    prior = max(previous, key=lambda event: int(event[key])) if previous else None
    after = min(following, key=lambda event: int(event[key])) if following else None
    nearest = min(events, key=lambda event: abs(int(event[key]) - step))

    def _record(event: Mapping[str, Any] | None) -> dict[str, Any] | None:
        if event is None:
            return None
        return {
            "step": int(event[key]),
            "signed_distance_steps": int(event[key]) - int(step),
            "event": event.get("event"),
            "state": event.get("state") or event.get("state_name"),
        }

    return {
        "previous": _record(prior),
        "next": _record(after),
        "absolute_nearest": _record(nearest),
    }


def _close_top_rows(
    arrays: Mapping[str, np.ndarray],
    dry_fit: Mapping[str, Any],
    objective_weights: np.ndarray,
) -> list[dict[str, Any]]:
    output: list[dict[str, Any]] = []
    errors = []
    for rank, frozen in enumerate(dry_fit["terminal_top_error_rows"], start=1):
        row = int(frozen["row_index"])
        action = int(frozen["action_dimension"])
        expected = {
            "case_id": str(arrays["case_ids"][row]),
            "step_index": int(arrays["step_indices"][row]),
            "stratum_id": str(arrays["stratum_ids"][row]),
            "tranche_id": str(arrays["tranche_ids"][row]),
            "reset": bool(arrays["reset_mask"][row]),
        }
        if any(frozen.get(name) != value for name, value in expected.items()):
            raise RuntimeError(f"frozen top row {row} metadata drifted")
        if np.float32(frozen["target"]).tobytes() != arrays["actions"][
            row, action
        ].tobytes() or not math.isclose(
            float(frozen["source_equal_stratum_weight"]),
            float(objective_weights[row]),
            rel_tol=0.0,
            abs_tol=1.0e-12,
        ):
            raise RuntimeError(f"frozen top row {row} target/weight drifted")
        error = float(frozen["absolute_error"])
        errors.append(error)
        output.append(
            {
                **dict(frozen),
                "rank": rank,
                "episode_id": str(arrays["episode_ids"][row]),
                "origin": str(arrays["origins"][row]),
                "objective_weight": float(objective_weights[row]),
                "full_target": _float_list(arrays["actions"][row]),
                "student_phase_state": _phase_state(arrays["observations"][row]),
            }
        )
    if (
        errors != sorted(errors, reverse=True)
        or min(errors) <= ERROR_THRESHOLD
        or len(output) != TOP_LIMIT
    ):
        raise RuntimeError("frozen top-20 error ordering/threshold drifted")
    return output


def _neighbor_record(
    row: int,
    selected: np.ndarray,
    distances: np.ndarray,
    arrays: Mapping[str, np.ndarray],
) -> dict[str, Any] | None:
    candidates = selected[selected != row]
    if not len(candidates):
        return None
    neighbor = int(candidates[np.argmin(distances[candidates])])
    delta = arrays["actions"][neighbor].astype(np.float64) - arrays["actions"][
        row
    ].astype(np.float64)
    return {
        "row_index": neighbor,
        "normalized_effective_l2": float(distances[neighbor]),
        "case_id": str(arrays["case_ids"][neighbor]),
        "episode_id": str(arrays["episode_ids"][neighbor]),
        "stratum_id": str(arrays["stratum_ids"][neighbor]),
        "tranche_id": str(arrays["tranche_ids"][neighbor]),
        "step_index": int(arrays["step_indices"][neighbor]),
        "target": _float_list(arrays["actions"][neighbor]),
        "target_delta_neighbor_minus_outlier": _float_list(delta),
        "target_linf_delta": float(np.max(np.abs(delta))),
        "target_l2_delta": float(np.linalg.norm(delta)),
    }


def _nearest_corpus_audit(
    rows: Sequence[Mapping[str, Any]],
    arrays: Mapping[str, np.ndarray],
    normalized: np.ndarray,
) -> dict[str, Any]:
    effective = np.ascontiguousarray(normalized[:, 2:], dtype=np.float64)
    all_indices = np.arange(len(effective), dtype=np.int64)
    cases = arrays["case_ids"].astype(str)
    episodes = arrays["episode_ids"].astype(str)
    output: list[dict[str, Any]] = []
    for record in rows:
        row = int(record["row_index"])
        distances = np.linalg.norm(effective - effective[row], axis=1)
        distances[row] = np.inf
        output.append(
            {
                "row_index": row,
                "nearest_any": _neighbor_record(row, all_indices, distances, arrays),
                "nearest_same_episode": _neighbor_record(
                    row,
                    np.flatnonzero(episodes == episodes[row]),
                    distances,
                    arrays,
                ),
                "nearest_other_episode": _neighbor_record(
                    row,
                    np.flatnonzero(episodes != episodes[row]),
                    distances,
                    arrays,
                ),
                "nearest_other_case": _neighbor_record(
                    row,
                    np.flatnonzero(cases != cases[row]),
                    distances,
                    arrays,
                ),
                "local_density": {
                    str(radius): int(np.count_nonzero(distances <= radius))
                    for radius in (0.1, 0.25, 0.5, 1.0)
                },
            }
        )

    def _collision(columns: Sequence[int]) -> dict[str, int]:
        groups: dict[bytes, list[int]] = collections.defaultdict(list)
        projected = np.ascontiguousarray(arrays["observations"][:, columns])
        for index, observation in enumerate(projected):
            groups[observation.tobytes()].append(index)
        duplicate = [indices for indices in groups.values() if len(indices) > 1]
        conflicting = [
            indices
            for indices in duplicate
            if not np.all(arrays["actions"][indices] == arrays["actions"][indices[0]])
        ]
        return {
            "unique_inputs": len(groups),
            "duplicate_groups": len(duplicate),
            "conflicting_duplicate_groups": len(conflicting),
            "conflicting_rows": sum(len(indices) for indices in conflicting),
        }

    return {
        "distance_contract": (
            "frozen-v8r1p1-base z-score; disabled clock columns 0:2 excluded"
        ),
        "exact_collision_full35": _collision(tuple(range(35))),
        "exact_collision_effective33": _collision(tuple(range(2, 35))),
        "top_rows": output,
    }


def _transition_record(step: int, payload: Mapping[str, Any]) -> dict[str, Any]:
    transitions = payload.get("accepted_transitions_this_step", ())
    if not transitions:
        raise RuntimeError("teacher transition record requested without transition")
    transition = transitions[-1]
    return {
        "step_index": int(step),
        "event": str(transition.get("event")),
        "state_name": str(payload.get("state_name")),
        "from_state_id": float(transition.get("from_state_id")),
        "to_state_id": float(transition.get("to_state_id")),
        "event_time_s": float(transition.get("event_time_s")),
        "cycle_reject_reason": str(transition.get("cycle_reject_reason", "")),
    }


def _observer_bundle(
    case_id: str,
    arrays: Mapping[str, np.ndarray],
) -> dict[str, Any]:
    case_root = R9_RUN_ROOT / "observer_collection" / case_id
    replay_path = case_root / "replay_boundaries.npz"
    trace_path = case_root / "trace.json"
    labels_path = case_root / "observer_labels" / "labels.npz"
    replay = observer.load_probe_replay_strict(
        replay_path, contract_module=replay_contract
    )
    official = observer.replay_teacher_views(replay)
    phase_fsm = observer._default_phase_fsm_factory(  # noqa: SLF001
        replay.config, replay.fsm_module, replay.fsm_class
    )
    shadow = coherent_teacher.LegacyGaitShadow(phase_fsm)
    teachers: list[np.ndarray] = []
    payloads: list[dict[str, Any]] = []
    body_weight = float(replay.arrays["body_weight_n"][0])
    for boundary, student in enumerate(replay.arrays["actor_observations"]):
        teacher = coherent_teacher.build_teacher_view(
            student,
            coherent_teacher.EXPECTED_ACTOR_FEATURE_NAMES,
            observer._boundary_info(replay, boundary),  # noqa: SLF001
            body_weight_n=body_weight,
            shadow=shadow,
            reset_boundary=boundary == 0,
        )
        teachers.append(teacher)
        payloads.append(shadow.phase_payload())
    teacher_array = np.ascontiguousarray(teachers, dtype=np.float32)
    if teacher_array.tobytes() != official.teacher_observations.tobytes():
        raise RuntimeError(f"manual teacher replay drifted for {case_id}")

    trace = _strict_json(trace_path)
    if not isinstance(trace, list) or len(trace) != replay.n_steps:
        raise RuntimeError(f"trace shape drifted for {case_id}")
    for index, row in enumerate(trace):
        observed = np.ascontiguousarray(row["v26_observation"], dtype=np.float32)
        if (
            row.get("step") != index + 1
            or observed.tobytes()
            != replay.arrays["actor_observations"][index].tobytes()
        ):
            raise RuntimeError(f"trace/replay closure failed for {case_id}")

    corpus_rows = np.flatnonzero(
        arrays["stratum_ids"].astype(str) == f"observer::{case_id}"
    )
    corpus_rows = corpus_rows[
        np.argsort(arrays["step_indices"][corpus_rows], kind="stable")
    ]
    with np.load(labels_path, allow_pickle=False) as loaded:
        label_observations = np.ascontiguousarray(loaded["observations"])
        label_actions = np.ascontiguousarray(loaded["actions"])
    if (
        len(corpus_rows) != replay.n_steps
        or not np.array_equal(
            arrays["step_indices"][corpus_rows],
            np.arange(1, replay.n_steps + 1, dtype=np.int64),
        )
        or arrays["observations"][corpus_rows].tobytes()
        != replay.arrays["actor_observations"].tobytes()
        or label_observations.tobytes() != replay.arrays["actor_observations"].tobytes()
        or arrays["actions"][corpus_rows].tobytes() != label_actions.tobytes()
    ):
        raise RuntimeError(f"corpus/replay/labels closure failed for {case_id}")

    v26_transitions: list[dict[str, Any]] = []
    for row in trace:
        phase = row["phase_fsm"]
        for transition in phase["accepted_transitions_this_step"]:
            v26_transitions.append(
                {
                    "step_index": int(row["step"]),
                    "event": str(transition.get("event")),
                    "state_name": str(phase["state_name"]),
                    "from_state_id": float(transition.get("from_state_id")),
                    "to_state_id": float(transition.get("to_state_id")),
                    "event_time_s": float(transition.get("event_time_s")),
                    "cycle_reject_reason": str(
                        transition.get("cycle_reject_reason", "")
                    ),
                }
            )
    teacher_transitions = [
        _transition_record(step, payload)
        for step, payload in enumerate(payloads, start=1)
        if payload.get("accepted_transitions_this_step")
    ]
    return {
        "case_id": case_id,
        "replay": replay,
        "students": official.student_observations,
        "teachers": official.teacher_observations,
        "payloads": payloads,
        "trace": trace,
        "corpus_rows": corpus_rows,
        "v26_transitions": v26_transitions,
        "teacher_transitions": teacher_transitions,
        "closure": {
            "rows": replay.n_steps,
            "replayed_boundaries": official.replayed_boundary_count,
            "changed_only_mutable_10_24_count": official.changed_only_mutable_count,
            "invariant_columns_byte_exact_count": (
                official.invariant_columns_byte_exact_count
            ),
            "column_24_changed_count": official.column_24_changed_count,
            "corpus_replay_labels_trace_byte_exact": True,
            "policy_queries": 0,
            "environment_reset_calls": 0,
            "environment_step_calls": 0,
        },
    }


def _focus_window(
    bundle: Mapping[str, Any],
    arrays: Mapping[str, np.ndarray],
    normalized: np.ndarray,
    std: np.ndarray,
) -> dict[str, Any]:
    case_id = str(bundle["case_id"])
    start = FOCUS_CENTER_STEP - FOCUS_RADIUS
    stop = FOCUS_CENTER_STEP + FOCUS_RADIUS
    corpus_rows = bundle["corpus_rows"]
    students = bundle["students"]
    teachers = bundle["teachers"]
    payloads = bundle["payloads"]
    trace = bundle["trace"]
    rows: list[dict[str, Any]] = []
    for step in range(start, stop + 1):
        offset = step - 1
        row = int(corpus_rows[offset])
        student = students[offset]
        teacher = teachers[offset]
        phase = trace[offset]["phase_fsm"]
        payload = payloads[offset]
        prior_row = int(corpus_rows[offset - 1]) if offset else None
        target_delta = (
            arrays["actions"][row].astype(np.float64)
            - arrays["actions"][prior_row].astype(np.float64)
            if prior_row is not None
            else np.zeros(2, dtype=np.float64)
        )
        step_l2 = (
            float(
                np.linalg.norm(
                    normalized[row, 2:].astype(np.float64)
                    - normalized[prior_row, 2:].astype(np.float64)
                )
            )
            if prior_row is not None
            else 0.0
        )
        changed = [
            index
            for index in range(10, 25)
            if student[index].tobytes() != teacher[index].tobytes()
        ]
        rows.append(
            {
                "step_index": step,
                "row_index": row,
                "time_s": float(trace[offset]["time_s"]),
                "target": _float_list(arrays["actions"][row]),
                "target_delta_from_previous": _float_list(target_delta),
                "target_linf_jump_from_previous": float(np.max(np.abs(target_delta))),
                "normalized_student_l2_from_previous": step_l2,
                "v26_state": str(phase["state_name"]),
                "v26_expected_next_event": str(phase["expected_next_event"]),
                "v26_accepted_events": [
                    str(item.get("event"))
                    for item in phase["accepted_transitions_this_step"]
                ],
                "v26_sensor_events": [
                    str(item.get("event")) for item in phase["sensor_events_this_step"]
                ],
                "legacy_events_delivered": [
                    f"{item.get('side')}::{item.get('event')}"
                    for item in trace[offset]["legacy_online_events"]
                ],
                "teacher_state": str(payload["state_name"]),
                "teacher_expected_next_event": str(payload["expected_next_event"]),
                "teacher_timeout_exceeded": float(payload["timeout_exceeded"]),
                "teacher_accepted_events": [
                    str(item.get("event"))
                    for item in payload["accepted_transitions_this_step"]
                ],
                "student_10_24": _float_list(student[10:25]),
                "teacher_10_24": _float_list(teacher[10:25]),
                "teacher_changed_columns": changed,
                "normalized_student_teacher_block_l2": float(
                    np.linalg.norm(
                        (
                            teacher[10:25].astype(np.float64)
                            - student[10:25].astype(np.float64)
                        )
                        / std[10:25].astype(np.float64)
                    )
                ),
            }
        )
    target_jumps = [row["target_linf_jump_from_previous"] for row in rows]
    max_jump_index = int(np.argmax(target_jumps))
    return {
        "case_id": case_id,
        "step_range_inclusive": [start, stop],
        "feature_columns": list(range(10, 25)),
        "feature_names": arrays["actor_feature_names"][10:25].astype(str).tolist(),
        "v26_transitions_full_episode": bundle["v26_transitions"],
        "teacher_transitions_full_episode": bundle["teacher_transitions"],
        "maximum_target_jump_in_window": {
            "arrives_at_step": int(rows[max_jump_index]["step_index"]),
            "linf": float(target_jumps[max_jump_index]),
            "delta": rows[max_jump_index]["target_delta_from_previous"],
        },
        "rows": rows,
    }


def _rank_feature_delta(
    first: np.ndarray,
    second: np.ndarray,
    std: np.ndarray,
    names: np.ndarray,
    *,
    columns: Sequence[int],
) -> list[dict[str, Any]]:
    output = []
    for column in columns:
        raw = float(second[column]) - float(first[column])
        normalized = raw / float(std[column])
        output.append(
            {
                "column": int(column),
                "feature": str(names[column]),
                "raw_second_minus_first": raw,
                "normalized_second_minus_first": normalized,
                "absolute_normalized_delta": abs(normalized),
            }
        )
    return sorted(
        output,
        key=lambda record: (
            -float(record["absolute_normalized_delta"]),
            int(record["column"]),
        ),
    )


def _aligned_focus_comparison(
    nominal: Mapping[str, Any],
    seed127: Mapping[str, Any],
    arrays: Mapping[str, np.ndarray],
    std: np.ndarray,
) -> dict[str, Any]:
    start = FOCUS_CENTER_STEP - FOCUS_RADIUS
    stop = FOCUS_CENTER_STEP + FOCUS_RADIUS
    names = arrays["actor_feature_names"].astype(str)
    output: list[dict[str, Any]] = []
    for step in range(start, stop + 1):
        offset = step - 1
        nominal_student = nominal["students"][offset]
        seed_student = seed127["students"][offset]
        nominal_teacher = nominal["teachers"][offset]
        seed_teacher = seed127["teachers"][offset]
        nominal_row = int(nominal["corpus_rows"][offset])
        seed_row = int(seed127["corpus_rows"][offset])
        target_delta = arrays["actions"][seed_row].astype(np.float64) - arrays[
            "actions"
        ][nominal_row].astype(np.float64)
        output.append(
            {
                "step_index": step,
                "nominal_v26_state": str(
                    nominal["trace"][offset]["phase_fsm"]["state_name"]
                ),
                "seed127_v26_state": str(
                    seed127["trace"][offset]["phase_fsm"]["state_name"]
                ),
                "nominal_teacher_state": str(nominal["payloads"][offset]["state_name"]),
                "seed127_teacher_state": str(seed127["payloads"][offset]["state_name"]),
                "normalized_student_effective_l2": float(
                    np.linalg.norm(
                        (
                            seed_student[2:].astype(np.float64)
                            - nominal_student[2:].astype(np.float64)
                        )
                        / std[2:].astype(np.float64)
                    )
                ),
                "normalized_teacher_effective_l2": float(
                    np.linalg.norm(
                        (
                            seed_teacher[2:].astype(np.float64)
                            - nominal_teacher[2:].astype(np.float64)
                        )
                        / std[2:].astype(np.float64)
                    )
                ),
                "target_seed127_minus_nominal": _float_list(target_delta),
                "target_linf_delta": float(np.max(np.abs(target_delta))),
                "student_10_24_seed127_minus_nominal": _float_list(
                    seed_student[10:25].astype(np.float64)
                    - nominal_student[10:25].astype(np.float64)
                ),
                "teacher_10_24_seed127_minus_nominal": _float_list(
                    seed_teacher[10:25].astype(np.float64)
                    - nominal_teacher[10:25].astype(np.float64)
                ),
            }
        )
    center = FOCUS_CENTER_STEP - start
    nominal_center = nominal["students"][FOCUS_CENTER_STEP - 1]
    seed_center = seed127["students"][FOCUS_CENTER_STEP - 1]
    nominal_teacher = nominal["teachers"][FOCUS_CENTER_STEP - 1]
    seed_teacher = seed127["teachers"][FOCUS_CENTER_STEP - 1]
    max_target = max(output, key=lambda record: float(record["target_linf_delta"]))
    return {
        "step_range_inclusive": [start, stop],
        "feature_columns": list(range(10, 25)),
        "feature_names": names[10:25].tolist(),
        "rows": output,
        "step_385": output[center],
        "step_385_student_block_feature_delta_ranking": _rank_feature_delta(
            nominal_center,
            seed_center,
            std,
            names,
            columns=tuple(range(10, 25)),
        ),
        "step_385_teacher_block_feature_delta_ranking": _rank_feature_delta(
            nominal_teacher,
            seed_teacher,
            std,
            names,
            columns=tuple(range(10, 25)),
        ),
        "maximum_aligned_target_divergence": max_target,
        "all_21_v26_states_are_stance_in_both_cases": all(
            row["nominal_v26_state"] == "STANCE_AFTER_HS"
            and row["seed127_v26_state"] == "STANCE_AFTER_HS"
            for row in output
        ),
    }


def _cluster_rows(
    rows: Sequence[Mapping[str, Any]],
    arrays: Mapping[str, np.ndarray],
    normalized: np.ndarray,
) -> list[dict[str, Any]]:
    grouped: dict[tuple[str, str, str, str], list[Mapping[str, Any]]] = (
        collections.defaultdict(list)
    )
    for row in rows:
        grouped[
            (
                str(row["stratum_id"]),
                str(row["case_id"]),
                str(row["episode_id"]),
                str(row["tranche_id"]),
            )
        ].append(row)
    clusters: list[dict[str, Any]] = []
    for key in sorted(grouped):
        ordered = sorted(grouped[key], key=lambda row: int(row["step_index"]))
        parts: list[list[Mapping[str, Any]]] = []
        for row in ordered:
            if (
                not parts
                or int(row["step_index"]) > int(parts[-1][-1]["step_index"]) + 1
            ):
                parts.append([row])
            else:
                parts[-1].append(row)
        for part in parts:
            episode_id = key[2]
            episode_rows = _episode_indices(arrays, episode_id)
            step_to_row = {
                int(arrays["step_indices"][row]): int(row) for row in episode_rows
            }
            start = int(part[0]["step_index"])
            stop = int(part[-1]["step_index"])
            window_steps = [
                step
                for step in sorted(step_to_row)
                if start - FOCUS_RADIUS <= step <= stop + FOCUS_RADIUS
            ]
            jumps: list[dict[str, Any]] = []
            for step in window_steps:
                if step - 1 not in step_to_row:
                    continue
                current = step_to_row[step]
                previous = step_to_row[step - 1]
                target_delta = arrays["actions"][current].astype(np.float64) - arrays[
                    "actions"
                ][previous].astype(np.float64)
                jumps.append(
                    {
                        "arrives_at_step": step,
                        "target_linf": float(np.max(np.abs(target_delta))),
                        "target_delta": _float_list(target_delta),
                        "normalized_student_l2": float(
                            np.linalg.norm(
                                normalized[current, 2:].astype(np.float64)
                                - normalized[previous, 2:].astype(np.float64)
                            )
                        ),
                    }
                )
            event_steps = _corpus_event_steps(arrays, episode_id)
            clusters.append(
                {
                    "stratum_id": key[0],
                    "case_id": key[1],
                    "episode_id": episode_id,
                    "tranche_id": key[3],
                    "step_range_inclusive": [start, stop],
                    "row_count": len(part),
                    "rows": [int(row["row_index"]) for row in part],
                    "ranks": [int(row["rank"]) for row in part],
                    "action_dimensions": sorted(
                        {int(row["action_dimension"]) for row in part}
                    ),
                    "max_abs_error": max(float(row["absolute_error"]) for row in part),
                    "objective_weight_min": min(
                        float(row["objective_weight"]) for row in part
                    ),
                    "objective_weight_max": max(
                        float(row["objective_weight"]) for row in part
                    ),
                    "student_phase_states": sorted(
                        {str(row["student_phase_state"]) for row in part}
                    ),
                    "nearest_corpus_feature_event": _nearest_step(start, event_steps),
                    "corpus_feature_events_in_plus_minus_10_window": [
                        event
                        for event in event_steps
                        if start - FOCUS_RADIUS
                        <= int(event["step_index"])
                        <= stop + FOCUS_RADIUS
                    ],
                    "largest_target_jump_in_plus_minus_10_window": (
                        max(jumps, key=lambda item: float(item["target_linf"]))
                        if jumps
                        else None
                    ),
                    "largest_student_step_in_plus_minus_10_window": (
                        max(
                            jumps,
                            key=lambda item: float(item["normalized_student_l2"]),
                        )
                        if jumps
                        else None
                    ),
                }
            )
    return sorted(
        clusters,
        key=lambda cluster: (
            min(cluster["ranks"]),
            cluster["stratum_id"],
            cluster["step_range_inclusive"][0],
        ),
    )


def _observer_top_event_association(
    rows: Sequence[Mapping[str, Any]],
    bundles: Mapping[str, Mapping[str, Any]],
) -> list[dict[str, Any]]:
    output = []
    for row in rows:
        case_id = str(row["case_id"])
        if case_id not in bundles:
            continue
        bundle = bundles[case_id]
        step = int(row["step_index"])
        payload = bundle["payloads"][step - 1]
        trace_phase = bundle["trace"][step - 1]["phase_fsm"]
        output.append(
            {
                "rank": int(row["rank"]),
                "row_index": int(row["row_index"]),
                "case_id": case_id,
                "step_index": step,
                "action_dimension": int(row["action_dimension"]),
                "absolute_error": float(row["absolute_error"]),
                "v26_state": str(trace_phase["state_name"]),
                "teacher_state": str(payload["state_name"]),
                "nearest_v26_transition": _nearest_step(
                    step, bundle["v26_transitions"]
                ),
                "nearest_teacher_transition": _nearest_step(
                    step, bundle["teacher_transitions"]
                ),
            }
        )
    return output


def analyze() -> dict[str, Any]:
    arrays, dry_fit = _load_inputs()
    objective_weights = _reset3_weights(arrays)
    normalized, mean, std = _normalized_observations(arrays)
    top_rows = _close_top_rows(arrays, dry_fit, objective_weights)
    bundles = {case_id: _observer_bundle(case_id, arrays) for case_id in FOCUS_CASES}
    # Seed 128 is needed only to classify its one frozen top row against the
    # nearest persisted V26/legacy transitions; no extra fit prediction occurs.
    bundles["stochastic_nominal_seed_128"] = _observer_bundle(
        "stochastic_nominal_seed_128", arrays
    )
    focus_windows = {
        case_id: _focus_window(bundles[case_id], arrays, normalized, std)
        for case_id in FOCUS_CASES
    }
    aligned = _aligned_focus_comparison(
        bundles[FOCUS_CASES[0]], bundles[FOCUS_CASES[1]], arrays, std
    )
    clusters = _cluster_rows(top_rows, arrays, normalized)
    nearest = _nearest_corpus_audit(top_rows, arrays, normalized)
    associations = _observer_top_event_association(top_rows, bundles)

    top_by_stratum = collections.Counter(str(row["stratum_id"]) for row in top_rows)
    top_by_action = collections.Counter(
        int(row["action_dimension"]) for row in top_rows
    )
    base_minus = [
        row
        for row in top_rows
        if row["stratum_id"] == "base::deterministic_offset_minus_0p20"
    ]
    timeout_cases = set(FOCUS_CASES)
    timeout_rows = [row for row in top_rows if row["case_id"] in timeout_cases]
    observer_weight = float(top_rows[0]["objective_weight"])
    base_minus_weight = float(base_minus[0]["objective_weight"])
    initial = dry_fit["initial_metrics"]["global_metrics"]
    terminal = dry_fit["terminal_metrics"]["global_metrics"]
    stratum_errors = dry_fit["terminal_stratum_errors"]
    timeout_steps = {
        case_id: [
            int(item["step_index"])
            for item in bundles[case_id]["teacher_transitions"]
            if item["event"] == "timeout"
        ]
        for case_id in FOCUS_CASES
    }
    if timeout_steps != {
        "deterministic_offset_nominal": [384],
        "stochastic_nominal_seed_127": [386],
    }:
        raise RuntimeError("focus teacher-timeout evidence drifted")
    if not aligned["all_21_v26_states_are_stance_in_both_cases"]:
        raise RuntimeError("focus V26 stance evidence drifted")
    if (
        len(base_minus) != 13
        or len(timeout_rows) != 6
        or top_by_action != {0: 17, 1: 3}
        or nearest["exact_collision_effective33"]["conflicting_duplicate_groups"] != 0
    ):
        raise RuntimeError("terminal outlier classification drifted")

    return {
        "status": STATUS,
        "passed": True,
        "scope": "READ_ONLY_FROZEN_TOP20_CORPUS_REPLAY_TRACE_NO_FIT_NO_ENVIRONMENT",
        "working_directory": str(REPO_ROOT),
        "frozen_inputs": {
            "corpus": {
                "path": str(CORPUS_PATH.relative_to(REPO_ROOT)),
                "sha256": EXPECTED_CORPUS_SHA256,
                "rows": len(arrays["observations"]),
            },
            "extended_dry_fit": {
                "path": str(DRY_FIT_RESULT_PATH.relative_to(REPO_ROOT)),
                "sha256": EXPECTED_DRY_FIT_SHA256,
                "decision": dry_fit["decision"],
                "terminal_state_persisted": False,
            },
        },
        "inventory_limit": {
            "requested_error_threshold": ERROR_THRESHOLD,
            "frozen_terminal_rows_available": TOP_LIMIT,
            "all_frozen_top20_are_above_threshold": True,
            "all_rows_above_threshold_enumerable": False,
            "unknown_additional_rows_above_threshold": True,
            "reason": (
                "the terminal extended-fit state was intentionally not persisted; "
                "rerunning the fit is outside this read-only diagnostic, so only "
                "the frozen terminal top-20 can be analyzed"
            ),
        },
        "feature_contract": {
            "all_actor_feature_names": arrays["actor_feature_names"]
            .astype(str)
            .tolist(),
            "analyzed_phase_columns": list(range(10, 25)),
            "analyzed_phase_feature_names": (
                arrays["actor_feature_names"][10:25].astype(str).tolist()
            ),
            "normalization_mean_sha256": hashlib.sha256(mean.tobytes()).hexdigest(),
            "normalization_std_sha256": hashlib.sha256(std.tobytes()).hexdigest(),
        },
        "frozen_top20": top_rows,
        "top20_counts": {
            "by_stratum": dict(sorted(top_by_stratum.items())),
            "by_action_dimension": {
                str(key): value for key, value in sorted(top_by_action.items())
            },
            "base_minus_rows": len(base_minus),
            "focus_teacher_timeout_rows": len(timeout_rows),
        },
        "contiguous_clusters": clusters,
        "nearest_corpus": nearest,
        "observer_closures": {
            case_id: bundles[case_id]["closure"] for case_id in bundles
        },
        "focus_windows_plus_minus_10": focus_windows,
        "nominal_vs_seed127_aligned_comparison": aligned,
        "observer_top_event_association": associations,
        "causal_assessment": {
            "primary_observer_cause": {
                "classification": "TEACHER_STUDENT_TRANSITION_ALIAS",
                "confidence": "HIGH",
                "evidence": {
                    "nominal_teacher_legacy_timeout_step": 384,
                    "seed127_teacher_legacy_timeout_step": 386,
                    "focus_top_rows": len(timeout_rows),
                    "focus_top_steps": sorted(
                        {
                            f"{row['case_id']}::{row['step_index']}"
                            for row in timeout_rows
                        }
                    ),
                    "v26_state_through_both_375_395_windows": ("STANCE_AFTER_HS"),
                    "v26_transition_count_inside_both_windows": 0,
                    "nominal_max_target_jump": focus_windows[
                        "deterministic_offset_nominal"
                    ]["maximum_target_jump_in_window"],
                    "seed127_max_target_jump": focus_windows[
                        "stochastic_nominal_seed_127"
                    ]["maximum_target_jump_in_window"],
                    "step385_aligned_comparison": aligned["step_385"],
                },
                "interpretation": (
                    "the labels are queried on a reconstructed legacy teacher "
                    "view whose swing timeout is absent from the persisted V26 "
                    "student phase state; a smooth V26 neighborhood therefore "
                    "contains a sharp, case-shifted target transition"
                ),
            },
            "primary_base_minus_cause": {
                "classification": "LOW_PER_ROW_OBJECTIVE_WEIGHT",
                "confidence": "HIGH",
                "evidence": {
                    "top20_rows": len(base_minus),
                    "base_minus_rows_in_corpus": int(
                        stratum_errors["base::deterministic_offset_minus_0p20"]["rows"]
                    ),
                    "base_minus_top_row_weight": base_minus_weight,
                    "observer_focus_top_row_weight": observer_weight,
                    "observer_to_base_minus_weight_ratio": (
                        observer_weight / base_minus_weight
                    ),
                    "base_minus_unweighted_sse_share": stratum_errors[
                        "base::deterministic_offset_minus_0p20"
                    ]["unweighted_sse_share"],
                    "base_minus_objective_sse_share": stratum_errors[
                        "base::deterministic_offset_minus_0p20"
                    ]["objective_sse_share"],
                    "dominant_smooth_cluster_steps": [415, 421],
                },
                "interpretation": (
                    "equal total stratum mass spreads 500 units across 3,732 "
                    "base-minus rows; its smooth local tails are much cheaper per "
                    "row than the 500-row observer strata and remain underfit"
                ),
            },
            "exact_geometric_alias": {
                "classification": "NOT_SUPPORTED",
                "confidence": "HIGH",
                "evidence": nearest["exact_collision_effective33"],
                "interpretation": (
                    "there are no conflicting exact effective inputs; the observer "
                    "alias is semantic/latent-history aliasing, not a byte-exact "
                    "duplicate-label contradiction"
                ),
            },
            "capacity_or_iteration_limit": {
                "classification": "SECONDARY_AND_NOT_IDENTIFIABLE_IN_ISOLATION",
                "confidence": "MEDIUM",
                "evidence": {
                    "architecture": "W512",
                    "adamw_epochs": dry_fit["optimizer"]["adamw_epochs"],
                    "lbfgs_closure_calls": dry_fit["optimizer"]["lbfgs_closure_calls"],
                    "initial_global_rmse": initial["rmse"],
                    "terminal_global_rmse": terminal["rmse"],
                    "rmse_relative_reduction": 1.0
                    - float(terminal["rmse"]) / float(initial["rmse"]),
                    "initial_global_max_abs": initial["max_abs_error"],
                    "terminal_global_max_abs": terminal["max_abs_error"],
                    "max_abs_relative_reduction": 1.0
                    - float(terminal["max_abs_error"])
                    / float(initial["max_abs_error"]),
                    "final_loss": dry_fit["history"][-1]["loss"],
                    "loss_at_lbfgs_closure_2000": dry_fit["history"][-2]["loss"],
                },
                "interpretation": (
                    "the long fit reduced average error much more than the maximum "
                    "and was still slowly reducing loss at the cap; width, optimizer "
                    "budget, and MSE tail allocation are confounded, but none repairs "
                    "the missing legacy-timeout semantics"
                ),
            },
            "secondary_singletons": {
                "seed128_step318": (
                    "ordinary transition-adjacent action-1 tail: three corpus steps "
                    "before the V26 HS/state transition at step 321 (trace accepts "
                    "the transition at step 320), without a legacy timeout"
                ),
                "base_minus_step130": (
                    "underweighted action-1 tail seven corpus steps before its "
                    "TO/state transition at step 137"
                ),
            },
            "overall": (
                "The terminal failure is mixed: semantic teacher/student transition "
                "aliasing dominates the observer maximum, while per-row weighting "
                "dominates the numerous base-minus tails. Capacity/iteration is a "
                "secondary fit-expression issue, not the sole root cause."
            ),
        },
        "safety": {
            "fit_calls": 0,
            "teacher_policy_queries": 0,
            "environment_reset_calls": 0,
            "environment_step_calls": 0,
            "candidate_published": False,
            "r9_files_modified": False,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT_PATH)
    args = parser.parse_args()
    output = args.output.expanduser().resolve()
    try:
        output.relative_to(DIAGNOSTIC_ROOT.resolve())
    except ValueError as exc:
        raise RuntimeError(
            "output must remain inside the R10 diagnostics tree"
        ) from exc
    payload = analyze()
    output.parent.mkdir(parents=True, exist_ok=True)
    serialized = json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n"
    temporary = output.with_name(f".{output.name}.tmp")
    temporary.write_text(serialized, encoding="utf-8")
    temporary.replace(output)
    print(json.dumps({"status": payload["status"], "output": str(output)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
