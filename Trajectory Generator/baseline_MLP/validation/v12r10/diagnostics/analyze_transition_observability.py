"""Audit whether the legacy-H0 labels are observable from deployable V26 state.

This diagnostic is deliberately read-only with respect to every frozen lineage.
It opens the closed R7/R8/R9 replays, labels, traces, the R9 corpus and selected
R10 diagnostic results.  It reconstructs the authoritative label-only legacy
teacher views without loading or querying a policy.  It never fits a model and
never opens an environment.

The central question is not whether a sufficiently large network can memorize
the finite corpus.  It is whether the target induced by the stateful legacy
teacher projection is a robust Markov function of the 35 deployable V26 actor
features.  Exact duplicate checks alone cannot establish that property, so the
audit also measures hidden transition cliffs and nearest neighbours on opposite
sides of the legacy timeout state.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from collections import Counter
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


def _discover_repo_root(source: Path) -> Path:
    for candidate in source.resolve().parents:
        if (
            (candidate / "AGENTS.md").is_file()
            and (candidate / "Trajectory Generator").is_dir()
            and (candidate / "validation").is_dir()
        ):
            return candidate
    raise RuntimeError("repository root could not be discovered")


SOURCE = Path(__file__).resolve()
DIAGNOSTIC_ROOT = SOURCE.parent
RESULT_ROOT = DIAGNOSTIC_ROOT / "results"
DEFAULT_OUTPUT = RESULT_ROOT / "transition_observability.json"
REPO_ROOT = _discover_repo_root(SOURCE)
BASELINE_ROOT = REPO_ROOT / "Trajectory Generator" / "baseline_MLP"
LOCAL_VALIDATION_ROOT = BASELINE_ROOT / "validation"
for _root in (
    REPO_ROOT,
    REPO_ROOT / "validation",
    REPO_ROOT / "Trajectory Generator",
    BASELINE_ROOT,
    LOCAL_VALIDATION_ROOT,
    LOCAL_VALIDATION_ROOT / "v12r3",
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_primary_split_v12r3_pure_probe_observer_labeler as observer  # noqa: E402


CORPUS_PATH = LOCAL_VALIDATION_ROOT / "v12r9/h0_v12r9_run_20260814/fit/corpus.npz"
CORPUS_SUMMARY_PATH = (
    LOCAL_VALIDATION_ROOT / "v12r9/h0_v12r9_run_20260814/fit/summary.json"
)
FEATURE_MANIFEST_PATH = (
    LOCAL_VALIDATION_ROOT
    / "v12r9/h0_v12r9_run_20260814/fit/rl_module_recovery/actor_feature_manifest.json"
)
R10_FORENSICS_PATH = RESULT_ROOT / "r9_fit_forensics.json"
R10_W512_PATH = RESULT_ROOT / "r9_extended_uniform_reset3_dry_fit.json"
R10_W1024_PATH = RESULT_ROOT / "w1024_r6_residual_reset3_dry_fit.json"
R10_IRLS_PATH = RESULT_ROOT / "r9_extended_reset3_two_stage_irls_tail_dry_fit.json"

CASE_IDS = (
    "deterministic_offset_plus_0p20",
    "deterministic_offset_minus_0p20",
    "deterministic_offset_nominal",
    "stochastic_nominal_seed_126",
    "stochastic_nominal_seed_127",
    "stochastic_nominal_seed_128",
)
V7_PLUS = (
    LOCAL_VALIDATION_ROOT
    / "v12r7/h0_v12r7_run_20260814/observer_collection"
    / "deterministic_offset_plus_0p20"
)
V8_ROOT = LOCAL_VALIDATION_ROOT / "v12r8/h0_v12r8_run_20260814/observer_collection"
V9_ROOT = LOCAL_VALIDATION_ROOT / "v12r9/h0_v12r9_run_20260814/observer_collection"
CASE_PATHS: Mapping[str, Mapping[str, Path]] = {
    "deterministic_offset_plus_0p20": {
        "replay": V7_PLUS / "replay_boundaries.npz",
        "trace": V7_PLUS / "trace.json",
        "labels": (
            V8_ROOT / "deterministic_offset_plus_0p20/observer_labels/labels.npz"
        ),
    },
    "deterministic_offset_minus_0p20": {
        "replay": (V8_ROOT / "deterministic_offset_minus_0p20/replay_boundaries.npz"),
        "trace": V8_ROOT / "deterministic_offset_minus_0p20/trace.json",
        "labels": (
            V9_ROOT / "deterministic_offset_minus_0p20/observer_labels/labels.npz"
        ),
    },
    **{
        case_id: {
            "replay": V9_ROOT / case_id / "replay_boundaries.npz",
            "trace": V9_ROOT / case_id / "trace.json",
            "labels": V9_ROOT / case_id / "observer_labels/labels.npz",
        }
        for case_id in CASE_IDS[2:]
    },
}

FEATURE_COUNT = 35
ACTION_COUNT = 2
DISABLED_CLOCK_COLUMNS = (0, 1)
EFFECTIVE_COLUMNS = tuple(range(2, FEATURE_COUNT))
DETECTOR_FSM_COLUMNS = tuple(range(10, 25))
TIMER_COLUMNS = (16, 22, 23, 24)
STD_FLOOR = np.float32(1.0e-4)
NEAR_DISTANCE_CEILING = 1.0e-2
NEAR_TARGET_DELTA_CEILING = 1.0e-3
ERROR_GATE_CEILING = 0.060


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _record(path: Path) -> dict[str, Any]:
    resolved = path.resolve()
    return {
        "path": resolved.relative_to(REPO_ROOT).as_posix(),
        "sha256": _sha256(resolved),
        "size_bytes": resolved.stat().st_size,
    }


def _strict_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _load_npz(path: Path) -> dict[str, np.ndarray]:
    with np.load(path, allow_pickle=False) as archive:
        return {
            name: np.ascontiguousarray(archive[name].copy()) for name in archive.files
        }


def _phase_code(row: np.ndarray) -> str:
    flags = tuple(bool(float(row[index]) > 0.5) for index in (17, 18, 19))
    if flags == (True, False, False):
        return "WAIT_HS"
    if flags == (False, True, False):
        return "STANCE_AFTER_HS"
    if flags == (False, False, True):
        return "SWING_AFTER_TO"
    expected = tuple(bool(float(row[index]) > 0.5) for index in (20, 21))
    if flags == (False, False, False) and expected == (False, False):
        return "NO_EXPECTED_EVENT"
    return "INCONSISTENT"


def _quantiles(values: Sequence[float]) -> dict[str, float | int | None]:
    array = np.asarray(values, dtype=np.float64)
    if array.size == 0:
        return {
            "count": 0,
            "min": None,
            "p25": None,
            "median": None,
            "p75": None,
            "p95": None,
            "max": None,
        }
    return {
        "count": int(array.size),
        "min": float(np.min(array)),
        "p25": float(np.quantile(array, 0.25)),
        "median": float(np.quantile(array, 0.50)),
        "p75": float(np.quantile(array, 0.75)),
        "p95": float(np.quantile(array, 0.95)),
        "max": float(np.max(array)),
    }


def _normalization(corpus: Mapping[str, np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    selected = np.flatnonzero(corpus["tranche_ids"].astype(str) == "v8r1p1_base")
    if selected.shape != (3000,):
        raise RuntimeError("frozen base-normalization selection drifted")
    source = np.ascontiguousarray(corpus["observations"][selected], dtype=np.float32)
    mean = source.mean(axis=0, dtype=np.float64).astype(np.float32)
    raw_std = source.std(axis=0, dtype=np.float64).astype(np.float32)
    std = np.ascontiguousarray(np.maximum(raw_std, STD_FLOOR), dtype=np.float32)
    if not (
        mean[0].tobytes() == np.float32(0.0).tobytes()
        and mean[1].tobytes() == np.float32(1.0).tobytes()
        and std[0].tobytes() == STD_FLOOR.tobytes()
        and std[1].tobytes() == STD_FLOOR.tobytes()
    ):
        raise RuntimeError("frozen normalization contract drifted")
    return np.ascontiguousarray(mean), std


def _normalize(
    observations: np.ndarray, mean: np.ndarray, std: np.ndarray
) -> np.ndarray:
    result = np.ascontiguousarray(
        (np.asarray(observations, dtype=np.float32) - mean) / std,
        dtype=np.float32,
    )
    if not np.all(np.isfinite(result)):
        raise RuntimeError("normalized observations are non-finite")
    if not np.all(result[:, list(DISABLED_CLOCK_COLUMNS)] == np.float32(0.0)):
        raise RuntimeError("disabled clock columns are not bit zero")
    return result


def _replay_teacher_with_payloads(
    replay: observer.LoadedReplay,
) -> tuple[np.ndarray, list[dict[str, Any]]]:
    """Independently expose payloads while matching the canonical replay bytes."""

    canonical = observer.replay_teacher_views(replay)
    phase_fsm = observer._default_phase_fsm_factory(  # noqa: SLF001
        replay.config,
        replay.fsm_module,
        replay.fsm_class,
    )
    shadow = observer.coherent_teacher.LegacyGaitShadow(phase_fsm)
    students = replay.arrays["actor_observations"]
    body_weight = float(replay.arrays["body_weight_n"][0])
    teachers: list[np.ndarray] = []
    payloads: list[dict[str, Any]] = []
    for boundary in range(replay.n_steps):
        teacher = observer.coherent_teacher.build_teacher_view(
            students[boundary],
            observer.coherent_teacher.EXPECTED_ACTOR_FEATURE_NAMES,
            observer._boundary_info(replay, boundary),  # noqa: SLF001
            body_weight_n=body_weight,
            shadow=shadow,
            reset_boundary=boundary == 0,
        )
        teachers.append(teacher)
        payloads.append(dict(shadow.phase_payload()))
    shadow.consume(
        observer._boundary_info(replay, replay.n_steps),  # noqa: SLF001
        body_weight_n=body_weight,
        reset_boundary=False,
    )
    teacher_array = np.ascontiguousarray(teachers, dtype=np.float32)
    if teacher_array.tobytes(order="C") != canonical.teacher_observations.tobytes(
        order="C"
    ):
        raise RuntimeError("independent teacher payload replay drifted")
    return teacher_array, payloads


def _trace_audit(
    trace_path: Path,
    replay: observer.LoadedReplay,
) -> dict[str, Any]:
    trace = _strict_json(trace_path)
    if not isinstance(trace, list) or len(trace) != replay.n_steps:
        raise RuntimeError(f"trace length drifted: {trace_path}")
    actor_exact = 0
    shifted_phase_exact = 0
    v26_timeout_pulses = 0
    v26_timeout_state_rows = 0
    accepted_events: Counter[str] = Counter()
    legacy_events = 0
    for index, row in enumerate(trace):
        if not isinstance(row, Mapping) or int(row.get("step", -1)) != index + 1:
            raise RuntimeError("trace row identity drifted")
        actor = np.asarray(row.get("v26_observation"), dtype=np.float32)
        if actor.shape != (FEATURE_COUNT,):
            raise RuntimeError("trace actor observation shape drifted")
        actor_exact += int(
            actor.tobytes(order="C")
            == replay.arrays["actor_observations"][index].tobytes(order="C")
        )
        phase = row.get("phase_fsm")
        if not isinstance(phase, Mapping):
            raise RuntimeError("trace phase payload is missing")
        name = str(phase.get("state_name", ""))
        v26_timeout_pulses += int(float(phase.get("timeout_exceeded", 0.0)) > 0.0)
        v26_timeout_state_rows += int(name == "TIMEOUT")
        for event in phase.get("accepted_transitions_this_step", []):
            if isinstance(event, Mapping):
                accepted_events[str(event.get("event", "unknown"))] += 1
        raw_legacy = row.get("legacy_online_events", [])
        if not isinstance(raw_legacy, list):
            raise RuntimeError("trace legacy event journal is malformed")
        legacy_events += len(raw_legacy)
        if index + 1 < replay.n_steps:
            shifted_phase_exact += int(
                _phase_code(replay.arrays["actor_observations"][index + 1]) == name
            )
    if actor_exact != replay.n_steps:
        raise RuntimeError("trace/replay actor observations are not byte exact")
    return {
        "rows": replay.n_steps,
        "actor_observations_byte_exact_count": actor_exact,
        "post_step_phase_matches_next_actor_boundary_count": shifted_phase_exact,
        "post_step_phase_comparison_count": max(0, replay.n_steps - 1),
        "v26_timeout_pulse_count": v26_timeout_pulses,
        "v26_timeout_state_row_count": v26_timeout_state_rows,
        "accepted_transition_counts": dict(sorted(accepted_events.items())),
        "legacy_event_journal_count": legacy_events,
    }


def _case_data(
    case_id: str,
    paths: Mapping[str, Path],
    corpus: Mapping[str, np.ndarray],
) -> dict[str, Any]:
    replay = observer.load_probe_replay_strict(paths["replay"])
    labels = _load_npz(paths["labels"])
    required = {
        "observations",
        "actions",
        "reset_mask",
        "actor_feature_names",
        "case_ids",
        "step_indices",
    }
    if not required.issubset(labels):
        raise RuntimeError(f"label schema is incomplete for {case_id}")
    rows = replay.n_steps
    if (
        labels["observations"].shape != (rows, FEATURE_COUNT)
        or labels["actions"].shape != (rows, ACTION_COUNT)
        or labels["observations"].tobytes(order="C")
        != replay.arrays["actor_observations"].tobytes(order="C")
        or any(str(value) != case_id for value in labels["case_ids"])
        or not np.array_equal(labels["step_indices"], np.arange(1, rows + 1))
    ):
        raise RuntimeError(f"replay/label closure drifted for {case_id}")
    teacher, payloads = _replay_teacher_with_payloads(replay)

    corpus_index = np.flatnonzero(
        corpus["stratum_ids"].astype(str) == f"observer::{case_id}"
    )
    if (
        corpus_index.shape != (rows,)
        or not np.array_equal(
            corpus["step_indices"][corpus_index], np.arange(1, rows + 1)
        )
        or corpus["observations"][corpus_index].tobytes(order="C")
        != labels["observations"].tobytes(order="C")
        or corpus["actions"][corpus_index].tobytes(order="C")
        != labels["actions"].tobytes(order="C")
    ):
        raise RuntimeError(f"observer labels/corpus closure drifted for {case_id}")
    return {
        "case_id": case_id,
        "paths": {name: _record(path) for name, path in paths.items()},
        "observations": labels["observations"],
        "actions": labels["actions"],
        "teacher_observations": teacher,
        "payloads": payloads,
        "step_indices": labels["step_indices"],
        "corpus_indices": corpus_index,
        "trace_audit": _trace_audit(paths["trace"], replay),
        "replay_config": dict(replay.config),
        "replay_config_sha256": replay.config_sha256,
        "event_count": replay.event_count,
    }


def _transition_record(
    case: Mapping[str, Any],
    index: int,
    normalized: np.ndarray,
) -> dict[str, Any]:
    observations = case["observations"]
    teachers = case["teacher_observations"]
    actions = case["actions"]
    payloads = case["payloads"]
    previous = index - 1
    delta = (
        normalized[index, list(EFFECTIVE_COLUMNS)]
        - normalized[previous, list(EFFECTIVE_COLUMNS)]
    )
    distance_l2 = float(np.linalg.norm(delta))
    action_delta = np.abs(
        actions[index].astype(np.float64) - actions[previous].astype(np.float64)
    )
    teacher_elapsed_previous = max(
        float(teachers[previous, 22]), float(teachers[previous, 23])
    )
    teacher_elapsed_current = max(
        float(teachers[index, 22]), float(teachers[index, 23])
    )
    previous_timeout = str(payloads[previous].get("state_name")) == "TIMEOUT"
    current_timeout = str(payloads[index].get("state_name")) == "TIMEOUT"
    student_previous = _phase_code(observations[previous])
    student_current = _phase_code(observations[index])
    student_pulse = bool(
        np.any(observations[index, 12:14] > np.float32(0.5))
        or np.any(observations[previous, 12:14] > np.float32(0.5))
    )
    teacher_pulse = bool(
        np.any(teachers[index, 12:14] > np.float32(0.5))
        or np.any(teachers[previous, 12:14] > np.float32(0.5))
    )
    teacher_state_previous = str(payloads[previous].get("state_name"))
    teacher_state_current = str(payloads[index].get("state_name"))
    student_discrete = student_previous != student_current or student_pulse
    teacher_discrete = teacher_state_previous != teacher_state_current or teacher_pulse
    post_timeout_elapsed_reset = bool(
        previous_timeout
        and current_timeout
        and teacher_elapsed_previous > 0.5
        and teacher_elapsed_current < 1.0e-7
    )
    return {
        "case_id": case["case_id"],
        "previous_step": int(case["step_indices"][previous]),
        "current_step": int(case["step_indices"][index]),
        "student_phase_previous": student_previous,
        "student_phase_current": student_current,
        "teacher_phase_previous": teacher_state_previous,
        "teacher_phase_current": teacher_state_current,
        "teacher_timeout_entry": bool(current_timeout and not previous_timeout),
        "post_timeout_elapsed_reset": post_timeout_elapsed_reset,
        "student_discrete_transition_or_pulse": student_discrete,
        "teacher_discrete_transition_or_pulse": teacher_discrete,
        "legacy_change_hidden_from_student_discrete_block": bool(
            (teacher_discrete or post_timeout_elapsed_reset) and not student_discrete
        ),
        "student_stance_elapsed_norm_previous": float(observations[previous, 22]),
        "student_stance_elapsed_norm_current": float(observations[index, 22]),
        "student_swing_elapsed_norm_previous": float(observations[previous, 23]),
        "student_swing_elapsed_norm_current": float(observations[index, 23]),
        "teacher_stance_elapsed_norm_previous": float(teachers[previous, 22]),
        "teacher_stance_elapsed_norm_current": float(teachers[index, 22]),
        "teacher_swing_elapsed_norm_previous": float(teachers[previous, 23]),
        "teacher_swing_elapsed_norm_current": float(teachers[index, 23]),
        "normalized_effective_l2": distance_l2,
        "normalized_effective_rms": distance_l2 / math.sqrt(len(EFFECTIVE_COLUMNS)),
        "target_previous": actions[previous].astype(float).tolist(),
        "target_current": actions[index].astype(float).tolist(),
        "target_delta_linf": float(np.max(action_delta)),
        "target_delta_l2": float(np.linalg.norm(action_delta)),
        "local_target_slope_per_normalized_rms": float(
            np.max(action_delta)
            / max(distance_l2 / math.sqrt(len(EFFECTIVE_COLUMNS)), 1.0e-15)
        ),
    }


def _case_transition_audit(
    case: Mapping[str, Any],
    mean: np.ndarray,
    std: np.ndarray,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    observations = case["observations"]
    payloads = case["payloads"]
    normalized = _normalize(observations, mean, std)
    timeout = np.asarray(
        [str(payload.get("state_name")) == "TIMEOUT" for payload in payloads],
        dtype=np.bool_,
    )
    transitions = [
        _transition_record(case, index, normalized)
        for index in range(1, len(observations))
    ]
    timeout_entries = [row for row in transitions if row["teacher_timeout_entry"]]
    post_resets = [row for row in transitions if row["post_timeout_elapsed_reset"]]
    hidden = [
        row
        for row in transitions
        if row["legacy_change_hidden_from_student_discrete_block"]
    ]
    action_cliffs = [row for row in transitions if row["target_delta_linf"] > 0.060]
    student_codes = [_phase_code(row) for row in observations]
    teacher_codes = [str(payload.get("state_name")) for payload in payloads]
    mismatch = sum(left != right for left, right in zip(student_codes, teacher_codes))
    timeout_student_codes = Counter(
        student_codes[index] for index in np.flatnonzero(timeout)
    )
    timer_ranges: dict[str, Any] = {}
    for column in TIMER_COLUMNS:
        name = str(case["feature_names"][column])
        timer_ranges[name] = {
            "legacy_timeout_rows": _quantiles(observations[timeout, column]),
            "legacy_non_timeout_rows": _quantiles(observations[~timeout, column]),
        }
    return (
        {
            "rows": len(observations),
            "legacy_teacher_state_counts": dict(sorted(Counter(teacher_codes).items())),
            "v26_student_state_counts": dict(sorted(Counter(student_codes).items())),
            "teacher_student_phase_mismatch_count": mismatch,
            "legacy_timeout_row_count": int(np.count_nonzero(timeout)),
            "legacy_timeout_entry_count": len(timeout_entries),
            "legacy_post_timeout_elapsed_reset_count": len(post_resets),
            "v26_state_during_legacy_timeout": dict(
                sorted(timeout_student_codes.items())
            ),
            "hidden_legacy_transition_count": len(hidden),
            "adjacent_target_cliff_over_0p060_count": len(action_cliffs),
            "timeout_entries": timeout_entries,
            "post_timeout_elapsed_resets": post_resets,
            "v26_timer_ranges_by_legacy_timeout_side": timer_ranges,
            "trace": case["trace_audit"],
            "legacy_config_hard_timeouts_s": {
                "stance": float(case["replay_config"]["stance_hard_timeout_s"]),
                "swing": float(case["replay_config"]["swing_hard_timeout_s"]),
            },
        },
        transitions,
    )


def _pair_record(
    source_index: int,
    neighbour_index: int,
    observations: np.ndarray,
    actions: np.ndarray,
    cases: np.ndarray,
    steps: np.ndarray,
    payloads: Sequence[Mapping[str, Any]],
    distance_l2: float,
    columns: Sequence[int],
) -> dict[str, Any]:
    action_delta = np.abs(
        actions[source_index].astype(np.float64)
        - actions[neighbour_index].astype(np.float64)
    )
    return {
        "timeout_case_id": str(cases[source_index]),
        "timeout_step": int(steps[source_index]),
        "non_timeout_case_id": str(cases[neighbour_index]),
        "non_timeout_step": int(steps[neighbour_index]),
        "timeout_student_phase": _phase_code(observations[source_index]),
        "non_timeout_student_phase": _phase_code(observations[neighbour_index]),
        "timeout_teacher_phase": str(payloads[source_index].get("state_name")),
        "non_timeout_teacher_phase": str(payloads[neighbour_index].get("state_name")),
        "normalized_l2": float(distance_l2),
        "normalized_rms": float(distance_l2 / math.sqrt(len(columns))),
        "target_timeout": actions[source_index].astype(float).tolist(),
        "target_non_timeout": actions[neighbour_index].astype(float).tolist(),
        "target_delta_linf": float(np.max(action_delta)),
        "target_delta_l2": float(np.linalg.norm(action_delta)),
        "v26_timer_timeout": observations[source_index, list(TIMER_COLUMNS)]
        .astype(float)
        .tolist(),
        "v26_timer_non_timeout": observations[neighbour_index, list(TIMER_COLUMNS)]
        .astype(float)
        .tolist(),
    }


def _nearest_cross_side(
    normalized: np.ndarray,
    observations: np.ndarray,
    actions: np.ndarray,
    cases: np.ndarray,
    steps: np.ndarray,
    payloads: Sequence[Mapping[str, Any]],
    timeout: np.ndarray,
    *,
    columns: Sequence[int],
) -> dict[str, Any]:
    timeout_indices = np.flatnonzero(timeout)
    other_indices = np.flatnonzero(~timeout)
    selected_timeout = normalized[timeout_indices][:, list(columns)].astype(np.float64)
    selected_other = normalized[other_indices][:, list(columns)].astype(np.float64)
    if len(timeout_indices) == 0 or len(other_indices) == 0:
        raise RuntimeError("cross-side nearest-neighbour audit has an empty side")
    squared = (
        np.sum(np.square(selected_timeout), axis=1)[:, None]
        + np.sum(np.square(selected_other), axis=1)[None, :]
        - 2.0 * selected_timeout @ selected_other.T
    )
    distances = np.sqrt(np.maximum(squared, 0.0))
    nearest_local = np.argmin(distances, axis=1)
    nearest_distance = distances[np.arange(len(timeout_indices)), nearest_local]
    neighbours = other_indices[nearest_local]
    records = [
        _pair_record(
            int(source),
            int(neighbour),
            observations,
            actions,
            cases,
            steps,
            payloads,
            float(distance),
            columns,
        )
        for source, neighbour, distance in zip(
            timeout_indices, neighbours, nearest_distance
        )
    ]
    target_delta = np.asarray(
        [float(record["target_delta_linf"]) for record in records], dtype=np.float64
    )
    rms = nearest_distance / math.sqrt(len(columns))
    by_smallest_distance = sorted(
        records,
        key=lambda row: (
            float(row["normalized_rms"]),
            -float(row["target_delta_linf"]),
            str(row["timeout_case_id"]),
            int(row["timeout_step"]),
        ),
    )[:20]
    by_largest_target_delta = sorted(
        records,
        key=lambda row: (
            -float(row["target_delta_linf"]),
            float(row["normalized_rms"]),
            str(row["timeout_case_id"]),
            int(row["timeout_step"]),
        ),
    )[:20]
    return {
        "columns": list(columns),
        "timeout_rows": len(timeout_indices),
        "non_timeout_rows": len(other_indices),
        "nearest_normalized_l2": _quantiles(nearest_distance),
        "nearest_normalized_rms": _quantiles(rms),
        "nearest_target_delta_linf": _quantiles(target_delta),
        "nearest_pairs_target_delta_over_0p060_count": int(
            np.count_nonzero(target_delta > ERROR_GATE_CEILING)
        ),
        "nearest_pairs_rms_below_0p25_and_target_delta_over_0p060_count": int(
            np.count_nonzero((rms < 0.25) & (target_delta > ERROR_GATE_CEILING))
        ),
        "smallest_distance_pairs": by_smallest_distance,
        "largest_target_delta_pairs": by_largest_target_delta,
    }


def _same_case_nearest_summary(
    normalized: np.ndarray,
    observations: np.ndarray,
    actions: np.ndarray,
    cases: np.ndarray,
    steps: np.ndarray,
    payloads: Sequence[Mapping[str, Any]],
    timeout: np.ndarray,
) -> dict[str, Any]:
    output: dict[str, Any] = {}
    for case_id in CASE_IDS:
        selected = np.flatnonzero(cases.astype(str) == case_id)
        case_timeout = timeout[selected]
        if not np.any(case_timeout):
            output[case_id] = {
                "available": False,
                "reason": "no legacy timeout row in the closed prefix",
            }
            continue
        local = _nearest_cross_side(
            normalized[selected],
            observations[selected],
            actions[selected],
            cases[selected],
            steps[selected],
            [payloads[index] for index in selected],
            case_timeout,
            columns=EFFECTIVE_COLUMNS,
        )
        output[case_id] = {
            "available": True,
            "nearest_normalized_rms": local["nearest_normalized_rms"],
            "nearest_target_delta_linf": local["nearest_target_delta_linf"],
            "largest_target_delta_pairs": local["largest_target_delta_pairs"][:5],
        }
    return output


def _global_adjacent_corpus_audit(
    corpus: Mapping[str, np.ndarray], normalized: np.ndarray
) -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    episode_ids = corpus["episode_ids"].astype(str)
    for episode_id in sorted(set(episode_ids)):
        selected = np.flatnonzero(episode_ids == episode_id)
        selected = selected[np.argsort(corpus["step_indices"][selected], kind="stable")]
        for previous, current in zip(selected[:-1], selected[1:]):
            if (
                int(corpus["step_indices"][current])
                != int(corpus["step_indices"][previous]) + 1
            ):
                continue
            delta = (
                normalized[current, list(EFFECTIVE_COLUMNS)]
                - normalized[previous, list(EFFECTIVE_COLUMNS)]
            )
            distance = float(np.linalg.norm(delta)) / math.sqrt(len(EFFECTIVE_COLUMNS))
            action_delta = float(
                np.max(
                    np.abs(
                        corpus["actions"][current].astype(np.float64)
                        - corpus["actions"][previous].astype(np.float64)
                    )
                )
            )
            student_discrete = bool(
                _phase_code(corpus["observations"][current])
                != _phase_code(corpus["observations"][previous])
                or np.any(corpus["observations"][current, 12:14] > 0.5)
                or np.any(corpus["observations"][previous, 12:14] > 0.5)
            )
            if action_delta > ERROR_GATE_CEILING:
                rows.append(
                    {
                        "episode_id": episode_id,
                        "case_id": str(corpus["case_ids"][current]),
                        "stratum_id": str(corpus["stratum_ids"][current]),
                        "tranche_id": str(corpus["tranche_ids"][current]),
                        "previous_step": int(corpus["step_indices"][previous]),
                        "current_step": int(corpus["step_indices"][current]),
                        "target_delta_linf": action_delta,
                        "normalized_effective_rms": distance,
                        "student_discrete_transition_or_pulse": student_discrete,
                        "student_phase_previous": _phase_code(
                            corpus["observations"][previous]
                        ),
                        "student_phase_current": _phase_code(
                            corpus["observations"][current]
                        ),
                    }
                )
    ordered = sorted(
        rows,
        key=lambda row: (
            -float(row["target_delta_linf"]),
            float(row["normalized_effective_rms"]),
            str(row["episode_id"]),
            int(row["current_step"]),
        ),
    )
    return {
        "episode_count": len(set(episode_ids)),
        "adjacent_target_cliff_over_0p060_count": len(rows),
        "cliffs_without_student_discrete_transition_or_pulse_count": sum(
            not bool(row["student_discrete_transition_or_pulse"]) for row in rows
        ),
        "largest_cliffs": ordered[:30],
    }


def _r10_evidence() -> dict[str, Any]:
    forensics = _strict_json(R10_FORENSICS_PATH)
    w512 = _strict_json(R10_W512_PATH)
    w1024 = _strict_json(R10_W1024_PATH)
    irls = _strict_json(R10_IRLS_PATH)
    w512_metrics = w512["terminal_metrics"]
    w1024_metrics = w1024["primary"]["metrics"]
    return {
        "inputs": {
            "r9_forensics": _record(R10_FORENSICS_PATH),
            "extended_w512": _record(R10_W512_PATH),
            "residual_w1024": _record(R10_W1024_PATH),
            "two_stage_irls": _record(R10_IRLS_PATH),
        },
        "finite_corpus_exact_conflict_lower_bound": forensics["collision_lower_bounds"],
        "w512": {
            "decision": w512["decision"],
            "global_metrics": w512_metrics["global_metrics"],
            "worst_row": w512_metrics["worst_row"],
        },
        "w1024": {
            "decision": w1024["decision"],
            "global_metrics": w1024_metrics["global_metrics"],
            "worst_row": w1024_metrics["worst_row"],
        },
        "same_worst_row_across_w512_w1024": bool(
            w512_metrics["worst_row"]["case_id"]
            == w1024_metrics["worst_row"]["case_id"]
            == "deterministic_offset_nominal"
            and int(w512_metrics["worst_row"]["step_index"])
            == int(w1024_metrics["worst_row"]["step_index"])
            == 385
        ),
        "two_stage_irls": {
            "decision": irls["decision"],
            "global_rmse": irls["phase_b_terminal_observation"]["global_rmse"],
            "global_max_abs_error": irls["phase_b_terminal_observation"][
                "global_max_abs_error"
            ],
            "terminal_model_result_is_fail": irls["terminal_interpretation"][
                "model_result_is_fail_independent_of_fold_instrumentation_abort"
            ],
        },
    }


def _options_and_recommendation(
    *,
    timeout_rows: int,
    v26_timeout_rows: int,
    hidden_transitions: int,
    same_worst: bool,
) -> dict[str, Any]:
    evidence_complete = bool(
        timeout_rows > 0
        and v26_timeout_rows == 0
        and hidden_transitions > 0
        and same_worst
    )
    return {
        "finite_sample_conclusion": (
            "No exact conflicting labels were observed, so mathematical "
            "impossibility of a lookup on this finite corpus is not proven. "
            "The deployed Markov35 semantics are nevertheless not identified: "
            "the label is generated by a stateful legacy shadow whose timeout "
            "state and elapsed reset are absent from the V26 actor state."
        ),
        "options": {
            "A_continue_gate_aligned_markov35_fitting": {
                "decision": "REJECT_AS_PRIMARY_BLOCKER_RESOLUTION",
                "reason": (
                    "W512, W1024 and tail-weighted optimization preserve the same "
                    "hidden-transition error family. More capacity may memorize "
                    "the closed paths but cannot make the legacy projection a "
                    "well-defined deployable Markov target."
                ),
            },
            "B_relabel_with_v26_coherent_projection": {
                "decision": "RECOMMEND",
                "required_semantics": (
                    "Freeze a stateless byte-deterministic projection P of the "
                    "exact deployable V26-35 row, and define every imitation "
                    "target as H0(P(x)). P may not consume legacy events, analog "
                    "legacy detector fields, replay history or a hidden shadow. "
                    "Identity P(x)=x is the smallest auditable design."
                ),
                "detector_compatibility": (
                    "Collection and deployment remain teacher-free with the "
                    "binary V26 detector active; the teacher is queried only "
                    "offline on information available to the deployed actor."
                ),
            },
            "C_add_v26_derived_timer_feature": {
                "decision": "REJECT_WITH_CURRENT_EVIDENCE",
                "reason": (
                    "V26 cycle duration, stance elapsed, swing elapsed and cycle "
                    "credit already occupy columns 16, 22, 23 and 24. A stateless "
                    "function of the same row adds no information about the "
                    "divergent legacy event history. A V26-history accumulator "
                    "would be a new stateful policy contract, not a Markov35 fix."
                ),
            },
            "D_stateful_adapter_or_shadow": {
                "decision": "REJECT_LEGACY_SHADOW",
                "reason": (
                    "The exact adapter that restores the current targets is "
                    "LegacyGaitShadow, which consumes legacy_online_events and "
                    "legacy analog detector state. Serving it online would "
                    "reactivate the dependency the binary V26 migration removes. "
                    "A V26-only recurrent adapter is causally possible but is a "
                    "materially new recurrent-policy project and does not justify "
                    "retaining the legacy target."
                ),
            },
        },
        "univocal_recommendation": (
            "SELECT_OPTION_B_AND_BLOCK_THE_CURRENT_LEGACY_SHADOW_LABEL_CONTRACT"
        ),
        "recommendation_evidence_complete": evidence_complete,
        "fail_closed_criteria": [
            "P must be a pure stateless function of one exact V26 float32 actor row.",
            "No legacy_online_events, LegacyGaitShadow, legacy analog GRF, or hidden history may enter P or deployed inference.",
            "Relabel from immutable replays; do not recollect physical rollouts merely to change labels.",
            f"Exact duplicate target conflicts must remain zero; for effective normalized L2 <= {NEAR_DISTANCE_CEILING:g}, target Linf delta must be <= {NEAR_TARGET_DELTA_CEILING:g}.",
            "Keep every existing RMSE, max-error, reset, per-case and observer gate unchanged.",
            "A failed offline gate blocks candidate publication; no fallback to A, C or D is implicit.",
            "All pure-policy development, Q3, checkpoint-zero, morphology and EnvRunner gates must be rerun on the new lineage.",
            "Every rollout and training consumer must attest binary_active_v26 routing and zero teacher/legacy served-action dependency.",
        ],
    }


def run() -> dict[str, Any]:
    corpus = _load_npz(CORPUS_PATH)
    summary = _strict_json(CORPUS_SUMMARY_PATH)
    manifest = _strict_json(FEATURE_MANIFEST_PATH)
    if (
        corpus["observations"].shape != (11875, FEATURE_COUNT)
        or corpus["actions"].shape != (11875, ACTION_COUNT)
        or _sha256(CORPUS_PATH) != summary["corpus"]["sha256"]
        or list(corpus["actor_feature_names"].astype(str))
        != manifest["actor_feature_names"]
        or int(manifest["actor_feature_count"]) != FEATURE_COUNT
        or tuple(manifest["disabled_clock_columns"]) != DISABLED_CLOCK_COLUMNS
    ):
        raise RuntimeError("frozen R9 corpus/feature manifest closure failed")
    feature_names = tuple(corpus["actor_feature_names"].astype(str))
    mean, std = _normalization(corpus)
    normalized_corpus = _normalize(corpus["observations"], mean, std)

    cases: list[dict[str, Any]] = []
    case_audits: dict[str, Any] = {}
    all_transitions: list[dict[str, Any]] = []
    for case_id in CASE_IDS:
        case = _case_data(case_id, CASE_PATHS[case_id], corpus)
        case["feature_names"] = feature_names
        audit, transitions = _case_transition_audit(case, mean, std)
        cases.append(case)
        case_audits[case_id] = audit
        all_transitions.extend(transitions)

    observations = np.ascontiguousarray(
        np.concatenate([case["observations"] for case in cases]), dtype=np.float32
    )
    actions = np.ascontiguousarray(
        np.concatenate([case["actions"] for case in cases]), dtype=np.float32
    )
    case_values = np.concatenate(
        [
            np.full(len(case["observations"]), case["case_id"], dtype="U64")
            for case in cases
        ]
    )
    steps = np.concatenate([case["step_indices"] for case in cases])
    payloads = [payload for case in cases for payload in case["payloads"]]
    timeout = np.asarray(
        [str(payload.get("state_name")) == "TIMEOUT" for payload in payloads],
        dtype=np.bool_,
    )
    normalized = _normalize(observations, mean, std)
    v26_timeout = np.asarray(
        [_phase_code(row) == "NO_EXPECTED_EVENT" for row in observations],
        dtype=np.bool_,
    )
    hidden = [
        row
        for row in all_transitions
        if row["legacy_change_hidden_from_student_discrete_block"]
    ]
    timeout_related = [
        row
        for row in all_transitions
        if row["teacher_timeout_entry"] or row["post_timeout_elapsed_reset"]
    ]
    target_cliffs = [row for row in all_transitions if row["target_delta_linf"] > 0.060]
    ordered_cliffs = sorted(
        target_cliffs,
        key=lambda row: (
            -float(row["target_delta_linf"]),
            float(row["normalized_effective_rms"]),
            str(row["case_id"]),
            int(row["current_step"]),
        ),
    )
    r10 = _r10_evidence()
    recommendation = _options_and_recommendation(
        timeout_rows=int(np.count_nonzero(timeout)),
        v26_timeout_rows=int(np.count_nonzero(v26_timeout)),
        hidden_transitions=len(hidden),
        same_worst=bool(r10["same_worst_row_across_w512_w1024"]),
    )
    checks = {
        "frozen_r9_inputs_closed": True,
        "all_six_replays_labels_and_corpus_closed": True,
        "trace_actor_rows_byte_exact": all(
            audit["trace"]["actor_observations_byte_exact_count"] == audit["rows"]
            for audit in case_audits.values()
        ),
        "legacy_timeout_observed": bool(np.any(timeout)),
        "v26_timeout_absent_on_observer_rows": not bool(np.any(v26_timeout)),
        "hidden_legacy_transition_observed": bool(hidden),
        "same_r10_worst_row_after_width_doubling": bool(
            r10["same_worst_row_across_w512_w1024"]
        ),
        "elapsed_cycle_features_already_available": tuple(
            feature_names[index] for index in TIMER_COLUMNS
        )
        == (
            "online_left_cycle_duration_s",
            "phase_stance_elapsed_norm",
            "phase_swing_elapsed_norm",
            "phase_cycle_progress_credit",
        ),
        "recommendation_evidence_complete": recommendation[
            "recommendation_evidence_complete"
        ],
        "zero_fit_zero_environment_by_construction": True,
    }
    if not all(checks.values()):
        raise RuntimeError(f"transition observability audit failed closed: {checks}")

    result = {
        "schema_version": 1,
        "status": "COMPLETE_H0_V12R10_TRANSITION_OBSERVABILITY_AUDIT",
        "passed": True,
        "training_ready": False,
        "current_label_contract_decision": "BLOCK_LEGACY_SHADOW_PROJECTED_MARKOV35",
        "scope": {
            "read_only": True,
            "model_fit_calls": 0,
            "environment_reset_calls": 0,
            "environment_step_calls": 0,
            "teacher_policy_load_calls": 0,
            "teacher_policy_query_calls": 0,
            "candidate_publications": 0,
            "protected_or_reserve_trial_opens": 0,
        },
        "diagnostic_source": _record(SOURCE),
        "frozen_inputs": {
            "corpus": _record(CORPUS_PATH),
            "corpus_summary": _record(CORPUS_SUMMARY_PATH),
            "actor_feature_manifest": _record(FEATURE_MANIFEST_PATH),
            "cases": {case["case_id"]: case["paths"] for case in cases},
        },
        "actor_contract": {
            "feature_count": FEATURE_COUNT,
            "feature_names": list(feature_names),
            "disabled_clock_columns": list(DISABLED_CLOCK_COLUMNS),
            "effective_markov_columns": list(EFFECTIVE_COLUMNS),
            "available_elapsed_cycle_features": [
                {"index": index, "name": feature_names[index]}
                for index in TIMER_COLUMNS
            ],
            "absent_fields": [
                "legacy_timeout_state",
                "legacy_stance_elapsed",
                "legacy_swing_elapsed",
                "legacy_event_history",
                "episode_elapsed_time",
            ],
            "v26_timeout_diagnostic_is_not_an_actor_scalar": "timeout_exceeded",
        },
        "case_audits": case_audits,
        "aggregate_transition_evidence": {
            "observer_rows": len(observations),
            "legacy_timeout_rows": int(np.count_nonzero(timeout)),
            "v26_timeout_rows": int(np.count_nonzero(v26_timeout)),
            "teacher_timeout_entry_count": sum(
                bool(row["teacher_timeout_entry"]) for row in all_transitions
            ),
            "teacher_post_timeout_elapsed_reset_count": sum(
                bool(row["post_timeout_elapsed_reset"]) for row in all_transitions
            ),
            "hidden_legacy_transition_count": len(hidden),
            "adjacent_target_cliff_over_0p060_count": len(target_cliffs),
            "adjacent_target_cliffs_without_student_discrete_transition_count": sum(
                not bool(row["student_discrete_transition_or_pulse"])
                for row in target_cliffs
            ),
            "timeout_entry_and_elapsed_reset_records": timeout_related,
            "largest_adjacent_target_cliffs": ordered_cliffs[:30],
            "hidden_transition_target_delta_linf": _quantiles(
                [float(row["target_delta_linf"]) for row in hidden]
            ),
            "timeout_related_target_delta_linf": _quantiles(
                [float(row["target_delta_linf"]) for row in timeout_related]
            ),
        },
        "nearest_neighbour_cross_side": {
            "effective_33_features_global": _nearest_cross_side(
                normalized,
                observations,
                actions,
                case_values,
                steps,
                payloads,
                timeout,
                columns=EFFECTIVE_COLUMNS,
            ),
            "detector_fsm_15_features_global": _nearest_cross_side(
                normalized,
                observations,
                actions,
                case_values,
                steps,
                payloads,
                timeout,
                columns=DETECTOR_FSM_COLUMNS,
            ),
            "effective_33_features_within_case": _same_case_nearest_summary(
                normalized,
                observations,
                actions,
                case_values,
                steps,
                payloads,
                timeout,
            ),
        },
        "full_corpus_adjacent_audit": _global_adjacent_corpus_audit(
            corpus, normalized_corpus
        ),
        "r10_fit_evidence": r10,
        "interpretation": {
            "exact_collision_result": (
                "The finite corpus has no exact conflicting target for an exact "
                "35-field row. This is necessary but not sufficient for a robust "
                "Markov contract."
            ),
            "causal_observability_result": (
                "Legacy timeout entry and its following elapsed reset are produced "
                "by a hidden legacy event history. During those rows the deployed "
                "V26 FSM remains in a different valid state and changes smoothly."
            ),
            "capacity_result": (
                "Doubling hidden width reduces aggregate error but leaves the exact "
                "same nominal step-385 row as the worst error, which is consistent "
                "with a target-semantic cliff rather than a width-only blocker."
            ),
            "impossibility_precision": (
                "Impossibility is not proved on the finite sample. Generalizable "
                "Markov35 imitation of the current stateful legacy projection is "
                "semantically fragile and must not be certified training-ready."
            ),
        },
        "recommendation": recommendation,
        "checks": checks,
    }
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    result = run()
    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    payload = (
        json.dumps(
            result,
            sort_keys=True,
            indent=2,
            allow_nan=False,
        )
        + "\n"
    )
    output.write_text(payload, encoding="utf-8")
    try:
        output_label = output.relative_to(REPO_ROOT).as_posix()
    except ValueError:
        output_label = output.as_posix()
    print(
        json.dumps(
            {
                "status": result["status"],
                "passed": result["passed"],
                "training_ready": result["training_ready"],
                "current_label_contract_decision": result[
                    "current_label_contract_decision"
                ],
                "output": output_label,
                "output_sha256": _sha256(output),
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
