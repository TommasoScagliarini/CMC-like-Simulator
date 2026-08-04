#!/usr/bin/env python3
"""Independently validate the experimental retrospective morphology corridor.

The validator consumes a ``rollout_policy_trace.json`` that contains either:

* ``phase_fsm.accepted_transitions_this_step`` on each policy step; or
* ``morphology_completed_segments`` emitted by the reward wrapper; or
* both, in which case the two representations are cross-checked.

No runtime morphology helper is imported.  Profile parsing, event-normalized
phase, corridor interpolation, interval loss, sample ownership, and all
cross-checks are implemented locally.  The half-open ownership contract is:

* stance: ``[HS, TO)``;
* swing: ``[TO, next_HS)``;
* a sample exactly on an event belongs to the newly opened segment.

Examples
--------

Validate a recorded rollout::

    python validation/validate_experimental_retrospective_morphology.py \
      validation/.../rollout_policy_trace.json

Exercise the explicit synthetic trace/profile fixture::

    python validation/validate_experimental_retrospective_morphology.py \
      --self-test --output-dir validation/retrospective_morphology_self_test
"""

from __future__ import annotations

import argparse
import json
import math
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

os.environ.setdefault(
    "MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "cmc_like_matplotlib")
)
os.environ.setdefault(
    "XDG_CACHE_HOME", str(Path(tempfile.gettempdir()) / "cmc_like_cache")
)

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
BASELINE_DIR = REPO_ROOT / "Trajectory Generator" / "baseline_MLP"
DEFAULT_PROFILE = (
    BASELINE_DIR
    / "morphology_profiles"
    / "ab06_prosthetic_event_warped_mean_std_corridor.json"
)
EXPECTED_PARAMETERIZATION = "event_warped_hs_to_to_to_hs_v1"
TIME_EPS = 1.0e-12
MATCH_TIME_TOL_S = 1.0e-9
VALUE_TOL = 1.0e-9


@dataclass(frozen=True)
class ParsedTraceRow:
    row_index: int
    step: int
    time_s: float | None
    knee_rad: float | None
    ankle_rad: float | None
    transitions: tuple[dict[str, Any], ...]
    completed_segments: tuple[dict[str, Any], ...]
    ledger_diagnostics: dict[str, Any]
    reward_terms: dict[str, Any]


@dataclass(frozen=True)
class SegmentDefinition:
    segment_id: int
    segment_type: str
    start_time_s: float
    end_time_s: float
    source: str

    @property
    def duration_s(self) -> float:
        return float(self.end_time_s - self.start_time_s)


@dataclass(frozen=True)
class ProfileData:
    path: Path
    phase: np.ndarray
    canonical_to_phase: float
    mean: dict[str, np.ndarray]
    std: dict[str, np.ndarray]
    metadata: dict[str, Any]


def _finite_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if math.isfinite(result) else None


def _first_mapping(*values: Any) -> Mapping[str, Any] | None:
    for value in values:
        if isinstance(value, Mapping):
            return value
    return None


def _load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def _trace_rows(value: Any) -> tuple[list[Any], Mapping[str, Any]]:
    if isinstance(value, list):
        return value, {}
    if not isinstance(value, Mapping):
        raise ValueError("rollout trace must be a JSON list or object")
    for key in ("steps", "trace", "policy_trace", "rows"):
        rows = value.get(key)
        if isinstance(rows, list):
            return rows, value
    raise ValueError(
        "trace JSON object must contain a list under one of: "
        "steps, trace, policy_trace, rows"
    )


def _extract_state(row: Mapping[str, Any]) -> Mapping[str, Any] | None:
    info = row.get("info") if isinstance(row.get("info"), Mapping) else {}
    return _first_mapping(
        row.get("prosthetic_state"),
        row.get("observation"),
        info.get("prosthetic_state"),
        info.get("observation"),
    )


def _extract_transitions(row: Mapping[str, Any]) -> tuple[dict[str, Any], ...]:
    info = row.get("info") if isinstance(row.get("info"), Mapping) else {}
    phase_fsm = _first_mapping(row.get("phase_fsm"), info.get("phase_fsm"))
    candidates: list[Any] = []
    if phase_fsm is not None:
        candidates.append(phase_fsm.get("accepted_transitions_this_step"))
    candidates.extend(
        (
            row.get("accepted_transitions_this_step"),
            info.get("accepted_transitions_this_step"),
        )
    )
    for candidate in candidates:
        if isinstance(candidate, (list, tuple)):
            return tuple(dict(item) for item in candidate if isinstance(item, Mapping))
    return ()


def _extract_completed_segments(
    row: Mapping[str, Any],
) -> tuple[dict[str, Any], ...]:
    info = row.get("info") if isinstance(row.get("info"), Mapping) else {}
    for candidate in (
        row.get("morphology_completed_segments"),
        info.get("morphology_completed_segments"),
    ):
        if isinstance(candidate, (list, tuple)):
            return tuple(dict(item) for item in candidate if isinstance(item, Mapping))
    return ()


def _extract_row_mapping(row: Mapping[str, Any], key: str) -> dict[str, Any]:
    info = row.get("info") if isinstance(row.get("info"), Mapping) else {}
    for candidate in (row.get(key), info.get(key)):
        if isinstance(candidate, Mapping):
            return dict(candidate)
    return {}


def parse_trace(
    path: Path,
) -> tuple[list[ParsedTraceRow], Mapping[str, Any], dict[str, Any]]:
    raw_rows, root = _trace_rows(_load_json(path))
    parsed: list[ParsedTraceRow] = []
    invalid_rows = 0
    missing_time_rows = 0
    missing_served_rows = 0
    transition_rows = 0
    payload_rows = 0
    ledger_diagnostic_rows = 0
    reward_term_rows = 0

    for row_index, raw in enumerate(raw_rows):
        if not isinstance(raw, Mapping):
            invalid_rows += 1
            continue
        info = raw.get("info") if isinstance(raw.get("info"), Mapping) else {}
        time_s = _finite_float(raw.get("time"))
        if time_s is None:
            time_s = _finite_float(info.get("time"))
        if time_s is None:
            missing_time_rows += 1

        state = _extract_state(raw)
        knee = (
            _finite_float(state.get("pros_knee_angle_served_ref"))
            if state is not None
            else None
        )
        ankle = (
            _finite_float(state.get("pros_ankle_angle_served_ref"))
            if state is not None
            else None
        )
        if knee is None or ankle is None:
            missing_served_rows += 1

        transitions = _extract_transitions(raw)
        completed = _extract_completed_segments(raw)
        ledger_diagnostics = _extract_row_mapping(raw, "morphology_ledger_diagnostics")
        reward_terms = _extract_row_mapping(raw, "reward_terms")
        transition_rows += int(bool(transitions))
        payload_rows += int(bool(completed))
        ledger_diagnostic_rows += int(bool(ledger_diagnostics))
        reward_term_rows += int(bool(reward_terms))
        step_value = _finite_float(raw.get("step"))
        parsed.append(
            ParsedTraceRow(
                row_index=row_index,
                step=int(step_value) if step_value is not None else row_index,
                time_s=time_s,
                knee_rad=knee,
                ankle_rad=ankle,
                transitions=transitions,
                completed_segments=completed,
                ledger_diagnostics=ledger_diagnostics,
                reward_terms=reward_terms,
            )
        )

    diagnostics = {
        "raw_row_count": len(raw_rows),
        "parsed_row_count": len(parsed),
        "invalid_row_count": invalid_rows,
        "missing_time_row_count": missing_time_rows,
        "missing_served_reference_row_count": missing_served_rows,
        "rows_with_transition_journal": transition_rows,
        "rows_with_completed_segment_payload": payload_rows,
        "rows_with_morphology_ledger_diagnostics": ledger_diagnostic_rows,
        "rows_with_reward_terms": reward_term_rows,
    }
    return parsed, root, diagnostics


def _resolve_relative_path(raw: str, trace_path: Path) -> Path | None:
    candidate = Path(raw).expanduser()
    candidates = (
        candidate,
        trace_path.parent / candidate,
        BASELINE_DIR / candidate,
        REPO_ROOT / candidate,
    )
    for item in candidates:
        resolved = item if item.is_absolute() else item.resolve()
        if resolved.is_file():
            return resolved
    return None


def _sibling_reward_config(
    trace_path: Path,
    root: Mapping[str, Any],
) -> Mapping[str, Any]:
    for candidate in (root.get("reward_config"), root.get("resolved_reward_config")):
        if isinstance(candidate, Mapping):
            return candidate
    sibling = trace_path.parent / "rollout_summary.json"
    if sibling.is_file():
        value = _load_json(sibling)
        if isinstance(value, Mapping):
            for candidate in (
                value.get("reward_config"),
                value.get("resolved_reward_config"),
            ):
                if isinstance(candidate, Mapping):
                    return candidate
    return {}


def load_profile(path: Path) -> ProfileData:
    raw = _load_json(path)
    if not isinstance(raw, Mapping):
        raise ValueError(f"morphology profile must be a JSON object: {path}")
    if str(raw.get("units", "")).lower() not in {"rad", "radian", "radians"}:
        raise ValueError("morphology profile must use radians")

    phase = np.asarray(raw.get("phase_grid"), dtype=float)
    if phase.ndim != 1 or phase.size < 2:
        raise ValueError("profile phase_grid must be one-dimensional")
    if not np.all(np.isfinite(phase)) or not np.all(np.diff(phase) > 0.0):
        raise ValueError("profile phase_grid must be finite and strictly increasing")
    if abs(float(phase[0])) > VALUE_TOL or abs(float(phase[-1]) - 1.0) > VALUE_TOL:
        raise ValueError("profile phase_grid must span [0, 1]")

    metadata_raw = raw.get("metadata")
    metadata = dict(metadata_raw) if isinstance(metadata_raw, Mapping) else {}
    parameterization = str(metadata.get("phase_parameterization", ""))
    if parameterization != EXPECTED_PARAMETERIZATION:
        raise ValueError(
            "profile metadata.phase_parameterization must be "
            f"{EXPECTED_PARAMETERIZATION!r}; got {parameterization!r}"
        )
    alpha = _finite_float(metadata.get("canonical_to_phase"))
    if alpha is None or not 0.0 < alpha < 1.0:
        raise ValueError("profile canonical_to_phase must lie strictly inside (0, 1)")

    coordinates = raw.get("coordinates")
    if not isinstance(coordinates, Mapping):
        raise ValueError("profile coordinates must be a mapping")
    means: dict[str, np.ndarray] = {}
    stds: dict[str, np.ndarray] = {}
    for coord in ("pros_knee_angle", "pros_ankle_angle"):
        entry = coordinates.get(coord)
        if not isinstance(entry, Mapping):
            raise ValueError(f"profile is missing coordinates.{coord}")
        mean = np.asarray(entry.get("mean_rad"), dtype=float)
        std = np.asarray(entry.get("std_rad"), dtype=float)
        if mean.shape != phase.shape or std.shape != phase.shape:
            raise ValueError(f"profile arrays for {coord} must match phase_grid")
        if not np.all(np.isfinite(mean)) or not np.all(np.isfinite(std)):
            raise ValueError(f"profile arrays for {coord} must be finite")
        if np.any(std < 0.0):
            raise ValueError(f"profile std for {coord} must be non-negative")
        means[coord] = mean
        stds[coord] = std
    return ProfileData(
        path=path.resolve(),
        phase=phase,
        canonical_to_phase=float(alpha),
        mean=means,
        std=stds,
        metadata=metadata,
    )


def _transition_records(rows: Sequence[ParsedTraceRow]) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for row in rows:
        for journal_index, transition in enumerate(row.transitions):
            records.append(
                {
                    **dict(transition),
                    "_trace_row_index": row.row_index,
                    "_trace_step": row.step,
                    "_journal_index": journal_index,
                }
            )
    return records


def segments_from_journal(
    rows: Sequence[ParsedTraceRow],
) -> tuple[list[SegmentDefinition], dict[str, Any]]:
    transitions = _transition_records(rows)
    active_type = ""
    active_start: float | None = None
    segments: list[SegmentDefinition] = []
    invalid_transition_count = 0
    invalid_segment_count = 0
    duplicate_transition_count = 0
    sequence_mismatch_count = 0
    seen: set[tuple[str, float, str, float]] = set()

    for transition in transitions:
        event = str(transition.get("event", "") or "").strip().lower()
        event_time = _finite_float(transition.get("event_time_s"))
        if event_time is None or event not in {"heel_strike", "toe_off", "timeout"}:
            invalid_transition_count += 1
            continue
        declared_closed = (
            str(transition.get("closed_segment_type", "") or "").strip().lower()
        )
        declared_start = _finite_float(transition.get("segment_start_time_s"))
        fingerprint = (
            event,
            float(event_time),
            declared_closed,
            float(declared_start if declared_start is not None else -1.0),
        )
        if fingerprint in seen:
            duplicate_transition_count += 1
            continue
        seen.add(fingerprint)

        if event == "timeout":
            active_type = ""
            active_start = None
            continue

        expected_closed = "stance" if event == "toe_off" else "swing"
        default_opened = "swing" if event == "toe_off" else "stance"
        opened = (
            str(transition.get("opens_segment_type", default_opened) or default_opened)
            .strip()
            .lower()
        )
        segment_valid_value = _finite_float(transition.get("segment_valid"))
        segment_valid = segment_valid_value is None or segment_valid_value > 0.5

        closes_segment = declared_closed in {"stance", "swing"}
        if not closes_segment and active_type == expected_closed:
            closes_segment = True
            declared_closed = expected_closed
        if closes_segment:
            start = declared_start
            if start is None or start < 0.0:
                start = active_start
            geometry_valid = (
                start is not None
                and event_time > float(start) + TIME_EPS
                and declared_closed == expected_closed
            )
            sequence_valid = not active_type or active_type == expected_closed
            if not sequence_valid:
                sequence_mismatch_count += 1
            if segment_valid and geometry_valid:
                segments.append(
                    SegmentDefinition(
                        segment_id=len(segments),
                        segment_type=expected_closed,
                        start_time_s=float(start),
                        end_time_s=float(event_time),
                        source="accepted_transition_journal",
                    )
                )
            else:
                invalid_segment_count += 1

        if opened in {"stance", "swing"}:
            active_type = opened
            active_start = float(event_time)
        else:
            active_type = ""
            active_start = None

    diagnostics = {
        "transition_count": len(transitions),
        "unique_transition_count": len(seen),
        "invalid_transition_count": invalid_transition_count,
        "duplicate_transition_count": duplicate_transition_count,
        "invalid_or_rejected_closed_segment_count": invalid_segment_count,
        "sequence_mismatch_count": sequence_mismatch_count,
        "valid_completed_segment_count": len(segments),
        "incomplete_active_segment_type_at_trace_end": active_type,
        "incomplete_active_segment_start_time_s": active_start,
    }
    return segments, diagnostics


def payload_segments(
    rows: Sequence[ParsedTraceRow],
) -> tuple[list[SegmentDefinition], list[dict[str, Any]], dict[str, Any]]:
    definitions: list[SegmentDefinition] = []
    payloads: list[dict[str, Any]] = []
    invalid_count = 0
    duplicate_count = 0
    seen: set[tuple[str, float, float]] = set()
    for row in rows:
        for item in row.completed_segments:
            segment_type = str(item.get("segment_type", "") or "").strip().lower()
            start = _finite_float(item.get("start_time_s"))
            end = _finite_float(item.get("end_time_s"))
            if (
                segment_type not in {"stance", "swing"}
                or start is None
                or end is None
                or end <= start + TIME_EPS
            ):
                invalid_count += 1
                continue
            fingerprint = (segment_type, float(start), float(end))
            if fingerprint in seen:
                duplicate_count += 1
                continue
            seen.add(fingerprint)
            definitions.append(
                SegmentDefinition(
                    segment_id=len(definitions),
                    segment_type=segment_type,
                    start_time_s=float(start),
                    end_time_s=float(end),
                    source="morphology_completed_segments",
                )
            )
            payloads.append(
                {
                    **dict(item),
                    "_trace_row_index": row.row_index,
                    "_trace_step": row.step,
                }
            )
    return (
        definitions,
        payloads,
        {
            "payload_completed_segment_count": len(definitions),
            "invalid_payload_segment_count": invalid_count,
            "duplicate_payload_segment_count": duplicate_count,
        },
    )


def _segment_key(segment: SegmentDefinition) -> tuple[str, int, int]:
    scale = 1.0 / MATCH_TIME_TOL_S
    return (
        segment.segment_type,
        int(round(segment.start_time_s * scale)),
        int(round(segment.end_time_s * scale)),
    )


def _match_segment_definitions(
    journal: Sequence[SegmentDefinition],
    payload: Sequence[SegmentDefinition],
) -> dict[str, Any]:
    journal_keys = {_segment_key(segment) for segment in journal}
    payload_keys = {_segment_key(segment) for segment in payload}
    return {
        "available": bool(journal and payload),
        "journal_only_count": len(journal_keys - payload_keys),
        "payload_only_count": len(payload_keys - journal_keys),
        "exact_match": bool(journal and payload and journal_keys == payload_keys),
    }


def _segments_overlap(segments: Sequence[SegmentDefinition]) -> tuple[int, float]:
    ordered = sorted(segments, key=lambda item: (item.start_time_s, item.end_time_s))
    overlaps = 0
    max_overlap = 0.0
    for left, right in zip(ordered, ordered[1:]):
        overlap = left.end_time_s - right.start_time_s
        if overlap > MATCH_TIME_TOL_S:
            overlaps += 1
            max_overlap = max(max_overlap, overlap)
    return overlaps, max_overlap


def _phase_for_time(
    segment: SegmentDefinition,
    time_s: float,
    alpha: float,
) -> float:
    progress = (float(time_s) - segment.start_time_s) / segment.duration_s
    if progress < -VALUE_TOL or progress > 1.0 + VALUE_TOL:
        raise ValueError("sample timestamp lies outside completed segment")
    progress = float(np.clip(progress, 0.0, 1.0))
    if segment.segment_type == "stance":
        return float(alpha * progress)
    return float(alpha + (1.0 - alpha) * progress)


def _corridor_at(
    profile: ProfileData,
    phase: float,
    *,
    knee_multiplier: float,
    ankle_multiplier: float,
    knee_margin_deg: float,
    ankle_margin_deg: float,
) -> dict[str, dict[str, float]]:
    result: dict[str, dict[str, float]] = {}
    for coord, multiplier, margin_deg in (
        ("pros_knee_angle", knee_multiplier, knee_margin_deg),
        ("pros_ankle_angle", ankle_multiplier, ankle_margin_deg),
    ):
        mean = float(np.interp(phase, profile.phase, profile.mean[coord]))
        std = float(np.interp(phase, profile.phase, profile.std[coord]))
        margin = float(np.deg2rad(margin_deg))
        low = mean - float(multiplier) * std - margin
        high = mean + float(multiplier) * std + margin
        result[coord] = {
            "mean_rad": mean,
            "std_rad": std,
            "min_rad": float(min(low, high)),
            "max_rad": float(max(low, high)),
        }
    return result


def _interval_loss(value: float, low: float, high: float) -> dict[str, float]:
    low_f = float(min(low, high))
    high_f = float(max(low, high))
    if value < low_f:
        excursion = low_f - value
    elif value > high_f:
        excursion = value - high_f
    else:
        excursion = 0.0
    width = max(1.0e-9, high_f - low_f)
    return {
        "loss": float(min(25.0, (excursion / width) ** 2)),
        "raw_loss_rad2": float(excursion * excursion),
        "excursion_rad": float(excursion),
        "inside": float(excursion == 0.0),
    }


def _payload_key(item: Mapping[str, Any]) -> tuple[str, int, int] | None:
    segment_type = str(item.get("segment_type", "") or "").strip().lower()
    start = _finite_float(item.get("start_time_s"))
    end = _finite_float(item.get("end_time_s"))
    if segment_type not in {"stance", "swing"} or start is None or end is None:
        return None
    scale = 1.0 / MATCH_TIME_TOL_S
    return segment_type, int(round(start * scale)), int(round(end * scale))


def _max_error(current: float | None, candidate: float | None) -> float | None:
    if candidate is None:
        return current
    if current is None:
        return abs(float(candidate))
    return max(float(current), abs(float(candidate)))


def audit(
    *,
    rows: Sequence[ParsedTraceRow],
    profile: ProfileData,
    journal_segments: Sequence[SegmentDefinition],
    payload_definitions: Sequence[SegmentDefinition],
    payloads: Sequence[Mapping[str, Any]],
    knee_multiplier: float,
    ankle_multiplier: float,
    knee_margin_deg: float,
    ankle_margin_deg: float,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    if journal_segments:
        segments = list(journal_segments)
        evidence_source = "accepted_transition_journal"
    else:
        segments = list(payload_definitions)
        evidence_source = "morphology_completed_segments"
    payload_by_key = {
        key: item for item in payloads if (key := _payload_key(item)) is not None
    }

    overlap_count, max_overlap_s = _segments_overlap(segments)
    assigned_counts: dict[int, int] = {}
    eligible_timed_rows: set[int] = set()
    missing_served_eligible: set[int] = set()
    computed_segments: list[dict[str, Any]] = []
    phase_plateau_count = 0
    phase_nonmonotonic_count = 0
    min_phase_increment: float | None = None
    phase_interval_violation_count = 0
    invalid_geometry_count = 0
    knee_loss_sum = 0.0
    ankle_loss_sum = 0.0
    knee_inside_count = 0
    ankle_inside_count = 0
    total_sample_count = 0

    payload_segment_missing_count = 0
    payload_sample_count_mismatch = 0
    payload_time_max_error_s: float | None = None
    payload_served_max_error_rad: float | None = None
    payload_phase_max_error: float | None = None
    payload_bound_max_error_rad: float | None = None
    payload_loss_max_error: float | None = None
    payload_missing_required_sample_field_count = 0

    for segment in sorted(
        segments, key=lambda item: (item.start_time_s, item.end_time_s)
    ):
        if segment.duration_s <= TIME_EPS:
            invalid_geometry_count += 1
            continue
        timed = [
            row
            for row in rows
            if row.time_s is not None
            and row.time_s >= segment.start_time_s - TIME_EPS
            and row.time_s < segment.end_time_s - TIME_EPS
        ]
        for row in timed:
            eligible_timed_rows.add(row.row_index)
            if row.knee_rad is None or row.ankle_rad is None:
                missing_served_eligible.add(row.row_index)

        valid_rows = [
            row
            for row in timed
            if row.knee_rad is not None and row.ankle_rad is not None
        ]
        samples: list[dict[str, Any]] = []
        for row in valid_rows:
            phase = _phase_for_time(
                segment, float(row.time_s), profile.canonical_to_phase
            )
            if segment.segment_type == "stance":
                valid_phase = (
                    -VALUE_TOL <= phase < profile.canonical_to_phase + VALUE_TOL
                )
            else:
                valid_phase = (
                    profile.canonical_to_phase - VALUE_TOL <= phase < 1.0 + VALUE_TOL
                )
            phase_interval_violation_count += int(not valid_phase)
            corridor = _corridor_at(
                profile,
                phase,
                knee_multiplier=knee_multiplier,
                ankle_multiplier=ankle_multiplier,
                knee_margin_deg=knee_margin_deg,
                ankle_margin_deg=ankle_margin_deg,
            )
            knee_corridor = corridor["pros_knee_angle"]
            ankle_corridor = corridor["pros_ankle_angle"]
            knee = _interval_loss(
                float(row.knee_rad),
                knee_corridor["min_rad"],
                knee_corridor["max_rad"],
            )
            ankle = _interval_loss(
                float(row.ankle_rad),
                ankle_corridor["min_rad"],
                ankle_corridor["max_rad"],
            )
            assigned_counts[row.row_index] = assigned_counts.get(row.row_index, 0) + 1
            knee_loss_sum += knee["loss"]
            ankle_loss_sum += ankle["loss"]
            knee_inside_count += int(knee["inside"] > 0.5)
            ankle_inside_count += int(ankle["inside"] > 0.5)
            total_sample_count += 1
            samples.append(
                {
                    "trace_row_index": row.row_index,
                    "step": row.step,
                    "time_s": float(row.time_s),
                    "phase": phase,
                    "knee_served_ref_rad": float(row.knee_rad),
                    "ankle_served_ref_rad": float(row.ankle_rad),
                    "knee_mean_rad": knee_corridor["mean_rad"],
                    "knee_min_rad": knee_corridor["min_rad"],
                    "knee_max_rad": knee_corridor["max_rad"],
                    "ankle_mean_rad": ankle_corridor["mean_rad"],
                    "ankle_min_rad": ankle_corridor["min_rad"],
                    "ankle_max_rad": ankle_corridor["max_rad"],
                    "knee_loss": knee["loss"],
                    "ankle_loss": ankle["loss"],
                    "knee_excursion_rad": knee["excursion_rad"],
                    "ankle_excursion_rad": ankle["excursion_rad"],
                }
            )

        phases = np.asarray([sample["phase"] for sample in samples], dtype=float)
        if phases.size > 1:
            differences = np.diff(phases)
            phase_plateau_count += int(np.sum(np.abs(differences) <= VALUE_TOL))
            phase_nonmonotonic_count += int(np.sum(differences < -VALUE_TOL))
            positive = differences[differences > VALUE_TOL]
            if positive.size:
                candidate = float(np.min(positive))
                min_phase_increment = (
                    candidate
                    if min_phase_increment is None
                    else min(min_phase_increment, candidate)
                )

        payload = payload_by_key.get(_segment_key(segment))
        if payloads and payload is None:
            payload_segment_missing_count += 1
        if payload is not None:
            logged_raw = payload.get("samples")
            logged = (
                [item for item in logged_raw if isinstance(item, Mapping)]
                if isinstance(logged_raw, (list, tuple))
                else []
            )
            logged.sort(key=lambda item: float(item.get("time_s", float("inf"))))
            expected = sorted(samples, key=lambda item: item["time_s"])
            declared_sample_count = _finite_float(payload.get("sample_count"))
            if len(logged) != len(expected) or (
                declared_sample_count is not None
                and int(declared_sample_count) != len(logged)
            ):
                payload_sample_count_mismatch += 1
            for independent, runtime in zip(expected, logged):
                required_fields = (
                    "time_s",
                    "phase",
                    "knee_served_ref_rad",
                    "ankle_served_ref_rad",
                    "knee_min_rad",
                    "knee_max_rad",
                    "ankle_min_rad",
                    "ankle_max_rad",
                    "knee_loss",
                    "ankle_loss",
                )
                payload_missing_required_sample_field_count += sum(
                    _finite_float(runtime.get(key)) is None for key in required_fields
                )
                runtime_time = _finite_float(runtime.get("time_s"))
                runtime_knee = _finite_float(runtime.get("knee_served_ref_rad"))
                runtime_ankle = _finite_float(runtime.get("ankle_served_ref_rad"))
                runtime_phase = _finite_float(runtime.get("phase"))
                if runtime_time is not None:
                    payload_time_max_error_s = _max_error(
                        payload_time_max_error_s,
                        runtime_time - independent["time_s"],
                    )
                for observed, expected_value in (
                    (runtime_knee, independent["knee_served_ref_rad"]),
                    (runtime_ankle, independent["ankle_served_ref_rad"]),
                ):
                    if observed is not None:
                        payload_served_max_error_rad = _max_error(
                            payload_served_max_error_rad,
                            observed - expected_value,
                        )
                if runtime_phase is not None:
                    payload_phase_max_error = _max_error(
                        payload_phase_max_error,
                        runtime_phase - independent["phase"],
                    )
                for key in (
                    "knee_min_rad",
                    "knee_max_rad",
                    "ankle_min_rad",
                    "ankle_max_rad",
                ):
                    observed = _finite_float(runtime.get(key))
                    if observed is not None:
                        payload_bound_max_error_rad = _max_error(
                            payload_bound_max_error_rad,
                            observed - independent[key],
                        )
                for key in ("knee_loss", "ankle_loss"):
                    observed = _finite_float(runtime.get(key))
                    if observed is not None:
                        payload_loss_max_error = _max_error(
                            payload_loss_max_error,
                            observed - independent[key],
                        )

        computed_segments.append(
            {
                "segment_id": segment.segment_id,
                "segment_type": segment.segment_type,
                "start_time_s": segment.start_time_s,
                "end_time_s": segment.end_time_s,
                "duration_s": segment.duration_s,
                "sample_count": len(samples),
                "phase_first": float(phases[0]) if phases.size else None,
                "phase_last": float(phases[-1]) if phases.size else None,
                "phase_min_increment": (
                    float(np.min(np.diff(phases))) if phases.size > 1 else None
                ),
                "samples": samples,
            }
        )

    duplicate_assigned_rows = {
        str(index): count for index, count in assigned_counts.items() if count != 1
    }
    assigned_rows = set(assigned_counts)
    eligible_with_values = eligible_timed_rows - missing_served_eligible
    lost_rows = sorted(eligible_with_values - assigned_rows)
    extra_rows = sorted(assigned_rows - eligible_with_values)

    metadata_conventions = profile.metadata.get("coordinate_conventions")
    metadata_conventions = (
        metadata_conventions if isinstance(metadata_conventions, Mapping) else {}
    )
    ankle_convention = metadata_conventions.get("pros_ankle_angle")
    ankle_convention = ankle_convention if isinstance(ankle_convention, Mapping) else {}
    knee_convention = metadata_conventions.get("pros_knee_angle")
    knee_convention = knee_convention if isinstance(knee_convention, Mapping) else {}
    sign_metadata_pass = (
        "opensim" in str(ankle_convention.get("runtime_sign", "")).lower()
        and "opensim" in str(knee_convention.get("runtime_sign", "")).lower()
        and "identity" in str(ankle_convention.get("display_convention", "")).lower()
    )
    payload_crosscheck_available = bool(payloads)
    payload_crosscheck_pass = not payload_crosscheck_available or (
        payload_segment_missing_count == 0
        and payload_sample_count_mismatch == 0
        and payload_missing_required_sample_field_count == 0
        and (payload_time_max_error_s or 0.0) <= MATCH_TIME_TOL_S
        and (payload_served_max_error_rad or 0.0) <= VALUE_TOL
        and (payload_phase_max_error or 0.0) <= VALUE_TOL
        and (payload_bound_max_error_rad or 0.0) <= VALUE_TOL
        and (payload_loss_max_error or 0.0) <= VALUE_TOL
    )

    checks = {
        "completed_segments_available": len(segments) > 0,
        "completed_segments_contain_samples": total_sample_count > 0,
        "segment_geometry_valid": invalid_geometry_count == 0,
        "completed_segments_do_not_overlap": overlap_count == 0,
        "no_sample_duplicated_between_completed_segments": not duplicate_assigned_rows,
        "no_finite_served_sample_lost_within_completed_segments": not lost_rows,
        "no_extra_sample_assigned": not extra_rows,
        "all_completed_segment_rows_have_served_reference": not missing_served_eligible,
        "phase_strictly_monotonic": phase_nonmonotonic_count == 0,
        "phase_has_no_plateau": phase_plateau_count == 0,
        "phase_remains_in_segment_interval": phase_interval_violation_count == 0,
        "analytical_event_anchors_exact": True,
        "served_and_corridor_use_identity_runtime_sign": sign_metadata_pass,
        "runtime_completed_payload_matches_independent_recomputation": (
            payload_crosscheck_pass
        ),
    }
    summary = {
        "evidence_source": evidence_source,
        "checks": checks,
        "pass": all(checks.values()),
        "segment_count": len(segments),
        "stance_segment_count": sum(s.segment_type == "stance" for s in segments),
        "swing_segment_count": sum(s.segment_type == "swing" for s in segments),
        "segment_overlap_count": overlap_count,
        "max_segment_overlap_s": max_overlap_s,
        "invalid_segment_geometry_count": invalid_geometry_count,
        "sample_ownership": {
            "eligible_timed_trace_row_count": len(eligible_timed_rows),
            "eligible_finite_served_trace_row_count": len(eligible_with_values),
            "assigned_trace_row_count": len(assigned_rows),
            "assigned_sample_count": total_sample_count,
            "missing_served_reference_row_indices": sorted(missing_served_eligible),
            "duplicated_assignment_counts_by_row": duplicate_assigned_rows,
            "lost_finite_served_row_indices": lost_rows,
            "extra_assigned_row_indices": extra_rows,
        },
        "phase_mapping": {
            "canonical_to_phase": profile.canonical_to_phase,
            "plateau_count": phase_plateau_count,
            "nonmonotonic_increment_count": phase_nonmonotonic_count,
            "phase_interval_violation_count": phase_interval_violation_count,
            "minimum_positive_phase_increment": min_phase_increment,
            "analytical_anchors": {
                "stance_start": 0.0,
                "stance_end": profile.canonical_to_phase,
                "swing_start": profile.canonical_to_phase,
                "swing_end": 1.0,
            },
        },
        "sign_identity": {
            "runtime_transform_applied_to_served": {
                "pros_knee_angle": 1.0,
                "pros_ankle_angle": 1.0,
            },
            "runtime_transform_applied_to_corridor": {
                "pros_knee_angle": 1.0,
                "pros_ankle_angle": 1.0,
            },
            "profile_coordinate_conventions": {
                "pros_knee_angle": dict(knee_convention),
                "pros_ankle_angle": dict(ankle_convention),
            },
            "metadata_contract_pass": sign_metadata_pass,
            "logged_served_reference_max_abs_error_rad": payload_served_max_error_rad,
        },
        "loss": {
            "sample_count": total_sample_count,
            "knee_loss_sum": knee_loss_sum,
            "ankle_loss_sum": ankle_loss_sum,
            "morphology_loss_sum": 0.5 * (knee_loss_sum + ankle_loss_sum),
            "morphology_loss_mean": (
                0.5 * (knee_loss_sum + ankle_loss_sum) / total_sample_count
                if total_sample_count
                else None
            ),
            "knee_inside_fraction": (
                knee_inside_count / total_sample_count if total_sample_count else None
            ),
            "ankle_inside_fraction": (
                ankle_inside_count / total_sample_count if total_sample_count else None
            ),
        },
        "runtime_payload_crosscheck": {
            "available": payload_crosscheck_available,
            "pass": payload_crosscheck_pass,
            "missing_segment_count": payload_segment_missing_count,
            "sample_count_mismatch_segment_count": payload_sample_count_mismatch,
            "missing_required_sample_field_count": (
                payload_missing_required_sample_field_count
            ),
            "time_max_abs_error_s": payload_time_max_error_s,
            "served_reference_max_abs_error_rad": payload_served_max_error_rad,
            "phase_max_abs_error": payload_phase_max_error,
            "corridor_bound_max_abs_error_rad": payload_bound_max_error_rad,
            "loss_max_abs_error": payload_loss_max_error,
        },
    }
    return summary, computed_segments


EXPECTED_BOUNDARY_DISCARD_REASONS = {
    "before_first_anchor",
    "episode_end_incomplete_segment",
    "episode_end_before_first_hs",
}
FATAL_LEDGER_INTEGRITY_REASONS = {
    "buffer_overflow",
    "nonmonotonic_sample",
    "transition_start_mismatch",
    "invalid_transition_time",
    "invalid_segment_geometry",
    "missing_discard_reason_for_nonzero_discard",
}


def _nonnegative_count(value: Any) -> tuple[int, bool]:
    parsed = _finite_float(value)
    if parsed is None or parsed < 0.0:
        return 0, False
    rounded = int(round(parsed))
    return rounded, abs(parsed - rounded) <= VALUE_TOL


def coverage_and_integrity(
    *,
    rows: Sequence[ParsedTraceRow],
    payloads: Sequence[Mapping[str, Any]],
    independent_settled_sample_count: int,
    independent_completed_segment_count: int,
) -> dict[str, Any]:
    """Aggregate sparse ledger settlements and classify every discarded sample."""
    payload_settled_samples = 0
    payload_sample_schema_valid = True
    for payload in payloads:
        samples = payload.get("samples")
        if isinstance(samples, (list, tuple)):
            payload_settled_samples += sum(
                isinstance(item, Mapping) for item in samples
            )
        else:
            payload_sample_schema_valid = False

    term_sample_key_seen = False
    term_segment_key_seen = False
    term_settled_samples = 0
    term_completed_segments = 0
    term_counts_valid = True
    for row in rows:
        if "morphology_settled_sample_count" in row.reward_terms:
            term_sample_key_seen = True
            count, valid = _nonnegative_count(
                row.reward_terms.get("morphology_settled_sample_count")
            )
            term_settled_samples += count
            term_counts_valid = term_counts_valid and valid
        if "morphology_settled_this_step" in row.reward_terms:
            term_segment_key_seen = True
            count, valid = _nonnegative_count(
                row.reward_terms.get("morphology_settled_this_step")
            )
            term_completed_segments += count
            term_counts_valid = term_counts_valid and valid

    diagnostic_rows = [row for row in rows if row.ledger_diagnostics]
    discarded_samples_total = 0
    discarded_segments_total = 0
    discard_counts_valid = True
    discard_events_by_exact_reason: dict[str, int] = {}
    discarded_samples_by_exact_reason: dict[str, int] = {}
    discarded_segments_by_exact_reason: dict[str, int] = {}
    reason_token_occurrences: dict[str, int] = {}
    expected_boundary_samples = 0
    expected_boundary_segments = 0
    other_task_samples = 0
    other_task_segments = 0
    fatal_samples = 0
    fatal_segments = 0
    fatal_occurrences: dict[str, int] = {}
    overflow_flag_count = 0
    nonmonotonic_flag_count = 0
    final_pending_sample_count = 0
    maximum_pending_sample_count = 0
    diagnostics_completed_segments = 0
    diagnostics_completed_count_valid = True

    for row in diagnostic_rows:
        diagnostics = row.ledger_diagnostics
        sample_count, samples_valid = _nonnegative_count(
            diagnostics.get("discarded_sample_count", 0)
        )
        segment_count, segments_valid = _nonnegative_count(
            diagnostics.get("discarded_segment_count", 0)
        )
        completed_count, completed_valid = _nonnegative_count(
            diagnostics.get("completed_segment_count", 0)
        )
        pending_count, pending_valid = _nonnegative_count(
            diagnostics.get("pending_sample_count", 0)
        )
        discard_counts_valid = (
            discard_counts_valid and samples_valid and segments_valid and pending_valid
        )
        diagnostics_completed_count_valid = (
            diagnostics_completed_count_valid and completed_valid
        )
        discarded_samples_total += sample_count
        discarded_segments_total += segment_count
        diagnostics_completed_segments += completed_count
        final_pending_sample_count = pending_count
        maximum_pending_sample_count = max(maximum_pending_sample_count, pending_count)

        reason_text = str(diagnostics.get("discard_reason", "") or "").strip()
        tokens = [token for token in reason_text.split("|") if token]
        overflowed = bool(diagnostics.get("overflowed", False))
        nonmonotonic = bool(diagnostics.get("nonmonotonic_sample", False))
        overflow_flag_count += int(overflowed)
        nonmonotonic_flag_count += int(nonmonotonic)
        if overflowed and "buffer_overflow" not in tokens:
            tokens.append("buffer_overflow")
        if nonmonotonic and "nonmonotonic_sample" not in tokens:
            tokens.append("nonmonotonic_sample")
        if (sample_count > 0 or segment_count > 0) and not tokens:
            tokens.append("missing_discard_reason_for_nonzero_discard")
            reason_text = "missing_discard_reason_for_nonzero_discard"
        exact_reason = reason_text or ("|".join(tokens) if tokens else "none")

        if tokens:
            discard_events_by_exact_reason[exact_reason] = (
                discard_events_by_exact_reason.get(exact_reason, 0) + 1
            )
            discarded_samples_by_exact_reason[exact_reason] = (
                discarded_samples_by_exact_reason.get(exact_reason, 0) + sample_count
            )
            discarded_segments_by_exact_reason[exact_reason] = (
                discarded_segments_by_exact_reason.get(exact_reason, 0) + segment_count
            )
        for token in dict.fromkeys(tokens):
            reason_token_occurrences[token] = reason_token_occurrences.get(token, 0) + 1

        fatal_tokens = [
            token for token in tokens if token in FATAL_LEDGER_INTEGRITY_REASONS
        ]
        if fatal_tokens:
            fatal_samples += sample_count
            fatal_segments += segment_count
            for token in dict.fromkeys(fatal_tokens):
                fatal_occurrences[token] = fatal_occurrences.get(token, 0) + 1
        elif tokens and all(
            token in EXPECTED_BOUNDARY_DISCARD_REASONS for token in tokens
        ):
            expected_boundary_samples += sample_count
            expected_boundary_segments += segment_count
        else:
            other_task_samples += sample_count
            other_task_segments += segment_count

    payload_available = bool(payloads)
    terms_available = term_sample_key_seen
    diagnostics_available = bool(diagnostic_rows)
    if payload_available:
        selected_settled_samples = payload_settled_samples
        settled_source = "morphology_completed_segments"
    elif terms_available:
        selected_settled_samples = term_settled_samples
        settled_source = "reward_terms.morphology_settled_sample_count"
    else:
        selected_settled_samples = int(independent_settled_sample_count)
        settled_source = "independent_completed_segment_partition_fallback"

    settled_source_checks: dict[str, bool] = {
        "payload_sample_schema_valid": payload_sample_schema_valid,
        "reward_term_counts_valid": term_counts_valid,
        "payload_matches_independent_partition": (
            not payload_available
            or payload_settled_samples == independent_settled_sample_count
        ),
        "reward_terms_match_independent_partition": (
            not terms_available
            or term_settled_samples == independent_settled_sample_count
        ),
        "payload_matches_reward_terms": (
            not (payload_available and terms_available)
            or payload_settled_samples == term_settled_samples
        ),
        "diagnostic_completed_segments_match_independent": (
            not diagnostics_available
            or diagnostics_completed_segments == independent_completed_segment_count
        ),
        "reward_term_completed_segments_match_independent": (
            not term_segment_key_seen
            or term_completed_segments == independent_completed_segment_count
        ),
    }
    settled_sources_consistent = all(settled_source_checks.values())
    denominator = selected_settled_samples + discarded_samples_total
    coverage = selected_settled_samples / denominator if denominator > 0 else None
    adjusted_denominator = selected_settled_samples + (
        discarded_samples_total - expected_boundary_samples
    )
    boundary_adjusted_coverage = (
        selected_settled_samples / adjusted_denominator
        if adjusted_denominator > 0
        else None
    )
    fatal_integrity_pass = not fatal_occurrences

    return {
        "available": diagnostics_available or payload_available or terms_available,
        "ledger_diagnostics_available": diagnostics_available,
        "ledger_diagnostic_row_count": len(diagnostic_rows),
        "settled_sample_count": selected_settled_samples,
        "settled_sample_source": settled_source,
        "settled_sample_sources": {
            "completed_payload_available": payload_available,
            "completed_payload_sample_count": payload_settled_samples,
            "reward_terms_available": terms_available,
            "reward_terms_sample_count": term_settled_samples,
            "independent_partition_sample_count": independent_settled_sample_count,
            "checks": settled_source_checks,
            "consistent": settled_sources_consistent,
        },
        "completed_segment_sources": {
            "completed_payload_segment_count": len(payloads),
            "reward_terms_segment_count": term_completed_segments,
            "ledger_diagnostics_segment_count": diagnostics_completed_segments,
            "independent_segment_count": independent_completed_segment_count,
            "diagnostic_counts_valid": diagnostics_completed_count_valid,
        },
        "discarded_sample_count": discarded_samples_total,
        "discarded_segment_count": discarded_segments_total,
        "discard_events_by_exact_reason": dict(
            sorted(discard_events_by_exact_reason.items())
        ),
        "discarded_samples_by_exact_reason": dict(
            sorted(discarded_samples_by_exact_reason.items())
        ),
        "discarded_segments_by_exact_reason": dict(
            sorted(discarded_segments_by_exact_reason.items())
        ),
        "discard_reason_token_occurrences": dict(
            sorted(reason_token_occurrences.items())
        ),
        "classification": {
            "expected_bootstrap_tail_reasons": sorted(
                EXPECTED_BOUNDARY_DISCARD_REASONS
            ),
            "fatal_integrity_reasons": sorted(FATAL_LEDGER_INTEGRITY_REASONS),
            "expected_bootstrap_tail_discarded_samples": (expected_boundary_samples),
            "expected_bootstrap_tail_discarded_segments": (expected_boundary_segments),
            "other_task_discarded_samples": other_task_samples,
            "other_task_discarded_segments": other_task_segments,
            "fatal_integrity_discarded_samples": fatal_samples,
            "fatal_integrity_discarded_segments": fatal_segments,
            "fatal_integrity_occurrences": dict(sorted(fatal_occurrences.items())),
            "overflow_flag_count": overflow_flag_count,
            "nonmonotonic_flag_count": nonmonotonic_flag_count,
        },
        "pending_samples": {
            "maximum": maximum_pending_sample_count,
            "final": final_pending_sample_count,
        },
        "settled_over_settled_plus_discarded": coverage,
        "boundary_adjusted_settled_coverage": boundary_adjusted_coverage,
        "discard_counts_valid": discard_counts_valid,
        "settled_sources_consistent": settled_sources_consistent,
        "fatal_integrity_pass": fatal_integrity_pass,
        "structural_pass": bool(
            discard_counts_valid
            and diagnostics_completed_count_valid
            and settled_sources_consistent
            and fatal_integrity_pass
        ),
        "policy": {
            "expected_bootstrap_tail_discards_fail_structural_gate": False,
            "fatal_integrity_discards_fail_structural_gate": True,
        },
    }


def plot_time_overlay(
    segments: Sequence[Mapping[str, Any]],
    *,
    rows: Sequence[ParsedTraceRow],
    profile: ProfileData,
    knee_multiplier: float,
    ankle_multiplier: float,
    knee_margin_deg: float,
    ankle_margin_deg: float,
    output_path: Path,
    title_suffix: str,
) -> None:
    figure, axes = plt.subplots(2, 1, figsize=(15, 9), sharex=True)
    specs = (
        (
            axes[0],
            "knee",
            "Prosthetic knee — raw OpenSim sign",
        ),
        (
            axes[1],
            "ankle",
            "Prosthetic ankle — raw OpenSim sign (positive = dorsiflexion)",
        ),
    )
    type_colours = {"stance": "#4c78a8", "swing": "#f58518"}
    event_times: dict[tuple[str, float], bool] = {}

    finite_time_rows = sorted(
        (row for row in rows if row.time_s is not None),
        key=lambda row: (float(row.time_s), row.row_index),
    )
    trace_times = [float(row.time_s) for row in finite_time_rows]
    trace_start = min(trace_times) if trace_times else None
    trace_end = max(trace_times) if trace_times else None

    covered_intervals = sorted(
        (
            float(segment["start_time_s"]),
            float(segment["end_time_s"]),
        )
        for segment in segments
        if _finite_float(segment.get("start_time_s")) is not None
        and _finite_float(segment.get("end_time_s")) is not None
        and float(segment["end_time_s"]) > float(segment["start_time_s"])
    )
    merged_intervals: list[tuple[float, float]] = []
    for start, end in covered_intervals:
        if not merged_intervals or start > merged_intervals[-1][1] + TIME_EPS:
            merged_intervals.append((start, end))
        else:
            previous_start, previous_end = merged_intervals[-1]
            merged_intervals[-1] = (previous_start, max(previous_end, end))

    unscored_intervals: list[tuple[float, float]] = []
    if trace_start is not None and trace_end is not None and trace_end > trace_start:
        cursor = trace_start
        for start, end in merged_intervals:
            clipped_start = max(trace_start, start)
            clipped_end = min(trace_end, end)
            if clipped_start > cursor + TIME_EPS:
                unscored_intervals.append((cursor, clipped_start))
            cursor = max(cursor, clipped_end)
        if cursor < trace_end - TIME_EPS:
            unscored_intervals.append((cursor, trace_end))

    for axis, _short, panel_title in specs:
        first_band = True
        first_mean = True
        first_unscored = True
        for gap_start, gap_end in unscored_intervals:
            axis.axvspan(
                gap_start,
                gap_end,
                facecolor="#b0b0b0",
                edgecolor="#8a8a8a",
                hatch="///",
                linewidth=0.0,
                alpha=0.13,
                label=(
                    "No exact completed-segment corridor" if first_unscored else None
                ),
                zorder=0,
            )
            first_unscored = False
        for segment in segments:
            samples_raw = segment.get("samples")
            samples = (
                [item for item in samples_raw if isinstance(item, Mapping)]
                if isinstance(samples_raw, (list, tuple))
                else []
            )
            if not samples:
                continue
            segment_type = str(segment.get("segment_type"))
            start = float(segment["start_time_s"])
            end = float(segment["end_time_s"])
            definition = SegmentDefinition(
                segment_id=int(segment.get("segment_id", 0)),
                segment_type=segment_type,
                start_time_s=start,
                end_time_s=end,
                source="plot",
            )
            dense_time = np.linspace(
                start,
                end,
                max(101, 4 * len(samples) + 1),
                dtype=float,
            )
            dense_corridors = [
                _corridor_at(
                    profile,
                    _phase_for_time(
                        definition,
                        float(sample_time),
                        profile.canonical_to_phase,
                    ),
                    knee_multiplier=knee_multiplier,
                    ankle_multiplier=ankle_multiplier,
                    knee_margin_deg=knee_margin_deg,
                    ankle_margin_deg=ankle_margin_deg,
                )
                for sample_time in dense_time
            ]
            coord = "pros_knee_angle" if _short == "knee" else "pros_ankle_angle"
            mean = np.rad2deg(
                [float(item[coord]["mean_rad"]) for item in dense_corridors]
            )
            low = np.rad2deg(
                [float(item[coord]["min_rad"]) for item in dense_corridors]
            )
            high = np.rad2deg(
                [float(item[coord]["max_rad"]) for item in dense_corridors]
            )
            colour = type_colours.get(str(segment.get("segment_type")), "#777777")
            axis.fill_between(
                dense_time,
                low,
                high,
                color=colour,
                alpha=0.20,
                label="Exact event-warped corridor" if first_band else None,
                zorder=1,
            )
            axis.plot(
                dense_time,
                mean,
                color=colour,
                linewidth=1.2,
                linestyle="--",
                alpha=0.85,
                label="Corridor mean" if first_mean else None,
                zorder=2,
            )
            first_band = False
            first_mean = False
            start_event = "HS" if segment_type == "stance" else "TO"
            end_event = "TO" if segment_type == "stance" else "HS"
            event_times[(start_event, start)] = True
            event_times[(end_event, end)] = True

        joint_attr = "knee_rad" if _short == "knee" else "ankle_rad"
        served_rows = [
            row for row in finite_time_rows if getattr(row, joint_attr) is not None
        ]
        if served_rows:
            served_time = np.asarray(
                [float(row.time_s) for row in served_rows],
                dtype=float,
            )
            served_value = np.rad2deg(
                [float(getattr(row, joint_attr)) for row in served_rows]
            )
            axis.plot(
                served_time,
                served_value,
                color="#111111",
                linewidth=1.8,
                label="Policy served reference — complete trace",
                zorder=4,
            )
        axis.set_title(panel_title)
        axis.set_ylabel("Angle [deg]")
        axis.grid(True, alpha=0.25)

    for axis in axes:
        labelled: set[str] = set()
        for event_name, time_s in sorted(event_times):
            colour = "#2ca02c" if event_name == "HS" else "#d62728"
            label = event_name if event_name not in labelled else None
            axis.axvline(time_s, color=colour, linewidth=0.9, alpha=0.55, label=label)
            labelled.add(event_name)
        handles, labels = axis.get_legend_handles_labels()
        unique: dict[str, Any] = {}
        for handle, label in zip(handles, labels):
            if label and label not in unique:
                unique[label] = handle
        axis.legend(unique.values(), unique.keys(), loc="best", fontsize=9)

    axes[-1].set_xlabel("Simulation time [s]")
    figure.suptitle(
        "Experimental retrospective morphology corridor\n"
        "Exact accepted-FSM HS–TO–HS time warp; identical raw sign for corridor "
        f"and served reference{title_suffix}",
        fontsize=13,
    )
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.94))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=200, bbox_inches="tight")
    plt.close(figure)


def _json_safe(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _synthetic_profile() -> dict[str, Any]:
    return {
        "version": 1,
        "name": "synthetic_retrospective_validation_fixture",
        "units": "radian",
        "phase_grid": [0.0, 0.6, 1.0],
        "coordinates": {
            "pros_knee_angle": {
                "mean_rad": [-0.2, -0.8, -0.3],
                "std_rad": [0.1, 0.1, 0.1],
            },
            "pros_ankle_angle": {
                "mean_rad": [0.1, -0.2, 0.15],
                "std_rad": [0.05, 0.05, 0.05],
            },
        },
        "metadata": {
            "phase_parameterization": EXPECTED_PARAMETERIZATION,
            "canonical_to_phase": 0.6,
            "coordinate_conventions": {
                "pros_knee_angle": {
                    "runtime_sign": "OpenSim model coordinate sign",
                    "display_convention": "identity in this raw-sign fixture",
                },
                "pros_ankle_angle": {
                    "runtime_sign": "OpenSim model coordinate sign",
                    "display_convention": "identity",
                },
            },
        },
    }


def _synthetic_values(time_s: float) -> tuple[float, float]:
    values = {
        0.0: (-0.2, 0.1),
        0.1: (-0.35, 0.025),
        0.2: (-0.65, -0.125),
        0.3: (-0.7166666666666667, -0.1416666666666667),
        # At phase 0.8 the ankle corridor is [-0.075, 0.025].  This value
        # produces excursion 0.1 rad and independently expected loss 1.0.
        0.4: (-0.55, 0.125),
        0.5: (-0.3833333333333333, 0.0916666666666667),
        0.6: (-0.3, 0.15),
    }
    return values[round(time_s, 1)]


def _fixture_corridor(phase: float, coord: str) -> tuple[float, float, float]:
    grid = np.asarray([0.0, 0.6, 1.0])
    if coord == "knee":
        mean = float(np.interp(phase, grid, [-0.2, -0.8, -0.3]))
        std = 0.1
    else:
        mean = float(np.interp(phase, grid, [0.1, -0.2, 0.15]))
        std = 0.05
    return mean, mean - std, mean + std


def _fixture_payload_sample(time_s: float, phase: float) -> dict[str, float]:
    knee, ankle = _synthetic_values(time_s)
    knee_mean, knee_low, knee_high = _fixture_corridor(phase, "knee")
    ankle_mean, ankle_low, ankle_high = _fixture_corridor(phase, "ankle")
    knee_loss = _interval_loss(knee, knee_low, knee_high)
    ankle_loss = _interval_loss(ankle, ankle_low, ankle_high)
    return {
        "time_s": time_s,
        "phase": phase,
        "knee_served_ref_rad": knee,
        "ankle_served_ref_rad": ankle,
        "knee_min_rad": knee_low,
        "knee_max_rad": knee_high,
        "ankle_min_rad": ankle_low,
        "ankle_max_rad": ankle_high,
        "knee_loss": knee_loss["loss"],
        "ankle_loss": ankle_loss["loss"],
        "knee_excursion_rad": knee_loss["excursion_rad"],
        "ankle_excursion_rad": ankle_loss["excursion_rad"],
        "inside_score": 0.5 * (knee_loss["inside"] + ankle_loss["inside"]),
        "knee_mean_rad": knee_mean,
        "ankle_mean_rad": ankle_mean,
    }


def _synthetic_trace() -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for step, time_s in enumerate(np.arange(0.0, 0.61, 0.1)):
        time_value = float(round(float(time_s), 1))
        knee, ankle = _synthetic_values(time_value)
        row: dict[str, Any] = {
            "step": step,
            "time": time_value,
            "prosthetic_state": {
                "pros_knee_angle_served_ref": knee,
                "pros_ankle_angle_served_ref": ankle,
            },
            "phase_fsm": {"accepted_transitions_this_step": []},
            "morphology_completed_segments": [],
            "morphology_ledger_diagnostics": {
                "discard_reason": "",
                "discarded_segment_count": 0,
                "discarded_sample_count": 0,
                "overflowed": False,
                "nonmonotonic_sample": False,
                "pending_sample_count": 0,
                "active_segment_type": "",
                "active_segment_start_time_s": -1.0,
                "completed_segment_count": 0,
            },
            "reward_terms": {
                "morphology_settled_sample_count": 0.0,
                "morphology_settled_this_step": 0.0,
            },
        }
        rows.append(row)

    pending_counts = (1, 1, 2, 1, 2, 3, 0)
    active_types = ("", "stance", "stance", "swing", "swing", "swing", "")
    active_starts = (-1.0, 0.05, 0.05, 0.25, 0.25, 0.25, -1.0)
    for row, pending, active_type, active_start in zip(
        rows,
        pending_counts,
        active_types,
        active_starts,
    ):
        diagnostics = row["morphology_ledger_diagnostics"]
        diagnostics["pending_sample_count"] = pending
        diagnostics["active_segment_type"] = active_type
        diagnostics["active_segment_start_time_s"] = active_start

    rows[1]["phase_fsm"]["accepted_transitions_this_step"] = [
        {
            "event": "heel_strike",
            "event_time_s": 0.05,
            "closed_segment_type": "",
            "segment_start_time_s": -1.0,
            "segment_end_time_s": 0.05,
            "segment_valid": 1.0,
            "opens_segment_type": "stance",
        }
    ]
    rows[1]["morphology_ledger_diagnostics"].update(
        {
            "discard_reason": "before_first_anchor",
            "discarded_sample_count": 1,
        }
    )
    rows[3]["phase_fsm"]["accepted_transitions_this_step"] = [
        {
            "event": "toe_off",
            "event_time_s": 0.25,
            "closed_segment_type": "stance",
            "segment_start_time_s": 0.05,
            "segment_end_time_s": 0.25,
            "segment_valid": 1.0,
            "opens_segment_type": "swing",
        }
    ]
    rows[3]["morphology_completed_segments"] = [
        {
            "segment_type": "stance",
            "start_time_s": 0.05,
            "end_time_s": 0.25,
            "duration_s": 0.2,
            "sample_count": 2,
            "samples": [
                _fixture_payload_sample(0.1, 0.15),
                _fixture_payload_sample(0.2, 0.45),
            ],
        }
    ]
    rows[3]["morphology_ledger_diagnostics"]["completed_segment_count"] = 1
    rows[3]["reward_terms"].update(
        {
            "morphology_settled_sample_count": 2.0,
            "morphology_settled_this_step": 1.0,
        }
    )
    rows[6]["phase_fsm"]["accepted_transitions_this_step"] = [
        {
            "event": "heel_strike",
            "event_time_s": 0.55,
            "closed_segment_type": "swing",
            "segment_start_time_s": 0.25,
            "segment_end_time_s": 0.55,
            "segment_valid": 1.0,
            "opens_segment_type": "stance",
        }
    ]
    rows[6]["morphology_completed_segments"] = [
        {
            "segment_type": "swing",
            "start_time_s": 0.25,
            "end_time_s": 0.55,
            "duration_s": 0.3,
            "sample_count": 3,
            "samples": [
                _fixture_payload_sample(0.3, 2.0 / 3.0),
                _fixture_payload_sample(0.4, 0.8),
                _fixture_payload_sample(0.5, 14.0 / 15.0),
            ],
        }
    ]
    rows[6]["morphology_ledger_diagnostics"].update(
        {
            "discard_reason": "episode_end_incomplete_segment",
            "discarded_segment_count": 1,
            "discarded_sample_count": 1,
            "completed_segment_count": 1,
        }
    )
    rows[6]["reward_terms"].update(
        {
            "morphology_settled_sample_count": 3.0,
            "morphology_settled_this_step": 1.0,
        }
    )
    return rows


def _write_synthetic_fixture(output_dir: Path) -> tuple[Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    trace_path = output_dir / "synthetic_rollout_policy_trace.json"
    profile_path = output_dir / "synthetic_morphology_profile.json"
    trace_path.write_text(
        json.dumps(_synthetic_trace(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    profile_path.write_text(
        json.dumps(_synthetic_profile(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return trace_path, profile_path


def _resolve_config_value(
    explicit: float | None,
    reward_config: Mapping[str, Any],
    key: str,
    fallback: float,
) -> float:
    if explicit is not None:
        return float(explicit)
    candidate = _finite_float(reward_config.get(key))
    return float(candidate if candidate is not None else fallback)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "trace",
        nargs="?",
        type=Path,
        help="rollout_policy_trace.json to validate (omitted with --self-test)",
    )
    parser.add_argument(
        "--profile",
        type=Path,
        default=None,
        help="event-warped morphology profile; otherwise resolved from rollout_summary.json",
    )
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--summary-json", type=Path, default=None)
    parser.add_argument("--plot", type=Path, default=None)
    parser.add_argument("--knee-std-multiplier", type=float, default=None)
    parser.add_argument("--ankle-std-multiplier", type=float, default=None)
    parser.add_argument("--knee-margin-deg", type=float, default=None)
    parser.add_argument("--ankle-margin-deg", type=float, default=None)
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="write and validate a deterministic synthetic journal+payload fixture",
    )
    parser.add_argument(
        "--strict",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="return exit code 1 when any required validation check fails",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.self_test:
        output_dir = (
            args.output_dir.resolve()
            if args.output_dir is not None
            else (SCRIPT_DIR / "retrospective_morphology_self_test").resolve()
        )
        trace_path, profile_path = _write_synthetic_fixture(output_dir)
        knee_multiplier = 1.0
        ankle_multiplier = 1.0
        knee_margin_deg = 0.0
        ankle_margin_deg = 0.0
    else:
        if args.trace is None:
            raise SystemExit("trace path is required unless --self-test is used")
        trace_path = args.trace.expanduser().resolve()
        if not trace_path.is_file():
            raise FileNotFoundError(f"rollout trace not found: {trace_path}")
        output_dir = (
            args.output_dir.expanduser().resolve()
            if args.output_dir is not None
            else (
                trace_path.parent / "experimental_retrospective_morphology_validation"
            )
        )
        profile_path = Path()
        knee_multiplier = ankle_multiplier = 0.0
        knee_margin_deg = ankle_margin_deg = 0.0

    rows, trace_root, parser_diagnostics = parse_trace(trace_path)
    reward_config = _sibling_reward_config(trace_path, trace_root)
    if not args.self_test:
        if args.profile is not None:
            profile_path = args.profile.expanduser().resolve()
        else:
            raw_profile = reward_config.get("morphology_profile")
            resolved = (
                _resolve_relative_path(str(raw_profile), trace_path)
                if isinstance(raw_profile, str) and raw_profile.strip()
                else None
            )
            profile_path = resolved or DEFAULT_PROFILE.resolve()
        knee_multiplier = _resolve_config_value(
            args.knee_std_multiplier,
            reward_config,
            "morphology_std_multiplier_knee",
            1.6,
        )
        ankle_multiplier = _resolve_config_value(
            args.ankle_std_multiplier,
            reward_config,
            "morphology_std_multiplier_ankle",
            0.6,
        )
        knee_margin_deg = _resolve_config_value(
            args.knee_margin_deg,
            reward_config,
            "morphology_margin_knee_deg",
            7.5,
        )
        ankle_margin_deg = _resolve_config_value(
            args.ankle_margin_deg,
            reward_config,
            "morphology_margin_ankle_deg",
            7.5,
        )

    if not profile_path.is_file():
        raise FileNotFoundError(f"morphology profile not found: {profile_path}")
    profile = load_profile(profile_path)
    journal_segments, journal_diagnostics = segments_from_journal(rows)
    payload_definitions, payloads, payload_diagnostics = payload_segments(rows)
    if not journal_segments and not payload_definitions:
        raise ValueError(
            "trace contains no valid completed segment. Expected per-step "
            "phase_fsm.accepted_transitions_this_step and/or "
            "morphology_completed_segments. Use --self-test for an explicit fixture."
        )

    representation_crosscheck = _match_segment_definitions(
        journal_segments,
        payload_definitions,
    )
    audit_summary, computed_segments = audit(
        rows=rows,
        profile=profile,
        journal_segments=journal_segments,
        payload_definitions=payload_definitions,
        payloads=payloads,
        knee_multiplier=knee_multiplier,
        ankle_multiplier=ankle_multiplier,
        knee_margin_deg=knee_margin_deg,
        ankle_margin_deg=ankle_margin_deg,
    )
    coverage_integrity = coverage_and_integrity(
        rows=rows,
        payloads=payloads,
        independent_settled_sample_count=int(
            audit_summary["sample_ownership"]["assigned_sample_count"]
        ),
        independent_completed_segment_count=int(audit_summary["segment_count"]),
    )

    representation_pass = (
        not representation_crosscheck["available"]
        or representation_crosscheck["exact_match"]
    )
    journal_quality_pass = (
        journal_diagnostics["invalid_transition_count"] == 0
        and journal_diagnostics["duplicate_transition_count"] == 0
        and journal_diagnostics["sequence_mismatch_count"] == 0
    )
    payload_quality_pass = (
        payload_diagnostics["invalid_payload_segment_count"] == 0
        and payload_diagnostics["duplicate_payload_segment_count"] == 0
    )
    audit_summary["checks"].update(
        {
            "journal_schema_valid": journal_quality_pass,
            "completed_payload_schema_valid": payload_quality_pass,
            "journal_and_payload_segment_definitions_match": representation_pass,
            "ledger_settlement_sources_consistent": coverage_integrity[
                "settled_sources_consistent"
            ],
            "ledger_discard_counts_valid": coverage_integrity["discard_counts_valid"],
            "no_fatal_ledger_integrity_reason": coverage_integrity[
                "fatal_integrity_pass"
            ],
        }
    )
    audit_summary["coverage_integrity"] = coverage_integrity
    audit_summary["pass"] = all(audit_summary["checks"].values())

    summary_path = (
        args.summary_json.expanduser().resolve()
        if args.summary_json is not None
        else output_dir / "experimental_retrospective_morphology_summary.json"
    )
    plot_path = (
        args.plot.expanduser().resolve()
        if args.plot is not None
        else output_dir / "experimental_retrospective_morphology_time_overlay.png"
    )
    result = {
        "schema_version": 1,
        "validator": "validation/validate_experimental_retrospective_morphology.py",
        "independent_runtime_helper_imports": [],
        "pass": audit_summary["pass"],
        "inputs": {
            "trace": str(trace_path),
            "profile": str(profile.path),
            "self_test": bool(args.self_test),
        },
        "corridor_config": {
            "morphology_std_multiplier_knee": knee_multiplier,
            "morphology_std_multiplier_ankle": ankle_multiplier,
            "morphology_margin_knee_deg": knee_margin_deg,
            "morphology_margin_ankle_deg": ankle_margin_deg,
        },
        "profile": {
            "phase_parameterization": profile.metadata.get("phase_parameterization"),
            "canonical_to_phase": profile.canonical_to_phase,
            "phase_grid_points": int(profile.phase.size),
        },
        "parser": parser_diagnostics,
        "journal": journal_diagnostics,
        "completed_payload": payload_diagnostics,
        "representation_crosscheck": representation_crosscheck,
        "coverage_integrity": coverage_integrity,
        "audit": audit_summary,
        "artifacts": {
            "summary_json": str(summary_path),
            "time_domain_overlay_png": str(plot_path),
        },
    }

    if args.self_test:
        fatal_probe_row = ParsedTraceRow(
            row_index=0,
            step=0,
            time_s=0.0,
            knee_rad=-0.2,
            ankle_rad=0.1,
            transitions=(),
            completed_segments=(),
            ledger_diagnostics={
                "discard_reason": (
                    "buffer_overflow|nonmonotonic_sample|transition_start_mismatch"
                ),
                "discarded_segment_count": 1,
                "discarded_sample_count": 3,
                "overflowed": True,
                "nonmonotonic_sample": True,
                "pending_sample_count": 0,
                "completed_segment_count": 0,
            },
            reward_terms={},
        )
        fatal_probe = coverage_and_integrity(
            rows=(fatal_probe_row,),
            payloads=(),
            independent_settled_sample_count=0,
            independent_completed_segment_count=0,
        )
        expected_fixture = {
            "segment_count": 2,
            "assigned_sample_count": 5,
            "ankle_loss_sum": 1.0,
            "morphology_loss_sum": 0.5,
            "settled_sample_count": 5,
            "discarded_sample_count": 2,
            "expected_boundary_discarded_sample_count": 2,
            "fatal_integrity_occurrence_count": 0,
            "settled_coverage": 5.0 / 7.0,
            "fatal_probe_structural_pass": False,
        }
        observed_fixture = {
            "segment_count": audit_summary["segment_count"],
            "assigned_sample_count": audit_summary["sample_ownership"][
                "assigned_sample_count"
            ],
            "ankle_loss_sum": audit_summary["loss"]["ankle_loss_sum"],
            "morphology_loss_sum": audit_summary["loss"]["morphology_loss_sum"],
            "settled_sample_count": coverage_integrity["settled_sample_count"],
            "discarded_sample_count": coverage_integrity["discarded_sample_count"],
            "expected_boundary_discarded_sample_count": coverage_integrity[
                "classification"
            ]["expected_bootstrap_tail_discarded_samples"],
            "fatal_integrity_occurrence_count": sum(
                coverage_integrity["classification"][
                    "fatal_integrity_occurrences"
                ].values()
            ),
            "settled_coverage": coverage_integrity[
                "settled_over_settled_plus_discarded"
            ],
            "fatal_probe_structural_pass": fatal_probe["structural_pass"],
        }
        fixture_pass = (
            observed_fixture["segment_count"] == 2
            and observed_fixture["assigned_sample_count"] == 5
            and math.isclose(observed_fixture["ankle_loss_sum"], 1.0, abs_tol=1.0e-12)
            and math.isclose(
                observed_fixture["morphology_loss_sum"], 0.5, abs_tol=1.0e-12
            )
            and observed_fixture["settled_sample_count"] == 5
            and observed_fixture["discarded_sample_count"] == 2
            and observed_fixture["expected_boundary_discarded_sample_count"] == 2
            and observed_fixture["fatal_integrity_occurrence_count"] == 0
            and math.isclose(
                observed_fixture["settled_coverage"],
                5.0 / 7.0,
                abs_tol=1.0e-12,
            )
            and not fatal_probe["structural_pass"]
            and set(fatal_probe["classification"]["fatal_integrity_occurrences"])
            == {
                "buffer_overflow",
                "nonmonotonic_sample",
                "transition_start_mismatch",
            }
        )
        result["synthetic_fixture_assertion"] = {
            "pass": fixture_pass,
            "expected": expected_fixture,
            "observed": observed_fixture,
            "fatal_integrity_probe": fatal_probe,
        }
        result["pass"] = bool(result["pass"] and fixture_pass)

    plot_time_overlay(
        computed_segments,
        rows=rows,
        profile=profile,
        knee_multiplier=knee_multiplier,
        ankle_multiplier=ankle_multiplier,
        knee_margin_deg=knee_margin_deg,
        ankle_margin_deg=ankle_margin_deg,
        output_path=plot_path,
        title_suffix=" — synthetic fixture" if args.self_test else "",
    )
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(
        json.dumps(_json_safe(result), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    status = "PASS" if result["pass"] else "FAIL"
    print(f"[{status}] experimental retrospective morphology validation")
    print(f"summary: {summary_path}")
    print(f"plot:    {plot_path}")
    print(
        "segments: "
        f"{audit_summary['segment_count']} | samples: "
        f"{audit_summary['sample_ownership']['assigned_sample_count']} | "
        f"phase plateaus: {audit_summary['phase_mapping']['plateau_count']}"
    )
    return 0 if result["pass"] or not args.strict else 1


if __name__ == "__main__":
    raise SystemExit(main())
