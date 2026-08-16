#!/usr/bin/env python3
"""Compare persisted V12R11/R10 +0.20 traces with the safe V8R1P1 tape.

This is a deliberately offline diagnostic.  It reads locked JSON evidence and
uses only the Python standard library.  It never imports project runtime code,
loads a checkpoint, queries a policy/teacher, builds an environment, fits a
model, or executes a rollout.  Its only optional write is the deterministic
result JSON in this diagnostic directory.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[6]
TG_VALIDATION = REPO_ROOT / "Trajectory Generator/baseline_MLP/validation"
CASE_ID = "deterministic_offset_plus_0p20"
PENETRATION_LIMIT_M = 0.025
INVARIANT_COLUMNS = tuple(range(2, 10)) + tuple(range(25, 35))
EXPECTED_ACTION_DIM = 2

R11_CASE = (
    TG_VALIDATION
    / "v12r11/diagnostics/h0_on_v26_plus_probe/artifacts"
    / "20260815_deterministic_offset_plus_0p20"
)
R10_RUN = TG_VALIDATION / "v12r10/h0_v12r10_run_20260815"
R10_CASE = R10_RUN / "development" / CASE_ID
SAFE_CASE = (
    REPO_ROOT
    / "validation/h0_primary_grf_split_adaptation_runs"
    / "2026-08-07_h0_primary_split_v8r1p1_v26_residual"
    / "teacher_replay"
    / CASE_ID
)
DEFAULT_OUTPUT = Path(__file__).resolve().parent / "results/comparison.json"

LOCKED_FILES = {
    R11_CASE / "trace.json": (
        "b5dc7a277fc013ecc1685354c969d9702d343d00925406fa04066032dae42a7d"
    ),
    R11_CASE / "summary.json": (
        "de39911c3b91a9d6fffc4f76d5a5671f68d3c8549ba2177d7034a8dd5201304e"
    ),
    R11_CASE / "gate.json": (
        "d93e6dc42278cf9b4c0a6e2d1713fb8ca7d5077c0f8b4cdc2b341f4e26782bef"
    ),
    R11_CASE / "closure_receipt.json": (
        "a0a694fae32b3edf41b0c159247dbb9602fefab52dd2428cdaa7b4a64f5e9ccf"
    ),
    R10_RUN / "pipeline_ledger.json": (
        "cf50e9450e29abbb8ef9ce759b825a6d5e09905fb6b62d6f2161feed3f6f1cb1"
    ),
    SAFE_CASE / "trace.json": (
        "c3ca347011ce79f5c8d1d3235d1a1b2b595eb7fcfafdc1a74f60dc34e640887d"
    ),
    SAFE_CASE / "summary.json": (
        "b8853f5e3b2ba6027f3edb310dee427ead087e29bdd0c720ddf3ea56dde53f5f"
    ),
    SAFE_CASE / "gate.json": (
        "90dd5bc18dbe7ff70b49ac69b36d1d92d7d7b41d9ce1403cf6e0bf1666f9fb52"
    ),
}
LOCKED_R10_JOURNAL = {
    "count": 212,
    "size_bytes": 2_275_738,
    "manifest_sha256": (
        "a311bb924f12127ad4f068aa5653085ffeb21a320f0178335987cf9253408a9e"
    ),
}

VECTOR_THRESHOLDS = (0.0, 1e-9, 1e-6, 1e-4, 1e-3, 1e-2, 5e-2, 1e-1, 1.0)
ACTION_THRESHOLDS = (0.0, 1e-6, 1e-4, 1e-3, 1e-2, 5e-2, 1e-1, 2e-1)
PENETRATION_THRESHOLDS = (0.0, 1e-9, 1e-7, 1e-6, 1e-5, 1e-4, 1e-3)


class ComparisonError(RuntimeError):
    """Raised if immutable evidence or its expected schema has drifted."""


def _relative(path: Path) -> str:
    return path.resolve().relative_to(REPO_ROOT).as_posix()


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _canonical_bytes(value: Any) -> bytes:
    return json.dumps(
        value,
        allow_nan=False,
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")


def _record(path: Path) -> dict[str, Any]:
    payload = path.read_bytes()
    return {
        "path": _relative(path),
        "sha256": _sha256_bytes(payload),
        "size_bytes": len(payload),
    }


def _load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _finite_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _verify_locked_files() -> list[dict[str, Any]]:
    records = []
    for path, expected_sha256 in LOCKED_FILES.items():
        record = _record(path)
        if record["sha256"] != expected_sha256:
            raise ComparisonError(f"locked artifact drifted: {record['path']}")
        records.append(record)
    return records


def _load_r10_journal() -> tuple[list[dict[str, Any]], dict[str, Any]]:
    files = sorted((R10_CASE / "steps").glob("*.json"))
    expected_names = [f"{step:06d}.json" for step in range(1, len(files) + 1)]
    if [path.name for path in files] != expected_names:
        raise ComparisonError("R10 journal filenames are not contiguous")
    records = [_record(path) for path in files]
    compact = [
        {
            "path": Path(record["path"]).name,
            "sha256": record["sha256"],
            "size_bytes": record["size_bytes"],
        }
        for record in records
    ]
    journal = {
        "algorithm": "sha256(canonical_json([{path,sha256,size_bytes},...]))",
        "count": len(records),
        "size_bytes": sum(record["size_bytes"] for record in records),
        "manifest_sha256": _sha256_bytes(_canonical_bytes(compact)),
        "first_step": records[0],
        "final_step": records[-1],
    }
    for key, expected in LOCKED_R10_JOURNAL.items():
        if journal[key] != expected:
            raise ComparisonError(f"R10 locked journal drifted at {key}")
    rows = [_load_json(path) for path in files]
    return rows, journal


def _vector(row: Mapping[str, Any], key: str, size: int) -> list[float]:
    value = row.get(key)
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes))
        or len(value) != size
        or not all(_finite_number(item) for item in value)
    ):
        raise ComparisonError(f"invalid {key} vector")
    return [float(item) for item in value]


def _matrix(
    rows: Sequence[Mapping[str, Any]], key: str, *, columns: Sequence[int] | None = None
) -> list[list[float]]:
    expected_size = 35 if columns is not None else EXPECTED_ACTION_DIM
    matrix = [_vector(row, key, expected_size) for row in rows]
    if columns is None:
        return matrix
    return [[row[column] for column in columns] for row in matrix]


def _validate_rows(
    rows: Sequence[Any],
    *,
    label: str,
    observation_key: str,
    time_key: str,
    expected_count: int,
) -> list[dict[str, Any]]:
    if len(rows) != expected_count or not all(isinstance(row, Mapping) for row in rows):
        raise ComparisonError(f"{label} row count/schema drifted")
    typed = [dict(row) for row in rows]
    for step, row in enumerate(typed, start=1):
        if row.get("step") != step:
            raise ComparisonError(f"{label} steps are not contiguous")
        if not _finite_number(row.get(time_key)):
            raise ComparisonError(f"{label} time is invalid at step {step}")
        _vector(row, observation_key, 35)
        _vector(
            row,
            "frozen_teacher_mean" if label == "safe" else "raw_action",
            EXPECTED_ACTION_DIM,
        )
        _vector(row, "applied_action", EXPECTED_ACTION_DIM)
    return typed


def _load_inputs() -> dict[str, Any]:
    locked_records = _verify_locked_files()
    r11 = _validate_rows(
        _load_json(R11_CASE / "trace.json"),
        label="r11",
        observation_key="v26_observation",
        time_key="time_s",
        expected_count=209,
    )
    r10_raw, r10_journal = _load_r10_journal()
    r10 = _validate_rows(
        r10_raw,
        label="r10",
        observation_key="v26_observation",
        time_key="time_s",
        expected_count=212,
    )
    safe = _validate_rows(
        _load_json(SAFE_CASE / "trace.json"),
        label="safe",
        observation_key="v25_observation",
        time_key="runtime_time_s",
        expected_count=500,
    )
    safe_summary = _load_json(SAFE_CASE / "summary.json")
    r11_summary = _load_json(R11_CASE / "summary.json")
    r11_gate = _load_json(R11_CASE / "gate.json")
    safe_gate = _load_json(SAFE_CASE / "gate.json")

    actor_names = safe_summary.get("actor_feature_names")
    if not isinstance(actor_names, list) or len(actor_names) != 35:
        raise ComparisonError("safe actor feature names drifted")
    if tuple(safe_summary.get("invariant_columns", ())) != INVARIANT_COLUMNS:
        raise ComparisonError("safe invariant-column contract drifted")
    if r11_gate.get("diagnostic_integrity_passed") is not True:
        raise ComparisonError("R11 diagnostic integrity is not PASS")
    if r11_gate.get("physical_passed") is not False:
        raise ComparisonError("R11 physical disposition drifted")
    if safe_gate.get("passed") is not True:
        raise ComparisonError("safe replay gate is not PASS")
    if r11_summary.get("end_reason") != "grf_penetration":
        raise ComparisonError("R11 end reason drifted")
    if safe_summary.get("end_reason") != "episode_time_limit":
        raise ComparisonError("safe end reason drifted")

    common = min(len(r11), len(r10), len(safe))
    for index in range(common):
        times = (
            float(r11[index]["time_s"]),
            float(r10[index]["time_s"]),
            float(safe[index]["runtime_time_s"]),
        )
        if times[0] != times[1] or times[0] != times[2]:
            raise ComparisonError(f"time alignment drifted at step {index + 1}")

    safe_target_exact = all(
        row["frozen_teacher_mean"]
        == row["frozen_raw_action"]
        == row["queried_teacher_mean"]
        == row["applied_action"]
        for row in safe
    )
    if not safe_target_exact:
        raise ComparisonError("safe target/action binding drifted")

    return {
        "locked_records": locked_records,
        "r10_journal": r10_journal,
        "r11": r11,
        "r10": r10,
        "safe": safe,
        "r11_summary": r11_summary,
        "r11_gate": r11_gate,
        "safe_summary": safe_summary,
        "safe_gate": safe_gate,
        "actor_names": actor_names,
    }


def _mean(values: Sequence[float]) -> float:
    return math.fsum(values) / len(values)


def _root_mean_square(values: Sequence[float]) -> float:
    return math.sqrt(math.fsum(value * value for value in values) / len(values))


def _threshold_key(value: float) -> str:
    return "gt_" + format(value, ".12g")


def _first_exceedance(values: Sequence[float], threshold: float) -> int | None:
    return next(
        (index for index, value in enumerate(values, start=1) if value > threshold),
        None,
    )


def _vector_difference(
    left: Sequence[Sequence[float]],
    right: Sequence[Sequence[float]],
    *,
    thresholds: Sequence[float],
    dimension_names: Sequence[str],
) -> dict[str, Any]:
    count = min(len(left), len(right))
    if count == 0 or len(dimension_names) != len(left[0]):
        raise ComparisonError("invalid vector comparison")
    signed = [
        [left[index][dim] - right[index][dim] for dim in range(len(left[index]))]
        for index in range(count)
    ]
    row_max = [max(abs(value) for value in row) for row in signed]
    flat = [value for row in signed for value in row]
    maximum = max(
        (
            (abs(signed[index][dim]), index, dim)
            for index in range(count)
            for dim in range(len(signed[index]))
        ),
        key=lambda item: item[0],
    )
    per_dimension = {}
    for dim, name in enumerate(dimension_names):
        values = [row[dim] for row in signed]
        max_index = max(range(count), key=lambda index: abs(values[index]))
        per_dimension[name] = {
            "mean_signed": _mean(values),
            "mean_absolute": _mean([abs(value) for value in values]),
            "root_mean_square": _root_mean_square(values),
            "maximum_absolute": abs(values[max_index]),
            "maximum_absolute_step": max_index + 1,
            "signed_at_maximum_absolute": values[max_index],
        }
    return {
        "aligned_row_count": count,
        "first_exact_vector_mismatch_step": next(
            (
                index + 1
                for index in range(count)
                if list(left[index]) != list(right[index])
            ),
            None,
        ),
        "first_row_max_exceedance_step": {
            _threshold_key(threshold): _first_exceedance(row_max, threshold)
            for threshold in thresholds
        },
        "root_mean_square_all_values": _root_mean_square(flat),
        "mean_absolute_all_values": _mean([abs(value) for value in flat]),
        "maximum_absolute": maximum[0],
        "maximum_absolute_step": maximum[1] + 1,
        "maximum_absolute_dimension": dimension_names[maximum[2]],
        "initial_signed_delta": signed[0],
        "terminal_aligned_signed_delta": signed[-1],
        "per_dimension": per_dimension,
    }


def _scalar_difference(
    left: Sequence[float], right: Sequence[float], thresholds: Sequence[float]
) -> dict[str, Any]:
    count = min(len(left), len(right))
    signed = [left[index] - right[index] for index in range(count)]
    absolute = [abs(value) for value in signed]
    max_index = max(range(count), key=absolute.__getitem__)
    return {
        "aligned_row_count": count,
        "first_exact_mismatch_step": next(
            (index + 1 for index, value in enumerate(signed) if value != 0.0), None
        ),
        "first_absolute_exceedance_step": {
            _threshold_key(threshold): _first_exceedance(absolute, threshold)
            for threshold in thresholds
        },
        "root_mean_square": _root_mean_square(signed),
        "mean_absolute": _mean(absolute),
        "maximum_absolute": absolute[max_index],
        "maximum_absolute_step": max_index + 1,
        "signed_at_maximum_absolute": signed[max_index],
        "initial_signed_delta": signed[0],
        "terminal_aligned_signed_delta": signed[-1],
    }


def _penetrations(rows: Sequence[Mapping[str, Any]], label: str) -> list[float]:
    if label == "safe":
        values = [row.get("reward_terms", {}).get("grf_penetration_m") for row in rows]
    else:
        values = [row.get("grf_penetration_m") for row in rows]
    if not all(_finite_number(value) for value in values):
        raise ComparisonError(f"invalid {label} penetration")
    return [float(value) for value in values]


def _rise_to_peak(values: Sequence[float]) -> dict[str, Any]:
    peak_index = max(range(len(values)), key=values.__getitem__)
    start = peak_index
    while start > 0 and values[start] > values[start - 1]:
        start -= 1
    steps = peak_index - start
    return {
        "start_step": start + 1,
        "peak_step": peak_index + 1,
        "row_count": peak_index - start + 1,
        "start_m": values[start],
        "peak_m": values[peak_index],
        "rise_m": values[peak_index] - values[start],
        "mean_rise_per_step_m": (
            (values[peak_index] - values[start]) / steps if steps else 0.0
        ),
        "last_increment_m": (
            values[peak_index] - values[peak_index - 1] if peak_index else 0.0
        ),
    }


def _sustained_positive_suffix_start(values: Sequence[float]) -> int | None:
    if not values or values[-1] <= 0.0:
        return None
    index = len(values) - 1
    while index > 0 and values[index - 1] > 0.0:
        index -= 1
    return index + 1


def _penetration_summary(
    values: Sequence[float], rows: Sequence[Mapping[str, Any]]
) -> dict[str, Any]:
    maximum_index = max(range(len(values)), key=values.__getitem__)
    final = rows[-1]
    return {
        "row_count": len(rows),
        "maximum_m": values[maximum_index],
        "maximum_step": maximum_index + 1,
        "minimum_clearance_to_limit_m": PENETRATION_LIMIT_M - values[maximum_index],
        "limit_passed": values[maximum_index] < PENETRATION_LIMIT_M,
        "first_step_above_0p020_m": _first_exceedance(values, 0.020),
        "first_step_above_0p024_m": _first_exceedance(values, 0.024),
        "terminal_step": int(final["step"]),
        "terminal_time_s": float(final.get("time_s", final.get("runtime_time_s"))),
        "terminal_m": values[-1],
        "terminal_end_reason": final.get("end_reason"),
        "rise_to_global_peak": _rise_to_peak(values),
    }


def _clipping_summary(
    rows: Sequence[Mapping[str, Any]], *, raw_key: str
) -> dict[str, Any]:
    clipped_steps = []
    clipped_values = []
    max_raw_abs = 0.0
    for row in rows:
        raw = _vector(row, raw_key, EXPECTED_ACTION_DIM)
        applied = _vector(row, "applied_action", EXPECTED_ACTION_DIM)
        max_raw_abs = max(max_raw_abs, *(abs(value) for value in raw))
        changed = [
            dim for dim in range(EXPECTED_ACTION_DIM) if raw[dim] != applied[dim]
        ]
        if changed:
            clipped_steps.append(int(row["step"]))
            clipped_values.extend(
                {"step": int(row["step"]), "dimension": dim} for dim in changed
            )
    return {
        "clipped_step_count": len(clipped_steps),
        "clipped_value_count": len(clipped_values),
        "clipped_steps": clipped_steps,
        "clipped_values": clipped_values,
        "maximum_raw_absolute": max_raw_abs,
    }


def _events(
    rows: Sequence[Mapping[str, Any]], *, time_key: str
) -> list[dict[str, Any]]:
    events = []
    for row in rows:
        phase = row.get("phase_fsm", {})
        accepted = phase.get("accepted_transitions_this_step", [])
        if not isinstance(accepted, list):
            raise ComparisonError("invalid accepted-event journal")
        for event in accepted:
            events.append(
                {
                    "ordinal": len(events) + 1,
                    "step": int(row["step"]),
                    "row_time_s": float(row[time_key]),
                    "event": event["event"],
                    "event_time_s": float(event["event_time_s"]),
                    "delivered_time_s": float(event["delivered_time_s"]),
                }
            )
    return events


def _event_alignment(
    candidate: Sequence[Mapping[str, Any]], safe: Sequence[Mapping[str, Any]]
) -> list[dict[str, Any]]:
    pairs = []
    for index in range(min(len(candidate), len(safe))):
        left = candidate[index]
        right = safe[index]
        pairs.append(
            {
                "ordinal": index + 1,
                "event": left["event"],
                "event_type_matches": left["event"] == right["event"],
                "candidate_step": left["step"],
                "safe_step": right["step"],
                "step_delta": left["step"] - right["step"],
                "event_time_delta_s": left["event_time_s"] - right["event_time_s"],
                "delivery_time_delta_s": (
                    left["delivered_time_s"] - right["delivered_time_s"]
                ),
            }
        )
    return pairs


def _population_stats(
    matrix: Sequence[Sequence[float]],
) -> tuple[list[float], list[float]]:
    means = [_mean([row[dim] for row in matrix]) for dim in range(len(matrix[0]))]
    scales = [
        math.sqrt(_mean([(row[dim] - means[dim]) ** 2 for row in matrix]))
        for dim in range(len(matrix[0]))
    ]
    if any(scale <= 0.0 or not math.isfinite(scale) for scale in scales):
        raise ComparisonError(
            "safe invariant feature has zero/nonfinite population scale"
        )
    return means, scales


def _standardize(
    matrix: Sequence[Sequence[float]], means: Sequence[float], scales: Sequence[float]
) -> list[list[float]]:
    return [
        [(row[dim] - means[dim]) / scales[dim] for dim in range(len(row))]
        for row in matrix
    ]


def _rms_distance(left: Sequence[float], right: Sequence[float]) -> float:
    return math.sqrt(
        math.fsum((a - b) ** 2 for a, b in zip(left, right, strict=True)) / len(left)
    )


def _leave_one_out_nearest(matrix: Sequence[Sequence[float]]) -> list[float]:
    nearest = [math.inf] * len(matrix)
    for left in range(len(matrix)):
        for right in range(left + 1, len(matrix)):
            distance = _rms_distance(matrix[left], matrix[right])
            if distance < nearest[left]:
                nearest[left] = distance
            if distance < nearest[right]:
                nearest[right] = distance
    return nearest


def _nearest_to_reference(
    queries: Sequence[Sequence[float]], reference: Sequence[Sequence[float]]
) -> tuple[list[float], list[int]]:
    distances = []
    indices = []
    for query in queries:
        best_index = 0
        best = math.inf
        for index, row in enumerate(reference):
            distance = _rms_distance(query, row)
            if distance < best:
                best = distance
                best_index = index
        distances.append(best)
        indices.append(best_index)
    return distances, indices


def _quantile(values: Sequence[float], probability: float) -> float:
    ordered = sorted(values)
    position = (len(ordered) - 1) * probability
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def _true_intervals(flags: Sequence[bool]) -> list[dict[str, int]]:
    intervals = []
    start = None
    for index, value in enumerate((*flags, False), start=1):
        if value and start is None:
            start = index
        elif not value and start is not None:
            intervals.append({"start_step": start, "end_step": index - 1})
            start = None
    return intervals


def _nearest_detail(
    query: Sequence[float],
    reference: Sequence[Sequence[float]],
    nearest_index: int,
    feature_names: Sequence[str],
    scales: Sequence[float],
) -> dict[str, Any]:
    contributions = []
    neighbor = reference[nearest_index]
    for dim, name in enumerate(feature_names):
        signed_z = (query[dim] - neighbor[dim]) / scales[dim]
        contributions.append(
            {
                "column": INVARIANT_COLUMNS[dim],
                "feature": name,
                "query": query[dim],
                "nearest_safe": neighbor[dim],
                "signed_z_delta": signed_z,
                "absolute_z_delta": abs(signed_z),
            }
        )
    contributions.sort(key=lambda item: item["absolute_z_delta"], reverse=True)
    return {
        "nearest_safe_step": nearest_index + 1,
        "top_feature_deltas": contributions[:6],
    }


def _box_support(
    queries: Sequence[Sequence[float]],
    safe: Sequence[Sequence[float]],
    feature_names: Sequence[str],
) -> dict[str, Any]:
    minimums = [min(row[dim] for row in safe) for dim in range(len(safe[0]))]
    maximums = [max(row[dim] for row in safe) for dim in range(len(safe[0]))]
    row_flags = []
    feature_counts = [0] * len(safe[0])
    feature_first = [None] * len(safe[0])
    for step, row in enumerate(queries, start=1):
        outside = False
        for dim, value in enumerate(row):
            if value < minimums[dim] or value > maximums[dim]:
                outside = True
                feature_counts[dim] += 1
                if feature_first[dim] is None:
                    feature_first[dim] = step
        row_flags.append(outside)
    features = [
        {
            "column": INVARIANT_COLUMNS[dim],
            "feature": feature_names[dim],
            "safe_minimum": minimums[dim],
            "safe_maximum": maximums[dim],
            "outside_row_count": feature_counts[dim],
            "first_outside_step": feature_first[dim],
        }
        for dim in range(len(safe[0]))
        if feature_counts[dim]
    ]
    return {
        "outside_row_count": sum(row_flags),
        "first_outside_step": next(
            (index for index, flag in enumerate(row_flags, start=1) if flag), None
        ),
        "outside_intervals": _true_intervals(row_flags),
        "violated_features": features,
    }


def _support_summary(
    queries: Sequence[Sequence[float]],
    *,
    safe_raw: Sequence[Sequence[float]],
    safe_z: Sequence[Sequence[float]],
    safe_exact: set[tuple[float, ...]],
    means: Sequence[float],
    scales: Sequence[float],
    loo_p99: float,
    loo_max: float,
    feature_names: Sequence[str],
) -> dict[str, Any]:
    query_z = _standardize(queries, means, scales)
    nearest, nearest_indices = _nearest_to_reference(query_z, safe_z)
    p99_flags = [value > loo_p99 for value in nearest]
    envelope_flags = [value > loo_max for value in nearest]
    exact_flags = [tuple(row) in safe_exact for row in queries]
    max_index = max(range(len(nearest)), key=nearest.__getitem__)
    first_p99 = next(
        (index for index, flag in enumerate(p99_flags, start=1) if flag), None
    )
    detail_steps = {1, len(queries), max_index + 1}
    if first_p99 is not None:
        detail_steps.add(first_p99)
    details = {}
    for step in sorted(detail_steps):
        detail = _nearest_detail(
            queries[step - 1],
            safe_raw,
            nearest_indices[step - 1],
            feature_names,
            scales,
        )
        detail["nearest_neighbor_rms_z"] = nearest[step - 1]
        details[str(step)] = detail
    return {
        "exact_safe_row_match_count": sum(exact_flags),
        "first_non_exact_safe_row_step": next(
            (index for index, flag in enumerate(exact_flags, start=1) if not flag), None
        ),
        "nearest_neighbor_rms_z": {
            "maximum": nearest[max_index],
            "maximum_step": max_index + 1,
            "terminal": nearest[-1],
            "terminal_nearest_safe_step": nearest_indices[-1] + 1,
            "p99_calibrated_threshold": loo_p99,
            "above_p99_row_count": sum(p99_flags),
            "first_above_p99_step": first_p99,
            "above_p99_intervals": _true_intervals(p99_flags),
            "maximum_loo_envelope_threshold": loo_max,
            "beyond_maximum_loo_envelope_row_count": sum(envelope_flags),
            "first_beyond_maximum_loo_envelope_step": next(
                (index for index, flag in enumerate(envelope_flags, start=1) if flag),
                None,
            ),
            "beyond_maximum_loo_envelope_intervals": _true_intervals(envelope_flags),
        },
        "empirical_per_feature_box": _box_support(queries, safe_raw, feature_names),
        "selected_step_details": details,
    }


def _aligned_standardized_difference(
    candidate: Sequence[Sequence[float]],
    safe: Sequence[Sequence[float]],
    scales: Sequence[float],
    feature_names: Sequence[str],
) -> dict[str, Any]:
    count = min(len(candidate), len(safe))
    distances = []
    for index in range(count):
        distances.append(
            math.sqrt(
                math.fsum(
                    ((candidate[index][dim] - safe[index][dim]) / scales[dim]) ** 2
                    for dim in range(len(scales))
                )
                / len(scales)
            )
        )
    max_index = max(range(count), key=distances.__getitem__)
    terminal_contributions = sorted(
        (
            {
                "column": INVARIANT_COLUMNS[dim],
                "feature": feature_names[dim],
                "signed_z_delta": (
                    (candidate[count - 1][dim] - safe[count - 1][dim]) / scales[dim]
                ),
            }
            for dim in range(len(scales))
        ),
        key=lambda item: abs(item["signed_z_delta"]),
        reverse=True,
    )
    return {
        "rms_z_maximum": distances[max_index],
        "rms_z_maximum_step": max_index + 1,
        "rms_z_terminal": distances[-1],
        "first_exceedance_step": {
            _threshold_key(threshold): _first_exceedance(distances, threshold)
            for threshold in (0.1, 0.25, 0.5, 1.0, 2.0)
        },
        "terminal_top_feature_deltas": terminal_contributions[:6],
    }


def _safe_target_contract(
    safe_features: Sequence[Sequence[float]],
    safe_actions: Sequence[Sequence[float]],
) -> dict[str, Any]:
    grouped: dict[tuple[float, ...], set[tuple[float, ...]]] = {}
    for features, action in zip(safe_features, safe_actions, strict=True):
        grouped.setdefault(tuple(features), set()).add(tuple(action))
    conflicts = [targets for targets in grouped.values() if len(targets) > 1]
    return {
        "row_count": len(safe_features),
        "unique_invariant_feature_row_count": len(grouped),
        "duplicate_invariant_feature_row_count": len(safe_features) - len(grouped),
        "conflicting_target_group_count": len(conflicts),
        "target_field": "frozen_teacher_mean",
        "target_binding_exact_for_all_rows": True,
        "target_action_range": [
            {
                "dimension": dim,
                "minimum": min(action[dim] for action in safe_actions),
                "maximum": max(action[dim] for action in safe_actions),
            }
            for dim in range(EXPECTED_ACTION_DIM)
        ],
    }


def _window_difference(
    candidate: Sequence[Sequence[float]],
    safe: Sequence[Sequence[float]],
    *,
    start_step: int,
    end_step: int,
    names: Sequence[str],
) -> dict[str, Any]:
    left = candidate[start_step - 1 : end_step]
    right = safe[start_step - 1 : end_step]
    result = _vector_difference(
        left,
        right,
        thresholds=ACTION_THRESHOLDS,
        dimension_names=names,
    )
    # Rebase window-local steps to episode steps.
    for key in ("maximum_absolute_step",):
        result[key] += start_step - 1
    for metrics in result["per_dimension"].values():
        metrics["maximum_absolute_step"] += start_step - 1
    result["start_step"] = start_step
    result["end_step"] = end_step
    result["safe_target_range"] = [
        {
            "dimension": names[dim],
            "minimum": min(row[dim] for row in right),
            "maximum": max(row[dim] for row in right),
        }
        for dim in range(EXPECTED_ACTION_DIM)
    ]
    result.pop("first_row_max_exceedance_step")
    result.pop("first_exact_vector_mismatch_step")
    return result


def build_result() -> dict[str, Any]:
    inputs = _load_inputs()
    r11 = inputs["r11"]
    r10 = inputs["r10"]
    safe = inputs["safe"]
    actor_names = inputs["actor_names"]
    invariant_names = [actor_names[index] for index in INVARIANT_COLUMNS]
    action_names = ("action_0", "action_1")

    r11_features = _matrix(r11, "v26_observation", columns=INVARIANT_COLUMNS)
    r10_features = _matrix(r10, "v26_observation", columns=INVARIANT_COLUMNS)
    safe_features = _matrix(safe, "v25_observation", columns=INVARIANT_COLUMNS)
    r11_actions = _matrix(r11, "raw_action")
    r10_actions = _matrix(r10, "raw_action")
    safe_actions = _matrix(safe, "frozen_teacher_mean")
    r11_penetration = _penetrations(r11, "r11")
    r10_penetration = _penetrations(r10, "r10")
    safe_penetration = _penetrations(safe, "safe")

    means, scales = _population_stats(safe_features)
    safe_z = _standardize(safe_features, means, scales)
    safe_loo = _leave_one_out_nearest(safe_z)
    loo_p99 = _quantile(safe_loo, 0.99)
    loo_max = max(safe_loo)
    safe_exact = {tuple(row) for row in safe_features}

    r11_events = _events(r11, time_key="time_s")
    r10_events = _events(r10, time_key="time_s")
    safe_events = _events(safe, time_key="runtime_time_s")
    safe_peak = _rise_to_peak(safe_penetration)
    critical_start = safe_peak["start_step"]

    r11_safe_pen_delta = [
        r11_penetration[index] - safe_penetration[index]
        for index in range(len(r11_penetration))
    ]
    r10_safe_pen_delta = [
        r10_penetration[index] - safe_penetration[index]
        for index in range(len(r10_penetration))
    ]

    result = {
        "schema_version": 1,
        "status": "PASS_V12R11_DIRECT_H0_SAFE_TAPE_OFFLINE_COMPARISON",
        "diagnostic_passed": True,
        "execution_mode": {
            "persisted_json_only": True,
            "project_runtime_imported": False,
            "checkpoint_or_policy_loaded": False,
            "policy_or_teacher_queried": False,
            "environment_built_or_executed": False,
            "fit_executed": False,
            "rollout_executed": False,
        },
        "diagnostic_script": _record(Path(__file__).resolve()),
        "locked_input_records": inputs["locked_records"],
        "r10_step_journal": inputs["r10_journal"],
        "comparison_contract": {
            "case_id": CASE_ID,
            "step_alignment": "same one-based step and exact persisted row time",
            "action_alignment": {
                "r11": "raw_action",
                "r10": "raw_action",
                "safe": "frozen_teacher_mean",
            },
            "penetration_alignment": {
                "r11": "grf_penetration_m",
                "r10": "grf_penetration_m",
                "safe": "reward_terms.grf_penetration_m",
            },
            "invariant_columns": list(INVARIANT_COLUMNS),
            "invariant_feature_names": invariant_names,
            "row_counts": {"r11": len(r11), "r10": len(r10), "safe": len(safe)},
            "penetration_limit_m": PENETRATION_LIMIT_M,
        },
        "exact_divergence_onset": {
            "r11_vs_safe": {
                "action": _vector_difference(
                    r11_actions,
                    safe_actions,
                    thresholds=ACTION_THRESHOLDS,
                    dimension_names=action_names,
                ),
                "invariant_observation": _vector_difference(
                    r11_features,
                    safe_features,
                    thresholds=VECTOR_THRESHOLDS,
                    dimension_names=invariant_names,
                ),
                "penetration": _scalar_difference(
                    r11_penetration, safe_penetration, PENETRATION_THRESHOLDS
                ),
            },
            "r10_vs_safe": {
                "action": _vector_difference(
                    r10_actions,
                    safe_actions,
                    thresholds=ACTION_THRESHOLDS,
                    dimension_names=action_names,
                ),
                "invariant_observation": _vector_difference(
                    r10_features,
                    safe_features,
                    thresholds=VECTOR_THRESHOLDS,
                    dimension_names=invariant_names,
                ),
                "penetration": _scalar_difference(
                    r10_penetration, safe_penetration, PENETRATION_THRESHOLDS
                ),
            },
            "r11_vs_r10": {
                "action": _vector_difference(
                    r11_actions,
                    r10_actions,
                    thresholds=ACTION_THRESHOLDS,
                    dimension_names=action_names,
                ),
                "invariant_observation": _vector_difference(
                    r11_features,
                    r10_features,
                    thresholds=VECTOR_THRESHOLDS,
                    dimension_names=invariant_names,
                ),
                "penetration": _scalar_difference(
                    r11_penetration, r10_penetration, PENETRATION_THRESHOLDS
                ),
            },
        },
        "action_dynamics": {
            "r11": _clipping_summary(r11, raw_key="raw_action"),
            "r10": _clipping_summary(r10, raw_key="raw_action"),
            "safe": _clipping_summary(safe, raw_key="frozen_teacher_mean"),
            "safe_peak_rise_window": {
                "safe_start_step": critical_start,
                "safe_peak_step": safe_peak["peak_step"],
                "safe_post_peak_step": safe_peak["peak_step"] + 1,
                "r11_vs_safe": _window_difference(
                    r11_actions,
                    safe_actions,
                    start_step=critical_start,
                    end_step=len(r11),
                    names=action_names,
                ),
                "r10_vs_safe": _window_difference(
                    r10_actions,
                    safe_actions,
                    start_step=critical_start,
                    end_step=len(r10),
                    names=action_names,
                ),
            },
        },
        "penetration_dynamics": {
            "r11": _penetration_summary(r11_penetration, r11),
            "r10": _penetration_summary(r10_penetration, r10),
            "safe": _penetration_summary(safe_penetration, safe),
            "safe_peak_rise": safe_peak,
            "r11_minus_safe": {
                "sustained_positive_suffix_start_step": (
                    _sustained_positive_suffix_start(r11_safe_pen_delta)
                ),
                "terminal_aligned_delta_m": r11_safe_pen_delta[-1],
                "safe_at_candidate_terminal_m": safe_penetration[len(r11) - 1],
                "candidate_last_increment_m": (
                    r11_penetration[-1] - r11_penetration[-2]
                ),
                "safe_same_step_increment_m": (
                    safe_penetration[len(r11) - 1] - safe_penetration[len(r11) - 2]
                ),
            },
            "r10_minus_safe": {
                "sustained_positive_suffix_start_step": (
                    _sustained_positive_suffix_start(r10_safe_pen_delta)
                ),
                "terminal_aligned_delta_m": r10_safe_pen_delta[-1],
                "safe_at_candidate_terminal_m": safe_penetration[len(r10) - 1],
                "candidate_last_increment_m": (
                    r10_penetration[-1] - r10_penetration[-2]
                ),
                "safe_same_step_increment_m": (
                    safe_penetration[len(r10) - 1] - safe_penetration[len(r10) - 2]
                ),
            },
        },
        "event_timelines": {
            "r11": r11_events,
            "r10": r10_events,
            "safe": safe_events,
            "r11_vs_safe_first_events": _event_alignment(r11_events, safe_events),
            "r10_vs_safe_first_events": _event_alignment(r10_events, safe_events),
            "r11_vs_r10_first_events": _event_alignment(r11_events, r10_events),
        },
        "support_18_invariant_features": {
            "definition": {
                "standardization": "safe-tape population mean/std per feature",
                "distance": "RMS z-distance to nearest safe-tape row",
                "nominal_ood": "distance > safe leave-one-out p99",
                "beyond_empirical_envelope": (
                    "distance > maximum safe leave-one-out nearest-neighbor distance"
                ),
                "box_ood": "any feature outside its full safe-tape min/max",
                "causality_note": (
                    "These are empirical support diagnostics, not certified physical "
                    "safety boundaries."
                ),
            },
            "safe_calibration": {
                "row_count": len(safe_features),
                "feature_population_mean": means,
                "feature_population_std": scales,
                "loo_nearest_neighbor_rms_z": {
                    "minimum": min(safe_loo),
                    "median": _quantile(safe_loo, 0.5),
                    "p90": _quantile(safe_loo, 0.90),
                    "p95": _quantile(safe_loo, 0.95),
                    "p99": loo_p99,
                    "maximum": loo_max,
                    "maximum_step": max(range(len(safe_loo)), key=safe_loo.__getitem__)
                    + 1,
                },
            },
            "r11": _support_summary(
                r11_features,
                safe_raw=safe_features,
                safe_z=safe_z,
                safe_exact=safe_exact,
                means=means,
                scales=scales,
                loo_p99=loo_p99,
                loo_max=loo_max,
                feature_names=invariant_names,
            ),
            "r10": _support_summary(
                r10_features,
                safe_raw=safe_features,
                safe_z=safe_z,
                safe_exact=safe_exact,
                means=means,
                scales=scales,
                loo_p99=loo_p99,
                loo_max=loo_max,
                feature_names=invariant_names,
            ),
            "time_aligned_standardized_difference": {
                "r11_vs_safe": _aligned_standardized_difference(
                    r11_features, safe_features, scales, invariant_names
                ),
                "r10_vs_safe": _aligned_standardized_difference(
                    r10_features, safe_features, scales, invariant_names
                ),
            },
        },
        "v12r12_fit_requirements": {
            "safe_target_contract": _safe_target_contract(safe_features, safe_actions),
            "predictive_input_contract": {
                "dimension": len(INVARIANT_COLUMNS),
                "columns": list(INVARIANT_COLUMNS),
                "features": invariant_names,
                "reason": (
                    "These mechanical/reference columns are byte-invariant across the "
                    "locked V8R1P1 replay adapter and avoid detector-semantic aliasing."
                ),
            },
            "must_reproduce": [
                (
                    "The locked V8R1P1 frozen_teacher_mean joint action, not direct H0 "
                    "and not the R10 LegacyGaitShadow-derived labels."
                ),
                (
                    "The full 500-row +0.20 mapping on the 18 invariant features, with "
                    "special gate reporting for the safe penetration-rise window."
                ),
                (
                    "The safe first-event schedule (TO step 109, HS step 148) and two "
                    "subsequent completed cycles as closed-loop outcomes."
                ),
                (
                    "A 500-step time-limit episode with zero clipping and penetration "
                    "strictly below 0.025 m; the locked safe maximum is only about "
                    "0.000676 m below the limit."
                ),
                (
                    "Candidate-exposed recovery labels before any one-shot: both failed "
                    "candidates leave nominal safe-tape support, so static on-tape error "
                    "alone cannot establish autonomous closure."
                ),
            ],
            "critical_safe_reference": {
                "source_trace": _record(SAFE_CASE / "trace.json"),
                "feature_field": "v25_observation[invariant_columns]",
                "target_field": "frozen_teacher_mean",
                "penetration_field": "reward_terms.grf_penetration_m",
                "safe_peak_rise": safe_peak,
                "safe_step_209": {
                    "target_action": safe_actions[208],
                    "penetration_m": safe_penetration[208],
                },
                "safe_step_211_peak": {
                    "target_action": safe_actions[210],
                    "penetration_m": safe_penetration[210],
                },
                "safe_step_212_post_peak": {
                    "target_action": safe_actions[211],
                    "penetration_m": safe_penetration[211],
                },
            },
            "interpretation_limit": (
                "Observed action deltas identify reproduction targets but are not causal "
                "or certified per-action tolerances; only a new preregistered physical "
                "development can test closure."
            ),
        },
        "conclusions": [
            (
                "All three runs start from the same 18-feature row. R11 and R10 choose "
                "different actions at step 1, and their invariant state rows first differ "
                "from the safe tape at step 2: the divergence is closed-loop compounding, "
                "not a late detector discontinuity."
            ),
            (
                "R11 removes R10 clipping but still terminates at step 209 from "
                "penetration; zero clipping is necessary but not sufficient. R10 fails at "
                "step 212 with four clipped steps, while the safe tape reaches its local "
                "peak at step 211 and then unloads."
            ),
            (
                "The failures use different joint-action error patterns: R11's largest "
                "safe-aligned error is action_1, while R10 combines an action_0 overshoot "
                "with a large action_1 undershoot. A successor must reproduce the joint "
                "safe controller rather than clamp one channel."
            ),
            (
                "R11 advances toe-off 13 steps and R10 advances it 9 steps relative to the "
                "safe replay; both collect only TO+HS before failure, versus six safe "
                "events and two valid cycles."
            ),
            (
                "Both failed trajectories eventually exceed the maximum calibrated safe "
                "nearest-neighbor envelope on the 18 invariant features. Therefore V12R12 "
                "needs safe-tape imitation plus candidate-exposed recovery coverage, with "
                "the +0.20 case retained as the first fail-closed physical discriminator."
            ),
        ],
    }
    result["canonical_payload_sha256_excluding_this_field"] = _sha256_bytes(
        _canonical_bytes(result)
    )
    return result


def _payload(result: Mapping[str, Any]) -> bytes:
    return (
        json.dumps(
            result,
            allow_nan=False,
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        )
        + "\n"
    ).encode("utf-8")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="Recompute, verify locks, and compare with an existing result without writing.",
    )
    parser.add_argument("--stdout", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    result = build_result()
    payload = _payload(result)
    output = args.output.expanduser().resolve()
    if args.check_only:
        if output.exists() and output.read_bytes() != payload:
            raise ComparisonError(f"deterministic result mismatch: {_relative(output)}")
        print(f"check-only payload_sha256={_sha256_bytes(payload)}")
    else:
        output.parent.mkdir(parents=True, exist_ok=True)
        temporary = output.with_suffix(output.suffix + ".tmp")
        temporary.write_bytes(payload)
        temporary.replace(output)
        print(f"wrote {_relative(output)} sha256={_sha256_bytes(payload)}")
    if args.stdout:
        print(payload.decode("utf-8"), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
