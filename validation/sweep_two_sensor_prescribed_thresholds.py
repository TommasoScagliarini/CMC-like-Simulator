"""Preregistered detector-only threshold selection on prescribed AB06 data.

The script deliberately separates three chronological stages:

``development``
    Evaluates the frozen ON/OFF grid and locks exactly one candidate.
``validation``
    Evaluates only the development winner and the unchanged 5/2 N baseline.
``sealed``
    Opens the final block only after a passing validation manifest.

Every candidate uses the production :class:`ProstheticPhaseFSM`; only the two
force thresholds vary and the sensor dwell remains fixed at 30 ms.  No policy,
checkpoint, reward, or PPO code is imported or modified.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sys
import traceback
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
for _path in (REPO_ROOT, TRAJECTORY_ROOT):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from online_grf import load_online_grf_profile  # noqa: E402
from path_resolver import resolve_repo_path  # noqa: E402
from setup_io import read_setup_xml  # noqa: E402
from validation.validate_online_grf import (  # noqa: E402
    _external_grf,
    _sample_spheres,
)
from validation.validate_online_grf_events import (  # noqa: E402
    match_events,
    smooth_force,
    strict_event_pass,
)
from validation.validate_two_sensor_prescribed_replay import (  # noqa: E402
    DEFAULT_PROFILE,
    DEFAULT_SETUP,
    PHASE_STANCE,
    _accepted_gait_transitions_for_gate,
    _current_runtime_fsm_config,
    _fsm_phase_from_state_id,
    _json_safe,
    _model_body_weight_n,
    _ordered_event_diagnostic,
    _phase_classification_gate,
    _plot,
    _prescribed_prosthetic_kinematics,
    _reference_events_from_prescribed_grf,
    _regional_loads_and_penetrations,
    _run_production_fsm,
    _semantic_gate,
)


DEFAULT_PROTOCOL = (
    REPO_ROOT / "validation/two_sensor_prescribed_threshold_sweep_protocol.json"
)
DEFAULT_OUTPUT_ROOT = (
    REPO_ROOT
    / "validation/two_sensor_prescribed_threshold_sweep_runs/2026-07-21_fullspan_v1"
)
BASELINE_ID = "on005_off002"
STAGES = ("development", "validation", "sealed")
EVENT_TIME_FIELDS = {"event_time_s", "confirmed_time_s"}
PHASE_REFERENCE_MODES = {"instantaneous_grf", "validated_event_intervals"}


class ProtocolError(ValueError):
    """Raised when a frozen protocol or stage lineage is not trustworthy."""


@dataclass(frozen=True)
class Candidate:
    candidate_id: str
    on_threshold_n: float
    off_threshold_n: float


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _portable(path: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(REPO_ROOT.resolve()).as_posix()
    except ValueError:
        return resolved.as_posix()


def _finite_number(value: Any, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ProtocolError(f"{label} must be numeric") from exc
    if not math.isfinite(result):
        raise ProtocolError(f"{label} must be finite")
    return result


def _threshold_token(value: float) -> str:
    number = float(value)
    if abs(number - round(number)) <= 1e-12:
        return f"{int(round(number)):03d}"
    return f"{number:05.2f}".replace(".", "p")


def _candidate_id(on_threshold_n: float, off_threshold_n: float) -> str:
    return (
        f"on{_threshold_token(on_threshold_n)}_"
        f"off{_threshold_token(off_threshold_n)}"
    )


def load_and_validate_protocol(
    path: str | Path = DEFAULT_PROTOCOL,
) -> dict[str, Any]:
    """Load the preregistration and reject drift/leakage fail-closed."""
    protocol_path = resolve_repo_path(path).resolve()
    try:
        raw = json.loads(protocol_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"cannot load protocol: {protocol_path}") from exc
    if (
        not isinstance(raw, dict)
        or raw.get("schema_version") not in {1, 2}
    ):
        raise ProtocolError("unsupported threshold protocol schema")
    if raw.get("frozen_before_execution") is not True:
        raise ProtocolError("protocol must be frozen before execution")

    split = raw.get("chronological_split")
    if not isinstance(split, dict):
        raise ProtocolError("chronological_split is required")
    expected_blocks = {
        "development_time_block_s": [11.99, 50.0],
        "validation_time_block_s": [50.0, 100.0],
        "sealed_time_block_s": [100.0, 155.045],
    }
    for name, expected in expected_blocks.items():
        observed = split.get(name)
        if not isinstance(observed, list) or len(observed) != 2:
            raise ProtocolError(f"invalid {name}")
        values = [_finite_number(value, name) for value in observed]
        if values != expected:
            raise ProtocolError(f"frozen split drift for {name}: {values}")
    if split.get("selection_may_read_validation_metrics") is not False:
        raise ProtocolError("development selection may not read validation")
    if split.get("selection_or_validation_may_read_sealed_metrics") is not False:
        raise ProtocolError("development/validation may not read sealed")

    replay = raw.get("replay")
    if not isinstance(replay, dict):
        raise ProtocolError("replay contract is required")
    if _finite_number(replay.get("primary_sample_dt_s"), "primary dt") != 0.01:
        raise ProtocolError("primary detector gate must run at 10 ms")
    if _finite_number(replay.get("sensitivity_sample_dt_s"), "sensitivity dt") != 0.001:
        raise ProtocolError("sensitivity must run at 1 ms")
    if _finite_number(replay.get("sensor_dwell_s"), "sensor dwell") != 0.03:
        raise ProtocolError("sensor dwell is frozen at 30 ms")
    schema_version = int(raw["schema_version"])
    if schema_version >= 2:
        if "primary_event_time_field" not in replay:
            raise ProtocolError("V3 requires primary_event_time_field")
        if "diagnostic_event_time_field" not in replay:
            raise ProtocolError("V3 requires diagnostic_event_time_field")
        if "phase_reference_mode" not in replay:
            raise ProtocolError("V3 requires phase_reference_mode")
    primary_time_field = str(
        replay.get("primary_event_time_field", "event_time_s")
    )
    diagnostic_time_field = str(
        replay.get(
            "diagnostic_event_time_field",
            (
                "confirmed_time_s"
                if primary_time_field == "event_time_s"
                else "event_time_s"
            ),
        )
    )
    if primary_time_field not in EVENT_TIME_FIELDS:
        raise ProtocolError(
            f"unsupported primary event timestamp: {primary_time_field}"
        )
    if diagnostic_time_field not in EVENT_TIME_FIELDS:
        raise ProtocolError(
            f"unsupported diagnostic event timestamp: {diagnostic_time_field}"
        )
    if primary_time_field == diagnostic_time_field:
        raise ProtocolError("primary and diagnostic event timestamps must differ")
    phase_reference_mode = str(
        replay.get("phase_reference_mode", "instantaneous_grf")
    )
    if phase_reference_mode not in PHASE_REFERENCE_MODES:
        raise ProtocolError(
            f"unsupported phase reference mode: {phase_reference_mode}"
        )
    if schema_version >= 2 and (
        primary_time_field != "confirmed_time_s"
        or diagnostic_time_field != "event_time_s"
        or phase_reference_mode != "validated_event_intervals"
    ):
        raise ProtocolError(
            "V3 must gate confirmed_time_s, diagnose event_time_s, and use "
            "validated_event_intervals for phase"
        )

    grid = raw.get("candidate_grid")
    if not isinstance(grid, dict):
        raise ProtocolError("candidate_grid is required")
    on_values = [_finite_number(value, "ON threshold") for value in grid.get("on_thresholds_n", [])]
    off_values = [_finite_number(value, "OFF threshold") for value in grid.get("off_thresholds_n", [])]
    if not on_values or not off_values or len(set(on_values)) != len(on_values) or len(set(off_values)) != len(off_values):
        raise ProtocolError("candidate grid must contain unique non-empty axes")
    if any(off < 0.0 for off in off_values):
        raise ProtocolError("OFF thresholds must be non-negative")
    append_external_baseline = bool(grid.get("append_baseline_outside_cartesian", False))
    if not append_external_baseline and any(
        off >= on for on in on_values for off in off_values
    ):
        raise ProtocolError("every frozen OFF threshold must be below every ON threshold")
    if append_external_baseline and any(
        off >= min(on_values) for off in off_values
    ):
        raise ProtocolError(
            "the V2 Cartesian development grid must keep every OFF below every ON"
        )
    baseline = grid.get("baseline")
    if not isinstance(baseline, dict):
        raise ProtocolError("baseline definition is required")
    if (
        baseline.get("candidate_id") != BASELINE_ID
        or _finite_number(baseline.get("on_threshold_n"), "baseline ON") != 5.0
        or _finite_number(baseline.get("off_threshold_n"), "baseline OFF") != 2.0
    ):
        raise ProtocolError("baseline must remain exactly 5/2 N")

    sources = raw.get("sources")
    if not isinstance(sources, dict) or not sources:
        raise ProtocolError("hash-pinned sources are required")
    for label, source in sources.items():
        if not isinstance(source, dict):
            raise ProtocolError(f"invalid source record: {label}")
        source_path = resolve_repo_path(str(source.get("path", ""))).resolve()
        if not source_path.is_file():
            raise ProtocolError(f"missing pinned source {label}: {source_path}")
        observed_hash = _sha256(source_path)
        if observed_hash != source.get("sha256"):
            raise ProtocolError(
                f"source hash drift for {label}: {observed_hash} != "
                f"{source.get('sha256')}"
            )
    raw["_protocol_path"] = protocol_path.as_posix()
    raw["_protocol_sha256"] = _sha256(protocol_path)
    raw["_primary_event_time_field"] = primary_time_field
    raw["_diagnostic_event_time_field"] = diagnostic_time_field
    raw["_phase_reference_mode"] = phase_reference_mode
    return raw


def build_candidate_grid(protocol: Mapping[str, Any]) -> list[Candidate]:
    grid = protocol["candidate_grid"]
    result = [
        Candidate(
            candidate_id=_candidate_id(float(on), float(off)),
            on_threshold_n=float(on),
            off_threshold_n=float(off),
        )
        for on in grid["on_thresholds_n"]
        for off in grid["off_thresholds_n"]
    ]
    if bool(grid.get("append_baseline_outside_cartesian", False)):
        baseline = grid["baseline"]
        result.append(
            Candidate(
                candidate_id=str(baseline["candidate_id"]),
                on_threshold_n=float(baseline["on_threshold_n"]),
                off_threshold_n=float(baseline["off_threshold_n"]),
            )
        )
    if len({item.candidate_id for item in result}) != len(result):
        raise ProtocolError("candidate IDs are not unique")
    baseline = [item for item in result if item.candidate_id == BASELINE_ID]
    if len(baseline) != 1 or (
        baseline[0].on_threshold_n,
        baseline[0].off_threshold_n,
    ) != (5.0, 2.0):
        raise ProtocolError("candidate grid does not contain the exact baseline")
    return result


def binary_metrics(
    reference: Sequence[bool] | np.ndarray,
    predicted: Sequence[bool] | np.ndarray,
) -> dict[str, Any]:
    ref = np.asarray(reference, dtype=bool)
    pred = np.asarray(predicted, dtype=bool)
    if ref.ndim != 1 or pred.ndim != 1 or ref.shape != pred.shape:
        raise ValueError("binary metric inputs must be equal-length 1-D arrays")
    tp = int(np.count_nonzero(ref & pred))
    fp = int(np.count_nonzero(~ref & pred))
    fn = int(np.count_nonzero(ref & ~pred))
    tn = int(np.count_nonzero(~ref & ~pred))
    precision = tp / max(1, tp + fp)
    recall = tp / max(1, tp + fn)
    f1 = 2.0 * precision * recall / max(1e-12, precision + recall)
    iou = tp / max(1, tp + fp + fn)
    return {
        "samples": int(ref.size),
        "true_positive": tp,
        "false_positive": fp,
        "false_negative": fn,
        "true_negative": tn,
        "precision": float(precision),
        "recall": float(recall),
        "f1": float(f1),
        "iou": float(iou),
        "accuracy": float((tp + tn) / max(1, ref.size)),
    }


def _required_finite(row: Mapping[str, Any], names: Iterable[str]) -> None:
    for name in names:
        if name not in row:
            raise ProtocolError(f"candidate row is missing {name}")
        _finite_number(row[name], name)


def _candidate_distance(row: Mapping[str, Any], candidates: Mapping[str, Candidate]) -> float:
    candidate = candidates.get(str(row["candidate_id"]))
    if candidate is None:
        raise ProtocolError(f"unknown candidate ID: {row.get('candidate_id')}")
    return abs(candidate.on_threshold_n - 5.0) / 15.0 + abs(candidate.off_threshold_n - 2.0) / 2.5


def _plateau_ids(
    rows: Sequence[Mapping[str, Any]],
    protocol: Mapping[str, Any],
) -> set[str]:
    """Return stable candidates; partial unit fixtures bypass the full-grid rule."""
    full_grid = build_candidate_grid(protocol)
    by_id = {str(row["candidate_id"]): row for row in rows}
    if set(by_id) != {item.candidate_id for item in full_grid}:
        return {str(row["candidate_id"]) for row in rows if bool(row["eligible"])}
    on_values = [float(value) for value in protocol["candidate_grid"]["on_thresholds_n"]]
    off_values = [float(value) for value in protocol["candidate_grid"]["off_thresholds_n"]]
    candidate_by_id = {item.candidate_id: item for item in full_grid}
    rules = protocol["stable_plateau"]
    required = int(rules["required_axis_adjacent_eligible_neighbors"])
    timing_limit = float(rules["maximum_neighbor_worst_timing_score_increase"])
    iou_limit = float(rules["maximum_neighbor_confirmed_fsm_iou_decrease"])
    stable: set[str] = set()
    for candidate_id, row in by_id.items():
        if not bool(row["eligible"]):
            continue
        candidate = candidate_by_id[candidate_id]
        if (
            candidate.on_threshold_n not in on_values
            or candidate.off_threshold_n not in off_values
        ):
            # The unchanged 5/2 N comparator may be appended outside a V2
            # low-force Cartesian development grid.  It is evaluated and
            # reported but cannot establish a local low-threshold plateau.
            continue
        on_index = on_values.index(candidate.on_threshold_n)
        off_index = off_values.index(candidate.off_threshold_n)
        neighbor_values: list[tuple[float, float]] = []
        for delta_on, delta_off in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            next_on = on_index + delta_on
            next_off = off_index + delta_off
            if 0 <= next_on < len(on_values) and 0 <= next_off < len(off_values):
                neighbor_values.append((on_values[next_on], off_values[next_off]))
        acceptable = 0
        for on_value, off_value in neighbor_values:
            neighbor = by_id[_candidate_id(on_value, off_value)]
            if not bool(neighbor["eligible"]):
                continue
            timing_increase = float(neighbor["worst_event_normalized_max_abs_error"]) - float(row["worst_event_normalized_max_abs_error"])
            iou_decrease = float(row["confirmed_fsm_stance_iou"]) - float(neighbor["confirmed_fsm_stance_iou"])
            if timing_increase <= timing_limit + 1e-12 and iou_decrease <= iou_limit + 1e-12:
                acceptable += 1
        if acceptable >= required:
            stable.add(candidate_id)
    return stable


def select_development_candidate(
    rows: Sequence[Mapping[str, Any]],
    protocol: Mapping[str, Any],
) -> dict[str, Any]:
    if not rows:
        raise ProtocolError("development produced no candidate rows")
    required = [
        "worst_event_normalized_max_abs_error",
        "mean_event_normalized_mean_abs_error",
        "confirmed_fsm_stance_iou",
    ]
    if int(protocol.get("schema_version", 1)) < 2:
        required.append("sensor_contact_stance_iou")
    grid = {item.candidate_id: item for item in build_candidate_grid(protocol)}
    normalized: list[dict[str, Any]] = []
    for raw in rows:
        row = dict(raw)
        candidate_id = str(row.get("candidate_id", ""))
        if candidate_id not in grid:
            raise ProtocolError(f"unknown development candidate: {candidate_id}")
        _required_finite(row, required)
        normalized.append(row)
    stable = _plateau_ids(normalized, protocol)
    eligible = [
        row
        for row in normalized
        if bool(row.get("eligible")) and str(row["candidate_id"]) in stable
    ]
    if not eligible:
        raise ProtocolError("no eligible candidate belongs to a stable plateau")
    if int(protocol.get("schema_version", 1)) >= 2:
        eligible.sort(
            key=lambda row: (
                float(row["worst_event_normalized_max_abs_error"]),
                float(row["mean_event_normalized_mean_abs_error"]),
                -float(row["confirmed_fsm_stance_iou"]),
                _candidate_distance(row, grid),
                str(row["candidate_id"]),
            )
        )
    else:
        eligible.sort(
            key=lambda row: (
                float(row["worst_event_normalized_max_abs_error"]),
                float(row["mean_event_normalized_mean_abs_error"]),
                -float(row["sensor_contact_stance_iou"]),
                -float(row["confirmed_fsm_stance_iou"]),
                _candidate_distance(row, grid),
                str(row["candidate_id"]),
            )
        )
    selected = dict(eligible[0])
    selected["stable_plateau_candidate_ids"] = sorted(stable)
    return selected


def evaluate_holdout_gate(
    candidate: Mapping[str, Any],
    baseline: Mapping[str, Any],
    protocol: Mapping[str, Any],
) -> dict[str, Any]:
    gate = protocol["sealed_validation_gate"]
    required_candidate_fields = {
        "candidate_id",
        "exact_reference_and_detector_event_counts",
        "precision",
        "recall",
        "max_abs_hs_error_s",
        "max_abs_toe_off_error_s",
        "invalid_or_timeout_transition_count",
        "unaccepted_sensor_gait_event_count",
        "exact_hs_to_toe_off_to_hs_order_and_cycle_count",
        "confirmation_latency_in_range",
        "forbidden_phase_mismatch_count",
        "unknown_fsm_phase_samples",
        "confirmed_fsm_stance_f1",
        "confirmed_fsm_stance_iou",
        "worst_event_normalized_max_abs_error",
    }
    required_baseline_fields = {
        "candidate_id",
        "confirmed_fsm_stance_iou",
        "worst_event_normalized_max_abs_error",
    }
    missing_candidate = sorted(required_candidate_fields - set(candidate))
    missing_baseline = sorted(required_baseline_fields - set(baseline))
    maximum_timing_regression = float(
        gate.get(
            "maximum_confirmed_time_worst_timing_regression_vs_baseline",
            gate.get("maximum_worst_timing_regression_vs_baseline", 0.0),
        )
    )
    checks = {
        "required_metrics_present": not missing_candidate and not missing_baseline,
        "exact_event_counts": bool(candidate.get("exact_reference_and_detector_event_counts", False)),
        "precision": float(candidate.get("precision", float("nan"))) >= float(gate["precision"]),
        "recall": float(candidate.get("recall", float("nan"))) >= float(gate["recall"]),
        "hs_timing": float(candidate.get("max_abs_hs_error_s", float("inf"))) <= float(gate["max_abs_hs_error_s"]) + 1e-12,
        "toe_off_timing": float(candidate.get("max_abs_toe_off_error_s", float("inf"))) <= float(gate["max_abs_toe_off_error_s"]) + 1e-12,
        "no_invalid_or_timeout": int(candidate.get("invalid_or_timeout_transition_count", -1)) == 0,
        "no_unaccepted_sensor_events": int(candidate.get("unaccepted_sensor_gait_event_count", -1)) == 0,
        "ordered_cycles": bool(candidate.get("exact_hs_to_toe_off_to_hs_order_and_cycle_count", False)),
        "confirmation_latency": bool(candidate.get("confirmation_latency_in_range", False)),
        "no_forbidden_phase_mismatch": int(candidate.get("forbidden_phase_mismatch_count", -1)) == 0,
        "no_unknown_phase": int(candidate.get("unknown_fsm_phase_samples", -1)) == 0,
        "fsm_f1": float(candidate.get("confirmed_fsm_stance_f1", float("nan"))) >= float(gate["minimum_confirmed_fsm_stance_f1"]),
        "fsm_iou": float(candidate.get("confirmed_fsm_stance_iou", float("nan"))) >= float(gate["minimum_confirmed_fsm_stance_iou"]),
        "fsm_iou_vs_baseline": float(candidate.get("confirmed_fsm_stance_iou", float("-inf"))) + float(gate["maximum_confirmed_fsm_iou_regression_vs_baseline"]) >= float(baseline.get("confirmed_fsm_stance_iou", float("inf"))),
        "timing_vs_baseline": float(candidate.get("worst_event_normalized_max_abs_error", float("inf"))) <= float(baseline.get("worst_event_normalized_max_abs_error", float("-inf"))) + maximum_timing_regression,
    }
    if int(protocol.get("schema_version", 1)) >= 2:
        expected_time_field = str(
            protocol.get(
                "_primary_event_time_field",
                protocol["replay"]["primary_event_time_field"],
            )
        )
        checks["primary_event_time_field"] = bool(
            candidate.get("primary_event_time_field") == expected_time_field
            and baseline.get("primary_event_time_field")
            == expected_time_field
        )
    return {
        "ok": bool(all(checks.values())),
        "checks": checks,
        "candidate_id": str(candidate.get("candidate_id", "")),
        "baseline_id": str(baseline.get("candidate_id", "")),
        "missing_candidate_fields": missing_candidate,
        "missing_baseline_fields": missing_baseline,
    }


def validate_stage_manifest(
    manifest: Mapping[str, Any],
    *,
    expected_stage: str,
    protocol_sha256: str,
    selected_candidate_id: str,
    prerequisite_sha256: str,
) -> dict[str, Any]:
    if expected_stage not in {"validation", "sealed"}:
        raise ProtocolError("lineage validation applies only to holdout stages")
    expected_prerequisite = "development" if expected_stage == "validation" else "validation"
    checks = {
        "schema_version": manifest.get("schema_version") == 1,
        "stage": manifest.get("stage") == expected_stage,
        "protocol_sha256": manifest.get("protocol_sha256") == protocol_sha256,
        "selected_candidate_id": manifest.get("selected_candidate_id") == selected_candidate_id,
        "prerequisite_stage": manifest.get("prerequisite_stage") == expected_prerequisite,
        "prerequisite_status": manifest.get("prerequisite_status") == "PASS",
        "prerequisite_manifest_sha256": manifest.get("prerequisite_manifest_sha256") == prerequisite_sha256,
    }
    candidate_ids = set(str(value) for value in manifest.get("candidate_ids", []))
    checks["candidate_scope"] = candidate_ids == {selected_candidate_id, BASELINE_ID}
    failed = sorted(name for name, value in checks.items() if not value)
    if failed:
        raise ProtocolError(f"invalid {expected_stage} manifest lineage: {failed}")
    return dict(manifest)


def _stage_block(protocol: Mapping[str, Any], stage: str) -> tuple[float, float]:
    key = {
        "development": "development_time_block_s",
        "validation": "validation_time_block_s",
        "sealed": "sealed_time_block_s",
    }[stage]
    start, end = protocol["chronological_split"][key]
    return float(start), float(end)


def _validate_development_prerequisite(
    manifest: Mapping[str, Any],
    protocol: Mapping[str, Any],
) -> str:
    """Reconstruct a development lock before any holdout is opened."""
    required_checks = {
        "schema_version": manifest.get("schema_version") == 1,
        "stage": manifest.get("stage") == "development",
        "status": manifest.get("status") == "PASS",
        "ok": manifest.get("ok") is True,
        "protocol_id": manifest.get("protocol_id") == protocol["protocol_id"],
        "protocol_sha256": manifest.get("protocol_sha256")
        == protocol["_protocol_sha256"],
        "policy_or_training_used": manifest.get("policy_or_training_used")
        is False,
        "runtime_configuration_modified": manifest.get(
            "runtime_configuration_modified"
        )
        is False,
    }
    failed = sorted(name for name, ok in required_checks.items() if not ok)
    if failed:
        raise ProtocolError(f"invalid development prerequisite: {failed}")

    minimum_cycles = int(
        protocol["chronological_split"]["minimum_development_cycles"]
    )
    if int(manifest.get("reference_cycle_count", -1)) < minimum_cycles:
        raise ProtocolError("development prerequisite has too few cycles")
    if int(protocol.get("schema_version", 1)) >= 2:
        if manifest.get("primary_event_time_field") != protocol.get(
            "_primary_event_time_field"
        ):
            raise ProtocolError("development primary event timestamp drift")
        sensitivity_gate = manifest.get("sensitivity_1ms_gate")
        if (
            manifest.get("sensitivity_1ms_required_for_freeze") is not True
            or not isinstance(sensitivity_gate, Mapping)
            or sensitivity_gate.get("ok") is not True
        ):
            raise ProtocolError("development 1 ms sensitivity did not pass")

    expected_grid = {
        item.candidate_id: item for item in build_candidate_grid(protocol)
    }
    candidate_ids = [str(value) for value in manifest.get("candidate_ids", [])]
    if (
        len(candidate_ids) != len(expected_grid)
        or len(set(candidate_ids)) != len(candidate_ids)
        or set(candidate_ids) != set(expected_grid)
    ):
        raise ProtocolError("development prerequisite does not contain exact grid")
    rows = manifest.get("primary_candidate_rows")
    if not isinstance(rows, list) or len(rows) != len(expected_grid):
        raise ProtocolError("development prerequisite candidate rows are incomplete")
    row_ids = [str(row.get("candidate_id", "")) for row in rows]
    if len(set(row_ids)) != len(row_ids) or set(row_ids) != set(expected_grid):
        raise ProtocolError("development prerequisite row IDs are invalid")
    for row in rows:
        candidate = expected_grid[str(row["candidate_id"])]
        if (
            _finite_number(row.get("on_threshold_n"), "row ON threshold")
            != candidate.on_threshold_n
            or _finite_number(row.get("off_threshold_n"), "row OFF threshold")
            != candidate.off_threshold_n
        ):
            raise ProtocolError("development candidate threshold drift")

    reconstructed = select_development_candidate(rows, protocol)
    selected_id = str(manifest.get("selected_candidate_id", ""))
    recorded_selection = manifest.get("development_selection")
    if (
        not selected_id
        or reconstructed["candidate_id"] != selected_id
        or not isinstance(recorded_selection, Mapping)
        or str(recorded_selection.get("candidate_id", "")) != selected_id
    ):
        raise ProtocolError("development selection cannot be reconstructed")
    selected = expected_grid[selected_id]
    thresholds = manifest.get("selected_thresholds_n")
    if not isinstance(thresholds, Mapping) or (
        _finite_number(thresholds.get("on"), "selected ON")
        != selected.on_threshold_n
        or _finite_number(thresholds.get("off"), "selected OFF")
        != selected.off_threshold_n
        or _finite_number(thresholds.get("dwell_s"), "selected dwell")
        != float(protocol["replay"]["sensor_dwell_s"])
    ):
        raise ProtocolError("development selected thresholds are inconsistent")
    return selected_id


def _load_prerequisite(
    protocol: Mapping[str, Any],
    stage: str,
    prerequisite_path: Path,
) -> tuple[dict[str, Any], Candidate, str]:
    prerequisite = json.loads(prerequisite_path.read_text(encoding="utf-8"))
    prerequisite_sha = _sha256(prerequisite_path)
    expected_stage = "development" if stage == "validation" else "validation"
    if prerequisite.get("stage") != expected_stage or prerequisite.get("status") != "PASS":
        raise ProtocolError(f"{stage} requires a passing {expected_stage} manifest")
    if prerequisite.get("protocol_sha256") != protocol["_protocol_sha256"]:
        raise ProtocolError("prerequisite protocol hash mismatch")
    selected_id = str(prerequisite.get("selected_candidate_id", ""))
    if stage == "validation":
        selected_id = _validate_development_prerequisite(
            prerequisite,
            protocol,
        )
    if stage == "sealed":
        development_path = prerequisite.get("development_manifest_path")
        development_sha = prerequisite.get("development_manifest_sha256")
        if not development_path or not development_sha:
            raise ProtocolError("validation manifest lacks development lineage")
        development_resolved = resolve_repo_path(str(development_path)).resolve()
        if _sha256(development_resolved) != development_sha:
            raise ProtocolError("development manifest changed after validation")
    grid = {item.candidate_id: item for item in build_candidate_grid(protocol)}
    if selected_id not in grid:
        raise ProtocolError(f"unknown selected candidate: {selected_id}")
    return prerequisite, grid[selected_id], prerequisite_sha


def _prepare_inputs(
    protocol: Mapping[str, Any],
    stage: str,
    *,
    sample_dt_s: float,
) -> dict[str, Any]:
    start_s, block_end_s = _stage_block(protocol, stage)
    setup = replace(
        read_setup_xml(resolve_repo_path(DEFAULT_SETUP)),
        t_start=start_s,
        t_end=block_end_s,
    )
    replay_cfg = protocol["replay"]
    reference_events, reference_provenance = _reference_events_from_prescribed_grf(
        setup,
        threshold_n=float(replay_cfg["prescribed_contact_threshold_n"]),
        min_contact_duration_s=float(replay_cfg["reference_min_contact_duration_s"]),
        min_cycle_duration_s=float(replay_cfg["reference_min_cycle_duration_s"]),
    )
    reference_hs_count = int(len(reference_events["heel_strike"]))
    reference_cycle_count = int(len(reference_events["toe_off"]))
    minimum_cycle_key = f"minimum_{stage}_cycles"
    minimum_cycles = int(protocol["chronological_split"][minimum_cycle_key])
    if reference_hs_count != reference_cycle_count + 1:
        raise ProtocolError(
            f"{stage} reference does not form complete HS/TO/.../HS cycles"
        )
    if reference_cycle_count < minimum_cycles:
        raise ProtocolError(
            f"{stage} has {reference_cycle_count} complete cycles; "
            f"minimum is {minimum_cycles}"
        )
    # Stop just after confirmation of the right-boundary HS.  This preserves
    # that event while preventing an incomplete following cycle from producing
    # an unmatched TO near an arbitrary split boundary.
    last_complete_hs_s = float(reference_events["heel_strike"][-1])
    effective_end_s = min(
        block_end_s,
        last_complete_hs_s + float(replay_cfg["sensor_dwell_s"]) + sample_dt_s,
    )
    setup = replace(setup, t_end=effective_end_s)
    times = np.arange(
        start_s,
        effective_end_s + 0.25 * sample_dt_s,
        sample_dt_s,
        dtype=float,
    )
    profile = load_online_grf_profile(resolve_repo_path(DEFAULT_PROFILE))
    samples = _sample_spheres(
        setup,
        profile,
        times,
        str(protocol["replay"]["sea_plugin"]),
    )
    loads, penetrations, aggregate = _regional_loads_and_penetrations(profile, samples)
    kinematics = _prescribed_prosthetic_kinematics(setup, times)
    prescribed_vertical_n = np.asarray(_external_grf(setup, times)["left"][:, 1], dtype=float)
    return {
        "setup": setup,
        "profile": profile,
        "times": times,
        "loads": loads,
        "penetrations": penetrations,
        "aggregate": aggregate,
        "kinematics": kinematics,
        "prescribed_vertical_n": prescribed_vertical_n,
        "reference_events": reference_events,
        "reference_provenance": reference_provenance,
        "body_weight_n": _model_body_weight_n(setup.model_file),
        "block_time_range_s": [start_s, block_end_s],
        "effective_replay_time_range_s": [float(times[0]), float(times[-1])],
    }


def _accepted_events_for_time_field(
    accepted: Sequence[Mapping[str, Any]],
    time_field: str,
) -> dict[str, np.ndarray]:
    """Return gate transitions at one explicit finite timestamp, fail-closed."""
    if time_field not in EVENT_TIME_FIELDS:
        raise ProtocolError(f"unsupported event timestamp field: {time_field}")
    transitions = _accepted_gait_transitions_for_gate(accepted)
    result: dict[str, np.ndarray] = {}
    for event in ("heel_strike", "toe_off"):
        values: list[float] = []
        for index, item in enumerate(transitions[event]):
            if time_field not in item:
                raise ProtocolError(
                    f"{event}[{index}] is missing {time_field}"
                )
            value = _finite_number(
                item[time_field], f"{event}[{index}].{time_field}"
            )
            values.append(value)
        result[event] = np.asarray(values, dtype=float)
    return result


def _exact_event_order(
    accepted: Sequence[Mapping[str, Any]],
    expected_cycles: int,
    *,
    time_field: str = "event_time_s",
) -> bool:
    transitions = _accepted_gait_transitions_for_gate(accepted)
    ordered = sorted(
        [dict(item) for event in ("heel_strike", "toe_off") for item in transitions[event]],
        key=lambda item: _finite_number(
            item.get(time_field), f"accepted transition {time_field}"
        ),
    )
    expected = ["heel_strike"]
    for _ in range(expected_cycles):
        expected.extend(("toe_off", "heel_strike"))
    return [str(item["event"]) for item in ordered] == expected


def _confirmation_latency_gate(
    accepted: Sequence[Mapping[str, Any]],
    *,
    dwell_s: float,
    sample_dt_s: float,
) -> dict[str, Any]:
    transitions = _accepted_gait_transitions_for_gate(accepted)
    latencies = [
        float(item["confirmed_time_s"]) - float(item["event_time_s"])
        for event in ("heel_strike", "toe_off")
        for item in transitions[event]
    ]
    low = dwell_s - 1e-9
    high = dwell_s + sample_dt_s + 1e-9
    return {
        "ok": bool(latencies and all(low <= value <= high for value in latencies)),
        "minimum_s": float(min(latencies)) if latencies else None,
        "maximum_s": float(max(latencies)) if latencies else None,
        "required_range_s": [float(dwell_s), float(dwell_s + sample_dt_s)],
        "values_s": latencies,
    }


def _evaluate_candidate(
    candidate: Candidate,
    inputs: Mapping[str, Any],
    protocol: Mapping[str, Any],
    *,
    sample_dt_s: float,
) -> tuple[dict[str, Any], dict[str, Any]]:
    runtime_cfg = _current_runtime_fsm_config()
    fsm_cfg = replace(
        runtime_cfg,
        sensor_on_threshold_n=float(candidate.on_threshold_n),
        sensor_off_threshold_n=float(candidate.off_threshold_n),
        sensor_dwell_s=float(protocol["replay"]["sensor_dwell_s"]),
    )
    replay = _run_production_fsm(
        inputs["times"],
        inputs["loads"],
        inputs["penetrations"],
        inputs["aggregate"],
        inputs["kinematics"],
        body_weight_n=float(inputs["body_weight_n"]),
        fsm_config=fsm_cfg,
    )
    reference_events = inputs["reference_events"]
    primary_time_field = str(
        protocol.get(
            "_primary_event_time_field",
            protocol["replay"].get(
                "primary_event_time_field", "event_time_s"
            ),
        )
    )
    diagnostic_time_field = str(
        protocol.get(
            "_diagnostic_event_time_field",
            protocol["replay"].get(
                "diagnostic_event_time_field", "confirmed_time_s"
            ),
        )
    )
    events_by_time_field = {
        field: _accepted_events_for_time_field(replay["accepted"], field)
        for field in EVENT_TIME_FIELDS
    }
    primary_events = events_by_time_field[primary_time_field]
    diagnostic_events = events_by_time_field[diagnostic_time_field]
    onset_events = events_by_time_field["event_time_s"]
    hs_tolerance = float(protocol["replay"]["hs_tolerance_s"])
    to_tolerance = float(protocol["replay"]["toe_off_tolerance_s"])
    event_metrics = {
        "heel_strike": match_events(
            reference_events["heel_strike"],
            primary_events["heel_strike"],
            hs_tolerance,
        ),
        "toe_off": match_events(
            reference_events["toe_off"],
            primary_events["toe_off"],
            to_tolerance,
        ),
    }
    ordered = {
        "heel_strike": _ordered_event_diagnostic(
            reference_events["heel_strike"],
            primary_events["heel_strike"],
            hs_tolerance,
        ),
        "toe_off": _ordered_event_diagnostic(
            reference_events["toe_off"],
            primary_events["toe_off"],
            to_tolerance,
        ),
    }
    diagnostic_ordered = {
        "heel_strike": _ordered_event_diagnostic(
            reference_events["heel_strike"],
            diagnostic_events["heel_strike"],
            hs_tolerance,
        ),
        "toe_off": _ordered_event_diagnostic(
            reference_events["toe_off"],
            diagnostic_events["toe_off"],
            to_tolerance,
        ),
    }
    semantic = _semantic_gate(
        replay,
        inputs["times"],
        inputs["loads"],
        sensor_on_threshold_n=float(candidate.on_threshold_n),
        expected_complete_cycles=int(len(reference_events["toe_off"])),
    )
    phase = _phase_classification_gate(
        inputs["times"],
        inputs["prescribed_vertical_n"],
        reference_events,
        primary_events,
        replay,
        prescribed_threshold_n=float(protocol["replay"]["prescribed_contact_threshold_n"]),
        hs_tolerance_s=hs_tolerance,
        to_tolerance_s=to_tolerance,
        sensor_dwell_s=float(fsm_cfg.sensor_dwell_s),
        reference_phase_mode=str(
            protocol.get(
                "_phase_reference_mode",
                protocol["replay"].get(
                    "phase_reference_mode", "instantaneous_grf"
                ),
            )
        ),
        primary_event_time_field=primary_time_field,
        onset_events=onset_events,
    )
    phase_arrays = phase["_arrays"]
    strict_mask = np.asarray(phase_arrays["strict_mask"], dtype=bool)
    reference_contact = np.asarray(phase_arrays["reference_phase"], dtype=int) == PHASE_STANCE
    fsm_phase = _fsm_phase_from_state_id(np.asarray(replay["state_id"], dtype=float))
    fsm_contact = fsm_phase == PHASE_STANCE
    sensor_union = (np.asarray(replay["heel_contact"]) > 0.5) | (np.asarray(replay["toe_contact"]) > 0.5)
    fsm_contact_metrics = binary_metrics(reference_contact[strict_mask], fsm_contact[strict_mask])
    sensor_contact_metrics = binary_metrics(reference_contact[strict_mask], sensor_union[strict_mask])
    legacy_signal = smooth_force(inputs["times"], inputs["aggregate"], 0.1) > 15.0
    legacy_contact_metrics = binary_metrics(reference_contact[strict_mask], legacy_signal[strict_mask])
    latency = _confirmation_latency_gate(
        replay["accepted"],
        dwell_s=float(fsm_cfg.sensor_dwell_s),
        sample_dt_s=float(sample_dt_s),
    )
    exact_order = _exact_event_order(
        replay["accepted"],
        len(reference_events["toe_off"]),
        time_field=primary_time_field,
    )
    timing_ok = strict_event_pass(
        event_metrics,
        hs_tolerance_s=hs_tolerance,
        to_tolerance_s=to_tolerance,
    ) and all(bool(item["all_within_tolerance"]) for item in ordered.values())
    hs_max_raw = float(ordered["heel_strike"]["timing_max_abs_s"])
    to_max_raw = float(ordered["toe_off"]["timing_max_abs_s"])
    # Unequal counts intentionally yield NaN in the diagnostic helper.  Keep
    # those candidates reportable and ineligible instead of aborting the whole
    # frozen sweep because one grid point has a missing/extra event.
    hs_max = hs_max_raw if math.isfinite(hs_max_raw) else 999.0
    to_max = to_max_raw if math.isfinite(to_max_raw) else 999.0
    ordered_errors = [
        abs(float(value)) / tolerance
        for name, tolerance in (("heel_strike", hs_tolerance), ("toe_off", to_tolerance))
        for value in ordered[name]["ordered_errors_s"]
    ]
    worst_normalized = max(hs_max / hs_tolerance, to_max / to_tolerance)
    mean_normalized = float(np.mean(ordered_errors)) if ordered_errors else 1.0e9
    final_payload = replay["fsm"].payload()
    invalid_or_timeout = len(replay["invalid_steps"]) + len(semantic["timeout_transitions"])
    unknown_samples = int(phase["settled_outside_transition_windows_agreement"]["unknown_fsm_samples"])
    exact_counts = bool(
        ordered["heel_strike"]["equal_counts"]
        and ordered["toe_off"]["equal_counts"]
    )
    eligible = bool(
        timing_ok
        and semantic["ok"]
        and phase["ok"]
        and latency["ok"]
        and exact_order
        and exact_counts
        and unknown_samples == 0
    )
    row = {
        "candidate_id": candidate.candidate_id,
        "on_threshold_n": float(candidate.on_threshold_n),
        "off_threshold_n": float(candidate.off_threshold_n),
        "sample_dt_s": float(sample_dt_s),
        "primary_event_time_field": primary_time_field,
        "diagnostic_event_time_field": diagnostic_time_field,
        "eligible": eligible,
        "exact_reference_and_detector_event_counts": exact_counts,
        "precision": float(min(event_metrics["heel_strike"]["precision"], event_metrics["toe_off"]["precision"])),
        "recall": float(min(event_metrics["heel_strike"]["recall"], event_metrics["toe_off"]["recall"])),
        "max_abs_hs_error_s": hs_max,
        "max_abs_toe_off_error_s": to_max,
        f"diagnostic_max_abs_{diagnostic_time_field}_hs_error_s": float(
            diagnostic_ordered["heel_strike"]["timing_max_abs_s"]
        ),
        f"diagnostic_max_abs_{diagnostic_time_field}_toe_off_error_s": float(
            diagnostic_ordered["toe_off"]["timing_max_abs_s"]
        ),
        "worst_event_normalized_max_abs_error": float(worst_normalized),
        "mean_event_normalized_mean_abs_error": mean_normalized,
        "invalid_or_timeout_transition_count": int(invalid_or_timeout),
        "unaccepted_sensor_gait_event_count": int(len(semantic["unaccepted_sensor_gait_events"])),
        "exact_hs_to_toe_off_to_hs_order_and_cycle_count": exact_order,
        "confirmation_latency_in_range": bool(latency["ok"]),
        "forbidden_phase_mismatch_count": int(phase["forbidden_mismatch_count"]),
        "unknown_fsm_phase_samples": unknown_samples,
        "sensor_contact_stance_f1": float(sensor_contact_metrics["f1"]),
        "sensor_contact_stance_iou": float(sensor_contact_metrics["iou"]),
        "confirmed_fsm_stance_f1": float(fsm_contact_metrics["f1"]),
        "confirmed_fsm_stance_iou": float(fsm_contact_metrics["iou"]),
        "legacy_compatible_aggregate_contact_f1": float(legacy_contact_metrics["f1"]),
        "legacy_compatible_aggregate_contact_iou": float(legacy_contact_metrics["iou"]),
        "reference_hs_count": int(len(reference_events["heel_strike"])),
        "reference_to_count": int(len(reference_events["toe_off"])),
        "predicted_hs_count": int(len(primary_events["heel_strike"])),
        "predicted_to_count": int(len(primary_events["toe_off"])),
        "observed_valid_cycle_count": int(final_payload["valid_cycle_count"]),
    }
    detail = {
        "row": row,
        "primary_event_time_field": primary_time_field,
        "diagnostic_event_time_field": diagnostic_time_field,
        "primary_event_metrics": event_metrics,
        "primary_event_diagnostics": ordered,
        "diagnostic_event_diagnostics_not_gate": diagnostic_ordered,
        # Backward-compatible aliases: they always point to the primary gate.
        "event_metrics": event_metrics,
        "ordered_event_diagnostics": ordered,
        "semantic_gate": semantic,
        "phase_state_validation": {key: value for key, value in phase.items() if key != "_arrays"},
        "confirmation_latency": latency,
        "contact_metrics": {
            "confirmed_fsm_stance": fsm_contact_metrics,
            "stable_sensor_union": sensor_contact_metrics,
            "legacy_compatible_aggregate_smoothed": legacy_contact_metrics,
        },
        "events": {
            "reference": {key: values.tolist() for key, values in reference_events.items()},
            "primary": {key: values.tolist() for key, values in primary_events.items()},
            "diagnostic": {
                key: values.tolist() for key, values in diagnostic_events.items()
            },
            "onset": {key: values.tolist() for key, values in onset_events.items()},
            "confirmed": {
                key: values.tolist()
                for key, values in events_by_time_field["confirmed_time_s"].items()
            },
            "predicted": {key: values.tolist() for key, values in primary_events.items()},
        },
    }
    if diagnostic_time_field == "confirmed_time_s":
        detail["confirmed_time_diagnostics_not_primary_gate"] = (
            diagnostic_ordered
        )
        row["diagnostic_max_abs_confirmed_hs_error_s"] = float(
            diagnostic_ordered["heel_strike"]["timing_max_abs_s"]
        )
        row["diagnostic_max_abs_confirmed_toe_off_error_s"] = float(
            diagnostic_ordered["toe_off"]["timing_max_abs_s"]
        )
    if diagnostic_time_field == "event_time_s":
        row["diagnostic_onset_max_abs_hs_error_s"] = float(
            diagnostic_ordered["heel_strike"]["timing_max_abs_s"]
        )
        row["diagnostic_onset_max_abs_toe_off_error_s"] = float(
            diagnostic_ordered["toe_off"]["timing_max_abs_s"]
        )
    return row, detail


def _write_candidate_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    if not rows:
        return
    fieldnames = sorted({str(key) for row in rows for key in row})
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})


def _write_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(_json_safe(dict(payload)), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _plot_event_error_diagnostics(
    path: Path,
    detail: Mapping[str, Any],
) -> None:
    """Plot primary and diagnostic timestamp errors without mixing roles."""
    import matplotlib.pyplot as plt

    primary = detail["primary_event_diagnostics"]
    diagnostic = detail["diagnostic_event_diagnostics_not_gate"]
    primary_field = str(detail["primary_event_time_field"])
    diagnostic_field = str(detail["diagnostic_event_time_field"])
    figure, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=False)
    for axis, event, tolerance_ms in (
        (axes[0], "heel_strike", 50.0),
        (axes[1], "toe_off", 80.0),
    ):
        primary_ms = 1000.0 * np.asarray(
            primary[event]["ordered_errors_s"], dtype=float
        )
        diagnostic_ms = 1000.0 * np.asarray(
            diagnostic[event]["ordered_errors_s"], dtype=float
        )
        indices = np.arange(1, len(primary_ms) + 1)
        axis.plot(
            indices,
            primary_ms,
            "o-",
            label=f"primary gate: {primary_field}",
        )
        axis.plot(
            indices,
            diagnostic_ms,
            "s--",
            label=f"diagnostic only: {diagnostic_field}",
        )
        axis.axhline(tolerance_ms, color="tab:red", linestyle=":")
        axis.axhline(-tolerance_ms, color="tab:red", linestyle=":")
        axis.axhline(0.0, color="black", linewidth=0.8)
        axis.set_ylabel(f"{event} error [ms]")
        axis.set_xlabel("ordered prescribed event index")
        axis.grid(True, alpha=0.25)
        axis.legend(loc="best")
    figure.suptitle(
        "Two-sensor prescribed timing — primary and diagnostic timestamps "
        "remain decision-separated"
    )
    figure.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=170)
    plt.close(figure)


def _run_stage(
    protocol: dict[str, Any],
    stage: str,
    output_root: Path,
    prerequisite_path: Path | None,
) -> dict[str, Any]:
    if int(protocol.get("schema_version", 1)) >= 2 and stage == "sealed":
        raise ProtocolError(
            "V3 keeps the sealed block unevaluated; opening it requires a "
            "separately authorized successor protocol"
        )
    stage_dir = output_root / stage
    if stage_dir.exists() and any(stage_dir.iterdir()):
        raise FileExistsError(f"no-clobber stage directory is not empty: {stage_dir}")
    stage_dir.mkdir(parents=True, exist_ok=True)
    grid = build_candidate_grid(protocol)
    baseline = next(item for item in grid if item.candidate_id == BASELINE_ID)
    prerequisite: dict[str, Any] | None = None
    prerequisite_sha: str | None = None
    selected: Candidate | None = None
    if stage == "development":
        candidates = grid
    else:
        if prerequisite_path is None:
            raise ProtocolError(f"{stage} requires --prerequisite-manifest")
        prerequisite, selected, prerequisite_sha = _load_prerequisite(
            protocol,
            stage,
            prerequisite_path,
        )
        candidates = list({item.candidate_id: item for item in (selected, baseline)}.values())

    primary_dt = float(protocol["replay"]["primary_sample_dt_s"])
    primary_inputs = _prepare_inputs(protocol, stage, sample_dt_s=primary_dt)
    rows: list[dict[str, Any]] = []
    details: dict[str, Any] = {}
    for candidate in candidates:
        row, detail = _evaluate_candidate(
            candidate,
            primary_inputs,
            protocol,
            sample_dt_s=primary_dt,
        )
        rows.append(row)
        details[candidate.candidate_id] = detail
    _write_candidate_csv(stage_dir / "primary_candidate_metrics.csv", rows)

    selection: dict[str, Any] | None = None
    gate_result: dict[str, Any] | None = None
    sensitivity: dict[str, Any] = {}
    sensitivity_gate_result: dict[str, Any] | None = None
    selection_error: str | None = None
    if stage == "development":
        try:
            selection = select_development_candidate(rows, protocol)
        except ProtocolError as exc:
            # A scientifically valid negative development result is a FAIL
            # manifest, not an execution ERROR.  Persist every grid row so the
            # failure can be audited without reopening later data blocks.
            selection_error = str(exc)
            diagnostic_row = min(
                rows,
                key=lambda row: (
                    float(row["worst_event_normalized_max_abs_error"]),
                    float(row["mean_event_normalized_mean_abs_error"]),
                    -float(row["confirmed_fsm_stance_iou"]),
                    str(row["candidate_id"]),
                ),
            )
            selected = next(
                item
                for item in grid
                if item.candidate_id == diagnostic_row["candidate_id"]
            )
            selected_id = ""
            primary_ok = False
        else:
            selected_id = str(selection["candidate_id"])
            selected = next(item for item in grid if item.candidate_id == selected_id)
            primary_ok = bool(selection["eligible"])
    else:
        assert selected is not None
        selected_id = selected.candidate_id
        row_by_id = {str(row["candidate_id"]): row for row in rows}
        gate_result = evaluate_holdout_gate(
            row_by_id[selected_id],
            row_by_id[BASELINE_ID],
            protocol,
        )
        primary_ok = bool(gate_result["ok"])

    sensitivity_required = bool(
        stage != "development" or int(protocol.get("schema_version", 1)) >= 2
    )
    sensitivity_ok = not sensitivity_required
    # Sensitivity is opened only after primary PASS.  In V3 development the
    # primary ranking is already locked before this step, so 1 ms can only
    # confirm or reject that winner; it can never select a replacement.
    if primary_ok and sensitivity_required:
        assert selected is not None
        sensitivity_dt = float(protocol["replay"]["sensitivity_sample_dt_s"])
        sensitivity_inputs = _prepare_inputs(
            protocol,
            stage,
            sample_dt_s=sensitivity_dt,
        )
        sensitivity_candidates = list(
            {
                item.candidate_id: item
                for item in (selected, baseline)
            }.values()
        )
        for sensitivity_candidate in sensitivity_candidates:
            row, detail = _evaluate_candidate(
                sensitivity_candidate,
                sensitivity_inputs,
                protocol,
                sample_dt_s=sensitivity_dt,
            )
            sensitivity[sensitivity_candidate.candidate_id] = detail
        sensitivity_rows = {
            candidate_id: detail["row"]
            for candidate_id, detail in sensitivity.items()
        }
        sensitivity_gate_result = evaluate_holdout_gate(
            sensitivity_rows[selected.candidate_id],
            sensitivity_rows[BASELINE_ID],
            protocol,
        )
        sensitivity_ok = bool(sensitivity_gate_result["ok"])
    status_ok = bool(primary_ok and sensitivity_ok)

    assert selected is not None
    selected_detail = details[selected.candidate_id]
    _plot(
        stage_dir / "selected_primary_event_timing.png",
        primary_inputs["times"],
        primary_inputs["prescribed_vertical_n"],
        primary_inputs["reference_events"],
        {key: np.asarray(value, dtype=float) for key, value in selected_detail["events"]["predicted"].items()},
        primary_inputs["loads"],
        # Plot helper needs the replay arrays, so recompute only the selected
        # candidate; this is cheap compared with the already cached geometry.
        _run_production_fsm(
            primary_inputs["times"],
            primary_inputs["loads"],
            primary_inputs["penetrations"],
            primary_inputs["aggregate"],
            primary_inputs["kinematics"],
            body_weight_n=float(primary_inputs["body_weight_n"]),
            fsm_config=replace(
                _current_runtime_fsm_config(),
                sensor_on_threshold_n=selected.on_threshold_n,
                sensor_off_threshold_n=selected.off_threshold_n,
                sensor_dwell_s=float(protocol["replay"]["sensor_dwell_s"]),
            ),
        ),
        prescribed_threshold_n=float(protocol["replay"]["prescribed_contact_threshold_n"]),
        sensor_on_threshold_n=selected.on_threshold_n,
        sensor_off_threshold_n=selected.off_threshold_n,
    )
    _plot_event_error_diagnostics(
        stage_dir / "diagnostic_or_selected_event_errors.png",
        selected_detail,
    )

    manifest: dict[str, Any] = {
        "schema_version": 1,
        "status": "PASS" if status_ok else "FAIL",
        "ok": status_ok,
        "stage": stage,
        "protocol_id": protocol["protocol_id"],
        "protocol_schema_version": int(protocol["schema_version"]),
        "protocol_path": _portable(Path(protocol["_protocol_path"])),
        "protocol_sha256": protocol["_protocol_sha256"],
        "primary_event_time_field": protocol["_primary_event_time_field"],
        "diagnostic_event_time_field": protocol[
            "_diagnostic_event_time_field"
        ],
        "phase_reference_mode": protocol["_phase_reference_mode"],
        "candidate_ids": [candidate.candidate_id for candidate in candidates],
        "selected_candidate_id": (
            selected.candidate_id if selection_error is None else None
        ),
        "selected_thresholds_n": (
            {
                "on": selected.on_threshold_n,
                "off": selected.off_threshold_n,
                "dwell_s": float(protocol["replay"]["sensor_dwell_s"]),
            }
            if selection_error is None
            else None
        ),
        "block_time_range_s": primary_inputs["block_time_range_s"],
        "effective_replay_time_range_s": primary_inputs["effective_replay_time_range_s"],
        "reference_cycle_count": int(len(primary_inputs["reference_events"]["toe_off"])),
        "primary_sample_dt_s": primary_dt,
        "primary_candidate_rows": rows,
        "primary_candidate_details": details,
        "development_selection": selection,
        "development_selection_error": selection_error,
        "diagnostic_plot_candidate_id": (
            selected.candidate_id if selection_error is not None else None
        ),
        "holdout_gate": gate_result,
        "sensitivity_1ms": sensitivity,
        "sensitivity_1ms_gate": sensitivity_gate_result,
        "sensitivity_1ms_required_for_freeze": sensitivity_required,
        "reference_provenance": primary_inputs["reference_provenance"],
        "artifacts": {
            "candidate_csv": _portable(stage_dir / "primary_candidate_metrics.csv"),
            "selected_plot": _portable(stage_dir / "selected_primary_event_timing.png"),
            "event_error_plot": _portable(
                stage_dir / "diagnostic_or_selected_event_errors.png"
            ),
        },
        "policy_or_training_used": False,
        "runtime_configuration_modified": False,
    }
    if prerequisite is not None and prerequisite_path is not None:
        manifest.update(
            {
                "prerequisite_stage": str(prerequisite["stage"]),
                "prerequisite_status": str(prerequisite["status"]),
                "prerequisite_manifest_path": _portable(prerequisite_path),
                "prerequisite_manifest_sha256": prerequisite_sha,
            }
        )
        if stage == "validation":
            manifest["development_manifest_path"] = _portable(prerequisite_path)
            manifest["development_manifest_sha256"] = prerequisite_sha
        else:
            manifest["development_manifest_path"] = prerequisite["development_manifest_path"]
            manifest["development_manifest_sha256"] = prerequisite["development_manifest_sha256"]
        validate_stage_manifest(
            manifest,
            expected_stage=stage,
            protocol_sha256=protocol["_protocol_sha256"],
            selected_candidate_id=selected.candidate_id,
            prerequisite_sha256=str(prerequisite_sha),
        )
    manifest_path = stage_dir / "manifest.json"
    _write_json(manifest_path, manifest)
    return manifest


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", required=True, choices=STAGES)
    parser.add_argument("--protocol", default=str(DEFAULT_PROTOCOL))
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT))
    parser.add_argument("--prerequisite-manifest", default="")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    output_root = resolve_repo_path(args.output_root).resolve()
    failure_path = output_root / str(args.stage) / "failure.json"
    try:
        protocol = load_and_validate_protocol(args.protocol)
        prerequisite = (
            resolve_repo_path(args.prerequisite_manifest).resolve()
            if args.prerequisite_manifest
            else None
        )
        manifest = _run_stage(protocol, str(args.stage), output_root, prerequisite)
        print(json.dumps({
            "stage": manifest["stage"],
            "status": manifest["status"],
            "selected_candidate_id": manifest["selected_candidate_id"],
            "reference_cycle_count": manifest["reference_cycle_count"],
            "manifest": _portable(output_root / str(args.stage) / "manifest.json"),
        }, indent=2))
        return 0 if manifest["ok"] else 2
    except Exception as exc:
        failure_path.parent.mkdir(parents=True, exist_ok=True)
        _write_json(
            failure_path,
            {
                "status": "ERROR",
                "stage": str(args.stage),
                "error_type": type(exc).__name__,
                "error": str(exc),
                "traceback": traceback.format_exc(),
            },
        )
        print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
