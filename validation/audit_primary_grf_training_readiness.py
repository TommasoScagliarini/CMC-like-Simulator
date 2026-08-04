"""Offline, fail-closed forensics for primary-GRF readiness steps 0.0--0.2.

The harness consumes only frozen protocols and already-existing artifacts.  It
does not launch Ray, a policy rollout, forward integration, or protected
validation/sealed trials.  Development evidence is the preregistered
``3 trials x 4 plateaus x 2 cadences`` IK matrix; trial-01 historical IK,
saved CoordinateStates, and the historical H0 rollout are diagnostic-only
streams.

Two input routes are implemented:

* canonical precomputed ``trace_csv`` + ``per_sphere_csv`` records for
  dependency-light API/contract tests; and
* an offline adapter over existing IK/CoordinateStates artifacts which reuses
  :mod:`validation.validate_online_grf` contact-law and sampling helpers.

Production preflight refuses the precomputed route because it cannot prove
that the frozen OpenSim profile was actually loaded.  The raw adapter imports
OpenSim lazily and never advances a simulation.  All scientific outputs are
immutable and no-clobber.
"""

from __future__ import annotations

import argparse
import csv
import ctypes
import errno
import hashlib
import json
import math
import os
import re
import shutil
import sys
import uuid
from dataclasses import dataclass, replace
from pathlib import Path, PurePosixPath, PureWindowsPath
from typing import Any, Callable, Iterable, Mapping, Sequence

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if __package__:
    from . import readiness_gatekeeper as gate
else:  # pragma: no cover - exercised by direct CLI use.
    repo_root_text = str(REPO_ROOT)
    if repo_root_text not in sys.path:
        sys.path.insert(0, repo_root_text)
    from validation import readiness_gatekeeper as gate


SCHEMA_VERSION = 1
STEP_ID = "0"
AUTHORIZED_TRIALS = ("02", "04", "08")
PROTECTED_TRIALS = frozenset({"03", "05", "06", "07"})
CADENCES_S = (0.001, 0.010)
_TIME_GRID_ATOL_S = 1e-10
INPUT_MODES = frozenset(
    {"ik_prescribed", "coordinate_states", "h0_historical"}
)
PRIMARY_SIDE = "left"
CROSSING_THRESHOLDS_N = (0.5, 5.0, 20.0)
ORACLE_THRESHOLD_N = 20.0
ORACLE_EVENT_MIN_CONTACT_DURATION_S = 0.05
ORACLE_EVENT_MIN_CYCLE_DURATION_S = 0.30
ORACLE_EVENT_PAIRED_CADENCE_RIGHT_MARGIN_S = max(CADENCES_S)
OUTPUT_FILENAMES = (
    "summary.json",
    "trace.csv",
    "events.csv",
    "per_sphere.csv",
    "root_cause.json",
    "timing_plot.png",
    "penetration_plot.png",
)
_WINDOWS_RESERVED = frozenset(
    {"CON", "PRN", "AUX", "NUL"}
    | {f"COM{index}" for index in range(1, 10)}
    | {f"LPT{index}" for index in range(1, 10)}
)
_ALLOCATION = {
    "03": "PRIMARY_VALIDATION_ONE_SHOT",
    "05": "DETECTOR_VALIDATION_ONE_SHOT",
    "06": "DETECTOR_SEALED_ONE_SHOT",
    "07": "PRIMARY_SEALED_ONE_SHOT",
}
_CANONICAL_GLOBAL_AUDIT = (
    "validation/global_data_access_audit_2026-07-23.json"
)
_CANONICAL_GLOBAL_AUDIT_SHA256 = (
    "296f7fed463fff07a0125f12b0bbbc8dcff2176cb02f406d6437bd88380ab694"
)
_REQUIRED_STATIC_PROTOCOL_SOURCES = {
    "online_grf_python": "online_grf.py",
    "model_loader": "model_loader.py",
    "simulation_runner": "simulation_runner.py",
    "trajectory_adapter": "Trajectory Generator/osim_trj_cmc_like.py",
    "gatekeeper": "validation/readiness_gatekeeper.py",
    "gatekeeper_tests": "validation/test_readiness_gatekeeper.py",
    "primary_audit": "validation/audit_primary_grf_training_readiness.py",
    "primary_audit_tests": "validation/test_primary_grf_training_readiness.py",
    "online_grf_validator": "validation/validate_online_grf.py",
    "online_grf_event_matching": "validation/validate_online_grf_events.py",
    "forward_state_grid": "validation/validate_two_sensor_forward_states.py",
    "plantar_mesh_audit": "validation/audit_two_sensor_prescribed_geometry.py",
    "setup_io": "setup_io.py",
    "output_io": "output.py",
    "kinematics_interpolator": "kinematics_interpolator.py",
    "simulator_config": "config.py",
    "path_resolver": "path_resolver.py",
}
_REQUIRED_DYNAMIC_PROTOCOL_SOURCES = frozenset(
    {"model", "plantar_mesh", "reserve_actuators", "online_grf_plugin"}
)


class PrimaryAuditError(ValueError):
    """Base class for a fail-closed primary-GRF audit refusal."""


class PreflightError(PrimaryAuditError):
    """Raised before any semantic unit trace is decoded."""


class TraceContractError(PrimaryAuditError):
    """Raised when an offline trace is absent, ambiguous, or non-finite."""


@dataclass(frozen=True)
class SphereSeries:
    name: str
    side: str
    normal_force_n: np.ndarray
    penetration_m: np.ndarray
    clearance_m: np.ndarray
    center_height_m: np.ndarray
    slip_speed_m_s: np.ndarray


@dataclass(frozen=True)
class SideSeries:
    oracle_force_n: np.ndarray
    oracle_cop_m: np.ndarray
    oracle_moment_nm: np.ndarray
    primary_force_n: np.ndarray
    primary_normal_force_n: np.ndarray
    primary_cop_m: np.ndarray
    primary_moment_nm: np.ndarray
    primary_penetration_m: np.ndarray
    primary_slip_speed_m_s: np.ndarray
    primary_in_contact: np.ndarray
    mesh_min_clearance_m: np.ndarray | None


@dataclass(frozen=True)
class UnitTrace:
    unit_id: str
    trial_id: str
    plateau_id: str
    cadence_s: float
    input_mode: str
    evidence_role: str
    start_s: float
    end_s: float
    times_s: np.ndarray
    sides: Mapping[str, SideSeries]
    spheres: tuple[SphereSeries, ...]
    detector_events: tuple[Mapping[str, Any], ...] = ()
    reference_unit_id: str | None = None
    plateau_speed_mps: float | None = None
    surface_velocity_mps: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class PreflightContext:
    repo_root: Path
    protocol_path: Path
    protocol_sha256: str
    protocol: Mapping[str, Any]
    audit_path: Path
    audit_sha256: str
    ledger_path: Path
    ledger_sha256: str
    receipt_path: Path
    receipt_sha256: str
    output_dir: Path
    primary_profile_path: Path
    primary_profile_record: Mapping[str, Any]
    authorized_units: tuple[Mapping[str, Any], ...]
    diagnostic_units: tuple[Mapping[str, Any], ...]


def _strict_object(path: Path, label: str) -> dict[str, Any]:
    value = gate.load_json_strict(path)
    if not isinstance(value, Mapping):
        raise PreflightError(f"{label} must contain a JSON object: {path}")
    return dict(value)


def safe_protocol_relative_path(value: Any, *, label: str) -> PurePosixPath:
    """Validate a portable relative path, including Windows device/ADS rules."""

    if not isinstance(value, str) or not value.strip():
        raise PreflightError(f"{label} must be a non-empty path string")
    if value != value.strip():
        raise PreflightError(f"{label} has leading or trailing whitespace")
    if "\\" in value:
        raise PreflightError(f"{label} must use portable '/' separators")
    windows = PureWindowsPath(value)
    if (
        PurePosixPath(value).is_absolute()
        or windows.is_absolute()
        or bool(windows.drive)
    ):
        raise PreflightError(f"{label} must be repository-relative")
    if ":" in value:
        raise PreflightError(f"{label} contains a forbidden Windows ADS separator")
    raw_parts = value.split("/")
    if any(part in {"", ".", ".."} for part in raw_parts):
        raise PreflightError(f"{label} contains a non-canonical path component")
    path = PurePosixPath(value)
    if not path.parts or any(part in {"", ".", ".."} for part in path.parts):
        raise PreflightError(f"{label} contains an unsafe path component")
    for part in path.parts:
        if part.endswith((" ", ".")):
            raise PreflightError(
                f"{label} contains a Windows-ambiguous trailing character"
            )
        stem = part.split(".", 1)[0].upper()
        if stem in _WINDOWS_RESERVED:
            raise PreflightError(
                f"{label} contains reserved Windows device name {part!r}"
            )
        if any(ord(character) < 32 for character in part):
            raise PreflightError(f"{label} contains a control character")
    return path


def _resolve_safe_record(
    record: Any,
    repo_root: Path,
    *,
    label: str,
) -> Path:
    if not isinstance(record, Mapping):
        raise PreflightError(f"{label} must be a path/hash record")
    safe_protocol_relative_path(record.get("path"), label=f"{label}.path")
    try:
        return gate.verify_file_record(record, repo_root, label=label)
    except gate.GatekeeperError as exc:
        raise PreflightError(str(exc)) from exc


def _actual_record(path: Path, repo_root: Path) -> dict[str, Any]:
    try:
        return gate.source_record(path, repo_root)
    except gate.GatekeeperError as exc:
        raise PreflightError(str(exc)) from exc


def _require_exact_record(
    payload: Mapping[str, Any],
    field: str,
    actual_path: Path,
    repo_root: Path,
    *,
    label: str,
) -> None:
    expected = payload.get(field)
    if not isinstance(expected, Mapping):
        raise PreflightError(f"{label}.{field} must be a path/hash record")
    _resolve_safe_record(expected, repo_root, label=f"{label}.{field}")
    actual = _actual_record(actual_path, repo_root)
    if (
        expected.get("path") != actual["path"]
        or expected.get("sha256") != actual["sha256"]
    ):
        raise PreflightError(
            f"{label}.{field} does not bind the supplied {field} file"
        )


def _validate_audit(audit: Mapping[str, Any], repo_root: Path) -> None:
    exact = {
        "schema_version": 1,
        "status": "PASS",
        "decision": "GLOBAL_DATA_BUDGET_ADOPTABLE",
        "source_hashes_match": True,
        "protected_outcome_access_found": False,
        "protected_run_authorized": False,
        "protected_conditions_metadata_semantic_access_started": True,
        "protected_component_outcome_semantic_access_started": False,
    }
    for field, expected in exact.items():
        if type(audit.get(field)) is not type(expected) or audit.get(field) != expected:
            raise PreflightError(
                f"global audit {field} must be exactly {expected!r}"
            )
    if "protected_semantic_access_started" in audit:
        raise PreflightError(
            "global audit contains obsolete ambiguous protected-access field"
        )
    if audit.get("failed_checks") != []:
        raise PreflightError("global audit failed_checks must be empty")
    source_records = audit.get("source_records")
    source_items = list(
        _iter_source_records(
            source_records,
            label="global_audit.source_records",
        )
    )
    if not source_items:
        raise PreflightError("global audit source_records must not be empty")
    for label, record in source_items:
        _resolve_safe_record(record, repo_root, label=label)
    try:
        gate.verify_source_hashes(
            source_records,
            repo_root,
            label="global_audit.source_records",
        )
    except gate.GatekeeperError as exc:
        raise PreflightError(str(exc)) from exc
    allocation = audit.get("frozen_allocation")
    trials = audit.get("protected_trials")
    if not isinstance(allocation, Mapping) or not isinstance(trials, Mapping):
        raise PreflightError(
            "global audit frozen_allocation/protected_trials must be objects"
        )
    for trial_id, expected_role in _ALLOCATION.items():
        allocation_entry = allocation.get(trial_id)
        if not isinstance(allocation_entry, Mapping):
            raise PreflightError(
                f"global audit frozen_allocation is missing trial {trial_id}"
            )
        if allocation_entry.get("role") != expected_role:
            raise PreflightError(
                f"global allocation role for trial {trial_id} must be "
                f"{expected_role}"
            )
        trial = trials.get(trial_id)
        if not isinstance(trial, Mapping):
            raise PreflightError(f"global audit is missing trial {trial_id}")
        for label, entry in (
            ("frozen_allocation", allocation_entry),
            ("protected_trials", trial),
        ):
            if entry.get("component_outcome_semantic_access_started") is not False:
                raise PreflightError(
                    f"protected trial {trial_id} {label} reports outcome access"
                )
            if entry.get("one_shot_status") != "NOT_CONSUMED":
                raise PreflightError(
                    f"protected trial {trial_id} {label} is already consumed"
                )
        if trial.get("status") != "CLOSED":
            raise PreflightError(
                f"protected trial {trial_id} must remain CLOSED"
            )
        if trial.get("outcome_access_receipts_found") != []:
            raise PreflightError(
                f"protected trial {trial_id} already has an outcome receipt"
            )


def _iter_source_records(
    records: Any,
    *,
    label: str,
) -> Iterable[tuple[str, Mapping[str, Any]]]:
    if isinstance(records, Mapping):
        for name, record in records.items():
            if not isinstance(name, str) or not isinstance(record, Mapping):
                raise PreflightError(f"{label} must contain named file records")
            yield f"{label}.{name}", record
        return
    if isinstance(records, Sequence) and not isinstance(records, (str, bytes)):
        for index, record in enumerate(records):
            if not isinstance(record, Mapping):
                raise PreflightError(f"{label}[{index}] must be a file record")
            yield f"{label}[{index}]", record
        return
    raise PreflightError(f"{label} must be an object or array of file records")


def _verify_unit_file_records(
    unit: Mapping[str, Any],
    repo_root: Path,
    *,
    label: str,
) -> None:
    record_fields = (
        "trace_csv",
        "per_sphere_csv",
        "trace_primary_profile",
        "detector_events_csv",
        "setup",
        "ik_sto",
        "grf_sto",
        "external_loads_xml",
        "sea_plugin",
        "states_sto",
        "online_grf_sto",
        "unit_source_receipt",
    )
    for field in record_fields:
        if field in unit and unit[field] is not None:
            _resolve_safe_record(
                unit[field],
                repo_root,
                label=f"{label}.{field}",
            )


def _validate_unit_source_receipt(
    unit: Mapping[str, Any],
    repo_root: Path,
    *,
    label: str,
    evidence_role: str,
) -> None:
    receipt_path = _resolve_safe_record(
        unit.get("unit_source_receipt"),
        repo_root,
        label=f"{label}.unit_source_receipt",
    )
    receipt = _strict_object(receipt_path, f"{label}.unit_source_receipt")
    expected_access_role = (
        "DEVELOPMENT_OPEN"
        if evidence_role == "development"
        else "HISTORICAL_CONSUMED"
    )
    if (
        type(receipt.get("schema_version")) is not int
        or receipt.get("schema_version") != 1
        or receipt.get("status") != "PASS"
        or receipt.get("unit_id") != unit.get("unit_id")
        or receipt.get("source_trial_id") != unit.get("trial_id")
        or receipt.get("plateau_id") != unit.get("plateau_id")
        or receipt.get("access_role") != expected_access_role
        or receipt.get("protected_component_outcome") is not False
    ):
        raise PreflightError(f"{label}.unit_source_receipt identity mismatch")
    for field in ("cadence_s", "start_s", "end_s"):
        value = receipt.get(field)
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            or not math.isclose(
                float(value),
                float(unit[field]),
                rel_tol=0.0,
                abs_tol=1e-12,
            )
        ):
            raise PreflightError(
                f"{label}.unit_source_receipt {field} mismatch"
            )
    if evidence_role == "development":
        if (
            receipt.get("plateau_speed_mps") != unit.get("plateau_speed_mps")
            or receipt.get("surface_velocity_mps")
            != unit.get("surface_velocity_mps")
        ):
            raise PreflightError(
                f"{label}.unit_source_receipt plateau velocity mismatch"
            )
    elif (
        receipt.get("plateau_speed_mps") is not None
        or receipt.get("surface_velocity_mps") is not None
    ):
        raise PreflightError(
            f"{label}.unit_source_receipt historical velocity must be null/omitted"
        )
    semantic_fields = {
        "setup",
        "ik_sto",
        "grf_sto",
        "external_loads_xml",
    }
    if unit["input_mode"] in {"coordinate_states", "h0_historical"}:
        semantic_fields.add("states_sto")
    if unit["input_mode"] == "h0_historical":
        semantic_fields.add("online_grf_sto")
    if "trace_csv" in unit:
        semantic_fields.update({"trace_csv", "per_sphere_csv"})
    semantic_sources = receipt.get("semantic_sources")
    if not isinstance(semantic_sources, Mapping) or set(semantic_sources) != semantic_fields:
        raise PreflightError(
            f"{label}.unit_source_receipt semantic_sources mismatch"
        )
    explicit_trials: set[str] = set()
    for field in sorted(semantic_fields):
        expected = unit[field]
        observed = semantic_sources[field]
        if not isinstance(observed, Mapping) or (
            observed.get("path") != expected.get("path")
            or observed.get("sha256") != expected.get("sha256")
        ):
            raise PreflightError(
                f"{label}.unit_source_receipt does not bind {field}"
            )
        explicit_trials.update(
            match.group(1)
            for match in re.finditer(
                r"(?:trial[_-]?|treadmill_01_)(0[1-8])",
                str(observed.get("path", "")),
                flags=re.IGNORECASE,
            )
        )
    if explicit_trials != {str(unit["trial_id"])}:
        raise PreflightError(
            f"{label} source paths declare trial(s) {sorted(explicit_trials)}, "
            f"not {unit['trial_id']}"
        )


def _validate_unit_spec(
    unit: Any,
    repo_root: Path,
    *,
    label: str,
    evidence_role: str,
) -> dict[str, Any]:
    if not isinstance(unit, Mapping):
        raise PreflightError(f"{label} must be an object")
    result = dict(unit)
    for field in ("unit_id", "trial_id", "plateau_id", "input_mode"):
        if not isinstance(result.get(field), str) or not result[field]:
            raise PreflightError(f"{label}.{field} must be a non-empty string")
    if result["input_mode"] not in INPUT_MODES:
        raise PreflightError(f"{label}.input_mode is unsupported")
    cadence = result.get("cadence_s")
    if (
        isinstance(cadence, bool)
        or not isinstance(cadence, (int, float))
        or not math.isfinite(float(cadence))
        or not any(
            math.isclose(float(cadence), expected, rel_tol=0.0, abs_tol=1e-12)
            for expected in CADENCES_S
        )
    ):
        raise PreflightError(f"{label}.cadence_s must be 0.001 or 0.010")
    if result["trial_id"] in PROTECTED_TRIALS:
        raise PreflightError(
            f"{label} attempts to open protected trial {result['trial_id']}"
        )
    for field in ("start_s", "end_s"):
        value = result.get(field)
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
        ):
            raise PreflightError(f"{label}.{field} must be finite")
        result[field] = float(value)
    if result["end_s"] <= result["start_s"]:
        raise PreflightError(f"{label} has an invalid time interval")
    if evidence_role == "development":
        if result["trial_id"] not in AUTHORIZED_TRIALS:
            raise PreflightError(f"{label} is not a development trial")
        if result["input_mode"] != "ik_prescribed":
            raise PreflightError(
                f"{label} development evidence must use ik_prescribed"
            )
        speed = result.get("plateau_speed_mps")
        velocity = result.get("surface_velocity_mps")
        if (
            isinstance(speed, bool)
            or not isinstance(speed, (int, float))
            or not math.isfinite(float(speed))
            or type(velocity) is not list
            or len(velocity) != 3
            or any(
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
                for value in velocity
            )
            or not np.allclose(
                np.asarray(velocity, dtype=float),
                np.asarray([0.0, 0.0, float(speed)]),
                rtol=0.0,
                atol=1e-12,
            )
        ):
            raise PreflightError(
                f"{label} must freeze plateau_speed_mps and "
                "surface_velocity_mps=[0,0,speed]"
            )
        result["plateau_speed_mps"] = float(speed)
        result["surface_velocity_mps"] = [
            float(value)
            for value in velocity
        ]
    else:
        if result["input_mode"] == "ik_prescribed" and result["trial_id"] != "01":
            raise PreflightError(
                f"{label} diagnostic IK replay is restricted to consumed trial 01"
            )
    for field in (
        "setup",
        "ik_sto",
        "grf_sto",
        "external_loads_xml",
        "sea_plugin",
    ):
        if field not in result:
            raise PreflightError(f"{label} requires hash-bound {field}")
    if "unit_source_receipt" not in result:
        raise PreflightError(f"{label} requires hash-bound unit_source_receipt")
    if result["input_mode"] in {"coordinate_states", "h0_historical"} and (
        "states_sto" not in result
    ):
        raise PreflightError(f"{label} requires hash-bound states_sto")
    if result["input_mode"] == "h0_historical" and "online_grf_sto" not in result:
        raise PreflightError(f"{label} requires observed online_grf_sto")
    has_precomputed = "trace_csv" in result or "per_sphere_csv" in result
    if has_precomputed:
        if "trace_csv" not in result or "per_sphere_csv" not in result:
            raise PreflightError(
                f"{label} precomputed route requires trace_csv and per_sphere_csv"
            )
        if "trace_primary_profile" not in result:
            raise PreflightError(
                f"{label} precomputed route requires trace_primary_profile"
            )
    _verify_unit_file_records(result, repo_root, label=label)
    _validate_unit_source_receipt(
        result,
        repo_root,
        label=label,
        evidence_role=evidence_role,
    )
    result["cadence_s"] = float(cadence)
    result["evidence_role"] = evidence_role
    return result


def _validate_unit_matrix(
    authorized_units: Sequence[Mapping[str, Any]],
    diagnostic_units: Sequence[Mapping[str, Any]],
) -> None:
    if len(authorized_units) != 24:
        raise PreflightError(
            f"authorized_units must contain exactly 24 units, got {len(authorized_units)}"
        )
    expected = {
        (trial_id, plateau_id, cadence)
        for trial_id in AUTHORIZED_TRIALS
        for plateau_id in ("01", "02", "03", "04")
        for cadence in CADENCES_S
    }
    observed: set[tuple[str, str, float]] = set()
    ids: set[str] = set()
    for unit in authorized_units:
        unit_id = str(unit["unit_id"])
        if unit_id in ids:
            raise PreflightError(f"duplicate unit_id {unit_id!r}")
        ids.add(unit_id)
        trial_id = str(unit["trial_id"])
        plateau_id = str(unit["plateau_id"])
        observed.add((trial_id, plateau_id, float(unit["cadence_s"])))
    if observed != expected:
        raise PreflightError(
            "authorized_units do not form the exact "
            "3 trial x 4 plateau x 2 cadence matrix"
        )
    grouped: dict[tuple[str, str], list[Mapping[str, Any]]] = {}
    for unit in authorized_units:
        grouped.setdefault(
            (str(unit["trial_id"]), str(unit["plateau_id"])),
            [],
        ).append(unit)
    canonical_windows: dict[tuple[str, str], tuple[float, float]] = {}
    for key, pair in grouped.items():
        if len(pair) != 2:
            raise PreflightError(
                f"development trial/plateau {key} must have exactly two cadences"
            )
        reference = pair[0]
        for candidate in pair[1:]:
            for field in ("start_s", "end_s", "plateau_speed_mps"):
                if not math.isclose(
                    float(candidate[field]),
                    float(reference[field]),
                    rel_tol=0.0,
                    abs_tol=1e-12,
                ):
                    raise PreflightError(
                        f"development trial/plateau {key} cadences must bind "
                        f"the same {field}"
                    )
            if not np.allclose(
                np.asarray(candidate["surface_velocity_mps"], dtype=float),
                np.asarray(reference["surface_velocity_mps"], dtype=float),
                rtol=0.0,
                atol=1e-12,
            ):
                raise PreflightError(
                    f"development trial/plateau {key} cadences must bind the "
                    "same surface_velocity_mps"
                )
        canonical_windows[key] = (
            float(reference["start_s"]),
            float(reference["end_s"]),
        )
    for trial_id in AUTHORIZED_TRIALS:
        ordered = [
            (plateau_id, canonical_windows[(trial_id, plateau_id)])
            for plateau_id in ("01", "02", "03", "04")
        ]
        for (left_id, left_window), (right_id, right_window) in zip(
            ordered,
            ordered[1:],
        ):
            if left_window[1] > right_window[0] + 1e-12:
                raise PreflightError(
                    f"development trial {trial_id} plateau windows must be "
                    f"distinct, ordered, and non-overlapping: "
                    f"{left_id}={left_window}, {right_id}={right_window}"
                )
    diagnostic_modes = {str(unit["input_mode"]) for unit in diagnostic_units}
    required = {"ik_prescribed", "coordinate_states", "h0_historical"}
    if not required.issubset(diagnostic_modes):
        raise PreflightError(
            "diagnostic_units require trial-01 IK, CoordinateStates, and H0 streams"
        )
    diagnostic_by_id = {
        str(unit["unit_id"]): unit
        for unit in diagnostic_units
    }
    for unit in diagnostic_units:
        if str(unit["unit_id"]) in ids:
            raise PreflightError(f"duplicate unit_id {unit['unit_id']!r}")
        ids.add(str(unit["unit_id"]))
        if unit["input_mode"] in {"coordinate_states", "h0_historical"}:
            reference_id = unit.get("reference_unit_id")
            reference = diagnostic_by_id.get(str(reference_id))
            if (
                reference is None
                or reference["input_mode"] != "ik_prescribed"
                or any(
                    unit[field] != reference[field]
                    for field in ("trial_id", "plateau_id", "cadence_s")
                )
            ):
                raise PreflightError(
                    f"diagnostic unit {unit['unit_id']!r} requires a same-window "
                    "trial/plateau/cadence IK reference"
                )


def preflight(
    *,
    protocol_path: str | Path,
    audit_path: str | Path,
    ledger_path: str | Path,
    receipt_path: str | Path,
    repo_root: str | Path = REPO_ROOT,
) -> PreflightContext:
    """Validate all non-semantic prerequisites before decoding any unit trace."""

    root = Path(repo_root).resolve()
    paths = {
        "protocol": Path(protocol_path).resolve(),
        "audit": Path(audit_path).resolve(),
        "ledger": Path(ledger_path).resolve(),
        "receipt": Path(receipt_path).resolve(),
    }
    for label, path in paths.items():
        try:
            path.relative_to(root)
        except ValueError as exc:
            raise PreflightError(f"{label} lies outside repo_root: {path}") from exc
        if not path.is_file():
            raise PreflightError(f"missing {label}: {path}")

    protocol = _strict_object(paths["protocol"], "primary protocol")
    audit = _strict_object(paths["audit"], "global data audit")
    ledger = _strict_object(paths["ledger"], "execution ledger")
    receipt = _strict_object(paths["receipt"], "run start receipt")
    protocol_sha = gate.sha256_file(paths["protocol"])
    audit_sha = gate.sha256_file(paths["audit"])
    ledger_sha = gate.sha256_file(paths["ledger"])
    receipt_sha = gate.sha256_file(paths["receipt"])
    if root == REPO_ROOT.resolve():
        canonical_audit = root / _CANONICAL_GLOBAL_AUDIT
        if (
            paths["audit"] != canonical_audit
            or audit_sha != _CANONICAL_GLOBAL_AUDIT_SHA256
        ):
            raise PreflightError(
                "production preflight requires the canonical frozen global "
                "data audit path and SHA-256"
            )

    if (
        type(protocol.get("schema_version")) is not int
        or protocol.get("schema_version") != 1
        or protocol.get("step_id") != "0"
    ):
        raise PreflightError("primary protocol must be schema v1, step 0")
    if protocol.get("status") != "FROZEN":
        raise PreflightError("primary protocol status must be exactly FROZEN")
    physical_contract = protocol.get("physical_contract")
    if not isinstance(physical_contract, Mapping):
        raise PreflightError("protocol.physical_contract must be an object")
    expected_contract_fields = {
        "primary_applied_side",
        "right_physical_support",
        "detector_role",
        "detector_may_affect_primary_metrics",
    }
    if (
        set(physical_contract) != expected_contract_fields
        or physical_contract.get("primary_applied_side") != "left"
        or physical_contract.get("right_physical_support") != "prescribed"
        or physical_contract.get("detector_may_affect_primary_metrics") is not False
        or physical_contract.get("detector_role")
        not in {"disabled", "shadow_diagnostic"}
    ):
        raise PreflightError(
            "physical_contract must freeze left primary, right prescribed, "
            "and a disabled/shadow detector excluded from primary metrics"
        )
    _validate_audit(audit, root)
    _require_exact_record(
        protocol,
        "global_data_access_audit",
        paths["audit"],
        root,
        label="protocol",
    )
    sources = protocol.get("sources")
    if not isinstance(sources, Mapping):
        raise PreflightError("protocol.sources must be a named object")
    required_source_names = (
        set(_REQUIRED_STATIC_PROTOCOL_SOURCES)
        | set(_REQUIRED_DYNAMIC_PROTOCOL_SOURCES)
    )
    missing_source_names = sorted(required_source_names.difference(sources))
    if missing_source_names:
        raise PreflightError(
            f"protocol.sources is missing required records: {missing_source_names}"
        )
    source_items = list(_iter_source_records(sources, label="protocol.sources"))
    if not source_items:
        raise PreflightError("protocol.sources must not be empty")
    for label, record in source_items:
        _resolve_safe_record(record, root, label=label)
    for name, required_path in _REQUIRED_STATIC_PROTOCOL_SOURCES.items():
        if sources[name].get("path") != required_path:
            raise PreflightError(
                f"protocol.sources.{name}.path must be exactly {required_path!r}"
            )
    try:
        gate.verify_source_hashes(sources, root, label="protocol.sources")
    except gate.GatekeeperError as exc:
        raise PreflightError(str(exc)) from exc

    primary_record = protocol.get("primary_profile")
    primary_path = _resolve_safe_record(
        primary_record,
        root,
        label="protocol.primary_profile",
    )
    _resolve_safe_record(
        protocol.get("candidate_grid"),
        root,
        label="protocol.candidate_grid",
    )

    if (
        type(ledger.get("schema_version")) is not int
        or ledger.get("schema_version") != 1
        or ledger.get("step_id") != "0"
    ):
        raise PreflightError("execution ledger must be schema v1, step 0")
    if ledger.get("status") != "RUNNING":
        raise PreflightError("execution ledger status must be exactly RUNNING")
    if ledger.get("protocol_sha256") != protocol_sha:
        raise PreflightError("execution ledger protocol hash mismatch")
    if ledger.get("global_data_access_audit_sha256") != audit_sha:
        raise PreflightError("execution ledger global audit hash mismatch")

    if (
        type(receipt.get("schema_version")) is not int
        or receipt.get("schema_version") != 1
        or receipt.get("step_id") != "0"
    ):
        raise PreflightError("run start receipt must be schema v1, step 0")
    if receipt.get("status") != "RUNNING":
        raise PreflightError("run start receipt status must be exactly RUNNING")
    if receipt.get("no_clobber") is not True:
        raise PreflightError("run start receipt no_clobber must be true")
    process_isolation = receipt.get("process_isolation")
    if (
        not isinstance(process_isolation, Mapping)
        or process_isolation.get("ray_processes_active") is not False
        or process_isolation.get("opensim_processes_active") is not False
        or process_isolation.get("baseline_new_process_required") is not True
        or not isinstance(process_isolation.get("checked_at_utc"), str)
        or not process_isolation["checked_at_utc"].strip()
        or not isinstance(process_isolation.get("inspection_command"), str)
        or not process_isolation["inspection_command"].strip()
    ):
        raise PreflightError("receipt process_isolation contract is not PASS")
    gate_tests = receipt.get("gate_0_0_tests")
    if type(gate_tests) is not list or not gate_tests:
        raise PreflightError("receipt gate_0_0_tests must be non-empty")
    for index, test in enumerate(gate_tests):
        if (
            not isinstance(test, Mapping)
            or set(test)
            != {"command", "passed", "passed_count", "failed_count"}
            or not isinstance(test.get("command"), str)
            or not test["command"].strip()
            or test.get("passed") is not True
            or type(test.get("passed_count")) is not int
            or test["passed_count"] <= 0
            or type(test.get("failed_count")) is not int
            or test["failed_count"] != 0
        ):
            raise PreflightError(f"receipt gate_0_0_tests[{index}] is not PASS")
    baseline_launch = receipt.get("baseline_launch")
    if (
        not isinstance(baseline_launch, Mapping)
        or not isinstance(baseline_launch.get("command"), str)
        or not baseline_launch["command"].strip()
        or baseline_launch.get("fresh_process") is not True
    ):
        raise PreflightError("receipt baseline_launch must require a fresh process")
    _require_exact_record(
        receipt,
        "protocol",
        paths["protocol"],
        root,
        label="receipt",
    )
    _require_exact_record(
        receipt,
        "global_data_access_audit",
        paths["audit"],
        root,
        label="receipt",
    )
    _require_exact_record(
        receipt,
        "execution_ledger",
        paths["ledger"],
        root,
        label="receipt",
    )
    if (
        receipt.get("run_id") != ledger.get("run_id")
        or not isinstance(receipt.get("run_id"), str)
        or not receipt["run_id"]
    ):
        raise PreflightError("ledger/receipt run_id mismatch")
    expected_receipt_path = (
        root
        / "validation"
        / "primary_grf_runs"
        / receipt["run_id"]
        / "run_start_receipt.json"
    ).resolve()
    if paths["receipt"] != expected_receipt_path:
        raise PreflightError(
            "run start receipt must use validation/primary_grf_runs/"
            "<run_id>/run_start_receipt.json"
        )
    receipt_profile = receipt.get("primary_profile")
    if not isinstance(receipt_profile, Mapping) or (
        receipt_profile.get("path") != primary_record.get("path")
        or receipt_profile.get("sha256") != primary_record.get("sha256")
    ):
        raise PreflightError("receipt primary profile binding mismatch")
    _resolve_safe_record(receipt_profile, root, label="receipt.primary_profile")
    if receipt.get("protected_trials_opened") != []:
        raise PreflightError("receipt must report protected_trials_opened=[]")

    authorized_raw = protocol.get("authorized_units")
    diagnostic_raw = protocol.get("diagnostic_units")
    if type(authorized_raw) is not list or type(diagnostic_raw) is not list:
        raise PreflightError(
            "protocol authorized_units and diagnostic_units must be arrays"
        )
    authorized = tuple(
        _validate_unit_spec(
            unit,
            root,
            label=f"authorized_units[{index}]",
            evidence_role="development",
        )
        for index, unit in enumerate(authorized_raw)
    )
    diagnostic = tuple(
        _validate_unit_spec(
            unit,
            root,
            label=f"diagnostic_units[{index}]",
            evidence_role="diagnostic",
        )
        for index, unit in enumerate(diagnostic_raw)
    )
    precomputed_unit_ids = [
        str(unit["unit_id"])
        for unit in authorized + diagnostic
        if "trace_csv" in unit or "per_sphere_csv" in unit
    ]
    if precomputed_unit_ids:
        raise PreflightError(
            "production preflight forbids precomputed trace_csv/per_sphere_csv "
            "because they bypass the frozen OpenSim profile loader; use the "
            "hash-bound raw offline adapter for units "
            f"{precomputed_unit_ids}"
        )
    _validate_unit_matrix(authorized, diagnostic)

    forensics = protocol.get("baseline_forensics")
    if not isinstance(forensics, Mapping):
        raise PreflightError("protocol.baseline_forensics must be an object")
    relative_output = safe_protocol_relative_path(
        forensics.get("output_dir"),
        label="baseline_forensics.output_dir",
    )
    if relative_output.as_posix() != "baseline_forensics":
        raise PreflightError(
            "baseline_forensics.output_dir must be exactly 'baseline_forensics'"
        )
    crossing_thresholds = forensics.get("crossing_thresholds_n")
    if (
        type(crossing_thresholds) is not list
        or len(crossing_thresholds) != len(CROSSING_THRESHOLDS_N)
        or any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            for value in crossing_thresholds
        )
        or tuple(float(value) for value in crossing_thresholds)
        != CROSSING_THRESHOLDS_N
    ):
        raise PreflightError(
            "crossing_thresholds_n must be exactly [0.5, 5.0, 20.0]"
        )
    oracle_threshold = forensics.get("oracle_threshold_n")
    if (
        isinstance(oracle_threshold, bool)
        or not isinstance(oracle_threshold, (int, float))
        or not math.isfinite(float(oracle_threshold))
        or float(oracle_threshold) != 20.0
    ):
        raise PreflightError("oracle_threshold_n must be exactly 20 N")
    expected_conditioning = {
        "scope": "development_oracle_and_primary_20n",
        "algorithm": "StreamingGaitEventDetector",
        "threshold_n": 20.0,
        "confirmation_threshold_n": 20.0,
        "min_contact_duration_s": ORACLE_EVENT_MIN_CONTACT_DURATION_S,
        "min_cycle_duration_s": ORACLE_EVENT_MIN_CYCLE_DURATION_S,
        "paired_cadence_right_margin_s": (
            ORACLE_EVENT_PAIRED_CADENCE_RIGHT_MARGIN_S
        ),
        "common_last_sample_policy": (
            "max_cadence_grid_from_frozen_window"
        ),
        "boundary_censor_policy": (
            "exclude_from_gating_retain_diagnostic"
        ),
        "historical_timing_policy": "preserve_linear_interpolated_raw",
    }
    conditioning = forensics.get("oracle_event_conditioning")
    if (
        not isinstance(conditioning, Mapping)
        or set(conditioning) != set(expected_conditioning)
        or any(
            conditioning.get(field) != expected
            or type(conditioning.get(field)) is not type(expected)
            for field, expected in expected_conditioning.items()
        )
    ):
        raise PreflightError(
            "baseline_forensics.oracle_event_conditioning must freeze the "
            "canonical 20 N / 0.05 s contact / 0.30 s cycle contract"
        )
    expected_output_contract = {
        "artifact_count": len(OUTPUT_FILENAMES),
        "csv_publication": "single_pass_streaming_fixed_schema",
        "staging": "private_sibling_o_excl_fsync_then_revalidate",
        "finalization": "atomic_directory_rename_noreplace",
        "commit_marker": "atomic_seven_artifact_directory_commit",
        "per_sphere_scope": (
            "primary_left_sphere_per_complete_oracle_cycle"
        ),
        "no_complete_cycle_fallback": (
            "whole_window_diagnostic_only_excluded_from_gating"
        ),
        "per_sphere_row_bound": (
            "sphere_count_x_max_1_complete_oracle_cycle_count"
        ),
        "dominant_sphere_tie_break": (
            "normal_impulse_desc_then_peak_force_desc_then_name_asc"
        ),
    }
    output_contract = forensics.get("output_contract")
    if (
        not isinstance(output_contract, Mapping)
        or set(output_contract) != set(expected_output_contract)
        or any(
            output_contract.get(field) != expected
            or type(output_contract.get(field)) is not type(expected)
            for field, expected in expected_output_contract.items()
        )
    ):
        raise PreflightError(
            "baseline_forensics.output_contract must freeze bounded "
            "single-pass staged publication"
        )
    output_dir = paths["receipt"].parent.joinpath(*relative_output.parts)
    if output_dir.exists() or os.path.lexists(output_dir):
        raise gate.NoClobberError(
            f"refusing occupied baseline_forensics output: {output_dir}"
        )

    return PreflightContext(
        repo_root=root,
        protocol_path=paths["protocol"],
        protocol_sha256=protocol_sha,
        protocol=protocol,
        audit_path=paths["audit"],
        audit_sha256=audit_sha,
        ledger_path=paths["ledger"],
        ledger_sha256=ledger_sha,
        receipt_path=paths["receipt"],
        receipt_sha256=receipt_sha,
        output_dir=output_dir,
        primary_profile_path=primary_path,
        primary_profile_record=dict(primary_record),
        authorized_units=authorized,
        diagnostic_units=diagnostic,
    )


def verify_loaded_primary_profile(
    loaded_path: str | Path,
    expected_record: Mapping[str, Any],
    repo_root: str | Path,
) -> dict[str, Any]:
    """Bind the profile actually passed to the loader to the frozen record."""

    root = Path(repo_root).resolve()
    path = Path(loaded_path).resolve()
    actual = _actual_record(path, root)
    if (
        actual["path"] != expected_record.get("path")
        or actual["sha256"] != expected_record.get("sha256")
    ):
        raise PreflightError(
            "the primary profile actually loaded differs from the frozen profile"
        )
    return actual


def _finite_array(
    value: Any,
    *,
    label: str,
    shape: tuple[int, ...] | None = None,
    nonnegative: bool = False,
) -> np.ndarray:
    array = np.asarray(value)
    if array.dtype.kind not in "biuf":
        raise TraceContractError(f"{label} must be numeric")
    converted = np.asarray(array, dtype=float)
    if shape is not None and converted.shape != shape:
        raise TraceContractError(
            f"{label} shape mismatch: expected {shape}, observed {converted.shape}"
        )
    if not np.all(np.isfinite(converted)):
        raise TraceContractError(f"{label} contains NaN or Infinity")
    if nonnegative and np.any(converted < 0.0):
        raise TraceContractError(f"{label} contains negative values")
    return converted


def validate_unit_trace(trace: UnitTrace) -> UnitTrace:
    """Validate one complete, finite offline metric input bundle."""

    if trace.input_mode not in INPUT_MODES:
        raise TraceContractError(f"unsupported input mode {trace.input_mode!r}")
    if trace.trial_id in PROTECTED_TRIALS:
        raise TraceContractError(
            f"unit trace attempts to use protected trial {trace.trial_id}"
        )
    if trace.evidence_role == "development":
        if (
            trace.plateau_speed_mps is None
            or not math.isfinite(float(trace.plateau_speed_mps))
            or trace.surface_velocity_mps is None
            or len(trace.surface_velocity_mps) != 3
            or not np.allclose(
                np.asarray(trace.surface_velocity_mps, dtype=float),
                np.asarray([0.0, 0.0, float(trace.plateau_speed_mps)]),
                rtol=0.0,
                atol=1e-12,
            )
        ):
            raise TraceContractError(
                "development trace has invalid plateau surface velocity"
            )
    elif (
        trace.plateau_speed_mps is not None
        or trace.surface_velocity_mps is not None
    ):
        raise TraceContractError(
            "historical diagnostic trace must not declare fitted velocity"
        )
    if (
        isinstance(trace.start_s, bool)
        or isinstance(trace.end_s, bool)
        or not isinstance(trace.start_s, (int, float))
        or not isinstance(trace.end_s, (int, float))
        or not math.isfinite(float(trace.start_s))
        or not math.isfinite(float(trace.end_s))
        or float(trace.end_s) <= float(trace.start_s)
    ):
        raise TraceContractError("trace start_s/end_s window is invalid")
    if (
        isinstance(trace.cadence_s, bool)
        or not isinstance(trace.cadence_s, (int, float))
        or not math.isfinite(float(trace.cadence_s))
        or float(trace.cadence_s) <= 0.0
    ):
        raise TraceContractError("trace cadence_s must be finite and positive")
    times = _finite_array(trace.times_s, label="times_s")
    if times.ndim != 1 or times.size < 2 or np.any(np.diff(times) <= 0.0):
        raise TraceContractError("times_s must be a strictly increasing vector")
    if not math.isclose(
        float(times[0]),
        float(trace.start_s),
        rel_tol=0.0,
        abs_tol=_TIME_GRID_ATOL_S,
    ):
        raise TraceContractError(
            "times_s must be anchored exactly at frozen start_s"
        )
    if float(times[-1]) > float(trace.end_s):
        raise TraceContractError(
            "times_s must not sample beyond frozen end_s"
        )
    end_gap_s = float(trace.end_s) - float(times[-1])
    if end_gap_s >= float(trace.cadence_s) - _TIME_GRID_ATOL_S:
        raise TraceContractError(
            "times_s must end at the last cadence tick not beyond frozen "
            "end_s, with a terminal gap smaller than cadence_s"
        )
    time_steps = np.diff(times)
    dt = float(np.median(time_steps))
    if not np.allclose(
        time_steps,
        trace.cadence_s,
        rtol=0.0,
        atol=_TIME_GRID_ATOL_S,
    ):
        raise TraceContractError(
            f"trace cadence mismatch: declared {trace.cadence_s}, observed {dt}"
        )
    if PRIMARY_SIDE not in trace.sides:
        raise TraceContractError("required primary side 'left' is unavailable")
    n = len(times)
    for side, series in trace.sides.items():
        if side not in {"left", "right"}:
            raise TraceContractError(f"unsupported side {side!r}")
        for field in (
            "oracle_force_n",
            "oracle_cop_m",
            "oracle_moment_nm",
            "primary_force_n",
            "primary_cop_m",
            "primary_moment_nm",
        ):
            _finite_array(
                getattr(series, field),
                label=f"{side}.{field}",
                shape=(n, 3),
            )
        for field, nonnegative in (
            ("primary_normal_force_n", True),
            ("primary_penetration_m", True),
            ("primary_slip_speed_m_s", True),
        ):
            _finite_array(
                getattr(series, field),
                label=f"{side}.{field}",
                shape=(n,),
                nonnegative=nonnegative,
            )
        contact = np.asarray(series.primary_in_contact)
        if contact.shape != (n,) or contact.dtype.kind != "b":
            raise TraceContractError(
                f"{side}.primary_in_contact must be a boolean vector"
            )
        if series.mesh_min_clearance_m is not None:
            _finite_array(
                series.mesh_min_clearance_m,
                label=f"{side}.mesh_min_clearance_m",
                shape=(n,),
            )
    left_spheres = 0
    names: set[str] = set()
    for sphere in trace.spheres:
        if not sphere.name or sphere.name in names:
            raise TraceContractError("sphere names must be non-empty and unique")
        names.add(sphere.name)
        if sphere.side == PRIMARY_SIDE:
            left_spheres += 1
        if sphere.side not in trace.sides:
            raise TraceContractError(
                f"sphere {sphere.name} references absent side {sphere.side}"
            )
        for field, nonnegative in (
            ("normal_force_n", True),
            ("penetration_m", True),
            ("slip_speed_m_s", True),
            ("clearance_m", False),
            ("center_height_m", False),
        ):
            _finite_array(
                getattr(sphere, field),
                label=f"sphere.{sphere.name}.{field}",
                shape=(n,),
                nonnegative=nonnegative,
            )
    if left_spheres == 0:
        raise TraceContractError("required primary side has no per-sphere evidence")
    if trace.sides[PRIMARY_SIDE].mesh_min_clearance_m is None:
        raise TraceContractError(
            "required primary side has no plantar-mesh clearance evidence"
        )
    for index, event in enumerate(trace.detector_events):
        if not isinstance(event, Mapping):
            raise TraceContractError(f"detector_events[{index}] must be an object")
        if event.get("side") not in trace.sides:
            raise TraceContractError(f"detector_events[{index}] has invalid side")
        if event.get("event_name") not in {"heel_strike", "toe_off"}:
            raise TraceContractError(
                f"detector_events[{index}].event_name must be "
                "'heel_strike' or 'toe_off'"
            )
        if not isinstance(event.get("sensor"), str) or not event["sensor"]:
            raise TraceContractError(
                f"detector_events[{index}].sensor is invalid"
            )
        event_times: dict[str, float] = {}
        for field in ("onset_time_s", "confirmed_time_s"):
            value = event.get(field)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
            ):
                raise TraceContractError(
                    f"detector_events[{index}].{field} must be finite"
                )
            event_times[field] = float(value)
        if not (
            float(times[0])
            <= event_times["onset_time_s"]
            <= event_times["confirmed_time_s"]
            <= float(times[-1])
        ):
            raise TraceContractError(
                f"detector_events[{index}] must satisfy "
                "trace_start <= onset <= confirmed <= trace_end"
            )
    return trace


def _anchored_time_grid(
    start_s: float,
    end_s: float,
    cadence_s: float,
) -> np.ndarray:
    """Build the maximal uniform grid anchored at start without overshooting."""

    if (
        not all(math.isfinite(value) for value in (start_s, end_s, cadence_s))
        or cadence_s <= 0.0
        or end_s <= start_s
    ):
        raise TraceContractError("cannot build an invalid offline time grid")
    interval_count = int(
        math.floor(
            (end_s - start_s + _TIME_GRID_ATOL_S) / cadence_s
        )
    )
    while interval_count > 0 and (
        start_s + interval_count * cadence_s
        > end_s + _TIME_GRID_ATOL_S
    ):
        interval_count -= 1
    if interval_count < 1:
        raise TraceContractError(
            "offline time window must contain at least one cadence interval"
        )
    times = start_s + np.arange(interval_count + 1, dtype=float) * cadence_s
    if times[-1] > end_s or math.isclose(
        float(times[-1]),
        end_s,
        rel_tol=0.0,
        abs_tol=_TIME_GRID_ATOL_S,
    ):
        times[-1] = end_s
    if np.any(times > end_s):
        raise TraceContractError("offline time grid would sample beyond end_s")
    return times


_VECTOR_COLUMNS = {
    "oracle_force_n": (
        "oracle_force_x_n",
        "oracle_force_y_n",
        "oracle_force_z_n",
    ),
    "oracle_cop_m": (
        "oracle_cop_x_m",
        "oracle_cop_y_m",
        "oracle_cop_z_m",
    ),
    "oracle_moment_nm": (
        "oracle_moment_x_nm",
        "oracle_moment_y_nm",
        "oracle_moment_z_nm",
    ),
    "primary_force_n": (
        "primary_force_x_n",
        "primary_force_y_n",
        "primary_force_z_n",
    ),
    "primary_cop_m": (
        "primary_cop_x_m",
        "primary_cop_y_m",
        "primary_cop_z_m",
    ),
    "primary_moment_nm": (
        "primary_moment_x_nm",
        "primary_moment_y_nm",
        "primary_moment_z_nm",
    ),
}
_SCALAR_COLUMNS = (
    "primary_normal_force_n",
    "primary_penetration_m",
    "primary_slip_speed_m_s",
)
_TRACE_CSV_FIELDS = (
    "unit_id",
    "trial_id",
    "plateau_id",
    "cadence_s",
    "input_mode",
    "evidence_role",
    "reference_unit_id",
    "time_s",
    "cycle_index",
    "side",
    *(
        column
        for columns in _VECTOR_COLUMNS.values()
        for column in columns
    ),
    *_SCALAR_COLUMNS,
    "primary_in_contact",
    "mesh_min_clearance_m",
    "dominant_sphere_name",
    "dominant_sphere_normal_force_n",
    "dominant_sphere_penetration_m",
    "dominant_sphere_clearance_m",
    "dominant_sphere_center_height_m",
    "dominant_sphere_slip_speed_m_s",
)
_PER_SPHERE_CSV_FIELDS = (
    "unit_id",
    "trial_id",
    "plateau_id",
    "cadence_s",
    "input_mode",
    "evidence_role",
    "reference_unit_id",
    "side",
    "sphere_name",
    "scope_kind",
    "diagnostic_only",
    "excluded_from_gating",
    "cycle_index",
    "cycle_start_s",
    "cycle_end_s",
    "left_boundary_censored",
    "right_boundary_censored",
    "sample_count",
    "penetration_present",
    "first_penetration_time_s",
    "last_penetration_time_s",
    "penetration_episode_count",
    "penetration_duration_s",
    "normal_force_peak_n",
    "normal_force_peak_time_s",
    "normal_force_impulse_ns",
    "penetration_max_m",
    "penetration_p95_active_m",
    "slip_speed_max_m_s",
    "slip_speed_p95_active_m_s",
    "clearance_min_m",
    "clearance_min_time_s",
    "center_height_at_min_clearance_m",
    "clearance_at_force_peak_m",
    "center_height_at_force_peak_m",
    "is_dominant",
    "dominant_sphere_name",
    "dominant_sphere_normal_impulse_ns",
    "dominant_sphere_force_peak_n",
    "dominance_fraction",
)


def _read_csv_rows(path: Path, *, label: str) -> tuple[list[str], list[dict[str, str]]]:
    try:
        with path.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            fields = list(reader.fieldnames or [])
            if len(fields) != len(set(fields)):
                raise TraceContractError(f"{label} contains duplicate CSV columns")
            rows = [dict(row) for row in reader]
    except (OSError, UnicodeError, csv.Error) as exc:
        raise TraceContractError(f"cannot read {label}: {path}: {exc}") from exc
    if not fields or not rows:
        raise TraceContractError(f"{label} must contain a header and data rows")
    return fields, rows


def _csv_float(row: Mapping[str, str], field: str, *, label: str) -> float:
    if field not in row:
        raise TraceContractError(f"{label} is missing column {field!r}")
    try:
        value = float(row[field])
    except (TypeError, ValueError) as exc:
        raise TraceContractError(f"{label}.{field} is not numeric") from exc
    if not math.isfinite(value):
        raise TraceContractError(f"{label}.{field} is not finite")
    return value


def _csv_bool(row: Mapping[str, str], field: str, *, label: str) -> bool:
    if field not in row:
        raise TraceContractError(f"{label} is missing column {field!r}")
    value = str(row[field]).strip().lower()
    if value in {"1", "true"}:
        return True
    if value in {"0", "false"}:
        return False
    raise TraceContractError(f"{label}.{field} must be boolean 0/1/true/false")


def load_precomputed_unit(
    unit: Mapping[str, Any],
    repo_root: str | Path,
) -> UnitTrace:
    """Load one canonical precomputed trace after re-verifying live hashes."""

    root = Path(repo_root).resolve()
    trace_path = _resolve_safe_record(
        unit.get("trace_csv"),
        root,
        label=f"{unit.get('unit_id')}.trace_csv",
    )
    sphere_path = _resolve_safe_record(
        unit.get("per_sphere_csv"),
        root,
        label=f"{unit.get('unit_id')}.per_sphere_csv",
    )
    _fields, rows = _read_csv_rows(trace_path, label="trace_csv")
    by_side: dict[str, list[dict[str, str]]] = {}
    for index, row in enumerate(rows):
        side = str(row.get("side", ""))
        if side not in {"left", "right"}:
            raise TraceContractError(f"trace_csv row {index} has invalid side")
        by_side.setdefault(side, []).append(row)
    if PRIMARY_SIDE not in by_side:
        raise TraceContractError("trace_csv has no required left primary stream")

    common_times: np.ndarray | None = None
    sides: dict[str, SideSeries] = {}
    for side, side_rows in sorted(by_side.items()):
        times = np.asarray(
            [
                _csv_float(row, "time_s", label=f"trace_csv.{side}[{index}]")
                for index, row in enumerate(side_rows)
            ],
            dtype=float,
        )
        if common_times is None:
            common_times = times
        elif not np.array_equal(times, common_times):
            raise TraceContractError("trace_csv sides do not share an exact time grid")
        vectors = {
            name: np.asarray(
                [
                    [
                        _csv_float(
                            row,
                            column,
                            label=f"trace_csv.{side}[{row_index}]",
                        )
                        for column in columns
                    ]
                    for row_index, row in enumerate(side_rows)
                ],
                dtype=float,
            )
            for name, columns in _VECTOR_COLUMNS.items()
        }
        scalars = {
            name: np.asarray(
                [
                    _csv_float(
                        row,
                        name,
                        label=f"trace_csv.{side}[{row_index}]",
                    )
                    for row_index, row in enumerate(side_rows)
                ],
                dtype=float,
            )
            for name in _SCALAR_COLUMNS
        }
        contacts = np.asarray(
            [
                _csv_bool(
                    row,
                    "primary_in_contact",
                    label=f"trace_csv.{side}[{row_index}]",
                )
                for row_index, row in enumerate(side_rows)
            ],
            dtype=bool,
        )
        mesh_values = [row.get("mesh_min_clearance_m", "") for row in side_rows]
        if all(str(value).strip() == "" for value in mesh_values):
            mesh = None
        elif any(str(value).strip() == "" for value in mesh_values):
            raise TraceContractError(
                f"trace_csv.{side}.mesh_min_clearance_m is partially missing"
            )
        else:
            mesh = np.asarray(
                [
                    _csv_float(
                        row,
                        "mesh_min_clearance_m",
                        label=f"trace_csv.{side}[{row_index}]",
                    )
                    for row_index, row in enumerate(side_rows)
                ],
                dtype=float,
            )
        sides[side] = SideSeries(
            **vectors,
            **scalars,
            primary_in_contact=contacts,
            mesh_min_clearance_m=mesh,
        )
    assert common_times is not None

    _sphere_fields, sphere_rows = _read_csv_rows(
        sphere_path,
        label="per_sphere_csv",
    )
    grouped: dict[tuple[str, str], list[dict[str, str]]] = {}
    for index, row in enumerate(sphere_rows):
        name = str(row.get("sphere_name", "")).strip()
        side = str(row.get("side", "")).strip()
        if not name or side not in sides:
            raise TraceContractError(
                f"per_sphere_csv row {index} has invalid sphere identity"
            )
        grouped.setdefault((side, name), []).append(row)
    spheres: list[SphereSeries] = []
    for (side, name), group in sorted(grouped.items()):
        times = np.asarray(
            [
                _csv_float(
                    row,
                    "time_s",
                    label=f"per_sphere_csv.{name}[{index}]",
                )
                for index, row in enumerate(group)
            ]
        )
        if not np.array_equal(times, common_times):
            raise TraceContractError(
                f"per_sphere_csv sphere {name} does not share trace time grid"
            )
        values = {}
        for field in (
            "normal_force_n",
            "penetration_m",
            "clearance_m",
            "center_height_m",
            "slip_speed_m_s",
        ):
            values[field] = np.asarray(
                [
                    _csv_float(
                        row,
                        field,
                        label=f"per_sphere_csv.{name}[{index}]",
                    )
                    for index, row in enumerate(group)
                ],
                dtype=float,
            )
        spheres.append(SphereSeries(name=name, side=side, **values))

    detector_events: tuple[Mapping[str, Any], ...] = ()
    detector_record = unit.get("detector_events_csv")
    if detector_record is not None:
        detector_path = _resolve_safe_record(
            detector_record,
            root,
            label=f"{unit.get('unit_id')}.detector_events_csv",
        )
        _detector_fields, detector_rows = _read_csv_rows(
            detector_path,
            label="detector_events_csv",
        )
        parsed = []
        for index, row in enumerate(detector_rows):
            parsed.append(
                {
                    "side": str(row.get("side", "")),
                    "sensor": str(row.get("sensor", "")),
                    "event_name": str(row.get("event_name", "")),
                    "onset_time_s": _csv_float(
                        row,
                        "onset_time_s",
                        label=f"detector_events_csv[{index}]",
                    ),
                    "confirmed_time_s": _csv_float(
                        row,
                        "confirmed_time_s",
                        label=f"detector_events_csv[{index}]",
                    ),
                }
            )
        detector_events = tuple(parsed)

    trace = UnitTrace(
        unit_id=str(unit["unit_id"]),
        trial_id=str(unit["trial_id"]),
        plateau_id=str(unit["plateau_id"]),
        cadence_s=float(unit["cadence_s"]),
        input_mode=str(unit["input_mode"]),
        evidence_role=str(unit["evidence_role"]),
        start_s=float(unit["start_s"]),
        end_s=float(unit["end_s"]),
        times_s=common_times,
        sides=sides,
        spheres=tuple(spheres),
        detector_events=detector_events,
        reference_unit_id=(
            str(unit["reference_unit_id"])
            if unit.get("reference_unit_id") is not None
            else None
        ),
        plateau_speed_mps=unit.get("plateau_speed_mps"),
        surface_velocity_mps=(
            tuple(unit["surface_velocity_mps"])
            if unit.get("surface_velocity_mps") is not None
            else None
        ),
    )
    return validate_unit_trace(trace)


def _crossing_times(
    times: np.ndarray,
    values: np.ndarray,
    threshold: float,
    *,
    rising: bool,
) -> np.ndarray:
    """Reuse the established linear-interpolated online-GRF crossing helper."""

    time_array = np.asarray(times, dtype=float)
    value_array = np.asarray(values, dtype=float)
    try:
        from validation.validate_online_grf import _crossing_times as established
    except ModuleNotFoundError as exc:
        if exc.name != "opensim":
            raise
        before = value_array[:-1] <= threshold
        after = value_array[1:] > threshold
        mask = before & after if rising else ~before & ~after
        crossings = []
        for index in np.flatnonzero(mask):
            delta = value_array[index + 1] - value_array[index]
            fraction = (
                0.0
                if abs(delta) < 1e-12
                else (threshold - value_array[index]) / delta
            )
            crossings.append(
                time_array[index]
                + fraction * (time_array[index + 1] - time_array[index])
            )
        result = np.asarray(crossings, dtype=float)
    else:
        result = established(
            time_array,
            value_array,
            float(threshold),
            bool(rising),
        )
    if rising and value_array[0] > threshold:
        result = np.concatenate(([time_array[0]], np.asarray(result, dtype=float)))
    if not rising and value_array[-1] > threshold:
        result = np.concatenate((np.asarray(result, dtype=float), [time_array[-1]]))
    return _finite_array(result, label="crossing_times")


def _match_events(
    reference: Iterable[float],
    predicted: Iterable[float],
    tolerance_s: float,
) -> dict[str, Any]:
    try:
        from validation.validate_online_grf_events import match_events
    except ModuleNotFoundError as exc:
        if exc.name != "opensim":
            raise
        ref = np.asarray(list(reference), dtype=float)
        pred = np.asarray(list(predicted), dtype=float)
        candidates = sorted(
            (
                abs(float(pred[pred_index] - ref[ref_index])),
                ref_index,
                pred_index,
            )
            for ref_index in range(len(ref))
            for pred_index in range(len(pred))
            if abs(float(pred[pred_index] - ref[ref_index])) <= tolerance_s
        )
        matched_ref: set[int] = set()
        matched_pred: set[int] = set()
        pairs: list[dict[str, float]] = []
        for absolute_error, ref_index, pred_index in candidates:
            if ref_index in matched_ref or pred_index in matched_pred:
                continue
            matched_ref.add(ref_index)
            matched_pred.add(pred_index)
            error = float(pred[pred_index] - ref[ref_index])
            pairs.append(
                {
                    "reference_time_s": float(ref[ref_index]),
                    "predicted_time_s": float(pred[pred_index]),
                    "error_s": error,
                    "absolute_error_s": absolute_error,
                }
            )
        errors = np.asarray([pair["error_s"] for pair in pairs], dtype=float)
        tp = len(pairs)
        fp = len(pred) - tp
        fn = len(ref) - tp
        precision = tp / max(1, tp + fp)
        recall = tp / max(1, tp + fn)
        f1 = 2.0 * precision * recall / max(1e-12, precision + recall)
        return {
            "reference_count": int(len(ref)),
            "predicted_count": int(len(pred)),
            "matched_count": int(tp),
            "false_positives": int(fp),
            "false_negatives": int(fn),
            "precision": float(precision),
            "recall": float(recall),
            "f1": float(f1),
            "timing_bias_s": float(np.mean(errors)) if len(errors) else float("nan"),
            "timing_mae_s": (
                float(np.mean(np.abs(errors))) if len(errors) else float("nan")
            ),
            "timing_max_abs_s": (
                float(np.max(np.abs(errors))) if len(errors) else float("nan")
            ),
            "unmatched_reference": [
                float(ref[index])
                for index in range(len(ref))
                if index not in matched_ref
            ],
            "unmatched_predicted": [
                float(pred[index])
                for index in range(len(pred))
                if index not in matched_pred
            ],
            "pairs": pairs,
        }
    return match_events(reference, predicted, tolerance_s)


def _json_finite(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _json_finite(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_finite(item) for item in value]
    if isinstance(value, np.ndarray):
        return _json_finite(value.tolist())
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    if isinstance(value, (np.integer, int)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        number = float(value)
        return number if math.isfinite(number) else None
    return value


def _hash_array(digest: Any, name: str, value: np.ndarray) -> None:
    array = np.ascontiguousarray(value)
    digest.update(name.encode("utf-8"))
    digest.update(str(array.dtype).encode("ascii"))
    digest.update(str(array.shape).encode("ascii"))
    digest.update(array.tobytes())


def metric_input_fingerprint(trace: UnitTrace) -> str:
    """Digest only oracle/mesh/primary inputs; detector fields are excluded."""

    validate_unit_trace(trace)
    digest = hashlib.sha256()
    for text in (
        trace.unit_id,
        trace.trial_id,
        trace.plateau_id,
        trace.input_mode,
        trace.evidence_role,
        trace.reference_unit_id or "<none>",
        f"{trace.cadence_s:.17g}",
        f"{trace.start_s:.17g}",
        f"{trace.end_s:.17g}",
    ):
        digest.update(text.encode("utf-8"))
        digest.update(b"\0")
    _hash_array(digest, "times_s", np.asarray(trace.times_s, dtype="<f8"))
    for side, series in sorted(trace.sides.items()):
        digest.update(side.encode("utf-8"))
        for field in (
            "oracle_force_n",
            "oracle_cop_m",
            "oracle_moment_nm",
            "primary_force_n",
            "primary_normal_force_n",
            "primary_cop_m",
            "primary_moment_nm",
            "primary_penetration_m",
            "primary_slip_speed_m_s",
            "primary_in_contact",
        ):
            _hash_array(digest, field, np.asarray(getattr(series, field)))
        if series.mesh_min_clearance_m is not None:
            _hash_array(
                digest,
                "mesh_min_clearance_m",
                np.asarray(series.mesh_min_clearance_m, dtype="<f8"),
            )
    for sphere in sorted(trace.spheres, key=lambda item: (item.side, item.name)):
        digest.update(f"{sphere.side}/{sphere.name}".encode("utf-8"))
        for field in (
            "normal_force_n",
            "penetration_m",
            "clearance_m",
            "center_height_m",
            "slip_speed_m_s",
        ):
            _hash_array(digest, field, np.asarray(getattr(sphere, field), dtype="<f8"))
    return digest.hexdigest()


def _event_vector(
    times: np.ndarray,
    force: np.ndarray,
    threshold_n: float,
) -> dict[str, np.ndarray]:
    return {
        "heel_strike": _crossing_times(
            times,
            force,
            threshold_n,
            rising=True,
        ),
        "toe_off": _crossing_times(
            times,
            force,
            threshold_n,
            rising=False,
        ),
    }


def _canonical_conditioned_events(
    times: np.ndarray,
    force: np.ndarray,
    threshold_n: float,
) -> dict[str, np.ndarray]:
    """Reuse the canonical causal reference detector, with an exact fallback."""

    try:
        from validation.validate_online_grf_events import detect_events
    except ModuleNotFoundError as exc:
        if exc.name != "opensim":
            raise
        candidate_start: float | None = None
        confirmed_start: float | None = None
        last_hs: float | None = None
        above = False
        heel_strikes: list[float] = []
        confirmed_times: list[float] = []
        toe_offs: list[float] = []
        for time_value, force_value in zip(times, force):
            time_s = float(time_value)
            value = float(force_value)
            current_above = value > threshold_n
            if current_above and not above:
                candidate_start = time_s
            if current_above and candidate_start is not None:
                duration = time_s - candidate_start
                if (
                    confirmed_start is None
                    and duration >= ORACLE_EVENT_MIN_CONTACT_DURATION_S
                ):
                    cycle_duration = (
                        None
                        if last_hs is None
                        else candidate_start - last_hs
                    )
                    if (
                        last_hs is None
                        or cycle_duration >= ORACLE_EVENT_MIN_CYCLE_DURATION_S
                    ):
                        heel_strikes.append(candidate_start)
                        confirmed_times.append(time_s)
                        last_hs = candidate_start
                    confirmed_start = candidate_start
            if not current_above and above:
                if confirmed_start is not None:
                    toe_offs.append(time_s)
                candidate_start = None
                confirmed_start = None
            above = current_above
        detected = {
            "heel_strikes": np.asarray(heel_strikes, dtype=float),
            "heel_strike_confirmed_times": np.asarray(
                confirmed_times,
                dtype=float,
            ),
            "toe_offs": np.asarray(toe_offs, dtype=float),
        }
    else:
        detected = detect_events(
            np.asarray(times, dtype=float),
            np.asarray(force, dtype=float),
            threshold_n=float(threshold_n),
            confirmation_threshold_n=float(threshold_n),
            min_contact_duration_s=ORACLE_EVENT_MIN_CONTACT_DURATION_S,
            min_cycle_duration_s=ORACLE_EVENT_MIN_CYCLE_DURATION_S,
        )

    # Normalize the canonical state machine at the two scientific boundaries:
    # nominal cadence arithmetic receives the audit time tolerance, and a
    # toe-off is retained only when its contact emitted an accepted HS.  The
    # runtime detector otherwise emits an orphan TO after a minimum-cycle
    # suppression.  This loop is causal and preserves first-above/first-below
    # sample timing.
    sample_times = np.asarray(times, dtype=float)
    sample_force = np.asarray(force, dtype=float)
    accepted_heel_strikes: list[float] = []
    accepted_confirmed_times: list[float] = []
    accepted_toe_offs: list[float] = []
    candidate_start: float | None = None
    accepted_contact = False
    last_accepted_hs: float | None = None
    above = False
    for time_value, force_value in zip(sample_times, sample_force):
        time_s = float(time_value)
        current_above = float(force_value) > float(threshold_n)
        if current_above and not above:
            candidate_start = time_s
            accepted_contact = False
        if current_above and candidate_start is not None and not accepted_contact:
            dwell_ok = (
                time_s - candidate_start + _TIME_GRID_ATOL_S
                >= ORACLE_EVENT_MIN_CONTACT_DURATION_S
            )
            cycle_ok = (
                last_accepted_hs is None
                or candidate_start
                - last_accepted_hs
                + _TIME_GRID_ATOL_S
                >= ORACLE_EVENT_MIN_CYCLE_DURATION_S
            )
            if dwell_ok:
                if cycle_ok:
                    accepted_heel_strikes.append(candidate_start)
                    accepted_confirmed_times.append(time_s)
                    last_accepted_hs = candidate_start
                    accepted_contact = True
                else:
                    # Mark this contact resolved so it cannot emit an orphan
                    # TO or become accepted later during the same stance.
                    candidate_start = None
        if not current_above and above:
            if accepted_contact:
                accepted_toe_offs.append(time_s)
            candidate_start = None
            accepted_contact = False
        above = current_above
    return {
        "heel_strikes": np.asarray(accepted_heel_strikes, dtype=float),
        "heel_strike_confirmed_times": np.asarray(
            accepted_confirmed_times,
            dtype=float,
        ),
        "toe_offs": np.asarray(accepted_toe_offs, dtype=float),
    }


def _conditioned_event_bundle(
    trace: UnitTrace,
    force: np.ndarray,
    threshold_n: float,
) -> dict[str, Any]:
    """Return genuine development events plus boundary-censored diagnostics."""

    times = np.asarray(trace.times_s, dtype=float)
    values = np.asarray(force, dtype=float)
    left_censored = bool(values[0] > threshold_n)
    right_censored = bool(values[-1] > threshold_n)
    censored_records: list[dict[str, Any]] = []
    if left_censored:
        below = np.flatnonzero(values <= threshold_n)
        first_below = int(below[0]) if below.size else len(times)
        censored_records.extend(
            (
                {
                    "event_name": "heel_strike",
                    "event_time_s": float(times[0]),
                    "confirmed_time_s": None,
                    "boundary": "left",
                },
                {
                    "event_name": "toe_off",
                    "event_time_s": (
                        float(times[first_below])
                        if first_below < len(times)
                        else float(trace.end_s)
                    ),
                    "confirmed_time_s": (
                        float(times[first_below])
                        if first_below < len(times)
                        else float(trace.end_s)
                    ),
                    "boundary": "left",
                },
            )
        )
        detection_start = first_below
    else:
        detection_start = 0
    detected = (
        _canonical_conditioned_events(
            times[detection_start:],
            values[detection_start:],
            threshold_n,
        )
        if detection_start < len(times)
        else {
            "heel_strikes": np.asarray([], dtype=float),
            "heel_strike_confirmed_times": np.asarray([], dtype=float),
            "toe_offs": np.asarray([], dtype=float),
        }
    )
    all_hs = np.asarray(detected["heel_strikes"], dtype=float)
    all_confirmed = np.asarray(
        detected["heel_strike_confirmed_times"],
        dtype=float,
    )
    all_to = np.asarray(detected["toe_offs"], dtype=float)
    genuine_hs = all_hs
    genuine_confirmed = all_confirmed
    candidate_to = all_to
    complete_cycles: list[dict[str, Any]] = []
    incomplete_cycles: list[dict[str, Any]] = []
    accepted_to: list[float] = []
    accepted_hs_indices: set[int] = set()
    common_last_sample_s = float(
        _anchored_time_grid(
            float(trace.start_s),
            float(trace.end_s),
            max(CADENCES_S),
        )[-1]
    )
    for cycle_index in range(max(0, len(genuine_hs) - 1)):
        start = float(genuine_hs[cycle_index])
        end = float(genuine_hs[cycle_index + 1])
        cycle_to = candidate_to[
            (candidate_to > start + _TIME_GRID_ATOL_S)
            & (candidate_to <= end + _TIME_GRID_ATOL_S)
        ]
        record = {
            "cycle_index": cycle_index,
            "cycle_start_s": start,
            "cycle_end_s": end,
            "toe_off_count": int(len(cycle_to)),
        }
        right_margin_ok = (
            end
            + ORACLE_EVENT_MIN_CONTACT_DURATION_S
            + ORACLE_EVENT_PAIRED_CADENCE_RIGHT_MARGIN_S
            < common_last_sample_s - _TIME_GRID_ATOL_S
        )
        record["common_right_margin_ok"] = right_margin_ok
        if len(cycle_to) == 1 and right_margin_ok:
            record["toe_off_s"] = float(cycle_to[0])
            complete_cycles.append(record)
            accepted_to.append(float(cycle_to[0]))
            accepted_hs_indices.update((cycle_index, cycle_index + 1))
        else:
            record["exclusion_reason"] = (
                "right_boundary_common_confirmation_margin"
                if len(cycle_to) == 1
                else "toe_off_count_not_one"
            )
            incomplete_cycles.append(record)
    last_hs = float(genuine_hs[-1]) if genuine_hs.size else -math.inf
    for event_time in candidate_to[candidate_to > last_hs]:
        censored_records.append(
            {
                "event_name": "toe_off",
                "event_time_s": float(event_time),
                "confirmed_time_s": float(event_time),
                "boundary": "right_unclosed_cycle",
            }
        )
    if right_censored:
        censored_records.append(
            {
                "event_name": "toe_off",
                "event_time_s": float(trace.end_s),
                "confirmed_time_s": float(trace.end_s),
                "boundary": "right",
            }
        )
    accepted_hs = genuine_hs[sorted(accepted_hs_indices)]
    accepted_confirmed = genuine_confirmed[sorted(accepted_hs_indices)]
    excluded_hs_indices = sorted(
        set(range(len(genuine_hs))).difference(accepted_hs_indices)
    )
    for index in excluded_hs_indices:
        censored_records.append(
            {
                "event_name": "heel_strike",
                "event_time_s": float(genuine_hs[index]),
                "confirmed_time_s": float(genuine_confirmed[index]),
                "boundary": "right_unclosed_cycle",
            }
        )
    return {
        "heel_strike": _finite_array(
            accepted_hs,
            label="conditioned_heel_strikes",
        ),
        "heel_strike_confirmed": _finite_array(
            accepted_confirmed,
            label="conditioned_heel_strike_confirmed",
        ),
        "toe_off": _finite_array(
            np.asarray(accepted_to, dtype=float),
            label="conditioned_toe_offs",
        ),
        "complete_cycles": tuple(complete_cycles),
        "incomplete_cycles": tuple(incomplete_cycles),
        "genuine_heel_strike": _finite_array(
            genuine_hs,
            label="conditioned_genuine_heel_strikes",
        ),
        "genuine_heel_strike_confirmed": _finite_array(
            genuine_confirmed,
            label="conditioned_genuine_heel_strike_confirmed",
        ),
        "genuine_toe_off": _finite_array(
            candidate_to,
            label="conditioned_genuine_toe_offs",
        ),
        "common_last_sample_s": common_last_sample_s,
        "censored_records": tuple(censored_records),
        "left_boundary_censored": left_censored,
        "right_boundary_censored": right_censored,
        "conditioning": {
            "algorithm": "StreamingGaitEventDetector",
            "threshold_n": float(threshold_n),
            "confirmation_threshold_n": float(threshold_n),
            "min_contact_duration_s": ORACLE_EVENT_MIN_CONTACT_DURATION_S,
            "min_cycle_duration_s": ORACLE_EVENT_MIN_CYCLE_DURATION_S,
            "paired_cadence_right_margin_s": (
                ORACLE_EVENT_PAIRED_CADENCE_RIGHT_MARGIN_S
            ),
            "common_last_sample_policy": (
                "max_cadence_grid_from_frozen_window"
            ),
            "boundary_censor_policy": (
                "exclude_from_gating_retain_diagnostic"
            ),
        },
    }


def _metric_event_bundle(
    trace: UnitTrace,
    force: np.ndarray,
    threshold_n: float,
) -> dict[str, Any]:
    """Condition 20-N development events; preserve historical raw timing."""

    if (
        trace.evidence_role == "development"
        and math.isclose(
            float(threshold_n),
            ORACLE_THRESHOLD_N,
            rel_tol=0.0,
            abs_tol=1e-12,
        )
    ):
        return _conditioned_event_bundle(trace, force, threshold_n)
    raw = _event_vector(
        np.asarray(trace.times_s, dtype=float),
        np.asarray(force, dtype=float),
        threshold_n,
    )
    complete_cycles: list[dict[str, Any]] = []
    incomplete_cycles: list[dict[str, Any]] = []
    for index in range(max(0, len(raw["heel_strike"]) - 1)):
        start = float(raw["heel_strike"][index])
        end = float(raw["heel_strike"][index + 1])
        toe_offs = raw["toe_off"][
            (raw["toe_off"] > start + _TIME_GRID_ATOL_S)
            & (raw["toe_off"] <= end + _TIME_GRID_ATOL_S)
        ]
        record = {
            "cycle_index": index,
            "cycle_start_s": start,
            "cycle_end_s": end,
            "toe_off_count": int(len(toe_offs)),
        }
        if len(toe_offs) == 1:
            record["toe_off_s"] = float(toe_offs[0])
            complete_cycles.append(record)
        else:
            record["exclusion_reason"] = "toe_off_count_not_one"
            incomplete_cycles.append(record)
    return {
        **raw,
        "heel_strike_confirmed": np.asarray(
            raw["heel_strike"],
            dtype=float,
        ),
        "genuine_heel_strike": np.asarray(raw["heel_strike"], dtype=float),
        "genuine_heel_strike_confirmed": np.asarray(
            raw["heel_strike"],
            dtype=float,
        ),
        "genuine_toe_off": np.asarray(raw["toe_off"], dtype=float),
        "censored_records": (),
        "complete_cycles": tuple(complete_cycles),
        "incomplete_cycles": tuple(incomplete_cycles),
        "common_last_sample_s": float(trace.times_s[-1]),
        "left_boundary_censored": False,
        "right_boundary_censored": False,
        "conditioning": {
            "algorithm": "linear_interpolated_historical_raw",
            "threshold_n": float(threshold_n),
            "boundary_censor_policy": "historical_diagnostic_preserved",
        },
    }


def _event_matches(
    reference: Mapping[str, np.ndarray],
    predicted: Mapping[str, np.ndarray],
    *,
    tolerance_s: float,
) -> dict[str, Any]:
    return {
        name: _json_finite(
            _match_events(reference[name], predicted[name], tolerance_s)
        )
        for name in ("heel_strike", "toe_off")
    }


def _cycle_index(event_time_s: float, heel_strikes: np.ndarray) -> int | None:
    if heel_strikes.size < 2:
        return None
    index = int(np.searchsorted(heel_strikes, event_time_s, side="right") - 1)
    if index < 0 or index >= len(heel_strikes) - 1:
        return None
    return index


def _complete_cycle_index(
    event_time_s: float,
    events: Mapping[str, Any],
) -> int | None:
    """Map a time only into a scientifically accepted complete cycle."""

    for record in events.get("complete_cycles", ()):
        start = float(record["cycle_start_s"])
        end = float(record["cycle_end_s"])
        if (
            event_time_s >= start - _TIME_GRID_ATOL_S
            and event_time_s < end - _TIME_GRID_ATOL_S
        ):
            return int(record["cycle_index"])
    return None


def _detector_event_cycle_index(
    event: Mapping[str, Any],
    oracle_events: Mapping[str, Any],
) -> int | None:
    """Associate one detector event with a complete oracle HS-to-HS cycle."""

    confirmed = float(event["confirmed_time_s"])
    complete_index = _complete_cycle_index(confirmed, oracle_events)
    if complete_index is not None or oracle_events.get("complete_cycles"):
        return complete_index
    return _cycle_index(
        confirmed,
        np.asarray(oracle_events.get("heel_strike", ()), dtype=float),
    )


_EVENT_CSV_FIELDS = (
    "unit_id",
    "trial_id",
    "plateau_id",
    "cadence_s",
    "input_mode",
    "evidence_role",
    "side",
    "source",
    "signal",
    "event_name",
    "threshold",
    "event_time_s",
    "confirmed_time_s",
    "cycle_index",
    "conditioning",
    "boundary_censored",
    "boundary",
    "diagnostic_only",
    "excluded_from_gating",
)


def event_rows(trace: UnitTrace) -> list[dict[str, Any]]:
    """Build oracle, mesh, sphere, primary-threshold, and detector event rows."""

    rows: list[dict[str, Any]] = []
    oracle_by_side: dict[str, Mapping[str, Any]] = {}

    def base_row(
        *,
        side: str,
        source: str,
        signal: str,
        event_name: str,
        threshold: float | None,
        event_time_s: float,
        confirmed_time_s: float | None,
        cycle_index: int | None,
        conditioning: str,
        boundary_censored: bool,
        boundary: str | None,
        diagnostic_only: bool,
        excluded_from_gating: bool,
    ) -> dict[str, Any]:
        return {
            "unit_id": trace.unit_id,
            "trial_id": trace.trial_id,
            "plateau_id": trace.plateau_id,
            "cadence_s": trace.cadence_s,
            "input_mode": trace.input_mode,
            "evidence_role": trace.evidence_role,
            "side": side,
            "source": source,
            "signal": signal,
            "event_name": event_name,
            "threshold": threshold,
            "event_time_s": event_time_s,
            "confirmed_time_s": confirmed_time_s,
            "cycle_index": cycle_index,
            "conditioning": conditioning,
            "boundary_censored": boundary_censored,
            "boundary": boundary,
            "diagnostic_only": diagnostic_only,
            "excluded_from_gating": excluded_from_gating,
        }

    for side, series in sorted(trace.sides.items()):
        oracle = _metric_event_bundle(
            trace,
            series.oracle_force_n[:, 1],
            ORACLE_THRESHOLD_N,
        )
        oracle_by_side[side] = oracle

        def append_raw_crossings(
            source: str,
            signal_name: str,
            threshold: float,
            signal: np.ndarray,
        ) -> None:
            for event_name, rising in (("heel_strike", True), ("toe_off", False)):
                for event_time in _crossing_times(
                    trace.times_s,
                    signal,
                    threshold,
                    rising=rising,
                ):
                    rows.append(
                        base_row(
                            side=side,
                            source=source,
                            signal=signal_name,
                            event_name=event_name,
                            threshold=float(threshold),
                            event_time_s=float(event_time),
                            confirmed_time_s=None,
                            cycle_index=_complete_cycle_index(
                                float(event_time),
                                oracle,
                            ),
                            conditioning="linear_interpolated_raw",
                            boundary_censored=False,
                            boundary=None,
                            diagnostic_only=True,
                            excluded_from_gating=True,
                        )
                    )

        def append_metric_events(
            *,
            source: str,
            signal_name: str,
            bundle: Mapping[str, Any],
        ) -> None:
            gating_hs = np.asarray(bundle["heel_strike"], dtype=float)
            gating_to = np.asarray(bundle["toe_off"], dtype=float)
            genuine_hs = np.asarray(
                bundle.get("genuine_heel_strike", gating_hs),
                dtype=float,
            )
            genuine_confirmed = np.asarray(
                bundle.get("genuine_heel_strike_confirmed", genuine_hs),
                dtype=float,
            )
            genuine_to = np.asarray(
                bundle.get("genuine_toe_off", gating_to),
                dtype=float,
            )
            emitted: set[tuple[str, float]] = set()
            for event_time, confirmed_time in zip(
                genuine_hs,
                genuine_confirmed,
            ):
                accepted = bool(
                    np.any(
                        np.isclose(
                            gating_hs,
                            event_time,
                            rtol=0.0,
                            atol=_TIME_GRID_ATOL_S,
                        )
                    )
                )
                emitted.add(("heel_strike", float(event_time)))
                rows.append(
                    base_row(
                        side=side,
                        source=source,
                        signal=signal_name,
                        event_name="heel_strike",
                        threshold=ORACLE_THRESHOLD_N,
                        event_time_s=float(event_time),
                        confirmed_time_s=float(confirmed_time),
                        cycle_index=_complete_cycle_index(
                            float(event_time),
                            bundle,
                        ),
                        conditioning=str(
                            bundle["conditioning"]["algorithm"]
                        ),
                        boundary_censored=not accepted,
                        boundary=(
                            "right_unclosed_cycle" if not accepted else None
                        ),
                        diagnostic_only=not accepted,
                        excluded_from_gating=not accepted,
                    )
                )
            for event_time in genuine_to:
                accepted = bool(
                    np.any(
                        np.isclose(
                            gating_to,
                            event_time,
                            rtol=0.0,
                            atol=_TIME_GRID_ATOL_S,
                        )
                    )
                )
                emitted.add(("toe_off", float(event_time)))
                rows.append(
                    base_row(
                        side=side,
                        source=source,
                        signal=signal_name,
                        event_name="toe_off",
                        threshold=ORACLE_THRESHOLD_N,
                        event_time_s=float(event_time),
                        confirmed_time_s=float(event_time),
                        cycle_index=_complete_cycle_index(
                            float(event_time),
                            bundle,
                        ),
                        conditioning=str(
                            bundle["conditioning"]["algorithm"]
                        ),
                        boundary_censored=not accepted,
                        boundary=(
                            "right_unclosed_cycle" if not accepted else None
                        ),
                        diagnostic_only=not accepted,
                        excluded_from_gating=not accepted,
                    )
                )
            for record in bundle.get("censored_records", ()):
                key = (
                    str(record["event_name"]),
                    float(record["event_time_s"]),
                )
                if key in emitted:
                    continue
                rows.append(
                    base_row(
                        side=side,
                        source=source,
                        signal=signal_name,
                        event_name=str(record["event_name"]),
                        threshold=ORACLE_THRESHOLD_N,
                        event_time_s=float(record["event_time_s"]),
                        confirmed_time_s=(
                            float(record["confirmed_time_s"])
                            if record.get("confirmed_time_s") is not None
                            else None
                        ),
                        cycle_index=None,
                        conditioning=str(
                            bundle["conditioning"]["algorithm"]
                        ),
                        boundary_censored=True,
                        boundary=str(record["boundary"]),
                        diagnostic_only=True,
                        excluded_from_gating=True,
                    )
                )

        append_metric_events(
            source=(
                "oracle_conditioned_20n"
                if trace.evidence_role == "development"
                else "oracle_historical_raw"
            ),
            signal_name="vertical_force_n",
            bundle=oracle,
        )
        append_raw_crossings(
            "oracle_raw_crossing_diagnostic",
            "vertical_force_n",
            ORACLE_THRESHOLD_N,
            series.oracle_force_n[:, 1],
        )
        for threshold in CROSSING_THRESHOLDS_N:
            append_raw_crossings(
                "primary_raw_crossing_diagnostic",
                "normal_force_n",
                threshold,
                series.primary_normal_force_n,
            )
        primary_20n = _metric_event_bundle(
            trace,
            series.primary_normal_force_n,
            ORACLE_THRESHOLD_N,
        )
        append_metric_events(
            source=(
                "primary_conditioned_20n"
                if trace.evidence_role == "development"
                else "primary_historical_raw"
            ),
            signal_name="normal_force_n",
            bundle=primary_20n,
        )
        if series.mesh_min_clearance_m is not None:
            append_raw_crossings(
                "mesh",
                "negative_clearance_m",
                0.0,
                -series.mesh_min_clearance_m,
            )
        for sphere in sorted(
            (item for item in trace.spheres if item.side == side),
            key=lambda item: item.name,
        ):
            append_raw_crossings(
                "primary_sphere",
                sphere.name,
                0.0,
                sphere.penetration_m,
            )
    for event in trace.detector_events:
        rows.append(
            base_row(
                side=str(event["side"]),
                source="detector_diagnostic_only",
                signal=str(event["sensor"]),
                event_name=str(event["event_name"]),
                threshold=None,
                event_time_s=float(event["onset_time_s"]),
                confirmed_time_s=float(event["confirmed_time_s"]),
                cycle_index=_detector_event_cycle_index(
                    event,
                    oracle_by_side[event["side"]],
                ),
                conditioning="detector_feature_stream",
                boundary_censored=False,
                boundary=None,
                diagnostic_only=True,
                excluded_from_gating=True,
            )
        )
    rows.sort(
        key=lambda row: (
            row["unit_id"],
            row["side"],
            float(row["event_time_s"]),
            row["source"],
            row["signal"],
        )
    )
    if any(tuple(row) != _EVENT_CSV_FIELDS for row in rows):
        raise TraceContractError("events.csv row violates the fixed schema")
    return rows


def _ratio(numerator: float, denominator: float) -> float | None:
    if not math.isfinite(numerator) or not math.isfinite(denominator):
        return None
    if abs(denominator) <= 1e-12:
        return None
    return float(numerator / denominator)


def _integral(values: np.ndarray, times: np.ndarray) -> float:
    if hasattr(np, "trapezoid"):
        return float(np.trapezoid(values, times))
    return float(np.trapz(values, times))  # pragma: no cover - NumPy < 2.


def _vector_error_metrics(
    reference: np.ndarray,
    predicted: np.ndarray,
) -> dict[str, float]:
    error = np.asarray(predicted, dtype=float) - np.asarray(reference, dtype=float)
    rmse = float(np.sqrt(np.mean(np.sum(error * error, axis=1))))
    reference_rms = float(
        np.sqrt(np.mean(np.sum(np.asarray(reference, dtype=float) ** 2, axis=1)))
    )
    return {
        "rmse": rmse,
        "nrmse": rmse / max(1.0, reference_rms),
        "max_abs_component_error": float(np.max(np.abs(error))),
    }


def _first_or_none(values: np.ndarray) -> float | None:
    return float(values[0]) if len(values) else None


def _last_or_none(values: np.ndarray) -> float | None:
    return float(values[-1]) if len(values) else None


def _finite_defined(value: Any) -> bool:
    if isinstance(value, Mapping):
        return all(_finite_defined(item) for item in value.values())
    if isinstance(value, (list, tuple)):
        return all(_finite_defined(item) for item in value)
    if value is None:
        return False
    if isinstance(value, bool):
        return True
    if isinstance(value, (int, float, np.integer, np.floating)):
        return math.isfinite(float(value))
    return True


def _cycle_metrics(
    trace: UnitTrace,
    side: str,
    oracle_events: Mapping[str, Any],
    primary_events: Mapping[str, Any],
) -> list[dict[str, Any]]:
    """Compute non-averaged physical/timing evidence for every complete cycle."""

    series = trace.sides[side]
    times = np.asarray(trace.times_s, dtype=float)
    side_spheres = [sphere for sphere in trace.spheres if sphere.side == side]
    sphere_penetration = (
        np.maximum.reduce([sphere.penetration_m for sphere in side_spheres])
        if side_spheres
        else np.zeros_like(times)
    )
    cycles: list[dict[str, Any]] = []
    oracle_heel_strikes = np.asarray(
        oracle_events["heel_strike"],
        dtype=float,
    )
    primary_heel_strikes = np.asarray(
        primary_events["heel_strike"],
        dtype=float,
    )
    primary_toe_offs = np.asarray(primary_events["toe_off"], dtype=float)
    oracle_toe_offs = np.asarray(oracle_events["toe_off"], dtype=float)
    complete_cycle_records = [
        record
        for record in oracle_events.get("complete_cycles", ())
        if int(record.get("toe_off_count", 0)) == 1
        and (
            trace.evidence_role != "development"
            or record.get("common_right_margin_ok") is True
        )
    ]
    for cycle_record in complete_cycle_records:
        cycle_index = int(cycle_record["cycle_index"])
        start = float(cycle_record["cycle_start_s"])
        end = float(cycle_record["cycle_end_s"])
        mask = (times >= start - 1e-12) & (times <= end + 1e-12)
        cycle_times = times[mask]
        if len(cycle_times) < 2:
            raise TraceContractError(
                f"cycle {cycle_index} has fewer than two samples"
            )
        oracle_vertical = series.oracle_force_n[mask, 1]
        primary_normal = series.primary_normal_force_n[mask]
        primary_hs = primary_heel_strikes[
            (primary_heel_strikes >= start - _TIME_GRID_ATOL_S)
            & (primary_heel_strikes < end - _TIME_GRID_ATOL_S)
        ]
        primary_to = primary_toe_offs[
            (primary_toe_offs > start + _TIME_GRID_ATOL_S)
            & (primary_toe_offs <= end + _TIME_GRID_ATOL_S)
        ]
        oracle_to = oracle_toe_offs[
            (oracle_toe_offs > start + _TIME_GRID_ATOL_S)
            & (oracle_toe_offs <= end + _TIME_GRID_ATOL_S)
        ]
        mesh_hs = (
            _crossing_times(
                cycle_times,
                -series.mesh_min_clearance_m[mask],
                0.0,
                rising=True,
            )
            if series.mesh_min_clearance_m is not None
            else np.asarray([], dtype=float)
        )
        sphere_hs = _crossing_times(
            cycle_times,
            sphere_penetration[mask],
            0.0,
            rising=True,
        )
        oracle_impulse = _integral(oracle_vertical, cycle_times)
        primary_impulse = _integral(primary_normal, cycle_times)
        oracle_contact = oracle_vertical > ORACLE_THRESHOLD_N
        primary_contact = primary_normal > 1e-12
        cop_mask = (
            oracle_contact
            & primary_contact
            & np.all(np.isfinite(series.oracle_cop_m[mask]), axis=1)
            & np.all(np.isfinite(series.primary_cop_m[mask]), axis=1)
        )
        cop_rmse = (
            float(
                np.sqrt(
                    np.mean(
                        (
                            series.primary_cop_m[mask][cop_mask][:, (0, 2)]
                            - series.oracle_cop_m[mask][cop_mask][:, (0, 2)]
                        )
                        ** 2
                    )
                )
            )
            if np.any(cop_mask)
            else None
        )
        first_primary = _first_or_none(primary_hs)
        first_sphere = _first_or_none(sphere_hs)
        primary_delay = (
            first_primary - start
            if first_primary is not None
            else None
        )
        oracle_to_time = _first_or_none(oracle_to)
        primary_to_time = _first_or_none(primary_to)
        toe_delay = (
            primary_to_time - oracle_to_time
            if primary_to_time is not None and oracle_to_time is not None
            else None
        )
        metrics = {
            "cycle_index": cycle_index,
            "cycle_start_s": start,
            "cycle_end_s": end,
            "oracle_toe_off_s": (
                float(cycle_record["toe_off_s"])
                if cycle_record.get("toe_off_s") is not None
                else _first_or_none(oracle_to)
            ),
            "mesh_contact_present": bool(mesh_hs.size),
            "mesh_first_contact_s": _first_or_none(mesh_hs),
            "sphere_penetration_present": bool(sphere_hs.size),
            "sphere_first_penetration_s": first_sphere,
            "mesh_to_sphere_contact_delay_s": (
                first_sphere - _first_or_none(mesh_hs)
                if first_sphere is not None and mesh_hs.size
                else None
            ),
            "primary_20n_present": bool(primary_hs.size),
            "primary_first_20n_s": first_primary,
            "primary_hs_delay_median_s": primary_delay,
            "primary_hs_delay_max_abs_s": (
                abs(primary_delay) if primary_delay is not None else None
            ),
            "primary_to_delay_median_s": toe_delay,
            "force_rise_after_penetration_s": (
                first_primary - first_sphere
                if first_primary is not None and first_sphere is not None
                else None
            ),
            "oracle_vertical_impulse_ns": oracle_impulse,
            "primary_normal_impulse_ns": primary_impulse,
            "impulse_ratio": _ratio(primary_impulse, oracle_impulse),
            "cop_horizontal_rmse_m": cop_rmse,
            "force": _vector_error_metrics(
                series.oracle_force_n[mask],
                series.primary_force_n[mask],
            ),
            "moment": _vector_error_metrics(
                series.oracle_moment_nm[mask],
                series.primary_moment_nm[mask],
            ),
            "penetration_max_m": float(
                np.max(series.primary_penetration_m[mask])
            ),
            "slip_speed_max_m_s": float(
                np.max(series.primary_slip_speed_m_s[mask])
            ),
            "closed_loop_additional_delay_s": None,
        }
        metrics["required_metrics_finite"] = _finite_defined(
            {
                "primary_hs_delay_median_s": primary_delay,
                "primary_to_delay_median_s": toe_delay,
                "impulse_ratio": metrics["impulse_ratio"],
                "cop_horizontal_rmse_m": cop_rmse,
                "force": metrics["force"],
                "moment": metrics["moment"],
                "penetration_max_m": metrics["penetration_max_m"],
                "slip_speed_max_m_s": metrics["slip_speed_max_m_s"],
            }
        )
        cycles.append(_json_finite(metrics))
    return cycles


def _side_primary_metrics(
    trace: UnitTrace,
    side: str,
    *,
    event_pairing_tolerance_s: float,
) -> dict[str, Any]:
    series = trace.sides[side]
    times = np.asarray(trace.times_s, dtype=float)
    oracle_vertical = np.asarray(series.oracle_force_n[:, 1], dtype=float)
    primary_normal = np.asarray(series.primary_normal_force_n, dtype=float)
    oracle_events = _metric_event_bundle(
        trace,
        oracle_vertical,
        ORACLE_THRESHOLD_N,
    )
    primary_events = {
        threshold: _metric_event_bundle(trace, primary_normal, threshold)
        for threshold in CROSSING_THRESHOLDS_N
    }
    matches = _event_matches(
        oracle_events,
        primary_events[20.0],
        tolerance_s=event_pairing_tolerance_s,
    )
    hs_pairs = matches["heel_strike"]["pairs"]
    to_pairs = matches["toe_off"]["pairs"]
    hs_delays = [float(pair["error_s"]) for pair in hs_pairs]
    to_delays = [float(pair["error_s"]) for pair in to_pairs]

    oracle_impulse = _integral(oracle_vertical, times)
    primary_impulse = _integral(primary_normal, times)
    oracle_contact = oracle_vertical > ORACLE_THRESHOLD_N
    primary_contact = primary_normal > 1e-12
    cop_mask = (
        oracle_contact
        & primary_contact
        & np.all(np.isfinite(series.oracle_cop_m), axis=1)
        & np.all(np.isfinite(series.primary_cop_m), axis=1)
    )
    cop_horizontal_rmse = (
        float(
            np.sqrt(
                np.mean(
                    (
                        series.primary_cop_m[cop_mask][:, (0, 2)]
                        - series.oracle_cop_m[cop_mask][:, (0, 2)]
                    )
                    ** 2
                )
            )
        )
        if np.any(cop_mask)
        else None
    )
    active = np.asarray(series.primary_in_contact, dtype=bool)
    slip_p95 = (
        float(np.percentile(series.primary_slip_speed_m_s[active], 95))
        if np.any(active)
        else 0.0
    )
    side_spheres = [sphere for sphere in trace.spheres if sphere.side == side]
    sphere_penetration = (
        np.maximum.reduce([sphere.penetration_m for sphere in side_spheres])
        if side_spheres
        else np.zeros_like(times)
    )
    sphere_crossings = _crossing_times(
        times,
        sphere_penetration,
        0.0,
        rising=True,
    )
    mesh_crossings = (
        _crossing_times(
            times,
            -series.mesh_min_clearance_m,
            0.0,
            rising=True,
        )
        if series.mesh_min_clearance_m is not None
        else np.asarray([], dtype=float)
    )
    first_sphere = _first_or_none(sphere_crossings)
    first_primary = _first_or_none(
        np.asarray(
            primary_events[20.0].get(
                "genuine_heel_strike",
                primary_events[20.0]["heel_strike"],
            ),
            dtype=float,
        )
    )
    first_mesh = _first_or_none(mesh_crossings)
    force_rise_after_penetration = (
        first_primary - first_sphere
        if first_primary is not None and first_sphere is not None
        else None
    )
    core = {
        "sample_count": int(len(times)),
        "time_start_s": float(times[0]),
        "time_end_s": float(times[-1]),
        "oracle_20n": {
            "heel_strikes_s": oracle_events["heel_strike"].tolist(),
            "heel_strike_confirmed_times_s": oracle_events[
                "heel_strike_confirmed"
            ].tolist(),
            "toe_offs_s": oracle_events["toe_off"].tolist(),
            "genuine_heel_strikes_s": oracle_events[
                "genuine_heel_strike"
            ].tolist(),
            "genuine_heel_strike_confirmed_times_s": oracle_events[
                "genuine_heel_strike_confirmed"
            ].tolist(),
            "genuine_toe_offs_s": oracle_events[
                "genuine_toe_off"
            ].tolist(),
            "complete_cycle_count": len(
                oracle_events["complete_cycles"]
            ),
            "invalid_cycle_count": sum(
                record.get("exclusion_reason") == "toe_off_count_not_one"
                for record in oracle_events["incomplete_cycles"]
            ),
            "boundary_excluded_cycle_count": sum(
                record.get("exclusion_reason")
                == "right_boundary_common_confirmation_margin"
                for record in oracle_events["incomplete_cycles"]
            ),
            "common_last_sample_s": oracle_events[
                "common_last_sample_s"
            ],
            "left_boundary_censored": oracle_events[
                "left_boundary_censored"
            ],
            "right_boundary_censored": oracle_events[
                "right_boundary_censored"
            ],
            "censored_events": _json_finite(
                oracle_events["censored_records"]
            ),
            "conditioning": dict(oracle_events["conditioning"]),
        },
        "primary_crossings": {
            f"{threshold:g}N": {
                "heel_strikes_s": primary_events[threshold]["heel_strike"].tolist(),
                "toe_offs_s": primary_events[threshold]["toe_off"].tolist(),
                "genuine_heel_strikes_s": primary_events[threshold][
                    "genuine_heel_strike"
                ].tolist(),
                "genuine_toe_offs_s": primary_events[threshold][
                    "genuine_toe_off"
                ].tolist(),
                "left_boundary_censored": primary_events[threshold][
                    "left_boundary_censored"
                ],
                "right_boundary_censored": primary_events[threshold][
                    "right_boundary_censored"
                ],
            }
            for threshold in CROSSING_THRESHOLDS_N
        },
        "primary_raw_crossings": {
            f"{threshold:g}N": {
                name: values.tolist()
                for name, values in _event_vector(
                    times,
                    primary_normal,
                    threshold,
                ).items()
            }
            for threshold in CROSSING_THRESHOLDS_N
        },
        "primary_20n_event_matching": matches,
        "primary_hs_delay_median_s": (
            float(np.median(hs_delays)) if hs_delays else None
        ),
        "primary_hs_delay_max_abs_s": (
            float(np.max(np.abs(hs_delays))) if hs_delays else None
        ),
        "primary_to_delay_median_s": (
            float(np.median(to_delays)) if to_delays else None
        ),
        "force": _vector_error_metrics(
            series.oracle_force_n,
            series.primary_force_n,
        ),
        "moment": _vector_error_metrics(
            series.oracle_moment_nm,
            series.primary_moment_nm,
        ),
        "oracle_vertical_impulse_ns": oracle_impulse,
        "primary_normal_impulse_ns": primary_impulse,
        "impulse_ratio": _ratio(primary_impulse, oracle_impulse),
        "cop_horizontal_rmse_m": cop_horizontal_rmse,
        "penetration_max_m": float(np.max(series.primary_penetration_m)),
        "penetration_p95_m": float(
            np.percentile(series.primary_penetration_m, 95)
        ),
        "slip_speed_max_m_s": float(
            np.max(series.primary_slip_speed_m_s)
        ),
        "slip_speed_p95_active_m_s": slip_p95,
        "mesh_contact_present": bool(mesh_crossings.size),
        "mesh_first_contact_s": first_mesh,
        "mesh_last_contact_s": _last_or_none(
            _crossing_times(
                times,
                -series.mesh_min_clearance_m,
                0.0,
                rising=False,
            )
            if series.mesh_min_clearance_m is not None
            else np.asarray([], dtype=float)
        ),
        "sphere_penetration_present": bool(sphere_crossings.size),
        "sphere_first_penetration_s": first_sphere,
        "mesh_to_sphere_contact_delay_s": (
            first_sphere - first_mesh
            if first_sphere is not None and first_mesh is not None
            else None
        ),
        "primary_20n_present": bool(
            primary_events[20.0]["genuine_heel_strike"].size
        ),
        "primary_first_20n_s": first_primary,
        "force_rise_after_penetration_s": force_rise_after_penetration,
        "closed_loop_additional_delay_s": None,
        "cycles": _cycle_metrics(
            trace,
            side,
            oracle_events,
            primary_events[20.0],
        ),
    }
    required_finite = {
        "force": core["force"],
        "moment": core["moment"],
        "oracle_vertical_impulse_ns": oracle_impulse,
        "primary_normal_impulse_ns": primary_impulse,
        "penetration_max_m": core["penetration_max_m"],
        "slip_speed_max_m_s": core["slip_speed_max_m_s"],
    }
    core["required_metrics_finite"] = _finite_defined(required_finite)
    return _json_finite(core)


def analyze_unit(
    trace: UnitTrace,
    *,
    event_pairing_tolerance_s: float = 0.30,
) -> dict[str, Any]:
    """Compute primary metrics with detector diagnostics kept strictly separate."""

    validate_unit_trace(trace)
    if (
        not math.isfinite(event_pairing_tolerance_s)
        or event_pairing_tolerance_s <= 0.0
    ):
        raise TraceContractError("event_pairing_tolerance_s must be positive")
    primary_metrics = {
        side: _side_primary_metrics(
            trace,
            side,
            event_pairing_tolerance_s=event_pairing_tolerance_s,
        )
        for side in sorted(trace.sides)
    }
    detector_by_side: dict[str, Any] = {}
    for side, metrics in primary_metrics.items():
        side_events = [
            event
            for event in trace.detector_events
            if event["side"] == side and event["event_name"] == "heel_strike"
        ]
        onsets = sorted(float(event["onset_time_s"]) for event in side_events)
        confirmed = sorted(
            float(event["confirmed_time_s"]) for event in side_events
        )
        oracle_events = {
            "heel_strike": np.asarray(
                metrics["oracle_20n"]["heel_strikes_s"],
                dtype=float,
            ),
            "toe_off": np.asarray(
                metrics["oracle_20n"]["toe_offs_s"],
                dtype=float,
            ),
        }
        detector_cycles: list[dict[str, Any]] = []
        for cycle in metrics.get("cycles", []):
            cycle_index = int(cycle["cycle_index"])
            cycle_events = [
                event
                for event in side_events
                if _detector_event_cycle_index(event, oracle_events)
                == cycle_index
            ]
            cycle_onsets = sorted(
                float(event["onset_time_s"]) for event in cycle_events
            )
            cycle_confirmed = sorted(
                float(event["confirmed_time_s"]) for event in cycle_events
            )
            cycle_physical = [
                value
                for value in (
                    cycle.get("mesh_first_contact_s"),
                    cycle.get("primary_first_20n_s"),
                )
                if value is not None
            ]
            detector_cycles.append(
                {
                    "cycle_index": cycle_index,
                    "heel_strike_event_count": len(cycle_events),
                    "onset_hs_first_s": (
                        cycle_onsets[0] if cycle_onsets else None
                    ),
                    "confirmed_hs_first_s": (
                        cycle_confirmed[0] if cycle_confirmed else None
                    ),
                    "onset_lead_vs_mesh_or_primary_s": (
                        min(cycle_physical) - cycle_onsets[0]
                        if cycle_onsets and cycle_physical
                        else None
                    ),
                    "confirmation_lead_vs_mesh_or_primary_s": (
                        min(cycle_physical) - cycle_confirmed[0]
                        if cycle_confirmed and cycle_physical
                        else None
                    ),
                }
            )
        physical_candidates = [
            value
            for value in (
                metrics["mesh_first_contact_s"],
                metrics["primary_first_20n_s"],
            )
            if value is not None
        ]
        cycle_onset_leads = [
            float(cycle["onset_lead_vs_mesh_or_primary_s"])
            for cycle in detector_cycles
            if cycle["onset_lead_vs_mesh_or_primary_s"] is not None
        ]
        cycle_confirmation_leads = [
            float(cycle["confirmation_lead_vs_mesh_or_primary_s"])
            for cycle in detector_cycles
            if cycle["confirmation_lead_vs_mesh_or_primary_s"] is not None
        ]
        detector_by_side[side] = {
            "onset_hs_first_s": onsets[0] if onsets else None,
            "confirmed_hs_first_s": confirmed[0] if confirmed else None,
            "onset_lead_vs_mesh_or_primary_s": (
                max(cycle_onset_leads)
                if cycle_onset_leads
                else (
                    min(physical_candidates) - onsets[0]
                    if onsets and physical_candidates
                    else None
                )
            ),
            "confirmation_lead_vs_mesh_or_primary_s": (
                max(cycle_confirmation_leads)
                if cycle_confirmation_leads
                else (
                    min(physical_candidates) - confirmed[0]
                    if confirmed and physical_candidates
                    else None
                )
            ),
            "cycles": detector_cycles,
        }
    detector_diagnostics = {
        "role": "diagnostic_only_excluded_from_primary_metric_inputs",
        "event_count": len(trace.detector_events),
        "events": _json_finite(trace.detector_events),
        "by_side": detector_by_side,
    }
    return {
        "schema_version": SCHEMA_VERSION,
        "unit_id": trace.unit_id,
        "trial_id": trace.trial_id,
        "plateau_id": trace.plateau_id,
        "cadence_s": trace.cadence_s,
        "input_mode": trace.input_mode,
        "evidence_role": trace.evidence_role,
        "reference_unit_id": trace.reference_unit_id,
        "metric_input_sha256": metric_input_fingerprint(trace),
        "primary_metrics": primary_metrics,
        "detector_diagnostics": detector_diagnostics,
    }


def _apply_closed_loop_delays(
    analyses: Mapping[str, dict[str, Any]],
) -> None:
    for unit_id, analysis in analyses.items():
        reference_id = analysis.get("reference_unit_id")
        if reference_id is None:
            continue
        reference = analyses.get(str(reference_id))
        if reference is None:
            raise TraceContractError(
                f"diagnostic unit {unit_id} has missing reference_unit_id "
                f"{reference_id!r}"
            )
        for field in ("trial_id", "plateau_id", "cadence_s"):
            if analysis[field] != reference[field]:
                raise TraceContractError(
                    f"diagnostic unit {unit_id} and reference {reference_id} "
                    f"differ in {field}"
                )
        for side, metrics in analysis["primary_metrics"].items():
            if side not in reference["primary_metrics"]:
                raise TraceContractError(
                    f"reference {reference_id} is missing side {side}"
                )
            current_delay = metrics["primary_hs_delay_median_s"]
            reference_delay = reference["primary_metrics"][side][
                "primary_hs_delay_median_s"
            ]
            metrics["closed_loop_additional_delay_s"] = (
                float(current_delay - reference_delay)
                if current_delay is not None and reference_delay is not None
                else None
            )
            reference_cycles = {
                int(cycle["cycle_index"]): cycle
                for cycle in reference["primary_metrics"][side]["cycles"]
            }
            if len(reference_cycles) != len(metrics["cycles"]):
                raise TraceContractError(
                    f"diagnostic unit {unit_id} and reference {reference_id} "
                    "have different complete-cycle counts"
                )
            for cycle in metrics["cycles"]:
                cycle_index = int(cycle["cycle_index"])
                reference_cycle = reference_cycles.get(cycle_index)
                if reference_cycle is None or not (
                    math.isclose(
                        float(cycle["cycle_start_s"]),
                        float(reference_cycle["cycle_start_s"]),
                        rel_tol=0.0,
                        abs_tol=1e-8,
                    )
                    and math.isclose(
                        float(cycle["cycle_end_s"]),
                        float(reference_cycle["cycle_end_s"]),
                        rel_tol=0.0,
                        abs_tol=1e-8,
                    )
                ):
                    raise TraceContractError(
                        f"diagnostic unit {unit_id} cycle {cycle_index} "
                        "does not match its prescribed reference window"
                    )
                cycle_delay = cycle["primary_hs_delay_median_s"]
                reference_cycle_delay = reference_cycle[
                    "primary_hs_delay_median_s"
                ]
                cycle["closed_loop_additional_delay_s"] = (
                    float(cycle_delay - reference_cycle_delay)
                    if cycle_delay is not None
                    and reference_cycle_delay is not None
                    else None
                )


def _refresh_required_metric_completeness(
    analyses: Mapping[str, dict[str, Any]],
) -> None:
    """Gate every non-optional scalar after cross-unit delays are available."""

    for analysis in analyses.values():
        for metrics in analysis["primary_metrics"].values():
            matches = metrics["primary_20n_event_matching"]
            event_complete = all(
                event["reference_count"] > 0
                and event["predicted_count"] > 0
                and event["matched_count"] > 0
                and event["precision"] == 1.0
                and event["recall"] == 1.0
                for event in matches.values()
            )
            required = {
                "force": metrics["force"],
                "moment": metrics["moment"],
                "oracle_vertical_impulse_ns": metrics[
                    "oracle_vertical_impulse_ns"
                ],
                "primary_normal_impulse_ns": metrics[
                    "primary_normal_impulse_ns"
                ],
                "impulse_ratio": metrics["impulse_ratio"],
                "cop_horizontal_rmse_m": metrics["cop_horizontal_rmse_m"],
                "primary_hs_delay_median_s": metrics[
                    "primary_hs_delay_median_s"
                ],
                "primary_hs_delay_max_abs_s": metrics[
                    "primary_hs_delay_max_abs_s"
                ],
                "primary_to_delay_median_s": metrics[
                    "primary_to_delay_median_s"
                ],
                "penetration_max_m": metrics["penetration_max_m"],
                "slip_speed_max_m_s": metrics["slip_speed_max_m_s"],
            }
            if analysis["input_mode"] in {
                "coordinate_states",
                "h0_historical",
            }:
                required["closed_loop_additional_delay_s"] = metrics[
                    "closed_loop_additional_delay_s"
                ]
            cycle_complete = bool(metrics["cycles"])
            cycle_contract_complete = bool(
                metrics["oracle_20n"]["complete_cycle_count"] > 0
                and metrics["oracle_20n"]["invalid_cycle_count"] == 0
            )
            for cycle in metrics["cycles"]:
                cycle_required = {
                    "primary_hs_delay_median_s": cycle[
                        "primary_hs_delay_median_s"
                    ],
                    "primary_to_delay_median_s": cycle[
                        "primary_to_delay_median_s"
                    ],
                    "impulse_ratio": cycle["impulse_ratio"],
                    "cop_horizontal_rmse_m": cycle["cop_horizontal_rmse_m"],
                    "force": cycle["force"],
                    "moment": cycle["moment"],
                    "penetration_max_m": cycle["penetration_max_m"],
                    "slip_speed_max_m_s": cycle["slip_speed_max_m_s"],
                }
                if analysis["input_mode"] in {
                    "coordinate_states",
                    "h0_historical",
                }:
                    cycle_required["closed_loop_additional_delay_s"] = cycle[
                        "closed_loop_additional_delay_s"
                    ]
                cycle["required_metrics_finite"] = _finite_defined(
                    cycle_required
                )
                cycle_complete = bool(
                    cycle_complete and cycle["required_metrics_finite"]
                )
            metrics["required_metrics_finite"] = bool(
                event_complete
                and cycle_complete
                and cycle_contract_complete
                and _finite_defined(required)
            )


def validate_development_cadence_event_pairs(
    traces: Sequence[UnitTrace],
) -> dict[str, Any]:
    """Fail closed unless each 1/10-ms development pair has common cycles."""

    grouped: dict[tuple[str, str], list[UnitTrace]] = {}
    for trace in traces:
        if trace.evidence_role != "development":
            continue
        validate_unit_trace(trace)
        grouped.setdefault((trace.trial_id, trace.plateau_id), []).append(
            trace
        )
    expected_groups = {
        (trial_id, plateau_id)
        for trial_id in AUTHORIZED_TRIALS
        for plateau_id in ("01", "02", "03", "04")
    }
    if set(grouped) != expected_groups:
        raise TraceContractError(
            "development cadence-pair validation requires the exact "
            "3-trial x 4-plateau matrix"
        )

    tolerance_s = max(CADENCES_S) + _TIME_GRID_ATOL_S
    pair_records: list[dict[str, Any]] = []
    for identity in sorted(grouped):
        pair = sorted(grouped[identity], key=lambda trace: trace.cadence_s)
        if (
            len(pair) != 2
            or not math.isclose(
                pair[0].cadence_s,
                min(CADENCES_S),
                rel_tol=0.0,
                abs_tol=1e-12,
            )
            or not math.isclose(
                pair[1].cadence_s,
                max(CADENCES_S),
                rel_tol=0.0,
                abs_tol=1e-12,
            )
        ):
            raise TraceContractError(
                f"development cadence pair {identity} is incomplete"
            )
        signal_counts: dict[str, int] = {}
        for signal_name, getter in (
            (
                "oracle",
                lambda trace: trace.sides[PRIMARY_SIDE].oracle_force_n[:, 1],
            ),
            (
                "primary",
                lambda trace: trace.sides[
                    PRIMARY_SIDE
                ].primary_normal_force_n,
            ),
        ):
            bundles = [
                _conditioned_event_bundle(
                    trace,
                    np.asarray(getter(trace), dtype=float),
                    ORACLE_THRESHOLD_N,
                )
                for trace in pair
            ]
            records = [
                list(bundle["complete_cycles"])
                for bundle in bundles
            ]
            if len(records[0]) != len(records[1]) or not records[0]:
                raise TraceContractError(
                    f"development cadence pair {identity} {signal_name} "
                    "has unequal or empty complete-cycle counts"
                )
            if (
                len(bundles[0]["heel_strike"])
                != len(bundles[1]["heel_strike"])
                or len(bundles[0]["toe_off"])
                != len(bundles[1]["toe_off"])
            ):
                raise TraceContractError(
                    f"development cadence pair {identity} {signal_name} "
                    "has unequal conditioned 20-N event counts"
                )
            for left, right in zip(records[0], records[1]):
                if int(left["cycle_index"]) != int(right["cycle_index"]):
                    raise TraceContractError(
                        f"development cadence pair {identity} {signal_name} "
                        "has unequal complete-cycle indices"
                    )
                for field in (
                    "cycle_start_s",
                    "cycle_end_s",
                    "toe_off_s",
                ):
                    if (
                        abs(float(left[field]) - float(right[field]))
                        > tolerance_s
                    ):
                        raise TraceContractError(
                            f"development cadence pair {identity} "
                            f"{signal_name} {field} differs by more than "
                            f"{max(CADENCES_S):g} s"
                        )
            signal_counts[signal_name] = len(records[0])
        pair_records.append(
            {
                "trial_id": identity[0],
                "plateau_id": identity[1],
                "oracle_complete_cycle_count": signal_counts["oracle"],
                "primary_complete_cycle_count": signal_counts["primary"],
            }
        )
    return {
        "status": "PASS",
        "pair_count": len(pair_records),
        "event_time_tolerance_s": max(CADENCES_S),
        "signals": ["oracle", "primary"],
        "pairs": pair_records,
        "protected_trials_opened": [],
    }


def classification_thresholds(protocol: Mapping[str, Any]) -> dict[str, float]:
    forensics = protocol.get("baseline_forensics")
    raw = (
        forensics.get("root_cause_thresholds")
        if isinstance(forensics, Mapping)
        else None
    )
    required = {
        "timing_late_s",
        "material_force_rise_lag_s",
        "detector_early_lead_s",
        "penetration_limit_m",
        "impulse_ratio_min",
        "impulse_ratio_max",
    }
    if not isinstance(raw, Mapping) or set(raw) != required:
        raise PreflightError(
            "baseline_forensics.root_cause_thresholds must contain exactly "
            f"{sorted(required)}"
        )
    result: dict[str, float] = {}
    for field in sorted(required):
        value = raw[field]
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
        ):
            raise PreflightError(f"root-cause threshold {field} must be finite")
        result[field] = float(value)
    if not (
        result["timing_late_s"] > 0.0
        and result["material_force_rise_lag_s"] > 0.0
        and result["detector_early_lead_s"] >= 0.0
        and result["penetration_limit_m"] > 0.0
        and 0.0 < result["impulse_ratio_min"] <= result["impulse_ratio_max"]
    ):
        raise PreflightError("root-cause thresholds have invalid ordering/range")
    return result


_CLASS_ORDER = (
    "geometry_ground_primary",
    "material_contact_law",
    "initialization_or_closed_loop",
    "detector_early_not_primary_error",
    "physical_primary_support",
    "unclassified",
)
_PARAMETER_GROUPS = {
    "geometry_ground_primary": "primary_ground_or_sphere_geometry",
    "material_contact_law": "primary_material_or_state_only_residual",
    "initialization_or_closed_loop": "initialization_diagnostic_only",
    "detector_early_not_primary_error": "none_primary_detector_remains_frozen",
    "physical_primary_support": "primary_support_discriminator_required",
    "unclassified": "discriminating_experiment_required",
}


def classify_root_cause(
    metrics: Mapping[str, Any],
    *,
    thresholds: Mapping[str, float],
    reference_metrics: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Deterministically apply the five preregistered 0.2 evidence rules."""

    classes: list[str] = []
    if metrics.get("required_metrics_finite") is False:
        classes.append("unclassified")
    mesh_to_sphere_delay = metrics.get("mesh_to_sphere_contact_delay_s")
    if metrics.get("mesh_contact_present") and (
        not metrics.get("sphere_penetration_present")
        or (
            mesh_to_sphere_delay is not None
            and mesh_to_sphere_delay > thresholds["timing_late_s"]
        )
    ):
        classes.append("geometry_ground_primary")
    force_rise = metrics.get("force_rise_after_penetration_s")
    if metrics.get("sphere_penetration_present") and (
        not metrics.get("primary_20n_present")
        or (
            force_rise is not None
            and force_rise > thresholds["material_force_rise_lag_s"]
        )
    ):
        classes.append("material_contact_law")
    current_delay = metrics.get("primary_hs_delay_max_abs_s")
    reference_delay = (
        reference_metrics.get("primary_hs_delay_max_abs_s")
        if reference_metrics is not None
        else None
    )
    additional = metrics.get("closed_loop_additional_delay_s")
    if (
        reference_delay is not None
        and reference_delay <= thresholds["timing_late_s"]
        and current_delay is not None
        and current_delay > thresholds["timing_late_s"]
        and additional is not None
        and additional > 0.0
    ):
        classes.append("initialization_or_closed_loop")
    detector_lead = metrics.get("detector_lead_vs_mesh_or_primary_s")
    if (
        detector_lead is not None
        and detector_lead > thresholds["detector_early_lead_s"]
    ):
        classes.append("detector_early_not_primary_error")
    impulse_ratio = metrics.get("impulse_ratio")
    timing_correct = (
        current_delay is not None
        and current_delay <= thresholds["timing_late_s"]
    )
    support_failure = (
        metrics.get("penetration_max_m", 0.0)
        > thresholds["penetration_limit_m"]
        or impulse_ratio is None
        or not (
            thresholds["impulse_ratio_min"]
            <= impulse_ratio
            <= thresholds["impulse_ratio_max"]
        )
    )
    if timing_correct and support_failure:
        classes.append("physical_primary_support")
    if not classes:
        classes.append("unclassified")
    ordered = [item for item in _CLASS_ORDER if item in classes]
    groups = sorted({_PARAMETER_GROUPS[item] for item in ordered})
    return {
        "classes": ordered,
        "modifiable_parameter_groups": groups,
        "requires_discriminating_experiment": bool(
            "unclassified" in ordered
            or "physical_primary_support" in ordered
            or {
                "geometry_ground_primary",
                "material_contact_law",
            }.issubset(ordered)
        ),
        "detector_fsm_reward_policy_frozen": True,
    }


def classify_all_units(
    analyses: Mapping[str, dict[str, Any]],
    *,
    thresholds: Mapping[str, float],
) -> dict[str, Any]:
    def build() -> dict[str, Any]:
        classifications: dict[str, Any] = {}
        for unit_id in sorted(analyses):
            analysis = analyses[unit_id]
            reference = analyses.get(str(analysis.get("reference_unit_id")))
            by_side = {}
            for side in (PRIMARY_SIDE,):
                metrics = analysis["primary_metrics"].get(side)
                if not isinstance(metrics, Mapping):
                    raise TraceContractError(
                        f"unit {unit_id!r} is missing applied-side metrics"
                    )
                reference_metrics = (
                    reference["primary_metrics"].get(side)
                    if reference is not None
                    else None
                )
                classification_metrics = dict(metrics)
                detector_side = analysis["detector_diagnostics"]["by_side"].get(
                    side,
                    {},
                )
                classification_metrics["detector_lead_vs_mesh_or_primary_s"] = (
                    detector_side.get("onset_lead_vs_mesh_or_primary_s")
                )
                unit_result = classify_root_cause(
                    classification_metrics,
                    thresholds=thresholds,
                    reference_metrics=reference_metrics,
                )
                reference_cycles = {
                    int(cycle["cycle_index"]): cycle
                    for cycle in (
                        reference_metrics.get("cycles", [])
                        if reference_metrics is not None
                        else []
                    )
                }
                detector_cycles = {
                    int(cycle["cycle_index"]): cycle
                    for cycle in detector_side.get("cycles", [])
                }
                cycle_results = []
                for cycle in metrics.get("cycles", []):
                    cycle_metrics = dict(cycle)
                    cycle_metrics[
                        "detector_lead_vs_mesh_or_primary_s"
                    ] = detector_cycles.get(
                        int(cycle["cycle_index"]),
                        {},
                    ).get("onset_lead_vs_mesh_or_primary_s")
                    cycle_results.append(
                        {
                            "cycle_index": int(cycle["cycle_index"]),
                            **classify_root_cause(
                                cycle_metrics,
                                thresholds=thresholds,
                                reference_metrics=reference_cycles.get(
                                    int(cycle["cycle_index"])
                                ),
                            ),
                        }
                    )
                if not cycle_results:
                    cycle_results = [
                        {
                            "cycle_index": None,
                            "classes": ["unclassified"],
                            "modifiable_parameter_groups": [
                                _PARAMETER_GROUPS["unclassified"]
                            ],
                            "requires_discriminating_experiment": True,
                            "detector_fsm_reward_policy_frozen": True,
                        }
                    ]
                aggregate_classes = {
                    class_name
                    for cycle_result in cycle_results
                    for class_name in cycle_result["classes"]
                }
                for diagnostic_class in (
                    "initialization_or_closed_loop",
                    "detector_early_not_primary_error",
                ):
                    if diagnostic_class in unit_result["classes"]:
                        aggregate_classes.add(diagnostic_class)
                ordered_classes = [
                    class_name
                    for class_name in _CLASS_ORDER
                    if class_name in aggregate_classes
                ]
                mixed_geometry_material = {
                    "geometry_ground_primary",
                    "material_contact_law",
                }.issubset(aggregate_classes)
                by_side[side] = {
                    "classes": ordered_classes,
                    "modifiable_parameter_groups": sorted(
                        {
                            _PARAMETER_GROUPS[class_name]
                            for class_name in ordered_classes
                        }
                    ),
                    "requires_discriminating_experiment": bool(
                        mixed_geometry_material
                        or any(
                            result["requires_discriminating_experiment"]
                            for result in cycle_results
                        )
                    ),
                    "mixed_cycle_geometry_material": mixed_geometry_material,
                    "cycles": cycle_results,
                    "detector_fsm_reward_policy_frozen": True,
                    "detector_lead_classification_basis": "onset_time_s",
                }
            classifications[unit_id] = {
                "trial_id": analysis["trial_id"],
                "plateau_id": analysis["plateau_id"],
                "cadence_s": analysis["cadence_s"],
                "input_mode": analysis["input_mode"],
                "evidence_role": analysis["evidence_role"],
                "classification_gate_role": (
                    "required"
                    if analysis["evidence_role"] == "development"
                    else "diagnostic_only"
                ),
                "sides": by_side,
            }
        return classifications

    classifications = build()
    repeated = build()
    encoded = json.dumps(
        classifications,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    deterministic = repeated == classifications
    classification_complete = all(
        analysis["primary_metrics"][side].get("required_metrics_finite") is True
        and "unclassified" not in side_result["classes"]
        and not side_result["requires_discriminating_experiment"]
        for unit_id, unit in classifications.items()
        if unit["evidence_role"] == "development"
        for side, side_result in unit["sides"].items()
        for analysis in (analyses[unit_id],)
    )
    return {
        "schema_version": SCHEMA_VERSION,
        "status": (
            "PASS"
            if deterministic and classification_complete
            else ("BLOCKED" if deterministic else "ERROR")
        ),
        "classification_deterministic": deterministic,
        "classification_complete": classification_complete,
        "classification_sha256": hashlib.sha256(encoded).hexdigest(),
        "thresholds": dict(thresholds),
        "units": classifications,
        "protected_trials_opened": [],
    }


def _same_resolved_path(observed: Path, expected: Path, *, label: str) -> None:
    if observed.resolve() != expected.resolve():
        raise TraceContractError(
            f"{label} path mismatch: setup/loader selected {observed}, "
            f"frozen record selected {expected}"
        )


def _external_data_file(external_loads_xml: Path) -> Path:
    from xml.etree import ElementTree

    try:
        root = ElementTree.parse(external_loads_xml).getroot()
    except (OSError, ElementTree.ParseError) as exc:
        raise TraceContractError(
            f"cannot parse frozen ExternalLoads XML {external_loads_xml}: {exc}"
        ) from exc
    values = [
        (node.text or "").strip()
        for node in root.iter()
        if node.tag.rsplit("}", 1)[-1].lower() == "datafile"
        and (node.text or "").strip()
    ]
    if len(values) != 1:
        raise TraceContractError(
            "ExternalLoads XML must contain exactly one non-empty datafile"
        )
    data = Path(values[0])
    return (
        data.resolve()
        if data.is_absolute()
        else (external_loads_xml.parent / data).resolve()
    )


def _plugin_loader_token(
    plugin_file: Path,
    *,
    platform_name: str | None = None,
) -> str:
    """Convert a hash-bound platform library into OpenSim's bare load token."""

    name = plugin_file.name
    lower = name.lower()
    for suffix in (".dylib", ".dll", ".so"):
        if lower.endswith(suffix):
            name = name[: -len(suffix)]
            break
    platform = os.name if platform_name is None else platform_name
    if platform != "nt" and name.startswith("lib"):
        name = name[3:]
    if not name:
        raise TraceContractError(f"cannot derive plugin token from {plugin_file}")
    return str(plugin_file.parent / name)


def _sample_subset(samples: Mapping[str, Any], indices: np.ndarray) -> dict[str, Any]:
    return {
        group: {
            name: np.asarray(values, dtype=float)[indices]
            for name, values in entries.items()
        }
        for group, entries in samples.items()
    }


def _profile_with_foot_stations(profile: Any) -> tuple[Any, dict[str, str]]:
    left = sorted(
        (sphere for sphere in profile.spheres if sphere.side == PRIMARY_SIDE),
        key=lambda sphere: sphere.name,
    )
    if not left:
        raise TraceContractError("primary profile has no left spheres")
    frames = {sphere.frame for sphere in left}
    if len(frames) != 1 or not next(iter(frames)).rstrip("/").endswith("/foot_l"):
        raise TraceContractError(
            "mesh clearance requires all left primary spheres on /bodyset/foot_l"
        )
    stations = {
        "origin": (0.0, 0.0, 0.0),
        "x": (0.1, 0.0, 0.0),
        "y": (0.0, 0.1, 0.0),
        "z": (0.0, 0.0, 0.1),
    }
    names = {
        key: f"__primary_grf_readiness_foot_l_{key}"
        for key in stations
    }
    existing = {sphere.name for sphere in profile.spheres}
    if existing.intersection(names.values()):
        raise TraceContractError("profile collides with reserved mesh station names")
    extra = tuple(
        replace(
            left[0],
            name=names[key],
            location=location,
            radius=0.001,
        )
        for key, location in stations.items()
    )
    return replace(profile, spheres=tuple(profile.spheres) + extra), names


def _hash_bound_samefile(
    observed_path: Path,
    bound_source_paths: frozenset[Path],
) -> Path | None:
    """Return the hash-verified source naming the same filesystem object.

    ``bound_source_paths`` contains only protocol records already rehashed by
    preflight.  ``samefile`` adds portable inode/file-ID identity so a
    case-variant spelling on case-insensitive macOS or Windows filesystems does
    not invalidate that binding.  Any filesystem inspection error fails closed.
    """

    for candidate in sorted(bound_source_paths, key=lambda path: str(path)):
        try:
            if os.path.samefile(observed_path, candidate):
                return candidate
        except (OSError, ValueError):
            continue
    return None


def _load_hash_bound_mesh_triangles(
    observed_path: Path,
    bound_source_paths: frozenset[Path],
    loader: Callable[[Path], Any],
) -> Any:
    """Load only through the canonical path whose hash preflight verified."""

    bound_mesh_path = _hash_bound_samefile(observed_path, bound_source_paths)
    if bound_mesh_path is None:
        raise TraceContractError(
            "plantar mesh is not hash-bound in protocol.sources: "
            f"{observed_path}"
        )
    return loader(bound_mesh_path)


def _mesh_clearance(
    *,
    setup: Any,
    samples: Mapping[str, Any],
    station_names: Mapping[str, str],
    ground_origin: np.ndarray,
    ground_normal: np.ndarray,
    bound_source_paths: frozenset[Path],
) -> np.ndarray:
    from validation.audit_two_sensor_prescribed_geometry import (
        _foot_frame_kinematics,
        _load_stl_triangles,
        _resolve_left_foot_mesh,
    )

    mesh_path = _resolve_left_foot_mesh(Path(setup.model_file)).resolve()
    vertices = _load_hash_bound_mesh_triangles(
        mesh_path,
        bound_source_paths,
        _load_stl_triangles,
    ).reshape(-1, 3)
    if not np.all(np.isfinite(vertices)):
        raise TraceContractError("plantar mesh contains non-finite vertices")
    foot_origins, rotations = _foot_frame_kinematics(samples, station_names)
    normal_local = np.einsum("nji,j->ni", rotations, ground_normal)
    support = _convex_mesh_min_support(vertices, normal_local)
    return (
        (foot_origins - ground_origin) @ ground_normal
        + support
    )


def _convex_mesh_min_support(
    vertices: Any,
    directions: Any,
    *,
    chunk_size: int = 4096,
) -> np.ndarray:
    """Exact mesh support via convex-hull vertices and bounded matrix chunks."""

    points = _finite_array(vertices, label="mesh.vertices")
    normals = _finite_array(directions, label="mesh.support_directions")
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 1:
        raise TraceContractError("mesh vertices must have shape (N, 3)")
    if normals.ndim != 2 or normals.shape[1] != 3:
        raise TraceContractError("mesh directions must have shape (M, 3)")
    if type(chunk_size) is not int or chunk_size <= 0:
        raise TraceContractError("mesh support chunk_size must be positive")
    unique = np.unique(points, axis=0)
    hull_points = unique
    if len(unique) >= 4:
        try:
            from scipy.spatial import ConvexHull, QhullError
        except ImportError:
            hull_points = unique
        else:
            try:
                hull = ConvexHull(unique)
                hull_points = unique[np.asarray(hull.vertices, dtype=int)]
            except QhullError:
                hull_points = unique
    result = np.empty(len(normals), dtype=float)
    for start in range(0, len(normals), chunk_size):
        stop = min(len(normals), start + chunk_size)
        result[start:stop] = np.min(
            normals[start:stop] @ hull_points.T,
            axis=1,
        )
    return result


def _finite_cop(
    cop: Any,
    normal_force: np.ndarray,
    ground_origin: np.ndarray,
    *,
    label: str,
) -> np.ndarray:
    result = np.asarray(cop, dtype=float).copy()
    if result.shape != (len(normal_force), 3):
        raise TraceContractError(f"{label} has an invalid COP shape")
    active = np.asarray(normal_force, dtype=float) > 1e-12
    if np.any(active & ~np.all(np.isfinite(result), axis=1)):
        raise TraceContractError(f"{label} has non-finite active-contact COP")
    result[~active] = ground_origin
    return _finite_array(result, label=label, shape=result.shape)


def _saved_online_wrench(
    path: Path,
    selected_times: np.ndarray,
) -> dict[str, dict[str, np.ndarray]]:
    from output import _read_storage_table

    source_times, columns, data = _read_storage_table(str(path))
    source_times = _finite_array(source_times, label="online_grf_sto.time")
    column_index = {name: index for index, name in enumerate(columns)}
    insertion = np.searchsorted(source_times, selected_times, side="left")
    upper = np.clip(insertion, 0, len(source_times) - 1)
    lower = np.clip(insertion - 1, 0, len(source_times) - 1)
    choose_lower = np.abs(source_times[lower] - selected_times) <= np.abs(
        source_times[upper] - selected_times
    )
    indices = np.where(choose_lower, lower, upper).astype(int)
    if np.any(np.diff(indices) <= 0):
        raise TraceContractError(
            "online_grf_sto cannot represent selected state timestamps uniquely"
        )
    error = float(np.max(np.abs(source_times[indices] - selected_times)))
    if error > 1e-8:
        raise TraceContractError(
            f"online_grf_sto/state timestamp mismatch ({error:.9g} s)"
        )

    def scalar(name: str) -> np.ndarray:
        if name not in column_index:
            raise TraceContractError(f"online_grf_sto missing column {name!r}")
        return _finite_array(
            data[indices, column_index[name]],
            label=f"online_grf_sto.{name}",
        )

    result: dict[str, dict[str, np.ndarray]] = {}
    for side in ("left", "right"):
        def vector(name: str) -> np.ndarray:
            return np.column_stack(
                [
                    scalar(f"{side}_{name}_{axis}")
                    for axis in ("x", "y", "z")
                ]
            )

        contact_raw = scalar(f"{side}_in_contact")
        if np.any(~np.isclose(contact_raw, np.round(contact_raw), atol=1e-12)) or np.any(
            (contact_raw < 0.0) | (contact_raw > 1.0)
        ):
            raise TraceContractError(
                f"online_grf_sto.{side}_in_contact must be binary"
            )
        result[side] = {
            "force": vector("force"),
            "moment": vector("moment"),
            "cop": vector("cop"),
            "normal_force": scalar(f"{side}_normal_force"),
            "penetration": scalar(f"{side}_penetration"),
            "slip_speed": scalar(f"{side}_slip_speed"),
            "in_contact": contact_raw.astype(bool),
        }
    return result


def _offline_detector_events(
    unit: Mapping[str, Any],
    root: Path,
) -> tuple[Mapping[str, Any], ...]:
    record = unit.get("detector_events_csv")
    if record is None:
        return ()
    path = _resolve_safe_record(
        record,
        root,
        label=f"{unit['unit_id']}.detector_events_csv",
    )
    _fields, rows = _read_csv_rows(path, label="detector_events_csv")
    return tuple(
        {
            "side": str(row.get("side", "")),
            "sensor": str(row.get("sensor", "")),
            "event_name": str(row.get("event_name", "")),
            "onset_time_s": _csv_float(
                row,
                "onset_time_s",
                label=f"detector_events_csv[{index}]",
            ),
            "confirmed_time_s": _csv_float(
                row,
                "confirmed_time_s",
                label=f"detector_events_csv[{index}]",
            ),
        }
        for index, row in enumerate(rows)
    )


def _raw_adapter_dependencies() -> dict[str, Callable[..., Any]]:
    """Import root and validation helpers used by the direct-CLI raw adapter."""

    from online_grf import load_online_grf_profile
    from setup_io import read_setup_xml
    from validation.validate_online_grf import (
        _calculate_wrench,
        _external_wrench,
        _sample_spheres,
        _sample_spheres_from_coordinate_states,
    )
    from validation.validate_two_sensor_forward_states import (
        _runtime_grid_indices,
    )

    return {
        "load_online_grf_profile": load_online_grf_profile,
        "read_setup_xml": read_setup_xml,
        "calculate_wrench": _calculate_wrench,
        "external_wrench": _external_wrench,
        "sample_spheres": _sample_spheres,
        "sample_spheres_from_coordinate_states": (
            _sample_spheres_from_coordinate_states
        ),
        "runtime_grid_indices": _runtime_grid_indices,
    }


def build_offline_artifact_unit(
    unit: Mapping[str, Any],
    repo_root: str | Path,
    *,
    primary_profile_path: Path,
    primary_profile_record: Mapping[str, Any],
    bound_source_paths: frozenset[Path],
) -> UnitTrace:
    """Build a canonical trace from frozen artifacts without integration."""

    root = Path(repo_root).resolve()
    paths = {
        field: _resolve_safe_record(
            unit[field],
            root,
            label=f"{unit['unit_id']}.{field}",
        )
        for field in (
            "setup",
            "ik_sto",
            "grf_sto",
            "external_loads_xml",
            "sea_plugin",
        )
    }
    if unit["input_mode"] in {"coordinate_states", "h0_historical"}:
        paths["states_sto"] = _resolve_safe_record(
            unit["states_sto"],
            root,
            label=f"{unit['unit_id']}.states_sto",
        )
    if unit["input_mode"] == "h0_historical":
        paths["online_grf_sto"] = _resolve_safe_record(
            unit["online_grf_sto"],
            root,
            label=f"{unit['unit_id']}.online_grf_sto",
        )

    try:
        dependencies = _raw_adapter_dependencies()
        load_online_grf_profile = dependencies["load_online_grf_profile"]
        read_setup_xml = dependencies["read_setup_xml"]
        _calculate_wrench = dependencies["calculate_wrench"]
        _external_wrench = dependencies["external_wrench"]
        _sample_spheres = dependencies["sample_spheres"]
        _sample_spheres_from_coordinate_states = dependencies[
            "sample_spheres_from_coordinate_states"
        ]
        _runtime_grid_indices = dependencies["runtime_grid_indices"]

        verify_loaded_primary_profile(
            primary_profile_path,
            primary_profile_record,
            root,
        )
        profile = load_online_grf_profile(
            primary_profile_path,
            required_sides=(PRIMARY_SIDE,),
        )
        if unit["evidence_role"] == "development":
            profile = replace(
                profile,
                ground=replace(
                    profile.ground,
                    surface_velocity=tuple(unit["surface_velocity_mps"]),
                ),
            )
        verify_loaded_primary_profile(
            primary_profile_path,
            primary_profile_record,
            root,
        )
        setup = read_setup_xml(paths["setup"])
        _same_resolved_path(
            Path(setup.kinematics_file),
            paths["ik_sto"],
            label="setup kinematics",
        )
        if setup.external_loads_xml is None:
            raise TraceContractError("setup does not provide ExternalLoads")
        _same_resolved_path(
            Path(setup.external_loads_xml),
            paths["external_loads_xml"],
            label="setup ExternalLoads",
        )
        _same_resolved_path(
            _external_data_file(paths["external_loads_xml"]),
            paths["grf_sto"],
            label="ExternalLoads datafile",
        )
        for label, path in (
            ("model", Path(setup.model_file).resolve()),
            ("reserve actuators", Path(setup.reserve_actuators_xml).resolve()),
        ):
            if path not in bound_source_paths:
                raise TraceContractError(
                    f"setup {label} is not hash-bound in protocol.sources: {path}"
                )

        sampling_profile, station_names = _profile_with_foot_stations(profile)
        plugin_token = _plugin_loader_token(paths["sea_plugin"])
        start_s = float(unit.get("start_s", setup.t_start))
        end_s = float(unit.get("end_s", setup.t_end))
        cadence_s = float(unit["cadence_s"])
        if (
            not math.isfinite(start_s)
            or not math.isfinite(end_s)
            or end_s <= start_s
        ):
            raise TraceContractError("unit has an invalid offline sampling window")
        if (
            start_s < float(setup.t_start) - 1e-9
            or end_s > float(setup.t_end) + 1e-9
        ):
            raise TraceContractError(
                "frozen unit window lies outside the setup time interval"
            )
        if unit["input_mode"] == "ik_prescribed":
            times = _anchored_time_grid(start_s, end_s, cadence_s)
            samples = _sample_spheres(
                setup,
                sampling_profile,
                times,
                plugin_token,
            )
        else:
            native_times, native_samples = _sample_spheres_from_coordinate_states(
                setup,
                sampling_profile,
                paths["states_sto"],
                plugin_token,
            )
            native_time_array = np.asarray(native_times, dtype=float)
            if (
                native_time_array.ndim != 1
                or native_time_array.size < 2
                or native_time_array[0] > start_s + 1e-9
                or native_time_array[-1] < end_s - 1e-9
            ):
                raise TraceContractError(
                    "saved states do not cover the complete frozen unit window"
                )
            indices, _grid_contract = _runtime_grid_indices(
                native_time_array,
                sample_dt_s=cadence_s,
                start_s=start_s,
                end_s=end_s,
            )
            times = native_time_array[indices]
            samples = _sample_subset(native_samples, indices)

        normal = np.asarray(profile.ground.normal, dtype=float)
        normal /= np.linalg.norm(normal)
        origin = np.asarray(profile.ground.origin, dtype=float)
        surface_velocity = np.asarray(
            profile.ground.surface_velocity,
            dtype=float,
        )
        oracle = _external_wrench(setup, times)
        reconstructed = _calculate_wrench(profile, samples)
        observed = (
            _saved_online_wrench(paths["online_grf_sto"], times)
            if unit["input_mode"] == "h0_historical"
            else None
        )

        spheres: list[SphereSeries] = []
        side_penetrations: dict[str, list[np.ndarray]] = {
            "left": [],
            "right": [],
        }
        side_slips: dict[str, list[np.ndarray]] = {"left": [], "right": []}
        for sphere in profile.spheres:
            center = _finite_array(
                samples["centers"][sphere.name],
                label=f"{sphere.name}.center",
                shape=(len(times), 3),
            )
            velocity = _finite_array(
                samples["velocities"][sphere.name],
                label=f"{sphere.name}.velocity",
                shape=(len(times), 3),
            )
            center_height = (center - origin) @ normal
            clearance = center_height - float(sphere.radius)
            penetration = np.maximum(0.0, -clearance)
            relative_velocity = velocity - surface_velocity
            normal_velocity = relative_velocity @ normal
            tangent = relative_velocity - normal_velocity[:, None] * normal
            slip = np.linalg.norm(tangent, axis=1)
            sphere_profile = replace(profile, spheres=(sphere,))
            sphere_wrench = _calculate_wrench(sphere_profile, samples)[sphere.side]
            normal_force = _finite_array(
                sphere_wrench["normal_force"],
                label=f"{sphere.name}.normal_force",
                shape=(len(times),),
                nonnegative=True,
            )
            spheres.append(
                SphereSeries(
                    name=sphere.name,
                    side=sphere.side,
                    normal_force_n=normal_force,
                    penetration_m=penetration,
                    clearance_m=clearance,
                    center_height_m=center_height,
                    slip_speed_m_s=slip,
                )
            )
            side_penetrations[sphere.side].append(penetration)
            side_slips[sphere.side].append(slip)

        mesh = _mesh_clearance(
            setup=setup,
            samples=samples,
            station_names=station_names,
            ground_origin=origin,
            ground_normal=normal,
            bound_source_paths=bound_source_paths,
        )
        sides: dict[str, SideSeries] = {}
        for side in ("left", "right"):
            if side == "right":
                prescribed_normal = np.maximum(
                    0.0,
                    np.asarray(oracle[side]["force"], dtype=float) @ normal,
                )
                sides[side] = SideSeries(
                    oracle_force_n=_finite_array(
                        oracle[side]["force"],
                        label="right.oracle_force",
                        shape=(len(times), 3),
                    ),
                    oracle_cop_m=_finite_array(
                        oracle[side]["cop"],
                        label="right.oracle_cop",
                        shape=(len(times), 3),
                    ),
                    oracle_moment_nm=_finite_array(
                        oracle[side]["moment"],
                        label="right.oracle_moment",
                        shape=(len(times), 3),
                    ),
                    primary_force_n=_finite_array(
                        oracle[side]["force"],
                        label="right.prescribed_support_force",
                        shape=(len(times), 3),
                    ),
                    primary_normal_force_n=prescribed_normal,
                    primary_cop_m=_finite_array(
                        oracle[side]["cop"],
                        label="right.prescribed_support_cop",
                        shape=(len(times), 3),
                    ),
                    primary_moment_nm=_finite_array(
                        oracle[side]["moment"],
                        label="right.prescribed_support_moment",
                        shape=(len(times), 3),
                    ),
                    primary_penetration_m=np.zeros(len(times), dtype=float),
                    primary_slip_speed_m_s=np.zeros(len(times), dtype=float),
                    primary_in_contact=prescribed_normal > 1e-12,
                    mesh_min_clearance_m=None,
                )
                continue
            physical_penetration = np.maximum.reduce(side_penetrations[side])
            physical_slip = np.maximum.reduce(side_slips[side])
            source = observed[side] if observed is not None else reconstructed[side]
            normal_force = _finite_array(
                source["normal_force"],
                label=f"{side}.primary_normal_force",
                shape=(len(times),),
                nonnegative=True,
            )
            penetration = (
                _finite_array(
                    source["penetration"],
                    label=f"{side}.observed_penetration",
                    shape=(len(times),),
                    nonnegative=True,
                )
                if observed is not None
                else physical_penetration
            )
            slip = (
                _finite_array(
                    source["slip_speed"],
                    label=f"{side}.observed_slip",
                    shape=(len(times),),
                    nonnegative=True,
                )
                if observed is not None
                else physical_slip
            )
            in_contact = (
                np.asarray(source["in_contact"], dtype=bool)
                if observed is not None
                else physical_penetration > 0.0
            )
            sides[side] = SideSeries(
                oracle_force_n=_finite_array(
                    oracle[side]["force"],
                    label=f"{side}.oracle_force",
                    shape=(len(times), 3),
                ),
                oracle_cop_m=_finite_array(
                    oracle[side]["cop"],
                    label=f"{side}.oracle_cop",
                    shape=(len(times), 3),
                ),
                oracle_moment_nm=_finite_array(
                    oracle[side]["moment"],
                    label=f"{side}.oracle_moment",
                    shape=(len(times), 3),
                ),
                primary_force_n=_finite_array(
                    source["force"],
                    label=f"{side}.primary_force",
                    shape=(len(times), 3),
                ),
                primary_normal_force_n=normal_force,
                primary_cop_m=_finite_cop(
                    source["cop"],
                    normal_force,
                    origin,
                    label=f"{side}.primary_cop",
                ),
                primary_moment_nm=_finite_array(
                    source["moment"],
                    label=f"{side}.primary_moment",
                    shape=(len(times), 3),
                ),
                primary_penetration_m=penetration,
                primary_slip_speed_m_s=slip,
                primary_in_contact=in_contact,
                mesh_min_clearance_m=mesh if side == PRIMARY_SIDE else None,
            )
    except TraceContractError:
        raise
    except (ImportError, OSError, ValueError, RuntimeError, KeyError, IndexError) as exc:
        raise TraceContractError(
            f"offline OpenSim adapter failed for {unit['unit_id']!r}: {exc}"
        ) from exc

    return validate_unit_trace(
        UnitTrace(
            unit_id=str(unit["unit_id"]),
            trial_id=str(unit["trial_id"]),
            plateau_id=str(unit["plateau_id"]),
            cadence_s=float(unit["cadence_s"]),
            input_mode=str(unit["input_mode"]),
            evidence_role=str(unit["evidence_role"]),
            start_s=float(unit["start_s"]),
            end_s=float(unit["end_s"]),
            times_s=np.asarray(times, dtype=float),
            sides=sides,
            spheres=tuple(spheres),
            detector_events=_offline_detector_events(unit, root),
            reference_unit_id=(
                str(unit["reference_unit_id"])
                if unit.get("reference_unit_id") is not None
                else None
            ),
            plateau_speed_mps=unit.get("plateau_speed_mps"),
            surface_velocity_mps=(
                tuple(unit["surface_velocity_mps"])
                if unit.get("surface_velocity_mps") is not None
                else None
            ),
        )
    )


def default_trace_builder(
    context: PreflightContext,
) -> Callable[[Mapping[str, Any], Path], UnitTrace]:
    """Bind the executable no-integration OpenSim adapter to one preflight."""

    bound: set[Path] = set()
    for label, record in _iter_source_records(
        context.protocol["sources"],
        label="protocol.sources",
    ):
        bound.add(_resolve_safe_record(record, context.repo_root, label=label))
    bound_paths = frozenset(path.resolve() for path in bound)

    def build(unit: Mapping[str, Any], root: Path) -> UnitTrace:
        return build_offline_artifact_unit(
            unit,
            root,
            primary_profile_path=context.primary_profile_path,
            primary_profile_record=context.primary_profile_record,
            bound_source_paths=bound_paths,
        )

    return build


def load_unit_trace(
    unit: Mapping[str, Any],
    repo_root: str | Path,
    *,
    trace_builder: Callable[[Mapping[str, Any], Path], UnitTrace] | None = None,
) -> UnitTrace:
    """Load one unit without advancing a simulation.

    Canonical precomputed traces are dependency-light and support all three
    declared input modes.  A caller may supply an offline OpenSim sampler for
    frozen IK/CoordinateStates/H0 state artifacts; the returned identity is
    checked against the frozen unit record before any metric is accepted.
    """

    root = Path(repo_root).resolve()
    if "trace_csv" in unit and "per_sphere_csv" in unit:
        return load_precomputed_unit(unit, root)
    if trace_builder is None:
        raise TraceContractError(
            f"unit {unit.get('unit_id')!r} uses {unit.get('input_mode')!r} "
            "artifacts but no offline trace_builder was supplied; convert the "
            "already-existing artifact to the canonical trace/per-sphere "
            "contract or inject a no-integration OpenSim sampler"
        )
    trace = validate_unit_trace(trace_builder(unit, root))
    expected = {
        "unit_id": str(unit["unit_id"]),
        "trial_id": str(unit["trial_id"]),
        "plateau_id": str(unit["plateau_id"]),
        "cadence_s": float(unit["cadence_s"]),
        "input_mode": str(unit["input_mode"]),
        "evidence_role": str(unit["evidence_role"]),
        "start_s": float(unit["start_s"]),
        "end_s": float(unit["end_s"]),
        "reference_unit_id": (
            str(unit["reference_unit_id"])
            if unit.get("reference_unit_id") is not None
            else None
        ),
        "plateau_speed_mps": unit.get("plateau_speed_mps"),
        "surface_velocity_mps": (
            tuple(unit["surface_velocity_mps"])
            if unit.get("surface_velocity_mps") is not None
            else None
        ),
    }
    observed = {
        field: getattr(trace, field)
        for field in expected
    }
    if observed != expected:
        raise TraceContractError(
            f"offline trace_builder identity mismatch for {unit['unit_id']!r}: "
            f"expected {expected}, observed {observed}"
        )
    return trace


def _dominant_sphere(
    spheres: Sequence[SphereSeries],
    index: int,
) -> SphereSeries:
    if not spheres:
        raise TraceContractError("cannot select a dominant sphere from an empty side")
    return min(
        spheres,
        key=lambda sphere: (-float(sphere.normal_force_n[index]), sphere.name),
    )


def trace_rows(traces: Sequence[UnitTrace]) -> Iterable[dict[str, Any]]:
    """Yield the canonical aggregate trace table in fixed-schema order."""

    for trace in sorted(traces, key=lambda item: item.unit_id):
        validate_unit_trace(trace)
        for side, series in sorted(trace.sides.items()):
            oracle_events = _metric_event_bundle(
                trace,
                series.oracle_force_n[:, 1],
                ORACLE_THRESHOLD_N,
            )
            side_spheres = sorted(
                (sphere for sphere in trace.spheres if sphere.side == side),
                key=lambda sphere: sphere.name,
            )
            for index, time_s in enumerate(trace.times_s):
                dominant = (
                    _dominant_sphere(side_spheres, index)
                    if side_spheres
                    else None
                )
                row: dict[str, Any] = {
                    "unit_id": trace.unit_id,
                    "trial_id": trace.trial_id,
                    "plateau_id": trace.plateau_id,
                    "cadence_s": trace.cadence_s,
                    "input_mode": trace.input_mode,
                    "evidence_role": trace.evidence_role,
                    "reference_unit_id": trace.reference_unit_id,
                    "time_s": float(time_s),
                    "cycle_index": _complete_cycle_index(
                        float(time_s),
                        oracle_events,
                    ),
                    "side": side,
                }
                for field, columns in _VECTOR_COLUMNS.items():
                    values = np.asarray(getattr(series, field), dtype=float)[index]
                    row.update(
                        {
                            column: float(values[component])
                            for component, column in enumerate(columns)
                        }
                    )
                for field in _SCALAR_COLUMNS:
                    row[field] = float(np.asarray(getattr(series, field))[index])
                row["primary_in_contact"] = bool(series.primary_in_contact[index])
                row["mesh_min_clearance_m"] = (
                    float(series.mesh_min_clearance_m[index])
                    if series.mesh_min_clearance_m is not None
                    else None
                )
                row.update(
                    {
                        "dominant_sphere_name": (
                            dominant.name if dominant is not None else None
                        ),
                        "dominant_sphere_normal_force_n": (
                            float(dominant.normal_force_n[index])
                            if dominant is not None
                            else None
                        ),
                        "dominant_sphere_penetration_m": (
                            float(dominant.penetration_m[index])
                            if dominant is not None
                            else None
                        ),
                        "dominant_sphere_clearance_m": (
                            float(dominant.clearance_m[index])
                            if dominant is not None
                            else None
                        ),
                        "dominant_sphere_center_height_m": (
                            float(dominant.center_height_m[index])
                            if dominant is not None
                            else None
                        ),
                        "dominant_sphere_slip_speed_m_s": (
                            float(dominant.slip_speed_m_s[index])
                            if dominant is not None
                            else None
                        ),
                    }
                )
                if tuple(row) != _TRACE_CSV_FIELDS:
                    raise TraceContractError(
                        "trace.csv row violates the fixed schema"
                    )
                yield row


def _sphere_scope_metrics(
    sphere: SphereSeries,
    times: np.ndarray,
    mask: np.ndarray,
) -> dict[str, Any]:
    """Summarize one sphere without retaining any per-sample dictionaries."""

    scope_times = np.asarray(times[mask], dtype=float)
    if scope_times.size < 2:
        raise TraceContractError("per-sphere scope has fewer than two samples")
    force = np.asarray(sphere.normal_force_n[mask], dtype=float)
    penetration = np.asarray(sphere.penetration_m[mask], dtype=float)
    clearance = np.asarray(sphere.clearance_m[mask], dtype=float)
    height = np.asarray(sphere.center_height_m[mask], dtype=float)
    slip = np.asarray(sphere.slip_speed_m_s[mask], dtype=float)
    active = penetration > 0.0
    active_indices = np.flatnonzero(active)
    episode_starts = np.flatnonzero(active & ~np.r_[False, active[:-1]])
    penetration_duration_s = float(
        np.sum(np.diff(scope_times)[active[:-1]])
    )
    force_peak_index = int(np.argmax(force))
    clearance_min_index = int(np.argmin(clearance))
    return {
        "sample_count": int(scope_times.size),
        "penetration_present": bool(active_indices.size),
        "first_penetration_time_s": (
            float(scope_times[active_indices[0]])
            if active_indices.size
            else None
        ),
        "last_penetration_time_s": (
            float(scope_times[active_indices[-1]])
            if active_indices.size
            else None
        ),
        "penetration_episode_count": int(episode_starts.size),
        "penetration_duration_s": penetration_duration_s,
        "normal_force_peak_n": float(force[force_peak_index]),
        "normal_force_peak_time_s": float(scope_times[force_peak_index]),
        "normal_force_impulse_ns": _integral(force, scope_times),
        "penetration_max_m": float(np.max(penetration)),
        "penetration_p95_active_m": (
            float(np.percentile(penetration[active], 95))
            if np.any(active)
            else 0.0
        ),
        "slip_speed_max_m_s": float(np.max(slip)),
        "slip_speed_p95_active_m_s": (
            float(np.percentile(slip[active], 95))
            if np.any(active)
            else 0.0
        ),
        "clearance_min_m": float(clearance[clearance_min_index]),
        "clearance_min_time_s": float(scope_times[clearance_min_index]),
        "center_height_at_min_clearance_m": float(
            height[clearance_min_index]
        ),
        "clearance_at_force_peak_m": float(clearance[force_peak_index]),
        "center_height_at_force_peak_m": float(height[force_peak_index]),
    }


def per_sphere_rows(traces: Sequence[UnitTrace]) -> Iterable[dict[str, Any]]:
    """Yield bounded primary-left sphere summaries, one row per cycle scope."""

    for trace in sorted(traces, key=lambda item: item.unit_id):
        validate_unit_trace(trace)
        side = PRIMARY_SIDE
        spheres = sorted(
            (sphere for sphere in trace.spheres if sphere.side == side),
            key=lambda sphere: sphere.name,
        )
        if not spheres:
            raise TraceContractError(
                f"unit {trace.unit_id!r} has no primary-left sphere evidence"
            )
        oracle = _metric_event_bundle(
            trace,
            trace.sides[side].oracle_force_n[:, 1],
            ORACLE_THRESHOLD_N,
        )
        scopes: list[dict[str, Any]] = [
            {
                **dict(record),
                "scope_kind": (
                    "COMPLETE_CONDITIONED_ORACLE_CYCLE"
                    if trace.evidence_role == "development"
                    else "COMPLETE_HISTORICAL_RAW_ORACLE_CYCLE"
                ),
                "diagnostic_only": trace.evidence_role != "development",
                "excluded_from_gating": trace.evidence_role != "development",
                "left_boundary_censored": False,
                "right_boundary_censored": False,
            }
            for record in oracle["complete_cycles"]
        ]
        if not scopes:
            scopes = [
                {
                    "cycle_index": None,
                    "cycle_start_s": float(trace.start_s),
                    "cycle_end_s": float(trace.times_s[-1]),
                    "scope_kind": "WHOLE_WINDOW_NO_COMPLETE_ORACLE_CYCLE",
                    "diagnostic_only": True,
                    "excluded_from_gating": True,
                    "left_boundary_censored": bool(
                        oracle["left_boundary_censored"]
                    ),
                    "right_boundary_censored": bool(
                        oracle["right_boundary_censored"]
                    ),
                }
            ]
        times = np.asarray(trace.times_s, dtype=float)
        for scope in scopes:
            start = float(scope["cycle_start_s"])
            end = float(scope["cycle_end_s"])
            mask = (
                (times >= start - _TIME_GRID_ATOL_S)
                & (times <= end + _TIME_GRID_ATOL_S)
            )
            summaries = {
                sphere.name: _sphere_scope_metrics(sphere, times, mask)
                for sphere in spheres
            }
            dominant = min(
                spheres,
                key=lambda sphere: (
                    -float(
                        summaries[sphere.name]["normal_force_impulse_ns"]
                    ),
                    -float(summaries[sphere.name]["normal_force_peak_n"]),
                    sphere.name,
                ),
            )
            dominant_summary = summaries[dominant.name]
            total_impulse = float(
                sum(
                    summary["normal_force_impulse_ns"]
                    for summary in summaries.values()
                )
            )
            dominance_fraction = (
                float(
                    dominant_summary["normal_force_impulse_ns"]
                    / total_impulse
                )
                if total_impulse > 1e-12
                else 0.0
            )
            for sphere in spheres:
                row = {
                    "unit_id": trace.unit_id,
                    "trial_id": trace.trial_id,
                    "plateau_id": trace.plateau_id,
                    "cadence_s": trace.cadence_s,
                    "input_mode": trace.input_mode,
                    "evidence_role": trace.evidence_role,
                    "reference_unit_id": trace.reference_unit_id,
                    "side": sphere.side,
                    "sphere_name": sphere.name,
                    "scope_kind": scope["scope_kind"],
                    "diagnostic_only": scope["diagnostic_only"],
                    "excluded_from_gating": scope["excluded_from_gating"],
                    "cycle_index": scope["cycle_index"],
                    "cycle_start_s": start,
                    "cycle_end_s": end,
                    "left_boundary_censored": scope[
                        "left_boundary_censored"
                    ],
                    "right_boundary_censored": scope[
                        "right_boundary_censored"
                    ],
                    **summaries[sphere.name],
                    "is_dominant": sphere.name == dominant.name,
                    "dominant_sphere_name": dominant.name,
                    "dominant_sphere_normal_impulse_ns": dominant_summary[
                        "normal_force_impulse_ns"
                    ],
                    "dominant_sphere_force_peak_n": dominant_summary[
                        "normal_force_peak_n"
                    ],
                    "dominance_fraction": dominance_fraction,
                }
                if tuple(row) != _PER_SPHERE_CSV_FIELDS:
                    raise TraceContractError(
                        "per_sphere.csv row violates the fixed schema"
                    )
                yield row


def _validate_reproduction_targets(
    protocol: Mapping[str, Any],
) -> list[dict[str, Any]]:
    forensics = protocol.get("baseline_forensics")
    raw = (
        forensics.get("reproduction_targets")
        if isinstance(forensics, Mapping)
        else None
    )
    if type(raw) is not list or not raw:
        raise PreflightError(
            "baseline_forensics.reproduction_targets must be a non-empty array"
        )
    result: list[dict[str, Any]] = []
    identifiers: set[str] = set()
    valid_aggregations = {"median", "max", "min"}
    for index, target in enumerate(raw):
        label = f"reproduction_targets[{index}]"
        if not isinstance(target, Mapping):
            raise PreflightError(f"{label} must be an object")
        required = {
            "target_id",
            "input_mode",
            "side",
            "metric",
            "aggregation",
            "expected_s",
            "absolute_tolerance_s",
            "evidence_role",
            "unit_ids",
        }
        if not required.issubset(target):
            raise PreflightError(f"{label} is missing required fields")
        target_id = target["target_id"]
        if not isinstance(target_id, str) or not target_id or target_id in identifiers:
            raise PreflightError(f"{label}.target_id must be unique and non-empty")
        identifiers.add(target_id)
        if target["input_mode"] not in INPUT_MODES:
            raise PreflightError(f"{label}.input_mode is unsupported")
        if target["side"] not in {"left", "right"}:
            raise PreflightError(f"{label}.side is unsupported")
        if target["metric"] not in {
            "primary_hs_delay_median_s",
            "primary_hs_delay_max_abs_s",
        }:
            raise PreflightError(f"{label}.metric is not a timing metric")
        if target["aggregation"] not in valid_aggregations:
            raise PreflightError(f"{label}.aggregation is unsupported")
        normalized = dict(target)
        for field in ("expected_s", "absolute_tolerance_s"):
            value = target[field]
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
            ):
                raise PreflightError(f"{label}.{field} must be finite")
            normalized[field] = float(value)
        if normalized["absolute_tolerance_s"] < 0.0:
            raise PreflightError(f"{label}.absolute_tolerance_s must be nonnegative")
        if target["evidence_role"] not in {
            "development",
            "diagnostic",
        }:
            raise PreflightError(f"{label}.evidence_role is invalid")
        if (
            type(target["unit_ids"]) is not list
            or not target["unit_ids"]
            or any(
                not isinstance(unit_id, str) or not unit_id
                for unit_id in target["unit_ids"]
            )
            or len(set(target["unit_ids"])) != len(target["unit_ids"])
        ):
            raise PreflightError(f"{label}.unit_ids must be a unique string array")
        result.append(normalized)
    modes = {target["input_mode"] for target in result}
    if not {"ik_prescribed", "h0_historical"}.issubset(modes):
        raise PreflightError(
            "reproduction targets must cover ik_prescribed and h0_historical"
        )
    return result


def evaluate_reproduction(
    analyses: Mapping[str, Mapping[str, Any]],
    protocol: Mapping[str, Any],
) -> dict[str, Any]:
    """Evaluate only timing targets frozen before baseline observation."""

    target_results: list[dict[str, Any]] = []
    for target in _validate_reproduction_targets(protocol):
        selected: list[tuple[str, float]] = []
        allowed_ids = set(target["unit_ids"])
        matched_ids: set[str] = set()
        missing_metric_ids: list[str] = []
        for unit_id, analysis in sorted(analyses.items()):
            if analysis["input_mode"] != target["input_mode"]:
                continue
            if analysis["evidence_role"] != target["evidence_role"]:
                continue
            if allowed_ids and unit_id not in allowed_ids:
                continue
            matched_ids.add(unit_id)
            side_metrics = analysis["primary_metrics"].get(target["side"])
            value = (
                side_metrics.get(target["metric"])
                if isinstance(side_metrics, Mapping)
                else None
            )
            if value is not None and math.isfinite(float(value)):
                selected.append((unit_id, float(value)))
            else:
                missing_metric_ids.append(unit_id)
        missing_requested_ids = sorted(allowed_ids.difference(matched_ids))
        complete = not missing_metric_ids and not missing_requested_ids
        if not selected or not complete:
            observed = None
            passed = False
        else:
            values = np.asarray([value for _unit_id, value in selected], dtype=float)
            operation = target["aggregation"]
            if operation == "median":
                observed = float(np.median(values))
            elif operation == "max":
                observed = float(np.max(values))
            else:
                observed = float(np.min(values))
            passed = (
                abs(observed - target["expected_s"])
                <= target["absolute_tolerance_s"]
            )
        target_results.append(
            {
                "target_id": target["target_id"],
                "input_mode": target["input_mode"],
                "side": target["side"],
                "metric": target["metric"],
                "aggregation": target["aggregation"],
                "expected_s": target["expected_s"],
                "absolute_tolerance_s": target["absolute_tolerance_s"],
                "observed_s": observed,
                "selected_unit_ids": [unit_id for unit_id, _value in selected],
                "missing_metric_unit_ids": missing_metric_ids,
                "missing_requested_unit_ids": missing_requested_ids,
                "status": "PASS" if passed else "ERROR",
            }
        )
    passed = all(target["status"] == "PASS" for target in target_results)
    return {
        "status": "PASS" if passed else "ERROR",
        "source_hashes_match": True,
        "targets": target_results,
        "protected_trials_opened": [],
    }


def _csv_scalar(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, (bool, np.bool_)):
        return "true" if bool(value) else "false"
    if isinstance(value, (float, np.floating)):
        number = float(value)
        if not math.isfinite(number):
            raise TraceContractError("refusing non-finite CSV output")
        return format(number, ".17g")
    if isinstance(value, (int, np.integer)):
        return str(int(value))
    if isinstance(value, str):
        return value
    raise TraceContractError(
        f"unsupported CSV output type {type(value).__name__}"
    )


def _stage_streaming_csv(
    staging_dir: Path,
    filename: str,
    rows: Iterable[Mapping[str, Any]],
    *,
    fields: Sequence[str],
    label: str,
) -> tuple[Path, int]:
    """Consume a CSV iterator once into an fsynced O_EXCL staging file."""

    canonical_fields = tuple(fields)
    if (
        not canonical_fields
        or len(canonical_fields) != len(set(canonical_fields))
        or any(not isinstance(field, str) or not field for field in canonical_fields)
    ):
        raise TraceContractError(f"{label} has an invalid fixed schema")
    path = staging_dir / filename
    descriptor: int | None = None
    created = False
    row_count = 0
    try:
        descriptor = os.open(
            path,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL,
            0o600,
        )
        created = True
        with os.fdopen(
            descriptor,
            "w",
            encoding="utf-8",
            newline="",
        ) as stream:
            descriptor = None
            writer = csv.DictWriter(
                stream,
                fieldnames=canonical_fields,
                lineterminator="\n",
                extrasaction="raise",
            )
            writer.writeheader()
            for index, row in enumerate(rows):
                if not isinstance(row, Mapping):
                    raise TraceContractError(
                        f"{label} row {index} is not an object"
                    )
                if tuple(row) != canonical_fields:
                    raise TraceContractError(
                        f"{label} row {index} does not match the fixed "
                        "column order"
                    )
                writer.writerow(
                    {
                        field: _csv_scalar(row[field])
                        for field in canonical_fields
                    }
                )
                row_count += 1
            if row_count == 0:
                raise TraceContractError(
                    f"{label} must contain at least one row"
                )
            stream.flush()
            os.fsync(stream.fileno())
    except Exception:
        if descriptor is not None:
            os.close(descriptor)
        if created:
            try:
                path.unlink()
            except FileNotFoundError:
                pass
        raise
    return path, row_count


def _stage_bytes(
    staging_dir: Path,
    filename: str,
    payload: bytes,
) -> Path:
    """Write one closed, fsynced O_EXCL byte artifact into private staging."""

    if not isinstance(payload, bytes):
        raise PrimaryAuditError(f"{filename} staging payload must be bytes")
    path = staging_dir / filename
    descriptor: int | None = None
    created = False
    try:
        descriptor = os.open(
            path,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL,
            0o600,
        )
        created = True
        with os.fdopen(descriptor, "wb") as stream:
            descriptor = None
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
    except Exception:
        if descriptor is not None:
            os.close(descriptor)
        if created:
            try:
                path.unlink()
            except FileNotFoundError:
                pass
        raise
    return path


def _create_private_staging_directory(output_dir: Path) -> Path:
    """Exclusively create a hidden sibling directory on the destination FS."""

    output_dir.parent.mkdir(parents=True, exist_ok=True)
    staging = output_dir.parent / (
        f".{output_dir.name}.stage-{os.getpid()}-{uuid.uuid4().hex}"
    )
    try:
        staging.mkdir(mode=0o700)
    except FileExistsError as exc:  # pragma: no cover - UUID collision.
        raise gate.NoClobberError(
            f"refusing occupied staging directory: {staging}"
        ) from exc
    return staging


def _fsync_directory(path: Path) -> None:
    """Best-effort POSIX directory durability; Windows has no dir fsync."""

    if os.name == "nt":  # pragma: no cover - Windows-specific behavior.
        return
    descriptor = os.open(path, os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _atomic_publish_directory_no_clobber(
    staging_dir: Path,
    output_dir: Path,
) -> None:
    """Atomically rename a complete stage without replacing any destination."""

    if not staging_dir.is_dir():
        raise PrimaryAuditError(
            f"staging directory is unavailable: {staging_dir}"
        )
    if os.path.lexists(output_dir):
        raise gate.NoClobberError(
            f"refusing occupied output directory: {output_dir}"
        )
    if os.name == "nt":  # pragma: no cover - exercised on Windows CI.
        try:
            os.rename(staging_dir, output_dir)
        except OSError as exc:
            if os.path.lexists(output_dir):
                raise gate.NoClobberError(
                    f"refusing occupied output directory: {output_dir}"
                ) from exc
            raise
        return

    libc = ctypes.CDLL(None, use_errno=True)
    source_bytes = os.fsencode(staging_dir)
    destination_bytes = os.fsencode(output_dir)
    if sys.platform == "darwin":
        rename_exclusive = getattr(libc, "renamex_np", None)
        if rename_exclusive is None:
            raise PrimaryAuditError(
                "renamex_np(RENAME_EXCL) is unavailable; refusing "
                "non-atomic publication"
            )
        rename_exclusive.argtypes = [
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_uint,
        ]
        rename_exclusive.restype = ctypes.c_int
        result = rename_exclusive(
            source_bytes,
            destination_bytes,
            0x00000004,  # RENAME_EXCL from macOS sys/stdio.h.
        )
    elif sys.platform.startswith("linux"):
        rename_noreplace = getattr(libc, "renameat2", None)
        if rename_noreplace is None:
            raise PrimaryAuditError(
                "renameat2(RENAME_NOREPLACE) is unavailable; refusing "
                "non-atomic publication"
            )
        rename_noreplace.argtypes = [
            ctypes.c_int,
            ctypes.c_char_p,
            ctypes.c_int,
            ctypes.c_char_p,
            ctypes.c_uint,
        ]
        rename_noreplace.restype = ctypes.c_int
        result = rename_noreplace(
            -100,  # AT_FDCWD.
            source_bytes,
            -100,
            destination_bytes,
            0x00000001,  # RENAME_NOREPLACE.
        )
    else:  # pragma: no cover - fail closed on unsupported POSIX targets.
        raise PrimaryAuditError(
            f"atomic no-replace directory publication is unsupported on "
            f"{sys.platform}"
        )
    if result == 0:
        return
    error_number = ctypes.get_errno()
    if error_number in {errno.EEXIST, errno.ENOTEMPTY}:
        raise gate.NoClobberError(
            f"refusing occupied output directory: {output_dir}"
        )
    raise OSError(
        error_number,
        os.strerror(error_number),
        str(output_dir),
    )


def _json_bytes(payload: Any) -> bytes:
    strict = _json_finite(payload)
    gate.loads_json_strict(
        json.dumps(strict, sort_keys=True, allow_nan=False),
        source="<forensics-output>",
    )
    return (
        json.dumps(strict, indent=2, sort_keys=True, allow_nan=False) + "\n"
    ).encode("utf-8")


def _plot_bytes(
    traces: Sequence[UnitTrace],
    analyses: Mapping[str, Mapping[str, Any]],
) -> dict[str, bytes]:
    """Render deterministic diagnostic plots entirely in memory."""

    import io

    try:
        import matplotlib

        matplotlib.use("Agg", force=True)
        from matplotlib import pyplot as plt
    except ImportError as exc:
        raise PrimaryAuditError(
            "matplotlib is required to render the two preregistered PNG artifacts"
        ) from exc

    timing_figure, timing_axis = plt.subplots(figsize=(11, 5), constrained_layout=True)
    timing_units: list[str] = []
    timing_values: list[float] = []
    timing_colors: list[str] = []
    palette = {
        "ik_prescribed": "#1f77b4",
        "coordinate_states": "#ff7f0e",
        "h0_historical": "#d62728",
    }
    for unit_id, analysis in sorted(analyses.items()):
        value = analysis["primary_metrics"][PRIMARY_SIDE][
            "primary_hs_delay_median_s"
        ]
        if value is not None:
            timing_units.append(unit_id)
            timing_values.append(1000.0 * float(value))
            timing_colors.append(palette[analysis["input_mode"]])
    timing_axis.scatter(
        range(len(timing_values)),
        timing_values,
        c=timing_colors,
        s=18,
    )
    timing_axis.axhline(0.0, color="black", linewidth=0.8)
    timing_axis.set_ylabel("Primary HS delay vs oracle [ms]")
    timing_axis.set_xlabel("Frozen audit unit (sorted)")
    timing_axis.set_title("Primary GRF timing — detector excluded")
    timing_axis.grid(True, alpha=0.25)
    timing_buffer = io.BytesIO()
    timing_figure.savefig(
        timing_buffer,
        format="png",
        dpi=150,
        metadata={"Software": "primary-grf-readiness-v1"},
    )
    plt.close(timing_figure)

    penetration_figure, penetration_axis = plt.subplots(
        figsize=(11, 5),
        constrained_layout=True,
    )
    for trace in sorted(traces, key=lambda item: item.unit_id):
        series = trace.sides[PRIMARY_SIDE]
        penetration_axis.plot(
            trace.times_s - trace.times_s[0],
            1000.0 * series.primary_penetration_m,
            color=palette[trace.input_mode],
            linewidth=0.7,
            alpha=0.45,
        )
    penetration_axis.set_ylabel("Primary penetration [mm]")
    penetration_axis.set_xlabel("Time from unit start [s]")
    penetration_axis.set_title("Primary support penetration — left applied side")
    penetration_axis.grid(True, alpha=0.25)
    penetration_buffer = io.BytesIO()
    penetration_figure.savefig(
        penetration_buffer,
        format="png",
        dpi=150,
        metadata={"Software": "primary-grf-readiness-v1"},
    )
    plt.close(penetration_figure)
    return {
        "timing_plot.png": timing_buffer.getvalue(),
        "penetration_plot.png": penetration_buffer.getvalue(),
    }


def _write_bytes_no_clobber(path: Path, payload: bytes) -> Path:
    """Publish one immutable byte stream without replacing any path."""

    if os.path.lexists(path):
        raise gate.NoClobberError(f"refusing existing output: {path}")
    staging = path.parent / f".{path.name}.tmp-{os.getpid()}-{uuid.uuid4().hex}"
    descriptor: int | None = None
    try:
        descriptor = os.open(
            staging,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL,
            0o644,
        )
        with os.fdopen(descriptor, "wb") as stream:
            descriptor = None
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.link(staging, path)
        except FileExistsError as exc:
            raise gate.NoClobberError(f"refusing existing output: {path}") from exc
    finally:
        if descriptor is not None:
            os.close(descriptor)
        try:
            staging.unlink()
        except FileNotFoundError:
            pass
    return path


def _revalidate_before_publish(context: PreflightContext) -> PreflightContext:
    """Re-hash every frozen binding immediately before output publication."""

    refreshed = preflight(
        protocol_path=context.protocol_path,
        audit_path=context.audit_path,
        ledger_path=context.ledger_path,
        receipt_path=context.receipt_path,
        repo_root=context.repo_root,
    )
    if (
        refreshed.protocol_sha256 != context.protocol_sha256
        or refreshed.audit_sha256 != context.audit_sha256
        or refreshed.ledger_sha256 != context.ledger_sha256
        or refreshed.receipt_sha256 != context.receipt_sha256
        or refreshed.output_dir != context.output_dir
    ):
        raise PreflightError("frozen run bindings changed during offline audit")
    verify_loaded_primary_profile(
        refreshed.primary_profile_path,
        refreshed.primary_profile_record,
        refreshed.repo_root,
    )
    return refreshed


def publish_forensics(
    context: PreflightContext,
    *,
    traces: Sequence[UnitTrace],
    analyses: Mapping[str, Mapping[str, Any]],
    root_cause: Mapping[str, Any],
    reproduction: Mapping[str, Any],
    loaded_profile_record: Mapping[str, Any],
    cadence_pair_validation: Mapping[str, Any] | None = None,
    plot_renderer: Callable[
        [Sequence[UnitTrace], Mapping[str, Mapping[str, Any]]],
        Mapping[str, bytes],
    ] = _plot_bytes,
) -> Path:
    """Publish the seven preregistered no-clobber artifacts."""

    if os.path.lexists(context.output_dir):
        raise gate.NoClobberError(
            f"refusing occupied baseline_forensics output: "
            f"{context.output_dir}"
        )
    expected_ids = {
        str(unit["unit_id"])
        for unit in context.authorized_units + context.diagnostic_units
    }
    trace_ids = [trace.unit_id for trace in traces]
    if len(trace_ids) != len(set(trace_ids)) or set(trace_ids) != expected_ids:
        raise TraceContractError("trace set does not exactly match frozen unit IDs")
    if set(analyses) != expected_ids:
        raise TraceContractError("analysis set does not exactly match frozen unit IDs")
    paired_cadence = (
        dict(cadence_pair_validation)
        if cadence_pair_validation is not None
        else validate_development_cadence_event_pairs(traces)
    )
    if (
        paired_cadence.get("status") != "PASS"
        or paired_cadence.get("pair_count") != 12
    ):
        raise TraceContractError(
            "development cadence-pair event validation is not PASS"
        )
    finite = all(
        analysis["primary_metrics"][PRIMARY_SIDE]["required_metrics_finite"]
        for analysis in analyses.values()
        if analysis["evidence_role"] == "development"
    )
    summary_status = (
        "PASS"
        if finite
        and root_cause.get("status") == "PASS"
        and reproduction.get("status") == "PASS"
        else (
            "ERROR"
            if reproduction.get("status") == "ERROR"
            or root_cause.get("status") == "ERROR"
            else "BLOCKED"
        )
    )
    summary: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "step_id": STEP_ID,
        "status": summary_status,
        "protocol_sha256": context.protocol_sha256,
        "global_data_access_audit_sha256": context.audit_sha256,
        "execution_ledger_sha256": context.ledger_sha256,
        "run_start_receipt_sha256": context.receipt_sha256,
        "primary_profile_loaded": dict(loaded_profile_record),
        "source_hashes_match": True,
        "required_metrics_finite": finite,
        "unit_count": len(traces),
        "authorized_development_unit_count": len(context.authorized_units),
        "diagnostic_unit_count": len(context.diagnostic_units),
        "input_modes": sorted({trace.input_mode for trace in traces}),
        "physical_contract": dict(context.protocol["physical_contract"]),
        "classified_primary_sides": [PRIMARY_SIDE],
        "detector_role": "diagnostic_only_excluded_from_primary_metric_inputs",
        "development_cadence_pair_validation": paired_cadence,
        "output_contract": dict(
            context.protocol["baseline_forensics"]["output_contract"]
        ),
        "reproduction": reproduction,
        "root_cause_status": root_cause.get("status"),
        "units": {unit_id: analyses[unit_id] for unit_id in sorted(analyses)},
        "protected_trials_opened": [],
    }
    staging_dir = _create_private_staging_directory(context.output_dir)
    stage_owned = True
    try:
        staged: dict[str, Path] = {}
        staged["trace.csv"], trace_row_count = _stage_streaming_csv(
            staging_dir,
            "trace.csv",
            trace_rows(traces),
            fields=_TRACE_CSV_FIELDS,
            label="trace.csv",
        )

        def iter_events() -> Iterable[Mapping[str, Any]]:
            for trace in sorted(traces, key=lambda item: item.unit_id):
                yield from event_rows(trace)

        staged["events.csv"], event_row_count = _stage_streaming_csv(
            staging_dir,
            "events.csv",
            iter_events(),
            fields=_EVENT_CSV_FIELDS,
            label="events.csv",
        )
        staged["per_sphere.csv"], per_sphere_row_count = (
            _stage_streaming_csv(
                staging_dir,
                "per_sphere.csv",
                per_sphere_rows(traces),
                fields=_PER_SPHERE_CSV_FIELDS,
                label="per_sphere.csv",
            )
        )
        staged["root_cause.json"] = _stage_bytes(
            staging_dir,
            "root_cause.json",
            _json_bytes(root_cause),
        )
        rendered = dict(plot_renderer(traces, analyses))
        if set(rendered) != {"timing_plot.png", "penetration_plot.png"}:
            raise PrimaryAuditError(
                "plot renderer returned an incomplete artifact set"
            )
        for name, content in rendered.items():
            if (
                not isinstance(content, bytes)
                or not content.startswith(b"\x89PNG\r\n\x1a\n")
            ):
                raise PrimaryAuditError(f"{name} is not a PNG byte stream")
            staged[name] = _stage_bytes(staging_dir, name, content)
        summary["forensics_row_counts"] = {
            "trace.csv": trace_row_count,
            "events.csv": event_row_count,
            "per_sphere.csv": per_sphere_row_count,
        }
        staged["summary.json"] = _stage_bytes(
            staging_dir,
            "summary.json",
            _json_bytes(summary),
        )
        if set(staged) != set(OUTPUT_FILENAMES):
            raise PrimaryAuditError("forensics staged artifact set is incomplete")
        _fsync_directory(staging_dir)

        # This is intentionally after every iterator has completed and every
        # artifact is closed/fsynced.  Any source/profile drift invalidates the
        # private stage before its atomic no-replace directory commit.
        refreshed = _revalidate_before_publish(context)
        for path in staged.values():
            os.chmod(path, 0o644)
        os.chmod(staging_dir, 0o755)
        _fsync_directory(staging_dir)
        _fsync_directory(refreshed.output_dir.parent)
        _atomic_publish_directory_no_clobber(
            staging_dir,
            refreshed.output_dir,
        )
        stage_owned = False
        try:
            _fsync_directory(refreshed.output_dir)
            _fsync_directory(refreshed.output_dir.parent)
        except Exception as exc:
            preserved = [
                _actual_record(
                    refreshed.output_dir / name,
                    refreshed.repo_root,
                )
                for name in OUTPUT_FILENAMES
            ]
            try:
                gate.write_failure_json(
                    refreshed.output_dir,
                    step_id=STEP_ID,
                    status="ERROR",
                    protocol_sha256=refreshed.protocol_sha256,
                    phase="post_commit_durability",
                    last_data_access={
                        "semantic_access_started": True,
                        "scope": "development_and_historical_only",
                        "protected_trials_opened": [],
                    },
                    failed_checks=[
                        "baseline_forensics_post_commit_durability"
                    ],
                    artifacts=preserved,
                    source_hashes_match=True,
                    data_access_receipts=[
                        _actual_record(
                            refreshed.receipt_path,
                            refreshed.repo_root,
                        )
                    ],
                    error=str(exc),
                )
            except Exception:
                pass
            raise
    finally:
        if stage_owned:
            shutil.rmtree(staging_dir, ignore_errors=True)
    if summary_status != "PASS":
        preserved = [
            _actual_record(
                refreshed.output_dir / name,
                refreshed.repo_root,
            )
            for name in OUTPUT_FILENAMES
        ]
        gate.write_failure_json(
            refreshed.output_dir,
            step_id=STEP_ID,
            status=summary_status,
            protocol_sha256=refreshed.protocol_sha256,
            phase="evaluate_baseline_forensics",
            last_data_access={
                "semantic_access_started": True,
                "scope": "development_and_historical_only",
                "protected_trials_opened": [],
            },
            failed_checks=[
                (
                    "baseline_reproduction"
                    if reproduction.get("status") != "PASS"
                    else "root_cause_classification"
                )
            ],
            artifacts=preserved,
            source_hashes_match=True,
            data_access_receipts=[
                _actual_record(refreshed.receipt_path, refreshed.repo_root)
            ],
            error=(
                "frozen baseline was not reproducible"
                if reproduction.get("status") != "PASS"
                else "root-cause classification requires more evidence"
            ),
        )
    return refreshed.output_dir


def _execute_offline_audit(
    context: PreflightContext,
    *,
    trace_builder: Callable[[Mapping[str, Any], Path], UnitTrace] | None = None,
    plot_renderer: Callable[
        [Sequence[UnitTrace], Mapping[str, Mapping[str, Any]]],
        Mapping[str, bytes],
    ] = _plot_bytes,
) -> Path:
    """Run only frozen offline artifact analysis; never start OpenSim/Ray."""

    loaded_profile_record = verify_loaded_primary_profile(
        context.primary_profile_path,
        context.primary_profile_record,
        context.repo_root,
    )
    profile = gate.load_json_strict(context.primary_profile_path)
    if not isinstance(profile, Mapping):
        raise PreflightError("the primary profile actually loaded is not an object")
    verify_loaded_primary_profile(
        context.primary_profile_path,
        context.primary_profile_record,
        context.repo_root,
    )
    effective_builder = trace_builder or default_trace_builder(context)
    units = context.authorized_units + context.diagnostic_units
    traces = [
        load_unit_trace(unit, context.repo_root, trace_builder=effective_builder)
        for unit in units
    ]
    cadence_pair_validation = validate_development_cadence_event_pairs(traces)
    tolerance = context.protocol["baseline_forensics"].get(
        "event_pairing_tolerance_s",
        0.30,
    )
    if (
        isinstance(tolerance, bool)
        or not isinstance(tolerance, (int, float))
        or not math.isfinite(float(tolerance))
        or float(tolerance) <= 0.0
    ):
        raise PreflightError("event_pairing_tolerance_s must be finite and positive")
    analyses = {
        trace.unit_id: analyze_unit(
            trace,
            event_pairing_tolerance_s=float(tolerance),
        )
        for trace in traces
    }
    _apply_closed_loop_delays(analyses)
    _refresh_required_metric_completeness(analyses)
    root_cause = classify_all_units(
        analyses,
        thresholds=classification_thresholds(context.protocol),
    )
    reproduction = evaluate_reproduction(analyses, context.protocol)
    return publish_forensics(
        context,
        traces=traces,
        analyses=analyses,
        root_cause=root_cause,
        reproduction=reproduction,
        loaded_profile_record=loaded_profile_record,
        cadence_pair_validation=cadence_pair_validation,
        plot_renderer=plot_renderer,
    )


def _write_prepublication_failure(
    context: PreflightContext,
    exc: Exception,
) -> None:
    if os.path.lexists(context.output_dir):
        return
    try:
        gate.claim_output_directory(context.output_dir)
        gate.write_failure_json(
            context.output_dir,
            step_id=STEP_ID,
            status=(
                "BLOCKED"
                if isinstance(exc, (PreflightError, gate.HashVerificationError))
                else "ERROR"
            ),
            protocol_sha256=context.protocol_sha256,
            phase="offline_artifact_decode_or_analysis",
            last_data_access={
                "semantic_access_started": True,
                "scope": "development_and_historical_only",
                "protected_trials_opened": [],
            },
            failed_checks=["baseline_forensics_execution"],
            artifacts=[],
            source_hashes_match=not isinstance(
                exc,
                (PreflightError, gate.HashVerificationError),
            ),
            data_access_receipts=[
                _actual_record(context.receipt_path, context.repo_root)
            ],
            error=str(exc),
        )
    except Exception:
        return


def execute_offline_audit(
    context: PreflightContext,
    *,
    trace_builder: Callable[[Mapping[str, Any], Path], UnitTrace] | None = None,
    plot_renderer: Callable[
        [Sequence[UnitTrace], Mapping[str, Mapping[str, Any]]],
        Mapping[str, bytes],
    ] = _plot_bytes,
) -> Path:
    """Execute the offline audit and preserve failure evidence on every refusal."""

    try:
        return _execute_offline_audit(
            context,
            trace_builder=trace_builder,
            plot_renderer=plot_renderer,
        )
    except Exception as exc:
        _write_prepublication_failure(context, exc)
        raise


def _write_cli_preflight_failure(
    *,
    protocol_path: str | Path,
    receipt_path: str | Path,
    repo_root: str | Path,
    exc: Exception,
    status: str,
) -> None:
    """Best-effort immutable evidence for a refusal before context creation.

    The destination is derived only from the resolved CLI receipt location,
    which must have the canonical one-run shape below ``repo_root``.  No unit
    artifact is opened and no receipt payload is trusted to choose a path.
    """

    try:
        root = Path(repo_root).resolve()
        protocol = Path(protocol_path).resolve()
        receipt = Path(receipt_path).resolve()
        runs_root = (root / "validation" / "primary_grf_runs").resolve()
        relative_receipt = receipt.relative_to(runs_root)
        if (
            len(relative_receipt.parts) != 2
            or relative_receipt.parts[1] != "run_start_receipt.json"
            or not protocol.is_file()
            or not receipt.is_file()
        ):
            return
        protocol.relative_to(root)
        output_dir = receipt.parent / "baseline_forensics"
        if os.path.lexists(output_dir):
            return
        protocol_sha = gate.sha256_file(protocol)
        receipt_record = _actual_record(receipt, root)
        gate.claim_output_directory(output_dir)
        gate.write_failure_json(
            output_dir,
            step_id=STEP_ID,
            status=status,
            protocol_sha256=protocol_sha,
            phase="preflight",
            last_data_access={
                "semantic_access_started": False,
                "protected_trials_opened": [],
            },
            failed_checks=["baseline_forensics_preflight"],
            artifacts=[],
            source_hashes_match=False,
            data_access_receipts=[receipt_record],
            error=str(exc),
        )
    except Exception:
        return


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Offline primary-GRF baseline forensics (steps 0.0-0.2)",
    )
    parser.add_argument("--protocol", required=True, type=Path)
    parser.add_argument("--audit", required=True, type=Path)
    parser.add_argument("--ledger", required=True, type=Path)
    parser.add_argument("--receipt", required=True, type=Path)
    parser.add_argument(
        "--execute-offline",
        action="store_true",
        help="sample only frozen artifacts without integration and publish forensics",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        context = preflight(
            protocol_path=args.protocol,
            audit_path=args.audit,
            ledger_path=args.ledger,
            receipt_path=args.receipt,
            repo_root=REPO_ROOT,
        )
        if args.execute_offline:
            output = execute_offline_audit(context)
            print(output.relative_to(REPO_ROOT).as_posix())
            summary = gate.load_json_strict(output / "summary.json")
            return 0 if summary.get("status") == "PASS" else 3
        else:
            print(
                json.dumps(
                    {
                        "status": "PASS",
                        "mode": "preflight_only",
                        "protocol_sha256": context.protocol_sha256,
                        "output_dir": context.output_dir.relative_to(
                            REPO_ROOT
                        ).as_posix(),
                        "protected_trials_opened": [],
                    },
                    sort_keys=True,
                )
            )
        return 0
    except (gate.GatekeeperError, PrimaryAuditError) as exc:
        _write_cli_preflight_failure(
            protocol_path=args.protocol,
            receipt_path=args.receipt,
            repo_root=REPO_ROOT,
            exc=exc,
            status="BLOCKED",
        )
        print(f"REFUSED: {exc}", file=sys.stderr)
        return 2
    except Exception as exc:  # pragma: no cover - defensive CLI boundary.
        _write_cli_preflight_failure(
            protocol_path=args.protocol,
            receipt_path=args.receipt,
            repo_root=REPO_ROOT,
            exc=exc,
            status="ERROR",
        )
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


__all__ = [
    "AUTHORIZED_TRIALS",
    "CADENCES_S",
    "INPUT_MODES",
    "PreflightContext",
    "PreflightError",
    "PrimaryAuditError",
    "SideSeries",
    "SphereSeries",
    "TraceContractError",
    "UnitTrace",
    "analyze_unit",
    "classification_thresholds",
    "classify_all_units",
    "classify_root_cause",
    "evaluate_reproduction",
    "event_rows",
    "execute_offline_audit",
    "load_precomputed_unit",
    "load_unit_trace",
    "main",
    "metric_input_fingerprint",
    "per_sphere_rows",
    "preflight",
    "publish_forensics",
    "safe_protocol_relative_path",
    "trace_rows",
    "validate_unit_trace",
    "verify_loaded_primary_profile",
]


if __name__ == "__main__":
    raise SystemExit(main())
