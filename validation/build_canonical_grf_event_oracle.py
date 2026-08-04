"""Build and validate the cadence-independent prescribed-GRF event oracle.

The builder is deliberately independent from OpenSim and from the online GRF
implementation.  It reads the one prescribed left vertical-force channel
declared by an ``ExternalLoads`` XML, requires an exact 1 ms source lattice,
and emits one immutable event ledger.  Downstream 1 ms and batched 10 ms
consumers must reuse this ledger; they must not threshold GRF again.

Only development trials 02/04/08 are addressable by the bundle CLI.  Protected
trials 05/06 and reserve trials 03/07 have no path or dispatch branch here.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import shutil
import sys
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path, PurePosixPath, PureWindowsPath
from typing import Any, Mapping, Sequence

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = Path(__file__).resolve().parent
for _path in (REPO_ROOT, VALIDATION_ROOT):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import readiness_gatekeeper as gate  # noqa: E402


SCHEMA_VERSION = 1
EVENT_CONTRACT_ID = "primary_grf_split_v1+two_sensor_highrate_v1"
AUTHORIZED_DEVELOPMENT_TRIALS = ("02", "04", "08")
FORBIDDEN_TRIALS = frozenset({"01", "03", "05", "06", "07"})
DEFAULT_CONTRACT = REPO_ROOT / "validation/two_sensor_v17_high_rate_contract.json"
DEFAULT_OUTPUT_DIR = (
    REPO_ROOT
    / "validation/canonical_event_oracles/2026-08-03_v17_development"
)
TIME_GRID_ATOL_S = 1.0e-10


class OracleContractError(ValueError):
    """Raised when an input or derived ledger violates the frozen contract."""


def _canonical_sha256(value: Any) -> str:
    try:
        encoded = json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise OracleContractError(f"non-finite/non-JSON oracle value: {exc}") from exc
    return hashlib.sha256(encoded).hexdigest()


def _finite_float(value: Any, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise OracleContractError(f"{label} must be numeric") from exc
    if not math.isfinite(result):
        raise OracleContractError(f"{label} must be finite")
    return result


def _portable_repo_path(value: Any, *, label: str) -> Path:
    if not isinstance(value, str) or not value.strip() or value != value.strip():
        raise OracleContractError(f"{label} must be a non-empty trimmed path")
    if "\\" in value or ":" in value:
        raise OracleContractError(f"{label} must use a portable relative path")
    posix = PurePosixPath(value)
    windows = PureWindowsPath(value)
    if posix.is_absolute() or windows.is_absolute() or windows.drive:
        raise OracleContractError(f"{label} must be repository-relative")
    if not posix.parts or any(part in {"", ".", ".."} for part in posix.parts):
        raise OracleContractError(f"{label} contains an unsafe path component")
    resolved = REPO_ROOT.joinpath(*posix.parts).resolve()
    try:
        resolved.relative_to(REPO_ROOT)
    except ValueError as exc:
        raise OracleContractError(f"{label} escapes the repository") from exc
    return resolved


def _source_record(path: Path) -> dict[str, Any]:
    source = path.resolve()
    try:
        relative = source.relative_to(REPO_ROOT).as_posix()
    except ValueError as exc:
        raise OracleContractError(f"source escapes repository: {source}") from exc
    if not source.is_file():
        raise OracleContractError(f"source is missing: {relative}")
    return {
        "path": relative,
        "sha256": gate.sha256_file(source),
        "size_bytes": int(source.stat().st_size),
    }


def _verify_source_record(record: Mapping[str, Any], *, label: str) -> Path:
    path = _portable_repo_path(record.get("path"), label=f"{label}.path")
    expected = record.get("sha256")
    if not gate.is_sha256(expected):
        raise OracleContractError(f"{label}.sha256 is not a canonical SHA-256")
    if not path.is_file():
        raise OracleContractError(f"{label} is missing: {path}")
    observed = gate.sha256_file(path)
    if observed != expected:
        raise OracleContractError(
            f"{label} hash mismatch: expected {expected}, observed {observed}"
        )
    return path


def _read_storage_table(path: Path) -> tuple[np.ndarray, tuple[str, ...], np.ndarray]:
    """Read a strict, finite whitespace-delimited OpenSim Storage table."""

    header: dict[str, str] = {}
    with path.open("r", encoding="utf-8") as stream:
        found_end = False
        for line_number, line in enumerate(stream, start=1):
            stripped = line.strip()
            if stripped.lower() == "endheader":
                found_end = True
                break
            match = re.match(r"^([^=\s]+)\s*(?:=|\s)\s*(.+)$", stripped)
            if match:
                header[match.group(1).lower()] = match.group(2).strip()
        if not found_end:
            raise OracleContractError(f"Storage has no endheader: {path}")

        column_line = ""
        for line in stream:
            if line.strip():
                column_line = line.strip()
                break
        if not column_line:
            raise OracleContractError(f"Storage has no column header: {path}")
        columns = tuple(re.split(r"\s+", column_line))
        if len(columns) != len(set(columns)):
            raise OracleContractError(f"Storage has duplicate columns: {path}")
        if not columns or columns[0] != "time":
            raise OracleContractError(f"Storage first column must be 'time': {path}")

        rows: list[list[float]] = []
        for line_number, line in enumerate(stream, start=line_number + 2):
            stripped = line.strip()
            if not stripped:
                continue
            fields = re.split(r"\s+", stripped)
            if len(fields) != len(columns):
                raise OracleContractError(
                    f"Storage row {line_number} has {len(fields)} fields; "
                    f"expected {len(columns)}"
                )
            try:
                row = [float(field) for field in fields]
            except ValueError as exc:
                raise OracleContractError(
                    f"Storage row {line_number} contains a non-number"
                ) from exc
            if not all(math.isfinite(value) for value in row):
                raise OracleContractError(
                    f"Storage row {line_number} contains a non-finite value"
                )
            rows.append(row)

    if not rows:
        raise OracleContractError(f"Storage contains no data rows: {path}")
    data = np.asarray(rows, dtype=float)
    declared_columns = header.get("datacolumns") or header.get("ncolumns")
    declared_rows = header.get("datarows") or header.get("nrows")
    if declared_columns is not None and int(declared_columns) != data.shape[1]:
        raise OracleContractError("Storage declared column count does not match data")
    if declared_rows is not None and int(declared_rows) != data.shape[0]:
        raise OracleContractError("Storage declared row count does not match data")
    return data[:, 0].copy(), columns, data


def resolve_prescribed_left_vertical_source(
    external_loads_xml: Path,
) -> tuple[Path, str]:
    """Resolve exactly one left ExternalForce and its +Y force column."""

    try:
        root = ET.parse(external_loads_xml).getroot()
    except (ET.ParseError, OSError) as exc:
        raise OracleContractError(
            f"cannot parse ExternalLoads XML {external_loads_xml}: {exc}"
        ) from exc
    candidates: list[tuple[str, str]] = []
    for external_force in root.findall(".//ExternalForce"):
        name = str(external_force.attrib.get("name", ""))
        body = str(external_force.findtext("applied_to_body", "")).strip().lower()
        identifier = str(external_force.findtext("force_identifier", "")).strip()
        if body in {"foot_l", "/bodyset/foot_l"}:
            candidates.append((name, identifier))
    if len(candidates) != 1 or not candidates[0][1]:
        raise OracleContractError(
            "ExternalLoads must declare exactly one left foot force with a "
            f"non-empty identifier; observed {candidates}"
        )
    data_file_text = str(root.findtext(".//datafile", "")).strip()
    if not data_file_text:
        raise OracleContractError("ExternalLoads XML has no datafile")
    data_fragment = PurePosixPath(data_file_text.replace("\\", "/"))
    if data_fragment.is_absolute() or any(part == ".." for part in data_fragment.parts):
        raise OracleContractError("ExternalLoads datafile must be local and relative")
    data_path = external_loads_xml.parent.joinpath(*data_fragment.parts).resolve()
    if not data_path.is_file():
        raise OracleContractError(f"prescribed GRF datafile is missing: {data_path}")
    return data_path, f"{candidates[0][1]}y"


def validate_time_grid(
    times_s: Sequence[float] | np.ndarray,
    *,
    expected_dt_s: float = 0.001,
) -> np.ndarray:
    times = np.asarray(times_s, dtype=float)
    dt = _finite_float(expected_dt_s, "expected_dt_s")
    if times.ndim != 1 or times.size < 2 or not np.all(np.isfinite(times)):
        raise OracleContractError("time grid must be finite, one-dimensional and non-empty")
    differences = np.diff(times)
    if np.any(differences <= 0.0):
        raise OracleContractError("time grid must be strictly increasing and unique")
    if dt <= 0.0 or not np.allclose(differences, dt, rtol=0.0, atol=TIME_GRID_ATOL_S):
        observed = (float(np.min(differences)), float(np.max(differences)))
        raise OracleContractError(
            f"time grid must be exactly {dt:.9f} s; observed range {observed}"
        )
    return times


def _contact_runs(contact: np.ndarray) -> list[tuple[int, int]]:
    changes = np.diff(contact.astype(np.int8), prepend=np.int8(0), append=np.int8(0))
    starts = np.flatnonzero(changes == 1)
    ends = np.flatnonzero(changes == -1)
    if starts.size != ends.size:
        raise OracleContractError("internal contact-run extraction mismatch")
    return [(int(start), int(end)) for start, end in zip(starts, ends)]


def _boundary_state(
    contacts: Sequence[Mapping[str, Any]],
    events: Sequence[Mapping[str, Any]],
    *,
    time_s: float,
    side: str,
) -> dict[str, Any]:
    tolerance = TIME_GRID_ATOL_S
    active_ids: list[str] = []
    for contact in contacts:
        observed_start = float(contact["observed_start_time_s"])
        toe_off = contact["toe_off_time_s"]
        active = observed_start <= time_s + tolerance and (
            toe_off is None or time_s < float(toe_off) - tolerance
        )
        if active:
            active_ids.append(str(contact["contact_id"]))
    event_ids = [
        str(event["event_id"])
        for event in events
        if abs(float(event["event_time_s"]) - time_s) <= tolerance
    ]
    return {
        "boundary": side,
        "time_s": float(time_s),
        "contact_active": bool(active_ids),
        "active_contact_ids": active_ids,
        "event_ids_at_boundary": event_ids,
    }


def build_canonical_ledger(
    times_s: Sequence[float] | np.ndarray,
    left_vertical_force_n: Sequence[float] | np.ndarray,
    *,
    trial_id: str,
    source: Mapping[str, Any],
    threshold_n: float = 20.0,
    min_contact_duration_s: float = 0.05,
    min_cycle_duration_s: float = 0.30,
    sample_dt_s: float = 0.001,
) -> dict[str, Any]:
    """Derive one sample-timestamp event ledger from prescribed left Fy."""

    if trial_id not in AUTHORIZED_DEVELOPMENT_TRIALS:
        raise OracleContractError(
            f"trial {trial_id!r} is not authorized development data"
        )
    times = validate_time_grid(times_s, expected_dt_s=sample_dt_s)
    force = np.asarray(left_vertical_force_n, dtype=float)
    if force.ndim != 1 or force.shape != times.shape or not np.all(np.isfinite(force)):
        raise OracleContractError("left vertical force must be finite and match time")
    threshold = _finite_float(threshold_n, "threshold_n")
    min_contact = _finite_float(min_contact_duration_s, "min_contact_duration_s")
    min_cycle = _finite_float(min_cycle_duration_s, "min_cycle_duration_s")
    if threshold < 0.0 or min_contact <= 0.0 or min_cycle <= 0.0:
        raise OracleContractError("oracle thresholds/durations must be positive")

    contact_mask = force > threshold
    contacts: list[dict[str, Any]] = []
    rejected: list[dict[str, Any]] = []
    events: list[dict[str, Any]] = []
    accepted_contacts: list[dict[str, Any]] = []

    for ordinal, (start, end_exclusive) in enumerate(_contact_runs(contact_mask), start=1):
        contact_id = f"contact_{ordinal:04d}"
        left_boundary_censored = start == 0 and bool(contact_mask[0])
        right_boundary_open = end_exclusive == times.size
        observed_start = float(times[start])
        toe_off_time = (
            None if right_boundary_open else float(times[end_exclusive])
        )
        duration = (
            # At the right boundary there is no following sample, so only the
            # elapsed observed support can prove persistence.  Never assume an
            # extra sample interval beyond the source.
            float(times[-1] - times[start])
            if right_boundary_open
            else float(times[end_exclusive] - times[start])
        )
        if left_boundary_censored:
            contacts.append(
                {
                    "contact_id": contact_id,
                    "status": "left_boundary_censored",
                    "observed_start_time_s": observed_start,
                    "onset_time_s": None,
                    "toe_off_time_s": toe_off_time,
                    "observed_duration_s": duration,
                    "right_boundary_open": right_boundary_open,
                }
            )
            continue
        if duration + TIME_GRID_ATOL_S < min_contact:
            item = {
                "contact_id": contact_id,
                "status": "rejected_short_contact",
                "observed_start_time_s": observed_start,
                "onset_time_s": observed_start,
                "toe_off_time_s": toe_off_time,
                "observed_duration_s": duration,
                "right_boundary_open": right_boundary_open,
            }
            contacts.append(item)
            rejected.append(item.copy())
            continue

        if right_boundary_open:
            confirmation_time = observed_start + min_contact
            confirmation_indices = np.flatnonzero(
                times[start:] >= confirmation_time - TIME_GRID_ATOL_S
            )
            if confirmation_indices.size == 0:
                raise OracleContractError("open contact cannot satisfy persistence")
            confirmation_index = start + int(confirmation_indices[0])
            confirmation_time = float(times[confirmation_index])
        else:
            confirmation_time = min(observed_start + min_contact, float(toe_off_time))

        hs_id = f"hs_{len(accepted_contacts) + 1:04d}"
        hs_event = {
            "event_id": hs_id,
            "event": "heel_strike",
            "event_time_s": observed_start,
            "sample_index": int(start),
            "persistence_confirmed_time_s": float(confirmation_time),
            "contact_id": contact_id,
            "boundary_role": "interior",
        }
        events.append(hs_event)
        to_id: str | None = None
        if toe_off_time is not None:
            to_id = f"to_{len(accepted_contacts) + 1:04d}"
            events.append(
                {
                    "event_id": to_id,
                    "event": "toe_off",
                    "event_time_s": float(toe_off_time),
                    "sample_index": int(end_exclusive),
                    "contact_id": contact_id,
                    "boundary_role": "interior",
                }
            )
        item = {
            "contact_id": contact_id,
            "status": "accepted",
            "observed_start_time_s": observed_start,
            "onset_time_s": observed_start,
            "toe_off_time_s": toe_off_time,
            "observed_duration_s": duration,
            "persistence_confirmed_time_s": float(confirmation_time),
            "right_boundary_open": right_boundary_open,
            "heel_strike_event_id": hs_id,
            "toe_off_event_id": to_id,
        }
        contacts.append(item)
        accepted_contacts.append(item)

    events.sort(key=lambda item: (float(item["event_time_s"]), item["event"]))
    cycles: list[dict[str, Any]] = []
    for index, (current, following) in enumerate(
        zip(accepted_contacts, accepted_contacts[1:]), start=1
    ):
        hs = float(current["onset_time_s"])
        next_hs = float(following["onset_time_s"])
        duration = next_hs - hs
        if duration + TIME_GRID_ATOL_S < min_cycle:
            raise OracleContractError(
                f"accepted HS interval {duration:.9f} s is below {min_cycle:.9f} s"
            )
        toe_off = current["toe_off_time_s"]
        if toe_off is None or not (hs < float(toe_off) < next_hs):
            raise OracleContractError("accepted contacts do not form strict HS-TO-HS")
        cycles.append(
            {
                "cycle_id": f"cycle_{index:04d}",
                "heel_strike_time_s": hs,
                "toe_off_time_s": float(toe_off),
                "next_heel_strike_time_s": next_hs,
                "duration_s": duration,
                "heel_strike_event_id": current["heel_strike_event_id"],
                "toe_off_event_id": current["toe_off_event_id"],
                "next_heel_strike_event_id": following["heel_strike_event_id"],
            }
        )

    boundaries = {
        "left": _boundary_state(
            contacts, events, time_s=float(times[0]), side="left"
        ),
        "right": _boundary_state(
            contacts, events, time_s=float(times[-1]), side="right"
        ),
        "policy": {
            "left_active_contact": "censor_unknown_onset_and_its_toe_off",
            "right_active_contact": "retain_interior_hs_but_emit_no_synthetic_toe_off",
            "event_at_exact_boundary": "record_explicitly_in_event_ids_at_boundary",
        },
    }
    core = {
        "event_contract_id": EVENT_CONTRACT_ID,
        "trial_id": trial_id,
        "sample_dt_s": float(sample_dt_s),
        "threshold_n": threshold,
        "min_contact_duration_s": min_contact,
        "min_cycle_duration_s": min_cycle,
        "time_grid": {
            "start_s": float(times[0]),
            "end_s": float(times[-1]),
            "sample_count": int(times.size),
            "strictly_increasing_unique": True,
        },
        "contacts": contacts,
        "rejected_contacts": rejected,
        "events": events,
        "cycles": cycles,
        "boundaries": boundaries,
    }
    return {
        "schema_version": SCHEMA_VERSION,
        "ledger_kind": "canonical_prescribed_left_fy_event_oracle",
        "source": dict(source),
        "scientific_core": core,
        "scientific_core_sha256": _canonical_sha256(core),
    }


def add_window_views(
    ledger: Mapping[str, Any],
    windows: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    """Add cadence-independent complete-cycle views without re-thresholding."""

    result = dict(ledger)
    core = dict(result["scientific_core"])
    contacts = list(core["contacts"])
    events = list(core["events"])
    cycles = list(core["cycles"])
    grid_start = float(core["time_grid"]["start_s"])
    grid_end = float(core["time_grid"]["end_s"])
    dt = float(core["sample_dt_s"])
    views: list[dict[str, Any]] = []
    prior_end = -math.inf
    for ordinal, window in enumerate(windows, start=1):
        view_id = str(window.get("view_id", f"plateau_{ordinal:02d}"))
        start = _finite_float(window.get("start_s"), f"{view_id}.start_s")
        end = _finite_float(window.get("end_s"), f"{view_id}.end_s")
        if not (grid_start <= start < end <= grid_end):
            raise OracleContractError(f"{view_id} escapes the source time grid")
        if start < prior_end - TIME_GRID_ATOL_S:
            raise OracleContractError("oracle windows must be ordered and non-overlapping")
        prior_end = end
        for value, label in ((start, "start"), (end, "end")):
            lattice = (value - grid_start) / dt
            if abs(lattice - round(lattice)) > TIME_GRID_ATOL_S / dt:
                raise OracleContractError(f"{view_id}.{label} is off the 1 ms lattice")

        owned_cycles = [
            cycle
            for cycle in cycles
            if float(cycle["heel_strike_time_s"]) >= start - TIME_GRID_ATOL_S
            and float(cycle["next_heel_strike_time_s"]) <= end + TIME_GRID_ATOL_S
        ]
        event_ids: list[str] = []
        for cycle in owned_cycles:
            for key in (
                "heel_strike_event_id",
                "toe_off_event_id",
                "next_heel_strike_event_id",
            ):
                event_id = str(cycle[key])
                if event_id not in event_ids:
                    event_ids.append(event_id)
        by_id = {str(event["event_id"]): event for event in events}
        scoreable_events = [by_id[event_id] for event_id in event_ids]
        scoreable_events.sort(
            key=lambda item: (float(item["event_time_s"]), item["event"])
        )
        view = {
            "view_id": view_id,
            "interval_s": [start, end],
            "speed_mps": _finite_float(window.get("speed_mps"), f"{view_id}.speed_mps"),
            "left_boundary": _boundary_state(
                contacts, events, time_s=start, side="left"
            ),
            "right_boundary": _boundary_state(
                contacts, events, time_s=end, side="right"
            ),
            "complete_cycles": owned_cycles,
            "scoreable_events": scoreable_events,
            "counts": {
                "complete_cycles": len(owned_cycles),
                "heel_strike": sum(
                    event["event"] == "heel_strike" for event in scoreable_events
                ),
                "toe_off": sum(
                    event["event"] == "toe_off" for event in scoreable_events
                ),
            },
            "consumer_contract": (
                "reuse scoreable_events verbatim at both cadences; never "
                "threshold prescribed GRF in a consumer"
            ),
        }
        view["view_sha256"] = _canonical_sha256(view)
        views.append(view)

    core["views"] = views
    result["scientific_core"] = core
    result["scientific_core_sha256"] = _canonical_sha256(core)
    return result


def validate_ledger(payload: Mapping[str, Any]) -> dict[str, Any]:
    """Validate a serialized ledger without reading its original GRF."""

    if payload.get("schema_version") != SCHEMA_VERSION:
        raise OracleContractError("unsupported oracle ledger schema")
    if payload.get("ledger_kind") != "canonical_prescribed_left_fy_event_oracle":
        raise OracleContractError("unexpected oracle ledger kind")
    core = payload.get("scientific_core")
    if not isinstance(core, Mapping):
        raise OracleContractError("ledger scientific_core must be an object")
    if core.get("event_contract_id") != EVENT_CONTRACT_ID:
        raise OracleContractError("ledger event_contract_id drifted")
    if core.get("trial_id") not in AUTHORIZED_DEVELOPMENT_TRIALS:
        raise OracleContractError("ledger trial is not authorized development data")
    if payload.get("scientific_core_sha256") != _canonical_sha256(core):
        raise OracleContractError("ledger scientific_core_sha256 mismatch")
    events = core.get("events")
    cycles = core.get("cycles")
    if not isinstance(events, list) or not isinstance(cycles, list):
        raise OracleContractError("ledger events/cycles must be arrays")
    event_ids: set[str] = set()
    previous_time = -math.inf
    for event in events:
        if not isinstance(event, Mapping):
            raise OracleContractError("event entries must be objects")
        event_id = str(event.get("event_id", ""))
        if not event_id or event_id in event_ids:
            raise OracleContractError("event ids must be unique and non-empty")
        event_ids.add(event_id)
        event_time = _finite_float(event.get("event_time_s"), "event_time_s")
        if event_time < previous_time - TIME_GRID_ATOL_S:
            raise OracleContractError("events are not time ordered")
        previous_time = event_time
        if event.get("event") not in {"heel_strike", "toe_off"}:
            raise OracleContractError("unknown event type")
    for cycle in cycles:
        hs = _finite_float(cycle.get("heel_strike_time_s"), "cycle.hs")
        toe = _finite_float(cycle.get("toe_off_time_s"), "cycle.to")
        next_hs = _finite_float(cycle.get("next_heel_strike_time_s"), "cycle.next_hs")
        if not hs < toe < next_hs:
            raise OracleContractError("cycle is not strict HS-TO-HS")
        if next_hs - hs + TIME_GRID_ATOL_S < float(core["min_cycle_duration_s"]):
            raise OracleContractError("cycle is shorter than the frozen minimum")
        for key in (
            "heel_strike_event_id",
            "toe_off_event_id",
            "next_heel_strike_event_id",
        ):
            if cycle.get(key) not in event_ids:
                raise OracleContractError(f"cycle references unknown event via {key}")
    _canonical_sha256(payload)
    return dict(payload)


def _load_contract(path: Path) -> dict[str, Any]:
    value = gate.load_json_strict(path)
    if not isinstance(value, Mapping):
        raise OracleContractError("V17 contract must be a JSON object")
    contract = dict(value)
    if contract.get("event_contract_id") != EVENT_CONTRACT_ID:
        raise OracleContractError("V17 contract event id drifted")
    split = contract.get("data_split")
    if not isinstance(split, Mapping):
        raise OracleContractError("V17 contract data_split is missing")
    if tuple(split.get("development", ())) != AUTHORIZED_DEVELOPMENT_TRIALS:
        raise OracleContractError("V17 development split drifted")
    if set(split.get("forbidden_to_builder", ())) != FORBIDDEN_TRIALS:
        raise OracleContractError("V17 forbidden split drifted")
    trials = contract.get("development_sources")
    if not isinstance(trials, Mapping) or set(trials) != set(AUTHORIZED_DEVELOPMENT_TRIALS):
        raise OracleContractError("contract must expose exactly trials 02/04/08")
    return contract


def build_development_bundle(
    *,
    contract_path: Path = DEFAULT_CONTRACT,
    output_dir: Path = DEFAULT_OUTPUT_DIR,
) -> Path:
    """Build an immutable three-trial development oracle bundle."""

    contract_path = contract_path.resolve()
    contract = _load_contract(contract_path)
    output_dir = output_dir.resolve()
    try:
        output_dir.relative_to(REPO_ROOT / "validation")
    except ValueError as exc:
        raise OracleContractError("oracle output must stay under validation/") from exc
    if os.path.lexists(output_dir):
        raise OracleContractError(f"refusing occupied output directory: {output_dir}")
    output_relative = output_dir.relative_to(REPO_ROOT).as_posix()

    built: list[tuple[str, dict[str, Any]]] = []
    oracle_contract = contract["oracle"]
    for trial_id in AUTHORIZED_DEVELOPMENT_TRIALS:
        trial = contract["development_sources"][trial_id]
        external_record = trial.get("external_loads")
        grf_record = trial.get("prescribed_grf")
        if not isinstance(external_record, Mapping) or not isinstance(grf_record, Mapping):
            raise OracleContractError(f"trial {trial_id} source records are incomplete")
        external_path = _verify_source_record(
            external_record, label=f"trial_{trial_id}.external_loads"
        )
        expected_grf_path = _verify_source_record(
            grf_record, label=f"trial_{trial_id}.prescribed_grf"
        )
        resolved_grf_path, vertical_column = resolve_prescribed_left_vertical_source(
            external_path
        )
        if resolved_grf_path != expected_grf_path:
            raise OracleContractError(
                f"trial {trial_id} XML resolves {resolved_grf_path}, not locked GRF"
            )
        times, columns, data = _read_storage_table(expected_grf_path)
        if vertical_column not in columns:
            raise OracleContractError(
                f"trial {trial_id} is missing left vertical column {vertical_column!r}"
            )
        force = data[:, columns.index(vertical_column)]
        source = {
            "external_loads": _source_record(external_path),
            "prescribed_grf": _source_record(expected_grf_path),
            "vertical_force_column": vertical_column,
            "vertical_axis": "+Y",
            "side": "left",
        }
        ledger = build_canonical_ledger(
            times,
            force,
            trial_id=trial_id,
            source=source,
            threshold_n=oracle_contract["contact_threshold_n"],
            min_contact_duration_s=oracle_contract["hs_min_persistence_s"],
            min_cycle_duration_s=oracle_contract["min_cycle_duration_s"],
            sample_dt_s=oracle_contract["sample_dt_s"],
        )
        ledger = add_window_views(ledger, trial["plateaus"])
        validate_ledger(ledger)
        built.append((trial_id, ledger))

    staging = output_dir.parent / (
        f".{output_dir.name}.tmp-{os.getpid()}-{uuid.uuid4().hex}"
    )
    if os.path.lexists(staging):
        raise OracleContractError(f"unexpected occupied staging path: {staging}")
    staging.mkdir(parents=True)
    try:
        ledger_records: list[dict[str, Any]] = []
        for trial_id, ledger in built:
            filename = f"trial_{trial_id}_canonical_event_ledger.json"
            path = gate.write_json_no_clobber(staging / filename, ledger)
            ledger_records.append(
                {
                    "trial_id": trial_id,
                    "role": "DEVELOPMENT",
                    "path": f"{output_relative}/{filename}",
                    "sha256": gate.sha256_file(path),
                    "scientific_core_sha256": ledger["scientific_core_sha256"],
                    "cycle_count": len(ledger["scientific_core"]["cycles"]),
                    "plateau_view_count": len(ledger["scientific_core"]["views"]),
                }
            )
        manifest = {
            "schema_version": 1,
            "manifest_id": "AB06_V17_CANONICAL_EVENT_ORACLE_DEVELOPMENT_2026-08-03",
            "status": "CANONICAL_ORACLE_FROZEN_DEVELOPMENT_ONLY",
            "event_contract_id": EVENT_CONTRACT_ID,
            "contract": _source_record(contract_path),
            "builder": _source_record(Path(__file__)),
            "data_access": {
                "opened_trials": list(AUTHORIZED_DEVELOPMENT_TRIALS),
                "protected_trials_opened": [],
                "reserve_trials_opened": [],
                "trial_01_used": False,
            },
            "consumer_rule": (
                "sequential 1 ms and batched 10 ms consumers reuse these "
                "absolute event timestamps without GRF re-thresholding"
            ),
            "ledgers": ledger_records,
        }
        gate.write_json_no_clobber(staging / "manifest.json", manifest)
        try:
            staging.rename(output_dir)
        except FileExistsError as exc:
            raise OracleContractError(
                f"refusing occupied output directory: {output_dir}"
            ) from exc
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return output_dir


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument(
        "--build-development",
        action="store_true",
        help="build only the locked 02/04/08 development bundle",
    )
    action.add_argument(
        "--validate-ledger",
        type=Path,
        help="validate an existing ledger without reading source GRF",
    )
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    if args.validate_ledger is not None:
        payload = gate.load_json_strict(args.validate_ledger)
        if not isinstance(payload, Mapping):
            raise OracleContractError("ledger must be a JSON object")
        validate_ledger(payload)
        print(f"PASS {args.validate_ledger}")
        return 0
    output = build_development_bundle(
        contract_path=args.contract,
        output_dir=args.output_dir,
    )
    print(f"PASS {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
