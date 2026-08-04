"""Development-only V15 diagnosis of two-sensor HS routing feasibility.

V15 does not change the production FSM, detector profile, runtime configuration,
or any V14.2 artifact.  It reuses only the frozen V14.2 preprocessing products
for DEV02/04/08 and the V14.2 prescribed reference-cycle records.  The V13
heel/toe geometry is sampled once per trial and cadence, then pure offline
diagnostics compare two causal routing semantics:

* ``heel_only`` -- the current production interpretation;
* ``first_stable_regional`` -- the first debounced heel or forefoot contact.

The threshold/dwell map is diagnostic and cannot promote a configuration.  A
separate one-factor estimate computes the optimistic ground-normal plantar
offset (reported as equivalent ``Hy``) required for the frozen V13 heel signal
to remain over 0.5 N long enough to confirm an HS within +50 ms at 30 ms dwell.
No alternative geometry is sampled by that estimate.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sys
import traceback
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Iterable, Mapping, Sequence

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
for _path in (REPO_ROOT, VALIDATION_ROOT):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import sweep_two_sensor_cross_speed_v14_2 as v142  # noqa: E402


v14 = v142.v14
v1 = v14.v1

SCHEMA_VERSION = 15
PROTOCOL_ID = "AB06_TWO_SENSOR_ROUTING_DIAGNOSTIC_2026-07-22_V15"
BASELINE_ID = "V13_BASELINE"
ALLOWED_TRIALS = ("02", "04", "08")
FORBIDDEN_TRIALS = ("01", "03", "05", "06", "07")
CADENCES = (("runtime_10ms", 0.010), ("fine_1ms", 0.001))
ROUTING_MODES = ("heel_only", "first_stable_regional")
SENSOR_ON_GRID_N = (0.3, 0.4, 0.5, 0.75, 1.0)
DWELL_GRID_S = (0.02, 0.03, 0.04)
EXPECTED_UNIT_COUNT = len(ALLOWED_TRIALS) * 4 * len(CADENCES)
EXPECTED_UNITS_PER_CADENCE = EXPECTED_UNIT_COUNT // len(CADENCES)
EXPECTED_EVENT_AND_HY_ROWS = 736
EXPECTED_INCUMBENT_HS_EVENTS = 736
EXPECTED_FEASIBILITY_ROWS = (
    EXPECTED_UNIT_COUNT
    * len(SENSOR_ON_GRID_N)
    * len(DWELL_GRID_S)
    * len(ROUTING_MODES)
)
EXPECTED_SOURCE_KEYS = {
    "v15_runner",
    "v15_tests",
    "v14_2_runner",
    "v14_runner",
    "v13_profile",
}
EXPECTED_PARENT_SOURCE_KEYS = {
    "protocol",
    "manifest",
    "development_candidate_lock",
    "development_stage1_details",
}
EXPECTED_PREPROCESSING_KEYS = {
    "preprocessing_lock",
    "ik_motion",
    "external_loads",
    "grf",
}
EXPECTED_TRIAL_INTERVALS = {
    "02": [9.875, 153.08],
    "04": [12.485, 156.025],
    "08": [10.69, 154.89],
}
DEFAULT_PROTOCOL = VALIDATION_ROOT / "two_sensor_v15_routing_protocol.json"
DEFAULT_OUTPUT_DIR = (
    VALIDATION_ROOT
    / "two_sensor_v15_routing_runs/2026-07-22_ab06_dev02_04_08_v15"
)
PARENT_RUN_DIR = (
    VALIDATION_ROOT
    / "two_sensor_cross_speed_v14_2_runs/"
    "2026-07-22_ab06_cross_speed_v14_2_boundary_recovery"
)
NUMERIC_TOLERANCE = 1.0e-12


class ProtocolError(RuntimeError):
    """Raised when the frozen V15 diagnostic contract is violated."""


class NoClobberError(ProtocolError):
    """Raised when a one-shot V15 destination is already occupied."""


@dataclass(frozen=True)
class SensorEdge:
    sensor: str
    edge: str
    onset_s: float
    confirmed_s: float


@dataclass(frozen=True)
class RoutedHS:
    routing_mode: str
    source_sensor: str
    onset_s: float
    confirmed_s: float


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _canonical_sha256(value: Any) -> str:
    raw = json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


def _portable(path: Path) -> str:
    try:
        return path.resolve().relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return path.resolve().as_posix()


def _resolve(path: str | Path) -> Path:
    value = Path(path)
    return (REPO_ROOT / value).resolve() if not value.is_absolute() else value.resolve()


def _load_object(path: Path, *, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"cannot load {label}: {_portable(path)}") from exc
    if not isinstance(value, dict):
        raise ProtocolError(f"{label} must be a JSON object: {_portable(path)}")
    return value


def _validate_source_record(
    record: Mapping[str, Any],
    *,
    label: str,
    required_parent: Path | None = None,
) -> Path:
    if set(record) != {"path", "sha256"}:
        raise ProtocolError(f"invalid source record for {label}")
    path = _resolve(str(record["path"]))
    if required_parent is not None:
        try:
            path.relative_to(required_parent.resolve())
        except ValueError as exc:
            raise ProtocolError(f"{label} escapes its frozen parent directory") from exc
    if not path.is_file():
        raise ProtocolError(f"missing V15 source {label}: {_portable(path)}")
    observed = _sha256(path)
    if observed != str(record["sha256"]):
        raise ProtocolError(
            f"V15 source hash drift for {label}: {observed} != {record['sha256']}"
        )
    return path


def _require_exact_sequence(
    observed: Iterable[Any], expected: Sequence[Any], *, label: str
) -> None:
    if list(observed) != list(expected):
        raise ProtocolError(f"{label} drifted: {list(observed)} != {list(expected)}")


def load_and_validate_protocol(
    path: str | Path = DEFAULT_PROTOCOL,
) -> dict[str, Any]:
    protocol_path = _resolve(path)
    if protocol_path != DEFAULT_PROTOCOL.resolve():
        raise ProtocolError("V15 accepts only its canonical frozen protocol path")
    raw = _load_object(protocol_path, label="V15 protocol")
    checks = {
        "schema_version": raw.get("schema_version") == SCHEMA_VERSION,
        "protocol_id": raw.get("protocol_id") == PROTOCOL_ID,
        "frozen": raw.get("frozen_before_execution") is True,
        "stage": raw.get("stage") == "development_only_diagnostic",
        "baseline": raw.get("geometry", {}).get("candidate_id") == BASELINE_ID,
        "no_geometry_sampling": raw.get("geometry", {}).get(
            "alternative_geometry_sampling_allowed"
        ) is False,
        "no_runtime_change": raw.get("non_actions", {}).get(
            "runtime_or_fsm_modification_allowed"
        ) is False,
        "no_promotion": raw.get("decision_contract", {}).get(
            "candidate_selection_or_promotion_allowed"
        ) is False,
    }
    if not all(checks.values()):
        raise ProtocolError(f"V15 top-level contract drifted: {checks}")

    split = raw.get("split", {})
    _require_exact_sequence(
        split.get("DEVELOPMENT", ()), ALLOWED_TRIALS, label="development split"
    )
    _require_exact_sequence(
        split.get("FORBIDDEN", ()), FORBIDDEN_TRIALS, label="forbidden split"
    )
    if split.get("VALIDATION") != ["05"] or split.get("SEALED") != ["06"]:
        raise ProtocolError("V15 holdout identities drifted")
    if split.get("RESERVE") != ["03", "07"]:
        raise ProtocolError("V15 reserve identities drifted")

    sampling = raw.get("sampling", {})
    _require_exact_sequence(
        sampling.get("cadences", {}).keys(),
        [label for label, _dt in CADENCES],
        label="cadence labels",
    )
    for label, dt in CADENCES:
        if not math.isclose(
            float(sampling["cadences"][label]), dt, abs_tol=NUMERIC_TOLERANCE
        ):
            raise ProtocolError(f"V15 cadence drifted: {label}")
    _require_exact_sequence(
        raw.get("routing", {}).get("modes", ()),
        ROUTING_MODES,
        label="routing modes",
    )

    feasibility = raw.get("feasibility_map", {})
    thresholds = [float(value) for value in feasibility.get("sensor_on_n", ())]
    dwells = [float(value) for value in feasibility.get("dwell_s", ())]
    off_n = float(feasibility.get("sensor_off_n", math.nan))
    if (
        thresholds != list(SENSOR_ON_GRID_N)
        or dwells != list(DWELL_GRID_S)
        or not math.isclose(off_n, 0.25, abs_tol=NUMERIC_TOLERANCE)
        or float(feasibility.get("hs_tolerance_s", math.nan)) != 0.05
    ):
        raise ProtocolError("invalid V15 threshold/dwell feasibility grid")

    hy = raw.get("hy_clearance_estimate", {})
    hy_checks = {
        "threshold": float(hy.get("sensor_on_n", math.nan)) == 0.5,
        "dwell": float(hy.get("dwell_s", math.nan)) == 0.03,
        "deadline": float(hy.get("maximum_confirmed_error_s", math.nan)) == 0.05,
        "lookback": float(hy.get("search_lookback_s", math.nan)) == 0.25,
        "frozen_signal": hy.get("source_geometry") == BASELINE_ID,
        "no_resample": hy.get("new_geometry_station_sampling_allowed") is False,
        "normal_only": hy.get("reported_quantity")
        == "equivalent_plantar_ground_normal_offset_mm",
        "checkpoints": hy.get("reporting_checkpoints_mm")
        == [0.0, 0.5, 0.75, 1.0, 2.0],
    }
    if not all(hy_checks.values()):
        raise ProtocolError(f"V15 Hy estimate contract drifted: {hy_checks}")
    if float(raw.get("event_association", {}).get("maximum_distance_s", math.nan)) != 0.25:
        raise ProtocolError("V15 event-association window drifted")

    sources = raw.get("sources")
    if not isinstance(sources, dict) or set(sources) != EXPECTED_SOURCE_KEYS:
        raise ProtocolError("V15 hash-pinned source keyset drifted")
    for label, record in sources.items():
        if not isinstance(record, Mapping):
            raise ProtocolError(f"invalid V15 source record: {label}")
        _validate_source_record(record, label=f"sources.{label}")

    parent = raw.get("parent_v14_2", {})
    parent_sources = parent.get("sources")
    if (
        not isinstance(parent_sources, dict)
        or set(parent_sources) != EXPECTED_PARENT_SOURCE_KEYS
    ):
        raise ProtocolError("V15 parent V14.2 source keyset drifted")
    for label, record in parent_sources.items():
        if not isinstance(record, Mapping):
            raise ProtocolError(f"invalid parent source record: {label}")
        _validate_source_record(record, label=f"parent_v14_2.{label}")

    preprocessing = raw.get("preprocessing", {}).get("trials")
    if not isinstance(preprocessing, dict) or tuple(preprocessing) != ALLOWED_TRIALS:
        raise ProtocolError("V15 preprocessing scope must be exactly DEV02/04/08")
    parent_preprocessed = (PARENT_RUN_DIR / "preprocessed").resolve()
    for trial_id, records in preprocessing.items():
        if (
            trial_id not in ALLOWED_TRIALS
            or not isinstance(records, Mapping)
            or set(records) != EXPECTED_PREPROCESSING_KEYS
        ):
            raise ProtocolError("invalid V15 preprocessing trial")
        expected_parent = (parent_preprocessed / f"trial_{trial_id}").resolve()
        for label, record in records.items():
            if not isinstance(record, Mapping):
                raise ProtocolError(f"invalid preprocessing source: {trial_id}.{label}")
            _validate_source_record(
                record,
                label=f"preprocessing.{trial_id}.{label}",
                required_parent=expected_parent,
            )

    if tuple(raw.get("trials", {})) != ALLOWED_TRIALS:
        raise ProtocolError("V15 trial interval keyset drifted")
    observed_intervals = {
        trial_id: raw["trials"][trial_id].get("trial_interval_s")
        for trial_id in ALLOWED_TRIALS
    }
    if observed_intervals != EXPECTED_TRIAL_INTERVALS:
        raise ProtocolError("V15 trial intervals drifted")
    if raw.get("geometry", {}).get("profile") != sources.get("v13_profile"):
        raise ProtocolError("V15 geometry profile/source binding drifted")

    raw["_protocol_path"] = protocol_path.as_posix()
    raw["_protocol_sha256"] = _sha256(protocol_path)
    return raw


def validate_parent_provenance(protocol: Mapping[str, Any]) -> dict[str, Any]:
    parent = protocol["parent_v14_2"]
    manifest_path = _resolve(parent["sources"]["manifest"]["path"])
    lock_path = _resolve(parent["sources"]["development_candidate_lock"]["path"])
    manifest = _load_object(manifest_path, label="V14.2 manifest")
    lock = _load_object(lock_path, label="V14.2 development lock")
    checks = {
        "parent_status_fail": manifest.get("status") == "FAIL",
        "parent_conclusion": manifest.get("conclusion")
        == "V13_RETAINED_NO_DEVELOPMENT_FINALIST",
        "no_parent_finalist": lock.get("finalist_id") is None,
        "validation_forbidden": lock.get("validation_semantic_access_allowed")
        is False,
        "validation_unopened": manifest.get("validation", {}).get("opened") is False,
        "sealed_unopened": manifest.get("sealed", {}).get("opened") is False,
        "reserve_unopened": manifest.get("reserve", {}).get("opened") is False,
        "development_exact": sorted(
            lock.get("development_access_receipts", {}).keys()
        )
        == list(ALLOWED_TRIALS),
    }
    if not all(checks.values()):
        raise ProtocolError(f"V14.2 parent state drifted: {checks}")
    return {
        "status": "PASS_V14_2_DEVELOPMENT_ONLY_PARENT_VERIFIED",
        "checks": checks,
        "manifest": {
            "path": _portable(manifest_path),
            "sha256": _sha256(manifest_path),
        },
        "development_candidate_lock": {
            "path": _portable(lock_path),
            "sha256": _sha256(lock_path),
        },
    }


def _validate_preprocessing_trial(
    protocol: Mapping[str, Any], trial_id: str
) -> SimpleNamespace:
    if trial_id not in ALLOWED_TRIALS:
        raise ProtocolError(f"V15 preprocessing access forbidden for trial {trial_id}")
    records = protocol["preprocessing"]["trials"][trial_id]
    paths = {
        label: _resolve(record["path"]) for label, record in records.items()
    }
    lock = _load_object(paths["preprocessing_lock"], label=f"trial {trial_id} lock")
    parent_trial = protocol["trials"][trial_id]
    expected_interval = [float(value) for value in parent_trial["trial_interval_s"]]
    checks = {
        "trial": lock.get("trial_id") == trial_id,
        "stage": lock.get("stage") == "development",
        "status": lock.get("status")
        == "V14_1_PREPROCESSING_FROZEN_BEFORE_DETECTOR_REPLAY",
        "interval": lock.get("analysis_interval_s") == expected_interval,
        "marker_ik": lock.get("dataset_ik_used_downstream") is False,
        "coverage": lock.get("all_sources_cover_analysis_interval") is True,
        "absolute_time": lock.get("absolute_timestamps_no_rezero") is True,
        "no_adaptation": lock.get(
            "adaptive_crop_resample_or_interpolation_used"
        )
        is False,
    }
    preprocessed_files = lock.get("preprocessed_files", {})
    checks["ik_bound"] = preprocessed_files.get("ik_motion", {}).get("sha256") == _sha256(
        paths["ik_motion"]
    )
    checks["external_bound"] = preprocessed_files.get("external_loads", {}).get(
        "sha256"
    ) == _sha256(paths["external_loads"])
    checks["grf_bound"] = preprocessed_files.get("grf", {}).get("sha256") == _sha256(
        paths["grf"]
    )
    if not all(checks.values()):
        raise ProtocolError(f"trial {trial_id} preprocessing lock drifted: {checks}")

    live = lock.get("live_replay_inputs", {})
    model = _validate_source_record(live["model"], label=f"trial {trial_id} model")
    reserve = _validate_source_record(
        live["reserve_actuators"], label=f"trial {trial_id} reserve"
    )
    plugin = live.get("plugin", {})
    plugin_binary = _resolve(str(plugin.get("binary_path", "")))
    if (
        not plugin_binary.is_file()
        or _sha256(plugin_binary) != plugin.get("binary_sha256")
    ):
        raise ProtocolError(f"trial {trial_id} plugin identity drifted")
    setup = v14.setup_io.build_simulation_setup(
        model_file=model,
        kinematics_file=paths["ik_motion"],
        external_loads_xml=paths["external_loads"],
        reserve_actuators_xml=reserve,
        t_start=expected_interval[0],
        t_end=expected_interval[1],
        grf_mode="prescribed",
    )
    return SimpleNamespace(
        trial_id=trial_id,
        setup=setup,
        lock=lock,
        lock_path=paths["preprocessing_lock"],
        plugin_loader=str(plugin["loader_basename"]),
    )


def _jsonl_objects(path: Path) -> Iterable[dict[str, Any]]:
    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            try:
                value = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ProtocolError(
                    f"invalid JSONL at {_portable(path)}:{line_number}"
                ) from exc
            if not isinstance(value, dict):
                raise ProtocolError("V14.2 detail row is not an object")
            yield value


def load_frozen_references(
    protocol: Mapping[str, Any], trial_id: str
) -> tuple[dict[str, Any], ...]:
    if trial_id not in ALLOWED_TRIALS:
        raise ProtocolError(f"V15 reference access forbidden for trial {trial_id}")
    details_path = _resolve(
        protocol["parent_v14_2"]["sources"]["development_stage1_details"]["path"]
    )
    by_plateau: dict[int, list[dict[str, Any]]] = {}
    for item in _jsonl_objects(details_path):
        row = item.get("row", {})
        if row.get("candidate_id") != BASELINE_ID or row.get("trial_id") != trial_id:
            continue
        plateau = int(row["plateau_index"])
        by_plateau.setdefault(plateau, []).append(item)
    if sorted(by_plateau) != [1, 2, 3, 4]:
        raise ProtocolError(f"trial {trial_id} lacks four frozen V13 references")

    result: list[dict[str, Any]] = []
    for plateau_index in range(1, 5):
        entries = by_plateau[plateau_index]
        if len(entries) != 2 or {x["row"]["cadence"] for x in entries} != {
            label for label, _dt in CADENCES
        }:
            raise ProtocolError("V14.2 reference cadence cardinality drifted")
        references = [entry["reference"] for entry in entries]
        digests = {reference["reference_event_sha256"] for reference in references}
        if len(digests) != 1:
            raise ProtocolError("10 ms and 1 ms reference sets differ")
        canonical_events = references[0]["events"]
        if any(reference["events"] != canonical_events for reference in references[1:]):
            raise ProtocolError("V14.2 reference arrays differ across cadence")
        expected_digest = _canonical_sha256(canonical_events)
        if expected_digest not in digests:
            raise ProtocolError("V14.2 reference digest does not bind its events")
        result.append(
            {
                "trial_id": trial_id,
                "plateau_index": plateau_index,
                "speed_mps": float(references[0]["speed_mps"]),
                "plateau_interval_s": [
                    float(value) for value in references[0]["plateau_interval_s"]
                ],
                "events": {
                    "heel_strike": np.asarray(
                        canonical_events["heel_strike"], dtype=float
                    ),
                    "toe_off": np.asarray(canonical_events["toe_off"], dtype=float),
                },
                "reference_event_sha256": expected_digest,
            }
        )
    return tuple(result)


def _expected_unit_keys() -> set[tuple[str, int, str]]:
    return {
        (trial_id, plateau_index, cadence)
        for trial_id in ALLOWED_TRIALS
        for plateau_index in range(1, 5)
        for cadence, _sample_dt_s in CADENCES
    }


def _strict_grid_indices(
    values: Sequence[float],
    *,
    trial_start_s: float,
    sample_dt_s: float,
    label: str,
) -> list[int]:
    array = np.asarray(values, dtype=float)
    if (
        array.ndim != 1
        or not np.all(np.isfinite(array))
        or np.any(np.diff(array) <= 0.0)
        or sample_dt_s <= 0.0
    ):
        raise ProtocolError(f"invalid ordered HS sequence for {label}")
    raw_indices = (array - float(trial_start_s)) / float(sample_dt_s)
    indices = np.rint(raw_indices).astype(np.int64)
    reconstructed = float(trial_start_s) + indices * float(sample_dt_s)
    if np.any(np.abs(reconstructed - array) > 1.0e-9):
        raise ProtocolError(f"off-grid HS timestamp in {label}")
    return [int(value) for value in indices]


def _sequence_payload(
    *,
    key: tuple[str, int, str],
    reference_event_sha256: str,
    onset_indices: Sequence[int],
    confirmed_indices: Sequence[int],
    source_sensors: Sequence[str],
) -> dict[str, Any]:
    return {
        "unit": [key[0], key[1], key[2]],
        "reference_event_sha256": str(reference_event_sha256),
        "onset_hs_sample_indices": [int(value) for value in onset_indices],
        "confirmed_hs_sample_indices": [
            int(value) for value in confirmed_indices
        ],
        "source_sensors": [str(value) for value in source_sensors],
    }


def load_v14_2_incumbent_sequences(
    protocol: Mapping[str, Any],
) -> dict[str, Any]:
    """Load and cross-check every frozen V14.2 incumbent HS sequence."""

    details_path = _resolve(
        protocol["parent_v14_2"]["sources"]["development_stage1_details"]["path"]
    )
    expected_keys = _expected_unit_keys()
    by_unit: dict[tuple[str, int, str], dict[str, Any]] = {}
    for item in _jsonl_objects(details_path):
        row = item.get("row", {})
        if row.get("candidate_id") != BASELINE_ID:
            continue
        key = (
            str(row["trial_id"]),
            int(row["plateau_index"]),
            str(row["cadence"]),
        )
        if key not in expected_keys:
            raise ProtocolError(f"V14.2 incumbent escaped DEV unit set: {key}")
        if key in by_unit:
            raise ProtocolError(f"duplicate V14.2 incumbent unit: {key}")
        sample_dt_s = dict(CADENCES)[key[2]]
        if not math.isclose(
            float(row["sample_dt_s"]),
            sample_dt_s,
            rel_tol=0.0,
            abs_tol=NUMERIC_TOLERANCE,
        ):
            raise ProtocolError(f"V14.2 cadence/sample interval mismatch: {key}")

        reference = item["reference"]
        reference_events = reference["events"]
        reference_digest = str(reference["reference_event_sha256"])
        if _canonical_sha256(reference_events) != reference_digest:
            raise ProtocolError(f"V14.2 reference digest mismatch: {key}")
        reference_hs = np.asarray(reference_events["heel_strike"], dtype=float)
        confirmed_errors = np.asarray(
            item["confirmed_timing"]["heel_strike"]["ordered_errors_s"],
            dtype=float,
        )
        onset_errors = np.asarray(
            item["onset_timing"]["heel_strike"]["ordered_errors_s"],
            dtype=float,
        )
        if (
            reference_hs.ndim != 1
            or reference_hs.size < 1
            or confirmed_errors.shape != reference_hs.shape
            or onset_errors.shape != reference_hs.shape
            or int(row["reference_hs_count"]) != reference_hs.size
            or int(row["predicted_hs_count"]) != reference_hs.size
        ):
            raise ProtocolError(f"V14.2 ordered HS cardinality mismatch: {key}")
        confirmed_s = (reference_hs + confirmed_errors).tolist()
        onset_s = (reference_hs + onset_errors).tolist()
        phase_confirmed_s = [
            float(record["confirmed_time_s"])
            for record in item["phase"]["confirmed_state_hold_gate"]["records"]
            if record["event"] == "heel_strike"
        ]
        metrics = item["event_metrics"]["heel_strike"]
        metric_confirmed_s = sorted(
            [float(record["predicted_time_s"]) for record in metrics["pairs"]]
            + [float(value) for value in metrics["unmatched_predicted"]]
        )
        trial_start_s = float(protocol["trials"][key[0]]["trial_interval_s"][0])
        onset_indices = _strict_grid_indices(
            onset_s,
            trial_start_s=trial_start_s,
            sample_dt_s=sample_dt_s,
            label=f"V14.2 onset {key}",
        )
        confirmed_indices = _strict_grid_indices(
            confirmed_s,
            trial_start_s=trial_start_s,
            sample_dt_s=sample_dt_s,
            label=f"V14.2 confirmed {key}",
        )
        if _strict_grid_indices(
            phase_confirmed_s,
            trial_start_s=trial_start_s,
            sample_dt_s=sample_dt_s,
            label=f"V14.2 phase confirmed {key}",
        ) != confirmed_indices:
            raise ProtocolError(f"V14.2 phase/ordered HS mismatch: {key}")
        if _strict_grid_indices(
            metric_confirmed_s,
            trial_start_s=trial_start_s,
            sample_dt_s=sample_dt_s,
            label=f"V14.2 event-metric confirmed {key}",
        ) != confirmed_indices:
            raise ProtocolError(f"V14.2 metric/ordered HS mismatch: {key}")
        payload = _sequence_payload(
            key=key,
            reference_event_sha256=reference_digest,
            onset_indices=onset_indices,
            confirmed_indices=confirmed_indices,
            source_sensors=["heel"] * len(confirmed_indices),
        )
        by_unit[key] = {
            "payload": payload,
            "sha256": _canonical_sha256(payload),
            "event_count": len(confirmed_indices),
        }
    if set(by_unit) != expected_keys:
        raise ProtocolError(
            "V14.2 incumbent unit set mismatch: "
            f"{sorted(by_unit)} != {sorted(expected_keys)}"
        )
    ordered_payloads = [by_unit[key]["payload"] for key in sorted(by_unit)]
    event_count = sum(int(record["event_count"]) for record in by_unit.values())
    if event_count != EXPECTED_INCUMBENT_HS_EVENTS:
        raise ProtocolError(
            f"V14.2 incumbent HS count drifted: {event_count} != "
            f"{EXPECTED_INCUMBENT_HS_EVENTS}"
        )
    return {
        "by_unit": by_unit,
        "unit_count": len(by_unit),
        "event_count": event_count,
        "aggregate_sha256": _canonical_sha256(ordered_payloads),
    }


def _json_list_field(row: Mapping[str, Any], field: str) -> list[Any]:
    value = row[field]
    if isinstance(value, str):
        value = json.loads(value)
    if not isinstance(value, list):
        raise ProtocolError(f"V15 sequence field is not a list: {field}")
    return value


def incumbent_v14_2_parity(
    protocol: Mapping[str, Any],
    feasibility_rows: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    """Require exact full-sequence parity with frozen V14.2 incumbent HS."""

    selected_rows = [
        row
        for row in feasibility_rows
        if row["routing_mode"] == "heel_only"
        and math.isclose(
            float(row["sensor_on_threshold_n"]),
            0.5,
            rel_tol=0.0,
            abs_tol=NUMERIC_TOLERANCE,
        )
        and math.isclose(
            float(row["sensor_dwell_s"]),
            0.03,
            rel_tol=0.0,
            abs_tol=NUMERIC_TOLERANCE,
        )
    ]
    observed = {
        (str(row["trial_id"]), int(row["plateau_index"]), str(row["cadence"])): row
        for row in selected_rows
    }
    parent = load_v14_2_incumbent_sequences(protocol)
    expected_keys = _expected_unit_keys()
    records: list[dict[str, Any]] = []
    observed_payloads: list[dict[str, Any]] = []
    for key in sorted(expected_keys):
        current = observed.get(key)
        expected = parent["by_unit"][key]
        checks = {"unit_present": current is not None}
        observed_sha256: str | None = None
        if current is not None:
            trial_start_s = float(
                protocol["trials"][key[0]]["trial_interval_s"][0]
            )
            sample_dt_s = dict(CADENCES)[key[2]]
            onset_s = [
                float(value)
                for value in _json_list_field(
                    current, "ordered_routed_onset_s_json"
                )
            ]
            confirmed_s = [
                float(value)
                for value in _json_list_field(
                    current, "ordered_routed_confirmed_s_json"
                )
            ]
            sources = [
                str(value)
                for value in _json_list_field(
                    current, "ordered_routed_source_sensor_json"
                )
            ]
            onset_indices = _strict_grid_indices(
                onset_s,
                trial_start_s=trial_start_s,
                sample_dt_s=sample_dt_s,
                label=f"V15 onset {key}",
            )
            confirmed_indices = _strict_grid_indices(
                confirmed_s,
                trial_start_s=trial_start_s,
                sample_dt_s=sample_dt_s,
                label=f"V15 confirmed {key}",
            )
            payload = _sequence_payload(
                key=key,
                reference_event_sha256=str(current["reference_event_sha256"]),
                onset_indices=onset_indices,
                confirmed_indices=confirmed_indices,
                source_sensors=sources,
            )
            observed_payloads.append(payload)
            observed_sha256 = _canonical_sha256(payload)
            checks.update(
                {
                    "row_sequence_digest": observed_sha256
                    == str(current["routed_hs_sequence_sha256"]),
                    "exact_onset_confirmed_source_sequence": payload
                    == expected["payload"],
                    "exact_sequence_digest": observed_sha256
                    == expected["sha256"],
                }
            )
        records.append(
            {
                "trial_id": key[0],
                "plateau_index": key[1],
                "cadence": key[2],
                "expected_sequence_sha256": expected["sha256"],
                "observed_sequence_sha256": observed_sha256,
                "checks": checks,
                "ok": all(checks.values()),
            }
        )
    observed_keyset_ok = bool(
        len(selected_rows) == len(observed) == EXPECTED_UNIT_COUNT
        and set(observed) == expected_keys
    )
    observed_aggregate = (
        _canonical_sha256(observed_payloads)
        if len(observed_payloads) == EXPECTED_UNIT_COUNT
        else None
    )
    ok = bool(
        observed_keyset_ok
        and len(records) == EXPECTED_UNIT_COUNT
        and all(item["ok"] for item in records)
        and observed_aggregate == parent["aggregate_sha256"]
    )
    return {
        "status": (
            "PASS_EXACT_V14_2_INCUMBENT_HS_SEQUENCE_PARITY"
            if ok
            else "FAIL_INCUMBENT_SIGNAL_ROUTER_PARITY"
        ),
        "ok": ok,
        "unit_count": len(records),
        "event_count": parent["event_count"],
        "observed_keyset_exact": observed_keyset_ok,
        "expected_aggregate_sequence_sha256": parent["aggregate_sha256"],
        "observed_aggregate_sequence_sha256": observed_aggregate,
        "records": records,
    }


def raw_rising_crossings(
    times: Sequence[float] | np.ndarray,
    force_n: Sequence[float] | np.ndarray,
    threshold_n: float,
) -> list[float]:
    sample_times = np.asarray(times, dtype=float)
    force = np.asarray(force_n, dtype=float)
    if sample_times.ndim != 1 or force.shape != sample_times.shape:
        raise ProtocolError("raw crossing inputs must be aligned 1-D arrays")
    active = force >= float(threshold_n)
    indices = np.flatnonzero(active & ~np.r_[False, active[:-1]])
    return [float(sample_times[index]) for index in indices]


def debounce_sensor_edges(
    times: Sequence[float] | np.ndarray,
    force_n: Sequence[float] | np.ndarray,
    *,
    sensor: str,
    on_threshold_n: float,
    off_threshold_n: float,
    dwell_s: float,
) -> list[SensorEdge]:
    """Pure reproduction of the production per-sensor hysteresis/debounce."""

    sample_times = np.asarray(times, dtype=float)
    force = np.asarray(force_n, dtype=float)
    if (
        sample_times.ndim != 1
        or force.shape != sample_times.shape
        or sample_times.size < 2
        or not np.all(np.isfinite(sample_times))
        or not np.all(np.isfinite(force))
        or np.any(np.diff(sample_times) <= 0.0)
    ):
        raise ProtocolError("invalid debounce trace")
    if not (0.0 <= off_threshold_n < on_threshold_n and dwell_s >= 0.0):
        raise ProtocolError("invalid debounce parameters")

    active = False
    pending_target: bool | None = None
    pending_since: float | None = None
    edges: list[SensorEdge] = []
    for time_s, value in zip(sample_times, force):
        requested: bool | None = None
        if active and value <= off_threshold_n:
            requested = False
        elif not active and value >= on_threshold_n:
            requested = True
        if requested is None:
            pending_target = None
            pending_since = None
            continue
        if pending_target != requested:
            pending_target = requested
            pending_since = float(time_s)
        if pending_since is None or float(time_s) - pending_since + 1e-12 < dwell_s:
            continue
        active = bool(requested)
        edges.append(
            SensorEdge(
                sensor=sensor,
                edge="contact_on" if active else "contact_off",
                onset_s=float(pending_since),
                confirmed_s=float(time_s),
            )
        )
        pending_target = None
        pending_since = None
    return edges


def route_hs_events(
    heel_edges: Sequence[SensorEdge],
    forefoot_edges: Sequence[SensorEdge],
    *,
    routing_mode: str,
) -> list[RoutedHS]:
    """Route debounced regional edges without invoking or changing the FSM."""

    if routing_mode not in ROUTING_MODES:
        raise ProtocolError(f"unsupported V15 routing mode: {routing_mode}")
    ordered = sorted(
        [*heel_edges, *forefoot_edges],
        key=lambda item: (
            item.confirmed_s,
            item.onset_s,
            0 if item.sensor == "heel" else 1,
            0 if item.edge == "contact_off" else 1,
        ),
    )
    contact = {"heel": False, "forefoot": False}
    armed = True
    result: list[RoutedHS] = []
    for edge in ordered:
        contact[edge.sensor] = edge.edge == "contact_on"
        if not contact["heel"] and not contact["forefoot"]:
            armed = True
            continue
        if not armed or edge.edge != "contact_on":
            continue
        eligible = routing_mode == "first_stable_regional" or edge.sensor == "heel"
        if not eligible:
            continue
        result.append(
            RoutedHS(
                routing_mode=routing_mode,
                source_sensor=edge.sensor,
                onset_s=edge.onset_s,
                confirmed_s=edge.confirmed_s,
            )
        )
        armed = False
    return result


def route_hs_trace(
    times: Sequence[float] | np.ndarray,
    heel_force_n: Sequence[float] | np.ndarray,
    forefoot_force_n: Sequence[float] | np.ndarray,
    heel_edges: Sequence[SensorEdge],
    forefoot_edges: Sequence[SensorEdge],
    *,
    routing_mode: str,
    on_threshold_n: float,
    off_threshold_n: float,
    dwell_s: float,
) -> list[RoutedHS]:
    """Route stable ON edges with the production sensor-guard contract.

    This remains a signal-level router, not a replacement for the production
    gait-state FSM.  It reproduces the guard's conservative startup handling,
    raw-clear arming, and unconditional rearm when the last debounced contact
    falls.  It cannot reproduce gait-state rejection or timeout behavior.
    """

    if routing_mode not in ROUTING_MODES:
        raise ProtocolError(f"unsupported V15 routing mode: {routing_mode}")
    sample_times = np.asarray(times, dtype=float)
    heel_force = np.asarray(heel_force_n, dtype=float)
    forefoot_force = np.asarray(forefoot_force_n, dtype=float)
    if (
        sample_times.ndim != 1
        or heel_force.shape != sample_times.shape
        or forefoot_force.shape != sample_times.shape
        or sample_times.size < 2
        or np.any(np.diff(sample_times) <= 0.0)
    ):
        raise ProtocolError("invalid V15 sample-wise router trace")

    edges_by_time: dict[float, list[SensorEdge]] = {}
    for edge in (*heel_edges, *forefoot_edges):
        edges_by_time.setdefault(round(float(edge.confirmed_s), 12), []).append(edge)

    if not (0.0 <= off_threshold_n < on_threshold_n and dwell_s >= 0.0):
        raise ProtocolError("invalid V15 router thresholds/dwell")

    contact = {"heel": False, "forefoot": False}
    raw_clear_since_s: float | None = None
    startup_heel_only_since_s: float | None = None
    startup_resolved = False
    bootstrap_reported = False
    armed = False
    routed: list[RoutedHS] = []
    for index, time_s in enumerate(sample_times):
        startup_unresolved = bool(
            not startup_resolved and not armed and not bootstrap_reported
        )
        raw_startup_heel_only = bool(
            heel_force[index] >= on_threshold_n
            and forefoot_force[index] <= off_threshold_n
        )
        if startup_unresolved and raw_startup_heel_only:
            if startup_heel_only_since_s is None:
                startup_heel_only_since_s = float(time_s)
        else:
            startup_heel_only_since_s = None

        step_edges = sorted(
            edges_by_time.get(round(float(time_s), 12), ()),
            key=lambda item: (
                item.onset_s,
                0 if item.sensor == "heel" else 1,
                0 if item.edge == "contact_off" else 1,
            ),
        )
        for edge in step_edges:
            contact[edge.sensor] = edge.edge == "contact_on"

        both_raw_clear = bool(
            heel_force[index] <= off_threshold_n
            and forefoot_force[index] <= off_threshold_n
        )
        if not both_raw_clear:
            raw_clear_since_s = None
        elif raw_clear_since_s is None:
            raw_clear_since_s = float(time_s)
        if (
            raw_clear_since_s is not None
            and float(time_s) - raw_clear_since_s + NUMERIC_TOLERANCE >= dwell_s
            and not contact["heel"]
            and not contact["forefoot"]
        ):
            armed = True
            startup_resolved = True

        on_edges = [edge for edge in step_edges if edge.edge == "contact_on"]
        off_edges = [edge for edge in step_edges if edge.edge == "contact_off"]
        heel_rose = next(
            (edge for edge in on_edges if edge.sensor == "heel"), None
        )
        startup_heel_only_ready = bool(
            startup_unresolved
            and raw_startup_heel_only
            and startup_heel_only_since_s is not None
            and float(time_s) - startup_heel_only_since_s + NUMERIC_TOLERANCE
            >= dwell_s
        )

        # Match the production startup contract: only a sustained heel-only
        # pattern is an HS. Toe-only or heel+forefoot is a partial-stance
        # bootstrap and must first clear before a new HS can be armed.
        if (
            startup_unresolved
            and not armed
            and (contact["heel"] or contact["forefoot"])
            and not bootstrap_reported
        ):
            if (
                startup_heel_only_ready
                and heel_rose is not None
                and contact["heel"]
                and not contact["forefoot"]
            ):
                routed.append(
                    RoutedHS(
                        routing_mode=routing_mode,
                        source_sensor="heel",
                        onset_s=float(startup_heel_only_since_s),
                        confirmed_s=float(time_s),
                    )
                )
            bootstrap_reported = True
            startup_resolved = True
            startup_heel_only_since_s = None
            armed = False

        if armed:
            if routing_mode == "heel_only":
                eligible = [edge for edge in on_edges if edge.sensor == "heel"]
            else:
                eligible = on_edges
            if eligible:
                selected = min(
                    eligible,
                    key=lambda item: (
                        item.onset_s,
                        0 if item.sensor == "heel" else 1,
                    ),
                )
                routed.append(
                    RoutedHS(
                        routing_mode=routing_mode,
                        source_sensor=selected.sensor,
                        onset_s=selected.onset_s,
                        confirmed_s=selected.confirmed_s,
                    )
                )
                armed = False
                startup_resolved = True

        # Production rearms as soon as the final stable OFF edge leaves both
        # latches inactive. The other raw signal may legitimately remain in
        # the hysteresis band; requiring another raw-clear dwell is incorrect.
        if (
            off_edges
            and not contact["heel"]
            and not contact["forefoot"]
        ):
            armed = True
            startup_resolved = True
            bootstrap_reported = True
            startup_heel_only_since_s = None
    return routed


def _nearest_unique(
    references: Sequence[float] | np.ndarray,
    candidates: Sequence[float],
    *,
    maximum_distance_s: float,
) -> dict[int, int]:
    """Maximum-cardinality, minimum-error chronological association."""

    ref = np.asarray(references, dtype=float)
    candidate = np.asarray(candidates, dtype=float)

    @lru_cache(maxsize=None)
    def solve(i: int, j: int) -> tuple[int, float, tuple[tuple[int, int], ...]]:
        if i >= ref.size or j >= candidate.size:
            return 0, 0.0, ()
        options = [solve(i + 1, j), solve(i, j + 1)]
        distance = abs(float(candidate[j] - ref[i]))
        if distance <= maximum_distance_s + NUMERIC_TOLERANCE:
            count, cost, pairs = solve(i + 1, j + 1)
            options.append((count + 1, cost + distance, ((i, j), *pairs)))
        return min(
            options,
            key=lambda item: (-item[0], item[1], item[2]),
        )

    _count, _cost, pairs = solve(0, 0)
    return dict(pairs)


def _local_routed_events(
    routed: Sequence[RoutedHS], reference: Mapping[str, Any]
) -> list[RoutedHS]:
    refs = np.asarray(reference["events"]["heel_strike"], dtype=float)
    plateau_start, plateau_end = reference["plateau_interval_s"]
    # Preserve signed out-of-tolerance boundary errors for the diagnostic.
    # Strict matching still uses 50 ms, while this 250 ms ownership context is
    # the same frozen maximum distance used for order-preserving association.
    lower = max(float(plateau_start), float(refs[0]) - 0.25)
    upper = min(float(plateau_end) - NUMERIC_TOLERANCE, float(refs[-1]) + 0.25)
    return [item for item in routed if lower <= item.confirmed_s <= upper]


def feasibility_row(
    reference: Mapping[str, Any],
    routed: Sequence[RoutedHS],
    *,
    trial_id: str,
    cadence: str,
    sample_dt_s: float,
    trial_start_s: float,
    threshold_n: float,
    dwell_s: float,
    routing_mode: str,
    tolerance_s: float,
) -> dict[str, Any]:
    refs = np.asarray(reference["events"]["heel_strike"], dtype=float)
    local = _local_routed_events(routed, reference)
    matching = v14.thresholds.match_events(
        refs, [item.confirmed_s for item in local], tolerance_s
    )
    associated_indices = _nearest_unique(
        refs,
        [item.confirmed_s for item in local],
        maximum_distance_s=0.25,
    )
    diagnostic_errors = [
        float(local[candidate_index].confirmed_s - refs[reference_index])
        for reference_index, candidate_index in sorted(associated_indices.items())
    ]
    diagnostic_outside = sum(
        abs(value) > tolerance_s + NUMERIC_TOLERANCE
        for value in diagnostic_errors
    )
    diagnostic_failed = diagnostic_outside + int(len(refs) - len(diagnostic_errors))
    exact_counts = len(local) == len(refs)
    feasible = bool(
        exact_counts
        and matching["matched_count"] == len(refs)
        and matching["precision"] == 1.0
        and matching["recall"] == 1.0
    )
    source_counts = {
        sensor: sum(item.source_sensor == sensor for item in local)
        for sensor in ("heel", "forefoot")
    }
    key = (trial_id, int(reference["plateau_index"]), cadence)
    ordered_onset_s = [float(item.onset_s) for item in local]
    ordered_confirmed_s = [float(item.confirmed_s) for item in local]
    ordered_sources = [str(item.source_sensor) for item in local]
    reference_digest = str(
        reference.get("reference_event_sha256")
        or _canonical_sha256(
            {
                event: np.asarray(values, dtype=float).tolist()
                for event, values in reference["events"].items()
            }
        )
    )
    sequence_payload = _sequence_payload(
        key=key,
        reference_event_sha256=reference_digest,
        onset_indices=_strict_grid_indices(
            ordered_onset_s,
            trial_start_s=trial_start_s,
            sample_dt_s=sample_dt_s,
            label=f"routed onset {key}",
        ),
        confirmed_indices=_strict_grid_indices(
            ordered_confirmed_s,
            trial_start_s=trial_start_s,
            sample_dt_s=sample_dt_s,
            label=f"routed confirmed {key}",
        ),
        source_sensors=ordered_sources,
    )
    return {
        "trial_id": trial_id,
        "plateau_index": int(reference["plateau_index"]),
        "speed_mps": float(reference["speed_mps"]),
        "cadence": cadence,
        "sample_dt_s": sample_dt_s,
        "trial_start_s": trial_start_s,
        "routing_mode": routing_mode,
        "sensor_on_threshold_n": threshold_n,
        "sensor_off_threshold_n": 0.25,
        "sensor_dwell_s": dwell_s,
        "reference_hs_count": int(len(refs)),
        "predicted_hs_count": int(len(local)),
        "matched_hs_count": int(matching["matched_count"]),
        "precision": float(matching["precision"]),
        "recall": float(matching["recall"]),
        "max_abs_confirmed_error_s": (
            None
            if not matching["pairs"]
            else max(float(item["absolute_error_s"]) for item in matching["pairs"])
        ),
        "unmatched_reference_count": len(matching["unmatched_reference"]),
        "unmatched_predicted_count": len(matching["unmatched_predicted"]),
        "diagnostic_associated_reference_count": len(diagnostic_errors),
        "diagnostic_failed_reference_count": diagnostic_failed,
        "diagnostic_confirmed_error_min_s": (
            None if not diagnostic_errors else min(diagnostic_errors)
        ),
        "diagnostic_confirmed_error_max_s": (
            None if not diagnostic_errors else max(diagnostic_errors)
        ),
        "diagnostic_confirmed_error_max_abs_s": (
            None if not diagnostic_errors else max(abs(x) for x in diagnostic_errors)
        ),
        "reference_event_sha256": reference_digest,
        "ordered_routed_onset_s_json": json.dumps(
            ordered_onset_s, separators=(",", ":"), allow_nan=False
        ),
        "ordered_routed_confirmed_s_json": json.dumps(
            ordered_confirmed_s, separators=(",", ":"), allow_nan=False
        ),
        "ordered_routed_source_sensor_json": json.dumps(
            ordered_sources, separators=(",", ":"), allow_nan=False
        ),
        "routed_hs_sequence_sha256": _canonical_sha256(sequence_payload),
        "heel_routed_count": source_counts["heel"],
        "forefoot_routed_count": source_counts["forefoot"],
        "exact_count_and_50ms_timing_feasible": feasible,
        "diagnostic_only_not_a_candidate_gate": True,
    }


def required_ground_normal_offsets_m(
    profile: Any,
    sphere: Any,
    samples: Mapping[str, Any],
    *,
    threshold_n: float,
) -> np.ndarray:
    """Invert the frozen Hunt-Crossley normal law at every V13 sample."""

    normal = np.asarray(profile.ground.normal, dtype=float)
    normal /= np.linalg.norm(normal)
    origin = np.asarray(profile.ground.origin, dtype=float)
    centers = np.asarray(samples["centers"][sphere.name], dtype=float)
    velocities = np.asarray(samples["velocities"][sphere.name], dtype=float)
    material = sphere.material or profile.material
    penetration_raw = float(sphere.radius) - (centers - origin) @ normal
    normal_velocity = (
        velocities - np.asarray(profile.ground.surface_velocity, dtype=float)
    ) @ normal
    epsilon = float(material.smoothing)
    damping_raw = 1.0 - float(material.dissipation) * normal_velocity
    damping = 0.5 * (
        damping_raw + np.sqrt(damping_raw * damping_raw + epsilon * epsilon)
    )
    required_smoothed_penetration = np.power(
        float(threshold_n) / (float(material.stiffness) * damping),
        1.0 / float(material.exponent),
    )
    required_raw_penetration = required_smoothed_penetration - (
        epsilon * epsilon / (4.0 * required_smoothed_penetration)
    )
    result = np.maximum(0.0, required_raw_penetration - penetration_raw)
    if not np.all(np.isfinite(result)) or np.any(result < 0.0):
        raise ProtocolError("non-finite V15 Hy clearance estimate")
    return result


def minimum_hy_offset_for_reference(
    times: Sequence[float] | np.ndarray,
    required_offsets_m: Sequence[float] | np.ndarray,
    *,
    reference_hs_s: float,
    dwell_s: float,
    maximum_confirmed_error_s: float,
    search_lookback_s: float,
) -> dict[str, Any]:
    """Find the optimistic minimum constant offset over a causal dwell window."""

    sample_times = np.asarray(times, dtype=float)
    required = np.asarray(required_offsets_m, dtype=float)
    earliest_confirmation = float(reference_hs_s) - float(maximum_confirmed_error_s)
    deadline = float(reference_hs_s) + float(maximum_confirmed_error_s)
    start_limit = float(reference_hs_s) - float(search_lookback_s)
    candidates: list[tuple[float, int, int]] = []
    for onset_index in np.flatnonzero(
        (sample_times >= start_limit - NUMERIC_TOLERANCE)
        & (sample_times <= deadline + NUMERIC_TOLERANCE)
    ):
        target = float(sample_times[onset_index]) + float(dwell_s)
        confirmation_index = int(np.searchsorted(sample_times, target - 1e-12))
        if confirmation_index >= sample_times.size:
            continue
        confirmed_s = float(sample_times[confirmation_index])
        if (
            confirmed_s < earliest_confirmation - NUMERIC_TOLERANCE
            or confirmed_s > deadline + NUMERIC_TOLERANCE
        ):
            continue
        offset = float(np.max(required[onset_index : confirmation_index + 1]))
        candidates.append((offset, int(onset_index), confirmation_index))
    if not candidates:
        return {
            "feasible_window_found": False,
            "equivalent_plantar_ground_normal_offset_mm": None,
            "window_onset_s": None,
            "window_confirmed_s": None,
            "confirmed_error_s": None,
            "window_only_optimistic_bound": True,
            "rising_edge_and_rearm_not_proven": True,
        }
    offset, onset_index, confirmation_index = min(
        candidates,
        key=lambda item: (item[0], sample_times[item[2]], sample_times[item[1]]),
    )
    return {
        "feasible_window_found": True,
        "equivalent_plantar_ground_normal_offset_mm": 1000.0 * offset,
        "window_onset_s": float(sample_times[onset_index]),
        "window_confirmed_s": float(sample_times[confirmation_index]),
        "confirmed_error_s": float(sample_times[confirmation_index])
        - float(reference_hs_s),
        "window_only_optimistic_bound": True,
        "rising_edge_and_rearm_not_proven": True,
    }


def _focus_cell(protocol: Mapping[str, Any], reference: Mapping[str, Any]) -> str:
    key = f"{reference['trial_id']}/P{int(reference['plateau_index'])}"
    return str(protocol["focus_cells"].get(key, "context"))


def _hy_checkpoint_columns(
    minimum_offset_mm: float | None, checkpoints_mm: Sequence[float]
) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for checkpoint in checkpoints_mm:
        token = str(float(checkpoint)).replace(".", "p")
        result[f"window_bound_at_or_below_hy_{token}mm"] = bool(
            minimum_offset_mm is not None
            and float(checkpoint) + 1e-9 >= float(minimum_offset_mm)
        )
    return result


def _event_detail_rows(
    reference: Mapping[str, Any],
    *,
    trial_id: str,
    cadence: str,
    sample_dt_s: float,
    raw_by_sensor: Mapping[str, Sequence[float]],
    stable_by_sensor: Mapping[str, Sequence[SensorEdge]],
    routed_by_mode: Mapping[str, Sequence[RoutedHS]],
    association_window_s: float,
    focus_label: str,
) -> list[dict[str, Any]]:
    refs = np.asarray(reference["events"]["heel_strike"], dtype=float)
    raw_maps = {
        sensor: _nearest_unique(
            refs, values, maximum_distance_s=association_window_s
        )
        for sensor, values in raw_by_sensor.items()
    }
    stable_on = {
        sensor: [item for item in values if item.edge == "contact_on"]
        for sensor, values in stable_by_sensor.items()
    }
    stable_maps = {
        sensor: _nearest_unique(
            refs,
            [item.confirmed_s for item in values],
            maximum_distance_s=association_window_s,
        )
        for sensor, values in stable_on.items()
    }
    route_maps = {
        mode: _nearest_unique(
            refs,
            [item.confirmed_s for item in values],
            maximum_distance_s=association_window_s,
        )
        for mode, values in routed_by_mode.items()
    }
    rows: list[dict[str, Any]] = []
    for reference_index, reference_s in enumerate(refs):
        row: dict[str, Any] = {
            "trial_id": trial_id,
            "plateau_index": int(reference["plateau_index"]),
            "speed_mps": float(reference["speed_mps"]),
            "cadence": cadence,
            "sample_dt_s": sample_dt_s,
            "reference_hs_index": reference_index,
            "reference_hs_s": float(reference_s),
            "focus_label": focus_label,
            "sensor_on_threshold_n": 0.5,
            "sensor_off_threshold_n": 0.25,
            "sensor_dwell_s": 0.03,
        }
        for sensor in ("heel", "forefoot"):
            raw_index = raw_maps[sensor].get(reference_index)
            edge_index = stable_maps[sensor].get(reference_index)
            edge = None if edge_index is None else stable_on[sensor][edge_index]
            row[f"{sensor}_raw_threshold_s"] = (
                None if raw_index is None else float(raw_by_sensor[sensor][raw_index])
            )
            row[f"{sensor}_stable_onset_s"] = None if edge is None else edge.onset_s
            row[f"{sensor}_stable_confirmed_s"] = (
                None if edge is None else edge.confirmed_s
            )
            row[f"{sensor}_confirmed_error_s"] = (
                None if edge is None else edge.confirmed_s - float(reference_s)
            )
        for mode in ROUTING_MODES:
            route_index = route_maps[mode].get(reference_index)
            event = None if route_index is None else routed_by_mode[mode][route_index]
            row[f"{mode}_source_sensor"] = (
                None if event is None else event.source_sensor
            )
            row[f"{mode}_onset_s"] = None if event is None else event.onset_s
            row[f"{mode}_confirmed_s"] = None if event is None else event.confirmed_s
            row[f"{mode}_confirmed_error_s"] = (
                None if event is None else event.confirmed_s - float(reference_s)
            )
        rows.append(row)
    return rows


def _sample_v13_trial(
    protocol: Mapping[str, Any], artifacts: SimpleNamespace, profile: Any
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]], dict[str, Any]]:
    trial_id = artifacts.trial_id
    references = load_frozen_references(protocol, trial_id)
    sensors = v1._left_sensor_spheres(profile)
    heel_sphere = sensors["left_heel"]
    event_rows: list[dict[str, Any]] = []
    feasibility_rows: list[dict[str, Any]] = []
    hy_rows: list[dict[str, Any]] = []
    trace_hashes: dict[str, Any] = {}
    trial_start, trial_end = (
        float(value) for value in protocol["trials"][trial_id]["trial_interval_s"]
    )
    feasibility = protocol["feasibility_map"]
    hy_contract = protocol["hy_clearance_estimate"]

    for cadence, sample_dt_s in CADENCES:
        intervals = int(
            math.floor((trial_end - trial_start) / sample_dt_s + NUMERIC_TOLERANCE)
        )
        times = trial_start + np.arange(intervals + 1, dtype=float) * sample_dt_s
        samples = v1._sample_spheres(
            artifacts.setup, profile, times, artifacts.plugin_loader
        )
        loads, _penetrations, _aggregate = (
            v14.thresholds._regional_loads_and_penetrations(profile, samples)
        )
        heel_force = np.asarray(loads["left_heel"], dtype=float)
        forefoot_force = np.asarray(loads["left_toe"], dtype=float)
        trace_hashes[cadence] = {
            "times_sha256": _canonical_sha256(times.tolist()),
            "heel_force_sha256": _canonical_sha256(heel_force.tolist()),
            "forefoot_force_sha256": _canonical_sha256(forefoot_force.tolist()),
            "sample_count": int(times.size),
        }

        incumbent_edges = {
            "heel": debounce_sensor_edges(
                times,
                heel_force,
                sensor="heel",
                on_threshold_n=0.5,
                off_threshold_n=0.25,
                dwell_s=0.03,
            ),
            "forefoot": debounce_sensor_edges(
                times,
                forefoot_force,
                sensor="forefoot",
                on_threshold_n=0.5,
                off_threshold_n=0.25,
                dwell_s=0.03,
            ),
        }
        incumbent_routed = {
            mode: route_hs_trace(
                times,
                heel_force,
                forefoot_force,
                incumbent_edges["heel"],
                incumbent_edges["forefoot"],
                routing_mode=mode,
                on_threshold_n=0.5,
                off_threshold_n=0.25,
                dwell_s=0.03,
            )
            for mode in ROUTING_MODES
        }
        incumbent_raw = {
            "heel": raw_rising_crossings(times, heel_force, 0.5),
            "forefoot": raw_rising_crossings(times, forefoot_force, 0.5),
        }

        required_hy = required_ground_normal_offsets_m(
            profile, heel_sphere, samples, threshold_n=0.5
        )
        for reference in references:
            focus_label = _focus_cell(protocol, reference)
            event_rows.extend(
                _event_detail_rows(
                    reference,
                    trial_id=trial_id,
                    cadence=cadence,
                    sample_dt_s=sample_dt_s,
                    raw_by_sensor=incumbent_raw,
                    stable_by_sensor=incumbent_edges,
                    routed_by_mode=incumbent_routed,
                    association_window_s=float(
                        protocol["event_association"]["maximum_distance_s"]
                    ),
                    focus_label=focus_label,
                )
            )
            for index, reference_s in enumerate(reference["events"]["heel_strike"]):
                estimate = minimum_hy_offset_for_reference(
                    times,
                    required_hy,
                    reference_hs_s=float(reference_s),
                    dwell_s=float(hy_contract["dwell_s"]),
                    maximum_confirmed_error_s=float(
                        hy_contract["maximum_confirmed_error_s"]
                    ),
                    search_lookback_s=float(hy_contract["search_lookback_s"]),
                )
                minimum_offset_mm = estimate[
                    "equivalent_plantar_ground_normal_offset_mm"
                ]
                hy_rows.append(
                    {
                        "trial_id": trial_id,
                        "plateau_index": int(reference["plateau_index"]),
                        "speed_mps": float(reference["speed_mps"]),
                        "cadence": cadence,
                        "sample_dt_s": sample_dt_s,
                        "reference_hs_index": index,
                        "reference_hs_s": float(reference_s),
                        "focus_label": focus_label,
                        "sensor_on_threshold_n": 0.5,
                        "sensor_dwell_s": 0.03,
                        **estimate,
                        **_hy_checkpoint_columns(
                            minimum_offset_mm,
                            [
                                float(value)
                                for value in hy_contract["reporting_checkpoints_mm"]
                            ],
                        ),
                        "estimate_scope": (
                            "optimistic_constant_ground_normal_clearance_shift_"
                            "on_frozen_v13_signal"
                        ),
                        "estimate_is_optimistic_lower_bound_only": True,
                        "upper_bound_or_global_hy_intersection_computed": False,
                        "not_exact_rotating_foot_frame_local_y": True,
                        "new_geometry_sampled": False,
                    }
                )

        for threshold_n in feasibility["sensor_on_n"]:
            for dwell_s in feasibility["dwell_s"]:
                edges = {
                    "heel": debounce_sensor_edges(
                        times,
                        heel_force,
                        sensor="heel",
                        on_threshold_n=float(threshold_n),
                        off_threshold_n=float(feasibility["sensor_off_n"]),
                        dwell_s=float(dwell_s),
                    ),
                    "forefoot": debounce_sensor_edges(
                        times,
                        forefoot_force,
                        sensor="forefoot",
                        on_threshold_n=float(threshold_n),
                        off_threshold_n=float(feasibility["sensor_off_n"]),
                        dwell_s=float(dwell_s),
                    ),
                }
                for mode in ROUTING_MODES:
                    routed = route_hs_trace(
                        times,
                        heel_force,
                        forefoot_force,
                        edges["heel"],
                        edges["forefoot"],
                        routing_mode=mode,
                        on_threshold_n=float(threshold_n),
                        off_threshold_n=float(feasibility["sensor_off_n"]),
                        dwell_s=float(dwell_s),
                    )
                    for reference in references:
                        feasibility_rows.append(
                            feasibility_row(
                                reference,
                                routed,
                                trial_id=trial_id,
                                cadence=cadence,
                                sample_dt_s=sample_dt_s,
                                trial_start_s=trial_start,
                                threshold_n=float(threshold_n),
                                dwell_s=float(dwell_s),
                                routing_mode=mode,
                                tolerance_s=float(feasibility["hs_tolerance_s"]),
                            )
                        )

    return event_rows, feasibility_rows, hy_rows, trace_hashes


def _json_safe(value: Any) -> Any:
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _write_json_exclusive(path: Path, payload: Mapping[str, Any]) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as handle:
        json.dump(_json_safe(dict(payload)), handle, indent=2, sort_keys=True)
        handle.write("\n")
    return path


def _write_jsonl_exclusive(path: Path, rows: Sequence[Mapping[str, Any]]) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as handle:
        for row in rows:
            handle.write(
                json.dumps(
                    _json_safe(dict(row)), sort_keys=True, separators=(",", ":")
                )
                + "\n"
            )
    return path


def _write_csv_exclusive(path: Path, rows: Sequence[Mapping[str, Any]]) -> Path:
    if not rows:
        raise ProtocolError(f"refusing to write empty V15 CSV: {_portable(path)}")
    fieldnames = list(rows[0])
    if any(set(row) != set(fieldnames) for row in rows):
        raise ProtocolError("V15 CSV rows have inconsistent columns")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(_json_safe(list(rows)))
    return path


def _artifact_record(path: Path) -> dict[str, Any]:
    return {
        "path": _portable(path),
        "sha256": _sha256(path),
        "bytes": path.stat().st_size,
    }


def _preflight_no_clobber(output_dir: Path) -> None:
    if output_dir.exists():
        raise NoClobberError(f"V15 output destination exists: {_portable(output_dir)}")
    if output_dir.resolve() != DEFAULT_OUTPUT_DIR.resolve():
        raise NoClobberError("V15 execution is restricted to its canonical destination")


def _run_start_receipt(
    output_dir: Path, protocol: Mapping[str, Any]
) -> Path:
    output_dir.mkdir(parents=True, exist_ok=False)
    return _write_json_exclusive(
        output_dir / "run_start_receipt.json",
        {
            "schema_version": SCHEMA_VERSION,
            "status": "V15_DEVELOPMENT_DIAGNOSTIC_STARTED",
            "protocol": {
                "protocol_id": protocol["protocol_id"],
                "path": _portable(Path(str(protocol["_protocol_path"]))),
                "sha256": protocol["_protocol_sha256"],
            },
            "authorized_trials": list(ALLOWED_TRIALS),
            "forbidden_trials": list(FORBIDDEN_TRIALS),
            "development_only": True,
            "written_before_sphere_sampling_and_output_generation": True,
            "runtime_or_fsm_modified": False,
            "new_geometry_sampled": False,
            "candidate_selection_or_promotion_allowed": False,
            "rerun_allowed": False,
        },
    )


def preflight_live_inputs(protocol: Mapping[str, Any]) -> dict[str, Any]:
    """Validate every live DEV input before consuming the one-shot output."""

    parent = validate_parent_provenance(protocol)
    incumbent_sequences = load_v14_2_incumbent_sequences(protocol)
    trial_checks: dict[str, Any] = {}
    for trial_id in ALLOWED_TRIALS:
        artifacts = _validate_preprocessing_trial(protocol, trial_id)
        references = load_frozen_references(protocol, trial_id)
        trial_checks[trial_id] = {
            "preprocessing_lock_sha256": _sha256(artifacts.lock_path),
            "plateau_count": len(references),
            "reference_hs_count": sum(
                len(item["events"]["heel_strike"]) for item in references
            ),
        }
    profile_path = _resolve(protocol["geometry"]["profile"]["path"])
    profile = v1.load_online_grf_profile(profile_path, required_sides=("left",))
    sensors = v1._left_sensor_spheres(profile)
    observed_spheres = [
        {
            "name": sphere.name,
            "frame": sphere.frame,
            "location": list(sphere.location),
            "radius": float(sphere.radius),
        }
        for sphere in profile.spheres
    ]
    if (
        len(profile.spheres) != 2
        or set(sensors) != {"left_heel", "left_toe"}
        or observed_spheres != protocol["geometry"]["spheres"]
    ):
        raise ProtocolError("V13 detector geometry drifted during V15 preflight")
    return {
        "status": "PASS_V15_FULL_LIVE_INPUT_PREFLIGHT",
        "parent": parent,
        "incumbent_hs_sequence_lock": {
            "unit_count": incumbent_sequences["unit_count"],
            "event_count": incumbent_sequences["event_count"],
            "aggregate_sha256": incumbent_sequences["aggregate_sha256"],
        },
        "trials": trial_checks,
        "profile_sha256": _sha256(profile_path),
        "destination_unconsumed": True,
    }


def run_diagnostic(
    protocol: Mapping[str, Any], output_dir: Path = DEFAULT_OUTPUT_DIR
) -> dict[str, Any]:
    parent = validate_parent_provenance(protocol)
    receipt = output_dir / "run_start_receipt.json"
    if not receipt.is_file():
        raise ProtocolError("V15 run receipt must precede semantic sampling")
    profile_path = _resolve(protocol["geometry"]["profile"]["path"])
    profile = v1.load_online_grf_profile(profile_path, required_sides=("left",))
    sensors = v1._left_sensor_spheres(profile)
    if len(profile.spheres) != 2 or set(sensors) != {"left_heel", "left_toe"}:
        raise ProtocolError("V15 requires exact frozen V13 heel + toe geometry")
    expected_spheres = protocol["geometry"]["spheres"]
    observed_spheres = [
        {
            "name": sphere.name,
            "frame": sphere.frame,
            "location": list(sphere.location),
            "radius": float(sphere.radius),
        }
        for sphere in profile.spheres
    ]
    if observed_spheres != expected_spheres:
        raise ProtocolError("V13 detector geometry drifted before V15 sampling")

    all_events: list[dict[str, Any]] = []
    all_feasibility: list[dict[str, Any]] = []
    all_hy: list[dict[str, Any]] = []
    provenance: dict[str, Any] = {}
    for trial_id in ALLOWED_TRIALS:
        artifacts = _validate_preprocessing_trial(protocol, trial_id)
        events, feasibility, hy_rows, trace_hashes = _sample_v13_trial(
            protocol, artifacts, profile
        )
        all_events.extend(events)
        all_feasibility.extend(feasibility)
        all_hy.extend(hy_rows)
        provenance[trial_id] = {
            "preprocessing_lock": {
                "path": _portable(artifacts.lock_path),
                "sha256": _sha256(artifacts.lock_path),
            },
            "trace_hashes": trace_hashes,
        }

    observed_cardinality = {
        "event_detail_rows": len(all_events),
        "feasibility_rows": len(all_feasibility),
        "hy_rows": len(all_hy),
    }
    expected_cardinality = {
        "event_detail_rows": EXPECTED_EVENT_AND_HY_ROWS,
        "feasibility_rows": EXPECTED_FEASIBILITY_ROWS,
        "hy_rows": EXPECTED_EVENT_AND_HY_ROWS,
    }
    if observed_cardinality != expected_cardinality:
        raise ProtocolError(
            "V15 output cardinality drifted before write: "
            f"{observed_cardinality} != {expected_cardinality}"
        )
    incumbent_parity = incumbent_v14_2_parity(protocol, all_feasibility)
    if not incumbent_parity["ok"]:
        raise ProtocolError(
            "V15 incumbent full HS sequence does not match frozen V14.2"
        )

    event_path = _write_jsonl_exclusive(
        output_dir / "v15_incumbent_event_details.jsonl", all_events
    )
    feasibility_path = _write_csv_exclusive(
        output_dir / "v15_threshold_dwell_routing_feasibility.csv",
        all_feasibility,
    )
    hy_path = _write_csv_exclusive(
        output_dir / "v15_hy_clearance_feasibility.csv", all_hy
    )

    feasible_by_mode: dict[str, int] = {}
    for mode in ROUTING_MODES:
        feasible_by_mode[mode] = sum(
            bool(row["exact_count_and_50ms_timing_feasible"])
            for row in all_feasibility
            if row["routing_mode"] == mode
        )
    global_configuration_summary: list[dict[str, Any]] = []
    for threshold_n in SENSOR_ON_GRID_N:
        for dwell_s in DWELL_GRID_S:
            for mode in ROUTING_MODES:
                group = [
                    row
                    for row in all_feasibility
                    if row["routing_mode"] == mode
                    and math.isclose(
                        float(row["sensor_on_threshold_n"]),
                        threshold_n,
                        abs_tol=NUMERIC_TOLERANCE,
                    )
                    and math.isclose(
                        float(row["sensor_dwell_s"]),
                        dwell_s,
                        abs_tol=NUMERIC_TOLERANCE,
                    )
                ]
                if len(group) != EXPECTED_UNIT_COUNT:
                    raise ProtocolError("V15 global configuration lost a unit")
                finite_errors = [
                    value
                    for row in group
                    for value in (
                        row["diagnostic_confirmed_error_min_s"],
                        row["diagnostic_confirmed_error_max_s"],
                    )
                    if value is not None
                ]
                global_configuration_summary.append(
                    {
                        "sensor_on_threshold_n": threshold_n,
                        "sensor_off_threshold_n": 0.25,
                        "sensor_dwell_s": dwell_s,
                        "routing_mode": mode,
                        "unit_count": len(group),
                        "feasible_unit_count": sum(
                            bool(row["exact_count_and_50ms_timing_feasible"])
                            for row in group
                        ),
                        "failed_reference_count": sum(
                            int(row["diagnostic_failed_reference_count"])
                            for row in group
                        ),
                        "confirmed_error_min_s": (
                            min(finite_errors) if finite_errors else None
                        ),
                        "confirmed_error_max_s": (
                            max(finite_errors) if finite_errors else None
                        ),
                        "all_24_units_feasible": all(
                            row["exact_count_and_50ms_timing_feasible"]
                            for row in group
                        ),
                        "signal_level_only_not_full_fsm_gate": True,
                    }
                )
    incumbent_routing_dwell_summary: list[dict[str, Any]] = []
    for cadence, _sample_dt in CADENCES:
        for mode in ROUTING_MODES:
            for dwell_s in protocol["feasibility_map"]["dwell_s"]:
                group = [
                    row
                    for row in all_feasibility
                    if row["cadence"] == cadence
                    and row["routing_mode"] == mode
                    and math.isclose(
                        float(row["sensor_on_threshold_n"]),
                        0.5,
                        abs_tol=NUMERIC_TOLERANCE,
                    )
                    and math.isclose(
                        float(row["sensor_dwell_s"]),
                        float(dwell_s),
                        abs_tol=NUMERIC_TOLERANCE,
                    )
                ]
                finite_min = [
                    float(row["diagnostic_confirmed_error_min_s"])
                    for row in group
                    if row["diagnostic_confirmed_error_min_s"] is not None
                ]
                finite_max = [
                    float(row["diagnostic_confirmed_error_max_s"])
                    for row in group
                    if row["diagnostic_confirmed_error_max_s"] is not None
                ]
                incumbent_routing_dwell_summary.append(
                    {
                        "cadence": cadence,
                        "routing_mode": mode,
                        "sensor_on_threshold_n": 0.5,
                        "sensor_dwell_s": float(dwell_s),
                        "failed_reference_count": sum(
                            int(row["diagnostic_failed_reference_count"])
                            for row in group
                        ),
                        "confirmed_error_min_s": min(finite_min)
                        if finite_min
                        else None,
                        "confirmed_error_max_s": max(finite_max)
                        if finite_max
                        else None,
                        "unit_count": len(group),
                        "all_12_units_feasible": bool(
                            len(group) == EXPECTED_UNITS_PER_CADENCE
                            and all(
                                row["exact_count_and_50ms_timing_feasible"]
                                for row in group
                            )
                        ),
                    }
                )
    focus_hy = [
        row
        for row in all_hy
        if row["trial_id"] == "02"
        and row["plateau_index"] == 3
        and row["reference_hs_index"] == 0
    ]
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "status": "PASS_DIAGNOSTIC_ONLY_NO_CANDIDATE_SELECTED",
        "protocol": {
            "protocol_id": protocol["protocol_id"],
            "path": _portable(Path(str(protocol["_protocol_path"]))),
            "sha256": protocol["_protocol_sha256"],
        },
        "parent": parent,
        "scope": {
            "development_trials": list(ALLOWED_TRIALS),
            "validation_opened": False,
            "sealed_opened": False,
            "reserve_opened": False,
            "trial01_opened": False,
        },
        "geometry": {
            "candidate_id": BASELINE_ID,
            "profile": protocol["geometry"]["profile"],
            "new_geometry_sampled": False,
            "hy_is_signal_only_equivalent_ground_normal_offset": True,
            "hy_is_lower_bound_only_no_global_interval_claim": True,
        },
        "counts": {
            **observed_cardinality,
            "expected": expected_cardinality,
            "passing_unit_rows_by_mode_across_all_grid_rows": feasible_by_mode,
        },
        "global_threshold_dwell_routing_summary": global_configuration_summary,
        "incumbent_v14_2_parity": incumbent_parity,
        "incumbent_threshold_routing_dwell_summary": (
            incumbent_routing_dwell_summary
        ),
        "focus_02_p3_first_hs_hy": focus_hy,
        "provenance": provenance,
        "artifacts": {
            "event_details": _artifact_record(event_path),
            "routing_feasibility": _artifact_record(feasibility_path),
            "hy_clearance_feasibility": _artifact_record(hy_path),
        },
        "non_actions": {
            "runtime_or_fsm_modified": False,
            "profile_or_registry_modified": False,
            "candidate_selected_or_promoted": False,
            "training_run": False,
            "holdout_or_reserve_opened": False,
        },
        "interpretation_limits": protocol["interpretation_limits"],
    }
    manifest_path = _write_json_exclusive(output_dir / "manifest.json", manifest)
    return {**manifest, "manifest_sha256": _sha256(manifest_path)}


def _write_failure(output_dir: Path, exc: Exception) -> None:
    if not output_dir.is_dir():
        return
    path = output_dir / "failure.json"
    if path.exists():
        return
    _write_json_exclusive(
        path,
        {
            "schema_version": SCHEMA_VERSION,
            "status": "ERROR_AFTER_V15_DESTINATION_CONSUMED",
            "error": f"{type(exc).__name__}: {exc}",
            "traceback": traceback.format_exc(),
            "rerun_allowed": False,
            "validation_opened": False,
            "sealed_opened": False,
            "reserve_opened": False,
        },
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol", default=str(DEFAULT_PROTOCOL))
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument(
        "--execute",
        action="store_true",
        help="consume the canonical V15 destination and run DEV02/04/08",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="validate frozen inputs without sampling or writing output",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    if args.execute and args.check:
        raise SystemExit("--execute and --check are mutually exclusive")
    protocol = load_and_validate_protocol(args.protocol)
    live_preflight = preflight_live_inputs(protocol)
    if not args.execute:
        print(
            json.dumps(
                {
                    "status": "PASS_V15_READ_ONLY_PREFLIGHT",
                    "protocol_id": protocol["protocol_id"],
                    "protocol_sha256": protocol["_protocol_sha256"],
                    "authorized_trials": list(ALLOWED_TRIALS),
                    "forbidden_trials": list(FORBIDDEN_TRIALS),
                    "canonical_run_not_started": True,
                    "live_input_preflight": live_preflight,
                },
                indent=2,
                sort_keys=True,
            )
        )
        return 0

    output_dir = _resolve(args.output_dir)
    _preflight_no_clobber(output_dir)
    _run_start_receipt(output_dir, protocol)
    try:
        result = run_diagnostic(protocol, output_dir)
    except Exception as exc:
        _write_failure(output_dir, exc)
        print(f"V15 diagnostic failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(_json_safe(result), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
