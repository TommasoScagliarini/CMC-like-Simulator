"""Acquire immutable 1 ms V18 heel/toe detector traces on development data.

This module intentionally performs no event scoring.  It reuses the frozen
V14.2 prescribed-replay setup only to sample the fixed V17 two-sphere detector
profile.  Selection acquisition can reach trials 02 and 04 only.  Trial 08 is
reserved for a later one-shot command after a finalist lock exists and is not
reachable from this runner version.

V15's signal-only router and its V14 cadence-dependent references are never
called.  Downstream V18 scoring must consume the already-frozen canonical V17
event ledgers verbatim.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import sys
import traceback
import uuid
from pathlib import Path, PurePosixPath, PureWindowsPath
from typing import Any, Mapping, Sequence

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
for _path in (REPO_ROOT, VALIDATION_ROOT):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import diagnose_two_sensor_v15_routing as v15  # noqa: E402


SCHEMA_VERSION = 18
PROTOCOL_ID = "AB06_TWO_SENSOR_SIGNAL_SEMANTICS_2026-08-03_V18"
EVENT_CONTRACT_ID = "primary_grf_split_v1+two_sensor_highrate_v1"
SELECTION_TRIALS = ("02", "04")
INTERNAL_HOLDOUT_TRIAL = "08"
FORBIDDEN_TRIALS = frozenset({"01", "03", "05", "06", "07"})
SAMPLE_DT_S = 0.001
EXPECTED_SAMPLE_COUNTS = {"02": 143206, "04": 143541, "08": 144201}
EXPECTED_INTERVALS_S = {
    "02": (9.875, 153.08),
    "04": (12.485, 156.025),
    "08": (10.69, 154.89),
}
DEFAULT_PROTOCOL = VALIDATION_ROOT / "two_sensor_v18_signal_semantics_protocol.json"
DEFAULT_SELECTION_OUTPUT = (
    VALIDATION_ROOT
    / "two_sensor_v18_signal_semantics_runs/"
    "2026-08-03_ab06_dev02_04_v18_selection_traces"
)
V15_PROTOCOL = VALIDATION_ROOT / "two_sensor_v15_routing_protocol.json"
V17_PROFILE = (
    VALIDATION_ROOT
    / "experimental_detector_profiles/two_sensor_v17_high_rate_v13_geometry.json"
)
TRACE_COLUMNS = (
    "time_s",
    "left_heel_normal_n",
    "left_toe_normal_n",
    "left_heel_penetration_m",
    "left_toe_penetration_m",
)
NUMERIC_TOLERANCE = 1.0e-10


class V18AcquisitionError(RuntimeError):
    """Raised when V18 provenance or trace integrity fails closed."""


class V18NoClobberError(V18AcquisitionError):
    """Raised when an immutable V18 destination is already consumed."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _reject_json_constant(value: str) -> None:
    raise V18AcquisitionError(f"non-finite JSON constant is forbidden: {value}")


def load_json_object(path: Path, *, label: str) -> dict[str, Any]:
    try:
        value = json.loads(
            path.read_text(encoding="utf-8"),
            parse_constant=_reject_json_constant,
        )
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise V18AcquisitionError(f"cannot load {label}: {path}") from exc
    if not isinstance(value, dict):
        raise V18AcquisitionError(f"{label} must be a JSON object")
    return value


def _portable_repo_path(value: Any, *, label: str) -> Path:
    if not isinstance(value, str) or not value or value != value.strip():
        raise V18AcquisitionError(f"{label} must be a trimmed relative path")
    if "\\" in value or ":" in value:
        raise V18AcquisitionError(f"{label} must use portable '/' separators")
    posix = PurePosixPath(value)
    windows = PureWindowsPath(value)
    if posix.is_absolute() or windows.is_absolute() or windows.drive:
        raise V18AcquisitionError(f"{label} must be repository-relative")
    if not posix.parts or any(part in {"", ".", ".."} for part in posix.parts):
        raise V18AcquisitionError(f"{label} contains an unsafe path component")
    path = REPO_ROOT.joinpath(*posix.parts).resolve()
    try:
        path.relative_to(REPO_ROOT)
    except ValueError as exc:
        raise V18AcquisitionError(f"{label} escapes the repository") from exc
    return path


def _source_record(path: Path) -> dict[str, Any]:
    resolved = path.resolve()
    if not resolved.is_file():
        raise V18AcquisitionError(f"missing source: {resolved}")
    try:
        portable = resolved.relative_to(REPO_ROOT).as_posix()
    except ValueError as exc:
        raise V18AcquisitionError(f"source escapes repository: {resolved}") from exc
    return {
        "path": portable,
        "sha256": sha256_file(resolved),
        "size_bytes": int(resolved.stat().st_size),
    }


def _require_source(record: Any, *, label: str) -> Path:
    if not isinstance(record, Mapping) or set(record) != {"path", "sha256"}:
        raise V18AcquisitionError(f"invalid source record: {label}")
    path = _portable_repo_path(record["path"], label=f"{label}.path")
    expected = record["sha256"]
    if (
        not isinstance(expected, str)
        or len(expected) != 64
        or any(char not in "0123456789abcdef" for char in expected)
    ):
        raise V18AcquisitionError(f"invalid SHA-256: {label}")
    if not path.is_file():
        raise V18AcquisitionError(f"missing pinned source: {label}")
    observed = sha256_file(path)
    if observed != expected:
        raise V18AcquisitionError(
            f"source hash drift for {label}: {observed} != {expected}"
        )
    return path


def load_and_validate_protocol(
    path: str | Path = DEFAULT_PROTOCOL,
) -> dict[str, Any]:
    protocol_path = Path(path).resolve()
    if protocol_path != DEFAULT_PROTOCOL.resolve():
        raise V18AcquisitionError("V18 accepts only its canonical protocol path")
    raw = load_json_object(protocol_path, label="V18 protocol")
    checks = {
        "schema": raw.get("schema_version") == SCHEMA_VERSION,
        "id": raw.get("protocol_id") == PROTOCOL_ID,
        "frozen": raw.get("frozen_before_execution") is True,
        "stage": raw.get("stage") == "development_only_signal_semantics",
        "event_contract": raw.get("event_contract_id") == EVENT_CONTRACT_ID,
        "sample_dt": raw.get("sampling", {}).get("sample_dt_s") == SAMPLE_DT_S,
        "no_ik_rerun": raw.get("sampling", {}).get("ik_rerun_allowed") is False,
        "canonical_only": raw.get("oracle", {}).get("canonical_ledgers_only") is True,
        "no_training": raw.get("non_actions", {}).get("training_allowed") is False,
        "no_primary_change": raw.get("non_actions", {}).get(
            "primary_grf_modification_allowed"
        )
        is False,
    }
    if not all(checks.values()):
        raise V18AcquisitionError(f"V18 protocol contract drifted: {checks}")

    split = raw.get("split", {})
    if split.get("DIAGNOSIS_CHALLENGE") != ["02"]:
        raise V18AcquisitionError("V18 diagnosis split drifted")
    if split.get("SELECTION") != ["04"]:
        raise V18AcquisitionError("V18 selection split drifted")
    if split.get("INTERNAL_HOLDOUT") != [INTERNAL_HOLDOUT_TRIAL]:
        raise V18AcquisitionError("V18 internal holdout drifted")
    if split.get("FORBIDDEN") != sorted(FORBIDDEN_TRIALS):
        raise V18AcquisitionError("V18 forbidden split drifted")

    intervals = raw.get("sampling", {}).get("trial_intervals_s")
    counts = raw.get("sampling", {}).get("expected_sample_counts")
    if not isinstance(intervals, Mapping) or not isinstance(counts, Mapping):
        raise V18AcquisitionError("V18 sampling maps are missing")
    for trial_id, expected_interval in EXPECTED_INTERVALS_S.items():
        if intervals.get(trial_id) != list(expected_interval):
            raise V18AcquisitionError(f"V18 interval drifted for trial {trial_id}")
        if counts.get(trial_id) != EXPECTED_SAMPLE_COUNTS[trial_id]:
            raise V18AcquisitionError(f"V18 count drifted for trial {trial_id}")

    semantics = raw.get("semantics", {})
    if semantics.get("candidate_ids") != [
        "heel_only",
        "first_stable_regional",
        "combined_load",
    ]:
        raise V18AcquisitionError("V18 candidate set drifted")
    detector = raw.get("detector", {})
    if detector != {
        "sensor_on_threshold_n": 0.5,
        "sensor_off_threshold_n": 0.25,
        "sensor_dwell_s": 0.03,
        "policy_step_s": 0.01,
    }:
        raise V18AcquisitionError("V18 detector parameters drifted")

    sources = raw.get("sources")
    expected_source_keys = {
        "v18_plan",
        "v18_acquisition_runner",
        "v18_acquisition_tests",
        "v15_protocol",
        "v15_runner",
        "v17_profile",
        "v17_contract",
        "v17_failure_receipt",
        "canonical_oracle_manifest",
        "canonical_oracle_trial_02",
        "canonical_oracle_trial_04",
        "canonical_oracle_trial_08",
        "primary_core_lock",
    }
    if not isinstance(sources, Mapping) or set(sources) != expected_source_keys:
        raise V18AcquisitionError("V18 source keyset drifted")
    source_paths = {
        label: _require_source(record, label=f"sources.{label}")
        for label, record in sources.items()
    }
    if source_paths["v15_protocol"] != V15_PROTOCOL.resolve():
        raise V18AcquisitionError("V18 is not bound to the canonical V15 protocol")
    if source_paths["v17_profile"] != V17_PROFILE.resolve():
        raise V18AcquisitionError("V18 is not bound to the canonical V17 profile")

    failure = load_json_object(
        source_paths["v17_failure_receipt"], label="V17 failure receipt"
    )
    if failure.get("decision") != "TERMINAL_SEQUENTIAL_FAIL_NO_RETUNING":
        raise V18AcquisitionError("V17 terminal FAIL provenance drifted")
    if failure.get("gate_id") != "TWO_SENSOR_HIGH_RATE_DEVELOPMENT_READY":
        raise V18AcquisitionError("V17 gate identity drifted")
    if failure.get("data_access", {}).get("protected_trials_opened") != []:
        raise V18AcquisitionError("V17 protected data state drifted")

    raw["_protocol_path"] = protocol_path.as_posix()
    raw["_protocol_sha256"] = sha256_file(protocol_path)
    raw["_source_paths"] = source_paths
    return raw


def build_time_grid(trial_id: str) -> np.ndarray:
    if trial_id not in EXPECTED_INTERVALS_S:
        raise V18AcquisitionError(f"unauthorized V18 trial: {trial_id}")
    start_s, end_s = EXPECTED_INTERVALS_S[trial_id]
    intervals = int(math.floor((end_s - start_s) / SAMPLE_DT_S + 1.0e-12))
    times = start_s + np.arange(intervals + 1, dtype=float) * SAMPLE_DT_S
    if times.size != EXPECTED_SAMPLE_COUNTS[trial_id]:
        raise V18AcquisitionError(
            f"time-grid count mismatch for {trial_id}: {times.size}"
        )
    if times[-1] > end_s + NUMERIC_TOLERANCE:
        raise V18AcquisitionError(f"time grid overshoots trial {trial_id}")
    return times


def validate_trace_arrays(
    *,
    trial_id: str,
    times: Sequence[float] | np.ndarray,
    heel_force_n: Sequence[float] | np.ndarray,
    toe_force_n: Sequence[float] | np.ndarray,
    heel_penetration_m: Sequence[float] | np.ndarray,
    toe_penetration_m: Sequence[float] | np.ndarray,
) -> dict[str, Any]:
    if trial_id not in EXPECTED_SAMPLE_COUNTS:
        raise V18AcquisitionError(f"unauthorized V18 trace trial: {trial_id}")
    arrays = {
        "time_s": np.asarray(times, dtype=float),
        "left_heel_normal_n": np.asarray(heel_force_n, dtype=float),
        "left_toe_normal_n": np.asarray(toe_force_n, dtype=float),
        "left_heel_penetration_m": np.asarray(heel_penetration_m, dtype=float),
        "left_toe_penetration_m": np.asarray(toe_penetration_m, dtype=float),
    }
    expected_count = EXPECTED_SAMPLE_COUNTS[trial_id]
    if any(value.ndim != 1 or value.size != expected_count for value in arrays.values()):
        raise V18AcquisitionError(f"trace shape/count mismatch for trial {trial_id}")
    if any(not np.all(np.isfinite(value)) for value in arrays.values()):
        raise V18AcquisitionError(f"non-finite V18 trace sample for trial {trial_id}")
    if np.any(arrays["left_heel_normal_n"] < 0.0) or np.any(
        arrays["left_toe_normal_n"] < 0.0
    ):
        raise V18AcquisitionError(f"negative detector force for trial {trial_id}")
    if np.any(arrays["left_heel_penetration_m"] < 0.0) or np.any(
        arrays["left_toe_penetration_m"] < 0.0
    ):
        raise V18AcquisitionError(f"negative detector penetration for trial {trial_id}")
    deltas = np.diff(arrays["time_s"])
    if np.any(deltas <= 0.0) or not np.allclose(
        deltas, SAMPLE_DT_S, rtol=0.0, atol=NUMERIC_TOLERANCE
    ):
        raise V18AcquisitionError(f"missing/duplicate/off-grid timestamp in trial {trial_id}")
    expected_times = build_time_grid(trial_id)
    if not np.allclose(
        arrays["time_s"], expected_times, rtol=0.0, atol=NUMERIC_TOLERANCE
    ):
        raise V18AcquisitionError(f"absolute timestamp drift for trial {trial_id}")
    return {
        "sample_count": expected_count,
        "start_time_s": float(arrays["time_s"][0]),
        "last_sample_time_s": float(arrays["time_s"][-1]),
        "nominal_interval_end_s": float(EXPECTED_INTERVALS_S[trial_id][1]),
        "sample_dt_s": SAMPLE_DT_S,
        "heel_force_min_n": float(np.min(arrays["left_heel_normal_n"])),
        "heel_force_max_n": float(np.max(arrays["left_heel_normal_n"])),
        "toe_force_min_n": float(np.min(arrays["left_toe_normal_n"])),
        "toe_force_max_n": float(np.max(arrays["left_toe_normal_n"])),
        "heel_penetration_max_m": float(
            np.max(arrays["left_heel_penetration_m"])
        ),
        "toe_penetration_max_m": float(
            np.max(arrays["left_toe_penetration_m"])
        ),
        "all_finite": True,
        "timestamps_unique_monotonic_exact_1ms": True,
    }


def _atomic_write_json(path: Path, payload: Mapping[str, Any]) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        raise V18NoClobberError(f"refusing to overwrite {path}")
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    try:
        with temporary.open("x", encoding="utf-8", newline="\n") as stream:
            json.dump(dict(payload), stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        if path.exists():
            raise V18NoClobberError(f"refusing to overwrite {path}")
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()
    return path


def _atomic_write_trace_csv(
    path: Path,
    *,
    times: np.ndarray,
    heel_force_n: np.ndarray,
    toe_force_n: np.ndarray,
    heel_penetration_m: np.ndarray,
    toe_penetration_m: np.ndarray,
) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        raise V18NoClobberError(f"refusing to overwrite {path}")
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    try:
        with temporary.open("x", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream, lineterminator="\n")
            writer.writerow(TRACE_COLUMNS)
            for row in zip(
                times,
                heel_force_n,
                toe_force_n,
                heel_penetration_m,
                toe_penetration_m,
            ):
                writer.writerow(tuple(format(float(value), ".17g") for value in row))
            stream.flush()
            os.fsync(stream.fileno())
        if path.exists():
            raise V18NoClobberError(f"refusing to overwrite {path}")
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()
    return path


def preflight(protocol: Mapping[str, Any]) -> dict[str, Any]:
    v15_protocol = v15.load_and_validate_protocol(V15_PROTOCOL)
    profile = v15.v1.load_online_grf_profile(V17_PROFILE, required_sides=("left",))
    sensors = v15.v1._left_sensor_spheres(profile)
    if len(profile.spheres) != 2 or set(sensors) != {"left_heel", "left_toe"}:
        raise V18AcquisitionError("V18 requires exactly one heel and one toe sphere")
    trial_checks: dict[str, Any] = {}
    for trial_id in (*SELECTION_TRIALS, INTERNAL_HOLDOUT_TRIAL):
        artifacts = v15._validate_preprocessing_trial(v15_protocol, trial_id)
        grid = build_time_grid(trial_id)
        trial_checks[trial_id] = {
            "preprocessing_lock": _source_record(artifacts.lock_path),
            "expected_sample_count": int(grid.size),
            "start_time_s": float(grid[0]),
            "last_sample_time_s": float(grid[-1]),
            "sampling_authorized_now": trial_id in SELECTION_TRIALS,
        }
    oracle_paths = protocol["_source_paths"]
    for trial_id in (*SELECTION_TRIALS, INTERNAL_HOLDOUT_TRIAL):
        ledger = load_json_object(
            oracle_paths[f"canonical_oracle_trial_{trial_id}"],
            label=f"canonical oracle trial {trial_id}",
        )
        core = ledger.get("scientific_core", {})
        if (
            core.get("trial_id") != trial_id
            or core.get("event_contract_id") != EVENT_CONTRACT_ID
            or core.get("sample_dt_s") != SAMPLE_DT_S
        ):
            raise V18AcquisitionError(f"canonical oracle drift for trial {trial_id}")
    return {
        "status": "PASS_V18_SELECTION_ACQUISITION_PREFLIGHT",
        "protocol_sha256": protocol["_protocol_sha256"],
        "profile": _source_record(V17_PROFILE),
        "roles": sorted(sensors),
        "trials": trial_checks,
        "selection_destination_unconsumed": not DEFAULT_SELECTION_OUTPUT.exists(),
        "forbidden_trials_reachable": False,
        "internal_holdout_sampled": False,
    }


def _sample_trial(trial_id: str, v15_protocol: Mapping[str, Any], profile: Any) -> tuple[
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    dict[str, Any],
]:
    if trial_id not in SELECTION_TRIALS:
        raise V18AcquisitionError(
            f"selection acquisition cannot sample trial {trial_id}"
        )
    artifacts = v15._validate_preprocessing_trial(v15_protocol, trial_id)
    times = build_time_grid(trial_id)
    samples = v15.v1._sample_spheres(
        artifacts.setup,
        profile,
        times,
        artifacts.plugin_loader,
    )
    loads, penetrations, _aggregate = (
        v15.v14.thresholds._regional_loads_and_penetrations(profile, samples)
    )
    heel_force = np.asarray(loads["left_heel"], dtype=float)
    toe_force = np.asarray(loads["left_toe"], dtype=float)
    heel_penetration = np.asarray(penetrations["left_heel"], dtype=float)
    toe_penetration = np.asarray(penetrations["left_toe"], dtype=float)
    integrity = validate_trace_arrays(
        trial_id=trial_id,
        times=times,
        heel_force_n=heel_force,
        toe_force_n=toe_force,
        heel_penetration_m=heel_penetration,
        toe_penetration_m=toe_penetration,
    )
    integrity["preprocessing_lock"] = _source_record(artifacts.lock_path)
    return (
        times,
        heel_force,
        toe_force,
        heel_penetration,
        toe_penetration,
        integrity,
    )


def execute_selection_acquisition(
    protocol: Mapping[str, Any],
    output_dir: Path = DEFAULT_SELECTION_OUTPUT,
) -> dict[str, Any]:
    destination = output_dir.resolve()
    if destination != DEFAULT_SELECTION_OUTPUT.resolve():
        raise V18NoClobberError("V18 selection output path is fixed")
    if destination.exists():
        raise V18NoClobberError(f"V18 selection destination exists: {destination}")
    destination.mkdir(parents=True, exist_ok=False)
    _atomic_write_json(
        destination / "run_start_receipt.json",
        {
            "schema_version": SCHEMA_VERSION,
            "status": "V18_SELECTION_TRACE_ACQUISITION_STARTED",
            "protocol_id": PROTOCOL_ID,
            "protocol_sha256": protocol["_protocol_sha256"],
            "authorized_trials": list(SELECTION_TRIALS),
            "internal_holdout_trial": INTERNAL_HOLDOUT_TRIAL,
            "internal_holdout_sampled": False,
            "forbidden_trials": sorted(FORBIDDEN_TRIALS),
            "candidate_scoring_performed": False,
            "rerun_allowed": False,
        },
    )

    try:
        v15_protocol = v15.load_and_validate_protocol(V15_PROTOCOL)
        profile = v15.v1.load_online_grf_profile(
            V17_PROFILE, required_sides=("left",)
        )
        trace_records: dict[str, Any] = {}
        for trial_id in SELECTION_TRIALS:
            (
                times,
                heel_force,
                toe_force,
                heel_penetration,
                toe_penetration,
                integrity,
            ) = _sample_trial(trial_id, v15_protocol, profile)
            trace_path = _atomic_write_trace_csv(
                destination / f"trial_{trial_id}_detector_trace_1ms.csv",
                times=times,
                heel_force_n=heel_force,
                toe_force_n=toe_force,
                heel_penetration_m=heel_penetration,
                toe_penetration_m=toe_penetration,
            )
            trace_records[trial_id] = {
                "artifact": _source_record(trace_path),
                "columns": list(TRACE_COLUMNS),
                "integrity": integrity,
            }

        manifest_payload = {
            "schema_version": SCHEMA_VERSION,
            "status": "PASS_V18_SELECTION_RAW_TRACE_LOCKED",
            "gate_id": "V18_RAW_TRACE_LOCKED",
            "protocol": {
                "id": PROTOCOL_ID,
                "path": DEFAULT_PROTOCOL.relative_to(REPO_ROOT).as_posix(),
                "sha256": protocol["_protocol_sha256"],
            },
            "profile": _source_record(V17_PROFILE),
            "sample_dt_s": SAMPLE_DT_S,
            "selection_trials_sampled": list(SELECTION_TRIALS),
            "internal_holdout_trial": INTERNAL_HOLDOUT_TRIAL,
            "internal_holdout_sampled": False,
            "protected_trials_opened": [],
            "reserve_trials_opened": [],
            "candidate_scoring_performed": False,
            "traces": trace_records,
            "interpretation": (
                "Raw detector signals only; no event semantic is selected by this "
                "receipt. Canonical V17 ledgers are mandatory downstream."
            ),
        }
        manifest_path = _atomic_write_json(destination / "manifest.json", manifest_payload)
        return {
            "status": manifest_payload["status"],
            "manifest": _source_record(manifest_path),
            "traces": trace_records,
        }
    except Exception as exc:
        failure_path = destination / "failure.json"
        if not failure_path.exists():
            _atomic_write_json(
                failure_path,
                {
                    "schema_version": SCHEMA_VERSION,
                    "status": "ERROR_AFTER_V18_SELECTION_DESTINATION_CONSUMED",
                    "error": f"{type(exc).__name__}: {exc}",
                    "traceback": traceback.format_exc(),
                    "rerun_allowed": False,
                    "internal_holdout_sampled": False,
                    "protected_trials_opened": [],
                    "reserve_trials_opened": [],
                },
            )
        raise


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--check", action="store_true")
    action.add_argument("--execute-selection", action="store_true")
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_SELECTION_OUTPUT)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        protocol = load_and_validate_protocol(args.protocol)
        check = preflight(protocol)
        if args.check:
            print(json.dumps(check, indent=2, sort_keys=True, allow_nan=False))
            return 0
        result = execute_selection_acquisition(protocol, args.output_dir)
        print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
        return 0
    except Exception as exc:
        print(f"V18 acquisition failed closed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
