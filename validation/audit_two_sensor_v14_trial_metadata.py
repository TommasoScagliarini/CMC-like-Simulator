"""Metadata-only preflight for the AB06 V14 two-sensor trial split.

This module has a deliberately narrow data-access boundary:

* ``conditions/treadmill_02_01.mat`` through ``treadmill_08_01.mat`` may be
  decoded, and only to recover ``trialStarts``, ``trialEnds`` and the MCOS
  ``Header``/``Speed`` table (including its units);
* every other raw stream, plus trial 01's conditions file, is handled only as
  an opaque byte source for existence, size and SHA-256 identity;
* IK, force-plate, gait-cycle and marker values are never decoded, and no
  conversion or detector/event evaluation is performed.

The frozen split is a cross-speed interpolation design, not a set of
exchangeable repeated trials.  Passing this audit only authorizes freezing a
separate V14 development protocol.  It does not open any development,
validation, sealed or reserve event data.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import platform
import re
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from io import BytesIO
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

import numpy as np
import scipy
from scipy.io import loadmat


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RAW_ROOT = REPO_ROOT / "models/AB06-raw/10_09_18/treadmill"
DEFAULT_OUTPUT = (
    REPO_ROOT
    / "validation/two_sensor_v14_metadata_runs/"
    "2026-07-22_ab06_trials_01_08/metadata_audit.json"
)
MARKER_CALIBRATED_MODEL = (
    REPO_ROOT
    / "models/AB06_SEASEA-raw/osimxml/AB06_SEASEA_marker_calibrated.osim"
)
EXPECTED_MARKER_CALIBRATED_MODEL_SHA256 = (
    "98cfcbc4f7155ea4576f583654fbd50a6e8bd2f2f33ff0894c9f3f24dce5fa8d"
)

SCHEMA_VERSION = 1
AUDIT_ID = "AB06_TWO_SENSOR_V14_TRIAL_METADATA_2026-07-22"
TRIAL_IDS = tuple(f"{index:02d}" for index in range(1, 9))
SEMANTIC_CONDITION_TRIAL_IDS = tuple(f"{index:02d}" for index in range(2, 9))

# This is intentionally a complete inventory of the locally available EPIC
# treadmill stream families.  Only ``conditions`` is ever passed to loadmat.
INVENTORY_STREAMS = (
    "conditions",
    "emg",
    "fp",
    "gcLeft",
    "gcRight",
    "gon",
    "id",
    "ik",
    "ik_offset",
    "imu",
    "jp",
    "markers",
)

# These are downstream constraints, not actions taken by this audit.  Keeping
# them here prevents a later development protocol from silently reverting to
# the dataset-provided IK or the historical smoke-test IK path.
DOWNSTREAM_IK_CONTRACT: dict[str, Any] = {
    "opensim_version": "4.5.2",
    "kinematics_source": "marker_based_inverse_kinematics_from_converted_TRC",
    "marker_calibrated_model": {
        "path": (
            "models/AB06_SEASEA-raw/osimxml/"
            "AB06_SEASEA_marker_calibrated.osim"
        ),
        "sha256": EXPECTED_MARKER_CALIBRATED_MODEL_SHA256,
    },
    "marker_count": 28,
    "all_marker_tasks_apply": True,
    "all_marker_task_weights": 1.0,
    "accuracy": 1.0e-5,
    "trial01_consumed_parity_requirement": "exact_numeric_parity",
    "dataset_ik_allowed": False,
    "ik_smoke_allowed": False,
}

DOWNSTREAM_DETECTOR_CONTRACT: dict[str, Any] = {
    "toe_radius_parameter": "reduction_from_V13",
    "toe_radius_reduction_grid_mm": [0.0, 0.05, 0.10, 0.15, 0.20, 0.25],
    "toe_radius_increase_allowed": False,
    "expected_unique_heel_locations": 8,
    "expected_unique_toe_locations": 7,
    "expected_detector_stations": 15,
    "expected_primary_load_spheres": 8,
    "expected_total_sampled_stations": 23,
}

FROZEN_SPLIT: dict[str, tuple[str, ...]] = {
    "CONSUMED": ("01",),
    "DEVELOPMENT": ("02", "04", "08"),
    "VALIDATION": ("05",),
    "SEALED": ("06",),
    "RESERVE": ("03", "07"),
}

_TRIAL_ROLE = {
    trial_id: role
    for role, trial_ids in FROZEN_SPLIT.items()
    for trial_id in trial_ids
}

EXPECTED_COLUMNS = ("Header", "Speed")
EXPECTED_UNITS = ("s", "m/s")
EXPECTED_SAMPLE_DT_S = 0.001
PLATEAU_MIN_DURATION_S = 30.0
NUMERIC_TOLERANCE = 1.0e-9


@dataclass(frozen=True)
class Plateau:
    """One exact non-zero constant-speed run in the conditions table."""

    start_s: float
    end_s: float
    speed_mps: float
    sample_count: int

    @property
    def duration_s(self) -> float:
        return float(self.end_s - self.start_s)

    def to_json(self) -> dict[str, float | int]:
        return {
            "start_s": float(self.start_s),
            "end_s": float(self.end_s),
            "duration_s": self.duration_s,
            "speed_mps": float(self.speed_mps),
            "sample_count": int(self.sample_count),
        }


@dataclass(frozen=True)
class ConditionMetadata:
    """The only semantic values authorized by this metadata audit."""

    trial_id: str
    trial_start_s: float
    trial_end_s: float
    columns: tuple[str, ...]
    units: tuple[str, ...]
    table_sample_count: int
    table_time_start_s: float
    table_time_end_s: float
    table_sample_dt_s: float
    table_time_grid_uniform: bool
    plateaus: tuple[Plateau, ...]

    @property
    def trial_duration_s(self) -> float:
        return float(self.trial_end_s - self.trial_start_s)

    @property
    def chronological_speed_schedule_mps(self) -> tuple[float, ...]:
        return tuple(float(item.speed_mps) for item in self.plateaus)

    @property
    def sorted_speed_levels_mps(self) -> tuple[float, ...]:
        return tuple(sorted(self.chronological_speed_schedule_mps))

    def to_json(self) -> dict[str, Any]:
        return {
            "trial_id": self.trial_id,
            "trial_start_s": float(self.trial_start_s),
            "trial_end_s": float(self.trial_end_s),
            "trial_duration_s": self.trial_duration_s,
            "columns": list(self.columns),
            "units": list(self.units),
            "table_sample_count": int(self.table_sample_count),
            "table_time_start_s": float(self.table_time_start_s),
            "table_time_end_s": float(self.table_time_end_s),
            "table_sample_dt_s": float(self.table_sample_dt_s),
            "table_time_grid_uniform": bool(self.table_time_grid_uniform),
            "chronological_speed_schedule_mps": list(
                self.chronological_speed_schedule_mps
            ),
            "sorted_speed_levels_mps": list(self.sorted_speed_levels_mps),
            "plateaus": [item.to_json() for item in self.plateaus],
        }


def _p(start: float, end: float, speed: float, count: int) -> Plateau:
    return Plateau(start_s=start, end_s=end, speed_mps=speed, sample_count=count)


# These values were obtained from the explicitly authorized conditions-only
# metadata inspection.  They are now executable fail-closed expectations.
FROZEN_CONDITIONS: dict[str, dict[str, Any]] = {
    "02": {
        "trial_interval_s": (9.864, 153.090),
        "sample_count": 158_500,
        "table_time_interval_s": (0.0, 158.499),
        "plateaus": (
            _p(11.698, 44.883, 0.55, 33_186),
            _p(47.550, 79.897, 1.35, 32_348),
            _p(81.231, 114.911, 1.75, 33_681),
            _p(117.578, 149.927, 0.95, 32_350),
        ),
    },
    "03": {
        "trial_interval_s": (8.851, 154.410),
        "sample_count": 156_575,
        "table_time_interval_s": (0.0, 156.574),
        "plateaus": (
            _p(13.011, 46.027, 0.60, 33_017),
            _p(48.694, 81.040, 1.40, 32_347),
            _p(82.374, 116.061, 1.80, 33_688),
            _p(118.728, 151.078, 1.00, 32_351),
        ),
    },
    "04": {
        "trial_interval_s": (12.472, 156.035),
        "sample_count": 159_740,
        "table_time_interval_s": (0.0, 159.739),
        "plateaus": (
            _p(14.639, 47.489, 0.65, 32_851),
            _p(50.156, 82.505, 1.45, 32_350),
            _p(83.839, 117.522, 1.85, 33_684),
            _p(120.189, 152.539, 1.05, 32_351),
        ),
    },
    "05": {
        "trial_interval_s": (15.231, 158.950),
        "sample_count": 161_400,
        "table_time_interval_s": (0.0, 161.399),
        "plateaus": (
            _p(17.565, 50.242, 0.70, 32_678),
            _p(52.909, 85.259, 1.50, 32_351),
            _p(86.593, 120.272, 1.90, 33_680),
            _p(122.939, 155.286, 1.10, 32_348),
        ),
    },
    "06": {
        "trial_interval_s": (11.152, 155.040),
        "sample_count": 163_420,
        "table_time_interval_s": (0.0, 163.419),
        "plateaus": (
            _p(13.652, 46.166, 0.75, 32_515),
            _p(48.833, 81.181, 1.55, 32_349),
            _p(82.515, 116.196, 1.95, 33_682),
            _p(118.863, 151.211, 1.15, 32_349),
        ),
    },
    "07": {
        "trial_interval_s": (9.842, 153.905),
        "sample_count": 155_320,
        "table_time_interval_s": (0.0, 155.319),
        "plateaus": (
            _p(12.509, 44.859, 0.80, 32_351),
            _p(47.526, 79.876, 1.60, 32_351),
            _p(81.210, 114.890, 2.00, 33_681),
            _p(117.557, 149.906, 1.20, 32_350),
        ),
    },
    "08": {
        "trial_interval_s": (10.678, 154.900),
        "sample_count": 159_180,
        "table_time_interval_s": (0.0, 159.179),
        "plateaus": (
            _p(13.512, 45.696, 0.85, 32_185),
            _p(48.363, 80.708, 1.65, 32_346),
            _p(82.042, 115.723, 2.05, 33_682),
            _p(118.390, 150.738, 1.25, 32_349),
        ),
    },
}


class MetadataAuditError(RuntimeError):
    """Raised when the metadata-only access contract cannot be honored."""


def _trial_file(raw_root: Path, stream: str, trial_id: str) -> Path:
    return raw_root / stream / f"treadmill_{trial_id}_01.mat"


def _trial_id_from_path(path: Path) -> str:
    match = re.fullmatch(r"treadmill_(\d{2})_01\.mat", path.name)
    if match is None:
        raise MetadataAuditError(f"unexpected trial filename: {path.name}")
    return match.group(1)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _display_path(path: Path, repo_root: Path) -> str:
    try:
        return path.resolve().relative_to(repo_root.resolve()).as_posix()
    except ValueError:
        return path.resolve().as_posix()


def _identity_record(path: Path, repo_root: Path, access_mode: str) -> dict[str, Any]:
    """Inspect an opaque source through stat and SHA-256 only."""

    record: dict[str, Any] = {
        "path": _display_path(path, repo_root),
        "access_mode": access_mode,
        "exists": path.is_file(),
    }
    if not record["exists"]:
        record.update({"size_bytes": None, "sha256": None})
        return record
    stat = path.stat()
    record.update({"size_bytes": int(stat.st_size), "sha256": _sha256(path)})
    return record


def _scalar_float(value: Any, label: str) -> float:
    array = np.asarray(value, dtype=float)
    if array.size != 1 or not np.isfinite(array.ravel()[0]):
        raise MetadataAuditError(f"{label} is not one finite scalar")
    return float(array.ravel()[0])


def _cell_text(value: Any, label: str) -> str:
    current = value
    while isinstance(current, np.ndarray):
        if current.size != 1:
            raise MetadataAuditError(f"{label} is not one MATLAB cell string")
        current = current.ravel()[0]
    if isinstance(current, bytes):
        return current.decode("utf-8")
    if isinstance(current, (str, np.str_)):
        return str(current)
    raise MetadataAuditError(f"{label} has unsupported string type")


def _cell_texts(value: Any, label: str) -> tuple[str, ...]:
    array = np.asarray(value, dtype=object)
    return tuple(
        _cell_text(item, f"{label}[{index}]")
        for index, item in enumerate(array.ravel())
    )


def _constant_nonzero_plateaus(
    times: np.ndarray,
    speeds: np.ndarray,
    *,
    minimum_duration_s: float = PLATEAU_MIN_DURATION_S,
) -> tuple[Plateau, ...]:
    """Return exact non-zero constant runs lasting at least the frozen minimum."""

    times = np.asarray(times, dtype=float).ravel()
    speeds = np.asarray(speeds, dtype=float).ravel()
    if times.size != speeds.size or times.size < 2:
        raise MetadataAuditError("Header and Speed columns have incompatible sizes")
    if not (np.all(np.isfinite(times)) and np.all(np.isfinite(speeds))):
        raise MetadataAuditError("conditions table contains non-finite values")
    if np.any(np.diff(times) <= 0.0):
        raise MetadataAuditError("conditions Header is not strictly increasing")

    starts = np.concatenate(([0], np.flatnonzero(np.diff(speeds) != 0.0) + 1))
    ends = np.concatenate((starts[1:] - 1, [speeds.size - 1]))
    plateaus: list[Plateau] = []
    for start, end in zip(starts, ends, strict=True):
        duration_s = float(times[end] - times[start])
        speed_mps = float(speeds[start])
        if speed_mps > 0.0 and duration_s >= minimum_duration_s:
            plateaus.append(
                Plateau(
                    start_s=float(times[start]),
                    end_s=float(times[end]),
                    speed_mps=speed_mps,
                    sample_count=int(end - start + 1),
                )
            )
    return tuple(plateaus)


def read_condition_metadata(path: str | Path) -> ConditionMetadata:
    """Decode the narrow conditions allowlist for trials 02--08.

    The parent-directory and trial checks are part of the security boundary:
    this function refuses to parse a similarly shaped MAT file from IK, FP,
    gait-cycle or any other stream.
    """

    condition_path = Path(path).resolve()
    trial_id = _trial_id_from_path(condition_path)
    if condition_path.parent.name != "conditions":
        raise MetadataAuditError("semantic MAT decode is restricted to conditions")
    if trial_id not in SEMANTIC_CONDITION_TRIAL_IDS:
        raise MetadataAuditError(
            f"semantic conditions access is not authorized for trial {trial_id}"
        )

    outer = loadmat(
        condition_path,
        variable_names=["trialStarts", "trialEnds", "__function_workspace__"],
        struct_as_record=False,
        squeeze_me=False,
    )
    required = {"trialStarts", "trialEnds", "__function_workspace__"}
    if not required.issubset(outer):
        missing = sorted(required.difference(outer))
        raise MetadataAuditError(f"conditions MAT is missing variables: {missing}")

    trial_start_s = _scalar_float(outer["trialStarts"], "trialStarts")
    trial_end_s = _scalar_float(outer["trialEnds"], "trialEnds")
    workspace_bytes = np.asarray(
        outer["__function_workspace__"], dtype=np.uint8
    ).ravel().tobytes()
    if len(workspace_bytes) < 9:
        raise MetadataAuditError("MATLAB function workspace is truncated")

    # MATLAB stores the table's MCOS heap inside __function_workspace__.  The
    # embedded payload lacks the ordinary 128-byte MAT v5 header; reconstruct
    # that header in memory without writing or converting any source file.
    header = (
        b"MATLAB 5.0 MAT-file, Platform: PYTHON, Created on:".ljust(116, b" ")
        + bytes(8)
        + workspace_bytes[:4]
    )
    embedded = loadmat(
        BytesIO(header + workspace_bytes[8:]),
        variable_names=["__function_workspace__"],
        struct_as_record=False,
        squeeze_me=False,
    )
    try:
        workspace = embedded["__function_workspace__"][0, 0]
        heap = workspace.MCOS["arr"][0]
        columns = heap[2, 0]
        times = np.asarray(columns[0, 0], dtype=float).ravel()
        speeds = np.asarray(columns[0, 1], dtype=float).ravel()
        names = _cell_texts(heap[7, 0], "VariableNames")
        properties = heap[8, 0][0, 0]
        units = _cell_texts(properties.VariableUnits, "VariableUnits")
    except (AttributeError, IndexError, KeyError, TypeError, ValueError) as exc:
        raise MetadataAuditError("cannot decode frozen MCOS conditions table") from exc

    if times.size != speeds.size or times.size < 2:
        raise MetadataAuditError("conditions table columns have invalid lengths")
    differences = np.diff(times)
    sample_dt_s = float(np.median(differences))
    uniform = bool(
        np.allclose(
            differences,
            EXPECTED_SAMPLE_DT_S,
            rtol=0.0,
            atol=NUMERIC_TOLERANCE,
        )
    )
    plateaus = _constant_nonzero_plateaus(times, speeds)
    return ConditionMetadata(
        trial_id=trial_id,
        trial_start_s=trial_start_s,
        trial_end_s=trial_end_s,
        columns=names,
        units=units,
        table_sample_count=int(times.size),
        table_time_start_s=float(times[0]),
        table_time_end_s=float(times[-1]),
        table_sample_dt_s=sample_dt_s,
        table_time_grid_uniform=uniform,
        plateaus=plateaus,
    )


def _close(left: float, right: float) -> bool:
    return math.isclose(
        float(left), float(right), rel_tol=0.0, abs_tol=NUMERIC_TOLERANCE
    )


def _plateau_matches(left: Plateau, right: Plateau) -> bool:
    return bool(
        _close(left.start_s, right.start_s)
        and _close(left.end_s, right.end_s)
        and _close(left.speed_mps, right.speed_mps)
        and int(left.sample_count) == int(right.sample_count)
    )


def validate_condition_metadata(metadata: ConditionMetadata) -> dict[str, bool]:
    """Apply all frozen per-trial conditions checks."""

    expected = FROZEN_CONDITIONS.get(metadata.trial_id)
    if expected is None:
        raise MetadataAuditError(f"no frozen condition for trial {metadata.trial_id}")
    expected_trial = expected["trial_interval_s"]
    expected_table = expected["table_time_interval_s"]
    expected_plateaus = expected["plateaus"]
    plateau_match = len(metadata.plateaus) == len(expected_plateaus) and all(
        _plateau_matches(observed, frozen)
        for observed, frozen in zip(metadata.plateaus, expected_plateaus, strict=True)
    )
    return {
        "exact_trial_interval": bool(
            _close(metadata.trial_start_s, expected_trial[0])
            and _close(metadata.trial_end_s, expected_trial[1])
        ),
        "exact_columns": metadata.columns == EXPECTED_COLUMNS,
        "exact_units": metadata.units == EXPECTED_UNITS,
        "exact_table_sample_count": int(metadata.table_sample_count)
        == int(expected["sample_count"]),
        "exact_table_time_interval": bool(
            _close(metadata.table_time_start_s, expected_table[0])
            and _close(metadata.table_time_end_s, expected_table[1])
        ),
        "exact_1ms_time_grid": bool(
            metadata.table_time_grid_uniform
            and _close(metadata.table_sample_dt_s, EXPECTED_SAMPLE_DT_S)
        ),
        "exact_plateau_intervals_and_speeds": plateau_match,
    }


def validate_split(split: Mapping[str, Sequence[str]]) -> dict[str, bool]:
    """Check exact roles, disjointness and complete trial coverage."""

    normalized = {role: tuple(values) for role, values in split.items()}
    flattened = [trial for values in normalized.values() for trial in values]
    return {
        "exact_frozen_roles": normalized == FROZEN_SPLIT,
        "roles_are_disjoint": len(flattened) == len(set(flattened)),
        "all_trials_01_08_covered": set(flattened) == set(TRIAL_IDS),
        "single_validation_trial": len(normalized.get("VALIDATION", ())) == 1,
        "single_sealed_trial": len(normalized.get("SEALED", ())) == 1,
    }


def _cross_speed_interpolation_checks(
    conditions: Mapping[str, ConditionMetadata],
) -> tuple[dict[str, bool], list[dict[str, Any]]]:
    """Verify that 05 and 06 are interpolated within DEV at all four strata."""

    needed = ("02", "04", "05", "06", "08")
    if any(trial_id not in conditions for trial_id in needed):
        return {"all_four_strata_interpolated": False}, []
    sorted_levels = {
        trial_id: conditions[trial_id].sorted_speed_levels_mps
        for trial_id in needed
    }
    if any(len(levels) != 4 for levels in sorted_levels.values()):
        return {"all_four_strata_interpolated": False}, []

    rows: list[dict[str, Any]] = []
    all_interpolated = True
    monotonic_by_trial = True
    for index in range(4):
        dev = [sorted_levels[trial_id][index] for trial_id in ("02", "04", "08")]
        validation = sorted_levels["05"][index]
        sealed = sorted_levels["06"][index]
        ordered = bool(dev[0] < dev[1] < validation < sealed < dev[2])
        within_dev_envelope = bool(
            min(dev) < validation < max(dev) and min(dev) < sealed < max(dev)
        )
        all_interpolated = all_interpolated and within_dev_envelope
        monotonic_by_trial = monotonic_by_trial and ordered
        rows.append(
            {
                "speed_stratum_index": index,
                "development_mps": {
                    "02": float(dev[0]),
                    "04": float(dev[1]),
                    "08": float(dev[2]),
                },
                "validation_05_mps": float(validation),
                "sealed_06_mps": float(sealed),
                "validation_and_sealed_inside_development_envelope": (
                    within_dev_envelope
                ),
                "ordered_02_04_05_06_08": ordered,
            }
        )
    return {
        "all_four_strata_interpolated": all_interpolated,
        "ordered_02_04_05_06_08_at_each_stratum": monotonic_by_trial,
        "design_is_cross_speed_interpolation_not_exchangeable_replicates": True,
    }, rows


def _inventory(
    raw_root: Path, repo_root: Path
) -> tuple[dict[str, Any], list[str]]:
    inventory: dict[str, Any] = {}
    missing: list[str] = []
    for trial_id in TRIAL_IDS:
        streams: dict[str, Any] = {}
        for stream in INVENTORY_STREAMS:
            path = _trial_file(raw_root, stream, trial_id)
            semantic = stream == "conditions" and trial_id in (
                SEMANTIC_CONDITION_TRIAL_IDS
            )
            access_mode = (
                "conditions_metadata_semantic_allowlist"
                if semantic
                else "opaque_stat_and_sha256_only"
            )
            if stream in {"ik", "ik_offset"}:
                access_mode += "_forbidden_as_downstream_kinematics"
            record = _identity_record(path, repo_root, access_mode)
            streams[stream] = record
            if not record["exists"]:
                missing.append(record["path"])
        inventory[trial_id] = {
            "role": _TRIAL_ROLE[trial_id],
            "streams": streams,
        }
    return inventory, missing


def _fp_size_diagnostic(inventory: Mapping[str, Any]) -> dict[str, Any]:
    sizes = {
        trial_id: int(inventory[trial_id]["streams"]["fp"]["size_bytes"])
        for trial_id in SEMANTIC_CONDITION_TRIAL_IDS
        if inventory[trial_id]["streams"]["fp"]["size_bytes"] is not None
    }
    if len(sizes) != len(SEMANTIC_CONDITION_TRIAL_IDS):
        return {
            "status": "INCOMPLETE_INVENTORY",
            "sizes_bytes": sizes,
            "outliers_over_1p4x_median": [],
        }
    median = float(np.median(list(sizes.values())))
    ratios = {key: float(value / median) for key, value in sizes.items()}
    outliers = sorted(key for key, ratio in ratios.items() if ratio > 1.4)
    return {
        "status": "DIAGNOSTIC_ONLY_NOT_SEMANTICALLY_INSPECTED",
        "median_size_bytes": median,
        "sizes_bytes": sizes,
        "ratio_to_median": ratios,
        "outliers_over_1p4x_median": outliers,
        "requires_later_role_specific_preflight": bool(outliers),
    }


def build_metadata_audit(
    raw_root: str | Path = DEFAULT_RAW_ROOT,
    *,
    repo_root: str | Path = REPO_ROOT,
    condition_reader: Callable[[str | Path], ConditionMetadata] = (
        read_condition_metadata
    ),
) -> dict[str, Any]:
    """Build the metadata-only audit without writing an output file."""

    root = Path(raw_root).resolve()
    repository = Path(repo_root).resolve()
    inventory, missing = _inventory(root, repository)

    conditions: dict[str, ConditionMetadata] = {}
    condition_checks: dict[str, dict[str, bool]] = {}
    for trial_id in SEMANTIC_CONDITION_TRIAL_IDS:
        path = _trial_file(root, "conditions", trial_id)
        if not path.is_file():
            continue
        metadata = condition_reader(path)
        if metadata.trial_id != trial_id:
            raise MetadataAuditError(
                f"conditions reader returned trial {metadata.trial_id} for {trial_id}"
            )
        conditions[trial_id] = metadata
        condition_checks[trial_id] = validate_condition_metadata(metadata)

    split_checks = validate_split(FROZEN_SPLIT)
    interpolation_checks, interpolation_rows = _cross_speed_interpolation_checks(
        conditions
    )
    all_conditions_present = set(conditions) == set(SEMANTIC_CONDITION_TRIAL_IDS)
    all_conditions_frozen = bool(
        all_conditions_present
        and all(all(checks.values()) for checks in condition_checks.values())
    )
    required_checks = {
        "inventory_complete": not missing,
        "exact_frozen_split": all(split_checks.values()),
        "all_authorized_conditions_decoded": all_conditions_present,
        "all_condition_contracts_match": all_conditions_frozen,
        "cross_speed_interpolation_contract": all(
            interpolation_checks.values()
        ),
        "semantic_access_restricted_to_conditions_02_08": True,
        "no_conversion_or_event_evaluation": True,
    }
    ok = bool(all(required_checks.values()))

    local_speeds = sorted(
        {
            speed
            for metadata in conditions.values()
            for speed in metadata.sorted_speed_levels_mps
        }
    )
    fp_diagnostic = _fp_size_diagnostic(inventory)
    warnings: list[dict[str, Any]] = []
    if fp_diagnostic.get("outliers_over_1p4x_median"):
        warnings.append(
            {
                "code": "OPAQUE_FP_SIZE_OUTLIER",
                "trials": fp_diagnostic["outliers_over_1p4x_median"],
                "meaning": (
                    "File-size identity diagnostic only; FP values were not "
                    "decoded. Recheck the affected stream only under a later "
                    "role-specific protocol if it becomes eligible."
                ),
            }
        )
    if local_speeds and max(local_speeds) > 1.85:
        warnings.append(
            {
                "code": "LOCAL_SPEED_RANGE_REQUIRES_DOCUMENTATION_CROSSCHECK",
                "local_declared_max_speed_mps": float(max(local_speeds)),
                "meaning": (
                    "The local Speed table is authoritative for this split; "
                    "external dataset documentation was not read by this audit."
                ),
            }
        )

    script_path = Path(__file__).resolve()
    test_path = script_path.with_name("test_two_sensor_v14_trial_metadata.py")
    source_identity = {
        "auditor": _identity_record(
            script_path, repository, "python_source_identity_only"
        )
    }
    if test_path.is_file():
        source_identity["tests"] = _identity_record(
            test_path, repository, "python_source_identity_only"
        )

    calibrated_model_identity = _identity_record(
        MARKER_CALIBRATED_MODEL,
        repository,
        "opaque_stat_and_sha256_provenance_check",
    )
    calibrated_model_matches = bool(
        calibrated_model_identity["exists"]
        and calibrated_model_identity["sha256"]
        == EXPECTED_MARKER_CALIBRATED_MODEL_SHA256
    )
    required_checks["marker_calibrated_model_identity"] = calibrated_model_matches
    ok = bool(all(required_checks.values()))

    return {
        "schema_version": SCHEMA_VERSION,
        "audit_id": AUDIT_ID,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "status": "PASS_METADATA_ONLY" if ok else "FAIL_METADATA_AUDIT",
        "ok": ok,
        "objective": (
            "Freeze the AB06 trials 01-08 source inventory, conditions-only "
            "speed schedules and cross-speed role split without decoding any "
            "trajectory, force, gait-event or marker stream."
        ),
        "interpretation": {
            "design": "cross_speed_interpolation",
            "trials_are_exchangeable_replicates": False,
            "leave_one_trial_out_claim_allowed": False,
            "pass_authorizes_event_access": False,
            "next_allowed_action": (
                "freeze_and_test_a_separate_V14_development_protocol"
            ),
        },
        "access_contract": {
            "semantic_allowlist": [
                _display_path(
                    _trial_file(root, "conditions", trial_id), repository
                )
                for trial_id in SEMANTIC_CONDITION_TRIAL_IDS
            ],
            "semantic_values_allowed": [
                "trialStarts",
                "trialEnds",
                "MCOS table Header",
                "MCOS table Speed",
                "MCOS table VariableNames",
                "MCOS table VariableUnits",
            ],
            "all_other_sources": "opaque_stat_and_sha256_only",
            "event_or_trajectory_values_decoded": [],
            "conversion_performed": False,
            "detector_evaluation_performed": False,
        },
        "downstream_requirements_not_executed_by_this_audit": {
            "inverse_kinematics": DOWNSTREAM_IK_CONTRACT,
            "detector_sampling": DOWNSTREAM_DETECTOR_CONTRACT,
            "marker_calibrated_model_identity": calibrated_model_identity,
        },
        "frozen_split": {
            role: list(trial_ids) for role, trial_ids in FROZEN_SPLIT.items()
        },
        "role_state_after_audit": {
            "01": "CONSUMED_PRE_V14_DIAGNOSTIC_ONLY",
            "02": "DEVELOPMENT_METADATA_ONLY_EVENTS_CLOSED",
            "03": "RESERVE_METADATA_ONLY_EVENTS_CLOSED",
            "04": "DEVELOPMENT_METADATA_ONLY_EVENTS_CLOSED",
            "05": "VALIDATION_METADATA_ONLY_ONE_SHOT_CLOSED",
            "06": "SEALED_METADATA_ONLY_ONE_SHOT_CLOSED",
            "07": "RESERVE_METADATA_ONLY_EVENTS_CLOSED",
            "08": "DEVELOPMENT_METADATA_ONLY_EVENTS_CLOSED",
        },
        "checks": {
            "required": required_checks,
            "split": split_checks,
            "conditions_by_trial": condition_checks,
            "cross_speed_interpolation": interpolation_checks,
        },
        "cross_speed_strata": interpolation_rows,
        "condition_metadata": {
            trial_id: metadata.to_json()
            for trial_id, metadata in sorted(conditions.items())
        },
        "inventory": inventory,
        "missing_inventory_paths": missing,
        "opaque_fp_size_diagnostic": fp_diagnostic,
        "warnings": warnings,
        "source_identity": source_identity,
        "runtime": {
            "python": platform.python_version(),
            "numpy": np.__version__,
            "scipy": scipy.__version__,
            "platform": platform.platform(),
        },
        "non_actions": {
            "conditions_01_semantically_decoded": False,
            "ik_semantically_decoded": False,
            "fp_semantically_decoded": False,
            "gcLeft_semantically_decoded": False,
            "gcRight_semantically_decoded": False,
            "markers_semantically_decoded": False,
            "raw_files_converted": False,
            "reference_events_extracted": False,
            "detector_candidates_evaluated": False,
            "development_protocol_created": False,
            "validation_opened": False,
            "sealed_opened": False,
            "reserve_opened": False,
        },
    }


def write_json_no_clobber(path: str | Path, payload: Mapping[str, Any]) -> Path:
    """Create one JSON output while refusing to modify an existing file."""

    output = Path(path).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    serialized = json.dumps(
        payload,
        indent=2,
        sort_keys=True,
        allow_nan=False,
    ) + "\n"
    try:
        with output.open("x", encoding="utf-8") as handle:
            handle.write(serialized)
    except FileExistsError as exc:
        raise MetadataAuditError(f"refusing to overwrite existing output: {output}") from exc
    return output


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Audit AB06 V14 trial metadata without opening event streams."
    )
    parser.add_argument("--raw-root", type=Path, default=DEFAULT_RAW_ROOT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    output = args.output.resolve()
    if output.exists():
        print(
            json.dumps(
                {
                    "status": "ERROR_NO_CLOBBER",
                    "output": output.as_posix(),
                },
                sort_keys=True,
            ),
            file=sys.stderr,
        )
        return 2
    try:
        payload = build_metadata_audit(args.raw_root)
        written = write_json_no_clobber(output, payload)
    except (MetadataAuditError, OSError, ValueError) as exc:
        print(
            json.dumps(
                {"status": "ERROR", "error": str(exc)}, sort_keys=True
            ),
            file=sys.stderr,
        )
        return 2
    print(
        json.dumps(
            {
                "status": payload["status"],
                "ok": payload["ok"],
                "output": written.as_posix(),
            },
            sort_keys=True,
        )
    )
    return 0 if payload["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
