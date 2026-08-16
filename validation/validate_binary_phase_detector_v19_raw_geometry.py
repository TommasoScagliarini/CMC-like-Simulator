"""Replay V19 binary geometry on DEV02/04 without interpreting HS or TO.

This validator reads only the frozen marker-based IK products and the V19
point/plane profile.  It does not read GRF, an event oracle, or the phase FSM.
The output therefore describes raw bit continuity, not event accuracy.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import opensim  # noqa: E402

from binary_phase_detector import (  # noqa: E402
    BINARY_PHASE_ROLES,
    BinaryPhaseDetectorSampler,
    load_binary_phase_detector_profile,
)
from config import SimulatorConfig  # noqa: E402
from kinematics_interpolator import KinematicsInterpolator  # noqa: E402
from model_loader import _load_plugin  # noqa: E402


PROFILE_PATH = (
    REPO_ROOT
    / "validation/experimental_detector_profiles/"
    "two_point_binary_v19_outsole_25mm.json"
)
PROFILE_SHA256 = "fddb17f7bd24e004504de662676d7b5a2cb9e5d0fda77de8dea2664c0b5c7a86"
RUN_ROOT = (
    REPO_ROOT
    / "validation/two_sensor_cross_speed_v14_2_runs/"
    "2026-07-22_ab06_cross_speed_v14_2_boundary_recovery/preprocessed"
)
LOCK_PATHS = {
    trial_id: RUN_ROOT
    / f"trial_{trial_id}/treadmill_{trial_id}_01_preprocessing_lock.json"
    for trial_id in ("02", "04")
}
EXPECTED_INTERVALS_S = {
    "02": (9.875, 153.08),
    "04": (12.485, 156.025),
}
EXPECTED_SAMPLE_COUNTS = {"02": 143206, "04": 143541}
SAMPLE_DT_S = 0.001


class V19RawGeometryError(RuntimeError):
    """Raised when V19 raw geometry cannot be replayed exactly."""


def _reject_json_constant(value: str) -> None:
    raise V19RawGeometryError(f"non-finite JSON constant is forbidden: {value}")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_object(path: Path, *, label: str) -> dict[str, Any]:
    try:
        value = json.loads(
            path.read_text(encoding="utf-8"),
            parse_constant=_reject_json_constant,
        )
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise V19RawGeometryError(f"cannot load {label}: {path}") from exc
    if not isinstance(value, dict):
        raise V19RawGeometryError(f"{label} must be a JSON object")
    return value


def _pinned_repo_file(record: object, *, label: str) -> Path:
    if not isinstance(record, Mapping):
        raise V19RawGeometryError(f"{label} source record is missing")
    raw_path = record.get("path")
    expected = record.get("sha256")
    if (
        not isinstance(raw_path, str)
        or not raw_path
        or "\\" in raw_path
        or Path(raw_path).is_absolute()
    ):
        raise V19RawGeometryError(f"{label} source path is not portable")
    path = (REPO_ROOT / raw_path).resolve()
    try:
        path.relative_to(REPO_ROOT)
    except ValueError as exc:
        raise V19RawGeometryError(f"{label} source escapes repository") from exc
    if not path.is_file():
        raise V19RawGeometryError(f"{label} source is missing: {path}")
    observed = _sha256(path)
    if observed != expected:
        raise V19RawGeometryError(
            f"{label} source hash drifted: {observed} != {expected}"
        )
    return path


def _time_grid(trial_id: str) -> np.ndarray:
    start, end = EXPECTED_INTERVALS_S[trial_id]
    expected_count = EXPECTED_SAMPLE_COUNTS[trial_id]
    times = start + np.arange(expected_count, dtype=float) * SAMPLE_DT_S
    if abs(float(times[-1]) - end) > 1e-10:
        raise V19RawGeometryError(f"trial {trial_id} time grid drifted")
    return times


def _longest_run_s(mask: np.ndarray) -> float:
    values = np.asarray(mask, dtype=bool)
    if values.ndim != 1 or values.size == 0 or not np.any(values):
        return 0.0
    padded = np.r_[False, values, False]
    starts = np.flatnonzero(padded[1:] & ~padded[:-1])
    ends = np.flatnonzero(~padded[1:] & padded[:-1])
    return float(np.max(ends - starts) * SAMPLE_DT_S)


def _mask_summary(mask: np.ndarray) -> dict[str, float | int]:
    values = np.asarray(mask, dtype=bool)
    return {
        "sample_count": int(np.count_nonzero(values)),
        "fraction": float(np.mean(values)),
        "maximum_run_s": _longest_run_s(values),
    }


def _trial_inputs(trial_id: str) -> dict[str, Any]:
    lock_path = LOCK_PATHS[trial_id]
    lock = _load_object(lock_path, label=f"trial {trial_id} preprocessing lock")
    if (
        lock.get("trial_id") != trial_id
        or lock.get("status")
        != "V14_1_PREPROCESSING_FROZEN_BEFORE_DETECTOR_REPLAY"
        or lock.get("analysis_interval_s")
        != list(EXPECTED_INTERVALS_S[trial_id])
        or lock.get("dataset_ik_used_downstream") is not False
    ):
        raise V19RawGeometryError(f"trial {trial_id} preprocessing lock drifted")
    live = lock.get("live_replay_inputs")
    if not isinstance(live, Mapping):
        raise V19RawGeometryError(f"trial {trial_id} live inputs are missing")
    plugin = live.get("plugin")
    if not isinstance(plugin, Mapping):
        raise V19RawGeometryError(f"trial {trial_id} plugin lock is missing")
    binary_path_raw = plugin.get("binary_path")
    binary_sha = plugin.get("binary_sha256")
    loader_basename = plugin.get("loader_basename")
    if not all(isinstance(value, str) and value for value in (
        binary_path_raw,
        binary_sha,
        loader_basename,
    )):
        raise V19RawGeometryError(f"trial {trial_id} plugin lock is malformed")
    binary_path = (REPO_ROOT / str(binary_path_raw)).resolve()
    if not binary_path.is_file() or _sha256(binary_path) != binary_sha:
        raise V19RawGeometryError(f"trial {trial_id} plugin binary drifted")
    return {
        "lock_path": lock_path,
        "lock_sha256": _sha256(lock_path),
        "model_path": _pinned_repo_file(live.get("model"), label="model"),
        "ik_path": _pinned_repo_file(lock.get("ik_motion"), label="IK motion"),
        "plugin_loader": str(loader_basename),
        "plugin_binary_path": binary_path,
        "plugin_binary_sha256": str(binary_sha),
    }


def _sample_trial(
    trial_id: str,
    profile: Any,
) -> tuple[dict[str, Any], dict[str, Any]]:
    inputs = _trial_inputs(trial_id)
    _load_plugin(str(inputs["plugin_loader"]))
    model = opensim.Model(str(inputs["model_path"]))
    state = model.initSystem()
    sampler = BinaryPhaseDetectorSampler(model, profile)

    cfg = SimulatorConfig()
    cfg.model_bundle_dir = str(Path(inputs["model_path"]).parent)
    cfg.model_file = str(inputs["model_path"])
    cfg.kinematics_file = str(inputs["ik_path"])
    cfg.t_start, cfg.t_end = EXPECTED_INTERVALS_S[trial_id]
    kin = KinematicsInterpolator(cfg)
    times = _time_grid(trial_id)
    coordinate_set = model.getCoordinateSet()
    clearances = {
        role: np.empty(times.size, dtype=float) for role in BINARY_PHASE_ROLES
    }
    contacts = {
        role: np.empty(times.size, dtype=bool) for role in BINARY_PHASE_ROLES
    }

    for row, time_s in enumerate(times):
        q, _qdot, _qddot = kin.get(float(time_s))
        state.setTime(float(time_s))
        for index in range(coordinate_set.getSize()):
            coordinate = coordinate_set.get(index)
            name = coordinate.getName()
            if name in q:
                coordinate.setValue(state, float(q[name]), False)
        reading = sampler.sample(state, float(time_s))
        for role in BINARY_PHASE_ROLES:
            clearances[role][row] = float(reading.signed_clearance_m[role])
            bit = reading.contacts[role]
            if type(bit) is not bool:
                raise V19RawGeometryError(
                    f"trial {trial_id} {role} produced a non-boolean bit"
                )
            contacts[role][row] = bit

    if any(not np.all(np.isfinite(values)) for values in clearances.values()):
        raise V19RawGeometryError(f"trial {trial_id} has non-finite clearance")
    heel = contacts["left_heel"]
    toe = contacts["left_toe"]
    masks = {
        "heel_only": heel & ~toe,
        "toe_only": toe & ~heel,
        "both_on": heel & toe,
        "either_on": heel | toe,
        "both_off": ~heel & ~toe,
    }
    disjoint_total = int(
        np.count_nonzero(masks["heel_only"])
        + np.count_nonzero(masks["toe_only"])
        + np.count_nonzero(masks["both_on"])
        + np.count_nonzero(masks["both_off"])
    )
    if disjoint_total != times.size:
        raise V19RawGeometryError(f"trial {trial_id} bit partition is incomplete")
    result = {
        "sample_count": int(times.size),
        "start_time_s": float(times[0]),
        "end_time_s": float(times[-1]),
        "sample_dt_s": SAMPLE_DT_S,
        "channels": {
            role: {
                **_mask_summary(contacts[role]),
                "transition_count": int(
                    np.count_nonzero(contacts[role][1:] != contacts[role][:-1])
                ),
                "minimum_signed_clearance_m": float(np.min(clearances[role])),
                "maximum_signed_clearance_m": float(np.max(clearances[role])),
            }
            for role in BINARY_PHASE_ROLES
        },
        "joint_states": {
            name: _mask_summary(mask) for name, mask in masks.items()
        },
        "disjoint_partition_fraction_sum": float(
            sum(
                np.mean(masks[name])
                for name in ("heel_only", "toe_only", "both_on", "both_off")
            )
        ),
        "nonfinite_clearance_count": 0,
        "nonboolean_bit_count": 0,
    }
    json.dumps(result, allow_nan=False)
    provenance = {
        "preprocessing_lock": {
            "path": Path(inputs["lock_path"]).relative_to(REPO_ROOT).as_posix(),
            "sha256": inputs["lock_sha256"],
        },
        "model": {
            "path": Path(inputs["model_path"]).relative_to(REPO_ROOT).as_posix(),
            "sha256": _sha256(Path(inputs["model_path"])),
        },
        "ik_motion": {
            "path": Path(inputs["ik_path"]).relative_to(REPO_ROOT).as_posix(),
            "sha256": _sha256(Path(inputs["ik_path"])),
        },
        "plugin_binary": {
            "path": Path(inputs["plugin_binary_path"])
            .relative_to(REPO_ROOT)
            .as_posix(),
            "sha256": inputs["plugin_binary_sha256"],
        },
    }
    return result, provenance


def validate_raw_geometry() -> dict[str, Any]:
    if not PROFILE_PATH.is_file() or _sha256(PROFILE_PATH) != PROFILE_SHA256:
        raise V19RawGeometryError("V19 binary profile hash drifted")
    profile = load_binary_phase_detector_profile(PROFILE_PATH)
    trials: dict[str, Any] = {}
    sources: dict[str, Any] = {}
    for trial_id in ("02", "04"):
        trials[trial_id], sources[trial_id] = _sample_trial(trial_id, profile)
    result = {
        "schema_version": 19,
        "status": "PASS_V19_RAW_BINARY_GEOMETRY_ONLY",
        "profile": {
            "path": PROFILE_PATH.relative_to(REPO_ROOT).as_posix(),
            "sha256": PROFILE_SHA256,
        },
        "detector_sample_dt_s": SAMPLE_DT_S,
        "development_trials_sampled": ["02", "04"],
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "grf_read": False,
        "oracle_read": False,
        "fsm_called": False,
        "hs_to_semantics_evaluated": False,
        "events_emitted": False,
        "trials": trials,
        "sources": sources,
    }
    json.dumps(result, allow_nan=False)
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args(argv)
    try:
        result = validate_raw_geometry()
    except Exception as exc:
        print(
            f"V19 raw geometry failed closed: {type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        return 2
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
