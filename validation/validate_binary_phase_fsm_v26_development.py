"""Validate the single V26 heel-qualified candidate on DEV02/04/08.

The geometry is the frozen V25 force-free two-point profile.  One raw Boolean
ledger is reconstructed at 1 ms from the pinned IK/model inputs, then consumed
both sample-by-sample and in 10 ms batches.  Canonical event ledgers are read
only for scoring; prescribed GRF is never a runtime event source.  Trials
05/06 and reserves 03/07 are outside this executable's allowlist.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
import platform
import sys
import uuid
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
VALIDATION_ROOT = REPO_ROOT / "validation"
for root in (REPO_ROOT, TRAJECTORY_ROOT, VALIDATION_ROOT):
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))

import opensim  # noqa: E402

import sweep_binary_phase_detector_v21_geometry as v21  # noqa: E402
import sweep_binary_phase_detector_v25_geometry as v25  # noqa: E402
import validate_binary_phase_fsm_v20_development as v20  # noqa: E402
from binary_phase_fsm_v26 import (  # noqa: E402
    HeelQualifiedBinaryPhaseFSM,
    V26_EVENT_CONTRACT_ID,
    V26_SOURCE,
)
from config import SimulatorConfig  # noqa: E402
from kinematics_interpolator import KinematicsInterpolator  # noqa: E402
from model_loader import _load_plugin  # noqa: E402


SCHEMA_VERSION = 26
ANALYSIS_ID = "AB06_BINARY_PHASE_FSM_V26_DEV02_04_08"
TRIALS = ("02", "04", "08")
MODES = ("sequential_1ms", "batched_10ms_same_samples")
SAMPLE_DT_S = 0.001
DEBOUNCE_S = 0.005
POLICY_SAMPLES = 10
MIN_ACTOR_STANCE_S = 0.05
MIN_ACTOR_SWING_S = 0.25
PROFILE_PATH = (
    VALIDATION_ROOT
    / "binary_phase_detector_v25_geometry_runs"
    / "2026-08-04_local_reach_sweep_dev02_04_08"
    / "selected_candidate_profile.json"
)
PROFILE_SHA256 = (
    "db704e502b99e49bea6d89493812bafdac748f8ce8d3ce28214ff624078539a2"
)
V25_FREEZE_PATH = (
    VALIDATION_ROOT / "binary_phase_detector_v25_development_candidate_freeze_lock.json"
)
V25_FREEZE_SHA256 = (
    "04ecfe68937bc0d4baa3be9ab9b62060b20eb92c2f218f8540db1cebe423d346"
)
DEFAULT_RECEIPT = (
    VALIDATION_ROOT / "binary_phase_fsm_v26_development_receipt.json"
)


class V26DevelopmentError(RuntimeError):
    """Fail-closed V26 validation error."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _canonical_sha256(value: object) -> str:
    encoded = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _strict_json(path: Path, expected_sha256: str) -> dict[str, Any]:
    if not path.is_file() or _sha256(path) != expected_sha256:
        raise V26DevelopmentError(f"hash-pinned source drifted: {path}")

    def reject(value: str) -> None:
        raise V26DevelopmentError(f"non-finite JSON constant: {value}")

    value = json.loads(path.read_text(encoding="utf-8"), parse_constant=reject)
    if not isinstance(value, dict):
        raise V26DevelopmentError(f"JSON root must be an object: {path}")
    return value


def _platform_preflight() -> dict[str, Any]:
    system = platform.system()
    machine = platform.machine().lower()
    if system != "Darwin" or machine not in {"arm64", "aarch64"}:
        raise V26DevelopmentError(
            "V26 numerical replay is currently attested only on macOS arm64"
        )
    if TRIALS != ("02", "04", "08"):
        raise V26DevelopmentError("development allowlist drifted")
    if set(TRIALS) & {"03", "05", "06", "07"}:
        raise V26DevelopmentError("protected/reserve access is forbidden")
    return {
        "system": system,
        "machine": machine,
        "numerical_claim_scope": "macOS-arm64-only",
        "portable_scope": "FSM, adapter, routing, and synthetic tests",
    }


def _profile_locations() -> dict[str, np.ndarray]:
    profile = _strict_json(PROFILE_PATH, PROFILE_SHA256)
    points = profile.get("points")
    if not isinstance(points, list) or len(points) != 2:
        raise V26DevelopmentError("V25 profile must contain two points")
    locations: dict[str, np.ndarray] = {}
    for point in points:
        if not isinstance(point, Mapping):
            raise V26DevelopmentError("V25 point is malformed")
        name = str(point.get("name", ""))
        location = np.asarray(point.get("location"), dtype=float)
        if name not in {"left_heel", "left_toe"} or location.shape != (3,):
            raise V26DevelopmentError("V25 point identity/shape drifted")
        if not np.all(np.isfinite(location)):
            raise V26DevelopmentError("V25 point is non-finite")
        locations[name] = location
    if set(locations) != {"left_heel", "left_toe"}:
        raise V26DevelopmentError("V25 point roles drifted")
    return locations


def _acquire_trace(
    trial_id: str,
    *,
    locations: Mapping[str, np.ndarray],
    progress: bool,
) -> dict[str, Any]:
    affine = v25._acquire_affine_trial(
        trial_id,
        v21=v21,
        opensim=opensim,
        SimulatorConfig=SimulatorConfig,
        KinematicsInterpolator=KinematicsInterpolator,
        load_plugin=_load_plugin,
        progress_options={
            "width": 36,
            "min_redraw_interval_s": 0.2,
            "non_tty_interval_s": 5.0,
            "enabled": bool(progress),
        },
    )
    channels: dict[str, np.ndarray] = {}
    for name, location in locations.items():
        clearance = (
            affine.normal_coefficients @ np.asarray(location, dtype=float)
            + affine.normal_offset
        )
        if clearance.shape != affine.time_s.shape or not np.all(
            np.isfinite(clearance)
        ):
            raise V26DevelopmentError(
                f"trial {trial_id} produced malformed clearance"
            )
        channels[name] = np.asarray(clearance <= 0.0, dtype=bool)
    packed = np.packbits(
        np.column_stack(
            (channels["left_heel"], channels["left_toe"])
        ).reshape(-1),
        bitorder="little",
    )
    return {
        "time_s": np.asarray(affine.time_s, dtype=float),
        "heel": channels["left_heel"],
        "toe": channels["left_toe"],
        "source": copy.deepcopy(dict(affine.source)),
        "bit_trace_sha256": hashlib.sha256(packed.tobytes()).hexdigest(),
    }


def _sample(time_s: float, heel: bool, toe: bool) -> dict[str, object]:
    return {
        "time_s": float(time_s),
        "left_heel_contact": bool(heel),
        "left_toe_contact": bool(toe),
    }


def _run_mode(trace: Mapping[str, Any], mode: str) -> dict[str, Any]:
    if mode not in MODES:
        raise V26DevelopmentError(f"unknown mode: {mode}")
    times = np.asarray(trace["time_s"], dtype=float)
    heel = np.asarray(trace["heel"], dtype=bool)
    toe = np.asarray(trace["toe"], dtype=bool)
    fsm = HeelQualifiedBinaryPhaseFSM()
    fsm.reset(
        time_s=float(times[0]),
        heel_contact=bool(heel[0]),
        toe_contact=bool(toe[0]),
    )
    all_events: list[dict[str, Any]] = []
    all_transitions: list[dict[str, Any]] = []
    all_cancellations: list[dict[str, Any]] = []
    snapshots: list[dict[str, Any]] = []
    first = 1
    while first < times.size:
        stop = min(first + POLICY_SAMPLES, times.size)
        previous = float(times[first - 1])
        boundary = float(times[stop - 1])
        batch = [
            _sample(times[index], heel[index], toe[index])
            for index in range(first, stop)
        ]
        if mode == "batched_10ms_same_samples":
            payload = fsm.update_policy_step(
                time_s=boundary,
                previous_time_s=previous,
                sensor_samples=batch,
            )
        else:
            batch_events: list[dict[str, Any]] = []
            batch_transitions: list[dict[str, Any]] = []
            batch_cancellations: list[dict[str, Any]] = []
            for entry in batch:
                payload = fsm.update_sample(
                    time_s=float(entry["time_s"]),
                    heel_contact=bool(entry["left_heel_contact"]),
                    toe_contact=bool(entry["left_toe_contact"]),
                    delivered_time_s=boundary,
                )
                batch_events.extend(payload["events_this_step"])
                batch_transitions.extend(
                    payload["contact_state_transitions_this_step"]
                )
                batch_cancellations.extend(
                    payload["candidate_cancellations_this_step"]
                )
            payload = fsm.payload()
            payload["events_this_step"] = batch_events
            payload["contact_state_transitions_this_step"] = batch_transitions
            payload["candidate_cancellations_this_step"] = batch_cancellations
        all_events.extend(payload["events_this_step"])
        all_transitions.extend(payload["contact_state_transitions_this_step"])
        all_cancellations.extend(payload["candidate_cancellations_this_step"])
        snapshots.append(payload)
        first = stop
    result = {
        "events": all_events,
        "contact_state_transitions": all_transitions,
        "candidate_cancellations": all_cancellations,
        "boundary_snapshots_sha256": _canonical_sha256(snapshots),
        "final_payload": fsm.payload(),
        "boundary_count": len(snapshots),
    }
    json.dumps(result, allow_nan=False)
    return result


def _signal_and_actor_gate(
    trace: Mapping[str, Any], events: Sequence[Mapping[str, Any]]
) -> dict[str, Any]:
    times = np.asarray(trace["time_s"], dtype=float)
    heel = np.asarray(trace["heel"], dtype=bool)
    toe = np.asarray(trace["toe"], dtype=bool)
    stance_durations: list[float] = []
    swing_durations: list[float] = []
    last_hs: float | None = None
    last_to: float | None = None
    event_bit_assertions: list[dict[str, Any]] = []
    for event in events:
        name = str(event["event"])
        onset = float(event["event_time_s"])
        index = int(round((onset - float(times[0])) / SAMPLE_DT_S))
        on_grid = 0 <= index < times.size and abs(times[index] - onset) <= 1e-9
        heel_bit = bool(heel[index]) if on_grid else False
        toe_bit = bool(toe[index]) if on_grid else False
        bit_pass = bool(
            on_grid
            and (
                (name == "heel_strike" and heel_bit)
                or (name == "toe_off" and not heel_bit and not toe_bit)
            )
        )
        event_bit_assertions.append(
            {
                "event": name,
                "event_time_s": onset,
                "heel": heel_bit,
                "toe": toe_bit,
                "pass": bit_pass,
            }
        )
        if name == "heel_strike":
            if last_to is not None and (last_hs is None or last_to > last_hs):
                swing_durations.append(onset - last_to)
            last_hs = onset
        else:
            if last_hs is not None and (last_to is None or last_hs > last_to):
                stance_durations.append(onset - last_hs)
            last_to = onset

    toe_only = toe & ~heel
    padded = np.r_[False, toe_only, False]
    starts = np.flatnonzero(padded[1:] & ~padded[:-1])
    ends = np.flatnonzero(~padded[1:] & padded[:-1]) - 1
    toe_only_runs = [
        {
            "start_time_s": float(times[start]),
            "end_time_s": float(times[end]),
            "sample_count": int(end - start + 1),
        }
        for start, end in zip(starts, ends)
    ]
    pending = events[-1] if events else None
    source_pass = all(
        event.get("source") == V26_SOURCE
        and event.get("event_contract_id") == V26_EVENT_CONTRACT_ID
        and abs(
            float(event["confirmed_time_s"])
            - float(event["event_time_s"])
            - DEBOUNCE_S
        )
        <= 1e-9
        for event in events
    )
    stance_pass = bool(
        stance_durations
        and min(stance_durations) >= MIN_ACTOR_STANCE_S - 1e-12
    )
    swing_pass = bool(
        swing_durations
        and min(swing_durations) >= MIN_ACTOR_SWING_S - 1e-12
    )
    gate_pass = bool(
        source_pass
        and all(item["pass"] for item in event_bit_assertions)
        and stance_pass
        and swing_pass
    )
    return {
        "event_source_and_timing_pass": source_pass,
        "event_onset_bits": event_bit_assertions,
        "toe_only_run_count": len(toe_only_runs),
        "toe_only_sample_count": int(np.count_nonzero(toe_only)),
        "maximum_toe_only_run_s": (
            max(item["sample_count"] for item in toe_only_runs) * SAMPLE_DT_S
            if toe_only_runs
            else 0.0
        ),
        "stance_duration_count": len(stance_durations),
        "minimum_stance_duration_s": min(stance_durations),
        "stance_guard_s": MIN_ACTOR_STANCE_S,
        "stance_pass": stance_pass,
        "swing_duration_count": len(swing_durations),
        "minimum_swing_duration_s": min(swing_durations),
        "swing_guard_s": MIN_ACTOR_SWING_S,
        "swing_pass": swing_pass,
        "last_event": None if pending is None else str(pending["event"]),
        "pass": gate_pass,
    }


def validate_development(*, progress: bool = True) -> dict[str, Any]:
    platform_receipt = _platform_preflight()
    _strict_json(V25_FREEZE_PATH, V25_FREEZE_SHA256)
    locations = _profile_locations()
    ledgers = v25._load_oracles()
    if set(ledgers) != set(TRIALS):
        raise V26DevelopmentError("oracle scope drifted")

    units: list[dict[str, Any]] = []
    parity: list[dict[str, Any]] = []
    trace_receipts: dict[str, Any] = {}
    for trial_id in TRIALS:
        trace = _acquire_trace(
            trial_id,
            locations=locations,
            progress=progress,
        )
        mode_results = {mode: _run_mode(trace, mode) for mode in MODES}
        sequential = mode_results[MODES[0]]
        batched = mode_results[MODES[1]]
        parity_record = {
            "trial_id": trial_id,
            "events_exact": sequential["events"] == batched["events"],
            "contact_state_transitions_exact": sequential[
                "contact_state_transitions"
            ]
            == batched["contact_state_transitions"],
            "candidate_cancellations_exact": sequential[
                "candidate_cancellations"
            ]
            == batched["candidate_cancellations"],
            "boundary_snapshots_exact": sequential[
                "boundary_snapshots_sha256"
            ]
            == batched["boundary_snapshots_sha256"],
            "final_payload_exact": sequential["final_payload"]
            == batched["final_payload"],
        }
        parity_record["pass"] = all(parity_record.values())
        parity.append(parity_record)
        signal_gate = _signal_and_actor_gate(trace, sequential["events"])
        for mode in MODES:
            for view in ledgers[trial_id]["scientific_core"]["views"]:
                units.append(
                    v20._score_view(
                        trial_id=trial_id,
                        mode=mode,
                        trace=trace,
                        events=mode_results[mode]["events"],
                        view=view,
                        parity_pass=bool(parity_record["pass"]),
                    )
                )
        trace_receipts[trial_id] = {
            "sample_count": int(np.asarray(trace["time_s"]).size),
            "start_time_s": float(trace["time_s"][0]),
            "end_time_s": float(trace["time_s"][-1]),
            "bit_trace_sha256": trace["bit_trace_sha256"],
            "baseline": {
                "heel": bool(trace["heel"][0]),
                "toe": bool(trace["toe"][0]),
            },
            "event_count": {
                name: sum(
                    event["event"] == name for event in sequential["events"]
                )
                for name in ("heel_strike", "toe_off")
            },
            "source": trace["source"],
            "signal_and_actor_gate": signal_gate,
        }

    all_pass = bool(
        len(units) == 24
        and all(item["pass"] for item in units)
        and len(parity) == 3
        and all(item["pass"] for item in parity)
        and all(
            item["signal_and_actor_gate"]["pass"]
            for item in trace_receipts.values()
        )
    )
    result = {
        "schema_version": SCHEMA_VERSION,
        "analysis_id": ANALYSIS_ID,
        "status": (
            "V26_DEVELOPMENT_READY"
            if all_pass
            else "FAIL_V26_DEVELOPMENT_TERMINAL"
        ),
        "pass": all_pass,
        "candidate": {
            "geometry": "V25 frozen force-free binary points",
            "fsm": "V26 heel-qualified functional contact FSM",
            "source": V26_SOURCE,
            "event_contract_id": V26_EVENT_CONTRACT_ID,
            "sample_dt_s": SAMPLE_DT_S,
            "debounce_s": DEBOUNCE_S,
            "policy_step_s": 0.010,
            "hs_semantics": "first stable heel contact",
            "to_semantics": "first stable AIR",
            "toe_only_swing_semantics": "diagnostic_only_no_gait_event",
            "startup_any_contact_semantics": (
                "partial_stance_without_synthetic_hs"
            ),
            "sweep_performed": False,
            "retuning_performed": False,
        },
        "data_access": {
            "development_trials_opened": list(TRIALS),
            "protected_trials_opened": [],
            "reserve_trials_opened": [],
            "prescribed_grf_runtime_event_source": False,
            "canonical_ledgers_read_for_scoring": list(TRIALS),
        },
        "gate": {
            "unit_count": len(units),
            "unit_pass_count": sum(item["pass"] for item in units),
            "parity_trial_count": len(parity),
            "parity_pass_count": sum(item["pass"] for item in parity),
            "f1_minimum": 0.95,
            "iou_minimum": 0.90,
            "confirmed_hs_tolerance_s": v20.HS_TOLERANCE_S,
            "confirmed_to_tolerance_s": v20.TO_TOLERANCE_S,
            "delivered_hs_tolerance_s": v20.DELIVERED_HS_TOLERANCE_S,
            "delivered_to_tolerance_s": v20.DELIVERED_TO_TOLERANCE_S,
            "minimum_actor_stance_s": MIN_ACTOR_STANCE_S,
            "minimum_actor_swing_s": MIN_ACTOR_SWING_S,
        },
        "platform": platform_receipt,
        "parity": parity,
        "traces": trace_receipts,
        "units": units,
        "sources": {
            "profile": {
                "path": PROFILE_PATH.relative_to(REPO_ROOT).as_posix(),
                "sha256": PROFILE_SHA256,
            },
            "v25_freeze": {
                "path": V25_FREEZE_PATH.relative_to(REPO_ROOT).as_posix(),
                "sha256": V25_FREEZE_SHA256,
            },
            "v20_fsm_preserved": {
                "path": "Trajectory Generator/binary_phase_fsm.py",
                "sha256": _sha256(TRAJECTORY_ROOT / "binary_phase_fsm.py"),
            },
            "v26_fsm": {
                "path": "Trajectory Generator/binary_phase_fsm_v26.py",
                "sha256": _sha256(
                    TRAJECTORY_ROOT / "binary_phase_fsm_v26.py"
                ),
            },
            "validator": {
                "path": Path(__file__).relative_to(REPO_ROOT).as_posix(),
                "sha256": _sha256(Path(__file__)),
            },
            "oracles": {
                trial_id: {
                    "path": str(v25.TRIAL_SPECS[trial_id]["oracle"]),
                    "sha256": str(
                        v25.TRIAL_SPECS[trial_id]["oracle_sha256"]
                    ),
                    "scientific_core_sha256": str(
                        v25.TRIAL_SPECS[trial_id]["oracle_core_sha256"]
                    ),
                }
                for trial_id in TRIALS
            },
        },
        "next_stage": (
            "V26_RUNTIME_AND_V7_REPLAY"
            if all_pass
            else "STOP_NO_FALLBACK_NO_RETUNING"
        ),
    }
    json.dumps(result, allow_nan=False)
    return result


def _atomic_no_clobber_json(path: Path, payload: Mapping[str, Any]) -> Path:
    path = path.resolve()
    if path.exists():
        raise V26DevelopmentError(f"refusing to overwrite receipt: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    try:
        with temporary.open("x", encoding="utf-8", newline="\n") as stream:
            json.dump(dict(payload), stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()
    return path


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write-receipt", action="store_true")
    parser.add_argument("--no-progress", action="store_true")
    args = parser.parse_args(argv)
    try:
        result = validate_development(progress=not args.no_progress)
        if args.write_receipt:
            _atomic_no_clobber_json(DEFAULT_RECEIPT, result)
    except Exception as exc:
        print(
            f"V26 development validation failed closed: "
            f"{type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        return 2
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
