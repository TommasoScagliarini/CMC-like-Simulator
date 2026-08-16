"""Validate the frozen V20 binary HS/TO FSM on DEV02/04 only.

The geometry trace is sampled from the frozen V19 two-point profile.  Event
references are read verbatim from the pinned canonical 1 ms ledgers; this file
never thresholds prescribed GRF and never opens trial 08 or protected trials.
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
for import_root in (REPO_ROOT, TRAJECTORY_ROOT, VALIDATION_ROOT):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

import opensim  # noqa: E402

import build_canonical_grf_event_oracle as canonical_oracle  # noqa: E402
import validate_binary_phase_detector_v19_raw_geometry as v19  # noqa: E402
from binary_phase_detector import (  # noqa: E402
    BinaryPhaseDetectorSampler,
    load_binary_phase_detector_profile,
)
from binary_phase_fsm import BinaryPhaseFSM  # noqa: E402
from config import SimulatorConfig  # noqa: E402
from kinematics_interpolator import KinematicsInterpolator  # noqa: E402
from model_loader import _load_plugin  # noqa: E402


SCHEMA_VERSION = 20
ANALYSIS_ID = "AB06_BINARY_PHASE_FSM_V20_DEV02_04_SHADOW"
TRIALS = ("02", "04")
MODES = ("sequential_1ms", "batched_10ms_same_samples")
SAMPLE_DT_S = 0.001
DEBOUNCE_S = 0.005
POLICY_SAMPLES = 10
HS_TOLERANCE_S = 0.050
TO_TOLERANCE_S = 0.080
DELIVERED_HS_TOLERANCE_S = 0.060
DELIVERED_TO_TOLERANCE_S = 0.090
RIGHT_OBSERVATION_MARGIN_S = 0.060
MIN_ACCEPTED_FLIGHT_S = 0.030
PROFILE_PATH = v19.PROFILE_PATH
PROFILE_SHA256 = v19.PROFILE_SHA256
ORACLE_PATHS = {
    "02": VALIDATION_ROOT
    / "canonical_event_oracles/2026-08-03_v17_development/"
    "trial_02_canonical_event_ledger.json",
    "04": VALIDATION_ROOT
    / "canonical_event_oracles/2026-08-03_v17_development/"
    "trial_04_canonical_event_ledger.json",
}
ORACLE_SHA256 = {
    "02": "acfb502bd742055dda49ae9f5398900f87f33368c434e986c98fae127c98894d",
    "04": "4f48813bd8c6bd5117cd52926e9dc921b01296dc548c0d51f3799171d398f813",
}
EXPECTED_VIEW_COUNTS = {
    "02": ((22, 21, 21), (33, 32, 32), (38, 37, 37), (27, 26, 26)),
    "04": ((24, 23, 23), (33, 32, 32), (38, 37, 37), (29, 28, 28)),
}
DEFAULT_RECEIPT = VALIDATION_ROOT / "binary_phase_fsm_v20_development_receipt.json"


class V20DevelopmentError(RuntimeError):
    """Raised when the V20 development evaluation cannot remain fail-closed."""


def _reject_json_constant(value: str) -> None:
    raise V20DevelopmentError(f"non-finite JSON constant is forbidden: {value}")


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


def _strict_json_object(path: Path, *, expected_sha256: str) -> dict[str, Any]:
    if not path.is_file() or _sha256(path) != expected_sha256:
        raise V20DevelopmentError(f"hash-pinned source drifted: {path}")
    try:
        value = json.loads(
            path.read_text(encoding="utf-8"),
            parse_constant=_reject_json_constant,
        )
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise V20DevelopmentError(f"cannot read strict JSON: {path}") from exc
    if not isinstance(value, dict):
        raise V20DevelopmentError(f"JSON root must be an object: {path}")
    return value


def _platform_preflight() -> dict[str, Any]:
    system = platform.system()
    machine = platform.machine().lower()
    if system != "Darwin" or machine not in {"arm64", "aarch64"}:
        raise V20DevelopmentError(
            "numerical V20 replay is currently attested only on macOS arm64"
        )
    if tuple(TRIALS) != ("02", "04"):
        raise V20DevelopmentError("development trial allowlist drifted")
    if "08" in TRIALS or any(item in TRIALS for item in ("05", "06")):
        raise V20DevelopmentError("holdout/protected trial access is forbidden")
    return {
        "system": system,
        "machine": machine,
        "numerical_claim_scope": "macOS-arm64-only",
        "portable_scope": "FSM implementation and synthetic unit tests",
    }


def _load_oracles() -> dict[str, dict[str, Any]]:
    ledgers: dict[str, dict[str, Any]] = {}
    for trial_id in TRIALS:
        ledger = _strict_json_object(
            ORACLE_PATHS[trial_id], expected_sha256=ORACLE_SHA256[trial_id]
        )
        canonical_oracle.validate_ledger(ledger)
        core = ledger.get("scientific_core")
        if not isinstance(core, Mapping) or core.get("trial_id") != trial_id:
            raise V20DevelopmentError(f"oracle trial identity drifted: {trial_id}")
        views = core.get("views")
        if not isinstance(views, list) or len(views) != 4:
            raise V20DevelopmentError(f"trial {trial_id} must expose four views")
        observed_counts = tuple(
            (
                int(view["counts"]["heel_strike"]),
                int(view["counts"]["toe_off"]),
                int(view["counts"]["complete_cycles"]),
            )
            for view in views
        )
        if observed_counts != EXPECTED_VIEW_COUNTS[trial_id]:
            raise V20DevelopmentError(
                f"trial {trial_id} canonical view counts drifted"
            )
        ledgers[trial_id] = ledger
    if set(ledgers) != set(TRIALS):
        raise V20DevelopmentError("oracle access was not exactly DEV02/04")
    return ledgers


def _acquire_binary_trace(trial_id: str, profile: Any) -> dict[str, Any]:
    """Replay the frozen V19 geometry without reading GRF or an oracle."""

    inputs = v19._trial_inputs(trial_id)
    _load_plugin(str(inputs["plugin_loader"]))
    model = opensim.Model(str(inputs["model_path"]))
    state = model.initSystem()
    sampler = BinaryPhaseDetectorSampler(model, profile)

    cfg = SimulatorConfig()
    cfg.model_bundle_dir = str(Path(inputs["model_path"]).parent)
    cfg.model_file = str(inputs["model_path"])
    cfg.kinematics_file = str(inputs["ik_path"])
    cfg.t_start, cfg.t_end = v19.EXPECTED_INTERVALS_S[trial_id]
    kin = KinematicsInterpolator(cfg)
    times = v19._time_grid(trial_id)
    heel = np.empty(times.size, dtype=bool)
    toe = np.empty(times.size, dtype=bool)
    coordinates = model.getCoordinateSet()

    for row, time_s in enumerate(times):
        q, _qdot, _qddot = kin.get(float(time_s))
        state.setTime(float(time_s))
        for index in range(coordinates.getSize()):
            coordinate = coordinates.get(index)
            name = coordinate.getName()
            if name in q:
                coordinate.setValue(state, float(q[name]), False)
        reading = sampler.sample(state, float(time_s))
        heel_bit = reading.contacts["left_heel"]
        toe_bit = reading.contacts["left_toe"]
        if type(heel_bit) is not bool or type(toe_bit) is not bool:
            raise V20DevelopmentError(
                f"trial {trial_id} geometry produced a non-native bool"
            )
        heel[row] = heel_bit
        toe[row] = toe_bit

    if (
        times.size != v19.EXPECTED_SAMPLE_COUNTS[trial_id]
        or not np.all(np.isfinite(times))
        or not np.all(np.diff(times) > 0.0)
    ):
        raise V20DevelopmentError(f"trial {trial_id} trace lattice drifted")
    packed = np.packbits(np.column_stack((heel, toe)).reshape(-1), bitorder="little")
    return {
        "time_s": times,
        "heel": heel,
        "toe": toe,
        "baseline": {
            "time_s": float(times[0]),
            "left_heel_contact": bool(heel[0]),
            "left_toe_contact": bool(toe[0]),
        },
        "bit_trace_sha256": hashlib.sha256(packed.tobytes()).hexdigest(),
        "sample_count": int(times.size),
        "source": {
            "preprocessing_lock": {
                "path": Path(inputs["lock_path"])
                .relative_to(REPO_ROOT)
                .as_posix(),
                "sha256": str(inputs["lock_sha256"]),
            },
            "model": {
                "path": Path(inputs["model_path"])
                .relative_to(REPO_ROOT)
                .as_posix(),
                "sha256": _sha256(Path(inputs["model_path"])),
            },
            "ik_motion": {
                "path": Path(inputs["ik_path"])
                .relative_to(REPO_ROOT)
                .as_posix(),
                "sha256": _sha256(Path(inputs["ik_path"])),
            },
            "plugin_binary": {
                "path": Path(inputs["plugin_binary_path"])
                .relative_to(REPO_ROOT)
                .as_posix(),
                "sha256": str(inputs["plugin_binary_sha256"]),
            },
        },
    }


def _public_sample(time_s: float, heel: bool, toe: bool) -> dict[str, object]:
    return {
        "time_s": float(time_s),
        "left_heel_contact": bool(heel),
        "left_toe_contact": bool(toe),
    }


def _run_mode(trace: Mapping[str, Any], mode: str) -> dict[str, Any]:
    if mode not in MODES:
        raise V20DevelopmentError(f"unknown consumption mode: {mode}")
    times = np.asarray(trace["time_s"], dtype=float)
    heel = np.asarray(trace["heel"], dtype=bool)
    toe = np.asarray(trace["toe"], dtype=bool)
    fsm = BinaryPhaseFSM()
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
        previous_time = float(times[first - 1])
        boundary = float(times[stop - 1])
        raw_samples = [
            _public_sample(times[index], heel[index], toe[index])
            for index in range(first, stop)
        ]
        if mode == "batched_10ms_same_samples":
            payload = fsm.update_policy_step(
                time_s=boundary,
                previous_time_s=previous_time,
                sensor_samples=raw_samples,
            )
        else:
            batch_events: list[dict[str, Any]] = []
            batch_transitions: list[dict[str, Any]] = []
            batch_cancellations: list[dict[str, Any]] = []
            for entry in raw_samples:
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


def _ordered_match_count(
    reference: Sequence[float], predicted: Sequence[float], tolerance: float
) -> int:
    i = 0
    j = 0
    matches = 0
    while i < len(reference) and j < len(predicted):
        if predicted[j] < reference[i] - tolerance:
            j += 1
        elif predicted[j] > reference[i] + tolerance:
            i += 1
        else:
            matches += 1
            i += 1
            j += 1
    return matches


def _owned_events(
    events: Sequence[Mapping[str, Any]],
    view: Mapping[str, Any],
    event_name: str,
) -> list[dict[str, Any]]:
    refs = [
        float(item["event_time_s"])
        for item in view["scoreable_events"]
        if item["event"] == event_name
    ]
    start_s, end_s = (float(value) for value in view["interval_s"])
    tolerance = HS_TOLERANCE_S if event_name == "heel_strike" else TO_TOLERANCE_S
    closing_hs = max(
        float(item["event_time_s"])
        for item in view["scoreable_events"]
        if item["event"] == "heel_strike"
    )
    confirmation_limit = closing_hs + RIGHT_OBSERVATION_MARGIN_S
    primary_start = max(start_s, refs[0] - tolerance)
    primary_end = min(refs[-1] + tolerance, confirmation_limit, end_s - 1e-12)
    return [
        dict(item)
        for item in events
        if item.get("event") == event_name
        and float(item["event_time_s"]) >= start_s - 1e-12
        and primary_start - 1e-12
        <= float(item["confirmed_time_s"])
        <= primary_end + 1e-12
    ]


def _phase_metrics(
    trace: Mapping[str, Any],
    events: Sequence[Mapping[str, Any]],
    view: Mapping[str, Any],
) -> dict[str, Any]:
    oracle_events = list(view["scoreable_events"])
    first_hs = next(
        float(item["event_time_s"])
        for item in oracle_events
        if item["event"] == "heel_strike"
    )
    last_hs = max(
        float(item["event_time_s"])
        for item in oracle_events
        if item["event"] == "heel_strike"
    )
    times = np.asarray(trace["time_s"], dtype=float)
    mask = (times >= first_hs - 1e-12) & (times < last_hs - 1e-12)
    scoring_times = times[mask]
    if scoring_times.size == 0:
        raise V20DevelopmentError("empty phase scoring interval")

    reference = np.zeros(scoring_times.size, dtype=bool)
    contact = False
    oracle_index = 0
    ordered_oracle = sorted(
        oracle_events, key=lambda item: float(item["event_time_s"])
    )
    for index, time_s in enumerate(scoring_times):
        while (
            oracle_index < len(ordered_oracle)
            and float(ordered_oracle[oracle_index]["event_time_s"])
            <= time_s + 1e-12
        ):
            contact = ordered_oracle[oracle_index]["event"] == "heel_strike"
            oracle_index += 1
        reference[index] = contact

    predicted = np.zeros(scoring_times.size, dtype=bool)
    contact = False
    candidate_index = 0
    ordered_candidate = sorted(
        events, key=lambda item: float(item["confirmed_time_s"])
    )
    while (
        candidate_index < len(ordered_candidate)
        and float(ordered_candidate[candidate_index]["confirmed_time_s"])
        < scoring_times[0] - 1e-12
    ):
        contact = ordered_candidate[candidate_index]["event"] == "heel_strike"
        candidate_index += 1
    for index, time_s in enumerate(scoring_times):
        while (
            candidate_index < len(ordered_candidate)
            and float(ordered_candidate[candidate_index]["confirmed_time_s"])
            <= time_s + 1e-12
        ):
            contact = ordered_candidate[candidate_index]["event"] == "heel_strike"
            candidate_index += 1
        predicted[index] = contact

    tp = int(np.count_nonzero(reference & predicted))
    fp = int(np.count_nonzero(~reference & predicted))
    fn = int(np.count_nonzero(reference & ~predicted))
    f1 = 2.0 * tp / max(1, 2 * tp + fp + fn)
    iou = tp / max(1, tp + fp + fn)
    return {
        "sample_count": int(scoring_times.size),
        "true_positive": tp,
        "false_positive": fp,
        "false_negative": fn,
        "f1": float(f1),
        "iou": float(iou),
        "pass": bool(f1 >= 0.95 and iou >= 0.90),
    }


def _score_view(
    *,
    trial_id: str,
    mode: str,
    trace: Mapping[str, Any],
    events: Sequence[Mapping[str, Any]],
    view: Mapping[str, Any],
    parity_pass: bool,
) -> dict[str, Any]:
    event_metrics: dict[str, Any] = {}
    event_gate = True
    predicted_by_name: dict[str, list[dict[str, Any]]] = {}
    for event_name, tolerance, delivered_tolerance in (
        ("heel_strike", HS_TOLERANCE_S, DELIVERED_HS_TOLERANCE_S),
        ("toe_off", TO_TOLERANCE_S, DELIVERED_TO_TOLERANCE_S),
    ):
        reference_times = [
            float(item["event_time_s"])
            for item in view["scoreable_events"]
            if item["event"] == event_name
        ]
        predicted = _owned_events(events, view, event_name)
        predicted_by_name[event_name] = predicted
        confirmed = [float(item["confirmed_time_s"]) for item in predicted]
        delivered = [float(item["delivered_time_s"]) for item in predicted]
        onset = [float(item["event_time_s"]) for item in predicted]
        matched = _ordered_match_count(reference_times, confirmed, tolerance)
        exact_count = len(predicted) == len(reference_times)
        confirmed_errors = (
            [value - expected for value, expected in zip(confirmed, reference_times)]
            if exact_count
            else []
        )
        delivered_errors = (
            [value - expected for value, expected in zip(delivered, reference_times)]
            if exact_count
            else []
        )
        onset_errors = (
            [value - expected for value, expected in zip(onset, reference_times)]
            if exact_count
            else []
        )
        confirmation_delays = [
            float(item["confirmed_time_s"]) - float(item["event_time_s"])
            for item in predicted
        ]
        delivery_delays = [
            float(item["delivered_time_s"]) - float(item["confirmed_time_s"])
            for item in predicted
        ]
        nearest_confirmed_errors = [
            min(confirmed, key=lambda value: abs(value - reference)) - reference
            for reference in reference_times
        ] if confirmed else []
        nearest_onset_errors = [
            min(onset, key=lambda value: abs(value - reference)) - reference
            for reference in reference_times
        ] if onset else []
        precision = matched / max(1, len(predicted))
        recall = matched / max(1, len(reference_times))
        passed = bool(
            exact_count
            and matched == len(reference_times)
            and precision == 1.0
            and recall == 1.0
            and confirmed_errors
            and max(abs(value) for value in confirmed_errors) <= tolerance + 1e-12
            and max(abs(value) for value in delivered_errors)
            <= delivered_tolerance + 1e-12
            and all(
                abs(value - DEBOUNCE_S) <= 1e-9
                for value in confirmation_delays
            )
            and all(-1e-12 <= value <= 0.010 + 1e-12 for value in delivery_delays)
        )
        event_gate = bool(event_gate and passed)
        event_metrics[event_name] = {
            "reference_count": len(reference_times),
            "predicted_count": len(predicted),
            "exact_count": exact_count,
            "matched_within_tolerance": matched,
            "precision": float(precision),
            "recall": float(recall),
            "maximum_absolute_onset_error_s": (
                max(abs(value) for value in onset_errors) if onset_errors else None
            ),
            "nearest_confirmed_error_min_s": (
                min(nearest_confirmed_errors) if nearest_confirmed_errors else None
            ),
            "nearest_confirmed_error_max_s": (
                max(nearest_confirmed_errors) if nearest_confirmed_errors else None
            ),
            "nearest_confirmed_error_median_s": (
                float(np.median(nearest_confirmed_errors))
                if nearest_confirmed_errors
                else None
            ),
            "nearest_onset_error_median_s": (
                float(np.median(nearest_onset_errors))
                if nearest_onset_errors
                else None
            ),
            "maximum_absolute_confirmed_error_s": (
                max(abs(value) for value in confirmed_errors)
                if confirmed_errors
                else None
            ),
            "maximum_absolute_delivered_error_s": (
                max(abs(value) for value in delivered_errors)
                if delivered_errors
                else None
            ),
            "confirmation_delay_min_s": (
                min(confirmation_delays) if confirmation_delays else None
            ),
            "confirmation_delay_max_s": (
                max(confirmation_delays) if confirmation_delays else None
            ),
            "delivery_delay_min_s": min(delivery_delays) if delivery_delays else None,
            "delivery_delay_max_s": max(delivery_delays) if delivery_delays else None,
            "pass": passed,
        }

    merged = sorted(
        [
            *predicted_by_name["heel_strike"],
            *predicted_by_name["toe_off"],
        ],
        key=lambda item: (float(item["confirmed_time_s"]), str(item["event"])),
    )
    expected_order = [item["event"] for item in view["scoreable_events"]]
    observed_order = [item["event"] for item in merged]
    exact_order = observed_order == expected_order
    cycles = sum(
        observed_order[index : index + 3]
        == ["heel_strike", "toe_off", "heel_strike"]
        for index in range(max(0, len(observed_order) - 2))
    )
    expected_cycles = int(view["counts"]["complete_cycles"])
    accepted_flights: list[float] = []
    for index, event in enumerate(merged):
        if event["event"] != "toe_off":
            continue
        following_hs = next(
            (
                future
                for future in merged[index + 1 :]
                if future["event"] == "heel_strike"
            ),
            None,
        )
        if following_hs is not None:
            accepted_flights.append(
                float(following_hs["event_time_s"])
                - float(event["confirmed_time_s"])
            )
    accepted_flight_pass = bool(
        len(accepted_flights) >= expected_cycles
        and min(accepted_flights) >= MIN_ACCEPTED_FLIGHT_S - 1e-12
    )
    phase = _phase_metrics(trace, events, view)
    unit_pass = bool(
        event_gate
        and exact_order
        and cycles == expected_cycles
        and accepted_flight_pass
        and phase["pass"]
        and parity_pass
    )
    return {
        "trial_id": trial_id,
        "view_id": str(view["view_id"]),
        "speed_mps": float(view["speed_mps"]),
        "consumption_mode": mode,
        "events": event_metrics,
        "exact_global_event_order": exact_order,
        "expected_complete_cycle_count": expected_cycles,
        "observed_complete_cycle_count": cycles,
        "minimum_accepted_flight_s": (
            min(accepted_flights) if accepted_flights else None
        ),
        "accepted_flight_pass": accepted_flight_pass,
        "confirmed_phase": phase,
        "scalar_batch_parity_pass": parity_pass,
        "pass": unit_pass,
    }


def validate_development() -> dict[str, Any]:
    platform_receipt = _platform_preflight()
    if not PROFILE_PATH.is_file() or _sha256(PROFILE_PATH) != PROFILE_SHA256:
        raise V20DevelopmentError("V19 binary point profile hash drifted")
    profile = load_binary_phase_detector_profile(PROFILE_PATH)
    ledgers = _load_oracles()

    units: list[dict[str, Any]] = []
    parity: list[dict[str, Any]] = []
    traces: dict[str, Any] = {}
    landing_leaders = {"heel": 0, "toe": 0, "both": 0, "unknown": 0}
    for trial_id in TRIALS:
        trace = _acquire_binary_trace(trial_id, profile)
        mode_results = {mode: _run_mode(trace, mode) for mode in MODES}
        sequential = mode_results[MODES[0]]
        batched = mode_results[MODES[1]]
        parity_record = {
            "trial_id": trial_id,
            "events_exact": sequential["events"] == batched["events"],
            "contact_state_transitions_exact": (
                sequential["contact_state_transitions"]
                == batched["contact_state_transitions"]
            ),
            "candidate_cancellations_exact": (
                sequential["candidate_cancellations"]
                == batched["candidate_cancellations"]
            ),
            "boundary_snapshots_sha256_exact": (
                sequential["boundary_snapshots_sha256"]
                == batched["boundary_snapshots_sha256"]
            ),
            "final_payload_exact": (
                sequential["final_payload"] == batched["final_payload"]
            ),
            "sequential_boundary_snapshots_sha256": sequential[
                "boundary_snapshots_sha256"
            ],
            "batched_boundary_snapshots_sha256": batched[
                "boundary_snapshots_sha256"
            ],
        }
        parity_record["pass"] = bool(
            parity_record["events_exact"]
            and parity_record["contact_state_transitions_exact"]
            and parity_record["candidate_cancellations_exact"]
            and parity_record["boundary_snapshots_sha256_exact"]
            and parity_record["final_payload_exact"]
        )
        parity.append(parity_record)

        for event in sequential["events"]:
            if event["event"] == "heel_strike":
                leader = str(event.get("contact_leader", "unknown"))
                landing_leaders[leader if leader in landing_leaders else "unknown"] += 1
        for mode in MODES:
            for view in ledgers[trial_id]["scientific_core"]["views"]:
                units.append(
                    _score_view(
                        trial_id=trial_id,
                        mode=mode,
                        trace=trace,
                        events=mode_results[mode]["events"],
                        view=view,
                        parity_pass=bool(parity_record["pass"]),
                    )
                )
        traces[trial_id] = {
            "sample_count": trace["sample_count"],
            "start_time_s": float(trace["time_s"][0]),
            "end_time_s": float(trace["time_s"][-1]),
            "bit_trace_sha256": trace["bit_trace_sha256"],
            "baseline": trace["baseline"],
            "source": trace["source"],
            "event_count": {
                name: sum(
                    event["event"] == name for event in sequential["events"]
                )
                for name in ("heel_strike", "toe_off")
            },
            "candidate_cancellation_count": len(
                sequential["candidate_cancellations"]
            ),
        }

    all_pass = bool(
        len(units) == 16
        and all(item["pass"] for item in units)
        and len(parity) == 2
        and all(item["pass"] for item in parity)
    )
    result = {
        "schema_version": SCHEMA_VERSION,
        "analysis_id": ANALYSIS_ID,
        "status": (
            "BINARY_FSM_DEV02_04_SHADOW_READY_TRIAL08_CLOSED"
            if all_pass
            else "FAIL_V20_BINARY_FSM_DEVELOPMENT_TERMINAL_TRIAL08_CLOSED"
        ),
        "pass": all_pass,
        "candidate": {
            "geometry": "V19 two-point force-free binary detector",
            "fsm": "V20 functional AIR/CONTACT FSM",
            "sample_dt_s": SAMPLE_DT_S,
            "debounce_s": DEBOUNCE_S,
            "policy_step_s": 0.010,
            "hs_semantics": "first stable heel-or-toe contact",
            "to_semantics": "first stable both-off",
            "sweep_performed": False,
            "retuning_performed": False,
            "runtime_mode": "binary_shadow",
            "legacy_events_authoritative": True,
        },
        "data_access": {
            "development_trials_opened": list(TRIALS),
            "trial_08_opened": False,
            "protected_trials_opened": [],
            "reserve_trials_opened": [],
            "prescribed_grf_read_by_v20": False,
            "canonical_ledgers_read": list(TRIALS),
        },
        "gate": {
            "unit_count": len(units),
            "unit_pass_count": sum(item["pass"] for item in units),
            "parity_trial_count": len(parity),
            "parity_pass_count": sum(item["pass"] for item in parity),
            "f1_minimum": 0.95,
            "iou_minimum": 0.90,
            "confirmed_hs_tolerance_s": HS_TOLERANCE_S,
            "confirmed_to_tolerance_s": TO_TOLERANCE_S,
            "delivered_hs_tolerance_s": DELIVERED_HS_TOLERANCE_S,
            "delivered_to_tolerance_s": DELIVERED_TO_TOLERANCE_S,
            "minimum_accepted_flight_s": MIN_ACCEPTED_FLIGHT_S,
        },
        "platform": platform_receipt,
        "landing_leader_diagnostics": landing_leaders,
        "parity": parity,
        "units": units,
        "traces": traces,
        "sources": {
            "profile": {
                "path": PROFILE_PATH.relative_to(REPO_ROOT).as_posix(),
                "sha256": PROFILE_SHA256,
            },
            "fsm": {
                "path": "Trajectory Generator/binary_phase_fsm.py",
                "sha256": _sha256(TRAJECTORY_ROOT / "binary_phase_fsm.py"),
            },
            "validator": {
                "path": Path(__file__).relative_to(REPO_ROOT).as_posix(),
                "sha256": _sha256(Path(__file__)),
            },
            "oracles": {
                trial_id: {
                    "path": ORACLE_PATHS[trial_id]
                    .relative_to(REPO_ROOT)
                    .as_posix(),
                    "sha256": ORACLE_SHA256[trial_id],
                }
                for trial_id in TRIALS
            },
        },
        "next_stage": (
            "DISCUSS_BEFORE_ANY_TRIAL08_ACCESS"
            if all_pass
            else "STOP_NO_FALLBACK_NO_RETUNING"
        ),
    }
    json.dumps(result, allow_nan=False)
    return result


def _atomic_no_clobber_json(path: Path, payload: Mapping[str, Any]) -> Path:
    path = path.resolve()
    if path.exists():
        raise V20DevelopmentError(f"refusing to overwrite receipt: {path}")
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
    parser.add_argument(
        "--write-receipt",
        action="store_true",
        help=f"write the no-clobber receipt to {DEFAULT_RECEIPT}",
    )
    args = parser.parse_args(argv)
    try:
        result = validate_development()
        if args.write_receipt:
            _atomic_no_clobber_json(DEFAULT_RECEIPT, result)
    except Exception as exc:
        print(
            f"V20 development validation failed closed: "
            f"{type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        return 2
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
