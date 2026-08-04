#!/usr/bin/env python3
"""Validate the V9 two-sensor detector when replay starts on heel contact.

The primary experiment derives the first complete left heel strike from the
prescribed vertical GRF and starts a five-second replay exactly at that event.
The V9 heel and forefoot loads are used only as event guards.  Prescribed GRF
supplies both ``normal_force_bw`` and ``in_contact`` to the production
``ProstheticPhaseFSM``.

A short replay at the original setup start is an independent boundary check:
both sensors are already loaded there, so the existing partial-stance
bootstrap must remain active and no synthetic startup HS may be accepted.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import traceback
from dataclasses import asdict, replace
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
for path in (REPO_ROOT, TRAJECTORY_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from online_grf import load_online_grf_profile  # noqa: E402
from model_loader import _load_plugin  # noqa: E402
from path_resolver import resolve_repo_path  # noqa: E402
from prosthetic_phase_fsm import (  # noqa: E402
    STANCE_AFTER_HS,
    ProstheticPhaseFSMConfig,
)
from setup_io import read_setup_xml  # noqa: E402
from validation.validate_online_grf import (  # noqa: E402
    _external_grf,
    _sample_spheres,
)
from validation.validate_two_sensor_prescribed_replay import (  # noqa: E402
    CURRENT_TRAINING_CONFIG,
    DEFAULT_SEA_PLUGIN,
    DEFAULT_SETUP,
    _current_runtime_fsm_config,
    _events_for_timing_gate,
    _left_sensor_spheres,
    _model_body_weight_n,
    _prescribed_prosthetic_kinematics,
    _reference_events_from_prescribed_grf,
    _regional_loads_and_penetrations,
    _run_production_fsm,
)


DEFAULT_DETECTOR_PROFILE = (
    REPO_ROOT
    / "validation"
    / "experimental_detector_profiles"
    / "two_sensor_v9_H2p50_X3p25_F79p0_P35p00.json"
)
DEFAULT_OUTPUT_DIR = (
    REPO_ROOT
    / "validation"
    / "two_sensor_startup_hs_runs"
    / "2026-07-22_ab06_v9_prescribed"
)
V9_CANDIDATE_ID = "H2p50_X3p25_F79p0_P35p00"
SAMPLE_DTS_S = (0.01, 0.001)
CROP_DURATION_S = 5.0
SENSOR_ON_THRESHOLD_N = 0.5
SENSOR_OFF_THRESHOLD_N = 0.25
SENSOR_DWELL_S = 0.03
PRESCRIBED_CONTACT_THRESHOLD_N = 20.0
REFERENCE_MIN_CONTACT_DURATION_S = 0.05
REFERENCE_MIN_CYCLE_DURATION_S = 0.30
EXPECTED_CROP_HS = 4
EXPECTED_CROP_TO = 3
EXPECTED_CROP_CYCLES = 3
HS_TIMING_TOLERANCE_S = 0.050
TO_TIMING_TOLERANCE_S = 0.080
NUMERIC_TOLERANCE = 1.0e-9


def _portable_path(path: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(REPO_ROOT.resolve()).as_posix()
    except ValueError:
        return resolved.as_posix()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _source_record(path: Path) -> dict[str, Any]:
    resolved = path.resolve()
    if not resolved.is_file():
        raise FileNotFoundError(f"validation source not found: {resolved}")
    return {
        "path": _portable_path(resolved),
        "sha256": _sha256(resolved),
        "bytes": int(resolved.stat().st_size),
    }


def _json_safe(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, np.ndarray):
        return [_json_safe(item) for item in value.tolist()]
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    if isinstance(value, (np.integer, int)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        number = float(value)
        return number if math.isfinite(number) else None
    return value


def detector_contract(profile_path: Path) -> dict[str, Any]:
    """Require the exact experimental V9 candidate with two left spheres."""
    raw = json.loads(profile_path.read_text(encoding="utf-8"))
    metadata = raw.get("metadata") if isinstance(raw, Mapping) else None
    if not isinstance(metadata, Mapping):
        raise ValueError("V9 detector profile requires metadata")
    if metadata.get("candidate_id") != V9_CANDIDATE_ID:
        raise ValueError("detector profile is not the selected V9 candidate")

    profile = load_online_grf_profile(profile_path, required_sides=("left",))
    roles = _left_sensor_spheres(profile)
    if len(profile.spheres) != 2 or set(roles) != {"left_heel", "left_toe"}:
        raise ValueError("startup validation requires exactly heel + forefoot")
    return {
        "candidate_id": V9_CANDIDATE_ID,
        "candidate_status": str(metadata.get("status", "")),
        "sphere_count": len(profile.spheres),
        "roles": {role: sphere.name for role, sphere in roles.items()},
        "locations_m": {
            role: [float(value) for value in sphere.location]
            for role, sphere in roles.items()
        },
        "radii_m": {
            role: float(sphere.radius) for role, sphere in roles.items()
        },
        "detector_only": True,
        "generates_grf": False,
    }


def evaluation_times(start_s: float, duration_s: float, dt_s: float) -> np.ndarray:
    """Return an endpoint-inclusive grid without accumulating ``arange`` drift."""
    if not all(math.isfinite(value) for value in (start_s, duration_s, dt_s)):
        raise ValueError("time-grid inputs must be finite")
    if duration_s <= 0.0 or dt_s <= 0.0:
        raise ValueError("duration and dt must be positive")
    intervals_f = duration_s / dt_s
    intervals = int(round(intervals_f))
    if abs(intervals_f - intervals) > 1.0e-10:
        raise ValueError("duration must contain an integer number of samples")
    return float(start_s) + np.arange(intervals + 1, dtype=float) * float(dt_s)


def crop_reference_events(
    reference_events: Mapping[str, np.ndarray],
    *,
    start_s: float,
    end_s: float,
) -> dict[str, np.ndarray]:
    if end_s <= start_s:
        raise ValueError("reference crop end must follow its start")
    result: dict[str, np.ndarray] = {}
    for event in ("heel_strike", "toe_off"):
        values = np.asarray(reference_events[event], dtype=float)
        result[event] = values[
            (values >= float(start_s) - NUMERIC_TOLERANCE)
            & (values <= float(end_s) + NUMERIC_TOLERANCE)
        ]
    return result


def reference_shape(reference_events: Mapping[str, np.ndarray]) -> dict[str, Any]:
    heel = np.asarray(reference_events["heel_strike"], dtype=float)
    toe = np.asarray(reference_events["toe_off"], dtype=float)
    ordered = bool(
        heel.size == toe.size + 1
        and all(
            hs < to < next_hs
            for hs, to, next_hs in zip(heel, toe, heel[1:])
        )
    )
    return {
        "heel_strike_count": int(heel.size),
        "toe_off_count": int(toe.size),
        "complete_cycle_count": int(toe.size if ordered else 0),
        "strict_hs_to_toe_off_to_hs_order": ordered,
        "supports_expected_4_hs_3_to_3_cycles": bool(
            ordered
            and heel.size == EXPECTED_CROP_HS
            and toe.size == EXPECTED_CROP_TO
        ),
    }


def _fsm_config() -> ProstheticPhaseFSMConfig:
    return replace(
        _current_runtime_fsm_config(),
        sensor_on_threshold_n=SENSOR_ON_THRESHOLD_N,
        sensor_off_threshold_n=SENSOR_OFF_THRESHOLD_N,
        sensor_dwell_s=SENSOR_DWELL_S,
    )


def _sample_replay(
    *,
    setup: Any,
    detector_profile: Any,
    times: np.ndarray,
    sea_plugin: str,
    fsm_config: ProstheticPhaseFSMConfig,
    body_weight_n: float,
) -> dict[str, Any]:
    samples = _sample_spheres(setup, detector_profile, times, sea_plugin)
    loads, _detector_penetrations, _detector_aggregate = (
        _regional_loads_and_penetrations(detector_profile, samples)
    )
    prescribed_vertical_n = np.asarray(
        _external_grf(setup, times)["left"][:, 1], dtype=float
    )
    prescribed_contact = (
        prescribed_vertical_n >= PRESCRIBED_CONTACT_THRESHOLD_N
    )
    # _run_production_fsm consumes a two-array penetration union.  Encode the
    # prescribed-GRF contact bit in one slot and an exact zero in the other;
    # detector penetration is deliberately excluded from FSM load evidence.
    contact_evidence = {
        "left_heel": prescribed_contact.astype(float),
        "left_toe": np.zeros(times.shape, dtype=float),
    }
    kinematics = _prescribed_prosthetic_kinematics(setup, times)
    replay = _run_production_fsm(
        times,
        loads,
        contact_evidence,
        np.maximum(prescribed_vertical_n, 0.0),
        kinematics,
        body_weight_n=body_weight_n,
        fsm_config=fsm_config,
    )
    return {
        "times": times,
        "loads": loads,
        "prescribed_vertical_n": prescribed_vertical_n,
        "prescribed_contact": prescribed_contact,
        "replay": replay,
    }


def _unaccepted_gait_candidates(replay: Mapping[str, Any]) -> list[dict[str, Any]]:
    accepted_keys = {
        (str(item.get("event", "")), round(float(item["event_time_s"]), 10))
        for item in replay["accepted"]
        if item.get("event") in {"heel_strike", "toe_off"}
    }
    return [
        dict(item)
        for item in replay["candidates"]
        if item.get("event") in {"heel_strike", "toe_off"}
        and (str(item["event"]), round(float(item["time"]), 10))
        not in accepted_keys
    ]


def _ordered_timing_diagnostic(
    reference: np.ndarray,
    predicted: np.ndarray,
    *,
    tolerance_s: float,
) -> dict[str, Any]:
    reference_arr = np.asarray(reference, dtype=float)
    predicted_arr = np.asarray(predicted, dtype=float)
    equal_counts = bool(reference_arr.size == predicted_arr.size)
    errors = (
        predicted_arr - reference_arr
        if equal_counts
        else np.asarray([], dtype=float)
    )
    return {
        "equal_counts": equal_counts,
        "ordered_errors_s": errors,
        "max_abs_error_s": (
            float(np.max(np.abs(errors))) if errors.size else None
        ),
        "tolerance_s": float(tolerance_s),
        "all_within_tolerance": bool(
            equal_counts
            and errors.size > 0
            and np.all(np.abs(errors) <= float(tolerance_s) + NUMERIC_TOLERANCE)
        ),
        "role": "prescribed_event_timing_gate",
    }


def evaluate_startup_crop(
    sample: Mapping[str, Any],
    reference_events: Mapping[str, np.ndarray],
    *,
    sample_dt_s: float,
) -> dict[str, Any]:
    times = np.asarray(sample["times"], dtype=float)
    loads = sample["loads"]
    replay = sample["replay"]
    predicted = _events_for_timing_gate(replay["accepted"])
    final = replay["fsm"].payload()
    startup_candidates = [
        dict(item)
        for item in replay["candidates"]
        if item.get("event") == "heel_strike"
        and bool(item.get("startup_contact", False))
    ]
    startup_transitions = [
        dict(item)
        for item in replay["accepted"]
        if item.get("event") == "heel_strike"
        and bool(item.get("startup_contact", False))
    ]
    dwell_mask = times <= times[0] + SENSOR_DWELL_S + NUMERIC_TOLERANCE
    heel = np.asarray(loads["left_heel"], dtype=float)
    toe = np.asarray(loads["left_toe"], dtype=float)
    stable_initial_heel_only = bool(
        np.all(heel[dwell_mask] >= SENSOR_ON_THRESHOLD_N)
        and np.all(toe[dwell_mask] <= SENSOR_OFF_THRESHOLD_N)
    )
    expected_shape = reference_shape(reference_events)
    expected_sequence = [
        event
        for pair in zip(
            ["heel_strike"] * EXPECTED_CROP_TO,
            ["toe_off"] * EXPECTED_CROP_TO,
        )
        for event in pair
    ] + ["heel_strike"]
    observed_sequence = [
        str(item.get("event", ""))
        for item in replay["accepted"]
        if item.get("event") in {"heel_strike", "toe_off"}
    ]
    unaccepted = _unaccepted_gait_candidates(replay)
    timing = {
        "heel_strike": _ordered_timing_diagnostic(
            np.asarray(reference_events["heel_strike"], dtype=float),
            predicted["heel_strike"],
            tolerance_s=HS_TIMING_TOLERANCE_S,
        ),
        "toe_off": _ordered_timing_diagnostic(
            np.asarray(reference_events["toe_off"], dtype=float),
            predicted["toe_off"],
            tolerance_s=TO_TIMING_TOLERANCE_S,
        ),
    }

    startup_onset_error = None
    startup_latency = None
    if len(startup_transitions) == 1:
        startup_onset_error = abs(
            float(startup_transitions[0]["event_time_s"])
            - float(reference_events["heel_strike"][0])
        )
        startup_latency = (
            float(startup_transitions[0]["confirmed_time_s"])
            - float(startup_transitions[0]["event_time_s"])
        )

    checks = {
        "oracle_supports_expected_shape": bool(
            expected_shape["supports_expected_4_hs_3_to_3_cycles"]
        ),
        "initial_pattern_is_heel_on_toe_off_for_full_dwell": (
            stable_initial_heel_only
        ),
        "exactly_one_startup_hs_candidate": len(startup_candidates) == 1,
        "exactly_one_startup_hs_accepted": len(startup_transitions) == 1,
        "startup_hs_onset_matches_prescribed_crop_start": bool(
            startup_onset_error is not None
            and startup_onset_error <= NUMERIC_TOLERANCE
        ),
        "startup_hs_confirmation_uses_configured_dwell": bool(
            startup_latency is not None
            and abs(startup_latency - SENSOR_DWELL_S)
            <= max(NUMERIC_TOLERANCE, float(sample_dt_s) * 1.0e-8)
        ),
        "exact_reference_and_detector_counts": bool(
            predicted["heel_strike"].size
            == np.asarray(reference_events["heel_strike"]).size
            and predicted["toe_off"].size
            == np.asarray(reference_events["toe_off"]).size
        ),
        "all_hs_within_50ms": bool(
            timing["heel_strike"]["all_within_tolerance"]
        ),
        "all_toe_off_within_80ms": bool(
            timing["toe_off"]["all_within_tolerance"]
        ),
        "exact_hs_to_toe_off_to_hs_order": observed_sequence
        == expected_sequence,
        "exact_valid_cycles": int(final["valid_cycle_count"])
        == EXPECTED_CROP_CYCLES,
        "zero_invalid_events": int(final["invalid_event_count"]) == 0
        and not replay["invalid_steps"],
        "zero_timeout": float(final["timeout_exceeded"]) == 0.0,
        "zero_unaccepted_gait_candidates": not unaccepted,
        "all_accepted_transitions_match_sensor_candidates": all(
            bool(item.get("matched_sensor_candidate", False))
            for item in replay["accepted"]
            if item.get("event") in {"heel_strike", "toe_off"}
        ),
    }
    timing_check_names = (
        "all_hs_within_50ms",
        "all_toe_off_within_80ms",
    )
    startup_rule_checks = {
        key: value
        for key, value in checks.items()
        if key not in timing_check_names
    }
    timing_checks = {key: checks[key] for key in timing_check_names}
    startup_rule_ok = bool(all(startup_rule_checks.values()))
    timing_ok = bool(
        checks["exact_reference_and_detector_counts"]
        and all(timing_checks.values())
    )
    return {
        "ok": bool(startup_rule_ok and timing_ok),
        "startup_rule_ok": startup_rule_ok,
        "prescribed_timing_ok": timing_ok,
        "sample_dt_s": float(sample_dt_s),
        "time_range_s": [float(times[0]), float(times[-1])],
        "sample_count": int(times.size),
        "checks": checks,
        "reference_shape": expected_shape,
        "reference_events": {
            key: np.asarray(value, dtype=float)
            for key, value in reference_events.items()
        },
        "predicted_events": predicted,
        "ordered_timing_gate": timing,
        "startup": {
            "candidate_count": len(startup_candidates),
            "accepted_count": len(startup_transitions),
            "onset_abs_error_s": startup_onset_error,
            "confirmation_latency_s": startup_latency,
            "candidate": startup_candidates[0] if startup_candidates else None,
            "transition": (
                startup_transitions[0] if startup_transitions else None
            ),
        },
        "initial_detector_loads_n": {
            "heel": float(heel[0]),
            "toe": float(toe[0]),
        },
        "stable_initial_heel_only_through_dwell": stable_initial_heel_only,
        "accepted_gait_sequence": observed_sequence,
        "unaccepted_gait_candidates": unaccepted,
        "final_fsm": {
            key: final[key]
            for key in (
                "state_name",
                "valid_hs_count",
                "valid_to_count",
                "valid_cycle_count",
                "invalid_event_count",
                "timeout_exceeded",
            )
        },
    }


def evaluate_setup_start_bootstrap(
    sample: Mapping[str, Any], *, sample_dt_s: float
) -> dict[str, Any]:
    times = np.asarray(sample["times"], dtype=float)
    loads = sample["loads"]
    replay = sample["replay"]
    final = replay["fsm"].payload()
    heel = np.asarray(loads["left_heel"], dtype=float)
    toe = np.asarray(loads["left_toe"], dtype=float)
    startup_candidates = [
        item
        for item in replay["candidates"]
        if item.get("event") == "heel_strike"
        and bool(item.get("startup_contact", False))
    ]
    startup_transitions = [
        item
        for item in replay["accepted"]
        if item.get("event") == "heel_strike"
        and bool(item.get("startup_contact", False))
    ]
    checks = {
        "initial_pattern_is_heel_on_toe_on": bool(
            heel[0] >= SENSOR_ON_THRESHOLD_N
            and toe[0] >= SENSOR_ON_THRESHOLD_N
        ),
        "partial_stance_bootstrap_active": bool(
            float(final["sensor_partial_stance_active"]) == 1.0
            and int(final["state_id"]) == STANCE_AFTER_HS
        ),
        "zero_startup_hs_candidates": not startup_candidates,
        "zero_startup_hs_transitions": not startup_transitions,
        "zero_valid_hs": int(final["valid_hs_count"]) == 0,
        "zero_invalid_events": int(final["invalid_event_count"]) == 0
        and not replay["invalid_steps"],
    }
    return {
        "ok": bool(all(checks.values())),
        "sample_dt_s": float(sample_dt_s),
        "time_range_s": [float(times[0]), float(times[-1])],
        "sample_count": int(times.size),
        "checks": checks,
        "initial_detector_loads_n": {
            "heel": float(heel[0]),
            "toe": float(toe[0]),
        },
        "final_fsm": {
            key: final[key]
            for key in (
                "state_id",
                "state_name",
                "sensor_partial_stance_active",
                "valid_hs_count",
                "valid_to_count",
                "valid_cycle_count",
                "invalid_event_count",
            )
        },
    }


def _cross_resolution_gate(results: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    runtime = results["runtime_10ms"]["predicted_events"]
    fine = results["fine_1ms"]["predicted_events"]
    event_deltas: dict[str, Any] = {}
    checks: dict[str, bool] = {}
    for event in ("heel_strike", "toe_off"):
        runtime_values = np.asarray(runtime[event], dtype=float)
        fine_values = np.asarray(fine[event], dtype=float)
        equal_counts = bool(runtime_values.size == fine_values.size)
        maximum = (
            float(np.max(np.abs(runtime_values - fine_values)))
            if equal_counts and runtime_values.size
            else None
        )
        event_deltas[event] = {
            "equal_counts": equal_counts,
            "max_abs_timestamp_difference_s": maximum,
        }
        checks[f"{event}_equal_counts"] = equal_counts
        checks[f"{event}_within_runtime_cadence"] = bool(
            maximum is not None
            and maximum <= SAMPLE_DTS_S[0] + NUMERIC_TOLERANCE
        )
    return {
        "ok": bool(all(checks.values())),
        "checks": checks,
        "event_deltas": event_deltas,
    }


def run_validation(output_dir: Path, *, sea_plugin: str) -> dict[str, Any]:
    setup_path = resolve_repo_path(DEFAULT_SETUP).resolve()
    detector_path = DEFAULT_DETECTOR_PROFILE.resolve()
    setup = read_setup_xml(setup_path)
    detector = load_online_grf_profile(detector_path, required_sides=("left",))
    contract = detector_contract(detector_path)
    config = _fsm_config()
    _load_plugin(str(resolve_repo_path(sea_plugin)))
    body_weight_n = _model_body_weight_n(setup.model_file)

    full_reference, reference_provenance = _reference_events_from_prescribed_grf(
        setup,
        threshold_n=PRESCRIBED_CONTACT_THRESHOLD_N,
        min_contact_duration_s=REFERENCE_MIN_CONTACT_DURATION_S,
        min_cycle_duration_s=REFERENCE_MIN_CYCLE_DURATION_S,
    )
    initial_hs_s = float(full_reference["heel_strike"][0])
    crop_end_s = initial_hs_s + CROP_DURATION_S
    crop_reference = crop_reference_events(
        full_reference,
        start_s=initial_hs_s,
        end_s=crop_end_s,
    )
    shape = reference_shape(crop_reference)
    if not shape["supports_expected_4_hs_3_to_3_cycles"]:
        raise ValueError(
            "prescribed oracle does not support the expected five-second crop: "
            f"{shape}"
        )

    startup_results: dict[str, Any] = {}
    bootstrap_results: dict[str, Any] = {}
    for label, sample_dt_s in (
        ("runtime_10ms", SAMPLE_DTS_S[0]),
        ("fine_1ms", SAMPLE_DTS_S[1]),
    ):
        startup_times = evaluation_times(
            initial_hs_s, CROP_DURATION_S, sample_dt_s
        )
        startup_sample = _sample_replay(
            setup=setup,
            detector_profile=detector,
            times=startup_times,
            sea_plugin=sea_plugin,
            fsm_config=config,
            body_weight_n=body_weight_n,
        )
        startup_results[label] = evaluate_startup_crop(
            startup_sample,
            crop_reference,
            sample_dt_s=sample_dt_s,
        )

        bootstrap_times = evaluation_times(
            float(setup.t_start), SENSOR_DWELL_S + sample_dt_s, sample_dt_s
        )
        bootstrap_sample = _sample_replay(
            setup=setup,
            detector_profile=detector,
            times=bootstrap_times,
            sea_plugin=sea_plugin,
            fsm_config=config,
            body_weight_n=body_weight_n,
        )
        bootstrap_results[label] = evaluate_setup_start_bootstrap(
            bootstrap_sample,
            sample_dt_s=sample_dt_s,
        )

    cross_resolution = _cross_resolution_gate(startup_results)
    startup_rule_ok = bool(
        all(result["startup_rule_ok"] for result in startup_results.values())
        and all(result["ok"] for result in bootstrap_results.values())
        and cross_resolution["ok"]
    )
    timing_ok = bool(
        all(result["prescribed_timing_ok"] for result in startup_results.values())
    )
    ok = bool(startup_rule_ok and timing_ok)
    report = {
        "schema_version": 1,
        "status": "PASS" if ok else "FAIL",
        "ok": ok,
        "objective": (
            "validate causal startup HS classification for the exact V9 "
            "heel+forefoot detector without training"
        ),
        "input_type": "prescribed_ik_and_prescribed_vertical_grf",
        "detector_contract": contract,
        "fsm_contract": {
            "class": "prosthetic_phase_fsm.ProstheticPhaseFSM",
            "configuration": asdict(config),
            "event_guards": "V9 heel and forefoot detector loads only",
            "normal_force_bw": "prescribed left vertical GRF / body weight",
            "in_contact": (
                "prescribed left vertical GRF >= 20 N; detector penetration "
                "is excluded"
            ),
            "detector_generates_grf": False,
        },
        "prescribed_oracle": {
            "derivation": reference_provenance,
            "startup_hs_selection": (
                "first complete heel_strike returned by prescribed-GRF parser"
            ),
            "initial_heel_strike_s": initial_hs_s,
            "crop_duration_s": CROP_DURATION_S,
            "crop_end_s": crop_end_s,
            "crop_reference_shape": shape,
            "no_hardcoded_event_timestamp": True,
        },
        "startup_crop": startup_results,
        "original_setup_start_bootstrap": bootstrap_results,
        "cross_resolution": cross_resolution,
        "decision": {
            "startup_rule_gate": {
                "ok": startup_rule_ok,
                "requires_both_cadences": True,
                "includes_original_setup_start_non_regression": True,
            },
            "downstream_prescribed_timing_gate": {
                "ok": timing_ok,
                "heel_strike_tolerance_s": HS_TIMING_TOLERANCE_S,
                "toe_off_tolerance_s": TO_TIMING_TOLERANCE_S,
                "interpretation": (
                    "tests the unchanged V9 geometry/timing on the crop, "
                    "separately from the startup classification rule"
                ),
            },
            "overall_ok": ok,
        },
        "sources": {
            "validator": _source_record(Path(__file__)),
            "production_fsm": _source_record(
                TRAJECTORY_ROOT / "prosthetic_phase_fsm.py"
            ),
            "training_config_for_non_sensor_fsm_gates": _source_record(
                CURRENT_TRAINING_CONFIG
            ),
            "detector_profile": _source_record(detector_path),
            "setup": _source_record(setup_path),
            "model": _source_record(setup.model_file),
            "prescribed_kinematics": _source_record(setup.kinematics_file),
        },
        "non_actions": {
            "training_run": False,
            "policy_or_checkpoint_modified": False,
            "fsm_modified_by_validator": False,
            "detector_profile_modified_or_promoted": False,
            "prescribed_data_modified": False,
        },
        "interpretation": [
            (
                "PASS validates the new episode-boundary classification and "
                "event/cycle completeness; it does not promote the V9 profile."
            ),
            (
                "All ordered events are gated against the established 50 ms "
                "HS and 80 ms TO prescribed tolerances at both cadences."
            ),
        ],
        "artifact": {
            "summary_json": _portable_path(output_dir / "summary.json")
        },
    }
    return _json_safe(report)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument("--sea-plugin", default=DEFAULT_SEA_PLUGIN)
    parser.add_argument("--overwrite", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    output_dir = resolve_repo_path(args.output_dir).resolve()
    summary_path = output_dir / "summary.json"
    if summary_path.exists() and not args.overwrite:
        print(
            json.dumps(
                {
                    "status": "ERROR",
                    "ok": False,
                    "error": f"refusing to overwrite {summary_path}",
                },
                indent=2,
            )
        )
        return 2
    output_dir.mkdir(parents=True, exist_ok=True)
    try:
        report = run_validation(output_dir, sea_plugin=str(args.sea_plugin))
    except Exception as exc:  # pragma: no cover - exercised by integration faults
        failure = {
            "schema_version": 1,
            "status": "ERROR",
            "ok": False,
            "error": f"{type(exc).__name__}: {exc}",
            "traceback": traceback.format_exc(),
        }
        summary_path.write_text(
            json.dumps(failure, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(failure, indent=2))
        return 2

    summary_path.write_text(
        json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(
        json.dumps(
            {
                "status": report["status"],
                "decision": report["decision"],
                "startup_crop": {
                    label: {
                        "ok": result["ok"],
                        "final_fsm": result["final_fsm"],
                    }
                    for label, result in report["startup_crop"].items()
                },
                "original_setup_start_bootstrap": {
                    label: result["ok"]
                    for label, result in report[
                        "original_setup_start_bootstrap"
                    ].items()
                },
                "summary": _portable_path(summary_path),
            },
            indent=2,
        )
    )
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
