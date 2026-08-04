"""Validate the production two-sensor FSM on saved forward states.

This is the forward-state counterpart of
``validate_two_sensor_prescribed_replay.py``.  It deliberately uses the same
``results/sim_output_states.sto`` source used by the historical single-signal
detector validation, while retaining the prescribed left vertical GRF as the
independent event/phase oracle.

The virtual heel and toe loads are reconstructed separately from the saved
coordinate states and streamed through the real ``ProstheticPhaseFSM`` with
the exact current runtime gates.  No validation-only gait state machine is
implemented here.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import traceback
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
for path in (REPO_ROOT, TRAJECTORY_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from online_grf import load_online_grf_profile  # noqa: E402
from output import _read_storage_table  # noqa: E402
from path_resolver import resolve_repo_path  # noqa: E402
from setup_io import read_setup_xml  # noqa: E402
from validation.validate_online_grf import (  # noqa: E402
    _external_grf,
    _sample_spheres_from_coordinate_states,
)
from validation.validate_online_grf_events import (  # noqa: E402
    match_events,
    strict_event_pass,
)
from validation.validate_two_sensor_prescribed_replay import (  # noqa: E402
    DEFAULT_PROFILE,
    DEFAULT_SEA_PLUGIN,
    DEFAULT_SETUP,
    PHASE_STANCE,
    PHASE_UNKNOWN,
    _current_runtime_fsm_config,
    _current_training_segment_duration_s,
    _events_for_timing_gate,
    _fsm_config_payload,
    _fsm_phase_from_state_id,
    _json_safe,
    _left_sensor_spheres,
    _model_body_weight_n,
    _ordered_event_diagnostic,
    _phase_classification_gate,
    _plot,
    _plot_phase_validation,
    _portable_path,
    _prescribed_prosthetic_kinematics,
    _reference_events_from_prescribed_grf,
    _regional_loads_and_penetrations,
    _run_production_fsm,
    _semantic_gate,
    _source_record,
    _write_phase_samples_csv,
)


DEFAULT_STATES_STO = "results/sim_output_states.sto"
DEFAULT_OUTPUT_DIR = "validation/two_sensor_forward_states_runs/latest"
HISTORICAL_PROFILE_PATH = (
    "online_grf_profiles/"
    "AB06_SEASEA_stiff321_500_pi_preliminary_calibrated.json"
)
HISTORICAL_PROFILE_SHA256 = (
    "61ea948a3c0613e5c0e684a3197de118c7116e36188fca6993da79ce713fd99e"
)
HISTORICAL_RENAME_COMMIT = "bdbf99c11ced6f4ddb7fd5697aab472887894274"
HISTORICAL_FORWARD_VALIDATION_SUMMARY = (
    REPO_ROOT
    / "results/online_grf_event_validation_forward_states_recommended/summary.json"
)


def _runtime_grid_indices(
    native_times: np.ndarray,
    *,
    sample_dt_s: float,
    start_s: float,
    end_s: float,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Select existing state rows nearest to an anchored runtime grid.

    Values are never interpolated.  This keeps the reconstructed virtual
    sensor inputs tied to states that were actually saved by the simulator.
    """
    times = np.asarray(native_times, dtype=float)
    if times.ndim != 1 or times.size < 2 or not np.all(np.isfinite(times)):
        raise ValueError("native state times must be a finite increasing 1-D array")
    if not np.all(np.diff(times) > 0.0):
        raise ValueError("native state times must be strictly increasing")
    if not math.isfinite(sample_dt_s) or sample_dt_s <= 0.0:
        raise ValueError("sample_dt_s must be finite and positive")

    clipped_start = max(float(start_s), float(times[0]))
    clipped_end = min(float(end_s), float(times[-1]))
    if clipped_end <= clipped_start:
        raise ValueError("saved states do not overlap the prescribed setup window")
    interval_count = int(
        math.floor(
            (clipped_end - clipped_start + 1.0e-12) / float(sample_dt_s)
        )
    )
    requested = clipped_start + np.arange(interval_count + 1) * float(sample_dt_s)
    insertion = np.searchsorted(times, requested, side="left")
    upper = np.clip(insertion, 0, times.size - 1)
    lower = np.clip(insertion - 1, 0, times.size - 1)
    choose_lower = np.abs(times[lower] - requested) <= np.abs(
        times[upper] - requested
    )
    indices = np.where(choose_lower, lower, upper).astype(int)
    if np.any(np.diff(indices) <= 0):
        raise ValueError(
            "requested cadence is finer than the saved-state grid or selects "
            "duplicate rows"
        )

    native_dt = float(np.median(np.diff(times)))
    alignment_errors = times[indices] - requested
    maximum_allowed = max(1.0e-9, 0.51 * native_dt)
    maximum_error = float(np.max(np.abs(alignment_errors)))
    if maximum_error > maximum_allowed:
        raise ValueError(
            "saved-state timestamps cannot represent the requested runtime grid: "
            f"max error={maximum_error:.9g} s, allowed={maximum_allowed:.9g} s"
        )
    selected_times = times[indices]
    selected_steps = np.diff(selected_times)
    return indices, {
        "native_sample_count": int(times.size),
        "native_median_dt_s": native_dt,
        "selected_sample_count": int(indices.size),
        "requested_sample_dt_s": float(sample_dt_s),
        "selected_median_dt_s": float(np.median(selected_steps)),
        "maximum_grid_alignment_error_s": maximum_error,
        "selection_method": "nearest existing row; no state interpolation",
        "selected_time_range_s": [
            float(selected_times[0]),
            float(selected_times[-1]),
        ],
    }


def _historical_profile_identity(
    current_profile_source: Mapping[str, Any],
) -> dict[str, Any]:
    """Record the hash-locked identity with the old detector profile.

    Git commit ``bdbf99c...`` renamed the historical profile to the current
    filename with 100% similarity.  The old blob and current file both hash to
    the constant below.  The comparison is fail-closed if the current file is
    edited later.
    """
    current_hash = str(current_profile_source.get("sha256", ""))
    return {
        "historical_profile_path": HISTORICAL_PROFILE_PATH,
        "current_profile_path": current_profile_source.get("path"),
        "historical_blob_sha256": HISTORICAL_PROFILE_SHA256,
        "current_file_sha256": current_hash,
        "byte_identical": current_hash == HISTORICAL_PROFILE_SHA256,
        "provenance": {
            "git_rename_commit": HISTORICAL_RENAME_COMMIT,
            "git_similarity_percent": 100,
            "audit_command": (
                "git show <rename_commit>^:<historical_path> | shasum -a 256"
            ),
            "historical_forward_validation_summary": _source_record(
                HISTORICAL_FORWARD_VALIDATION_SUMMARY
            ),
        },
    }


def _forward_run_status(states_path: Path) -> dict[str, Any]:
    status_path = states_path.with_name("sim_output_run_status.txt")
    source = _source_record(status_path)
    values: dict[str, str] = {}
    for raw_line in status_path.read_text(encoding="utf-8").splitlines():
        key, separator, value = raw_line.partition("=")
        if separator:
            values[key.strip()] = value.strip()
    if values.get("status") != "complete":
        raise ValueError(f"forward run is not complete: {values.get('status')!r}")
    return {"source": source, "values": values}


def _historical_forward_benchmark() -> dict[str, Any]:
    source = _source_record(HISTORICAL_FORWARD_VALIDATION_SUMMARY)
    raw = json.loads(
        HISTORICAL_FORWARD_VALIDATION_SUMMARY.read_text(encoding="utf-8")
    )
    best = raw["best_candidate"]
    left = best["sides"]["left"]
    return {
        "source": source,
        "input_type": raw["input_type"],
        "states_sto": raw["states_sto"],
        "sample_dt_s": float(raw["sample_dt_s"]),
        "detector_parameters": {
            "low_threshold_n": float(best["low_threshold_n"]),
            "confirmation_threshold_n": float(
                best["confirmation_threshold_n"]
            ),
            "min_contact_duration_s": float(best["min_contact_duration_s"]),
            "causal_smoothing_window_s": float(best["smoothing_window_s"]),
        },
        "heel_strike": left["events"]["heel_strike"],
        "toe_off": left["events"]["toe_off"],
        "aggregate_smoothed_contact": left["contact"],
        "comparison_caveat": (
            "The historical contact score thresholds a 100 ms causally smoothed "
            "aggregate load at 15 N. The current metric uses the union of two "
            "separately debounced 5/2 N sensor latches, so this is a comparative "
            "benchmark, not a mathematically identical signal definition."
        ),
    }


def _subset_samples(
    samples: Mapping[str, Mapping[str, np.ndarray]],
    indices: np.ndarray,
) -> dict[str, dict[str, np.ndarray]]:
    selected: dict[str, dict[str, np.ndarray]] = {}
    for group, values in samples.items():
        selected[group] = {}
        for name, raw in values.items():
            array = np.asarray(raw, dtype=float)
            if array.ndim < 1:
                raise ValueError(f"sample array {group}/{name} has no sample axis")
            selected[group][name] = array[indices].copy()
    return selected


def _forward_prosthetic_kinematics(
    columns: Sequence[str],
    data: np.ndarray,
    indices: np.ndarray,
) -> dict[str, np.ndarray]:
    column_index = {name: index for index, name in enumerate(columns)}
    required = {
        "knee_rad": "pros_knee_angle_q",
        "ankle_rad": "pros_ankle_angle_q",
    }
    result: dict[str, np.ndarray] = {}
    for output_name, column_name in required.items():
        if column_name not in column_index:
            raise ValueError(
                f"CoordinateStates file is missing required {column_name!r}"
            )
        values = np.asarray(data[indices, column_index[column_name]], dtype=float)
        if not np.all(np.isfinite(values)):
            raise FloatingPointError(f"non-finite forward {column_name}")
        result[output_name] = values
    return result


def _binary_contact_metrics(
    reference_contact: np.ndarray,
    predicted_contact: np.ndarray,
    mask: np.ndarray,
) -> dict[str, Any]:
    """Return contact confusion counts and P/R/F1/IoU on one sample mask."""
    reference = np.asarray(reference_contact, dtype=bool)
    predicted = np.asarray(predicted_contact, dtype=bool)
    selected = np.asarray(mask, dtype=bool)
    if reference.shape != predicted.shape or reference.shape != selected.shape:
        raise ValueError("contact arrays and mask must have identical shapes")
    ref = reference[selected]
    pred = predicted[selected]
    tp = int(np.count_nonzero(ref & pred))
    fp = int(np.count_nonzero(~ref & pred))
    fn = int(np.count_nonzero(ref & ~pred))
    tn = int(np.count_nonzero(~ref & ~pred))
    precision = tp / max(1, tp + fp)
    recall = tp / max(1, tp + fn)
    f1 = 2.0 * precision * recall / max(1.0e-12, precision + recall)
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


def _contact_reports(
    times: np.ndarray,
    prescribed_vertical_n: np.ndarray,
    replay: Mapping[str, Any],
    *,
    prescribed_threshold_n: float,
    first_complete_hs_s: float,
) -> dict[str, Any]:
    reference_contact = (
        np.asarray(prescribed_vertical_n, dtype=float)
        > float(prescribed_threshold_n)
    )
    sensor_union = (
        np.asarray(replay["heel_contact"], dtype=float) > 0.5
    ) | (np.asarray(replay["toe_contact"], dtype=float) > 0.5)
    fsm_phase = _fsm_phase_from_state_id(np.asarray(replay["state_id"], dtype=float))
    fsm_stance = fsm_phase == PHASE_STANCE
    full_mask = np.ones(reference_contact.shape, dtype=bool)
    strict_mask = np.asarray(times, dtype=float) >= float(first_complete_hs_s) - 1e-12
    return {
        "reference_definition": (
            f"prescribed left Fy > {float(prescribed_threshold_n):g} N"
        ),
        "fsm_stance": {
            "definition": "production FSM state == STANCE_AFTER_HS",
            "unknown_phase_samples_full": int(
                np.count_nonzero(fsm_phase == PHASE_UNKNOWN)
            ),
            "full_window": _binary_contact_metrics(
                reference_contact, fsm_stance, full_mask
            ),
            "strict_complete_event_window": _binary_contact_metrics(
                reference_contact, fsm_stance, strict_mask
            ),
        },
        "debounced_sensor_union": {
            "definition": "stable heel contact OR stable toe contact",
            "full_window": _binary_contact_metrics(
                reference_contact, sensor_union, full_mask
            ),
            "strict_complete_event_window": _binary_contact_metrics(
                reference_contact, sensor_union, strict_mask
            ),
        },
        "strict_window_start_s": float(first_complete_hs_s),
        "role": (
            "diagnostic contact-shape comparison; event timing and settled "
            "phase-state gates remain the fail-closed acceptance criteria"
        ),
    }


def _write_events_csv(
    destination: Path,
    reference_events: Mapping[str, np.ndarray],
    predicted_events: Mapping[str, np.ndarray],
    accepted: Sequence[Mapping[str, Any]],
) -> None:
    confirmed = {
        event: [
            float(item["confirmed_time_s"])
            for item in accepted
            if str(item.get("event", "")) == event
            and not (
                event == "toe_off"
                and float(item.get("segment_valid", 1.0)) == 0.0
            )
        ]
        for event in ("heel_strike", "toe_off")
    }
    rows: list[dict[str, Any]] = []
    for event in ("heel_strike", "toe_off"):
        reference = np.asarray(reference_events[event], dtype=float)
        predicted = np.asarray(predicted_events[event], dtype=float)
        row_count = max(reference.size, predicted.size, len(confirmed[event]))
        for index in range(row_count):
            reference_time = float(reference[index]) if index < reference.size else None
            predicted_time = float(predicted[index]) if index < predicted.size else None
            rows.append(
                {
                    "event": event,
                    "event_index": index,
                    "reference_time_s": reference_time,
                    "detector_onset_time_s": predicted_time,
                    "fsm_confirmation_time_s": (
                        confirmed[event][index]
                        if index < len(confirmed[event])
                        else None
                    ),
                    "onset_error_s": (
                        predicted_time - reference_time
                        if reference_time is not None and predicted_time is not None
                        else None
                    ),
                }
            )
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "event",
                "event_index",
                "reference_time_s",
                "detector_onset_time_s",
                "fsm_confirmation_time_s",
                "onset_error_s",
            ],
        )
        writer.writeheader()
        writer.writerows(rows)


def _write_detector_samples_csv(
    destination: Path,
    times: np.ndarray,
    prescribed_vertical_n: np.ndarray,
    loads: Mapping[str, np.ndarray],
    replay: Mapping[str, Any],
) -> None:
    fsm_phase = _fsm_phase_from_state_id(np.asarray(replay["state_id"], dtype=float))
    table = np.column_stack(
        (
            np.asarray(times, dtype=float),
            np.asarray(prescribed_vertical_n, dtype=float),
            np.asarray(loads["left_heel"], dtype=float),
            np.asarray(loads["left_toe"], dtype=float),
            np.asarray(replay["heel_contact"], dtype=int),
            np.asarray(replay["toe_contact"], dtype=int),
            np.asarray(replay["state_id"], dtype=int),
            fsm_phase,
        )
    )
    destination.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(
        destination,
        table,
        delimiter=",",
        header=(
            "time_s,prescribed_left_fy_n,left_heel_load_n,left_toe_load_n,"
            "heel_stable_contact,toe_stable_contact,fsm_state_id,"
            "fsm_phase_minus1unknown_0swing_1stance"
        ),
        comments="",
        fmt=["%.9f", "%.9f", "%.9f", "%.9f", "%d", "%d", "%d", "%d"],
    )


def run_validation(args: argparse.Namespace) -> dict[str, Any]:
    setup_path = resolve_repo_path(args.setup).resolve()
    profile_path = resolve_repo_path(args.profile).resolve()
    states_path = resolve_repo_path(args.states_sto).resolve()
    output_dir = resolve_repo_path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    setup = read_setup_xml(setup_path)
    profile = load_online_grf_profile(profile_path)
    runtime_fsm_config = _current_runtime_fsm_config()
    runtime_dt_s = _current_training_segment_duration_s()
    matches_runtime_cadence = math.isclose(
        float(args.sample_dt), runtime_dt_s, abs_tol=1e-12
    )
    if not matches_runtime_cadence and not bool(args.sensitivity):
        raise ValueError(
            "a non-runtime cadence is sensitivity-only; pass --sensitivity: "
            f"requested={args.sample_dt}, current={runtime_dt_s}"
        )
    if bool(args.sensitivity) and matches_runtime_cadence:
        raise ValueError(
            "--sensitivity requires a cadence different from current runtime"
        )

    native_times, columns, state_data = _read_storage_table(str(states_path))
    sampled_times, all_samples = _sample_spheres_from_coordinate_states(
        setup,
        profile,
        states_path,
        args.sea_plugin,
    )
    if not np.array_equal(np.asarray(native_times), np.asarray(sampled_times)):
        raise AssertionError("state parser and contact sampler returned different grids")
    indices, grid_contract = _runtime_grid_indices(
        native_times,
        sample_dt_s=float(args.sample_dt),
        start_s=float(setup.t_start),
        end_s=float(setup.t_end),
    )
    times = np.asarray(native_times, dtype=float)[indices]
    samples = _subset_samples(all_samples, indices)
    prosthetic_kinematics = _forward_prosthetic_kinematics(
        columns,
        state_data,
        indices,
    )
    prescribed_kinematics = _prescribed_prosthetic_kinematics(setup, times)
    forward_path_perturbation = {
        "pros_knee_rmse_deg": float(
            np.rad2deg(
                np.sqrt(
                    np.mean(
                        (
                            prosthetic_kinematics["knee_rad"]
                            - prescribed_kinematics["knee_rad"]
                        )
                        ** 2
                    )
                )
            )
        ),
        "pros_ankle_rmse_deg": float(
            np.rad2deg(
                np.sqrt(
                    np.mean(
                        (
                            prosthetic_kinematics["ankle_rad"]
                            - prescribed_kinematics["ankle_rad"]
                        )
                        ** 2
                    )
                )
            )
        ),
        "interpretation": (
            "integration/path perturbation of the same prescribed trial; not "
            "an independent gait or holdout distribution"
        ),
    }

    reference_events, reference_provenance = _reference_events_from_prescribed_grf(
        setup,
        threshold_n=float(args.prescribed_threshold_n),
        min_contact_duration_s=float(args.reference_min_contact_duration_s),
        min_cycle_duration_s=float(args.reference_min_cycle_duration_s),
    )
    loads, penetrations, aggregate = _regional_loads_and_penetrations(
        profile,
        samples,
    )
    replay = _run_production_fsm(
        times,
        loads,
        penetrations,
        aggregate,
        prosthetic_kinematics,
        body_weight_n=_model_body_weight_n(setup.model_file),
        fsm_config=runtime_fsm_config,
    )
    predicted_events = _events_for_timing_gate(replay["accepted"])
    metrics = {
        "heel_strike": match_events(
            reference_events["heel_strike"],
            predicted_events["heel_strike"],
            float(args.hs_tolerance_s),
        ),
        "toe_off": match_events(
            reference_events["toe_off"],
            predicted_events["toe_off"],
            float(args.to_tolerance_s),
        ),
    }
    ordered_event_diagnostics = {
        "heel_strike": _ordered_event_diagnostic(
            reference_events["heel_strike"],
            predicted_events["heel_strike"],
            float(args.hs_tolerance_s),
        ),
        "toe_off": _ordered_event_diagnostic(
            reference_events["toe_off"],
            predicted_events["toe_off"],
            float(args.to_tolerance_s),
        ),
    }
    timing_ok = strict_event_pass(
        metrics,
        hs_tolerance_s=float(args.hs_tolerance_s),
        to_tolerance_s=float(args.to_tolerance_s),
    )
    semantic = _semantic_gate(
        replay,
        times,
        loads,
        sensor_on_threshold_n=float(runtime_fsm_config.sensor_on_threshold_n),
        expected_complete_cycles=int(reference_events["toe_off"].size),
    )
    prescribed_vertical_n = np.asarray(
        _external_grf(setup, times)["left"][:, 1],
        dtype=float,
    )
    phase_validation = _phase_classification_gate(
        times,
        prescribed_vertical_n,
        reference_events,
        predicted_events,
        replay,
        prescribed_threshold_n=float(args.prescribed_threshold_n),
        hs_tolerance_s=float(args.hs_tolerance_s),
        to_tolerance_s=float(args.to_tolerance_s),
        sensor_dwell_s=float(runtime_fsm_config.sensor_dwell_s),
    )
    phase_report = {
        key: value for key, value in phase_validation.items() if key != "_arrays"
    }
    contacts = _contact_reports(
        times,
        prescribed_vertical_n,
        replay,
        prescribed_threshold_n=float(args.prescribed_threshold_n),
        first_complete_hs_s=float(reference_events["heel_strike"][0]),
    )
    historical_benchmark = _historical_forward_benchmark()
    historical_contact = historical_benchmark["aggregate_smoothed_contact"]
    current_sensor_contact = contacts["debounced_sensor_union"]["full_window"]
    historical_contact_benchmark_pass = bool(
        float(current_sensor_contact["f1"]) >= float(historical_contact["f1"])
        and float(current_sensor_contact["iou"])
        >= float(historical_contact["iou"])
    )

    event_plot = output_dir / "two_sensor_forward_states_event_timing.png"
    phase_plot = output_dir / "two_sensor_forward_states_phase_state.png"
    phase_csv = output_dir / "two_sensor_forward_states_phase_samples.csv"
    detector_csv = output_dir / "two_sensor_forward_states_detector_samples.csv"
    events_csv = output_dir / "two_sensor_forward_states_events.csv"
    _plot(
        event_plot,
        times,
        prescribed_vertical_n,
        reference_events,
        predicted_events,
        loads,
        replay,
        prescribed_threshold_n=float(args.prescribed_threshold_n),
        sensor_on_threshold_n=float(runtime_fsm_config.sensor_on_threshold_n),
        sensor_off_threshold_n=float(runtime_fsm_config.sensor_off_threshold_n),
    )
    _plot_phase_validation(
        phase_plot,
        times,
        prescribed_vertical_n,
        phase_validation,
    )
    _write_phase_samples_csv(
        phase_csv,
        times,
        prescribed_vertical_n,
        phase_validation,
        replay,
    )
    _write_detector_samples_csv(
        detector_csv,
        times,
        prescribed_vertical_n,
        loads,
        replay,
    )
    _write_events_csv(
        events_csv,
        reference_events,
        predicted_events,
        replay["accepted"],
    )

    final_payload = replay["fsm"].payload()
    ok = bool(timing_ok and semantic["ok"] and phase_report["ok"])
    profile_source = _source_record(profile_path)
    report = {
        "schema_version": 1,
        "status": "PASS" if ok else "FAIL",
        "ok": ok,
        "objective": (
            "validate separate virtual heel/toe loads reconstructed from the "
            "same saved forward CoordinateStates used by the historical "
            "detector validation, through the production two_sensor FSM"
        ),
        "input_type": "saved_forward_coordinate_states",
        "sources": {
            "setup": _source_record(setup_path),
            "model": _source_record(setup.model_file),
            "forward_states": _source_record(states_path),
            "forward_run_status": _forward_run_status(states_path),
            "detector_profile": profile_source,
            "historical_detector_profile_identity": _historical_profile_identity(
                profile_source
            ),
            "reference": reference_provenance,
        },
        "sampling_contract": {
            **grid_contract,
            "current_training_segment_duration_s": float(runtime_dt_s),
            "matches_current_runtime_cadence": matches_runtime_cadence,
            "validation_role": (
                "primary_runtime_gate"
                if matches_runtime_cadence
                else "native_grid_sensitivity_only"
            ),
        },
        "forward_path_perturbation": forward_path_perturbation,
        "detector_contract": {
            "fsm_class": "prosthetic_phase_fsm.ProstheticPhaseFSM",
            "event_source": "two_sensor",
            "exact_current_training_fsm_config": _fsm_config_payload(
                runtime_fsm_config
            ),
            "sensor_roles": {
                role: sphere.name
                for role, sphere in _left_sensor_spheres(profile).items()
            },
            "kinematics_supplied_to_fsm": (
                "pros_knee_angle_q and pros_ankle_angle_q from saved forward states"
            ),
            "separate_loads_reconstruct_left_aggregate": True,
        },
        "strict_gate": {
            "heel_strike": {
                "required_precision": 1.0,
                "required_recall": 1.0,
                "max_abs_error_s": float(args.hs_tolerance_s),
            },
            "toe_off": {
                "required_precision": 1.0,
                "required_recall": 1.0,
                "max_abs_error_s": float(args.to_tolerance_s),
            },
            "timing_ok": bool(timing_ok),
            "semantic_ok": bool(semantic["ok"]),
            "phase_state_ok": bool(phase_report["ok"]),
        },
        "classification": {
            "strict_detector_gate": "PASS" if ok else "FAIL",
            "historical_contact_signal_parity": (
                "PASS" if historical_contact_benchmark_pass else "FAIL"
            ),
            "historical_contact_parity_affects_strict_detector_gate": False,
            "interpretation": (
                "HS/TO timing, semantics, and production FSM phase are the "
                "acceptance gate. Contact-score parity is reported separately "
                "because the historical and current contact signals are not "
                "mathematically identical."
            ),
        },
        "events": {
            "reference": {
                key: values.tolist() for key, values in reference_events.items()
            },
            "accepted_for_timing_gate": {
                key: values.tolist() for key, values in predicted_events.items()
            },
            "all_accepted_transitions": replay["accepted"],
            "all_sensor_candidates": replay["candidates"],
            "all_sensor_edges": replay["sensor_edges"],
        },
        "metrics": metrics,
        "ordered_event_diagnostics": ordered_event_diagnostics,
        "semantic_gate": semantic,
        "phase_state_validation": phase_report,
        "contact_classification": contacts,
        "historical_single_signal_forward_benchmark": {
            **historical_benchmark,
            "current_debounced_union_full_window": current_sensor_contact,
            "historical_contact_benchmark_pass": (
                historical_contact_benchmark_pass
            ),
        },
        "final_fsm": {
            "state_name": final_payload["state_name"],
            "valid_hs_count": final_payload["valid_hs_count"],
            "valid_to_count": final_payload["valid_to_count"],
            "valid_cycle_count": final_payload["valid_cycle_count"],
            "invalid_event_count": final_payload["invalid_event_count"],
            "timeout_exceeded": final_payload["timeout_exceeded"],
        },
        "load_summary_n": {
            role: {
                "min": float(np.min(values)),
                "median": float(np.median(values)),
                "max": float(np.max(values)),
            }
            for role, values in loads.items()
        },
        "artifacts": {
            "summary_json": _portable_path(output_dir / "summary.json"),
            "event_plot_png": _portable_path(event_plot),
            "phase_plot_png": _portable_path(phase_plot),
            "phase_samples_csv": _portable_path(phase_csv),
            "detector_samples_csv": _portable_path(detector_csv),
            "events_csv": _portable_path(events_csv),
        },
        "notes": [
            (
                "The detector consumes only reconstructed heel/toe loads; the "
                "prescribed GRF is used exclusively as validation oracle."
            ),
            (
                "This saved forward run is not holdout data: it tracks the same "
                "prescribed IK trial and its prescribed GRF is time-driven. The "
                "forceplate channel is separate from the detector loads, but this "
                "replay only checks robustness to forward integration/path "
                "perturbation, not generalization to a new gait."
            ),
            (
                "Contact P/R/F1/IoU are reported both for the confirmed FSM "
                "stance state and for the union of the two debounced contacts."
            ),
            (
                "The 10 ms run is the primary env/FSM gate. A 1 ms invocation "
                "requires --sensitivity and remains a non-primary parity check "
                "against the cadence of the historical validator."
            ),
        ],
    }
    safe_report = _json_safe(report)
    (output_dir / "summary.json").write_text(
        json.dumps(safe_report, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return safe_report


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Validate the production two-sensor FSM on the same saved forward "
            "CoordinateStates used by the historical detector validation."
        )
    )
    parser.add_argument("--setup", default=DEFAULT_SETUP)
    parser.add_argument("--profile", default=DEFAULT_PROFILE)
    parser.add_argument("--sea-plugin", default=DEFAULT_SEA_PLUGIN)
    parser.add_argument("--states-sto", default=DEFAULT_STATES_STO)
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR)
    parser.add_argument(
        "--sample-dt",
        type=float,
        default=_current_training_segment_duration_s(),
        help="Must match the current env/FSM segment duration.",
    )
    parser.add_argument(
        "--sensitivity",
        action="store_true",
        help=(
            "Allow a non-runtime cadence as a separately classified sensitivity "
            "run; it never replaces the primary 10 ms gate."
        ),
    )
    parser.add_argument("--prescribed-threshold-n", type=float, default=20.0)
    parser.add_argument(
        "--reference-min-contact-duration-s",
        type=float,
        default=0.050,
    )
    parser.add_argument(
        "--reference-min-cycle-duration-s",
        type=float,
        default=0.300,
    )
    parser.add_argument("--hs-tolerance-s", type=float, default=0.050)
    parser.add_argument("--to-tolerance-s", type=float, default=0.080)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    try:
        report = run_validation(args)
    except Exception as exc:
        output_dir = resolve_repo_path(args.output_dir).resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        failure = {
            "schema_version": 1,
            "status": "ERROR",
            "ok": False,
            "error": f"{type(exc).__name__}: {exc}",
            "traceback": traceback.format_exc(),
        }
        (output_dir / "summary.json").write_text(
            json.dumps(failure, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(failure, indent=2))
        return 2

    concise = {
        "status": report["status"],
        "heel_strike": report["metrics"]["heel_strike"],
        "toe_off": report["metrics"]["toe_off"],
        "phase_state_ok": report["phase_state_validation"]["ok"],
        "contact_classification": report["contact_classification"],
        "artifacts": report["artifacts"],
    }
    print(json.dumps(concise, indent=2, allow_nan=False))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
