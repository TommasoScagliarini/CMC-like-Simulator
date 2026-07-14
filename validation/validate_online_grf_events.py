"""Validate online GRF as a prosthetic gait-event signal.

This script intentionally gates only event shape/timing. Vertical-force
magnitude, impulse, COP, and moments are reported as diagnostics elsewhere; they
do not decide PASS/FAIL here.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any, Iterable

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from online_grf import StreamingGaitEventDetector, load_online_grf_profile
from path_resolver import resolve_repo_path
from setup_io import read_setup_xml

DEFAULT_SETUP = "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml"
DEFAULT_PROFILE = (
    "online_grf_profiles/"
    "AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json"
)
DEFAULT_SEA_PLUGIN = "plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff"


def _float_list(text: str) -> list[float]:
    values = [float(item.strip()) for item in str(text).split(",") if item.strip()]
    if not values:
        raise ValueError("expected at least one comma-separated float")
    return values


def _threshold_grid(minimum: float, maximum: float, step: float) -> np.ndarray:
    if step <= 0.0:
        raise ValueError("threshold step must be positive")
    if maximum < minimum:
        raise ValueError("threshold maximum must be >= minimum")
    return np.arange(minimum, maximum + step * 0.25, step, dtype=float)


def _causal_moving_average(values: np.ndarray, window_samples: int) -> np.ndarray:
    values = np.asarray(values, dtype=float)
    if window_samples <= 1:
        return values.copy()
    cumsum = np.cumsum(np.insert(values, 0, 0.0))
    out = np.empty_like(values)
    for index in range(values.size):
        start = max(0, index + 1 - window_samples)
        count = index + 1 - start
        out[index] = (cumsum[index + 1] - cumsum[start]) / count
    return out


def smooth_force(
    times: np.ndarray,
    vertical_force: np.ndarray,
    window_s: float,
) -> np.ndarray:
    """Causal smoothing so validation does not use future samples."""
    if window_s <= 0.0 or len(times) < 2:
        return np.asarray(vertical_force, dtype=float).copy()
    dt = float(np.median(np.diff(times)))
    window_samples = max(1, int(round(float(window_s) / max(dt, 1.0e-12))))
    return _causal_moving_average(np.asarray(vertical_force, dtype=float), window_samples)


def detect_events(
    times: np.ndarray,
    vertical_force: np.ndarray,
    *,
    threshold_n: float,
    confirmation_threshold_n: float | None,
    min_contact_duration_s: float,
    min_cycle_duration_s: float,
) -> dict[str, np.ndarray]:
    """Run the same streaming event logic used by the simulator."""
    detector = StreamingGaitEventDetector(
        threshold_n,
        min_contact_duration_s,
        min_cycle_duration_s,
        confirmation_threshold_n,
    )
    events: list[dict[str, Any]] = []
    for time, force in zip(times, vertical_force):
        events.extend(
            detector.update(float(time), {"left": float(force), "right": 0.0})
        )
    hs = [
        float(event["time"])
        for event in events
        if event.get("side") == "left" and event.get("event") == "heel_strike"
    ]
    hs_confirmed = [
        float(event.get("confirmed_time", event["time"]))
        for event in events
        if event.get("side") == "left" and event.get("event") == "heel_strike"
    ]
    to = [
        float(event["time"])
        for event in events
        if event.get("side") == "left" and event.get("event") == "toe_off"
    ]
    return {
        "heel_strikes": np.asarray(hs, dtype=float),
        "heel_strike_confirmed_times": np.asarray(hs_confirmed, dtype=float),
        "toe_offs": np.asarray(to, dtype=float),
    }


def _event_array(events: dict[str, np.ndarray], event_name: str) -> np.ndarray:
    if event_name == "heel_strike":
        return np.asarray(events.get("heel_strikes", []), dtype=float)
    if event_name == "toe_off":
        return np.asarray(events.get("toe_offs", []), dtype=float)
    raise ValueError(f"unsupported event name: {event_name}")


def match_events(
    reference: Iterable[float],
    predicted: Iterable[float],
    tolerance_s: float,
) -> dict[str, Any]:
    """Greedy one-to-one event matching within a tolerance window."""
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
    f1 = 2.0 * precision * recall / max(1.0e-12, precision + recall)
    unmatched_reference = [
        float(ref[index]) for index in range(len(ref)) if index not in matched_ref
    ]
    unmatched_predicted = [
        float(pred[index]) for index in range(len(pred)) if index not in matched_pred
    ]
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
        "unmatched_reference": unmatched_reference,
        "unmatched_predicted": unmatched_predicted,
        "pairs": pairs,
    }


def event_metrics(
    reference_events: dict[str, np.ndarray],
    predicted_events: dict[str, np.ndarray],
    *,
    hs_tolerance_s: float,
    to_tolerance_s: float,
) -> dict[str, dict[str, Any]]:
    return {
        "heel_strike": match_events(
            _event_array(reference_events, "heel_strike"),
            _event_array(predicted_events, "heel_strike"),
            hs_tolerance_s,
        ),
        "toe_off": match_events(
            _event_array(reference_events, "toe_off"),
            _event_array(predicted_events, "toe_off"),
            to_tolerance_s,
        ),
    }


def strict_event_pass(
    metrics: dict[str, dict[str, Any]],
    *,
    hs_tolerance_s: float,
    to_tolerance_s: float,
) -> bool:
    hs = metrics["heel_strike"]
    to = metrics["toe_off"]
    hs_max = hs["timing_max_abs_s"]
    to_max = to["timing_max_abs_s"]
    return bool(
        hs["precision"] == 1.0
        and hs["recall"] == 1.0
        and to["precision"] == 1.0
        and to["recall"] == 1.0
        and math.isfinite(hs_max)
        and math.isfinite(to_max)
        and hs_max <= hs_tolerance_s
        and to_max <= to_tolerance_s
    )


def contact_metrics(
    reference_force: np.ndarray,
    predicted_force: np.ndarray,
    *,
    reference_threshold_n: float,
    predicted_threshold_n: float,
) -> dict[str, float]:
    ref_contact = np.asarray(reference_force, dtype=float) > reference_threshold_n
    pred_contact = np.asarray(predicted_force, dtype=float) > predicted_threshold_n
    tp = int(np.count_nonzero(ref_contact & pred_contact))
    fp = int(np.count_nonzero(~ref_contact & pred_contact))
    fn = int(np.count_nonzero(ref_contact & ~pred_contact))
    tn = int(np.count_nonzero(~ref_contact & ~pred_contact))
    precision = tp / max(1, tp + fp)
    recall = tp / max(1, tp + fn)
    f1 = 2.0 * precision * recall / max(1.0e-12, precision + recall)
    iou = tp / max(1, tp + fp + fn)
    accuracy = (tp + tn) / max(1, tp + fp + fn + tn)
    return {
        "precision": float(precision),
        "recall": float(recall),
        "f1": float(f1),
        "iou": float(iou),
        "accuracy": float(accuracy),
    }


def cycles_from_events(events: dict[str, np.ndarray]) -> list[dict[str, float]]:
    hs = np.asarray(events.get("heel_strikes", []), dtype=float)
    to = np.asarray(events.get("toe_offs", []), dtype=float)
    cycles: list[dict[str, float]] = []
    for index, hs_time in enumerate(hs):
        next_hs = hs[index + 1] if index + 1 < len(hs) else float("nan")
        toe_after = to[to > hs_time + 1.0e-12]
        to_time = float(toe_after[0]) if len(toe_after) else float("nan")
        stance_duration = (
            to_time - float(hs_time) if math.isfinite(to_time) else float("nan")
        )
        period = (
            float(next_hs) - float(hs_time)
            if math.isfinite(float(next_hs))
            else float("nan")
        )
        duty_factor = (
            stance_duration / period
            if math.isfinite(stance_duration) and math.isfinite(period) and period > 0
            else float("nan")
        )
        cycles.append(
            {
                "cycle_index": float(index),
                "heel_strike_s": float(hs_time),
                "toe_off_s": float(to_time),
                "next_heel_strike_s": float(next_hs),
                "stance_duration_s": float(stance_duration),
                "period_s": float(period),
                "duty_factor": float(duty_factor),
            }
        )
    return cycles


def _write_csv(path: Path, rows: list[dict[str, Any]], fieldnames: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def _load_profiles(args: argparse.Namespace) -> list[Path]:
    paths: list[Path] = []
    for item in args.profile or []:
        paths.append(resolve_repo_path(item))
    for pattern in args.profile_glob or []:
        paths.extend(sorted(resolve_repo_path(".").glob(pattern)))
    if not paths:
        paths.append(resolve_repo_path(DEFAULT_PROFILE))
    unique: list[Path] = []
    seen: set[Path] = set()
    for path in paths:
        resolved = path.resolve()
        if resolved not in seen:
            seen.add(resolved)
            unique.append(resolved)
    return unique


def _sample_inputs(
    setup,
    profile,
    times: np.ndarray,
    *,
    states_sto: str,
    sea_plugin: str,
) -> tuple[np.ndarray, dict, str]:
    from validation.validate_online_grf import (
        _sample_spheres,
        _sample_spheres_from_coordinate_states,
    )

    if states_sto:
        state_times, samples = _sample_spheres_from_coordinate_states(
            setup,
            profile,
            states_sto,
            sea_plugin,
        )
        return state_times, samples, "saved_forward_coordinate_states"
    samples = _sample_spheres(setup, profile, times, sea_plugin)
    return times, samples, "ik_replay"


def _external_vertical_grf(setup, times: np.ndarray) -> dict[str, np.ndarray]:
    from validation.validate_online_grf import _external_grf

    external = _external_grf(setup, times)
    return {side: np.asarray(external[side][:, 1], dtype=float) for side in external}


def _online_vertical_grf(profile, samples: dict) -> dict[str, np.ndarray]:
    from validation.validate_online_grf import _calculate_grf

    predicted = _calculate_grf(profile, samples)
    return {side: np.asarray(predicted[side][:, 1], dtype=float) for side in predicted}


def _candidate_score(item: dict[str, Any], primary_side: str) -> tuple:
    side = item["sides"][primary_side]
    hs = side["events"]["heel_strike"]
    to = side["events"]["toe_off"]
    return (
        not bool(side["pass"]),
        -(hs["f1"] + to["f1"]),
        hs["false_positives"]
        + hs["false_negatives"]
        + to["false_positives"]
        + to["false_negatives"],
        max(
            999.0 if not math.isfinite(hs["timing_max_abs_s"]) else hs["timing_max_abs_s"],
            999.0 if not math.isfinite(to["timing_max_abs_s"]) else to["timing_max_abs_s"],
        ),
        -side["contact"]["f1"],
    )


def _plot_events(
    path: Path,
    times: np.ndarray,
    reference: dict[str, np.ndarray],
    predicted: dict[str, np.ndarray],
    reference_events: dict[str, dict[str, np.ndarray]],
    predicted_events: dict[str, dict[str, np.ndarray]],
    *,
    profile_label: str,
    params: dict[str, float],
) -> str:
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:  # pragma: no cover - optional local dependency.
        return f"plot skipped: {type(exc).__name__}: {exc}"

    figure, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    for axis, side in zip(axes, ("left", "right")):
        axis.plot(times, reference[side], label="prescribed Fy", linewidth=1.5)
        axis.plot(times, predicted[side], label="online Fy", linewidth=1.2)
        for event_time in reference_events[side]["heel_strikes"]:
            axis.axvline(event_time, color="tab:blue", alpha=0.35, linewidth=0.8)
        for event_time in reference_events[side]["toe_offs"]:
            axis.axvline(
                event_time,
                color="tab:blue",
                alpha=0.35,
                linewidth=0.8,
                linestyle="--",
            )
        for event_time in predicted_events[side]["heel_strikes"]:
            axis.axvline(event_time, color="tab:orange", alpha=0.35, linewidth=0.8)
        for event_time in predicted_events[side]["toe_offs"]:
            axis.axvline(
                event_time,
                color="tab:orange",
                alpha=0.35,
                linewidth=0.8,
                linestyle="--",
            )
        axis.set_ylabel(f"{side} Fy [N]")
        axis.grid(True, alpha=0.25)
        axis.legend(loc="upper right")
    axes[-1].set_xlabel("time [s]")
    figure.suptitle(
        "Online GRF event validation: "
        f"{profile_label}, low={params['low_threshold_n']:.3g} N, "
        f"confirm={params['confirmation_threshold_n']:.3g} N, "
        f"min_contact={params['min_contact_duration_s']:.3g} s, "
        f"smooth={params['smoothing_window_s']:.3g} s"
    )
    figure.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=160)
    plt.close(figure)
    return ""


def run_validation(args: argparse.Namespace) -> dict[str, Any]:
    setup_path = resolve_repo_path(args.setup)
    setup = read_setup_xml(setup_path)
    if setup.external_loads_xml is None:
        raise ValueError("event validation requires prescribed ExternalLoads")

    profiles = _load_profiles(args)
    output_dir = resolve_repo_path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    times = np.arange(
        float(setup.t_start),
        float(setup.t_end) + args.sample_dt * 0.25,
        args.sample_dt,
        dtype=float,
    )

    low_thresholds = (
        np.asarray(_float_list(args.threshold_values), dtype=float)
        if args.threshold_values
        else _threshold_grid(
            args.threshold_min,
            args.threshold_max,
            args.threshold_step,
        )
    )
    confirmation_thresholds = (
        np.asarray(_float_list(args.confirmation_values), dtype=float)
        if args.confirmation_values
        else _threshold_grid(
            args.confirmation_min,
            args.confirmation_max,
            args.confirmation_step,
        )
    )
    min_contact_values = _float_list(args.min_contact_duration_values)
    smoothing_values = _float_list(args.smoothing_window_s_values)

    report_profiles: list[dict[str, Any]] = []
    sweep_rows: list[dict[str, Any]] = []
    best_overall: dict[str, Any] | None = None
    best_signals: tuple[dict[str, np.ndarray], dict[str, dict[str, np.ndarray]]] | None = None
    reference_for_best: dict[str, dict[str, np.ndarray]] | None = None
    reference_force_for_best: dict[str, np.ndarray] | None = None
    times_for_best: np.ndarray | None = None
    input_type_for_best = "ik_replay"
    input_type = "ik_replay"

    for profile_path in profiles:
        profile = load_online_grf_profile(profile_path)
        profile_times, samples, input_type = _sample_inputs(
            setup,
            profile,
            times,
            states_sto=args.states_sto,
            sea_plugin=args.sea_plugin,
        )
        reference_force = _external_vertical_grf(setup, profile_times)
        online_force_raw = _online_vertical_grf(profile, samples)
        reference_events: dict[str, dict[str, np.ndarray]] = {}
        for side in ("left", "right"):
            reference_events[side] = detect_events(
                profile_times,
                reference_force[side],
                threshold_n=args.prescribed_threshold,
                confirmation_threshold_n=args.prescribed_threshold,
                min_contact_duration_s=args.reference_min_contact_duration,
                min_cycle_duration_s=args.min_cycle_duration,
            )

        profile_candidates: list[dict[str, Any]] = []
        for smoothing_window in smoothing_values:
            online_force = {
                side: smooth_force(profile_times, force, smoothing_window)
                for side, force in online_force_raw.items()
            }
            for min_contact in min_contact_values:
                for low_threshold in low_thresholds:
                    for confirmation_threshold in confirmation_thresholds:
                        if confirmation_threshold < low_threshold:
                            continue
                        candidate_events: dict[str, dict[str, np.ndarray]] = {}
                        side_reports: dict[str, Any] = {}
                        for side in ("left", "right"):
                            candidate_events[side] = detect_events(
                                profile_times,
                                online_force[side],
                                threshold_n=float(low_threshold),
                                confirmation_threshold_n=float(confirmation_threshold),
                                min_contact_duration_s=float(min_contact),
                                min_cycle_duration_s=float(args.min_cycle_duration),
                            )
                            metrics = event_metrics(
                                reference_events[side],
                                candidate_events[side],
                                hs_tolerance_s=float(args.hs_tolerance),
                                to_tolerance_s=float(args.to_tolerance),
                            )
                            contact = contact_metrics(
                                reference_force[side],
                                online_force[side],
                                reference_threshold_n=float(args.prescribed_threshold),
                                predicted_threshold_n=float(low_threshold),
                            )
                            side_reports[side] = {
                                "events": metrics,
                                "contact": contact,
                                "pass": strict_event_pass(
                                    metrics,
                                    hs_tolerance_s=float(args.hs_tolerance),
                                    to_tolerance_s=float(args.to_tolerance),
                                ),
                            }
                        primary = side_reports[args.primary_side]
                        candidate = {
                            "profile": str(profile_path),
                            "profile_name": profile_path.name,
                            "low_threshold_n": float(low_threshold),
                            "confirmation_threshold_n": float(confirmation_threshold),
                            "min_contact_duration_s": float(min_contact),
                            "smoothing_window_s": float(smoothing_window),
                            "primary_side": str(args.primary_side),
                            "primary_pass": bool(primary["pass"]),
                            "sides": side_reports,
                        }
                        profile_candidates.append(candidate)
                        sweep_rows.append(_sweep_row(candidate, args.primary_side))

        profile_candidates.sort(key=lambda item: _candidate_score(item, args.primary_side))
        best_profile = profile_candidates[0]
        report_profiles.append(
            {
                "profile": str(profile_path),
                "profile_name": profile_path.name,
                "best_candidate": best_profile,
                "top_candidates": profile_candidates[: int(args.max_report_candidates)],
                "reference_event_counts": {
                    side: {
                        "heel_strikes": int(
                            len(reference_events[side]["heel_strikes"])
                        ),
                        "toe_offs": int(len(reference_events[side]["toe_offs"])),
                    }
                    for side in ("left", "right")
                },
            }
        )
        if best_overall is None or _candidate_score(
            best_profile, args.primary_side
        ) < _candidate_score(best_overall, args.primary_side):
            best_overall = best_profile
            reference_for_best = reference_events
            reference_force_for_best = reference_force
            times_for_best = profile_times
            input_type_for_best = input_type
            best_forces = {
                side: smooth_force(
                    profile_times,
                    online_force_raw[side],
                    float(best_profile["smoothing_window_s"]),
                )
                for side in ("left", "right")
            }
            best_events = {
                side: detect_events(
                    profile_times,
                    best_forces[side],
                    threshold_n=float(best_profile["low_threshold_n"]),
                    confirmation_threshold_n=float(
                        best_profile["confirmation_threshold_n"]
                    ),
                    min_contact_duration_s=float(
                        best_profile["min_contact_duration_s"]
                    ),
                    min_cycle_duration_s=float(args.min_cycle_duration),
                )
                for side in ("left", "right")
            }
            best_signals = (best_forces, best_events)

    assert best_overall is not None
    assert best_signals is not None
    assert reference_for_best is not None
    assert reference_force_for_best is not None
    assert times_for_best is not None

    sweep_rows.sort(
        key=lambda row: (
            row["primary_pass"] != "True",
            -float(row["primary_hs_f1"]),
            -float(row["primary_to_f1"]),
            float(row["primary_event_count_error"]),
            float(row["primary_max_event_error_s"]),
        )
    )
    _write_csv(output_dir / "sweep_ranking.csv", sweep_rows, list(sweep_rows[0]))

    profile_rows = [
        _profile_ranking_row(profile_report, args.primary_side)
        for profile_report in report_profiles
    ]
    profile_rows.sort(
        key=lambda row: (
            row["primary_pass"] != "True",
            -float(row["primary_hs_f1"]),
            -float(row["primary_to_f1"]),
            float(row["primary_event_count_error"]),
            float(row["primary_max_event_error_s"]),
        )
    )
    _write_csv(output_dir / "profile_ranking.csv", profile_rows, list(profile_rows[0]))

    reference_rows: list[dict[str, Any]] = []
    for side in ("left", "right"):
        for cycle in cycles_from_events(reference_for_best[side]):
            reference_rows.append({"side": side, **cycle})
    _write_csv(
        output_dir / "reference_events.csv",
        reference_rows,
        [
            "side",
            "cycle_index",
            "heel_strike_s",
            "toe_off_s",
            "next_heel_strike_s",
            "stance_duration_s",
            "period_s",
            "duty_factor",
        ],
    )

    match_rows: list[dict[str, Any]] = []
    for side in ("left", "right"):
        for event_name in ("heel_strike", "toe_off"):
            pairs = best_overall["sides"][side]["events"][event_name]["pairs"]
            for pair in pairs:
                match_rows.append({"side": side, "event": event_name, **pair})
    _write_csv(
        output_dir / "best_event_matches.csv",
        match_rows,
        [
            "side",
            "event",
            "reference_time_s",
            "predicted_time_s",
            "error_s",
            "absolute_error_s",
        ],
    )

    plot_warning = ""
    if args.plot:
        plot_warning = _plot_events(
            resolve_repo_path(args.plot),
            times_for_best,
            reference_force_for_best,
            best_signals[0],
            reference_for_best,
            best_signals[1],
            profile_label=Path(best_overall["profile"]).name,
            params=best_overall,
        )
    else:
        plot_warning = _plot_events(
            output_dir / "online_grf_event_timing.png",
            times_for_best,
            reference_force_for_best,
            best_signals[0],
            reference_for_best,
            best_signals[1],
            profile_label=Path(best_overall["profile"]).name,
            params=best_overall,
        )

    final_pass = bool(best_overall["sides"][args.primary_side]["pass"])
    report = {
        "ok": final_pass,
        "status": "PASS" if final_pass else "FAIL",
        "objective": "event_shape_timing_only",
        "setup": str(setup_path.resolve()),
        "input_type": input_type_for_best,
        "states_sto": (
            str(resolve_repo_path(args.states_sto).resolve())
            if args.states_sto
            else ""
        ),
        "time_range": [float(times_for_best[0]), float(times_for_best[-1])],
        "sample_dt_s": float(args.sample_dt),
        "primary_side": str(args.primary_side),
        "strict_gate": {
            "hs_precision": 1.0,
            "hs_recall": 1.0,
            "to_precision": 1.0,
            "to_recall": 1.0,
            "hs_max_abs_error_s": float(args.hs_tolerance),
            "to_max_abs_error_s": float(args.to_tolerance),
        },
        "prescribed_reference": {
            "threshold_n": float(args.prescribed_threshold),
            "min_contact_duration_s": float(args.reference_min_contact_duration),
            "min_cycle_duration_s": float(args.min_cycle_duration),
        },
        "best_candidate": best_overall,
        "profiles": report_profiles,
        "artifacts": {
            "summary_json": str((output_dir / "summary.json").resolve()),
            "reference_events_csv": str(
                (output_dir / "reference_events.csv").resolve()
            ),
            "best_event_matches_csv": str(
                (output_dir / "best_event_matches.csv").resolve()
            ),
            "sweep_ranking_csv": str((output_dir / "sweep_ranking.csv").resolve()),
            "profile_ranking_csv": str(
                (output_dir / "profile_ranking.csv").resolve()
            ),
            "plot": str(
                resolve_repo_path(args.plot).resolve()
                if args.plot
                else (output_dir / "online_grf_event_timing.png").resolve()
            ),
            "report_md": str((output_dir / "report.md").resolve()),
        },
        "plot_warning": plot_warning,
        "notes": [
            "Magnitude, impulse, COP, and moment agreement do not affect this gate.",
            "Right side is diagnostic unless --primary-side right is selected.",
        ],
    }
    (output_dir / "summary.json").write_text(
        json.dumps(report, indent=2, allow_nan=True) + "\n",
        encoding="utf-8",
    )
    _write_markdown_report(output_dir / "report.md", report)
    return report


def _sweep_row(candidate: dict[str, Any], primary_side: str) -> dict[str, Any]:
    side = candidate["sides"][primary_side]
    hs = side["events"]["heel_strike"]
    to = side["events"]["toe_off"]
    count_error = (
        hs["false_positives"]
        + hs["false_negatives"]
        + to["false_positives"]
        + to["false_negatives"]
    )
    max_errors = [
        value
        for value in (hs["timing_max_abs_s"], to["timing_max_abs_s"])
        if math.isfinite(float(value))
    ]
    return {
        "profile_name": candidate["profile_name"],
        "profile": candidate["profile"],
        "low_threshold_n": candidate["low_threshold_n"],
        "confirmation_threshold_n": candidate["confirmation_threshold_n"],
        "min_contact_duration_s": candidate["min_contact_duration_s"],
        "smoothing_window_s": candidate["smoothing_window_s"],
        "primary_side": primary_side,
        "primary_pass": str(bool(side["pass"])),
        "primary_hs_precision": hs["precision"],
        "primary_hs_recall": hs["recall"],
        "primary_hs_f1": hs["f1"],
        "primary_hs_max_abs_error_s": hs["timing_max_abs_s"],
        "primary_to_precision": to["precision"],
        "primary_to_recall": to["recall"],
        "primary_to_f1": to["f1"],
        "primary_to_max_abs_error_s": to["timing_max_abs_s"],
        "primary_event_count_error": int(count_error),
        "primary_max_event_error_s": max(max_errors) if max_errors else float("nan"),
        "primary_contact_f1": side["contact"]["f1"],
        "primary_contact_iou": side["contact"]["iou"],
    }


def _profile_ranking_row(
    profile_report: dict[str, Any],
    primary_side: str,
) -> dict[str, Any]:
    row = _sweep_row(profile_report["best_candidate"], primary_side)
    row["profile_reference_hs_count"] = profile_report["reference_event_counts"][
        primary_side
    ]["heel_strikes"]
    row["profile_reference_to_count"] = profile_report["reference_event_counts"][
        primary_side
    ]["toe_offs"]
    return row


def _fmt(value: Any, digits: int = 6) -> str:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return str(value)
    if not math.isfinite(number):
        return "nan"
    return f"{number:.{digits}g}"


def _write_markdown_report(path: Path, report: dict[str, Any]) -> None:
    best = report["best_candidate"]
    primary_side = report["primary_side"]
    primary = best["sides"][primary_side]
    hs = primary["events"]["heel_strike"]
    to = primary["events"]["toe_off"]
    contact = primary["contact"]
    gate = report["strict_gate"]
    artifacts = report["artifacts"]
    lines = [
        "# Online GRF Event Validation",
        "",
        f"Status: **{report['status']}**",
        "",
        "Objective: validate online GRF as an HS/TO timing and contact-shape "
        "signal. Magnitude, impulse, COP, and moments are diagnostic only.",
        "",
        "## Strict Gate",
        "",
        f"- Primary side: `{primary_side}`",
        f"- HS precision/recall: {gate['hs_precision']}/{gate['hs_recall']}",
        f"- TO precision/recall: {gate['to_precision']}/{gate['to_recall']}",
        f"- HS max abs error <= {_fmt(gate['hs_max_abs_error_s'])} s",
        f"- TO max abs error <= {_fmt(gate['to_max_abs_error_s'])} s",
        "",
        "## Recommended Profile And Detector",
        "",
        f"- Profile: `{best['profile_name']}`",
        f"- Low threshold: {_fmt(best['low_threshold_n'])} N",
        f"- Confirmation threshold: {_fmt(best['confirmation_threshold_n'])} N",
        f"- Min contact duration: {_fmt(best['min_contact_duration_s'])} s",
        f"- Causal smoothing window: {_fmt(best['smoothing_window_s'])} s",
        "",
        "## Primary-Side Results",
        "",
        "| Event | Precision | Recall | F1 | Max abs error [s] | FP | FN |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
        "| HS | "
        f"{_fmt(hs['precision'])} | {_fmt(hs['recall'])} | {_fmt(hs['f1'])} | "
        f"{_fmt(hs['timing_max_abs_s'])} | {hs['false_positives']} | "
        f"{hs['false_negatives']} |",
        "| TO | "
        f"{_fmt(to['precision'])} | {_fmt(to['recall'])} | {_fmt(to['f1'])} | "
        f"{_fmt(to['timing_max_abs_s'])} | {to['false_positives']} | "
        f"{to['false_negatives']} |",
        "",
        "Secondary contact interval metrics:",
        "",
        f"- Contact precision: {_fmt(contact['precision'])}",
        f"- Contact recall: {_fmt(contact['recall'])}",
        f"- Contact F1: {_fmt(contact['f1'])}",
        f"- Contact IoU: {_fmt(contact['iou'])}",
        "",
        "## Artifacts",
        "",
        f"- Summary JSON: `{artifacts['summary_json']}`",
        f"- Reference events CSV: `{artifacts['reference_events_csv']}`",
        f"- Best event matches CSV: `{artifacts['best_event_matches_csv']}`",
        f"- Sweep ranking CSV: `{artifacts['sweep_ranking_csv']}`",
        f"- Profile ranking CSV: `{artifacts['profile_ranking_csv']}`",
        f"- Plot: `{artifacts['plot']}`",
    ]
    if report.get("plot_warning"):
        lines.extend(["", f"Plot warning: `{report['plot_warning']}`"])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Validate online GRF as an HS/TO timing signal against prescribed GRF."
        )
    )
    parser.add_argument("--setup", default=DEFAULT_SETUP)
    parser.add_argument("--profile", action="append", default=[])
    parser.add_argument(
        "--profile-glob",
        action="append",
        default=[],
        help="Optional repo-relative glob, e.g. 'online_grf_profiles/*.json'.",
    )
    parser.add_argument("--states-sto", default="")
    parser.add_argument("--sample-dt", type=float, default=0.001)
    parser.add_argument("--prescribed-threshold", type=float, default=20.0)
    parser.add_argument("--reference-min-contact-duration", type=float, default=0.05)
    parser.add_argument("--min-cycle-duration", type=float, default=0.30)
    parser.add_argument("--threshold-min", type=float, default=5.0)
    parser.add_argument("--threshold-max", type=float, default=200.0)
    parser.add_argument("--threshold-step", type=float, default=5.0)
    parser.add_argument(
        "--threshold-values",
        default="",
        help="Optional comma-separated low-threshold values; overrides min/max/step.",
    )
    parser.add_argument("--confirmation-min", type=float, default=5.0)
    parser.add_argument("--confirmation-max", type=float, default=200.0)
    parser.add_argument("--confirmation-step", type=float, default=5.0)
    parser.add_argument(
        "--confirmation-values",
        default="",
        help=(
            "Optional comma-separated confirmation-threshold values; overrides "
            "min/max/step."
        ),
    )
    parser.add_argument(
        "--min-contact-duration-values",
        default="0.03,0.05,0.07",
        help="Comma-separated detector min-contact durations to sweep.",
    )
    parser.add_argument(
        "--smoothing-window-s-values",
        default="0,0.005,0.010,0.020",
        help="Comma-separated causal smoothing windows to sweep.",
    )
    parser.add_argument("--primary-side", choices=("left", "right"), default="left")
    parser.add_argument("--hs-tolerance", type=float, default=0.050)
    parser.add_argument("--to-tolerance", type=float, default=0.080)
    parser.add_argument(
        "--output-dir",
        default="results/online_grf_event_validation",
    )
    parser.add_argument("--plot", default="")
    parser.add_argument("--max-report-candidates", type=int, default=25)
    parser.add_argument("--sea-plugin", default=DEFAULT_SEA_PLUGIN)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    report = run_validation(args)
    print(
        json.dumps(
            {
                "status": report["status"],
                "primary_side": report["primary_side"],
                "best_profile": report["best_candidate"]["profile_name"],
                "best_detector": {
                    "low_threshold_n": report["best_candidate"]["low_threshold_n"],
                    "confirmation_threshold_n": report["best_candidate"][
                        "confirmation_threshold_n"
                    ],
                    "min_contact_duration_s": report["best_candidate"][
                        "min_contact_duration_s"
                    ],
                    "smoothing_window_s": report["best_candidate"][
                        "smoothing_window_s"
                    ],
                },
                "left": report["best_candidate"]["sides"]["left"],
                "right": report["best_candidate"]["sides"]["right"],
                "artifacts": report["artifacts"],
            },
            indent=2,
            allow_nan=True,
        )
    )
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
