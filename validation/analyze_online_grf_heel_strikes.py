"""Validate onlineGRF heel strikes against prescribed GRF events."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from online_grf import load_online_grf_profile
from path_resolver import resolve_repo_path
from setup_io import read_setup_xml
from validation.validate_online_grf import (
    _calculate_grf,
    _external_grf,
    _sample_spheres,
    _sample_spheres_from_coordinate_states,
)


def _sustained_contact_events(
    times: np.ndarray,
    vertical_force: np.ndarray,
    threshold: float,
    min_contact_duration: float,
) -> dict[str, np.ndarray]:
    contact = vertical_force > threshold
    changes = np.diff(np.concatenate(([False], contact, [False])).astype(int))
    starts = np.flatnonzero(changes == 1)
    ends = np.flatnonzero(changes == -1)
    heel_strikes = []
    toe_offs = []
    durations = []
    for start, end in zip(starts, ends):
        start_time = float(times[start])
        end_time = float(times[min(end, len(times) - 1)])
        duration = end_time - start_time
        if duration + 1.0e-12 < min_contact_duration:
            continue
        heel_strikes.append(start_time)
        toe_offs.append(end_time)
        durations.append(duration)
    return {
        "heel_strikes": np.asarray(heel_strikes, dtype=float),
        "toe_offs": np.asarray(toe_offs, dtype=float),
        "contact_durations": np.asarray(durations, dtype=float),
    }


def _confirmed_contact_events(
    times: np.ndarray,
    vertical_force: np.ndarray,
    low_threshold: float,
    confirmation_threshold: float,
    min_contact_duration: float,
) -> dict[str, np.ndarray]:
    contact = vertical_force > low_threshold
    changes = np.diff(np.concatenate(([False], contact, [False])).astype(int))
    starts = np.flatnonzero(changes == 1)
    ends = np.flatnonzero(changes == -1)
    heel_strikes = []
    confirmation_times = []
    toe_offs = []
    for start, end in zip(starts, ends):
        end_index = min(end, len(times) - 1)
        start_time = float(times[start])
        end_time = float(times[end_index])
        if end_time - start_time + 1.0e-12 < min_contact_duration:
            continue
        segment = vertical_force[start:end]
        confirmed = np.flatnonzero(segment > confirmation_threshold)
        if not len(confirmed):
            continue
        heel_strikes.append(start_time)
        confirmation_times.append(float(times[start + int(confirmed[0])]))
        toe_offs.append(end_time)
    heel_strikes_array = np.asarray(heel_strikes, dtype=float)
    confirmation_array = np.asarray(confirmation_times, dtype=float)
    confirmation_array = np.maximum(
        confirmation_array,
        heel_strikes_array + min_contact_duration,
    )
    return {
        "heel_strikes": heel_strikes_array,
        "confirmation_times": confirmation_array,
        "confirmation_latencies": confirmation_array - heel_strikes_array,
        "toe_offs": np.asarray(toe_offs, dtype=float),
    }


def _match_events(
    reference: np.ndarray,
    predicted: np.ndarray,
    tolerance: float,
) -> dict:
    reference = np.asarray(reference, dtype=float)
    predicted = np.asarray(predicted, dtype=float)
    candidates = sorted(
        (
            abs(float(predicted[pred_index] - reference[ref_index])),
            ref_index,
            pred_index,
        )
        for ref_index in range(len(reference))
        for pred_index in range(len(predicted))
        if abs(float(predicted[pred_index] - reference[ref_index])) <= tolerance
    )
    matched_reference = set()
    matched_predicted = set()
    pairs = []
    for absolute_error, ref_index, pred_index in candidates:
        if ref_index in matched_reference or pred_index in matched_predicted:
            continue
        matched_reference.add(ref_index)
        matched_predicted.add(pred_index)
        error = float(predicted[pred_index] - reference[ref_index])
        pairs.append(
            {
                "reference_time_s": float(reference[ref_index]),
                "predicted_time_s": float(predicted[pred_index]),
                "error_s": error,
                "absolute_error_s": absolute_error,
            }
        )
    errors = np.asarray([pair["error_s"] for pair in pairs], dtype=float)
    tp = len(pairs)
    fp = len(predicted) - tp
    fn = len(reference) - tp
    precision = tp / max(1, tp + fp)
    recall = tp / max(1, tp + fn)
    f1 = 2.0 * precision * recall / max(1.0e-12, precision + recall)
    return {
        "reference_count": int(len(reference)),
        "predicted_count": int(len(predicted)),
        "matched_count": int(tp),
        "false_positives": int(fp),
        "false_negatives": int(fn),
        "precision": float(precision),
        "recall": float(recall),
        "f1": float(f1),
        "timing_bias_s": float(np.mean(errors)) if len(errors) else float("nan"),
        "timing_mae_s": float(np.mean(np.abs(errors))) if len(errors) else float("nan"),
        "timing_max_abs_s": (
            float(np.max(np.abs(errors))) if len(errors) else float("nan")
        ),
        "pairs": pairs,
    }


def _threshold_score(side_reports: dict[str, dict]) -> tuple[float, float, int]:
    f1 = float(np.mean([side_reports[side]["f1"] for side in ("left", "right")]))
    maes = [
        side_reports[side]["timing_mae_s"]
        for side in ("left", "right")
        if np.isfinite(side_reports[side]["timing_mae_s"])
    ]
    mae = float(np.mean(maes)) if maes else float("inf")
    count_error = sum(
        side_reports[side]["false_positives"] + side_reports[side]["false_negatives"]
        for side in ("left", "right")
    )
    return f1, mae, count_error


def _perfect_threshold_ranges(threshold_reports: list[dict]) -> list[dict]:
    perfect = [
        report
        for report in threshold_reports
        if report["macro_f1"] >= 1.0 - 1.0e-12
        and report["total_count_error"] == 0
    ]
    if not perfect:
        return []
    ranges = []
    current = [perfect[0]]
    for report in perfect[1:]:
        previous = current[-1]
        if report["threshold_n"] - previous["threshold_n"] <= 1.0 + 1.0e-9:
            current.append(report)
        else:
            ranges.append(current)
            current = [report]
    ranges.append(current)
    return [
        {
            "min_threshold_n": float(items[0]["threshold_n"]),
            "max_threshold_n": float(items[-1]["threshold_n"]),
            "width_n": float(items[-1]["threshold_n"] - items[0]["threshold_n"]),
        }
        for items in ranges
    ]


def _plot(
    destination: Path,
    times: np.ndarray,
    reference: dict[str, np.ndarray],
    predicted: dict[str, np.ndarray],
    prescribed_threshold: float,
    online_threshold: float,
) -> None:
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(2, 1, figsize=(13, 8), sharex=True)
    for axis, side in zip(axes, ("left", "right")):
        axis.plot(times, reference[side][:, 1], label="prescribed", linewidth=1.5)
        axis.plot(times, predicted[side][:, 1], label="online_sensor", linewidth=1.2)
        axis.axhline(
            prescribed_threshold,
            color="tab:blue",
            linestyle=":",
            label=f"prescribed threshold {prescribed_threshold:g} N",
        )
        axis.axhline(
            online_threshold,
            color="tab:orange",
            linestyle=":",
            label=f"online threshold {online_threshold:g} N",
        )
        axis.set_ylabel(f"{side} vertical GRF [N]")
        axis.grid(True, alpha=0.25)
        axis.legend(loc="upper right")
    axes[-1].set_xlabel("time [s]")
    figure.tight_layout()
    destination.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(destination, dpi=160)
    plt.close(figure)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--setup", required=True)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--states-sto", default="")
    parser.add_argument("--sample-dt", type=float, default=0.005)
    parser.add_argument("--prescribed-threshold", type=float, default=20.0)
    parser.add_argument("--threshold-min", type=float, default=5.0)
    parser.add_argument("--threshold-max", type=float, default=200.0)
    parser.add_argument("--threshold-step", type=float, default=1.0)
    parser.add_argument("--min-contact-duration", type=float, default=0.05)
    parser.add_argument("--match-tolerance", type=float, default=0.10)
    parser.add_argument("--report", required=True)
    parser.add_argument("--plot", default="")
    parser.add_argument(
        "--sea-plugin",
        default="plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff",
    )
    args = parser.parse_args()

    setup = read_setup_xml(args.setup)
    if setup.external_loads_xml is None:
        raise ValueError("Heel-strike analysis requires prescribed ExternalLoads.")
    profile = load_online_grf_profile(args.profile)
    if args.states_sto:
        times, samples = _sample_spheres_from_coordinate_states(
            setup,
            profile,
            args.states_sto,
            args.sea_plugin,
        )
        input_type = "saved_forward_coordinate_states"
    else:
        times = np.arange(
            setup.t_start,
            setup.t_end + args.sample_dt * 0.25,
            args.sample_dt,
        )
        samples = _sample_spheres(setup, profile, times, args.sea_plugin)
        input_type = "ik_replay"
    reference = _external_grf(setup, times)
    predicted = _calculate_grf(profile, samples)
    reference_events = {
        side: _sustained_contact_events(
            times,
            reference[side][:, 1],
            args.prescribed_threshold,
            args.min_contact_duration,
        )
        for side in ("left", "right")
    }

    threshold_reports = []
    thresholds = np.arange(
        args.threshold_min,
        args.threshold_max + args.threshold_step * 0.25,
        args.threshold_step,
    )
    for threshold in thresholds:
        side_reports = {}
        for side in ("left", "right"):
            online_events = _sustained_contact_events(
                times,
                predicted[side][:, 1],
                float(threshold),
                args.min_contact_duration,
            )
            side_reports[side] = _match_events(
                reference_events[side]["heel_strikes"],
                online_events["heel_strikes"],
                args.match_tolerance,
            )
        macro_f1, timing_mae, count_error = _threshold_score(side_reports)
        threshold_reports.append(
            {
                "threshold_n": float(threshold),
                "macro_f1": macro_f1,
                "macro_timing_mae_s": timing_mae,
                "total_count_error": int(count_error),
                "sides": side_reports,
            }
        )
    best = min(
        threshold_reports,
        key=lambda item: (
            -item["macro_f1"],
            item["total_count_error"],
            item["macro_timing_mae_s"],
        ),
    )
    dual_threshold_reports = []
    for confirmation_threshold in thresholds[thresholds >= args.prescribed_threshold]:
        side_reports = {}
        confirmation_latencies = []
        for side in ("left", "right"):
            online_events = _confirmed_contact_events(
                times,
                predicted[side][:, 1],
                args.prescribed_threshold,
                float(confirmation_threshold),
                args.min_contact_duration,
            )
            side_reports[side] = _match_events(
                reference_events[side]["heel_strikes"],
                online_events["heel_strikes"],
                args.match_tolerance,
            )
            confirmation_latencies.extend(online_events["confirmation_latencies"])
        macro_f1, timing_mae, count_error = _threshold_score(side_reports)
        dual_threshold_reports.append(
            {
                "low_threshold_n": float(args.prescribed_threshold),
                "confirmation_threshold_n": float(confirmation_threshold),
                "macro_f1": macro_f1,
                "macro_timing_mae_s": timing_mae,
                "total_count_error": int(count_error),
                "mean_confirmation_latency_s": (
                    float(np.mean(confirmation_latencies))
                    if confirmation_latencies
                    else float("nan")
                ),
                "max_confirmation_latency_s": (
                    float(np.max(confirmation_latencies))
                    if confirmation_latencies
                    else float("nan")
                ),
                "sides": side_reports,
            }
        )
    best_dual = min(
        dual_threshold_reports,
        key=lambda item: (
            -item["macro_f1"],
            item["total_count_error"],
            item["mean_confirmation_latency_s"],
            item["macro_timing_mae_s"],
        ),
    )
    report = {
        "setup": str(Path(args.setup).resolve()),
        "profile": str(Path(args.profile).resolve()),
        "input_type": input_type,
        "states_sto": (
            str(resolve_repo_path(args.states_sto).resolve())
            if args.states_sto
            else ""
        ),
        "samples": int(len(times)),
        "time_range": [float(times[0]), float(times[-1])],
        "prescribed_threshold_n": float(args.prescribed_threshold),
        "min_contact_duration_s": float(args.min_contact_duration),
        "match_tolerance_s": float(args.match_tolerance),
        "best_online_threshold": best,
        "perfect_single_threshold_ranges": _perfect_threshold_ranges(
            threshold_reports
        ),
        "recommended_dual_threshold": best_dual,
        "perfect_dual_confirmation_threshold_ranges": _perfect_threshold_ranges(
            [
                {
                    **item,
                    "threshold_n": item["confirmation_threshold_n"],
                }
                for item in dual_threshold_reports
            ]
        ),
        "default_20n_online_threshold": next(
            (
                item
                for item in threshold_reports
                if abs(item["threshold_n"] - 20.0) < 1.0e-9
            ),
            None,
        ),
        "vertical_force_summary": {
            side: {
                "correlation": float(
                    np.corrcoef(reference[side][:, 1], predicted[side][:, 1])[0, 1]
                ),
                "reference_peak_n": float(np.max(reference[side][:, 1])),
                "online_peak_n": float(np.max(predicted[side][:, 1])),
            }
            for side in ("left", "right")
        },
    }
    destination = resolve_repo_path(args.report)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    if args.plot:
        _plot(
            resolve_repo_path(args.plot),
            times,
            reference,
            predicted,
            args.prescribed_threshold,
            best["threshold_n"],
        )
    print(
        json.dumps(
            {
                "best_single_threshold": best,
                "recommended_dual_threshold": best_dual,
                "perfect_dual_confirmation_threshold_ranges": report[
                    "perfect_dual_confirmation_threshold_ranges"
                ],
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
