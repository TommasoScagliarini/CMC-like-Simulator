#!/usr/bin/env python3
"""Decompose V26 trajectory latency with the recorded policy held fixed.

The audit is deliberately offline: it does not run PPO, change the policy, or
mutate the simulator.  For each deterministic rollout it compares four signals
at the end of every 10 ms policy segment:

1. actor raw target, converted from normalized absolute action to radians;
2. target after the target-to-target slew limiter;
3. reference served by the C2 third-order reference model;
4. physical OpenSim coordinate.

Each stage is compared both to the preceding stage in wall-clock time and to a
phase-normalized prescribed CMC-like reference using the same HS->TO / TO->HS
event warping as the morphology profile.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import tempfile
from pathlib import Path
from typing import Any

# Keep the audit runnable in sandboxed/macOS environments where ~/.matplotlib
# is read-only.  Respect an explicit caller-provided cache path.
_MPL_CACHE = Path(tempfile.gettempdir()) / "cmc_v26_latency_matplotlib"
_MPL_CACHE.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(_MPL_CACHE))
_XDG_CACHE = Path(tempfile.gettempdir()) / "cmc_v26_latency_cache"
_XDG_CACHE.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("XDG_CACHE_HOME", str(_XDG_CACHE))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE = (
    REPO_ROOT
    / "Trajectory Generator/baseline_MLP/morphology_profiles/"
    "ab06_prosthetic_event_warped_mean_std_corridor.json"
)
DEFAULT_PRESCRIBED_KINEMATICS = REPO_ROOT / "results/sim_output_kinematics.sto"
DEFAULT_PRESCRIBED_EVENTS = REPO_ROOT / "results/sim_output_gait_events.csv"
DEFAULT_RUNS = (
    REPO_ROOT
    / "Trajectory Generator/runs/rollout/"
    "MLP_ExNovo_V26C_J20_pilot_2026-08-28_best_rollout",
    REPO_ROOT
    / "Trajectory Generator/runs/rollout/"
    "MLP_ExNovo_V26C_J20_morphology_0p025_50updates_2026-08-28_best_rollout",
    REPO_ROOT
    / "Trajectory Generator/runs/rollout/"
    "MLP_ExNovo_V26C_J20_morphology_0p10_50updates_2026-08-28_best_rollout",
    REPO_ROOT
    / "Trajectory Generator/runs/rollout/"
    "MLP_ExNovo_V26C_J20_morphology_1p05_50updates_2026-08-29_best_rollout",
)
DEFAULT_OUTPUT_JSON = (
    REPO_ROOT
    / "validation/v26_reference_latency_audit/"
    "2026-08-30_frozen_policy_latency_decomposition.json"
)
DEFAULT_OUTPUT_PLOT = (
    REPO_ROOT
    / "plot/08_30_2026_v26_reference_latency_audit/"
    "01_frozen_policy_latency_decomposition.png"
)

GRID = np.linspace(0.0, 1.0, 201)
SHIFT_GRID = np.linspace(-0.25, 0.25, 501)
JOINTS = ("knee", "ankle")
STAGES = ("raw_actor", "after_slew", "served", "actual")
STAGE_PAIRS = (
    ("raw_actor", "after_slew"),
    ("after_slew", "served"),
    ("served", "actual"),
    ("raw_actor", "served"),
)

# Exact active action mapping from CMCEnvConfig.absolute_bounds_rad.  The audit
# fails if the rollout is not in absolute mode, rather than silently applying
# these bounds to another action contract.
ABSOLUTE_BOUNDS_RAD = {
    "knee": (-1.5, 0.0),
    "ankle": (-0.7, 0.7),
}


def _read_opensim_table(path: Path) -> dict[str, np.ndarray]:
    lines = path.read_text(encoding="utf-8").splitlines()
    try:
        end = next(
            index
            for index, line in enumerate(lines)
            if line.strip().lower() == "endheader"
        )
    except StopIteration as exc:
        raise ValueError(f"Missing endheader in {path}") from exc
    names = lines[end + 1].split()
    data = np.loadtxt(lines[end + 2 :], dtype=float)
    if data.ndim == 1:
        data = data[None, :]
    if data.shape[1] != len(names):
        raise ValueError(f"Column mismatch in {path}")
    return {name: data[:, index] for index, name in enumerate(names)}


def _prescribed_cycles(path: Path) -> list[tuple[float, float, float]]:
    rows = list(csv.DictReader(path.open(encoding="utf-8")))
    cycles = []
    for row in rows:
        if row.get("side") != "left":
            continue
        heel_strike = float(row["cycle_start"])
        toe_off = heel_strike + float(row["contact_duration_s"])
        next_heel_strike = float(row["cycle_end"])
        if heel_strike < toe_off < next_heel_strike:
            cycles.append((heel_strike, toe_off, next_heel_strike))
    return cycles


def _online_cycles(path: Path) -> list[tuple[float, float, float]]:
    rows = [
        row
        for row in csv.DictReader(path.open(encoding="utf-8"))
        if row.get("side") == "left"
    ]
    events = [(float(row["time"]), str(row["event"])) for row in rows]
    cycles = []
    for index, (heel_strike, event) in enumerate(events):
        if event != "heel_strike":
            continue
        toe_off = next(
            (time for time, name in events[index + 1 :] if name == "toe_off"),
            None,
        )
        next_heel_strike = next(
            (time for time, name in events[index + 1 :] if name == "heel_strike"),
            None,
        )
        if (
            toe_off is not None
            and next_heel_strike is not None
            and heel_strike < toe_off < next_heel_strike
        ):
            cycles.append((heel_strike, toe_off, next_heel_strike))
    return cycles


def _inverse_event_warp(
    cycle: tuple[float, float, float],
    alpha: float,
) -> np.ndarray:
    heel_strike, toe_off, next_heel_strike = cycle
    return np.where(
        GRID <= alpha,
        heel_strike + (GRID / alpha) * (toe_off - heel_strike),
        toe_off
        + ((GRID - alpha) / (1.0 - alpha)) * (next_heel_strike - toe_off),
    )


def _resample_complete_cycles(
    time: np.ndarray,
    signals: dict[str, np.ndarray],
    cycles: list[tuple[float, float, float]],
    alpha: float,
) -> tuple[dict[str, np.ndarray], list[tuple[float, float, float]]]:
    valid_cycles = [
        cycle
        for cycle in cycles
        if cycle[0] >= time[0] - 1e-9 and cycle[2] <= time[-1] + 1e-9
    ]
    if not valid_cycles:
        raise ValueError("No complete cycles lie inside the sampled trace")
    result: dict[str, np.ndarray] = {}
    for name, values in signals.items():
        result[name] = np.asarray(
            [
                np.interp(_inverse_event_warp(cycle, alpha), time, values)
                for cycle in valid_cycles
            ],
            dtype=float,
        )
    return result, valid_cycles


def _rmse(first: np.ndarray, second: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(first - second))))


def _correlation(first: np.ndarray, second: np.ndarray) -> float:
    if np.std(first) <= 0.0 or np.std(second) <= 0.0:
        return float("nan")
    return float(np.corrcoef(first, second)[0, 1])


def _best_phase_advance(
    reference: np.ndarray,
    candidate: np.ndarray,
) -> dict[str, float | int]:
    trials: list[tuple[float, float, float, int]] = []
    for advance in SHIFT_GRID:
        source_phase = GRID + advance
        valid = (source_phase >= 0.0) & (source_phase <= 1.0)
        shifted = np.interp(source_phase[valid], GRID, candidate)
        ref = reference[valid]
        trials.append(
            (
                float(advance),
                _rmse(ref, shifted),
                _correlation(ref, shifted),
                int(np.count_nonzero(valid)),
            )
        )
    best = min(trials, key=lambda item: item[1])
    return {
        "candidate_advance_phase": best[0],
        "rmse_rad": best[1],
        "pearson": best[2],
        "overlap_samples": best[3],
    }


def _best_wall_clock_lag(
    upstream: np.ndarray,
    downstream: np.ndarray,
    dt_s: float,
    max_lag_steps: int = 30,
) -> dict[str, float | int]:
    trials = []
    for lag in range(max_lag_steps + 1):
        left = upstream[: None if lag == 0 else -lag]
        right = downstream[lag:]
        trials.append(
            (
                lag,
                _rmse(left, right),
                _correlation(left, right),
                len(left),
            )
        )
    best = min(trials, key=lambda item: item[1])
    return {
        "lag_steps": int(best[0]),
        "lag_s": float(best[0] * dt_s),
        "rmse_rad": float(best[1]),
        "pearson": float(best[2]),
        "overlap_samples": int(best[3]),
        "zero_lag_rmse_rad": _rmse(upstream, downstream),
        "zero_lag_pearson": _correlation(upstream, downstream),
    }


def _map_absolute_action(action: np.ndarray, joint: str) -> np.ndarray:
    low, high = ABSOLUTE_BOUNDS_RAD[joint]
    return low + 0.5 * (action + 1.0) * (high - low)


def _trace_signals(
    trace: list[dict[str, Any]],
) -> tuple[np.ndarray, dict[str, dict[str, np.ndarray]]]:
    time = np.asarray([float(row["time"]) for row in trace], dtype=float)
    if len(time) < 2 or np.any(np.diff(time) <= 0.0):
        raise ValueError("Trace times must be strictly increasing")
    for row in trace:
        segment_times = np.asarray(row["policy_segment_times"], dtype=float)
        if abs(float(segment_times[-1]) - float(row["time"])) > 1e-8:
            raise ValueError("Raw/slew target endpoint is not aligned to trace time")

    raw_actions = np.asarray([row["raw_policy_action"] for row in trace], dtype=float)
    after_slew = np.asarray(
        [row["policy_segment_values"][-1] for row in trace], dtype=float
    )
    signals = {
        "knee": {
            "raw_actor": _map_absolute_action(raw_actions[:, 0], "knee"),
            "after_slew": after_slew[:, 0],
            "served": np.asarray(
                [
                    row["prosthetic_state"]["pros_knee_angle_served_ref"]
                    for row in trace
                ],
                dtype=float,
            ),
            "actual": np.asarray(
                [row["prosthetic_state"]["pros_knee_angle"] for row in trace],
                dtype=float,
            ),
        },
        "ankle": {
            "raw_actor": _map_absolute_action(raw_actions[:, 1], "ankle"),
            "after_slew": after_slew[:, 1],
            "served": np.asarray(
                [
                    row["prosthetic_state"]["pros_ankle_angle_served_ref"]
                    for row in trace
                ],
                dtype=float,
            ),
            "actual": np.asarray(
                [row["prosthetic_state"]["pros_ankle_angle"] for row in trace],
                dtype=float,
            ),
        },
    }
    return time, signals


def _governor_summary(
    trace: list[dict[str, Any]],
) -> dict[str, Any]:
    target_error = np.asarray(
        [
            row["reference_governor_diagnostics"]["target_error_rms_rad"]
            for row in trace
        ],
        dtype=float,
    )
    result: dict[str, Any] = {
        "target_error_rms_rad_mean": {
            "knee": float(np.mean(target_error[:, 0])),
            "ankle": float(np.mean(target_error[:, 1])),
        }
    }
    for key in (
        "velocity_limit_fraction",
        "acceleration_limit_fraction",
        "jerk_limit_fraction",
    ):
        values = np.asarray(
            [float(row["reference_governor_diagnostics"].get(key, 0.0)) for row in trace]
        )
        result[key] = {
            "mean": float(np.mean(values)),
            "max": float(np.max(values)),
            "nonzero_steps": int(np.count_nonzero(values > 0.0)),
        }
    return result


def _analyze_run(
    run_dir: Path,
    prescribed_mean: dict[str, np.ndarray],
    alpha: float,
) -> tuple[dict[str, Any], dict[str, dict[str, np.ndarray]]]:
    summary = json.loads((run_dir / "rollout_summary.json").read_text())
    if summary.get("action_mode") != "absolute":
        raise ValueError(f"{run_dir.name}: audit requires action_mode=absolute")
    trace = json.loads((run_dir / "rollout_policy_trace.json").read_text())
    time, signals = _trace_signals(trace)
    dt_s = float(np.median(np.diff(time)))
    if abs(dt_s - 0.01) > 1e-8:
        raise ValueError(f"{run_dir.name}: expected 10 ms policy step, got {dt_s}")

    cycles = _online_cycles(run_dir / "sim_outputs/rollout_episode_gait_events_online.csv")
    flat_signals = {
        f"{joint}_{stage}": signals[joint][stage]
        for joint in JOINTS
        for stage in STAGES
    }
    phase_curves, analyzed_cycles = _resample_complete_cycles(
        time,
        flat_signals,
        cycles,
        alpha,
    )
    mean_curves = {
        joint: {
            stage: np.mean(phase_curves[f"{joint}_{stage}"], axis=0)
            for stage in STAGES
        }
        for joint in JOINTS
    }
    detected_period_mean_s = float(np.mean([cycle[2] - cycle[0] for cycle in cycles]))
    analyzed_period_mean_s = float(
        np.mean([cycle[2] - cycle[0] for cycle in analyzed_cycles])
    )

    result: dict[str, Any] = {
        "run_dir": str(run_dir.relative_to(REPO_ROOT)),
        "morphology_weight": float(summary["reward_config"]["morphology_weight"]),
        "steps": len(trace),
        "policy_dt_s": dt_s,
        "detected_cycles": len(cycles),
        "analyzed_complete_cycles": len(analyzed_cycles),
        "detected_period_mean_s": detected_period_mean_s,
        "analyzed_period_mean_s": analyzed_period_mean_s,
        "morphology_reward_delay_s": float(
            summary["reward_config"]["morphology_reward_delay_s"]
        ),
        "governor": _governor_summary(trace),
        "joints": {},
    }

    for joint in JOINTS:
        joint_result: dict[str, Any] = {
            "phase_vs_prescribed": {},
            "wall_clock_stage_lags": {},
            "stage_landmarks": {},
        }
        toe_index = int(np.argmin(np.abs(GRID - alpha)))
        for stage in STAGES:
            curve = mean_curves[joint][stage]
            phase = _best_phase_advance(
                prescribed_mean[joint],
                curve,
            )
            phase["lag_ms_at_detected_period"] = float(
                1000.0
                * float(phase["candidate_advance_phase"])
                * detected_period_mean_s
            )
            joint_result["phase_vs_prescribed"][stage] = phase
            joint_result["stage_landmarks"][stage] = {
                "value_near_to_rad": float(curve[toe_index]),
                "minimum_rad": float(np.min(curve)),
                "minimum_phase": float(GRID[int(np.argmin(curve))]),
                "maximum_rad": float(np.max(curve)),
                "maximum_phase": float(GRID[int(np.argmax(curve))]),
            }

        raw = signals[joint]["raw_actor"]
        limited = signals[joint]["after_slew"]
        delta = raw - limited
        joint_result["slew_intervention"] = {
            "changed_fraction": float(np.mean(np.abs(delta) > 1e-10)),
            "rmse_raw_vs_after_slew_rad": _rmse(raw, limited),
            "mean_abs_change_rad": float(np.mean(np.abs(delta))),
            "max_abs_change_rad": float(np.max(np.abs(delta))),
        }
        for upstream, downstream in STAGE_PAIRS:
            joint_result["wall_clock_stage_lags"][
                f"{upstream}_to_{downstream}"
            ] = _best_wall_clock_lag(
                signals[joint][upstream],
                signals[joint][downstream],
                dt_s,
            )

        phase_lags = {
            stage: float(
                joint_result["phase_vs_prescribed"][stage][
                    "lag_ms_at_detected_period"
                ]
            )
            for stage in STAGES
        }
        joint_result["incremental_phase_lag_ms"] = {
            "actor_already_present": phase_lags["raw_actor"],
            "slew_added": phase_lags["after_slew"] - phase_lags["raw_actor"],
            "filter_added": phase_lags["served"] - phase_lags["after_slew"],
            "physical_tracking_added": phase_lags["actual"] - phase_lags["served"],
        }
        result["joints"][joint] = joint_result
    return result, mean_curves


def _aggregate(results: list[dict[str, Any]]) -> dict[str, Any]:
    aggregate: dict[str, Any] = {}
    for joint in JOINTS:
        aggregate[joint] = {}
        for key in (
            "actor_already_present",
            "slew_added",
            "filter_added",
            "physical_tracking_added",
        ):
            values = np.asarray(
                [result["joints"][joint]["incremental_phase_lag_ms"][key] for result in results]
            )
            aggregate[joint][f"{key}_ms"] = {
                "mean": float(np.mean(values)),
                "min": float(np.min(values)),
                "max": float(np.max(values)),
            }
        direct = {}
        for upstream, downstream in STAGE_PAIRS:
            name = f"{upstream}_to_{downstream}"
            values = np.asarray(
                [
                    result["joints"][joint]["wall_clock_stage_lags"][name]["lag_s"]
                    for result in results
                ]
            )
            direct[name] = {
                "mean_ms": float(1000.0 * np.mean(values)),
                "min_ms": float(1000.0 * np.min(values)),
                "max_ms": float(1000.0 * np.max(values)),
            }
        aggregate[joint]["direct_wall_clock_lags"] = direct
    return aggregate


def _plot(
    output: Path,
    alpha: float,
    prescribed_mean: dict[str, np.ndarray],
    representative: dict[str, dict[str, np.ndarray]],
    run_name: str,
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(2, 1, figsize=(12, 9), sharex=True)
    colors = {
        "raw_actor": "#7b2cbf",
        "after_slew": "#e76f51",
        "served": "#1d3557",
        "actual": "#2a9d8f",
    }
    labels = {
        "raw_actor": "Actor raw target",
        "after_slew": "Target after slew",
        "served": "Served after 4 Hz filter",
        "actual": "Physical coordinate",
    }
    for axis, joint in zip(axes, JOINTS):
        display_sign = -1.0 if joint == "knee" else 1.0
        axis.plot(
            100.0 * GRID,
            display_sign * prescribed_mean[joint],
            color="black",
            linewidth=2.8,
            label="CMC-like prescribed actual",
        )
        for stage in STAGES:
            axis.plot(
                100.0 * GRID,
                display_sign * representative[joint][stage],
                color=colors[stage],
                linewidth=1.8,
                label=labels[stage],
            )
        axis.axvline(100.0 * alpha, color="0.4", linestyle="--", linewidth=1.2)
        axis.text(
            100.0 * alpha + 0.7,
            0.96,
            "TO",
            transform=axis.get_xaxis_transform(),
            va="top",
            color="0.35",
        )
        axis.grid(alpha=0.25)
        axis.set_ylabel(f"{joint.capitalize()} angle [rad]")
        axis.set_title(
            f"{joint.capitalize()} — frozen-policy signal chain"
            + (" (flexion positive)" if joint == "knee" else "")
        )
    axes[-1].set_xlabel("Event-warped gait phase [%]")
    handles, labels_out = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels_out, loc="lower center", ncol=3, frameon=False)
    fig.suptitle(f"V26 reference latency decomposition\n{run_name}", fontsize=14)
    fig.tight_layout(rect=(0.0, 0.08, 1.0, 0.94))
    fig.savefig(output, dpi=180)
    plt.close(fig)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--rollout-dir",
        action="append",
        type=Path,
        help="Rollout directory; repeat for multiple runs. Defaults to all four V26 runs.",
    )
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument(
        "--prescribed-kinematics",
        type=Path,
        default=DEFAULT_PRESCRIBED_KINEMATICS,
    )
    parser.add_argument(
        "--prescribed-events",
        type=Path,
        default=DEFAULT_PRESCRIBED_EVENTS,
    )
    parser.add_argument("--output-json", type=Path, default=DEFAULT_OUTPUT_JSON)
    parser.add_argument("--output-plot", type=Path, default=DEFAULT_OUTPUT_PLOT)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    run_dirs = tuple(args.rollout_dir or DEFAULT_RUNS)
    profile = json.loads(args.profile.read_text(encoding="utf-8"))
    alpha = float(profile["metadata"]["canonical_to_phase"])
    if not 0.0 < alpha < 1.0:
        raise ValueError("Invalid morphology canonical TO phase")

    prescribed = _read_opensim_table(args.prescribed_kinematics)
    prescribed_signals = {
        "knee": prescribed["pros_knee_angle"],
        "ankle": prescribed["pros_ankle_angle"],
    }
    prescribed_curves, prescribed_cycles = _resample_complete_cycles(
        prescribed["time"],
        prescribed_signals,
        _prescribed_cycles(args.prescribed_events),
        alpha,
    )
    prescribed_mean = {
        joint: np.mean(prescribed_curves[joint], axis=0) for joint in JOINTS
    }
    toe_index = int(np.argmin(np.abs(GRID - alpha)))

    results = []
    curves_by_run: dict[str, dict[str, dict[str, np.ndarray]]] = {}
    for run_dir in run_dirs:
        resolved = run_dir if run_dir.is_absolute() else REPO_ROOT / run_dir
        result, curves = _analyze_run(resolved, prescribed_mean, alpha)
        results.append(result)
        curves_by_run[resolved.name] = curves

    payload = {
        "ok": True,
        "audit": "v26_frozen_policy_latency_decomposition_v1",
        "policy_mutated": False,
        "training_executed": False,
        "canonical_to_phase": alpha,
        "absolute_bounds_rad": ABSOLUTE_BOUNDS_RAD,
        "prescribed": {
            "kinematics": str(args.prescribed_kinematics.relative_to(REPO_ROOT)),
            "events": str(args.prescribed_events.relative_to(REPO_ROOT)),
            "complete_cycles": len(prescribed_cycles),
            "mean_landmarks": {
                joint: {
                    "value_near_to_rad": float(prescribed_mean[joint][toe_index]),
                    "minimum_rad": float(np.min(prescribed_mean[joint])),
                    "minimum_phase": float(
                        GRID[int(np.argmin(prescribed_mean[joint]))]
                    ),
                    "maximum_rad": float(np.max(prescribed_mean[joint])),
                    "maximum_phase": float(
                        GRID[int(np.argmax(prescribed_mean[joint]))]
                    ),
                }
                for joint in JOINTS
            },
        },
        "runs": results,
        "aggregate": _aggregate(results),
        "interpretation_contract": {
            "positive_phase_advance": (
                "candidate occurs later than prescribed and must be advanced"
            ),
            "wall_clock_lag": (
                "non-negative downstream sample lag minimizing time-domain RMSE"
            ),
            "incremental_phase_lag": (
                "difference of independently optimized phase lags; diagnostic, not additive causal proof"
            ),
        },
    }
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")

    representative_name = run_dirs[-1].name
    _plot(
        args.output_plot,
        alpha,
        prescribed_mean,
        curves_by_run[representative_name],
        representative_name,
    )
    print(
        json.dumps(
            {
                "ok": True,
                "output_json": str(args.output_json),
                "output_plot": str(args.output_plot),
                "aggregate": payload["aggregate"],
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
