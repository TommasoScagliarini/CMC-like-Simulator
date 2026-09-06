#!/usr/bin/env python3
"""Aggregate the V26C/J20 frozen-actor sigma exploration smoke sweep.

The script deliberately separates normalized policy noise from the physical
target perturbation that survives the per-step target slew limiter.  It does
not train or mutate a checkpoint.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path
from typing import Any, Iterable


JOINTS = ("knee", "ankle")
ACTION_OFFSET = (-0.75, 0.0)
ACTION_SCALE = (0.75, 0.70)
ACTION_LOW = (-1.5, -0.7)
ACTION_HIGH = (0.0, 0.7)
SLEW_PER_STEP = (0.025, 0.020)


def _load(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _finite(values: Iterable[Any]) -> list[float]:
    result: list[float] = []
    for value in values:
        try:
            number = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(number):
            result.append(number)
    return result


def _rms(values: Iterable[Any]) -> float | None:
    data = _finite(values)
    return math.sqrt(sum(value * value for value in data) / len(data)) if data else None


def _mean(values: Iterable[Any]) -> float | None:
    data = _finite(values)
    return statistics.fmean(data) if data else None


def _max(values: Iterable[Any]) -> float | None:
    data = _finite(values)
    return max(data) if data else None


def _last_numeric(rows: list[dict[str, Any]], section: str, key: str) -> float:
    for row in reversed(rows):
        value = (row.get(section) or {}).get(key)
        try:
            return float(value)
        except (TypeError, ValueError):
            pass
    return 0.0


def _max_numeric(rows: list[dict[str, Any]], section: str, keys: tuple[str, ...]) -> float:
    values = []
    for row in rows:
        block = row.get(section) or {}
        for key in keys:
            if key in block:
                values.append(block[key])
    return _max(values) or 0.0


def _timeout_rows(rows: list[dict[str, Any]], side: float) -> int:
    count = 0
    for row in rows:
        terms = row.get("reward_terms") or {}
        if float(terms.get("phase_timeout_exceeded", 0.0) or 0.0) <= 0.0:
            continue
        if math.isclose(float(terms.get("phase_timeout_side", 0.0) or 0.0), side):
            count += 1
    return count


def _physical(action: list[float]) -> list[float]:
    return [ACTION_OFFSET[i] + ACTION_SCALE[i] * float(action[i]) for i in range(2)]


def _slew(target: list[float], anchor: list[float]) -> list[float]:
    result = []
    for i in range(2):
        limited = min(max(target[i], anchor[i] - SLEW_PER_STEP[i]), anchor[i] + SLEW_PER_STEP[i])
        result.append(min(max(limited, ACTION_LOW[i]), ACTION_HIGH[i]))
    return result


def _actor_configuration(summary: dict[str, Any]) -> dict[str, Any]:
    checkpoint = Path(str(summary.get("checkpoint", "")))
    report_path = checkpoint.parent / "exploration_configuration_report.json"
    if not report_path.is_file():
        return {"sigma": [0.0, 0.0], "mean_parameters_exact": None, "ppo_updates": None}
    report = _load(report_path)
    config = report.get("configuration") or {}
    return {
        "sigma": config.get("sigma", [0.0, 0.0]),
        "mean_parameters_exact": config.get("mean_parameters_exact"),
        "ppo_updates": report.get("ppo_updates"),
    }


def _deterministic_trace_for(run_dir: Path, experiment: Path) -> list[dict[str, Any]] | None:
    name = run_dir.name
    historical = (
        experiment.parent
        / "v26c_july_replica_2026-08-26"
        / "j19b_runs"
        / "j19b_closed_loop_v26c_2026-08-27_r1"
    )
    if "minus0p20" in name:
        baseline = experiment / "rollouts" / "deterministic_seed123_minus0p20"
        fallback = historical / "j19b_cell_B_trace.json"
    elif "plus0p20" in name:
        baseline = experiment / "rollouts" / "deterministic_seed123_plus0p20"
        fallback = historical / "j19b_cell_C_trace.json"
    else:
        baseline = experiment / "rollouts" / "deterministic_seed123_nominal"
        fallback = historical / "j19b_cell_A_trace.json"
    trace_path = baseline / "rollout_policy_trace.json"
    if trace_path.is_file():
        return _load(trace_path)
    return _load(fallback) if fallback.is_file() else None


def analyze_run(run_dir: Path, experiment: Path) -> dict[str, Any]:
    summary = _load(run_dir / "rollout_summary.json")
    trace_path = run_dir / "rollout_policy_trace.json"
    rows: list[dict[str, Any]] = _load(trace_path) if trace_path.is_file() else []
    actor = _actor_configuration(summary)
    sigma = [float(value) for value in actor["sigma"]]

    normalized_noise = [[], []]
    physical_noise = [[], []]
    slew_noise = [[], []]
    slew_residual = [[], []]
    slew_intervened = [0, 0]
    valid_slew_rows = 0
    for row in rows:
        noise = row.get("exploration_action_noise")
        mean = row.get("policy_action_mean")
        raw = row.get("raw_policy_action")
        if mean is None and summary.get("action_selection") == "deterministic":
            mean = raw
        obs = row.get("actor_observation_before") or {}
        segments = row.get("policy_segment_values") or []
        if noise is not None and len(noise) == 2:
            for i in range(2):
                normalized_noise[i].append(float(noise[i]))
                physical_noise[i].append(float(noise[i]) * ACTION_SCALE[i])
        if not (mean and raw and segments and len(mean) == len(raw) == 2):
            continue
        anchor = [
            obs.get("pros_knee_angle_previous_endpoint"),
            obs.get("pros_ankle_angle_previous_endpoint"),
        ]
        if any(value is None for value in anchor):
            continue
        mean_target = _physical(mean)
        raw_target = _physical(raw)
        mean_slew = _slew(mean_target, [float(value) for value in anchor])
        raw_slew = _slew(raw_target, [float(value) for value in anchor])
        logged_slew = [float(value) for value in segments[-1]]
        for i in range(2):
            slew_noise[i].append(raw_slew[i] - mean_slew[i])
            slew_residual[i].append(logged_slew[i] - raw_slew[i])
            if abs(raw_slew[i] - raw_target[i]) > 1e-9:
                slew_intervened[i] += 1
        valid_slew_rows += 1

    deterministic = _deterministic_trace_for(run_dir, experiment)
    served_divergence = [[], []]
    physical_divergence = [[], []]
    if deterministic and summary.get("action_selection") != "deterministic":
        for stochastic_row, deterministic_row in zip(rows, deterministic):
            stochastic_state = stochastic_row.get("prosthetic_state") or {}
            deterministic_state = deterministic_row.get("prosthetic_state") or {}
            for i, joint in enumerate(JOINTS):
                served_key = f"pros_{joint}_angle_served_ref"
                state_key = f"pros_{joint}_angle"
                if served_key in stochastic_state and served_key in deterministic_state:
                    served_divergence[i].append(
                        float(stochastic_state[served_key]) - float(deterministic_state[served_key])
                    )
                if state_key in stochastic_state and state_key in deterministic_state:
                    physical_divergence[i].append(
                        float(stochastic_state[state_key]) - float(deterministic_state[state_key])
                    )

    reward_rows = [row.get("reward_terms") or {} for row in rows]
    governor_target_error = [[], []]
    for row in rows:
        values = (row.get("reference_governor_diagnostics") or {}).get(
            "target_error_rms_rad"
        )
        if isinstance(values, list) and len(values) == 2:
            for i in range(2):
                governor_target_error[i].append(values[i])
    penetration = _finite(block.get("grf_penetration_m") for block in reward_rows)
    knee_q = _finite((row.get("prosthetic_state") or {}).get("pros_knee_angle") for row in rows)
    ankle_q = _finite((row.get("prosthetic_state") or {}).get("pros_ankle_angle") for row in rows)
    knee_min = min(knee_q) if knee_q else None
    knee_max = max(knee_q) if knee_q else None
    ankle_min = min(ankle_q) if ankle_q else None
    ankle_max = max(ankle_q) if ankle_q else None
    kinematic_pass = bool(
        knee_q
        and ankle_q
        and ankle_min is not None
        and ankle_min <= -0.03
        and ankle_max - ankle_min >= 0.30
        and knee_max - knee_min >= 0.60
        and all(-1.5 <= value < 0.0 for value in knee_q)
        and all(-0.7 <= value <= 0.7 for value in ankle_q)
    )
    metric: dict[str, Any] = {
        "run": run_dir.name,
        "action_selection": summary.get("action_selection"),
        "seed": summary.get("action_seed"),
        "start_offset_s": summary.get("episode_start_offset_s"),
        "sigma_knee": sigma[0],
        "sigma_ankle": sigma[1],
        "mean_parameters_exact": actor["mean_parameters_exact"],
        "ppo_updates": actor["ppo_updates"],
        "ok": summary.get("ok"),
        "steps": summary.get("steps"),
        "end_reason": summary.get("end_reason"),
        "terminated": summary.get("terminated"),
        "truncated": summary.get("truncated"),
        "episode_return": summary.get("episode_return"),
        "reward_mean": summary.get("reward_mean"),
        "reward_min": summary.get("reward_min"),
        "reward_max": summary.get("reward_max"),
        "action_abs_max": summary.get("action_abs_max"),
        "pelvis_ty_min_m": summary.get("pelvis_ty_min"),
        "reserve_norm_max_nm": summary.get("reserve_norm_max_nm"),
        "penetration_max_mm": 1000.0 * float(summary.get("grf_penetration_max_m") or 0.0),
        "penetration_steps_gt20": sum(value > 0.020 for value in penetration),
        "penetration_steps_ge25": sum(value >= 0.025 for value in penetration),
        "penetration_steps_gt28": sum(value > 0.028 for value in penetration),
        "valid_hs": summary.get("phase_valid_hs_count"),
        "valid_to": summary.get("phase_valid_to_count"),
        "valid_cycles": summary.get("phase_valid_cycle_count"),
        "invalid_events": summary.get("invalid_event_count"),
        "telemetry_hs_to_coherent": bool(
            (summary.get("phase_valid_hs_count") or 0) >= (summary.get("phase_valid_cycle_count") or 0)
            and (summary.get("phase_valid_to_count") or 0) >= (summary.get("phase_valid_cycle_count") or 0)
        ),
        "knee_min_rad": knee_min,
        "knee_max_rad": knee_max,
        "knee_amplitude_rad": knee_max - knee_min if knee_q else None,
        "ankle_min_rad": ankle_min,
        "ankle_max_rad": ankle_max,
        "ankle_amplitude_rad": ankle_max - ankle_min if ankle_q else None,
        "historical_kinematic_gate_pass": kinematic_pass,
        "action_clipped_fraction": summary.get("action_clipped_fraction"),
        "phase_timeout_stance": _timeout_rows(rows, 1.0),
        "phase_timeout_swing": _timeout_rows(rows, 2.0),
        "phase_stance_timeout_loss_max": _max_numeric(
            rows, "reward_terms", ("phase_stance_timeout_loss",)
        ),
        "phase_swing_timeout_loss_max": _max_numeric(
            rows, "reward_terms", ("phase_swing_timeout_loss",)
        ),
        "resync_count": _max_numeric(rows, "phase_fsm", ("resync_count",)),
        "hs_cancelled_count": _max_numeric(rows, "phase_fsm", ("hs_cancelled_count",)),
        "morphology_contract_failure": sum(
            float(block.get("morphology_causal_failed_closed", 0.0) or 0.0) > 0.0
            for block in reward_rows
        ),
        "morphology_knee_excursion_max_rad": _max_numeric(
            rows, "reward_terms", ("morphology_knee_excursion_rad", "morphology_hard_knee_excursion_rad")
        ),
        "morphology_ankle_excursion_max_rad": _max_numeric(
            rows, "reward_terms", ("morphology_ankle_excursion_rad", "morphology_hard_ankle_excursion_rad")
        ),
        "morphology_knee_loss_mean": _mean(
            block.get("morphology_knee_loss") for block in reward_rows
        ),
        "morphology_ankle_loss_mean": _mean(
            block.get("morphology_ankle_loss") for block in reward_rows
        ),
        "sea_tau_input_saturation_fraction_max": _max_numeric(
            rows, "reward_terms", ("sea_tau_input_saturation_fraction",)
        ),
        "sea_knee_tau_input_abs_max_nm": _max_numeric(
            rows, "reward_terms", ("pros_knee_angle_sea_tau_input_abs_max_nm",)
        ),
        "sea_ankle_tau_input_abs_max_nm": _max_numeric(
            rows, "reward_terms", ("pros_ankle_angle_sea_tau_input_abs_max_nm",)
        ),
        "sea_knee_motor_accel_abs_max_rad_s2": _max_numeric(
            rows, "reward_terms", ("pros_knee_angle_sea_motor_accel_abs_max_rad_s2",)
        ),
        "sea_ankle_motor_accel_abs_max_rad_s2": _max_numeric(
            rows, "reward_terms", ("pros_ankle_angle_sea_motor_accel_abs_max_rad_s2",)
        ),
        "sea_u_abs_max": _max_numeric(rows, "reward_terms", ("u_abs_max",)),
        "governor_velocity_limit_fraction_mean": _mean(
            (row.get("reference_governor_diagnostics") or {}).get("velocity_limit_fraction") for row in rows
        ),
        "governor_acceleration_limit_fraction_mean": _mean(
            (row.get("reference_governor_diagnostics") or {}).get("acceleration_limit_fraction") for row in rows
        ),
        "governor_jerk_limit_fraction_mean": _mean(
            (row.get("reference_governor_diagnostics") or {}).get("jerk_limit_fraction") for row in rows
        ),
    }
    for i, joint in enumerate(JOINTS):
        raw_rms = _rms(physical_noise[i])
        survived_rms = _rms(slew_noise[i])
        metric.update(
            {
                f"noise_normalized_{joint}_rms": _rms(normalized_noise[i]),
                f"noise_physical_{joint}_rms_rad": raw_rms,
                f"noise_after_slew_{joint}_rms_rad": survived_rms,
                f"noise_slew_pass_ratio_{joint}": (
                    survived_rms / raw_rms if raw_rms and survived_rms is not None else None
                ),
                f"slew_intervention_{joint}_fraction": (
                    slew_intervened[i] / valid_slew_rows if valid_slew_rows else None
                ),
                f"slew_reconstruction_{joint}_max_abs_error_rad": _max(
                    abs(value) for value in slew_residual[i]
                ),
                f"served_vs_deterministic_{joint}_rms_rad": _rms(served_divergence[i]),
                f"physical_vs_deterministic_{joint}_rms_rad": _rms(physical_divergence[i]),
                f"governor_target_error_{joint}_rms_rad": _rms(
                    governor_target_error[i]
                ),
                f"governor_target_error_{joint}_max_rad": _max(
                    governor_target_error[i]
                ),
            }
        )
    metric["nominal_smoke_pass"] = bool(
        metric["steps"] == 500
        and (metric["valid_cycles"] or 0) >= 2
        and metric["penetration_max_mm"] <= 28.0
        and (metric["action_clipped_fraction"] or 0.0) == 0.0
        and metric["invalid_events"] == 0
        and metric["telemetry_hs_to_coherent"]
        and metric["historical_kinematic_gate_pass"]
        and metric["phase_timeout_stance"] == 0.0
        and metric["phase_timeout_swing"] == 0.0
        and metric["resync_count"] == 0.0
        and metric["hs_cancelled_count"] == 0.0
        and metric["morphology_contract_failure"] == 0.0
    )
    return metric


def _write_csv(path: Path, runs: list[dict[str, Any]]) -> None:
    fields = sorted({key for run in runs for key in run})
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(runs)


def _group_isotropic(
    runs: list[dict[str, Any]], nominal_start_s: float
) -> list[dict[str, Any]]:
    groups: dict[float, list[dict[str, Any]]] = {}
    for run in runs:
        if (
            run["action_selection"] == "deterministic"
            or run["sigma_knee"] != run["sigma_ankle"]
            or not math.isclose(
                float(run["start_offset_s"]), nominal_start_s, rel_tol=0.0, abs_tol=1e-9
            )
        ):
            continue
        groups.setdefault(run["sigma_knee"], []).append(run)
    result = []
    for sigma, members in sorted(groups.items()):
        completed = [member for member in members if member["steps"] == 500]
        passed = [member for member in members if member["nominal_smoke_pass"]]
        result.append(
            {
                "sigma": sigma,
                "n": len(members),
                "completed_500": len(completed),
                "passes": len(passed),
                "all_pass": all(member["nominal_smoke_pass"] for member in members),
                "failed_seeds": [
                    member["seed"] for member in members if not member["nominal_smoke_pass"]
                ],
                "penetration_max_mm": _max(member["penetration_max_mm"] for member in members),
                "penetration_seeds_ge25": sum(
                    member["penetration_steps_ge25"] > 0 for member in members
                ),
                "return_mean": _mean(member["episode_return"] for member in members),
                "return_mean_completed_500": _mean(
                    member["episode_return"] for member in completed
                ),
                "return_min": min(member["episode_return"] for member in members),
                "reward_min": min(member["reward_min"] for member in members),
                "knee_after_slew_rms_rad_mean": _mean(
                    member["noise_after_slew_knee_rms_rad"] for member in members
                ),
                "ankle_after_slew_rms_rad_mean": _mean(
                    member["noise_after_slew_ankle_rms_rad"] for member in members
                ),
                "knee_slew_pass_ratio_mean": _mean(
                    member["noise_slew_pass_ratio_knee"] for member in members
                ),
                "ankle_slew_pass_ratio_mean": _mean(
                    member["noise_slew_pass_ratio_ankle"] for member in members
                ),
                "knee_slew_intervention_fraction_mean": _mean(
                    member["slew_intervention_knee_fraction"] for member in members
                ),
                "ankle_slew_intervention_fraction_mean": _mean(
                    member["slew_intervention_ankle_fraction"] for member in members
                ),
                "knee_governor_target_error_rms_rad_mean": _mean(
                    member["governor_target_error_knee_rms_rad"] for member in members
                ),
                "ankle_governor_target_error_rms_rad_mean": _mean(
                    member["governor_target_error_ankle_rms_rad"] for member in members
                ),
                "knee_served_divergence_rms_rad_mean": _mean(
                    member["served_vs_deterministic_knee_rms_rad"] for member in members
                ),
                "ankle_served_divergence_rms_rad_mean": _mean(
                    member["served_vs_deterministic_ankle_rms_rad"] for member in members
                ),
                "knee_morphology_loss_mean": _mean(
                    member["morphology_knee_loss_mean"] for member in members
                ),
                "ankle_morphology_loss_mean": _mean(
                    member["morphology_ankle_loss_mean"] for member in members
                ),
            }
        )
    return result


def _group_multistart(
    runs: list[dict[str, Any]], nominal_start_s: float
) -> list[dict[str, Any]]:
    groups: dict[float, list[dict[str, Any]]] = {}
    for run in runs:
        if (
            run["action_selection"] == "deterministic"
            or run["sigma_knee"] != run["sigma_ankle"]
            or math.isclose(
                float(run["start_offset_s"]), nominal_start_s, rel_tol=0.0, abs_tol=1e-9
            )
        ):
            continue
        groups.setdefault(run["sigma_knee"], []).append(run)
    result = []
    for sigma, members in sorted(groups.items()):
        result.append(
            {
                "sigma": sigma,
                "n": len(members),
                "passes": sum(member["nominal_smoke_pass"] for member in members),
                "all_pass": all(member["nominal_smoke_pass"] for member in members),
                "starts": [
                    {
                        "start_offset_s": member["start_offset_s"],
                        "pass": member["nominal_smoke_pass"],
                        "steps": member["steps"],
                        "penetration_max_mm": member["penetration_max_mm"],
                        "valid_cycles": member["valid_cycles"],
                        "episode_return": member["episode_return"],
                    }
                    for member in sorted(members, key=lambda item: item["start_offset_s"])
                ],
            }
        )
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("experiment", type=Path)
    args = parser.parse_args()
    experiment = args.experiment.resolve()
    protocol = _load(experiment / "protocol.json")
    nominal_start_s = float(protocol["invariants"]["episode_start_nominal_s"])
    run_dirs = sorted(
        path.parent for path in (experiment / "rollouts").glob("*/rollout_summary.json")
    )
    runs = [analyze_run(path, experiment) for path in run_dirs]
    grouped = _group_isotropic(runs, nominal_start_s)
    multistart = _group_multistart(runs, nominal_start_s)
    payload = {
        "experiment": str(experiment),
        "run_count": len(runs),
        "runs": runs,
        "isotropic": grouped,
        "multi_start": multistart,
    }
    with (experiment / "analysis.json").open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, allow_nan=False)
        handle.write("\n")
    _write_csv(experiment / "run_metrics.csv", runs)
    print(json.dumps(payload, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
