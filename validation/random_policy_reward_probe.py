"""Random-policy baseline for the ex-novo reward validation plan.

This is not a trained-policy rollout.  It builds the same training environment
from ``training_exnovo_cfg.yaml``, samples random actions from the advertised
action space, and records reward/FSM diagnostics for the F1 gate.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from collections.abc import Mapping
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np


REPO = Path(__file__).resolve().parents[1]
TRAJECTORY_DIR = REPO / "Trajectory Generator"
BASELINE_DIR = TRAJECTORY_DIR / "baseline_MLP"
DEFAULT_CONFIG = BASELINE_DIR / "training_exnovo_cfg.yaml"
DEFAULT_OUTPUT_ROOT = REPO / "validation" / "reward_policy_runs"
PRESCRIBED_ALIGNED_REWARD_MEAN = 0.12663565886713068
PRESCRIBED_SIMILARITY_THRESHOLD = 0.70 * PRESCRIBED_ALIGNED_REWARD_MEAN

for candidate in (REPO, TRAJECTORY_DIR, BASELINE_DIR):
    text = str(candidate)
    if text not in sys.path:
        sys.path.insert(0, text)

import env_factory  # noqa: E402
import reward_function  # noqa: E402
import training_config  # noqa: E402


def _cli_path(value: str | os.PathLike[str]) -> Path:
    text = os.fspath(value)
    if os.name != "nt":
        text = text.replace("\\", "/")
    return Path(text).expanduser()


def _resolve_repo_path(value: str | os.PathLike[str]) -> Path:
    path = _cli_path(value)
    if not path.is_absolute():
        path = REPO / path
    return path.resolve()


def _resolve_output_dir(value: str | None) -> Path:
    if value:
        return _resolve_repo_path(value)
    return (
        DEFAULT_OUTPUT_ROOT
        / f"{datetime.now():%Y-%m-%d_%H%M%S}_random_policy"
    ).resolve()


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return [_jsonable(item) for item in value.tolist()]
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def _finite_float(value: Any) -> float | None:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def _numeric_stats(rows: list[Mapping[str, Any]]) -> dict[str, dict[str, float]]:
    keys = sorted({str(key) for row in rows for key in row})
    result: dict[str, dict[str, float]] = {}
    for key in keys:
        values: list[float] = []
        for row in rows:
            value = _finite_float(row.get(key))
            if value is not None:
                values.append(value)
        if values:
            arr = np.asarray(values, dtype=float)
            result[key] = {
                "mean": float(np.mean(arr)),
                "min": float(np.min(arr)),
                "max": float(np.max(arr)),
                "last": float(arr[-1]),
            }
    return result


def _write_csv(path: Path, rows: list[Mapping[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    keys = sorted({str(key) for row in rows for key in row})
    if not keys:
        keys = ["status"]
        rows = [{"status": ""}]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: _jsonable(row.get(key, "")) for key in keys})


def _load_config(args: argparse.Namespace) -> tuple[Path, dict[str, Any], dict[str, Any]]:
    config_path = training_config.resolve_config_path(args.config)
    cfg = training_config.load(config_path)
    flat, reward_overrides = training_config.to_argparse_defaults(cfg)
    if args.reward_json:
        reward_overrides.update(
            reward_function.load_reward_overrides(args.reward_json) or {}
        )
    return config_path, flat, reward_overrides


def _env_config(
    flat: Mapping[str, Any],
    reward_overrides: Mapping[str, Any],
    output_dir: Path,
    args: argparse.Namespace,
) -> dict[str, Any]:
    record_outputs = bool(args.record_outputs)
    return {
        "setup_xml_path": flat.get(
            "setup_xml",
            "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml",
        ),
        "segment_duration": float(flat.get("segment_duration", 0.01)),
        "episode_duration": float(
            args.episode_duration
            if args.episode_duration is not None
            else flat.get("episode_duration", 5.0)
        ),
        "policy_knots": int(flat.get("policy_knots", 1)),
        "action_mode": str(flat.get("action_mode", "absolute")),
        "max_delta_rad": float(flat.get("max_delta_rad", 0.35)),
        "enable_pros_ref_governor": bool(flat.get("pros_ref_governor", True)),
        "pros_ref_model": str(
            flat.get("pros_ref_model", "butterworth3_jerk_limited")
        ),
        "pros_ref_lpf_cutoff_hz": float(flat.get("pros_ref_cutoff_hz", 4.0)),
        "pros_ref_velocity_limit_rad_s": {
            "pros_knee_angle": float(
                flat.get("pros_knee_ref_velocity_limit_rad_s", 6.0)
            ),
            "pros_ankle_angle": float(
                flat.get("pros_ankle_ref_velocity_limit_rad_s", 3.5)
            ),
        },
        "pros_ref_acceleration_limit_rad_s2": {
            "pros_knee_angle": float(
                flat.get("pros_knee_ref_acceleration_limit_rad_s2", 60.0)
            ),
            "pros_ankle_angle": float(
                flat.get("pros_ankle_ref_acceleration_limit_rad_s2", 55.0)
            ),
        },
        "pros_ref_jerk_limit_rad_s3": {
            "pros_knee_angle": float(
                flat.get("pros_knee_ref_jerk_limit_rad_s3", 3000.0)
            ),
            "pros_ankle_angle": float(
                flat.get("pros_ankle_ref_jerk_limit_rad_s3", 2750.0)
            ),
        },
        "gait_clock_enable": bool(flat.get("gait_clock_enable", False)),
        "actor_cyclic_phase_only": bool(flat.get("actor_cyclic_phase_only", True)),
        "include_reference_state_observation": bool(
            flat.get("include_reference_state_observation", True)
        ),
        "deployable_minimal_observation": bool(
            flat.get("deployable_minimal_observation", False)
        ),
        "imitation_initialize_to_target": bool(
            flat.get("imitation_initialize_to_target", False)
        ),
        "reward_reference_range_floor": float(
            flat.get("reward_reference_range_floor", 0.05)
        ),
        "reward_reference_velocity_range_floor": float(
            flat.get("reward_reference_velocity_range_floor", 0.1)
        ),
        "random_init": bool(flat.get("random_init", False)),
        "episode_start_offset_s": float(flat.get("episode_start_offset_s", 0.0)),
        "rebuild_model_on_reset": False,
        "record_outputs": record_outputs,
        "save_outputs_on_close": record_outputs,
        "output_dir": str(output_dir / "sim_outputs"),
        "output_prefix": "random_policy",
        "fail_fast": True,
        "grf_mode": str(flat.get("grf_mode", "online_sensor")),
        "online_grf_profile_file": str(flat.get("online_grf_profile", "")),
        "online_grf_detector_profile_file": flat.get("online_grf_detector_profile"),
        "include_online_grf_observation": bool(
            flat.get("online_grf_observation", True)
        ),
        "critic_privileged_observation": bool(
            flat.get("asymmetric_actor_critic", True)
        ),
        "prescribed_grf_disabled_sides": list(
            flat.get("disable_prescribed_grf_side", []) or []
        ),
        "online_grf_applied_sides": list(
            flat.get("online_grf_applied_side", []) or []
        ),
        "step_wall_timeout_s": float(flat.get("step_wall_timeout_s", 60.0)),
        "grf_penetration_penalty_threshold_m": float(
            flat.get("grf_penetration_penalty_threshold_m", 0.012)
        ),
        "grf_penetration_termination_m": float(
            flat.get("grf_penetration_termination_m", 0.017)
        ),
        "reward": dict(reward_overrides),
    }


def _trace_row(
    step: int,
    reward: float,
    terminated: bool,
    truncated: bool,
    info: Mapping[str, Any],
) -> dict[str, Any]:
    terms = info.get("reward_terms", {})
    if not isinstance(terms, Mapping):
        terms = {}
    components = info.get("reward_components", {})
    if not isinstance(components, Mapping):
        components = {}
    fsm = info.get("phase_fsm", {})
    if not isinstance(fsm, Mapping):
        fsm = {}
    return {
        "step": step,
        "time": float(info.get("time", float("nan"))),
        "reward": float(reward),
        "terminated": float(terminated),
        "truncated": float(truncated),
        "end_reason": info.get("end_reason") or "",
        "phase_valid_hs_count": terms.get("phase_valid_hs_count", 0.0),
        "phase_valid_to_count": terms.get("phase_valid_to_count", 0.0),
        "phase_valid_cycle_count": terms.get("phase_valid_cycle_count", 0.0),
        "phase_timeout_loss": terms.get("phase_timeout_loss", 0.0),
        "phase_timeout_penalty_term": components.get(
            "phase_timeout_penalty_term", 0.0
        ),
        "phase_event_progress_score": terms.get(
            "phase_event_progress_score", 0.0
        ),
        "phase_cycle_complete_bonus": terms.get(
            "phase_cycle_complete_bonus", 0.0
        ),
        "phase_cycle_rejected_this_step": terms.get(
            "phase_cycle_rejected_this_step", 0.0
        ),
        "invalid_event_count": terms.get("invalid_event_count", 0.0),
        "fsm_state_name": fsm.get("state_name", ""),
        "reward_base": components.get("reward_base", 0.0),
        "oob_term": components.get("oob_term", 0.0),
        "prosthetic_joint_range_term": components.get(
            "prosthetic_joint_range_term", 0.0
        ),
        "policy_action_clip_loss": terms.get("policy_action_clip_loss", 0.0),
    }


def _write_summary_md(path: Path, summary: Mapping[str, Any]) -> None:
    criteria = summary.get("pass_criteria", {})
    metrics = summary.get("metrics", {})
    lines = [
        "# Random Policy Reward Probe",
        "",
        f"- Status: {summary.get('status', 'UNKNOWN')}",
        f"- Output: `{summary.get('output_dir', '')}`",
        "",
        "## Criteria",
        "",
    ]
    if isinstance(criteria, Mapping):
        lines.extend(
            f"- {key}: {'PASS' if bool(value) else 'FAIL'}"
            for key, value in sorted(criteria.items())
        )
    lines.extend(["", "## Metrics", ""])
    if isinstance(metrics, Mapping):
        lines.extend(f"- {key}: {value}" for key, value in sorted(metrics.items()))
    lines.extend(["", "## Files", "", "- `summary.json`", "- `trace.csv`"])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(args: argparse.Namespace) -> dict[str, Any]:
    output_dir = _resolve_output_dir(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    config_path, flat, reward_overrides = _load_config(args)
    env_config = _env_config(flat, reward_overrides, output_dir, args)
    env = env_factory.make_cmc_env(env_config)
    rng = np.random.default_rng(args.seed)
    trace_rows: list[dict[str, Any]] = []
    rewards: list[float] = []
    nan_count = 0
    terminated = False
    truncated = False
    final_info: Mapping[str, Any] = {}
    try:
        obs, reset_info = env.reset(seed=args.seed)
        if not np.all(np.isfinite(obs)):
            nan_count += 1
        final_info = reset_info
        max_steps = int(
            args.max_steps
            if args.max_steps is not None
            else math.ceil(float(env_config["episode_duration"]) / float(env_config["segment_duration"])) + 2
        )
        for step in range(1, max_steps + 1):
            action = rng.uniform(env.action_space.low, env.action_space.high).astype(
                np.float32
            )
            obs, reward, terminated, truncated, info = env.step(action)
            if not np.all(np.isfinite(obs)) or not math.isfinite(float(reward)):
                nan_count += 1
            final_info = info
            rewards.append(float(reward))
            trace_rows.append(_trace_row(step, reward, terminated, truncated, info))
            if terminated or truncated:
                break
    finally:
        env.close()

    stats = _numeric_stats(trace_rows)
    final_fsm = final_info.get("phase_fsm", {}) if isinstance(final_info, Mapping) else {}
    if not isinstance(final_fsm, Mapping):
        final_fsm = {}
    reward_mean = float(np.mean(rewards)) if rewards else 0.0
    metrics = {
        "steps": float(len(rewards)),
        "episode_return": float(np.sum(rewards)) if rewards else 0.0,
        "reward_mean": reward_mean,
        "reward_min": float(np.min(rewards)) if rewards else 0.0,
        "reward_max": float(np.max(rewards)) if rewards else 0.0,
        "prescribed_aligned_reward_mean": PRESCRIBED_ALIGNED_REWARD_MEAN,
        "random_similarity_threshold_70pct": PRESCRIBED_SIMILARITY_THRESHOLD,
        "valid_cycle_count_final": float(
            final_fsm.get("valid_cycle_count", 0.0) or 0.0
        ),
        "invalid_event_count_final": float(
            final_fsm.get("invalid_event_count", 0.0) or 0.0
        ),
        "phase_timeout_penalty_term_mean": stats.get(
            "phase_timeout_penalty_term",
            {},
        ).get("mean", 0.0),
        "nan_count": float(nan_count),
    }
    criteria = {
        "no_nan": nan_count == 0,
        "completed_or_cleanly_truncated": bool(terminated or truncated),
        "random_below_70pct_prescribed": reward_mean
        < PRESCRIBED_SIMILARITY_THRESHOLD,
    }
    status = "PASS" if all(criteria.values()) else "FAIL"
    summary = {
        "ok": status == "PASS",
        "test_id": "random_policy_baseline",
        "status": status,
        "output_dir": str(output_dir),
        "config": str(config_path),
        "seed": int(args.seed),
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "end_reason": final_info.get("end_reason") if isinstance(final_info, Mapping) else None,
        "pass_criteria": criteria,
        "metrics": metrics,
        "trace_stats": stats,
        "final_fsm": _jsonable(final_fsm),
        "env_config": _jsonable(env_config),
        "reward_config": reward_function.RewardConfig.from_mapping(
            reward_overrides
        ).to_dict(),
    }
    _write_csv(output_dir / "trace.csv", trace_rows)
    (output_dir / "summary.json").write_text(
        json.dumps(_jsonable(summary), indent=2),
        encoding="utf-8",
    )
    _write_summary_md(output_dir / "summary.md", summary)
    print(json.dumps(_jsonable(summary), indent=2))
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--reward-json", default=None)
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--episode-duration", type=float, default=None)
    parser.add_argument("--max-steps", type=int, default=None)
    parser.add_argument("--seed", type=int, default=123)
    parser.add_argument("--record-outputs", action="store_true")
    return parser.parse_args()


if __name__ == "__main__":
    run(parse_args())
