"""Offline audit scenarios for the ex-novo reward contract.

The suite is intentionally runnable without a trained policy.  Prescribed gait
checks delegate to ``prescribed_reward_probe.py``; synthetic counterexamples use
the reward and prosthetic phase FSM directly so they stay fast and deterministic.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import subprocess
import sys
from collections.abc import Mapping
from dataclasses import replace
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np


REPO = Path(__file__).resolve().parents[1]
TRAJECTORY_DIR = REPO / "Trajectory Generator"
BASELINE_DIR = TRAJECTORY_DIR / "baseline_MLP"
DEFAULT_CONFIG = BASELINE_DIR / "training_exnovo_cfg.yaml"
DEFAULT_OUTPUT_ROOT = REPO / "validation" / "reward_audit_runs"

for candidate in (REPO, TRAJECTORY_DIR, BASELINE_DIR):
    text = str(candidate)
    if text not in sys.path:
        sys.path.insert(0, text)

import reward_function  # noqa: E402
import training_config  # noqa: E402
from prosthetic_phase_fsm import ProstheticPhaseFSM, ProstheticPhaseFSMConfig  # noqa: E402


SCENARIOS = (
    "prescribed_aligned",
    "prescribed_long",
    "prescribed_misaligned",
    "static_leg",
    "compressive_stance",
    "missing_to",
    "missing_second_hs",
    "swing_load",
    "joint_oob",
    "slip_injection",
    "morphology_corridor",
    "fake_cycle_ankle_only",
)

PRESCRIBED_SCENARIOS = {
    "prescribed_aligned": {
        "start_time": 13.946870983805102,
        "end_time": 17.99,
        "gate_role": "positive_gate",
    },
    "prescribed_long": {
        "start_time": 13.946870983805102,
        "end_time": 21.0,
        "gate_role": "positive_long_gate",
    },
    "prescribed_misaligned": {
        "start_time": 12.99,
        "end_time": 17.99,
        "gate_role": "diagnostic_edge_start",
    },
}


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


def _resolve_config_path(value: str | os.PathLike[str] | None) -> Path:
    if value is None:
        return DEFAULT_CONFIG.resolve()
    path = _cli_path(value)
    if path.is_absolute():
        return path.resolve()
    cwd_candidate = (Path.cwd() / path).resolve()
    if cwd_candidate.is_file():
        return cwd_candidate
    baseline_candidate = (BASELINE_DIR / path).resolve()
    if baseline_candidate.is_file():
        return baseline_candidate
    return (REPO / path).resolve()


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


def _stat(
    summary: Mapping[str, Any],
    section: str,
    key: str,
    field: str = "mean",
    default: float = 0.0,
) -> float:
    try:
        value = summary[section][key][field]
    except Exception:
        return float(default)
    finite = _finite_float(value)
    return float(default if finite is None else finite)


def _load_reward_config(
    config_arg: str | os.PathLike[str] | None,
    reward_json: str | None,
) -> tuple[Path, dict[str, Any], reward_function.RewardConfig]:
    config_path = _resolve_config_path(config_arg)
    cfg = training_config.load(config_path)
    flat, reward_overrides = training_config.to_argparse_defaults(cfg)
    if reward_json:
        reward_overrides.update(
            reward_function.load_reward_overrides(reward_json) or {}
        )
    return config_path, flat, reward_function.RewardConfig.from_mapping(reward_overrides)


def _phase_config_from_reward(
    reward_cfg: reward_function.RewardConfig,
) -> ProstheticPhaseFSMConfig:
    return ProstheticPhaseFSMConfig(
        min_stance_duration_s=float(reward_cfg.phase_min_stance_duration_s),
        min_swing_duration_s=float(reward_cfg.phase_min_swing_duration_s),
        landing_window_start_s=float(reward_cfg.phase_landing_window_start_s),
        landing_window_end_s=float(reward_cfg.phase_landing_window_end_s),
        stance_hard_timeout_s=float(reward_cfg.phase_stance_hard_timeout_s),
        swing_hard_timeout_s=float(reward_cfg.phase_swing_hard_timeout_s),
        landing_force_full_credit_bw=float(reward_cfg.contact_load_target_bw),
        min_stance_contact_fraction=float(
            reward_cfg.phase_min_stance_contact_fraction
        ),
        min_stance_load_bw_s=float(reward_cfg.phase_min_stance_load_bw_s),
        min_cycle_knee_excursion_rad=float(
            reward_cfg.phase_min_cycle_knee_excursion_rad
        ),
        hs_event_credit=float(reward_cfg.phase_hs_event_credit),
        toe_off_event_credit=float(reward_cfg.phase_to_event_credit),
        cycle_complete_bonus=float(reward_cfg.phase_cycle_complete_bonus),
        failure_extra_penalty=float(reward_cfg.phase_failure_extra_penalty),
    )


def _score(loss: float, weight: float) -> float:
    return 1.0 / (1.0 + float(weight) * float(loss))


def _huber_loss(value: float, delta: float = 1.0, cap: float = 25.0) -> float:
    delta = max(1e-9, abs(float(delta)))
    x = abs(float(value))
    if x <= delta:
        loss = 0.5 * x * x
    else:
        loss = delta * (x - 0.5 * delta)
    return float(min(cap, loss))


def _huber_interval_loss(
    value: float,
    low: float,
    high: float,
    scale: float,
    *,
    cap: float = 25.0,
) -> float:
    low_f = float(min(low, high))
    high_f = float(max(low, high))
    value_f = float(value)
    scale_f = max(1e-9, abs(float(scale)))
    if value_f < low_f:
        return _huber_loss((low_f - value_f) / scale_f, cap=cap)
    if value_f > high_f:
        return _huber_loss((value_f - high_f) / scale_f, cap=cap)
    return 0.0


def _phase_terms_from_fsm(
    fsm_payload: Mapping[str, Any],
    reward_cfg: reward_function.RewardConfig,
) -> dict[str, float]:
    period_soft_low = (
        float(reward_cfg.phase_period_nominal_s)
        - float(reward_cfg.phase_period_soft_margin_s)
    )
    period_soft_high = (
        float(reward_cfg.phase_period_nominal_s)
        + float(reward_cfg.phase_period_soft_margin_s)
    )
    event_order_loss = float(fsm_payload.get("invalid_event_loss", 0.0) or 0.0)
    last_period_s = float(fsm_payload.get("last_period_s", 0.0) or 0.0)
    previous_period_s = float(fsm_payload.get("previous_period_s", 0.0) or 0.0)
    last_stance_fraction = float(
        fsm_payload.get("last_stance_fraction", 0.0) or 0.0
    )
    phase_period_loss = (
        _huber_interval_loss(
            last_period_s,
            period_soft_low,
            period_soft_high,
            reward_cfg.phase_period_soft_margin_s,
        )
        if last_period_s > 1e-9
        else 0.0
    )
    phase_period_hard_loss = (
        _huber_interval_loss(
            last_period_s,
            reward_cfg.phase_period_hard_min_s,
            reward_cfg.phase_period_hard_max_s,
            reward_cfg.phase_period_soft_margin_s,
        )
        if last_period_s > 1e-9
        else 0.0
    )
    phase_periodicity_loss = (
        _huber_loss(
            (last_period_s - previous_period_s)
            / max(1e-9, float(reward_cfg.phase_periodicity_scale_s))
        )
        if last_period_s > 1e-9 and previous_period_s > 1e-9
        else 0.0
    )
    stance_fraction_loss = (
        _huber_interval_loss(
            last_stance_fraction,
            reward_cfg.phase_stance_fraction_min,
            reward_cfg.phase_stance_fraction_max,
            max(
                1e-9,
                0.5
                * (
                    float(reward_cfg.phase_stance_fraction_max)
                    - float(reward_cfg.phase_stance_fraction_min)
                ),
            ),
        )
        if last_stance_fraction > 1e-9
        else 0.0
    )
    stance_elapsed_s = float(fsm_payload.get("stance_elapsed_s", 0.0) or 0.0)
    swing_elapsed_s = float(fsm_payload.get("swing_elapsed_s", 0.0) or 0.0)
    stance_timeout_loss = _huber_interval_loss(
        stance_elapsed_s,
        0.0,
        reward_cfg.phase_stance_timeout_s,
        reward_cfg.phase_timeout_scale_s,
    )
    swing_timeout_loss = _huber_interval_loss(
        swing_elapsed_s,
        0.0,
        reward_cfg.phase_swing_timeout_s,
        reward_cfg.phase_timeout_scale_s,
    )
    timeout_loss = float(stance_timeout_loss + swing_timeout_loss)
    phase_regularity_loss = float(
        float(reward_cfg.phase_event_order_weight) * event_order_loss
        + float(reward_cfg.phase_period_weight)
        * (phase_period_loss + phase_period_hard_loss)
        + float(reward_cfg.phase_periodicity_weight) * phase_periodicity_loss
        + float(reward_cfg.phase_stance_fraction_weight) * stance_fraction_loss
        + float(reward_cfg.phase_timeout_weight) * timeout_loss
    )
    valid_cycle_count = float(fsm_payload.get("valid_cycle_count", 0.0) or 0.0)
    phase_regular_score = (
        _score(phase_regularity_loss, reward_cfg.phase_regularity_weight)
        if valid_cycle_count > 0.0
        else 0.0
    )
    return {
        "phase_regular_score": float(phase_regular_score),
        "phase_regularity_loss": float(phase_regularity_loss),
        "phase_event_order_loss": float(event_order_loss),
        "phase_period_loss": float(phase_period_loss),
        "phase_period_hard_loss": float(phase_period_hard_loss),
        "phase_periodicity_loss": float(phase_periodicity_loss),
        "phase_stance_fraction_loss": float(stance_fraction_loss),
        "phase_timeout_loss": float(timeout_loss),
        "phase_stance_timeout_loss": float(stance_timeout_loss),
        "phase_swing_timeout_loss": float(swing_timeout_loss),
        "phase_timeout_exceeded": float(
            fsm_payload.get("timeout_exceeded", 0.0) or 0.0
        ),
        "phase_timeout_side": float(fsm_payload.get("timeout_side", 0.0) or 0.0),
    }


def _reward_terms_from_fsm(
    fsm_payload: Mapping[str, Any],
    *,
    normal_force_bw: float,
    in_contact: bool,
    reward_cfg: reward_function.RewardConfig,
) -> dict[str, float]:
    state_id = int(float(fsm_payload.get("state_id", 0.0) or 0.0))
    stance_expected = 1.0 if state_id == 1 else 0.0
    swing_expected = 1.0 if state_id == 2 else 0.0
    force_bw = max(0.0, float(normal_force_bw))
    contact_load_score = (
        float(
            np.clip(
                force_bw / max(1e-9, float(reward_cfg.contact_load_target_bw)),
                0.0,
                1.0,
            )
        )
        if stance_expected and in_contact
        else 0.0
    )
    swing_unloading_loss = (
        max(0.0, force_bw - float(reward_cfg.swing_unloading_force_tol_bw)) ** 2
        if swing_expected
        else 0.0
    )
    contact_overload_loss = max(
        0.0,
        force_bw - float(reward_cfg.contact_load_max_bw),
    ) ** 2
    phase_terms = _phase_terms_from_fsm(fsm_payload, reward_cfg)
    return {
        reward_function.TRACKING_LOSS: 0.0,
        reward_function.REFERENCE_LOSS: 0.0,
        reward_function.BIO_LOSS: 0.0,
        reward_function.EFFORT_LOSS: 0.0,
        reward_function.SMOOTHNESS_LOSS: 0.0,
        reward_function.SATURATION_LOSS: 0.0,
        reward_function.SAFETY_LOSS: 0.0,
        reward_function.GRF_PENETRATION_LOSS: 0.0,
        "contact_load_score": contact_load_score,
        "contact_load_loss": 1.0 - contact_load_score,
        "contact_overload_loss": float(contact_overload_loss),
        "swing_unloading_loss": float(swing_unloading_loss),
        "grf_slip_loss": 0.0,
        **phase_terms,
        "phase_event_progress_score": float(
            fsm_payload.get("phase_event_progress_score", 0.0) or 0.0
        ),
        "landing_window_contact_score": float(
            fsm_payload.get("landing_window_contact_score", 0.0) or 0.0
        ),
        "invalid_event_loss": float(
            fsm_payload.get("invalid_event_loss", 0.0) or 0.0
        ),
        "contact_validity_loss": float(
            fsm_payload.get("contact_validity_loss", 0.0) or 0.0
        ),
        "phase_clawback_penalty": float(
            fsm_payload.get("phase_clawback_penalty", 0.0) or 0.0
        ),
        "phase_failure_extra_penalty": float(
            fsm_payload.get("phase_failure_extra_penalty", 0.0) or 0.0
        ),
        "phase_cycle_complete_bonus": float(
            fsm_payload.get("phase_cycle_complete_bonus", 0.0) or 0.0
        ),
        "phase_cycle_failed_this_step": float(
            fsm_payload.get("phase_cycle_failed_this_step", 0.0) or 0.0
        ),
        "phase_pending_cycle_credit": float(
            fsm_payload.get("pending_cycle_credit", 0.0) or 0.0
        ),
        "phase_valid_hs_count": float(
            fsm_payload.get("valid_hs_count", 0.0) or 0.0
        ),
        "phase_valid_to_count": float(
            fsm_payload.get("valid_to_count", 0.0) or 0.0
        ),
        "phase_valid_cycle_count": float(
            fsm_payload.get("valid_cycle_count", 0.0) or 0.0
        ),
        "invalid_event_count": float(
            fsm_payload.get("invalid_event_count", 0.0) or 0.0
        ),
        "phase_stance_contact_time_s": float(
            fsm_payload.get("stance_contact_time_s", 0.0) or 0.0
        ),
        "phase_stance_load_integral_bw_s": float(
            fsm_payload.get("stance_load_integral_bw_s", 0.0) or 0.0
        ),
        "phase_stance_contact_fraction": float(
            fsm_payload.get("stance_contact_fraction", 0.0) or 0.0
        ),
        "phase_stance_mean_load_bw": float(
            fsm_payload.get("stance_mean_load_bw", 0.0) or 0.0
        ),
        "phase_cycle_knee_excursion_rad": float(
            fsm_payload.get("cycle_knee_excursion_rad", 0.0) or 0.0
        ),
        "phase_cycle_ankle_excursion_rad": float(
            fsm_payload.get("cycle_ankle_excursion_rad", 0.0) or 0.0
        ),
        "phase_cycle_rejected_this_step": float(
            fsm_payload.get("cycle_rejected_this_step", 0.0) or 0.0
        ),
        "prosthetic_normal_force_bw": float(force_bw),
        "prosthetic_stance_expected": stance_expected,
        "prosthetic_swing_expected": swing_expected,
        "prosthetic_joint_range_loss": 0.0,
        "morphology_loss": 0.0,
        "reserve_residual_loss": 0.0,
        "pelvis_height_loss": 0.0,
    }


def _run_synthetic_sequence(
    *,
    subcase: str,
    samples: list[dict[str, Any]],
    reward_cfg: reward_function.RewardConfig,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], dict[str, Any]]:
    class _SyntheticEnv(reward_function.gym.Env):
        action_space = reward_function.gym.spaces.Box(
            low=-1.0, high=1.0, shape=(1,), dtype=np.float32
        )

    fsm = ProstheticPhaseFSM(_phase_config_from_reward(reward_cfg))
    reward_wrapper = reward_function.RewardShapingWrapper(
        _SyntheticEnv(),
        reward_cfg,
    )
    trace_rows: list[dict[str, Any]] = []
    event_rows: list[dict[str, Any]] = []
    rewards: list[float] = []
    terms_rows: list[dict[str, Any]] = []
    component_rows: list[dict[str, Any]] = []
    for step, sample in enumerate(samples, start=1):
        event_name = sample.get("event")
        events = []
        if event_name:
            event = {
                "side": "left",
                "event": str(event_name),
                "time": float(sample["time"]),
            }
            if sample.get("contact_duration_s") is not None:
                event["contact_duration_s"] = float(sample["contact_duration_s"])
            events.append(event)
            event_rows.append({"subcase": subcase, "step": step, **event})
        force_bw = float(sample.get("normal_force_bw", 0.0) or 0.0)
        in_contact = bool(sample.get("in_contact", False))
        payload = fsm.update(
            time_s=float(sample["time"]),
            events=events,
            normal_force_bw=force_bw,
            in_contact=in_contact,
            prosthetic_knee_angle_rad=sample.get("knee_rad"),
            prosthetic_ankle_angle_rad=sample.get("ankle_rad"),
        )
        terms = _reward_terms_from_fsm(
            payload,
            normal_force_bw=force_bw,
            in_contact=in_contact,
            reward_cfg=reward_cfg,
        )
        overrides = sample.get("term_overrides", {})
        override_terms = (
            {str(key): float(value) for key, value in overrides.items()}
            if isinstance(overrides, Mapping)
            else {}
        )
        last_hs = fsm.last_valid_hs_time
        last_to = fsm.last_valid_to_time
        cycle_duration = float(fsm.last_period_s)
        gait_phase = (
            float(
                np.clip(
                    (float(sample["time"]) - float(last_hs)) / cycle_duration,
                    0.0,
                    1.0,
                )
            )
            if last_hs is not None and cycle_duration > 1e-9
            else 0.0
        )
        task_terms = reward_wrapper._task_reward_terms(
            {
                "online_gait": {
                    "sides": {
                        "left": {
                            "normal_force_bw": force_bw,
                            "in_contact": in_contact,
                            "heel_strike": event_name == "heel_strike",
                            "toe_off": event_name == "toe_off",
                            "last_heel_strike_time": last_hs,
                            "last_toe_off_time": last_to,
                            "cycle_duration_s": cycle_duration,
                            "gait_phase": gait_phase,
                        }
                    }
                },
                "online_grf": {
                    "left": {
                        "penetration": float(sample.get("penetration_m", 0.0)),
                        "slip_speed": float(sample.get("slip_speed_m_s", 0.0)),
                    }
                },
                "phase_fsm": payload,
                "observation": {
                    "pros_knee_angle": float(sample.get("knee_rad", 0.0)),
                    "pros_ankle_angle": float(sample.get("ankle_rad", 0.0)),
                    "pros_knee_angle_served_ref": float(
                        sample.get("knee_rad", 0.0)
                    ),
                    "pros_ankle_angle_served_ref": float(
                        sample.get("ankle_rad", 0.0)
                    ),
                },
                "reward_terms": override_terms,
                "end_reason": sample.get("end_reason"),
            }
        )
        terms.update(task_terms)
        if isinstance(overrides, Mapping):
            terms.update(override_terms)
        reference = sample.get("reference")
        reward, components = reward_function.compute_reward(
            terms,
            reward_cfg,
            reference=reference,
        )
        grf_slip_term = float(reward_cfg.grf_slip_weight) * float(
            terms.get("grf_slip_loss", 0.0)
        )
        rewards.append(float(reward))
        terms_rows.append(terms)
        component_rows.append(components)
        trace_rows.append(
            {
                "subcase": subcase,
                "step": step,
                "time": float(sample["time"]),
                "event": event_name or "",
                "reward": float(reward),
                "reward_base": components.get("reward_base", 0.0),
                "normal_force_bw": force_bw,
                "in_contact": float(in_contact),
                "knee_rad": float(sample.get("knee_rad", 0.0)),
                "ankle_rad": float(sample.get("ankle_rad", 0.0)),
                "fsm_state_id": payload.get("state_id", 0.0),
                "fsm_state_name": payload.get("state_name", ""),
                "invalid_event_type": payload.get("invalid_event_type", ""),
                "cycle_reject_reason": payload.get("cycle_reject_reason", ""),
                "phase_valid_hs_count": payload.get("valid_hs_count", 0.0),
                "phase_valid_to_count": payload.get("valid_to_count", 0.0),
                "phase_valid_cycle_count": payload.get("valid_cycle_count", 0.0),
                "invalid_event_count": payload.get("invalid_event_count", 0.0),
                "phase_event_progress_score": payload.get(
                    "phase_event_progress_score", 0.0
                ),
                "phase_cycle_complete_bonus": payload.get(
                    "phase_cycle_complete_bonus", 0.0
                ),
                "contact_load_score": terms.get("contact_load_score", 0.0),
                "contact_load_dense_active": terms.get(
                    "contact_load_dense_active", 0.0
                ),
                "contact_load_evidence_complete": terms.get(
                    "contact_load_evidence_complete", 0.0
                ),
                "contact_support_to_score": terms.get(
                    "contact_support_to_score", 0.0
                ),
                "contact_support_pending_dense_reward": terms.get(
                    "contact_support_pending_dense_reward", 0.0
                ),
                "contact_support_clawback_penalty": terms.get(
                    "contact_support_clawback_penalty", 0.0
                ),
                "contact_support_clawback_term": components.get(
                    "contact_support_clawback_term", 0.0
                ),
                "landing_window_contact_score": terms.get(
                    "landing_window_contact_score", 0.0
                ),
                "swing_unloading_loss": terms.get("swing_unloading_loss", 0.0),
                "phase_regular_score": terms.get("phase_regular_score", 0.0),
                "phase_timeout_loss": terms.get("phase_timeout_loss", 0.0),
                "phase_stance_timeout_loss": terms.get(
                    "phase_stance_timeout_loss", 0.0
                ),
                "phase_swing_timeout_loss": terms.get(
                    "phase_swing_timeout_loss", 0.0
                ),
                "phase_timeout_penalty_term": components.get(
                    "phase_timeout_penalty_term", 0.0
                ),
                "phase_clawback_penalty": payload.get(
                    "phase_clawback_penalty", 0.0
                ),
                "phase_failure_extra_penalty": payload.get(
                    "phase_failure_extra_penalty", 0.0
                ),
                "phase_cycle_rejected_this_step": payload.get(
                    "cycle_rejected_this_step", 0.0
                ),
                "phase_stance_contact_fraction": payload.get(
                    "stance_contact_fraction", 0.0
                ),
                "phase_stance_load_integral_bw_s": payload.get(
                    "stance_load_integral_bw_s", 0.0
                ),
                "phase_cycle_knee_excursion_rad": payload.get(
                    "cycle_knee_excursion_rad", 0.0
                ),
                "phase_cycle_ankle_excursion_rad": payload.get(
                    "cycle_ankle_excursion_rad", 0.0
                ),
                "grf_slip_loss": terms.get("grf_slip_loss", 0.0),
                "grf_slip_term": grf_slip_term,
                "reward_without_grf_slip": float(reward) + grf_slip_term,
                "prosthetic_joint_range_loss": terms.get(
                    "prosthetic_joint_range_loss", 0.0
                ),
                "prosthetic_joint_range_term": components.get(
                    "prosthetic_joint_range_term", 0.0
                ),
                "morphology_loss": terms.get("morphology_loss", 0.0),
                "morphology_term": components.get("morphology_term", 0.0),
                "oob_loss": components.get("oob_loss", 0.0),
                "oob_term": components.get("oob_term", 0.0),
            }
        )
    final_payload = fsm.payload()
    return trace_rows, event_rows, {
        "steps": len(samples),
        "episode_return": float(np.sum(rewards)) if rewards else 0.0,
        "reward_mean": float(np.mean(rewards)) if rewards else 0.0,
        "reward_min": float(np.min(rewards)) if rewards else 0.0,
        "reward_max": float(np.max(rewards)) if rewards else 0.0,
        "final_fsm": _jsonable(final_payload),
        "trace_stats": _numeric_stats(trace_rows),
        "reward_terms_stats": _numeric_stats(terms_rows),
        "reward_components_stats": _numeric_stats(component_rows),
    }


def run_fake_cycle_ankle_only(args: argparse.Namespace, output_dir: Path) -> dict[str, Any]:
    config_path, _flat, reward_cfg = _load_reward_config(args.config, args.reward_json)
    output_dir.mkdir(parents=True, exist_ok=True)

    no_load = [
        {
            "time": 0.00,
            "event": "heel_strike",
            "normal_force_bw": 0.0,
            "in_contact": False,
            "knee_rad": -0.60,
            "ankle_rad": 0.00,
        },
        {
            "time": 0.35,
            "normal_force_bw": 0.0,
            "in_contact": False,
            "knee_rad": -0.60,
            "ankle_rad": 0.28,
        },
        {
            "time": 0.65,
            "event": "toe_off",
            "normal_force_bw": 0.0,
            "in_contact": False,
            "knee_rad": -0.59,
            "ankle_rad": -0.24,
        },
        {
            "time": 0.95,
            "event": "heel_strike",
            "normal_force_bw": 0.0,
            "in_contact": False,
            "knee_rad": -0.59,
            "ankle_rad": 0.22,
        },
    ]
    loaded_static_knee = [
        {
            "time": 0.00,
            "event": "heel_strike",
            "normal_force_bw": 0.70,
            "in_contact": True,
            "knee_rad": -0.60,
            "ankle_rad": 0.00,
        },
        {
            "time": 0.30,
            "normal_force_bw": 0.70,
            "in_contact": True,
            "knee_rad": -0.595,
            "ankle_rad": 0.30,
        },
        {
            "time": 0.60,
            "event": "toe_off",
            "normal_force_bw": 0.00,
            "in_contact": False,
            "knee_rad": -0.590,
            "ankle_rad": -0.25,
        },
        {
            "time": 0.95,
            "normal_force_bw": 0.00,
            "in_contact": False,
            "knee_rad": -0.585,
            "ankle_rad": 0.24,
        },
        {
            "time": 1.15,
            "event": "heel_strike",
            "normal_force_bw": 0.70,
            "in_contact": True,
            "knee_rad": -0.580,
            "ankle_rad": -0.20,
        },
    ]

    trace_rows: list[dict[str, Any]] = []
    event_rows: list[dict[str, Any]] = []
    subcase_summaries: dict[str, Any] = {}
    for subcase, samples in (
        ("no_load_oscillation", no_load),
        ("loaded_static_knee", loaded_static_knee),
    ):
        trace, events, sub_summary = _run_synthetic_sequence(
            subcase=subcase,
            samples=samples,
            reward_cfg=reward_cfg,
        )
        trace_rows.extend(trace)
        event_rows.extend(events)
        subcase_summaries[subcase] = sub_summary

    all_stats = _numeric_stats(trace_rows)
    loaded = subcase_summaries["loaded_static_knee"]
    no_load_summary = subcase_summaries["no_load_oscillation"]
    loaded_final = loaded["final_fsm"]
    no_load_final = no_load_summary["final_fsm"]
    reject_count = _stat(
        {"trace_stats": all_stats},
        "trace_stats",
        "phase_cycle_rejected_this_step",
        "max",
    )
    criteria = {
        "no_valid_cycles": float(loaded_final.get("valid_cycle_count", 0.0)) == 0.0
        and float(no_load_final.get("valid_cycle_count", 0.0)) == 0.0,
        "no_cycle_complete_bonus": _stat(
            {"trace_stats": all_stats},
            "trace_stats",
            "phase_cycle_complete_bonus",
            "max",
        )
        == 0.0,
        "loaded_static_knee_rejected": reject_count >= 1.0
        and str(loaded_final.get("cycle_reject_reason", ""))
        == "cycle_knee_excursion_too_low",
        "no_load_rejects_toe_off": "stance_contact_too_low"
        in {
            str(row.get("invalid_event_type", ""))
            for row in trace_rows
            if row.get("subcase") == "no_load_oscillation"
        },
        "knee_excursion_below_gate": _stat(
            loaded,
            "trace_stats",
            "phase_cycle_knee_excursion_rad",
            "max",
        )
        < float(reward_cfg.phase_min_cycle_knee_excursion_rad),
        "ankle_oscillation_present": _stat(
            loaded,
            "trace_stats",
            "phase_cycle_ankle_excursion_rad",
            "max",
        )
        >= 0.40,
        "reward_mean_low": _stat(
            {"trace_stats": all_stats},
            "trace_stats",
            "reward",
            "mean",
        )
        <= 0.15,
    }
    status = "PASS" if all(criteria.values()) else "FAIL"
    summary = {
        "ok": status == "PASS",
        "test_id": "fake_cycle_ankle_only",
        "status": status,
        "gate_role": "negative_gate",
        "output_dir": str(output_dir),
        "config": str(config_path),
        "pass_criteria": criteria,
        "metrics": {
            "reward_mean": _stat(
                {"trace_stats": all_stats}, "trace_stats", "reward", "mean"
            ),
            "cycle_complete_bonus_max": _stat(
                {"trace_stats": all_stats},
                "trace_stats",
                "phase_cycle_complete_bonus",
                "max",
            ),
            "cycle_rejected_max": reject_count,
            "loaded_final_valid_cycles": float(
                loaded_final.get("valid_cycle_count", 0.0)
            ),
            "loaded_knee_excursion_max_rad": _stat(
                loaded,
                "trace_stats",
                "phase_cycle_knee_excursion_rad",
                "max",
            ),
            "loaded_ankle_excursion_max_rad": _stat(
                loaded,
                "trace_stats",
                "phase_cycle_ankle_excursion_rad",
                "max",
            ),
            "phase_min_cycle_knee_excursion_rad": float(
                reward_cfg.phase_min_cycle_knee_excursion_rad
            ),
            "phase_min_stance_contact_fraction": float(
                reward_cfg.phase_min_stance_contact_fraction
            ),
            "phase_min_stance_load_bw_s": float(
                reward_cfg.phase_min_stance_load_bw_s
            ),
        },
        "notes": [
            "Synthetic HS-TO-HS sequences oscillate the ankle while keeping knee excursion below the configured gate.",
            "The loaded subcase has stance contact/load evidence, so the rejection must come from the knee-excursion gate.",
        ],
        "subcases": subcase_summaries,
        "trace_stats": all_stats,
        "reward_config": reward_cfg.to_dict(),
    }
    _write_csv(output_dir / "trace.csv", trace_rows)
    _write_csv(output_dir / "online_events.csv", event_rows)
    _write_summary(output_dir, summary)
    return summary


def _write_synthetic_audit(
    *,
    output_dir: Path,
    config_path: Path,
    test_id: str,
    gate_role: str,
    trace_rows: list[dict[str, Any]],
    event_rows: list[dict[str, Any]],
    subcase_summaries: dict[str, Any],
    reward_cfg: reward_function.RewardConfig,
    criteria: dict[str, bool],
    metrics: dict[str, float],
    notes: list[str],
) -> dict[str, Any]:
    status = "PASS" if all(criteria.values()) else "FAIL"
    summary = {
        "ok": status == "PASS",
        "test_id": test_id,
        "status": status,
        "gate_role": gate_role,
        "output_dir": str(output_dir),
        "config": str(config_path),
        "pass_criteria": criteria,
        "metrics": metrics,
        "notes": notes,
        "subcases": subcase_summaries,
        "trace_stats": _numeric_stats(trace_rows),
        "reward_config": reward_cfg.to_dict(),
    }
    _write_csv(output_dir / "trace.csv", trace_rows)
    _write_csv(output_dir / "online_events.csv", event_rows)
    _write_summary(output_dir, summary)
    return summary


def _run_single_synthetic_scenario(
    args: argparse.Namespace,
    output_dir: Path,
    *,
    test_id: str,
    gate_role: str,
    samples: list[dict[str, Any]],
    metric_builder,
    criteria_builder,
    notes: list[str],
) -> dict[str, Any]:
    config_path, _flat, reward_cfg = _load_reward_config(args.config, args.reward_json)
    output_dir.mkdir(parents=True, exist_ok=True)
    trace_rows, event_rows, sub_summary = _run_synthetic_sequence(
        subcase=test_id,
        samples=samples,
        reward_cfg=reward_cfg,
    )
    subcases = {test_id: sub_summary}
    stats = _numeric_stats(trace_rows)
    metrics = metric_builder(stats, subcases, reward_cfg)
    criteria = criteria_builder(metrics, stats, subcases, reward_cfg)
    return _write_synthetic_audit(
        output_dir=output_dir,
        config_path=config_path,
        test_id=test_id,
        gate_role=gate_role,
        trace_rows=trace_rows,
        event_rows=event_rows,
        subcase_summaries=subcases,
        reward_cfg=reward_cfg,
        criteria=criteria,
        metrics=metrics,
        notes=notes,
    )


def _common_negative_metrics(
    stats: Mapping[str, Any],
    subcases: Mapping[str, Any],
    _reward_cfg: reward_function.RewardConfig,
) -> dict[str, float]:
    final_fsm = next(iter(subcases.values()))["final_fsm"]
    return {
        "reward_mean": _stat({"trace_stats": stats}, "trace_stats", "reward"),
        "reward_min": _stat({"trace_stats": stats}, "trace_stats", "reward", "min"),
        "phase_timeout_penalty_term_mean": _stat(
            {"trace_stats": stats},
            "trace_stats",
            "phase_timeout_penalty_term",
        ),
        "phase_stance_timeout_loss_mean": _stat(
            {"trace_stats": stats},
            "trace_stats",
            "phase_stance_timeout_loss",
        ),
        "phase_swing_timeout_loss_mean": _stat(
            {"trace_stats": stats},
            "trace_stats",
            "phase_swing_timeout_loss",
        ),
        "phase_event_progress_score_mean": _stat(
            {"trace_stats": stats},
            "trace_stats",
            "phase_event_progress_score",
        ),
        "contact_load_score_mean": _stat(
            {"trace_stats": stats},
            "trace_stats",
            "contact_load_score",
        ),
        "landing_window_contact_score_mean": _stat(
            {"trace_stats": stats},
            "trace_stats",
            "landing_window_contact_score",
        ),
        "swing_unloading_loss_mean": _stat(
            {"trace_stats": stats},
            "trace_stats",
            "swing_unloading_loss",
        ),
        "valid_hs_count_final": float(final_fsm.get("valid_hs_count", 0.0) or 0.0),
        "valid_to_count_final": float(final_fsm.get("valid_to_count", 0.0) or 0.0),
        "valid_cycle_count_final": float(
            final_fsm.get("valid_cycle_count", 0.0) or 0.0
        ),
    }


def run_static_leg(args: argparse.Namespace, output_dir: Path) -> dict[str, Any]:
    samples = [
        {"time": 0.00, "event": "heel_strike", "knee_rad": -0.60, "ankle_rad": 0.0},
        {"time": 0.80, "knee_rad": -0.60, "ankle_rad": 0.0},
        {"time": 1.55, "knee_rad": -0.60, "ankle_rad": 0.0},
        {"time": 2.25, "knee_rad": -0.60, "ankle_rad": 0.0},
    ]

    def criteria(metrics, _stats, _subcases, _cfg):
        return {
            "reward_mean_le_0p10": metrics["reward_mean"] <= 0.10,
            "no_valid_cycles": metrics["valid_cycle_count_final"] == 0.0,
            "timeout_penalty_positive": metrics[
                "phase_timeout_penalty_term_mean"
            ]
            > 0.0,
            "phase_event_progress_low": metrics[
                "phase_event_progress_score_mean"
            ]
            <= 0.05,
            "contact_load_not_compensating": metrics["contact_load_score_mean"]
            <= 0.20,
        }

    return _run_single_synthetic_scenario(
        args,
        output_dir,
        test_id="static_leg",
        gate_role="negative_gate",
        samples=samples,
        metric_builder=_common_negative_metrics,
        criteria_builder=criteria,
        notes=[
            "Synthetic static prosthesis: initial HS only, no contact/load progression and no TO.",
        ],
    )


def run_compressive_stance(
    args: argparse.Namespace,
    output_dir: Path,
) -> dict[str, Any]:
    samples = [
        {
            "time": 0.00,
            "event": "heel_strike",
            "normal_force_bw": 0.20,
            "in_contact": True,
            "penetration_m": 0.002,
            "knee_rad": -0.20,
            "ankle_rad": 0.04,
        },
        {
            "time": 0.10,
            "normal_force_bw": 0.25,
            "in_contact": True,
            "penetration_m": 0.005,
            "knee_rad": -0.21,
            "ankle_rad": 0.05,
        },
        {
            "time": 0.20,
            "normal_force_bw": 0.35,
            "in_contact": True,
            "penetration_m": 0.009,
            "knee_rad": -0.22,
            "ankle_rad": 0.06,
        },
        {
            "time": 0.30,
            "normal_force_bw": 0.45,
            "in_contact": True,
            "penetration_m": 0.011,
            "knee_rad": -0.23,
            "ankle_rad": 0.07,
        },
        {
            "time": 0.36,
            "normal_force_bw": 0.50,
            "in_contact": True,
            "penetration_m": 0.0221,
            "knee_rad": -0.24,
            "ankle_rad": 0.08,
            "end_reason": "grf_penetration",
            "term_overrides": {
                "safety_loss": 1.0,
                "grf_penetration_loss": 1.0,
                "terminated": 1.0,
            },
        },
    ]

    def metrics(stats, subcases, cfg):
        values = _common_negative_metrics(stats, subcases, cfg)
        case = next(iter(subcases.values()))
        values.update(
            {
                "episode_return": float(case.get("episode_return", 0.0)),
                "contact_support_clawback_max": _stat(
                    {"trace_stats": stats},
                    "trace_stats",
                    "contact_support_clawback_penalty",
                    "max",
                ),
                "contact_support_to_score_max": _stat(
                    {"trace_stats": stats},
                    "trace_stats",
                    "contact_support_to_score",
                    "max",
                ),
                "contact_load_score_last": _stat(
                    {"trace_stats": stats},
                    "trace_stats",
                    "contact_load_score",
                    "last",
                ),
            }
        )
        return values

    def criteria(values, _stats, _subcases, _cfg):
        return {
            "no_to": values["valid_to_count_final"] == 0.0,
            "no_valid_cycles": values["valid_cycle_count_final"] == 0.0,
            "episode_return_negative": values["episode_return"] < 0.0,
            "provisional_credit_clawed_back": values[
                "contact_support_clawback_max"
            ]
            > 0.0,
            "no_to_confirmation_credit": values[
                "contact_support_to_score_max"
            ]
            == 0.0,
            "terminal_contact_credit_zero": values["contact_load_score_last"]
            == 0.0,
        }

    return _run_single_synthetic_scenario(
        args,
        output_dir,
        test_id="compressive_stance",
        gate_role="negative_gate",
        samples=samples,
        metric_builder=metrics,
        criteria_builder=criteria,
        notes=[
            "Synthetic H2 failure: monotonic load/penetration, no prosthetic TO, terminal penetration guard.",
            "Dense support credit must stop after minimum evidence and be clawed back at failure.",
        ],
    )


def run_missing_to(args: argparse.Namespace, output_dir: Path) -> dict[str, Any]:
    samples = [
        {
            "time": 0.00,
            "event": "heel_strike",
            "normal_force_bw": 0.70,
            "in_contact": True,
            "knee_rad": -0.60,
            "ankle_rad": 0.0,
        },
        {"time": 0.80, "normal_force_bw": 0.70, "in_contact": True, "knee_rad": -0.55, "ankle_rad": 0.05},
        {"time": 1.55, "normal_force_bw": 0.70, "in_contact": True, "knee_rad": -0.54, "ankle_rad": 0.05},
        {"time": 1.90, "normal_force_bw": 0.70, "in_contact": True, "knee_rad": -0.54, "ankle_rad": 0.05},
        {"time": 2.25, "normal_force_bw": 0.70, "in_contact": True, "knee_rad": -0.54, "ankle_rad": 0.05},
    ]

    def criteria(metrics, _stats, _subcases, _cfg):
        return {
            "has_initial_hs": metrics["valid_hs_count_final"] >= 1.0,
            "no_to": metrics["valid_to_count_final"] == 0.0,
            "no_valid_cycles": metrics["valid_cycle_count_final"] == 0.0,
            "stance_timeout_loss_positive": metrics[
                "phase_stance_timeout_loss_mean"
            ]
            > 0.0,
            "timeout_penalty_positive": metrics[
                "phase_timeout_penalty_term_mean"
            ]
            > 0.0,
            "reward_mean_le_0p15": metrics["reward_mean"] <= 0.15,
            "reward_min_negative": metrics["reward_min"] < 0.0,
        }

    return _run_single_synthetic_scenario(
        args,
        output_dir,
        test_id="missing_to",
        gate_role="negative_gate",
        samples=samples,
        metric_builder=_common_negative_metrics,
        criteria_builder=criteria,
        notes=[
            "Synthetic missing-TO case keeps stance load but never emits toe_off, exercising soft and hard stance timeout.",
        ],
    )


def run_missing_second_hs(args: argparse.Namespace, output_dir: Path) -> dict[str, Any]:
    samples = [
        {
            "time": 0.00,
            "event": "heel_strike",
            "normal_force_bw": 0.70,
            "in_contact": True,
            "knee_rad": -0.60,
            "ankle_rad": 0.0,
        },
        {"time": 0.30, "normal_force_bw": 0.70, "in_contact": True, "knee_rad": -0.45, "ankle_rad": 0.1},
        {"time": 0.60, "event": "toe_off", "normal_force_bw": 0.00, "in_contact": False, "knee_rad": -0.35, "ankle_rad": -0.1},
        {"time": 0.90, "normal_force_bw": 0.00, "in_contact": False, "knee_rad": -0.35, "ankle_rad": -0.1},
        {"time": 1.20, "normal_force_bw": 0.00, "in_contact": False, "knee_rad": -0.36, "ankle_rad": -0.1},
        {"time": 1.50, "normal_force_bw": 0.00, "in_contact": False, "knee_rad": -0.36, "ankle_rad": -0.1},
        {"time": 1.75, "normal_force_bw": 0.00, "in_contact": False, "knee_rad": -0.36, "ankle_rad": -0.1},
    ]

    def criteria(metrics, _stats, _subcases, _cfg):
        return {
            "one_hs": metrics["valid_hs_count_final"] == 1.0,
            "has_to": metrics["valid_to_count_final"] >= 1.0,
            "no_valid_cycles": metrics["valid_cycle_count_final"] == 0.0,
            "swing_timeout_loss_positive": metrics[
                "phase_swing_timeout_loss_mean"
            ]
            > 0.0,
            "timeout_penalty_positive": metrics[
                "phase_timeout_penalty_term_mean"
            ]
            > 0.0,
            "landing_contact_low": metrics["landing_window_contact_score_mean"]
            <= 0.05,
            "reward_mean_le_0p05": metrics["reward_mean"] <= 0.05,
            "reward_min_negative": metrics["reward_min"] < 0.0,
        }

    return _run_single_synthetic_scenario(
        args,
        output_dir,
        test_id="missing_second_hs",
        gate_role="negative_gate",
        samples=samples,
        metric_builder=_common_negative_metrics,
        criteria_builder=criteria,
        notes=[
            "Synthetic HS->TO sequence omits the second HS, so swing timeout and clawback must dominate.",
        ],
    )


def run_swing_load(args: argparse.Namespace, output_dir: Path) -> dict[str, Any]:
    samples = [
        {
            "time": 0.00,
            "event": "heel_strike",
            "normal_force_bw": 0.70,
            "in_contact": True,
            "knee_rad": -0.60,
            "ankle_rad": 0.0,
        },
        {"time": 0.30, "normal_force_bw": 0.70, "in_contact": True, "knee_rad": -0.45, "ankle_rad": 0.1},
        {"time": 0.60, "event": "toe_off", "normal_force_bw": 0.00, "in_contact": False, "knee_rad": -0.35, "ankle_rad": -0.1},
        {"time": 0.75, "normal_force_bw": 1.50, "in_contact": True, "knee_rad": -0.35, "ankle_rad": -0.1},
        {"time": 0.85, "normal_force_bw": 1.50, "in_contact": True, "knee_rad": -0.35, "ankle_rad": -0.1},
        {"time": 0.95, "normal_force_bw": 0.00, "in_contact": False, "knee_rad": -0.35, "ankle_rad": -0.1},
    ]

    def criteria(metrics, _stats, _subcases, _cfg):
        return {
            "swing_unloading_loss_positive": metrics["swing_unloading_loss_mean"]
            > 0.0,
            "contact_load_not_compensating": metrics["contact_load_score_mean"]
            <= 0.35,
            "reward_below_prescribed_margin": metrics["reward_mean"] <= 0.1865,
        }

    return _run_single_synthetic_scenario(
        args,
        output_dir,
        test_id="swing_load",
        gate_role="negative_gate",
        samples=samples,
        metric_builder=_common_negative_metrics,
        criteria_builder=criteria,
        notes=[
            "Synthetic swing-load case injects vertical force shortly after TO, before the landing window should reward contact.",
        ],
    )


def run_joint_oob(args: argparse.Namespace, output_dir: Path) -> dict[str, Any]:
    out_of_band_reference = [[0.30, 0.85]]
    samples = [
        {
            "time": 0.00,
            "knee_rad": 0.30,
            "ankle_rad": 0.85,
            "reference": out_of_band_reference,
            "term_overrides": {"prosthetic_joint_range_loss": 1.0},
        },
        {
            "time": 0.30,
            "knee_rad": 0.30,
            "ankle_rad": 0.85,
            "reference": out_of_band_reference,
            "term_overrides": {"prosthetic_joint_range_loss": 1.0},
        },
        {
            "time": 0.60,
            "knee_rad": 0.30,
            "ankle_rad": 0.85,
            "reference": out_of_band_reference,
            "term_overrides": {"prosthetic_joint_range_loss": 1.0},
        },
    ]

    def metrics(stats, subcases, cfg):
        out = _common_negative_metrics(stats, subcases, cfg)
        out.update(
            {
                "prosthetic_joint_range_loss_mean": _stat(
                    {"trace_stats": stats},
                    "trace_stats",
                    "prosthetic_joint_range_loss",
                ),
                "oob_loss_mean": _stat({"trace_stats": stats}, "trace_stats", "oob_loss"),
                "oob_term_mean": _stat({"trace_stats": stats}, "trace_stats", "oob_term"),
            }
        )
        return out

    def criteria(metrics, _stats, _subcases, _cfg):
        return {
            "joint_range_loss_positive": metrics[
                "prosthetic_joint_range_loss_mean"
            ]
            > 0.0,
            "oob_loss_positive": metrics["oob_loss_mean"] > 0.0,
            "reward_below_prescribed_margin": metrics["reward_mean"] <= 0.1865,
        }

    return _run_single_synthetic_scenario(
        args,
        output_dir,
        test_id="joint_oob",
        gate_role="negative_gate",
        samples=samples,
        metric_builder=metrics,
        criteria_builder=criteria,
        notes=[
            "Synthetic joint-OOB case injects actual joint range loss and out-of-band commanded reference.",
        ],
    )


def run_slip_injection(args: argparse.Namespace, output_dir: Path) -> dict[str, Any]:
    config_path, _flat, reward_cfg = _load_reward_config(args.config, args.reward_json)
    output_dir.mkdir(parents=True, exist_ok=True)
    samples = [
        {"time": 0.00, "term_overrides": {"grf_slip_loss": 0.25}},
        {"time": 0.30, "term_overrides": {"grf_slip_loss": 0.25}},
        {"time": 0.60, "term_overrides": {"grf_slip_loss": 0.25}},
    ]
    enabled_cfg = replace(reward_cfg, grf_slip_weight=1.0)
    trace_rows: list[dict[str, Any]] = []
    event_rows: list[dict[str, Any]] = []
    subcases: dict[str, Any] = {}
    for subcase, cfg in (("current_weight_zero", reward_cfg), ("enabled_weight_one", enabled_cfg)):
        trace, events, summary = _run_synthetic_sequence(
            subcase=subcase,
            samples=samples,
            reward_cfg=cfg,
        )
        trace_rows.extend(trace)
        event_rows.extend(events)
        subcases[subcase] = summary
    current = subcases["current_weight_zero"]
    enabled = subcases["enabled_weight_one"]
    current_stats = current["trace_stats"]
    enabled_stats = enabled["trace_stats"]
    metrics = {
        "current_grf_slip_loss_mean": _stat(
            current, "trace_stats", "grf_slip_loss"
        ),
        "current_grf_slip_term_mean": _stat(
            current, "trace_stats", "grf_slip_term"
        ),
        "current_reward_mean": _stat(current, "trace_stats", "reward"),
        "current_reward_without_grf_slip_mean": _stat(
            current,
            "trace_stats",
            "reward_without_grf_slip",
        ),
        "enabled_grf_slip_term_mean": _stat(
            enabled,
            "trace_stats",
            "grf_slip_term",
        ),
        "enabled_reward_mean": _stat(enabled, "trace_stats", "reward"),
        "reward_drop_enabled_vs_current": _stat(current, "trace_stats", "reward")
        - _stat(enabled, "trace_stats", "reward"),
        "current_trace_rows": float(len(current_stats)),
        "enabled_trace_rows": float(len(enabled_stats)),
    }
    criteria = {
        "current_slip_diagnostic_present": metrics["current_grf_slip_loss_mean"]
        > 0.0,
        "current_slip_term_zero": metrics["current_grf_slip_term_mean"] == 0.0,
        "current_reward_unchanged_without_slip": abs(
            metrics["current_reward_without_grf_slip_mean"]
            - metrics["current_reward_mean"]
        )
        <= 1e-12,
        "enabled_slip_term_positive": metrics["enabled_grf_slip_term_mean"] > 0.0,
        "enabled_reward_drop_ge_0p10": metrics[
            "reward_drop_enabled_vs_current"
        ]
        >= 0.10,
    }
    return _write_synthetic_audit(
        output_dir=output_dir,
        config_path=config_path,
        test_id="slip_injection",
        gate_role="diagnostic_gate",
        trace_rows=trace_rows,
        event_rows=event_rows,
        subcase_summaries=subcases,
        reward_cfg=reward_cfg,
        criteria=criteria,
        metrics=metrics,
        notes=[
            "C6a uses the current config with grf_slip_weight=0.0.",
            "C6b uses an in-memory diagnostic config with grf_slip_weight=1.0.",
        ],
    )


def run_morphology_corridor(args: argparse.Namespace, output_dir: Path) -> dict[str, Any]:
    config_path, _flat, reward_cfg = _load_reward_config(args.config, args.reward_json)
    output_dir.mkdir(parents=True, exist_ok=True)
    current_samples = [
        {"time": 0.00, "term_overrides": {"morphology_loss": 0.20}},
        {"time": 0.30, "term_overrides": {"morphology_loss": 0.20}},
    ]
    low_loss_samples = [
        {"time": 0.00, "term_overrides": {"morphology_loss": 0.01}},
        {"time": 0.30, "term_overrides": {"morphology_loss": 0.01}},
    ]
    high_loss_samples = [
        {"time": 0.00, "term_overrides": {"morphology_loss": 0.50}},
        {"time": 0.30, "term_overrides": {"morphology_loss": 0.50}},
    ]
    enabled_cfg = replace(reward_cfg, morphology_weight=0.5)
    trace_rows: list[dict[str, Any]] = []
    event_rows: list[dict[str, Any]] = []
    subcases: dict[str, Any] = {}
    for subcase, samples, cfg in (
        ("current_weight_zero", current_samples, reward_cfg),
        ("enabled_low_loss", low_loss_samples, enabled_cfg),
        ("enabled_high_loss", high_loss_samples, enabled_cfg),
    ):
        trace, events, summary = _run_synthetic_sequence(
            subcase=subcase,
            samples=samples,
            reward_cfg=cfg,
        )
        trace_rows.extend(trace)
        event_rows.extend(events)
        subcases[subcase] = summary
    current = subcases["current_weight_zero"]
    low = subcases["enabled_low_loss"]
    high = subcases["enabled_high_loss"]
    metrics = {
        "current_morphology_loss_mean": _stat(
            current,
            "trace_stats",
            "morphology_loss",
        ),
        "current_morphology_term_mean": _stat(
            current,
            "trace_stats",
            "morphology_term",
        ),
        "enabled_low_morphology_loss_mean": _stat(
            low,
            "trace_stats",
            "morphology_loss",
        ),
        "enabled_high_morphology_loss_mean": _stat(
            high,
            "trace_stats",
            "morphology_loss",
        ),
        "enabled_low_reward_mean": _stat(low, "trace_stats", "reward"),
        "enabled_high_reward_mean": _stat(high, "trace_stats", "reward"),
    }
    criteria = {
        "current_term_zero": metrics["current_morphology_term_mean"] == 0.0,
        "current_loss_finite": math.isfinite(metrics["current_morphology_loss_mean"]),
        "enabled_low_loss_below_high": metrics[
            "enabled_low_morphology_loss_mean"
        ]
        < metrics["enabled_high_morphology_loss_mean"],
        "enabled_high_reward_lower": metrics["enabled_high_reward_mean"]
        < metrics["enabled_low_reward_mean"],
    }
    return _write_synthetic_audit(
        output_dir=output_dir,
        config_path=config_path,
        test_id="morphology_corridor",
        gate_role="diagnostic_gate",
        trace_rows=trace_rows,
        event_rows=event_rows,
        subcase_summaries=subcases,
        reward_cfg=reward_cfg,
        criteria=criteria,
        metrics=metrics,
        notes=[
            "Synthetic diagnostic: verifies morphology_weight=0 is non-penalizing and a temporary positive weight is active.",
            "This does not replace a future prescribed/OpenSim corridor replay with perturbed kinematics.",
        ],
    )


def _run_prescribed_probe(args: argparse.Namespace, output_dir: Path, scenario: str) -> dict[str, Any]:
    config_path = _resolve_config_path(args.config)
    spec = PRESCRIBED_SCENARIOS[scenario]
    start_time = float(args.start_time if args.start_time is not None else spec["start_time"])
    end_time = float(args.end_time if args.end_time is not None else spec["end_time"])
    output_dir.mkdir(parents=True, exist_ok=True)

    cmd = [
        sys.executable,
        str(REPO / "validation" / "prescribed_reward_probe.py"),
        "--config",
        str(config_path),
        "--dynamics-contract",
        "prescribed_pure",
        "--start-time",
        f"{start_time:.15g}",
        "--end-time",
        f"{end_time:.15g}",
        "--output-dir",
        str(output_dir),
        "--step-wall-timeout-s",
        f"{float(args.step_wall_timeout_s):.15g}",
    ]
    if args.reward_json:
        cmd.extend(["--reward-json", str(args.reward_json)])
    if args.record_sim_outputs:
        cmd.append("--record-sim-outputs")

    stdout_path = output_dir / "probe_stdout.txt"
    stderr_path = output_dir / "probe_stderr.txt"
    with stdout_path.open("w", encoding="utf-8") as stdout, stderr_path.open(
        "w", encoding="utf-8"
    ) as stderr:
        completed = subprocess.run(
            cmd,
            cwd=str(REPO),
            stdout=stdout,
            stderr=stderr,
            check=False,
        )

    summary_path = output_dir / "summary.json"
    if summary_path.is_file():
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
    else:
        summary = {
            "ok": False,
            "output_dir": str(output_dir),
            "config": str(config_path),
            "window": {"start_time": start_time, "end_time": end_time},
        }
        _write_csv(output_dir / "trace.csv", [])
        _write_csv(output_dir / "online_events.csv", [])

    final_fsm = summary.get("final_fsm", {})
    if not isinstance(final_fsm, Mapping):
        final_fsm = {}
    metrics = {
        "returncode": float(completed.returncode),
        "reward_mean": float(summary.get("reward_mean", 0.0) or 0.0),
        "episode_return": float(summary.get("episode_return", 0.0) or 0.0),
        "reward_min": float(summary.get("reward_min", 0.0) or 0.0),
        "grf_slip_loss_mean": _stat(
            summary, "reward_terms_stats", "grf_slip_loss"
        ),
        "prosthetic_slip_speed_m_s_mean": _stat(
            summary, "reward_terms_stats", "prosthetic_slip_speed_m_s"
        ),
        "phase_timeout_loss_mean": _stat(
            summary, "reward_terms_stats", "phase_timeout_loss"
        ),
        "phase_timeout_penalty_term_mean": _stat(
            summary, "reward_components_stats", "phase_timeout_penalty_term"
        ),
        "invalid_event_count_final": float(
            final_fsm.get("invalid_event_count", 0.0) or 0.0
        ),
        "valid_hs_count_final": float(final_fsm.get("valid_hs_count", 0.0) or 0.0),
        "valid_to_count_final": float(final_fsm.get("valid_to_count", 0.0) or 0.0),
        "valid_cycle_count_final": float(
            final_fsm.get("valid_cycle_count", 0.0) or 0.0
        ),
        "last_period_s": float(final_fsm.get("last_period_s", 0.0) or 0.0),
        "last_stance_fraction": float(
            final_fsm.get("last_stance_fraction", 0.0) or 0.0
        ),
        "prosthetic_joint_range_loss_mean": _stat(
            summary, "reward_terms_stats", "prosthetic_joint_range_loss"
        ),
        "oob_term_mean": _stat(summary, "reward_components_stats", "oob_term"),
        "contact_support_to_score_max": _stat(
            summary, "reward_terms_stats", "contact_support_to_score", "max"
        ),
        "contact_support_dense_confirmed_reward_max": _stat(
            summary,
            "reward_terms_stats",
            "contact_support_dense_confirmed_reward",
            "max",
        ),
        "contact_support_clawback_penalty_max": _stat(
            summary,
            "reward_terms_stats",
            "contact_support_clawback_penalty",
            "max",
        ),
    }

    if scenario == "prescribed_aligned":
        criteria = {
            "process_ok": completed.returncode == 0,
            "probe_ok": bool(summary.get("ok", False)),
            "reward_mean_ge_0p10": metrics["reward_mean"] >= 0.10,
            "episode_return_positive": metrics["episode_return"] > 0.0,
            "reward_min_ge_minus_1p25": metrics["reward_min"] >= -1.25,
            "well_timed_to_support": metrics["contact_support_to_score_max"]
            >= 0.95,
            "dense_support_confirmed": metrics[
                "contact_support_dense_confirmed_reward_max"
            ]
            > 0.0,
            "partial_stance_clawback_bounded": metrics[
                "contact_support_clawback_penalty_max"
            ]
            <= 1.25,
            "grf_slip_zero": abs(metrics["grf_slip_loss_mean"]) <= 1e-12,
            "prosthetic_slip_zero": abs(metrics["prosthetic_slip_speed_m_s_mean"])
            <= 1e-12,
            "phase_timeout_zero": abs(metrics["phase_timeout_loss_mean"]) <= 1e-12,
            "phase_timeout_term_zero": abs(
                metrics["phase_timeout_penalty_term_mean"]
            )
            <= 1e-12,
            "invalid_events_zero": metrics["invalid_event_count_final"] == 0.0,
            "valid_hs_ge_3": metrics["valid_hs_count_final"] >= 3.0,
            "valid_to_ge_2": metrics["valid_to_count_final"] >= 2.0,
            "valid_cycles_ge_2": metrics["valid_cycle_count_final"] >= 2.0,
            "joint_range_loss_zero": abs(
                metrics["prosthetic_joint_range_loss_mean"]
            )
            <= 1e-12,
            "oob_term_zero": abs(metrics["oob_term_mean"]) <= 1e-12,
        }
    elif scenario == "prescribed_long":
        criteria = {
            "process_ok": completed.returncode == 0,
            "probe_ok": bool(summary.get("ok", False)),
            "reward_mean_ge_0p10": metrics["reward_mean"] >= 0.10,
            "episode_return_positive": metrics["episode_return"] > 0.0,
            "grf_slip_zero": abs(metrics["grf_slip_loss_mean"]) <= 1e-12,
            "phase_timeout_zero": abs(metrics["phase_timeout_loss_mean"]) <= 1e-12,
            "invalid_events_zero": metrics["invalid_event_count_final"] == 0.0,
            "valid_cycles_ge_4": metrics["valid_cycle_count_final"] >= 4.0,
            "last_period_in_hard_band": 0.90
            <= metrics["last_period_s"]
            <= 2.20,
            "last_stance_fraction_in_band": 0.50
            <= metrics["last_stance_fraction"]
            <= 0.80,
        }
    else:
        criteria = {
            "process_ok": completed.returncode == 0,
            "probe_ok": bool(summary.get("ok", False)),
            "diagnostic_reward_positive": metrics["reward_mean"] > 0.0,
            "at_least_one_cycle": metrics["valid_cycle_count_final"] >= 1.0,
        }
    status = "PASS" if all(criteria.values()) else "FAIL"
    summary.update(
        {
            "test_id": scenario,
            "status": status,
            "gate_role": spec["gate_role"],
            "pass_criteria": criteria,
            "metrics": metrics,
            "notes": [
                "Executed via validation/prescribed_reward_probe.py.",
                f"stdout: {stdout_path.name}; stderr: {stderr_path.name}",
            ],
        }
    )
    _write_summary(output_dir, summary)
    return summary


def _run_unimplemented(args: argparse.Namespace, output_dir: Path, scenario: str) -> dict[str, Any]:
    config_path = _resolve_config_path(args.config)
    output_dir.mkdir(parents=True, exist_ok=True)
    summary = {
        "ok": False,
        "test_id": scenario,
        "status": "BLOCKED",
        "gate_role": "planned_not_implemented",
        "output_dir": str(output_dir),
        "config": str(config_path),
        "pass_criteria": {},
        "metrics": {},
        "notes": [
            "Scenario listed in the validation plan but not implemented in reward_audit_suite.py yet.",
            "No trained policy or checkpoint is required for this placeholder output.",
        ],
    }
    _write_csv(output_dir / "trace.csv", [{"scenario": scenario, "status": "BLOCKED"}])
    _write_csv(output_dir / "online_events.csv", [])
    _write_summary(output_dir, summary)
    return summary


def _run_preflight(args: argparse.Namespace, output_root: Path) -> dict[str, Any]:
    checks = [
        ("validate_training_config", [sys.executable, str(REPO / "validation" / "validate_training_config.py")]),
        ("test_reward_function", [sys.executable, str(REPO / "validation" / "test_reward_function.py")]),
    ]
    rows: list[dict[str, Any]] = []
    log_dir = output_root / "_preflight"
    log_dir.mkdir(parents=True, exist_ok=True)
    for name, cmd in checks:
        stdout_path = log_dir / f"{name}_stdout.txt"
        stderr_path = log_dir / f"{name}_stderr.txt"
        with stdout_path.open("w", encoding="utf-8") as stdout, stderr_path.open(
            "w", encoding="utf-8"
        ) as stderr:
            completed = subprocess.run(
                cmd,
                cwd=str(REPO),
                stdout=stdout,
                stderr=stderr,
                check=False,
            )
        rows.append(
            {
                "check": name,
                "returncode": completed.returncode,
                "status": "PASS" if completed.returncode == 0 else "FAIL",
                "stdout": str(stdout_path),
                "stderr": str(stderr_path),
            }
        )
    status = "PASS" if all(row["returncode"] == 0 for row in rows) else "FAIL"
    summary = {
        "ok": status == "PASS",
        "test_id": "preflight",
        "status": status,
        "pass_criteria": {
            row["check"]: row["returncode"] == 0 for row in rows
        },
        "metrics": {row["check"]: row["returncode"] for row in rows},
        "notes": ["Preflight executes the plan's A1 validators."],
        "checks": rows,
    }
    _write_csv(log_dir / "trace.csv", rows)
    _write_csv(log_dir / "online_events.csv", [])
    _write_summary(log_dir, summary)
    return summary


def _write_summary(output_dir: Path, summary: Mapping[str, Any]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "summary.json").write_text(
        json.dumps(_jsonable(summary), indent=2),
        encoding="utf-8",
    )
    criteria = summary.get("pass_criteria", {})
    if not isinstance(criteria, Mapping):
        criteria = {}
    metrics = summary.get("metrics", {})
    if not isinstance(metrics, Mapping):
        metrics = {}
    notes = summary.get("notes", [])
    if not isinstance(notes, list):
        notes = [str(notes)]
    lines = [
        f"# Reward Audit - {summary.get('test_id', 'unknown')}",
        "",
        f"- Status: {summary.get('status', 'UNKNOWN')}",
        f"- Gate role: {summary.get('gate_role', 'n/a')}",
        f"- Output: `{summary.get('output_dir', output_dir)}`",
        "",
        "## Criteria",
        "",
    ]
    if criteria:
        lines.extend(
            f"- {key}: {'PASS' if bool(value) else 'FAIL'}"
            for key, value in sorted(criteria.items())
        )
    else:
        lines.append("- n/a")
    lines.extend(["", "## Metrics", ""])
    if metrics:
        lines.extend(f"- {key}: {value}" for key, value in sorted(metrics.items()))
    else:
        lines.append("- n/a")
    lines.extend(["", "## Notes", ""])
    if notes:
        lines.extend(f"- {note}" for note in notes)
    else:
        lines.append("- n/a")
    lines.extend(["", "## Files", "", "- `summary.json`", "- `summary.md`", "- `trace.csv`"])
    if (output_dir / "online_events.csv").exists():
        lines.append("- `online_events.csv`")
    (output_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def _scenario_output_dir(args: argparse.Namespace, scenario: str, many: bool) -> Path:
    if args.output_dir:
        base = _resolve_repo_path(args.output_dir)
        return base / scenario if many else base
    return DEFAULT_OUTPUT_ROOT / f"{datetime.now():%Y-%m-%d_%H%M%S}_{scenario}"


def run_scenario(args: argparse.Namespace, scenario: str, output_dir: Path) -> dict[str, Any]:
    if scenario in PRESCRIBED_SCENARIOS:
        return _run_prescribed_probe(args, output_dir, scenario)
    if scenario == "static_leg":
        return run_static_leg(args, output_dir)
    if scenario == "compressive_stance":
        return run_compressive_stance(args, output_dir)
    if scenario == "missing_to":
        return run_missing_to(args, output_dir)
    if scenario == "missing_second_hs":
        return run_missing_second_hs(args, output_dir)
    if scenario == "swing_load":
        return run_swing_load(args, output_dir)
    if scenario == "joint_oob":
        return run_joint_oob(args, output_dir)
    if scenario == "slip_injection":
        return run_slip_injection(args, output_dir)
    if scenario == "morphology_corridor":
        return run_morphology_corridor(args, output_dir)
    if scenario == "fake_cycle_ankle_only":
        return run_fake_cycle_ankle_only(args, output_dir)
    return _run_unimplemented(args, output_dir, scenario)


def run(args: argparse.Namespace) -> dict[str, Any]:
    scenarios = list(args.scenario or ["fake_cycle_ankle_only"])
    if "all" in scenarios:
        scenarios = list(SCENARIOS)
    scenarios = list(dict.fromkeys(scenarios))
    many = len(scenarios) > 1
    output_root = _resolve_repo_path(args.output_dir) if args.output_dir else DEFAULT_OUTPUT_ROOT

    summaries: list[dict[str, Any]] = []
    if args.preflight:
        summaries.append(_run_preflight(args, output_root))
    for scenario in scenarios:
        if scenario not in SCENARIOS:
            raise ValueError(f"unknown scenario: {scenario}")
        output_dir = _scenario_output_dir(args, scenario, many)
        summaries.append(run_scenario(args, scenario, output_dir))

    status_order = {"FAIL": 2, "BLOCKED": 1, "PASS": 0}
    aggregate_status = max(
        (str(item.get("status", "FAIL")) for item in summaries),
        key=lambda status: status_order.get(status, 2),
    )
    aggregate = {
        "ok": aggregate_status == "PASS",
        "test_id": "reward_audit_suite",
        "status": aggregate_status,
        "scenario_count": len(scenarios),
        "summaries": [
            {
                "test_id": item.get("test_id"),
                "status": item.get("status"),
                "output_dir": item.get("output_dir"),
            }
            for item in summaries
        ],
    }
    print(json.dumps(_jsonable(aggregate), indent=2))
    return aggregate


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario",
        action="append",
        default=None,
        help=(
            "Scenario to run. May be repeated. Use 'all' to materialize every "
            "planned scenario, including BLOCKED placeholders."
        ),
    )
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--reward-json", default=None)
    parser.add_argument("--start-time", type=float, default=None)
    parser.add_argument("--end-time", type=float, default=None)
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--step-wall-timeout-s", type=float, default=60.0)
    parser.add_argument("--record-sim-outputs", action="store_true")
    parser.add_argument(
        "--preflight",
        action="store_true",
        help="Run validate_training_config.py and test_reward_function.py first.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    run(parse_args())
