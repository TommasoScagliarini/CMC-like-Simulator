"""Evaluate the ex-novo reward on a prescribed/IK gait window.

By default the probe keeps the dynamics prescribed and derives the reward
contact/phase inputs from the prescribed GRF file.  The ``training_like``
dynamics contract instead uses the same online-GRF application/disabled-side
lists as the training YAML.  In both modes the prosthesis command is a
zero-delta action around the prescribed IK trajectory, so the reward is tested
on a known walking reference rather than on a learned policy.

Default window: 12.99 s -> 17.99 s.
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
from xml.etree import ElementTree as ET

import numpy as np
import gymnasium as gym


REPO = Path(__file__).resolve().parents[1]
TRAJECTORY_DIR = REPO / "Trajectory Generator"
BASELINE_DIR = TRAJECTORY_DIR / "baseline_MLP"
DEFAULT_CONFIG = BASELINE_DIR / "training_exnovo_cfg.yaml"
DEFAULT_OUTPUT_ROOT = REPO / "validation" / "prescribed_reward_probe_runs"

for path in (REPO, TRAJECTORY_DIR, BASELINE_DIR):
    text = str(path)
    if text not in sys.path:
        sys.path.insert(0, text)

import _bootstrap  # noqa: E402

_bootstrap.ensure_sim_paths()

import reward_function  # noqa: E402
import training_config  # noqa: E402
from osim_trj_cmc_like import CMCEnvConfig, CMCLikeProsthesisTrajectoryEnv  # noqa: E402
from prosthetic_phase_fsm import ProstheticPhaseFSM, ProstheticPhaseFSMConfig  # noqa: E402


STATE_NAMES = {
    0: "WAIT_HS",
    1: "STANCE_AFTER_HS",
    2: "SWING_AFTER_TO",
    4: "TIMEOUT",
}

CONTRACT_PRESCRIBED_PURE = "prescribed_pure"
CONTRACT_TRAINING_LIKE = "training_like"
CONTRACTS = (CONTRACT_PRESCRIBED_PURE, CONTRACT_TRAINING_LIKE)


def _cli_path(value: str | Path) -> Path:
    text = os.fspath(value)
    if os.name != "nt":
        text = text.replace("\\", "/")
    return Path(text).expanduser()


def _resolve_repo_path(value: str | Path) -> Path:
    path = _cli_path(value)
    if not path.is_absolute():
        path = REPO / path
    return path.resolve()


def _resolve_output_dir(value: str | None) -> Path:
    if value:
        path = _cli_path(value)
        if not path.is_absolute():
            path = REPO / path
        return path.resolve()
    return (
        DEFAULT_OUTPUT_ROOT
        / f"prescribed_reward_probe_{datetime.now():%Y%m%d_%H%M%S}"
    ).resolve()


def _local_tag(element: ET.Element) -> str:
    return element.tag.rsplit("}", 1)[-1]


def _read_setup_times(setup_xml: str | Path) -> tuple[float, float]:
    setup_path = _resolve_repo_path(setup_xml)
    tree = ET.parse(setup_path)
    root = tree.getroot()
    values: dict[str, float] = {}
    for node in root.iter():
        tag = _local_tag(node)
        if tag in {"t_start", "t_end"}:
            values[tag] = float((node.text or "").strip())
    if "t_start" not in values or "t_end" not in values:
        raise ValueError(f"setup XML is missing t_start/t_end: {setup_path}")
    return values["t_start"], values["t_end"]


def _finite_float(value: Any) -> float | None:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return [_jsonable(item) for item in value.tolist()]
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def _numeric_stats(rows: list[Mapping[str, Any]]) -> dict[str, dict[str, float]]:
    keys = sorted({str(key) for row in rows for key in row})
    result: dict[str, dict[str, float]] = {}
    for key in keys:
        values = []
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
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: _jsonable(row.get(key, "")) for key in keys})


def _load_config(args: argparse.Namespace) -> tuple[dict[str, Any], reward_function.RewardConfig]:
    cfg_path = training_config.resolve_config_path(args.config)
    cfg = training_config.load(cfg_path)
    flat, reward_overrides = training_config.to_argparse_defaults(cfg)
    if args.reward_json:
        reward_overrides.update(
            reward_function.load_reward_overrides(args.reward_json) or {}
        )
    reward_cfg = reward_function.RewardConfig.from_mapping(reward_overrides)
    return flat, reward_cfg


def _oracle_left_cycles(env: CMCLikeProsthesisTrajectoryEnv) -> list[dict[str, float]]:
    from output import _cycles_from_vertical_grf, _read_storage_table

    grf_file = getattr(env.ctx, "grf_data_file", "")
    columns = getattr(env.ctx, "grf_vertical_force_columns", {})
    source_col = columns.get("left") if isinstance(columns, dict) else None
    if not grf_file or source_col is None:
        return []
    time, col_names, data = _read_storage_table(grf_file)
    col_idx = {name: i for i, name in enumerate(col_names)}
    idx = col_idx[source_col]
    cycles = _cycles_from_vertical_grf(
        time,
        data[:, idx],
        float(env.cfg.grf_contact_threshold_n),
        float(env._episode_start),
        float(env._episode_end),
        float(getattr(env.cfg, "grf_min_contact_duration_s", 0.0)),
        float(getattr(env.cfg, "grf_min_cycle_duration_s", 0.0)),
    )
    return [
        {
            "cycle_start": float(start),
            "cycle_end": float(end),
            "cycle_duration_s": float(end - start),
            "contact_duration_s": float(contact),
        }
        for start, end, contact in cycles
    ]


def _threshold_contact_events(
    time: np.ndarray,
    vertical_grf: np.ndarray,
    *,
    threshold: float,
    start_time: float,
    end_time: float,
    min_contact_duration_s: float,
) -> list[dict[str, float | str | None]]:
    events: list[dict[str, float | str | None]] = []
    contact_start: float | None = None
    contacts: list[tuple[float, float]] = []
    for i in range(1, len(time)):
        was_contact = vertical_grf[i - 1] > threshold
        is_contact = vertical_grf[i] > threshold
        if not was_contact and is_contact:
            denom = vertical_grf[i] - vertical_grf[i - 1]
            frac = 0.0 if abs(denom) < 1e-12 else (threshold - vertical_grf[i - 1]) / denom
            contact_start = float(time[i - 1] + frac * (time[i] - time[i - 1]))
            continue
        if was_contact and not is_contact and contact_start is not None:
            denom = vertical_grf[i] - vertical_grf[i - 1]
            frac = 0.0 if abs(denom) < 1e-12 else (threshold - vertical_grf[i - 1]) / denom
            contact_end = float(time[i - 1] + frac * (time[i] - time[i - 1]))
            if contact_end - contact_start >= min_contact_duration_s:
                contacts.append((contact_start, contact_end))
            contact_start = None
    if contact_start is not None:
        contact_end = float(time[-1])
        if contact_end - contact_start >= min_contact_duration_s:
            contacts.append((contact_start, contact_end))

    hs_times = [start for start, _end in contacts]
    eps = 1e-9
    for idx, (contact_start, contact_end) in enumerate(contacts):
        if start_time - eps <= contact_start <= end_time + eps:
            cycle_duration = (
                hs_times[idx] - hs_times[idx - 1] if idx > 0 else None
            )
            events.append(
                {
                    "time": float(contact_start),
                    "event": "heel_strike",
                    "cycle_duration_s": (
                        float(cycle_duration) if cycle_duration is not None else None
                    ),
                }
            )
        if start_time - eps <= contact_end <= end_time + eps:
            events.append(
                {
                    "time": float(contact_end),
                    "event": "toe_off",
                    "cycle_duration_s": None,
                }
            )
    return sorted(events, key=lambda item: float(item["time"]))


class PrescribedGRFRewardInputWrapper(gym.Wrapper):
    """Replace online contact/FSM payloads with prescribed-GRF equivalents."""

    def __init__(
        self,
        env: CMCLikeProsthesisTrajectoryEnv,
        reward_cfg: reward_function.RewardConfig,
    ) -> None:
        super().__init__(env)
        self.reward_cfg = reward_cfg
        self.body_weight_n = max(1e-9, float(getattr(env, "_body_weight_n", 1.0)))
        self.threshold_n = float(env.cfg.grf_contact_threshold_n)
        self.start_time = float(getattr(env, "_episode_start", 0.0))
        self.end_time = float(getattr(env, "_episode_end", 0.0))
        self.series = self._load_prescribed_grf_series(env)
        self.events_by_side = self._build_events()
        self.phase_fsm = ProstheticPhaseFSM(
            ProstheticPhaseFSMConfig(
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
        )
        self._last_probe_time: float | None = None
        self._gait_state: dict[str, dict[str, float | None]] = {}

    @staticmethod
    def _load_prescribed_grf_series(
        env: CMCLikeProsthesisTrajectoryEnv,
    ) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        from output import _read_storage_table

        grf_file = getattr(env.ctx, "grf_data_file", "")
        columns = getattr(env.ctx, "grf_vertical_force_columns", {})
        if not grf_file or not isinstance(columns, Mapping):
            raise RuntimeError("prescribed GRF file/columns are unavailable")
        time, col_names, data = _read_storage_table(grf_file)
        col_idx = {name: i for i, name in enumerate(col_names)}
        series: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        for side in ("left", "right"):
            source_col = columns.get(side)
            idx = col_idx.get(source_col) if source_col else None
            if idx is None:
                raise RuntimeError(f"missing prescribed vertical GRF column for {side}")
            series[side] = (time, data[:, idx])
        return series

    def _build_events(self) -> dict[str, list[dict[str, Any]]]:
        events_by_side: dict[str, list[dict[str, Any]]] = {}
        min_contact = float(getattr(self.env.cfg, "grf_min_contact_duration_s", 0.0))
        for side, (time, vertical) in self.series.items():
            side_events = _threshold_contact_events(
                time,
                vertical,
                threshold=self.threshold_n,
                start_time=self.start_time,
                end_time=self.end_time,
                min_contact_duration_s=min_contact,
            )
            start_force = float(np.interp(self.start_time, time, vertical))
            starts_in_contact = start_force > self.threshold_n
            has_initial_hs = any(
                str(event.get("event")) == "heel_strike"
                and abs(float(event["time"]) - self.start_time) <= 1e-6
                for event in side_events
            )
            if starts_in_contact and not has_initial_hs:
                side_events.insert(
                    0,
                    {
                        "time": float(self.start_time),
                        "event": "heel_strike",
                        "cycle_duration_s": None,
                        "bootstrap_initial_contact": 1.0,
                    },
                )
            events_by_side[side] = [
                {"side": side, **event}
                for event in side_events
            ]
        return events_by_side

    def reset(self, **kwargs):
        self.phase_fsm.reset()
        self._last_probe_time = self.start_time - 1e-9
        self._gait_state = {
            side: {
                "last_heel_strike_time": None,
                "last_toe_off_time": None,
                "cycle_duration_s": 0.0,
            }
            for side in ("left", "right")
        }
        return self.env.reset(**kwargs)

    def step(self, action):
        obs, reward, terminated, truncated, info = self.env.step(action)
        return obs, reward, terminated, truncated, self._override_info(info)

    def _events_until(self, time_s: float) -> list[dict[str, Any]]:
        previous = self._last_probe_time
        if previous is None:
            previous = self.start_time - 1e-9
        eps = 1e-9
        events = []
        for side_events in self.events_by_side.values():
            for event in side_events:
                event_time = float(event["time"])
                if previous < event_time <= time_s + eps:
                    events.append(dict(event))
        return sorted(events, key=lambda item: float(item["time"]))

    def _normal_force(self, side: str, time_s: float) -> float:
        time, vertical = self.series[side]
        return max(0.0, float(np.interp(time_s, time, vertical)))

    def _update_gait_state(self, events: list[dict[str, Any]]) -> None:
        for event in events:
            side = str(event.get("side", "")).lower()
            if side not in self._gait_state:
                continue
            state = self._gait_state[side]
            event_name = str(event.get("event", "")).lower()
            event_time = float(event.get("time", 0.0) or 0.0)
            if event_name == "heel_strike":
                previous_hs = state.get("last_heel_strike_time")
                if previous_hs is not None and event_time > float(previous_hs):
                    state["cycle_duration_s"] = event_time - float(previous_hs)
                state["last_heel_strike_time"] = event_time
            elif event_name == "toe_off":
                state["last_toe_off_time"] = event_time

    def _override_info(self, info: Mapping[str, Any]) -> dict[str, Any]:
        out = dict(info)
        time_s = float(out.get("time", getattr(self.env, "t", self.start_time)))
        events = self._events_until(time_s)
        self._update_gait_state(events)

        grf_sides: dict[str, dict[str, Any]] = {}
        gait_sides: dict[str, dict[str, Any]] = {}
        for side in ("left", "right"):
            normal_force = self._normal_force(side, time_s)
            normal_bw = normal_force / self.body_weight_n
            in_contact = normal_force > self.threshold_n
            state = self._gait_state[side]
            last_hs = state.get("last_heel_strike_time")
            cycle_duration = float(state.get("cycle_duration_s") or 0.0)
            phase = (
                float(np.clip((time_s - float(last_hs)) / cycle_duration, 0.0, 1.0))
                if last_hs is not None and cycle_duration > 1e-9
                else 0.0
            )
            grf_sides[side] = {
                "force": [0.0, normal_force, 0.0],
                "moment": [0.0, 0.0, 0.0],
                "cop": [float("nan"), float("nan"), float("nan")],
                "normal_force": normal_force,
                "penetration": 0.0,
                "slip_speed": 0.0,
                "in_contact": bool(in_contact),
                "source": "prescribed_grf",
            }
            event_names = {
                str(event.get("event", "")).lower()
                for event in events
                if str(event.get("side", "")).lower() == side
            }
            gait_sides[side] = {
                "normal_force_n": normal_force,
                "normal_force_bw": normal_bw,
                "in_contact": bool(in_contact),
                "heel_strike": "heel_strike" in event_names,
                "toe_off": "toe_off" in event_names,
                "last_heel_strike_time": last_hs,
                "last_toe_off_time": state.get("last_toe_off_time"),
                "cycle_duration_s": cycle_duration,
                "gait_phase": phase,
            }

        left = grf_sides["left"]
        observation = out.get("observation", {})
        if not isinstance(observation, Mapping):
            observation = {}
        phase_fsm = self.phase_fsm.update(
            time_s=time_s,
            events=events,
            normal_force_bw=float(left["normal_force"]) / self.body_weight_n,
            in_contact=bool(left["in_contact"]),
            prosthetic_knee_angle_rad=observation.get("pros_knee_angle"),
            prosthetic_ankle_angle_rad=observation.get("pros_ankle_angle"),
        )

        out["online_grf_model"] = _jsonable(out.get("online_grf", {}))
        out["online_gait_model"] = _jsonable(out.get("online_gait", {}))
        out["online_events_model"] = _jsonable(out.get("online_events", []))
        out["phase_fsm_model"] = _jsonable(out.get("phase_fsm", {}))
        out["online_grf"] = {"available": True, "source": "prescribed_grf", **grf_sides}
        out["online_gait"] = {
            "available": True,
            "source": "prescribed_grf",
            "sides": gait_sides,
        }
        out["online_events"] = events
        out["phase_fsm"] = phase_fsm
        out["prescribed_reward_input"] = {
            "enabled": True,
            "grf_source": "prescribed_external_loads",
            "kinematics_source": "prescribed_ik_zero_delta",
            "slip_speed_source": "disabled_no_prescribed_slip_measure",
        }
        self._last_probe_time = time_s
        return out


def _build_env(
    args: argparse.Namespace,
    flat: Mapping[str, Any],
    reward_cfg: reward_function.RewardConfig,
    output_dir: Path,
) -> CMCLikeProsthesisTrajectoryEnv:
    setup_xml = args.setup_xml or flat.get("setup_xml")
    if not setup_xml:
        raise ValueError("setup XML not provided and not found in config")
    setup_xml_path = _resolve_repo_path(setup_xml)
    setup_t_start, setup_t_end = _read_setup_times(setup_xml_path)
    if args.start_time < setup_t_start - 1e-9:
        raise ValueError(
            f"start_time {args.start_time} is before setup t_start {setup_t_start}"
        )
    if args.end_time > setup_t_end + 1e-9:
        raise ValueError(f"end_time {args.end_time} is after setup t_end {setup_t_end}")
    if args.end_time <= args.start_time:
        raise ValueError("end_time must be greater than start_time")

    contract = str(args.dynamics_contract)
    if contract == CONTRACT_TRAINING_LIKE:
        online_grf_applied_sides = list(flat.get("online_grf_applied_side", []) or [])
        prescribed_grf_disabled_sides = list(
            flat.get("disable_prescribed_grf_side", []) or []
        )
        prescribed_grf_disabled_sides = sorted(
            set(prescribed_grf_disabled_sides) | set(online_grf_applied_sides)
        )
    else:
        online_grf_applied_sides = []
        prescribed_grf_disabled_sides = []

    return CMCLikeProsthesisTrajectoryEnv(
        CMCEnvConfig(
            setup_xml_path=str(setup_xml_path),
            segment_duration=float(args.segment_duration or flat.get("segment_duration", 0.01)),
            episode_duration=float(args.end_time - args.start_time),
            episode_start_offset_s=float(args.start_time - setup_t_start),
            policy_knots=int(args.policy_knots or flat.get("policy_knots", 1)),
            action_mode="delta",
            max_delta_rad=0.0,
            enable_pros_ref_governor=bool(flat.get("pros_ref_governor", True)),
            pros_ref_model=str(flat.get("pros_ref_model", "butterworth3_jerk_limited")),
            pros_ref_lpf_cutoff_hz=float(flat.get("pros_ref_cutoff_hz", 4.0)),
            pros_ref_velocity_limit_rad_s={
                "pros_knee_angle": float(
                    flat.get("pros_knee_ref_velocity_limit_rad_s", 6.0)
                ),
                "pros_ankle_angle": float(
                    flat.get("pros_ankle_ref_velocity_limit_rad_s", 3.5)
                ),
            },
            pros_ref_acceleration_limit_rad_s2={
                "pros_knee_angle": float(
                    flat.get("pros_knee_ref_acceleration_limit_rad_s2", 60.0)
                ),
                "pros_ankle_angle": float(
                    flat.get("pros_ankle_ref_acceleration_limit_rad_s2", 55.0)
                ),
            },
            pros_ref_jerk_limit_rad_s3={
                "pros_knee_angle": float(
                    flat.get("pros_knee_ref_jerk_limit_rad_s3", 3000.0)
                ),
                "pros_ankle_angle": float(
                    flat.get("pros_ankle_ref_jerk_limit_rad_s3", 2750.0)
                ),
            },
            grf_mode="online_sensor",
            online_grf_profile_file=str(
                args.online_grf_profile
                or flat.get(
                    "online_grf_profile",
                    "online_grf_profiles/"
                    "AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json",
                )
            ),
            online_grf_detector_profile_file=(
                args.online_grf_detector_profile
                if args.online_grf_detector_profile is not None
                else flat.get("online_grf_detector_profile")
            ),
            include_online_grf_observation=True,
            online_grf_applied_sides=online_grf_applied_sides,
            prescribed_grf_disabled_sides=prescribed_grf_disabled_sides,
            gait_clock_enable=False,
            actor_cyclic_phase_only=bool(flat.get("actor_cyclic_phase_only", True)),
            include_reference_state_observation=bool(
                flat.get("include_reference_state_observation", True)
            ),
            deployable_minimal_observation=False,
            imitation_initialize_to_target=False,
            random_init=False,
            fail_fast=True,
            record_outputs=bool(args.record_sim_outputs),
            save_outputs_on_close=bool(args.record_sim_outputs),
            output_dir=str(output_dir / "sim_outputs"),
            output_prefix="prescribed_reward_probe",
            step_wall_timeout_s=float(args.step_wall_timeout_s),
            grf_penetration_penalty_threshold_m=float(
                args.grf_penetration_penalty_threshold_m
                if args.grf_penetration_penalty_threshold_m is not None
                else flat.get("grf_penetration_penalty_threshold_m", 0.012)
            ),
            grf_penetration_termination_m=float(
                args.grf_penetration_termination_m
                if args.grf_penetration_termination_m is not None
                else flat.get("grf_penetration_termination_m", 0.017)
            ),
            grf_ankle_moment_flip_tau_tol_nm=float(
                reward_cfg.grf_ankle_moment_flip_tau_tol_nm
            ),
            grf_ankle_moment_flip_force_threshold_n=float(
                reward_cfg.grf_ankle_moment_flip_force_threshold_n
            ),
            phase_min_stance_duration_s=float(reward_cfg.phase_min_stance_duration_s),
            phase_min_swing_duration_s=float(reward_cfg.phase_min_swing_duration_s),
            phase_landing_window_start_s=float(
                reward_cfg.phase_landing_window_start_s
            ),
            phase_landing_window_end_s=float(reward_cfg.phase_landing_window_end_s),
            phase_stance_hard_timeout_s=float(reward_cfg.phase_stance_hard_timeout_s),
            phase_swing_hard_timeout_s=float(reward_cfg.phase_swing_hard_timeout_s),
            phase_landing_force_full_credit_bw=float(reward_cfg.contact_load_target_bw),
            phase_min_stance_contact_fraction=float(
                reward_cfg.phase_min_stance_contact_fraction
            ),
            phase_min_stance_load_bw_s=float(reward_cfg.phase_min_stance_load_bw_s),
            phase_min_cycle_knee_excursion_rad=float(
                reward_cfg.phase_min_cycle_knee_excursion_rad
            ),
            phase_hs_event_credit=float(reward_cfg.phase_hs_event_credit),
            phase_to_event_credit=float(reward_cfg.phase_to_event_credit),
            phase_cycle_complete_bonus=float(reward_cfg.phase_cycle_complete_bonus),
            phase_failure_extra_penalty=float(reward_cfg.phase_failure_extra_penalty),
        )
    )


def _trace_row(
    step: int,
    reward: float,
    terminated: bool,
    truncated: bool,
    info: Mapping[str, Any],
    reward_cfg: reward_function.RewardConfig,
) -> dict[str, Any]:
    terms = info.get("reward_terms", {})
    if not isinstance(terms, Mapping):
        terms = {}
    components = info.get("reward_components", {})
    if not isinstance(components, Mapping):
        components = {}
    obs = info.get("observation", {})
    if not isinstance(obs, Mapping):
        obs = {}
    fsm = info.get("phase_fsm", {})
    if not isinstance(fsm, Mapping):
        fsm = {}
    gait = info.get("online_gait", {})
    sides = gait.get("sides") if isinstance(gait, Mapping) else {}
    left = sides.get("left") if isinstance(sides, Mapping) else {}
    if not isinstance(left, Mapping):
        left = {}
    online_grf = info.get("online_grf", {})
    online_left = online_grf.get("left", {}) if isinstance(online_grf, Mapping) else {}
    if not isinstance(online_left, Mapping):
        online_left = {}
    state_id = int(float(fsm.get("state_id", terms.get("phase_fsm_state_id", 0.0)) or 0.0))
    grf_slip_term = float(reward_cfg.grf_slip_weight) * float(
        terms.get("grf_slip_loss", 0.0) or 0.0
    )
    return {
        "step": step,
        "time": float(info.get("time", float("nan"))),
        "reward": float(reward),
        "terminated": float(terminated),
        "truncated": float(truncated),
        "end_reason": info.get("end_reason") or "",
        "fsm_state_id": float(state_id),
        "fsm_state_name": STATE_NAMES.get(state_id, "UNKNOWN"),
        "phase_valid_hs_count": terms.get("phase_valid_hs_count", 0.0),
        "phase_valid_to_count": terms.get("phase_valid_to_count", 0.0),
        "phase_valid_cycle_count": terms.get("phase_valid_cycle_count", 0.0),
        "phase_stance_elapsed_s": terms.get("phase_stance_elapsed_s", 0.0),
        "phase_swing_elapsed_s": terms.get("phase_swing_elapsed_s", 0.0),
        "phase_last_period_s": terms.get("phase_last_period_s", 0.0),
        "phase_last_stance_fraction": terms.get("phase_last_stance_fraction", 0.0),
        "phase_timeout_exceeded": terms.get("phase_timeout_exceeded", 0.0),
        "phase_timeout_side": terms.get("phase_timeout_side", 0.0),
        "phase_event_progress_score": terms.get("phase_event_progress_score", 0.0),
        "phase_regular_score": terms.get("phase_regular_score", 0.0),
        "landing_window_active": terms.get("landing_window_active", 0.0),
        "landing_window_contact_score": terms.get(
            "landing_window_contact_score", 0.0
        ),
        "contact_load_score": terms.get("contact_load_score", 0.0),
        "prosthetic_normal_force_bw": terms.get("prosthetic_normal_force_bw", 0.0),
        "prosthetic_slip_speed_m_s": terms.get("prosthetic_slip_speed_m_s", 0.0),
        "grf_slip_loss": terms.get("grf_slip_loss", 0.0),
        "prosthetic_stance_expected": terms.get("prosthetic_stance_expected", 0.0),
        "prosthetic_swing_expected": terms.get("prosthetic_swing_expected", 0.0),
        "policy_action_clip_loss": terms.get("policy_action_clip_loss", 0.0),
        "policy_action_clip_fraction": terms.get(
            "policy_action_clip_fraction", 0.0
        ),
        "oob_loss": components.get("oob_loss", 0.0),
        "oob_term": components.get("oob_term", 0.0),
        "prosthetic_joint_range_loss": terms.get(
            "prosthetic_joint_range_loss", 0.0
        ),
        "prosthetic_joint_range_term": components.get(
            "prosthetic_joint_range_term", 0.0
        ),
        "morphology_available": terms.get("morphology_available", 0.0),
        "morphology_phase": terms.get("morphology_phase", 0.0),
        "fsm_morphology_phase": terms.get("fsm_morphology_phase", 0.0),
        "morphology_loss": terms.get("morphology_loss", 0.0),
        "reward_base": components.get("reward_base", 0.0),
        "phase_timeout_penalty_term": components.get(
            "phase_timeout_penalty_term", 0.0
        ),
        "phase_clawback_penalty_term": components.get(
            "phase_clawback_penalty_term", 0.0
        ),
        "exnovo_task_term": components.get("exnovo_task_term", 0.0),
        "grf_slip_term": grf_slip_term,
        "reward_without_grf_slip": float(reward) + grf_slip_term,
        "penalty": components.get("penalty", 0.0),
        "pros_knee_angle": obs.get("pros_knee_angle", 0.0),
        "pros_ankle_angle": obs.get("pros_ankle_angle", 0.0),
        "pros_knee_angle_served_ref": obs.get("pros_knee_angle_served_ref", 0.0),
        "pros_ankle_angle_served_ref": obs.get("pros_ankle_angle_served_ref", 0.0),
        "left_in_contact": float(bool(left.get("in_contact", False))),
        "online_left_in_contact": float(bool(online_left.get("in_contact", False))),
        "online_left_normal_force_n": float(
            online_left.get("normal_force", 0.0) or 0.0
        ),
        "online_left_penetration_m": float(
            online_left.get("penetration", 0.0) or 0.0
        ),
        "online_left_slip_speed_m_s": float(
            online_left.get("slip_speed", 0.0) or 0.0
        ),
        "left_heel_strike": float(bool(left.get("heel_strike", False))),
        "left_toe_off": float(bool(left.get("toe_off", False))),
        "left_last_heel_strike_time": left.get("last_heel_strike_time") or 0.0,
        "left_last_toe_off_time": left.get("last_toe_off_time") or 0.0,
    }


def run(args: argparse.Namespace) -> dict[str, Any]:
    output_dir = _resolve_output_dir(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    flat, reward_cfg = _load_config(args)
    base = _build_env(args, flat, reward_cfg, output_dir)
    reward_input_source = "env_online_payload"
    reward_input_env: gym.Env = base
    if args.dynamics_contract == CONTRACT_PRESCRIBED_PURE:
        reward_input_env = PrescribedGRFRewardInputWrapper(base, reward_cfg)
        reward_input_source = "prescribed_grf"
    env = reward_function.RewardShapingWrapper(reward_input_env, reward_cfg)

    rewards: list[float] = []
    trace_rows: list[dict[str, Any]] = []
    terms_rows: list[Mapping[str, Any]] = []
    components_rows: list[Mapping[str, Any]] = []
    online_events: list[dict[str, Any]] = []
    final_info: Mapping[str, Any] = {}
    terminated = False
    truncated = False
    oracle_cycles: list[dict[str, float]] = []

    try:
        _, reset_info = env.reset(seed=args.seed)
        final_info = reset_info
        oracle_cycles = _oracle_left_cycles(base)
        max_steps = int(math.ceil((args.end_time - args.start_time) / base.env_cfg.segment_duration)) + 2
        action = np.zeros(base.action_space.shape, dtype=np.float32)
        for step in range(1, max_steps + 1):
            _, reward, terminated, truncated, info = env.step(action)
            final_info = info
            rewards.append(float(reward))
            trace_rows.append(
                _trace_row(step, reward, terminated, truncated, info, reward_cfg)
            )
            terms = info.get("reward_terms")
            if isinstance(terms, Mapping):
                terms_rows.append(dict(terms))
            components = info.get("reward_components")
            if isinstance(components, Mapping):
                components_rows.append(dict(components))
            for event in info.get("online_events", []) or []:
                if isinstance(event, Mapping):
                    online_events.append(
                        {
                            "step": step,
                            "probe_time": float(info.get("time", float("nan"))),
                            **_jsonable(event),
                        }
                    )
            if terminated or truncated:
                break
    finally:
        env.close()

    final_terms = terms_rows[-1] if terms_rows else {}
    final_components = components_rows[-1] if components_rows else {}
    trace_stats = _numeric_stats(trace_rows)
    summary = {
        "ok": True,
        "output_dir": str(output_dir),
        "config": str(training_config.resolve_config_path(args.config)),
        "window": {
            "start_time": float(args.start_time),
            "end_time": float(args.end_time),
            "duration_s": float(args.end_time - args.start_time),
            "segment_duration_s": float(base.env_cfg.segment_duration),
        },
        "dynamics_mode": {
            "contract": str(args.dynamics_contract),
            "reward_input_source": reward_input_source,
            "grf_mode": str(getattr(base.cfg, "grf_mode", "")),
            "online_grf_profile_file": str(
                getattr(base.cfg, "online_grf_profile_file", "")
            ),
            "online_grf_detector_profile_file": str(
                getattr(base.cfg, "online_grf_detector_profile_file", "")
            ),
            "online_grf_applied_sides": list(
                getattr(base.cfg, "online_grf_applied_sides", [])
            ),
            "prescribed_grf_disabled_sides": list(
                getattr(base.cfg, "prescribed_grf_disabled_sides", [])
            ),
            "action_mode": "delta",
            "max_delta_rad": 0.0,
            "grf_penetration_penalty_threshold_m": float(
                base.env_cfg.grf_penetration_penalty_threshold_m
            ),
            "grf_penetration_termination_m": float(
                base.env_cfg.grf_penetration_termination_m
            ),
        },
        "steps": len(rewards),
        "episode_return": float(np.sum(rewards)) if rewards else 0.0,
        "reward_mean": float(np.mean(rewards)) if rewards else 0.0,
        "reward_min": float(np.min(rewards)) if rewards else 0.0,
        "reward_max": float(np.max(rewards)) if rewards else 0.0,
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "end_reason": final_info.get("end_reason") if isinstance(final_info, Mapping) else None,
        "final_fsm": _jsonable(final_info.get("phase_fsm", {}))
        if isinstance(final_info, Mapping)
        else {},
        "final_reward_terms": _jsonable(final_terms),
        "final_reward_components": _jsonable(final_components),
        "trace_stats": trace_stats,
        "reward_terms_stats": _numeric_stats(terms_rows),
        "reward_components_stats": _numeric_stats(components_rows),
        "online_events": online_events,
        "oracle_left_cycles": oracle_cycles,
        "prescribed_reward_input": _jsonable(
            final_info.get("prescribed_reward_input", {})
            if isinstance(final_info, Mapping)
            else {}
        ),
        "reward_config": reward_cfg.to_dict(),
    }

    _write_csv(output_dir / "trace.csv", trace_rows)
    _write_csv(output_dir / "online_events.csv", online_events)
    (output_dir / "summary.json").write_text(
        json.dumps(_jsonable(summary), indent=2),
        encoding="utf-8",
    )
    _write_summary_md(output_dir / "summary.md", summary)
    print(json.dumps(_jsonable(summary), indent=2))
    return summary


def _stat(summary: Mapping[str, Any], section: str, key: str, field: str = "mean") -> str:
    try:
        value = summary[section][key][field]
    except Exception:
        return "n/a"
    if isinstance(value, float):
        return f"{value:.6g}"
    return str(value)


def _write_summary_md(path: Path, summary: Mapping[str, Any]) -> None:
    final_fsm = summary.get("final_fsm", {})
    if not isinstance(final_fsm, Mapping):
        final_fsm = {}
    dynamics = summary.get("dynamics_mode", {})
    if not isinstance(dynamics, Mapping):
        dynamics = {}
    lines = [
        "# Prescribed Reward Probe",
        "",
        f"- Window: {summary['window']['start_time']:.3f} -> "
        f"{summary['window']['end_time']:.3f} s",
        f"- Steps: {summary['steps']}",
        f"- Return: {summary['episode_return']:.6g}",
        f"- Reward mean: {summary['reward_mean']:.6g}",
        f"- End: terminated={summary['terminated']}, "
        f"truncated={summary['truncated']}, reason={summary.get('end_reason')}",
        "",
        "## Dynamics Contract",
        "",
        f"- Contract: {dynamics.get('contract', 'n/a')}",
        f"- Reward input source: {dynamics.get('reward_input_source', 'n/a')}",
        f"- GRF mode: {dynamics.get('grf_mode', 'n/a')}",
        f"- Online applied sides: {dynamics.get('online_grf_applied_sides', [])}",
        f"- Prescribed disabled sides: "
        f"{dynamics.get('prescribed_grf_disabled_sides', [])}",
        "",
        "## FSM",
        "",
        f"- Final state: {final_fsm.get('state_name', 'n/a')} "
        f"({final_fsm.get('state_id', 'n/a')})",
        f"- Valid HS: {final_fsm.get('valid_hs_count', 'n/a')}",
        f"- Valid TO: {final_fsm.get('valid_to_count', 'n/a')}",
        f"- Valid cycles: {final_fsm.get('valid_cycle_count', 'n/a')}",
        f"- Last period: {final_fsm.get('last_period_s', 'n/a')} s",
        f"- Last stance fraction: {final_fsm.get('last_stance_fraction', 'n/a')}",
        f"- Timeout: {final_fsm.get('timeout_exceeded', 'n/a')} "
        f"side={final_fsm.get('timeout_side', 'n/a')}",
        "",
        "## Key Means",
        "",
        f"- reward_base: {_stat(summary, 'reward_components_stats', 'reward_base')}",
        f"- reward_without_grf_slip: {_stat(summary, 'trace_stats', 'reward_without_grf_slip')}",
        f"- grf_slip_term: {_stat(summary, 'trace_stats', 'grf_slip_term')}",
        f"- grf_slip_loss: {_stat(summary, 'reward_terms_stats', 'grf_slip_loss')}",
        f"- prosthetic_slip_speed_m_s: {_stat(summary, 'reward_terms_stats', 'prosthetic_slip_speed_m_s')}",
        f"- contact_load_score: {_stat(summary, 'reward_terms_stats', 'contact_load_score')}",
        f"- landing_window_active: {_stat(summary, 'reward_terms_stats', 'landing_window_active')}",
        f"- landing_window_contact_score: {_stat(summary, 'reward_terms_stats', 'landing_window_contact_score')}",
        f"- phase_regular_score: {_stat(summary, 'reward_terms_stats', 'phase_regular_score')}",
        f"- phase_timeout_loss: {_stat(summary, 'reward_terms_stats', 'phase_timeout_loss')}",
        f"- policy_action_clip_loss: {_stat(summary, 'reward_terms_stats', 'policy_action_clip_loss')}",
        f"- prosthetic_joint_range_loss: {_stat(summary, 'reward_terms_stats', 'prosthetic_joint_range_loss')}",
        f"- oob_term: {_stat(summary, 'reward_components_stats', 'oob_term')}",
        "",
        "## Files",
        "",
        "- `summary.json`",
        "- `trace.csv`",
        "- `online_events.csv`",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--setup-xml", default=None)
    parser.add_argument("--reward-json", default=None)
    parser.add_argument(
        "--dynamics-contract",
        choices=CONTRACTS,
        default=CONTRACT_PRESCRIBED_PURE,
        help=(
            "prescribed_pure keeps prescribed GRFs applied on both sides and "
            "uses onlineGRF only as a sensor; training_like uses the GRF "
            "application/disabled-side lists from the YAML."
        ),
    )
    parser.add_argument("--start-time", type=float, default=12.99)
    parser.add_argument("--end-time", type=float, default=17.99)
    parser.add_argument("--segment-duration", type=float, default=None)
    parser.add_argument("--policy-knots", type=int, default=None)
    parser.add_argument("--online-grf-profile", default=None)
    parser.add_argument("--online-grf-detector-profile", default=None)
    parser.add_argument("--seed", type=int, default=123)
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--step-wall-timeout-s", type=float, default=60.0)
    parser.add_argument(
        "--grf-penetration-penalty-threshold-m",
        type=float,
        default=None,
        help="Soft penetration threshold used by the env during this probe.",
    )
    parser.add_argument(
        "--grf-penetration-termination-m",
        type=float,
        default=None,
        help="Hard penetration termination used by the env during this probe.",
    )
    parser.add_argument(
        "--record-sim-outputs",
        action="store_true",
        help="Also write full .sto simulator outputs under output_dir/sim_outputs.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    run(parse_args())
