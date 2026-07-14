"""Roll out the phase-based imitation target without a learned policy.

This is the plant-feasibility gate for imitation training: it sends the corrected
periodic target through the same absolute-action mapping, reference LPF,
prosthesis controller, SEA plugin, and hybrid-GRF dynamics used by PPO.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from datetime import datetime
from pathlib import Path
from typing import Mapping
from xml.etree import ElementTree as ET

import numpy as np

THIS_DIR = Path(__file__).resolve().parent
TRAJ_GEN_DIR = THIS_DIR.parent
REPO_ROOT = TRAJ_GEN_DIR.parent
RUNS_ROOT = TRAJ_GEN_DIR / "runs"
ROLLOUT_RUNS_ROOT = RUNS_ROOT / "rollout"
for path in (THIS_DIR, TRAJ_GEN_DIR, REPO_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import _bootstrap  # noqa: E402

_bootstrap.ensure_sim_paths()

import reward_function  # noqa: E402
from osim_trj_cmc_like import CMCEnvConfig, CMCLikeProsthesisTrajectoryEnv  # noqa: E402


DEFAULT_SETUP = (
    "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml"
)
DEFAULT_ONLINE_GRF_PROFILE = (
    "online_grf_profiles/"
    "AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json"
)


def _cli_path(value: str | Path) -> Path:
    text = os.fspath(value)
    if os.name != "nt":
        text = text.replace("\\", "/")
    return Path(text).expanduser()


def _resolve_output_dir(value: str | None) -> Path:
    if value:
        path = _cli_path(value)
        if path.is_absolute():
            resolved = path.resolve()
            try:
                rel = resolved.relative_to(RUNS_ROOT.resolve())
            except ValueError:
                return resolved
            if rel.parts and rel.parts[0].lower() == "rollout":
                return resolved
            return (ROLLOUT_RUNS_ROOT / rel).resolve()
        parts = path.parts
        if parts and parts[0].lower() == "trajectory generator":
            parts = parts[1:]
        if parts and parts[0].lower() == "runs":
            parts = parts[1:]
        if parts and parts[0].lower() == "rollout":
            parts = parts[1:]
        path = ROLLOUT_RUNS_ROOT / (Path(*parts) if parts else Path())
    else:
        path = ROLLOUT_RUNS_ROOT / f"imitation_oracle_{datetime.now():%Y%m%d_%H%M%S}"
    return path.resolve()


def _local_tag(element: ET.Element) -> str:
    return element.tag.rsplit("}", 1)[-1]


def _setup_with_knee_kp(
    setup_xml: str,
    knee_kp: float | None,
    output_dir: Path,
) -> str:
    if knee_kp is None:
        return setup_xml
    setup_path = _cli_path(setup_xml)
    if not setup_path.is_absolute():
        setup_path = REPO_ROOT / setup_path
    setup_tree = ET.parse(setup_path)
    model_node = next(
        node for node in setup_tree.getroot().iter() if _local_tag(node) == "model_file"
    )
    model_path = Path((model_node.text or "").strip())
    if not model_path.is_absolute():
        model_path = REPO_ROOT / model_path

    model_tree = ET.parse(model_path)
    changed = False
    for actuator in model_tree.getroot().iter():
        if _local_tag(actuator) != "SeriesElasticActuator":
            continue
        if actuator.attrib.get("name") != "SEA_Knee":
            continue
        for child in actuator:
            if _local_tag(child) == "Kp":
                child.text = f"{float(knee_kp):.12g}"
                changed = True
                break
    if not changed:
        raise RuntimeError("Could not find SEA_Knee/Kp in the model XML")

    variant_dir = output_dir / "model_variant"
    variant_dir.mkdir(parents=True, exist_ok=True)
    variant_model = variant_dir / f"{model_path.stem}_knee_kp_{knee_kp:g}.osim"
    variant_setup = variant_dir / f"{setup_path.stem}_knee_kp_{knee_kp:g}.xml"
    model_tree.write(variant_model, encoding="utf-8", xml_declaration=True)
    model_node.text = str(variant_model.resolve())
    setup_tree.write(variant_setup, encoding="utf-8", xml_declaration=True)
    return str(variant_setup.resolve())


def _oracle_action(env: CMCLikeProsthesisTrajectoryEnv, target_t: float) -> np.ndarray:
    n = int(env.env_cfg.policy_knots)
    future = env.t + (
        np.arange(1, n + 1, dtype=float) / float(n)
    ) * max(float(target_t) - env.t, env.cfg.dt)
    values = np.empty((n, len(env.cfg.pros_coords)), dtype=float)
    for i, time_value in enumerate(future):
        q_target, _, _ = env.imitation_target(float(time_value))
        for j, coord_name in enumerate(env.cfg.pros_coords):
            values[i, j] = q_target[coord_name]

    mode = env.env_cfg.action_mode.lower()
    if mode == "raw":
        return values
    if mode == "delta":
        action = np.empty_like(values)
        for i, time_value in enumerate(future):
            q_base, _, _ = env.base_kin.get(float(time_value))
            for j, coord_name in enumerate(env.cfg.pros_coords):
                action[i, j] = (
                    values[i, j] - q_base[coord_name]
                ) / env._delta_scale[j]
        return np.clip(action, -1.0, 1.0)
    if mode != "absolute":
        raise ValueError(f"Unsupported action_mode: {env.env_cfg.action_mode}")

    bounds = env.env_cfg.absolute_bounds_rad
    if bounds is None:
        raise ValueError("absolute_bounds_rad is required for oracle absolute mode")
    action = np.empty_like(values)
    for j, coord_name in enumerate(env.cfg.pros_coords):
        low, high = bounds[coord_name]
        action[:, j] = 2.0 * (values[:, j] - low) / (high - low) - 1.0
    return np.clip(action, -1.0, 1.0)


def _numeric_means(rows: list[Mapping[str, object]]) -> dict[str, float]:
    keys = sorted({key for row in rows for key in row})
    result: dict[str, float] = {}
    for key in keys:
        values = []
        for row in rows:
            try:
                value = float(row[key])
            except (KeyError, TypeError, ValueError):
                continue
            if math.isfinite(value):
                values.append(value)
        if values:
            result[key] = float(np.mean(values))
    return result


def run(args: argparse.Namespace) -> dict:
    output_dir = _resolve_output_dir(args.output_dir)
    sim_outputs = output_dir / "sim_outputs"
    output_dir.mkdir(parents=True, exist_ok=True)
    setup_xml = _setup_with_knee_kp(args.setup_xml, args.knee_kp, output_dir)
    reward_overrides = reward_function.load_reward_overrides(args.reward_json) or {}
    reward_overrides["reward_mode"] = "imitation"
    reward_cfg = reward_function.RewardConfig.from_mapping(reward_overrides)

    base = CMCLikeProsthesisTrajectoryEnv(
        CMCEnvConfig(
            setup_xml_path=setup_xml,
            segment_duration=args.segment_duration,
            episode_duration=args.episode_duration,
            policy_knots=args.policy_knots,
            action_mode="absolute",
            enable_pros_ref_governor=args.pros_ref_governor,
            pros_ref_model=args.pros_ref_model,
            pros_ref_lpf_cutoff_hz=args.pros_ref_cutoff_hz,
            pros_ref_velocity_limit_rad_s={
                "pros_knee_angle": args.pros_knee_ref_velocity_limit_rad_s,
                "pros_ankle_angle": args.pros_ankle_ref_velocity_limit_rad_s,
            },
            pros_ref_acceleration_limit_rad_s2={
                "pros_knee_angle": args.pros_knee_ref_acceleration_limit_rad_s2,
                "pros_ankle_angle": args.pros_ankle_ref_acceleration_limit_rad_s2,
            },
            pros_ref_jerk_limit_rad_s3={
                "pros_knee_angle": args.pros_knee_ref_jerk_limit_rad_s3,
                "pros_ankle_angle": args.pros_ankle_ref_jerk_limit_rad_s3,
            },
            actor_cyclic_phase_only=args.actor_cyclic_phase_only,
            include_reference_state_observation=args.include_reference_state_observation,
            random_init=False,
            imitation_initialize_to_target=args.imitation_initialize_to_target,
            fail_fast=True,
            record_outputs=True,
            save_outputs_on_close=True,
            output_dir=str(sim_outputs),
            output_prefix="oracle_episode",
            grf_mode=args.grf_mode,
            online_grf_profile_file=args.online_grf_profile,
            include_online_grf_observation=True,
            online_grf_applied_sides=args.online_grf_applied_side,
            step_wall_timeout_s=args.step_wall_timeout_s,
        )
    )
    env = reward_function.RewardShapingWrapper(base, reward_cfg)
    rewards: list[float] = []
    reward_terms: list[Mapping[str, object]] = []
    reward_components: list[Mapping[str, object]] = []
    terminated = truncated = False
    info: dict = {}
    try:
        env.reset(seed=args.seed)
        max_steps = int(math.ceil(args.episode_duration / args.segment_duration)) + 2
        for _ in range(max_steps):
            target_t = min(base.t + args.segment_duration, base._episode_end)
            action = _oracle_action(base, target_t)
            _, reward, terminated, truncated, info = env.step(action)
            rewards.append(float(reward))
            terms = info.get("reward_terms")
            if isinstance(terms, Mapping):
                reward_terms.append(terms)
            components = info.get("reward_components")
            if isinstance(components, Mapping):
                reward_components.append(components)
            if terminated or truncated:
                break
    finally:
        env.close()

    summary = {
        "ok": True,
        "steps": len(rewards),
        "episode_return": float(np.sum(rewards)),
        "reward_mean": float(np.mean(rewards)) if rewards else 0.0,
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "end_reason": info.get("end_reason"),
        "output_dir": str(output_dir),
        "knee_kp_override": args.knee_kp,
        "reference_governor": {
            "enabled": bool(args.pros_ref_governor),
            "model": args.pros_ref_model,
            "cutoff_hz": args.pros_ref_cutoff_hz,
            "velocity_limit_rad_s": {
                "pros_knee_angle": args.pros_knee_ref_velocity_limit_rad_s,
                "pros_ankle_angle": args.pros_ankle_ref_velocity_limit_rad_s,
            },
            "acceleration_limit_rad_s2": {
                "pros_knee_angle": args.pros_knee_ref_acceleration_limit_rad_s2,
                "pros_ankle_angle": args.pros_ankle_ref_acceleration_limit_rad_s2,
            },
        },
        "imitation_target": base._imitation_target_summary(),
        "reward_config": reward_cfg.to_dict(),
        "reward_terms_mean": _numeric_means(reward_terms),
        "reward_components_mean": _numeric_means(reward_components),
    }
    summary_path = output_dir / "oracle_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--setup-xml", default=DEFAULT_SETUP)
    parser.add_argument("--knee-kp", type=float, default=None)
    parser.add_argument("--output-dir")
    parser.add_argument("--segment-duration", type=float, default=0.01)
    parser.add_argument("--episode-duration", type=float, default=2.0)
    parser.add_argument("--policy-knots", type=int, default=3)
    parser.add_argument(
        "--pros-ref-governor",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument(
        "--pros-ref-model",
        choices=("second_order", "butterworth3_jerk_limited"),
        default="second_order",
    )
    parser.add_argument("--pros-ref-cutoff-hz", type=float, default=6.0)
    parser.add_argument(
        "--pros-knee-ref-velocity-limit-rad-s", type=float, default=6.0
    )
    parser.add_argument(
        "--pros-ankle-ref-velocity-limit-rad-s", type=float, default=3.5
    )
    parser.add_argument(
        "--pros-knee-ref-acceleration-limit-rad-s2", type=float, default=60.0
    )
    parser.add_argument(
        "--pros-ankle-ref-acceleration-limit-rad-s2", type=float, default=55.0
    )
    parser.add_argument(
        "--pros-knee-ref-jerk-limit-rad-s3", type=float, default=3000.0
    )
    parser.add_argument(
        "--pros-ankle-ref-jerk-limit-rad-s3", type=float, default=2750.0
    )
    parser.add_argument(
        "--actor-cyclic-phase-only",
        action=argparse.BooleanOptionalAction,
        default=False,
    )
    parser.add_argument(
        "--include-reference-state-observation",
        action=argparse.BooleanOptionalAction,
        default=False,
    )
    parser.add_argument(
        "--imitation-initialize-to-target",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--seed", type=int, default=123)
    parser.add_argument("--grf-mode", default="online_sensor")
    parser.add_argument("--online-grf-profile", default=DEFAULT_ONLINE_GRF_PROFILE)
    parser.add_argument(
        "--online-grf-applied-side",
        nargs="*",
        choices=("left", "right"),
        default=["left"],
    )
    parser.add_argument(
        "--reward-json",
        default=str(THIS_DIR / "reward_imitation.json"),
    )
    parser.add_argument("--step-wall-timeout-s", type=float, default=60.0)
    return parser.parse_args()


if __name__ == "__main__":
    run(parse_args())
