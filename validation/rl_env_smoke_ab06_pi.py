"""
Smoke tests for osim_trj_cmc_like.py on the AB06 SEA PI setup.

Run:
    conda run -n envCMC-like python validation/rl_env_smoke_ab06_pi.py

Optional recorded-output smoke:
    conda run -n envCMC-like python validation/rl_env_smoke_ab06_pi.py --recorded-output

Inspect existing simulator outputs without constructing the AB06 env:
    conda run -n envCMC-like python validation/rl_env_smoke_ab06_pi.py --inspect-existing-results
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
# The RL adapter was relocated under "Trajectory Generator/" alongside the SNN
# trajectory-generator code. Keep it importable from there too; the repo root
# still takes precedence so the simulator modules (config, simulation_runner,
# ...) resolve against the canonical tree.
_TRAJ_GEN_DIR = REPO_ROOT / "Trajectory Generator"
if _TRAJ_GEN_DIR.is_dir() and str(_TRAJ_GEN_DIR) not in sys.path:
    sys.path.append(str(_TRAJ_GEN_DIR))

from osim_trj_cmc_like import CMCEnvConfig, CMCLikeProsthesisTrajectoryEnv


SETUP_XML = "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml"
ONLINE_GRF_PROFILE = (
    "online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_sensor_basis.json"
)
RECORDED_OUTPUT_DIR = "results/_rl_env_ab06_pi_smoke_20260529"
EXISTING_RESULTS_DIR = "results"


def _preflight_setup_files() -> None:
    setup_path = REPO_ROOT / SETUP_XML
    if not setup_path.is_file():
        raise FileNotFoundError(f"Setup XML not found: {setup_path}")

    root = ET.parse(setup_path).getroot()
    values = {
        child.tag.rsplit("}", 1)[-1]: (child.text or "").strip()
        for node in root
        for child in node
    }
    required = (
        "model_file",
        "kinematics_file",
        "external_loads_xml",
        "reserve_actuators_xml",
    )
    missing = []
    for key in required:
        raw = values.get(key, "")
        path = Path(raw)
        if not path.is_absolute():
            path = REPO_ROOT / raw
        if not path.is_file():
            missing.append((key, str(path)))

    if missing:
        lines = ["AB06 PI setup references missing file(s):"]
        lines.extend(f"  {key}: {path}" for key, path in missing)
        raise FileNotFoundError("\n".join(lines))


def _make_env(
    *,
    recorded_output: bool = False,
    segment_duration: float = 0.002,
    episode_duration: float = 0.002,
    online_grf: bool = False,
) -> CMCLikeProsthesisTrajectoryEnv:
    return CMCLikeProsthesisTrajectoryEnv(
        CMCEnvConfig(
            setup_xml_path=SETUP_XML,
            segment_duration=segment_duration,
            policy_knots=1,
            episode_duration=episode_duration,
            action_mode="delta",
            record_outputs=recorded_output,
            save_outputs_on_close=recorded_output,
            output_dir=RECORDED_OUTPUT_DIR if recorded_output else None,
            rebuild_model_on_reset=False,
            fail_fast=True,
            grf_mode="online_sensor" if online_grf else None,
            online_grf_profile_file=ONLINE_GRF_PROFILE if online_grf else None,
            include_online_grf_observation=online_grf,
        )
    )


def _assert_finite_obs(obs: np.ndarray, label: str) -> None:
    if not np.all(np.isfinite(obs)):
        bad = np.flatnonzero(~np.isfinite(obs))
        raise AssertionError(f"{label}: non-finite observation at indices {bad[:10]}")


def _assert_step_ok(
    obs: np.ndarray,
    reward: float,
    terminated: bool,
    truncated: bool,
    info: dict,
    *,
    expected_time: float,
    label: str,
) -> None:
    _assert_finite_obs(obs, label)
    if not math.isfinite(float(reward)):
        raise AssertionError(f"{label}: reward is not finite: {reward}")
    if terminated:
        raise AssertionError(f"{label}: terminated=True, info={info}")
    if not truncated:
        raise AssertionError(f"{label}: expected truncated=True for short episode")
    if info.get("end_reason") != "episode_time_limit":
        raise AssertionError(f"{label}: unexpected end_reason={info.get('end_reason')}")
    if abs(float(info["time"]) - expected_time) > 5e-7:
        raise AssertionError(
            f"{label}: time {info['time']} != expected {expected_time}"
        )
    if "reward_terms" not in info:
        raise AssertionError(f"{label}: missing reward_terms")
    if "policy_segment_values" not in info:
        raise AssertionError(f"{label}: missing policy_segment_values")
    if not np.all(np.isfinite(info["policy_segment_values"])):
        raise AssertionError(f"{label}: non-finite policy_segment_values")


def run_single_step_smoke() -> None:
    env = _make_env()
    try:
        obs, info = env.reset(seed=1)
        print("reset_time", info["time"])
        print("obs_shape", obs.shape)
        print("obs_finite", np.all(np.isfinite(obs)))
        print("action_shape", env.action_space.shape)
        _assert_finite_obs(obs, "single_step_reset")
        if abs(float(info["time"]) - 11.99) > 1e-9:
            raise AssertionError(f"reset_time {info['time']} != 11.99")
        if tuple(env.action_space.shape) != (1, 2):
            raise AssertionError(f"unexpected action shape: {env.action_space.shape}")

        action = np.zeros(env.action_space.shape, dtype=np.float32)
        obs, reward, terminated, truncated, info = env.step(action)
        print("step_time", info["time"])
        print("reward", reward)
        print("terminated", terminated)
        print("truncated", truncated)
        print("obs_finite", np.all(np.isfinite(obs)))
        print("reward_terms", info["reward_terms"])
        _assert_step_ok(
            obs,
            reward,
            terminated,
            truncated,
            info,
            expected_time=11.992,
            label="single_step_zero",
        )
    finally:
        env.close()


def run_reset_reuse_benchmark() -> None:
    env = _make_env()
    try:
        timings = []
        for i in range(3):
            start = time.perf_counter()
            obs, info = env.reset(seed=i)
            elapsed = time.perf_counter() - start
            timings.append(elapsed)
            _assert_finite_obs(obs, f"reset_reuse_{i}")
            if abs(float(info["time"]) - 11.99) > 1e-9:
                raise AssertionError(f"reset {i}: time {info['time']} != 11.99")
        print("reset_reuse_times_s", [round(v, 6) for v in timings])
    finally:
        env.close()


def run_action_sanity() -> None:
    for value in (0.0, 0.1, -0.1):
        env = _make_env()
        try:
            obs, _info = env.reset(seed=2)
            _assert_finite_obs(obs, f"action_{value}_reset")
            action = np.full(env.action_space.shape, value, dtype=np.float32)
            obs, reward, terminated, truncated, info = env.step(action)
            print(
                "action_sanity",
                value,
                "time",
                info["time"],
                "reward",
                reward,
                "terminated",
                terminated,
                "truncated",
                truncated,
            )
            _assert_step_ok(
                obs,
                reward,
                terminated,
                truncated,
                info,
                expected_time=11.992,
                label=f"action_{value}",
            )
        finally:
            env.close()


def _read_numeric_rows(path: Path) -> np.ndarray:
    rows: list[list[float]] = []
    header_done = False
    with path.open("r", encoding="utf-8", errors="replace") as fh:
        for raw in fh:
            line = raw.strip()
            if not line:
                continue
            if line.lower() == "endheader":
                header_done = True
                continue
            if not header_done:
                continue
            try:
                rows.append([float(value) for value in line.split()])
            except ValueError:
                continue
    if not rows:
        raise AssertionError(f"No numeric rows found in {path}")
    return np.asarray(rows, dtype=float)


def run_recorded_output_smoke() -> None:
    env = _make_env(recorded_output=True)
    try:
        obs, _info = env.reset(seed=3)
        _assert_finite_obs(obs, "recorded_reset")
        action = np.zeros(env.action_space.shape, dtype=np.float32)
        obs, reward, terminated, truncated, info = env.step(action)
        _assert_step_ok(
            obs,
            reward,
            terminated,
            truncated,
            info,
            expected_time=11.992,
            label="recorded_output",
        )
    finally:
        env.close()

    out_dir = Path(RECORDED_OUTPUT_DIR)
    if not out_dir.is_dir():
        raise AssertionError(f"Recorded output directory missing: {out_dir}")

    sto_files = sorted(out_dir.glob("*.sto"))
    if not sto_files:
        raise AssertionError(f"No .sto files written in {out_dir}")
    print("recorded_output_files", [path.name for path in sto_files])

    for path in sto_files[:5]:
        data = _read_numeric_rows(path)
        if not np.any(np.isfinite(data)):
            raise AssertionError(f"No finite numeric values in {path}")


def run_online_grf_observation_smoke() -> None:
    env = _make_env(
        segment_duration=0.08,
        episode_duration=0.08,
        online_grf=True,
    )
    try:
        obs, info = env.reset(seed=4)
        _assert_finite_obs(obs, "online_grf_reset")
        if info.get("grf_mode") != "online_sensor":
            raise AssertionError(f"unexpected grf_mode: {info.get('grf_mode')}")

        feature_names = tuple(env.observation_feature_names)
        required_features = {
            "online_left_normal_grf_bw",
            "online_left_heel_strike",
            "online_left_gait_phase",
            "online_right_normal_grf_bw",
            "online_right_heel_strike",
            "online_right_gait_phase",
        }
        missing = required_features.difference(feature_names)
        if missing:
            raise AssertionError(f"missing onlineGRF observation features: {missing}")

        action = np.zeros(env.action_space.shape, dtype=np.float32)
        obs, reward, terminated, truncated, info = env.step(action)
        _assert_step_ok(
            obs,
            reward,
            terminated,
            truncated,
            info,
            expected_time=12.07,
            label="online_grf_observation",
        )
        gait = info.get("online_gait")
        if not isinstance(gait, dict) or not gait.get("available"):
            raise AssertionError(f"missing online_gait payload: {gait}")
        events = info.get("online_events")
        if not isinstance(events, list) or not any(
            event.get("event") == "heel_strike" for event in events
        ):
            raise AssertionError(f"missing confirmed heel strike: {events}")
        for side in ("left", "right"):
            normal_bw = float(gait["sides"][side]["normal_force_bw"])
            if not math.isfinite(normal_bw) or normal_bw < 0.0:
                raise AssertionError(f"invalid {side} normal_force_bw={normal_bw}")
        print("online_grf_events", events)
        print("online_grf_observation_shape", obs.shape)
    finally:
        env.close()


def inspect_existing_results(max_files: int = 10) -> None:
    """Check already-written simulator .sto outputs without using them as inputs."""
    results_dir = REPO_ROOT / EXISTING_RESULTS_DIR
    if not results_dir.is_dir():
        raise FileNotFoundError(f"Results directory not found: {results_dir}")

    sto_files = sorted(results_dir.rglob("sim_output_*.sto"))
    if not sto_files:
        raise FileNotFoundError(f"No sim_output_*.sto files found in {results_dir}")

    checked = 0
    for path in sto_files[:max_files]:
        data = _read_numeric_rows(path)
        finite = bool(np.isfinite(data).all())
        print(
            "existing_result",
            path.relative_to(REPO_ROOT),
            "rows",
            data.shape[0],
            "cols",
            data.shape[1],
            "finite",
            finite,
        )
        if not finite:
            raise AssertionError(f"Non-finite values found in {path}")
        checked += 1

    print("existing_results_checked", checked, "of", len(sto_files))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--recorded-output",
        action="store_true",
        help="Also run the recorded-output smoke and inspect written .sto files.",
    )
    parser.add_argument(
        "--inspect-existing-results",
        action="store_true",
        help="Inspect already-written results/sim_output_*.sto files without constructing the AB06 env.",
    )
    parser.add_argument(
        "--online-grf",
        action="store_true",
        help="Also verify online_sensor GRF/events in the RL observation and info.",
    )
    parser.add_argument(
        "--max-result-files",
        type=int,
        default=10,
        help="Maximum number of existing .sto output files to inspect.",
    )
    args = parser.parse_args()

    print("setup_xml", SETUP_XML)
    if args.inspect_existing_results:
        inspect_existing_results(max_files=args.max_result_files)
        if not args.recorded_output:
            return

    _preflight_setup_files()
    run_single_step_smoke()
    run_reset_reuse_benchmark()
    run_action_sanity()
    if args.online_grf:
        run_online_grf_observation_smoke()
    if args.recorded_output:
        run_recorded_output_smoke()
    print("rl_env_smoke_ab06_pi_ok")


if __name__ == "__main__":
    main()
