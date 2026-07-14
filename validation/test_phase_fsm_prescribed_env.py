"""Smoke the prosthetic phase FSM in the real CMC-like env.

This uses prescribed GRF for dynamics while keeping the online detector as a
sensor (`grf_mode=online_sensor`, no applied online sides). The goal is to prove
that the env-owned FSM sees a prescribed-data HS-TO-HS sequence and exposes it
in the actor observation and `info["phase_fsm"]`.
"""

from __future__ import annotations

import sys
import os
from pathlib import Path

import numpy as np


REPO = Path(__file__).resolve().parents[1]
TRAJECTORY_DIR = REPO / "Trajectory Generator"
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(TRAJECTORY_DIR))


def _oracle_left_cycles(env) -> list[tuple[float, float, float]]:
    from output import _cycles_from_vertical_grf, _read_storage_table

    grf_file = getattr(env.ctx, "grf_data_file", "")
    columns = getattr(env.ctx, "grf_vertical_force_columns", {})
    source_col = columns.get("left") if isinstance(columns, dict) else None
    if not grf_file or source_col is None:
        return []
    time, col_names, data = _read_storage_table(grf_file)
    col_idx = {name: i for i, name in enumerate(col_names)}
    idx = col_idx[source_col]
    return _cycles_from_vertical_grf(
        time,
        data[:, idx],
        float(env.cfg.grf_contact_threshold_n),
        float(env._episode_start),
        float(env._episode_end),
        float(getattr(env.cfg, "grf_min_contact_duration_s", 0.0)),
        float(getattr(env.cfg, "grf_min_cycle_duration_s", 0.0)),
    )


def main() -> int:
    try:
        from osim_trj_cmc_like import CMCEnvConfig, CMCLikeProsthesisTrajectoryEnv
    except Exception as exc:
        print(f"[SKIP] OpenSim env unavailable: {type(exc).__name__}: {exc}")
        return 0

    env = CMCLikeProsthesisTrajectoryEnv(
        CMCEnvConfig(
            setup_xml_path=(
                "models/AB06_SEASEA_Threadmill/"
                "AB06_SEASEA_stiff321_500_pi_setup.xml"
            ),
            segment_duration=0.02,
            episode_duration=2.4,
            episode_start_offset_s=1.8,
            policy_knots=1,
            action_mode="delta",
            max_delta_rad=0.0,
            grf_mode="online_sensor",
            online_grf_profile_file=(
                "online_grf_profiles/"
                "AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json"
            ),
            online_grf_detector_profile_file=(
                "online_grf_profiles/"
                "AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json"
            ),
            include_online_grf_observation=True,
            online_grf_applied_sides=[],
            prescribed_grf_disabled_sides=[],
            gait_clock_enable=False,
            actor_cyclic_phase_only=True,
            include_reference_state_observation=False,
            deployable_minimal_observation=True,
            fail_fast=True,
            step_wall_timeout_s=60.0,
        )
    )
    try:
        obs, info = env.reset(seed=123)
        oracle_cycles = _oracle_left_cycles(env)
        assert oracle_cycles, "No prescribed left HS-HS oracle cycle in test window."

        actor_names = set(env.actor_feature_names)
        for name in (
            "pros_knee_angle",
            "pros_ankle_angle",
            "phase_fsm_wait_hs",
            "phase_fsm_stance_after_hs",
            "phase_fsm_swing_after_to",
            "phase_expected_hs",
            "phase_expected_to",
            "phase_cycle_progress_credit",
        ):
            assert name in actor_names, f"Missing actor feature {name}"
        for forbidden in (
            "pros_knee_angle_vel",
            "SEA_Knee_motor_angle",
            "pros_knee_angle_served_ref",
            "pros_knee_angle_previous_endpoint",
        ):
            assert forbidden not in actor_names, f"Non-deployable actor feature {forbidden}"

        final_info = info
        if os.environ.get("CMC_FULL_FSM_PRESCRIBED_TEST") == "1":
            action = np.zeros(env.action_space.shape, dtype=np.float32)
            for _ in range(140):
                obs, reward, terminated, truncated, final_info = env.step(action)
                fsm = final_info.get("phase_fsm", {})
                if float(fsm.get("valid_cycle_count", 0.0) or 0.0) >= 1.0:
                    break
                if terminated or truncated:
                    break
        else:
            cycle_start, cycle_end, contact_duration = oracle_cycles[0]
            toe_off = float(cycle_start + contact_duration)
            env._phase_fsm.update(
                time_s=float(cycle_start),
                events=[
                    {"side": "left", "event": "heel_strike", "time": float(cycle_start)}
                ],
                normal_force_bw=0.7,
                in_contact=True,
            )
            env._phase_fsm.update(
                time_s=toe_off,
                events=[{"side": "left", "event": "toe_off", "time": toe_off}],
                normal_force_bw=0.0,
                in_contact=False,
            )
            payload = env._phase_fsm.update(
                time_s=float(cycle_end),
                events=[
                    {"side": "left", "event": "heel_strike", "time": float(cycle_end)}
                ],
                normal_force_bw=0.7,
                in_contact=True,
            )
            env._phase_fsm_payload = payload
            obs, obs_dict = env._get_observation()
            final_info = {
                "observation": obs_dict,
                "phase_fsm": payload,
                "online_grf_detector": {},
            }

        fsm = final_info.get("phase_fsm", {})
        assert float(fsm.get("valid_hs_count", 0.0) or 0.0) >= 2.0, fsm
        assert float(fsm.get("valid_to_count", 0.0) or 0.0) >= 1.0, fsm
        assert float(fsm.get("valid_cycle_count", 0.0) or 0.0) >= 1.0, fsm
        last_period = float(fsm.get("last_period_s", 0.0) or 0.0)
        oracle_periods = [float(end - start) for start, end, _ in oracle_cycles]
        assert min(abs(last_period - period) for period in oracle_periods) <= 0.15, (
            last_period,
            oracle_periods,
        )
        assert "phase_fsm" in final_info
        assert "online_grf_detector" in final_info
        print("[PASS] prescribed CMC-like FSM smoke")
        return 0
    finally:
        env.close()


if __name__ == "__main__":
    raise SystemExit(main())
