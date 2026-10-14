"""Exercise the real env step and shaping wrapper at an interrupted contact step.

Only integration/observation acquisition is stubbed; no OpenSim rollout runs.
"""
import copy
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock

import gymnasium as gym
import numpy as np
import pytest

from osim_trj_cmc_like import CMCEnvConfig, CMCLikeProsthesisTrajectoryEnv
from simulation_runner import OnlineGRFPenetrationLimitExceeded
from unified_MLP_39.reward_function import RewardConfig, RewardShapingWrapper


def _failure(*, legacy=False):
    if legacy:
        return OnlineGRFPenetrationLimitExceeded("legacy text says 0.039 m; do not parse")
    contact = {
        "left": {
            "force": [0.0, 500.0, 0.0], "moment": [0.0, 0.0, 34.0],
            "cop": [0.1, 0.0, 0.35], "normal_force": 500.0,
            "penetration": 0.031, "slip_speed": 0.0, "in_contact": True,
        },
        "right": {
            "force": [0.0, 0.0, 0.0], "moment": [0.0, 0.0, 0.0],
            "cop": [0.0, 0.0, 0.0], "normal_force": 0.0,
            "penetration": 0.0, "slip_speed": 0.0, "in_contact": False,
        },
        "sensors": {},
    }
    return OnlineGRFPenetrationLimitExceeded(
        "measured contact", time_s=17.648, side="left", penetration_m=0.031,
        limit_m=0.03, online_grf=contact,
    )


def _env(*, legacy=False, state_time=17.648):
    env = CMCLikeProsthesisTrajectoryEnv.__new__(CMCLikeProsthesisTrajectoryEnv)
    env.env_cfg = CMCEnvConfig(
        policy_knots=1, segment_duration=0.01, record_outputs=False,
        fail_fast=True, binary_phase_fsm_mode="binary_active",
        truncation_penalty=1.0, reward_safety_weight=2.0,
        grf_penetration_penalty_threshold_m=0.020,
        grf_penetration_termination_m=0.028,
        reward_grf_penetration_weight=1.0,
        grf_ankle_moment_flip_tau_tol_nm=8.0,
        grf_ankle_moment_flip_force_threshold_n=50.0,
    )
    env.cfg = SimpleNamespace(
        t_start=12.0, t_end=30.0, grf_mode="online_sensor",
        online_grf_applied_sides=["left"], prescribed_grf_disabled_sides=[],
        pros_coords=["pros_knee_angle", "pros_ankle_angle"],
    )
    env.ctx = None
    env.t = 17.64
    env._episode_start = 12.99
    env._episode_end = 17.99
    env._step_index = 465
    env._observation_feature_names = ("test_observation",)
    env._last_sea_segment_diagnostics = {"previous_complete_segment": True}
    env._online_grf = {"left": {"penetration": 0.027}}
    env._phase_fsm_payload = {"state_name": "STANCE_AFTER_HS", "state_id": 1.0}
    env.action_space = gym.spaces.Box(-1.0, 1.0, shape=(1, 2), dtype=np.float64)
    env.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(1,), dtype=np.float64)
    exc = _failure(legacy=legacy)
    env.runner = SimpleNamespace(
        step_until=Mock(side_effect=exc),
        current_time=state_time,
        state=SimpleNamespace(getTime=lambda: state_time),
    )
    env.kin = SimpleNamespace(set_segment=Mock(), reference_governor_diagnostics={})
    segment = (np.array([17.64, 17.65]), np.zeros((2, 2)), np.zeros((2, 2)))
    env._action_to_segment = Mock(return_value=segment)
    env._command_rate_terms = Mock(return_value={})
    transport_observations = []

    def clear_failed_segment(step_info):
        assert step_info == {}
        env._online_grf = {}

    def phase_transport(**kwargs):
        transport_observations.append((copy.deepcopy(env._online_grf), kwargs))

    env._update_online_gait_state = Mock(side_effect=clear_failed_segment)
    env._update_binary_active_phase_path = Mock(side_effect=phase_transport)
    env._get_observation = Mock(return_value=(np.array([0.0]), {}))
    env._get_reward = Mock(return_value=(1.0, {
        "sound_imitation_loss": 0.0, "served_imitation_loss": 0.0,
    }))
    env._phase_fsm_reward_terms = Mock(return_value={})
    env._imitation_target_payload = Mock(return_value={})
    env._online_info_payload = lambda: {"online_grf": copy.deepcopy(env._online_grf)}
    env._binary_phase_info_payload = Mock(return_value={})
    # Real wrench projection below uses a nonzero cross-product contribution:
    # z . (M - center x F) = 34 - 50 = -16 Nm, loss ((16-8)/8)^2 = 1.
    env._prosthetic_ankle_center_axis_ground = Mock(return_value=(
        np.array([0.1, 0.2, 0.3]), np.array([0.0, 0.0, 1.0]),
    ))
    return env, exc, transport_observations


def test_hard_step_preserves_measured_contact_and_policy_boundary():
    env, exc, transport = _env()
    obs, reward, terminated, truncated, info = env.step(np.zeros((1, 2)))
    assert terminated and not truncated
    assert info["end_reason"] == "grf_penetration_hard"
    # Ordinary guard is also true after recovery, but must not rename hard failure.
    assert env._unsafe_end_reason({}) == "grf_penetration"
    assert env._step_index == 465
    assert env.t == 17.64
    assert info["time"] == 17.64
    assert not info["segment_completed"]
    assert info["terminal_contact"]["time_s"] == 17.648
    assert info["terminal_contact"]["penetration_m"] == 0.031
    assert info["terminal_contact"]["limit_m"] == 0.03
    terms = info["reward_terms"]
    assert terms["segment_completed"] == 0.0
    assert terms["grf_hard_contact_available"] == 1.0
    assert terms["grf_penetration_m"] == 0.031
    assert terms["grf_penetration_loss"] == pytest.approx(1.890625)
    assert terms["safety_loss"] == 1.0
    assert reward == pytest.approx(1.0 - 2.0 - 1.890625)
    assert terms["grf_ankle_moment_flip_available"] == 1.0
    assert terms["grf_ankle_moment_flip_tau_nm"] == pytest.approx(-16.0)
    assert terms["grf_ankle_moment_flip_loss"] == pytest.approx(1.0)
    assert transport == [({}, {"require_batch": False, "require_primary_sample": False})]
    assert info["phase_fsm"] == {"state_name": "STANCE_AFTER_HS", "state_id": 1.0}
    exc.online_grf["left"]["force"][1] = -1.0
    assert env._online_grf["left"]["force"][1] == 500.0
    assert info["terminal_contact"]["online_grf"]["left"]["force"][1] == 500.0


def test_legacy_exception_does_not_invent_contact_from_text_or_threshold():
    env, exc, transport = _env(legacy=True)
    _, reward, terminated, truncated, info = env.step(np.zeros((1, 2)))
    assert terminated and not truncated
    assert info["end_reason"] == "grf_penetration_hard"
    assert info["terminal_contact"] == {
        "source": "hard_penetration_guard", "available": False,
        "unavailable_reason": "legacy_exception_without_contact_sample",
    }
    terms = info["reward_terms"]
    assert terms["grf_hard_contact_available"] == 0.0
    assert terms["grf_penetration_m"] == 0.0
    assert terms["grf_penetration_loss"] == 0.0
    assert terms["safety_loss"] == 1.0
    assert reward == -1.0
    assert env.t == 17.64 and env._step_index == 465
    assert transport == [({}, {"require_batch": False, "require_primary_sample": False})]


def test_geometry_timestamp_mismatch_keeps_penetration_but_disables_moment():
    env, _, _ = _env(state_time=17.64)
    _, _, _, _, info = env.step(np.zeros((1, 2)))
    terms = info["reward_terms"]
    assert terms["grf_penetration_m"] == 0.031
    assert terms["grf_penetration_loss"] == pytest.approx(1.890625)
    assert terms["grf_ankle_moment_flip_available"] == 0.0
    assert terms["grf_ankle_moment_flip_loss"] == 0.0
    env._prosthetic_ankle_center_axis_ground.assert_not_called()


def test_real_reward_wrapper_penetration_weights_share_one_corrected_transition():
    results = []
    cfg = RewardConfig(
        reward_mode="imitation", blend_served_imitation=0.8, blend_imitation=0.2,
        blend_imitation_tracking=0.0, grf_penetration_weight=1.0,
        grf_ankle_moment_flip_weight=0.1, safety_weight=2.0, oob_weight=0.0,
    )
    for weight in (1.0, 2.0):
        env, _, _ = _env()
        wrapped = RewardShapingWrapper(env, replace(cfg, grf_penetration_weight=weight))
        _, reward, terminated, truncated, info = wrapped.step(np.zeros((1, 2)))
        assert terminated and not truncated
        assert info["end_reason"] == "grf_penetration_hard"
        assert info["reward_components"]["safety_term"] == 2.0
        assert info["reward_components"]["grf_ankle_moment_flip_term"] == pytest.approx(0.1)
        assert info["reward_components"]["grf_penetration_term"] == pytest.approx(weight * 1.890625)
        assert reward == pytest.approx(1.0 - 2.0 - 0.1 - weight * 1.890625)
        results.append((reward, info))
    assert results[0][0] - results[1][0] == pytest.approx(1.890625)
    assert results[0][1]["terminal_contact"] == results[1][1]["terminal_contact"]
    assert results[0][1]["reward_terms"] == results[1][1]["reward_terms"]


@pytest.mark.parametrize("field,value", [
    ("penetration_m", 0.032), ("limit_m", 0.032), ("time_s", float("nan")),
    ("side", "missing"),
])
def test_inconsistent_structured_metadata_is_not_silently_fabricated(field, value):
    exc = _failure()
    setattr(exc, field, value)
    with pytest.raises(ValueError, match="Hard penetration guard"):
        CMCLikeProsthesisTrajectoryEnv._hard_penetration_contact(exc)
