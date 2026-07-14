"""Focused behavioral checks for baseline_MLP/reward_function.py."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np


BASELINE_DIR = (
    Path(__file__).resolve().parents[1] / "Trajectory Generator" / "baseline_MLP"
)
TRAJECTORY_DIR = Path(__file__).resolve().parents[1] / "Trajectory Generator"
sys.path.insert(0, str(TRAJECTORY_DIR))
sys.path.insert(0, str(BASELINE_DIR))

from prosthetic_phase_fsm import ProstheticPhaseFSM, ProstheticPhaseFSMConfig  # noqa: E402
import reward_function  # noqa: E402
import training_config  # noqa: E402


TESTS = []


def _test(func):
    TESTS.append(func)
    return func


MORPHOLOGY_PROFILE = (
    BASELINE_DIR / "morphology_profiles" / "ab06_prosthetic_mean_std_corridor.json"
)


@_test
def test_v4_three_objective_blend() -> None:
    cfg = reward_function.RewardConfig.from_mapping(
        {
            "reward_mode": "imitation",
            "blend_served_imitation": 0.65,
            "blend_imitation": 0.20,
            "blend_imitation_tracking": 0.15,
            "oob_weight": 0.0,
        }
    )
    terms = {
        "served_imitation_loss": 0.25,
        "sound_imitation_loss": 0.50,
        "tracking_loss": 0.125,
    }

    reward, components = reward_function.compute_reward(terms, cfg)

    expected = (
        0.65 / (1.0 + 8.0 * 0.25)
        + 0.20 / (1.0 + 8.0 * 0.50)
        + 0.15 / (1.0 + 8.0 * 0.125)
    )
    assert math.isclose(reward, expected, rel_tol=0.0, abs_tol=1e-12)
    assert math.isclose(components["reward_base"], expected, abs_tol=1e-12)


@_test
def test_penetration_is_applied_after_clip() -> None:
    cfg = reward_function.RewardConfig(
        reward_mode="imitation",
        blend_served_imitation=1.0,
        blend_imitation=0.0,
        blend_imitation_tracking=0.0,
        grf_penetration_weight=5.0,
        oob_weight=0.0,
    )

    reward, components = reward_function.compute_reward(
        {"served_imitation_loss": 0.0, "grf_penetration_loss": 0.004},
        cfg,
    )

    assert math.isclose(reward, 0.98, rel_tol=0.0, abs_tol=1e-12)
    assert math.isclose(
        components["grf_penetration_term"], 0.02, rel_tol=0.0, abs_tol=1e-12
    )


@_test
def test_component_level_command_penalties_are_explicit() -> None:
    cfg = reward_function.RewardConfig(
        reward_mode="imitation",
        blend_served_imitation=1.0,
        blend_imitation=0.0,
        blend_imitation_tracking=0.0,
        jerk_ref_weight=0.25,
        segment_delta_weight=0.5,
        oob_weight=0.0,
    )

    reward, components = reward_function.compute_reward(
        {
            "served_imitation_loss": 0.0,
            "jerk_ref_loss": 0.4,
            "segment_delta_loss": 0.2,
        },
        cfg,
    )

    assert math.isclose(reward, 0.8, rel_tol=0.0, abs_tol=1e-12)
    assert math.isclose(components["penalty"], 0.2, rel_tol=0.0, abs_tol=1e-12)
    assert components["jerk_ref_loss"] == 0.4
    assert components["segment_delta_loss"] == 0.2


@_test
def test_training_cfg_enables_current_imitation_penetration_penalty() -> None:
    cfg = training_config.load(BASELINE_DIR / "training_cfg.yaml")
    _, reward = training_config.to_argparse_defaults(cfg)
    resolved = reward_function.RewardConfig.from_mapping(reward)

    assert resolved.reward_mode == "imitation"
    assert resolved.blend_served_imitation == 0.80
    assert resolved.grf_penetration_weight == 0.5


@_test
def test_exnovo_config_enables_task_based_reward_terms() -> None:
    cfg = training_config.load(BASELINE_DIR / "training_exnovo_cfg.yaml")
    _, reward = training_config.to_argparse_defaults(cfg)
    resolved = reward_function.RewardConfig.from_mapping(reward)

    assert resolved.reward_mode == "ex_novo"
    assert resolved.blend_tracking == 0.0
    assert resolved.blend_reference == 0.0
    assert resolved.blend_bio == 0.0
    assert resolved.blend_contact_load == 0.10
    assert resolved.blend_contact_support_to == 0.30
    assert resolved.contact_load_confidence_full_bw == 0.20
    assert resolved.contact_load_dense_evidence_limit_bw_s == 0.04
    assert resolved.contact_load_penetration_full_reward_m == 0.010
    assert resolved.contact_load_penetration_zero_reward_m == 0.012
    assert resolved.contact_support_to_window_start_s == 0.79
    assert resolved.contact_support_to_window_end_s == 1.26
    assert resolved.contact_support_failure_clawback_weight == 1.0
    assert resolved.grf_penetration_weight == 0.05
    assert resolved.blend_phase_regular > 0.0
    assert resolved.phase_period_nominal_s == 1.58
    assert resolved.phase_timeout_penalty_weight == 0.50
    assert resolved.phase_stance_hard_timeout_s == 2.20
    assert resolved.phase_swing_hard_timeout_s == 1.10
    assert resolved.phase_min_stance_duration_s == 0.30
    assert resolved.phase_min_swing_duration_s == 0.25
    assert resolved.phase_landing_window_start_s == 0.35
    assert resolved.phase_landing_window_end_s == 0.85
    assert resolved.phase_hs_event_credit == 0.10
    assert resolved.phase_to_event_credit == 0.20
    assert resolved.phase_cycle_complete_bonus == 0.70
    assert resolved.phase_failure_extra_penalty == 0.05
    assert resolved.blend_phase_event_progress == 1.00
    assert resolved.phase_clawback_penalty_weight == 1.00
    assert resolved.blend_landing_window_contact == 0.25
    assert resolved.phase_invalid_event_weight == 0.10
    assert resolved.phase_contact_validity_weight == 0.10
    assert resolved.phase_min_stance_contact_fraction == 0.20
    assert resolved.phase_min_stance_load_bw_s == 0.04
    assert resolved.phase_min_cycle_knee_excursion_rad == 0.12
    assert resolved.grf_slip_weight == 0.0
    assert resolved.policy_action_clip_weight == 0.25
    assert resolved.oob_q_min == (-1.25, -0.60)
    assert resolved.prosthetic_joint_q_min == (-1.30, -0.60)
    assert resolved.prosthetic_joint_range_weight == 2.0
    assert resolved.reserve_residual_weight == 0.0
    assert resolved.pelvis_height_weight == 0.0
    assert resolved.morphology_profile.endswith(
        "morphology_profiles/ab06_prosthetic_mean_std_corridor.json"
    )
    assert resolved.morphology_weight == 0.0
    assert resolved.morphology_std_multiplier_knee == 1.6
    assert resolved.morphology_std_multiplier_ankle == 0.6
    assert resolved.morphology_margin_knee_deg == 7.5
    assert resolved.morphology_margin_ankle_deg == 7.5


@_test
def test_morphology_profile_loads_ab06_corridor() -> None:
    profile = reward_function._load_morphology_profile(MORPHOLOGY_PROFILE)
    assert profile is not None
    assert profile["metadata"]["n_cycles"] == 123
    assert math.isclose(
        profile["metadata"]["mean_to_phase"], 0.6223299989, abs_tol=1e-9
    )

    phase_grid = profile["phase_grid"]
    assert phase_grid.shape == (201,)
    assert math.isclose(float(phase_grid[0]), 0.0, abs_tol=1e-12)
    assert math.isclose(float(phase_grid[-1]), 1.0, abs_tol=1e-12)
    assert np.all(np.diff(phase_grid) > 0.0)

    for coord_name in ("pros_knee_angle", "pros_ankle_angle"):
        coord = profile["coordinates"][coord_name]
        assert coord["mean_rad"].shape == phase_grid.shape
        assert coord["std_rad"].shape == phase_grid.shape
        assert np.all(np.isfinite(coord["mean_rad"]))
        assert np.all(np.isfinite(coord["std_rad"]))
        assert np.all(coord["std_rad"] >= 0.0)

    cfg = reward_function.RewardConfig()
    for phase in (0.0, 0.5, 0.999):
        corridor = reward_function._morphology_corridor_at(profile, phase, cfg)
        for coord_name in ("pros_knee_angle", "pros_ankle_angle"):
            low = corridor[coord_name]["min_rad"]
            high = corridor[coord_name]["max_rad"]
            assert np.isfinite(low)
            assert np.isfinite(high)
            assert low <= high


@_test
def test_morphology_weight_zero_is_diagnostic_only() -> None:
    cfg = reward_function.RewardConfig(
        reward_mode="ex_novo",
        blend_tracking=1.0,
        blend_reference=0.0,
        blend_bio=0.0,
        blend_contact_load=0.0,
        blend_phase_regular=0.0,
        morphology_weight=0.0,
        oob_weight=0.0,
    )
    reward_with, components = reward_function.compute_reward(
        {"tracking_loss": 0.0, "morphology_loss": 10.0},
        cfg,
    )
    reward_without, _ = reward_function.compute_reward({"tracking_loss": 0.0}, cfg)

    assert math.isclose(reward_with, reward_without, rel_tol=0.0, abs_tol=1e-12)
    assert components["morphology_loss"] == 10.0
    assert components["morphology_term"] == 0.0


@_test
def test_phase_timeout_penalty_is_applied_after_clip() -> None:
    cfg = reward_function.RewardConfig(
        reward_mode="ex_novo",
        blend_tracking=1.0,
        blend_reference=0.0,
        blend_bio=0.0,
        blend_contact_load=0.0,
        blend_phase_regular=0.0,
        phase_timeout_penalty_weight=0.1,
        oob_weight=0.0,
    )

    reward, components = reward_function.compute_reward(
        {"tracking_loss": 0.0, "phase_timeout_loss": 2.0},
        cfg,
    )

    assert math.isclose(components["reward_base"], 1.0, rel_tol=0.0, abs_tol=1e-12)
    assert math.isclose(
        components["phase_timeout_penalty_term"], 0.2, rel_tol=0.0, abs_tol=1e-12
    )
    assert math.isclose(reward, 0.8, rel_tol=0.0, abs_tol=1e-12)


@_test
def test_phase_fsm_valid_hs_to_hs_cycle() -> None:
    fsm = ProstheticPhaseFSM()
    fsm.update(
        time_s=0.0,
        events=[{"side": "left", "event": "heel_strike", "time": 0.0}],
        normal_force_bw=0.7,
        in_contact=True,
    )
    fsm.update(
        time_s=0.6,
        events=[{"side": "left", "event": "toe_off", "time": 0.6}],
        normal_force_bw=0.0,
        in_contact=False,
    )
    payload = fsm.update(
        time_s=1.4,
        events=[{"side": "left", "event": "heel_strike", "time": 1.4}],
        normal_force_bw=0.7,
        in_contact=True,
    )

    assert payload["valid_cycle_count"] == 1.0
    assert math.isclose(payload["phase_event_progress_score"], 0.8, abs_tol=1e-12)
    assert math.isclose(payload["phase_cycle_complete_bonus"], 0.7, abs_tol=1e-12)
    assert math.isclose(payload["pending_cycle_credit"], 0.1, abs_tol=1e-12)
    assert payload["invalid_event_this_step"] == 0.0
    assert payload["state_name"] == "STANCE_AFTER_HS"


@_test
def test_phase_fsm_rejects_static_knee_fake_cycle() -> None:
    fsm = ProstheticPhaseFSM(
        ProstheticPhaseFSMConfig(min_cycle_knee_excursion_rad=0.12)
    )
    fsm.update(
        time_s=0.0,
        events=[{"side": "left", "event": "heel_strike", "time": 0.0}],
        normal_force_bw=0.7,
        in_contact=True,
        prosthetic_knee_angle_rad=-0.65,
        prosthetic_ankle_angle_rad=0.0,
    )
    fsm.update(
        time_s=0.4,
        events=[{"side": "left", "event": "toe_off", "time": 0.4}],
        normal_force_bw=0.0,
        in_contact=False,
        prosthetic_knee_angle_rad=-0.64,
        prosthetic_ankle_angle_rad=0.2,
    )
    payload = fsm.update(
        time_s=0.8,
        events=[{"side": "left", "event": "heel_strike", "time": 0.8}],
        normal_force_bw=0.7,
        in_contact=True,
        prosthetic_knee_angle_rad=-0.63,
        prosthetic_ankle_angle_rad=-0.2,
    )

    assert payload["valid_cycle_count"] == 0.0
    assert payload["cycle_rejected_this_step"] == 1.0
    assert payload["invalid_event_type"] == "cycle_knee_excursion_too_low"
    assert payload["phase_cycle_complete_bonus"] == 0.0
    assert payload["cycle_knee_excursion_rad"] < 0.12


@_test
def test_phase_fsm_rejects_to_without_stance_contact() -> None:
    fsm = ProstheticPhaseFSM(
        ProstheticPhaseFSMConfig(min_stance_contact_fraction=0.20)
    )
    fsm.update(
        time_s=0.0,
        events=[{"side": "left", "event": "heel_strike", "time": 0.0}],
        normal_force_bw=0.7,
        in_contact=True,
    )
    fsm.update(
        time_s=0.3,
        events=[],
        normal_force_bw=0.0,
        in_contact=False,
    )
    payload = fsm.update(
        time_s=0.6,
        events=[{"side": "left", "event": "toe_off", "time": 0.6}],
        normal_force_bw=0.0,
        in_contact=False,
    )

    assert payload["valid_to_count"] == 0.0
    assert payload["invalid_event_this_step"] == 1.0
    assert payload["invalid_event_type"] == "stance_contact_too_low"
    assert payload["state_name"] == "STANCE_AFTER_HS"


@_test
def test_phase_fsm_double_hs_is_invalid_not_timeout() -> None:
    fsm = ProstheticPhaseFSM()
    fsm.update(
        time_s=0.0,
        events=[{"side": "left", "event": "heel_strike", "time": 0.0}],
        normal_force_bw=0.7,
        in_contact=True,
    )
    payload = fsm.update(
        time_s=0.2,
        events=[{"side": "left", "event": "heel_strike", "time": 0.2}],
        normal_force_bw=0.7,
        in_contact=True,
    )

    assert payload["invalid_event_this_step"] == 1.0
    assert payload["invalid_event_type"] == "double_hs_before_to"
    assert payload["timeout_exceeded"] == 0.0
    assert payload["state_name"] == "STANCE_AFTER_HS"


@_test
def test_phase_fsm_early_to_is_invalid() -> None:
    fsm = ProstheticPhaseFSM()
    fsm.update(
        time_s=0.0,
        events=[{"side": "left", "event": "heel_strike", "time": 0.0}],
        normal_force_bw=0.7,
        in_contact=True,
    )
    payload = fsm.update(
        time_s=0.01,
        events=[{"side": "left", "event": "toe_off", "time": 0.01}],
        normal_force_bw=0.0,
        in_contact=False,
    )

    assert payload["invalid_event_this_step"] == 1.0
    assert payload["invalid_event_type"] == "to_too_early_after_hs"
    assert payload["state_name"] == "STANCE_AFTER_HS"


@_test
def test_phase_fsm_swing_timeout() -> None:
    fsm = ProstheticPhaseFSM(ProstheticPhaseFSMConfig(swing_hard_timeout_s=1.30))
    fsm.update(
        time_s=0.0,
        events=[{"side": "left", "event": "heel_strike", "time": 0.0}],
        normal_force_bw=0.7,
        in_contact=True,
    )
    fsm.update(
        time_s=0.1,
        events=[{"side": "left", "event": "toe_off", "time": 0.1}],
        normal_force_bw=0.0,
        in_contact=False,
    )
    payload = fsm.update(
        time_s=1.45,
        events=[],
        normal_force_bw=0.0,
        in_contact=False,
    )

    assert payload["timeout_exceeded"] == 1.0
    assert payload["timeout_side"] == 2.0
    assert payload["state_name"] == "TIMEOUT"
    assert math.isclose(payload["phase_clawback_penalty"], 0.3, abs_tol=1e-12)
    assert math.isclose(payload["phase_failure_extra_penalty"], 0.05, abs_tol=1e-12)
    assert math.isclose(payload["pending_cycle_credit"], 0.0, abs_tol=1e-12)


@_test
def test_phase_timeout_hard_guard_terminates_wrapper() -> None:
    class DummyEnv(reward_function.gym.Env):
        def __init__(self) -> None:
            self.action_space = reward_function.gym.spaces.Box(
                low=-1.0, high=1.0, shape=(1, 2), dtype=np.float32
            )
            self._step = 0

        def step(self, action):
            self._step += 1
            if self._step == 1:
                info = {
                    "time": 0.0,
                    "online_events": [
                        {"side": "left", "event": "heel_strike", "time": 0.0}
                    ],
                }
            elif self._step == 2:
                info = {
                    "time": 0.1,
                    "online_events": [
                        {"side": "left", "event": "toe_off", "time": 0.1}
                    ],
                }
            else:
                info = {"time": 1.45, "online_events": []}
            info["phase_fsm"] = {
                "stance_elapsed_s": 0.0,
                "swing_elapsed_s": 1.35 if self._step >= 3 else 0.0,
                "timeout_exceeded": 1.0 if self._step >= 3 else 0.0,
                "timeout_side": 2.0 if self._step >= 3 else 0.0,
                "phase_event_progress_score": 0.5,
            }
            info["reward_terms"] = {"tracking_loss": 0.0}
            return np.zeros(1, dtype=np.float32), 0.0, False, False, info

        def reset(self, *, seed=None, options=None):
            self._step = 0
            return np.zeros(1, dtype=np.float32), {}

    cfg = reward_function.RewardConfig(
        reward_mode="ex_novo",
        blend_tracking=1.0,
        phase_timeout_penalty_weight=0.1,
        phase_swing_timeout_s=0.90,
        phase_swing_hard_timeout_s=1.30,
        oob_weight=0.0,
    )
    env = reward_function.RewardShapingWrapper(DummyEnv(), cfg)
    action = np.zeros((1, 2), dtype=np.float32)

    env.step(action)
    env.step(action)
    _, reward, terminated, truncated, info = env.step(action)

    assert terminated is True
    assert truncated is False
    assert info["end_reason"] == "phase_timeout:swing"
    assert info["reward_terms"]["phase_timeout_exceeded"] == 1.0
    assert info["reward_terms"]["phase_timeout_side"] == 2.0
    assert reward < 1.0


@_test
def test_phase_event_progress_increases_exnovo_base() -> None:
    cfg = reward_function.RewardConfig(
        reward_mode="ex_novo",
        blend_tracking=0.0,
        blend_reference=0.0,
        blend_bio=0.0,
        blend_contact_load=0.0,
        blend_phase_regular=0.0,
        blend_phase_event_progress=0.2,
        oob_weight=0.0,
    )

    reward, components = reward_function.compute_reward(
        {"phase_event_progress_score": 0.5},
        cfg,
    )

    assert math.isclose(reward, 0.1, rel_tol=0.0, abs_tol=1e-12)
    assert math.isclose(
        components["phase_event_progress_score"], 0.5, abs_tol=1e-12
    )


@_test
def test_phase_fsm_event_credit_is_one_shot() -> None:
    fsm = ProstheticPhaseFSM()
    hs_payload = fsm.update(
        time_s=0.0,
        events=[{"side": "left", "event": "heel_strike", "time": 0.0}],
        normal_force_bw=0.7,
        in_contact=True,
    )
    idle_payload = fsm.update(
        time_s=0.1,
        events=[],
        normal_force_bw=0.7,
        in_contact=True,
    )

    assert math.isclose(hs_payload["phase_event_progress_score"], 0.1, abs_tol=1e-12)
    assert math.isclose(hs_payload["cycle_progress_credit"], 0.25, abs_tol=1e-12)
    assert math.isclose(idle_payload["phase_event_progress_score"], 0.0, abs_tol=1e-12)
    assert math.isclose(idle_payload["cycle_progress_credit"], 0.25, abs_tol=1e-12)


@_test
def test_phase_clawback_makes_failed_half_cycle_negative() -> None:
    cfg = reward_function.RewardConfig(
        reward_mode="ex_novo",
        blend_tracking=0.0,
        blend_reference=0.0,
        blend_bio=0.0,
        blend_contact_load=0.0,
        blend_phase_regular=0.0,
        blend_phase_event_progress=1.0,
        phase_clawback_penalty_weight=1.0,
        oob_weight=0.0,
    )

    reward, components = reward_function.compute_reward(
        {
            "phase_event_progress_score": 0.3,
            "phase_clawback_penalty": 0.3,
            "phase_failure_extra_penalty": 0.05,
        },
        cfg,
    )

    assert math.isclose(components["reward_base"], 0.3, abs_tol=1e-12)
    assert math.isclose(
        components["phase_clawback_penalty_term"], 0.35, abs_tol=1e-12
    )
    assert math.isclose(reward, -0.05, abs_tol=1e-12)


@_test
def test_phase_regular_score_requires_completed_cycle() -> None:
    class DummyEnv(reward_function.gym.Env):
        action_space = reward_function.gym.spaces.Box(
            low=-1.0, high=1.0, shape=(1,), dtype=np.float32
        )

    wrapper = reward_function.RewardShapingWrapper(
        DummyEnv(),
        reward_function.RewardConfig(),
    )

    partial = wrapper._phase_regular_terms(
        {
            "phase_fsm": {
                "phase_event_progress_score": 0.5,
                "cycle_progress_credit": 0.5,
                "valid_cycle_count": 0.0,
            }
        }
    )
    complete = wrapper._phase_regular_terms(
        {
            "phase_fsm": {
                "phase_event_progress_score": 0.8,
                "cycle_progress_credit": 1.0,
                "valid_cycle_count": 1.0,
                "last_period_s": 1.58,
                "last_stance_fraction": 0.6,
            }
        }
    )

    assert partial["phase_regular_score"] == 0.0
    assert complete["phase_regular_score"] > 0.0


@_test
def test_landing_window_contact_score_increases_exnovo_base() -> None:
    cfg = reward_function.RewardConfig(
        reward_mode="ex_novo",
        blend_tracking=0.0,
        blend_reference=0.0,
        blend_bio=0.0,
        blend_contact_load=0.0,
        blend_phase_regular=0.0,
        blend_landing_window_contact=0.15,
        oob_weight=0.0,
    )

    reward, components = reward_function.compute_reward(
        {"landing_window_contact_score": 0.8},
        cfg,
    )

    assert math.isclose(reward, 0.12, rel_tol=0.0, abs_tol=1e-12)
    assert components["landing_window_contact_score"] == 0.8


@_test
def test_contact_support_dense_credit_stops_and_is_clawed_back_on_failure() -> None:
    class DummyEnv(reward_function.gym.Env):
        action_space = reward_function.gym.spaces.Box(
            low=-1.0, high=1.0, shape=(1,), dtype=np.float32
        )

    cfg = reward_function.RewardConfig(
        reward_mode="ex_novo",
        blend_tracking=0.0,
        blend_reference=0.0,
        blend_bio=0.0,
        blend_contact_load=0.10,
        blend_contact_support_to=0.30,
        blend_phase_regular=0.0,
        contact_load_confidence_full_bw=0.20,
        contact_load_dense_evidence_limit_bw_s=0.04,
        contact_support_to_window_start_s=0.79,
        contact_support_to_window_end_s=1.26,
        contact_support_failure_clawback_weight=1.0,
        phase_min_stance_duration_s=0.30,
        phase_stance_timeout_s=1.45,
        oob_weight=0.0,
    )
    wrapper = reward_function.RewardShapingWrapper(DummyEnv(), cfg)

    def stance_info(load_integral: float, *, terminal: bool = False):
        return {
            "online_gait": {
                "sides": {
                    "left": {
                        "normal_force_bw": 0.20,
                        "in_contact": True,
                        "gait_phase": 0.0,
                        "cycle_duration_s": 0.0,
                        "last_heel_strike_time": 0.0,
                        "last_toe_off_time": None,
                    }
                }
            },
            "online_grf": {"left": {"penetration": 0.005, "slip_speed": 0.0}},
            "phase_fsm": {
                "state_id": 1.0,
                "valid_hs_count": 1.0,
                "valid_to_count": 0.0,
                "stance_load_integral_bw_s": load_integral,
            },
            "reward_terms": {"terminated": float(terminal)},
            "end_reason": "grf_penetration" if terminal else None,
        }

    first = wrapper._task_reward_terms(stance_info(0.01))
    complete = wrapper._task_reward_terms(stance_info(0.04))
    failed = wrapper._task_reward_terms(stance_info(0.05, terminal=True))

    assert first["contact_load_score"] == 1.0
    assert math.isclose(
        first["contact_support_pending_dense_reward"], 0.10, abs_tol=1e-12
    )
    assert complete["contact_load_score"] == 0.0
    assert complete["contact_load_evidence_complete"] == 1.0
    assert math.isclose(
        failed["contact_support_clawback_penalty"], 0.10, abs_tol=1e-12
    )
    reward, components = reward_function.compute_reward(failed, cfg)
    assert math.isclose(reward, -0.10, abs_tol=1e-12)
    assert math.isclose(components["contact_support_clawback_term"], 0.10)


@_test
def test_contact_support_credit_is_confirmed_only_by_well_timed_valid_to() -> None:
    class DummyEnv(reward_function.gym.Env):
        action_space = reward_function.gym.spaces.Box(
            low=-1.0, high=1.0, shape=(1,), dtype=np.float32
        )

    cfg = reward_function.RewardConfig(
        reward_mode="ex_novo",
        blend_tracking=0.0,
        blend_reference=0.0,
        blend_bio=0.0,
        blend_contact_load=0.10,
        blend_contact_support_to=0.30,
        blend_phase_regular=0.0,
        contact_load_confidence_full_bw=0.20,
        contact_load_dense_evidence_limit_bw_s=0.04,
        contact_support_to_window_start_s=0.79,
        contact_support_to_window_end_s=1.26,
        contact_support_failure_clawback_weight=1.0,
        phase_min_stance_duration_s=0.30,
        phase_stance_timeout_s=1.45,
        oob_weight=0.0,
    )
    wrapper = reward_function.RewardShapingWrapper(DummyEnv(), cfg)
    wrapper._task_reward_terms(
        {
            "online_gait": {
                "sides": {
                    "left": {
                        "normal_force_bw": 0.20,
                        "in_contact": True,
                        "gait_phase": 0.0,
                        "cycle_duration_s": 0.0,
                        "last_heel_strike_time": 0.0,
                        "last_toe_off_time": None,
                    }
                }
            },
            "online_grf": {"left": {"penetration": 0.005, "slip_speed": 0.0}},
            "phase_fsm": {
                "state_id": 1.0,
                "valid_hs_count": 1.0,
                "valid_to_count": 0.0,
                "stance_load_integral_bw_s": 0.01,
            },
            "reward_terms": {},
        }
    )
    toe_off = wrapper._task_reward_terms(
        {
            "online_gait": {
                "sides": {
                    "left": {
                        "normal_force_bw": 0.0,
                        "in_contact": False,
                        "gait_phase": 0.0,
                        "cycle_duration_s": 0.0,
                        "last_heel_strike_time": 0.0,
                        "last_toe_off_time": 1.0,
                    }
                }
            },
            "online_grf": {"left": {"penetration": 0.0, "slip_speed": 0.0}},
            "phase_fsm": {
                "state_id": 2.0,
                "valid_hs_count": 1.0,
                "valid_to_count": 1.0,
                "stance_load_integral_bw_s": 0.05,
            },
            "reward_terms": {},
        }
    )

    assert toe_off["contact_support_to_timing_score"] == 1.0
    assert toe_off["contact_support_to_score"] == 1.0
    assert math.isclose(
        toe_off["contact_support_dense_confirmed_reward"], 0.10, abs_tol=1e-12
    )
    assert toe_off["contact_support_clawback_penalty"] == 0.0
    reward, _ = reward_function.compute_reward(toe_off, cfg)
    assert math.isclose(reward, 0.30, abs_tol=1e-12)


@_test
def test_contact_support_to_uses_mean_stance_penetration_quality() -> None:
    class DummyEnv(reward_function.gym.Env):
        action_space = reward_function.gym.spaces.Box(
            low=-1.0, high=1.0, shape=(1,), dtype=np.float32
        )

    cfg = reward_function.RewardConfig(
        blend_contact_support_to=0.30,
        contact_support_to_window_start_s=0.79,
        contact_support_to_window_end_s=1.26,
        phase_min_stance_duration_s=0.30,
        phase_stance_timeout_s=1.45,
    )
    wrapper = reward_function.RewardShapingWrapper(DummyEnv(), cfg)
    stance = {
        "phase_fsm": {
            "state_id": 1.0,
            "valid_hs_count": 1.0,
            "valid_to_count": 0.0,
            "stance_load_integral_bw_s": 0.0,
        },
        "reward_terms": {},
    }
    wrapper._contact_support_terms(
        stance,
        stance_expected=True,
        candidate_contact_score=0.0,
        penetration_quality=1.0,
    )
    wrapper._contact_support_terms(
        stance,
        stance_expected=True,
        candidate_contact_score=0.0,
        penetration_quality=0.0,
    )
    toe_off = wrapper._contact_support_terms(
        {
            "online_gait": {
                "sides": {
                    "left": {
                        "last_heel_strike_time": 0.0,
                        "last_toe_off_time": 1.0,
                    }
                }
            },
            "phase_fsm": {
                "state_id": 2.0,
                "valid_hs_count": 1.0,
                "valid_to_count": 1.0,
                "stance_load_integral_bw_s": 0.0,
            },
            "reward_terms": {},
        },
        stance_expected=False,
        candidate_contact_score=0.0,
        penetration_quality=1.0,
    )

    assert toe_off["contact_support_min_penetration_quality"] == 0.0
    assert math.isclose(
        toe_off["contact_support_mean_penetration_quality"], 0.5, abs_tol=1e-12
    )
    assert math.isclose(toe_off["contact_support_to_score"], 0.5, abs_tol=1e-12)


@_test
def test_task_phase_expectation_prefers_accepted_fsm_state() -> None:
    class DummyEnv(reward_function.gym.Env):
        action_space = reward_function.gym.spaces.Box(
            low=-1.0, high=1.0, shape=(1,), dtype=np.float32
        )

    def task_terms(*, fsm_state: int, raw_swing: bool):
        wrapper = reward_function.RewardShapingWrapper(
            DummyEnv(), reward_function.RewardConfig()
        )
        return wrapper._task_reward_terms(
            {
                "online_gait": {
                    "sides": {
                        "left": {
                            "normal_force_bw": 0.40,
                            "in_contact": True,
                            "gait_phase": 0.90 if raw_swing else 0.10,
                            "cycle_duration_s": 1.50,
                            "last_heel_strike_time": 0.0,
                            "last_toe_off_time": 0.10 if raw_swing else None,
                        }
                    }
                },
                "online_grf": {
                    "left": {"penetration": 0.005, "slip_speed": 0.0}
                },
                "phase_fsm": {
                    "state_id": float(fsm_state),
                    "state_name": {
                        0: "WAIT_HS",
                        1: "STANCE_AFTER_HS",
                        2: "SWING_AFTER_TO",
                    }[fsm_state],
                    "valid_hs_count": 1.0,
                    "valid_to_count": float(fsm_state == 2),
                    "stance_load_integral_bw_s": 0.01,
                },
                "reward_terms": {},
            }
        )

    accepted_stance = task_terms(fsm_state=1, raw_swing=True)
    accepted_swing = task_terms(fsm_state=2, raw_swing=False)
    wait_hs_fallback = task_terms(fsm_state=0, raw_swing=True)

    assert accepted_stance["prosthetic_stance_expected"] == 1.0
    assert accepted_stance["prosthetic_swing_expected"] == 0.0
    assert accepted_stance["swing_unloading_loss"] == 0.0
    assert accepted_stance["prosthetic_phase_expectation_source_id"] == 1.0
    assert accepted_swing["prosthetic_stance_expected"] == 0.0
    assert accepted_swing["prosthetic_swing_expected"] == 1.0
    assert accepted_swing["swing_unloading_loss"] > 0.0
    assert accepted_swing["prosthetic_phase_expectation_source_id"] == 1.0
    assert wait_hs_fallback["prosthetic_stance_expected"] == 0.0
    assert wait_hs_fallback["prosthetic_swing_expected"] == 1.0
    assert wait_hs_fallback["prosthetic_phase_expectation_source_id"] == 0.0


@_test
def test_contact_support_early_to_gets_no_confirmation_bonus() -> None:
    assert reward_function._soft_window_score(0.30, 0.79, 1.26, 0.30, 1.45) == 0.0
    assert math.isclose(
        reward_function._soft_window_score(0.545, 0.79, 1.26, 0.30, 1.45),
        0.5,
        abs_tol=1e-12,
    )
    assert reward_function._soft_window_score(1.0, 0.79, 1.26, 0.30, 1.45) == 1.0


@_test
def test_invalid_event_penalizes_without_termination_term() -> None:
    cfg = reward_function.RewardConfig(
        reward_mode="ex_novo",
        blend_tracking=1.0,
        blend_reference=0.0,
        blend_bio=0.0,
        blend_contact_load=0.0,
        blend_phase_regular=0.0,
        phase_invalid_event_weight=0.1,
        oob_weight=0.0,
    )

    reward, components = reward_function.compute_reward(
        {"tracking_loss": 0.0, "invalid_event_loss": 1.0},
        cfg,
    )

    assert math.isclose(reward, 0.9, rel_tol=0.0, abs_tol=1e-12)
    assert components["invalid_event_loss"] == 1.0


@_test
def test_morphology_penalty_is_applied_after_clip() -> None:
    cfg = reward_function.RewardConfig(
        reward_mode="ex_novo",
        blend_tracking=1.0,
        blend_reference=0.0,
        blend_bio=0.0,
        blend_contact_load=0.0,
        blend_phase_regular=0.0,
        morphology_weight=0.1,
        oob_weight=0.0,
    )

    reward, components = reward_function.compute_reward(
        {"tracking_loss": 0.0, "morphology_loss": 2.0},
        cfg,
    )

    assert math.isclose(components["reward_base"], 1.0, rel_tol=0.0, abs_tol=1e-12)
    assert math.isclose(components["morphology_term"], 0.2, rel_tol=0.0, abs_tol=1e-12)
    assert math.isclose(reward, 0.8, rel_tol=0.0, abs_tol=1e-12)


@_test
def test_morphology_margin_widens_corridor_in_radians() -> None:
    profile = reward_function._load_morphology_profile(MORPHOLOGY_PROFILE)
    assert profile is not None
    zero_margin = reward_function.RewardConfig()
    knee_margin = reward_function.RewardConfig(morphology_margin_knee_deg=5.0)

    base = reward_function._morphology_corridor_at(profile, 0.5, zero_margin)
    widened = reward_function._morphology_corridor_at(profile, 0.5, knee_margin)
    margin_rad = math.radians(5.0)

    assert math.isclose(
        widened["pros_knee_angle"]["min_rad"],
        base["pros_knee_angle"]["min_rad"] - margin_rad,
        rel_tol=0.0,
        abs_tol=1e-12,
    )
    assert math.isclose(
        widened["pros_knee_angle"]["max_rad"],
        base["pros_knee_angle"]["max_rad"] + margin_rad,
        rel_tol=0.0,
        abs_tol=1e-12,
    )
    assert math.isclose(
        widened["pros_ankle_angle"]["min_rad"],
        base["pros_ankle_angle"]["min_rad"],
        rel_tol=0.0,
        abs_tol=1e-12,
    )


@_test
def test_fsm_morphology_phase_uses_nominal_bootstrap_timing() -> None:
    cfg = reward_function.RewardConfig(
        phase_period_nominal_s=1.58,
        prosthetic_stance_phase_end=0.68,
    )
    stance_duration = 1.58 * 0.68
    swing_duration = 1.58 * (1.0 - 0.68)

    stance_phase, stance_available, stance_source = reward_function._fsm_morphology_phase(
        {
            "phase_fsm": {
                "state_id": 1.0,
                "stance_elapsed_s": 0.5 * stance_duration,
                "valid_cycle_count": 0.0,
            }
        },
        cfg,
    )
    swing_phase, swing_available, swing_source = reward_function._fsm_morphology_phase(
        {
            "phase_fsm": {
                "state_id": 2.0,
                "swing_elapsed_s": 0.5 * swing_duration,
                "valid_cycle_count": 0.0,
            }
        },
        cfg,
    )

    assert stance_available == 1.0
    assert stance_source == 1.0
    assert math.isclose(stance_phase, 0.34, rel_tol=0.0, abs_tol=1e-12)
    assert swing_available == 1.0
    assert swing_source == 1.0
    assert math.isclose(swing_phase, 0.84, rel_tol=0.0, abs_tol=1e-12)


@_test
def test_fsm_morphology_phase_uses_measured_cycle_after_completion() -> None:
    cfg = reward_function.RewardConfig(
        phase_period_nominal_s=1.58,
        prosthetic_stance_phase_end=0.68,
    )
    phase, available, source = reward_function._fsm_morphology_phase(
        {
            "phase_fsm": {
                "state_id": 1.0,
                "stance_elapsed_s": 0.3,
                "valid_cycle_count": 1.0,
                "last_period_s": 1.2,
                "last_stance_fraction": 0.5,
            }
        },
        cfg,
    )

    assert available == 1.0
    assert source == 2.0
    assert math.isclose(phase, 0.25, rel_tol=0.0, abs_tol=1e-12)


@_test
def test_morphology_prefers_fsm_phase_over_online_gait_phase() -> None:
    class DummyEnv(reward_function.gym.Env):
        action_space = reward_function.gym.spaces.Box(
            low=-1.0, high=1.0, shape=(1,), dtype=np.float32
        )

    cfg = reward_function.RewardConfig(
        morphology_profile=str(MORPHOLOGY_PROFILE),
        phase_period_nominal_s=1.58,
        prosthetic_stance_phase_end=0.68,
    )
    wrapper = reward_function.RewardShapingWrapper(DummyEnv(), cfg)
    swing_duration = 1.58 * (1.0 - 0.68)

    terms = wrapper._morphology_terms(
        {
            "phase_fsm": {
                "state_id": 2.0,
                "swing_elapsed_s": 0.5 * swing_duration,
                "valid_cycle_count": 0.0,
            },
            "online_gait": {"sides": {"left": {"gait_phase": 0.0}}},
            "observation": {
                "pros_knee_angle_served_ref": -0.7,
                "pros_ankle_angle_served_ref": 0.1,
            },
        }
    )

    assert terms["morphology_available"] == 1.0
    assert math.isclose(terms["morphology_phase"], 0.84, rel_tol=0.0, abs_tol=1e-12)
    assert math.isclose(
        terms["fsm_morphology_phase"],
        0.84,
        rel_tol=0.0,
        abs_tol=1e-12,
    )
    assert terms["morphology_phase_source_id"] == 1.0
    assert terms["morphology_phase_fsm_available"] == 1.0


@_test
def test_morphology_is_unavailable_before_first_fsm_hs() -> None:
    class DummyEnv(reward_function.gym.Env):
        action_space = reward_function.gym.spaces.Box(
            low=-1.0, high=1.0, shape=(1,), dtype=np.float32
        )

    cfg = reward_function.RewardConfig(morphology_profile=str(MORPHOLOGY_PROFILE))
    wrapper = reward_function.RewardShapingWrapper(DummyEnv(), cfg)

    terms = wrapper._morphology_terms(
        {
            "phase_fsm": {"state_id": 0.0},
            "online_gait": {"sides": {"left": {"gait_phase": 0.0}}},
            "observation": {
                "pros_knee_angle_served_ref": -0.2,
                "pros_ankle_angle_served_ref": 0.1,
            },
        }
    )

    assert terms["morphology_available"] == 0.0
    assert terms["morphology_phase"] == 0.0


if __name__ == "__main__":
    for test in TESTS:
        test()
        print(f"[PASS] {test.__name__}")
    print(f"ALL REWARD TESTS PASSED ({len(TESTS)})")
