"""The trainable RIGHT reference preserves V26 targets and its unshifted clock."""
from types import SimpleNamespace

import numpy as np
import pytest

from osim_trj_cmc_like import CMCEnvConfig, GaitPhaseClock, PhaseBasedImitationTarget
from unified_MLP_39.env import UnifiedProsthesisEnv, CMCLikeProsthesisTrajectoryEnv, CLOCK_FEATURES
from unified_MLP_39.phase_target import (
    COORDINATES, PrescribedRightV26PhaseTarget, RIGHT_PHASE_SHIFTS, RIGHT_TARGET_SOURCE,
)
from unified_MLP_39.target_availability import EpisodeAvailability, target_feature_names


class RightKinematics:
    time_bounds = (0.0, 2.2)
    left_value = 100.0

    def get(self, t):
        cycle = int(t >= 1.0)
        period = 1.2 if cycle else 1.0
        phase = (t - cycle) / period
        angle = 2.0 * np.pi * phase
        return ({"knee_angle_r": -0.6 + 0.25 * np.cos(angle) + 0.08 * cycle,
                 "ankle_angle_r": 0.2 * np.sin(angle) + 0.03 * cycle,
                 COORDINATES[0]: self.left_value, COORDINATES[1]: self.left_value}, {}, {})


def targets(kin=None):
    kin = kin or RightKinematics()
    cfg = CMCEnvConfig()
    native = PrescribedRightV26PhaseTarget(
        kin, [0.0, 1.0, 2.2], sound_coordinates=cfg.imitation_sound_coords,
        time_window=(0.0, 2.2),
    )
    historical = PhaseBasedImitationTarget(
        kin, GaitPhaseClock([0.0, 1.0, 2.2], phase_offset=0.0),
        cfg.imitation_sound_coords, {COORDINATES[0]: 0.465, COORDINATES[1]: 0.452},
        fallback_phase_shift=0.5, phase_samples=200, time_window=(0.0, 2.2),
    )
    return native, historical


def bare_env():
    env = UnifiedProsthesisEnv.__new__(UnifiedProsthesisEnv)
    env.target_reference = "right_v26"
    env.target_velocity_scale = 1.0
    env.base_kin = RightKinematics()
    env.env_cfg = CMCEnvConfig()
    env.cfg = SimpleNamespace(pros_coords=COORDINATES, t_start=0.0, t_end=2.2)
    env._target_availability = EpisodeAvailability(1.0)
    sides = []
    env._load_sound_heel_strikes = lambda side: sides.append(side) or [0.0, 1.0, 2.2]
    env._imitation_position_ranges_rad, env._imitation_velocity_ranges_rad_s = env._build_imitation_error_ranges()
    assert sides == ["right"]
    env.t = 1.3
    env._n_actor = 35
    env._actor_feature_names = CLOCK_FEATURES + tuple(f"actor_{i}" for i in range(33))
    env._privileged_feature_names = tuple(f"critic_{i}" for i in range(49))
    env._observation_feature_names = env._actor_feature_names + env._privileged_feature_names
    return env


def test_target_and_ranges_exactly_match_historical_v26_algorithm():
    native, historical = targets()
    env = bare_env()
    times = np.linspace(0.0, 2.2, 200)
    for t in times:
        assert native.get(t) == historical.get(t) == env.imitation_target(t)
    for joint in COORDINATES:
        assert env._imitation_position_ranges_rad[joint] == np.ptp([
            historical.get(t)[0][joint] for t in times])
        assert env._imitation_velocity_ranges_rad_s[joint] == np.ptp([
            historical.get(t)[1][joint] for t in times])
    summary = env._imitation_target_summary()
    assert summary["source"] == RIGHT_TARGET_SOURCE
    assert summary["phase_shifts"] == RIGHT_PHASE_SHIFTS
    assert summary["time_alignment"] == "prescribed_right_gait_phase"


def test_right_target_does_not_read_left_prescribed_coordinates():
    kin = RightKinematics()
    before = targets(kin)[0].get(1.3)
    kin.left_value = np.nan
    after = targets(kin)[0].get(1.3)
    assert before == after


def test_right_phase_actor_clock_is_unshifted_and_all_six_channels_mask_together(monkeypatch):
    env = bare_env()
    base = np.arange(84, dtype=np.float32)
    monkeypatch.setattr(CMCLikeProsthesisTrajectoryEnv, "_get_observation", lambda self: (base, {}))
    env._target_availability.visible = True
    visible, named = env._get_observation()
    payload = env._imitation_target_payload()
    q, qdot, phases = env.imitation_target(env.t)
    assert phases[COORDINATES[0]] == pytest.approx((0.25 - 0.465) % 1.0)
    assert phases[COORDINATES[1]] == pytest.approx((0.25 - 0.452) % 1.0)
    # Clock is right phase 0.25, not the shifted knee target phase 0.785.
    np.testing.assert_allclose(visible[:2], [1.0, 0.0], atol=1e-7)
    np.testing.assert_array_equal(visible[35:39], np.asarray([
        q[COORDINATES[0]], qdot[COORDINATES[0]],
        q[COORDINATES[1]], qdot[COORDINATES[1]],
    ], dtype=np.float32))
    assert env.actor_feature_names[35:39] == target_feature_names("right_v26")
    assert all(name in named for name in target_feature_names("right_v26"))
    assert env._gait_clock_summary()["phase"] == pytest.approx(0.25)
    env._target_availability.visible = False
    hidden, _ = env._get_observation()
    np.testing.assert_array_equal(hidden[[0, 1, 35, 36, 37, 38]], np.zeros(6))
    assert payload == env._imitation_target_payload()
    assert env._imitation_target_summary()["actor_clock_visible"] is False
    assert env._gait_clock_summary()["actor_visible"] is False
    for observation in (visible, hidden):
        assert observation.shape == (88,)
        np.testing.assert_array_equal(observation[2:35], base[2:35])
        np.testing.assert_array_equal(observation[39:], base[35:])


@pytest.mark.parametrize("time", [np.nan, np.inf, -1, 3, True])
def test_invalid_target_times_fail_closed(time):
    with pytest.raises(ValueError, match="prescribed-right"):
        targets()[0].get(time)


@pytest.mark.parametrize("strikes", [[], [0.0], [0.0, np.nan], [1.0, 0.0], [0.0, 0.0], [4.0, 5.0]])
def test_unusable_right_cycles_fail_without_temporal_fallback(strikes):
    with pytest.raises(ValueError, match="prescribed-right"):
        PrescribedRightV26PhaseTarget(
            RightKinematics(), strikes, sound_coordinates=CMCEnvConfig().imitation_sound_coords,
        )


def test_factory_propagates_right_reference_and_default_left(monkeypatch):
    from unified_MLP_39 import env_factory
    captured = []
    marker = object()

    def constructor(config, **kwargs):
        captured.append(kwargs)
        return marker

    monkeypatch.setattr(env_factory, "UnifiedProsthesisEnv", constructor)
    monkeypatch.setattr(env_factory, "FlattenClipAction", lambda env: env)
    monkeypatch.setattr(env_factory.reward_function, "RewardShapingWrapper", lambda env, config: env)
    assert env_factory.make_cmc_env({"target_reference": "right_v26"}) is marker
    assert captured[-1]["target_reference"] == "right_v26"
    assert env_factory.make_cmc_env() is marker
    assert captured[-1]["target_reference"] == "left"


def test_unknown_reference_rejected_before_simulator_loading():
    with pytest.raises(ValueError, match="target_reference"):
        UnifiedProsthesisEnv(CMCEnvConfig(), target_reference="right")
