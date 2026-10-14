"""The teacher follows left gait phase, with matching observation and loss targets."""
import copy
from types import SimpleNamespace

import numpy as np
import pytest

from unified_MLP_39 import actor_transition
from unified_MLP_39.env import UnifiedProsthesisEnv, CMCLikeProsthesisTrajectoryEnv, CLOCK_FEATURES
from unified_MLP_39.phase_target import PrescribedLeftPhaseTarget, COORDINATES, TARGET_SOURCE
from unified_MLP_39.target_availability import EpisodeAvailability, TARGET_FEATURES
from osim_trj_cmc_like import CMCEnvConfig


class Kinematics:
    time_bounds = (0.0, 2.2)
    right_value = 100.0

    def get(self, t):
        cycle = int(t >= 1.0)
        period = 1.2 if cycle else 1.0
        phase = (t - cycle) / period
        angle = 2 * np.pi * phase
        return (
            {COORDINATES[0]: -0.6 + 0.25 * np.cos(angle) + 0.08 * cycle,
             COORDINATES[1]: 0.2 * np.sin(angle) + 0.03 * cycle,
             "knee_angle_r": self.right_value, "ankle_angle_r": self.right_value},
            {COORDINATES[0]: -0.25 * 2 * np.pi * np.sin(angle) / period,
             COORDINATES[1]: 0.2 * 2 * np.pi * np.cos(angle) / period}, {},
        )


def teacher(kin=None, strikes=(0.0, 1.0, 2.2)):
    return PrescribedLeftPhaseTarget(kin or Kinematics(), strikes, time_window=(0.0, 2.2))


def test_same_phase_same_position_and_velocity_scales_with_period():
    kin = Kinematics()
    target = teacher(kin)
    fast, slow = target.get(0.25), target.get(1.3)
    for joint in COORDINATES:
        assert fast[2][joint] == pytest.approx(0.25)
        assert slow[2][joint] == pytest.approx(0.25)
        assert fast[0][joint] == pytest.approx(slow[0][joint], abs=1e-12)
        assert slow[1][joint] == pytest.approx(fast[1][joint] / 1.2, abs=1e-12)
    # Different raw cycles become one mean phase template, not a t lookup.
    assert fast[0][COORDINATES[0]] == pytest.approx(-0.56)
    assert fast[0][COORDINATES[1]] == pytest.approx(0.215)
    assert kin.get(0.25)[0][COORDINATES[0]] != pytest.approx(fast[0][COORDINATES[0]])


def test_zero_shift_left_heel_strike_and_no_right_leg_dependency():
    kin = Kinematics()
    baseline = teacher(kin).get(1.0)
    kin.right_value = np.nan
    changed = teacher(kin).get(1.0)
    assert baseline == changed
    assert baseline[2] == dict.fromkeys(COORDINATES, 0.0)
    assert baseline[0][COORDINATES[0]] == pytest.approx(-0.31)
    assert baseline[0][COORDINATES[1]] == pytest.approx(0.015)
    assert teacher().summary()["phase_shifts"] == dict.fromkeys(COORDINATES, 0.0)


@pytest.mark.parametrize("time", [0.137, 0.98, 1.43, 2.1])
def test_target_velocity_matches_position_derivative(time):
    target = teacher()
    step = 1e-6
    before, after = target.get(time - step)[0], target.get(time + step)[0]
    velocity = target.get(time)[1]
    for joint in COORDINATES:
        assert velocity[joint] == pytest.approx((after[joint] - before[joint]) / (2 * step), abs=1e-7)


@pytest.mark.parametrize("strikes", [[], [0.0], [0.0, np.nan], [1.0, 0.0], [0.0, 0.0], [4.0, 5.0]])
def test_missing_or_invalid_left_cycles_fail_without_temporal_fallback(strikes):
    with pytest.raises(ValueError, match="prescribed-left"):
        teacher(strikes=strikes)


def test_missing_coordinate_fails_even_if_other_spline_exists():
    kin = Kinematics()
    original = kin.get
    def missing(t):
        q, qdot, _ = original(t)
        q.pop(COORDINATES[1])
        return q, qdot, {}
    kin.get = missing
    with pytest.raises(ValueError, match="left-coordinate"):
        teacher(kin)


@pytest.mark.parametrize("time", [np.nan, np.inf, -1, 3, True])
def test_invalid_target_time_is_rejected(time):
    with pytest.raises(ValueError, match="prescribed-left"):
        teacher().get(time)


def bare_env(target_velocity_scale=10.0):
    env = UnifiedProsthesisEnv.__new__(UnifiedProsthesisEnv)
    env.base_kin = Kinematics()
    env.env_cfg = CMCEnvConfig()
    env.cfg = SimpleNamespace(pros_coords=COORDINATES, t_start=0.0, t_end=2.2)
    env._target_availability = EpisodeAvailability(1.0)
    env.target_velocity_scale = target_velocity_scale
    sides = []
    env._load_sound_heel_strikes = lambda side: sides.append(side) or [0.0, 1.0, 2.2]
    env._imitation_position_ranges_rad, env._imitation_velocity_ranges_rad_s = env._build_imitation_error_ranges()
    assert sides == ["left"]
    return env


def test_environment_ranges_use_phase_teacher_and_rebuild_it():
    env = bare_env()
    target = env._prescribed_left_teacher
    times = np.linspace(0.0, 2.2, env.env_cfg.imitation_phase_samples)
    for joint in COORDINATES:
        expected = np.ptp([target.get(t)[0][joint] for t in times])
        assert env._imitation_position_ranges_rad[joint] == pytest.approx(expected)
    env._build_imitation_error_ranges()
    assert env._prescribed_left_teacher is not target


@pytest.mark.parametrize("scale", [1.0, 10.0])
def test_mask_hides_observation_but_preserves_teacher_and_reward_inputs(monkeypatch, scale):
    env = bare_env(scale)
    env.t = 1.3
    env._n_actor = 35
    env._actor_feature_names = CLOCK_FEATURES + tuple(f"actor_{i}" for i in range(33))
    env._privileged_feature_names = tuple(f"critic_{i}" for i in range(49))
    env._observation_feature_names = env._actor_feature_names + env._privileged_feature_names
    base = np.arange(84, dtype=np.float32)
    monkeypatch.setattr(CMCLikeProsthesisTrajectoryEnv, "_get_observation", lambda self: (base, {}))
    env._target_availability.visible = True
    visible, _ = env._get_observation()
    payload = env._imitation_target_payload()
    q, qdot = payload["q"], payload["qdot"]
    expected = [q[COORDINATES[0]], qdot[COORDINATES[0]] / scale,
                q[COORDINATES[1]], qdot[COORDINATES[1]] / scale]
    np.testing.assert_allclose(visible[35:39], expected, atol=1e-7)
    np.testing.assert_allclose(visible[:2], [1, 0], atol=1e-7)  # left phase .25, zero shift
    env._target_availability.visible = False
    hidden, _ = env._get_observation()
    assert payload == env._imitation_target_payload()
    assert payload["source"] == TARGET_SOURCE
    for obs in (visible, hidden):
        assert obs.shape == (88,)
        np.testing.assert_array_equal(obs[2:35], base[2:35])
        np.testing.assert_array_equal(obs[39:], base[35:])
    np.testing.assert_array_equal(hidden[:2], [0, 0])
    np.testing.assert_array_equal(hidden[35:39], [0, 0, 0, 0])
    assert env._imitation_target_summary()["clock_actor_observation"] is True
    assert env._imitation_target_summary()["actor_clock_visible"] is False
    assert env._imitation_target_summary()["target_velocity_scale"] == scale


def contract(scale=10.0):
    names = list(CLOCK_FEATURES) + [f"actor_{i}" for i in range(33)] + list(TARGET_FEATURES)
    return actor_transition.build_contract(names, names + [f"critic_{i}" for i in range(49)], [-1, -1], [1, 1], target_velocity_scale=scale)


@pytest.mark.parametrize("legacy_version", ["unified_mlp39_v1", "unified_mlp39_v2"])
def test_legacy_checkpoint_contract_is_rejected(legacy_version):
    current = contract()
    assert current["schema_version"] == "unified_mlp39_v3"
    old = copy.deepcopy(current)
    old["schema_version"] = legacy_version
    with pytest.raises(ValueError, match="phase contract"):
        actor_transition.validate_contract(old, current)
    old = copy.deepcopy(current)
    old["target_channels"]["version"] = "prescribed_left_current_q_qdot_fixed_scale_v1"
    with pytest.raises(ValueError, match="contract mismatch"):
        actor_transition.validate_contract(old, current)


def test_dynamic_scale_contract_rejects_cross_scale_restore_and_channel_drift(tmp_path):
    from unified_MLP_39.target_availability import PRESCRIBED_INDICES
    scale1, scale10 = contract(1.0), contract(10.0)
    for expected in (scale1, scale10):
        path = tmp_path / str(expected["target_channels"]["target_velocity_scale"])
        actor_transition.write_contract(path, expected)
        actor_transition.validate_contract(actor_transition.read_contract(path), expected)
        assert expected["target_channels"]["actor_indices"] == list(PRESCRIBED_INDICES)
    with pytest.raises(ValueError, match="target_channels"):
        actor_transition.validate_contract(scale1, scale10)
    altered = copy.deepcopy(scale1)
    altered["target_channels"]["normalization_scale"][3] = 10.0
    with pytest.raises(ValueError, match="contract mismatch"):
        actor_transition.validate_contract(altered, scale1)
    altered = copy.deepcopy(scale1)
    altered["actor_feature_names"][0] = "other_clock"
    altered["observation_feature_names"][0] = "other_clock"
    with pytest.raises(ValueError, match="contract mismatch"):
        actor_transition.validate_contract(altered, scale1)
