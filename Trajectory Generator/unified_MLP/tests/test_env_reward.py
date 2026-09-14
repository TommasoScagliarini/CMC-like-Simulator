"""Contract tests without advancing OpenSim or running PPO."""
from dataclasses import asdict, replace
from pathlib import Path
from types import SimpleNamespace
import re

import gymnasium as gym
import numpy as np
import pytest
import yaml

from unified_MLP import env_factory, reward_function as unified
from unified_MLP.env import (
    CLOCK_FEATURES, TARGET_COORDINATES, UnifiedProsthesisEnv,
)
from baseline_MLP import reward_function as baseline
from osim_trj_cmc_like import CMCEnvConfig, CMCLikeProsthesisTrajectoryEnv


PILOT = Path(__file__).resolve().parents[2] / "baseline_MLP" / "experimental_configs" / "exnovo_v26c_j20_pilot_2026-08-28.yaml"


def pilot_reward(mode="ex_novo"):
    values = yaml.safe_load(PILOT.read_text())["reward"]
    return unified.RewardConfig.from_mapping({**values, "reward_mode": mode})


def test_finite_exnovo_reward_is_exactly_baseline():
    """Cover all scalar input fields, both morphology weighting paths and clipping."""
    source = Path(baseline.__file__).read_text()
    keys = set(re.findall(r'reward_terms\.get\("([^"]+)"', source))
    keys.update(value for name, value in vars(baseline).items()
                if name.endswith("_LOSS") and isinstance(value, str))
    rng = np.random.default_rng(6042)
    for split in (False, True):
        cfg = pilot_reward()
        if split:
            cfg = replace(cfg, morphology_knee_weight=0.4, morphology_ankle_weight=0.7)
        original = baseline.RewardConfig.from_mapping(cfg.to_dict())
        for _ in range(24):
            terms = {key: float(rng.uniform(0, 2)) for key in sorted(keys)}
            reference = rng.uniform(-1.4, 0.6, size=(2, 2))
            actual = unified.compute_reward(terms, cfg, reference=reference)[0]
            expected = baseline.compute_reward(terms, original, reference=reference)[0]
            assert actual == expected


def test_imitation_excludes_all_exnovo_rewards_even_nonfinite():
    cfg = pilot_reward("imitation")
    terms = {"sound_imitation_loss": 0.11, "served_imitation_loss": 0.07,
             "tracking_loss": 0.03, "safety_loss": 0.0}
    excluded = (
        "reference_loss", "bio_loss", "contact_load_score",
        "contact_support_to_score", "phase_regular_score", "phase_event_progress_score",
        "landing_window_contact_score", "phase_clawback_penalty",
        "phase_failure_extra_penalty", "contact_support_clawback_penalty",
        "swing_unloading_loss", "contact_overload_loss", "grf_slip_loss",
        "reserve_residual_loss", "pelvis_height_loss", "morphology_loss",
        "morphology_knee_loss", "morphology_ankle_loss",
    )
    expected = unified.compute_reward(terms, cfg)[0]
    for bad in (float("nan"), float("inf"), 1e9):
        actual, components = unified.compute_reward({**terms, **dict.fromkeys(excluded, bad)}, cfg)
        assert np.isfinite(actual) and actual == expected
        assert components["morphology_term"] == 0.0
        assert components["exnovo_task_term"] == 0.0
        assert components["phase_clawback_penalty_term"] == 0.0
    # Scalar gating cannot mutate the wrapper's causal-buffer configuration.
    assert cfg.morphology_weight == pilot_reward("imitation").morphology_weight


def test_exnovo_ignores_nonfinite_imitation_and_zero_weight_penalties():
    cfg = replace(pilot_reward(), effort_weight=0.0)
    expected = unified.compute_reward({}, cfg)[0]
    result, _ = unified.compute_reward({
        "sound_imitation_loss": np.nan,
        "served_imitation_loss": np.inf,
        "effort_loss": np.nan,
    }, cfg)
    assert result == expected


@pytest.mark.parametrize("mode", ["imitation", "ex_novo"])
def test_shared_safety_and_timeout_remain_active(mode):
    cfg = replace(pilot_reward(mode), safety_weight=2.0, phase_timeout_penalty_weight=0.5)
    base = unified.compute_reward({}, cfg)[0]
    actual = unified.compute_reward({"safety_loss": 0.3, "phase_timeout_loss": 0.4}, cfg)[0]
    assert actual == pytest.approx(base - 0.8)


class FakeKinematics:
    time_bounds = (0.0, 4.0)
    right = 123.0

    def get(self, t):
        return (
            {"pros_knee_angle": -0.2 * t, "pros_ankle_angle": 0.1 * t,
             "knee_angle_r": self.right, "ankle_angle_r": -self.right},
            {"pros_knee_angle": -0.2, "pros_ankle_angle": 0.1,
             "knee_angle_r": self.right, "ankle_angle_r": -self.right},
            {},
        )


def bare_env():
    env = UnifiedProsthesisEnv.__new__(UnifiedProsthesisEnv)
    env.base_kin = FakeKinematics()
    env.env_cfg = CMCEnvConfig()
    env.cfg = SimpleNamespace(pros_coords=TARGET_COORDINATES, t_start=0.0, t_end=4.0)
    return env


def test_prescribed_target_is_left_same_time_and_derivatives():
    env = bare_env()
    target = env.imitation_target(1.5)
    assert target == ({"pros_knee_angle": -0.2 * 1.5, "pros_ankle_angle": 0.1 * 1.5},
                      {"pros_knee_angle": -0.2, "pros_ankle_angle": 0.1}, {})
    env.base_kin.right = np.nan
    assert env.imitation_target(1.5) == target
    q_range, velocity_range = env._build_imitation_error_ranges()
    assert q_range == {"pros_knee_angle": 0.8, "pros_ankle_angle": 0.4}
    assert all(value > 0 for value in velocity_range.values())


@pytest.mark.parametrize("bad", [np.nan, np.inf, True])
def test_missing_or_invalid_left_target_fails_without_right_fallback(bad):
    env = bare_env()
    env.base_kin.get = lambda t: ({"pros_knee_angle": bad, "pros_ankle_angle": 0.1},
                                  {"pros_knee_angle": 0.0, "pros_ankle_angle": 0.0}, {})
    with pytest.raises(ValueError, match="prescribed-left"):
        env.imitation_target(1.0)
    env.base_kin.get = lambda t: ({"knee_angle_r": 0.0}, {}, {})
    with pytest.raises(ValueError, match="missing"):
        env.imitation_target(1.0)


def test_disabled_clock_slots_are_both_zero(monkeypatch):
    env = bare_env()
    env._n_actor = 35
    env._actor_feature_names = CLOCK_FEATURES + tuple(f"actor_{i}" for i in range(33))
    env._observation_feature_names = env._actor_feature_names
    values = dict(zip(env._actor_feature_names, np.arange(35, dtype=float)))
    original = np.arange(35, dtype=np.float32)
    monkeypatch.setattr(CMCLikeProsthesisTrajectoryEnv, "_get_observation", lambda self: (original, values))
    observation, info = env._get_observation()
    assert observation[:2].tolist() == [0.0, 0.0]
    assert all(info[name] == 0 for name in CLOCK_FEATURES)
    np.testing.assert_array_equal(observation[2:], original[2:])
    assert original[1] == 1.0


class DummyEnv(gym.Env):
    action_space = gym.spaces.Box(-1.0, 1.0, (1, 2), dtype=np.float32)
    observation_space = gym.spaces.Box(-np.inf, np.inf, (35,), dtype=np.float32)

    def __init__(self, cfg):
        self.env_cfg = cfg


def test_factory_has_one_actor_and_reset_contract(monkeypatch):
    monkeypatch.setattr(env_factory, "UnifiedProsthesisEnv", DummyEnv)
    stages = [env_factory.make_cmc_env({"reward": {"reward_mode": mode}})
              for mode in ("imitation", "ex_novo")]
    assert asdict(stages[0].unwrapped.env_cfg) == asdict(stages[1].unwrapped.env_cfg)
    assert stages[0].action_space.shape == (2,)
    assert not stages[0].unwrapped.env_cfg.include_imitation_target_observation
    assert not stages[0].unwrapped.env_cfg.imitation_initialize_to_target
    assert env_factory.ENV_NAME != "cmc_traj_env"


@pytest.mark.parametrize("override", [
    {"gait_clock_enable": True}, {"policy_knots": 2},
    {"include_imitation_target_observation": True},
    {"imitation_initialize_to_target": True},
    {"binary_phase_actor_fsm_version": "v2"},
])
def test_factory_rejects_contract_drift(override):
    with pytest.raises(ValueError, match="unified v1"):
        env_factory.build_env_config(override)


@pytest.mark.parametrize("mode", ["imitation", "ex_novo"])
def test_causal_morphology_buffer_is_preserved_in_both_stages(mode):
    cfg = pilot_reward(mode)
    wrapper = unified.RewardShapingWrapper(DummyEnv(None), cfg)
    assert wrapper._causal_morphology_buffer is not None
    assert wrapper._causal_morphology_buffer.delay_s == 0.04
    assert wrapper.reward_config is cfg
