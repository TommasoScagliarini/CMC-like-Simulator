"""V3 input semantics, episode masking, and checkpoint compatibility."""
import argparse
from types import SimpleNamespace

import gymnasium as gym
import numpy as np
import pytest
import torch

from unified_MLP_39 import actor_transition, training_config, runtime_contract
from unified_MLP_39.asymmetric_rl_module import AsymmetricActorCriticTorchRLModule
from unified_MLP_39.target_availability import (
    EpisodeAvailability, PRESCRIBED_INDICES, PRESCRIBED_CLOCK_FEATURES, TARGET_FEATURES,
    add_cli_arguments, configuration, env_options, extend_observation, velocity_scale,
)


def contract(scale):
    names = list(PRESCRIBED_CLOCK_FEATURES) + [f"actor_{i}" for i in range(33)] + list(TARGET_FEATURES)
    return actor_transition.build_contract(names, names + [f"critic_{i}" for i in range(49)], [-1, -1], [1, 1], target_velocity_scale=scale)


def module():
    torch.manual_seed(123)
    return AsymmetricActorCriticTorchRLModule(
        observation_space=gym.spaces.Box(-np.inf, np.inf, (88,), np.float32),
        action_space=gym.spaces.Box(-1.0, 1.0, (2,), np.float32),
        model_config={"n_actor": 39, "fcnet_hiddens": [256, 256], "fcnet_activation": "tanh"},
    )


@pytest.mark.parametrize("scale", [0., -1., np.nan, np.inf, -np.inf, True])
def test_invalid_velocity_scale_fails_cli_yaml_and_environment_options(scale):
    with pytest.raises(ValueError, match="finite and positive"):
        velocity_scale(scale)
    with pytest.raises(ValueError, match="target_velocity_scale"):
        training_config.to_argparse_defaults({"target_availability": {"target_velocity_scale": scale}})
    with pytest.raises(ValueError, match="finite and positive"):
        env_options(SimpleNamespace(target_velocity_scale=scale))


@pytest.mark.parametrize("scale", [1., 10.])
def test_cli_yaml_snapshot_env_and_contract_share_one_scale(scale, tmp_path):
    parser = argparse.ArgumentParser()
    add_cli_arguments(parser)
    defaults, reward = training_config.to_argparse_defaults({
        "target_availability": {"target_velocity_scale": 3., "target_availability_probability": 1.},
    })
    parser.set_defaults(**defaults)
    args = parser.parse_args(["--target-velocity-scale", str(scale)])
    assert configuration(args)["target_velocity_scale"] == scale
    assert env_options(args)["target_velocity_scale"] == scale
    assert env_options(args, evaluation=True)["target_velocity_scale"] == scale
    path = tmp_path / "training_cfg.resolved.yaml"
    training_config.dump_resolved(args, reward, path)
    snapshot = training_config.load(path)
    assert snapshot["target_availability"]["target_velocity_scale"] == scale
    assert contract(scale)["target_channels"]["normalization_scale"] == [1, 1, 1, scale, 1, scale]


def test_episode_draw_masks_clock_and_four_targets_together_without_masking_online_signals():
    availability = EpisodeAvailability(.5, seed=321)
    base = np.arange(84, dtype=np.float32)
    raw = [-.5, 2., .3, -1.]
    seen = set()
    for _ in range(20):
        visible = availability.begin_episode()
        seen.add(visible)
        observations = [extend_observation(base, raw, visible, phase=phase) for phase in (.1, .2, .3)]
        for phase, obs in zip((.1, .2, .3), observations):
            expected = [np.sin(2*np.pi*phase), np.cos(2*np.pi*phase), -.5, .2, .3, -.1]
            np.testing.assert_allclose(obs[list(PRESCRIBED_INDICES)], expected if visible else np.zeros(6), atol=1e-7)
            np.testing.assert_array_equal(obs[2:35], base[2:35])
            np.testing.assert_array_equal(obs[39:], base[35:])
    assert seen == {True, False}
    ex_novo = EpisodeAvailability(1., reward_mode="ex_novo")
    assert ex_novo.probability == 0.0 and ex_novo.begin_episode() is False


@pytest.mark.parametrize("phase", ["critic_warmup", "ex_novo"])
def test_module_defense_masks_all_six_in_actor_and_critic_and_survives_restore(phase):
    policy = module()
    policy.set_training_phase(phase)
    obs = torch.randn(8, 88)
    changed = obs.clone()
    changed[:, list(PRESCRIBED_INDICES)] += 50.
    before = obs.clone()
    for candidate in (policy, module()):
        candidate.set_state(policy.get_state())
        torch.testing.assert_close(candidate.forward_inference({"obs": obs})["action_dist_inputs"],
                                   candidate.forward_inference({"obs": changed})["action_dist_inputs"], rtol=0, atol=0)
        torch.testing.assert_close(candidate.compute_values({"obs": obs}), candidate.compute_values({"obs": changed}), rtol=0, atol=0)
        masked = candidate._masked_observation({"obs": changed})
        torch.testing.assert_close(masked[:, list(PRESCRIBED_INDICES)], torch.zeros(8, 6), rtol=0, atol=0)
    torch.testing.assert_close(obs, before, rtol=0, atol=0)


def test_imitation_module_preserves_visible_clock_and_targets():
    policy = module()
    obs = torch.randn(8, 88)
    assert policy._masked_observation({"obs": obs}) is obs
    changed = obs.clone()
    changed[:, :2] += 2.
    assert not torch.equal(policy.forward_inference({"obs": obs})["action_dist_inputs"], policy.forward_inference({"obs": changed})["action_dist_inputs"])


@pytest.mark.parametrize("scale", [1., 10.])
def test_export_load_uses_dynamic_contract_and_rejects_scale_mismatch(scale, tmp_path):
    policy = module()
    path = tmp_path / "rl_module"
    policy.save_to_path(str(path))
    actor_transition.write_contract(path, contract(scale))
    loaded, _ = actor_transition.load_actor_source(path, contract(scale))
    assert actor_transition.actor_state_digest(loaded) == actor_transition.actor_state_digest(policy.get_state())
    with pytest.raises(ValueError, match="target_channels"):
        actor_transition.load_actor_source(path, contract(10. if scale == 1. else 1.))


def test_runtime_compatibility_checks_scale_independently_of_execution_budget():
    config = {"model": {"num_hidden_layers": 2, "dim_hidden_layers": 256, "fcnet_activation": "tanh",
                        "asymmetric_actor_critic": True, "rl_module_kind": "asymmetric"},
              "simulation": {"action_mode": "absolute", "iterations": 5}, "grf": {"grf_mode": "online_sensor"},
              "target_availability": {"target_velocity_scale": 10.}}
    other = {**config, "simulation": {"action_mode": "absolute", "iterations": 100}}
    runtime_contract.validate_runtime_compatibility(config, other)
    other["target_availability"] = {"target_velocity_scale": 1.}
    with pytest.raises(ValueError, match="target_velocity_scale"):
        runtime_contract.validate_runtime_compatibility(config, other)
