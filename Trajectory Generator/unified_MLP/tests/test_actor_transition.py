"""Transition invariants exercised without OpenSim or Ray cluster startup."""

from __future__ import annotations

import copy
import json
import os
from pathlib import Path
import subprocess
import sys

import gymnasium as gym
import numpy as np
import pytest
import torch

from unified_MLP.actor_transition import (
    MODULE_CLASS,
    actor_mean_digest,
    actor_state_digest,
    build_contract,
    configure_constant_std,
    copy_actor_state,
    load_actor_source,
    read_contract,
    resolve_module_export,
    validate_contract,
    validate_module_contract,
    write_contract,
)
from unified_MLP.asymmetric_rl_module import AsymmetricActorCriticTorchRLModule


def _contract():
    actor = [f"actor_{i}" for i in range(35)]
    return build_contract(actor, actor + ["critic_0", "critic_1"], [-1.0, -1.0], [1.0, 1.0])


def _module(seed=1, **config):
    torch.manual_seed(seed)
    return AsymmetricActorCriticTorchRLModule(
        observation_space=gym.spaces.Box(-np.inf, np.inf, shape=(37,), dtype=np.float32),
        action_space=gym.spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32),
        model_config={"n_actor": 35, "fcnet_hiddens": [256, 256], "fcnet_activation": "tanh", **config},
    )


def _critic_state(module):
    return {key: value.copy() for key, value in module.get_state().items() if key.startswith(("vf.", "vf_encoder."))}


def test_direct_actor_copy_preserves_new_critic_and_phase():
    source, target = _module(1), _module(2)
    target.set_training_phase("critic_warmup")
    before = target.get_state()
    initialized, report = copy_actor_state(source.get_state(), before, _contract(), _contract())
    target.set_state(initialized)
    assert report["actor_exact"] and report["critic_and_phase_unchanged"]
    assert actor_state_digest(source.get_state()) == actor_state_digest(target.get_state())
    assert target.get_training_phase_state()["phase"] == "critic_warmup"
    assert all(np.array_equal(value, target.get_state()[key]) for key, value in before.items() if key.startswith("vf"))
    obs = torch.randn(17, 37)
    assert torch.equal(source.forward_inference({"obs": obs})["action_dist_inputs"], target.forward_inference({"obs": obs})["action_dist_inputs"])


@pytest.mark.parametrize("change", ["order", "full_order", "bounds", "class", "version"])
def test_contract_rejects_mismatches(change):
    expected = _contract()
    source = copy.deepcopy(expected)
    if change == "order":
        source["actor_feature_names"][0:2] = reversed(source["actor_feature_names"][0:2])
        source["observation_feature_names"][0:2] = reversed(source["observation_feature_names"][0:2])
    elif change == "full_order":
        source["observation_feature_names"][-2:] = reversed(source["observation_feature_names"][-2:])
    elif change == "bounds":
        source["action_high"][0] = 2.0
    elif change == "class":
        source["module_class"] = "asymmetric_rl_module.AsymmetricActorCriticTorchRLModule"
    else:
        source["schema_version"] = 1
    with pytest.raises(ValueError):
        validate_contract(source, expected)


def test_actor_copy_rejects_inconsistent_duplicate_tensor_and_shapes():
    source, target = _module().get_state(), _module(2).get_state()
    source = {key: np.array(value, copy=True) for key, value in source.items()}
    source["pi_encoder.0.weight"][0, 0] += 1
    with pytest.raises(ValueError, match="duplicate"):
        copy_actor_state(source, target, _contract(), _contract())
    source = _module().get_state()
    source["pi.1.weight"] = np.zeros((2, 256), dtype=np.float32)
    with pytest.raises(ValueError, match="shape"):
        copy_actor_state(source, target, _contract(), _contract())


@pytest.mark.parametrize("as_torch", [False, True])
def test_sigma_configuration_preserves_entire_mean_network(as_torch):
    module = _module()
    original = module.state_dict() if as_torch else module.get_state()
    original_digest = actor_state_digest(original)
    state, report = configure_constant_std(original, sigma=0.010)
    assert report["mean_parameters_exact"]
    assert actor_state_digest(original) == original_digest
    assert actor_mean_digest(original) == actor_mean_digest(state)
    module.set_state(state)
    logits = module.forward_inference({"obs": torch.randn(128, 37)})["action_dist_inputs"]
    torch.testing.assert_close(logits[:, 2:].exp(), torch.full((128, 2), 0.010))


@pytest.mark.parametrize("sigma", [0, -0.1, np.nan, np.inf, [0.1, 0.2, 0.3]])
def test_sigma_rejects_invalid_values(sigma):
    with pytest.raises(ValueError, match="sigma"):
        configure_constant_std(_module().get_state(), sigma=sigma)


def test_critic_warmup_freezes_all_actor_aliases_and_resume_restores_phase():
    module = _module()
    state, _ = configure_constant_std(module.get_state(), sigma=0.010)
    module.set_state(state)
    module.set_training_phase("critic_warmup")
    actor_digest = actor_state_digest(module.get_state())
    critic_before = _critic_state(module)
    optimizer = torch.optim.Adam(module.parameters(), lr=1e-3)
    observations = {"obs": torch.randn(32, 37)}
    for _ in range(3):
        optimizer.zero_grad(set_to_none=True)
        output = module.forward_train(observations)
        loss = module.compute_values(observations, embeddings=output["embeddings"]).square().mean()
        loss += output["action_dist_inputs"].square().mean()
        loss.backward()
        assert all(parameter.grad is None for parameter in module.pi.parameters())
        optimizer.step()
    assert actor_state_digest(module.get_state()) == actor_digest
    assert any(not np.array_equal(value, module.get_state()[key]) for key, value in critic_before.items())
    fresh = _module(123, freeze_actor=False, freeze_logstd=False)
    fresh.set_state(module.get_state())
    assert fresh.get_training_phase_state() == {"phase": "critic_warmup", "freeze_actor": True, "freeze_logstd": True}
    assert "_unified_training_phase" in fresh.get_state(inference_only=True)
    # A fresh optimizer is used for the transition. Its logstd momentum is zero;
    # critic-only warm-up creates no actor momentum before PPO starts.
    module.set_training_phase("ex_novo")
    for _ in range(3):
        optimizer.zero_grad(set_to_none=True)
        logits = module.forward_train(observations)["action_dist_inputs"]
        logits.square().mean().backward()
        optimizer.step()
    assert actor_mean_digest(module.get_state()) != actor_mean_digest(state)
    np.testing.assert_array_equal(module.get_state()["pi.1.weight"][2:], state["pi.1.weight"][2:])
    np.testing.assert_array_equal(module.get_state()["pi.1.bias"][2:], state["pi.1.bias"][2:])


def test_imitation_logstd_is_learnable_and_privileged_suffix_never_reaches_actor():
    module = _module()
    assert module.get_training_phase_state()["phase"] == "imitation"
    optimizer = torch.optim.Adam(module.parameters(), lr=1e-3)
    obs = torch.randn(16, 37)
    changed = obs.clone()
    changed[:, 35:] += 100
    assert torch.equal(module.forward_inference({"obs": obs})["action_dist_inputs"], module.forward_inference({"obs": changed})["action_dist_inputs"])
    before = module.get_state()["pi.1.bias"].copy()
    module.forward_train({"obs": obs})["action_dist_inputs"].square().mean().backward()
    optimizer.step()
    assert not np.array_equal(before[2:], module.get_state()["pi.1.bias"][2:])


def test_export_reload_in_fresh_process_preserves_class_actor_and_phase(tmp_path):
    module = _module()
    module.set_training_phase("ex_novo")
    export = tmp_path / "rl_module_final"
    module.save_to_path(str(export))
    write_contract(export, _contract())
    assert read_contract(export)["module_class"] == MODULE_CLASS
    assert resolve_module_export(tmp_path) == export
    state, provenance = load_actor_source(export, _contract())
    assert actor_state_digest(state) == actor_state_digest(module.get_state())
    assert not provenance["critic_state_loaded"] and not provenance["optimizer_state_loaded"]
    code = '''import json, sys
from ray.rllib.core.rl_module.rl_module import RLModule
from unified_MLP.actor_transition import actor_state_digest, read_contract, validate_module_contract
m = RLModule.from_checkpoint(sys.argv[1])
validate_module_contract(m, read_contract(sys.argv[1]))
print(json.dumps({"digest": actor_state_digest(m.get_state()), "phase": m.get_training_phase_state()}))
'''
    env = dict(os.environ)
    traj_dir = str(Path(__file__).resolve().parents[2])
    env["PYTHONPATH"] = os.pathsep.join([traj_dir, env.get("PYTHONPATH", "")])
    completed = subprocess.run([sys.executable, "-c", code, str(export)], env=env, check=True, capture_output=True, text=True, timeout=90)
    result = json.loads(completed.stdout.strip().splitlines()[-1])
    assert result["digest"] == actor_state_digest(module.get_state())
    assert result["phase"] == module.get_training_phase_state()


def test_source_requires_unified_manifest_and_actual_module_class(tmp_path):
    module = _module()
    export = tmp_path / "module"
    module.save_to_path(str(export))
    with pytest.raises(FileNotFoundError, match="unified_contract"):
        load_actor_source(export, _contract())
    validate_module_contract(module, _contract())
    wrong = _contract()
    wrong["action_low"] = [-2.0, -1.0]
    with pytest.raises(ValueError, match="action_low"):
        validate_module_contract(module, wrong)
    module.pi_encoder[1] = torch.nn.ReLU()
    with pytest.raises(ValueError, match="tanh"):
        validate_module_contract(module, _contract())


def test_baseline_module_cannot_be_initialized_as_unified_even_with_manifest(tmp_path):
    from baseline_MLP.asymmetric_rl_module import AsymmetricActorCriticTorchRLModule as BaselineModule

    baseline = BaselineModule(
        observation_space=gym.spaces.Box(-np.inf, np.inf, shape=(37,), dtype=np.float32),
        action_space=gym.spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32),
        model_config={"n_actor": 35},
    )
    export = tmp_path / "baseline_export"
    baseline.save_to_path(str(export))
    write_contract(export, _contract())
    with pytest.raises(ValueError, match="not the unified V1 class"):
        load_actor_source(export, _contract())
