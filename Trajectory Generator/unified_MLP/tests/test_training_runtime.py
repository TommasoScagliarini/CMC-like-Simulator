"""Trainer lifecycle and resume decisions without a live Ray/OpenSim runtime."""

from __future__ import annotations

import copy
from types import SimpleNamespace

import gymnasium as gym
import numpy as np
import pytest
import torch

from unified_MLP.actor_transition import (
    actor_state_digest, build_contract, configure_constant_std, read_contract,
    resolve_module_export, write_contract,
)
from unified_MLP.asymmetric_rl_module import AsymmetricActorCriticTorchRLModule
from unified_MLP import training_runtime as runtime


def _module(seed):
    torch.manual_seed(seed)
    return AsymmetricActorCriticTorchRLModule(
        observation_space=gym.spaces.Box(-np.inf, np.inf, shape=(37,), dtype=np.float32),
        action_space=gym.spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32),
        model_config={"n_actor": 35},
    )


def _fixture(monkeypatch, mode="ex_novo"):
    learner = _module(1)
    state, _ = configure_constant_std(learner.get_state())
    learner.set_state(state)
    algo = SimpleNamespace(learner=learner, phase_history=[])
    names = [f"feature_{i}" for i in range(35)]
    args = SimpleNamespace(
        _unified_contract=build_contract(names, names + ["critic_0", "critic_1"], [-1.0]*2, [1.0]*2),
        init_actor_from=None, lr=3e-6,
    )
    class RewardConfig:
        @classmethod
        def from_mapping(cls, value):
            result = cls()
            result.values = dict(value)
            result.reward_mode = value["reward_mode"]
            return result

        def to_dict(self):
            return self.values

    driver = SimpleNamespace(
        reward_function=SimpleNamespace(RewardConfig=RewardConfig),
        _learner_module_state=lambda algo: algo.learner.get_state(),
        _reapply_optimizer_learning_rate=lambda algo, value: setattr(algo, "learning_rate", value),
    )
    def set_phase(algo, phase, driver, args):
        algo.phase_history.append(phase)
        algo.learner.set_training_phase(phase)
    monkeypatch.setattr(runtime, "set_phase", set_phase)
    return algo, args, driver, {"reward_mode": mode, "weight": 1.0}


def test_resume_completed_warmup_keeps_actor_and_skips_reinitialization(tmp_path, monkeypatch):
    algo, args, driver, reward = _fixture(monkeypatch)
    runtime.initialize(algo, args, tmp_path, None, reward, driver)
    before = runtime.before_iteration(algo, args, driver)
    assert before["phase"] == "critic_warmup"
    assert algo.learning_rate == 1e-4
    optimizer = torch.optim.Adam(algo.learner.parameters(), lr=algo.learning_rate)
    optimizer.zero_grad(set_to_none=True)
    algo.learner.compute_values({"obs": torch.randn(16, 37)}).square().mean().backward()
    optimizer.step()
    audit = runtime.after_iteration(algo, 1, before, args, driver)
    assert audit["actor_exact"] and audit["critic_changed"]
    assert algo.learner.get_training_phase_state()["phase"] == "ex_novo"
    assert algo.learning_rate == args.lr
    checkpoint = tmp_path / "checkpoint_1"
    checkpoint.mkdir()
    runtime.save_metadata(algo, checkpoint)
    saved_module_state = copy.deepcopy(algo.learner.get_state())
    saved_actor = actor_state_digest(saved_module_state)
    # Emulate PPO restoration into a new process/module with different initial flags.
    algo.learner = _module(333)
    algo.learner.set_state(saved_module_state)
    resume_dir = tmp_path / "resumed"
    resume_dir.mkdir()
    runtime.initialize(algo, args, resume_dir, checkpoint, reward, driver)
    next_phase = runtime.before_iteration(algo, args, driver)
    assert next_phase["phase"] == "ex_novo"
    assert algo._unified_training_state["last_completed_iteration"] == 1
    assert actor_state_digest(algo.learner.get_state()) == saved_actor


def test_resume_before_warmup_runs_it_once_and_reward_change_is_rejected(tmp_path, monkeypatch):
    algo, args, driver, reward = _fixture(monkeypatch)
    runtime.initialize(algo, args, tmp_path, None, reward, driver)
    checkpoint = tmp_path / "checkpoint_initial"
    checkpoint.mkdir()
    runtime.save_metadata(algo, checkpoint)
    runtime.initialize(algo, args, tmp_path, checkpoint, reward, driver)
    assert runtime.before_iteration(algo, args, driver)["phase"] == "critic_warmup"
    with pytest.raises(ValueError, match="retain its reward"):
        runtime.initialize(algo, args, tmp_path, checkpoint, {**reward, "reward_mode": "imitation"}, driver)


def test_imitation_never_performs_critic_only_phase(tmp_path, monkeypatch):
    algo, args, driver, reward = _fixture(monkeypatch, mode="imitation")
    runtime.initialize(algo, args, tmp_path, None, reward, driver)
    before = runtime.before_iteration(algo, args, driver)
    assert before["phase"] == "imitation"
    assert algo.learner.get_training_phase_state() == {
        "phase": "imitation", "freeze_actor": False, "freeze_logstd": False,
    }


def test_critic_only_audit_rejects_no_critic_progress(tmp_path, monkeypatch):
    algo, args, driver, reward = _fixture(monkeypatch)
    runtime.initialize(algo, args, tmp_path, None, reward, driver)
    before = runtime.before_iteration(algo, args, driver)
    with pytest.raises(RuntimeError, match="Critic-only warm-up failed"):
        runtime.after_iteration(algo, 1, before, args, driver)
    assert not algo._unified_training_state["critic_warmup_completed"]


def test_checkpoint_contract_is_found_only_in_known_rllib_layout(tmp_path, monkeypatch):
    algo, args, driver, reward = _fixture(monkeypatch)
    module_path = tmp_path / "learner_group" / "learner" / "rl_module" / "default_policy"
    module_path.mkdir(parents=True)
    (module_path / "module_state.pkl").touch()
    write_contract(tmp_path, args._unified_contract)
    assert resolve_module_export(tmp_path) == module_path
    assert read_contract(module_path) == args._unified_contract


@pytest.mark.parametrize("multi_module", [False, True])
def test_runner_phase_handles_standalone_and_multi_module(multi_module):
    module = _module(1)
    runner = SimpleNamespace(module={"default_policy": module} if multi_module else module)
    phase = runtime._set_phase_on_runner(runner, phase="critic_warmup")
    assert phase == {"phase": "critic_warmup", "freeze_actor": True, "freeze_logstd": True}
