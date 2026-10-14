"""Exercise mask callbacks against the real Gymnasium wrappers used by RLlib."""
from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import gymnasium as gym
import numpy as np
import pytest
from gymnasium.wrappers.vector import DictInfoToList


@pytest.fixture(params=["installed", "bundle"])
def target(request):
    # Load by path: the older feasibility suite imports a temporary package
    # under the same unified_MLP_39 name in the same pytest process.
    package = Path(__file__).resolve().parents[1]
    path = (package if request.param == "installed" else
            package.parent / "unified_MLP" / "validation" / "feasibility39_bundle")
    spec = importlib.util.spec_from_file_location("mask_worker_test", path / "target_availability.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class MaskEnv(gym.Env):
    observation_space = gym.spaces.Box(-1.0, 1.0, (1,), np.float32)
    action_space = gym.spaces.Discrete(2)

    def __init__(self, target, seed):
        self._target_availability = target.EpisodeAvailability(0.0, seed)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._target_availability.reset_seed(seed)
        self._target_availability.begin_episode()
        return np.zeros(1, np.float32), {}

    def step(self, action):
        self._target_availability.record_transition()
        return np.zeros(1, np.float32), 0.0, False, False, {}


@pytest.fixture
def runner_factory(target):
    vectors = []

    def create(width=2, worker=1):
        environments = [MaskEnv(target, 100 + i) for i in range(width)]
        # Match RLlib's DictInfoToList(SyncVectorEnv(...)); also exercise
        # another vector wrapper and ordinary wrappers on individual envs.
        vector = DictInfoToList(gym.vector.VectorWrapper(gym.vector.SyncVectorEnv([
            lambda env=env: gym.Wrapper(gym.wrappers.TimeLimit(env, max_episode_steps=100))
            for env in environments
        ])))
        vectors.append(vector)
        return SimpleNamespace(env=vector, worker_index=worker), environments

    yield create
    for vector in vectors:
        vector.close()


def test_wrapped_vector_updates_every_environment_and_counts_steps(target, runner_factory):
    runner, envs = runner_factory()
    assert not hasattr(runner.env, "envs")  # The original failure condition.
    assert target._runner_environments(runner) == envs
    result = target._runner_apply(runner, 1.0, {})
    assert result["environments"] == 2
    runner.env.reset(seed=42)
    runner.env.step(np.array([0, 1]))
    snapshot = target._runner_snapshot(runner)
    assert all(state["visible"] for state in snapshot["states"])
    assert all(state["total_steps"] == state["visible_steps"] == 1
               for state in snapshot["states"])
    target._runner_apply(runner, 0.0, {})
    assert all(env._target_availability.visible for env in envs)
    runner.env.reset()
    assert all(not env._target_availability.visible for env in envs)


def test_single_wrapped_environment_and_environmentless_local_runner(target):
    env = MaskEnv(target, 123)
    wrapped = gym.Wrapper(env)
    try:
        assert target._runner_environments(SimpleNamespace(env=wrapped)) == [env]
        assert target._runner_environments(SimpleNamespace(env=None)) == []
        assert target._runner_environments(SimpleNamespace()) == []
    finally:
        wrapped.close()


def test_vector_mask_checkpoint_restores_each_rng_in_order(target, runner_factory):
    original, original_envs = runner_factory()
    target._runner_apply(original, 0.5, {})
    original.env.reset(seed=7)
    original.env.step(np.array([0, 1]))
    saved = json.loads(json.dumps(target._runner_snapshot(original)))
    expected = [[env._target_availability.begin_episode() for _ in range(20)]
                for env in original_envs]
    restored, restored_envs = runner_factory()
    target._runner_apply(restored, 0.5, {saved["worker_index"]: saved["states"]})
    restored.env.reset(seed=999)  # Must retain the checkpoint stream.
    actual = [[env._target_availability.visible] +
              [env._target_availability.begin_episode() for _ in range(19)]
              for env in restored_envs]
    assert actual == expected
    assert all(env._target_availability.total_steps == 1 for env in restored_envs)


def test_resume_rejects_changed_vector_width_before_mutation(target, runner_factory):
    original, _ = runner_factory(width=2)
    saved = target._runner_snapshot(original)
    restored, envs = runner_factory(width=1)
    with pytest.raises(ValueError, match="vector width changed"):
        target._runner_apply(restored, 1.0, {"1": saved["states"]})
    assert envs[0]._target_availability.probability == 0.0


@pytest.mark.parametrize("root, message", [
    (SimpleNamespace(num_envs=2), "locally accessible"),
    (SimpleNamespace(envs=[], num_envs=2), "vector width disagrees"),
    (SimpleNamespace(), "controller missing"),
])
def test_unsupported_or_incomplete_environment_topology_fails_closed(target, root, message):
    with pytest.raises(ValueError, match=message):
        target._runner_apply(SimpleNamespace(env=root), 1.0, {})


def test_callbacks_preserve_worker_coverage_and_vector_batch_counts(target, runner_factory):
    runner, _ = runner_factory()
    local = SimpleNamespace(env=None, worker_index=0)
    runners = [local, runner]
    group = SimpleNamespace(foreach_env_runner=lambda func, **kwargs: [func(r) for r in runners])
    algo = SimpleNamespace(env_runner_group=group,
                           _unified_training_state={"reward_mode": "imitation", "last_completed_iteration": 0})
    args = SimpleNamespace(target_availability_probability=1.0, num_env_runners=1,
                           startup_timeout_s=1, train_batch_size=2)
    target.initialize_runtime(algo, args, None)
    target.before_iteration(algo, args)
    runner.env.reset(seed=42)
    runner.env.step(np.array([0, 1]))
    batch = target.after_iteration(algo, 1, args)
    assert batch["total_steps"] == batch["visible_steps"] == 2
    assert batch["visible_fraction"] == 1.0
    assert batch["matches_expected_train_batch_size"]
    saved = json.loads(json.dumps(algo._unified_training_state))
    algo._unified_training_state = saved
    target.initialize_runtime(algo, args, "checkpoint")
    runners[:] = [local]
    with pytest.raises(RuntimeError, match="not applied to any"):
        target.before_iteration(algo, args)
    # A new worker cannot silently replace a saved one.
    replacement, _ = runner_factory(worker=2)
    runners.append(replacement)
    with pytest.raises(ValueError, match="missing target mask RNG workers"):
        target.before_iteration(algo, args)
    with pytest.raises(RuntimeError, match="incomplete per-worker"):
        target.after_iteration(algo, 2, args)
