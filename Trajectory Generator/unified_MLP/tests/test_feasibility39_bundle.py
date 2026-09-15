"""Deferred variant materialized only under pytest's temporary directory."""
from __future__ import annotations

import copy
import hashlib
import importlib
import json
from pathlib import Path
from types import SimpleNamespace

import gymnasium as gym
import numpy as np
import pytest
import torch

from unified_MLP.validation.feasibility39_bundle import build_variant


@pytest.fixture(scope="module")
def variant(tmp_path_factory):
    path = build_variant.materialize_variant(tmp_path_factory.mktemp("variant39") / "unified_MLP_39")
    import sys
    sys.path.insert(0, str(path.parent))
    modules = {name: importlib.import_module(f"unified_MLP_39.{name}") for name in (
        "target_availability", "env", "actor_transition", "asymmetric_rl_module",
        "training_config", "training_runtime", "reward_function",
    )}
    yield SimpleNamespace(path=path, **modules)
    sys.path.remove(str(path.parent))


def module(variant, seed=8):
    torch.manual_seed(seed)
    return variant.asymmetric_rl_module.AsymmetricActorCriticTorchRLModule(
        observation_space=gym.spaces.Box(-np.inf, np.inf, (88,), np.float32),
        action_space=gym.spaces.Box(-1.0, 1.0, (2,), np.float32),
        model_config={"n_actor": 39},
    )


def contract(variant):
    actor = [f"actor_{i}" for i in range(35)] + list(variant.target_availability.TARGET_FEATURES)
    full = actor + [f"critic_{i}" for i in range(49)]
    return variant.actor_transition.build_contract(actor, full, [-1, -1], [1, 1])


def test_materialization_isolated_namespace_and_source_integrity(variant):
    receipt = json.loads((variant.path / "variant39_build_receipt.json").read_text())
    for relative, digest in receipt["source_sha256"].items():
        assert hashlib.sha256((build_variant.SOURCE / relative).read_bytes()).hexdigest() == digest
    assert not (variant.path / "runs").exists()
    assert not (variant.path / "tests").exists()
    assert not (variant.path / "validation").exists()
    for path in variant.path.glob("*.py"):
        compile(path.read_text(), str(path), "exec")
        assert "from unified_MLP." not in path.read_text()
        assert "from unified_MLP import" not in path.read_text()
    with pytest.raises(FileExistsError):
        build_variant.materialize_variant(variant.path)


def test_schema39_normalization_version_and_base35_preserved(variant):
    target = variant.target_availability
    base = np.arange(84, dtype=np.float32)
    before = base.copy()
    visible = target.extend_observation(base, [0.4, 2, -0.3, -5], True)
    hidden = target.extend_observation(base, [9, 80, -4, -10], False)
    assert visible.shape == hidden.shape == (88,)
    np.testing.assert_array_equal(visible[:2], [0, 0])
    np.testing.assert_array_equal(visible[2:35], base[2:35])
    np.testing.assert_array_equal(visible[39:], base[35:])
    np.testing.assert_allclose(visible[35:39], [0.4, 0.2, -0.3, -0.5])
    np.testing.assert_array_equal(hidden[35:39], [0, 0, 0, 0])
    np.testing.assert_array_equal(base, before)
    manifest = contract(variant)
    assert manifest["schema_version"] == "unified_mlp39_v1"
    variant.actor_transition.validate_module_contract(module(variant), manifest)
    incompatible = copy.deepcopy(manifest)
    incompatible["target_channels"]["normalization_scale"][1] = 1
    with pytest.raises(ValueError, match="normalization"):
        variant.actor_transition.validate_contract(manifest, incompatible)


def test_mask_does_not_change_original_imitation_teacher(variant, monkeypatch):
    env = variant.env.UnifiedProsthesisEnv.__new__(variant.env.UnifiedProsthesisEnv)
    env._target_availability = variant.target_availability.EpisodeAvailability(0)
    env._n_actor = 35
    env._actor_feature_names = variant.env.CLOCK_FEATURES + tuple(f"actor_{i}" for i in range(33))
    env._privileged_feature_names = tuple(f"critic_{i}" for i in range(49))
    env._observation_feature_names = env._actor_feature_names + env._privileged_feature_names
    env.t = 1.5
    env.base_kin = SimpleNamespace(time_bounds=(0, 4), get=lambda t: (
        {"pros_knee_angle": -0.2*t, "pros_ankle_angle": 0.1*t},
        {"pros_knee_angle": -0.2, "pros_ankle_angle": 0.1}, {},
    ))
    monkeypatch.setattr(variant.env.CMCLikeProsthesisTrajectoryEnv, "_get_observation", lambda self: (np.arange(84, dtype=np.float32), {}))
    teacher = env.imitation_target(env.t)
    hidden, info = env._get_observation()
    assert env.imitation_target(env.t) == teacher
    assert all(info[name] == 0 for name in variant.target_availability.TARGET_FEATURES)
    env._target_availability.set_probability(1)
    env._target_availability.begin_episode()
    visible, _ = env._get_observation()
    assert env.imitation_target(env.t) == teacher
    assert visible[35] == pytest.approx(teacher[0]["pros_knee_angle"])
    np.testing.assert_array_equal(hidden[39:], visible[39:])
    # Reward implementation is copied byte-for-byte except namespace imports.
    source = (build_variant.SOURCE / "reward_function.py").read_text().replace("unified_MLP", "unified_MLP_39")
    assert (variant.path / "reward_function.py").read_text() == source


def test_episode_mask_fixed_across_ppo_epochs_with_unit_ratio(variant):
    target = variant.target_availability
    availability = target.EpisodeAvailability(0.5, seed=45)
    availability.begin_episode()
    collected = target.extend_observation(np.ones(84), [0.5, 2, -0.1, -1], availability.visible)
    state = availability.get_state()
    policy = module(variant)
    batch = {"obs": torch.tensor(collected).unsqueeze(0)}
    first = policy.forward_exploration(batch)["action_dist_inputs"]
    mean, logstd = first.chunk(2, -1)
    action = mean.detach() + 0.1
    old_logp = torch.distributions.Normal(mean, logstd.exp()).log_prob(action).sum(-1).detach()
    availability.set_probability(1 - float(availability.visible))
    # Changing next-episode probability cannot alter already collected samples.
    for _ in range(4):
        current = policy.forward_train(batch)["action_dist_inputs"]
        mean, logstd = current.chunk(2, -1)
        logp = torch.distributions.Normal(mean, logstd.exp()).log_prob(action).sum(-1)
        assert torch.equal((logp - old_logp).exp(), torch.ones(1))
        assert torch.equal(first, current)
    assert availability.visible == state["visible"]
    assert availability.get_state()["rng_state"] == state["rng_state"]


def test_positive_control_visible_teacher_influences_actor_and_critic(variant):
    policy = module(variant)
    obs = torch.randn(6, 88, requires_grad=True)
    policy.set_training_phase("imitation")
    logits = policy.forward_train({"obs": obs})["action_dist_inputs"]
    (logits.square().sum() + policy.compute_values({"obs": obs}).square().sum()).backward()
    assert torch.count_nonzero(obs.grad[:, 35:39]) > 0
    changed = obs.detach().clone()
    changed[:, 35:39] += 10
    assert not torch.equal(logits, policy.forward_train({"obs": changed})["action_dist_inputs"])


@pytest.mark.parametrize("phase", ["critic_warmup", "ex_novo"])
def test_hidden_target_forward_and_input_gradient_independence(variant, phase):
    policy = module(variant)
    policy.set_training_phase(phase)
    source = torch.randn(6, 88)
    expected_logits = policy.forward_train({"obs": source})["action_dist_inputs"]
    expected_values = policy.compute_values({"obs": source})
    for values in (torch.zeros(6, 4), torch.randn(6, 4) * 100, source.roll(2, 0)[:, 35:39]):
        changed = source.clone()
        changed[:, 35:39] = values
        changed.requires_grad_(True)
        logits = policy.forward_train({"obs": changed})["action_dist_inputs"]
        value = policy.compute_values({"obs": changed})
        assert torch.equal(logits, expected_logits)
        assert torch.equal(value, expected_values)
        loss = value.square().sum()
        if logits.requires_grad:
            loss = loss + logits.square().sum()
        loss.backward()
        assert torch.count_nonzero(changed.grad[:, 35:39]) == 0


def test_exnovo_ppo_update_independent_of_all_teacher_input_copies(variant):
    """Actual RLlib PPO loss and paired Adam updates, no Ray cluster required."""
    from ray.rllib.algorithms.ppo import PPOConfig
    from ray.rllib.algorithms.ppo.torch.ppo_torch_learner import PPOTorchLearner
    from ray.rllib.core.columns import Columns
    from ray.rllib.evaluation.postprocessing import Postprocessing
    first, second = module(variant), module(variant)
    first.set_training_phase("ex_novo")
    second.set_training_phase("ex_novo")
    original = copy.deepcopy(first.state_dict())
    raw = torch.randn(12, 88)
    shifted = raw.clone()
    shifted[:, 35:39] = torch.randn(12, 4) * 200
    action = torch.randn(12, 2) * 0.1
    results = []
    cfg = variant.reward_function.RewardConfig.from_mapping({"reward_mode": "ex_novo"})
    reward_a = variant.reward_function.compute_reward({"sound_imitation_loss": 0.1, "served_imitation_loss": 0.2}, cfg)[0]
    reward_b = variant.reward_function.compute_reward({"sound_imitation_loss": 1000, "served_imitation_loss": 300}, cfg)[0]
    assert reward_a == reward_b
    for policy, obs, reward in ((first, raw, reward_a), (second, shifted, reward_b)):
        optimizer = torch.optim.Adam(policy.parameters(), lr=3e-6)
        optimizer.zero_grad()
        values = policy.compute_values({"obs": obs})
        returns = torch.linspace(-1, 1, 12) + float(reward)
        advantage = returns - values.detach()
        forward = policy.forward_train({"obs": obs})
        logits = forward["action_dist_inputs"]
        dist = policy.get_train_action_dist_cls().from_logits(logits)
        logp = dist.logp(action)
        old_logp = logp.detach() - 0.01
        learner = SimpleNamespace(
            module={"default_policy": policy},
            entropy_coeff_schedulers_per_module={"default_policy": SimpleNamespace(get_current_value=lambda: 0.0)},
            curr_kl_coeffs_per_module={"default_policy": torch.tensor(0.2)},
            metrics=SimpleNamespace(log_dict=lambda *a, **kw: None),
        )
        batch = {Columns.OBS: obs, Columns.ACTIONS: action, Columns.ACTION_LOGP: old_logp,
                 Columns.ACTION_DIST_INPUTS: logits.detach(), Postprocessing.ADVANTAGES: advantage,
                 Postprocessing.VALUE_TARGETS: returns}
        loss = PPOTorchLearner.compute_loss_for_module(
            learner, module_id="default_policy", config=PPOConfig(), batch=batch, fwd_out=forward,
        )
        loss.backward()
        optimizer.step()
        results.append((values.detach(), advantage, loss.detach(), copy.deepcopy(policy.state_dict()), optimizer.state_dict()))
    for a, b in zip(results[0][:3], results[1][:3]):
        assert torch.equal(a, b)
    for key in results[0][3]:
        assert torch.equal(results[0][3][key], results[1][3][key])
    assert not torch.equal(original["pi.1.weight"][:2], results[0][3]["pi.1.weight"][:2])
    assert torch.equal(original["pi.1.weight"][2:], results[0][3]["pi.1.weight"][2:])
    assert not torch.equal(original["vf.weight"], results[0][3]["vf.weight"])
    for key, state in results[0][4]["state"].items():
        for name, value in state.items():
            assert torch.equal(value, results[1][4]["state"][key][name])


def test_mask_rng_resume_curriculum_boundaries_and_exnovo_forced_zero(variant):
    target = variant.target_availability
    config = {"probability": 1.0, "curriculum_probabilities": [1, .75, .5, .25, 0], "block_iterations": 10, "mask_seed": 123}
    assert [target.probability_for_iteration(config, i, "imitation") for i in (1, 10, 11, 20, 21, 31, 41, 50)] == [1, 1, .75, .75, .5, .25, 0, 0]
    assert all(target.probability_for_iteration(config, i, "ex_novo") == 0 for i in range(1, 52))
    original = target.EpisodeAvailability(.5, 123)
    for _ in range(13):
        original.begin_episode()
        for _ in range(5):
            original.record_transition()
    saved = json.loads(json.dumps(original.get_state()))
    expected = [original.begin_episode() for _ in range(50)]
    restored = target.EpisodeAvailability(.5, 123)
    restored.restore_rng(saved)
    assert restored.total_steps == 65
    assert restored.visible_steps == saved["visible_steps"]
    restored.reset_seed(999)  # RLlib's initialization reset cannot erase resume RNG.
    assert [restored.begin_episode() for _ in range(50)] == expected
    closed = target.EpisodeAvailability(1.0, 123, "ex_novo")
    assert not any(closed.begin_episode() for _ in range(100))


def test_contract_save_reload_phase_and_target_independence(variant, tmp_path):
    from ray.rllib.core.rl_module.rl_module import RLModule
    policy = module(variant)
    policy.set_training_phase("ex_novo")
    path = tmp_path / "actor"
    policy.save_to_path(path)
    restored = RLModule.from_checkpoint(path)
    assert restored.get_training_phase_state() == policy.get_training_phase_state()
    obs = torch.randn(4, 88)
    changed = obs.clone()
    changed[:, 35:39] += 10
    assert torch.equal(restored.forward_inference({"obs": changed})["action_dist_inputs"], policy.forward_inference({"obs": obs})["action_dist_inputs"])
    import os
    import subprocess
    import sys
    child = '''import json, sys, torch
from ray.rllib.core.rl_module.rl_module import RLModule
from unified_MLP_39.actor_transition import actor_state_digest
module = RLModule.from_checkpoint(sys.argv[1])
obs = torch.randn(4,88)
changed = obs.clone()
changed[:,35:39] += 100
assert torch.equal(module.forward_inference({"obs":obs})["action_dist_inputs"], module.forward_inference({"obs":changed})["action_dist_inputs"])
print(json.dumps({"phase":module.get_training_phase_state(), "digest":actor_state_digest(module.get_state())}))
'''
    environment = dict(os.environ)
    environment["PYTHONPATH"] = os.pathsep.join([str(variant.path.parent), str(build_variant.SOURCE.parent)])
    result = subprocess.run([sys.executable, "-c", child, str(path)], env=environment, check=True, capture_output=True, text=True, timeout=60)
    data = json.loads(result.stdout.strip().splitlines()[-1])
    assert data["phase"]["phase"] == "ex_novo"
    assert data["digest"] == variant.actor_transition.actor_state_digest(policy.get_state())


def test_runtime_curriculum_and_rng_metadata_roundtrip(variant, tmp_path):
    target = variant.target_availability
    env = SimpleNamespace(_target_availability=target.EpisodeAvailability(.75, 123))
    runner = SimpleNamespace(worker_index=1, env=SimpleNamespace(envs=[env]))
    group = SimpleNamespace(foreach_env_runner=lambda func, **kwargs: [func(runner)])
    algo = SimpleNamespace(env_runner_group=group, _unified_training_state={"reward_mode": "imitation", "last_completed_iteration": 10})
    args = SimpleNamespace(target_availability_probability=1, target_curriculum_probabilities=[1, .75, .5, .25, 0], target_curriculum_block_iterations=10, target_mask_seed=123, startup_timeout_s=10)
    target.initialize_runtime(algo, args, None)
    audit = target.before_iteration(algo, args)
    assert audit["probability"] == .75 and audit["next_iteration"] == 11
    env._target_availability.begin_episode()
    for _ in range(4):
        env._target_availability.record_transition()
    audit = target.after_iteration(algo, 11, args)
    assert audit["total_steps"] == 4
    assert audit["visible_steps"] == 4 * env._target_availability.visible
    assert algo._unified_training_state["target_availability"]["last_batch"] == audit
    saved = json.loads(json.dumps(algo._unified_training_state))
    expected_state = saved["target_availability"]["runner_rng_states"]["1"][0]
    env._target_availability = target.EpisodeAvailability(.75, 123)
    algo._unified_training_state = saved
    target.initialize_runtime(algo, args, tmp_path)
    target.before_iteration(algo, args)
    assert env._target_availability.get_state()["rng_state"] == expected_state["rng_state"]
    assert env._target_availability.total_steps == 4
    args.target_curriculum_probabilities = [1, 0]
    with pytest.raises(ValueError, match="identical schedule"):
        target.initialize_runtime(algo, args, tmp_path)


def test_effective_visibility_counters_follow_episode_and_resume(variant):
    target = variant.target_availability
    state = target.EpisodeAvailability(1)
    state.begin_episode()
    state.set_probability(0)
    for _ in range(7):
        state.record_transition()
    assert state.transition_counts() == {"total_steps": 7, "visible_steps": 7}
    state.begin_episode()
    for _ in range(3):
        state.record_transition()
    restored = target.EpisodeAvailability(0)
    restored.restore_rng(json.loads(json.dumps(state.get_state())))
    restored.reset_seed(123)
    restored.begin_episode()
    restored.record_transition()
    assert restored.transition_counts() == {"total_steps": 11, "visible_steps": 7}


def test_missing_requested_worker_rejects_partial_curriculum_broadcast(variant):
    target = variant.target_availability
    env = SimpleNamespace(_target_availability=target.EpisodeAvailability(0))
    runner = SimpleNamespace(worker_index=1, env=SimpleNamespace(envs=[env]))
    algo = SimpleNamespace(env_runner_group=SimpleNamespace(foreach_env_runner=lambda func, **kwargs: [func(runner)]),
                           _unified_training_state={"reward_mode": "imitation", "last_completed_iteration": 0})
    args = SimpleNamespace(num_env_runners=2, startup_timeout_s=1)
    target.initialize_runtime(algo, args, None)
    with pytest.raises(ValueError, match="missing target mask RNG workers"):
        target.before_iteration(algo, args)


def test_config_fields_roundtrip_and_rollout_probability_override(variant, tmp_path):
    args = SimpleNamespace(target_availability_probability=0, target_curriculum_probabilities=[1, .75, .5, .25, 0], target_curriculum_block_iterations=10, target_mask_seed=55)
    config = variant.training_config.resolve_values(args, {"reward_mode": "imitation"})
    flat, _ = variant.training_config.to_argparse_defaults(config)
    assert flat["target_availability_probability"] == 0
    assert variant.target_availability.env_options(args, evaluation=True)["target_availability_probability"] == 0
    assert variant.target_availability.env_options(args, evaluation=False)["target_availability_probability"] == 1
    checkpoint = tmp_path / "checkpoint"
    checkpoint.mkdir()
    (checkpoint / "unified_training_state.json").write_text(json.dumps({"last_completed_iteration": 40}))
    args.resume_from = str(checkpoint)
    assert variant.target_availability.env_options(args)["target_availability_probability"] == 0
