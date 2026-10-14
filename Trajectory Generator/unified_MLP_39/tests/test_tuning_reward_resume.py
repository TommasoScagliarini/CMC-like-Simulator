"""Reward tuning forks retain saved actors and rewarm only the visible critic."""
import copy
from types import SimpleNamespace

import gymnasium as gym
import numpy as np
import pytest
import torch

from unified_MLP_39 import actor_transition, training_runtime, training_config
from unified_MLP_39.asymmetric_rl_module import AsymmetricActorCriticTorchRLModule
from unified_MLP_39.reward_function import RewardConfig
from unified_MLP_39.reward_resume import TUNING_REWARD_KEYS, prepare_resume_state


def state_fixture():
    reward = RewardConfig(reward_mode="imitation").to_dict()
    return {"reward_mode": "imitation", "reward_config": reward,
            "last_completed_iteration": 80, "critic_warmup_completed": True}, reward


def fork(state, reward, tmp_path, **kwargs):
    return prepare_resume_state(state, reward, allow_override=False,
        allow_tuning_fork=True, critic_rewarm_iterations=1,
        source=tmp_path / "source/checkpoint_best", output=tmp_path / "branch", **kwargs)


def test_unchanged_control_rewarms_and_saved_continuation_never_repeats(tmp_path):
    state, reward = state_fixture()
    updated, audit = fork(state, reward, tmp_path)
    assert "imitation_critic_rewarm_remaining" not in state
    assert updated["imitation_critic_rewarm_remaining"] == 1
    assert audit["control_reward_unchanged"] and audit["changed_reward_fields"] == {}
    assert training_runtime.training_phase(updated) == "imitation_critic_rewarm"
    for remaining in (1, 0):
        updated["imitation_critic_rewarm_remaining"] = remaining
        resumed, next_audit = prepare_resume_state(updated, reward, allow_override=False,
            allow_tuning_fork=True, critic_rewarm_iterations=1,
            source=tmp_path / "branch/checkpoint_last", output=tmp_path / "branch")
        assert resumed == updated and next_audit is None
        assert training_runtime.training_phase(resumed) == ("imitation_critic_rewarm" if remaining else "imitation")


def test_tuning_only_changes_allowed_keys_and_preserves_historical_defaults(tmp_path):
    state, reward = state_fixture()
    target = {**reward, "ankle_positive_torque_weight": .05}
    updated, audit = fork(state, target, tmp_path)
    assert updated["reward_config"] == target
    assert list(audit["changed_reward_fields"]) == ["ankle_positive_torque_weight"]
    assert not audit["actor_reinitialized"] and not audit["optimizer_reinitialized"]
    historical = copy.deepcopy(state)
    for key in TUNING_REWARD_KEYS:
        historical["reward_config"].pop(key)
    normal, no_fork = prepare_resume_state(historical, reward, allow_override=False,
        source=tmp_path / "old", output=tmp_path / "old")
    assert no_fork is None and normal["reward_config"] == reward
    # A historical penetration-only fork must still work after schema extension.
    penetration, audit = prepare_resume_state(historical,
        {**reward, "grf_penetration_weight": 2.}, allow_override=True,
        source=tmp_path / "old", output=tmp_path / "penetration")
    assert list(audit["changed_reward_fields"]) == ["grf_penetration_weight"]
    assert "imitation_critic_rewarm_remaining" not in penetration


@pytest.mark.parametrize("change", [
    {"grf_penetration_weight": 2.}, {"imitation_weight": 3.},
    {"ankle_positive_torque_weight": -.1}, {"ankle_positive_torque_weight": float("nan")},
    {"ankle_positive_torque_scale_nm": 0.}, {"kinematic_jerk_scale_rad_s3": True},
])
def test_disallowed_reward_forks_rejected(tmp_path, change):
    state, reward = state_fixture()
    with pytest.raises(ValueError):
        fork(state, {**reward, **change}, tmp_path)


def test_new_rewards_still_require_explicit_fork_and_preserve_source(tmp_path):
    state, reward = state_fixture()
    with pytest.raises(ValueError, match="retain its reward"):
        prepare_resume_state(state, {**reward, "knee_torque_rate_weight": .1},
            allow_override=False, source=tmp_path / "source", output=tmp_path / "branch")
    with pytest.raises(ValueError, match="separate output"):
        prepare_resume_state(state, reward, allow_override=False,
            allow_tuning_fork=True, critic_rewarm_iterations=1,
            source=tmp_path / "source/checkpoint", output=tmp_path / "source")
    with pytest.raises(ValueError, match="mutually exclusive"):
        prepare_resume_state(state, reward, allow_override=True, allow_tuning_fork=True,
            source=tmp_path / "source", output=tmp_path / "branch")


def policy_fixture():
    torch.manual_seed(123)
    return AsymmetricActorCriticTorchRLModule(
        observation_space=gym.spaces.Box(-np.inf, np.inf, (88,), np.float32),
        action_space=gym.spaces.Box(-1., 1., (2,), np.float32),
        model_config={"n_actor": 39, "fcnet_hiddens": [16, 16], "fcnet_activation": "tanh"})


def test_visible_rewarm_preserves_actor_adam_and_checkpoints_then_trains_imitation():
    policy = policy_fixture()
    optimizer = torch.optim.Adam(policy.parameters(), lr=1e-4)
    learner = SimpleNamespace(module={"default_policy": policy}, _named_optimizers={"default": optimizer})
    obs = torch.randn(8, 88)

    def update():
        optimizer.zero_grad(set_to_none=True)
        logits = policy.forward_train({"obs": obs})["action_dist_inputs"]
        loss = (logits - 1.).square().mean() + (policy.compute_values({"obs": obs}) - 2.).square().mean()
        loss.backward()
        optimizer.step()

    update()  # Populate meaningful Adam moments before freezing.
    actor = actor_transition.actor_state_digest(policy.get_state())
    critic = training_runtime.critic_digest(policy.get_state())
    moments = training_runtime._actor_optimizer_on_learner(learner)
    mean = policy.forward_inference({"obs": obs})["action_dist_inputs"].detach().clone()
    policy.set_training_phase("imitation_critic_rewarm")
    assert policy._masked_observation({"obs": obs}) is obs
    assert policy.get_training_phase_state()["freeze_actor"]
    changed = obs.clone()
    changed[:, [0, 1, 35, 36, 37, 38]] += 50.
    assert not torch.equal(policy.forward_inference({"obs": obs})["action_dist_inputs"],
                           policy.forward_inference({"obs": changed})["action_dist_inputs"])
    for _ in range(2):
        update()
        assert actor_transition.actor_state_digest(policy.get_state()) == actor
        assert training_runtime._actor_optimizer_on_learner(learner) == moments
    assert training_runtime.critic_digest(policy.get_state()) != critic
    torch.testing.assert_close(policy.forward_inference({"obs": obs})["action_dist_inputs"], mean, rtol=0, atol=0)
    restored = policy_fixture()
    restored.set_state(policy.get_state())
    assert restored.get_training_phase_state()["phase"] == "imitation_critic_rewarm"
    assert restored._masked_observation({"obs": obs}) is obs
    policy.set_training_phase("imitation")
    update()
    assert actor_transition.actor_state_digest(policy.get_state()) != actor
    assert training_runtime._actor_optimizer_on_learner(learner) != moments


def test_rewarm_runtime_countdown_and_adam_audit(monkeypatch, tmp_path):
    state, reward = state_fixture()
    state, _ = fork(state, reward, tmp_path)
    algo = SimpleNamespace(_unified_training_state=state)
    parameters = {"pi.1.weight": np.ones((4, 2)), "pi.1.bias": np.zeros(4),
                  "vf.weight": np.ones((1, 2))}
    phases = []
    monkeypatch.setattr(training_runtime, "set_phase", lambda algo, phase, driver, args: phases.append(phase))
    monkeypatch.setattr(training_runtime.target_availability, "before_iteration", lambda *a: {})
    monkeypatch.setattr(training_runtime.target_availability, "after_iteration", lambda *a: {})
    driver = SimpleNamespace(_learner_module_state=lambda algo: copy.deepcopy(parameters),
        _reapply_optimizer_learning_rate=lambda *a: None,
        _learner_call_results=lambda *a: [{"default": {"state_digest": "fixed"}}])
    args = SimpleNamespace(lr=1e-4)
    before = training_runtime.before_iteration(algo, args, driver)
    parameters["vf.weight"] += 1
    audit = training_runtime.after_iteration(algo, 81, before, args, driver)
    assert audit["actor_optimizer_exact"] and audit["actor_exact"] and audit["critic_changed"]
    assert state["imitation_critic_rewarm_remaining"] == 0
    assert state["imitation_critic_rewarm_completed_iterations"] == 1
    assert state["last_completed_iteration"] == 81
    assert phases == ["imitation_critic_rewarm", "imitation"]
    assert training_runtime.before_iteration(algo, args, driver)["phase"] == "imitation"


def test_tuning_flags_load_from_run_yaml():
    defaults, _ = training_config.to_argparse_defaults({"run": {
        "resume_tuning_reward_fork": True, "imitation_critic_rewarm_iterations": 1}})
    assert defaults["resume_tuning_reward_fork"] is True
    assert defaults["imitation_critic_rewarm_iterations"] == 1
