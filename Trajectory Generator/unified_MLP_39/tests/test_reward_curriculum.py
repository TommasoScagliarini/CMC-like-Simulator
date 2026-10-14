"""Reward selection and episode-level guidance are independent only by opt-in."""
import copy
from types import SimpleNamespace

import gymnasium as gym
import numpy as np
import pytest
import torch

from unified_MLP_39 import target_availability as target, reward_resume
from unified_MLP_39.asymmetric_rl_module import AsymmetricActorCriticTorchRLModule
from unified_MLP_39.reward_function import RewardConfig


@pytest.mark.parametrize("objective", ["imitation", "ex_novo"])
def test_episode_masks_are_identical_between_objectives_and_resume(objective):
    left = target.EpisodeAvailability(.5, 123, "imitation", "episode")
    right = target.EpisodeAvailability(.5, 123, objective, "episode")
    for _ in range(12):
        assert left.begin_episode() == right.begin_episode()
    saved = copy.deepcopy(right.get_state())
    restored = target.EpisodeAvailability(.5, 123, objective, "episode")
    restored.restore_rng(saved)
    restored.reset_seed(999)
    assert [restored.begin_episode() for _ in range(20)] == [right.begin_episode() for _ in range(20)]
    restored.set_probability(0)
    assert not any(restored.begin_episode() for _ in range(20))


def test_old_exnovo_is_still_always_hidden_and_new_schedule_is_explicit():
    old = target.EpisodeAvailability(1, 123, "ex_novo")
    assert not old.begin_episode()
    cfg = target.configuration(SimpleNamespace(target_mask_mode="episode", target_curriculum_probabilities=[.75,.5,.25,0], target_curriculum_block_iterations=5))
    assert [target.probability_for_iteration(cfg, i, "ex_novo") for i in [1,6,11,16]] == [.75,.5,.25,0]
    cfg.pop("mask_mode")
    assert target.probability_for_iteration(cfg, 1, "ex_novo") == 0


@pytest.mark.parametrize("phase", ["critic_warmup", "imitation", "ex_novo"])
def test_module_keeps_collector_mask_and_serializes_mode(phase, tmp_path):
    module = AsymmetricActorCriticTorchRLModule(
        observation_space=gym.spaces.Box(-np.inf, np.inf, (88,), np.float32),
        action_space=gym.spaces.Box(-1.,1., (2,), np.float32),
        model_config={"n_actor":39,"fcnet_hiddens":[256,256],"target_mask_mode":"episode"})
    module.set_training_phase(phase, freeze_logstd=True)
    obs = torch.randn(4,88)
    obs[2:, list(target.PRESCRIBED_INDICES)] = 0
    torch.testing.assert_close(module._masked_observation({"obs":obs}), obs)
    path = tmp_path/'module'
    module.save_to_path(path)
    restored = AsymmetricActorCriticTorchRLModule.from_checkpoint(path)
    assert restored._target_mask_mode == "episode"
    torch.testing.assert_close(restored._masked_observation({"obs":obs}), obs)
    torch.testing.assert_close(module._policy_logits({"obs":obs}), restored._policy_logits({"obs":obs}))


def test_legacy_reward_metadata_migrates_only_inactive_default():
    new = RewardConfig().to_dict()
    old = dict(new); old.pop("morphology_termination_mode")
    assert reward_resume._canonical_reward(old) == new


def test_exnovo_schedule_can_fork_without_changing_mask_mode():
    args=SimpleNamespace(target_mask_mode="episode",target_availability_probability=.75)
    state={"reward_mode":"ex_novo","last_completed_iteration":6,
           "target_availability":{"configuration":target.configuration(args),"last_completed_iteration":6}}
    args.resume_target_curriculum_fork=True
    args.target_curriculum_iteration_offset=6
    args.target_curriculum_probabilities=[.5]
    audit=target.prepare_curriculum_resume(state,args,"checkpoint")
    assert audit["mask_rng_retained"]
    args.target_mask_mode="stage_default"
    with pytest.raises(ValueError):
        target.prepare_curriculum_resume(state,args,"checkpoint")


@pytest.mark.parametrize('objective', ['imitation', 'ex_novo'])
@pytest.mark.parametrize('termination_mode', ['objective', 'ex_novo'])
def test_morphology_failure_termination_can_be_matched_across_objectives(monkeypatch, objective, termination_mode):
    from pathlib import Path
    import yaml
    from unified_MLP_39.reward_function import RewardShapingWrapper

    class Env(gym.Env):
        observation_space = gym.spaces.Box(-np.inf, np.inf, (88,), np.float32)
        action_space = gym.spaces.Box(-1., 1., (2,), np.float32)

        def step(self, action):
            return np.zeros(88, np.float32), 0., False, False, {'reward_terms': {
                'sound_imitation_loss': .1, 'served_imitation_loss': .1}}

    data = yaml.safe_load((Path(__file__).resolve().parents[1]/'configs/ex_novo.yaml').read_text())['reward']
    cfg = RewardConfig.from_mapping(dict(data, reward_mode=objective, morphology_termination_mode=termination_mode))
    wrapper = RewardShapingWrapper(Env(), cfg)
    monkeypatch.setattr(wrapper, '_task_reward_terms', lambda info: {'morphology_causal_failed_closed': 1.})
    _, _, terminated, truncated, info = wrapper.step(np.zeros(2, np.float32))
    expected = objective == 'ex_novo' or termination_mode == 'ex_novo'
    assert terminated == expected
    assert not truncated
    if expected:
        assert info['end_reason'] == 'morphology_causal_contract_failure'
    if objective == 'imitation':
        assert info['reward_components']['morphology_term'] == 0
