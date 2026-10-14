"""A reward fork preserves learned state and cannot silently change objectives."""
import copy
import json
import pickle
from types import SimpleNamespace

import numpy as np
import pytest
from ray.rllib.utils.metrics.metrics_logger import MetricsLogger

from unified_MLP_39.reward_function import RewardConfig
from unified_MLP_39.reward_resume import (
    prepare_resume_state, audit_restored_parameters, audit_runner_reward,
    clear_old_return_metrics,
)
from unified_MLP_39.training_runtime import critic_digest


def fixture():
    reward = RewardConfig(reward_mode="imitation", grf_penetration_weight=1.0).to_dict()
    return {"reward_mode": "imitation", "reward_config": reward,
            "last_completed_iteration": 72, "critic_warmup_completed": True,
            "target_availability": {"probability": 1.0}}, reward


def test_explicit_fork_changes_only_reward_and_preserves_source_and_phase(tmp_path):
    state, old = fixture()
    before = copy.deepcopy(state)
    source = tmp_path / "source/checkpoint"
    target = {**old, "grf_penetration_weight": 2.0}
    updated, audit = prepare_resume_state(state, target, allow_override=True,
                                         source=source, output=tmp_path / "branch")
    assert state == before
    assert updated["reward_config"] == target
    assert updated["critic_warmup_completed"] is True
    assert updated["last_completed_iteration"] == 72
    assert updated["target_availability"] == state["target_availability"]
    assert audit["changed_reward_fields"] == {"grf_penetration_weight": {"before": 1.0, "after": 2.0}}
    assert not audit["optimizer_reinitialized"]
    # Checkpoint JSON after the fork resumes normally and retains provenance.
    saved = json.loads(json.dumps(updated))
    resumed, second_audit = prepare_resume_state(saved, target, allow_override=False,
                    source=tmp_path / "branch/checkpoint", output=tmp_path / "branch")
    assert resumed == saved and second_audit is None


@pytest.mark.parametrize("change,flag,message", [
    ({"grf_penetration_weight": 2}, False, "retain its reward"),
    ({"grf_penetration_weight": 2, "command_rate_weight": 99}, True, "only grf_penetration"),
    ({"reward_mode": "ex_novo"}, True, "reward mode"),
    ({"grf_penetration_weight": float("nan")}, True, "finite"),
    ({"grf_penetration_weight": -1}, True, "nonnegative"),
])
def test_unrequested_changes_rejected(tmp_path, change, flag, message):
    state, old = fixture()
    with pytest.raises(ValueError, match=message):
        prepare_resume_state(state, {**old, **change}, allow_override=flag,
                             source=tmp_path / "source", output=tmp_path / "branch")


def test_cannot_mix_old_run_history_or_overwrite_source(tmp_path):
    state, old = fixture()
    target = {**old, "grf_penetration_weight": 2}
    with pytest.raises(ValueError, match="separate output"):
        prepare_resume_state(state, target, allow_override=True,
            source=tmp_path / "run/milestone/checkpoint", output=tmp_path / "run")
    output = tmp_path / "branch"
    output.mkdir()
    (output / "train_iterations.jsonl").write_text('{"iteration":73}\n')
    with pytest.raises(ValueError, match="existing training history"):
        prepare_resume_state(state, target, allow_override=True,
                             source=tmp_path / "other", output=output)


def test_parameter_audit_detects_critic_change(tmp_path):
    state = {"pi.1.weight": np.ones((4, 2), dtype=np.float32),
             "pi.1.bias": np.zeros(4, dtype=np.float32),
             "vf.weight": np.ones((1, 2), dtype=np.float32)}
    path = tmp_path / "learner_group/learner/rl_module/default_policy/module_state.pkl"
    path.parent.mkdir(parents=True)
    path.write_bytes(pickle.dumps(state))
    assert audit_restored_parameters(tmp_path, state, critic_digest)["actor_exact"]
    state["vf.weight"][0, 0] += 1
    with pytest.raises(RuntimeError, match="changed checkpoint parameters"):
        audit_restored_parameters(tmp_path, state, critic_digest)


def test_live_wrapper_reward_audit_rejects_stale_worker():
    reward = RewardConfig(reward_mode="imitation", grf_penetration_weight=2)
    base = SimpleNamespace(target_reference="left", target_velocity_scale=1,
        _target_availability=SimpleNamespace(probability=1),
        env_cfg=SimpleNamespace(binary_phase_actor_fsm_version="v3"))
    env = SimpleNamespace(reward_config=reward, unwrapped=base)
    # RLlib DictInfoToList exposes the vector through .unwrapped, not .envs.
    runner = SimpleNamespace(worker_index=1, env=SimpleNamespace(
        unwrapped=SimpleNamespace(envs=[env])))
    assert audit_runner_reward(runner, expected=reward.to_dict())["envs"][0]["penetration_weight"] == 2
    with pytest.raises(RuntimeError, match="Live EnvRunner reward"):
        audit_runner_reward(runner, expected=RewardConfig().to_dict())


def test_return_summaries_cleared_but_lifetime_counters_preserved():
    metrics = MetricsLogger()
    metrics.log_value(("env_runners", "episode_return_mean"), 200.0)
    metrics.log_value(("env_runners", "num_env_steps_sampled_lifetime"), 294912, reduce="sum")
    metrics.log_value(("learners", "default_policy", "vf_loss"), 3.0)
    removed = clear_old_return_metrics(SimpleNamespace(metrics=metrics))
    assert removed == ["env_runners--episode_return_mean"]
    assert ("env_runners", "episode_return_mean") not in metrics
    assert metrics.peek(("env_runners", "num_env_steps_sampled_lifetime")) == 294912
    assert metrics.peek(("learners", "default_policy", "vf_loss")) == 3.0
