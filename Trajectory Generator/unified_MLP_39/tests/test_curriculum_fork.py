"""A new mask schedule must retain the checkpoint state and its local origin."""
import copy
import json
from types import SimpleNamespace

import pytest

from unified_MLP_39 import target_availability as target


def arguments(**changes):
    values = dict(target_availability_probability=0.75,
                  target_curriculum_probabilities=[0.75, 0.5, 0.25, 0.0],
                  target_curriculum_block_iterations=25,
                  target_curriculum_iteration_offset=90,
                  target_mask_seed=123, target_velocity_scale=1.0,
                  target_reference="left", resume_target_curriculum_fork=True)
    return SimpleNamespace(**(values | changes))


def checkpoint_state():
    args = arguments(target_availability_probability=1.0,
                     target_curriculum_probabilities=[],
                     target_curriculum_block_iterations=10,
                     target_curriculum_iteration_offset=0)
    mask = target.EpisodeAvailability(1.0, 123)
    for _ in range(5):
        mask.begin_episode()
    return dict(reward_mode="imitation", last_completed_iteration=90,
                other_training_state={"sentinel": [1, 2, 3]},
                target_availability=dict(configuration=target.configuration(args),
                                         last_completed_iteration=90,
                                         runner_rng_states={"1": [mask.get_state()]}))


def test_full_100_update_schedule_has_exact_boundaries():
    config = target.configuration(arguments())
    values = [target.probability_for_iteration(config, i, "imitation") for i in range(91, 191)]
    assert values == [0.75] * 25 + [0.5] * 25 + [0.25] * 25 + [0.0] * 25
    assert target.probability_for_iteration(config, 191, "imitation") == 0.0
    assert target.probability_for_iteration(config, 91, "ex_novo") == 0.0


def test_fork_retains_rng_and_unrelated_state_then_resumes_without_restarting():
    state = checkpoint_state()
    before = copy.deepcopy(state)
    algo = SimpleNamespace(_unified_training_state=state)
    args = arguments()
    target.initialize_runtime(algo, args, "CP90")
    assert state["other_training_state"] == before["other_training_state"]
    assert state["last_completed_iteration"] == 90
    assert algo._target_rng_restore == before["target_availability"]["runner_rng_states"]
    data = state["target_availability"]
    assert data["curriculum_forks"][0]["source_iteration"] == 90
    # Simulate a checkpoint written at the first stage boundary.
    state["last_completed_iteration"] = data["last_completed_iteration"] = 115
    saved = json.loads(json.dumps(state))
    algo._unified_training_state = saved
    target.initialize_runtime(algo, args, "CP115")
    assert saved == state
    assert target.probability_for_iteration(data["configuration"], 116, "imitation") == 0.5
    assert len(saved["target_availability"]["curriculum_forks"]) == 1


@pytest.mark.parametrize("changes,message", [
    ({"resume_target_curriculum_fork": False}, "identical schedule"),
    ({"target_mask_seed": 999}, "mask seed"),
    ({"target_velocity_scale": 10}, "scale"),
    ({"target_reference": "right_v26"}, "reference"),
    ({"target_curriculum_iteration_offset": 0}, "immediately after"),
    ({"target_curriculum_iteration_offset": 89}, "immediately after"),
])
def test_invalid_fork_does_not_mutate_source(changes, message):
    state = checkpoint_state()
    before = copy.deepcopy(state)
    with pytest.raises(ValueError, match=message):
        target.initialize_runtime(SimpleNamespace(_unified_training_state=state), arguments(**changes), "CP90")
    assert state == before


def test_new_schedule_requires_resume_and_imitation():
    state = checkpoint_state()
    with pytest.raises(ValueError, match="requires a full checkpoint"):
        target.initialize_runtime(SimpleNamespace(_unified_training_state=state), arguments(), None)
    state["reward_mode"] = "ex_novo"
    with pytest.raises(ValueError, match="imitation"):
        target.prepare_curriculum_resume(state, arguments(), "CP90")


def test_env_build_uses_same_schedule_as_runtime_and_resume(tmp_path):
    state = checkpoint_state()
    metadata = tmp_path / "unified_training_state.json"
    metadata.write_text(json.dumps(state))
    args = arguments(resume_from=str(tmp_path))
    assert target.env_options(args)["target_availability_probability"] == 0.75
    algo = SimpleNamespace(_unified_training_state=state)
    target.initialize_runtime(algo, args, str(tmp_path))
    for iteration, visibility in [(115, 0.5), (140, 0.25), (165, 0.0), (190, 0.0)]:
        state["last_completed_iteration"] = state["target_availability"]["last_completed_iteration"] = iteration
        metadata.write_text(json.dumps(state))
        assert target.env_options(args)["target_availability_probability"] == visibility
    # Evaluation uses its explicit probability, never the training schedule.
    assert target.env_options(args, evaluation=True)["target_availability_probability"] == 0.75


def test_legacy_default_schedule_remains_compatible():
    args = SimpleNamespace(target_availability_probability=1.0)
    assert "iteration_offset" not in target.configuration(args)
    state = {"last_completed_iteration": 0, "reward_mode": "imitation"}
    algo = SimpleNamespace(_unified_training_state=state)
    target.initialize_runtime(algo, args, None)
    target.initialize_runtime(algo, args, "legacy")


def test_block_restart_restores_rng_but_draws_new_mask_for_fully_hidden_phase():
    previous = target.EpisodeAvailability(1.0, 123)
    previous.begin_episode()
    previous.record_transition()
    saved = previous.get_state()
    restored = target.EpisodeAvailability(0.0, 123)
    restored.restore_rng(saved)
    restored.reset_seed(999)
    assert restored.begin_episode() is False
    restored.record_transition()
    assert restored.visible_steps == saved["visible_steps"]
    assert restored.total_steps == saved["total_steps"] + 1
