"""KL sidecar restore survives adaptation and rejects ambiguous checkpoint state."""
import hashlib
from types import SimpleNamespace

import pytest
import torch

from unified_MLP_39.learner_state_resume import (
    capture_on_learner, restore_training_state, save_training_state,
)


def learner_fixture():
    module = torch.nn.Linear(3, 2)
    optimizer = torch.optim.Adam(module.parameters(), lr=1e-4)
    scheduler = SimpleNamespace(use_schedule=False, get_current_value=lambda: 1e-4)
    entropy = SimpleNamespace(use_schedule=False, get_current_value=lambda: 0.0)
    return SimpleNamespace(
        module={'default_policy': module},
        _named_optimizers={'default_policy_default_optimizer': optimizer},
        _optimizer_lr_schedules={optimizer: scheduler}, _lr_schedulers={},
        entropy_coeff_schedulers_per_module={'default_policy': entropy},
        curr_kl_coeffs_per_module={'default_policy': torch.tensor(0.2)},
    )


def call_learner(learner, callback, **kwargs):
    return [callback(learner, **kwargs)]


def checkpoint_fixture(tmp_path):
    state_path = tmp_path / 'learner_group/learner/state.pkl'
    state_path.parent.mkdir(parents=True)
    state_path.write_bytes(b'opaque-checkpoint-state')
    return hashlib.sha256(state_path.read_bytes()).hexdigest()


def test_sidecar_restores_adapted_kl_exactly(tmp_path):
    source_hash = checkpoint_fixture(tmp_path)
    learner = learner_fixture()
    learner.curr_kl_coeffs_per_module['default_policy'].fill_(0.675)
    expected = capture_on_learner(learner)
    save_training_state(learner, tmp_path, call_learner)
    learner.curr_kl_coeffs_per_module['default_policy'].fill_(0.2)
    report = restore_training_state(learner, tmp_path, call_learner, source_hash)
    assert report['passed'] and report['source'] == 'checkpoint_sidecar'
    assert capture_on_learner(learner) == expected
    kl = report['learners'][0]['kl_coefficient']['default_policy']
    assert kl['restored'] == expected['modules']['default_policy']['kl_coeff']
    assert kl['before'] != kl['restored']


def test_sidecar_must_match_checkpoint_hash(tmp_path):
    checkpoint_fixture(tmp_path)
    learner = learner_fixture()
    save_training_state(learner, tmp_path, call_learner)
    with pytest.raises(RuntimeError, match='does not match checkpoint'):
        restore_training_state(learner, tmp_path, call_learner, 'different-state-hash')


def test_unknown_legacy_checkpoint_fails_closed(tmp_path):
    with pytest.raises(RuntimeError, match='no verified PPO adaptive KL state'):
        restore_training_state(learner_fixture(), tmp_path, call_learner, 'unknown')


def test_changed_schedule_is_not_silently_reinitialized():
    learner = learner_fixture()
    learner.entropy_coeff_schedulers_per_module['default_policy'].use_schedule = True
    with pytest.raises(RuntimeError, match='constant entropy'):
        capture_on_learner(learner)
