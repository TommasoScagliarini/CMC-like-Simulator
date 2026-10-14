"""Regression for Adam state becoming orphaned during RLlib checkpoint restore."""
import copy
from types import SimpleNamespace

import pytest
import torch
from ray.rllib.utils.numpy import convert_to_numpy
from ray.rllib.utils.torch_utils import convert_to_torch_tensor

from unified_MLP_39.optimizer_resume import optimizer_digest, restore_on_learner


def fixture_state():
    torch.manual_seed(123)
    module = torch.nn.Linear(3, 2)
    optimizer = torch.optim.Adam(module.parameters(), lr=1e-4)
    for index in range(3):
        for parameter in module.parameters():
            parameter.grad = torch.full_like(parameter, 0.1 * (index + 1))
        optimizer.step()
    payload = {'default_policy_default_optimizer': {
        'module_id': 'default_policy',
        'state': convert_to_numpy(optimizer.state_dict()),
    }}
    return module, optimizer, payload


def test_recovered_moments_match_uninterrupted_next_adam_update():
    original_module, original_optimizer, payload = fixture_state()
    module = copy.deepcopy(original_module)
    optimizer = torch.optim.Adam(module.parameters(), lr=1e-4)
    name = next(iter(payload))
    # Reproduce the exact RLlib 2.55 restore conversion. Integer IDs become
    # scalar tensors, leaving the original moments under orphaned integer keys.
    optimizer.load_state_dict(convert_to_torch_tensor(payload[name]['state']))
    assert len(optimizer.state) == 2
    assert all(parameter not in optimizer.state for parameter in module.parameters())
    learner = SimpleNamespace(module={'default_policy': module},
                              _named_optimizers={name: optimizer})
    before = [parameter.detach().clone() for parameter in module.parameters()]
    report = restore_on_learner(learner, source_optimizers=payload)
    assert report['passed'] and report['parameters_unchanged']
    assert report['optimizers'][0]['orphan_state_count_before'] == 2
    assert report['optimizers'][0]['linked_state_count'] == 2
    assert report['optimizers'][0]['steps'] == [3.0]
    assert all(torch.equal(left, right) for left, right in zip(before, module.parameters()))
    assert optimizer_digest(optimizer.state_dict()) == optimizer_digest(original_optimizer.state_dict())
    # The next deterministic update must be identical to uninterrupted Adam,
    # which checks that moments are used, not merely present in serialization.
    for index, (left, right) in enumerate(zip(original_module.parameters(), module.parameters())):
        gradient = torch.arange(left.numel(), dtype=left.dtype).reshape(left.shape) * 0.07 + index
        left.grad = gradient.clone()
        right.grad = gradient.clone()
    original_optimizer.step()
    optimizer.step()
    assert all(torch.equal(left, right) for left, right in zip(original_module.parameters(), module.parameters()))
    assert optimizer_digest(optimizer.state_dict()) == optimizer_digest(original_optimizer.state_dict())
    assert {float(state['step']) for state in optimizer.state.values()} == {4.0}


def test_missing_runtime_optimizer_fails_closed():
    module, _, payload = fixture_state()
    learner = SimpleNamespace(module={'default_policy': module}, _named_optimizers={})
    with pytest.raises(RuntimeError, match='Missing optimizer'):
        restore_on_learner(learner, source_optimizers=payload)


def test_restore_is_idempotent_with_working_torch_state_mapping():
    module, optimizer, payload = fixture_state()
    learner = SimpleNamespace(module={'default_policy': module},
                              _named_optimizers={next(iter(payload)): optimizer})
    report = restore_on_learner(learner, source_optimizers=payload)
    assert report['optimizers'][0]['orphan_state_count_before'] == 0
    assert report['optimizers'][0]['state_exact']
