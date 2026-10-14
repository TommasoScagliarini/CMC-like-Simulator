"""Restore Adam without converting serialized parameter IDs into Torch tensors.

RLlib2.55 converts the entire optimizer state to tensors, including the integer
IDs in param_groups['params']. PyTorch then leaves moment state under orphaned
integer keys instead of attaching it to live Parameters. Restore the original
checkpoint payload with only moment tensors converted, and audit identity.
"""
from __future__ import annotations

import copy
import hashlib
import json
import pickle
from pathlib import Path

import numpy as np
import torch


def _array(value):
    if isinstance(value, torch.Tensor):
        return value.detach().cpu().numpy()
    return np.asarray(value)


def optimizer_digest(state):
    """Fingerprint groups, parameter IDs, moment arrays and step values."""
    h = hashlib.sha256()
    h.update(json.dumps(state['param_groups'], sort_keys=True).encode())
    for parameter_id, values in sorted(state['state'].items()):
        h.update(str(parameter_id).encode())
        for key, value in sorted(values.items()):
            h.update(key.encode())
            if key == 'step':
                h.update(repr(float(value)).encode())
            else:
                a = _array(value)
                h.update(str(a.dtype).encode())
                h.update(str(a.shape).encode())
                h.update(a.tobytes())
    return h.hexdigest()


def restore_on_learner(learner, *, source_optimizers):
    parameters = {f'{module_id}.{name}': p
                  for module_id in learner.module.keys()
                  for name, p in learner.module[module_id].named_parameters()}
    before = {name: p.detach().cpu().clone() for name, p in parameters.items()}
    reports = []
    for name, source in source_optimizers.items():
        if name not in learner._named_optimizers:
            raise RuntimeError(f'Missing optimizer on resume: {name}')
        optimizer = learner._named_optimizers[name]
        if not isinstance(optimizer, torch.optim.Adam):
            raise RuntimeError(f'Unified resume requires Adam, got {type(optimizer)}')
        original = source['state']
        expected = optimizer_digest(original)
        live_parameters = [p for g in optimizer.param_groups for p in g['params']]
        live_ids = {id(p) for p in live_parameters}
        orphan_before = sum(id(p) not in live_ids for p in optimizer.state)
        converted = copy.deepcopy(original)
        # Keep param_groups['params'] and state dictionary keys as plain IDs.
        for parameter_id, values in converted['state'].items():
            if not isinstance(parameter_id, int):
                raise RuntimeError('Checkpoint optimizer parameter IDs must be integers')
            for key, value in values.items():
                if isinstance(value, np.ndarray) or key == 'step':
                    values[key] = torch.as_tensor(value)
        optimizer.load_state_dict(converted)
        group_ids = [i for g in original['param_groups'] for i in g['params']]
        live_parameters = [p for g in optimizer.param_groups for p in g['params']]
        if len(group_ids) != len(live_parameters):
            raise RuntimeError('Optimizer parameter topology changed on resume')
        expected_parameters = [p for i, p in zip(group_ids, live_parameters)
                               if i in original['state']]
        if {id(p) for p in optimizer.state} != {id(p) for p in expected_parameters}:
            raise RuntimeError('Restored Adam moments are not attached to live parameters')
        actual = optimizer_digest(optimizer.state_dict())
        if actual != expected:
            raise RuntimeError(f'Adam checkpoint contents changed on restore: {name}')
        reports.append({
            'name': name, 'module_id': source['module_id'],
            'state_count': len(original['state']),
            'linked_state_count': len(expected_parameters),
            'orphan_state_count_before': orphan_before,
            'steps': sorted({float(v['step']) for v in optimizer.state.values()}),
            'state_exact': True, 'source_digest': expected, 'restored_digest': actual,
        })
    if set(source_optimizers) != set(learner._named_optimizers):
        raise RuntimeError('Checkpoint and runtime optimizer names disagree')
    for name, parameter in parameters.items():
        if name not in before or not torch.equal(parameter.detach().cpu(), before[name]):
            raise RuntimeError('Optimizer restoration changed an actor/critic parameter')
    return {'passed': True, 'parameters_unchanged': True, 'optimizers': reports}


def restore_and_audit(algo, source_checkpoint, call_learners):
    source_checkpoint = Path(source_checkpoint).resolve()
    path = source_checkpoint / 'learner_group' / 'learner' / 'state.pkl'
    raw = path.read_bytes()
    state = pickle.loads(raw)
    source = state.get('optimizer')
    if not isinstance(source, dict) or not source:
        raise RuntimeError('Full resume checkpoint is missing optimizer state')
    reports = call_learners(algo, restore_on_learner, source_optimizers=source)
    if not reports or not all(r['passed'] for r in reports):
        raise RuntimeError('Adam resume audit did not cover the Learner')
    from unified_MLP_39.learner_state_resume import restore_training_state
    state_sha256 = hashlib.sha256(raw).hexdigest()
    ppo_state = restore_training_state(algo, source_checkpoint, call_learners, state_sha256)
    return {'passed': True, 'source_checkpoint': str(source_checkpoint),
            'source_state_sha256': state_sha256, 'learners': reports,
            'ppo_training_state': ppo_state}
