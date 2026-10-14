"""Preserve PPO's adaptive KL state, omitted from RLlib 2.55 checkpoints.

This variant currently uses constant LR and entropy. Reject an unhandled
scheduler instead of silently resuming it at a different point.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path

import torch

SIDECAR = 'unified_learner_state.json'
# Verified from the historical iteration-40 row. The empty metrics_logger
# values in this exact checkpoint cannot recover the terminal KL coefficient.
# Never infer a value for any other legacy checkpoint.
LEGACY_40_STATE_SHA256 = '0ea41cc4c5f67726acda36e85edd622e67275a0e7a47a1a527c0cbc374ca7fc5'
LEGACY_40_KL = 0.20000000298023224


def capture_on_learner(learner):
    if getattr(learner, '_lr_schedulers', None):
        raise RuntimeError('Unified resume does not support Torch LR schedulers')
    modules = {}
    for module_id in learner.module.keys():
        entropy = learner.entropy_coeff_schedulers_per_module[module_id]
        if entropy.use_schedule:
            raise RuntimeError('Unified resume requires constant entropy coefficient')
        modules[module_id] = {
            'kl_coeff': float(learner.curr_kl_coeffs_per_module[module_id]),
            'entropy_coeff': float(entropy.get_current_value()),
        }
    learning_rates = {}
    for name, optimizer in learner._named_optimizers.items():
        schedule = learner._optimizer_lr_schedules.get(optimizer)
        if schedule is not None and schedule.use_schedule:
            raise RuntimeError('Unified resume requires constant learning rate')
        learning_rates[name] = {
            'schedule_value': None if schedule is None else float(schedule.get_current_value()),
            'parameter_group_values': [float(group['lr']) for group in optimizer.param_groups],
        }
    return {'modules': modules, 'learning_rates': learning_rates,
            'constant_schedules': True}


def restore_on_learner(learner, *, source):
    before = capture_on_learner(learner)
    if set(source['modules']) != set(before['modules']):
        raise RuntimeError('PPO learner-state module names changed')
    # LR is restored by Adam; both configured schedules are constant. A
    # discrepancy here means the recipe differs, and must not be guessed away.
    if source['learning_rates'] != before['learning_rates']:
        raise RuntimeError('PPO checkpoint and runtime learning-rate state differ')
    reports = {}
    for module_id, expected in source['modules'].items():
        if expected['entropy_coeff'] != before['modules'][module_id]['entropy_coeff']:
            raise RuntimeError('PPO checkpoint and runtime entropy coefficient differ')
        value = float(expected['kl_coeff'])
        if not math.isfinite(value) or value < 0:
            raise RuntimeError('Invalid checkpoint KL coefficient')
        tensor = learner.curr_kl_coeffs_per_module[module_id]
        with torch.no_grad():
            tensor.copy_(torch.as_tensor(value, dtype=tensor.dtype, device=tensor.device))
        restored = float(tensor)
        if restored != value:
            raise RuntimeError('KL coefficient changed precision during restore')
        reports[module_id] = {'source': value, 'before': before['modules'][module_id]['kl_coeff'],
                              'restored': restored, 'passed': True}
    if capture_on_learner(learner) != source:
        raise RuntimeError('PPO auxiliary learner state did not restore exactly')
    return {'passed': True, 'kl_coefficient': reports, 'constant_schedules_verified': True}


def save_training_state(algo, checkpoint, call_learners):
    checkpoint = Path(checkpoint)
    reports = call_learners(algo, capture_on_learner)
    if len(reports) != 1:
        raise RuntimeError('Unified learner-state sidecar requires one Learner')
    state_sha = hashlib.sha256((checkpoint / 'learner_group/learner/state.pkl').read_bytes()).hexdigest()
    payload = {'schema_version': 1, 'learner_state_sha256': state_sha,
               'learners': reports}
    temporary = checkpoint / (SIDECAR + '.tmp')
    temporary.write_text(json.dumps(payload, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    os.replace(temporary, checkpoint / SIDECAR)


def restore_training_state(algo, checkpoint, call_learners, learner_state_sha256):
    path = Path(checkpoint) / SIDECAR
    if path.exists():
        payload = json.loads(path.read_text(encoding='utf-8'))
        if payload.get('schema_version') != 1 or payload.get('learner_state_sha256') != learner_state_sha256:
            raise RuntimeError('PPO learner-state sidecar does not match checkpoint')
        source_kind = 'checkpoint_sidecar'
        states = payload['learners']
    elif learner_state_sha256 == LEGACY_40_STATE_SHA256:
        states = call_learners(algo, capture_on_learner)
        if len(states) != 1 or set(states[0]['modules']) != {'default_policy'}:
            raise RuntimeError('Historical iteration-40 Learner topology changed')
        if states[0]['modules']['default_policy']['entropy_coeff'] != 0.0:
            raise RuntimeError('Historical iteration-40 entropy configuration changed')
        for item in states[0]['learning_rates'].values():
            if item['schedule_value'] != 1e-4 or item['parameter_group_values'] != [1e-4]:
                raise RuntimeError('Historical iteration-40 LR configuration changed')
        states[0]['modules']['default_policy']['kl_coeff'] = LEGACY_40_KL
        source_kind = 'verified_historical_iteration40_hash_and_history'
    else:
        raise RuntimeError('Legacy checkpoint has no verified PPO adaptive KL state')
    if len(states) != 1:
        raise RuntimeError('Unified learner-state restore requires one Learner')
    reports = call_learners(algo, restore_on_learner, source=states[0])
    if len(reports) != 1 or not reports[0]['passed']:
        raise RuntimeError('PPO learner-state audit failed')
    return {'passed': True, 'source': source_kind, 'learners': reports,
            'sidecar': str(path) if path.exists() else None}
