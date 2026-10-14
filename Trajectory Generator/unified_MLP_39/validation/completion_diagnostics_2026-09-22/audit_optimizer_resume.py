"""Offline real-checkpoint restore audit: no Ray initialization or simulation."""
from __future__ import annotations

import hashlib
import json
import pickle
from pathlib import Path
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PACKAGE_ROOT.parent))
from unified_MLP_39._bootstrap import ensure_sim_paths
ensure_sim_paths()
from baseline_MLP import win_runtime  # noqa: F401
from unified_MLP_39.optimizer_resume import restore_on_learner
from unified_MLP_39.learner_state_resume import restore_training_state


def main():
    root = PACKAGE_ROOT / 'runs/training/MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21'
    result = {'simulation_started': False, 'learner_updates_executed': 0,
              'historical_checkpoints_modified': False, 'checkpoint_steps': {}, 'restore_probes': {}}
    for iteration in (1, 24, 25, 26, 27, 29, 40):
        path = root / f'milestone_iteration_{iteration:06d}/checkpoint_last/learner_group/learner/state.pkl'
        state = pickle.loads(path.read_bytes())
        result['checkpoint_steps'][str(iteration)] = {
            name: sorted({float(value['step']) for value in value['state']['state'].values()})
            for name, value in state['optimizer'].items()
        }
    for iteration in (25, 40):
        checkpoint = root / f'milestone_iteration_{iteration:06d}/checkpoint_last'
        directory = checkpoint / 'learner_group/learner'
        constructor = pickle.loads((directory / 'class_and_ctor_args.pkl').read_bytes())
        args, kwargs = constructor['ctor_args_and_kwargs']
        learner = constructor['class'](*args, **kwargs)
        learner.build()
        learner.restore_from_path(directory.resolve())
        source_bytes = (directory / 'state.pkl').read_bytes()
        source = pickle.loads(source_bytes)
        before_state = learner.get_state()
        untouched_keys = [key for key in before_state if key not in ('optimizer', 'rl_module')]
        before_other = {key: hashlib.sha256(pickle.dumps(before_state[key])).hexdigest()
                        for key in untouched_keys}
        audit = restore_on_learner(learner, source_optimizers=source['optimizer'])
        after_state = learner.get_state()
        after_other = {key: hashlib.sha256(pickle.dumps(after_state[key])).hexdigest()
                       for key in untouched_keys}
        if before_other != after_other:
            raise RuntimeError('Repair changed a learner counter or metrics state')
        audit.update(source_checkpoint=str(checkpoint.resolve()),
                     source_state_sha256=hashlib.sha256(source_bytes).hexdigest(),
                     learner_non_optimizer_state_unchanged=True,
                     checked_non_optimizer_state_keys=untouched_keys)
        if iteration == 40:
            audit['ppo_training_state'] = restore_training_state(
                learner, checkpoint, lambda local, function, **kw: [function(local, **kw)],
                hashlib.sha256(source_bytes).hexdigest())
        result['restore_probes'][str(iteration)] = audit
    output = Path(__file__).with_name('optimizer_resume_audit.json')
    output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
