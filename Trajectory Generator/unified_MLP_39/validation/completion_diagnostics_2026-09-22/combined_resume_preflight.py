"""Run the production Adam+KL resume helper on checkpoint40, offline only."""
from __future__ import annotations

import hashlib
import json
import pickle
from pathlib import Path
import sys

import numpy as np
import ray

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PACKAGE_ROOT.parent))
from unified_MLP_39._bootstrap import ensure_sim_paths
ensure_sim_paths()
from baseline_MLP import win_runtime  # noqa: F401
from unified_MLP_39.optimizer_resume import restore_and_audit


def tree_hashes(root):
    return {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(root.rglob('*')) if p.is_file()}


def arrays(module):
    return {key: value.detach().cpu().numpy().copy() for key, value in module.state_dict().items()}


def digests(state):
    result = {}
    for group, prefixes in [('actor', ('pi.', 'pi_encoder.')), ('critic', ('vf.', 'vf_encoder.')),
                            ('full_module', ('',))]:
        digest = hashlib.sha256()
        for key, value in sorted(state.items()):
            if not key.startswith(prefixes):
                continue
            value = np.asarray(value)
            digest.update(key.encode())
            digest.update(str(value.dtype).encode())
            digest.update(str(value.shape).encode())
            digest.update(value.tobytes())
        result[group] = digest.hexdigest()
    return result


def main():
    checkpoint = (PACKAGE_ROOT / 'runs/training/MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21'
                  / 'milestone_iteration_000040/checkpoint_last').resolve()
    before_files = tree_hashes(checkpoint)
    learner_dir = checkpoint / 'learner_group/learner'
    constructor = pickle.loads((learner_dir / 'class_and_ctor_args.pkl').read_bytes())
    args, kwargs = constructor['ctor_args_and_kwargs']
    learner = constructor['class'](*args, **kwargs)
    learner.build()
    learner.restore_from_path(learner_dir)
    module = learner.module['default_policy']
    before_weights = arrays(module)
    source_weights = pickle.loads((learner_dir / 'rl_module/default_policy/module_state.pkl').read_bytes())
    if before_weights.keys() != source_weights.keys() or not all(
            np.array_equal(before_weights[key], source_weights[key]) for key in before_weights):
        raise RuntimeError('RLlib module restore did not exactly reproduce checkpoint weights')
    before_counters = learner.get_state()
    other_keys = [key for key in before_counters if key not in ('optimizer', 'rl_module')]
    before_other = {key: hashlib.sha256(pickle.dumps(before_counters[key])).hexdigest() for key in other_keys}
    audit = restore_and_audit(
        learner, checkpoint, lambda local, function, **kw: [function(local, **kw)])
    after_weights = arrays(module)
    after_counters = learner.get_state()
    after_other = {key: hashlib.sha256(pickle.dumps(after_counters[key])).hexdigest() for key in other_keys}
    after_files = tree_hashes(checkpoint)
    if before_files != after_files or before_other != after_other or digests(before_weights) != digests(after_weights):
        raise RuntimeError('Preflight changed checkpoint, model weights, or unrelated learner state')
    if ray.is_initialized():
        raise RuntimeError('Preflight must not initialize Ray')
    audit.update(
        checkpoint_files_unchanged=True,
        checkpoint_file_count=len(before_files),
        checkpoint_file_sha256=before_files,
        source_weights_match=True,
        weights_unchanged=True,
        weight_digests={'source': digests(source_weights), 'before': digests(before_weights),
                        'after': digests(after_weights)},
        learner_non_optimizer_state_unchanged=True,
        checked_non_optimizer_state_keys=other_keys,
        ray_initialized=False, environments_created=0, simulation_steps=0, learner_updates=0,
    )
    output = Path(__file__).with_suffix('.json')
    output.write_text(json.dumps(audit, indent=2) + '\n')
    print(json.dumps({key: value for key, value in audit.items() if key != 'checkpoint_file_sha256'}, indent=2))


if __name__ == '__main__':
    main()
