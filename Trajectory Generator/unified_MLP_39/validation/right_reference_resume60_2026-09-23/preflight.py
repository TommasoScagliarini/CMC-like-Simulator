"""Check checkpoint40 restoration locally without Ray, environments or updates."""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import pickle
import sys

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "VECLIB_MAXIMUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ[key] = "1"
sys.path.insert(0, str(PACKAGE.parent))
from unified_MLP_39._bootstrap import ensure_sim_paths
ensure_sim_paths()
from baseline_MLP import win_runtime  # noqa: F401
import numpy as np
import ray
import torch
from unified_MLP_39.optimizer_resume import restore_and_audit

torch.set_num_threads(1)
CHECKPOINT = PACKAGE / "runs/training/MLP39_v26_aligned_right_phase_fsmv3_40iter_2026-09-23/milestone_iteration_000040/checkpoint_last"


def hashes():
    return {str(p.relative_to(CHECKPOINT)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(CHECKPOINT.rglob("*")) if p.is_file()}


def arrays(module):
    return {key: value.detach().cpu().numpy().copy() for key, value in module.state_dict().items()}


def main():
    before_files = hashes()
    learner_dir = CHECKPOINT / "learner_group/learner"
    constructor = pickle.loads((learner_dir / "class_and_ctor_args.pkl").read_bytes())
    args, kwargs = constructor["ctor_args_and_kwargs"]
    learner = constructor["class"](*args, **kwargs)
    learner.build()
    learner.restore_from_path(learner_dir)
    module = learner.module["default_policy"]
    before_weights = arrays(module)
    source_weights = pickle.loads((learner_dir / "rl_module/default_policy/module_state.pkl").read_bytes())
    assert before_weights.keys() == source_weights.keys()
    assert all(np.array_equal(before_weights[key], source_weights[key]) for key in before_weights)
    before_state = learner.get_state()
    other_keys = [key for key in before_state if key not in ("optimizer", "rl_module")]
    before_other = {key: hashlib.sha256(pickle.dumps(before_state[key])).hexdigest() for key in other_keys}
    audit = restore_and_audit(learner, CHECKPOINT, lambda local, function, **kw: [function(local, **kw)])
    after_weights = arrays(module)
    after_state = learner.get_state()
    after_other = {key: hashlib.sha256(pickle.dumps(after_state[key])).hexdigest() for key in other_keys}
    assert all(np.array_equal(before_weights[key], after_weights[key]) for key in before_weights)
    assert before_files == hashes() and before_other == after_other
    assert not ray.is_initialized()
    assert audit["passed"] and audit["ppo_training_state"]["source"] == "checkpoint_sidecar"
    audit.update(checkpoint_files_unchanged=True, checkpoint_file_count=len(before_files),
        source_weights_match=True, weights_unchanged=True, learner_non_optimizer_state_unchanged=True,
        checked_non_optimizer_state_keys=other_keys, ray_initialized=False, environments_created=0,
        simulation_steps=0, learner_updates=0, torch_threads=torch.get_num_threads())
    (HERE / "preflight.json").write_text(json.dumps(audit, indent=2) + "\n")
    print(json.dumps(audit, indent=2))


if __name__ == "__main__":
    main()
