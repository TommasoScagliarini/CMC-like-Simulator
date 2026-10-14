"""Existing supervised training driver with an isolated module selection hook."""
import json
import os
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
sys.path.insert(0, str(PACKAGE.parent))


def install(treatment):
    from unified_MLP_39 import train_ppo_mlp as driver, training_runtime
    from unified_MLP_39.validation.masking_verification_2026_10_01.sigma_module import (
        SigmaComparisonModule, audit_learner, audit_runner)
    original = driver._asymmetric_module_selection
    initialize = driver._initialize_unified_run
    after = training_runtime.after_iteration
    trace = json.loads((PACKAGE / "runs/rollout/unified_MLP_39/runs/rollout/MLP39_LEFT_fresh_best90_2026-09-30_deterministic/rollout_policy_trace.json").read_text())
    import numpy as np
    probes = np.asarray([r["actor_observation_vector_before"] for r in trace[::25]], np.float32)
    hidden = probes.copy()
    hidden[:, [0, 1, 35, 36, 37, 38]] = 0
    probes = np.concatenate([probes, hidden]).tolist()

    def selection(args, **kwargs):
        _, cfg = original(args, **kwargs)
        cfg["diagnostic_sigma_treatment"] = treatment
        return SigmaComparisonModule, cfg

    def initial(algo, args, output_dir, resume_path, reward_overrides):
        initialize(algo, args, output_dir, resume_path, reward_overrides)
        learners = driver._learner_call_results(algo, audit_learner, probes=probes)
        runners = algo.env_runner_group.foreach_env_runner(
            lambda r: audit_runner(r, probes), local_env_runner=True,
            timeout_seconds=args.startup_timeout_s)
        digest = learners[0]["reference_sha256"]
        if not all(r["reference_ready"] and r["reference_sha256"] == digest and
                   r["logstd_max_delta_vs_cp90_on_fixed_probes"] == 0 for r in learners+runners):
            raise RuntimeError("Sampler/learner CP90 distribution initialization differs")
        algo._sigma_reference_digest = digest
        record = {"treatment": treatment, "learner": learners, "runners": runners,
                  "source_checkpoint": str(resume_path), "initial_sigma_exact": True}
        Path(output_dir, "sigma_initialization_audit.json").write_text(json.dumps(record,indent=2)+"\n")

    def completed(algo, iteration, before, args, driver_module):
        record = after(algo, iteration, before, args, driver_module)
        reports = driver._learner_call_results(algo, audit_learner, probes=probes)
        if not all(r["reference_sha256"] == algo._sigma_reference_digest for r in reports):
            raise RuntimeError("Immutable CP90 reference changed")
        record["sigma_diagnostic"] = reports
        return record

    driver._asymmetric_module_selection = selection
    driver._initialize_unified_run = initial
    training_runtime.after_iteration = completed
    driver._SCRIPT_PATH = Path(__file__).resolve()
    return driver


if __name__ == "__main__":
    install(os.environ["CMC_SIGMA_TREATMENT"]).main()
