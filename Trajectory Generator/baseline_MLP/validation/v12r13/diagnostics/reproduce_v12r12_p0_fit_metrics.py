"""Forensic READ-ONLY in-memory reproduction of the failed V12R12 P0 fit.

Imports the frozen fitter and calls its pure functions only. No canonical
write, no retry: the deterministic seed/schedule reproduce the exact failed
fit so the unpersisted metrics can be recovered for successor design.
"""
import json
import sys
from pathlib import Path

NS = Path("/Users/tommy/Documents/CMC-like-Simulator - Claude/Trajectory Generator/baseline_MLP/validation/v12r12")
sys.path.insert(0, str(NS))

import h0_v12r12_masked_teacher_fitter as fitter  # noqa: E402

corpus = fitter.load_fit_corpus("p0")
normalization = fitter.build_masked_normalization(corpus.observations)
_module, source_state = fitter._load_source_module_and_state()  # noqa: SLF001
result = fitter.fit_candidate_in_memory(
    source_state=source_state, corpus=corpus, normalization=normalization
)
gate = fitter.fit_gate(
    metrics=result.metrics,
    state_audit=result.state_audit,
    fold_audit=result.fold_audit,
)
m = result.metrics
out = {
    "global_rmse": m["global"]["rmse"],
    "global_max_abs": m["global"]["max_abs_error"],
    "reset_max_abs": m["reset_max_abs_error"],
    "transition_window_max_abs": m["transition_window_max_abs_error"],
    "temporal_first_diff_max_abs": m["temporal_first_difference_max_abs_error"],
    "worst_per_case_action_rmse": m["worst_per_case_or_action_rmse"],
    "worst_per_trajectory_action_rmse": m["worst_per_trajectory_or_action_rmse"],
    "predicted_mean_max_abs": m["predicted_mean_max_abs"],
    "limits": {
        "global_rmse": fitter.GLOBAL_RMSE_LIMIT,
        "global_max_abs": fitter.GLOBAL_MAX_ABS_LIMIT,
        "per_case_action_rmse": fitter.PER_CASE_ACTION_RMSE_LIMIT,
        "reset_max_abs": fitter.RESET_MAX_ABS_LIMIT,
        "transition_max_abs": fitter.TRANSITION_MAX_ABS_LIMIT,
        "first_diff_max_abs": fitter.TEMPORAL_FIRST_DIFFERENCE_MAX_ABS_LIMIT,
    },
    "gate_passed": gate["passed"],
    "failed_checks": sorted(k for k, v in gate["checks"].items() if v is False),
    "optimizer": {
        "terminal_objective": result.optimizer_audit["terminal_objective"],
        "terminal_mse": result.optimizer_audit["terminal_mse"],
        "lbfgs_closure_calls": result.optimizer_audit["lbfgs_closure_calls"],
    },
    "per_case_rmse": {k: v["rmse"] for k, v in m["per_case"].items()},
}
print(json.dumps(out, indent=2, sort_keys=True, allow_nan=False))
