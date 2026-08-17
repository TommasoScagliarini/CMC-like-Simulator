"""Diagnostic READ-ONLY capacity probe (option A).

Fits fresh-init masked W512 and W256 (control) on the REAL V12R16 P2 corpus
(11.404 rows, receipt PASS) with the exact frozen schedule, and reports the
per-trajectory RMSE tail — especially the two truncated seed-127 recovery
trajectories where the W256 H0-init candidate carries its worst error
(0.006841) and physically dies. No canonical write, no retry: pure math.
"""
import json
import sys
from pathlib import Path

import numpy as np

NS = Path(
    "/Users/tommy/Documents/CMC-like-Simulator - Claude/"
    "Trajectory Generator/baseline_MLP/validation/v12r16"
)
sys.path.insert(0, str(NS))

import h0_v12r16_masked_teacher_fitter as fitter  # noqa: E402

import torch  # noqa: E402


def run_fit(hidden: int) -> dict:
    corpus = fitter.load_fit_corpus("p2")
    normalization = fitter.build_masked_normalization(corpus.observations)
    normalized = fitter.normalized_masked_observations(
        corpus.observations, normalization
    )
    torch.set_num_threads(fitter.TORCH_THREADS)
    torch.use_deterministic_algorithms(True)
    torch.manual_seed(fitter.SEED)
    model = torch.nn.Sequential(
        torch.nn.Linear(35, hidden),
        torch.nn.Tanh(),
        torch.nn.Linear(hidden, hidden),
        torch.nn.Tanh(),
        torch.nn.Linear(hidden, 2),
    )
    mask = torch.as_tensor(fitter.MASKED_COLUMNS, dtype=torch.long)
    with torch.no_grad():
        model[0].weight.index_fill_(1, mask, 0.0)
    model[0].weight.register_hook(
        lambda gradient: gradient.index_fill(1, mask, 0.0)
    )
    x = torch.as_tensor(normalized, dtype=torch.float32)
    y = torch.as_tensor(corpus.actions, dtype=torch.float32)
    reset_mask = torch.as_tensor(corpus.step_indices == 1, dtype=torch.bool)
    keys = corpus.trajectory_ids.astype(str)
    _, inverse, counts = np.unique(keys, return_inverse=True, return_counts=True)
    factors = (float(fitter.ROWS_PER_CASE) / counts.astype(np.float64))[inverse]
    mass = torch.as_tensor(
        np.ascontiguousarray(factors, dtype=np.float32), dtype=torch.float32
    )
    optimizer = torch.optim.AdamW(
        model.parameters(),
        lr=fitter.ADAMW_RATES[0],
        weight_decay=fitter.ADAMW_WEIGHT_DECAY,
        foreach=False,
    )
    for epoch in range(1, fitter.ADAMW_EPOCHS + 1):
        rate = fitter.adamw_rate(epoch)
        for group in optimizer.param_groups:
            group["lr"] = rate
        optimizer.zero_grad(set_to_none=True)
        objective, _mse, _r = fitter._fit_objective(  # noqa: SLF001
            model(x), y, reset_mask, mass
        )
        objective.backward()
        torch.nn.utils.clip_grad_norm_(
            model.parameters(), fitter.GRADIENT_CLIP_NORM
        )
        optimizer.step()
        with torch.no_grad():
            model[0].weight.index_fill_(1, mask, 0.0)
    lbfgs = torch.optim.LBFGS(
        model.parameters(),
        lr=fitter.LBFGS_LR,
        max_iter=fitter.LBFGS_MAX_ITER,
        max_eval=fitter.LBFGS_MAX_EVAL,
        tolerance_grad=fitter.LBFGS_TOLERANCE_GRAD,
        tolerance_change=fitter.LBFGS_TOLERANCE_CHANGE,
        history_size=fitter.LBFGS_HISTORY_SIZE,
        line_search_fn="strong_wolfe",
    )

    def closure():
        lbfgs.zero_grad(set_to_none=True)
        objective, _m, _r = fitter._fit_objective(  # noqa: SLF001
            model(x), y, reset_mask, mass
        )
        objective.backward()
        return objective

    lbfgs.step(closure)
    with torch.no_grad():
        model[0].weight.index_fill_(1, mask, 0.0)
        prediction = model(x).numpy()
    error = prediction - corpus.actions
    per_traj = {}
    for tid in dict.fromkeys(keys):
        sel = keys == tid
        per_traj[tid] = float(np.sqrt(np.mean(np.square(error[sel]))))
    return {
        "hidden": hidden,
        "global_rmse": float(np.sqrt(np.mean(np.square(error)))),
        "global_max_abs": float(np.max(np.abs(error))),
        "reset_max_abs": float(
            np.max(np.abs(error[corpus.step_indices == 1]))
        ),
        "worst_per_trajectory_rmse": float(max(per_traj.values())),
        "worst_trajectory": max(per_traj, key=per_traj.get),
        "seed127_alpha050_rmse": per_traj.get("alpha_0p50__stochastic_nominal_seed_127"),
        "seed127_alpha075_rmse": per_traj.get("alpha_0p75__stochastic_nominal_seed_127"),
        "top5": sorted(per_traj.items(), key=lambda kv: -kv[1])[:5],
    }


results = {}
for hidden in (256, 512):
    results[f"w{hidden}_fresh"] = run_fit(hidden)
    print(json.dumps(results[f"w{hidden}_fresh"], indent=2), flush=True)
print("=== CONFRONTO ===")
print(json.dumps({
    "w256_h0init_reference": {
        "global_rmse": 0.004981690059032371,
        "worst_per_trajectory_rmse": 0.006840683757686013,
    },
    "w256_fresh": {k: results["w256_fresh"][k] for k in ("global_rmse", "worst_per_trajectory_rmse")},
    "w512_fresh": {k: results["w512_fresh"][k] for k in ("global_rmse", "worst_per_trajectory_rmse")},
}, indent=2))
