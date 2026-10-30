"""V26B R0 FEASIBILITY DIAGNOSTIC (architect order after the R0 FAIL audit, 2026-08-24).

ADDITIVE tooling: nothing frozen is rewritten (rev3 parents, rev3a amendment, V1,
V26, R0 datasets/log/report untouched).  NO export, NO rollout, NO threshold/
beta/budget/protocol change.  Purpose: distinguish TARGET CONFLICT from an
optimisation/role limit behind the R0 offline-gate FAIL.

Deliverables implemented here:
1. Q1/Q2 of the INITIAL V1 state on the frozen R0 datasets;
2. quantitative obs35 separability between the roles (standardised features,
   cross-role vs intra-role nearest-neighbour distance distributions, target gap
   on cross-role NN pairs, deterministic kNN holdout as an empirical bound);
3. two MONO-ROLE diagnostic fits (same V1 init, same frozen budget/regs), each
   evaluated on BOTH Q1 and Q2 — feasibility audit only, never exportable;
5. HARDENED full-SHA pins of the IK privileged caches (the v26b_v2 prefixes are
   cross-checked against these full digests).
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path
from typing import Any, Mapping

import numpy as np

HERE = Path(__file__).resolve().parent
VALIDATION_DIR = HERE.parent
for _entry in (str(VALIDATION_DIR / "f0_freeze_2026-08-22"), str(VALIDATION_DIR / "f1_ablation_2026-08-23"), str(VALIDATION_DIR / "f2r_bridge_2026-08-23"), str(HERE)):
    if _entry not in sys.path:
        sys.path.insert(0, _entry)

import f0_common as C  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_labeller as L  # noqa: E402
import f2r_refit as RF  # noqa: E402
import v26b_anchors as VA  # noqa: E402
import v26b_v2 as V2  # noqa: E402


class DiagError(RuntimeError):
    pass


# --- (5) FULL IK-cache digests (hardened pins; v26b_v2 prefixes cross-checked) --------------------
IK_CACHE_DIGESTS_FULL = {
    "minus020": "f97ad154f75541626565f7d6fd392dface4f723c2861143f1dc8172536767230",
    "nominal": "3dd878d4d6d2930d730c1a67f39d6799f20221e7659a435c02d062bfd553d9b0",
    "plus020": "f15d624cc910b815ee4c511e6702e1312cc4be0f67e93ae435b3e14b850f7d20",
}


def verify_ik_caches_full() -> dict[str, str]:
    """Fail-closed FULL-digest verification of the pinned F2R privileged caches (+ prefix
    consistency with the frozen v26b_v2 pins)."""
    out = {}
    for start in R.STARTS:
        cc = L.load_cache(R.OUT_CACHE, start)
        d = cc.digest()
        if d != IK_CACHE_DIGESTS_FULL[start]:
            raise DiagError(f"cache {start}: digest {d} != FULL pin {IK_CACHE_DIGESTS_FULL[start]}")
        if not d.startswith(V2.PIN_F2R_CACHE_DIGESTS[start]):
            raise DiagError(f"cache {start}: frozen v26b_v2 prefix pin inconsistent with the full digest")
        if cc.ik_action is None or cc.rows != 500:
            raise DiagError(f"cache {start}: missing u_IK or wrong grid")
        out[start] = d
    return out


# --- dataset loading (frozen R0 npz, content-addressed) -------------------------------------------
R0_DATASETS = {  # frozen R0 dataset files; digests verified against the frozen dataset receipt
    "task": "v26b_dataset_R0_task_20260824T143724.npz",
    "pres": "v26b_dataset_R0_pres_20260824T143724.npz",
}


def load_r0_datasets() -> tuple[dict[str, np.ndarray], dict[str, np.ndarray], dict[str, str]]:
    ds_dir = VA.OUT_ROOT / "datasets"
    receipt = json.loads((ds_dir / "v26b_dataset_R0_receipt_20260824T143724.json").read_text(encoding="utf-8"))
    shas = {}
    out = {}
    for name in ("task", "pres"):
        fname = R0_DATASETS[name]
        path = ds_dir / fname
        sha = C.sha256_file(path)
        if sha != receipt["files"][name]["sha256"]:
            raise DiagError(f"{fname}: digest {sha} != frozen dataset receipt")
        z = np.load(path)
        out[name] = {k: z[k] for k in z.files}
        shas[name] = sha
    task, pres = out["task"], out["pres"]
    V2.assert_r0_contract(task, pres)
    return task, pres, shas


# --- (2) separability analysis --------------------------------------------------------------------

def standardise_union(task_obs: np.ndarray, pres_obs: np.ndarray, *, eps: float = 1e-9) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    union = np.concatenate([task_obs, pres_obs]).astype(np.float64)
    mu = union.mean(axis=0); sd = union.std(axis=0)
    constant = sd <= eps
    keep = ~constant
    zt = ((task_obs.astype(np.float64) - mu) / np.where(constant, 1.0, sd))[:, keep]
    zp = ((pres_obs.astype(np.float64) - mu) / np.where(constant, 1.0, sd))[:, keep]
    info = {"features_total": int(union.shape[1]), "features_used": int(keep.sum()), "constant_feature_indices_excluded": np.where(constant)[0].tolist()}
    return zt.astype(np.float32), zp.astype(np.float32), info


def _nn_min_dist(A: np.ndarray, B: np.ndarray, *, exclude_self: bool = False, chunk: int = 512) -> tuple[np.ndarray, np.ndarray]:
    """min_j ||A_i - B_j|| and argmin (chunked, float64 accumulation of squared distances)."""
    nb = B.shape[0]
    b2 = np.sum(B.astype(np.float64) ** 2, axis=1)
    best = np.full(A.shape[0], np.inf); best_j = np.zeros(A.shape[0], dtype=np.int64)
    for s in range(0, A.shape[0], chunk):
        a = A[s: s + chunk].astype(np.float64)
        d2 = np.sum(a ** 2, axis=1)[:, None] + b2[None, :] - 2.0 * (a @ B.astype(np.float64).T)
        if exclude_self:
            for r in range(a.shape[0]):
                d2[r, s + r] = np.inf
        j = np.argmin(d2, axis=1)
        best[s: s + chunk] = np.sqrt(np.maximum(d2[np.arange(a.shape[0]), j], 0.0))
        best_j[s: s + chunk] = j
    return best, best_j


def _pct(x: np.ndarray) -> dict[str, float]:
    q = np.percentile(x, [5, 25, 50, 75, 95])
    return {"p5": float(q[0]), "p25": float(q[1]), "p50": float(q[2]), "p75": float(q[3]), "p95": float(q[4]), "mean": float(np.mean(x))}


def separability_analysis(task: Mapping[str, np.ndarray], pres: Mapping[str, np.ndarray]) -> dict[str, Any]:
    zt, zp, feat = standardise_union(task["obs35"], pres["obs35"])
    cross_t, jt = _nn_min_dist(zt, zp)          # task row -> nearest pres row
    cross_p, jp = _nn_min_dist(zp, zt)          # pres row -> nearest task row
    intra_t, _ = _nn_min_dist(zt, zt, exclude_self=True)
    intra_p, _ = _nn_min_dist(zp, zp, exclude_self=True)
    gap = np.abs(task["actions"].astype(np.float64) - pres["actions"].astype(np.float64)[jt])  # u_IK(task) vs u_T(nearest pres)
    order = np.argsort(cross_t)
    decile = order[: max(1, order.size // 10)]
    ratio_t = cross_t / np.maximum(intra_t, 1e-12)
    return {
        "standardisation": feat,
        "language_note": "cross-role nearest neighbours are NEAR states, never claimed identical (0 bitwise-identical rows across roles)",
        "nn_distance_std_units": {
            "cross_task_to_pres": _pct(cross_t), "cross_pres_to_task": _pct(cross_p),
            "intra_task": _pct(intra_t), "intra_pres": _pct(intra_p),
            "ratio_cross_over_intra_task_rows": _pct(ratio_t),
        },
        "target_gap_on_cross_nn_pairs_abs": {
            "knee": _pct(gap[:, 0]), "ankle": _pct(gap[:, 1]),
            "closest_decile_pairs": {"n": int(decile.size), "knee_mean": float(np.mean(gap[decile, 0])), "ankle_mean": float(np.mean(gap[decile, 1])), "cross_dist_p50_of_decile": float(np.median(cross_t[decile]))},
        },
    }


def knn_holdout_bounds(task: Mapping[str, np.ndarray], pres: Mapping[str, np.ndarray], *, k_list: tuple[int, ...] = (1, 5), holdout_stride: int = 5) -> dict[str, Any]:
    """Deterministic kNN holdout: every ``holdout_stride``-th row of each role is held out; the
    predictor is fit on the remaining UNION rows (each keeping its own role target).  The held-out
    per-role per-joint RMSE is the empirical bound of a pure state-lookup; intra-role-only kNN on
    the same split is the local noise floor."""
    zt, zp, _ = standardise_union(task["obs35"], pres["obs35"])
    yt = task["actions"].astype(np.float64); yp = pres["actions"].astype(np.float64)
    ht = np.zeros(zt.shape[0], bool); ht[::holdout_stride] = True
    hp = np.zeros(zp.shape[0], bool); hp[::holdout_stride] = True
    train_X = np.concatenate([zt[~ht], zp[~hp]]); train_y = np.concatenate([yt[~ht], yp[~hp]])
    def knn_rmse(Xq: np.ndarray, yq: np.ndarray, X: np.ndarray, y: np.ndarray, k: int, chunk: int = 256) -> list[float]:
        b2 = np.sum(X.astype(np.float64) ** 2, axis=1)
        preds = np.empty_like(yq)
        for s in range(0, Xq.shape[0], chunk):
            a = Xq[s: s + chunk].astype(np.float64)
            d2 = np.sum(a ** 2, axis=1)[:, None] + b2[None, :] - 2.0 * (a @ X.astype(np.float64).T)
            idx = np.argpartition(d2, kth=k - 1, axis=1)[:, :k]
            preds[s: s + chunk] = np.mean(y[idx], axis=1)
        return np.sqrt(np.mean((preds - yq) ** 2, axis=0)).tolist()
    out: dict[str, Any] = {"holdout_stride": holdout_stride, "deterministic": True}
    for k in k_list:
        out[f"k{k}"] = {
            "union_train": {
                "pres_holdout_rmse_vs_uT": knn_rmse(zp[hp], yp[hp], train_X, train_y, k),
                "task_holdout_rmse_vs_uIK": knn_rmse(zt[ht], yt[ht], train_X, train_y, k),
            },
            "intra_role_train_only": {
                "pres_holdout_rmse_vs_uT": knn_rmse(zp[hp], yp[hp], zp[~hp], yp[~hp], k),
                "task_holdout_rmse_vs_uIK": knn_rmse(zt[ht], yt[ht], zt[~ht], yt[~ht], k),
            },
        }
    return out


# --- (3) mono-role diagnostic fit (same loop, single role; NEVER exportable) ----------------------

def fit_mono_role(init_state: Mapping[str, Any], role: Mapping[str, Any], *, budget: Mapping[str, Any] | None = None, progress: bool = False) -> tuple[dict[str, Any], dict[str, Any]]:
    """FEASIBILITY AUDIT ONLY (architect-ordered diagnostic; not tuning, no export path exists).

    The v26b_v2.fit_r0 loop (itself bit-proven against the immutable
    ``f2r_refit.fit_student_preserving``) with a SINGLE role term::

        L = role_norm + lambda_clip*clip + lambda_phi*aux + lambda_anchor*||theta - theta_init||^2

    Same frozen budget (epochs/lr/batch/seed) and identical regularisers."""
    import torch
    import torch.nn.functional as F

    budget = dict(budget or V2.BUDGET_R0)
    V2._role_contract(role, allowed_purposes=V2._TASK_PURPOSES | V2._PRES_PURPOSES, what="mono_role")
    info = RF.validate_init_state(init_state, expected_actor_digest=None)
    torch.use_deterministic_algorithms(True); torch.set_num_threads(1); torch.manual_seed(int(budget["seed"])); np.random.seed(int(budget["seed"]))
    f32 = np.float32
    obs = torch.as_tensor(np.asarray(role["obs35"], dtype=f32))
    tgt = torch.as_tensor(np.asarray(role["actions"], dtype=f32))
    clk = torch.as_tensor(np.asarray(role["clock"], dtype=f32))
    var_role = np.var(np.asarray(role["actions"], dtype=np.float64), axis=0)
    if np.any(var_role <= 0.0):
        raise DiagError("degenerate per-joint variance")
    vr = torch.as_tensor(var_role.astype(f32))
    W1 = torch.nn.Parameter(torch.as_tensor(np.array(init_state["pi.0.0.weight"], dtype=f32))); b1 = torch.nn.Parameter(torch.as_tensor(np.array(init_state["pi.0.0.bias"], dtype=f32)))
    W2 = torch.nn.Parameter(torch.as_tensor(np.array(init_state["pi.0.2.weight"], dtype=f32))); b2 = torch.nn.Parameter(torch.as_tensor(np.array(init_state["pi.0.2.bias"], dtype=f32)))
    W3m = torch.nn.Parameter(torch.as_tensor(np.array(init_state["pi.1.weight"][: R.ACTION_DIM], dtype=f32))); b3m = torch.nn.Parameter(torch.as_tensor(np.array(init_state["pi.1.bias"][: R.ACTION_DIM], dtype=f32)))
    hidden = int(W2.shape[0]); g = torch.Generator().manual_seed(int(budget["seed"]))
    Wh = torch.nn.Parameter(torch.randn(2, hidden, generator=g) * 0.01); bh = torch.nn.Parameter(torch.zeros(2))
    mean_params = [W1, b1, W2, b2, W3m, b3m]; anchor_t = [p.detach().clone() for p in mean_params]
    clock_mask = torch.ones_like(W1); clock_mask[:, list(R.CLOCK_COLUMNS)] = 0.0
    with torch.no_grad():
        W1.mul_(clock_mask)
    logstd_w = np.zeros((R.ACTION_DIM, hidden), dtype=f32); logstd_b = np.asarray(np.log(np.full(R.ACTION_DIM, float(R.SIGMA_CONSTANT))), dtype=f32)
    encoder = lambda x: torch.tanh(torch.tanh(x @ W1.T + b1) @ W2.T + b2)  # noqa: E731
    opt = torch.optim.Adam(mean_params + [Wh, bh], lr=float(budget["lr"])); rng = np.random.default_rng(int(budget["seed"]))
    n = int(obs.shape[0]); epochs = int(budget["epochs"]); batch = int(budget["batch_size"])
    lam_clip, lam_phi, lam_a = float(budget["clip_weight"]), float(budget["lambda_phi"]), float(budget["lambda_anchor"])
    history = []
    for epoch in range(1, epochs + 1):
        perm = rng.permutation(n); losses = []
        for s0 in range(0, n, batch):
            idx = torch.as_tensor(perm[s0: s0 + batch]); opt.zero_grad(set_to_none=True)
            h2 = encoder(obs[idx]); m = h2 @ W3m.T + b3m; hp = h2 @ Wh.T + bh
            l_role = (((m - tgt[idx]) ** 2) / vr).mean(dim=0).mean()
            loss = l_role + lam_clip * torch.relu(torch.abs(m) - 1.0).square().mean() + lam_phi * F.mse_loss(hp, clk[idx])
            if lam_a > 0.0:
                loss = loss + lam_a * torch.stack([(p - a0).square().sum() for p, a0 in zip(mean_params, anchor_t)]).sum()
            loss.backward(); opt.step()
            with torch.no_grad():
                W1.mul_(clock_mask)
            losses.append(float(loss.item()))
        history.append({"epoch": epoch, "loss": float(np.mean(losses))})
        if progress and (epoch == 1 or epoch % 50 == 0):
            print(json.dumps(history[-1]), flush=True)
    w3 = np.concatenate([W3m.detach().numpy().astype(f32), logstd_w], axis=0); b3 = np.concatenate([b3m.detach().numpy().astype(f32), logstd_b], axis=0)
    canon = {"pi.0.0.weight": W1.detach().numpy().astype(f32).copy(), "pi.0.0.bias": b1.detach().numpy().astype(f32).copy(), "pi.0.2.weight": W2.detach().numpy().astype(f32).copy(), "pi.0.2.bias": b2.detach().numpy().astype(f32).copy(), "pi.1.weight": w3.astype(f32).copy(), "pi.1.bias": b3.astype(f32).copy()}
    new_state = {}
    for key in RF.EXPECTED_KEY_ORDER:
        new_state[key] = canon[key].copy() if key in canon else canon[RF.ALIAS_KEYS[key]].copy()
    RF.validate_init_state(new_state, expected_actor_digest=None)
    report = {"tool": "v26b_r0_diag.fit_mono_role (feasibility audit only; never exported)", "rows": n, "budget": budget, "var_role_per_joint": var_role.tolist(), "init_actor_digest": info["actor_digest"], "new_actor_digest": RF.actor_state_digest(new_state), "history_first_last": {"first": history[0], "last": history[-1]}}
    return new_state, report


def q_both(init_state: Mapping[str, Any], state: Mapping[str, Any], task: Mapping[str, Any], pres: Mapping[str, Any]) -> dict[str, Any]:
    c = V2.evaluate_q_criteria(init_state, state, task, pres)
    return {"Q1_rmse_vs_uT_anchors": c["Q1"]["rmse_knee_ankle_vs_V26_mean_on_anchors"], "Q1_pass": c["Q1"]["pass"], "Q2_rmse_vs_uIK_alt": c["Q2"]["rmse_knee_ankle_vs_uIK_on_alt_start_rows"], "Q2_pass": c["Q2"]["pass"], "Q3_pass": c["Q3"]["pass"]}


# --- runner ---------------------------------------------------------------------------------------

def run_diagnostic(out_path: Path, *, progress: bool = True) -> dict[str, Any]:
    if out_path.exists():
        raise DiagError(f"no-clobber: {out_path} exists")
    lineage = V2.verify_lineage()
    caches = verify_ik_caches_full()
    task, pres, ds_shas = load_r0_datasets()
    init = V2.load_v1_init()
    result: dict[str, Any] = {
        "schema": "v26b_r0_feasibility_diag.1",
        "scope": "diagnostic only: no export, no rollout, no threshold/beta/budget/protocol change; frozen artefacts untouched",
        "lineage": lineage, "ik_cache_digests_full": caches, "dataset_sha256": ds_shas,
    }
    result["q_of_V1_init"] = q_both(init, init, task, pres)
    if progress:
        print("V1 init:", json.dumps(result["q_of_V1_init"]), flush=True)
    result["separability"] = separability_analysis(task, pres)
    if progress:
        print("separability done", flush=True)
    result["knn_bounds"] = knn_holdout_bounds(task, pres)
    if progress:
        print("knn done", flush=True)
    for name, role in (("pres_only", pres), ("task_only", task)):
        st, rep = fit_mono_role(init, role, progress=progress)
        result[f"fit_{name}"] = {**rep, "q_both": q_both(init, st, task, pres)}
        if progress:
            print(name, json.dumps(result[f"fit_{name}"]["q_both"]), flush=True)
    payload = json.dumps(result, indent=2, ensure_ascii=False, default=str) + "\n"
    import os
    part = out_path.with_name(out_path.name + f".part-{os.getpid()}")
    with open(part, "x", encoding="utf-8") as fh:
        fh.write(payload); fh.flush(); os.fsync(fh.fileno())
    os.replace(part, out_path)
    result["_saved"] = {"path": str(out_path), "sha256": C.sha256_file(out_path)}
    return result


def main(argv=None) -> int:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    out = VA.OUT_ROOT / f"r0_feasibility_diag_{stamp}.json"
    result = run_diagnostic(out)
    print(json.dumps({"saved": result["_saved"]}, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
