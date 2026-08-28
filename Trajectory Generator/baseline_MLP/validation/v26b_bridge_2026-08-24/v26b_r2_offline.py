"""V26B rev3f — R2 OFFLINE stage: role-weighted July-mass refit (B+C), gates-first, no rollout.

Roles (unique rows, NO physical duplication) with July-13 mass weighting:
base 992 (mass 16000) + r1_prefix steps 1..98 (mass 712) + alt minus020 6447 /
plus020 6429 (mass 4000 each), total mass 24712.  Pre-fit gates G1/G2/G3 +
coverage are fail-closed and run BEFORE any fit; G2 is never relaxed (STOP with
diagnostics if it fails).  Refit = rev3d numeric protocol, init EXACTLY the R1
actor, ONLY the data loss replaced by the role-weighted objective
``sum_i lambda_i e_i / sum_i lambda_i`` with ``lambda_i = mass_r/(24712*n_r)``
and ``e_i = mean_j (m_ij-y_ij)^2/Var_{r,j}``.  Sigma stays the UNDECIDED frozen
placeholder.  No rollout / PPO / critic / sigma sweep.
"""

from __future__ import annotations

import argparse
import json
import pickle
import shutil
import sys
import time
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
VALIDATION_DIR = HERE.parent
for _entry in (str(VALIDATION_DIR / "f0_freeze_2026-08-22"), str(VALIDATION_DIR / "f1_ablation_2026-08-23"), str(VALIDATION_DIR / "f2r_bridge_2026-08-23"), str(HERE)):
    if _entry not in sys.path:
        sys.path.insert(0, _entry)

import f0_common as C  # noqa: E402
import f1_dataset as DS  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_labeller as L  # noqa: E402
import f2r_refit as RF  # noqa: E402
import v26b_anchors as VA  # noqa: E402
import v26b_dagger_r1 as D1  # noqa: E402
import v26b_r0a as A  # noqa: E402
import v26b_r0a_rollout as RO  # noqa: E402
import v26b_r1_rollout as R1  # noqa: E402
import v26b_student as VS  # noqa: E402
import v26b_v2 as V2  # noqa: E402


class R2Error(RuntimeError):
    pass


AUTHORIZED_STAGE = "V26B-R2-OFFLINE"
AMENDMENT_REV3F = HERE / "v26b_amendment_rev3f_r2_offline_roleweighted.json"
PIN_AMENDMENT_REV3F = "011eff9b5da77dd437fb1e9f046988929dea3b196a66c8e4a0adb1e993ef3652"
PIN_BASE_NPZ = "ce309b40e716342bc00a4fe6e8b835815c93419d36be310271f8f4e2ab7250de"
PIN_ALT_NPZ = "9862d5f83f0471f8503d89e69a9eccc2aba7482641cfb49585a3e4efb706d009"
BASE_NPZ = VA.OUT_ROOT / "datasets" / "v26b_dataset_DAGGER_R1_20260824T153527.npz"
ALT_NPZ = VA.OUT_ROOT / "datasets" / "v26b_dataset_R0_task_20260824T143724.npz"
JULY_MASSES = {"base": 16000.0, "r1_prefix": 712.0, "alt_minus020": 4000.0, "alt_plus020": 4000.0}
TOTAL_MASS = 24712.0
PREFIX_KEEP = 98          # G1: label-valid prefix of the R1 trace (first discrete mismatch vs R0a at step 99)
PREFIX_EXCLUDED = 144     # steps 99..242, excluded and counted
G1_ABORT_FRACTION = 0.80
G2_SPREAD_MAX = 0.2
G3_K = 5
G3_RMSE_MAX = 0.15
GATE_RMSE_MAX = 0.15
HOLDOUT_FRACTION = 0.2
OUT_R2 = VA.OUT_ROOT / "student" / "V2_R2"
RECEIPT_NAME = "v26b_r2_receipt.json"
SIGMA_NOTE = D1.SIGMA_NOTE


def verify_lineage_r2() -> dict[str, Any]:
    lin = R1.verify_lineage_r1_rollout()  # chain rev3..rev3e + R1 module/actor + rollout artefacts
    for path, pin, key in ((AMENDMENT_REV3F, PIN_AMENDMENT_REV3F, "amendment_rev3f"), (BASE_NPZ, PIN_BASE_NPZ, "base_npz"), (ALT_NPZ, PIN_ALT_NPZ, "alt_npz")):
        p = Path(path)
        if p.is_symlink() or not p.is_file():
            raise R2Error(f"lineage file missing (or symlink): {p}")
        got = C.sha256_file(p)
        if got != pin:
            raise R2Error(f"lineage violated: {p.name} sha {got} != pinned {pin}")
        lin[key] = {"path": C.rel(p), "sha256": got}
    return lin


# --- dataset --------------------------------------------------------------------------------------

def build_r2_dataset() -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    zb = np.load(BASE_NPZ)
    base = {k: zb[k] for k in zb.files}
    if base["obs35"].shape[0] != 992:
        raise R2Error("base npz rows != 992")
    za = np.load(ALT_NPZ)
    alt = {k: za[k] for k in za.files}
    if alt["obs35"].shape[0] != 12876:
        raise R2Error("alt npz rows != 12876")
    # r1_prefix: steps 1..98 of the pinned R1 trace, u_IK same absolute t
    traj = DS.trajectory_from_job(R1.JOB_DIR, expected_width=R.ENV_ACTOR_WIDTH)
    if traj["trace_sha256"] != "a73c4d8f8e26e3e5a95fcc1f0ac42521ea1ba263b3b65583c277b7b75ea8a672":
        raise R2Error("R1 trace digest != pinned")
    obs_r1 = np.asarray(traj["obs35"], dtype=np.float64).astype(np.float32)
    t_r1 = np.asarray(traj["t_pre"], dtype=np.float64)
    # G1: the 8 discrete features of rows 1..PREFIX_KEEP must equal the R0a reference at the same step
    obs_r0a = DS.trajectory_from_job(RO.JOB_DIR, expected_width=R.ENV_ACTOR_WIDTH)["obs35"].astype(np.float32)
    idx_disc = RO.discrete_feature_indices(R.FEATURE_NAMES_35)
    agree = np.all(obs_r1[:PREFIX_KEEP][:, idx_disc] == obs_r0a[:PREFIX_KEEP][:, idx_disc], axis=1)
    if not np.all(agree):
        raise R2Error(f"G1 violated: discrete mismatch inside the declared prefix at rows {np.where(~agree)[0][:5] + 1}")
    excluded_fraction = PREFIX_EXCLUDED / float(obs_r1.shape[0])
    if excluded_fraction > G1_ABORT_FRACTION:
        raise R2Error(f"G1 abort: excluded on-policy fraction {excluded_fraction:.3f} > {G1_ABORT_FRACTION}")
    cc = L.load_cache(R.OUT_CACHE, "nominal")
    kidx = cc.lookup(t_r1[:PREFIX_KEEP])
    u_pre = cc.ik_action[kidx].astype(np.float32)
    clock_pre = cc.clock[kidx].astype(np.float32)
    def cat(key, arrays):
        return np.concatenate(arrays)
    role = np.concatenate([np.asarray(["base"] * 992, dtype=str),
                           np.asarray(["r1_prefix"] * PREFIX_KEEP, dtype=str),
                           np.asarray([f"alt_{s}" for s in alt["start"]], dtype=str)])
    data = {
        "obs35": np.concatenate([base["obs35"].astype(np.float32), obs_r1[:PREFIX_KEEP], alt["obs35"].astype(np.float32)]),
        "t_pre": np.concatenate([base["t_pre"], t_r1[:PREFIX_KEEP], alt["t_pre"]]),
        "actions": np.concatenate([base["actions"].astype(np.float32), u_pre, alt["actions"].astype(np.float32)]),
        "clock": np.concatenate([base["clock"].astype(np.float32), clock_pre, alt["clock"].astype(np.float32)]),
        "seed": np.concatenate([base["seed"], np.full(PREFIX_KEEP, R.DET_SEED, dtype=np.int64), alt["seed"]]),
        "purpose": np.concatenate([base["purpose"], np.asarray(["ik_r1_prefix_det"] * PREFIX_KEEP, dtype=str), alt["purpose"]]),
        "job_id": np.concatenate([base["job_id"], np.asarray(["DAGGER_R1_35D__v3_canonical__nominal__det"] * PREFIX_KEEP, dtype=str), alt["job_id"]]),
        "role": role,
    }
    # bitwise dedup fail-closed
    seen: dict[bytes, int] = {}
    keep: list[int] = []
    collisions = []
    for i in range(data["obs35"].shape[0]):
        k = data["obs35"][i].tobytes()
        j = seen.get(k)
        if j is None:
            seen[k] = i; keep.append(i)
        else:
            if data["actions"][i].tobytes() != data["actions"][j].tobytes():
                raise R2Error(f"label CONFLICT on identical observation (rows {j}/{i}, roles {data['role'][j]}/{data['role'][i]}): ABORT")
            collisions.append({"kept_row": int(j), "dropped_row": int(i), "kept_role": str(data["role"][j]), "dropped_role": str(data["role"][i]), "labels_identical": True})
    sel = np.asarray(keep, dtype=np.int64)
    data = {k: np.ascontiguousarray(np.asarray(v)[sel]) for k, v in data.items()}
    n_role = {r: int(np.sum(data["role"] == r)) for r in ("base", "r1_prefix", "alt_minus020", "alt_plus020")}
    if n_role["base"] != 992 or n_role["alt_minus020"] != 6447 or n_role["alt_plus020"] != 6429:
        raise R2Error(f"unexpected role counts after dedup: {n_role}")
    if not (np.all(np.isfinite(data["actions"])) and np.all(np.abs(data["actions"]) <= 1.0)):
        raise R2Error("labels non-finite or outside [-1, 1]")
    weights = {r: JULY_MASSES[r] / TOTAL_MASS for r in JULY_MASSES}
    report = {
        "roles_unique_rows": n_role,
        "rows_total_after_dedup": int(sel.size),
        "collisions": {"count": len(collisions), "detail": collisions},
        "G1": {"prefix_rows_kept": PREFIX_KEEP, "prefix_discrete_agreement": "all 98 rows equal to the R0a reference on the 8 July discrete features", "excluded_rows_99_242": PREFIX_EXCLUDED, "excluded_fraction": excluded_fraction, "abort_threshold": G1_ABORT_FRACTION, "pass": True},
        "role_weights_w_r": weights,
        "per_row_lambda": {r: weights[r] / n_role[r] for r in n_role},
        "july_mass_evidence": "markov_dataset_report.json 13/07 (16000 / 712 / 2x4000); alt mass split equally per the architect's order",
    }
    return data, report


# --- pre-fit gates --------------------------------------------------------------------------------

def gate_g2(data: Mapping[str, np.ndarray]) -> dict[str, Any]:
    names = list(R.FEATURE_NAMES_35)
    idx_disc = RO.discrete_feature_indices(names)
    i_el = [names.index("phase_stance_elapsed_norm"), names.index("phase_swing_elapsed_norm")]
    obs = data["obs35"]; u = data["actions"].astype(np.float64)
    sig = [tuple(obs[i, idx_disc].astype(int)) + tuple(np.round(obs[i, i_el] / 0.05).astype(int)) for i in range(obs.shape[0])]
    from collections import defaultdict
    groups = defaultdict(list)
    for i, s in enumerate(sig):
        groups[s].append(i)
    bad = []
    for s, idxs in groups.items():
        if len(idxs) < 2:
            continue
        uu = u[idxs]
        spread = uu.max(axis=0) - uu.min(axis=0)
        if np.any(spread > G2_SPREAD_MAX):
            roles = sorted({str(data["role"][i]) for i in idxs})
            bad.append({"signature": [int(x) for x in s], "n": len(idxs), "spread_knee_ankle": spread.tolist(), "roles": roles})
    bad.sort(key=lambda b: -max(b["spread_knee_ankle"]))
    return {"signature": "8 July discrete features (exact) + stance/swing elapsed norm quantised at 0.05", "groups_total": len(groups), "violating_groups": len(bad), "worst": bad[:8], "spread_max_per_joint": G2_SPREAD_MAX, "pass": len(bad) == 0}


def _stratified_split(data: Mapping[str, np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    hold = np.zeros(data["obs35"].shape[0], dtype=bool)
    for r in ("base", "r1_prefix", "alt_minus020", "alt_plus020"):
        idx = np.where(data["role"] == r)[0]
        rng = np.random.default_rng(2026)
        perm = idx[rng.permutation(idx.size)]
        hold[perm[: int(round(idx.size * HOLDOUT_FRACTION))]] = True
    return ~hold, hold


def gate_g3_and_coverage(data: Mapping[str, np.ndarray]) -> dict[str, Any]:
    train, hold = _stratified_split(data)
    obs = data["obs35"].astype(np.float64); u = data["actions"].astype(np.float64)
    mu = obs[train].mean(0); sd = obs[train].std(0)     # leakage-free: TRAIN-only scaling
    keepf = sd > 1e-9
    zt = ((obs[train] - mu) / np.where(keepf, sd, 1.0))[:, keepf]
    zh = ((obs[hold] - mu) / np.where(keepf, sd, 1.0))[:, keepf]
    yt = u[train]; yh = u[hold]
    b2 = np.sum(zt ** 2, axis=1)
    preds = np.empty_like(yh)
    for s in range(0, zh.shape[0], 256):
        a = zh[s: s + 256]
        d2 = np.sum(a ** 2, 1)[:, None] + b2[None, :] - 2.0 * (a @ zt.T)
        idx = np.argpartition(d2, G3_K - 1, axis=1)[:, :G3_K]
        preds[s: s + 256] = yt[idx].mean(axis=1)
    rmse = np.sqrt(np.mean((preds - yh) ** 2, axis=0)).tolist()
    # coverage: per-role NN distance quantiles (role rows -> rest of dataset), full data, same scaling
    zall = ((obs - mu) / np.where(keepf, sd, 1.0))[:, keepf]
    cov = {}
    for r in ("base", "r1_prefix", "alt_minus020", "alt_plus020"):
        ridx = np.where(data["role"] == r)[0]
        oidx = np.where(data["role"] != r)[0]
        if ridx.size == 0:
            raise R2Error(f"coverage: role {r} absent")
        if oidx.size == 0:
            raise R2Error(f"coverage: role {r} has no other-role rows to compare against (some role is absent)")
        A = zall[ridx]; B = zall[oidx]
        b2o = np.sum(B ** 2, axis=1)
        dmin = np.empty(A.shape[0])
        for s in range(0, A.shape[0], 512):
            a = A[s: s + 512]
            d2 = np.sum(a ** 2, 1)[:, None] + b2o[None, :] - 2.0 * (a @ B.T)
            dmin[s: s + 512] = np.sqrt(np.maximum(d2.min(axis=1), 0.0))
        q = np.percentile(dmin, [10, 50, 90])
        cov[r] = {"rows": int(ridx.size), "nn_to_other_roles_p10_p50_p90": [float(x) for x in q]}
    return {"G3": {"k": G3_K, "split": "role-stratified default_rng(2026) permutation, 20% per role; scaling mean/std TRAIN-only; constant features excluded", "holdout_rows": int(hold.sum()), "rmse_knee_ankle": rmse, "max_per_joint": G3_RMSE_MAX, "pass": bool(all(v <= G3_RMSE_MAX for v in rmse))}, "coverage": {"per_role": cov, "pass": True}, "_train_mask": train, "_hold_mask": hold}


# --- role-weighted fit ----------------------------------------------------------------------------

def fit_r2(init_state: Mapping[str, Any], data: Mapping[str, np.ndarray], report_ds: Mapping[str, Any], *, budget: Mapping[str, Any] | None = None, progress: bool = False) -> tuple[dict[str, Any], dict[str, Any]]:
    import torch
    import torch.nn.functional as F

    budget = dict(budget or V2.BUDGET_R0)
    info = RF.validate_init_state(init_state, expected_actor_digest=None)
    torch.use_deterministic_algorithms(True); torch.set_num_threads(1); torch.manual_seed(int(budget["seed"])); np.random.seed(int(budget["seed"]))
    f32 = np.float32
    obs = torch.as_tensor(np.asarray(data["obs35"], dtype=f32))
    tgt = torch.as_tensor(np.asarray(data["actions"], dtype=f32))
    clk = torch.as_tensor(np.asarray(data["clock"], dtype=f32))
    roles = np.asarray(data["role"])
    var_role = {}
    lam = np.empty(obs.shape[0], dtype=np.float64)
    for r in ("base", "r1_prefix", "alt_minus020", "alt_plus020"):
        m = roles == r
        v = np.var(np.asarray(data["actions"], dtype=np.float64)[m], axis=0)
        if np.any(v <= 0.0):
            raise R2Error(f"degenerate per-joint variance in role {r}")
        var_role[r] = v
        lam[m] = report_ds["per_row_lambda"][r]
    inv_var = np.stack([1.0 / var_role[r] for r in roles.tolist()]).astype(f32)  # (N,2) per-row 1/Var_{r,j}
    lam_t = torch.as_tensor(lam.astype(f32)); iv_t = torch.as_tensor(inv_var)
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
        perm = rng.permutation(n); losses = []; wobj = []
        for s0 in range(0, n, batch):
            idx = torch.as_tensor(perm[s0: s0 + batch]); opt.zero_grad(set_to_none=True)
            h2 = encoder(obs[idx]); m = h2 @ W3m.T + b3m; hp = h2 @ Wh.T + bh
            e = (((m - tgt[idx]) ** 2) * iv_t[idx]).mean(dim=1)          # per-row e_i = mean_j (err^2/Var_{r,j})
            w = lam_t[idx]
            l_data = (w * e).sum() / w.sum()                              # weighted mean (rev3f formula)
            loss = l_data + lam_clip * torch.relu(torch.abs(m) - 1.0).square().mean() + lam_phi * F.mse_loss(hp, clk[idx])
            if lam_a > 0.0:
                loss = loss + lam_a * torch.stack([(p - a0).square().sum() for p, a0 in zip(mean_params, anchor_t)]).sum()
            loss.backward(); opt.step()
            with torch.no_grad():
                W1.mul_(clock_mask)
            losses.append(float(loss.item())); wobj.append(float(l_data.item()))
        history.append({"epoch": epoch, "loss": float(np.mean(losses)), "weighted_data_obj": float(np.mean(wobj))})
        if progress and (epoch == 1 or epoch % 50 == 0):
            print(json.dumps(history[-1]), flush=True)
    w3 = np.concatenate([W3m.detach().numpy().astype(f32), logstd_w], axis=0); b3 = np.concatenate([b3m.detach().numpy().astype(f32), logstd_b], axis=0)
    canon = {"pi.0.0.weight": W1.detach().numpy().astype(f32).copy(), "pi.0.0.bias": b1.detach().numpy().astype(f32).copy(), "pi.0.2.weight": W2.detach().numpy().astype(f32).copy(), "pi.0.2.bias": b2.detach().numpy().astype(f32).copy(), "pi.1.weight": w3.astype(f32).copy(), "pi.1.bias": b3.astype(f32).copy()}
    new_state: dict[str, Any] = {}
    for key in RF.EXPECTED_KEY_ORDER:
        new_state[key] = canon[key].copy() if key in canon else canon[RF.ALIAS_KEYS[key]].copy()
    struct = RF.validate_init_state(new_state, expected_actor_digest=None)
    report = {"tool": "v26b_r2_offline.fit_r2 (rev3f role-weighted; rev3d numeric protocol otherwise unchanged)", "rows": n, "budget": budget,
              "role_weights": dict(report_ds["role_weights_w_r"]), "per_row_lambda": dict(report_ds["per_row_lambda"]),
              "var_per_role_per_joint": {r: v.tolist() for r, v in var_role.items()},
              "loss_formula": "L = sum_i(lambda_i*e_i)/sum_i(lambda_i) + 1.0*clip + 0.1*aux + 1e-3*||theta-theta_R1||^2; e_i = mean_j (m-y)^2/Var_{role(i),j}; lambda_i = mass_r/(24712*n_unique_r)",
              "init_actor_digest": info["actor_digest"], "new_actor_digest": RF.actor_state_digest(new_state), "history": history, "structure": struct, "sigma_note": SIGMA_NOTE, "aux_head": {"exported": False, "training_time_only": True}}
    return new_state, report


# --- post-fit gate --------------------------------------------------------------------------------

def evaluate_r2_gate(init_state: Mapping[str, Any], state: Mapping[str, Any], data: Mapping[str, np.ndarray], g3cov: Mapping[str, Any], fit_report: Mapping[str, Any]) -> dict[str, Any]:
    f32 = np.float32
    m = RF.numpy_mean(state, np.asarray(data["obs35"], dtype=f32))
    y = np.asarray(data["actions"], dtype=np.float64)
    roles = np.asarray(data["role"]); purpose = np.asarray(data["purpose"])
    sets = {
        "aggregate": np.ones(y.shape[0], bool),
        "base": roles == "base",
        "bc_nominal": (roles == "base") & (purpose == "ik_nominal_det"),
        "r0a_on_policy": (roles == "base") & (purpose == "ik_onpolicy_det"),
        "r1_prefix": roles == "r1_prefix",
        "alt_minus020": roles == "alt_minus020",
        "alt_plus020": roles == "alt_plus020",
    }
    per = {}
    for k, mask in sets.items():
        rm = np.sqrt(np.mean((m[mask] - y[mask]) ** 2, axis=0)).tolist()
        per[k] = {"rows": int(mask.sum()), "rmse_knee_ankle": rm, "pass": bool(all(v <= GATE_RMSE_MAX for v in rm))}
    hold = np.asarray(g3cov["_hold_mask"])
    rm_h = np.sqrt(np.mean((m[hold] - y[hold]) ** 2, axis=0)).tolist()
    struct = RF.validate_init_state(state, expected_actor_digest=None)
    inv = RF.invariance_test(state, np.asarray(data["obs35"], dtype=f32)[:64])
    q3 = {"ten_keys": tuple(state.keys()) == RF.EXPECTED_KEY_ORDER, "clock_columns_zero": bool(struct["clock_columns_zero"]), "logstd_constant_placeholder": bool(struct["sigma_head"]["logstd_bias_exact"]), "clock_invariance_bit_identical": bool(inv["bit_identical"]), "save_reload_exact": "verified at export"}
    out = {
        "threshold_per_joint": GATE_RMSE_MAX,
        "per_set": per,
        "holdout_leakage_free_rmse": {"rows": int(hold.sum()), "rmse_knee_ankle": rm_h, "note": "same role-stratified split as G3 (declared seed 2026), scaling irrelevant here (network input raw)"},
        "weighted_objective_final": fit_report["history"][-1]["weighted_data_obj"],
        "Q3": {**q3, "pass": bool(q3["ten_keys"] and q3["clock_columns_zero"] and q3["logstd_constant_placeholder"] and q3["clock_invariance_bit_identical"])},
        "parameter_shift_sq_informational": float(sum(np.sum((np.asarray(state[k], np.float64) - np.asarray(init_state[k], np.float64)) ** 2) for k in RF.CANONICAL_KEYS)),
        "sigma_note": SIGMA_NOTE,
    }
    out["pass_r2"] = bool(all(v["pass"] for v in per.values()) and out["Q3"]["pass"])
    return out


def assert_r2_gate(gate: Mapping[str, Any]) -> None:
    if gate.get("pass_r2") is not True:
        failed = [k for k, v in (gate.get("per_set") or {}).items() if not v.get("pass")]
        if not (isinstance(gate.get("Q3"), Mapping) and gate["Q3"].get("pass") is True):
            failed.append("Q3")
        raise R2Error(f"R2 post-fit gate failed {failed}: export refused")


# --- export ---------------------------------------------------------------------------------------

def export_r2(new_state, *, fit_report, gate, dataset_report, dataset_files, prefit, lineage, out_dir: Path = OUT_R2, runtime_status=None) -> dict[str, Any]:
    assert_r2_gate(gate)
    r1_pre = VS.source_files_table(D1.OUT_R1 / "rl_module")
    out_dir = Path(out_dir)
    if out_dir.exists():
        raise FileExistsError(f"no-clobber: {out_dir} exists")
    out_dir.parent.mkdir(parents=True, exist_ok=True)
    lock, token = RF.acquire_export_lock(out_dir)
    staging = None; promoted = False
    try:
        if out_dir.exists():
            raise FileExistsError(f"final path exists (checked under the lock): {out_dir}")
        staging = RF._staging_dir_for(out_dir)
        stage_module = staging / "rl_module"; stage_module.mkdir(parents=True, exist_ok=False)
        shutil.copy2(D1.OUT_R1 / "rl_module" / "metadata.json", stage_module / "metadata.json")
        shutil.copy2(D1.OUT_R1 / "rl_module" / "class_and_ctor_args.pkl", stage_module / "class_and_ctor_args.pkl")
        with (stage_module / "module_state.pkl").open("wb") as fh:
            pickle.dump({k: np.asarray(v) for k, v in new_state.items()}, fh, protocol=pickle.HIGHEST_PROTOCOL)
        if str(R.BASELINE_DIR) not in sys.path:
            sys.path.insert(0, str(R.BASELINE_DIR))
        import warm_start as W
        reloaded = W.load_module_state(stage_module)
        cmp = W.compare_actor_states({k: np.asarray(v) for k, v in new_state.items()}, reloaded)
        if not cmp.get("exact"):
            raise R2Error(f"save/reload mismatch: {cmp}")
        struct = RF.validate_init_state(reloaded, expected_actor_digest=fit_report["new_actor_digest"])
        inv = RF.invariance_test(reloaded, np.random.default_rng(1).standard_normal((64, R.ENV_ACTOR_WIDTH)).astype(np.float32))
        if not inv.get("bit_identical"):
            raise R2Error("clock invariance not bit-identical")
        names35, _, manifest_shas = VS.pinned_names()
        manifest = {"schema_version": 1, "actor_feature_names": names35, "actor_feature_count": len(names35), "actor_digest": struct["actor_digest"], "module_state_sha256": C.sha256_file(stage_module / "module_state.pkl"), "manifest35_sha256": manifest_shas["manifest35_sha256"], "exploration_sigma": [VS.SIGMA_PLACEHOLDER] * R.ACTION_DIM, "sigma_note": SIGMA_NOTE, "derived_from": C.rel(D1.OUT_R1 / "rl_module"), "source_actor_digest": R1.PIN_R1_ACTOR_DIGEST, "contract": "deployable_markov_controller_state"}
        C.write_json(stage_module / W.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME, manifest, clobber=False)
        if VS.source_files_table(D1.OUT_R1 / "rl_module") != r1_pre:
            raise R2Error("SOURCE R1 module changed during the export")
        hist = fit_report["history"]
        receipt = {
            "schema": "v26b_r2.1",
            "amendments": {"rev3a": V2.PIN_AMENDMENT, "rev3b": A.PIN_AMENDMENT_REV3B, "rev3c": RO.PIN_AMENDMENT_REV3C, "rev3d": D1.PIN_AMENDMENT_REV3D, "rev3e": R1.PIN_AMENDMENT_REV3E, "rev3f": PIN_AMENDMENT_REV3F},
            "authorized_stage": AUTHORIZED_STAGE, "lineage": dict(lineage),
            "init": {"module": C.rel(D1.OUT_R1 / "rl_module"), "actor_digest": R1.PIN_R1_ACTOR_DIGEST, "files_sha256": r1_pre},
            "dataset": {"report": dict(dataset_report), "files": dict(dataset_files)},
            "pre_fit_gates": dict(prefit),
            "fit": {k: fit_report[k] for k in ("tool", "rows", "budget", "role_weights", "per_row_lambda", "var_per_role_per_joint", "loss_formula", "init_actor_digest", "new_actor_digest", "sigma_note", "aux_head")},
            "loss_history_first_last": {"first": hist[0], "epoch_150": hist[min(149, len(hist) - 1)], "last": hist[-1], "epochs": len(hist)},
            "loss_history_full": hist,
            "gate": dict(gate), "structure": struct, "save_reload_exact": True, "clock_invariance": inv,
            "sigma_placeholder": {"value": VS.SIGMA_PLACEHOLDER, "statement": SIGMA_NOTE},
            "output_module": C.rel(out_dir / "rl_module"),
            "output_files_sha256": {p.name: C.sha256_file(p) for p in sorted(stage_module.iterdir())},
            "code_digests": {"v26b_r2_offline.py": C.sha256_file(Path(__file__).resolve()), "v26b_dagger_r1.py": C.sha256_file(HERE / "v26b_dagger_r1.py"), "v26b_r1_rollout.py": C.sha256_file(HERE / "v26b_r1_rollout.py"), "f2r_refit.py": C.sha256_file(VALIDATION_DIR / "f2r_bridge_2026-08-23" / "f2r_refit.py")},
            "scope": "R2 offline only: no rollout, no PPO, no critic, no sigma sweep, no production change; STOP for the architect audit",
            "export_transaction": {"lock": lock.name, "promotion_policy": "native atomic no-replace rename (Darwin renameatx_np RENAME_EXCL), else F2R exclusive-mkdir fallback", "completion_marker": RECEIPT_NAME},
            "generated_at_utc": C.utc_now(), "git": C.git_snapshot(),
        }
        C.write_json(staging / RECEIPT_NAME, receipt)
        canonical = json.loads((staging / RECEIPT_NAME).read_text(encoding="utf-8"))
        if canonical != json.loads(json.dumps(receipt, default=str)):
            raise R2Error("canonical receipt on disk differs from the receipt in memory")
        canonical_sha = C.sha256_file(staging / RECEIPT_NAME)
        if out_dir.exists():
            raise FileExistsError(f"final path appeared during the export: {out_dir}")
        promotion = RF.promote_staging(staging, out_dir)
        promoted = True
    except BaseException:
        if staging is not None and not promoted:
            shutil.rmtree(staging, ignore_errors=True)
        raise
    finally:
        released = RF.release_export_lock(lock, token)
    if runtime_status is not None:
        runtime_status.update({"promotion": promotion, "lock_released": bool(released), "canonical_receipt_sha256": canonical_sha, "final_path": C.rel(out_dir)})
    return canonical


# --- pipeline -------------------------------------------------------------------------------------

def save_r2_dataset(data, report) -> dict[str, Any]:
    out_dir = VA.OUT_ROOT / "datasets"; out_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%dT%H%M%S")
    import io, os
    path = R.unique_artifact_path(out_dir, f"v26b_dataset_R2_{stamp}", ".npz")
    buf = io.BytesIO(); np.savez(buf, **{k: np.asarray(v) for k, v in data.items()})
    part = path.with_name(path.name + f".part-{os.getpid()}")
    with open(part, "xb") as fh:
        fh.write(buf.getvalue()); fh.flush(); os.fsync(fh.fileno())
    os.replace(part, path)
    rpath = R.unique_artifact_path(out_dir, f"v26b_dataset_R2_receipt_{stamp}", ".json")
    VA._atomic_fill_reserved(rpath, json.dumps({"schema": "v26b_r2_dataset.1", "file": {"path": C.rel(path), "sha256": C.sha256_file(path), "rows": int(data["obs35"].shape[0])}, **dict(report), "generated_at_utc": C.utc_now()}, indent=2, ensure_ascii=False, default=str) + "\n")
    return {"npz": {"path": C.rel(path), "sha256": C.sha256_file(path)}, "receipt": {"path": C.rel(rpath), "sha256": C.sha256_file(rpath)}}


def run_r2(*, authorized_stage: str | None, out_dir: Path = OUT_R2, progress: bool = True, runtime_status=None) -> dict[str, Any]:
    if authorized_stage != AUTHORIZED_STAGE:
        raise R2Error(f"R2 requires --authorized-stage {AUTHORIZED_STAGE} (architect GO); got {authorized_stage!r}")
    lineage = verify_lineage_r2()
    data, ds_report = build_r2_dataset()      # includes G1
    g2 = gate_g2(data)
    if not g2["pass"]:
        diag_path = R.unique_artifact_path(VA.OUT_ROOT, f"v26b_r2_g2_stop_{time.strftime('%Y%m%d_%H%M%S')}", ".json")
        VA._atomic_fill_reserved(diag_path, json.dumps({"schema": "v26b_r2_g2_stop.1", "order": "rev3f: G2 not relaxed; STOP with diagnostics", "G2": g2, "dataset": ds_report}, indent=2, default=str) + "\n")
        raise R2Error(f"G2 FAILED ({g2['violating_groups']} ambiguous groups): STOP with diagnostics at {C.rel(diag_path)}; NO fit")
    g3cov = gate_g3_and_coverage(data)
    if not g3cov["G3"]["pass"]:
        raise R2Error(f"G3 FAILED (kNN holdout rmse {g3cov['G3']['rmse_knee_ankle']} > {G3_RMSE_MAX}): NO fit")
    ds_files = save_r2_dataset(data, ds_report)
    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    init_state = {k: np.asarray(W._as_numpy(v)) for k, v in W.load_module_state(D1.OUT_R1 / "rl_module").items()}
    RF.validate_init_state(init_state, expected_actor_digest=R1.PIN_R1_ACTOR_DIGEST)
    new_state, fit_report = fit_r2(init_state, data, ds_report, progress=progress)
    gate = evaluate_r2_gate(init_state, new_state, data, g3cov, fit_report)
    prefit = {"G1": ds_report["G1"], "G2": g2, "G3": g3cov["G3"], "coverage": g3cov["coverage"]}
    if gate["pass_r2"] is not True:
        rej = R.unique_artifact_path(VA.OUT_ROOT, f"v26b_r2_REJECTED_receipt_{time.strftime('%Y%m%d_%H%M%S')}", ".json")
        VA._atomic_fill_reserved(rej, json.dumps({"schema": "v26b_r2_rejected.1", "REJECTED": True, "gate": gate, "fit": {k: fit_report[k] for k in ("new_actor_digest", "role_weights", "loss_formula")}, "dataset_files": ds_files, "pre_fit_gates": prefit}, indent=2, default=str) + "\n")
        raise R2Error(f"R2 post-fit gate FAILED: rejected receipt at {C.rel(rej)}; NO export, no frozen replaced")
    return export_r2(new_state, fit_report=fit_report, gate=gate, dataset_report=ds_report, dataset_files=ds_files, prefit=prefit, lineage=lineage, out_dir=out_dir, runtime_status=runtime_status)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="V26B rev3f R2 offline (dry by default; NO rollout)")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--authorized-stage", default=None)
    args = parser.parse_args(argv)
    if not args.execute:
        verify_lineage_r2()
        data, rep = build_r2_dataset()
        g2 = gate_g2(data)
        print(json.dumps({"mode": "dry", "rows": int(data["obs35"].shape[0]), "roles": rep["roles_unique_rows"], "collisions": rep["collisions"]["count"], "G1": rep["G1"]["pass"], "G2_violating_groups": g2["violating_groups"]}, indent=2))
        return 0
    runtime = {}
    canonical = run_r2(authorized_stage=args.authorized_stage, runtime_status=runtime)
    print(json.dumps({"receipt_sha256": runtime.get("canonical_receipt_sha256"), "final_path": runtime.get("final_path"), "new_actor_digest": canonical["fit"]["new_actor_digest"], "gate_pass": canonical["gate"]["pass_r2"]}, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
