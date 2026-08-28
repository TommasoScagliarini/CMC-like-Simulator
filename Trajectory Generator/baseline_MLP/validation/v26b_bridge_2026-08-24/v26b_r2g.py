"""V26B rev3g — R2 July-faithful offline stage (flat mass-weighted MSE, July anchor,
July physical scaling with exact R1 function preservation, blocked gates).

Binding rules in ``v26b_amendment_rev3g_r2_july_faithful.json`` (pinned below).
No rollout / PPO / critic / sigma sweep / ex-novo; production untouched; sigma
stays the UNDECIDED frozen placeholder.
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
import f2r_common as R  # noqa: E402
import f2r_refit as RF  # noqa: E402
import v26b_anchors as VA  # noqa: E402  (output root)
import v26b_dagger_r1 as D1  # noqa: E402
import v26b_r0a as A  # noqa: E402
import v26b_r0a_rollout as RO  # noqa: E402
import v26b_r1_rollout as R1  # noqa: E402
import v26b_r2_offline as R2  # noqa: E402
import v26b_student as VS  # noqa: E402
import v26b_v2 as V2  # noqa: E402


class R2GError(RuntimeError):
    pass


AUTHORIZED_STAGE = "V26B-R2G"
AMENDMENT_REV3G = HERE / "v26b_amendment_rev3g_r2_july_faithful.json"
PIN_AMENDMENT_REV3G = "bb4401fc7003d1c8a5c30ed04fe75a1bd5d3fbec555fc91c2f5e681fa12656b5"
MASSES = {"base": 0.6474587245063127, "r1_prefix": 0.028811913240530916, "alt_minus020": 0.16186468112657817, "alt_plus020": 0.16186468112657817}
SCALES_BY_NAME = {"pros_knee_angle_previous_endpoint": 1.0, "pros_knee_angle_served_ref": 1.0, "pros_knee_angle_served_ref_vel": 4.0, "pros_knee_angle_served_ref_accel": 60.0, "pros_knee_angle_sea_u": 1.0,
                  "pros_ankle_angle_previous_endpoint": 1.0, "pros_ankle_angle_served_ref": 1.0, "pros_ankle_angle_served_ref_vel": 3.5, "pros_ankle_angle_served_ref_accel": 55.0, "pros_ankle_angle_sea_u": 1.0}
ANCHOR_WEIGHT_JULY = 0.01
PRESERVATION_TOL = 1e-5
GATE_RMSE_MAX = 0.15
KAPPA_M = 10.0
SF_ENVELOPE = 2.0
OUT_R2G = VA.OUT_ROOT / "student" / "V2_R2G"
RECEIPT_NAME = "v26b_r2g_receipt.json"
SIGMA_NOTE = D1.SIGMA_NOTE
BUDGET = {"epochs": 300, "batch_size": 256, "lr": 1e-4, "seed": 2026, "clip_weight": 1.0}


def scale_vector() -> tuple[np.ndarray, dict[str, Any]]:
    names = list(R.FEATURE_NAMES_35)
    vec = np.ones(35, dtype=np.float64)
    table = {}
    for n, s in SCALES_BY_NAME.items():
        if n not in names:
            raise R2GError(f"scaled feature {n!r} absent from the pinned 35D manifest")
        i = names.index(n)
        vec[i] = float(s)
        table[n] = {"index": i, "scale": float(s)}
    return vec, table


def verify_lineage_r2g() -> dict[str, Any]:
    lin = R2.verify_lineage_r2()  # chain rev3..rev3f + base/alt npz + R1 parent artefacts
    got = C.sha256_file(AMENDMENT_REV3G)
    if got != PIN_AMENDMENT_REV3G:
        raise R2GError(f"rev3g amendment sha {got} != pinned {PIN_AMENDMENT_REV3G}")
    am = json.loads(AMENDMENT_REV3G.read_text(encoding="utf-8"))
    # the amendment pins the PRE-EXISTING dataset builder; verify it has not been edited since
    pinned_builder = am["parents_immutable"]["preexisting_tooling"]["v26b_r2_offline.py (build_r2_dataset + G1)"]
    disk_builder = C.sha256_file(HERE / "v26b_r2_offline.py")
    if disk_builder != pinned_builder:
        raise R2GError(f"v26b_r2_offline.py changed after the rev3g amendment: {disk_builder} != {pinned_builder}")
    hp = am["parents_immutable"]["historical_parent_jul_h0"]
    run = C.REPO / hp["run"]
    for rel, pin in (("adaptation_report.json", hp["adaptation_report_sha256"]), ("run_summary.json", hp["run_summary_sha256"]), ("rl_module_target_adapted/module_state.pkl", hp["module_state_sha256_byte_identical_to_JUL_H0"])):
        got = C.sha256_file(run / rel)
        if got != pin:
            raise R2GError(f"historical parent file {rel} sha {got} != pinned")
    masses = am["exact_masses_correcting_rev3f_text"]
    for k, v in MASSES.items():
        if abs(float(masses[k]) - v) > 1e-15:
            raise R2GError(f"mass {k} mismatch between tool and amendment")
    lin["amendment_rev3g"] = {"path": C.rel(AMENDMENT_REV3G), "sha256": PIN_AMENDMENT_REV3G}
    lin["historical_parent_verified"] = True
    return lin


# --- split (preregistered) ------------------------------------------------------------------------

def preregistered_split(data: Mapping[str, np.ndarray]) -> dict[str, np.ndarray]:
    N = data["obs35"].shape[0]
    roles = np.asarray(data["role"]); jobs = np.asarray(data["job_id"]); t = data["t_pre"]
    rng = np.random.default_rng(2026)
    hold = np.zeros(N, bool); embargo = np.zeros(N, bool)
    held_traj = {}
    for r in ("alt_minus020", "alt_plus020"):
        tra = sorted(set(jobs[roles == r].tolist()))
        held = list(np.asarray(tra)[rng.permutation(len(tra))[:3]])
        held_traj[r] = held
        hold |= np.isin(jobs, held) & (roles == r)
    step = np.zeros(N, int)
    for j in sorted(set(jobs.tolist())):
        mj = jobs == j; o = np.argsort(t[mj]); s = np.empty(mj.sum(), int); s[o] = np.arange(1, mj.sum() + 1); step[mj] = s
    mb = roles == "base"
    hold |= mb & (step >= 201) & (step <= 300)
    embargo |= mb & (((step >= 191) & (step <= 200)) | ((step >= 301) & (step <= 310)))
    mp = roles == "r1_prefix"
    hold |= mp & (step >= 41) & (step <= 60)
    embargo |= mp & (((step >= 36) & (step <= 40)) | ((step >= 61) & (step <= 65)))
    train = ~(hold | embargo)
    return {"train": train, "hold": hold, "embargo": embargo, "step_in_trace": step, "held_trajectories": held_traj}


# --- prefit gates ---------------------------------------------------------------------------------

def _standardise(obs: np.ndarray, train: np.ndarray) -> np.ndarray:
    mu = obs[train].mean(0); sd = obs[train].std(0); keep = sd > 1e-9
    return ((obs - mu) / np.where(keep, sd, 1.0))[:, keep]


def _knn_rmse(z: np.ndarray, u: np.ndarray, train: np.ndarray, mask: np.ndarray, k: int = 5) -> list[float]:
    zt = z[train]; yt = u[train]; b2 = np.sum(zt ** 2, 1)
    zh = z[mask]; yh = u[mask]; preds = np.empty_like(yh)
    for s in range(0, zh.shape[0], 256):
        a = zh[s: s + 256]
        d2 = np.sum(a ** 2, 1)[:, None] + b2[None, :] - 2.0 * a @ zt.T
        idx = np.argpartition(d2, k - 1, 1)[:, :k]
        preds[s: s + 256] = yt[idx].mean(1)
    return np.sqrt(np.mean((preds - yh) ** 2, 0)).tolist()


def prefit_gates(data: Mapping[str, np.ndarray], split: Mapping[str, np.ndarray], ds_report: Mapping[str, Any]) -> dict[str, Any]:
    obs = data["obs35"].astype(np.float64); u = data["actions"].astype(np.float64)
    roles = np.asarray(data["role"]); jobs = np.asarray(data["job_id"]); step = split["step_in_trace"]
    train, hold = split["train"], split["hold"]
    z = _standardise(obs, train)
    # G3-blocked BINDING
    g3 = {"aggregate": {"rows": int(hold.sum()), "rmse": _knn_rmse(z, u, train, hold)}}
    for r in ("base", "r1_prefix", "alt_minus020", "alt_plus020"):
        m = hold & (roles == r)
        g3[r] = {"rows": int(m.sum()), "rmse": _knn_rmse(z, u, train, m)}
    g3_pass = all(all(v <= GATE_RMSE_MAX for v in g3[k]["rmse"]) for k in g3)
    # envelope magnitude BINDING + diagnostics
    zf = _standardise(obs, np.ones(obs.shape[0], bool))  # full-data standardisation for the pair geometry (declared)
    adj_d, adj_du = [], []
    t = data["t_pre"]
    for j in sorted(set(jobs.tolist())):
        mj = np.where(jobs == j)[0]; o = mj[np.argsort(t[mj])]
        adj_d.append(np.linalg.norm(zf[o[1:]] - zf[o[:-1]], axis=1)); adj_du.append(np.abs(u[o[1:]] - u[o[:-1]]))
    adj_d = np.concatenate(adj_d); adj_du = np.concatenate(adj_du)
    slope = adj_du / np.maximum(adj_d[:, None], 1e-9)
    Q99 = np.percentile(slope, 99, axis=0)
    r_star = float(np.percentile(adj_d, 50)); d_max = 2.0 * r_star
    b2 = np.sum(zf ** 2, 1); k = 10
    worst_excess = 0.0; viol_mag = 0; env_viol = 0; pairs_total = 0
    adj_tail = int(np.sum(np.any(adj_du > SF_ENVELOPE * Q99 * adj_d[:, None], axis=1)))
    N = obs.shape[0]
    for s0 in range(0, N, 512):
        a = zf[s0: s0 + 512]
        d2 = np.sum(a ** 2, 1)[:, None] + b2[None, :] - 2.0 * a @ zf.T
        for rl in range(a.shape[0]):
            i = s0 + rl; d2[rl, i] = np.inf
            for jn in np.argpartition(d2[rl], k - 1)[:k]:
                if i >= jn: continue
                d = float(np.sqrt(max(d2[rl, jn], 0.0)))
                if d > d_max: continue
                if jobs[i] == jobs[jn] and abs(int(step[i]) - int(step[jn])) <= 2: continue
                pairs_total += 1
                du = np.abs(u[i] - u[jn]); env = SF_ENVELOPE * Q99 * d
                exc = float(np.max(du / np.maximum(env, 1e-12)))
                if np.any(du > env): env_viol += 1
                if exc > worst_excess: worst_excess = exc
                if exc > KAPPA_M: viol_mag += 1
    # G3 row-random (DIAGNOSTIC only)
    rngd = np.random.default_rng(2026)
    perm = rngd.permutation(N); hr = np.zeros(N, bool); hr[perm[: int(round(N * 0.2))]] = True
    zr = _standardise(obs, ~hr)
    g3_rand = _knn_rmse(zr, u, ~hr, hr)
    # counterfactual 144 post-mismatch rows (must be REJECTED by the blocked gate)
    import f1_dataset as DS
    import f2r_labeller as L
    rows_r1 = json.loads((R1.JOB_DIR / "rollout_policy_trace.json").read_text(encoding="utf-8"))
    obs_r1 = np.asarray([rw["actor_observation_vector_before"] for rw in rows_r1], dtype=np.float64)
    traj = DS.trajectory_from_job(R1.JOB_DIR, expected_width=R.ENV_ACTOR_WIDTH)
    t_r1 = np.asarray(traj["t_pre"], dtype=np.float64)
    cc = L.load_cache(R.OUT_CACHE, "nominal")
    u_r1 = cc.ik_action[cc.lookup(t_r1)].astype(np.float64)
    mu = obs[train].mean(0); sd = obs[train].std(0); keep = sd > 1e-9
    zp = ((obs_r1[98:242] - mu) / np.where(keep, sd, 1.0))[:, keep]
    zt = z[train]; yt = u[train]; b2t = np.sum(zt ** 2, 1)
    preds = np.empty((144, 2))
    for s0 in range(0, 144, 128):
        a = zp[s0: s0 + 128]
        d2 = np.sum(a ** 2, 1)[:, None] + b2t[None, :] - 2.0 * a @ zt.T
        idx = np.argpartition(d2, 4, 1)[:, :5]
        preds[s0: s0 + 128] = yt[idx].mean(1)
    cf = np.sqrt(np.mean((preds - u_r1[98:242]) ** 2, 0)).tolist()
    cf_rejected = bool(max(cf) > GATE_RMSE_MAX)
    out = {
        "G1": ds_report["G1"],
        "G3_blocked_binding": {**g3, "max_per_joint": GATE_RMSE_MAX, "pass": bool(g3_pass), "split": "group-holdout 3/13 per alt start (rng 2026), blocked base[201-300]/prefix[41-60] with embargo 10/5"},
        "envelope_magnitude_binding": {"r_star": r_star, "d_max": d_max, "Q99_knee_ankle": Q99.tolist(), "SF": SF_ENVELOPE, "kappa_m": KAPPA_M, "pairs_considered": pairs_total, "violations_kappa_m": viol_mag, "worst_excess": worst_excess, "pass": bool(viol_mag == 0)},
        "diagnostics_not_binding": {"envelope_violations_SF2": env_viol, "envelope_density": env_viol / max(pairs_total, 1), "adjacent_tail_rate": adj_tail / max(adj_d.size, 1), "excess_ratio_density_vs_adjacent": (env_viol / max(pairs_total, 1)) / max(adj_tail / max(adj_d.size, 1), 1e-12), "G3_row_random_rmse": g3_rand},
        "counterfactual_144_postmismatch": {"knn_vs_sametime_label_rmse": cf, "rejected_by_blocked_gate": cf_rejected},
    }
    out["pass_prefit"] = bool(out["G1"]["pass"] and out["G3_blocked_binding"]["pass"] and out["envelope_magnitude_binding"]["pass"] and cf_rejected)
    return out


# --- scaling transforms + preservation tests ------------------------------------------------------

def transform_state_to_scaled(state: Mapping[str, Any], vec: np.ndarray) -> dict[str, np.ndarray]:
    out = {k: np.asarray(v).copy() for k, v in state.items()}
    for wk in ("pi.0.0.weight", "pi_encoder.0.weight"):
        w = out[wk].astype(np.float64)
        w = w * vec[None, :]
        out[wk] = w.astype(np.float32)
    return out


def transform_W1_to_raw(W1_scaled: np.ndarray, vec: np.ndarray) -> np.ndarray:
    return (W1_scaled.astype(np.float64) / vec[None, :]).astype(np.float32)


def preservation_test(state_raw: Mapping[str, Any], state_scaled: Mapping[str, Any], obs_raw: np.ndarray, vec: np.ndarray) -> float:
    m_raw = RF.numpy_mean(state_raw, obs_raw.astype(np.float32))
    m_scaled = RF.numpy_mean(state_scaled, (obs_raw.astype(np.float64) / vec[None, :]).astype(np.float32))
    return float(np.max(np.abs(m_raw - m_scaled)))


# --- July-faithful fit (scaled space, flat weighted MSE, July anchor, no aux) ---------------------

def fit_r2g(init_scaled: Mapping[str, Any], data: Mapping[str, np.ndarray], split: Mapping[str, np.ndarray], vec: np.ndarray, *, budget: Mapping[str, Any] | None = None, progress: bool = False) -> tuple[dict[str, Any], dict[str, Any]]:
    import torch

    budget = dict(budget or BUDGET)
    train = split["train"]
    roles_all = np.asarray(data["role"])
    obs_raw = np.asarray(data["obs35"], dtype=np.float64)
    obs_scaled = (obs_raw / vec[None, :]).astype(np.float32)
    f32 = np.float32
    obs_t = torch.as_tensor(obs_scaled[train])
    tgt = torch.as_tensor(np.asarray(data["actions"], dtype=f32)[train])
    roles = roles_all[train]
    n_train = {r: int(np.sum(roles == r)) for r in MASSES}
    if any(v == 0 for v in n_train.values()):
        raise R2GError(f"empty role in the train part: {n_train}")
    lam = np.empty(roles.shape[0], dtype=np.float64)
    for r, w in MASSES.items():
        lam[roles == r] = w / n_train[r]
    lam_t = torch.as_tensor(lam.astype(f32))
    torch.use_deterministic_algorithms(True); torch.set_num_threads(1); torch.manual_seed(int(budget["seed"])); np.random.seed(int(budget["seed"]))
    W1 = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.0.0.weight"], dtype=f32))); b1 = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.0.0.bias"], dtype=f32)))
    W2 = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.0.2.weight"], dtype=f32))); b2 = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.0.2.bias"], dtype=f32)))
    W3m = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.1.weight"][: R.ACTION_DIM], dtype=f32))); b3m = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.1.bias"][: R.ACTION_DIM], dtype=f32)))
    mean_params = [W1, b1, W2, b2, W3m, b3m]
    anchor_t = [p.detach().clone() for p in mean_params]  # theta_R1 in the SAME (scaled) parametric space
    clock_mask = torch.ones_like(W1); clock_mask[:, list(R.CLOCK_COLUMNS)] = 0.0
    with torch.no_grad():
        W1.mul_(clock_mask)
    hidden = int(W2.shape[0])
    logstd_w = np.zeros((R.ACTION_DIM, hidden), dtype=f32); logstd_b = np.asarray(np.log(np.full(R.ACTION_DIM, float(R.SIGMA_CONSTANT))), dtype=f32)
    encoder = lambda x: torch.tanh(torch.tanh(x @ W1.T + b1) @ W2.T + b2)  # noqa: E731
    opt = torch.optim.Adam(mean_params, lr=float(budget["lr"]))
    rng = np.random.default_rng(int(budget["seed"]))
    n = int(obs_t.shape[0]); epochs = int(budget["epochs"]); batch = int(budget["batch_size"]); lam_clip = float(budget["clip_weight"])
    history = []
    for epoch in range(1, epochs + 1):
        perm = rng.permutation(n); losses = []; dobj = []
        for s0 in range(0, n, batch):
            idx = torch.as_tensor(perm[s0: s0 + batch]); opt.zero_grad(set_to_none=True)
            m = encoder(obs_t[idx]) @ W3m.T + b3m
            e = ((m - tgt[idx]) ** 2).mean(dim=1)                      # FLAT per-row MSE (no variance normalisation)
            w = lam_t[idx]
            l_data = (w * e).sum() / w.sum()
            anchor_loss = torch.stack([(p - a0).square().mean() for p, a0 in zip(mean_params, anchor_t)]).sum()  # July semantics
            loss = l_data + lam_clip * torch.relu(torch.abs(m) - 1.0).square().mean() + ANCHOR_WEIGHT_JULY * anchor_loss
            loss.backward(); opt.step()
            with torch.no_grad():
                W1.mul_(clock_mask)
            losses.append(float(loss.item())); dobj.append(float(l_data.item()))
        history.append({"epoch": epoch, "loss": float(np.mean(losses)), "weighted_flat_mse": float(np.mean(dobj))})
        if progress and (epoch == 1 or epoch % 50 == 0):
            print(json.dumps(history[-1]), flush=True)
    w3 = np.concatenate([W3m.detach().numpy().astype(f32), logstd_w], axis=0); b3 = np.concatenate([b3m.detach().numpy().astype(f32), logstd_b], axis=0)
    scaled_fit = {"pi.0.0.weight": W1.detach().numpy().astype(f32).copy(), "pi.0.0.bias": b1.detach().numpy().astype(f32).copy(), "pi.0.2.weight": W2.detach().numpy().astype(f32).copy(), "pi.0.2.bias": b2.detach().numpy().astype(f32).copy(), "pi.1.weight": w3.astype(f32).copy(), "pi.1.bias": b3.astype(f32).copy()}
    report = {"tool": "v26b_r2g.fit_r2g (rev3g July-faithful: flat mass-weighted MSE, July anchor 0.01*sum(mean_t), no aux, scaled space)", "rows_train": n, "budget": budget, "masses": MASSES, "lambda_per_row_by_role": {r: MASSES[r] / n_train[r] for r in MASSES}, "n_train_by_role": n_train, "history": history, "sigma_note": SIGMA_NOTE}
    return scaled_fit, report


def export_state_from_scaled(scaled_fit: Mapping[str, np.ndarray], vec: np.ndarray) -> dict[str, np.ndarray]:
    W1_raw = transform_W1_to_raw(np.asarray(scaled_fit["pi.0.0.weight"]), vec)
    canon = {"pi.0.0.weight": W1_raw, "pi.0.0.bias": np.asarray(scaled_fit["pi.0.0.bias"]).copy(), "pi.0.2.weight": np.asarray(scaled_fit["pi.0.2.weight"]).copy(), "pi.0.2.bias": np.asarray(scaled_fit["pi.0.2.bias"]).copy(), "pi.1.weight": np.asarray(scaled_fit["pi.1.weight"]).copy(), "pi.1.bias": np.asarray(scaled_fit["pi.1.bias"]).copy()}
    out: dict[str, np.ndarray] = {}
    for key in RF.EXPECTED_KEY_ORDER:
        out[key] = canon[key].copy() if key in canon else canon[RF.ALIAS_KEYS[key]].copy()
    return out


# --- postfit --------------------------------------------------------------------------------------

def postfit_gates(init_raw: Mapping[str, Any], export_raw: Mapping[str, Any], scaled_fit: Mapping[str, Any], data: Mapping[str, np.ndarray], split: Mapping[str, np.ndarray], vec: np.ndarray, t1_maxabs: float) -> dict[str, Any]:
    f32 = np.float32
    obs_raw = np.asarray(data["obs35"], dtype=np.float64)
    u = np.asarray(data["actions"], dtype=np.float64)
    roles = np.asarray(data["role"]); train, hold = split["train"], split["hold"]
    m_exp = RF.numpy_mean(export_raw, obs_raw.astype(f32))
    m_scaled = RF.numpy_mean(scaled_fit, (obs_raw / vec[None, :]).astype(f32))
    t2_maxabs = float(np.max(np.abs(m_exp - m_scaled)))
    def rmse(mask):
        return np.sqrt(np.mean((m_exp[mask] - u[mask]) ** 2, axis=0)).tolist()
    sets = {"aggregate": np.ones(u.shape[0], bool), "base": roles == "base", "r1_prefix": roles == "r1_prefix", "alt_minus020": roles == "alt_minus020", "alt_plus020": roles == "alt_plus020"}
    holdout = {k: {"rows": int((hold & m).sum()), "rmse": rmse(hold & m), "pass": bool(all(v <= GATE_RMSE_MAX for v in rmse(hold & m)))} for k, m in sets.items()}
    traincorp = {k: {"rows": int((train & m).sum()), "rmse": rmse(train & m)} for k, m in sets.items()}
    struct = RF.validate_init_state(export_raw, expected_actor_digest=None)
    inv = RF.invariance_test(export_raw, obs_raw.astype(f32)[:64])
    q3 = {"ten_keys": tuple(export_raw.keys()) == RF.EXPECTED_KEY_ORDER, "clock_columns_zero": bool(struct["clock_columns_zero"]), "logstd_constant_placeholder": bool(struct["sigma_head"]["logstd_bias_exact"]), "clock_invariance_bit_identical": bool(inv["bit_identical"]), "no_critic": True, "save_reload_exact": "verified at export"}
    out = {
        "holdout_leakage_controlled_BINDING": {**holdout, "max_per_joint": GATE_RMSE_MAX, "note": "rows NEVER seen by the fit (train excluded holdout+embargo)"},
        "fit_corpus_train_metrics_distinct": traincorp,
        "function_preservation": {"T1_prefit_maxabs": t1_maxabs, "T2_export_maxabs": t2_maxabs, "tol": PRESERVATION_TOL, "pass": bool(t1_maxabs <= PRESERVATION_TOL and t2_maxabs <= PRESERVATION_TOL)},
        "Q3": {**q3, "pass": bool(all(v is True for k, v in q3.items() if isinstance(v, bool)))},
        "actor_digest_new": struct["actor_digest"],
        "parameter_shift_sq_vs_R1_raw": float(sum(np.sum((np.asarray(export_raw[k], np.float64) - np.asarray(init_raw[k], np.float64)) ** 2) for k in RF.CANONICAL_KEYS)),
        "sigma_note": SIGMA_NOTE,
    }
    out["pass_postfit"] = bool(all(holdout[k]["pass"] for k in holdout) and out["Q3"]["pass"] and out["function_preservation"]["pass"])
    return out


def provenance_block() -> dict[str, Any]:
    """Full amendments/provenance block, buildable BEFORE any fit (anti-recurrence for the
    2026-08-24 AttributeError: every constant is referenced from its OWNING module and
    validated as a 64-hex digest here)."""
    block = {"rev3a": V2.PIN_AMENDMENT, "rev3b": A.PIN_AMENDMENT_REV3B, "rev3c": RO.PIN_AMENDMENT_REV3C, "rev3d": D1.PIN_AMENDMENT_REV3D, "rev3e": R1.PIN_AMENDMENT_REV3E, "rev3f": R2.PIN_AMENDMENT_REV3F, "rev3g": PIN_AMENDMENT_REV3G}
    for k, v in block.items():
        if not (isinstance(v, str) and len(v) == 64 and all(c in "0123456789abcdef" for c in v)):
            raise R2GError(f"provenance block: {k} is not a 64-hex digest ({v!r})")
    return block


# --- pipeline -------------------------------------------------------------------------------------

def run_r2g(*, authorized_stage: str | None, out_dir: Path = OUT_R2G, progress: bool = True, runtime_status=None) -> dict[str, Any]:
    if authorized_stage != AUTHORIZED_STAGE:
        raise R2GError(f"R2G requires --authorized-stage {AUTHORIZED_STAGE} (architect GO); got {authorized_stage!r}")
    lineage = verify_lineage_r2g()
    provenance = provenance_block()  # built BEFORE any fit (fails fast on any broken constant)
    vec, scale_table = scale_vector()
    data, ds_report = R2.build_r2_dataset()
    split = preregistered_split(data)
    prefit = prefit_gates(data, split, ds_report)
    if not prefit["pass_prefit"]:
        diag = R.unique_artifact_path(VA.OUT_ROOT, f"v26b_r2g_prefit_stop_{time.strftime('%Y%m%d_%H%M%S')}", ".json")
        VA._atomic_fill_reserved(diag, json.dumps({"schema": "v26b_r2g_prefit_stop.1", "prefit": prefit}, indent=2, default=str) + "\n")
        raise R2GError(f"prefit binding gate FAILED: STOP without fit/export; diagnostics at {C.rel(diag)}")
    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    init_raw = {k: np.asarray(W._as_numpy(v)) for k, v in W.load_module_state(D1.OUT_R1 / "rl_module").items()}
    RF.validate_init_state(init_raw, expected_actor_digest=R1.PIN_R1_ACTOR_DIGEST)
    init_scaled = transform_state_to_scaled(init_raw, vec)
    t1 = preservation_test(init_raw, init_scaled, np.asarray(data["obs35"]), vec)  # mandatory prefit T1 on ALL corpus rows
    if t1 > PRESERVATION_TOL:
        raise R2GError(f"T1 function-preservation FAILED: max abs {t1:.3e} > {PRESERVATION_TOL}")
    scaled_fit, fit_report = fit_r2g(init_scaled, data, split, vec, progress=progress)
    export_raw = export_state_from_scaled(scaled_fit, vec)
    post = postfit_gates(init_raw, export_raw, scaled_fit, data, split, vec, t1)
    if not post["pass_postfit"]:
        rej = R.unique_artifact_path(VA.OUT_ROOT, f"v26b_r2g_REJECTED_{time.strftime('%Y%m%d_%H%M%S')}", ".json")
        VA._atomic_fill_reserved(rej, json.dumps({"schema": "v26b_r2g_rejected.1", "REJECTED": True, "postfit": post, "prefit": prefit, "fit": {k: fit_report[k] for k in ("masses", "n_train_by_role")}}, indent=2, default=str) + "\n")
        raise R2GError(f"postfit gate FAILED: rejected artefact at {C.rel(rej)}; NO export")
    # export (transactional, template = R1 module)
    r1_pre = VS.source_files_table(D1.OUT_R1 / "rl_module")
    out_dir = Path(out_dir)
    if out_dir.exists():
        raise FileExistsError(f"no-clobber: {out_dir} exists")
    out_dir.parent.mkdir(parents=True, exist_ok=True)
    lock, token = RF.acquire_export_lock(out_dir)
    staging = None; promoted = False
    try:
        if out_dir.exists():
            raise FileExistsError("final path exists (under lock)")
        staging = RF._staging_dir_for(out_dir)
        stage_module = staging / "rl_module"; stage_module.mkdir(parents=True, exist_ok=False)
        shutil.copy2(D1.OUT_R1 / "rl_module" / "metadata.json", stage_module / "metadata.json")
        shutil.copy2(D1.OUT_R1 / "rl_module" / "class_and_ctor_args.pkl", stage_module / "class_and_ctor_args.pkl")
        with (stage_module / "module_state.pkl").open("wb") as fh:
            pickle.dump({k: np.asarray(v) for k, v in export_raw.items()}, fh, protocol=pickle.HIGHEST_PROTOCOL)
        reloaded = W.load_module_state(stage_module)
        cmp = W.compare_actor_states({k: np.asarray(v) for k, v in export_raw.items()}, reloaded)
        if not cmp.get("exact"):
            raise R2GError(f"save/reload mismatch: {cmp}")
        struct = RF.validate_init_state(reloaded, expected_actor_digest=post["actor_digest_new"])
        smoke = RF.numpy_mean(reloaded, np.asarray(data["obs35"], dtype=np.float32)[:16])  # export-space inference smoke test
        if not np.all(np.isfinite(smoke)):
            raise R2GError("export-space inference smoke test produced non-finite outputs")
        names35, _, manifest_shas = VS.pinned_names()
        manifest = {"schema_version": 1, "actor_feature_names": names35, "actor_feature_count": 35, "actor_digest": struct["actor_digest"], "module_state_sha256": C.sha256_file(stage_module / "module_state.pkl"), "manifest35_sha256": manifest_shas["manifest35_sha256"], "exploration_sigma": [VS.SIGMA_PLACEHOLDER] * R.ACTION_DIM, "sigma_note": SIGMA_NOTE, "derived_from": C.rel(D1.OUT_R1 / "rl_module"), "source_actor_digest": R1.PIN_R1_ACTOR_DIGEST, "contract": "deployable_markov_controller_state", "note_scaling": "July physical scaling absorbed at export: the module consumes RAW physical observations"}
        C.write_json(stage_module / W.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME, manifest, clobber=False)
        if VS.source_files_table(D1.OUT_R1 / "rl_module") != r1_pre:
            raise R2GError("SOURCE R1 module changed during the export")
        hist = fit_report["history"]
        receipt = {
            "schema": "v26b_r2g.1",
            "amendments": provenance_block(),
            "authorized_stage": AUTHORIZED_STAGE, "lineage": {k: v for k, v in lineage.items() if not isinstance(v, np.ndarray)},
            "init": {"module": C.rel(D1.OUT_R1 / "rl_module"), "actor_digest": R1.PIN_R1_ACTOR_DIGEST, "files_sha256": r1_pre},
            "scaling": {"table": scale_table, "preservation": {"T1_maxabs": post["function_preservation"]["T1_prefit_maxabs"], "T2_maxabs": post["function_preservation"]["T2_export_maxabs"], "tol": PRESERVATION_TOL}},
            "dataset": {"report": dict(ds_report), "split": {"train": int(split["train"].sum()), "hold": int(split["hold"].sum()), "embargo": int(split["embargo"].sum()), "held_trajectories": split["held_trajectories"]}},
            "prefit_gates": prefit,
            "fit": {k: fit_report[k] for k in ("tool", "rows_train", "budget", "masses", "lambda_per_row_by_role", "n_train_by_role", "sigma_note")},
            "loss_history_first_last": {"first": hist[0], "epoch_150": hist[min(149, len(hist) - 1)], "last": hist[-1]},
            "loss_history_full": hist,
            "postfit_gates": post,
            "structure": struct, "save_reload_exact": True,
            "sigma_placeholder": {"value": VS.SIGMA_PLACEHOLDER, "statement": SIGMA_NOTE},
            "output_module": C.rel(out_dir / "rl_module"),
            "output_files_sha256": {p.name: C.sha256_file(p) for p in sorted(stage_module.iterdir())},
            "code_digests": {"v26b_r2g.py": C.sha256_file(Path(__file__).resolve()), "test_v26b_r2g.py": C.sha256_file(HERE / "test_v26b_r2g.py") if (HERE / "test_v26b_r2g.py").is_file() else None, "v26b_r2_offline.py": C.sha256_file(HERE / "v26b_r2_offline.py")},
            "scope": "R2G offline only: no rollout, no PPO, no critic, no sigma sweep, no production change; STOP for the architect audit",
            "export_transaction": {"lock": lock.name, "promotion_policy": "native atomic no-replace rename (Darwin renameatx_np RENAME_EXCL), else F2R exclusive-mkdir fallback", "completion_marker": RECEIPT_NAME},
            "generated_at_utc": C.utc_now(), "git": C.git_snapshot(),
        }
        C.write_json(staging / RECEIPT_NAME, receipt)
        canonical = json.loads((staging / RECEIPT_NAME).read_text(encoding="utf-8"))
        if canonical != json.loads(json.dumps(receipt, default=str)):
            raise R2GError("canonical receipt differs from memory")
        canonical_sha = C.sha256_file(staging / RECEIPT_NAME)
        if out_dir.exists():
            raise FileExistsError("final path appeared during export")
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


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="V26B rev3g R2 July-faithful (dry by default; NO rollout)")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--authorized-stage", default=None)
    args = parser.parse_args(argv)
    if not args.execute:
        verify_lineage_r2g()
        vec, table = scale_vector()
        data, rep = R2.build_r2_dataset()
        split = preregistered_split(data)
        print(json.dumps({"mode": "dry", "rows": int(data["obs35"].shape[0]), "train": int(split["train"].sum()), "hold": int(split["hold"].sum()), "scales": {k: v["scale"] for k, v in table.items()}}, indent=2))
        return 0
    runtime = {}
    canonical = run_r2g(authorized_stage=args.authorized_stage, runtime_status=runtime)
    print(json.dumps({"receipt_sha256": runtime.get("canonical_receipt_sha256"), "final_path": runtime.get("final_path"), "new_actor_digest": canonical["postfit_gates"]["actor_digest_new"], "postfit_pass": canonical["postfit_gates"]["pass_postfit"]}, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
