"""V26B rev4b — one July-faithful DAgger round initialised exclusively from S1A.

Token V26B-REV4B-JULY-DAGGER.  Replicates the verified July 11/07 protocol: aggregate =
concat(500-row BC-IK teacher corpus, tile(S1A 392-row on-policy prefix, 4)) with time-aligned
teacher labels, no truncation and no dedup; loss = MSE + 1.0*clip + 0.1*logstd + 1e-5*anchor;
Adam lr 3e-4, batch 64, up to 400 epochs with early stopping on a 20% validation split and
patience 60, seed 123, NO gradient clipping, logstd restored and clock columns zeroed after every
step.  No phase auxiliary head, no hinge, no S0D preservation.

Blocking preflight first; the fit runs only on GO; a single deterministic nominal rollout runs
automatically only if every binding offline gate passes.  No new collection, no retry, no PPO,
no sigma choice, no promotion.
"""

from __future__ import annotations

import argparse
import json
import pickle
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import v26b_l20_rollout as LR  # noqa: E402       (rev4a lineage, unmodified)
import v26b_s1a_rollout as SA  # noqa: E402       (frozen S1A trace + gate semantics)
import v26b_s1a_bc as A  # noqa: E402             (S1A module + flags)
import v26b_s1_prereg as S1  # noqa: E402         (frozen BC-IK 500-row view)
import v26b_s1c_protocol as SC  # noqa: E402      (decode)
import v26b_s1b_rollout as SB  # noqa: E402       (gate helpers + prose scanner)
import v26b_s0d_rollout as SR  # noqa: E402
import v26b_r0a_rollout as RO  # noqa: E402
import v26b_r1_rollout as R1  # noqa: E402
import v26b_r2i_rollout_postrun_audit as PA  # noqa: E402
import v26b_r2g as G  # noqa: E402
import v26b_anchors as VA  # noqa: E402
import v26b_student as VS  # noqa: E402
import f0_common as C  # noqa: E402
import f1_common as F1  # noqa: E402
import f1_dataset as DS  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_refit as RF  # noqa: E402


class Rev4bError(RuntimeError):
    pass


AUTHORIZED_STAGE = "V26B-REV4B-JULY-DAGGER"
AMENDMENT_REV4B = HERE / "v26b_amendment_rev4b_july_dagger_s1a.json"
PIN_AMENDMENT_REV4B = "883ffc9a0217fa6fefcabf820b382717b8ad2e8f7ccd5814461b0bb4218f0f93"

S1A_MODULE = A.OUT_S1A / "rl_module"
PIN_S1A_ACTOR = A.P0_ACTOR_DIGEST
PIN_S1A_TRACE = "6546befcd4a2e26711137a807cd43a797e47abd62500243fe6c015d7abcfcf21"
PREFIX_ROWS = 392

# July values, verified from source (rev4b JULY_VALUES_VERIFIED_FROM_SOURCE_NOT_MEMORY)
J_EPOCHS = 400
J_BATCH = 64
J_LR = 3e-4
J_VAL_FRACTION = 0.20
J_PATIENCE = 60
J_CLIP_W = 1.0
J_LOGSTD_W = 0.1
J_ANCHOR_W = 1e-5
J_SEED = 123
J_TRACE_REPEAT = 4

OUT_CAND = VA.OUT_ROOT / "candidates" / "REV4B_JULY_DAGGER_35D_NONDEPLOYABLE"
JOB_DIR = VA.OUT_ROOT / "rollouts" / "rev4b_nominal_det" / "REV4B_35D__v3_canonical__nominal__det"
RECEIPT_NAME = "v26b_rev4b_receipt.json"
ROLLOUT_RECEIPT_NAME = "v26b_rev4b_rollout_receipt.json"
LOG_NAME = "rev4b_nominal_det_rollout.log"
PASS_STATUS = "CLOSED_LOOP_ELIGIBLE_PENDING_ARCHITECT_AUDIT"
FAIL_STATUS = "CLOSED_LOOP_FAIL_QUARANTINED"
B3_REFERENCE_THRESHOLD = -0.03


def _amendment() -> dict[str, Any]:
    got = C.sha256_file(AMENDMENT_REV4B)
    if got != PIN_AMENDMENT_REV4B:
        raise Rev4bError(f"rev4b sha {got} != pinned")
    return json.loads(AMENDMENT_REV4B.read_text(encoding="utf-8"))


def verify_lineage() -> dict[str, Any]:
    lin = LR.verify_lineage()
    got = C.sha256_file(AMENDMENT_REV4B)
    if got != PIN_AMENDMENT_REV4B:
        raise Rev4bError(f"rev4b sha {got} != pinned")
    lin["amendment_rev4b"] = got
    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    d = W.actor_state_digest(W.load_module_state(S1A_MODULE))
    if d != PIN_S1A_ACTOR:
        raise Rev4bError(f"S1A actor digest {d} != pinned")
    lin["s1a_actor_digest"] = d
    lin["l20_not_promoted"] = "L20 stays CLOSED_LOOP_ELIGIBLE_PENDING_ARCHITECT_AUDIT and is not init, source or anchor here"
    return lin


# --- blocking preflight ------------------------------------------------------------------------------

def preflight() -> dict[str, Any]:
    """Every item is fail-closed: a failure raises and NO fit is run."""
    view = S1.build_s1_view()
    obs500 = np.asarray(view["obs"], dtype=np.float32)
    u_ik500 = np.asarray(view["u_ik"], dtype=np.float64)
    traj = DS.trajectory_from_job(SA.JOB_DIR, expected_width=R.ENV_ACTOR_WIDTH)
    if traj["trace_sha256"] != PIN_S1A_TRACE:
        raise Rev4bError(f"S1A trace digest {traj['trace_sha256']} != pinned")
    rows = json.loads((SA.JOB_DIR / "rollout_policy_trace.json").read_text(encoding="utf-8"))
    n = len(rows)
    if n != PREFIX_ROWS:
        raise Rev4bError(f"S1A prefix has {n} rows, expected {PREFIX_ROWS}")
    steps = [int(r["step"]) for r in rows]
    if steps != list(range(1, n + 1)):
        raise Rev4bError("the S1A step sequence is not contiguous 1..N (July fail-closed rule)")
    obsS = np.asarray(traj["obs35"], dtype=np.float32)
    if obsS.shape != (n, len(R.FEATURE_NAMES_35)) or not np.all(np.isfinite(obsS)):
        raise Rev4bError("S1A obs35 malformed or non-finite")
    for i, r in enumerate(rows):
        rec = np.asarray(r["actor_observation_vector_before"], dtype=np.float32).reshape(-1)
        if not np.array_equal(rec, obsS[i]):
            raise Rev4bError(f"row {i + 1}: obs35 differs from actor_observation_vector_before")
    tS = np.asarray(traj["t_pre"], dtype=np.float64)
    t500 = np.asarray(DS.trajectory_from_job(SR.JOB_DIR, expected_width=R.ENV_ACTOR_WIDTH)["t_pre"], dtype=np.float64)
    dt = float(np.abs(tS - t500[:n]).max())
    if dt != 0.0:
        raise Rev4bError(f"time alignment broken: max |t_S1A - t_corpus| = {dt}")
    if n > len(u_ik500):
        raise Rev4bError("the prefix exceeds the teacher horizon")
    labels = u_ik500[:n].copy()                       # time-aligned teacher, teacher_index = step - 1
    k500 = [obs500[i].tobytes() for i in range(len(obs500))]
    kS = [obsS[i].tobytes() for i in range(n)]
    dup_c = len(k500) - len(set(k500))
    dup_p = len(kS) - len(set(kS))
    m500: dict[bytes, np.ndarray] = {}
    for i, k in enumerate(k500):
        m500.setdefault(k, u_ik500[i])
    collisions = [i for i, k in enumerate(kS) if k in m500]
    conflicts = [i for i in collisions if not np.array_equal(m500[kS[i]], labels[i])]
    if conflicts:
        raise Rev4bError(f"{len(conflicts)} collisions carry CONFLICTING labels: fail-closed")
    q_ik = SC.decode_action(u_ik500)[:, 1]
    neg = q_ik < 0.0
    neg_prefix = neg[:n]
    wins, s = [], None
    for i, f in enumerate(neg_prefix):
        if f and s is None:
            s = i
        if not f and s is not None:
            wins.append([s + 1, i]); s = None
    if s is not None:
        wins.append([s + 1, n])
    if int(neg_prefix.sum()) == 0:
        raise Rev4bError("the prefix covers no negative-ankle-target row: sign recovery would be unaddressable")
    init_state = _load_s1a()
    logstd_ref = {k: np.asarray(init_state[k])[R.ACTION_DIM:] for k in ("pi.1.weight", "pi.1.bias")}
    struct = RF.validate_init_state(init_state, expected_actor_digest=PIN_S1A_ACTOR)
    if not struct["sigma_head"]["logstd_bias_exact"]:
        raise Rev4bError("the S1A logstd head is not the frozen placeholder")
    return {
        "verdict": "GO",
        "s1a_trace": {"sha256": traj["trace_sha256"], "rows": n, "steps_contiguous": True,
                      "obs35_finite": True, "obs35_matches_recorded_vectors": True},
        "time_alignment": {"max_abs_difference_s": dt, "rule": "teacher_index = step - 1, time-aligned prescribed teacher"},
        "labels": {"source": "u_IK AB06 same-time from the pinned nominal cache", "rows": int(n),
                   "not_healthy_symmetry": True, "not_time_shifted": True,
                   "sha256": __import__("hashlib").sha256(labels.tobytes()).hexdigest()},
        "dedup_and_collisions": {"exact_duplicates_corpus": dup_c, "exact_duplicates_prefix": dup_p,
                                 "bitwise_collisions_corpus_prefix": len(collisions),
                                 "collisions_with_conflicting_labels": len(conflicts),
                                 "july_rule": "July performed NO dedup; this is an AUDIT, and only a label conflict is fail-closed"},
        "composition": {"teacher_corpus_rows": int(len(obs500)), "prefix_rows": int(n),
                        "trace_repeat": J_TRACE_REPEAT,
                        "aggregate_rows": int(len(obs500) + n * J_TRACE_REPEAT),
                        "on_policy_share": float(n * J_TRACE_REPEAT / (len(obs500) + n * J_TRACE_REPEAT)),
                        "july_on_policy_share": 0.352,
                        "no_s0d_l20_a2_data": True},
        "negative_target_coverage": {
            "corpus_negative_rows": int(neg.sum()), "prefix_negative_rows": int(neg_prefix.sum()),
            "prefix_negative_windows": wins,
            "alignment_permits_sign_recovery": True,
            "proof": ("the labels on those rows are the teacher's own NEGATIVE ankle commands "
                      f"(minimum {float(q_ik[:n][neg_prefix].min()):.5f} rad decoded), delivered at the exact same time index as the "
                      "on-policy observation, so a network that fits them reproduces a negative command on states the actor actually reaches")},
        "logstd": {"byte_identical_placeholder": True,
                   "reference_sha256": __import__("hashlib").sha256(np.concatenate([logstd_ref["pi.1.weight"].ravel(), logstd_ref["pi.1.bias"].ravel()]).tobytes()).hexdigest()},
        "sigma": "UNRESOLVED and not chosen",
        "_view": view, "_obsS": obsS, "_labels": labels, "_init": init_state, "_neg": neg,
    }


def _load_s1a() -> dict[str, Any]:
    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    st = {k: np.asarray(W._as_numpy(v)) for k, v in W.load_module_state(S1A_MODULE).items()}
    d = RF.validate_init_state(st, expected_actor_digest=PIN_S1A_ACTOR)["actor_digest"]
    if d != PIN_S1A_ACTOR:
        raise Rev4bError("init is not the pinned S1A actor")
    return st


def build_aggregate(pre: Mapping[str, Any]) -> dict[str, np.ndarray]:
    """July aggregate_dagger_traces: concat(teacher, tile(visited, repeat)); no truncation, no dedup."""
    view = pre["_view"]
    obs_t = np.asarray(view["obs"], dtype=np.float32)
    act_t = np.asarray(view["u_ik"], dtype=np.float32)
    obs_v = np.asarray(pre["_obsS"], dtype=np.float32)
    act_v = np.asarray(pre["_labels"], dtype=np.float32)
    return {"observations": np.concatenate([obs_t, np.tile(obs_v, (J_TRACE_REPEAT, 1))], axis=0),
            "actions": np.concatenate([act_t, np.tile(act_v, (J_TRACE_REPEAT, 1))], axis=0)}


# --- July fit ------------------------------------------------------------------------------------------

def fit_july(init_scaled: Mapping[str, Any], obs_scaled: np.ndarray, targets: np.ndarray,
             *, progress: bool = False) -> tuple[dict[str, Any], dict[str, Any]]:
    import torch

    f32 = np.float32
    torch.use_deterministic_algorithms(True); torch.set_num_threads(1)
    torch.manual_seed(J_SEED); np.random.seed(J_SEED)
    X = torch.as_tensor(obs_scaled.astype(f32)); Y = torch.as_tensor(targets.astype(f32))
    n = int(X.shape[0])
    rng = np.random.default_rng(J_SEED)
    perm = rng.permutation(n)
    n_val = int(round(J_VAL_FRACTION * n))
    val_idx = torch.as_tensor(perm[:n_val]); tr_idx = perm[n_val:]
    P = lambda k: torch.nn.Parameter(torch.as_tensor(np.array(init_scaled[k], dtype=f32)))  # noqa: E731
    W1, b1, W2, b2 = P("pi.0.0.weight"), P("pi.0.0.bias"), P("pi.0.2.weight"), P("pi.0.2.bias")
    W3 = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.1.weight"], dtype=f32)))
    b3 = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.1.bias"], dtype=f32)))
    params = [W1, b1, W2, b2, W3, b3]
    anchor = [p.detach().clone() for p in params]
    src_logstd_w = W3.detach()[R.ACTION_DIM:].clone(); src_logstd_b = b3.detach()[R.ACTION_DIM:].clone()
    clock_mask = torch.ones_like(W1); clock_mask[:, list(R.CLOCK_COLUMNS)] = 0.0
    with torch.no_grad():
        W1.mul_(clock_mask)
    fwd = lambda x: torch.tanh(torch.tanh(x @ W1.T + b1) @ W2.T + b2) @ W3.T + b3  # noqa: E731
    opt = torch.optim.Adam(params, lr=J_LR)
    # July: source_logstd is the SOURCE module's log-std output on the same observations,
    # precomputed once and used as a distillation target (here it is the frozen placeholder,
    # constant because the log-std weight rows are zero, so the term is identically zero).
    with torch.no_grad():
        source_logstd = fwd(X)[:, R.ACTION_DIM:].detach().clone()

    def restore_logstd():
        with torch.no_grad():
            W3[R.ACTION_DIM:] = src_logstd_w
            b3[R.ACTION_DIM:] = src_logstd_b

    def val_loss() -> float:
        with torch.no_grad():
            m = fwd(X[val_idx])[:, : R.ACTION_DIM]
            return float(((m - Y[val_idx]) ** 2).mean().item())

    best = val_loss(); best_state = [p.detach().clone() for p in params]; best_epoch = 0; bad = 0
    hist = []
    for epoch in range(1, J_EPOCHS + 1):
        order = rng.permutation(tr_idx)
        losses = []
        for s0 in range(0, len(order), J_BATCH):
            idx = torch.as_tensor(order[s0: s0 + J_BATCH]); opt.zero_grad(set_to_none=True)
            logits = fwd(X[idx])
            means = logits[:, : R.ACTION_DIM]
            mean_loss = ((means - Y[idx]) ** 2).mean()
            clip_loss = torch.relu(torch.abs(means) - 1.0).square().mean()
            logstd_loss = ((logits[:, R.ACTION_DIM:] - source_logstd[idx]) ** 2).mean()
            anchor_loss = torch.stack([(p - a).square().mean() for p, a in zip(params, anchor)]).mean()
            loss = mean_loss + J_CLIP_W * clip_loss + J_LOGSTD_W * logstd_loss + J_ANCHOR_W * anchor_loss
            loss.backward(); opt.step()
            restore_logstd()
            with torch.no_grad():
                W1.mul_(clock_mask)
            losses.append(float(loss.item()))
        v = val_loss()
        hist.append({"epoch": epoch, "train_loss": float(np.mean(losses)), "val_mse": v})
        if v < best - 1e-12:
            best = v; best_epoch = epoch; bad = 0
            best_state = [p.detach().clone() for p in params]
        else:
            bad += 1
            if bad >= J_PATIENCE:
                break
        if progress and (epoch == 1 or epoch % 50 == 0):
            print(json.dumps(hist[-1]), flush=True)
    with torch.no_grad():
        for p, b in zip(params, best_state):
            p.copy_(b)
        W3[R.ACTION_DIM:] = src_logstd_w; b3[R.ACTION_DIM:] = src_logstd_b
        W1.mul_(clock_mask)
    scaled = {"pi.0.0.weight": W1.detach().numpy().astype(f32).copy(), "pi.0.0.bias": b1.detach().numpy().astype(f32).copy(),
              "pi.0.2.weight": W2.detach().numpy().astype(f32).copy(), "pi.0.2.bias": b2.detach().numpy().astype(f32).copy(),
              "pi.1.weight": W3.detach().numpy().astype(f32).copy(), "pi.1.bias": b3.detach().numpy().astype(f32).copy()}
    rep = {"protocol": "July 11/07 verified", "epochs_requested": J_EPOCHS, "epochs_run": len(hist),
           "best_epoch": best_epoch, "batch_size": J_BATCH, "learning_rate": J_LR,
           "validation_fraction": J_VAL_FRACTION, "validation_rows": int(n_val), "training_rows": int(len(tr_idx)),
           "patience": J_PATIENCE, "clip_weight": J_CLIP_W, "logstd_weight": J_LOGSTD_W,
           "anchor_weight": J_ANCHOR_W, "seed": J_SEED, "gradient_clipping": "none",
           "early_stopping": "validation MSE with patience; best weights restored",
           "best_val_mse": best, "history": hist}
    return scaled, rep


# --- offline gates (July/IK role) --------------------------------------------------------------------

def offline_gates(init_raw, export_raw, scaled, pre, vec, t1: float, save_reload: bool) -> dict[str, Any]:
    f32 = np.float32
    view = pre["_view"]
    obs500 = np.asarray(view["obs"], dtype=np.float32)
    u_ik = np.asarray(view["u_ik"], dtype=np.float64)
    obsS = np.asarray(pre["_obsS"], dtype=np.float32)
    labS = np.asarray(pre["_labels"], dtype=np.float64)
    m500 = RF.numpy_mean(export_raw, obs500)
    mS = RF.numpy_mean(export_raw, obsS)
    i500 = RF.numpy_mean(init_raw, obs500)
    iS = RF.numpy_mean(init_raw, obsS)
    obs_all = np.concatenate([obs500, obsS]); tgt_all = np.concatenate([u_ik, labS])
    m_all = RF.numpy_mean(export_raw, obs_all); i_all = RF.numpy_mean(init_raw, obs_all)
    rmse = lambda d: [float(np.sqrt(np.mean(d[:, j] ** 2))) for j in (0, 1)]  # noqa: E731
    agg_before = float(np.sqrt(np.mean((i_all - tgt_all) ** 2)))
    agg_after = float(np.sqrt(np.mean((m_all - tgt_all) ** 2)))
    t2 = float(np.max(np.abs(m500 - RF.numpy_mean(scaled, (obs500.astype(np.float64) / vec[None, :]).astype(f32)))))
    struct = RF.validate_init_state(export_raw, expected_actor_digest=None)
    inv = RF.invariance_test(export_raw, obs500[:64])
    integrity = {"ten_keys": tuple(export_raw.keys()) == RF.EXPECTED_KEY_ORDER,
                 "clock_columns_zero": bool(struct["clock_columns_zero"]),
                 "clock_invariance_bit_identical": bool(inv["bit_identical"]),
                 "logstd_byte_identical_to_S1A": bool(np.array_equal(np.asarray(export_raw["pi.1.weight"])[R.ACTION_DIM:], np.asarray(init_raw["pi.1.weight"])[R.ACTION_DIM:])
                                                      and np.array_equal(np.asarray(export_raw["pi.1.bias"])[R.ACTION_DIM:], np.asarray(init_raw["pi.1.bias"])[R.ACTION_DIM:])),
                 "save_reload_exact": bool(save_reload), "no_critic": True}
    binding = {
        "integrity_invariants": {"binding": True, **integrity, "pass": bool(all(integrity.values()))},
        "function_preservation": {"binding": True, "T1_maxabs": float(t1), "T2_maxabs": t2,
                                  "tol": G.PRESERVATION_TOL, "pass": bool(t1 <= G.PRESERVATION_TOL and t2 <= G.PRESERVATION_TOL)},
        "fit_convergence": {"binding": True, "aggregate_rmse_before": agg_before, "aggregate_rmse_after": agg_after,
                            "rule": "adapted aggregate RMSE strictly lower than the initial one (the only July-documentable offline criterion; 11/07 reports 0.973332 -> 0.011002)",
                            "pass": bool(agg_after < agg_before)},
    }
    q_ik = SC.decode_action(u_ik)[:, 1]; neg = q_ik < 0.0
    q_cmd = SC.decode_action(m500)[:, 1]
    def shape(pred, ref):
        out = {}
        for j, jn in ((0, "knee"), (1, "ankle")):
            x, y = pred[:, j], ref[:, j]
            sx, sy = float(x.std()), float(y.std())
            deg = bool(np.all(y >= 0.0) or np.all(y <= 0.0))
            out[jn] = {"rmse": float(np.sqrt(np.mean((x - y) ** 2))),
                       "pearson_r": (float(np.corrcoef(x, y)[0, 1]) if sx > 0 and sy > 0 else None),
                       "amplitude_ratio": (sx / sy if sy > 0 else None),
                       "sign_agreement": (None if deg else float(np.mean(np.sign(x) == np.sign(y)))),
                       "sign_agreement_verdict": ("VOID_degenerate_reference" if deg else "valid")}
        return out
    measures = {
        "note": "PREREGISTERED MEASURES without invented thresholds; promotion is left to the Codex audit",
        "vs_IK_on_corpus_500": shape(m500, u_ik), "vs_IK_on_prefix_392": shape(mS, labS),
        "rmse_per_joint_aggregate": rmse(m_all - tgt_all),
        "negative_window_coverage": {
            "rows": int(neg.sum()),
            "fraction_positive_command_before": float(np.mean(SC.decode_action(i500)[neg, 1] > 0.0)),
            "fraction_positive_command_after": float(np.mean(q_cmd[neg] > 0.0)),
            "min_commanded_ankle_in_windows": float(q_cmd[neg].min())},
        "max_abs_vs_S0D_INFORMATIONAL": _max_abs_vs_s0d(export_raw, view),
        "healthy_symmetry_DIAGNOSTIC": _healthy(m500, view),
    }
    failed = [k for k, v in binding.items() if not v["pass"]]
    return {"binding": binding, "failed": failed, "all_binding_pass": bool(not failed),
            "preregistered_measures": measures, "actor_digest": struct["actor_digest"], "structure": struct}


def _max_abs_vs_s0d(export_raw, view) -> dict[str, Any]:
    s0d = _s0d_state()
    d = RF.numpy_mean(export_raw, np.asarray(view["obs"], dtype=np.float32)) - RF.numpy_mean(s0d, np.asarray(view["obs"], dtype=np.float32))
    return {"max_abs_per_joint": [float(np.abs(d[:, j]).max()) for j in (0, 1)],
            "mean_abs_per_joint": [float(np.abs(d[:, j]).mean()) for j in (0, 1)],
            "status": "INFORMATIONAL, never a gate: S1C-3 proved max_abs <= 0.25 mathematically incompatible with sign recovery"}


def _healthy(m500, view) -> dict[str, Any]:
    import f2r_labeller as LB
    h = np.asarray(LB.load_cache(R.OUT_CACHE, "nominal").targets)[:, [0, 2]]
    q = SC.decode_action(m500)
    return {"rmse_per_joint": [float(np.sqrt(np.mean((q[:, j] - h[:, j]) ** 2))) for j in (0, 1)],
            "status": "DIAGNOSTIC, never binding: the target of this stage is the prosthetic IK"}


def _s0d_state():
    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    return {k: np.asarray(W._as_numpy(v)) for k, v in W.load_module_state(SR.S0D_MODULE).items()}


# --- closed-loop stage ---------------------------------------------------------------------------------

def rollout_command(python_exe: str = SB.PYTHON_EXE) -> list[str]:
    return [python_exe, str(F1.ROLLOUT_EVAL), "--checkpoint", str(OUT_CAND / "rl_module"),
            "--no-auto-config", "--config", str(F1.RUNTIME_CONFIG),
            "--episode-start-offset-s", repr(float(R.EXACT_STARTS["nominal"])),
            "--action-selection", "deterministic", "--seed", str(R.DET_SEED),
            "--output-dir", str(JOB_DIR), "--record-outputs", "--record-policy-trace",
            *[str(a) for a in F1.JOB_TIMEOUT_ARGS], *[str(a) for a in C.RUNTIMES[F1.TARGET_RUNTIME]["extra_args"]]]


def qualitative_ankle_gate(job_dir: Path) -> dict[str, Any]:
    """Binding: negative realised ankle values must APPEAR inside the ex-ante negative-IK window."""
    traj = DS.trajectory_from_job(job_dir, expected_width=R.ENV_ACTOR_WIDTH)
    obs = np.asarray(traj["obs35"], dtype=np.float64)
    import f2r_labeller as LB
    cc = LB.load_cache(R.OUT_CACHE, "nominal")
    idx = cc.lookup(np.asarray(traj["t_pre"], dtype=np.float64))
    q_ik = SC.decode_action(cc.ik_action[idx].astype(np.float64))[:, 1]
    neg = q_ik < 0.0
    ankle = obs[:, RO.IDX_ANKLE_Q]
    cmd = SC.decode_action(np.asarray(traj["b_raw_action"], dtype=np.float64))[:, 1]
    win_min = float(ankle[neg].min()) if neg.any() else None
    return {"ex_ante_window": "rows of this actor's own trace whose decoded IK ankle target is negative (never the phase field, which is null)",
            "rows_in_window": int(neg.sum()),
            "min_realised_ankle_q_in_window": win_min,
            "binding_test": "min realised ankle_q inside the window < 0 (negative values must appear)",
            "binding": True, "pass": bool(win_min is not None and win_min < 0.0),
            "documented_reference_threshold": B3_REFERENCE_THRESHOLD,
            "meets_documented_-0.03": (bool(win_min <= B3_REFERENCE_THRESHOLD) if win_min is not None else None),
            "threshold_note": "the -0.03 value is the DOCUMENTED F2R gate B3, whose native phase window [0.55,0.80] is unavailable; its use for promotion is left to the Codex audit",
            "fraction_positive_command_in_window": (float(np.mean(cmd[neg] > 0.0)) if neg.any() else None),
            "L20_reference_fraction_positive": 0.8865979381443299,
            "sign_improvement_vs_L20": "PREREGISTERED MEASURE, no invented threshold; promotion left to the audit"}


def run_closed_loop(cand_receipt_sha: str, python_exe: str = SB.PYTHON_EXE) -> dict[str, Any]:
    """ONE deterministic nominal rollout, executed only after every binding offline gate passed."""
    if JOB_DIR.exists():
        raise Rev4bError(f"no-clobber: {JOB_DIR} exists (exactly one launch; no retry)")
    cmd = rollout_command(python_exe)
    JOB_DIR.mkdir(parents=True, exist_ok=False)
    VA.OUT_LOGS.mkdir(parents=True, exist_ok=True)
    log_path = VA.OUT_LOGS / LOG_NAME
    t0 = time.time()
    with open(log_path, "x", encoding="utf-8") as log:
        log.write(" ".join(cmd) + "\n\n"); log.flush()
        proc = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, cwd=str(R.BASELINE_DIR))
    rec: dict[str, Any] = {"schema": "v26b_rev4b_rollout.1", "authorized_stage": AUTHORIZED_STAGE,
                           "amendment_rev4b": PIN_AMENDMENT_REV4B, "candidate_receipt_sha256": cand_receipt_sha,
                           "deployable": False, "sigma": "UNRESOLVED; deterministic mean only",
                           "command": cmd, "returncode": int(proc.returncode),
                           "duration_s": round(time.time() - t0, 3), "log": C.rel(log_path),
                           "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}
    status = "FAILED"
    try:
        if proc.returncode != 0:
            raise Rev4bError(f"rollout returncode {proc.returncode} (log {log_path})")
        analysis = RO.analyse_rollout(JOB_DIR)
        rows = json.loads((JOB_DIR / "rollout_policy_trace.json").read_text(encoding="utf-8"))
        rowscan = R1.counter_rowscan(rows)
        seven = SB.eligibility_gates(analysis, rowscan)
        ankle = qualitative_ankle_gate(JOB_DIR)
        rec["analysis"] = analysis
        rec["fsm_counters_rowscan"] = rowscan
        rec["seven_gates"] = seven
        rec["qualitative_ankle_gate"] = ankle
        rec["diagnostics"] = {"action_stats_whole_trace": R1.action_stats(rows),
                              "b3_phase_window": PA.b3_late_stance(rows),
                              "closed_loop_comparison": LR.closed_loop_comparison(JOB_DIR)}
        ok = bool(seven["all_pass"] and ankle["pass"])
        rec["all_binding_pass"] = ok
        rec["failed"] = list(seven["failed"]) + ([] if ankle["pass"] else ["qualitative_ankle_gate"])
        status = PASS_STATUS if ok else FAIL_STATUS
        rec["promotion"] = {"promoted": False, "note": "no promotion in this stage; the architect decides after the audit"}
    finally:
        rec["status"] = status
        rec["inherited_prose_allowlisted"] = SB.assert_no_deployable_marking_local(rec, "rollout_receipt")
        R.write_json_exclusive(JOB_DIR / ROLLOUT_RECEIPT_NAME, rec)
    return rec


def run_stage(*, authorized_stage: str | None, progress: bool = True) -> dict[str, Any]:
    if authorized_stage != AUTHORIZED_STAGE:
        raise Rev4bError(f"requires --authorized-stage {AUTHORIZED_STAGE}; got {authorized_stage!r}")
    lineage = verify_lineage()
    pre = preflight()
    if OUT_CAND.exists():
        raise FileExistsError(f"no-clobber: {OUT_CAND} exists")
    agg = build_aggregate(pre)
    vec, scale_table = G.scale_vector()
    init_raw = pre["_init"]
    init_scaled = G.transform_state_to_scaled(init_raw, vec)
    t1 = G.preservation_test(init_raw, init_scaled, np.asarray(pre["_view"]["obs"], dtype=np.float32), vec)
    if t1 > G.PRESERVATION_TOL:
        raise Rev4bError(f"prefit T1 FAILED {t1:.3e}")
    obs_scaled = (agg["observations"].astype(np.float64) / vec[None, :]).astype(np.float32)
    scaled, frep = fit_july(init_scaled, obs_scaled, agg["actions"], progress=progress)
    export_raw = G.export_state_from_scaled(scaled, vec)
    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    OUT_CAND.parent.mkdir(parents=True, exist_ok=True)
    lock, tok = RF.acquire_export_lock(OUT_CAND)
    staging = None; promoted = False
    try:
        staging = RF._staging_dir_for(OUT_CAND)
        sm = staging / "rl_module"; sm.mkdir(parents=True, exist_ok=False)
        shutil.copy2(S1A_MODULE / "metadata.json", sm / "metadata.json")
        shutil.copy2(S1A_MODULE / "class_and_ctor_args.pkl", sm / "class_and_ctor_args.pkl")
        with (sm / "module_state.pkl").open("wb") as fh:
            pickle.dump({k: np.asarray(v) for k, v in export_raw.items()}, fh, protocol=pickle.HIGHEST_PROTOCOL)
        reloaded = W.load_module_state(sm)
        exact = bool(W.compare_actor_states({k: np.asarray(v) for k, v in export_raw.items()}, reloaded).get("exact"))
        gates = offline_gates(init_raw, export_raw, scaled, pre, vec, t1, exact)
        names35, _, mshas = VS.pinned_names()
        manifest = {"schema_version": 1, "actor_feature_names": names35, "actor_feature_count": 35,
                    "actor_digest": gates["actor_digest"], "module_state_sha256": C.sha256_file(sm / "module_state.pkl"),
                    "manifest35_sha256": mshas["manifest35_sha256"],
                    "exploration_sigma": [VS.SIGMA_PLACEHOLDER] * R.ACTION_DIM, "sigma_note": G.SIGMA_NOTE,
                    "derived_from": C.rel(S1A_MODULE), "source_actor_digest": PIN_S1A_ACTOR,
                    "contract": A.CONTRACT_STRING, "deployable": False, "sigma_unresolved": True,
                    "protocol": "July 11/07 faithful DAgger round 1 from S1A",
                    "offline_verdict": ("PASS" if gates["all_binding_pass"] else "FAIL"),
                    "status": "INTERMEDIATE CANDIDATE (rev4b); no closed-loop evidence at materialisation time"}
        A.assert_no_deployable_marking(manifest, "manifest")
        C.write_json(sm / W.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME, manifest, clobber=False)
        receipt = {"schema": "v26b_rev4b_fit.1", "amendment_rev4b": PIN_AMENDMENT_REV4B,
                   "authorized_stage": AUTHORIZED_STAGE, "deployable": False, "sigma_unresolved": True,
                   "lineage": lineage,
                   "init": {"module": C.rel(S1A_MODULE), "actor_digest": PIN_S1A_ACTOR,
                            "role": "S1A is collection policy (frozen 392-step rollout), fit init and anchor parameter"},
                   "preflight": {k: v for k, v in pre.items() if not k.startswith("_")},
                   "dataset": {"teacher_corpus_rows": int(len(pre["_view"]["obs"])), "prefix_rows": int(len(pre["_obsS"])),
                               "trace_repeat": J_TRACE_REPEAT, "aggregate_rows": int(agg["observations"].shape[0]),
                               "operation": "concat(teacher, tile(visited, repeat)); no truncation, no dedup (July verbatim)"},
                   "july_protocol": {k: v for k, v in frep.items() if k != "history"},
                   "loss_history_first_last": {"first": frep["history"][0], "last": frep["history"][-1]},
                   "loss_history_full": frep["history"],
                   "scaling": {"table": scale_table, "T1_maxabs": t1, "declared_deviation": "July physical scaling, function-preserving, absorbed at export"},
                   "offline_gates": gates, "save_reload_exact": exact,
                   "output_module": C.rel(OUT_CAND / "rl_module"),
                   "output_files_sha256": {p.name: C.sha256_file(p) for p in sorted(sm.iterdir())},
                   "scope": "one July-faithful DAgger round; no new collection, no PPO, no sigma, no promotion",
                   "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}
        A.assert_no_deployable_marking(receipt, "receipt")
        C.write_json(staging / RECEIPT_NAME, receipt)
        csha = C.sha256_file(staging / RECEIPT_NAME)
        RF.promote_staging(staging, OUT_CAND); promoted = True
    except BaseException:
        if staging is not None and not promoted:
            shutil.rmtree(staging, ignore_errors=True)
        raise
    finally:
        RF.release_export_lock(lock, tok)
    out = {"candidate": C.rel(OUT_CAND), "receipt_sha256": csha, "offline": gates}
    if gates["all_binding_pass"]:
        out["rollout"] = run_closed_loop(csha)
    else:
        out["rollout"] = {"status": "NOT_RUN", "reason": "a binding offline gate failed; the closed-loop stage is not triggered"}
    return out


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="V26B rev4b July-faithful DAgger from S1A")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--authorized-stage", default=None)
    args = parser.parse_args(argv)
    if args.preflight_only:
        verify_lineage()
        pre = preflight()
        print(json.dumps({k: v for k, v in pre.items() if not k.startswith("_")}, indent=2, default=str))
        return 0
    if not args.execute:
        verify_lineage()
        print(json.dumps({"mode": "dry", "ok": True}, indent=2))
        return 0
    out = run_stage(authorized_stage=args.authorized_stage)
    r = out["rollout"]
    print(json.dumps({"candidate": out["candidate"], "receipt_sha256": out["receipt_sha256"],
                      "offline_pass": out["offline"]["all_binding_pass"], "offline_failed": out["offline"]["failed"],
                      "rollout_status": r.get("status"), "rollout_failed": r.get("failed"),
                      "seven_gates": (r.get("seven_gates") or {}).get("gates"),
                      "ankle_gate": r.get("qualitative_ankle_gate")}, indent=2, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
