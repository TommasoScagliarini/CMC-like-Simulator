"""V26B rev3o — S1 IK-AB06 offline adaptation FIT (token V26B-S1-FIT granted).

Init: S0D 35D ONLY (481dd0d2..., module_state cda6d893...).  Task: the 380 S0D-visited
rows of the pinned nominal trace (steps 1-190 and 311-500) with same-time u_IK AB06
labels; steps 191-310 (blocked holdout 201-300 + embargo) are NEVER trained on, and the
rev3m source holdout never enters training.  Numerics frozen by the chain: 300 fixed
epochs / batch 256 / Adam lr 1e-4 / seed 2026, flat 2-output MSE + 1.0 action-clip
penalty + 1e-5 July anchor (mean over the 6 FULL tensors, 0.5 head factors) towards
theta_S0D, July physical scaling absorbed at export, logstd restored/frozen, no aux.

Binding offline gates: G_task RMSE vs u_IK on the out-of-sample holdout 201-300
<= 0.15/joint; Q1 vs the S0D deterministic mean on the rev3m source holdout <= 0.10/joint;
Q3 invariants; T1/T2 <= 1e-5.  Full-500 metrics and the drift vs S0D on the task rows are
DIAGNOSTIC only.  Any FAIL -> REJECTED artifact, no checkpoint published, STOP.

Offline stage only: this module never executes a closed-loop episode.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import pickle
import shutil
import sys
import time
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import v26b_s1_pregate_rev3n as N  # noqa: E402  (rev3n lineage, unmodified)
import v26b_s1_prereg as S1  # noqa: E402       (rev3l view, unmodified)
import v26b_s0d as S0  # noqa: E402             (rev3k split, read-only)
import v26b_s0d_fit as F0  # noqa: E402         (frozen numerics reused by reference)
import v26b_s0d_rollout as SR  # noqa: E402     (S0D module pins, read-only)
import v26b_r2g as G  # noqa: E402              (scaling/preservation, read-only)
import v26b_r2i as I  # noqa: E402              (anchor_loss_july_exact, read-only)
import v26b_anchors as VA  # noqa: E402
import v26b_student as VS  # noqa: E402
import f0_common as C  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_refit as RF  # noqa: E402


class S1FitError(RuntimeError):
    pass


AUTHORIZED_STAGE = "V26B-S1-FIT"
AMENDMENT_REV3O = HERE / "v26b_amendment_rev3o_s1_fit_execution.json"
PIN_AMENDMENT_REV3O = "abe463c3d64b4b91a0d5ef83517159bcc6fc417d029be4dd51db9c92a0dfc978"
REV3N_RECEIPT = VA.OUT_ROOT / "v26b_s1_pregate_rev3n_receipt_20260824_185328.json"
PIN_REV3N_RECEIPT = "923ffc5ca8fb4d3577d9894e90db151a92bad089d3669b38fb9553484e6f2634"

S0D_MODULE = SR.S0D_MODULE
PIN_S0D_ACTOR = "481dd0d22919fc1ec04cdb722409b9711caeb61d57449c210aad7386375b764a"
PIN_S0D_MODULE_STATE = "cda6d893138444908b4fcc908dc0045bd0df317ef0f6bbc72f70e84629e14597"
FORBIDDEN_INIT_NAMES = ("JUL_H0", "R0a", "R1", "R2G", "R2I")
# Exclusivity is enforced POSITIVELY (strictly stronger than a blacklist): the loaded init
# must hash to PIN_S0D_ACTOR and its module_state.pkl to PIN_S0D_MODULE_STATE, so no other
# actor of the chain can be used as init; labels come only from u_IK and the S0D mean.

TRAIN_STEPS_EXCLUDED = ((191, 200), (201, 300), (301, 310))
EXPECTED_TRAIN_ROWS = 380
EXPECTED_HOLD_ROWS = 100
EXPECTED_SOURCE_ROWS = 4480
EXPECTED_SOURCE_SHARED = 6
EXPECTED_SOURCE_USABLE = 4474

G_TASK_MAX = S1.PRE_GATE_RMSE_MAX        # 0.15, reused by reference (never redefined)
Q1_MAX = F0.POSTFIT_RMSE_MAX             # 0.10, reused by reference
ANCHOR_WEIGHT = F0.ANCHOR_WEIGHT_JULY_1107   # 1e-5 July 11/07
BUDGET = dict(F0.BUDGET)                 # 300 / 256 / 1e-4 / seed 2026 / clip 1.0
OUT_S1 = VA.OUT_ROOT / "student" / "S1_IK_AB06_35D"
RECEIPT_NAME = "v26b_s1_fit_receipt.json"
CONTEXT = {"JUL_H0_july_final": {"offline_aggregate_rmse": 0.008144, "closed_loop": "500/500 (July guards 15/25mm)"},
           "S0D": {"fit_holdout": [0.0811, 0.0793], "closed_loop": "500/500 (v3 guards)"}}


# --- lineage -------------------------------------------------------------------------------------

def verify_lineage_s1_fit() -> dict[str, Any]:
    lin = N.verify_lineage_rev3n()          # rev3l+rev3m+rev3n+S0D chain+AB06 cache+additivity proof
    got = C.sha256_file(AMENDMENT_REV3O)
    if got != PIN_AMENDMENT_REV3O:
        raise S1FitError(f"rev3o sha {got} != pinned")
    lin["amendment_rev3o"] = got
    got = C.sha256_file(REV3N_RECEIPT)
    if got != PIN_REV3N_RECEIPT:
        raise S1FitError(f"rev3n pre-gate receipt sha {got} != pinned")
    rec = json.loads(REV3N_RECEIPT.read_text(encoding="utf-8"))
    if rec["pre_gate"]["binding"]["pass"] is not True:
        raise S1FitError("rev3n pre-gate receipt does not record PASS")
    lin["rev3n_pregate_receipt"] = got
    lin["rev3n_pregate_median_per_joint"] = rec["pre_gate"]["binding"]["median_per_joint"]
    got = C.sha256_file(S0D_MODULE / "module_state.pkl")
    if got != PIN_S0D_MODULE_STATE:
        raise S1FitError(f"S0D module_state sha {got} != pinned")
    lin["init_module_state"] = got
    return lin


# --- data ----------------------------------------------------------------------------------------

def build_s1_task() -> dict[str, Any]:
    """The rev3l view with the leakage contract enforced fail-closed."""
    view = S1.build_s1_view()
    step = np.arange(1, 501)
    train, hold = view["train"], view["hold"]
    if int(train.sum()) != EXPECTED_TRAIN_ROWS or int(hold.sum()) != EXPECTED_HOLD_ROWS:
        raise S1FitError(f"split {int(train.sum())}/{int(hold.sum())} != {EXPECTED_TRAIN_ROWS}/{EXPECTED_HOLD_ROWS}")
    for lo, hi in TRAIN_STEPS_EXCLUDED:
        if np.any((step[train] >= lo) & (step[train] <= hi)):
            raise S1FitError(f"LEAKAGE: training rows inside excluded steps [{lo},{hi}]")
    if not np.array_equal(step[hold], np.arange(201, 301)):
        raise S1FitError("holdout is not exactly steps 201-300")
    view["records"]["train_steps"] = [[1, 190], [311, 500]]
    view["records"]["excluded_steps"] = [list(x) for x in TRAIN_STEPS_EXCLUDED]
    view["records"]["train_obs_sha256"] = hashlib.sha256(view["obs"][train].tobytes()).hexdigest()
    view["records"]["train_labels_sha256"] = hashlib.sha256(view["u_ik"][train].tobytes()).hexdigest()
    view["records"]["holdout_obs_sha256"] = hashlib.sha256(view["obs"][hold].tobytes()).hexdigest()
    return view


def build_source_holdout(view: Mapping[str, Any], s0d_raw: Mapping[str, Any]) -> dict[str, Any]:
    """rev3m preservation set: rev3k provenance-held rows minus the bitwise-shared task rows;
    labels = S0D deterministic mean on the SAME state.  Never used for training."""
    split = S0.build_split()
    ho = split["assign"]["hold_idx"]
    src = np.asarray(split["obs"])[ho]
    task_keys = {view["obs"][i].tobytes() for i in range(view["obs"].shape[0])}
    shared = np.asarray([src[i].tobytes() in task_keys for i in range(src.shape[0])])
    if int(src.shape[0]) != EXPECTED_SOURCE_ROWS or int(shared.sum()) != EXPECTED_SOURCE_SHARED:
        raise S1FitError(f"source holdout {src.shape[0]} rows / {int(shared.sum())} shared != {EXPECTED_SOURCE_ROWS}/{EXPECTED_SOURCE_SHARED}")
    keep = src[~shared].astype(np.float32)
    if keep.shape[0] != EXPECTED_SOURCE_USABLE:
        raise S1FitError(f"usable source rows {keep.shape[0]} != {EXPECTED_SOURCE_USABLE}")
    labels = RF.numpy_mean(s0d_raw, keep)
    return {"obs": keep, "labels": labels,
            "records": {"rows_before_exclusion": EXPECTED_SOURCE_ROWS, "bitwise_shared_excluded": EXPECTED_SOURCE_SHARED,
                        "usable_rows": int(keep.shape[0]), "obs_sha256": hashlib.sha256(keep.tobytes()).hexdigest(),
                        "labels_sha256": hashlib.sha256(np.asarray(labels, np.float32).tobytes()).hexdigest(),
                        "labels_rule": "S0D deterministic mean, same state (numpy forward of the pinned init)",
                        "never_in_training": True}}


# --- fit -----------------------------------------------------------------------------------------

def fit_s1(init_scaled: Mapping[str, Any], view: Mapping[str, Any], vec: np.ndarray, *,
           budget: Mapping[str, Any] | None = None, progress: bool = False) -> tuple[dict[str, Any], dict[str, Any]]:
    import torch

    budget = dict(budget or BUDGET)
    if "patience" in budget or "validation_fraction" in budget:
        raise S1FitError("no early stopping / validation model selection (frozen chain numerics)")
    f32 = np.float32
    tr = view["train"]
    obs_scaled = (view["obs"].astype(np.float64) / vec[None, :]).astype(f32)
    obs_t = torch.as_tensor(obs_scaled[tr])
    tgt = torch.as_tensor(view["u_ik"][tr].astype(f32))
    torch.use_deterministic_algorithms(True); torch.set_num_threads(1)
    torch.manual_seed(int(budget["seed"])); np.random.seed(int(budget["seed"]))
    W1 = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.0.0.weight"], dtype=f32)))
    b1 = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.0.0.bias"], dtype=f32)))
    W2 = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.0.2.weight"], dtype=f32)))
    b2 = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.0.2.bias"], dtype=f32)))
    W3m = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.1.weight"][: R.ACTION_DIM], dtype=f32)))
    b3m = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.1.bias"][: R.ACTION_DIM], dtype=f32)))
    mean_params = [W1, b1, W2, b2, W3m, b3m]
    anchor_t = [p.detach().clone() for p in mean_params]          # theta_S0D in the scaled space
    clock_mask = torch.ones_like(W1); clock_mask[:, list(R.CLOCK_COLUMNS)] = 0.0
    with torch.no_grad():
        W1.mul_(clock_mask)
    encoder = lambda x: torch.tanh(torch.tanh(x @ W1.T + b1) @ W2.T + b2)  # noqa: E731
    opt = torch.optim.Adam(mean_params, lr=float(budget["lr"]))
    rng = np.random.default_rng(int(budget["seed"]))
    n = int(obs_t.shape[0]); epochs = int(budget["epochs"]); batch = int(budget["batch_size"])
    lam_clip = float(budget["clip_weight"]); history = []
    for epoch in range(1, epochs + 1):
        perm = rng.permutation(n); losses = []; dmse = []; anch = []
        for s0 in range(0, n, batch):
            idx = torch.as_tensor(perm[s0: s0 + batch]); opt.zero_grad(set_to_none=True)
            m = encoder(obs_t[idx]) @ W3m.T + b3m
            l_data = ((m - tgt[idx]) ** 2).mean()                          # flat row-uniform 2-output MSE
            anchor = I.anchor_loss_july_exact(mean_params, anchor_t)       # mean of the 6 FULL tensors
            loss = l_data + lam_clip * torch.relu(torch.abs(m) - 1.0).square().mean() + ANCHOR_WEIGHT * anchor
            loss.backward(); opt.step()
            with torch.no_grad():
                W1.mul_(clock_mask)
            losses.append(float(loss.item())); dmse.append(float(l_data.item())); anch.append(float(anchor.item()))
        history.append({"epoch": epoch, "loss": float(np.mean(losses)), "flat_mse": float(np.mean(dmse)), "anchor_term": float(np.mean(anch))})
        if progress and (epoch == 1 or epoch % 50 == 0):
            print(json.dumps(history[-1]), flush=True)
    hidden = int(W2.shape[0])
    logstd_w = np.zeros((R.ACTION_DIM, hidden), dtype=f32)
    logstd_b = np.asarray(np.log(np.full(R.ACTION_DIM, float(R.SIGMA_CONSTANT))), dtype=f32)
    w3 = np.concatenate([W3m.detach().numpy().astype(f32), logstd_w], axis=0)
    b3 = np.concatenate([b3m.detach().numpy().astype(f32), logstd_b], axis=0)
    scaled_fit = {"pi.0.0.weight": W1.detach().numpy().astype(f32).copy(), "pi.0.0.bias": b1.detach().numpy().astype(f32).copy(),
                  "pi.0.2.weight": W2.detach().numpy().astype(f32).copy(), "pi.0.2.bias": b2.detach().numpy().astype(f32).copy(),
                  "pi.1.weight": w3.astype(f32).copy(), "pi.1.bias": b3.astype(f32).copy()}
    report = {"tool": "v26b_s1_fit.fit_s1 (rev3o IK-AB06 adaptation)", "rows_train": n, "budget": budget,
              "anchor": {"weight": ANCHOR_WEIGHT, "semantics": "July 11/07 exact: mean over the 6 FULL tensors, explicit 0.5 head factors, logstd restored/frozen, towards theta_S0D_scaled"},
              "loss": "row-uniform flat 2-output MSE vs u_IK + 1.0*action-clip penalty + 1e-5*anchor; no aux; no masses; no variance normalisation",
              "history": history, "sigma_note": G.SIGMA_NOTE}
    return scaled_fit, report


# --- gates ---------------------------------------------------------------------------------------

def gate_decision(g_task: Sequence[float], q1: Sequence[float], q3_pass: bool, t1: float, t2: float) -> dict[str, Any]:
    """Pure decision function (testable with synthetic values); thresholds never relaxed."""
    gt = {"metric": "RMSE vs u_IK on the OUT-OF-SAMPLE blocked holdout steps 201-300", "per_joint": [float(v) for v in g_task],
          "max_per_joint": G_TASK_MAX, "binding": True, "pass": bool(all(float(v) <= G_TASK_MAX for v in g_task))}
    q = {"metric": "RMSE vs the S0D deterministic mean on the rev3m source holdout (never trained on)", "per_joint": [float(v) for v in q1],
         "max_per_joint": Q1_MAX, "binding": True, "pass": bool(all(float(v) <= Q1_MAX for v in q1))}
    fp = {"T1_maxabs": float(t1), "T2_maxabs": float(t2), "tol": G.PRESERVATION_TOL, "binding": True,
          "pass": bool(float(t1) <= G.PRESERVATION_TOL and float(t2) <= G.PRESERVATION_TOL)}
    q3 = {"binding": True, "pass": bool(q3_pass)}
    return {"G_task": gt, "Q1_source_holdout_rev3m": q, "Q3_invariants": q3, "function_preservation": fp,
            "pass_all": bool(gt["pass"] and q["pass"] and q3["pass"] and fp["pass"])}


def evaluate(init_raw: Mapping[str, Any], export_raw: Mapping[str, Any], scaled_fit: Mapping[str, Any],
             view: Mapping[str, Any], src: Mapping[str, Any], vec: np.ndarray, t1: float) -> dict[str, Any]:
    f32 = np.float32
    obs = view["obs"].astype(np.float64)
    m_exp = RF.numpy_mean(export_raw, obs.astype(f32))
    m_scaled = RF.numpy_mean(scaled_fit, (obs / vec[None, :]).astype(f32))
    m_src_exp = RF.numpy_mean(export_raw, src["obs"])
    m_src_scaled = RF.numpy_mean(scaled_fit, (src["obs"].astype(np.float64) / vec[None, :]).astype(f32))
    t2 = float(max(np.max(np.abs(m_exp - m_scaled)), np.max(np.abs(m_src_exp - m_src_scaled))))
    tr, ho = view["train"], view["hold"]
    u_ik, u_own = view["u_ik"], view["u_own"]
    rmse = lambda d: np.sqrt(np.mean(d ** 2, axis=0)).tolist()  # noqa: E731
    g_task = rmse(m_exp[ho] - u_ik[ho])
    q1 = rmse(m_src_exp - src["labels"])
    struct = RF.validate_init_state(export_raw, expected_actor_digest=None)
    inv = RF.invariance_test(export_raw, obs.astype(f32)[:64])
    q3 = {"ten_keys": tuple(export_raw.keys()) == RF.EXPECTED_KEY_ORDER,
          "clock_columns_zero": bool(struct["clock_columns_zero"]),
          "logstd_frozen_placeholder": bool(struct["sigma_head"]["logstd_bias_exact"]),
          "logstd_bit_identical_to_init": bool(np.array_equal(np.asarray(export_raw["pi.1.weight"])[R.ACTION_DIM:], np.asarray(init_raw["pi.1.weight"])[R.ACTION_DIM:])
                                               and np.array_equal(np.asarray(export_raw["pi.1.bias"])[R.ACTION_DIM:], np.asarray(init_raw["pi.1.bias"])[R.ACTION_DIM:])),
          "clock_invariance_bit_identical": bool(inv["bit_identical"]), "no_critic": True}
    m_init = RF.numpy_mean(init_raw, obs.astype(f32))
    drift = np.abs(m_exp - m_init)
    gates = gate_decision(g_task, q1, all(q3.values()), t1, t2)
    gates["Q3_invariants"].update(q3)
    gates["diagnostics"] = {
        "full_500_rmse_vs_uIK": rmse(m_exp - u_ik),
        "train_in_sample_rmse_vs_uIK": rmse(m_exp[tr] - u_ik[tr]),
        "drift_vs_S0D_on_500_task_rows": {"mean_abs_per_joint": drift.mean(0).tolist(), "max_abs_per_joint": drift.max(0).tolist(),
                                          "rmse_per_joint": rmse(m_exp - m_init), "binding": False,
                                          "expected": "~0.38 knee / ~0.30 ankle: the intentional structural gap towards u_IK"},
        "s0d_baseline_rmse_vs_uIK_holdout": rmse(m_init[ho] - u_ik[ho]),
        "s0d_baseline_rmse_vs_uIK_full500": rmse(m_init - u_ik),
        "gap_own_vs_ik_frozen_fact": view["records"]["gap_own_vs_ik"],
        "parameter_shift_sq_vs_S0D_raw": float(sum(np.sum((np.asarray(export_raw[k], np.float64) - np.asarray(init_raw[k], np.float64)) ** 2) for k in RF.CANONICAL_KEYS)),
        "mandatory_comparison_context": CONTEXT}
    gates["actor_digest_new"] = struct["actor_digest"]
    gates["structure"] = struct
    return gates


# --- run -----------------------------------------------------------------------------------------

def run_s1_fit(*, authorized_stage: str | None, out_dir: Path = OUT_S1, progress: bool = True,
               runtime_status: dict[str, Any] | None = None) -> dict[str, Any]:
    if authorized_stage != AUTHORIZED_STAGE:
        raise S1FitError(f"the S1 fit requires --authorized-stage {AUTHORIZED_STAGE}; got {authorized_stage!r}")
    lineage = verify_lineage_s1_fit()
    vec, scale_table = G.scale_vector()
    view = build_s1_task()
    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    init_raw = {k: np.asarray(W._as_numpy(v)) for k, v in W.load_module_state(S0D_MODULE).items()}
    struct0 = RF.validate_init_state(init_raw, expected_actor_digest=PIN_S0D_ACTOR)
    if struct0["actor_digest"] != PIN_S0D_ACTOR:
        raise S1FitError("init is not the pinned S0D actor")
    src = build_source_holdout(view, init_raw)
    init_scaled = G.transform_state_to_scaled(init_raw, vec)
    t1 = G.preservation_test(init_raw, init_scaled, view["obs"], vec)
    if t1 > G.PRESERVATION_TOL:
        raise S1FitError(f"prefit T1 FAILED: {t1:.3e}")
    scaled_fit, fit_report = fit_s1(init_scaled, view, vec, progress=progress)
    export_raw = G.export_state_from_scaled(scaled_fit, vec)
    gates = evaluate(init_raw, export_raw, scaled_fit, view, src, vec, t1)
    if not gates["pass_all"]:
        rej = R.unique_artifact_path(VA.OUT_ROOT, f"v26b_s1_fit_REJECTED_{time.strftime('%Y%m%d_%H%M%S')}", ".json")
        VA._atomic_fill_reserved(rej, json.dumps({"schema": "v26b_s1_fit_rejected.1", "REJECTED": True,
                                                  "amendment_rev3o": PIN_AMENDMENT_REV3O, "lineage": lineage,
                                                  "split": view["records"], "source_holdout": src["records"],
                                                  "gates": gates, "fit": {k: fit_report[k] for k in ("tool", "rows_train", "budget", "anchor", "loss")},
                                                  "loss_history_full": fit_report["history"],
                                                  "policy": "no checkpoint published; thresholds/metrics/labels unchanged; STOP",
                                                  "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}, indent=2, default=str) + "\n")
        raise S1FitError(f"offline gates FAILED: no export; diagnosis at {C.rel(rej)}")
    s0d_pre = VS.source_files_table(S0D_MODULE)
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
        shutil.copy2(S0D_MODULE / "metadata.json", stage_module / "metadata.json")
        shutil.copy2(S0D_MODULE / "class_and_ctor_args.pkl", stage_module / "class_and_ctor_args.pkl")
        with (stage_module / "module_state.pkl").open("wb") as fh:
            pickle.dump({k: np.asarray(v) for k, v in export_raw.items()}, fh, protocol=pickle.HIGHEST_PROTOCOL)
        reloaded = W.load_module_state(stage_module)
        cmp = W.compare_actor_states({k: np.asarray(v) for k, v in export_raw.items()}, reloaded)
        if not cmp.get("exact"):
            raise S1FitError(f"save/reload mismatch: {cmp}")
        struct = RF.validate_init_state(reloaded, expected_actor_digest=gates["actor_digest_new"])
        names35, _, manifest_shas = VS.pinned_names()
        manifest = {"schema_version": 1, "actor_feature_names": names35, "actor_feature_count": 35,
                    "actor_digest": struct["actor_digest"], "module_state_sha256": C.sha256_file(stage_module / "module_state.pkl"),
                    "manifest35_sha256": manifest_shas["manifest35_sha256"], "exploration_sigma": [VS.SIGMA_PLACEHOLDER] * R.ACTION_DIM,
                    "sigma_note": G.SIGMA_NOTE, "derived_from": C.rel(S0D_MODULE), "source_actor_digest": PIN_S0D_ACTOR,
                    "contract": "deployable_markov_controller_state",
                    "note": "S1 IK-AB06 offline adaptation from S0D (rev3o); July physical scaling absorbed: raw physical observations"}
        C.write_json(stage_module / W.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME, manifest, clobber=False)
        if VS.source_files_table(S0D_MODULE) != s0d_pre:
            raise S1FitError("SOURCE S0D changed during export")
        hist = fit_report["history"]
        receipt = {"schema": "v26b_s1_fit.1", "amendment_rev3o": PIN_AMENDMENT_REV3O, "authorized_stage": AUTHORIZED_STAGE,
                   "lineage": lineage,
                   "init": {"module": C.rel(S0D_MODULE), "actor_digest": PIN_S0D_ACTOR, "module_state_sha256": PIN_S0D_MODULE_STATE,
                            "files_sha256": s0d_pre, "exclusive": f"S0D only; {'/'.join(FORBIDDEN_INIT_NAMES)} forbidden as init and as labels"},
                   "split": view["records"], "source_holdout_rev3m": src["records"],
                   "scaling": {"table": scale_table, "T1_maxabs": t1, "T2_maxabs": gates["function_preservation"]["T2_maxabs"], "tol": G.PRESERVATION_TOL},
                   "fit": {k: fit_report[k] for k in ("tool", "rows_train", "budget", "anchor", "loss", "sigma_note")},
                   "loss_curve_essential": {"first": hist[0], "epoch_50": hist[49], "epoch_150": hist[149], "epoch_300": hist[-1]},
                   "loss_history_full": hist, "gates": gates, "structure": struct, "save_reload_exact": True,
                   "sigma_placeholder": {"value": VS.SIGMA_PLACEHOLDER, "statement": G.SIGMA_NOTE},
                   "output_module": C.rel(out_dir / "rl_module"),
                   "output_files_sha256": {p.name: C.sha256_file(p) for p in sorted(stage_module.iterdir())},
                   "code_digests": {"v26b_s1_fit.py": C.sha256_file(Path(__file__).resolve()),
                                    "test_v26b_s1_fit.py": C.sha256_file(HERE / "test_v26b_s1_fit.py") if (HERE / "test_v26b_s1_fit.py").is_file() else None,
                                    "v26b_s1_pregate_rev3n.py": C.sha256_file(HERE / "v26b_s1_pregate_rev3n.py"),
                                    "v26b_s1_prereg.py": C.sha256_file(HERE / "v26b_s1_prereg.py"),
                                    "v26b_s0d_fit.py": C.sha256_file(HERE / "v26b_s0d_fit.py"),
                                    "v26b_r2g.py": C.sha256_file(HERE / "v26b_r2g.py"), "v26b_r2i.py": C.sha256_file(HERE / "v26b_r2i.py")},
                   "scope": "S1 offline fit + offline gates ONLY: no closed-loop evaluation (separate future token), no PPO/critic/DAgger/sigma sweep/multistart/morphology fit",
                   "export_transaction": {"lock": lock.name, "promotion_policy": "native atomic no-replace rename, else F2R fallback", "completion_marker": RECEIPT_NAME},
                   "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}
        C.write_json(staging / RECEIPT_NAME, receipt)
        canonical = json.loads((staging / RECEIPT_NAME).read_text(encoding="utf-8"))
        if canonical != json.loads(json.dumps(receipt, default=str)):
            raise S1FitError("canonical receipt differs from memory")
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
        runtime_status.update({"promotion": promotion, "lock_released": bool(released),
                               "canonical_receipt_sha256": canonical_sha, "final_path": C.rel(out_dir)})
    return canonical


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="V26B rev3o S1 IK-AB06 offline fit (token-gated; offline only)")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--authorized-stage", default=None)
    args = parser.parse_args(argv)
    if not args.execute:
        verify_lineage_s1_fit()
        print(json.dumps({"mode": "dry", "lineage_ok": True}, indent=2))
        return 0
    runtime = {}
    canonical = run_s1_fit(authorized_stage=args.authorized_stage, runtime_status=runtime)
    g = canonical["gates"]
    print(json.dumps({"receipt_sha256": runtime.get("canonical_receipt_sha256"), "final_path": runtime.get("final_path"),
                      "new_actor_digest": g["actor_digest_new"], "pass_all": g["pass_all"],
                      "G_task": g["G_task"]["per_joint"], "Q1": g["Q1_source_holdout_rev3m"]["per_joint"],
                      "full500_diag": g["diagnostics"]["full_500_rmse_vs_uIK"]}, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
