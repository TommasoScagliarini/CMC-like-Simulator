"""V26B rev3p — S1P multi-role fit (token V26B-S1P-MULTIROLE-FIT).

Cause fixed: under rev3o's mono-role objective the 1e-5 parameter anchor was numerically
inert and could not preserve the S0D function.  rev3p keeps every threshold, label and
numeric of the chain and changes ONLY the objective, adopting the proven F2R T1R
group-balanced two-role mechanism:

    L = mean_task(row MSE vs u_IK) + beta * mean_pres(row MSE vs the S0D mean)
        + 1.0 * action-clip penalty (all batch rows) + 1e-5 * July exact anchor -> theta_S0D

beta = 1.0 frozen a priori (T1R recipe).  Per-role means are computed separately inside each
mini-batch drawn from ONE deterministic shuffled union of the two roles; a role absent from a
batch contributes exactly zero and such batches are counted.  No target-variance
normalisation, no auxiliary phase head, no masses.

Roles: task = the 380 rev3o rows (steps 1-190, 311-500) with same-time AB06 u_IK; preservation
= the 14834 unique rev3k train rows relabelled with the pinned S0D deterministic mean.  The
rev3m source holdout (4474 rows) is never trained on and stays the independent Q1 gate.

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
from typing import Any, Callable, Mapping

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import v26b_s1_fit as FIT  # noqa: E402   (rev3o roles/gates/pins, unmodified)
import v26b_s0d as S0  # noqa: E402       (rev3k split, read-only)
import v26b_r2g as G  # noqa: E402        (scaling/preservation, read-only)
import v26b_r2i as I  # noqa: E402        (anchor_loss_july_exact, read-only)
import v26b_anchors as VA  # noqa: E402
import v26b_student as VS  # noqa: E402
import f0_common as C  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_refit as RF  # noqa: E402


class S1PError(RuntimeError):
    pass


AUTHORIZED_STAGE = "V26B-S1P-MULTIROLE-FIT"
AMENDMENT_REV3P = HERE / "v26b_amendment_rev3p_s1_multirole_fit.json"
PIN_AMENDMENT_REV3P = "617440544a6193646b1e5f8e5d3e4d91f56f36e93b9accfe6fbfd5ebbf3d21e6"
REV3O_REJECTED = VA.OUT_ROOT / "v26b_s1_fit_REJECTED_20260824_190422.json"
PIN_REV3O_REJECTED = "a559bd691eb5934f32a820e46e13660f010f0b54e445c82d4f13e485154d558a"
PIN_REV3O_TOOL = "a6d7163ef8089ca9efb5b8b31fe92d2132efa357b1c69d814f6d9531ea845916"

BETA = 1.0                                   # T1R recipe, frozen a priori
EXPECTED_PRES_ROWS = 14834
BUDGET = dict(FIT.BUDGET)                    # 300 / 256 / 1e-4 / seed 2026 / clip 1.0
ANCHOR_WEIGHT = FIT.ANCHOR_WEIGHT            # 1e-5 July 11/07, by reference
G_TASK_MAX = FIT.G_TASK_MAX                  # 0.15, by reference
Q1_MAX = FIT.Q1_MAX                          # 0.10, by reference
OUT_S1P = VA.OUT_ROOT / "student" / "S1P_IK_AB06_MULTIROLE_35D"
RECEIPT_NAME = "v26b_s1p_fit_receipt.json"


# --- lineage -------------------------------------------------------------------------------------

def verify_lineage_s1p() -> dict[str, Any]:
    lin = FIT.verify_lineage_s1_fit()        # rev3l/m/n/o + S0D chain + AB06 cache + additivity proof
    got = C.sha256_file(AMENDMENT_REV3P)
    if got != PIN_AMENDMENT_REV3P:
        raise S1PError(f"rev3p sha {got} != pinned")
    lin["amendment_rev3p"] = got
    got = C.sha256_file(REV3O_REJECTED)
    if got != PIN_REV3O_REJECTED:
        raise S1PError(f"rev3o REJECTED artifact sha {got} != pinned (must stay immutable)")
    lin["rev3o_rejected_artifact"] = got
    got = C.sha256_file(HERE / "v26b_s1_fit.py")
    if got != PIN_REV3O_TOOL:
        raise S1PError(f"rev3o tool sha {got} != pinned (rev3p must stay additive)")
    lin["rev3o_tool"] = got
    if FIT.OUT_S1.exists():
        raise S1PError(f"rev3o published a checkpoint at {FIT.OUT_S1}: contradicts the REJECTED record")
    lin["rev3o_no_checkpoint_published"] = True
    return lin


# --- roles ---------------------------------------------------------------------------------------

def build_roles(init_raw: Mapping[str, Any]) -> dict[str, Any]:
    """Task = the 380 rev3o rows (u_IK labels); preservation = the 14834 rev3k train rows
    relabelled with the pinned S0D deterministic mean.  Every leakage claim re-checked fail-closed."""
    view = FIT.build_s1_task()
    src = FIT.build_source_holdout(view, init_raw)       # rev3m gate set (4474), never trained
    split = S0.build_split()
    tr = split["assign"]["train_idx"]
    pres_obs = np.asarray(split["obs"])[tr].astype(np.float32)
    if pres_obs.shape[0] != EXPECTED_PRES_ROWS:
        raise S1PError(f"preservation rows {pres_obs.shape[0]} != {EXPECTED_PRES_ROWS}")
    pres_keys = [pres_obs[i].tobytes() for i in range(pres_obs.shape[0])]
    if len(set(pres_keys)) != EXPECTED_PRES_ROWS:
        raise S1PError("preservation rows are not bitwise unique")
    task_keys = {view["obs"][i].tobytes() for i in range(view["obs"].shape[0])}
    n_task_overlap = sum(k in task_keys for k in pres_keys)
    if n_task_overlap != 0:
        raise S1PError(f"LEAKAGE: {n_task_overlap} preservation rows collide bitwise with the 500 task states")
    gate_keys = {src["obs"][i].tobytes() for i in range(src["obs"].shape[0])}
    n_gate_overlap = sum(k in gate_keys for k in pres_keys)
    if n_gate_overlap != 0:
        raise S1PError(f"LEAKAGE: {n_gate_overlap} preservation rows collide bitwise with the rev3m source holdout")
    pres_labels = np.asarray(RF.numpy_mean(init_raw, pres_obs), dtype=np.float32)
    task_mask = view["train"]
    records = {
        "task": {"rows": int(task_mask.sum()), "steps": [[1, 190], [311, 500]],
                 "labels": "same-time AB06 u_IK (pinned nominal cache)",
                 "obs_sha256": view["records"]["train_obs_sha256"], "labels_sha256": view["records"]["train_labels_sha256"]},
        "preservation_train": {"rows": int(pres_obs.shape[0]), "source": "rev3k S0D split train_idx (unique obs35)",
                               "labels": "S0D deterministic mean on the SAME obs35 (numpy forward of the pinned init)",
                               "obs_sha256": hashlib.sha256(pres_obs.tobytes()).hexdigest(),
                               "labels_sha256": hashlib.sha256(pres_labels.tobytes()).hexdigest(),
                               "bitwise_unique": True},
        "leakage_checks_fail_closed": {"pres_vs_500_task_states": n_task_overlap,
                                       "pres_vs_rev3m_source_holdout": n_gate_overlap,
                                       "task_rows_inside_excluded_steps": 0,
                                       "verdict": "zero overlap re-confirmed at execution time"},
        "gate_set_rev3m": src["records"], "union_rows": int(task_mask.sum() + pres_obs.shape[0]),
    }
    return {"view": view, "src": src, "pres_obs": pres_obs, "pres_labels": pres_labels, "records": records}


# --- objective -----------------------------------------------------------------------------------

def role_term(m: Any, y: Any, mask: Any) -> Any:
    """Group-balanced FLAT MSE of ONE role inside a mini-batch: mean over the role's rows and the
    2 outputs, independent of how many rows of the role the batch holds; exactly zero when the
    batch holds none (proven T1R mechanism, without the T1R variance normalisation)."""
    import torch

    if not bool(mask.any()):
        return torch.zeros((), dtype=m.dtype)
    return ((m[mask] - y[mask]) ** 2).mean()


def fit_s1p(init_scaled: Mapping[str, Any], roles: Mapping[str, Any], vec: np.ndarray, *,
            budget: Mapping[str, Any] | None = None, beta: float = BETA, progress: bool = False) -> tuple[dict[str, Any], dict[str, Any]]:
    import torch

    budget = dict(budget or BUDGET)
    if "patience" in budget or "validation_fraction" in budget:
        raise S1PError("no early stopping / validation model selection (frozen chain numerics)")
    if not (float(beta) > 0.0 and np.isfinite(beta)):
        raise S1PError("beta must be a positive finite number")
    f32 = np.float32
    view = roles["view"]; tr = view["train"]
    task_obs = (view["obs"].astype(np.float64)[tr] / vec[None, :]).astype(f32)
    pres_obs = (roles["pres_obs"].astype(np.float64) / vec[None, :]).astype(f32)
    task_y = view["u_ik"][tr].astype(f32); pres_y = roles["pres_labels"].astype(f32)
    n_task, n_pres = int(task_obs.shape[0]), int(pres_obs.shape[0])
    obs_t = torch.as_tensor(np.concatenate([task_obs, pres_obs]))
    tgt_t = torch.as_tensor(np.concatenate([task_y, pres_y]))
    is_task = torch.as_tensor(np.concatenate([np.ones(n_task, bool), np.zeros(n_pres, bool)]))
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
    n = n_task + n_pres; epochs = int(budget["epochs"]); batch = int(budget["batch_size"])
    lam_clip = float(budget["clip_weight"]); beta = float(beta)
    history = []; steps = 0; empty_task = 0; empty_pres = 0; task_rows_seen = 0; pres_rows_seen = 0
    for epoch in range(1, epochs + 1):
        perm = rng.permutation(n); losses = []; lt = []; lp = []; anch = []
        for s0 in range(0, n, batch):
            idx = torch.as_tensor(perm[s0: s0 + batch]); opt.zero_grad(set_to_none=True)
            m = encoder(obs_t[idx]) @ W3m.T + b3m
            mb = is_task[idx]
            n_t = int(mb.sum().item()); task_rows_seen += n_t; pres_rows_seen += int(mb.numel() - n_t)
            if n_t == 0:
                empty_task += 1
            if n_t == int(mb.numel()):
                empty_pres += 1
            l_task = role_term(m, tgt_t[idx], mb)
            l_pres = role_term(m, tgt_t[idx], ~mb)
            anchor = I.anchor_loss_july_exact(mean_params, anchor_t)
            loss = (l_task + beta * l_pres
                    + lam_clip * torch.relu(torch.abs(m) - 1.0).square().mean()
                    + ANCHOR_WEIGHT * anchor)
            loss.backward(); opt.step(); steps += 1
            with torch.no_grad():
                W1.mul_(clock_mask)
            losses.append(float(loss.item())); lt.append(float(l_task.item())); lp.append(float(l_pres.item())); anch.append(float(anchor.item()))
        history.append({"epoch": epoch, "loss": float(np.mean(losses)), "task_mse": float(np.mean(lt)),
                        "pres_mse": float(np.mean(lp)), "anchor_term": float(np.mean(anch))})
        if progress and (epoch == 1 or epoch % 25 == 0):
            print(json.dumps(history[-1]), flush=True)
    hidden = int(W2.shape[0])
    logstd_w = np.zeros((R.ACTION_DIM, hidden), dtype=f32)
    logstd_b = np.asarray(np.log(np.full(R.ACTION_DIM, float(R.SIGMA_CONSTANT))), dtype=f32)
    w3 = np.concatenate([W3m.detach().numpy().astype(f32), logstd_w], axis=0)
    b3 = np.concatenate([b3m.detach().numpy().astype(f32), logstd_b], axis=0)
    scaled_fit = {"pi.0.0.weight": W1.detach().numpy().astype(f32).copy(), "pi.0.0.bias": b1.detach().numpy().astype(f32).copy(),
                  "pi.0.2.weight": W2.detach().numpy().astype(f32).copy(), "pi.0.2.bias": b2.detach().numpy().astype(f32).copy(),
                  "pi.1.weight": w3.astype(f32).copy(), "pi.1.bias": b3.astype(f32).copy()}
    batches_per_epoch = -(-n // batch)
    report = {"tool": "v26b_s1p_multirole.fit_s1p (rev3p T1R-style group-balanced two-role)",
              "beta": beta, "beta_provenance": "F2R T1R / V26B preservation recipe, frozen a priori",
              "budget": budget, "optimizer_steps": int(steps), "batches_per_epoch": int(batches_per_epoch),
              "rows": {"task": n_task, "preservation": n_pres, "union": int(n)},
              "role_masses": {"task_rows_seen_total": int(task_rows_seen), "pres_rows_seen_total": int(pres_rows_seen),
                              "mean_task_rows_per_batch": float(task_rows_seen / steps), "mean_pres_rows_per_batch": float(pres_rows_seen / steps),
                              "batches_without_task_rows": int(empty_task), "batches_without_preservation_rows": int(empty_pres),
                              "empty_role_contributes": "exactly zero for that role in that batch (T1R mechanism)"},
              "anchor": {"weight": ANCHOR_WEIGHT, "semantics": "July 11/07 exact: mean over the 6 FULL tensors (0.5 head factors) towards theta_S0D_scaled",
                         "role": "secondary regularizer only; preservation is carried by the beta-weighted role term",
                         "first_epoch": history[0]["anchor_term"], "last_epoch": history[-1]["anchor_term"],
                         "weighted_last_epoch": float(ANCHOR_WEIGHT * history[-1]["anchor_term"])},
              "loss": "mean_task(row MSE vs u_IK) + beta*mean_pres(row MSE vs S0D mean) + 1.0*clip(all rows) + 1e-5*anchor; no variance normalisation; no aux head; no masses",
              "history": history, "sigma_note": G.SIGMA_NOTE}
    return scaled_fit, report


# --- gates ---------------------------------------------------------------------------------------

def publication_decision(gates: Mapping[str, Any]) -> dict[str, Any]:
    """A checkpoint is published ONLY when every binding gate passes."""
    failed = [k for k in ("G_task", "Q1_source_holdout_rev3m", "Q3_invariants", "function_preservation")
              if not bool(gates[k]["pass"])]
    return {"publish": bool(not failed and bool(gates["pass_all"])), "failed_binding_gates": failed}


def evaluate_s1p(init_raw, export_raw, scaled_fit, roles, vec, t1) -> dict[str, Any]:
    gates = FIT.evaluate(init_raw, export_raw, scaled_fit, roles["view"], roles["src"], vec, t1)
    m_pres = RF.numpy_mean(export_raw, roles["pres_obs"])
    d = m_pres - roles["pres_labels"]
    gates["diagnostics"]["preservation_train_rmse_vs_S0D"] = np.sqrt(np.mean(d ** 2, axis=0)).tolist()
    gates["diagnostics"]["preservation_train_max_abs_vs_S0D"] = np.abs(d).max(0).tolist()
    gates["diagnostics"]["role_note"] = ("the preservation TRAIN rows are in-sample by construction; the binding "
                                         "preservation measurement is Q1 on the independent rev3m source holdout")
    return gates


# --- run -----------------------------------------------------------------------------------------

def run_s1p_fit(*, authorized_stage: str | None, out_dir: Path = OUT_S1P, reject_dir: Path = VA.OUT_ROOT,
                progress: bool = True, runtime_status: dict[str, Any] | None = None,
                _fit_fn: Callable[..., Any] | None = None) -> dict[str, Any]:
    if authorized_stage != AUTHORIZED_STAGE:
        raise S1PError(f"the S1P multi-role fit requires --authorized-stage {AUTHORIZED_STAGE}; got {authorized_stage!r}")
    lineage = verify_lineage_s1p()
    vec, scale_table = G.scale_vector()
    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    init_raw = {k: np.asarray(W._as_numpy(v)) for k, v in W.load_module_state(FIT.S0D_MODULE).items()}
    struct0 = RF.validate_init_state(init_raw, expected_actor_digest=FIT.PIN_S0D_ACTOR)
    if struct0["actor_digest"] != FIT.PIN_S0D_ACTOR:
        raise S1PError("init is not the pinned S0D actor")
    roles = build_roles(init_raw)
    init_scaled = G.transform_state_to_scaled(init_raw, vec)
    t1 = G.preservation_test(init_raw, init_scaled, roles["view"]["obs"], vec)
    if t1 > G.PRESERVATION_TOL:
        raise S1PError(f"prefit T1 FAILED: {t1:.3e}")
    scaled_fit, fit_report = (_fit_fn or fit_s1p)(init_scaled, roles, vec, progress=progress)
    export_raw = G.export_state_from_scaled(scaled_fit, vec)
    gates = evaluate_s1p(init_raw, export_raw, scaled_fit, roles, vec, t1)
    decision = publication_decision(gates)
    if not decision["publish"]:
        rej = R.unique_artifact_path(Path(reject_dir), f"v26b_s1p_fit_REJECTED_{time.strftime('%Y%m%d_%H%M%S')}", ".json")
        VA._atomic_fill_reserved(rej, json.dumps({"schema": "v26b_s1p_fit_rejected.1", "REJECTED": True,
                                                  "amendment_rev3p": PIN_AMENDMENT_REV3P, "authorized_stage": AUTHORIZED_STAGE,
                                                  "lineage": lineage, "roles": roles["records"], "gates": gates,
                                                  "publication_decision": decision,
                                                  "fit": {k: fit_report[k] for k in ("tool", "beta", "budget", "optimizer_steps", "rows", "role_masses", "anchor", "loss") if k in fit_report},
                                                  "loss_history_full": fit_report.get("history"),
                                                  "policy": "no candidate checkpoint published; thresholds/metrics/labels unchanged; rev3o artifacts untouched; STOP",
                                                  "sigma_question": "UNRESOLVED (0.005 is only the serialisation placeholder)",
                                                  "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}, indent=2, default=str) + "\n")
        raise S1PError(f"binding gates FAILED {decision['failed_binding_gates']}: no export; diagnosis at {C.rel(rej)}")
    s0d_pre = VS.source_files_table(FIT.S0D_MODULE)
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
        shutil.copy2(FIT.S0D_MODULE / "metadata.json", stage_module / "metadata.json")
        shutil.copy2(FIT.S0D_MODULE / "class_and_ctor_args.pkl", stage_module / "class_and_ctor_args.pkl")
        with (stage_module / "module_state.pkl").open("wb") as fh:
            pickle.dump({k: np.asarray(v) for k, v in export_raw.items()}, fh, protocol=pickle.HIGHEST_PROTOCOL)
        reloaded = W.load_module_state(stage_module)
        cmp = W.compare_actor_states({k: np.asarray(v) for k, v in export_raw.items()}, reloaded)
        if not cmp.get("exact"):
            raise S1PError(f"save/reload mismatch: {cmp}")
        struct = RF.validate_init_state(reloaded, expected_actor_digest=gates["actor_digest_new"])
        names35, _, manifest_shas = VS.pinned_names()
        manifest = {"schema_version": 1, "actor_feature_names": names35, "actor_feature_count": 35,
                    "actor_digest": struct["actor_digest"], "module_state_sha256": C.sha256_file(stage_module / "module_state.pkl"),
                    "manifest35_sha256": manifest_shas["manifest35_sha256"], "exploration_sigma": [VS.SIGMA_PLACEHOLDER] * R.ACTION_DIM,
                    "sigma_note": G.SIGMA_NOTE, "derived_from": C.rel(FIT.S0D_MODULE), "source_actor_digest": FIT.PIN_S0D_ACTOR,
                    "contract": "deployable_markov_controller_state",
                    "note": "S1P multi-role IK-AB06 adaptation with S0D function preservation (rev3p); July physical scaling absorbed"}
        C.write_json(stage_module / W.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME, manifest, clobber=False)
        if VS.source_files_table(FIT.S0D_MODULE) != s0d_pre:
            raise S1PError("SOURCE S0D changed during export")
        hist = fit_report["history"]
        receipt = {"schema": "v26b_s1p_fit.1", "amendment_rev3p": PIN_AMENDMENT_REV3P, "authorized_stage": AUTHORIZED_STAGE,
                   "lineage": lineage,
                   "init": {"module": C.rel(FIT.S0D_MODULE), "actor_digest": FIT.PIN_S0D_ACTOR,
                            "module_state_sha256": FIT.PIN_S0D_MODULE_STATE, "files_sha256": s0d_pre,
                            "enforcement": "positive digest pin; JUL_H0/R0a/R1/R2G/R2I/S1-rev3o forbidden as init and labels"},
                   "roles": roles["records"], "split_task_view": roles["view"]["records"],
                   "scaling": {"table": scale_table, "T1_maxabs": t1, "T2_maxabs": gates["function_preservation"]["T2_maxabs"], "tol": G.PRESERVATION_TOL},
                   "fit": {k: fit_report[k] for k in ("tool", "beta", "beta_provenance", "budget", "optimizer_steps", "batches_per_epoch", "rows", "role_masses", "anchor", "loss", "sigma_note")},
                   "loss_curve_essential": {"first": hist[0], "epoch_50": hist[49], "epoch_150": hist[149], "epoch_300": hist[-1]},
                   "loss_history_full": hist, "gates": gates, "publication_decision": decision,
                   "structure": struct, "save_reload_exact": True,
                   "sigma_placeholder": {"value": VS.SIGMA_PLACEHOLDER, "statement": G.SIGMA_NOTE,
                                         "question": "UNRESOLVED: do not assume 0.005 for any later stochastic stage"},
                   "output_module": C.rel(out_dir / "rl_module"),
                   "output_files_sha256": {p.name: C.sha256_file(p) for p in sorted(stage_module.iterdir())},
                   "code_digests": {"v26b_s1p_multirole.py": C.sha256_file(Path(__file__).resolve()),
                                    "test_v26b_s1p_multirole.py": C.sha256_file(HERE / "test_v26b_s1p_multirole.py") if (HERE / "test_v26b_s1p_multirole.py").is_file() else None,
                                    "v26b_s1_fit.py": PIN_REV3O_TOOL,
                                    "v26b_s1_pregate_rev3n.py": C.sha256_file(HERE / "v26b_s1_pregate_rev3n.py"),
                                    "v26b_s1_prereg.py": C.sha256_file(HERE / "v26b_s1_prereg.py"),
                                    "v26b_r2g.py": C.sha256_file(HERE / "v26b_r2g.py"), "v26b_r2i.py": C.sha256_file(HERE / "v26b_r2i.py")},
                   "scope": "S1P offline multi-role fit + offline gates ONLY: no closed-loop evaluation (separate future token), no PPO/critic/DAgger/sigma sweep/multistart/morphology fit",
                   "export_transaction": {"lock": lock.name, "promotion_policy": "native atomic no-replace rename, else F2R fallback", "completion_marker": RECEIPT_NAME},
                   "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}
        C.write_json(staging / RECEIPT_NAME, receipt)
        canonical = json.loads((staging / RECEIPT_NAME).read_text(encoding="utf-8"))
        if canonical != json.loads(json.dumps(receipt, default=str)):
            raise S1PError("canonical receipt differs from memory")
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
    parser = argparse.ArgumentParser(description="V26B rev3p S1P multi-role fit (token-gated; offline only)")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--authorized-stage", default=None)
    args = parser.parse_args(argv)
    if not args.execute:
        verify_lineage_s1p()
        print(json.dumps({"mode": "dry", "lineage_ok": True}, indent=2))
        return 0
    runtime = {}
    canonical = run_s1p_fit(authorized_stage=args.authorized_stage, runtime_status=runtime)
    g = canonical["gates"]
    print(json.dumps({"receipt_sha256": runtime.get("canonical_receipt_sha256"), "final_path": runtime.get("final_path"),
                      "new_actor_digest": g["actor_digest_new"], "pass_all": g["pass_all"],
                      "G_task": g["G_task"]["per_joint"], "Q1": g["Q1_source_holdout_rev3m"]["per_joint"],
                      "pres_train_rmse": g["diagnostics"]["preservation_train_rmse_vs_S0D"],
                      "full500_diag": g["diagnostics"]["full_500_rmse_vs_uIK"]}, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
