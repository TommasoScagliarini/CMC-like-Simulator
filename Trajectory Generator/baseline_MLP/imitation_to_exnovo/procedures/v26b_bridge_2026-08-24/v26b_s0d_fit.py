"""V26B rev3k — S0D FIT (token V26B-S0D-FIT granted): pure V26->35D distillation + postfit gate.

Exactly rev3k: init V1 only; 14834 unique TRAIN rows row-uniform; flat 2-output
MSE on RAW V26-mean labels; action-clip penalty 1.0 (historical); anchor = July
1e-5 * mean over the 6 FULL tensors (explicit 0.5 head factors, logstd
restored/frozen); July physical scaling with T1/T2 <= 1e-5; 300 fixed epochs /
batch 256 / lr 1e-4 / seed 2026; no aux/IK/PPO/critic.  Postfit binding: RMSE
<= 0.10 per joint on the holdout aggregate, each start and each cell with the
corrected PROVENANCE masks.  Export no-clobber only on full PASS.  NO rollout.
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
sys.path.insert(0, str(HERE))

import v26b_s0d as S  # noqa: E402
import v26b_r2g as G  # noqa: E402  (scaling/transform/preservation, read-only)
import v26b_r2i as I  # noqa: E402  (anchor_loss_july_exact, read-only)
import v26b_anchors as VA  # noqa: E402
import v26b_student as VS  # noqa: E402
import f0_common as C  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_refit as RF  # noqa: E402


class S0DFitError(RuntimeError):
    pass


AUTHORIZED_STAGE = "V26B-S0D-FIT"
PIN_PREGATE_TOOL = "b9c2f275d1c502683b459a83f2ddd00f445175ec597d871529adbf418d66cb87"
PREGATE_RECEIPT = VA.OUT_ROOT / "v26b_s0d_pregate_receipt_20260824_171225.json"
PIN_PREGATE_RECEIPT = "ee2379d1ea98f79287a0476e354605d93d4964da2c158f1def5992c290572c36"
V1_MODULE = VA.OUT_ROOT / "student" / "V1_35D_transplant" / "rl_module"
PIN_V1_ACTOR_DIGEST = "ae846220a6f7f1ac1289ccc9636e3ad2e5bc7842ba7ece0b62bb9d7590e7f587"
ANCHOR_WEIGHT_JULY_1107 = 1e-5   # rev3k: historical BC/DAgger 11/07 weight (NOT rev3i's 0.01)
POSTFIT_RMSE_MAX = 0.10          # rev3k: existing Q1 threshold
EXPECTED_TRAIN_DIAG = {"rows_beyond_abs1": 490, "clip_rmse": [0.02392823, 0.01035782]}
BUDGET = {"epochs": 300, "batch_size": 256, "lr": 1e-4, "seed": 2026, "clip_weight": 1.0}
OUT_S0D = VA.OUT_ROOT / "student" / "S0D_35D_DISTILLED"
RECEIPT_NAME = "v26b_s0d_fit_receipt.json"


def verify_lineage_fit() -> dict[str, Any]:
    lin = S.verify_lineage_s0d()
    got = C.sha256_file(HERE / "v26b_s0d.py")
    if got != PIN_PREGATE_TOOL:
        raise S0DFitError(f"pre-gate tool sha {got} != pinned {PIN_PREGATE_TOOL}")
    got = C.sha256_file(PREGATE_RECEIPT)
    if got != PIN_PREGATE_RECEIPT:
        raise S0DFitError(f"pre-gate receipt sha {got} != pinned {PIN_PREGATE_RECEIPT}")
    rec = json.loads(PREGATE_RECEIPT.read_text(encoding="utf-8"))
    if rec["pre_gate"]["pass"] is not True:
        raise S0DFitError("pre-gate receipt does not record PASS")
    lin["pregate_tool_sha256"] = PIN_PREGATE_TOOL
    lin["pregate_receipt_sha256"] = PIN_PREGATE_RECEIPT
    if C.sha256_file(V1_MODULE.parent / "v26b_v1_receipt.json") != "65b447ea331d58b457fe6df48e67697dbd6693986552411c0f9780c75ffb995c":
        raise S0DFitError("V1 receipt changed")
    return lin


def train_label_diagnostics(split: Mapping[str, Any]) -> dict[str, Any]:
    tr = split["assign"]["train_idx"]
    u = split["labels"][tr]
    over = int(np.sum(np.any(np.abs(u) > 1.0, axis=1)))
    clip_rmse = np.sqrt(np.mean((u - np.clip(u, -1.0, 1.0)) ** 2, axis=0))
    diag = {"train_rows": int(tr.size), "rows_beyond_abs1": over, "clip_rmse_train": [float(v) for v in clip_rmse],
            "note": "labels are the RAW (pre-clip) V26 mean as frozen by rev3k; the runtime env clip is a DISTINCT mechanism applied at execution, not to the labels"}
    if over != EXPECTED_TRAIN_DIAG["rows_beyond_abs1"] or any(abs(clip_rmse[j] - EXPECTED_TRAIN_DIAG["clip_rmse"][j]) > 1e-6 for j in range(2)):
        raise S0DFitError(f"train label diagnostics {diag} != Codex-verified expectations {EXPECTED_TRAIN_DIAG}")
    return diag


def fit_s0d(init_scaled: Mapping[str, Any], split: Mapping[str, Any], vec: np.ndarray, *, budget: Mapping[str, Any] | None = None, progress: bool = False) -> tuple[dict[str, Any], dict[str, Any]]:
    import torch

    budget = dict(budget or BUDGET)
    if "patience" in budget or "validation_fraction" in budget:
        raise S0DFitError("no early stopping / validation model selection (rev3k)")
    tr = split["assign"]["train_idx"]
    f32 = np.float32
    obs_scaled = (split["obs"].astype(np.float64) / vec[None, :]).astype(f32)
    obs_t = torch.as_tensor(obs_scaled[tr])
    tgt = torch.as_tensor(split["labels"][tr].astype(f32))
    torch.use_deterministic_algorithms(True); torch.set_num_threads(1); torch.manual_seed(int(budget["seed"])); np.random.seed(int(budget["seed"]))
    W1 = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.0.0.weight"], dtype=f32))); b1 = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.0.0.bias"], dtype=f32)))
    W2 = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.0.2.weight"], dtype=f32))); b2 = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.0.2.bias"], dtype=f32)))
    W3m = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.1.weight"][: R.ACTION_DIM], dtype=f32))); b3m = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.1.bias"][: R.ACTION_DIM], dtype=f32)))
    mean_params = [W1, b1, W2, b2, W3m, b3m]
    anchor_t = [p.detach().clone() for p in mean_params]  # theta_V1 in the scaled space
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
        perm = rng.permutation(n); losses = []; dmse = []
        for s0 in range(0, n, batch):
            idx = torch.as_tensor(perm[s0: s0 + batch]); opt.zero_grad(set_to_none=True)
            m = encoder(obs_t[idx]) @ W3m.T + b3m
            l_data = ((m - tgt[idx]) ** 2).mean()                       # flat row-uniform 2-output MSE
            anchor = I.anchor_loss_july_exact(mean_params, anchor_t)    # mean of 6 FULL tensors (0.5 head factors)
            loss = l_data + lam_clip * torch.relu(torch.abs(m) - 1.0).square().mean() + ANCHOR_WEIGHT_JULY_1107 * anchor
            loss.backward(); opt.step()
            with torch.no_grad():
                W1.mul_(clock_mask)
            losses.append(float(loss.item())); dmse.append(float(l_data.item()))
        history.append({"epoch": epoch, "loss": float(np.mean(losses)), "flat_mse": float(np.mean(dmse))})
        if progress and (epoch == 1 or epoch % 50 == 0):
            print(json.dumps(history[-1]), flush=True)
    w3 = np.concatenate([W3m.detach().numpy().astype(f32), logstd_w], axis=0); b3 = np.concatenate([b3m.detach().numpy().astype(f32), logstd_b], axis=0)
    scaled_fit = {"pi.0.0.weight": W1.detach().numpy().astype(f32).copy(), "pi.0.0.bias": b1.detach().numpy().astype(f32).copy(), "pi.0.2.weight": W2.detach().numpy().astype(f32).copy(), "pi.0.2.bias": b2.detach().numpy().astype(f32).copy(), "pi.1.weight": w3.astype(f32).copy(), "pi.1.bias": b3.astype(f32).copy()}
    report = {"tool": "v26b_s0d_fit.fit_s0d (rev3k pure distillation)", "rows_train": n, "budget": budget, "anchor": {"weight": ANCHOR_WEIGHT_JULY_1107, "semantics": "July 11/07 exact: mean over the 6 FULL tensors, explicit 0.5 head factors, logstd restored/frozen, towards theta_V1_scaled"}, "loss": "row-uniform flat 2-output MSE + 1.0*action-clip penalty + 1e-5*anchor; no aux; no variance normalisation; no masses", "history": history, "sigma_note": G.SIGMA_NOTE}
    return scaled_fit, report


def postfit_gate(init_raw: Mapping[str, Any], export_raw: Mapping[str, Any], scaled_fit: Mapping[str, Any], split: Mapping[str, Any], vec: np.ndarray, t1: float) -> dict[str, Any]:
    f32 = np.float32
    obs = split["obs"].astype(np.float64); u = split["labels"]
    tr = split["assign"]["train_idx"]; ho = split["assign"]["hold_idx"]
    m_exp = RF.numpy_mean(export_raw, obs.astype(f32))
    m_scaled = RF.numpy_mean(scaled_fit, (obs / vec[None, :]).astype(f32))
    t2 = float(np.max(np.abs(m_exp - m_scaled)))
    prov_h = [split["assign"]["provenance"][i] for i in ho.tolist()]
    held = split["records"]["held_jobs_per_cell"]
    d_h = m_exp[ho] - u[ho]
    def rmse_mask(mask):
        return np.sqrt(np.mean(d_h[mask] ** 2, axis=0)).tolist()
    res = {"aggregate": {"rows": int(ho.size), "rmse": rmse_mask(np.ones(ho.size, bool))}}
    for st in S.DECLARED_START_ORDER:
        sj = {held[f"{st}|{sg}"] for sg in S.DECLARED_SIGMA_ORDER}
        m = np.asarray([bool(p & sj) for p in prov_h])
        res[f"start_{st}"] = {"rows": int(m.sum()), "rmse": rmse_mask(m)}
    for st in S.DECLARED_START_ORDER:
        for sg in S.DECLARED_SIGMA_ORDER:
            job = held[f"{st}|{sg}"]
            m = np.asarray([job in p for p in prov_h])
            res[f"cell_{st}_{sg}"] = {"rows": int(m.sum()), "rmse": rmse_mask(m)}
    binding_pass = all(all(x <= POSTFIT_RMSE_MAX for x in v["rmse"]) for v in res.values())
    d_t = m_exp[tr] - u[tr]
    train_rmse = np.sqrt(np.mean(d_t ** 2, axis=0)).tolist()
    struct = RF.validate_init_state(export_raw, expected_actor_digest=None)
    inv = RF.invariance_test(export_raw, obs.astype(f32)[:64])
    q3 = {"ten_keys": tuple(export_raw.keys()) == RF.EXPECTED_KEY_ORDER, "clock_columns_zero": bool(struct["clock_columns_zero"]), "logstd_constant_placeholder": bool(struct["sigma_head"]["logstd_bias_exact"]), "clock_invariance_bit_identical": bool(inv["bit_identical"]), "no_critic": True, "save_reload_exact": "verified at export"}
    out = {"threshold_per_joint": POSTFIT_RMSE_MAX, "threshold_source": "existing Q1 (rev3 protocol), rev3k",
           "holdout_binding_provenance_masks": res, "pass_binding": bool(binding_pass),
           "train_metrics_distinct": {"rows": int(tr.size), "rmse": train_rmse},
           "function_preservation": {"T1_maxabs": t1, "T2_maxabs": t2, "tol": G.PRESERVATION_TOL, "pass": bool(t1 <= G.PRESERVATION_TOL and t2 <= G.PRESERVATION_TOL)},
           "Q3": {**q3, "pass": bool(all(v is True for k, v in q3.items() if isinstance(v, bool)))},
           "actor_digest_new": struct["actor_digest"],
           "parameter_shift_sq_vs_V1_raw": float(sum(np.sum((np.asarray(export_raw[k], np.float64) - np.asarray(init_raw[k], np.float64)) ** 2) for k in RF.CANONICAL_KEYS)),
           "sigma_note": G.SIGMA_NOTE}
    out["pass_postfit"] = bool(binding_pass and out["Q3"]["pass"] and out["function_preservation"]["pass"])
    return out


def run_s0d_fit(*, authorized_stage: str | None, out_dir: Path = OUT_S0D, progress: bool = True, runtime_status=None) -> dict[str, Any]:
    if authorized_stage != AUTHORIZED_STAGE:
        raise S0DFitError(f"S0D fit requires --authorized-stage {AUTHORIZED_STAGE}; got {authorized_stage!r}")
    lineage = verify_lineage_fit()
    vec, scale_table = G.scale_vector()
    split = S.build_split()
    rec = json.loads(PREGATE_RECEIPT.read_text(encoding="utf-8"))
    if split["records"]["train_idx_sha256"] != rec["split"]["train_idx_sha256"] or split["records"]["holdout_idx_sha256"] != rec["split"]["holdout_idx_sha256"]:
        raise S0DFitError("rebuilt split differs from the PASS pre-gate receipt")
    diag = train_label_diagnostics(split)
    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    init_raw = {k: np.asarray(W._as_numpy(v)) for k, v in W.load_module_state(V1_MODULE).items()}
    RF.validate_init_state(init_raw, expected_actor_digest=PIN_V1_ACTOR_DIGEST)
    init_scaled = G.transform_state_to_scaled(init_raw, vec)
    t1 = G.preservation_test(init_raw, init_scaled, split["obs"], vec)
    if t1 > G.PRESERVATION_TOL:
        raise S0DFitError(f"T1 FAILED: {t1:.3e}")
    scaled_fit, fit_report = fit_s0d(init_scaled, split, vec, progress=progress)
    export_raw = G.export_state_from_scaled(scaled_fit, vec)
    post = postfit_gate(init_raw, export_raw, scaled_fit, split, vec, t1)
    if not post["pass_postfit"]:
        rej = R.unique_artifact_path(VA.OUT_ROOT, f"v26b_s0d_fit_REJECTED_{time.strftime('%Y%m%d_%H%M%S')}", ".json")
        VA._atomic_fill_reserved(rej, json.dumps({"schema": "v26b_s0d_fit_rejected.1", "REJECTED": True, "postfit": post, "train_diag": diag}, indent=2, default=str) + "\n")
        raise S0DFitError(f"postfit gate FAILED: no export; diagnosis at {C.rel(rej)}")
    # export (transactional; template = V1 module serialization)
    v1_pre = VS.source_files_table(V1_MODULE)
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
        shutil.copy2(V1_MODULE / "metadata.json", stage_module / "metadata.json")
        shutil.copy2(V1_MODULE / "class_and_ctor_args.pkl", stage_module / "class_and_ctor_args.pkl")
        with (stage_module / "module_state.pkl").open("wb") as fh:
            pickle.dump({k: np.asarray(v) for k, v in export_raw.items()}, fh, protocol=pickle.HIGHEST_PROTOCOL)
        reloaded = W.load_module_state(stage_module)
        cmp = W.compare_actor_states({k: np.asarray(v) for k, v in export_raw.items()}, reloaded)
        if not cmp.get("exact"):
            raise S0DFitError(f"save/reload mismatch: {cmp}")
        struct = RF.validate_init_state(reloaded, expected_actor_digest=post["actor_digest_new"])
        names35, _, manifest_shas = VS.pinned_names()
        manifest = {"schema_version": 1, "actor_feature_names": names35, "actor_feature_count": 35, "actor_digest": struct["actor_digest"], "module_state_sha256": C.sha256_file(stage_module / "module_state.pkl"), "manifest35_sha256": manifest_shas["manifest35_sha256"], "exploration_sigma": [VS.SIGMA_PLACEHOLDER] * R.ACTION_DIM, "sigma_note": G.SIGMA_NOTE, "derived_from": C.rel(V1_MODULE), "source_actor_digest": PIN_V1_ACTOR_DIGEST, "contract": "deployable_markov_controller_state", "note": "S0D pure V26 distillation (rev3k); July physical scaling absorbed: raw physical observations"}
        C.write_json(stage_module / W.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME, manifest, clobber=False)
        if VS.source_files_table(V1_MODULE) != v1_pre:
            raise S0DFitError("SOURCE V1 changed during export")
        hist = fit_report["history"]
        receipt = {"schema": "v26b_s0d_fit.1", "amendment_rev3k": S.PIN_AMENDMENT_REV3K, "authorized_stage": AUTHORIZED_STAGE,
                   "lineage": lineage, "pregate": {"tool_sha256": PIN_PREGATE_TOOL, "receipt_sha256": PIN_PREGATE_RECEIPT, "pass": True},
                   "init": {"module": C.rel(V1_MODULE), "actor_digest": PIN_V1_ACTOR_DIGEST, "files_sha256": v1_pre},
                   "split": split["records"], "train_label_diagnostics": diag,
                   "scaling": {"table": scale_table, "T1_maxabs": t1, "T2_maxabs": post["function_preservation"]["T2_maxabs"], "tol": G.PRESERVATION_TOL},
                   "fit": {k: fit_report[k] for k in ("tool", "rows_train", "budget", "anchor", "loss", "sigma_note")},
                   "loss_history_first_last": {"first": hist[0], "epoch_150": hist[149], "last": hist[-1]}, "loss_history_full": hist,
                   "postfit_gates": post, "structure": struct, "save_reload_exact": True,
                   "sigma_placeholder": {"value": VS.SIGMA_PLACEHOLDER, "statement": G.SIGMA_NOTE},
                   "output_module": C.rel(out_dir / "rl_module"), "output_files_sha256": {p.name: C.sha256_file(p) for p in sorted(stage_module.iterdir())},
                   "code_digests": {"v26b_s0d_fit.py": C.sha256_file(Path(__file__).resolve()), "test_v26b_s0d_fit.py": C.sha256_file(HERE / "test_v26b_s0d_fit.py") if (HERE / "test_v26b_s0d_fit.py").is_file() else None, "v26b_s0d.py": PIN_PREGATE_TOOL, "v26b_r2g.py": C.sha256_file(HERE / "v26b_r2g.py"), "v26b_r2i.py": C.sha256_file(HERE / "v26b_r2i.py")},
                   "scope": "S0D fit + offline gate ONLY: no rollout (separate future stage, not authorised), no PPO/critic/IK/DAgger/sigma sweep",
                   "export_transaction": {"lock": lock.name, "promotion_policy": "native atomic no-replace rename, else F2R fallback", "completion_marker": RECEIPT_NAME},
                   "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}
        C.write_json(staging / RECEIPT_NAME, receipt)
        canonical = json.loads((staging / RECEIPT_NAME).read_text(encoding="utf-8"))
        if canonical != json.loads(json.dumps(receipt, default=str)):
            raise S0DFitError("canonical receipt differs from memory")
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
    parser = argparse.ArgumentParser(description="V26B rev3k S0D fit (token-gated; NO rollout)")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--authorized-stage", default=None)
    args = parser.parse_args(argv)
    if not args.execute:
        verify_lineage_fit()
        print(json.dumps({"mode": "dry", "lineage_ok": True}, indent=2))
        return 0
    runtime = {}
    canonical = run_s0d_fit(authorized_stage=args.authorized_stage, runtime_status=runtime)
    print(json.dumps({"receipt_sha256": runtime.get("canonical_receipt_sha256"), "final_path": runtime.get("final_path"), "new_actor_digest": canonical["postfit_gates"]["actor_digest_new"], "postfit_pass": canonical["postfit_gates"]["pass_postfit"]}, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
