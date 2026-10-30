"""V26B rev3x — S1C-1 offline fit of the three preregistered phase-weighted candidates.

Token V26B-S1C-1 (offline fit only).  Each candidate starts FRESH from the pinned S0D and trains
the full 35D mean network with the rev3t two-role loss where the TASK role uses a preregistered
phase-localised weight w on the push-off rows (decoded IK ankle target negative):

    L = r * mean_anchor(MSE vs u_S0D) + 1 * weighted_mean_task(MSE vs u_IK) + 1.0*clip + 1e-5*anchor

Binding offline gates: the rev3s hierarchy I -> P -> T -> D, thresholds unchanged.  The rev3v
quality gates G1..G5 are CLOSED-LOOP and cannot be evaluated here; only non-binding surrogates are
reported.  The preregistered causal diagnostic measures command PERSISTENCE and replays it through
the rev3w validated production replica.

No rollout, episode, collection, promotion, sigma or production change.
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

import v26b_s1c0_probe as PR  # noqa: E402       (rev3w validated production replica)
import v26b_s1c_protocol as SC  # noqa: E402     (decode, quality metrics)
import v26b_s1b_fit as T  # noqa: E402           (rev3t fit machinery, unmodified)
import v26b_s1b_protocol as B  # noqa: E402      (rev3s corpus + offline hierarchy, unmodified)
import v26b_s1b_rollout as SB  # noqa: E402
import v26b_s1a_bc as A  # noqa: E402
import v26b_s1_fit as FIT  # noqa: E402
import v26b_s0d_rollout as SR  # noqa: E402
import v26b_r2g as G  # noqa: E402
import v26b_r2i as I  # noqa: E402
import v26b_anchors as VA  # noqa: E402
import v26b_student as VS  # noqa: E402
import f0_common as C  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_refit as RF  # noqa: E402


class S1C1Error(RuntimeError):
    pass


AUTHORIZED_STAGE = "V26B-S1C-1"
AMENDMENT_REV3X = HERE / "v26b_amendment_rev3x_s1c1_fit.json"
PIN_AMENDMENT_REV3X = "041bbc38dec9209d1b0420be8b641da4224fb256f11e420204710476e871f433"
ADDENDUM_REV3W_A = HERE / "v26b_addendum_rev3w_a_probe_corrections.json"
PIN_ADDENDUM_REV3W_A = "7c4f39b9e0e2e2a579a189dffd35807ba71cada88c0e2b20c2a25c9f8214935b"
FUTURE_ROLLOUT_TOKEN = "V26B-S1C-NOMINAL-ROLLOUT"

R_RATIO = 5.0
CANDIDATES = ({"id": "W2", "w": 2.0}, {"id": "W4", "w": 4.0}, {"id": "W8", "w": 8.0})
EPOCHS = 300
CAND_ROOT = VA.OUT_ROOT / "candidates"
RECEIPT_NAME = "v26b_s1c1_fit_receipt.json"
AGG_NAME = "v26b_s1c1_fit_aggregate"


def out_dir_for(cid: str) -> Path:
    return CAND_ROOT / f"S1C1_{cid}_35D_NONDEPLOYABLE"


def verify_lineage() -> dict[str, Any]:
    lin = PR.verify_lineage_probe()
    for path, pin, key in ((AMENDMENT_REV3X, PIN_AMENDMENT_REV3X, "amendment_rev3x"),
                           (ADDENDUM_REV3W_A, PIN_ADDENDUM_REV3W_A, "addendum_rev3w_a")):
        got = C.sha256_file(path)
        if got != pin:
            raise S1C1Error(f"{key} sha {got} != pinned")
        lin[key] = got
    rec = json.loads(PR.AMENDMENT_REV3W.read_text(encoding="utf-8"))
    lin["probe_verdict_of_record"] = "REACHABLE (probe receipt 00e6141f...)"
    return lin


def push_off_mask(view: Mapping[str, Any]) -> np.ndarray:
    """Frozen definition: rows whose decoded AB06 IK ankle target is negative."""
    return SC.decode_action(view["u_ik"])[:, 1] < 0.0


def weighted_role_term(m, y, mask, weights):
    """Weighted per-role mean: sum(w*mse)/sum(w) over the role's rows in the batch; 0 if absent."""
    import torch
    if not bool(mask.any()):
        return torch.zeros((), dtype=m.dtype)
    se = ((m[mask] - y[mask]) ** 2).mean(dim=1)
    w = weights[mask]
    return (w * se).sum() / w.sum()


def fit_candidate(init_scaled, corpus, vec, w: float, *, epochs: int = EPOCHS, progress: bool = False):
    import torch
    f32 = np.float32
    view = corpus["view"]; tr = view["train"]
    obs_s = (view["obs"].astype(np.float64)[tr] / vec[None, :]).astype(f32)
    y_anchor = corpus["own"][tr].astype(f32); y_task = view["u_ik"][tr].astype(f32)
    push = push_off_mask(view)[tr]
    n_role = int(obs_s.shape[0])
    obs_t = torch.as_tensor(np.concatenate([obs_s, obs_s]))
    tgt_t = torch.as_tensor(np.concatenate([y_anchor, y_task]))
    is_anchor = torch.as_tensor(np.concatenate([np.ones(n_role, bool), np.zeros(n_role, bool)]))
    w_row = np.concatenate([np.ones(n_role, np.float32), np.where(push, np.float32(w), np.float32(1.0))])
    w_t = torch.as_tensor(w_row)
    torch.use_deterministic_algorithms(True); torch.set_num_threads(1)
    torch.manual_seed(T.SEED); np.random.seed(T.SEED)
    P = lambda k: torch.nn.Parameter(torch.as_tensor(np.array(init_scaled[k], dtype=f32)))  # noqa: E731
    W1, b1, W2, b2 = P("pi.0.0.weight"), P("pi.0.0.bias"), P("pi.0.2.weight"), P("pi.0.2.bias")
    W3m = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.1.weight"][: R.ACTION_DIM], dtype=f32)))
    b3m = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.1.bias"][: R.ACTION_DIM], dtype=f32)))
    mean_params = [W1, b1, W2, b2, W3m, b3m]
    anchor_t = [p.detach().clone() for p in mean_params]
    clock_mask = torch.ones_like(W1); clock_mask[:, list(R.CLOCK_COLUMNS)] = 0.0
    with torch.no_grad():
        W1.mul_(clock_mask)
    enc = lambda x: torch.tanh(torch.tanh(x @ W1.T + b1) @ W2.T + b2)  # noqa: E731
    opt = torch.optim.Adam(mean_params, lr=T.LR)
    rng = np.random.default_rng(T.SEED)
    n = 2 * n_role; steps = 0; empty_a = empty_t = 0; hist = []
    for epoch in range(1, int(epochs) + 1):
        perm = rng.permutation(n); ls, la, lt, an = [], [], [], []
        for s0 in range(0, n, T.BATCH):
            idx = torch.as_tensor(perm[s0: s0 + T.BATCH]); opt.zero_grad(set_to_none=True)
            m = enc(obs_t[idx]) @ W3m.T + b3m
            mb = is_anchor[idx]
            if not bool(mb.any()):
                empty_a += 1
            if not bool((~mb).any()):
                empty_t += 1
            l_a = weighted_role_term(m, tgt_t[idx], mb, torch.ones_like(w_t[idx]))
            l_t = weighted_role_term(m, tgt_t[idx], ~mb, w_t[idx])
            anchor = I.anchor_loss_july_exact(mean_params, anchor_t)
            loss = (R_RATIO * l_a + 1.0 * l_t
                    + T.CLIP_W * torch.relu(torch.abs(m) - 1.0).square().mean()
                    + T.ANCHOR_W * anchor)
            loss.backward(); opt.step(); steps += 1
            with torch.no_grad():
                W1.mul_(clock_mask)
            ls.append(float(loss.item())); la.append(float(l_a.item())); lt.append(float(l_t.item())); an.append(float(anchor.item()))
        hist.append({"epoch": epoch, "loss": float(np.mean(ls)), "anchor_role": float(np.mean(la)),
                     "task_role_weighted": float(np.mean(lt)), "param_anchor": float(np.mean(an))})
        if progress and (epoch == 1 or epoch % 100 == 0 or epoch == int(epochs)):
            print(json.dumps({"w": w, **hist[-1]}), flush=True)
    hidden = int(W2.shape[0])
    lw = np.zeros((R.ACTION_DIM, hidden), dtype=f32)
    lb = np.asarray(np.log(np.full(R.ACTION_DIM, float(R.SIGMA_CONSTANT))), dtype=f32)
    scaled = {"pi.0.0.weight": W1.detach().numpy().astype(f32).copy(), "pi.0.0.bias": b1.detach().numpy().astype(f32).copy(),
              "pi.0.2.weight": W2.detach().numpy().astype(f32).copy(), "pi.0.2.bias": b2.detach().numpy().astype(f32).copy(),
              "pi.1.weight": np.concatenate([W3m.detach().numpy().astype(f32), lw], 0).copy(),
              "pi.1.bias": np.concatenate([b3m.detach().numpy().astype(f32), lb], 0).copy()}
    rep = {"w": float(w), "r": R_RATIO, "epochs": int(epochs), "optimizer_steps": steps,
           "push_off_rows": int(push.sum()), "task_rows": n_role, "union_rows": n,
           "batches_without_anchor_rows": empty_a, "batches_without_task_rows": empty_t,
           "task_weighting": "weighted per-role mean sum(w*mse)/sum(w); w on push-off rows, 1 elsewhere",
           "history": hist}
    return scaled, rep


# --- non-binding surrogates + preregistered diagnostic ---------------------------------------------------

def run_lengths(flags) -> list[int]:
    """Lengths of the maximal runs of consecutive True values (preregistered persistence metric)."""
    runs, cur = [], 0
    for f in flags:
        if bool(f):
            cur += 1
        elif cur:
            runs.append(cur); cur = 0
    if cur:
        runs.append(cur)
    return runs


def commanded_surrogates(state_raw, corpus, label: str) -> dict[str, Any]:
    """G2-G4 SURROGATE in COMMANDED-JOINT space on the frozen 500 rows. Never a gate."""
    view = corpus["view"]
    obs32 = view["obs"].astype(np.float32)
    cmd = SC.decode_action(RF.numpy_mean(state_raw, obs32))
    ik = SC.decode_action(view["u_ik"])
    healthy = np.asarray(corpus["healthy"])
    out = {"label": label, "status": "SURROGATE ONLY - not the rev3v gate, cannot substitute it, never promotes"}
    for tag, ref in (("vs_IK_commanded", ik), ("vs_healthy_commanded", healthy)):
        blk = {}
        for j, jn in ((0, "knee"), (1, "ankle")):
            x, y = cmd[:, j], ref[:, j]
            sx, sy = float(x.std()), float(y.std())
            blk[jn] = {"rmse": float(np.sqrt(np.mean((x - y) ** 2))),
                       "pearson_r": (float(np.corrcoef(x, y)[0, 1]) if sx > 0 and sy > 0 else None),
                       "amplitude_ratio": (sx / sy if sy > 0 else None)}
        out[tag] = blk
    return out


def persistence_diagnostic(state_raw, corpus, label: str) -> dict[str, Any]:
    """PREREGISTERED (rev3x): persistence of the negative ankle command on the ORDERED held-out rows,
    plus the predicted served reference through the rev3w validated production replica."""
    view = corpus["view"]; ho = view["hold"]
    obs32 = view["obs"].astype(np.float32)[ho]
    cmd = SC.decode_action(RF.numpy_mean(state_raw, obs32))[:, 1]
    neg = cmd < 0.0
    runs = run_lengths(neg)
    q_cmd = PR.slew_limit(cmd, anchor0=float(cmd[0]))
    q, v, a = float(np.asarray(view["obs"], np.float64)[ho][0, PR.IDX["pros_ankle_angle_served_ref"]]), 0.0, 0.0
    served = []
    for qc in q_cmd:
        q, v, a = PR.reference_step(q, v, a, float(qc))
        served.append(q)
    served = np.asarray(served)
    return {"label": label, "rows": int(neg.size),
            "duty_cycle_negative_command": float(neg.mean()),
            "runs": {"count": len(runs), "mean_length": (float(np.mean(runs)) if runs else 0.0),
                     "max_length": (int(max(runs)) if runs else 0)},
            "commanded_ankle": {"min": float(cmd.min()), "mean": float(cmd.mean())},
            "predicted_served_reference_open_loop": {"min": float(served.min()),
                                                     "frac_negative": float(np.mean(served < 0.0)),
                                                     "final": float(served[-1])},
            "replica": "rev3w validated production replica (slew limiter + third-order jerk-limited model)",
            "limitation": "OPEN-LOOP COUNTERFACTUAL on S0D-visited states: it is not what this actor would visit in closed loop",
            "B3_verdict": "INDETERMINATE - the phase field is identically zero, so no rigorous B3 statement is possible",
            "is_a_gate": False}


def run_fit(*, authorized_stage: str | None, progress: bool = True) -> dict[str, Any]:
    if authorized_stage != AUTHORIZED_STAGE:
        raise S1C1Error(f"the S1C-1 fit requires --authorized-stage {AUTHORIZED_STAGE}; got {authorized_stage!r}")
    lineage = verify_lineage()
    corpus = B.build_corpus()
    cc = __import__("f2r_labeller").load_cache(R.OUT_CACHE, "nominal")
    corpus["healthy"] = np.asarray(cc.targets)[:, [0, 2]]
    vec, scale_table = G.scale_vector()
    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    s0d_raw = T.fresh_init()
    refs = {"S0D": {"surrogates": commanded_surrogates(s0d_raw, corpus, "S0D"),
                    "persistence": persistence_diagnostic(s0d_raw, corpus, "S0D")}}
    a2_raw = {k: np.asarray(W._as_numpy(v)) for k, v in W.load_module_state(SB.CANDIDATE_MODULE).items()}
    refs["A2"] = {"surrogates": commanded_surrogates(a2_raw, corpus, "A2"),
                  "persistence": persistence_diagnostic(a2_raw, corpus, "A2")}
    ik_cmd = SC.decode_action(corpus["view"]["u_ik"])[:, 1][corpus["view"]["hold"]]
    negs = ik_cmd < 0.0
    runs = run_lengths(negs)
    refs["AB06_IK_target"] = {"persistence": {"label": "AB06_IK_target", "rows": int(negs.size),
                                              "duty_cycle_negative_command": float(negs.mean()),
                                              "runs": {"count": len(runs), "mean_length": (float(np.mean(runs)) if runs else 0.0),
                                                       "max_length": (int(max(runs)) if runs else 0)},
                                              "commanded_ankle": {"min": float(ik_cmd.min()), "mean": float(ik_cmd.mean())},
                                              "is_a_gate": False}}
    results = []
    for cand in CANDIDATES:
        cid, w = cand["id"], cand["w"]
        out_dir = out_dir_for(cid)
        if out_dir.exists():
            raise FileExistsError(f"no-clobber: {out_dir} exists (each candidate is fitted exactly once)")
        init_raw = T.fresh_init()
        init_scaled = G.transform_state_to_scaled(init_raw, vec)
        t1 = G.preservation_test(init_raw, init_scaled, corpus["view"]["obs"], vec)
        if t1 > G.PRESERVATION_TOL:
            raise S1C1Error(f"{cid}: prefit T1 FAILED {t1:.3e}")
        scaled, frep = fit_candidate(init_scaled, corpus, vec, w, progress=progress)
        export_raw = G.export_state_from_scaled(scaled, vec)
        out_dir.parent.mkdir(parents=True, exist_ok=True)
        lock, tok = RF.acquire_export_lock(out_dir)
        staging = None; promoted = False
        try:
            staging = RF._staging_dir_for(out_dir)
            sm = staging / "rl_module"; sm.mkdir(parents=True, exist_ok=False)
            shutil.copy2(SR.S0D_MODULE / "metadata.json", sm / "metadata.json")
            shutil.copy2(SR.S0D_MODULE / "class_and_ctor_args.pkl", sm / "class_and_ctor_args.pkl")
            with (sm / "module_state.pkl").open("wb") as fh:
                pickle.dump({k: np.asarray(v) for k, v in export_raw.items()}, fh, protocol=pickle.HIGHEST_PROTOCOL)
            reloaded = W.load_module_state(sm)
            exact = bool(W.compare_actor_states({k: np.asarray(v) for k, v in export_raw.items()}, reloaded).get("exact"))
            if not exact:
                raise S1C1Error(f"{cid}: save/reload mismatch")
            metrics = T.evaluate_candidate(init_raw, export_raw, scaled, corpus, vec, t1, exact)
            hier = B.offline_hierarchy(metrics)
            quarantined = hier["verdict"] != "PASS"
            sur = commanded_surrogates(export_raw, corpus, cid)
            per = persistence_diagnostic(export_raw, corpus, cid)
            names35, _, mshas = VS.pinned_names()
            manifest = {"schema_version": 1, "actor_feature_names": names35, "actor_feature_count": 35,
                        "actor_digest": metrics["_actor_digest"], "module_state_sha256": C.sha256_file(sm / "module_state.pkl"),
                        "manifest35_sha256": mshas["manifest35_sha256"],
                        "exploration_sigma": [VS.SIGMA_PLACEHOLDER] * R.ACTION_DIM, "sigma_note": G.SIGMA_NOTE,
                        "derived_from": C.rel(SR.S0D_MODULE), "source_actor_digest": FIT.PIN_S0D_ACTOR,
                        "contract": A.CONTRACT_STRING, **A.MANDATORY_FLAGS,
                        "candidate_id": cid, "phase_weight_w": w, "anchor_target_ratio_r": R_RATIO, "epochs": EPOCHS,
                        "offline_verdict": hier["verdict"], "quarantined": quarantined, "may_not_be_source": quarantined,
                        "status": "INTERMEDIATE CANDIDATE (rev3x): phase-weighted anchored bridge; no closed-loop evidence exists for it",
                        "only_actor_with_closed_loop_evidence": FIT.PIN_S0D_ACTOR}
            A.assert_no_deployable_marking(manifest, f"{cid}.manifest")
            C.write_json(sm / W.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME, manifest, clobber=False)
            receipt = {"schema": "v26b_s1c1_fit.1", "amendment_rev3x": PIN_AMENDMENT_REV3X,
                       "authorized_stage": AUTHORIZED_STAGE, **A.MANDATORY_FLAGS,
                       "quarantined": quarantined, "may_not_be_source": quarantined,
                       "candidate": {**cand, "r": R_RATIO, "epochs": EPOCHS}, "lineage": lineage,
                       "init": {"module": C.rel(SR.S0D_MODULE), "actor_digest": FIT.PIN_S0D_ACTOR,
                                "module_state_sha256": FIT.PIN_S0D_MODULE_STATE, "fresh_reload_per_candidate": True},
                       "fit": {k: v for k, v in frep.items() if k != "history"},
                       "loss_curve_essential": {"first": frep["history"][0], "last": frep["history"][-1]},
                       "loss_history_full": frep["history"],
                       "scaling": {"T1_maxabs": metrics["_T1"], "T2_maxabs": metrics["_T2"], "tol": G.PRESERVATION_TOL},
                       "metrics": {k: v for k, v in metrics.items() if not k.startswith("_")},
                       "binding_offline_hierarchy": hier,
                       "G2_G4_offline_surrogates_non_binding": {"candidate": sur, "S0D": refs["S0D"]["surrogates"], "A2": refs["A2"]["surrogates"]},
                       "preregistered_causal_diagnostic": {"candidate": per, "S0D": refs["S0D"]["persistence"],
                                                            "A2": refs["A2"]["persistence"], "AB06_IK_target": refs["AB06_IK_target"]["persistence"]},
                       "closed_loop_gates_not_evaluable_here": "rev3v G1..G5 require a rollout, which is not authorised in this stage",
                       "structure": metrics["_structure"], "save_reload_exact": exact,
                       "output_module": C.rel(out_dir / "rl_module"),
                       "output_files_sha256": {p.name: C.sha256_file(p) for p in sorted(sm.iterdir())},
                       "scope": "S1C-1 offline fit ONLY: no rollout, episode, collection, promotion, sigma or production change",
                       "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}
            A.assert_no_deployable_marking(receipt, f"{cid}.receipt")
            C.write_json(staging / RECEIPT_NAME, receipt)
            csha = C.sha256_file(staging / RECEIPT_NAME)
            RF.promote_staging(staging, out_dir); promoted = True
        except BaseException:
            if staging is not None and not promoted:
                shutil.rmtree(staging, ignore_errors=True)
            raise
        finally:
            RF.release_export_lock(lock, tok)
        results.append({"id": cid, "w": w, "verdict": hier["verdict"], "failed_level": hier["failed_level"],
                        "quarantined": quarantined, "actor_digest": metrics["_actor_digest"],
                        "path": C.rel(out_dir), "receipt_sha256": csha,
                        "metrics": {k: v for k, v in metrics.items() if not k.startswith("_")},
                        "surrogates": sur, "persistence": per})
        if progress:
            print(json.dumps({"candidate": cid, "verdict": hier["verdict"], "failed": hier["failed_level"],
                              "duty": per["duty_cycle_negative_command"], "max_run": per["runs"]["max_length"]}), flush=True)
    survivors = [x["id"] for x in results if x["verdict"] == "PASS"]
    agg = {"schema": "v26b_s1c1_fit_aggregate.1", "amendment_rev3x": PIN_AMENDMENT_REV3X,
           "authorized_stage": AUTHORIZED_STAGE, **A.MANDATORY_FLAGS,
           "candidates_executed": [x["id"] for x in results], "results": results,
           "offline_survivors": survivors,
           "survivor_meaning": "passed the rev3s offline hierarchy only; NOT walking, NOT promoted, closed-loop gates not evaluated",
           "references": refs, "lineage": lineage,
           "B3_status": "INDETERMINATE and a separate open blocker: the phase field is identically zero; no production or logging change was made",
           "next_stage_locked": FUTURE_ROLLOUT_TOKEN,
           "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}
    A.assert_no_deployable_marking(agg, "aggregate")
    path = R.unique_artifact_path(CAND_ROOT, f"{AGG_NAME}_{time.strftime('%Y%m%d_%H%M%S')}", ".json")
    VA._atomic_fill_reserved(path, json.dumps(agg, indent=2, ensure_ascii=False, default=str) + "\n")
    return {"aggregate_path": C.rel(path), "aggregate_sha256": C.sha256_file(path), "results": results, "survivors": survivors}


def run_rollout(*_, **kw):
    raise S1C1Error(f"the closed-loop evaluation requires --authorized-stage {FUTURE_ROLLOUT_TOKEN}, NOT granted; got {kw.get('authorized_stage')!r}")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="V26B rev3x S1C-1 offline fit (token-gated; offline only)")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--authorized-stage", default=None)
    args = parser.parse_args(argv)
    if not args.execute:
        verify_lineage()
        print(json.dumps({"mode": "dry", "ok": True, "candidates": [c["id"] for c in CANDIDATES]}, indent=2))
        return 0
    out = run_fit(authorized_stage=args.authorized_stage)
    print(json.dumps({"aggregate": out["aggregate_path"], "aggregate_sha256": out["aggregate_sha256"],
                      "verdicts": {x["id"]: [x["verdict"], x["failed_level"]] for x in out["results"]},
                      "survivors": out["survivors"]}, indent=2, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
