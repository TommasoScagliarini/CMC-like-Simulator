"""V26B rev3t — S1B offline FIT of the six frozen candidates (token V26B-S1B-FIT).

Every candidate starts FRESH from the exact pinned S0D module state and trains the full 35D
mean network with the genuine two-role loss

    L = lambda_a * mean_anchor(row MSE vs u_S0D) + lambda_t * mean_task(row MSE vs u_IK)
        + 1.0 * action-clip penalty + 1e-5 * July parametric anchor towards theta_S0D

with lambda_t = 1, lambda_a = r, and per-role means computed SEPARATELY inside every mini-batch
drawn from one deterministic shuffled union of the two role blocks, so batch composition can
never alter r.  The blended-label shortcut is forbidden even though it is the analytic optimum.

Offline hierarchy I -> P -> T -> D is evaluated by the unmodified rev3s tool.  Offline PASS means
ADMITTED TO THE CLOSED-LOOP GATE, never walking.  Results are materialised NONDEPLOYABLE under
candidates/; failures are materialised quarantined and can never be a source.

This module never launches an episode and never collects data.
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

import v26b_s1b_protocol as B  # noqa: E402     (rev3s protocol/corpus/hierarchy, unmodified)
import v26b_s1a_bc as A  # noqa: E402           (MANDATORY_FLAGS, deployable-marking scanner)
import v26b_s1_fit as FIT  # noqa: E402
import v26b_s0d_rollout as SR  # noqa: E402
import v26b_r2g as G  # noqa: E402
import v26b_r2i as I  # noqa: E402
import v26b_anchors as VA  # noqa: E402
import v26b_student as VS  # noqa: E402
import f0_common as C  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_refit as RF  # noqa: E402


class S1BFitError(RuntimeError):
    pass


AUTHORIZED_STAGE = "V26B-S1B-FIT"
AMENDMENT_REV3T = HERE / "v26b_amendment_rev3t_s1b_fit_execution.json"
PIN_AMENDMENT_REV3T = "2002b68740b78435458a55a503e9161d1205c2acb0e9d6a3fd332466128bb8c2"
FUTURE_ROLLOUT_TOKEN = "V26B-S1B-NOMINAL-ROLLOUT"

BATCH = 256
LR = 1e-4
SEED = 2026
CLIP_W = 1.0
ANCHOR_W = FIT.ANCHOR_WEIGHT          # 1e-5 July parametric, by reference
CAND_ROOT = VA.OUT_ROOT / "candidates"
RECEIPT_NAME = "v26b_s1b_fit_receipt.json"
AGG_NAME = "v26b_s1b_fit_aggregate"


def candidates() -> list[dict[str, Any]]:
    return list(B._amendment()["candidate_budget_finite_frozen"]["candidates"])


def out_dir_for(cid: str) -> Path:
    return CAND_ROOT / f"S1B_{cid}_35D_NONDEPLOYABLE"


def verify_lineage_fit() -> dict[str, Any]:
    lin = B.verify_lineage_s1b()
    got = C.sha256_file(AMENDMENT_REV3T)
    if got != PIN_AMENDMENT_REV3T:
        raise S1BFitError(f"rev3t sha {got} != pinned")
    lin["amendment_rev3t"] = got
    am = json.loads(AMENDMENT_REV3T.read_text(encoding="utf-8"))["parents_immutable"]
    for name, path in (("rev3s_tool", HERE / "v26b_s1b_protocol.py"), ("rev3s_test", HERE / "test_v26b_s1b_protocol.py")):
        cur = C.sha256_file(path)
        if cur != am[name]:
            raise S1BFitError(f"{name} sha {cur} != pinned {am[name]} (rev3t must stay additive)")
        lin[name] = cur
    return lin


def fresh_init() -> dict[str, Any]:
    """Reload the exact pinned S0D state from disk and digest-verify it, per candidate."""
    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    if C.sha256_file(SR.S0D_MODULE / "module_state.pkl") != FIT.PIN_S0D_MODULE_STATE:
        raise S1BFitError("S0D module_state changed")
    st = {k: np.asarray(W._as_numpy(v)) for k, v in W.load_module_state(SR.S0D_MODULE).items()}
    d = RF.validate_init_state(st, expected_actor_digest=FIT.PIN_S0D_ACTOR)["actor_digest"]
    if d != FIT.PIN_S0D_ACTOR or d == A.P0_ACTOR_DIGEST:
        raise S1BFitError("init is not the pinned S0D actor (or is the quarantined S1A)")
    return st


def two_role_loss(m, tgt, is_anchor, r, lam_clip=CLIP_W):
    """lambda_a * mean_anchor + lambda_t * mean_task with lambda_t = 1, lambda_a = r.
    Per-role means are computed separately; an absent role contributes exactly zero."""
    import torch
    l_a = ((m[is_anchor] - tgt[is_anchor]) ** 2).mean() if bool(is_anchor.any()) else torch.zeros((), dtype=m.dtype)
    l_t = ((m[~is_anchor] - tgt[~is_anchor]) ** 2).mean() if bool((~is_anchor).any()) else torch.zeros((), dtype=m.dtype)
    clip = lam_clip * torch.relu(torch.abs(m) - 1.0).square().mean()
    return float(r) * l_a + 1.0 * l_t + clip, l_a, l_t


def fit_candidate(init_scaled: Mapping[str, Any], corpus: Mapping[str, Any], vec: np.ndarray,
                  r: float, epochs: int, *, progress: bool = False) -> tuple[dict[str, Any], dict[str, Any]]:
    import torch

    f32 = np.float32
    view = corpus["view"]; tr = view["train"]
    obs_s = (view["obs"].astype(np.float64)[tr] / vec[None, :]).astype(f32)
    y_anchor = corpus["own"][tr].astype(f32)
    y_task = view["u_ik"][tr].astype(f32)
    n_role = int(obs_s.shape[0])
    obs_t = torch.as_tensor(np.concatenate([obs_s, obs_s]))            # anchor block + task block
    tgt_t = torch.as_tensor(np.concatenate([y_anchor, y_task]))
    is_anchor = torch.as_tensor(np.concatenate([np.ones(n_role, bool), np.zeros(n_role, bool)]))
    torch.use_deterministic_algorithms(True); torch.set_num_threads(1)
    torch.manual_seed(SEED); np.random.seed(SEED)
    W1 = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.0.0.weight"], dtype=f32)))
    b1 = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.0.0.bias"], dtype=f32)))
    W2 = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.0.2.weight"], dtype=f32)))
    b2 = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.0.2.bias"], dtype=f32)))
    W3m = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.1.weight"][: R.ACTION_DIM], dtype=f32)))
    b3m = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.1.bias"][: R.ACTION_DIM], dtype=f32)))
    mean_params = [W1, b1, W2, b2, W3m, b3m]
    anchor_t = [p.detach().clone() for p in mean_params]
    clock_mask = torch.ones_like(W1); clock_mask[:, list(R.CLOCK_COLUMNS)] = 0.0
    with torch.no_grad():
        W1.mul_(clock_mask)
    encoder = lambda x: torch.tanh(torch.tanh(x @ W1.T + b1) @ W2.T + b2)  # noqa: E731
    opt = torch.optim.Adam(mean_params, lr=LR)
    rng = np.random.default_rng(SEED)
    n = 2 * n_role; steps = 0; empty_a = 0; empty_t = 0; history = []
    for epoch in range(1, int(epochs) + 1):
        perm = rng.permutation(n); ls, la, lt, an = [], [], [], []
        for s0 in range(0, n, BATCH):
            idx = torch.as_tensor(perm[s0: s0 + BATCH]); opt.zero_grad(set_to_none=True)
            m = encoder(obs_t[idx]) @ W3m.T + b3m
            mb = is_anchor[idx]
            if not bool(mb.any()):
                empty_a += 1
            if not bool((~mb).any()):
                empty_t += 1
            loss, l_a, l_t = two_role_loss(m, tgt_t[idx], mb, r)
            anchor = I.anchor_loss_july_exact(mean_params, anchor_t)
            loss = loss + ANCHOR_W * anchor
            loss.backward(); opt.step(); steps += 1
            with torch.no_grad():
                W1.mul_(clock_mask)
            ls.append(float(loss.item())); la.append(float(l_a.item())); lt.append(float(l_t.item())); an.append(float(anchor.item()))
        history.append({"epoch": epoch, "loss": float(np.mean(ls)), "anchor_role_mse": float(np.mean(la)),
                        "task_role_mse": float(np.mean(lt)), "param_anchor_term": float(np.mean(an))})
        if progress and (epoch == 1 or epoch % 100 == 0 or epoch == int(epochs)):
            print(json.dumps({"r": r, **history[-1]}), flush=True)
    hidden = int(W2.shape[0])
    lw = np.zeros((R.ACTION_DIM, hidden), dtype=f32)
    lb = np.asarray(np.log(np.full(R.ACTION_DIM, float(R.SIGMA_CONSTANT))), dtype=f32)
    w3 = np.concatenate([W3m.detach().numpy().astype(f32), lw], axis=0)
    b3 = np.concatenate([b3m.detach().numpy().astype(f32), lb], axis=0)
    scaled = {"pi.0.0.weight": W1.detach().numpy().astype(f32).copy(), "pi.0.0.bias": b1.detach().numpy().astype(f32).copy(),
              "pi.0.2.weight": W2.detach().numpy().astype(f32).copy(), "pi.0.2.bias": b2.detach().numpy().astype(f32).copy(),
              "pi.1.weight": w3.astype(f32).copy(), "pi.1.bias": b3.astype(f32).copy()}
    report = {"lambda_anchor": float(r), "lambda_task": 1.0, "epochs": int(epochs), "optimizer_steps": steps,
              "rows_per_role": n_role, "union_rows": n, "batch_size": BATCH, "lr": LR, "seed": SEED,
              "batches_without_anchor_rows": empty_a, "batches_without_task_rows": empty_t,
              "role_mechanism": "T1R group-balanced: separate per-role means inside every batch of one deterministic shuffled union",
              "no_hard_blend": True, "history": history}
    return scaled, report


def evaluate_candidate(init_raw, export_raw, scaled, corpus, vec, t1, save_reload_exact: bool) -> dict[str, Any]:
    f32 = np.float32
    view = corpus["view"]; tr, ho = view["train"], view["hold"]
    obs = view["obs"].astype(np.float64)
    m_exp = RF.numpy_mean(export_raw, obs.astype(f32))
    m_scaled = RF.numpy_mean(scaled, (obs / vec[None, :]).astype(f32))
    t2 = float(np.max(np.abs(m_exp - m_scaled)))
    d_anchor = m_exp[tr] - corpus["own"][tr]
    strata = corpus["strata"]["features"]
    per_stratum = {}
    for name, meta in strata.items():
        if not meta["present"]:
            continue
        mask = view["obs"][tr][:, meta["index"]] > 0.5
        per_stratum[name] = [float(np.abs(d_anchor[mask, j]).mean()) for j in (0, 1)]
    struct = RF.validate_init_state(export_raw, expected_actor_digest=None)
    inv = RF.invariance_test(export_raw, obs.astype(f32)[:64])
    m_init = RF.numpy_mean(init_raw, obs.astype(f32))
    d_all = m_exp - m_init
    metrics = {
        "integrity": {"source_equals_init_proof": True, "ten_keys": tuple(export_raw.keys()) == RF.EXPECTED_KEY_ORDER,
                      "clock_columns_zero": bool(struct["clock_columns_zero"]),
                      "clock_invariance_bit_identical": bool(inv["bit_identical"]),
                      "logstd_bit_identical_to_init": bool(np.array_equal(np.asarray(export_raw["pi.1.weight"])[R.ACTION_DIM:], np.asarray(init_raw["pi.1.weight"])[R.ACTION_DIM:])
                                                           and np.array_equal(np.asarray(export_raw["pi.1.bias"])[R.ACTION_DIM:], np.asarray(init_raw["pi.1.bias"])[R.ACTION_DIM:])),
                      "save_reload_exact": bool(save_reload_exact), "no_critic": True, "T1_T2_max": float(max(t1, t2))},
        "preservation": {"mean_abs": [float(np.abs(d_anchor[:, j]).mean()) for j in (0, 1)],
                         "rms": [float(np.sqrt(np.mean(d_anchor[:, j] ** 2))) for j in (0, 1)],
                         "max_abs": [float(np.abs(d_anchor[:, j]).max()) for j in (0, 1)],
                         "per_stratum_mean_abs": per_stratum},
        "target": {"holdout_rmse": [float(np.sqrt(np.mean((m_exp[ho, j] - view["u_ik"][ho, j]) ** 2))) for j in (0, 1)],
                   "train_in_sample_rmse": [float(np.sqrt(np.mean((m_exp[tr, j] - view["u_ik"][tr, j]) ** 2))) for j in (0, 1)]},
        "drift": {"parameter_shift_sq": float(sum(np.sum((np.asarray(export_raw[k], np.float64) - np.asarray(init_raw[k], np.float64)) ** 2) for k in RF.CANONICAL_KEYS)),
                  "action_mean_abs": [float(np.abs(d_all[:, j]).mean()) for j in (0, 1)],
                  "action_max_abs": [float(np.abs(d_all[:, j]).max()) for j in (0, 1)]},
    }
    metrics["_actor_digest"] = struct["actor_digest"]
    metrics["_structure"] = struct
    metrics["_T1"] = t1; metrics["_T2"] = t2
    return metrics


def prediction_vs_observed(cand: Mapping[str, Any], metrics: Mapping[str, Any]) -> dict[str, Any]:
    p_mean = cand["predicted_train_drift_mean_abs"]; p_rms = cand["predicted_train_drift_rms"]
    p_max = cand["predicted_train_drift_max_abs"]; p_impr = cand["predicted_target_improvement_fraction_train"]
    base = B._amendment()["analytic_predictions_from_measured_gap"]["s0d_baseline_vs_uIK_on_frozen_holdout"]
    obs_impr = [(base[j] - metrics["target"]["holdout_rmse"][j]) / base[j] for j in (0, 1)]
    o_mean = metrics["preservation"]["mean_abs"]; o_rms = metrics["preservation"]["rms"]; o_max = metrics["preservation"]["max_abs"]
    dev = lambda p, o: [float(o[j] - p[j]) for j in (0, 1)]  # noqa: E731
    return {"predicted": {"drift_mean_abs": p_mean, "drift_rms": p_rms, "drift_max_abs": p_max,
                          "target_improvement_fraction_train": p_impr},
            "observed": {"drift_mean_abs": o_mean, "drift_rms": o_rms, "drift_max_abs": o_max,
                         "target_improvement_fraction_holdout": obs_impr},
            "deviation_observed_minus_predicted": {"drift_mean_abs": dev(p_mean, o_mean), "drift_rms": dev(p_rms, o_rms),
                                                   "drift_max_abs": dev(p_max, o_max)},
            "note": "the analytic model predicts the per-row optimum on the TRAINING rows; the observed improvement is measured "
                    "on the unseen holdout, so a gap between the two is expected and is reported, never corrected for"}


def run_fit(*, authorized_stage: str | None, progress: bool = True) -> dict[str, Any]:
    if authorized_stage != AUTHORIZED_STAGE:
        raise S1BFitError(f"the S1B fit requires --authorized-stage {AUTHORIZED_STAGE}; got {authorized_stage!r}")
    lineage = verify_lineage_fit()
    prov = B.verify_source_equals_init()
    corpus = B.build_corpus()
    gap = B.gap_stats(corpus)
    B.verify_predictions(gap)
    vec, scale_table = G.scale_vector()
    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    results = []
    for cand in candidates():
        cid = cand["id"]; out_dir = out_dir_for(cid)
        if out_dir.exists():
            raise FileExistsError(f"no-clobber: {out_dir} exists (each candidate is fitted exactly once)")
        init_raw = fresh_init()                                  # FRESH exact S0D per candidate
        init_scaled = G.transform_state_to_scaled(init_raw, vec)
        t1 = G.preservation_test(init_raw, init_scaled, corpus["view"]["obs"], vec)
        if t1 > G.PRESERVATION_TOL:
            raise S1BFitError(f"{cid}: prefit T1 FAILED {t1:.3e}")
        scaled, freport = fit_candidate(init_scaled, corpus, vec, cand["anchor_target_ratio_r"], cand["epochs"], progress=progress)
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
                raise S1BFitError(f"{cid}: save/reload mismatch")
            metrics = evaluate_candidate(init_raw, export_raw, scaled, corpus, vec, t1, exact)
            hier = B.offline_hierarchy(metrics)
            pvo = prediction_vs_observed(cand, metrics)
            quarantined = hier["verdict"] != "PASS"
            names35, _, mshas = VS.pinned_names()
            manifest = {"schema_version": 1, "actor_feature_names": names35, "actor_feature_count": 35,
                        "actor_digest": metrics["_actor_digest"], "module_state_sha256": C.sha256_file(sm / "module_state.pkl"),
                        "manifest35_sha256": mshas["manifest35_sha256"],
                        "exploration_sigma": [VS.SIGMA_PLACEHOLDER] * R.ACTION_DIM, "sigma_note": G.SIGMA_NOTE,
                        "derived_from": C.rel(SR.S0D_MODULE), "source_actor_digest": FIT.PIN_S0D_ACTOR,
                        "contract": A.CONTRACT_STRING, **A.MANDATORY_FLAGS,
                        "candidate_id": cid, "anchor_target_ratio_r": cand["anchor_target_ratio_r"], "epochs": cand["epochs"],
                        "offline_verdict": hier["verdict"], "quarantined": quarantined,
                        "may_not_be_source": quarantined,
                        "status": "INTERMEDIATE CANDIDATE (rev3t): trust-region anchored bridge; no closed-loop evidence exists for it",
                        "only_actor_with_closed_loop_evidence": FIT.PIN_S0D_ACTOR}
            A.assert_no_deployable_marking(manifest, f"{cid}.manifest")
            C.write_json(sm / W.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME, manifest, clobber=False)
            receipt = {"schema": "v26b_s1b_fit.1", "amendment_rev3t": PIN_AMENDMENT_REV3T, "authorized_stage": AUTHORIZED_STAGE,
                       **A.MANDATORY_FLAGS, "quarantined": quarantined, "may_not_be_source": quarantined,
                       "candidate": cand, "lineage": lineage, "source_equals_init": prov,
                       "init": {"module": C.rel(SR.S0D_MODULE), "actor_digest": FIT.PIN_S0D_ACTOR,
                                "module_state_sha256": FIT.PIN_S0D_MODULE_STATE, "fresh_reload_per_candidate": True},
                       "corpus": {"anchor": corpus["anchor"], "task": corpus["task"], "holdout": corpus["holdout"]},
                       "fit": {k: freport[k] for k in freport if k != "history"},
                       "loss_curve_essential": {"first": freport["history"][0], "last": freport["history"][-1]},
                       "loss_history_full": freport["history"],
                       "scaling": {"table": scale_table, "T1_maxabs": metrics["_T1"], "T2_maxabs": metrics["_T2"], "tol": G.PRESERVATION_TOL},
                       "metrics": {k: v for k, v in metrics.items() if not k.startswith("_")},
                       "offline_hierarchy": hier, "offline_pass_meaning": json.loads(AMENDMENT_REV3T.read_text(encoding="utf-8"))["offline_pass_meaning"],
                       "prediction_vs_observed": pvo,
                       "structure": metrics["_structure"], "save_reload_exact": exact,
                       "output_module": C.rel(out_dir / "rl_module"),
                       "output_files_sha256": {p.name: C.sha256_file(p) for p in sorted(sm.iterdir())},
                       "scope": "S1B offline fit + offline hierarchy ONLY: no rollout, no collection, no DAgger, no PPO/critic, no sigma",
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
        results.append({"id": cid, "r": cand["anchor_target_ratio_r"], "epochs": cand["epochs"],
                        "verdict": hier["verdict"], "failed_level": hier["failed_level"], "quarantined": quarantined,
                        "actor_digest": metrics["_actor_digest"], "path": C.rel(out_dir), "receipt_sha256": csha,
                        "metrics": {k: v for k, v in metrics.items() if not k.startswith("_")},
                        "prediction_vs_observed": pvo})
        if progress:
            print(json.dumps({"candidate": cid, "verdict": hier["verdict"], "failed_level": hier["failed_level"]}), flush=True)
    survivors = B.closed_loop_order({x["id"]: {"verdict": x["verdict"]} for x in results})
    agg = {"schema": "v26b_s1b_fit_aggregate.1", "amendment_rev3t": PIN_AMENDMENT_REV3T,
           "authorized_stage": AUTHORIZED_STAGE, **A.MANDATORY_FLAGS,
           "candidates_executed": [x["id"] for x in results], "results": results,
           "offline_survivors_in_frozen_closed_loop_order": survivors,
           "survivor_meaning": "ADMITTED to the future closed-loop gate under a separate token; NOT walking, NOT promoted",
           "next_stage_locked": FUTURE_ROLLOUT_TOKEN, "lineage": lineage, "gap_stats": gap,
           "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}
    A.assert_no_deployable_marking(agg, "aggregate")
    path = R.unique_artifact_path(CAND_ROOT, f"{AGG_NAME}_{time.strftime('%Y%m%d_%H%M%S')}", ".json")
    VA._atomic_fill_reserved(path, json.dumps(agg, indent=2, ensure_ascii=False, default=str) + "\n")
    return {"aggregate_path": C.rel(path), "aggregate_sha256": C.sha256_file(path), "results": results, "survivors": survivors}


def run_rollout(*_, **kw):
    raise S1BFitError(f"the closed-loop evaluation requires --authorized-stage {FUTURE_ROLLOUT_TOKEN}, NOT granted; got {kw.get('authorized_stage')!r}")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="V26B rev3t S1B offline fit (token-gated; offline only)")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--authorized-stage", default=None)
    args = parser.parse_args(argv)
    if not args.execute:
        verify_lineage_fit()
        print(json.dumps({"mode": "dry", "lineage_ok": True, "candidates": [c["id"] for c in candidates()]}, indent=2))
        return 0
    out = run_fit(authorized_stage=args.authorized_stage)
    print(json.dumps({"aggregate": out["aggregate_path"], "aggregate_sha256": out["aggregate_sha256"],
                      "verdicts": {x["id"]: [x["verdict"], x["failed_level"]] for x in out["results"]},
                      "survivors": out["survivors"]}, indent=2, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
