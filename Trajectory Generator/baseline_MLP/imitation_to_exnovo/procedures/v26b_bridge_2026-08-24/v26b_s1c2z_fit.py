"""V26B rev3z — S1C-2Z offline fit: frozen pointwise objective + ankle sign-violation hinge.

Token V26B-S1C-2Z-FIT (offline fit only).  The frozen two-role pointwise objective is used
UNCHANGED - the task labels are never blended, weighted or replaced - and a single additive term
is appended:

    L = L_frozen_pointwise + lambda_h * mean_over_hinge_rows_in_batch( relu(0.7 * m_ankle)^2 )

with margin delta = 0.  The hinge is exactly zero where the predicted ankle command is already
non-positive, and acts only on TRAIN rows of the negative IK windows excluding the early window
(steps 6-14), whose non-observability was established by S1C-2.  Those rows stay in the fit under
the ordinary pointwise loss and under S0D preservation; only the hinge does not act on them.

The frozen T1R batching is untouched: one deterministic shuffled union of the two role blocks with
per-role means computed separately inside every batch.  No sequential minibatches.

No rollout, episode, collection, promotion, sigma choice or production change.
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

import v26b_s1c2_preflight as PF  # noqa: E402   (rev3y lineage, unmodified)
import v26b_s1c1_fit as F1  # noqa: E402         (rev3x persistence/surrogates, unmodified)
import v26b_s1c0_probe as PR  # noqa: E402       (rev3w validated production replica)
import v26b_s1c_protocol as SC  # noqa: E402     (decode)
import v26b_s1b_fit as T  # noqa: E402           (rev3t machinery: fresh_init, two_role_loss, numerics)
import v26b_s1b_protocol as B  # noqa: E402      (rev3s corpus + offline hierarchy)
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
import f2r_labeller as L  # noqa: E402
import f2r_refit as RF  # noqa: E402


class S1C2ZError(RuntimeError):
    pass


AUTHORIZED_STAGE = "V26B-S1C-2Z-FIT"
AMENDMENT_REV3Z = HERE / "v26b_amendment_rev3z_s1c2z_hinge.json"
PIN_AMENDMENT_REV3Z = "7f642c18bec1ba8bd07e15bca576e017eae2db079138e2d9b16b2310410101a9"
FUTURE_ROLLOUT_TOKEN = "V26B-S1C-2Z-NOMINAL-ROLLOUT"

R_RATIO = 5.0
EPOCHS = 300
DELTA = 0.0
ANKLE_DECODE_GAIN = 0.7
EARLY_WINDOW = (6, 14)
HOLDOUT_WINDOW = (267, 290)
CANDIDATES = ({"id": "L05", "lambda_h": 0.5}, {"id": "L10", "lambda_h": 1.0}, {"id": "L20", "lambda_h": 2.0})
CAND_ROOT = VA.OUT_ROOT / "candidates"
RECEIPT_NAME = "v26b_s1c2z_fit_receipt.json"
AGG_NAME = "v26b_s1c2z_fit_aggregate"
STATE_FAIL = "OFFLINE_FAILED_QUARANTINED"
STATE_PASS = "OFFLINE_PASSED_ROLLOUT_NOT_AUTHORISED"


def out_dir_for(cid: str) -> Path:
    return CAND_ROOT / f"S1C2Z_{cid}_35D_NONDEPLOYABLE"


def _amendment() -> dict[str, Any]:
    got = C.sha256_file(AMENDMENT_REV3Z)
    if got != PIN_AMENDMENT_REV3Z:
        raise S1C2ZError(f"rev3z sha {got} != pinned")
    return json.loads(AMENDMENT_REV3Z.read_text(encoding="utf-8"))


def verify_lineage() -> dict[str, Any]:
    lin = PF.verify_lineage()
    got = C.sha256_file(AMENDMENT_REV3Z)
    if got != PIN_AMENDMENT_REV3Z:
        raise S1C2ZError(f"rev3z sha {got} != pinned")
    lin["amendment_rev3z"] = got
    am = _amendment()
    if am["candidates_closed_grid"]["lambda_h"] != [0.5, 1.0, 2.0]:
        raise S1C2ZError("the frozen lambda grid changed")
    lin["rev3y_rejected_unexecuted"] = "no rev3y window-mean candidate exists on disk"
    for cid in ("BETA050", "BETA075", "BETA100"):
        if (CAND_ROOT / f"S1C2_{cid}_35D_NONDEPLOYABLE").exists():
            raise S1C2ZError("a rev3y candidate was materialised: it must remain unexecuted")
    return lin


# --- row selections --------------------------------------------------------------------------------

def windows(view: Mapping[str, Any]) -> list[tuple[int, int]]:
    y = SC.decode_action(view["u_ik"])[:, 1] < 0.0
    step = np.arange(1, 501)
    out, s = [], None
    for i, f in enumerate(y):
        if f and s is None:
            s = i
        if not f and s is not None:
            out.append((int(step[s]), int(step[i - 1]))); s = None
    if s is not None:
        out.append((int(step[s]), int(step[-1])))
    return out


def hinge_mask(view: Mapping[str, Any]) -> np.ndarray:
    """TRAIN rows with a negative IK ankle target, EXCLUDING the early window steps 6-14."""
    y = SC.decode_action(view["u_ik"])[:, 1] < 0.0
    step = np.arange(1, 501)
    early = (step >= EARLY_WINDOW[0]) & (step <= EARLY_WINDOW[1])
    return view["train"] & y & ~early


def hinge_term(m, is_hinge, lam: float, delta: float = DELTA):
    """lambda * mean(relu(0.7*m_ankle + delta)^2) over the hinge rows present in the batch; 0 if none."""
    import torch
    if not bool(is_hinge.any()):
        return torch.zeros((), dtype=m.dtype)
    q = ANKLE_DECODE_GAIN * m[is_hinge][:, 1] + float(delta)
    return float(lam) * torch.relu(q).square().mean()


# --- fit --------------------------------------------------------------------------------------------

def fit_candidate(init_scaled, corpus, vec, lam: float, *, epochs: int = EPOCHS, progress: bool = False):
    import torch

    f32 = np.float32
    view = corpus["view"]; tr = view["train"]
    obs_s = (view["obs"].astype(np.float64)[tr] / vec[None, :]).astype(f32)
    y_anchor = corpus["own"][tr].astype(f32)
    y_task = view["u_ik"][tr].astype(f32)                    # LABELS UNCHANGED
    hm = hinge_mask(view)[tr]
    n_role = int(obs_s.shape[0])
    obs_t = torch.as_tensor(np.concatenate([obs_s, obs_s]))
    tgt_t = torch.as_tensor(np.concatenate([y_anchor, y_task]))
    is_anchor = torch.as_tensor(np.concatenate([np.ones(n_role, bool), np.zeros(n_role, bool)]))
    is_hinge = torch.as_tensor(np.concatenate([np.zeros(n_role, bool), hm]))   # TASK block only
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
    n = 2 * n_role
    steps = 0; empty_a = empty_t = empty_h = 0; hist = []
    for epoch in range(1, int(epochs) + 1):
        perm = rng.permutation(n)
        ls, la, lt, lh, an = [], [], [], [], []
        for s0 in range(0, n, T.BATCH):
            idx = torch.as_tensor(perm[s0: s0 + T.BATCH]); opt.zero_grad(set_to_none=True)
            m = enc(obs_t[idx]) @ W3m.T + b3m
            mb = is_anchor[idx]; hb = is_hinge[idx]
            if not bool(mb.any()):
                empty_a += 1
            if not bool((~mb).any()):
                empty_t += 1
            if not bool(hb.any()):
                empty_h += 1
            base, l_a, l_t = T.two_role_loss(m, tgt_t[idx], mb, R_RATIO)   # frozen, unchanged
            l_h = hinge_term(m, hb, lam)
            anchor = I.anchor_loss_july_exact(mean_params, anchor_t)
            loss = base + l_h + T.ANCHOR_W * anchor
            loss.backward(); opt.step(); steps += 1
            with torch.no_grad():
                W1.mul_(clock_mask)
            ls.append(float(loss.item())); la.append(float(l_a.item())); lt.append(float(l_t.item()))
            lh.append(float(l_h.item())); an.append(float(anchor.item()))
        hist.append({"epoch": epoch, "loss": float(np.mean(ls)), "anchor_role": float(np.mean(la)),
                     "task_role": float(np.mean(lt)), "hinge": float(np.mean(lh)), "param_anchor": float(np.mean(an))})
        if progress and (epoch == 1 or epoch % 100 == 0 or epoch == int(epochs)):
            print(json.dumps({"lambda_h": lam, **hist[-1]}), flush=True)
    hidden = int(W2.shape[0])
    lw = np.zeros((R.ACTION_DIM, hidden), dtype=f32)
    lb = np.asarray(np.log(np.full(R.ACTION_DIM, float(R.SIGMA_CONSTANT))), dtype=f32)
    scaled = {"pi.0.0.weight": W1.detach().numpy().astype(f32).copy(), "pi.0.0.bias": b1.detach().numpy().astype(f32).copy(),
              "pi.0.2.weight": W2.detach().numpy().astype(f32).copy(), "pi.0.2.bias": b2.detach().numpy().astype(f32).copy(),
              "pi.1.weight": np.concatenate([W3m.detach().numpy().astype(f32), lw], 0).copy(),
              "pi.1.bias": np.concatenate([b3m.detach().numpy().astype(f32), lb], 0).copy()}
    rep = {"lambda_h": float(lam), "delta": DELTA, "r": R_RATIO, "epochs": int(epochs), "optimizer_steps": steps,
           "hinge_rows": int(hm.sum()), "train_rows": n_role, "union_rows": n,
           "batches_without_anchor_rows": empty_a, "batches_without_task_rows": empty_t,
           "batches_without_hinge_rows": empty_h,
           "labels_modified": False,
           "batching": "frozen T1R shuffled union with separate per-role means; no sequential minibatches",
           "history": hist}
    return scaled, rep


# --- morphology + sign diagnostics (non binding) -------------------------------------------------------

def morphology_diagnostics(state_raw, corpus, label: str) -> dict[str, Any]:
    view = corpus["view"]
    obs32 = view["obs"].astype(np.float32)
    pred = SC.decode_action(RF.numpy_mean(state_raw, obs32))
    ik = SC.decode_action(view["u_ik"])
    step = np.arange(1, 501)
    out: dict[str, Any] = {"label": label, "is_a_gate": False}
    for j, jn in ((0, "knee"), (1, "ankle")):
        d = pred[:, j] - ik[:, j]
        out[f"pointwise_{jn}"] = {"rmse": float(np.sqrt(np.mean(d ** 2))), "max_abs": float(np.abs(d).max())}
    wins = windows(view)
    per_win, frac_pos = [], {}
    sl_p = sl_t = cu_p = cu_t = 0.0
    for a, b in wins:
        m = (step >= a) & (step <= b)
        p = pred[m, 1]; t = ik[m, 1]
        entry = {"window": [a, b], "rows": int(m.sum()),
                 "range_pred": float(np.ptp(p)), "range_ik": float(np.ptp(t)),
                 "range_ratio": float(np.ptp(p) / max(np.ptp(t), 1e-12)),
                 "fraction_positive_command": float(np.mean(p > 0.0)),
                 "in_train": bool(view["train"][m].all()), "in_holdout": bool(view["hold"][m].all())}
        if m.sum() > 1:
            dp, dt = np.diff(p), np.diff(t)
            sl_p += float(np.sum(dp ** 2)); sl_t += float(np.sum(dt ** 2))
        if m.sum() > 2:
            cp, ct = np.diff(p, 2), np.diff(t, 2)
            cu_p += float(np.sum(cp ** 2)); cu_t += float(np.sum(ct ** 2))
        per_win.append(entry)
        if (a, b) == EARLY_WINDOW:
            frac_pos["early_transient_6_14"] = entry["fraction_positive_command"]
        if (a, b) == HOLDOUT_WINDOW:
            frac_pos["stable_holdout_267_290"] = entry["fraction_positive_command"]
    out["within_window"] = {"per_window": per_win,
                            "slope_energy_ratio_pred_over_ik": float(sl_p / max(sl_t, 1e-12)),
                            "curvature_energy_ratio_pred_over_ik": float(cu_p / max(cu_t, 1e-12)),
                            "note": "ratios of the FITTED output to the UNCHANGED IK target; 1.0 means the shape energy is preserved"}
    out["fraction_positive_in_negative_windows"] = frac_pos
    return out


# --- run ------------------------------------------------------------------------------------------------

def run_fit(*, authorized_stage: str | None, progress: bool = True) -> dict[str, Any]:
    if authorized_stage != AUTHORIZED_STAGE:
        raise S1C2ZError(f"the S1C-2Z fit requires --authorized-stage {AUTHORIZED_STAGE}; got {authorized_stage!r}")
    lineage = verify_lineage()
    corpus = B.build_corpus()
    corpus["healthy"] = np.asarray(L.load_cache(R.OUT_CACHE, "nominal").targets)[:, [0, 2]]
    import hashlib
    labels_before = hashlib.sha256(corpus["view"]["u_ik"].tobytes()).hexdigest()
    vec, scale_table = G.scale_vector()
    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    s0d_raw = T.fresh_init()
    a2_raw = {k: np.asarray(W._as_numpy(v)) for k, v in W.load_module_state(SB.CANDIDATE_MODULE).items()}
    refs = {
        "S0D": {"persistence": F1.persistence_diagnostic(s0d_raw, corpus, "S0D"),
                "surrogates": F1.commanded_surrogates(s0d_raw, corpus, "S0D"),
                "morphology": morphology_diagnostics(s0d_raw, corpus, "S0D")},
        "A2": {"persistence": F1.persistence_diagnostic(a2_raw, corpus, "A2"),
               "surrogates": F1.commanded_surrogates(a2_raw, corpus, "A2"),
               "morphology": morphology_diagnostics(a2_raw, corpus, "A2")},
    }
    ik_hold = SC.decode_action(corpus["view"]["u_ik"])[:, 1][corpus["view"]["hold"]]
    negs = ik_hold < 0.0
    runs = F1.run_lengths(negs)
    refs["AB06_IK_target"] = {"persistence": {"label": "AB06_IK_target", "rows": int(negs.size),
                                              "duty_cycle_negative_command": float(negs.mean()),
                                              "runs": {"count": len(runs), "mean_length": (float(np.mean(runs)) if runs else 0.0),
                                                       "max_length": (int(max(runs)) if runs else 0)},
                                              "commanded_ankle": {"min": float(ik_hold.min()), "mean": float(ik_hold.mean())},
                                              "is_a_gate": False}}
    results = []
    for cand in CANDIDATES:
        cid, lam = cand["id"], cand["lambda_h"]
        out_dir = out_dir_for(cid)
        if out_dir.exists():
            raise FileExistsError(f"no-clobber: {out_dir} exists (each candidate is fitted exactly once)")
        init_raw = T.fresh_init()
        init_scaled = G.transform_state_to_scaled(init_raw, vec)
        t1 = G.preservation_test(init_raw, init_scaled, corpus["view"]["obs"], vec)
        if t1 > G.PRESERVATION_TOL:
            raise S1C2ZError(f"{cid}: prefit T1 FAILED {t1:.3e}")
        scaled, frep = fit_candidate(init_scaled, corpus, vec, lam, progress=progress)
        if hashlib.sha256(corpus["view"]["u_ik"].tobytes()).hexdigest() != labels_before:
            raise S1C2ZError(f"{cid}: the IK task labels were modified during the fit")
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
                raise S1C2ZError(f"{cid}: save/reload mismatch")
            metrics = T.evaluate_candidate(init_raw, export_raw, scaled, corpus, vec, t1, exact)
            hier = B.offline_hierarchy(metrics)
            passed = hier["verdict"] == "PASS"
            state = STATE_PASS if passed else STATE_FAIL
            per = F1.persistence_diagnostic(export_raw, corpus, cid)
            sur = F1.commanded_surrogates(export_raw, corpus, cid)
            mor = morphology_diagnostics(export_raw, corpus, cid)
            names35, _, mshas = VS.pinned_names()
            manifest = {"schema_version": 1, "actor_feature_names": names35, "actor_feature_count": 35,
                        "actor_digest": metrics["_actor_digest"], "module_state_sha256": C.sha256_file(sm / "module_state.pkl"),
                        "manifest35_sha256": mshas["manifest35_sha256"],
                        "exploration_sigma": [VS.SIGMA_PLACEHOLDER] * R.ACTION_DIM, "sigma_note": G.SIGMA_NOTE,
                        "derived_from": C.rel(SR.S0D_MODULE), "source_actor_digest": FIT.PIN_S0D_ACTOR,
                        "contract": A.CONTRACT_STRING, "deployable": False, "sigma_unresolved": True,
                        "candidate_id": cid, "lambda_h": lam, "delta": DELTA, "anchor_target_ratio_r": R_RATIO, "epochs": EPOCHS,
                        "offline_verdict": hier["verdict"], "offline_state": state,
                        "quarantined": (not passed), "may_not_be_source": (not passed),
                        "rollout_status": ("not authorised; requires a separate architect token" if passed
                                           else "no rollout pending, planned or permitted for a candidate that failed the offline hierarchy"),
                        "status": "INTERMEDIATE CANDIDATE (rev3z): pointwise labels unchanged, ankle sign hinge added",
                        "only_actor_with_closed_loop_evidence": FIT.PIN_S0D_ACTOR}
            A.assert_no_deployable_marking(manifest, f"{cid}.manifest")
            C.write_json(sm / W.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME, manifest, clobber=False)
            receipt = {"schema": "v26b_s1c2z_fit.1", "amendment_rev3z": PIN_AMENDMENT_REV3Z,
                       "authorized_stage": AUTHORIZED_STAGE, "deployable": False, "sigma_unresolved": True,
                       "offline_state": state, "quarantined": (not passed), "may_not_be_source": (not passed),
                       "candidate": {**cand, "delta": DELTA, "r": R_RATIO, "epochs": EPOCHS}, "lineage": lineage,
                       "init": {"module": C.rel(SR.S0D_MODULE), "actor_digest": FIT.PIN_S0D_ACTOR,
                                "module_state_sha256": FIT.PIN_S0D_MODULE_STATE, "fresh_reload_per_candidate": True},
                       "objective": {"form": "L_frozen_pointwise + lambda_h * mean(relu(0.7*m_ankle)^2) over hinge rows",
                                     "labels_unchanged_sha256": labels_before,
                                     "hinge_rows": frep["hinge_rows"], "early_window_excluded_from_hinge_only": list(EARLY_WINDOW),
                                     "holdout_window_never_fit": list(HOLDOUT_WINDOW)},
                       "fit": {k: v for k, v in frep.items() if k != "history"},
                       "loss_curve_essential": {"first": frep["history"][0], "last": frep["history"][-1]},
                       "loss_history_full": frep["history"],
                       "scaling": {"T1_maxabs": metrics["_T1"], "T2_maxabs": metrics["_T2"], "tol": G.PRESERVATION_TOL},
                       "metrics": {k: v for k, v in metrics.items() if not k.startswith("_")},
                       "binding_offline_hierarchy": hier,
                       "diagnostics_non_binding": {"persistence": per, "morphology": mor, "surrogates": sur,
                                                   "references": refs},
                       "B3": "INDETERMINATE - the phase field is identically zero",
                       "structure": metrics["_structure"], "save_reload_exact": exact,
                       "output_module": C.rel(out_dir / "rl_module"),
                       "output_files_sha256": {p.name: C.sha256_file(p) for p in sorted(sm.iterdir())},
                       "scope": "S1C-2Z offline fit ONLY: no rollout, episode, collection, promotion, sigma or production change",
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
        results.append({"id": cid, "lambda_h": lam, "verdict": hier["verdict"], "failed_level": hier["failed_level"],
                        "offline_state": state, "actor_digest": metrics["_actor_digest"],
                        "path": C.rel(out_dir), "receipt_sha256": csha,
                        "metrics": {k: v for k, v in metrics.items() if not k.startswith("_")},
                        "persistence": per, "morphology": mor, "surrogates": sur})
        if progress:
            print(json.dumps({"candidate": cid, "verdict": hier["verdict"], "failed": hier["failed_level"],
                              "state": state, "duty": per["duty_cycle_negative_command"],
                              "max_run": per["runs"]["max_length"],
                              "slope_ratio": mor["within_window"]["slope_energy_ratio_pred_over_ik"]}), flush=True)
    survivors = [x["id"] for x in results if x["verdict"] == "PASS"]
    agg = {"schema": "v26b_s1c2z_fit_aggregate.1", "amendment_rev3z": PIN_AMENDMENT_REV3Z,
           "authorized_stage": AUTHORIZED_STAGE, "deployable": False, "sigma_unresolved": True,
           "candidates_executed": [x["id"] for x in results], "results": results,
           "offline_survivors": survivors,
           "state_by_candidate": {x["id"]: x["offline_state"] for x in results},
           "state_rule": "derived from the offline verdict, never inherited (rev3x status addendum)",
           "survivor_meaning": "OFFLINE_PASSED_ROLLOUT_NOT_AUTHORISED: admitted to a future closed-loop gate under its own token; NOT walking, NOT promoted",
           "references": refs, "lineage": lineage,
           "B3": "INDETERMINATE and a separate open blocker; no production or logging change was made",
           "next_stage_token_if_any_survivor": FUTURE_ROLLOUT_TOKEN,
           "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}
    A.assert_no_deployable_marking(agg, "aggregate")
    path = R.unique_artifact_path(CAND_ROOT, f"{AGG_NAME}_{time.strftime('%Y%m%d_%H%M%S')}", ".json")
    VA._atomic_fill_reserved(path, json.dumps(agg, indent=2, ensure_ascii=False, default=str) + "\n")
    return {"aggregate_path": C.rel(path), "aggregate_sha256": C.sha256_file(path),
            "results": results, "survivors": survivors}


def run_rollout(*_, **kw):
    raise S1C2ZError(f"the closed-loop evaluation requires --authorized-stage {FUTURE_ROLLOUT_TOKEN}, NOT granted; got {kw.get('authorized_stage')!r}")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="V26B rev3z S1C-2Z offline fit (token-gated; offline only)")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--authorized-stage", default=None)
    args = parser.parse_args(argv)
    if not args.execute:
        verify_lineage()
        print(json.dumps({"mode": "dry", "ok": True, "candidates": [c["id"] for c in CANDIDATES]}, indent=2))
        return 0
    out = run_fit(authorized_stage=args.authorized_stage)
    print(json.dumps({"aggregate": out["aggregate_path"], "aggregate_sha256": out["aggregate_sha256"],
                      "verdicts": {x["id"]: [x["verdict"], x["failed_level"], x["offline_state"]] for x in out["results"]},
                      "survivors": out["survivors"]}, indent=2, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
