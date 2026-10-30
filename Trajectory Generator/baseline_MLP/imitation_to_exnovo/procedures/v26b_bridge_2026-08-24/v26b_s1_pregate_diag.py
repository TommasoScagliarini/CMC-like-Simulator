"""V26B S1 pre-gate FAIL diagnostics (READ-ONLY; architect review order 2026-08-24).

Decomposes the marginal knee FAIL (kNN5 RMSE 0.15220686 > 0.15) into: hold-window
arbitrariness, kNN-metric sensitivity, local coverage, and training density.
No fit/export/rollout; rev3l/rev3m untouched; single no-clobber JSON output.
Built-in sanity: reproduces the binding receipt pair bit-exact before anything else.
"""
from __future__ import annotations
import json, time
import numpy as np
import v26b_s1_prereg as S1
import v26b_student as VS

RECEIPT_BINDING = [0.15220686466232133, 0.10626628039651334]  # a437f1ef... receipt
K_GRID = [1, 3, 5, 7, 10]
FRACTIONS = [0.75, 0.5, 0.25]
REPS = 10


def _knn(obs, train, hold, y, k):
    mu = obs[train].mean(0); sd = obs[train].std(0); keep = sd > 1e-9
    z = ((obs - mu) / np.where(keep, sd, 1.0))[:, keep]
    zt, zh, yt = z[train], z[hold], y[train]
    d2 = np.sum(zh ** 2, 1)[:, None] + np.sum(zt ** 2, 1)[None, :] - 2.0 * zh @ zt.T
    idx = np.argpartition(d2, k - 1, 1)[:, :k]
    preds = yt[idx].mean(1)
    rmse = np.sqrt(np.mean((preds - y[hold]) ** 2, axis=0))
    return rmse, preds, d2, idx, keep


def masks_for_window(step, lo, hi, embargo=10):
    hold = (step >= lo) & (step <= hi)
    emb = ((step >= lo - embargo) & (step < lo)) | ((step > hi) & (step <= hi + embargo))
    return ~(hold | emb), hold


def main() -> None:
    view = S1.build_s1_view()
    obs = view["obs"].astype(np.float64)
    u_ik, u_own = view["u_ik"], view["u_own"]
    step = np.arange(1, 501)
    names35, _sha = VS.OA.read_manifest_names(VS.C.ACTOR_MANIFEST_35, expected_sha256=VS.C.ACTOR_MANIFEST_35_SHA256, sha256_fn=VS.C.sha256_file)

    # 0) sanity: reproduce the binding receipt numbers bit-exact with the declared window
    rmse0, preds0, d2_0, idx0, keep0 = _knn(obs, view["train"], view["hold"], u_ik, 5)
    assert rmse0.tolist() == RECEIPT_BINDING, f"binding not reproduced: {rmse0.tolist()}"

    # 1) window sweep: every contiguous 100-step hold window, same embargo/instrument
    sweep = []
    for lo in range(1, 402):
        tr, ho = masks_for_window(step, lo, lo + 99)
        r_ik = _knn(obs, tr, ho, u_ik, 5)[0]
        r_own = _knn(obs, tr, ho, u_own, 5)[0]
        sweep.append((lo, float(r_ik[0]), float(r_ik[1]), float(r_own[0]), float(r_own[1]), int(tr.sum())))
    arr = np.array([(a, b, c, d, e) for a, b, c, d, e, _ in sweep])
    knee, ankle = arr[:, 1], arr[:, 2]
    at201 = arr[arr[:, 0] == 201][0]
    sweep_stats = {
        "windows": 401, "hold_len": 100, "embargo": 10,
        "knee_vs_uIK": {"median": float(np.median(knee)), "mean": float(knee.mean()), "p90": float(np.quantile(knee, .9)), "min": float(knee.min()), "max": float(knee.max()),
                        "frac_windows_gt_0p15": float((knee > 0.15).mean()), "value_at_201": float(at201[1]), "percentile_rank_of_201": float((knee <= at201[1]).mean())},
        "ankle_vs_uIK": {"median": float(np.median(ankle)), "mean": float(ankle.mean()), "max": float(ankle.max()), "frac_windows_gt_0p15": float((ankle > 0.15).mean()), "value_at_201": float(at201[2])},
        "own_label_baseline": {"knee_median": float(np.median(arr[:, 3])), "ankle_median": float(np.median(arr[:, 4])),
                               "knee_frac_gt_0p15": float((arr[:, 3] > 0.15).mean()), "ankle_frac_gt_0p15": float((arr[:, 4] > 0.15).mean())},
    }

    # 2) blocked 5-fold CV (symmetric, window-free): folds at starts 1/101/201/301/401
    folds = []
    for lo in (1, 101, 201, 301, 401):
        tr, ho = masks_for_window(step, lo, lo + 99)
        r_ik = _knn(obs, tr, ho, u_ik, 5)[0]
        folds.append({"steps": [lo, lo + 99], "knee": float(r_ik[0]), "ankle": float(r_ik[1]), "train_rows": int(tr.sum())})
    fk = np.array([f["knee"] for f in folds]); fa = np.array([f["ankle"] for f in folds])
    five_fold = {"folds": folds,
                 "knee": {"median": float(np.median(fk)), "mean": float(fk.mean()), "max": float(fk.max()), "n_folds_le_0p15": int((fk <= 0.15).sum())},
                 "ankle": {"median": float(np.median(fa)), "mean": float(fa.mean()), "max": float(fa.max()), "n_folds_le_0p15": int((fa <= 0.15).sum())}}

    # 3) k sensitivity on the DECLARED window
    ksens = {str(k): _knn(obs, view["train"], view["hold"], u_ik, k)[0].tolist() for k in K_GRID}

    # 4) local coverage / aliasing on the declared window (knee)
    ho_idx = np.where(view["hold"])[0]
    d5 = np.sort(d2_0, 1)[:, :5]
    nn_dist = np.sqrt(np.maximum(d5, 0).mean(1))
    yt = u_ik[view["train"]]
    nn_lab_std = yt[idx0][:, :, 0].std(1)
    err_knee = np.abs(preds0[:, 0] - u_ik[view["hold"], 0])
    slope = np.abs(np.gradient(u_ik[:, 0]))[view["hold"]]
    def corr(a, b): return float(np.corrcoef(a, b)[0, 1])
    phase_cols = [i for i, n in enumerate(names35) if n.startswith("phase_fsm")]
    order = np.argsort(-err_knee)[:8]
    top = [{"step": int(step[ho_idx[i]]), "abs_err_knee": float(err_knee[i]), "nn_dist": float(nn_dist[i]), "nn_label_std_knee": float(nn_lab_std[i]),
            "u_ik_knee": float(u_ik[ho_idx[i], 0]), "u_own_knee": float(u_own[ho_idx[i], 0]),
            "fsm_phase": names35[phase_cols[int(np.argmax(obs[ho_idx[i], phase_cols]))]]} for i in order]
    coverage = {"corr_err_vs_nn_dist": corr(err_knee, nn_dist), "corr_err_vs_nn_label_std": corr(err_knee, nn_lab_std),
                "corr_err_vs_uik_local_slope": corr(err_knee, slope),
                "nn_dist_median_201": float(np.median(nn_dist)), "top8_rows": top}

    # 5) training-density curve on the declared window (rng 2026, read-only)
    rng = np.random.default_rng(2026)
    tr_idx = np.where(view["train"])[0]
    dens = {}
    for f in FRACTIONS:
        vals = []
        for _ in range(REPS):
            sub = np.zeros(500, bool); sub[rng.choice(tr_idx, int(round(f * tr_idx.size)), replace=False)] = True
            vals.append(_knn(obs, sub, view["hold"], u_ik, 5)[0].tolist())
        v = np.array(vals)
        dens[str(f)] = {"knee_mean": float(v[:, 0].mean()), "knee_std": float(v[:, 0].std()), "ankle_mean": float(v[:, 1].mean())}
    dens["1.0"] = {"knee_mean": RECEIPT_BINDING[0], "knee_std": 0.0, "ankle_mean": RECEIPT_BINDING[1]}

    out = {"schema": "v26b_s1_pregate_diag.1", "read_only": True,
           "binding_receipt_sha256": "a437f1ef0d7762db5914d14be19b390ddc6ada9a126d9db2ef8f810d08a43389",
           "sanity_bitexact_reproduction": {"pass": True, "values": RECEIPT_BINDING},
           "selftest_vs_binding_note": "the 14-check SELFTEST validates tooling mechanics (pins, guards, disjointness, masks); the binding pre-gate is a DATA gate - tests PASS does not imply gate PASS",
           "window_sweep": sweep_stats, "blocked_5fold": five_fold, "k_sensitivity_window201": ksens,
           "local_coverage_window201_knee": coverage, "density_curve_window201": dens,
           "instrument_context": {"rev3b_origin": "kNN<=0.15 calibrated on the anchor dataset (train 14834 rows)", "s1_train_rows": 380,
                                  "density_ratio": 14834 / 380},
           "generated_at_utc": S1.C.utc_now(), "code_sha256_self": None}
    path = S1.R.unique_artifact_path(S1.VA.OUT_ROOT, f"v26b_s1_pregate_diag_{time.strftime('%Y%m%d_%H%M%S')}", ".json")
    out["code_sha256_self"] = S1.C.sha256_file(S1.Path(__file__).resolve())
    S1.VA._atomic_fill_reserved(path, json.dumps(out, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps({"diag_path": S1.C.rel(path), "diag_sha256": S1.C.sha256_file(path),
                      "sweep": sweep_stats, "five_fold": five_fold, "ksens": ksens,
                      "coverage": {k: v for k, v in coverage.items() if k != "top8_rows"},
                      "top3": coverage["top8_rows"][:3], "density": dens}, indent=1))


if __name__ == "__main__":
    main()
