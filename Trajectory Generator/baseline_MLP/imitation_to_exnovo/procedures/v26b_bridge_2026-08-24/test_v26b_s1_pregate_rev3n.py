"""Self-test rev3n symmetrized S1 pre-gate (read-only; no fit, no receipt written)."""
from __future__ import annotations
import json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_s1_pregate_rev3n as N  # noqa: E402
import v26b_s1_prereg as S1  # noqa: E402
import f0_common as C  # noqa: E402

CHECKS = 0
def check(cond, what):
    global CHECKS; assert cond, what; CHECKS += 1
def expect(fn, exc, what):
    global CHECKS
    try: fn()
    except exc as e: CHECKS += 1; return e
    raise AssertionError(f"expected {exc.__name__}: {what}")

def main() -> int:
    lin = N.verify_lineage_rev3n()
    check(lin["amendment_rev3n"] == N.PIN_AMENDMENT_REV3N and lin["amendment_rev3l"] == S1.PIN_AMENDMENT_REV3L
          and lin["amendment_rev3m"] == S1.PIN_AMENDMENT_REV3M, "lineage: rev3n + rev3l + rev3m pinned")
    for attr in ("PIN_AMENDMENT_REV3N", "PIN_ORIGINAL_RECEIPT"):
        old = getattr(N, attr)
        try:
            setattr(N, attr, "0" * 64); expect(N.verify_lineage_rev3n, N.Rev3nError, f"tampered {attr} -> refused")
        finally: setattr(N, attr, old)
    check(lin["original_pregate_receipt"] == "a437f1ef0d7762db5914d14be19b390ddc6ada9a126d9db2ef8f810d08a43389",
          "original FAIL receipt preserved immutable (evidence)")
    proof = lin["original_tooling_byte_reproducible"]
    check(proof["v26b_s1_prereg.py"] == "858037282a7928e79b664419f7d47302dfd7703beb68db5e95f74d1f32ac25a0"
          and proof["test_v26b_s1_prereg.py"] == "1074e70f8003501ea68acebf0fed0bb23d9b237ea831d92568e815edba4fa40c",
          "rev3n is ADDITIVE: the original tooling is byte-identical to the digests recorded in the FAIL receipt")

    # instrument constants unchanged (no post-hoc relaxation possible without breaking this)
    check(N.RMSE_MAX is S1.PRE_GATE_RMSE_MAX and N.RMSE_MAX == 0.15 and N.K_NEIGHBOURS == 5 and N.EMBARGO is S1.EMBARGO and N.EMBARGO == 10,
          "threshold 0.15 / k=5 / embargo 10 reused from rev3l, never redefined")

    # fold geometry: zero free parameters, symmetric, leakage-safe
    check(N.FOLDS == ((1, 100), (101, 200), (201, 300), (301, 400), (401, 500)), "5 symmetric blocked folds as prescribed")
    holds, trains, embs = [], [], []
    for lo, hi in N.FOLDS:
        tr, ho, em = N.fold_masks(lo, hi)
        check(not np.any(tr & ho) and not np.any(tr & em) and not np.any(ho & em), f"fold [{lo},{hi}]: disjoint masks")
        holds.append(ho); trains.append(tr); embs.append(em)
    stacked = np.stack(holds)
    check(stacked.sum(0).max() == 1 and stacked.any(0).all() and [int(h.sum()) for h in holds] == [100] * 5,
          "the 5 holds are disjoint, cover all 500 rows exactly once, 100 rows each")
    check([int(t.sum()) for t in trains] == [390, 380, 380, 380, 390] and [int(e.sum()) for e in embs] == [10, 20, 20, 20, 10],
          "train/embargo counts follow from geometry (edge folds have a one-sided embargo)")
    view = S1.build_s1_view()
    check(np.array_equal(holds[2], view["hold"]) and np.array_equal(trains[2], view["train"]),
          "fold 3 IS the original rev3l hold window 201-300 (same masks)")

    gate = N.pre_gate_rev3n(view)
    f = gate["folds"]
    check(f[2]["rmse_vs_uIK"] == N.FOLD3_BINDING == [0.15220686466232133, 0.10626628039651334],
          "fold 3 reproduces the frozen FAIL pair BIT-EXACT (same code path as the pinned rev3l pre-gate)")
    per = [x["rmse_vs_uIK"] for x in f]
    b = gate["binding"]
    check(b["median_per_joint"] == np.median(np.asarray(per), axis=0).tolist() and b["threshold"] == 0.15,
          "primary metric = median of the 5 folds per joint, threshold unchanged")
    check(b["pass"] == bool(b["median_per_joint"][0] <= 0.15 and b["median_per_joint"][1] <= 0.15), "pass rule = median <= 0.15 on BOTH joints")
    check(b["max_per_joint"] == np.asarray(per).max(0).tolist() and b["n_folds_le_threshold"] == [int((np.asarray(per)[:, j] <= 0.15).sum()) for j in (0, 1)],
          "max and fold counts reported (non-binding)")
    check(all(set([0, 1]) <= set(x["features_excluded_constant_on_train"]) for x in f), "dead clock excluded in every fold (plus fold-specific train-constant features, recorded)")

    # median semantics probed with synthetic fold values (no data involved)
    syn = N.pass_rule([[0.10, 0.10], [0.12, 0.10], [0.14, 0.10], [0.30, 0.10], [0.40, 0.10]])
    check(syn["pass"] is True and syn["median_per_joint"][0] == 0.14 and syn["max_per_joint"][0] == 0.40,
          "median rule passes where max would fail, and still reports the max (declared behaviour)")
    check(N.pass_rule([[0.16, 0.10]] * 5)["pass"] is False, "median rule fails when the median exceeds 0.15")
    expect(lambda: N.pass_rule([[0.1, 0.1]] * 4), N.Rev3nError, "wrong fold count -> fail-closed")

    src = gate["source_holdout_rev3m"]
    check(src["constructible_without_leakage"] is True and src["usable_rows"] == src["source_holdout_rows_rev3k"] - src["bitwise_shared_with_task_excluded"],
          "rev3m source holdout still constructible; shared rows excluded and counted")
    pr = gate["future_postfit_rules"]
    check(pr["G_task_blocked_holdout_steps"] == [201, 300] and pr["G_task_blocked_holdout_max"] == 0.15
          and pr["G_task_blocked_holdout_must_be_out_of_sample"] is True
          and pr["G_task_excluded_from_fit_training_rows"] == [[191, 200], [201, 300], [301, 310]]
          and pr["Q1_source_holdout_rev3m_max"] == 0.10 and pr["Q1_source_holdout_rev3m_binding"] is True,
          "future post-fit: blocked holdout 201-300 stays a SEPARATE out-of-sample gate; Q1 rev3m <= 0.10 binding")

    e = expect(lambda: N.run_fit(authorized_stage=None), N.Rev3nError, "rev3n fit guard refuses")
    check("V26B-S1-FIT" in str(e) and "NOT granted" in str(e), "guard names the future token")
    expect(lambda: N.run_fit(authorized_stage="V26B-S1-REV3N-PREREG-MEDIAN"), N.Rev3nError, "the rev3n token does NOT unlock the fit")
    expect(lambda: S1.run_fit(authorized_stage="V26B-S1-REV3N-PREREG-MEDIAN"), S1.S1Error, "the rev3n token does NOT unlock the rev3l fit either")
    check(C.sha256_file(S1.AMENDMENT_REV3L) == S1.PIN_AMENDMENT_REV3L and C.sha256_file(S1.AMENDMENT_REV3M) == S1.PIN_AMENDMENT_REV3M
          and C.sha256_file(N.AMENDMENT_REV3N) == N.PIN_AMENDMENT_REV3N, "rev3l/rev3m/rev3n untouched")
    print(json.dumps({"selftest": "PASS", "checks": CHECKS,
                      "per_fold_rmse_vs_uIK": per, "median_per_joint": b["median_per_joint"], "pass": b["pass"]}, indent=1))
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
