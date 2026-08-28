"""Self-test S1 prereg (read-only; no fit)."""
from __future__ import annotations
import json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
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
    lin = S1.verify_lineage_s1()
    check(lin["amendment_rev3l"] == S1.PIN_AMENDMENT_REV3L and lin["s0d_rollout_receipt"] == S1.PIN_S0D_ROLLOUT_RECEIPT and lin["nominal_cache"] == S1.PIN_NOMINAL_CACHE, "lineage: rev3l + S0D chain + AB06 cache")
    for attr in ("PIN_AMENDMENT_REV3L", "PIN_S0D_ROLLOUT_RECEIPT"):
        old = getattr(S1, attr)
        try:
            setattr(S1, attr, "0"*64); expect(S1.verify_lineage_s1, S1.S1Error, f"tampered {attr} -> refused")
        finally: setattr(S1, attr, old)
    view = S1.build_s1_view()
    r = view["records"]
    check(r["rows"] == 500 and r["train"] + r["hold"] + r["embargo"] == 500 and r["hold"] == 100 and r["embargo"] == 20, "split blocked+embargo: 380/100/20")
    check(not np.any(view["train"] & view["hold"]) and not np.any(view["train"] & view["embargo"]), "no overlap train/hold/embargo")
    check(abs(r["gap_own_vs_ik"]["knee_mean"] - 0.3758) < 1e-3, "frozen structural gap reproduced (knee mean 0.3758)")
    g = S1.pre_gate(view)
    check(set([0, 1]) <= set(g["features_excluded_constant_on_train"]), "dead clock excluded (plus any train-constant feature, e.g. phase_fsm_wait_hs==0 on the S0D trace - fact)")
    src = g["source_holdout_rev3m"]
    check(src["constructible_without_leakage"] is True and src["usable_rows"] == src["source_holdout_rows_rev3k"] - src["bitwise_shared_with_task_excluded"], "rev3m source holdout constructible; shared rows excluded and counted")
    check(lin["amendment_rev3m"] == S1.PIN_AMENDMENT_REV3M, "rev3m pinned in lineage")
    check("knn_rmse_vs_S0D_own_holdout" in g["diagnostics"] and "JUL_H0_july_final" in g["diagnostics"]["mandatory_comparison_context"], "diagnostics: own-label kNN + mandatory JUL_H0/S0D comparison context")
    e = expect(lambda: S1.run_fit(authorized_stage=None), S1.S1Error, "fit refused")
    check("V26B-S1-FIT" in str(e) and "NOT granted" in str(e), "guard names the future token")
    expect(lambda: S1.run_fit(authorized_stage="V26B-S1-PREREG-READONLY"), S1.S1Error, "prereg token does NOT unlock the fit")
    check(C.sha256_file(S1.AMENDMENT_REV3L) == S1.PIN_AMENDMENT_REV3L, "rev3l untouched")
    print(f"SELFTEST PASS ({CHECKS} checks)")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
