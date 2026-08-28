"""Static + numerical self-test of the rev3y S1C-2 preflight. Read-only; no fit, no rollout."""
from __future__ import annotations
import json, sys, tempfile
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_s1c2_preflight as P  # noqa: E402
import v26b_s1c1_fit as F1  # noqa: E402
import v26b_s1c_protocol as SC  # noqa: E402
import v26b_s1_pregate_rev3n as N  # noqa: E402
import f0_common as C  # noqa: E402
import f2r_common as R  # noqa: E402

CHECKS = 0
def check(cond, what):
    global CHECKS; assert cond, what; CHECKS += 1
def expect(fn, exc, what):
    global CHECKS
    try: fn()
    except exc as e: CHECKS += 1; return e
    raise AssertionError(f"expected {exc.__name__}: {what}")

def main() -> int:
    # --- lineage, pins, immutability of the parents ------------------------------------------------
    lin = P.verify_lineage()
    check(lin["amendment_rev3y"] == P.PIN_AMENDMENT_REV3Y and lin["addendum_rev3x_a_status"] == P.PIN_ADDENDUM_STATUS
          and lin["amendment_rev3x"] == F1.PIN_AMENDMENT_REV3X, "lineage pins rev3x, the status addendum and rev3y")
    check(lin["s1c1_aggregate_unmutated"] == P.PIN_S1C1_AGGREGATE == "58874483683ff3f9eb6741f7f752fbb5b9c8fec96b3277c3543f0eae58b7d3e4"
          and lin["s1c1_survivors"] == [], "the S1C-1 aggregate is byte-identical and still records zero survivors")
    for attr in ("PIN_AMENDMENT_REV3Y", "PIN_ADDENDUM_STATUS", "PIN_S1C1_AGGREGATE"):
        old = getattr(P, attr)
        try:
            setattr(P, attr, "0" * 64); expect(P.verify_lineage, P.PreflightError, f"tampered {attr} -> refused")
        finally: setattr(P, attr, old)

    # --- the status addendum is authoritative and does not mutate anything ----------------------------
    ad = json.loads(P.ADDENDUM_STATUS.read_text())
    check(all(ad["authoritative_status"][k].startswith("OFFLINE_FAILED_QUARANTINED") for k in ("W2", "W4", "W8")),
          "the addendum gives W2/W4/W8 the authoritative status OFFLINE_FAILED_QUARANTINED")
    check("no rollout pending" in ad["authoritative_status"]["W2"] and ad["parent"]["aggregate"]["status"].startswith("PRESERVED"),
          "it states that no rollout is pending for a failed candidate and preserves the aggregate")
    check("not_a_data_error" in ad["defect_corrected"], "the addendum records that only the status wording is corrected")

    # --- tokens and guards ------------------------------------------------------------------------------
    for bad in (None, "V26B-S1C-1", "V26B-S1C-2-FIT", "v26b-s1c-2-preflight"):
        e = expect(lambda b=bad: P.run_audit(authorized_stage=b), P.PreflightError, f"token {bad!r} refused")
        check("V26B-S1C-2-PREFLIGHT" in str(e), "guard names this stage's token")
    e = expect(lambda: P.run_fit(authorized_stage=P.AUTHORIZED_STAGE), P.PreflightError, "the S1C-2 fit is refused here")
    check("V26B-S1C-2-FIT" in str(e) and "NOT granted" in str(e), "the fit guard names its own future token")
    src = (HERE / "v26b_s1c2_preflight.py").read_text()
    check(not any(t in src for t in ("subprocess.run(", "Popen", "import torch", "backward()", "opt.step", "rollout_eval")),
          "no fit or closed-loop primitive in the preflight module")

    # --- label rule and structure -------------------------------------------------------------------------
    data = P.build()
    y2 = (SC.decode_action(data["view"]["u_ik"])[:, 1] < 0.0).astype(int)
    check(np.array_equal(data["y"], y2) and data["positives"] == 97 and data["rows"] == 500,
          "y = 1 iff the DECODED IK ankle target is negative; 97 positives on 500 rows")
    check(data["runs"] == [9, 21, 6, 24, 5, 23, 9] and sum(data["runs"]) == 97,
          "seven contiguous runs, matching the structure frozen in rev3y")

    # --- feature sets ---------------------------------------------------------------------------------------
    fs = P.feature_sets()
    check(len(fs["runtime_available"]) == 33 and 0 not in fs["runtime_available"] and 1 not in fs["runtime_available"],
          "the runtime set is the 35 minus the two dead clock columns")
    check(len(fs["physical_only_diagnostic"]) == 25
          and all(i not in fs["physical_only_diagnostic"] for i in P.PHASE_FAMILY),
          "the physical-only diagnostic set additionally drops the whole online phase family")
    check([R.FEATURE_NAMES_35[i] for i in P.PHASE_FAMILY][:2] == ["online_left_gait_phase_sin", "online_left_gait_phase_cos"],
          "the phase family is identified by name, not by position alone")

    # --- folds partition the rows exactly once ------------------------------------------------------------------
    cov = np.zeros(500, int)
    for lo, hi in N.FOLDS:
        _, ho, _ = N.fold_masks(lo, hi)
        cov[ho] += 1
    check(np.all(cov == 1), "the five frozen folds partition the 500 rows exactly once (each row held out once)")

    # --- metric correctness on synthetic confusion matrices -------------------------------------------------------
    yt = np.array([1, 1, 1, 1, 0, 0, 0, 0]); yp = np.array([1, 1, 0, 0, 0, 0, 0, 1])
    m = P.pooled_metrics(yt, yp)
    check(m["tp"] == 2 and m["fn"] == 2 and m["tn"] == 3 and m["fp"] == 1
          and abs(m["recall_positive"] - 0.5) < 1e-12 and abs(m["specificity"] - 0.75) < 1e-12
          and abs(m["balanced_accuracy"] - 0.625) < 1e-12, "pooled metrics computed correctly")
    perf = P.pooled_metrics(yt, yt)
    check(perf["balanced_accuracy"] == 1.0 and perf["recall_positive"] == 1.0, "a perfect prediction gives 1.0")

    # --- GO/NO-GO logic on fixtures (each condition failing in isolation) -------------------------------------------
    null = {"mean_balanced_accuracy": 0.48}
    ok = P.go_no_go({"pooled": {"balanced_accuracy": 0.80, "recall_positive": 0.70}}, null)
    check(ok["verdict"] == "GO" and all(ok["conditions"].values()), "all three conditions met -> GO")
    for ba, rec, nl, failing in ((0.74, 0.70, 0.48, "balanced_accuracy>=0.75"),
                                 (0.80, 0.59, 0.48, "recall_positive>=0.60"),
                                 (0.80, 0.70, 0.75, "margin_over_block_null>=0.10")):
        v = P.go_no_go({"pooled": {"balanced_accuracy": ba, "recall_positive": rec}}, {"mean_balanced_accuracy": nl})
        check(v["verdict"] == "NO_GO" and v["conditions"][failing] is False
              and sum(1 for c in v["conditions"].values() if not c) == 1, f"{failing} alone forces NO_GO")
    check(P.GO_BALANCED_ACC == 0.75 and P.GO_RECALL == 0.60 and P.GO_NULL_MARGIN == 0.10
          and ok["criterion"]["frozen_in"].startswith("rev3y"), "the thresholds are the ones frozen in rev3y")

    # --- aliasing contrast behaves as expected on synthetic data ------------------------------------------------------
    rng = np.random.default_rng(0)
    n = len(data["view"]["obs"])
    sep = data.copy(); sep["obs"] = rng.normal(size=(n, 35)); sep["obs"][data["y"] == 1] += 8.0
    c_sep = P.nn_distance_contrast(sep, list(range(2, 35)))
    ali = data.copy(); ali["obs"] = rng.normal(size=(n, 35))
    c_ali = P.nn_distance_contrast(ali, list(range(2, 35)))
    check(c_sep["positive"]["ratio_cross_over_intra_median"] > c_ali["positive"]["ratio_cross_over_intra_median"]
          and c_ali["positive"]["fraction_with_cross_nn_closer_than_intra"] > 0.5,
          "on separable synthetic data the cross/intra ratio is large; on pure noise the classes alias")

    # --- null control ---------------------------------------------------------------------------------------------------
    check(P.NULL_BLOCK == 25 and P.NULL_REPS == 20 and P.NULL_SEED == 2026,
          "the block-permutation null uses the frozen block size, repetitions and seed")

    # --- no-clobber and the executed-nothing record ------------------------------------------------------------------------
    tmp = Path(tempfile.mkdtemp(prefix="s1c2_"))
    try:
        o1 = P.run_audit(authorized_stage=P.AUTHORIZED_STAGE, out_dir=tmp)
        o2 = P.run_audit(authorized_stage=P.AUTHORIZED_STAGE, out_dir=tmp)
        check(o1["receipt_path"] != o2["receipt_path"]
              and C.sha256_file(C.REPO / Path(o1["receipt_path"])) == o1["receipt_sha256"],
              "no-clobber: a second audit reserves a new path and leaves the first byte-identical")
        ex = o1["receipt"]["executed_in_this_stage"]
        check(all(v is False for v in ex.values()) and set(ex) >= {"fit", "rollout", "collection", "promotion", "sigma_choice"},
              "the receipt records that nothing was executed, sigma included")
        check(len(o1["receipt"]["dataset_limits_declared"]) >= 4, "the dataset limits are declared in the receipt")
    finally:
        import shutil; shutil.rmtree(tmp, ignore_errors=True)
    print(json.dumps({"selftest": "PASS", "checks": CHECKS}, indent=1))
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
