"""Self-test rev3s S1B protocol (validation only; no fit, no rollout, no collection)."""
from __future__ import annotations
import json, sys, tempfile
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_s1b_protocol as B  # noqa: E402
import v26b_s1a_bc as A  # noqa: E402
import v26b_s1_fit as FIT  # noqa: E402
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

def metrics(ok=True, **over):
    m = {"integrity": {"source_equals_init_proof": True, "ten_keys": True, "clock_columns_zero": True,
                       "clock_invariance_bit_identical": True, "logstd_bit_identical_to_init": True,
                       "save_reload_exact": True, "no_critic": True, "T1_T2_max": 1e-7},
         "preservation": {"mean_abs": [0.042, 0.033], "rms": [0.056, 0.045], "max_abs": [0.159, 0.133],
                          "per_stratum_mean_abs": {"phase_fsm_stance_after_hs": [0.05, 0.04]}},
         "target": {"holdout_rmse": [0.52, 0.38]},
         "drift": {"parameter_shift_sq": 0.12, "action_mean_abs": [0.041, 0.033]}}
    for k, v in over.items():
        m[k] = {**m[k], **v}
    return m

def main() -> int:
    # --- lineage + tamper ------------------------------------------------------------------------
    lin = B.verify_lineage_s1b()
    check(lin["amendment_rev3s"] == B.PIN_AMENDMENT_REV3S and lin["amendment_rev3r"] == B.SA.PIN_AMENDMENT_REV3R
          and lin["amendment_rev3q"] == A.PIN_AMENDMENT_REV3Q, "lineage pins rev3l -> rev3s")
    check(lin["s1a_status"] == "NOT_ELIGIBLE_FAIL", "S1A is recorded NOT_ELIGIBLE and quarantined")
    old = B.PIN_AMENDMENT_REV3S
    try:
        B.PIN_AMENDMENT_REV3S = "0" * 64
        expect(B.verify_lineage_s1b, B.S1BError, "tampered rev3s pin -> refused")
        expect(B._amendment, B.S1BError, "the amendment reader is pin-guarded")
    finally:
        B.PIN_AMENDMENT_REV3S = old

    # --- token / stage negatives ------------------------------------------------------------------
    for bad in (None, "V26B-S1A-BC-FIT", "V26B-S1A-NOMINAL-ROLLOUT", "V26B-S1B-FIT", "V26B-S1B-NOMINAL-ROLLOUT",
                "v26b-s1b-anchored-protocol"):
        e = expect(lambda b=bad: B.dry_run(authorized_stage=b), B.S1BError, f"token {bad!r} refused")
        check("V26B-S1B-ANCHORED-PROTOCOL" in str(e), "guard names this stage's token")
    for fn, tok in ((B.run_fit, "V26B-S1B-FIT"), (B.run_rollout, "V26B-S1B-NOMINAL-ROLLOUT"),
                    (B.run_collection, "V26B-S0D-ALTSTART-COLLECTION")):
        e = expect(lambda f=fn: f(authorized_stage=B.AUTHORIZED_STAGE), B.S1BError, "future stage refused")
        check(tok in str(e) and "NOT granted" in str(e), f"the {tok} guard names its own future token and is never unlocked here")

    # --- absence of fit / rollout / collection primitives --------------------------------------------
    src = (HERE / "v26b_s1b_protocol.py").read_text()
    check(not any(t in src for t in ("subprocess", "rollout_eval", "os.system", "Popen", "import torch",
                                     "backward()", "opt.step", "Adam(")),
          "the module contains no fit and no closed-loop primitive")
    check("np.tile" not in src and "--nominal-repeat" not in src, "no tiling of the anchor corpus (forbidden by rev3)")

    # --- source == init ------------------------------------------------------------------------------
    prov = B.verify_source_equals_init()
    check(prov["rows"] == 500 and prov["completion"]["complete_500_time_limit"] is True
          and prov["rollout_receipt_sha256"] == B.PIN_S0D_ROLLOUT_RECEIPT, "the S0D trace is its complete 500/500 nominal run")
    check(prov["numpy_forward_vs_recorded"]["max_abs"] <= B.SOURCE_EQ_INIT_TOL
          and prov["init_actor_digest"] == B.PIN_S0D_ACTOR, "source==init proven by numpy forward vs recorded actions")
    old_tol = B.SOURCE_EQ_INIT_TOL
    try:
        B.SOURCE_EQ_INIT_TOL = 1e-12
        expect(B.verify_source_equals_init, B.S1BError, "an impossible tolerance makes the proof fail closed (the check is live)")
    finally:
        B.SOURCE_EQ_INIT_TOL = old_tol
    check(A.P0_ACTOR_DIGEST in B.FORBIDDEN_SOURCES.values() and B.PIN_S0D_ACTOR not in B.FORBIDDEN_SOURCES.values(),
          "S1A is listed as a forbidden source; S0D is not")

    # --- corpus, split, leakage ------------------------------------------------------------------------
    corpus = B.build_corpus()
    check(corpus["anchor"]["rows"] == 380 and corpus["task"]["rows"] == 380
          and corpus["holdout"]["rows"] == 100 and corpus["embargo_rows"] == 20, "frozen split 380/100/20 reused")
    check(corpus["anchor"]["obs_sha256"] == corpus["task"]["obs_sha256"] and corpus["identical_inputs"] is True,
          "both roles share EXACTLY the same input rows (the structural finding, recorded not hidden)")
    check(corpus["anchor"]["labels_sha256"] != corpus["task"]["labels_sha256"], "the two roles carry different labels")
    step = np.arange(1, 501); tr = corpus["view"]["train"]
    check(not np.any((step[tr] >= 191) & (step[tr] <= 310)), "no training row inside the holdout+embargo band")
    check(corpus["holdout"]["unseen_by_both_roles"] is True, "the holdout is unseen by BOTH roles")
    strat = corpus["strata"]["features"]
    check(len(strat) >= 6 and all(("index" in v and "rows" in v) for v in strat.values()),
          "discrete preservation strata built from the July discrete components")

    # --- gap + analytic predictions ---------------------------------------------------------------------
    gap = B.gap_stats(corpus)
    check(abs(gap["on_500_rows"]["mean_abs"][0] - 0.3758064782444853) == 0.0, "measured gap reproduces the frozen rev3l fact EXACTLY")
    bd = gap["baseline_definition"]
    check(bd["binding_for_the_T_gate"] == [0.6214394597731364, 0.4569210122991978]
          and abs(gap["s0d_baseline_vs_uIK_on_holdout_from_recorded_actions"][0] - bd["binding_for_the_T_gate"][0]) < 1e-6,
          "the T-gate baseline is the frozen FORWARD-based one; the recorded-actions variant agrees within the source==init tolerance")
    preds = B.verify_predictions(gap)
    check(len(preds["candidates"]) == 6, "the budget is finite: exactly 6 candidates")
    for c in preds["candidates"]:
        r = c["anchor_target_ratio_r"]; f = 1.0 / (1.0 + r)
        check(c["predicted_target_improvement_fraction_train"] == round(f, 6)
              and abs(c["predicted_train_drift_mean_abs"][0] - gap["on_500_rows"]["mean_abs"][0] * f) <= 5e-7,
              f"candidate r={r}: closed-form prediction reproduced (values are contractually rounded to 6 decimals)")
    am = json.loads(B.AMENDMENT_REV3S.read_text())
    ratios = [c["anchor_target_ratio_r"] for c in am["candidate_budget_finite_frozen"]["candidates"]]
    check(ratios == [3, 5, 8, 12, 20, 8] and am["candidate_budget_finite_frozen"]["open_search"] is False,
          "the grid is exactly the frozen finite one and declares no open search")

    # --- immutable thresholds ------------------------------------------------------------------------------
    h = am["offline_selection_hierarchy_fail_closed"]
    check(h["P_preservation_on_anchors"]["mean_abs_max_per_joint"] == 0.10
          and h["P_preservation_on_anchors"]["max_abs_max_per_joint"] == 0.25
          and h["T_target_improvement"]["min_relative_improvement_per_joint"] == 0.10
          and h["D_drift"]["parameter_shift_sq_max"] == 0.5, "thresholds are those frozen in rev3s")
    cl = am["closed_loop_gate_preregistered_separate_token"]["binding_gates"]
    check(cl["valid_cycle_count"] == ">= 1" and cl["penetration"].startswith("max <= 0.020")
          and cl["phase_timeout_stance"] == 0 and cl["hs_cancelled"] == "max 0 and final 0", "closed-loop gates preregistered unchanged")

    # --- offline hierarchy: pass, then each level failing in isolation with short circuit -------------------
    ok = B.offline_hierarchy(metrics())
    check(ok["verdict"] == "PASS" and ok["allowed_to_be_tested_closed_loop"] is True
          and all(v["result"] == "pass" for v in ok["levels"].values()), "a clean fixture passes all four levels")
    cases = [("I_integrity", {"integrity": {"clock_columns_zero": False}}),
             ("P_preservation", {"preservation": {"max_abs": [0.26, 0.1]}}),
             ("T_target_improvement", {"target": {"holdout_rmse": [0.60, 0.38]}}),
             ("D_drift", {"drift": {"parameter_shift_sq": 0.9}})]
    order = am["offline_selection_hierarchy_fail_closed"]["order"]
    for level, over in cases:
        res = B.offline_hierarchy(metrics(**over))
        check(res["verdict"] == "FAIL" and res["failed_level"] == level and res["allowed_to_be_tested_closed_loop"] is False,
              f"{level} fails in isolation")
        later = order[order.index(level) + 1:]
        check(all(res["levels"][l]["result"] == "not_evaluated" for l in later), f"short circuit after {level}")
    res = B.offline_hierarchy(metrics(preservation={"per_stratum_mean_abs": {"phase_fsm_swing_after_to": [0.11, 0.02]}}))
    check(res["failed_level"] == "P_preservation" and any("stratum" in v for v in res["levels"]["P_preservation"]["violations"]),
          "a single discrete stratum out of bound fails preservation (no gait phase silently sacrificed)")
    check("NO offline metric" in ok["no_walking_claim"], "the hierarchy explicitly refuses to claim walking")

    # --- closed-loop order restricted to survivors -----------------------------------------------------------
    off = {"A6": {"verdict": "PASS"}, "A3": {"verdict": "PASS"}, "A2": {"verdict": "FAIL"}, "A1": {"verdict": "FAIL"},
           "A4": {"verdict": "FAIL"}, "A5": {"verdict": "PASS"}}
    check(B.closed_loop_order(off) == ["A6", "A3", "A5"], "frozen order, most preserving first, survivors only")
    check(B.closed_loop_order({}) == [], "no survivor -> nothing is evaluated closed loop")

    # --- classification: bridge, not DAgger --------------------------------------------------------------------
    cls = am["classification"]
    check("NOT DAgger" in cls["what_this_is_NOT"] and "SUPERVISED ANCHORED BRIDGE" in cls["what_this_is"],
          "the stage is classified as a supervised anchored bridge, explicitly not DAgger")

    # --- no-clobber -----------------------------------------------------------------------------------------------
    tmp = Path(tempfile.mkdtemp(prefix="s1b_dry_"))
    try:
        o1 = B.dry_run(authorized_stage=B.AUTHORIZED_STAGE, out_dir=tmp)
        p1 = C.REPO / Path(o1["receipt_path"])
        check(p1.is_file(), "dry-run receipt written")
        o2 = B.dry_run(authorized_stage=B.AUTHORIZED_STAGE, out_dir=tmp)
        p2 = C.REPO / Path(o2["receipt_path"])
        check(p1 != p2 and p2.is_file() and C.sha256_file(p1) == o1["receipt_sha256"],
              "no-clobber: a second dry-run reserves a NEW path and leaves the first receipt byte-identical")
        check(o1["receipt"]["executed_in_this_stage"] == {"fit": False, "rollout": False, "collection": False, "export": False},
              "the dry-run receipt records that nothing was executed")
    finally:
        import shutil; shutil.rmtree(tmp, ignore_errors=True)
    print(json.dumps({"selftest": "PASS", "checks": CHECKS}, indent=1))
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
