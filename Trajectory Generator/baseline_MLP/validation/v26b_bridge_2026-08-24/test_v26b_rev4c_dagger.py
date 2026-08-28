"""Self-test rev4c balance experiment. Stage-aware; includes the label-hygiene regressions."""
from __future__ import annotations
import json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_rev4c_dagger as D  # noqa: E402
import v26b_rev4b_dagger as B4  # noqa: E402
import v26b_l20_rollout as LR  # noqa: E402
import v26b_s1a_bc as A  # noqa: E402
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
    ran = D.OUT_CAND.exists()

    # --- lineage, pins, framing ---------------------------------------------------------------------
    lin = D.verify_lineage()
    check(lin["amendment_rev4c"] == D.PIN_AMENDMENT_REV4C and lin["addendum_rev4b_a"] == D.PIN_ADDENDUM_REV4B_A
          and lin["amendment_rev4b"] == B4.PIN_AMENDMENT_REV4B, "lineage pins rev4b, its addendum and rev4c")
    check("collection budget" in lin["framing"] and "operator-divergent" in lin["framing"],
          "the framing states the cap is a collection budget and that rev4c is operator-divergent")
    am = json.loads(D.AMENDMENT_REV4C.read_text())
    fr = am["ARCHITECTURAL_FRAMING_BINDING"]
    check("CONTROLLED DAgger COLLECTION BUDGET" in fr["what_the_68_cap_IS"]
          and any("causally valid prefix" in s for s in fr["what_the_68_cap_IS_NOT"])
          and any("literal replication" in s for s in fr["what_the_68_cap_IS_NOT"]),
          "the amendment refuses both the 'causally valid prefix' and the 'literal replication' readings")
    check("not faithful to the July OPERATOR" in fr["fidelity_statement"].replace("NOT", "not"),
          "the fidelity statement is explicit")
    for attr in ("PIN_AMENDMENT_REV4C", "PIN_ADDENDUM_REV4B_A"):
        old = getattr(D, attr)
        try:
            setattr(D, attr, "0" * 64); expect(D.verify_lineage, D.Rev4cError, f"tampered {attr} -> refused")
        finally: setattr(D, attr, old)

    # --- token guards -----------------------------------------------------------------------------------
    for bad in (None, "V26B-REV4B-JULY-DAGGER", "V26B-L20-NOMINAL-ROLLOUT", "v26b-rev4c-balance"):
        e = expect(lambda b=bad: D.run_stage(authorized_stage=b), D.Rev4cError, f"token {bad!r} refused")
        check("V26B-REV4C-BALANCE" in str(e), "guard names this stage's token")

    # --- single variable: only the budget changes ----------------------------------------------------------
    check(D.PREFIX_CAP == 68 and D.EXPECTED_AGGREGATE == 772 and B4.J_TRACE_REPEAT == 4,
          "the cap is 68 and the aggregate is 772")
    for k in ("J_EPOCHS", "J_BATCH", "J_LR", "J_VAL_FRACTION", "J_PATIENCE", "J_CLIP_W", "J_LOGSTD_W",
              "J_ANCHOR_W", "J_SEED", "J_TRACE_REPEAT"):
        check(hasattr(B4, k), f"{k} is reused from rev4b, not redefined")
    src = (HERE / "v26b_rev4c_dagger.py").read_text()
    check("B4.fit_july" in src, "the July fit function is reused unmodified from rev4b")
    check(not any(f"{k} =" in src for k in ("J_EPOCHS", "J_BATCH", "J_LR", "J_ANCHOR_W", "J_SEED")),
          "rev4c redefines no July hyperparameter: the only changed variable is the budget")

    # --- REGRESSION: no hard-coded foreign actor labels, no foreign offline_reference_block ------------------
    expect(lambda: D.assert_no_foreign_labels({"kinematics_L20": {}}), D.Rev4cError,
           "a kinematics_L20 key is refused")
    expect(lambda: D.assert_no_foreign_labels({"a": {"L20_vs_S0D_same_states": 1}}), D.Rev4cError,
           "a nested L20_* key is refused")
    expect(lambda: D.assert_no_foreign_labels({"diagnostics": {"offline_reference_block": {"x": 1}}}), D.Rev4cError,
           "a foreign offline_reference_block is refused at any depth")
    expect(lambda: D.assert_no_foreign_labels([{"S0D_vs_uIK_on_L20_states": 1}]), D.Rev4cError,
           "the scan descends into lists")
    D.assert_no_foreign_labels({"actor_label": "REV4C", "kinematics": {"knee_q": {}},
                               "actions_vs_source_actor": {"source": "the S0D actor"}})
    check(True, "the parameterised rev4c shape passes the hygiene scan")
    old_rec = json.loads((LR.JOB_DIR / LR.RECEIPT_NAME).read_text())
    expect(lambda: D.assert_no_foreign_labels(old_rec), D.Rev4cError,
           "the OLD rev4b-style receipt shape would be refused by the new contract (regression proven)")
    check("actor_label" in src and 'f"CLOSED-LOOP metrics on the states {label}' in src,
          "the analyzer parameterises the actor label instead of hard-coding it")
    D.assert_no_foreign_labels({"lineage": {"rev3z_aggregate_states_preserved": {"L05": "x", "L20": "y"}}})
    check(True, "a PROVENANCE subtree may legitimately name other candidates: lineage is exempt from the key scan")
    expect(lambda: D.assert_no_foreign_labels({"diagnostics": {"kinematics_L20": {}}}), D.Rev4cError,
           "the exemption does NOT extend to diagnostics or metrics")
    expect(lambda: D.assert_no_foreign_labels({"lineage": {"offline_reference_block": {}}}), D.Rev4cError,
           "offline_reference_block stays forbidden even inside a provenance subtree")
    check(D.EXEMPT_PROVENANCE_SUBTREES == ("lineage", "parents_immutable", "preflight")
          and D.INCIDENT_RECORD["disclosed"] is True,
          "the exempt subtrees are declared and the execution incident is recorded in the module")

    # --- preflight on the capped prefix ------------------------------------------------------------------------
    pre = D.preflight()
    check(pre["verdict"] == "GO" and pre["prefix_cap"] == 68, "preflight GO on the capped prefix")
    check(pre["steps_contiguous_1_to_68"] is True and pre["obs35_matches_recorded_vectors"] is True
          and pre["obs35_and_labels_finite"] is True and pre["time_alignment_max_abs_difference_s"] == 0.0,
          "steps contiguous, obs35 exact, labels finite, time alignment exact")
    check(pre["duplicates_in_capped_prefix"] == 0 and pre["collisions_with_conflicting_labels"] == 0,
          "no duplicates and no conflicting collisions")
    comp = pre["composition"]
    check(comp["aggregate_rows"] == 772 and abs(comp["on_policy_share"] - 0.3523316062176166) < 1e-12,
          "aggregate 772 and on-policy share equal to July's to machine precision")
    check(pre["negative_ankle_coverage"]["rows_in_capped_prefix"] == 9
          and pre["negative_ankle_coverage"]["rows_in_full_corpus"] == 97,
          "negative-ankle coverage reported: the cap keeps only 9 of the prefix's negative rows")
    old_cap = D.PREFIX_CAP
    try:
        D.PREFIX_CAP = 67
        expect(D.preflight, D.Rev4cError, "a cap that breaks the expected aggregate fails closed")
    finally:
        D.PREFIX_CAP = old_cap

    # --- aggregation ---------------------------------------------------------------------------------------------
    agg = D.build_aggregate(pre)
    check(agg["observations"].shape[0] == 772 and agg["actions"].shape[0] == 772, "aggregate shape")
    obs500 = np.asarray(pre["_view"]["obs"], dtype=np.float32)
    check(np.array_equal(agg["observations"][:500], obs500), "the first 500 rows are the corpus verbatim")
    blk = agg["observations"][500:568]
    for rep in range(1, 4):
        check(np.array_equal(agg["observations"][500 + rep * 68: 500 + (rep + 1) * 68], blk),
              f"tile repetition {rep + 1} is an exact copy")

    # --- criteria ---------------------------------------------------------------------------------------------------
    check(D.SURVIVAL_THRESHOLD_STEPS == 100, "the primary causal threshold is survival > 100 steps")
    crit = am["CRITERIA"]
    check(crit["primary_causal_test"]["NO_GO_hypothesis_refuted"].startswith("<= 100")
          and "no_automatic_promotion" in crit, "criteria recorded, including no automatic promotion")

    # --- no PPO, no sigma, no other candidate -----------------------------------------------------------------------
    check(not any(t in src for t in ("train_ppo", "PPOConfig", "sigma_sweep", "choose_sigma", "set_sigma")),
          "no PPO or sigma machinery in the module (the words 'PPO' and 'sigma' appear only in prohibition prose)")
    check("No PPO" in src and "UNRESOLVED" in src, "the module states the PPO and sigma prohibitions explicitly")
    check(src.count("subprocess.run(") == 1, "exactly one harness invocation")

    # --- post-run -----------------------------------------------------------------------------------------------------
    stage = {"post_run": ran}
    if ran:
        rec = json.loads((D.OUT_CAND / D.RECEIPT_NAME).read_text())
        D.assert_no_foreign_labels(rec, "materialised receipt")
        check(True, "the materialised fit receipt passes the hygiene contract")
        check(rec["deployable"] is False and rec["actor_label"] == "REV4C", "flags and actor label")
        check(rec["dataset"]["aggregate_rows"] == 772 and rec["dataset"]["prefix_rows"] == 68, "dataset recorded")
        jp = rec["july_protocol"]
        check(jp["batch_size"] == 64 and jp["learning_rate"] == 3e-4 and jp["anchor_weight"] == 1e-5
              and jp["seed"] == 123, "the July hyperparameters are unchanged from rev4b")
        check(rec["offline"]["binding"]["integrity_invariants"]["logstd_byte_identical_to_init"] is True,
              "logstd byte-identical to S1A after the fit")
        m = rec["offline"]["measures"]
        check(all(k in m for k in ("on_corpus_500", "on_capped_prefix_68", "on_all_97_negative_rows_of_the_corpus")),
              "metrics reported on corpus, capped prefix and all 97 negative rows")
        expect(lambda: D.run_stage(authorized_stage=D.AUTHORIZED_STAGE, progress=False), FileExistsError,
               "a second execution is refused")
        stage["offline_pass"] = rec["offline"]["all_binding_pass"]
        if D.JOB_DIR.exists():
            rr = json.loads((D.JOB_DIR / D.ROLLOUT_RECEIPT_NAME).read_text())
            D.assert_no_foreign_labels(rr, "materialised rollout receipt")
            check(True, "the materialised rollout receipt passes the hygiene contract")
            check(rr["promotion"]["promoted"] is False and "no automatic promotion" in rr["promotion"]["note"],
                  "no promotion, automatic or otherwise")
            pct = rr["primary_causal_test"]
            check(pct["threshold"] == 100 and pct["verdict"] in ("SUPPORTED", "REFUTED"),
                  "the primary causal test is recorded with its verdict")
            stage["survival_steps"] = pct["observed_steps"]; stage["causal_verdict"] = pct["verdict"]
    print(json.dumps({"selftest": "PASS", "checks": CHECKS, **stage}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
