"""Self-test rev4d repeat experiment. Stage-aware; includes key AND prose hygiene regressions."""
from __future__ import annotations
import json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_rev4d_dagger as D  # noqa: E402
import v26b_rev4c_dagger as C4  # noqa: E402
import v26b_rev4b_dagger as B4  # noqa: E402
import v26b_s1b_rollout as SB  # noqa: E402

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

    # --- lineage, pins, exclusive chain ------------------------------------------------------------------
    lin = D.verify_lineage()
    check(lin["amendment_rev4d"] == D.PIN_AMENDMENT_REV4D and lin["addendum_rev4c_a"] == D.PIN_ADDENDUM_REV4C_A
          and lin["amendment_rev4c"] == C4.PIN_AMENDMENT_REV4C and lin["amendment_rev4b"] == B4.PIN_AMENDMENT_REV4B,
          "lineage pins rev4b, rev4c, the rev4c addendum and rev4d")
    check("benchmark and protocol reference only" in lin["july_artifacts_role"]
          and "never init, never labels" in lin["july_artifacts_role"],
          "July artifacts are declared benchmark-only, never init and never labels")
    am = json.loads(D.AMENDMENT_REV4D.read_text())
    le = am["lineage_exclusive"]
    check(le["init_anchor_collection"].startswith("EXCLUSIVELY the S1A actor 8f3e0ce1")
          and "NO new collection" in le["init_anchor_collection"],
          "S1A is the sole init, anchor and collection policy; no new collection")
    check(am["ARCHITECTURAL_FRAMING_BINDING"]["single_variable"].startswith(
        "REV4D differs from REV4B in EXACTLY ONE dimension: trace_repeat 4 -> 1"),
        "the amendment states the single variable")
    check("declared_residual_confound" in am["ARCHITECTURAL_FRAMING_BINDING"]
          and "0.5895" in am["ARCHITECTURAL_FRAMING_BINDING"]["declared_residual_confound"],
          "the repeat/leakage residual confound is declared in advance")
    check(am["frozen_hyperparameters"]["logstd_note"].endswith("is NOT a choice of sigma")
          and am["frozen_hyperparameters"]["sigma"].startswith("UNRESOLVED"),
          "the logstd placeholder is explicitly not a sigma choice")
    for attr in ("PIN_AMENDMENT_REV4D", "PIN_ADDENDUM_REV4C_A"):
        old = getattr(D, attr)
        try:
            setattr(D, attr, "0" * 64); expect(D.verify_lineage, D.Rev4dError, f"tampered {attr} -> refused")
        finally: setattr(D, attr, old)

    # --- token guards ------------------------------------------------------------------------------------
    for bad in (None, "V26B-REV4C-BALANCE", "V26B-REV4B-JULY-DAGGER", "v26b-rev4d-repeat"):
        e = expect(lambda b=bad: D.run_stage(authorized_stage=b), D.Rev4dError, f"token {bad!r} refused")
        check("V26B-REV4D-REPEAT" in str(e), "guard names this stage's token")

    # --- THE single variable ------------------------------------------------------------------------------
    check(D.TRACE_REPEAT == 1 and B4.J_TRACE_REPEAT == 4, "the single variable: repeat 1 here, 4 in rev4b")
    check(D.PREFIX_ROWS == 392 and D.INTERPOLATION_STEPS == 0 and D.EXPECTED_AGGREGATE == 892,
          "full prefix, no interpolation, 892 rows")
    check(D.EXPECTED_SHARE == 392 / 892, "share is exactly 392/892")
    src = (HERE / "v26b_rev4d_dagger.py").read_text()
    check("B4.fit_july" in src, "the July fit function is reused unmodified")
    check(not any(f"{k} =" in src for k in ("J_EPOCHS", "J_BATCH", "J_LR", "J_ANCHOR_W", "J_SEED", "J_PATIENCE")),
          "rev4d redefines no July hyperparameter")
    for k in ("J_EPOCHS", "J_BATCH", "J_LR", "J_VAL_FRACTION", "J_PATIENCE", "J_CLIP_W", "J_LOGSTD_W",
              "J_ANCHOR_W", "J_SEED"):
        check(hasattr(B4, k), f"{k} inherited from rev4b")
    check("np.tile(obs_v, (TRACE_REPEAT, 1))" in src, "the repeat is the only knob in the aggregation")

    # --- REGRESSION: key hygiene ---------------------------------------------------------------------------
    expect(lambda: D.assert_no_foreign_labels({"kinematics_L20": {}}), D.Rev4dError, "kinematics_L20 key refused")
    expect(lambda: D.assert_no_foreign_labels({"a": {"L20_vs_S0D_same_states": 1}}), D.Rev4dError, "nested L20_* refused")
    expect(lambda: D.assert_no_foreign_labels({"d": {"offline_reference_block": {"x": 1}}}), D.Rev4dError,
           "foreign offline_reference_block refused at depth")
    expect(lambda: D.assert_no_foreign_labels({"lineage": {"offline_reference_block": {}}}), D.Rev4dError,
           "offline_reference_block refused even inside a provenance subtree")
    expect(lambda: D.assert_no_foreign_labels([{"S0D_vs_uIK_on_L20_states": 1}]), D.Rev4dError, "scan descends into lists")
    D.assert_no_foreign_labels({"lineage": {"rev3z_aggregate_states_preserved": {"L05": "x", "L20": "y"}}})
    check(True, "provenance subtrees may legitimately name other candidates")

    # --- REGRESSION: PROSE hygiene (the rev4c A2 defect) ----------------------------------------------------
    frozen = "closed-loop eligibility of A2. A PASS marks ONLY CLOSED_LOOP_ELIGIBLE_PENDING_ARCHITECT_AUDIT"
    expect(lambda: D.assert_no_foreign_labels({"seven_gates": {"meaning": frozen}}), D.Rev4dError,
           "the UN-RELABELLED frozen S1B prose is now caught (the exact rev4c defect)")
    expect(lambda: D.assert_no_foreign_labels({"scope": "metrics on the states L20 actually visited"}), D.Rev4dError,
           "foreign 'states X actually visited' prose is caught")
    rel = D.relabel_seven_gates({"gates": {}, "failed": [], "all_pass": False, "meaning": frozen}, D.ACTOR_LABEL)
    check(rel["meaning"].startswith("closed-loop eligibility of REV4D.")
          and "NOT a promotion" in rel["meaning"] and "does NOT resolve sigma" in rel["meaning"],
          "relabel_seven_gates rewrites the prose with this actor's label")
    check("meaning_relabelled_from_frozen_tool" in rel, "the relabelling is disclosed in the receipt")
    D.assert_no_foreign_labels(rel)
    check(True, "the relabelled block passes the prose scan")
    check(rel["gates"] is not None and set(rel) >= {"gates", "failed", "all_pass"},
          "relabelling preserves the frozen gate arithmetic keys")
    check("closed-loop eligibility of A2" in (HERE / "v26b_s1b_rollout.py").read_text(),
          "the frozen S1B tool is NOT modified: its prose is untouched on disk")
    D.assert_no_foreign_labels({"scope": "CLOSED-LOOP metrics on the states REV4D actually visited",
                                "actions_vs_source_actor": {"source": "the S0D actor, used only as a numerical reference"}})
    check(True, "correct rev4d prose and legitimate S0D references pass")
    D.assert_no_foreign_labels({"framing": {"reference_arm": "REV4B (prefix 392, repeat 4)"},
                                "reference_points": {"rev4b_repeat4_prefix392": 42}})
    check(True, "declared framing and benchmark subtrees may name the comparison arms")

    # --- preflight ------------------------------------------------------------------------------------------
    pre = D.preflight()
    check(pre["verdict"] == "GO" and pre["prefix_rows"] == 392 and pre["trace_repeat"] == 1,
          "preflight GO on the full prefix with repeat 1")
    check(pre["steps_contiguous_1_to_392"] is True and pre["obs35_matches_recorded_vectors"] is True
          and pre["obs35_and_labels_finite"] is True and pre["time_alignment_max_abs_difference_s"] == 0.0,
          "contiguity, obs35 exactness, finiteness and exact time alignment")
    check(pre["collisions_with_conflicting_labels"] == 0, "no conflicting collisions")
    nc = pre["negative_ankle_coverage"]
    check(nc["distinct_negative_time_indices"] == 65 and nc["windows"] == D.EXPECTED_NEGATIVE_WINDOWS
          and nc["rows_in_full_corpus"] == 97,
          "negative coverage exactly 65 distinct indices over the 5 expected windows")
    comp = pre["composition"]
    check(comp["aggregate_rows"] == 892 and comp["on_policy_share"] == 392 / 892
          and comp["on_policy_share_exact_fraction"] == "392/892", "composition and share exact")
    check(abs(comp["rev4b_share_for_contrast"] - 1568 / 2068) < 1e-15, "the rev4b contrast share is recorded")
    for attr, val in (("TRACE_REPEAT", 2), ("PREFIX_ROWS", 391)):
        old = getattr(D, attr)
        try:
            setattr(D, attr, val); expect(D.preflight, D.Rev4dError, f"a wrong {attr} fails closed")
        finally: setattr(D, attr, old)
    old_w = D.EXPECTED_NEGATIVE_WINDOWS
    try:
        D.EXPECTED_NEGATIVE_WINDOWS = [[1, 2]]
        expect(D.preflight, D.Rev4dError, "a wrong expected coverage fails closed")
    finally:
        D.EXPECTED_NEGATIVE_WINDOWS = old_w

    # --- aggregation ------------------------------------------------------------------------------------------
    agg = D.build_aggregate(pre)
    check(agg["observations"].shape[0] == 892 and agg["actions"].shape[0] == 892, "aggregate shape 892")
    obs500 = np.asarray(pre["_view"]["obs"], dtype=np.float32)
    check(np.array_equal(agg["observations"][:500], obs500), "first 500 rows are the corpus verbatim")
    check(np.array_equal(agg["observations"][500:], np.asarray(pre["_obsS"], dtype=np.float32)),
          "the on-policy block is the prefix exactly once: no tiling")
    check(len({agg["observations"][i].tobytes() for i in range(500, 892)}) == 392,
          "no exact duplicates in the on-policy block, so no copies straddle the validation split")

    # --- criteria -------------------------------------------------------------------------------------------
    check(D.SURVIVAL_THRESHOLD_STEPS == 116, "primary gate threshold is 116")
    check(am["CRITERIA"]["primary_gate"]["rule"] == "STRICTLY GREATER THAN 116", "the rule is strict")
    check(am["CRITERIA"]["primary_gate"]["reference_points"]["rev4c_repeat4_prefix68"] == 116
          and am["CRITERIA"]["primary_gate"]["reference_points"]["rev4b_repeat4_prefix392"] == 42,
          "reference points recorded")
    check("no_automatic_promotion" in am["CRITERIA"], "no automatic promotion")

    # --- prohibitions ----------------------------------------------------------------------------------------
    check(not any(t in src for t in ("train_ppo", "PPOConfig", "sigma_sweep", "choose_sigma", "set_sigma")),
          "no PPO or sigma machinery")
    check(src.count("subprocess.run(") == 1, "exactly one harness invocation")

    # --- post-run --------------------------------------------------------------------------------------------
    stage = {"post_run": ran}
    if ran:
        rec = json.loads((D.OUT_CAND / D.RECEIPT_NAME).read_text())
        D.assert_no_foreign_labels(rec, "materialised receipt")
        check(True, "the materialised fit receipt passes key AND prose hygiene")
        check(rec["deployable"] is False and rec["actor_label"] == "REV4D", "flags and actor label")
        ds = rec["dataset"]
        check(ds["aggregate_rows"] == 892 and ds["prefix_rows"] == 392 and ds["trace_repeat"] == 1
              and ds["interpolation_steps"] == 0, "dataset recorded")
        jp = rec["july_protocol"]
        check(jp["batch_size"] == 64 and jp["learning_rate"] == 3e-4 and jp["anchor_weight"] == 1e-5
              and jp["seed"] == 123, "July hyperparameters unchanged")
        check(rec["offline"]["binding"]["integrity_invariants"]["logstd_byte_identical_to_init"] is True,
              "logstd byte-identical to the S1A init")
        check(rec["save_reload_exact"] is True, "save/reload exact")
        m = rec["offline"]["measures"]
        check(all(k in m for k in ("on_corpus_500", "on_full_prefix_392", "on_all_97_negative_rows_of_the_corpus")),
              "metrics on corpus, full prefix and all 97 negative rows")
        expect(lambda: D.run_stage(authorized_stage=D.AUTHORIZED_STAGE, progress=False), FileExistsError,
               "a second execution is refused")
        stage["offline_pass"] = rec["offline"]["all_binding_pass"]
        if D.JOB_DIR.exists():
            rr = json.loads((D.JOB_DIR / D.ROLLOUT_RECEIPT_NAME).read_text())
            D.assert_no_foreign_labels(rr, "materialised rollout receipt")
            check(True, "the materialised rollout receipt passes key AND prose hygiene")
            check(rr["seven_gates"]["meaning"].startswith("closed-loop eligibility of REV4D."),
                  "the seven-gate prose names REV4D, not a foreign actor")
            check(rr["promotion"]["promoted"] is False, "not promoted")
            pg = rr["primary_gate"]
            check(pg["threshold"] == 116 and pg["rule"] == "STRICTLY GREATER THAN 116"
                  and pg["verdict"] in ("PASS", "FAIL")
                  and pg["verdict"] == ("PASS" if pg["observed_steps"] > 116 else "FAIL"),
                  "the primary gate is recorded and its verdict follows the strict rule")
            stage["survival_steps"] = pg["observed_steps"]; stage["primary_gate"] = pg["verdict"]
    print(json.dumps({"selftest": "PASS", "checks": CHECKS, **stage}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
