"""Self-test rev4b July-faithful DAgger. Stage-aware: valid before AND after the single execution."""
from __future__ import annotations
import json, sys, tempfile
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_rev4b_dagger as D  # noqa: E402
import v26b_l20_rollout as LR  # noqa: E402
import v26b_s1a_bc as A  # noqa: E402
import v26b_s1a_rollout as SA  # noqa: E402
import v26b_s1c_protocol as SC  # noqa: E402
import v26b_s1b_rollout as SB  # noqa: E402
import f0_common as C  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_refit as RF  # noqa: E402

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

    # --- lineage and pins -------------------------------------------------------------------------
    lin = D.verify_lineage()
    check(lin["amendment_rev4b"] == D.PIN_AMENDMENT_REV4B and lin["amendment_rev4a"] == LR.PIN_AMENDMENT_REV4A,
          "lineage pins rev4a and rev4b")
    check(lin["s1a_actor_digest"] == D.PIN_S1A_ACTOR == A.P0_ACTOR_DIGEST, "init is the pinned S1A actor")
    check("not init, source or anchor" in lin["l20_not_promoted"], "L20 is explicitly excluded from this stage")
    old = D.PIN_AMENDMENT_REV4B
    try:
        D.PIN_AMENDMENT_REV4B = "0" * 64
        expect(D.verify_lineage, D.Rev4bError, "tampered rev4b pin -> refused")
        expect(D._amendment, D.Rev4bError, "the amendment reader is pin-guarded")
    finally:
        D.PIN_AMENDMENT_REV4B = old

    # --- July values are the VERIFIED ones, not the August ones -------------------------------------
    check(D.J_EPOCHS == 400 and D.J_BATCH == 64 and D.J_LR == 3e-4 and D.J_VAL_FRACTION == 0.20
          and D.J_PATIENCE == 60 and D.J_CLIP_W == 1.0 and D.J_LOGSTD_W == 0.1
          and D.J_ANCHOR_W == 1e-5 and D.J_SEED == 123 and D.J_TRACE_REPEAT == 4,
          "the July constants match the values verified from target_domain_imitation.py / target_domain_dagger.py")
    check(D.J_ANCHOR_W != 1e-3 and D.J_BATCH != 256 and D.J_SEED != 2026,
          "the August R-series values (anchor 1e-3, batch 256, seed 2026) are NOT used")
    am = json.loads(D.AMENDMENT_REV4B.read_text())
    check(am["ANCHOR_1e-3_NOT_CONFIRMED_BY_JULY"]["verification_result"].startswith("NOT CONFIRMED")
          and "1e-05" in am["ANCHOR_1e-3_NOT_CONFIRMED_BY_JULY"]["decision"],
          "the amendment records that 1e-3 is NOT the July value and that 1e-5 is used")
    check(am["JULY_VALUES_VERIFIED_FROM_SOURCE_NOT_MEMORY"]["gradient_clipping"].startswith("NONE"),
          "no gradient clipping, as verified in the July sources")
    src = (HERE / "v26b_rev4b_dagger.py").read_text()
    check("clip_grad" not in src, "the fit performs no gradient clipping")
    check(all(t not in src for t in ("phase_aux", "hinge_term", "preservation_role", "two_role_loss")),
          "no phase auxiliary head, no hinge, no S0D preservation term")

    # --- token guards -------------------------------------------------------------------------------
    for bad in (None, "V26B-L20-NOMINAL-ROLLOUT", "V26B-S1C-2Z-FIT", "v26b-rev4b-july-dagger"):
        e = expect(lambda b=bad: D.run_stage(authorized_stage=b), D.Rev4bError, f"token {bad!r} refused")
        check("V26B-REV4B-JULY-DAGGER" in str(e), "guard names this stage's token")

    # --- preflight: all items, and its fail-closed nature ---------------------------------------------
    pre = D.preflight()
    check(pre["verdict"] == "GO", "preflight GO")
    check(pre["s1a_trace"]["rows"] == 392 and pre["s1a_trace"]["sha256"] == D.PIN_S1A_TRACE
          and pre["s1a_trace"]["steps_contiguous"] is True and pre["s1a_trace"]["obs35_matches_recorded_vectors"] is True,
          "392 real rows, contiguous order, digest verified, obs35 matches the recorded vectors")
    check(pre["time_alignment"]["max_abs_difference_s"] == 0.0
          and "teacher_index = step - 1" in pre["time_alignment"]["rule"],
          "causal same-time alignment with the July index rule")
    check(pre["labels"]["not_healthy_symmetry"] is True and pre["labels"]["not_time_shifted"] is True
          and pre["labels"]["rows"] == 392, "labels recomputed with the same teacher semantics")
    dc = pre["dedup_and_collisions"]
    check(dc["exact_duplicates_corpus"] == 0 and dc["exact_duplicates_prefix"] == 0
          and dc["collisions_with_conflicting_labels"] == 0, "dedup and collision audit clean")
    comp = pre["composition"]
    check(comp["teacher_corpus_rows"] == 500 and comp["prefix_rows"] == 392 and comp["trace_repeat"] == 4
          and comp["aggregate_rows"] == 2068 and comp["no_s0d_l20_a2_data"] is True,
          "dataset = BC-IK corpus + tiled S1A prefix only")
    cov = pre["negative_target_coverage"]
    check(cov["prefix_negative_rows"] == 65 and len(cov["prefix_negative_windows"]) == 5
          and cov["alignment_permits_sign_recovery"] is True and "NEGATIVE ankle commands" in cov["proof"],
          "negative-target coverage quantified with the alignment proof")
    check(pre["logstd"]["byte_identical_placeholder"] is True and "UNRESOLVED" in pre["sigma"],
          "logstd byte-identical to the S1A placeholder; sigma unresolved")
    real = D.PIN_S1A_TRACE
    try:
        D.PIN_S1A_TRACE = "0" * 64
        expect(D.preflight, D.Rev4bError, "a wrong trace digest makes the preflight fail closed")
    finally:
        D.PIN_S1A_TRACE = real
    real_n = D.PREFIX_ROWS
    try:
        D.PREFIX_ROWS = 391
        expect(D.preflight, D.Rev4bError, "a wrong row count makes the preflight fail closed")
    finally:
        D.PREFIX_ROWS = real_n

    # --- July aggregation semantics --------------------------------------------------------------------
    agg = D.build_aggregate(pre)
    check(agg["observations"].shape[0] == 2068 and agg["actions"].shape[0] == 2068,
          "aggregate = 500 + 392*4 rows")
    obs500 = np.asarray(pre["_view"]["obs"], dtype=np.float32)
    check(np.array_equal(agg["observations"][:500], obs500), "the first 500 rows are the teacher corpus verbatim")
    blk = agg["observations"][500:500 + 392]
    for rep in range(1, 4):
        check(np.array_equal(agg["observations"][500 + rep * 392: 500 + (rep + 1) * 392], blk),
              f"tile repetition {rep + 1} is an exact copy: no dedup, no truncation")
    check(np.array_equal(agg["actions"][500:500 + 392], np.asarray(pre["_labels"], dtype=np.float32)),
          "the on-policy labels are the time-aligned teacher actions")

    # --- offline gate semantics on fixtures ---------------------------------------------------------------
    g = {"binding": {"integrity_invariants": {"pass": True}, "function_preservation": {"pass": True},
                     "fit_convergence": {"pass": True}}}
    failed = [k for k, v in g["binding"].items() if not v["pass"]]
    check(failed == [], "fixture with all binding gates passing")
    check(D.B3_REFERENCE_THRESHOLD == -0.03, "the documented F2R B3 reference threshold is -0.03")

    # --- no promotion, no sigma, no collection ------------------------------------------------------------
    check('"deployable": True' not in src and "deployable=True" not in src, "no deployable marking is ever set")
    check("F1.ROLLOUT_EVAL" in src and src.count("subprocess.run(") == 1
          and "collect" not in src.lower().replace("collection", ""),
          "the module invokes the frozen rollout harness exactly once and performs no collection")
    check(D.PASS_STATUS == "CLOSED_LOOP_ELIGIBLE_PENDING_ARCHITECT_AUDIT", "a PASS marks only eligibility pending audit")

    # --- post-run ------------------------------------------------------------------------------------------
    stage = {"post_run": ran}
    if ran:
        rec = json.loads((D.OUT_CAND / D.RECEIPT_NAME).read_text())
        check(rec["deployable"] is False and rec["sigma_unresolved"] is True, "candidate flags")
        check(rec["init"]["actor_digest"] == D.PIN_S1A_ACTOR and "collection policy" in rec["init"]["role"],
              "S1A is init, collection policy and anchor")
        jp = rec["july_protocol"]
        check(jp["batch_size"] == 64 and jp["learning_rate"] == 3e-4 and jp["anchor_weight"] == 1e-5
              and jp["seed"] == 123 and jp["patience"] == 60 and jp["gradient_clipping"] == "none",
              "the receipt records the verified July hyperparameters")
        check(rec["dataset"]["aggregate_rows"] == 2068 and "no truncation, no dedup" in rec["dataset"]["operation"],
              "the receipt records the July aggregation")
        og = rec["offline_gates"]
        check(og["binding"]["integrity_invariants"]["logstd_byte_identical_to_S1A"] is True,
              "logstd byte-identical to S1A after the fit")
        check("INFORMATIONAL" in og["preregistered_measures"]["max_abs_vs_S0D_INFORMATIONAL"]["status"]
              and "DIAGNOSTIC" in og["preregistered_measures"]["healthy_symmetry_DIAGNOSTIC"]["status"],
              "max_abs vs S0D is informational and healthy symmetry is diagnostic")
        expect(lambda: D.run_stage(authorized_stage=D.AUTHORIZED_STAGE, progress=False), FileExistsError,
               "a second execution is refused")
        stage["offline_pass"] = og["all_binding_pass"]
        if D.JOB_DIR.exists():
            rr = json.loads((D.JOB_DIR / D.ROLLOUT_RECEIPT_NAME).read_text())
            check(rr["deployable"] is False and rr["status"] in (D.PASS_STATUS, D.FAIL_STATUS), "rollout receipt flags and status")
            check(rr["qualitative_ankle_gate"]["binding"] is True
                  and "never the phase field" in rr["qualitative_ankle_gate"]["ex_ante_window"],
                  "the qualitative ankle gate is binding and uses the ex-ante IK window")
            check(rr["analysis"]["trace_sha256"] == C.sha256_file(D.JOB_DIR / "rollout_policy_trace.json"),
                  "rollout trace digest matches the receipt")
            stage["rollout_status"] = rr["status"]; stage["rollout_failed"] = rr.get("failed")
    print(json.dumps({"selftest": "PASS", "checks": CHECKS, **stage}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
