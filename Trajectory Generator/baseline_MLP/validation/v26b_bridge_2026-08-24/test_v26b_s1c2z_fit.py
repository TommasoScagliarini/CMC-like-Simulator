"""Static + numerical self-test of the rev3z S1C-2Z fit. Stage-aware; no rollout, no production change."""
from __future__ import annotations
import hashlib, json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_s1c2z_fit as Z  # noqa: E402
import v26b_s1c2_preflight as PF  # noqa: E402
import v26b_s1c1_fit as F1  # noqa: E402
import v26b_s1c_protocol as SC  # noqa: E402
import v26b_s1b_fit as T  # noqa: E402
import v26b_s1b_protocol as B  # noqa: E402
import v26b_s1b_rollout as SB  # noqa: E402
import v26b_s1a_bc as A  # noqa: E402
import v26b_s1_fit as FIT  # noqa: E402
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
    ran = Z.out_dir_for("L05").exists()

    # --- lineage / pins / prior artifacts untouched -------------------------------------------------
    lin = Z.verify_lineage()
    check(lin["amendment_rev3z"] == Z.PIN_AMENDMENT_REV3Z and lin["amendment_rev3y"] == PF.PIN_AMENDMENT_REV3Y
          and lin["addendum_rev3x_a_status"] == PF.PIN_ADDENDUM_STATUS, "lineage pins rev3x-addendum, rev3y and rev3z")
    check(lin["s1c1_aggregate_unmutated"] == PF.PIN_S1C1_AGGREGATE, "the S1C-1 aggregate is still byte-identical")
    check("no rev3y window-mean candidate exists" in lin["rev3y_rejected_unexecuted"],
          "rev3y is recorded as rejected and unexecuted")
    old = Z.PIN_AMENDMENT_REV3Z
    try:
        Z.PIN_AMENDMENT_REV3Z = "0" * 64
        expect(Z.verify_lineage, Z.S1C2ZError, "tampered rev3z pin -> refused")
        expect(Z._amendment, Z.S1C2ZError, "the amendment reader is pin-guarded")
    finally:
        Z.PIN_AMENDMENT_REV3Z = old

    # --- token guards -------------------------------------------------------------------------------
    for bad in (None, "V26B-S1C-2-FIT", "V26B-S1C-2-PREFLIGHT", "V26B-S1C-1", "v26b-s1c-2z-fit"):
        e = expect(lambda b=bad: Z.run_fit(authorized_stage=b), Z.S1C2ZError, f"token {bad!r} refused")
        check("V26B-S1C-2Z-FIT" in str(e), "guard names this stage's token")
    e = expect(lambda: Z.run_rollout(authorized_stage=Z.AUTHORIZED_STAGE), Z.S1C2ZError, "rollout refused here")
    check("NOT granted" in str(e), "the rollout guard names its own future token")
    src = (HERE / "v26b_s1c2z_fit.py").read_text()
    check(not any(t in src for t in ("subprocess.run(", "Popen", "rollout_eval", "--checkpoint", "np.tile")),
          "no closed-loop, collection or tiling primitive in the fit module")
    check("if False else" not in src and "sha256_bytes" not in src, "no dead code left in the fit module")

    # --- closed grid, numerics, margin --------------------------------------------------------------
    check([c["id"] for c in Z.CANDIDATES] == ["L05", "L10", "L20"]
          and [c["lambda_h"] for c in Z.CANDIDATES] == [0.5, 1.0, 2.0], "exactly the three preregistered lambdas")
    check(Z.DELTA == 0.0 and Z.ANKLE_DECODE_GAIN == 0.7 and Z.R_RATIO == 5.0 and Z.EPOCHS == 300
          and T.BATCH == 256 and T.LR == 1e-4 and T.SEED == 2026 and Z.T.ANCHOR_W is FIT.ANCHOR_WEIGHT,
          "delta 0, ankle gain 0.7, r 5, 300 epochs, batch 256, Adam 1e-4, seed 2026, July anchor by reference")
    am = json.loads(Z.AMENDMENT_REV3Z.read_text())
    check(am["candidates_closed_grid"]["open_search"] is False and am["candidates_closed_grid"]["no_fourth_value"] is True
          and am["objective"]["margin_delta"] == 0.0, "the amendment freezes a closed grid and delta = 0")

    # --- the hinge: zero for q<=0, positive only for q>0 ------------------------------------------------
    import torch
    m = torch.tensor([[0.0, -0.5], [0.0, -1e-9], [0.0, 0.0], [0.0, 1e-3], [0.0, 0.5]])
    allrows = torch.ones(5, dtype=torch.bool)
    for i in range(3):
        one = torch.zeros(5, dtype=torch.bool); one[i] = True
        check(float(Z.hinge_term(m, one, 1.0)) == 0.0, f"hinge is EXACTLY zero for q <= 0 (row {i})")
    for i in (3, 4):
        one = torch.zeros(5, dtype=torch.bool); one[i] = True
        q = 0.7 * float(m[i, 1])
        got = float(Z.hinge_term(m, one, 1.0))
        check(abs(got - q * q) <= 1e-6 * max(q * q, 1e-12) and got > 0.0,
              f"hinge equals (0.7*m_ankle)^2 for q > 0 (row {i}), to float32 relative precision")
    check(abs(float(Z.hinge_term(m, allrows, 2.0)) - 2.0 * float(Z.hinge_term(m, allrows, 1.0))) < 1e-15,
          "the hinge scales linearly with lambda_h")
    check(float(Z.hinge_term(m, torch.zeros(5, dtype=torch.bool), 1.0)) == 0.0,
          "a batch without hinge rows contributes exactly zero")
    knee_only = torch.tensor([[5.0, -0.5]])
    check(float(Z.hinge_term(knee_only, torch.ones(1, dtype=torch.bool), 1.0)) == 0.0,
          "the hinge reads the ANKLE channel only: a large positive knee command does not trigger it")

    # --- row selection: hinge rows, early window, holdout ------------------------------------------------
    corpus = B.build_corpus(); view = corpus["view"]
    step = np.arange(1, 501)
    hm = Z.hinge_mask(view)
    y = SC.decode_action(view["u_ik"])[:, 1] < 0.0
    check(int(hm.sum()) == 64, "64 hinge rows, as frozen in rev3z")
    check(bool(np.all(y[hm])) and bool(np.all(view["train"][hm])), "every hinge row is a TRAIN row with a negative IK target")
    early = (step >= 6) & (step <= 14)
    check(int((view["train"] & y & early).sum()) == 9 and int((hm & early).sum()) == 0,
          "the 9 early-window train rows are excluded from the hinge")
    check(int((view["train"] & y).sum()) == 73 and int(hm.sum()) == 73 - 9,
          "the hinge acts on the 73 negative train rows minus the 9 early ones")
    check(not np.any(hm & view["hold"]) and not np.any(hm & view["embargo"]),
          "no hinge row lies in the holdout or in the embargo")
    check(int((view["hold"] & y).sum()) == 24 and Z.HOLDOUT_WINDOW == (267, 290),
          "the holdout window [267,290] holds 24 negative rows and never enters the fit")
    wins = Z.windows(view)
    check(wins == [(6, 14), (112, 132), (174, 179), (267, 290), (330, 334), (423, 445), (482, 490)],
          "the seven windows match the frozen structure")

    # --- labels are NOT modified -------------------------------------------------------------------------
    before = hashlib.sha256(view["u_ik"].tobytes()).hexdigest()
    v2 = FIT.build_s1_task()
    check(hashlib.sha256(v2["u_ik"].tobytes()).hexdigest() == before,
          "the IK task labels are byte-identical across rebuilds: nothing blends or weights them")
    check("weighted" not in src.split("def fit_candidate")[1].split("def ")[0]
          and "window_mean" not in src and "beta" not in src.split("def fit_candidate")[1].split("def ")[0],
          "the fit path contains no weighting, blending or window-mean of the labels")
    check("T.two_role_loss(" in src, "the frozen two-role pointwise loss is reused unmodified")

    # --- frozen T1R semantics: shuffled union, per-role means, no sequential batches -----------------------
    body = src.split("def fit_candidate")[1].split("def morphology_diagnostics")[0]
    check("rng.permutation(n)" in body and "np.concatenate([obs_s, obs_s])" in body,
          "one deterministic shuffled union of the two identical-input role blocks")
    check("sequential" not in body.lower().replace("no sequential minibatches", ""),
          "no sequential batching is introduced")
    check("is_hinge = torch.as_tensor(np.concatenate([np.zeros(n_role, bool), hm]))" in body,
          "the hinge is applied to the TASK block only, so it cannot be double counted on the anchor block")
    mm = torch.tensor([[0.0, 1.0], [0.0, 1.0]])
    anchor_block = torch.tensor([True, False])
    check(float(Z.hinge_term(mm, ~anchor_block, 1.0)) == float(Z.hinge_term(mm, torch.tensor([False, True]), 1.0)),
          "the hinge mask selects exactly the task-block rows")

    # --- init exclusivity ------------------------------------------------------------------------------------
    d = RF.validate_init_state(T.fresh_init(), expected_actor_digest=FIT.PIN_S0D_ACTOR)["actor_digest"]
    check(d == FIT.PIN_S0D_ACTOR and d != A.P0_ACTOR_DIGEST and d != SB.PIN_A2_ACTOR,
          "init is the pinned S0D and differs from S1A and A2")
    forb = am["unchanged_and_pinned"]["forbidden_as_init_source_anchor"]
    check(all(k in forb for k in ("W2", "W4", "W8", "A2", "S1A")), "the forbidden init/source list is explicit")

    # --- logstd / clock invariants on the init ------------------------------------------------------------------
    init = T.fresh_init()
    st = RF.validate_init_state(init, expected_actor_digest=FIT.PIN_S0D_ACTOR)
    check(bool(st["clock_columns_zero"]) and bool(st["sigma_head"]["logstd_bias_exact"])
          and tuple(init.keys()) == RF.EXPECTED_KEY_ORDER, "init: clock columns zero, logstd placeholder, 10 keys")

    # --- output naming, deployable flag, state derivation ---------------------------------------------------------
    for cid in ("L05", "L20"):
        p = Z.out_dir_for(cid)
        check(p.name == f"S1C2Z_{cid}_35D_NONDEPLOYABLE" and p.parent.name == "candidates"
              and "student" not in p.resolve().parts, f"{cid} materialises under candidates/, never student/")
    check(Z.STATE_FAIL == "OFFLINE_FAILED_QUARANTINED" and Z.STATE_PASS == "OFFLINE_PASSED_ROLLOUT_NOT_AUTHORISED",
          "the two offline states are exactly the ones the architect prescribed")
    expect(lambda: A.assert_no_deployable_marking({"deployable": True}), A.S1AError, "a deployable marking is refused")

    # --- gates unchanged ---------------------------------------------------------------------------------------------
    h = json.loads(B.AMENDMENT_REV3S.read_text())["offline_selection_hierarchy_fail_closed"]
    check(h["P_preservation_on_anchors"]["max_abs_max_per_joint"] == 0.25
          and h["T_target_improvement"]["min_relative_improvement_per_joint"] == 0.10
          and h["D_drift"]["parameter_shift_sq_max"] == 0.5 and h["order"] == ["I_integrity", "P_preservation", "T_target_improvement", "D_drift"],
          "the rev3s/rev3v gate thresholds and order are byte-for-byte unchanged")
    check("B.offline_hierarchy" in src, "the unmodified hierarchy tool evaluates the gates")

    # --- diagnostics are non binding ------------------------------------------------------------------------------------
    mor = Z.morphology_diagnostics(T.fresh_init(), corpus, "S0D")
    check(mor["is_a_gate"] is False and "slope_energy_ratio_pred_over_ik" in mor["within_window"]
          and set(mor["fraction_positive_in_negative_windows"]) == {"early_transient_6_14", "stable_holdout_267_290"},
          "morphology diagnostics are non-gating and split the two requested windows")
    check(len(mor["within_window"]["per_window"]) == 7, "all seven windows are reported")

    # --- post-run -----------------------------------------------------------------------------------------------------------
    stage = {"post_run": ran}
    if ran:
        verd = {}
        for c in Z.CANDIDATES:
            rr = json.loads((Z.out_dir_for(c["id"]) / Z.RECEIPT_NAME).read_text())
            passed = rr["binding_offline_hierarchy"]["verdict"] == "PASS"
            check(rr["candidate"]["lambda_h"] == c["lambda_h"] and rr["deployable"] is False, f"{c['id']}: lambda and flag")
            check(rr["offline_state"] == (Z.STATE_PASS if passed else Z.STATE_FAIL)
                  and rr["quarantined"] is (not passed) and rr["may_not_be_source"] is (not passed),
                  f"{c['id']}: the state is DERIVED from the offline verdict")
            check(rr["objective"]["labels_unchanged_sha256"] == before and rr["fit"]["labels_modified"] is False,
                  f"{c['id']}: the receipt proves the labels were unchanged")
            check(rr["objective"]["hinge_rows"] == 64 and rr["objective"]["early_window_excluded_from_hinge_only"] == [6, 14],
                  f"{c['id']}: hinge row selection recorded")
            check(rr["init"]["actor_digest"] == FIT.PIN_S0D_ACTOR, f"{c['id']}: fresh S0D init")
            verd[c["id"]] = rr["binding_offline_hierarchy"]["verdict"]
        expect(lambda: Z.run_fit(authorized_stage=Z.AUTHORIZED_STAGE, progress=False), FileExistsError,
               "a second execution is refused (each candidate fitted exactly once)")
        stage["verdicts"] = verd
    print(json.dumps({"selftest": "PASS", "checks": CHECKS, **stage}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
