"""Self-test rev3t S1B offline fit. Stage-aware: valid before AND after the single execution."""
from __future__ import annotations
import json, pickle, shutil, sys, tempfile
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_s1b_fit as F  # noqa: E402
import v26b_s1b_protocol as B  # noqa: E402
import v26b_s1a_bc as A  # noqa: E402
import v26b_s1_fit as FIT  # noqa: E402
import v26b_s0d_rollout as SR  # noqa: E402
import v26b_r2g as G  # noqa: E402
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
    ran_before = F.out_dir_for("A1").exists()

    # --- lineage + tamper ---------------------------------------------------------------------
    lin = F.verify_lineage_fit()
    check(lin["amendment_rev3t"] == F.PIN_AMENDMENT_REV3T and lin["amendment_rev3s"] == B.PIN_AMENDMENT_REV3S
          and lin["amendment_rev3r"] == B.SA.PIN_AMENDMENT_REV3R, "lineage pins rev3l -> rev3t")
    check(lin["rev3s_tool"] == "9dbbabdd45c2233b416ccb8ccd81a7b84f5bb01b9423d6db9ec5dc78a75cc2be"
          and lin["rev3s_test"] == "4c8cda930e5e1afbb69a9ef1477bab82189701fc8fccfc158f6463b215138160",
          "the rev3s tool and test are byte-identical to the pins: rev3t is additive")
    old = F.PIN_AMENDMENT_REV3T
    try:
        F.PIN_AMENDMENT_REV3T = "0" * 64
        expect(F.verify_lineage_fit, F.S1BFitError, "tampered rev3t pin -> refused")
    finally:
        F.PIN_AMENDMENT_REV3T = old

    # --- tokens: negatives, and the positive proven without fitting -------------------------------
    for bad in (None, "V26B-S1B-ANCHORED-PROTOCOL", "V26B-S1A-BC-FIT", "V26B-S1B-NOMINAL-ROLLOUT",
                "V26B-S1-FIT", "v26b-s1b-fit"):
        e = expect(lambda b=bad: F.run_fit(authorized_stage=b), F.S1BFitError, f"token {bad!r} refused")
        check("V26B-S1B-FIT" in str(e), "guard names the required token")
    real_out = F.out_dir_for
    try:
        F.out_dir_for = lambda cid: A.OUT_S1A          # an existing dir: no-clobber must fire at once
        expect(lambda: F.run_fit(authorized_stage=F.AUTHORIZED_STAGE, progress=False), FileExistsError,
               "the correct token passes the guard and stops at the no-clobber check, writing nothing")
    finally:
        F.out_dir_for = real_out
    e = expect(lambda: F.run_rollout(authorized_stage=F.AUTHORIZED_STAGE), F.S1BFitError, "rollout refused here")
    check("V26B-S1B-NOMINAL-ROLLOUT" in str(e) and "NOT granted" in str(e), "the rollout guard names its own future token")
    expect(lambda: B.run_collection(authorized_stage=F.AUTHORIZED_STAGE), B.S1BError, "collection still refused")

    # --- immutable grid -----------------------------------------------------------------------------
    cands = F.candidates()
    check([c["id"] for c in cands] == ["A1", "A2", "A3", "A4", "A5", "A6"], "exactly the six frozen candidates")
    check([c["anchor_target_ratio_r"] for c in cands] == [3, 5, 8, 12, 20, 8]
          and [c["epochs"] for c in cands] == [300, 300, 300, 300, 300, 60], "frozen ratios and epoch budgets")
    check(F.BATCH == 256 and F.LR == 1e-4 and F.SEED == 2026 and F.CLIP_W == 1.0 and F.ANCHOR_W is FIT.ANCHOR_WEIGHT,
          "frozen numerics reused, anchor weight by reference")

    # --- fresh exact S0D init per candidate ------------------------------------------------------------
    i1 = F.fresh_init(); i2 = F.fresh_init()
    d1 = RF.validate_init_state(i1, expected_actor_digest=FIT.PIN_S0D_ACTOR)["actor_digest"]
    check(d1 == FIT.PIN_S0D_ACTOR and all(np.array_equal(i1[k], i2[k]) for k in i1), "two fresh loads give the identical pinned S0D")
    i1["pi.1.bias"] = np.asarray(i1["pi.1.bias"]) + 1.0
    i3 = F.fresh_init()
    check(np.array_equal(i3["pi.1.bias"], i2["pi.1.bias"]), "a mutated copy cannot contaminate the next candidate's init")
    check(A.P0_ACTOR_DIGEST != FIT.PIN_S0D_ACTOR, "S1A digest differs from S0D: the quarantine check is meaningful")

    # --- loss ratio: batch composition can never alter r ------------------------------------------------
    import torch
    m = torch.tensor([[0.2, 0.1], [0.4, 0.3], [0.6, 0.5], [0.8, 0.7]])
    tgt = torch.tensor([[0.0, 0.0], [0.0, 0.0], [1.0, 1.0], [1.0, 1.0]])
    mask_bal = torch.tensor([True, True, False, False])
    for r in (3.0, 5.0, 8.0, 20.0):
        tot, la, lt = F.two_role_loss(m, tgt, mask_bal, r, lam_clip=0.0)
        exp_a = float(((m[mask_bal] - tgt[mask_bal]) ** 2).mean()); exp_t = float(((m[~mask_bal] - tgt[~mask_bal]) ** 2).mean())
        check(abs(float(la) - exp_a) < 1e-12 and abs(float(lt) - exp_t) < 1e-12
              and abs(float(tot) - (r * exp_a + exp_t)) < 1e-6, f"loss = r*mean_anchor + 1*mean_task exactly (r={r})")
    m2 = torch.cat([m[:2], m[:2], m[2:]]); tgt2 = torch.cat([tgt[:2], tgt[:2], tgt[2:]])
    mask_skew = torch.tensor([True, True, True, True, False, False])
    _, la2, lt2 = F.two_role_loss(m2, tgt2, mask_skew, 8.0, lam_clip=0.0)
    _, la1, lt1 = F.two_role_loss(m, tgt, mask_bal, 8.0, lam_clip=0.0)
    check(abs(float(la2) - float(la1)) < 1e-12 and abs(float(lt2) - float(lt1)) < 1e-12,
          "duplicating the anchor rows in a batch does NOT change either per-role mean: r is invariant to batch composition")
    _, la0, lt0 = F.two_role_loss(m, tgt, torch.zeros(4, dtype=torch.bool), 8.0, lam_clip=0.0)
    check(float(la0) == 0.0 and float(lt0) > 0.0, "an absent role contributes exactly zero")
    src = (HERE / "v26b_s1b_fit.py").read_text()
    check("two_role_loss" in src and "blend" not in src.replace("no_hard_blend", "").replace("blended-label", ""),
          "no hard-blend shortcut in the fit path: genuine two-role gradient descent")
    check("B.offline_hierarchy" in src, "the offline hierarchy is the unmodified rev3s one")

    # --- deterministic repeatability on a micro path (1 epoch; the real fits are NOT doubled) ------------
    corpus = B.build_corpus(); vec, _ = G.scale_vector()
    init = F.fresh_init(); init_s = G.transform_state_to_scaled(init, vec)
    a, ra = F.fit_candidate(init_s, corpus, vec, 8.0, 1)
    b, rb = F.fit_candidate(F.G.transform_state_to_scaled(F.fresh_init(), vec), corpus, vec, 8.0, 1)
    check(all(np.array_equal(a[k], b[k]) for k in a) and ra["optimizer_steps"] == rb["optimizer_steps"] == 3,
          "two micro-fits from a fresh init are bit-identical: deterministic and reproducible")
    check(ra["union_rows"] == 760 and ra["rows_per_role"] == 380 and ra["no_hard_blend"] is True,
          "the union is the two 380-row role blocks")

    # --- save/reload + logstd -------------------------------------------------------------------------------
    tmp = Path(tempfile.mkdtemp(prefix="s1b_saveload_"))
    try:
        if str(R.BASELINE_DIR) not in sys.path: sys.path.insert(0, str(R.BASELINE_DIR))
        import warm_start as W
        exp = G.export_state_from_scaled(a, vec)
        shutil.copy2(SR.S0D_MODULE / "metadata.json", tmp / "metadata.json")
        shutil.copy2(SR.S0D_MODULE / "class_and_ctor_args.pkl", tmp / "class_and_ctor_args.pkl")
        with (tmp / "module_state.pkl").open("wb") as fh:
            pickle.dump({k: np.asarray(v) for k, v in exp.items()}, fh, protocol=pickle.HIGHEST_PROTOCOL)
        back = W.load_module_state(tmp)
        check(W.compare_actor_states({k: np.asarray(v) for k, v in exp.items()}, back).get("exact") is True, "save/reload bit-exact")
        base = F.fresh_init()
        check(np.array_equal(np.asarray(exp["pi.1.weight"])[R.ACTION_DIM:], np.asarray(base["pi.1.weight"])[R.ACTION_DIM:])
              and np.array_equal(np.asarray(exp["pi.1.bias"])[R.ACTION_DIM:], np.asarray(base["pi.1.bias"])[R.ACTION_DIM:]),
              "logstd rows rebuilt bit-identical to the init")
        st = RF.validate_init_state(exp, expected_actor_digest=None)
        check(bool(st["clock_columns_zero"]) and tuple(exp.keys()) == RF.EXPECTED_KEY_ORDER, "clock columns zero, 10 keys")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    # --- output location and no promotion ---------------------------------------------------------------------
    for cid in ("A1", "A6"):
        p = F.out_dir_for(cid)
        check(p.name == f"S1B_{cid}_35D_NONDEPLOYABLE" and p.parent.name == "candidates"
              and "student" not in p.resolve().parts, f"{cid} materialises under candidates/, never student/")
    check(A.MANDATORY_FLAGS["deployable"] is False and A.MANDATORY_FLAGS["rollout_pending"] is True, "non-deployable flags reused from rev3q")
    expect(lambda: A.assert_no_deployable_marking({"deployable": True}), A.S1AError, "a deployable marking would be refused")

    # --- no production / rollout / collection primitives ---------------------------------------------------------
    check(not any(t in src for t in ("subprocess", "rollout_eval", "os.system", "Popen", "--checkpoint", "np.tile")),
          "no closed-loop, collection or tiling primitive in the fit module")

    # --- post-run verification ------------------------------------------------------------------------------------
    stage = {"post_run": F.out_dir_for("A1").exists()}
    if stage["post_run"]:
        verdicts = {}
        for c in F.candidates():
            d = F.out_dir_for(c["id"]); rec = json.loads((d / F.RECEIPT_NAME).read_text(encoding="utf-8"))
            man = json.loads((d / "rl_module" / "actor_feature_manifest.json").read_text(encoding="utf-8"))
            check(rec["candidate"]["id"] == c["id"] and rec["deployable"] is False and rec["rollout_pending"] is True, f"{c['id']}: receipt flags")
            check(man["offline_verdict"] == rec["offline_hierarchy"]["verdict"]
                  and man["quarantined"] == (rec["offline_hierarchy"]["verdict"] != "PASS")
                  and man["may_not_be_source"] == man["quarantined"], f"{c['id']}: manifest verdict/quarantine consistent")
            check(rec["init"]["actor_digest"] == FIT.PIN_S0D_ACTOR and rec["init"]["fresh_reload_per_candidate"] is True, f"{c['id']}: fresh S0D init recorded")
            check(C.sha256_file(d / "rl_module" / "module_state.pkl") == rec["output_files_sha256"]["module_state.pkl"], f"{c['id']}: module digest matches receipt")
            verdicts[c["id"]] = rec["offline_hierarchy"]["verdict"]
        expect(lambda: F.run_fit(authorized_stage=F.AUTHORIZED_STAGE, progress=False), FileExistsError,
               "a second execution is refused (each candidate is fitted exactly once)")
        check(not (SR.F.OUT_S0D.parent / "S1B_A1_35D_NONDEPLOYABLE").exists(), "nothing was written under student/")
        stage["verdicts"] = verdicts
    print(json.dumps({"selftest": "PASS", "checks": CHECKS, **stage}, indent=1))
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
