"""Static + numerical self-test of the rev3x S1C-1 fit. Stage-aware; no rollout, no production change."""
from __future__ import annotations
import json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_s1c1_fit as F  # noqa: E402
import v26b_s1c0_probe as PR  # noqa: E402
import v26b_s1c_protocol as SC  # noqa: E402
import v26b_s1b_protocol as B  # noqa: E402
import v26b_s1b_fit as T  # noqa: E402
import v26b_s1_fit as FIT  # noqa: E402
import v26b_s1a_bc as A  # noqa: E402
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
    ran = F.out_dir_for("W2").exists()

    # --- lineage + addendum ------------------------------------------------------------------------
    lin = F.verify_lineage()
    check(lin["amendment_rev3x"] == F.PIN_AMENDMENT_REV3X and lin["addendum_rev3w_a"] == F.PIN_ADDENDUM_REV3W_A
          and lin["amendment_rev3w"] == PR.PIN_AMENDMENT_REV3W, "lineage pins rev3w, its addendum and rev3x")
    for attr in ("PIN_AMENDMENT_REV3X", "PIN_ADDENDUM_REV3W_A"):
        old = getattr(F, attr)
        try:
            setattr(F, attr, "0" * 64); expect(F.verify_lineage, F.S1C1Error, f"tampered {attr} -> refused")
        finally: setattr(F, attr, old)
    ad = json.loads(F.ADDENDUM_REV3W_A.read_text())
    c1 = ad["correction_1_magnitude_wording"]
    check(abs(1e-3 / 1.038e-4 - 9.634) < 0.01 and "9.634" in c1["correct_statement"] and "NOT two" in c1["correct_statement"],
          "the addendum states the real factor 9.634x (just under one order of magnitude), not two")
    c2 = ad["correction_2_test_digest"]
    check(c2["value_in_the_receipt"] is None
          and c2["final_test_digest"] == "dc8486aae5acf8664f8f4353d203b679cd296cc799fdac15790b3f9e9cc2e912",
          "the addendum records the final test digest and keeps the receipt value null")
    rec = json.loads((PR.OUT_DIR / "v26b_s1c0_probe_20260824_205625.json").read_text())
    check(rec["code_digests"]["test_v26b_s1c0_probe.py"] is None
          and C.sha256_file(PR.OUT_DIR / "v26b_s1c0_probe_20260824_205625.json") == "00e6141f4051117ef2425d144e062fd9cb61708a70e18e5d38b56864f67d8d2c",
          "the original probe receipt is byte-identical and still carries null: it was never rewritten")

    # --- tokens and guards --------------------------------------------------------------------------
    for bad in (None, "V26B-S1C-FEASIBILITY-PROBE", "V26B-S1B-FIT", "v26b-s1c-1"):
        e = expect(lambda b=bad: F.run_fit(authorized_stage=b), F.S1C1Error, f"token {bad!r} refused")
        check("V26B-S1C-1" in str(e), "guard names this stage's token")
    e = expect(lambda: F.run_rollout(authorized_stage=F.AUTHORIZED_STAGE), F.S1C1Error, "rollout refused")
    check("V26B-S1C-NOMINAL-ROLLOUT" in str(e) and "NOT granted" in str(e), "the rollout guard names its future token")
    src = (HERE / "v26b_s1c1_fit.py").read_text()
    check(not any(t in src for t in ("subprocess.run(", "Popen", "rollout_eval", "--checkpoint", "np.tile")),
          "no closed-loop, collection or tiling primitive in the fit module")

    # --- frozen grid and numerics ---------------------------------------------------------------------
    check([c["id"] for c in F.CANDIDATES] == ["W2", "W4", "W8"] and [c["w"] for c in F.CANDIDATES] == [2.0, 4.0, 8.0],
          "exactly the three preregistered candidates w = 2, 4, 8")
    check(F.R_RATIO == 5.0 and F.EPOCHS == 300 and T.BATCH == 256 and T.LR == 1e-4 and T.SEED == 2026
          and F.T.ANCHOR_W is FIT.ANCHOR_WEIGHT, "r = 5, 300 epochs, batch 256, Adam 1e-4, seed 2026, July anchor by reference")
    am = json.loads(F.AMENDMENT_REV3X.read_text())
    check(am["objective"]["r"] == 5 and len(am["candidates"]) == 3
          and am["prohibitions"].count("no fourth attempt") == 1, "the amendment freezes the grid and forbids a fourth attempt")

    # --- push-off definition ----------------------------------------------------------------------------
    corpus = B.build_corpus()
    push = F.push_off_mask(corpus["view"])
    ik_ankle = SC.decode_action(corpus["view"]["u_ik"])[:, 1]
    check(np.array_equal(push, ik_ankle < 0.0), "push-off = rows whose DECODED IK ankle target is negative")
    check(int(push[corpus["view"]["train"]].sum()) == 73, "73 push-off rows among the 380 training rows, as frozen")

    # --- weighted role term: semantics and r-preservation --------------------------------------------------
    import torch
    m = torch.tensor([[0.0, 0.0], [1.0, 1.0], [2.0, 2.0], [3.0, 3.0]])
    y = torch.zeros_like(m)
    mask = torch.tensor([True, True, True, True])
    ones = torch.ones(4)
    check(abs(float(F.weighted_role_term(m, y, mask, ones)) - float(((m - y) ** 2).mean())) < 1e-12,
          "with unit weights the weighted term reduces EXACTLY to the plain per-role mean")
    w = torch.tensor([1.0, 1.0, 4.0, 4.0])
    se = ((m - y) ** 2).mean(dim=1)
    check(abs(float(F.weighted_role_term(m, y, mask, w)) - float((w * se).sum() / w.sum())) < 1e-12,
          "the weighted term equals sum(w*mse)/sum(w) in closed form")
    half = torch.tensor([True, True, False, False])
    check(abs(float(F.weighted_role_term(m, y, half, ones)) - float(((m[half]) ** 2).mean())) < 1e-12,
          "only the role's own rows enter the term")
    check(float(F.weighted_role_term(m, y, torch.zeros(4, dtype=torch.bool), ones)) == 0.0,
          "an absent role contributes exactly zero")
    big = torch.tensor([1.0, 1.0, 100.0, 100.0])
    check(float(F.weighted_role_term(m, y, mask, big)) > float(F.weighted_role_term(m, y, mask, w)),
          "raising w shifts the term towards the weighted rows (the intended localisation)")
    check("torch.ones_like(w_t[idx])" in src, "the ANCHOR role is always unit-weighted: the weighting cannot alter r")

    # --- run-length metric (preregistered) ------------------------------------------------------------------
    check(F.run_lengths([0, 1, 1, 0, 1, 1, 1, 0]) == [2, 3] and F.run_lengths([1, 1]) == [2]
          and F.run_lengths([0, 0]) == [] and F.run_lengths([1, 0, 1]) == [1, 1],
          "run lengths counted as maximal runs of consecutive negatives, tail run included")

    # --- diagnostics are declared non-binding ---------------------------------------------------------------
    s0d = T.fresh_init()
    per = F.persistence_diagnostic(s0d, corpus, "S0D")
    check(per["is_a_gate"] is False and per["B3_verdict"].startswith("INDETERMINATE")
          and "OPEN-LOOP COUNTERFACTUAL" in per["limitation"],
          "the persistence diagnostic is not a gate, declares B3 INDETERMINATE and states its counterfactual nature")
    check(0.0 <= per["duty_cycle_negative_command"] <= 1.0 and per["rows"] == 100,
          "the diagnostic runs on the 100 ordered held-out rows")
    import f2r_labeller as L
    corpus["healthy"] = np.asarray(L.load_cache(R.OUT_CACHE, "nominal").targets)[:, [0, 2]]
    sur = F.commanded_surrogates(s0d, corpus, "S0D")
    check("SURROGATE ONLY" in sur["status"] and "never promotes" in sur["status"],
          "the G2-G4 surrogates are labelled non-binding and non-promoting")

    # --- init exclusivity and output location -----------------------------------------------------------------
    d = RF.validate_init_state(T.fresh_init(), expected_actor_digest=FIT.PIN_S0D_ACTOR)["actor_digest"]
    check(d == FIT.PIN_S0D_ACTOR and d != A.P0_ACTOR_DIGEST and d != SB.PIN_A2_ACTOR,
          "the init is the pinned S0D and differs from S1A and from A2")
    for cid in ("W2", "W8"):
        p = F.out_dir_for(cid)
        check(p.name == f"S1C1_{cid}_35D_NONDEPLOYABLE" and p.parent.name == "candidates"
              and "student" not in p.resolve().parts, f"{cid} materialises under candidates/, never student/")

    # --- post-run ----------------------------------------------------------------------------------------------
    stage = {"post_run": ran}
    if ran:
        verd = {}
        for c in F.CANDIDATES:
            rr = json.loads((F.out_dir_for(c["id"]) / F.RECEIPT_NAME).read_text())
            check(rr["candidate"]["w"] == c["w"] and rr["deployable"] is False and rr["rollout_pending"] is True,
                  f"{c['id']}: receipt flags and weight")
            check(rr["init"]["actor_digest"] == FIT.PIN_S0D_ACTOR and rr["fit"]["push_off_rows"] == 73,
                  f"{c['id']}: fresh S0D init and frozen push-off count")
            check(rr["preregistered_causal_diagnostic"]["candidate"]["is_a_gate"] is False,
                  f"{c['id']}: the diagnostic is recorded as non-gating")
            verd[c["id"]] = rr["binding_offline_hierarchy"]["verdict"]
        expect(lambda: F.run_fit(authorized_stage=F.AUTHORIZED_STAGE, progress=False), FileExistsError,
               "a second execution is refused (each candidate fitted exactly once)")
        stage["verdicts"] = verd
    print(json.dumps({"selftest": "PASS", "checks": CHECKS, **stage}, indent=1))
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
