"""Self-test S0D fit (before execution): lineage/tamper, semantics, budget, guards."""
from __future__ import annotations
import json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_s0d_fit as F  # noqa: E402
import v26b_s0d as S  # noqa: E402
import v26b_r2g as G  # noqa: E402
import v26b_r2i as I  # noqa: E402
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
    lin = F.verify_lineage_fit()
    check(lin["pregate_tool_sha256"] == F.PIN_PREGATE_TOOL and lin["pregate_receipt_sha256"] == F.PIN_PREGATE_RECEIPT, "lineage: rev3k + pre-gate tool b9c2f275 + PASS receipt ee2379d1 + V1")
    for attr in ("PIN_PREGATE_TOOL", "PIN_PREGATE_RECEIPT"):
        old = getattr(F, attr)
        try:
            setattr(F, attr, "0"*64); expect(F.verify_lineage_fit, F.S0DFitError, f"tampered {attr} -> refused")
        finally: setattr(F, attr, old)
    # token guard
    e = expect(lambda: F.run_s0d_fit(authorized_stage=None), F.S0DFitError, "no token -> refused")
    check("V26B-S0D-FIT" in str(e), "guard names the token")
    expect(lambda: F.run_s0d_fit(authorized_stage="V26B-S0D-PREGATE"), F.S0DFitError, "wrong token refused")
    # budget constants: 300 fixed, no early stop
    check(F.BUDGET == {"epochs": 300, "batch_size": 256, "lr": 1e-4, "seed": 2026, "clip_weight": 1.0}, "budget == rev3k (300 fixed/b256/lr1e-4/seed2026/clip1.0)")
    check(F.ANCHOR_WEIGHT_JULY_1107 == 1e-5 and F.POSTFIT_RMSE_MAX == 0.10, "anchor 1e-5 (July 11/07) + postfit 0.10 (Q1)")
    # anchor semantics: F uses I.anchor_loss_july_exact (already value+gradient-proven vs FULL reference in test_v26b_r2i)
    import torch
    mp = [torch.nn.Parameter(torch.ones(4, 35)), torch.nn.Parameter(torch.ones(4)), torch.nn.Parameter(torch.ones(4, 4)), torch.nn.Parameter(torch.ones(4)), torch.nn.Parameter(torch.ones(2, 4)), torch.nn.Parameter(torch.ones(2))]
    with torch.no_grad():
        val = float(I.anchor_loss_july_exact(mp, [torch.zeros_like(p) for p in mp]))
    check(abs(val - 5.0/6.0) < 1e-6, "anchor formula = mean([1,1,1,1,.5,.5]) (July semantics, 0.5 head factors)")
    # real init V1 + T1 + label diagnostics + tiny fit semantics
    split = S.build_split()
    diag = F.train_label_diagnostics(split)
    check(diag["rows_beyond_abs1"] == 490 and abs(diag["clip_rmse_train"][0] - 0.02392823) < 1e-6, "Codex diagnostics reproduced (490 rows; clip-RMSE 0.0239/0.0104)")
    check("RAW" in diag["note"] and "DISTINCT" in diag["note"], "raw-label + runtime-clip distinction recorded")
    bad = {**split}
    if str(R.BASELINE_DIR) not in sys.path: sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    init_raw = {k: np.asarray(W._as_numpy(v)) for k, v in W.load_module_state(F.V1_MODULE).items()}
    check(RF.actor_state_digest(init_raw) == F.PIN_V1_ACTOR_DIGEST, "init V1 digest verified")
    vec, _ = G.scale_vector()
    init_scaled = G.transform_state_to_scaled(init_raw, vec)
    t1 = G.preservation_test(init_raw, init_scaled, split["obs"], vec)
    check(t1 <= 1e-5, f"T1 on all 19314 rows: {t1:.2e} <= 1e-5")
    tiny = {"epochs": 2, "batch_size": 256, "lr": 1e-4, "seed": 2026, "clip_weight": 1.0}
    st, rep = F.fit_s0d(init_scaled, split, vec, budget=tiny)
    check(rep["history"][-1]["loss"] <= rep["history"][0]["loss"], "tiny fit: loss non-increasing")
    check(rep["anchor"]["weight"] == 1e-5 and "no aux" in rep["loss"] and "row-uniform" in rep["loss"], "fit report: anchor 1e-5, no aux, row-uniform flat MSE")
    expect(lambda: F.fit_s0d(init_scaled, split, vec, budget={**tiny, "patience": 60}), F.S0DFitError, "early-stopping budget refused")
    exp_state = G.export_state_from_scaled(st, vec)
    struct = RF.validate_init_state(exp_state, expected_actor_digest=None)
    check(struct["clock_columns_zero"] and struct["sigma_head"]["logstd_bias_exact"], "Q3 structural on export state (clock zero, logstd placeholder)")
    m_s = RF.numpy_mean(st, (split["obs"].astype(np.float64) / vec[None, :]).astype(np.float32)[:256])
    m_e = RF.numpy_mean(exp_state, split["obs"].astype(np.float32)[:256])
    check(float(np.max(np.abs(m_s - m_e))) <= 1e-5, "T2 export equivalence on tiny-fit state")
    # postfit gate machinery: provenance masks (reuse) + no-clobber
    post = F.postfit_gate(init_raw, exp_state, st, split, vec, t1)
    ho = split["assign"]["hold_idx"]; prov_h = [split["assign"]["provenance"][i] for i in ho.tolist()]
    held = split["records"]["held_jobs_per_cell"]
    job = held["nominal|0.005"]
    check(post["holdout_binding_provenance_masks"]["cell_nominal_0.005"]["rows"] == sum(1 for p in prov_h if job in p), "postfit cell mask == provenance membership (corrected rule)")
    check(post["train_metrics_distinct"]["rows"] == 14834, "train metrics separate (14834)")
    # stage-aware (Codex final correction): pre-run -> precondition; post-run -> exported artefact integrity
    if not F.OUT_S0D.exists():
        check(True, "pre-run: no export dir (no-clobber ready)")
    else:
        rec_p = F.OUT_S0D / F.RECEIPT_NAME
        check(rec_p.is_file() and C.sha256_file(rec_p) == "1b2dbab43bd945ca8afb1c9651c8de969e30813a539c99623e06ff116ed9e859", "post-run: completion marker present, receipt sha == 1b2dbab4...")
        check(C.sha256_file(F.OUT_S0D / "rl_module" / "module_state.pkl") == "cda6d893138444908b4fcc908dc0045bd0df317ef0f6bbc72f70e84629e14597", "post-run: module_state hash == cda6d893...")
        rr = json.loads(rec_p.read_text())
        check(rr["postfit_gates"]["actor_digest_new"] == "481dd0d22919fc1ec04cdb722409b9711caeb61d57449c210aad7386375b764a", "post-run: actor digest == 481dd0d2...")
        parent = F.OUT_S0D.parent
        check(not any(q.name.startswith(".staging-") for q in parent.iterdir()) and not (parent / f".{F.OUT_S0D.name}.lock").exists(), "post-run: no orphan staging/lock")
    check(C.sha256_file(F.HERE / "v26b_s0d.py") == F.PIN_PREGATE_TOOL, "pre-gate tool untouched")
    print(f"SELFTEST PASS ({CHECKS} checks)")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
