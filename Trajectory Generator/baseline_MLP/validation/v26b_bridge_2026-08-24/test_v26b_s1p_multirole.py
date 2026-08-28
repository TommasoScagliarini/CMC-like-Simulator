"""Self-test rev3p S1P multi-role fit. Stage-aware: valid before AND after the fit run."""
from __future__ import annotations
import json, pickle, shutil, sys, tempfile
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_s1p_multirole as P  # noqa: E402
import v26b_s1_fit as FIT  # noqa: E402
import v26b_s1_prereg as S1  # noqa: E402
import v26b_s0d_fit as F0  # noqa: E402
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
    published_before = P.OUT_S1P.exists()

    # --- lineage / immutability ---------------------------------------------------------------
    lin = P.verify_lineage_s1p()
    check(lin["amendment_rev3p"] == P.PIN_AMENDMENT_REV3P and lin["amendment_rev3o"] == FIT.PIN_AMENDMENT_REV3O
          and lin["amendment_rev3n"] == FIT.N.PIN_AMENDMENT_REV3N and lin["amendment_rev3l"] == S1.PIN_AMENDMENT_REV3L
          and lin["amendment_rev3m"] == S1.PIN_AMENDMENT_REV3M, "lineage pins rev3l/m/n/o/p")
    check(lin["rev3o_rejected_artifact"] == "a559bd691eb5934f32a820e46e13660f010f0b54e445c82d4f13e485154d558a"
          and lin["rev3o_tool"] == "a6d7163ef8089ca9efb5b8b31fe92d2132efa357b1c69d814f6d9531ea845916"
          and lin["rev3o_no_checkpoint_published"] is True, "rev3o REJECTED evidence and tool immutable; rev3o published nothing")
    for attr in ("PIN_AMENDMENT_REV3P", "PIN_REV3O_REJECTED", "PIN_REV3O_TOOL"):
        old = getattr(P, attr)
        try:
            setattr(P, attr, "0" * 64); expect(P.verify_lineage_s1p, P.S1PError, f"tampered {attr} -> refused")
        finally: setattr(P, attr, old)

    # --- token guards (negative) ----------------------------------------------------------------
    for bad in (None, "V26B-S1-FIT", "V26B-S1-REV3N-PREREG-MEDIAN", "V26B-S0D-FIT", "v26b-s1p-multirole-fit"):
        e = expect(lambda b=bad: P.run_s1p_fit(authorized_stage=b), P.S1PError, f"token {bad!r} refused")
        check("V26B-S1P-MULTIROLE-FIT" in str(e), "guard names the required token")

    # --- init: exclusively the pinned S0D --------------------------------------------------------
    if str(R.BASELINE_DIR) not in sys.path: sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    init_raw = {k: np.asarray(W._as_numpy(v)) for k, v in W.load_module_state(FIT.S0D_MODULE).items()}
    st0 = RF.validate_init_state(init_raw, expected_actor_digest=FIT.PIN_S0D_ACTOR)
    check(st0["actor_digest"] == "481dd0d22919fc1ec04cdb722409b9711caeb61d57449c210aad7386375b764a", "init = pinned S0D actor")
    v1_raw = {k: np.asarray(W._as_numpy(v)) for k, v in W.load_module_state(F0.V1_MODULE).items()}
    expect(lambda: RF.validate_init_state(v1_raw, expected_actor_digest=FIT.PIN_S0D_ACTOR), Exception,
           "a different chain actor is refused against the S0D pin")

    # --- roles: leakage and collisions ------------------------------------------------------------
    roles = P.build_roles(init_raw)
    rec = roles["records"]; view = roles["view"]; step = np.arange(1, 501)
    check(rec["task"]["rows"] == 380 and rec["preservation_train"]["rows"] == 14834 and rec["union_rows"] == 15214, "role sizes 380 + 14834 = 15214")
    check(not np.any((step[view["train"]] >= 191) & (step[view["train"]] <= 310)), "no task row inside steps 191-310")
    lk = rec["leakage_checks_fail_closed"]
    check(lk["pres_vs_500_task_states"] == 0 and lk["pres_vs_rev3m_source_holdout"] == 0, "zero bitwise overlap: preservation vs task states AND vs the rev3m gate set (recorded)")
    task_keys = {view["obs"][i].tobytes() for i in range(500)}
    pres_keys = {roles["pres_obs"][i].tobytes() for i in range(roles["pres_obs"].shape[0])}
    gate_keys = {roles["src"]["obs"][i].tobytes() for i in range(roles["src"]["obs"].shape[0])}
    check(len(pres_keys) == 14834 and not (pres_keys & task_keys) and not (pres_keys & gate_keys) and not (gate_keys & task_keys),
          "the three row sets are pairwise bitwise-disjoint -> no label collision is possible")
    check(np.array_equal(roles["pres_labels"], np.asarray(RF.numpy_mean(init_raw, roles["pres_obs"]), np.float32))
          and roles["pres_labels"].shape == (14834, R.ACTION_DIM), "preservation labels = S0D deterministic mean, reproducible")
    check(roles["src"]["records"]["usable_rows"] == 4474 and roles["src"]["records"]["never_in_training"] is True, "rev3m gate set 4474 rows, never trained")
    # a deliberately poisoned preservation set must be refused
    import v26b_s0d as S0
    real_split = S0.build_split
    try:
        def poisoned():
            s = real_split(); s = dict(s)
            obs = np.asarray(s["obs"]).copy(); obs[s["assign"]["train_idx"][0]] = view["obs"][0]
            s["obs"] = obs; return s
        S0.build_split = poisoned
        e = expect(lambda: P.build_roles(init_raw), P.S1PError, "a preservation row colliding with a task state is refused")
        check("LEAKAGE" in str(e), "the refusal is explicitly a leakage refusal")
    finally:
        S0.build_split = real_split

    # --- role-weight semantics: group-balanced, beta frozen ---------------------------------------
    import torch
    check(P.BETA == 1.0 and P.ANCHOR_WEIGHT is FIT.ANCHOR_WEIGHT and P.ANCHOR_WEIGHT == 1e-5, "beta 1.0 frozen; anchor 1e-5 by reference")
    check(P.BUDGET == {"epochs": 300, "batch_size": 256, "lr": 1e-4, "seed": 2026, "clip_weight": 1.0}, "numerics unchanged 300/256/1e-4/2026/clip 1.0")
    check(P.G_TASK_MAX is FIT.G_TASK_MAX and P.G_TASK_MAX == 0.15 and P.Q1_MAX is FIT.Q1_MAX and P.Q1_MAX == 0.10, "gate thresholds reused by reference")
    m = torch.tensor([[0.0, 0.0], [1.0, 1.0], [2.0, 2.0], [3.0, 3.0]])
    y = torch.zeros_like(m)
    mk = torch.tensor([True, True, False, False])
    check(float(P.role_term(m, y, mk)) == float(((m[mk] - y[mk]) ** 2).mean()) == 0.5, "role term = flat MSE over the role rows only")
    check(float(P.role_term(m, y, ~mk)) == 6.5, "the other role is computed on its own rows only")
    check(float(P.role_term(m, y, torch.zeros(4, dtype=torch.bool))) == 0.0
          and P.role_term(m, y, torch.zeros(4, dtype=torch.bool)).dtype == m.dtype, "empty role contributes exactly zero")
    m2 = torch.cat([m, m[2:], m[2:]]); y2 = torch.zeros_like(m2)
    mk2 = torch.tensor([True, True] + [False] * 6)
    check(float(P.role_term(m2, y2, ~mk2)) == 6.5 == float(P.role_term(m, y, ~mk)), "group-balanced: tripling the rows of a role does NOT change its per-role mean")
    check(float(P.role_term(m, y, mk) + P.BETA * P.role_term(m, y, ~mk)) == 0.5 + 6.5, "L_data = task term + beta * preservation term")

    # --- anchor identity (6 FULL tensors, explicit 0.5 head factors) -------------------------------
    g = torch.Generator().manual_seed(11)
    Pp = [torch.nn.Parameter(torch.rand(s, generator=g)) for s in [(4, 3), (4,), (5, 4), (5,), (2, 5), (2,)]]
    A = [torch.rand(p.shape, generator=g) for p in Pp]
    manual = torch.stack([(Pp[0] - A[0]).square().mean(), (Pp[1] - A[1]).square().mean(), (Pp[2] - A[2]).square().mean(),
                          (Pp[3] - A[3]).square().mean(), 0.5 * (Pp[4] - A[4]).square().mean(), 0.5 * (Pp[5] - A[5]).square().mean()]).mean()
    check(torch.allclose(I.anchor_loss_july_exact(Pp, A), manual, atol=0, rtol=0), "anchor = mean of the 6 FULL tensor terms with 0.5 head factors")

    # --- gate + publication semantics (synthetic) --------------------------------------------------
    ok = FIT.gate_decision([0.14, 0.10], [0.09, 0.05], True, 0.0, 0.0)
    check(P.publication_decision(ok)["publish"] is True and P.publication_decision(ok)["failed_binding_gates"] == [], "publish only when every binding gate passes")
    for bad, name in ((FIT.gate_decision([0.1501, 0.10], [0.09, 0.05], True, 0.0, 0.0), "G_task"),
                      (FIT.gate_decision([0.14, 0.10], [0.1001, 0.05], True, 0.0, 0.0), "Q1_source_holdout_rev3m"),
                      (FIT.gate_decision([0.14, 0.10], [0.09, 0.05], False, 0.0, 0.0), "Q3_invariants"),
                      (FIT.gate_decision([0.14, 0.10], [0.09, 0.05], True, 0.0, 2e-5), "function_preservation")):
        d = P.publication_decision(bad)
        check(d["publish"] is False and d["failed_binding_gates"] == [name], f"{name} FAIL -> no publication")

    # --- no-publish-on-fail, end to end (stub fit = the untouched init) ----------------------------
    tmp = Path(tempfile.mkdtemp(prefix="s1p_nopub_"))
    try:
        def stub_fit(init_scaled, roles_, vec, progress=False):
            f32 = np.float32
            sf = {k: np.array(init_scaled[k], dtype=f32) for k in ("pi.0.0.weight", "pi.0.0.bias", "pi.0.2.weight", "pi.0.2.bias", "pi.1.weight", "pi.1.bias")}
            return sf, {"tool": "stub(no-op)", "history": [{"epoch": 1, "loss": 0.0}]}
        e = expect(lambda: P.run_s1p_fit(authorized_stage=P.AUTHORIZED_STAGE, out_dir=tmp / "S1P_never",
                                         reject_dir=tmp, progress=False, _fit_fn=stub_fit),
                   P.S1PError, "gates fail on the untouched init -> refusal")
        check("binding gates FAILED" in str(e) and "G_task" in str(e), "the refusal names the failing binding gate")
        rej = list(tmp.glob("v26b_s1p_fit_REJECTED_*.json"))
        check(len(rej) == 1, "exactly one REJECTED artifact written")
        rj = json.loads(rej[0].read_text())
        check(rj["REJECTED"] is True and rj["publication_decision"]["publish"] is False
              and rj["gates"]["G_task"]["pass"] is False and rj["roles"]["leakage_checks_fail_closed"]["pres_vs_500_task_states"] == 0,
              "REJECTED artifact carries gates, decision and leakage record")
        check(not (tmp / "S1P_never").exists(), "NO candidate checkpoint directory on failure")
        check(not any(p.name.startswith(".staging") or p.name.endswith(".lock") for p in tmp.iterdir()), "no staging/lock residue on failure")
        check(P.OUT_S1P.exists() == published_before, "the real S1P path was never touched by the failure path")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    # --- T1 / Q3 / save-reload ---------------------------------------------------------------------
    vec, _ = G.scale_vector()
    init_scaled = G.transform_state_to_scaled(init_raw, vec)
    t1 = G.preservation_test(init_raw, init_scaled, view["obs"], vec)
    check(t1 <= G.PRESERVATION_TOL, f"prefit T1 {t1:.3e} <= 1e-5")
    inv = RF.invariance_test(init_raw, view["obs"].astype(np.float32)[:64])
    check(bool(inv["bit_identical"]) and bool(st0["clock_columns_zero"]) and bool(st0["sigma_head"]["logstd_bias_exact"])
          and tuple(init_raw.keys()) == RF.EXPECTED_KEY_ORDER, "Q3 on the init: 10 keys, clock zero + bit-identical invariance, logstd placeholder")
    tmp2 = Path(tempfile.mkdtemp(prefix="s1p_saveload_"))
    try:
        shutil.copy2(FIT.S0D_MODULE / "metadata.json", tmp2 / "metadata.json")
        shutil.copy2(FIT.S0D_MODULE / "class_and_ctor_args.pkl", tmp2 / "class_and_ctor_args.pkl")
        with (tmp2 / "module_state.pkl").open("wb") as fh:
            pickle.dump({k: np.asarray(v) for k, v in init_raw.items()}, fh, protocol=pickle.HIGHEST_PROTOCOL)
        check(W.compare_actor_states({k: np.asarray(v) for k, v in init_raw.items()}, W.load_module_state(tmp2)).get("exact") is True,
              "save/reload round trip bit-exact")
    finally:
        shutil.rmtree(tmp2, ignore_errors=True)

    # --- sigma stays unresolved ---------------------------------------------------------------------
    check("UNDECIDED" in G.SIGMA_NOTE and "placeholder" in G.SIGMA_NOTE, "sigma note declares 0.005 undecided / serialisation placeholder")
    check(not any("sigma" in k.lower() for k in ok.keys()), "no gate reads sigma")

    # --- offline only --------------------------------------------------------------------------------
    srctxt = (HERE / "v26b_s1p_multirole.py").read_text()
    check(not any(tok in srctxt for tok in ("subprocess", "rollout_eval", "os.system", "Popen", "--checkpoint")),
          "the fit module contains no closed-loop execution primitive")

    # --- post-run verification -----------------------------------------------------------------------
    stage = {"post_run": P.OUT_S1P.exists()}
    if stage["post_run"]:
        r = json.loads((P.OUT_S1P / P.RECEIPT_NAME).read_text(encoding="utf-8"))
        pub = {k: np.asarray(W._as_numpy(v)) for k, v in W.load_module_state(P.OUT_S1P / "rl_module").items()}
        stp = RF.validate_init_state(pub, expected_actor_digest=r["gates"]["actor_digest_new"])
        check(r["gates"]["pass_all"] is True and r["publication_decision"]["publish"] is True, "receipt records a full PASS")
        check(stp["actor_digest"] == r["gates"]["actor_digest_new"] and r["init"]["actor_digest"] == FIT.PIN_S0D_ACTOR, "published module matches the receipt; init pinned to S0D")
        check(C.sha256_file(P.OUT_S1P / "rl_module" / "module_state.pkl") == r["output_files_sha256"]["module_state.pkl"], "published module_state matches the receipt digest")
        check(not any(p.name.startswith(".staging") or p.name.endswith(".lock") for p in P.OUT_S1P.parent.iterdir()), "no orphan staging/lock next to the export")
        check(FIT.OUT_S1.exists() is False, "rev3o path still empty (never overwritten)")
        stage["published_actor_digest"] = stp["actor_digest"]
    print(json.dumps({"selftest": "PASS", "checks": CHECKS, "prefit_T1": t1, **stage}, indent=1))
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
