"""Self-test S1 IK-AB06 fit (rev3o). Stage-aware: valid before AND after the fit run."""
from __future__ import annotations
import json, pickle, shutil, sys, tempfile
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
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
    post_run = (FIT.OUT_S1 / "rl_module").is_dir()

    # --- lineage / pins ---------------------------------------------------------------------
    lin = FIT.verify_lineage_s1_fit()
    check(lin["amendment_rev3o"] == FIT.PIN_AMENDMENT_REV3O and lin["amendment_rev3n"] == FIT.N.PIN_AMENDMENT_REV3N
          and lin["amendment_rev3l"] == S1.PIN_AMENDMENT_REV3L and lin["amendment_rev3m"] == S1.PIN_AMENDMENT_REV3M,
          "lineage pins rev3l/rev3m/rev3n/rev3o")
    check(lin["rev3n_pregate_receipt"] == FIT.PIN_REV3N_RECEIPT == "923ffc5ca8fb4d3577d9894e90db151a92bad089d3669b38fb9553484e6f2634"
          and lin["rev3n_pregate_median_per_joint"] == [0.1294335601758372, 0.0917108885270355], "rev3n PASS receipt pinned and read")
    check(lin["init_module_state"] == FIT.PIN_S0D_MODULE_STATE == "cda6d893138444908b4fcc908dc0045bd0df317ef0f6bbc72f70e84629e14597", "S0D module_state pinned")
    for attr in ("PIN_AMENDMENT_REV3O", "PIN_REV3N_RECEIPT", "PIN_S0D_MODULE_STATE"):
        old = getattr(FIT, attr)
        try:
            setattr(FIT, attr, "0" * 64); expect(FIT.verify_lineage_s1_fit, FIT.S1FitError, f"tampered {attr} -> refused")
        finally: setattr(FIT, attr, old)

    # --- init: exclusively S0D --------------------------------------------------------------
    if str(R.BASELINE_DIR) not in sys.path: sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    init_raw = {k: np.asarray(W._as_numpy(v)) for k, v in W.load_module_state(FIT.S0D_MODULE).items()}
    st0 = RF.validate_init_state(init_raw, expected_actor_digest=FIT.PIN_S0D_ACTOR)
    check(st0["actor_digest"] == FIT.PIN_S0D_ACTOR == "481dd0d22919fc1ec04cdb722409b9711caeb61d57449c210aad7386375b764a", "init = pinned S0D actor")
    v1_raw = {k: np.asarray(W._as_numpy(v)) for k, v in W.load_module_state(F0.V1_MODULE).items()}
    expect(lambda: RF.validate_init_state(v1_raw, expected_actor_digest=FIT.PIN_S0D_ACTOR), Exception,
           "a different chain actor (V1) is refused against the S0D pin -> no JUL_H0/R0a/R1/R2G/R2I init")
    check(FIT.FORBIDDEN_INIT_NAMES == ("JUL_H0", "R0a", "R1", "R2G", "R2I"), "forbidden inits declared")

    # --- split / leakage --------------------------------------------------------------------
    view = FIT.build_s1_task()
    step = np.arange(1, 501); tr, ho = view["train"], view["hold"]
    check(int(tr.sum()) == 380 and int(ho.sum()) == 100 and int(view["embargo"].sum()) == 20, "380 train / 100 holdout / 20 embargo")
    check(not np.any((step[tr] >= 191) & (step[tr] <= 310)), "NO training row inside steps 191-310 (holdout + embargo)")
    check(np.array_equal(step[ho], np.arange(201, 301)), "holdout is exactly steps 201-300")
    check(sorted(set(step[tr].tolist())) == list(range(1, 191)) + list(range(311, 501)), "train = steps 1-190 and 311-500")
    check(not np.any(tr & ho) and not np.any(tr & view["embargo"]), "train disjoint from holdout and embargo")
    for k in ("train_obs_sha256", "train_labels_sha256", "holdout_obs_sha256"):
        check(len(view["records"][k]) == 64, f"{k} recorded")

    # --- source holdout rev3m: separate, never trained on ------------------------------------
    src = FIT.build_source_holdout(view, init_raw)
    check(src["records"]["rows_before_exclusion"] == 4480 and src["records"]["bitwise_shared_excluded"] == 6
          and src["records"]["usable_rows"] == 4474 and src["obs"].shape[0] == 4474, "source holdout 4480 - 6 shared = 4474")
    task_keys = {view["obs"][i].tobytes() for i in range(500)}
    check(not any(src["obs"][i].tobytes() in task_keys for i in range(src["obs"].shape[0])), "source holdout bitwise disjoint from ALL 500 task rows (hence from the 380 train rows)")
    check(src["labels"].shape == (4474, R.ACTION_DIM) and np.array_equal(src["labels"], RF.numpy_mean(init_raw, src["obs"])), "labels = S0D deterministic mean, same state, reproducible")
    check(src["records"]["never_in_training"] is True, "source holdout declared out of training")

    # --- anchor: mean over the 6 FULL tensors, 0.5 head factors ------------------------------
    import torch
    g = torch.Generator().manual_seed(7)
    P = [torch.nn.Parameter(torch.rand(s, generator=g)) for s in [(4, 3), (4,), (5, 4), (5,), (2, 5), (2,)]]
    A = [torch.rand(p.shape, generator=g) for p in P]
    manual = torch.stack([(P[0] - A[0]).square().mean(), (P[1] - A[1]).square().mean(), (P[2] - A[2]).square().mean(),
                          (P[3] - A[3]).square().mean(), 0.5 * (P[4] - A[4]).square().mean(), 0.5 * (P[5] - A[5]).square().mean()]).mean()
    check(torch.allclose(I.anchor_loss_july_exact(P, A), manual, atol=0, rtol=0), "anchor = mean of the 6 FULL tensor terms with explicit 0.5 head factors")
    check(FIT.ANCHOR_WEIGHT is F0.ANCHOR_WEIGHT_JULY_1107 and FIT.ANCHOR_WEIGHT == 1e-5, "anchor weight = July 11/07 1e-5, reused by reference")
    check(FIT.BUDGET == {"epochs": 300, "batch_size": 256, "lr": 1e-4, "seed": 2026, "clip_weight": 1.0}, "frozen numerics 300/256/1e-4/2026/clip 1.0")

    # --- token guards -----------------------------------------------------------------------
    for bad in (None, "V26B-S1-REV3N-PREREG-MEDIAN", "V26B-S0D-FIT", "v26b-s1-fit"):
        e = expect(lambda b=bad: FIT.run_s1_fit(authorized_stage=b), FIT.S1FitError, f"token {bad!r} refused")
        check("V26B-S1-FIT" in str(e), "guard names the required token")

    # --- offline only: no closed-loop execution primitives -----------------------------------
    srctxt = (HERE / "v26b_s1_fit.py").read_text()
    check(not any(tok in srctxt for tok in ("subprocess", "rollout_eval", "os.system", "Popen", "--checkpoint")),
          "the fit module contains no closed-loop execution primitive")

    # --- gate semantics (synthetic; thresholds are references, never redefined) --------------
    check(FIT.G_TASK_MAX is S1.PRE_GATE_RMSE_MAX and FIT.G_TASK_MAX == 0.15 and FIT.Q1_MAX is F0.POSTFIT_RMSE_MAX and FIT.Q1_MAX == 0.10,
          "G_task 0.15 and Q1 0.10 reused by reference")
    ok = FIT.gate_decision([0.14, 0.10], [0.09, 0.05], True, 0.0, 0.0)
    check(ok["pass_all"] and ok["G_task"]["binding"] and ok["Q1_source_holdout_rev3m"]["binding"], "all-pass case")
    check(FIT.gate_decision([0.1501, 0.10], [0.09, 0.05], True, 0.0, 0.0)["pass_all"] is False, "G_task > 0.15 -> FAIL")
    check(FIT.gate_decision([0.14, 0.10], [0.09, 0.1001], True, 0.0, 0.0)["pass_all"] is False, "Q1 > 0.10 -> FAIL")
    check(FIT.gate_decision([0.14, 0.10], [0.09, 0.05], False, 0.0, 0.0)["pass_all"] is False, "Q3 invariants FAIL -> FAIL")
    check(FIT.gate_decision([0.14, 0.10], [0.09, 0.05], True, 0.0, 2e-5)["pass_all"] is False, "T2 > 1e-5 -> FAIL")

    # --- T1 / Q3 / save-reload mechanics -----------------------------------------------------
    vec, _ = G.scale_vector()
    init_scaled = G.transform_state_to_scaled(init_raw, vec)
    t1 = G.preservation_test(init_raw, init_scaled, view["obs"], vec)
    check(t1 <= G.PRESERVATION_TOL, f"prefit T1 {t1:.3e} <= 1e-5")
    inv = RF.invariance_test(init_raw, view["obs"].astype(np.float32)[:64])
    check(bool(inv["bit_identical"]) and bool(st0["clock_columns_zero"]) and bool(st0["sigma_head"]["logstd_bias_exact"])
          and tuple(init_raw.keys()) == RF.EXPECTED_KEY_ORDER, "Q3 on the init: 10 keys, clock zero + bit-identical invariance, logstd placeholder")
    tmp = Path(tempfile.mkdtemp(prefix="s1_saveload_"))
    try:
        shutil.copy2(FIT.S0D_MODULE / "metadata.json", tmp / "metadata.json")
        shutil.copy2(FIT.S0D_MODULE / "class_and_ctor_args.pkl", tmp / "class_and_ctor_args.pkl")
        with (tmp / "module_state.pkl").open("wb") as fh:
            pickle.dump({k: np.asarray(v) for k, v in init_raw.items()}, fh, protocol=pickle.HIGHEST_PROTOCOL)
        back = W.load_module_state(tmp)
        check(W.compare_actor_states({k: np.asarray(v) for k, v in init_raw.items()}, back).get("exact") is True, "save/reload round trip is bit-exact")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    # --- post-run verification (only once the fit has published) -----------------------------
    stage = {"post_run": post_run}
    if post_run:
        rp = FIT.OUT_S1 / FIT.RECEIPT_NAME
        rec = json.loads(rp.read_text(encoding="utf-8"))
        pub = {k: np.asarray(W._as_numpy(v)) for k, v in W.load_module_state(FIT.OUT_S1 / "rl_module").items()}
        stp = RF.validate_init_state(pub, expected_actor_digest=rec["gates"]["actor_digest_new"])
        check(stp["actor_digest"] == rec["gates"]["actor_digest_new"] and rec["gates"]["pass_all"] is True, "published module matches the receipt digest and the receipt records PASS")
        check(rec["init"]["actor_digest"] == FIT.PIN_S0D_ACTOR and rec["amendment_rev3o"] == FIT.PIN_AMENDMENT_REV3O, "receipt pins the S0D init and rev3o")
        check(C.sha256_file(FIT.OUT_S1 / "rl_module" / "module_state.pkl") == rec["output_files_sha256"]["module_state.pkl"], "published module_state matches the receipt digest")
        check(not any(p.name.startswith(".staging") or p.name.endswith(".lock") for p in FIT.OUT_S1.parent.iterdir()), "no orphan staging/lock next to the export")
        stage["published_actor_digest"] = stp["actor_digest"]
    print(json.dumps({"selftest": "PASS", "checks": CHECKS, "prefit_T1": t1, **stage}, indent=1))
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
