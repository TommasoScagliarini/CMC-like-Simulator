"""Self-test rev3q S1A materialisation. Stage-aware: valid before AND after the run."""
from __future__ import annotations
import json, shutil, sys, tempfile
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_s1a_bc as A  # noqa: E402
import v26b_s1p_multirole as P  # noqa: E402
import v26b_s1_fit as FIT  # noqa: E402
import v26b_s1_prereg as S1  # noqa: E402
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

def synth(g_task=(0.13, 0.12), q1=(0.49, 0.34), q3=True, t1=0.0, t2=0.0, digest=None):
    g = FIT.gate_decision(list(g_task), list(q1), q3, t1, t2)
    g["actor_digest_new"] = digest if digest is not None else A.P0_ACTOR_DIGEST
    return g

def main() -> int:
    published_before = A.OUT_S1A.exists()

    # --- lineage rev3l -> rev3q ------------------------------------------------------------------
    lin = A.verify_lineage_s1a()
    check(lin["amendment_rev3q"] == A.PIN_AMENDMENT_REV3Q and lin["amendment_rev3p"] == P.PIN_AMENDMENT_REV3P
          and lin["amendment_rev3o"] == FIT.PIN_AMENDMENT_REV3O and lin["amendment_rev3n"] == FIT.N.PIN_AMENDMENT_REV3N
          and lin["amendment_rev3l"] == S1.PIN_AMENDMENT_REV3L and lin["amendment_rev3m"] == S1.PIN_AMENDMENT_REV3M,
          "lineage pins the whole chain rev3l -> rev3q")
    check(lin["rev3o_tool"] == P.PIN_REV3O_TOOL and lin["rev3p_tool"] == A.PIN_REV3P_TOOL
          and lin["rev3o_rejected_artifact"] == P.PIN_REV3O_REJECTED, "prior tools and the rev3o REJECTED evidence pinned")
    check(lin["rev3o_no_checkpoint_published"] is True and lin["rev3p_no_checkpoint_published"] is True
          and lin["output_outside_student"] is True, "no prior checkpoint exists; output declared outside student/")
    for attr in ("PIN_AMENDMENT_REV3Q", "PIN_REV3P_TOOL"):
        old = getattr(A, attr)
        try:
            setattr(A, attr, "0" * 64); expect(A.verify_lineage_s1a, A.S1AError, f"tampered {attr} -> refused")
        finally: setattr(A, attr, old)

    # --- tokens: negatives (incl. every previously used token) and the positive -------------------
    for bad in (None, "V26B-S1-FIT", "V26B-S1P-MULTIROLE-FIT", "V26B-S1-REV3N-PREREG-MEDIAN",
                "V26B-S0D-FIT", "V26B-S1-PREREG-READONLY", "v26b-s1a-bc-fit"):
        e = expect(lambda b=bad: A.run_s1a(authorized_stage=b), A.S1AError, f"token {bad!r} refused")
        check("V26B-S1A-BC-FIT" in str(e), "guard names the required token")
    e = expect(lambda: A.run_s1a(authorized_stage=A.AUTHORIZED_STAGE, out_dir=Path("student/nope")),
               A.S1AError, "the correct token passes the token guard and stops at the student/ guard")
    check("student/" in str(e), "the positive token proceeds past the token check (fails later, on the path rule)")

    # --- output must never live under student/ ----------------------------------------------------
    check("student" not in A.OUT_S1A.resolve().parts and A.OUT_S1A.name == "S1A_IK_AB06_35D_NONDEPLOYABLE"
          and A.OUT_S1A.parent.name == "candidates", "the declared output is candidates/S1A_..._NONDEPLOYABLE, outside student/")
    expect(lambda: A.assert_not_under_student(Path("a/student/b")), A.S1AError, "any student/ path refused")

    # --- P0: values, provenance, and every single-item mismatch -----------------------------------
    rej = json.loads((A.VA.OUT_ROOT / "v26b_s1_fit_REJECTED_20260824_190422.json").read_text())
    rg = rej["gates"]
    check(A.P0_G_TASK == rg["G_task"]["per_joint"] and A.P0_Q1_DIAGNOSTIC == rg["Q1_source_holdout_rev3m"]["per_joint"]
          and A.P0_T1 == rg["function_preservation"]["T1_maxabs"] and A.P0_T2 == rg["function_preservation"]["T2_maxabs"]
          and A.P0_ACTOR_DIGEST == rg["actor_digest_new"], "the five P0 pins come verbatim from the immutable rev3o REJECTED artifact")
    check(A.p0_check(synth(g_task=A.P0_G_TASK, q1=A.P0_Q1_DIAGNOSTIC, t1=A.P0_T1, t2=A.P0_T2))["pass"] is True, "P0 passes on exact reproduction")
    up = lambda x: float(np.nextafter(x, np.inf))  # a TRUE 1-ulp perturbation  # noqa: E731
    for label, kw in (("G_task", {"g_task": (up(A.P0_G_TASK[0]), A.P0_G_TASK[1])}),
                      ("Q1", {"q1": (A.P0_Q1_DIAGNOSTIC[0], up(A.P0_Q1_DIAGNOSTIC[1]))}),
                      ("T1", {"t1": up(A.P0_T1)}), ("T2", {"t2": up(A.P0_T2)}),
                      ("digest", {"digest": "0" * 64})):
        base = {"g_task": A.P0_G_TASK, "q1": A.P0_Q1_DIAGNOSTIC, "t1": A.P0_T1, "t2": A.P0_T2}
        base.update(kw)
        r = A.p0_check(synth(**base))
        check(r["pass"] is False and len(r["mismatch"]) == 1, f"P0 catches a 1-ulp/one-item deviation in {label}")

    # --- Q1 is diagnostic: it can never block ------------------------------------------------------
    ok_p0 = A.p0_check(synth(g_task=A.P0_G_TASK, q1=A.P0_Q1_DIAGNOSTIC, t1=A.P0_T1, t2=A.P0_T2))
    g_hi_q1 = synth(g_task=(0.13, 0.12), q1=(0.90, 0.90))
    d = A.materialisation_decision(g_hi_q1, ok_p0)
    check(d["materialise"] is True and d["Q1_is_binding"] is False and d["Q1_observed"] == [0.90, 0.90],
          "a synthetic Q1 far beyond 0.10 does NOT block materialisation")
    check(A.rev3q_gate_view(g_hi_q1)["Q1_source_holdout_rev3m"]["binding"] is False
          and "pass_all" not in A.rev3q_gate_view(g_hi_q1), "the published gate view marks Q1 non-binding and carries no global pass_all")

    # --- each binding gate blocks in isolation ------------------------------------------------------
    for label, g, p0 in (("G_task", synth(g_task=(0.1501, 0.12)), ok_p0),
                         ("Q3_invariants", synth(q3=False), ok_p0),
                         ("function_preservation", synth(t2=2e-5), ok_p0),
                         ("P0", synth(), A.p0_check(synth(digest="1" * 64)))):
        dd = A.materialisation_decision(g, p0)
        check(dd["materialise"] is False and dd["failed_binding_gates"] == [label], f"{label} alone blocks materialisation")

    # --- no deployable marking or alias --------------------------------------------------------------
    A.assert_no_deployable_marking({**A.MANDATORY_FLAGS, "contract": A.CONTRACT_STRING, "p": str(A.OUT_S1A)}, "ok")
    check(True, "the prescribed flags, contract string and output path pass the no-deployable-marking scan")
    expect(lambda: A.assert_no_deployable_marking({"deployable": True}), A.S1AError, "deployable:true refused")
    expect(lambda: A.assert_no_deployable_marking({"contract": "deployable_markov_controller_state"}), A.S1AError,
           "the historical deployable contract string is refused as an alias")
    check(A.MANDATORY_FLAGS == {"deployable": False, "rollout_pending": True, "sigma_unresolved": True,
                                "promotion_requires": "closed-loop nominal rollout under a separate token"}, "mandatory flags exactly as prescribed")

    # --- leakage / embargo / split --------------------------------------------------------------------
    view = FIT.build_s1_task(); step = np.arange(1, 501)
    check(int(view["train"].sum()) == 380 and int(view["hold"].sum()) == 100 and int(view["embargo"].sum()) == 20, "380/100/20")
    check(not np.any((step[view["train"]] >= 191) & (step[view["train"]] <= 310)), "no training row inside steps 191-310")
    check(np.array_equal(step[view["hold"]], np.arange(201, 301)), "holdout is exactly steps 201-300")
    check(A.G_TASK_MAX is FIT.G_TASK_MAX and A.G_TASK_MAX == 0.15, "G_task threshold reused by reference, unchanged")

    # --- init invariants: clock zero, logstd frozen ----------------------------------------------------
    if str(R.BASELINE_DIR) not in sys.path: sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    init_raw = {k: np.asarray(W._as_numpy(v)) for k, v in W.load_module_state(FIT.S0D_MODULE).items()}
    st0 = RF.validate_init_state(init_raw, expected_actor_digest=FIT.PIN_S0D_ACTOR)
    check(bool(st0["clock_columns_zero"]) and bool(st0["sigma_head"]["logstd_bias_exact"])
          and tuple(init_raw.keys()) == RF.EXPECTED_KEY_ORDER, "init: clock columns zero, logstd placeholder, 10 keys")

    # --- no-publish-on-fail, end to end -----------------------------------------------------------------
    tmp = Path(tempfile.mkdtemp(prefix="s1a_nopub_"))
    try:
        def stub_fit(init_scaled, view_, vec, progress=False):
            f32 = np.float32
            sf = {k: np.array(init_scaled[k], dtype=f32) for k in ("pi.0.0.weight", "pi.0.0.bias", "pi.0.2.weight", "pi.0.2.bias", "pi.1.weight", "pi.1.bias")}
            return sf, {"tool": "stub(no-op)", "history": [{"epoch": 1, "loss": 0.0}]}
        e = expect(lambda: A.run_s1a(authorized_stage=A.AUTHORIZED_STAGE, out_dir=tmp / "S1A_never",
                                     reject_dir=tmp, progress=False, _fit_fn=stub_fit),
                   A.S1AError, "a wrong candidate is refused")
        check("materialisation gates FAILED" in str(e) and "P0" in str(e), "the refusal names P0 among the failed gates")
        arts = list(tmp.glob("v26b_s1a_bc_REJECTED_*.json"))
        check(len(arts) == 1, "exactly one REJECTED artifact")
        rj = json.loads(arts[0].read_text())
        check(rj["REJECTED"] is True and rj["P0"]["pass"] is False and rj["deployable"] is False
              and rj["rollout_pending"] is True and rj["sigma_unresolved"] is True, "REJECTED artifact carries P0 and the mandatory flags")
        check(not (tmp / "S1A_never").exists(), "NO candidate directory on failure")
        check(not any(p.name.startswith(".staging") or p.name.endswith(".lock") for p in tmp.iterdir()), "no staging/lock residue")
        check(A.OUT_S1A.exists() == published_before, "the real output path is untouched by the failure path")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    # --- offline only ------------------------------------------------------------------------------------
    src = (HERE / "v26b_s1a_bc.py").read_text()
    check(not any(t in src for t in ("subprocess", "rollout_eval", "os.system", "Popen", "--checkpoint")),
          "the module contains no closed-loop execution primitive")
    check("if False else" not in src, "no dead conditional code left in the module")

    # --- post-run verification ------------------------------------------------------------------------------
    stage = {"post_run": A.OUT_S1A.exists()}
    if stage["post_run"]:
        rec = json.loads((A.OUT_S1A / A.RECEIPT_NAME).read_text(encoding="utf-8"))
        man = json.loads((A.OUT_S1A / "rl_module" / "actor_feature_manifest.json").read_text(encoding="utf-8"))
        pub = {k: np.asarray(W._as_numpy(v)) for k, v in W.load_module_state(A.OUT_S1A / "rl_module").items()}
        stp = RF.validate_init_state(pub, expected_actor_digest=A.P0_ACTOR_DIGEST)
        check(stp["actor_digest"] == A.P0_ACTOR_DIGEST == rec["gates"]["actor_digest_new"], "published actor digest == the P0 pin")
        check(rec["P0_reproducibility"]["pass"] is True and rec["decision"]["materialise"] is True
              and rec["decision"]["Q1_is_binding"] is False, "receipt records P0 PASS and Q1 non-binding")
        for k, v in A.MANDATORY_FLAGS.items():
            check(rec[k] == v and man[k] == v, f"mandatory flag {k} present in receipt and manifest")
        A.assert_no_deployable_marking(man, "published manifest")
        check(True, "published manifest carries no deployable marking or alias")
        check(bool(stp["clock_columns_zero"]) and np.array_equal(np.asarray(pub["pi.1.weight"])[R.ACTION_DIM:], np.asarray(init_raw["pi.1.weight"])[R.ACTION_DIM:])
              and np.array_equal(np.asarray(pub["pi.1.bias"])[R.ACTION_DIM:], np.asarray(init_raw["pi.1.bias"])[R.ACTION_DIM:]),
              "published module: clock columns zero and logstd rows bit-identical to the init")
        check("student" not in A.OUT_S1A.resolve().parts, "published outside student/")
        check(C.sha256_file(A.OUT_S1A / "rl_module" / "module_state.pkl") == rec["output_files_sha256"]["module_state.pkl"], "module_state digest matches the receipt")
        check(not any(p.name.startswith(".staging") or p.name.endswith(".lock") for p in A.OUT_S1A.parent.iterdir()), "no orphan staging/lock")
        check(FIT.OUT_S1.exists() is False and P.OUT_S1P.exists() is False, "rev3o and rev3p paths still empty")
        stage["published_actor_digest"] = stp["actor_digest"]
    print(json.dumps({"selftest": "PASS", "checks": CHECKS, **stage}, indent=1))
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
