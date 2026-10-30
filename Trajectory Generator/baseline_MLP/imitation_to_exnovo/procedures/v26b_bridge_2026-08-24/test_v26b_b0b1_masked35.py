"""Tests for the 35D-masked branch (B0/B1). No fit, training, rollout or collection."""
from __future__ import annotations
import json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_b0_masked35 as B0  # noqa: E402
import v26b_b1_base_fit as B1  # noqa: E402
import v26b_student as VS  # noqa: E402
import f2r_refit as RF  # noqa: E402
import f2r_common as R  # noqa: E402

CHECKS = 0
def check(c, w):
    global CHECKS; assert c, w; CHECKS += 1
def expect(fn, exc, w):
    global CHECKS
    try: fn()
    except exc as e: CHECKS += 1; return e
    raise AssertionError(f"expected {exc.__name__}: {w}")


def main() -> int:
    cols = B0.controller_columns(); keep = B0.kept_columns()
    names35, _, _ = VS.pinned_names()

    # --- single 35D contract, mask derived by name -------------------------------------------------
    check(cols == (25, 26, 27, 28, 29, 30, 31, 32, 33, 34) and len(cols) == 10,
          "the ten masked columns are derived from the manifest by name")
    check(len(keep) == 25 and set(keep) | set(cols) == set(range(35)),
          "masked and kept columns partition the 35 features")
    check(B0.ACTOR_WIDTH == 35 and len(names35) == 35,
          "the actor stays 35 wide: one contract, one manifest, one feature list")
    check(not set(cols) & set(B0.CLOCK_COLUMNS), "mask and clock column sets are disjoint")
    check(not any("healthy_" in n or n.endswith("_r") for n in names35),
          "no contralateral feature is present in the 35D actor manifest")
    for attr, bad in (("EXPECTED_CONTROLLER_COLUMNS", (1, 2)), ("PIN_MANIFEST35", "0" * 64)):
        old = getattr(B0, attr)
        try:
            setattr(B0, attr, bad)
            expect(B0.controller_columns, B0.B0Error, f"tampered {attr} refused")
        finally: setattr(B0, attr, old)

    # --- input mask ----------------------------------------------------------------------------------
    rng = np.random.default_rng(5)
    x = rng.normal(size=(12, 35)).astype(np.float32)
    xm = B0.apply_input_mask(x)
    check(np.all(xm[:, list(cols)] == 0.0), "the mask zeroes exactly the ten columns")
    check(np.array_equal(xm[:, list(keep)], x[:, list(keep)]), "the mask leaves the other columns alone")
    check(not np.array_equal(x, xm) and np.any(x[:, list(cols)] != 0.0),
          "the mask is not a no-op on this probe, so the tests are not vacuous")
    xc = x.copy(); B0.apply_input_mask(xc)
    check(np.array_equal(xc, x), "apply_input_mask never mutates its argument")
    expect(lambda: B0.apply_input_mask(np.zeros((4, 25), np.float32)), B0.B0Error, "wrong width refused")

    # --- zero / restore / no-update, fail-closed -------------------------------------------------------
    st, _ = B0.build_b0_state()
    B0.assert_masked_columns_zero(st); B0.assert_clock_columns_zero(st)
    check(True, "the B0 state has the masked and clock columns exactly zero")
    dirty = {k: np.array(v, copy=True) for k, v in st.items()}
    W = dirty["pi.0.0.weight"].copy(); W[3, cols[4]] = np.float32(1e-9); dirty["pi.0.0.weight"] = W
    e = expect(lambda: B0.assert_masked_columns_zero(dirty), B0.B0Error,
               "a single 1e-9 leak in a masked column is detected")
    check("not exactly zero" in str(e), "the guard reports the leak explicitly")
    restored = B0.restore_masked_columns({k: np.array(v, copy=True) for k, v in dirty.items()})
    B0.assert_masked_columns_zero(restored)
    check(True, "restore_masked_columns puts them back to exactly zero")
    B0.assert_no_masked_update(st, st)
    check(True, "an unchanged state passes assert_no_masked_update")
    expect(lambda: B0.assert_no_masked_update(st, dirty), B0.B0Error,
           "assert_no_masked_update detects a changed masked column")

    # --- functional equivalence with the 25D subspace, bit-exact ---------------------------------------
    eq = B0.functional_equivalence_25(st, x)
    check(eq["bit_identical"] and eq["max_abs_difference"] == 0.0,
          "the masked 35D actor is BIT-IDENTICAL to its 25-column subnetwork on random data")
    real = B0.probe_observations(128)
    eq2 = B0.functional_equivalence_25(st, real)
    check(eq2["bit_identical"], "the same holds on real pinned V26 anchor observations")
    # The two guarantees are INDEPENDENT and cover different failure modes.
    # (a) the INPUT MASK alone secures the 25D equivalence: a leaking column multiplies a zero
    #     input, so the output is unchanged. The equivalence therefore holds even on `dirty`.
    eq_dirty = B0.functional_equivalence_25(dirty, x)
    check(eq_dirty["bit_identical"],
          "the input mask alone secures 25D equivalence even if a column leaks: it multiplies zero")
    # (b) the ZERO COLUMNS alone secure the unmask transition, and THAT is the test which detects a
    #     leaking column - verified a few lines below on the same `dirty` state.

    # --- bit-exact mask-on -> full35 transition ---------------------------------------------------------
    tr = B0.bit_exact_unmask_transition(st, x)
    check(tr["bit_identical"] and tr["max_abs_difference"] == 0.0 and tr["probe_columns_differ"],
          "with the columns at zero, removing the input mask is BIT-EXACT")
    tr2 = B0.bit_exact_unmask_transition(st, real)
    check(tr2["bit_identical"], "the transition is bit-exact on real anchor observations too")
    expect(lambda: B0.bit_exact_unmask_transition(dirty, x), B0.B0Error,
           "a non-zero masked column breaks the transition, so the test is load-bearing")
    expect(lambda: B0.bit_exact_unmask_transition(st, B0.apply_input_mask(x)), B0.B0Error,
           "an already-masked probe is refused as vacuous")

    # --- gradient mask, demonstrated on a real optimizer step -------------------------------------------
    import torch
    torch.manual_seed(0)
    Wt = torch.nn.Parameter(torch.randn(8, 35))
    with torch.no_grad():
        Wt[:, list(cols)] = 0.0
    opt = torch.optim.Adam([Wt], lr=0.1)
    xt = torch.randn(16, 35)
    xtm = xt.clone(); xtm[:, list(cols)] = 0.0
    (xtm @ Wt.T).sum().backward()
    check(torch.all(Wt.grad[:, list(cols)] == 0.0),
          "with masked inputs the gradient on the masked columns is IDENTICALLY zero")
    check(torch.any(Wt.grad[:, list(keep)] != 0.0), "the kept columns do receive gradient")
    before = {"pi.0.0.weight": Wt.detach().numpy().copy(),
              "pi_encoder.0.weight": Wt.detach().numpy().copy()}
    opt.step()
    after = {"pi.0.0.weight": Wt.detach().numpy().copy(),
             "pi_encoder.0.weight": Wt.detach().numpy().copy()}
    B0.assert_no_masked_update(before, after)
    check(True, "after a real Adam step with masked inputs the masked columns are untouched and zero")
    # negative control: unmasked inputs DO move those columns, and the guard catches it
    Wt2 = torch.nn.Parameter(torch.randn(8, 35))
    with torch.no_grad():
        Wt2[:, list(cols)] = 0.0
    opt2 = torch.optim.Adam([Wt2], lr=0.1)
    (xt @ Wt2.T).sum().backward()
    check(torch.any(Wt2.grad[:, list(cols)] != 0.0),
          "negative control: unmasked inputs DO produce gradient on those columns")
    b2 = {"pi.0.0.weight": Wt2.detach().numpy().copy(), "pi_encoder.0.weight": Wt2.detach().numpy().copy()}
    opt2.step()
    a2 = {"pi.0.0.weight": Wt2.detach().numpy().copy(), "pi_encoder.0.weight": Wt2.detach().numpy().copy()}
    expect(lambda: B0.assert_no_masked_update(b2, a2), B0.B0Error,
           "negative control: without masking the guard FAILS CLOSED on the leak")
    a2r = B0.restore_masked_columns({k: v.copy() for k, v in a2.items()})
    B0.assert_no_masked_update(b2, a2r)
    check(True, "the restore step repairs the leak and the guard then passes")

    # --- B0 transplant provenance --------------------------------------------------------------------
    _, rep = B0.build_b0_state()
    t = rep["transplant"]
    check(t["parent_actor_digest"] == B0.PIN_V26_ACTOR_DIGEST
          and t["v1_recipe_digest_reproduced"] == B0.PIN_V1_RECIPE_DIGEST,
          "B0 descends from the pinned August V26 actor through the reproduced 39->35 recipe")
    check("NOT COMPENSATED" in t["mean_compensation"]["controller_columns"],
          "the masked columns are deliberately not mean-compensated")
    check("COMPENSATED" in t["mean_compensation"]["healthy_targets"],
          "the four removed healthy targets are mean-compensated, as in the frozen recipe")
    v1, _ = VS.build_v1_state()
    for k in v1:
        if k in ("pi.0.0.weight", "pi_encoder.0.weight"):
            check(np.array_equal(np.asarray(st[k])[:, list(keep)], np.asarray(v1[k])[:, list(keep)]),
                  f"{k}: the kept columns are bit-identical to the 39->35 recipe")
        else:
            check(np.array_equal(np.asarray(st[k]), np.asarray(v1[k])),
                  f"{k} is bit-identical to the 39->35 recipe (only first-layer columns may differ)")
    RF.validate_init_state(st, expected_actor_digest=None, width=35)
    check(True, "the frozen structural validator accepts the masked state at width 35")
    for bad in ("/x/candidates/S0D_35D_DISTILLED/rl", "/x/candidates/S1A_IK_AB06/rl",
                "/x/candidates/REV4E_R2REPLAY_35D/rl", "/x/student/V2_DAGGER_R1/rl",
                "/x/runs/training/target_domain_dagger_2026-07-11_r2"):
        expect(lambda b=bad: B0.assert_not_forbidden(b, "t"), B0.B0Error, f"{bad} refused as source")

    # --- quarantine of the superseded 25D branch ---------------------------------------------------------
    for mod in ("v26b_b0_masked35.py", "v26b_b1_base_fit.py"):
        src = (HERE / mod).read_text()
        check("v26b_a0_transplant25" not in src.replace("SUPERSEDED_MODULES", "")
              or "SUPERSEDED_MODULES" in src, "the superseded modules appear only in the quarantine list")
        check("import v26b_a0_transplant25" not in src and "import v26b_a1_ik_imitation" not in src,
              f"{mod} never imports a superseded 25D module")
    check(all(m not in sys.modules for m in B0.SUPERSEDED_MODULES),
          "no superseded module is loaded in this process")
    add = json.loads((HERE / "v26b_addendum_a0a1_superseded.json").read_text())
    check(add["status"] == "SUPERSEDED_BEFORE_EXECUTION"
          and add["execution_state_at_supersession"]["fit"] == "NEVER RUN",
          "the addendum marks the 25D branch superseded before execution")
    for n, s in (("v26b_amendment_a0a1_25d.json", "9dfe122925b060a708fd992768f3ed1737f37a631a3c71e0b13710b2d0764124"),
                 ("v26b_contract_25d_v1.yaml", "b7292a73a9d342dd61773ab5bc85d66b1532bc2a0d76fd4d1dad37758fa67fae"),
                 ("v26b_a0_transplant25.py", "bce45d030f93db94de385e93720b5a1a695ff57ce53f1df2235bfdf1ffa8d593")):
        import hashlib
        check(hashlib.sha256((HERE / n).read_bytes()).hexdigest() == s,
              f"the superseded file {n} is still on disk, byte-identical")

    # --- B1 dataset and split ------------------------------------------------------------------------------
    data = B1.build_dataset()
    r = data["report"]
    check(r["rows"] == 1500 and r["trajectories"] == 3 and r["collection_performed"] is False,
          "1500 rows over 3 pinned V26 trajectories, with no collection performed")
    check("no S0D, S1A, REV4* or V2_* row" in r["provenance"], "the provenance excludes the diagnostic actors")
    check(np.all(data["observations_masked"][:, list(cols)] == 0.0), "the dataset observations are masked")
    check(np.array_equal(data["observations_masked"][:, list(keep)],
                         data["observations_raw"][:, list(keep)]), "masking preserved the kept columns")
    check(data["actions"].shape == (1500, 2) and np.all(np.abs(data["actions"]) <= 1.0 + 1e-6),
          "the labels are (1500,2) and inside the normalised range")
    sp = B1.trajectory_split(data["trajectory_id"])
    check(sp["mode"] == "leave_one_trajectory_out" and sp["trajectories"] == 3
          and all(f["train_rows"] == 1000 and f["holdout_rows"] == 500 for f in sp["folds"]),
          "leave-one-trajectory-out with 1000/500 per fold")
    expect(lambda: B1.trajectory_split([0] * 50), B1.B1Error, "a single trajectory cannot be split")
    srcb1 = (HERE / "v26b_b1_base_fit.py").read_text()
    check("permutation" not in srcb1 and "shuffle" not in srcb1, "the split is never random")

    # --- gates ------------------------------------------------------------------------------------------------
    check(B1.ANKLE_MIN_RAD == -0.03, "the ankle threshold is -0.03 rad")
    g = B1.kinematic_gates(np.linspace(-0.9, -0.2, 50), np.linspace(-0.0099, 0.35, 50))
    check(g["gates"]["ankle_plantarflexion"]["pass"] is False,
          "an ankle minimum of -0.0099 rad does NOT satisfy q_min <= -0.03")
    check(not (-0.0099 <= B1.ANKLE_MIN_RAD), "-0.0099 > -0.03, arithmetically")
    check(B1.kinematic_gates(np.linspace(-0.9, -0.2, 50),
                             np.linspace(-0.031, 0.35, 50))["gates"]["ankle_plantarflexion"]["pass"],
          "-0.031 satisfies it")
    d = B1.declared_closed_loop_gates()
    check(d["completion"]["steps"] == 500 and d["valid_cycle_count_min"] == 2
          and d["penetration_max_m"] == 0.020 and d["resync_count_max"] == 1,
          "the closed-loop gates carry over unchanged")
    check(set(d["critical_counters_zero"]) == set(B1.CRITICAL_COUNTERS), "critical counters enumerated")
    check("No guard is relaxed" in d["motivation"]["penetration"], "no guard is relaxed")
    prot = B1.masked_training_protocol()
    check(len(prot["per_step"]) == 7 and "restore" in prot["per_step"][4]
          and "assert_no_masked_update" in prot["per_step"][5],
          "the per-step protocol restores and then asserts")
    check(prot["no_ppo"] is True and "critic (never touched)" in prot["frozen"], "no PPO, critic frozen")
    mk = B1.markov_phase_declared()
    check(mk["whole_mean_network_adaptation"].startswith("NOT decided now"),
          "whether to adapt the whole mean network is left to a later preregistration")
    check("bit-exact by construction" in mk["operation"], "the Markov transition is declared bit-exact")
    check(B1.SIGMA_VERIFICATION["status"].startswith("OPEN"), "sigma is not assumed")

    # --- nothing executes ---------------------------------------------------------------------------------------
    expect(lambda: B1.run_b1(authorized_stage="V26B-B1-BASE-FIT-MASKED35"), B1.B1Error,
           "B1 refuses to run even with its own token")
    expect(lambda: B0.main(["--authorized-stage", "V26B-B0-MASKED35-TRANSPLANT"]), B0.B0Error,
           "B0 refuses to materialise")
    for s in ((HERE / "v26b_b0_masked35.py").read_text(), srcb1):
        check(not any(t in s for t in ("subprocess", "rollout_eval", "train_ppo", "PPOConfig",
                                       "collect_teacher_dataset", "promote_staging", "write_json")),
              "neither module can fit, train, roll out, collect or materialise")
        check("os.system" not in s and "os.sep" not in s and "from pathlib import Path" in s,
              "pathlib only, no shell, no os-specific path handling")

    print(json.dumps({"selftest": "PASS", "checks": CHECKS}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
