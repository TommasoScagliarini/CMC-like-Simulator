"""Self-test rev4e July-R2-protocol replay. Stage-aware.
The interpolation semantics are proven by DIFFERENTIAL COMPARISON against the real July function
(target_domain_imitation.aggregate_dagger_traces), not asserted."""
from __future__ import annotations
import json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_rev4e_r2_replay as E  # noqa: E402
import v26b_rev4d_dagger as D4  # noqa: E402
import v26b_rev4c_dagger as C4  # noqa: E402
import v26b_rev4b_dagger as B4  # noqa: E402
import v26b_student as VS  # noqa: E402
import f2r_common as R  # noqa: E402
sys.path.insert(0, str(R.BASELINE_DIR))
import target_domain_imitation as JULY  # noqa: E402   (PROTOCOL reference only: the function, never July data)

CHECKS = 0
def check(cond, what):
    global CHECKS; assert cond, what; CHECKS += 1
def expect(fn, exc, what):
    global CHECKS
    try: fn()
    except exc as e: CHECKS += 1; return e
    raise AssertionError(f"expected {exc.__name__}: {what}")


def main() -> int:
    ran = E.OUT_CAND.exists()
    names35, _, _ = VS.pinned_names()

    # --- lineage, pins, the four mandatory declarations -----------------------------------------------
    lin = E.verify_lineage()
    check(lin["amendment_rev4e"] == E.PIN_AMENDMENT_REV4E and lin["amendment_rev4d"] == D4.PIN_AMENDMENT_REV4D
          and lin["amendment_rev4c"] == C4.PIN_AMENDMENT_REV4C and lin["amendment_rev4b"] == B4.PIN_AMENDMENT_REV4B,
          "lineage pins rev4b, rev4c, rev4d and rev4e")
    am = json.loads(E.AMENDMENT_REV4E.read_text())
    dec = am["MANDATORY_DECLARATIONS"]
    check(set(dec) == {"a_multivariate_protocol_replay", "b_attribution_limits",
                       "c_split_leakage_is_expected", "d_known_risk_of_the_drifted_band"},
          "all four mandatory declarations are present")
    check("NOT A ONE-VARIABLE EXPERIMENT" in dec["a_multivariate_protocol_replay"]
          and "FOUR dimensions change together" in dec["a_multivariate_protocol_replay"],
          "(a) the replay is declared multivariate")
    check("DOES NOT ATTRIBUTE CAUSALITY" in dec["b_attribution_limits"]
          and "A FAILURE CLOSES" in dec["b_attribution_limits"], "(b) attribution limits declared")
    check("DIAGNOSTIC ONLY" in dec["c_split_leakage_is_expected"] and "88" in dec["c_split_leakage_is_expected"],
          "(c) split leakage declared with its magnitude")
    check("22.267" in dec["d_known_risk_of_the_drifted_band"] and "10.134" in dec["d_known_risk_of_the_drifted_band"]
          and "SMALL deviations" in dec["d_known_risk_of_the_drifted_band"],
          "(d) the drifted-band risk is declared with measured drift")
    check(lin["init_anchor_module"]["actor_digest"] == E.PIN_INIT_ACTOR_DIGEST
          and lin["init_anchor_module"]["files_sha256"]["module_state.pkl"] == E.PIN_INIT_FILES["module_state.pkl"],
          "init/anchor module pinned by SHA")
    check(lin["trace_1_s1a"] == E.PIN_TRACE1_S1A and lin["trace_2_rev4c"] == E.PIN_TRACE2_REV4C,
          "both traces pinned by SHA")
    for attr in ("PIN_AMENDMENT_REV4E", "PIN_INIT_ACTOR_DIGEST", "PIN_TRACE2_REV4C"):
        old = getattr(E, attr)
        try:
            setattr(E, attr, "0" * 64)
            expect(E.verify_lineage if attr != "PIN_INIT_ACTOR_DIGEST" else E.preflight,
                   E.Rev4eError, f"tampered {attr} -> refused")
        finally: setattr(E, attr, old)

    # --- July artifacts must never be operational -----------------------------------------------------
    for bad in ("/x/runs/training/target_domain_dagger_2026-07-11_r2/rl_module_target_adapted",
                "/x/runs/rollout/2026-07-11_target_dagger_r2_recorded/rollout_policy_trace.json",
                "/x/target_domain_imitation_2026-07-11_v2/teacher_dataset.npz"):
        e = expect(lambda b=bad: E.assert_no_july_artifact([b]), E.Rev4eError, f"July path {bad} refused")
        check("JULY ARTIFACT" in str(e), "the guard names the violation")
    E.assert_no_july_artifact([E.INIT_MODULE, E.TRACE2_JOB])
    check(True, "current-lineage paths pass the July guard")

    # --- INTERPOLATION: differential proof against the real July function -----------------------------
    rng = np.random.default_rng(7)
    n_t, n_v = 12, 5
    tobs = rng.normal(size=(n_t, 35)).astype(np.float32)
    tact = rng.normal(size=(n_t, 2)).astype(np.float32)
    times = np.arange(n_t, dtype=np.float64)
    vobs = rng.normal(size=(n_v, 35)).astype(np.float32)
    teacher = {"observations": tobs, "actions": tact, "times": times,
               "actor_feature_names": np.asarray(names35, dtype=str)}
    trace = [{"step": i + 1, "actor_observation_vector_before": vobs[i].tolist()} for i in range(n_v)]
    for k in (0, 1, 2, 3):
        agg, summ = JULY.aggregate_dagger_traces(teacher, [trace], trace_repeat=1, interpolation_steps=k)
        mine = []
        for i in range(n_v):
            mine.extend(E.july_interpolate(vobs[i], tobs[i], k))
        mine = np.asarray(mine, dtype=np.float32)
        july_pool = np.asarray(agg["observations"][n_t:], dtype=np.float32)
        check(np.array_equal(mine, july_pool),
              f"july_interpolate reproduces the real July pool BIT-IDENTICALLY at k={k}")
        check(summ["unique_dagger_samples"] == n_v * (1 + k),
              f"July's own summary confirms the total factor is 1+k at k={k}")
    # the July rule's own discrete indices must equal our pinned constant
    agg2, _ = JULY.aggregate_dagger_traces(teacher, [trace], trace_repeat=1, interpolation_steps=2)
    disc = [i for i, nm in enumerate(names35)
            if nm.endswith(("_in_contact", "_heel_strike", "_toe_off", "_saturated"))
            or nm.startswith(("phase_fsm_", "phase_expected_"))]
    check(tuple(disc) == E.DISCRETE_INDICES,
          "the pinned discrete indices equal those the July rule derives from the 35D names")
    loc = E.july_interpolate(vobs[0], tobs[0], 2)
    check(len(loc) == 3 and np.array_equal(loc[0], vobs[0].astype(np.float32)), "k=2 gives [raw, i1, i2]")
    for j in (1, 2):
        check(np.array_equal(loc[j][list(E.DISCRETE_INDICES)], vobs[0].astype(np.float32)[list(E.DISCRETE_INDICES)]),
              f"child {j} pins the discrete columns to the visited row")
    cont = [c for c in range(35) if c not in E.DISCRETE_INDICES]
    for j, a in ((1, 1 / 3), (2, 2 / 3)):
        want = tobs[0] + np.float32(a) * (vobs[0] - tobs[0])
        check(np.allclose(loc[j][cont], want[cont], rtol=0, atol=1e-6), f"child {j} sits at alpha={a:.4f}")
    expect(lambda: E.july_interpolate(vobs[0], tobs[0], -1), E.Rev4eError, "negative k refused")
    check(E.INTERP_FACTOR == 3 and E.INTERPOLATION_STEPS == 2, "2 points added, factor 3")

    # --- token guards -----------------------------------------------------------------------------------
    for bad in (None, "V26B-REV4D-REPEAT", "V26B-REV4C-BALANCE", "v26b-rev4e-r2-replay"):
        e = expect(lambda b=bad: E.run_stage(authorized_stage=b), E.Rev4eError, f"token {bad!r} refused")
        check("V26B-REV4E-R2-REPLAY" in str(e), "guard names this stage's token")

    # --- hygiene: keys and prose ------------------------------------------------------------------------
    expect(lambda: E.assert_no_foreign_labels({"kinematics_L20": {}}), E.Rev4eError, "foreign key refused")
    expect(lambda: E.assert_no_foreign_labels({"d": {"offline_reference_block": {}}}), E.Rev4eError,
           "offline_reference_block refused at depth")
    expect(lambda: E.assert_no_foreign_labels({"seven_gates": {"meaning": "closed-loop eligibility of A2. A PASS"}}),
           E.Rev4eError, "un-relabelled frozen S1B prose refused")
    rel = E.relabel_seven_gates({"gates": {}, "failed": [], "all_pass": False,
                                 "meaning": "closed-loop eligibility of A2. A PASS"}, E.ACTOR_LABEL)
    check(rel["meaning"].startswith("closed-loop eligibility of REV4E."), "prose relabelled to REV4E")
    E.assert_no_foreign_labels(rel)
    check("closed-loop eligibility of A2" in (HERE / "v26b_s1b_rollout.py").read_text(),
          "the frozen S1B tool is not modified on disk")

    # --- hyperparameters unchanged -----------------------------------------------------------------------
    src = (HERE / "v26b_rev4e_r2_replay.py").read_text()
    check("B4.fit_july" in src, "the July fit function is reused unmodified")
    check(not any(f"{k} =" in src for k in ("J_EPOCHS", "J_BATCH", "J_LR", "J_ANCHOR_W", "J_SEED", "J_PATIENCE")),
          "no July hyperparameter is redefined")
    check(B4.J_SEED == 123 and B4.J_EPOCHS == 400 and B4.J_BATCH == 64 and B4.J_LR == 3e-4
          and B4.J_ANCHOR_W == 1e-5 and B4.J_PATIENCE == 60 and B4.J_CLIP_W == 1.0 and B4.J_LOGSTD_W == 0.1,
          "the frozen July hyperparameters are the ones the amendment declares")
    fj = (HERE / "v26b_rev4b_dagger.py").read_text()
    check("anchor = [p.detach().clone() for p in params]" in fj,
          "fit_july anchors to the parameters it receives, so the anchor IS the init")

    # --- preflight ----------------------------------------------------------------------------------------
    pre = E.preflight()
    check(pre["verdict"] == "GO", "preflight GO")
    comp = pre["composition"]
    check(comp["raw_visited_rows"] == 184 and comp["interpolated_rows"] == 368
          and comp["unique_dagger_rows"] == 552 and comp["aggregate_rows"] == 1052
          and comp["on_policy_share"] == 552 / 1052 and comp["on_policy_share_exact_fraction"] == "552/1052",
          "184 raw, 368 interpolated, 552 unique, 1052 aggregate, exact share")
    check(comp["formula"] == "500 + (184 x 3) x 1 = 1052", "the formula is recorded literally")
    check(pre["teacher_corpus_rows"] == 500 and [t["rows"] for t in pre["traces"]] == [68, 116],
          "teacher 500 and traces 68 + 116")
    check(all(v == 0.0 for v in pre["time_alignment_max_abs_difference_s"].values()),
          "exact time alignment on both traces")
    check(pre["duplicates_with_conflicting_labels"] == 0 and pre["collisions_with_conflicting_labels"] == 0,
          "zero conflicting labels")
    nc = pre["negative_ankle_coverage"]
    check(nc["distinct_negative_time_indices"] == 14 and nc["windows"] == E.EXPECTED_NEGATIVE_WINDOWS
          and nc["rows_in_full_corpus"] == 97, "negative coverage 14 distinct over [[6,14],[112,116]]")
    check(pre["interpolation"]["children_verified"] == 184 * 2
          and pre["interpolation"]["total_factor"] == 3
          and pre["interpolation"]["discrete_indices_pinned"] == list(E.DISCRETE_INDICES),
          "every interpolated child verified: discrete pinned and on-segment")
    check(pre["no_july_artifact_operational"] is True, "no July artifact operational")
    check(pre["init_anchor"]["actor_digest"] == E.PIN_INIT_ACTOR_DIGEST
          and pre["init_anchor"]["ten_keys"] is True and pre["init_anchor"]["clock_columns_zero"] is True,
          "init/anchor validated structurally")
    for attr, val in (("TRACE2_ROWS", 115), ("RAW_VISITED", 183), ("INTERPOLATION_STEPS", 1),
                      ("EXPECTED_DISTINCT_NEGATIVE", 13)):
        old = getattr(E, attr)
        try:
            setattr(E, attr, val)
            expect(E.preflight, E.Rev4eError, f"a wrong {attr} fails closed")
        finally: setattr(E, attr, old)

    # --- aggregation ---------------------------------------------------------------------------------------
    agg = E.build_aggregate(pre)
    check(agg["observations"].shape[0] == 1052 and agg["actions"].shape[0] == 1052, "aggregate 1052")
    obsT = np.asarray(pre["_view"]["obs"], dtype=np.float32)
    check(np.array_equal(agg["observations"][:500], obsT), "first 500 rows are the teacher corpus verbatim")
    check(np.array_equal(agg["observations"][500:], np.asarray(pre["_pool"], dtype=np.float32)),
          "the on-policy block is the pool exactly once (repeat 1)")
    idx = pre["_idx"]
    check(all(idx[p] == idx[p + 1] == idx[p + 2] for p in range(0, 552, 3)),
          "each group of 3 shares one teacher index, so raw and children share the label")
    check([pre["_kind"][j] for j in range(3)] == ["raw", "interp_1", "interp_2"], "July row order per visited row")

    # --- criteria --------------------------------------------------------------------------------------------
    check(E.SURVIVAL_THRESHOLD_STEPS == 116 and am["CRITERIA"]["primary_gate"]["rule"] == "STRICTLY GREATER THAN 116",
          "primary gate strictly > 116")
    check(E.SECONDARY_MARKER_MIN_CYCLES == 1 and am["CRITERIA"]["secondary_marker"]["measure"] == "valid_cycle_count",
          "secondary marker is valid_cycle_count >= 1")
    check("no_automatic_promotion" in am["CRITERIA"], "no automatic promotion")
    check(not any(t in src for t in ("train_ppo", "PPOConfig", "sigma_sweep", "choose_sigma", "set_sigma")),
          "no PPO or sigma machinery")
    check(src.count("subprocess.run(") == 1, "exactly one harness invocation")

    # --- post-run ---------------------------------------------------------------------------------------------
    stage = {"post_run": ran}
    if ran:
        rec = json.loads((E.OUT_CAND / E.RECEIPT_NAME).read_text())
        E.assert_no_foreign_labels(rec, "materialised receipt")
        check(rec["deployable"] is False and rec["actor_label"] == "REV4E", "flags and label")
        check(rec["init_anchor_module"]["actor_digest"] == E.PIN_INIT_ACTOR_DIGEST, "init/anchor recorded")
        ds = rec["dataset"]
        check(ds["aggregate_rows"] == 1052 and ds["raw_visited_rows"] == 184 and ds["interpolated_rows"] == 368
              and ds["trace_repeat"] == 1 and ds["interpolation_steps"] == 2, "dataset recorded")
        jp = rec["july_protocol"]
        check(jp["batch_size"] == 64 and jp["learning_rate"] == 3e-4 and jp["anchor_weight"] == 1e-5
              and jp["seed"] == 123, "July hyperparameters unchanged")
        check(rec["offline"]["binding"]["integrity_invariants"]["logstd_byte_identical_to_init"] is True
              and rec["save_reload_exact"] is True, "logstd byte-identical to the init; save/reload exact")
        m = rec["offline"]["measures"]
        check(all(k in m for k in ("on_corpus_500", "on_trace1_s1a_rows_1_68", "on_trace2_rows_1_68",
                                   "on_trace2_rows_69_116", "on_all_97_negative_rows_of_the_corpus")),
              "per-band metrics reported separately")
        check("DIAGNOSTIC ONLY" in rec["validation_note"], "validation MSE marked diagnostic")
        expect(lambda: E.run_stage(authorized_stage=E.AUTHORIZED_STAGE, progress=False), FileExistsError,
               "a second execution is refused")
        stage["offline_pass"] = rec["offline"]["all_binding_pass"]
        if E.JOB_DIR.exists():
            rr = json.loads((E.JOB_DIR / E.ROLLOUT_RECEIPT_NAME).read_text())
            E.assert_no_foreign_labels(rr, "materialised rollout receipt")
            check(rr["seven_gates"]["meaning"].startswith("closed-loop eligibility of REV4E."),
                  "the seven-gate prose names REV4E")
            check(rr["promotion"]["promoted"] is False, "not promoted")
            pg = rr["primary_gate"]; sm2 = rr["secondary_marker"]
            check(pg["verdict"] == ("PASS" if pg["observed_steps"] > 116 else "FAIL"), "primary gate rule applied")
            check(sm2["verdict"] == ("PASS" if sm2["observed"] >= 1 else "FAIL"), "secondary marker rule applied")
            check("DOES NOT ATTRIBUTE CAUSALITY" in rr["attribution_limits"],
                  "the rollout receipt carries the attribution limits")
            stage.update(survival_steps=pg["observed_steps"], primary_gate=pg["verdict"],
                         valid_cycles=sm2["observed"], secondary_marker=sm2["verdict"])
    print(json.dumps({"selftest": "PASS", "checks": CHECKS, **stage}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
