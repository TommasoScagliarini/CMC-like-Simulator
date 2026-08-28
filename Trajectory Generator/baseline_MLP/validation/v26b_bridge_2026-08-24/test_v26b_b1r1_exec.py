"""Fail-closed tests for the B1R1 executor. Synthetic fits only; no rollout, no collection."""
from __future__ import annotations
import inspect, json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_b1r1_exec as X  # noqa: E402
import v26b_b_exec as EX  # noqa: E402
import v26b_b0_masked35 as B0  # noqa: E402
import v26b_b1r1_loco_study as LO  # noqa: E402
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
    # --- pins and provenance -----------------------------------------------------------------------
    pins = X.verify_pins()
    check(set(pins) == {"amendment_b_masked35", "july_protocol_pin", "loco_comparator_addendum"},
          "the three governing documents are pinned")
    for attr in ("PIN_AMENDMENT", "PIN_JULY", "PIN_LOCO_ADDENDUM"):
        old = getattr(X, attr)
        try:
            setattr(X, attr, "0" * 64); expect(X.verify_pins, X.B1R1Error, f"tampered {attr} refused")
        finally: setattr(X, attr, old)
    init, parent = X.load_parent()
    check(parent["actor_digest"] == X.PIN_B0_ACTOR_DIGEST
          and "B0_35D_MASKED" in parent["module"], "the parent is B0 and only B0")
    check("B1_BASE35_MASKED" in X.FORBIDDEN_PARENTS and "REV4E" in X.FORBIDDEN_PARENTS,
          "the B1 NO-GO candidate and every diagnostic actor are forbidden parents")
    old = X.B0_DIR
    try:
        X.B0_DIR = Path("/x/candidates/B1_BASE35_MASKED")
        e = expect(X.load_parent, X.B1R1Error, "a B1 parent is refused by name")
        check("forbidden parent" in str(e), "the guard names the violation")
    finally: X.B0_DIR = old
    old2 = X.PIN_B0_ACTOR_DIGEST
    try:
        X.PIN_B0_ACTOR_DIGEST = "0" * 64
        expect(X.load_parent, X.B1R1Error, "a wrong parent digest is refused")
    finally: X.PIN_B0_ACTOR_DIGEST = old2

    # --- contract invariants on the parent -----------------------------------------------------------
    check(int(np.asarray(init["pi.0.0.weight"]).shape[1]) == 35, "actor width 35")
    B0.assert_masked_columns_zero(init); B0.assert_clock_columns_zero(init)
    check(list(B0.controller_columns()) == list(range(25, 35)), "the masked columns are 25:35")
    check(list(B0.CLOCK_COLUMNS) == [0, 1], "the clock columns are 0 and 1")
    check(list(LO.SUPPORT_COLUMNS) == list(range(2, 25)), "the 23D support is columns 2..24")

    # --- fit_masked is reused UNCHANGED ---------------------------------------------------------------
    src = (HERE / "v26b_b1r1_exec.py").read_text()
    check("EX.fit_masked(" in src and "def fit_masked" not in src,
          "the executor calls v26b_b_exec.fit_masked and does not redefine it")
    body = inspect.getsource(EX.fit_masked)
    for token in ("restore_logstd()", "project_columns()", "assert_projected()",
                  "J_CLIP_W", "J_LOGSTD_W", "J_ANCHOR_W"):
        check(token in body, f"fit_masked still carries {token}: its semantics are untouched")
    check((EX.J_EPOCHS, EX.J_BATCH, EX.J_LR, EX.J_PATIENCE, EX.J_CLIP_W, EX.J_LOGSTD_W,
           EX.J_ANCHOR_W, EX.J_SEED) == (400, 64, 3e-4, 60, 1.0, 0.1, 1e-5, 123),
          "the pinned July hyperparameters are unchanged")
    check(X.EPOCHS_MAX == 400 and X.PATIENCE == 60, "the executor uses the same budget and patience")

    # --- the pooled rule on synthetic curves ------------------------------------------------------------
    a = [1.0, 0.5, 0.4, 0.45, 0.46]
    b = [1.0, 0.6, 0.3, 0.35, 0.36]
    p = X.pooled_curve([a, b], [100, 300])
    check(np.allclose(p, [1.0, (0.5 * 100 + 0.6 * 300) / 400, (0.4 * 100 + 0.3 * 300) / 400,
                          (0.45 * 100 + 0.35 * 300) / 400, (0.46 * 100 + 0.36 * 300) / 400]),
          "the pooled curve is the row-weighted mean, not the plain mean")
    check(not np.allclose(p, np.mean([a, b], axis=0)), "and the weighting is material")
    expect(lambda: X.pooled_curve([a, b[:3]], [1, 1]), X.B1R1Error, "curves of unequal length refused")
    expect(lambda: X.pooled_curve([a, b], [1]), X.B1R1Error, "a missing row count is refused")
    expect(lambda: X.pooled_curve([a, b], [0, 1]), X.B1R1Error, "a zero row count is refused")
    expect(lambda: X.pooled_curve([[1.0, float("nan")]], [1]), X.B1R1Error, "non-finite MSE refused")
    expect(lambda: X.pooled_curve([], []), X.B1R1Error, "an empty curve set is refused")

    s = X.pooled_best_epoch([1.0, 0.5, 0.4, 0.45, 0.46], patience=2)
    check(s["e_star"] == 3 and s["stopped_at_epoch"] == 5, "the first minimum is selected and the stop recorded")
    # ties: equal values must NOT move e*, because the comparison is strict
    s2 = X.pooled_best_epoch([1.0, 0.3, 0.3, 0.3, 0.3], patience=3)
    check(s2["e_star"] == 2, "a tie keeps the EARLIEST minimising epoch, never a later equal one")
    s3 = X.pooled_best_epoch([1.0, 0.3, 0.3, 0.2999999999, 0.3], patience=10)
    check(s3["e_star"] == 2,
          "an improvement smaller than 1e-9 does NOT count, exactly as production's strict rule")
    s4 = X.pooled_best_epoch([1.0, 0.3, 0.3, 0.29, 0.3], patience=10)
    check(s4["e_star"] == 4, "an improvement larger than 1e-9 does count")
    s5 = X.pooled_best_epoch([0.5, 0.6, 0.7], patience=2)
    check(s5["e_star"] == 1 and s5["stopped_at_epoch"] == 3, "a monotone rise stops and keeps epoch 1")
    s6 = X.pooled_best_epoch([1.0, 0.9, 0.8, 0.7], patience=10)
    check(s6["e_star"] == 4 and s6["stopped_at_epoch"] is None,
          "a curve that never goes stale runs out of epochs and keeps the last minimum")
    expect(lambda: X.pooled_best_epoch([]), X.B1R1Error, "an empty pooled curve is refused")
    expect(lambda: X.pooled_best_epoch([1.0], patience=0), X.B1R1Error, "patience 0 is refused")
    check(X.POOLED_IMPROVEMENT_EPS == 1e-9, "the improvement epsilon matches production")

    # --- the two-pass reproduction is bit-identical ------------------------------------------------------
    rng = np.random.default_rng(2)
    n = 96
    obs = B0.apply_input_mask(rng.normal(size=(n, 35)).astype(np.float32))
    act = np.clip(rng.normal(size=(n, 2)) * 0.2, -1, 1).astype(np.float32)
    tr = np.arange(0, 64); va = np.arange(64, n)
    k = 5
    long_state, long_rep = EX.fit_masked(init, obs, act, train_idx=tr, val_idx=va,
                                         epochs=12, patience=None)
    short_state, short_rep = EX.fit_masked(init, obs, act, train_idx=tr, val_idx=None,
                                           epochs=k, patience=None)
    long_k, _ = EX.fit_masked(init, obs, act, train_idx=tr, val_idx=None, epochs=k, patience=None)
    check(all(np.array_equal(np.asarray(short_state[key]), np.asarray(long_k[key]))
              for key in short_state),
          "re-running for k epochs is deterministic and reproducible bit-for-bit")
    check(len(long_rep["history"]) == 12 and all(h["val_mse"] is not None for h in long_rep["history"]),
          "pass one records a validation MSE at every epoch with no early stop")
    check(short_rep["selection_mode"] == "fixed_final_epoch" and short_rep["best_epoch"] == k,
          "pass two runs exactly k epochs and keeps the final state")
    B0.assert_masked_columns_zero(short_state); B0.assert_clock_columns_zero(short_state)
    B0.assert_no_masked_update(init, short_state)
    check(True, "the masked and clock columns survive the fit exactly zero and unchanged")
    check(np.array_equal(np.asarray(short_state["pi.1.weight"])[2:],
                         np.asarray(init["pi.1.weight"])[2:]),
          "the log-std head is byte-identical to B0")

    # --- split, embargo, signatures ------------------------------------------------------------------------
    inp = X.build_inputs()
    check(inp["obs"].shape == (1500, 35) and inp["wait_rows"].size == 187,
          "1500 rows and the 187 WAIT rows are located")
    sp = inp["split"]
    check(sp["n_folds"] == 6 and sp["embargo_rows"] == 20, "six folds, embargo 20")
    cyc = [(f["name"], f["cycle"], f["local_lo"], f["local_hi"]) for f in sp["folds"]]
    check(cyc == [("minus020", 1, 187, 334), ("minus020", 2, 335, 488),
                  ("nominal", 1, 164, 314), ("nominal", 2, 315, 467),
                  ("plus020", 1, 144, 294), ("plus020", 2, 295, 447)],
          "the six held-out cycles are exactly the approved ranges")
    check(all(np.all(inp["raw"][f["_val"], LO.WAIT_COLUMN] <= 0.5) for f in sp["folds"]),
          "no held-out cycle contains a WAIT row")
    lost = {f["fold"]: f["wait_rows_lost_to_embargo"] for f in sp["folds"]}
    check(lost[0] == 20 and all(v == 0 for k2, v in lost.items() if k2 != 0),
          "only fold 0 loses WAIT rows to the embargo, and it loses exactly 20")
    check(sp["folds"][0]["wait_rows_in_train"] == 167, "fold 0 keeps 167 of the 187 WAIT rows")
    q = LO.quantify(sp, inp["raw"], inp["act"], inp["traj"])
    check(q["aggregate"]["min_temporal_distance"] == 21
          and not q["aggregate"]["any_temporal_leakage"]
          and not q["aggregate"]["any_signature_missing_from_train"],
          "embargo honoured, no leakage, every discrete signature covered")

    # --- gate thresholds -----------------------------------------------------------------------------------
    check((X.FOLD_HELDOUT_RMSE_MAX, X.POOLED_VALIDATION_RMSE_MAX, X.FINAL_RMSE_MAX,
           X.FINAL_PER_JOINT_RMSE_MAX, X.FINAL_PER_JOINT_MAXABS_MAX)
          == (0.05, 0.03, 0.02, 0.03, 0.15),
          "the B1 gates are carried over unchanged")
    w = X.WAIT_RECONSTRUCTION
    check((w["aggregate_rmse_max"], w["per_joint_rmse_max"], w["per_joint_max_abs_max"])
          == (0.02, 0.03, 0.15), "the WAIT gate reuses the all-data thresholds")
    check("NOT a generalisation" in w["statement"] or "NOT a "
          in w["statement"], "the WAIT gate declares it is not a generalisation result")
    check("wait_reconstruction_NOT_A_GENERALISATION_GATE" in src,
          "the receipt key itself refuses to call the WAIT gate a heldout")

    # --- preflight and token guard ---------------------------------------------------------------------------
    pre = X.preflight()
    check(pre["verdict"] in ("GO", "BLOCKED") and pre["contract"]["actor_width"] == 35,
          "the preflight reports the contract")
    check(pre["functional_equivalence_25"]["bit_identical"],
          "B0 is still bit-identical to its 25-column subnetwork before the fit")
    for bad in (None, "V26B-B1-BASE-FIT-MASKED35", "v26b-b1r1-base-fit-loco"):
        e = expect(lambda b=bad: X.run(authorized_stage=b), X.B1R1Error, f"token {bad!r} refused")
        check("V26B-B1R1-BASE-FIT-LOCO" in str(e), "the guard names this stage's token")

    # --- prohibitions ------------------------------------------------------------------------------------------
    check(not any(t in src for t in ("subprocess", "rollout_eval", "collect_teacher_dataset",
                                     "train_ppo", "PPOConfig", "markov")),
          "the executor cannot roll out, collect, run PPO or enter the Markov phase")
    check("os.system" not in src and "os.sep" not in src and "from pathlib import Path" in src,
          "pathlib only, no shell, no os-specific path handling")
    check("if False" not in src, "no dead conditional remains in the executor")
    check(all(m not in sys.modules for m in B0.SUPERSEDED_MODULES),
          "no superseded 25D module is loaded")

    print(json.dumps({"selftest": "PASS", "checks": CHECKS}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
