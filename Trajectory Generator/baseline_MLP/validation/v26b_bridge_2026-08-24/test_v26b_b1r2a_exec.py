"""Tests for the B1R2-A budget arm. No fit, no rollout, no collection, no promotion.

Everything here runs BEFORE any training is allowed to start.
"""
from __future__ import annotations
import inspect, json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_b1r2a_exec as A  # noqa: E402
import v26b_b1r1_exec as R1  # noqa: E402
import v26b_b1r2_diagnostics as D  # noqa: E402
import v26b_b_exec as EX  # noqa: E402
import v26b_b0_masked35 as B0  # noqa: E402
import v26b_b1r1_loco_study as LO  # noqa: E402
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
    src = (HERE / "v26b_b1r2a_exec.py").read_text()
    run_src = inspect.getsource(A.run)

    # --- pre-registration is pinned and immutable ------------------------------------------------
    pre = A.verify_prereg()
    check(pre[A.PREREG_NAME] == A.PREREG_SHA and pre[A.ADDENDUM_NAME] == A.ADDENDUM_SHA,
          "the amendment and its addendum match their pinned hashes")
    amd = json.loads((HERE / A.PREREG_NAME).read_text())
    check(amd["matrix_rows"] == 1 and [a["arm"] for a in amd["experiment_matrix"]] == ["A"],
          "the experiment matrix has exactly ONE row, arm A")
    check(int(amd["experiment_matrix"][0]["epochs_max"]) == A.EPOCHS_MAX_A == 1200,
          "arm A is the 1200-epoch budget")
    add = json.loads((HERE / A.ADDENDUM_NAME).read_text())
    check(add["amends_sha256"] == A.PREREG_SHA,
          "the addendum pins the exact amendment it clarifies, so it cannot drift from it")
    check(add["correction_1_baseline_qualification"]["what_changed_and_why"]["classification"]
          == "METHODOLOGICAL PREREQUISITE, not a second experimental arm",
          "the validation-design correction is documented as a prerequisite, not a second arm")
    check("6 folds / 910" in add["correction_1_baseline_qualification"]["correct_statement"]
          or "910" in json.dumps(add["correction_1_baseline_qualification"]),
          "the addendum states explicitly that B1R1 was 6 folds / 910 rows")
    orig = A.PREREG_SHA
    A.PREREG_SHA = "0" * 64
    expect(A.verify_prereg, A.B1R2AError, "a tampered pre-registration must be refused")
    A.PREREG_SHA = orig

    # --- gates are imported from B1R1 and NOT relaxed ---------------------------------------------
    g = A.verify_gates_not_relaxed()
    check(A.GATE_FOLD_HELDOUT_RMSE_MAX == R1.FOLD_HELDOUT_RMSE_MAX == 0.05
          and A.GATE_POOLED_VALIDATION_RMSE_MAX == R1.POOLED_VALIDATION_RMSE_MAX == 0.03
          and A.GATE_FINAL_RMSE_MAX == R1.FINAL_RMSE_MAX == 0.02
          and A.GATE_FINAL_PER_JOINT_RMSE_MAX == R1.FINAL_PER_JOINT_RMSE_MAX == 0.03
          and A.GATE_FINAL_PER_JOINT_MAXABS_MAX == R1.FINAL_PER_JOINT_MAXABS_MAX == 0.15,
          "every threshold equals the B1R1 pin, unrelaxed")
    check(A.WAIT_RECONSTRUCTION is R1.WAIT_RECONSTRUCTION,
          "the WAIT reconstruction gate is the B1R1 object itself, not a copy")
    check(A.PATIENCE is R1.PATIENCE and A.POOLED_IMPROVEMENT_EPS is R1.POOLED_IMPROVEMENT_EPS,
          "patience 60 and eps 1e-9 are inherited, not restated")
    for tok in ("R1.FOLD_HELDOUT_RMSE_MAX", "R1.POOLED_VALIDATION_RMSE_MAX", "R1.FINAL_RMSE_MAX",
                "R1.FINAL_PER_JOINT_RMSE_MAX", "R1.FINAL_PER_JOINT_MAXABS_MAX",
                "R1.WAIT_RECONSTRUCTION", "R1.PATIENCE"):
        check(tok in src, f"{tok} is imported rather than hardcoded")
    orig_gate = A.GATE_FINAL_RMSE_MAX
    A.GATE_FINAL_RMSE_MAX = 0.99
    expect(A.verify_gates_not_relaxed, A.B1R2AError, "a relaxed gate must be refused")
    A.GATE_FINAL_RMSE_MAX = orig_gate

    # --- the eleven folds ---------------------------------------------------------------------------
    fd = A.folds_11()
    check(fd["n_folds"] == 11 and fd["n_complete_cycle_folds"] == 8 and fd["n_tail_folds"] == 3,
          "11 folds = 8 structurally complete cycles + 3 tails")
    check(fd["heldout_rows_total"] == 1313, "the folds hold out all 1313 non-WAIT rows")
    val_all = np.concatenate([f["_val"] for f in fd["folds"]])
    check(len(val_all) == len(np.unique(val_all)) == 1313,
          "every non-WAIT row is held out EXACTLY once; none twice")
    raw = fd["inp"]["raw"]
    check(not any(np.any(raw[f["_val"], LO.WAIT_COLUMN] > 0.5) for f in fd["folds"]),
          "no fold ever validates a WAIT row")
    check(all(f["wait_rows_in_train"] > 0 for f in fd["folds"]),
          "WAIT rows stay in training in every fold, reconstruction-only")
    traj = fd["inp"]["traj"]
    for f in fd["folds"]:
        v, t = set(f["_val"].tolist()), set(f["_train"].tolist())
        check(not (v & t), f"fold {f['fold']} never trains on a row it validates")
        dmin = min((abs(int(i) - int(j)) for i in f["_val"] for j in f["_train"]
                    if traj[i] == traj[j]), default=None)
        check(dmin is None or dmin > A.EMBARGO_ROWS,
              f"fold {f['fold']} honours the {A.EMBARGO_ROWS}-row embargo")
    cov = D.full_coverage(embargo=A.EMBARGO_ROWS)
    check(cov["n_folds"] == fd["n_folds"]
          and cov["n_complete_cycle_folds"] == fd["n_complete_cycle_folds"]
          and cov["n_tail_folds"] == fd["n_tail_folds"]
          and [c["validation_rows"] for c in cov["folds"]]
              == [f["validation_rows"] for f in fd["folds"]],
          "folds_11 matches the audited Rev1 coverage design fold for fold; it cannot drift")

    # --- the pooled curve IS the out-of-fold MSE over all 1313 non-WAIT rows -----------------------
    act = np.asarray(fd["inp"]["act"], dtype=np.float64)
    rng = np.random.default_rng(7)
    pred = act + rng.normal(scale=0.05, size=act.shape)          # an arbitrary prediction field
    per_fold, rows = [], []
    for f in fd["folds"]:
        d = pred[f["_val"]] - act[f["_val"]]
        per_fold.append([float((d ** 2).mean())])
        rows.append(f["validation_rows"])
    pooled = R1.pooled_curve(per_fold, rows)[0]
    direct = float(((pred[val_all] - act[val_all]) ** 2).mean())
    check(abs(pooled - direct) < 1e-12,
          "MEASURED: the row-weighted pooled MSE equals the out-of-fold MSE over all 1313 non-WAIT "
          "rows exactly, so the selection rule ignores no row and double-counts none")
    check(abs(sum(rows) - 1313) < 1e-9, "and the weights sum to 1313")

    # --- the selection rule: July's, reused unchanged ---------------------------------------------
    check(A.select_epoch.__doc__ and "R1.pooled_best_epoch" in inspect.getsource(A.select_epoch),
          "e* comes from the B1R1 rule function, not a reimplementation")
    def curves_from(vals):
        return [{"curve_mse": list(vals), "validation_rows": 1}]
    s = A.select_epoch(curves_from([5.0, 4.0, 3.0] + [3.0] * 1197), budget=1200)
    check(s["e_star"] == 3, "e* is the FIRST epoch attaining the minimum, ties never move it later")
    s = A.select_epoch(curves_from([5.0, 4.0, 3.0] + [3.0 - 1e-12] * 1197), budget=1200)
    check(s["e_star"] == 3,
          "an improvement smaller than eps does not reset the counter nor move e*")
    v = [1.0 / (i + 1) for i in range(1200)]
    s = A.select_epoch(curves_from(v), budget=1200)
    check(s["e_star"] == 1200 and s["stopped_at_epoch"] is None,
          "a curve still improving at the budget edge exhausts no patience and reports it")
    v = [1.0] * 5 + [0.5] + [1.0] * 1194
    s = A.select_epoch(curves_from(v), budget=1200)
    check(s["e_star"] == 6 and s["stopped_at_epoch"] == 66,
          f"the 60-epoch patience stops the curve 60 epochs after its minimum, got {s}")
    check(abs(s["pooled_rmse_at_e_star"] - 0.5 ** 0.5) < 1e-12,
          "the pooled RMSE at e* is the square root of the pooled MSE, comparable to the 0.03 gate")

    # --- the budget readout is DIAGNOSTIC at 400 and BINDING at 1200 --------------------------------
    v = [1.0 - 0.0005 * i for i in range(1200)]                   # improves throughout
    ro = A.budget_readout(curves_from(v))
    check(ro["at_400"]["e_star"] == 400 and ro["at_1200"]["e_star"] == 1200,
          "truncating at 400 and running to 1200 select different epochs on the same curve")
    check(ro["budget_was_binding"] is True and ro["e_star_moved_past_400"] is True,
          "a curve still improving past 400 is reported as a binding budget")
    check(ro["budget_still_insufficient"] is True and ro["patience_exhausted_within_1200"] is False,
          "and, never having exhausted its patience, it is reported as STILL insufficient")
    v = [1.0] * 5 + [0.5] + [1.0] * 1194                          # minimum at 6, well inside 400
    ro = A.budget_readout(curves_from(v))
    check(ro["at_400"]["e_star"] == ro["at_1200"]["e_star"] == 6
          and ro["budget_was_binding"] is False
          and ro["patience_exhausted_within_1200"] is True,
          "an interior minimum gives the same e* at both budgets and is reported as not binding")
    check(A.BASELINE_BUDGET_FOR_READOUT == R1.EPOCHS_MAX == 400,
          "the diagnostic truncation is exactly July's 400, inherited")
    check("diagnostic_only" in ro and "binding_selection" in ro,
          "the receipt records which arm binds and which is diagnostic")
    # the 400 truncation can never influence the binding selection
    sel_alone = A.select_epoch(curves_from(v), budget=A.EPOCHS_MAX_A)
    check(sel_alone["e_star"] == ro["at_1200"]["e_star"],
          "e* is a function of the 1200 curve alone; the 400 readout cannot change it")
    check("budget_readout" not in run_src.split("gates = {")[0].split("sel = select_epoch")[1],
          "the binding selection is not derived from the diagnostic readout")

    # --- fit_masked is reused with UNCHANGED semantics ---------------------------------------------
    check("def fit_masked" not in src, "the executor never redefines fit_masked")
    check("EX.fit_masked(" in src, "it calls the pinned fit_masked")
    fm = inspect.getsource(EX.fit_masked)
    for tok in ("restore_logstd", "project_columns", "assert_projected",
                "J_CLIP_W", "J_LOGSTD_W", "J_ANCHOR_W"):
        check(tok in fm, f"fit_masked still carries {tok}")

    # --- twin bit-identity for the first epochs, on a REAL fold ------------------------------------
    init, _ = R1.load_parent()
    obs, acts = fd["inp"]["obs"], fd["inp"]["act"]
    f0 = fd["folds"][6]                                           # the smallest fold, 11 rows
    _, long_rep = EX.fit_masked(init, obs, acts, train_idx=f0["_train"], val_idx=f0["_val"],
                                epochs=6, patience=None)
    _, short_rep = EX.fit_masked(init, obs, acts, train_idx=f0["_train"], val_idx=f0["_val"],
                                 epochs=3, patience=None)
    long_curve = [h["val_mse"] for h in long_rep["history"]]
    short_curve = [h["val_mse"] for h in short_rep["history"]]
    check(len(long_curve) == 6 and len(short_curve) == 3,
          "no early stop: exactly the requested number of epochs is recorded")
    check(all(a == b for a, b in zip(long_curve[:3], short_curve)),
          "BIT-IDENTICAL: the first 3 epochs of a 6-epoch run reproduce a 3-epoch run exactly, so "
          "materialising the curve and applying the rule afterwards cannot change e*")
    st3, rep3 = EX.fit_masked(init, obs, acts, train_idx=f0["_train"], val_idx=None,
                              epochs=3, patience=None)
    check(rep3["selection_mode"] == "fixed_final_epoch",
          "pass two runs in fixed mode, keeping the epoch-e* weights rather than a best")
    m3 = EX.metrics(st3, obs, acts, f0["_val"])
    # the two paths evaluate the same weights in float32, one through torch and one through numpy,
    # so they agree to a few ulp rather than bit-exactly. Measured: 3.2e-08 relative, ~0.3 ulp of
    # float32. The trajectory identity itself is proved bit-exactly by the curve-prefix check above.
    rel = abs(m3["aggregate_rmse"] ** 2 - short_curve[2]) / short_curve[2]
    check(rel < 1e-6,
          f"the pass-two state at epoch k scores the pass-one validation MSE at epoch k to within "
          f"float32 rounding (measured relative difference {rel:.3g})")
    check(np.finfo(np.float32).eps > rel,
          "and that difference is smaller than one float32 epsilon, so it is rounding, not drift")
    _, repv = EX.fit_masked(init, obs, acts, train_idx=f0["_train"], val_idx=f0["_val"],
                            epochs=3, patience=None)
    stv = _
    check(repv["best_epoch"] != 3,
          "in validation mode fit_masked returns its BEST epoch, not the last one")
    check(not all(np.array_equal(np.asarray(stv[k]), np.asarray(st3[k])) for k in st3),
          "which is exactly why pass two re-runs in fixed mode: the pass-one return value is the "
          "wrong state to evaluate at e*")

    # --- 35D / mask / logstd / critic invariants ----------------------------------------------------
    check(int(np.asarray(st3["pi.0.0.weight"]).shape[1]) == B0.ACTOR_WIDTH == 35,
          "the student stays 35D; no 25D actor is ever built")
    B0.assert_masked_columns_zero(st3, "short fit")
    B0.assert_clock_columns_zero(st3, "short fit")
    B0.assert_no_masked_update(init, st3)
    check(True, "columns 0:2 and 25:35 are exactly zero and never updated")
    w1 = np.asarray(st3["pi.0.0.weight"])
    check(np.all(w1[:, list(B0.CLOCK_COLUMNS)] == 0.0)
          and np.all(w1[:, list(B0.controller_columns())] == 0.0),
          "measured directly on the weights, not only via the assertion helpers")
    check(np.array_equal(np.asarray(st3["pi.1.weight"])[R.ACTION_DIM:],
                         np.asarray(init["pi.1.weight"])[R.ACTION_DIM:])
          and np.array_equal(np.asarray(st3["pi.1.bias"])[R.ACTION_DIM:],
                             np.asarray(init["pi.1.bias"])[R.ACTION_DIM:]),
          "the log-std head stays byte-identical to B0")
    check(not any("vf" in k or "critic" in k for k in st3),
          "no critic tensor is ever present in the state")
    check(len(list(B0.controller_columns())) == 10 and list(B0.CLOCK_COLUMNS) == [0, 1],
          "the mask covers the 10 controller columns and the 2 clock columns")

    # --- clip-loss is NOT gradient clipping ----------------------------------------------------------
    check(A.CLIP_LOSS_WEIGHT_J_CLIP_W == EX.J_CLIP_W == 1.0,
          "clip 1.0 is J_CLIP_W, the action-bound penalty weight")
    check(A.GRADIENT_CLIPPING == "none", "gradient clipping is explicitly none")
    check("clip_grad" not in src and "clip_grad" not in fm,
          "no gradient-norm or gradient-value clipping exists in the executor or in fit_masked")
    check("torch.relu(torch.abs(means) - 1.0).square()" in fm,
          "the clip term is relu(|mean| - 1)^2 on the OUTPUT, inside the loss")
    check("clip_disambiguation" in src and "gradient_clipping" in src,
          "the receipt carries both fields explicitly so they can never be confused")
    check("W1.grad.mul_(col_mask)" in fm,
          "the only gradient manipulation is the column mask, which is masking, not clipping")

    # --- nothing may run, promote or leak without its gate ------------------------------------------
    # Capability is about IDENTIFIERS, not prose. The module's own text says "no DAgger", so any
    # substring test would flag that sentence instead of a real call. Tokenising and keeping only
    # NAME tokens excludes every string literal and comment by construction.
    import io, tokenize
    identifiers = {t.string for t in tokenize.generate_tokens(io.StringIO(src).readline)
                   if t.type == tokenize.NAME}
    check("fit_masked" in identifiers and "folds_11" in identifiers,
          "the tokeniser sees the identifiers that are genuinely present")
    check("DAgger" not in identifiers and "no DAgger" in src,
          "and it does NOT see the word DAgger from the prose that denies it")
    for name in ("subprocess", "rollout_eval", "collect_teacher_dataset", "train_ppo",
                 "PPOConfig", "dagger", "DAgger", "run_dagger", "clip_grad_norm_"):
        check(name not in identifiers,
              f"the executor never references the identifier {name}: it cannot roll out, collect, "
              "run PPO, clip gradients or start DAgger")
    i_go = run_src.index('if verdict == "GO"')
    check("CANDIDATE_DIR" not in run_src[:i_go],
          "the candidate directory is touched ONLY inside the GO branch")
    check("staging.rename(CANDIDATE_DIR)" in run_src[i_go:],
          "promotion happens only after every binding gate has passed")
    check('failed = sorted(k for k, v in gates.items() if not v["pass"])' in run_src
          and 'verdict = "GO" if not failed else "NO-GO"' in run_src,
          "the verdict is derived from the gates, never asserted by hand")
    check("clobber=False" in src, "receipts are written no-clobber, additive")
    check(A.DIAG_DIR.name == "b1r2a" and "diagnostics" in str(A.DIAG_DIR),
          "the diagnostic artefacts land under diagnostics/, separate from candidates/")
    check("if a.dry" in src and "if a.preflight" in src and "--authorized-stage" in src,
          "a dry mode and a preflight exist, and the fit refuses to start without its stage token")
    expect(lambda: A.run(authorized_stage=None), A.B1R2AError,
           "running without the authorised stage token must be refused")
    expect(lambda: A.run(authorized_stage="V26B-B1R1-BASE-FIT-LOCO"), A.B1R2AError,
           "another stage's token must not authorise this one")
    check("os.system" not in src and "from pathlib import Path" in src,
          "pathlib only, no shell")
    check("if False" not in src, "no dead conditional")

    print(json.dumps({"selftest": "PASS", "checks": CHECKS}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
