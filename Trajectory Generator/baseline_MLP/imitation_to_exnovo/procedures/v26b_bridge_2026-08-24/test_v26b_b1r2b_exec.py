"""Tests for the B1R2-B learning-rate screening arm. No fit of the experiment, no rollout,
no collection, no DAgger, no promotion.

All of this runs BEFORE any training is allowed to start.
"""
from __future__ import annotations
import inspect, io, json, sys, tokenize
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_b1r2b_exec as B  # noqa: E402
import v26b_b1r2a_exec as A  # noqa: E402
import v26b_b1r1_exec as R1  # noqa: E402
import v26b_b_exec as EX  # noqa: E402
import v26b_b0_masked35 as B0  # noqa: E402
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
    src = (HERE / "v26b_b1r2b_exec.py").read_text()

    # --- pre-registration ---------------------------------------------------------------------
    pre = B.verify_prereg()
    check(pre[B.PREREG_NAME] == B.PREREG_SHA, "the amendment matches its pinned hash")
    amd = json.loads((HERE / B.PREREG_NAME).read_text())
    check(amd["matrix_rows"] == 1 and [a["arm"] for a in amd["experiment_matrix"]] == ["B"],
          "the experiment matrix has exactly ONE row, arm B")
    check(float(amd["experiment_matrix"][0]["learning_rate"]) == B.LR_ARM_B == 1e-4,
          "arm B is the 1e-4 learning rate")
    check(amd["derived_from"]["sha256"] == A.PREREG_SHA,
          "the amendment pins the arm-A amendment it derives from")
    orig = B.PREREG_SHA
    B.PREREG_SHA = "0" * 64
    expect(B.verify_prereg, B.B1R2BError, "a tampered pre-registration must be refused")
    B.PREREG_SHA = orig

    # --- ONLY the learning rate changed, asserted programmatically ------------------------------
    only = B.verify_only_lr_changed()
    check(only["only_difference"] == {"learning_rate": {"arm_A": 3e-4, "arm_B": 1e-4}},
          "the single declared difference is the learning rate")
    check(only["fold_row_sets_identical_to_arm_A"] is True,
          "every fold holds out exactly the rows arm A held out")
    for name, attr in B.INHERITED_FROM_ARM_A.items():
        check(getattr(B, name) == getattr(A, attr), f"{name} equals arm A's {attr}")
    check(all(isinstance(v, str) for v in B.INHERITED_FROM_ARM_A.values()),
          "the inheritance map stores attribute NAMES, so values are read live and a divergence "
          "introduced after import is still caught")
    check(B.EPOCHS_MAX == A.EPOCHS_MAX_A == 1200 and B.PATIENCE == A.PATIENCE == 60
          and B.EMBARGO_ROWS == A.EMBARGO_ROWS == 20,
          "budget, patience and embargo are inherited unchanged")
    check(B.WAIT_RECONSTRUCTION is A.WAIT_RECONSTRUCTION is R1.WAIT_RECONSTRUCTION,
          "the WAIT gate is the same object all the way back to the B1R1 executor")
    check((B.GATE_FOLD_HELDOUT_RMSE_MAX, B.GATE_POOLED_VALIDATION_RMSE_MAX, B.GATE_FINAL_RMSE_MAX,
           B.GATE_FINAL_PER_JOINT_RMSE_MAX, B.GATE_FINAL_PER_JOINT_MAXABS_MAX)
          == (0.05, 0.03, 0.02, 0.03, 0.15),
          "every gate threshold is the unrelaxed B1R1 value")
    check(EX.J_LR == 3e-4 and EX.J_BATCH == 64 and EX.J_SEED == 123 and EX.J_CLIP_W == 1.0
          and EX.J_LOGSTD_W == 0.1 and EX.J_ANCHOR_W == 1e-5,
          "the pinned July module constants are untouched, J_LR included")
    saved = B.EPOCHS_MAX
    B.EPOCHS_MAX = 999
    expect(B.verify_only_lr_changed, B.B1R2BError, "a second changed field must be refused")
    B.EPOCHS_MAX = saved
    saved_lr = B.LR_ARM_B
    B.LR_ARM_B = B.LR_ARM_A
    expect(B.verify_only_lr_changed, B.B1R2BError, "arm B must actually differ from arm A")
    B.LR_ARM_B = saved_lr

    # --- the lr injection preserves fit_masked's default behaviour -------------------------------
    sig = inspect.signature(EX.fit_masked)
    check("lr" in sig.parameters and sig.parameters["lr"].default is None,
          "fit_masked takes an optional lr whose default is None")
    check(sig.parameters["lr"].kind is inspect.Parameter.KEYWORD_ONLY,
          "and it is keyword-only, so no positional call can change it by accident")
    fm = inspect.getsource(EX.fit_masked)
    check("lr = J_LR if lr is None else float(lr)" in fm,
          "omitting lr resolves to the pinned J_LR")
    check("torch.optim.Adam(params, lr=lr)" in fm, "the resolved lr reaches Adam")
    for tok in ("restore_logstd", "project_columns", "assert_projected",
                "J_CLIP_W", "J_LOGSTD_W", "J_ANCHOR_W", "J_BATCH", "J_SEED"):
        check(tok in fm, f"fit_masked still carries {tok}, so only lr became injectable")
    check("def fit_masked" not in src, "the arm never redefines fit_masked")

    # --- MECHANICAL proof that only lr changed: run the arm-B path at arm-A's lr ------------------
    init, _ = R1.load_parent()
    fd = B.folds_11()
    obs, act = fd["inp"]["obs"], fd["inp"]["act"]
    f6 = fd["folds"][6]                                     # the 11-row tail fold, cheapest
    _, rep_a = EX.fit_masked(init, obs, act, train_idx=f6["_train"], val_idx=f6["_val"],
                             epochs=4, patience=None)                    # default lr
    _, rep_b = EX.fit_masked(init, obs, act, train_idx=f6["_train"], val_idx=f6["_val"],
                             epochs=4, patience=None, lr=EX.J_LR)        # explicit, same value
    ca = [h["val_mse"] for h in rep_a["history"]]
    cb = [h["val_mse"] for h in rep_b["history"]]
    check(ca == cb,
          "BIT-IDENTICAL: passing lr=J_LR explicitly reproduces the default path exactly, so the "
          "injection changed nothing except making the value settable")
    check(float(rep_b["learning_rate"]) == EX.J_LR, "the report records the lr actually used")
    _, rep_c = EX.fit_masked(init, obs, act, train_idx=f6["_train"], val_idx=f6["_val"],
                             epochs=4, patience=None, lr=B.LR_ARM_B)
    cc = [h["val_mse"] for h in rep_c["history"]]
    # val_mse is recorded AFTER each epoch's 21 batches, so a different lr already separates the
    # curves at epoch 1. The shared starting point is B0 itself, which is lr-independent.
    check(cc != ca, "a different lr produces a different trajectory from the first recorded epoch")
    b0_val = EX.metrics(init, obs, act, f6["_val"])["aggregate_rmse"] ** 2
    check(all(c != b0_val for c in (ca[0], cc[0])),
          "both runs have already taken a step by the time epoch 1 is recorded")
    check(float(rep_c["learning_rate"]) == B.LR_ARM_B, "the arm-B lr is recorded as 1e-4")
    check(abs(cc[0] - b0_val) < abs(ca[0] - b0_val),
          "and the smaller lr has moved LESS far from B0 after one epoch, as a lower rate must")

    # --- the integrity control is a DATASET invariant, never a falsifier -------------------------
    ctl = B.verify_conflict_control()
    check(ctl["role"] == "integrity control, NOT a falsifier", "its role is declared")
    check(ctl["invariant_to_learning_rate"] is True, "and its invariance is declared")
    got = ctl["observed"]
    for k, v in B.CONFLICT_CONTROL_PINNED.items():
        check(got[k] == v, f"the control value {k} matches its pin EXACTLY ({v!r})")
    check(got["median_abs_target_delta_at_lt_0_15"] == 0.1670604795217514
          and got["cycle1_within_median_abs_target_delta"] == 0.04486663639545441
          and got["head_within_median_abs_target_delta"] == 0.0,
          "head-vs-cycle1 conflict 0.16706 against 0.04487 within cycle1 and 0.0 within head")
    check(got["min_pair_distance"] == 0.030914171900546916 > 1e-3,
          "the closest pair is 0.0309 apart, three orders of magnitude above the collision "
          "thresholds, so this is overlap and NOT an exact collision")
    saved_pin = dict(B.CONFLICT_CONTROL_PINNED)
    B.CONFLICT_CONTROL_PINNED["pairs_total"] = 1
    e = expect(B.verify_conflict_control, B.B1R2BError,
               "a moved control must INVALIDATE the run")
    check("INVALID" in str(e), "and the message says the run is invalid, not that a gate failed")
    B.CONFLICT_CONTROL_PINNED.clear(); B.CONFLICT_CONTROL_PINNED.update(saved_pin)
    fal_src = inspect.getsource(B.falsifiers)
    check("conflict" in fal_src and "INTEGRITY CONTROL" in fal_src,
          "the falsifier block states explicitly why the conflict is absent from it")
    check("F1_head_folds_above_gate_at_own_best" in fal_src
          and "F2_pooled_does_not_beat_arm_A_oracle" in fal_src,
          "exactly the two intervention-sensitive falsifiers are implemented")

    # --- FULL-CURVE convergence is NOT the selection rule's patience -----------------------------
    check(B.FULL_CURVE_STABLE_EPOCHS == 60 and B.FULL_CURVE_ARGMIN_MAX == 1140
          and B.FULL_CURVE_ARGMIN_MAX == B.EPOCHS_MAX - B.FULL_CURVE_STABLE_EPOCHS,
          "convergence needs the global minimum at or before epoch 1140, i.e. 1200 minus 60")
    # arm A's own shape: scan stopped at 297, global minimum at 1171 and LOWER
    v = [1.0] * 236 + [0.5] + [0.9] * 873 + [0.4] + [0.45] * 89
    sel_like = {"e_star": 237, "stopped_at_epoch": 297}
    d = B.convergence_diagnostics(v, sel_like)
    check(d["pooled_global_argmin"] == 1111 or d["pooled_global_argmin"] == 1110
          or v[d["pooled_global_argmin"] - 1] == 0.4,
          "the diagnostic finds the global minimum, not the epoch the scan stopped at")
    check(d["selection_scan_stopped_at_epoch"] == 297,
          "the scan's stopping epoch is reported SEPARATELY")

    # THE DISSOCIATION PROOF, on a curve with arm A's exact pathology: an early dip that exhausts
    # the selection patience, and a deeper GLOBAL minimum far beyond the headroom limit. The
    # selection is computed by the real rule, not hand-written.
    # the prefix must DESCEND, otherwise the flat run would exhaust the patience before the dip and
    # the curve would not exhibit the pathology it is meant to demonstrate
    patho = [1.0 - 0.004 * i for i in range(99)] + [0.5] + [0.9] * 1080 + [0.3] + [0.35] * 19
    check(len(patho) == 1200, "the pathological curve spans the whole budget")
    check(all(patho[i + 1] < patho[i] for i in range(98)),
          "its prefix descends strictly, so the stale counter keeps resetting up to the dip")
    sel_real = R1.pooled_best_epoch(patho, patience=B.PATIENCE, eps=B.POOLED_IMPROVEMENT_EPS)
    dp = B.convergence_diagnostics(patho, sel_real)
    check(sel_real["stopped_at_epoch"] is not None and sel_real["stopped_at_epoch"] == 160,
          "the SELECTION patience IS exhausted here, at epoch 160")
    check(sel_real["e_star"] == 100, "and the rule selects e*=100, the early dip")
    check(dp["pooled_global_argmin"] == 1181 > B.FULL_CURVE_ARGMIN_MAX,
          "yet the GLOBAL minimum of the full curve is at epoch 1181, beyond the 1140 headroom")
    check(dp["full_curve_converged_at_budget"] is False and dp["argmin_within_headroom"] is False,
          "so the full curve is NOT converged even though the selection patience was exhausted: "
          "the two quantities are proved DISSOCIATED, not merely reported in different fields")
    check(dp["falsifiers_are_readable"] is False,
          "and the falsifiers are therefore unreadable on such a curve")
    check(patho[dp["pooled_global_argmin"] - 1] < patho[sel_real["e_star"] - 1],
          "the missed minimum is also BETTER than the selected one, exactly as in arm A "
          "(1171 at 0.001697248 against 237 at 0.002309152)")
    check("NOT the selection rule's patience" in d["distinct_from_selection_patience"],
          "and the receipt states they must not be conflated")
    check("1171" in d["distinct_from_selection_patience"],
          "citing arm A, whose scan stopped at 297 while its true minimum was at 1171")
    # a curve whose minimum is past 1140 is NOT converged, even though a scan would have stopped
    late = [1.0 - 0.0001 * i for i in range(1200)]
    dl = B.convergence_diagnostics(late, {"e_star": 1200, "stopped_at_epoch": None})
    check(dl["pooled_global_argmin"] == 1200 and dl["argmin_within_headroom"] is False
          and dl["full_curve_converged_at_budget"] is False
          and dl["falsifiers_are_readable"] is False,
          "a minimum at the budget edge is NOT converged and the falsifiers become unreadable")
    early = [1.0] * 99 + [0.5] + [0.6] * 1100
    de = B.convergence_diagnostics(early, {"e_star": 100, "stopped_at_epoch": 160})
    check(de["pooled_global_argmin"] == 100 and de["argmin_within_headroom"] is True
          and de["no_improvement_in_following_60"] is True
          and de["full_curve_converged_at_budget"] is True
          and de["falsifiers_are_readable"] is True,
          "an early global minimum with a flat tail IS converged and the falsifiers apply")
    check(B.ARM_A_FULL_CURVE_ARGMIN == 1171
          and de["arm_A_full_curve_converged_at_budget"] is False,
          "MEASURED: under this definition arm A itself was NOT converged (1171 > 1140), which is "
          "recorded so arm A's own NO-GO is not over-read either")
    # the wording ban is checked over the WHOLE module source, not just one returned dict:
    # an earlier version of this check scanned only the diagnostic dict and missed an occurrence
    # that lived in the falsifier block.
    low = " ".join(src.lower().split())
    check("interior" not in low,
          "the module never uses 'interior' anywhere: the criterion is a full-curve argmin with "
          "headroom, and 'interior minimum' invited the conflation this diagnostic exists to stop")
    for blob in (json.dumps(d), json.dumps(dp), json.dumps(B.falsifiers(
            [{"fold": k, "validation_rows": 100, "own_best_rmse": 0.06,
              "curve_mse": [0.01] * 1200} for k in range(11)], 0.05, dp))):
        check("interior" not in blob.lower(),
              "and no receipt field it emits contains the word either")

    # --- falsifiers are unreadable when the curve has not converged ------------------------------
    curves = [{"fold": k, "validation_rows": 100, "own_best_rmse": 0.06,
               "curve_mse": [0.01] * 1200} for k in range(11)]
    f_ok = B.falsifiers(curves, 0.05, de)
    check(f_ok["readable"] is True and f_ok["hypothesis_falsified"] is True,
          "with a converged curve the falsifiers are read and both fire here")
    f_no = B.falsifiers(curves, 0.05, dl)
    check(f_no["readable"] is False and f_no["hypothesis_falsified"] is None
          and f_no["F1_head_folds_above_gate_at_own_best"]["fired"] is None
          and f_no["F2_pooled_does_not_beat_arm_A_oracle"]["fired"] is None,
          "with a non-converged curve NO falsifier may be read: they report None, not False")
    check(B.ARM_A_POOLED_SINGLE_EPOCH_ORACLE_RMSE == 0.04120
          and B.FALSIFIER_HEAD_FOLDS == (7, 9),
          "F2 compares against arm A's single-epoch oracle; F1 watches the two head folds")

    # --- the selection rule itself is untouched ---------------------------------------------------
    check("R1.pooled_best_epoch" in inspect.getsource(A.select_epoch)
          and "A.select_epoch" in inspect.getsource(B.select_epoch),
          "e* still comes from the July rule via arm A, not from a reimplementation")
    check("FULL_CURVE" not in inspect.getsource(B.select_epoch),
          "the convergence diagnostic cannot reach the binding selection")

    # --- 35D / mask / logstd / critic invariants ---------------------------------------------------
    st, _ = EX.fit_masked(init, obs, act, train_idx=f6["_train"], val_idx=None,
                          epochs=3, patience=None, lr=B.LR_ARM_B)
    check(int(np.asarray(st["pi.0.0.weight"]).shape[1]) == B0.ACTOR_WIDTH == 35,
          "the student stays 35D at the arm's learning rate; no 25D actor is built")
    B0.assert_masked_columns_zero(st, "arm B short fit")
    B0.assert_clock_columns_zero(st, "arm B short fit")
    B0.assert_no_masked_update(init, st)
    w1 = np.asarray(st["pi.0.0.weight"])
    check(np.all(w1[:, list(B0.CLOCK_COLUMNS)] == 0.0)
          and np.all(w1[:, list(B0.controller_columns())] == 0.0),
          "columns 0:2 and 25:35 are exactly zero, measured on the weights")
    check(np.array_equal(np.asarray(st["pi.1.weight"])[R.ACTION_DIM:],
                         np.asarray(init["pi.1.weight"])[R.ACTION_DIM:])
          and np.array_equal(np.asarray(st["pi.1.bias"])[R.ACTION_DIM:],
                             np.asarray(init["pi.1.bias"])[R.ACTION_DIM:]),
          "the log-std head stays byte-identical to B0 at the lower learning rate")
    check(not any("vf" in k or "critic" in k for k in st), "no critic tensor is ever present")

    # --- clip-loss is not gradient clipping ---------------------------------------------------------
    check(B.CLIP_LOSS_WEIGHT_J_CLIP_W == EX.J_CLIP_W == 1.0 and B.GRADIENT_CLIPPING == "none",
          "clip 1.0 is the action-bound loss weight; gradient clipping is none")
    check("torch.relu(torch.abs(means) - 1.0).square()" in fm,
          "the clip term acts on the OUTPUT inside the loss")
    check("clip_disambiguation" in src, "the receipt carries the disambiguation explicitly")

    # --- nothing may run, promote or leak without its gate ------------------------------------------
    identifiers = {t.string for t in tokenize.generate_tokens(io.StringIO(src).readline)
                   if t.type == tokenize.NAME}
    check("fit_masked" in identifiers and "folds_11" in identifiers,
          "the tokeniser sees identifiers that are genuinely present")
    check("DAgger" not in identifiers and "no DAgger" in src,
          "and not the word DAgger from the prose that denies it")
    for name in ("subprocess", "rollout_eval", "collect_teacher_dataset", "train_ppo",
                 "PPOConfig", "dagger", "DAgger", "run_dagger", "clip_grad_norm_"):
        check(name not in identifiers,
              f"the arm never references {name}: no rollout, collection, PPO, DAgger or grad clip")
    run_src = inspect.getsource(B.run)
    i_go = run_src.index('if verdict == "GO"')
    check("CANDIDATE_DIR" not in run_src[:i_go],
          "the candidate directory is touched ONLY inside the GO branch")
    check("staging.rename(CANDIDATE_DIR)" in run_src[i_go:],
          "promotion happens only after every binding gate has passed")
    check('failed = sorted(k for k, v in gates.items() if not v["pass"])' in run_src
          and 'verdict = "GO" if not failed else "NO-GO"' in run_src,
          "the verdict is derived from the gates, never asserted by hand")
    check('forbidden_conclusions' in run_src,
          "the receipt records the conclusions this arm may NOT support")
    check("INCONCLUSIVE SCREENING" in run_src,
          "a non-converged NO-GO is relabelled as an inconclusive screening in the receipt")
    check("clobber=False" in src, "receipts are written no-clobber, additive")
    check(B.DIAG_DIR.name == "b1r2b" and "diagnostics" in str(B.DIAG_DIR),
          "diagnostics land separately from candidates")
    expect(lambda: B.run(authorized_stage=None), B.B1R2BError,
           "running without the authorised stage token must be refused")
    expect(lambda: B.run(authorized_stage="V26B-B1R2A-BUDGET"), B.B1R2BError,
           "arm A's token must not authorise arm B")
    check("os.system" not in src and "from pathlib import Path" in src, "pathlib only, no shell")
    check("if False" not in src, "no dead conditional")

    print(json.dumps({"selftest": "PASS", "checks": CHECKS}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
