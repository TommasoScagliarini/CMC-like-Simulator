"""V26B B1R2-B - the learning-rate SCREENING arm. Derived from B1R2-A, one field changed.

ONE experimental variable: lr 3e-4 -> 1e-4.  Everything else - parent, mask, dataset, folds,
selection rule, budget, patience, gates, loss weights, seed, batch, and the absence of gradient
clipping - is inherited from v26b_b1r2a_exec and ASSERTED EQUAL PROGRAMMATICALLY, not by prose.

`clip 1.0` is J_CLIP_W, an ACTION-BOUND PENALTY ON THE LOSS: relu(|mean| - 1)^2 on the network
output before backward().  It is NOT gradient clipping, which remains absent everywhere.

THREE ARCHITECT CORRECTIONS ARE BINDING HERE:

  1. The head-vs-cycle1 target conflict is computed from the DATASET ALONE and is invariant to the
     learning rate.  It is an INTEGRITY / SANITY CONTROL, never a falsifier.  If it moves, the data
     pipeline has broken and the run is INVALIDATED whatever the gates say.

  2. With the learning rate cut threefold and the budget FIXED at 1200 epochs, this arm covers less
     optimisation distance than arm A.  A NO-GO is therefore confounded between 'the sharpness
     hypothesis is false' and 'the optimisation did not arrive'.  CONCLUDING A LIMIT OF THE
     OBSERVATION CONTRACT FROM THIS ARM IS FORBIDDEN, whatever the outcome.  A full-curve
     convergence diagnostic is recorded so the result can be read, and a non-converged curve must
     be reported as an inconclusive screening, never as an informative NO-GO.

  3. THE SELECTION RULE'S PATIENCE IS NOT CONVERGENCE.  Exhausting the July scan means the rule
     stopped looking, not that the curve settled.  Arm A is the proof: its scan stopped at epoch
     297 while the global minimum of its full 1200-epoch pooled curve was at 1171 and was LOWER.
     Convergence is therefore defined separately, on the FULL pooled curve: converged iff the
     global minimum falls at or before epoch 1140 and no epoch in the following 60 improves on it
     by more than 1e-9.  An argmin later than 1140 means NOT CONVERGED at this budget.  The binding
     selection of e* is unchanged and remains July's patience-60 rule.

No rollout, no collection, no DAgger, no Markov phase, no PPO.  A candidate is promoted only if
every binding gate passes AND the integrity control matches exactly.

Cross-platform: pathlib only, no shell, no os-specific path handling.
"""

from __future__ import annotations

import argparse
import json
import pickle
import shutil
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import v26b_b_exec as EX  # noqa: E402
import v26b_b0_masked35 as B0  # noqa: E402
import v26b_b1r1_exec as R1  # noqa: E402
import v26b_b1r1_loco_study as LO  # noqa: E402
import v26b_b1r2_diagnostics as D  # noqa: E402
import v26b_b1r2a_exec as A  # noqa: E402
import v26b_anchors as VA  # noqa: E402
import v26b_student as VS  # noqa: E402
import f0_common as C  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_refit as RF  # noqa: E402


class B1R2BError(RuntimeError):
    pass


STAGE = "V26B-B1R2B-LR"

PREREG_NAME = "v26b_amendment_b1r2b_lr.json"
PREREG_SHA = "0bf106e79ba4def044cfabb5ad7e264c238a9983284d5b43a7a93778fcfd10ce"

# ================================================================ THE ONE VARIABLE ===============
LR_ARM_B = 1e-4
LR_ARM_A = EX.J_LR                                  # 3e-4, the pinned July constant, untouched

# ---- everything else is INHERITED from arm A, never restated -----------------------------------
EPOCHS_MAX = A.EPOCHS_MAX_A
PATIENCE = A.PATIENCE
POOLED_IMPROVEMENT_EPS = A.POOLED_IMPROVEMENT_EPS
EMBARGO_ROWS = A.EMBARGO_ROWS
GATE_FOLD_HELDOUT_RMSE_MAX = A.GATE_FOLD_HELDOUT_RMSE_MAX
GATE_POOLED_VALIDATION_RMSE_MAX = A.GATE_POOLED_VALIDATION_RMSE_MAX
GATE_FINAL_RMSE_MAX = A.GATE_FINAL_RMSE_MAX
GATE_FINAL_PER_JOINT_RMSE_MAX = A.GATE_FINAL_PER_JOINT_RMSE_MAX
GATE_FINAL_PER_JOINT_MAXABS_MAX = A.GATE_FINAL_PER_JOINT_MAXABS_MAX
WAIT_RECONSTRUCTION = A.WAIT_RECONSTRUCTION
CLIP_LOSS_WEIGHT_J_CLIP_W = A.CLIP_LOSS_WEIGHT_J_CLIP_W
GRADIENT_CLIPPING = A.GRADIENT_CLIPPING

# arm A results this arm is measured against, pinned from its receipt
ARM_A_RECEIPT_SHA = "a4ebd8d6f023c8749212fe5c24c7fae36675ef2335c3b7a097981494b9262f4e"
ARM_A_POOLED_SINGLE_EPOCH_ORACLE_RMSE = 0.04120
ARM_A_PATIENCE_EXHAUSTED_AT = 297          # the SELECTION scan stopped here
ARM_A_FULL_CURVE_ARGMIN = 1171             # the FULL curve's global minimum was here, and lower
FALSIFIER_HEAD_FOLDS = (7, 9)

# --- FULL-CURVE convergence diagnostic, distinct from the selection rule's patience -------------
# The selection rule for e* is UNCHANGED. This is a separate, purely diagnostic criterion on the
# complete pooled curve: converged iff the global minimum lands with at least 60 epochs of budget
# left after it and nothing in those 60 improves on it by more than 1e-9.
FULL_CURVE_STABLE_EPOCHS = 60
FULL_CURVE_ARGMIN_MAX = EPOCHS_MAX - FULL_CURVE_STABLE_EPOCHS      # 1140

DIAG_DIR = VA.OUT_ROOT / "diagnostics" / "b1r2b"
CANDIDATE_DIR = VA.OUT_ROOT / "candidates" / "B1R2B_BASE35_LR1E4"
RECEIPT_NAME = "v26b_b1r2b_receipt.json"
MODULE_DIR_NAME = "rl_module"
ACTOR_LABEL = "B1R2B_BASE35_LR1E4"

# ============================== integrity control: a DATASET invariant, NOT a falsifier ==========

CONFLICT_CONTROL_PINNED = {
    "head_swing_rows": 188,
    "cycle1_swing_rows": 262,
    "pairs_total": 49256,
    "pairs_closer_than_0_15": 92,
    "median_abs_target_delta_at_lt_0_15": 0.1670604795217514,
    "median_abs_target_delta_all_pairs": 0.4387725368142128,
    "min_pair_distance": 0.030914171900546916,
    "cycle1_within_pairs_lt_0_15": 46,
    "cycle1_within_median_abs_target_delta": 0.04486663639545441,
    "head_within_pairs_lt_0_15": 54,
    "head_within_median_abs_target_delta": 0.0,
}
CONFLICT_RADIUS = 0.15
SWING_SIGNATURE = (0.0, 0.0, 1.0, 1.0, 0.0, 0.5)
REGIME_COLUMNS = (14, 15, 16)
REGIME_VALUES = (0.0, 1.0, 0.0)


def conflict_control() -> dict[str, Any]:
    """Head-swing vs cycle1-swing target conflict, computed from the DATASET ONLY.

    It contains no model output, so it cannot move under a learning-rate change.  It is therefore
    an integrity control: a mismatch means the dataset, caches, labels, ordering or row
    construction have changed, and the run must be invalidated - it says nothing about the
    hypothesis.  This is the architect's correction 1, and the reason this quantity was removed
    from the falsifier list.
    """
    g = D.row_groups()
    inp = g["inp"]
    raw, act, traj = inp["raw"], np.asarray(inp["act"], dtype=np.float64), inp["traj"]
    sig = LO.discrete_signature(raw)
    X = raw[:, list(LO.SUPPORT_COLUMNS)].astype(np.float64)
    in_regime = np.all(np.stack([raw[:, c] for c in REGIME_COLUMNS], 1)
                       == np.array(REGIME_VALUES), axis=1)
    heads: set[int] = set()
    for s in g["uncovered_segments"]:
        if s["structure"] == "complete_cycle":
            heads |= set(range(s["global_lo"], s["global_hi"] + 1))
    covered = set(int(i) for i in g["cycle"])
    hs = sorted(i for i in range(len(sig)) if sig[i] == SWING_SIGNATURE and i in heads)
    c1 = sorted(i for i in range(len(sig)) if sig[i] == SWING_SIGNATURE
                and i in covered and in_regime[i])

    def cross(A_: Sequence[int], B_: Sequence[int], same: bool) -> tuple[np.ndarray, np.ndarray]:
        d: list[float] = []
        t: list[float] = []
        for ii, a in enumerate(A_):
            for b in (B_[ii + 1:] if same else B_):
                if traj[a] == traj[b] and abs(a - b) <= EMBARGO_ROWS:
                    continue
                d.append(float(np.linalg.norm(X[a] - X[b])))
                t.append(float(np.max(np.abs(act[a] - act[b]))))
        return np.asarray(d), np.asarray(t)

    d, t = cross(hs, c1, False)
    m = d < CONFLICT_RADIUS
    dc, tc = cross(c1, c1, True)
    mc = dc < CONFLICT_RADIUS
    dh, th = cross(hs, hs, True)
    mh = dh < CONFLICT_RADIUS
    return {"head_swing_rows": len(hs), "cycle1_swing_rows": len(c1),
            "pairs_total": int(d.size), "pairs_closer_than_0_15": int(m.sum()),
            "median_abs_target_delta_at_lt_0_15": float(np.median(t[m])),
            "median_abs_target_delta_all_pairs": float(np.median(t)),
            "min_pair_distance": float(d.min()),
            "cycle1_within_pairs_lt_0_15": int(mc.sum()),
            "cycle1_within_median_abs_target_delta": float(np.median(tc[mc])),
            "head_within_pairs_lt_0_15": int(mh.sum()),
            "head_within_median_abs_target_delta": float(np.median(th[mh]))}


def verify_conflict_control() -> dict[str, Any]:
    """Exact equality. A mismatch INVALIDATES the run; it never falsifies the hypothesis."""
    got = conflict_control()
    bad = {k: {"pinned": v, "observed": got[k]} for k, v in CONFLICT_CONTROL_PINNED.items()
           if got[k] != v}
    if bad:
        raise B1R2BError(f"INTEGRITY CONTROL FAILED - the data pipeline changed, the run is "
                         f"INVALID and its verdict must not be interpreted: {bad}")
    return {"status": "MATCHES EXACTLY", "role": "integrity control, NOT a falsifier",
            "invariant_to_learning_rate": True, "observed": got}


# ================================ only-lr-changed, asserted programmatically =====================

# local name -> the arm-A attribute it must equal. The local value is read LIVE from this module's
# globals at verification time, never snapshotted, so a divergence introduced after import is still
# caught.
INHERITED_FROM_ARM_A = {
    "EPOCHS_MAX": "EPOCHS_MAX_A",
    "PATIENCE": "PATIENCE",
    "POOLED_IMPROVEMENT_EPS": "POOLED_IMPROVEMENT_EPS",
    "EMBARGO_ROWS": "EMBARGO_ROWS",
    "GATE_FOLD_HELDOUT_RMSE_MAX": "GATE_FOLD_HELDOUT_RMSE_MAX",
    "GATE_POOLED_VALIDATION_RMSE_MAX": "GATE_POOLED_VALIDATION_RMSE_MAX",
    "GATE_FINAL_RMSE_MAX": "GATE_FINAL_RMSE_MAX",
    "GATE_FINAL_PER_JOINT_RMSE_MAX": "GATE_FINAL_PER_JOINT_RMSE_MAX",
    "GATE_FINAL_PER_JOINT_MAXABS_MAX": "GATE_FINAL_PER_JOINT_MAXABS_MAX",
    "WAIT_RECONSTRUCTION": "WAIT_RECONSTRUCTION",
    "CLIP_LOSS_WEIGHT_J_CLIP_W": "CLIP_LOSS_WEIGHT_J_CLIP_W",
    "GRADIENT_CLIPPING": "GRADIENT_CLIPPING",
}


def verify_only_lr_changed() -> dict[str, Any]:
    """Every configuration value except the learning rate must equal arm A's, by identity."""
    for name, attr in INHERITED_FROM_ARM_A.items():
        mine = globals()[name]
        theirs = getattr(A, attr)
        if mine != theirs:
            raise B1R2BError(f"{name} diverges from arm A: {mine!r} != {theirs!r}")
    # the July per-batch hyperparameters are module-level in v26b_b_exec and must be untouched
    july = {"J_BATCH": 64, "J_CLIP_W": 1.0, "J_LOGSTD_W": 0.1, "J_ANCHOR_W": 1e-5, "J_SEED": 123,
            "J_LR": 3e-4}
    for k, v in july.items():
        if getattr(EX, k) != v:
            raise B1R2BError(f"the pinned July constant {k} was modified: {getattr(EX, k)} != {v}")
    if LR_ARM_B == LR_ARM_A:
        raise B1R2BError("arm B must differ from arm A in the learning rate")
    # the fold design must be the SAME object-for-object as arm A's
    mine_fd = folds_11()
    theirs_fd = A.folds_11()
    for k in ("n_folds", "n_complete_cycle_folds", "n_tail_folds", "heldout_rows_total",
              "embargo_rows", "semantics"):
        if mine_fd[k] != theirs_fd[k]:
            raise B1R2BError(f"the fold design diverges from arm A at {k}")
    for f, h in zip(mine_fd["folds"], theirs_fd["folds"]):
        if not (np.array_equal(f["_val"], h["_val"]) and np.array_equal(f["_train"], h["_train"])):
            raise B1R2BError(f"fold {f['fold']} has different row sets from arm A")
    return {"only_difference": {"learning_rate": {"arm_A": LR_ARM_A, "arm_B": LR_ARM_B}},
            "inherited_identical": sorted(INHERITED_FROM_ARM_A),
            "july_constants_untouched": july,
            "fold_row_sets_identical_to_arm_A": True}


def folds_11(*, embargo: int = EMBARGO_ROWS) -> dict[str, Any]:
    """The arm-A fold design, reused unchanged."""
    return A.folds_11(embargo=embargo)


def verify_prereg() -> dict[str, Any]:
    p = HERE / PREREG_NAME
    if not p.is_file():
        raise B1R2BError(f"pre-registration missing: {PREREG_NAME}")
    got = C.sha256_file(p)
    if got != PREREG_SHA:
        raise B1R2BError(f"pre-registration was modified: {got} != {PREREG_SHA}")
    pre = json.loads(p.read_text())
    if pre["stage_token"] != STAGE:
        raise B1R2BError("the amendment does not authorise this stage")
    if pre["matrix_rows"] != 1 or [a["arm"] for a in pre["experiment_matrix"]] != ["B"]:
        raise B1R2BError("the experiment matrix must have exactly one row, arm B")
    if float(pre["experiment_matrix"][0]["learning_rate"]) != LR_ARM_B:
        raise B1R2BError("the amendment's learning rate does not match the executor")
    if int(pre["experiment_matrix"][0]["epochs_max"]) != EPOCHS_MAX:
        raise B1R2BError("the amendment's budget does not match the inherited budget")
    return {PREREG_NAME: got}


# ================================================================ the two passes =================

def fold_curves(init: Mapping[str, np.ndarray], fd: Mapping[str, Any],
                *, lr: float = LR_ARM_B, progress: bool = True) -> list[dict[str, Any]]:
    """Pass one at the arm's learning rate. Structurally arm A's pass one; only `lr` is threaded."""
    obs, act = fd["inp"]["obs"], fd["inp"]["act"]
    out = []
    for f in fd["folds"]:
        _, rep = EX.fit_masked(init, obs, act, train_idx=f["_train"], val_idx=f["_val"],
                               epochs=EPOCHS_MAX, patience=None, progress=False, lr=lr)
        curve = [float(h["val_mse"]) for h in rep["history"]]
        if len(curve) != EPOCHS_MAX:
            raise B1R2BError(f"fold {f['fold']}: {len(curve)} epochs, expected {EPOCHS_MAX}")
        if float(rep["learning_rate"]) != float(lr):
            raise B1R2BError("the fit did not use the requested learning rate")
        out.append({"fold": int(f["fold"]), "kind": f["kind"], "origin": f["origin"],
                    "name": f["name"], "detail": f["detail"],
                    "validation_rows": f["validation_rows"], "train_rows": f["train_rows"],
                    "wait_rows_in_train": f["wait_rows_in_train"],
                    "own_best_epoch_not_used": int(rep["best_epoch"]),
                    "own_best_rmse": float(np.sqrt(min(curve))),
                    "curve_mse": curve})
        if progress:
            print(json.dumps({"pass": 1, "fold": int(f["fold"]), "kind": f["kind"],
                              "min_mse": float(min(curve)),
                              "argmin": int(np.argmin(curve)) + 1}), flush=True)
    return out


def fold_states_at(init: Mapping[str, np.ndarray], fd: Mapping[str, Any], e_star: int,
                   *, lr: float = LR_ARM_B, progress: bool = True) -> list[dict[str, Any]]:
    """Pass two at the arm's learning rate."""
    obs, act = fd["inp"]["obs"], fd["inp"]["act"]
    out = []
    for f in fd["folds"]:
        state, rep = EX.fit_masked(init, obs, act, train_idx=f["_train"], val_idx=None,
                                   epochs=e_star, patience=None, progress=False, lr=lr)
        B0.assert_masked_columns_zero(state, f"fold {f['fold']} at e*")
        B0.assert_clock_columns_zero(state, f"fold {f['fold']} at e*")
        B0.assert_no_masked_update(init, state)
        before = EX.metrics(init, obs, act, f["_val"])
        after = EX.metrics(state, obs, act, f["_val"])
        out.append({"fold": int(f["fold"]), "kind": f["kind"], "origin": f["origin"],
                    "name": f["name"], "detail": f["detail"],
                    "validation_rows": f["validation_rows"],
                    "evaluated_at_epoch": int(e_star),
                    "heldout_before_b0": before, "heldout_at_e_star": after,
                    "improved_over_b0": bool(after["aggregate_rmse"] < before["aggregate_rmse"]),
                    "epochs_run": int(rep["epochs_run"])})
        if progress:
            print(json.dumps({"pass": 2, "fold": int(f["fold"]),
                              "heldout_rmse": after["aggregate_rmse"]}), flush=True)
    return out


def select_epoch(curves: Sequence[Mapping[str, Any]], *, budget: int = EPOCHS_MAX) -> dict[str, Any]:
    """Arm A's selection function, reused unchanged."""
    return A.select_epoch(curves, budget=budget)


def convergence_diagnostics(pooled_curve: Sequence[float],
                            sel: Mapping[str, Any]) -> dict[str, Any]:
    """Records whether the FULL pooled curve converged at this budget, so the falsifiers can be
    read at all.  Convergence here means the GLOBAL argmin lands with at least 60 epochs of budget
    left after it (argmin <= 1140) and nothing in those 60 improves on it by more than 1e-9.
    The criterion is stated only in those terms - a global argmin with headroom - because any
    looser phrasing invited the conflation with the selection rule's patience that correction 3
    exists to prevent.

    Architect correction 2: at a threefold lower learning rate with the budget fixed, a NO-GO is
    confounded between a false hypothesis and insufficient advancement.  These numbers do not
    change the single variable; they make the outcome readable."""
    pooled = np.asarray(pooled_curve, dtype=float)
    e = int(sel["e_star"])
    lo, hi = max(0, e - 26), min(pooled.size, e + 25)
    win = pooled[lo:hi]

    # --- FULL-CURVE convergence, a DIFFERENT quantity from the selection's patience --------------
    # Arm A is the proof that the two must not be conflated: its selection patience was exhausted
    # at epoch 297, yet the global minimum of its full 1200-epoch curve was at 1171 and was BETTER.
    # Exhausting the scan says the July rule stopped looking; it says nothing about convergence.
    argmin = int(np.argmin(pooled)) + 1
    tail_has_no_improvement = bool(
        np.all(pooled[argmin: argmin + FULL_CURVE_STABLE_EPOCHS] > pooled[argmin - 1] - 1e-9))
    enough_headroom = bool(argmin <= FULL_CURVE_ARGMIN_MAX)
    converged = bool(enough_headroom and tail_has_no_improvement)
    return {
        "definition": (f"the FULL pooled curve is converged at this budget when its GLOBAL minimum "
                       f"falls at or before epoch {FULL_CURVE_ARGMIN_MAX} and is followed by at "
                       f"least {FULL_CURVE_STABLE_EPOCHS} epochs with no improvement greater than "
                       f"1e-9. An argmin later than {FULL_CURVE_ARGMIN_MAX} means NOT CONVERGED at "
                       f"the budget."),
        "distinct_from_selection_patience": (
            "This is NOT the selection rule's patience. Exhausting the scan means the July rule "
            "stopped looking, not that the curve converged. Arm A proves the difference: its scan "
            f"stopped at epoch {ARM_A_PATIENCE_EXHAUSTED_AT} while the global minimum of its full "
            "curve was at epoch 1171 and was lower. The binding selection of e* is unchanged."),
        "pooled_global_argmin": argmin,
        "pooled_global_min": float(pooled.min()),
        "argmin_within_headroom": enough_headroom,
        "argmin_max_for_convergence": FULL_CURVE_ARGMIN_MAX,
        "no_improvement_in_following_60": tail_has_no_improvement,
        "full_curve_converged_at_budget": converged,
        "falsifiers_are_readable": converged,
        "selection_scan_stopped_at_epoch": sel["stopped_at_epoch"],
        "arm_A_selection_scan_stopped_at_epoch": ARM_A_PATIENCE_EXHAUSTED_AT,
        "arm_A_full_curve_argmin": ARM_A_FULL_CURVE_ARGMIN,
        "arm_A_full_curve_converged_at_budget": bool(ARM_A_FULL_CURVE_ARGMIN
                                                     <= FULL_CURVE_ARGMIN_MAX),
        "pooled_at_epochs": {str(k): (float(pooled[k - 1]) if k <= pooled.size else None)
                             for k in (100, 200, 400, 600, 800, 1000, 1200)},
        "pooled_spread_around_e_star": (float((win.max() - win.min()) / win.min())
                                        if win.size else None),
        "arm_A_spread_around_its_e_star": 0.44,
        "interpretation_rule": (
            "If full_curve_converged_at_budget is FALSE the curve is still descending at the budget "
            "edge: the falsifiers DO NOT APPLY and the outcome must be reported as an INCONCLUSIVE "
            "SCREENING with insufficient advancement, never as an informative NO-GO."),
        "forbidden_conclusion": (
            "A NO-GO in this arm does NOT establish a limit of the observation contract. The lower "
            "learning rate at a fixed budget covers less optimisation distance than arm A, so the "
            "verdict alone cannot separate a false sharpness hypothesis from an optimisation that "
            "did not arrive."),
    }


def falsifiers(curves: Sequence[Mapping[str, Any]], pooled_rmse: float,
               diag: Mapping[str, Any]) -> dict[str, Any]:
    by_fold = {int(c["fold"]): c for c in curves}
    f1 = {f: float(by_fold[f]["own_best_rmse"]) for f in FALSIFIER_HEAD_FOLDS}
    f1_fired = all(v > GATE_FOLD_HELDOUT_RMSE_MAX for v in f1.values())
    f2_fired = bool(pooled_rmse >= ARM_A_POOLED_SINGLE_EPOCH_ORACLE_RMSE)
    readable = bool(diag["falsifiers_are_readable"])
    return {"readable": readable,
            "readability_condition": (f"the FULL pooled curve must have converged at this budget: "
                                      f"global argmin at or before epoch {FULL_CURVE_ARGMIN_MAX} "
                                      f"with no improvement greater than 1e-9 in the following "
                                      f"{FULL_CURVE_STABLE_EPOCHS}. Otherwise the screening is "
                                      f"inconclusive and no falsifier may be read. This is NOT the "
                                      f"selection rule's patience."),
            "F1_head_folds_above_gate_at_own_best": {
                "threshold": GATE_FOLD_HELDOUT_RMSE_MAX, "observed": f1,
                "fired": (bool(f1_fired) if readable else None)},
            "F2_pooled_does_not_beat_arm_A_oracle": {
                "threshold": ARM_A_POOLED_SINGLE_EPOCH_ORACLE_RMSE, "observed": pooled_rmse,
                "fired": (bool(f2_fired) if readable else None)},
            "hypothesis_falsified": (bool(f1_fired or f2_fired) if readable else None),
            "note": "the head-vs-cycle1 conflict is an INTEGRITY CONTROL and is deliberately "
                    "absent from this list: it cannot move under a learning-rate change"}


# ================================================================ preflight ======================

def preflight() -> dict[str, Any]:
    """No-write, fail-closed. Nothing may be fitted before this returns GO."""
    prereg = verify_prereg()
    only_lr = verify_only_lr_changed()
    control = verify_conflict_control()
    pins = R1.verify_pins()
    init, parent = R1.load_parent()
    fd = folds_11()
    cov = D.full_coverage(embargo=EMBARGO_ROWS)
    if cov["aggregate"]["any_temporal_leakage"] or cov["aggregate"]["any_signature_missing"]:
        raise B1R2BError("the design leaks temporally or misses a signature")
    if cov["aggregate"]["min_temporal_distance"] != EMBARGO_ROWS + 1:
        raise B1R2BError("the embargo is not honoured")
    equiv = B0.functional_equivalence_25(init, fd["inp"]["raw"])
    blockers = []
    if CANDIDATE_DIR.exists():
        blockers.append(f"no-clobber: {C.rel(CANDIDATE_DIR)} exists")
    if (DIAG_DIR / RECEIPT_NAME).exists():
        blockers.append(f"no-clobber: {C.rel(DIAG_DIR / RECEIPT_NAME)} exists")
    return {"verdict": ("GO" if not blockers else "BLOCKED"), "stage": STAGE, "blockers": blockers,
            "role": "SCREENING, one variable",
            "pre_registration": prereg, "only_lr_changed": only_lr,
            "integrity_control": control, "pins": pins, "parent": parent,
            "single_variable": {"name": "learning rate", "arm_A": LR_ARM_A, "arm_B": LR_ARM_B,
                                "budget_fixed_at": EPOCHS_MAX,
                                "caveat": "a lower rate at a fixed budget covers less optimisation "
                                          "distance; a NO-GO cannot establish a contract limit"},
            "optimizer": {"name": "Adam", "batch": EX.J_BATCH, "lr": LR_ARM_B, "seed": EX.J_SEED,
                          "clip_loss_weight_J_CLIP_W": CLIP_LOSS_WEIGHT_J_CLIP_W,
                          "clip_loss_is": "an action-bound penalty on the loss, "
                                          "relu(|mean| - 1)^2, applied before backward()",
                          "gradient_clipping": GRADIENT_CLIPPING,
                          "logstd_weight": EX.J_LOGSTD_W, "anchor_weight": EX.J_ANCHOR_W},
            "contract": {"actor_width": B0.ACTOR_WIDTH,
                         "masked_columns": list(B0.controller_columns()),
                         "clock_columns": list(B0.CLOCK_COLUMNS),
                         "support_columns_23d": list(LO.SUPPORT_COLUMNS)},
            "design": {k: v for k, v in fd.items() if k not in ("g", "inp", "folds")},
            "folds": [{k: v for k, v in f.items() if not k.startswith("_")} for f in fd["folds"]],
            "coverage_aggregate": cov["aggregate"],
            "dataset": fd["inp"]["report"], "wait_rows": int(fd["inp"]["wait_rows"].size),
            "functional_equivalence_25": equiv,
            "gates": {"fold_heldout_rmse_max": GATE_FOLD_HELDOUT_RMSE_MAX,
                      "pooled_validation_rmse_max": GATE_POOLED_VALIDATION_RMSE_MAX,
                      "final_rmse_max": GATE_FINAL_RMSE_MAX,
                      "final_per_joint_rmse_max": GATE_FINAL_PER_JOINT_RMSE_MAX,
                      "final_per_joint_max_abs_max": GATE_FINAL_PER_JOINT_MAXABS_MAX,
                      "wait_reconstruction": WAIT_RECONSTRUCTION,
                      "source": "imported from v26b_b1r1_exec via arm A, not relaxed"}}


# ================================================================ driver =========================

def run(*, authorized_stage: str | None, progress: bool = True) -> dict[str, Any]:
    if authorized_stage != STAGE:
        raise B1R2BError(f"requires --authorized-stage {STAGE}; got {authorized_stage!r}")
    pre = preflight()
    if pre["blockers"]:
        raise B1R2BError(f"preflight BLOCKED: {pre['blockers']}")
    init, parent = R1.load_parent()
    fd = folds_11()
    obs, act = fd["inp"]["obs"], fd["inp"]["act"]
    wait_rows = fd["inp"]["wait_rows"]

    curves = fold_curves(init, fd, lr=LR_ARM_B, progress=progress)
    sel = select_epoch(curves)
    pooled = sel.pop("pooled")
    e_star = int(sel["e_star"])
    pooled_rmse = float(sel["pooled_rmse_at_e_star"])
    diag = convergence_diagnostics(pooled, sel)
    fal = falsifiers(curves, pooled_rmse, diag)
    folds = fold_states_at(init, fd, e_star, lr=LR_ARM_B, progress=progress)

    final_state, final_rep = EX.fit_masked(init, obs, act, train_idx=np.arange(len(obs)),
                                           val_idx=None, epochs=e_star, patience=None,
                                           progress=progress, lr=LR_ARM_B)
    final_m = EX.metrics(final_state, obs, act, None)
    wait_m = EX.metrics(final_state, obs, act, wait_rows)
    g = fd["g"]
    regions = {"b1r1_covered_cycles": g["cycle"], "b1r1_uncovered_startup_tails": g["uncovered"],
               "structurally_complete_cycles": g["structurally_complete"], "tails": g["tails"],
               "wait": g["wait"]}
    region_metrics = {k: EX.metrics(final_state, obs, act, v) for k, v in regions.items()}

    integrity = {
        "ten_keys": tuple(final_state.keys()) == RF.EXPECTED_KEY_ORDER,
        "width_35": int(np.asarray(final_state["pi.0.0.weight"]).shape[1]) == B0.ACTOR_WIDTH,
        "masked_columns_zero": True, "clock_columns_zero": True, "no_masked_update": True,
        "logstd_byte_identical_to_b0": bool(
            np.array_equal(np.asarray(final_state["pi.1.weight"])[R.ACTION_DIM:],
                           np.asarray(init["pi.1.weight"])[R.ACTION_DIM:])
            and np.array_equal(np.asarray(final_state["pi.1.bias"])[R.ACTION_DIM:],
                               np.asarray(init["pi.1.bias"])[R.ACTION_DIM:])),
        "no_critic": True,
    }
    try:
        B0.assert_masked_columns_zero(final_state, "final")
        B0.assert_clock_columns_zero(final_state, "final")
        B0.assert_no_masked_update(init, final_state)
    except B0.B0Error as exc:
        integrity["masked_columns_zero"] = False
        integrity["no_masked_update"] = False
        integrity["violation"] = str(exc)
    equiv = B0.functional_equivalence_25(final_state, fd["inp"]["raw"])

    gates = {
        "integrity_invariants": {"binding": True, **integrity,
                                 "pass": all(v for v in integrity.values() if isinstance(v, bool))},
        "functional_equivalence_25": {"binding": True, "bit_identical": equiv["bit_identical"],
                                      "pass": bool(equiv["bit_identical"])},
        "every_fold_improves_over_b0": {"binding": True,
                                        "per_fold": [f["improved_over_b0"] for f in folds],
                                        "pass": all(f["improved_over_b0"] for f in folds)},
        "fold_heldout_rmse": {"binding": True, "threshold": GATE_FOLD_HELDOUT_RMSE_MAX,
                              "observed": [f["heldout_at_e_star"]["aggregate_rmse"] for f in folds],
                              "pass": all(f["heldout_at_e_star"]["aggregate_rmse"]
                                          <= GATE_FOLD_HELDOUT_RMSE_MAX for f in folds)},
        "pooled_validation_rmse": {"binding": True, "threshold": GATE_POOLED_VALIDATION_RMSE_MAX,
                                   "observed": pooled_rmse,
                                   "pass": bool(pooled_rmse <= GATE_POOLED_VALIDATION_RMSE_MAX)},
        "final_aggregate_rmse": {"binding": True, "threshold": GATE_FINAL_RMSE_MAX,
                                 "observed": final_m["aggregate_rmse"],
                                 "pass": bool(final_m["aggregate_rmse"] <= GATE_FINAL_RMSE_MAX)},
        "final_per_joint_rmse": {"binding": True, "threshold": GATE_FINAL_PER_JOINT_RMSE_MAX,
                                 "observed": {k: v["rmse"] for k, v in final_m["per_joint"].items()},
                                 "pass": all(v["rmse"] <= GATE_FINAL_PER_JOINT_RMSE_MAX
                                             for v in final_m["per_joint"].values())},
        "final_per_joint_max_abs": {"binding": True, "threshold": GATE_FINAL_PER_JOINT_MAXABS_MAX,
                                    "observed": {k: v["max_abs"]
                                                 for k, v in final_m["per_joint"].items()},
                                    "pass": all(v["max_abs"] <= GATE_FINAL_PER_JOINT_MAXABS_MAX
                                                for v in final_m["per_joint"].values())},
        "wait_reconstruction": {"binding": True, **WAIT_RECONSTRUCTION,
                                "non_generalisation": True, "observed": wait_m,
                                "pass": bool(
                                    wait_m["aggregate_rmse"]
                                    <= WAIT_RECONSTRUCTION["aggregate_rmse_max"]
                                    and all(v["rmse"] <= WAIT_RECONSTRUCTION["per_joint_rmse_max"]
                                            for v in wait_m["per_joint"].values())
                                    and all(v["max_abs"]
                                            <= WAIT_RECONSTRUCTION["per_joint_max_abs_max"]
                                            for v in wait_m["per_joint"].values()))},
    }
    failed = sorted(k for k, v in gates.items() if not v["pass"])
    verdict = "GO" if not failed else "NO-GO"
    if not diag["falsifiers_are_readable"] and verdict == "NO-GO":
        outcome = "INCONCLUSIVE SCREENING - insufficient advancement, not an informative NO-GO"
    else:
        outcome = verdict

    receipt = {
        "schema": "v26b_b1r2b_lr.1", "authorized_stage": STAGE,
        "role": "SCREENING, one variable",
        "pre_registration": pre["pre_registration"],
        "only_lr_changed": pre["only_lr_changed"],
        "integrity_control_NOT_a_falsifier": pre["integrity_control"],
        "single_variable": pre["single_variable"],
        "optimizer": pre["optimizer"],
        "clip_loss_weight_J_CLIP_W": CLIP_LOSS_WEIGHT_J_CLIP_W,
        "gradient_clipping": GRADIENT_CLIPPING,
        "clip_disambiguation": ("clip 1.0 is J_CLIP_W, an action-bound penalty on the loss "
                                "relu(|mean| - 1)^2 applied to the network output before "
                                "backward(). It is NOT gradient clipping, which is absent."),
        "pins": pre["pins"], "parent": parent,
        "deployable": False, "sigma_unresolved": True,
        "dataset": fd["inp"]["report"], "design": pre["design"], "folds": pre["folds"],
        "coverage_aggregate": pre["coverage_aggregate"],
        "convergence_diagnostics": diag,
        "falsifiers": fal,
        "pooled_selection": sel,
        "pooled_curve": [float(x) for x in pooled],
        "fold_curves": curves,
        "folds_evaluated_at_e_star": folds,
        "pooled_validation_rmse_at_e_star": pooled_rmse,
        "arm_A_comparison": {"receipt_sha256": ARM_A_RECEIPT_SHA,
                             "single_epoch_oracle_rmse": ARM_A_POOLED_SINGLE_EPOCH_ORACLE_RMSE,
                             "patience_exhausted_at": ARM_A_PATIENCE_EXHAUSTED_AT},
        "july_protocol": {k: v for k, v in final_rep.items()
                          if k not in ("history", "history_first_last")},
        "final_fit": {"epochs": e_star, "selection_mode": final_rep["selection_mode"],
                      "history_first_last": final_rep["history_first_last"],
                      "metrics_all_1500_rows": final_m},
        "region_breakdown_DIAGNOSTIC_ONLY": region_metrics,
        "wait_reconstruction": {**WAIT_RECONSTRUCTION, "observed": wait_m,
                                "rows": int(wait_rows.size)},
        "functional_equivalence_25": equiv,
        "gates": gates, "failed": failed, "verdict": verdict, "outcome": outcome,
        "forbidden_conclusions": [
            "A NO-GO here does NOT establish a limit of the observation contract.",
            "A non-converged curve must be reported as an inconclusive screening.",
            "No comparison against July's 6.63e-05, which is not a generalisation measurement.",
        ],
        "scope": ("base-phase offline screening only. No rollout, no collection, no Markov phase, "
                  "no DAgger. A NO-GO forbids any rollout, forbids post-hoc hyperparameter "
                  "correction, and forbids any autonomous follow-up arm."),
        "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}

    DIAG_DIR.mkdir(parents=True, exist_ok=True)
    sm = DIAG_DIR / MODULE_DIR_NAME
    sm.mkdir(parents=True, exist_ok=True)
    names35, _, mshas = VS.pinned_names()
    with (sm / "module_state.pkl").open("wb") as fh:
        pickle.dump({k: np.asarray(v) for k, v in final_state.items()}, fh,
                    protocol=pickle.HIGHEST_PROTOCOL)
    for f in ("metadata.json", "class_and_ctor_args.pkl"):
        shutil.copy2(R1.B0_DIR / MODULE_DIR_NAME / f, sm / f)
    manifest = {"schema_version": 1, "actor_feature_names": names35, "actor_feature_count": 35,
                "actor_digest": RF.actor_state_digest(final_state),
                "module_state_sha256": C.sha256_file(sm / "module_state.pkl"),
                "manifest35_sha256": mshas["manifest35_sha256"],
                "exploration_sigma": [B0.SIGMA_PLACEHOLDER] * R.ACTION_DIM,
                "sigma_note": B0.SIGMA_STATEMENT,
                "derived_from": C.rel(R1.B0_DIR / MODULE_DIR_NAME),
                "source_actor_digest": R1.PIN_B0_ACTOR_DIGEST,
                "deployable": False, "sigma_unresolved": True, "actor_label": ACTOR_LABEL,
                "controller_state_mask": {"active": True,
                                          "columns": list(B0.controller_columns()),
                                          "mask_value": float(B0.MASK_VALUE)},
                "clock_columns": list(B0.CLOCK_COLUMNS),
                "offline_verdict": verdict, "outcome": outcome,
                "status": ("B1R2-B screening artefact. NOT a candidate unless every binding gate "
                           "passes; never deployable; no rollout has been run.")}
    C.write_json(sm / "actor_feature_manifest.json", manifest, clobber=False)
    receipt["output_files_sha256"] = {p.name: C.sha256_file(p) for p in sorted(sm.iterdir())}
    C.write_json(DIAG_DIR / RECEIPT_NAME, receipt, clobber=False)
    receipt_sha = C.sha256_file(DIAG_DIR / RECEIPT_NAME)

    promoted = None
    if verdict == "GO":
        staging = CANDIDATE_DIR.parent / (CANDIDATE_DIR.name + ".staging")
        shutil.rmtree(staging, ignore_errors=True)
        try:
            shutil.copytree(DIAG_DIR, staging)
            if CANDIDATE_DIR.exists():
                raise B1R2BError(f"no-clobber: {CANDIDATE_DIR} appeared during staging")
            staging.rename(CANDIDATE_DIR)
            promoted = C.rel(CANDIDATE_DIR)
        except BaseException:
            shutil.rmtree(staging, ignore_errors=True)
            raise

    return {"verdict": verdict, "outcome": outcome, "failed": failed,
            "receipt_sha256": receipt_sha, "e_star": e_star,
            "convergence_diagnostics": diag, "falsifiers": fal,
            "pooled_validation_rmse": pooled_rmse,
            "fold_heldout_rmse": [f["heldout_at_e_star"]["aggregate_rmse"] for f in folds],
            "final_metrics": final_m, "wait_reconstruction": wait_m,
            "region_breakdown": region_metrics,
            "diagnostics_dir": C.rel(DIAG_DIR), "promoted_candidate": promoted}


def main(argv: Sequence[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="V26B B1R2-B learning-rate screening arm")
    p.add_argument("--preflight", action="store_true")
    p.add_argument("--dry", action="store_true", help="design, rule and controls only, no fit")
    p.add_argument("--authorized-stage", default=None)
    p.add_argument("--no-progress", action="store_true")
    a = p.parse_args(argv)
    if a.dry:
        verify_prereg()
        only_lr = verify_only_lr_changed()
        control = verify_conflict_control()
        fd = folds_11()
        print(json.dumps({"mode": "dry", "only_lr_changed": only_lr,
                          "integrity_control": control,
                          "design": {k: v for k, v in fd.items()
                                     if k not in ("g", "inp", "folds")}}, indent=2, default=str))
        return 0
    if a.preflight:
        print(json.dumps(preflight(), indent=2, default=str))
        return 0
    r = run(authorized_stage=a.authorized_stage, progress=not a.no_progress)
    print(json.dumps(r, indent=2, default=str))
    return 0 if r["verdict"] == "GO" else 1


if __name__ == "__main__":
    sys.exit(main())
