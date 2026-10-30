"""V26B B1R2-A - the budget arm, executed under a pre-registered amendment.

ONE experimental variable: the optimisation budget, 400 -> 1200 epochs, measured against the
CORRECTED Rev1 validation design (11 folds / 1313 held-out rows), NOT against B1R1 (6 folds / 910).
The design correction is a methodological prerequisite, not a second arm; see the addendum.

Everything else is inherited unchanged from B1R1: fit_masked with unchanged semantics, Adam,
batch 64, lr 3e-4, seed 123, flat MSE plus the clip-loss / logstd / anchor terms, patience 60,
eps 1e-9, the dataset and its labels, the 35D masked student, the B0 parent, and every gate
threshold, which is IMPORTED from v26b_b1r1_exec and never restated or relaxed here.

`clip 1.0` is J_CLIP_W, an ACTION-BOUND PENALTY ON THE LOSS: relu(|mean| - 1)^2 applied to the
network output before backward().  It is NOT gradient clipping.  No gradient-norm or
gradient-value clipping exists anywhere in this pipeline; the only gradient manipulation is the
first-layer column mask, which is masking, not clipping.

Selection, pre-registered and free of future-reading:
  1. eleven independent fits, all byte-identical from B0, each running the FULL budget with no
     internal early stop, recording val_mse at every epoch;
  2. pooled(e) = row-weighted mean of the fold curves.  Because the eleven folds partition the
     non-WAIT rows exactly once, pooled(e) IS the out-of-fold MSE over EVERY one of the 1313
     non-WAIT rows - no row ignored, none counted twice;
  3. e* = July's rule on pooled(), patience 60, strict improvement > 1e-9, first minimum;
  4. every fold is evaluated at the SAME e*, never at its own best;
  5. the final fit restarts byte-identically from B0 on all 1500 rows for exactly e* epochs.

The budget readout applies the same rule twice to the SAME recorded curves, truncated at 400 and
over the full 1200.  The 400 truncation is DIAGNOSTIC ONLY and can never change e*, a gate or the
verdict.

No rollout, no collection, no DAgger, no Markov phase, no PPO.  A candidate is promoted ONLY if
every binding gate passes; otherwise the run stops offline and the evidence is preserved.

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
import v26b_anchors as VA  # noqa: E402
import v26b_student as VS  # noqa: E402
import f0_common as C  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_refit as RF  # noqa: E402


class B1R2AError(RuntimeError):
    pass


STAGE = "V26B-B1R2A-BUDGET"

# ---- the pre-registration, pinned by content hash; the fit refuses to start without it ----------
PREREG_NAME = "v26b_amendment_b1r2a_budget.json"
PREREG_SHA = "b64888a6eeaf0fbad98b2168a343f1e2caf5a4a1a380935345470e18980fc09d"
ADDENDUM_NAME = "v26b_addendum_b1r2a_baseline_and_clip.json"
ADDENDUM_SHA = "3d24928de06b3a842c4089d782f8e0ddb3ceaac731f98f23d768b6251e2c6f9a"

# ---- the single experimental variable ----------------------------------------------------------
EPOCHS_MAX_A = 1200
BASELINE_BUDGET_FOR_READOUT = R1.EPOCHS_MAX          # 400, diagnostic truncation only

# ---- everything below is IMPORTED, never restated, never relaxed -------------------------------
PATIENCE = R1.PATIENCE
POOLED_IMPROVEMENT_EPS = R1.POOLED_IMPROVEMENT_EPS
EMBARGO_ROWS = D.EMBARGO_ROWS
GATE_FOLD_HELDOUT_RMSE_MAX = R1.FOLD_HELDOUT_RMSE_MAX
GATE_POOLED_VALIDATION_RMSE_MAX = R1.POOLED_VALIDATION_RMSE_MAX
GATE_FINAL_RMSE_MAX = R1.FINAL_RMSE_MAX
GATE_FINAL_PER_JOINT_RMSE_MAX = R1.FINAL_PER_JOINT_RMSE_MAX
GATE_FINAL_PER_JOINT_MAXABS_MAX = R1.FINAL_PER_JOINT_MAXABS_MAX
WAIT_RECONSTRUCTION = R1.WAIT_RECONSTRUCTION

# `clip 1.0` is a loss term, not gradient clipping. Recorded separately so the two can never be
# confused in a receipt or a report.
CLIP_LOSS_WEIGHT_J_CLIP_W = EX.J_CLIP_W
GRADIENT_CLIPPING = "none"

DIAG_DIR = VA.OUT_ROOT / "diagnostics" / "b1r2a"
CANDIDATE_DIR = VA.OUT_ROOT / "candidates" / "B1R2A_BASE35_BUDGET1200"
RECEIPT_NAME = "v26b_b1r2a_receipt.json"
MODULE_DIR_NAME = "rl_module"
ACTOR_LABEL = "B1R2A_BASE35_BUDGET1200"


# ================================================================ pre-registration ===============

def verify_prereg() -> dict[str, Any]:
    """Fail-closed: the amendment and its addendum must exist and match their pinned hashes."""
    out = {}
    for name, pin in ((PREREG_NAME, PREREG_SHA), (ADDENDUM_NAME, ADDENDUM_SHA)):
        p = HERE / name
        if not p.is_file():
            raise B1R2AError(f"pre-registration missing: {name}")
        got = C.sha256_file(p)
        if got != pin:
            raise B1R2AError(f"pre-registration {name} was modified: {got} != {pin}")
        out[name] = got
    pre = json.loads((HERE / PREREG_NAME).read_text())
    if pre["stage_token"] != STAGE:
        raise B1R2AError("the amendment does not authorise this stage")
    if pre["matrix_rows"] != 1 or [a["arm"] for a in pre["experiment_matrix"]] != ["A"]:
        raise B1R2AError("the experiment matrix must have exactly one row, arm A")
    if int(pre["experiment_matrix"][0]["epochs_max"]) != EPOCHS_MAX_A:
        raise B1R2AError("the amendment's budget does not match the executor")
    return out


def verify_gates_not_relaxed() -> dict[str, Any]:
    """Every threshold must be the object imported from the B1R1 executor, not a local copy."""
    pairs = {"fold_heldout_rmse_max": (GATE_FOLD_HELDOUT_RMSE_MAX, R1.FOLD_HELDOUT_RMSE_MAX),
             "pooled_validation_rmse_max": (GATE_POOLED_VALIDATION_RMSE_MAX,
                                            R1.POOLED_VALIDATION_RMSE_MAX),
             "final_rmse_max": (GATE_FINAL_RMSE_MAX, R1.FINAL_RMSE_MAX),
             "final_per_joint_rmse_max": (GATE_FINAL_PER_JOINT_RMSE_MAX,
                                          R1.FINAL_PER_JOINT_RMSE_MAX),
             "final_per_joint_max_abs_max": (GATE_FINAL_PER_JOINT_MAXABS_MAX,
                                             R1.FINAL_PER_JOINT_MAXABS_MAX)}
    for k, (mine, theirs) in pairs.items():
        if mine != theirs:
            raise B1R2AError(f"gate {k} diverges from the B1R1 pin: {mine} != {theirs}")
    if WAIT_RECONSTRUCTION != R1.WAIT_RECONSTRUCTION:
        raise B1R2AError("the WAIT reconstruction gate diverges from the B1R1 pin")
    return {k: v[0] for k, v in pairs.items()}


# ================================================================ the eleven folds ===============

def folds_11(*, embargo: int = EMBARGO_ROWS) -> dict[str, Any]:
    """The corrected Rev1 design: 8 structurally complete cycles + 3 tails, with index arrays.

    The block set is exactly the one v26b_b1r2_diagnostics.full_coverage quantifies; the test
    cross-checks the two against each other so this function cannot drift from the audited design.
    """
    g = D.row_groups()
    inp = g["inp"]
    raw, traj = inp["raw"], inp["traj"]
    n = len(raw)
    blocks: list[dict[str, Any]] = []
    for f in inp["split"]["folds"]:
        blocks.append({"kind": "complete_cycle", "origin": "b1r1_loco", "name": f["name"],
                       "detail": int(f["cycle"]), "rows": np.asarray(f["_val"], dtype=int)})
    for p in g["uncovered_segments"]:
        blocks.append({"kind": p["structure"], "origin": "b1r1_uncovered", "name": p["name"],
                       "detail": p["position"],
                       "rows": np.arange(p["global_lo"], p["global_hi"] + 1, dtype=int)})
    seen = np.concatenate([b["rows"] for b in blocks])
    if len(seen) != len(np.unique(seen)):
        raise B1R2AError("the folds overlap: a row would be held out more than once")
    non_wait = np.array([i for i in range(n) if i not in set(g["wait"].tolist())], dtype=int)
    if not np.array_equal(np.sort(seen), np.sort(non_wait)):
        raise B1R2AError("the folds do not cover every non-WAIT row exactly once")
    folds: list[dict[str, Any]] = []
    for k, b in enumerate(blocks):
        val = b["rows"]
        is_val = np.zeros(n, dtype=bool); is_val[val] = True
        emb = np.zeros(n, dtype=bool)
        tid = int(traj[val[0]])
        idx = np.where(traj == tid)[0]
        for p in np.where(is_val[idx])[0]:
            lo = max(0, p - embargo); hi = min(len(idx) - 1, p + embargo)
            emb[idx[lo:hi + 1]] = True
        train = np.where(~is_val & ~emb)[0]
        if train.size == 0:
            raise B1R2AError(f"fold {k} has an empty training set")
        if np.any(raw[val, LO.WAIT_COLUMN] > 0.5):
            raise B1R2AError(f"fold {k} would validate WAIT rows, which is forbidden")
        folds.append({"fold": k, "kind": b["kind"], "origin": b["origin"], "name": b["name"],
                      "detail": b["detail"],
                      "_val": val, "_train": train,
                      "validation_rows": int(val.size), "train_rows": int(train.size),
                      "dropped_to_embargo": int(n - val.size - train.size),
                      "wait_rows_in_train": int(np.sum(raw[train, LO.WAIT_COLUMN] > 0.5))})
    n_complete = sum(1 for f in folds if f["kind"] == "complete_cycle")
    n_tail = sum(1 for f in folds if f["kind"] == "tail")
    if len(folds) != 11 or n_complete != 8 or n_tail != 3:
        raise B1R2AError(f"expected 11 folds = 8 complete cycles + 3 tails, got {len(folds)} = "
                         f"{n_complete} + {n_tail}")
    total_val = sum(f["validation_rows"] for f in folds)
    if total_val != 1313:
        raise B1R2AError(f"the folds must hold out all 1313 non-WAIT rows, got {total_val}")
    return {"g": g, "inp": inp, "folds": folds, "n_folds": len(folds),
            "n_complete_cycle_folds": n_complete, "n_tail_folds": n_tail,
            "heldout_rows_total": total_val, "embargo_rows": embargo,
            "semantics": g["structure_summary"]["semantics"]}


# ================================================================ selection ======================

def select_epoch(curves: Sequence[Mapping[str, Any]], *, budget: int) -> dict[str, Any]:
    """July's rule, reused unchanged from the B1R1 executor, on the pooled curve truncated to
    `budget` epochs.  Returns the selection plus the pooled curve actually used."""
    mse = [list(c["curve_mse"])[:budget] for c in curves]
    rows = [int(c["validation_rows"]) for c in curves]
    pooled = R1.pooled_curve(mse, rows)
    sel = R1.pooled_best_epoch(pooled, patience=PATIENCE, eps=POOLED_IMPROVEMENT_EPS)
    return {"budget": int(budget), "pooled": [float(x) for x in pooled], **sel,
            "pooled_rmse_at_e_star": float(np.sqrt(sel["pooled_mse_at_e_star"]))}


def budget_readout(curves: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    """The measurement of the hypothesis: the same rule applied to the same curves at 400 and 1200.

    DIAGNOSTIC ONLY for the 400 arm - it can never change e*, a gate or the verdict."""
    short = select_epoch(curves, budget=BASELINE_BUDGET_FOR_READOUT)
    full = select_epoch(curves, budget=EPOCHS_MAX_A)
    improved = float(short["pooled_mse_at_e_star"] - full["pooled_mse_at_e_star"])
    return {"binding_selection": "the 1200-epoch arm only",
            "diagnostic_only": "the 400-epoch truncation; it cannot change e*, a gate or a verdict",
            "at_400": {k: v for k, v in short.items() if k != "pooled"},
            "at_1200": {k: v for k, v in full.items() if k != "pooled"},
            "e_star_moved_past_400": bool(full["e_star"] > short["e_star"]),
            "pooled_mse_improvement": improved,
            "budget_was_binding": bool(full["e_star"] > short["e_star"]
                                       or improved > POOLED_IMPROVEMENT_EPS),
            "patience_exhausted_within_1200": full["stopped_at_epoch"] is not None,
            "budget_still_insufficient": full["stopped_at_epoch"] is None,
            "note": ("If the patience is not exhausted within 1200 epochs the budget is STILL the "
                     "binding constraint. That outcome is reported, never fixed: extending the "
                     "budget again would require a new authorised arm.")}


# ================================================================ the two passes =================

def fold_curves(init: Mapping[str, np.ndarray], fd: Mapping[str, Any],
                *, progress: bool = True) -> list[dict[str, Any]]:
    """Pass one: every fold runs the FULL budget with no early stop, recording its curve."""
    obs, act = fd["inp"]["obs"], fd["inp"]["act"]
    out = []
    for f in fd["folds"]:
        _, rep = EX.fit_masked(init, obs, act, train_idx=f["_train"], val_idx=f["_val"],
                               epochs=EPOCHS_MAX_A, patience=None, progress=False)
        curve = [float(h["val_mse"]) for h in rep["history"]]
        if len(curve) != EPOCHS_MAX_A:
            raise B1R2AError(f"fold {f['fold']}: {len(curve)} epochs recorded, "
                             f"expected {EPOCHS_MAX_A}")
        out.append({"fold": int(f["fold"]), "kind": f["kind"], "origin": f["origin"],
                    "name": f["name"], "detail": f["detail"],
                    "validation_rows": f["validation_rows"], "train_rows": f["train_rows"],
                    "wait_rows_in_train": f["wait_rows_in_train"],
                    "own_best_epoch_not_used": int(rep["best_epoch"]),
                    "curve_mse": curve})
        if progress:
            print(json.dumps({"pass": 1, "fold": int(f["fold"]), "kind": f["kind"],
                              "rows": f["validation_rows"], "min_mse": float(min(curve)),
                              "argmin": int(np.argmin(curve)) + 1}), flush=True)
    return out


def fold_states_at(init: Mapping[str, np.ndarray], fd: Mapping[str, Any], e_star: int,
                   *, progress: bool = True) -> list[dict[str, Any]]:
    """Pass two: re-run every fold for exactly e* epochs and score its own held-out block."""
    obs, act = fd["inp"]["obs"], fd["inp"]["act"]
    out = []
    for f in fd["folds"]:
        state, rep = EX.fit_masked(init, obs, act, train_idx=f["_train"], val_idx=None,
                                   epochs=e_star, patience=None, progress=False)
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
            print(json.dumps({"pass": 2, "fold": int(f["fold"]), "kind": f["kind"],
                              "heldout_rmse": after["aggregate_rmse"]}), flush=True)
    return out


# ================================================================ preflight ======================

def preflight() -> dict[str, Any]:
    """No-write, fail-closed. Nothing may be fitted before this returns GO."""
    prereg = verify_prereg()
    gates = verify_gates_not_relaxed()
    pins = R1.verify_pins()
    init, parent = R1.load_parent()
    fd = folds_11()
    cov = D.full_coverage(embargo=EMBARGO_ROWS)
    if cov["n_folds"] != fd["n_folds"] or cov["n_complete_cycle_folds"] != fd["n_complete_cycle_folds"]:
        raise B1R2AError("folds_11 has drifted from the audited Rev1 coverage design")
    if cov["aggregate"]["any_temporal_leakage"]:
        raise B1R2AError("the design leaks temporally")
    if cov["aggregate"]["any_signature_missing"]:
        raise B1R2AError("a discrete signature is missing from a training set")
    if cov["aggregate"]["min_temporal_distance"] != EMBARGO_ROWS + 1:
        raise B1R2AError("the embargo is not honoured")
    equiv = B0.functional_equivalence_25(init, fd["inp"]["raw"])
    blockers = []
    if CANDIDATE_DIR.exists():
        blockers.append(f"no-clobber: {C.rel(CANDIDATE_DIR)} exists")
    if (DIAG_DIR / RECEIPT_NAME).exists():
        blockers.append(f"no-clobber: {C.rel(DIAG_DIR / RECEIPT_NAME)} exists")
    return {"verdict": ("GO" if not blockers else "BLOCKED"), "stage": STAGE, "blockers": blockers,
            "pre_registration": prereg, "pins": pins, "parent": parent,
            "single_variable": {"name": "optimisation budget",
                                "baseline": f"{BASELINE_BUDGET_FOR_READOUT} epochs",
                                "arm_A": f"{EPOCHS_MAX_A} epochs",
                                "measured_against": "the CORRECTED Rev1 design, 11 folds / 1313 "
                                                    "held-out rows, not B1R1's 6 folds / 910",
                                "design_correction_is": "a methodological prerequisite, not a "
                                                        "second arm"},
            "optimizer": {"name": "Adam", "batch": EX.J_BATCH, "lr": EX.J_LR, "seed": EX.J_SEED,
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
            "selection_rule": {"epochs_max": EPOCHS_MAX_A, "patience": PATIENCE,
                               "eps": POOLED_IMPROVEMENT_EPS,
                               "pooled": "row-weighted mean of the 11 fold curves, which equals the "
                                         "out-of-fold MSE over all 1313 non-WAIT rows",
                               "e_star": "first epoch attaining the pooled minimum",
                               "evaluation": "every fold at the SAME e*, never at its own best",
                               "no_future_reading": "the final all-1500 fit never enters selection"},
            "gates": {**gates, "wait_reconstruction": WAIT_RECONSTRUCTION,
                      "source": "imported from v26b_b1r1_exec, not relaxed"}}


# ================================================================ driver =========================

def run(*, authorized_stage: str | None, progress: bool = True) -> dict[str, Any]:
    if authorized_stage != STAGE:
        raise B1R2AError(f"requires --authorized-stage {STAGE}; got {authorized_stage!r}")
    pre = preflight()
    if pre["blockers"]:
        raise B1R2AError(f"preflight BLOCKED: {pre['blockers']}")
    init, parent = R1.load_parent()
    fd = folds_11()
    obs, act = fd["inp"]["obs"], fd["inp"]["act"]
    wait_rows = fd["inp"]["wait_rows"]

    curves = fold_curves(init, fd, progress=progress)
    readout = budget_readout(curves)
    sel = select_epoch(curves, budget=EPOCHS_MAX_A)
    pooled = sel.pop("pooled")
    e_star = int(sel["e_star"])
    pooled_rmse = float(sel["pooled_rmse_at_e_star"])
    folds = fold_states_at(init, fd, e_star, progress=progress)

    final_state, final_rep = EX.fit_masked(init, obs, act, train_idx=np.arange(len(obs)),
                                           val_idx=None, epochs=e_star, patience=None,
                                           progress=progress)
    final_m = EX.metrics(final_state, obs, act, None)
    wait_m = EX.metrics(final_state, obs, act, wait_rows)

    # region breakdown, diagnostic: it never gates, it explains a failure
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

    receipt = {
        "schema": "v26b_b1r2a_budget.1", "authorized_stage": STAGE,
        "pre_registration": pre["pre_registration"],
        "single_variable": pre["single_variable"],
        "optimizer": pre["optimizer"],
        "clip_loss_weight_J_CLIP_W": CLIP_LOSS_WEIGHT_J_CLIP_W,
        "gradient_clipping": GRADIENT_CLIPPING,
        "clip_disambiguation": ("clip 1.0 is J_CLIP_W, an action-bound penalty on the loss "
                                "relu(|mean| - 1)^2 applied to the network output before "
                                "backward(). It is NOT gradient clipping, which is absent."),
        "pins": pre["pins"], "parent": parent,
        "deployable": False, "sigma_unresolved": True,
        "dataset": fd["inp"]["report"],
        "design": pre["design"], "folds": pre["folds"],
        "coverage_aggregate": pre["coverage_aggregate"],
        "selection_rule": pre["selection_rule"],
        "budget_readout": readout,
        "pooled_selection": sel,
        "pooled_curve": [float(x) for x in pooled],
        "fold_curves": curves,
        "folds_evaluated_at_e_star": folds,
        "pooled_validation_rmse_at_e_star": pooled_rmse,
        "july_protocol": {k: v for k, v in final_rep.items()
                          if k not in ("history", "history_first_last")},
        "final_fit": {"epochs": e_star, "selection_mode": final_rep["selection_mode"],
                      "history_first_last": final_rep["history_first_last"],
                      "metrics_all_1500_rows": final_m},
        "region_breakdown_DIAGNOSTIC_ONLY": region_metrics,
        "wait_reconstruction": {**WAIT_RECONSTRUCTION, "observed": wait_m,
                                "rows": int(wait_rows.size)},
        "functional_equivalence_25": equiv,
        "gates": gates, "failed": failed, "verdict": verdict,
        "scope": ("base-phase offline fit only. No rollout, no collection, no Markov phase, no "
                  "DAgger. A NO-GO forbids any rollout and forbids post-hoc hyperparameter "
                  "correction and any autonomous arm B."),
        "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}

    # the diagnostic artefacts are written whatever the verdict, so evidence survives a failure
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
                "deployable": False, "sigma_unresolved": True,
                "actor_label": ACTOR_LABEL,
                "controller_state_mask": {"active": True,
                                          "columns": list(B0.controller_columns()),
                                          "mask_value": float(B0.MASK_VALUE)},
                "clock_columns": list(B0.CLOCK_COLUMNS),
                "offline_verdict": verdict,
                "status": ("B1R2-A diagnostic artefact. NOT a candidate unless every binding gate "
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
                raise B1R2AError(f"no-clobber: {CANDIDATE_DIR} appeared during staging")
            staging.rename(CANDIDATE_DIR)
            promoted = C.rel(CANDIDATE_DIR)
        except BaseException:
            shutil.rmtree(staging, ignore_errors=True)
            raise

    return {"verdict": verdict, "failed": failed, "receipt_sha256": receipt_sha,
            "e_star": e_star, "budget_readout": readout,
            "pooled_validation_rmse": pooled_rmse,
            "fold_heldout_rmse": [f["heldout_at_e_star"]["aggregate_rmse"] for f in folds],
            "final_metrics": final_m, "wait_reconstruction": wait_m,
            "region_breakdown": region_metrics,
            "diagnostics_dir": C.rel(DIAG_DIR), "promoted_candidate": promoted}


def main(argv: Sequence[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="V26B B1R2-A budget arm")
    p.add_argument("--preflight", action="store_true")
    p.add_argument("--dry", action="store_true", help="design and rule only, no parent, no fit")
    p.add_argument("--authorized-stage", default=None)
    p.add_argument("--no-progress", action="store_true")
    a = p.parse_args(argv)
    if a.dry:
        verify_prereg(); verify_gates_not_relaxed()
        fd = folds_11()
        print(json.dumps({"mode": "dry", "design": {k: v for k, v in fd.items()
                                                    if k not in ("g", "inp", "folds")},
                          "folds": [{k: v for k, v in f.items() if not k.startswith("_")}
                                    for f in fd["folds"]]}, indent=2, default=str))
        return 0
    if a.preflight:
        print(json.dumps(preflight(), indent=2, default=str))
        return 0
    r = run(authorized_stage=a.authorized_stage, progress=not a.no_progress)
    print(json.dumps(r, indent=2, default=str))
    return 0 if r["verdict"] == "GO" else 1


if __name__ == "__main__":
    sys.exit(main())
