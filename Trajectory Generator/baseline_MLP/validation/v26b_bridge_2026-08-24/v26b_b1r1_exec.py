"""V26B B1R1 executor - leave-one-complete-cycle-out base fit of the masked 35D student.

Parent: EXCLUSIVELY the B0 masked transplant of the August V26 actor.  The B1 NO-GO candidate is
never init, never anchor, never data.  No rollout, no collection, no Markov phase.

Selection rule, PINNED BEFORE THE FIT (architect decision 2026-08-25):

  1. six independent fit trajectories, all starting BYTE-IDENTICAL from B0;
  2. every fold records its validation MSE at every epoch 1..400, with no early stop inside the fold;
  3. for each epoch the POOLED validation MSE is the row-weighted mean over the six folds;
  4. July's early-stopping rule is then applied DETERMINISTICALLY to that pooled curve: strict
     improvement by more than 1e-9 resets the stale counter, and the curve is considered stopped
     when it has not improved for 60 consecutive epochs;
  5. e* is the FIRST epoch attaining the pooled minimum;
  6. every fold is evaluated at THAT SAME e*, never at its own best epoch;
  7. the final fit restarts from B0 on all 1500 rows for exactly e* epochs, with no validation.

Because the fits are deterministic, running a fold for exactly e* epochs reproduces the state that
the 400-epoch run held at epoch e*.  Materialising the full curves first and applying the rule
afterwards therefore cannot change e*; the test proves the reproduction is bit-identical.

The fit itself is v26b_b_exec.fit_masked, reused UNCHANGED: the July loss, the per-step order, the
hard-zero projection of the clock and controller columns and the frozen log-std head are all its
own semantics, pinned against the production source.

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

import v26b_b_exec as EX  # noqa: E402      (fit_masked and metrics, reused unchanged)
import v26b_b0_masked35 as B0  # noqa: E402
import v26b_b1_base_fit as B1  # noqa: E402
import v26b_b1r1_loco_study as LO  # noqa: E402
import v26b_student as VS  # noqa: E402
import v26b_anchors as VA  # noqa: E402
import f0_common as C  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_refit as RF  # noqa: E402


class B1R1Error(RuntimeError):
    pass


STAGE = "V26B-B1R1-BASE-FIT-LOCO"

# --- pinned governing documents ------------------------------------------------------------------
AMENDMENT = HERE / "v26b_amendment_b_masked35.json"
PIN_AMENDMENT = "0830d82feb641043dffc4457c366a8dc6aa12979a3d5605e4a424903e19bf3ae"
JULY_PIN = HERE / "v26b_july_imitation_protocol_pin.json"
PIN_JULY = "0198e10965000ed908194083e23a682fec72fe5fc9c2c22e0f022a4f30a049d7"
LOCO_ADDENDUM = HERE / "v26b_addendum_b1r1_blocked5_comparator.json"
PIN_LOCO_ADDENDUM = "2781bc72d2b19e5a36eb6d31e6b73579c2ae62996f01288d5994aa732496641a"

# --- parent: B0 only ------------------------------------------------------------------------------
B0_DIR = B0.OUT_DIR
PIN_B0_ACTOR_DIGEST = "b2429a8217f99ed7ab4e61620480f797d4617c7713b857624e85c095bed57c55"
FORBIDDEN_PARENTS = ("B1_BASE35_MASKED", "S0D_35D_DISTILLED", "S1A_", "S1B_", "S1C1_", "S1C2Z_",
                     "REV4B", "REV4C", "REV4D", "REV4E", "V2_", "2026-07")

# --- selection rule, pinned before the fit ----------------------------------------------------------
EPOCHS_MAX = 400
PATIENCE = 60
POOLED_IMPROVEMENT_EPS = 1e-9

# --- binding offline gates, unchanged from B1 plus the WAIT reconstruction gate ----------------------
FOLD_HELDOUT_RMSE_MAX = 0.05
POOLED_VALIDATION_RMSE_MAX = 0.03
FINAL_RMSE_MAX = 0.02
FINAL_PER_JOINT_RMSE_MAX = 0.03
FINAL_PER_JOINT_MAXABS_MAX = 0.15
WAIT_RECONSTRUCTION = {"aggregate_rmse_max": 0.02, "per_joint_rmse_max": 0.03,
                       "per_joint_max_abs_max": 0.15,
                       "statement": "RECONSTRUCTION on training rows, explicitly NOT a "
                                    "generalisation result and never to be reported as one"}

OUT_DIR = VA.OUT_ROOT / "candidates" / "B1R1_BASE35_LOCO"
RECEIPT_NAME = "v26b_b1r1_receipt.json"
MODULE_DIR_NAME = "rl_module"


# ================================================================ provenance =====================

def verify_pins() -> dict[str, str]:
    out: dict[str, str] = {}
    for path, pin, key in ((AMENDMENT, PIN_AMENDMENT, "amendment_b_masked35"),
                           (JULY_PIN, PIN_JULY, "july_protocol_pin"),
                           (LOCO_ADDENDUM, PIN_LOCO_ADDENDUM, "loco_comparator_addendum")):
        got = C.sha256_file(path)
        if got != pin:
            raise B1R1Error(f"{key} sha {got} != pinned {pin}")
        out[key] = got
    if any(m in sys.modules for m in B0.SUPERSEDED_MODULES):
        raise B1R1Error("a superseded 25D module is loaded; the quarantine forbids it")
    return out


def load_parent() -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    """The ONLY admissible parent is B0. B1 and every diagnostic actor are refused by name."""
    module = B0_DIR / MODULE_DIR_NAME
    s = str(module)
    for bad in FORBIDDEN_PARENTS:
        if bad in s:
            raise B1R1Error(f"forbidden parent {bad!r} in {s}")
    p = module / "module_state.pkl"
    if not p.is_file():
        raise B1R1Error(f"B0 is not materialised: {C.rel(p)} is missing")
    with p.open("rb") as fh:
        st = {k: np.asarray(v) for k, v in pickle.load(fh).items()}
    digest = RF.actor_state_digest(st)
    if digest != PIN_B0_ACTOR_DIGEST:
        raise B1R1Error(f"B0 actor digest {digest} != pinned {PIN_B0_ACTOR_DIGEST}")
    RF.validate_init_state(st, expected_actor_digest=None, width=B0.ACTOR_WIDTH)
    B0.assert_masked_columns_zero(st, "B0 parent")
    B0.assert_clock_columns_zero(st, "B0 parent")
    return st, {"module": C.rel(module), "actor_digest": digest,
                "module_state_sha256": C.sha256_file(p),
                "role": "the ONLY parent; every fold and the final fit start byte-identical from it"}


# ================================================================ the pinned selection rule ======

def pooled_curve(fold_mse: Sequence[Sequence[float]], rows: Sequence[int]) -> list[float]:
    """Row-weighted mean validation MSE per epoch over the folds."""
    if not fold_mse:
        raise B1R1Error("no fold curve supplied")
    n = len(fold_mse[0])
    if any(len(c) != n for c in fold_mse):
        raise B1R1Error("the fold curves have different lengths")
    if len(rows) != len(fold_mse):
        raise B1R1Error("one row count per fold is required")
    w = np.asarray(rows, dtype=np.float64)
    if np.any(w <= 0):
        raise B1R1Error("row counts must be positive")
    M = np.asarray(fold_mse, dtype=np.float64)
    if not np.all(np.isfinite(M)):
        raise B1R1Error("non-finite validation MSE in a fold curve")
    return (M * w[:, None]).sum(axis=0) / w.sum()


def pooled_best_epoch(pooled: Sequence[float], *, patience: int = PATIENCE,
                      eps: float = POOLED_IMPROVEMENT_EPS) -> dict[str, Any]:
    """July's early-stopping rule applied to the pooled curve, deterministically.

    Strict improvement by more than eps resets the stale counter, exactly as production does with
    `val_mse < best_validation_mse - 1e-9`.  The curve is considered stopped when it has not
    improved for `patience` consecutive epochs.  e* is the FIRST epoch attaining the minimum, which
    the strict comparison already guarantees among equal values.
    """
    v = [float(x) for x in pooled]
    if not v:
        raise B1R1Error("empty pooled curve")
    if patience < 1:
        raise B1R1Error("patience must be >= 1")
    best = float("inf")
    best_epoch = 0
    stale = 0
    stopped_at = None
    for e, x in enumerate(v, start=1):
        if x < best - eps:
            best = x
            best_epoch = e
            stale = 0
        else:
            stale += 1
        if stale >= patience:
            stopped_at = e
            break
    if best_epoch < 1:
        raise B1R1Error("the pooled curve never improved on its first value")
    return {"e_star": int(best_epoch), "pooled_mse_at_e_star": float(best),
            "stopped_at_epoch": (None if stopped_at is None else int(stopped_at)),
            "epochs_available": len(v),
            "rule": "first epoch attaining the pooled minimum; stale counter reset only on strict "
                    f"improvement greater than {eps}; stop after {patience} stale epochs"}


# ================================================================ the fit =========================

def build_inputs() -> dict[str, Any]:
    d = B1.build_dataset()
    obs = np.asarray(d["observations_masked"], dtype=np.float32)
    raw = np.asarray(d["observations_raw"], dtype=np.float32)
    act = np.asarray(d["actions"], dtype=np.float32)
    traj = np.asarray(d["trajectory_id"], dtype=np.int64)
    split = LO.loco_split(raw, traj)
    if split["n_folds"] != 6:
        raise B1R1Error(f"expected six folds, got {split['n_folds']}")
    wait = np.where(raw[:, LO.WAIT_COLUMN] > 0.5)[0]
    if wait.size != 187:
        raise B1R1Error(f"expected 187 WAIT_HS rows, found {wait.size}")
    return {"obs": obs, "raw": raw, "act": act, "traj": traj, "split": split,
            "wait_rows": wait, "report": d["report"]}


def fold_curves(init: Mapping[str, np.ndarray], inp: Mapping[str, Any],
                *, progress: bool = True) -> list[dict[str, Any]]:
    """Pass one: every fold runs the full epoch budget with NO early stop, recording its curve."""
    out = []
    for f in inp["split"]["folds"]:
        state, rep = EX.fit_masked(init, inp["obs"], inp["act"],
                                   train_idx=f["_train"], val_idx=f["_val"],
                                   epochs=EPOCHS_MAX, patience=None, progress=False)
        curve = [float(h["val_mse"]) for h in rep["history"]]
        if len(curve) != EPOCHS_MAX:
            raise B1R1Error(f"fold {f['fold']}: {len(curve)} epochs recorded, expected {EPOCHS_MAX}")
        out.append({"fold": int(f["fold"]), "name": f["name"], "cycle": int(f["cycle"]),
                    "validation_rows": int(f["validation_rows"]),
                    "train_rows": int(f["train_rows"]),
                    "wait_rows_in_train": int(f["wait_rows_in_train"]),
                    "own_best_epoch_not_used": int(rep["best_epoch"]),
                    "curve_mse": curve})
        if progress:
            print(json.dumps({"pass": 1, "fold": int(f["fold"]),
                              "min_mse": float(min(curve)),
                              "argmin": int(int(np.argmin(curve)) + 1)}), flush=True)
    return out


def fold_states_at(init: Mapping[str, np.ndarray], inp: Mapping[str, Any], e_star: int,
                   *, progress: bool = True) -> list[dict[str, Any]]:
    """Pass two: re-run every fold for exactly e* epochs and evaluate it on its own held-out cycle."""
    out = []
    for f in inp["split"]["folds"]:
        state, rep = EX.fit_masked(init, inp["obs"], inp["act"],
                                   train_idx=f["_train"], val_idx=None,
                                   epochs=e_star, patience=None, progress=False)
        B0.assert_masked_columns_zero(state, f"fold {f['fold']} at e*")
        B0.assert_clock_columns_zero(state, f"fold {f['fold']} at e*")
        B0.assert_no_masked_update(init, state)
        before = EX.metrics(init, inp["obs"], inp["act"], f["_val"])
        after = EX.metrics(state, inp["obs"], inp["act"], f["_val"])
        out.append({"fold": int(f["fold"]), "name": f["name"], "cycle": int(f["cycle"]),
                    "evaluated_at_epoch": int(e_star),
                    "heldout_before_b0": before, "heldout_at_e_star": after,
                    "improved_over_b0": bool(after["aggregate_rmse"] < before["aggregate_rmse"]),
                    "epochs_run": int(rep["epochs_run"])})
        if progress:
            print(json.dumps({"pass": 2, "fold": int(f["fold"]),
                              "heldout_rmse": after["aggregate_rmse"]}), flush=True)
    return out


# ================================================================ driver ==========================

def preflight() -> dict[str, Any]:
    """No-write, fail-closed. Everything the fit needs must exist and match its pin."""
    pins = verify_pins()
    init, parent = load_parent()
    inp = build_inputs()
    q = LO.quantify(inp["split"], inp["raw"], inp["act"], inp["traj"])
    if q["aggregate"]["any_temporal_leakage"]:
        raise B1R1Error("the split leaks temporally")
    if q["aggregate"]["any_signature_missing_from_train"]:
        raise B1R1Error("a discrete signature is missing from a training set")
    if q["aggregate"]["min_temporal_distance"] != LO.EMBARGO_ROWS + 1:
        raise B1R1Error("the embargo is not honoured")
    for f in inp["split"]["folds"]:
        if np.any(inp["raw"][f["_val"], LO.WAIT_COLUMN] > 0.5):
            raise B1R1Error(f"fold {f['fold']} validates WAIT_HS rows, which is forbidden")
    equiv = B0.functional_equivalence_25(init, inp["raw"])
    blockers = []
    if OUT_DIR.exists():
        blockers.append(f"no-clobber: {C.rel(OUT_DIR)} exists")
    return {"verdict": ("GO" if not blockers else "BLOCKED"), "stage": STAGE, "blockers": blockers,
            "pins": pins, "parent": parent,
            "contract": {"actor_width": B0.ACTOR_WIDTH,
                         "masked_columns": list(B0.controller_columns()),
                         "clock_columns": list(B0.CLOCK_COLUMNS),
                         "support_columns_23d": list(LO.SUPPORT_COLUMNS)},
            "dataset": inp["report"], "wait_rows": int(inp["wait_rows"].size),
            "split": {k: v for k, v in inp["split"].items() if k != "folds"},
            "split_quantified": q["aggregate"],
            "functional_equivalence_25": equiv,
            "selection_rule": {"epochs_max": EPOCHS_MAX, "patience": PATIENCE,
                               "eps": POOLED_IMPROVEMENT_EPS,
                               "rule": "pooled row-weighted validation MSE; e* is the first epoch "
                                       "attaining its minimum under July's stale rule; every fold "
                                       "is evaluated at that same e*"},
            "gates": {"fold_heldout_rmse_max": FOLD_HELDOUT_RMSE_MAX,
                      "pooled_validation_rmse_max": POOLED_VALIDATION_RMSE_MAX,
                      "final_rmse_max": FINAL_RMSE_MAX,
                      "final_per_joint_rmse_max": FINAL_PER_JOINT_RMSE_MAX,
                      "final_per_joint_max_abs_max": FINAL_PER_JOINT_MAXABS_MAX,
                      "wait_reconstruction": WAIT_RECONSTRUCTION}}


def run(*, authorized_stage: str | None, progress: bool = True) -> dict[str, Any]:
    if authorized_stage != STAGE:
        raise B1R1Error(f"requires --authorized-stage {STAGE}; got {authorized_stage!r}")
    pre = preflight()
    if pre["blockers"]:
        raise B1R1Error(f"preflight BLOCKED: {pre['blockers']}")
    init, parent = load_parent()
    inp = build_inputs()

    curves = fold_curves(init, inp, progress=progress)
    pooled = pooled_curve([c["curve_mse"] for c in curves],
                          [c["validation_rows"] for c in curves])
    sel = pooled_best_epoch(pooled)
    e_star = sel["e_star"]
    folds = fold_states_at(init, inp, e_star, progress=progress)
    pooled_rmse = float(np.sqrt(sel["pooled_mse_at_e_star"]))

    final_state, final_rep = EX.fit_masked(init, inp["obs"], inp["act"],
                                           train_idx=np.arange(len(inp["obs"])), val_idx=None,
                                           epochs=e_star, patience=None, progress=progress)
    final_m = EX.metrics(final_state, inp["obs"], inp["act"], None)
    wait_m = EX.metrics(final_state, inp["obs"], inp["act"], inp["wait_rows"])

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
    equiv = B0.functional_equivalence_25(final_state, inp["raw"])

    gates = {
        "integrity_invariants": {"binding": True, **integrity,
                                 "pass": all(v for v in integrity.values() if isinstance(v, bool))},
        "functional_equivalence_25": {"binding": True, "bit_identical": equiv["bit_identical"],
                                      "pass": bool(equiv["bit_identical"])},
        "every_fold_improves_over_b0": {"binding": True,
                                        "per_fold": [f["improved_over_b0"] for f in folds],
                                        "pass": all(f["improved_over_b0"] for f in folds)},
        "fold_heldout_rmse": {"binding": True, "threshold": FOLD_HELDOUT_RMSE_MAX,
                              "observed": [f["heldout_at_e_star"]["aggregate_rmse"] for f in folds],
                              "pass": all(f["heldout_at_e_star"]["aggregate_rmse"] <= FOLD_HELDOUT_RMSE_MAX
                                          for f in folds)},
        "pooled_validation_rmse": {"binding": True, "threshold": POOLED_VALIDATION_RMSE_MAX,
                                   "observed": pooled_rmse,
                                   "pass": bool(pooled_rmse <= POOLED_VALIDATION_RMSE_MAX)},
        "final_aggregate_rmse": {"binding": True, "threshold": FINAL_RMSE_MAX,
                                 "observed": final_m["aggregate_rmse"],
                                 "pass": bool(final_m["aggregate_rmse"] <= FINAL_RMSE_MAX)},
        "final_per_joint_rmse": {"binding": True, "threshold": FINAL_PER_JOINT_RMSE_MAX,
                                 "observed": {k: v["rmse"] for k, v in final_m["per_joint"].items()},
                                 "pass": all(v["rmse"] <= FINAL_PER_JOINT_RMSE_MAX
                                             for v in final_m["per_joint"].values())},
        "final_per_joint_max_abs": {"binding": True, "threshold": FINAL_PER_JOINT_MAXABS_MAX,
                                    "observed": {k: v["max_abs"] for k, v in final_m["per_joint"].items()},
                                    "pass": all(v["max_abs"] <= FINAL_PER_JOINT_MAXABS_MAX
                                                for v in final_m["per_joint"].values())},
        "wait_reconstruction_NOT_A_GENERALISATION_GATE": {
            "binding": True, **WAIT_RECONSTRUCTION,
            "observed": {"aggregate_rmse": wait_m["aggregate_rmse"],
                         "per_joint": wait_m["per_joint"]},
            "rows": int(inp["wait_rows"].size),
            "pass": bool(wait_m["aggregate_rmse"] <= WAIT_RECONSTRUCTION["aggregate_rmse_max"]
                         and all(v["rmse"] <= WAIT_RECONSTRUCTION["per_joint_rmse_max"]
                                 for v in wait_m["per_joint"].values())
                         and all(v["max_abs"] <= WAIT_RECONSTRUCTION["per_joint_max_abs_max"]
                                 for v in wait_m["per_joint"].values()))},
    }
    failed = [k for k, v in gates.items() if not v["pass"]]
    verdict = "GO" if not failed else "NO-GO"

    staging = OUT_DIR.parent / (OUT_DIR.name + ".staging")
    if staging.exists():
        shutil.rmtree(staging)
    sm = staging / MODULE_DIR_NAME
    sm.mkdir(parents=True, exist_ok=False)
    csha = None
    try:
        names35, _, mshas = VS.pinned_names()
        with (sm / "module_state.pkl").open("wb") as fh:
            pickle.dump({k: np.asarray(v) for k, v in final_state.items()}, fh,
                        protocol=pickle.HIGHEST_PROTOCOL)
        for f in ("metadata.json", "class_and_ctor_args.pkl"):
            shutil.copy2(B0_DIR / MODULE_DIR_NAME / f, sm / f)
        manifest = {"schema_version": 1, "actor_feature_names": names35, "actor_feature_count": 35,
                    "actor_digest": RF.actor_state_digest(final_state),
                    "module_state_sha256": C.sha256_file(sm / "module_state.pkl"),
                    "manifest35_sha256": mshas["manifest35_sha256"],
                    "exploration_sigma": [B0.SIGMA_PLACEHOLDER] * R.ACTION_DIM,
                    "sigma_note": B0.SIGMA_STATEMENT,
                    "derived_from": C.rel(B0_DIR / MODULE_DIR_NAME),
                    "source_actor_digest": PIN_B0_ACTOR_DIGEST,
                    "deployable": False, "sigma_unresolved": True,
                    "actor_label": "B1R1_BASE35_LOCO",
                    "controller_state_mask": {"active": True,
                                              "columns": list(B0.controller_columns()),
                                              "mask_value": float(B0.MASK_VALUE)},
                    "clock_columns": list(B0.CLOCK_COLUMNS),
                    "offline_verdict": verdict,
                    "status": "BASE-PHASE CANDIDATE (B1R1); never promoted, never deployable; "
                              "no rollout has been run"}
        C.write_json(sm / "actor_feature_manifest.json", manifest, clobber=False)
        receipt = {"schema": "v26b_b1r1_base_fit.1", "authorized_stage": STAGE,
                   "pins": pre["pins"], "parent": parent,
                   "deployable": False, "sigma_unresolved": True,
                   "dataset": inp["report"],
                   "split": {k: v for k, v in inp["split"].items() if k != "folds"},
                   "split_quantified": pre["split_quantified"],
                   "selection_rule": pre["selection_rule"],
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
                   "final_history": final_rep["history"],
                   "wait_reconstruction": {**WAIT_RECONSTRUCTION, "observed": wait_m,
                                           "rows": int(inp["wait_rows"].size)},
                   "functional_equivalence_25": equiv,
                   "gates": gates, "failed": failed, "verdict": verdict,
                   "output_module": C.rel(OUT_DIR / MODULE_DIR_NAME),
                   "output_files_sha256": {p.name: C.sha256_file(p) for p in sorted(sm.iterdir())},
                   "scope": "base-phase offline fit only. No rollout, no collection, no Markov "
                            "phase, no promotion. A NO-GO forbids any rollout and forbids post-hoc "
                            "hyperparameter correction",
                   "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}
        C.write_json(staging / RECEIPT_NAME, receipt, clobber=False)
        csha = C.sha256_file(staging / RECEIPT_NAME)
        if OUT_DIR.exists():
            raise B1R1Error(f"no-clobber: {OUT_DIR} appeared during staging")
        staging.rename(OUT_DIR)
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return {"verdict": verdict, "failed": failed, "receipt_sha256": csha,
            "e_star": e_star, "pooled_selection": sel,
            "pooled_validation_rmse": pooled_rmse,
            "fold_heldout_rmse": [f["heldout_at_e_star"]["aggregate_rmse"] for f in folds],
            "final_metrics": final_m, "wait_reconstruction": wait_m}


def main(argv: Sequence[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="V26B B1R1: leave-one-complete-cycle-out base fit")
    p.add_argument("--preflight", action="store_true")
    p.add_argument("--authorized-stage", default=None)
    p.add_argument("--no-progress", action="store_true")
    a = p.parse_args(argv)
    if a.preflight or a.authorized_stage is None:
        print(json.dumps(preflight(), indent=2, default=str))
        return 0
    print(json.dumps(run(authorized_stage=a.authorized_stage, progress=not a.no_progress),
                     indent=2, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
