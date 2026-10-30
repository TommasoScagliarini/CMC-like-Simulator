"""Tests for the corrected B1R1 leave-one-complete-cycle-out study. Never trains, never writes an actor."""
from __future__ import annotations
import inspect, json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_b1r1_loco_study as L  # noqa: E402
import v26b_b1r1_split_study as PREV  # noqa: E402
import v26b_b0_masked35 as B0  # noqa: E402
import v26b_b1_base_fit as B1  # noqa: E402

CHECKS = 0
def check(c, w):
    global CHECKS; assert c, w; CHECKS += 1
def expect(fn, exc, w):
    global CHECKS
    try: fn()
    except exc as e: CHECKS += 1; return e
    raise AssertionError(f"expected {exc.__name__}: {w}")


def main() -> int:
    d = B1.build_dataset()
    obs = np.asarray(d["observations_masked"], np.float32)
    act = np.asarray(d["actions"], np.float32)
    traj = np.asarray(d["trajectory_id"], np.int64)

    # --- correction 1: the support is 23D, not 25D -----------------------------------------------
    check(len(L.SUPPORT_COLUMNS) == 23 and list(L.SUPPORT_COLUMNS) == list(range(2, 25)),
          "the support is 23 columns, 2..24")
    check(not set(L.SUPPORT_COLUMNS) & set(B0.CLOCK_COLUMNS),
          "the two hard-zeroed clock columns are excluded explicitly, not by a variance filter")
    check(set(L.SUPPORT_COLUMNS) | set(B0.CLOCK_COLUMNS) == set(B0.kept_columns()),
          "support plus clock reconstructs the kept columns exactly")
    sup = L.support_matrix(obs)
    check(sup.shape == (1500, 23), "the support matrix is (1500, 23)")
    expect(lambda: L.support_matrix(np.zeros((4, 25))), L.LocoError, "a wrong width is refused")

    # --- correction 2: the scaler is fit on TRAIN only -------------------------------------------
    train = np.arange(0, 1000); val = np.arange(1000, 1500)
    sc = L.train_only_scaler(sup, train)
    check(np.allclose(sc["mu"], sup[train].mean(0)) and np.allclose(sc["sd"], sup[train].std(0)),
          "the scaler statistics come from the training rows only")
    check(not np.allclose(sc["mu"], sup.mean(0)),
          "and they DIFFER from the whole-dataset statistics, so the correction is material")
    sc2 = L.train_only_scaler(sup, np.arange(500, 1500))
    check(not np.allclose(sc["mu"], sc2["mu"]), "a different training set gives a different scaler")
    src = (HERE / "v26b_b1r1_loco_study.py").read_text()
    body = inspect.getsource(L.support_and_baseline)
    check("train_only_scaler(sup, train)" in body and ".mean(0)" not in body,
          "support_and_baseline never standardises with whole-dataset statistics")
    expect(lambda: L.train_only_scaler(np.zeros((10, 23)), np.arange(5)), L.LocoError,
           "a training set with no varying column is refused")

    # --- correction 3: exact discrete signatures over (17,18,19,20,21,24) --------------------------
    check(L.DISCRETE_SIGNATURE_COLUMNS == (17, 18, 19, 20, 21, 24)
          and L.CONTINUOUS_PHASE_COLUMNS == (22, 23),
          "the signature uses 17,18,19,20,21,24 and treats 22,23 as continuous")
    sig = L.discrete_signature(obs)
    check(len(sig) == 1500 and isinstance(sig[0], tuple) and len(sig[0]) == 6,
          "one hashable 6-tuple per row")
    check(len({s for s in sig}) == 5, "the dataset contains exactly 5 distinct discrete signatures")
    cov = L.signature_coverage(obs, val, train)
    check(set(cov) >= {"pass", "missing_signatures", "distinct_signatures_in_validation"},
          "signature coverage reports what is missing")
    only = np.where(obs[:, 17] > 0.5)[0]
    other = np.where(obs[:, 17] <= 0.5)[0]
    bad = L.signature_coverage(obs, only[:50], other)
    check(bad["pass"] is False and bad["validation_rows_whose_signature_is_absent_from_train"] == 50,
          "holding out WAIT rows against a WAIT-free train is detected as a missing signature")
    check(L.signature_coverage(obs, other[:50], other[50:])["pass"] is True,
          "a covered validation passes")

    # --- the six cycles are exactly STANCE+SWING pairs of the segmentation --------------------------
    segs = PREV.segments(obs, traj)
    for cyc in L.CYCLES:
        tsegs = [s for s in segs if s["trajectory"] == cyc["trajectory"]]
        st = [s for s in tsegs if s["local_lo"] == cyc["local_lo"]]
        check(len(st) == 1 and st[0]["state"] == "STANCE",
              f"{cyc['name']} cycle {cyc['cycle']} starts at a STANCE segment boundary")
        k = tsegs.index(st[0])
        check(k + 1 < len(tsegs) and tsegs[k + 1]["state"] == "SWING"
              and tsegs[k + 1]["local_hi"] == cyc["local_hi"],
              f"{cyc['name']} cycle {cyc['cycle']} ends at the following SWING segment boundary")
    check(len(L.CYCLES) == 6, "six complete cycles, two per trajectory")

    # --- the split ----------------------------------------------------------------------------------
    sp = L.loco_split(obs, traj)
    check(sp["n_folds"] == 6 and sp["embargo_rows"] == 20, "six folds with an embargo of 20")
    q = L.quantify(sp, obs, act, traj)
    for f in q["per_fold"]:
        check(f["temporal"]["min_temporal_distance_same_trajectory"] == 21,
              f"fold {f['fold']}: the minimum temporal distance is the embargo plus one")
        check(f["temporal"]["validation_rows_with_an_immediate_training_neighbour"] == 0,
              f"fold {f['fold']}: no validation row has an immediate training neighbour")
        check(f["discrete_signature_coverage"]["pass"],
              f"fold {f['fold']}: every validation signature is present in training")
        check(f["support"]["support_dimensions"] == 23, f"fold {f['fold']}: 23 live support dimensions")
        check(f["support"]["scaler"] == "fit on TRAINING ROWS ONLY", "the scaler is declared")
        check(f["wait_rows_in_train"] >= int(0.80 * f["wait_rows_total"]),
              f"fold {f['fold']}: the WAIT training floor holds")
    check(all(np.all(obs[f["_val"], 17] <= 0.5) for f in sp["folds"]),
          "NO held-out cycle contains a single WAIT_HS row")
    lost = {f["fold"]: f["wait_rows_lost_to_embargo"] for f in sp["folds"]}
    check(lost[0] == 20 and all(v == 0 for k, v in lost.items() if k != 0),
          "only the fold whose cycle abuts the WAIT segment loses WAIT rows, and it loses 20")
    allv = np.concatenate([f["_val"] for f in sp["folds"]])
    check(len(allv) == len(np.unique(allv)) == 910, "the six validation sets are disjoint, 910 rows")
    sp2 = L.loco_split(obs, traj)
    check(all(np.array_equal(a["_val"], b["_val"]) for a, b in zip(sp["folds"], sp2["folds"])),
          "the split is deterministic")
    expect(lambda: L.loco_split(obs, traj, embargo=-1), L.LocoError, "a negative embargo is refused")
    old = L.CYCLES
    try:
        L.CYCLES = ({"trajectory": 0, "name": "x", "cycle": 1, "local_lo": 0, "local_hi": 50},)
        expect(lambda: L.loco_split(obs, traj), L.LocoError,
               "a cycle containing WAIT rows is refused: WAIT must never be validated")
    finally:
        L.CYCLES = old
    old2 = L.WAIT_TRAIN_MIN_FRACTION
    try:
        L.WAIT_TRAIN_MIN_FRACTION = 0.999
        expect(lambda: L.loco_split(obs, traj), L.LocoError, "the WAIT training floor fails closed")
    finally:
        L.WAIT_TRAIN_MIN_FRACTION = old2

    # --- the formal WAIT verification -----------------------------------------------------------------
    wf = L.wait_formal_verification(obs, traj)
    check(wf["occurrences"] == 1 and wf["runs"] == [(0, 186)] and wf["rows"] == 187
          and wf["trajectories_containing_it"] == [0],
          "WAIT_HS occurs exactly once, rows 0..186, in one trajectory")
    check(wf["recurrence_fraction_lag_gt_20_closer_than_lag1_median"] == 0.0,
          "the segment never revisits a state: recurrence is exactly zero")
    check(wf["by_embargo"]["embargo_20"]["ratio_to_lag1_median"] > 5.0,
          "at embargo 20 the nearest surviving WAIT neighbour is more than five times farther "
          "than an adjacent row")
    check("No in-distribution WAIT heldout exists" in wf["verdict"], "the verdict is stated")

    # --- the WAIT gate is a reconstruction gate, not a generalisation claim -----------------------------
    g = L.WAIT_GATE
    check(g["aggregate_rmse_max"] == 0.02 and g["per_joint_rmse_max"] == 0.03
          and g["per_joint_max_abs_max"] == 0.15,
          "the WAIT gate reuses the all-data thresholds and relaxes nothing")
    check("never be reported as evidence of WAIT generalisation" in g["honesty_statement"],
          "the gate states what it is not")

    # --- comparator preserved -----------------------------------------------------------------------------
    check("v26b_b1r1_split_study" in src, "the superseded proposal is kept as a comparator")
    r = L.run_study(write=False)
    check(set(r["comparison"]) == {"loco_6fold", "superseded_blocked_5fold_recomputed",
                                   "leave_one_trajectory_out", "july_random_80_20"},
          "all four splits are compared under the corrected instruments")
    a6 = r["comparison"]["loco_6fold"]["aggregate"]
    a5 = r["comparison"]["superseded_blocked_5fold_recomputed"]["aggregate"]
    check(a6["any_temporal_leakage"] is False and a6["any_signature_missing_from_train"] is False,
          "the six-fold split leaks nothing and covers every signature")
    check(r["comparison"]["july_random_80_20"]["aggregate"]["any_temporal_leakage"] is True,
          "the July random split still leaks under the corrected instruments")
    check(r["comparison"]["leave_one_trajectory_out"]["aggregate"]["any_signature_missing_from_train"] is True,
          "leave-one-trajectory-out still misses a signature")
    check(a6["dropped_median"] < a5["dropped_median"] / 5,
          "the six-fold embargo costs less than a fifth of the superseded one")

    # --- nothing trains ---------------------------------------------------------------------------------------
    check(not any(t in src for t in ("torch", "optim", "backward", "fit_masked", "adapt_actor",
                                     "subprocess", "rollout_eval", "module_state.pkl")),
          "the study imports no training machinery and writes no actor")
    check("os.system" not in src and "os.sep" not in src and "from pathlib import Path" in src,
          "pathlib only, no shell, no os-specific path handling")

    print(json.dumps({"selftest": "PASS", "checks": CHECKS}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
