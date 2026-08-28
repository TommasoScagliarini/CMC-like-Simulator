"""Tests for the B1R1 read-only split study. The script must never train or modify an actor."""
from __future__ import annotations
import json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_b1r1_split_study as S  # noqa: E402
import v26b_b0_masked35 as B0  # noqa: E402

CHECKS = 0
def check(c, w):
    global CHECKS; assert c, w; CHECKS += 1
def expect(fn, exc, w):
    global CHECKS
    try: fn()
    except exc as e: CHECKS += 1; return e
    raise AssertionError(f"expected {exc.__name__}: {w}")


def main() -> int:
    d = S.load_dataset()
    obs, traj = d["obs_raw"], d["traj"]
    check(obs.shape == (1500, 35) and set(np.unique(traj).tolist()) == {0, 1, 2},
          "all three trajectories are kept: 1500 rows over 3 trajectories")

    # --- inventory -------------------------------------------------------------------------------
    inv = S.inventory(obs, traj)
    check(inv["live_feature_indices"] == list(range(17, 25))
          and inv["live_feature_names"][0] == "phase_fsm_wait_hs"
          and inv["live_feature_names"][-1] == "phase_cycle_progress_credit",
          "the inventory covers exactly the live FSM features 17..24")
    w = inv["wait_hs_summary"]
    check(w["total_rows"] == 187 and w["trajectories_containing_it"] == ["minus020"],
          "phase_fsm_wait_hs=1 appears in ONE trajectory only, 187 rows")
    check(inv["per_trajectory"]["minus020"]["wait_hs"]["contiguous"] is True
          and inv["per_trajectory"]["minus020"]["wait_hs"]["first"] == 1
          and inv["per_trajectory"]["minus020"]["wait_hs"]["last"] == 187,
          "it is a single contiguous head segment, rows 1..187")
    check(inv["per_trajectory"]["nominal"]["state_counts"]["WAIT_HS"] == 0
          and inv["per_trajectory"]["plus020"]["state_counts"]["WAIT_HS"] == 0,
          "neither nominal nor plus020 contains a single WAIT_HS row")
    for t in ("minus020", "nominal", "plus020"):
        sc = inv["per_trajectory"][t]["state_counts"]
        check(sum(sc.values()) == 500, f"{t}: the three one-hot states partition its 500 rows")
        check(len(inv["per_trajectory"][t]["transitions"]) >= 5, f"{t}: transitions are recorded")
    bad = obs.copy(); bad[0, 17] = bad[0, 18] = bad[0, 19] = 0.0
    expect(lambda: S.fsm_labels(bad), S.StudyError, "a row with no FSM one-hot fails closed")

    # --- segments and blocks ------------------------------------------------------------------------
    segs = S.segments(obs, traj)
    check(len(segs) == 22, "22 maximal FSM segments")
    check(sum(s["rows"] for s in segs) == 1500, "the segments partition every row")
    from collections import Counter
    check(dict(Counter(s["state"] for s in segs)) == {"WAIT_HS": 1, "STANCE": 11, "SWING": 10},
          "one WAIT_HS segment, eleven STANCE, ten SWING")
    b40 = S.blocks(segs, 40)
    check(len(b40) == 51 and sum(1 for b in b40 if b["state"] == "WAIT_HS") == 5,
          "block_max 40 gives 51 blocks and splits WAIT_HS into exactly 5")
    check(all(b["rows"] <= 40 for b in b40), "no block exceeds the maximum")
    check(sum(b["rows"] for b in b40) == 1500, "the blocks partition every row")
    expect(lambda: S.blocks(segs, 0), S.StudyError, "a non-positive block size is refused")

    # --- candidate split ------------------------------------------------------------------------------
    c = S.candidate_split(obs, traj)
    check(c["block_max_rows"] == 40 and c["n_folds"] == 5 and c["embargo_rows"] == 20,
          "the preregistered parameters are block 40, 5 folds, embargo 20")
    check(c["blocks_per_state"]["WAIT_HS"] == 5, "five WAIT_HS blocks for five folds")
    wait_per_fold = Counter(b["fold"] for b in c["blocks"] if b["state"] == "WAIT_HS")
    check(sorted(wait_per_fold.values()) == [1, 1, 1, 1, 1],
          "every fold receives exactly one WAIT_HS block, so every fold TESTS that state")
    for f in c["folds"]:
        check(f["states_in_validation_missing_from_train"] == [],
              f"fold {f['fold']}: no state is in validation but absent from training")
        check(f["train_state_counts"]["WAIT_HS"] > 0,
              f"fold {f['fold']}: WAIT_HS is present in training")
        check(f["validation_rows"] > 0 and f["train_rows"] > 0, "no degenerate fold")
    allv = np.concatenate([f["_val"] for f in c["folds"]])
    check(len(allv) == len(np.unique(allv)), "the folds' validation sets are disjoint")
    check(len(np.unique(allv)) == 1500, "every row is validated exactly once")
    q = S.quantify(c, d)
    check(q["aggregate"]["any_state_missing_from_train"] is False, "no missing state overall")
    check(q["aggregate"]["any_temporal_leakage"] is False, "no temporal leakage")
    check(q["aggregate"]["min_temporal_distance_over_folds"] == 21,
          "the minimum temporal distance equals the embargo plus one")
    for f in q["per_fold"]:
        check(f["temporal"]["validation_rows_with_an_immediate_training_neighbour"] == 0,
              f"fold {f['fold']}: no validation row has an immediate training neighbour")
        check(set(f["nn_label_baseline"]["per_joint_rmse"]) == {"knee", "ankle"},
              "the nearest-neighbour label baseline is reported per joint")
        check("nn_distance_median" in f["continuous_support"]
              and "fraction_beyond_train_internal_p99" in f["continuous_support"],
              "the continuous support uses the same instrument as B1")

    # --- determinism ------------------------------------------------------------------------------------
    c2 = S.candidate_split(obs, traj)
    check(all(np.array_equal(a["_val"], b["_val"]) and np.array_equal(a["_train"], b["_train"])
              for a, b in zip(c["folds"], c2["folds"])), "the split is deterministic across calls")
    check([b["fold"] for b in c["blocks"]] == [b["fold"] for b in c2["blocks"]],
          "block-to-fold assignment is deterministic")
    src = (HERE / "v26b_b1r1_split_study.py").read_text()
    # the CALL SITE, not the word: "permutation" also occurs in the comparator's docstring.
    import inspect
    call_lines = [i for i, ln in enumerate(src.splitlines(), 1) if ".permutation(" in ln]
    rnd_src = inspect.getsource(S.random_split)
    check(len(call_lines) == 1 and ".permutation(" in rnd_src,
          f"the only .permutation( call site is inside random_split (found at lines {call_lines})")
    for fn in (S.candidate_split, S.blocks, S.segments, S.loto_split):
        check(".permutation(" not in inspect.getsource(fn),
              f"{fn.__name__} contains no randomisation")
    expect(lambda: S.candidate_split(obs, traj, n_folds=1), S.StudyError, "n_folds 1 refused")
    expect(lambda: S.candidate_split(obs, traj, embargo=-1), S.StudyError, "negative embargo refused")

    # --- fail-closed on state coverage ---------------------------------------------------------------------
    e = expect(lambda: S.candidate_split(obs, traj, n_folds=5, block_max=400),
               S.StudyError, "a block size that keeps WAIT_HS whole leaves a fold without it")
    check("validation but not in training" in str(e), "the guard names the violation")

    # --- comparators -----------------------------------------------------------------------------------------
    ql = S.quantify(S.loto_split(traj, obs), d)
    check(ql["aggregate"]["any_state_missing_from_train"] is True,
          "leave-one-trajectory-out DOES leave a state out of training")
    miss = [f for f in ql["per_fold"] if f["states_in_validation_missing_from_train"]]
    check(len(miss) == 1 and miss[0]["holdout_trajectory"] == "minus020"
          and miss[0]["states_in_validation_missing_from_train"] == ["WAIT_HS"],
          "and it is exactly WAIT_HS on the minus020 fold")
    qr = S.quantify(S.random_split(traj, obs), d)
    check(qr["aggregate"]["any_temporal_leakage"] is True,
          "the July random 80/20 DOES leak temporally")
    check(qr["per_fold"][0]["temporal"]["min_temporal_distance_same_trajectory"] == 1,
          "its minimum temporal distance is one step")
    check(qr["per_fold"][0]["temporal"]["fraction"] > 0.9,
          "over 90% of its validation rows have an immediate training neighbour")

    # --- declarations ---------------------------------------------------------------------------------------------
    check(S.BEST_EPOCH_RULE["epochs_max"] == 400 and S.BEST_EPOCH_RULE["patience"] == 60,
          "the epoch budget and patience are July's pinned values")
    check("POOLED-FOLD EARLY STOPPING" in S.BEST_EPOCH_RULE["rule"]
          and "no B1 result" in S.BEST_EPOCH_RULE["not_justified_by"],
          "one rule, and it is not justified by any B1 result")
    g = S.CANDIDATE_GATES
    check(g["per_fold_heldout_aggregate_rmse"]["threshold"] == 0.05
          and g["pooled_row_weighted_validation_rmse"]["threshold"] == 0.03
          and g["final_all_data_aggregate_rmse"]["threshold"] == 0.02
          and g["final_per_joint_rmse"]["threshold"] == 0.03
          and g["final_per_joint_max_abs"]["threshold"] == 0.15,
          "every carried-over gate keeps its B1 numeric value")
    check(g["per_fsm_state_heldout_rmse"]["threshold"] == 0.05
          and g["per_fsm_state_heldout_rmse"]["unchanged_from_b1"] is False,
          "the new per-state gate is added, and it is stricter, not looser")
    check("no threshold is relaxed" in g["note"], "nothing is relaxed for B1")
    check(S.DECORRELATION_TABLE[1] == 6.5 and S.DECORRELATION_TABLE[20] == 67.5
          and "50% crossing is at lag 16" in S.EMBARGO_RULE,
          "the embargo is fixed by a measured decorrelation criterion")

    # --- the script trains nothing and writes no actor ---------------------------------------------------------------
    check(not any(t in src for t in ("torch", "optim", "backward", "fit_masked", "adapt_actor",
                                     "subprocess", "rollout_eval", "module_state.pkl")),
          "the study imports no training machinery and writes no actor")
    check("os.system" not in src and "os.sep" not in src and "from pathlib import Path" in src,
          "pathlib only, no shell, no os-specific path handling")

    print(json.dumps({"selftest": "PASS", "checks": CHECKS}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
