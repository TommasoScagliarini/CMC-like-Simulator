"""V26B B1R1 - READ-ONLY split study. It never trains, never writes an actor, never rolls out.

Purpose: design a deterministic grouped/blocked temporal validation for the base phase that

  * keeps all three pinned V26 trajectories (minus020, nominal, plus020) in the dataset and in
    the final fit;
  * guarantees that every FSM state present in a fold's VALIDATION is also present in its TRAIN;
  * never puts temporally adjacent rows of the same block on both sides of the split.

The study inventories the live FSM features (indices 17..24), builds the candidate split,
quantifies every fold, and compares the candidate against leave-one-trajectory-out and against
the July random 80/20.  It loads actor states only to read weights; it modifies nothing.

Cross-platform: pathlib only, no shell, no os-specific path handling.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import v26b_b0_masked35 as B0  # noqa: E402
import v26b_b1_base_fit as B1  # noqa: E402
import v26b_student as VS  # noqa: E402
import v26b_anchors as VA  # noqa: E402
import f0_common as C  # noqa: E402


class StudyError(RuntimeError):
    pass


STAGE = "V26B-B1R1-SPLIT-STUDY"
TRAJECTORY_NAMES = ("minus020", "nominal", "plus020")
FSM_ONEHOT = {"WAIT_HS": 17, "STANCE": 18, "SWING": 19}
FSM_EXPECTED = {"expected_hs": 20, "expected_to": 21}
FSM_CONTINUOUS = {"stance_elapsed_norm": 22, "swing_elapsed_norm": 23, "cycle_progress_credit": 24}
LIVE_DISCRETE_RANGE = (17, 25)

# --- candidate split parameters, preregistrable ---------------------------------------------------
BLOCK_MAX_ROWS = 40
N_FOLDS = 5
EMBARGO_ROWS = 20

# --- decorrelation measurement that fixes the embargo (measured, not conventional) ---------------
DECORRELATION_TABLE = {1: 6.5, 5: 20.9, 10: 35.8, 15: 49.8, 16: 52.7, 20: 67.5, 30: 89.7}
EMBARGO_RULE = ("the smallest multiple of five steps at which the MEDIAN distance between two rows "
                "of the same trajectory at that lag reaches at least half the median distance "
                "between decorrelated rows of that trajectory. Measured on the 25 live features: "
                "the 50% crossing is at lag 16, so lag 15 (49.8%) fails and lag 20 (67.5%) is the "
                "smallest admissible multiple of five")

# --- best-epoch rule, transferred from the July protocol to a K-fold design -----------------------
BEST_EPOCH_RULE = {
    "rule": "POOLED-FOLD EARLY STOPPING. At every epoch e compute the mean validation MSE over the "
            "K folds, weighted by validation rows. Select e* as the FIRST epoch attaining the "
            "minimum of that pooled curve, with July's patience applied to the pooled curve. The "
            "final fit then trains on all 1500 rows for exactly e* epochs with no validation",
    "tie_handling": "the FIRST (earliest) minimising epoch, which is what July's strict "
                    "'val_mse < best - 1e-9' comparison already selects among equal values",
    "epochs_max": 400, "patience": 60,
    "justification_july": "July's own rule is early stopping on validation MSE with patience 60, "
                          "max 400 epochs and best weights restored. 400 and 60 are the pinned "
                          "historical values; changing either would be a post-hoc choice",
    "justification_geometry": "the split assigns every state class round-robin, so the K validation "
                              "sets are exchangeable by construction: each holds one WAIT_HS block "
                              "and a comparable share of STANCE and SWING. Under exchangeability the "
                              "row-weighted pooled mean is the natural combination of the K curves "
                              "and its argmin is a single deterministic integer. A median of K "
                              "per-fold argmins has no July precedent, needs a tie rule for even K, "
                              "and discards the depth of each curve",
    "not_justified_by": "no B1 result is used to choose this rule or these numbers",
}

# --- candidate gates, none relaxed relative to B1 --------------------------------------------------
CANDIDATE_GATES = {
    "every_fold_improves_over_the_base_init": {"threshold": None, "unchanged_from_b1": True},
    "per_fold_heldout_aggregate_rmse": {"threshold": 0.05, "unchanged_from_b1": True},
    "pooled_row_weighted_validation_rmse": {"threshold": 0.03, "unchanged_from_b1": True},
    "final_all_data_aggregate_rmse": {"threshold": 0.02, "unchanged_from_b1": True},
    "final_per_joint_rmse": {"threshold": 0.03, "unchanged_from_b1": True},
    "final_per_joint_max_abs": {"threshold": 0.15, "unchanged_from_b1": True},
    "per_fsm_state_heldout_rmse": {"threshold": 0.05, "unchanged_from_b1": False,
                                   "status": "NEW and STRICTER: every FSM state present in a fold's "
                                             "validation must meet the same 0.05 bar, WAIT_HS "
                                             "included. B1 had no state-stratified gate at all"},
    "note": "no threshold is relaxed to accommodate the B1 NO-GO, which stands unmodified",
}

OUT_DIR = VA.OUT_ROOT / "diagnostics" / "b1r1_split_study"
RECEIPT_NAME = "v26b_b1r1_split_study.json"


# ================================================================= inventory ====================

def fsm_labels(obs: np.ndarray) -> np.ndarray:
    o = np.asarray(obs, dtype=float)
    lab = np.full(len(o), "UNKNOWN", dtype=object)
    for name, col in FSM_ONEHOT.items():
        lab[o[:, col] > 0.5] = name
    if np.any(lab == "UNKNOWN"):
        raise StudyError("a row carries no FSM one-hot: fail closed")
    return lab


def load_dataset() -> dict[str, Any]:
    d = B1.build_dataset()
    return {"obs_raw": np.asarray(d["observations_raw"], dtype=np.float32),
            "obs_masked": np.asarray(d["observations_masked"], dtype=np.float32),
            "actions": np.asarray(d["actions"], dtype=np.float32),
            "traj": np.asarray(d["trajectory_id"], dtype=np.int64),
            "report": d["report"]}


def inventory(obs: np.ndarray, traj: np.ndarray) -> dict[str, Any]:
    """Frequencies and transitions of the live FSM features, per trajectory and per segment."""
    names, _, _ = VS.pinned_names()
    out: dict[str, Any] = {"live_feature_indices": list(range(*LIVE_DISCRETE_RANGE)),
                           "live_feature_names": names[LIVE_DISCRETE_RANGE[0]:LIVE_DISCRETE_RANGE[1]],
                           "per_trajectory": {}}
    for tid, tname in enumerate(TRAJECTORY_NAMES):
        m = traj == tid
        s = np.asarray(obs[m], dtype=float)
        lab = fsm_labels(s)
        transitions = [{"at_row": int(k + 1), "from": str(lab[k - 1]), "to": str(lab[k])}
                       for k in range(1, len(lab)) if lab[k] != lab[k - 1]]
        wait = np.where(s[:, FSM_ONEHOT["WAIT_HS"]] > 0.5)[0]
        out["per_trajectory"][tname] = {
            "rows": int(m.sum()),
            "state_counts": {k: int((lab == k).sum()) for k in FSM_ONEHOT},
            "expected_counts": {k: int((s[:, c] > 0.5).sum()) for k, c in FSM_EXPECTED.items()},
            "continuous_ranges": {k: [float(s[:, c].min()), float(s[:, c].max())]
                                  for k, c in FSM_CONTINUOUS.items()},
            "first_state": str(lab[0]), "transitions": transitions,
            "wait_hs": ({"rows": int(wait.size), "first": int(wait[0] + 1), "last": int(wait[-1] + 1),
                         "contiguous": bool(np.all(np.diff(wait) == 1))}
                        if wait.size else {"rows": 0}),
        }
    total_wait = sum(v["state_counts"]["WAIT_HS"] for v in out["per_trajectory"].values())
    holders = [k for k, v in out["per_trajectory"].items() if v["state_counts"]["WAIT_HS"] > 0]
    out["wait_hs_summary"] = {
        "total_rows": total_wait, "trajectories_containing_it": holders,
        "finding": ("phase_fsm_wait_hs=1 exists in ONE trajectory only and only as a single "
                    "contiguous head segment. Any split that holds that whole trajectory out "
                    "leaves ZERO WAIT_HS rows in training, so the fold is an extrapolation, not "
                    "a generalisation test")}
    return out


def segments(obs: np.ndarray, traj: np.ndarray) -> list[dict[str, Any]]:
    """Maximal contiguous runs of constant FSM state, per trajectory."""
    segs: list[dict[str, Any]] = []
    for tid, tname in enumerate(TRAJECTORY_NAMES):
        idx = np.where(traj == tid)[0]
        lab = fsm_labels(obs[idx])
        start = 0
        for k in range(1, len(lab) + 1):
            if k == len(lab) or lab[k] != lab[k - 1]:
                segs.append({"trajectory": tid, "trajectory_name": tname, "state": str(lab[start]),
                             "local_lo": int(start), "local_hi": int(k - 1),
                             "rows": int(k - start),
                             "global_rows": [int(idx[start]), int(idx[k - 1])]})
                start = k
    return segs


def blocks(segs: Sequence[Mapping[str, Any]], block_max: int = BLOCK_MAX_ROWS) -> list[dict[str, Any]]:
    """Deterministic subdivision of every segment into near-equal contiguous blocks <= block_max."""
    if block_max < 1:
        raise StudyError("block_max must be >= 1")
    out: list[dict[str, Any]] = []
    for s in segs:
        n = int(s["rows"])
        nb = int(np.ceil(n / block_max))
        edges = np.linspace(0, n, nb + 1).astype(int)
        g0 = int(s["global_rows"][0])
        for b in range(nb):
            lo, hi = int(edges[b]), int(edges[b + 1]) - 1
            out.append({"state": s["state"], "trajectory": s["trajectory"],
                        "trajectory_name": s["trajectory_name"],
                        "segment_local_lo": s["local_lo"],
                        "global_lo": g0 + lo, "global_hi": g0 + hi, "rows": hi - lo + 1})
    return out


# ================================================================= candidate split ==============

def candidate_split(obs: np.ndarray, traj: np.ndarray, *, n_folds: int = N_FOLDS,
                    block_max: int = BLOCK_MAX_ROWS, embargo: int = EMBARGO_ROWS) -> dict[str, Any]:
    """Grouped blocked temporal split, deterministic and fail-closed.

    1. segment each trajectory into maximal runs of constant FSM state;
    2. subdivide each run into near-equal contiguous blocks of at most block_max rows;
    3. order the blocks by (state, trajectory, global_lo) - a total, reproducible order;
    4. assign fold = rank-within-its-state-class modulo n_folds, so every state class is spread
       round-robin over the folds;
    5. validation(f) = the rows of the blocks assigned to f;
    6. train(f) = every other row whose temporal distance from ANY validation row of the SAME
       trajectory exceeds the embargo;
    7. fail closed if a state present in validation(f) is absent from train(f).
    """
    if n_folds < 2:
        raise StudyError("n_folds must be >= 2")
    if embargo < 0:
        raise StudyError("embargo must be >= 0")
    segs = segments(obs, traj)
    blks = blocks(segs, block_max)
    ordered = sorted(range(len(blks)),
                     key=lambda i: (blks[i]["state"], blks[i]["trajectory"], blks[i]["global_lo"]))
    rank_in_state: dict[str, int] = {}
    for i in ordered:
        st = blks[i]["state"]
        r = rank_in_state.get(st, 0)
        blks[i]["fold"] = r % n_folds
        blks[i]["rank_in_state"] = r
        rank_in_state[st] = r + 1
    n = len(obs)
    lab = np.empty(n, dtype=object)
    for tid in range(len(TRAJECTORY_NAMES)):
        idx = np.where(traj == tid)[0]
        lab[idx] = fsm_labels(obs[idx])
    folds: list[dict[str, Any]] = []
    for f in range(n_folds):
        val = np.concatenate([np.arange(b["global_lo"], b["global_hi"] + 1)
                              for b in blks if b["fold"] == f]) if any(b["fold"] == f for b in blks) \
            else np.zeros(0, dtype=int)
        val = np.unique(val.astype(int))
        if val.size == 0:
            raise StudyError(f"fold {f} has an empty validation set")
        is_val = np.zeros(n, dtype=bool); is_val[val] = True
        embargoed = np.zeros(n, dtype=bool)
        for tid in range(len(TRAJECTORY_NAMES)):
            idx = np.where(traj == tid)[0]
            local_val = np.where(is_val[idx])[0]
            if local_val.size == 0:
                continue
            for p in local_val:
                lo = max(0, p - embargo); hi = min(len(idx) - 1, p + embargo)
                embargoed[idx[lo:hi + 1]] = True
        train = np.where(~is_val & ~embargoed)[0]
        if train.size == 0:
            raise StudyError(f"fold {f} has an empty training set")
        v_states = set(np.unique(lab[val]).tolist())
        t_states = set(np.unique(lab[train]).tolist())
        missing = sorted(v_states - t_states)
        if missing:
            raise StudyError(f"fold {f}: states {missing} appear in validation but not in training")
        folds.append({"fold": f, "validation_rows": int(val.size), "train_rows": int(train.size),
                      "dropped_to_embargo": int(n - val.size - train.size),
                      "validation_blocks": int(sum(1 for b in blks if b["fold"] == f)),
                      "validation_state_counts": {k: int((lab[val] == k).sum()) for k in FSM_ONEHOT},
                      "train_state_counts": {k: int((lab[train] == k).sum()) for k in FSM_ONEHOT},
                      "states_in_validation_missing_from_train": missing,
                      "_val": val, "_train": train})
    return {"mode": "grouped_blocked_temporal", "n_folds": n_folds, "block_max_rows": block_max,
            "embargo_rows": embargo, "n_blocks": len(blks), "n_segments": len(segs),
            "blocks_per_state": {k: int(sum(1 for b in blks if b["state"] == k)) for k in FSM_ONEHOT},
            "blocks": [{k: v for k, v in b.items()} for b in blks], "folds": folds}


def loto_split(traj: np.ndarray, obs: np.ndarray) -> dict[str, Any]:
    lab = np.empty(len(obs), dtype=object)
    for tid in range(len(TRAJECTORY_NAMES)):
        idx = np.where(traj == tid)[0]
        lab[idx] = fsm_labels(obs[idx])
    folds = []
    for tid, tname in enumerate(TRAJECTORY_NAMES):
        val = np.where(traj == tid)[0]; train = np.where(traj != tid)[0]
        v_states = set(np.unique(lab[val]).tolist()); t_states = set(np.unique(lab[train]).tolist())
        folds.append({"fold": tid, "holdout_trajectory": tname,
                      "validation_rows": int(val.size), "train_rows": int(train.size),
                      "dropped_to_embargo": 0,
                      "validation_state_counts": {k: int((lab[val] == k).sum()) for k in FSM_ONEHOT},
                      "train_state_counts": {k: int((lab[train] == k).sum()) for k in FSM_ONEHOT},
                      "states_in_validation_missing_from_train": sorted(v_states - t_states),
                      "_val": val, "_train": train})
    return {"mode": "leave_one_trajectory_out", "n_folds": len(folds), "folds": folds}


def random_split(traj: np.ndarray, obs: np.ndarray, *, fraction: float = 0.20,
                 seed: int = 123) -> dict[str, Any]:
    """The July seeded random permutation split, reproduced for comparison only."""
    n = len(obs)
    rng = np.random.default_rng(seed)
    perm = rng.permutation(n)
    n_val = max(1, int(round(n * fraction)))
    val = np.sort(perm[:n_val]); train = np.sort(perm[n_val:])
    lab = np.empty(n, dtype=object)
    for tid in range(len(TRAJECTORY_NAMES)):
        idx = np.where(traj == tid)[0]
        lab[idx] = fsm_labels(obs[idx])
    v_states = set(np.unique(lab[val]).tolist()); t_states = set(np.unique(lab[train]).tolist())
    return {"mode": "july_random_80_20", "n_folds": 1, "seed": seed, "fraction": fraction,
            "folds": [{"fold": 0, "validation_rows": int(val.size), "train_rows": int(train.size),
                       "dropped_to_embargo": 0,
                       "validation_state_counts": {k: int((lab[val] == k).sum()) for k in FSM_ONEHOT},
                       "train_state_counts": {k: int((lab[train] == k).sum()) for k in FSM_ONEHOT},
                       "states_in_validation_missing_from_train": sorted(v_states - t_states),
                       "_val": val, "_train": train}]}


# ================================================================= quantification ===============

def temporal_leakage(val: np.ndarray, train: np.ndarray, traj: np.ndarray) -> dict[str, Any]:
    """Minimum temporal distance between a validation row and a training row of the SAME
    trajectory, and how many validation rows have an immediate temporal neighbour in training."""
    is_train = np.zeros(len(traj), dtype=bool); is_train[train] = True
    min_dist = None
    adjacent = 0
    for tid in range(len(TRAJECTORY_NAMES)):
        idx = np.where(traj == tid)[0]
        pos = {int(g): i for i, g in enumerate(idx)}
        lv = np.array([pos[int(g)] for g in val if int(g) in pos], dtype=int)
        lt = np.array([pos[int(g)] for g in train if int(g) in pos], dtype=int)
        if lv.size == 0 or lt.size == 0:
            continue
        lt_sorted = np.sort(lt)
        ins = np.searchsorted(lt_sorted, lv)
        for k, p in enumerate(lv):
            cands = []
            if ins[k] < lt_sorted.size:
                cands.append(abs(int(lt_sorted[ins[k]]) - int(p)))
            if ins[k] > 0:
                cands.append(abs(int(lt_sorted[ins[k] - 1]) - int(p)))
            if cands:
                dmin = min(cands)
                min_dist = dmin if min_dist is None else min(min_dist, dmin)
                if dmin <= 1:
                    adjacent += 1
    return {"min_temporal_distance_same_trajectory": (None if min_dist is None else int(min_dist)),
            "validation_rows_with_an_immediate_training_neighbour": int(adjacent),
            "fraction": float(adjacent) / float(max(1, len(val))),
            "leakage": bool(min_dist is not None and min_dist <= 1)}


def continuous_support(obs_masked: np.ndarray, val: np.ndarray, train: np.ndarray) -> dict[str, Any]:
    """Same instrument used on B1: nearest-neighbour distance in the 25 live features."""
    from scipy.spatial import cKDTree
    keep = list(B0.kept_columns())
    Z = np.asarray(obs_masked, dtype=np.float64)[:, keep]
    sd = Z.std(0); live = np.where(sd > 1e-12)[0]
    Zn = (Z[:, live] - Z[:, live].mean(0)) / sd[live]
    tr = Zn[train]; va = Zn[val]
    tree = cKDTree(tr)
    d, _ = tree.query(va, k=1)
    inner, _ = tree.query(tr, k=2)
    p99 = float(np.quantile(inner[:, 1], 0.99))
    return {"live_dimensions": int(len(live)),
            "nn_distance_median": float(np.median(d)), "nn_distance_p90": float(np.quantile(d, 0.9)),
            "nn_distance_max": float(d.max()),
            "train_internal_p99": p99,
            "fraction_beyond_train_internal_p99": float((d > p99).mean())}


def nn_label_baseline(obs_masked: np.ndarray, actions: np.ndarray, val: np.ndarray,
                      train: np.ndarray) -> dict[str, Any]:
    """Model-free baseline: predict each validation label with its nearest training neighbour's
    label, in the same 25-feature metric. It measures how learnable a fold is, without any fit."""
    from scipy.spatial import cKDTree
    keep = list(B0.kept_columns())
    Z = np.asarray(obs_masked, dtype=np.float64)[:, keep]
    sd = Z.std(0); live = np.where(sd > 1e-12)[0]
    Zn = (Z[:, live] - Z[:, live].mean(0)) / sd[live]
    _, j = cKDTree(Zn[train]).query(Zn[val], k=1)
    pred = np.asarray(actions, dtype=np.float64)[train][j]
    truth = np.asarray(actions, dtype=np.float64)[val]
    dd = pred - truth
    return {"aggregate_rmse": float(np.sqrt(np.mean(dd ** 2))),
            "per_joint_rmse": {jn: float(np.sqrt(np.mean(dd[:, k] ** 2)))
                               for k, jn in ((0, "knee"), (1, "ankle"))},
            "per_joint_max_abs": {jn: float(np.max(np.abs(dd[:, k])))
                                  for k, jn in ((0, "knee"), (1, "ankle"))},
            "note": "no model is fitted; this is a pure nearest-neighbour transfer of labels"}


def quantify(split: Mapping[str, Any], data: Mapping[str, Any]) -> dict[str, Any]:
    out = []
    for f in split["folds"]:
        val, train = f["_val"], f["_train"]
        out.append({**{k: v for k, v in f.items() if not k.startswith("_")},
                    "temporal": temporal_leakage(val, train, data["traj"]),
                    "continuous_support": continuous_support(data["obs_masked"], val, train),
                    "nn_label_baseline": nn_label_baseline(data["obs_masked"], data["actions"],
                                                           val, train)})
    agg = {"worst_nn_label_rmse": max(x["nn_label_baseline"]["aggregate_rmse"] for x in out),
           "mean_nn_label_rmse": float(np.mean([x["nn_label_baseline"]["aggregate_rmse"] for x in out])),
           "worst_fraction_beyond_p99": max(x["continuous_support"]["fraction_beyond_train_internal_p99"]
                                            for x in out),
           "any_state_missing_from_train": any(x["states_in_validation_missing_from_train"] for x in out),
           "any_temporal_leakage": any(x["temporal"]["leakage"] for x in out),
           "min_temporal_distance_over_folds": min(
               (x["temporal"]["min_temporal_distance_same_trajectory"] for x in out
                if x["temporal"]["min_temporal_distance_same_trajectory"] is not None), default=None)}
    return {"mode": split["mode"], "n_folds": split["n_folds"], "per_fold": out, "aggregate": agg}


def run_study(*, write: bool = True) -> dict[str, Any]:
    data = load_dataset()
    inv = inventory(data["obs_raw"], data["traj"])
    segs = segments(data["obs_raw"], data["traj"])
    cand = candidate_split(data["obs_raw"], data["traj"])
    comparison = {"candidate_grouped_blocked": quantify(cand, data),
                  "leave_one_trajectory_out": quantify(loto_split(data["traj"], data["obs_raw"]), data),
                  "july_random_80_20": quantify(random_split(data["traj"], data["obs_raw"]), data)}
    receipt = {"schema": "v26b_b1r1_split_study.1", "stage": STAGE,
               "kind": "READ-ONLY study. No training, no actor modification, no rollout",
               "dataset": data["report"],
               "fsm_inventory": inv,
               "segments": segs,
               "candidate_split_parameters": {"block_max_rows": BLOCK_MAX_ROWS,
                                              "n_folds": N_FOLDS, "embargo_rows": EMBARGO_ROWS,
                                              "embargo_rule": EMBARGO_RULE,
                                              "decorrelation_percent_by_lag": DECORRELATION_TABLE,
                                              "block_max_rule": "40 rows makes the single WAIT_HS "
                                                  "segment split into exactly 5 blocks, one per "
                                                  "fold, so every fold tests that state"},
               "best_epoch_rule": BEST_EPOCH_RULE,
               "candidate_gates": CANDIDATE_GATES,
               "candidate_split_structure": {k: v for k, v in cand.items() if k != "folds"},
               "comparison": comparison,
               "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}
    if write:
        OUT_DIR.mkdir(parents=True, exist_ok=True)
        C.write_json(OUT_DIR / RECEIPT_NAME, receipt, clobber=False)
    return receipt


def main(argv: Sequence[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="V26B B1R1 read-only split study")
    p.add_argument("--run", action="store_true")
    p.add_argument("--no-write", action="store_true")
    a = p.parse_args(argv)
    if not a.run:
        load_dataset(); print(json.dumps({"mode": "dry", "ok": True}, indent=2)); return 0
    r = run_study(write=not a.no_write)
    comp = r["comparison"]
    print(json.dumps({k: {"n_folds": v["n_folds"], **v["aggregate"]} for k, v in comp.items()},
                     indent=2, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
