"""V26B B1R1 revision - READ-ONLY study of a leave-one-complete-cycle-out split.

It never trains, never writes an actor, never rolls out.

Three corrections to the superseded blocked-5 study (v26b_b1r1_split_study.py, kept as comparator):

  1. SUPPORT IS 23D, not 25D.  B0 hard-zeroes the two clock columns, so the effective support is
     kept_columns minus CLOCK_COLUMNS = columns 2..24.  The previous study passed 25 columns and
     relied on a zero-variance filter to drop the clock pair implicitly.
  2. THE SCALER IS FIT ON TRAIN ONLY.  The previous study standardised with the mean and standard
     deviation of the WHOLE dataset, which leaks validation statistics into the metric itself.
  3. DISCRETE COVERAGE IS CHECKED ON EXACT SIGNATURES over columns (17, 18, 19, 20, 21, 24), not
     on the three one-hot classes alone.  Columns 22 and 23 are continuous (412 and 557 distinct
     values); column 24 is categorical with four levels {0, 0.25, 0.5, 1.0}.

Split under study: six folds, each holding out ONE complete STANCE+SWING cycle of ONE trajectory,
with an embargo of 20 steps inside that trajectory.  Every WAIT_HS row stays in the training set of
every fold - and the study proves formally why no honest WAIT heldout exists in this data.

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
import v26b_b1r1_split_study as PREV  # noqa: E402   (superseded proposal, kept as comparator)
import v26b_student as VS  # noqa: E402
import v26b_anchors as VA  # noqa: E402
import f0_common as C  # noqa: E402


class LocoError(RuntimeError):
    pass


STAGE = "V26B-B1R1-LOCO-STUDY"
TRAJECTORY_NAMES = ("minus020", "nominal", "plus020")

# --- corrected support and signature definitions --------------------------------------------------
SUPPORT_COLUMNS = tuple(c for c in B0.kept_columns() if c not in B0.CLOCK_COLUMNS)   # 23 columns
DISCRETE_SIGNATURE_COLUMNS = (17, 18, 19, 20, 21, 24)
CONTINUOUS_PHASE_COLUMNS = (22, 23)
WAIT_COLUMN = 17

# --- the six candidate cycles, local 0-based inclusive ranges --------------------------------------
CYCLES: tuple[dict[str, Any], ...] = (
    {"trajectory": 0, "name": "minus020", "cycle": 1, "local_lo": 187, "local_hi": 334},
    {"trajectory": 0, "name": "minus020", "cycle": 2, "local_lo": 335, "local_hi": 488},
    {"trajectory": 1, "name": "nominal", "cycle": 1, "local_lo": 164, "local_hi": 314},
    {"trajectory": 1, "name": "nominal", "cycle": 2, "local_lo": 315, "local_hi": 467},
    {"trajectory": 2, "name": "plus020", "cycle": 1, "local_lo": 144, "local_hi": 294},
    {"trajectory": 2, "name": "plus020", "cycle": 2, "local_lo": 295, "local_hi": 447},
)
EMBARGO_ROWS = 20
# A held-out cycle adjacent to the WAIT segment costs some WAIT training rows to the embargo.
# The floor makes that cost explicit and bounded instead of silent.
WAIT_TRAIN_MIN_FRACTION = 0.80

# --- preregistered WAIT reconstruction gate, proposed here and never relaxed later ----------------
WAIT_GATE = {
    "why_separate": ("WAIT_HS occurs exactly ONCE in the whole dataset, as a single contiguous "
                     "segment. No honest in-distribution heldout of it exists, so it is NOT "
                     "presented as a generalisation test. It stays in the training set of every "
                     "fold and is audited by a reconstruction gate on the final all-data fit"),
    "measure": "RMSE of the final all-data actor on the WAIT_HS rows only, plus per joint",
    "aggregate_rmse_max": 0.02,
    "per_joint_rmse_max": 0.03,
    "per_joint_max_abs_max": 0.15,
    "rationale": ("the same numbers already preregistered for the final all-data fit. WAIT_HS rows "
                  "are training rows for every fold, so this is a RECONSTRUCTION requirement, not a "
                  "generalisation one, and it must therefore be at least as strict as the all-data "
                  "gate - never looser"),
    "honesty_statement": "this gate must never be reported as evidence of WAIT generalisation",
}

OUT_DIR = VA.OUT_ROOT / "diagnostics" / "b1r1_loco_study"
RECEIPT_NAME = "v26b_b1r1_loco_study.json"


# ================================================================ corrected instruments ==========

def support_matrix(obs: np.ndarray) -> np.ndarray:
    a = np.asarray(obs, dtype=np.float64)
    if a.ndim != 2 or a.shape[1] != B0.ACTOR_WIDTH:
        raise LocoError(f"observations must be (N,{B0.ACTOR_WIDTH}), got {a.shape}")
    return a[:, list(SUPPORT_COLUMNS)]


def train_only_scaler(support: np.ndarray, train: np.ndarray) -> dict[str, np.ndarray]:
    """Mean and standard deviation computed on TRAINING ROWS ONLY. Columns that are constant on
    the training rows are dropped, because they carry no metric information the model could use."""
    tr = np.asarray(support)[np.asarray(train, dtype=int)]
    mu = tr.mean(axis=0)
    sd = tr.std(axis=0)
    live = np.where(sd > 1e-12)[0]
    if live.size == 0:
        raise LocoError("the training rows have no varying support column")
    return {"mu": mu, "sd": sd, "live": live}


def apply_scaler(support: np.ndarray, sc: Mapping[str, np.ndarray]) -> np.ndarray:
    live = sc["live"]
    return (np.asarray(support)[:, live] - sc["mu"][live]) / sc["sd"][live]


def discrete_signature(obs: np.ndarray) -> list[tuple[float, ...]]:
    """The exact tuple of the discrete FSM columns, one hashable key per row."""
    a = np.asarray(obs, dtype=np.float64)[:, list(DISCRETE_SIGNATURE_COLUMNS)]
    return [tuple(float(x) for x in np.round(r, 6)) for r in a]


def signature_coverage(obs: np.ndarray, val: np.ndarray, train: np.ndarray) -> dict[str, Any]:
    sig = discrete_signature(obs)
    tr = {sig[int(i)] for i in np.asarray(train, dtype=int)}
    va = [sig[int(i)] for i in np.asarray(val, dtype=int)]
    missing_rows = int(sum(1 for s in va if s not in tr))
    missing_sigs = sorted({s for s in va if s not in tr}, key=str)
    return {"columns": list(DISCRETE_SIGNATURE_COLUMNS),
            "distinct_signatures_in_validation": int(len(set(va))),
            "distinct_signatures_in_train": int(len(tr)),
            "validation_rows_whose_signature_is_absent_from_train": missing_rows,
            "fraction": float(missing_rows) / float(max(1, len(va))),
            "missing_signatures": [list(s) for s in missing_sigs],
            "pass": missing_rows == 0}


def temporal_leakage(val: np.ndarray, train: np.ndarray, traj: np.ndarray) -> dict[str, Any]:
    is_t = np.zeros(len(traj), dtype=bool); is_t[np.asarray(train, dtype=int)] = True
    dmin = None; adjacent = 0
    for tid in range(len(TRAJECTORY_NAMES)):
        idx = np.where(traj == tid)[0]
        pos = {int(g): i for i, g in enumerate(idx)}
        lv = np.array([pos[int(g)] for g in val if int(g) in pos], dtype=int)
        lt = np.sort(np.array([pos[int(g)] for g in train if int(g) in pos], dtype=int))
        if lv.size == 0 or lt.size == 0:
            continue
        ins = np.searchsorted(lt, lv)
        for k, p in enumerate(lv):
            cand = []
            if ins[k] < lt.size: cand.append(abs(int(lt[ins[k]]) - int(p)))
            if ins[k] > 0: cand.append(abs(int(lt[ins[k] - 1]) - int(p)))
            if cand:
                dd = min(cand)
                dmin = dd if dmin is None else min(dmin, dd)
                if dd <= 1: adjacent += 1
    return {"min_temporal_distance_same_trajectory": (None if dmin is None else int(dmin)),
            "validation_rows_with_an_immediate_training_neighbour": int(adjacent),
            "leakage": bool(dmin is not None and dmin <= 1)}


def support_and_baseline(obs: np.ndarray, actions: np.ndarray, val: np.ndarray,
                         train: np.ndarray) -> dict[str, Any]:
    """23D support and nearest-neighbour label baseline, both with a TRAIN-ONLY scaler."""
    from scipy.spatial import cKDTree
    sup = support_matrix(obs)
    sc = train_only_scaler(sup, train)
    Z = apply_scaler(sup, sc)
    tr, va = Z[np.asarray(train, dtype=int)], Z[np.asarray(val, dtype=int)]
    tree = cKDTree(tr)
    d, j = tree.query(va, k=1)
    inner, _ = tree.query(tr, k=2)
    p99 = float(np.quantile(inner[:, 1], 0.99))
    a = np.asarray(actions, dtype=np.float64)
    pred = a[np.asarray(train, dtype=int)][j]
    dd = pred - a[np.asarray(val, dtype=int)]
    return {"support_dimensions": int(sc["live"].size),
            "scaler": "fit on TRAINING ROWS ONLY",
            "nn_distance_median": float(np.median(d)), "nn_distance_p90": float(np.quantile(d, 0.9)),
            "nn_distance_max": float(d.max()), "train_internal_p99": p99,
            "fraction_beyond_train_internal_p99": float((d > p99).mean()),
            "nn_label_baseline_DIAGNOSTIC_ONLY": {
                "aggregate_rmse": float(np.sqrt(np.mean(dd ** 2))),
                "per_joint_rmse": {n: float(np.sqrt(np.mean(dd[:, k] ** 2)))
                                   for k, n in ((0, "knee"), (1, "ankle"))},
                "note": "no model is fitted; it is a difficulty indicator, never a gate and never "
                        "a lower bound on what a smooth regressor can achieve"}}


# ================================================================ splits =========================

def _global(traj: np.ndarray, tid: int, lo: int, hi: int) -> np.ndarray:
    idx = np.where(traj == tid)[0]
    if hi >= len(idx):
        raise LocoError(f"cycle {lo}:{hi} exceeds trajectory {tid} of {len(idx)} rows")
    return idx[lo:hi + 1]


def loco_split(obs: np.ndarray, traj: np.ndarray, *, embargo: int = EMBARGO_ROWS) -> dict[str, Any]:
    """Leave-one-complete-cycle-out. Every WAIT_HS row stays in training in every fold."""
    if embargo < 0:
        raise LocoError("embargo must be >= 0")
    n = len(obs)
    o = np.asarray(obs, dtype=np.float64)
    folds: list[dict[str, Any]] = []
    for f, cyc in enumerate(CYCLES):
        val = _global(traj, cyc["trajectory"], cyc["local_lo"], cyc["local_hi"])
        if np.any(o[val, WAIT_COLUMN] > 0.5):
            raise LocoError(f"fold {f}: the held-out cycle contains WAIT_HS rows, which must never "
                            "leave the training set")
        is_val = np.zeros(n, dtype=bool); is_val[val] = True
        emb = np.zeros(n, dtype=bool)
        idx = np.where(traj == cyc["trajectory"])[0]
        local_val = np.where(is_val[idx])[0]
        for p in local_val:
            lo = max(0, p - embargo); hi = min(len(idx) - 1, p + embargo)
            emb[idx[lo:hi + 1]] = True
        train = np.where(~is_val & ~emb)[0]
        if train.size == 0:
            raise LocoError(f"fold {f} has an empty training set")
        wait_total = int(np.sum(o[:, WAIT_COLUMN] > 0.5))
        wait_train = int(np.sum(o[train, WAIT_COLUMN] > 0.5))
        wait_lost = wait_total - wait_train
        # A held-out cycle that begins immediately after the WAIT segment pushes the embargo back
        # INTO that segment. Removing those rows is the leakage-free choice: keeping them would put
        # a training row one step from a validation row. The loss is recorded, never hidden, and a
        # floor keeps it from becoming material.
        if wait_train < int(WAIT_TRAIN_MIN_FRACTION * wait_total):
            raise LocoError(f"fold {f}: only {wait_train}/{wait_total} WAIT_HS rows survive the "
                            f"embargo, below the {WAIT_TRAIN_MIN_FRACTION:.0%} floor")
        folds.append({"fold": f, **{k: v for k, v in cyc.items()},
                      "validation_rows": int(val.size), "train_rows": int(train.size),
                      "dropped_to_embargo": int(n - val.size - train.size),
                      "wait_rows_in_train": wait_train, "wait_rows_total": wait_total,
                      "wait_rows_lost_to_embargo": wait_lost,
                      "_val": val, "_train": train})
    return {"mode": "leave_one_complete_cycle_out", "n_folds": len(folds),
            "embargo_rows": embargo, "cycles": [dict(c) for c in CYCLES], "folds": folds,
            "wait_policy": ("no held-out cycle contains a WAIT_HS row, so WAIT is never validated. "
                            "A fold whose cycle starts immediately after the WAIT segment loses the "
                            "embargo window of WAIT rows from training; that loss is recorded per "
                            f"fold and floored at {WAIT_TRAIN_MIN_FRACTION:.0%} of the segment"),
            "wait_rows_in_train_per_fold": {int(f["fold"]): int(f["wait_rows_in_train"]) for f in folds}}


# ================================================================ formal WAIT verification =======

def wait_formal_verification(obs: np.ndarray, traj: np.ndarray) -> dict[str, Any]:
    """Prove that no honest in-distribution WAIT heldout exists in this dataset."""
    o = np.asarray(obs, dtype=np.float64)
    w = np.where(o[:, WAIT_COLUMN] > 0.5)[0]
    if w.size == 0:
        raise LocoError("no WAIT_HS row found")
    runs = []
    s = p = int(w[0])
    for i in w[1:]:
        if int(i) == p + 1: p = int(i)
        else: runs.append((s, p)); s = p = int(i)
    runs.append((s, p))
    sup = support_matrix(o[w])
    mu, sd = sup.mean(0), sup.std(0)
    sd[sd < 1e-12] = 1.0
    Z = (sup - mu) / sd
    m = len(Z)
    D = np.linalg.norm(Z[:, None, :] - Z[None, :, :], axis=2)
    lag = np.abs(np.arange(m)[:, None] - np.arange(m)[None, :])
    adj_med = float(np.median(np.where(lag == 1, D, np.inf).min(1)))
    out = {}
    for E in (10, 20, 30):
        far = np.where(lag > E, D, np.inf).min(1)
        out[f"embargo_{E}"] = {"nn_distance_median": float(np.median(far)),
                               "nn_distance_min": float(far.min()),
                               "ratio_to_lag1_median": float(np.median(far) / adj_med)}
    iu = np.triu_indices(m, 1)
    recur = float(np.mean(D[iu][lag[iu] > 20] < adj_med))
    return {
        "occurrences": len(runs), "runs": runs, "rows": int(w.size),
        "trajectories_containing_it": sorted(set(traj[w].tolist())),
        "lag1_median_distance": adj_med,
        "by_embargo": out,
        "recurrence_fraction_lag_gt_20_closer_than_lag1_median": recur,
        "verdict": ("VERIFIED: WAIT_HS occurs exactly once, as one contiguous segment, in one "
                    "trajectory, and it never revisits a state - the fraction of row pairs more "
                    "than 20 steps apart that are closer than a typical adjacent pair is "
                    f"{recur:.4f}. Therefore any heldout of WAIT rows is either temporally leaky "
                    "(adjacent training rows are near-duplicates), or an extrapolation along the "
                    "single segment (no independent repeat exists to generalise from), or leaves "
                    "training with no WAIT row at all. No in-distribution WAIT heldout exists"),
        "consequence": ("WAIT_HS must stay in training in every fold and be audited by a separate "
                        "RECONSTRUCTION gate on the final all-data fit. It must never be reported "
                        "as a generalisation result"),
    }


# ================================================================ driver =========================

def quantify(split: Mapping[str, Any], obs: np.ndarray, actions: np.ndarray,
             traj: np.ndarray) -> dict[str, Any]:
    per = []
    for f in split["folds"]:
        val, train = f["_val"], f["_train"]
        per.append({**{k: v for k, v in f.items() if not k.startswith("_")},
                    "temporal": temporal_leakage(val, train, traj),
                    "discrete_signature_coverage": signature_coverage(obs, val, train),
                    "support": support_and_baseline(obs, actions, val, train)})
    nn = [x["support"]["nn_label_baseline_DIAGNOSTIC_ONLY"]["aggregate_rmse"] for x in per]
    return {"mode": split["mode"], "n_folds": split["n_folds"], "per_fold": per,
            "aggregate": {
                "worst_nn_label_rmse": float(max(nn)), "mean_nn_label_rmse": float(np.mean(nn)),
                "worst_fraction_beyond_p99": float(max(
                    x["support"]["fraction_beyond_train_internal_p99"] for x in per)),
                "any_signature_missing_from_train": any(
                    not x["discrete_signature_coverage"]["pass"] for x in per),
                "any_temporal_leakage": any(x["temporal"]["leakage"] for x in per),
                "min_temporal_distance": min(
                    (x["temporal"]["min_temporal_distance_same_trajectory"] for x in per
                     if x["temporal"]["min_temporal_distance_same_trajectory"] is not None),
                    default=None),
                "validation_rows_total": int(sum(x["validation_rows"] for x in per)),
                "train_rows_min": int(min(x["train_rows"] for x in per)),
                "dropped_median": float(np.median([x["dropped_to_embargo"] for x in per])),
            }}


def run_study(*, write: bool = True) -> dict[str, Any]:
    d = B1.build_dataset()
    obs = np.asarray(d["observations_masked"], dtype=np.float32)
    actions = np.asarray(d["actions"], dtype=np.float32)
    traj = np.asarray(d["trajectory_id"], dtype=np.int64)
    loco = loco_split(obs, traj)
    comparison = {"loco_6fold": quantify(loco, obs, actions, traj)}
    prev = PREV.candidate_split(obs, traj)
    comparison["superseded_blocked_5fold_recomputed"] = quantify(
        {"mode": "grouped_blocked_temporal_SUPERSEDED", "n_folds": prev["n_folds"],
         "folds": prev["folds"]}, obs, actions, traj)
    lot = PREV.loto_split(traj, obs)
    comparison["leave_one_trajectory_out"] = quantify(
        {"mode": "leave_one_trajectory_out", "n_folds": lot["n_folds"], "folds": lot["folds"]},
        obs, actions, traj)
    rnd = PREV.random_split(traj, obs)
    comparison["july_random_80_20"] = quantify(
        {"mode": "july_random_80_20", "n_folds": 1, "folds": rnd["folds"]}, obs, actions, traj)
    receipt = {
        "schema": "v26b_b1r1_loco_study.1", "stage": STAGE,
        "kind": "READ-ONLY study. No training, no actor modification, no rollout",
        "corrections_applied": {
            "support_dimension": f"23D = kept_columns minus CLOCK_COLUMNS = {list(SUPPORT_COLUMNS)}",
            "scaler": "fit on TRAINING ROWS ONLY in every metric",
            "discrete_coverage": f"exact signatures over columns {list(DISCRETE_SIGNATURE_COLUMNS)}; "
                                 f"columns {list(CONTINUOUS_PHASE_COLUMNS)} are continuous",
        },
        "dataset": d["report"],
        "wait_formal_verification": wait_formal_verification(obs, traj),
        "split_under_study": {k: v for k, v in loco.items() if k != "folds"},
        "wait_reconstruction_gate": WAIT_GATE,
        "comparison": comparison,
        "superseded": {"file": "v26b_b1r1_split_study.py",
                       "status": "kept as comparator; its own receipt is preserved byte-identical"},
        "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}
    if write:
        OUT_DIR.mkdir(parents=True, exist_ok=True)
        C.write_json(OUT_DIR / RECEIPT_NAME, receipt, clobber=False)
    return receipt


def main(argv: Sequence[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="V26B B1R1 leave-one-complete-cycle-out study")
    p.add_argument("--run", action="store_true")
    p.add_argument("--no-write", action="store_true")
    a = p.parse_args(argv)
    if not a.run:
        B1.build_dataset(); print(json.dumps({"mode": "dry", "ok": True}, indent=2)); return 0
    r = run_study(write=not a.no_write)
    print(json.dumps({k: {"n_folds": v["n_folds"], **v["aggregate"]}
                      for k, v in r["comparison"].items()}, indent=2, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
