"""V26B rev3n — S1 pre-gate instrument symmetrization (READ-ONLY).

Token V26B-S1-REV3N-PREREG-MEDIAN (NOT V26B-S1-FIT). Same feasibility instrument as
rev3l (kNN k=5, labels u_IK AB06, train-only standardization, threshold 0.15/joint),
applied to the 5 symmetric blocked temporal folds prescribed by amendment rev3n with
embargo 10; PRIMARY metric = MEDIAN of the 5 per-fold RMSE, per joint.

Additive by construction: `v26b_s1_prereg.py` and `test_v26b_s1_prereg.py` are NOT
modified, so the digests recorded inside the original FAIL receipt a437f1ef... stay
byte-reproducible (verified fail-closed here), and fold 3 ([201,300] = the original
hold window) must reproduce that receipt's binding pair bit-exact.

No fit, no export, no rollout.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import v26b_s1_prereg as S1  # noqa: E402  (rev3l tooling, unmodified)
import v26b_anchors as VA  # noqa: E402
import f0_common as C  # noqa: E402
import f2r_common as R  # noqa: E402


class Rev3nError(RuntimeError):
    pass


AMENDMENT_REV3N = HERE / "v26b_amendment_rev3n_s1_pregate_symmetric_5fold.json"
PIN_AMENDMENT_REV3N = "0677ad577336240b26cf6947e9e3d60d74632a35724f49b5f06bf4bc5aa377e3"

FOLDS: tuple[tuple[int, int], ...] = ((1, 100), (101, 200), (201, 300), (301, 400), (401, 500))
EMBARGO = S1.EMBARGO                 # 10, reused (never redefined)
RMSE_MAX = S1.PRE_GATE_RMSE_MAX      # 0.15, reused (never relaxed)
K_NEIGHBOURS = 5
FUTURE_FIT_TOKEN = S1.FUTURE_FIT_TOKEN

ORIGINAL_RECEIPT = VA.OUT_ROOT / "v26b_s1_pregate_receipt_20260824_173741.json"
PIN_ORIGINAL_RECEIPT = "a437f1ef0d7762db5914d14be19b390ddc6ada9a126d9db2ef8f810d08a43389"
FOLD3_BINDING = [0.15220686466232133, 0.10626628039651334]

POSTFIT_RULES = {
    "G_task_blocked_holdout_steps": [201, 300],
    "G_task_blocked_holdout_max": 0.15,
    "G_task_blocked_holdout_must_be_out_of_sample": True,
    "G_task_excluded_from_fit_training_rows": [[191, 200], [201, 300], [301, 310]],
    "full_500_metrics": "diagnostic only",
    "Q1_source_holdout_rev3m_max": 0.10,
    "Q1_source_holdout_rev3m_binding": True,
    "parameter_anchor_july": 1e-5,
    "drift_vs_S0D_on_task_rows": "diagnostic only",
}


def fold_masks(lo: int, hi: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    step = np.arange(1, 501)
    hold = (step >= lo) & (step <= hi)
    embargo = ((step >= lo - EMBARGO) & (step < lo)) | ((step > hi) & (step <= hi + EMBARGO))
    return ~(hold | embargo), hold, embargo


def knn_rmse(obs: np.ndarray, train: np.ndarray, hold: np.ndarray, y: np.ndarray,
             k: int = K_NEIGHBOURS) -> tuple[list[float], list[int]]:
    """Bit-identical code path to rev3l `pre_gate` (batched, argpartition k-1)."""
    mu = obs[train].mean(0); sd = obs[train].std(0); keep = sd > 1e-9
    excluded = [int(i) for i in np.where(~keep)[0]]
    z = ((obs - mu) / np.where(keep, sd, 1.0))[:, keep]
    zt = z[train]; yt = y[train]; b2 = np.sum(zt ** 2, 1)
    zh = z[hold]; preds = np.empty((int(hold.sum()), y.shape[1]))
    for s in range(0, zh.shape[0], 256):
        a = zh[s: s + 256]
        d2 = np.sum(a ** 2, 1)[:, None] + b2[None, :] - 2.0 * a @ zt.T
        idx = np.argpartition(d2, k - 1, 1)[:, :k]
        preds[s: s + 256] = yt[idx].mean(1)
    return np.sqrt(np.mean((preds - y[hold]) ** 2, axis=0)).tolist(), excluded


def pass_rule(fold_rmse: Sequence[Sequence[float]]) -> dict[str, Any]:
    """PRIMARY gate rev3n: median of the folds <= RMSE_MAX for BOTH joints."""
    a = np.asarray(fold_rmse, dtype=np.float64)
    if a.ndim != 2 or a.shape[0] != len(FOLDS):
        raise Rev3nError(f"expected {len(FOLDS)} folds, got shape {a.shape}")
    med = np.median(a, axis=0).tolist()
    return {"primary_metric": "median of the 5 blocked folds, per joint",
            "median_per_joint": med, "max_per_joint": a.max(0).tolist(), "mean_per_joint": a.mean(0).tolist(),
            "n_folds_le_threshold": [int((a[:, j] <= RMSE_MAX).sum()) for j in range(a.shape[1])],
            "threshold": RMSE_MAX, "pass": bool(all(v <= RMSE_MAX for v in med))}


def verify_lineage_rev3n() -> dict[str, Any]:
    lin = S1.verify_lineage_s1()   # rev3l + rev3m + S0D chain + AB06 cache (fail-closed)
    got = C.sha256_file(AMENDMENT_REV3N)
    if got != PIN_AMENDMENT_REV3N:
        raise Rev3nError(f"rev3n sha {got} != pinned")
    lin["amendment_rev3n"] = got
    got = C.sha256_file(ORIGINAL_RECEIPT)
    if got != PIN_ORIGINAL_RECEIPT:
        raise Rev3nError(f"original FAIL receipt sha {got} != pinned (evidence must stay immutable)")
    lin["original_pregate_receipt"] = got
    recorded = json.loads(ORIGINAL_RECEIPT.read_text())["code_digests"]
    proof = {}
    for name, want in recorded.items():
        now = C.sha256_file(HERE / name)
        if now != want:
            raise Rev3nError(f"{name} changed ({now} != {want} recorded in the FAIL receipt): rev3n must stay additive")
        proof[name] = now
    lin["original_tooling_byte_reproducible"] = proof
    return lin


def run_fit(*, authorized_stage: str | None = None, **_):
    raise Rev3nError(f"the S1 fit requires --authorized-stage {FUTURE_FIT_TOKEN}, NOT granted "
                     f"(rev3n authorises the symmetrized pre-gate only); got {authorized_stage!r}")


def pre_gate_rev3n(view: Mapping[str, Any]) -> dict[str, Any]:
    obs = view["obs"].astype(np.float64)
    u_ik, u_own = view["u_ik"], view["u_own"]
    folds: list[dict[str, Any]] = []
    for lo, hi in FOLDS:
        train, hold, embargo = fold_masks(lo, hi)
        if np.any(train & hold) or np.any(train & embargo) or np.any(hold & embargo):
            raise Rev3nError(f"fold [{lo},{hi}] masks overlap")
        rmse, excluded = knn_rmse(obs, train, hold, u_ik)
        rmse_own, _ = knn_rmse(obs, train, hold, u_own)
        folds.append({"steps": [lo, hi], "train_rows": int(train.sum()), "hold_rows": int(hold.sum()),
                      "embargo_rows": int(embargo.sum()), "rmse_vs_uIK": rmse,
                      "features_excluded_constant_on_train": excluded,
                      "diag_rmse_vs_S0D_own_labels": rmse_own})
    if folds[2]["rmse_vs_uIK"] != FOLD3_BINDING:
        raise Rev3nError(f"fold 3 must reproduce the frozen FAIL pair bit-exact: {folds[2]['rmse_vs_uIK']} != {FOLD3_BINDING}")
    binding = pass_rule([f["rmse_vs_uIK"] for f in folds])
    binding["per_fold_rmse_vs_uIK"] = [f["rmse_vs_uIK"] for f in folds]
    binding["source"] = "kNN5 threshold reused from rev3b R0a; instrument symmetrized by rev3n (median of 5 blocked folds)"
    own = pass_rule([f["diag_rmse_vs_S0D_own_labels"] for f in folds])
    src = S1.verify_source_holdout_constructible(view)
    return {"binding": binding, "folds": folds, "source_holdout_rev3m": src,
            "diagnostics": {
                "own_label_baseline_same_instrument": {"median_per_joint": own["median_per_joint"], "per_fold": own["per_fold_rmse_vs_uIK"] if "per_fold_rmse_vs_uIK" in own else [f["diag_rmse_vs_S0D_own_labels"] for f in folds],
                                                        "note": "S0D's OWN actions under the same instrument: rollout-certified behaviour, reported as instrument-resolution context; NOT a gate"},
                "fold3_reproduces_original_fail_bitexact": FOLD3_BINDING,
                "original_single_window_outcome": {"receipt": PIN_ORIGINAL_RECEIPT, "knee": FOLD3_BINDING[0], "ankle": FOLD3_BINDING[1],
                                                   "status": "preserved immutable evidence; superseded ONLY as pre-gate instrument"},
                "drift_on_task_rows_is_DIAGNOSTIC_only_rev3m": True,
                "gap_own_vs_ik": view["records"]["gap_own_vs_ik"],
                "mandatory_comparison_context": {"JUL_H0_july_final": {"offline_aggregate_rmse": 0.008144, "closed_loop": "500/500 (July guards 15/25mm)"},
                                                 "S0D": {"fit_holdout": [0.0811, 0.0793], "rollout": "500/500 (v3 guards)"}}},
            "future_postfit_rules": POSTFIT_RULES}


def run_pregate_rev3n() -> dict[str, Any]:
    lineage = verify_lineage_rev3n()
    view = S1.build_s1_view()          # unchanged rev3l view: 500 rows, u_IK labels, frozen gap check
    gate = pre_gate_rev3n(view)
    receipt = {"schema": "v26b_s1_pregate_rev3n.1",
               "token": "V26B-S1-REV3N-PREREG-MEDIAN",
               "amendment_rev3n": PIN_AMENDMENT_REV3N,
               "instrument": {"estimator": f"kNN k={K_NEIGHBOURS}", "labels": "u_IK AB06 (pinned nominal cache)",
                              "folds": [list(f) for f in FOLDS], "embargo_steps": EMBARGO,
                              "threshold_per_joint": RMSE_MAX, "primary_metric": "median of the 5 per-fold RMSE, per joint",
                              "unchanged_from_rev3l": ["estimator", "k", "labels", "train-only standardization", "constant-feature exclusion", "threshold"]},
               "lineage": lineage, "split": view["records"], "pre_gate": gate,
               "scope": "S1 symmetrized pre-gate ONLY: no fit (V26B-S1-FIT not granted), no export, no rollout",
               "code_digests": {"v26b_s1_pregate_rev3n.py": C.sha256_file(Path(__file__).resolve()),
                                "test_v26b_s1_pregate_rev3n.py": C.sha256_file(HERE / "test_v26b_s1_pregate_rev3n.py") if (HERE / "test_v26b_s1_pregate_rev3n.py").is_file() else None},
               "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}
    path = R.unique_artifact_path(VA.OUT_ROOT, f"v26b_s1_pregate_rev3n_receipt_{time.strftime('%Y%m%d_%H%M%S')}", ".json")
    VA._atomic_fill_reserved(path, json.dumps(receipt, indent=2, ensure_ascii=False, default=str) + "\n")
    return {"receipt_path": C.rel(path), "receipt_sha256": C.sha256_file(path), "gate": gate}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="V26B rev3n S1 symmetrized pre-gate (read-only)")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args(argv)
    if not args.execute:
        verify_lineage_rev3n()
        print(json.dumps({"mode": "dry", "lineage_ok": True}, indent=2))
        return 0
    out = run_pregate_rev3n()
    b = out["gate"]["binding"]
    print(json.dumps({"pregate_rev3n_pass": b["pass"], "median_per_joint": b["median_per_joint"],
                      "per_fold_rmse_vs_uIK": b["per_fold_rmse_vs_uIK"], "max_per_joint": b["max_per_joint"],
                      "n_folds_le_threshold": b["n_folds_le_threshold"],
                      "receipt": out["receipt_path"], "receipt_sha256": out["receipt_sha256"]}, indent=2))
    return 0 if b["pass"] else 3


if __name__ == "__main__":
    sys.exit(main())
