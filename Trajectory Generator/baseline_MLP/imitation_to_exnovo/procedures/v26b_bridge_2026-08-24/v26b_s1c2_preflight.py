"""V26B rev3y — S1C-2 preflight: observability/aliasing audit of the 35D state (read-only).

Token V26B-S1C-2-PREFLIGHT.  Answers, on the FROZEN splits and with leakage-safe controls, whether
the 35D inputs distinguish the rows of the continuous negative AB06 IK ankle window from their
non-negative neighbours, and applies the GO/NO-GO criterion frozen in rev3y BEFORE any
classification was run.

No fit, rollout, episode, collection, promotion, sigma choice or production change.
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

import v26b_s1c1_fit as F1  # noqa: E402        (rev3x lineage, unmodified)
import v26b_s1c_protocol as SC  # noqa: E402
import v26b_s1_pregate_rev3n as N  # noqa: E402  (frozen 5-fold geometry)
import v26b_s1_fit as FIT  # noqa: E402
import v26b_anchors as VA  # noqa: E402
import f0_common as C  # noqa: E402
import f2r_common as R  # noqa: E402


class PreflightError(RuntimeError):
    pass


AUTHORIZED_STAGE = "V26B-S1C-2-PREFLIGHT"
AMENDMENT_REV3Y = HERE / "v26b_amendment_rev3y_s1c2_preflight.json"
PIN_AMENDMENT_REV3Y = "eb54fd266461e56728dd5387f6019a3eb23243eb4c4e0e79541cfeaf520139c6"
ADDENDUM_STATUS = HERE / "v26b_addendum_rev3x_a_aggregate_status_semantics.json"
PIN_ADDENDUM_STATUS = "de53d529343edd5d82af786036368d7b885ad686af18469088a3dc8ccb563cf5"
S1C1_AGGREGATE = VA.OUT_ROOT / "candidates" / "v26b_s1c1_fit_aggregate_20260824_210626.json"
PIN_S1C1_AGGREGATE = "58874483683ff3f9eb6741f7f752fbb5b9c8fec96b3277c3543f0eae58b7d3e4"
FUTURE_FIT_TOKEN = "V26B-S1C-2-FIT"

DEAD_CLOCK = (0, 1)
PHASE_FAMILY = (14, 15, 16, 20, 21, 22, 23, 24)
K = 5
NULL_BLOCK = 25
NULL_REPS = 20
NULL_SEED = 2026
GO_BALANCED_ACC = 0.75
GO_RECALL = 0.60
GO_NULL_MARGIN = 0.10
OUT_DIR = VA.OUT_ROOT / "s1c2_preflight"


def _amendment() -> dict[str, Any]:
    got = C.sha256_file(AMENDMENT_REV3Y)
    if got != PIN_AMENDMENT_REV3Y:
        raise PreflightError(f"rev3y sha {got} != pinned")
    return json.loads(AMENDMENT_REV3Y.read_text(encoding="utf-8"))


def verify_lineage() -> dict[str, Any]:
    lin = F1.verify_lineage()
    for path, pin, key in ((AMENDMENT_REV3Y, PIN_AMENDMENT_REV3Y, "amendment_rev3y"),
                           (ADDENDUM_STATUS, PIN_ADDENDUM_STATUS, "addendum_rev3x_a_status"),
                           (S1C1_AGGREGATE, PIN_S1C1_AGGREGATE, "s1c1_aggregate_unmutated")):
        got = C.sha256_file(path)
        if got != pin:
            raise PreflightError(f"{key} sha {got} != pinned")
        lin[key] = got
    agg = json.loads(S1C1_AGGREGATE.read_text(encoding="utf-8"))
    if agg["offline_survivors"]:
        raise PreflightError("the S1C-1 aggregate no longer records zero survivors")
    lin["s1c1_survivors"] = agg["offline_survivors"]
    return lin


# --- data ------------------------------------------------------------------------------------------

def build() -> dict[str, Any]:
    view = FIT.build_s1_task()
    y = (SC.decode_action(view["u_ik"])[:, 1] < 0.0).astype(np.int64)
    obs = view["obs"].astype(np.float64)
    runs, cur = [], 0
    for f in y:
        if f:
            cur += 1
        elif cur:
            runs.append(cur); cur = 0
    if cur:
        runs.append(cur)
    return {"obs": obs, "y": y, "view": view, "runs": runs,
            "positives": int(y.sum()), "rows": int(y.size)}


def feature_sets() -> dict[str, list[int]]:
    keep_all = [i for i in range(len(R.FEATURE_NAMES_35)) if i not in DEAD_CLOCK]
    physical = [i for i in keep_all if i not in PHASE_FAMILY]
    return {"runtime_available": keep_all, "physical_only_diagnostic": physical}


def _knn_fold(obs, train, hold, y, cols, k=K):
    mu = obs[train][:, cols].mean(0); sd = obs[train][:, cols].std(0)
    keep = sd > 1e-9
    z = ((obs[:, cols] - mu) / np.where(keep, sd, 1.0))[:, keep]
    zt, zh, yt = z[train], z[hold], y[train]
    d2 = np.sum(zh ** 2, 1)[:, None] + np.sum(zt ** 2, 1)[None, :] - 2.0 * zh @ zt.T
    idx = np.argpartition(d2, k - 1, 1)[:, :k]
    votes = yt[idx].mean(1)
    return (votes >= 0.5).astype(np.int64), votes, int((~keep).sum())


def pooled_metrics(y_true, y_pred) -> dict[str, float]:
    y_true = np.asarray(y_true); y_pred = np.asarray(y_pred)
    tp = int(np.sum((y_pred == 1) & (y_true == 1))); fn = int(np.sum((y_pred == 0) & (y_true == 1)))
    tn = int(np.sum((y_pred == 0) & (y_true == 0))); fp = int(np.sum((y_pred == 1) & (y_true == 0)))
    tpr = tp / max(tp + fn, 1); tnr = tn / max(tn + fp, 1)
    prec = tp / max(tp + fp, 1)
    return {"balanced_accuracy": 0.5 * (tpr + tnr), "recall_positive": tpr, "specificity": tnr,
            "precision_positive": prec, "tp": tp, "fp": fp, "tn": tn, "fn": fn,
            "accuracy": (tp + tn) / max(len(y_true), 1)}


def classify(data: Mapping[str, Any], cols: Sequence[int], y: np.ndarray | None = None) -> dict[str, Any]:
    obs = data["obs"]; y = data["y"] if y is None else y
    pred = np.zeros_like(y); covered = np.zeros(len(y), bool); per_fold = []
    for lo, hi in N.FOLDS:
        tr, ho, _ = N.fold_masks(lo, hi)
        p, _, dropped = _knn_fold(obs, tr, ho, y, list(cols))
        pred[ho] = p; covered[ho] = True
        per_fold.append({"steps": [lo, hi], "positives_in_hold": int(y[ho].sum()),
                         "constant_features_dropped": dropped, **pooled_metrics(y[ho], p)})
    if not covered.all():
        raise PreflightError("the 5 folds do not partition the 500 rows")
    return {"pooled": pooled_metrics(y, pred), "per_fold": per_fold}


def block_permutation_null(data: Mapping[str, Any], cols: Sequence[int]) -> dict[str, Any]:
    rng = np.random.default_rng(NULL_SEED)
    y = data["y"]; n = len(y)
    nb = n // NULL_BLOCK
    vals = []
    for _ in range(NULL_REPS):
        blocks = y.reshape(nb, NULL_BLOCK)[rng.permutation(nb)].reshape(-1)
        vals.append(classify(data, cols, y=blocks)["pooled"]["balanced_accuracy"])
    v = np.asarray(vals)
    return {"block_size": NULL_BLOCK, "repetitions": NULL_REPS, "seed": NULL_SEED,
            "mean_balanced_accuracy": float(v.mean()), "max": float(v.max()), "std": float(v.std())}


def nn_distance_contrast(data: Mapping[str, Any], cols: Sequence[int]) -> dict[str, Any]:
    """Aliasing check on the frozen 380 training rows: cross-class vs intra-class NN distance."""
    obs = data["obs"]; y = data["y"]; tr = data["view"]["train"]
    X = obs[tr][:, list(cols)]; yy = y[tr]
    mu, sd = X.mean(0), X.std(0)
    keep = sd > 1e-9
    Z = ((X - mu) / np.where(keep, sd, 1.0))[:, keep]
    d2 = np.sum(Z ** 2, 1)[:, None] + np.sum(Z ** 2, 1)[None, :] - 2.0 * Z @ Z.T
    np.fill_diagonal(d2, np.inf)
    out = {}
    for cls, name in ((1, "positive"), (0, "negative")):
        m = yy == cls
        same = np.sqrt(np.maximum(d2[np.ix_(m, m)].min(1), 0.0))
        other = np.sqrt(np.maximum(d2[np.ix_(m, ~m)].min(1), 0.0))
        out[name] = {"rows": int(m.sum()),
                     "intra_class_nn_median": float(np.median(same)), "cross_class_nn_median": float(np.median(other)),
                     "ratio_cross_over_intra_median": float(np.median(other) / max(np.median(same), 1e-12)),
                     "fraction_with_cross_nn_closer_than_intra": float(np.mean(other < same))}
    out["reading"] = ("a ratio near 1 and a large fraction of rows whose nearest neighbour belongs to the OTHER class "
                      "indicate aliasing: the state does not separate the window from its neighbours")
    return out


def conditional_variance(data: Mapping[str, Any], cols: Sequence[int]) -> dict[str, Any]:
    """kNN regression of the signed IK ankle target: how much variance the state explains."""
    obs = data["obs"]; tgt = SC.decode_action(data["view"]["u_ik"])[:, 1]
    pred = np.zeros_like(tgt)
    for lo, hi in N.FOLDS:
        tr, ho, _ = N.fold_masks(lo, hi)
        mu = obs[tr][:, list(cols)].mean(0); sd = obs[tr][:, list(cols)].std(0)
        keep = sd > 1e-9
        z = ((obs[:, list(cols)] - mu) / np.where(keep, sd, 1.0))[:, keep]
        zt, zh = z[tr], z[ho]
        d2 = np.sum(zh ** 2, 1)[:, None] + np.sum(zt ** 2, 1)[None, :] - 2.0 * zh @ zt.T
        idx = np.argpartition(d2, K - 1, 1)[:, :K]
        pred[ho] = tgt[tr][idx].mean(1)
    resid = tgt - pred
    return {"var_target": float(np.var(tgt)), "var_residual": float(np.var(resid)),
            "residual_over_target_variance": float(np.var(resid) / max(np.var(tgt), 1e-12)),
            "rmse": float(np.sqrt(np.mean(resid ** 2)))}


def go_no_go(runtime: Mapping[str, Any], null: Mapping[str, Any]) -> dict[str, Any]:
    ba = runtime["pooled"]["balanced_accuracy"]; rec = runtime["pooled"]["recall_positive"]
    margin = ba - null["mean_balanced_accuracy"]
    conds = {"balanced_accuracy>=0.75": bool(ba >= GO_BALANCED_ACC),
             "recall_positive>=0.60": bool(rec >= GO_RECALL),
             "margin_over_block_null>=0.10": bool(margin >= GO_NULL_MARGIN)}
    ok = all(conds.values())
    return {"criterion": {"balanced_accuracy_min": GO_BALANCED_ACC, "recall_min": GO_RECALL,
                          "null_margin_min": GO_NULL_MARGIN, "frozen_in": "rev3y, before any classification"},
            "observed": {"balanced_accuracy": ba, "recall_positive": rec,
                         "null_mean_balanced_accuracy": null["mean_balanced_accuracy"], "margin": margin},
            "conditions": conds, "verdict": "GO" if ok else "NO_GO",
            "consequence": ("the S1C-2 protocol may be prepared; the fit still requires a separate architect token"
                            if ok else
                            "the S1C-2 fit is NOT authorised; the minimal architectural correction is proposed as a separate stage")}


def run_fit(*_, **kw):
    raise PreflightError(f"the S1C-2 fit requires --authorized-stage {FUTURE_FIT_TOKEN}, NOT granted; got {kw.get('authorized_stage')!r}")


def run_audit(*, authorized_stage: str | None, out_dir: Path = OUT_DIR) -> dict[str, Any]:
    if authorized_stage != AUTHORIZED_STAGE:
        raise PreflightError(f"requires --authorized-stage {AUTHORIZED_STAGE}; got {authorized_stage!r}")
    lineage = verify_lineage()
    data = build()
    fs = feature_sets()
    res = {}
    for name, cols in fs.items():
        res[name] = {"n_features": len(cols), "classification": classify(data, cols),
                     "conditional_variance": conditional_variance(data, cols),
                     "nn_distance_contrast": nn_distance_contrast(data, cols)}
    null = block_permutation_null(data, fs["runtime_available"])
    verdict = go_no_go(res["runtime_available"]["classification"], null)
    am = _amendment()
    receipt = {"schema": "v26b_s1c2_preflight.1", "authorized_stage": AUTHORIZED_STAGE,
               "amendment_rev3y": PIN_AMENDMENT_REV3Y, "lineage": lineage,
               "label": {"rule": "y = 1 iff the decoded IK ankle target is negative",
                         "positives": data["positives"], "rows": data["rows"], "run_lengths": data["runs"]},
               "feature_sets": {k: {"count": len(v), "excluded": ([R.FEATURE_NAMES_35[i] for i in DEAD_CLOCK] if k == "runtime_available"
                                                                   else [R.FEATURE_NAMES_35[i] for i in DEAD_CLOCK + PHASE_FAMILY])} for k, v in fs.items()},
               "results": res, "block_permutation_null": null, "go_no_go": verdict,
               "objective_family_comparison": am["OBJECTIVE_FAMILY_COMPARISON_NO_TRAINING"],
               "s1c2_protocol_conditional": am["S1C_2_PROTOCOL_CONDITIONAL_ON_GO"],
               "minimal_architectural_correction_if_no_go": am["MINIMAL_ARCHITECTURAL_CORRECTION_IF_NO_GO"],
               "dataset_limits_declared": [
                   "a SINGLE deterministic trajectory of 500 rows: 97 positives in 7 runs, so the effective sample of windows is 7, not 97",
                   "the states are the ones S0D visited: the audit describes observability on THAT distribution, not on states another actor would reach",
                   "fold [301,400] holds only 5 positives, so its per-fold figures are weak and are reported separately",
                   "no new collection was made and none is proposed here"],
               "executed_in_this_stage": {"fit": False, "rollout": False, "episode": False, "collection": False,
                                          "promotion": False, "production_change": False, "sigma_choice": False},
               "code_digests": {"v26b_s1c2_preflight.py": C.sha256_file(Path(__file__).resolve()),
                                "test_v26b_s1c2_preflight.py": C.sha256_file(HERE / "test_v26b_s1c2_preflight.py") if (HERE / "test_v26b_s1c2_preflight.py").is_file() else None},
               "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}
    out_dir = Path(out_dir); out_dir.mkdir(parents=True, exist_ok=True)
    path = R.unique_artifact_path(out_dir, f"v26b_s1c2_preflight_{time.strftime('%Y%m%d_%H%M%S')}", ".json")
    VA._atomic_fill_reserved(path, json.dumps(receipt, indent=2, ensure_ascii=False, default=str) + "\n")
    return {"receipt_path": C.rel(path), "receipt_sha256": C.sha256_file(path), "receipt": receipt}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="V26B rev3y S1C-2 preflight audit (read-only)")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--authorized-stage", default=None)
    args = parser.parse_args(argv)
    if not args.execute:
        verify_lineage()
        print(json.dumps({"mode": "lineage-only", "ok": True}, indent=2))
        return 0
    out = run_audit(authorized_stage=args.authorized_stage)
    r = out["receipt"]
    print(json.dumps({"receipt": out["receipt_path"], "receipt_sha256": out["receipt_sha256"],
                      "runtime_available": r["results"]["runtime_available"]["classification"]["pooled"],
                      "physical_only": r["results"]["physical_only_diagnostic"]["classification"]["pooled"],
                      "null": r["block_permutation_null"], "go_no_go": r["go_no_go"],
                      "executed": r["executed_in_this_stage"]}, indent=2, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
