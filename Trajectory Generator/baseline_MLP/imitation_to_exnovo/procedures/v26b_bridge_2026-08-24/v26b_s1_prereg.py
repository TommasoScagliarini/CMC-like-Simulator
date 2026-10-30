"""V26B rev3l — S1 IK-AB06 preregistration tooling: dataset view + read-only pre-gate.

Token V26B-S1-PREREG-READONLY: split + kNN pre-gate + diagnostics ONLY; the fit
is locked behind the distinct future token V26B-S1-FIT (not granted).
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any, Mapping

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import v26b_anchors as VA  # noqa: E402
import v26b_s0d_rollout as SR  # noqa: E402
import f0_common as C  # noqa: E402
import f1_dataset as DS  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_labeller as L  # noqa: E402


class S1Error(RuntimeError):
    pass


AMENDMENT_REV3L = HERE / "v26b_amendment_rev3l_s1_ik_ab06_prereg.json"
PIN_AMENDMENT_REV3L = "7545201df4e5aefb4fdccca9efc1425a9a4dde581a4c20269c0d3fd86aa794d5"
FUTURE_FIT_TOKEN = "V26B-S1-FIT"
PRE_GATE_RMSE_MAX = 0.15   # reused rev3b R0a gate
PIN_S0D_ROLLOUT_RECEIPT = "cbec1a671b7cdf2980881ec4ca33f69534e66d94cbbfa86c7b9cd9c1a39412b7"
AMENDMENT_REV3M = HERE / "v26b_amendment_rev3m_s1_gate_correction.json"
PIN_AMENDMENT_REV3M = "8af3ad654e49e19f5a6d2dbcf128faf5ce424206a5def715d177f7fbda93e0c0"
PIN_NOMINAL_CACHE = "3dd878d4d6d2930d730c1a67f39d6799f20221e7659a435c02d062bfd553d9b0"
HOLD_WINDOW = (201, 300)
EMBARGO = 10
GAP_EXPECT = {"knee_mean": 0.3758, "ankle_mean": 0.3014}  # frozen structural fact (rev3l)


def verify_lineage_s1() -> dict[str, Any]:
    lin = SR.verify_lineage()  # S0D chain (fit receipt, module, actor, addendum, production pins)
    got = C.sha256_file(AMENDMENT_REV3L)
    if got != PIN_AMENDMENT_REV3L:
        raise S1Error(f"rev3l sha {got} != pinned")
    lin["amendment_rev3l"] = got
    got = C.sha256_file(AMENDMENT_REV3M)
    if got != PIN_AMENDMENT_REV3M:
        raise S1Error(f"rev3m sha {got} != pinned")
    lin["amendment_rev3m"] = got
    got = C.sha256_file(SR.JOB_DIR / SR.RECEIPT_NAME)
    if got != PIN_S0D_ROLLOUT_RECEIPT:
        raise S1Error("S0D rollout receipt changed")
    cc = L.load_cache(R.OUT_CACHE, "nominal")
    if cc.digest() != PIN_NOMINAL_CACHE:
        raise S1Error("nominal AB06 IK cache digest changed")
    lin["s0d_rollout_receipt"] = PIN_S0D_ROLLOUT_RECEIPT
    lin["nominal_cache"] = PIN_NOMINAL_CACHE
    return lin


def build_s1_view() -> dict[str, Any]:
    """500 S0D-visited rows + u_IK AB06 labels + S0D own actions (diagnostics)."""
    traj = DS.trajectory_from_job(SR.JOB_DIR, expected_width=R.ENV_ACTOR_WIDTH)
    if traj["trace_sha256"] != json.loads((SR.JOB_DIR / SR.RECEIPT_NAME).read_text())["analysis"]["trace_sha256"]:
        raise S1Error("S0D trace digest != receipt")
    obs = np.asarray(traj["obs35"], dtype=np.float64).astype(np.float32)
    t = np.asarray(traj["t_pre"], dtype=np.float64)
    cc = L.load_cache(R.OUT_CACHE, "nominal")
    idx = cc.lookup(t)  # exact grid, fail-closed
    u_ik = cc.ik_action[idx].astype(np.float64)
    u_own = np.asarray(traj["b_raw_action"], dtype=np.float64)
    if obs.shape[0] != 500:
        raise S1Error(f"S0D rows {obs.shape[0]} != 500")
    gap = np.abs(u_own - u_ik)
    if abs(float(gap[:, 0].mean()) - GAP_EXPECT["knee_mean"]) > 1e-3 or abs(float(gap[:, 1].mean()) - GAP_EXPECT["ankle_mean"]) > 1e-3:
        raise S1Error(f"own-vs-IK gap deviates from the frozen rev3l fact: {gap.mean(0)}")
    step = np.arange(1, 501)
    hold = (step >= HOLD_WINDOW[0]) & (step <= HOLD_WINDOW[1])
    embargo = ((step >= HOLD_WINDOW[0] - EMBARGO) & (step < HOLD_WINDOW[0])) | ((step > HOLD_WINDOW[1]) & (step <= HOLD_WINDOW[1] + EMBARGO))
    train = ~(hold | embargo)
    return {"obs": obs, "u_ik": u_ik, "u_own": u_own, "train": train, "hold": hold, "embargo": embargo,
            "records": {"rows": 500, "train": int(train.sum()), "hold": int(hold.sum()), "embargo": int(embargo.sum()),
                        "hold_window_steps": list(HOLD_WINDOW), "embargo_steps": EMBARGO,
                        "gap_own_vs_ik": {"knee_mean": float(gap[:, 0].mean()), "knee_p90": float(np.quantile(gap[:, 0], .9)), "knee_max": float(gap[:, 0].max()), "ankle_mean": float(gap[:, 1].mean()), "ankle_max": float(gap[:, 1].max())}}}


def verify_source_holdout_constructible(view: Mapping[str, Any]) -> dict[str, Any]:
    """rev3m: the Q1 source holdout = rev3k provenance-held anchor rows, labels = S0D mean
    same-state, minus any bitwise overlap with the 500 IK task rows (excluded + counted)."""
    import v26b_s0d as S0
    split = S0.build_split()
    ho = split["assign"]["hold_idx"]
    src_obs = split["obs"][ho]
    task_keys = {view["obs"][i].tobytes() for i in range(view["obs"].shape[0])}
    shared = [i for i in range(src_obs.shape[0]) if src_obs[i].tobytes() in task_keys]
    usable = src_obs.shape[0] - len(shared)
    return {"source_holdout_rows_rev3k": int(src_obs.shape[0]), "bitwise_shared_with_task_excluded": len(shared), "usable_rows": int(usable),
            "constructible_without_leakage": bool(usable > 0),
            "labels_rule": "S0D deterministic mean same-state (numpy forward, actor 481dd0d2...); disjoint row sets -> no label conflict by construction",
            "provenance_aware": "held rows come from whole held-out trajectories (rev3k); task rows from the S0D rollout trace"}


def pre_gate(view: Mapping[str, Any]) -> dict[str, Any]:
    obs = view["obs"].astype(np.float64)
    tr, ho = view["train"], view["hold"]
    mu = obs[tr].mean(0); sd = obs[tr].std(0); keep = sd > 1e-9
    excluded = [int(i) for i in np.where(~keep)[0]]
    z = ((obs - mu) / np.where(keep, sd, 1.0))[:, keep]
    zt = z[tr]; yt = view["u_ik"][tr]; b2 = np.sum(zt ** 2, 1)
    zh = z[ho]; preds = np.empty((int(ho.sum()), 2))
    for s in range(0, zh.shape[0], 256):
        a = zh[s: s + 256]
        d2 = np.sum(a ** 2, 1)[:, None] + b2[None, :] - 2.0 * a @ zt.T
        idx = np.argpartition(d2, 4, 1)[:, :5]
        preds[s: s + 256] = yt[idx].mean(1)
    rmse_ik = np.sqrt(np.mean((preds - view["u_ik"][ho]) ** 2, axis=0)).tolist()
    # diagnostics: kNN with OWN labels (S0D behaviour identifiability, context) + comparisons
    yt2 = view["u_own"][tr]; preds2 = np.empty_like(preds)
    for s in range(0, zh.shape[0], 256):
        a = zh[s: s + 256]
        d2 = np.sum(a ** 2, 1)[:, None] + b2[None, :] - 2.0 * a @ zt.T
        idx = np.argpartition(d2, 4, 1)[:, :5]
        preds2[s: s + 256] = yt2[idx].mean(1)
    rmse_own = np.sqrt(np.mean((preds2 - view["u_own"][ho]) ** 2, axis=0)).tolist()
    src = verify_source_holdout_constructible(view)
    out = {"binding": {"knn_k5_rmse_vs_uIK_holdout": rmse_ik, "max_per_joint": PRE_GATE_RMSE_MAX, "source": "reused rev3b R0a gate", "pass": bool(all(v <= PRE_GATE_RMSE_MAX for v in rmse_ik))},
           "source_holdout_rev3m": src,
           "features_excluded_constant_on_train": excluded,
           "diagnostics": {"knn_rmse_vs_S0D_own_holdout": rmse_own, "drift_on_task_rows_is_DIAGNOSTIC_only_rev3m": True, "gap_own_vs_ik": view["records"]["gap_own_vs_ik"],
                           "mandatory_comparison_context": {"JUL_H0_july_final": {"offline_aggregate_rmse": 0.008144, "closed_loop": "500/500 (July guards 15/25mm)"}, "S0D": {"fit_holdout": [0.0811, 0.0793], "rollout": "500/500 (v3 guards)"}}}}
    return out


def run_fit(*, authorized_stage: str | None = None, **_):
    raise S1Error(f"the S1 fit requires --authorized-stage {FUTURE_FIT_TOKEN}, NOT granted (rev3l authorises prereg/pre-gate only); got {authorized_stage!r}")


def run_pregate() -> dict[str, Any]:
    lineage = verify_lineage_s1()
    view = build_s1_view()
    gate = pre_gate(view)
    receipt = {"schema": "v26b_s1_pregate.1", "amendment_rev3l": PIN_AMENDMENT_REV3L, "lineage": lineage, "split": view["records"], "pre_gate": gate,
               "scope": "S1 prereg/pre-gate ONLY (token V26B-S1-PREREG-READONLY): no fit (V26B-S1-FIT not granted), no export, no rollout",
               "code_digests": {"v26b_s1_prereg.py": C.sha256_file(Path(__file__).resolve()), "test_v26b_s1_prereg.py": C.sha256_file(HERE / "test_v26b_s1_prereg.py") if (HERE / "test_v26b_s1_prereg.py").is_file() else None},
               "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}
    path = R.unique_artifact_path(VA.OUT_ROOT, f"v26b_s1_pregate_receipt_{time.strftime('%Y%m%d_%H%M%S')}", ".json")
    VA._atomic_fill_reserved(path, json.dumps(receipt, indent=2, ensure_ascii=False, default=str) + "\n")
    return {"receipt_path": C.rel(path), "receipt_sha256": C.sha256_file(path), "gate": gate, "split": view["records"]}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="V26B rev3l S1 prereg pre-gate (read-only)")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args(argv)
    if not args.execute:
        verify_lineage_s1()
        print(json.dumps({"mode": "dry", "lineage_ok": True}, indent=2))
        return 0
    out = run_pregate()
    print(json.dumps({"pregate_pass": out["gate"]["binding"]["pass"], "rmse_vs_uIK": out["gate"]["binding"]["knn_k5_rmse_vs_uIK_holdout"], "receipt": out["receipt_path"], "receipt_sha256": out["receipt_sha256"]}, indent=2))
    return 0 if out["gate"]["binding"]["pass"] else 3


if __name__ == "__main__":
    sys.exit(main())
