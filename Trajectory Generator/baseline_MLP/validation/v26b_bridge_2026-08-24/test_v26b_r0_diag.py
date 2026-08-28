"""Self-test for the additive R0 feasibility diagnostic (synthetic + read-only real checks)."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import v26b_r0_diag as D  # noqa: E402
import v26b_v2 as V2  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_refit as RF  # noqa: E402

CHECKS = 0


def check(cond: bool, what: str) -> None:
    global CHECKS
    assert cond, what
    CHECKS += 1


def expect(fn, exc, what: str):
    global CHECKS
    try:
        fn()
    except exc as e:
        CHECKS += 1
        return e
    raise AssertionError(f"expected {exc.__name__}: {what}")


def synth_role(n, seed, purpose, target_offset=0.0):
    rng = np.random.default_rng(seed)
    obs = rng.normal(0, 0.5, (n, 35)).astype(np.float32)
    return {"obs35": obs, "actions": (rng.uniform(-0.5, 0.5, (n, 2)) + target_offset).astype(np.float32),
            "clock": rng.normal(0, 0.7, (n, 2)).astype(np.float32), "seed": np.full(n, 1000, np.int64),
            "purpose": np.asarray([purpose] * n, dtype=str)}


def main() -> int:
    # --- (5) full-digest cache pins (real, read-only) -----------------------------------------
    caches = D.verify_ik_caches_full()
    check(all(len(v) == 64 for v in caches.values()), "full 64-hex cache digests verified")
    check(all(caches[s].startswith(V2.PIN_F2R_CACHE_DIGESTS[s]) for s in R.STARTS), "v26b_v2 prefix pins consistent with the full pins")
    old = dict(D.IK_CACHE_DIGESTS_FULL)
    try:
        D.IK_CACHE_DIGESTS_FULL = {**old, "nominal": "0" * 64}
        expect(D.verify_ik_caches_full, D.DiagError, "tampered full pin -> refused")
    finally:
        D.IK_CACHE_DIGESTS_FULL = old

    # --- separability: known-answer synthetic cases -------------------------------------------
    rng = np.random.default_rng(5)
    base = rng.normal(0, 1.0, (60, 35)).astype(np.float32)
    far = base + np.float32(50.0)
    A = {"obs35": base, "actions": rng.uniform(-1, 1, (60, 2)).astype(np.float32)}
    B = {"obs35": far, "actions": rng.uniform(-1, 1, (60, 2)).astype(np.float32)}
    sep = D.separability_analysis(A, B)
    check(sep["nn_distance_std_units"]["cross_task_to_pres"]["p50"] > 3 * sep["nn_distance_std_units"]["intra_task"]["p95"], "far clouds: cross-NN >> intra-NN")
    near = {"obs35": base + rng.normal(0, 1e-3, base.shape).astype(np.float32), "actions": (A["actions"] + np.float32(0.4)).astype(np.float32)}
    sep2 = D.separability_analysis(A, near)
    check(sep2["nn_distance_std_units"]["cross_task_to_pres"]["p50"] < sep2["nn_distance_std_units"]["intra_task"]["p50"], "near clouds: cross-NN below intra-NN")
    check(abs(sep2["target_gap_on_cross_nn_pairs_abs"]["knee"]["mean"] - 0.4) < 0.05, "target gap on cross pairs recovered (~0.4)")
    note = sep2["language_note"].lower()
    check(sep2["standardisation"]["features_used"] <= 35 and "near states" in note and "never claimed identical" in note, "language: near states, never claimed identical")

    # --- kNN bounds: conflict vs separable known answers --------------------------------------
    # identical states, conflicting targets -> union kNN error ~ half-gap; intra-role ~ small
    X = rng.normal(0, 1.0, (200, 35)).astype(np.float32)
    tA = {"obs35": X, "actions": np.full((200, 2), -0.2, np.float32)}
    tB = {"obs35": X.copy(), "actions": np.full((200, 2), +0.2, np.float32)}
    kb = D.knn_holdout_bounds(tA, tB, k_list=(1,), holdout_stride=5)
    union_rmse = kb["k1"]["union_train"]["pres_holdout_rmse_vs_uT"]
    intra_rmse = kb["k1"]["intra_role_train_only"]["pres_holdout_rmse_vs_uT"]
    check(min(union_rmse) > 0.15 and max(intra_rmse) < 1e-6, "conflicting identical states: union kNN ~ half-gap, intra ~ 0")
    # far clouds, per-role constant targets -> union kNN succeeds on both
    tC = {"obs35": rng.normal(0, 1, (200, 35)).astype(np.float32), "actions": np.full((200, 2), -0.2, np.float32)}
    tD = {"obs35": rng.normal(0, 1, (200, 35)).astype(np.float32) + np.float32(30.0), "actions": np.full((200, 2), +0.2, np.float32)}
    kb2 = D.knn_holdout_bounds(tC, tD, k_list=(1,), holdout_stride=5)
    check(max(kb2["k1"]["union_train"]["pres_holdout_rmse_vs_uT"]) < 1e-6 and max(kb2["k1"]["union_train"]["task_holdout_rmse_vs_uIK"]) < 1e-6, "separable clouds: union kNN exact on both roles")

    # --- mono-role fit: sanity on synthetic ----------------------------------------------------
    sys.path.insert(0, str(HERE))
    from test_v26b_v2 import synth_init_35
    init = synth_init_35()
    role = synth_role(64, 9, "ik_alt_stoch")
    budget = {"epochs": 40, "batch_size": 16, "lr": 1e-3, "seed": 2026, "clip_weight": 1.0, "lambda_phi": 0.1, "lambda_anchor": 0.0}
    st, rep = D.fit_mono_role(init, role, budget=budget)
    check(rep["history_first_last"]["last"]["loss"] < rep["history_first_last"]["first"]["loss"] * 0.5, "mono-role loss decreases on its role")
    check(tuple(st.keys()) == RF.EXPECTED_KEY_ORDER and np.all(np.asarray(st["pi.0.0.weight"])[:, 0:2] == 0.0), "mono-role output: 10 keys, clock zero")
    check(np.all(np.asarray(st["pi.1.weight"])[2:4] == 0.0) and np.all(np.asarray(st["pi.1.bias"])[2:4] == np.float32(np.log(0.005))), "log-std constant preserved")
    bad = {**role, "purpose": np.asarray(["det"] * 64, dtype=str)}
    expect(lambda: D.fit_mono_role(init, bad, budget=budget), Exception, "invalid purpose refused in mono-role fit")
    check(not hasattr(D, "export_r0") and "export" not in [n for n in dir(D) if "export" in n.lower()], "diagnostic module has NO export path")

    # --- frozen artefact loading (real, read-only) --------------------------------------------
    task, pres, shas = D.load_r0_datasets()
    check(task["obs35"].shape == (12876, 35) and pres["obs35"].shape == (6438, 35), "frozen R0 datasets load and match the receipt digests")
    check(len(shas["task"]) == 64 and len(shas["pres"]) == 64, "dataset digests recorded")

    print(f"SELFTEST PASS ({CHECKS} checks)")
    return 0


def json_dumps_low(obj) -> str:
    import json
    return json.dumps(obj).lower()


if __name__ == "__main__":
    raise SystemExit(main())
