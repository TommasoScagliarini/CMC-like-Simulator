"""Self-test of f2r_observability on synthetic StartData (S0: no real data, no rollout).

Two synthetic worlds with 3 JUL-like starts + 3 anchor-like starts (different
phase offsets at reset):

* ``informative``: four kinematic columns are smooth harmonics of the
  prescribed phase (+ small noise), every other column is noise; the online
  clock triplet is (0, 1, 0) before ``first_cycle_step`` and a lagged clock
  after -> ridge and MLP pre-cycle PASS;
* ``noise``: the informative columns are replaced by noise -> FAIL.

Plus the structural asserts: fold leakage, row-hash leakage, column
selection, forbidden inputs, train-only standardisation, phase arithmetic.
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import f2r_common as R  # noqa: E402
import f1_obs_adapter as OA  # noqa: E402
import f2r_observability as OBS  # noqa: E402

CHECKS = 0


def check(cond: bool, label: str) -> None:
    global CHECKS
    CHECKS += 1
    if not cond:
        raise AssertionError(f"CHECK FAILED: {label}")


def expect(fn, exc, label):
    try:
        fn()
    except exc:
        check(True, label)
        return
    raise AssertionError(f"expected {exc.__name__}: {label}")


HS = [10.0 + 1.1 * k for k in range(16)]  # prescribed sound-side heel strikes (synthetic)
N_ROWS = 300
FIRST_CYCLE = 150


def make_world(*, informative: bool, seed: int = 0, n: int = N_ROWS, first_cycle: int = FIRST_CYCLE) -> tuple[dict[str, OBS.StartData], dict[str, OBS.StartData]]:
    rng = np.random.default_rng(seed)
    jul: dict[str, OBS.StartData] = {}
    anchors: dict[str, OBS.StartData] = {}
    for start in R.STARTS:
        for kind, table in (("jul_synthetic", jul), ("anchor_synthetic", anchors)):
            reset = 11.99 + R.EXACT_STARTS[start]  # same start -> same env reset for every policy (real structure)
            t_pre = reset + 0.01 * np.arange(n)
            phi = OBS.prescribed_phase_array(t_pre, HS)
            X = rng.standard_normal((n, 35))
            X[:, 0], X[:, 1] = 0.0, 1.0  # dead prescribed clock pair under v3
            if informative:
                gain = 1.0 + 0.05 * rng.standard_normal()  # actor-specific amplitude (anchors come from another actor)
                X[:, 2] = -0.75 + 0.6 * gain * np.sin(2 * np.pi * phi) + 0.01 * rng.standard_normal(n)
                X[:, 3] = 0.6 * gain * np.cos(2 * np.pi * phi) + 0.01 * rng.standard_normal(n)
                X[:, 4] = 0.3 * np.sin(4 * np.pi * phi) + 0.01 * rng.standard_normal(n)
                X[:, 5] = 0.3 * np.cos(4 * np.pi * phi) + 0.01 * rng.standard_normal(n)
            X[:first_cycle, 14], X[:first_cycle, 15], X[:first_cycle, 16] = 0.0, 1.0, 0.0
            X[first_cycle:, 14] = np.sin(2 * np.pi * (phi[first_cycle:] - 0.1))
            X[first_cycle:, 15] = np.cos(2 * np.pi * (phi[first_cycle:] - 0.1))
            X[first_cycle:, 16] = 1.1
            if kind.startswith("anchor"):
                X[0] = jul[start].X[0]  # policy-independent reset observation: bitwise identical to the JUL row 0 of the same start
            table[start] = OBS.StartData(start=start, X=X, phi=phi, first_cycle_step=first_cycle, source=kind, t_pre=t_pre, reset_time=reset)
    return jul, anchors


def write_fake_job(dir_: Path, *, steps: int, start: str, reset_time: float, first_cycle: int, seed: int = 5) -> None:
    rng = np.random.default_rng(seed)
    dir_.mkdir(parents=True)
    times = [reset_time + 0.01 * (k + 1) for k in range(steps)]
    rows = []
    for k in range(steps):
        obs = rng.standard_normal(35).tolist()
        obs[0], obs[1] = 0.0, 1.0
        obs[14], obs[15], obs[16] = (0.0, 1.0, 0.0) if k < first_cycle else (0.2, 0.9, 1.3)
        rows.append({"step": k + 1, "time": times[k], "actor_observation_vector_before": obs, "raw_policy_action": [0.1, -0.2]})
    (dir_ / OBS.TRACE_FILE).write_text(json.dumps(rows), encoding="utf-8")
    (dir_ / OBS.RESET_FILE).write_text(json.dumps({"time": reset_time}), encoding="utf-8")
    (dir_ / OBS.SUMMARY_FILE).write_text(json.dumps({"ok": True, "steps": steps, "n_actor": 35, "action_seed": 123, "action_selection": "deterministic", "episode_start_offset_s": R.EXACT_STARTS[start]}), encoding="utf-8")


def main() -> int:
    t0 = time.time()
    protocol = R.load_protocol()
    p0 = protocol["p0"]
    names = list(R.FEATURE_NAMES_35)

    # --- phase arithmetic ---------------------------------------------------------------
    phi = np.linspace(0.0, 1.0, 97, endpoint=False)
    check(np.allclose(OBS.phase_from_sincos(OBS.sincos_from_phase(phi)), phi, atol=1e-12), "phase_from_sincos round trip")
    check(OBS.phase_from_sincos(np.array([[0.0, 1.0]]))[0] == 0.0 and abs(OBS.phase_from_sincos(np.array([[0.0, -1.0]]))[0] - 0.5) < 1e-12, "phase 0 at (0,1), 0.5 at (0,-1)")
    check(np.allclose(OBS.circular_error(np.array([0.02, 0.98, 0.5]), np.array([0.98, 0.02, 0.0])), [0.04, 0.04, 0.5]), "circular error wraps")
    expect(lambda: OBS.phase_from_sincos(np.zeros((3, 3))), R.F2RContractError, "phase_from_sincos shape")
    # prescribed_phase vs direct computation (bracket, before first, after last) and wrap
    hs = [10.0, 11.0, 12.5]
    check(abs(OBS.prescribed_phase(10.25, hs) - 0.25) < 1e-12 and abs(OBS.prescribed_phase(11.75, hs) - 0.5) < 1e-12, "prescribed_phase bracket")
    check(abs(OBS.prescribed_phase(9.75, hs) - 0.75) < 1e-12, "prescribed_phase before first strike extrapolates with the first period and wraps")
    check(abs(OBS.prescribed_phase(14.0, hs) - 0.0) < 1e-12 and abs(OBS.prescribed_phase(13.25, hs) - 0.5) < 1e-12, "prescribed_phase after last strike extrapolates with the last period")
    check(abs(OBS.prescribed_phase(10.25, hs, offset=0.5) - 0.75) < 1e-12 and abs(OBS.prescribed_phase(10.75, hs, offset=0.5) - 0.25) < 1e-12, "prescribed_phase offset wraps")
    check(all(0.0 <= OBS.prescribed_phase(t, hs) < 1.0 for t in np.linspace(5.0, 20.0, 301)), "prescribed_phase in [0,1)")
    direct = [((t - 10.0) / 1.0) % 1.0 if t < 11.0 else ((t - 11.0) / 1.5) % 1.0 for t in (10.3, 11.2, 12.4)]
    check(np.allclose([OBS.prescribed_phase(t, hs) for t in (10.3, 11.2, 12.4)], direct), "prescribed_phase equals the direct formula")
    expect(lambda: OBS.prescribed_phase(1.0, [10.0]), R.F2RContractError, "prescribed_phase needs >= 2 strikes")
    expect(lambda: OBS.prescribed_phase(1.0, [10.0, 9.0]), R.F2RContractError, "prescribed_phase needs sorted strikes")

    # --- column selection -------------------------------------------------------------
    pre = OBS.select_columns(names, pre_cycle=True)
    post = OBS.select_columns(names, pre_cycle=False)
    check(0 not in pre and 1 not in pre and 0 not in post and 1 not in post, "select_columns excludes (0,1) always")
    check(all(i not in pre for i in R.ONLINE_CLOCK_TRIPLET) and all(i in post for i in R.ONLINE_CLOCK_TRIPLET), "select_columns excludes (14,15,16) only when pre_cycle")
    check(len(pre) == 30 and len(post) == 33 and pre == sorted(pre), "pre 30 / post 33 columns")
    for bad in ("time", "t_pre", "step", "index", "gait_phase_sin"):
        expect(lambda b=bad: OBS.select_columns(names, pre_cycle=True, extra_columns=[b]), R.F2RContractError, f"requesting {bad} raises")
    expect(lambda: OBS.select_columns(names, pre_cycle=True, extra_columns=["online_left_gait_phase_sin"]), R.F2RContractError, "requesting the online clock in pre_cycle raises")
    check(OBS.select_columns(names, pre_cycle=False, extra_columns=["online_left_gait_phase_sin"]) == post, "allowed extra column is a no-op")
    expect(lambda: OBS.select_columns(names, pre_cycle=True, extra_columns=["nonexistent_feature"]), R.F2RContractError, "unknown extra column raises")
    expect(lambda: OBS.select_columns(names[::-1], pre_cycle=True), R.F2RContractError, "names must be the pinned manifest order")
    expect(lambda: OBS.assert_columns([0, 2, 3]), R.F2RContractError, "assert_columns refuses the prescribed clock column")
    check(OBS.assert_columns(pre)[2] is True and OBS.assert_columns(post)[2] is False, "assert_columns infers the variant")

    # --- folds -------------------------------------------------------------------------
    folds = OBS.build_folds()
    check(len(folds) == 3 and [f.held_out for f in folds] == list(R.STARTS), "3 LOSO folds")
    check(all(f.held_out not in f.train_starts and len(f.train_starts) == 2 and f.anchors == tuple(R.STARTS) for f in folds), "fold structure")
    expect(lambda: OBS.build_folds(("a", "a", "b")), R.F2RContractError, "build_folds needs 3 distinct starts")

    # --- synthetic worlds --------------------------------------------------------------
    jul, anchors = make_world(informative=True)
    check(OBS.first_cycle_step_from_X(jul["nominal"].X) == FIRST_CYCLE, "first_cycle_step from the online triplet")
    X_never = jul["nominal"].X.copy()
    X_never[:, 14], X_never[:, 15], X_never[:, 16] = 0.0, 1.0, 0.0
    check(OBS.first_cycle_step_from_X(X_never) == N_ROWS, "first_cycle_step = N when the triplet never activates")
    fold0 = folds[0]
    r = OBS.run_fold(fold0, jul, anchors, columns=pre, estimator="ridge", protocol_p0=p0)
    check(r["n_train"] == 5 * N_ROWS - 1 and r["n_test"] == N_ROWS and r["n_train_jul"] == 2 * N_ROWS and r["n_train_anchor"] == 3 * N_ROWS - 1, "fold sizes (amendment B: one same-start anchor reset row out of train; test full)")
    check(r["metrics"]["pre"]["n"] == FIRST_CYCLE and r["metrics"]["post"]["n"] == N_ROWS - FIRST_CYCLE and r["metrics"]["all"]["n"] == N_ROWS, "pre/post/all row counts")
    check(r["metrics"]["all"]["median"] < 0.02 and r["metrics"]["all"]["p90"] < 0.05, f"ridge pre_cycle recovers the phase (median {r['metrics']['all']['median']:.4f})")
    # standardisation stats from train only
    Xtr = np.concatenate([jul[s].X for s in fold0.train_starts] + [anchors[a].X[1:] if a == fold0.held_out else anchors[a].X for a in fold0.anchors], axis=0)
    check(np.array_equal(np.asarray(r["standardisation"]["mean"]), Xtr[:, pre].mean(axis=0)) and np.array_equal(np.asarray(r["standardisation"]["std"]), Xtr[:, pre].std(axis=0)), "standardisation mean/std == train-only statistics")
    Xall = np.concatenate([Xtr, jul[fold0.held_out].X], axis=0)
    check(not np.allclose(np.asarray(r["standardisation"]["mean"]), Xall[:, pre].mean(axis=0)), "standardisation mean != train+test mean")
    check(r["standardisation"]["fitted_on"] == "train_only" and r["standardisation"]["constant_columns"] == [], "no constant columns in the pre variant")
    r_post = OBS.run_fold(fold0, jul, anchors, columns=post, estimator="ridge", protocol_p0=p0)
    check(r_post["variant"] == "post_cycle" and r["variant"] == "pre_cycle" and len(r_post["columns"]) == 33, "variant inferred from the columns")
    check(all(n not in R.FORBIDDEN_P0_INPUT_NAMES for n in r["column_names"] + r_post["column_names"]), "no forbidden names in the used columns")
    # --- amendment B: same-start anchor reset row excluded from TRAIN (test unchanged) -------------
    rr = r["reset_row_rule"]
    check(rr["test_rows_excluded"] == 0 and rr["test_rows"] == N_ROWS and r["n_test"] == N_ROWS, "test set unchanged (reset row kept in test)")
    check(rr["train_anchor_reset_rows_excluded"] == 1 and rr["excluded"] == {"set": "train", "source": f"anchor:{fold0.held_out}", "start": fold0.held_out, "index": 0, "t_pre": anchors[fold0.held_out].t_pre[0], "reset_time": anchors[fold0.held_out].reset_time, "reason": rr["excluded"]["reason"], "bitwise_identical_to_test_row0": True} and rr["excluded"]["t_pre"] == rr["excluded"]["reset_time"], "exactly one train row excluded: same-start anchor index 0, t_pre == reset time, bitwise identical to test row 0")
    check(rr["train_rows_before"] == 5 * N_ROWS and rr["train_rows_after"] == 5 * N_ROWS - 1 and r["n_train"] == 5 * N_ROWS - 1 and r["n_train_anchor"] == 3 * N_ROWS - 1 and rr["anchor_rows_retained"] == 3 * N_ROWS - 1, "train size before/after; all other anchor rows and the other two JUL starts retained")
    check(rr["collisions_before_rule"] == {"full_rows": 1, "selected_columns": 1} and rr["collisions_after_rule"] == {"full_rows": 0, "selected_columns": 0} and r["leakage_check"]["full_row_collisions"] == 0, "collision counts: exactly one before (the reset duplicate), zero after; strict post-filter assert passed")
    parts, prov = OBS.apply_reset_row_rule(fold0, jul, anchors, cols=pre)
    check([n for n, _, _ in parts] == [f"jul:{x}" for x in fold0.train_starts] + [f"anchor:{a}[1:]" if a == fold0.held_out else f"anchor:{a}" for a in fold0.anchors] and prov == rr, "apply_reset_row_rule (no fit) reproduces the provenance and the train parts")
    # negatives: structural identification, never a duplicate whitelist
    def with_anchor(start, **kw):
        a = anchors[start]
        base = dict(start=a.start, X=a.X.copy(), phi=a.phi.copy(), first_cycle_step=a.first_cycle_step, source=a.source, t_pre=a.t_pre.copy(), reset_time=a.reset_time)
        base.update(kw)
        return {**anchors, start: OBS.StartData(**base)}
    def with_jul(start, **kw):
        j = jul[start]
        base = dict(start=j.start, X=j.X.copy(), phi=j.phi.copy(), first_cycle_step=j.first_cycle_step, source=j.source, t_pre=j.t_pre.copy(), reset_time=j.reset_time)
        base.update(kw)
        return {**jul, start: OBS.StartData(**base)}
    h = fold0.held_out
    Xm = anchors[h].X.copy(); Xm[0, 7] += 1e-9
    expect(lambda: OBS.apply_reset_row_rule(fold0, jul, with_anchor(h, X=Xm), cols=pre), R.F2RContractError, "missing expected reset duplicate (anchor row 0 differs by 1e-9) -> fail closed")
    Xa = anchors[h].X.copy(); Xa[10] = Xa[0]
    expect(lambda: OBS.apply_reset_row_rule(fold0, jul, with_anchor(h, X=Xa), cols=pre), R.F2RContractError, "ambiguous: anchor reset row occurs twice within the anchor -> fail closed")
    Xj = jul[h].X.copy(); Xj[12] = Xj[0]
    expect(lambda: OBS.apply_reset_row_rule(fold0, with_jul(h, X=Xj), anchors, cols=pre), R.F2RContractError, "ambiguous: held-out reset row occurs twice within the test set -> fail closed")
    Xn = anchors[h].X.copy(); Xn[5] = jul[h].X[9]
    expect(lambda: OBS.apply_reset_row_rule(fold0, jul, with_anchor(h, X=Xn), cols=pre), R.F2RContractError, "additional non-reset duplicate (held-out row 9 inside the anchor) -> fail closed")
    other = [a for a in R.STARTS if a != h][0]
    Xo = anchors[other].X.copy(); Xo[3] = jul[h].X[0]
    expect(lambda: OBS.apply_reset_row_rule(fold0, jul, with_anchor(other, X=Xo), cols=pre), R.F2RContractError, "reset row duplicated in ANOTHER anchor (not the same-start one) -> fail closed")
    expect(lambda: OBS.apply_reset_row_rule(fold0, jul, with_anchor(h, reset_time=anchors[h].reset_time + 0.01, t_pre=anchors[h].t_pre + 0.01), cols=pre), R.F2RContractError, "anchor reset time != held-out reset time -> fail closed")
    expect(lambda: OBS.apply_reset_row_rule(fold0, jul, with_anchor(h, t_pre=None, reset_time=None), cols=pre), R.F2RContractError, "anchor without time provenance -> fail closed")
    expect(lambda: OBS.apply_reset_row_rule(fold0, with_jul(h, t_pre=None, reset_time=None), anchors, cols=pre), R.F2RContractError, "held-out test without time provenance -> fail closed")
    expect(lambda: OBS.StartData(start=h, X=jul[h].X, phi=jul[h].phi, first_cycle_step=FIRST_CYCLE, source="x", t_pre=jul[h].t_pre, reset_time=jul[h].reset_time - 1e-6), R.F2RContractError, "StartData refuses t_pre[0] != reset_time")
    no_anchor_fold = OBS.FoldSpec(held_out=h, train_starts=fold0.train_starts, anchors=tuple(a for a in R.STARTS if a != h))
    expect(lambda: OBS.apply_reset_row_rule(no_anchor_fold, jul, anchors, cols=pre), R.F2RContractError, "fold without the same-start anchor -> fail closed (protocol keeps all 3 anchors)")
    # leakage asserts
    bad_fold = OBS.FoldSpec(held_out="nominal", train_starts=("nominal", "plus020"), anchors=tuple(R.STARTS))
    expect(lambda: OBS.run_fold(bad_fold, jul, anchors, columns=pre, estimator="ridge", protocol_p0=p0), R.F2RContractError, "held-out start in train_starts raises")
    leaky = dict(anchors)
    inj = anchors["plus020"]
    leaky["plus020"] = OBS.StartData(start="plus020", X=np.concatenate([inj.X, jul["minus020"].X[:7]], axis=0), phi=np.concatenate([inj.phi, jul["minus020"].phi[:7]]), first_cycle_step=inj.first_cycle_step, source="tampered")
    expect(lambda: OBS.run_fold(folds[0], jul, leaky, columns=pre, estimator="ridge", protocol_p0=p0), R.F2RContractError, "held-out rows injected into an anchor raise (row-hash)")
    leaky_jul = dict(jul)
    leaky_jul["plus020"] = OBS.StartData(start="plus020", X=np.concatenate([jul["plus020"].X, jul["minus020"].X[:1]], axis=0), phi=np.concatenate([jul["plus020"].phi, jul["minus020"].phi[:1]]), first_cycle_step=FIRST_CYCLE, source="tampered")
    expect(lambda: OBS.run_fold(folds[0], leaky_jul, anchors, columns=pre, estimator="ridge", protocol_p0=p0), R.F2RContractError, "a single held-out row in another JUL start raises")
    expect(lambda: OBS.run_fold(folds[0], jul, anchors, columns=[0] + pre, estimator="ridge", protocol_p0=p0), R.F2RContractError, "run_fold refuses the prescribed clock column")
    expect(lambda: OBS.run_fold(folds[0], jul, anchors, columns=pre, estimator="knn", protocol_p0=p0), R.F2RContractError, "unknown estimator")
    expect(lambda: OBS.run_fold(folds[0], {k: v for k, v in jul.items() if k != "plus020"}, anchors, columns=pre, estimator="ridge", protocol_p0=p0), R.F2RContractError, "missing JUL start")
    # estimators direct
    Z = np.random.default_rng(3).standard_normal((50, 4))
    Wtrue = np.array([[1.0, -2.0], [0.5, 0.25], [0.0, 1.0], [-1.0, 0.0]])
    Y = Z @ Wtrue + np.array([0.3, -0.1])
    pr = OBS.fit_ridge(Z, Y, 0.0)
    check(np.allclose(pr(Z), Y, atol=1e-9) and np.allclose(pr.weights[:-1], Wtrue, atol=1e-9), "ridge lambda 0 = OLS with intercept")
    pr2 = OBS.fit_ridge(Z, Y, 5.0)
    check(np.linalg.norm(pr2.weights[:-1]) < np.linalg.norm(pr.weights[:-1]), "ridge shrinks the weights")
    expect(lambda: OBS.fit_ridge(Z, Y[:, :1], 0.01), R.F2RContractError, "ridge needs (sin, cos) targets")
    mh = dict(p0["estimators"]["mlp"])
    mh["epochs"] = 5
    pm1 = OBS.fit_mlp(Z, Y, mh, 2026)
    pm2 = OBS.fit_mlp(Z, Y, mh, 2026)
    check(np.array_equal(pm1(Z), pm2(Z)) and pm1(Z).shape == (50, 2), "fit_mlp deterministic (bitwise) under the same seed")
    check(pm1.hparams["hidden"] == [64, 64] and pm1.hparams["batch_size"] == OBS.MLP_BATCH_SIZE_DEFAULT and pm1.hparams["seed"] == 2026, "mlp hparams recorded")
    expect(lambda: OBS.fit_mlp(Z, Y, {**mh, "activation": "relu"}, 2026), R.F2RContractError, "MLP activation frozen to tanh")

    # --- evaluate_p0: informative world -> PASS ----------------------------------------------
    res = OBS.evaluate_p0(jul, anchors, protocol, real=False)
    check(res["executed_on_real_data"] is False and res["protocol_id"] == R.PROTOCOL_ID, "executed_on_real_data flag + protocol id")
    check(res["pass"] is True and res["n_pass"] == 3, f"informative world PASS (mlp pre_cycle per fold {res['per_fold_pass']})")
    check(all(OBS.fold_passes(r["metrics"]["all"], res["thresholds"]) for r in res["results"]["pre_cycle"]["ridge"]), "ridge pre_cycle passes too")
    check(all(r["metrics"]["all"]["median"] < 0.02 for r in res["results"]["pre_cycle"]["mlp"]), "mlp pre_cycle median small")
    check(res["fold_plus020_reported"] and res["fold_plus020"]["pre_cycle"]["mlp"]["all"]["median"] is not None, "fold +0.20 reported")
    check(set(res["results"]) == {"pre_cycle", "post_cycle"} and all(set(v) == {"ridge", "mlp"} and all(len(x) == 3 for x in v.values()) for v in res["results"].values()), "3 folds x 2 variants x 2 estimators")
    check(res["rule"] == {"estimator": "mlp", "variant": "pre_cycle", "rows": "all", "min_folds_pass": 2, "of_folds": 3, "median_max": 0.05, "p90_max": 0.2}, "frozen rule recorded")
    check(res["amendment_B_reset_row"]["train_anchor_reset_rows_excluded_per_fold"] == 1 and res["amendment_B_reset_row"]["test_rows_excluded"] == 0 and set(res["reset_row_rule_per_fold"]) == set(R.STARTS) and all(v["train_anchor_reset_rows_excluded"] == 1 and v["test_rows_excluded"] == 0 and v["collisions_after_rule"] == {"full_rows": 0, "selected_columns": 0} for v in res["reset_row_rule_per_fold"].values()), "evaluate_p0 records amendment B and the per-fold reset-row provenance (consistent across variants/estimators)")
    check(all(n not in R.FORBIDDEN_P0_INPUT_NAMES for v in res["column_names"].values() for n in v), "no time/index names anywhere")
    check(all(r["standardisation"]["fitted_on"] == "train_only" and r["leakage_check"]["full_row_collisions"] == 0 for v in res["results"].values() for e in v.values() for r in e), "every fold: train-only standardisation, zero collisions")
    json.dumps(res)  # JSON-serialisable
    check(True, "result JSON-serialisable")
    table = OBS.decision_table(res)
    check("P0 PASS" in table and "plus020" in table, "decision table")

    # --- noise world -> FAIL ----------------------------------------------------------------
    jul_n, anchors_n = make_world(informative=False, seed=1)
    res_n = OBS.evaluate_p0(jul_n, anchors_n, protocol, real=False)
    check(res_n["pass"] is False and res_n["n_pass"] == 0, f"noise world FAIL (per fold {res_n['per_fold_pass']})")
    check(all(r["metrics"]["all"]["median"] > 0.15 for r in res_n["results"]["pre_cycle"]["mlp"] + res_n["results"]["pre_cycle"]["ridge"]), "noise world: phase not recoverable pre_cycle")
    check(all(r["metrics"]["post"]["median"] < 0.05 for r in res_n["results"]["post_cycle"]["ridge"]), "noise world: post_cycle variant recovers the phase after the first cycle from the online triplet")
    check("P0 FAIL" in OBS.decision_table(res_n), "decision table FAIL")

    # --- StartData validation -------------------------------------------------------------
    expect(lambda: OBS.StartData(start="nominal", X=np.zeros((4, 34)), phi=np.zeros(4), first_cycle_step=0, source="x"), R.F2RContractError, "StartData width")
    expect(lambda: OBS.StartData(start="nominal", X=np.zeros((4, 35)), phi=np.ones(4), first_cycle_step=0, source="x"), R.F2RContractError, "StartData phi in [0,1)")
    expect(lambda: OBS.StartData(start="nominal", X=np.zeros((4, 35)), phi=np.zeros(4), first_cycle_step=5, source="x"), R.F2RContractError, "StartData first_cycle_step range")
    expect(lambda: OBS.evaluate_p0({k: v for k, v in jul.items() if k != "nominal"}, anchors, protocol), R.F2RContractError, "evaluate_p0 missing start")

    # --- read-only loader on a synthetic job dir -----------------------------------------------
    tmp = R.portable_tempdir("f2r_obs_")
    job = tmp / "FAKE__v3_canonical__nominal__det"
    reset_time = 11.99 + R.EXACT_STARTS["nominal"]
    write_fake_job(job, steps=40, start="nominal", reset_time=reset_time, first_cycle=25)
    sd = OBS.load_start_data(job, HS, source="synthetic_job")
    check(sd.start == "nominal" and sd.n == 40 and sd.first_cycle_step == 25 and sd.source == "synthetic_job", "load_start_data parses start/rows/first cycle")
    t_pre = OA.t_pre_from_trace(reset_time, [reset_time + 0.01 * (k + 1) for k in range(40)])
    check(np.array_equal(sd.phi, OBS.prescribed_phase_array(t_pre, HS)) and sd.phi[0] == OBS.prescribed_phase(reset_time, HS), "phi = prescribed phase at t_pre (reset time first)")
    check(sd.X.shape == (40, 35) and sd.X.dtype == np.float64, "X rows float64")
    expect(lambda: OBS.load_start_data(job, HS, source="s", start="plus020"), R.F2RContractError, "declared start must match the summary")
    expect(lambda: OBS.load_start_data(tmp / "missing", HS, source="s"), R.F2RContractError, "missing job dir")
    # --- real P0 entry point: fail-closed on the pin verdict BEFORE any trace is read (P0 itself is NOT executed)
    calls = {"load": 0}

    def never_load(*a, **k):
        calls["load"] += 1
        raise AssertionError("load_start_data must not be called when the pins do not match")

    p0_out = tmp / "p0_refused"
    expect(lambda: OBS.run_p0_real(p0_out, verify_pins=lambda: {"all_match": False, "nominal": {"all_match": False}}, load=never_load), R.F2RContractError, "run_p0_real aborts when verify_anchor_pins()['all_match'] is False")
    expect(lambda: OBS.run_p0_real(p0_out, verify_pins=lambda: {"nominal": {}}, load=never_load), R.F2RContractError, "run_p0_real aborts when all_match is missing (not a bare call)")
    expect(lambda: OBS.run_p0_real(p0_out, verify_pins=lambda: None, load=never_load), R.F2RContractError, "run_p0_real aborts on a non-mapping verdict")
    check(calls["load"] == 0 and not p0_out.exists(), "no trace read and nothing written after the anchor-pin abort (P0 not executed)")
    ok_anchor = lambda: {"all_match": True, **{s: {k: {"disk": "d" * 64, "pinned": "d" * 64, "match": True} for k in R.ANCHOR_FILES} for s in R.STARTS}}  # noqa: E731
    expect(lambda: OBS.run_p0_real(p0_out, verify_pins=ok_anchor, verify_jul=lambda: {"all_match": False, "nominal": {"problems": ["trace digest mismatch"]}}, load=never_load), R.F2RContractError, "anchors ok but JUL_H0 job pins failing -> abort before any load")
    expect(lambda: OBS.run_p0_real(p0_out, verify_pins=ok_anchor, verify_jul=lambda: {"minus020": {}}, load=never_load), R.F2RContractError, "JUL verdict without all_match -> abort")
    expect(lambda: OBS.run_p0_real(p0_out, verify_pins=ok_anchor, verify_jul=lambda: None, load=never_load), R.F2RContractError, "JUL verdict not a mapping -> abort")
    check(calls["load"] == 0 and not p0_out.exists(), "loader never called and nothing written after the JUL-pin aborts")
    # real preflight (anchors + JUL_H0 jobs + F0 chain) passes and the loader is the NEXT step — P0 itself is not executed (loader raises)
    try:
        OBS.run_p0_real(p0_out, load=never_load)
        raise AssertionError("never_load must have been reached")
    except AssertionError as exc:
        check("must not be called" in str(exc) and calls["load"] == 1 and not p0_out.exists(), "real pins: preflight passes, the (spied) loader is the next call, nothing written, P0 not executed")
    pre = OBS.p0_input_preflight()
    check(pre["verified"] is True and pre["anchors"]["all_match"] and pre["jul_h0_p0_jobs"]["all_match"] and all(len(pre["anchors"]["digests"][s]) == 6 and all(len(x) == 64 for x in pre["anchors"]["digests"][s].values()) for s in R.STARTS) and all(set(pre["jul_h0_p0_jobs"]["digests"][s]) == {"summary", "trace", "reset", "receipt"} and all(len(x) == 64 for x in pre["jul_h0_p0_jobs"]["digests"][s].values()) for s in R.STARTS), "preflight record: 6 anchor digests + 4 JUL digests per start, auditable in the P0 provenance")
    check(pre["jul_h0_p0_jobs"]["digests"]["nominal"]["trace"] == R.P0_JUL_PINS["nominal"]["trace_sha256"] and pre["jul_h0_p0_jobs"]["f0_analysis"]["ok"] and pre["jul_h0_p0_jobs"]["job_ids"]["nominal"] == "JUL_H0__v3_canonical__nominal__det", "preflight digests equal the pins; F0 analysis chain ok")
    expect(lambda: OBS.main(["--real", "--out-dir", str(tmp / "p0_s0")]), SystemExit, "CLI --real refused without --authorized-stage S1")
    check(not (tmp / "p0_s0").exists(), "refused CLI wrote nothing")
    # --- real data, read-only: the reset-row rule identifies exactly the reset duplicate in every fold (no fit, no P0)
    hs_real = OBS.prescribed_heel_strikes_from_anchor()
    jul_real = {s: OBS.load_start_data(p, hs_real, source="jul_det_v3", start=s) for s, p in R.P0_JUL_JOBS.items()}
    anc_real = {s: OBS.load_start_data(a["job_dir"], hs_real, source="anchor_aiso6clk", start=s) for s, a in R.ANCHORS.items()}
    expected_rows = {"minus020": 500, "nominal": 500, "plus020": 176}
    for f in OBS.build_folds():
        parts_r, prov_r = OBS.apply_reset_row_rule(f, jul_real, anc_real, cols=OBS.select_columns(names, pre_cycle=True))
        other_rows = sum(expected_rows[x] for x in f.train_starts)
        check(prov_r["test_rows"] == expected_rows[f.held_out] and prov_r["test_rows_excluded"] == 0 and prov_r["train_anchor_reset_rows_excluded"] == 1 and prov_r["anchor_rows_retained"] == 1499 and prov_r["train_rows_before"] == other_rows + 1500 and prov_r["train_rows_after"] == other_rows + 1499, f"real {f.held_out}: test {expected_rows[f.held_out]} rows kept, train = other JUL {other_rows} + 1499 anchor rows")
        check(prov_r["collisions_before_rule"]["full_rows"] == 1 and prov_r["collisions_after_rule"] == {"full_rows": 0, "selected_columns": 0} and prov_r["excluded"]["index"] == 0 and prov_r["excluded"]["t_pre"] == prov_r["excluded"]["reset_time"] == jul_real[f.held_out].reset_time and prov_r["excluded"]["bitwise_identical_to_test_row0"], f"real {f.held_out}: exactly the reset duplicate (t_pre == reset time {prov_r['excluded']['reset_time']:.6f}) excluded; zero collisions after")
    elapsed = time.time() - t0
    print(f"SELFTEST PASS ({CHECKS} checks) [{elapsed:.1f}s]")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
