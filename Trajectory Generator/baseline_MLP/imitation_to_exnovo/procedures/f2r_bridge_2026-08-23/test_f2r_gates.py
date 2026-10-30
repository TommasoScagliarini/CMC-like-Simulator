"""Self-test of f2r_gates.

(a) REAL, READ-ONLY: ``extract_record`` on the pinned A_iso6clk nominal anchor
    (F1 ``aiso_clk_diag``), values checked against the preregistered ranges, then
    gates A/B/C on that record replicated over the 3 starts (T1/T2 prediction:
    A PASS, B2/B3/B5 FAIL, C PASS -> T3 trigger True);
(b) synthetic records: every aggregation rule, None handling, gates V and R,
    ``evaluate_round`` promote / not-promoted paths, thresholds read from the
    protocol;
(c) a synthetic job directory (temp only) exercising the STO fallback route and
    the phase-aligned metrics on analytically known series.

No rollout, no env, no write outside a temp dir.  Final line: SELFTEST PASS (N checks).
"""

from __future__ import annotations

import copy
import json
import math
import shutil
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import f2r_common as R  # noqa: E402
import f0_common as C  # noqa: E402
import f1_obs_adapter as OA  # noqa: E402
import f1_dataset as DS  # noqa: E402
import f2r_gates as G  # noqa: E402
from test_f1_obs_adapter import FakeClock  # noqa: E402  (F1 test helper, read-only import)

CHECKS = 0
STARTS = R.STARTS


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


def approx(value, target, tol) -> bool:
    return value is not None and math.isfinite(float(value)) and abs(float(value) - float(target)) <= tol


def within(value, lo, hi) -> bool:
    return value is not None and math.isfinite(float(value)) and lo <= float(value) <= hi


# --- synthetic records ----------------------------------------------------------------------


def rec(start: str, **over):
    base = dict(
        job_id=f"SYN__{start}__det", start=start, seed=R.DET_SEED, action_selection="deterministic", steps=500, end_reason="episode_time_limit",
        horizon_completed=True, phase_timeout=False, valid_cycles=3, valid_hs=4, valid_to=3, invalid_events=0, hs_cancelled_count=0, resync_count=0,
        first_hs_delay_s=1.0, online_clock_informative_s=2.5, online_periods_s=[1.50, 1.55], drift_cycle_per_s=0.01,
        knee_rom=0.85, knee_min=-1.0, knee_max=-0.15, ankle_min=-0.10, ankle_max=0.25,
        penetration_max_m=0.015, reserve_norm_max_nm=900.0, action_abs_max_raw=1.1, action_clipped_steps=5,
        stance_fraction=0.60, knee_rmse=0.12, knee_r=0.90, ankle_rmse=0.20, ankle_min_window=-0.05, settled_fraction=0.80, dropped_fraction=0.10,
    )
    base.update(over)
    return base


def det3(**over_by_start):
    return {s: rec(s, **over_by_start.get(s, {})) for s in STARTS}


def v3(**over_by_start):
    return {s: rec(s, **{"seed": R.VALIDATION_SEED, "action_selection": "stochastic", "job_id": f"SYN__{s}__stoch_seed125", **over_by_start.get(s, {})}) for s in STARTS}


def r9(over=None):
    over = over or {}
    out = []
    for s in STARTS:
        for k in (123, 124, 125):
            out.append(rec(s, **{"seed": k, "action_selection": "stochastic", "job_id": f"SYN__{s}__stoch_seed{k}", **over.get((s, k), {})}))
    return out


def flags(*pairs):
    return {s: v for s, v in zip(STARTS, pairs)}


# --- (b) pure gate logic -------------------------------------------------------------------------


def test_rules_and_thresholds(thr):
    check(G.apply_rule(flags(True, True, True), "all3") is True, "all3 pass")
    check(G.apply_rule(flags(True, False, True), "all3") is False, "all3 one False fails")
    check(G.apply_rule(flags(True, False, True), "2of3") is True, "2of3 with two True passes")
    check(G.apply_rule(flags(False, False, True), "2of3") is False, "2of3 with one True fails")
    check(G.apply_rule(flags((True, True), (True, True), (True, False)), "all3_and_2of3") is True, "A2 rule: all >=1 and two >=2")
    check(G.apply_rule(flags((True, True), (True, False), (True, False)), "all3_and_2of3") is False, "A2 rule: only one start with >=2 fails")
    check(G.apply_rule(flags((False, True), (True, True), (True, True)), "all3_and_2of3") is False, "A2 rule: a start without cycles fails")
    expect(lambda: G.apply_rule(flags(True, True, True), "majority"), R.F2RContractError, "unknown rule refused")
    expect(lambda: G.apply_rule({"nominal": True, "plus020": True}, "all3"), R.F2RContractError, "missing start refused")
    expect(lambda: G.apply_rule(flags(True, None, True), "all3"), R.F2RContractError, "non-bool flag refused")
    expect(lambda: G.apply_rule(flags(True, True, True), "all3_and_2of3"), R.F2RContractError, "A2 rule needs (bool, bool) pairs")
    check(thr["B"]["B3_ankle_negative_window"]["min_max_rad"] == -0.03, "B3 threshold read from the protocol (-0.03)")
    check(thr == R.load_protocol()["gates"], "gate_thresholds returns protocol['gates'] unchanged")
    check(thr["A"]["A2_cycles"]["rule"] == "all3_and_2of3" and thr["B"]["B2_knee_vs_corridor"]["rule"] == "2of3" and thr["C"]["C3_reserve_norm_max_nm_per_start"]["rule"] == "all3", "rules read from the protocol")
    bad = copy.deepcopy(R.load_protocol())
    del bad["gates"]["B"]["B6_corridor_coverage"]
    expect(lambda: G.gate_thresholds(bad), R.F2RContractError, "protocol/code check-name drift refused")
    bad = copy.deepcopy(R.load_protocol())
    bad["gates"]["V"]["seed"] = 124
    expect(lambda: G.gate_thresholds(bad), R.F2RContractError, "V seed other than the held-out 125 refused")
    bad = copy.deepcopy(R.load_protocol())
    bad["gates"]["A"]["A1_first_hs_delay_s"]["rule"] = "best_of_3"
    expect(lambda: G.gate_thresholds(bad), R.F2RContractError, "unknown rule in the protocol refused")
    check(G.rule_fraction(["horizon >= 8/9", ">= 1 valid cycle in >= 8/9"], "horizon", jobs=9) == 8, "rule fraction parsed from the protocol string")
    check(G.rule_fraction(thr["V"]["rules"], "horizon", jobs=3) == 3 and G.rule_fraction(thr["R"]["rules"], "valid cycle", jobs=9) == 8, "V 3/3 and R 8/9 parsed")
    expect(lambda: G.rule_fraction(["horizon >= 8/9"], "horizon", jobs=6), R.F2RContractError, "fraction denominator must equal the job count")
    expect(lambda: G.rule_fraction(["horizon always"], "horizon", jobs=9), R.F2RContractError, "rule without fraction refused")
    expect(lambda: G.rule_fraction(["horizon 8/9", "horizon 7/9"], "horizon", jobs=9), R.F2RContractError, "ambiguous rule keyword refused")


def test_gate_A(thr):
    base = G.gate_A(det3(), thr)
    check(base["pass"] is True and all(c["pass"] for c in base["checks"].values()) and tuple(base["checks"]) == G.A_CHECKS and base["failed_checks"] == [], "gate A all-pass synthetic")
    c = base["checks"]["A1_first_hs_delay_s"]
    check(set(c) >= {"pass", "rule", "values_by_start", "threshold"} and c["threshold"] == thr["A"]["A1_first_hs_delay_s"] and c["values_by_start"]["nominal"] == 1.0, "check structure carries rule/values/threshold")
    g = G.gate_A(det3(nominal={"first_hs_delay_s": None}), thr)
    check(g["pass"] is False and g["failed_checks"] == ["A1_first_hs_delay_s"], "A1 None fails")
    check(G.gate_A(det3(plus020={"first_hs_delay_s": 2.5}), thr)["checks"]["A1_first_hs_delay_s"]["pass"] is False, "A1 above max fails")
    check(G.gate_A(det3(plus020={"first_hs_delay_s": 0.1}), thr)["checks"]["A1_first_hs_delay_s"]["pass"] is False, "A1 below min fails")
    check(G.gate_A(det3(plus020={"first_hs_delay_s": 0.15}), thr)["checks"]["A1_first_hs_delay_s"]["pass"] is True, "A1 boundary inclusive")
    a2 = lambda m, n, p: G.gate_A(det3(minus020={"valid_cycles": m}, nominal={"valid_cycles": n}, plus020={"valid_cycles": p}), thr)["checks"]["A2_cycles"]["pass"]  # noqa: E731
    check(a2(1, 1, 2) is False and a2(1, 2, 2) is True and a2(0, 3, 3) is False and a2(2, 2, 2) is True, "A2 all3_and_2of3 semantics")
    check(G.gate_A(det3(nominal={"resync_count": 2}), thr)["checks"]["A3_hs_cancelled_max"]["pass"] is False, "A3 resync 2 fails")
    check(G.gate_A(det3(nominal={"resync_count": 1}), thr)["checks"]["A3_hs_cancelled_max"]["pass"] is True, "A3 resync 1 passes")
    check(G.gate_A(det3(nominal={"hs_cancelled_count": 1}), thr)["checks"]["A3_hs_cancelled_max"]["pass"] is False, "A3 one cancellation fails")
    check(G.gate_A(det3(nominal={"hs_cancelled_count": None}), thr)["checks"]["A3_hs_cancelled_max"]["pass"] is False, "A3 absent counter fails")
    check(G.gate_A(det3(nominal={"steps": 499, "horizon_completed": False}), thr)["checks"]["A4_horizon"]["pass"] is False, "A4 499 steps fails")
    check(G.gate_A(det3(nominal={"end_reason": "grf_penetration", "horizon_completed": False}), thr)["checks"]["A4_horizon"]["pass"] is False, "A4 grf_penetration fails")
    check(G.gate_A(det3(nominal={"phase_timeout": True}), thr)["checks"]["A4_horizon"]["pass"] is False, "A4 phase timeout fails")
    check(G.gate_A(det3(nominal={"online_clock_informative_s": None}), thr)["checks"]["A5_online_clock_informative_within_s"]["pass"] is False, "A5 never informative fails")
    check(G.gate_A(det3(nominal={"online_clock_informative_s": 3.6}), thr)["checks"]["A5_online_clock_informative_within_s"]["pass"] is False, "A5 3.6 s fails")
    check(G.gate_A(det3(nominal={"online_clock_informative_s": 3.5}), thr)["checks"]["A5_online_clock_informative_within_s"]["pass"] is True, "A5 3.5 s passes")
    a6 = lambda **o: G.gate_A(det3(nominal=o), thr)["checks"]["A6_online_period_s"]["pass"]  # noqa: E731
    check(a6(online_periods_s=[]) is False and a6(online_periods_s=[1.8]) is False and a6(online_periods_s=[1.5, 1.71]) is False and a6(online_periods_s=[1.45, 1.70]) is True, "A6 period range")
    check(a6(drift_cycle_per_s=-0.06) is False and a6(drift_cycle_per_s=None) is False and a6(drift_cycle_per_s=-0.055) is True, "A6 drift bound (None fails)")
    check(G.gate_A(det3(nominal={"stance_fraction": None}), thr)["checks"]["A7_stance_fraction_min"]["pass"] is False, "A7 None fails")
    check(G.gate_A(det3(nominal={"stance_fraction": 0.29}), thr)["checks"]["A7_stance_fraction_min"]["pass"] is False and G.gate_A(det3(nominal={"stance_fraction": 0.30}), thr)["checks"]["A7_stance_fraction_min"]["pass"] is True, "A7 boundary")
    expect(lambda: G.gate_A(det3(nominal={"seed": 125}), thr), R.F2RContractError, "seed 125 record in the det gates refused")
    expect(lambda: G.gate_A(det3(nominal={"seed": 124}), thr), R.F2RContractError, "seed 124 record in the det gates refused")
    expect(lambda: G.gate_A(det3(nominal={"action_selection": "stochastic"}), thr), R.F2RContractError, "stochastic record in the det gates refused")
    expect(lambda: G.gate_A({s: r for s, r in det3().items() if s != "plus020"}, thr), R.F2RContractError, "missing start refused")
    expect(lambda: G.gate_A({**det3(), "extra": rec("nominal")}, thr), R.F2RContractError, "extra key refused")
    expect(lambda: G.gate_A({**det3(), "plus020": rec("nominal")}, thr), R.F2RContractError, "start/key mismatch refused")
    broken = det3()
    del broken["nominal"]["knee_rmse"]
    expect(lambda: G.gate_A(broken, thr), R.F2RContractError, "record missing a key refused")
    expect(lambda: G.gate_A([rec(s) for s in STARTS], thr), R.F2RContractError, "list instead of start-keyed mapping refused")


def test_gate_B(thr):
    base = G.gate_B(det3(), thr)
    check(base["pass"] is True and tuple(base["checks"]) == G.B_CHECKS, "gate B all-pass synthetic")
    check(G.gate_B(det3(minus020={"knee_rom": 0.59}), thr)["checks"]["B1_knee_rom_min_rad"]["pass"] is False, "B1 all3: one start below 0.60 fails")
    b2 = lambda **o: G.gate_B(det3(**o), thr)["checks"]["B2_knee_vs_corridor"]["pass"]  # noqa: E731
    check(b2(nominal={"knee_rmse": 0.17}) is True, "B2 2of3: one failing start passes")
    check(b2(nominal={"knee_rmse": 0.17}, plus020={"knee_r": 0.84}) is False, "B2 2of3: two failing starts fail")
    check(b2(nominal={"knee_r": None}, plus020={"knee_rmse": None}) is False, "B2 None at two starts fails")
    check(b2(nominal={"knee_r": None, "knee_rmse": None}) is True, "B2 None at one start still 2of3")
    b3 = lambda **o: G.gate_B(det3(**o), thr)["checks"]["B3_ankle_negative_window"]["pass"]  # noqa: E731
    check(b3(nominal={"ankle_min_window": -0.03}) is True, "B3 boundary -0.03 passes")
    check(b3(nominal={"ankle_min_window": -0.02}, plus020={"ankle_min_window": 0.1}) is False, "B3 two starts above -0.03 fail")
    check(b3(nominal={"ankle_min_window": None}, minus020={"ankle_min_window": None}) is False, "B3 None at two starts fails")
    check(G.gate_B(det3(nominal={"ankle_rmse": 0.26}, plus020={"ankle_rmse": 0.26}), thr)["checks"]["B4_ankle_rmse_max_rad"]["pass"] is False, "B4 two starts above 0.25 fail")
    b5 = lambda **o: G.gate_B(det3(**o), thr)["checks"]["B5_stance_fraction"]["pass"]  # noqa: E731
    check(b5(nominal={"stance_fraction": 0.45}, plus020={"stance_fraction": 0.45}) is False and b5(nominal={"stance_fraction": 0.76}, plus020={"stance_fraction": 0.76}) is False, "B5 outside [0.50, 0.75] fails")
    check(b5(nominal={"stance_fraction": 0.50}, plus020={"stance_fraction": 0.75}) is True, "B5 boundaries inclusive")
    check(b5(nominal={"stance_fraction": None}, plus020={"stance_fraction": None}) is False, "B5 None at two starts fails")
    b6 = lambda **o: G.gate_B(det3(**o), thr)["checks"]["B6_corridor_coverage"]["pass"]  # noqa: E731
    check(b6(nominal={"settled_fraction": 0.59}) is False and b6(nominal={"dropped_fraction": 0.41}) is False and b6(nominal={"settled_fraction": None}) is False, "B6 all3 coverage bounds (None fails)")
    check(b6(nominal={"settled_fraction": 0.60, "dropped_fraction": 0.40}) is True, "B6 boundaries inclusive")


def test_gate_C(thr):
    base = G.gate_C(det3(), thr)
    check(base["pass"] is True and tuple(base["checks"]) == G.C_CHECKS, "gate C all-pass synthetic")
    check(G.gate_C(det3(nominal={"penetration_max_m": 0.021}), thr)["checks"]["C1_penetration_max_m"]["pass"] is False, "C1 21 mm fails")
    check(G.gate_C(det3(nominal={"penetration_max_m": None}), thr)["checks"]["C1_penetration_max_m"]["pass"] is False, "C1 None fails")
    g = G.gate_C(det3(nominal={"end_reason": "grf_penetration", "horizon_completed": False}), thr)
    check(g["checks"]["C2_no_grf_penetration_termination"]["pass"] is False and g["pass"] is False, "C2 grf_penetration termination fails")
    check(G.gate_C(det3(nominal={"reserve_norm_max_nm": 1000.1}), thr)["checks"]["C3_reserve_norm_max_nm_per_start"]["pass"] is False and G.gate_C(det3(nominal={"reserve_norm_max_nm": 1000.0}), thr)["checks"]["C3_reserve_norm_max_nm_per_start"]["pass"] is True, "C3 per-start bound 1000 Nm")
    check(G.gate_C(det3(nominal={"action_clipped_steps": 31}), thr)["checks"]["C4_clipped_steps_max"]["pass"] is False and G.gate_C(det3(nominal={"action_clipped_steps": 30}), thr)["checks"]["C4_clipped_steps_max"]["pass"] is True, "C4 clipped steps bound 30")
    check(G.gate_C(det3(nominal={"action_abs_max_raw": 1.31}), thr)["checks"]["C5_action_abs_max"]["pass"] is False and G.gate_C(det3(nominal={"action_abs_max_raw": 1.3}), thr)["checks"]["C5_action_abs_max"]["pass"] is True, "C5 raw action bound 1.3")
    f = G.c123_flags(rec("nominal", penetration_max_m=0.02, reserve_norm_max_nm=1000.0), thr)
    check(all(f.values()) and set(f) == {"C1_penetration_max_m", "C2_no_grf_penetration_termination", "C3_reserve_norm_max_nm_per_start"}, "c123_flags boundaries")


def test_gate_V(thr):
    base = G.gate_V(v3(), thr)
    check(base["pass"] is True and base["seed"] == 125 and all(c["pass"] for c in base["checks"].values()), "gate V all-pass synthetic")
    g = G.gate_V(v3(nominal={"end_reason": "grf_penetration", "horizon_completed": False, "steps": 312}), thr)
    check(g["pass"] is False and g["checks"]["V_no_grf_penetration"]["pass"] is False and g["checks"]["V_horizon"]["pass"] is False and g["checks"]["V_C1_C3_each_job"]["pass"] is False, "V FAIL on a seed-125 grf_penetration job")
    check(G.gate_V(v3(plus020={"valid_cycles": 0}), thr)["pass"] is False, "V needs >= 1 valid cycle at each start")
    check(G.gate_V(v3(plus020={"horizon_completed": False, "steps": 499}), thr)["pass"] is False, "V needs horizon 3/3")
    check(G.gate_V(v3(minus020={"penetration_max_m": 0.021}), thr)["checks"]["V_C1_C3_each_job"]["pass"] is False, "V C1 per job")
    check(G.gate_V(v3(minus020={"reserve_norm_max_nm": 1001.0}), thr)["checks"]["V_C1_C3_each_job"]["pass"] is False, "V C3 per job")
    check(G.gate_V(v3(minus020={"knee_rmse": 0.5, "stance_fraction": None, "ankle_min_window": 0.3}), thr)["pass"] is True, "V does not evaluate B-type metrics")
    expect(lambda: G.gate_V(v3(nominal={"seed": 123}), thr), R.F2RContractError, "seed-123 record in gate V refused")
    expect(lambda: G.gate_V(v3(nominal={"action_selection": "deterministic"}), thr), R.F2RContractError, "deterministic record in gate V refused")
    expect(lambda: G.gate_V({s: r for s, r in v3().items() if s != "nominal"}, thr), R.F2RContractError, "gate V needs the 3 starts")


def test_gate_R(thr):
    base = G.gate_R(r9(), thr)
    check(base["pass"] is True and base["jobs"] == 9 and base["seeds"] == [123, 124, 125] and base["checks"]["R_horizon"]["count"] == 9, "gate R 9/9 synthetic")
    g = G.gate_R(r9({("nominal", 124): {"horizon_completed": False, "steps": 400}}), thr)
    check(g["pass"] is True and g["checks"]["R_horizon"]["count"] == 8, "gate R horizon 8/9 passes")
    g = G.gate_R(r9({("nominal", 124): {"horizon_completed": False, "steps": 400}, ("plus020", 125): {"horizon_completed": False, "steps": 100}}), thr)
    check(g["pass"] is False and g["checks"]["R_horizon"]["pass"] is False, "gate R horizon 7/9 fails")
    check(G.gate_R(r9({("minus020", 123): {"valid_cycles": 0}}), thr)["pass"] is True, "gate R valid cycle 8/9 passes")
    check(G.gate_R(r9({("minus020", 123): {"valid_cycles": 0}, ("minus020", 124): {"valid_cycles": 0}}), thr)["pass"] is False, "gate R valid cycle 7/9 fails")
    check(G.gate_R(r9({("plus020", 123): {"penetration_max_m": 0.0201}}), thr)["pass"] is False, "gate R C1 on each job")
    check(G.gate_R(r9({("plus020", 123): {"end_reason": "grf_penetration", "horizon_completed": False}}), thr)["checks"]["R_C1_C3_each_job"]["pass"] is False, "gate R C2 on each job")
    check(G.gate_R({r["job_id"]: r for r in r9()}, thr)["pass"] is True, "gate R accepts a mapping of records")
    expect(lambda: G.gate_R(r9()[:8], thr), R.F2RContractError, "gate R needs exactly 9 records")
    dup = r9()
    dup[1] = dict(dup[0])
    expect(lambda: G.gate_R(dup, thr), R.F2RContractError, "duplicate (start, seed) refused")
    sealed = r9()
    sealed[2]["seed"] = 126
    expect(lambda: G.gate_R(sealed, thr), R.F2RContractError, "sealed seed refused in gate R")
    det = r9()
    det[0]["action_selection"] = "deterministic"
    expect(lambda: G.gate_R(det, thr), R.F2RContractError, "deterministic record refused in gate R")


def test_evaluate_round(thr):
    protocol = R.load_protocol()
    res = G.evaluate_round(det3(), v3(), protocol)
    check(res["promotion"] is True and res["failed_gates"] == [] and res["next"] == G.NEXT_PROMOTE and res["t3_trigger"] is False and res["V"]["status"] == "EVALUATED" and res["protocol_id"] == R.PROTOCOL_ID, "evaluate_round promote path")
    check(all(res[g]["pass"] is True for g in ("A", "B", "C", "V")), "all gates reported on the promote path")
    res = G.evaluate_round(det3(nominal={"ankle_min_window": 0.1}, plus020={"ankle_min_window": 0.2}), v3(), protocol)
    check(res["promotion"] is False and res["failed_gates"] == ["B"] and res["next"] == G.NEXT_NOT_PROMOTED and res["t3_trigger"] is True and res["C"]["pass"] is True and res["V"]["pass"] is True, "evaluate_round not promoted on B3 (all gates still computed) -> T3 trigger")
    res = G.evaluate_round(det3(), None, protocol)
    check(res["promotion"] is False and res["failed_gates"] == ["V"] and res["V"]["status"] == "NOT_EVALUATED" and res["t3_trigger"] is False, "no seed-125 records -> V not evaluated -> not promoted")
    res = G.evaluate_round(det3(), v3(nominal={"end_reason": "grf_penetration", "horizon_completed": False}), protocol)
    check(res["promotion"] is False and res["failed_gates"] == ["V"] and res["next"] == G.NEXT_NOT_PROMOTED, "seed-125 grf_penetration -> V FAIL -> promotion False")
    res = G.evaluate_round(det3(nominal={"first_hs_delay_s": None, "ankle_min_window": 0.1}, plus020={"ankle_min_window": 0.2}), v3(), protocol)
    check(res["failed_gates"] == ["A", "B"] and res["t3_trigger"] is False, "T3 trigger requires gate A PASS")
    res = G.evaluate_round(det3(nominal={"stance_fraction": 0.45}, plus020={"stance_fraction": 0.45}), v3(), protocol)
    check(res["failed_gates"] == ["B"] and res["t3_trigger"] is True and G.t3_trigger(res) is True, "B5 FAIL with A PASS triggers T3")
    res = G.evaluate_round(det3(nominal={"knee_r": 0.5}, plus020={"knee_r": 0.5}), v3(), protocol)
    check(res["t3_trigger"] is True, "B2 FAIL with A PASS triggers T3")
    res = G.evaluate_round(det3(nominal={"knee_rom": 0.5}), v3(), protocol)
    check(res["failed_gates"] == ["B"] and res["t3_trigger"] is False, "B1-only failure does not trigger T3")
    check(G.evaluate_round(det3(), v3()) ["promotion"] is True, "protocol defaults to the frozen JSON")
    expect(lambda: G.t3_trigger({"A": {"pass": True}}), R.F2RContractError, "t3_trigger on a malformed round result refused")
    expect(lambda: G.evaluate_round(det3(), v3(nominal={"seed": 124}), protocol), R.F2RContractError, "seed-124 record offered as V refused")


# --- helpers on synthetic arrays ---------------------------------------------------------------------


def test_phase_helpers(corridor):
    hs = [10.0, 11.2, 12.5, 13.6, 15.0]
    clock = FakeClock(hs, offset=0.3)
    t = np.concatenate([np.linspace(8.0, 17.0, 451), np.asarray(hs)])
    mine = G.prescribed_phase(t, hs, offset=0.3)
    ref = np.asarray([clock.phase(x) for x in t])
    check(np.allclose(mine, ref, atol=1e-12) and np.all((mine >= 0) & (mine < 1)), "prescribed_phase == GaitPhaseClock.phase algebra (incl. extrapolation and offset)")
    check(np.allclose(G.prescribed_phase(t, hs), np.asarray([clock.raw_phase(x) for x in t]), atol=1e-12), "prescribed_phase offset 0 == raw phase")
    expect(lambda: G.prescribed_phase([1.0], [10.0]), R.F2RContractError, "prescribed clock needs >= 2 heel strikes")
    ph = np.linspace(0.0, 0.999, 500)
    check(np.allclose(G.online_phase(np.sin(2 * np.pi * ph), np.cos(2 * np.pi * ph)), ph, atol=1e-12), "online_phase inverts sin/cos")
    names = list(R.FEATURE_NAMES_35)
    check(G.online_clock_columns(names) == (14, 15, 16), "online clock columns derived from the manifest names")
    expect(lambda: G.online_clock_columns(names[1:] + names[:1]), R.F2RContractError, "names35 mismatch refused")
    n = 400
    reset = 20.0
    t_pre = reset + 0.01 * np.arange(n)
    obs = np.zeros((n, 35))
    obs[:, 1] = 1.0
    obs[:, 15] = 1.0
    first = 150
    slope, d0 = -0.03, 0.1
    pres = G.prescribed_phase(t_pre, hs_syn := [20.0, 21.5, 23.0, 24.5, 26.0])
    online = (pres[first:] - (d0 + slope * (t_pre[first:] - t_pre[first]))) % 1.0
    obs[first:, 14] = np.sin(2 * np.pi * online)
    obs[first:, 15] = np.cos(2 * np.pi * online)
    obs[first:, 16] = 1.5
    obs[300:310, 16] = 1.5000004  # sub-microsecond jitter must not create a new period
    diag = G.online_clock_diagnostics(obs, t_pre, reset)
    check(diag["informative_index"] == first and approx(diag["informative_s"], 1.5, 1e-12) and diag["periods_s"] == [1.5], "online clock onset + distinct periods (rounded)")
    dr = G.drift_cycle_per_s(obs, t_pre, hs_syn)
    check(approx(dr["slope"], slope, 1e-9) and dr["rows"] == n - first, "drift slope recovered from synthetic online clock")
    dead = np.zeros((n, 35))
    dead[:, 15] = 1.0
    check(G.online_clock_diagnostics(dead, t_pre, reset)["informative_s"] is None and G.drift_cycle_per_s(dead, t_pre, hs_syn)["slope"] is None, "never-informative clock -> None")
    check(G.drift_cycle_per_s(obs[: first + 1], t_pre[: first + 1], hs_syn)["slope"] is None, "single informative row -> drift None")
    sat = obs.copy()
    sat[200:260, 14] = 0.0
    sat[200:260, 15] = 1.0
    dr2 = G.drift_cycle_per_s(sat, t_pre, hs_syn)
    check(dr2["rows"] == n - first - 60 and approx(dr2["slope"], slope, 1e-9), "saturated (0,1) rows excluded from the drift fit")
    check(dr["estimator"] == "unwrapped_circular_difference" and dr["wrap_crossings"] == 0 and approx(dr["slope_wrapped_legacy"], slope, 1e-9), "amendment A6-unwrap: no crossing -> unwrapped == legacy slope, estimator recorded")
    # --- amendment A6-unwrap: offset that crosses +-0.5 cycle (physiological half-cycle lag) with a slow true drift
    for d0x, slx in ((-0.47, -0.03), (0.52, -0.03), (0.47, +0.03)):  # offsets that cross -0.5 / start beyond +0.5 / cross +0.5
        cross = np.zeros((n, 35)); cross[:, 1] = 1.0; cross[:, 15] = 1.0
        online_x = (pres[first:] - (d0x + slx * (t_pre[first:] - t_pre[first]))) % 1.0
        cross[first:, 14] = np.sin(2 * np.pi * online_x); cross[first:, 15] = np.cos(2 * np.pi * online_x); cross[first:, 16] = 1.5
        drx = G.drift_cycle_per_s(cross, t_pre, hs_syn)
        check(approx(drx["slope"], slx, 1e-9) and drx["wrap_crossings"] >= 1 and abs(drx["slope_wrapped_legacy"] - slx) > 0.1, f"offset {d0x:+.2f}, true drift {slx:+.2f}: crossing +-0.5 -> unwrapped slope {drx['slope']:+.4f} exact while the legacy wrapped slope ({drx['slope_wrapped_legacy']:+.3f}) is inflated")
    nanobs = obs.copy(); nanobs[200, 14] = np.nan
    check(G.drift_cycle_per_s(nanobs, t_pre, hs_syn)["slope"] is None, "non-finite phase -> drift None (fail-closed)")
    check(G.stance_fraction_of([]) is None, "stance fraction without cycles is None")
    cycles = [{"hs_start_s": 0.0, "toe_off_s": 0.6, "hs_end_s": 1.0, "duration_s": 1.0}, {"hs_start_s": 1.0, "toe_off_s": 1.5, "hs_end_s": 2.0, "duration_s": 1.0}, {"hs_start_s": 2.0, "toe_off_s": 2.7, "hs_end_s": 3.0, "duration_s": 1.0}]
    check(approx(G.stance_fraction_of(cycles), 0.6, 1e-12), "stance fraction = median over cycles")
    expect(lambda: G.stance_fraction_of([{"hs_start_s": 0.0, "toe_off_s": 1.2, "hs_end_s": 1.0, "duration_s": 1.0}]), R.F2RContractError, "malformed cycle refused")
    check(approx(corridor["alpha"], 0.6223, 1e-4) and corridor["profile"]["sha256"] == R.CORRIDOR_PROFILE["sha256"] and corridor["reference"]["pros_knee_angle"].shape == (101,), "corridor reference pinned, alpha 0.6223, 101-point grid")
    check(approx(float(np.min(corridor["reference"]["pros_ankle_angle"])), -0.311, 2e-3) and approx(float(np.min(corridor["reference"]["pros_knee_angle"])), -1.038, 2e-3), "corridor minima as frozen in the protocol")
    expect(lambda: G.corridor_reference(expected_sha256="0" * 64), R.F2RContractError, "corridor digest mismatch refused")
    check(G.first_accepted_event_time([{"phase_fsm": {"accepted_transitions_this_step": []}}, {"phase_fsm": {"accepted_transitions_this_step": [{"event": "heel_strike_cancelled", "event_time_s": 1.0}, {"event": "heel_strike", "event_time_s": 2.5}]}}], "heel_strike") == 2.5 and G.first_accepted_event_time([{"phase_fsm": {}}], "heel_strike") is None, "first accepted heel strike (cancellation ignored)")
    # phase-aligned metrics on a series identical to the corridor mean
    alpha = corridor["alpha"]
    t = 30.0 + 0.001 * np.arange(4000)
    cyc = [{"step": 1, "hs_start_s": 30.5, "toe_off_s": 31.4, "hs_end_s": 32.0, "duration_s": 1.5}, {"step": 2, "hs_start_s": 32.0, "toe_off_s": 32.9, "hs_end_s": 33.5, "duration_s": 1.5}]
    ph = np.zeros_like(t)
    for c in cyc:
        st = (t >= c["hs_start_s"]) & (t <= c["toe_off_s"])
        sw = (t > c["toe_off_s"]) & (t <= c["hs_end_s"])
        ph[st] = alpha * (t[st] - c["hs_start_s"]) / (c["toe_off_s"] - c["hs_start_s"])
        ph[sw] = alpha + (1 - alpha) * (t[sw] - c["toe_off_s"]) / (c["hs_end_s"] - c["toe_off_s"])
    series = {coord: np.interp(ph, corridor["grid"], corridor["reference"][coord]) for coord in R.PROS_COORDS}
    series["pros_ankle_angle"] = series["pros_ankle_angle"] + 0.05
    pm = G.phase_aligned_metrics(cyc, t, series, corridor["reference"], alpha=alpha, window=(0.55, 0.80))
    check(pm["knee_rmse"] < 1e-3 and pm["knee_r"] > 0.999 and approx(pm["ankle_rmse"], 0.05, 1e-3) and pm["cycles_used"] == 2, "phase-aligned rmse/r on a corridor-identical series")
    win = (corridor["grid"] >= 0.55) & (corridor["grid"] <= 0.80)
    check(approx(pm["ankle_min_window"], float(np.min(corridor["reference"]["pros_ankle_angle"][win])) + 0.05, 2e-3), "ankle_min_window = window minimum of the resampled ankle")
    empty = G.phase_aligned_metrics([], t, series, corridor["reference"], alpha=alpha, window=(0.55, 0.80))
    check(empty["knee_rmse"] is None and empty["knee_r"] is None and empty["ankle_min_window"] is None, "no cycle -> phase-aligned metrics None")
    expect(lambda: G.phase_aligned_metrics([{"hs_start_s": 33.0, "toe_off_s": 33.8, "hs_end_s": 34.5, "duration_s": 1.5}], t, series, corridor["reference"], alpha=alpha, window=(0.55, 0.80)), R.F2RContractError, "cycle beyond the recorded kinematics refused")
    expect(lambda: G.phase_aligned_metrics(cyc, t, series, corridor["reference"], alpha=alpha, window=(0.8, 0.55)), R.F2RContractError, "invalid window refused")


# --- (c) synthetic job directory (temp) ------------------------------------------------------------------


def write_sto(path: Path, names: list[str], data: np.ndarray) -> None:
    lines = [path.stem, "version=1", f"nRows={data.shape[0]}", f"nColumns={data.shape[1]}", "inDegrees=no", "", "endheader", "\t".join(names)]
    lines += ["\t".join(f"{v:.8f}" for v in row) for row in data]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_synthetic_job(root: Path, corridor) -> dict:
    """500-step deterministic job at plus020: 3 accepted FSM cycles (period 1.5 s,
    stance 0.6), online clock informative after 1.0 s with period 1.5 s and a
    -0.03 cycle/s drift vs the prescribed clock, kinematics equal to the corridor
    mean (ankle +0.05) inside the cycles, penetration 18 mm, reserve norm 500 Nm."""
    job = root / "SYN__v3_canonical__plus020__det"
    (job / "sim_outputs").mkdir(parents=True)
    n, dt, reset = 500, 0.01, 10.0
    times = [reset + dt * (k + 1) for k in range(n)]
    t_pre = np.asarray([reset] + times[:-1])
    hs = [10.0, 11.5, 13.0, 14.5, 16.0]
    events = [("heel_strike", 10.5, None, 1.0, -1.0), ("toe_off", 11.4, 10.5, 1.0, -1.0), ("heel_strike", 12.0, 11.4, 1.0, 1.0), ("toe_off", 12.9, 12.0, 1.0, -1.0), ("heel_strike", 13.5, 12.9, 1.0, 1.0), ("toe_off", 14.4, 13.5, 1.0, -1.0), ("heel_strike", 14.99, 14.4, 1.0, 1.0)]
    first, slope, d0 = 100, -0.03, 0.1
    pres = G.prescribed_phase(t_pre, hs)
    rows = []
    valid_cycles = 0
    for k in range(n):
        obs = [0.0] * 35
        obs[1] = 1.0
        obs[15] = 1.0
        if k >= first:
            ph = (pres[k] - (d0 + slope * (t_pre[k] - t_pre[first]))) % 1.0
            obs[14], obs[15], obs[16] = float(np.sin(2 * np.pi * ph)), float(np.cos(2 * np.pi * ph)), 1.5
        trs = []
        for ev, et, seg, sv, cv in events:
            if (k == 0 and et <= times[0]) or (k > 0 and times[k - 1] < et <= times[k]):
                trs.append({"event": ev, "event_time_s": et, "segment_start_time_s": seg, "segment_valid": sv, "cycle_valid": cv, "cycle_reject_reason": ""})
                if ev == "heel_strike" and cv == 1.0:
                    valid_cycles += 1
        a = [-0.9 if k == 7 else 0.3 * math.sin(0.05 * k), -0.4]
        rows.append({"step": k + 1, "time": times[k], "actor_observation_vector_before": obs, "raw_policy_action": a, "applied_policy_action": a, "policy_action_mean": None, "exploration_action_noise": None,
                     "phase_fsm": {"accepted_transitions_this_step": trs, "valid_cycle_count": float(valid_cycles), "valid_hs_count": 4.0 if k == n - 1 else 0.0, "valid_to_count": 3.0, "invalid_event_count": 0.0, "resync_count": 0.0, "hs_cancelled_count": 0.0, "invalid_event_this_step": 0.0, "resync_event_this_step": 0.0, "cycle_rejected_this_step": 0.0},
                     "morphology_causal_diagnostics": {"total_resolved_sample_count": 400, "total_dropped_sample_count": 50, "failed_closed": False}})
    (job / DS.TRACE_FILE).write_text(json.dumps(rows), encoding="utf-8")
    (job / DS.RESET_FILE).write_text(json.dumps({"time": reset, "episode_start_offset_s": R.EXACT_STARTS["plus020"]}), encoding="utf-8")
    (job / DS.SUMMARY_FILE).write_text(json.dumps({"ok": True, "steps": n, "n_actor": 35, "action_seed": 123, "action_selection": "deterministic", "episode_start_offset_s": R.EXACT_STARTS["plus020"], "end_reason": "episode_time_limit", "phase_valid_cycle_count": 3, "phase_valid_hs_count": 4, "phase_valid_to_count": 3, "invalid_event_count": 0, "action_clipped_steps": 0, "action_abs_max": 0.9, "grf_penetration_max_m": 0.018, "reserve_norm_max_nm": 500.0}), encoding="utf-8")
    t = reset + 0.001 * np.arange(n * 10)
    alpha = corridor["alpha"]
    cycles = [(10.5, 11.4, 12.0), (12.0, 12.9, 13.5), (13.5, 14.4, 14.99)]
    ph = np.zeros_like(t)
    for hs0, to, hs1 in cycles:
        st = (t >= hs0) & (t <= to)
        sw = (t > to) & (t <= hs1)
        ph[st] = alpha * (t[st] - hs0) / (to - hs0)
        ph[sw] = alpha + (1 - alpha) * (t[sw] - to) / (hs1 - to)
    knee = np.interp(ph, corridor["grid"], corridor["reference"]["pros_knee_angle"])
    ankle = np.interp(ph, corridor["grid"], corridor["reference"]["pros_ankle_angle"]) + 0.05
    write_sto(job / "sim_outputs" / "rollout_episode_kinematics.sto", ["time", "pelvis_tx", "pros_knee_angle", "pros_ankle_angle"], np.column_stack([t, np.zeros_like(t), knee, ankle]))
    pen = np.zeros_like(t)
    pen[1234] = 0.018
    write_sto(job / "sim_outputs" / "rollout_episode_online_grf.sto", ["time", "left_normal_force", "left_penetration", "left_in_contact"], np.column_stack([t, 400.0 * (pen > 0), pen, (pen > 0).astype(float)]))
    write_sto(job / "sim_outputs" / "rollout_episode_reserve_torques.sto", ["time", "pelvis_tx_reserve_torque", "pelvis_ty_reserve_torque", "pros_knee_angle_reserve_torque", "pros_ankle_angle_reserve_torque"], np.column_stack([t, np.full_like(t, 300.0), np.full_like(t, 400.0), np.zeros_like(t), np.zeros_like(t)]))
    win = (corridor["grid"] >= 0.55) & (corridor["grid"] <= 0.80)
    return {"job": job, "hs": hs, "slope": slope, "ankle_min_window": float(np.min(corridor["reference"]["pros_ankle_angle"][win])) + 0.05, "knee_rom": float(np.ptp(corridor["reference"]["pros_knee_angle"]))}


def test_synthetic_job(names35, corridor, thr):
    tmp = R.portable_tempdir("f2r_gates_")
    try:
        syn = build_synthetic_job(tmp, corridor)
        r = G.extract_record(syn["job"], {"job_id": "SYN_plus020"}, names35=names35, heel_strikes=syn["hs"], corridor=corridor)
        check(r["source"]["metrics_route"] == "sto_fallback" and r["job_id"] == "SYN_plus020" and r["start"] == "plus020" and r["seed"] == 123 and r["action_selection"] == "deterministic", "synthetic job: fallback route, start from the exact offset")
        check(r["steps"] == 500 and r["horizon_completed"] is True and r["phase_timeout"] is False and r["end_reason"] == "episode_time_limit", "synthetic job: horizon")
        check(r["valid_cycles"] == 3 and r["valid_hs"] == 4 and r["valid_to"] == 3 and r["invalid_events"] == 0 and r["hs_cancelled_count"] == 0 and r["resync_count"] == 0 and r["source"]["fsm_cycles_extracted"] == 3, "synthetic job: FSM counters and extracted chain")
        check(approx(r["first_hs_delay_s"], 0.5, 1e-12) and approx(r["online_clock_informative_s"], 1.0, 1e-9) and r["online_periods_s"] == [1.5], "synthetic job: first HS delay, online onset, periods")
        check(approx(r["drift_cycle_per_s"], syn["slope"], 1e-6), "synthetic job: drift slope")
        check(approx(r["stance_fraction"], 0.6, 1e-9), "synthetic job: stance fraction median")
        check(r["knee_rmse"] < 1e-3 and r["knee_r"] > 0.999 and approx(r["ankle_rmse"], 0.05, 1e-3) and approx(r["ankle_min_window"], syn["ankle_min_window"], 2e-3), "synthetic job: phase-aligned metrics vs corridor")
        check(approx(r["settled_fraction"], 0.8, 1e-12) and approx(r["dropped_fraction"], 0.1, 1e-12), "synthetic job: corridor coverage fractions")
        check(approx(r["penetration_max_m"], 0.018, 1e-9) and approx(r["reserve_norm_max_nm"], 500.0, 1e-6) and approx(r["action_abs_max_raw"], 0.9, 1e-6) and r["action_clipped_steps"] == 0, "synthetic job: penetration / reserve / actions")
        check(approx(r["knee_rom"], syn["knee_rom"], 1e-3) and approx(r["ankle_min"], float(np.min(corridor["reference"]["pros_ankle_angle"])) + 0.05, 2e-3), "synthetic job: kinematics extrema")
        check(all(k in r for k in G.RECORD_KEYS) and r["record_schema"] == G.RECORD_SCHEMA, "synthetic job: record schema complete")
        recs = {s: {**r, "start": s} for s in STARTS}
        res = G.evaluate_round(recs, {s: {**r, "start": s, "seed": 125, "action_selection": "stochastic"} for s in STARTS})
        check(res["promotion"] is True and res["failed_gates"] == [] and res["t3_trigger"] is False, "synthetic corridor-identical job passes A, B, C and V (positive control)")
        r_no_hs = G.extract_record(syn["job"], {}, names35=names35, corridor=corridor)
        check(r_no_hs["drift_cycle_per_s"] is None and r_no_hs["source"]["heel_strikes_given"] is False, "without heel strikes the drift is None")
        check(G.gate_A({s: {**r_no_hs, "start": s} for s in STARTS}, thr)["checks"]["A6_online_period_s"]["pass"] is False, "None drift fails A6 (fail-closed)")
        expect(lambda: G.extract_record(tmp / "missing", {}, names35=names35, corridor=corridor), R.F2RContractError, "missing job dir refused")
        expect(lambda: G.extract_record(syn["job"], {}, names35=list(names35)[::-1], corridor=corridor), R.F2RContractError, "wrong manifest names refused")
        expect(lambda: G.extract_record(syn["job"], {"seed": 124}, names35=names35, corridor=corridor), R.F2RContractError, "receipt seed != summary seed refused")
        expect(lambda: G.extract_record(syn["job"], {"start": "nominal"}, names35=names35, corridor=corridor), R.F2RContractError, "receipt start != summary start refused")
        summary = json.loads((syn["job"] / DS.SUMMARY_FILE).read_text(encoding="utf-8"))
        summary["episode_start_offset_s"] = 1.0
        (syn["job"] / DS.SUMMARY_FILE).write_text(json.dumps(summary), encoding="utf-8")
        expect(lambda: G.extract_record(syn["job"], {}, names35=names35, corridor=corridor), R.F2RContractError, "start offset outside the exact starts refused")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# --- (a) real anchor, read-only ---------------------------------------------------------------------------


def test_real_anchor(names35, corridor, thr):
    spec = R.ANCHORS["nominal"]
    job = Path(spec["job_dir"])
    check(job.is_dir() and (job / "f1_receipt.json").is_file(), "nominal anchor job directory present")
    receipt = C.read_json(job / "f1_receipt.json")
    check(G.has_f1_receipt_fields(receipt), "anchor receipt carries the F1 provenance fields")
    hs = C.read_json(job / "f1_adapter_summary.json")["reconstructor"]["clock_heel_strike_times_s"]
    r = G.extract_record(job, receipt, names35=names35, heel_strikes=hs, corridor=corridor)
    check(r["source"]["metrics_route"] == "f1_job_metrics" and r["source"]["trace_sha256"] == spec["trace_sha256"] and r["source"]["summary_sha256"] == spec["summary_sha256"], "real record via f1_job_metrics on the pinned trace")
    check(r["job_id"] == "A_ISO39CLK_V3__v3_canonical__nominal__det" and r["start"] == "nominal" and r["seed"] == 123 and r["action_selection"] == "deterministic", "real record identity")
    check(r["valid_cycles"] == 2 and r["steps"] == 500 and r["horizon_completed"] is True and r["phase_timeout"] is False, "real: 2 valid cycles, 500 steps, horizon completed")
    check(r["hs_cancelled_count"] == 0 and r["resync_count"] == 1, "real: 0 cancellations, 1 resync")
    check(within(r["first_hs_delay_s"], 1.6, 1.7), "real: first accepted heel strike 1.6-1.7 s after reset")
    check(within(r["online_clock_informative_s"], 3.1, 3.2), "real: online clock informative 3.1-3.2 s after reset")
    check(len(r["online_periods_s"]) == 2 and approx(r["online_periods_s"][0], 1.511, 1e-3) and approx(r["online_periods_s"][1], 1.525, 1e-3), "real: online periods ~[1.511, 1.525]")
    check(within(r["stance_fraction"], 0.40, 0.42), "real: stance fraction 0.40-0.42")
    check(within(r["knee_rmse"], 0.27, 0.29) and within(r["knee_r"], 0.33, 0.37), "real: knee rmse 0.27-0.29, r 0.33-0.37 vs corridor")
    check(within(r["ankle_min_window"], 0.15, 0.18), "real: ankle window minimum 0.15-0.18 (never negative)")
    check(approx(r["penetration_max_m"], 0.0126, 5e-4) and approx(r["reserve_norm_max_nm"], 926.0, 2.0), "real: penetration ~12.6 mm, reserve ~926 Nm")
    check(r["drift_cycle_per_s"] is not None and r["drift_cycle_per_s"] < 0 and 0.02 <= abs(r["drift_cycle_per_s"]) <= 0.04, "real: online clock drifts slower than the prescribed one (|drift| 0.02-0.04 cycle/s)")
    # non-regression of the A6 amendment on the three pinned anchors: unwrapped == legacy (no crossing), values as recorded before the amendment
    anchor_expected = {"minus020": -0.02723918026874567, "nominal": -0.020580501688906794, "plus020": -0.019427144478613448}
    for s_a, a in R.ANCHORS.items():
        tr = DS.trajectory_from_job(Path(a["job_dir"]), expected_width=35)
        d_a = G.drift_cycle_per_s(np.asarray(tr["obs35"], dtype=np.float64), np.asarray(tr["t_pre"], dtype=np.float64), G.prescribed_heel_strikes())
        check(approx(d_a["slope"], anchor_expected[s_a], 1e-9) and approx(d_a["slope_wrapped_legacy"], anchor_expected[s_a], 1e-9) and d_a["wrap_crossings"] == 0, f"anchor {s_a}: drift unchanged by the amendment ({d_a['slope']:+.4f}, no wrap crossing)")
    check(approx(r["settled_fraction"], 0.85, 1e-9) and approx(r["dropped_fraction"], 0.15, 1e-9), "real: corridor coverage 425/500 settled, 75/500 dropped")
    check(r["knee_rom"] > 0.6 and r["action_clipped_steps"] == 12 and approx(r["action_abs_max_raw"], 1.2742, 1e-3), "real: knee ROM, clipped steps, raw action max")
    bare = G.extract_record(job, {"job_id": receipt["job_id"]}, names35=names35, heel_strikes=hs, corridor=corridor)
    check(bare["source"]["metrics_route"] == "sto_fallback" and all(bare[k] == r[k] for k in G.RECORD_KEYS), "STO fallback route reproduces the f1_job_metrics scalars exactly")
    recs = {s: {**r, "start": s} for s in STARTS}
    a, b, c = G.gate_A(recs, thr), G.gate_B(recs, thr), G.gate_C(recs, thr)
    check(a["pass"] is True and all(ch["pass"] for ch in a["checks"].values()), "real: gate A PASS (stance 0.41 >= 0.30, clock informative, periods in range)")
    bc = {k: v["pass"] for k, v in b["checks"].items()}
    check(bc["B1_knee_rom_min_rad"] is True and bc["B2_knee_vs_corridor"] is False and bc["B3_ankle_negative_window"] is False and bc["B5_stance_fraction"] is False and bc["B6_corridor_coverage"] is True and b["pass"] is False, "real: gate B -> B1 pass, B2/B3/B5 FAIL, B6 pass")
    check(bc["B4_ankle_rmse_max_rad"] is False, "real: B4 ankle rmse 0.27 > 0.25 FAIL")
    check(c["pass"] is True, "real: gate C PASS")
    res = G.evaluate_round(recs, None)
    check(G.t3_trigger(res) is True and res["t3_trigger"] is True and res["failed_gates"] == ["B", "V"] and res["next"] == G.NEXT_NOT_PROMOTED, "real: preregistered prediction -> T3 trigger True (A PASS, B3 FAIL)")


def test_t1r(thr):
    """T1R corrective commissioning: A hard, B/C informative, V/R NOT_EVALUATED, unlock_t2 = A.pass, never promotion/T3; unlock verification fail-closed."""
    import hashlib, pickle, shutil
    protocol = R.load_protocol()
    res = G.evaluate_round(det3(nominal={"ankle_min_window": 0.1}, plus020={"ankle_min_window": 0.2}), None, protocol, phase="T1R")  # A PASS, B FAIL
    check(res["unlock_t2"] is True and res["promotion"] is False and res["next"] == "STOP_audit_T2_unlocked_on_A_pass" and res["t3_trigger"] is False and res["B"]["informative"] is True and res["C"]["informative"] is True and res["V"]["status"] == "NOT_EVALUATED" and res["A"]["pass"] is True and res["B"]["pass"] is False, "T1R with A PASS / B FAIL: unlock_t2 True, no promotion, STOP, no T3 trigger, B/C informative, V not evaluated")
    res = G.evaluate_round(det3(nominal={"online_clock_informative_s": 4.7}), None, protocol, phase="T1R")  # A FAIL (A5)
    check(res["unlock_t2"] is False and res["promotion"] is False and res["next"] == "STOP_audit_T2_locked_A_failed" and res["t3_trigger"] is False and res["A"]["pass"] is False, "T1R with A FAIL: unlock_t2 False, STOP, no T3 trigger")
    res = G.evaluate_round(det3(nominal={"stance_fraction": 0.45}, plus020={"stance_fraction": 0.45}), None, protocol, phase="T1R")  # would trigger T3 in T2 semantics
    check(res["t3_trigger"] is False and res["unlock_t2"] is True, "B5 FAIL with A PASS never raises a T3 trigger from T1R")
    expect(lambda: G.evaluate_round(det3(), v3(), protocol, phase="T1R"), R.F2RContractError, "T1R never accepts seed-125 validation records")
    # --- verify_t1r_unlock on a synthetic layout (temp root)
    root = R.portable_tempdir("f2r_t1r_unlock_")
    gate_dir, roll, refit = root / "gate", root / "rollouts", root / "refit"
    u0 = G.verify_t1r_unlock(gate_dir=gate_dir, rollouts_dir=roll, refit_dir=refit)
    check(u0["unlocked"] is False and "exactly one" in u0["reason"], "no T1R gate artefact -> locked")
    module = refit / "T1R" / "round_1" / "rl_module_student"; module.mkdir(parents=True)
    (module / "module_state.pkl").write_bytes(pickle.dumps({"synthetic": True})); msha = hashlib.sha256((module / "module_state.pkl").read_bytes()).hexdigest()
    records = {}
    for s_ in R.STARTS:
        jd = roll / "T1R" / "round_1" / f"STUDENT_T1R_r1__v3_canonical__{s_}__det"; jd.mkdir(parents=True)
        (jd / "rollout_policy_trace.json").write_text(json.dumps([{"step": 1, "time": 1.0}]), encoding="utf-8"); (jd / "rollout_summary.json").write_text(json.dumps({"steps": 1}), encoding="utf-8")
        tsha = hashlib.sha256((jd / "rollout_policy_trace.json").read_bytes()).hexdigest(); ssha = hashlib.sha256((jd / "rollout_summary.json").read_bytes()).hexdigest()
        (jd / "f2r_receipt.json").write_text(json.dumps({"status": "ok", "returncode": 0, "seed": 123, "action_selection": "deterministic", "phase": "T1R", "module_state_sha256": msha, "trace_sha256": tsha}), encoding="utf-8")
        records[s_] = {"source": {"trace_sha256": tsha, "summary_sha256": ssha}}
    gate_dir.mkdir()
    good = {"phase": "T1R", "round": 1, "unlock_t2": True, "A": {"pass": True}, "records": {"det": records}}
    (gate_dir / "gate_T1R_round_1_20260823_000000.json").write_text(json.dumps(good), encoding="utf-8")
    real_W = None
    try:
        sys.path.insert(0, str(R.BASELINE_DIR)); import warm_start as W
        real_W = (W.load_module_state, W.actor_state_digest)
        W.load_module_state = lambda m: {"synthetic": True}; W.actor_state_digest = lambda st: "d" * 64
        u1 = G.verify_t1r_unlock(gate_dir=gate_dir, rollouts_dir=roll, refit_dir=refit)
    finally:
        if real_W:
            W.load_module_state, W.actor_state_digest = real_W
    check(u1["unlocked"] is True and u1["student_module_state_sha256"] == msha and u1["student_actor_digest"] == "d" * 64 and u1["gate_sha256"], "consistent layout (gate A PASS + jobs + receipts bound to the module) -> unlocked")
    (gate_dir / "gate_T1R_round_1_20260823_000001.json").write_text(json.dumps(good), encoding="utf-8")
    check(G.verify_t1r_unlock(gate_dir=gate_dir, rollouts_dir=roll, refit_dir=refit)["unlocked"] is False, "two T1R gate artefacts -> ambiguous -> locked")
    (gate_dir / "gate_T1R_round_1_20260823_000001.json").unlink()
    bad = dict(good); bad["unlock_t2"] = False; bad["A"] = {"pass": False}
    (gate_dir / "gate_T1R_round_1_20260823_000000.json").write_text(json.dumps(bad), encoding="utf-8")
    check("did not pass" in G.verify_t1r_unlock(gate_dir=gate_dir, rollouts_dir=roll, refit_dir=refit)["reason"], "gate with A FAIL -> locked")
    (gate_dir / "gate_T1R_round_1_20260823_000000.json").write_text(json.dumps(good), encoding="utf-8")
    tr = roll / "T1R" / "round_1" / "STUDENT_T1R_r1__v3_canonical__nominal__det" / "rollout_policy_trace.json"; tr.write_text(tr.read_text() + " ", encoding="utf-8")
    check("differ from the T1R job on disk" in G.verify_t1r_unlock(gate_dir=gate_dir, rollouts_dir=roll, refit_dir=refit)["reason"], "tampered T1R trace -> locked")
    shutil.rmtree(root, ignore_errors=True)


def main() -> int:
    thr = G.gate_thresholds()
    names35, _ = OA.read_manifest_names(C.ACTOR_MANIFEST_35, expected_sha256=C.ACTOR_MANIFEST_35_SHA256, sha256_fn=C.sha256_file)
    corridor = G.corridor_reference()
    test_rules_and_thresholds(thr)
    test_gate_A(thr)
    test_gate_B(thr)
    test_gate_C(thr)
    test_gate_V(thr)
    test_gate_R(thr)
    test_evaluate_round(thr)
    test_t1r(thr)
    test_phase_helpers(corridor)
    test_synthetic_job(names35, corridor, thr)
    test_real_anchor(names35, corridor, thr)
    print(f"SELFTEST PASS ({CHECKS} checks)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
