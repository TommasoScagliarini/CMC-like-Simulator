"""Static + numerical self-test of the rev3w S1C-0 probe. No episode, no fit, no production change."""
from __future__ import annotations
import json, sys, tempfile
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_s1c0_probe as P  # noqa: E402
import v26b_s1c_protocol as SC  # noqa: E402
import v26b_s0d_rollout as SR  # noqa: E402
import f0_common as C  # noqa: E402

CHECKS = 0
def check(cond, what):
    global CHECKS; assert cond, what; CHECKS += 1
def expect(fn, exc, what):
    global CHECKS
    try: fn()
    except exc as e: CHECKS += 1; return e
    raise AssertionError(f"expected {exc.__name__}: {what}")

def main() -> int:
    # --- lineage, pins, tamper ------------------------------------------------------------------
    lin = P.verify_lineage_probe()
    check(lin["amendment_rev3w"] == P.PIN_AMENDMENT_REV3W and lin["amendment_rev3v"] == SC.PIN_AMENDMENT_REV3V,
          "lineage pins rev3l -> rev3w")
    check(lin["env_source_sha256"] == P.PIN_ENV_SOURCE,
          "the production env source is pinned: the reconstruction is bound to the exact file it was read from")
    for attr in ("PIN_AMENDMENT_REV3W", "PIN_ENV_SOURCE"):
        old = getattr(P, attr)
        try:
            setattr(P, attr, "0" * 64); expect(P.verify_lineage_probe, P.ProbeError, f"tampered {attr} -> refused")
        finally: setattr(P, attr, old)

    # --- tokens and guards -----------------------------------------------------------------------
    for bad in (None, "V26B-S1C-PROTOCOL", "V26B-S1C-FIT", "v26b-s1c-feasibility-probe"):
        e = expect(lambda b=bad: P.run_probe(authorized_stage=b), P.ProbeError, f"token {bad!r} refused")
        check("V26B-S1C-FEASIBILITY-PROBE" in str(e), "guard names this stage's token")
    e = expect(lambda: P.run_fit(authorized_stage=P.AUTHORIZED_STAGE), P.ProbeError, "S1C-1 fit refused here")
    check("V26B-S1C-FIT" in str(e) and "NOT granted" in str(e), "the fit guard names its own future token")
    src = (HERE / "v26b_s1c0_probe.py").read_text()
    check(not any(t in src for t in ("subprocess.run(", "Popen", "import torch", "backward()", "opt.step", "rollout_eval")),
          "no episode, fit or closed-loop primitive in the probe")

    # --- frozen parameters match the amendment ------------------------------------------------------
    fp = json.loads(P.AMENDMENT_REV3W.read_text())["RECONSTRUCTED_PRODUCTION_CHAIN"]["frozen_ankle_parameters"]
    check(fp["target_slew_rate_limit_rad_s"] == P.SLEW_RAD_S == 2.0 and fp["cutoff_hz"] == P.CUTOFF_HZ == 4.0
          and fp["ref_velocity_limit_rad_s"] == P.VEL_LIM and fp["ref_acceleration_limit_rad_s2"] == P.ACC_LIM
          and fp["ref_jerk_limit_rad_s3"] == P.JERK_LIM and fp["fine_steps_per_policy_step"] == P.N_FINE,
          "the module constants are exactly the ones frozen in rev3w")
    check(abs(P.WC - 25.132741228718345) < 1e-12 and P.B3_THRESHOLD is SC.B3_THRESHOLD and P.B3_THRESHOLD == -0.03,
          "wc from the 4 Hz cutoff; the B3 threshold is reused by reference")

    # --- operator 3: decode identities --------------------------------------------------------------
    d = P.decode_absolute(np.array([-1.0, 0.0, 1.0, -5.0, 5.0]))
    check(np.allclose(d, [-0.7, 0.0, 0.7, -0.7, 0.7]),
          "decode maps a=-1/0/+1 to -0.7/0/+0.7 and clips out-of-range actions first (operator 1)")

    # --- operator 4: slew limiter ---------------------------------------------------------------------
    q = P.slew_limit([-0.7] * 50)
    steps = np.diff(np.concatenate([[0.0], q]))
    check(np.all(np.abs(steps) <= 0.02 + 1e-12) and abs(q[0] + 0.02) < 1e-12,
          "the endpoint never moves more than 2.0*0.01 = 0.02 rad per policy step, from the reset anchor 0")
    check(abs(q[-1] - (-0.7)) < 1e-9 and np.argmin(np.abs(np.asarray(q) + 0.7)) == 34,
          "a sustained command reaches -0.7 in 35 steps (0.7/0.02), monotonically")
    osc = P.slew_limit([-0.7, 0.7] * 25)
    check(abs(osc.min()) < 0.03, "an OSCILLATING command leaves the endpoint near zero: the limiter absorbs the spikes")

    # --- operator 7: reference model properties ---------------------------------------------------------
    q, v, a = 0.2, 0.0, 0.0
    for _ in range(200):
        q, v, a = P.reference_step(q, v, a, -0.7)
    check(abs(q + 0.7) < 1e-6 and abs(v) < 1e-6, "unit DC gain: a constant command is tracked exactly in steady state")
    q, v, a = 0.0, 0.0, 0.0
    vs, as_ = [], []
    for _ in range(60):
        q, v, a = P.reference_step(q, v, a, -0.7)
        vs.append(abs(v)); as_.append(abs(a))
    check(max(vs) <= P.VEL_LIM + 1e-9 and max(as_) <= P.ACC_LIM + 1e-9,
          "the governor keeps velocity and acceleration inside the frozen limits under the most aggressive command")
    qg, _, _ = P.reference_step(0.0, 0.0, 0.0, -0.7, governor=True)
    qn, _, _ = P.reference_step(0.0, 0.0, 0.0, -0.7, governor=False)
    check(qg != qn, "the governor is actually active on an aggressive command (it changes the result)")

    # --- validation on the frozen traces, and its DISCRIMINATIVE power -------------------------------------
    rep = P.replay(SR.JOB_DIR)
    check(rep["validated"] is True and rep["best_served_ref_alignment_steps"] == 1,
          "the re-implementation reproduces the recorded served reference, with a ONE-STEP alignment")
    err = rep["served_ref_alignment"]["1"]["max_abs_after_burn_in"]
    check(err <= 1.5e-4, f"post-burn-in max error {err:.2e} rad, an order of magnitude inside the 1e-3 tolerance")
    old = P.WC
    try:
        P.WC = 2.0 * np.pi * 6.0          # the WRONG cutoff assumed before the code audit
        bad = P.replay(SR.JOB_DIR)
        check(bad["validated"] is False,
              "with the 6 Hz cutoff the replay FAILS validation: the test discriminates the true parameter")
    finally:
        P.WC = old
    check(abs(rep["slew_limited_command"]["min"] - rep["recorded_previous_endpoint"]["min"]) < 1e-4,
          "the replicated slew-limited endpoint reproduces the RECORDED previous_endpoint minimum")
    check(rep["raw_command"]["frac_negative"] > 0.10 and rep["slew_limited_command"]["frac_negative"] < 0.02
          and rep["raw_command"]["min"] < -0.4 and rep["slew_limited_command"]["min"] > -0.05,
          "the raw command asks for deep plantarflexion but the slew-limited endpoint barely goes negative")
    check(rep["slew_saturated_fraction"] > 0.85, "the slew limiter saturates on the large majority of steps")

    # --- envelope monotonicity and threshold logic --------------------------------------------------------
    q0, v0, a0 = rep["_states"][150]
    e20 = P.envelope(q0, v0, a0, float(rep["_ep"][150]), 20)
    e38 = P.envelope(q0, v0, a0, float(rep["_ep"][150]), 38)
    e100 = P.envelope(q0, v0, a0, float(rep["_ep"][150]), 100)
    check(e100["min_reached"] <= e38["min_reached"] <= e20["min_reached"],
          "the reachable minimum is monotone non-increasing in the horizon")
    check(e38["steps_to_threshold"] is not None and e38["steps_to_threshold"] <= 38,
          "from a mid-trace state the threshold is crossed inside the B3 window")

    # --- verdict logic on fixtures --------------------------------------------------------------------------
    good = {"validated": True}
    check(P.verdict({"validated": False}, -0.5, 100, 100, -0.7)["verdict"] == "INDETERMINATE", "failed validation -> INDETERMINATE")
    check(P.verdict(good, -0.5, 100, 100, +0.10)["verdict"] == "UNREACHABLE_a_mathematical", "steady state above threshold -> (a)")
    check(P.verdict(good, -0.01, 40, 100, -0.7)["verdict"] == "UNREACHABLE_b_within_horizon", "reachable in the limit but not in the window -> (b)")
    check(P.verdict(good, -0.5, 100, 100, -0.7)["verdict"] == "REACHABLE", "all states cross inside the window -> REACHABLE")
    check("does NOT assert" in P.verdict(good, -0.5, 100, 100, -0.7)["scope_limit"],
          "the REACHABLE verdict carries its scope limit: reference path only, not gait compatibility")

    # --- case (c): the B3 field ---------------------------------------------------------------------------------
    b3 = P.b3_field_status(SR.JOB_DIR)
    check(b3["all_zero"] is True and "NOT EVALUABLE" in b3["case_c"],
          "case (c) recorded separately: the B3 phase field is identically zero on the frozen trace")

    # --- no-clobber ---------------------------------------------------------------------------------------------
    tmp = Path(tempfile.mkdtemp(prefix="s1c0_"))
    try:
        o1 = P.run_probe(authorized_stage=P.AUTHORIZED_STAGE, out_dir=tmp, stride=100)
        o2 = P.run_probe(authorized_stage=P.AUTHORIZED_STAGE, out_dir=tmp, stride=100)
        check(o1["receipt_path"] != o2["receipt_path"]
              and C.sha256_file(C.REPO / Path(o1["receipt_path"])) == o1["receipt_sha256"],
              "no-clobber: a second probe reserves a new path and leaves the first byte-identical")
        check(o1["receipt"]["executed_in_this_stage"] == {"episode": False, "fit": False, "collection": False,
                                                          "promotion": False, "production_change": False},
              "the receipt records that nothing was executed")
    finally:
        import shutil; shutil.rmtree(tmp, ignore_errors=True)
    print(json.dumps({"selftest": "PASS", "checks": CHECKS}, indent=1))
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
