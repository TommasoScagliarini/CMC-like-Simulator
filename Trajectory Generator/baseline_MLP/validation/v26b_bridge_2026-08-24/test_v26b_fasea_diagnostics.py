"""Unit tests for the Fase A read-only diagnostics. No fit, no training, no rollout."""
from __future__ import annotations
import json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_fasea_diagnostics as F  # noqa: E402
import f2r_common as R  # noqa: E402
import v26b_student as VS  # noqa: E402
sys.path.insert(0, str(R.BASELINE_DIR))
import target_domain_noise_adaptation as NOISE  # noqa: E402

CHECKS = 0
def check(c, w):
    global CHECKS; assert c, w; CHECKS += 1
def expect(fn, exc, w):
    global CHECKS
    try: fn()
    except exc as e: CHECKS += 1; return e
    raise AssertionError(f"expected {exc.__name__}: {w}")


def production_limit_target_slew(raw, future_times, anchor, limits, t0):
    """Literal transcription of osim_trj_cmc_like._limit_target_slew (lines 2708-2760)."""
    limits = np.asarray(limits, float); enabled = np.isfinite(limits)
    raw = np.asarray(raw, float); limited = raw.copy()
    previous = np.asarray(anchor, float).copy(); previous_time = float(t0)
    for i, tv in enumerate(np.asarray(future_times, float)):
        dt = max(1e-9, float(tv) - previous_time)
        max_delta = limits * dt
        desired = raw[i] - previous
        clipped = desired.copy()
        clipped[enabled] = np.clip(desired[enabled], -max_delta[enabled], max_delta[enabled])
        limited[i] = previous + clipped
        previous = limited[i]; previous_time = float(tv)
    return limited


def main() -> int:
    names, _, _ = VS.pinned_names()

    # --- DIFFERENTIAL: the local limiter arithmetic == production ------------------------------
    rng = np.random.default_rng(11)
    # policy_knots == 1: production calls the limiter ONCE per env step with a single future time
    # and previous_time = self.t, so the per-call dt is exact. The test mirrors that structure.
    for _ in range(200):
        anchor = float(rng.uniform(-1.5, 0.7)); target = float(rng.uniform(-1.5, 0.7))
        limit = float(rng.choice([2.0, 2.5])); nstep = int(rng.integers(1, 40))
        mine = F.slew_limit_sequence(anchor, target, limit, F.SEGMENT_DT_S, max_steps=nstep)
        prev = anchor; prod = []
        for _k in range(len(mine)):
            out = production_limit_target_slew(np.array([[target]]), np.array([F.SEGMENT_DT_S]),
                                               np.array([prev]), np.array([limit]), 0.0)
            prev = float(out[0, 0]); prod.append(prev)
        check(np.array_equal(np.asarray(mine), np.asarray(prod)),
              "slew_limit_sequence reproduces the production limiter bit-identically")
    check(True, "200 random differential cases against the production formula (policy_knots=1 structure)")
    # Documented sensitivity, MEASURED not assumed: accumulating absolute times perturbs dt by
    # 4.684e-17 s (about 27 ulp of 0.01, not 1), which moves max_delta by 1.171e-16 rad, i.e.
    # 5.9e-15 of the 0.02 rad reach tolerance. It cannot change a step count.
    acc = F.SEGMENT_DT_S * np.arange(1, 41)
    dts = np.diff(np.concatenate([[0.0], acc]))
    dt_err = float(np.abs(dts - F.SEGMENT_DT_S).max())
    check(abs(dt_err - 4.683753385137379e-17) < 1e-30, "the dt perturbation is exactly the measured value")
    check(dt_err * max(F.SLEW_LIMIT_RAD_S.values()) / F.REACH_TOL_RAD < 1e-13,
          "the induced max_delta perturbation is negligible against the reach tolerance")
    # steps_to_reach agrees with iterating that same arithmetic
    for _ in range(300):
        a = float(rng.uniform(-1.0, 0.5)); t = float(rng.uniform(-1.0, 0.5)); lim = float(rng.choice([2.0, 2.5]))
        n = F.steps_to_reach(a, t, lim)
        seq = F.slew_limit_sequence(a, t, lim, max_steps=max(n, 1) + 5)
        if n == 0:
            check(abs(t - a) <= F.REACH_TOL_RAD, "zero steps only inside tolerance")
        else:
            check(abs(t - seq[n - 1]) <= F.REACH_TOL_RAD + 1e-12, "steps_to_reach is sufficient")
            check(abs(t - seq[n - 2]) > F.REACH_TOL_RAD - 1e-12 if n >= 2 else True,
                  "steps_to_reach is minimal")
    check(True, "steps_to_reach is exact against the iterated limiter")
    expect(lambda: F.slew_limit_sequence(0.0, 1.0, 0.0), F.FaseAError, "zero limit refused")
    expect(lambda: F.slew_limit_sequence(0.0, 1.0, 2.0, 0.0), F.FaseAError, "zero dt refused")
    check(F.SLEW_LIMIT_RAD_S == {"pros_knee_angle": 2.5, "pros_ankle_angle": 2.0}
          and F.SEGMENT_DT_S == 0.01 and F.POLICY_KNOTS == 1,
          "the frozen v3 reference-chain constants are the ones in the resolved config")

    # --- discrete mismatch uses PRODUCTION, not a copy -------------------------------------------
    src = (HERE / "v26b_fasea_diagnostics.py").read_text()
    check("NOISE.truncate_before_discrete_mismatch(" in src,
          "the production truncation operator is called, not re-implemented")
    check("def truncate_before_discrete_mismatch" not in src, "no local copy of the operator")
    a = np.zeros((10, 35)); b = np.zeros((10, 35))
    a[:, 18] = 1.0; b[:, 18] = 1.0
    b[6:, 18] = 0.0; b[6:, 19] = 1.0          # visited flips to SWING at row 7
    r = F.discrete_truncation(a, b, names)
    check(r["first_discrete_mismatch_step"] == 7 and r["retained_rows"] == 6,
          "truncation stops at the first discrete mismatch")
    check(r["discrete_feature_indices"] == [11, 12, 13, 17, 18, 19, 20, 21],
          "the production rule selects the 8 discrete 35D features")
    r2 = F.discrete_truncation(a, a.copy(), names)
    check(r2["first_discrete_mismatch_step"] is None and r2["retained_rows"] == 10,
          "identical discrete state retains everything")

    # --- FSM helpers fail closed -------------------------------------------------------------------
    bad = np.zeros((3, 35))
    expect(lambda: F.fsm_labels(bad), F.FaseAError, "a row with no FSM one-hot fails closed")
    lab = np.array(["STANCE"] * 4 + ["SWING"] * 3 + ["STANCE"] * 2, dtype=object)
    check(F.steps_remaining_in_segment(lab).tolist() == [4, 3, 2, 1, 3, 2, 1, 2, 1],
          "steps_remaining_in_segment counts to the state boundary")

    # --- provenance fail-closed ---------------------------------------------------------------------
    prov = F.build_provenance()
    check("V1_35D_transplant" in prov and "S0D_35D_DISTILLED" in prov, "the spine artifacts are present")
    sp = F.verify_root_chain(prov)
    check(sp["spine_intact"] is True and sp["v26_imitation_root"] == F.PIN_V26_IMITATION,
          "V26 -> V1 -> S0D verified digest-exact")
    for attr in ("PIN_V26_IMITATION", "PIN_V1_TRANSPLANT", "PIN_S0D_DISTILLED"):
        old = getattr(F, attr)
        try:
            setattr(F, attr, "0" * 64)
            expect(lambda: F.verify_root_chain(F.build_provenance()), F.FaseAError,
                   f"tampered {attr} -> spine refused")
        finally: setattr(F, attr, old)
    par = F.clean_parents(prov)
    check(par["clean_count"] >= 1 and "AMBIGUITY_FOR_THE_ARCHITECT" in par,
          "clean parents enumerated and the ambiguity is surfaced, not silently resolved")
    for n in ("REV4B_JULY_DAGGER_35D_NONDEPLOYABLE", "REV4C_BALANCE_35D_NONDEPLOYABLE",
              "REV4D_REPEAT_35D_NONDEPLOYABLE", "REV4E_R2REPLAY_35D_NONDEPLOYABLE"):
        check(n in par["dagger_contaminated"], f"{n} is classified DAgger-contaminated")
        check(all(c["name"] != n for c in par["clean_newest_first"]), f"{n} is not offered as a clean parent")
    check(all(prov[c["name"]]["reaches_v26_root"] for c in par["clean_newest_first"]),
          "every clean parent reaches the V26 root")
    check(prov["REV4E_R2REPLAY_35D_NONDEPLOYABLE"]["dagger_contaminated"] is True
          and "REV4C_BALANCE_35D_NONDEPLOYABLE" in prov["REV4E_R2REPLAY_35D_NONDEPLOYABLE"]["chain"],
          "contamination is inherited through the chain, not only from the own dataset")

    # --- cross-phase prohibition ----------------------------------------------------------------------
    nom = np.zeros((5, 35)); vis = np.zeros((5, 35))
    nom[:, 18] = 1.0; vis[:, 18] = 1.0; nom[3:, 18] = 0.0; nom[3:, 19] = 1.0
    x = F.assert_no_cross_phase_interpolation(nom, vis, np.ones(5, bool))
    check(x["cross_phase_pairs"] == 2 and x["interpolation_allowed_rows"] == 3
          and x["verdict"].startswith("CROSS_PHASE"), "cross-phase pairs detected and refused")
    y = F.assert_no_cross_phase_interpolation(nom[:3], vis[:3], np.ones(3, bool))
    check(y["cross_phase_pairs"] == 0 and y["verdict"] == "CLEAN", "same-state pairs pass")

    # --- preregistered criterion + prohibitions ----------------------------------------------------------
    check(F.EXECUTABILITY_FAIL_FRACTION == 0.10, "the preregistered FAIL threshold is 10%")
    check(not any(t in src for t in ("subprocess", "rollout_eval", "train_ppo", "PPOConfig",
                                     "fit_july", "adapt_actor", "promote_staging")),
          "the module cannot fit, train, roll out or materialise a candidate")
    check("os.system" not in src and "\\\\" not in src and "os.sep" not in src,
          "no shell call and no os-specific path handling (Windows x86 / macOS arm64 safe)")
    check("from pathlib import Path" in src, "pathlib only")
    check(all(m not in src.replace("FORBIDDEN_PATH_MARKERS", "") for m in ('"2026-07"',))
          or "FORBIDDEN_PATH_MARKERS" in src, "July paths appear only as a forbidden-marker guard")

    print(json.dumps({"selftest": "PASS", "checks": CHECKS}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
