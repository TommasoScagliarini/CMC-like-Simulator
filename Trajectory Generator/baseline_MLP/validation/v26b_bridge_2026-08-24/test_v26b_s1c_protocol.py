"""Static self-test rev3v S1C: sign/target audit, gate semantics, guards. No fit, no rollout."""
from __future__ import annotations
import json, sys, tempfile
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_s1c_protocol as S  # noqa: E402
import v26b_s1b_rollout as SB  # noqa: E402
import v26b_s0d_rollout as SR  # noqa: E402
import f0_common as C  # noqa: E402
import f2r_common as R  # noqa: E402

CHECKS = 0
def check(cond, what):
    global CHECKS; assert cond, what; CHECKS += 1
def expect(fn, exc, what):
    global CHECKS
    try: fn()
    except exc as e: CHECKS += 1; return e
    raise AssertionError(f"expected {exc.__name__}: {what}")

S0D_BASE = {"pearson_vs_IK": [0.9175, 0.7794]}
def metrics(knee_ik=0.10, ankle_ik=0.12, knee_hz=0.12, ankle_hz=0.047, r=(0.92, 0.79), amp=(0.95, 0.90), ankle_win=-0.05):
    mk = lambda rm, rr, aa: {"rmse": rm, "pearson_r": rr, "amplitude_ratio": aa,
                             "sign_agreement": {"value": None, "verdict": "VOID_degenerate_reference"}}
    return {"vs_prosthetic_IK_target": {"knee": mk(knee_ik, r[0], amp[0]), "ankle": mk(ankle_ik, r[1], amp[1])},
            "vs_healthy_symmetry_diagnostic": {"knee": mk(knee_hz, 0.9, 1.0), "ankle": mk(ankle_hz, 0.9, 1.0)},
            "ankle_window_min": ankle_win}

def main() -> int:
    # --- lineage + tamper ---------------------------------------------------------------------------
    lin = S.verify_lineage_s1c()
    check(lin["amendment_rev3v"] == S.PIN_AMENDMENT_REV3V and lin["amendment_rev3u"] == SB.PIN_AMENDMENT_REV3U,
          "lineage pins rev3l -> rev3v")
    check("REFUSED" in lin["a2_promotion_refused_by_architect"], "the architect's refusal of A2's qualitative promotion is recorded")
    old = S.PIN_AMENDMENT_REV3V
    try:
        S.PIN_AMENDMENT_REV3V = "0" * 64
        expect(S.verify_lineage_s1c, S.S1CError, "tampered rev3v pin -> refused")
        expect(S._amendment, S.S1CError, "the amendment reader is pin-guarded")
    finally:
        S.PIN_AMENDMENT_REV3V = old

    # --- tokens and future-stage guards ---------------------------------------------------------------
    for bad in (None, "V26B-S1B-NOMINAL-ROLLOUT", "V26B-S1C-FIT", "v26b-s1c-protocol"):
        e = expect(lambda b=bad: S.build_audit(authorized_stage=b), S.S1CError, f"token {bad!r} refused")
        check("V26B-S1C-PROTOCOL" in str(e), "guard names this stage's token")
    for fn, tok in ((S.run_probe, "V26B-S1C-FEASIBILITY-PROBE"), (S.run_fit, "V26B-S1C-FIT"), (S.run_rollout, "V26B-S1C-NOMINAL-ROLLOUT")):
        e = expect(lambda f=fn: f(authorized_stage=S.AUTHORIZED_STAGE), S.S1CError, "future stage refused")
        check(tok in str(e) and "NOT granted" in str(e), f"{tok} is named and never unlocked here")
    src = (HERE / "v26b_s1c_protocol.py").read_text()
    check(not any(t in src for t in ("subprocess.run(", "Popen", "import torch", "backward()", "opt.step", "Adam(")),
          "the module contains no fit and no closed-loop primitive")

    # --- decode convention ------------------------------------------------------------------------------
    a = np.array([[0.0, 0.0], [1.0, 1.0], [-1.0, -1.0]])
    q = S.decode_action(a)
    check(np.allclose(q[:, 0], [-0.75, 0.0, -1.5]) and np.allclose(q[:, 1], [0.0, 0.7, -0.7]),
          "decode matches PROTOCOL_F2R sec.3: q_knee = 0.75a - 0.75, q_ankle = 0.7a, and spans the declared bounds")
    check(S.KNEE_DECODE == (0.75, -0.75) and S.ANKLE_DECODE == (0.70, 0.0) and S.B3_THRESHOLD == -0.03,
          "decode constants and the B3 threshold are the frozen ones")

    # --- the sign-agreement bug: degenerate references make it VOID --------------------------------------
    pos_only = np.array([0.1, 0.2, 0.3]); mixed = np.array([-0.1, 0.2, -0.3])
    check(S.sign_degeneracy(pos_only)["degenerate"] is True and S.sign_degeneracy(mixed)["degenerate"] is False,
          "sign degeneracy detected when a reference never changes sign")
    va = S.sign_agreement(pos_only, pos_only)
    check(va["value"] is None and va["verdict"] == "VOID_degenerate_reference",
          "sign agreement against a sign-degenerate reference is VOID, not 1.0 (the rev3u receipt artefact)")
    vb = S.sign_agreement(np.array([-0.1, 0.2, -0.3]), mixed)
    check(vb["verdict"] == "valid" and vb["value"] == 1.0, "against a non-degenerate reference it is computed normally")

    # --- the two references really are different, and the healthy one is degenerate -------------------------
    tgt = S.audit_targets()
    check(tgt["healthy_contralateral_reference"]["ankle"]["degenerate"] is True
          and tgt["healthy_contralateral_reference"]["ankle"]["frac_negative"] == 0.0,
          "the healthy contralateral ankle never goes negative: it cannot test plantarflexion")
    check(tgt["prosthetic_IK_target_decoded"]["ankle"]["frac_negative"] > 0.15
          and tgt["prosthetic_IK_target_decoded"]["ankle"]["min"] < -0.15,
          "the prosthetic IK target DOES contain a negative (plantarflexion) tract")
    check(tgt["rmse_between_the_two_references"]["ankle"] > 0.13,
          "the two references are far apart on the ankle: optimising towards one moves away from the other")

    # --- command chain: where plantarflexion is lost ----------------------------------------------------------
    ch = S.command_chain_audit(SR.JOB_DIR)
    check(ch["commanded_q_ankle"]["frac_negative"] > 0.0 and ch["commanded_q_ankle"]["min"] < -0.4,
          "S0D already commands deep plantarflexion")
    check(ch["served_reference"]["frac_negative"] == 0.0 and ch["realised_q_ankle"]["frac_negative"] == 0.0,
          "the served reference and the realised joint never go negative")
    check(ch["sea_u_ankle"]["frac_abs_gt_0p99"] == 0.0, "the SEA is not saturated: the block is upstream of the actuator")
    check(ch["reference_tracking_error"]["mean_abs"] < 0.01, "the realised joint follows the served reference closely")
    check("never reaches the joint" in ch["verdict"], "the audit states the command/realisation mismatch explicitly")

    # --- gates: each binding one fails in isolation, and A2 would FAIL the missing gate --------------------------
    ok = S.quality_gates(metrics(), s0d_baseline=S0D_BASE)
    check(ok["all_binding_pass"] is True and ok["failed"] == [], "a clean fixture passes every binding gate")
    for name, kw in (("G2_non_regression_vs_IK", {"ankle_ik": 0.20}),
                     ("G3_non_regression_symmetry", {"ankle_hz": 0.20}),
                     ("G4_shape", {"amp": (0.95, 0.50)})):
        g = S.quality_gates(metrics(**kw), s0d_baseline=S0D_BASE)
        check(g["all_binding_pass"] is False and g["failed"] == [name], f"{name} fails in isolation")
    g = S.quality_gates(metrics(r=(0.80, 0.79)), s0d_baseline=S0D_BASE)
    check(g["failed"] == ["G4_shape"], "a Pearson drop beyond 0.02 fails the shape gate")
    a2 = S.quality_metrics(SB.JOB_DIR)
    ga2 = S.quality_gates(a2, s0d_baseline=S0D_BASE)
    check(ga2["gates"]["G2_non_regression_vs_IK"]["pass"] is False,
          "A2 FAILS G2 on the ankle (0.15229 > 0.13458): this is precisely the gate that was missing in rev3u")
    check(ga2["gates"]["G3_non_regression_symmetry"]["pass"] is False, "A2 also fails the symmetry non-regression gate")
    check(ga2["all_binding_pass"] is False, "A2 would NOT pass the S1C quality gates: the architect's refusal is reproduced numerically")

    # --- G5 stays non-binding until the feasibility probe says REACHABLE ------------------------------------------
    for reach, binding in ((None, False), (False, False), (True, True)):
        g = S.quality_gates(metrics(), s0d_baseline=S0D_BASE, ankle_negative_reachable=reach)
        g5 = g["gates"]["G5_ankle_negative_excursion"]
        check(g5["binding"] is binding and (g5["pass"] is None if not binding else g5["pass"] is True),
              f"G5 binding={binding} when reachable={reach}")
        check(("OPEN BLOCKER" in g5["status"]) is (not binding), "G5 is declared an open blocker until proven reachable")
    g = S.quality_gates(metrics(ankle_win=+0.02), s0d_baseline=S0D_BASE, ankle_negative_reachable=True)
    check(g["gates"]["G5_ankle_negative_excursion"]["pass"] is False and "G5_ankle_negative_excursion" in g["failed"],
          "when reachable, a non-negative ankle window minimum fails G5")

    # --- finite candidate set, no sigma assumption ------------------------------------------------------------------
    am = json.loads(S.AMENDMENT_REV3V.read_text())
    cnd = am["S1C_PROTOCOL"]["S1C_1_candidates_finite"]
    check(cnd["count"] == 3 and cnd["open_search"] is False and "w in {2, 4, 8}" in cnd["varied"],
          "S1C-1 is a finite 3-candidate grid with no open search")
    check("only if s1c-0 returns reachable" in cnd["conditional"].lower(), "no fit is run against an unproven objective")
    check("UNRESOLVED" in am["S1C_PROTOCOL"]["sigma"] and "0.005" in am["S1C_PROTOCOL"]["sigma"],
          "sigma stays unresolved and is not assumed")
    check(am["S1C_PROTOCOL"]["init"].startswith("EXCLUSIVELY S0D"), "S1C starts exclusively from S0D")

    # --- no-clobber of the audit artefact ---------------------------------------------------------------------------
    tmp = Path(tempfile.mkdtemp(prefix="s1c_audit_"))
    try:
        o1 = S.build_audit(authorized_stage=S.AUTHORIZED_STAGE, out_dir=tmp)
        o2 = S.build_audit(authorized_stage=S.AUTHORIZED_STAGE, out_dir=tmp)
        check(o1["receipt_path"] != o2["receipt_path"]
              and C.sha256_file(C.REPO / Path(o1["receipt_path"])) == o1["receipt_sha256"],
              "no-clobber: a second audit reserves a new path and leaves the first byte-identical")
        check(o1["receipt"]["executed_in_this_stage"] == {"fit": False, "rollout": False, "collection": False,
                                                          "probe": False, "promotion": False},
              "the audit receipt records that nothing was executed")
    finally:
        import shutil; shutil.rmtree(tmp, ignore_errors=True)
    print(json.dumps({"selftest": "PASS", "checks": CHECKS}, indent=1))
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
