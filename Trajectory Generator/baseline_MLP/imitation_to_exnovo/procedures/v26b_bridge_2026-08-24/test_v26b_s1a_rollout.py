"""Self-test rev3r S1A nominal rollout. Stage-aware: valid before AND after the single launch."""
from __future__ import annotations
import json, sys, tempfile
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_s1a_rollout as SA  # noqa: E402
import v26b_s1a_bc as A  # noqa: E402
import v26b_r0a_rollout as RO  # noqa: E402
import v26b_r1_rollout as R1  # noqa: E402
import v26b_r2i_rollout_postrun_audit as PA  # noqa: E402
import f0_common as C  # noqa: E402
import f1_common as F1  # noqa: E402
import f2r_common as R  # noqa: E402

CHECKS = 0
def check(cond, what):
    global CHECKS; assert cond, what; CHECKS += 1
def expect(fn, exc, what):
    global CHECKS
    try: fn()
    except exc as e: CHECKS += 1; return e
    raise AssertionError(f"expected {exc.__name__}: {what}")

def row(step, *, fc=0.0, te=0.0, side=0.0, hs=0, rs=0, inv=0, vhs=0, vto=0, vc=0, phase=0.0, ankle=0.0, raw=(0.1, 0.1)):
    return {"step": step,
            "reward_terms": {"morphology_causal_failed_closed": fc, "phase_timeout_exceeded": te, "phase_timeout_side": side,
                             "grf_penetration_m": 0.001, "reserve_norm_nm": 1.0, PA.PHASE_KEY: phase},
            "phase_fsm": {"hs_cancelled_count": hs, "resync_count": rs, "invalid_event_count": inv,
                          "valid_hs_count": vhs, "valid_to_count": vto, "valid_cycle_count": vc},
            "raw_policy_action": list(raw), "applied_policy_action": list(raw),
            "actor_observation_vector_before": [0.0] * 35}

def analysis_fx(steps=500, end="episode_time_limit", stance=0, swing=0, mcf=(0, 0.0, False), pen=0.013):
    return {"completion": {"steps": steps, "end_reason": end},
            "counters": {"corrected_per_row": {"phase_timeout_stance": stance, "phase_timeout_swing": swing,
                                               "morphology_causal_contract_failure": {"rows_positive": mcf[0], "max": mcf[1], "failure": mcf[2]}}},
            "trajectory_quality": {"penetration_m": {"max": pen}}}

def rowscan_fx(hs=(0.0, 0.0), rs=(0.0, 0.0), vc=(2.0, 2.0)):
    return {"hs_cancelled_count": {"max_over_rows": hs[0], "final": hs[1]},
            "resync_count": {"max_over_rows": rs[0], "final": rs[1]},
            "valid_cycle_count": {"max_over_rows": vc[0], "final": vc[1]}}

def main() -> int:
    ran_before = SA.JOB_DIR.exists()

    # --- lineage rev3l -> rev3r + tamper ------------------------------------------------------------
    lin = SA.verify_lineage_rollout()
    check(lin["amendment_rev3r"] == SA.PIN_AMENDMENT_REV3R and lin["amendment_rev3q"] == A.PIN_AMENDMENT_REV3Q
          and lin["amendment_rev3p"] == SA.A.P.PIN_AMENDMENT_REV3P, "lineage pins rev3l -> rev3r")
    check(lin["candidate_actor_digest"] == SA.PIN_CANDIDATE_ACTOR == "8f3e0ce17eff7c741dcf72de6d0fec0c372f9dbc7a9b3119d41c155ec8603e35",
          "candidate actor digest pinned")
    check(lin["production_pins_verified"] is True, "rollout_eval / v3 yaml / corridor pins verified")
    for attr in ("PIN_AMENDMENT_REV3R", "PIN_CANDIDATE_RECEIPT", "PIN_CANDIDATE_ACTOR"):
        old = getattr(SA, attr)
        try:
            setattr(SA, attr, "0" * 64); expect(SA.verify_lineage_rollout, SA.S1ARolloutError, f"tampered {attr} -> refused")
        finally: setattr(SA, attr, old)
    old = dict(SA.PIN_CANDIDATE_FILES)
    try:
        SA.PIN_CANDIDATE_FILES["module_state.pkl"] = "0" * 64
        expect(lambda: SA.verify_candidate("t"), SA.S1ARolloutError, "a changed candidate module file is refused")
    finally:
        SA.PIN_CANDIDATE_FILES.clear(); SA.PIN_CANDIDATE_FILES.update(old)

    # --- candidate strictly read-only and non-deployable ---------------------------------------------
    st = SA.verify_candidate("test")
    check(st["module_files_sha256"] == SA.PIN_CANDIDATE_FILES and st["receipt_sha256"] == SA.PIN_CANDIDATE_RECEIPT, "candidate byte state matches the pins")
    man = json.loads((SA.CANDIDATE_MODULE / "actor_feature_manifest.json").read_text())
    rec = json.loads((SA.CANDIDATE_DIR / A.RECEIPT_NAME).read_text())
    check(man["deployable"] is False and rec["deployable"] is False and man["rollout_pending"] is True
          and rec["rollout_pending"] is True and man["sigma_unresolved"] is True, "candidate flags: non-deployable, rollout pending, sigma unresolved")
    A.assert_no_deployable_marking(man, "manifest"); check(True, "candidate manifest carries no deployable marking or alias")
    saved = dict(A.MANDATORY_FLAGS)
    try:
        A.MANDATORY_FLAGS["deployable"] = True
        expect(lambda: SA.verify_candidate("t"), SA.S1ARolloutError, "the flag check is live (a deployable:true expectation fails against the real artifact)")
    finally:
        A.MANDATORY_FLAGS.clear(); A.MANDATORY_FLAGS.update(saved)
    src = (HERE / "v26b_s1a_rollout.py").read_text()
    check("CANDIDATE_DIR" in src and "write_json" not in src.split("def verify_candidate")[0].split("CANDIDATE_DIR")[0], "no write path targets the candidate directory")
    check(all(tok not in src for tok in ("shutil.copy", "shutil.rmtree", "unlink(", "write_text(")), "the driver never writes into any existing artifact")

    # --- exact command ---------------------------------------------------------------------------------
    cmd = SA.rollout_command()
    frozen = json.loads(SA.AMENDMENT_REV3R.read_text())["command_resolved"]
    check(cmd == frozen, "the driver command is byte-identical to the one frozen in rev3r")
    check(cmd[2] == "--checkpoint" and cmd[3] == str(SA.CANDIDATE_MODULE) and "--no-auto-config" in cmd
          and cmd[cmd.index("--config") + 1] == str(F1.RUNTIME_CONFIG)
          and cmd[cmd.index("--episode-start-offset-s") + 1] == "1.956870983805102"
          and cmd[cmd.index("--action-selection") + 1] == "deterministic"
          and cmd[cmd.index("--seed") + 1] == "123"
          and cmd[cmd.index("--output-dir") + 1] == str(SA.JOB_DIR)
          and "--record-outputs" in cmd and "--record-policy-trace" in cmd
          and cmd[cmd.index("--step-timeout-s") + 1] == "900" and cmd[cmd.index("--startup-timeout-s") + 1] == "900",
          "exact rev3c/e/j command shape with the prescribed flags and values")
    check("student" not in str(SA.JOB_DIR) and "--action-selection" in cmd and "sigma" not in " ".join(cmd).lower(),
          "output outside student/, deterministic mean, no operational sigma on the command line")

    # --- tokens: negatives, then the positive proven by reaching the no-clobber guard --------------------
    for bad in (None, "V26B-S1A-BC-FIT", "V26B-S1-FIT", "V26B-S1P-MULTIROLE-FIT", "V26B-S0D-NOMINAL-ROLLOUT", "v26b-s1a-nominal-rollout"):
        e = expect(lambda b=bad: SA.run_rollout(authorized_stage=b), SA.S1ARolloutError, f"token {bad!r} refused")
        check("V26B-S1A-NOMINAL-ROLLOUT" in str(e), "guard names the required token")
    tmp = Path(tempfile.mkdtemp(prefix="s1a_rollout_guard_"))
    try:
        e = expect(lambda: SA.run_rollout(authorized_stage=SA.AUTHORIZED_STAGE, job_dir=tmp),
                   SA.S1ARolloutError, "the correct token passes the guard and stops at no-clobber")
        check("no-clobber" in str(e), "exactly-one-launch: an existing job dir blocks the run (no retry possible)")
    finally:
        tmp.rmdir()

    # --- whole-trace parsers: never last-row-only ---------------------------------------------------------
    rows = [row(1), row(2, te=1.0, side=1.0), row(3, te=1.0, side=2.0), row(4, fc=1.0),
            row(5, hs=1, rs=2, inv=3, vhs=2, vto=2, vc=1), row(6, hs=0, rs=1, inv=3, vhs=2, vto=2, vc=2)]
    cc = RO.corrected_counters(rows)
    check(cc["rows"] == 6 and cc["phase_timeout_stance"] == 1 and cc["phase_timeout_swing"] == 1
          and cc["morphology_causal_contract_failure"] == {"rows_positive": 1, "max": 1.0, "failure": True},
          "corrected_counters scans ALL rows (mid-trace events invisible in the last row are counted)")
    rsn = R1.counter_rowscan(rows)
    check(rsn["hs_cancelled_count"]["max_over_rows"] == 1.0 and rsn["hs_cancelled_count"]["final"] == 0.0
          and rsn["resync_count"]["max_over_rows"] == 2.0 and rsn["resync_count"]["final"] == 1.0,
          "counter_rowscan records max AND final, which differ here by construction")
    act = R1.action_stats(rows)
    check(act["rows"] == 6 and act["rows_raw_neq_applied"] == 0, "action_stats reads the whole trace")
    b3 = PA.b3_late_stance(rows)
    check(b3["verdict"] == "not_evaluable_no_valid_phase_or_cycle" and b3["phase_all_zero"] is True and b3["b3_window_min"] is None,
          "B3 is not_evaluable when the phase field is all zero, and never falls back to a global minimum")
    b3b = PA.b3_late_stance([row(1, phase=0.10), row(2, phase=0.60), row(3, phase=0.95)])
    check(b3b["verdict"] == "evaluable" and b3b["rows_in_window"] == 1, "B3 uses ONLY the rows inside the [0.55, 0.80] phase window")

    # --- fixtures: every binding gate, in isolation ---------------------------------------------------------
    base = SA.eligibility_gates(analysis_fx(), rowscan_fx())
    check(base["all_pass"] is True and base["failed"] == [] and len(base["gates"]) == 7, "the seven gates pass on a clean fixture")
    cases = [("completion_500_time_limit", analysis_fx(steps=499), rowscan_fx()),
             ("phase_timeout_zero", analysis_fx(stance=1), rowscan_fx()),
             ("phase_timeout_zero", analysis_fx(swing=1), rowscan_fx()),
             ("morphology_causal_contract_failure", analysis_fx(mcf=(1, 1.0, True)), rowscan_fx()),
             ("penetration", analysis_fx(pen=0.0201), rowscan_fx()),
             ("hs_cancelled_zero", analysis_fx(), rowscan_fx(hs=(1.0, 0.0))),
             ("resync_at_most_one", analysis_fx(), rowscan_fx(rs=(2.0, 1.0))),
             ("valid_cycle_at_least_one", analysis_fx(), rowscan_fx(vc=(1.0, 0.0)))]
    for name, an, rs in cases:
        g = SA.eligibility_gates(an, rs)
        check(g["all_pass"] is False and g["failed"] == [name], f"fixture breaks exactly {name}")
    g = SA.eligibility_gates(analysis_fx(end="grf_penetration", steps=197, pen=0.0289), rowscan_fx())
    check(set(g["failed"]) == {"completion_500_time_limit", "penetration"}, "a grf_penetration ending fails both completion and penetration")
    check("NOT a deployable promotion" in base["meaning"], "the gate block states it is an eligibility gate, not a promotion")

    # --- no promotion / no anchor construction -----------------------------------------------------------------
    check(not any(t in src for t in ("np.tile", "--nominal-repeat", "write_dagger_manifest(", "anchor_manifest",
                                     "build_anchor", "collect_anchor_rows")),
          "the driver calls no anchor-construction or DAgger-manifest routine (the only 'DAgger' occurrence is a July comparison LABEL, not a call)")
    check(src.count("DAgger") == 1 and '"11_07_DAgger_r2"' in src, "the single DAgger mention is the documentary July homolog label")
    check('"deployable": True' not in src and "deployable=True" not in src and "MANDATORY_FLAGS" in src,
          "the driver never sets a deployable flag; the non-deployable flags come from the frozen rev3q constant")

    # --- post-run verification --------------------------------------------------------------------------------
    stage = {"post_run": SA.JOB_DIR.exists()}
    if stage["post_run"]:
        r = json.loads((SA.JOB_DIR / SA.RECEIPT_NAME).read_text(encoding="utf-8"))
        check(r["command"] == frozen and r["amendment_rev3r"] == SA.PIN_AMENDMENT_REV3R, "receipt records the frozen command and rev3r")
        check(r["candidate"]["files_sha256"] == SA.PIN_CANDIDATE_FILES and r["candidate_after"]["module_files_sha256"] == SA.PIN_CANDIDATE_FILES
              and r["candidate_after"]["receipt_sha256"] == SA.PIN_CANDIDATE_RECEIPT,
              "candidate byte-immutable BEFORE and AFTER the run")
        check(r["candidate"]["deployable"] is False and r["candidate"]["rollout_pending"] is True and r["anchors_not_built"] is True,
              "the candidate stays non-deployable / rollout_pending and no anchors were built")
        e = expect(lambda: SA.run_rollout(authorized_stage=SA.AUTHORIZED_STAGE), SA.S1ARolloutError, "a second launch is refused")
        check("no-clobber" in str(e), "exactly-one-launch enforced after the run (no retry)")
        for f in ("rollout_policy_trace.json", "rollout_summary.json"):
            check((SA.JOB_DIR / f).is_file(), f"{f} preserved")
        check(r["analysis"]["trace_sha256"] == C.sha256_file(SA.JOB_DIR / "rollout_policy_trace.json"), "receipt trace digest matches the preserved trace")
        stage["status"] = r["status"]; stage["failed_gates"] = r.get("eligibility", {}).get("failed")
    print(json.dumps({"selftest": "PASS", "checks": CHECKS, **stage}, indent=1))
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
