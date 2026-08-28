"""Self-test rev3u A2 nominal rollout. Stage-aware: valid before AND after the single launch."""
from __future__ import annotations
import json, sys, tempfile
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_s1b_rollout as SB  # noqa: E402
import v26b_s1b_fit as F  # noqa: E402
import v26b_s1a_rollout as SA  # noqa: E402
import v26b_s1a_bc as A  # noqa: E402
import v26b_r0a_rollout as RO  # noqa: E402
import v26b_r1_rollout as R1  # noqa: E402
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

def row(step, *, fc=0.0, te=0.0, side=0.0, hs=0, rs=0, vc=0):
    return {"step": step,
            "reward_terms": {"morphology_causal_failed_closed": fc, "phase_timeout_exceeded": te, "phase_timeout_side": side,
                             "grf_penetration_m": 0.001, "reserve_norm_nm": 1.0},
            "phase_fsm": {"hs_cancelled_count": hs, "resync_count": rs, "invalid_event_count": 0,
                          "valid_hs_count": 0, "valid_to_count": 0, "valid_cycle_count": vc},
            "raw_policy_action": [0.1, 0.1], "applied_policy_action": [0.1, 0.1],
            "actor_observation_vector_before": [0.0] * 35}

def an(steps=500, end="episode_time_limit", stance=0, swing=0, mcf=(0, 0.0, False), pen=0.013):
    return {"completion": {"steps": steps, "end_reason": end},
            "counters": {"corrected_per_row": {"phase_timeout_stance": stance, "phase_timeout_swing": swing,
                                               "morphology_causal_contract_failure": {"rows_positive": mcf[0], "max": mcf[1], "failure": mcf[2]}}},
            "trajectory_quality": {"penetration_m": {"max": pen}}}

def rs(hs=(0.0, 0.0), r_=(0.0, 0.0), vc=(2.0, 2.0)):
    return {"hs_cancelled_count": {"max_over_rows": hs[0], "final": hs[1]},
            "resync_count": {"max_over_rows": r_[0], "final": r_[1]},
            "valid_cycle_count": {"max_over_rows": vc[0], "final": vc[1]}}

def main() -> int:
    ran_before = SB.JOB_DIR.exists()

    # --- lineage + tamper ------------------------------------------------------------------------
    lin = SB.verify_lineage_rollout()
    check(lin["amendment_rev3u"] == SB.PIN_AMENDMENT_REV3U and lin["amendment_rev3t"] == F.PIN_AMENDMENT_REV3T,
          "lineage pins rev3l -> rev3u")
    check(lin["s1b_fit_aggregate"] == SB.PIN_AGGREGATE == "0539475cda88030e0af059b394974938f64a03b80490134295ca0ed486ae47e6"
          and lin["offline_survivors"] == ["A2"], "the aggregate is pinned at its VERIFIED digest and records A2 as sole survivor")
    check(lin["a2_actor_digest"] == SB.PIN_A2_ACTOR == "cde2c8e6356f833d9e1108b542286c2c9265f8f69bb4c0a894c36136c5d0916c",
          "A2 actor digest verified")
    check(lin["production_pins_verified"] is True and lin["harness_equivalence_vs_s1a"]["equivalent"] is True,
          "production pins verified and harness equivalent to the S1A rollout")
    for attr in ("PIN_AMENDMENT_REV3U", "PIN_AGGREGATE", "PIN_A2_ACTOR", "PIN_A2_RECEIPT"):
        old = getattr(SB, attr)
        try:
            setattr(SB, attr, "0" * 64); expect(SB.verify_lineage_rollout, SB.S1BRolloutError, f"tampered {attr} -> refused")
        finally: setattr(SB, attr, old)

    # --- the reported (truncated) digests must NOT be accepted ---------------------------------------
    am = json.loads(SB.AMENDMENT_REV3U.read_text())
    disc = am["architect_digest_transcription_discrepancy"]
    check(len(disc["reported_aggregate"].split()[0]) == 63 and len(disc["reported_a2_receipt"].split()[0]) == 59
          and len(SB.PIN_AGGREGATE) == 64 and len(SB.PIN_A2_RECEIPT) == 64,
          "the amendment records the truncated reported strings verbatim and pins the 64-char verified ones")

    # --- harness equivalence: only checkpoint and output-dir may differ --------------------------------
    cmd = SB.rollout_command(); frozen = am["harness_equivalence"]["command_resolved"]
    check(cmd == frozen, "driver command byte-identical to the one frozen in rev3u")
    s1a = json.loads(SA.AMENDMENT_REV3R.read_text())["command_resolved"]  # rev3r keeps it top level
    diff = [i for i in range(len(cmd)) if cmd[i] != s1a[i]]
    check(len(cmd) == len(s1a) and diff == [cmd.index("--checkpoint") + 1, cmd.index("--output-dir") + 1],
          "the A2 command differs from the S1A one ONLY in --checkpoint and --output-dir")
    check(cmd[cmd.index("--action-selection") + 1] == "deterministic" and cmd[cmd.index("--seed") + 1] == "123"
          and cmd[cmd.index("--episode-start-offset-s") + 1] == "1.956870983805102"
          and cmd[cmd.index("--config") + 1] == str(F1.RUNTIME_CONFIG)
          and "sigma" not in " ".join(cmd).lower(), "deterministic mean, frozen start/seed/config, no sigma on the command line")
    real = SB.rollout_command
    try:
        SB.rollout_command = lambda python_exe=SB.PYTHON_EXE: [x for x in real(python_exe)][:-1] + ["--stall-timeout-s", "1"]
        expect(SB.harness_equivalence, SB.S1BRolloutError, "a harness deviation beyond checkpoint/output-dir is refused")
    finally:
        SB.rollout_command = real

    # --- candidate strictly read-only, non-deployable, not quarantined -----------------------------------
    st = SB.verify_candidate("test")
    check(st["receipt_sha256"] == SB.PIN_A2_RECEIPT, "A2 fit receipt byte-identical to the pin")
    man = json.loads((SB.CANDIDATE_MODULE / "actor_feature_manifest.json").read_text())
    check(man["deployable"] is False and man["rollout_pending"] is True and man["sigma_unresolved"] is True
          and man["quarantined"] is False and man["offline_verdict"] == "PASS", "A2 flags: survivor, non-deployable, rollout pending")
    saved = dict(A.MANDATORY_FLAGS)
    try:
        A.MANDATORY_FLAGS["deployable"] = True
        expect(lambda: SB.verify_candidate("t"), SB.S1BRolloutError, "the flag check is live")
    finally:
        A.MANDATORY_FLAGS.clear(); A.MANDATORY_FLAGS.update(saved)
    check(SB.CANDIDATE_ID == "A2" and "student" not in SB.JOB_DIR.resolve().parts
          and "student" not in SB.CANDIDATE_DIR.resolve().parts, "only A2, and nothing under student/")
    for other in ("A1", "A3", "A4", "A5", "A6"):
        rec = json.loads((F.out_dir_for(other) / F.RECEIPT_NAME).read_text())
        check(rec["quarantined"] is True, f"{other} stays quarantined and is not rolled out")

    # --- tokens: negatives, then the positive proven at the no-clobber guard --------------------------------
    for bad in (None, "V26B-S1B-FIT", "V26B-S1A-NOMINAL-ROLLOUT", "V26B-S0D-NOMINAL-ROLLOUT", "v26b-s1b-nominal-rollout"):
        e = expect(lambda b=bad: SB.run_rollout(authorized_stage=b), SB.S1BRolloutError, f"token {bad!r} refused")
        check("V26B-S1B-NOMINAL-ROLLOUT" in str(e), "guard names the required token")
    tmp = Path(tempfile.mkdtemp(prefix="s1b_roll_guard_"))
    try:
        e = expect(lambda: SB.run_rollout(authorized_stage=SB.AUTHORIZED_STAGE, job_dir=tmp),
                   SB.S1BRolloutError, "correct token passes the guard and stops at no-clobber")
        check("no-clobber" in str(e), "single-rollout guard: an existing job dir blocks the launch")
    finally:
        tmp.rmdir()

    # --- counter parsing whole-trace ----------------------------------------------------------------------
    rows = [row(1), row(2, te=1.0, side=1.0), row(3, te=1.0, side=2.0), row(4, fc=1.0),
            row(5, hs=1, rs=2, vc=1), row(6, hs=0, rs=1, vc=2)]
    cc = RO.corrected_counters(rows)
    check(cc["phase_timeout_stance"] == 1 and cc["phase_timeout_swing"] == 1
          and cc["morphology_causal_contract_failure"] == {"rows_positive": 1, "max": 1.0, "failure": True},
          "counters computed over ALL rows, never from end_reason or the last row")
    sc = R1.counter_rowscan(rows)
    check(sc["hs_cancelled_count"]["max_over_rows"] == 1.0 and sc["hs_cancelled_count"]["final"] == 0.0
          and sc["resync_count"]["max_over_rows"] == 2.0, "max AND final recorded, and they differ here")

    # --- gate fixtures: each binding gate in isolation ------------------------------------------------------
    base = SB.eligibility_gates(an(), rs())
    check(base["all_pass"] is True and len(base["gates"]) == 7 and "NOT a promotion" in base["meaning"],
          "seven gates pass on a clean fixture; the meaning states it is not a promotion")
    for name, a_, r_ in (("completion_500_time_limit", an(steps=499), rs()),
                         ("phase_timeout_zero", an(stance=1), rs()),
                         ("phase_timeout_zero", an(swing=1), rs()),
                         ("morphology_causal_contract_failure", an(mcf=(1, 1.0, True)), rs()),
                         ("penetration", an(pen=0.0201), rs()),
                         ("hs_cancelled_zero", an(), rs(hs=(1.0, 0.0))),
                         ("resync_at_most_one", an(), rs(r_=(2.0, 1.0))),
                         ("valid_cycle_at_least_one", an(), rs(vc=(1.0, 0.0)))):
        g = SB.eligibility_gates(a_, r_)
        check(g["all_pass"] is False and g["failed"] == [name], f"fixture breaks exactly {name}")
    check(SB.PENETRATION_MAX_M is SA.PENETRATION_MAX_M and SB.PENETRATION_MAX_M == 0.020, "penetration bound reused by reference")

    # --- no promotion / no anchors / sigma unresolved ---------------------------------------------------------
    src = (HERE / "v26b_s1b_rollout.py").read_text()
    check(not any(t in src for t in ("np.tile", "--nominal-repeat", "write_dagger_manifest(", "anchor_manifest")),
          "the driver builds no anchors and no DAgger manifest")
    check('"deployable": True' not in src and "deployable=True" not in src, "the driver never sets a deployable flag")
    check(SB.PASS_STATUS == "CLOSED_LOOP_ELIGIBLE_PENDING_ARCHITECT_AUDIT", "a PASS marks only eligibility pending audit")
    check("UNRESOLVED" in src and "sigma" in src.lower(), "the receipt declares sigma unresolved")

    # --- local scanner: inherited prose allowed and recorded, real markings still refused -----------------------
    prose = SB.INHERITED_PROSE_ALLOWLIST[0]
    seen = SB.assert_no_deployable_marking_local({"analysis": {"note_35d": prose}, "deployable": False}, "fx")
    check(seen == [prose], "documentary prose inherited verbatim from a frozen tool is allowed AND recorded")
    expect(lambda: SB.assert_no_deployable_marking_local({"deployable": True}), SB.S1BRolloutError, "deployable:true still refused")
    expect(lambda: SB.assert_no_deployable_marking_local({"contract": "deployable_markov_controller_state"}),
           SB.S1BRolloutError, "the historical alias contract string still refused")
    expect(lambda: SB.assert_no_deployable_marking_local({"note": "this actor is deployable now"}),
           SB.S1BRolloutError, "any non-allow-listed occurrence still fails closed")
    check(SB.assert_no_deployable_marking_local({"s": "NON-DEPLOYABLE candidate"}) == [], "the NON-DEPLOYABLE form is accepted without allow-listing")

    # --- finalize path: launches nothing, token-gated, refuses without artifacts or with a receipt --------------
    fsrc = (HERE / "v26b_s1b_rollout.py").read_text()
    body = fsrc.split("def finalize_receipt")[1].split("def main")[0]
    check("subprocess.run(" not in body and ".mkdir(" not in body and "Popen" not in body,
          "finalize_receipt contains NO process launch and NO directory creation (the word 'subprocess' appears only in its docstring)")
    e = expect(lambda: SB.finalize_receipt(authorized_stage="V26B-S1B-FIT"), SB.S1BRolloutError, "finalize is token-gated")
    check("V26B-S1B-NOMINAL-ROLLOUT" in str(e), "finalize names the required token")
    tmp2 = Path(tempfile.mkdtemp(prefix="s1b_fin_"))
    try:
        e = expect(lambda: SB.finalize_receipt(authorized_stage=SB.AUTHORIZED_STAGE, job_dir=tmp2),
                   SB.S1BRolloutError, "finalize refuses when no rollout artifacts exist")
        check("cannot finalize" in str(e), "finalize cannot fabricate a receipt for a rollout that never ran")
    finally:
        tmp2.rmdir()

    # --- post-run verification -------------------------------------------------------------------------------
    stage = {"post_run": SB.JOB_DIR.exists()}
    if stage["post_run"]:
        rec = json.loads((SB.JOB_DIR / SB.RECEIPT_NAME).read_text(encoding="utf-8"))
        check(rec["command"] == frozen and rec["amendment_rev3u"] == SB.PIN_AMENDMENT_REV3U, "receipt records the frozen command and rev3u")
        check(rec["candidate"]["actor_digest"] == SB.PIN_A2_ACTOR
              and rec["candidate_after"]["receipt_sha256"] == SB.PIN_A2_RECEIPT
              and rec["candidate_after"]["module_files_sha256"] == rec["lineage"]["candidate_before"]["module_files_sha256"],
              "candidate byte-immutable before and after")
        check(rec["candidate"]["deployable"] is False and rec["anchors_not_built"] is True, "non-deployable; no anchors built")
        check(rec["status"] in (SB.PASS_STATUS, "CLOSED_LOOP_FAIL_QUARANTINED"), "status is either eligibility-pending-audit or quarantined")
        e = expect(lambda: SB.run_rollout(authorized_stage=SB.AUTHORIZED_STAGE), SB.S1BRolloutError, "a second rollout is refused")
        check("no-clobber" in str(e), "exactly-one-launch enforced after the run")
        check(rec["analysis"]["trace_sha256"] == C.sha256_file(SB.JOB_DIR / "rollout_policy_trace.json"), "trace digest matches the receipt")
        comp = rec["diagnostics"]["closed_loop_comparison"]
        check("Diagnostic only, never a gate" in comp["scope"] and "offline_reference_block" in comp,
              "the comparison is labelled diagnostic and keeps the offline block separate")
        check(rec["inherited_prose_allowlisted"] == [SB.INHERITED_PROSE_ALLOWLIST[0]], "exactly the one inherited prose string was allow-listed")
        check(rec["returncode"] == 0 and "proven by the executed code path" in rec["returncode_provenance"],
              "the receipt states how the return code is known, since it was finalized after the run")
        check(rec["receipt_finalized_after_tooling_incident"]["rollout_relaunched"] is False,
              "the incident record states explicitly that the rollout was NOT relaunched")
        e = expect(lambda: SB.finalize_receipt(authorized_stage=SB.AUTHORIZED_STAGE), SB.S1BRolloutError, "finalize refuses when a receipt exists")
        check("already exists" in str(e), "no second receipt can be written")
        stage["status"] = rec["status"]; stage["failed_gates"] = rec["eligibility"]["failed"]
    print(json.dumps({"selftest": "PASS", "checks": CHECKS, **stage}, indent=1))
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
