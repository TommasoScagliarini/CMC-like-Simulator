"""Self-test rev4a L20 nominal rollout. Stage-aware: valid before AND after the single launch."""
from __future__ import annotations
import json, sys, tempfile
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_l20_rollout as LR  # noqa: E402
import v26b_s1c2z_fit as Z  # noqa: E402
import v26b_s1b_rollout as SB  # noqa: E402
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
    ran = LR.JOB_DIR.exists()

    # --- lineage, pins, proposal preserved ------------------------------------------------------
    lin = LR.verify_lineage()
    check(lin["amendment_rev4a"] == LR.PIN_AMENDMENT_REV4A and lin["amendment_rev3z"] == Z.PIN_AMENDMENT_REV3Z,
          "lineage pins rev3z and rev4a")
    check(lin["proposal_preserved_non_authoritative"] == LR.PIN_PROPOSAL, "the non-authoritative proposal is preserved byte-identical")
    check(lin["l20_actor_digest"] == LR.PIN_L20_ACTOR == "71d21f309ccf1df7bf8aac60cbf8d1c4322586a6f0fe959f8f522624ee23db55",
          "L20 actor digest verified")
    check(lin["production_pins_verified"] is True and lin["harness_equivalence_vs_a2"]["equivalent"] is True,
          "production pins verified and harness equivalent to the A2 rollout")
    for attr in ("PIN_AMENDMENT_REV4A", "PIN_PROPOSAL", "PIN_L20_ACTOR", "PIN_L20_RECEIPT"):
        old = getattr(LR, attr)
        try:
            setattr(LR, attr, "0" * 64); expect(LR.verify_lineage, LR.L20RolloutError, f"tampered {attr} -> refused")
        finally: setattr(LR, attr, old)

    # --- historical outcomes are NOT rewritten ---------------------------------------------------------
    check(lin["rev3z_aggregate_states_preserved"] == {"L05": Z.STATE_FAIL, "L10": Z.STATE_FAIL, "L20": Z.STATE_FAIL},
          "the rev3z aggregate still records all three candidates as OFFLINE_FAILED_QUARANTINED")
    rec = json.loads((LR.CANDIDATE_DIR / Z.RECEIPT_NAME).read_text())
    check(rec["offline_state"] == Z.STATE_FAIL
          and rec["binding_offline_hierarchy"]["levels"]["P_preservation"]["violations"] == ["max_abs>0.25"]
          and rec["binding_offline_hierarchy"]["verdict"] == "FAIL",
          "L20's own receipt still records the FAIL on max_abs: nothing was re-scored retroactively")
    am = json.loads(LR.AMENDMENT_REV4A.read_text())
    d = am["MAX_ABS_DEMOTION_SCOPED"]
    check("historical_outcomes_NOT_rewritten" in d and "ONLY to admitting L20" in d["scope_limit"],
          "the demotion is explicitly scoped and does not rewrite history")
    check(all(k in " ".join(d["unchanged_and_still_binding_elsewhere"]) for k in ("mean_abs", "rms", "T target", "closed-loop")),
          "every other gate remains binding")

    # --- the mandatory documentary correction is recorded ------------------------------------------------
    corr = am["MANDATORY_DOCUMENTARY_CORRECTION_DAGGER"]
    check("REJECTED as stated" in corr["architect_ruling"] and "anchor 1e-3" in corr["protocol_features_to_which_the_degradation_is_attributed"],
          "the DAgger correction and the protocol attribution are recorded")
    check("CONFIRMED" in corr["worker_verification"] and "0.001" in corr["worker_verification"],
          "the worker verified the 1e-3 anchor against the R0a and R1 receipts")
    check(corr["forward_rule"]["if_L20_FAILS"].startswith("the next architectural candidate REMAINS")
          and corr["forward_rule"]["binding"].startswith("this rule is recorded now"),
          "the forward rule is recorded BEFORE the outcome is known")

    # --- harness equivalence: only checkpoint and output-dir may differ -------------------------------------
    cmd = LR.rollout_command()
    a2 = json.loads(SB.AMENDMENT_REV3U.read_text())["harness_equivalence"]["command_resolved"]
    diff = [i for i in range(len(cmd)) if cmd[i] != a2[i]]
    check(len(cmd) == len(a2) and diff == [cmd.index("--checkpoint") + 1, cmd.index("--output-dir") + 1],
          "the L20 command differs from the A2 one ONLY in --checkpoint and --output-dir")
    check(cmd[cmd.index("--action-selection") + 1] == "deterministic" and cmd[cmd.index("--seed") + 1] == "123"
          and cmd[cmd.index("--episode-start-offset-s") + 1] == "1.956870983805102"
          and cmd[cmd.index("--config") + 1] == str(F1.RUNTIME_CONFIG) and "sigma" not in " ".join(cmd).lower(),
          "deterministic mean, frozen start/seed/config, no sigma on the command line")
    real = LR.rollout_command
    try:
        LR.rollout_command = lambda python_exe=LR.PYTHON_EXE: real(python_exe)[:-1] + ["--stall-timeout-s", "1"]
        expect(LR.harness_equivalence, LR.L20RolloutError, "a harness deviation beyond checkpoint/output-dir is refused")
    finally:
        LR.rollout_command = real

    # --- candidate read-only, non-deployable ------------------------------------------------------------------
    st = LR.verify_candidate("test")
    check(st["fit_receipt_sha256"] == LR.PIN_L20_RECEIPT, "L20 fit receipt byte-identical to the pin")
    man = json.loads((LR.CANDIDATE_MODULE / "actor_feature_manifest.json").read_text())
    check(man["deployable"] is False and man["sigma_unresolved"] is True and man["candidate_id"] == "L20"
          and man["lambda_h"] == 2.0, "L20 flags and identity")
    check("student" not in LR.JOB_DIR.resolve().parts and "student" not in LR.CANDIDATE_DIR.resolve().parts,
          "nothing under student/")
    for other in ("L05", "L10"):
        check(Z.out_dir_for(other).exists() and not (LR.JOB_DIR.parent.parent / f"{other.lower()}_nominal_det").exists(),
              f"{other} exists but is NOT rolled out")

    # --- tokens: negatives, then the positive proven at the no-clobber guard --------------------------------------
    for bad in (None, "V26B-S1C-2Z-FIT", "V26B-S1B-NOMINAL-ROLLOUT", "V26B-S1C-3-DIAG", "v26b-l20-nominal-rollout"):
        e = expect(lambda b=bad: LR.run_rollout(authorized_stage=b), LR.L20RolloutError, f"token {bad!r} refused")
        check("V26B-L20-NOMINAL-ROLLOUT" in str(e), "guard names the required token")
    tmp = Path(tempfile.mkdtemp(prefix="l20_guard_"))
    try:
        e = expect(lambda: LR.run_rollout(authorized_stage=LR.AUTHORIZED_STAGE, job_dir=tmp),
                   LR.L20RolloutError, "the correct token passes the guard and stops at no-clobber")
        check("no-clobber" in str(e), "single-rollout guard: an existing job dir blocks the launch")
    finally:
        tmp.rmdir()

    # --- counters whole-trace and the seven gates in isolation --------------------------------------------------------
    rows = [row(1), row(2, te=1.0, side=1.0), row(3, te=1.0, side=2.0), row(4, fc=1.0), row(5, hs=1, rs=2, vc=1), row(6, rs=1, vc=2)]
    cc = RO.corrected_counters(rows)
    check(cc["phase_timeout_stance"] == 1 and cc["phase_timeout_swing"] == 1
          and cc["morphology_causal_contract_failure"] == {"rows_positive": 1, "max": 1.0, "failure": True},
          "counters computed over ALL rows, never from end_reason")
    sc = R1.counter_rowscan(rows)
    check(sc["hs_cancelled_count"]["max_over_rows"] == 1.0 and sc["hs_cancelled_count"]["final"] == 0.0,
          "max and final recorded and different here")
    base = LR.eligibility_gates(an(), rs())
    check(base["all_pass"] is True and len(base["gates"]) == 7 and "NOT a promotion" in base["meaning"],
          "the seven gates pass on a clean fixture and the meaning denies promotion")
    for name, a_, r_ in (("completion_500_time_limit", an(steps=499), rs()),
                         ("phase_timeout_zero", an(stance=1), rs()),
                         ("phase_timeout_zero", an(swing=1), rs()),
                         ("morphology_causal_contract_failure", an(mcf=(1, 1.0, True)), rs()),
                         ("penetration", an(pen=0.0201), rs()),
                         ("hs_cancelled_zero", an(), rs(hs=(1.0, 0.0))),
                         ("resync_at_most_one", an(), rs(r_=(2.0, 1.0))),
                         ("valid_cycle_at_least_one", an(), rs(vc=(1.0, 0.0)))):
        g = LR.eligibility_gates(a_, r_)
        check(g["all_pass"] is False and g["failed"] == [name], f"fixture breaks exactly {name}")
    check(LR.PENETRATION_MAX_M is SB.PENETRATION_MAX_M and LR.PENETRATION_MAX_M == 0.020,
          "the penetration bound is reused by reference and unchanged")

    # --- no fit, no anchors, no promotion, sigma untouched --------------------------------------------------------------
    src = (HERE / "v26b_l20_rollout.py").read_text()
    check(not any(t in src for t in ("import torch", "backward()", "opt.step", "np.tile", "write_dagger_manifest(")),
          "the driver performs no fit and builds no anchors")
    check('"deployable": True' not in src and "deployable=True" not in src, "the driver never sets a deployable flag")
    check(LR.PASS_STATUS == "CLOSED_LOOP_ELIGIBLE_PENDING_ARCHITECT_AUDIT" and LR.FAIL_STATUS == "CLOSED_LOOP_FAIL_QUARANTINED",
          "the two outcome states are the prescribed ones")
    check("UNRESOLVED" in src and "does NOT resolve sigma" in src, "the receipt declares sigma unresolved")

    # --- post-run ---------------------------------------------------------------------------------------------------------
    stage = {"post_run": ran}
    if ran:
        r = json.loads((LR.JOB_DIR / LR.RECEIPT_NAME).read_text(encoding="utf-8"))
        check(r["command"] == cmd and r["amendment_rev4a"] == LR.PIN_AMENDMENT_REV4A, "receipt records the command and rev4a")
        check(r["candidate"]["actor_digest"] == LR.PIN_L20_ACTOR
              and r["candidate_after"]["fit_receipt_sha256"] == LR.PIN_L20_RECEIPT
              and r["candidate_after"]["module_files_sha256"] == r["lineage"]["candidate_before"]["module_files_sha256"],
              "candidate byte-immutable before and after")
        check(r["candidate"]["recorded_offline_state"] == Z.STATE_FAIL and r["candidate"]["deployable"] is False,
              "the receipt preserves the recorded offline FAIL and the non-deployable flag")
        check(r["status"] in (LR.PASS_STATUS, LR.FAIL_STATUS) and r["anchors_not_built"] is True,
              "status is one of the two prescribed ones and no anchors were built")
        e = expect(lambda: LR.run_rollout(authorized_stage=LR.AUTHORIZED_STAGE), LR.L20RolloutError, "a second rollout is refused")
        check("no-clobber" in str(e), "exactly-one-launch enforced after the run")
        check(r["analysis"]["trace_sha256"] == C.sha256_file(LR.JOB_DIR / "rollout_policy_trace.json"),
              "trace digest matches the receipt")
        comp = r["diagnostics"]["closed_loop_comparison"]
        check("never a gate" in comp["scope"] and "offline_reference_block" in comp
              and set(comp["homologs"]) == {"S0D", "A2", "S1A"}, "the comparison is diagnostic and covers the three homologs")
        stage["status"] = r["status"]; stage["failed_gates"] = r["eligibility"]["failed"]
    print(json.dumps({"selftest": "PASS", "checks": CHECKS, **stage}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
