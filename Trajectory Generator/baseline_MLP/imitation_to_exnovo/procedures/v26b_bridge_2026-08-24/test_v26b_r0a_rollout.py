"""Self-test for the rev3c rollout tooling (no rollout executed; synthetic + read-only real)."""

from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import v26b_r0a_rollout as RO  # noqa: E402
import v26b_r0a as A  # noqa: E402
import f0_common as C  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_labeller as L  # noqa: E402
import f1_obs_adapter as OA  # noqa: E402
import f1_dataset as DS  # noqa: E402

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


def main() -> int:
    tmp = R.portable_tempdir("v26b_ro_selftest_")
    try:
        # --- lineage -------------------------------------------------------------------------
        lin = RO.verify_lineage_rollout()
        check(lin["amendment_rev3c"]["sha256"] == RO.PIN_AMENDMENT_REV3C and lin["r0a_receipt_sha256"] == RO.PIN_R0A_RECEIPT, "rev3c lineage verified (rev3b chain + rev3c + R0a receipt/module + config + rollout_eval + full cache pins)")
        for attr in ("PIN_AMENDMENT_REV3C", "PIN_R0A_RECEIPT"):
            old = getattr(RO, attr)
            try:
                setattr(RO, attr, "0" * 64)
                expect(RO.verify_lineage_rollout, RO.RolloutError, f"tampered {attr} -> refused")
            finally:
                setattr(RO, attr, old)

        # --- July discrete rule --------------------------------------------------------------
        idx = RO.discrete_feature_indices(R.FEATURE_NAMES_35)
        check(idx == [11, 12, 13, 17, 18, 19, 20, 21], "8 discrete features at the known 35D indices (July name filter)")
        check(RO.discrete_feature_indices(["a_saturated", "x", "phase_fsm_y", "b_in_contact"]) == [0, 2, 3], "filter matches _saturated/phase_fsm_/_in_contact")
        rng = np.random.default_rng(4)
        ref = rng.normal(0, 1, (50, 35)).astype(np.float32)
        same = ref.copy(); same[:, 5] += 1.0  # continuous-only difference
        m = RO.first_discrete_mismatch(ref, same, R.FEATURE_NAMES_35)
        check(m["first_mismatch_step"] is None and m["label_valid_prefix_rows"] == 50, "continuous differences never trigger the discrete mismatch")
        flip = ref.copy(); flip[30, 17] = 1.0 - flip[30, 17]  # phase_fsm_wait_hs flip at row 30 (0-based)
        m2 = RO.first_discrete_mismatch(ref, flip, R.FEATURE_NAMES_35)
        check(m2["first_mismatch_step"] == 31 and m2["label_valid_prefix_rows"] == 30, "first mismatch 1-based, kept prefix = step-1 (July convention)")

        # --- command shape -------------------------------------------------------------------
        cmd = RO.rollout_command("/usr/bin/python3")
        check(cmd[cmd.index("--checkpoint") + 1].endswith("student/V2_R0a/rl_module") and "--no-auto-config" in cmd, "command targets the R0a module with the explicit pinned config")
        check(cmd[cmd.index("--action-selection") + 1] == "deterministic" and cmd[cmd.index("--seed") + 1] == "123", "deterministic seed-123 (no exploration)")
        check(cmd[cmd.index("--episode-start-offset-s") + 1] == repr(float(R.EXACT_STARTS["nominal"])), "exact nominal start")
        check("--f1-adapter" not in cmd and "stochastic" not in cmd, "no adapter flag, no stochastic")

        # --- real read-only analysis fixture: the pinned det nominal V26 anchor job ----------
        an = RO.analyse_rollout(Path(R.ANCHORS["nominal"]["job_dir"]))
        check(an["completion"]["complete_500_time_limit"] is True and an["completion"]["steps"] == 500, "anchor fixture: complete 500 episode_time_limit")
        check(an["counters"]["invalid_event_count"] == 1 and an["counters"]["resync_count"] == 1, "anchor fixture: known counters (invalid=1 resync=1)")
        check(any(e["kind"] == "invalid" and e["step"] == 491 for e in an["counters"]["events"]), "anchor fixture: known invalid event at step 491")
        check(an["first_discrete_fsm_mismatch"]["first_mismatch_step"] is None, "anchor vs itself: no discrete mismatch")
        check(an["july_report_level_gates"]["penetration_v3_guards"]["below_hard"] is True and an["july_report_level_gates"]["clipping"]["action_clipped_steps"] == 12, "anchor fixture: known penetration/clipping facts")
        q = an["trajectory_quality"]
        check(np.isfinite(q["rmse_vs_prescribed_targets_diagnostic"]["knee_q"]) and q["knee_q"]["outside_bounds_steps"] == 0, "trajectory quality finite, knee within bounds")

        # --- synthetic complete job -> DAgger manifest ---------------------------------------
        job = tmp / "synthetic_complete"
        job.mkdir(parents=True)
        real = DS.trajectory_from_job(Path(R.ANCHORS["nominal"]["job_dir"]), expected_width=35)
        reset_time = float(real["reset_time"])
        real_rows = json.loads((Path(R.ANCHORS["nominal"]["job_dir"]) / "rollout_policy_trace.json").read_text())
        times = [float(r["time"]) for r in real_rows]  # exact env grid floats
        obs = real["obs35"].astype(np.float32).copy()
        obs[100:, 5] += np.float32(0.01)  # continuous drift only (labels stay valid)
        fsm_tmpl = {"timeout_exceeded": False, "timeout_side": None, "hs_cancelled_count": 0, "resync_count": 0, "invalid_event_count": 0, "valid_cycle_count": 2, "valid_hs_count": 2, "valid_to_count": 2, "state_name": "SWING_AFTER_TO", "invalid_event_this_step": False, "resync_event_this_step": False}
        rows = [{"step": i + 1, "time": times[i], "actor_observation_vector_before": [float(v) for v in obs[i]],
                 "raw_policy_action": [0.1, 0.1], "policy_action_mean": None, "phase_fsm": dict(fsm_tmpl),
                 "reward_terms": {"grf_penetration_m": 0.012, "reserve_norm_nm": 300.0, "morphology_causal_failed_closed": 0.0, "phase_timeout_exceeded": 0.0, "phase_timeout_side": 0.0}} for i in range(500)]
        (job / "rollout_policy_trace.json").write_text(json.dumps(rows))
        (job / "rollout_summary.json").write_text(json.dumps({"n_actor": 35, "steps": 500, "action_seed": 123, "action_selection": "deterministic", "episode_start_offset_s": float(R.EXACT_STARTS["nominal"]), "end_reason": "episode_time_limit", "episode_return": 30.0, "action_clipped_steps": 0, "morphology_settled_segments": 400, "morphology_discarded_segments": 0, "morphology_settled_samples": 400, "morphology_discarded_samples": 90}))
        (job / "rollout_reset_diagnostics.json").write_text(json.dumps({"time": reset_time}))
        an2 = RO.analyse_rollout(job)
        check(an2["completion"]["complete_500_time_limit"] and an2["first_discrete_fsm_mismatch"]["first_mismatch_step"] is None, "synthetic complete job analysed; continuous drift -> no mismatch")
        man = RO.write_dagger_manifest(job, an2, lin)
        payload = json.loads((job / "v26b_dagger_plan_manifest.json").read_text())
        check(payload["status"].startswith("PLAN/MANIFEST ONLY") and payload["counts"]["rows_after_truncation"] == 500, "manifest written: plan-only, 500 valid rows")
        check(payload["labels"]["cache_digest"] == A.IK_CACHE_DIGESTS_FULL["nominal"] and payload["labels"]["u_ik_finite_in_bounds"], "labels bound to the FULL-pinned nominal cache")
        check("teacher_actions[step-1]" in payload["labels"]["rule"], "July same-step semantics cited in the manifest")
        expect(lambda: RO.write_dagger_manifest(job, an2, lin), RO.RolloutError, "manifest no-clobber")
        # early-ended job -> no manifest
        an_early = json.loads(json.dumps(an2)); an_early["completion"]["complete_500_time_limit"] = False
        expect(lambda: RO.write_dagger_manifest(job, an_early, lin), RO.RolloutError, "manifest refused for an early-ended rollout")
        # mismatch truncation propagates to counts
        flip_obs = obs.copy(); flip_obs[200, 17] = 1.0 - flip_obs[200, 17]
        m3 = RO.first_discrete_mismatch(real["obs35"].astype(np.float32), flip_obs, R.FEATURE_NAMES_35)
        check(m3["first_mismatch_step"] == 201 and m3["label_valid_prefix_rows"] == 200, "discrete flip at row 200 -> kept 200")

        # --- corrected per-row counters (architect audit) ------------------------------------
        def rt_row(fc=0.0, te=0.0, side=0.0):
            return {"grf_penetration_m": 0.012, "reserve_norm_nm": 300.0, "morphology_causal_failed_closed": fc,
                    "phase_timeout_exceeded": te, "phase_timeout_side": side,
                    "morphology_causal_cancelled_transition_count": 0.0, "morphology_causal_dropped_pending_count": 1.0,
                    "morphology_causal_dropped_wait_hs_count": 0.0, "morphology_causal_timeout_transition_count": 0.0,
                    "morphology_causal_terminal_flushed": 0.0, "morphology_causal_delay_s": 0.05}
        synth = [{"step": i + 1, "reward_terms": rt_row()} for i in range(20)]
        synth[4]["reward_terms"] = rt_row(te=1.0, side=1.0)   # stance timeout row
        synth[5]["reward_terms"] = rt_row(te=1.0, side=1.0)
        synth[11]["reward_terms"] = rt_row(te=1.0, side=2.0)  # swing timeout row
        synth[15]["reward_terms"] = rt_row(fc=1.0)            # causal failure row
        cc = RO.corrected_counters(synth)
        check(cc["phase_timeout_stance"] == 2 and cc["phase_timeout_swing"] == 1 and cc["phase_timeout_exceeded_rows"] == 3, "timeouts counted over ALL rows, split by side 1/2")
        check(cc["morphology_causal_contract_failure"]["failure"] is True and cc["morphology_causal_contract_failure"]["rows_positive"] == 1, "causal failure from per-row failed_closed, not end_reason")
        check(cc["morphology_causal_diagnostics_last_and_max"]["morphology_causal_dropped_pending_count"]["max"] == 1.0, "causal diagnostics recorded (last/max)")
        clean = RO.corrected_counters([{"step": 1, "reward_terms": rt_row()}])
        check(clean["morphology_causal_contract_failure"]["failure"] is False and clean["phase_timeout_stance"] == 0, "clean rows -> zero everywhere")
        broken = [{"step": 1, "reward_terms": {"grf_penetration_m": 0.0}}]
        expect(lambda: RO.corrected_counters(broken), RO.RolloutError, "missing mandated field -> fail-closed")
        # real anchor trace: corrected counters all zero
        rrows = json.loads((Path(R.ANCHORS["nominal"]["job_dir"]) / "rollout_policy_trace.json").read_text())
        rc = RO.corrected_counters(rrows)
        check(rc["morphology_causal_contract_failure"]["failure"] is False and rc["phase_timeout_stance"] == 0 and rc["phase_timeout_swing"] == 0, "real anchor: corrected counters all zero")

        # --- audit addendum writer on the synthetic job --------------------------------------
        (job / RO.RECEIPT_NAME).write_text(json.dumps({"schema": "v26b_r0a_rollout.1", "analysis": {"trace_sha256": C.sha256_file(job / "rollout_policy_trace.json"), "counters": {"morphology": {"causal_contract_failure": False}, "phase_timeout": {"timeout_exceeded": False}}}}))
        add = RO.write_audit_addendum(job)
        payload = json.loads((job / "v26b_r0a_rollout_receipt_audit_addendum.json").read_text())
        check(payload["parent_receipt"]["sha256"] == C.sha256_file(job / RO.RECEIPT_NAME) and payload["parent_receipt"]["preserved"] is True, "addendum binds the preserved parent receipt by digest")
        check(payload["trace"]["sha256"] == C.sha256_file(job / "rollout_policy_trace.json"), "addendum binds the immutable trace by digest")
        check(payload["corrected"]["phase_timeout_stance"] == 0 and "superseded_fields_in_parent" in payload, "corrected metrics + superseded fields recorded")
        expect(lambda: RO.write_audit_addendum(job), RO.RolloutError, "addendum no-clobber")
        # manifest gating check: only 500/500 gates the manifest (counters never block)
        an_counters = json.loads(json.dumps(an2)); an_counters["counters"]["invalid_event_count"] = 3
        job2 = tmp / "synthetic_manifest2"; shutil.copytree(job, job2)
        (job2 / "v26b_dagger_plan_manifest.json").unlink()
        man2 = RO.write_dagger_manifest(job2, an_counters, lin)
        check(man2["rows_after_truncation"] == 500, "nonzero counters never block the manifest; only 500/500 gates it")

        # --- guard ---------------------------------------------------------------------------
        e = expect(lambda: RO.run_rollout(authorized_stage=None), RO.RolloutError, "rollout without the stage token refused")
        check("V26B-R0A-ROLLOUT" in str(e), "refusal names the token (guard precedes any write: first statement of run_rollout)")
        expect(lambda: RO.run_rollout(authorized_stage="V26B-V2-R0A"), RO.RolloutError, "wrong stage token refused")

        print(f"SELFTEST PASS ({CHECKS} checks)")
        return 0
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
