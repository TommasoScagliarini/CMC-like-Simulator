"""POST-RUN-SAFE self-test for the rev3j R2I rollout tooling (no rollout executed)."""
from __future__ import annotations
import json, shutil, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_r2i_rollout as RR  # noqa: E402
import v26b_r0a_rollout as RO  # noqa: E402
import v26b_r1_rollout as R1  # noqa: E402
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

def main() -> int:
    tmp = R.portable_tempdir("v26b_r2i_ro_")
    try:
        lin = RR.verify_lineage_rollout_r2i()
        check(lin["amendment_rev3j"]["sha256"] == RR.PIN_AMENDMENT_REV3J and lin["r2i_actor_digest"] == RR.PIN_R2I_ACTOR_DIGEST, "rev3j lineage: chain + R2I module/manifest + production pins")
        check(lin["production_pins_verified"] is True, "rollout_eval/v3 yaml/corridor pins verified")
        for attr in ("PIN_AMENDMENT_REV3J", "PIN_R2I_RECEIPT", "PIN_R2I_ACTOR_DIGEST"):
            old = getattr(RR, attr)
            try:
                setattr(RR, attr, "0"*64); expect(RR.verify_lineage_rollout_r2i, RR.R2IRolloutError, f"tampered {attr} -> refused")
            finally: setattr(RR, attr, old)
        cmd = RR.rollout_command("/usr/bin/python3")
        check(cmd[cmd.index("--checkpoint") + 1].endswith("student/V2_R2I/rl_module"), "command targets V2_R2I")
        check(cmd[cmd.index("--action-selection") + 1] == "deterministic" and cmd[cmd.index("--seed") + 1] == "123" and "--no-auto-config" in cmd and cmd[cmd.index("--episode-start-offset-s") + 1] == repr(float(R.EXACT_STARTS["nominal"])), "exact rev3c/rev3e command shape (det, seed 123, pinned config, exact start)")
        check("stochastic" not in cmd and "--f1-adapter" not in cmd, "no stochastic, no adapter")
        # diagnostics on the real R1 trace fixture (read-only)
        rows = json.loads((R1.JOB_DIR / "rollout_policy_trace.json").read_text())
        obs = np.asarray([r["actor_observation_vector_before"] for r in rows], dtype=np.float32)
        ant = RR.ankle_negative_tract(rows, obs)
        check(ant["ankle_min_overall"] < 0 and ant["n_negative_runs"] >= 1 and 0 < ant["negative_fraction_steps"] < 1, "ankle negative-tract diagnostic on the R1 fixture")
        check("DIAGNOSTIC" in ant["b3_reference_diagnostic"]["note"], "B3 recorded as diagnostic, not a gate")
        import f1_dataset as DS
        traj = DS.trajectory_from_job(R1.JOB_DIR, expected_width=35)
        rs = RR.reference_stats(traj["t_pre"])
        check(rs["rows"] == 242 and rs["prescribed_ankle_q"]["min"] < rs["prescribed_ankle_q"]["max"] and len(rs["cache_digest_full"]) == 64, "prescribed reference stats bound to the FULL cache digest")
        check(rs["prescribed_ankle_q"]["negative_fraction"] == 0.0, "healthy prescribed ankle never negative on this grid (fact: the negative tract is a PROSTHETIC-joint property, B3)")
        a1 = json.loads((R1.JOB_DIR / R1.RECEIPT_NAME).read_text())["analysis"]
        st = R1.action_stats(rows)
        cmp3 = RR.compare_three_way(a1, st)
        check(cmp3["r0a"]["steps_end"][0] == 493 and cmp3["r1"]["steps_end"][0] == 242 and cmp3["r2i"]["steps_end"][0] == 242, "three-way comparison built (r2i slot filled by the fixture)")
        # guard + no-side-effect with monkeypatched JOB_DIR (post-run-safe by construction)
        real = RR.JOB_DIR
        try:
            RR.JOB_DIR = tmp / "fake_job"
            e = expect(lambda: RR.run_rollout(authorized_stage=None), RR.R2IRolloutError, "no token -> refused")
            check("V26B-R2I-ROLLOUT" in str(e) and not (tmp / "fake_job").exists(), "refusal names token; nothing written")
            expect(lambda: RR.run_rollout(authorized_stage="V26B-R2I"), RR.R2IRolloutError, "wrong token refused")
            (tmp / "fake_job").mkdir()
            e = expect(lambda: RR.run_rollout(authorized_stage=RR.AUTHORIZED_STAGE), RR.R2IRolloutError, "existing dir -> no-clobber even with the right token")
            check("no-clobber" in str(e) and not any((tmp / "fake_job").iterdir()), "no-clobber refusal, dir untouched")
        finally:
            RR.JOB_DIR = real
        check(C.sha256_file(RR.R2I_MODULE / "module_state.pkl") == RR.PIN_R2I_MODULE_FILES["module_state.pkl"], "R2I module untouched")
        print(f"SELFTEST PASS ({CHECKS} checks)")
        return 0
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

if __name__ == "__main__":
    raise SystemExit(main())
