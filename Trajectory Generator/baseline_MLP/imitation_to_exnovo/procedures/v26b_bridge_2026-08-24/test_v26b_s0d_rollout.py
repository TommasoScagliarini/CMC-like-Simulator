"""Self-test S0D rollout tooling (post-run-safe; no rollout executed)."""
from __future__ import annotations
import json, shutil, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_s0d_rollout as SR  # noqa: E402
import v26b_r2i_rollout_postrun_audit as PA  # noqa: E402
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
    tmp = R.portable_tempdir("v26b_s0d_ro_")
    try:
        lin = SR.verify_lineage()
        check(lin["s0d_fit_receipt"] == SR.PIN_FIT_RECEIPT and lin["s0d_actor_digest"] == SR.PIN_ACTOR_DIGEST and lin["s0d_module_state"] == SR.PIN_MODULE_STATE, "lineage: fit receipt + module_state + actor digest + addendum + production pins")
        for attr in ("PIN_FIT_RECEIPT", "PIN_ACTOR_DIGEST"):
            old = getattr(SR, attr)
            try:
                setattr(SR, attr, "0"*64); expect(SR.verify_lineage, SR.S0DRolloutError, f"tampered {attr} -> refused")
            finally: setattr(SR, attr, old)
        cmd = SR.rollout_command("/usr/bin/python3")
        check(cmd[cmd.index("--checkpoint") + 1].endswith("S0D_35D_DISTILLED/rl_module") and cmd[cmd.index("--action-selection") + 1] == "deterministic" and cmd[cmd.index("--seed") + 1] == "123" and "--no-auto-config" in cmd, "exact rev3c/e/j nominal det command")
        check("stochastic" not in cmd and "--f1-adapter" not in cmd, "no sigma/adapter")
        # B3: correct-by-construction (reuses PA.b3_late_stance) - synthetic evaluable/not
        def synth(ph, an):
            return [{"step": i+1, "reward_terms": {PA.PHASE_KEY: p}, "actor_observation_vector_before": [0.0]*4 + [a] + [0.0]*30} for i, (p, a) in enumerate(zip(ph, an))]
        s1 = PA.b3_late_stance(synth([0.6, 0.7], [-0.05, 0.1]))
        check(s1["verdict"] == "evaluable" and s1["meets_-0.03_in_window"] is True, "B3 window evaluable path")
        s2 = PA.b3_late_stance(synth([0.0, 0.0], [-0.5, -0.5]))
        check(s2["verdict"] == "not_evaluable_no_valid_phase_or_cycle" and s2["b3_window_min"] is None, "B3 not_evaluable when phase absent (global min never used)")
        real = SR.JOB_DIR
        try:
            SR.JOB_DIR = tmp / "fake"
            e = expect(lambda: SR.run_rollout(authorized_stage=None), SR.S0DRolloutError, "no token -> refused")
            check("V26B-S0D-NOMINAL-ROLLOUT" in str(e) and not (tmp / "fake").exists(), "guard names token, no side effects")
            expect(lambda: SR.run_rollout(authorized_stage="V26B-S0D-FIT"), SR.S0DRolloutError, "wrong token refused")
            (tmp / "fake").mkdir()
            expect(lambda: SR.run_rollout(authorized_stage=SR.AUTHORIZED_STAGE), SR.S0DRolloutError, "no-clobber with right token")
        finally:
            SR.JOB_DIR = real
        check(C.sha256_file(SR.S0D_MODULE / "module_state.pkl") == SR.PIN_MODULE_STATE, "S0D module untouched")
        print(f"SELFTEST PASS ({CHECKS} checks)")
        return 0
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

if __name__ == "__main__":
    raise SystemExit(main())
