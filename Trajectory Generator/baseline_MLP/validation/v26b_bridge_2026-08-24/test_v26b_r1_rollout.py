"""Self-test for the rev3e R1 rollout tooling (no rollout executed)."""
from __future__ import annotations
import json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_r1_rollout as R1  # noqa: E402
import v26b_r0a_rollout as RO  # noqa: E402
import f2r_common as R  # noqa: E402
import f0_common as C  # noqa: E402

CHECKS = 0
def check(cond, what):
    global CHECKS
    assert cond, what
    CHECKS += 1
def expect(fn, exc, what):
    global CHECKS
    try:
        fn()
    except exc as e:
        CHECKS += 1
        return e
    raise AssertionError(f"expected {exc.__name__}: {what}")

def main() -> int:
    lin = R1.verify_lineage_r1_rollout()
    check(lin["amendment_rev3e"]["sha256"] == R1.PIN_AMENDMENT_REV3E and lin["r1_actor_digest"] == R1.PIN_R1_ACTOR_DIGEST, "rev3e lineage: chain + R1 receipt/module + actor digest + 35D manifest verified pre-launch")
    check(lin["r1_manifest_verified"] is True, "35D manifest content verified")
    for attr in ("PIN_AMENDMENT_REV3E", "PIN_R1_RECEIPT", "PIN_R1_ACTOR_DIGEST"):
        old = getattr(R1, attr)
        try:
            setattr(R1, attr, "0" * 64)
            expect(R1.verify_lineage_r1_rollout, R1.R1RolloutError, f"tampered {attr} -> refused")
        finally:
            setattr(R1, attr, old)
    cmd = R1.rollout_command("/usr/bin/python3")
    check(cmd[cmd.index("--checkpoint") + 1].endswith("student/V2_DAGGER_R1/rl_module"), "command targets the R1 module")
    check(cmd[cmd.index("--action-selection") + 1] == "deterministic" and cmd[cmd.index("--seed") + 1] == "123" and "--no-auto-config" in cmd, "det seed-123 pinned config (same rev3c protocol)")
    check("stochastic" not in cmd and "--f1-adapter" not in cmd, "no stochastic, no adapter")
    # action stats + rowscan on the real R0a trace (read-only homologous fixture)
    rows = json.loads((RO.JOB_DIR / "rollout_policy_trace.json").read_text())
    st = R1.action_stats(rows)
    check(st["rows"] == 493 and st["raw"]["max_abs"][0] > 1.0 and st["saturation_fraction_rows_absraw_gt_1"][0] > 0.0, "R0a fixture: knee raw saturation visible over the whole trace")
    check(st["final_20_steps_mean_raw"][0] > 1.0, "R0a fixture: saturated final knee commands (comparison hook)")
    cs = R1.counter_rowscan(rows)
    check(cs["invalid_event_count"]["max_over_rows"] == 0.0 and cs["valid_cycle_count"]["final"] == 2.0, "rowscan counters: max AND final over all rows")
    broken = [dict(rows[0]) for _ in range(2)]
    broken[1]["phase_fsm"] = {}
    expect(lambda: R1.counter_rowscan(broken), R1.R1RolloutError, "missing counter key -> fail-closed")
    an0 = json.loads((RO.JOB_DIR / RO.RECEIPT_NAME).read_text())["analysis"]
    cmpb = R1.compare_with_r0a(an0, st)
    check(cmpb["r0a"]["steps_end"][0] == 493 and cmpb["july_homologous_only"]["july_bc_steps"] == 68, "comparison block: R0a facts + July homologous steps only")
    check("NO biological-quality conclusion" in cmpb["no_quality_conclusion"], "no quality conclusion from the offline gate")
    e = expect(lambda: R1.run_r1_rollout(authorized_stage=None), R1.R1RolloutError, "no token -> refused")
    check("V26B-R1-ROLLOUT" in str(e), "refusal names the token")
    expect(lambda: R1.run_r1_rollout(authorized_stage="V26B-DAGGER-R1"), R1.R1RolloutError, "wrong token refused")
    check(not R1.JOB_DIR.exists(), "job dir untouched by refusals")
    check(C.sha256_file(R1.R1_MODULE / "module_state.pkl") == R1.PIN_R1_MODULE_FILES["module_state.pkl"], "R1 module untouched")
    print(f"SELFTEST PASS ({CHECKS} checks)")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
