"""POST-RUN-SAFE test: originals immutability + corrected B3 computation (restricted task)."""
from __future__ import annotations
import json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_r2i_rollout_postrun_audit as PA  # noqa: E402
import v26b_r2i_rollout as RR  # noqa: E402
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
    pre = {"receipt": C.sha256_file(RR.JOB_DIR / RR.RECEIPT_NAME), "trace": C.sha256_file(RR.JOB_DIR / "rollout_policy_trace.json"), "log": C.sha256_file(PA.SIM_LOG)}
    # originals immutability + tamper negative
    h = PA.verify_originals()
    check(h["primary_receipt"] == PA.PIN_PRIMARY_RECEIPT == "6d31bc82137914540c6969540c621373206f51c336257f10852c66b47716e14b", "primary receipt == expected pin")
    check(h["trace"] == PA.PIN_TRACE and h["simulator_log"] == PA.PIN_SIM_LOG, "trace + simulator log pinned")
    old = PA.PIN_TRACE
    try:
        PA.PIN_TRACE = "0"*64; expect(PA.verify_originals, PA.PostrunError, "tampered trace pin -> refused")
    finally: PA.PIN_TRACE = old
    # corrected B3 on the REAL trace
    rows = json.loads((RR.JOB_DIR / "rollout_policy_trace.json").read_text())
    b3 = PA.b3_late_stance(rows)
    check(b3["rows"] == 197 and b3["rows_in_window"] == 0 and b3["phase_all_zero"] is True, "real trace: phase 0 on all 197 rows, 0 in [0.55,0.80]")
    check(b3["verdict"] == "not_evaluable_no_valid_phase_or_cycle" and b3["b3_window_min"] is None, "B3 verdict: not evaluable, NOT PASS")
    check(abs(b3["ankle_min_overall_distinct_diagnostic"] - (-0.18176153302192688)) < 1e-12, "global ankle min distinct diagnostic preserved")
    # synthetic: valid phases -> window computed correctly (meet and not-meet)
    def synth(phase_vals, ankle_vals):
        return [{"step": i+1, "reward_terms": {PA.PHASE_KEY: p}, "actor_observation_vector_before": [0.0]*4 + [a] + [0.0]*30} for i, (p, a) in enumerate(zip(phase_vals, ankle_vals))]
    s = PA.b3_late_stance(synth([0.1, 0.6, 0.7, 0.9], [0.2, -0.05, 0.01, -0.5]))
    check(s["rows_in_window"] == 2 and abs(s["b3_window_min"] - (-0.05)) < 1e-12 and s["verdict"] == "evaluable" and s["meets_-0.03_in_window"] is True, "synthetic: window rows counted, min in-window only, meets true")
    s2 = PA.b3_late_stance(synth([0.6, 0.7], [0.1, -0.01]))
    check(s2["meets_-0.03_in_window"] is False, "synthetic: -0.01 > -0.03 -> not met (global never used)")
    expect(lambda: PA.b3_late_stance([{"step": 1, "reward_terms": {}, "actor_observation_vector_before": [0.0]*35}]), PA.PostrunError, "missing phase field -> fail-closed")
    # addendum on disk (written by the writer before this test run)
    ap = RR.JOB_DIR / PA.ADDENDUM_NAME
    check(ap.is_file(), "addendum present next to the primary receipt")
    add = json.loads(ap.read_text())
    check(add["artefacts_immutable"]["primary_receipt"]["sha256"] == PA.PIN_PRIMARY_RECEIPT and add["b3_corrected"]["verdict"] == "not_evaluable_no_valid_phase_or_cycle", "addendum pins the receipt and carries the corrected verdict")
    check(add["superseded_field_in_primary_receipt"]["value"]["meets"] is True and "GLOBAL" in add["superseded_field_in_primary_receipt"]["why_wrong"], "superseded field recorded with the reason")
    check("NOT EVALUATED" in add["v3_scope_note"] and "3 starts" in add["v3_scope_note"], "V3 scope note: single nominal = diagnostic pre-gate, V3 not evaluated")
    expect(PA.write_addendum, PA.PostrunError, "addendum no-clobber")
    # nothing altered
    check(C.sha256_file(RR.JOB_DIR / RR.RECEIPT_NAME) == pre["receipt"] and C.sha256_file(RR.JOB_DIR / "rollout_policy_trace.json") == pre["trace"] and C.sha256_file(PA.SIM_LOG) == pre["log"], "originals byte-identical after the suite")
    print(f"SELFTEST PASS ({CHECKS} checks)")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
