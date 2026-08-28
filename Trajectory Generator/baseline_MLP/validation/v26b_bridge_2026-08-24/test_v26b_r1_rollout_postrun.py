"""POST-RUN-SAFE selftest for the rev3e R1 rollout (Codex HOLD correction A).

Re-runnable at any time after the rollout: refusal/no-side-effect tests run
against a MONKEYPATCHED temporary job dir; the real receipt/trace/report are
verified read-only by digest and never altered.  The pinned pre-run selftest
``test_v26b_r1_rollout.py`` stays untouched (its 19/19 is a historical fact)."""

from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import v26b_r1_postrun_audit as PA  # noqa: E402
import v26b_r1_rollout as R1  # noqa: E402
import v26b_r0a_rollout as RO  # noqa: E402
import v26b_dagger_r1 as D1  # noqa: E402
import f0_common as C  # noqa: E402
import f1_common as F1  # noqa: E402
import f2r_common as R  # noqa: E402

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
    tmp = R.portable_tempdir("v26b_r1_postrun_")
    pre = {"receipt": C.sha256_file(R1.JOB_DIR / R1.RECEIPT_NAME), "trace": C.sha256_file(R1.JOB_DIR / "rollout_policy_trace.json")}
    try:
        # --- frozen hashes (read-only) -------------------------------------------------------
        frozen = PA.verify_frozen()
        check(frozen["receipt"] == PA.PIN_R1_ROLLOUT_RECEIPT and frozen["trace"] == PA.PIN_R1_ROLLOUT_TRACE and frozen["report"] == PA.PIN_R1_REPORT, "receipt/trace/report byte-identical to the pins")
        old = PA.PIN_R1_ROLLOUT_TRACE
        try:
            PA.PIN_R1_ROLLOUT_TRACE = "0" * 64
            expect(PA.verify_frozen, PA.PostrunError, "tampered trace pin -> refused")
        finally:
            PA.PIN_R1_ROLLOUT_TRACE = old

        # --- lineage + actor digest + production unchanged -----------------------------------
        lin = R1.verify_lineage_r1_rollout()
        check(lin["r1_actor_digest"] == R1.PIN_R1_ACTOR_DIGEST and lin["amendment_rev3e"]["sha256"] == R1.PIN_AMENDMENT_REV3E, "full lineage + actor digest verified post-run")
        check(C.sha256_file(Path(F1.ROLLOUT_EVAL)) == R.ROLLOUT_EVAL_SHA256_PINNED and C.sha256_file(Path(F1.RUNTIME_CONFIG)) == F1.RUNTIME_CONFIG_SHA256, "production unchanged (rollout_eval + v3 config still pinned)")

        # --- exactly-one-rollout evidence ----------------------------------------------------
        ev = PA.exactly_one_rollout_evidence()
        check(ev["job_dirs"] == [R1.JOB_DIR.name] and ev["single_receipt"] and ev["returncode"] == 0 and ev["receipt_status"] == "ENDED_EARLY", "exactly one job dir, one receipt, rc=0, ENDED_EARLY")

        # --- independent counters from the trace JSON (never from the receipt) ----------------
        ic = PA.independent_counters(R1.JOB_DIR / "rollout_policy_trace.json")
        check(ic["rows"] == 242 and ic["morphology_causal_contract_failure"]["failure"] is False, "independent: 242 rows, causal failure 0")
        check(ic["phase_timeout_stance"] == 0 and ic["phase_timeout_swing"] == 0, "independent: timeouts 0/0")
        dw = ic["diagnostics"]["morphology_causal_dropped_wait_hs_count"]
        tf = ic["diagnostics"]["morphology_causal_terminal_flushed"]
        check(dw["max"] == 1.0 and dw["last"] == 0.0 and dw["rows_positive"] == 109, "independent: dropped_wait_hs max=1 last=0 (109 rows>0)")
        check(tf["max"] == 1.0 and tf["last"] == 1.0 and tf["first_positive_step"] == 242, "independent: terminal_flushed max=1 last=1 at step 242")
        rec = json.loads((R1.JOB_DIR / R1.RECEIPT_NAME).read_text())
        rd = rec["analysis"]["counters"]["corrected_per_row"]["morphology_causal_diagnostics_last_and_max"]
        check(rd["morphology_causal_dropped_wait_hs_count"]["max"] == 1.0 and rd["morphology_causal_terminal_flushed"]["last"] == 1.0, "receipt already carried the correct values (the report text was the misstatement)")

        # --- addendum exists, content-addressed, consistent ----------------------------------
        ap = R1.JOB_DIR / PA.ADDENDUM_NAME
        check(ap.is_file(), "audit addendum present next to the receipt")
        add = json.loads(ap.read_text())
        check(add["parents_immutable"]["rollout_receipt"]["sha256"] == PA.PIN_R1_ROLLOUT_RECEIPT and add["parents_immutable"]["diagnosis_report"]["sha256"] == PA.PIN_R1_REPORT, "addendum pins receipt/trace/report")
        check(add["highlight"]["morphology_causal_dropped_wait_hs_count"]["max"] == 1.0 and add["highlight"]["morphology_causal_terminal_flushed"]["last"] == 1.0, "addendum: authoritative corrected values")
        check(all(v["match"] for v in add["receipt_vs_independent_consistency"].values()), "receipt vs independent recomputation: consistent")
        expect(PA.write_addendum, PA.PostrunError, "addendum no-clobber")

        # --- refusal / no-side-effect in a MONKEYPATCHED job dir (post-run-safe) --------------
        real_job = R1.JOB_DIR
        try:
            R1.JOB_DIR = tmp / "fake_job_dir"
            e = expect(lambda: R1.run_r1_rollout(authorized_stage=None), R1.R1RolloutError, "no token -> refused")
            check("V26B-R1-ROLLOUT" in str(e) and not (tmp / "fake_job_dir").exists(), "refusal names the token; monkeypatched dir untouched")
            expect(lambda: R1.run_r1_rollout(authorized_stage="V26B-DAGGER-R1"), R1.R1RolloutError, "wrong token refused")
            check(not (tmp / "fake_job_dir").exists(), "still no side effects")
            (tmp / "fake_job_dir").mkdir()
            e = expect(lambda: R1.run_r1_rollout(authorized_stage=R1.AUTHORIZED_STAGE), R1.R1RolloutError, "existing dir -> no-clobber refusal even with the right token")
            check("no-clobber" in str(e) and not any((tmp / "fake_job_dir").iterdir()), "no-clobber refusal; dir left empty")
        finally:
            R1.JOB_DIR = real_job

        # --- nothing altered by this suite ---------------------------------------------------
        check(C.sha256_file(R1.JOB_DIR / R1.RECEIPT_NAME) == pre["receipt"] and C.sha256_file(R1.JOB_DIR / "rollout_policy_trace.json") == pre["trace"], "receipt/trace byte-identical after the suite")
        check(C.sha256_file(D1.OUT_R1 / "rl_module" / "module_state.pkl") == R1.PIN_R1_MODULE_FILES["module_state.pkl"], "R1 module untouched")

        print(f"SELFTEST PASS ({CHECKS} checks)")
        return 0
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
