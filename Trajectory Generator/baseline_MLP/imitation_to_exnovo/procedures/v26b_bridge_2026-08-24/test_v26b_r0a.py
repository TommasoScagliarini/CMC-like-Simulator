"""Self-test for the productive R0a tooling (rev3b): lineage, full pins, dataset, gate, export."""

from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import v26b_r0a as A  # noqa: E402
import v26b_v2 as V2  # noqa: E402
import f0_common as C  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_refit as RF  # noqa: E402

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
    tmp = R.portable_tempdir("v26b_r0a_selftest_")
    try:
        # --- lineage rev3b -------------------------------------------------------------------
        lin = A.verify_lineage_rev3b()
        check(lin["amendment_rev3b"]["sha256"] == A.PIN_AMENDMENT_REV3B and lin["diagnostic_report"]["sha256"] == A.PIN_DIAG_REPORT, "rev3b lineage verified (amendment + diagnostic + negative evidence)")
        check(lin["r0_fail_negative_evidence"]["sha256"] == A.PIN_R0_FAIL_REPORT, "R0 FAIL report pinned as negative evidence")
        for attr in ("PIN_AMENDMENT_REV3B", "PIN_DIAG_REPORT", "PIN_R0_FAIL_REPORT"):
            old = getattr(A, attr)
            try:
                setattr(A, attr, "0" * 64)
                expect(A.verify_lineage_rev3b, A.R0aError, f"tampered {attr} -> lineage refused")
            finally:
                setattr(A, attr, old)
        am = json.loads(A.AMENDMENT_REV3B.read_text())
        check(am["parents_immutable"]["v26b_protocol.json"]["sha256"] == V2.PIN_PROTOCOL_JSON, "rev3b records rev3 parents")
        check("slew" in am["terminology_clarification"].lower() and "not dataset roles" in am["terminology_clarification"].lower(), "terminology: limiter anchors != dataset roles")
        check("16000" in am["next_stage_prepared_not_executed"] and "no assumptions" in am["next_stage_prepared_not_executed"], "next stage prepared in text only, with the 16000-anchor prerequisite")

        # --- full IK digests in the PRODUCTIVE path ------------------------------------------
        ik = A.verify_ik_caches_full_productive()
        check(all(len(d) == 64 for d in ik["digests"].values()), "productive path verifies FULL 64-hex cache digests")
        old = dict(A.IK_CACHE_DIGESTS_FULL)
        try:
            A.IK_CACHE_DIGESTS_FULL = {**old, "nominal": "0" * 64}
            expect(A.verify_ik_caches_full_productive, A.R0aError, "tampered full pin -> productive path refuses")
            expect(A.build_r0a_dataset, A.R0aError, "dataset build refuses under a tampered pin")
        finally:
            A.IK_CACHE_DIGESTS_FULL = old

        # --- dataset -------------------------------------------------------------------------
        role, ds = A.build_r0a_dataset()
        check(role["obs35"].shape == (500, 35) and ds["report"]["rows"] == 500 and ds["report"]["bitwise_duplicates"] == 0, "500 nominal-grid rows, 0 duplicates")
        check(set(np.unique(role["purpose"])) == {"ik_nominal_det"} and set(np.unique(role["job_id"])) == {"anchor_det_nominal"}, "single nominal det source, no alt-start")
        cc = ik["caches"]["nominal"]
        check(np.array_equal(role["actions"], cc.ik_action[cc.lookup(role["t_pre"])]), "labels bitwise == nominal cache u_IK")
        check(ds["train_idx"].size == 400 and ds["hold_idx"].size == 100, "July-style 400/100 diagnostic split")
        role2, ds2 = A.build_r0a_dataset()
        check(ds2["report"]["diagnostic_split_july_style"]["holdout_indices_sha256"] == ds["report"]["diagnostic_split_july_style"]["holdout_indices_sha256"], "split deterministic (same indices digest)")
        check("collect_teacher_dataset" in ds["report"]["deviation_from_july"] and "794-918" in ds["report"]["diagnostic_split_july_style"]["rule"], "July code cited exactly (dataset + split)")
        check("slew-limiter" in ds["report"]["terminology_note"], "terminology note in the dataset report")

        # --- fit smoke + determinism (tiny budget) -------------------------------------------
        init = V2.load_v1_init()
        tiny = {"epochs": 3, "batch_size": 256, "lr": 1e-4, "seed": 2026, "clip_weight": 1.0, "lambda_phi": 0.1, "lambda_anchor": 1e-3}
        s1, r1 = A.fit_r0a(init, role, budget=tiny)
        s2, r2 = A.fit_r0a(init, role, budget=tiny)
        check(r1["new_actor_digest"] == r2["new_actor_digest"], "fit deterministic (same digest)")
        check(r1["history"][-1]["loss"] < r1["history"][0]["loss"], "loss decreases")
        bad = {**role, "purpose": np.asarray(["ik_alt_stoch"] * 500, dtype=str)}
        expect(lambda: A.fit_r0a(init, bad, budget=tiny), Exception, "wrong purpose refused in the productive fit")

        # --- gate logic ----------------------------------------------------------------------
        exact = {**role, "actions": np.asarray(RF.numpy_mean(s1, role["obs35"]), dtype=np.float32)}
        g = A.evaluate_r0a_gate(init, s1, exact, ds)
        check(g["RMSE_vs_uIK_all_500"]["pass"] and g["Q3"]["pass"] and g["pass_r0a"], "gate passes when labels == model mean")
        check(isinstance(g["diagnostic_vs_V26_mean_rmse"], list) and len(g["diagnostic_vs_V26_mean_rmse"]) == 2, "V26-mean comparison recorded as diagnostics")
        check("not a criterion" in g["diagnostic_note"], "diagnostic note states: not a criterion")
        off = {**exact, "actions": exact["actions"] + np.float32(0.2)}
        g2 = A.evaluate_r0a_gate(init, s1, off, ds)
        check(g2["pass_r0a"] is False, "gate fails on a 0.2 offset")
        e = expect(lambda: A.assert_r0a_gate(g2), A.R0aError, "assert blocks on gate FAIL")
        check("RMSE_vs_uIK_all_500" in str(e) and "refused" in str(e), "refusal names the failed criterion")

        # --- export guard + tempdir export ---------------------------------------------------
        e = expect(lambda: A.run_r0a(authorized_stage=None, out_dir=tmp / "never"), A.R0aError, "R0a without the stage token refused")
        check("V26B-V2-R0A" in str(e) and not (tmp / "never").exists(), "refusal names the token; nothing written")
        expect(lambda: A.run_r0a(authorized_stage="V26B-V2-R0", out_dir=tmp / "never2"), A.R0aError, "wrong stage token refused")
        v1_state = V2.load_v1_init()
        fake_fit = {"tool": "test", "rows": 500, "budget": tiny, "var_role_per_joint": [0.1, 0.1], "init_actor_digest": V2.PIN_V1_ACTOR_DIGEST, "new_actor_digest": RF.actor_state_digest(v1_state), "aux_head": {}, "history": [{"epoch": 1, "loss": 1.0}]}
        fake_gate = {"pass_r0a": True, "RMSE_vs_uIK_all_500": {"pass": True}, "Q3": {"pass": True}}
        out = tmp / "r0a_export"
        runtime: dict = {}
        canonical = A.export_r0a(v1_state, fit_report=fake_fit, gate=fake_gate, dataset_report=ds["report"], lineage=lin, ik_digests=ik["digests"], out_dir=out, runtime_status=runtime)
        check((out / "rl_module" / "module_state.pkl").is_file() and (out / A.RECEIPT_NAME).is_file(), "export produced module + receipt")
        check(json.loads((out / A.RECEIPT_NAME).read_text()) == canonical, "canonical receipt == disk bytes")
        check(canonical["amendments"]["rev3b"]["sha256"] == A.PIN_AMENDMENT_REV3B and canonical["negative_evidence"]["r0_fail_report"]["sha256"] == A.PIN_R0_FAIL_REPORT, "receipt pins rev3b + negative evidence")
        check(canonical["ik_cache_digests_full"]["nominal"] == A.IK_CACHE_DIGESTS_FULL["nominal"], "receipt pins the FULL cache digests")
        check(runtime["lock_released"] is True, "lock released")
        expect(lambda: A.export_r0a(v1_state, fit_report=fake_fit, gate=fake_gate, dataset_report=ds["report"], lineage=lin, ik_digests=ik["digests"], out_dir=out), FileExistsError, "second export refused (no-clobber)")
        failing = {"pass_r0a": False, "RMSE_vs_uIK_all_500": {"pass": False}, "Q3": {"pass": True}}
        expect(lambda: A.export_r0a(v1_state, fit_report=fake_fit, gate=failing, dataset_report=ds["report"], lineage=lin, ik_digests=ik["digests"], out_dir=tmp / "never3"), A.R0aError, "export refused when the gate fails")
        check(not (tmp / "never3").exists(), "failed-gate export wrote nothing")

        # --- immutability --------------------------------------------------------------------
        check(C.sha256_file(A.AMENDMENT_REV3B) == A.PIN_AMENDMENT_REV3B, "rev3b amendment untouched")
        check(C.sha256_file(V2.V1_MODULE / "module_state.pkl") == V2.PIN_V1_MODULE_STATE, "V1 untouched")
        check(C.sha256_file(Path(R.TEACHER["module"]) / "module_state.pkl") == R.TEACHER["module_state_sha256"], "V26 untouched")
        check(C.sha256_file(A.R0_FAIL_REPORT) == A.PIN_R0_FAIL_REPORT, "frozen R0 FAIL report untouched")

        print(f"SELFTEST PASS ({CHECKS} checks)")
        return 0
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
