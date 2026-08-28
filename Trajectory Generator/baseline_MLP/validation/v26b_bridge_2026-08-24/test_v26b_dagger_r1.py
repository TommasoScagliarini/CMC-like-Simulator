"""Self-test for DAgger round 1 (rev3d): lineage, dataset, fit equivalence, gate, export."""

from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import v26b_dagger_r1 as D1  # noqa: E402
import v26b_r0a as A  # noqa: E402
import v26b_r0a_rollout as RO  # noqa: E402
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
    tmp = R.portable_tempdir("v26b_d1_selftest_")
    try:
        # --- lineage -------------------------------------------------------------------------
        lin = D1.verify_lineage_r1()
        check(lin["amendment_rev3d"]["sha256"] == D1.PIN_AMENDMENT_REV3D and lin["rollout_trace"]["sha256"] == D1.PIN_ROLLOUT_TRACE, "rev3d lineage verified (chain + trace + receipt + addendum)")
        for attr in ("PIN_AMENDMENT_REV3D", "PIN_ROLLOUT_TRACE", "PIN_ROLLOUT_ADDENDUM"):
            old = getattr(D1, attr)
            try:
                setattr(D1, attr, "0" * 64)
                expect(D1.verify_lineage_r1, D1.DaggerError, f"tampered {attr} -> refused")
            finally:
                setattr(D1, attr, old)
        am = json.loads(D1.AMENDMENT_REV3D.read_text())
        check("ONLY the rev3c clause" in am["supersession"]["target"], "rev3d supersedes only the 500/500 clause")
        check(any("UNDECIDED" in d for d in am["preregistered_deviations_from_july"]), "rev3d states sigma 0.005 undecided/unused")
        check(am["july_protocol_citation"]["r2_budget_verbatim"]["batch_size"] == 64 and am["july_protocol_citation"]["r2_budget_verbatim"]["anchor_weight"] == 1e-05, "July r2 budget cited verbatim")

        # --- dataset (real, read-only) -------------------------------------------------------
        data, rep = D1.build_r1_dataset()
        n = int(data["obs35"].shape[0])
        check(rep["roles"]["ik_nominal_det"] == 500 and rep["roles"]["ik_onpolicy_det"] == 493, "roles 500 BC + 493 on-policy")
        check(n == 993 - rep["collisions"]["count"], "rows_after_dedup = 993 - collisions")
        check(all(c["labels_identical"] for c in rep["collisions"]["detail"]), "every collision has identical labels (no conflict)")
        keys = {data["obs35"][i].tobytes() for i in range(n)}
        check(len(keys) == n, "aggregate bitwise-unique after dedup")
        check(set(np.unique(data["purpose"])) <= D1.ALLOWED_PURPOSES, "only the two round-1 purposes")
        check(np.all(np.abs(data["actions"]) <= 1.0) and np.all(np.isfinite(data["actions"])), "u_IK labels finite in [-1, 1]")
        check(rep["on_policy"]["includes_last_row"] is True and "AFTER" in rep["on_policy"]["note"], "last row included; guard-after-action documented")
        check("NO V26-anchor reference" in rep["no_truncation"] or "No V26-anchor" in rep["no_truncation"], "no truncation, no V26 reference")
        bad = {k: (np.asarray(["det"] * n, dtype=str) if k == "purpose" else v) for k, v in data.items()}
        expect(lambda: D1.fit_dagger_r1(V2.load_v1_init(), bad, budget={"epochs": 1, "batch_size": 256, "lr": 1e-4, "seed": 2026, "clip_weight": 1.0, "lambda_phi": 0.1, "lambda_anchor": 1e-3}), Exception, "forbidden purpose refused by the fit contract")

        # --- fit equivalence with the proven R0a loop ----------------------------------------
        init = D1.load_r0a_init()
        check(RF.actor_state_digest(init) == RO.PIN_R0A_ACTOR_DIGEST, "init = R0a exactly (digest pinned)")
        role500, _ = A.build_r0a_dataset()
        tiny = {"epochs": 3, "batch_size": 256, "lr": 1e-4, "seed": 2026, "clip_weight": 1.0, "lambda_phi": 0.1, "lambda_anchor": 1e-3}
        s_ref, r_ref = A.fit_r0a(init, role500, budget=tiny)
        s_new, r_new = D1.fit_dagger_r1(init, role500, budget=tiny)
        check(r_ref["new_actor_digest"] == r_new["new_actor_digest"], "fit_dagger_r1 bit-identical to the proven fit_r0a on the same single-purpose data")
        check("UNDECIDED" in r_new["sigma_note"], "fit report carries the sigma-undecided note")

        # --- gate logic ----------------------------------------------------------------------
        st, _ = D1.fit_dagger_r1(init, data, budget=tiny)
        exact = {**data, "actions": np.asarray(RF.numpy_mean(st, data["obs35"]), dtype=np.float32)}
        g = D1.evaluate_r1_gate(init, st, exact)
        check(g["pass_r1"] and all(g["per_role"][k]["pass"] for k in ("aggregate", "bc_nominal", "on_policy")), "gate passes when labels == model mean (all roles)")
        check("leakage-free" in g["holdout_diagnostic"]["split"] and "UNDECIDED" in g["sigma_note"], "holdout declared leakage-free; sigma note present")
        off = {**exact, "actions": exact["actions"].copy()}
        mask = np.asarray(off["purpose"]) == "ik_onpolicy_det"
        off["actions"][mask] += np.float32(0.2)
        g2 = D1.evaluate_r1_gate(init, st, off)
        check(g2["pass_r1"] is False and g2["per_role"]["on_policy"]["pass"] is False and g2["per_role"]["bc_nominal"]["pass"] is True, "per-role gate isolates the failing role")
        e = expect(lambda: D1.assert_r1_gate(g2), D1.DaggerError, "assert blocks on a role FAIL")
        check("on_policy" in str(e), "refusal names the failing role")

        # --- export guard + tempdir ----------------------------------------------------------
        e = expect(lambda: D1.run_r1(authorized_stage=None, out_dir=tmp / "never"), D1.DaggerError, "round 1 without the stage token refused")
        check("V26B-DAGGER-R1" in str(e), "refusal names the token")
        fake_fit = {"tool": "test", "rows": n, "budget": tiny, "var_per_joint": [0.1, 0.1], "init_actor_digest": RO.PIN_R0A_ACTOR_DIGEST, "new_actor_digest": RF.actor_state_digest(init), "sigma_note": D1.SIGMA_NOTE, "aux_head": {}, "history": [{"epoch": 1, "loss": 1.0}]}
        fake_gate = {"pass_r1": True, "per_role": {"aggregate": {"pass": True}, "bc_nominal": {"pass": True}, "on_policy": {"pass": True}}, "Q3": {"pass": True}}
        out = tmp / "r1_export"
        runtime: dict = {}
        canonical = D1.export_r1(init, fit_report=fake_fit, gate=fake_gate, dataset_report=rep, dataset_files={}, lineage=lin, out_dir=out, runtime_status=runtime)
        check((out / "rl_module" / "module_state.pkl").is_file() and json.loads((out / D1.RECEIPT_NAME).read_text()) == canonical, "export produced module + canonical receipt")
        check(canonical["amendments"]["rev3d"] == D1.PIN_AMENDMENT_REV3D and canonical["on_policy_source"]["trace_sha256"] == D1.PIN_ROLLOUT_TRACE, "receipt pins rev3d + on-policy trace")
        check("UNDECIDED" in canonical["sigma_placeholder"]["statement"], "receipt: sigma undecided/unused")
        check(runtime["lock_released"] is True, "lock released")
        expect(lambda: D1.export_r1(init, fit_report=fake_fit, gate=fake_gate, dataset_report=rep, dataset_files={}, lineage=lin, out_dir=out), FileExistsError, "second export refused")
        failing = {"pass_r1": False, "per_role": {"aggregate": {"pass": False}, "bc_nominal": {"pass": True}, "on_policy": {"pass": True}}, "Q3": {"pass": True}}
        expect(lambda: D1.export_r1(init, fit_report=fake_fit, gate=failing, dataset_report=rep, dataset_files={}, lineage=lin, out_dir=tmp / "never2"), D1.DaggerError, "export refused on gate FAIL")
        check(not (tmp / "never2").exists(), "failed-gate export wrote nothing")

        # --- frozen artefacts untouched ------------------------------------------------------
        check(C.sha256_file(RO.JOB_DIR / RO.RECEIPT_NAME) == D1.PIN_ROLLOUT_RECEIPT, "rollout receipt untouched")
        check(C.sha256_file(A.OUT_R0A / "rl_module" / "module_state.pkl") == "7f1ba2eda0b6b426eff07a97ea826ac6077aaeb8d92fdff1bc06429a3925830e", "R0a module untouched")
        check(C.sha256_file(D1.AMENDMENT_REV3D) == D1.PIN_AMENDMENT_REV3D, "rev3d untouched")

        print(f"SELFTEST PASS ({CHECKS} checks)")
        return 0
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
