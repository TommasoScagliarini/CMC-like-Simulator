"""Self-test for V26B V2/R0 (mirror equivalence, contracts, criteria, export; no real fit)."""

from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import v26b_v2 as V2  # noqa: E402
import v26b_anchors as VA  # noqa: E402
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


def synth_init_35(h: int = 8, seed: int = 11) -> dict[str, np.ndarray]:
    rng = np.random.default_rng(seed)
    W1 = rng.normal(0, 0.4, (h, 35)).astype(np.float32); W1[:, 0:2] = 0.0
    b1 = rng.normal(0, 0.2, (h,)).astype(np.float32)
    W2 = rng.normal(0, 0.4, (h, h)).astype(np.float32); b2 = rng.normal(0, 0.2, (h,)).astype(np.float32)
    W3 = np.concatenate([rng.normal(0, 0.4, (2, h)).astype(np.float32), np.zeros((2, h), np.float32)])
    b3 = np.concatenate([rng.normal(0, 0.2, (2,)).astype(np.float32), np.full(2, np.log(0.005), np.float32)])
    return {"pi_encoder.0.weight": W1.copy(), "pi_encoder.0.bias": b1.copy(), "pi_encoder.2.weight": W2.copy(), "pi_encoder.2.bias": b2.copy(),
            "pi.0.0.weight": W1.copy(), "pi.0.0.bias": b1.copy(), "pi.0.2.weight": W2.copy(), "pi.0.2.bias": b2.copy(),
            "pi.1.weight": W3.copy(), "pi.1.bias": b3.copy()}


def synth_role(n: int, seed: int, *, purposes: list[str], seeds_value: int = 123) -> dict[str, np.ndarray]:
    rng = np.random.default_rng(seed)
    return {"obs35": rng.normal(0, 0.5, (n, 35)).astype(np.float32),
            "actions": rng.uniform(-0.9, 0.9, (n, 2)).astype(np.float32),
            "clock": rng.normal(0, 0.7, (n, 2)).astype(np.float32),
            "seed": np.full(n, seeds_value, dtype=np.int64),
            "purpose": np.asarray([purposes[i % len(purposes)] for i in range(n)], dtype=str)}


def main() -> int:
    tmp = R.portable_tempdir("v26b_v2_selftest_")
    try:
        # --- lineage verification (real, read-only) -----------------------------------------
        lin = V2.verify_lineage()
        check(lin["protocol_json"]["sha256"] == V2.PIN_PROTOCOL_JSON and lin["protocol_md"]["sha256"] == V2.PIN_PROTOCOL_MD, "rev3 parents byte-identical to the V1-pinned hashes")
        check(lin["amendment_rev3a"]["sha256"] == V2.PIN_AMENDMENT, "rev3a amendment file pinned")
        am = json.loads(V2.AMENDMENT_FILE.read_text())
        check(am["parents_immutable"]["v26b_protocol.json"]["sha256"] == V2.PIN_PROTOCOL_JSON, "amendment records its parents' hashes")
        old = V2.PIN_PROTOCOL_JSON
        try:
            V2.PIN_PROTOCOL_JSON = "0" * 64
            expect(V2.verify_lineage, V2.V2Error, "wrong protocol pin -> lineage refused")
        finally:
            V2.PIN_PROTOCOL_JSON = old
        old = V2.PIN_AMENDMENT
        try:
            V2.PIN_AMENDMENT = "0" * 64
            expect(V2.verify_lineage, V2.V2Error, "wrong amendment pin -> lineage refused")
        finally:
            V2.PIN_AMENDMENT = old

        # --- mirror bit-equivalence with the immutable F2R fit -------------------------------
        init = synth_init_35()
        budget = {"epochs": 3, "batch_size": 16, "lr": 1e-4, "seed": 2026, "clip_weight": 1.0, "lambda_phi": 0.1, "lambda_anchor": 1e-3}
        task_num = synth_role(48, 21, purposes=["anchor", "det"])
        pres_num = synth_role(24, 22, purposes=["det"])
        task_v2 = {**task_num, "purpose": np.asarray(["ik_alt_stoch"] * 48, dtype=str)}
        pres_v2 = {**pres_num, "purpose": np.asarray(["anchor_nom_stoch"] * 24, dtype=str)}
        s_ref, r_ref = RF.fit_student_preserving(init, task_num, pres_num, budget=budget, beta=1.0)
        s_mir, r_mir = V2.fit_r0(init, task_v2, pres_v2, budget=budget, beta=1.0)
        check(r_ref["new_actor_digest"] == r_mir["new_actor_digest"], "mirror fit: BIT-IDENTICAL actor digest vs f2r_refit.fit_student_preserving")
        check(all(np.array_equal(np.asarray(s_ref[k]), np.asarray(s_mir[k])) for k in RF.EXPECTED_KEY_ORDER), "mirror fit: every tensor bit-identical")
        check([h["loss"] for h in r_ref["history"]] == [h["loss"] for h in r_mir["history"]], "mirror fit: identical loss history")
        check(r_mir["normalisation"]["var_task_per_joint"] == r_ref["normalisation"]["var_task_per_joint"], "variances fixed a priori, identical")
        check("criteria" in r_mir and "Q1" in r_mir["criteria"] and "P1" in r_ref["criteria"], "criteria: Q for the mirror, P for T1R")

        # --- V2 contract negatives -----------------------------------------------------------
        bad = {**task_v2, "seed": np.full(48, 124, dtype=np.int64)}
        expect(lambda: V2.assert_r0_contract(bad, pres_v2), Exception, "seed 124 refused in R0 roles")
        bad = {**task_v2, "seed": np.full(48, 125, dtype=np.int64)}
        expect(lambda: V2.assert_r0_contract(bad, pres_v2), Exception, "held-out seed 125 refused")
        bad = {**task_v2, "purpose": np.asarray(["det"] * 48, dtype=str)}
        expect(lambda: V2.assert_r0_contract(bad, pres_v2), V2.V2Error, "T1R purpose refused in a V2 role")
        bad = {**pres_v2, "purpose": np.asarray(["ik_alt_stoch"] * 24, dtype=str)}
        expect(lambda: V2.assert_r0_contract(task_v2, bad), V2.V2Error, "role-swapped purpose refused")
        bad = {**task_v2, "obs35": task_v2["obs35"][:, :34]}
        expect(lambda: V2.assert_r0_contract(bad, pres_v2), V2.V2Error, "wrong width refused")
        bad = {**task_v2, "actions": task_v2["actions"].copy()}
        bad["actions"][0, 0] = np.nan
        expect(lambda: V2.assert_r0_contract(bad, pres_v2), V2.V2Error, "non-finite refused")
        empty = {k: v[:0] for k, v in pres_v2.items()}
        expect(lambda: V2.assert_r0_contract(task_v2, empty), V2.V2Error, "empty role refused")
        check(V2.assert_r0_contract(task_v2, pres_v2)["task"]["rows"] == 48, "valid V2 contract accepted (seed 123 det anchors allowed)")
        t1000 = {**task_v2, "seed": np.full(48, 1000, dtype=np.int64)}
        check(V2.assert_r0_contract(t1000, pres_v2)["task"]["seeds"] == [1000], "anchor seed 1000 accepted")

        # --- Q criteria ----------------------------------------------------------------------
        st = s_mir
        exact_task = {**task_v2, "actions": np.asarray(RF.numpy_mean(st, task_v2["obs35"]), dtype=np.float32)}
        exact_pres = {**pres_v2, "actions": np.asarray(RF.numpy_mean(st, pres_v2["obs35"]), dtype=np.float32)}
        q = V2.evaluate_q_criteria(init, st, exact_task, exact_pres)
        check(q["Q1"]["pass"] and q["Q2"]["pass"] and q["Q3"]["pass"] and q["pass_r0"], "Q all pass when targets == model mean")
        check(q["Q2b"]["applicable"] is False and q["Q2b"]["amendment_sha256"] == V2.PIN_AMENDMENT, "Q2b not applicable at R0, bound to the rev3a amendment sha")
        off_pres = {**exact_pres, "actions": exact_pres["actions"] + np.float32(0.2)}
        q2 = V2.evaluate_q_criteria(init, st, exact_task, off_pres)
        check(q2["Q1"]["pass"] is False and q2["pass_r0"] is False, "Q1 fails when anchors deviate by 0.2")
        e = expect(lambda: V2.assert_q_r0({"criteria": q2}), V2.V2Error, "assert_q_r0 blocks on Q1 FAIL")
        check("Q1" in str(e) and "refused" in str(e), "refusal names the failed criterion")

        # --- real dataset build (read-only; revalidates the whole collection) ----------------
        task, pres, ds_report = V2.build_r0_dataset()
        check(task["obs35"].shape == (12876, 35) and pres["obs35"].shape == (6438, 35), "role counts 12876/6438 (start split)")
        check(set(np.unique(task["start"])) == {"minus020", "plus020"} and set(np.unique(pres["start"])) == {"nominal"}, "task = alt starts only; preservation = nominal only")
        check(ds_report["cross_role_collisions"]["count"] == 0, "0 cross-role bitwise collisions")
        check(np.all(np.abs(task["actions"]) <= 1.0), "u_IK labels inside [-1, 1]")
        rad = task["clock"].astype(np.float64) ** 2
        check(np.max(np.abs(rad[:, 0] + rad[:, 1] - 1.0)) < 1e-5, "prescribed clock rows on the unit circle")
        check(sorted({int(s) for s in task["seed"]}) == [123, 1000, 1001, 1002, 1003], "task seeds = det anchor 123 + anchor-only 1000-1003")
        # u_IK spot check vs direct cache lookup
        caches = V2.load_ik_caches()
        i = 137
        cc = caches[str(task["start"][i])]
        check(np.array_equal(task["actions"][i], cc.ik_action[cc.lookup(np.asarray([task["t_pre"][i]]))[0]]), "u_IK row equals the pinned cache lookup")
        V2.assert_r0_contract(task, pres)
        check(True, "real roles satisfy the V2 contract")

        # --- export path (tempdir; synthetic PASS report around the real V1 init state) ------
        v1_state = V2.load_v1_init()
        check(RF.actor_state_digest(v1_state) == V2.PIN_V1_ACTOR_DIGEST, "V1 init loads and matches its pinned digest")
        fake_hist = [{"epoch": 1, "loss": 1.0, "task_norm": 1.0, "pres_norm": 1.0}]
        fake_report = {"criteria": {"pass_r0": True, "Q1": {"pass": True}, "Q2": {"pass": True}, "Q3": {"pass": True}, "Q2b": {"applicable": False}},
                       "new_actor_digest": RF.actor_state_digest(v1_state), "history": fake_hist,
                       "tool": "test", "variant": "test", "beta": 1.0, "budget": budget, "optimizer_steps": 1, "epochs_run": 1,
                       "rows": {"task": 1, "preservation": 1}, "contract": {}, "normalisation": {}, "loss_form": "test",
                       "init_actor_digest": V2.PIN_V1_ACTOR_DIGEST, "aux_head": {}}
        out = tmp / "r0_export"
        runtime: dict = {}
        canonical = V2.export_r0(v1_state, fit_report=fake_report, dataset_receipt={"files": {}}, lineage=lin, out_dir=out, runtime_status=runtime)
        check((out / "rl_module" / "module_state.pkl").is_file() and (out / V2.RECEIPT_NAME).is_file(), "export produced module + receipt")
        check(json.loads((out / V2.RECEIPT_NAME).read_text()) == canonical, "canonical receipt == disk bytes")
        check(canonical["protocol_parents_immutable"]["v26b_protocol.json"] == V2.PIN_PROTOCOL_JSON and canonical["amendment_rev3a"]["sha256"] == V2.PIN_AMENDMENT, "receipt pins rev3 parents AND the separate amendment")
        check(canonical["init"]["v1_receipt_sha256"] == V2.PIN_V1_RECEIPT, "receipt pins the V1 receipt")
        check(runtime["lock_released"] is True and not (out.parent / f".{out.name}.lock").exists(), "lock released")
        expect(lambda: V2.export_r0(v1_state, fit_report=fake_report, dataset_receipt={}, lineage=lin, out_dir=out), FileExistsError, "second export refused (no-clobber)")
        failing = {**fake_report, "criteria": {"pass_r0": False, "Q1": {"pass": False}, "Q2": {"pass": True}, "Q3": {"pass": True}}}
        expect(lambda: V2.export_r0(v1_state, fit_report=failing, dataset_receipt={}, lineage=lin, out_dir=tmp / "never"), V2.V2Error, "export refused when Q fails")
        check(not (tmp / "never").exists(), "failed-Q export wrote nothing")

        # --- guard + immutability ------------------------------------------------------------
        e = expect(lambda: V2.run_r0(authorized_stage=None, out_dir=tmp / "never2"), V2.V2Error, "R0 without the stage token refused")
        check("V26B-V2-R0" in str(e), "refusal names the token")
        check(C.sha256_file(VA.PROTOCOL_JSON) == V2.PIN_PROTOCOL_JSON and C.sha256_file(VA.PROTOCOL_MD) == V2.PIN_PROTOCOL_MD, "rev3 parents still byte-identical after the suite")
        check(C.sha256_file(V2.V1_MODULE / "module_state.pkl") == V2.PIN_V1_MODULE_STATE, "V1 module untouched")
        check(C.sha256_file(Path(R.TEACHER["module"]) / "module_state.pkl") == R.TEACHER["module_state_sha256"], "V26 source untouched")

        print(f"SELFTEST PASS ({CHECKS} checks)")
        return 0
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
