"""Self-test for the V26B V1 transplant (synthetic + real read-only + tempdir export).

No rollout, no fit, no dataset build, no production change; the only writes are
inside a portable tempdir."""

from __future__ import annotations

import json
import pickle
import shutil
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import v26b_student as VS  # noqa: E402
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


def synth_39_state(h: int = 8, seed: int = 7) -> dict[str, np.ndarray]:
    rng = np.random.default_rng(seed)
    W1 = rng.normal(0, 0.4, (h, 39)).astype(np.float32)
    b1 = rng.normal(0, 0.2, (h,)).astype(np.float32)
    W2 = rng.normal(0, 0.4, (h, h)).astype(np.float32)
    b2 = rng.normal(0, 0.2, (h,)).astype(np.float32)
    W3 = rng.normal(0, 0.4, (4, h)).astype(np.float32)  # log-std rows 2:4 NONZERO (state-dependent)
    b3 = rng.normal(0, 0.2, (4,)).astype(np.float32)
    return {
        "pi_encoder.0.weight": W1.copy(), "pi_encoder.0.bias": b1.copy(),
        "pi_encoder.2.weight": W2.copy(), "pi_encoder.2.bias": b2.copy(),
        "pi.0.0.weight": W1.copy(), "pi.0.0.bias": b1.copy(),
        "pi.0.2.weight": W2.copy(), "pi.0.2.bias": b2.copy(),
        "pi.1.weight": W3.copy(), "pi.1.bias": b3.copy(),
    }


def main() -> int:
    tmp = R.portable_tempdir("v26b_v1_selftest_")
    try:
        names35, names39, manifest_shas = VS.pinned_names()
        check(len(names35) == 35 and len(names39) == 39, "pinned manifests read")

        # --- synthetic transplant ------------------------------------------------------------
        src = synth_39_state()
        check(float(np.linalg.norm(src["pi.1.weight"][2:4])) > 0.1, "synthetic log-std rows are state-dependent (nonzero)")
        m = [0.11, -0.42, 0.07, 0.33]
        new, rep = VS.transplant_39_to_35(src, names39, names35, m)
        check(tuple(new.keys()) == RF.EXPECTED_KEY_ORDER, "10 keys in the original order")
        for a, c in RF.ALIAS_KEYS.items():
            check(np.array_equal(new[a], new[c]) and new[a] is not new[c], f"alias {a} bit-identical, separate array")
        keep = rep["feature_mapping"]["kept_39d_indices"]
        for j35, name in enumerate(names35):
            if j35 >= 2:
                check(np.array_equal(new["pi.0.0.weight"][:, j35], src["pi.0.0.weight"][:, keep[j35]]), f"column {name} transferred bit-identically")
        check(np.all(new["pi.0.0.weight"][:, 0:2] == 0.0), "clock columns exactly zero")
        exp_b1 = (src["pi.0.0.bias"].astype(np.float64) + src["pi.0.0.weight"][:, 2:6].astype(np.float64) @ np.asarray(m)).astype(np.float32)
        check(np.array_equal(new["pi.0.0.bias"], exp_b1), "mean-bias compensation exact: b1 = f32(f64(b1) + W_t @ m)")
        check(np.array_equal(new["pi.0.2.weight"], src["pi.0.2.weight"]) and np.array_equal(new["pi.0.2.bias"], src["pi.0.2.bias"]), "hidden layer bit-identical")
        check(np.array_equal(new["pi.1.weight"][:2], src["pi.1.weight"][:2]) and np.array_equal(new["pi.1.bias"][:2], src["pi.1.bias"][:2]), "mean head bit-identical")
        check(np.all(new["pi.1.weight"][2:4] == 0.0) and np.all(new["pi.1.bias"][2:4] == np.float32(np.log(0.005))), "log-std rows zero, bias exactly float32(ln 0.005)")
        check(all(np.asarray(v).dtype == np.float32 for v in new.values()), "all float32")
        check(rep["mean_bias_compensation"]["mean_targets_float64"] == [float(x) for x in m], "mean vector echoed exactly")
        check(len(rep["mean_bias_compensation"]["delta_float64"]["sha256_float64_bytes"]) == 64, "delta digest recorded")
        check(rep["new_actor_digest"] != rep["source_actor_digest"], "digests distinct")
        new2, rep2 = VS.transplant_39_to_35(src, names39, names35, m)
        check(rep2["new_actor_digest"] == rep["new_actor_digest"], "transplant deterministic (same digest)")
        z = VS.transplant_39_to_35(src, names39, names35, [0.0, 0.0, 0.0, 0.0])
        check(np.array_equal(z[0]["pi.0.0.bias"], src["pi.0.0.bias"]), "zero mean vector -> bias unchanged (compensation isolated)")

        # --- negative -----------------------------------------------------------------------
        bad35 = list(names35); bad35[5], bad35[6] = bad35[6], bad35[5]
        expect(lambda: VS.transplant_39_to_35(src, names39, bad35, m), VS.V1Error, "order-broken 35D names -> bijection refused")
        bad39 = list(names39); bad39[2], bad39[7] = bad39[7], bad39[2]
        expect(lambda: VS.transplant_39_to_35(src, bad39, names35, m), VS.V1Error, "targets not at 2:6 -> refused")
        expect(lambda: VS.transplant_39_to_35(src, names39, names35, [0.1, 0.2, 0.3]), VS.V1Error, "3-element mean vector refused")
        broken = dict(src); broken["pi_encoder.0.weight"] = src["pi_encoder.0.weight"] + np.float32(1e-3)
        expect(lambda: VS.transplant_39_to_35(broken, names39, names35, m), VS.V1Error, "broken alias refused")
        nokey = {k: v for k, v in src.items() if k != "pi.1.bias"}
        expect(lambda: VS.transplant_39_to_35(nokey, names39, names35, m), VS.V1Error, "missing key refused")
        f64 = {k: (np.asarray(v, dtype=np.float64) if k == "pi.0.2.weight" else v) for k, v in src.items()}
        f64["pi_encoder.2.weight"] = f64["pi.0.2.weight"]
        expect(lambda: VS.transplant_39_to_35(f64, names39, names35, m), VS.V1Error, "float64 tensor refused")

        # --- real read-only build ------------------------------------------------------------
        means = VS.anchor_target_means()
        check(means["rows_total"] == 1500 and all(means["per_trace"][s]["rows"] == 500 for s in R.STARTS), "anchor means over 1500 rows (500 x 3 pinned traces)")
        check(len(means["pooled_mean_float64"]) == 4 and all(np.isfinite(means["pooled_mean_float64"])), "pooled mean finite 4-vector")
        state, reports = VS.build_v1_state()
        check(reports["transplant"]["source_actor_digest"] == R.TEACHER["actor_digest"], "source digest == pinned V26")
        struct = RF.validate_init_state(state, expected_actor_digest=None)
        check(struct["clock_columns_zero"] and struct["actor_digest"] == reports["transplant"]["new_actor_digest"], "full F2R structural battery passes on the real transplanted state")
        inv = RF.invariance_test(state, np.random.default_rng(3).standard_normal((32, 35)).astype(np.float32))
        check(inv["bit_identical"], "clock invariance bit-identical on the real state")
        fid = reports["fidelity"]
        check(set(fid["per_start"]) == set(R.STARTS) and all(np.isfinite(fid["per_start"][s]["per_joint"]["knee"]["rmse"]) for s in R.STARTS), "fidelity metrics finite for all 3 anchors")
        print("  fidelity overall:", json.dumps(fid["overall"]))
        state2, reports2 = VS.build_v1_state()
        check(reports2["transplant"]["new_actor_digest"] == reports["transplant"]["new_actor_digest"], "real build deterministic (same digest)")

        # --- guard + export to tempdir -------------------------------------------------------
        e = expect(lambda: VS.run_v1(authorized_stage=None, out_dir=tmp / "never"), VS.V1Error, "export without the stage token refused")
        check("V26B-V1" in str(e) and not (tmp / "never").exists(), "refusal names the token; nothing written")
        expect(lambda: VS.run_v1(authorized_stage="V26B-ANCHORS", out_dir=tmp / "never2"), VS.V1Error, "wrong stage token refused")
        out = tmp / "v1_export"
        runtime: dict = {}
        canonical = VS.run_v1(authorized_stage="V26B-V1", out_dir=out, runtime_status=runtime)
        mod = out / "rl_module"
        check(sorted(p.name for p in mod.iterdir()) == ["actor_feature_manifest.json", "class_and_ctor_args.pkl", "metadata.json", "module_state.pkl"], "module dir has exactly the 4 files")
        check((out / VS.RECEIPT_NAME).is_file() and not any(p.name.startswith(".staging-") for p in out.parent.iterdir()), "receipt present; staging gone")
        check(not (out.parent / f".{out.name}.lock").exists() and runtime["lock_released"] is True, "export lock released")
        disk = json.loads((out / VS.RECEIPT_NAME).read_text())
        check(disk == canonical, "returned receipt == receipt bytes on disk (canonical)")
        for name, sha in canonical["output_files_sha256"].items():
            check(C.sha256_file(mod / name) == sha, f"output digest bound: {name}")
        sys.path.insert(0, str(R.BASELINE_DIR))
        import warm_start as W
        reloaded = W.load_module_state(mod)
        st2 = RF.validate_init_state(reloaded, expected_actor_digest=canonical["transplant"]["new_actor_digest"])
        check(st2["actor_digest"] == canonical["transplant"]["new_actor_digest"], "final module reload: digest == receipt")
        with (mod / "class_and_ctor_args.pkl").open("rb") as fh:
            cc = pickle.load(fh)
        _, kw = cc["ctor_args_and_kwargs"]
        check(tuple(kw["observation_space"].shape) == (84,) and kw["model_config"]["n_actor"] == 35 and kw["model_config"]["n_full"] == 84, "ctor args: obs (84,), n_actor 35, n_full 84")
        with (Path(R.INIT_PRIMARY["module"]) / "class_and_ctor_args.pkl").open("rb") as fh:
            jul = pickle.load(fh)
        _, jkw = jul["ctor_args_and_kwargs"]
        check(tuple(jkw["observation_space"].shape) == tuple(kw["observation_space"].shape) and jkw["model_config"]["n_actor"] == kw["model_config"]["n_actor"], "ctor equivalent to the proven deployable 35D shape (JUL, informational)")
        mf = json.loads((mod / "actor_feature_manifest.json").read_text())
        check(mf["actor_feature_names"] == names35 and mf["module_state_sha256"] == C.sha256_file(mod / "module_state.pkl"), "35D manifest bound to the module state")
        check("NOT the operational sigma" in mf["sigma_note"], "manifest states the sigma placeholder is not operational")
        check(canonical["coverage_gate"]["sha256"] == VS.COVERAGE_SHA256 and canonical["source"]["files_sha256"]["module_state.pkl"] == R.TEACHER["module_state_sha256"], "receipt binds coverage JSON and source pins")
        expect(lambda: VS.run_v1(authorized_stage="V26B-V1", out_dir=out), FileExistsError, "second export to the same dir refused (no-clobber)")

        # --- source immutability -------------------------------------------------------------
        post = VS.source_files_table(Path(R.TEACHER["module"]))
        check(post["module_state.pkl"] == R.TEACHER["module_state_sha256"], "V26 source module untouched after everything")

        print(f"SELFTEST PASS ({CHECKS} checks)")
        return 0
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
