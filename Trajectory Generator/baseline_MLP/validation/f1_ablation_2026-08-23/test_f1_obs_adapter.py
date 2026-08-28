"""Self-test of f1_obs_adapter (synthetic data; the two pinned manifests are read)."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import f1_common as F1  # noqa: E402
import f0_common as C  # noqa: E402
import f1_obs_adapter as OA  # noqa: E402

CHECKS = 0


def check(cond: bool, label: str) -> None:
    global CHECKS
    CHECKS += 1
    if not cond:
        raise AssertionError(f"CHECK FAILED: {label}")


def expect_error(fn, label: str, exc=OA.AdapterError) -> None:
    try:
        fn()
    except exc:
        check(True, label)
        return
    raise AssertionError(f"expected {exc.__name__}: {label}")


class FakeClock:
    def __init__(self, hs, offset=0.0):
        self._hs = np.asarray(hs, dtype=float)
        self.available = self._hs.size >= 2
        self._offset = offset

    @property
    def heel_strike_times(self):
        return self._hs

    def _raw(self, t):
        hs = self._hs
        if t < hs[0]:
            return (t - hs[0]) / (hs[1] - hs[0])
        if t >= hs[-1]:
            return (t - hs[-1]) / (hs[-1] - hs[-2])
        k = int(np.searchsorted(hs, t, side="right")) - 1
        return (t - hs[k]) / (hs[k + 1] - hs[k])

    def phase(self, t):
        x = self._raw(t) - self._offset
        return float(x - np.floor(x))

    def raw_phase(self, t):
        x = self._raw(t)
        return float(x - np.floor(x))

    def local_period(self, t):
        return float(self._hs[1] - self._hs[0])


class FakeTarget:
    def __init__(self, clock, available=True, drop_key=None):
        self._clock = clock
        self.available = available
        self._drop = drop_key

    def get(self, t):
        ph = self._clock.raw_phase(t)
        q = {"pros_knee_angle": -0.5 - 0.4 * np.cos(2 * np.pi * ph), "pros_ankle_angle": 0.1 + 0.2 * np.sin(2 * np.pi * ph)}
        qd = {"pros_knee_angle": 0.4 * 2 * np.pi * np.sin(2 * np.pi * ph), "pros_ankle_angle": 0.2 * 2 * np.pi * np.cos(2 * np.pi * ph)}
        if self._drop:
            q.pop(self._drop, None)
        return q, qd, {"pros_knee_angle": ph, "pros_ankle_angle": ph}

    def summary(self):
        return {"available": self.available, "cycle_count": 3}


def main() -> int:
    names35, sha35 = OA.read_manifest_names(C.ACTOR_MANIFEST_35, expected_sha256=C.ACTOR_MANIFEST_35_SHA256, sha256_fn=C.sha256_file)
    names39, sha39 = OA.read_manifest_names(C.ACTOR_MANIFEST_39, expected_sha256=C.ACTOR_MANIFEST_39_SHA256, sha256_fn=C.sha256_file)
    check(len(names35) == 35 and len(names39) == 39, "pinned manifests widths")
    spec = OA.derive_insertion(names35, names39, manifest35_sha256=sha35, manifest39_sha256=sha39)
    check(spec.index == 2 and spec.count == 4 and spec.stop == 6, "insertion block at 2:6")
    check(spec.inserted_names == OA.TARGET_FEATURE_NAMES, "inserted names are the four healthy_* targets")
    check(names39[:2] == names35[:2] == list(OA.CLOCK_FEATURE_NAMES), "clock pair at 0:2 in both manifests")
    check(spec.to_dict()["manifest35_sha256"] == C.ACTOR_MANIFEST_35_SHA256, "manifest sha recorded")
    expect_error(lambda: OA.read_manifest_names(C.ACTOR_MANIFEST_35, expected_sha256="0" * 64, sha256_fn=C.sha256_file), "wrong manifest pin refused")
    # negative derivations
    bad = list(names39)
    bad[2], bad[6] = bad[6], bad[2]
    expect_error(lambda: OA.derive_insertion(names35, bad), "non-contiguous block refused")
    expect_error(lambda: OA.derive_insertion(names35, [n if n != names39[3] else "other_feature" for n in names39]), "wrong inserted name refused")
    expect_error(lambda: OA.derive_insertion(names35[:-1], names39), "missing 35D name refused")
    expect_error(lambda: OA.derive_insertion(names35, names39 + ["extra"]), "extra name refused")
    swapped = names35[2:] + names35[:2]
    expect_error(lambda: OA.derive_insertion(swapped, [n for n in names39 if n not in OA.TARGET_FEATURE_NAMES][2:] + names39[:2]), "clock not at 0:2 refused")
    # projection / insertion round trips
    rng = np.random.default_rng(0)
    for width, dtype in ((35, np.float64), (35, np.float32), (84, np.float64), (84, np.float32)):
        obs = rng.standard_normal(width).astype(dtype)
        vals = rng.standard_normal(4)
        obs_w = OA.insert_targets(obs, vals, spec)
        check(obs_w.shape == (width + 4,) and obs_w.dtype == dtype, f"insert keeps dtype/width {width}/{dtype.__name__}")
        check(np.array_equal(obs_w[2:6], np.asarray(vals, dtype=dtype)), "inserted values cast to the env dtype")
        proj = OA.project39to35(obs_w, spec)
        check(proj.dtype == dtype and np.array_equal(proj, obs), "project39to35 exact round trip")
        OA.assert_projection_exact(obs_w, obs, spec)
        check(True, "assert_projection_exact passes on genuine pair")
        tampered = obs_w.copy()
        tampered[7] = tampered[7] + np.asarray(1e-6, dtype=dtype)
        expect_error(lambda: OA.assert_projection_exact(tampered, obs, spec), "tampered projection refused")
        expect_error(lambda: OA.assert_projection_exact(obs_w.astype(np.float64 if dtype == np.float32 else np.float32), obs, spec), "dtype mismatch refused")
    batch = rng.standard_normal((5, 35)).astype(np.float32)
    ins = OA.insert_targets(batch, rng.standard_normal((5, 4)), spec)
    check(ins.shape == (5, 39) and np.array_equal(OA.project39to35(ins, spec), batch), "batched insertion/projection")
    expect_error(lambda: OA.insert_targets(batch, np.zeros((5, 3)), spec), "wrong target count refused")
    expect_error(lambda: OA.project39to35(np.zeros(5), spec), "too short vector refused")
    # reconstructor with synthetic objects
    clock = FakeClock([10.0, 11.0, 12.0, 13.0])
    target = FakeTarget(clock)
    recon = OA.PrescribedTargetReconstructor(clock, target, provenance={"builder": "test"})
    t = 11.25
    vals = recon.targets(t)
    q, qd, _ = target.get(t)
    check(vals.shape == (4,) and vals.dtype == np.float64, "targets shape/dtype")
    check(np.array_equal(vals, np.array([q["pros_knee_angle"], qd["pros_knee_angle"], q["pros_ankle_angle"], qd["pros_ankle_angle"]])), "targets follow env order knee q, knee qdot, ankle q, ankle qdot")
    s, c = recon.clock_sin_cos(t)
    check(abs(s - np.sin(2 * np.pi * 0.25)) < 1e-12 and abs(c - np.cos(2 * np.pi * 0.25)) < 1e-12, "clock sin/cos from phase")
    prov = recon.provenance
    check(prov["clock_n_cycles"] == 3 and prov["builder"] == "test" and prov["feature_order"] == list(OA.TARGET_FEATURE_NAMES), "provenance")
    expect_error(lambda: OA.PrescribedTargetReconstructor(FakeClock([10.0]), target), "unavailable clock refused")
    expect_error(lambda: OA.PrescribedTargetReconstructor(clock, FakeTarget(clock, available=False)), "unavailable target refused")
    expect_error(lambda: OA.PrescribedTargetReconstructor(clock, FakeTarget(clock, drop_key="pros_ankle_angle")).targets(t), "missing coordinate refused (no fallback)")
    # adapter modes
    obs84 = rng.standard_normal(84).astype(np.float32)
    obs84[0], obs84[1] = 0.0, 1.0
    ad = OA.ObservationAdapter(spec, "aiso4", recon)
    out = ad.adapt(obs84, t)
    check(out.shape == (88,) and out.dtype == np.float32 and np.array_equal(OA.project39to35(out, spec), obs84), "aiso4 inserts 4 targets, projection exact")
    check(np.array_equal(out[2:6], vals.astype(np.float32)) and out[0] == 0.0 and out[1] == 1.0, "aiso4 leaves the clock pair as B sees it")
    ad.adapt(obs84, t + 0.01)
    summ = ad.summary()
    check(summ["steps"] == 2 and summ["projection_assert_count"] == 2 and summ["all_steps_asserted"] and summ["insertion"]["insert_index"] == 2, "adapter summary counts")
    check(len(ad.trace()) == 2 and ad.trace()[1]["t_pre"] == t + 0.01 and ad.trace()[1]["obs_width_out"] == 88, "adapter trace rows")
    pt = OA.ObservationAdapter(spec, "passthrough", None)
    same = pt.adapt(obs84, t)
    check(same is obs84 and pt.summary()["projection_assert_count"] == 1, "passthrough returns the same object")
    ck = OA.ObservationAdapter(spec, "aiso6clk", recon)
    out6 = ck.adapt(obs84, t)
    check(out6.shape == (88,) and abs(float(out6[0]) - np.float32(s)) < 1e-7 and abs(float(out6[1]) - np.float32(c)) < 1e-7, "aiso6clk restores the clock pair")
    check(np.array_equal(OA.project39to35(out6, spec)[2:], obs84[2:]), "aiso6clk projection exact outside the clock pair")
    expect_error(lambda: OA.ObservationAdapter(spec, "bogus", recon), "unknown mode refused")
    expect_error(lambda: OA.ObservationAdapter(spec, "aiso4", None), "aiso4 without reconstructor refused")
    expect_error(lambda: ad.adapt(np.zeros(10, dtype=np.float32), t), "too narrow observation refused")
    # t_pre
    tp = OA.t_pre_from_trace(11.99, [12.0, 12.01, 12.02])
    check(tp == [11.99, 12.0, 12.01], "t_pre = [reset] + time[:-1]")
    expect_error(lambda: OA.t_pre_from_trace(11.99, [12.0, 12.0]), "non-increasing times refused")
    expect_error(lambda: OA.t_pre_from_trace(12.5, [12.0]), "first time <= reset refused")
    # cross-check (schema 2): recorded obs are the env's float32 cast; post-step targets are float64
    t_pre = [10.5 + 0.01 * k for k in range(20)]
    times = [t + 0.01 for t in t_pre]
    rows = np.zeros((20, 39))
    post = np.zeros((20, 4))
    for i, tt in enumerate(t_pre):
        rows[i, 2:6] = recon.targets(tt).astype(np.float32).astype(np.float64)  # what the env records (float32 obs)
        post[i] = recon.targets(times[i])  # what the env records post-step (float64)
    res = OA.crosscheck_targets_against_trace(recon, spec, rows, t_pre, obs_dtype=np.float32, post_step_targets=post, post_step_times=times)
    check(res["exact"] and res["exact_runtime_dtype"] and res["exact_float64_post_step"] and res["exact_rows_runtime_dtype"] == 20 and res["exact_cells_runtime_dtype"] == 80 and res["max_abs_diff_runtime_dtype"] == 0.0 and res["max_abs_diff_float64_post_step"] == 0.0 and res["observation_dtype"] == "float32" and res["schema_version"] == 2, "cross-check exact at the runtime dtype and in float64 post-step")
    check(res["recorded_representable_in_obs_dtype"] and res["max_abs_diff_float64_vs_recorded_informational"] > 0.0, "float32 quantisation of the recording reported as informational only")
    raw64 = np.zeros((20, 39))
    for i, tt in enumerate(t_pre):
        raw64[i, 2:6] = recon.targets(tt)
    expect_error(lambda: OA.crosscheck_targets_against_trace(recon, spec, raw64, t_pre, obs_dtype=np.float32, post_step_targets=post, post_step_times=times), "recorded float64 values not representable in float32 are refused (trace not from this contract)")
    res64 = OA.crosscheck_targets_against_trace(recon, spec, raw64, t_pre, obs_dtype=np.float64, post_step_targets=post, post_step_times=times)
    check(res64["exact"] and res64["observation_dtype"] == "float64", "dtype comes from the caller: float64 env contract also exact")
    rows2 = rows.copy()
    rows2[3, 4] = np.float32(np.nextafter(np.float32(rows2[3, 4]), np.float32(np.inf)))  # one float32 ulp off
    res2 = OA.crosscheck_targets_against_trace(recon, spec, rows2, t_pre, obs_dtype=np.float32, post_step_targets=post, post_step_times=times)
    check(not res2["exact"] and not res2["exact_runtime_dtype"] and res2["exact_float64_post_step"] and res2["exact_rows_runtime_dtype"] == 19 and res2["exact_cells_runtime_dtype"] == 79 and 0 < res2["max_abs_diff_runtime_dtype"] < 1e-5, "a single float32 ulp deviation is detected (no tolerance)")
    post2 = post.copy()
    post2[5, 1] += 1e-12
    res3 = OA.crosscheck_targets_against_trace(recon, spec, rows, t_pre, obs_dtype=np.float32, post_step_targets=post2, post_step_times=times)
    check(not res3["exact"] and res3["exact_runtime_dtype"] and not res3["exact_float64_post_step"] and res3["exact_cells_float64_post_step"] == 79 and 0 < res3["max_abs_diff_float64_post_step"] < 1e-11, "a 1e-12 float64 post-step deviation is detected (no tolerance)")
    expect_error(lambda: OA.crosscheck_targets_against_trace(recon, spec, rows, t_pre, obs_dtype=np.int32, post_step_targets=post, post_step_times=times), "non-floating observation dtype refused")
    expect_error(lambda: OA.crosscheck_targets_against_trace(recon, spec, rows, t_pre, obs_dtype=np.float32, post_step_targets=post[:-1], post_step_times=times), "post-step shape mismatch refused")
    # real env classes on synthetic kinematics (import only; no model, no simulation)
    try:
        if str(F1.BASELINE_DIR) not in sys.path:
            sys.path.insert(0, str(F1.BASELINE_DIR))
        import _bootstrap as B

        B.ensure_sim_paths()
        import osim_trj_cmc_like as ENV
    except Exception as exc:  # noqa: BLE001
        print(f"[skip] real env classes unavailable in this interpreter: {type(exc).__name__}: {exc}")
        ENV = None
    if ENV is not None:
        class FakeKin:
            time_bounds = (0.0, 20.0)

            def get(self, t):
                ph = t / 1.0
                q = {"knee_angle_r": -0.6 - 0.4 * np.cos(2 * np.pi * ph), "ankle_angle_r": 0.1 + 0.2 * np.sin(2 * np.pi * ph)}
                qd = {"knee_angle_r": 0.4 * 2 * np.pi * np.sin(2 * np.pi * ph), "ankle_angle_r": 0.2 * 2 * np.pi * np.cos(2 * np.pi * ph)}
                return q, qd, {}

        hs = [float(k) for k in range(5, 16)]  # period 1.0 s
        real_clock = ENV.GaitPhaseClock(hs, phase_offset=0.0)
        real_target = ENV.PhaseBasedImitationTarget(FakeKin(), real_clock, {"pros_knee_angle": "knee_angle_r", "pros_ankle_angle": "ankle_angle_r"}, {"pros_knee_angle": 0.5, "pros_ankle_angle": 0.5}, fallback_phase_shift=0.5, phase_samples=200, time_window=(5.0, 15.0))
        check(real_clock.available and real_target.available, "real GaitPhaseClock/PhaseBasedImitationTarget available on synthetic data")
        rr = OA.PrescribedTargetReconstructor(real_clock, real_target)
        for tt in (7.2, 9.87, 12.3):
            v = rr.targets(tt)
            q, qd, _ = real_target.get(tt)
            check(np.array_equal(v, np.array([q["pros_knee_angle"], qd["pros_knee_angle"], q["pros_ankle_angle"], qd["pros_ankle_angle"]])), f"real-class targets at t={tt} in env order")
            # the 0.5-cycle shift makes the prosthetic target the anti-phase of the sound leg
            check(abs(v[0] - (-0.6 - 0.4 * np.cos(2 * np.pi * (tt - 0.5)))) < 2e-3, f"phase-shifted periodic mean spline at t={tt}")
        check(rr.provenance["clock_n_cycles"] == 10 and rr.provenance["target_summary"]["cycle_count"] == 10, "real-class provenance")
    print(f"SELFTEST PASS ({CHECKS} checks)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
