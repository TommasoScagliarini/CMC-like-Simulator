"""Self-test of f2r_labeller.

(a) REAL, READ-ONLY: on the three pinned A_iso6clk anchors the T1 labeller (V26 39D
    teacher, float64 numpy forward) reproduces the recorded ``raw_policy_action``
    within 1e-5 (F1 verified 1.5e-7..2.6e-7) and the side-car cache grid equals the
    trace ``t_pre`` exactly; (b) encode/decode round trip + bit-equality with
    ``target_domain_imitation.encode_absolute_action``; (c) synthetic cache /
    IK / T1-T2-T3 composition; (d) fail-closed contract violations.
No rollout, no env, no file written outside a temp dir."""

from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import f2r_common as R  # noqa: E402  (puts F0/F1 on sys.path)
import f0_common as C  # noqa: E402
import f1_obs_adapter as OA  # noqa: E402
import f1_dataset as DS  # noqa: E402
import f2r_labeller as L  # noqa: E402

if str(R.F1_DIR) not in sys.path:
    sys.path.insert(0, str(R.F1_DIR))
from test_f1_obs_adapter import FakeClock, FakeTarget  # noqa: E402

if str(R.BASELINE_DIR) not in sys.path:
    sys.path.insert(0, str(R.BASELINE_DIR))
import warm_start as W  # noqa: E402  (production, import only)

CHECKS = 0
ERR = R.F2RContractError


def check(cond: bool, label: str) -> None:
    global CHECKS
    CHECKS += 1
    if not cond:
        raise AssertionError(f"CHECK FAILED: {label}")


def expect(fn, exc, label: str) -> None:
    try:
        fn()
    except exc:
        check(True, label)
        return
    raise AssertionError(f"expected {exc.__name__}: {label}")


def mkdtemp(prefix: str) -> Path:
    return R.portable_tempdir(prefix)  # portable (macOS/Windows), resolved, no hard-coded path


# --- synthetic objects ------------------------------------------------------------------------


class FakeBaseKin:
    """KinematicsInterpolator-like: get(t) -> (q, qdot, qddot) dicts; knee flexion negative."""

    def __init__(self, period: float = 1.2, t_end: float = 40.0, drop_key: str | None = None) -> None:
        self.period = float(period)
        self.time_bounds = (0.0, float(t_end))
        self._drop = drop_key

    def get(self, t: float):
        ph = 2.0 * np.pi * float(t) / self.period
        q = {"pros_knee_angle": -0.6 - 0.4 * np.cos(ph), "pros_ankle_angle": 0.05 + 0.2 * np.sin(ph), "knee_angle_r": 0.3}
        qd = {"pros_knee_angle": 0.4 * 2.0 * np.pi / self.period * np.sin(ph), "pros_ankle_angle": 0.2 * 2.0 * np.pi / self.period * np.cos(ph), "knee_angle_r": 0.0}
        qdd = {k: 0.0 for k in q}
        if self._drop:
            q.pop(self._drop, None)
        return q, qd, qdd


def synthetic_teacher_state(width: int = 39, hidden: int = 16, seed: int = 7) -> dict:
    rng = np.random.default_rng(seed)
    w1 = (0.3 * rng.standard_normal((hidden, width))).astype(np.float32)
    b1 = (0.1 * rng.standard_normal(hidden)).astype(np.float32)
    w2 = (0.3 * rng.standard_normal((hidden, hidden))).astype(np.float32)
    b2 = (0.1 * rng.standard_normal(hidden)).astype(np.float32)
    w3 = (0.3 * rng.standard_normal((4, hidden))).astype(np.float32)
    b3 = (0.1 * rng.standard_normal(4)).astype(np.float32)
    return {"pi.0.0.weight": w1, "pi.0.0.bias": b1, "pi.0.2.weight": w2, "pi.0.2.bias": b2, "pi.1.weight": w3, "pi.1.bias": b3}


def write_sidecar(job_dir: Path, spec: OA.InsertionSpec, rows: list[dict], *, mode: str = "aiso6clk", steps: int | None = None) -> Path:
    job_dir.mkdir(parents=True, exist_ok=True)
    summary = {"schema_version": 1, "driver": "synthetic_test", "driver_sha256": "0" * 64, "adapter_mode": mode, "mode": mode, "steps": len(rows) if steps is None else steps, "projection_assert_count": len(rows), "all_steps_asserted": True, "steps_match_rollout": True, "insertion": spec.to_dict(), "reconstructor": {"builder": "test", "clock_n_cycles": 3}}
    (job_dir / L.ADAPTER_SUMMARY_FILE).write_text(json.dumps(summary, indent=1), encoding="utf-8")
    (job_dir / L.ADAPTER_TRACE_FILE).write_text(json.dumps(rows), encoding="utf-8")
    return job_dir


def manual_u_T(teacher: dict, spec: OA.InsertionSpec, obs35: np.ndarray, targets: np.ndarray, clock: np.ndarray) -> np.ndarray:
    """The F1 aiso6clk route written out explicitly (reference for the labeller)."""
    obs_f32 = np.asarray(obs35, dtype=np.float32)
    obs39 = OA.insert_targets(obs_f32, np.asarray(targets, dtype=np.float32), spec)
    obs39[:, 0] = np.asarray(clock[:, 0], dtype=np.float32)
    obs39[:, 1] = np.asarray(clock[:, 1], dtype=np.float32)
    return DS.actor_logits_numpy(teacher, obs39.astype(np.float64))[:, :2].astype(np.float32)


# --- (a) real anchors, read-only ---------------------------------------------------------------


def test_real_anchors(spec: OA.InsertionSpec) -> None:
    pins = R.verify_anchor_pins()
    check(pins["all_match"] is True, "the three anchor job directories match their pinned SHA-256 (read-only)")
    module = Path(R.TEACHER["module"])
    check(C.sha256_file(module / "module_state.pkl") == R.TEACHER["module_state_sha256"], "teacher module_state.pkl pinned")
    state = W.load_module_state(module)
    digest = W.actor_state_digest(state)
    check(digest == R.TEACHER["actor_digest"], "teacher actor digest pinned (V26_39D)")
    teacher = DS.load_actor_arrays(state, expected_width=R.MODULE_WIDTH_39)
    worst = 0.0
    digests = set()
    for start, anchor in R.ANCHORS.items():
        job_dir = Path(anchor["job_dir"])
        check(pins[start]["all_match"] is True, f"{start}: anchor pins verified")
        cache = L.PrivilegedCache.from_adapter_sidecar(job_dir, pins=anchor)
        rows = json.loads((job_dir / "rollout_policy_trace.json").read_text(encoding="utf-8"))
        reset = json.loads((job_dir / "rollout_reset_diagnostics.json").read_text(encoding="utf-8"))
        t_pre = np.asarray(OA.t_pre_from_trace(float(reset["time"]), [float(r["time"]) for r in rows]), dtype=np.float64)
        obs35 = np.asarray([r["actor_observation_vector_before"] for r in rows], dtype=np.float32)
        raw = np.asarray([r["raw_policy_action"] for r in rows], dtype=np.float64)
        check(cache.rows == len(rows) == 500 and cache.t_pre.shape == t_pre.shape and np.array_equal(cache.t_pre, t_pre) and cache.t_pre.dtype == np.float64, f"{start}: cache t_pre equals the trace t_pre exactly ({cache.rows} rows)")
        check(cache.provenance["adapter_trace"]["sha256"] == anchor["adapter_trace_sha256"] and cache.provenance["adapter_summary"]["sha256"] == anchor["adapter_summary_sha256"] and cache.provenance["adapter_mode"] == "aiso6clk" and cache.provenance["pins_verified"]["adapter_trace_sha256"] == anchor["adapter_trace_sha256"], f"{start}: side-car SHA-256 recorded and pinned")
        check(cache.ik_action is None and cache.targets.shape == (500, 4) and cache.clock.shape == (500, 2) and not cache.t_pre.flags.writeable and not cache.targets.flags.writeable, f"{start}: side-car cache layout (no IK, read-only arrays)")
        lab = L.TeacherLabeller("T1", teacher, spec, cache, teacher_digest=digest)
        out = lab.label(obs35, t_pre)
        err = float(np.abs(out["u_T"].astype(np.float64) - raw).max())
        worst = max(worst, err)
        check(err <= 1e-5, f"{start}: max|u_T - raw_policy_action| = {err:.3e} <= 1e-5 on all 500 rows")
        check(out["u_T"].dtype == np.float32 and out["actions"].dtype == np.float32 and np.array_equal(out["actions"], out["u_T"]) and out["u_IK"] is None and out["components"] == {"knee": "u_T", "ankle": "u_T"} and out["rows"] == 500 and out["variant"] == "T1" and out["cache_digest"] == cache.digest(), f"{start}: T1 labels = u_T both joints")
        check(set(out) == {"actions", "u_T", "u_IK", "components", "rows", "cache_digest", "variant"}, f"{start}: label output carries actions only (no feature/time key)")
        sel = np.array([7, 3, 499, 0, 250, 250])
        out_sel = lab.label(obs35[sel], t_pre[sel])
        check(np.array_equal(out_sel["u_T"], out["u_T"][sel]), f"{start}: row subset in arbitrary order labelled identically (lookup by exact time)")
        out64 = lab.label(obs35.astype(np.float64), t_pre)
        check(np.array_equal(out64["u_T"], out["u_T"]), f"{start}: float64 obs rows are float32-cast (same labels)")
        obs39, idx = lab.teacher_observation(obs35, t_pre)
        check(obs39.shape == (500, 39) and obs39.dtype == np.float32 and np.array_equal(idx, np.arange(500)) and np.array_equal(OA.project39to35(obs39, spec)[:, 2:], obs35[:, 2:]) and np.array_equal(obs39[:, 2:6], cache.targets.astype(np.float32)) and np.array_equal(obs39[:, 0:2], cache.clock.astype(np.float32)), f"{start}: teacher observation = obs35 + float32 targets at 2:6 + float32 clock at 0:2")
        prov = lab.provenance()
        check(prov["teacher"]["actor_digest"] == digest and prov["variant"] == "T1" and prov["cache"]["digest"] == cache.digest() and prov["insertion"]["insert_index"] == 2 and prov["composition"] == {"knee": "u_T", "ankle": "u_T"} and "composition" in prov["composition_rule"] and prov["protocol"]["id"] == R.PROTOCOL_ID and prov["cache"]["provenance"]["job_dir"] == C.rel(job_dir), f"{start}: provenance (teacher digest, cache digest, spec, protocol rule)")
        expect(lambda: L.TeacherLabeller("T2", teacher, spec, cache), ERR, f"{start}: T2 on a side-car cache without ik_action refused")
        expect(lambda: L.TeacherLabeller("T3", teacher, spec, cache), ERR, f"{start}: T3 on a side-car cache without ik_action refused")
        expect(lambda: L.PrivilegedCache.from_adapter_sidecar(job_dir, pins={**anchor, "adapter_trace_sha256": "0" * 64}), ERR, f"{start}: wrong side-car pin refused")
        expect(lambda: lab.label(obs35, t_pre + 1e-9), ERR, f"{start}: off-grid t_pre refused on real data")
        digests.add(cache.digest())
    check(len(digests) == 3, "the three anchor caches have distinct digests")
    print(f"[real] worst max|u_T - raw_policy_action| over the 3 anchors = {worst:.3e}")


# --- (b) encode / decode -------------------------------------------------------------------------


def test_encode_decode() -> None:
    rng = np.random.default_rng(2026)
    vals = rng.uniform(-2.5, 2.5, size=(64, 2))  # includes out-of-bounds values
    a = L.encode_absolute_action(vals)
    check(a.shape == (64, 2) and a.dtype == np.float64 and float(a.min()) >= -1.0 and float(a.max()) <= 1.0, "encode shape/dtype/clip")
    manual = np.empty_like(vals)
    manual[:, 0] = 2.0 * (vals[:, 0] - (-1.5)) / (0.0 - (-1.5)) - 1.0
    manual[:, 1] = 2.0 * (vals[:, 1] - (-0.7)) / (0.7 - (-0.7)) - 1.0
    check(np.array_equal(a, np.clip(manual, -1.0, 1.0)), "encode = 2 (q - low)/(high - low) - 1, clipped (bit-exact)")
    check(bool(np.any(manual < -1.0) and np.any(manual > 1.0)), "sample contains out-of-bounds values on both sides")
    q = L.decode_absolute_action(a)
    check(np.allclose(q, np.clip(vals, [-1.5, -0.7], [0.0, 0.7]), rtol=0.0, atol=1e-12), "decode(encode(q)) = q clipped to the bounds")
    inb = rng.uniform([-1.5, -0.7], [0.0, 0.7], size=(64, 2))
    check(np.allclose(L.decode_absolute_action(L.encode_absolute_action(inb)), inb, rtol=0.0, atol=1e-12), "round trip inside the bounds")
    acts = rng.uniform(-1.0, 1.0, size=(64, 2))
    check(np.allclose(L.encode_absolute_action(L.decode_absolute_action(acts)), acts, rtol=0.0, atol=1e-12), "encode(decode(a)) = a")
    dq = L.decode_absolute_action(acts)
    check(np.allclose(dq[:, 0], 0.75 * acts[:, 0] - 0.75, rtol=0.0, atol=1e-12) and np.allclose(dq[:, 1], 0.7 * acts[:, 1], rtol=0.0, atol=1e-12), "protocol decode: knee q = 0.75 a - 0.75, ankle q = 0.7 a")
    corners = L.encode_absolute_action(np.array([[-1.5, -0.7], [0.0, 0.7], [-0.75, 0.0]]))
    check(np.array_equal(corners, np.array([[-1.0, -1.0], [1.0, 1.0], [0.0, 0.0]])), "bounds map exactly to -1 / +1 / 0")
    check(np.array_equal(L.decode_absolute_action(np.array([[-3.0, 4.0]])), np.array([[-1.5, 0.7]])), "decode clips the action to [-1, 1] first")
    try:
        import target_domain_imitation as TDI  # production, import only
    except Exception as exc:  # noqa: BLE001
        print(f"[skip] target_domain_imitation unavailable in this interpreter: {type(exc).__name__}: {exc}")
        TDI = None
    if TDI is not None:
        ref = TDI.encode_absolute_action(vals, R.PROS_COORDS, R.ABSOLUTE_BOUNDS_RAD)
        check(np.array_equal(ref, a) and ref.dtype == a.dtype, "bit-equal to target_domain_imitation.encode_absolute_action incl. clipping")
        ref2 = TDI.encode_absolute_action(inb, R.PROS_COORDS, R.ABSOLUTE_BOUNDS_RAD)
        check(np.array_equal(ref2, L.encode_absolute_action(inb)), "bit-equal to production inside the bounds")
    expect(lambda: L.encode_absolute_action(np.zeros(3)), ERR, "encode refuses 1-D input")
    expect(lambda: L.encode_absolute_action(np.zeros((3, 3))), ERR, "encode refuses 3 columns")
    expect(lambda: L.encode_absolute_action(np.array([[np.nan, 0.0]])), ERR, "encode refuses non-finite")
    expect(lambda: L.encode_absolute_action(np.zeros((2, 2)), bounds={"pros_knee_angle": (0.0, 0.0), "pros_ankle_angle": (-0.7, 0.7)}), ERR, "degenerate bounds refused")
    expect(lambda: L.encode_absolute_action(np.zeros((2, 2)), bounds={"pros_knee_angle": (-1.5, 0.0)}), ERR, "missing bound refused")
    expect(lambda: L.decode_absolute_action(np.zeros((2, 1))), ERR, "decode refuses wrong width")


# --- (c) synthetic cache / IK / composition ----------------------------------------------------


def test_synthetic(spec: OA.InsertionSpec) -> None:
    rng = np.random.default_rng(11)
    kin = FakeBaseKin()
    t_pre = 10.0 + 0.01 * np.arange(80, dtype=np.float64)
    ik0 = L.ik_actions_from_base_kin(kin, t_pre)
    ik5 = L.ik_actions_from_base_kin(kin, t_pre, lookahead_s=0.05)
    check(ik0.shape == (80, 2) and ik0.dtype == np.float32 and ik5.shape == (80, 2) and ik5.dtype == np.float32, "ik actions shape/dtype")
    direct0 = L.encode_absolute_action(np.asarray([[kin.get(t + 0.01 + 0.0)[0]["pros_knee_angle"], kin.get(t + 0.01 + 0.0)[0]["pros_ankle_angle"]] for t in t_pre.tolist()])).astype(np.float32)
    direct5 = L.encode_absolute_action(np.asarray([[kin.get(t + 0.01 + 0.05)[0]["pros_knee_angle"], kin.get(t + 0.01 + 0.05)[0]["pros_ankle_angle"]] for t in t_pre.tolist()])).astype(np.float32)
    check(np.array_equal(ik0, direct0), "u_IK(t) = encode(q_IK(t + 0.01)) bit-exact (lookahead 0)")
    check(np.array_equal(ik5, direct5) and not np.array_equal(ik0, ik5), "lookahead 0.05 shifts the teacher time by 0.05 s (differs from lookahead 0)")
    check(bool(np.all(L.decode_absolute_action(ik0.astype(np.float64))[:, 0] < 0.0)), "decoded knee target negative (flexion)")
    check(np.array_equal(L.ik_actions_from_base_kin(kin, t_pre, lookahead_s=0.0, segment_s=0.01), ik0), "explicit defaults identical")
    ik_end = L.ik_actions_from_base_kin(kin, t_pre, t_end=10.3)
    direct_end = L.encode_absolute_action(np.asarray([[kin.get(10.3)[0]["pros_knee_angle"], kin.get(10.3)[0]["pros_ankle_angle"]]])).astype(np.float32)
    check(np.array_equal(ik_end[:28], ik0[:28]) and bool(np.all(ik_end[30:] == direct_end)), "t_end clips the teacher time")
    expect(lambda: L.ik_actions_from_base_kin(kin, t_pre, lookahead_s=-0.01), ERR, "negative lookahead refused")
    expect(lambda: L.ik_actions_from_base_kin(kin, t_pre, segment_s=0.0), ERR, "non-positive segment refused")
    expect(lambda: L.ik_actions_from_base_kin(FakeBaseKin(drop_key="pros_ankle_angle"), t_pre), ERR, "missing IK coordinate refused")
    expect(lambda: L.ik_actions_from_base_kin(kin, t_pre.reshape(8, 10)), ERR, "2-D t_pre refused")

    class BadKin:
        def get(self, t):
            return {"pros_knee_angle": -0.5, "pros_ankle_angle": 0.0}

    expect(lambda: L.ik_actions_from_base_kin(BadKin(), t_pre), ERR, "base_kin returning a bare dict refused (tuple contract)")
    # synthetic cache from injected env-contract objects
    clock = FakeClock([8.0 + 1.1 * k for k in range(12)])
    target = FakeTarget(clock)
    recon = OA.PrescribedTargetReconstructor(clock, target, provenance={"builder": "test"})
    cache = L.PrivilegedCache.from_objects(recon, kin, t_pre, lookahead_s=0.0)
    check(cache.rows == 80 and len(cache) == 80 and cache.ik_action is not None and cache.ik_action.dtype == np.float32 and cache.targets.dtype == np.float64 and cache.clock.dtype == np.float64, "from_objects layout")
    check(all(np.array_equal(cache.targets[k], recon.targets(float(t_pre[k]))) for k in (0, 17, 79)) and all(tuple(cache.clock[k]) == recon.clock_sin_cos(float(t_pre[k])) for k in (0, 17, 79)), "targets / clock from the reconstructor at t_pre")
    check(np.array_equal(cache.ik_action, ik0), "cache ik_action = ik_actions_from_base_kin (lookahead 0)")
    check(cache.provenance["source"] == "objects" and cache.provenance["reconstructor"]["builder"] == "test" and cache.provenance["ik_action"]["lookahead_s"] == 0.0, "from_objects provenance")
    idx = cache.lookup(t_pre[[5, 0, 79, 5]])
    check(idx.tolist() == [5, 0, 79, 5] and idx.dtype == np.int64, "lookup by exact float equality")
    check(cache.lookup(np.zeros(0)).shape == (0,), "empty lookup")
    expect(lambda: cache.lookup(np.array([t_pre[5] + 1e-9])), ERR, "off-grid query (1e-9) refused")
    expect(lambda: cache.lookup(np.array([t_pre[0] - 0.01])), ERR, "query before the grid refused")
    expect(lambda: cache.lookup(np.array([t_pre[-1] + 0.01])), ERR, "query after the grid refused")
    expect(lambda: cache.lookup(np.array([np.nan])), ERR, "NaN query refused")
    expect(lambda: cache.lookup(t_pre.reshape(8, 10)), ERR, "2-D query refused")
    d1 = cache.digest()
    check(d1 == L.PrivilegedCache.from_objects(recon, kin, t_pre, lookahead_s=0.0).digest(), "digest deterministic")
    cache5 = L.PrivilegedCache.from_objects(recon, kin, t_pre, lookahead_s=0.05)
    cache_noik = L.PrivilegedCache.from_objects(recon, None, t_pre)
    check(len({d1, cache5.digest(), cache_noik.digest()}) == 3 and cache_noik.ik_action is None, "digest sensitive to ik_action (and its absence)")
    check(np.array_equal(cache_noik.with_ik_action(kin).ik_action, ik0) and cache_noik.with_ik_action(kin).digest() == d1, "with_ik_action attaches u_IK on the same grid (digest equals from_objects)")
    expect(lambda: cache.t_pre.__setitem__(0, 0.0), ValueError, "cache arrays are read-only")
    # teacher composition
    teacher = DS.load_actor_arrays(synthetic_teacher_state(), expected_width=39)
    obs = rng.standard_normal((80, 35)).astype(np.float32)
    obs[:, 0] = 0.0
    obs[:, 1] = 1.0
    expected_uT = manual_u_T(teacher, spec, obs, cache.targets, cache.clock)
    outs = {}
    for variant, comps in (("T1", {"knee": "u_T", "ankle": "u_T"}), ("T2", {"knee": "u_T", "ankle": "u_IK"}), ("T3", {"knee": "u_IK", "ankle": "u_IK"})):
        lab = L.TeacherLabeller(variant, teacher, spec, cache, teacher_digest="synthetic")
        out = lab.label(obs, t_pre)
        outs[variant] = out
        check(np.array_equal(out["u_T"], expected_uT) and out["u_T"].dtype == np.float32, f"{variant}: u_T = F1 aiso6clk route (float32 cast, targets 2:6, clock 0:2)")
        check(np.array_equal(out["u_IK"], ik0) and out["u_IK"].dtype == np.float32, f"{variant}: u_IK from the cache")
        check(out["components"] == comps and out["variant"] == variant and out["rows"] == 80 and out["cache_digest"] == d1 and out["actions"].dtype == np.float32 and out["actions"].shape == (80, 2), f"{variant}: components/metadata")
        check(lab.provenance()["variant_label"] == R.load_protocol()["variants"][variant]["label"] and lab.provenance()["composition"] == comps and lab.provenance()["teacher"]["actor_digest"] == "synthetic", f"{variant}: provenance reflects the protocol variant")
    check(np.array_equal(outs["T1"]["actions"], expected_uT), "T1: actions = u_T both joints")
    check(np.array_equal(outs["T2"]["actions"][:, 0], expected_uT[:, 0]) and np.array_equal(outs["T2"]["actions"][:, 1], ik0[:, 1]), "T2: knee = u_T[:,0], ankle = u_IK[:,1] bit-exact")
    check(not np.array_equal(outs["T2"]["actions"], expected_uT) and not np.array_equal(outs["T2"]["actions"], ik0), "T2 differs from both pure sources")
    check(np.array_equal(outs["T3"]["actions"], ik0), "T3: actions = u_IK both joints")
    perm = rng.permutation(80)
    out_p = L.TeacherLabeller("T2", teacher, spec, cache).label(obs[perm], t_pre[perm])
    check(np.array_equal(out_p["actions"], outs["T2"]["actions"][perm]), "permuted rows -> permuted labels (time indexes the cache only)")
    acts, comps = L.compose_actions("T2", expected_uT, ik0)
    check(np.array_equal(acts, outs["T2"]["actions"]) and comps == {"knee": "u_T", "ankle": "u_IK"}, "compose_actions pure function agrees with the labeller")
    expect(lambda: L.compose_actions("T2", expected_uT, None), ERR, "compose T2 without u_IK refused")
    expect(lambda: L.compose_actions("T1", expected_uT.astype(np.float64)), ERR, "compose refuses float64 (labels are float32)")
    expect(lambda: L.compose_actions("T3", expected_uT, ik0[:10]), ERR, "compose refuses shape mismatch")
    expect(lambda: L.compose_actions("T9", expected_uT, ik0), ERR, "compose refuses unknown variant")
    # side-car round trip through a synthetic directory
    tmp = mkdtemp("f2r_labeller_")
    try:
        rows = [{"step_index": i + 1, "t_pre": float(t), "obs_width_in": 84, "obs_width_out": 88, "targets": cache.targets[i].tolist(), "clock_sin_cos_inserted": cache.clock[i].tolist(), "projection_exact": True} for i, t in enumerate(t_pre.tolist())]
        job = write_sidecar(tmp / "job_ok", spec, rows)
        side = L.PrivilegedCache.from_adapter_sidecar(job)
        check(np.array_equal(side.t_pre, cache.t_pre) and np.array_equal(side.targets, cache.targets) and np.array_equal(side.clock, cache.clock) and side.ik_action is None and side.digest() == cache_noik.digest(), "synthetic side-car round trip (JSON floats exact)")
        check(side.provenance["source"] == "adapter_sidecar" and len(side.provenance["adapter_trace"]["sha256"]) == 64 and side.provenance["insertion"]["insert_index"] == 2, "side-car provenance")
        side_ik = side.with_ik_action(kin)
        out_side = L.TeacherLabeller("T2", teacher, spec, side_ik).label(obs, t_pre)
        check(np.array_equal(out_side["actions"], outs["T2"]["actions"]), "side-car cache + with_ik_action gives the same T2 labels")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# --- (d) fail-closed ---------------------------------------------------------------------------


def test_fail_closed(spec: OA.InsertionSpec) -> None:
    rng = np.random.default_rng(5)
    kin = FakeBaseKin()
    clock = FakeClock([8.0 + 1.1 * k for k in range(12)])
    recon = OA.PrescribedTargetReconstructor(clock, FakeTarget(clock))
    t_pre = 12.0 + 0.01 * np.arange(30, dtype=np.float64)
    cache = L.PrivilegedCache.from_objects(recon, kin, t_pre)
    cache_noik = L.PrivilegedCache.from_objects(recon, None, t_pre)
    teacher = DS.load_actor_arrays(synthetic_teacher_state(), expected_width=39)
    obs = rng.standard_normal((30, 35)).astype(np.float32)
    obs[:, 0] = 0.0
    obs[:, 1] = 1.0
    lab = L.TeacherLabeller("T1", teacher, spec, cache)
    check(lab.label(obs, t_pre)["rows"] == 30, "baseline synthetic labelling works")
    expect(lambda: lab.label(obs, t_pre + 1e-9), ERR, "off-grid t_pre refused")
    expect(lambda: lab.label(obs, t_pre + 0.005), ERR, "half-step t_pre refused (no interpolation)")
    expect(lambda: lab.label(obs[:, :34], t_pre), ERR, "obs width 34 refused")
    expect(lambda: lab.label(np.concatenate([obs, obs[:, :1]], axis=1), t_pre), ERR, "obs width 36 refused")
    bad = obs.copy()
    bad[3, 0] = 0.1
    expect(lambda: lab.label(bad, t_pre), ERR, "obs35[:,0] != 0 refused (dead clock constant)")
    bad = obs.copy()
    bad[0, 1] = 0.0
    expect(lambda: lab.label(bad, t_pre), ERR, "obs35[:,1] != 1 refused (dead clock constant)")
    bad = obs.copy()
    bad[2, 7] = np.nan
    expect(lambda: lab.label(bad, t_pre), ERR, "non-finite obs refused")
    expect(lambda: lab.label(obs, t_pre[:29]), ERR, "t_pre length mismatch refused")
    expect(lambda: lab.label(obs[:0], t_pre[:0]), ERR, "zero rows refused")
    expect(lambda: lab.label(obs.astype(object), t_pre), ERR, "object dtype refused")
    expect(lambda: L.TeacherLabeller("T9", teacher, spec, cache), ERR, "variant T9 refused")
    expect(lambda: L.TeacherLabeller("t1", teacher, spec, cache), ERR, "lower-case variant refused")
    expect(lambda: L.TeacherLabeller("T2", teacher, spec, cache_noik), ERR, "T2 without ik_action refused")
    expect(lambda: L.TeacherLabeller("T3", teacher, spec, cache_noik), ERR, "T3 without ik_action refused")
    check(L.TeacherLabeller("T1", teacher, spec, cache_noik).label(obs, t_pre)["u_IK"] is None, "T1 without ik_action allowed (u_IK None)")
    teacher35 = DS.load_actor_arrays(synthetic_teacher_state(width=35), expected_width=35)
    expect(lambda: L.TeacherLabeller("T1", teacher35, spec, cache), ERR, "35-wide teacher refused (teacher must consume 39)")
    expect(lambda: L.TeacherLabeller("T1", {k: v for k, v in teacher.items() if k != "pi.1.bias"}, spec, cache), ERR, "incomplete teacher arrays refused")
    expect(lambda: L.TeacherLabeller("T1", teacher, spec.to_dict(), cache), ERR, "spec must be an InsertionSpec")
    expect(lambda: L.TeacherLabeller("T1", teacher, spec, {"t_pre": t_pre}), ERR, "cache must be a PrivilegedCache")
    # cache construction
    expect(lambda: L.PrivilegedCache(t_pre=t_pre[::-1], targets=cache.targets, clock=cache.clock), ERR, "non-increasing t_pre refused")
    expect(lambda: L.PrivilegedCache(t_pre=t_pre, targets=cache.targets[:, :3], clock=cache.clock), ERR, "3 targets refused")
    expect(lambda: L.PrivilegedCache(t_pre=t_pre, targets=cache.targets, clock=np.full((30, 2), 0.5)), ERR, "clock off the unit circle refused")
    expect(lambda: L.PrivilegedCache(t_pre=t_pre, targets=cache.targets, clock=cache.clock, ik_action=cache.ik_action.astype(np.float64)), ERR, "float64 ik_action refused (must be float32)")
    expect(lambda: L.PrivilegedCache(t_pre=t_pre, targets=cache.targets, clock=cache.clock, ik_action=np.full((30, 2), 1.5, dtype=np.float32)), ERR, "ik_action outside [-1, 1] refused")
    expect(lambda: L.PrivilegedCache.from_objects(recon, kin, np.array([12.0, 12.0, 12.01])), ERR, "duplicate grid time refused")
    expect(lambda: L.PrivilegedCache.from_objects(recon, kin, np.zeros(0)), ERR, "empty grid refused")

    class ShortRecon:
        def targets(self, t):
            return np.zeros(3)

        def clock_sin_cos(self, t):
            return (0.0, 1.0)

    expect(lambda: L.PrivilegedCache.from_objects(ShortRecon(), None, t_pre), ERR, "reconstructor with 3 targets refused")
    # side-cars
    tmp = mkdtemp("f2r_labeller_fc_")
    try:
        rows = [{"step_index": i + 1, "t_pre": float(t), "targets": cache.targets[i].tolist(), "clock_sin_cos_inserted": cache.clock[i].tolist()} for i, t in enumerate(t_pre.tolist())]
        check(L.PrivilegedCache.from_adapter_sidecar(write_sidecar(tmp / "ok", spec, rows)).rows == 30, "minimal valid side-car accepted")
        rows_none = [{**r, "clock_sin_cos_inserted": None} for r in rows]
        expect(lambda: L.PrivilegedCache.from_adapter_sidecar(write_sidecar(tmp / "clock_none", spec, rows_none)), ERR, "side-car with clock None refused (every row needs the 2-element clock)")
        rows_one = [dict(r) for r in rows]
        rows_one[17]["clock_sin_cos_inserted"] = None
        expect(lambda: L.PrivilegedCache.from_adapter_sidecar(write_sidecar(tmp / "clock_one_none", spec, rows_one)), ERR, "a single row without clock refused")
        rows_three = [dict(r) for r in rows]
        rows_three[4]["clock_sin_cos_inserted"] = [0.0, 1.0, 0.0]
        expect(lambda: L.PrivilegedCache.from_adapter_sidecar(write_sidecar(tmp / "clock_three", spec, rows_three)), ERR, "3-element clock refused")
        expect(lambda: L.PrivilegedCache.from_adapter_sidecar(write_sidecar(tmp / "aiso4", spec, rows, mode="aiso4")), ERR, "adapter_mode aiso4 side-car refused")
        expect(lambda: L.PrivilegedCache.from_adapter_sidecar(write_sidecar(tmp / "passthrough", spec, rows, mode="passthrough")), ERR, "adapter_mode passthrough side-car refused")
        expect(lambda: L.PrivilegedCache.from_adapter_sidecar(write_sidecar(tmp / "steps", spec, rows, steps=31)), ERR, "summary steps != rows refused")
        rows_gap = [dict(r) for r in rows]
        rows_gap[10]["step_index"] = 12
        expect(lambda: L.PrivilegedCache.from_adapter_sidecar(write_sidecar(tmp / "gap", spec, rows_gap)), ERR, "non-contiguous step_index refused")
        rows_tg = [dict(r) for r in rows]
        rows_tg[0]["targets"] = rows_tg[0]["targets"][:3]
        expect(lambda: L.PrivilegedCache.from_adapter_sidecar(write_sidecar(tmp / "tg3", spec, rows_tg)), ERR, "3 targets in a side-car row refused")
        rows_t = [dict(r) for r in rows]
        rows_t[5]["t_pre"] = rows_t[4]["t_pre"]
        expect(lambda: L.PrivilegedCache.from_adapter_sidecar(write_sidecar(tmp / "tdup", spec, rows_t)), ERR, "non-increasing side-car t_pre refused")
        expect(lambda: L.PrivilegedCache.from_adapter_sidecar(write_sidecar(tmp / "empty", spec, [])), ERR, "empty side-car refused")
        expect(lambda: L.PrivilegedCache.from_adapter_sidecar(tmp / "missing"), ERR, "missing job dir refused")
        job = write_sidecar(tmp / "bad_insertion", spec, rows)
        summ = json.loads((job / L.ADAPTER_SUMMARY_FILE).read_text(encoding="utf-8"))
        summ["insertion"]["manifest35_sha256"] = "0" * 64
        (job / L.ADAPTER_SUMMARY_FILE).write_text(json.dumps(summ), encoding="utf-8")
        expect(lambda: L.PrivilegedCache.from_adapter_sidecar(job), ERR, "side-car insertion with a foreign manifest pin refused")
        job = write_sidecar(tmp / "not_asserted", spec, rows)
        summ = json.loads((job / L.ADAPTER_SUMMARY_FILE).read_text(encoding="utf-8"))
        summ["all_steps_asserted"] = False
        (job / L.ADAPTER_SUMMARY_FILE).write_text(json.dumps(summ), encoding="utf-8")
        expect(lambda: L.PrivilegedCache.from_adapter_sidecar(job), ERR, "side-car without all_steps_asserted refused")
        job = write_sidecar(tmp / "no_trace", spec, rows)
        (job / L.ADAPTER_TRACE_FILE).unlink()
        expect(lambda: L.PrivilegedCache.from_adapter_sidecar(job), ERR, "missing adapter trace refused")
        job = write_sidecar(tmp / "bad_json", spec, rows)
        (job / L.ADAPTER_TRACE_FILE).write_text("{not json", encoding="utf-8")
        expect(lambda: L.PrivilegedCache.from_adapter_sidecar(job), ERR, "malformed JSON refused")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# --- (e) persistence + CLI guards (temp dir; real anchors read-only) ----------------------------


def test_persistence() -> None:
    
    tmp = R.portable_tempdir("f2r_labeller_")
    out = L.build_caches_from_anchors(tmp / "cache")
    check(set(out) == set(R.STARTS) and all((Path(C.REPO) / v["npz"]).is_file() and (Path(C.REPO) / v["provenance"]).is_file() for v in out.values()), "build-cache --from-anchors writes npz + provenance per start (temp dir)")
    for start in R.STARTS:
        direct = L.PrivilegedCache.from_adapter_sidecar(R.ANCHORS[start]["job_dir"], pins=R.ANCHORS[start])
        back = L.load_cache(tmp / "cache", start, expected_digest=direct.digest())
        check(back.digest() == direct.digest() == out[start]["cache_digest"] and np.array_equal(back.t_pre, direct.t_pre) and np.array_equal(back.targets, direct.targets) and np.array_equal(back.clock, direct.clock) and back.ik_action is None, f"{start}: cache save/reload bit-exact (digest stable)")
    expect(lambda: L.build_caches_from_anchors(tmp / "cache"), FileExistsError, "no-clobber cache build")
    expect(lambda: L.load_cache(tmp / "cache", "nominal", expected_digest="0" * 64), ERR, "reload with a wrong expected digest refused")
    expect(lambda: L.main(["build-cache", "--with-ik", "--out-dir", str(tmp / "ik")]), SystemExit, "CLI --with-ik (env 0-step) refused without --authorized-stage S1")
    check(not (tmp / "ik").exists(), "refused --with-ik created nothing")
    cli_out = tmp / "cli"
    rc = L.main(["build-cache", "--from-anchors", "--out-dir", str(cli_out)])
    check(rc == 0 and sorted(p.name for p in cli_out.glob("*.npz")) == [f"privileged_cache_{s}.npz" for s in sorted(R.STARTS)], "CLI build-cache --from-anchors (env-free) materialises the three caches")


def main() -> int:
    names35, sha35 = OA.read_manifest_names(C.ACTOR_MANIFEST_35, expected_sha256=C.ACTOR_MANIFEST_35_SHA256, sha256_fn=C.sha256_file)
    names39, sha39 = OA.read_manifest_names(C.ACTOR_MANIFEST_39, expected_sha256=C.ACTOR_MANIFEST_39_SHA256, sha256_fn=C.sha256_file)
    spec = OA.derive_insertion(names35, names39, manifest35_sha256=sha35, manifest39_sha256=sha39)
    check(spec.index == 2 and spec.count == 4 and len(spec.names35) == 35 and len(spec.names39) == 39, "pinned manifests -> insertion block of 4 at index 2")
    check(L.COMPOSITION == {"T1": ("u_T", "u_T"), "T2": ("u_T", "u_IK"), "T3": ("u_IK", "u_IK")} and L.REQUIRED_ADAPTER_MODE == "aiso6clk", "composition table and required adapter mode")
    test_real_anchors(spec)
    test_encode_decode()
    test_synthetic(spec)
    test_fail_closed(spec)
    test_persistence()
    print(f"SELFTEST PASS ({CHECKS} checks)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
