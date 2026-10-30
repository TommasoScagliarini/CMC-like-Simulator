"""The V26C J3 runner, exercised end to end against a FAKE stack.

No physical rollout: the environment, the RLModule and (unless real torch is importable) the tensor
library are all test doubles. The immutable J2 leaf is read, never written: its six hashes are
compared before and after the suite.
"""
from __future__ import annotations
import contextlib, json, sys, tempfile, types
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26c_j3_closed_loop as J3  # noqa: E402
import v26c_j1_collect as J1  # noqa: E402

# J1's hardened helpers raise J1Error; either exception is a fail-closed refusal.
FAILCLOSED = (J3.J3Error, J1.J1Error)

CHECKS = 0


def check(c, w):
    global CHECKS
    assert c, w
    CHECKS += 1


def expect(fn, exc, w):
    global CHECKS
    try:
        fn()
    except exc as e:
        CHECKS += 1
        return e
    raise AssertionError(f"expected {exc.__name__}: {w}")


# ---------------------------------------------------------------- the tensor shim ---------------

class _T:
    """A numpy-backed stand-in for exactly the tensor operations the runner uses."""

    def __init__(self, a):
        self.a = np.asarray(a)

    @property
    def shape(self):
        return self.a.shape

    def reshape(self, *s):
        return _T(self.a.reshape(*s))

    def __getitem__(self, k):
        return _T(self.a[k])

    def __add__(self, o):
        return _T(self.a + (o.a if isinstance(o, _T) else o))

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self.a


class _FakeTorch:
    float32 = np.float32
    seeded: list[int] = []

    @staticmethod
    def as_tensor(v, dtype=None):
        return _T(np.asarray(v, dtype=dtype))

    @staticmethod
    def no_grad():
        return contextlib.nullcontext()

    @staticmethod
    def exp(t):
        return _T(np.exp(t.a))

    @classmethod
    def manual_seed(cls, s):
        cls.seeded.append(int(s))


def _dist_cls(offset):
    class D:
        def __init__(self, logits):
            self.logits = logits

        @classmethod
        def from_logits(cls, logits):
            return cls(logits)

        def to_deterministic(self):
            return self

        def sample(self):
            mean = self.logits[..., : self.logits.shape[-1] // 2]
            return mean if offset == 0.0 else mean + offset

    return D


class FakeModule:
    """forward_inference returns Gaussian logits: the mean followed by the log-std."""

    def __init__(self, tm, *, mean=(0.5, -0.25), log_std=(-2.0, -2.0), dist=True,
                 offset=0.0, n_actor=35, n_full=84):
        self.tm, self.dist, self.offset = tm, bool(dist), float(offset)
        self.mean = tuple(float(v) for v in mean)
        self.log_std = tuple(float(v) for v in log_std)
        self._n_actor, self._n_full = n_actor, n_full
        self.seen_obs: list[np.ndarray] = []

    def forward_inference(self, batch):
        obs = batch["obs"]
        self.seen_obs.append(np.asarray(obs.detach().cpu().numpy()).reshape(-1).copy())
        logits = np.asarray([list(self.mean) + list(self.log_std)], dtype=np.float32)
        return {"action_dist_inputs": self.tm.as_tensor(logits, dtype=self.tm.float32)}

    def get_inference_action_dist_cls(self):
        if not self.dist:
            raise RuntimeError("no inference distribution class: exercise the flat Box fallback")
        return _dist_cls(self.offset)


class FakeEnv:
    """A 500-step environment exposing PRODUCTION field names and the v3 counters."""

    def __init__(self, names, *, n_full=84, steps=500, penetration=0.0199, counts=None,
                 end_reason="episode_time_limit", stop_at=None, cancelled=0, resync=0,
                 timeout_at=None, timeout_side=1.0, nan_reward_at=None, drop_field=None,
                 knee=None, ankle=None, n_actor=None):
        self.names = tuple(names)
        self.n_full, self.steps, self.penetration = n_full, steps, penetration
        self.counts = counts or (lambda s: (min(3, s // 150), min(3, s // 150), min(2, s // 200)))
        self.end_reason, self.stop_at = end_reason, stop_at
        self.cancelled, self.resync = cancelled, resync
        self.timeout_at, self.timeout_side = timeout_at, timeout_side
        self.nan_reward_at, self.drop_field = nan_reward_at, drop_field
        self.knee = knee or (lambda s: -0.75 + 0.70 * (s - 1) / max(1, steps - 1))
        self.ankle = ankle or (lambda s: -0.20 + 0.35 * (s - 1) / max(1, steps - 1))
        self.action_space = types.SimpleNamespace(
            shape=(2,), low=np.full(2, -1.0, dtype=np.float32),
            high=np.full(2, 1.0, dtype=np.float32))
        self.unwrapped = types.SimpleNamespace(
            actor_feature_names=self.names,
            observation_feature_names=tuple(f"full_{i}" for i in range(n_full)),
            n_actor=(len(self.names) if n_actor is None else n_actor),
            env_cfg=types.SimpleNamespace(episode_duration=5.0, segment_duration=0.01),
            t=0.0)
        self.received: list[np.ndarray] = []
        self.step_index, self.closed, self.seed_used = 0, False, None

    def obs_at(self, step):
        v = np.zeros(self.n_full, dtype=np.float32)
        v[self.names.index("pros_knee_angle")] = self.knee(step)
        v[self.names.index("pros_ankle_angle")] = self.ankle(step)
        v[self.names.index("pros_knee_angle_vel")] = 0.1
        v[self.names.index("pros_ankle_angle_vel")] = -0.1
        return v

    def reset(self, seed=None):
        self.seed_used = seed
        self.step_index = 0
        self.unwrapped.t = 0.0
        return self.obs_at(1), {}

    def step(self, action):
        self.received.append(np.asarray(action, dtype=np.float64).reshape(-1).copy())
        self.step_index += 1
        s = self.step_index
        last = (s == self.steps) or (self.stop_at is not None and s == self.stop_at)
        hs, to, cycles = self.counts(s)
        timeout = 1.0 if (self.timeout_at is not None and s >= self.timeout_at) else 0.0
        side = self.timeout_side if timeout else 0.0
        fsm = {"resync_count": self.resync, "hs_cancelled_count": self.cancelled,
               "valid_cycle_count": cycles, "valid_hs_count": hs, "valid_to_count": to,
               "timeout_exceeded": timeout, "timeout_side": side,
               "fsm_behaviour_version": "v3", "event_source": "binary_active_v26"}
        rt = {"phase_valid_cycle_count": cycles, "phase_valid_hs_count": hs,
              "phase_valid_to_count": to, "phase_timeout_exceeded": timeout,
              "phase_timeout_side": side, "morphology_causal_failed_closed": 0.0,
              "grf_penetration_m": self.penetration, "reserve_norm_nm": 3.0}
        if self.drop_field:
            rt.pop(self.drop_field, None)
        self.unwrapped.t = round(s * 0.01, 10)
        reward = float("nan") if s == self.nan_reward_at else 1.0
        info = {"time": self.unwrapped.t, "reward_terms": rt, "phase_fsm": fsm,
                "end_reason": (self.end_reason if last else ""),
                "online_grf": {"left": {"fz": 120.0}}, "reset_diagnostics": {"ok": 1}}
        return self.obs_at(s + 1), reward, False, bool(last), info

    def close(self):
        self.closed = True


NAMES = tuple(json.loads((J3.J2_MODULE_DIR / "actor_feature_manifest.json").read_text()
                         )["actor_feature_names"])


def _stack(env, module, tm, **kw):
    return J3._Stack(name="fake", operational=False, torch_mod=tm,
                     load_module=lambda p: module, make_env=lambda cfg: env, **kw)


def _run(td, env, module, tm, *, leaf):
    return J3.run(authorized_stage=J3.STAGE, out_dir=Path(td) / leaf,
                  stack=_stack(env, module, tm), progress=False)


def main() -> int:
    tm = _FakeTorch
    j2_before = {n: J3._sha_file(J3.J2_LEAF / n) for n in J3.PIN_J2}
    with tempfile.TemporaryDirectory() as td:
        # ------------------------------------------------------------ refusals ------------------
        for stage in (None, "", "V26C-J3", "v26c-j3-closed-loop", "V26C-J2-BASE-FIT"):
            expect(lambda s=stage: J3.run(authorized_stage=s, out_dir=Path(td) / "x",
                                          stack=_stack(FakeEnv(NAMES), FakeModule(tm), tm),
                                          progress=False),
                   J3.J3Error, f"the token {stage!r} must be refused")
        expect(lambda: J3.run(authorized_stage=J3.STAGE, out_dir=None,
                              stack=_stack(FakeEnv(NAMES), FakeModule(tm), tm), progress=False),
               J3.J3Error, "an implicit output directory must be refused")
        check(not (Path(td) / "x").exists(), "and a refused call creates nothing")
        expect(lambda: J3.main(["--authorized-stage", "WRONG", "--out-dir", str(Path(td) / "y")]),
               J3.J3Error, "the CLI refuses a wrong token too")

        # ------------------------------------------------------------ the happy path ------------
        env, mod = FakeEnv(NAMES), FakeModule(tm)
        r = _run(td, env, mod, tm, leaf="ok")
        leaf = Path(td) / "ok"
        check(r["verdict"] == "PASS" and r["gate_pass"] is True,
              f"a conforming fake episode qualifies: {r['gate']['failed']}")
        check(r["summary"]["steps"] == 500 and r["summary"]["end_reason"] == "episode_time_limit",
              "500 steps ending on episode_time_limit")
        check(env.seed_used == 123 and 123 in _FakeTorch.seeded,
              "numpy, torch and env.reset are all seeded with 123")
        check(r["summary"]["valid_cycle_count"] == 2 and r["telemetry_integrity"]["pass"] is True,
              "with two valid cycles and self-consistent HS/TO evidence")
        check(r["qualification_technically_valid"] is True, "so the qualification is valid")
        check(sorted(p.name for p in leaf.iterdir() if p.is_file())
              == ["j3_kinematics.npz", "j3_trace.json", J3.RECEIPT_NAME],
              "the leaf holds the trace, the kinematics and the receipt")
        trace = json.loads((leaf / "j3_trace.json").read_text())
        check(len(trace) == 500, "the trace holds one row per step")
        row = trace[0]
        for field in ("step", "time_before", "time_after", "reward", "terminated", "truncated",
                      "end_reason", "actor_observation_vector_before", "raw_action", "policy_mean",
                      "policy_std_diagnostic", "applied_action_diagnostic",
                      "action_clipped_diagnostic", "action_selection_path", "reward_terms",
                      "phase_fsm", "prosthetic_state", "info_scalars"):
            check(field in row, f"every row records {field}")
        check(len(row["actor_observation_vector_before"]) == 35,
              "the recorded actor observation is the 35D vector the policy saw")
        check(isinstance(row["reward_terms"], dict) and isinstance(row["phase_fsm"], dict),
              "reward_terms and phase_fsm stay STRUCTURED, not stringified")
        check(row["phase_fsm"]["fsm_behaviour_version"] == "v3"
              and row["phase_fsm"]["event_source"] == "binary_active_v26",
              "and carry the v3 runtime identity")
        check(row["online_grf"] == {"left": {"fz": 120.0}},
              "relevant nested info is preserved recursively")
        check(row["prosthetic_state"]["pros_knee_angle"] < 0.0
              and abs(row["time_before"]) < 1e-12 and abs(row["time_after"] - 0.01) < 1e-12,
              "the per-step PRE-ACTION prosthetic state and both timestamps are recorded")
        check(r["kinematics"]["steps"] == 500
              and abs(r["kinematics"]["knee_min_rad"] - (-0.75)) < 1e-6,
              "the kinematics come from those same pre-action states")
        kin = np.load(leaf / "j3_kinematics.npz", allow_pickle=False)
        check(kin["knee_rad"].shape == (500,) and kin["ankle_rad"].shape == (500,),
              "and are saved alongside the trace")
        check(r["deterministic_semantics"]["max_abs_action_minus_mean"] == 0.0
              and r["summary"]["realized_noise_rms"] == [0.0, 0.0]
              and r["summary"]["action_noise_sigma"] == [0.0, 0.0],
              "the deterministic action IS the mean and the realised noise RMS is EXPLICITLY zero")
        check(r["deployable"] is False and r["promotion"] == "NONE"
              and r["next_stage_authorized"] is False,
              "a PASS is not deployable, not promoted and authorises no next stage")
        check(r["j1_lineage_preserved"]["j1_verdict"] == "FAIL",
              "and the J1 FAIL is carried into the J3 receipt unchanged")
        check(r["inputs_sha256"]["j2_artefacts"] == J3.PIN_J2
              and r["inputs_sha256"]["j1_receipt"] == J3.PIN_J1_RECEIPT
              and r["inputs_sha256"]["j1_amendment"] == J3.PIN_J1_AMENDMENT
              and r["inputs_sha256"]["pinned_runtime_config"] == J3.PIN_RUNTIME_CONFIG_SHA,
              "the receipt pins every input by hash")
        check(set(r["outputs_sha256"]) == {"j3_trace.json", "j3_kinematics.npz"}
              and r["outputs_sha256"]["j3_trace.json"] == J3._sha_file(leaf / "j3_trace.json"),
              "and hashes every output it wrote")
        check(r["stack"]["injected"] is True and r["stack"]["operational"] is False
              and "NOT operational evidence" in r["stack"]["note"],
              "a receipt produced with an injected test stack says so")
        check(r["quarantine"]["applies"] is False and "FORBIDDEN" in r["quarantine"]["retry"],
              "a PASS is not quarantined, and retry stays forbidden either way")
        check(env.closed is True, "the environment is closed afterwards")

        # ------------------------------------------------------------ the RAW action ------------
        env2 = FakeEnv(NAMES)
        mod2 = FakeModule(tm, mean=(1.5, -1.25))          # deliberately outside the action box
        r2 = _run(td, env2, mod2, tm, leaf="raw")
        received = np.asarray(env2.received)
        check(received.shape == (500, 2) and np.allclose(received[:, 0], 1.5)
              and np.allclose(received[:, 1], -1.25),
              "the environment is stepped with the RAW action, unclipped")
        t2 = json.loads((Path(td) / "raw" / "j3_trace.json").read_text())
        check(t2[0]["raw_action"] == [1.5, -1.25]
              and t2[0]["applied_action_diagnostic"] == [1.0, -1.0]
              and t2[0]["action_clipped_diagnostic"] is True
              and t2[0]["stepped_with"] == "raw_action",
              "while the clipped action is recorded only as a diagnostic mirror")
        check(r2["summary"]["action_clipped_steps"] == 500 and r2["verdict"] == "PASS"
              and r2["diagnostics_not_binding"]["action_clipped_steps"] == 500,
              "500 clipped steps are counted, reported as diagnostics and change no verdict")
        check(np.array_equal(np.asarray(mod2.seen_obs[0]).reshape(-1),
                             np.asarray(env2.obs_at(1)).reshape(-1)),
              "and the policy sees the FULL 84D observation, not the 35D slice")

        # ------------------------------------------------------------ the deterministic path ----
        r3 = _run(td, FakeEnv(NAMES), FakeModule(tm, dist=False), tm, leaf="fallback")
        t3 = json.loads((Path(td) / "fallback" / "j3_trace.json").read_text())
        check(trace[0]["action_selection_path"] == "inference_action_dist.to_deterministic()"
              ".sample()",
              "the normal path is the inference distribution's deterministic sample")
        check(t3[0]["action_selection_path"] == "flat_box_fallback_mean",
              "a module without an inference distribution falls back to the Gaussian mean")
        check(t3[0]["raw_action"] == trace[0]["raw_action"],
              "and both paths select the same action")
        check(t3[0]["policy_std_diagnostic"] == [float(np.float32(np.exp(-2.0)))] * 2,
              "the policy std is recorded as a diagnostic, never used to sample")
        check(r3["deterministic_semantics"]["rollout_eval_parity_checked"] is False,
              "with a fake stack there is no rollout_eval parity to claim")
        expect(lambda: _run(td, FakeEnv(NAMES), FakeModule(tm, offset=1e-6), tm, leaf="noisy"),
               J3.J3Error, "an action that differs from the mean is refused as noise")
        check(not (Path(td) / "noisy" / "j3_trace.json").exists(),
              "and no trace is written for it")
        try:
            import torch as _torch
        except Exception:
            _torch = None
        if _torch is not None:
            rr = _run(td, FakeEnv(NAMES), FakeModule(_torch), _torch, leaf="torch")
            tt = json.loads((Path(td) / "torch" / "j3_trace.json").read_text())
            check(rr["verdict"] == "PASS" and tt[0]["raw_action"] == trace[0]["raw_action"],
                  "REAL torch selects the same deterministic action as the shim")
        else:
            check(True, "torch is unavailable here; the shim path was exercised instead")

        # ------------------------------------------------------------ the runtime contract ------
        expect(lambda: _run(td, FakeEnv(NAMES, n_actor=33), FakeModule(tm), tm, leaf="c1"),
               J3.J3Error, "a 33D actor must be refused")
        expect(lambda: _run(td, FakeEnv(NAMES, n_full=80), FakeModule(tm), tm, leaf="c2"),
               J3.J3Error, "an 80D observation must be refused")
        bad = list(NAMES)
        bad[7] = "renamed_feature"
        expect(lambda: _run(td, FakeEnv(tuple(bad)), FakeModule(tm), tm, leaf="c3"),
               J3.J3Error, "a runtime manifest differing from the pinned J2 manifest is refused")
        expect(lambda: _run(td, FakeEnv(NAMES), FakeModule(tm, n_full=88), tm, leaf="c4"),
               J3.J3Error, "a module declaring n_full=88 is refused")
        expect(lambda: _run(td, FakeEnv(NAMES), FakeModule(tm, n_actor=39), tm, leaf="c5"),
               J3.J3Error, "a module declaring n_actor=39 is refused")

        # ------------------------------------------------------------ fail-closed telemetry -----
        expect(lambda: _run(td, FakeEnv(NAMES, nan_reward_at=3), FakeModule(tm), tm, leaf="nan"),
               FAILCLOSED, "a non-finite reward aborts before it can be serialised")
        expect(lambda: _run(td, FakeEnv(NAMES, drop_field="grf_penetration_m"), FakeModule(tm),
                            tm, leaf="drop"),
               FAILCLOSED, "a missing reward_terms field is never defaulted to zero")
        expect(lambda: _run(td, FakeEnv(NAMES, end_reason=""), FakeModule(tm), tm, leaf="noend"),
               FAILCLOSED, "an absent end_reason is never assumed to be a success")

        # ------------------------------------------------------------ ordinary FAIL -------------
        rf = _run(td, FakeEnv(NAMES, penetration=0.0201), FakeModule(tm), tm, leaf="fail")
        check(rf["verdict"] == "FAIL" and rf["gate"]["failed"] == ["max_penetration_m"],
              "a penetration of 20.1 mm fails the 0.020 m gate")
        check((Path(td) / "fail" / "j3_trace.json").exists()
              and (Path(td) / "fail" / J3.RECEIPT_NAME).exists()
              and rf["quarantine"]["applies"] is True
              and rf["quarantine"]["artefacts_preserved"] is True,
              "the evidence is PRESERVED and quarantined, not deleted or retried")
        check(rf["deployable"] is False and rf["promotion"] == "NONE"
              and rf["next_stage_authorized"] is False, "and it promotes nothing")
        rs = _run(td, FakeEnv(NAMES, stop_at=120, end_reason="phase_timeout:stance",
                              timeout_at=120), FakeModule(tm), tm, leaf="short")
        check(rs["verdict"] == "FAIL" and rs["summary"]["steps"] == 120
              and rs["summary"]["phase_timeout_stance"] >= 1
              and set(rs["gate"]["failed"]) >= {"steps", "end_reason", "phase_timeout_stance"},
              "an early stance timeout fails on the steps, the end reason and the timeout")

        # ------------------------------------------------------------ INVALID telemetry ---------
        ri = _run(td, FakeEnv(NAMES, counts=lambda s: (0, min(3, s // 150), min(2, s // 200))),
                  FakeModule(tm), tm, leaf="invalid")
        check(ri["verdict"] == "INVALID" and ri["gate_pass"] is True,
              "contradictory HS/TO evidence yields INVALID even when the behaviour would pass")
        check(ri["qualification_technically_valid"] is False
              and ri["telemetry_integrity"]["pass"] is False
              and ri["quarantine"]["applies"] is True,
              "and the receipt says the qualification is technically invalid, and quarantines it")

        # ------------------------------------------------------------ no-clobber ----------------
        expect(lambda: _run(td, FakeEnv(NAMES), FakeModule(tm), tm, leaf="ok"),
               J3.J3Error, "no-clobber: an existing leaf is never overwritten")
        check(json.loads((leaf / J3.RECEIPT_NAME).read_text())["outputs_sha256"]
              == r["outputs_sha256"],
              "and the existing leaf is left byte-identical")

        # ------------------------------------------------------------ failure leaves no leaf ----
        def _boom(cfg):
            raise RuntimeError("the environment could not be constructed")

        expect(lambda: J3.run(authorized_stage=J3.STAGE, out_dir=Path(td) / "boom",
                              stack=J3._Stack(name="fake", operational=False, torch_mod=tm,
                                              load_module=lambda p: FakeModule(tm),
                                              make_env=_boom),
                              progress=False),
               RuntimeError, "a construction failure propagates")
        check(not (Path(td) / "boom").exists(),
              "and removes only the empty leaf it had just created")
        for protected in (J3.J2_LEAF, J3.J2_MODULE_DIR, HERE, J3.J1_LEAF):
            expect(lambda p=protected: J3._remove_leaf(p), J3.J3Error,
                   f"_remove_leaf refuses to touch {protected.name}")

    check({n: J3._sha_file(J3.J2_LEAF / n) for n in J3.PIN_J2} == j2_before == J3.PIN_J2,
          "MEASURED: the immutable J2 leaf is byte-identical before and after this suite")
    # ONE authorised J3 rollout has been executed. Its leaf is pinned; no second run may appear.
    AUTHORISED_J3_LEAF = "j3_base_v26c_2026-08-26_r1"
    J3_PINS = {
        "v26c_j3_closed_loop_receipt.json":
            "34d856b0b4acabd000a1e6257767c6049a1f9f2147eb99d6f3801ca7559ff422",
        "j3_trace.json":
            "b36f85dc0b6aa8c0fa6d6d6b404ae8fdd51528129c3aeac5004451ec6d4bcbae",
        "j3_kinematics.npz":
            "402040d7cc794a26213ece8abd04a6e945fa46050f20d4ecc402bf78df0b97fc",
    }
    leaves = sorted(p.name for p in J3.OUT_ROOT.iterdir()) if J3.OUT_ROOT.is_dir() else []
    check(leaves in ([], [AUTHORISED_J3_LEAF]),
          f"MEASURED: at most the ONE authorised J3 leaf exists; found {leaves}")
    if leaves:
        leaf3 = J3.OUT_ROOT / AUTHORISED_J3_LEAF
        top = sorted(p.name for p in leaf3.iterdir() if p.is_file())
        check(top == sorted(J3_PINS),
              f"holding exactly the three pinned artefacts; found {top}")
        for n, pin in J3_PINS.items():
            check(J3._sha_file(leaf3 / n) == pin, f"and {n} is byte-identical to its pin")
        r3 = json.loads((leaf3 / "v26c_j3_closed_loop_receipt.json").read_text())
        check(r3["verdict"] == "FAIL" and r3["gate"]["failed"] == ["max_penetration_m"]
              and r3["deployable"] is False and r3["promotion"] == "NONE"
              and r3["next_stage_authorized"] is False
              and r3["quarantine"]["applies"] is True,
              "the executed rollout stays FAIL, undeployable, unpromoted and quarantined")
    print(json.dumps({"selftest": "PASS", "checks": CHECKS}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
