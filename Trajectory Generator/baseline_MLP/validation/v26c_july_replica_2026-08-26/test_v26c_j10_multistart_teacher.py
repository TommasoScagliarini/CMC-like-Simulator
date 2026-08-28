"""Fail-closed tests for the V26C J10 multistart prescribed-teacher collection.

Pure and synthetic: no physical environment is ever constructed. A fake stack drives the whole
two-cell collection so the write path, the gates, the coverage criterion and the provenance rules
can be exercised end to end in a temporary root. The authorised J10 leaf is never created and the
two real rollouts are never launched.
"""
from __future__ import annotations
import ast, builtins, io, json, os, shutil, sys, tempfile, tokenize
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26c_j10_multistart_teacher as J10  # noqa: E402
import v26c_j1_collect as J1  # noqa: E402
import v26c_j3_closed_loop as J3  # noqa: E402
import v26c_penetration_contract as PC  # noqa: E402

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


def snapshot(root: Path) -> dict[str, str]:
    out: dict[str, str] = {}
    for p in sorted(root.rglob("*")):
        if "__pycache__" in p.parts or not p.is_file():
            continue
        out[str(p.relative_to(root))] = J10._sha_file(p)
    return out


# ---------------------------------------------------------------- synthetic material ------------

NAMES = ("gait_phase_sin", "gait_phase_cos", "pros_knee_angle", "pros_knee_angle_vel",
         "pros_ankle_angle", "pros_ankle_angle_vel", "SEA_Knee_motor_angle", "SEA_Knee_motor_speed",
         "SEA_Ankle_motor_angle", "SEA_Ankle_motor_speed", "online_left_normal_grf_bw",
         "online_left_in_contact", "online_left_heel_strike", "online_left_toe_off",
         "online_left_gait_phase_sin", "online_left_gait_phase_cos",
         "online_left_cycle_duration_s", "phase_fsm_wait_hs", "phase_fsm_stance_after_hs",
         "phase_fsm_swing_after_to", "phase_expected_hs", "phase_expected_to",
         "phase_stance_elapsed_norm", "phase_swing_elapsed_norm", "phase_cycle_progress_credit",
         "pros_knee_angle_previous_endpoint", "pros_knee_angle_served_ref",
         "pros_knee_angle_served_ref_vel", "pros_knee_angle_served_ref_accel",
         "pros_knee_angle_sea_u", "pros_ankle_angle_previous_endpoint",
         "pros_ankle_angle_served_ref", "pros_ankle_angle_served_ref_vel",
         "pros_ankle_angle_served_ref_accel", "pros_ankle_angle_sea_u")


def good_summary(**over):
    s = {"steps": 500, "end_reason": "episode_time_limit", "valid_cycle_count": 2,
         "valid_hs_count": 3, "valid_to_count": 3, "phase_timeout_stance": 0,
         "phase_timeout_swing": 0, "morphology_causal_contract_failure": 0,
         "hs_cancelled_count": 0, "resync_count": 0, "action_clipped_steps": 0,
         "episode_return": 42.0, "realized_noise_rms": [0.0, 0.0],
         "action_noise_sigma": [0.0, 0.0], "max_reserve_norm_nm": 520.0,
         "mean_reserve_norm_nm": 150.0, "slew_limited_steps": 150}
    s.update(over)
    return s


def good_kin(n=500):
    return np.linspace(-1.0, -0.2, n), np.linspace(-0.35, 0.05, n)


def pen_ok(maxv=0.0229, n=500):
    a = np.full(n, 0.001)
    a[10] = maxv
    return a


_UNSET = object()


class FakeSpace:
    def __init__(self, shape=(2,)):
        self.shape = shape
        self.low = np.full(shape, -1.0, dtype=np.float32)
        self.high = np.full(shape, 1.0, dtype=np.float32)


class FakeSimCfg:
    def __init__(self, t_start=11.99, t_end=30.0):
        self.t_start = float(t_start)
        self.t_end = float(t_end)
        self.pros_coords = ("pros_knee_angle", "pros_ankle_angle")


class FakeEnvCfg:
    def __init__(self, duration=5.0):
        self.episode_duration = duration
        self.segment_duration = 0.01


class FakeBase:
    def __init__(self, t0, names, duration=5.0):
        self.t = float(t0)
        self.cfg = FakeSimCfg()
        self.env_cfg = FakeEnvCfg(duration)
        self._episode_end = float(t0) + 5.0
        self.n_actor = len(names)
        self.actor_feature_names = tuple(names)


class FakeEnv:
    built: list[dict] = []
    closed: list[str] = []
    saved: list[str] = []

    t_start = 11.99
    t_end = 30.0
    close_raises = False
    names = NAMES
    wait_hs_steps = 0          # how many steps report phase_fsm_wait_hs == 1
    clip_steps = 0             # how many steps emit an out-of-bounds teacher action

    def __init__(self, cfg):
        self.cfg = dict(cfg)
        self.offset = float(cfg["episode_start_offset_s"])
        self.start = J10.expected_reset_time(FakeEnv.t_start, FakeEnv.t_end,
                                             cfg.get("episode_duration"),
                                             self.offset)["expected_reset_time_s"]
        self.unwrapped = FakeBase(self.start, FakeEnv.names, cfg.get("episode_duration"))
        self.action_space = FakeSpace()
        self.n = 0
        self.record_outputs = bool(cfg.get("record_outputs", False))
        self.save_outputs_on_close = bool(cfg.get("save_outputs_on_close", False))
        self.output_prefix = cfg.get("output_prefix")
        self.out = Path(cfg["output_dir"])
        FakeEnv.built.append({"offset": self.offset, "output_dir": str(self.out),
                              "record_outputs": self.record_outputs,
                              "save_outputs_on_close": self.save_outputs_on_close})

    def reset(self, seed=None):
        self.n = 0
        self.unwrapped.t = self.start
        return self._obs(), {}

    def _obs(self):
        v = np.zeros(len(FakeEnv.names), dtype=np.float32)
        if self.n < FakeEnv.wait_hs_steps:
            v[FakeEnv.names.index("phase_fsm_wait_hs")] = 1.0
        knee, ankle = good_kin()
        i = min(self.n, 499)
        v[FakeEnv.names.index("pros_knee_angle")] = float(knee[i])
        v[FakeEnv.names.index("pros_ankle_angle")] = float(ankle[i])
        return v

    def step(self, action):
        self.n += 1
        self.unwrapped.t = self.start + 0.01 * self.n
        info = {"time": self.unwrapped.t,
                "end_reason": "episode_time_limit" if self.n >= 500 else "",
                "reward_terms": {"grf_penetration_m": float(pen_ok()[self.n - 1])},
                J1.FSM_KEY: {"behaviour_version": J1.EXPECTED_FSM_BEHAVIOUR_VERSION}}
        return self._obs(), 0.08, False, self.n >= 500, info

    def close(self):
        if FakeEnv.close_raises:
            raise RuntimeError("synthetic close failure")
        if self.record_outputs and self.save_outputs_on_close:
            self.out.mkdir(parents=True, exist_ok=True)
            prefix = self.output_prefix or "rl_episode"
            for k in range(19):
                (self.out / f"{prefix}_{k:02d}.sto").write_text("synthetic", encoding="utf-8")
            FakeEnv.saved.append(str(self.out))
        FakeEnv.closed.append(str(self.cfg["output_dir"]))


def fake_teacher(base, target_t, lookahead_s=0.0):
    """A deterministic prescribed action. Emits an out-of-bounds value on demand."""
    if FakeEnv.clip_steps and getattr(base, "_clip_left", 0) > 0:
        base._clip_left -= 1
        return np.array([2.5, -2.5], dtype=np.float32)
    return np.array([0.10, -0.05], dtype=np.float32)


class FakeHeld:
    def __init__(self, rng, shape, hold):
        self.rng, self.shape = rng, shape
        assert int(hold) == 1

    def next(self):
        return self.rng.standard_normal(self.shape).astype(np.float32)


def make_stack():
    return J10._Stack(name="synthetic", operational=False,
                      make_env=lambda cfg: FakeEnv(cfg),
                      teacher_action=fake_teacher, held_normal=FakeHeld)


def main() -> int:
    src = (HERE / "v26c_j10_multistart_teacher.py").read_text()
    tree = ast.parse(src)
    ids = {t.string for t in tokenize.generate_tokens(io.StringIO(src).readline)
           if t.type == tokenize.NAME}
    prereg = json.loads(J10.PREREG.read_text())
    before_all = snapshot(HERE)

    # ---------------------------------------------------------------- preregistration ----------
    check(prereg["kind"] == "ADDITIVE, IMMUTABLE, PREREGISTRATION"
          and prereg["stage_proposed"] == "V26C_J10_MULTISTART_TEACHER" == J10.STAGE,
          "the prereg is additive, immutable and PROPOSES the J10 stage")
    check("stage_authorised" not in prereg
          and prereg["authorisation_status"].startswith("NOT_GRANTED"),
          "it does not claim an authorisation it cannot grant")
    check(J10._sha_file(J10.PREREG) == J10.PIN_PREREG
          == "79b1b573eb45831e7333c3dbc539f76a7e9555986b38e73ea83a91590f8241d7",
          "and is pinned by exact hash")
    check(len(prereg["pinned_artefacts_sha256"]) == 7
          and all(J10._sha_file(HERE / r) == h
                  for r, h in prereg["pinned_artefacts_sha256"].items()),
          "all 7 local pins verify on disk")
    check(len(prereg["pinned_repo_artefacts_sha256"]) == 6
          and all(J10._sha_file(J10.REPO / r) == h
                  for r, h in prereg["pinned_repo_artefacts_sha256"].items()),
          "all 6 repo pins verify, including the parent and the teacher module")
    check(prereg["pinned_repo_artefacts_sha256"][
        "Trajectory Generator/runs/training/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter"
        "/rl_module_best/module_state.pkl"] == J10.PIN_PARENT_STATE_SHA
        == "0ba56eb703a238de41afd10d079c1cd59903ba20189e24d43b5c3a363cde15bd",
        "the architect's parent hash is the pinned one")
    check(prereg["pinned_repo_artefacts_sha256"][
        "Trajectory Generator/baseline_MLP/target_domain_imitation.py"]
        == J10._sha_file(J10.REPO / "Trajectory Generator/baseline_MLP/target_domain_imitation.py"),
        "the LABEL SOURCE module is pinned: a change to it changes every label")
    check(prereg["architect_decision"]["decision"].startswith("apply the July MULTISTART"),
          "the architect's decision is recorded verbatim as the premise")
    check("NEVER loaded" in prereg["operational_parent"]["j8_student_excluded"]
          and "label" in prereg["operational_parent"]["j8_student_excluded"],
          "and the J8 student is explicitly excluded as a label source")

    # ---------------------------------------------------------------- the matrix ---------------
    check(len(J10.MATRIX) == 2 and [c["id"] for c in J10.MATRIX] == ["B", "C"],
          "exactly two cells, in the frozen order B then C")
    check(J10.MATRIX[0]["offset_s"] == 1.756870983805102
          and J10.MATRIX[1]["offset_s"] == 2.156870983805102,
          "at the two architect-specified offsets")
    check(J10.OFFSET_UNIT == "seconds"
          and abs((J10.OFFSET_MINUS - J10.OFFSET_NOMINAL) + 0.20) < 1e-12
          and abs((J10.OFFSET_PLUS - J10.OFFSET_NOMINAL) - 0.20) < 1e-12,
          "the unit is SECONDS and the deltas are exactly -0.20 and +0.20")
    check(J10.OFFSET_NOMINAL not in J10.FROZEN_OFFSETS,
          "the nominal start is NOT re-collected: it already exists as the J1 leaf")
    check(J10.ROLLOUT_SEED == 123 and J10.SIGMA == (0.0, 0.0)
          and J10.TEACHER_LOOKAHEAD_S == 0.0 and J10.EXPECTED_STEPS == 500,
          "seed 123, sigma exactly zero, lookahead 0.0, 500 steps")

    # ---------------------------------------------------------------- parent + label source ----
    parent = J10.verify_parent()
    check(parent["module_state_sha256"] == J10.PIN_PARENT_STATE_SHA
          and parent["same_parent_as_j1"] is True,
          "the parent is the August V26 imitative, the same one J1 used")
    check(parent["j8_student_excluded"]["used_as_parent"] is False
          and parent["j8_student_excluded"]["used_as_policy"] is False
          and parent["j8_student_excluded"]["used_as_label_source"] is False,
          "the J8 student is excluded on all three counts")
    saved_pin = J10.PIN_PARENT_STATE_SHA
    try:
        J10.PIN_PARENT_STATE_SHA = "0" * 64
        expect(J10.verify_parent, Exception, "a changed parent must be refused")
    finally:
        J10.PIN_PARENT_STATE_SHA = saved_pin
    # the 39 vs 35 provenance fact, recorded rather than tripped over
    w = J10.parent_manifest_width()
    check(w["parent_manifest_feature_count"] == 39 and w["runtime_actor_width"] == 35,
          "the parent manifest declares 39 names while the runtime actor block is 35 wide")
    check(w["coverage_feature_in_parent_manifest"] is True,
          "the coverage feature exists in both the 39-wide manifest and the 35-wide runtime")
    check("never loaded" in w["parent_is_provenance_only"]
          and "cannot affect a single recorded value" in w["parent_is_provenance_only"],
          "and the parent is pinned for lineage only; it is never loaded, so the width "
          "difference cannot affect any recorded value")
    check("actor_feature_names()" not in src and "runtime_feature_names" in src,
          "the runner resolves the feature names from the LIVE ENV, never from that 39-wide "
          "manifest")
    tf = [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
          and n.name == "runtime_feature_names"]
    check(tf and "getattr" in ast.dump(tf[0]), "runtime_feature_names reads them off the env")

    # ---------------------------------------------------------------- offset-only mutation -----
    SENTINEL = HERE / "_j10_test_sentinel_never_created"
    base, cfg = J10.base_env_config(SENTINEL)
    check(base["record_outputs"] is True and base["save_outputs_on_close"] is True
          and bool(base.get("output_prefix")),
          "the base carries recording ON and an output_prefix")
    e = expect(lambda: J10.base_env_config(None), J10.J10Error, "a None root must be refused")
    check("REAL output root" in str(e), "naming the J9 defect it prevents")
    for c in J10.MATRIX:
        env_c, m = J10.cell_env_config(base, float(c["offset_s"]), SENTINEL / f"cell_{c['id']}")
        check(set(m["mutated_keys"]) <= {"episode_start_offset_s", "output_dir"},
              f"cell {c['id']}: only the offset and output_dir differ")
        check(m["stable_keys_count"] == len(base) - 2 and m["stable_keys_all_identical"] is True,
              f"cell {c['id']}: every STABLE key is identical, counted excluding BOTH mutable keys")
        check(env_c["record_outputs"] is True and env_c["save_outputs_on_close"] is True
              and env_c["output_prefix"] == base["output_prefix"],
              f"cell {c['id']}: recording and prefix are inherited, never re-declared")
        naive = [k for k in base if k != "episode_start_offset_s" and base[k] != env_c.get(k)]
        check(naive == ["output_dir"],
              "the naive J9-style exclusion would have flagged output_dir - which is why the "
              "guard excludes both declared-mutable keys")
    e = expect(lambda: J10.cell_env_config(base, J10.OFFSET_NOMINAL, SENTINEL / "x"),
               J10.J10Error, "the NOMINAL offset is not one of the two frozen ones")
    check("not one of the two frozen offsets" in str(e), "naming the frozen pair")
    check(not SENTINEL.exists(), "MEASURED: building configs created no directory")
    vf = [n for n in ast.walk(next(x for x in ast.walk(tree) if isinstance(x, ast.FunctionDef)
                                   and x.name == "cell_env_config"))
          if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
          and n.func.attr == "verify_env_config"]
    check(not vf, "STATIC: cell_env_config never re-runs J1.verify_env_config on the mutated config")

    # ---------------------------------------------------------------- reset arithmetic ---------
    r = J10.expected_reset_time(11.99, 30.0, 5.0, J10.OFFSET_MINUS)
    check(r["expected_reset_time_s"] == 11.99 + J10.OFFSET_MINUS == 13.746870983805103,
          f"B resets at t_start + offset = 13.746870983805103 ({r['expected_reset_time_s']})")
    r2 = J10.expected_reset_time(11.99, 30.0, 5.0, J10.OFFSET_PLUS)
    check(r2["expected_reset_time_s"] == 11.99 + J10.OFFSET_PLUS == 14.146870983805101,
          f"C resets at 14.146870983805101 ({r2['expected_reset_time_s']})")
    check(r["expected_reset_time_s"] != J10.OFFSET_MINUS,
          "PROVEN: the expected reset time is NOT the offset itself")
    cl = J10.expected_reset_time(11.99, 14.0, 5.0, J10.OFFSET_MINUS)
    check(cl["clamped"] is True and cl["expected_reset_time_s"] == 11.99,
          "the clamp to max_start is reproduced")
    check(J10.RESET_TIME_TOLERANCE_S == 1e-9, "the tolerance is 1e-9 s")

    # ---------------------------------------------------------------- the gates ----------------
    contract = PC.load_contract()
    knee, ankle = good_kin()
    pen = J10.penetration_report(pen_ok(), contract, "t")
    g = J10.evaluate_cell_gate(good_summary(), knee, ankle, pen)
    check(g["pass"] is True and g["failed"] == [], "a good cell PASSES every binding criterion")
    check("action_clipped_steps" in g["checks"] and g["action_clipping_is_binding"] is True,
          "action clipping is a BINDING criterion in this stage, unlike J1")
    bad = J10.evaluate_cell_gate(good_summary(action_clipped_steps=1), knee, ankle, pen)
    check(bad["checks"]["action_clipped_steps"] is False and bad["pass"] is False,
          "a single clipped step FAILS: a clipped label is not the label that was executed")
    for key, over in (("steps", {"steps": 499}), ("end_reason", {"end_reason": "terminated"}),
                      ("valid_cycles", {"valid_cycle_count": 1}),
                      ("phase_timeout_stance", {"phase_timeout_stance": 1}),
                      ("phase_timeout_swing", {"phase_timeout_swing": 1}),
                      ("morphology_causal_contract_failure",
                       {"morphology_causal_contract_failure": 1}),
                      ("hs_cancelled_count", {"hs_cancelled_count": 1}),
                      ("resync_count", {"resync_count": 2})):
        b = J10.evaluate_cell_gate(good_summary(**over), knee, ankle, pen)
        check(b["checks"][key] is False and key in b["failed"], f"the gate catches {key}")
    check(J10.J10_KINEMATIC_GATE == dict(J3.J3_KINEMATIC_GATE),
          "the kinematic gate is J3's, IDENTICAL to J9R1 and unchanged")
    shallow = np.linspace(-0.0099, 0.3901, 500)
    check(J10.evaluate_cell_gate(good_summary(), knee, shallow, pen)
          ["checks"]["kinematic_ankle_min"] is False, "ankle min -0.0099 FAILS")
    exact = np.linspace(-0.03, 0.37, 500)
    check(J10.evaluate_cell_gate(good_summary(), knee, exact, pen)
          ["checks"]["kinematic_ankle_min"] is True, "ankle min EXACTLY -0.03 PASSES")
    check(J10.penetration_report(pen_ok(0.028), contract, "t")["binding_pass"] is True
          and J10.penetration_report(pen_ok(0.0280001), contract, "t")["binding_pass"] is False,
          "EXACTLY 0.028 PASSES, above FAILS")
    b25 = J10.penetration_report(pen_ok(0.0269), contract, "t")
    check(b25["flags"]["above_soft_diagnostic"] is True and b25["binding_pass"] is True,
          "the 20 and 25 mm bands are DIAGNOSTIC")
    bands = {0.020, 0.025, 0.028}
    compared = [n for n in ast.walk(tree) if isinstance(n, ast.Compare)
                and any(isinstance(c, ast.Constant) and isinstance(c.value, float)
                        and c.value in bands for c in n.comparators + [n.left])]
    check(not compared, "AST: the runner compares against no penetration band")

    # ---------------------------------------------------------------- the coverage gate --------
    cov = J10.evaluate_coverage([{"id": "B", "wait_hs_rows": 12}, {"id": "C", "wait_hs_rows": 0}])
    check(cov["pass"] is True and cov["total_rows"] == 12 and cov["binding"] is True,
          "the coverage gate passes when the collection holds at least one WAIT_HS row")
    check(cov["scope"].startswith("collection-wide"),
          "and it is COLLECTION-WIDE, not per cell: one cell may legitimately hold none")
    none = J10.evaluate_coverage([{"id": "B", "wait_hs_rows": 0}, {"id": "C", "wait_hs_rows": 0}])
    check(none["pass"] is False,
          "a collection with zero WAIT_HS rows FAILS: it would not close the gap it exists to "
          "close")
    one = J10.evaluate_coverage([{"id": "B", "wait_hs_rows": 1}, {"id": "C", "wait_hs_rows": 0}])
    check(one["pass"] is True and J10.COVERAGE_MIN_ROWS == 1,
          "exactly ONE row is enough: no further threshold is invented")
    check(J10.COVERAGE_FEATURE == "phase_fsm_wait_hs"
          and "no minimum count beyond one" in cov["no_invented_thresholds"],
          "the feature is the measured J7 gap and no extra threshold is asserted")

    # ---------------------------------------------------------------- inert preflight ----------
    pre = J10.preflight()
    check(pre["verdict"] == "GO" and pre["blockers"] == [] and pre["read_only"] is True,
          "the preflight is GO and read-only")
    check(snapshot(HERE) == before_all,
          "MEASURED: not one byte under the validation root changed during the preflight")
    check(pre["inert"]["environment_constructed"] is False
          and pre["inert"]["policy_loaded"] is False
          and pre["inert"]["student_rollout"] is False
          and pre["inert"]["fit_executed"] is False
          and pre["inert"]["critic_touched"] is False
          and pre["inert"]["ppo_updates"] == 0,
          "no env, no policy, no student rollout, no fit, no critic, no PPO")
    check(pre["inert"]["heavy_modules_introduced_by_preflight"] == []
          and not any(m in sys.modules for m in ("torch", "ray", "opensim", "env_factory")),
          "MEASURED: no heavy module imported")
    check(pre["runtime"]["rllib_checkpoint_loader"] is False
          and pre["runtime"]["ray_cluster_started"] is False
          and pre["runtime"]["env_runners"] is False,
          "no RLlib loader at all: the action source is prescribed, not a policy")
    check(pre["preflight_sentinel"]["created_by_the_preflight"] is False
          and not J10.PREFLIGHT_SENTINEL.exists(),
          "and the sentinel output root was never created")
    tripped: list[str] = []
    real = {"open": builtins.open, "rename": os.rename, "mkdir": Path.mkdir,
            "write_text": Path.write_text, "rmtree": shutil.rmtree, "savez": np.savez_compressed}

    def boom(tag):
        def f(*a, **k):
            tripped.append(tag)
            raise AssertionError(f"the preflight attempted {tag}")
        return f

    def guarded_open(file, mode="r", *a, **k):
        if any(m in str(mode) for m in ("w", "a", "x", "+")):
            tripped.append(f"open({mode})")
            raise AssertionError("write during preflight")
        return real["open"](file, mode, *a, **k)

    os.rename, Path.mkdir = boom("os.rename"), boom("Path.mkdir")
    Path.write_text, shutil.rmtree = boom("Path.write_text"), boom("shutil.rmtree")
    np.savez_compressed = boom("np.savez_compressed")
    builtins.open = guarded_open
    real_import = builtins.__import__
    banned = {"torch", "ray", "env_factory", "opensim", "target_domain_imitation", "gymnasium"}

    def guarded_import(name, *a, **k):
        if name.split(".")[0] in banned:
            tripped.append(f"import {name}")
            raise AssertionError(f"heavy import {name}")
        return real_import(name, *a, **k)

    builtins.__import__ = guarded_import
    try:
        J10.preflight()
    finally:
        os.rename, Path.mkdir = real["rename"], real["mkdir"]
        Path.write_text, shutil.rmtree = real["write_text"], real["rmtree"]
        np.savez_compressed = real["savez"]
        builtins.open, builtins.__import__ = real["open"], real_import
    check(tripped == [], f"PROVEN INERT: no write primitive, no heavy import ({tripped})")
    e = expect(lambda: J10.main(["--out", "/tmp/x"]), J10.J10Error,
               "--out without --collect must be refused")
    check("meaningless without --collect" in str(e), "with a reason")

    # ---------------------------------------------------------------- destination guards -------
    leaf = J10.authorized_leaf()
    check(leaf == HERE / "j10_runs" / "j10_multistart_teacher_v26c_2026-08-27_r1",
          "the authorised leaf is the declared one")
    e = expect(lambda: J10.collect(str(leaf), None), J10.J10Error, "a missing stage token")
    check("--authorized-stage must be exactly" in str(e), "naming the exact token")
    for bad_tok in ("V26C_J9R1_CLOSED_LOOP", "v26c_j10_multistart_teacher", ""):
        expect(lambda b=bad_tok: J10.collect(str(leaf), b), J10.J10Error,
               f"the token {bad_tok!r} must be refused")
    e = expect(lambda: J10.collect(None, J10.STAGE), J10.J10Error, "--collect without --out")
    check("requires --out" in str(e), "and say so")
    for bad_out in (str(HERE), str(HERE / "j10_runs"), str(HERE / "j1_runs")):
        e = expect(lambda b=bad_out: J10.collect(b, J10.STAGE), J10.J10Error,
                   f"--out {bad_out} is not the authorised leaf")
        check("not the authorised leaf" in str(e), f"refused: {Path(bad_out).name}")
    check(snapshot(HERE) == before_all and not (HERE / "j10_runs").exists(),
          "MEASURED: every refusal wrote nothing; j10_runs does not exist")

    # ---------------------------------------------------------------- the full collection ------
    saved_root = J10.OUTPUT_ROOT_OVERRIDE
    tmp = Path(tempfile.mkdtemp(prefix="v26c_j10_test_"))
    real_pros, real_sum = J1._prosthetic_state, J1._summarise
    state = {"n": 0}

    def fake_pros(actor_obs, names):
        knee, ankle = good_kin()
        i = state["n"] % 500
        state["n"] += 1
        return {"pros_knee_angle": float(knee[i]), "pros_ankle_angle": float(ankle[i])}

    def fake_sum(trace, end_reason, clipped, rms):
        return good_summary(end_reason=end_reason, action_clipped_steps=clipped,
                            realized_noise_rms=list(rms), steps=len(trace))

    try:
        J10.OUTPUT_ROOT_OVERRIDE = tmp
        target = J10.authorized_leaf()
        lock = target.parent / J10.LOCK_NAME
        e = expect(lambda: J10.collect(str(target), J10.STAGE), J10.J10Error,
                   "the authorised collection refuses a set override without an injected stack")
        check("permitted only for synthetic" in str(e), "the override is test-only")

        J1._prosthetic_state, J1._summarise = fake_pros, fake_sum
        FakeEnv.wait_hs_steps = 12            # cell B will carry the missing state
        FakeEnv.built.clear(); FakeEnv.closed.clear(); FakeEnv.saved.clear()
        try:
            res = J10.collect(str(target), J10.STAGE, stack=make_stack(), progress=False)
        finally:
            J1._prosthetic_state, J1._summarise = real_pros, real_sum

        check(res["verdict"] == "PASS" and res["aggregate_pass"] is True,
              "the synthetic collection aggregates to PASS")
        check(len(res["cells"]) == 2 and [c["id"] for c in res["cells"]] == ["B", "C"],
              "both cells ran, in the frozen order")
        check(len(FakeEnv.built) == 2 and len(FakeEnv.closed) == 2 and len(FakeEnv.saved) == 2,
              "TWO environments built, TWO closed, TWO wrote their sim_outputs")
        check([b["offset"] for b in FakeEnv.built] == [J10.OFFSET_MINUS, J10.OFFSET_PLUS],
              "each env received its declared offset in seconds")
        check(res["coverage"]["pass"] is True and res["coverage"]["total_wait_hs_rows"] == 24,
              f"the coverage gate is satisfied ({res['coverage']['total_wait_hs_rows']} rows)")

        rc = json.loads((target / J10.RECEIPT_NAME).read_text())
        check(rc["stage"] == J10.STAGE and rc["cells_total"] == 2
              and rc["cells_behavioural_pass"] == 2 and rc["cells_telemetry_valid"] == 2,
              "the receipt records 2/2 behavioural and 2/2 telemetry-valid")
        check(rc["parent_unchanged"] is True
              and rc["parent_before"]["module_state_sha256"]
              == rc["parent_after"]["module_state_sha256"] == J10.PIN_PARENT_STATE_SHA,
              "the parent hash is checked BEFORE and AFTER and is unchanged")
        # THE TWO PROVENANCE CORRECTIONS, asserted on the committed receipt
        blob = json.dumps(rc)
        check(".staging" not in blob and J10.STAGING_NAME not in blob,
              "PROVENANCE: no staging path appears anywhere in the receipt")
        check("/Users/" not in blob.replace(rc["preregistration"]["file"], ""),
              "and no absolute user path leaks into the artefact paths")
        for c in rc["cells"]:
            a = c["artefacts"]
            check(a["teacher_dataset"] == f"j10_cell_{c['id']}_teacher_dataset.npz"
                  and a["sim_outputs"] == f"j10_cell_{c['id']}_sim_outputs",
                  f"cell {c['id']}: every artefact path is LEAF-RELATIVE")
            check((target / a["teacher_dataset"]).is_file()
                  and (target / a["sim_outputs"]).is_dir(),
                  f"cell {c['id']}: and every recorded path RESOLVES in the committed leaf")
            check(a["sim_outputs_file_count"] == 19,
                  f"cell {c['id']}: 19 sim_outputs, as the J1 leaf has")
            check(c["paths_are_leaf_relative"] is True, f"cell {c['id']}: declared as such")
        check(rc["provenance"]["no_staging_path_recorded"] is True
              and rc["provenance"]["valid_cycle_count_listed_as_diagnostic"] is False,
              "the receipt declares both provenance corrections explicitly")
        check("valid_cycle_count" not in rc["gate_specification"]["diagnostics_not_binding"]
              and "action_clipped_steps"
              not in rc["gate_specification"]["diagnostics_not_binding"],
              "CORRECTED: no binding measure is listed among the non-binding diagnostics")
        check(rc["cells"][0]["gate"]["checks"]["valid_cycles"] is True,
              "while valid_cycles remains a binding check")
        # the dataset itself
        for c in rc["cells"]:
            with np.load(target / c["artefacts"]["teacher_dataset"], allow_pickle=False) as z:
                keys = tuple(sorted(z.files))
                check(keys == ("action_noises", "actions", "actor_feature_names",
                               "executed_actions", "observations", "times"),
                      f"cell {c['id']}: the dataset holds exactly the six J1 keys ({keys})")
                check(z["observations"].shape == (500, 35)
                      and z["actions"].shape == (500, 2)
                      and z["executed_actions"].shape == (500, 2)
                      and z["times"].shape == (500,),
                      f"cell {c['id']}: raw obs35, teacher actions, executed, times")
                check(float(np.max(np.abs(z["action_noises"]))) == 0.0,
                      f"cell {c['id']}: the noise is exactly zero at sigma 0")
                check(np.array_equal(z["actions"], z["executed_actions"]),
                      f"cell {c['id']}: executed == prescribed teacher, bit for bit")
                check(len(z["actor_feature_names"]) == 35,
                      f"cell {c['id']}: 35 feature names, the RUNTIME width")
            check(c["label_semantics"]["policy_queried"] is False
                  and c["label_semantics"]["j8_student_used"] is False
                  and c["label_semantics"]["lookahead_s"] == 0.0,
                  f"cell {c['id']}: no policy was queried and the student was not used")
            rk = c["reset_check"]
            check(rk["reset_time_error_s"] <= 1e-9 and rk["cfg_t_start_s"] == 11.99,
                  f"cell {c['id']}: reset matches cfg.t_start + offset within 1e-9")
        check(rc["inert"]["policy_queried"] is False and rc["inert"]["student_used"] is False
              and rc["inert"]["ppo_updates"] == 0 and rc["inert"]["ray_cluster_started"] is False,
              "the receipt attests no policy, no student, no PPO, no Ray cluster")
        check(rc["outcome"]["fit_authorized"] is False
              and rc["outcome"]["next_stage_authorized"] is False,
              "and it authorises no fit and no next stage")
        json.dumps(rc, allow_nan=False)
        check(True, "the receipt is strictly JSON")

        after_first = snapshot(target)
        e = expect(lambda: J10.validate_out(str(target)), J10.J10Error, "a second collection")
        check("no-clobber and single-execution" in str(e), "is refused: the leaf exists")
        check(snapshot(target) == after_first, "and the existing leaf is untouched")

        # a coverage FAIL still COMMITS the evidence
        shutil.rmtree(target)
        J1._prosthetic_state, J1._summarise = fake_pros, fake_sum
        FakeEnv.wait_hs_steps = 0             # neither cell carries the missing state
        FakeEnv.built.clear(); FakeEnv.closed.clear(); FakeEnv.saved.clear()
        try:
            res2 = J10.collect(str(target), J10.STAGE, stack=make_stack(), progress=False)
        finally:
            J1._prosthetic_state, J1._summarise = real_pros, real_sum
            FakeEnv.wait_hs_steps = 12
        check(res2["verdict"] == "FAIL" and res2["aggregate_pass"] is False
              and res2["coverage"]["pass"] is False,
              "a collection without the missing state aggregates to FAIL on COVERAGE alone")
        check(target.is_dir() and (target / J10.RECEIPT_NAME).is_file(),
              "MEASURED: the FAIL evidence is STILL COMMITTED")
        rc2 = json.loads((target / J10.RECEIPT_NAME).read_text())
        check(rc2["cells_behavioural_pass"] == 2 and rc2["coverage"]["pass"] is False,
              "both cells passed behaviourally; only the coverage gate failed")
        check(len(FakeEnv.built) == 2, "and both cells still ran: no behavioural fail-fast")
        check(not lock.exists() and not (target.parent / J10.STAGING_NAME).exists(),
              "lock released and staging removed on the FAIL path too")

        # a technical failure commits NOTHING and preserves concurrent content
        shutil.rmtree(target)
        sentinel = tmp / "j10_runs" / "concurrent.txt"
        sentinel.write_text("other", encoding="utf-8")
        broken = make_stack()
        broken.make_env = lambda cfg: (_ for _ in ()).throw(RuntimeError("synthetic env failure"))
        expect(lambda: J10.collect(str(target), J10.STAGE, stack=broken), RuntimeError,
               "a technical exception must fail closed")
        check(not target.exists() and not (target.parent / J10.STAGING_NAME).exists()
              and not lock.exists(), "no leaf, no staging, no lock")
        check(sentinel.is_file(), "and the concurrent sentinel SURVIVED")
    finally:
        J10.OUTPUT_ROOT_OVERRIDE = saved_root
        J1._prosthetic_state, J1._summarise = real_pros, real_sum
        FakeEnv.wait_hs_steps = 0
        shutil.rmtree(tmp, ignore_errors=True)

    # ---------------------------------------------------------------- static guarantees --------
    imported = {n.names[0].name for n in ast.walk(tree) if isinstance(n, ast.Import)}
    imported |= {(n.module or "") for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)}
    check(imported == {"__future__", "argparse", "hashlib", "json", "os", "shutil", "sys",
                       "pathlib", "typing", "numpy", "v26c_j1_collect", "v26c_j3_closed_loop",
                       "v26c_penetration_contract", "env_factory", "exploration_noise",
                       "target_domain_imitation"},
          f"the import surface is the stdlib, numpy, J1/J3, the contract and the three "
          f"production modules ({sorted(imported)})")
    top = {n.names[0].name.split(".")[0] for n in tree.body if isinstance(n, ast.Import)}
    top |= {(n.module or "").split(".")[0] for n in tree.body if isinstance(n, ast.ImportFrom)}
    check(not (top & {"torch", "ray", "opensim", "env_factory", "target_domain_imitation"}),
          "no heavy import at module scope")
    ps = next(n for n in ast.walk(tree)
              if isinstance(n, ast.FunctionDef) and n.name == "production_stack")
    check({n.names[0].name for n in ast.walk(ps) if isinstance(n, ast.Import)}
          == {"env_factory", "exploration_noise", "target_domain_imitation"},
          "the three production modules are imported lazily, inside production_stack")
    check("RLModule" not in ids and "from_checkpoint" not in ids,
          "no RLModule is ever constructed: the label source is prescribed, not a policy")
    for name in ("PPOConfig", "train_ppo", "adapt_actor", "Adam", "backward", "optimizer"):
        check(name not in ids, f"the runner never references {name}")
    # AST, not substring: the forbidden names appear in FORBIDDEN_CALL_SITES as declared
    # STRING LITERALS. What must not exist is an actual CALL to any of them.
    called = set()
    for n in ast.walk(tree):
        if isinstance(n, ast.Call):
            f = n.func
            if isinstance(f, ast.Name):
                called.add(f.id)
            elif isinstance(f, ast.Attribute):
                called.add(f.attr)
    check("build_target_env_config" not in called,
          "AST: the historical builder is never CALLED")
    # the runner's own bare main() is legitimate; what is forbidden is ANOTHER module's .main()
    attr_calls = {n.func.attr for n in ast.walk(tree) if isinstance(n, ast.Call)
                  and isinstance(n.func, ast.Attribute)}
    check("main" not in attr_calls,
          "AST: no OTHER module's .main() is ever called - the teacher is reused function-wise")
    check("prescribed_teacher_action" in src,
          "and the sanctioned reuse, prescribed_teacher_action, is the label source")
    check(J10.ALLOWED_TEACHER_REUSE.split("(")[0].split(".")[-1] in src,
          "and the sanctioned reuse is prescribed_teacher_action")
    check(src.count("shutil.rmtree") == 1, "exactly ONE recursive removal")
    cf = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "collect")
    rm = [n for n in ast.walk(cf) if isinstance(n, ast.Call)
          and isinstance(n.func, ast.Attribute) and n.func.attr == "rmtree"]
    check(len(rm) == 1 and isinstance(rm[0].args[0], ast.Name)
          and rm[0].args[0].id == "staging_created",
          "and its only argument is staging_created")
    loop = next(n for n in ast.walk(cf) if isinstance(n, ast.For))
    check(not [n for n in ast.walk(loop) if isinstance(n, (ast.Break, ast.Continue))],
          "STATIC: the collection loop has no break and no continue - no behavioural fail-fast")
    check("O_CREAT | os.O_EXCL" in src, "the lock uses O_CREAT|O_EXCL")

    # ---------------------------------------------------------------- nothing disturbed --------
    check(not (HERE / "j10_runs").exists(),
          "MEASURED: this suite never created the authorised j10_runs")
    check(snapshot(HERE) == before_all,
          "MEASURED: not one byte under the validation root changed across the WHOLE suite")

    print(json.dumps({"selftest": "PASS", "checks": CHECKS}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
