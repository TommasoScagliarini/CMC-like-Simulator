"""Fail-closed tests for the V26C J9R1 closed-loop qualification.

Pure and synthetic: no physical environment is ever constructed. A fake stack drives the whole
six-cell matrix so the write path, the gate, the parity contracts and the aggregate can be
exercised end to end in a temporary root. The authorised J9 leaf is never created.
"""
from __future__ import annotations
import ast, builtins, io, json, os, shutil, sys, tempfile, tokenize
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26c_j9r1_closed_loop as J9  # noqa: E402  the CORRECTED runner under test
import v26c_j9_closed_loop as J9ORIG  # noqa: E402  the original, for equivalence
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
        out[str(p.relative_to(root))] = J9._sha_file(p)
    return out


# ---------------------------------------------------------------- synthetic summaries -----------

def good_summary(**over):
    s = {"steps": 500, "end_reason": "episode_time_limit", "valid_cycle_count": 2,
         "valid_hs_count": 3, "valid_to_count": 3, "phase_timeout_stance": 0,
         "phase_timeout_swing": 0, "morphology_causal_contract_failure": 0,
         "hs_cancelled_count": 0, "resync_count": 0, "action_clipped_steps": 0,
         "episode_return": 40.0, "realized_noise_rms": [0.0, 0.0], "policy_std": [0.005, 0.005]}
    s.update(over)
    return s


def good_kin(n=500):
    """Kinematics that pass every J3 criterion: knee strictly flexed, ankle deep enough."""
    knee = np.linspace(-1.0, -0.2, n)          # ROM 0.8 >= 0.60, all negative, inside (-1.5, 0)
    ankle = np.linspace(-0.35, 0.05, n)        # min -0.35 <= -0.03, ROM 0.40 >= 0.30
    return knee, ankle


def pen_ok(maxv=0.0269, n=500):
    a = np.full(n, 0.001)
    a[10] = maxv
    return a


# ---------------------------------------------------------------- the fake stack ----------------

_UNSET = object()


class FakeSpace:
    def __init__(self, shape=(2,)):
        self.shape = shape
        self.low = np.full(shape, -1.0, dtype=np.float32)
        self.high = np.full(shape, 1.0, dtype=np.float32)


class FakeSimCfg:
    """cfg.t_start / cfg.t_end as the production env exposes them."""

    def __init__(self, t_start=11.99, t_end=30.0):
        self.t_start = float(t_start)
        self.t_end = float(t_end)


class FakeEnvCfg:
    """env_cfg.episode_duration as the production env exposes it - the NORMATIVE source."""

    def __init__(self, episode_duration=5.0):
        self.episode_duration = episode_duration


class FakeBase:
    def __init__(self, t0, t_start=11.99, t_end=30.0, duration=5.0):
        self.t = float(t0)
        self.cfg = FakeSimCfg(t_start, t_end)
        self.env_cfg = FakeEnvCfg(duration)


class FakeEnv:
    """A synthetic environment. Records what it was built with; never touches OpenSim."""

    built: list[dict] = []
    closed: list[str] = []
    saved: list[str] = []

    t_start = 11.99
    t_end = 30.0
    close_raises = False
    live_duration: object = _UNSET        # set to a value to diverge from the declared one
    drop_cfg = False
    drop_env_cfg = False

    def __init__(self, cfg):
        self.cfg = dict(cfg)
        self.offset = float(cfg["episode_start_offset_s"])
        self.start = J9.expected_reset_time(
            FakeEnv.t_start, FakeEnv.t_end, cfg.get("episode_duration"),
            self.offset)["expected_reset_time_s"]
        self.unwrapped = FakeBase(self.start, FakeEnv.t_start, FakeEnv.t_end,
                                  FakeEnv.live_duration
                                  if FakeEnv.live_duration is not _UNSET
                                  else cfg.get("episode_duration"))
        if FakeEnv.drop_cfg:
            del self.unwrapped.cfg
        if FakeEnv.drop_env_cfg:
            del self.unwrapped.env_cfg
        self.action_space = FakeSpace()
        self.n = 0
        self.names = None
        # REALISTIC INSTRUMENTATION: the production env writes sim_outputs ONLY in close(), and
        # only when record_outputs AND save_outputs_on_close are both True
        # (osim_trj_cmc_like.py:2048-2053). Nothing is written in the constructor - writing there
        # unconditionally is exactly what let the J9 defect through the old test suite.
        self.record_outputs = bool(cfg.get("record_outputs", False))
        self.save_outputs_on_close = bool(cfg.get("save_outputs_on_close", False))
        self.output_prefix = cfg.get("output_prefix")
        self.out = Path(cfg["output_dir"])
        FakeEnv.built.append({"offset": self.offset, "output_dir": str(self.out),
                              "record_outputs": self.record_outputs,
                              "save_outputs_on_close": self.save_outputs_on_close,
                              "output_prefix": self.output_prefix})

    def reset(self, seed=None):
        self.n = 0
        self.unwrapped.t = self.start
        return np.zeros(J9.ACTOR_WIDTH, dtype=np.float32), {}

    def step(self, action):
        self.n += 1
        self.unwrapped.t = self.start + 0.01 * self.n
        knee, ankle = good_kin()
        obs = np.zeros(J9.ACTOR_WIDTH, dtype=np.float32)
        info = {"time": self.unwrapped.t,
                "end_reason": "episode_time_limit" if self.n >= 500 else "",
                "reward_terms": {"grf_penetration_m": float(pen_ok()[self.n - 1])},
                J1.FSM_KEY: {"behaviour_version": J1.EXPECTED_FSM_BEHAVIOUR_VERSION},
                "knee": float(knee[self.n - 1]), "ankle": float(ankle[self.n - 1])}
        return obs, 0.08, False, self.n >= 500, info

    def close(self):
        if FakeEnv.close_raises:
            raise RuntimeError("synthetic close failure")
        # THE PRODUCTION GATE, modelled faithfully (osim_trj_cmc_like.py:2048-2053):
        #     if record_outputs and save_outputs_on_close and runner is not None:
        #         self.runner.save_results()
        # Outputs appear ONLY here and ONLY when both flags are True. A constructor that wrote
        # unconditionally is what let the J9 defect pass the old suite.
        if self.record_outputs and self.save_outputs_on_close:
            self.out.mkdir(parents=True, exist_ok=True)
            prefix = self.output_prefix or "rl_episode"
            for name in ("states", "kinematics", "activations"):
                (self.out / f"{prefix}_{name}.sto").write_text("synthetic", encoding="utf-8")
            FakeEnv.saved.append(str(self.out))
        FakeEnv.closed.append(str(self.cfg["output_dir"]))


class FakeStack:
    name = "synthetic"
    operational = False

    def __init__(self):
        self.torch = None
        self.reference_action = None
        self.held_stochastic_action = None
        self.held_normal = None
        self.seeded: list[int] = []

    def seed(self, s):
        self.seeded.append(int(s))

    def load_module(self, path):
        return {"module": str(path)}

    def make_env(self, cfg):
        return FakeEnv(cfg)


def main() -> int:
    src = (HERE / "v26c_j9r1_closed_loop.py").read_text()
    tree = ast.parse(src)
    ids = {t.string for t in tokenize.generate_tokens(io.StringIO(src).readline)
           if t.type == tokenize.NAME}
    amend = json.loads(J9.PREREG.read_text())          # the J9R1 amendment
    prereg = json.loads((HERE / "v26c_j9_prereg_closed_loop.json").read_text())
    sci = amend["scientific_contract_unchanged"]
    before_all = snapshot(HERE)

    # ---------------------------------------------------------------- preregistration ----------
    check(amend["kind"] == "ADDITIVE, IMMUTABLE, INSTRUMENTATION AMENDMENT"
          and amend["stage_proposed"] == "V26C_J9R1_CLOSED_LOOP" == J9.STAGE,
          "the amendment is additive, immutable and PROPOSES the J9R1 stage")
    check("stage_authorised" not in amend,
          "it does NOT claim stage_authorised: proposing is not authorising")
    check(amend["authorisation_status"].startswith("NOT_GRANTED")
          and "fresh explicit user authorization required" in amend["authorisation_status"],
          "and its authorisation_status is NOT_GRANTED, requiring a fresh user authorisation")
    check("authorised_by" not in amend,
          "no authorisation is attributed to the user by this record")
    check(amend["authorisation_policy"]["fresh_user_authorization_required"] is True
          and amend["authorisation_policy"]["status"] == "NOT_GRANTED",
          "the policy says the same, unambiguously")
    check(J9.STAGE != J9ORIG.STAGE and J9.RELATIVE_LEAF != J9ORIG.RELATIVE_LEAF,
          "J9R1 is a SEPARATE stage with a separate leaf - not a retry into the J9 one")
    check(J9._sha_file(J9.PREREG) == J9.PIN_PREREG
          == "cf84b4747a9fd1897bb77eeead692ea774213f5f4e110c7ebc8aaf88f66ce6fa",
          "and the AMENDMENT is pinned by exact hash")
    check(len(amend["pinned_artefacts_sha256"]) == 18
          and all(J9._sha_file(HERE / r) == h
                  for r, h in amend["pinned_artefacts_sha256"].items()),
          "all 18 local pins verify on disk")
    check(len(amend["pinned_repo_artefacts_sha256"]) == 5
          and all(J9._sha_file(J9.REPO / r) == h
                  for r, h in amend["pinned_repo_artefacts_sha256"].items()),
          "all 5 repo pins verify: the runtime config plus the four live runtime sources")
    for rel, why in (("Trajectory Generator/osim_trj_cmc_like.py", "the environment itself"),
                     ("Trajectory Generator/baseline_MLP/rollout_eval.py", "the rollout helpers"),
                     ("Trajectory Generator/baseline_MLP/env_factory.py", "the env factory"),
                     ("Trajectory Generator/baseline_MLP/exploration_noise.py",
                      "the held noise generator")):
        check(rel in amend["pinned_repo_artefacts_sha256"]
              and amend["pinned_repo_artefacts_sha256"][rel] == J9._sha_file(J9.REPO / rel),
              f"{why} is pinned and verifies on disk")
    check(amend["pinned_repo_artefacts_sha256"]["Trajectory Generator/osim_trj_cmc_like.py"]
          == "26458424c44f18fa1dda20b830fa5e7e825c583cc5d10e6d019cb3bd9a0c6d24",
          "the environment is pinned at its current hash")
    prs = prereg["pinned_runtime_sources"]
    check("_initialise_episode" in prs["Trajectory Generator/osim_trj_cmc_like.py"]
          and "reset semantics this runner transcribes" in
          prs["Trajectory Generator/osim_trj_cmc_like.py"],
          "and the prereg explains that this pin ALSO authenticates the transcribed reset "
          "semantics")
    check("_held_stochastic_action" in prs["Trajectory Generator/baseline_MLP/rollout_eval.py"],
          "rollout_eval is pinned for the production stochastic path")
    en = prs["Trajectory Generator/baseline_MLP/exploration_noise.py"]
    check(amend["pinned_repo_artefacts_sha256"]
          ["Trajectory Generator/baseline_MLP/exploration_noise.py"]
          == "ea716c198cfce5e649a6d7bf5b046365c2dd9c3d3eb8f2c0a84f913e119eeb9f",
          "exploration_noise is pinned at the architect's hash")
    check("HeldStandardNormal" in en and "AUTHENTICATES THE HELD NOISE SEQUENCE" in en
          and "123/124/125" in en,
          "and the prereg explains that this pin authenticates the held noise SEQUENCE of the "
          "three stochastic cells")
    check("silently explore a different sequence" in en,
          "naming the exact risk it closes: same seeds, different draws")
    check("HeldStandardNormal" in src and "stack.held_normal" in src,
          "the runner imports and instantiates HeldStandardNormal directly")
    check(prs["not_modified"].startswith("this stage reads all four"),
          "and none of the four is modified")
    # the transcribed arithmetic must still be the environment's own
    envsrc = (J9.REPO / "Trajectory Generator" / "osim_trj_cmc_like.py").read_text()
    for frag in ("max_start = max(cfg.t_start, max_start)",
                 "requested_start = float(cfg.t_start) + max(",
                 "self._episode_start = min(requested_start, float(max_start))",
                 "self.t = self._episode_start"):
        check(frag in envsrc,
              f"MEASURED in the pinned environment source: {frag}")
    check(prereg["runtime"]["pinned_config_sha256"] == J9.PIN_RUNTIME_CONFIG_SHA
          == "a870cc38a77d853bbd5fba86b51cfcc3ef20a33a5823f4a42f1b968ba4a537db",
          "the runtime config is the architect's pinned one")

    # ---------------------------------------------------------------- the exact actor leaf ------
    check(sorted(J9.PIN_J8) == ["history.json", "rl_module/actor_feature_manifest.json",
                                "rl_module/class_and_ctor_args.pkl", "rl_module/metadata.json",
                                "rl_module/module_state.pkl",
                                "v26c_j8_recovery_fit_receipt.json"],
          "the J8 leaf is the exact six-file set")
    check(J9.PIN_J8["rl_module/module_state.pkl"]
          == "9c5b157156e6b9c2a69a16f14908d6750ac6acdad95516eba9ac9378912dbc82"
          and J9.PIN_J8["v26c_j8_recovery_fit_receipt.json"]
          == "13adc58b81ea372edef43a3d57f62b5f971e6be87c5242a8a0e234e993de6995",
          "at the architect's hashes")
    actor = J9.verify_actor()
    check(actor["file_count"] == 6 and actor["artefacts_sha256"] == dict(J9.PIN_J8),
          "MEASURED: the leaf on disk is exactly those six files at those hashes")
    mc = actor["mask_contract"]
    check(mc["clock_columns"] == [0, 1] and mc["clock_exactly_zero"] is True,
          "clock columns 0:2 are exactly zero")
    check(mc["controller_columns"] == list(range(25, 35)) and mc["controller_live"] is True,
          "controller columns 25:35 are LIVE")
    for key, block in mc["layers"].items():
        check(block["zero_columns"] == [0, 1],
              f"{key}: the ONLY zero columns are the clock")
        check(all(v > 0.0 for v in block["controller_norms"].values()),
              f"{key}: every controller column has a positive norm")
    check(mc["width"] == 35 and "25D" not in json.dumps(mc),
          "one 35D actor; no standalone 25D, no widening")
    sg = actor["sigma"]
    check(abs(sg["sigma_from_actor"][0] - 0.005) <= 1e-6 and sg["logstd_head_edited"] is False
          and sg["module_copied"] is False,
          "sigma 0.005 comes from the actor's own FROZEN log-std head, never edited or copied")
    lin = actor["lineage"]
    check(lin["parent_module_state_sha256"]
          == "0f182ea9f8939e2b7824e85c12c57343309c444680682b9bce5858dd74f9d130"
          and lin["dataset_sha256"]
          == "bb9b21f029063562bc0229fcc6601dd98e19d071f115811f7d8cb918be852e27"
          and lin["j8_verdict"] == "PASS",
          "the August V26 lineage is PROVEN from the J8 receipt: J2 parent + current J7 dataset")

    # ---------------------------------------------------------------- the matrix ---------------
    check(len(J9.MATRIX) == 6 and [c["id"] for c in J9.MATRIX] == list("ABCDEF"),
          "six cells, in the frozen order A B C D E F")
    check([c["mode"] for c in J9.MATRIX]
          == ["deterministic"] * 3 + ["stochastic_held"] * 3,
          "three deterministic then three stochastic")
    check([c["seed"] for c in J9.MATRIX] == [123, 123, 123, 123, 124, 125],
          "seeds 123/123/123 then 123/124/125")
    check([c["offset_s"] for c in J9.MATRIX]
          == [1.956870983805102, 1.756870983805102, 2.156870983805102] + [1.956870983805102] * 3,
          "offsets nominal / -0.20 / +0.20 then nominal three times")
    check(J9.OFFSET_UNIT == "seconds"
          and abs((J9.OFFSET_MINUS - J9.OFFSET_NOMINAL) + 0.20) < 1e-12
          and abs((J9.OFFSET_PLUS - J9.OFFSET_NOMINAL) - 0.20) < 1e-12,
          "the unit is SECONDS and the deltas are exactly -0.20 and +0.20 s")
    check(J9.NOISE_HOLD_STEPS == 1 and J9.SIGMA == 0.005 and J9.EXPECTED_STEPS == 500,
          "hold 1, sigma 0.005, 500 steps")
    uc = J9.unit_correction()
    check(uc["unit"] == "seconds" and uc["earlier_records_edited"] is False,
          "the unit correction is additive and edits no earlier record")
    check("NO numeric artefact was affected" in uc["no_numeric_artefact_affected"],
          "and states that no numeric artefact was affected")
    check("v26c_j7_prereg_markov_dataset.json" in uc["earlier_prose_error"]
          and "v26c_j4_recovery.py" in uc["earlier_prose_error"],
          "naming the frozen records that say 'rad'")

    # ---------------------------------------------------------------- offset-only mutation -----
    SENTINEL = HERE / "_j9r1_test_sentinel_never_created"
    base, cfg = J9.base_env_config(SENTINEL)
    check(float(base["episode_start_offset_s"]) == J9.OFFSET_NOMINAL,
          "the pinned base config carries the nominal offset")
    for off in (J9.OFFSET_NOMINAL, J9.OFFSET_MINUS, J9.OFFSET_PLUS):
        env, mut = J9.cell_env_config(base, cfg, off, Path("/tmp/j9_probe"))
        check(float(env["episode_start_offset_s"]) == off, f"the cell config carries {off}")
        check(set(mut["mutated_keys"]) <= {"episode_start_offset_s", "output_dir"},
              f"ONLY the offset and the output_dir differ ({mut['mutated_keys']})")
        # STABLE KEYS: exclude BOTH declared-mutable keys. Excluding only the offset - the J9
        # form - would flag the per-cell output_dir and fail on every cell now that the
        # recording-enabled base already carries one.
        stable = set(base) - {"episode_start_offset_s", "output_dir"}
        others = sorted(k for k in stable if base[k] != env.get(k))
        check(others == [], f"MEASURED: every STABLE field is identical ({others})")
        naive = [k for k in base if k != "episode_start_offset_s" and base[k] != env.get(k)]
        check(naive == ["output_dir"] or naive == [],
              f"and the naive J9-style exclusion would have flagged {naive} - which is why the "
              f"guard must exclude both declared-mutable keys")
        check(mut["base_verified_by_j1_before_mutation"] is True
              and mut["second_j1_verify_on_mutated_config"] is False
              and mut["stable_keys_count"] == len(base) - 2,
              "the base is J1-verified BEFORE the mutation, the verifier is not re-run, and every "
              "other field is proved equal to it")
    e = expect(lambda: J9.cell_env_config(base, cfg, 1.9, Path("/tmp/x")), J9.J9Error,
               "an offset outside the three frozen values must be refused")
    check("not one of the three frozen offsets" in str(e), "naming the frozen set")

    # the mutated key-set is a SUBSET of the allowed pair, and nominal vs multistart differ
    expected_per_cell = prereg["env_mutation_policy"]["expected_per_cell"]
    for c in J9.MATRIX:
        _, m = J9.cell_env_config(base, cfg, c["offset_s"], Path("/tmp/j9_probe"))
        check(set(m["mutated_keys"]) <= {"episode_start_offset_s", "output_dir"}
              and m["mutated_is_subset_of_allowed"] is True,
              f"cell {c['id']}: the mutated key-set is a SUBSET of the allowed pair")
        check(sorted(m["mutated_keys"]) == sorted(expected_per_cell[c["id"]]),
              f"cell {c['id']}: mutates exactly {expected_per_cell[c['id']]}")
    check(sorted(expected_per_cell["A"]) == ["output_dir"]
          and sorted(expected_per_cell["B"]) == ["episode_start_offset_s", "output_dir"],
          "A/D/E/F normally differ only in output_dir; B and C differ in both")
    check(all(J9.cell_env_config(base, cfg, c["offset_s"], Path("/tmp/p"))[1]
              ["second_j1_verify_on_mutated_config"] is False for c in J9.MATRIX),
          "and J1.verify_env_config is NEVER re-run on the mutated config")
    check("RUNTIME LITERAL CROSS-CHECK"
          in prereg["env_mutation_policy"]["no_second_j1_verify_on_mutated_config"],
          "the prereg records WHY: J1 pins the nominal offset in its runtime literal cross-check")
    vf = [n for n in ast.walk(next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
                                   and n.name == "cell_env_config"))
          if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
          and n.func.attr == "verify_env_config"]
    check(not vf, "STATIC: cell_env_config calls verify_env_config zero times")

    # ---------------------------- THE J9R1 CORRECTION: recording instrumentation ----------------
    check(base["record_outputs"] is True and base["save_outputs_on_close"] is True,
          "the base carries record_outputs True and save_outputs_on_close True")
    check(bool(base.get("output_prefix")) and bool(base.get("output_dir")),
          "and both output_prefix and output_dir are present")
    check(base["output_dir"].endswith("sim_outputs"),
          "the J1 builder appends its own sim_outputs component to the root")
    # a None root is EXACTLY the J9 defect and must be refused outright
    e = expect(lambda: J9.base_env_config(None), J9.J9Error,
               "a None output root must be refused")
    check("exactly the J9 defect" in str(e) and "record_outputs" in str(e),
          "naming the defect it prevents")
    # PROVEN NECESSARY: with the J9 configuration the flags really are off
    off, _ = J1.build_full_env_config(J1.load_pinned_config(), output_dir=None), None
    check(off["record_outputs"] is False and off["save_outputs_on_close"] is False
          and "output_dir" not in off and "output_prefix" not in off,
          "MEASURED: the J1 builder with output_dir=None disables recording and omits both path "
          "keys - the exact J9 failure mode")
    # every cell inherits the flags, and output_prefix never varies per cell
    for c in J9.MATRIX:
        env_c, _ = J9.cell_env_config(base, cfg, c["offset_s"], SENTINEL / f"cell_{c['id']}")
        check(env_c["record_outputs"] is True and env_c["save_outputs_on_close"] is True,
              f"cell {c['id']} inherits both recording flags as True")
        check(env_c["output_prefix"] == base["output_prefix"],
              f"cell {c['id']} inherits output_prefix unchanged - it is never mutated per cell")
        check(env_c["output_dir"] == str(SENTINEL / f"cell_{c['id']}"),
              f"cell {c['id']} consumes the exact final path, with no extra nesting")
    check(not SENTINEL.exists(),
          "MEASURED: building configs created no directory - the builder only formats strings")

    # the stable-key guard: the J9 form would have failed here, because output_dir is now in base
    _, m = J9.cell_env_config(base, cfg, J9.OFFSET_MINUS, SENTINEL / "x")
    check(m["stable_keys_count"] == len(set(base) - {"episode_start_offset_s", "output_dir"})
          and m["stable_keys_all_identical"] is True,
          "the guard counts STABLE keys, excluding both declared-mutable ones")
    check(m["stable_keys_count"] == len(base) - 2,
          f"which is len(base) - 2 = {len(base) - 2}, NOT len(base) - 1. The J9 form required "
          f"len(base) - 1 identical fields and would have failed every cell here.")
    check("recording_inherited_from_base" in m and m["recording_inherited_from_base"] is True,
          "and the report records that recording is inherited, not re-declared")
    check("fields_identical_to_verified_base" not in m,
          "the stale J9 counter is gone: no NameError, no misleading count")

    # ------------------------- instrumentation-only, proved field by field ----------------------
    eq = J9.verify_scientific_equivalence()
    check(eq["instrumentation_only"] is True and eq["all_equal"] is True
          and eq["fields_compared"] >= 24,
          f"{eq['fields_compared']} scientific constants compared against the ORIGINAL J9 runner, "
          f"all equal")
    for name in ("MATRIX", "EXPECTED_STEPS", "SIGMA", "SIGMA_TOLERANCE", "NOISE_HOLD_STEPS",
                 "J9_COMMON_GATE", "J9_KINEMATIC_GATE", "DIAGNOSTIC_NOT_BINDING",
                 "RESET_TIME_TOLERANCE_S", "RESET_SEMANTICS", "penetration_bands", "PIN_J8",
                 "FROZEN_OFFSETS", "OFFSET_UNIT", "ACTOR_WIDTH", "CLOCK_COLUMNS",
                 "CONTROLLER_COLUMNS"):
        check(eq["compared"][name]["equal"] is True, f"{name} is identical to the original J9")
    check(J9.MATRIX == J9ORIG.MATRIX and J9.J9_KINEMATIC_GATE == J9ORIG.J9_KINEMATIC_GATE
          and J9.J9_COMMON_GATE == J9ORIG.J9_COMMON_GATE and J9.SIGMA == J9ORIG.SIGMA
          and J9.EXPECTED_STEPS == J9ORIG.EXPECTED_STEPS
          and J9.NOISE_HOLD_STEPS == J9ORIG.NOISE_HOLD_STEPS
          and J9.RESET_TIME_TOLERANCE_S == J9ORIG.RESET_TIME_TOLERANCE_S
          and J9.DIAGNOSTIC_NOT_BINDING == J9ORIG.DIAGNOSTIC_NOT_BINDING,
          "and the test confirms it independently by importing both runners")
    check(eq["original_runner_sha256"] == J9.J9_RUNNER_SHA
          == "8b6c36db2fd9bd63bde41d9903d4e1eb1d1e312b8cbf790e9bf1ea5b4875ac67",
          "the original runner is hashed BEFORE import and is unedited")
    saved_j9 = J9.J9_RUNNER_SHA
    try:
        J9.J9_RUNNER_SHA = "0" * 64
        e = expect(J9.verify_scientific_equivalence, J9.J9Error,
                   "a changed original runner must be refused before import")
        check("refused before import" in str(e), "fail-closed, before executing it")
    finally:
        J9.J9_RUNNER_SHA = saved_j9

    # ------------------------------- reset semantics: offset is RELATIVE to cfg.t_start ---------
    r = J9.expected_reset_time(11.99, 30.0, 5.0, J9.OFFSET_NOMINAL)
    check(r["expected_reset_time_s"] == 11.99 + J9.OFFSET_NOMINAL == 13.946870983805102,
          f"t_start 11.99 + nominal offset gives 13.946870983805102 "
          f"({r['expected_reset_time_s']})")
    check(r["cfg_t_start_s"] == 11.99 and r["clamped"] is False
          and r["max_start_s"] == 25.0 and r["requested_unclamped_time_s"] == 13.946870983805102,
          "with t_start, max_start and the unclamped request all recorded")
    check(J9.expected_reset_time(11.99, 30.0, 5.0, J9.OFFSET_MINUS)["expected_reset_time_s"]
          == 11.99 + J9.OFFSET_MINUS
          and J9.expected_reset_time(11.99, 30.0, 5.0, J9.OFFSET_PLUS)["expected_reset_time_s"]
          == 11.99 + J9.OFFSET_PLUS,
          "and the same relation holds for the -0.20 s and +0.20 s cells")
    check(r["expected_reset_time_s"] != J9.OFFSET_NOMINAL,
          "PROVEN: the expected reset time is NOT the offset itself")
    # the clamp
    cl = J9.expected_reset_time(11.99, 14.0, 5.0, J9.OFFSET_NOMINAL)
    check(cl["max_start_s"] == 11.99 and cl["expected_reset_time_s"] == 11.99
          and cl["clamped"] is True,
          "CLAMP: when t_end - duration falls below t_start, max_start is t_start and the start "
          "is clamped to it")
    cl2 = J9.expected_reset_time(11.99, 20.0, 5.0, 10.0)
    check(abs(cl2["requested_unclamped_time_s"] - 21.99) < 1e-12 and cl2["max_start_s"] == 15.0
          and cl2["expected_reset_time_s"] == 15.0 and cl2["clamped"] is True,
          "CLAMP: a request beyond max_start is clamped to max_start and flagged")
    neg = J9.expected_reset_time(11.99, 30.0, 5.0, -1.0)
    check(neg["expected_reset_time_s"] == 11.99,
          "a negative offset is floored at 0.0 by max(0.0, offset), as production does")
    check(J9.RESET_TIME_TOLERANCE_S == 1e-9,
          "the reset tolerance is 1e-9 s, not a segment-sized window")
    check("RELATIVE TO cfg.t_start" in J9.RESET_TIME_TOLERANCE_WHY
          and "_initialise_episode" in J9.RESET_SEMANTICS,
          "and the semantics cite _initialise_episode explicitly")
    # The division of responsibility, verified structurally: the BASE is validated once by J1
    # inside base_env_config; cell_env_config never re-validates and only proves the diff.
    bfn = next(n for n in ast.walk(tree)
               if isinstance(n, ast.FunctionDef) and n.name == "base_env_config")
    base_verifies = [n for n in ast.walk(bfn) if isinstance(n, ast.Call)
                     and isinstance(n.func, ast.Attribute) and n.func.attr == "verify_env_config"]
    check(len(base_verifies) == 1,
          "base_env_config calls J1.verify_env_config exactly ONCE, on the unmutated base")
    check(J9.base_env_config(SENTINEL)[0]["episode_start_offset_s"] == J9.OFFSET_NOMINAL,
          "and refuses a pinned config whose offset is not the nominal one")

    # ---------------------------------------------------------------- the gate ------------------
    contract = PC.load_contract()
    knee, ankle = good_kin()
    pen = J9.penetration_report(pen_ok(), contract, "t")
    g = J9.evaluate_cell_gate(good_summary(), knee, ankle, pen)
    check(g["pass"] is True and g["failed"] == [], "a good cell PASSES every binding criterion")
    check(set(g["checks"]) >= {"steps", "end_reason", "valid_cycles", "phase_timeout_stance",
                               "phase_timeout_swing", "morphology_causal_contract_failure",
                               "hs_cancelled_count", "resync_count", "penetration_hard_binding",
                               "kinematic_ankle_min", "kinematic_ankle_amplitude",
                               "kinematic_knee_amplitude", "kinematic_knee_strictly_flexed"},
          "the gate carries the common criteria, the penetration band and the full J3 kinematics")
    check("max_penetration_m" not in g["checks"],
          "the old 0.020 penetration bar is NOT a binding criterion")
    check(J9.J9_KINEMATIC_GATE == dict(J3.J3_KINEMATIC_GATE),
          "the kinematic gate is J3's, UNCHANGED")
    for key, over in (("steps", {"steps": 499}), ("end_reason", {"end_reason": "terminated"}),
                      ("valid_cycles", {"valid_cycle_count": 1}),
                      ("phase_timeout_stance", {"phase_timeout_stance": 1}),
                      ("phase_timeout_swing", {"phase_timeout_swing": 1}),
                      ("morphology_causal_contract_failure",
                       {"morphology_causal_contract_failure": 1}),
                      ("hs_cancelled_count", {"hs_cancelled_count": 1}),
                      ("resync_count", {"resync_count": 2})):
        bad = J9.evaluate_cell_gate(good_summary(**over), knee, ankle, pen)
        check(bad["checks"][key] is False and key in bad["failed"] and bad["pass"] is False,
              f"the gate catches a violated {key}")
    check(J9.evaluate_cell_gate(good_summary(resync_count=1), knee, ankle, pen)["pass"] is True,
          "one resync is allowed; two are not")

    # ankle-min DIRECTION, explicitly
    shallow = np.linspace(-0.0099, 0.3901, 500)         # min -0.0099, ROM 0.40
    b = J9.evaluate_cell_gate(good_summary(), knee, shallow, pen)
    check(b["checks"]["kinematic_ankle_min"] is False,
          "ankle min -0.0099 FAILS: it is not at or below -0.03")
    exact = np.linspace(-0.03, 0.37, 500)               # min exactly -0.03, ROM 0.40
    b = J9.evaluate_cell_gate(good_summary(), knee, exact, pen)
    check(b["checks"]["kinematic_ankle_min"] is True,
          "ankle min EXACTLY -0.03 PASSES: the comparison is <= -0.03")
    deep = np.linspace(-0.31, 0.09, 500)
    check(J9.evaluate_cell_gate(good_summary(), knee, deep, pen)["checks"]["kinematic_ankle_min"]
          is True, "and more negative PASSES: more negative is better, not worse")
    check("ankle_min <= -0.03" in g["ankle_min_direction"],
          "the direction is stated explicitly in the gate output")
    narrow = np.linspace(-0.10, 0.10, 500)              # ROM 0.20 < 0.30
    check(J9.evaluate_cell_gate(good_summary(), knee, narrow, pen)
          ["checks"]["kinematic_ankle_amplitude"] is False, "ankle ROM below 0.30 FAILS")
    flat_knee = np.linspace(-0.5, -0.2, 500)            # ROM 0.30 < 0.60
    check(J9.evaluate_cell_gate(good_summary(), flat_knee, ankle, pen)
          ["checks"]["kinematic_knee_amplitude"] is False, "knee ROM below 0.60 FAILS")
    ext_knee = np.linspace(-1.0, 0.1, 500)              # not strictly flexed
    check(J9.evaluate_cell_gate(good_summary(), ext_knee, ankle, pen)
          ["checks"]["kinematic_knee_strictly_flexed"] is False,
          "a knee that leaves the flexed half-space FAILS")

    # penetration boundary, from the CONTRACT only
    exact28 = J9.penetration_report(pen_ok(0.028), contract, "t")
    check(exact28["binding_pass"] is True
          and J9.evaluate_cell_gate(good_summary(), knee, ankle,
                                    exact28)["checks"]["penetration_hard_binding"] is True,
          "EXACTLY 0.028 PASSES the binding band")
    above = J9.penetration_report(pen_ok(0.0280001), contract, "t")
    check(above["binding_pass"] is False
          and J9.evaluate_cell_gate(good_summary(), knee, ankle,
                                    above)["checks"]["penetration_hard_binding"] is False,
          "anything above 0.028 FAILS")
    band25 = J9.penetration_report(pen_ok(0.0269), contract, "t")
    check(band25["flags"]["above_soft_diagnostic"] is True
          and band25["flags"]["july_legacy_breach"] is True
          and band25["binding_pass"] is True,
          "the 20 mm and 25 mm bands are DIAGNOSTIC: breaching them still binding-PASSES")
    check(band25["evaluated_by"] == "v26c_penetration_contract.evaluate_series"
          and band25["contract_sha256"] == J9.PIN_CONTRACT,
          "every penetration statement comes from the pinned contract evaluator")
    # The bands appear in the runner only as PROSE (semantics strings the receipt carries). What
    # must not exist is a numeric threshold actually used: no comparison against them, and no
    # constant bound to a name. Checked structurally, not by substring.
    bands = {0.020, 0.025, 0.028}
    compared = [n for n in ast.walk(tree) if isinstance(n, ast.Compare)
                and any(isinstance(c, ast.Constant) and isinstance(c.value, float)
                        and c.value in bands for c in n.comparators + [n.left])]
    check(not compared,
          f"AST: the runner compares against no penetration band ({len(compared)} found)")
    assigned = [n for n in ast.walk(tree) if isinstance(n, ast.Assign)
                and isinstance(n.value, ast.Constant) and isinstance(n.value.value, float)
                and n.value.value in bands]
    check(not assigned, "and binds none of them to a name")
    check(J9.PIN_CONTRACT == "95a47d5317be4b1a2f55084fcb3548e479c2333093adc29b4205ad150d48e461",
          "the sole penetration authority is the architect's pinned contract")

    # the two frozen runners J9 imports are PINNED, so verify_prereg authenticates them
    for rel in ("v26c_j1_collect.py", "v26c_j3_closed_loop.py"):
        check(amend["pinned_artefacts_sha256"].get(rel) == J9._sha_file(HERE / rel),
              f"{rel} is pinned in the prereg and verifies on disk")
    check(set(prereg["imported_frozen_runners"]) >= {"v26c_j1_collect.py",
                                                     "v26c_j3_closed_loop.py"},
          "and the amendment records why they are pinned")
    e = None
    saved_pin = J9.PIN_PREREG
    try:
        J9.PIN_PREREG = "0" * 64
        e = expect(J9.verify_amendment, J9.J9Error, "a changed amendment")
    finally:
        J9.PIN_PREREG = saved_pin
    check("amendment changed" in str(e),
          "verify_amendment is fail-closed on its own hash")

    # accurate Ray wording: nothing claims 'ray: false' any more
    pr = J9.preflight()
    check("ray" not in pr["runtime"] and pr["runtime"]["ray_cluster_started"] is False
          and pr["runtime"]["ray_workers_started"] is False
          and pr["runtime"]["env_runners"] is False
          and pr["runtime"]["rllib_checkpoint_loader"] is True,
          "the runtime block distinguishes 'no cluster/worker/EnvRunner' from 'no ray import'")
    check("importing them is not starting a cluster" in pr["runtime"]["ray_import_note"],
          "and says so explicitly")
    check("starting a Ray cluster" in J9.FORBIDDEN_HERE
          and "starting Ray workers" in J9.FORBIDDEN_HERE
          and "Ray" not in J9.FORBIDDEN_HERE,
          "the forbidden list names the ACTIONS, not the import")

    # telemetry integrity is SEPARATE
    inv = J3.telemetry_integrity(good_summary(valid_hs_count=1, valid_cycle_count=2))
    check(inv["pass"] is False, "HS below the cycle count is a telemetry-integrity failure")
    check(J9.cell_verdict(g, inv) == "INVALID",
          "which makes the cell INVALID, not FAIL: a statement about the evidence")
    check("telemetry_integrity" not in g["checks"]
          and g["telemetry_integrity_evaluated_separately"] is True,
          "and it is never one of the behavioural checks")

    # ---------------------------------------------------------------- inert preflight -----------
    pre = J9.preflight()
    check(pre["verdict"] == "GO" and pre["blockers"] == [] and pre["read_only"] is True,
          "the preflight is GO and read-only")
    check(snapshot(HERE) == before_all,
          "MEASURED: not one byte under the validation root changed during the preflight")
    check(pre["inert"]["environment_constructed"] is False
          and pre["inert"]["environment_reset"] is False
          and pre["inert"]["environment_stepped"] is False
          and pre["inert"]["module_loaded"] is False
          and pre["inert"]["outputs_written"] is False
          and pre["inert"]["ppo_updates"] == 0
          and pre["inert"]["critic_touched"] is False,
          "and declares no env construct/reset/step, no module load, no write, no critic, no PPO")
    check(pre["inert"]["heavy_modules_introduced_by_preflight"] == []
          and not any(m in sys.modules for m in ("torch", "ray", "opensim", "env_factory",
                                                 "rollout_eval")),
          "MEASURED: no torch, RLlib, OpenSim, env_factory or rollout_eval import")
    check(len(pre["planned_matrix"]) == 6
          and [c["id"] for c in pre["planned_matrix"]] == list("ABCDEF")
          and all(c["offset_unit"] == "seconds" for c in pre["planned_matrix"]),
          "the preflight reports the planned matrix, in seconds")
    md = pre["multistart_disambiguation"]
    check("DEFERRED" in md["training_data"] and "cells B and C" in md["closed_loop_validation"]
          and "BINDING" in md["closed_loop_validation"]
          and "never be conflated" in md["they_are_different"],
          "and distinguishes deferred multistart TRAINING DATA from binding multistart "
          "closed-loop VALIDATION")
    check(pre["matrix_policy"]["behavioural_fail_fast"] is False
          and "6/6" in pre["matrix_policy"]["aggregate_pass_iff"],
          "no behavioural fail-fast; aggregate needs 6/6")

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
    banned = {"torch", "ray", "env_factory", "opensim", "rollout_eval", "gymnasium"}

    def guarded_import(name, *a, **k):
        if name.split(".")[0] in banned:
            tripped.append(f"import {name}")
            raise AssertionError(f"heavy import {name}")
        return real_import(name, *a, **k)

    builtins.__import__ = guarded_import
    try:
        pre2 = J9.preflight()
    finally:
        os.rename, Path.mkdir = real["rename"], real["mkdir"]
        Path.write_text, shutil.rmtree = real["write_text"], real["rmtree"]
        np.savez_compressed = real["savez"]
        builtins.open, builtins.__import__ = real["open"], real_import
    check(tripped == [], f"PROVEN INERT: no write primitive, no heavy import ({tripped})")
    check(pre2["runtime"]["env_config_sha256"] == pre["runtime"]["env_config_sha256"],
          "and two preflights build the identical env config")
    e = expect(lambda: J9.main(["--out", "/tmp/x"]), J9.J9Error,
               "--out without --run must be refused")
    check("meaningless without --run" in str(e), "with a reason")

    # ---------------------------------------------------------------- destination guards --------
    leaf = J9.authorized_leaf()
    check(leaf == HERE / "j9r1_runs" / "j9r1_closed_loop_v26c_2026-08-26_r1",
          "the J9R1 leaf is distinct from the J9 one")
    check(leaf != HERE / "j9_runs" / "j9_closed_loop_v26c_2026-08-26_r1",
          "and can never collide with it")
    e = expect(lambda: J9.run_matrix(str(leaf), None), J9.J9Error, "a missing stage token")
    check("--authorized-stage must be exactly" in str(e), "naming the exact token")
    for bad in ("V26C-J8-RECOVERY-FIT", "V26C-J9-CLOSED-LOOP-QUALIFICATION",
                "v26c_j9r1_closed_loop", ""):
        expect(lambda b=bad: J9.run_matrix(str(leaf), b), J9.J9Error,
               f"the token {bad!r} must be refused")
    e = expect(lambda: J9.run_matrix(None, J9.STAGE), J9.J9Error, "--run without --out")
    check("requires --out" in str(e), "and say so")
    for bad_out in (str(HERE), str(HERE / "j9r1_runs"), str(HERE / "j9_runs"),
                    str(HERE / "j8_runs")):
        e = expect(lambda b=bad_out: J9.run_matrix(b, J9.STAGE), J9.J9Error,
                   f"--out {bad_out} is not the authorised leaf")
        check("not the authorised leaf" in str(e), f"refused: {Path(bad_out).name}")
    check(snapshot(HERE) == before_all and not (HERE / "j9r1_runs").exists(),
          "MEASURED: every refusal wrote nothing; j9r1_runs does not exist")

    # ---------------------------------------------------------------- the full matrix, synthetic -
    saved_root = J9.OUTPUT_ROOT_OVERRIDE
    tmp = Path(tempfile.mkdtemp(prefix="v26c_j9_matrix_test_"))
    try:
        J9.OUTPUT_ROOT_OVERRIDE = tmp
        target = J9.authorized_leaf()
        lock = target.parent / J9.LOCK_NAME
        check(target == tmp / "j9r1_runs" / "j9r1_closed_loop_v26c_2026-08-26_r1",
              "the override redirects the ROOT only")
        e = expect(lambda: J9.run_matrix(str(target), J9.STAGE), J9.J9Error,
                   "the authorised matrix must refuse a set override without an injected stack")
        check("permitted only for synthetic" in str(e), "the override is test-only")

        # patch the per-step helpers the fake stack cannot provide
        real_det, real_pros, real_sum, real_ctr = (J3.deterministic_action, J1._prosthetic_state,
                                                   J1._summarise, J3._verify_runtime_contract)
        names = tuple(json.loads((J9.J8_MODULE_DIR / "actor_feature_manifest.json").read_text())
                      ["actor_feature_names"])
        state = {"n": 0}

        def fake_det(module, obs, shape, *, torch_mod=None):
            m = np.zeros(shape, dtype=np.float32)
            return m.copy(), m.copy(), np.full(shape, 0.005, dtype=np.float32), "fake_inference"

        def fake_pros(actor_obs, feature_names):
            knee, ankle = good_kin()
            i = state["n"] % 500
            state["n"] += 1
            return {"pros_knee_angle": float(knee[i]), "pros_ankle_angle": float(ankle[i])}

        def fake_sum(trace, end_reason, clipped, rms):
            return good_summary(end_reason=end_reason, action_clipped_steps=clipped,
                                realized_noise_rms=list(rms), steps=len(trace))

        def fake_ctr(base, module, obs, expected, cfg_):
            return {"actor_feature_names": list(expected)}

        def fake_held(module, obs, shape, unit):
            m = np.zeros(shape, dtype=np.float32)
            std = np.full(shape, 0.005, dtype=np.float32)
            noise = (std * unit).astype(np.float32)
            return (m + noise).astype(np.float32), m, std, noise

        class FakeHeld:
            def __init__(self, rng, shape, hold):
                self.rng, self.shape, self.hold = rng, shape, int(hold)
                assert self.hold == 1

            def next(self):
                return self.rng.standard_normal(self.shape).astype(np.float32)

        J3.deterministic_action = fake_det
        J1._prosthetic_state = fake_pros
        J1._summarise = fake_sum
        J3._verify_runtime_contract = fake_ctr
        stack = FakeStack()
        stack.held_stochastic_action = fake_held
        stack.held_normal = FakeHeld
        FakeEnv.built.clear()
        FakeEnv.closed.clear()
        FakeEnv.saved.clear()
        try:
            res = J9.run_matrix(str(target), J9.STAGE, stack=stack, progress=False)
        finally:
            J3.deterministic_action, J1._prosthetic_state = real_det, real_pros
            J1._summarise, J3._verify_runtime_contract = real_sum, real_ctr

        check(res["verdict"] == "PASS" and res["aggregate_pass"] is True,
              "the synthetic matrix aggregates to PASS")
        check(len(res["cells"]) == 6 and [c["id"] for c in res["cells"]] == list("ABCDEF"),
              "all six cells ran, in order")
        check(res["authoritative"] is False, "and it declares itself non-authoritative")
        check(len(FakeEnv.built) == 6 and len(FakeEnv.closed) == 6,
              "SIX environments were built and SIX were closed: one at a time, cleanly")
        # THE CORRECTION, observed at run time: every env got recording on, and every one saved
        check(all(b["record_outputs"] is True and b["save_outputs_on_close"] is True
                  for b in FakeEnv.built),
              "MEASURED AT RUN TIME: every one of the six environments received both recording "
              "flags as True")
        check(all(b["output_prefix"] for b in FakeEnv.built)
              and len({b["output_prefix"] for b in FakeEnv.built}) == 1,
              "each received an output_prefix, and the SAME one: it is inherited, never per-cell")
        check(len(FakeEnv.saved) == 6,
              "and all six wrote their sim_outputs in close(), which is the only place they can")
        check(sorted(Path(s).name for s in FakeEnv.saved)
              == sorted(f"j9_cell_{c}_sim_outputs" for c in "ABCDEF"),
              "into the preregistered per-cell layout j9_cell_<ID>_sim_outputs")
        offs = [b["offset"] for b in FakeEnv.built]
        check(offs == [J9.OFFSET_NOMINAL, J9.OFFSET_MINUS, J9.OFFSET_PLUS] +
              [J9.OFFSET_NOMINAL] * 3,
              f"each env got its declared offset in seconds ({offs})")
        check(stack.seeded == [123, 123, 123, 123, 124, 125],
              f"and each cell seeded its own value ({stack.seeded})")

        rc = json.loads((target / J9.RECEIPT_NAME).read_text())
        check(rc["stage"] == J9.STAGE and rc["cells_total"] == 6
              and rc["cells_behavioural_pass"] == 6 and rc["cells_telemetry_valid"] == 6,
              "the receipt records 6/6 behavioural and 6/6 telemetry-valid")
        check(rc["actor_unchanged"] is True
              and rc["actor_before"]["artefacts_sha256"] == rc["actor_after"]["artefacts_sha256"]
              == dict(J9.PIN_J8),
              "the actor hash is checked BEFORE and AFTER and is unchanged")
        check(rc["inert"] == {"fit_executed": False, "critic_touched": False, "ppo_updates": 0,
                              "actor_edited": False, "actor_copied": False,
                              "logstd_head_edited": False, "rllib_checkpoint_loader": True,
                              "ray_cluster_started": False, "ray_workers_started": False,
                              "env_runners": False},
              "no fit, no critic, no PPO, no actor edit; no Ray CLUSTER, WORKER or EnvRunner - "
              "while the RLlib checkpoint loader is honestly declared")
        check(rc["outcome"]["deployable"] is False and rc["outcome"]["promotion"] == "NONE"
              and rc["outcome"]["next_stage_authorized"] is False,
              "and it confers nothing")
        for cell in rc["cells"]:
            rk = cell["reset_check"]
            check(rk["reset_time_error_s"] <= 1e-9,
                  f"cell {cell['id']}: reset time matches t_start + offset within 1e-9")
            check(rk["cfg_t_start_s"] == 11.99
                  and abs(rk["expected_reset_time_s"]
                          - (11.99 + cell["episode_start_offset_s"])) < 1e-12,
                  f"cell {cell['id']}: expected = cfg.t_start + offset, NOT the offset itself")
            check(set(rk) >= {"cfg_t_start_s", "cfg_t_end_s", "episode_duration_s",
                              "requested_unclamped_time_s", "max_start_s",
                              "expected_reset_time_s", "actual_reset_time_s",
                              "reset_time_error_s", "clamped"},
                  f"cell {cell['id']}: every reset field is recorded")
            check(rk["clamped"] is False and rk["checked_immediately_after_reset"] is True,
                  f"cell {cell['id']}: unclamped here, and checked right after reset")
            check(cell["env_mutation"]["unit"] == "seconds"
                  and set(cell["env_mutation"]["mutated_keys"])
                  <= {"episode_start_offset_s", "output_dir"},
                  f"cell {cell['id']}: only the offset (in seconds) and output_dir moved")
            check(cell["action_semantics"]["stepped_with"] == "raw_action"
                  and cell["action_semantics"]["clipping_is_diagnostic"] is True,
                  f"cell {cell['id']}: the RAW action is stepped; clipping is diagnostic")
            check(cell["env_closed"] is True, f"cell {cell['id']}: its environment was closed")
            check(set(cell["outputs_sha256"]) == {f"j9_cell_{cell['id']}_trace.json",
                                                  f"j9_cell_{cell['id']}_kinematics.npz",
                                                  f"j9_cell_{cell['id']}_penetration.npz"},
                  f"cell {cell['id']}: trace, kinematics and penetration are all preserved")
        det = [c for c in rc["cells"] if c["mode"] == "deterministic"]
        sto = [c for c in rc["cells"] if c["mode"] == "stochastic_held"]
        check(len(det) == 3 and all(c["action_semantics"]["max_abs_action_minus_mean"] == 0.0
                                    for c in det),
              "in every deterministic cell the action IS the mean, exactly")
        check(len(sto) == 3 and all(c["action_semantics"]["sigma"] == 0.005
                                    and c["action_semantics"]["noise_hold_steps"] == 1
                                    for c in sto),
              "and every stochastic cell used sigma 0.005 with hold 1")
        check(not lock.exists() and res["lock_released"] is True,
              "the lock was released after the commit")
        check(not (target.parent / J9.STAGING_NAME).exists(), "no staging survives")
        json.dumps(rc, allow_nan=False)
        check(True, "the receipt is strictly JSON: no NaN, no Infinity")

        after_first = snapshot(target)
        e = expect(lambda: J9.validate_out(str(target)), J9.J9Error, "a second run")
        check("no-clobber and single-execution" in str(e), "is refused: the leaf exists")
        check(snapshot(target) == after_first, "and the existing leaf is untouched")

        # a foreign lock is fail-closed; a failure preserves concurrent content
        shutil.rmtree(target)
        lock.write_text("held", encoding="utf-8")
        e = expect(lambda: J9.run_matrix(str(target), J9.STAGE, stack=FakeStack()), J9.J9Error,
                   "a held lock must stop the matrix")
        # the inert preflight catches it BEFORE the acquisition, which is stricter, not weaker
        check("removes no lock it does not own" in str(e)
              and ("preflight BLOCKED" in str(e) or "lock already exists" in str(e)),
              f"fail-closed, and it removes no lock it does not own ({str(e)[:120]})")
        check("lock" in str(e).lower(), "the refusal names the lock")
        check(lock.read_text() == "held", "MEASURED: the foreign lock was left untouched")
        lock.unlink()
        sentinel = tmp / "j9r1_runs" / "concurrent.txt"
        sentinel.write_text("other", encoding="utf-8")
        broken = FakeStack()
        broken.make_env = lambda cfg: (_ for _ in ()).throw(RuntimeError("synthetic env failure"))
        e = expect(lambda: J9.run_matrix(str(target), J9.STAGE, stack=broken), RuntimeError,
                   "a technical exception must fail closed")
        check(not target.exists(), "no leaf was committed")
        check(not (target.parent / J9.STAGING_NAME).exists(), "the staging was removed")
        check(not lock.exists(), "the run released its own lock")
        check(sentinel.is_file(), "and the concurrent sentinel SURVIVED")

        # ------- a recording-OFF base is refused BEFORE any env is constructed or stepped -------
        FakeEnv.built.clear()
        FakeEnv.closed.clear()
        FakeEnv.saved.clear()
        real_base = J9.base_env_config

        def recording_off_base(root):
            env, cfg_ = real_base(root)
            env = dict(env)
            env["record_outputs"] = False          # the J9 configuration, smuggled back in
            return env, cfg_

        J9.base_env_config = recording_off_base
        st_off = FakeStack()
        st_off.held_stochastic_action = fake_held
        st_off.held_normal = FakeHeld
        try:
            e = expect(lambda: J9.run_matrix(str(target), J9.STAGE, stack=st_off, progress=False),
                       J9.J9Error, "a recording-off base must be refused")
        finally:
            J9.base_env_config = real_base
        check("record_outputs" in str(e) and "expected exactly True" in str(e)
              and "Refusing BEFORE" in str(e),
              f"naming the flag and the expectation ({str(e)[:90]})")
        check(FakeEnv.built == [] and FakeEnv.closed == [],
              "PROVEN: NO environment was constructed and none was stepped - the refusal happens "
              "before the 500-step run, not after it as in J9")
        check(not target.exists() and not (target.parent / J9.STAGING_NAME).exists()
              and not (target.parent / J9.LOCK_NAME).exists(),
              "and nothing was committed, staged or locked")

        # env.close() is FAIL-CLOSED: a failed close with no primary exception is technical
        sentinel.unlink()
        real_det2, real_pros2, real_sum2, real_ctr2 = (J3.deterministic_action,
                                                       J1._prosthetic_state, J1._summarise,
                                                       J3._verify_runtime_contract)
        J3.deterministic_action = fake_det
        J1._prosthetic_state = fake_pros
        J1._summarise = fake_sum
        J3._verify_runtime_contract = fake_ctr
        FakeEnv.close_raises = True
        st2 = FakeStack()
        st2.held_stochastic_action = fake_held
        st2.held_normal = FakeHeld
        try:
            e = expect(lambda: J9.run_matrix(str(target), J9.STAGE, stack=st2, progress=False),
                       Exception, "a failed env.close must fail the matrix technically")
        finally:
            FakeEnv.close_raises = False
            J3.deterministic_action, J1._prosthetic_state = real_det2, real_pros2
            J1._summarise, J3._verify_runtime_contract = real_sum2, real_ctr2
        check(isinstance(e, J9.J9Error) and "failed to close cleanly" in str(e),
              f"FAIL-CLOSED: a failed close raises a J9Error ({type(e).__name__})")
        check(e.__cause__ is not None and isinstance(e.__cause__, RuntimeError)
              and "synthetic close failure" in str(e.__cause__),
              "CHAINED to the original cause, never swallowed and never replaced")
        check("synthetic close failure" in str(e),
              "and the original message is carried in the J9Error text too")
        check(not target.exists(), "NO leaf was committed on a failed close")
        check(not (target.parent / J9.STAGING_NAME).exists(), "the staging was removed")
        check(not (target.parent / J9.LOCK_NAME).exists(), "and the lock was released")
        # STATIC: close is not swallowed on the success path
        rc_fn = next(n for n in ast.walk(tree)
                     if isinstance(n, ast.FunctionDef) and n.name == "run_cell")
        handlers = [h for n in ast.walk(rc_fn) if isinstance(n, ast.Try) for h in n.handlers]
        bare_close = [h for h in handlers
                      if any(isinstance(c, ast.Call) and isinstance(c.func, ast.Attribute)
                             and c.func.attr == "close" for c in ast.walk(h))]
        check(len(bare_close) == 1,
              "close is guarded in exactly one handler - the primary-exception path, which must "
              "not be masked")
        check(any(isinstance(n, ast.Try) and n.orelse for n in ast.walk(rc_fn)),
              "and the success path closes in an else block, where a failure raises")
        chained = [n for n in ast.walk(rc_fn) if isinstance(n, ast.Raise) and n.cause is not None
                   and isinstance(n.exc, ast.Call) and isinstance(n.exc.func, ast.Name)
                   and n.exc.func.id == "J9Error"]
        check(chained, "STATIC: the close failure raises J9Error ... from exc")
        # STATIC: the success-path close catches Exception, NOT BaseException
        else_handlers = [h for n in ast.walk(rc_fn) if isinstance(n, ast.Try) and n.orelse
                         for inner in n.orelse if isinstance(inner, ast.Try)
                         for h in inner.handlers]
        check(len(else_handlers) == 1
              and isinstance(else_handlers[0].type, ast.Name)
              and else_handlers[0].type.id == "Exception",
              "STATIC: the success-path close catches Exception, never BaseException - an "
              "interrupt must not be rewritten as a stage error")
        outer = [h for n in ast.walk(rc_fn) if isinstance(n, ast.Try) for h in n.handlers
                 if isinstance(h.type, ast.Name) and h.type.id == "BaseException"]
        check(outer, "while the primary-exception handler still catches BaseException")

        # DYNAMIC: a KeyboardInterrupt from close propagates UNREWRITTEN, and cleanup still runs
        class InterruptingEnv(FakeEnv):
            def close(self):
                raise KeyboardInterrupt("operator stop")

        st3 = FakeStack()
        st3.held_stochastic_action = fake_held
        st3.held_normal = FakeHeld
        st3.make_env = lambda cfg: InterruptingEnv(cfg)
        J3.deterministic_action = fake_det
        J1._prosthetic_state = fake_pros
        J1._summarise = fake_sum
        J3._verify_runtime_contract = fake_ctr
        try:
            e2 = expect(lambda: J9.run_matrix(str(target), J9.STAGE, stack=st3, progress=False),
                        KeyboardInterrupt, "a KeyboardInterrupt from close must propagate")
        finally:
            J3.deterministic_action, J1._prosthetic_state = real_det2, real_pros2
            J1._summarise, J3._verify_runtime_contract = real_sum2, real_ctr2
        check(isinstance(e2, KeyboardInterrupt) and not isinstance(e2, J9.J9Error),
              "PROVEN: the interrupt is NOT rewritten as a J9Error")
        check("operator stop" in str(e2), "and it carries the operator's own message")
        check(not target.exists(), "no leaf was committed on an interrupt")
        check(not (target.parent / J9.STAGING_NAME).exists()
              and not (target.parent / J9.LOCK_NAME).exists(),
              "and the outer cleanup STILL removed the staging and the lock: fail-closed on an "
              "interrupt too")

        # ------------------- the LIVE runtime values are normative and guarded ------------------
        def run_broken(**flags):
            for k, v in flags.items():
                setattr(FakeEnv, k, v)
            J3.deterministic_action = fake_det
            J1._prosthetic_state = fake_pros
            J1._summarise = fake_sum
            J3._verify_runtime_contract = fake_ctr
            s3 = FakeStack()
            s3.held_stochastic_action = fake_held
            s3.held_normal = FakeHeld
            try:
                return expect(lambda: J9.run_matrix(str(target), J9.STAGE, stack=s3,
                                                    progress=False),
                              J9.J9Error, f"the guard for {flags} must fire")
            finally:
                for k in flags:
                    setattr(FakeEnv, k, _UNSET if k == "live_duration" else False)
                J3.deterministic_action, J1._prosthetic_state = real_det2, real_pros2
                J1._summarise, J3._verify_runtime_contract = real_sum2, real_ctr2

        e = run_broken(drop_cfg=True)
        check("exposes no .cfg" in str(e),
              "a missing base.cfg is an explicit J9Error, not an AttributeError")
        check(not target.exists() and not (target.parent / J9.LOCK_NAME).exists(),
              "and nothing is committed, with the lock released")
        e = run_broken(drop_env_cfg=True)
        check("exposes no env_cfg.episode_duration" in str(e)
              and "refuses to guess" in str(e),
              "a missing base.env_cfg.episode_duration is an explicit J9Error that refuses to "
              "guess the duration")
        e = run_broken(live_duration=4.0)
        check("live episode_duration" in str(e) and "1e-12" in str(e),
              "a live duration diverging from the declared one is refused at 1e-12")
        check(not target.exists(), "and nothing is committed")
        # a divergence BELOW the tolerance is accepted
        FakeEnv.live_duration = 5.0 + 1e-13
        J3.deterministic_action = fake_det
        J1._prosthetic_state = fake_pros
        J1._summarise = fake_sum
        J3._verify_runtime_contract = fake_ctr
        s4 = FakeStack()
        s4.held_stochastic_action = fake_held
        s4.held_normal = FakeHeld
        try:
            ok = J9.run_matrix(str(target), J9.STAGE, stack=s4, progress=False)
        finally:
            FakeEnv.live_duration = _UNSET
            J3.deterministic_action, J1._prosthetic_state = real_det2, real_pros2
            J1._summarise, J3._verify_runtime_contract = real_sum2, real_ctr2
        check(ok["verdict"] == "PASS",
              "a divergence below 1e-12 is within tolerance and the matrix proceeds")
        rc2 = json.loads((target / J9.RECEIPT_NAME).read_text())
        ds = rc2["cells"][0]["reset_check"]["duration_sources"]
        check(ds["normative"] == "live_env_cfg_episode_duration"
              and ds["cross_check_tolerance_s"] == 1e-12 and ds["agree"] is True,
              "the receipt records BOTH duration sources and names the live one normative")
        check(rc2["cells"][0]["reset_check"]["sources"]["episode_duration"]
              == "live base.env_cfg.episode_duration"
              and rc2["cells"][0]["reset_check"]["sources"]["all_guarded_fail_closed"] is True,
              "and records that every reset input comes from the live environment, guarded")
        shutil.rmtree(target)
    finally:
        J9.OUTPUT_ROOT_OVERRIDE = saved_root
        shutil.rmtree(tmp, ignore_errors=True)

    # ------------------------------- a behavioural FAIL still COMMITS the evidence --------------
    tmp2 = Path(tempfile.mkdtemp(prefix="v26c_j9_fail_test_"))
    try:
        J9.OUTPUT_ROOT_OVERRIDE = tmp2
        target = J9.authorized_leaf()
        real_det, real_pros, real_sum, real_ctr = (J3.deterministic_action, J1._prosthetic_state,
                                                   J1._summarise, J3._verify_runtime_contract)
        state2 = {"n": 0}

        def failing_pros(actor_obs, feature_names):
            """An ankle that never reaches -0.03: kinematic_ankle_min must FAIL."""
            knee, _ = good_kin()
            shallow = np.linspace(-0.0099, 0.3901, 500)
            i = state2["n"] % 500
            state2["n"] += 1
            return {"pros_knee_angle": float(knee[i]), "pros_ankle_angle": float(shallow[i])}

        J3.deterministic_action = fake_det
        J1._prosthetic_state = failing_pros
        J1._summarise = fake_sum
        J3._verify_runtime_contract = fake_ctr
        st = FakeStack()
        st.held_stochastic_action = fake_held
        st.held_normal = FakeHeld
        FakeEnv.built.clear()
        FakeEnv.closed.clear()
        FakeEnv.saved.clear()
        try:
            res = J9.run_matrix(str(target), J9.STAGE, stack=st, progress=False)
        finally:
            J3.deterministic_action, J1._prosthetic_state = real_det, real_pros
            J1._summarise, J3._verify_runtime_contract = real_sum, real_ctr

        check(res["verdict"] == "FAIL" and res["aggregate_pass"] is False,
              "a behavioural failure aggregates to FAIL")
        check(target.is_dir() and (target / J9.RECEIPT_NAME).is_file(),
              "MEASURED: the FAIL evidence is STILL COMMITTED to the leaf")
        rcf = json.loads((target / J9.RECEIPT_NAME).read_text())
        check(rcf["cells_total"] == 6 and len(rcf["cells"]) == 6,
              "and ALL SIX cells ran: a behavioural FAIL never stops the matrix")
        check(len(FakeEnv.built) == 6 and len(FakeEnv.closed) == 6,
              "six environments were still built and closed")
        check(all("kinematic_ankle_min" in c["gate"]["failed"] for c in rcf["cells"]),
              "every cell records the ankle-min failure")
        check(all(c["telemetry_valid"] for c in rcf["cells"])
              and rcf["cells_telemetry_valid"] == 6,
              "while the telemetry stays valid: behavioural FAIL is not an integrity failure")
        check(rcf["cells_behavioural_pass"] == 0 and rcf["verdict"] == "FAIL",
              "0/6 behavioural PASS, aggregate FAIL")
        for c in rcf["cells"]:
            check(set(c["outputs_sha256"]) == {f"j9_cell_{c['id']}_trace.json",
                                               f"j9_cell_{c['id']}_kinematics.npz",
                                               f"j9_cell_{c['id']}_penetration.npz"},
                  f"cell {c['id']}: its full evidence is preserved despite the FAIL")
        check(not (target.parent / J9.LOCK_NAME).exists()
              and not (target.parent / J9.STAGING_NAME).exists(),
              "the lock was released and the staging removed on the FAIL path too")
        check(rcf["actor_unchanged"] is True, "and the actor is still unchanged")
    finally:
        J9.OUTPUT_ROOT_OVERRIDE = saved_root
        shutil.rmtree(tmp2, ignore_errors=True)

    # ---------------------------------------------------------------- no behavioural fail-fast --
    fail_fast = [n for n in ast.walk(tree) if isinstance(n, ast.Break)]
    run_matrix_fn = next(n for n in ast.walk(tree)
                         if isinstance(n, ast.FunctionDef) and n.name == "run_matrix")
    loop = next(n for n in ast.walk(run_matrix_fn) if isinstance(n, ast.For))
    check(not [n for n in ast.walk(loop) if isinstance(n, (ast.Break, ast.Continue))],
          "STATIC: the matrix loop contains no break and no continue - no behavioural fail-fast")
    check(not [n for n in ast.walk(loop) if isinstance(n, ast.If)
               and "behavioural_pass" in ast.dump(n)],
          "and it never branches on a cell's behavioural verdict")
    check(sci["behavioural_fail_fast"] is False
          and "still run" in prereg["matrix"]["fail_fast_rule"],
          "the preregistration declares the same")

    # ---------------------------------------------------------------- static guarantees ---------
    imported = {n.names[0].name for n in ast.walk(tree) if isinstance(n, ast.Import)}
    imported |= {(n.module or "") for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)}
    check(imported == {"__future__", "argparse", "hashlib", "importlib", "json", "os", "pickle",
                       "shutil", "sys", "pathlib", "typing", "numpy", "v26c_j1_collect",
                       "v26c_j3_closed_loop", "v26c_penetration_contract", "rollout_eval",
                       "exploration_noise"},
          f"the import surface is the stdlib, numpy, J1/J3, the contract and the two production "
          f"helpers ({sorted(imported)})")
    check("importlib.import_module(\"v26c_j9_closed_loop\")" in src,
          "the original J9 runner is imported by name ONLY inside the equivalence check, after "
          "its hash has been verified")
    top = {n.names[0].name.split(".")[0] for n in tree.body if isinstance(n, ast.Import)}
    top |= {(n.module or "").split(".")[0] for n in tree.body if isinstance(n, ast.ImportFrom)}
    check(not (top & {"torch", "ray", "opensim", "env_factory", "rollout_eval", "gymnasium"}),
          "no heavy import at module scope")
    ps = next(n for n in ast.walk(tree)
              if isinstance(n, ast.FunctionDef) and n.name == "production_stack")
    check({n.names[0].name for n in ast.walk(ps) if isinstance(n, ast.Import)}
          == {"rollout_eval", "exploration_noise"},
          "the production helpers are imported lazily, inside production_stack")
    check("_held_stochastic_action" in src and "HeldStandardNormal" in src,
          "the stochastic path is rollout_eval._held_stochastic_action with HeldStandardNormal")
    for name in ("PPOConfig", "train_ppo", "adapt_actor", "Adam", "backward", "optimizer"):
        check(name not in ids, f"the runner never references {name}")
    check(src.count("shutil.rmtree") == 1, "exactly ONE recursive removal")
    rm = [n for n in ast.walk(run_matrix_fn) if isinstance(n, ast.Call)
          and isinstance(n.func, ast.Attribute) and n.func.attr == "rmtree"]
    check(len(rm) == 1 and isinstance(rm[0].args[0], ast.Name)
          and rm[0].args[0].id == "staging_created",
          "and its only argument is staging_created")
    check("O_CREAT | os.O_EXCL" in src, "the lock uses O_CREAT|O_EXCL")

    # ---------------------------------------------------------------- nothing disturbed ---------
    check(not (HERE / "j9r1_runs").exists(),
          "MEASURED: this suite never created the authorised j9r1_runs")
    check(snapshot(HERE) == before_all,
          "MEASURED: not one byte under the validation root changed across the WHOLE suite")
    check(J9.verify_actor()["artefacts_sha256"] == dict(J9.PIN_J8),
          "and the J8 actor is byte-identical afterwards")

    print(json.dumps({"selftest": "PASS", "checks": CHECKS}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
