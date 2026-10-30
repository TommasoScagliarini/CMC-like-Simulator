#!/usr/bin/env python
"""Fail-closed tests for the V26C J12 closed-loop qualification of the J11 actor.

Pure and synthetic: no physical environment is ever constructed and no rollout is ever run. A
fake stack drives the whole six-cell matrix so the write path, the gate, the parity contracts,
the WAIT_HS counter and the post-commit verification can be exercised end to end in a temporary
root. The authorised J12 leaf is never created.

DELIBERATELY NOT BORN SPENT. The J9R1 suite asserts unconditionally that its own preflight is GO
and that its run directory does not exist; both died the moment J9R1 committed, which is why that
suite cannot be re-run today as evidence. Here every such assertion is CONDITIONAL on the leaf,
and the post-commit state is asserted instead once it exists.
"""
from __future__ import annotations
import ast, builtins, hashlib, io, json, os, pickle, shutil, sys, tempfile, tokenize
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26c_j12_closed_loop as J12  # noqa: E402   the runner under test
import v26c_j9r1_closed_loop as J9R1  # noqa: E402  the FROZEN scientific authority
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
        if p.is_file() and "__pycache__" not in p.parts:
            out[str(p.relative_to(root))] = hashlib.sha256(p.read_bytes()).hexdigest()
    return out


# ---------------------------------------------------------------- pinned truth ------------------
MANIFEST = json.loads((J12.J11_MODULE_DIR / "actor_feature_manifest.json").read_text())
NAMES = tuple(str(n) for n in MANIFEST["actor_feature_names"])
WAIT_IDX = NAMES.index("phase_fsm_wait_hs")
with (J12.J11_MODULE_DIR / "module_state.pkl").open("rb") as _fh:
    PINNED_STATE = {k: np.asarray(v) for k, v in pickle.load(_fh).items()}


def good_kin(n=500):
    t = np.linspace(0.0, 4.0 * np.pi, n)
    knee = -0.45 + 0.40 * np.sin(t)
    ankle = -0.05 + 0.22 * np.sin(t + 0.7)
    return knee, ankle


def pen_ok(n=500, maxv=0.0229):
    return np.abs(np.sin(np.linspace(0.0, np.pi, n))) * maxv


class FakeSpace:
    def __init__(self, shape=(2,)):
        self.shape = shape
        self.low = np.full(shape, -1.0, dtype=np.float32)
        self.high = np.full(shape, 1.0, dtype=np.float32)


class FakeSimCfg:
    def __init__(self, t_start=11.99, t_end=21.0):
        self.t_start = float(t_start)
        self.t_end = float(t_end)


class FakeEnvCfg:
    def __init__(self, episode_duration=5.0):
        self.episode_duration = episode_duration
        self.segment_duration = 0.01


class FakeBase:
    def __init__(self, t0, duration=5.0):
        self.t = float(t0)
        self.cfg = FakeSimCfg()
        self.env_cfg = FakeEnvCfg(duration)
        self.actor_feature_names = NAMES
        self.observation_feature_names = tuple(list(NAMES) + [f"full_{i}" for i in range(49)])
        self.n_actor = 35
        self.n_full = 84


class AsymmetricActorCriticTorchRLModule:
    """A stand-in with the EXACT class name the runner requires, carrying the pinned weights.

    The name is not decoration: the runner refuses any other class, because ray's checkpoint
    loader silently returns a weightless base RLModule when asymmetric_rl_module cannot be
    imported. A fake that did not carry the real name would not exercise that check.
    """

    empty_state = False
    wrong_weights = False

    def __init__(self, path):
        self.path = str(path)
        self._n_actor, self._n_full = 35, 84

    def get_state(self):
        if AsymmetricActorCriticTorchRLModule.empty_state:
            return {}
        st = {k: np.array(v, copy=True) for k, v in PINNED_STATE.items()}
        if AsymmetricActorCriticTorchRLModule.wrong_weights:
            st["pi.1.bias"] = st["pi.1.bias"] + np.float32(1.0)
        return st


class BareRLModule:
    """What ray actually hands back when the checkpoint class cannot be imported."""

    def __init__(self, path):
        self.path = str(path)

    def get_state(self):
        return {}


class FakeEnv:
    built: list[dict] = []
    closed: list[str] = []
    saved: list[str] = []

    close_raises = False
    n_output_files = 19
    wait_hs_steps = 0            # how many steps report phase_fsm_wait_hs == 1
    bad_cycles = False           # emit only one valid cycle, failing the binding gate

    def __init__(self, cfg):
        self.cfg = dict(cfg)
        self.offset = float(cfg["episode_start_offset_s"])
        self.start = J12.expected_reset_time(11.99, 21.0, cfg.get("episode_duration"),
                                             self.offset)["expected_reset_time_s"]
        self.unwrapped = FakeBase(self.start, cfg.get("episode_duration"))
        self.action_space = FakeSpace()
        self.n = 0
        self.record_outputs = bool(cfg.get("record_outputs", False))
        self.save_outputs_on_close = bool(cfg.get("save_outputs_on_close", False))
        self.output_prefix = cfg.get("output_prefix")
        self.out = Path(cfg["output_dir"])
        FakeEnv.built.append({"offset": self.offset, "output_dir": str(self.out),
                              "record_outputs": self.record_outputs,
                              "save_outputs_on_close": self.save_outputs_on_close})

    def _obs(self):
        # the production env returns the FULL 84-wide observation; the actor block is the
        # first 35 columns, which is what run_cell slices off
        v = np.zeros(84, dtype=np.float32)
        if self.n < FakeEnv.wait_hs_steps:
            v[WAIT_IDX] = 1.0
        knee, ankle = good_kin()
        i = min(self.n, 499)
        v[NAMES.index("pros_knee_angle")] = float(knee[i])
        v[NAMES.index("pros_ankle_angle")] = float(ankle[i])
        return v

    def reset(self, seed=None):
        self.n = 0
        self.unwrapped.t = self.start
        return self._obs(), {}

    def step(self, action):
        self.n += 1
        self.unwrapped.t = self.start + 0.01 * self.n
        cycles = 1 if FakeEnv.bad_cycles else min(3, 1 + self.n // 200)
        hs = cycles if FakeEnv.bad_cycles else cycles + (1 if self.n > 450 else 0)
        info = {"time": self.unwrapped.t,
                "end_reason": "episode_time_limit" if self.n >= 500 else "",
                "reward_terms": {"grf_penetration_m": float(pen_ok()[self.n - 1]),
                                 "phase_valid_cycle_count": float(cycles),
                                 "phase_valid_hs_count": float(hs),
                                 "phase_valid_to_count": float(cycles),
                                 "phase_timeout_exceeded": 0.0, "phase_timeout_side": 0.0,
                                 "morphology_causal_failed_closed": 0.0},
                J1.FSM_KEY: {"resync_count": 0, "hs_cancelled_count": 0,
                             "valid_cycle_count": cycles, "valid_hs_count": hs,
                             "valid_to_count": cycles, "timeout_exceeded": 0.0,
                             "timeout_side": 0.0,
                             "fsm_behaviour_version": J1.EXPECTED_FSM_BEHAVIOUR_VERSION,
                             "event_source": J1.EXPECTED_EVENT_SOURCE}}
        return self._obs(), 0.08, False, self.n >= 500, info

    def close(self):
        if FakeEnv.close_raises:
            raise RuntimeError("synthetic close failure")
        if self.record_outputs and self.save_outputs_on_close:
            self.out.mkdir(parents=True, exist_ok=True)
            prefix = self.output_prefix or "rl_episode"
            for k in range(FakeEnv.n_output_files):
                (self.out / f"{prefix}_{k:02d}.sto").write_text("synthetic", encoding="utf-8")
            FakeEnv.saved.append(str(self.out))
        FakeEnv.closed.append(str(self.cfg["output_dir"]))


class FakeHeld:
    def __init__(self, rng, shape, hold):
        self.rng, self.shape = rng, tuple(shape)
        assert int(hold) == 1
        self.calls = 0

    def next(self):
        self.calls += 1
        return self.rng.standard_normal(self.shape).astype(np.float32)


MEAN = np.array([0.10, -0.05], dtype=np.float32)
STD = np.full(2, J12.SIGMA, dtype=np.float32)


class FakeStack:
    name = "synthetic"
    operational = False
    module_cls = AsymmetricActorCriticTorchRLModule

    def __init__(self):
        self.seeded: list[int] = []
        self.torch = None
        self.held_normal = FakeHeld
        self.reference_action = lambda module, obs, shape: (
            MEAN.reshape(shape).copy(), MEAN.reshape(shape).copy(), STD.reshape(shape).copy(),
            "synthetic.reference")
        self.held_stochastic_action = lambda module, obs, shape, unit: (
            (MEAN.reshape(shape) + STD.reshape(shape) * unit).astype(np.float32),
            MEAN.reshape(shape).copy(), STD.reshape(shape).copy(),
            (STD.reshape(shape) * unit).astype(np.float32))

    def seed(self, s):
        self.seeded.append(int(s))

    def load_module(self, path):
        return FakeStack.module_cls(path)

    def make_env(self, cfg):
        return FakeEnv(cfg)


def fake_deterministic_action(module, obs, shape, torch_mod=None):
    """Stands in for J3's torch kernel.

    J3.deterministic_action is FROZEN and has its own suite; re-testing its torch arithmetic here
    would only prove that torch works. What this suite must prove is J12's own logic - the matrix,
    the gate, the WAIT_HS counter, the commit - so the kernel is substituted, and the substitution
    is asserted to be exactly the call site the runner uses.
    """
    return (MEAN.reshape(shape).copy(), MEAN.reshape(shape).copy(), STD.reshape(shape).copy(),
            "synthetic.deterministic")


def main() -> int:
    src = (HERE / "v26c_j12_closed_loop.py").read_text()
    tree = ast.parse(src)
    ids = {t.string for t in tokenize.generate_tokens(io.StringIO(src).readline)
           if t.type == tokenize.NAME}
    prereg = json.loads(J12.PREREG.read_text())
    before_all = snapshot(HERE)
    leaf_exists_at_start = J12.authorized_leaf().exists()

    # ---------------------------------------------------------------- preregistration ----------
    check(prereg["kind"] == "ADDITIVE, IMMUTABLE, PREREGISTRATION"
          and prereg["stage_proposed"] == "V26C_J12_CLOSED_LOOP_QUALIFICATION" == J12.STAGE,
          "the prereg is additive, immutable and proposes the J12 stage")
    check("stage_authorised" not in prereg
          and prereg["authorisation_status"].startswith("NOT_GRANTED"),
          "it does not claim an authorisation it cannot grant")
    check(J12._sha_file(J12.PREREG) == J12.PIN_PREREG
          == "5f0ed000b62c7c64ceaca8eca0d83d36e863619bf528a285aa7d596d63120bbe",
          "and is pinned by exact hash")
    check(all(J12._sha_file(HERE / r) == h
              for r, h in prereg["pinned_artefacts_sha256"].items()),
          f"all {len(prereg['pinned_artefacts_sha256'])} local pins verify on disk")
    check(all(J12._sha_file(J12.REPO / r) == h
              for r, h in prereg["pinned_repo_artefacts_sha256"].items()),
          f"all {len(prereg['pinned_repo_artefacts_sha256'])} repo pins verify on disk")
    for needed in ("rollout_eval.py", "exploration_noise.py", "osim_trj_cmc_like.py",
                   "env_factory.py", "asymmetric_rl_module.py", "training_cfg.resolved.yaml"):
        check(any(r.endswith(needed) for r in prereg["pinned_repo_artefacts_sha256"]),
              f"the runtime source {needed} is pinned")

    # ---------------------------------------------------------------- lineage ------------------
    check(prereg["actor"]["stage"] == "V26C_J11_MULTISTART_FIT"
          and prereg["actor"]["is_j8"] is False and prereg["actor"]["is_j4"] is False
          and prereg["actor"]["is_july"] is False,
          "the actor is J11: not J8, not J4, not July")
    check("j11_runs" in str(J12.J11_LEAF) and "j8_runs" not in str(J12.J11_LEAF),
          "and the runner points at the J11 leaf")
    j8_state = HERE / "j8_runs" / "j8_recovery_fit_v26c_2026-08-26_r1" / "rl_module" \
        / "module_state.pkl"
    check(J12._sha_file(j8_state) != J12.PIN_J11_MODULE_STATE,
          "MEASURED: the J8 module is NOT this stage's actor")
    check(len(J12.PIN_J11) == 8,
          "the J11 leaf is pinned file by file - EIGHT files, not J8's six")
    check(sorted(J12.PIN_J11) == sorted(
              str(p.relative_to(J12.J11_LEAF)).replace(os.sep, "/")
              for p in J12.J11_LEAF.rglob("*") if p.is_file()),
          "MEASURED: and the pin table is exactly the leaf's file set")

    # ---------------------------------------------------------------- the actor ----------------
    actor = J12.verify_actor()
    check(actor["file_count"] == 8, "verify_actor checks all eight files")
    check(actor["leaf_validity"]["commit_verification_pass"] is True
          and actor["leaf_validity"]["technical_invalid_marker"] is False
          and actor["leaf_validity"]["no_stale_lock_or_staging"] is True,
          "the J11 leaf is consumed only because its own commit verification PASSED")
    check(actor["mask_contract"]["clock_columns"] == [0, 1]
          and actor["mask_contract"]["clock_exactly_zero"] is True
          and actor["mask_contract"]["controller_live"] is True,
          "the mask contract: clock EXACTLY zero, controller LIVE")
    for key, layer in actor["mask_contract"]["layers"].items():
        check(layer["zero_columns"] == [0, 1],
              f"{key}: the ONLY zero columns are the clock")
        check(all(v > 0.0 for v in layer["controller_norms"].values()),
              f"{key}: every controller column is live")
    check(abs(actor["sigma"]["max_abs_deviation"]) <= J12.SIGMA_TOLERANCE
          and actor["sigma"]["logstd_head_edited"] is False,
          f"sigma is the actor's own, within {J12.SIGMA_TOLERANCE} "
          f"(deviation {actor['sigma']['max_abs_deviation']:.3g})")
    check(actor["lineage"]["parent_module_state_sha256"] == J12.PIN_J2_PARENT
          and actor["lineage"]["j7_dataset_sha256"] == J12.PIN_J7_DATASET
          and actor["lineage"]["cell_B_dataset_sha256"] == J12.PIN_J10R1_CELL_B
          and actor["lineage"]["cell_C_dataset_sha256"] == J12.PIN_J10R1_CELL_C
          and actor["lineage"]["aggregate_rows"] == 24713
          and actor["lineage"]["j11_verdict"] == "PASS",
          "the J11 receipt's lineage is read from ITS schema and every hash matches")
    check(actor["manifest"]["module_state_sha256"] == J12.PIN_J11_MODULE_STATE
          and actor["manifest"]["regenerated_not_copied"] is True
          and actor["manifest"]["deployable"] is False,
          "the J11 manifest describes the module beside it - the J8 defect is not inherited")
    j8_man = json.loads((HERE / "j8_runs" / "j8_recovery_fit_v26c_2026-08-26_r1" / "rl_module"
                         / "actor_feature_manifest.json").read_text())
    check(j8_man["module_state_sha256"] != J12._sha_file(j8_state),
          "MEASURED: J8's own manifest names a DIFFERENT module - the defect J11 corrected")

    # ---------------------------------------------------------------- the science ---------------
    eq = J12.verify_scientific_equivalence()
    check(eq["thresholds_invented_here"] == 0, "no threshold is invented here")
    check(eq["source_sha256"] == J12.PIN_J9R1_MODULE, "the J9R1 source is pinned by hash")
    check(J12.J12_COMMON_GATE == J9R1.J9_COMMON_GATE
          == {k: v for k, v in J1.J1_GATE.items() if k != "max_penetration_m_max"},
          "the common gate is J9R1's, which is J1_GATE minus the 0.020 soft bar")
    check(J12.J12_COMMON_GATE == {"steps_required": 500, "end_reason": "episode_time_limit",
                                  "valid_cycles_min": 2, "phase_timeout_stance_max": 0,
                                  "phase_timeout_swing_max": 0,
                                  "morphology_causal_contract_failure_max": 0,
                                  "hs_cancelled_count_max": 0, "resync_count_max": 1},
          "and every one of its eight values matches its pinned literal")
    check("max_penetration_m_max" not in J12.J12_COMMON_GATE,
          "the 0.020 bar is DROPPED: the sole binding penetration bar is 0.028")
    check(J12.J12_KINEMATIC_GATE == dict(J3.J3_KINEMATIC_GATE)
          == {"ankle_min_rad_max": -0.03, "ankle_amplitude_min_rad": 0.30,
              "knee_amplitude_min_rad": 0.60, "knee_strictly_flexed": True,
              "knee_bounds_rad": (-1.5, 0.0), "ankle_bounds_rad": (-0.7, 0.7)},
          "the kinematic gate is J3's, value for value")
    check((J12.SIGMA, J12.SIGMA_TOLERANCE, J12.NOISE_HOLD_STEPS, J12.EXPECTED_STEPS,
           J12.RESET_TIME_TOLERANCE_S) == (0.005, 1e-6, 1, 500, 1e-9),
          "sigma, its tolerance, the hold, the step count and the reset tolerance are the pinned "
          "literals")
    # the matrix, cell by cell, against BOTH the frozen source and the architect's contract
    want = [("A", "deterministic", 123, 1.956870983805102),
            ("B", "deterministic", 123, 1.756870983805102),
            ("C", "deterministic", 123, 2.156870983805102),
            ("D", "stochastic_held", 123, 1.956870983805102),
            ("E", "stochastic_held", 124, 1.956870983805102),
            ("F", "stochastic_held", 125, 1.956870983805102)]
    got = [(c["id"], c["mode"], c["seed"], c["offset_s"]) for c in J12.MATRIX]
    check(got == want, f"the matrix is exactly A-F as contracted ({got})")
    check([(c["id"], c["mode"], c["seed"], c["offset_s"]) for c in J9R1.MATRIX] == want,
          "and it is value-identical to the frozen J9R1 matrix")
    check(len(J12.MATRIX) == 6 and prereg["matrix"]["cells_count"] == 6
          and prereg["matrix"]["steps_per_cell"] == 500
          and prereg["matrix"]["behavioural_fail_fast"] is False,
          "six cells, 500 steps each, no behavioural fail-fast")
    # a drifted J9R1 constant must STOP this stage
    real_gate = J9R1.J9_COMMON_GATE
    try:
        J9R1.J9_COMMON_GATE = {**real_gate, "valid_cycles_min": 3}
        expect(J12.verify_scientific_equivalence, J12.J12Error,
               "a drifted J9R1 threshold stops this stage")
    finally:
        J9R1.J9_COMMON_GATE = real_gate
    check(J12.verify_scientific_equivalence()["thresholds_invented_here"] == 0, "restored")

    # ---------------------------------------------------------------- penetration ---------------
    contract = PC.load_contract()
    check(contract["sha256"] == J12.PIN_CONTRACT, "the penetration contract is pinned")
    for value, band, binding in ((0.0199, "within_soft", True),
                                 (0.020, "within_soft", True),
                                 (0.0201, "above_soft_below_july_legacy", True),
                                 (0.025, "july_legacy_breach_within_hard", True),
                                 (0.028, "july_legacy_breach_within_hard", True),
                                 (0.0281, "above_hard", False)):
        ev = PC.classify(value, contract)
        check(ev["band"] == band and ev["binding_pass"] is binding,
              f"penetration {value}: band {band}, binding_pass {binding}")
    check(PC.classify(0.028, contract)["binding_pass"] is True,
          "EXACTLY 0.028 PASSES - the bound is inclusive")
    check(PC.classify(0.020, contract)["flags"]["above_soft_diagnostic"] is False,
          "and exactly 0.020 is NOT above the soft bar")
    # AST, not substring: the three values appear in PROSE all over the module (docstrings and
    # receipt explanations), which is what they are for. What must not exist is a NUMERIC
    # literal, because that would be a threshold the runner owns instead of the contract.
    numeric = [n for n in ast.walk(tree) if isinstance(n, ast.Constant)
               and isinstance(n.value, float) and n.value in (0.020, 0.025, 0.028)]
    check(not numeric,
          f"AST: no penetration threshold exists as a NUMERIC literal in the runner "
          f"(offenders at lines {[n.lineno for n in numeric]}); the contract owns all three")
    pen_fn = next(n for n in ast.walk(tree)
                  if isinstance(n, ast.FunctionDef) and n.name == "penetration_report")
    calls = {n.func.attr for n in ast.walk(pen_fn) if isinstance(n, ast.Call)
             and isinstance(n.func, ast.Attribute)}
    check("evaluate_series" in calls,
          "and the report delegates to the contract's evaluate_series")

    # ---------------------------------------------------------------- WAIT_HS is DIAGNOSTIC ----
    check(J12.WAIT_HS_IS_BINDING is False and prereg["wait_hs"]["binding"] is False,
          "WAIT_HS is DIAGNOSTIC: attaching a threshold would be a NEW gate")
    check(J12.WAIT_HS_FEATURE == "phase_fsm_wait_hs" and WAIT_IDX == 17,
          f"the feature is resolved by name; in the runtime order it sits at {WAIT_IDX}")
    gate_fn = next(n for n in ast.walk(tree)
                   if isinstance(n, ast.FunctionDef) and n.name == "evaluate_cell_gate")
    check("wait_hs" not in ast.dump(gate_fn) and "WAIT_HS" not in ast.dump(gate_fn),
          "STATIC: the binding gate never mentions WAIT_HS")
    check("valid_cycle_count" not in J12.DIAGNOSTIC_NOT_BINDING,
          "valid_cycle_count is BINDING, so it is not listed among the diagnostics")
    check("valid_cycle_count" in J9R1.DIAGNOSTIC_NOT_BINDING,
          "MEASURED: J9R1 did list it; that self-contradiction is corrected here")

    # ---------------------------------------------------------------- no stale J9/J8 labels ----
    # Every label a reader could mistake for "this stage is about J9 or J8" must be gone. The
    # J8/J9 references that REMAIN are historical evidence, and each is checked to be so.
    check("six-file leaf" not in src and "EIGHT-file J11 leaf" in src,
          "verify_actor's docstring says EIGHT-file J11 leaf, not six")
    check("J9 lock" not in src and src.count("J12 lock") >= 2,
          "both lock messages say J12 lock: the preflight blocker and the commit refusal")
    check("j9_cell_" not in src and "j12_cell_<ID>_trace.json" in src,
          "the preflight's would_write names j12_cell_<ID>_* artefacts")
    check("the J8 manifest holds" not in src and "the J11 manifest holds" in src,
          "the feature-count error names the J11 manifest")
    check("the J8 actor changed" not in src and "the J11 actor changed" in src,
          "the post-matrix drift error names the J11 actor")
    check("chained J9Error" not in src and "chained J12Error" in src,
          "the close-failure comment names J12Error, the exception actually raised")
    ap = next(n for n in ast.walk(tree) if isinstance(n, ast.Call)
              and isinstance(n.func, ast.Attribute) and n.func.attr == "ArgumentParser")
    desc = next(k.value.value for k in ap.keywords if k.arg == "description")
    check("J12" in desc and "J9" not in desc,
          f"the argparse description names J12, not J9 ({desc!r})")
    # and the J8/J9 references that survive are HISTORICAL, each one checked
    for historical in ("NOT J8, NOT J4, NOT a July checkpoint",
                       "the J9 originals, pinned as IMMUTABLE EVIDENCE",
                       "the original J9 preregistration changed",
                       "the J8 defect J11 corrected",
                       "actor_before"):
        check(historical in src, f"the historical reference {historical!r} is retained")
    check(J12.J9_PREREG_SHA and J12.J9_RUNNER_SHA and J12.J9_TEST_SHA
          and J12.J9_AUTHORIZATION_SHA and J12.J9_FAILURE_RECORD_SHA,
          "the five superseded J9 artefacts are still pinned as immutable evidence")

    # ---------------------------------------------------------------- the env-mutation rule ----
    pol = J12.ENV_MUTATION_POLICY
    check(pol["scientific_or_runtime_fields_mutable"] == ["episode_start_offset_s"],
          "episode_start_offset_s is the ONLY scientific or runtime field the matrix varies")
    odesc = pol["output_dir_is_not_a_scientific_field"]
    check(pol["instrumentation_fields_mutable"] == ["output_dir"]
          and "no scientific or runtime meaning" in odesc
          and "enters no gate" in odesc and "instrumentation" in odesc,
          "output_dir is per-cell INSTRUMENTATION - no scientific meaning, no gate - and so is "
          "not an exception to the immutability rule")
    check(any("ONLY one the matrix varies" in f for f in J12.FORBIDDEN_HERE),
          "and FORBIDDEN_HERE states the rule unambiguously")
    check(not any(f == "mutating any env field but episode_start_offset_s"
                  for f in J12.FORBIDDEN_HERE),
          "the old ambiguous phrasing is gone")
    # the guard that actually enforces it
    cec = next(n for n in ast.walk(tree)
               if isinstance(n, ast.FunctionDef) and n.name == "cell_env_config")
    stable = [n for n in ast.walk(cec) if isinstance(n, ast.Assign)
              and any(isinstance(t, ast.Name) and t.id == "stable_keys" for t in n.targets)]
    check(stable, "cell_env_config computes a stable-key set")
    literals = {n.value for n in ast.walk(cec)
                if isinstance(n, ast.Constant) and isinstance(n.value, str)}
    check("episode_start_offset_s" in literals and "output_dir" in literals,
          "and names exactly those two keys as the movable ones")

    # ---------------------------------------------------------------- multistart, truthfully ---
    md = J12.preflight()["multistart_disambiguation"]
    check("PRESENT" in md["training_data"] and "24713" in md["training_data"],
          "the multistart TRAINING data is declared PRESENT: the J11 actor was fitted on it")
    check("J10R1" in md["which_multistart_data"] and "July" in md["which_multistart_data"],
          "and it is the AUGUST J10R1 cells, not the deferred July datasets")
    check("FALSE about the J11 actor" in md["corrected_from_j9r1"],
          "the J9R1 sentence, true of J8 and false of J11, is explicitly corrected")
    check(md["closed_loop_validation"].startswith("cells B and C"),
          "while the closed-loop validation cells stay binding")

    # ---------------------------------------------------------------- preflight is INERT -------
    real = {"mkdir": Path.mkdir, "rename": os.rename, "write_text": Path.write_text,
            "savez": np.savez_compressed}
    banned = []

    def deny(name):
        def f(*a, **k):
            banned.append(name)
            raise AssertionError(f"the preflight called {name}")
        return f
    Path.mkdir, os.rename = deny("mkdir"), deny("rename")
    Path.write_text, np.savez_compressed = deny("write_text"), deny("savez")
    try:
        pre = J12.preflight()
    finally:
        Path.mkdir, os.rename = real["mkdir"], real["rename"]
        Path.write_text, np.savez_compressed = real["write_text"], real["savez"]
    check(not banned, f"MEASURED: the preflight called no write primitive ({banned})")
    check(pre["read_only"] is True and pre["inert"]["leaf_created"] is False
          and pre["inert"]["staging_created"] is False and pre["inert"]["lock_taken"] is False
          and pre["inert"]["outputs_written"] is False
          and pre["inert"]["ppo_updates"] == 0 and pre["inert"]["fit_executed"] is False,
          "and reports itself inert on every axis")
    check(pre["inert"]["heavy_modules_introduced_by_preflight"] == [],
          "MEASURED: it introduced no heavy module")
    check("torch" not in sys.modules and "ray" not in sys.modules,
          "MEASURED: torch and ray are STILL absent from sys.modules after a full preflight")
    check(not J12.PREFLIGHT_SENTINEL.exists()
          and pre["preflight_sentinel"]["created_by_the_preflight"] is False,
          "and the sentinel output root was never created")
    if not leaf_exists_at_start:
        check(pre["verdict"] == "GO" and pre["blockers"] == [],
              "with the leaf absent the preflight is GO")
    else:
        check(pre["verdict"] == "BLOCKED"
              and any("already exists" in b for b in pre["blockers"]),
              "with the leaf present the preflight is BLOCKED - the correct answer AFTER the "
              "single execution, so this assertion does not expire")
    top = {n.names[0].name.split(".")[0] for n in tree.body if isinstance(n, ast.Import)}
    top |= {(n.module or "").split(".")[0] for n in tree.body if isinstance(n, ast.ImportFrom)}
    check(not (top & {"torch", "ray", "opensim", "rollout_eval", "exploration_noise",
                      "env_factory", "gymnasium"}),
          "no heavy import at module scope")
    ps = next(n for n in ast.walk(tree)
              if isinstance(n, ast.FunctionDef) and n.name == "production_stack")
    lazy = {n.names[0].name for n in ast.walk(ps) if isinstance(n, ast.Import)}
    check({"rollout_eval", "exploration_noise"} <= lazy,
          "the production modules are imported lazily, inside production_stack")

    # ---------------------------------------------------------------- destination guards -------
    expect(lambda: J12.validate_stage(None), J12.J12Error, "a missing stage token is refused")
    for bad in ("V26C_J9R1_CLOSED_LOOP", "V26C_J12_CLOSED_LOOP_QUALIFICATION ", ""):
        expect(lambda b=bad: J12.validate_stage(b), J12.J12Error,
               f"the stage token {bad!r} is refused")
    J12.validate_stage(J12.STAGE)
    check(True, "and the exact token is accepted")
    expect(lambda: J12.validate_out(None), J12.J12Error, "--out is required")
    expect(lambda: J12.validate_out("/tmp/elsewhere"), J12.J12Error,
           "a different destination is refused")

    # ---------------------------------------------------------------- the whole matrix ---------
    # STATIC first: prove the runner really does call J3's frozen kernel, so substituting it
    # below is a substitution of the real call site and not of some parallel path.
    rc_fn = next(n for n in ast.walk(tree)
                 if isinstance(n, ast.FunctionDef) and n.name == "run_cell")
    kernel_calls = {n.func.attr for n in ast.walk(rc_fn) if isinstance(n, ast.Call)
                    and isinstance(n.func, ast.Attribute)
                    and isinstance(n.func.value, ast.Name) and n.func.value.id == "J3"}
    check("deterministic_action" in kernel_calls,
          "the deterministic path goes through J3.deterministic_action, the frozen kernel")
    check("telemetry_integrity" in kernel_calls,
          "and telemetry integrity through J3.telemetry_integrity")

    tmp = Path(tempfile.mkdtemp(prefix="v26c_j12_matrix_"))
    saved_root = J12.OUTPUT_ROOT_OVERRIDE
    real_det = J3.deterministic_action
    try:
        J3.deterministic_action = fake_deterministic_action
        J12.OUTPUT_ROOT_OVERRIDE = tmp
        target = tmp / "j12_runs" / "j12_closed_loop_v26c_2026-08-27_r1"
        lock = target.parent / J12.LOCK_NAME
        FakeEnv.wait_hs_steps = 12

        # ---- the silent-load failure is caught
        FakeStack.module_cls = BareRLModule
        FakeEnv.built.clear()
        e = expect(lambda: J12.run_matrix(str(target), J12.STAGE, stack=FakeStack(),
                                          progress=False),
                   J12.J12Error, "a weightless module is REFUSED")
        check("carries NO state" in str(e) and "empty shell" in str(e),
              f"naming the exact silent-failure mode ray produces: {e}")
        check(not target.exists(), "and nothing was committed")
        FakeStack.module_cls = AsymmetricActorCriticTorchRLModule

        AsymmetricActorCriticTorchRLModule.empty_state = True
        try:
            e = expect(lambda: J12.run_matrix(str(target), J12.STAGE, stack=FakeStack(),
                                              progress=False),
                       J12.J12Error, "an empty get_state() is REFUSED even from the right class")
        finally:
            AsymmetricActorCriticTorchRLModule.empty_state = False
        AsymmetricActorCriticTorchRLModule.wrong_weights = True
        try:
            e = expect(lambda: J12.run_matrix(str(target), J12.STAGE, stack=FakeStack(),
                                              progress=False),
                       J12.J12Error, "weights that differ from the pinned module_state are REFUSED")
            check("differ from the pinned module_state" in str(e), f"and say so: {e}")
        finally:
            AsymmetricActorCriticTorchRLModule.wrong_weights = False

        # ---- exactly 19 sim_outputs
        for bad_n in (18, 20):
            FakeEnv.n_output_files = bad_n
            FakeEnv.built.clear()
            e = expect(lambda: J12.run_matrix(str(target), J12.STAGE, stack=FakeStack(),
                                              progress=False),
                       J12.J12Error, f"sim_outputs with {bad_n} files is REFUSED")
            check(f"holds {bad_n} regular files" in str(e) and "expected EXACTLY 19" in str(e),
                  f"and it is the COUNT guard that refused it, not an earlier check: {e}")
            check(not target.exists(), f"and a {bad_n}-file cell commits NOTHING")
        FakeEnv.n_output_files = 19

        # ---- a foreign lock is fail-closed
        target.parent.mkdir(parents=True, exist_ok=True)
        held = json.dumps({"stage": J12.STAGE, "pid": 999999})
        lock.write_text(held, encoding="utf-8")
        e = expect(lambda: J12.run_matrix(str(target), J12.STAGE, stack=FakeStack(),
                                          progress=False),
                   J12.J12Error, "a held lock is fail-closed")
        check(lock.read_text() == held, "MEASURED: the foreign lock was left untouched")
        check(J12.preflight()["verdict"] == "BLOCKED",
              "and the read-only preflight reports it as a blocker")
        lock.unlink()

        # ---- the real matrix
        FakeEnv.built.clear(); FakeEnv.closed.clear(); FakeEnv.saved.clear()
        res = J12.run_matrix(str(target), J12.STAGE, stack=FakeStack(), progress=False)
        rc = json.loads((target / J12.RECEIPT_NAME).read_text())
        check(len(FakeEnv.built) == 6 and len(FakeEnv.closed) == 6,
              "six environments were built and every one was closed")
        check([c["id"] for c in rc["cells"]] == ["A", "B", "C", "D", "E", "F"],
              "the six cells ran in the frozen order A to F")
        check([c["mode"] for c in rc["cells"]]
              == ["deterministic"] * 3 + ["stochastic_held"] * 3,
              "three deterministic then three stochastic_held")
        check([c["seed"] for c in rc["cells"]] == [123, 123, 123, 123, 124, 125],
              "with the contracted seeds")
        check(rc["verdict"] == "PASS" and rc["aggregate_rule"].startswith("PASS iff 6/6"),
              f"the aggregate rule is 6/6 behavioural AND 6/6 telemetry-valid ({rc['verdict']})")

        # ---- per-cell visibility, every field the contract requires
        for c in rc["cells"]:
            t = c["telemetry"]
            for k in ("phase_timeout_stance", "phase_timeout_swing",
                      "morphology_causal_contract_failure", "resync_count",
                      "hs_cancelled_count", "valid_cycle_count", "valid_hs_count",
                      "valid_to_count", "steps", "end_reason"):
                check(k in t, f"cell {c['id']}: {k} is surfaced")
            check(t["steps"] == 500 and t["end_reason"] == "episode_time_limit",
                  f"cell {c['id']}: 500 steps, ended on the time limit")
            check(c["wait_hs"]["rows"] == 12 and c["wait_hs"]["index"] == WAIT_IDX
                  and c["wait_hs"]["binding"] is False,
                  f"cell {c['id']}: WAIT_HS rows counted and marked non-binding")
            k = c["kinematics"]
            check(all(x in k for x in ("knee_min_rad", "knee_max_rad", "ankle_min_rad",
                                       "ankle_max_rad")),
                  f"cell {c['id']}: kinematics surfaced")
            p = c["penetration"]
            check(set(p["counts"]) == {"above_soft_diagnostic", "at_or_above_july_legacy",
                                       "above_hard_binding"},
                  f"cell {c['id']}: all THREE penetration bands reported")
            check(p["sole_binding"] == "above_hard_binding"
                  and p["soft_and_july_are_diagnostic"] is True,
                  f"cell {c['id']}: only the hard band binds")
            check(c["sim_outputs_regular_file_count"] == 19
                  and c["sim_outputs_count_is_exact"] is True,
                  f"cell {c['id']}: exactly 19 sim_outputs")
            check(c["module_load_check"]["class"] == "AsymmetricActorCriticTorchRLModule"
                  and c["module_load_check"]["weights_match_pinned_module_state"] is True,
                  f"cell {c['id']}: the loaded module was verified against the pinned weights")
            check(c["reset_check"]["reset_time_error_s"] == 0.0,
                  f"cell {c['id']}: the reset time is exact")
            check(c["telemetry_integrity"]["pass"] is True, f"cell {c['id']}: telemetry integrity")
            check(c["gate"]["failed"] == [] and c["verdict"] == "PASS",
                  f"cell {c['id']}: gate {c['gate']['failed']}")
            check(not str(c["sim_outputs"]).startswith("/")
                  and ".." not in str(c["sim_outputs"]).split("/"),
                  f"cell {c['id']}: sim_outputs is LEAF-RELATIVE, not a staging path")
        check(J12.STAGING_NAME not in (target / J12.RECEIPT_NAME).read_text(),
              "no staging path appears anywhere in the receipt")

        # ---- the commit is verified
        cv = json.loads((target / J12.COMMIT_VERIFICATION_NAME).read_text())
        check(cv["pass"] is True and not cv["paths_missing"] and not cv["hash_mismatches"]
              and cv["receipt_matches_staging_bytes"] is True,
              f"the post-commit verification passed over {cv['files_checked']} files")
        check(not (target / J12.TECHNICAL_INVALID_NAME).exists(),
              "the born-invalid marker is GONE after a passing verification")
        for rel, sha in rc["committed_files_sha256"].items():
            check((target / rel).is_file() and J12._sha_file(target / rel) == sha,
                  f"MEASURED: {rel} resolves and reproduces its hash")
        check(res["commit_verification"]["pass"] is True, "and the summary reports it")

        # ---- tampering is caught
        victim = target / sorted(rc["committed_files_sha256"])[0]
        keep = victim.read_bytes()
        victim.write_bytes(keep + b"tampered")
        check(J12.verify_committed_leaf(target)["pass"] is False,
              "MEASURED: a tampered committed file is caught")
        victim.write_bytes(keep)
        check(J12.verify_committed_leaf(target)["pass"] is True, "and restoring it passes again")
        check(J12.verify_committed_leaf(target, expected_receipt_sha="0" * 64)["pass"] is False,
              "MEASURED: a receipt whose bytes differ from the staged bytes FAILS")
        for bad in ("/etc/passwd", "../outside", "", "a/../../b"):
            expect(lambda b=bad: J12._resolve_inside(target, b), J12.J12Error,
                   f"a recorded path {bad!r} is refused as not leaf-relative")

        # ---- a behavioural FAIL does not stop the matrix
        shutil.rmtree(target)
        FakeEnv.built.clear()
        FakeEnv.bad_cycles = True
        try:
            res2 = J12.run_matrix(str(target), J12.STAGE, stack=FakeStack(), progress=False)
        finally:
            FakeEnv.bad_cycles = False
        rc2 = json.loads((target / J12.RECEIPT_NAME).read_text())
        check(res2["verdict"] == "FAIL", "one valid cycle FAILS the binding gate")
        check(all("valid_cycles" in c["gate"]["failed"] for c in rc2["cells"]),
              "on valid_cycles specifically")
        check(len(FakeEnv.built) == 6,
              "MEASURED: and all six cells still ran - there is no behavioural fail-fast")
        check((target / J12.COMMIT_VERIFICATION_NAME).is_file()
              and json.loads((target / J12.COMMIT_VERIFICATION_NAME).read_text())["pass"] is True,
              "a behavioural FAIL still commits a fully VERIFIED leaf")

        # ---- a technical failure commits NOTHING and preserves concurrent content
        shutil.rmtree(target)
        sentinel = tmp / "j12_runs" / "concurrent.txt"
        sentinel.write_text("other", encoding="utf-8")
        FakeEnv.close_raises = True
        try:
            expect(lambda: J12.run_matrix(str(target), J12.STAGE, stack=FakeStack(),
                                          progress=False),
                   Exception, "a failing env.close is a TECHNICAL failure")
        finally:
            FakeEnv.close_raises = False
        check(not target.exists() and not lock.exists()
              and not (target.parent / J12.STAGING_NAME).exists(),
              "no leaf, no lock, no staging")
        check(sentinel.is_file(), "and the concurrent sentinel SURVIVED")

        # ---- a FAILED verification never leaves an apparently-valid leaf
        FakeEnv.built.clear()
        real_verify = J12.verify_committed_leaf
        J12.verify_committed_leaf = lambda leaf, **k: {"pass": False, "paths_missing": ["x"],
                                                       "hash_mismatches": []}
        try:
            expect(lambda: J12.run_matrix(str(target), J12.STAGE, stack=FakeStack(),
                                          progress=False),
                   J12.J12Error, "a FAILED post-commit verification RAISES")
        finally:
            J12.verify_committed_leaf = real_verify
        check(target.is_dir() and (target / J12.RECEIPT_NAME).is_file(),
              "MEASURED: the leaf is PRESERVED - a corrupted commit is evidence about the commit")
        check((target / J12.TECHNICAL_INVALID_NAME).is_file(),
              "MEASURED: and is marked TECHNICAL_INVALID, so it never reads as valid")
        check(not lock.exists(), "and the lock is still released")
    finally:
        J3.deterministic_action = real_det
        J12.OUTPUT_ROOT_OVERRIDE = saved_root
        FakeEnv.wait_hs_steps = 0
        FakeEnv.n_output_files = 19
        FakeEnv.bad_cycles = False
        FakeEnv.close_raises = False
        FakeStack.module_cls = AsymmetricActorCriticTorchRLModule
        shutil.rmtree(tmp, ignore_errors=True)

    # ---------------------------------------------------------------- static guarantees --------
    cf = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "run_matrix")
    nodes = list(ast.walk(cf))
    rename_line = next(n.lineno for n in nodes if isinstance(n, ast.Call)
                       and isinstance(n.func, ast.Attribute) and n.func.attr == "rename")
    marker_writes = [n.lineno for n in nodes if isinstance(n, ast.Call)
                     and isinstance(n.func, ast.Attribute) and n.func.attr == "write_text"
                     and any(isinstance(d, ast.Name) and d.id == "TECHNICAL_INVALID_NAME"
                             for d in ast.walk(n.func.value))]
    check(marker_writes and min(marker_writes) < rename_line,
          f"STATIC: the invalidity marker is written into STAGING at line {min(marker_writes)}, "
          f"BEFORE the rename at line {rename_line}")
    unlinks = [n.lineno for n in nodes if isinstance(n, ast.Call)
               and isinstance(n.func, ast.Attribute) and n.func.attr == "unlink"
               and isinstance(n.func.value, ast.Name) and n.func.value.id == "marker"]
    check(unlinks and min(unlinks) > rename_line,
          "and removed only AFTER the commit, as the last write of a passing run")
    vc = next(n for n in nodes if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
              and n.func.id == "verify_committed_leaf")
    inner = max((n for n in nodes if isinstance(n, ast.Try)
                 and any(c is vc for b in n.body for c in ast.walk(b))),
                key=lambda n: n.lineno)
    check({h.type.id for h in inner.handlers if isinstance(h.type, ast.Name)} == {"Exception"},
          "STATIC: the verification catches Exception, never BaseException")
    check(src.count("shutil.rmtree") == 1, "exactly ONE recursive removal")
    rm = [n for n in nodes if isinstance(n, ast.Call)
          and isinstance(n.func, ast.Attribute) and n.func.attr == "rmtree"]
    check(len(rm) == 1 and isinstance(rm[0].args[0], ast.Name)
          and rm[0].args[0].id == "staging_created",
          "and its only argument is staging_created")
    loop = next(n for n in nodes if isinstance(n, ast.For)
                and any(isinstance(c, ast.Call) and isinstance(c.func, ast.Name)
                        and c.func.id == "run_cell" for c in ast.walk(n)))
    check(not [n for n in ast.walk(loop) if isinstance(n, (ast.Break, ast.Continue))],
          "STATIC: the matrix loop has no break and no continue - no behavioural fail-fast")
    check("O_CREAT | os.O_EXCL" in src, "the lock uses O_CREAT|O_EXCL")
    for name in ("PPOConfig", "train_ppo", "adapt_actor", "Adam", "backward", "optimizer"):
        check(name not in ids, f"the runner never references {name}")

    # ---------------------------------------------------------------- nothing disturbed --------
    check(not (HERE / "j12_runs").exists() if not leaf_exists_at_start else True,
          "MEASURED: this suite never created the authorised j12_runs")
    check(snapshot(HERE) == before_all,
          "MEASURED: not one byte under the validation root changed across the WHOLE suite")

    print(json.dumps({"selftest": "PASS", "checks": CHECKS}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
