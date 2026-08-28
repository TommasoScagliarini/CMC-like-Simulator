"""Fail-closed tests for the V26C J6 recovery probe.

Offline only: the collector is exercised against the fake stack from the J3 runner suite, the
prefix rule against the real July production function, and every pinned artefact is read but never
written. No environment, no dataset, no fit, no critic, no PPO.
"""
from __future__ import annotations
import ast, builtins, io, json, shutil, sys, tempfile, tokenize
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26c_j6_recovery_probe as J6  # noqa: E402
import v26c_j3_closed_loop as J3  # noqa: E402
import v26c_j1_collect as J1  # noqa: E402
import v26c_penetration_contract as PC  # noqa: E402
from test_v26c_j3_runner import FakeEnv, FakeModule, _FakeTorch  # noqa: E402

FAILCLOSED = (J6.J6Error, J3.J3Error, J1.J1Error, PC.ContractError)
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


NAMES = J6.actor_feature_names()


class ExploringModule(FakeModule):
    """A fake module with BOTH forward paths, so the parity requirement can be exercised."""

    def __init__(self, tm, *, explore_mean_delta=0.0, explore_log_std_delta=0.0, **kw):
        super().__init__(tm, **kw)
        self.explore_mean_delta = float(explore_mean_delta)
        self.explore_log_std_delta = float(explore_log_std_delta)
        self.exploration_calls = 0

    def forward_exploration(self, batch):
        self.exploration_calls += 1
        mean = [m + self.explore_mean_delta for m in self.mean]
        log_std = [v + self.explore_log_std_delta for v in self.log_std]
        logits = np.asarray([list(mean) + list(log_std)], dtype=np.float32)
        return {"action_dist_inputs": self.tm.as_tensor(logits, dtype=self.tm.float32)}


def held_stochastic_like_production(module, obs, action_shape, unit_noise):
    """A faithful stand-in for rollout_eval._held_stochastic_action, on the fake torch shim.

    Same shape as production: forward_exploration -> logits -> mean, std = exp(log_std),
    applied = std * unit, action = mean + applied.
    """
    tm = module.tm
    obs_t = tm.as_tensor(np.asarray(obs), dtype=tm.float32).reshape(1, -1)
    fwd = module.forward_exploration({"obs": obs_t})
    logits = fwd["action_dist_inputs"]
    dim = logits.shape[-1] // 2
    mean = np.asarray(logits[..., :dim].detach().cpu().numpy(), dtype=np.float32).reshape(
        action_shape)
    std = np.exp(np.asarray(logits[..., dim:].detach().cpu().numpy(),
                            dtype=np.float32)).astype(np.float32).reshape(action_shape)
    applied = (std * np.asarray(unit_noise, dtype=np.float32)).astype(np.float32)
    return (mean + applied).astype(np.float32), mean, std, applied


LOGSTD_0005 = float(np.log(0.005))


def main() -> int:
    src = (HERE / "v26c_j6_recovery_probe.py").read_text()
    tree = ast.parse(src)
    ids = {t.string for t in tokenize.generate_tokens(io.StringIO(src).readline)
           if t.type == tokenize.NAME}
    tm = _FakeTorch
    amendment = json.loads(J6.AMENDMENT.read_text())
    pinned_before = {rel: J6._sha_file(HERE / rel)
                     for rel in amendment["pinned_artefacts_sha256"]}

    # ---------------------------------------------------------------- the amendment -------------
    check(amendment["kind"] == "ADDITIVE, IMMUTABLE"
          and amendment["stage_authorised"] == "V26C-J6-RECOVERY-PROBE",
          "the amendment is additive and names the J6 stage")
    check(J6._sha_file(J6.AMENDMENT) == J6.PIN_AMENDMENT, "and is pinned by exact hash")
    check(len(amendment["pinned_artefacts_sha256"]) == 12
          and all(J6._sha_file(HERE / r) == h
                  for r, h in amendment["pinned_artefacts_sha256"].items()),
          "all 12 pinned artefacts verify on disk")
    op = amendment["operational_parent"]
    check(op["module_state_sha256"]
          == "0f182ea9f8939e2b7824e85c12c57343309c444680682b9bce5858dd74f9d130"
          and "14a3630f" in op["not_the_j4_module"],
          "the parent is the J2 module, and the J4 module is named as explicitly refused")
    check("never a parent" in op["j4_and_j5_are_evidence_only"],
          "J4 and J5 are declared evidence only")
    check(amendment["sigma"]["status"] == "HYPOTHESIS UNDER TEST, NOT A SELECTED VALUE"
          and amendment["sigma"]["value"] == 0.005
          and amendment["sigma"]["no_module_copy_required"]["finding"].startswith(
              "the pinned J2 module ALREADY carries sigma"),
          "sigma 0.005 is a hypothesis, and no module copy is required")
    check(amendment["sigma"]["noise"]["seeds"] == [123, 124, 125]
          and amendment["sigma"]["noise"]["process"].startswith("held standard normal, hold = 1"),
          "three seeds and a one-step held noise")
    al = amendment["alignment_and_labels"]
    check(al["nominal_trace_for_alignment_and_self_anchor"]["sha256"] == J6.PIN_NOMINAL_TRACE
          and "J1 teacher trace is NOT the alignment reference"
          in al["nominal_trace_for_alignment_and_self_anchor"]["why"],
          "the J3 trace is the alignment reference and J1 explicitly is not")
    check(al["truncation_rule"]["must_not_be_relaxed"] is True
          and "119, 118 and 119 are DESCRIPTIVE" in al["truncation_rule"]["no_minimum_length_gate"],
          "the truncation rule is binding and July's prefixes are descriptive only")
    check(al["future_dataset_shape"]["multistart_block"].startswith("OMITTED / DEFERRED")
          and al["future_dataset_shape"]["status"] == "SPECIFICATION ONLY - not built by this stage",
          "the multistart block is deferred and the dataset is specification only")
    for item in ("using J4 or J5 as a parent, as weights, as labels or as a dataset",
                 "editing the log-std head or deriving any module", "critic warm-up or PPO",
                 "ex-novo training", "inventing a numeric gate on prefix length"):
        check(item in amendment["forbidden"], f"{item} is forbidden")

    # ---------------------------------------------------------------- parent and sigma ----------
    parent = J6.verify_parent()
    check(parent["module_state_sha256"]
          == "0f182ea9f8939e2b7824e85c12c57343309c444680682b9bce5858dd74f9d130"
          and parent["module_state_sha256"] != J6.J4_MODULE_STATE_SHA,
          "the verified parent is the J2 module, not the J4 one")
    for rel in list(J6.PIN_PARENT):
        saved = J6.PIN_PARENT[rel]
        J6.PIN_PARENT[rel] = "0" * 64
        expect(J6.verify_parent, J6.J6Error, f"a mutated parent artefact {rel} must be refused")
        J6.PIN_PARENT[rel] = saved
    state = J6.load_parent_state()
    sig = J6.verify_sigma(state)
    check(sig["sigma"] == 0.005 and sig["max_abs_deviation"] < 1e-9,
          f"the parent already carries sigma 0.005 (deviation {sig['max_abs_deviation']:.2e})")
    check(sig["module_copy_required"] is False and sig["logstd_head_edited"] is False,
          "so no module copy is made and no log-std head is edited")
    check(sig["mean_actor_bit_exact"] is True and sig["non_logstd_tensors_bit_exact"] is True
          and sig["save_reload_exact"] is True
          and "by identity" in sig["why_trivially_satisfied"],
          "and the bit-exactness obligations hold by identity, not by comparison")
    # the module on disk IS the module used: prove the three obligations directly
    import pickle
    with (J6.PARENT_MODULE_DIR / "module_state.pkl").open("rb") as fh:
        reloaded = {k: np.asarray(v) for k, v in pickle.load(fh).items()}
    check(sorted(reloaded) == sorted(state)
          and all(np.array_equal(reloaded[k], state[k]) for k in state),
          "a save/reload of the parent is bit-identical (it is never rewritten)")
    action_dim = state["pi.1.weight"].shape[0] // 2
    check(all(np.array_equal(state[k], reloaded[k]) for k in state if k != "pi.1.bias")
          and np.array_equal(state["pi.1.weight"][:action_dim],
                             reloaded["pi.1.weight"][:action_dim]),
          "the mean actor and every non-log-std tensor are bit-exact")
    # a parent whose sigma differs must be refused, not edited
    for label, mutate in (("a non-zero log-std weight", lambda s: s.__setitem__(
                              "pi.1.weight", _with(s["pi.1.weight"], (action_dim, 0), 0.1))),
                          ("a different sigma", lambda s: s.__setitem__(
                              "pi.1.bias", _with(s["pi.1.bias"], (action_dim,), -3.0)))):
        bad = {k: np.array(v, copy=True) for k, v in state.items()}
        mutate(bad)
        e = expect(lambda b=bad: J6.verify_sigma(b), J6.J6Error, f"{label} must be refused")
        check("REFUSES to edit" in str(e) or "not zero" in str(e),
              f"{label}: the stage refuses rather than editing the head")
    masked = J6.verify_masked_columns(state)
    check(all(v == list(J3.MASKED_COLUMNS) for v in masked.values())
          and list(J3.MASKED_COLUMNS) == [0, 1] + list(range(25, 35)),
          "the BASE student still masks the clock AND the controller memory")

    # ---------------------------------------------------------------- alignment reference -------
    ali = J6.verify_alignment_reference()
    check(ali["nominal_rows"] == 500
          and ali["pins_sha256"]["j3_trace"] == J6.PIN_NOMINAL_TRACE,
          "the J3 nominal trace is pinned and holds 500 rows")
    check(ali["measured_justification"]["j1_teacher_vs_student_retained"] == 12
          and ali["measured_justification"]["j3_student_vs_student_retained"] == 500,
          "the reference choice is justified by the measured 12 vs 500")
    for attr in ("PIN_NOMINAL_TRACE", "PIN_TEACHER_DATASET", "PIN_AMENDMENT"):
        saved = getattr(J6, attr)
        setattr(J6, attr, "0" * 64)
        expect(lambda: (J6.verify_alignment_reference(), J6.verify_amendment()),
               J6.J6Error, f"a changed {attr} must be refused")
        setattr(J6, attr, saved)

    # ---------------------------------------------------------------- the truncation rule -------
    sys.path.insert(0, str(J6.BASELINE))
    try:
        import target_domain_noise_adaptation as JN
    except Exception as exc:                                     # pragma: no cover
        raise AssertionError("the July module must be importable to prove the transcription; run "
                             f"under /opt/anaconda3/envs/envCMC-rllib/bin/python ({exc})")
    check(list(J6.discrete_feature_indices(NAMES))
          == list(JN._discrete_feature_indices(NAMES)),
          "J6's discrete selector is identical to July's _discrete_feature_indices")
    nominal = json.loads(J6.NOMINAL_TRACE.read_text())
    teacher = json.loads((J6.TEACHER_LEAF / "teacher_trace.json").read_text())
    for label, a, b in (("J3 vs teacher", nominal, teacher),
                        ("teacher vs J3", teacher, nominal),
                        ("J3 vs itself", nominal, nominal)):
        mine, rep = J6.truncate_before_discrete_mismatch(a, b, NAMES)
        theirs, their_rep = JN.truncate_before_discrete_mismatch(a, b, NAMES)
        check(rep["retained_steps"] == their_rep["retained_steps"]
              and rep["first_discrete_mismatch_step"] == their_rep["first_discrete_mismatch_step"],
              f"{label}: J6 agrees with the July function ({rep['retained_steps']} rows)")
    mine, rep = J6.truncate_before_discrete_mismatch(nominal, nominal, NAMES)
    check(rep["retained_steps"] == 500 and rep["first_discrete_mismatch_step"] is None,
          "a trace identical to the nominal one retains all 500 rows")
    mine, rep = J6.truncate_before_discrete_mismatch(nominal, teacher, NAMES)
    check(rep["retained_steps"] == 12 and rep["first_discrete_mismatch_step"] == 13,
          "while the teacher trace still truncates at 12 - the reason J1 is not the reference")
    fn = next(f for f in ast.walk(tree) if isinstance(f, ast.FunctionDef)
              and f.name == "truncate_before_discrete_mismatch")
    check([a.arg for a in fn.args.args] == ["nominal_rows", "recovery_rows", "names"]
          and not fn.args.defaults and not fn.args.kwonlyargs,
          "the rule takes no tolerance, window or slack parameter")

    # ---------------------------------------------------------------- preflight is inert --------
    tripped: list[str] = []
    real_import = builtins.__import__
    banned = {"env_factory", "torch", "ray", "opensim", "rollout_eval", "gymnasium",
              "exploration_noise", "target_domain_imitation"}

    def guarded(name, *a, **k):  # noqa: ANN001
        if name.split(".")[0] in banned:
            tripped.append(name)
            raise AssertionError(f"preflight imported {name}")
        return real_import(name, *a, **k)

    builtins.__import__ = guarded
    try:
        pre = J6.preflight()
    finally:
        builtins.__import__ = real_import
    check(tripped == [], f"PROVEN INERT: the preflight imported no heavy stack ({tripped})")
    check(pre["verdict"] == "GO" and pre["blockers"] == [], "the preflight is GO")
    check(pre["inert"]["environment_constructed"] is False
          and pre["inert"]["dataset_built"] is False and pre["inert"]["fit_executed"] is False,
          "nothing is constructed, no dataset is built, no fit runs")
    check(pre["authorisation"]["execution_requires"]
          == f"--authorized-stage {J6.STAGE} --seed <123|124|125>",
          "execution needs the exact token and one of the three seeds")
    check(sorted(int(k) for k in pre["authorisation"]["planned_leaves"]) == [123, 124, 125],
          "the three planned leaves are declared")
    check(pre["penetration_contract"]["thresholds_m"]
          == {"soft_diagnostic": 0.020, "july_legacy": 0.025, "hard_binding": 0.028}
          and pre["penetration_contract"]["evaluated_by"]
          == "v26c_penetration_contract.evaluate_series",
          "penetration is delegated to the contract helper")
    check(pre["truncation"]["no_prefix_length_gate"] is True
          and pre["truncation"]["july_retained_prefixes_descriptive_only"] == [119, 118, 119],
          "no prefix-length gate; July's numbers are descriptive")
    check(pre["future_dataset_specification_only"]["built_here"] is False
          and pre["future_dataset_specification_only"]["multistart_block"]
          == "OMITTED / DEFERRED",
          "the future dataset is specification only, multistart deferred")
    check(pre["outcome_policy"] == {"deployable": False, "promotion": "NONE",
                                    "next_stage_authorized": False},
          "and the stage promotes nothing")

    # ---------------------------------------------------------------- refusals -----------------
    with tempfile.TemporaryDirectory() as td:
        def stack_for(env, mod, *, with_helper=True, july=None):
            st = J3._Stack(name="fake", operational=False, torch_mod=tm,
                           load_module=lambda p: mod, make_env=lambda cfg: env)
            if with_helper:
                st.held_stochastic_action = held_stochastic_like_production
            if july is not None:
                st.july_truncate = july
            return st
        for stage in (None, "", "V26C-J6", "V26C-J3-CLOSED-LOOP", "V26C-J4-RECOVERY"):
            expect(lambda st=stage: J6.collect(authorized_stage=st, seed=123,
                                               out_dir=Path(td) / "x",
                                               stack=stack_for(FakeEnv(NAMES), ExploringModule(tm, log_std=(LOGSTD_0005,) * 2)),
                                               progress=False),
                   J6.J6Error, f"the token {stage!r} must be refused")
        for bad_seed in (0, 1, 126, 999, -1):
            expect(lambda sd=bad_seed: J6.collect(authorized_stage=J6.STAGE, seed=sd,
                                                  out_dir=Path(td) / "y",
                                                  stack=stack_for(FakeEnv(NAMES), ExploringModule(tm, log_std=(LOGSTD_0005,) * 2)),
                                                  progress=False),
                   J6.J6Error, f"the unauthorised seed {bad_seed} must be refused")
        check(not (Path(td) / "x").exists() and not (Path(td) / "y").exists(),
              "and a refused call creates nothing")
        expect(lambda: J6.main(["--authorized-stage", J6.STAGE]), J6.J6Error,
               "the CLI refuses to collect without --seed")
        expect(lambda: J6.main(["--audit"]), J6.J6Error,
               "--audit requires an explicit --out")

        # ------------------------------------------------------------ the happy path ------------
        env = FakeEnv(NAMES)
        mod = ExploringModule(tm, log_std=(LOGSTD_0005,) * 2)
        rec = J6.collect(authorized_stage=J6.STAGE, seed=123, out_dir=Path(td) / "seed123",
                         stack=stack_for(env, mod), progress=False)
        leaf = Path(td) / "seed123"
        check(rec["verdict"] == "PASS" and rec["seed"] == 123,
              f"a conforming fake collection completes: {rec['verdict']}")
        check(rec["summary"]["steps"] == 500
              and rec["summary"]["end_reason"] == "episode_time_limit",
              "500 steps ending on episode_time_limit")
        check(env.seed_used == 123 and 123 in _FakeTorch.seeded,
              "the seed reaches numpy, torch and env.reset")
        check(sorted(q.name for q in leaf.iterdir() if q.is_file())
              == ["j6_penetration.npz", "j6_trace.json", J6.RECEIPT_NAME],
              "the leaf holds the trace, the penetration series and the receipt")
        trace = json.loads((leaf / "j6_trace.json").read_text())
        check(len(trace) == 500 and len(trace[0]["actor_observation_vector_before"]) == 35,
              "the trace holds 500 rows of 35D observations")
        # the action really is mean + sigma * held noise, and the RAW action is stepped
        row = trace[0]
        # the runner adds in float32, so the identity is asserted in float32, not float64
        m = np.asarray(row["policy_mean"], dtype=np.float32)
        n = np.asarray(row["action_noise"], dtype=np.float32)
        r = np.asarray(row["raw_action"], dtype=np.float32)
        check(np.array_equal(r, (m + n).astype(np.float32)),
              "raw_action == float32(policy_mean + action_noise), bit for bit")
        residual = float(np.abs(np.asarray(row["raw_action"], dtype=np.float64)
                                - (np.asarray(row["policy_mean"], dtype=np.float64)
                                   + np.asarray(row["action_noise"], dtype=np.float64))).max())
        check(residual <= np.spacing(np.float32(np.abs(m).max())),
              f"and in float64 the residual {residual:.3e} is at most one float32 ulp of the "
              "operand, which is exactly the rounding of the float32 add")
        check(row["stepped_with"] == "raw_action"
              and np.allclose(np.asarray(env.received[0]), r),
              "and the environment is stepped with that RAW action")
        check(row["action_selection_path"] == "stochastic_held(sigma=0.005, hold=1)",
              "the selection path records sigma and the hold")
        noises = np.asarray([q["action_noise"] for q in trace])
        check(float(np.abs(noises).max()) > 0.0,
              "the exploration noise is actually non-zero")
        realized = np.sqrt((noises ** 2).mean(axis=0))
        check(all(abs(v - 0.005) < 0.005 for v in realized),
              f"and its realised RMS is of order sigma ({realized.tolist()})")
        check(rec["exploration"]["sigma"] == 0.005
              and rec["exploration"]["hold_steps"] == 1
              and rec["exploration"]["logstd_head_edited"] is False,
              "the receipt records sigma, the hold and that no head was edited")
        # penetration comes from the contract helper
        pen = rec["penetration_under_contract"]
        check(set(pen["counts"]) == {"above_soft_diagnostic", "at_or_above_july_legacy",
                                     "above_hard_binding"},
              "penetration is reported in the three contract bands")
        check(pen["thresholds_m"] == {"soft_diagnostic": 0.020, "july_legacy": 0.025,
                                      "hard_binding": 0.028},
              "with the contract thresholds")
        # the prefix is measured against J3, and no gate is applied to it
        pref = rec["prefix_against_j3_nominal"]
        check("retained_steps" in pref and "first_discrete_mismatch_step" in pref,
              "the receipt records the aligned prefix against the J3 nominal trace")
        check(rec["no_prefix_length_gate"] is True
              and rec["july_retained_prefixes_descriptive_only"] == [119, 118, 119],
              "and applies no length gate")
        check(rec["builds_no_dataset"] is True and rec["fits_nothing"] is True
              and rec["critic_trained"] is False and rec["ppo_updates"] == 0,
              "the receipt states it builds no dataset and fits nothing")
        check(rec["deployable"] is False and rec["promotion"] == "NONE"
              and rec["next_stage_authorized"] is False, "and promotes nothing")
        check(rec["inputs_sha256"]["parent_artefacts"] == J6.PIN_PARENT
              and rec["inputs_sha256"]["nominal_trace"] == J6.PIN_NOMINAL_TRACE,
              "every input is pinned by hash in the receipt")
        check(J6.RECEIPT_NAME not in rec["outputs_sha256"]
              and rec["outputs_sha256_excludes"]["file"] == J6.RECEIPT_NAME,
              "the receipt does not hash itself")
        check(env.closed is True, "the environment is closed")

        # no-clobber and production-leaf enforcement
        expect(lambda: J6.collect(authorized_stage=J6.STAGE, seed=123, out_dir=leaf,
                                  stack=stack_for(FakeEnv(NAMES), ExploringModule(tm, log_std=(LOGSTD_0005,) * 2)),
                                  progress=False),
               J6.J6Error, "no-clobber: an existing leaf is never overwritten")
        # a fake env whose std is not sigma must abort
        expect(lambda: J6.collect(authorized_stage=J6.STAGE, seed=124,
                                  out_dir=Path(td) / "badstd",
                                  stack=stack_for(FakeEnv(NAMES),
                                                  ExploringModule(tm, log_std=(-2.0, -2.0))),
                                  progress=False),
               FAILCLOSED, "a policy std that is not sigma must abort")
        bad_leaf = Path(td) / "badstd"
        # the amendment declares "artefacts preserved and quarantined" on failure, so a leaf that
        # was already created stays - but it must carry NO trace and NO receipt, so a partial
        # collection can never be mistaken for a result
        check(not (bad_leaf / "j6_trace.json").exists()
              and not (bad_leaf / J6.RECEIPT_NAME).exists()
              and not (bad_leaf / "j6_penetration.npz").exists(),
              "a failed collection writes no trace, no penetration series and no receipt")

        # ------------------------------------------------------------ the offline audit ---------
        audit = J6.audit_prefixes([leaf])
        check(audit["kind"].startswith("OFFLINE PREFIX AUDIT")
              and audit["builds_no_dataset"] is True and audit["fits_nothing"] is True,
              "the audit is offline and builds nothing")
        entry = audit["traces"][leaf.name]
        check(entry["rows"] == 500 and "retained_steps" in entry["prefix"],
              "it reports the retained prefix per trace")
        check(entry["penetration"]["thresholds_m"]["hard_binding"] == 0.028,
              "and the penetration bands from the contract")
        check(audit["no_prefix_length_gate"].startswith("the retained lengths are REPORTED"),
              "with no threshold applied")
        out = Path(td) / "audit.json"
        check(J6.main(["--audit", "--out", str(out)]) == 0, "the CLI writes an audit")
        expect(lambda: J6.main(["--audit", "--out", str(out)]), J6.J6Error,
               "no-clobber on the audit output")

    # ------------------------------------------- production action parity -----------------------
    with tempfile.TemporaryDirectory() as td2:
        def stack2(env, mod, **kw):
            st = J3._Stack(name="fake", operational=False, torch_mod=tm,
                           load_module=lambda p: mod, make_env=lambda cfg: env)
            st.held_stochastic_action = kw.get("helper", held_stochastic_like_production)
            return st
        # the exploration path is REALLY used
        env_p = FakeEnv(NAMES)
        mod_p = ExploringModule(tm, log_std=(LOGSTD_0005,) * 2)
        rp = J6.collect(authorized_stage=J6.STAGE, seed=124, out_dir=Path(td2) / "parity",
                        stack=stack2(env_p, mod_p), progress=False)
        check(mod_p.exploration_calls == 500,
              f"forward_exploration is called on every step ({mod_p.exploration_calls})")
        check(len(mod_p.seen_obs) == 500,
              "and forward_inference is called on every step too, for the parity check")
        ex = rp["exploration"]
        check(ex["action_path"].startswith("rollout_eval._held_stochastic_action")
              and ex["effective_std_used"] is True
              and ex["inference_exploration_parity_steps"] == 500,
              "the receipt records the production action path and 500 parity steps")
        tr = json.loads((Path(td2) / "parity" / "j6_trace.json").read_text())
        row0 = tr[0]
        st_row = np.asarray(row0["policy_std_diagnostic"], dtype=np.float32)
        check(np.allclose(st_row, 0.005, atol=1e-9),
              "the std written to the trace is the policy's EFFECTIVE std")
        check(np.array_equal(np.asarray(row0["action_noise"], dtype=np.float32),
                             (st_row * np.asarray(row0["action_noise"], dtype=np.float32)
                              / st_row).astype(np.float32)),
              "and the noise is the one the helper applied")

        # THE PARITY FALSIFIER: if forward_exploration diverged from forward_inference, abort.
        for label, kwargs in (("a shifted exploration mean", {"explore_mean_delta": 1e-3}),
                              ("a different exploration std",
                               {"explore_log_std_delta": 0.5})):
            bad_mod = ExploringModule(tm, log_std=(LOGSTD_0005,) * 2, **kwargs)
            e = expect(lambda m=bad_mod: J6.collect(
                           authorized_stage=J6.STAGE, seed=125,
                           out_dir=Path(td2) / f"diverge_{abs(hash(label)) % 9999}",
                           stack=stack2(FakeEnv(NAMES), m), progress=False),
                       J6.J6Error, f"{label} must abort the probe")
            check("forward_exploration and forward_inference disagree" in str(e)
                  or "disagree on the std" in str(e),
                  f"{label}: the failure names the divergence")
        # a stack without the production helper is refused outright
        no_helper = J3._Stack(name="fake", operational=False, torch_mod=tm,
                              load_module=lambda p: ExploringModule(tm,
                                                                    log_std=(LOGSTD_0005,) * 2),
                              make_env=lambda cfg: FakeEnv(NAMES))
        e = expect(lambda: J6.collect(authorized_stage=J6.STAGE, seed=123,
                                      out_dir=Path(td2) / "nohelper", stack=no_helper,
                                      progress=False),
                   J6.J6Error, "a stack without held_stochastic_action must be refused")
        check("refuses to improvise" in str(e), "because the stage never improvises its own action")
        check("_held_stochastic_action" in src and "forward_exploration" in src,
              "and the runner names the production helper explicitly")

    # ------------------------------------------- the full binding gate --------------------------
    check(set(J6.J6_GATE) == {"steps_required", "end_reason", "valid_cycles_min",
                              "phase_timeout_stance_max", "phase_timeout_swing_max",
                              "morphology_causal_contract_failure_max", "hs_cancelled_count_max",
                              "resync_count_max", "max_penetration_m_max_binding"},
          "the gate carries every J1/J3 criterion plus the binding penetration band")
    for k in ("steps_required", "end_reason", "valid_cycles_min", "phase_timeout_stance_max",
              "phase_timeout_swing_max", "morphology_causal_contract_failure_max",
              "hs_cancelled_count_max", "resync_count_max"):
        check(J6.J6_GATE[k] == J1.J1_GATE[k], f"{k} is inherited verbatim from J1_GATE")
    check(J6.J6_GATE["max_penetration_m_max_binding"] == 0.028
          and "max_penetration_m_max" not in J6.J6_GATE,
          "and the 0.020 criterion is NOT a binding gate here")
    contract = PC.load_contract()
    ok_summary = {"steps": 500, "end_reason": "episode_time_limit", "valid_cycle_count": 2,
                  "valid_hs_count": 3, "valid_to_count": 3, "phase_timeout_stance": 0,
                  "phase_timeout_swing": 0, "morphology_causal_contract_failure": 0,
                  "hs_cancelled_count": 0, "resync_count": 0, "action_clipped_steps": 0}
    shallow = PC.evaluate_series([0.0261] * 10, contract)
    g = J6.evaluate_gate(ok_summary, shallow)
    check(g["pass"] is True and g["failed"] == [],
          "an intact episode at 26.1 mm passes: > 20 and >= 25 are diagnostic only")
    check(g["penetration_bands"]["soft_diagnostic_above"] is True
          and g["penetration_bands"]["july_legacy_breach"] is True
          and g["penetration_bands"]["hard_binding_pass"] is True,
          "while both diagnostic flags are still reported")
    deep = PC.evaluate_series([0.0281], contract)
    check(J6.evaluate_gate(ok_summary, deep)["failed"] == ["penetration_hard_binding"],
          "28.1 mm fails on the binding band alone")
    check(J6.evaluate_gate(ok_summary, PC.evaluate_series([0.028], contract))["pass"] is True,
          "and exactly 28.0 mm passes")
    # every non-penetration criterion, individually, with penetration well inside the band
    for field, bad, expected in (("steps", 499, "steps"),
                                 ("end_reason", "phase_timeout:stance", "end_reason"),
                                 ("valid_cycle_count", 1, "valid_cycles"),
                                 ("phase_timeout_stance", 1, "phase_timeout_stance"),
                                 ("phase_timeout_swing", 1, "phase_timeout_swing"),
                                 ("morphology_causal_contract_failure", 1,
                                  "morphology_causal_contract_failure"),
                                 ("hs_cancelled_count", 1, "hs_cancelled_count"),
                                 ("resync_count", 2, "resync_count")):
        r = J6.evaluate_gate({**ok_summary, field: bad}, shallow)
        check(r["pass"] is False and r["failed"] == [expected],
              f"a violating {field} fails the gate on {expected}, at 26.1 mm penetration")
    check(J6.evaluate_gate({**ok_summary, "resync_count": 1}, shallow)["pass"] is True,
          "while resync_count == 1 is still within the bound")
    check(J6.evaluate_gate({**ok_summary, "action_clipped_steps": 500}, shallow)["pass"] is True,
          "and clipping remains diagnostic")

    # the same, end to end through collect(): early termination and each counter
    with tempfile.TemporaryDirectory() as td3:
        def stack3(env, mod):
            st = J3._Stack(name="fake", operational=False, torch_mod=tm,
                           load_module=lambda p: mod, make_env=lambda cfg: env)
            st.held_stochastic_action = held_stochastic_like_production
            return st
        cases = {
            "early_stop": dict(stop_at=120, end_reason="phase_timeout:stance", timeout_at=120),
            "one_cycle": dict(counts=lambda s: (min(3, s // 150), min(3, s // 150), 0)),
            "cancelled": dict(cancelled=1),
            "resync": dict(resync=2),
            "swing_timeout": dict(timeout_at=300, timeout_side=2.0),
        }
        for name, kw in cases.items():
            mod_c = ExploringModule(tm, log_std=(LOGSTD_0005,) * 2)
            rc = J6.collect(authorized_stage=J6.STAGE, seed=123,
                            out_dir=Path(td3) / name,
                            stack=stack3(FakeEnv(NAMES, **kw), mod_c), progress=False)
            check(rc["verdict"] == "FAIL" and rc["gate_failed"],
                  f"{name}: the collection is FAIL with failures {rc['gate_failed']}")
            check(rc["penetration_under_contract"]["binding_pass"] is True,
                  f"{name}: and it fails DESPITE penetration being inside the binding band")
            check(rc["quarantine"]["applies"] is True,
                  f"{name}: the leaf is quarantined")
        for name, expected in (("early_stop", {"steps", "end_reason", "phase_timeout_stance"}),
                               ("one_cycle", {"valid_cycles"}),
                               ("cancelled", {"hs_cancelled_count"}),
                               ("resync", {"resync_count"}),
                               ("swing_timeout", {"phase_timeout_swing"})):
            rec_c = json.loads((Path(td3) / name / J6.RECEIPT_NAME).read_text())
            check(set(rec_c["gate_failed"]) >= expected,
                  f"{name}: the receipt names {sorted(expected)}")
            check(rec_c["gate"]["measures"]["max_penetration_m"] <= 0.028
                  and rec_c["gate"]["source"].startswith("v26c_j1_collect.J1_GATE"),
                  f"{name}: gate, measures and source are all recorded")

    # ------------------------------------------- truncation parity ------------------------------
    check("july_truncate" in src and "july_function_parity" in src,
          "the runner proves its transcription against the July function at runtime")
    check("target_domain_noise_adaptation.truncate_before_discrete_mismatch" in src,
          "naming that function explicitly")
    with tempfile.TemporaryDirectory() as td4:
        def bad_truncate(nominal, rows, names):
            return list(rows[:7]), {"original_steps": len(rows), "retained_steps": 7,
                                    "first_discrete_mismatch_step": 8}
        st = J3._Stack(name="fake", operational=False, torch_mod=tm,
                       load_module=lambda p: ExploringModule(tm, log_std=(LOGSTD_0005,) * 2),
                       make_env=lambda cfg: FakeEnv(NAMES))
        st.held_stochastic_action = held_stochastic_like_production
        st.july_truncate = bad_truncate
        e = expect(lambda: J6.collect(authorized_stage=J6.STAGE, seed=123,
                                      out_dir=Path(td4) / "trunc", stack=st, progress=False),
                   J6.J6Error, "a transcription disagreeing with the July function must abort")
        check("disagrees with" in str(e), "and the failure says so")

    # ------------------------------------------- rev1 / rev2 ------------------------------------
    check(J6.AMENDMENT.name.endswith("_rev2.json")
          and J6._sha_file(J6.AMENDMENT) == J6.PIN_AMENDMENT,
          "the runner is pinned to the rev2 amendment")
    check(J6.AMENDMENT_REV1.is_file()
          and J6._sha_file(J6.AMENDMENT_REV1) == J6.PIN_AMENDMENT_REV1
          == "e063946ac323ace9120a1de6214e69f57b1cdf336bf52faaabb0930d0a1a3fa6",
          "and rev1 is preserved byte-identical as a forensic draft")
    rev2 = json.loads(J6.AMENDMENT.read_text())
    check(rev2["revision"] == 2
          and rev2["supersedes"]["sha256"] == J6.PIN_AMENDMENT_REV1
          and "PRESERVED AS A FORENSIC DRAFT" in rev2["supersedes"]["status"],
          "rev2 declares what it supersedes and that rev1 is preserved")
    check(len(rev2["supersedes"]["what_changed"]) == 3,
          "and lists the three changes")
    check(rev2["action_semantics"]["helper"] == "rollout_eval._held_stochastic_action"
          and "EFFECTIVE std" in rev2["action_semantics"]["std"],
          "rev2 records the production action semantics")
    check(rev2["binding_gate"]["criteria"]["max_penetration_m_max_binding"] == 0.028
          and rev2["binding_gate"]["criteria"]["steps_required"] == 500
          and "non-zero on ANY binding failure" in rev2["binding_gate"]["cli"],
          "and the full binding gate, with a non-zero CLI on any failure")
    saved_pin = J6.PIN_AMENDMENT_REV1
    J6.PIN_AMENDMENT_REV1 = "0" * 64
    e = expect(J6.verify_amendment, J6.J6Error, "an edited rev1 must abort")
    check("was edited" in str(e), "because rev1 is preserved evidence")
    J6.PIN_AMENDMENT_REV1 = saved_pin

    # ---------------------------------------------------------------- static guarantees ---------
    heavy = {"torch", "ray", "env_factory", "rollout_eval", "opensim", "exploration_noise"}
    top = {n.names[0].name.split(".")[0] for n in tree.body if isinstance(n, ast.Import)}
    top |= {(n.module or "").split(".")[0] for n in tree.body if isinstance(n, ast.ImportFrom)}
    check(not (top & heavy), f"no heavy import at file scope ({sorted(top & heavy)})")
    for name in ("adapt_actor", "aggregate_dagger_traces", "build_markov_recovery_dataset",
                 "PPOConfig", "train_ppo", "optimizer", "Adam", "backward", "critic"):
        check(name not in ids, f"the module never references {name}")
    check("v26c_j4_recovery" not in src and "v26c_j5_revalidation" not in src,
          "J6 imports neither the J4 nor the J5 tooling")
    check("PC.evaluate_series" in src and "v26c_penetration_contract" in src,
          "penetration is evaluated only through the contract helper")
    check(not J6.OUT_ROOT.exists() or not any(J6.OUT_ROOT.iterdir()),
          "MEASURED: no J6 run output exists - no physical collection has been run")
    after = {rel: J6._sha_file(HERE / rel) for rel in amendment["pinned_artefacts_sha256"]}
    check(after == pinned_before == amendment["pinned_artefacts_sha256"],
          "MEASURED: every pinned artefact is byte-identical after the whole suite")

    print(json.dumps({"selftest": "PASS", "checks": CHECKS}, indent=1))
    return 0


def _with(arr, index, value):
    out = np.array(arr, copy=True)
    out[index] = value
    return out


if __name__ == "__main__":
    raise SystemExit(main())
