"""Tests for the V26C J1 collector. No environment is ever constructed, reset or stepped."""
from __future__ import annotations
import builtins, copy, io, json, sys, tokenize
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26c_j1_collect as C  # noqa: E402

CHECKS = 0
def check(c, w):
    global CHECKS; assert c, w; CHECKS += 1
def expect(fn, exc, w):
    global CHECKS
    try: fn()
    except exc as e: CHECKS += 1; return e
    raise AssertionError(f"expected {exc.__name__}: {w}")


def main() -> int:
    src = (HERE / "v26c_j1_collect.py").read_text()
    ids = {t.string for t in tokenize.generate_tokens(io.StringIO(src).readline)
           if t.type == tokenize.NAME}

    # ---------------------------------------------------------------- pinned config -------------
    check(C.PIN_RUNTIME_CONFIG_SHA
          == "a870cc38a77d853bbd5fba86b51cfcc3ef20a33a5823f4a42f1b968ba4a537db",
          "the runtime config pin is the required one")
    cfg = C.load_pinned_config()
    check(C._sha_file(C.PINNED_CONFIG) == C.PIN_RUNTIME_CONFIG_SHA,
          "the config on disk matches its pin")
    orig_pin = C.PIN_RUNTIME_CONFIG_SHA
    C.PIN_RUNTIME_CONFIG_SHA = "0" * 64
    expect(C.load_pinned_config, C.J1Error, "a changed config must be refused")
    C.PIN_RUNTIME_CONFIG_SHA = orig_pin

    # ---------------------------------------------------------------- parent --------------------
    par = C.verify_parent()
    check(par["module_state_sha256"] == C.PIN_PARENT_STATE_SHA
          == "0ba56eb703a238de41afd10d079c1cd59903ba20189e24d43b5c3a363cde15bd",
          "the exclusive operative parent is the V26 August imitation checkpoint")
    check("MLP_imitation_native_v26_08-20-2026" in par["path"], "and its path names that run")
    for marker in C.FORBIDDEN_PARENT_MARKERS:
        check(marker not in str(C.PARENT_DIR),
              f"the parent path carries no July marker {marker!r}")
    check("methodological evidence only" in par["july_status"],
          "July artefacts are declared evidence only, never operative inputs")

    # ---------------------------------------------------------------- the historical builder ----
    check("build_target_env_config" not in ids,
          "the collector never references the incomplete historical env builder")
    check("prescribed_teacher_action" in ids,
          "but it DOES reuse prescribed_teacher_action, the required label source")
    nh = C.verify_no_historical_builder()
    check("build_target_env_config" in nh["forbidden_call_sites_absent"],
          "the check records what it forbids")
    check("prescribed_teacher_action" in nh["allowed_read_only_reuse"],
          "and what it requires")

    # ---------------------------------------------------------------- full env_config -----------
    env = C.build_full_env_config(cfg)
    contract = C.verify_env_config(env, cfg)
    check(contract["status"] == "PASS", "the assembled env_config satisfies the v3 contract")
    check(contract["reward_block_keys"] == 125,
          "the FULL reward block is forwarded, all 125 keys, morphology and corridor included")
    check(len(C.V3_ENV_TO_CONFIG) == 12, "twelve required v3 env fields")
    for k in ("binary_phase_detector_profile_file", "phase_fsm_input_mode",
              "phase_sensor_on_threshold_n", "phase_sensor_off_threshold_n",
              "phase_sensor_dwell_s", "detector_sample_dt_s", "event_contract_id",
              "binary_phase_fsm_mode", "binary_phase_invalid_event_policy",
              "binary_phase_actor_fsm_version", "binary_phase_debounce_s",
              "binary_phase_event_contract_id"):
        check(k in env, f"{k} is present in the assembled env_config")
    check(env["event_contract_id"] == "legacy_events_v1", "event_contract_id = legacy_events_v1")
    check(env["binary_phase_event_contract_id"] == "binary_point_v25+heel_qualified_fsm_v2",
          "binary_phase_event_contract_id = binary_point_v25+heel_qualified_fsm_v2")
    check(env["binary_phase_actor_fsm_version"] == "v3", "the FSM version is v3")
    check(env["event_contract_id"] != env["binary_phase_event_contract_id"],
          "the two contract ids are distinct values")
    check(C.CONFIG_TO_ENV_RENAME
          == {"binary_phase_detector_profile": "binary_phase_detector_profile_file"},
          "exactly one config -> env rename")
    check(env["binary_phase_detector_profile_file"]
          .endswith("selected_candidate_profile.json")
          and Path(env["binary_phase_detector_profile_file"]).is_file(),
          "the detector profile is resolved to an existing file")
    check("binary_phase_detector_profile" not in env,
          "and the config-side name does not leak into the env")
    check(env["grf_penetration_penalty_threshold_m"] == 0.020
          and env["grf_penetration_termination_m"] == 0.028,
          "soft guard 0.020 m and hard termination 0.028 m, both present")
    check(env["reward"]["morphology_causal_event_contract_id"]
          == "binary_point_v25+heel_qualified_fsm_v2",
          "the morphology causal contract id travels in the reward block")

    # semantic equivalence with the production builder's key set
    ro = (C.BASELINE / "rollout_eval.py").read_text()
    for k in C.V3_ENV_TO_CONFIG:
        check(f'"{k}"' in ro, f"{k} is a real env key used by rollout_eval")

    # ---------------------------------------------------------------- NEGATIVE MUTATION ---------
    def mutate(value):
        """A DIFFERENT non-empty, non-null value of a plausible kind."""
        if isinstance(value, bool):
            return not value
        if isinstance(value, (int, float)):
            return float(value) + 1.0
        if isinstance(value, str):
            return value + "_MUTATED"
        if isinstance(value, list):
            return value + ["_MUTATED"]
        return "_MUTATED"

    # EVERY verified field: deletion and a different non-empty value must both fail
    exp_all = C.canonical_expectations(cfg)
    check(len(exp_all) == 44, "forty-four env fields are verified value-for-value")
    for key in exp_all:
        m = copy.deepcopy(env); m.pop(key)
        e = expect(lambda mm=m: C.verify_env_config(mm, cfg), C.J1Error,
                   f"omitting {key} must fail closed")
        check(key in str(e), f"and the failure names {key}")
        m = copy.deepcopy(env); m[key] = mutate(env[key])
        e = expect(lambda mm=m: C.verify_env_config(mm, cfg), C.J1Error,
                   f"a DIFFERENT non-empty {key} must fail closed, not merely an empty one")
        check(key in str(e), f"and that failure also names {key}")
    # the twelve v3 fields specifically: a wrong but non-empty value must never pass
    for key in C.V3_ENV_TO_CONFIG:
        m = copy.deepcopy(env); m[key] = "___NON_EMPTY_BUT_WRONG___"
        expect(lambda mm=m: C.verify_env_config(mm, cfg), C.J1Error,
               f"a non-empty but wrong {key} must fail: presence is not enough")
    # EVERY reward key, not a sample of seven
    check(len(env["reward"]) == 125, "the pinned reward block has 125 keys")
    for key in sorted(env["reward"]):
        m = copy.deepcopy(env); m["reward"].pop(key)
        expect(lambda mm=m: C.verify_env_config(mm, cfg), C.J1Error,
               f"omitting reward.{key} must fail closed")
        m = copy.deepcopy(env); m["reward"][key] = mutate(env["reward"][key])
        expect(lambda mm=m: C.verify_env_config(mm, cfg), C.J1Error,
               f"a mutated reward.{key} must fail closed")
    morph = [k for k in env["reward"] if k.startswith("morphology")]
    check(len(morph) == 17, "all seventeen morphology keys are inside the verified block")
    m = copy.deepcopy(env); m["reward"]["___EXTRA___"] = 1.0
    expect(lambda mm=m: C.verify_env_config(mm, cfg), C.J1Error,
           "an EXTRA reward key must fail too: the key set must match exactly")
    m = copy.deepcopy(env); m.pop("reward")
    expect(lambda mm=m: C.verify_env_config(mm, cfg), C.J1Error,
           "a missing reward block must fail")
    m = copy.deepcopy(env); m["reward"] = {"morphology_profile": env["reward"]["morphology_profile"]}
    expect(lambda mm=m: C.verify_env_config(mm, cfg), C.J1Error,
           "a truncated reward block must fail even though the surviving key is correct")
    # booleans must not satisfy numeric expectations
    m = copy.deepcopy(env); m["policy_knots"] = True
    expect(lambda mm=m: C.verify_env_config(mm, cfg), C.J1Error,
           "True must not pass where 1 is expected")
    check(C.PROTOCOL_FIXED_ENV["rebuild_model_on_reset"] is False
          and env["rebuild_model_on_reset"] is False,
          "rebuild_model_on_reset is False, as in training and in the historical collector")
    # the two contract ids: swap and merge
    m = copy.deepcopy(env)
    m["event_contract_id"], m["binary_phase_event_contract_id"] = (
        env["binary_phase_event_contract_id"], env["event_contract_id"])
    e = expect(lambda mm=m: C.verify_env_config(mm, cfg), C.J1Error,
               "SWAPPING the two contract ids must fail closed")
    check("SWAPPED" in str(e) or "event_contract_id" in str(e),
          "and the failure identifies the contract ids")
    m = copy.deepcopy(env)
    m["event_contract_id"] = env["binary_phase_event_contract_id"]
    expect(lambda mm=m: C.verify_env_config(mm, cfg), C.J1Error,
           "MERGING the two contract ids into one value must fail closed")
    # a profile path that does not exist
    m = copy.deepcopy(env); m["binary_phase_detector_profile_file"] = "/nonexistent/profile.json"
    expect(lambda mm=m: C.verify_env_config(mm, cfg), C.J1Error,
           "a detector profile that is not on disk must fail closed")

    # ---------------------------------------------------------------- collection semantics ------
    sem = C.verify_collection_semantics(C.COLLECTION_SEMANTICS)
    check(sem["pinned"] == {"seed": 123, "teacher_lookahead_s": 0.0,
                            "action_noise_sigma": [0.0, 0.0], "action_noise_hold_steps": 1,
                            "action_noise_hold_duration_s": 0.01},
          "the July collection semantics are pinned verbatim")
    check("not defaults" in sem["note"], "and declared to be artefact values, not defaults")
    check("prescribed_teacher_action" in sem["label_source"]
          and "no other label source" in sem["label_source"],
          "labels may come only from prescribed_teacher_action on teacher-visited states")
    for key, value in C.COLLECTION_SEMANTICS.items():
        m = dict(C.COLLECTION_SEMANTICS); m.pop(key)
        expect(lambda mm=m: C.verify_collection_semantics(mm), C.J1Error,
               f"omitting the semantic {key} must fail closed")
        m = dict(C.COLLECTION_SEMANTICS)
        m[key] = [9.9, 9.9] if isinstance(value, list) else (
            float(value) + 1.0 if isinstance(value, (int, float)) else "___WRONG___")
        expect(lambda mm=m: C.verify_collection_semantics(mm), C.J1Error,
               f"a mutated semantic {key} must fail closed")

    # ---------------------------------------------------------------- J1 gate -------------------
    check(C.J1_GATE == {"steps_required": 500, "end_reason": "episode_time_limit",
                        "valid_cycles_min": 2, "phase_timeout_stance_max": 0,
                        "phase_timeout_swing_max": 0,
                        "morphology_causal_contract_failure_max": 0,
                        "hs_cancelled_count_max": 0, "resync_count_max": 1,
                        "max_penetration_m_max": 0.020},
          "the J1 binding gate is exactly the COMMON gate")
    for k in C.J1_GATE_EXCLUDES_KINEMATICS:
        check(k not in C.J1_GATE, f"the kinematic criterion {k} is NOT a J1 binding gate")
    check("action_clipped_steps" not in C.J1_GATE
          and C.DIAGNOSTIC_NOT_BINDING == ("action_clipped_steps",),
          "action clipping is a diagnostic, never a J1 gate")
    # ------------------------------------------------------------ PRODUCTION-SHAPED FIXTURES ----
    # Shapes copied from the real rollout journal and reward_function; values are synthetic.
    def row(step: int, *, cycles=2, hs=3, to=3, resync=0, cancelled=0,
            timeout_exceeded=0.0, timeout_side=0.0, morph_failed=0.0, pen=0.001,
            drop_fsm_key=None, fsm_extra=None, rt_extra=None, no_fsm=False):
        fsm = {
            "valid_cycle_count": float(cycles), "valid_hs_count": float(hs),
            "valid_to_count": float(to), "resync_count": float(resync),
            "hs_cancelled_count": float(cancelled),
            "timeout_exceeded": float(timeout_exceeded), "timeout_side": float(timeout_side),
            "fsm_behaviour_version": "v3", "event_source": "binary_active_v26",
            "state_name": "STANCE_AFTER_HS", "state_id": 1.0,
            "cycle_progress_credit": 0.25, "stance_elapsed_s": 0.518,
            "accepted_transitions_this_step": [], "cycle_reject_reason": "",
        }
        if drop_fsm_key:
            fsm.pop(drop_fsm_key)
        if fsm_extra:
            fsm.update(fsm_extra)
        rt = {
            "phase_valid_cycle_count": float(cycles), "phase_valid_hs_count": float(hs),
            "phase_valid_to_count": float(to),
            "phase_timeout_exceeded": float(timeout_exceeded),
            "phase_timeout_side": float(timeout_side),
            "morphology_causal_failed_closed": float(morph_failed),
            "grf_penetration_m": float(pen), "reserve_norm_nm": 158.4,
        }
        if rt_extra:
            rt.update(rt_extra)
        r = {"step": step, "reward": 0.128, "reward_terms": rt,
             "observation": {"pros_knee_angle": -0.31, "pros_knee_angle_vel": 0.9,
                             "pros_ankle_angle": 0.05, "pros_ankle_angle_vel": -0.4}}
        if not no_fsm:
            r["phase_fsm"] = fsm
        return r

    nominal = [row(i) for i in range(1, 501)]
    s = C._summarise(nominal, "episode_time_limit", 0)
    check(s["valid_cycle_count"] == 2,
          "PRODUCTION KEYS: phase_fsm.valid_cycle_count yields 2 cycles. Reading July's "
          "unprefixed reward_terms names, as rev0 did, would have reported ZERO and failed J1")
    check("valid_cycle_count" not in nominal[0]["reward_terms"]
          and "phase_valid_cycle_count" in nominal[0]["reward_terms"],
          "and the fixture proves reward_terms exposes only the phase_ prefixed name")
    check(C._evaluate_gate(s)["pass"] is True, "the nominal fixture passes the J1 gate")
    check(s["fsm_behaviour_versions"] == ["v3"] and s["event_sources"] == ["binary_active_v26"],
          "the FSM behaviour version and event source are recorded")
    # v3 cancellation contract: valid_hs may DECREASE, so MAX not last
    decreasing = [row(1, hs=4), row(2, hs=4), row(3, hs=2)]
    check(C._summarise(decreasing, "episode_time_limit", 0)["valid_hs_count"] == 4,
          "cumulative counters aggregate with MAX: reading the last row would understate hs")
    # counters that must fail
    check(C._evaluate_gate(C._summarise([row(i, resync=2) for i in range(1, 501)],
                                        "episode_time_limit", 0))["pass"] is False,
          "resync_count 2 fails the gate")
    check(C._evaluate_gate(C._summarise([row(i, cancelled=1) for i in range(1, 501)],
                                        "episode_time_limit", 0))["pass"] is False,
          "hs_cancelled_count 1 fails the gate")
    # a missing counter must RAISE, never default to zero
    for key in ("resync_count", "hs_cancelled_count", "valid_cycle_count",
                "fsm_behaviour_version", "event_source"):
        e = expect(lambda k=key: C._summarise([row(1, drop_fsm_key=k)], "episode_time_limit", 0),
                   C.J1Error, f"a missing phase_fsm.{key} must fail closed, never default to 0")
        check(key in str(e), f"and the failure names {key}")
    e = expect(lambda: C._summarise([row(1, no_fsm=True)], "episode_time_limit", 0), C.J1Error,
               "an absent phase_fsm mapping must fail closed")
    check("phase_fsm" in str(e), "and the failure names phase_fsm")
    # timeouts and morphology failure from the REAL fields
    st = C._summarise([row(1, timeout_exceeded=1.0, timeout_side=1.0)], "episode_time_limit", 0)
    check(st["phase_timeout_stance"] == 1 and st["phase_timeout_swing"] == 0,
          "timeout_side 1.0 is stance, read from phase_timeout_exceeded + phase_timeout_side")
    sw = C._summarise([row(1, timeout_exceeded=1.0, timeout_side=2.0)], "episode_time_limit", 0)
    check(sw["phase_timeout_swing"] == 1, "timeout_side 2.0 is swing")
    check(C._evaluate_gate(st)["pass"] is False and C._evaluate_gate(sw)["pass"] is False,
          "either timeout fails the gate")
    er = C._summarise([row(1)], "phase_timeout:stance", 0)
    check(er["phase_timeout_stance"] == 1,
          "the end_reason phase_timeout:stance is also counted as evidence")
    mf = C._summarise([row(1, morph_failed=1.0)], "episode_time_limit", 0)
    check(mf["morphology_causal_contract_failure"] == 1
          and C._evaluate_gate(mf)["pass"] is False,
          "morphology_causal_failed_closed is the real field and it fails the gate")
    check(C._summarise([row(1)], "morphology_causal_contract_failure",
                       0)["morphology_causal_contract_failure"] == 1,
          "and the matching end_reason is counted too")
    expect(lambda: C._summarise([row(1, timeout_exceeded=1.0, timeout_side=7.0)],
                                "episode_time_limit", 0), C.J1Error,
           "an unknown timeout side must fail closed rather than be ignored")
    # non-finite and non-integer required evidence
    expect(lambda: C._summarise([row(1, fsm_extra={"resync_count": float("nan")})],
                                "episode_time_limit", 0), C.J1Error,
           "a non-finite counter must fail closed")
    expect(lambda: C._summarise([row(1, fsm_extra={"resync_count": 1.5})],
                                "episode_time_limit", 0), C.J1Error,
           "a non-integer counter must fail closed")
    expect(lambda: C._summarise([row(1, rt_extra={"grf_penetration_m": float("inf")})],
                                "episode_time_limit", 0), C.J1Error,
           "a non-finite penetration must fail closed")
    # the reward_terms exposure must agree with the FSM counters
    expect(lambda: C._summarise([row(1, rt_extra={"phase_valid_cycle_count": 9.0})],
                                "episode_time_limit", 0), C.J1Error,
           "a disagreement between reward_terms and phase_fsm must fail closed")
    # structured evidence survives conversion
    conv = C._jsonable(row(1), "row")
    check(isinstance(conv["phase_fsm"], dict) and isinstance(conv["reward_terms"], dict)
          and isinstance(conv["observation"], dict),
          "nested mappings remain MAPPINGS: rev0 stringified them and made the trace false")
    check(isinstance(conv["phase_fsm"]["accepted_transitions_this_step"], list),
          "nested lists remain lists")
    check(conv["phase_fsm"]["event_source"] == "binary_active_v26", "and values survive intact")
    expect(lambda: C._jsonable({"x": float("nan")}, "f"), C.J1Error,
           "a non-finite value is rejected, never silently converted")
    # prosthetic state extracted BY NAME
    names = tuple(f"f{i}" for i in range(35))
    names = names[:2] + ("pros_knee_angle", "pros_knee_angle_vel", "pros_ankle_angle",
                         "pros_ankle_angle_vel") + names[6:]
    vec = [float(i) for i in range(35)]
    ps = C._prosthetic_state(vec, names)
    check(ps == {"pros_knee_angle": 2.0, "pros_knee_angle_vel": 3.0,
                 "pros_ankle_angle": 4.0, "pros_ankle_angle_vel": 5.0},
          "knee and ankle q and qdot are extracted by NAME from the feature manifest")
    expect(lambda: C._prosthetic_state(vec, tuple(f"f{i}" for i in range(35))), C.J1Error,
           "a manifest without the prosthetic features must fail closed")

    # ------------------------------------------------------ (1) FULL-LENGTH BRANCH --------------
    # 500 steps, neither terminated nor truncated: the path rev1 could not survive.
    full = [row(i) for i in range(1, 501)]
    for r in full:
        r["end_reason"] = ""
    full[-1]["end_reason"] = "episode_time_limit"
    check(all("info" not in r for r in full),
          "rows carry no 'info' key: rev1 read trace[-1]['info'] and would have raised KeyError")
    er = C._resolve_end_reason(full)
    check(er == "episode_time_limit",
          "the full-length branch resolves the end_reason from the LAST ROW, no KeyError")
    s_full = C._summarise(full, er, 0)
    check(C._evaluate_gate(s_full)["pass"] is True,
          "and a genuine 500-step episode_time_limit passes the gate")
    blank = [row(i) for i in range(1, 501)]
    for r in blank:
        r["end_reason"] = ""
    e = expect(lambda: C._resolve_end_reason(blank), C.J1Error,
               "a missing end_reason must FAIL CLOSED, never be assumed to be episode_time_limit")
    check("refusing to assume" in str(e), "and the message says the success is not invented")
    noattr = [row(i) for i in range(1, 501)]
    for r in noattr:
        r.pop("end_reason", None)
    expect(lambda: C._resolve_end_reason(noattr), C.J1Error,
           "an absent end_reason key must fail closed too, without KeyError")
    expect(lambda: C._resolve_end_reason([]), C.J1Error, "an empty trace has no end_reason")

    # ------------------------------------------------------ (2) LEAF CLEANUP ---------------------
    import shutil as _shutil
    root = C.OUT_ROOT
    created_root = not root.exists()
    root.mkdir(parents=True, exist_ok=True)
    sibling = root / "j1_PRIOR_SIBLING"
    sibling.mkdir(exist_ok=True)
    (sibling / "keep.json").write_text("{}", encoding="utf-8")
    leaf = root / "j1_LEAF_UNDER_TEST"
    leaf.mkdir(exist_ok=True)
    (leaf / "sim_outputs").mkdir(exist_ok=True)          # what SimulationRunner.__init__ creates
    (leaf / "sim_outputs" / "partial.sto").write_text("x", encoding="utf-8")
    C._remove_own_leaf(leaf)
    check(not leaf.exists(),
          "MEASURED: a leaf populated by a failing env construction is removed entirely")
    check(sibling.exists() and (sibling / "keep.json").is_file(),
          "and the sibling run is untouched")
    check(root.exists(), "and the root survives")
    expect(lambda: C._remove_own_leaf(root), C.J1Error,
           "removing the ROOT must be refused outright")
    expect(lambda: C._remove_own_leaf(HERE), C.J1Error,
           "and so must removing the module directory")
    outside = HERE / "_not_a_leaf_under_test"
    outside.mkdir(exist_ok=True)
    C._remove_own_leaf(outside)
    check(outside.exists(),
          "a directory outside the run root is never auto-removed; it is left in place")
    outside.rmdir()
    check("SimulationRunner.__init__ calls os.makedirs" in src,
          "and the collector documents WHY the cleanup exists")

    # end-to-end: a factory that creates sim_outputs and THEN raises must leave no leaf
    import types
    fake_leaf = root / "j1_FACTORY_FAILURE"
    made: list[str] = []

    def _boom(env_config):  # noqa: ANN001
        target = Path(env_config["output_dir"])
        target.mkdir(parents=True, exist_ok=True)      # exactly what SimulationRunner does
        (target / "partial.sto").write_text("x", encoding="utf-8")
        made.append(str(target))
        raise RuntimeError("simulated construction failure after makedirs")

    saved = {name: sys.modules.get(name)
             for name in ("env_factory", "exploration_noise", "target_domain_imitation")}
    try:
        fake_factory = types.ModuleType("env_factory")
        fake_factory.make_cmc_env = _boom
        sys.modules["env_factory"] = fake_factory
        sys.modules.setdefault("exploration_noise", types.ModuleType("exploration_noise"))
        sys.modules.setdefault("target_domain_imitation",
                               types.ModuleType("target_domain_imitation"))
        expect(lambda: C.collect(authorized_stage=C.STAGE, out_dir=fake_leaf, progress=False),
               RuntimeError, "the simulated construction failure propagates")
    finally:
        for name, mod in saved.items():
            if mod is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = mod
    check(made and Path(made[0]).name == "sim_outputs",
          "MEASURED: the factory really did create out/sim_outputs before failing")
    check(not fake_leaf.exists(),
          "and the leaf is GONE afterwards: a failed construction leaves nothing behind")
    check(sibling.exists(), "while the sibling run is still untouched")
    check(root.exists(), "and the root still exists, so future preflights stay GO")
    i_mk = src.find("out.mkdir(parents=True, exist_ok=False)")
    i_make = src.find("env_factory.make_cmc_env")
    check(0 < i_mk < i_make and "_remove_own_leaf(out)" in src[i_mk:],
          "the construction is wrapped so a failure removes the leaf it just created")
    _shutil.rmtree(sibling, ignore_errors=True)
    if created_root:
        _shutil.rmtree(root, ignore_errors=True)

    # ------------------------------------------------------ (3) NON-FINITE, FAIL-CLOSED ----------
    check("info exposes no 'time'" in src and 'info["time"]' in src,
          "info.time is required and validated before the row is appended")
    check('_finite(base.t, f"step {step}: time_before")' in src
          and '_finite(reward, f"step {step}: reward")' in src,
          "time_before and reward are validated before the row is appended")
    i_val = src.find("t_after = _finite")
    i_row = src.find('row: dict[str, Any] = {')
    check(0 < i_val < i_row, "the validation happens BEFORE the row is constructed")
    check(src.count("allow_nan=False") >= 2,
          "both the trace and the receipt are serialised with allow_nan=False")
    expect(lambda: json.dumps({"x": float("nan")}, allow_nan=False), ValueError,
           "and allow_nan=False genuinely rejects a NaN")
    bad_ret = [row(1), row(2)]
    bad_ret[1]["reward"] = float("nan")
    expect(lambda: C._summarise(bad_ret, "episode_time_limit", 0), C.J1Error,
           "a NaN reward makes episode_return non-finite and must fail closed")
    check("episode_return" in src and '_finite(sum(' in src,
          "episode_return is itself checked for finiteness")

    # ------------------------------------------------------ (4) REQUIRED REWARD TELEMETRY --------
    check(set(C.RT_REQUIRED) == {"phase_valid_cycle_count", "phase_valid_hs_count",
                                 "phase_valid_to_count", "phase_timeout_exceeded",
                                 "phase_timeout_side", "morphology_causal_failed_closed",
                                 "grf_penetration_m"},
          "seven reward_terms fields are required on every row")
    for key in C.RT_REQUIRED:
        r = row(1); r["reward_terms"].pop(key)
        e = expect(lambda rr=r: C._summarise([rr], "episode_time_limit", 0), C.J1Error,
                   f"a missing reward_terms.{key} must fail closed, never default to 0.0")
        check(key in str(e), f"and the failure names {key}")
        r = row(1); r["reward_terms"][key] = float("nan")
        expect(lambda rr=r: C._summarise([rr], "episode_time_limit", 0), C.J1Error,
               f"a non-finite reward_terms.{key} must fail closed")
    # the FSM/reward cross-check is MANDATORY, on counters and on the timeout pair
    for rt_key, fsm_key in (("phase_valid_cycle_count", "valid_cycle_count"),
                            ("phase_valid_hs_count", "valid_hs_count"),
                            ("phase_valid_to_count", "valid_to_count"),
                            ("phase_timeout_exceeded", "timeout_exceeded"),
                            ("phase_timeout_side", "timeout_side")):
        r = row(1, timeout_exceeded=1.0, timeout_side=1.0)
        r["reward_terms"][rt_key] = float(r["reward_terms"][rt_key]) + 1.0
        e = expect(lambda rr=r: C._summarise([rr], "episode_time_limit", 0), C.J1Error,
                   f"reward_terms.{rt_key} disagreeing with phase_fsm.{fsm_key} must fail closed")
        check(rt_key in str(e) and fsm_key in str(e), "and the failure names both sides")

    # ------------------------------------------------------ (5) RUNTIME IDENTITY -----------------
    check(C.EXPECTED_FSM_BEHAVIOUR_VERSION == "v3"
          and C.EXPECTED_EVENT_SOURCE == "binary_active_v26",
          "the preregistered runtime identity is FSM v3 with the V26 event source")
    for bad_version in ("v2", "", "V3", "v3 "):
        r = row(1, fsm_extra={"fsm_behaviour_version": bad_version})
        e = expect(lambda rr=r: C._summarise([rr], "episode_time_limit", 0), C.J1Error,
                   f"fsm_behaviour_version {bad_version!r} must fail closed")
        check("fsm_behaviour_version" in str(e), "and the failure names the field")
    for bad_source in ("binary_active", "legacy_events", "", "binary_active_v25"):
        r = row(1, fsm_extra={"event_source": bad_source})
        e = expect(lambda rr=r: C._summarise([rr], "episode_time_limit", 0), C.J1Error,
                   f"event_source {bad_source!r} must fail closed")
        check("event_source" in str(e), "and the failure names the field")
    mixed = [row(1), row(2, fsm_extra={"fsm_behaviour_version": "v2"})]
    expect(lambda: C._summarise(mixed, "episode_time_limit", 0), C.J1Error,
           "a single non-v3 row anywhere in the episode fails the whole collection")

    # ------------------------------------------------------ (6) REALIZED NOISE -------------------
    s_noise = C._summarise([row(1)], "episode_time_limit", 0, [0.0, 0.0])
    check(s_noise["realized_noise_rms"] == [0.0, 0.0],
          "the realised noise RMS is recorded in the summary")
    check(s_noise["action_noise_sigma"] == [0.0, 0.0],
          "alongside the sigma actually used, which stays [0.0, 0.0] in J1")
    check(C.COLLECTION_SEMANTICS["action_noise_sigma"] == [0.0, 0.0],
          "and no hyperparameter changed")
    check('"realized_noise_rms": summary["realized_noise_rms"]' in src,
          "the receipt exposes it too")

    good = {"steps": 500, "end_reason": "episode_time_limit", "valid_cycle_count": 2,
            "phase_timeout_stance": 0, "phase_timeout_swing": 0,
            "morphology_causal_contract_failure": 0, "hs_cancelled_count": 0,
            "resync_count": 1, "max_penetration_m": 0.0199, "action_clipped_steps": 7}
    g = C._evaluate_gate(good)
    check(g["pass"] is True and g["failed"] == [], "a conforming episode passes")
    check(g["kinematic_quality_applies"] is False, "and kinematics are declared non-applicable")
    check(C._evaluate_gate({**good, "action_clipped_steps": 500})["pass"] is True,
          "clipping alone never fails the J1 gate")
    for field, bad in (("steps", 499), ("end_reason", "grf_penetration"),
                       ("valid_cycle_count", 1), ("phase_timeout_stance", 1),
                       ("phase_timeout_swing", 1), ("morphology_causal_contract_failure", 1),
                       ("hs_cancelled_count", 1), ("resync_count", 2),
                       ("max_penetration_m", 0.0201)):
        r = C._evaluate_gate({**good, field: bad})
        check(r["pass"] is False, f"a violating {field} fails the gate")
    check(C._evaluate_gate({**good, "max_penetration_m": 0.020})["pass"] is True,
          "the penetration bound is inclusive at exactly 0.020 m")

    # ---------------------------------------------------------------- PREFLIGHT IS INERT --------
    # sentinel: any attempt to construct, reset or step an environment raises immediately
    tripped: list[str] = []
    real_import = builtins.__import__
    banned = {"env_factory", "exploration_noise", "target_domain_imitation", "torch", "ray",
              "opensim"}

    def guarded(name, *a, **k):  # noqa: ANN001
        root = name.split(".")[0]
        if root in banned:
            tripped.append(name)
            raise AssertionError(f"preflight imported the heavy module {name}")
        return real_import(name, *a, **k)

    builtins.__import__ = guarded
    try:
        pre = C.preflight()
    finally:
        builtins.__import__ = real_import
    check(tripped == [],
          f"PROVEN INERT: --preflight imported no env module and touched no environment "
          f"(tripped={tripped})")
    check(pre["verdict"] == "GO" and pre["blockers"] == [], "the preflight verdict is GO")
    check(pre["inert"] == {"environment_constructed": False, "environment_reset": False,
                           "environment_stepped": False,
                           "note": "the env factory is imported inside collect() only"},
          "and it declares its own inertness")
    for token in ("env_factory", "exploration_noise", "make_cmc_env", "env.reset", "env.step"):
        idx = src.find("def collect(")
        check(src.find(token) == -1 or src.find(token) > idx,
              f"{token} appears only inside collect(), never at module scope or in preflight")
    check(pre["execution_requires"] == f"--authorized-stage {C.STAGE}",
          "collection requires an explicit execution token")
    check(pre["env_config_sha256"] == C._sha_obj(env), "the receipt pins the env_config hash")
    check(pre["actor_contract"]["width"] == 35
          and pre["actor_contract"]["masked_columns"] == list(C.MASKED_COLUMNS)
          and "no widening" in pre["actor_contract"]["note"],
          "one 35D actor, columns 0,1 and 25..34 masked, no standalone 25D artefact, no widening")
    check("contralateral absence is intentional" in pre["actor_contract"]["note"],
          "and the contralateral absence is declared intentional")
    check(pre["future_todo_non_operational"]
          == ["LOTO (B1)", "LOCO (B1R1)", "B1R2", "B1R2-A", "B1R2-B"],
          "LOTO/LOCO/B1R1/B1R2 remain future, non-operational TODOs")

    # ---------------------------------------------------------------- execution is gated --------
    expect(lambda: C.collect(authorized_stage=None), C.J1Error,
           "collection without the token must be refused")
    expect(lambda: C.collect(authorized_stage="V26C-J0-AUDIT"), C.J1Error,
           "another stage's token must not authorise the collection")
    check("no-clobber" in src and "mkdir(parents=True, exist_ok=False)" in src,
          "the output directory is fresh and no-clobber")
    check(C.OUT_ROOT.name == "j1_runs" and str(HERE) in str(C.OUT_ROOT),
          "outputs land under this additive directory only")
    # Before the user's authorisation this asserted that OUT_ROOT was empty. That precondition is
    # spent: exactly one authorised collection has since run. The invariant that still matters is
    # that there is AT MOST ONE run and that it is the authorised leaf.
    leaves = sorted(p.name for p in C.OUT_ROOT.iterdir()
                    if p.is_dir()) if C.OUT_ROOT.is_dir() else []
    check(leaves in ([], ["j1_nominal_v26c_2026-08-26_r1"]),
          f"at most ONE collection exists and it is the authorised leaf; found {leaves}")
    if leaves:
        leaf = C.OUT_ROOT / leaves[0]
        for name, pin in (("teacher_dataset.npz",
                           "724d11342da3f3610152d7bd4cc7ca0dc1e8eb8c26a5b7c0947eb2451d1f8c41"),
                          ("teacher_trace.json",
                           "39af8f0b2d4b8f7e44f917e82ea0e435fa3889c4d022d06b5d6373212c691bd3"),
                          ("v26c_j1_collection_receipt.json",
                           "f54028d58dc9bfde01ede3c2a72f7ea63b67aeead02979291b03cd468bf37cdd")):
            check((leaf / name).is_file() and C._sha_file(leaf / name) == pin,
                  f"the collected {name} is byte-identical to the value pinned in the amendment")
        rec = json.loads((leaf / "v26c_j1_collection_receipt.json").read_text())
        check(rec["verdict"] == "FAIL" and rec["gate"]["failed"] == ["max_penetration_m"],
              "the recorded verdict is still FAIL on max_penetration_m alone, not re-scored")
        check(rec["summary"]["steps"] == 500
              and rec["summary"]["end_reason"] == "episode_time_limit",
              "and the run is still the 500-step episode_time_limit collection")

    # ---------------------------------------------------------------- traced evidence -----------
    for field in ("reward_terms", "grf_penetration_m", "valid_cycle_count",
                  "phase_timeout_stance", "phase_timeout_swing",
                  "morphology_causal_contract_failure", "hs_cancelled_count", "resync_count",
                  "action_clipped_steps", "actor_observation_vector_before", "teacher_action",
                  "executed_action", "end_reason"):
        check(field in src, f"the collector traces {field}")
    check("np.savez_compressed" in src and "teacher_dataset.npz" in src
          and "teacher_trace.json" in src,
          "it emits the dataset and the full trace")
    check("outputs_sha256" in src and "env_config_sha256" in src,
          "and lineage/config/gate receipts with hashes")

    # ---------------------------------------------------------------- cannot fit or promote -----
    for name in ("fit_masked", "adapt_actor", "train_ppo", "PPOConfig", "dagger", "DAgger",
                 "subprocess", "os"):
        check(name not in ids, f"the collector never references {name}: it cannot fit or promote")
    check("if False" not in src, "no dead conditional")

    print(json.dumps({"selftest": "PASS", "checks": CHECKS}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
