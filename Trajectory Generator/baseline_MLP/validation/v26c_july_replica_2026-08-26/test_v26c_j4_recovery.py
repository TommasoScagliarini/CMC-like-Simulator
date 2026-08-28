"""Fail-closed tests for the V26C J4 recovery tooling.

No environment, no training, no writes to any pinned artefact: mutations are proven on temporary
copies only. Every July rule asserted here is compared against the July source file or run artefact
it came from, not against a value retyped into this test.
"""
from __future__ import annotations
import ast, builtins, contextlib, io, json, os, shutil, sys, tempfile, tokenize
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26c_j4_recovery as J4  # noqa: E402

REPO = J4.REPO
BASELINE = J4.BASELINE
JULY_SELECTED = (REPO / "Trajectory Generator" / "runs" / "training"
                 / "target_domain_markov35_phase_aligned_scaled_full_r32_alt8_2026-07-13")
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


def main() -> int:
    src = (HERE / "v26c_j4_recovery.py").read_text()
    tree = ast.parse(src)
    ids = {t.string for t in tokenize.generate_tokens(io.StringIO(src).readline)
           if t.type == tokenize.NAME}
    amendment = json.loads(J4.AMENDMENT.read_text())

    # ---------------------------------------------------------------- the amendment -------------
    check(amendment["kind"] == "ADDITIVE, IMMUTABLE"
          and amendment["stage_amended"] == "V26C-J3-CLOSED-LOOP",
          "the amendment is additive and immutable")
    dno = amendment["does_not_overwrite"]
    check(dno["j3_verdict"] == "FAIL" and dno["j3_failed_gates"] == ["max_penetration_m"]
          and dno["j3_deployable"] is False and dno["j3_promotion"] == "NONE"
          and dno["j3_next_stage_authorized"] is False and dno["j3_quarantine_applies"] is True,
          "it states that the J3 receipt stays FAIL, undeployable and quarantined")
    check(dno["j1_verdict_unchanged"] == "FAIL", "and that the J1 FAIL is unchanged")
    th = amendment["thresholds_unchanged"]
    check(th["grf_penetration_soft_diagnostic_m"] == 0.020
          and th["grf_penetration_hard_guard_m"] == 0.028
          and "Neither is relaxed" in th["statement"],
          "0.020 m stays the soft diagnostic threshold and 0.028 m the hard guard")
    for name, pin in amendment["pinned_artefacts_sha256"].items():
        path = HERE / name
        check(path.is_file() and J4._sha_file(path) == pin,
              f"the amendment pins the REAL hash of {name}")
    check(amendment["authorises"]["scope"] == "RECOVERY ONLY", "the scope is recovery only")
    for f in ("retry of the current J3 rollout", "sigma / exploration noise of any kind",
              "critic training or PPO updates", "ex-novo training",
              "promotion of any artefact", "LOTO", "LOCO", "B1R1", "B1R2",
              "a standalone 25D actor", "widening the actor beyond 35D",
              "contralateral features"):
        check(f in amendment["authorises"]["forbidden"], f"{f} is forbidden by the amendment")
    check(amendment["revalidation"]["authorised_now"] is False
          and amendment["progressive_recovery_iterations"]["authorisation"].endswith(
              "NO physical rollout of any iteration is authorised by this amendment."),
          "no physical rollout is authorised")

    # the stage-dependent masking, stated explicitly
    base_mask = amendment["actor_contract"]["base_stage_masking"]
    rec_mask = amendment["actor_contract"]["recovery_stage_masking"]
    check(base_mask["hard_zero_columns"] == [0, 1] + list(range(25, 35))
          and base_mask["applies_to"] == ["V26C-J2-BASE-FIT", "V26C-J3-CLOSED-LOOP"],
          "the BASE stages hard-zero the clock AND the controller memory")
    check(rec_mask["hard_zero_columns"] == [0, 1]
          and rec_mask["trainable_controller_memory_columns"] == list(range(25, 35))
          and rec_mask["applies_to"] == ["V26C-J4-RECOVERY"],
          "the RECOVERY stage hard-zeroes only the clock and trains columns 25..34")
    check("is NOT widened" in rec_mask["rationale"] and "35D" in rec_mask["rationale"],
          "while the actor stays 35D")
    check("REMAIN ABSENT" in rec_mask["contralateral_features"],
          "and contralateral features remain absent")

    # ---------------------------------------------------------------- July provenance -----------
    july = json.loads((JULY_SELECTED / "run_summary.json").read_text())
    hp = july["adaptation"]["hyperparameters"]
    for key, july_key in (("seed", "seed"), ("batch_size", "batch_size"),
                          ("learning_rate", "learning_rate"),
                          ("validation_fraction", "validation_fraction"),
                          ("patience", "patience"), ("clip_weight", "clip_weight"),
                          ("logstd_weight", "logstd_weight"), ("anchor_weight", "anchor_weight"),
                          ("freeze_logstd_head", "freeze_logstd_head")):
        check(J4.JULY_RECOVERY_HP[key] == hp[july_key],
              f"{key} = {hp[july_key]!r} is READ FROM the selected July run, not invented")
    check(J4.JULY_RECOVERY_HP["train_full_actor"] is True
          and hp["trainable_first_layer_features"] is None,
          "train_full_actor matches July's null trainable_first_layer_features")
    check(J4.MARKOV_CONTROLLER_FEATURE_SCALES == hp["first_layer_feature_scales"],
          "and the ten physical scales are byte-equal to July's")
    check(july["adaptation"]["epochs_requested"] == J4.JULY_RECOVERY_HP["epochs"] == 400,
          "the epoch budget is July's own 400")
    check(july["adaptation"]["critic_trained"] is False
          and july["adaptation"]["ppo_updates"] == 0,
          "July trained no critic and ran no PPO update")
    check(july["adaptation"]["disabled_clock_column_norms"] == {"gait_phase_sin": 0.0,
                                                                "gait_phase_cos": 0.0},
          "July kept the clock columns at exactly zero")
    learned = july["learned_column_norms"]
    check(len(learned) == 10 and all(v > 0.0 for v in learned.values()),
          "while ALL TEN controller-memory columns ended NON-zero: they were trainable")
    # the July source rules, compared against the production files they come from
    markov_src = (BASELINE / "target_domain_markov_adaptation.py").read_text()
    noise_src = (BASELINE / "target_domain_noise_adaptation.py").read_text()
    imit_src = (BASELINE / "target_domain_imitation.py").read_text()
    check('"--stop-before-discrete-mismatch"' in markov_src
          and "default=True" in markov_src.split('"--stop-before-discrete-mismatch"')[1][:400],
          "July enables the discrete-mismatch truncation by default")
    check("after which fixed-step teacher labels are phase-invalid" in markov_src,
          "and says why: after the mismatch the fixed-step labels are phase-invalid")
    check('_in_contact", "_heel_strike", "_toe_off", "_saturated"' in noise_src
          and 'phase_fsm_", "phase_expected_' in noise_src,
          "July's discrete features are the contact/event/FSM ones")
    check("first_layer_weight[:, feature_names.index(name)].div_(scale)" in imit_src,
          "July folds the physical scale into the first layer before saving")
    check("_zero_disabled_clock_columns(module, feature_names)" in imit_src,
          "and re-zeroes the clock columns after every optimiser step")
    scales_literal = {
        "pros_knee_angle_served_ref_vel": 4.0, "pros_knee_angle_served_ref_accel": 60.0,
        "pros_ankle_angle_served_ref_vel": 3.5, "pros_ankle_angle_served_ref_accel": 55.0}
    for name, value in scales_literal.items():
        check(J4.MARKOV_CONTROLLER_FEATURE_SCALES[name] == value
              and f'"{name}": {value}' in markov_src,
              f"the scale {name} = {value} is quoted from the July source")

    # ---------------------------------------------------------------- the schema ---------------
    names = J4.feature_names()
    disc = J4.discrete_feature_indices(names)
    check(list(disc) == [11, 12, 13, 17, 18, 19, 20, 21],
          f"July's discrete selector picks the 8 contact/event/FSM columns, got {list(disc)}")
    check([names[i] for i in disc][0] == "online_left_in_contact",
          "the first of which is online_left_in_contact")
    ctrl = J4.trainable_controller_features(names)
    check(len(ctrl) == 10 and tuple(names.index(n) for n in ctrl) == tuple(range(25, 35)),
          "the ten trainable controller features sit exactly at columns 25..34")
    check(all(not n.endswith(("_sea_u_abs", "_sea_u_saturated")) for n in ctrl),
          "and exclude _sea_u_abs / _sea_u_saturated, as July does")
    check(J4.clock_indices(names) == (0, 1), "the clock columns are 0 and 1")
    check(set(ctrl) == set(J4.MARKOV_CONTROLLER_FEATURE_SCALES),
          "every trainable column has a declared physical scale")

    # ---------------------------------------------------------------- pinned inputs -------------
    inputs = J4.verify_inputs()
    check(inputs["j3_verdict_preserved"] == "FAIL"
          and inputs["j3_failed"] == ["max_penetration_m"]
          and inputs["j1_verdict_preserved"] == "FAIL",
          "the J1 and J3 FAIL verdicts are read back from their own receipts")
    check(inputs["j3_max_penetration_m"] == 0.02704966381076714,
          "with the J3 penetration unchanged")
    check("neither re-scores nor retroactively promotes" in inputs["statement"],
          "and J4 states that it promotes neither")
    for rel in list(J4.PIN_INPUTS):
        saved = J4.PIN_INPUTS[rel]
        J4.PIN_INPUTS[rel] = "0" * 64
        e = expect(J4.verify_inputs, J4.J4Error, f"a mutated {rel} must be refused")
        check(rel in str(e), f"and the failure names {rel}")
        J4.PIN_INPUTS[rel] = saved
    check(all(J4._sha_file(HERE / r) == h for r, h in J4.PIN_INPUTS.items()),
          "no pinned input was touched by those refusals")
    check(J4.PIN_AMENDMENT == J4._sha_file(J4.AMENDMENT)
          and J4.PIN_AMENDMENT
          == "fed5b81666782902ed4ab0187da457cdfbf0516dd676c049fdfe41e48d21614f",
          "the recovery-only amendment is pinned by EXACT hash, not merely recomputed")
    saved_amend = J4.PIN_AMENDMENT
    J4.PIN_AMENDMENT = "0" * 64
    e = expect(J4.verify_inputs, J4.J4Error, "a changed amendment must be refused")
    check("amendment changed" in str(e), "and the failure says the amendment changed")
    J4.PIN_AMENDMENT = saved_amend
    check(J4.verify_inputs()["amendment_sha256"] == saved_amend,
          "and verification passes again with the true pin")
    # THE AMENDMENT IS THE MANIFEST, verified at RUNTIME by verify_inputs - not only by this test
    manifest = amendment["pinned_artefacts_sha256"]
    got = inputs["amendment_manifest_sha256"]
    check(inputs["amendment_manifest_entries"] == len(manifest) == 12 and got == manifest,
          f"verify_inputs checks all {len(manifest)} amendment-pinned artefacts at runtime")
    check("v26c_j3_closed_loop.py" in got
          and got["v26c_j3_closed_loop.py"]
          == "f14e7ce06161228ef81f44f38863269b283002661d97e7995d48106152850759",
          "including the J3 runner that produced the recovery trace")
    check(set(manifest) - set(J4.PIN_INPUTS) != set(),
          "the manifest covers artefacts the runner's own PIN_INPUTS does not list")
    # a manifest entry that no longer matches must abort - proven by editing a COPY of the
    # amendment and pointing the module at it, never by touching the real one
    with tempfile.TemporaryDirectory() as td2:
        forged = Path(td2) / "forged_amendment.json"
        bad = json.loads(J4.AMENDMENT.read_text())
        bad["pinned_artefacts_sha256"]["v26c_j3_closed_loop.py"] = "0" * 64
        forged.write_text(json.dumps(bad, indent=2), encoding="utf-8")
        saved_path, saved_pin = J4.AMENDMENT, J4.PIN_AMENDMENT
        J4.AMENDMENT, J4.PIN_AMENDMENT = forged, J4._sha_file(forged)
        e = expect(J4.verify_inputs, J4.J4Error,
                   "a manifest whose pin no longer matches the file must abort")
        check("v26c_j3_closed_loop.py changed" in str(e), "naming the artefact that moved")
        missing = json.loads(J4.AMENDMENT.read_text())
        missing["pinned_artefacts_sha256"] = {"does_not_exist.json": "0" * 64}
        forged.write_text(json.dumps(missing, indent=2), encoding="utf-8")
        J4.PIN_AMENDMENT = J4._sha_file(forged)
        e = expect(J4.verify_inputs, J4.J4Error, "a manifest naming a missing file must abort")
        check("which is missing" in str(e), "saying the artefact is missing")
        forged.write_text(json.dumps({**bad, "pinned_artefacts_sha256": {}}, indent=2),
                          encoding="utf-8")
        J4.PIN_AMENDMENT = J4._sha_file(forged)
        expect(J4.verify_inputs, J4.J4Error, "an empty manifest must abort")
        J4.AMENDMENT, J4.PIN_AMENDMENT = saved_path, saved_pin
    check(J4.verify_inputs()["amendment_manifest_entries"] == 12,
          "and the real amendment verifies clean afterwards")
    check(J4._sha_file(J4.AMENDMENT) == saved_amend,
          "the real amendment was never modified: only temporary copies were forged")

    # ---------------------------------------------------------------- the truncation rule -------
    nominal = json.loads((J4.J1_LEAF / "teacher_trace.json").read_text())
    recovery = json.loads((J4.J3_LEAF / "j3_trace.json").read_text())
    kept, report = J4.truncate_before_discrete_mismatch(nominal, recovery, names)
    check(report["first_discrete_mismatch_step"] == 13 and report["retained_steps"] == 12
          and len(kept) == 12,
          f"INDEPENDENTLY MEASURED: the July-strict prefix is 12 rows, mismatch at step "
          f"{report['first_discrete_mismatch_step']}")
    col = report["mismatching_columns"]
    check(len(col) == 1 and col[0]["index"] == 11
          and col[0]["feature"] == "online_left_in_contact"
          and col[0]["nominal"] == 0.0 and col[0]["recovery"] == 1.0,
          "the mismatch is online_left_in_contact, teacher 0 vs student 1")
    same, same_report = J4.truncate_before_discrete_mismatch(nominal, nominal, names)
    check(len(same) == 500 and same_report["first_discrete_mismatch_step"] is None,
          "a trace identical to the nominal one keeps all 500 rows")
    perturbed = [dict(r) for r in nominal]
    v = list(perturbed[300]["actor_observation_vector_before"])
    v[17] = 1.0 - v[17]
    perturbed[300] = {**perturbed[300], "actor_observation_vector_before": v}
    _, pr = J4.truncate_before_discrete_mismatch(nominal, perturbed, names)
    check(pr["first_discrete_mismatch_step"] == 301 and pr["retained_steps"] == 300,
          "a single flipped FSM flag truncates exactly at that step")
    contin = [dict(r) for r in nominal[:5]]
    contin[3] = {**contin[3], "step": 99}
    expect(lambda: J4._obs_matrix(contin, len(names), "t"), J4.J4Error,
           "a non-contiguous trace is refused")
    sig = ast.parse(src)
    fn = next(f for f in ast.walk(sig) if isinstance(f, ast.FunctionDef)
              and f.name == "truncate_before_discrete_mismatch")
    check([a.arg for a in fn.args.args] == ["nominal_rows", "recovery_rows", "names"]
          and not fn.args.kwonlyargs and not fn.args.defaults,
          "the rule takes NO tolerance, window or slack parameter: it cannot be relaxed")

    # ---------------------------------------------------------------- the aggregate -------------
    built = J4.build_recovery_dataset()
    rep = built["report"]
    check(J4.NOMINAL_REPEAT == 32 and J4.RECOVERY_REPEAT == 2,
          "the SELECTED July composition is used: nominal 32x, recovery 2x")
    check(J4.JULY_RECOVERY_COMPOSITION["nominal_repeat"] == J4.NOMINAL_REPEAT
          and J4.JULY_RECOVERY_COMPOSITION["recovery_repeat"] == J4.RECOVERY_REPEAT,
          "and it matches the July run it is taken from, not the 11 July 1x/1x composition")
    check(rep["nominal_states"] == 500 and rep["nominal_repeat"] == 32
          and rep["nominal_training_samples"] == 16000,
          "500 nominal states repeated 32 times")
    check(rep["recovery_steps"] == 12 and rep["recovery_repeat"] == 2
          and rep["recovery_training_samples"] == 24,
          "the 12-row aligned prefix repeated twice")
    check(rep["aggregate_samples"] == 16024 and built["observations"].shape == (16024, 35)
          and built["actions"].shape == (16024, 2) and built["times"].shape == (16024,),
          f"for an aggregate of 16024 rows, got {rep['aggregate_samples']}")
    check("SELF-DISTILLED" in rep["nominal_label_source"],
          "the nominal block is self-distilled, as July anchors it")
    with np.load(J4.J1_LEAF / "teacher_dataset.npz") as arch:
        t_obs = np.asarray(arch["observations"], dtype=np.float32)
        t_act = np.asarray(arch["actions"], dtype=np.float32)
    masked = t_obs.copy()
    masked[:, [0, 1]] = 0.0
    own = J4.student_mean_actions(masked)
    check(np.array_equal(built["actions"][:500], own),
          "the anchor labels ARE the pinned student's own means, not the teacher's actions")
    check(not np.array_equal(built["actions"][:500], t_act[:500]),
          "and are therefore distinct from the teacher labels")
    check(float(np.abs(own - t_act).max()) > 0.0 and rep["nominal_label_rms"] > 0.0,
          "the self-distilled labels are non-degenerate")
    # BIT-EQUALITY WITH JULY. The comparison is against the July function itself, in its native
    # float32 precision - not against any re-derived reference.
    try:
        sys.path.insert(0, str(BASELINE))
        import target_domain_noise_adaptation as JN
        import target_domain_markov_adaptation as JM
    except Exception as exc:                                     # pragma: no cover
        raise AssertionError(
            "the July modules must be importable to prove bit-equality; run this suite under "
            f"/opt/anaconda3/envs/envCMC-rllib/bin/python ({exc})")
    st = J4.load_parent_state()
    july_logits = JN._forward(st, np.asarray(masked, np.float32))
    ours_logits = J4.july_forward(st, np.asarray(masked, np.float32))
    check(ours_logits.dtype == np.float32 and np.asarray(masked, np.float32).dtype == np.float32,
          "the forward runs in NATIVE float32, as July does")
    check(np.array_equal(july_logits, ours_logits),
          "j4.july_forward is BIT-EQUAL to target_domain_noise_adaptation._forward")
    july_means = JM._actor_means(st, masked)
    check(np.array_equal(july_means, J4.student_mean_actions(masked)),
          "and student_mean_actions is BIT-EQUAL to target_domain_markov_adaptation._actor_means "
          "on the 500 pinned nominal states")
    check(np.array_equal(own, np.asarray(july_means, dtype=np.float32)),
          "so the stored anchor labels are exactly July's means, cast back without loss")
    check(np.allclose(july_logits[:, 2:], -5.29831743, atol=1e-6),
          "and the frozen log-std columns are the expected constant")
    check(np.all(np.isfinite(built["actions"])) and np.all(np.isfinite(built["observations"])),
          "the whole aggregate is finite")
    check(np.array_equal(built["observations"][:500], masked)
          and np.array_equal(built["observations"][500:1000], masked),
          "the nominal states are tiled verbatim, clock columns projected to zero")
    check(np.array_equal(built["actions"][16000:16012], t_act[:12])
          and np.array_equal(built["actions"][16012:16024], t_act[:12]),
          "the recovery labels are the teacher actions of the same 12 steps, tiled twice")
    check(rep["label_rule"].startswith("recovery labels are teacher_actions[step - 1]"),
          "labels are the time-aligned teacher actions")
    check(rep["time_alignment_verified"]["rows_checked"] == 12
          and rep["time_alignment_verified"]["tolerance_s"] == 1e-9,
          "every recovery row's time was checked against both the trace and the dataset")
    # NO minimum-length gate anywhere
    check(not hasattr(J4, "MIN_RETAINED_STEPS"),
          "MIN_RETAINED_STEPS no longer exists: it was never a July gate")
    check("MIN_RETAINED" not in src and "corpus_floor" not in src,
          "and no floor survives in the module source")
    cond = rep["corpus_condition"]
    check(cond["minimum_length_gate"] is None
          and cond["rule"] == "non-empty AND time-aligned, after July-strict truncation",
          "the fail-closed condition is the literal July one: non-empty and aligned")
    check("DESCRIPTIVE" in cond["note"],
          "July's 119/118/119 are recorded as descriptive, never as a threshold")
    amend_rule = amendment["recovery_alignment_rule"]
    check(amend_rule["july_retained_prefixes"]["status"].startswith("DESCRIPTIVE ONLY")
          and "NOT a threshold" in amend_rule["july_retained_prefixes"]["status"],
          "the amendment says the same")
    check("QUARANTINED candidate" in amend_rule["short_prefix_policy"]
          and "closed-loop" in amend_rule["short_prefix_policy"],
          "and that a short aligned prefix yields a quarantined candidate judged by closed-loop")
    # the declared derogation
    der = rep["multistart_derogation"]
    check(der["omitted"] is True and "USER DECISION" in der["why"]
          and der["july_additional_training_samples"] == 8000,
          "the multistart omission is declared as the single explicit derogation")
    check(amendment["user_decisions_recorded"]["multistart_deferred"][
              "july_additional_training_samples"] == 8000,
          "and the amendment records it too")
    expect(lambda: J4.build_recovery_dataset(nominal_repeat=0), J4.J4Error,
           "nominal_repeat must be >= 1")
    expect(lambda: J4.build_recovery_dataset(recovery_repeat=0), J4.J4Error,
           "recovery_repeat must be >= 1")

    check("build_markov_recovery_dataset" in J4.JULY_SOURCES["aggregation"]
          and "aggregate_dagger_traces" not in J4.JULY_SOURCES["aggregation"],
          "the 32/2 self-distilled composition is attributed to build_markov_recovery_dataset")
    check("aggregate_dagger_traces" in J4.JULY_SOURCES["aggregation_not_used"]
          and "TEACHER dataset at 1x" in J4.JULY_SOURCES["aggregation_not_used"],
          "and aggregate_dagger_traces is recorded as the earlier 1x composition, NOT used")
    check("_actor_means" in J4.JULY_SOURCES["nominal_self_distillation"]
          and "float32" in J4.JULY_SOURCES["nominal_self_distillation"],
          "the self-distillation source and its native precision are cited")

    # ---------------------------------------------------------------- preflight -----------------
    tripped: list[str] = []
    real_import = builtins.__import__
    banned = {"torch", "ray", "env_factory", "opensim", "rollout_eval", "gymnasium"}

    def guarded(name, *a, **k):  # noqa: ANN001
        if name.split(".")[0] in banned:
            tripped.append(name)
            raise AssertionError(f"preflight imported {name}")
        return real_import(name, *a, **k)

    builtins.__import__ = guarded
    try:
        pre = J4.preflight()
    finally:
        builtins.__import__ = real_import
    check(tripped == [], f"PROVEN INERT: the preflight imported no heavy stack ({tripped})")
    check(pre["inert"]["training_executed"] is False
          and pre["inert"]["files_written"] is False
          and pre["inert"]["environment_constructed"] is False,
          "and it declares that nothing was trained, written or constructed")
    check(pre["verdict"] == "GO" and pre["blockers"] == [],
          f"the preflight is GO on the literal July condition: {pre['blockers']}")
    check(pre["dataset"]["aggregate_samples"] == 16024,
          "over the 16024-row aggregate")
    check(pre["corpus_condition"]["minimum_length_gate"] is None,
          "with no invented minimum-length threshold")
    check(pre["composition"] == {"nominal_repeat": 32, "recovery_repeat": 2,
                                 "source": "the SELECTED final July Markov run, used coherently; "
                                           "never mixed with the 11 July DAgger 1x/1x composition"},
          "and the composition declared coherently")
    auth = pre["authorisation"]
    check(auth["authorised_by"].startswith("THE USER, explicitly"),
          "the authorisation is attributed to the USER, not the architect")
    check("reviewer, not approver" in auth["authorised_by"],
          "and says the architect is reviewer")
    check(auth["scope"] == "RECOVERY ONLY - ONE J4 fit"
          and auth["progressive_rounds_authorised"] is False
          and auth["j5_authorised"] is False
          and auth["physical_rollout_authorised"] is False,
          "ONE J4 fit is approved; progressive rounds, J5 and any rollout are not")
    sf = auth["soft_fail_accepted_by_user"]
    check(sf["hard_guard_m"] == 0.028 and sf["steps_above_hard_guard_j3"] == 0
          and "accepted by the user" in sf["statement"]
          and "NOT reintroduced as a blocker" in sf["statement"],
          "the J1/J3 soft fail is accepted and is not a blocker here; the 28 mm guard stands")
    check(auth["multistart_derogation"]["omitted"] is True,
          "and the multistart derogation is declared in the preflight too")
    check(pre["actor_contract"]["hard_zero_columns_this_stage"] == [0, 1]
          and pre["actor_contract"]["trainable_controller_columns"] == list(range(25, 35))
          and pre["actor_contract"]["widening"] == "NONE - the actor stays 35D"
          and pre["actor_contract"]["standalone_25d"] == "NONE",
          "the preflight states the recovery mask: clock hard-zero, controller trainable")
    check("ABSENT" in pre["actor_contract"]["contralateral_features"],
          "and that contralateral features stay absent")
    check(all(v == [0, 1] + list(range(25, 35))
              for v in pre["j2_parent"]["zero_columns_by_layer"].values()),
          "the J2 parent still arrives with all twelve base-stage columns at zero")
    diag = pre["dataset"]["discrete_agreement_diagnostics"]
    check(diag["steps_agreeing_on_all_discrete_columns"] == 462
          and diag["longest_contiguous_agreeing_run"]["steps"] == 101
          and "NOT A RELAXATION" in diag["status"],
          "the agreement diagnostics are reported and explicitly marked non-operative")
    check(pre["authorisation"]["physical_rollout_authorised"] is False
          and pre["authorisation"]["revalidation_required_before_any_claim"] is True,
          "no physical rollout is authorised and revalidation is required")

    # ---------------------------------------------------------------- refusals -----------------
    with tempfile.TemporaryDirectory() as td:
        for stage in (None, "", "V26C-J4", "V26C-J3-CLOSED-LOOP"):
            expect(lambda s=stage: J4.fit(authorized_stage=s, out_dir=Path(td) / "x"),
                   J4.J4Error, f"the token {stage!r} must be refused")
        expect(lambda: J4.fit(authorized_stage=J4.STAGE, out_dir=None),
               J4.J4Error, "an implicit output directory must be refused")
        check(not any(Path(td).iterdir()), "no leaf is created by any refused call")
        # no-clobber is refused BEFORE any heavy import, so this never starts a fit
        taken = Path(td) / "taken"
        taken.mkdir()
        e = expect(lambda: J4.fit(authorized_stage=J4.STAGE, out_dir=taken),
                   J4.J4Error, "an existing leaf is never overwritten")
        check("no-clobber" in str(e), "and the refusal says no-clobber")
        check(not any(taken.iterdir()), "the existing leaf is left untouched")
        for nr, rr in ((1, 1), (32, 1), (8, 2), (64, 2)):
            e = expect(lambda a=nr, b=rr: J4.fit(authorized_stage=J4.STAGE,
                                                 out_dir=Path(td) / f"o{a}_{b}",
                                                 nominal_repeat=a, recovery_repeat=b),
                       J4.J4Error, f"the write path must refuse the override {nr}/{rr}")
            check("authorised composition" in str(e) and "No override may be written" in str(e),
                  f"naming the authorised 32/2 composition ({nr}/{rr})")
        check(not (Path(td) / "o1_1").exists() and not (Path(td) / "o64_2").exists(),
              "and no leaf is created for a refused override")
        check(J4.build_recovery_dataset(nominal_repeat=1, recovery_repeat=1)["report"][
                  "aggregate_samples"] == 512,
              "while the BUILDER may still be exercised with other repeats, for testing only")
        with open(os.devnull, "w") as sink, contextlib.redirect_stdout(sink):
            cli = J4.main(["--preflight"])
        check(cli == 0, "the CLI exits zero: the preflight is GO")

    # ------------------------------------------- the pre-write verification, without fitting ----
    lines = src.splitlines()
    def line_of(fragment: str) -> int:
        return next(i for i, ln in enumerate(lines, start=1) if fragment in ln)
    mkdir_at = line_of("out.mkdir(parents=True, exist_ok=False)")
    required_guards = {
        "controller columns non-zero": 'raise J4Error(f"controller-memory columns stayed at zero',
        "clock bit-zero in both aliases": 'raise J4Error(f"the clock columns are not bit-zero in',
        "aliases bit-identical": 'are not bit-identical',
        "log-std head unchanged": 'the frozen log-std head changed in',
        "zero columns exactly the clock": 'after the fit the zero columns are',
    }
    for label, fragment in required_guards.items():
        at = line_of(fragment)
        check(at < mkdir_at,
              f"the guard '{label}' is enforced at line {at}, BEFORE the first write at "
              f"{mkdir_at}")
    for label, fragment in {
            "every fitted parameter is finite":
                'raise J4Error(f"the fitted parameter {key} contains non-finite values")',
            "every reported metric is finite":
                'raise J4Error(f"the metric {label} is not finite',
            "recovery error must improve": '"the recovery error did not improve: "'}.items():
        at = line_of(fragment)
        check(at < mkdir_at,
              f"the guard '{label}' is enforced at line {at}, BEFORE the first write at {mkdir_at}")
    check(line_of("if not (recovery_rmse_after < recovery_rmse_before):") < mkdir_at,
          "the improvement requirement is a strict inequality checked before any write")
    check("A direction, not a magnitude" in src or "not a numeric threshold" in src,
          "and is stated as a direction, with no magnitude prescribed")
    check('"status": "REFERENCE ONLY, never a gate"' in src,
          "while the July shift figures stay a reference, never a gate")
    check(src.count("math.isfinite") >= 1 and "np.isfinite(np.asarray(value))" in src,
          "finiteness is checked on both the parameters and the metrics")
    ver_start = line_of("verification = {")
    check(ver_start < mkdir_at, "and the verification block is assembled before any write")
    for key in ("nominal_self_distillation", "nominal_shift_vs_parent", "recovery_vs_teacher",
                "controller_column_norms", "controller_columns_all_positive",
                "clock_bit_zero_in_both_aliases", "aliases_bit_identical",
                "logstd_head_bit_identical_to_parent", "all_parameters_finite",
                "all_metrics_finite"):
        check(f'"{key}"' in src, f"the verification records {key}")
    for metric in ("rmse_before", "rmse_after", "max_abs", "per_action_rms"):
        check(f'"{metric}"' in src, f"including {metric}")
    check('"pre_write_verification": verification' in src,
          "and the receipt carries the whole verification block")
    # the same quantities are computable now, on the pinned parent, without any fit
    parent_state = J4.load_parent_state()
    for direct, encoder in (("pi.0.0.weight", "pi_encoder.0.weight"),
                            ("pi.0.0.bias", "pi_encoder.0.bias"),
                            ("pi.0.2.weight", "pi_encoder.2.weight"),
                            ("pi.0.2.bias", "pi_encoder.2.bias")):
        check(np.array_equal(parent_state[direct], parent_state[encoder]),
              f"the parent's {direct} and {encoder} aliases are already bit-identical")
    check(np.all(parent_state["pi.1.weight"][2:] == 0.0),
          "the parent's log-std output weights are zero, so the head can be frozen")
    check(float(np.abs(parent_state["pi.0.0.weight"][:, [0, 1]]).max()) == 0.0,
          "and its clock columns are bit-zero to begin with")
    parent_ctrl = {n: float(np.linalg.norm(parent_state["pi.0.0.weight"][:, names.index(n)]))
                   for n in ctrl}
    check(max(parent_ctrl.values()) == 0.0,
          "while ALL ten controller columns start at zero norm: the recovery stage must lift them")

    # the verdict is technical, never a closed-loop qualification
    check('"verdict_kind": "TECHNICAL / OFFLINE"' in src
          and '"closed_loop_qualified": False' in src,
          "a J4 PASS is recorded as TECHNICAL/OFFLINE, not as closed-loop qualification")
    check("asserts NOTHING about gait, penetration or deployability" in src,
          "and says explicitly what it does not assert")
    check('"deployable": False' in src and '"promotion": "NONE"' in src
          and '"next_stage_authorized": False' in src,
          "with deployable false, no promotion and no next stage")

    # ------------------------------------- post-crash finalisation, in TEMP copies only ---------
    check(J4.PIN_J4_PAYLOADS == {
        "rl_module/module_state.pkl":
            "14a3630f757a5da2055eb754f6249fad8e7989a6d5e6c18f526c76415dad31aa",
        "rl_module/class_and_ctor_args.pkl":
            "897e2f13695c52a411d49f957bdaf99ab864411334538703844f1b063857cd02",
        "rl_module/metadata.json":
            "3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12",
        "rl_module/actor_feature_manifest.json":
            "3454a6de085a14510874af8222e266eacac1dd194460e26902ffa29606df3c03",
        "recovery_dataset.npz":
            "28eda638bd5441698611fce7f9d9b65660b3e21581805434f41c57d1a745357e",
        "history.json":
            "e8a07fdfb77656a1a9f3c4d158b949f8a793c015106396015761748d50fb6b87"},
        "the six preserved payloads are frozen by exact hash")
    check(J4.PIN_FAILING_RUNNER
          == "dbf46cd559a1e6da6bf0ceb8ef29874db2fc311a91697b8c0e389958d67ae75d"
          and J4.PIN_ORIGINAL_LOG
          == "88e9312626d23ce49685576e263df1b93c0f4f8d077415cc1d0bdaadb188dbcf"
          and J4.ORIGINAL_EXIT_CODE == 1,
          "the failing runner, the original log and the exit code are frozen too")
    check(J4._sha_file(Path(__file__).resolve().parent / "v26c_j4_recovery.py")
          != J4.PIN_FAILING_RUNNER,
          "and the finaliser is NOT the runner that crashed: the shadowing fix is in place")
    # the shadowing defect itself
    check("for key, head_rows in" in src and "final[key][head_rows]" in src,
          "the log-std loop binds head_rows, never rows")
    fitfn_src = src[src.index("def fit("):src.index("def verify_j4_payloads(")]
    check("for key, rows in" not in fitfn_src,
          "so `rows` is no longer shadowed anywhere in fit()")
    check(fitfn_src.count("rows = len(obs)") == 1
          and '"rows": int(rows)' in fitfn_src,
          "and the aggregate row count still reaches the receipt")

    real_leaf = J4.J4_LEAF
    real_before = {rel: (J4._sha_file(real_leaf / rel),
                         (real_leaf / rel).stat().st_mtime_ns) for rel in J4.PIN_J4_PAYLOADS}
    check(J4.verify_j4_payloads(real_leaf) == J4.PIN_J4_PAYLOADS,
          "the real preserved leaf verifies against those pins")
    check(not (real_leaf / J4.RECEIPT_NAME).exists()
          and not (real_leaf / J4.FORENSIC_LOG_NAME).exists(),
          "and has NOT been finalised: neither the receipt nor the forensic log exists")

    with tempfile.TemporaryDirectory() as td3:
        work = Path(td3) / "leaf"
        shutil.copytree(real_leaf, work)
        before = {rel: (J4._sha_file(work / rel), (work / rel).stat().st_mtime_ns)
                  for rel in J4.PIN_J4_PAYLOADS}
        # no training may be reachable: trip on torch and on the module's own fit/optimiser
        tripped2: list[str] = []
        real_import2 = builtins.__import__

        def guarded2(name, *a, **k):  # noqa: ANN001
            if name.split(".")[0] in {"torch", "ray", "env_factory", "opensim"}:
                tripped2.append(name)
                raise AssertionError(f"finalisation imported {name}")
            return real_import2(name, *a, **k)

        called: list[str] = []
        saved_fit = J4.fit
        J4.fit = lambda *a, **k: called.append("fit")     # type: ignore[assignment]
        # instrument every text write so the receipt can be proven to be written exactly ONCE
        writes: list[str] = []
        real_write_text = Path.write_text

        def counting_write_text(self, *a, **k):           # noqa: ANN001
            writes.append(self.name)
            return real_write_text(self, *a, **k)

        Path.write_text = counting_write_text             # type: ignore[assignment]
        builtins.__import__ = guarded2
        try:
            rec = J4.finalize_existing(authorized_stage=J4.STAGE, leaf=work)
        finally:
            builtins.__import__ = real_import2
            Path.write_text = real_write_text             # type: ignore[assignment]
            J4.fit = saved_fit
        check(writes.count(J4.RECEIPT_NAME) == 1,
              f"the receipt is written EXACTLY once ({writes})")
        check(writes == [J4.RECEIPT_NAME],
              f"and it is the only text file finalisation writes ({writes})")
        check(called == [] and tripped2 == [],
              f"PROVEN: finalisation called no fit and imported no training stack ({called}, "
              f"{tripped2})")
        after = {rel: (J4._sha_file(work / rel), (work / rel).stat().st_mtime_ns)
                 for rel in J4.PIN_J4_PAYLOADS}
        check(after == before, "the six payloads are byte- AND mtime-identical afterwards")
        produced = sorted(str(q.relative_to(work)) for q in work.rglob("*") if q.is_file())
        check(produced == sorted(list(J4.PIN_J4_PAYLOADS) + list(J4.FINALISATION_WRITES)),
              f"exactly two files were added: {sorted(J4.FINALISATION_WRITES)}")
        check(J4._sha_file(work / J4.FORENSIC_LOG_NAME) == J4.PIN_ORIGINAL_LOG,
              "the forensic log copy is bit-identical to the original run log")
        # outputs_sha256: the six frozen payloads + the log, never the receipt itself
        outs = rec["outputs_sha256"]
        check(len(outs) == 7
              and set(outs) == set(J4.PIN_J4_PAYLOADS) | {J4.FORENSIC_LOG_NAME},
              f"outputs_sha256 holds all seven produced files, got {sorted(outs)}")
        for rel, pin in J4.PIN_J4_PAYLOADS.items():
            check(outs[rel] == pin == J4._sha_file(work / rel),
                  f"outputs_sha256[{rel}] is the frozen hash and matches the file")
        check(outs[J4.FORENSIC_LOG_NAME] == J4.PIN_ORIGINAL_LOG,
              "and the log entry is the pinned original hash")
        check(J4.RECEIPT_NAME not in outs
              and rec["outputs_sha256_excludes"]["file"] == J4.RECEIPT_NAME,
              "the receipt does not hash itself, and says so")
        on_disk = json.loads((work / J4.RECEIPT_NAME).read_text())
        check(on_disk["outputs_sha256"] == outs
              and on_disk["post_crash_finalisation"]["payload_sha256_after"]
              == J4.PIN_J4_PAYLOADS
              and on_disk["post_crash_finalisation"]["payload_sha256_unchanged"] is True,
              "the receipt ON DISK is already complete: no second write was needed")
        # the receipt
        check(rec["verdict"] == "PASS"
              and rec["verdict_kind"] == "TECHNICAL/OFFLINE - POST-CRASH FINALIZED",
              "the verdict is a post-crash finalised technical PASS")
        pcf = rec["post_crash_finalisation"]
        check(pcf["original_exit_code"] == 1 and pcf["training_executions"] == 1
              and pcf["retry"] is False and pcf["payloads_rewritten"] is False
              and pcf["training_invoked_during_finalisation"] is False
              and pcf["optimizer_constructed_during_finalisation"] is False,
              "it declares one training execution, no retry and no rewrite")
        check("not 'slice'" in pcf["original_exception"] and "rows" in pcf["original_failure_site"],
              "and records the exact exception and its site")
        check(pcf["failing_runner_sha256"] == J4.PIN_FAILING_RUNNER
              and pcf["finalizer_sha256"] != J4.PIN_FAILING_RUNNER,
              "with both the failing runner and the finaliser hashed")
        check(pcf["payload_sha256_before"] == pcf["payload_sha256_after"] == J4.PIN_J4_PAYLOADS
              and pcf["payload_sha256_unchanged"] is True,
              "and the payload hashes recorded identical before and after")
        check(rec["closed_loop_qualified"] is False and rec["deployable"] is False
              and rec["promotion"] == "NONE" and rec["next_stage_authorized"] is False
              and rec["critic_trained"] is False and rec["ppo_updates"] == 0,
              "no closed-loop qualification, no deployment, no promotion, no critic")
        for item in ("closed-loop rollout", "revalidation", "J5", "progressive recovery rounds",
                     "deployment", "promotion", "critic", "PPO", "ex-novo", "retry"):
            check(item in rec["not_authorised_by_this_receipt"],
                  f"{item} is explicitly not authorised by the receipt")
        # the reconstruction is real
        check(rec["selection"]["epochs_run"] == 69 and rec["selection"]["best_epoch"] == 9
              and rec["selection"]["stopped_by"] == "patience"
              and rec["selection"]["stale_epochs_at_stop"] == 60,
              f"selection reconstructed from history.json: {rec['selection']}")
        check(rec["split"]["rows"] == 16024 and rec["split"]["reconstructed"] is True
              and rec["split"]["training"] + rec["split"]["validation"] == 16024,
              "the split is reconstructed deterministically over 16024 rows")
        pw = rec["pre_write_verification"]
        check(pw["recovery_vs_teacher"]["rmse_after"] < pw["recovery_vs_teacher"]["rmse_before"]
              and pw["recovery_vs_teacher"]["improved"] is True,
              "the recovery error improved")
        check(pw["controller_columns_all_positive"] is True
              and min(pw["controller_column_norms"].values()) > 0.0
              and len(pw["controller_column_norms"]) == 10,
              "all ten controller columns have positive norm")
        check(pw["zero_columns_after_fit"] == [0, 1] and pw["aliases_bit_identical"] is True
              and pw["logstd_head_bit_identical_to_parent"] is True
              and pw["all_parameters_finite"] is True and pw["all_metrics_finite"] is True,
              "and every structural invariant holds")
        check(rec["dataset_sha256"] == J4.preflight()["dataset_sha256"],
              "the stored dataset content-hash equals the preflight's")
        # the payload IS the restored best state
        bs = pw["restored_best_state"]
        check(bs["restored_best_state_verified"] is True
              and rec["selection"]["restored_best_state_verified"] is True,
              "the receipt records that the restored best state was verified")
        check(bs["rtol"] == J4.BEST_STATE_RTOL == 1e-5
              and bs["atol"] == J4.BEST_STATE_ATOL == 1e-12
              and "float32" in bs["tolerance_rationale"],
              "with an explicit, justified float32 tolerance")
        check(abs(bs["recorded_best_validation_mse"] - 1.3680861457032734e-06) < 1e-18,
              "against the recorded best validation MSE of the real run")
        check(bs["relative_difference"] < bs["rtol"]
              and bs["relative_difference"] < 1e-6
              and bs["validation_rows"] == rec["split"]["validation"],
              f"and the reconstruction agrees to {bs['relative_difference']:.3e} relative")
        check(np.isclose(bs["reconstructed_validation_mse"],
                         bs["recorded_best_validation_mse"],
                         rtol=J4.BEST_STATE_RTOL, atol=J4.BEST_STATE_ATOL),
              "which is what the runner asserts with np.isclose")
        # the comparison would REJECT a module that is not the best state: a perturbed payload
        # moves the validation MSE far outside the tolerance
        with np.load(work / "recovery_dataset.npz") as arch:
            o_chk = np.asarray(arch["observations"], np.float32)
            a_chk = np.asarray(arch["actions"], np.float32)
        import pickle as _pickle
        with (work / "rl_module/module_state.pkl").open("rb") as fh:
            st_chk = {k: np.asarray(v, np.float32) for k, v in _pickle.load(fh).items()}
        perturbed = dict(st_chk)
        perturbed["pi.1.bias"] = st_chk["pi.1.bias"].copy()
        perturbed["pi.1.bias"][0] += np.float32(0.01)
        rows_chk = len(o_chk)
        rng_chk = np.random.default_rng(123)
        perm_chk = rng_chk.permutation(rows_chk)
        v_chk = np.sort(perm_chk[:int(round(rows_chk * 0.2))])
        bad_mse = float(np.mean((np.asarray(
            J4.student_mean_actions(o_chk[v_chk], perturbed), np.float32) - a_chk[v_chk]) ** 2))
        check(not np.isclose(bad_mse, bs["recorded_best_validation_mse"],
                             rtol=J4.BEST_STATE_RTOL, atol=J4.BEST_STATE_ATOL),
              f"a payload perturbed by 0.01 is rejected by the same comparison ({bad_mse:.3e})")

        # re-finalisation is refused
        e = expect(lambda: J4.finalize_existing(authorized_stage=J4.STAGE, leaf=work),
                   J4.J4Error, "a finalised leaf is never finalised twice")
        check("no-clobber" in str(e), "the refusal says no-clobber")
        for stage in (None, "", "V26C-J4"):
            expect(lambda st=stage: J4.finalize_existing(authorized_stage=st, leaf=work),
                   J4.J4Error, f"finalisation refuses the token {stage!r}")

    # a tampered payload must abort, and the missing / extra cases too
    with tempfile.TemporaryDirectory() as td4:
        work2 = Path(td4) / "leaf"
        shutil.copytree(real_leaf, work2)
        (work2 / "history.json").write_text("[]", encoding="utf-8")
        e = expect(lambda: J4.finalize_existing(authorized_stage=J4.STAGE, leaf=work2),
                   J4.J4Error, "a tampered payload must abort")
        check("history.json changed" in str(e), "naming the tampered payload")
        check(not (work2 / J4.RECEIPT_NAME).exists(), "and nothing is written")
        shutil.rmtree(work2)
        shutil.copytree(real_leaf, work2)
        (work2 / "recovery_dataset.npz").unlink()
        expect(lambda: J4.finalize_existing(authorized_stage=J4.STAGE, leaf=work2),
               J4.J4Error, "a missing payload must abort")
        shutil.rmtree(work2)
        shutil.copytree(real_leaf, work2)
        (work2 / "stray.txt").write_text("x", encoding="utf-8")
        expect(lambda: J4.finalize_existing(authorized_stage=J4.STAGE, leaf=work2),
               J4.J4Error, "an extra file in the leaf must abort")
        shutil.rmtree(work2)
        shutil.copytree(real_leaf, work2)
        saved_logpin = J4.PIN_ORIGINAL_LOG
        J4.PIN_ORIGINAL_LOG = "0" * 64
        e = expect(lambda: J4.finalize_existing(authorized_stage=J4.STAGE, leaf=work2),
                   J4.J4Error, "a changed original log must abort")
        check("original run log changed" in str(e), "naming the log")
        J4.PIN_ORIGINAL_LOG = saved_logpin
        check(not (work2 / J4.FORENSIC_LOG_NAME).exists(),
              "and no forensic copy is left behind by a refused finalisation")

    # ------------------------------------------- the hardened selection reconstruction ----------
    good_history = json.loads((real_leaf / "history.json").read_text())
    sel = J4._reconstruct_selection(good_history)
    check(sel["epochs_run"] == 69 and sel["best_epoch"] == 9
          and sel["stale_epochs_at_stop"] == 60 and sel["patience_fired"] is True
          and sel["stopped_by"] == "patience" and 1 <= sel["best_epoch"] <= sel["epochs_run"],
          f"the real history replays to {sel['best_epoch']}/{sel['epochs_run']}")
    def hist(entries):
        return [{"epoch": float(i), "train_loss": 1.0, "validation_mse": v}
                for i, v in entries]
    for label, bad in (
            ("an empty history", []),
            ("a non-list history", {"epoch": 1.0}),
            ("a non-mapping entry", [1.0]),
            ("a missing field", [{"epoch": 1.0, "train_loss": 1.0}]),
            ("a non-integral epoch", [{"epoch": 1.5, "train_loss": 1.0, "validation_mse": 1.0}]),
            ("a non-contiguous history",
             hist([(1, 1.0), (3, 0.5)])),
            ("a non-finite validation_mse",
             [{"epoch": 1.0, "train_loss": 1.0, "validation_mse": float("nan")}]),
            ("a non-finite train_loss",
             [{"epoch": 1.0, "train_loss": float("inf"), "validation_mse": 1.0}]),
            ("a history longer than the budget",
             hist([(i, 1.0 / i) for i in range(1, 402)])),
            ("an early stop with too few stale epochs",
             hist([(i, 1.0 / i) for i in range(1, 12)])),
            ("an early stop with too many stale epochs",
             hist([(1, 1.0)] + [(i, 2.0) for i in range(2, 70)])),
    ):
        expect(lambda h=bad: J4._reconstruct_selection(h), J4.J4Error,
               f"{label} must be refused")
    # patience reached BEFORE the end is impossible - the loop would have stopped there
    early = hist([(1, 1.0)] + [(i, 2.0) for i in range(2, 62)] + [(62, 0.5)]
                 + [(i, 2.0) for i in range(63, 123)])
    e = expect(lambda: J4._reconstruct_selection(early), J4.J4Error,
               "patience reached mid-history, then reset by an improvement, is impossible")
    check("the history is impossible" in str(e) and "stale epochs are reached at epoch 61" in str(e),
          f"and the refusal names the epoch where the loop must have stopped: {str(e)[:90]}")
    full_early = hist([(1, 1.0)] + [(i, 2.0) for i in range(2, 401)])
    e = expect(lambda: J4._reconstruct_selection(full_early), J4.J4Error,
               "a full-budget history with patience reached long before the end is impossible")
    check("continued to epoch 400" in str(e), "even when the length equals the budget")
    exact = hist([(1, 1.0)] + [(i, 2.0) for i in range(2, 62)])
    ok = J4._reconstruct_selection(exact)
    check(ok["epochs_run"] == 61 and ok["best_epoch"] == 1
          and ok["stale_epochs_at_stop"] == 60 and ok["stopped_by"] == "patience",
          "exactly 60 stale epochs below the budget is accepted")
    full = hist([(i, 1.0 / i) for i in range(1, 401)])
    okf = J4._reconstruct_selection(full)
    check(okf["epochs_run"] == 400 and okf["stopped_by"] == "epoch budget"
          and okf["patience_fired"] is False,
          "a full-budget run stops on the budget")
    boundary = hist([(i, 1.0 / i) for i in range(1, 341)] + [(i, 9.0) for i in range(341, 401)])
    okb = J4._reconstruct_selection(boundary)
    check(okb["epochs_run"] == 400 and okb["patience_fired"] is True
          and okb["stopped_by"] == "patience at the final epoch, which is also the epoch budget",
          "and the boundary case where both conditions coincide is documented, not guessed")

    # ------------------------------------------- the CLI forwards --original-log ----------------
    seen: dict[str, Any] = {}
    saved_fin = J4.finalize_existing
    J4.finalize_existing = (lambda **kw: seen.update(kw)  # type: ignore[assignment]
                            or {"verdict": "PASS", "verdict_kind": "x",
                                "post_crash_finalisation": {"files_written": [],
                                                            "original_log_sha256": "y"},
                                "outputs_sha256": {}})
    try:
        with open(os.devnull, "w") as sink, contextlib.redirect_stdout(sink):
            J4.main(["--finalize-existing", "--authorized-stage", J4.STAGE,
                     "--out-dir", "/tmp/leaf-x", "--original-log", "/tmp/log-y"])
    finally:
        J4.finalize_existing = saved_fin
    check(seen.get("original_log") == Path("/tmp/log-y")
          and seen.get("leaf") == Path("/tmp/leaf-x")
          and seen.get("authorized_stage") == J4.STAGE,
          f"the CLI forwards --original-log, --out-dir and the token ({seen})")
    seen.clear()
    J4.finalize_existing = (lambda **kw: seen.update(kw)  # type: ignore[assignment]
                            or {"verdict": "PASS", "verdict_kind": "x",
                                "post_crash_finalisation": {"files_written": [],
                                                            "original_log_sha256": "y"},
                                "outputs_sha256": {}})
    try:
        with open(os.devnull, "w") as sink, contextlib.redirect_stdout(sink):
            J4.main(["--finalize-existing", "--authorized-stage", J4.STAGE])
    finally:
        J4.finalize_existing = saved_fin
    check(seen.get("original_log") is None and seen.get("leaf") is None,
          "and omitting them falls back to the incident-specific defaults")
    check(J4.ORIGINAL_LOG.name == "j4_fit.log" and J4.PIN_ORIGINAL_LOG
          == "88e9312626d23ce49685576e263df1b93c0f4f8d077415cc1d0bdaadb188dbcf",
          "which remain pinned by hash")

    after_real = {rel: (J4._sha_file(real_leaf / rel),
                        (real_leaf / rel).stat().st_mtime_ns) for rel in J4.PIN_J4_PAYLOADS}
    check(after_real == real_before,
          "MEASURED: the REAL leaf is byte- and mtime-identical after the whole suite")
    check(not (real_leaf / J4.RECEIPT_NAME).exists()
          and not (real_leaf / J4.FORENSIC_LOG_NAME).exists(),
          "MEASURED: the real leaf is still NOT finalised")

    # ---------------------------------------------------------------- static guarantees ---------
    check(not any(i.lower().startswith("v26b") for i in ids),
          "the recovery module references no v26b identifier")
    imported = {n.names[0].name for n in ast.walk(tree) if isinstance(n, ast.Import)}
    imported |= {(n.module or "") for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)}
    check(not any(m.lower().startswith("v26b") for m in imported),
          "and imports no v26b module")
    check(not any("runs/training" in v for v in J4.PIN_INPUTS.values())
          and not any("runs/training" in k for k in J4.PIN_INPUTS),
          "no July run is an operational input: every pinned input is a V26C artefact")
    for name in ("sigma", "exploration_noise", "PPOConfig", "train_ppo", "critic", "value_head",
                 "forward_exploration", "make_cmc_env", "subprocess"):
        check(name not in ids, f"the module never references {name}")
    heavy = {"torch", "ray", "env_factory", "rollout_eval", "opensim"}
    top = {n.names[0].name.split(".")[0] for n in tree.body if isinstance(n, ast.Import)}
    top |= {(n.module or "").split(".")[0] for n in tree.body if isinstance(n, ast.ImportFrom)}
    check(not (top & heavy), f"and imports no heavy module at file scope ({sorted(top & heavy)})")
    fitfn = next(f for f in ast.walk(tree) if isinstance(f, ast.FunctionDef) and f.name == "fit")
    inside = {n.names[0].name.split(".")[0] for n in ast.walk(fitfn) if isinstance(n, ast.Import)}
    check("torch" in inside, "torch is imported inside fit() only")
    # ONE J4 execution has run. Its leaf is the authorised one and holds ONLY the six payloads.
    leaves = sorted(q.name for q in J4.OUT_ROOT.iterdir()) if J4.OUT_ROOT.is_dir() else []
    check(leaves == ["j4_recovery_v26c_2026-08-26_r1"],
          f"MEASURED: exactly the ONE authorised J4 leaf exists; found {leaves}")
    check(sorted(str(q.relative_to(J4.J4_LEAF)) for q in J4.J4_LEAF.rglob("*") if q.is_file())
          == sorted(J4.PIN_J4_PAYLOADS),
          "MEASURED: it holds exactly the six preserved payloads and nothing else")

    print(json.dumps({"selftest": "PASS", "checks": CHECKS}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
