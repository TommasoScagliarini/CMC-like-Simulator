"""Tests for the V26C J0 read-only audit. No fit, no rollout, no collection, no promotion."""
from __future__ import annotations
import inspect, io, json, sys, tokenize
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26c_j0_audit as J  # noqa: E402

CHECKS = 0
def check(c, w):
    global CHECKS; assert c, w; CHECKS += 1


def main() -> int:
    src = (HERE / "v26c_j0_audit.py").read_text()

    # --- the module is incapable of acting -------------------------------------------------------
    ids = {t.string for t in tokenize.generate_tokens(io.StringIO(src).readline)
           if t.type == tokenize.NAME}
    check("july_base_stage" in ids and "gate_reconciliation" in ids, "the tokeniser sees real names")
    for name in ("subprocess", "torch", "fit_masked", "adapt_actor", "rollout_eval", "train_ppo",
                 "PPOConfig", "dagger", "DAgger", "collect_teacher_dataset"):
        check(name not in ids, f"the audit never references {name}: it cannot fit, roll out, "
                               "collect, train or promote")
    check("open(" not in src.replace('.open("rb")', "").replace('.open("wb")', ""),
          "no bare file writes outside the receipt path")
    check("clobber" in src or "exists()" in src, "the receipt is written no-clobber")

    # --- TWO stages, correctly separated ---------------------------------------------------------
    base = J.july_base_stage()
    check(base["run"] == "target_domain_imitation_no_controller_memory_2026-07-13",
          "the BASE stage is the no-controller-memory run")
    check(base["dataset"]["samples"] == 500 and base["dataset"]["n_actor"] == 25,
          "500 pairs, n_actor 25")
    check("PRESCRIBED TEACHER" in base["dataset"]["provenance"],
          "the base labels are the PRESCRIBED TEACHER, not self-anchors")
    check(base["fit"]["scope"] == "the ENTIRE mean actor", "the base stage fits the whole actor")
    check(base["fit"]["split"]["training_samples"] == 400
          and base["fit"]["split"]["validation_samples"] == 100
          and base["fit"]["split"]["kind"] == "random",
          "random 80/20 split, 400/100")
    check(base["fit"]["epochs"]["requested"] == 400 and base["fit"]["epochs"]["best_epoch"] == 183,
          "400 epochs max, best 183")
    check(abs(base["fit"]["adapted_prediction"]["rmse"] - 0.011782177113774129) < 1e-15,
          "adapted RMSE 0.011782177113774129")
    hp = base["fit"]["hyperparameters"]
    check((hp["seed"], hp["batch_size"], hp["learning_rate"], hp["patience"],
           hp["clip_weight"], hp["logstd_weight"], hp["anchor_weight"], hp["validation_fraction"])
          == (123, 64, 3e-4, 60, 1.0, 0.1, 1e-05, 0.2),
          "the base-stage hyperparameters are recovered verbatim, and they are exactly the set the "
          "V26B branch had pinned as the July protocol")
    check(base["teacher_rollout_gate"]["gate_pass"] is True
          and base["teacher_rollout_gate"]["steps"] == 500
          and base["teacher_rollout_gate"]["max_penetration_m"] < 0.025,
          "the teacher rollout itself passed its gate: 500 steps, penetration below 0.025 m")
    check(base["parent"]["W1_shape"] == [256, 25] and base["parent"]["zero_columns"] == [0, 1],
          "July's base parent is a genuinely 25-wide network with the clock columns at zero")

    mk = J.july_markov_stage()
    check(mk["run"] == J.JULY_SELECTED_RUN, "the MARKOV stage is the selected run")
    check(mk["dataset"]["nominal_self_anchors"] == 16000
          and mk["dataset"]["phase_aligned_recovery"] == 712
          and mk["dataset"]["alternative_start_teacher_samples"] == 8000
          and mk["dataset"]["aggregate_samples"] == 24712,
          "16000 self-anchors + 712 phase-aligned recovery + 8000 multistart teacher = 24712")
    check("OWN policy_action_mean" in mk["dataset"]["nominal_label_source"],
          "the markov nominal labels are the source actor's OWN means, not a teacher")
    check(mk["fit"]["train_full_actor"] is True
          and "ENTIRE mean network" in mk["fit"]["scope"],
          "the selected markov stage adapts the WHOLE mean network, not only the ten columns")
    check(mk["parent"]["zero_columns"] == list(J.MASKED_COLUMNS),
          "its parent has columns 0,1 and 25..34 at exactly zero")

    # --- the two gate levels are reconciled, not conflated ---------------------------------------
    cen = J.july_markov_census()
    check(cen["strict_offline_gate_pass"] == [J.JULY_OFFLINE_STRICT_PASS_RUN],
          "only r96 passes the strict OFFLINE gate")
    check(cen["operationally_selected"] == J.JULY_SELECTED_RUN,
          "but the operationally selected run is full_r32_alt8")
    check("MUST NOT be used" in cen["warning"],
          "the census warns that run_summary.gate_pass alone misidentifies the July stage")
    gr = J.gate_reconciliation()
    check(gr["level_2_selection_gate"]["pass"] is True,
          "the selection gate passes for the selected actor")
    check("strict_offline_single_sample_shift_gate"
          in gr["level_2_selection_gate"]["criteria_false_but_not_blocking"],
          "even though the strict offline criterion inside it is false")
    check("Closed-loop start and exploration robustness are the primary warm-start gate"
          in gr["level_2_selection_gate"]["rationale_verbatim"],
          "the July rationale is quoted verbatim, not paraphrased")
    check(gr["decision"]["decision"] == "SELECT_MARKOV35_FOR_H0_H1_WITH_WARM_CRITIC"
          and gr["decision"]["training_ready"] is True
          and gr["decision"]["digest_matches_pin"] is True,
          "decision, training_ready and the selected digest a0801a9e... all check out")
    det = gr["closed_loop_evidence"]["deterministic"]
    sto = gr["closed_loop_evidence"]["stochastic_sigma_0_005"]
    check(len(det) == 3 and all(r["pass"] for r in det)
          and all(r["steps"] == 500 for r in det)
          and all(r["max_penetration_m"] < 0.025 for r in det)
          and all(r["clipped_steps"] == 0 for r in det),
          "3/3 deterministic starts: 500 steps, below the guard, zero clipping")
    check(len(sto) == 3 and all(r["pass"] for r in sto),
          "3/3 stochastic sigma 0.005 seeds")
    check(gr["root_cause_recorded_by_july"]["architecture_rejected"] is False
          and gr["root_cause_recorded_by_july"]["adaptation_protocol_failure"] is True,
          "July recorded the root cause as a protocol failure, NOT an architecture rejection")
    check("rejected rather than weakening the 0.025 m safety threshold" in gr["reconciliation"],
          "and the reconciliation records that the safety threshold was never weakened")

    # --- parent chain --------------------------------------------------------------------------
    pe = J.parent_equivalence()
    check(pe["v26_august_parent"]["matches_pin"] and pe["v26_august_parent"]["W1_shape"] == [256, 39],
          "the exclusive parent is the V26 August 39D imitation checkpoint")
    check(pe["b0_35d_masked"]["matches_pin"] and pe["v1_35d_transplant"]["matches_pin"],
          "V1 and B0 match their pins")
    check(pe["b0_is_v1_with_controller_columns_zeroed"] is True,
          "B0 is exactly V1 with columns 25..34 zeroed, everything else identical")
    check(pe["b0_matches_july_markov_parent_zero_set"] is True,
          "and its zero-column set equals July's markov parent: 0,1 and 25..34")
    check("no separate 25D network, no widening" in pe["declared_deviation"],
          "the 35D-masked-instead-of-25D deviation is declared explicitly")

    # --- the replay plan invents no default ------------------------------------------------------
    rp = J.replay_plan()
    check(rp["parent"]["exclusive"].endswith("rl_module_best")
          and rp["parent"]["no_july_checkpoint_or_dataset_as_operational_parent"] is True,
          "the operational parent is V26 August only")
    check(rp["stage_J2_base_fit"]["hyperparameters_recovered_verbatim"] == hp,
          "the replay reuses the recovered base hyperparameters verbatim, none invented")
    # --- July collection semantics are PINNED, not left to defaults -----------------------------
    cs = rp["stage_J1_collection"]["collection_semantics_pinned"]
    check(cs["seed"] == 123 and cs["teacher_lookahead_s"] == 0.0
          and cs["action_noise_sigma"] == [0.0, 0.0] and cs["action_noise_hold_steps"] == 1,
          "seed 123, lookahead 0.0, action_noise_sigma [0.0, 0.0], hold_steps 1 are pinned")
    check(cs["action_noise_hold_duration_s"] == 0.01,
          "and the hold duration 0.01 s that accompanies them")
    check("NOT DEFAULTS" in cs["status"] and "teacher_summary.json" in cs["provenance"],
          "they are declared July artefact values with their provenance, not defaults")
    # cross-check against the July artefacts themselves, not against a restated constant
    check(base["teacher_rollout_gate"]["steps"] == 500, "July's teacher rollout ran 500 steps")
    jt = json.loads((J.TG / "runs" / "training" / J.JULY_BASE_RUN
                     / "teacher_summary.json").read_text())
    check(jt["action_noise_sigma"] == cs["action_noise_sigma"]
          and jt["action_noise_hold_steps"] == cs["action_noise_hold_steps"]
          and jt["teacher_lookahead_s"] == cs["teacher_lookahead_s"],
          "the pinned semantics match teacher_summary.json field for field")
    check(jt["action_noise_realized_rms"] == [0.0, 0.0],
          "MEASURED: the July teacher rollout was strictly noiseless, realised RMS exactly zero")

    subs = rp["stage_J1_collection"]["must_not_substitute"]
    check(any("1500" in s for s in subs) and any("LOCO" in s for s in subs)
          and any("u_IK" in s for s in subs),
          "the 1500 anchors, LOTO/LOCO and the u_IK bridge are explicitly excluded")
    check(rp["stage_J4_markov"]["not_in_scope_now"] is True
          and "sigma is deferred" in rp["stage_J4_markov"]["open_choice"],
          "the markov stage is out of scope now and sigma is not assumed")
    todo = rp["future_todo_non_operational"]
    check(len(todo["items"]) == 5 and "not deleted" in todo["note"],
          "LOTO/LOCO/B1R1/B1R2/B1R2-A/B1R2-B are preserved as non-operational TODOs")
    check("USER holds exclusive authority" in rp["authority"],
          "user authority over deviations is recorded")

    # --- J1 and J3 gates are DISTINCT, and both come from the fixed V26B source -------------------
    cg = J.current_gate_contract()
    j1 = cg["j1_teacher_collection_gate"]
    j3 = cg["j3_actor_closed_loop_gate"]
    check(j1["criteria"] == J.V26B_COMMON_GATE,
          "J1 binds the COMMON gate: integrity, runtime, safety, contract")
    check(j1["kinematic_quality_applies"] is False and "dataset integrity" in j1["why_not"],
          "and NOT the kinematic gate, because that qualifies an actor's gait, not a dataset")
    check("kinematic_quality" not in j1["criteria"],
          "no kinematic criterion leaks into the J1 acceptance gate")
    check(j3["criteria"] == J.V26B_CLOSED_LOOP_GATE
          and j3["kinematic_quality"] == J.V26B_KINEMATIC_GATE,
          "J3 binds the COMMON gate PLUS the full kinematic quality")
    for k in J.V26B_COMMON_GATE:
        check(j3["criteria"][k] == J.V26B_COMMON_GATE[k],
              f"J3 inherits the common criterion {k} unchanged")
    # the kinematic values must equal the already-fixed source, read from it directly
    sys.path.insert(0, str(HERE.parent / "v26b_bridge_2026-08-24"))
    import v26b_b1_base_fit as B1  # noqa: E402
    src_gates = B1.declared_closed_loop_gates()
    check(J.V26B_KINEMATIC_GATE == src_gates["kinematic_quality"],
          "the kinematic gate equals v26b_b1_base_fit.declared_closed_loop_gates verbatim")
    check(J.V26B_COMMON_GATE["max_penetration_m_max"] == src_gates["penetration_max_m"] == 0.020
          and J.V26B_COMMON_GATE["valid_cycles_min"] == src_gates["valid_cycle_count_min"] == 2
          and J.V26B_COMMON_GATE["resync_count_max"] == src_gates["resync_count_max"] == 1
          and J.V26B_COMMON_GATE["steps_required"] == src_gates["completion"]["steps"] == 500,
          "and the common thresholds equal that same source, none relaxed")
    for c in src_gates["critical_counters_zero"]:
        check(J.V26B_COMMON_GATE[f"{c}_max"] == 0,
              f"the critical counter {c} must be exactly zero")
    check(J.V26B_KINEMATIC_GATE["ankle_min_rad"] == -0.03
          and J.V26B_KINEMATIC_GATE["ankle_amplitude_min_rad"] == 0.30
          and J.V26B_KINEMATIC_GATE["knee_amplitude_min_rad"] == 0.60
          and J.V26B_KINEMATIC_GATE["knee_strictly_flexed"] is True
          and J.V26B_KINEMATIC_GATE["within_bounds"] is True,
          "ankle <= -0.03 rad, ankle ROM >= 0.30, knee ROM >= 0.60, knee strictly flexed, in bounds")
    check(-0.0099 > J.V26B_KINEMATIC_GATE["ankle_min_rad"],
          "MEASURED: an ankle minimum of -0.0099 rad FAILS the gate, since -0.0099 > -0.03")
    check("-0.0099" in j3["worked_example"] and "FAILS" in j3["worked_example"],
          "and that worked example is recorded in the receipt")
    # action_clipped_steps is diagnostic, never binding
    check(J.DIAGNOSTIC_NOT_BINDING == ("action_clipped_steps",),
          "action_clipped_steps is declared diagnostic")
    check("action_clipped_steps" not in json.dumps(J.V26B_COMMON_GATE)
          and "action_clipped_steps" not in json.dumps(J.V26B_CLOSED_LOOP_GATE),
          "and appears in NO binding gate")
    check("already-approved source" in cg["diagnostics_not_binding"]["rule"],
          "promoting it would require an already-approved source, which does not exist")
    crit = j3["criteria"]
    check(cg["current_runtime_thresholds"]["grf_soft_threshold_m"] == 0.020
          and cg["current_runtime_thresholds"]["grf_hard_termination_m"] == 0.028
          and cg["current_runtime_thresholds"]["both_must_be_recorded_by_the_collector"] is True,
          "the current runtime carries soft 0.020 m and hard termination 0.028 m, both recorded")
    jh = cg["july_historical_only"]
    check(jh["guard_m"] == 0.025 and abs(jh["teacher_max_penetration_m"] - 0.02294380435912411) < 1e-15,
          "July's 0.025 m guard and 22.94 mm teacher penetration are kept as historical evidence")
    check("NOT ISOMETRIC" in jh["status"] and "never be reused as a current pass criterion"
          in jh["warning"],
          "and are explicitly marked non-isometric, never reusable as a current criterion")
    check(all(p > 0.020 for p in jh["selected_actor_closed_loop_max_penetration_m"]),
          "MEASURED: July's selected actor sat between 0.0233 and 0.0246 m, ABOVE the current "
          "0.020 m soft threshold - so July's closed-loop success does not transfer numerically")
    rp_gate = J.replay_plan()["stage_J3_closed_loop"]["gate"]
    check(rp_gate == crit, "J3 in the replay plan uses the binding gate, not a July number")
    j1_gate = J.replay_plan()["stage_J1_collection"]["gate"]
    check(j1_gate["binding"] == J.V26B_COMMON_GATE
          and j1_gate["kinematic_quality_applies"] is False
          and j1_gate["july_not_isometric"] is True,
          "J1 binds the COMMON gate only - not the kinematic one - and flags July as non-isometric")
    check(j1_gate["binding"] != crit,
          "and it is genuinely a different, weaker set than J3's: the two stages do not share a "
          "single gate")
    check("0.025" not in json.dumps(rp_gate) and "0.025" not in json.dumps(j1_gate["binding"]),
          "no July threshold leaks into a binding criterion")

    # --- NEGATIVE CONTROL: the historical builder is incomplete for v3 ---------------------------
    cc = J.collector_contract()
    nc = cc["negative_control"]
    check(nc["v3_keys_MISSING_from_historical_builder"] == list(J.V3_REQUIRED_ENV_KEYS),
          "MEASURED: build_target_env_config forwards NONE of the seven v3 detector/FSM keys")
    check(nc["forwards_morphology"] is False and nc["forwards_corridor"] is False,
          "and it forwards neither the morphology nor the corridor block")
    check("MUST NOT be used" in nc["verdict"] and "non-v3 detector" in nc["risk"],
          "so it is declared unusable for the current collection, with the risk stated")
    for ref, keys in cc["reference_builders"]["v3_keys_present"].items():
        check(len(keys) == len(J.V3_REQUIRED_ENV_KEYS),
              f"the reference builder {ref} does carry all seven keys, so the gap is specific to "
              "the historical one and not to the key list")
    req = cc["required_of_the_new_collector"]
    check(req["must_not_call"] == "build_target_env_config" or "build_target_env_config"
          in req["must_not_call"],
          "the new collector is forbidden from calling the historical builder")
    asserts = req["must_assert_into_its_receipt"]
    # the env key names must be the ACTUAL ones
    check(len(J.V3_REQUIRED_ENV_KEYS) == 12, "twelve v3 env keys are pinned")
    for k in ("binary_phase_detector_profile_file", "binary_phase_invalid_event_policy",
              "event_contract_id", "binary_phase_event_contract_id",
              "phase_sensor_on_threshold_n", "phase_sensor_off_threshold_n",
              "phase_sensor_dwell_s", "detector_sample_dt_s", "binary_phase_debounce_s",
              "binary_phase_fsm_mode", "phase_fsm_input_mode", "binary_phase_actor_fsm_version"):
        check(k in J.V3_REQUIRED_ENV_KEYS, f"{k} is pinned as a required env key")
    check(len(set(J.V3_REQUIRED_ENV_KEYS)) == 12, "no duplicate among the twelve")

    # --- the TWO contract ids are distinct keys with distinct values -----------------------------
    check(set(J.DUAL_CONTRACT_IDS) == {"event_contract_id", "binary_phase_event_contract_id"},
          "both contract ids are pinned as separate keys")
    check(J.DUAL_CONTRACT_IDS["event_contract_id"] == "legacy_events_v1"
          and J.DUAL_CONTRACT_IDS["binary_phase_event_contract_id"]
          == "binary_point_v25+heel_qualified_fsm_v2",
          "with their exact distinct values from the pinned config")
    check(J.DUAL_CONTRACT_IDS["event_contract_id"]
          != J.DUAL_CONTRACT_IDS["binary_phase_event_contract_id"],
          "the two values genuinely differ, so a swap is a real defect and not a no-op")
    # read the values straight from the pinned config, not from a restated constant
    cfg = (J.TG / "runs" / "training" / "MLP_ExNovo_B0820_fsmv3_fixedcorridor_50iter"
           / "training_cfg.resolved.yaml")
    check(J._sha(cfg) == J.PIN_RUNTIME_CONFIG_SHA, "the config read here IS the pinned one")
    text = cfg.read_text()
    # parse key: value anchored at the start of the stripped line. A bare substring would let
    # `morphology_causal_event_contract_id` false-match `event_contract_id`.
    assigned: dict[str, str] = {}
    for line in text.splitlines():
        stripped = line.strip()
        if ":" not in stripped or stripped.startswith("#"):
            continue
        k, _, v = stripped.partition(":")
        assigned.setdefault(k.strip(), v.strip())
    check(assigned.get("morphology_causal_event_contract_id")
          == "binary_point_v25+heel_qualified_fsm_v2",
          "the parser distinguishes morphology_causal_event_contract_id, which shares a suffix "
          "with event_contract_id and would defeat a substring test")
    for key, val in J.DUAL_CONTRACT_IDS.items():
        check(assigned.get(key) == val,
              f"the pinned config assigns {key}: {val} verbatim, on its own line")
    # NEGATIVE 1 - omission of either contract id must be detectable
    for missing in ("event_contract_id", "binary_phase_event_contract_id"):
        incomplete = tuple(k for k in J.V3_REQUIRED_ENV_KEYS if k != missing)
        check(missing not in incomplete and len(incomplete) == 11,
              f"an env config omitting {missing} is one key short of the contract")
        check(set(J.V3_REQUIRED_ENV_KEYS) - set(incomplete) == {missing},
              f"and the omission of {missing} is identifiable exactly, so it fails closed")
    # NEGATIVE 2 - swapping the two values must be detectable
    swapped = {"event_contract_id": J.DUAL_CONTRACT_IDS["binary_phase_event_contract_id"],
               "binary_phase_event_contract_id": J.DUAL_CONTRACT_IDS["event_contract_id"]}
    check(swapped != J.DUAL_CONTRACT_IDS,
          "swapping the two contract ids yields a configuration different from the pinned one")
    for key, val in swapped.items():
        check(assigned.get(key) != val,
              f"and the swapped assignment {key}: {val} is NOT what the pinned config holds")
    check(cc["dual_contract_ids"]["values"] == J.DUAL_CONTRACT_IDS
          and "never be merged, swapped or treated as a rename" in cc["dual_contract_ids"]["rule"],
          "the receipt records both ids with the no-merge, no-swap rule")
    check("523" in cc["dual_contract_ids"]["evidence"]
          and "1301" in cc["dual_contract_ids"]["evidence"],
          "citing the production builder lines where both keys appear side by side")

    # --- the ONLY genuine rename ------------------------------------------------------------------
    ren = cc["config_to_env_key_renames"]
    check(ren["table"] == {"binary_phase_detector_profile": "binary_phase_detector_profile_file"},
          "exactly ONE genuine config -> env rename: the detector profile")
    for not_a_rename in ("invalid_event_policy", "event_contract_id"):
        check(not_a_rename not in ren["table"],
              f"{not_a_rename} is NOT a rename and is no longer listed as one")
    check("both were WRONG" in ren["why"],
          "and the receipt records that the earlier two rename entries were wrong")
    check(assigned.get("binary_phase_invalid_event_policy") == "reject_continue",
          "MEASURED: the config already uses binary_phase_invalid_event_policy verbatim, "
          "value reject_continue, so it was never a rename")
    check("invalid_event_policy" not in assigned,
          "and no bare invalid_event_policy key exists in the config at all")
    for needle in ("binary_phase_actor_fsm_version == v3", "corridor profile",
                   "0.020 m AND hard termination 0.028 m", "V26B closed-loop gate criteria",
                   "binary_phase_detector_profile_file", "binary_phase_invalid_event_policy",
                   "binary_phase_event_contract_id", "phase_sensor_on_threshold_n",
                   "phase_sensor_off_threshold_n", "phase_sensor_dwell_s", "detector_sample_dt_s",
                   "pinned collection semantics",
                   "timeouts", "reward block", "morphology block", "start condition"):
        check(any(needle in a for a in asserts),
              f"the collector must assert '{needle}' into its receipt")
    check("fail-closed" in req["fail_closed"] or "abort" in req["fail_closed"],
          "a missing or mismatched field aborts before any environment step")
    check(J.PIN_RUNTIME_CONFIG_SHA
          == "a870cc38a77d853bbd5fba86b51cfcc3ef20a33a5823f4a42f1b968ba4a537db",
          "the pinned runtime config is the source of truth")
    # the audit must not itself be able to build an env
    check("build_target_env_config(" not in src and "target_domain_imitation" not in ids,
          "the audit reads the historical builder as TEXT and never imports or calls it")

    # --- open points are genuine unknowns, none blocking the preflight ---------------------------
    ops = J.open_points()
    check([o["id"] for o in ops] == ["O1", "O2", "O3"], "three open points")
    check(all(o["blocking_now"] is False for o in ops),
          "none blocks the preflight; each resolves at collection time or belongs to a later stage")

    # --- the receipt exists, is well formed and no-clobber ----------------------------------------
    rp_path = HERE / J.RECEIPT_NAME
    check(rp_path.is_file(), "the audit receipt was written")
    rec = json.loads(rp_path.read_text())
    check(rec["verdict"] == "GO-FOR-ARCHITECT-REVIEW", "the receipt records the verdict")
    check(set(rec) >= {"july_stage_base", "july_stage_markov_selected", "gate_reconciliation",
                       "parent_equivalence", "replay_plan", "open_points"},
          "the receipt carries both stages, the reconciliation, the parent chain and the plan")
    try:
        J.run_audit(write=True)
        raise AssertionError("a second write should have been refused")
    except J.J0Error as e:
        check("no-clobber" in str(e), "a second write is refused no-clobber")

    print(json.dumps({"selftest": "PASS", "checks": CHECKS}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
