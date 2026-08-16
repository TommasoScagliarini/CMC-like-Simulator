from __future__ import annotations

import copy
import hashlib
from pathlib import Path

import h0_primary_grf_split_v5_freeze_contract as v5
import h0_primary_split_v10s_safe_dagger_contract as contract


def _artifact_record(path: object, *, salt: str = "artifact") -> dict[str, object]:
    return {
        "path": path.as_posix() if hasattr(path, "as_posix") else str(path),
        "sha256": hashlib.sha256(salt.encode("utf-8")).hexdigest(),
        "size_bytes": len(salt) + 1,
    }


def _tree_record(path: object, *, salt: str = "tree") -> dict[str, object]:
    files = [
        {
            "path": relative,
            "sha256": hashlib.sha256(
                f"{salt}:{relative}".encode("utf-8")
            ).hexdigest(),
            "size_bytes": index,
        }
        for index, relative in enumerate(
            ("class_and_ctor_args.pkl", "module_state.pkl"), start=1
        )
    ]
    digest = hashlib.sha256()
    for row in files:
        digest.update(row["path"].encode("utf-8"))
        digest.update(b"\0")
        digest.update(row["sha256"].encode("ascii"))
        digest.update(b"\0")
        digest.update(str(row["size_bytes"]).encode("ascii"))
        digest.update(b"\n")
    return {
        "path": path.as_posix() if hasattr(path, "as_posix") else str(path),
        "tree_sha256": digest.hexdigest(),
        "file_count": len(files),
        "files": files,
    }


def _candidate_freeze_summary() -> dict[str, object]:
    candidate_module = _tree_record(contract.MODULE_PATHS["p3"], salt="p3")
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_fit_stage": "p3",
        "candidate_id": (
            contract.candidate_id_prefix("p3")
            + candidate_module["tree_sha256"][:16]
        ),
        "candidate_module": candidate_module,
        "p3_fit_passed": True,
        "candidate_frozen": True,
        "logstd_byte_exact": True,
        "critic_byte_exact": True,
        "every_fit_restarted_from_h0": True,
        "fit_actor_update_count": len(contract.FIT_STAGES),
        "fit_receipts": [
            {
                "fit_stage": stage,
                "receipt": _artifact_record(
                    contract.FIT_RECEIPT_PATHS[stage], salt=f"fit:{stage}"
                ),
            }
            for stage in contract.FIT_STAGES
        ],
        "collection_receipts": [
            {
                "round_index": round_index,
                "case_id": case_id,
                "receipt": _artifact_record(
                    contract.stage_receipt_path(
                        f"collect_r{round_index}__{case_id}"
                    ),
                    salt=f"collect:{round_index}:{case_id}",
                ),
            }
            for round_index in (1, 2, 3)
            for case_id in contract.COLLECTION_CASE_IDS
        ],
        "source_h0": _tree_record(
            contract.SOURCE_H0_MODULE_PATH, salt="source-h0"
        ),
        "teacher_evidence": _artifact_record(
            contract.TEACHER_EVIDENCE_RECEIPT_PATH, salt="teacher-evidence"
        ),
        "target_contract_id": contract.TARGET_CONTRACT_ID,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def _rollout_bindings() -> list[dict[str, object]]:
    bindings = []
    for case_id in contract.FINAL_CASE_IDS:
        destination = contract.canonical_final_case(case_id)["destination"]
        bindings.append(
            {
                "case_id": case_id,
                "passed": True,
                "receipt": _artifact_record(
                    f"{destination}/receipt.json", salt=f"receipt:{case_id}"
                ),
                "gate": _artifact_record(
                    f"{destination}/gate.json", salt=f"gate:{case_id}"
                ),
                "trace": _artifact_record(
                    f"{destination}/trace.json", salt=f"trace:{case_id}"
                ),
            }
        )
    return bindings


def _final_development_summary() -> dict[str, object]:
    candidate_module = _tree_record(contract.MODULE_PATHS["p3"], salt="p3")
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": (
            contract.candidate_id_prefix("p3")
            + candidate_module["tree_sha256"][:16]
        ),
        "candidate_module": candidate_module,
        "candidate_freeze": _artifact_record(
            contract.CANDIDATE_FREEZE_PATH, salt="candidate-freeze"
        ),
        "p3_fit_receipt": _artifact_record(
            contract.FIT_RECEIPT_PATHS["p3"], salt="p3-receipt"
        ),
        "p3_fit_gate": _artifact_record(
            contract.FIT_ROOTS["p3"] / "gate.json", salt="p3-gate"
        ),
        "case_ids": list(contract.FINAL_CASE_IDS),
        "rollout_receipt_count": len(contract.FINAL_CASE_IDS),
        "rollout_pass_count": len(contract.FINAL_CASE_IDS),
        "all_rollouts_passed": True,
        "rollout_bindings": _rollout_bindings(),
        "candidate_fit_stage": "p3",
        "candidate_frozen_before_final": True,
        "fit_actor_update_count": len(contract.FIT_STAGES),
        "every_fit_restarted_from_h0": True,
        "final_candidate_mean_query_count": (
            len(contract.FINAL_CASE_IDS) * contract.EXPECTED_STEPS
        ),
        "final_teacher_query_count": 0,
        "final_mean_blend_count": 0,
        "final_safety_intervention_count": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def _fit_summary(stage: str) -> dict[str, object]:
    counts = contract.expected_fit_counts(stage)
    dagger_receipt_count = (
        len(counts["completed_collection_rounds"])
        * contract.COLLECTION_CASE_COUNT_PER_ROUND
    )
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "protocol_id": contract.PROTOCOL_ID,
        "fit_stage": stage,
        "fit": copy.deepcopy(contract.FIT),
        "source_h0_id": contract.SOURCE_H0_ID,
        "initial_checkpoint_id": contract.SOURCE_H0_ID,
        "continued_from_previous_candidate": False,
        "trainable_scope": "full_mean_network",
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "teacher_evidence_id": contract.TEACHER_EVIDENCE_ID,
        "teacher_evidence_passed": True,
        "base_corpus_case_ids": list(contract.FINAL_CASE_IDS),
        **counts,
        "dagger_receipt_count": dagger_receipt_count,
        "duplicate_sample_count": 0,
        "report_checks": {
            "fixed_configuration": True,
            "save_reload_exact": True,
            "same_state_labels": True,
        },
        "corpus_audit": {
            "failed_v9_rows_used": 0,
            "dagger_sample_count": counts["dagger_sample_count"],
            "same_state_dagger_sample_count": counts["dagger_sample_count"],
        },
        "metrics": {
            "rmse": 0.006,
            "max_abs_error": 0.060,
            "reset_max_abs_error": 0.003,
        },
        "all_finite": True,
        "source_h0_byte_exact": True,
        "critic_byte_exact": True,
        "logstd_byte_exact": True,
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def _physical_summary(case: dict[str, object]) -> dict[str, object]:
    stochastic = case["action_selection"] == "stochastic"
    return {
        "case_id": case["case_id"],
        "action_selection": case["action_selection"],
        "episode_start_offset_s": case["episode_start_offset_s"],
        "action_seed": case["action_seed"],
        "runtime_seed": case["runtime_seed"],
        "sigma": case["sigma"],
        "steps": 500,
        "control_window_count": 5000,
        "raw_sensor_sample_count": 5000,
        "end_reason": "episode_time_limit",
        "terminated": False,
        "truncated": True,
        "phase_valid_cycle_count": 2,
        "grf_penetration_max_m": 0.024999,
        "action_clipped_values": 0,
        "fallback_count": 0,
        "timeout_count": 0,
        "safety_stop_count": 0,
        "sea_plugin_fallback_count": 0,
        "so_solver_unaccepted_count": 0,
        "hard_invalid_count": 0,
        "invalid_event_count": 0,
        "nonfinite_count": 0,
        "routing_failure_count": 0,
        "step_contract_failure_count": 0,
        "binary_event_failure_count": 0,
        "n_actor": 35,
        "n_observation": 84,
        "observation_dtype": "float32",
        "binary_phase_fsm_mode": "binary_active",
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "morphology_weight": 0.0,
        "random_noise_draw_count": 500 if stochastic else 0,
        "single_noise_application_count": 500,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def _collection_summary(round_index: int, case_id: str) -> dict[str, object]:
    case = contract.canonical_collection_case(case_id, round_index)
    return {
        **_physical_summary(case),
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.COLLECTION_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "round_index": round_index,
        "requested_alpha": case["requested_alpha"],
        "candidate_fit_stage": case["candidate_fit_stage"],
        "behavior": contract.COLLECTION_BEHAVIOR,
        "teacher_id": contract.TEACHER_ID,
        "teacher_evidence_id": contract.TEACHER_EVIDENCE_ID,
        "teacher_query_count": 500,
        "sample_count": 500,
        "persisted_label_count": 500,
        "candidate_mean_query_count": 500,
        "same_state_teacher_label_count": 500,
        "candidate_selected_before_teacher_count": 500,
        "served_action_teacher_dependency_count": 500,
        "mean_blend_count": 500,
        "blend_before_noise_count": 500,
        "noise_before_blend_count": 0,
        "multiple_noise_application_count": 0,
        "safety_latch_activation_m": 0.015,
        "safety_latch_release_m": 0.010,
        "safety_latch_release_phase": "SWING_AFTER_TO",
        "safety_signal_lag_steps": 1,
        "safety_intervention_diagnostic_only": True,
        "safety_latch_activation_count": 1,
        "safety_latch_release_count": 1,
        "safety_intervention_count": 8,
        "safety_latch_rule_violation_count": 0,
        "alpha_mismatch_count": 0,
        "mean_blend_mismatch_count": 0,
        "noise_application_mismatch_count": 0,
        "physical_gate_bypass_count": 0,
    }


def _final_summary(case_id: str) -> dict[str, object]:
    case = contract.canonical_final_case(case_id)
    return {
        **_physical_summary(case),
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.FINAL_ROLLOUT_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "behavior": contract.FINAL_BEHAVIOR,
        "candidate_fit_stage": "p3",
        "candidate_id": "H0_PRIMARY_SPLIT_V10S_P3_deadbeef",
        "candidate_mean_query_count": 500,
        "blending_enabled": False,
        "mean_blend_count": 0,
        "teacher_enabled": False,
        "teacher_query_count": 0,
        "served_action_teacher_dependency_count": 0,
        "safety_latch_enabled": False,
        "safety_intervention_count": 0,
        "safety_latch_activation_count": 0,
    }


def test_topology_is_exact_and_every_fit_restarts_from_h0() -> None:
    assert contract.FIT == v5.FIT
    assert contract.FIT_STAGES == ("p0", "p1", "p2", "p3")
    assert contract.ROUND_ALPHAS == {1: 0.25, 2: 0.50, 3: 0.75}
    assert len(contract.STAGE_IDS) == 18
    assert contract.STAGE_IDS[0] == "fit_p0"
    assert contract.STAGE_IDS[10] == "freeze_p3"
    assert contract.STAGE_IDS[-1] == "finalize_development"
    assert contract.AUTHORITY["source_h0_required_for_every_refit"] is True
    assert contract.AUTHORITY["retry_authorized"] is False
    assert contract.AUTHORITY["sweep_authorized"] is False
    assert contract.AUTHORITY["protected_trial_access_authorized"] is False


def test_manifest_covers_live_v26_runtime_and_every_declared_path_exists() -> None:
    critical_sources = {
        "environment_config_source",
        "v9_causal_teacher",
        "v9_causal_teacher_contract",
        "v8r1p1_collector",
        "v8r1_collector",
        "v26_event_collector",
        "v6_teacher_engine",
        "rollout_eval",
        "asymmetric_rl_module",
        "training_config",
        "environment_factory",
        "reward_function",
        "environment",
        "binary_phase_detector",
        "binary_phase_adapter_v26",
        "binary_phase_fsm_v26",
        "prosthetic_phase_fsm",
        "simulation_runner",
        "model_loader",
        "online_grf",
        "static_optimization",
        "kinematics_interpolator",
        "prosthesis_controller",
        "outer_loop",
        "inverse_dynamics",
        "output",
        "root_config",
        "path_resolver",
        "setup_io",
    }
    critical_inputs = {
        "source_h0_config",
        "v25_candidate_freeze",
        "v25_profile",
        "primary_grf_core_lock",
        "primary_grf_profile",
        "analog_detector_profile",
        "morphology_profile",
        "simulator_setup",
        "runtime_model",
        "runtime_kinematics",
        "external_loads",
        "prescribed_grf_data",
        "reserve_actuators",
        "macos_sea_plugin",
        "macos_online_grf_plugin",
        "v8r1p1_preflight",
        "v8r1p1_execution_lock",
        "v8r1_execution_lock",
    }
    assert critical_sources <= set(contract.SOURCE_RELATIVE_PATHS)
    assert critical_inputs <= set(contract.INPUT_RELATIVE_PATHS)

    for case_id in contract.FINAL_CASE_IDS:
        for artifact_name in ("trace", "summary", "gate", "receipt"):
            assert (
                f"base_{case_id}_{artifact_name}"
                in contract.INPUT_RELATIVE_PATHS
            )

    repository = Path(__file__).resolve().parents[1]
    missing = [
        f"{kind}:{name}:{relative}"
        for kind, records in (
            ("source", contract.SOURCE_RELATIVE_PATHS),
            ("input", contract.INPUT_RELATIVE_PATHS),
        )
        for name, relative in records.items()
        if not (repository / relative).is_file()
    ]
    assert missing == []


def test_cumulative_fit_counts_are_3000_then_one_thousand_per_round() -> None:
    assert [
        contract.expected_fit_counts(stage)["sample_count"]
        for stage in contract.FIT_STAGES
    ] == [3000, 4000, 5000, 6000]
    assert [
        contract.expected_fit_counts(stage)["reset_row_count"]
        for stage in contract.FIT_STAGES
    ] == [6, 8, 10, 12]

    for stage in contract.FIT_STAGES:
        assert contract.fit_gate(_fit_summary(stage), stage=stage)["passed"] is True


def test_offline_envelope_is_operational_and_strict_at_upper_boundary() -> None:
    assert contract.OFFLINE_THRESHOLDS == {
        "rmse_max": 0.006,
        "max_abs_error_max": 0.060,
        "reset_max_abs_error_max": 0.003,
    }
    assert contract.OFFLINE_THRESHOLD_PROVENANCE["scientific_claim"] is False
    summary = _fit_summary("p0")
    summary["metrics"]["max_abs_error"] = 0.060001
    gate = contract.fit_gate(summary, stage="p0")
    assert gate["passed"] is False
    assert gate["checks"]["max_abs"] is False


def test_fit_rejects_continuation_from_previous_candidate() -> None:
    summary = _fit_summary("p2")
    summary["continued_from_previous_candidate"] = True
    summary["initial_checkpoint_id"] = "H0_PRIMARY_SPLIT_V10S_P1_bad"
    gate = contract.fit_gate(summary, stage="p2")
    assert gate["passed"] is False
    assert gate["checks"]["restart_from_h0"] is False


def test_fit_requires_complete_engine_and_corpus_audits() -> None:
    for field, mutation, check in (
        (
            "report_checks",
            {"fixed_configuration": True, "save_reload_exact": False},
            "report_checks",
        ),
        (
            "corpus_audit",
            {
                "failed_v9_rows_used": 1,
                "dagger_sample_count": 0,
                "same_state_dagger_sample_count": 0,
            },
            "corpus_audit",
        ),
        ("dagger_receipt_count", 1, "dagger_receipts"),
    ):
        summary = _fit_summary("p0")
        summary[field] = mutation
        gate = contract.fit_gate(summary, stage="p0")
        assert gate["passed"] is False
        assert gate["checks"][check] is False


def test_collection_gate_accepts_intervention_but_not_physical_failure() -> None:
    summary = _collection_summary(
        2,
        "deterministic_offset_minus_0p20",
    )
    gate = contract.collection_gate(summary, round_index=2)
    assert gate["passed"] is True
    assert gate["physical_gate_relaxed"] is False

    boundary = copy.deepcopy(summary)
    boundary["grf_penetration_max_m"] = 0.025
    failed = contract.collection_gate(boundary, round_index=2)
    assert failed["passed"] is False
    assert failed["checks"]["penetration"] is False

    terminated = copy.deepcopy(summary)
    terminated["terminated"] = True
    terminated["truncated"] = False
    terminated["end_reason"] = "grf_penetration"
    assert contract.collection_gate(terminated, round_index=2)["passed"] is False


def test_collection_requires_scheduled_alpha_and_mean_then_single_noise() -> None:
    summary = _collection_summary(3, "stochastic_nominal_seed_126")
    assert contract.collection_gate(summary, round_index=3)["passed"] is True

    for field, value, check in (
        ("requested_alpha", 0.50, "requested_alpha"),
        ("noise_before_blend_count", 1, "blend_then_noise"),
        ("multiple_noise_application_count", 1, "blend_then_noise"),
        ("safety_latch_rule_violation_count", 1, "latch_exact"),
        ("physical_gate_bypass_count", 1, "physical_not_bypassed"),
    ):
        mutated = copy.deepcopy(summary)
        mutated[field] = value
        gate = contract.collection_gate(mutated, round_index=3)
        assert gate["passed"] is False
        assert gate["checks"][check] is False


def test_collection_requires_all_same_state_teacher_dependencies() -> None:
    summary = _collection_summary(1, "deterministic_offset_minus_0p20")
    for field in (
        "sample_count",
        "same_state_teacher_label_count",
        "candidate_selected_before_teacher_count",
        "served_action_teacher_dependency_count",
    ):
        mutated = copy.deepcopy(summary)
        mutated[field] = 499
        gate = contract.collection_gate(mutated, round_index=1)
        assert gate["passed"] is False
        assert gate["checks"]["labels_complete"] is False


def test_artifact_and_tree_helpers_are_pure_strict_hash_bindings() -> None:
    artifact = _artifact_record(contract.CANDIDATE_FREEZE_PATH)
    assert contract.artifact_record_matches(
        artifact, contract.CANDIDATE_FREEZE_PATH
    )
    bad_artifact = copy.deepcopy(artifact)
    bad_artifact["sha256"] = str(bad_artifact["sha256"]).upper()
    assert not contract.artifact_record_matches(
        bad_artifact, contract.CANDIDATE_FREEZE_PATH
    )

    tree = _tree_record(contract.MODULE_PATHS["p3"])
    assert contract.tree_record_matches(tree, contract.MODULE_PATHS["p3"])
    reordered = copy.deepcopy(tree)
    reordered["files"].reverse()
    assert not contract.tree_record_matches(
        reordered, contract.MODULE_PATHS["p3"]
    )
    wrong_path = copy.deepcopy(tree)
    wrong_path["path"] = contract.MODULE_PATHS["p2"].as_posix()
    assert not contract.tree_record_matches(
        wrong_path, contract.MODULE_PATHS["p3"]
    )


def test_candidate_freeze_binds_all_inputs_in_exact_order() -> None:
    summary = _candidate_freeze_summary()
    assert contract.candidate_freeze_gate(summary)["passed"] is True

    bad_id = copy.deepcopy(summary)
    bad_id["candidate_id"] = contract.candidate_id_prefix("p3") + "0" * 16
    gate = contract.candidate_freeze_gate(bad_id)
    assert gate["passed"] is False
    assert gate["checks"]["candidate_binding"] is False

    wrong_fit_order = copy.deepcopy(summary)
    wrong_fit_order["fit_receipts"][0], wrong_fit_order["fit_receipts"][1] = (
        wrong_fit_order["fit_receipts"][1],
        wrong_fit_order["fit_receipts"][0],
    )
    gate = contract.candidate_freeze_gate(wrong_fit_order)
    assert gate["passed"] is False
    assert gate["checks"]["fit_receipts"] is False

    bad_collection_hash = copy.deepcopy(summary)
    bad_collection_hash["collection_receipts"][0]["receipt"]["sha256"] = "z" * 64
    gate = contract.candidate_freeze_gate(bad_collection_hash)
    assert gate["passed"] is False
    assert gate["checks"]["collection_receipts"] is False


def test_final_rollout_forbids_teacher_blend_and_safety_latch() -> None:
    summary = _final_summary("stochastic_nominal_seed_128")
    assert contract.final_rollout_gate(summary)["passed"] is True

    for field, value, check in (
        ("teacher_query_count", 1, "no_teacher"),
        ("mean_blend_count", 1, "unblended"),
        ("safety_intervention_count", 1, "no_safety_latch"),
    ):
        mutated = copy.deepcopy(summary)
        mutated[field] = value
        gate = contract.final_rollout_gate(mutated)
        assert gate["passed"] is False
        assert gate["checks"][check] is False

    no_candidate_query = copy.deepcopy(summary)
    no_candidate_query["candidate_mean_query_count"] = 499
    gate = contract.final_rollout_gate(no_candidate_query)
    assert gate["passed"] is False
    assert gate["checks"]["candidate_queries"] is False


def test_stage_descriptors_and_paths_fail_closed() -> None:
    collection = contract.stage_descriptor(
        "collect_r1__deterministic_offset_minus_0p20"
    )
    assert collection["kind"] == "collection"
    assert collection["round_index"] == 1
    assert collection["case"]["requested_alpha"] == 0.25
    assert contract.stage_descriptor("freeze_p3") == {
        "kind": "freeze",
        "fit_stage": "p3",
    }
    assert contract.stage_receipt_path("fit_p2") == (
        contract.FIT_RECEIPT_PATHS["p2"]
    )
    try:
        contract.stage_descriptor("collect_r4__deterministic_offset_minus_0p20")
    except ValueError:
        pass
    else:  # pragma: no cover - explicit fail-closed assertion.
        raise AssertionError("unknown stage accepted")


def test_v10a_teacher_evidence_gate_requires_adjudicated_byte_exact_rows() -> None:
    receipt = {
        "schema_version": 94,
        "status": contract.V10A_EVIDENCE_STATUS,
        "passed": True,
        "scope": contract.V10A_EVIDENCE_SCOPE,
        "next_stage": contract.V10A_EVIDENCE_NEXT_STAGE,
        "scientific_evidence": {
            "coherent_teacher_evidence_accepted": True,
            "teacher_action_byte_exact_count": 500,
            "teacher_mean_byte_exact_count": 500,
            "teacher_view_byte_exact_count": 500,
        },
        "counterfactual_gate": {"passed": True},
        "original_protocol": {
            "preserved_as_fail": True,
            "retry_authorized": False,
        },
        "rollout_rerun_count": 0,
        "candidate_created": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    assert contract.teacher_evidence_gate(receipt)["passed"] is True
    receipt["scientific_evidence"]["teacher_mean_byte_exact_count"] = 499
    assert contract.teacher_evidence_gate(receipt)["passed"] is False


def test_final_development_requires_all_six_unassisted_rollouts() -> None:
    summary = _final_development_summary()
    gate = contract.final_development_gate(summary)
    assert gate["passed"] is True
    assert gate["next_stage"] == "V10Q_SEA_RESERVE_QUALIFICATION_REQUIRED"
    summary["rollout_pass_count"] = 5
    failed = contract.final_development_gate(summary)
    assert failed["passed"] is False
    assert failed["next_stage"] == "STOP_V10S_TERMINAL_NO_RETRY_OR_SWEEP"


def test_final_development_binds_ordered_receipt_gate_and_trace_records() -> None:
    summary = _final_development_summary()
    reversed_bindings = copy.deepcopy(summary)
    reversed_bindings["rollout_bindings"].reverse()
    gate = contract.final_development_gate(reversed_bindings)
    assert gate["passed"] is False
    assert gate["checks"]["rollout_bindings"] is False

    wrong_trace_path = copy.deepcopy(summary)
    wrong_trace_path["rollout_bindings"][0]["trace"]["path"] = (
        "validation/wrong/trace.json"
    )
    gate = contract.final_development_gate(wrong_trace_path)
    assert gate["passed"] is False
    assert gate["checks"]["rollout_bindings"] is False

    wrong_candidate_hash = copy.deepcopy(summary)
    wrong_candidate_hash["candidate_module"]["tree_sha256"] = "0" * 64
    gate = contract.final_development_gate(wrong_candidate_hash)
    assert gate["passed"] is False
    assert gate["checks"]["candidate_module"] is False
