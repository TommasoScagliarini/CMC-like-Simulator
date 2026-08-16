from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path

import h0_primary_split_v10s_safe_dagger_contract as v10s
import h0_primary_split_v11_weighted_full_mean_contract as contract


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


def _design_audit_receipt() -> dict[str, object]:
    metrics = {
        "rmse": 0.004,
        "max_abs_error": 0.050,
        "reset_max_abs_error": 0.002,
    }
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "contract_id": contract.FIT_CONTRACT_ID,
        "design_audit_id": contract.DESIGN_AUDIT_ID,
        "status": contract.DESIGN_AUDIT_PASS_STATUS,
        "gate": {"passed": True},
        "dry_run": True,
        "no_candidate_checkpoint": True,
        "candidate_checkpoint_paths": [],
        "candidate_checkpoints_persisted": 0,
        "source_bindings": {
            name: _artifact_record(path, salt=f"source:{name}")
            for name, path in contract.DESIGN_AUDIT_SOURCE_RELATIVE_PATHS.items()
        },
        "source_h0": _tree_record(contract.SOURCE_H0_MODULE_PATH, salt="source-h0"),
        "corpus": {
            "artifact": _artifact_record(
                contract.V10S_P0_CORPUS_PATH, salt="corpus"
            ),
            "rows": 3000,
            "observation_dim": 35,
            "action_dim": 2,
            "reset_rows": 6,
            "observations_sha256": (
                "d367a4697606f7c5d721823c973aabbbc86fb314e9b74e1529afc22e45a4d9ad"
            ),
            "actions_sha256": (
                "06df49086f71283238842f9903cf94271c03b9b09311eefc859b706c4e66aaa6"
            ),
            "reset_mask_sha256": (
                "95addb5dda30f1a4c4830d2334c2d7b1d51fc1e3caf1a44e9f8027fbd5302b57"
            ),
        },
        "fit_design": copy.deepcopy(contract.FIT),
        "gates": copy.deepcopy(contract.OFFLINE_THRESHOLDS),
        "observed_metrics": copy.deepcopy(metrics),
        "p0_reproduction_reference_metrics": copy.deepcopy(metrics),
        "deterministic_tolerance": copy.deepcopy(
            contract.DESIGN_AUDIT_DETERMINISTIC_TOLERANCE
        ),
        "preservation_audit": {
            "source_h0_byte_exact": True,
            "source_checkpoint_scope": "actor_only_rl_module",
            "critic_present": False,
            "critic_parameter_count": 0,
            "critic_byte_exact": True,
            "logstd_byte_exact": True,
            "disabled_clock_columns_0_1_bit_zero": True,
            "normalization_folded_into_first_layer": True,
            "fold_equivalence_passed": True,
            "no_runtime_normalization_wrapper": True,
            "no_prescribed_clock": True,
        },
        "actor_fit_executions": 1,
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "retry_authorized": False,
        "sweep_authorized": False,
        "artifacts_written": [contract.DESIGN_AUDIT_RECEIPT_PATH.as_posix()],
    }


def _fit_summary(stage: str) -> dict[str, object]:
    counts = contract.expected_fit_counts(stage)
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.FIT_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "fit_stage": stage,
        "fit_contract_id": contract.FIT_CONTRACT_ID,
        "fit": copy.deepcopy(contract.FIT),
        "actor_architecture": copy.deepcopy(contract.ACTOR_ARCHITECTURE),
        "normalization": copy.deepcopy(contract.BASE_CORPUS_NORMALIZATION),
        "sample_weighting": copy.deepcopy(contract.SAMPLE_WEIGHTING),
        "source_h0_id": contract.SOURCE_H0_ID,
        "initial_checkpoint_id": contract.SOURCE_H0_ID,
        "continued_from_previous_candidate": False,
        "trainable_scope": contract.TRAINABLE_SCOPE,
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "teacher_evidence_id": contract.TEACHER_EVIDENCE_ID,
        "teacher_evidence_passed": True,
        "v10s_terminal_failure_id": contract.V10S_TERMINAL_FAILURE_ID,
        "v10s_terminal_failure_passed": True,
        "design_audit_id": contract.DESIGN_AUDIT_ID,
        "design_audit_passed": True,
        "design_audit_receipt": _artifact_record(
            contract.DESIGN_AUDIT_RECEIPT_PATH, salt="design-audit"
        ),
        "base_corpus_case_ids": list(contract.FINAL_CASE_IDS),
        **counts,
        "dagger_receipt_count": (
            len(counts["completed_collection_rounds"])
            * contract.COLLECTION_CASE_COUNT_PER_ROUND
        ),
        "duplicate_sample_count": 0,
        "report_checks": {
            "fixed_configuration": True,
            "save_reload_exact": True,
            "weighted_loss_exact": True,
        },
        "corpus_audit": {
            "failed_v9_rows_used": 0,
            "dagger_sample_count": counts["dagger_sample_count"],
            "same_state_dagger_sample_count": counts["dagger_sample_count"],
        },
        "adamw_epochs_run": 3000,
        "lbfgs_completed": True,
        "deterministic_algorithms_enabled": True,
        "disabled_clock_column_indices": [0, 1],
        "disabled_clock_columns_bit_zero": True,
        "disabled_clock_columns_bit_zero_after_save_reload": True,
        "normalization_stats_from_base_corpus_only": True,
        "normalization_stats_frozen_across_stages": True,
        "normalization_folded_into_first_layer": True,
        "fold_equivalence_passed": True,
        "runtime_normalization_wrapper_present": False,
        "prescribed_clock_present": False,
        "anchor_used": False,
        "hard_polish_used": False,
        "metrics": {
            "rmse": 0.006,
            "max_abs_error": 0.060,
            "reset_max_abs_error": 0.003,
        },
        "design_audit_reproduction_within_tolerance": True,
        "all_finite": True,
        "source_h0_byte_exact": True,
        "source_checkpoint_scope": "actor_only_rl_module",
        "critic_present": False,
        "critic_parameter_count": 0,
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
        "fit_contract_id": contract.FIT_CONTRACT_ID,
        "design_audit_id": contract.DESIGN_AUDIT_ID,
        "design_audit_passed": True,
        "design_audit_receipt": _artifact_record(
            contract.DESIGN_AUDIT_RECEIPT_PATH
        ),
        "behavior": contract.FINAL_BEHAVIOR,
        "candidate_fit_stage": "p3",
        "candidate_id": "H0_PRIMARY_SPLIT_V11_P3_deadbeef",
        "candidate_mean_query_count": 500,
        "blending_enabled": False,
        "mean_blend_count": 0,
        "teacher_enabled": False,
        "teacher_query_count": 0,
        "served_action_teacher_dependency_count": 0,
        "safety_latch_enabled": False,
        "safety_intervention_count": 0,
        "safety_latch_activation_count": 0,
        "source_checkpoint_scope": "actor_only_rl_module",
        "critic_present": False,
        "critic_parameter_count": 0,
        "runtime_normalization_wrapper_present": False,
        "prescribed_clock_present": False,
        "normalization_folded_into_first_layer": True,
        "disabled_clock_columns_0_1_bit_zero": True,
    }


def _candidate_freeze_summary() -> dict[str, object]:
    candidate_module = _tree_record(contract.MODULE_PATHS["p3"], salt="p3")
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "protocol_id": contract.PROTOCOL_ID,
        "fit_contract_id": contract.FIT_CONTRACT_ID,
        "candidate_fit_stage": "p3",
        "candidate_id": (
            contract.candidate_id_prefix("p3")
            + candidate_module["tree_sha256"][:16]
        ),
        "candidate_module": candidate_module,
        "p3_fit_passed": True,
        "candidate_frozen": True,
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
            contract.TEACHER_EVIDENCE_RECEIPT_PATH, salt="teacher"
        ),
        "design_audit_id": contract.DESIGN_AUDIT_ID,
        "design_audit_passed": True,
        "design_audit_receipt": _artifact_record(
            contract.DESIGN_AUDIT_RECEIPT_PATH, salt="audit"
        ),
        "v10s_terminal_failure_id": contract.V10S_TERMINAL_FAILURE_ID,
        "v10s_terminal_failure_passed": True,
        "target_contract_id": contract.TARGET_CONTRACT_ID,
        "fit_actor_update_count": 4,
        "every_fit_restarted_from_h0": True,
        "source_checkpoint_scope": "actor_only_rl_module",
        "critic_present": False,
        "critic_parameter_count": 0,
        "normalization_folded_into_first_layer": True,
        "runtime_normalization_wrapper_present": False,
        "prescribed_clock_present": False,
        "disabled_clock_columns_0_1_bit_zero": True,
        "logstd_byte_exact": True,
        "critic_byte_exact": True,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def _rollout_bindings() -> list[dict[str, object]]:
    return [
        {
            "case_id": case_id,
            "passed": True,
            "receipt": _artifact_record(
                f"{contract.canonical_final_case(case_id)['destination']}/receipt.json",
                salt=f"receipt:{case_id}",
            ),
            "gate": _artifact_record(
                f"{contract.canonical_final_case(case_id)['destination']}/gate.json",
                salt=f"gate:{case_id}",
            ),
            "trace": _artifact_record(
                f"{contract.canonical_final_case(case_id)['destination']}/trace.json",
                salt=f"trace:{case_id}",
            ),
        }
        for case_id in contract.FINAL_CASE_IDS
    ]


def _final_development_summary() -> dict[str, object]:
    candidate_module = _tree_record(contract.MODULE_PATHS["p3"], salt="p3")
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "protocol_id": contract.PROTOCOL_ID,
        "fit_contract_id": contract.FIT_CONTRACT_ID,
        "candidate_id": (
            contract.candidate_id_prefix("p3")
            + candidate_module["tree_sha256"][:16]
        ),
        "candidate_module": candidate_module,
        "candidate_freeze": _artifact_record(contract.CANDIDATE_FREEZE_PATH),
        "p3_fit_receipt": _artifact_record(contract.FIT_RECEIPT_PATHS["p3"]),
        "p3_fit_gate": _artifact_record(contract.FIT_ROOTS["p3"] / "gate.json"),
        "design_audit_id": contract.DESIGN_AUDIT_ID,
        "design_audit_passed": True,
        "design_audit_receipt": _artifact_record(
            contract.DESIGN_AUDIT_RECEIPT_PATH
        ),
        "v10s_terminal_failure_id": contract.V10S_TERMINAL_FAILURE_ID,
        "v10s_terminal_failure_passed": True,
        "case_ids": list(contract.FINAL_CASE_IDS),
        "rollout_receipt_count": 6,
        "rollout_pass_count": 6,
        "all_rollouts_passed": True,
        "rollout_bindings": _rollout_bindings(),
        "candidate_fit_stage": "p3",
        "candidate_frozen_before_final": True,
        "fit_actor_update_count": 4,
        "every_fit_restarted_from_h0": True,
        "final_candidate_mean_query_count": 3000,
        "final_teacher_query_count": 0,
        "final_mean_blend_count": 0,
        "final_safety_intervention_count": 0,
        "source_checkpoint_scope": "actor_only_rl_module",
        "critic_present": False,
        "critic_parameter_count": 0,
        "runtime_normalization_wrapper_present": False,
        "prescribed_clock_present": False,
        "normalization_folded_into_first_layer": True,
        "disabled_clock_columns_0_1_bit_zero": True,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def test_identity_paths_and_topology_are_fresh_v11() -> None:
    assert contract.SCHEMA_VERSION == 110
    assert contract.REVISION == "2026-08-09"
    assert "V11" in contract.PROTOCOL_ID
    assert "2026-08-09" in contract.RUN_ROOT.as_posix()
    assert contract.FIT_STAGES == ("p0", "p1", "p2", "p3")
    assert contract.ROUND_ALPHAS == {1: 0.25, 2: 0.50, 3: 0.75}
    assert len(contract.STAGE_IDS) == 18
    assert contract.STAGE_IDS[0] == "fit_p0"
    assert contract.STAGE_IDS[-1] == "finalize_development"
    assert contract.AUTHORITY["retry_authorized"] is False
    assert contract.AUTHORITY["authority_text"] == (
        "autorizzo V11 weighted full-mean"
    )
    assert contract.AUTHORITY["sweep_authorized"] is False
    assert contract.AUTHORITY["protected_trial_access_authorized"] is False
    assert contract.AUTHORITY["reserve_trial_access_authorized"] is False


def test_fit_design_is_exact_weighted_normalized_two_phase_design() -> None:
    assert contract.ACTOR_ARCHITECTURE == {
        "kind": "standard_mean_mlp",
        "input_dim": 35,
        "hidden_dims": [256, 256],
        "output_dim": 2,
        "activation": "tanh",
        "residual_actor": False,
    }
    assert contract.BASE_CORPUS_NORMALIZATION["std_floor"] == 1.0e-4
    assert contract.BASE_CORPUS_NORMALIZATION[
        "fold_into_first_layer_before_save"
    ] is True
    assert contract.BASE_CORPUS_NORMALIZATION[
        "runtime_normalization_wrapper"
    ] is False
    assert contract.SAMPLE_WEIGHTING["reset_weight"] == 100.0
    assert contract.ADAMW["seed"] == 20260807
    assert contract.ADAMW["full_batch"] is True
    assert contract.ADAMW["epochs"] == 3000
    assert [row["learning_rate"] for row in contract.ADAMW["learning_rate_schedule"]] == [
        3.0e-4,
        1.0e-4,
        3.0e-5,
    ]
    assert contract.LBFGS["line_search_fn"] == "strong_wolfe"
    assert contract.LBFGS["max_iter"] == 300
    assert contract.FIT["disabled_clock_columns"] == [0, 1]
    assert contract.FIT["anchor_enabled"] is False
    assert contract.FIT["hard_polish_enabled"] is False


def test_manifest_contains_all_v10s_dependencies_and_v11_surface() -> None:
    assert set(v10s.SOURCE_RELATIVE_PATHS.values()) <= set(
        contract.SOURCE_RELATIVE_PATHS.values()
    )
    assert set(v10s.INPUT_RELATIVE_PATHS.values()) <= set(
        contract.INPUT_RELATIVE_PATHS.values()
    )
    assert contract.SOURCE_RELATIVE_PATHS["fit_engine"] == (
        "validation/h0_primary_split_v11_weighted_fit.py"
    )
    assert contract.SOURCE_RELATIVE_PATHS["runner"] == (
        "validation/run_h0_primary_split_v11_weighted_full_mean.py"
    )
    assert contract.INPUT_RELATIVE_PATHS["design_audit_receipt"] == (
        contract.DESIGN_AUDIT_RECEIPT_PATH.as_posix()
    )
    assert {
        "v10s_terminal_ledger",
        "v10s_p0_gate",
        "v10s_p0_summary",
        "v10s_p0_corpus",
    } <= set(contract.INPUT_RELATIVE_PATHS)


def test_real_v10s_terminal_failure_is_authoritative_and_fail_closed() -> None:
    root = Path(__file__).resolve().parents[1]
    ledger = json.loads((root / contract.V10S_TERMINAL_LEDGER_PATH).read_text())
    gate = json.loads((root / contract.V10S_P0_GATE_PATH).read_text())
    summary = json.loads((root / contract.V10S_P0_SUMMARY_PATH).read_text())
    result = contract.v10s_terminal_failure_gate(ledger, gate, summary)
    assert result["passed"] is True

    mutated = copy.deepcopy(ledger)
    mutated["retry_authorized"] = True
    failed = contract.v10s_terminal_failure_gate(mutated, gate, summary)
    assert failed["passed"] is False
    assert failed["checks"]["ledger_no_retry_sweep"] is False


def test_design_audit_receipt_is_mandatory_and_bound_to_exact_design() -> None:
    receipt = _design_audit_receipt()
    assert contract.design_audit_gate(receipt)["passed"] is True

    for field, value, check in (
        ("fit_design", {"wrong": True}, "fit_design"),
        ("no_candidate_checkpoint", False, "no_candidate_checkpoint"),
        ("candidate_checkpoint_paths", ["candidate"], "checkpoint_artifacts_absent"),
        ("candidate_checkpoints_persisted", 1, "checkpoint_artifacts_absent"),
        ("candidate_checkpoints_persisted", False, "checkpoint_artifacts_absent"),
        ("artifacts_written", ["validation/checkpoint.pkl"], "receipt_only_artifact"),
    ):
        mutated = copy.deepcopy(receipt)
        mutated[field] = value
        result = contract.design_audit_gate(mutated)
        assert result["passed"] is False
        assert result["checks"][check] is False

    mutated = _design_audit_receipt()
    mutated["p0_reproduction_reference_metrics"]["rmse"] += 1.0e-12
    result = contract.design_audit_gate(mutated)
    assert result["passed"] is False
    assert result["checks"]["p0_reference_equals_observed"] is False


def test_design_audit_current_source_hash_drift_fails_binding() -> None:
    receipt = _design_audit_receipt()
    current = {
        "source_bindings": copy.deepcopy(receipt["source_bindings"]),
        "source_h0": copy.deepcopy(receipt["source_h0"]),
        "corpus": copy.deepcopy(receipt["corpus"]["artifact"]),
    }
    assert contract.design_audit_current_binding_gate(
        receipt, current
    )["passed"] is True

    current["source_bindings"]["fitter"]["sha256"] = "f" * 64
    failed = contract.design_audit_current_binding_gate(receipt, current)
    assert failed["passed"] is False
    assert failed["checks"]["source_bindings_current"] is False


def test_design_audit_rejects_gate_failure_and_clock_or_wrapper_drift() -> None:
    receipt = _design_audit_receipt()
    receipt["observed_metrics"]["reset_max_abs_error"] = 0.0030001
    assert contract.design_audit_gate(receipt)["passed"] is False

    for field in (
        "disabled_clock_columns_0_1_bit_zero",
        "normalization_folded_into_first_layer",
        "fold_equivalence_passed",
        "no_runtime_normalization_wrapper",
        "no_prescribed_clock",
    ):
        mutated = _design_audit_receipt()
        mutated["preservation_audit"][field] = False
        result = contract.design_audit_gate(mutated)
        assert result["passed"] is False
        assert result["checks"]["preservation"] is False

    for field, value in (
        ("source_checkpoint_scope", "actor_critic_rl_module"),
        ("critic_present", True),
        ("critic_parameter_count", 1),
        # ``False == 0`` in Python; the contract must still reject a boolean.
        ("critic_parameter_count", False),
    ):
        mutated = _design_audit_receipt()
        mutated["preservation_audit"][field] = value
        result = contract.design_audit_gate(mutated)
        assert result["passed"] is False
        assert result["checks"]["preservation"] is False


def test_cumulative_fit_counts_and_all_four_fresh_fit_gates() -> None:
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


def test_fit_rejects_continuation_wrapper_clock_and_design_audit_drift() -> None:
    mutations = (
        ("continued_from_previous_candidate", True, "restart_from_h0"),
        ("source_checkpoint_scope", "actor_critic_rl_module", "actor_only_source"),
        ("critic_present", True, "actor_only_source"),
        ("critic_parameter_count", 1, "actor_only_source"),
        ("critic_parameter_count", False, "actor_only_source"),
        ("fold_equivalence_passed", False, "fold_equivalence"),
        ("runtime_normalization_wrapper_present", True, "normalization_folded"),
        ("prescribed_clock_present", True, "normalization_folded"),
        ("disabled_clock_columns_bit_zero", False, "clock_columns_zero"),
        ("design_audit_passed", False, "design_audit"),
    )
    for field, value, check in mutations:
        summary = _fit_summary("p0")
        summary[field] = value
        result = contract.fit_gate(summary, stage="p0")
        assert result["passed"] is False
        assert result["checks"][check] is False


def test_offline_gates_are_unchanged_and_boundary_is_inclusive() -> None:
    assert contract.OFFLINE_THRESHOLDS == {
        "rmse_max": 0.006,
        "max_abs_error_max": 0.060,
        "reset_max_abs_error_max": 0.003,
    }
    summary = _fit_summary("p0")
    assert contract.fit_gate(summary, stage="p0")["passed"] is True
    summary["metrics"]["rmse"] = 0.0060001
    assert contract.fit_gate(summary, stage="p0")["passed"] is False


def test_collection_schedule_and_physical_gate_remain_v10s_exact() -> None:
    summary = _collection_summary(2, "deterministic_offset_minus_0p20")
    result = contract.collection_gate(summary, round_index=2)
    assert result["passed"] is True
    assert result["physical_gate_relaxed"] is False

    summary["grf_penetration_max_m"] = 0.025
    failed = contract.collection_gate(summary, round_index=2)
    assert failed["passed"] is False
    assert failed["checks"]["penetration"] is False


def test_candidate_freeze_binds_design_audit_and_folded_actor() -> None:
    summary = _candidate_freeze_summary()
    assert contract.candidate_freeze_gate(summary)["passed"] is True
    summary["runtime_normalization_wrapper_present"] = True
    failed = contract.candidate_freeze_gate(summary)
    assert failed["passed"] is False
    assert failed["checks"]["normalization_folded"] is False

    prescribed = _candidate_freeze_summary()
    prescribed["prescribed_clock_present"] = True
    failed = contract.candidate_freeze_gate(prescribed)
    assert failed["passed"] is False
    assert failed["checks"]["no_prescribed_clock"] is False

    for field, value in (
        ("source_checkpoint_scope", "actor_critic_rl_module"),
        ("critic_present", True),
        ("critic_parameter_count", 1),
        ("critic_parameter_count", False),
    ):
        mutated = _candidate_freeze_summary()
        mutated[field] = value
        gate = contract.candidate_freeze_gate(mutated)
        assert gate["passed"] is False
        assert gate["checks"]["actor_only_source"] is False


def test_final_rollout_forbids_teacher_blend_latch_wrapper_and_clock() -> None:
    summary = _final_summary("stochastic_nominal_seed_128")
    assert contract.final_rollout_gate(summary)["passed"] is True
    for field, value, check in (
        ("fit_contract_id", "wrong", "fit_contract"),
        ("design_audit_passed", False, "design_audit"),
        ("source_checkpoint_scope", "actor_critic_rl_module", "actor_only_source"),
        ("critic_present", True, "actor_only_source"),
        ("critic_parameter_count", 1, "actor_only_source"),
        ("critic_parameter_count", False, "actor_only_source"),
        ("teacher_query_count", 1, "no_teacher"),
        ("mean_blend_count", 1, "unblended"),
        ("safety_intervention_count", 1, "no_safety_latch"),
        ("runtime_normalization_wrapper_present", True, "no_runtime_wrapper"),
        ("prescribed_clock_present", True, "no_prescribed_clock"),
        ("normalization_folded_into_first_layer", False, "folded_actor"),
        ("disabled_clock_columns_0_1_bit_zero", False, "clock_columns_zero"),
    ):
        mutated = copy.deepcopy(summary)
        mutated[field] = value
        result = contract.final_rollout_gate(mutated)
        assert result["passed"] is False
        assert result["checks"][check] is False


def test_final_development_requires_six_rollouts_then_v10q() -> None:
    summary = _final_development_summary()
    result = contract.final_development_gate(summary)
    assert result["passed"] is True
    assert result["next_stage"] == "V10Q_SEA_RESERVE_QUALIFICATION_REQUIRED"

    summary["rollout_pass_count"] = 5
    failed = contract.final_development_gate(summary)
    assert failed["passed"] is False
    assert failed["next_stage"] == "STOP_V11_TERMINAL_NO_RETRY_OR_SWEEP"

    for field, value, check in (
        ("design_audit_passed", False, "design_audit"),
        ("v10s_terminal_failure_passed", False, "v10s_terminal"),
        ("source_checkpoint_scope", "actor_critic_rl_module", "actor_only_source"),
        ("critic_present", True, "actor_only_source"),
        ("critic_parameter_count", 1, "actor_only_source"),
        ("critic_parameter_count", False, "actor_only_source"),
        ("normalization_folded_into_first_layer", False, "folded_actor"),
        ("disabled_clock_columns_0_1_bit_zero", False, "clock_columns_zero"),
    ):
        mutated = _final_development_summary()
        mutated[field] = value
        gate = contract.final_development_gate(mutated)
        assert gate["passed"] is False
        assert gate["checks"][check] is False


def test_unknown_stage_aliases_fail_closed() -> None:
    assert contract.stage_descriptor("freeze_p3") == {
        "kind": "freeze",
        "fit_stage": "p3",
    }
    assert contract.stage_receipt_path("fit_p2") == contract.FIT_RECEIPT_PATHS["p2"]
    try:
        contract.stage_descriptor("collect_r4__deterministic_offset_minus_0p20")
    except ValueError:
        pass
    else:  # pragma: no cover
        raise AssertionError("unknown V11 stage accepted")
