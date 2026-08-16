from __future__ import annotations

import copy
import hashlib
import json
from pathlib import PurePosixPath

import pytest

import h0_primary_split_v12_autonomy_recovery_contract as contract


_FILE_SHA = "ab" * 32


def _artifact(
    path: str | PurePosixPath, *, sha256: str = _FILE_SHA
) -> dict[str, object]:
    return {"path": PurePosixPath(path).as_posix(), "sha256": sha256, "size_bytes": 1}


def _canonical_json_artifact(
    path: str | PurePosixPath, payload: dict[str, object]
) -> dict[str, object]:
    encoded = (
        json.dumps(
            payload,
            indent=2,
            sort_keys=True,
            ensure_ascii=False,
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")
    return {
        "path": PurePosixPath(path).as_posix(),
        "sha256": hashlib.sha256(encoded).hexdigest(),
        "size_bytes": len(encoded),
    }


def _tree(path: str | PurePosixPath) -> dict[str, object]:
    root = PurePosixPath(path)
    file_record = _artifact(root / "weights.bin")
    digest = hashlib.sha256()
    digest.update(str(file_record["path"]).encode("utf-8"))
    digest.update(b"\0")
    digest.update(str(file_record["sha256"]).encode("ascii"))
    digest.update(b"\0")
    digest.update(str(file_record["size_bytes"]).encode("ascii"))
    digest.update(b"\n")
    return {
        "path": root.as_posix(),
        "tree_sha256": digest.hexdigest(),
        "file_count": 1,
        "files": [file_record],
    }


def _source_h0_tree() -> dict[str, object]:
    return {
        "path": contract.SOURCE_H0_MODULE_PATH.as_posix(),
        "tree_sha256": contract.SOURCE_H0_TREE_SHA256,
        "file_count": 3,
        "files": [
            {
                "path": "class_and_ctor_args.pkl",
                "sha256": (
                    "5c98f006d99a71a0f1ddcbb31d8d73fe0a6dade8401e679f6af5b1bc943b4228"
                ),
                "size_bytes": 2_262,
            },
            {
                "path": "metadata.json",
                "sha256": (
                    "3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12"
                ),
                "size_bytes": 197,
            },
            {
                "path": "module_state.pkl",
                "sha256": (
                    "44457ca5df7fa0e0e1f1d361d940136917fe8f71e984a1b0afaefb8ca3ced33b"
                ),
                "size_bytes": 604_772,
            },
        ],
    }


def _passing_probe(stage: str = "p0") -> dict[str, object]:
    case = contract.canonical_probe_case(stage)
    root = contract.PROBE_ROOT / stage
    module = _tree(contract.MODULE_PATHS[stage])
    zero_fields = (
        "action_clipped_values",
        "fallback_count",
        "timeout_count",
        "sea_plugin_fallback_count",
        "so_solver_unaccepted_count",
        "hard_invalid_count",
        "invalid_event_count",
        "nonfinite_count",
        "routing_failure_count",
        "step_contract_failure_count",
        "binary_event_failure_count",
        "physical_gate_bypass_count",
    )
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PURE_PROBE_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_fit_stage": stage,
        "case_id": case["case_id"],
        "action_selection": case["action_selection"],
        "episode_start_offset_s": case["episode_start_offset_s"],
        "action_seed": case["action_seed"],
        "runtime_seed": case["runtime_seed"],
        "sigma": case["sigma"],
        "behavior": contract.PROBE_BEHAVIOR,
        "candidate_module": module,
        "candidate_id": contract.candidate_id(stage, str(module["tree_sha256"])),
        "fit_receipt": _artifact(contract.FIT_RECEIPT_PATHS[stage]),
        "fit_receipt_passed": True,
        "fit_gate_passed": True,
        "worker_claim": _artifact(contract.worker_claim_path(f"probe_{stage}")),
        "run_start": _artifact(root / "run_start.json"),
        "trace": _artifact(root / "trace.json"),
        "partial_summary": _artifact(root / "partial_summary.json"),
        "replay_payload": _artifact(root / "replay_boundaries.npz"),
        "steps": 500,
        "trace_step_count": 500,
        "control_window_count": 5000,
        "raw_sensor_sample_count": 5000,
        "end_reason": "episode_time_limit",
        "terminated": False,
        "truncated": True,
        "safety_stop_count": 0,
        "phase_valid_cycle_count": 2,
        "grf_penetration_max_m": 0.024,
        "candidate_mean_query_count": 500,
        "teacher_enabled": False,
        "teacher_loaded_during_rollout": False,
        "teacher_query_count": 0,
        "served_action_teacher_dependency_count": 0,
        "blending_enabled": False,
        "mean_blend_count": 0,
        "safety_latch_enabled": False,
        "safety_intervention_count": 0,
        "safety_latch_activation_count": 0,
        "safety_latch_release_count": 0,
        "latch_active_at_episode_end": False,
        "random_noise_draw_count": 0,
        "single_noise_application_count": 500,
        "multiple_noise_application_count": 0,
        "noise_application_mismatch_count": 0,
        "offline_teacher_replay_boundary_count": 501,
        "previous_penetration_metadata_count": 500,
        "replay_schema": copy.deepcopy(contract.PROBE_REPLAY_SCHEMA),
        "n_actor": 35,
        "n_observation": 84,
        "observation_dtype": "float32",
        "binary_phase_fsm_mode": "binary_active",
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "target_contract_id": contract.TARGET_CONTRACT_ID,
        "logstd_byte_exact": True,
        "disabled_clock_column_indices": [0, 1],
        "disabled_clock_columns_bit_zero": True,
        "normalization_folded_into_first_layer": True,
        "runtime_normalization_wrapper_present": False,
        "morphology_weight": 0.0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        **{name: 0 for name in zero_fields},
    }


def _failed_physical_probe(stage: str = "p0", *, steps: int = 259) -> dict[str, object]:
    summary = _passing_probe(stage)
    summary.update(
        {
            "steps": steps,
            "trace_step_count": steps,
            "control_window_count": steps * 10,
            "raw_sensor_sample_count": steps * 10,
            "end_reason": "grf_penetration",
            "terminated": True,
            "truncated": False,
            "safety_stop_count": 1,
            "phase_valid_cycle_count": 1,
            "grf_penetration_max_m": 0.0255,
            "candidate_mean_query_count": steps,
            "single_noise_application_count": steps,
            "offline_teacher_replay_boundary_count": steps + 1,
            "previous_penetration_metadata_count": steps,
        }
    )
    return summary


def _probe_evidence(summary: dict[str, object], *, stage: str) -> dict[str, object]:
    result = contract.pure_probe_gate(summary, stage=stage)
    result["receipt"] = _artifact(contract.PROBE_RECEIPT_PATHS[stage])
    result["gate_artifact"] = _artifact(contract.PROBE_ROOT / stage / "gate.json")
    return result


def _passing_label(stage: str, evidence: dict[str, object]) -> dict[str, object]:
    rows = int(evidence["probe_step_count"])
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_fit_stage": stage,
        "probe_passed": evidence["passed"],
        "probe_integrity_passed": True,
        "probe_step_count": rows,
        "probe_candidate_id": evidence["candidate_id"],
        "probe_candidate_module_tree_sha256": evidence["candidate_module_tree_sha256"],
        "probe_trace": evidence["trace"],
        "probe_replay_payload": evidence["replay_payload"],
        "probe_receipt": evidence["receipt"],
        "probe_gate_artifact": evidence["gate_artifact"],
        "probe_replay_schema": copy.deepcopy(contract.PROBE_REPLAY_SCHEMA),
        "labelled_row_count": rows,
        "teacher_query_count": rows,
        "same_state_teacher_label_count": rows,
        "persisted_label_count": rows,
        "replayed_boundary_count": rows + 1,
        "replay_payload_missing_count": 0,
        "teacher_view_changes_only_10_24_count": rows,
        "invariant_columns_byte_exact_count": rows,
        "coverage_distance_count": rows,
        "coverage_reference_observations_sha256": contract.COVERAGE_WEIGHTING[
            "reference_observations_sha256"
        ],
        "coverage_normalization_mean_sha256": contract.COVERAGE_WEIGHTING[
            "normalization_mean_sha256"
        ],
        "coverage_normalization_std_sha256": contract.COVERAGE_WEIGHTING[
            "normalization_std_sha256"
        ],
        "coverage_reference_features_sha256": contract.COVERAGE_WEIGHTING[
            "normalized_feature_matrix_sha256"
        ],
        "coverage_loo_p95": contract.COVERAGE_WEIGHTING["loo_p95"],
        "coverage_new_row_query": copy.deepcopy(
            contract.COVERAGE_WEIGHTING["new_row_query"]
        ),
        "coverage_ood_row_count": min(59, rows),
        "previous_penetration_metadata_count": rows,
        "raw_sample_weight_count": rows,
        "normalized_sample_weight_count": rows,
        "reset_row_count": 1,
        "recovery_weighting": copy.deepcopy(contract.RECOVERY_WEIGHTING),
        "normalized_episode_mass": 500.0,
        "label_corpus": _artifact(contract.LABEL_CORPUS_PATHS[stage]),
        "worker_claim": _artifact(contract.worker_claim_path(f"label_{stage}")),
        "teacher_id": contract.TEACHER_ID,
        "teacher_evidence_id": contract.TEACHER_EVIDENCE_ID,
        "teacher_evidence_receipt": copy.deepcopy(contract.TEACHER_EVIDENCE_ARTIFACT),
        "source_h0": _source_h0_tree(),
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "action_served_count": 0,
        "teacher_loaded_after_probe_closed": True,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def _passing_fit(
    stage: str, labelled: dict[str, int] | None = None
) -> dict[str, object]:
    counts = contract.expected_fit_counts(stage, labelled_probe_rows=labelled)
    module = _tree(contract.MODULE_PATHS[stage])
    fit_artifacts = {
        "corpus": _artifact(contract.FIT_CORPUS_PATHS[stage]),
        "adaptation_history": _artifact(
            contract.FIT_ROOTS[stage] / "adaptation_history.json"
        ),
        "adaptation_report": _artifact(
            contract.FIT_ROOTS[stage] / "adaptation_report.json"
        ),
    }
    labelled_rows = {} if labelled is None else labelled
    probe_label_receipts = []
    for probe_stage in counts["pure_probe_label_stages"]:
        label_corpus = _artifact(contract.LABEL_CORPUS_PATHS[probe_stage])
        receipt_payload = {
            "schema_version": contract.SCHEMA_VERSION,
            "status": contract.OBSERVER_LABEL_PASS_STATUS,
            "passed": True,
            "protocol_id": contract.PROTOCOL_ID,
            "fit_stage": probe_stage,
            "labelled_row_count": labelled_rows[probe_stage],
            "label_corpus": label_corpus,
            "pipeline_claim": _artifact(contract.PIPELINE_CLAIM_PATH),
            "worker_claim": _artifact(
                contract.worker_claim_path(f"label_{probe_stage}")
            ),
        }
        probe_label_receipts.append(
            {
                "fit_stage": probe_stage,
                "receipt": _canonical_json_artifact(
                    contract.LABEL_RECEIPT_PATHS[probe_stage], receipt_payload
                ),
                "receipt_payload": receipt_payload,
                "label_corpus": label_corpus,
                "labelled_row_count": labelled_rows[probe_stage],
                "worker_claim": receipt_payload["worker_claim"],
                "passed": True,
            }
        )
    collection_corpus_receipts = []
    for round_index in counts["completed_v12_collection_rounds"]:
        for case_id in contract.COLLECTION_CASE_IDS:
            destination = PurePosixPath(
                contract.canonical_collection_case(case_id, round_index)["destination"]
            )
            label_corpus = _artifact(destination / "labels.npz")
            receipt_payload = {
                "schema_version": contract.SCHEMA_VERSION,
                "status": contract.COLLECTION_PASS_STATUS,
                "passed": True,
                "protocol_id": contract.PROTOCOL_ID,
                "round_index": round_index,
                "case_id": case_id,
                "sample_count": contract.EXPECTED_STEPS,
                "label_corpus": label_corpus,
                "pipeline_claim": _artifact(contract.PIPELINE_CLAIM_PATH),
                "worker_claim": _artifact(
                    contract.worker_claim_path(f"collect_r{round_index}__{case_id}")
                ),
            }
            collection_corpus_receipts.append(
                {
                    "round_index": round_index,
                    "case_id": case_id,
                    "receipt": _canonical_json_artifact(
                        contract.stage_receipt_path(
                            f"collect_r{round_index}__{case_id}"
                        ),
                        receipt_payload,
                    ),
                    "receipt_payload": receipt_payload,
                    "label_corpus": label_corpus,
                    "worker_claim": receipt_payload["worker_claim"],
                    "data_passed": True,
                }
            )
    episodes = (
        12
        + 2 * len(counts["completed_v12_collection_rounds"])
        + len(counts["pure_probe_label_stages"])
    )
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.FIT_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "fit_stage": stage,
        "fit_contract_id": contract.FIT_CONTRACT_ID,
        "fit": copy.deepcopy(contract.FIT),
        "actor_architecture": copy.deepcopy(contract.ACTOR_ARCHITECTURE),
        "normalization": copy.deepcopy(contract.BASE_CORPUS_NORMALIZATION),
        "trainable_scope": contract.FIT["trainable_scope"],
        "source_h0_id": contract.SOURCE_H0_ID,
        "source_h0": _source_h0_tree(),
        "initial_checkpoint_id": contract.SOURCE_H0_ID,
        "continued_from_previous_candidate": False,
        "candidate_module": module,
        "candidate_id": contract.candidate_id(stage, str(module["tree_sha256"])),
        "component_receipts": {
            "design_audit": _artifact(contract.DESIGN_AUDIT_RECEIPT_PATH),
            "execution_lock": _artifact(contract.EXECUTION_LOCK_PATH),
            "pipeline_claim": _artifact(contract.PIPELINE_CLAIM_PATH),
            "worker_claim": _artifact(contract.worker_claim_path(f"fit_{stage}")),
        },
        "design_audit_passed": True,
        "execution_lock_passed": True,
        "pipeline_claimed": True,
        "teacher_evidence_id": contract.TEACHER_EVIDENCE_ID,
        "teacher_evidence_passed": True,
        "teacher_evidence_receipt": copy.deepcopy(contract.TEACHER_EVIDENCE_ARTIFACT),
        "v11_seed_corpus": copy.deepcopy(contract.V11_P3_CORPUS_ARTIFACT),
        "v11_seed_corpus_audit_passed": True,
        "collection_corpus_receipts": collection_corpus_receipts,
        "probe_label_receipts": probe_label_receipts,
        "fit_artifacts": fit_artifacts,
        "corpus_artifact": fit_artifacts["corpus"],
        "fit_counts": counts,
        "sample_count": counts["sample_count"],
        "reset_row_count": counts["reset_row_count"],
        "labelled_probe_rows": labelled_rows,
        "corpus_audit": {
            "v11_seed_sample_count": counts["v11_seed_sample_count"],
            "v12_dagger_sample_count": counts["v12_dagger_sample_count"],
            "same_state_v12_dagger_sample_count": counts["v12_dagger_sample_count"],
            "pure_probe_label_sample_count": counts["pure_probe_label_sample_count"],
            "same_state_pure_probe_label_sample_count": counts[
                "pure_probe_label_sample_count"
            ],
            "sample_count": counts["sample_count"],
            "reset_row_count": counts["reset_row_count"],
            "duplicate_sample_count": 0,
            "all_finite": True,
        },
        "report_checks": {name: True for name in contract.FIT_REPORT_CHECK_NAMES},
        "adamw_epochs_run": contract.FIT["adamw"]["epochs"],
        "lbfgs_completed": True,
        "deterministic_algorithms_enabled": True,
        "metrics": {
            "rmse": 0.005,
            "max_abs_error": 0.05,
            "reset_max_abs_error": 0.002,
        },
        "episode_count": episodes,
        "episode_target_mass": 500.0,
        "normalized_total_sample_mass": episodes * 500.0,
        "recovery_weighting": copy.deepcopy(contract.RECOVERY_WEIGHTING),
        "coverage_weighting": copy.deepcopy(contract.COVERAGE_WEIGHTING),
        "shielded_nonreset_raw_weight": 1.0,
        "shielded_reset_raw_weight": 100.0,
        "row_loss": contract.RECOVERY_WEIGHTING["row_loss"],
        "corpus_loss_reduction": contract.RECOVERY_WEIGHTING["corpus_loss_reduction"],
        "disabled_clock_column_indices": [0, 1],
        "disabled_clock_columns_bit_zero": True,
        "disabled_clock_columns_bit_zero_after_save_reload": True,
        "normalization_stats_from_base_corpus_only": True,
        "normalization_stats_frozen_across_stages": True,
        "normalization_folded_into_first_layer": True,
        "runtime_normalization_wrapper_present": False,
        "prescribed_clock_present": False,
        "fold_equivalence_passed": True,
        "anchor_used": False,
        "hard_polish_used": False,
        "design_audit_reproduction_within_tolerance": True,
        "source_checkpoint_scope": "actor_only_rl_module",
        "critic_present": False,
        "critic_parameter_count": 0,
        "critic_byte_exact": True,
        "logstd_byte_exact": True,
        "duplicate_sample_count": 0,
        "all_finite": True,
        "source_h0_byte_exact": True,
        "target_contract_id": contract.TARGET_CONTRACT_ID,
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def _passing_collection(
    round_index: int = 1, case_id: str | None = None
) -> dict[str, object]:
    case_id = contract.COLLECTION_CASE_IDS[0] if case_id is None else case_id
    case = contract.canonical_collection_case(case_id, round_index)
    stage = str(case["candidate_fit_stage"])
    root = PurePosixPath(str(case["destination"]))
    draws = 500 if case["action_selection"] == "stochastic" else 0
    zero_fields = (
        "action_clipped_values",
        "fallback_count",
        "timeout_count",
        "safety_stop_count",
        "sea_plugin_fallback_count",
        "so_solver_unaccepted_count",
        "hard_invalid_count",
        "invalid_event_count",
        "nonfinite_count",
        "routing_failure_count",
        "step_contract_failure_count",
        "binary_event_failure_count",
        "physical_gate_bypass_count",
        "safety_latch_rule_violation_count",
        "alpha_mismatch_count",
        "mean_blend_mismatch_count",
        "noise_application_mismatch_count",
        "multiple_noise_application_count",
    )
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.COLLECTION_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "case_id": case_id,
        "round_index": round_index,
        "candidate_fit_stage": stage,
        "requested_alpha": case["requested_alpha"],
        "action_selection": case["action_selection"],
        "episode_start_offset_s": case["episode_start_offset_s"],
        "action_seed": case["action_seed"],
        "runtime_seed": case["runtime_seed"],
        "sigma": case["sigma"],
        "candidate_module": _tree(contract.MODULE_PATHS[stage]),
        "candidate_fit_receipt": _artifact(contract.FIT_RECEIPT_PATHS[stage]),
        "candidate_fit_gate_passed": True,
        "prior_label_receipt": _artifact(contract.LABEL_RECEIPT_PATHS[stage]),
        "prior_label_gate_passed": True,
        "sample_count": 500,
        "teacher_query_count": 500,
        "persisted_label_count": 500,
        "candidate_mean_query_count": 500,
        "same_state_teacher_label_count": 500,
        "candidate_selected_before_teacher_count": 500,
        "served_action_teacher_dependency_count": 500,
        "teacher_id": contract.TEACHER_ID,
        "teacher_evidence_id": contract.TEACHER_EVIDENCE_ID,
        "teacher_evidence_receipt": copy.deepcopy(contract.TEACHER_EVIDENCE_ARTIFACT),
        "source_h0": _source_h0_tree(),
        "mean_blend_count": 500,
        "blend_before_noise_count": 500,
        "noise_before_blend_count": 0,
        "random_noise_draw_count": draws,
        "single_noise_application_count": 500,
        "safety_latch_activation_m": contract.SAFETY_LATCH_ACTIVATION_M,
        "safety_latch_release_m": contract.SAFETY_LATCH_RELEASE_M,
        "safety_latch_release_phase": contract.SAFETY_LATCH_RELEASE_PHASE,
        "safety_signal_lag_steps": 1,
        "safety_intervention_diagnostic_only": True,
        "forced_teacher_takeover_count": 0,
        "forced_teacher_takeover_fraction": 0.0,
        "max_consecutive_takeover_steps": 0,
        "latch_active_at_episode_end": False,
        "collection_is_data_only": True,
        "autonomy_claimed": False,
        "steps": 500,
        "control_window_count": 5000,
        "raw_sensor_sample_count": 5000,
        "end_reason": "episode_time_limit",
        "terminated": False,
        "truncated": True,
        "phase_valid_cycle_count": 2,
        "grf_penetration_max_m": 0.024,
        "n_actor": 35,
        "n_observation": 84,
        "observation_dtype": "float32",
        "binary_phase_fsm_mode": "binary_active",
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "target_contract_id": contract.TARGET_CONTRACT_ID,
        "morphology_weight": 0.0,
        "run_start": _artifact(root / "run_start.json"),
        "trace": _artifact(root / "trace.json"),
        "partial_summary": _artifact(root / "partial_summary.json"),
        "label_corpus": _artifact(root / "labels.npz"),
        "worker_claim": _artifact(
            contract.worker_claim_path(f"collect_r{round_index}__{case_id}")
        ),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        **{name: 0 for name in zero_fields},
    }


def test_identity_authority_and_topology_are_design_only() -> None:
    assert contract.SCHEMA_VERSION == 120
    assert contract.AUTHORITY["design_and_freeze_authorized"] is True
    assert len(contract.STAGE_IDS) == 26
    for stage in contract.FIT_STAGES:
        fit_index = contract.STAGE_IDS.index(f"fit_{stage}")
        assert contract.STAGE_IDS[fit_index + 1 : fit_index + 3] == (
            f"probe_{stage}",
            f"label_{stage}",
        )
    assert contract.RUN_ROOT.as_posix().startswith(
        "Trajectory Generator/baseline_MLP/validation/h0_v12_runs/"
    )


def test_v11_inputs_and_coverage_receipt_are_fully_bound() -> None:
    assert len(contract.V11_COLLECTION_INPUTS) == 6
    assert len(contract.INPUT_RELATIVE_PATHS) == 35
    assert contract.INPUT_RELATIVE_PATHS["v11_final_failure_trace"] == (
        contract.V11_FINAL_FAILURE_TRACE_PATH.as_posix()
    )
    assert contract.COVERAGE_WEIGHTING["loo_p95"] == 0.07945888479650812
    assert contract.COVERAGE_WEIGHTING["included_feature_indices"] == list(range(2, 35))
    assert contract.COVERAGE_WEIGHTING["tie_audit"] == {
        "extended_query_k": 64,
        "unique_observation_count": 5990,
        "maximum_minimum_distance_tie_count": 6,
        "query_k_matches_extended_query": True,
    }
    assert "FAIL_CLOSED" in contract.COVERAGE_WEIGHTING["target_platform_policy"]


def test_weighting_is_causal_boundary_exact_and_mass_normalized() -> None:
    p95 = contract.COVERAGE_WEIGHTING["loo_p95"]
    assert contract.recovery_sample_weight(0.010, coverage_distance_rms_z=p95) == 1.0
    assert contract.recovery_sample_weight(0.0125) == pytest.approx(50.5)
    assert contract.recovery_sample_weight(0.015) == 100.0
    assert contract.recovery_sample_weight(0.0, reset_row=True) == 100.0
    assert (
        contract.recovery_sample_weight(0.0, coverage_distance_rms_z=p95 + 1.0e-12)
        == 100.0
    )
    assert contract.shielded_sample_weight() == 1.0
    assert contract.shielded_sample_weight(reset_row=True) == 100.0
    weights = contract.normalize_episode_sample_weights([1.0, 100.0, 1.0])
    assert sum(weights) == pytest.approx(500.0)
    assert contract.weighted_mean_row_loss([1.0, 3.0], [1.0, 3.0]) == 2.5
    with pytest.raises(ValueError):
        contract.recovery_sample_weight(-1.0)
    with pytest.raises(ValueError):
        contract.normalize_episode_sample_weights([])


def test_fit_counts_and_fit_gate_are_exact() -> None:
    assert contract.expected_fit_counts("p0")["sample_count"] == 6000
    labelled = {"p0": 259, "p1": 500, "p2": 411}
    assert (
        contract.expected_fit_counts("p3", labelled_probe_rows=labelled)["sample_count"]
        == 10170
    )
    summary = _passing_fit("p3", labelled)
    assert contract.fit_gate(summary, stage="p3")["passed"] is True
    mutated = copy.deepcopy(summary)
    mutated["continued_from_previous_candidate"] = True
    assert contract.fit_gate(mutated, stage="p3")["passed"] is False
    malformed = copy.deepcopy(summary)
    malformed["candidate_module"]["tree_sha256"] = "not-a-hash"
    assert contract.fit_gate(malformed, stage="p3")["passed"] is False
    unbound_rows = copy.deepcopy(summary)
    unbound_rows["probe_label_receipts"][0]["labelled_row_count"] += 1
    assert contract.fit_gate(unbound_rows, stage="p3")["passed"] is False
    incomplete_optimizer = copy.deepcopy(summary)
    incomplete_optimizer["lbfgs_completed"] = False
    assert contract.fit_gate(incomplete_optimizer, stage="p3")["passed"] is False
    missing_lock = copy.deepcopy(summary)
    missing_lock["component_receipts"]["execution_lock"]["path"] = "wrong.json"
    assert contract.fit_gate(missing_lock, stage="p3")["passed"] is False
    wrong_seed = copy.deepcopy(summary)
    wrong_seed["v11_seed_corpus"]["sha256"] = "cd" * 32
    assert contract.fit_gate(wrong_seed, stage="p3")["passed"] is False
    negative_metrics = copy.deepcopy(summary)
    negative_metrics["metrics"] = {
        "rmse": -1.0,
        "max_abs_error": -1.0,
        "reset_max_abs_error": -1.0,
    }
    assert contract.fit_gate(negative_metrics, stage="p3")["passed"] is False
    irrelevant_report = copy.deepcopy(summary)
    irrelevant_report["report_checks"] = {"irrelevant": True}
    assert contract.fit_gate(irrelevant_report, stage="p3")["passed"] is False
    unbound_probe_corpus = copy.deepcopy(summary)
    unbound_probe_corpus["probe_label_receipts"][0]["label_corpus"]["sha256"] = (
        "cd" * 32
    )
    assert contract.fit_gate(unbound_probe_corpus, stage="p3")["passed"] is False
    unbound_collection_corpus = copy.deepcopy(summary)
    unbound_collection_corpus["collection_corpus_receipts"][0]["label_corpus"][
        "sha256"
    ] = "cd" * 32
    assert contract.fit_gate(unbound_collection_corpus, stage="p3")["passed"] is False
    inconsistent_pipeline_claim = copy.deepcopy(summary)
    probe_binding = inconsistent_pipeline_claim["probe_label_receipts"][0]
    probe_binding["receipt_payload"]["pipeline_claim"]["sha256"] = "cd" * 32
    probe_binding["receipt"] = _canonical_json_artifact(
        contract.LABEL_RECEIPT_PATHS["p0"], probe_binding["receipt_payload"]
    )
    assert contract.fit_gate(inconsistent_pipeline_claim, stage="p3")["passed"] is False
    extra_probe_field = copy.deepcopy(summary)
    probe_binding = extra_probe_field["probe_label_receipts"][0]
    probe_binding["receipt_payload"]["contradictory"] = True
    probe_binding["receipt"] = _canonical_json_artifact(
        contract.LABEL_RECEIPT_PATHS["p0"], probe_binding["receipt_payload"]
    )
    assert contract.fit_gate(extra_probe_field, stage="p3")["passed"] is False
    extra_collection_field = copy.deepcopy(summary)
    collection_binding = extra_collection_field["collection_corpus_receipts"][0]
    collection_binding["receipt_payload"]["contradictory"] = True
    collection_binding["receipt"] = _canonical_json_artifact(
        contract.stage_receipt_path(
            f"collect_r{collection_binding['round_index']}__"
            f"{collection_binding['case_id']}"
        ),
        collection_binding["receipt_payload"],
    )
    assert contract.fit_gate(extra_collection_field, stage="p3")["passed"] is False
    wrong_h0 = copy.deepcopy(summary)
    wrong_h0["source_h0"] = _tree(contract.SOURCE_H0_MODULE_PATH)
    assert contract.fit_gate(wrong_h0, stage="p3")["passed"] is False
    wrong_teacher_evidence = copy.deepcopy(summary)
    wrong_teacher_evidence["teacher_evidence_receipt"]["sha256"] = "cd" * 32
    assert contract.fit_gate(wrong_teacher_evidence, stage="p3")["passed"] is False


def test_cases_descriptors_claims_receipts_and_mutations_are_complete() -> None:
    for stage in contract.FIT_STAGES:
        assert contract.stage_descriptor(f"probe_{stage}")["kind"] == "probe"
        assert contract.stage_descriptor(f"label_{stage}")["kind"] == "label"
    paths = contract.declared_mutation_paths()
    assert len(paths) == len(set(paths.values()))
    for stage_id in contract.STAGE_IDS:
        assert contract.worker_claim_path(stage_id) in paths.values()
        assert contract.stage_receipt_path(stage_id) in paths.values()
    assert all(
        path
        in {
            contract.PROTOCOL_FREEZE_PATH,
            contract.EXECUTION_LOCK_PATH,
            contract.DESIGN_AUDIT_RECEIPT_PATH,
        }
        or path == contract.RUN_ROOT
        or contract.RUN_ROOT in path.parents
        for path in paths.values()
    )


def test_probe_integrity_is_separate_from_recoverable_autonomy_failure() -> None:
    passing = _passing_probe("p0")
    assert contract.probe_integrity_gate(passing, stage="p0")["passed"] is True
    assert contract.pure_probe_gate(passing, stage="p0")["passed"] is True

    failed = _failed_physical_probe("p0")
    gate = contract.pure_probe_gate(failed, stage="p0")
    assert gate["integrity_passed"] is True
    assert gate["passed"] is False
    assert gate["recoverable_for_data_collection"] is True
    assert gate["next_stage"].startswith("OBSERVER_LABEL_P0_REQUIRED")

    corrupted = copy.deepcopy(failed)
    corrupted["teacher_loaded_during_rollout"] = True
    gate = contract.pure_probe_gate(corrupted, stage="p0")
    assert gate["integrity_passed"] is False
    assert gate["recoverable_for_data_collection"] is False
    assert gate["next_stage"] == "STOP_V12_PROBE_INTEGRITY_FAILURE"

    p3 = contract.pure_probe_gate(_failed_physical_probe("p3"), stage="p3")
    assert p3["integrity_passed"] is True
    assert p3["next_stage"] == "STOP_V12_TERMINAL"


def test_observer_labels_are_bound_to_exact_probe_prefix() -> None:
    early_evidence = _probe_evidence(_failed_physical_probe("p0"), stage="p0")
    summary = _passing_label("p0", early_evidence)
    gate = contract.observer_label_gate(
        summary, stage="p0", probe_evidence=early_evidence
    )
    assert gate["passed"] is True
    assert gate["next_stage"] == "SHIELDED_COLLECTION_R1_DATA_ONLY"

    p3_evidence = _probe_evidence(_passing_probe("p3"), stage="p3")
    p3 = _passing_label("p3", p3_evidence)
    assert (
        contract.observer_label_gate(p3, stage="p3", probe_evidence=p3_evidence)[
            "next_stage"
        ]
        == "FREEZE_P3"
    )
    p3["labelled_row_count"] = 259
    assert (
        contract.observer_label_gate(p3, stage="p3", probe_evidence=p3_evidence)[
            "passed"
        ]
        is False
    )

    failed_p3_evidence = _probe_evidence(_failed_physical_probe("p3"), stage="p3")
    invalid = _passing_label("p3", failed_p3_evidence)
    assert (
        contract.observer_label_gate(
            invalid, stage="p3", probe_evidence=failed_p3_evidence
        )["passed"]
        is False
    )


def test_collection_data_and_latch_gates_fail_closed_independently() -> None:
    summary = _passing_collection()
    data_gate = contract.collection_data_gate(summary, round_index=1)
    assert data_gate["passed"] is True
    latch = contract.latch_dependence_gate(
        summary, collection_data_passed=data_gate["passed"]
    )
    assert latch["passed"] is True
    assert latch["collection_data_reusable"] is True

    dependent = copy.deepcopy(summary)
    dependent.update(
        {
            "forced_teacher_takeover_count": 253,
            "forced_teacher_takeover_fraction": 253 / 500,
            "max_consecutive_takeover_steps": 90,
        }
    )
    latch = contract.latch_dependence_gate(dependent, collection_data_passed=True)
    assert latch["passed"] is False
    assert latch["collection_data_reusable"] is True

    malformed = copy.deepcopy(summary)
    malformed["forced_teacher_takeover_fraction"] = float("nan")
    data_gate = contract.collection_data_gate(malformed, round_index=1)
    assert data_gate["passed"] is False
    latch = contract.latch_dependence_gate(
        malformed, collection_data_passed=data_gate["passed"]
    )
    assert latch["collection_data_rejected"] is True
    assert latch["next_stage"] == "STOP_V12_COLLECTION_INTEGRITY_FAILURE"


def test_candidate_and_final_gates_require_complete_provenance() -> None:
    module = _tree(contract.MODULE_PATHS["p3"])
    freeze_summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "protocol_id": contract.PROTOCOL_ID,
        "fit_contract_id": contract.FIT_CONTRACT_ID,
        "candidate_fit_stage": "p3",
        "candidate_module": module,
        "candidate_id": contract.candidate_id("p3", str(module["tree_sha256"])),
        "fit_receipts": [
            {
                "fit_stage": stage,
                "receipt": _artifact(contract.FIT_RECEIPT_PATHS[stage]),
                "passed": True,
            }
            for stage in contract.FIT_STAGES
        ],
        "probe_receipts": [
            {
                "fit_stage": stage,
                "receipt": _artifact(contract.PROBE_RECEIPT_PATHS[stage]),
                "integrity_passed": True,
                "autonomy_passed": stage == "p3",
                "recoverable_for_data_collection": stage != "p3",
            }
            for stage in contract.FIT_STAGES
        ],
        "label_receipts": [
            {
                "fit_stage": stage,
                "receipt": _artifact(contract.LABEL_RECEIPT_PATHS[stage]),
                "passed": True,
            }
            for stage in contract.FIT_STAGES
        ],
        "collection_receipts": [
            {
                "round_index": round_index,
                "case_id": case_id,
                "receipt": _artifact(
                    contract.stage_receipt_path(f"collect_r{round_index}__{case_id}")
                ),
                "data_passed": True,
                "latch_independence_passed": False,
            }
            for round_index in (1, 2, 3)
            for case_id in contract.COLLECTION_CASE_IDS
        ],
        "pipeline_claim": _artifact(contract.PIPELINE_CLAIM_PATH),
        "worker_claim": _artifact(contract.worker_claim_path("freeze_p3")),
        "p3_fit_passed": True,
        "p3_probe_integrity_passed": True,
        "p3_probe_autonomy_passed": True,
        "p3_label_passed": True,
        "source_h0": _source_h0_tree(),
        "design_audit_receipt": _artifact(contract.DESIGN_AUDIT_RECEIPT_PATH),
        "design_audit_passed": True,
        "execution_lock": _artifact(contract.EXECUTION_LOCK_PATH),
        "fit_actor_update_count": 4,
        "every_fit_restarted_from_h0": True,
        "source_checkpoint_scope": "actor_only_rl_module",
        "critic_present": False,
        "critic_parameter_count": 0,
        "logstd_byte_exact": True,
        "normalization_folded_into_first_layer": True,
        "runtime_normalization_wrapper_present": False,
        "disabled_clock_columns_0_1_bit_zero": True,
        "candidate_frozen": True,
        "runtime_promoted": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "retry_authorized": False,
        "sweep_authorized": False,
        "rescue_authorized": False,
    }
    assert contract.candidate_freeze_gate(freeze_summary)["passed"] is True
    failed_fit = copy.deepcopy(freeze_summary)
    failed_fit["fit_receipts"][0]["passed"] = False
    assert contract.candidate_freeze_gate(failed_fit)["passed"] is False
    freeze_summary["p3_probe_autonomy_passed"] = False
    assert contract.candidate_freeze_gate(freeze_summary)["passed"] is False


def test_final_rollout_and_aggregate_remain_development_only() -> None:
    case = contract.canonical_final_case(contract.FINAL_CASE_IDS[0])
    root = PurePosixPath(case["destination"])
    module = _tree(contract.MODULE_PATHS["p3"])
    zero_fields = (
        "action_clipped_values",
        "fallback_count",
        "timeout_count",
        "safety_stop_count",
        "sea_plugin_fallback_count",
        "so_solver_unaccepted_count",
        "hard_invalid_count",
        "invalid_event_count",
        "nonfinite_count",
        "routing_failure_count",
        "step_contract_failure_count",
        "binary_event_failure_count",
        "physical_gate_bypass_count",
    )
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.FINAL_ROLLOUT_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        **{
            key: case[key]
            for key in (
                "case_id",
                "action_selection",
                "episode_start_offset_s",
                "action_seed",
                "runtime_seed",
                "sigma",
            )
        },
        "candidate_module": module,
        "candidate_id": contract.candidate_id("p3", str(module["tree_sha256"])),
        "candidate_freeze": _artifact(contract.CANDIDATE_FREEZE_PATH),
        "candidate_freeze_passed": True,
        "teacher_enabled": False,
        "teacher_loaded_during_rollout": False,
        "teacher_query_count": 0,
        "served_action_teacher_dependency_count": 0,
        "blending_enabled": False,
        "mean_blend_count": 0,
        "safety_latch_enabled": False,
        "safety_intervention_count": 0,
        "random_noise_draw_count": 0,
        "single_noise_application_count": 500,
        "multiple_noise_application_count": 0,
        "noise_application_mismatch_count": 0,
        "steps": 500,
        "control_window_count": 5000,
        "raw_sensor_sample_count": 5000,
        "end_reason": "episode_time_limit",
        "terminated": False,
        "truncated": True,
        "phase_valid_cycle_count": 2,
        "grf_penetration_max_m": 0.024,
        "n_actor": 35,
        "n_observation": 84,
        "observation_dtype": "float32",
        "binary_phase_fsm_mode": "binary_active",
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "target_contract_id": contract.TARGET_CONTRACT_ID,
        "morphology_weight": 0.0,
        "run_start": _artifact(root / "run_start.json"),
        "trace": _artifact(root / "trace.json"),
        "partial_summary": _artifact(root / "partial_summary.json"),
        "worker_claim": _artifact(
            contract.worker_claim_path(f"final__{case['case_id']}")
        ),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        **{name: 0 for name in zero_fields},
    }
    assert contract.final_rollout_gate(summary)["passed"] is True
    bindings = []
    for final_case in contract.FINAL_CASES:
        destination = PurePosixPath(final_case["destination"])
        bindings.append(
            {
                "case_id": final_case["case_id"],
                "passed": True,
                "receipt": _artifact(destination / "receipt.json"),
                "gate": _artifact(destination / "gate.json"),
                "summary": _artifact(destination / "summary.json"),
            }
        )
    aggregate = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.FINAL_DEVELOPMENT_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_module": module,
        "candidate_id": contract.candidate_id("p3", str(module["tree_sha256"])),
        "candidate_freeze": _artifact(contract.CANDIDATE_FREEZE_PATH),
        "candidate_freeze_passed": True,
        "pipeline_claim": _artifact(contract.PIPELINE_CLAIM_PATH),
        "worker_claim": _artifact(contract.worker_claim_path("finalize_development")),
        "rollout_bindings": bindings,
        "rollout_count": 6,
        "passing_rollout_count": 6,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "development_only": True,
        "runtime_promoted": False,
        "qualification_required": True,
        "retry_authorized": False,
        "sweep_authorized": False,
        "rescue_authorized": False,
    }
    gate = contract.final_development_gate(aggregate)
    assert gate["passed"] is True
    assert gate["next_stage"] == "WAIT_INDEPENDENT_QUALIFICATION_PROTOCOL"
