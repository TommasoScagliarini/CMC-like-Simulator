from __future__ import annotations

import copy

import h0_v12r5_case_balanced_contract as contract


def _metric(value: float = 0.001) -> dict[str, float]:
    return {
        "rmse": value,
        "max_abs_error": value,
        "reset_max_abs_error": value,
    }


def _pure_trace(case_id: str) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for step in range(1, contract.EXPECTED_STEPS + 1):
        rows.append(
            {
                "step": step,
                "schema_version": contract.SCHEMA_VERSION,
                "protocol_id": contract.PROTOCOL_ID,
                "stage_id": f"development__{case_id}",
                "case_id": case_id,
                "candidate_mean": [0.1, -0.2],
                "single_noise": [0.01, 0.02],
                "raw_action": [0.11, -0.18],
                "teacher_enabled": False,
                "blending_enabled": False,
                "safety_latch_enabled": False,
                "raw_sensor_sample_count": 10,
                "observer_raw_sensor_journal": {"samples": [{} for _ in range(10)]},
                **{name: 0 for name in contract.PURE_POLICY_COUNTER_FIELDS},
            }
        )
    return rows


def test_contract_self_check_and_no_collection_authority_are_exact() -> None:
    result = contract.contract_self_check()
    assert result["passed"] is True
    assert contract.REVISION == "2026-08-09"
    assert contract.AUTHORITY_TEXT == "esegui i punti 1-6"
    assert contract.AUTHORITY["new_environment_collection_authorized"] is False
    assert contract.AUTHORITY["qualification_execution_authorized"] is False
    assert contract.AUTHORITY["retry_authorized"] is False
    assert contract.AUTHORITY["runtime_promotion_authorized"] is False


def test_stage_order_is_source_only_one_fit_and_critical_first() -> None:
    assert len(contract.STAGE_IDS) == 11
    assert contract.STAGE_IDS[:4] == (
        "attest_locked_inputs",
        "assemble_case_balanced_corpus",
        "fit_case_balanced_candidate",
        "freeze_case_balanced_candidate",
    )
    assert contract.DEVELOPMENT_CASE_IDS == (
        "deterministic_offset_plus_0p20",
        "deterministic_offset_nominal",
        "deterministic_offset_minus_0p20",
        "stochastic_nominal_seed_126",
        "stochastic_nominal_seed_127",
        "stochastic_nominal_seed_128",
    )
    assert all(
        contract.stage_descriptor(stage_id)["kind"] != "collection"
        for stage_id in contract.STAGE_IDS
    )
    assert contract.STAGE_IDS[-1] == "finalize_development"


def test_corpus_weighting_fit_and_selection_are_frozen() -> None:
    counts = contract.expected_corpus_counts()
    assert counts == {
        "p2_sample_count": 8732,
        "nominal_pass_sample_count": 500,
        "nominal_student_exposed_sample_count": 255,
        "failed_plus_prefix_sample_count": 0,
        "sample_count": 9232,
        "episode_count": 19,
        "reset_row_count": 19,
        "case_count": 6,
        "case_target_mass": 1000.0,
        "normalized_total_sample_mass": 6000.0,
        "component_order": ["p2_corpus", "r4_nominal_pass_labels_only"],
    }
    assert contract.FIT["initial_checkpoint_id"] == contract.SOURCE_H0_ID
    assert contract.FIT["continued_from_p2"] is False
    assert contract.FIT["adamw"]["epochs"] == 3000
    assert contract.FIT["lbfgs"]["max_iter"] == 600
    assert contract.FIT["lbfgs"]["max_eval"] == 1200
    assert contract.WEIGHTING["case_target_mass"] == 1000.0
    assert contract.WEIGHTING["normalized_total_mass"] == 6000.0
    assert contract.CANDIDATE_SELECTION_RULE == (
        "SOLE_CASE_BALANCED_FIT_OUTPUT_FROM_LOCKED_R5_RUN"
    )
    assert contract.CANDIDATE_MODULE_PATH.as_posix().endswith(
        "h0_v12r5_run_20260809/fit/rl_module_target_adapted"
    )


def test_predecessor_q2_and_q3_bindings_are_exact() -> None:
    assert set(contract.R4_NOMINAL_REUSABLE_ARTIFACTS) == {
        "labels",
        "receipt",
        "gate",
        "summary",
        "trace",
    }
    assert set(contract.R4_PLUS_FAILURE_EVIDENCE) == {
        "labels_excluded",
        "failure",
        "gate",
        "summary",
    }
    assert contract.Q2_DESIGN_FREEZE_ARTIFACT["sha256"] == (
        "d92fac765bd192f43ef2e0420a8529e8bb21860a3f47420ab5948863fb53eaf5"
    )
    assert contract.Q3_DESIGN_FREEZE_ARTIFACT == {
        "path": contract.Q3_DESIGN_FREEZE_PATH.as_posix(),
        "sha256": "f2764f2cf16abfc168255056fdf0c1407d97c65100bb3965b866f38db084e56d",
        "size_bytes": 68465,
    }
    assert contract.Q3_DESIGN_FREEZE_PATH not in contract.Q3_UNOPENED_PATHS.values()
    assert contract.Q3_PREREQUISITES["terminal_ledger"] == {
        "path": contract.PIPELINE_LEDGER_PATH.as_posix(),
        "status": contract.PIPELINE_PASS_STATUS,
    }


def test_fit_gate_requires_every_registered_slice_and_critical_nonregression() -> None:
    hashes = {
        "p2_max_abs_error_sha256": contract.WEIGHTING[
            "expected_p2_max_abs_error_sha256"
        ],
        "hardness_sha256": contract.WEIGHTING["expected_hardness_sha256"],
        "source_risk_sha256": contract.WEIGHTING["expected_source_risk_sha256"],
        "normalized_weights_sha256": contract.WEIGHTING[
            "expected_normalized_weights_sha256"
        ],
    }
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.FIT_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "fit": copy.deepcopy(contract.FIT),
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "initial_checkpoint_id": contract.SOURCE_H0_ID,
        "continued_from_p2": False,
        "fit_counts": contract.expected_corpus_counts(),
        "sample_count": 9232,
        "episode_count": 19,
        "reset_row_count": 19,
        "failed_plus_prefix_rows_loaded": 0,
        "corpus_components": ["p2_corpus", "r4_nominal_pass_labels_only"],
        "weighting": copy.deepcopy(contract.WEIGHTING),
        "weight_hashes": hashes,
        "case_mass": {case_id: 1000.0 for case_id in contract.CASE_IDS},
        "normalized_total_sample_mass": 6000.0,
        "metrics": _metric(),
        "p2_subset_metrics": _metric(),
        "nominal_r4_pass_metrics": _metric(),
        "nominal_r4_student_exposed_metrics": _metric(),
        "per_case_metrics": {case_id: _metric() for case_id in contract.CASE_IDS},
        "critical_window": copy.deepcopy(contract.CRITICAL_WINDOW),
        "critical_window_metrics": {"rmse": 0.004, "max_abs_error": 0.03},
        "critical_window_p2_baseline_metrics": {
            "rmse": 0.005,
            "max_abs_error": 0.04,
        },
        "critical_window_p2_baseline_recomputed": True,
        "critical_window_p2_module_tree": copy.deepcopy(contract.P2_MODULE_TREE),
        "adamw_epochs_run": 3000,
        "lbfgs_max_iter": 600,
        "lbfgs_max_eval": 1200,
        "deterministic_algorithms_enabled": True,
        "source_h0_byte_exact": True,
        "logstd_byte_exact": True,
        "critic_present": False,
        "disabled_clock_columns_bit_zero": True,
        "save_reload_exact": True,
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "hard_polish_used": False,
        "fallback_used": False,
        "sweep_used": False,
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "target_contract_id": contract.TARGET_CONTRACT_ID,
        "detector_or_fsm_modified": False,
        "q2_paths_opened": [],
        "q3_paths_opened": [],
    }
    gate = contract.fit_gate(summary)
    assert gate["passed"] is True
    for name in (
        "global_metrics",
        "p2_subset_metrics",
        "nominal_r4_pass_metrics",
        "nominal_r4_student_exposed_metrics",
        "per_case_metrics",
        "critical_window_non_regression",
    ):
        assert gate["checks"][name] is True
    drifted = copy.deepcopy(summary)
    drifted["critical_window_metrics"]["rmse"] = 0.006
    assert contract.fit_gate(drifted)["passed"] is False


def test_candidate_and_aggregate_gates_fail_closed() -> None:
    tree_hash = "a" * 64
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.CANDIDATE_FREEZE_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_module": {"tree_sha256": tree_hash},
        "candidate_id": contract.candidate_id(tree_hash),
        "fit_passed": True,
        "candidate_frozen": True,
        "source_h0_byte_exact": True,
        "logstd_byte_exact": True,
        "critic_present": False,
        "save_reload_exact": True,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "runtime_promoted": False,
        "q2_paths_opened": [],
        "q3_paths_opened": [],
    }
    assert contract.candidate_freeze_gate(summary)["passed"] is True
    assert contract.fit_gate({})["passed"] is False
    assert contract.aggregate_development_gate({})["passed"] is False


def test_pure_policy_trace_recomputes_every_row_and_rejects_counter_cancellation() -> (
    None
):
    case_id = contract.DEVELOPMENT_CASE_IDS[0]
    trace = _pure_trace(case_id)
    audit = contract.pure_policy_trace_audit(trace, case_id=case_id)
    assert audit["passed"] is True
    assert audit["row_count"] == 500
    tampered = copy.deepcopy(trace)
    tampered[10]["teacher_query_count"] = 1
    tampered[11]["teacher_query_count"] = -1
    cancelled = contract.pure_policy_trace_audit(tampered, case_id=case_id)
    assert cancelled["counters"]["teacher_query_count"] == 0
    assert cancelled["per_row_zero_counters"] is False
    assert cancelled["passed"] is False
