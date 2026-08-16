from __future__ import annotations

import copy

import h0_v12r4_p3_coverage_contract as contract


def test_contract_self_check_and_authority_are_exact() -> None:
    result = contract.contract_self_check()
    assert result["passed"] is True
    assert contract.REVISION == "2026-08-09"
    assert contract.AUTHORITY_TEXT == "esegui i punti 1-6"
    assert contract.AUTHORITY["qualification_execution_authorized"] is False
    assert contract.AUTHORITY["retry_authorized"] is False
    assert contract.AUTHORITY["runtime_promotion_authorized"] is False


def test_stage_order_is_one_fit_one_freeze_nominal_first() -> None:
    assert len(contract.STAGE_IDS) == 15
    assert contract.STAGE_IDS[1:5] == tuple(
        f"collect_cov__{case_id}" for case_id in contract.COLLECTION_CASE_IDS
    )
    assert contract.STAGE_IDS[5:8] == (
        "assemble_corpus_p3",
        "fit_p3",
        "freeze_candidate_p3",
    )
    assert contract.STAGE_IDS[8] == "development__deterministic_offset_nominal"
    assert contract.STAGE_IDS[-1] == "finalize_development"


def test_corpus_fit_and_candidate_selection_are_frozen() -> None:
    assert contract.expected_corpus_counts()["sample_count"] == 10_732
    assert contract.expected_corpus_counts()["episode_count"] == 22
    assert contract.expected_corpus_counts()["normalized_total_sample_mass"] == 11_000.0
    assert contract.FIT["initial_checkpoint_id"] == contract.SOURCE_H0_ID
    assert contract.FIT["continued_from_p2"] is False
    assert contract.FIT["adamw"]["epochs"] == 3000
    assert contract.FIT["lbfgs"]["max_iter"] == 600
    assert contract.FIT["lbfgs"]["max_eval"] == 1200
    assert contract.CANDIDATE_SELECTION_RULE == "SOLE_FIT_P3_OUTPUT_FROM_LOCKED_R4_RUN"


def test_q2_paths_are_separate_and_design_is_not_an_unopened_output() -> None:
    prefix = "Trajectory Generator/baseline_MLP/validation/v12r4q2/"
    assert str(contract.Q2_DESIGN_FREEZE_PATH).startswith(prefix)
    assert contract.Q2_DESIGN_FREEZE_PATH not in contract.Q2_UNOPENED_PATHS.values()
    assert all(
        str(path).startswith(prefix) for path in contract.Q2_UNOPENED_PATHS.values()
    )
    assert str(contract.Q2_UNOPENED_PATHS["run_root"]).endswith(
        "h0_v12r4_q2_run_20260809"
    )


def test_five_q2_prerequisite_interfaces_are_exact() -> None:
    assert contract.Q2_PREREQUISITES == {
        "protocol_freeze": {
            "path": contract.PROTOCOL_FREEZE_PATH.as_posix(),
            "status": "PASS_H0_V12R4_P3_COVERAGE_PROTOCOL_FREEZE",
        },
        "execution_lock": {
            "path": contract.EXECUTION_LOCK_PATH.as_posix(),
            "status": "PASS_H0_V12R4_P3_COVERAGE_EXECUTION_LOCK",
        },
        "candidate_freeze": {
            "path": contract.CANDIDATE_FREEZE_PATH.as_posix(),
            "status": "PASS_H0_V12R4_P3_CANDIDATE_FREEZE",
        },
        "final_development_receipt": {
            "path": contract.FINAL_DEVELOPMENT_RECEIPT_PATH.as_posix(),
            "status": "PASS_H0_V12R4_P3_DEVELOPMENT",
        },
        "terminal_ledger": {
            "path": contract.PIPELINE_LEDGER_PATH.as_posix(),
            "status": "PASS_H0_V12R4_P3_COVERAGE_PIPELINE_TERMINAL",
        },
    }


def test_candidate_freeze_gate_binds_id_and_tree() -> None:
    tree_hash = "a" * 64
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.CANDIDATE_FREEZE_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
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
    }
    assert contract.candidate_freeze_gate(summary)["passed"] is True
    drifted = copy.deepcopy(summary)
    drifted["candidate_id"] = contract.candidate_id("b" * 64)
    assert contract.candidate_freeze_gate(drifted)["passed"] is False


def test_fit_gate_fails_closed_as_a_total_pure_function() -> None:
    gate = contract.fit_gate({})
    assert gate["passed"] is False
    assert gate["status"] == contract.TERMINAL_FAIL_STATUS
