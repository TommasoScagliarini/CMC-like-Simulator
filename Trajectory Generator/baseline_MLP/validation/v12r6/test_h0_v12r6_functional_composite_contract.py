"""Source-only tests for the V12R6 functional-composite contract."""

from __future__ import annotations

import copy
import os
import sys
from pathlib import Path
from typing import Any, Callable

import pytest


V12R6_ROOT = Path(__file__).resolve().parent
if os.fspath(V12R6_ROOT) not in sys.path:
    sys.path.insert(0, os.fspath(V12R6_ROOT))

import h0_v12r6_functional_composite_contract as contract  # noqa: E402


TREE_SHA256 = "a" * 64
CANDIDATE_MODULE = {
    "path": contract.CANDIDATE_MODULE_PATH.as_posix(),
    "tree_sha256": TREE_SHA256,
    "file_count": 5,
    "files": [],
}


def _valid_offline_evaluation() -> dict[str, Any]:
    metric_names = {
        "global",
        "p2_subset",
        "nominal_r4_pass",
        "nominal_r4_student_exposed",
        *(f"case::{case_id}" for case_id in contract.CASE_IDS),
    }
    return {
        "passed": True,
        "failed_checks": [],
        "metrics": {
            name: {
                "rmse": 0.001,
                "max_abs_error": 0.01,
                "reset_max_abs_error": 0.001,
            }
            for name in metric_names
        },
        "critical_window_metrics": {"rmse": 0.001, "max_abs_error": 0.01},
        "p2_critical_window_metrics": {"rmse": 0.002, "max_abs_error": 0.02},
    }


def _valid_synthesis_summary() -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.SYNTHESIS_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "synthesis": copy.deepcopy(contract.SYNTHESIS),
        "source_records": copy.deepcopy(contract.SOURCE_RECORDS),
        "candidate_module": copy.deepcopy(CANDIDATE_MODULE),
        "candidate_id": contract.candidate_id(TREE_SHA256),
        "functional_verification": {
            "passed": True,
            "max_abs_error_before_save": 1.2e-6,
            "max_abs_error_after_reload": 1.2e-6,
            "logstd_exact": True,
            "disabled_clock_columns_bit_zero": True,
            "save_reload_exact": True,
            "actor_feature_manifest_valid": True,
            "warm_start_target_512_compatible": True,
        },
        "offline_evaluation": _valid_offline_evaluation(),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "adamw_epochs_completed": 0,
        "lbfgs_closure_calls": 0,
        "actor_synthesis_stage_calls_attempted": 1,
        "actor_synthesis_executions_confirmed": 1,
        "new_collection_count": 0,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "target_contract_id": contract.TARGET_CONTRACT_ID,
        "detector_or_fsm_modified": False,
        "q2_paths_opened": [],
        "q3_paths_opened": [],
    }


def _valid_candidate_freeze_summary() -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.CANDIDATE_FREEZE_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "synthesis_passed": True,
        "candidate_module": copy.deepcopy(CANDIDATE_MODULE),
        "candidate_id": contract.candidate_id(TREE_SHA256),
        "candidate_frozen": True,
        "functional_equivalence_passed": True,
        "logstd_exact": True,
        "save_reload_exact": True,
        "warm_start_target_512_compatible": True,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "runtime_promoted": False,
        "q2_paths_opened": [],
        "q3_paths_opened": [],
    }


def _valid_aggregate_summary() -> dict[str, Any]:
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "protocol_id": contract.PROTOCOL_ID,
        "case_gates": [
            {"case_id": case_id, "passed": True}
            for case_id in contract.DEVELOPMENT_CASE_IDS
        ],
        "candidate_tree_unique_count": 1,
        "new_collection_count": 0,
        "development_count": 6,
        "environment_reset_calls": 6,
        "environment_step_calls": 3000,
        "raw_sensor_sample_count": 30_000,
        "teacher_query_count": 0,
        "pure_policy_trace_row_count": 3000,
        "actor_synthesis_stage_calls_attempted": 1,
        "actor_synthesis_executions_confirmed": 1,
        "actor_fit_stage_calls_attempted": 0,
        "actor_fit_executions_confirmed": 0,
        "actor_updates": 0,
        "adamw_epochs_completed": 0,
        "lbfgs_closure_calls": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "q2_paths_opened": [],
        "q3_paths_opened": [],
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
    }
    summary.update({name: 0 for name in contract.PURE_POLICY_COUNTER_FIELDS})
    return summary


def test_contract_self_check_architecture_authority_and_new_namespace() -> None:
    gate = contract.contract_self_check()

    assert gate["passed"] is True
    assert all(gate["checks"].values())
    assert contract.SYNTHESIS["alpha_p2"] == 0.70
    assert contract.SYNTHESIS["alpha_r5"] == 0.30
    assert contract.SYNTHESIS["parameter_fit"] is False
    assert contract.COMPOSITE_ARCHITECTURE["hidden_dims"] == [512, 512]
    assert contract.COMPOSITE_ARCHITECTURE["branch_order"] == ["p2", "r5"]
    assert contract.AUTHORITY["composite_synthesis_authorized"] is True
    assert contract.AUTHORITY["qualification_execution_authorized"] is False
    assert contract.VALIDATION_ROOT != contract.v12r5.VALIDATION_ROOT
    assert contract.RUN_ROOT != contract.v12r5.RUN_ROOT


def test_case_and_stage_order_is_critical_first_and_copy_safe() -> None:
    expected_cases = (
        "deterministic_offset_plus_0p20",
        "deterministic_offset_nominal",
        "deterministic_offset_minus_0p20",
        "stochastic_nominal_seed_126",
        "stochastic_nominal_seed_127",
        "stochastic_nominal_seed_128",
    )

    assert contract.DEVELOPMENT_CASE_IDS == expected_cases
    assert contract.STAGE_IDS[:3] == (
        "attest_terminal_inputs",
        "synthesize_functional_composite_candidate",
        "freeze_functional_composite_candidate",
    )
    assert contract.STAGE_IDS[3:-1] == tuple(
        f"development__{case_id}" for case_id in expected_cases
    )
    assert contract.STAGE_IDS[-1] == "finalize_development"
    assert contract.stage_descriptor(contract.STAGE_IDS[0]) == {
        "stage_id": "attest_terminal_inputs",
        "kind": "attestation",
    }
    critical = contract.canonical_development_case(expected_cases[0])
    assert (
        critical["destination"]
        == contract.DEVELOPMENT_PATHS[expected_cases[0]].as_posix()
    )
    assert critical["behavior"] == "R6_COMPOSITE_PURE_UNBLENDED_NO_TEACHER_NO_LATCH"
    critical["case_id"] = "tampered"
    assert (
        contract.canonical_development_case(expected_cases[0])["case_id"]
        == (expected_cases[0])
    )
    with pytest.raises(ValueError, match="unknown V12R6 development case"):
        contract.canonical_development_case("unknown")
    with pytest.raises(ValueError, match="unknown V12R6 stage"):
        contract.stage_descriptor("unknown")


@pytest.mark.parametrize("value", ("", "0" * 63, "g" * 64))
def test_candidate_id_rejects_invalid_tree_digests(value: str) -> None:
    with pytest.raises(ValueError):
        contract.candidate_id(value)


def test_valid_synthesis_gate_passes_every_registered_check() -> None:
    summary = _valid_synthesis_summary()

    gate = contract.synthesis_gate(summary)

    assert gate["passed"] is True
    assert gate["status"] == contract.SYNTHESIS_PASS_STATUS
    assert all(gate["checks"].values())
    assert gate["candidate_id"] == contract.candidate_id(TREE_SHA256)
    assert gate["next_stage"] == "FREEZE_FUNCTIONAL_COMPOSITE_CANDIDATE"
    assert summary == _valid_synthesis_summary()


SynthesisMutation = Callable[[dict[str, Any]], None]


@pytest.mark.parametrize(
    ("mutation", "failed_check"),
    (
        (lambda value: value.__setitem__("schema_version", False), "schema"),
        (
            lambda value: value["source_records"].pop("p2_module"),
            "sources",
        ),
        (lambda value: value.__setitem__("candidate_id", "wrong"), "candidate"),
        (
            lambda value: value["functional_verification"].__setitem__(
                "max_abs_error_after_reload", 2.1e-6
            ),
            "functional_equivalence",
        ),
        (
            lambda value: value["functional_verification"].__setitem__(
                "max_abs_error_before_save", float("nan")
            ),
            "functional_equivalence",
        ),
        (
            lambda value: value["functional_verification"].__setitem__(
                "logstd_exact", False
            ),
            "preservation",
        ),
        (
            lambda value: value["offline_evaluation"]["metrics"]["global"].__setitem__(
                "rmse", False
            ),
            "offline",
        ),
        (lambda value: value.__setitem__("actor_updates", False), "zero_fit"),
        (
            lambda value: value.__setitem__(
                "actor_synthesis_stage_calls_attempted", True
            ),
            "one_synthesis",
        ),
        (
            lambda value: value.__setitem__("environment_reset_calls", False),
            "no_environment",
        ),
        (
            lambda value: value.__setitem__("event_contract_id", "drifted"),
            "v26_unchanged",
        ),
        (
            lambda value: value["q3_paths_opened"].append("opened"),
            "qualification_closed",
        ),
    ),
)
def test_synthesis_gate_mutations_fail_closed(
    mutation: SynthesisMutation, failed_check: str
) -> None:
    summary = _valid_synthesis_summary()
    mutation(summary)

    gate = contract.synthesis_gate(summary)

    assert gate["passed"] is False
    assert gate["status"] == contract.TERMINAL_FAIL_STATUS
    assert gate["checks"][failed_check] is False
    assert gate["next_stage"] == "STOP_TERMINAL"


def test_candidate_freeze_gate_passes_and_rejects_bool_counter() -> None:
    valid = _valid_candidate_freeze_summary()

    gate = contract.candidate_freeze_gate(valid)

    assert gate["passed"] is True
    assert all(gate["checks"].values())
    assert gate["status"] == contract.CANDIDATE_FREEZE_PASS_STATUS
    invalid = copy.deepcopy(valid)
    invalid["critic_updates"] = False
    rejected = contract.candidate_freeze_gate(invalid)
    assert rejected["passed"] is False
    assert rejected["checks"]["zero_updates"] is False
    assert rejected["status"] == contract.TERMINAL_FAIL_STATUS


def test_valid_aggregate_gate_passes_in_exact_case_order() -> None:
    gate = contract.aggregate_development_gate(_valid_aggregate_summary())

    assert gate["passed"] is True
    assert gate["status"] == contract.FINAL_DEVELOPMENT_PASS_STATUS
    assert all(gate["checks"].values())
    assert gate["next_stage"] == "WAIT_SEPARATE_V12R6Q3_PROTOCOL"


AggregateMutation = Callable[[dict[str, Any]], None]


@pytest.mark.parametrize(
    ("mutation", "failed_check"),
    (
        (lambda value: value["case_gates"].reverse(), "six_of_six"),
        (
            lambda value: value["case_gates"][0].__setitem__("passed", 1),
            "six_of_six",
        ),
        (
            lambda value: value.__setitem__("candidate_tree_unique_count", True),
            "fixed_candidate",
        ),
        (
            lambda value: value.__setitem__("teacher_query_count", False),
            "activity",
        ),
        (
            lambda value: value.__setitem__(
                "actor_synthesis_executions_confirmed", True
            ),
            "activity",
        ),
        (lambda value: value.__setitem__("environment_step_calls", 2999), "activity"),
        (lambda value: value.__setitem__("retry_authorized", True), "no_retry"),
        (
            lambda value: value["q2_paths_opened"].append("opened"),
            "qualification_closed",
        ),
        (
            lambda value: value.__setitem__("checkpoint_zero_created", True),
            "no_promotion",
        ),
    ),
)
def test_aggregate_gate_mutations_fail_closed(
    mutation: AggregateMutation, failed_check: str
) -> None:
    summary = _valid_aggregate_summary()
    mutation(summary)

    gate = contract.aggregate_development_gate(summary)

    assert gate["passed"] is False
    assert gate["status"] == contract.TERMINAL_FAIL_STATUS
    assert gate["checks"][failed_check] is False
    assert gate["next_stage"] == "STOP_TERMINAL"
