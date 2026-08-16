"""Tests for the source-only, deferred-candidate V12R5-Q3 design."""

from __future__ import annotations

import ast
import copy
import os
import sys
from pathlib import Path

import pytest

Q3_ROOT = Path(__file__).resolve().parent
if str(Q3_ROOT) not in sys.path:
    sys.path.insert(0, str(Q3_ROOT))

import freeze_h0_v12r5_q3_qualification_design as freezer  # noqa: E402
import h0_v12r5_q3_qualification_contract as contract  # noqa: E402


def test_contract_self_check_is_pure_and_complete() -> None:
    gate = contract.contract_self_check()

    assert gate["passed"] is True
    assert len(gate["checks"]) == 16
    assert all(gate["checks"].values())


def test_authority_and_revision_are_exact() -> None:
    assert contract.SCHEMA_VERSION == 140
    assert contract.REVISION == "2026-08-09"
    assert contract.AUTHORITY_TEXT == "esegui i punti 1-6"
    assert contract.AUTHORITY["authority_date"] == "2026-08-09"
    assert contract.AUTHORITY["design_freeze_publication_authorized_now"] is True
    for name in (
        "noise_materialization_authorized",
        "qualification_execution_authorized",
        "actor_fit_authorized",
        "actor_updates_authorized",
        "critic_updates_authorized",
        "ppo_updates_authorized",
        "runtime_promotion_authorized",
        "checkpoint_zero_authorized",
        "positive_morphology_authorized",
    ):
        assert contract.AUTHORITY[name] is False


def test_contract_import_is_io_free_by_construction() -> None:
    source = Path(contract.__file__).read_text(encoding="utf-8")
    tree = ast.parse(source)
    imported_roots = {
        node.names[0].name.split(".")[0]
        for node in ast.walk(tree)
        if isinstance(node, ast.Import) and node.names
    }
    imported_roots.update(
        node.module.split(".")[0]
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom) and node.module
    )
    called_names = {
        node.func.id
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
    }
    called_attributes = {
        node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }

    assert imported_roots == {"__future__", "copy", "pathlib", "typing"}
    assert {"open", "exec", "eval", "compile"}.isdisjoint(called_names)
    assert {
        "read_text",
        "read_bytes",
        "write_text",
        "write_bytes",
        "mkdir",
        "touch",
        "unlink",
        "rename",
        "replace",
    }.isdisjoint(called_attributes)


def test_candidate_identity_is_really_deferred() -> None:
    assert contract.CANDIDATE_BINDING_STATE == "DEFERRED"
    assert contract.CANDIDATE_ID is None
    assert contract.CANDIDATE_MODULE is None

    role = contract.role_contract(contract.CANDIDATE_ROLE)
    assert role["actor_id"] is None
    assert role["candidate_id"] is None
    assert role["actor_module"] is None
    assert role["binding_policy"]["mutable_candidate_identifier_allowed"] is False
    assert role["binding_policy"]["candidate_substitution_allowed"] is False


def test_future_r5_interface_and_binding_split_are_exact() -> None:
    requirements = contract.prerequisite_requirements()

    assert contract.R5_RUN_ROOT.as_posix().endswith("h0_v12r5_run_20260809")
    assert contract.R5_CANDIDATE_MODULE_PATH.as_posix().endswith(
        "h0_v12r5_run_20260809/fit/rl_module_target_adapted"
    )

    assert [row["name"] for row in requirements] == [
        "r5_protocol_freeze",
        "r5_execution_lock",
        "r5_candidate_freeze_receipt",
        "r5_final_development_receipt",
        "r5_terminal_pass_ledger",
    ]
    assert [row["path"] for row in requirements] == [
        (
            "Trajectory Generator/baseline_MLP/validation/v12r5/"
            "h0_v12r5_case_balanced_protocol_freeze.json"
        ),
        (
            "Trajectory Generator/baseline_MLP/validation/v12r5/"
            "h0_v12r5_case_balanced_execution_lock.json"
        ),
        (
            "Trajectory Generator/baseline_MLP/validation/v12r5/"
            "h0_v12r5_run_20260809/candidate_freeze_receipt.json"
        ),
        (
            "Trajectory Generator/baseline_MLP/validation/v12r5/"
            "h0_v12r5_run_20260809/final_development_receipt.json"
        ),
        (
            "Trajectory Generator/baseline_MLP/validation/v12r5/"
            "h0_v12r5_run_20260809/pipeline_ledger.json"
        ),
    ]
    assert [row["required_status"] for row in requirements] == [
        "PASS_H0_V12R5_CASE_BALANCED_PROTOCOL_FREEZE",
        "PASS_H0_V12R5_CASE_BALANCED_EXECUTION_LOCK",
        "PASS_H0_V12R5_CASE_BALANCED_CANDIDATE_FREEZE",
        "PASS_H0_V12R5_CASE_BALANCED_DEVELOPMENT",
        "PASS_H0_V12R5_CASE_BALANCED_PIPELINE_TERMINAL",
    ]
    assert all(row["candidate_identity_required"] is False for row in requirements[:2])
    assert all(row["candidate_tree_required"] is False for row in requirements[:2])
    assert all(
        row["required_selection_rule"]
        == "SOLE_CASE_BALANCED_FIT_OUTPUT_FROM_LOCKED_R5_RUN"
        for row in requirements[:2]
    )
    assert all(row["candidate_identity_required"] is True for row in requirements[2:])
    assert all(row["candidate_tree_required"] is True for row in requirements[2:])
    assert requirements[-1]["terminal_required"] is True
    assert requirements[-1]["passed_required"] is True


def test_future_requirement_and_binding_helpers_return_deep_copies() -> None:
    requirements = contract.prerequisite_requirements()
    policy = contract.candidate_binding_policy()
    requirements[0]["path"] = "mutated"
    policy["selection_rule"] = "mutated"

    assert contract.prerequisite_requirements()[0]["path"] != "mutated"
    assert (
        contract.candidate_binding_policy()["selection_rule"]
        == "SOLE_CASE_BALANCED_FIT_OUTPUT_FROM_LOCKED_R5_RUN"
    )


def test_six_holdouts_are_exact_and_live_only_in_q3_namespace() -> None:
    cases = contract.canonical_cases()

    assert tuple(case["case_id"] for case in cases) == contract.CASE_IDS
    assert cases[0] == {
        "case_id": "deterministic_offset_minus_0p30",
        "action_selection": "deterministic",
        "episode_start_offset_s": 1.656870983805102,
        "action_seed": None,
        "runtime_seed": 129,
        "sigma": 0.0,
        "noise_tape": (
            "Trajectory Generator/baseline_MLP/validation/v12r5q3/"
            "h0_v12r5_q3_qualification_noise_tapes/deterministic_all_zero.npz"
        ),
    }
    assert cases[1]["episode_start_offset_s"] == 2.256870983805102
    assert tuple(case["action_seed"] for case in cases[2:]) == (130, 131, 132, 133)
    assert tuple(case["runtime_seed"] for case in cases[2:]) == (130, 131, 132, 133)
    assert all(case["sigma"] == 0.005 for case in cases[2:])
    assert all(
        case["noise_tape"].startswith(f"{contract.NOISE_ROOT.as_posix()}/")
        for case in cases
    )


def test_case_and_role_helpers_are_isolated_and_fail_closed() -> None:
    cases = contract.canonical_cases()
    cases[0]["runtime_seed"] = -1
    baseline = contract.role_contract(contract.BASELINE_ROLE)
    baseline["actor_module"]["tree_sha256"] = "mutated"

    assert contract.canonical_case(contract.CASE_IDS[0])["runtime_seed"] == 129
    assert contract.role_contract(contract.BASELINE_ROLE)["actor_module"] == (
        contract.SOURCE_H0_MODULE
    )
    with pytest.raises(ValueError, match="unknown Q3 qualification case"):
        contract.canonical_case("missing")
    with pytest.raises(ValueError, match="unknown Q3 qualification role"):
        contract.role_contract("missing")
    with pytest.raises(ValueError, match="unknown Q3 qualification role"):
        contract.canonical_rollout("missing", contract.CASE_IDS[0])


def test_rollout_matrix_is_baseline_six_then_candidate_six() -> None:
    assert len(contract.ROLLOUT_MATRIX) == 12
    assert tuple(row["role"] for row in contract.ROLLOUT_MATRIX) == (
        *(contract.BASELINE_ROLE for _ in range(6)),
        *(contract.CANDIDATE_ROLE for _ in range(6)),
    )
    assert tuple(row["case_id"] for row in contract.ROLLOUT_MATRIX[:6]) == (
        contract.CASE_IDS
    )
    assert tuple(row["case_id"] for row in contract.ROLLOUT_MATRIX[6:]) == (
        contract.CASE_IDS
    )
    assert all(
        row["destination"].startswith(f"{contract.RUN_ROOT.as_posix()}/")
        for row in contract.ROLLOUT_MATRIX
    )


def test_role_semantics_preserve_h0_and_fix_v26_candidate_contract() -> None:
    baseline = contract.role_contract(contract.BASELINE_ROLE)
    candidate = contract.role_contract(contract.CANDIDATE_ROLE)

    assert baseline["actor_id"] == "original_h0"
    assert baseline["actor_module"] == contract.SOURCE_H0_MODULE
    assert baseline["observation_semantics"] == "counterfactual_analog"
    assert baseline["event_contract_id"] == "legacy_events"
    assert baseline["binary_phase_fsm_mode"] == "disabled"
    assert candidate["actor_input_view"] == "primary_split"
    assert candidate["event_contract_id"] == "binary_point_v25+heel_qualified_fsm_v2"
    assert candidate["target_contract_id"] == (
        "primary_grf_split_v1+binary_point_v25+heel_qualified_fsm_v2"
    )
    assert candidate["binary_phase_fsm_mode"] == "binary_active"
    assert baseline["morphology_weight"] == candidate["morphology_weight"] == 0.0


def test_gate_constants_and_pairwise_tolerances_are_exact() -> None:
    assert contract.EXPECTED_STEPS == 500
    assert contract.EXPECTED_CONTROL_WINDOWS == 5_000
    assert contract.EXPECTED_RAW_SENSOR_SAMPLES == 5_000
    assert contract.MINIMUM_VALID_CYCLES == 2
    assert contract.PENETRATION_LIMIT_M == 0.025
    assert contract.RESERVE_TOLERANCES == (
        ("reserve_norm_nm.rms", 5.0, 0.05),
        ("reserve_norm_nm.abs_max", 5.0, 0.05),
        ("residual_norm_nm.rms", 1.0e-6, 0.05),
        ("residual_norm_nm.abs_max", 1.0e-6, 0.05),
    )
    assert len(contract.SEA_TOLERANCES) == 24
    assert all(row[1:] == (1.0e-6, 0.05) for row in contract.SEA_TOLERANCES)
    assert {"actor_updates", "critic_updates", "ppo_updates"} <= set(
        contract.ZERO_REQUIRED_COUNTS
    )


def test_tape_abi_and_expected_array_hashes_are_exact() -> None:
    assert contract.TAPE_ABI == {
        "array_key": "standard_normal",
        "shape": [500, 2],
        "dtype": "float32",
        "c_contiguous": True,
        "generator": "numpy.random.default_rng(seed).standard_normal",
        "cast": "astype(numpy.float32)",
        "standard_normal_pre_scaling": True,
        "hash_scheme": "sha256(dtype_ascii+compact_json_shape+contiguous_c_bytes)",
    }
    assert contract.EXPECTED_TAPE_ARRAY_SHA256 == {
        "deterministic_all_zero.npz": (
            "ab89c5ecd7d818ab19f726cffc9ce431f5889448c7a79f84927f7153e546782c"
        ),
        "stochastic_seed_130_standard_normal.npz": (
            "067a5fb858ba1a5365856367eb1de793d954c9cbc07fa1aaaed34520a08657fa"
        ),
        "stochastic_seed_131_standard_normal.npz": (
            "00ea968882002a0881451cc2d80a365b2f6a3ccbfd698fcbe3768ca572abae67"
        ),
        "stochastic_seed_132_standard_normal.npz": (
            "9905eddb8074676cfe1ac2feeee152d114e9532b8cee9844daad016fa4930b61"
        ),
        "stochastic_seed_133_standard_normal.npz": (
            "d25e7515c993742e24da93f01313348991ea35871f98dbf07cd767367ec44438"
        ),
    }


def test_holdout_provenance_states_the_independence_limit() -> None:
    provenance = contract.HOLDOUT_PROVENANCE

    assert provenance["status"] == "REUSED_UNOPENED_V6_HOLDOUTS_FROZEN_BEFORE_R4"
    assert provenance["q3_noise_tapes_materialized"] is False
    assert provenance["q3_qualification_rollouts_opened"] is False
    assert provenance["statistical_blindness_claimed"] is False
    assert provenance["r5_may_read_q3_noise"] is False
    assert provenance["r5_may_materialize_q3_noise"] is False
    assert provenance["r5_may_execute_q3_rollouts"] is False
    assert provenance["r5_may_consume_q3_outcomes_for_training_or_selection"] is False
    assert provenance["condition_values_are_exact_q2_reuse"] is True
    assert provenance["q2_design_freeze_opened_as_bound_history"] is True
    assert provenance["q2_runtime_outputs_opened"] == []
    assert provenance["q2_runtime_output_access_authorized"] is False


def test_historical_q1_p1s_q2_and_r4_are_byte_bound_but_excluded() -> None:
    assert contract.Q1_DESIGN_FREEZE_ARTIFACT == {
        "path": (
            "Trajectory Generator/baseline_MLP/validation/v12p1q/"
            "h0_v12r3_p1_qualification_design_freeze.json"
        ),
        "sha256": "c64928cb85df0e1f5d53c5f2e6eba52172b7c831ccf712a0113522bcff4ef686",
        "size_bytes": 21_303,
    }
    assert contract.P1S_TERMINAL_LEDGER_ARTIFACT["sha256"] == (
        "e0969d83ed36b4b7b10f50abde0a8f9d85c7d0aeb97a3318a42ded3e0aaa4b92"
    )
    assert contract.Q2_DESIGN_FREEZE_ARTIFACT == {
        "path": (
            "Trajectory Generator/baseline_MLP/validation/v12r4q2/"
            "h0_v12r4_q2_qualification_design_freeze.json"
        ),
        "sha256": "d92fac765bd192f43ef2e0420a8529e8bb21860a3f47420ab5948863fb53eaf5",
        "size_bytes": 32_166,
    }
    assert contract.R4_TERMINAL_LEDGER_ARTIFACT == {
        "path": (
            "Trajectory Generator/baseline_MLP/validation/v12r4/"
            "h0_v12r4_run_20260809/pipeline_ledger.json"
        ),
        "sha256": "4351fd5be9f35f0ab5f5166329344af7e6a8cbf9f0b87691e254d5b7704a805f",
        "size_bytes": 5_898,
    }
    assert contract.HISTORICAL_EXCLUSIONS["q1_candidate_is_not_q3_candidate"] is True
    assert contract.HISTORICAL_EXCLUSIONS["p1s_candidate_is_not_q3_candidate"] is True
    assert (
        contract.HISTORICAL_EXCLUSIONS["q2_deferred_candidate_is_not_q3_candidate"]
        is True
    )
    assert (
        contract.HISTORICAL_EXCLUSIONS[
            "r4_failed_lineage_candidate_is_not_q3_candidate"
        ]
        is True
    )
    assert (
        contract.HISTORICAL_EXCLUSIONS["historical_candidate_fallback_allowed"] is False
    )


def test_all_declared_mutations_stay_in_dated_q3_namespace() -> None:
    paths = contract.declared_mutation_paths()

    assert contract.RUN_ROOT.name == "h0_v12r5_q3_run_20260809"
    assert len(paths) == len(set(paths.values()))
    assert all(
        path.as_posix().startswith(f"{contract.VALIDATION_ROOT.as_posix()}/")
        for path in paths.values()
    )
    assert contract.QUALIFICATION_DESIGN_FREEZE_PATH != contract.PROTOCOL_FREEZE_PATH
    assert contract.PROTOCOL_FREEZE_PATH != contract.EXECUTION_LOCK_PATH
    assert contract.NEXT_STAGE_AFTER_Q3_PASS == "WAIT_SEPARATE_ZERO_UPDATE_PROTOCOL"


def test_source_closure_includes_cross_platform_attributes_and_safe_reuse_only() -> (
    None
):
    assert contract.DESIGN_SOURCE_RELATIVE_PATHS["q3_line_endings_policy"].endswith(
        "v12r5q3/.gitattributes"
    )
    attributes = freezer.resolve_relative(
        contract.DESIGN_SOURCE_RELATIVE_PATHS["q3_line_endings_policy"]
    ).read_text(encoding="utf-8")
    assert (
        attributes
        == "*.py text eol=lf\n*.json text eol=lf\n*.npz binary\n*.pkl binary\n"
    )
    assert contract.SAFE_SOURCE_REUSE["q1_gates"] == ("artifact_record_matches",)
    assert (
        contract.SAFE_SOURCE_REUSE["candidate_specific_q1_rollout_gates_reused"]
        is False
    )


def test_build_is_read_only_and_passes_without_publishing() -> None:
    canonical = freezer.resolve_relative(contract.QUALIFICATION_DESIGN_FREEZE_PATH)
    assert not os.path.lexists(canonical)

    payload = freezer.build_design_freeze()

    assert payload["passed"] is True
    assert payload["status"] == "PASS_H0_V12R5_Q3_QUALIFICATION_DESIGN_FREEZE"
    assert payload["freeze_kind"] == ("PRE_R5_DEFERRED_CANDIDATE_INDEPENDENT_Q3_DESIGN")
    assert not os.path.lexists(canonical)
    assert not os.path.lexists(freezer.resolve_relative(contract.NOISE_ROOT))
    assert not os.path.lexists(freezer.resolve_relative(contract.RUN_ROOT))


def test_build_binds_closed_sources_and_historical_exclusion() -> None:
    payload = freezer.build_design_freeze()

    assert payload["source_gate"]["passed"] is True
    assert tuple(payload["source_hashes"]) == tuple(
        contract.DESIGN_SOURCE_RELATIVE_PATHS
    )
    assert payload["historical_exclusion_gate"]["passed"] is True
    assert (
        payload["historical_exclusion_gate"]["checks"]["q1_design_byte_exact"] is True
    )
    assert (
        payload["historical_exclusion_gate"]["checks"]["p1s_terminal_fail_exact"]
        is True
    )
    assert payload["historical_exclusion_gate"]["checks"]["q2_design_byte_exact"]
    assert payload["historical_exclusion_gate"]["checks"]["r4_ledger_byte_exact"]
    assert payload["historical_exclusion_gate"]["checks"][
        "q2_passed_deferred_design_exact"
    ]
    assert payload["historical_exclusion_gate"]["checks"][
        "q2_contract_source_still_matches_immutable_design"
    ]
    assert payload["historical_exclusion_gate"]["checks"]["r4_terminal_fail_exact"]
    assert payload["design_gate"]["checks"]["q2_design_semantics_reused_exact"]


def test_q2_unopened_runtime_closure_uses_every_exact_frozen_source_path() -> None:
    paths = freezer._q2_unopened_runtime_paths()
    declared = dict(freezer.q2_contract.declared_mutation_paths())

    assert declared.pop("qualification_design_freeze") == (
        freezer.q2_contract.QUALIFICATION_DESIGN_FREEZE_PATH
    )
    assert len(paths) == len(set(paths.values())) == 100
    assert all(paths[name] == path for name, path in declared.items())
    assert paths["protocol_freeze"] == freezer.q2_contract.PROTOCOL_FREEZE_PATH
    assert paths["execution_lock"] == freezer.q2_contract.EXECUTION_LOCK_PATH
    assert paths["noise_root"] == freezer.q2_contract.NOISE_ROOT
    assert paths["noise_manifest"] == freezer.q2_contract.NOISE_MANIFEST_PATH
    assert paths["run_root"] == freezer.q2_contract.RUN_ROOT
    assert paths["pipeline_ledger"] == freezer.q2_contract.PIPELINE_LEDGER_PATH
    assert paths["baseline_outcomes_root"] == freezer.q2_contract.BASELINE_ROOT
    assert paths["candidate_outcomes_root"] == freezer.q2_contract.CANDIDATE_ROOT
    assert paths["pair_outcomes_root"] == freezer.q2_contract.PAIR_ROOT
    assert paths["final_outcomes_root"] == freezer.q2_contract.FINAL_ROOT
    assert paths["worker_claims_root"] == freezer.q2_contract.WORKER_CLAIMS_ROOT
    assert all(
        paths[f"worker_claim_{stage_id}"]
        == freezer.q2_contract.worker_claim_path(stage_id)
        for stage_id in freezer.q2_contract.STAGE_IDS
    )
    assert all(
        not ({"*", "?", "[", "]"} & set(path.as_posix())) for path in paths.values()
    )


def test_all_exact_q2_runtime_outputs_are_absent_and_audited() -> None:
    occupancy = freezer._occupancy_snapshot()
    q2_keys = set(freezer._q2_runtime_occupancy_paths())
    payload = freezer.build_design_freeze()
    evidence = payload["q2_unopened_runtime"]

    assert len(q2_keys) == 100
    assert all(occupancy[name] is True for name in q2_keys)
    assert all(payload["checks"][name] is True for name in q2_keys)
    assert payload["checks"]["all_exact_q2_runtime_outputs_remain_unopened"]
    assert evidence["path_count"] == 100
    assert set(evidence["paths"]) == q2_keys
    assert set(evidence["absence_checks"]) == q2_keys
    assert all(evidence["absence_checks"].values())
    assert evidence["all_absent"] is True
    assert evidence["design_freeze_is_bound_history_not_runtime"] == (
        contract.Q2_DESIGN_FREEZE_ARTIFACT
    )


def test_design_snapshot_is_exact_and_candidate_free() -> None:
    payload = freezer.build_design_freeze()
    snapshot = payload["design_snapshot"]

    assert payload["design_gate"]["passed"] is True
    assert payload["candidate_binding_state"] == "DEFERRED"
    assert payload["candidate_id"] is None
    assert payload["candidate_module"] is None
    assert snapshot["candidate_id"] is None
    assert snapshot["candidate_module"] is None
    assert len(snapshot["holdout_cases"]) == 6
    assert len(snapshot["rollout_matrix"]) == 12
    assert snapshot["matrix_order"] == "BASELINE_SIX_THEN_CANDIDATE_SIX"
    assert snapshot["aggregate"]["required_passing_pair_count"] == 6
    assert snapshot["aggregate"]["allowed_failed_pair_count"] == 0
    assert snapshot["post_pass_next_stage"] == "WAIT_SEPARATE_ZERO_UPDATE_PROTOCOL"
    assert snapshot["tape_abi"] == contract.TAPE_ABI
    assert snapshot["expected_tape_array_sha256"] == (
        contract.EXPECTED_TAPE_ARRAY_SHA256
    )


def test_tape_abi_is_recomputed_in_memory_without_materialization() -> None:
    payload = freezer.build_design_freeze()
    gate = payload["tape_abi_gate"]

    assert gate["passed"] is True
    assert gate["checks"]["closed_five_array_set"] is True
    assert gate["checks"]["shape_dtype_contiguity_exact"] is True
    assert gate["checks"]["array_hashes_exact"] is True
    assert {
        name: record["array_sha256"] for name, record in gate["records"].items()
    } == contract.EXPECTED_TAPE_ARRAY_SHA256
    assert not os.path.lexists(freezer.resolve_relative(contract.NOISE_ROOT))


def test_locked_access_has_no_future_hash_or_execution_authority() -> None:
    access = freezer.build_design_freeze()["qualification_access"]

    assert access["status"] == "LOCKED_PENDING_R5_TERMINAL_PASS_AND_CANDIDATE_BINDING"
    assert access["candidate_binding_state"] == "DEFERRED"
    assert access["candidate_id"] is None
    assert access["candidate_module"] is None
    assert access["future_prerequisite_hashes"] is None
    assert access["qualification_protocol_freeze"] is None
    assert access["qualification_execution_lock"] is None
    assert access["noise_manifest"] is None
    assert access["noise_materialization_authorized"] is False
    assert access["qualification_execution_authorized"] is False


def test_zero_design_activity_is_closed_and_all_zero() -> None:
    payload = freezer.build_design_freeze()

    assert payload["zero_design_activity_gate"]["passed"] is True
    assert all(
        type(value) is int and value == 0
        for value in payload["zero_design_activity"].values()
    )
    assert "noise_random_draws" not in payload["zero_design_activity"]
    assert payload["reference_tape_abi_activity"] == {
        "reference_tape_arrays_generated_in_memory": 5,
        "reference_stochastic_rng_draw_calls": 4,
        "reference_tape_files_written": 0,
        "reference_manifests_written": 0,
    }
    assert payload["reference_tape_arrays_generated_in_memory"] == 5
    assert payload["reference_stochastic_rng_draw_calls"] == 4
    assert payload["runtime_promoted"] is False
    assert payload["checkpoint_zero_created"] is False


def test_occupancy_proves_q1_q3_and_future_r5_outputs_unopened() -> None:
    payload = freezer.build_design_freeze()
    required = {
        "q3_protocol_freeze_absent",
        "q3_execution_lock_absent",
        "q3_noise_tapes_absent",
        "q3_run_root_absent",
        "q1_protocol_freeze_absent",
        "q1_execution_lock_absent",
        "q1_noise_root_absent",
        "q1_run_root_absent",
        "r5_protocol_freeze_absent",
        "r5_execution_lock_absent",
        "r5_candidate_freeze_receipt_absent",
        "r5_final_development_receipt_absent",
        "r5_pipeline_ledger_absent",
    }

    assert all(payload["checks"][name] is True for name in required)


def test_source_drift_fails_closed(monkeypatch: pytest.MonkeyPatch) -> None:
    drifted = copy.deepcopy(contract.FROZEN_EXTERNAL_SOURCE_ARTIFACTS)
    drifted["v6_qualification_contract"]["sha256"] = "0" * 64
    monkeypatch.setattr(contract, "FROZEN_EXTERNAL_SOURCE_ARTIFACTS", drifted)

    payload = freezer.build_design_freeze()

    assert payload["passed"] is False
    assert payload["source_gate"]["passed"] is False
    assert payload["checks"]["closed_source_hashes_bound"] is False


def test_design_drift_fails_closed(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(contract, "CANDIDATE_ID", "forbidden-placeholder")

    payload = freezer.build_design_freeze()

    assert payload["passed"] is False
    assert payload["design_gate"]["passed"] is False
    assert payload["checks"]["exact_deferred_design_preregistered"] is False


@pytest.mark.parametrize(
    "attribute",
    (
        "Q1_DESIGN_FREEZE_ARTIFACT",
        "Q2_DESIGN_FREEZE_ARTIFACT",
        "R4_TERMINAL_LEDGER_ARTIFACT",
    ),
)
def test_historical_artifact_drift_fails_closed(
    monkeypatch: pytest.MonkeyPatch, attribute: str
) -> None:
    drifted = copy.deepcopy(getattr(contract, attribute))
    drifted["sha256"] = "0" * 64
    monkeypatch.setattr(contract, attribute, drifted)

    payload = freezer.build_design_freeze()

    assert payload["passed"] is False
    assert payload["historical_exclusion_gate"]["passed"] is False
    assert payload["checks"]["historical_q1_p1s_q2_r4_excluded"] is False


def test_live_q2_contract_byte_drift_fails_closed(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    real_record = freezer._record
    q2_source_path = (
        "Trajectory Generator/baseline_MLP/validation/v12r4q2/"
        "h0_v12r4_q2_qualification_contract.py"
    )

    def drifted_record(path: object) -> dict[str, object]:
        record = real_record(path)
        if record["path"] == q2_source_path:
            record["sha256"] = "0" * 64
        return record

    monkeypatch.setattr(freezer, "_record", drifted_record)
    payload = freezer.build_design_freeze()

    assert payload["passed"] is False
    assert (
        payload["historical_exclusion_gate"]["checks"][
            "q2_contract_source_still_matches_immutable_design"
        ]
        is False
    )
    assert payload["checks"]["historical_q1_p1s_q2_r4_excluded"] is False


def test_q2_runtime_path_closure_drift_fails_closed(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    drifted = dict(freezer.q2_contract.declared_mutation_paths())
    drifted.pop("pipeline_ledger")
    monkeypatch.setattr(freezer.q2_contract, "declared_mutation_paths", lambda: drifted)

    with pytest.raises(
        freezer.V12R5Q3QualificationDesignFreezeError,
        match="runtime path closure drifted",
    ):
        freezer.build_design_freeze()


def test_one_present_q2_runtime_output_fails_occupancy(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    occupancy = freezer._occupancy_snapshot()
    occupancy["q2_runtime_pipeline_ledger_absent"] = False
    monkeypatch.setattr(freezer, "_occupancy_snapshot", lambda: occupancy)

    payload = freezer.build_design_freeze()

    assert payload["passed"] is False
    assert payload["checks"]["q2_runtime_pipeline_ledger_absent"] is False
    assert payload["checks"]["all_exact_q2_runtime_outputs_remain_unopened"] is False
    assert payload["q2_unopened_runtime"]["all_absent"] is False


def test_opened_output_fails_occupancy(monkeypatch: pytest.MonkeyPatch) -> None:
    occupancy = freezer._occupancy_snapshot()
    occupancy["q3_run_root_absent"] = False
    monkeypatch.setattr(freezer, "_occupancy_snapshot", lambda: occupancy)

    payload = freezer.build_design_freeze()

    assert payload["passed"] is False
    assert payload["checks"]["q3_run_root_absent"] is False


def test_temp_publish_verify_and_no_clobber(tmp_path: Path) -> None:
    destination = tmp_path / "q3-design.json"

    published = freezer.publish_design_freeze(
        output_path=destination,
        enforce_canonical_destination=False,
    )

    assert published["passed"] is True
    assert destination.read_bytes() == freezer.forensic.canonical_json_bytes(published)
    verified = freezer.verify_design_freeze(
        input_path=destination,
        enforce_canonical_destination=False,
    )
    assert verified == published
    with pytest.raises(freezer.V12R5Q3QualificationDesignFreezeError, match="clobber"):
        freezer.publish_design_freeze(
            output_path=destination,
            enforce_canonical_destination=False,
        )
    assert not os.path.lexists(
        freezer.resolve_relative(contract.QUALIFICATION_DESIGN_FREEZE_PATH)
    )


def test_canonical_destination_is_enforced_for_publication(tmp_path: Path) -> None:
    with pytest.raises(
        freezer.V12R5Q3QualificationDesignFreezeError,
        match="non-canonical design freeze destination",
    ):
        freezer.publish_design_freeze(output_path=tmp_path / "wrong.json")


def test_verify_rejects_byte_drift(tmp_path: Path) -> None:
    destination = tmp_path / "q3-design.json"
    freezer.publish_design_freeze(
        output_path=destination,
        enforce_canonical_destination=False,
    )
    destination.write_bytes(destination.read_bytes() + b" ")

    with pytest.raises(freezer.V12R5Q3QualificationDesignFreezeError):
        freezer.verify_design_freeze(
            input_path=destination,
            enforce_canonical_destination=False,
        )


@pytest.mark.parametrize("value", ["../escape.json", "a/../escape.json", ""])
def test_unsafe_relative_paths_are_rejected(value: str) -> None:
    with pytest.raises(
        freezer.V12R5Q3QualificationDesignFreezeError,
        match="unsafe repository-relative path",
    ):
        freezer.resolve_relative(value)


def test_symlink_components_are_rejected(tmp_path: Path) -> None:
    target = tmp_path / "target"
    target.mkdir()
    link = tmp_path / "link"
    try:
        link.symlink_to(target, target_is_directory=True)
    except OSError:
        pytest.skip("symlink creation is unavailable")

    with pytest.raises(
        freezer.V12R5Q3QualificationDesignFreezeError,
        match="link/reparse path rejected",
    ):
        freezer.resolve_relative(link / "design.json")


def test_windows_reparse_detection(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    probe = tmp_path / "probe"
    probe.write_text("x", encoding="utf-8")
    real_lstat = freezer.os.lstat

    class Metadata:
        st_mode = real_lstat(probe).st_mode
        st_file_attributes = 0x400

    monkeypatch.setattr(
        freezer.stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400, raising=False
    )
    monkeypatch.setattr(freezer.os, "lstat", lambda path: Metadata())

    assert freezer._is_link_or_reparse(probe) is True


def test_cli_build_only_does_not_publish(capsys: pytest.CaptureFixture[str]) -> None:
    canonical = freezer.resolve_relative(contract.QUALIFICATION_DESIGN_FREEZE_PATH)
    assert not os.path.lexists(canonical)

    assert freezer.main(["--build-only"]) == 0

    assert "PASS_H0_V12R5_Q3_QUALIFICATION_DESIGN_FREEZE" in capsys.readouterr().out
    assert not os.path.lexists(canonical)


def test_canonical_design_remains_unpublished() -> None:
    assert not os.path.lexists(
        freezer.resolve_relative(contract.QUALIFICATION_DESIGN_FREEZE_PATH)
    )
    assert not os.path.lexists(freezer.resolve_relative(contract.PROTOCOL_FREEZE_PATH))
    assert not os.path.lexists(freezer.resolve_relative(contract.EXECUTION_LOCK_PATH))
    assert not os.path.lexists(freezer.resolve_relative(contract.NOISE_ROOT))
    assert not os.path.lexists(freezer.resolve_relative(contract.RUN_ROOT))
    assert {path.name for path in Q3_ROOT.glob("*.py")} == {
        "__init__.py",
        "freeze_h0_v12r5_q3_qualification_design.py",
        "h0_v12r5_q3_qualification_contract.py",
        "test_h0_v12r5_q3_qualification_contract_and_design.py",
    }
