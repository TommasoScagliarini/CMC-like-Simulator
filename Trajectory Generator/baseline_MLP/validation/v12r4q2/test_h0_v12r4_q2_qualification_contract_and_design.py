"""Tests for the source-only, deferred-candidate V12R4-Q2 design."""

from __future__ import annotations

import copy
import os
import sys
from pathlib import Path

import pytest

Q2_ROOT = Path(__file__).resolve().parent
if str(Q2_ROOT) not in sys.path:
    sys.path.insert(0, str(Q2_ROOT))

import freeze_h0_v12r4_q2_qualification_design as freezer  # noqa: E402
import h0_v12r4_q2_qualification_contract as contract  # noqa: E402


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


def test_future_r4_interface_and_binding_split_are_exact() -> None:
    requirements = contract.prerequisite_requirements()

    assert [row["name"] for row in requirements] == [
        "r4_protocol_freeze",
        "r4_execution_lock",
        "r4_candidate_freeze_receipt",
        "r4_final_development_receipt",
        "r4_terminal_pass_ledger",
    ]
    assert [row["path"] for row in requirements] == [
        (
            "Trajectory Generator/baseline_MLP/validation/v12r4/"
            "h0_v12r4_p3_coverage_protocol_freeze.json"
        ),
        (
            "Trajectory Generator/baseline_MLP/validation/v12r4/"
            "h0_v12r4_p3_coverage_execution_lock.json"
        ),
        (
            "Trajectory Generator/baseline_MLP/validation/v12r4/"
            "h0_v12r4_run_20260809/candidate_freeze_receipt.json"
        ),
        (
            "Trajectory Generator/baseline_MLP/validation/v12r4/"
            "h0_v12r4_run_20260809/final_development_receipt.json"
        ),
        (
            "Trajectory Generator/baseline_MLP/validation/v12r4/"
            "h0_v12r4_run_20260809/pipeline_ledger.json"
        ),
    ]
    assert [row["required_status"] for row in requirements] == [
        "PASS_H0_V12R4_P3_COVERAGE_PROTOCOL_FREEZE",
        "PASS_H0_V12R4_P3_COVERAGE_EXECUTION_LOCK",
        "PASS_H0_V12R4_P3_CANDIDATE_FREEZE",
        "PASS_H0_V12R4_P3_DEVELOPMENT",
        "PASS_H0_V12R4_P3_COVERAGE_PIPELINE_TERMINAL",
    ]
    assert all(row["candidate_identity_required"] is False for row in requirements[:2])
    assert all(row["candidate_tree_required"] is False for row in requirements[:2])
    assert all(
        row["required_selection_rule"] == "SOLE_FIT_P3_OUTPUT_FROM_LOCKED_R4_RUN"
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
        == "SOLE_FIT_P3_OUTPUT_FROM_LOCKED_R4_RUN"
    )


def test_six_holdouts_are_exact_and_live_only_in_q2_namespace() -> None:
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
            "Trajectory Generator/baseline_MLP/validation/v12r4q2/"
            "h0_v12r4_q2_qualification_noise_tapes/deterministic_all_zero.npz"
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
    with pytest.raises(ValueError, match="unknown Q2 qualification case"):
        contract.canonical_case("missing")
    with pytest.raises(ValueError, match="unknown Q2 qualification role"):
        contract.role_contract("missing")
    with pytest.raises(ValueError, match="unknown Q2 qualification role"):
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
    assert provenance["q2_noise_tapes_materialized"] is False
    assert provenance["q2_qualification_rollouts_opened"] is False
    assert provenance["statistical_blindness_claimed"] is False
    assert provenance["r4_may_read_q2_noise"] is False
    assert provenance["r4_may_materialize_q2_noise"] is False
    assert provenance["r4_may_execute_q2_rollouts"] is False
    assert (
        provenance["r4_may_consume_q2_condition_tuples_for_training_or_selection"]
        is False
    )


def test_historical_q1_and_p1s_candidates_are_byte_bound_but_excluded() -> None:
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
    assert contract.HISTORICAL_EXCLUSIONS["q1_candidate_is_not_q2_candidate"] is True
    assert contract.HISTORICAL_EXCLUSIONS["p1s_candidate_is_not_q2_candidate"] is True
    assert (
        contract.HISTORICAL_EXCLUSIONS["historical_candidate_fallback_allowed"] is False
    )


def test_all_declared_mutations_stay_in_dated_q2_namespace() -> None:
    paths = contract.declared_mutation_paths()

    assert contract.RUN_ROOT.name == "h0_v12r4_q2_run_20260809"
    assert len(paths) == len(set(paths.values()))
    assert all(
        path.as_posix().startswith(f"{contract.VALIDATION_ROOT.as_posix()}/")
        for path in paths.values()
    )
    assert contract.QUALIFICATION_DESIGN_FREEZE_PATH != contract.PROTOCOL_FREEZE_PATH
    assert contract.PROTOCOL_FREEZE_PATH != contract.EXECUTION_LOCK_PATH
    assert contract.NEXT_STAGE_AFTER_Q2_PASS == "WAIT_SEPARATE_ZERO_UPDATE_PROTOCOL"


def test_source_closure_includes_cross_platform_attributes_and_safe_reuse_only() -> (
    None
):
    assert contract.DESIGN_SOURCE_RELATIVE_PATHS["q2_line_endings_policy"].endswith(
        "v12r4q2/.gitattributes"
    )
    attributes = freezer.resolve_relative(
        contract.DESIGN_SOURCE_RELATIVE_PATHS["q2_line_endings_policy"]
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
    assert payload["status"] == "PASS_H0_V12R4_Q2_QUALIFICATION_DESIGN_FREEZE"
    assert payload["freeze_kind"] == ("PRE_R4_DEFERRED_CANDIDATE_INDEPENDENT_Q2_DESIGN")
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

    assert access["status"] == "LOCKED_PENDING_R4_TERMINAL_PASS_AND_CANDIDATE_BINDING"
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


def test_occupancy_proves_q1_q2_and_future_r4_outputs_unopened() -> None:
    payload = freezer.build_design_freeze()
    required = {
        "q2_protocol_freeze_absent",
        "q2_execution_lock_absent",
        "q2_noise_tapes_absent",
        "q2_run_root_absent",
        "q1_protocol_freeze_absent",
        "q1_execution_lock_absent",
        "q1_noise_root_absent",
        "q1_run_root_absent",
        "r4_protocol_freeze_absent",
        "r4_execution_lock_absent",
        "r4_candidate_freeze_receipt_absent",
        "r4_final_development_receipt_absent",
        "r4_pipeline_ledger_absent",
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


def test_historical_artifact_drift_fails_closed(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    drifted = copy.deepcopy(contract.Q1_DESIGN_FREEZE_ARTIFACT)
    drifted["sha256"] = "0" * 64
    monkeypatch.setattr(contract, "Q1_DESIGN_FREEZE_ARTIFACT", drifted)

    payload = freezer.build_design_freeze()

    assert payload["passed"] is False
    assert payload["historical_exclusion_gate"]["passed"] is False
    assert payload["checks"]["historical_q1_p1s_excluded"] is False


def test_opened_output_fails_occupancy(monkeypatch: pytest.MonkeyPatch) -> None:
    occupancy = freezer._occupancy_snapshot()
    occupancy["q2_run_root_absent"] = False
    monkeypatch.setattr(freezer, "_occupancy_snapshot", lambda: occupancy)

    payload = freezer.build_design_freeze()

    assert payload["passed"] is False
    assert payload["checks"]["q2_run_root_absent"] is False


def test_temp_publish_verify_and_no_clobber(tmp_path: Path) -> None:
    destination = tmp_path / "q2-design.json"

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
    with pytest.raises(freezer.V12R4Q2QualificationDesignFreezeError, match="clobber"):
        freezer.publish_design_freeze(
            output_path=destination,
            enforce_canonical_destination=False,
        )
    assert not os.path.lexists(
        freezer.resolve_relative(contract.QUALIFICATION_DESIGN_FREEZE_PATH)
    )


def test_canonical_destination_is_enforced_for_publication(tmp_path: Path) -> None:
    with pytest.raises(
        freezer.V12R4Q2QualificationDesignFreezeError,
        match="non-canonical design freeze destination",
    ):
        freezer.publish_design_freeze(output_path=tmp_path / "wrong.json")


def test_verify_rejects_byte_drift(tmp_path: Path) -> None:
    destination = tmp_path / "q2-design.json"
    freezer.publish_design_freeze(
        output_path=destination,
        enforce_canonical_destination=False,
    )
    destination.write_bytes(destination.read_bytes() + b" ")

    with pytest.raises(freezer.V12R4Q2QualificationDesignFreezeError):
        freezer.verify_design_freeze(
            input_path=destination,
            enforce_canonical_destination=False,
        )


@pytest.mark.parametrize("value", ["../escape.json", "a/../escape.json", ""])
def test_unsafe_relative_paths_are_rejected(value: str) -> None:
    with pytest.raises(
        freezer.V12R4Q2QualificationDesignFreezeError,
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
        freezer.V12R4Q2QualificationDesignFreezeError,
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

    assert "PASS_H0_V12R4_Q2_QUALIFICATION_DESIGN_FREEZE" in capsys.readouterr().out
    assert not os.path.lexists(canonical)


def test_canonical_design_remains_unpublished() -> None:
    assert not os.path.lexists(
        freezer.resolve_relative(contract.QUALIFICATION_DESIGN_FREEZE_PATH)
    )
    assert not os.path.lexists(freezer.resolve_relative(contract.PROTOCOL_FREEZE_PATH))
    assert not os.path.lexists(freezer.resolve_relative(contract.EXECUTION_LOCK_PATH))
    assert not os.path.lexists(freezer.resolve_relative(contract.NOISE_ROOT))
    assert not os.path.lexists(freezer.resolve_relative(contract.RUN_ROOT))
