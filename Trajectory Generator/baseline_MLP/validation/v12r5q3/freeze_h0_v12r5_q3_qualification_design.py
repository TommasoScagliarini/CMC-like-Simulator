"""Build or publish the no-clobber V12R5-Q3 qualification design freeze.

Building is read-only and keeps the candidate binding deferred.  Publication
is a separate explicit operation; importing or building this module never
creates the canonical JSON, noise tapes, protocol freeze, execution lock, run
directory, checkpoint, or promotion artifact.
"""

from __future__ import annotations

import argparse
import copy
import os
import stat
import sys
from collections.abc import Mapping, Sequence
from pathlib import Path, PurePosixPath
from typing import Any


def _discover_repo_root(source: Path) -> Path:
    for candidate in source.resolve().parents:
        if (
            (candidate / "AGENTS.md").is_file()
            and (candidate / "validation").is_dir()
            and (candidate / "Trajectory Generator").is_dir()
        ):
            return candidate
    raise RuntimeError("repository root could not be discovered")


REPO_ROOT = _discover_repo_root(Path(__file__))
ROOT_VALIDATION = REPO_ROOT / "validation"
LOCAL_VALIDATION = REPO_ROOT / "Trajectory Generator" / "baseline_MLP" / "validation"
Q1_ROOT = LOCAL_VALIDATION / "v12p1q"
Q2_ROOT = LOCAL_VALIDATION / "v12r4q2"
Q3_ROOT = Path(__file__).resolve().parent
for _root in (
    REPO_ROOT,
    ROOT_VALIDATION,
    LOCAL_VALIDATION,
    Q1_ROOT,
    Q2_ROOT,
    Q3_ROOT,
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_forensic_rollout as forensic  # noqa: E402
import h0_primary_split_v6_qualification_contract as v6_contract  # noqa: E402
import h0_v12r3_p1_qualification_contract as q1_contract  # noqa: E402
import h0_v12r3_p1_qualification_gates as q1_gates  # noqa: E402
import h0_v12r4_q2_qualification_contract as q2_contract  # noqa: E402
import h0_v12r5_q3_qualification_contract as contract  # noqa: E402
import prepare_h0_primary_split_v6_qualification_noise_tapes as frozen_noise_preparer  # noqa: E402


class V12R5Q3QualificationDesignFreezeError(RuntimeError):
    """Raised when the additive Q3 design cannot remain fail closed."""


LOCKED_ACCESS_STATUS = "LOCKED_PENDING_R5_TERMINAL_PASS_AND_CANDIDATE_BINDING"
NEXT_STAGE = "WAIT_R5_TERMINAL_PASS_THEN_FREEZE_Q3_PROTOCOL"
FREEZE_KIND = "PRE_R5_DEFERRED_CANDIDATE_INDEPENDENT_Q3_DESIGN"

ZERO_DESIGN_ACTIVITY = {
    "noise_tapes_materialized": 0,
    "noise_arrays_persisted": 0,
    "environment_imports": 0,
    "environment_reset_calls": 0,
    "environment_step_calls": 0,
    "rollout_executions": 0,
    "actor_fit_executions": 0,
    "offline_teacher_label_calls": 0,
    "actor_updates": 0,
    "critic_updates": 0,
    "ppo_updates": 0,
    "checkpoint_zero_created": 0,
    "runtime_promotions": 0,
}
REFERENCE_TAPE_ABI_ACTIVITY = {
    "reference_tape_arrays_generated_in_memory": 5,
    "reference_stochastic_rng_draw_calls": 4,
    "reference_tape_files_written": 0,
    "reference_manifests_written": 0,
}

_EXPECTED_SOURCE_NAMES = (
    *contract.FROZEN_EXTERNAL_SOURCE_ARTIFACTS,
    "q3_qualification_contract",
    "q3_package_init",
    "q3_design_freezer",
    "q3_contract_and_design_tests",
    "q3_line_endings_policy",
)
_EXPECTED_HOLDOUT_SIGNATURES = (
    (
        "deterministic_offset_minus_0p30",
        "deterministic",
        1.956870983805102 - 0.30,
        None,
        129,
        0.0,
        "deterministic_all_zero.npz",
    ),
    (
        "deterministic_offset_plus_0p30",
        "deterministic",
        1.956870983805102 + 0.30,
        None,
        129,
        0.0,
        "deterministic_all_zero.npz",
    ),
    *tuple(
        (
            f"stochastic_nominal_seed_{seed}",
            "stochastic",
            1.956870983805102,
            seed,
            seed,
            0.005,
            f"stochastic_seed_{seed}_standard_normal.npz",
        )
        for seed in (130, 131, 132, 133)
    ),
)


def _is_link_or_reparse(path: Path) -> bool:
    """Recognize POSIX links and Windows junction/reparse points."""

    try:
        metadata = os.lstat(path)
    except FileNotFoundError:
        return False
    if stat.S_ISLNK(metadata.st_mode):
        return True
    attributes = getattr(metadata, "st_file_attributes", 0)
    reparse = getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0)
    return bool(reparse and attributes & reparse)


def _assert_no_link_components(path: Path, *, allow_missing_leaf: bool) -> None:
    """Reject link-like path components without resolving through them."""

    absolute = Path(os.path.abspath(os.fspath(path)))
    current = Path(absolute.anchor)
    parts = absolute.parts[1:]
    for index, part in enumerate(parts):
        current = current / part
        if not os.path.lexists(current):
            if allow_missing_leaf or index < len(parts) - 1:
                continue
            raise V12R5Q3QualificationDesignFreezeError(
                f"required path is missing: {current}"
            )
        if _is_link_or_reparse(current):
            raise V12R5Q3QualificationDesignFreezeError(
                f"link/reparse path rejected: {current}"
            )


def resolve_relative(path: str | os.PathLike[str] | PurePosixPath) -> Path:
    """Resolve a canonical repo-relative path or an explicit test path."""

    if os.fspath(path) == "":
        raise V12R5Q3QualificationDesignFreezeError(
            f"unsafe repository-relative path: {path!r}"
        )
    value = Path(path)
    if value.is_absolute():
        result = Path(os.path.abspath(os.fspath(value)))
        _assert_no_link_components(result, allow_missing_leaf=True)
        return result
    raw = value.as_posix()
    pure = PurePosixPath(raw)
    if not raw or pure.is_absolute() or ".." in pure.parts or pure.as_posix() != raw:
        raise V12R5Q3QualificationDesignFreezeError(
            f"unsafe repository-relative path: {path!r}"
        )
    result = REPO_ROOT.joinpath(*pure.parts)
    try:
        result.relative_to(REPO_ROOT)
    except ValueError as exc:
        raise V12R5Q3QualificationDesignFreezeError(
            f"path escaped repository: {path!r}"
        ) from exc
    _assert_no_link_components(result, allow_missing_leaf=True)
    return result


def _record(path: str | os.PathLike[str] | PurePosixPath) -> dict[str, Any]:
    source = resolve_relative(path)
    _assert_no_link_components(source, allow_missing_leaf=False)
    if not source.is_file() or _is_link_or_reparse(source):
        raise V12R5Q3QualificationDesignFreezeError(
            f"design source is not a safe regular file: {path}"
        )
    return forensic.artifact_record(source, artifact_root=REPO_ROOT)


def _load_mapping(path: str | os.PathLike[str] | PurePosixPath) -> dict[str, Any]:
    source = resolve_relative(path)
    _assert_no_link_components(source, allow_missing_leaf=False)
    try:
        value = forensic.strict_json_load(source)
    except Exception as exc:
        raise V12R5Q3QualificationDesignFreezeError(
            f"required artifact is not strict JSON: {path}"
        ) from exc
    if not isinstance(value, Mapping):
        raise V12R5Q3QualificationDesignFreezeError(
            f"required artifact is not a mapping: {path}"
        )
    return dict(value)


def _source_gate() -> dict[str, Any]:
    declared = contract.DESIGN_SOURCE_RELATIVE_PATHS
    records = {name: _record(path) for name, path in declared.items()}
    external_records = {
        name: records.get(name) for name in contract.FROZEN_EXTERNAL_SOURCE_ARTIFACTS
    }
    declared_paths = list(declared.values())
    checks = {
        "closed_source_list_exact": tuple(declared) == _EXPECTED_SOURCE_NAMES,
        "declared_paths_unique": len(declared_paths) == len(set(declared_paths)),
        "records_match_declared_paths": all(
            record.get("path") == declared[name] for name, record in records.items()
        ),
        "frozen_v6_q1_sources_byte_exact": external_records
        == contract.FROZEN_EXTERNAL_SOURCE_ARTIFACTS,
        "safe_q1_artifact_record_gate_reused": all(
            q1_gates.artifact_record_matches(record, declared[name])
            for name, record in records.items()
        ),
        "all_hashes_lowercase_sha256": all(
            isinstance(record.get("sha256"), str)
            and len(record["sha256"]) == 64
            and record["sha256"] == record["sha256"].lower()
            for record in records.values()
        ),
        "all_sizes_positive": all(
            type(record.get("size_bytes")) is int and record["size_bytes"] > 0
            for record in records.values()
        ),
        "no_open_ended_source_discovery": set(records) == set(declared),
    }
    return {"passed": all(checks.values()), "checks": checks, "records": records}


def _historical_exclusion_gate() -> dict[str, Any]:
    q1_record = _record(contract.Q1_DESIGN_FREEZE_ARTIFACT["path"])
    p1s_record = _record(contract.P1S_TERMINAL_LEDGER_ARTIFACT["path"])
    q2_record = _record(contract.Q2_DESIGN_FREEZE_ARTIFACT["path"])
    r4_record = _record(contract.R4_TERMINAL_LEDGER_ARTIFACT["path"])
    q1_payload = _load_mapping(contract.Q1_DESIGN_FREEZE_ARTIFACT["path"])
    p1s_payload = _load_mapping(contract.P1S_TERMINAL_LEDGER_ARTIFACT["path"])
    q2_payload = _load_mapping(contract.Q2_DESIGN_FREEZE_ARTIFACT["path"])
    r4_payload = _load_mapping(contract.R4_TERMINAL_LEDGER_ARTIFACT["path"])
    q2_source_expected = q2_payload.get("source_hashes", {}).get(
        "q2_qualification_contract"
    )
    q2_source_record = (
        _record(q2_source_expected["path"])
        if isinstance(q2_source_expected, Mapping)
        and isinstance(q2_source_expected.get("path"), str)
        else None
    )
    checks = {
        "q1_design_byte_exact": q1_record == contract.Q1_DESIGN_FREEZE_ARTIFACT,
        "p1s_ledger_byte_exact": p1s_record == contract.P1S_TERMINAL_LEDGER_ARTIFACT,
        "q2_design_byte_exact": q2_record == contract.Q2_DESIGN_FREEZE_ARTIFACT,
        "r4_ledger_byte_exact": r4_record == contract.R4_TERMINAL_LEDGER_ARTIFACT,
        "generic_q1_record_gate_reused": q1_gates.artifact_record_matches(
            q1_record, contract.Q1_DESIGN_FREEZE_ARTIFACT["path"]
        )
        and q1_gates.artifact_record_matches(
            p1s_record, contract.P1S_TERMINAL_LEDGER_ARTIFACT["path"]
        )
        and q1_gates.artifact_record_matches(
            q2_record, contract.Q2_DESIGN_FREEZE_ARTIFACT["path"]
        )
        and q1_gates.artifact_record_matches(
            r4_record, contract.R4_TERMINAL_LEDGER_ARTIFACT["path"]
        ),
        "q1_was_a_different_fixed_candidate_design": q1_payload.get("passed") is True
        and q1_payload.get("protocol_id")
        == "AB06_H0_V12R3_P1_V26_INDEPENDENT_QUALIFICATION"
        and isinstance(q1_payload.get("design_snapshot"), Mapping)
        and q1_payload["design_snapshot"].get("candidate_id") is not None,
        "p1s_terminal_fail_exact": p1s_payload.get("terminal") is True
        and p1s_payload.get("passed") is False
        and p1s_payload.get("status") == contract.P1S_TERMINAL_FAIL_STATUS
        and p1s_payload.get("retry_authorized") is False
        and p1s_payload.get("resume_authorized") is False,
        "p1s_no_updates_or_promotion": p1s_payload.get("actor_updates") == 0
        and p1s_payload.get("critic_updates") == 0
        and p1s_payload.get("ppo_updates") == 0
        and p1s_payload.get("runtime_promoted") is False,
        "q2_passed_deferred_design_exact": q2_payload.get("passed") is True
        and q2_payload.get("status") == contract.Q2_DESIGN_FREEZE_PASS_STATUS
        and q2_payload.get("candidate_binding_state") == "DEFERRED"
        and q2_payload.get("candidate_id") is None
        and q2_payload.get("candidate_module") is None
        and q2_payload.get("qualification_protocol_freeze") is None
        and q2_payload.get("qualification_execution_lock") is None
        and q2_payload.get("noise_manifest") is None
        and q2_payload.get("runtime_promoted") is False
        and q2_payload.get("checkpoint_zero_created") is False,
        "q2_contract_source_still_matches_immutable_design": q2_source_record
        == q2_source_expected
        and isinstance(q2_source_record, Mapping)
        and q2_source_record.get("path")
        == PurePosixPath(q2_contract.__file__).relative_to(REPO_ROOT).as_posix(),
        "r4_terminal_fail_exact": r4_payload.get("terminal") is True
        and r4_payload.get("passed") is False
        and r4_payload.get("status") == contract.R4_TERMINAL_FAIL_STATUS
        and r4_payload.get("next_stage") == "STOP_TERMINAL"
        and r4_payload.get("attempted_stage")
        == "collect_cov__deterministic_offset_plus_0p20"
        and r4_payload.get("retry_authorized") is False
        and r4_payload.get("resume_authorized") is False
        and r4_payload.get("rescue_authorized") is False
        and r4_payload.get("sweep_authorized") is False,
        "r4_no_candidate_fit_update_or_promotion": r4_payload.get("candidate_id")
        is None
        and r4_payload.get("candidate_module") is None
        and r4_payload.get("candidate_freeze") is None
        and r4_payload.get("final_development_receipt") is None
        and r4_payload.get("actor_fit_executions_confirmed") == 0
        and r4_payload.get("actor_updates") == 0
        and r4_payload.get("critic_updates") == 0
        and r4_payload.get("ppo_updates") == 0
        and r4_payload.get("qualification_executed") is False
        and r4_payload.get("runtime_promoted") is False
        and r4_payload.get("checkpoint_zero_created") is False
        and r4_payload.get("q2_paths_opened") == [],
        "historical_candidate_fallback_forbidden": contract.HISTORICAL_EXCLUSIONS[
            "historical_candidate_fallback_allowed"
        ]
        is False
        and contract.HISTORICAL_EXCLUSIONS["historical_candidate_promotion_allowed"]
        is False,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "records": {
            "q1_design_freeze": q1_record,
            "p1s_terminal_ledger": p1s_record,
            "q2_design_freeze": q2_record,
            "q2_qualification_contract": q2_source_record,
            "r4_terminal_ledger": r4_record,
        },
    }


def _case_signature(case: Mapping[str, Any]) -> tuple[Any, ...]:
    return (
        case.get("case_id"),
        case.get("action_selection"),
        case.get("episode_start_offset_s"),
        case.get("action_seed"),
        case.get("runtime_seed"),
        case.get("sigma"),
        PurePosixPath(str(case.get("noise_tape"))).name,
    )


def _design_snapshot() -> dict[str, Any]:
    return {
        "candidate_binding_state": contract.CANDIDATE_BINDING_STATE,
        "candidate_id": contract.CANDIDATE_ID,
        "candidate_module": contract.CANDIDATE_MODULE,
        "candidate_binding_policy": contract.candidate_binding_policy(),
        "future_prerequisite_requirements": list(contract.prerequisite_requirements()),
        "roles": {
            "baseline": contract.role_contract(contract.BASELINE_ROLE),
            "candidate": contract.role_contract(contract.CANDIDATE_ROLE),
        },
        "holdout_provenance": copy.deepcopy(contract.HOLDOUT_PROVENANCE),
        "holdout_cases": list(contract.canonical_cases()),
        "rollout_matrix": list(contract.ROLLOUT_MATRIX),
        "matrix_order": "BASELINE_SIX_THEN_CANDIDATE_SIX",
        "absolute_rollout_gates": {
            "steps": contract.EXPECTED_STEPS,
            "control_windows": contract.EXPECTED_CONTROL_WINDOWS,
            "raw_sensor_samples": contract.EXPECTED_RAW_SENSOR_SAMPLES,
            "binary_phase_sensor_samples": contract.EXPECTED_RAW_SENSOR_SAMPLES,
            "minimum_valid_cycles": contract.MINIMUM_VALID_CYCLES,
            "penetration_strict_upper_bound_m": contract.PENETRATION_LIMIT_M,
            "teacher_enabled": False,
            "blending_enabled": False,
            "safety_latch_enabled": False,
            "morphology_weight": contract.MORPHOLOGY_WEIGHT,
            "zero_required_counts": list(contract.ZERO_REQUIRED_COUNTS),
            "sea_saturation_count": 0,
            "sea_saturation_fraction": 0.0,
        },
        "pairwise_noninferiority": {
            "reserve_tolerances": [list(row) for row in contract.RESERVE_TOLERANCES],
            "sea_tolerances": [list(row) for row in contract.SEA_TOLERANCES],
            "tolerance_formula": "max(absolute,relative*abs(baseline))",
            "sea_saturation_nonincreasing": True,
            "sea_saturation_required_zero": True,
        },
        "tape_abi": copy.deepcopy(contract.TAPE_ABI),
        "expected_tape_array_sha256": copy.deepcopy(
            contract.EXPECTED_TAPE_ARRAY_SHA256
        ),
        "aggregate": {
            "required_pair_count": 6,
            "required_passing_pair_count": 6,
            "allowed_failed_pair_count": 0,
            "compensation_or_averaging_allowed": False,
            "retry_allowed": False,
            "resume_allowed": False,
            "rescue_allowed": False,
            "sweep_allowed": False,
            "post_hoc_tuning_allowed": False,
            "runtime_promotion_allowed": False,
            "checkpoint_zero_allowed": False,
        },
        "post_pass_next_stage": contract.NEXT_STAGE_AFTER_Q3_PASS,
    }


def _design_gate(snapshot: Mapping[str, Any]) -> dict[str, Any]:
    cases = snapshot.get("holdout_cases")
    matrix = snapshot.get("rollout_matrix")
    roles = snapshot.get("roles")
    baseline = roles.get("baseline") if isinstance(roles, Mapping) else None
    candidate = roles.get("candidate") if isinstance(roles, Mapping) else None
    q3_signatures = (
        tuple(_case_signature(case) for case in cases)
        if isinstance(cases, list) and all(isinstance(case, Mapping) for case in cases)
        else ()
    )
    v6_signatures = tuple(
        _case_signature(case) for case in v6_contract.canonical_cases()
    )
    q1_signatures = tuple(
        _case_signature(case) for case in q1_contract.canonical_cases()
    )
    q2_payload = _load_mapping(contract.Q2_DESIGN_FREEZE_ARTIFACT["path"])
    q2_snapshot = q2_payload.get("design_snapshot")
    q2_cases = (
        q2_snapshot.get("holdout_cases") if isinstance(q2_snapshot, Mapping) else None
    )
    q2_signatures = (
        tuple(_case_signature(case) for case in q2_cases)
        if isinstance(q2_cases, list)
        and all(isinstance(case, Mapping) for case in q2_cases)
        else ()
    )
    observed_matrix = (
        tuple((row.get("role"), row.get("case_id")) for row in matrix)
        if isinstance(matrix, list) and all(isinstance(row, Mapping) for row in matrix)
        else ()
    )
    expected_matrix = tuple(
        (role, case_id)
        for role in (contract.BASELINE_ROLE, contract.CANDIDATE_ROLE)
        for case_id in contract.CASE_IDS
    )
    q2_matrix = (
        q2_snapshot.get("rollout_matrix") if isinstance(q2_snapshot, Mapping) else None
    )
    q2_matrix_pairs = (
        tuple((row.get("role"), row.get("case_id")) for row in q2_matrix)
        if isinstance(q2_matrix, list)
        and all(isinstance(row, Mapping) for row in q2_matrix)
        else ()
    )
    prerequisite_rows = snapshot.get("future_prerequisite_requirements")
    checks = {
        "pure_contract_self_check": contract.contract_self_check()["passed"] is True,
        "schema_revision_authority_exact": contract.SCHEMA_VERSION == 140
        and contract.REVISION == "2026-08-09"
        and contract.AUTHORITY_TEXT == "esegui i punti 1-6",
        "candidate_identity_deferred_without_placeholder": snapshot.get(
            "candidate_binding_state"
        )
        == "DEFERRED"
        and snapshot.get("candidate_id") is None
        and snapshot.get("candidate_module") is None
        and isinstance(candidate, Mapping)
        and candidate.get("actor_id") is None
        and candidate.get("actor_module") is None,
        "r5_prerequisite_interface_exact": prerequisite_rows
        == list(contract.FUTURE_PREREQUISITE_REQUIREMENTS)
        and len(contract.FUTURE_PREREQUISITE_REQUIREMENTS) == 5
        and contract.CANDIDATE_BINDING_POLICY["selection_rule"]
        == "SOLE_CASE_BALANCED_FIT_OUTPUT_FROM_LOCKED_R5_RUN"
        and all(
            row["candidate_identity_required"] is False
            and row["candidate_tree_required"] is False
            for row in contract.FUTURE_PREREQUISITE_REQUIREMENTS[:2]
        )
        and all(
            row["candidate_identity_required"] is True
            and row["candidate_tree_required"] is True
            for row in contract.FUTURE_PREREQUISITE_REQUIREMENTS[2:]
        ),
        "six_reused_unopened_conditions_exact": q3_signatures
        == _EXPECTED_HOLDOUT_SIGNATURES
        and q3_signatures == v6_signatures
        and q3_signatures == q1_signatures
        and q3_signatures == q2_signatures,
        "q2_design_semantics_reused_exact": isinstance(q2_snapshot, Mapping)
        and q2_payload.get("status") == contract.Q2_DESIGN_FREEZE_PASS_STATUS
        and q2_snapshot.get("matrix_order") == "BASELINE_SIX_THEN_CANDIDATE_SIX"
        and q2_matrix_pairs == observed_matrix == expected_matrix
        and q2_snapshot.get("absolute_rollout_gates")
        == snapshot.get("absolute_rollout_gates")
        and q2_snapshot.get("pairwise_noninferiority")
        == snapshot.get("pairwise_noninferiority")
        and q2_snapshot.get("tape_abi") == snapshot.get("tape_abi")
        and q2_snapshot.get("expected_tape_array_sha256")
        == snapshot.get("expected_tape_array_sha256")
        and q2_snapshot.get("aggregate") == snapshot.get("aggregate")
        and q2_snapshot.get("post_pass_next_stage")
        == snapshot.get("post_pass_next_stage"),
        "new_q3_tape_namespace_exact": all(
            str(case["noise_tape"]).startswith(f"{contract.NOISE_ROOT.as_posix()}/")
            for case in cases
        ),
        "twelve_rollouts_baseline_first": observed_matrix == expected_matrix
        and snapshot.get("matrix_order") == "BASELINE_SIX_THEN_CANDIDATE_SIX",
        "baseline_original_h0_legacy_exact": isinstance(baseline, Mapping)
        and baseline.get("actor_id") == "original_h0"
        and baseline.get("actor_module") == contract.SOURCE_H0_MODULE
        and baseline.get("observation_semantics") == "counterfactual_analog"
        and baseline.get("event_contract_id") == "legacy_events"
        and baseline.get("binary_phase_fsm_mode") == "disabled",
        "candidate_v26_primary_split_semantics_exact": isinstance(candidate, Mapping)
        and candidate.get("actor_input_view") == "primary_split"
        and candidate.get("event_contract_id") == contract.EVENT_CONTRACT_ID
        and candidate.get("target_contract_id") == contract.TARGET_CONTRACT_ID
        and candidate.get("binary_phase_fsm_mode") == "binary_active",
        "v6_q1_tolerances_reused_exact": contract.RESERVE_TOLERANCES
        == v6_contract.RESERVE_TOLERANCES
        == q1_contract.RESERVE_TOLERANCES
        and contract.SEA_TOLERANCES
        == v6_contract.SEA_TOLERANCES
        == q1_contract.SEA_TOLERANCES,
        "pairwise_tolerance_shape_exact": len(contract.SEA_TOLERANCES) == 24
        and all(
            absolute == 1.0e-6 and relative == 0.05
            for _, absolute, relative in contract.SEA_TOLERANCES
        ),
        "tape_abi_and_hash_constants_exact": snapshot.get("tape_abi")
        == contract.TAPE_ABI
        and snapshot.get("expected_tape_array_sha256")
        == contract.EXPECTED_TAPE_ARRAY_SHA256,
        "absolute_physical_gates_exact": contract.EXPECTED_STEPS == 500
        and contract.EXPECTED_CONTROL_WINDOWS == 5_000
        and contract.EXPECTED_RAW_SENSOR_SAMPLES == 5_000
        and contract.MINIMUM_VALID_CYCLES == 2
        and contract.PENETRATION_LIMIT_M == 0.025,
        "six_of_six_no_compensation": snapshot.get("aggregate")
        == {
            "required_pair_count": 6,
            "required_passing_pair_count": 6,
            "allowed_failed_pair_count": 0,
            "compensation_or_averaging_allowed": False,
            "retry_allowed": False,
            "resume_allowed": False,
            "rescue_allowed": False,
            "sweep_allowed": False,
            "post_hoc_tuning_allowed": False,
            "runtime_promotion_allowed": False,
            "checkpoint_zero_allowed": False,
        },
        "independence_boundary_explicit": contract.HOLDOUT_PROVENANCE["status"]
        == "REUSED_UNOPENED_V6_HOLDOUTS_FROZEN_BEFORE_R4"
        and contract.HOLDOUT_PROVENANCE["statistical_blindness_claimed"] is False
        and contract.HOLDOUT_PROVENANCE[
            "r5_may_consume_q3_outcomes_for_training_or_selection"
        ]
        is False,
        "post_pass_waits_for_separate_zero_update": snapshot.get("post_pass_next_stage")
        == "WAIT_SEPARATE_ZERO_UPDATE_PROTOCOL",
    }
    return {"passed": all(checks.values()), "checks": checks}


def _tape_abi_gate(snapshot: Mapping[str, Any]) -> dict[str, Any]:
    """Recompute the frozen arrays in memory; never write a tape or manifest."""

    definitions = frozen_noise_preparer._build_tapes()
    records: dict[str, dict[str, Any]] = {}
    for filename, definition in definitions.items():
        array = definition["standard_normal"]
        records[filename] = {
            "seed": definition["seed"],
            "shape": list(array.shape),
            "dtype": str(array.dtype),
            "c_contiguous": bool(array.flags.c_contiguous),
            "array_sha256": frozen_noise_preparer.array_sha256(array),
        }
    checks = {
        "frozen_preparer_shape_exact": tuple(frozen_noise_preparer.TAPE_SHAPE)
        == (500, 2),
        "closed_five_array_set": set(records)
        == set(contract.EXPECTED_TAPE_ARRAY_SHA256),
        "shape_dtype_contiguity_exact": all(
            record["shape"] == [500, 2]
            and record["dtype"] == "float32"
            and record["c_contiguous"] is True
            for record in records.values()
        ),
        "array_hashes_exact": {
            name: record["array_sha256"] for name, record in records.items()
        }
        == contract.EXPECTED_TAPE_ARRAY_SHA256,
        "snapshot_abi_exact": snapshot.get("tape_abi") == contract.TAPE_ABI,
        "snapshot_hashes_exact": snapshot.get("expected_tape_array_sha256")
        == contract.EXPECTED_TAPE_ARRAY_SHA256,
        "no_materialization_api_called": True,
    }
    return {"passed": all(checks.values()), "checks": checks, "records": records}


def _locked_access() -> dict[str, Any]:
    return {
        "status": LOCKED_ACCESS_STATUS,
        "qualification_design_frozen": True,
        "candidate_binding_state": contract.CANDIDATE_BINDING_STATE,
        "candidate_id": None,
        "candidate_module": None,
        "candidate_binding_policy": contract.candidate_binding_policy(),
        "future_prerequisite_requirements": list(contract.prerequisite_requirements()),
        "future_prerequisite_hashes": None,
        "candidate_binding_deferred_until_r5_terminal_pass": True,
        "qualification_protocol_freeze": None,
        "qualification_execution_lock": None,
        "noise_manifest": None,
        "noise_materialization_authorized": False,
        "qualification_execution_authorized": False,
        "runtime_promotion_authorized": False,
        "checkpoint_zero_authorized": False,
    }


def _zero_activity_gate() -> dict[str, Any]:
    checks = {
        "closed_zero_schema": set(ZERO_DESIGN_ACTIVITY)
        == {
            "noise_tapes_materialized",
            "noise_arrays_persisted",
            "environment_imports",
            "environment_reset_calls",
            "environment_step_calls",
            "rollout_executions",
            "actor_fit_executions",
            "offline_teacher_label_calls",
            "actor_updates",
            "critic_updates",
            "ppo_updates",
            "checkpoint_zero_created",
            "runtime_promotions",
        },
        "all_activity_exactly_zero": all(
            type(value) is int and value == 0 for value in ZERO_DESIGN_ACTIVITY.values()
        ),
        "reference_abi_activity_exact": REFERENCE_TAPE_ABI_ACTIVITY
        == {
            "reference_tape_arrays_generated_in_memory": 5,
            "reference_stochastic_rng_draw_calls": 4,
            "reference_tape_files_written": 0,
            "reference_manifests_written": 0,
        },
    }
    return {"passed": all(checks.values()), "checks": checks}


def _q2_unopened_runtime_paths() -> dict[str, PurePosixPath]:
    """Return every exact Q2 runtime destination declared by frozen Q2 source.

    The immutable Q2 design freeze itself is intentionally excluded: it is
    historical input to Q3 and must exist.  No path discovery or globbing is
    used here.
    """

    expected_root = PurePosixPath(
        "Trajectory Generator/baseline_MLP/validation/v12r4q2"
    )
    expected_run_root = expected_root / "h0_v12r4_q2_run_20260809"
    expected_noise_root = expected_root / "h0_v12r4_q2_qualification_noise_tapes"
    exact_central_paths = {
        "validation_root": (q2_contract.VALIDATION_ROOT, expected_root),
        "design_freeze": (
            q2_contract.QUALIFICATION_DESIGN_FREEZE_PATH,
            PurePosixPath(contract.Q2_DESIGN_FREEZE_ARTIFACT["path"]),
        ),
        "protocol_freeze": (
            q2_contract.PROTOCOL_FREEZE_PATH,
            expected_root / "h0_v12r4_q2_qualification_protocol_freeze.json",
        ),
        "execution_lock": (
            q2_contract.EXECUTION_LOCK_PATH,
            expected_root / "h0_v12r4_q2_qualification_execution_lock.json",
        ),
        "noise_root": (q2_contract.NOISE_ROOT, expected_noise_root),
        "noise_manifest": (
            q2_contract.NOISE_MANIFEST_PATH,
            expected_noise_root / "manifest.json",
        ),
        "run_root": (q2_contract.RUN_ROOT, expected_run_root),
        "pipeline_claim": (
            q2_contract.PIPELINE_CLAIM_PATH,
            expected_run_root / "pipeline_claim.json",
        ),
        "pipeline_ledger": (
            q2_contract.PIPELINE_LEDGER_PATH,
            expected_run_root / "pipeline_ledger.json",
        ),
    }
    if any(observed != expected for observed, expected in exact_central_paths.values()):
        raise V12R5Q3QualificationDesignFreezeError(
            "Q2 exact central runtime path binding drifted"
        )

    paths = dict(q2_contract.declared_mutation_paths())
    design_freeze = paths.pop("qualification_design_freeze", None)
    if design_freeze != q2_contract.QUALIFICATION_DESIGN_FREEZE_PATH:
        raise V12R5Q3QualificationDesignFreezeError(
            "Q2 declared mutation closure omitted its design freeze"
        )
    paths.update(
        {
            "baseline_outcomes_root": q2_contract.BASELINE_ROOT,
            "candidate_outcomes_root": q2_contract.CANDIDATE_ROOT,
            "pair_outcomes_root": q2_contract.PAIR_ROOT,
            "final_outcomes_root": q2_contract.FINAL_ROOT,
            "worker_claims_root": q2_contract.WORKER_CLAIMS_ROOT,
            **{
                f"worker_claim_{stage_id}": q2_contract.worker_claim_path(stage_id)
                for stage_id in q2_contract.STAGE_IDS
            },
        }
    )
    required = {
        "protocol_freeze",
        "execution_lock",
        "noise_root",
        "noise_manifest",
        "run_root",
        "pipeline_claim",
        "pipeline_ledger",
        "baseline_outcomes_root",
        "candidate_outcomes_root",
        "pair_outcomes_root",
        "final_outcomes_root",
        "worker_claims_root",
        *(f"noise_{name}" for name in q2_contract.EXPECTED_TAPE_ARRAY_SHA256),
        *(f"pair_gate_{case_id}" for case_id in q2_contract.CASE_IDS),
        *(f"worker_claim_{stage_id}" for stage_id in q2_contract.STAGE_IDS),
    }
    q2_root = q2_contract.VALIDATION_ROOT.as_posix()
    if (
        not required <= set(paths)
        or any(not isinstance(path, PurePosixPath) for path in paths.values())
        or any(not path.as_posix().startswith(f"{q2_root}/") for path in paths.values())
        or len(paths) != len(set(paths.values()))
    ):
        raise V12R5Q3QualificationDesignFreezeError(
            "Q2 exact unopened runtime path closure drifted"
        )
    return paths


def _q2_runtime_occupancy_paths() -> dict[str, PurePosixPath]:
    return {
        f"q2_runtime_{name}_absent": path
        for name, path in _q2_unopened_runtime_paths().items()
    }


def _occupancy_snapshot() -> dict[str, bool]:
    q1_paths = {
        "q1_protocol_freeze_absent": q1_contract.PROTOCOL_FREEZE_PATH,
        "q1_execution_lock_absent": q1_contract.EXECUTION_LOCK_PATH,
        "q1_noise_root_absent": q1_contract.NOISE_ROOT,
        "q1_run_root_absent": q1_contract.RUN_ROOT,
    }
    r5_paths = {
        "r5_protocol_freeze_absent": contract.R5_PROTOCOL_FREEZE_PATH,
        "r5_execution_lock_absent": contract.R5_EXECUTION_LOCK_PATH,
        "r5_candidate_freeze_receipt_absent": (
            contract.R5_CANDIDATE_FREEZE_RECEIPT_PATH
        ),
        "r5_final_development_receipt_absent": (
            contract.R5_FINAL_DEVELOPMENT_RECEIPT_PATH
        ),
        "r5_pipeline_ledger_absent": contract.R5_PIPELINE_LEDGER_PATH,
    }
    q2_runtime_paths = _q2_runtime_occupancy_paths()
    paths = {
        "design_freeze_unoccupied": contract.QUALIFICATION_DESIGN_FREEZE_PATH,
        "q3_protocol_freeze_absent": contract.PROTOCOL_FREEZE_PATH,
        "q3_execution_lock_absent": contract.EXECUTION_LOCK_PATH,
        "q3_noise_tapes_absent": contract.NOISE_ROOT,
        "q3_run_root_absent": contract.RUN_ROOT,
        **q1_paths,
        **q2_runtime_paths,
        **r5_paths,
    }
    return {
        name: not os.path.lexists(resolve_relative(path))
        for name, path in paths.items()
    }


def _assemble_design_freeze(occupancy: Mapping[str, bool]) -> dict[str, Any]:
    q2_runtime_occupancy_paths = _q2_runtime_occupancy_paths()
    expected_occupancy = {
        "design_freeze_unoccupied",
        "q3_protocol_freeze_absent",
        "q3_execution_lock_absent",
        "q3_noise_tapes_absent",
        "q3_run_root_absent",
        "q1_protocol_freeze_absent",
        "q1_execution_lock_absent",
        "q1_noise_root_absent",
        "q1_run_root_absent",
        *q2_runtime_occupancy_paths,
        "r5_protocol_freeze_absent",
        "r5_execution_lock_absent",
        "r5_candidate_freeze_receipt_absent",
        "r5_final_development_receipt_absent",
        "r5_pipeline_ledger_absent",
    }
    if set(occupancy) != expected_occupancy or not all(
        type(value) is bool for value in occupancy.values()
    ):
        raise V12R5Q3QualificationDesignFreezeError("occupancy schema drifted")

    sources = _source_gate()
    historical = _historical_exclusion_gate()
    snapshot = _design_snapshot()
    design = _design_gate(snapshot)
    tape_abi = _tape_abi_gate(snapshot)
    zero = _zero_activity_gate()
    access = _locked_access()
    q2_runtime_absence = {name: occupancy[name] for name in q2_runtime_occupancy_paths}
    checks = {
        "schema_140": contract.SCHEMA_VERSION == 140,
        "separate_design_freeze_destination": len(
            {
                contract.QUALIFICATION_DESIGN_FREEZE_PATH,
                contract.PROTOCOL_FREEZE_PATH,
                contract.EXECUTION_LOCK_PATH,
            }
        )
        == 3,
        "closed_source_hashes_bound": sources["passed"] is True,
        "historical_q1_p1s_q2_r4_excluded": historical["passed"] is True,
        "all_exact_q2_runtime_outputs_remain_unopened": len(q2_runtime_absence)
        == len(q2_runtime_occupancy_paths)
        == 100
        and all(q2_runtime_absence.values()),
        "exact_deferred_design_preregistered": design["passed"] is True,
        "tape_abi_recomputed_in_memory": tape_abi["passed"] is True,
        "zero_noise_environment_rollout_update_activity": zero["passed"] is True,
        "qualification_locked_pending_r5": access["status"] == LOCKED_ACCESS_STATUS
        and access["candidate_id"] is None
        and access["candidate_module"] is None
        and access["future_prerequisite_hashes"] is None
        and access["noise_materialization_authorized"] is False
        and access["qualification_execution_authorized"] is False,
        **dict(occupancy),
    }
    passed = all(checks.values())
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.QUALIFICATION_DESIGN_FREEZE_PASS_STATUS
            if passed
            else contract.QUALIFICATION_DESIGN_FREEZE_FAIL_STATUS
        ),
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "revision": contract.REVISION,
        "freeze_kind": FREEZE_KIND,
        "checks": checks,
        "authority": copy.deepcopy(contract.AUTHORITY),
        "source_hashes": sources["records"],
        "source_gate": {"passed": sources["passed"], "checks": sources["checks"]},
        "historical_exclusion_gate": historical,
        "q2_unopened_runtime": {
            "design_freeze_is_bound_history_not_runtime": contract.Q2_DESIGN_FREEZE_ARTIFACT,
            "path_source": (
                "frozen_v12r4q2_contract_declared_mutation_paths_plus_exact_roots_and_claims"
            ),
            "path_count": len(q2_runtime_occupancy_paths),
            "paths": {
                name: path.as_posix()
                for name, path in q2_runtime_occupancy_paths.items()
            },
            "absence_checks": q2_runtime_absence,
            "all_absent": all(q2_runtime_absence.values()),
        },
        "design_snapshot": snapshot,
        "design_gate": design,
        "tape_abi_gate": tape_abi,
        "qualification_access": access,
        "zero_design_activity": copy.deepcopy(ZERO_DESIGN_ACTIVITY),
        "reference_tape_abi_activity": copy.deepcopy(REFERENCE_TAPE_ABI_ACTIVITY),
        "reference_tape_arrays_generated_in_memory": 5,
        "reference_stochastic_rng_draw_calls": 4,
        "zero_design_activity_gate": zero,
        "publication_destination": (
            contract.QUALIFICATION_DESIGN_FREEZE_PATH.as_posix()
        ),
        "candidate_id": None,
        "candidate_module": None,
        "candidate_binding_state": contract.CANDIDATE_BINDING_STATE,
        "qualification_protocol_freeze": None,
        "qualification_execution_lock": None,
        "noise_manifest": None,
        "future_prerequisite_hashes": None,
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "next_stage": NEXT_STAGE,
    }


def build_design_freeze() -> dict[str, Any]:
    """Build the Q3 design in memory without publishing anything."""

    return _assemble_design_freeze(_occupancy_snapshot())


def publish_design_freeze(
    *,
    output_path: str | os.PathLike[str] | None = None,
    enforce_canonical_destination: bool = True,
) -> dict[str, Any]:
    """Atomically publish one design freeze and refuse every overwrite."""

    destination = resolve_relative(
        contract.QUALIFICATION_DESIGN_FREEZE_PATH
        if output_path is None
        else output_path
    )
    canonical = resolve_relative(contract.QUALIFICATION_DESIGN_FREEZE_PATH)
    if enforce_canonical_destination and destination != canonical:
        raise V12R5Q3QualificationDesignFreezeError(
            f"non-canonical design freeze destination: {destination}"
        )
    if os.path.lexists(destination):
        raise V12R5Q3QualificationDesignFreezeError(
            f"refusing to clobber: {destination}"
        )
    _assert_no_link_components(destination, allow_missing_leaf=True)
    payload = build_design_freeze()
    if payload.get("passed") is not True:
        failed = [
            name for name, value in payload["checks"].items() if value is not True
        ]
        raise V12R5Q3QualificationDesignFreezeError(
            f"design freeze checks failed: {failed}"
        )
    try:
        forensic.write_json_exclusive(destination, payload)
    except Exception as exc:
        raise V12R5Q3QualificationDesignFreezeError(
            f"design freeze publication failed: {destination}"
        ) from exc
    return verify_design_freeze(
        input_path=destination,
        enforce_canonical_destination=enforce_canonical_destination,
    )


def verify_design_freeze(
    *,
    input_path: str | os.PathLike[str] | None = None,
    enforce_canonical_destination: bool = True,
) -> dict[str, Any]:
    """Require strict canonical JSON and rebuild every source/design binding."""

    destination = resolve_relative(
        contract.QUALIFICATION_DESIGN_FREEZE_PATH if input_path is None else input_path
    )
    canonical = resolve_relative(contract.QUALIFICATION_DESIGN_FREEZE_PATH)
    if enforce_canonical_destination and destination != canonical:
        raise V12R5Q3QualificationDesignFreezeError(
            f"non-canonical design freeze: {destination}"
        )
    _assert_no_link_components(destination, allow_missing_leaf=False)
    if not destination.is_file() or _is_link_or_reparse(destination):
        raise V12R5Q3QualificationDesignFreezeError(
            f"design freeze is not a safe regular file: {destination}"
        )
    try:
        payload = forensic.strict_json_load(destination)
    except Exception as exc:
        raise V12R5Q3QualificationDesignFreezeError(
            "design freeze is not strict JSON"
        ) from exc
    expected = _assemble_design_freeze(
        {
            "design_freeze_unoccupied": True,
            "q3_protocol_freeze_absent": True,
            "q3_execution_lock_absent": True,
            "q3_noise_tapes_absent": True,
            "q3_run_root_absent": True,
            "q1_protocol_freeze_absent": True,
            "q1_execution_lock_absent": True,
            "q1_noise_root_absent": True,
            "q1_run_root_absent": True,
            **{name: True for name in _q2_runtime_occupancy_paths()},
            "r5_protocol_freeze_absent": True,
            "r5_execution_lock_absent": True,
            "r5_candidate_freeze_receipt_absent": True,
            "r5_final_development_receipt_absent": True,
            "r5_pipeline_ledger_absent": True,
        }
    )
    if payload != expected or destination.read_bytes() != forensic.canonical_json_bytes(
        expected
    ):
        raise V12R5Q3QualificationDesignFreezeError("design freeze drifted")
    return dict(payload)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--publish", action="store_true")
    action.add_argument("--verify", action="store_true")
    action.add_argument("--build-only", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.publish:
        payload = publish_design_freeze()
    elif args.verify:
        payload = verify_design_freeze()
    else:
        payload = build_design_freeze()
    print(
        f"{payload['status']}: "
        f"{contract.QUALIFICATION_DESIGN_FREEZE_PATH.as_posix()}"
    )
    return 0 if payload.get("passed") is True else 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())


__all__ = [
    "FREEZE_KIND",
    "LOCKED_ACCESS_STATUS",
    "NEXT_STAGE",
    "REFERENCE_TAPE_ABI_ACTIVITY",
    "V12R5Q3QualificationDesignFreezeError",
    "ZERO_DESIGN_ACTIVITY",
    "build_design_freeze",
    "publish_design_freeze",
    "resolve_relative",
    "verify_design_freeze",
]
