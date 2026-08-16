"""Freeze the independent V12R3-P1 qualification design before development.

This module preregisters the already source-frozen V6 holdout conditions and
the exact P1/V26 comparison design.  Building is read-only.  Publication is an
explicit, atomic, no-clobber operation to a design-freeze path that is separate
from the future qualification protocol freeze.  It never materializes noise,
imports an environment, executes a rollout, updates a model, or unlocks the
qualification.
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
V12P1Q_ROOT = Path(__file__).resolve().parent
for _root in (REPO_ROOT, ROOT_VALIDATION, LOCAL_VALIDATION, V12P1Q_ROOT):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_forensic_rollout as forensic  # noqa: E402
import h0_primary_split_v6_qualification_contract as v6_contract  # noqa: E402
import h0_v12r3_p1_qualification_contract as contract  # noqa: E402


class V12R3P1QualificationDesignFreezeError(RuntimeError):
    """Raised when the prospective qualification design cannot stay closed."""


def _is_link_or_reparse(path: Path) -> bool:
    """Recognize POSIX symlinks and Windows junction/reparse points."""

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
    """Fail closed on link-like components without resolving through them."""

    absolute = Path(os.path.abspath(os.fspath(path)))
    anchor = Path(absolute.anchor)
    current = anchor
    for index, part in enumerate(absolute.parts[1:]):
        current = current / part
        if not os.path.lexists(current):
            if allow_missing_leaf or index < len(absolute.parts[1:]) - 1:
                continue
            raise V12R3P1QualificationDesignFreezeError(
                f"required path is missing: {current}"
            )
        if _is_link_or_reparse(current):
            raise V12R3P1QualificationDesignFreezeError(
                f"link/reparse path rejected: {current}"
            )


EXPECTED_P1_CANDIDATE_ID = (
    "AB06_H0_PRIMARY_SPLIT_V12R3_V26_AUTONOMY_RECOVERY:" "p1:ff34e153ae0ac9b6"
)
EXPECTED_P1_TREE_SHA256 = (
    "ff34e153ae0ac9b6f7b8d7d92766e47eecf087285020ac5332d9bd41170ac3ed"
)
EXPECTED_EVENT_CONTRACT_ID = "binary_point_v25+heel_qualified_fsm_v2"
EXPECTED_TARGET_CONTRACT_ID = (
    "primary_grf_split_v1+binary_point_v25+heel_qualified_fsm_v2"
)
LOCKED_ACCESS_STATUS = "LOCKED_PENDING_V12P1S_SIX_OF_SIX_PASS"
NEXT_STAGE = "WAIT_V12P1S_SIX_OF_SIX_THEN_FREEZE_QUALIFICATION_PROTOCOL"

ZERO_DESIGN_ACTIVITY = {
    "noise_tapes_materialized": 0,
    "noise_random_draws": 0,
    "environment_imports": 0,
    "environment_reset_calls": 0,
    "environment_step_calls": 0,
    "rollout_executions": 0,
    "actor_fit_executions": 0,
    "offline_teacher_label_calls": 0,
    "actor_updates": 0,
    "critic_updates": 0,
    "ppo_updates": 0,
}

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


def resolve_relative(path: str | os.PathLike[str] | PurePosixPath) -> Path:
    """Resolve one canonical repository-relative path or an explicit test path."""

    value = Path(path)
    if value.is_absolute():
        result = Path(os.path.abspath(os.fspath(value)))
        _assert_no_link_components(result, allow_missing_leaf=True)
        return result
    raw = value.as_posix()
    pure = PurePosixPath(raw)
    if not raw or pure.is_absolute() or ".." in pure.parts or pure.as_posix() != raw:
        raise V12R3P1QualificationDesignFreezeError(
            f"unsafe repository-relative path: {path!r}"
        )
    result = REPO_ROOT.joinpath(*pure.parts)
    try:
        result.relative_to(REPO_ROOT)
    except ValueError as exc:
        raise V12R3P1QualificationDesignFreezeError(
            f"path escaped repository: {path!r}"
        ) from exc
    _assert_no_link_components(result, allow_missing_leaf=True)
    return result


def _record(path: str | os.PathLike[str] | PurePosixPath) -> dict[str, Any]:
    source = resolve_relative(path)
    _assert_no_link_components(source, allow_missing_leaf=False)
    if not source.is_file() or _is_link_or_reparse(source):
        raise V12R3P1QualificationDesignFreezeError(
            f"design source is not a safe regular file: {path}"
        )
    return forensic.artifact_record(source, artifact_root=REPO_ROOT)


def _source_gate() -> dict[str, Any]:
    declared = contract.DESIGN_SOURCE_RELATIVE_PATHS
    records = {name: _record(path) for name, path in declared.items()}
    declared_paths = list(declared.values())
    v6_records = {
        name: records.get(name) for name in contract.V6_HOLDOUT_SOURCE_ARTIFACTS
    }
    checks = {
        "closed_eight_source_list": list(declared)
        == [
            "v6_qualification_contract",
            "v6_qualification_gates",
            "v6_noise_preparer",
            "qualification_contract",
            "qualification_gates",
            "qualification_design_freezer",
            "qualification_contract_and_gates_tests",
            "qualification_design_freezer_tests",
        ],
        "declared_paths_unique": len(declared_paths) == len(set(declared_paths)),
        "records_match_declared_paths": all(
            record.get("path") == declared[name] for name, record in records.items()
        ),
        "original_v6_sources_byte_exact": (
            v6_records == contract.V6_HOLDOUT_SOURCE_ARTIFACTS
        ),
        "all_hashes_and_sizes_valid": all(
            isinstance(record.get("sha256"), str)
            and len(record["sha256"]) == 64
            and type(record.get("size_bytes")) is int
            and record["size_bytes"] > 0
            for record in records.values()
        ),
        "no_future_open_ended_source_discovery": set(records) == set(declared),
    }
    return {"passed": all(checks.values()), "checks": checks, "records": records}


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
    baseline = contract.role_contract(contract.BASELINE_ROLE)
    candidate = contract.role_contract(contract.CANDIDATE_ROLE)
    return {
        "candidate_id": contract.P1_CANDIDATE_ID,
        "candidate_module": copy.deepcopy(contract.P1_CANDIDATE_MODULE),
        "roles": {"baseline": baseline, "candidate": candidate},
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
        "aggregate": {
            "required_pair_count": 6,
            "required_passing_pair_count": 6,
            "allowed_failed_pair_count": 0,
            "compensation_or_averaging_allowed": False,
            "retry_allowed": False,
            "rescue_allowed": False,
            "sweep_allowed": False,
            "post_hoc_tuning_allowed": False,
            "runtime_promotion_allowed": False,
        },
    }


def _design_gate(snapshot: Mapping[str, Any]) -> dict[str, Any]:
    cases = snapshot.get("holdout_cases")
    matrix = snapshot.get("rollout_matrix")
    roles = snapshot.get("roles")
    baseline = roles.get("baseline") if isinstance(roles, Mapping) else None
    candidate = roles.get("candidate") if isinstance(roles, Mapping) else None
    v6_cases = v6_contract.canonical_cases()
    v6_signatures = tuple(_case_signature(case) for case in v6_cases)
    new_signatures = (
        tuple(_case_signature(case) for case in cases)
        if isinstance(cases, list)
        else ()
    )
    expected_matrix = [
        (role, case_id)
        for role in (contract.BASELINE_ROLE, contract.CANDIDATE_ROLE)
        for case_id in contract.CASE_IDS
    ]
    observed_matrix = (
        [(row.get("role"), row.get("case_id")) for row in matrix]
        if isinstance(matrix, list) and all(isinstance(row, Mapping) for row in matrix)
        else []
    )
    zero_counts = set(contract.ZERO_REQUIRED_COUNTS)
    required_zero_counts = {
        "action_clipped_values",
        "fallback_count",
        "safety_latch_activation_count",
        "sea_plugin_fallback_count",
        "hard_invalid_count",
        "invalid_event_count",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
    }
    checks = {
        "schema_revision_authority_exact": contract.SCHEMA_VERSION == 125
        and contract.REVISION == "2026-08-09"
        and contract.AUTHORITY_TEXT == "esegui i punti 1-6",
        "exact_p1_identity": snapshot.get("candidate_id") == EXPECTED_P1_CANDIDATE_ID
        and isinstance(snapshot.get("candidate_module"), Mapping)
        and snapshot["candidate_module"].get("tree_sha256") == EXPECTED_P1_TREE_SHA256
        and snapshot.get("candidate_module") == contract.P1_CANDIDATE_MODULE,
        "six_v6_holdouts_exact_and_preexisting": len(new_signatures) == 6
        and new_signatures == _EXPECTED_HOLDOUT_SIGNATURES
        and new_signatures == v6_signatures,
        "twelve_rollouts_baseline_first": len(observed_matrix) == 12
        and observed_matrix == expected_matrix
        and snapshot.get("matrix_order") == "BASELINE_SIX_THEN_CANDIDATE_SIX",
        "baseline_original_h0_legacy_exact": isinstance(baseline, Mapping)
        and baseline.get("actor_id") == "original_h0"
        and baseline.get("actor_input_view") == "historical_analog"
        and baseline.get("observation_semantics") == "counterfactual_analog"
        and baseline.get("event_contract_id") == "legacy_events"
        and baseline.get("binary_phase_fsm_mode") == "disabled",
        "candidate_v26_primary_split_exact": isinstance(candidate, Mapping)
        and candidate.get("actor_id") == EXPECTED_P1_CANDIDATE_ID
        and candidate.get("actor_input_view") == "primary_split"
        and candidate.get("event_contract_id") == EXPECTED_EVENT_CONTRACT_ID
        and candidate.get("target_contract_id") == EXPECTED_TARGET_CONTRACT_ID
        and candidate.get("binary_phase_fsm_mode") == "binary_active",
        "morphology_zero": contract.MORPHOLOGY_WEIGHT == 0.0
        and baseline.get("morphology_weight") == 0.0
        and candidate.get("morphology_weight") == 0.0,
        "physical_gate_constants_exact": contract.EXPECTED_STEPS == 500
        and contract.EXPECTED_CONTROL_WINDOWS == 5_000
        and contract.EXPECTED_RAW_SENSOR_SAMPLES == 5_000
        and contract.MINIMUM_VALID_CYCLES == 2
        and contract.PENETRATION_LIMIT_M == 0.025
        and required_zero_counts <= zero_counts,
        "v6_tolerances_unchanged": contract.RESERVE_TOLERANCES
        == v6_contract.RESERVE_TOLERANCES
        and contract.SEA_TOLERANCES == v6_contract.SEA_TOLERANCES,
        "pairwise_tolerance_shape_exact": contract.RESERVE_TOLERANCES
        == (
            ("reserve_norm_nm.rms", 5.0, 0.05),
            ("reserve_norm_nm.abs_max", 5.0, 0.05),
            ("residual_norm_nm.rms", 1.0e-6, 0.05),
            ("residual_norm_nm.abs_max", 1.0e-6, 0.05),
        )
        and all(
            absolute == 1.0e-6 and relative == 0.05
            for _, absolute, relative in contract.SEA_TOLERANCES
        )
        and len(contract.SEA_TOLERANCES) == 24,
        "six_of_six_no_compensation": snapshot.get("aggregate")
        == {
            "required_pair_count": 6,
            "required_passing_pair_count": 6,
            "allowed_failed_pair_count": 0,
            "compensation_or_averaging_allowed": False,
            "retry_allowed": False,
            "rescue_allowed": False,
            "sweep_allowed": False,
            "post_hoc_tuning_allowed": False,
            "runtime_promotion_allowed": False,
        },
    }
    return {"passed": all(checks.values()), "checks": checks}


def _locked_access() -> dict[str, Any]:
    return {
        "status": LOCKED_ACCESS_STATUS,
        "qualification_design_frozen": True,
        "qualification_protocol_freeze": None,
        "qualification_execution_lock": None,
        "future_prerequisite_requirements": list(contract.prerequisite_requirements()),
        "future_prerequisite_hashes": None,
        "hash_binding_deferred_until_salvage_terminal_pass": True,
        "required_salvage_rollout_count": 6,
        "required_salvage_passing_rollout_count": 6,
        "required_salvage_failed_rollout_count": 0,
        "required_salvage_terminal_status": contract.SALVAGE_PIPELINE_PASS_STATUS,
        "noise_materialization_authorized": False,
        "qualification_execution_authorized": False,
        "runtime_promotion_authorized": False,
    }


def _zero_activity_gate() -> dict[str, Any]:
    checks = {
        "closed_zero_schema": set(ZERO_DESIGN_ACTIVITY)
        == {
            "noise_tapes_materialized",
            "noise_random_draws",
            "environment_imports",
            "environment_reset_calls",
            "environment_step_calls",
            "rollout_executions",
            "actor_fit_executions",
            "offline_teacher_label_calls",
            "actor_updates",
            "critic_updates",
            "ppo_updates",
        },
        "all_activity_exactly_zero": all(
            type(value) is int and value == 0 for value in ZERO_DESIGN_ACTIVITY.values()
        ),
    }
    return {"passed": all(checks.values()), "checks": checks}


def _occupancy_snapshot() -> dict[str, bool]:
    return {
        "design_freeze_unoccupied": not os.path.lexists(
            resolve_relative(contract.QUALIFICATION_DESIGN_FREEZE_PATH)
        ),
        "qualification_protocol_freeze_absent": not os.path.lexists(
            resolve_relative(contract.PROTOCOL_FREEZE_PATH)
        ),
        "qualification_execution_lock_absent": not os.path.lexists(
            resolve_relative(contract.EXECUTION_LOCK_PATH)
        ),
        "qualification_noise_root_absent": not os.path.lexists(
            resolve_relative(contract.NOISE_ROOT)
        ),
        "qualification_run_root_absent": not os.path.lexists(
            resolve_relative(contract.RUN_ROOT)
        ),
        "salvage_run_root_absent": not os.path.lexists(
            resolve_relative(contract.SALVAGE_RUN_ROOT)
        ),
    }


def _assemble_design_freeze(occupancy: Mapping[str, bool]) -> dict[str, Any]:
    expected_occupancy = {
        "design_freeze_unoccupied",
        "qualification_protocol_freeze_absent",
        "qualification_execution_lock_absent",
        "qualification_noise_root_absent",
        "qualification_run_root_absent",
        "salvage_run_root_absent",
    }
    if set(occupancy) != expected_occupancy or not all(
        type(value) is bool for value in occupancy.values()
    ):
        raise V12R3P1QualificationDesignFreezeError("occupancy schema drifted")

    sources = _source_gate()
    snapshot = _design_snapshot()
    design = _design_gate(snapshot)
    zero = _zero_activity_gate()
    access = _locked_access()
    checks = {
        "schema_125": contract.SCHEMA_VERSION == 125,
        "separate_design_freeze_destination": (
            contract.QUALIFICATION_DESIGN_FREEZE_PATH != contract.PROTOCOL_FREEZE_PATH
            and contract.QUALIFICATION_DESIGN_FREEZE_PATH
            != contract.EXECUTION_LOCK_PATH
        ),
        "closed_source_hashes_bound": sources["passed"] is True,
        "exact_design_preregistered": design["passed"] is True,
        "zero_noise_environment_rollout_update_activity": zero["passed"] is True,
        "qualification_locked_pending_v12p1s_six_of_six": (
            access["status"] == LOCKED_ACCESS_STATUS
            and access["future_prerequisite_hashes"] is None
            and access["required_salvage_rollout_count"] == 6
            and access["required_salvage_passing_rollout_count"] == 6
            and access["required_salvage_failed_rollout_count"] == 0
            and access["noise_materialization_authorized"] is False
            and access["qualification_execution_authorized"] is False
        ),
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
        "freeze_kind": "PRE_SALVAGE_INDEPENDENT_QUALIFICATION_DESIGN",
        "checks": checks,
        "authority": copy.deepcopy(contract.AUTHORITY),
        "source_hashes": sources["records"],
        "source_gate": {"passed": sources["passed"], "checks": sources["checks"]},
        "design_snapshot": snapshot,
        "design_gate": design,
        "qualification_access": access,
        "zero_design_activity": copy.deepcopy(ZERO_DESIGN_ACTIVITY),
        "zero_design_activity_gate": zero,
        "publication_destination": contract.QUALIFICATION_DESIGN_FREEZE_PATH.as_posix(),
        "qualification_protocol_freeze": None,
        "qualification_execution_lock": None,
        "noise_manifest": None,
        "salvage_artifact_hashes": None,
        "runtime_promoted": False,
        "next_stage": NEXT_STAGE,
    }


def build_design_freeze() -> dict[str, Any]:
    """Build the pre-development design freeze in memory without publishing."""

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
        raise V12R3P1QualificationDesignFreezeError(
            f"non-canonical design freeze destination: {destination}"
        )
    if os.path.lexists(destination):
        raise V12R3P1QualificationDesignFreezeError(
            f"refusing to clobber: {destination}"
        )
    _assert_no_link_components(destination, allow_missing_leaf=True)
    payload = build_design_freeze()
    if payload.get("passed") is not True:
        failed = [
            name for name, value in payload["checks"].items() if value is not True
        ]
        raise V12R3P1QualificationDesignFreezeError(
            f"design freeze checks failed: {failed}"
        )
    try:
        forensic.write_json_exclusive(destination, payload)
    except Exception as exc:
        raise V12R3P1QualificationDesignFreezeError(
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
    """Rebuild every source/design binding and require canonical identity."""

    destination = resolve_relative(
        contract.QUALIFICATION_DESIGN_FREEZE_PATH if input_path is None else input_path
    )
    canonical = resolve_relative(contract.QUALIFICATION_DESIGN_FREEZE_PATH)
    if enforce_canonical_destination and destination != canonical:
        raise V12R3P1QualificationDesignFreezeError(
            f"non-canonical design freeze: {destination}"
        )
    _assert_no_link_components(destination, allow_missing_leaf=False)
    if not destination.is_file() or _is_link_or_reparse(destination):
        raise V12R3P1QualificationDesignFreezeError(
            f"design freeze is not a safe regular file: {destination}"
        )
    try:
        payload = forensic.strict_json_load(destination)
    except Exception as exc:
        raise V12R3P1QualificationDesignFreezeError(
            "design freeze is not strict JSON"
        ) from exc
    expected = _assemble_design_freeze(
        {
            "design_freeze_unoccupied": True,
            "qualification_protocol_freeze_absent": True,
            "qualification_execution_lock_absent": True,
            "qualification_noise_root_absent": True,
            "qualification_run_root_absent": True,
            "salvage_run_root_absent": True,
        }
    )
    if payload != expected or destination.read_bytes() != forensic.canonical_json_bytes(
        expected
    ):
        raise V12R3P1QualificationDesignFreezeError("design freeze drifted")
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
    "LOCKED_ACCESS_STATUS",
    "V12R3P1QualificationDesignFreezeError",
    "ZERO_DESIGN_ACTIVITY",
    "build_design_freeze",
    "publish_design_freeze",
    "resolve_relative",
    "verify_design_freeze",
]
