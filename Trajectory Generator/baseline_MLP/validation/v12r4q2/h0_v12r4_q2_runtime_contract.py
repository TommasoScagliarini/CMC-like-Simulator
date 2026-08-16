"""Runtime overlay for the hash-frozen, candidate-deferred Q2 design.

The design contract must remain immutable with a null candidate.  This module
adds the later runtime binding slot: it accepts only the canonical P3 module
from the terminal-PASS V12R4 lineage and then presents the Q1 execution engine
with the same six-case/twelve-rollout interface.  Import is I/O free and leaves
the binding empty.
"""

from __future__ import annotations

import copy
from collections.abc import Mapping
from pathlib import PurePosixPath
from typing import Any

import h0_v12r4_q2_qualification_contract as design


SCHEMA_VERSION = design.SCHEMA_VERSION
REVISION = design.REVISION
PROTOCOL_ID = design.PROTOCOL_ID
PIPELINE_ID = design.PIPELINE_ID
VALIDATION_ROOT = design.VALIDATION_ROOT
RUN_ROOT = design.RUN_ROOT
NOISE_ROOT = design.NOISE_ROOT
NOISE_MANIFEST_PATH = design.NOISE_MANIFEST_PATH
BASELINE_ROOT = design.BASELINE_ROOT
CANDIDATE_ROOT = design.CANDIDATE_ROOT
PAIR_ROOT = design.PAIR_ROOT
FINAL_ROOT = design.FINAL_ROOT
WORKER_CLAIMS_ROOT = design.WORKER_CLAIMS_ROOT
QUALIFICATION_DESIGN_FREEZE_PATH = design.QUALIFICATION_DESIGN_FREEZE_PATH
PROTOCOL_FREEZE_PATH = design.PROTOCOL_FREEZE_PATH
EXECUTION_LOCK_PATH = design.EXECUTION_LOCK_PATH
PIPELINE_CLAIM_PATH = design.PIPELINE_CLAIM_PATH
PIPELINE_LEDGER_PATH = design.PIPELINE_LEDGER_PATH

PROTOCOL_FREEZE_PASS_STATUS = design.PROTOCOL_FREEZE_PASS_STATUS
PROTOCOL_FREEZE_FAIL_STATUS = "FAIL_H0_V12R4_Q2_QUALIFICATION_PROTOCOL_FREEZE"
EXECUTION_LOCK_PASS_STATUS = design.EXECUTION_LOCK_PASS_STATUS
ROLLOUT_COMPLETE_STATUS = "COMPLETE_H0_V12R4_Q2_QUALIFICATION_ROLLOUT"
ROLLOUT_PASS_STATUS = "PASS_H0_V12R4_Q2_QUALIFICATION_ROLLOUT"
ROLLOUT_FAIL_STATUS = "FAIL_H0_V12R4_Q2_QUALIFICATION_ROLLOUT"
PAIR_PASS_STATUS = design.PAIR_PASS_STATUS
PAIR_FAIL_STATUS = design.PAIR_FAIL_STATUS
AGGREGATE_COMPLETE_STATUS = "COMPLETE_H0_V12R4_Q2_INDEPENDENT_QUALIFICATION"
AGGREGATE_PASS_STATUS = design.AGGREGATE_PASS_STATUS
AGGREGATE_FAIL_STATUS = design.AGGREGATE_FAIL_STATUS
PREREQUISITE_COMPLETE_STATUS = "COMPLETE_H0_V12R4_Q2_R4_PREREQUISITES"
PREREQUISITE_PASS_STATUS = "PASS_H0_V12R4_Q2_R4_PREREQUISITES"
PREREQUISITE_FAIL_STATUS = "FAIL_H0_V12R4_Q2_R4_PREREQUISITES"

BASELINE_ROLE = design.BASELINE_ROLE
CANDIDATE_ROLE = design.CANDIDATE_ROLE
ROLE_ORDER = design.ROLE_ORDER
CASE_IDS = design.CASE_IDS
HOLDOUT_CASES = design.HOLDOUT_CASES
ROLLOUT_MATRIX = design.ROLLOUT_MATRIX
STAGE_IDS = design.STAGE_IDS
SOURCE_H0_MODULE = design.SOURCE_H0_MODULE
SOURCE_H0_MODULE_PATH = design.SOURCE_H0_MODULE["path"]
EXPECTED_STEPS = design.EXPECTED_STEPS
EXPECTED_CONTROL_WINDOWS = design.EXPECTED_CONTROL_WINDOWS
EXPECTED_RAW_SENSOR_SAMPLES = design.EXPECTED_RAW_SENSOR_SAMPLES
EXPECTED_ACTOR_FEATURES = design.EXPECTED_ACTOR_FEATURES
EXPECTED_FULL_FEATURES = design.EXPECTED_FULL_FEATURES
EXPECTED_DTYPE = design.EXPECTED_DTYPE
EXPECTED_ACTION_SHAPE = design.EXPECTED_ACTION_SHAPE
MINIMUM_VALID_CYCLES = design.MINIMUM_VALID_CYCLES
PENETRATION_LIMIT_M = design.PENETRATION_LIMIT_M
STOCHASTIC_SEEDS = design.STOCHASTIC_SEEDS
STOCHASTIC_SIGMA = design.STOCHASTIC_SIGMA
JOINTS = design.JOINTS
SEA_SIGNALS = design.SEA_SIGNALS
SEA_EXPECTED_SAMPLE_COUNTS = design.SEA_EXPECTED_SAMPLE_COUNTS
CONTINUOUS_AGGREGATIONS = design.CONTINUOUS_AGGREGATIONS
RESERVE_TOLERANCES = design.RESERVE_TOLERANCES
SEA_TOLERANCES = design.SEA_TOLERANCES
ZERO_REQUIRED_COUNTS = design.ZERO_REQUIRED_COUNTS
TARGET_CONTRACT_ID = design.TARGET_CONTRACT_ID
EVENT_CONTRACT_ID = design.EVENT_CONTRACT_ID
MORPHOLOGY_WEIGHT = design.MORPHOLOGY_WEIGHT
R4_CANDIDATE_MODULE_PATH = design.R4_CANDIDATE_MODULE_PATH
R4_CANDIDATE_SELECTION_RULE = design.R4_CANDIDATE_SELECTION_RULE
FUTURE_PREREQUISITE_REQUIREMENTS = design.FUTURE_PREREQUISITE_REQUIREMENTS

AUTHORITY = {
    "authority_date": REVISION,
    "authority_text": design.AUTHORITY_TEXT,
    "authority_scope": "V12R4_Q2_RUNTIME_SOURCE_READY_CONDITIONAL_EXECUTION",
    "runtime_source_implementation_authorized": True,
    "future_noise_materialization_after_r4_terminal_pass_authorized": True,
    "future_protocol_publication_after_noise_verification_authorized": True,
    "future_execution_lock_after_protocol_authorized": True,
    "future_one_shot_qualification_after_lock_authorized": True,
    "current_turn_noise_materialization_authorized": False,
    "current_turn_protocol_publication_authorized": False,
    "current_turn_execution_lock_publication_authorized": False,
    "current_turn_rollout_execution_authorized": False,
    "actor_fit_authorized": False,
    "offline_teacher_labeling_authorized": False,
    "actor_updates_authorized": False,
    "critic_updates_authorized": False,
    "ppo_updates_authorized": False,
    "retry_authorized": False,
    "resume_authorized": False,
    "rescue_authorized": False,
    "sweep_authorized": False,
    "runtime_promotion_authorized": False,
    "checkpoint_zero_authorized": False,
    "positive_morphology_authorized": False,
}

P1_CANDIDATE_ID: str | None = None
P1_CANDIDATE_MODULE: dict[str, Any] | None = None


class Q2CandidateBindingError(ValueError):
    """Raised when a runtime binding is not the canonical R4 P3 output."""


def _sha256(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def validate_candidate_binding(
    candidate_id: Any, candidate_module: Any
) -> dict[str, Any]:
    if not isinstance(candidate_module, Mapping):
        raise Q2CandidateBindingError("candidate module must be an artifact tree")
    module = copy.deepcopy(dict(candidate_module))
    tree_sha256 = module.get("tree_sha256")
    expected_id = f"h0_v12r4_p3::{tree_sha256}"
    if (
        module.get("path") != R4_CANDIDATE_MODULE_PATH.as_posix()
        or not _sha256(tree_sha256)
        or candidate_id != expected_id
        or type(module.get("file_count")) is not int
        or module["file_count"] < 1
        or not isinstance(module.get("files"), list)
        or len(module["files"]) != module["file_count"]
    ):
        raise Q2CandidateBindingError("candidate is not the canonical R4 P3 output")
    for row in module["files"]:
        if (
            not isinstance(row, Mapping)
            or set(row) != {"path", "sha256", "size_bytes"}
            or not isinstance(row.get("path"), str)
            or not row["path"]
            or not _sha256(row.get("sha256"))
            or type(row.get("size_bytes")) is not int
            or row["size_bytes"] < 1
        ):
            raise Q2CandidateBindingError("candidate tree contains a malformed file")
    return {"candidate_id": candidate_id, "candidate_module": module}


def bind_candidate(candidate_id: Any, candidate_module: Any) -> dict[str, Any]:
    """Bind the exact terminal R4 candidate once, idempotently."""

    global P1_CANDIDATE_ID, P1_CANDIDATE_MODULE
    binding = validate_candidate_binding(candidate_id, candidate_module)
    if P1_CANDIDATE_ID is not None and (
        P1_CANDIDATE_ID != binding["candidate_id"]
        or P1_CANDIDATE_MODULE != binding["candidate_module"]
    ):
        raise Q2CandidateBindingError("candidate binding is immutable")
    P1_CANDIDATE_ID = str(binding["candidate_id"])
    P1_CANDIDATE_MODULE = copy.deepcopy(binding["candidate_module"])
    return current_candidate_binding()


def clear_candidate_binding_for_tests() -> None:
    """Reset process-local state; tests only, never an artifact mutation."""

    global P1_CANDIDATE_ID, P1_CANDIDATE_MODULE
    P1_CANDIDATE_ID = None
    P1_CANDIDATE_MODULE = None


def current_candidate_binding() -> dict[str, Any]:
    if P1_CANDIDATE_ID is None or P1_CANDIDATE_MODULE is None:
        raise Q2CandidateBindingError("Q2 candidate binding remains deferred")
    return {
        "candidate_id": P1_CANDIDATE_ID,
        "candidate_module": copy.deepcopy(P1_CANDIDATE_MODULE),
    }


def prerequisite_requirements() -> tuple[dict[str, Any], ...]:
    return design.prerequisite_requirements()


def canonical_cases() -> tuple[dict[str, Any], ...]:
    return design.canonical_cases()


def canonical_case(case_id: str) -> dict[str, Any]:
    return design.canonical_case(case_id)


def role_contract(role: str) -> dict[str, Any]:
    binding = current_candidate_binding()
    result = design.role_contract(role)
    result["candidate_binding_state"] = "BOUND_FROM_R4_TERMINAL_PASS"
    result["candidate_id"] = binding["candidate_id"]
    if role == BASELINE_ROLE:
        return result
    if role == CANDIDATE_ROLE:
        result["actor_id"] = binding["candidate_id"]
        result["actor_module"] = copy.deepcopy(binding["candidate_module"])
        return result
    raise ValueError(f"unknown Q2 qualification role: {role!r}")


def canonical_rollout(role: str, case_id: str) -> dict[str, Any]:
    case = canonical_case(case_id)
    matches = [
        row
        for row in ROLLOUT_MATRIX
        if row["role"] == role and row["case_id"] == case_id
    ]
    if len(matches) != 1:
        raise ValueError(f"unknown Q2 qualification rollout: {role}/{case_id}")
    return {**case, **copy.deepcopy(matches[0]), **role_contract(role)}


def rollout_root(role: str, case_id: str) -> PurePosixPath:
    return PurePosixPath(canonical_rollout(role, case_id)["destination"])


def rollout_receipt_path(role: str, case_id: str) -> PurePosixPath:
    return rollout_root(role, case_id) / "receipt.json"


def pair_gate_path(case_id: str) -> PurePosixPath:
    return design.pair_gate_path(case_id)


def worker_claim_path(stage_id: str) -> PurePosixPath:
    return design.worker_claim_path(stage_id)


__all__ = [
    "AUTHORITY",
    "P1_CANDIDATE_ID",
    "P1_CANDIDATE_MODULE",
    "Q2CandidateBindingError",
    "bind_candidate",
    "canonical_case",
    "canonical_cases",
    "canonical_rollout",
    "clear_candidate_binding_for_tests",
    "current_candidate_binding",
    "pair_gate_path",
    "prerequisite_requirements",
    "role_contract",
    "rollout_receipt_path",
    "rollout_root",
    "validate_candidate_binding",
    "worker_claim_path",
]
