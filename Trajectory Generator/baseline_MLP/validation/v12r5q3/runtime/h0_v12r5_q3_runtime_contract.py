"""Candidate-bound runtime overlay for the immutable Q3 design contract.

The parent design remains candidate-deferred and hash-frozen.  This module
holds only process-local binding state and accepts exactly the canonical R5
case-balanced output selected by the terminal-PASS R5 lineage.  Importing it
does no filesystem I/O and creates no runtime artifact.
"""

from __future__ import annotations

import copy
import hashlib
import sys
from collections.abc import Mapping
from pathlib import Path, PurePosixPath
from typing import Any


Q3_ROOT = Path(__file__).resolve().parent.parent
if str(Q3_ROOT) not in sys.path:
    sys.path.insert(0, str(Q3_ROOT))

import h0_v12r5_q3_qualification_contract as design  # noqa: E402


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
PROTOCOL_FREEZE_FAIL_STATUS = "FAIL_H0_V12R5_Q3_QUALIFICATION_PROTOCOL_FREEZE"
EXECUTION_LOCK_PASS_STATUS = design.EXECUTION_LOCK_PASS_STATUS
ROLLOUT_COMPLETE_STATUS = "COMPLETE_H0_V12R5_Q3_QUALIFICATION_ROLLOUT"
ROLLOUT_PASS_STATUS = "PASS_H0_V12R5_Q3_QUALIFICATION_ROLLOUT"
ROLLOUT_FAIL_STATUS = "FAIL_H0_V12R5_Q3_QUALIFICATION_ROLLOUT"
PAIR_PASS_STATUS = design.PAIR_PASS_STATUS
PAIR_FAIL_STATUS = design.PAIR_FAIL_STATUS
AGGREGATE_COMPLETE_STATUS = "COMPLETE_H0_V12R5_Q3_INDEPENDENT_QUALIFICATION"
AGGREGATE_PASS_STATUS = design.AGGREGATE_PASS_STATUS
AGGREGATE_FAIL_STATUS = design.AGGREGATE_FAIL_STATUS
PREREQUISITE_COMPLETE_STATUS = "COMPLETE_H0_V12R5_Q3_R5_PREREQUISITES"
PREREQUISITE_PASS_STATUS = "PASS_H0_V12R5_Q3_R5_PREREQUISITES"
PREREQUISITE_FAIL_STATUS = "FAIL_H0_V12R5_Q3_R5_PREREQUISITES"

BASELINE_ROLE = design.BASELINE_ROLE
CANDIDATE_ROLE = design.CANDIDATE_ROLE
ROLE_ORDER = design.ROLE_ORDER
CASE_IDS = design.CASE_IDS
HOLDOUT_CASES = design.HOLDOUT_CASES
ROLLOUT_MATRIX = design.ROLLOUT_MATRIX
STAGE_IDS = design.STAGE_IDS
SOURCE_H0_MODULE = design.SOURCE_H0_MODULE
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
R5_CANDIDATE_MODULE_PATH = design.R5_CANDIDATE_MODULE_PATH
R5_CANDIDATE_SELECTION_RULE = design.R5_CANDIDATE_SELECTION_RULE
FUTURE_PREREQUISITE_REQUIREMENTS = design.FUTURE_PREREQUISITE_REQUIREMENTS
EXPECTED_TAPE_ARRAY_SHA256 = design.EXPECTED_TAPE_ARRAY_SHA256
TAPE_ABI = design.TAPE_ABI

AUTHORITY = {
    "authority_date": REVISION,
    "authority_text": design.AUTHORITY_TEXT,
    "authority_scope": "V12R5_Q3_RUNTIME_CONDITIONAL_ONE_SHOT",
    "runtime_source_implementation_authorized": True,
    "candidate_binding_requires_five_official_r5_verifiers": True,
    "qualification_execution_requires_frozen_protocol_and_lock": True,
    "one_shot": True,
    "retry_authorized": False,
    "resume_authorized": False,
    "rescue_authorized": False,
    "sweep_authorized": False,
    "post_hoc_tuning_authorized": False,
    "actor_fit_authorized": False,
    "offline_teacher_labeling_authorized": False,
    "actor_updates_authorized": False,
    "critic_updates_authorized": False,
    "ppo_updates_authorized": False,
    "runtime_promotion_authorized": False,
    "checkpoint_zero_authorized": False,
    "positive_morphology_authorized": False,
}

R5_CANDIDATE_ID: str | None = None
R5_CANDIDATE_MODULE: dict[str, Any] | None = None


class Q3CandidateBindingError(ValueError):
    """Raised when a runtime candidate is not the exact R5 output."""


def _sha256(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def _canonical_file_path(value: Any) -> bool:
    if not isinstance(value, str) or not value:
        return False
    path = PurePosixPath(value)
    return (
        not path.is_absolute() and ".." not in path.parts and path.as_posix() == value
    )


def _tree_digest(rows: list[dict[str, Any]]) -> str:
    digest = hashlib.sha256()
    for row in rows:
        digest.update(row["path"].encode("utf-8"))
        digest.update(b"\0")
        digest.update(row["sha256"].encode("ascii"))
        digest.update(b"\0")
        digest.update(str(row["size_bytes"]).encode("ascii"))
        digest.update(b"\n")
    return digest.hexdigest()


def validate_candidate_binding(
    candidate_id: Any, candidate_module: Any
) -> dict[str, Any]:
    if not isinstance(candidate_module, Mapping):
        raise Q3CandidateBindingError("candidate module must be an artifact tree")
    module = copy.deepcopy(dict(candidate_module))
    rows = module.get("files")
    if (
        set(module) != {"path", "tree_sha256", "file_count", "files"}
        or module.get("path") != R5_CANDIDATE_MODULE_PATH.as_posix()
        or not _sha256(module.get("tree_sha256"))
        or type(module.get("file_count")) is not int
        or module["file_count"] < 1
        or not isinstance(rows, list)
        or len(rows) != module["file_count"]
    ):
        raise Q3CandidateBindingError("candidate is not the canonical R5 output")
    normalized: list[dict[str, Any]] = []
    for row in rows:
        if (
            not isinstance(row, Mapping)
            or set(row) != {"path", "sha256", "size_bytes"}
            or not _canonical_file_path(row.get("path"))
            or not _sha256(row.get("sha256"))
            or type(row.get("size_bytes")) is not int
            or row["size_bytes"] < 1
        ):
            raise Q3CandidateBindingError("candidate tree contains a malformed file")
        normalized.append(copy.deepcopy(dict(row)))
    if [row["path"] for row in normalized] != sorted(
        row["path"] for row in normalized
    ) or len({row["path"] for row in normalized}) != len(normalized):
        raise Q3CandidateBindingError("candidate tree paths are not canonical")
    if _tree_digest(normalized) != module["tree_sha256"]:
        raise Q3CandidateBindingError("candidate tree digest does not match its files")
    expected_id = f"AB06_H0_V12R5_CASE_BALANCED:{module['tree_sha256']}"
    if candidate_id != expected_id:
        raise Q3CandidateBindingError("candidate identity does not bind the tree")
    return {"candidate_id": candidate_id, "candidate_module": module}


def bind_candidate(candidate_id: Any, candidate_module: Any) -> dict[str, Any]:
    """Bind the official-verifier-approved R5 candidate once per process."""

    global R5_CANDIDATE_ID, R5_CANDIDATE_MODULE
    binding = validate_candidate_binding(candidate_id, candidate_module)
    if R5_CANDIDATE_ID is not None and (
        R5_CANDIDATE_ID != binding["candidate_id"]
        or R5_CANDIDATE_MODULE != binding["candidate_module"]
    ):
        raise Q3CandidateBindingError("candidate binding is immutable")
    R5_CANDIDATE_ID = str(binding["candidate_id"])
    R5_CANDIDATE_MODULE = copy.deepcopy(binding["candidate_module"])
    return current_candidate_binding()


def clear_candidate_binding_for_tests() -> None:
    """Reset process-local state without touching any artifact."""

    global R5_CANDIDATE_ID, R5_CANDIDATE_MODULE
    R5_CANDIDATE_ID = None
    R5_CANDIDATE_MODULE = None


def current_candidate_binding() -> dict[str, Any]:
    if R5_CANDIDATE_ID is None or R5_CANDIDATE_MODULE is None:
        raise Q3CandidateBindingError("Q3 candidate binding remains deferred")
    return {
        "candidate_id": R5_CANDIDATE_ID,
        "candidate_module": copy.deepcopy(R5_CANDIDATE_MODULE),
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
    result["candidate_binding_state"] = "BOUND_FROM_R5_TERMINAL_PASS"
    result["candidate_id"] = binding["candidate_id"]
    if role == BASELINE_ROLE:
        return result
    if role == CANDIDATE_ROLE:
        result["actor_id"] = binding["candidate_id"]
        result["actor_module"] = copy.deepcopy(binding["candidate_module"])
        result.pop("binding_policy", None)
        return result
    raise ValueError(f"unknown Q3 qualification role: {role!r}")


def canonical_rollout(role: str, case_id: str) -> dict[str, Any]:
    case = canonical_case(case_id)
    matches = [
        row
        for row in ROLLOUT_MATRIX
        if row["role"] == role and row["case_id"] == case_id
    ]
    if len(matches) != 1:
        raise ValueError(f"unknown Q3 qualification rollout: {role}/{case_id}")
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
    "Q3CandidateBindingError",
    "R5_CANDIDATE_ID",
    "R5_CANDIDATE_MODULE",
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
