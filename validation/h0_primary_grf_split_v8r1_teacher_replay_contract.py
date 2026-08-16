"""Pure procedural-correction contract for V8R1/V26 teacher replay.

The original V8 execution is terminal history: its worker stopped before the
first policy step because the inherited V6 collector requested the historical
attribute name ``V25_ACTIVE_EVENT_CONTRACT_ID``.  V8R1 changes no scientific
semantics, geometry, timing, gate, action tape, or trial allocation.  It adds
only an explicit compatibility alias whose value is the frozen V26 contract
and moves every mutable artifact to a fresh no-clobber lineage.
"""

from __future__ import annotations

import copy
from pathlib import PurePosixPath
from typing import Any, Mapping

from validation import h0_primary_grf_split_v8_teacher_replay_contract as v8


SCHEMA_VERSION = 81
REVISION = "2026-08-07"
PROTOCOL_ID = "AB06_H0_PRIMARY_SPLIT_V8R1_V26_RESIDUAL_DAGGER"
COLLECTOR_ID = "H0_V8R1_V26_ACTIVE_TEACHER_ACTION_REPLAY"
LOCK_STATUS = "H0_PRIMARY_SPLIT_V8R1_V26_TEACHER_REPLAY_FROZEN"
PREFLIGHT_STATUS = "PASS_H0_PRIMARY_SPLIT_V8R1_V26_TEACHER_REPLAY_PREFLIGHT"
COMPATIBILITY_PREFLIGHT_STATUS = (
    "PASS_H0_PRIMARY_SPLIT_V8R1_V26_RESET_COMPATIBILITY_PREFLIGHT"
)
COMPATIBILITY_PREFLIGHT_FAIL_STATUS = (
    "FAIL_H0_PRIMARY_SPLIT_V8R1_V26_RESET_COMPATIBILITY_PREFLIGHT"
)
ROLLOUT_COLLECTED_STATUS = "H0_V8R1_V26_TEACHER_REPLAY_COLLECTED_UNGATED"
ROLLOUT_PASS_STATUS = "PASS_H0_V8R1_V26_TEACHER_REPLAY"
ROLLOUT_FAIL_STATUS = "FAIL_H0_V8R1_V26_TEACHER_REPLAY"
PROTOCOL_PASS_STATUS = "PASS_H0_V8R1_V26_TEACHER_REPLAY_DEVELOPMENT"
PROTOCOL_FAIL_STATUS = "FAIL_H0_V8R1_V26_TEACHER_REPLAY_DEVELOPMENT"

SOURCE_H0_ID = v8.SOURCE_H0_ID
SOURCE_OBSERVATION_CONTRACT_ID = v8.SOURCE_OBSERVATION_CONTRACT_ID
V26_ACTIVE_EVENT_CONTRACT_ID = v8.V26_ACTIVE_EVENT_CONTRACT_ID
# Procedural compatibility only.  The inherited V6 engine reads this exact
# attribute name; its value remains the V26 contract, never the V20 contract.
V25_ACTIVE_EVENT_CONTRACT_ID = V26_ACTIVE_EVENT_CONTRACT_ID
TARGET_OBSERVATION_CONTRACT_ID = v8.TARGET_OBSERVATION_CONTRACT_ID
V26_FSM_SOURCE = v8.V26_FSM_SOURCE
V26_ACTOR_ADAPTER_SOURCE = v8.V26_ACTOR_ADAPTER_SOURCE
SO_POLICY_ID = v8.SO_POLICY_ID

EXPECTED_STEPS = v8.EXPECTED_STEPS
EXPECTED_CONTROL_WINDOWS = v8.EXPECTED_CONTROL_WINDOWS
EXPECTED_RAW_SENSOR_SAMPLES = v8.EXPECTED_RAW_SENSOR_SAMPLES
EXPECTED_SAMPLES_PER_STEP = v8.EXPECTED_SAMPLES_PER_STEP
EXPECTED_ACTOR_FEATURES = v8.EXPECTED_ACTOR_FEATURES
EXPECTED_FULL_FEATURES = v8.EXPECTED_FULL_FEATURES
EXPECTED_ACTION_DIM = v8.EXPECTED_ACTION_DIM
EXPECTED_OBSERVATION_DTYPE = v8.EXPECTED_OBSERVATION_DTYPE
EXPECTED_SAMPLE_DT_S = v8.EXPECTED_SAMPLE_DT_S
EXPECTED_POLICY_DT_S = v8.EXPECTED_POLICY_DT_S
EXPECTED_EPISODE_DURATION_S = v8.EXPECTED_EPISODE_DURATION_S
STOCHASTIC_SIGMA = v8.STOCHASTIC_SIGMA
MORPHOLOGY_WEIGHT = v8.MORPHOLOGY_WEIGHT
PENETRATION_LIMIT_M = v8.PENETRATION_LIMIT_M
MINIMUM_VALID_CYCLES = v8.MINIMUM_VALID_CYCLES
INVARIANT_COLUMN_RANGES = v8.INVARIANT_COLUMN_RANGES
INVARIANT_COLUMNS = v8.INVARIANT_COLUMNS
EXPECTED_ACTOR_FEATURE_NAMES = v8.EXPECTED_ACTOR_FEATURE_NAMES
EXPECTED_OBSERVATION_FEATURE_NAMES = v8.EXPECTED_OBSERVATION_FEATURE_NAMES

RUN_ROOT = PurePosixPath(
    "validation/h0_primary_grf_split_adaptation_runs/"
    "2026-08-07_h0_primary_split_v8r1_v26_residual/teacher_replay"
)
LOCK_PATH = PurePosixPath(
    "validation/h0_primary_grf_split_v8r1_teacher_replay_execution_lock.json"
)
PREFLIGHT_RECEIPT_PATH = PurePosixPath(
    "validation/h0_primary_grf_split_v8r1_teacher_replay_preflight_receipt.json"
)
COMPATIBILITY_PREFLIGHT_PATH = PurePosixPath(
    "validation/h0_primary_grf_split_v8r1_compatibility_preflight_receipt.json"
)
EXECUTION_LEDGER_PATH = RUN_ROOT / "execution_ledger.json"
EXECUTION_CLAIM_PATH = RUN_ROOT / "execution_claim.json"
WORKER_CLAIMS_ROOT = RUN_ROOT / "worker_claims"

V5_RUN_ROOT = v8.V5_RUN_ROOT
V5_BASELINE_ROOT = v8.V5_BASELINE_ROOT

CASES = tuple(
    {
        **dict(case),
        "destination": (RUN_ROOT / str(case["case_id"])).as_posix(),
    }
    for case in v8.CASES
)
CASE_IDS = tuple(str(case["case_id"]) for case in CASES)

SOURCE_RELATIVE_PATHS = {
    **{
        name: relative
        for name, relative in v8.SOURCE_RELATIVE_PATHS.items()
        if name not in {"contract", "collector", "contract_tests"}
    },
    "contract": "validation/h0_primary_grf_split_v8r1_teacher_replay_contract.py",
    "collector": "validation/run_h0_primary_grf_split_v8r1_teacher_replay.py",
    "contract_tests": (
        "validation/test_h0_primary_grf_split_v8r1_teacher_replay_contract.py"
    ),
    "terminal_v8_contract": (
        "validation/h0_primary_grf_split_v8_teacher_replay_contract.py"
    ),
    "terminal_v8_collector": (
        "validation/run_h0_primary_grf_split_v8_teacher_replay.py"
    ),
    "terminal_v8_tests": (
        "validation/test_h0_primary_grf_split_v8_teacher_replay_contract.py"
    ),
}

INPUT_RELATIVE_PATHS = {
    **v8.INPUT_RELATIVE_PATHS,
    "v8_terminal_execution_ledger": (
        "validation/h0_primary_grf_split_adaptation_runs/"
        "2026-08-07_h0_primary_split_v8_v26_residual/teacher_replay/"
        "execution_ledger.json"
    ),
    "v8_terminal_worker_failure": (
        "validation/h0_primary_grf_split_adaptation_runs/"
        "2026-08-07_h0_primary_split_v8_v26_residual/teacher_replay/"
        "deterministic_offset_minus_0p20/failure.json"
    ),
    "v8_terminal_execution_lock": (
        "validation/h0_primary_grf_split_v8_teacher_replay_execution_lock.json"
    ),
    "v8_terminal_preflight_receipt": (
        "validation/h0_primary_grf_split_v8_teacher_replay_preflight_receipt.json"
    ),
    "v8r1_compatibility_preflight": COMPATIBILITY_PREFLIGHT_PATH.as_posix(),
}

AUTHORITY = {
    **v8.AUTHORITY,
    "v8r1_procedural_correction_authorized": True,
    "v8_terminal_history_preserved": True,
    "runtime_reset_preflight_authorized": True,
}


def canonical_case(case_id: str) -> dict[str, Any]:
    matches = [case for case in CASES if case["case_id"] == case_id]
    if len(matches) != 1:
        raise ValueError(f"unknown V8R1 teacher replay case: {case_id!r}")
    return copy.deepcopy(matches[0])


def replay_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    """Apply the frozen V8 scientific gate under fresh V8R1 identity."""

    proxy = copy.deepcopy(dict(summary))
    proxy["schema_version"] = v8.SCHEMA_VERSION
    proxy["status"] = v8.ROLLOUT_COLLECTED_STATUS
    proxy["protocol_id"] = v8.PROTOCOL_ID
    proxy["collector_id"] = v8.COLLECTOR_ID
    result = v8.replay_gate(proxy)
    passed = result.get("passed") is True
    return {
        "schema_version": SCHEMA_VERSION,
        "status": ROLLOUT_PASS_STATUS if passed else ROLLOUT_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "case_id": summary.get("case_id"),
        "checks": copy.deepcopy(result["checks"]),
        "gate_lineage": "V8_SCIENTIFIC_GATE_BYTE_FOR_BYTE_SEMANTICS",
        "procedural_correction": (
            "V25_ACTIVE_EVENT_CONTRACT_ID_ALIAS_TO_V26_EXACT"
        ),
        "authority": copy.deepcopy(AUTHORITY),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }


__all__ = [
    "AUTHORITY",
    "CASES",
    "CASE_IDS",
    "COLLECTOR_ID",
    "COMPATIBILITY_PREFLIGHT_PATH",
    "EXECUTION_CLAIM_PATH",
    "EXECUTION_LEDGER_PATH",
    "EXPECTED_ACTOR_FEATURE_NAMES",
    "EXPECTED_ACTOR_FEATURES",
    "EXPECTED_CONTROL_WINDOWS",
    "EXPECTED_FULL_FEATURES",
    "EXPECTED_OBSERVATION_FEATURE_NAMES",
    "EXPECTED_RAW_SENSOR_SAMPLES",
    "EXPECTED_STEPS",
    "INPUT_RELATIVE_PATHS",
    "INVARIANT_COLUMNS",
    "INVARIANT_COLUMN_RANGES",
    "LOCK_PATH",
    "LOCK_STATUS",
    "PREFLIGHT_RECEIPT_PATH",
    "PREFLIGHT_STATUS",
    "PROTOCOL_ID",
    "ROLLOUT_COLLECTED_STATUS",
    "ROLLOUT_FAIL_STATUS",
    "ROLLOUT_PASS_STATUS",
    "RUN_ROOT",
    "SCHEMA_VERSION",
    "SOURCE_RELATIVE_PATHS",
    "TARGET_OBSERVATION_CONTRACT_ID",
    "V25_ACTIVE_EVENT_CONTRACT_ID",
    "V26_ACTIVE_EVENT_CONTRACT_ID",
    "V26_ACTOR_ADAPTER_SOURCE",
    "V26_FSM_SOURCE",
    "WORKER_CLAIMS_ROOT",
    "canonical_case",
    "replay_gate",
]
