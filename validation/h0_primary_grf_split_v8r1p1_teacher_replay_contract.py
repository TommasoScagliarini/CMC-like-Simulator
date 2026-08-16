"""Final thin V8R1P1 overlay preserving V8/V8R1 terminal closures.

V8R1P1 keeps the inherited verifier's frozen procedural ``next_stage`` label
while retaining the exact V26 compatibility alias introduced by V8R1.  No
scientific value, gate, action tape, timing, geometry, or trial allocation is
changed.
"""

from __future__ import annotations

import copy
from pathlib import PurePosixPath
from typing import Any, Mapping

from validation import h0_primary_grf_split_v8r1_teacher_replay_contract as prior


SCHEMA_VERSION = 811
REVISION = "2026-08-07"
PROTOCOL_ID = "AB06_H0_PRIMARY_SPLIT_V8R1P1_V26_RESIDUAL_DAGGER"
COLLECTOR_ID = "H0_V8R1P1_V26_ACTIVE_TEACHER_ACTION_REPLAY"
LOCK_STATUS = "H0_PRIMARY_SPLIT_V8R1P1_V26_TEACHER_REPLAY_FROZEN"
PREFLIGHT_STATUS = "PASS_H0_PRIMARY_SPLIT_V8R1P1_V26_TEACHER_REPLAY_PREFLIGHT"
COMPATIBILITY_PREFLIGHT_STATUS = (
    "PASS_H0_PRIMARY_SPLIT_V8R1P1_V26_COMPATIBILITY_PREFLIGHT"
)
ROLLOUT_COLLECTED_STATUS = "H0_V8R1P1_V26_TEACHER_REPLAY_COLLECTED_UNGATED"
ROLLOUT_PASS_STATUS = "PASS_H0_V8R1P1_V26_TEACHER_REPLAY"
ROLLOUT_FAIL_STATUS = "FAIL_H0_V8R1P1_V26_TEACHER_REPLAY"
PROTOCOL_PASS_STATUS = "PASS_H0_V8R1P1_V26_TEACHER_REPLAY_DEVELOPMENT"
PROTOCOL_FAIL_STATUS = "FAIL_H0_V8R1P1_V26_TEACHER_REPLAY_DEVELOPMENT"

for _name in (
    "SOURCE_H0_ID",
    "SOURCE_OBSERVATION_CONTRACT_ID",
    "V26_ACTIVE_EVENT_CONTRACT_ID",
    "V25_ACTIVE_EVENT_CONTRACT_ID",
    "TARGET_OBSERVATION_CONTRACT_ID",
    "V26_FSM_SOURCE",
    "V26_ACTOR_ADAPTER_SOURCE",
    "SO_POLICY_ID",
    "EXPECTED_STEPS",
    "EXPECTED_CONTROL_WINDOWS",
    "EXPECTED_RAW_SENSOR_SAMPLES",
    "EXPECTED_SAMPLES_PER_STEP",
    "EXPECTED_ACTOR_FEATURES",
    "EXPECTED_FULL_FEATURES",
    "EXPECTED_ACTION_DIM",
    "EXPECTED_OBSERVATION_DTYPE",
    "EXPECTED_SAMPLE_DT_S",
    "EXPECTED_POLICY_DT_S",
    "EXPECTED_EPISODE_DURATION_S",
    "STOCHASTIC_SIGMA",
    "MORPHOLOGY_WEIGHT",
    "PENETRATION_LIMIT_M",
    "MINIMUM_VALID_CYCLES",
    "INVARIANT_COLUMN_RANGES",
    "INVARIANT_COLUMNS",
    "EXPECTED_ACTOR_FEATURE_NAMES",
    "EXPECTED_OBSERVATION_FEATURE_NAMES",
    "V5_RUN_ROOT",
    "V5_BASELINE_ROOT",
):
    globals()[_name] = getattr(prior, _name)

RUN_ROOT = PurePosixPath(
    "validation/h0_primary_grf_split_adaptation_runs/"
    "2026-08-07_h0_primary_split_v8r1p1_v26_residual/teacher_replay"
)
LOCK_PATH = PurePosixPath(
    "validation/h0_primary_grf_split_v8r1p1_teacher_replay_execution_lock.json"
)
PREFLIGHT_RECEIPT_PATH = PurePosixPath(
    "validation/h0_primary_grf_split_v8r1p1_teacher_replay_preflight_receipt.json"
)
COMPATIBILITY_PREFLIGHT_PATH = PurePosixPath(
    "validation/h0_primary_grf_split_v8r1p1_compatibility_preflight_receipt.json"
)
EXECUTION_LEDGER_PATH = RUN_ROOT / "execution_ledger.json"
EXECUTION_CLAIM_PATH = RUN_ROOT / "execution_claim.json"
WORKER_CLAIMS_ROOT = RUN_ROOT / "worker_claims"

CASES = tuple(
    {
        **dict(case),
        "destination": (RUN_ROOT / str(case["case_id"])).as_posix(),
    }
    for case in prior.CASES
)
CASE_IDS = tuple(str(case["case_id"]) for case in CASES)

SOURCE_RELATIVE_PATHS = {
    **{
        name: relative
        for name, relative in prior.SOURCE_RELATIVE_PATHS.items()
        if name not in {"contract", "collector", "contract_tests"}
    },
    "contract": (
        "validation/h0_primary_grf_split_v8r1p1_teacher_replay_contract.py"
    ),
    "collector": (
        "validation/run_h0_primary_grf_split_v8r1p1_teacher_replay.py"
    ),
    "contract_tests": (
        "validation/test_h0_primary_grf_split_v8r1p1_teacher_replay_contract.py"
    ),
    "preexecution_v8r1_contract": (
        "validation/h0_primary_grf_split_v8r1_teacher_replay_contract.py"
    ),
    "preexecution_v8r1_collector": (
        "validation/run_h0_primary_grf_split_v8r1_teacher_replay.py"
    ),
    "preexecution_v8r1_tests": (
        "validation/test_h0_primary_grf_split_v8r1_teacher_replay_contract.py"
    ),
}

INPUT_RELATIVE_PATHS = {
    **{
        name: relative
        for name, relative in prior.INPUT_RELATIVE_PATHS.items()
        if name != "v8r1_compatibility_preflight"
    },
    "v8r1_reset_compatibility_receipt": (
        "validation/h0_primary_grf_split_v8r1_compatibility_preflight_receipt.json"
    ),
    "v8r1_preexecution_preflight": (
        "validation/h0_primary_grf_split_v8r1_teacher_replay_preflight_receipt.json"
    ),
    "v8r1_preexecution_lock": (
        "validation/h0_primary_grf_split_v8r1_teacher_replay_execution_lock.json"
    ),
    "v8r1p1_compatibility_preflight": COMPATIBILITY_PREFLIGHT_PATH.as_posix(),
}

AUTHORITY = {
    **prior.AUTHORITY,
    "v8r1p1_literal_binding_correction_authorized": True,
    "v8r1_preexecution_closure_preserved": True,
}


def canonical_case(case_id: str) -> dict[str, Any]:
    matches = [case for case in CASES if case["case_id"] == case_id]
    if len(matches) != 1:
        raise ValueError(f"unknown V8R1P1 case: {case_id!r}")
    return copy.deepcopy(matches[0])


def replay_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    proxy = copy.deepcopy(dict(summary))
    proxy["schema_version"] = prior.SCHEMA_VERSION
    proxy["status"] = prior.ROLLOUT_COLLECTED_STATUS
    proxy["protocol_id"] = prior.PROTOCOL_ID
    proxy["collector_id"] = prior.COLLECTOR_ID
    result = prior.replay_gate(proxy)
    passed = result.get("passed") is True
    return {
        "schema_version": SCHEMA_VERSION,
        "status": ROLLOUT_PASS_STATUS if passed else ROLLOUT_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "case_id": summary.get("case_id"),
        "checks": copy.deepcopy(result["checks"]),
        "gate_lineage": "V8_SCIENTIFIC_GATE_UNCHANGED",
        "procedural_correction": "FROZEN_VERIFIER_NEXT_STAGE_LITERAL_PRESERVED",
        "authority": copy.deepcopy(AUTHORITY),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
