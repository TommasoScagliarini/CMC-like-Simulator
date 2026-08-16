"""Pure contract for the H0 primary-split V9 causal relabel stage."""

from __future__ import annotations

import copy
import math
from pathlib import PurePosixPath
from typing import Any, Mapping

from validation import h0_primary_grf_split_v8r1p1_teacher_replay_contract as source


SCHEMA_VERSION = 9
REVISION = "2026-08-07"
PROTOCOL_ID = "AB06_H0_PRIMARY_SPLIT_V9_V26_EVENT_CAUSAL_RESIDUAL"
COLLECTOR_ID = "H0_V9_V26_EVENT_CAUSAL_PRIVILEGED_RELABEL"
TARGET_ID = "H0_ANALOG_LOAD_CONTACT_WITH_V26_EVENTS_FSM_V1"
SOURCE_PROTOCOL_ID = source.PROTOCOL_ID
EVENT_CONTRACT_ID = source.V26_ACTIVE_EVENT_CONTRACT_ID

PREFLIGHT_STATUS = "PASS_H0_PRIMARY_SPLIT_V9_CAUSAL_RELABEL_PREFLIGHT"
LOCK_STATUS = "H0_PRIMARY_SPLIT_V9_CAUSAL_RELABEL_FROZEN"
CASE_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V9_CAUSAL_RELABEL_CASE"
CASE_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V9_CAUSAL_RELABEL_CASE"
PROTOCOL_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V9_CAUSAL_RELABEL"
PROTOCOL_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V9_CAUSAL_RELABEL"

EXPECTED_STEPS = 500
EXPECTED_ACTOR_FEATURES = 35
EXPECTED_ACTION_DIM = 2
EXPECTED_DTYPE = "float32"
PRIVILEGED_INDICES = (10, 11)
CAUSAL_INDICES = tuple(
    index for index in range(EXPECTED_ACTOR_FEATURES)
    if index not in PRIVILEGED_INDICES
)
EXPECTED_SIGMA = 0.005

RUN_ROOT = PurePosixPath(
    "validation/h0_primary_grf_split_adaptation_runs/"
    "2026-08-07_h0_primary_split_v9_v26_causal_residual"
)
RELABEL_ROOT = RUN_ROOT / "causal_teacher"
PREFLIGHT_PATH = PurePosixPath(
    "validation/h0_primary_split_v9_causal_teacher_preflight_receipt.json"
)
LOCK_PATH = PurePosixPath(
    "validation/h0_primary_split_v9_causal_teacher_execution_lock.json"
)
CLAIM_PATH = RELABEL_ROOT / "execution_claim.json"
LEDGER_PATH = RELABEL_ROOT / "execution_ledger.json"

SOURCE_ROOT = source.RUN_ROOT
SOURCE_LEDGER_PATH = source.EXECUTION_LEDGER_PATH
SOURCE_H0_MODULE_PATH = PurePosixPath(
    "validation/critic_warmup/"
    "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/rl_module_last"
)

CASES = tuple(
    {
        "case_id": str(case["case_id"]),
        "source_trace": (
            SOURCE_ROOT / str(case["case_id"]) / "trace.json"
        ).as_posix(),
        "source_receipt": (
            SOURCE_ROOT / str(case["case_id"]) / "receipt.json"
        ).as_posix(),
        "destination": (RELABEL_ROOT / str(case["case_id"])).as_posix(),
    }
    for case in source.CASES
)
CASE_IDS = tuple(case["case_id"] for case in CASES)

SOURCE_RELATIVE_PATHS = {
    "contract": "validation/h0_primary_split_v9_causal_teacher_contract.py",
    "runner": "validation/run_h0_primary_split_v9_causal_teacher.py",
    "helper": "validation/h0_primary_split_v9_causal_teacher.py",
    "helper_tests": "validation/test_h0_primary_split_v9_causal_teacher.py",
    "contract_tests": (
        "validation/test_h0_primary_split_v9_causal_teacher_contract.py"
    ),
    "source_contract": (
        "validation/h0_primary_grf_split_v8r1p1_teacher_replay_contract.py"
    ),
    "source_runner": (
        "validation/run_h0_primary_grf_split_v8r1p1_teacher_replay.py"
    ),
}

INPUT_RELATIVE_PATHS = {
    "source_ledger": SOURCE_LEDGER_PATH.as_posix(),
    "v26_development_receipt": (
        "validation/binary_phase_fsm_v26_development_receipt.json"
    ),
    "v26_v7_replay_receipt": (
        "validation/binary_phase_fsm_v26_v7_replay_receipt.json"
    ),
}

AUTHORITY = {
    "fresh_h0_primary_split_v9_authorized": True,
    "v8_terminal_fail_preserved": True,
    "event_causal_relabel_authorized": True,
    "teacher_privileged_indices": list(PRIVILEGED_INDICES),
    "legacy_event_or_fsm_label_authorized": False,
    "actor_updates_authorized": False,
    "critic_updates_authorized": False,
    "ppo_updates_authorized": False,
    "protected_trial_access_authorized": False,
    "reserve_trial_access_authorized": False,
}


def canonical_case(case_id: str) -> dict[str, Any]:
    rows = [case for case in CASES if case["case_id"] == case_id]
    if len(rows) != 1:
        raise ValueError(f"unknown V9 causal teacher case: {case_id!r}")
    return copy.deepcopy(rows[0])


def _finite(value: Any) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def case_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    case_id = summary.get("case_id")
    try:
        canonical_case(str(case_id))
        known = True
    except ValueError:
        known = False
    checks = {
        "schema": summary.get("schema_version") == SCHEMA_VERSION,
        "protocol": summary.get("protocol_id") == PROTOCOL_ID,
        "collector": summary.get("collector_id") == COLLECTOR_ID,
        "target": summary.get("target_id") == TARGET_ID,
        "known_case": known,
        "steps": summary.get("sample_count") == EXPECTED_STEPS,
        "layout": summary.get("observation_shape") == [EXPECTED_ACTOR_FEATURES]
        and summary.get("action_shape") == [EXPECTED_ACTION_DIM]
        and summary.get("dtype") == EXPECTED_DTYPE,
        "causal_columns": summary.get("causal_column_mismatch_count") == 0,
        "only_10_11_privileged": summary.get("teacher_privileged_indices")
        == list(PRIVILEGED_INDICES),
        "finite": summary.get("nonfinite_count") == 0,
        "bounded": summary.get("out_of_bounds_target_count") == 0,
        "deterministic_query": summary.get("teacher_query_mismatch_count") == 0,
        "nondegenerate": summary.get("target_equals_base_count")
        < EXPECTED_STEPS,
        "no_updates": summary.get("actor_updates") == 0
        and summary.get("critic_updates") == 0
        and summary.get("ppo_updates") == 0,
        "protected_closed": summary.get("protected_trials_opened") == [],
        "reserve_closed": summary.get("reserve_trials_opened") == [],
        "sigma": _finite(summary.get("teacher_sigma"))
        and abs(float(summary["teacher_sigma"]) - EXPECTED_SIGMA) <= 1.0e-8,
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": CASE_PASS_STATUS if passed else CASE_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "case_id": case_id,
        "checks": checks,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }


__all__ = [name for name in globals() if name.isupper()] + [
    "canonical_case",
    "case_gate",
]
