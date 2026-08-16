"""Execution-free contract for V12R7 same-state recovery imitation.

V12R7 is additive: the terminal V12R6 run remains immutable evidence.  The
frozen R6 actor is used only as a behaviour policy for six observer-only
rollouts.  H0 labels are queried offline, after each physical trace is closed,
and a single ordinary 35->512->512->2 actor is fitted on the locked base
corpus plus the same-state recovery rows.  Teacher, blending and safety latch
are forbidden in every candidate-development rollout.

Importing this module performs no I/O, inference, fitting, randomness,
environment access, or publication.
"""

from __future__ import annotations

import copy
import math
import sys
from pathlib import Path, PurePosixPath
from typing import Any, Mapping


_SOURCE = Path(__file__).resolve()
_LOCAL_VALIDATION = _SOURCE.parent.parent
_REPO_ROOT = _SOURCE.parents[4]
for _root in (
    _SOURCE.parent,
    _LOCAL_VALIDATION / "v12r6",
    _LOCAL_VALIDATION / "v12r5",
    _LOCAL_VALIDATION,
    _REPO_ROOT / "validation",
    _REPO_ROOT / "Trajectory Generator" / "baseline_MLP",
    _REPO_ROOT,
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_v12r6_functional_composite_contract as v12r6  # noqa: E402


SCHEMA_VERSION = 1270
REVISION = "2026-08-14"
PROTOCOL_ID = "AB06_H0_V12R7_SAME_STATE_RECOVERY_W512_V26"
PIPELINE_ID = "H0_V12R7_R6_OBSERVER_DAGGER_SINGLE_FIT_CRITICAL_FIRST"
FIT_CONTRACT_ID = "h0_v12r7_same_state_recovery_full_mean_w512_v1"
TOPOLOGY_ID = "V12R7_STANDARD_RECOVERY_W512_V1"
CANDIDATE_SELECTION_RULE = "SOLE_R7_SINGLE_FIT_OUTPUT_IF_ALL_LOCKED_GATES_PASS"
PROTOCOL_FREEZE_PASS_STATUS = "PASS_H0_V12R7_RECOVERY_PROTOCOL_FREEZE"
EXECUTION_LOCK_PASS_STATUS = "PASS_H0_V12R7_RECOVERY_EXECUTION_LOCK"
CANDIDATE_FREEZE_PASS_STATUS = "PASS_H0_V12R7_RECOVERY_CANDIDATE_FREEZE"
DEVELOPMENT_PASS_STATUS = "PASS_H0_V12R7_RECOVERY_DEVELOPMENT"
PIPELINE_TERMINAL_PASS_STATUS = "PASS_H0_V12R7_RECOVERY_PIPELINE_TERMINAL"
ACTOR_FEATURE_MANIFEST_STATUS = "H0_V12R7_RECOVERY_ACTOR_FEATURE_CONTRACT"

AUTHORITY = {
    "authority_date": REVISION,
    "authority_text": "procedi fino a un protocollo training ready",
    "source_implementation_authorized": True,
    "protocol_freeze_authorized": True,
    "execution_lock_authorized": True,
    "new_environment_collection_authorized": True,
    "observer_only_teacher_queries_authorized": True,
    "actor_fit_execution_authorized": True,
    "candidate_freeze_authorized": True,
    "development_execution_authorized": True,
    "qualification_execution_authorized": False,
    "checkpoint_zero_authorized": False,
    "positive_morphology_authorized": False,
    "retry_authorized": False,
    "resume_authorized": False,
    "alpha_sweep_authorized": False,
    "detector_or_fsm_change_authorized": False,
}

ROOT = PurePosixPath("Trajectory Generator/baseline_MLP/validation/v12r7")
PROTOCOL_FREEZE_PATH = ROOT / "h0_v12r7_recovery_protocol_freeze.json"
EXECUTION_LOCK_PATH = ROOT / "h0_v12r7_recovery_execution_lock.json"
RUN_ROOT = ROOT / "h0_v12r7_run_20260814"
CLAIM_PATH = RUN_ROOT / "pipeline_claim.json"
LEDGER_PATH = RUN_ROOT / "pipeline_ledger.json"
COLLECTION_ROOT = RUN_ROOT / "observer_collection"
FIT_ROOT = RUN_ROOT / "fit"
CORPUS_PATH = FIT_ROOT / "corpus.npz"
CANDIDATE_MODULE_PATH = FIT_ROOT / "rl_module_recovery"
FIT_SUMMARY_PATH = FIT_ROOT / "summary.json"
FIT_GATE_PATH = FIT_ROOT / "gate.json"
FIT_RECEIPT_PATH = FIT_ROOT / "receipt.json"
CANDIDATE_FREEZE_PATH = RUN_ROOT / "candidate_freeze_receipt.json"
DEVELOPMENT_ROOT = RUN_ROOT / "development"
FINAL_DEVELOPMENT_PATH = RUN_ROOT / "final_development_receipt.json"

R6_RUN_ROOT = v12r6.RUN_ROOT
R6_CANDIDATE_MODULE_PATH = v12r6.CANDIDATE_MODULE_PATH
R6_TERMINAL_LEDGER = v12r6.PIPELINE_LEDGER_PATH
R6_CANDIDATE_FREEZE = v12r6.CANDIDATE_FREEZE_PATH
BASE_CORPUS_PATH = v12r6.v12r5.CORPUS_PATH
R4_PLUS_LABELS_PATH = PurePosixPath(
    "Trajectory Generator/baseline_MLP/validation/v12r4/"
    "h0_v12r4_run_20260809/collect/"
    "deterministic_offset_plus_0p20/labels.npz"
)
SOURCE_H0_MODULE_PATH = v12r6.v12r5.SOURCE_H0_MODULE_PATH

LOCKED_INPUTS = {
    "r6_terminal_ledger": {
        "path": R6_TERMINAL_LEDGER.as_posix(),
        "sha256": "ce67aea83b1f98aa251ad130af1da25982435381c0aeddeafa0a56bd3274e340",
        "size_bytes": 6725,
    },
    "r6_candidate_freeze": {
        "path": R6_CANDIDATE_FREEZE.as_posix(),
        "sha256": "3bd5e983f2305a6f3a8b390d554efd9144b678da754cb86f248198985db1b4ad",
        "size_bytes": 2779,
    },
    "r6_candidate": {
        "path": R6_CANDIDATE_MODULE_PATH.as_posix(),
        "tree_sha256": "340c2c65c2300a90ce46c09837e679a99e5dea09ce3935574ef5345fafb709f3",
        "file_count": 5,
    },
    "base_corpus": {
        "path": BASE_CORPUS_PATH.as_posix(),
        "sha256": "db7014f5e8735062c2e340244babfaa13a4f571b14902d1a30fb9cd04cf23cf3",
        "size_bytes": 21_771_902,
        "rows": 9_232,
    },
    "r4_failed_plus_labels": {
        "path": R4_PLUS_LABELS_PATH.as_posix(),
        "sha256": "9ac32c57ceed2863ffc8b7b98d09b6590c04324f65e6fd3b104923a1c1f05981",
        "size_bytes": 342_790,
        "rows": 212,
    },
}

EXPECTED_ACTOR_FEATURES = 35
EXPECTED_FULL_FEATURES = 84
EXPECTED_ACTION_DIM = 2
EXPECTED_STEPS = 500
RAW_SAMPLES_PER_STEP = 10
EXPECTED_SIGMA = 0.005
DISABLED_CLOCK_COLUMNS = (0, 1)
PENETRATION_LIMIT_M = 0.025
MINIMUM_VALID_CYCLES = 2
MINIMUM_RECOVERABLE_PREFIX_STEPS = 100
TARGET_CONTRACT_ID = v12r6.TARGET_CONTRACT_ID
EVENT_CONTRACT_ID = v12r6.EVENT_CONTRACT_ID

COLLECTION_CASE_IDS = (
    "deterministic_offset_plus_0p20",
    "deterministic_offset_minus_0p20",
    "deterministic_offset_nominal",
    "stochastic_nominal_seed_126",
    "stochastic_nominal_seed_127",
    "stochastic_nominal_seed_128",
)
DEVELOPMENT_CASE_IDS = COLLECTION_CASE_IDS
COLLECTION_CASES = tuple(
    {
        **v12r6.canonical_development_case(case_id),
        "destination": (COLLECTION_ROOT / case_id).as_posix(),
        "behavior": "FROZEN_R6_PURE_POLICY_OBSERVER_ONLY",
    }
    for case_id in COLLECTION_CASE_IDS
)
DEVELOPMENT_CASES = tuple(
    {
        **v12r6.canonical_development_case(case_id),
        "destination": (DEVELOPMENT_ROOT / case_id).as_posix(),
        "behavior": "R7_PURE_UNBLENDED_NO_TEACHER_NO_LATCH",
    }
    for case_id in DEVELOPMENT_CASE_IDS
)

FIT = {
    "fit_contract_id": FIT_CONTRACT_ID,
    "initial_actor": "FROZEN_R6_W512_COMPOSITE",
    "architecture": {
        "kind": "standard_mean_mlp",
        "input_dim": 35,
        "hidden_dims": [512, 512],
        "output_dim": 2,
        "activation": "tanh",
        "residual_actor": False,
    },
    "base_corpus_rows": 9_232,
    "r4_recovery_rows": 212,
    "observer_cases": list(COLLECTION_CASE_IDS),
    "stratum_policy": "SIX_BASE_CASES_PLUS_SIX_OBSERVER_CASES_PLUS_R4_FAILURE",
    "stratum_target_mass": 500.0,
    "stratum_count": 13,
    "within_stratum_weighting": "UNIFORM",
    "optimizer": {
        "name": "AdamW_then_LBFGS",
        "seed": 20260814,
        "adamw_epochs": 2_000,
        "adamw_learning_rates": [3.0e-4, 1.0e-4, 3.0e-5],
        "adamw_boundaries": [1_000, 1_700, 2_000],
        "weight_decay": 1.0e-7,
        "lbfgs_max_iter": 300,
        "lbfgs_max_eval": 600,
    },
    "logstd_frozen": True,
    "clock_columns_bit_zero": True,
    "critic_present": False,
    "actor_fit_count": 1,
    "critic_update_count": 0,
    "ppo_update_count": 0,
}

OFFLINE_THRESHOLDS = {
    "global": {"rmse": 0.006, "max_abs_error": 0.060},
    "per_case": {"rmse": 0.006, "max_abs_error": 0.060},
    "reset_max_abs_error": 0.003,
    "r4_failed_plus": {"rmse": 0.006, "max_abs_error": 0.060},
    "observer_case": {"rmse": 0.006, "max_abs_error": 0.060},
}

STAGE_IDS = (
    *(f"collect_label__{case_id}" for case_id in COLLECTION_CASE_IDS),
    "fit_recovery_actor",
    "freeze_recovery_actor",
    *(f"development__{case_id}" for case_id in DEVELOPMENT_CASE_IDS),
    "finalize_development",
)


def _strict_int(value: Any, *, minimum: int = 0) -> bool:
    return type(value) is int and value >= minimum


def _zero_int(value: Any) -> bool:
    return type(value) is int and value == 0


def _finite(value: Any) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def canonical_collection_case(case_id: str) -> dict[str, Any]:
    for case in COLLECTION_CASES:
        if case["case_id"] == case_id:
            return copy.deepcopy(case)
    raise ValueError(f"unknown V12R7 collection case: {case_id!r}")


def canonical_development_case(case_id: str) -> dict[str, Any]:
    for case in DEVELOPMENT_CASES:
        if case["case_id"] == case_id:
            return copy.deepcopy(case)
    raise ValueError(f"unknown V12R7 development case: {case_id!r}")


def collection_case_root(case_id: str) -> PurePosixPath:
    canonical_collection_case(case_id)
    return COLLECTION_ROOT / case_id


def observer_label_path(case_id: str) -> PurePosixPath:
    return collection_case_root(case_id) / "observer_labels" / "labels.npz"


def candidate_id(tree_sha256: str) -> str:
    if (
        not isinstance(tree_sha256, str)
        or len(tree_sha256) != 64
        or any(char not in "0123456789abcdef" for char in tree_sha256)
    ):
        raise ValueError("candidate tree digest must be lowercase SHA-256")
    return f"AB06_H0_V12R7_RECOVERY_W512:{tree_sha256}"


def collection_integrity_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    steps = summary.get("steps")
    terminated = summary.get("terminated")
    truncated = summary.get("truncated")
    recoverable_end = (
        terminated is True
        and truncated is False
        and summary.get("end_reason") == "grf_penetration"
        and _strict_int(steps, minimum=MINIMUM_RECOVERABLE_PREFIX_STEPS)
    )
    complete_end = (
        terminated is False
        and truncated is True
        and summary.get("end_reason") == "episode_time_limit"
        and steps == EXPECTED_STEPS
    )
    checks = {
        "known_case": summary.get("case_id") in COLLECTION_CASE_IDS,
        "r6_candidate_exact": summary.get("candidate_tree_sha256")
        == LOCKED_INPUTS["r6_candidate"]["tree_sha256"],
        "length_and_end_valid": recoverable_end or complete_end,
        "teacher_absent": _zero_int(summary.get("teacher_query_count"))
        and summary.get("teacher_enabled") is False,
        "pure_action": summary.get("blending_enabled") is False
        and summary.get("safety_latch_enabled") is False,
        "trace_complete": summary.get("trace_step_count") == steps,
        "replay_complete": summary.get("replay_step_count") == steps
        and summary.get("replay_boundary_count") == steps + 1,
        "raw_sensor_count": summary.get("raw_sensor_sample_count")
        == steps * RAW_SAMPLES_PER_STEP,
        "v26_integrity": all(
            _zero_int(summary.get(name))
            for name in (
                "fallback_count",
                "hard_invalid_count",
                "duplicate_event_count",
                "out_of_order_event_count",
                "left_non_v26_source_count",
            )
        ),
        "runtime_integrity": all(
            _zero_int(summary.get(name))
            for name in (
                "action_clipped_values",
                "nonfinite_count",
                "sea_plugin_fallback_count",
                "so_solver_unaccepted_count",
            )
        ),
    }
    passed = all(checks.values())
    return {
        "status": "PASS_H0_V12R7_COLLECTION_INTEGRITY"
        if passed
        else "FAIL_H0_V12R7_COLLECTION_INTEGRITY",
        "passed": passed,
        "recoverable_prefix": bool(recoverable_end),
        "checks": checks,
    }


def label_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    rows = summary.get("labelled_row_count")
    checks = {
        "collection_passed": summary.get("collection_integrity_passed") is True,
        "case_known": summary.get("case_id") in COLLECTION_CASE_IDS,
        "rows_positive": _strict_int(rows, minimum=MINIMUM_RECOVERABLE_PREFIX_STEPS),
        "same_state_complete": summary.get("same_state_teacher_label_count") == rows,
        "teacher_queries_exact": summary.get("teacher_query_count") == rows,
        "no_environment_access": _zero_int(summary.get("environment_reset_calls"))
        and _zero_int(summary.get("environment_step_calls")),
        "shape_exact": summary.get("observation_shape") == [rows, 35]
        and summary.get("action_shape") == [rows, 2],
        "finite": _zero_int(summary.get("nonfinite_count")),
    }
    passed = all(checks.values())
    return {
        "status": "PASS_H0_V12R7_OBSERVER_LABELS"
        if passed
        else "FAIL_H0_V12R7_OBSERVER_LABELS",
        "passed": passed,
        "checks": checks,
    }


def _metric_pass(record: Any, threshold: Mapping[str, float]) -> bool:
    return isinstance(record, Mapping) and all(
        _finite(record.get(name)) and float(record[name]) <= limit
        for name, limit in threshold.items()
    )


def fit_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    per_case = summary.get("per_case_metrics")
    observer = summary.get("observer_case_metrics")
    checks = {
        "single_fit": type(summary.get("actor_fit_count")) is int
        and summary.get("actor_fit_count") == 1
        and type(summary.get("actor_updates")) is int
        and summary.get("actor_updates") == 1,
        "no_critic_or_ppo": _zero_int(summary.get("critic_updates"))
        and _zero_int(summary.get("ppo_updates")),
        "architecture": summary.get("hidden_dims") == [512, 512]
        and summary.get("actor_feature_count") == 35,
        "logstd_exact": summary.get("logstd_byte_exact") is True,
        "clock_zero": summary.get("disabled_clock_columns_bit_zero") is True,
        "save_reload_exact": summary.get("save_reload_exact") is True,
        "global": _metric_pass(
            summary.get("global_metrics"), OFFLINE_THRESHOLDS["global"]
        ),
        "reset": _finite(summary.get("reset_max_abs_error"))
        and float(summary["reset_max_abs_error"])
        <= OFFLINE_THRESHOLDS["reset_max_abs_error"],
        "per_case": isinstance(per_case, Mapping)
        and set(per_case) == set(COLLECTION_CASE_IDS)
        and all(
            _metric_pass(per_case[case_id], OFFLINE_THRESHOLDS["per_case"])
            for case_id in COLLECTION_CASE_IDS
        ),
        "r4_recovery": _metric_pass(
            summary.get("r4_failed_plus_metrics"), OFFLINE_THRESHOLDS["r4_failed_plus"]
        ),
        "observer_recovery": isinstance(observer, Mapping)
        and set(observer) == set(COLLECTION_CASE_IDS)
        and all(
            _metric_pass(observer[case_id], OFFLINE_THRESHOLDS["observer_case"])
            for case_id in COLLECTION_CASE_IDS
        ),
        "observer_plus_late": _metric_pass(
            summary.get("observer_plus_late_metrics"),
            OFFLINE_THRESHOLDS["observer_case"],
        ),
    }
    passed = all(checks.values())
    return {
        "status": "PASS_H0_V12R7_RECOVERY_FIT"
        if passed
        else "FAIL_H0_V12R7_RECOVERY_FIT",
        "passed": passed,
        "checks": checks,
    }


def development_gate(summary: Mapping[str, Any], *, case_id: str) -> dict[str, Any]:
    if case_id not in DEVELOPMENT_CASE_IDS:
        raise ValueError(f"unknown V12R7 development case: {case_id!r}")
    checks = {
        "case_exact": summary.get("case_id") == case_id,
        "full_horizon": summary.get("steps") == EXPECTED_STEPS
        and summary.get("terminated") is False
        and summary.get("truncated") is True
        and summary.get("end_reason") == "episode_time_limit",
        "cycles": _strict_int(
            summary.get("phase_valid_cycle_count"), minimum=MINIMUM_VALID_CYCLES
        ),
        "penetration": _finite(summary.get("grf_penetration_max_m"))
        and float(summary["grf_penetration_max_m"]) < PENETRATION_LIMIT_M,
        "pure_policy": summary.get("teacher_enabled") is False
        and _zero_int(summary.get("teacher_query_count"))
        and summary.get("blending_enabled") is False
        and summary.get("safety_latch_enabled") is False,
        "no_clipping_or_nonfinite": _zero_int(summary.get("action_clipped_values"))
        and _zero_int(summary.get("nonfinite_count")),
        "runtime_integrity": all(
            _zero_int(summary.get(name))
            for name in (
                "fallback_count",
                "hard_invalid_count",
                "duplicate_event_count",
                "out_of_order_event_count",
                "left_non_v26_source_count",
                "sea_plugin_fallback_count",
                "so_solver_unaccepted_count",
            )
        ),
    }
    passed = all(checks.values())
    return {
        "status": "PASS_H0_V12R7_PURE_DEVELOPMENT"
        if passed
        else "FAIL_H0_V12R7_PURE_DEVELOPMENT",
        "passed": passed,
        "case_id": case_id,
        "checks": checks,
    }


def contract_self_check() -> dict[str, Any]:
    checks = {
        "new_namespace": ROOT != v12r6.VALIDATION_ROOT,
        "six_collection_cases": len(COLLECTION_CASE_IDS) == 6
        and len(set(COLLECTION_CASE_IDS)) == 6,
        "critical_first": COLLECTION_CASE_IDS[0] == "deterministic_offset_plus_0p20"
        and DEVELOPMENT_CASE_IDS[0] == "deterministic_offset_plus_0p20",
        "r6_terminal_bound": LOCKED_INPUTS["r6_terminal_ledger"]["sha256"]
        == "ce67aea83b1f98aa251ad130af1da25982435381c0aeddeafa0a56bd3274e340",
        "ordinary_512_actor": FIT["architecture"]["hidden_dims"] == [512, 512]
        and FIT["architecture"]["residual_actor"] is False,
        "one_fit": FIT["actor_fit_count"] == 1,
        "no_runtime_teacher": all(
            case["behavior"] == "R7_PURE_UNBLENDED_NO_TEACHER_NO_LATCH"
            for case in DEVELOPMENT_CASES
        ),
        "no_retry_or_alpha_sweep": AUTHORITY["retry_authorized"] is False
        and AUTHORITY["alpha_sweep_authorized"] is False,
        "v26_unchanged": EVENT_CONTRACT_ID == v12r6.EVENT_CONTRACT_ID,
        "strict_penetration": PENETRATION_LIMIT_M == 0.025,
        "q3_and_zero_deferred": AUTHORITY["qualification_execution_authorized"] is False
        and AUTHORITY["checkpoint_zero_authorized"] is False,
    }
    return {
        "status": "PASS_H0_V12R7_RECOVERY_CONTRACT"
        if all(checks.values())
        else "FAIL_H0_V12R7_RECOVERY_CONTRACT",
        "passed": all(checks.values()),
        "checks": checks,
    }


__all__ = [name for name in globals() if name.isupper()] + [
    "candidate_id",
    "canonical_collection_case",
    "canonical_development_case",
    "collection_case_root",
    "collection_integrity_gate",
    "contract_self_check",
    "development_gate",
    "fit_gate",
    "label_gate",
    "observer_label_path",
]
