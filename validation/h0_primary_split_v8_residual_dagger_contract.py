"""Pure contract for the one-round V8R1P1/V26 residual-DAgger pipeline.

V8R1P1 is the fresh development lineage after the procedural V8 and V8R1
teacher replays closed before collection.  It consumes only the six gated V8R1P1
teacher-replay traces and the heel-qualified V26 event contract.  Exactly one
interim residual fit, two *full* 500-step DAgger rollouts, one final residual
fit, one candidate freeze, and six full development rollouts are authorized.
There is deliberately no partial-FSM-rejection success path.
"""

from __future__ import annotations

import copy
import math
from pathlib import PurePosixPath
from typing import Any, Mapping


SCHEMA_VERSION = 8
REVISION = "2026-08-07"
PROTOCOL_ID = "AB06_H0_PRIMARY_SPLIT_V8R1P1_V26_RESIDUAL_DAGGER"
PIPELINE_ID = "H0_V8R1P1_V26_RESIDUAL_ONE_ROUND_DAGGER"

PIPELINE_CLAIM_STATUS = "H0_PRIMARY_SPLIT_V8R1P1_PIPELINE_EXECUTION_CLAIMED"
WORKER_CLAIM_STATUS = "H0_PRIMARY_SPLIT_V8R1P1_PIPELINE_WORKER_CLAIMED"
PIPELINE_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V8R1P1_RESIDUAL_DAGGER_PIPELINE"
PIPELINE_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V8R1P1_RESIDUAL_DAGGER_PIPELINE"
P0_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V8R1P1_RESIDUAL_P0_INTERIM"
P0_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V8R1P1_RESIDUAL_P0_INTERIM"
DAGGER_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V8R1P1_DAGGER_COLLECTION"
DAGGER_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V8R1P1_DAGGER_COLLECTION"
P1_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V8R1P1_RESIDUAL_P1_FINAL_FIT"
P1_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V8R1P1_RESIDUAL_P1_FINAL_FIT"
CANDIDATE_FREEZE_STATUS = "H0_PRIMARY_SPLIT_V8R1P1_CANDIDATE_FROZEN"
DEVELOPMENT_ROLLOUT_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V8R1P1_DEVELOPMENT_ROLLOUT"
DEVELOPMENT_ROLLOUT_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V8R1P1_DEVELOPMENT_ROLLOUT"
DEVELOPMENT_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V8R1P1_DEVELOPMENT"
DEVELOPMENT_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V8R1P1_DEVELOPMENT"

SOURCE_H0_ID = "H0_MARKOV35_PHASE_ALIGNED_SIGMA0005_ITER1_RETRY"
TARGET_CONTRACT_ID = (
    "primary_grf_split_v1+binary_point_v25+heel_qualified_fsm_v2"
)
EVENT_CONTRACT_ID = "binary_point_v25+heel_qualified_fsm_v2"
SOURCE_EVENT_CONTRACT_ID = "legacy_events_v1"
SO_POLICY_ID = "verified_status0_max_iter_v1"

EXPECTED_STEPS = 500
EXPECTED_ACTOR_FEATURES = 35
EXPECTED_FULL_FEATURES = 84
EXPECTED_ACTION_DIM = 2
EXPECTED_DTYPE = "float32"
EXPECTED_CONTROL_WINDOWS = 5000
EXPECTED_RAW_SENSOR_SAMPLES = 5000
EXPECTED_POLICY_DT_S = 0.010
EXPECTED_SAMPLE_DT_S = 0.001
EXPECTED_SIGMA = 0.005
MORPHOLOGY_WEIGHT = 0.0
PENETRATION_LIMIT_M = 0.025
MINIMUM_VALID_CYCLES = 2

# V8 accepts no shortened DAgger corpus.  Both authorized cases must reach the
# normal 500-step episode boundary.
MIN_DAGGER_SAMPLES_PER_CASE = EXPECTED_STEPS

RESIDUAL_INPUT_INDICES = tuple(range(2, 35))
RESIDUAL_INPUT_COUNT = 33
RESIDUAL_HIDDEN_DIMS = (128, 128)
RESIDUAL_LIMITS = (0.175, 0.12)
NORMALIZATION_STD_FLOOR = 1.0e-4
FIT_SEED = 20260807
WEIGHT_DECAY = 1.0e-6
GRAD_CLIP_NORM = 5.0
RESET_ROW_WEIGHT = 100.0

P0_FIT = {
    "stage": "p0",
    "seed": FIT_SEED,
    "epochs": 8000,
    "full_batch": True,
    "optimizer": "AdamW",
    "weight_decay": WEIGHT_DECAY,
    "gradient_clip_norm": GRAD_CLIP_NORM,
    "reset_row_weight": RESET_ROW_WEIGHT,
    "normalization_columns_half_open": [2, 35],
    "normalization_std_floor": NORMALIZATION_STD_FLOOR,
    "learning_rate_schedule": [
        {"first_epoch": 1, "last_epoch": 2000, "learning_rate": 1.0e-3},
        {"first_epoch": 2001, "last_epoch": 5000, "learning_rate": 3.0e-4},
        {"first_epoch": 5001, "last_epoch": 8000, "learning_rate": 1.0e-4},
    ],
    "initial_checkpoint": "FROZEN_SOURCE_H0",
    "promotable": False,
}
P1_FIT = {
    "stage": "p1",
    "seed": FIT_SEED,
    "epochs": 4000,
    "full_batch": True,
    "optimizer": "AdamW",
    "weight_decay": WEIGHT_DECAY,
    "gradient_clip_norm": GRAD_CLIP_NORM,
    "reset_row_weight": RESET_ROW_WEIGHT,
    "normalization": "FROZEN_FROM_P0_TEACHER_CORPUS",
    "learning_rate_schedule": [
        {"first_epoch": 1, "last_epoch": 2000, "learning_rate": 3.0e-4},
        {"first_epoch": 2001, "last_epoch": 4000, "learning_rate": 1.0e-4},
    ],
    "initial_checkpoint": "P0_INTERIM",
    "promotable": True,
}

OFFLINE_THRESHOLDS = {
    "rmse_max": 0.0015,
    "max_abs_error_max": 0.015,
    "reset_max_abs_error_max": 1.0e-5,
}

RUN_ROOT = PurePosixPath(
    "validation/h0_primary_grf_split_adaptation_runs/"
    "2026-08-07_h0_primary_split_v8r1p1_v26_residual"
)
TEACHER_REPLAY_ROOT = RUN_ROOT / "teacher_replay"
ADAPTATION_ROOT = RUN_ROOT / "adaptation"
P0_ROOT = ADAPTATION_ROOT / "p0"
P0_MODULE_PATH = P0_ROOT / "rl_module_target_v26_residual_p0"
P0_RECEIPT_PATH = P0_ROOT / "receipt.json"
P1_ROOT = ADAPTATION_ROOT / "p1"
P1_MODULE_PATH = ADAPTATION_ROOT / "rl_module_target_v26_residual_p1"
P1_RECEIPT_PATH = P1_ROOT / "receipt.json"
CANDIDATE_FREEZE_PATH = ADAPTATION_ROOT / "candidate_freeze.json"
DAGGER_ROOT = RUN_ROOT / "dagger_round_1"
DEVELOPMENT_ROOT = RUN_ROOT / "development"
DEVELOPMENT_RECEIPT_PATH = DEVELOPMENT_ROOT / "receipt.json"
PIPELINE_CLAIM_PATH = RUN_ROOT / "pipeline_execution_claim.json"
PIPELINE_LEDGER_PATH = RUN_ROOT / "pipeline_execution_ledger.json"
WORKER_CLAIMS_ROOT = RUN_ROOT / "pipeline_worker_claims"

SOURCE_H0_MODULE_PATH = PurePosixPath(
    "validation/critic_warmup/"
    "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/rl_module_last"
)
TEACHER_REPLAY_LEDGER_PATH = TEACHER_REPLAY_ROOT / "execution_ledger.json"

_CASE_ROWS = (
    ("deterministic_offset_minus_0p20", "deterministic", 1.756870983805102, None, 123),
    ("deterministic_offset_nominal", "deterministic", 1.956870983805102, None, 123),
    ("deterministic_offset_plus_0p20", "deterministic", 2.156870983805102, None, 123),
    ("stochastic_nominal_seed_126", "stochastic", 1.956870983805102, 126, 126),
    ("stochastic_nominal_seed_127", "stochastic", 1.956870983805102, 127, 127),
    ("stochastic_nominal_seed_128", "stochastic", 1.956870983805102, 128, 128),
)

DEVELOPMENT_CASES = tuple(
    {
        "case_id": case_id,
        "action_selection": selection,
        "episode_start_offset_s": offset,
        "action_seed": action_seed,
        "runtime_seed": runtime_seed,
        "sigma": EXPECTED_SIGMA if selection == "stochastic" else 0.0,
        "teacher_trace": (TEACHER_REPLAY_ROOT / case_id / "trace.json").as_posix(),
        "teacher_receipt": (TEACHER_REPLAY_ROOT / case_id / "receipt.json").as_posix(),
        "destination": (DEVELOPMENT_ROOT / case_id).as_posix(),
    }
    for case_id, selection, offset, action_seed, runtime_seed in _CASE_ROWS
)
DEVELOPMENT_CASE_IDS = tuple(case["case_id"] for case in DEVELOPMENT_CASES)
DAGGER_CASE_IDS = (
    "deterministic_offset_minus_0p20",
    "stochastic_nominal_seed_126",
)
DAGGER_CASES = tuple(
    {
        **copy.deepcopy(
            next(row for row in DEVELOPMENT_CASES if row["case_id"] == case_id)
        ),
        "destination": (DAGGER_ROOT / case_id).as_posix(),
    }
    for case_id in DAGGER_CASE_IDS
)

STAGE_IDS = (
    "fit_p0",
    *(f"collect_dagger__{case_id}" for case_id in DAGGER_CASE_IDS),
    "fit_p1",
    "freeze_p1",
    *(f"develop__{case_id}" for case_id in DEVELOPMENT_CASE_IDS),
    "finalize_development",
)

AUTHORITY = {
    "fresh_v8r1p1_lineage_authorized": True,
    "heel_qualified_v26_required": True,
    "one_p0_fit_authorized": True,
    "one_dagger_round_authorized": True,
    "dagger_case_ids": list(DAGGER_CASE_IDS),
    "full_500_step_dagger_required": True,
    "partial_fsm_rejection_acceptance_authorized": False,
    "one_p1_continuation_authorized": True,
    "one_p1_freeze_authorized": True,
    "development_case_ids": list(DEVELOPMENT_CASE_IDS),
    "supervisor_only_sequential_workers_required": True,
    "retry_authorized": False,
    "second_dagger_round_authorized": False,
    "retuning_authorized": False,
    "critic_updates_authorized": False,
    "ppo_updates_authorized": False,
    "protected_trial_access_authorized": False,
    "reserve_trial_access_authorized": False,
    "runtime_promotion_authorized": False,
    "primary_grf_modification_authorized": False,
    "detector_retuning_authorized": False,
    "sea_semantic_modification_authorized": False,
}


def canonical_case(case_id: str, *, dagger: bool = False) -> dict[str, Any]:
    rows = DAGGER_CASES if dagger else DEVELOPMENT_CASES
    matches = [row for row in rows if row["case_id"] == case_id]
    if len(matches) != 1:
        raise ValueError(f"unknown V8R1P1 {'DAgger' if dagger else 'development'} case: {case_id!r}")
    return copy.deepcopy(matches[0])


def worker_claim_path(stage_id: str) -> PurePosixPath:
    try:
        index = STAGE_IDS.index(stage_id)
    except ValueError as exc:
        raise ValueError(f"unknown V8R1P1 pipeline stage: {stage_id!r}") from exc
    return WORKER_CLAIMS_ROOT / f"{index + 1:02d}_{stage_id}.json"


def stage_receipt_path(stage_id: str) -> PurePosixPath:
    if stage_id == "fit_p0":
        return P0_RECEIPT_PATH
    if stage_id.startswith("collect_dagger__"):
        case_id = stage_id.removeprefix("collect_dagger__")
        canonical_case(case_id, dagger=True)
        return DAGGER_ROOT / case_id / "receipt.json"
    if stage_id == "fit_p1":
        return P1_RECEIPT_PATH
    if stage_id == "freeze_p1":
        return CANDIDATE_FREEZE_PATH
    if stage_id.startswith("develop__"):
        case_id = stage_id.removeprefix("develop__")
        canonical_case(case_id)
        return DEVELOPMENT_ROOT / case_id / "receipt.json"
    if stage_id == "finalize_development":
        return DEVELOPMENT_RECEIPT_PATH
    raise ValueError(f"unknown V8R1P1 pipeline stage: {stage_id!r}")


def learning_rate(stage: str, epoch: int) -> float:
    fit = P0_FIT if stage == "p0" else P1_FIT if stage == "p1" else None
    if fit is None or type(epoch) is not int or not 1 <= epoch <= int(fit["epochs"]):
        raise ValueError(f"invalid {stage!r} epoch: {epoch!r}")
    for row in fit["learning_rate_schedule"]:
        if int(row["first_epoch"]) <= epoch <= int(row["last_epoch"]):
            return float(row["learning_rate"])
    raise RuntimeError("learning-rate schedule is incomplete")


def _finite_number(value: Any) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def fit_gate(summary: Mapping[str, Any], *, stage: str) -> dict[str, Any]:
    if stage not in {"p0", "p1"}:
        raise ValueError(f"unknown fit stage: {stage!r}")
    expected_fit = P0_FIT if stage == "p0" else P1_FIT
    teacher_samples = 6 * EXPECTED_STEPS
    expected_dagger = 0 if stage == "p0" else len(DAGGER_CASE_IDS) * EXPECTED_STEPS
    metrics = summary.get("metrics")
    checks = {
        "schema": summary.get("schema_version") == SCHEMA_VERSION,
        "protocol": summary.get("protocol_id") == PROTOCOL_ID,
        "stage": summary.get("fit_stage") == stage,
        "fit_exact": summary.get("fit") == expected_fit,
        "sample_count": summary.get("sample_count") == teacher_samples + expected_dagger,
        "teacher_sample_count": summary.get("teacher_sample_count") == teacher_samples,
        "dagger_sample_count": summary.get("dagger_sample_count") == expected_dagger,
        "reset_row_count": summary.get("reset_row_count") == (6 if stage == "p0" else 8),
        "metrics_object": isinstance(metrics, Mapping),
        "rmse": isinstance(metrics, Mapping)
        and _finite_number(metrics.get("rmse"))
        and float(metrics["rmse"]) <= OFFLINE_THRESHOLDS["rmse_max"],
        "max_abs": isinstance(metrics, Mapping)
        and _finite_number(metrics.get("max_abs_error"))
        and float(metrics["max_abs_error"]) <= OFFLINE_THRESHOLDS["max_abs_error_max"],
        "reset_max_abs": isinstance(metrics, Mapping)
        and _finite_number(metrics.get("reset_max_abs_error"))
        and float(metrics["reset_max_abs_error"])
        <= OFFLINE_THRESHOLDS["reset_max_abs_error_max"],
        "all_finite": summary.get("all_finite") is True,
        "base_h0_byte_exact": summary.get("base_h0_byte_exact") is True,
        "critic_byte_exact": summary.get("critic_byte_exact") is True,
        "logstd_byte_exact": summary.get("logstd_byte_exact") is True,
        "limits": summary.get("residual_limits") == list(RESIDUAL_LIMITS),
        "input_indices": summary.get("residual_input_indices") == list(RESIDUAL_INPUT_INDICES),
        "normalization_frozen": summary.get("normalization_frozen") is True,
        "optimizer_steps": summary.get("optimizer_steps") == expected_fit["epochs"],
        "actor_updates_exact": summary.get("actor_updates") == 1,
        "zero_critic_updates": summary.get("critic_updates") == 0,
        "zero_ppo_updates": summary.get("ppo_updates") == 0,
        "protected_closed": summary.get("protected_trials_opened") == [],
        "p0_nonpromotable": stage != "p0" or summary.get("promotable") is False,
        "p1_final": stage != "p1" or summary.get("promotable") is True,
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": (
            P0_PASS_STATUS if stage == "p0" and passed else
            P0_FAIL_STATUS if stage == "p0" else
            P1_PASS_STATUS if passed else P1_FAIL_STATUS
        ),
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "fit_stage": stage,
        "checks": checks,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }


def dagger_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    """Accept only a complete, normal 500-step V26 rollout."""

    case_id = summary.get("case_id")
    try:
        canonical_case(str(case_id), dagger=True)
        known = True
    except ValueError:
        known = False
    zero_fields = (
        "nonfinite_count",
        "action_clipped_values",
        "timeout_count",
        "invalid_event_count",
        "hard_invalid_count",
        "so_solver_unaccepted_count",
        "sea_plugin_fallback_count",
        "routing_failure_count",
        "served_action_teacher_dependency_count",
    )
    checks = {
        "schema": summary.get("schema_version") == SCHEMA_VERSION,
        "protocol": summary.get("protocol_id") == PROTOCOL_ID,
        "known_case": known,
        "round_exact": summary.get("dagger_round") == 1,
        "p0_behavior": summary.get("behavior") == "P0_CLOSED_LOOP_V25_BINARY_ACTIVE",
        "counterfactual_teacher": summary.get("teacher")
        == "H0_ONLINE_GRF_DETECTOR_LEGACY_EVENTS_SHADOW_FSM",
        "full_episode": summary.get("completion_mode") == "episode_time_limit"
        and summary.get("sample_count") == EXPECTED_STEPS
        and summary.get("persisted_sample_count") == EXPECTED_STEPS,
        "no_partial_rejection": summary.get("fsm_event_rejection") is None,
        "audited_windows": summary.get("control_window_count") == EXPECTED_CONTROL_WINDOWS,
        "raw_samples": summary.get("v25_raw_sensor_sample_count") == EXPECTED_RAW_SENSOR_SAMPLES,
        "finite_and_clean": all(summary.get(field) == 0 for field in zero_fields),
        "penetration": _finite_number(summary.get("grf_penetration_max_m"))
        and float(summary["grf_penetration_max_m"]) < PENETRATION_LIMIT_M,
        "normal_terminal": summary.get("terminated") is False
        and summary.get("truncated") is True
        and summary.get("end_reason") == "episode_time_limit",
        "cycles": type(summary.get("phase_valid_cycle_count")) is int
        and summary["phase_valid_cycle_count"] >= MINIMUM_VALID_CYCLES,
        "causal_order": summary.get("candidate_selected_before_teacher_count") == EXPECTED_STEPS,
        "binary_active": summary.get("binary_phase_fsm_mode") == "binary_active",
        "event_contract": summary.get("event_contract_id") == EVENT_CONTRACT_ID,
        "morphology_zero": summary.get("morphology_weight") == MORPHOLOGY_WEIGHT,
        "no_second_round": summary.get("dagger_rounds_completed") == 1,
        "zero_critic_updates": summary.get("critic_updates") == 0,
        "zero_ppo_updates": summary.get("ppo_updates") == 0,
        "protected_closed": summary.get("protected_trials_opened") == [],
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": DAGGER_PASS_STATUS if passed else DAGGER_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "case_id": case_id,
        "checks": checks,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }


def development_rollout_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    case_id = summary.get("case_id")
    try:
        expected = canonical_case(str(case_id))
    except ValueError:
        expected = None
    zero_fields = (
        "action_clipped_values",
        "fallback_count",
        "timeout_count",
        "safety_stop_count",
        "sea_plugin_fallback_count",
        "so_solver_unaccepted_count",
        "hard_invalid_count",
        "invalid_event_count",
        "nonfinite_count",
    )
    checks = {
        "schema": summary.get("schema_version") == SCHEMA_VERSION,
        "protocol": summary.get("protocol_id") == PROTOCOL_ID,
        "known_case": expected is not None,
        "condition_exact": expected is not None
        and summary.get("action_selection") == expected["action_selection"]
        and summary.get("episode_start_offset_s") == expected["episode_start_offset_s"]
        and summary.get("action_seed") == expected["action_seed"]
        and summary.get("runtime_seed") == expected["runtime_seed"],
        "steps": summary.get("steps") == EXPECTED_STEPS,
        "control_windows": summary.get("control_window_count") == EXPECTED_CONTROL_WINDOWS,
        "raw_samples": summary.get("v25_raw_sensor_sample_count") == EXPECTED_RAW_SENSOR_SAMPLES,
        "episode_time_limit": summary.get("end_reason") == "episode_time_limit",
        "not_terminated": summary.get("terminated") is False,
        "truncated": summary.get("truncated") is True,
        "cycles": type(summary.get("phase_valid_cycle_count")) is int
        and summary["phase_valid_cycle_count"] >= MINIMUM_VALID_CYCLES,
        "penetration": _finite_number(summary.get("grf_penetration_max_m"))
        and float(summary["grf_penetration_max_m"]) < PENETRATION_LIMIT_M,
        "zero_invalids": all(type(summary.get(field)) is int and summary[field] == 0 for field in zero_fields),
        "layout": summary.get("n_actor") == EXPECTED_ACTOR_FEATURES
        and summary.get("n_observation") == EXPECTED_FULL_FEATURES
        and summary.get("observation_dtype") == EXPECTED_DTYPE,
        "binary_active": summary.get("binary_phase_fsm_mode") == "binary_active",
        "event_contract": summary.get("event_contract_id") == EVENT_CONTRACT_ID,
        "morphology_zero": summary.get("morphology_weight") == MORPHOLOGY_WEIGHT,
        "candidate_id": isinstance(summary.get("candidate_id"), str)
        and summary["candidate_id"].startswith("H0_PRIMARY_SPLIT_V8R1P1_P1_"),
        "zero_actor_updates": summary.get("actor_updates") == 0,
        "zero_critic_updates": summary.get("critic_updates") == 0,
        "zero_ppo_updates": summary.get("ppo_updates") == 0,
        "protected_closed": summary.get("protected_trials_opened") == [],
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": DEVELOPMENT_ROLLOUT_PASS_STATUS if passed else DEVELOPMENT_ROLLOUT_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "case_id": case_id,
        "candidate_id": summary.get("candidate_id"),
        "checks": checks,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }


__all__ = [name for name in globals() if name.isupper()] + [
    "canonical_case",
    "dagger_gate",
    "development_rollout_gate",
    "fit_gate",
    "learning_rate",
    "stage_receipt_path",
    "worker_claim_path",
]
