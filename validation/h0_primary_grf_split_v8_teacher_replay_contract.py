"""Pure V8 contract for V26-active H0 teacher-action replay.

V8 is a fresh development lineage authorized after the terminal V7
diagnostic.  It reuses the six already-public V5 baseline action tapes, but
executes them through the frozen V25 geometry and the heel-qualified V26 FSM.
No protected trial, candidate update, optimizer, critic, or PPO operation is
part of this collection contract.

This module intentionally performs no filesystem, simulation, Torch, or RLlib
work.  The 35/84 layout constants are imported from the historical V6 pure
contract so that duplicating those long immutable name lists cannot introduce
an accidental observation-layout fork.
"""

from __future__ import annotations

import copy
import math
from pathlib import PurePosixPath
from typing import Any, Mapping

from validation import h0_primary_grf_split_v6_teacher_replay_contract as v6


SCHEMA_VERSION = 8
REVISION = "2026-08-07"
PROTOCOL_ID = "AB06_H0_PRIMARY_SPLIT_V8_V26_RESIDUAL_DAGGER"
COLLECTOR_ID = "H0_V8_V26_ACTIVE_TEACHER_ACTION_REPLAY"
LOCK_STATUS = "H0_PRIMARY_SPLIT_V8_V26_TEACHER_REPLAY_FROZEN"
PREFLIGHT_STATUS = "PASS_H0_PRIMARY_SPLIT_V8_V26_TEACHER_REPLAY_PREFLIGHT"
ROLLOUT_COLLECTED_STATUS = "H0_V8_V26_TEACHER_REPLAY_COLLECTED_UNGATED"
ROLLOUT_PASS_STATUS = "PASS_H0_V8_V26_TEACHER_REPLAY"
ROLLOUT_FAIL_STATUS = "FAIL_H0_V8_V26_TEACHER_REPLAY"
PROTOCOL_PASS_STATUS = "PASS_H0_V8_V26_TEACHER_REPLAY_DEVELOPMENT"
PROTOCOL_FAIL_STATUS = "FAIL_H0_V8_V26_TEACHER_REPLAY_DEVELOPMENT"
SOURCE_H0_ID = v6.SOURCE_H0_ID
SOURCE_OBSERVATION_CONTRACT_ID = v6.SOURCE_OBSERVATION_CONTRACT_ID
V26_ACTIVE_EVENT_CONTRACT_ID = "binary_point_v25+heel_qualified_fsm_v2"
TARGET_OBSERVATION_CONTRACT_ID = (
    "primary_grf_split_v1+binary_point_v25+heel_qualified_fsm_v2"
)
V26_FSM_SOURCE = "binary_phase_fsm_v26"
V26_ACTOR_ADAPTER_SOURCE = "v25_fsm_v26"
SO_POLICY_ID = v6.SO_POLICY_ID

EXPECTED_STEPS = v6.EXPECTED_STEPS
EXPECTED_CONTROL_WINDOWS = v6.EXPECTED_CONTROL_WINDOWS
EXPECTED_RAW_SENSOR_SAMPLES = v6.EXPECTED_RAW_SENSOR_SAMPLES
EXPECTED_SAMPLES_PER_STEP = v6.EXPECTED_SAMPLES_PER_STEP
EXPECTED_ACTOR_FEATURES = v6.EXPECTED_ACTOR_FEATURES
EXPECTED_FULL_FEATURES = v6.EXPECTED_FULL_FEATURES
EXPECTED_ACTION_DIM = v6.EXPECTED_ACTION_DIM
EXPECTED_OBSERVATION_DTYPE = v6.EXPECTED_OBSERVATION_DTYPE
EXPECTED_SAMPLE_DT_S = v6.EXPECTED_SAMPLE_DT_S
EXPECTED_POLICY_DT_S = v6.EXPECTED_POLICY_DT_S
EXPECTED_EPISODE_DURATION_S = v6.EXPECTED_EPISODE_DURATION_S
STOCHASTIC_SIGMA = v6.STOCHASTIC_SIGMA
MORPHOLOGY_WEIGHT = v6.MORPHOLOGY_WEIGHT
PENETRATION_LIMIT_M = v6.PENETRATION_LIMIT_M
MINIMUM_VALID_CYCLES = v6.MINIMUM_VALID_CYCLES
INVARIANT_COLUMN_RANGES = v6.INVARIANT_COLUMN_RANGES
INVARIANT_COLUMNS = v6.INVARIANT_COLUMNS
EXPECTED_ACTOR_FEATURE_NAMES = v6.EXPECTED_ACTOR_FEATURE_NAMES
EXPECTED_OBSERVATION_FEATURE_NAMES = v6.EXPECTED_OBSERVATION_FEATURE_NAMES

RUN_ROOT = PurePosixPath(
    "validation/h0_primary_grf_split_adaptation_runs/"
    "2026-08-07_h0_primary_split_v8_v26_residual/teacher_replay"
)
LOCK_PATH = PurePosixPath(
    "validation/h0_primary_grf_split_v8_teacher_replay_execution_lock.json"
)
PREFLIGHT_RECEIPT_PATH = PurePosixPath(
    "validation/h0_primary_grf_split_v8_teacher_replay_preflight_receipt.json"
)
EXECUTION_LEDGER_PATH = RUN_ROOT / "execution_ledger.json"
EXECUTION_CLAIM_PATH = RUN_ROOT / "execution_claim.json"
WORKER_CLAIMS_ROOT = RUN_ROOT / "worker_claims"

V5_RUN_ROOT = v6.V5_RUN_ROOT
V5_BASELINE_ROOT = v6.V5_BASELINE_ROOT

CASES = tuple(
    {
        **dict(case),
        "destination": (RUN_ROOT / str(case["case_id"])).as_posix(),
    }
    for case in v6.CASES
)
CASE_IDS = tuple(str(case["case_id"]) for case in CASES)

SOURCE_RELATIVE_PATHS = {
    "contract": "validation/h0_primary_grf_split_v8_teacher_replay_contract.py",
    "collector": "validation/run_h0_primary_grf_split_v8_teacher_replay.py",
    "v6_collector_engine": (
        "validation/run_h0_primary_grf_split_v6_teacher_replay.py"
    ),
    "v6_pure_contract": (
        "validation/h0_primary_grf_split_v6_teacher_replay_contract.py"
    ),
    "v6_preflight_helpers": (
        "validation/build_h0_primary_grf_split_v6_teacher_replay_preflight.py"
    ),
    "forensic_writer": "validation/h0_forensic_rollout.py",
    "legacy_v25_runner": "validation/run_h0_v25_abc_preflight.py",
    "so_classifier": "validation/h0_v3_so_recovery_contract.py",
    "primary_split_contract": (
        "Trajectory Generator/baseline_MLP/primary_grf_split_adaptation.py"
    ),
    "residual_module": (
        "Trajectory Generator/baseline_MLP/primary_split_v25_residual.py"
    ),
    "environment": "Trajectory Generator/osim_trj_cmc_like.py",
    "v26_binary_phase_adapter": (
        "Trajectory Generator/binary_phase_adapter_v26.py"
    ),
    "v26_binary_phase_fsm": "Trajectory Generator/binary_phase_fsm_v26.py",
    "actor_phase_fsm": "Trajectory Generator/prosthetic_phase_fsm.py",
    "binary_phase_detector": "binary_phase_detector.py",
    "rollout_eval": "Trajectory Generator/baseline_MLP/rollout_eval.py",
    "training_config": "Trajectory Generator/baseline_MLP/training_config.py",
    "simulation_runner": "simulation_runner.py",
    "static_optimization": "static_optimization.py",
    "model_loader": "model_loader.py",
    "online_grf": "online_grf.py",
    "root_config": "config.py",
    "contract_tests": (
        "validation/test_h0_primary_grf_split_v8_teacher_replay_contract.py"
    ),
}

INPUT_RELATIVE_PATHS = {
    name: relative
    for name, relative in v6.INPUT_RELATIVE_PATHS.items()
    if name != "v25_corrected_active_protocol"
}
INPUT_RELATIVE_PATHS.update(
    {
        "v26_development_receipt": (
            "validation/binary_phase_fsm_v26_development_receipt.json"
        ),
        "v26_v7_replay_receipt": (
            "validation/binary_phase_fsm_v26_v7_replay_receipt.json"
        ),
    }
)

AUTHORITY = {
    "v8_protocol_authorized": True,
    "development_teacher_action_replay_authorized": True,
    "binary_active_v26_execution_authorized": True,
    "v5_pass_baselines_reclassified_as_development": True,
    "v7_terminal_history_preserved": True,
    "dataset_collection_authorized": True,
    "supervisor_only_sequential_execution_required": True,
    "residual_candidate_creation_authorized": False,
    "actor_updates_authorized": False,
    "critic_updates_authorized": False,
    "ppo_updates_authorized": False,
    "training_authorized": False,
    "protected_trial_access_authorized": False,
    "reserve_trial_access_authorized": False,
    "runtime_promotion_authorized": False,
    "primary_grf_modification_authorized": False,
    "detector_retuning_authorized": False,
    "sea_semantic_modification_authorized": False,
}


def canonical_case(case_id: str) -> dict[str, Any]:
    """Return a fresh copy of one exact V8 development replay case."""

    matches = [case for case in CASES if case["case_id"] == case_id]
    if len(matches) != 1:
        raise ValueError(f"unknown V8 teacher replay case: {case_id!r}")
    return copy.deepcopy(matches[0])


def _finite_number(value: Any) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def _nonnegative_int(value: Any) -> bool:
    return type(value) is int and value >= 0


def _binary_gate_passes(value: Any) -> bool:
    if not isinstance(value, Mapping):
        return False
    zero_fields = (
        "duplicate_event_count",
        "out_of_order_event_count",
        "left_non_v26_source_count",
        "fallback_count",
        "hard_invalid_count",
    )
    events = value.get("events")
    event_count = value.get("event_count")
    rows_valid = isinstance(events, list) and bool(events)
    expected_event: str | None = None
    previous_event_time = -math.inf
    if rows_valid:
        for index, event in enumerate(events):
            if not isinstance(event, Mapping):
                rows_valid = False
                break
            name = event.get("event")
            event_time = event.get("event_time_s")
            confirmed_time = event.get("confirmed_time_s")
            delivered_time = event.get("delivered_time_s")
            if expected_event is None:
                expected_event = str(name)
            if (
                name not in {"heel_strike", "toe_off"}
                or name != expected_event
                or event.get("source") != V26_FSM_SOURCE
                or event.get("event_contract_id") != V26_ACTIVE_EVENT_CONTRACT_ID
                or not all(
                    _finite_number(timestamp)
                    for timestamp in (event_time, confirmed_time, delivered_time)
                )
                or float(event_time) <= previous_event_time
                or abs(float(confirmed_time) - float(event_time) - 0.005) > 1.0e-9
                or float(delivered_time) < float(confirmed_time) - 1.0e-9
                or float(delivered_time) - float(confirmed_time) > 0.010 + 1.0e-9
                or (
                    index == 0
                    and name == "toe_off"
                    and event.get("startup_partial_stance") is not True
                )
            ):
                rows_valid = False
                break
            expected_event = "toe_off" if name == "heel_strike" else "heel_strike"
            previous_event_time = float(event_time)
    return bool(
        value.get("passed") is True
        and value.get("sample_count") == EXPECTED_RAW_SENSOR_SAMPLES
        and type(event_count) is int
        and event_count >= 2 * MINIMUM_VALID_CYCLES
        and isinstance(events, list)
        and len(events) == event_count
        and rows_valid
        and all(value.get(field) == 0 for field in zero_fields)
        and all(_nonnegative_int(value.get(field)) for field in zero_fields)
    )


def replay_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    """Classify one persisted V26-active replay summary fail-closed."""

    case_id = summary.get("case_id")
    try:
        expected_case = canonical_case(str(case_id))
    except ValueError:
        expected_case = None
    binary_gate = summary.get("binary_phase_event_gate")
    checks = {
        "schema_version": summary.get("schema_version") == SCHEMA_VERSION,
        "ungated_status": summary.get("status") == ROLLOUT_COLLECTED_STATUS,
        "protocol_id": summary.get("protocol_id") == PROTOCOL_ID,
        "collector_id": summary.get("collector_id") == COLLECTOR_ID,
        "known_case": expected_case is not None,
        "condition_exact": expected_case is not None
        and summary.get("action_selection") == expected_case["action_selection"]
        and summary.get("episode_start_offset_s")
        == expected_case["episode_start_offset_s"]
        and summary.get("action_seed") == expected_case["action_seed"]
        and summary.get("runtime_seed") == expected_case["runtime_seed"],
        "source_h0_exact": summary.get("source_h0_id") == SOURCE_H0_ID,
        "source_contract_exact": summary.get("source_observation_contract_id")
        == SOURCE_OBSERVATION_CONTRACT_ID,
        "target_contract_exact": summary.get("target_observation_contract_id")
        == TARGET_OBSERVATION_CONTRACT_ID,
        "behavior_exact": summary.get("behavior")
        == "FROZEN_V5_RAW_ACTION_REPLAY",
        "binary_active": summary.get("binary_phase_fsm_mode") == "binary_active",
        "binary_contract_exact": summary.get("binary_phase_event_contract_id")
        == V26_ACTIVE_EVENT_CONTRACT_ID,
        "morphology_zero": _finite_number(summary.get("morphology_weight"))
        and float(summary["morphology_weight"]) == MORPHOLOGY_WEIGHT,
        "steps_exact": summary.get("steps") == EXPECTED_STEPS,
        "control_windows_exact": summary.get("control_window_count")
        == EXPECTED_CONTROL_WINDOWS,
        "raw_samples_exact": summary.get("v25_raw_sensor_sample_count")
        == EXPECTED_RAW_SENSOR_SAMPLES,
        "episode_time_limit": summary.get("end_reason") == "episode_time_limit",
        "not_terminated": summary.get("terminated") is False,
        "truncated": summary.get("truncated") is True,
        "minimum_cycles": type(summary.get("phase_valid_cycle_count")) is int
        and summary["phase_valid_cycle_count"] >= MINIMUM_VALID_CYCLES,
        "penetration": _finite_number(summary.get("grf_penetration_max_m"))
        and float(summary["grf_penetration_max_m"]) < PENETRATION_LIMIT_M,
        "invariants_exact": _nonnegative_int(summary.get("invariant_mismatch_count"))
        and summary["invariant_mismatch_count"] == 0,
        "teacher_means_exact": _nonnegative_int(
            summary.get("teacher_mean_mismatch_count")
        )
        and summary["teacher_mean_mismatch_count"] == 0,
        "times_exact": _nonnegative_int(summary.get("time_mismatch_count"))
        and summary["time_mismatch_count"] == 0,
        "step_contract_exact": _nonnegative_int(
            summary.get("step_contract_failure_count")
        )
        and summary["step_contract_failure_count"] == 0,
        "no_clipping": _nonnegative_int(summary.get("action_clipped_values"))
        and summary["action_clipped_values"] == 0,
        "no_timeout": _nonnegative_int(summary.get("timeout_count"))
        and summary["timeout_count"] == 0,
        "no_safety_stop": _nonnegative_int(summary.get("safety_stop_count"))
        and summary["safety_stop_count"] == 0,
        "no_invalid_event": _nonnegative_int(summary.get("invalid_event_count"))
        and summary["invalid_event_count"] == 0,
        "no_hard_invalid": _nonnegative_int(summary.get("hard_invalid_count"))
        and summary["hard_invalid_count"] == 0,
        "finite": _nonnegative_int(summary.get("nonfinite_count"))
        and summary["nonfinite_count"] == 0,
        "no_unaccepted_so": _nonnegative_int(
            summary.get("so_solver_unaccepted_count")
        )
        and summary["so_solver_unaccepted_count"] == 0,
        "no_sea_fallback": _nonnegative_int(
            summary.get("sea_plugin_fallback_count")
        )
        and summary["sea_plugin_fallback_count"] == 0,
        "binary_event_gate": _binary_gate_passes(binary_gate),
        "binary_event_cycle_consistency": isinstance(binary_gate, Mapping)
        and type(binary_gate.get("event_count")) is int
        and type(summary.get("phase_valid_cycle_count")) is int
        and binary_gate["event_count"] >= 2 * summary["phase_valid_cycle_count"],
        "actor_layout": summary.get("n_actor") == EXPECTED_ACTOR_FEATURES,
        "full_layout": summary.get("n_observation") == EXPECTED_FULL_FEATURES,
        "dtype": summary.get("observation_dtype") == EXPECTED_OBSERVATION_DTYPE,
        "actor_names_exact": summary.get("actor_feature_names")
        == list(EXPECTED_ACTOR_FEATURE_NAMES),
        "full_names_exact": summary.get("observation_feature_names")
        == list(EXPECTED_OBSERVATION_FEATURE_NAMES),
        "invariant_columns_exact": summary.get("invariant_columns")
        == list(INVARIANT_COLUMNS),
        "no_candidate": summary.get("candidate_created") is False,
        "zero_actor_updates": _nonnegative_int(summary.get("actor_updates"))
        and summary["actor_updates"] == 0,
        "zero_critic_updates": _nonnegative_int(summary.get("critic_updates"))
        and summary["critic_updates"] == 0,
        "zero_ppo_updates": _nonnegative_int(summary.get("ppo_updates"))
        and summary["ppo_updates"] == 0,
        "protected_closed": summary.get("protected_trials_opened") == [],
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": ROLLOUT_PASS_STATUS if passed else ROLLOUT_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "case_id": case_id,
        "checks": checks,
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
    "V26_ACTIVE_EVENT_CONTRACT_ID",
    "V26_ACTOR_ADAPTER_SOURCE",
    "V26_FSM_SOURCE",
    "WORKER_CLAIMS_ROOT",
    "canonical_case",
    "replay_gate",
]
