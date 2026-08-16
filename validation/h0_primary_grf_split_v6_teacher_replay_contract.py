"""Pure contract for V6 V25-active teacher-action replay development.

This module performs no filesystem, simulation, Torch, or RLlib work.  It
freezes the six V5 baseline trajectories that are reclassified as development,
the V25-active runtime semantics, the forensic output locations, and the
terminal gate consumed by the rollout collector.
"""

from __future__ import annotations

import copy
import math
from pathlib import PurePosixPath
from typing import Any, Mapping


SCHEMA_VERSION = 6
REVISION = "2026-08-06"
PROTOCOL_ID = "AB06_H0_PRIMARY_SPLIT_V6_V25_RESIDUAL_DAGGER"
COLLECTOR_ID = "H0_V6_V25_ACTIVE_TEACHER_ACTION_REPLAY"
LOCK_STATUS = "H0_PRIMARY_SPLIT_V6_V25_TEACHER_REPLAY_FROZEN"
PREFLIGHT_STATUS = "PASS_H0_PRIMARY_SPLIT_V6_V25_TEACHER_REPLAY_PREFLIGHT"
ROLLOUT_COLLECTED_STATUS = "H0_V6_V25_TEACHER_REPLAY_COLLECTED_UNGATED"
ROLLOUT_PASS_STATUS = "PASS_H0_V6_V25_TEACHER_REPLAY"
ROLLOUT_FAIL_STATUS = "FAIL_H0_V6_V25_TEACHER_REPLAY"
PROTOCOL_PASS_STATUS = "PASS_H0_V6_V25_TEACHER_REPLAY_DEVELOPMENT"
PROTOCOL_FAIL_STATUS = "FAIL_H0_V6_V25_TEACHER_REPLAY_DEVELOPMENT"
SOURCE_H0_ID = "H0_MARKOV35_PHASE_ALIGNED_SIGMA0005_ITER1_RETRY"
SOURCE_OBSERVATION_CONTRACT_ID = "historical_analog_h0_v1"
TARGET_OBSERVATION_CONTRACT_ID = (
    "primary_grf_split_v1+binary_point_v25+functional_contact_fsm_v1"
)
V25_ACTIVE_EVENT_CONTRACT_ID = "binary_point_v25+functional_contact_fsm_v1"
SO_POLICY_ID = "verified_status0_max_iter_v1"

EXPECTED_STEPS = 500
EXPECTED_CONTROL_WINDOWS = 5000
EXPECTED_RAW_SENSOR_SAMPLES = 5000
EXPECTED_SAMPLES_PER_STEP = 10
EXPECTED_ACTOR_FEATURES = 35
EXPECTED_FULL_FEATURES = 84
EXPECTED_ACTION_DIM = 2
EXPECTED_OBSERVATION_DTYPE = "float32"
EXPECTED_SAMPLE_DT_S = 0.001
EXPECTED_POLICY_DT_S = 0.010
EXPECTED_EPISODE_DURATION_S = 5.0
STOCHASTIC_SIGMA = 0.005
MORPHOLOGY_WEIGHT = 0.0
PENETRATION_LIMIT_M = 0.025
MINIMUM_VALID_CYCLES = 2
INVARIANT_COLUMN_RANGES = ((2, 10), (25, 35))
INVARIANT_COLUMNS = tuple(
    index for start, stop in INVARIANT_COLUMN_RANGES for index in range(start, stop)
)

RUN_ROOT = PurePosixPath(
    "validation/h0_primary_grf_split_adaptation_runs/"
    "2026-08-06_h0_primary_split_v6_v25_residual/teacher_replay"
)
LOCK_PATH = PurePosixPath(
    "validation/h0_primary_grf_split_v6_teacher_replay_execution_lock.json"
)
PREFLIGHT_RECEIPT_PATH = PurePosixPath(
    "validation/h0_primary_grf_split_v6_teacher_replay_preflight_receipt.json"
)
EXECUTION_LEDGER_PATH = RUN_ROOT / "execution_ledger.json"
EXECUTION_CLAIM_PATH = RUN_ROOT / "execution_claim.json"
WORKER_CLAIMS_ROOT = RUN_ROOT / "worker_claims"

V5_RUN_ROOT = PurePosixPath(
    "validation/h0_primary_grf_split_adaptation_runs/"
    "2026-08-06_h0_primary_split_v5_full_mean"
)
V5_BASELINE_ROOT = V5_RUN_ROOT / "qualification" / "baseline"

EXPECTED_ACTOR_FEATURE_NAMES = (
    "gait_phase_sin",
    "gait_phase_cos",
    "pros_knee_angle",
    "pros_knee_angle_vel",
    "pros_ankle_angle",
    "pros_ankle_angle_vel",
    "SEA_Knee_motor_angle",
    "SEA_Knee_motor_speed",
    "SEA_Ankle_motor_angle",
    "SEA_Ankle_motor_speed",
    "online_left_normal_grf_bw",
    "online_left_in_contact",
    "online_left_heel_strike",
    "online_left_toe_off",
    "online_left_gait_phase_sin",
    "online_left_gait_phase_cos",
    "online_left_cycle_duration_s",
    "phase_fsm_wait_hs",
    "phase_fsm_stance_after_hs",
    "phase_fsm_swing_after_to",
    "phase_expected_hs",
    "phase_expected_to",
    "phase_stance_elapsed_norm",
    "phase_swing_elapsed_norm",
    "phase_cycle_progress_credit",
    "pros_knee_angle_previous_endpoint",
    "pros_knee_angle_served_ref",
    "pros_knee_angle_served_ref_vel",
    "pros_knee_angle_served_ref_accel",
    "pros_knee_angle_sea_u",
    "pros_ankle_angle_previous_endpoint",
    "pros_ankle_angle_served_ref",
    "pros_ankle_angle_served_ref_vel",
    "pros_ankle_angle_served_ref_accel",
    "pros_ankle_angle_sea_u",
)
EXPECTED_OBSERVATION_FEATURE_NAMES = EXPECTED_ACTOR_FEATURE_NAMES + (
    "pros_knee_angle_sea_u_abs",
    "pros_knee_angle_sea_u_saturated",
    "pros_ankle_angle_sea_u_abs",
    "pros_ankle_angle_sea_u_saturated",
    "pros_knee_angle_target",
    "pros_knee_angle_target_vel",
    "pros_knee_angle_tracking_error",
    "pros_ankle_angle_target",
    "pros_ankle_angle_target_vel",
    "pros_ankle_angle_tracking_error",
    "pelvis_tx",
    "pelvis_tx_vel",
    "pelvis_tx_kin_ref",
    "pelvis_ty",
    "pelvis_ty_vel",
    "pelvis_ty_kin_ref",
    "pelvis_tz",
    "pelvis_tz_vel",
    "pelvis_tz_kin_ref",
    "pelvis_tilt",
    "pelvis_tilt_vel",
    "pelvis_tilt_kin_ref",
    "pelvis_list",
    "pelvis_list_vel",
    "pelvis_list_kin_ref",
    "pelvis_rotation",
    "pelvis_rotation_vel",
    "pelvis_rotation_kin_ref",
    "hip_flexion_r",
    "hip_flexion_r_vel",
    "hip_flexion_r_kin_ref",
    "knee_angle_r",
    "knee_angle_r_vel",
    "knee_angle_r_kin_ref",
    "ankle_angle_r",
    "ankle_angle_r_vel",
    "ankle_angle_r_kin_ref",
    "hip_flexion_l",
    "hip_flexion_l_vel",
    "hip_flexion_l_kin_ref",
    "hip_adduction_l",
    "hip_adduction_l_vel",
    "hip_adduction_l_kin_ref",
    "online_right_normal_grf_bw",
    "online_right_in_contact",
    "online_right_heel_strike",
    "online_right_toe_off",
    "online_right_gait_phase",
    "online_right_cycle_duration_s",
)

_CASE_ROWS = (
    ("deterministic_offset_minus_0p20", "deterministic", 1.756870983805102, None, 123),
    ("deterministic_offset_nominal", "deterministic", 1.956870983805102, None, 123),
    ("deterministic_offset_plus_0p20", "deterministic", 2.156870983805102, None, 123),
    ("stochastic_nominal_seed_126", "stochastic", 1.956870983805102, 126, 126),
    ("stochastic_nominal_seed_127", "stochastic", 1.956870983805102, 127, 127),
    ("stochastic_nominal_seed_128", "stochastic", 1.956870983805102, 128, 128),
)

CASES = tuple(
    {
        "case_id": case_id,
        "action_selection": selection,
        "episode_start_offset_s": offset,
        "action_seed": action_seed,
        "runtime_seed": runtime_seed,
        "sigma": STOCHASTIC_SIGMA if selection == "stochastic" else 0.0,
        "baseline_trace": (V5_BASELINE_ROOT / case_id / "trace.json").as_posix(),
        "baseline_summary": (V5_BASELINE_ROOT / case_id / "summary.json").as_posix(),
        "baseline_receipt": (V5_BASELINE_ROOT / case_id / "receipt.json").as_posix(),
        "destination": (RUN_ROOT / case_id).as_posix(),
    }
    for case_id, selection, offset, action_seed, runtime_seed in _CASE_ROWS
)
CASE_IDS = tuple(case["case_id"] for case in CASES)

SOURCE_RELATIVE_PATHS = {
    "contract": "validation/h0_primary_grf_split_v6_teacher_replay_contract.py",
    "collector": "validation/run_h0_primary_grf_split_v6_teacher_replay.py",
    "freezer": "validation/freeze_h0_primary_grf_split_v6_teacher_replay.py",
    "preflight_builder": (
        "validation/build_h0_primary_grf_split_v6_teacher_replay_preflight.py"
    ),
    "forensic_writer": "validation/h0_forensic_rollout.py",
    "primary_split_contract": (
        "Trajectory Generator/baseline_MLP/primary_grf_split_adaptation.py"
    ),
    "residual_module": (
        "Trajectory Generator/baseline_MLP/primary_split_v25_residual.py"
    ),
    "legacy_v25_runner": "validation/run_h0_v25_abc_preflight.py",
    "legacy_v25_comparator": "validation/compare_h0_v25_abc.py",
    "so_classifier": "validation/h0_v3_so_recovery_contract.py",
    "environment": "Trajectory Generator/osim_trj_cmc_like.py",
    "binary_phase_adapter": "Trajectory Generator/binary_phase_adapter.py",
    "binary_phase_fsm": "Trajectory Generator/binary_phase_fsm.py",
    "binary_phase_detector": "binary_phase_detector.py",
    "rollout_eval": "Trajectory Generator/baseline_MLP/rollout_eval.py",
    "training_config": "Trajectory Generator/baseline_MLP/training_config.py",
    "simulation_runner": "simulation_runner.py",
    "static_optimization": "static_optimization.py",
    "model_loader": "model_loader.py",
    "online_grf": "online_grf.py",
    "root_config": "config.py",
    "contract_tests": (
        "validation/test_h0_primary_grf_split_v6_teacher_replay_contract.py"
    ),
    "collector_tests": (
        "validation/test_run_h0_primary_grf_split_v6_teacher_replay.py"
    ),
    "freezer_tests": (
        "validation/test_freeze_h0_primary_grf_split_v6_teacher_replay.py"
    ),
    "forensic_writer_tests": "validation/test_h0_forensic_rollout.py",
    "residual_module_tests": "validation/test_primary_split_v25_residual.py",
}

INPUT_RELATIVE_PATHS = {
    "v5_terminal_qualification_ledger": (
        V5_RUN_ROOT / "qualification" / "qualification_execution_ledger.json"
    ).as_posix(),
    "v5_qualification_lock": (
        "validation/h0_primary_grf_split_v5_qualification_lock.json"
    ),
    "v5_execution_ledger": (V5_RUN_ROOT / "execution_ledger.json").as_posix(),
    "v5_holdout_receipt": (V5_RUN_ROOT / "holdout" / "receipt.json").as_posix(),
    "source_h0_config": (
        "validation/critic_warmup/"
        "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/"
        "training_cfg.resolved.yaml"
    ),
    "source_h0_module_state": (
        "validation/critic_warmup/"
        "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/"
        "rl_module_last/module_state.pkl"
    ),
    "source_h0_module_ctor": (
        "validation/critic_warmup/"
        "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/"
        "rl_module_last/class_and_ctor_args.pkl"
    ),
    "source_h0_module_metadata": (
        "validation/critic_warmup/"
        "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/"
        "rl_module_last/metadata.json"
    ),
    "actor_layout_reference": (
        "validation/h0_primary_grf_split_adaptation_runs/"
        "2026-08-06_h0_primary_split_v3_semantic_replay/"
        "replay/seed_123/summary.json"
    ),
    "v25_candidate_freeze": (
        "validation/binary_phase_detector_v25_development_candidate_freeze_lock.json"
    ),
    "v25_corrected_active_protocol": "validation/h0_v25_abc_protocol_corrected_lock.json",
    "v25_profile": (
        "validation/binary_phase_detector_v25_geometry_runs/"
        "2026-08-04_local_reach_sweep_dev02_04_08/"
        "selected_candidate_profile.json"
    ),
    "primary_grf_profile": (
        "online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json"
    ),
    "analog_teacher_profile": (
        "online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json"
    ),
}

AUTHORITY = {
    "development_teacher_action_replay_authorized": True,
    "binary_active_v25_execution_authorized": True,
    "v5_pass_baselines_reclassified_as_development": True,
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
    """Return a fresh copy of one exact development case."""

    matches = [case for case in CASES if case["case_id"] == case_id]
    if len(matches) != 1:
        raise ValueError(f"unknown V6 teacher replay case: {case_id!r}")
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
        "left_non_v25_source_count",
        "fallback_count",
        "hard_invalid_count",
    )
    event_count = value.get("event_count")
    events = value.get("events")
    event_rows_valid = False
    if isinstance(events, list) and events:
        expected_event: str | None = None
        previous_event_time = -math.inf
        event_rows_valid = True
        for index, event in enumerate(events):
            if not isinstance(event, Mapping):
                event_rows_valid = False
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
                or event.get("source") != "binary_phase_fsm_v20"
                or event.get("event_contract_id") != V25_ACTIVE_EVENT_CONTRACT_ID
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
                event_rows_valid = False
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
        and event_rows_valid
        and all(value.get(field) == 0 for field in zero_fields)
        and all(_nonnegative_int(value.get(field)) for field in zero_fields)
    )


def replay_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    """Classify one persisted V25-active replay summary fail-closed."""

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
        "behavior_exact": summary.get("behavior") == "FROZEN_V5_RAW_ACTION_REPLAY",
        "binary_active": summary.get("binary_phase_fsm_mode") == "binary_active",
        "binary_contract_exact": summary.get("binary_phase_event_contract_id")
        == V25_ACTIVE_EVENT_CONTRACT_ID,
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
        "no_unaccepted_so": _nonnegative_int(summary.get("so_solver_unaccepted_count"))
        and summary["so_solver_unaccepted_count"] == 0,
        "no_sea_fallback": _nonnegative_int(summary.get("sea_plugin_fallback_count"))
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
    "EXECUTION_LEDGER_PATH",
    "EXECUTION_CLAIM_PATH",
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
    "WORKER_CLAIMS_ROOT",
    "canonical_case",
    "replay_gate",
]
