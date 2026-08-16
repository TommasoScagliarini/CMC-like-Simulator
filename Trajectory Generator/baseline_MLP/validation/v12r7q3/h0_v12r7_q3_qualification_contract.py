"""Execution-free independent Q3 contract for the future V12R7 candidate.

The contract intentionally remains candidate- and artifact-deferred.  It fixes
the six public V6/Q2 conditions, baseline-first ordering, exact standard actor
interface, V26 detector runtime, and weight-zero causal morphology audit before
any tape, rollout, freeze, checkpoint, or promotion is allowed.

Importing this module performs no filesystem reads, random-number generation,
model loading, environment construction, fitting, or publication.
"""

from __future__ import annotations

import copy
import sys
from pathlib import Path, PurePosixPath
from typing import Any


_LOCAL_VALIDATION = Path(__file__).resolve().parent.parent
if str(_LOCAL_VALIDATION / "v12r7") not in sys.path:
    sys.path.insert(0, str(_LOCAL_VALIDATION / "v12r7"))

import h0_v12r7_recovery_contract as r7  # noqa: E402


SCHEMA_VERSION = 1273
REVISION = "2026-08-14"
PROTOCOL_ID = "AB06_H0_V12R7_Q3_V26_MORPHOLOGY_ZERO_QUALIFICATION"
PIPELINE_ID = "H0_V12R7_Q3_BASELINE_FIRST_SIX_CASE_PAIRED"
LINEAGE_STATE = "HISTORICAL_R7_TERMINAL_FAIL_NO_Q3_PUBLICATION"
HISTORICAL_TERMINAL_FAILURE = True
SUCCESSOR_NAMESPACE = "v12r8q3"

AUTHORITY = {
    "source_scaffold_authorized": True,
    "candidate_binding_deferred": True,
    "protocol_freeze_authorized_now": False,
    "noise_materialization_authorized_now": False,
    "qualification_execution_authorized_now": False,
    "checkpoint_zero_authorized_now": False,
    "positive_morphology_authorized_now": False,
    "actor_fit_authorized": False,
    "actor_updates_authorized": False,
    "critic_updates_authorized": False,
    "ppo_updates_authorized": False,
    "retry_authorized": False,
    "resume_authorized": False,
    "rescue_authorized": False,
    "sweep_authorized": False,
    "post_hoc_tuning_authorized": False,
    "runtime_promotion_authorized": False,
}

ROOT = PurePosixPath("Trajectory Generator/baseline_MLP/validation/v12r7q3")
RUN_ROOT = ROOT / "h0_v12r7_q3_run_20260814"
NOISE_ROOT = ROOT / "h0_v12r7_q3_noise_tapes"
BASELINE_ROOT = RUN_ROOT / "baseline"
CANDIDATE_ROOT = RUN_ROOT / "candidate"
PAIR_ROOT = RUN_ROOT / "pairs"
FINAL_ROOT = RUN_ROOT / "finalize"
PROTOCOL_FREEZE_PATH = ROOT / "h0_v12r7_q3_qualification_protocol_freeze.json"
EXECUTION_LOCK_PATH = ROOT / "h0_v12r7_q3_qualification_execution_lock.json"
PIPELINE_CLAIM_PATH = RUN_ROOT / "pipeline_claim.json"
PIPELINE_LEDGER_PATH = RUN_ROOT / "pipeline_ledger.json"
FINAL_RECEIPT_PATH = FINAL_ROOT / "receipt.json"

PREREQUISITE_PASS_STATUS = "PASS_H0_V12R7_Q3_R7_PREREQUISITES"
PREREQUISITE_FAIL_STATUS = "FAIL_H0_V12R7_Q3_R7_PREREQUISITES"
ROLLOUT_COMPLETE_STATUS = "COMPLETE_H0_V12R7_Q3_QUALIFICATION_ROLLOUT"
ROLLOUT_PASS_STATUS = "PASS_H0_V12R7_Q3_QUALIFICATION_ROLLOUT"
ROLLOUT_FAIL_STATUS = "FAIL_H0_V12R7_Q3_QUALIFICATION_ROLLOUT"
PAIR_PASS_STATUS = "PASS_H0_V12R7_Q3_QUALIFICATION_PAIR"
PAIR_FAIL_STATUS = "FAIL_H0_V12R7_Q3_QUALIFICATION_PAIR"
AGGREGATE_COMPLETE_STATUS = "COMPLETE_H0_V12R7_Q3_QUALIFICATION"
AGGREGATE_PASS_STATUS = "PASS_H0_V12R7_Q3_INDEPENDENT_QUALIFICATION"
AGGREGATE_FAIL_STATUS = "FAIL_H0_V12R7_Q3_INDEPENDENT_QUALIFICATION"
PIPELINE_TERMINAL_PASS_STATUS = "PASS_H0_V12R7_Q3_PIPELINE_TERMINAL"
NEXT_STAGE_AFTER_Q3_PASS = "WAIT_SEPARATE_CHECKPOINT_ZERO_PROTOCOL"

CANDIDATE_BINDING_STATE = "DEFERRED_UNTIL_R7_TERMINAL_PASS"
CANDIDATE_ID = None
CANDIDATE_MODULE = None
CANDIDATE_MODULE_PATH = r7.CANDIDATE_MODULE_PATH
CANDIDATE_SELECTION_RULE = r7.CANDIDATE_SELECTION_RULE
CANDIDATE_REQUIRED_FILES = (
    "actor_feature_manifest.json",
    "candidate_build_manifest.json",
    "class_and_ctor_args.pkl",
    "metadata.json",
    "module_state.pkl",
)

PREREQUISITE_REQUIREMENTS = (
    {
        "name": "r7_protocol_freeze",
        "path": r7.PROTOCOL_FREEZE_PATH.as_posix(),
        "required_status": r7.PROTOCOL_FREEZE_PASS_STATUS,
        "semantic_verifier": "verify_protocol_freeze",
        "candidate_required": False,
    },
    {
        "name": "r7_execution_lock",
        "path": r7.EXECUTION_LOCK_PATH.as_posix(),
        "required_status": r7.EXECUTION_LOCK_PASS_STATUS,
        "semantic_verifier": "verify_execution_lock",
        "candidate_required": False,
    },
    {
        "name": "r7_candidate_freeze_receipt",
        "path": r7.CANDIDATE_FREEZE_PATH.as_posix(),
        "required_status": r7.CANDIDATE_FREEZE_PASS_STATUS,
        "semantic_verifier": "verify_candidate_freeze_receipt",
        "candidate_required": True,
    },
    {
        "name": "r7_final_development_receipt",
        "path": r7.FINAL_DEVELOPMENT_PATH.as_posix(),
        "required_status": r7.DEVELOPMENT_PASS_STATUS,
        "semantic_verifier": "verify_final_development_receipt",
        "candidate_required": True,
    },
    {
        "name": "r7_terminal_pass_ledger",
        "path": r7.LEDGER_PATH.as_posix(),
        "required_status": r7.PIPELINE_TERMINAL_PASS_STATUS,
        "semantic_verifier": "verify_terminal_ledger",
        "candidate_required": True,
    },
)

EXPECTED_ACTOR_FEATURES = 35
EXPECTED_FULL_FEATURES = 84
EXPECTED_ACTION_SHAPE = (2,)
EXPECTED_DTYPE = "float32"
EXPECTED_HIDDENS = (512, 512)
DISABLED_CLOCK_COLUMNS = (0, 1)
ACTOR_FEATURE_NAMES = (
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
ACTOR_FEATURE_MANIFEST_KEYS = frozenset(
    {
        "schema_version",
        "status",
        "topology_id",
        "fit_contract_id",
        "actor_feature_count",
        "actor_feature_names",
        "fcnet_hiddens",
        "disabled_clock_columns",
        "actor_digest",
        "module_state_sha256",
    }
)

SOURCE_H0_MODULE = {
    "path": (
        "validation/critic_warmup/"
        "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/rl_module_last"
    ),
    "tree_sha256": "f7f6c898975af109412af8c3f1a338b5076f9fefcec1e2723673fd821f1f13ee",
}
SOURCE_H0_CONFIG_PATH = PurePosixPath(
    "validation/critic_warmup/"
    "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/"
    "training_cfg.resolved.yaml"
)
SOURCE_H0_CONFIG_SHA256 = (
    "6904f7a9000b63b5c1aab661ebcab4974dffdd1cfb8c731df6a953fc9234229e"
)
HISTORICAL_ANALOG_PROFILE_PATH = PurePosixPath(
    "online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json"
)
HISTORICAL_ANALOG_PROFILE_SHA256 = (
    "61ea948a3c0613e5c0e684a3197de118c7116e36188fca6993da79ce713fd99e"
)
BASELINE_ROLE = "baseline"
CANDIDATE_ROLE = "candidate"
ROLE_ORDER = (BASELINE_ROLE, CANDIDATE_ROLE)

PRIMARY_LOAD_CONTRACT_ID = "primary_grf_split_v1"
LEGACY_EVENT_CONTRACT_ID = "legacy_events_v1"
EVENT_CONTRACT_ID = "binary_point_v25+heel_qualified_fsm_v2"
TARGET_CONTRACT_ID = f"{PRIMARY_LOAD_CONTRACT_ID}+{EVENT_CONTRACT_ID}"
V26_ACTOR_EVENT_SOURCE = "binary_active_v26"
V26_BINARY_MODE = "binary_active"

DETECTOR_PROFILE_PATH = PurePosixPath(
    "validation/binary_phase_detector_v25_geometry_runs/"
    "2026-08-04_local_reach_sweep_dev02_04_08/selected_candidate_profile.json"
)
DETECTOR_PROFILE_SHA256 = (
    "db704e502b99e49bea6d89493812bafdac748f8ce8d3ce28214ff624078539a2"
)
MORPHOLOGY_PROFILE_PATH = PurePosixPath(
    "Trajectory Generator/baseline_MLP/morphology_profiles/"
    "ab06_prosthetic_event_warped_mean_std_corridor.json"
)
MORPHOLOGY_CONFIG_PATH = PurePosixPath(
    "Trajectory Generator/baseline_MLP/experimental_configs/"
    "morphology_event_anchored_causal_v26_candidate.yaml"
)
MORPHOLOGY_CONFIG_SHA256 = (
    "1287ad2b3e007b0c4846c8a2cb6daa65bf30cce85f994940c80b8c7348a65b5b"
)
MORPHOLOGY_PROFILE_CONFIG_VALUE = (
    "morphology_profiles/ab06_prosthetic_event_warped_mean_std_corridor.json"
)
MORPHOLOGY_PROFILE_SHA256 = (
    "33b1dd7cb0db40110a4f9c1b8c0dd49a498662211e6e132f0f3cefe8edc02a55"
)
MORPHOLOGY_PHASE_MODE = "event_anchored_causal_delayed_experimental"
MORPHOLOGY_DELAY_S = 0.04
MORPHOLOGY_MAX_DELIVERY_LATENCY_S = 0.01
MORPHOLOGY_WEIGHT = 0.0
MORPHOLOGY_ALLOW_EFFECTS = 0.0
MORPHOLOGY_HARD_TERMINATION_ENABLED = 0.0

CANDIDATE_RESOLVED_ENV_CONFIG = {
    "phase_fsm_input_mode": "legacy_events",
    "event_contract_id": LEGACY_EVENT_CONTRACT_ID,
    "binary_phase_fsm_mode": V26_BINARY_MODE,
    "binary_phase_detector_profile": DETECTOR_PROFILE_PATH.as_posix(),
    "binary_phase_detector_profile_sha256": DETECTOR_PROFILE_SHA256,
    "binary_phase_event_contract_id": EVENT_CONTRACT_ID,
    "binary_phase_debounce_s": 0.005,
    "detector_sample_dt_s": 0.001,
    "policy_step_s": 0.01,
    "actor_event_source": V26_ACTOR_EVENT_SOURCE,
    "morphology_profile": MORPHOLOGY_PROFILE_CONFIG_VALUE,
    "morphology_profile_sha256": MORPHOLOGY_PROFILE_SHA256,
    "morphology_phase_mode": MORPHOLOGY_PHASE_MODE,
    "morphology_reward_delay_s": MORPHOLOGY_DELAY_S,
    "morphology_max_delivery_latency_s": MORPHOLOGY_MAX_DELIVERY_LATENCY_S,
    "morphology_causal_event_contract_id": EVENT_CONTRACT_ID,
    "morphology_causal_allow_effects": MORPHOLOGY_ALLOW_EFFECTS,
    "morphology_experimental_allow_effects": MORPHOLOGY_ALLOW_EFFECTS,
    "morphology_weight": MORPHOLOGY_WEIGHT,
    "morphology_hard_termination_enabled": MORPHOLOGY_HARD_TERMINATION_ENABLED,
}
BASELINE_RESOLVED_ENV_CONFIG = {
    "phase_fsm_input_mode": "legacy_events",
    "event_contract_id": LEGACY_EVENT_CONTRACT_ID,
    "binary_phase_fsm_mode": "disabled",
    "actor_event_source": "legacy_events",
    "morphology_weight": 0.0,
    "morphology_hard_termination_enabled": 0.0,
}

EXPECTED_STEPS = 500
EXPECTED_CONTROL_WINDOWS = 5_000
EXPECTED_RAW_SENSOR_SAMPLES = 5_000
EXPECTED_REWARD_SAMPLES = 500
MINIMUM_VALID_CYCLES = 2
PENETRATION_LIMIT_M = 0.025
CANONICAL_OFFSET_S = 1.956870983805102
DETERMINISTIC_RUNTIME_SEED = 129
STOCHASTIC_SEEDS = (130, 131, 132, 133)
STOCHASTIC_SIGMA = 0.005

CASE_IDS = (
    "deterministic_offset_minus_0p30",
    "deterministic_offset_plus_0p30",
    "stochastic_nominal_seed_130",
    "stochastic_nominal_seed_131",
    "stochastic_nominal_seed_132",
    "stochastic_nominal_seed_133",
)
_ZERO_TAPE = NOISE_ROOT / "deterministic_all_zero.npz"
HOLDOUT_CASES = (
    {
        "case_id": CASE_IDS[0],
        "action_selection": "deterministic",
        "episode_start_offset_s": CANONICAL_OFFSET_S - 0.30,
        "action_seed": None,
        "runtime_seed": DETERMINISTIC_RUNTIME_SEED,
        "sigma": 0.0,
        "noise_tape": _ZERO_TAPE.as_posix(),
    },
    {
        "case_id": CASE_IDS[1],
        "action_selection": "deterministic",
        "episode_start_offset_s": CANONICAL_OFFSET_S + 0.30,
        "action_seed": None,
        "runtime_seed": DETERMINISTIC_RUNTIME_SEED,
        "sigma": 0.0,
        "noise_tape": _ZERO_TAPE.as_posix(),
    },
    *tuple(
        {
            "case_id": f"stochastic_nominal_seed_{seed}",
            "action_selection": "stochastic",
            "episode_start_offset_s": CANONICAL_OFFSET_S,
            "action_seed": seed,
            "runtime_seed": seed,
            "sigma": STOCHASTIC_SIGMA,
            "noise_tape": (
                NOISE_ROOT / f"stochastic_seed_{seed}_standard_normal.npz"
            ).as_posix(),
        }
        for seed in STOCHASTIC_SEEDS
    ),
)
EXPECTED_TAPE_ARRAY_SHA256 = {
    "deterministic_all_zero.npz": (
        "ab89c5ecd7d818ab19f726cffc9ce431f5889448c7a79f84927f7153e546782c"
    ),
    "stochastic_seed_130_standard_normal.npz": (
        "067a5fb858ba1a5365856367eb1de793d954c9cbc07fa1aaaed34520a08657fa"
    ),
    "stochastic_seed_131_standard_normal.npz": (
        "00ea968882002a0881451cc2d80a365b2f6a3ccbfd698fcbe3768ca572abae67"
    ),
    "stochastic_seed_132_standard_normal.npz": (
        "9905eddb8074676cfe1ac2feeee152d114e9532b8cee9844daad016fa4930b61"
    ),
    "stochastic_seed_133_standard_normal.npz": (
        "d25e7515c993742e24da93f01313348991ea35871f98dbf07cd767367ec44438"
    ),
}
ROLLOUT_MATRIX = tuple(
    {
        "role": role,
        "case_id": case_id,
        "destination": (
            (BASELINE_ROOT if role == BASELINE_ROLE else CANDIDATE_ROOT) / case_id
        ).as_posix(),
    }
    for role in ROLE_ORDER
    for case_id in CASE_IDS
)

MORPHOLOGY_ZERO_AB_CONTRACT = {
    "detector_sample_count": EXPECTED_RAW_SENSOR_SAMPLES,
    "reward_sample_count": EXPECTED_REWARD_SAMPLES,
    "baseline_branch": "MORPHOLOGY_DISABLED",
    "candidate_branch": MORPHOLOGY_PHASE_MODE,
    "morphology_weight": 0.0,
    "morphology_causal_allow_effects": 0.0,
    "morphology_hard_termination_enabled": 0.0,
    "reward_byte_identity_required": True,
    "action_byte_identity_required": True,
    "observation_byte_identity_required": True,
}

ZERO_REQUIRED_COUNTS = (
    "action_clipped_values",
    "fallback_count",
    "timeout_count",
    "safety_intervention_count",
    "sea_plugin_fallback_count",
    "so_solver_unaccepted_count",
    "hard_invalid_count",
    "invalid_event_count",
    "nonfinite_count",
    "routing_failure_count",
    "step_contract_failure_count",
    "multiple_noise_application_count",
    "noise_application_mismatch_count",
    "served_action_teacher_dependency_count",
    "teacher_query_count",
    "mean_blend_count",
    "actor_updates",
    "critic_updates",
    "ppo_updates",
)

JOINTS = ("pros_knee_angle", "pros_ankle_angle")
SEA_SIGNALS = (
    "torque_error_nm",
    "tau_spring_nm",
    "tau_spring_rate_nm_s",
    "motor_speed_rad_s",
    "motor_accel_rad_s2",
    "motor_power_w",
)
CONTINUOUS_AGGREGATIONS = ("rms", "abs_max")
RESERVE_TOLERANCES = (
    ("reserve_norm_nm.rms", 5.0, 0.05),
    ("reserve_norm_nm.abs_max", 5.0, 0.05),
    ("residual_norm_nm.rms", 1.0e-6, 0.05),
    ("residual_norm_nm.abs_max", 1.0e-6, 0.05),
)
SEA_TOLERANCES = tuple(
    (f"{joint}.{signal}.{aggregation}", 1.0e-6, 0.05)
    for joint in JOINTS
    for signal in SEA_SIGNALS
    for aggregation in CONTINUOUS_AGGREGATIONS
)
NONINFERIORITY_TOLERANCES = (*RESERVE_TOLERANCES, *SEA_TOLERANCES)

QUALIFICATION_SOURCE_PATHS = {
    "q3_package": (ROOT / "__init__.py").as_posix(),
    "q3_artifacts": (ROOT / "h0_v12r7_q3_artifacts.py").as_posix(),
    "q3_contract": (ROOT / "h0_v12r7_q3_qualification_contract.py").as_posix(),
    "q3_prerequisites": (ROOT / "h0_v12r7_q3_prerequisites.py").as_posix(),
    "q3_gates": (ROOT / "h0_v12r7_q3_qualification_gates.py").as_posix(),
    "q3_tests": (ROOT / "test_h0_v12r7_q3_source_scaffold.py").as_posix(),
    "q3_runtime_package": (ROOT / "runtime/__init__.py").as_posix(),
    "r7_contract": (r7.ROOT / "h0_v12r7_recovery_contract.py").as_posix(),
    "r7_protocol_freezer": (r7.ROOT / "freeze_h0_v12r7_recovery.py").as_posix(),
    "r7_runner": (r7.ROOT / "run_h0_v12r7_recovery.py").as_posix(),
    "mature_q3_physical_collector": (
        "Trajectory Generator/baseline_MLP/validation/v12r5q3/runtime/"
        "h0_v12r5_q3_physical_rollout.py"
    ),
    "forensic_writer": "validation/h0_forensic_rollout.py",
    "v6_role_runtime": "validation/run_h0_primary_split_v6_qualification.py",
    "v26_environment_source": ("validation/run_h0_primary_split_v9_causal_teacher.py"),
    "v12r3_physical_runtime": (
        "Trajectory Generator/baseline_MLP/validation/v12r3/"
        "run_h0_primary_split_v12r3_autonomy_recovery.py"
    ),
    "primary_split_pairing": (
        "Trajectory Generator/baseline_MLP/primary_grf_split_adaptation.py"
    ),
    "primary_split_v1_runtime": (
        "validation/run_h0_primary_grf_split_v1_adaptation.py"
    ),
    "primary_split_v3_runtime": (
        "validation/run_h0_primary_grf_split_v3_semantic_replay.py"
    ),
    "so_recovery_contract": "validation/h0_v3_so_recovery_contract.py",
    "binary_fsm_v26": "Trajectory Generator/binary_phase_fsm_v26.py",
    "binary_adapter_v26": "Trajectory Generator/binary_phase_adapter_v26.py",
    "prosthetic_phase_fsm": "Trajectory Generator/prosthetic_phase_fsm.py",
    "morphology_corridor": (
        "Trajectory Generator/baseline_MLP/experimental_morphology_corridor.py"
    ),
    "reward_wrapper": "Trajectory Generator/baseline_MLP/reward_function.py",
    "environment_factory": "Trajectory Generator/baseline_MLP/env_factory.py",
    "environment": "Trajectory Generator/osim_trj_cmc_like.py",
    "rollout_eval": "Trajectory Generator/baseline_MLP/rollout_eval.py",
    "asymmetric_module": ("Trajectory Generator/baseline_MLP/asymmetric_rl_module.py"),
}
DEFERRED_RUNTIME_SOURCE_PATHS = {
    "q3_protocol_freezer": (
        ROOT / "runtime/freeze_h0_v12r7_q3_qualification_protocol.py"
    ).as_posix(),
    "q3_noise_preparer": (
        ROOT / "runtime/prepare_h0_v12r7_q3_noise_tapes.py"
    ).as_posix(),
    "q3_physical_rollout": (
        ROOT / "runtime/h0_v12r7_q3_physical_rollout.py"
    ).as_posix(),
    "q3_runner": (ROOT / "runtime/run_h0_v12r7_q3_qualification.py").as_posix(),
    "q3_runtime_tests": (ROOT / "runtime/test_h0_v12r7_q3_runtime.py").as_posix(),
}
QUALIFICATION_INPUT_PATHS = {
    "detector_profile": DETECTOR_PROFILE_PATH.as_posix(),
    "morphology_profile": MORPHOLOGY_PROFILE_PATH.as_posix(),
    "morphology_config": MORPHOLOGY_CONFIG_PATH.as_posix(),
    "source_h0_config": SOURCE_H0_CONFIG_PATH.as_posix(),
    "historical_analog_profile": HISTORICAL_ANALOG_PROFILE_PATH.as_posix(),
    "baseline_shadow_v25_profile": DETECTOR_PROFILE_PATH.as_posix(),
}
QUALIFICATION_INPUT_SHA256 = {
    "detector_profile": DETECTOR_PROFILE_SHA256,
    "morphology_profile": MORPHOLOGY_PROFILE_SHA256,
    "morphology_config": MORPHOLOGY_CONFIG_SHA256,
    "source_h0_config": SOURCE_H0_CONFIG_SHA256,
    "historical_analog_profile": HISTORICAL_ANALOG_PROFILE_SHA256,
    "baseline_shadow_v25_profile": DETECTOR_PROFILE_SHA256,
}


def prerequisite_requirements() -> tuple[dict[str, Any], ...]:
    return copy.deepcopy(PREREQUISITE_REQUIREMENTS)


def canonical_cases() -> tuple[dict[str, Any], ...]:
    return copy.deepcopy(HOLDOUT_CASES)


def canonical_case(case_id: str) -> dict[str, Any]:
    for case in HOLDOUT_CASES:
        if case["case_id"] == case_id:
            return copy.deepcopy(case)
    raise ValueError(f"unknown V12R7-Q3 case: {case_id!r}")


def canonical_rollout(role: str, case_id: str) -> dict[str, Any]:
    if role not in ROLE_ORDER:
        raise ValueError(f"unknown V12R7-Q3 role: {role!r}")
    case = canonical_case(case_id)
    root = BASELINE_ROOT if role == BASELINE_ROLE else CANDIDATE_ROOT
    return {
        **case,
        "role": role,
        "destination": (root / case_id).as_posix(),
        "resolved_env_config": copy.deepcopy(
            BASELINE_RESOLVED_ENV_CONFIG
            if role == BASELINE_ROLE
            else CANDIDATE_RESOLVED_ENV_CONFIG
        ),
    }


def rollout_receipt_path(role: str, case_id: str) -> PurePosixPath:
    return (
        PurePosixPath(canonical_rollout(role, case_id)["destination"]) / "receipt.json"
    )


def pair_gate_path(case_id: str) -> PurePosixPath:
    canonical_case(case_id)
    return PAIR_ROOT / f"{case_id}.json"


def contract_self_check() -> dict[str, Any]:
    checks = {
        "candidate_deferred": CANDIDATE_ID is None and CANDIDATE_MODULE is None,
        "r7_binding_exact": CANDIDATE_MODULE_PATH == r7.CANDIDATE_MODULE_PATH
        and CANDIDATE_SELECTION_RULE == r7.CANDIDATE_SELECTION_RULE
        and len(PREREQUISITE_REQUIREMENTS) == 5,
        "exact_candidate_tree_interface": len(CANDIDATE_REQUIRED_FILES) == 5
        and len(set(CANDIDATE_REQUIRED_FILES)) == 5,
        "standard_actor_exact": EXPECTED_ACTOR_FEATURES == 35
        and EXPECTED_HIDDENS == (512, 512)
        and EXPECTED_ACTION_SHAPE == (2,)
        and len(ACTOR_FEATURE_NAMES) == 35,
        "baseline_first_six_plus_six": len(ROLLOUT_MATRIX) == 12
        and tuple(row["role"] for row in ROLLOUT_MATRIX[:6]) == (BASELINE_ROLE,) * 6
        and tuple(row["role"] for row in ROLLOUT_MATRIX[6:]) == (CANDIDATE_ROLE,) * 6,
        "v26_runtime_exact": CANDIDATE_RESOLVED_ENV_CONFIG["binary_phase_fsm_mode"]
        == "binary_active"
        and CANDIDATE_RESOLVED_ENV_CONFIG["binary_phase_event_contract_id"]
        == EVENT_CONTRACT_ID
        and CANDIDATE_RESOLVED_ENV_CONFIG["binary_phase_detector_profile_sha256"]
        == DETECTOR_PROFILE_SHA256,
        "morphology_zero_exact": CANDIDATE_RESOLVED_ENV_CONFIG["morphology_phase_mode"]
        == MORPHOLOGY_PHASE_MODE
        and CANDIDATE_RESOLVED_ENV_CONFIG["morphology_reward_delay_s"] == 0.04
        and CANDIDATE_RESOLVED_ENV_CONFIG["morphology_max_delivery_latency_s"] == 0.01
        and CANDIDATE_RESOLVED_ENV_CONFIG["morphology_causal_allow_effects"] == 0.0
        and CANDIDATE_RESOLVED_ENV_CONFIG["morphology_weight"] == 0.0
        and CANDIDATE_RESOLVED_ENV_CONFIG["morphology_hard_termination_enabled"] == 0.0,
        "five_thousand_sample_gate": EXPECTED_CONTROL_WINDOWS
        == EXPECTED_RAW_SENSOR_SAMPLES
        == 5_000,
        "source_closure_hooks_complete": bool(QUALIFICATION_SOURCE_PATHS)
        and bool(DEFERRED_RUNTIME_SOURCE_PATHS)
        and bool(QUALIFICATION_INPUT_PATHS),
        "execution_closed": AUTHORITY["protocol_freeze_authorized_now"] is False
        and AUTHORITY["noise_materialization_authorized_now"] is False
        and AUTHORITY["qualification_execution_authorized_now"] is False,
        "historical_terminal_fail": HISTORICAL_TERMINAL_FAILURE is True
        and LINEAGE_STATE == "HISTORICAL_R7_TERMINAL_FAIL_NO_Q3_PUBLICATION"
        and SUCCESSOR_NAMESPACE == "v12r8q3",
    }
    return {
        "status": "PASS_H0_V12R7_Q3_SOURCE_CONTRACT"
        if all(checks.values())
        else "FAIL_H0_V12R7_Q3_SOURCE_CONTRACT",
        "passed": all(checks.values()),
        "checks": checks,
    }


__all__ = [name for name in globals() if name.isupper()] + [
    "canonical_case",
    "canonical_cases",
    "canonical_rollout",
    "contract_self_check",
    "pair_gate_path",
    "prerequisite_requirements",
    "rollout_receipt_path",
]
