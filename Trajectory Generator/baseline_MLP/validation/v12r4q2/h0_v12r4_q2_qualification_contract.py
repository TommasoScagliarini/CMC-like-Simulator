"""Pure source contract for the deferred-candidate V12R4-Q2 qualification.

Q2 reuses the six unopened V6/Q1 condition values, but every future tape and
rollout lives in the additive ``v12r4q2`` namespace.  The conditions are
preregistered before V12R4 development and may not be read, materialized, or
used for model selection by V12R4.  They are therefore unopened, but they are
not claimed to be statistically blind because their numeric values appeared in
the earlier V6/Q1 source contracts.

The candidate identity is deliberately absent.  A later Q2 protocol freezer
may bind it only to the exact candidate named, byte-bound, and terminally
passed by the future V12R4 development lineage.  Importing this module performs
no filesystem access, random-number generation, environment/model import,
rollout, fitting, publication, checkpointing, or promotion.
"""

from __future__ import annotations

import copy
from pathlib import PurePosixPath
from typing import Any


SCHEMA_VERSION = 140
REVISION = "2026-08-09"
AUTHORITY_TEXT = "esegui i punti 1-6"
AUTHORITY_SCOPE = "V12R4_Q2_DEFERRED_CANDIDATE_DESIGN_SOURCE_ONLY"
PROTOCOL_ID = "AB06_H0_V12R4_Q2_V26_INDEPENDENT_QUALIFICATION_DESIGN"
PIPELINE_ID = "H0_V12R4_Q2_V26_SIX_CASE_PAIRED_QUALIFICATION"

VALIDATION_ROOT = PurePosixPath("Trajectory Generator/baseline_MLP/validation/v12r4q2")
RUN_ROOT = VALIDATION_ROOT / "h0_v12r4_q2_run_20260809"
NOISE_ROOT = VALIDATION_ROOT / "h0_v12r4_q2_qualification_noise_tapes"
NOISE_MANIFEST_PATH = NOISE_ROOT / "manifest.json"
BASELINE_ROOT = RUN_ROOT / "baseline"
CANDIDATE_ROOT = RUN_ROOT / "candidate"
PAIR_ROOT = RUN_ROOT / "pairs"
FINAL_ROOT = RUN_ROOT / "finalize"
WORKER_CLAIMS_ROOT = RUN_ROOT / "claims"

QUALIFICATION_DESIGN_FREEZE_PATH = (
    VALIDATION_ROOT / "h0_v12r4_q2_qualification_design_freeze.json"
)
PROTOCOL_FREEZE_PATH = (
    VALIDATION_ROOT / "h0_v12r4_q2_qualification_protocol_freeze.json"
)
EXECUTION_LOCK_PATH = VALIDATION_ROOT / "h0_v12r4_q2_qualification_execution_lock.json"
PIPELINE_CLAIM_PATH = RUN_ROOT / "pipeline_claim.json"
PIPELINE_LEDGER_PATH = RUN_ROOT / "pipeline_ledger.json"
FINAL_GATE_PATH = FINAL_ROOT / "gate.json"
FINAL_RECEIPT_PATH = FINAL_ROOT / "receipt.json"
FINAL_SUMMARY_PATH = FINAL_ROOT / "summary.json"
FINAL_FAILURE_PATH = FINAL_ROOT / "failure.json"

QUALIFICATION_DESIGN_FREEZE_PASS_STATUS = "PASS_H0_V12R4_Q2_QUALIFICATION_DESIGN_FREEZE"
QUALIFICATION_DESIGN_FREEZE_FAIL_STATUS = "FAIL_H0_V12R4_Q2_QUALIFICATION_DESIGN_FREEZE"
PROTOCOL_FREEZE_PASS_STATUS = "PASS_H0_V12R4_Q2_QUALIFICATION_PROTOCOL_FREEZE"
EXECUTION_LOCK_PASS_STATUS = "PASS_H0_V12R4_Q2_QUALIFICATION_EXECUTION_LOCK"
PAIR_PASS_STATUS = "PASS_H0_V12R4_Q2_QUALIFICATION_PAIR"
PAIR_FAIL_STATUS = "FAIL_H0_V12R4_Q2_QUALIFICATION_PAIR"
AGGREGATE_PASS_STATUS = "PASS_H0_V12R4_Q2_INDEPENDENT_QUALIFICATION"
AGGREGATE_FAIL_STATUS = "FAIL_H0_V12R4_Q2_INDEPENDENT_QUALIFICATION"
NEXT_STAGE_AFTER_Q2_PASS = "WAIT_SEPARATE_ZERO_UPDATE_PROTOCOL"

AUTHORITY = {
    "authority_date": REVISION,
    "authority_text": AUTHORITY_TEXT,
    "authority_scope": AUTHORITY_SCOPE,
    "source_scaffold_authorized": True,
    "design_freeze_source_authorized": True,
    "design_freeze_publication_authorized_now": True,
    "future_candidate_hash_binding_required": True,
    "noise_materialization_authorized": False,
    "qualification_execution_authorized": False,
    "actor_fit_authorized": False,
    "offline_teacher_labeling_authorized": False,
    "actor_updates_authorized": False,
    "critic_updates_authorized": False,
    "ppo_updates_authorized": False,
    "retry_authorized": False,
    "resume_authorized": False,
    "rescue_authorized": False,
    "sweep_authorized": False,
    "post_hoc_tuning_authorized": False,
    "runtime_promotion_authorized": False,
    "checkpoint_zero_authorized": False,
    "positive_morphology_authorized": False,
}

CANDIDATE_BINDING_STATE = "DEFERRED"
CANDIDATE_ID = None
CANDIDATE_MODULE = None
R4_ROOT = PurePosixPath("Trajectory Generator/baseline_MLP/validation/v12r4")
R4_RUN_ROOT = R4_ROOT / "h0_v12r4_run_20260809"
R4_PROTOCOL_FREEZE_PATH = R4_ROOT / "h0_v12r4_p3_coverage_protocol_freeze.json"
R4_EXECUTION_LOCK_PATH = R4_ROOT / "h0_v12r4_p3_coverage_execution_lock.json"
R4_CANDIDATE_MODULE_PATH = R4_RUN_ROOT / "fit_p3/rl_module_target_adapted"
R4_CANDIDATE_FREEZE_RECEIPT_PATH = R4_RUN_ROOT / "candidate_freeze_receipt.json"
R4_FINAL_DEVELOPMENT_RECEIPT_PATH = R4_RUN_ROOT / "final_development_receipt.json"
R4_PIPELINE_LEDGER_PATH = R4_RUN_ROOT / "pipeline_ledger.json"

R4_PROTOCOL_FREEZE_PASS_STATUS = "PASS_H0_V12R4_P3_COVERAGE_PROTOCOL_FREEZE"
R4_EXECUTION_LOCK_PASS_STATUS = "PASS_H0_V12R4_P3_COVERAGE_EXECUTION_LOCK"
R4_CANDIDATE_FREEZE_PASS_STATUS = "PASS_H0_V12R4_P3_CANDIDATE_FREEZE"
R4_FINAL_DEVELOPMENT_PASS_STATUS = "PASS_H0_V12R4_P3_DEVELOPMENT"
R4_PIPELINE_PASS_STATUS = "PASS_H0_V12R4_P3_COVERAGE_PIPELINE_TERMINAL"
R4_CANDIDATE_SELECTION_RULE = "SOLE_FIT_P3_OUTPUT_FROM_LOCKED_R4_RUN"

CANDIDATE_BINDING_POLICY = {
    "binding_state": CANDIDATE_BINDING_STATE,
    "candidate_id": CANDIDATE_ID,
    "candidate_module": CANDIDATE_MODULE,
    "source_lineage": "V12R4",
    "selection_rule": R4_CANDIDATE_SELECTION_RULE,
    "canonical_candidate_module_path": R4_CANDIDATE_MODULE_PATH.as_posix(),
    "same_candidate_id_required_across_all_prerequisites": True,
    "same_candidate_tree_required_across_all_prerequisites": True,
    "terminal_pass_required": True,
    "candidate_substitution_allowed": False,
    "candidate_alias_allowed": False,
    "mutable_candidate_identifier_allowed": False,
    "binding_may_occur_during_design_freeze": False,
    "binding_may_occur_only_in_future_protocol_freeze": True,
}

FUTURE_PREREQUISITE_REQUIREMENTS = (
    {
        "name": "r4_protocol_freeze",
        "path": R4_PROTOCOL_FREEZE_PATH.as_posix(),
        "required_status": R4_PROTOCOL_FREEZE_PASS_STATUS,
        "candidate_identity_required": False,
        "candidate_tree_required": False,
        "required_selection_rule": R4_CANDIDATE_SELECTION_RULE,
        "required_candidate_module_path": R4_CANDIDATE_MODULE_PATH.as_posix(),
    },
    {
        "name": "r4_execution_lock",
        "path": R4_EXECUTION_LOCK_PATH.as_posix(),
        "required_status": R4_EXECUTION_LOCK_PASS_STATUS,
        "candidate_identity_required": False,
        "candidate_tree_required": False,
        "required_selection_rule": R4_CANDIDATE_SELECTION_RULE,
        "required_candidate_module_path": R4_CANDIDATE_MODULE_PATH.as_posix(),
    },
    {
        "name": "r4_candidate_freeze_receipt",
        "path": R4_CANDIDATE_FREEZE_RECEIPT_PATH.as_posix(),
        "required_status": R4_CANDIDATE_FREEZE_PASS_STATUS,
        "candidate_identity_required": True,
        "candidate_tree_required": True,
    },
    {
        "name": "r4_final_development_receipt",
        "path": R4_FINAL_DEVELOPMENT_RECEIPT_PATH.as_posix(),
        "required_status": R4_FINAL_DEVELOPMENT_PASS_STATUS,
        "candidate_identity_required": True,
        "candidate_tree_required": True,
    },
    {
        "name": "r4_terminal_pass_ledger",
        "path": R4_PIPELINE_LEDGER_PATH.as_posix(),
        "required_status": R4_PIPELINE_PASS_STATUS,
        "candidate_identity_required": True,
        "candidate_tree_required": True,
        "terminal_required": True,
        "passed_required": True,
    },
)

SOURCE_H0_MODULE = {
    "path": (
        "validation/critic_warmup/"
        "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/rl_module_last"
    ),
    "tree_sha256": "f7f6c898975af109412af8c3f1a338b5076f9fefcec1e2723673fd821f1f13ee",
}

Q1_ROOT = PurePosixPath("Trajectory Generator/baseline_MLP/validation/v12p1q")
P1S_ROOT = PurePosixPath("Trajectory Generator/baseline_MLP/validation/v12p1s")
Q1_DESIGN_FREEZE_ARTIFACT = {
    "path": (Q1_ROOT / "h0_v12r3_p1_qualification_design_freeze.json").as_posix(),
    "sha256": "c64928cb85df0e1f5d53c5f2e6eba52172b7c831ccf712a0113522bcff4ef686",
    "size_bytes": 21_303,
}
P1S_TERMINAL_LEDGER_ARTIFACT = {
    "path": (P1S_ROOT / "h0_v12p1s_run_20260809/pipeline_ledger.json").as_posix(),
    "sha256": "e0969d83ed36b4b7b10f50abde0a8f9d85c7d0aeb97a3318a42ded3e0aaa4b92",
    "size_bytes": 5_674,
}
P1S_TERMINAL_FAIL_STATUS = "FAIL_H0_V12R3_P1_SALVAGE_DEVELOPMENT_TERMINAL"
HISTORICAL_EXCLUSIONS = {
    "q1_design_freeze": copy.deepcopy(Q1_DESIGN_FREEZE_ARTIFACT),
    "p1s_terminal_fail_ledger": copy.deepcopy(P1S_TERMINAL_LEDGER_ARTIFACT),
    "q1_candidate_is_not_q2_candidate": True,
    "p1s_candidate_is_not_q2_candidate": True,
    "historical_candidate_fallback_allowed": False,
    "historical_candidate_promotion_allowed": False,
}

FROZEN_EXTERNAL_SOURCE_ARTIFACTS = {
    "v6_qualification_contract": {
        "path": "validation/h0_primary_split_v6_qualification_contract.py",
        "sha256": "abeb365e786b7f1f4808f6c7d27d8b86f898131a7ede9db406b6e739a17376ec",
        "size_bytes": 10_346,
    },
    "v6_qualification_gates": {
        "path": "validation/compare_h0_primary_split_v6_qualification.py",
        "sha256": "07fb067d580ff93ced95329c6616d409665d9bf0918e49463aa87f232abad371",
        "size_bytes": 17_563,
    },
    "v6_noise_preparer": {
        "path": "validation/prepare_h0_primary_split_v6_qualification_noise_tapes.py",
        "sha256": "1d73370278c46e07b72b43f2b4e6c0fd6d355e3201d95b2f4796c644e8a93248",
        "size_bytes": 12_987,
    },
    "q1_qualification_contract": {
        "path": (Q1_ROOT / "h0_v12r3_p1_qualification_contract.py").as_posix(),
        "sha256": "fd0f47e3324e9befd71d1c943a71dbb5dc426cc94f1bbacc4353b54072ed3db9",
        "size_bytes": 21_592,
    },
    "q1_qualification_gates": {
        "path": (Q1_ROOT / "h0_v12r3_p1_qualification_gates.py").as_posix(),
        "sha256": "1d12d002cb7b84dee5b095e0f6ae516aedefa9688dfe869b2f08271d35b65487",
        "size_bytes": 24_530,
    },
    "q1_design_freezer": {
        "path": (Q1_ROOT / "freeze_h0_v12r3_p1_qualification_design.py").as_posix(),
        "sha256": "653e4a8193c87fb6cb367ca2d3f9c8d3476ee22a506cd47633e60bf949b78b7a",
        "size_bytes": 24_831,
    },
}

DESIGN_SOURCE_RELATIVE_PATHS = {
    **{
        name: record["path"]
        for name, record in FROZEN_EXTERNAL_SOURCE_ARTIFACTS.items()
    },
    "q2_qualification_contract": (
        VALIDATION_ROOT / "h0_v12r4_q2_qualification_contract.py"
    ).as_posix(),
    "q2_package_init": (VALIDATION_ROOT / "__init__.py").as_posix(),
    "q2_design_freezer": (
        VALIDATION_ROOT / "freeze_h0_v12r4_q2_qualification_design.py"
    ).as_posix(),
    "q2_contract_and_design_tests": (
        VALIDATION_ROOT / "test_h0_v12r4_q2_qualification_contract_and_design.py"
    ).as_posix(),
    "q2_line_endings_policy": (VALIDATION_ROOT / ".gitattributes").as_posix(),
}

SAFE_SOURCE_REUSE = {
    "v6_contract": (
        "canonical_cases",
        "RESERVE_TOLERANCES",
        "SEA_TOLERANCES",
    ),
    "q1_gates": ("artifact_record_matches",),
    "candidate_specific_q1_rollout_gates_reused": False,
    "reason": "Q1 rollout gates bind the historical P1 identity and Q1 paths",
}

HOLDOUT_PROVENANCE = {
    "status": "REUSED_UNOPENED_V6_HOLDOUTS_FROZEN_BEFORE_R4",
    "source_revision": "2026-08-06",
    "source_protocol_id": "AB06_H0_PRIMARY_SPLIT_V6_BINARY_EVENT_QUALIFICATION",
    "conditions_selected_before_r4_development": True,
    "condition_values_previously_visible_in_source": True,
    "statistical_blindness_claimed": False,
    "q2_noise_namespace_is_new": True,
    "q2_noise_tapes_materialized": False,
    "q2_qualification_rollouts_opened": False,
    "r4_may_read_q2_noise": False,
    "r4_may_materialize_q2_noise": False,
    "r4_may_execute_q2_rollouts": False,
    "r4_may_consume_q2_condition_tuples_for_training_or_selection": False,
    "candidate_outcomes_used_to_select_q2_holdouts": False,
    "q1_or_p1s_candidate_routing_reused": False,
    "v26_candidate_routing_required": True,
}

BASELINE_ROLE = "baseline"
CANDIDATE_ROLE = "candidate"
ROLE_ORDER = (BASELINE_ROLE, CANDIDATE_ROLE)
BASELINE_ACTOR_ID = "original_h0"
BASELINE_CONTRACT_ID = "original_h0+counterfactual_analog+legacy_events"
PRIMARY_LOAD_CONTRACT_ID = "primary_grf_split_v1"
EVENT_CONTRACT_ID = "binary_point_v25+heel_qualified_fsm_v2"
TARGET_CONTRACT_ID = "primary_grf_split_v1+binary_point_v25+heel_qualified_fsm_v2"
MORPHOLOGY_WEIGHT = 0.0

EXPECTED_STEPS = 500
EXPECTED_CONTROL_WINDOWS = 5_000
EXPECTED_RAW_SENSOR_SAMPLES = 5_000
MINIMUM_VALID_CYCLES = 2
PENETRATION_LIMIT_M = 0.025
EXPECTED_ACTOR_FEATURES = 35
EXPECTED_FULL_FEATURES = 84
EXPECTED_DTYPE = "float32"
EXPECTED_ACTION_SHAPE = (2,)
CANONICAL_OFFSET_S = 1.956870983805102
DETERMINISTIC_RUNTIME_SEED = 129
STOCHASTIC_SEEDS = (130, 131, 132, 133)
STOCHASTIC_SIGMA = 0.005
TAPE_ABI = {
    "array_key": "standard_normal",
    "shape": [500, 2],
    "dtype": "float32",
    "c_contiguous": True,
    "generator": "numpy.random.default_rng(seed).standard_normal",
    "cast": "astype(numpy.float32)",
    "standard_normal_pre_scaling": True,
    "hash_scheme": "sha256(dtype_ascii+compact_json_shape+contiguous_c_bytes)",
}
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

CASE_IDS = (
    "deterministic_offset_minus_0p30",
    "deterministic_offset_plus_0p30",
    "stochastic_nominal_seed_130",
    "stochastic_nominal_seed_131",
    "stochastic_nominal_seed_132",
    "stochastic_nominal_seed_133",
)
_ZERO_TAPE = NOISE_ROOT / "deterministic_all_zero.npz"
_CASE_ROWS = (
    (
        CASE_IDS[0],
        "deterministic",
        CANONICAL_OFFSET_S - 0.30,
        None,
        DETERMINISTIC_RUNTIME_SEED,
        0.0,
        _ZERO_TAPE,
    ),
    (
        CASE_IDS[1],
        "deterministic",
        CANONICAL_OFFSET_S + 0.30,
        None,
        DETERMINISTIC_RUNTIME_SEED,
        0.0,
        _ZERO_TAPE,
    ),
    *tuple(
        (
            f"stochastic_nominal_seed_{seed}",
            "stochastic",
            CANONICAL_OFFSET_S,
            seed,
            seed,
            STOCHASTIC_SIGMA,
            NOISE_ROOT / f"stochastic_seed_{seed}_standard_normal.npz",
        )
        for seed in STOCHASTIC_SEEDS
    ),
)
HOLDOUT_CASES = tuple(
    {
        "case_id": case_id,
        "action_selection": action_selection,
        "episode_start_offset_s": offset,
        "action_seed": action_seed,
        "runtime_seed": runtime_seed,
        "sigma": sigma,
        "noise_tape": noise_tape.as_posix(),
    }
    for (
        case_id,
        action_selection,
        offset,
        action_seed,
        runtime_seed,
        sigma,
        noise_tape,
    ) in _CASE_ROWS
)

ROLLOUT_MATRIX = tuple(
    {
        "role": role,
        "case_id": case["case_id"],
        "destination": (
            (BASELINE_ROOT if role == BASELINE_ROLE else CANDIDATE_ROOT)
            / str(case["case_id"])
        ).as_posix(),
    }
    for role in ROLE_ORDER
    for case in HOLDOUT_CASES
)
ROLLOUT_STAGE_IDS = tuple(
    f"rollout__{row['role']}__{row['case_id']}" for row in ROLLOUT_MATRIX
)
STAGE_IDS = (*ROLLOUT_STAGE_IDS, "finalize_qualification")

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
SEA_EXPECTED_SAMPLE_COUNTS = {
    "torque_error_nm": EXPECTED_RAW_SENSOR_SAMPLES,
    "tau_spring_nm": EXPECTED_RAW_SENSOR_SAMPLES,
    "tau_spring_rate_nm_s": 4_500,
    "motor_speed_rad_s": EXPECTED_RAW_SENSOR_SAMPLES,
    "motor_accel_rad_s2": EXPECTED_RAW_SENSOR_SAMPLES,
    "motor_power_w": EXPECTED_RAW_SENSOR_SAMPLES,
}
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
ZERO_REQUIRED_COUNTS = (
    "action_clipped_values",
    "fallback_count",
    "timeout_count",
    "safety_latch_activation_count",
    "safety_latch_release_count",
    "safety_intervention_count",
    "safety_stop_count",
    "sea_plugin_fallback_count",
    "so_solver_unaccepted_count",
    "hard_invalid_count",
    "invalid_event_count",
    "nonfinite_count",
    "routing_failure_count",
    "step_contract_failure_count",
    "physical_gate_bypass_count",
    "multiple_noise_application_count",
    "noise_application_mismatch_count",
    "served_action_teacher_dependency_count",
    "teacher_query_count",
    "mean_blend_count",
    "actor_updates",
    "critic_updates",
    "ppo_updates",
)


def prerequisite_requirements() -> tuple[dict[str, Any], ...]:
    """Return an isolated copy of the future R4 binding requirements."""

    return copy.deepcopy(FUTURE_PREREQUISITE_REQUIREMENTS)


def candidate_binding_policy() -> dict[str, Any]:
    """Return the immutable design-time candidate binding policy."""

    return copy.deepcopy(CANDIDATE_BINDING_POLICY)


def canonical_cases() -> tuple[dict[str, Any], ...]:
    """Return isolated copies of the six Q2 conditions."""

    return copy.deepcopy(HOLDOUT_CASES)


def canonical_case(case_id: str) -> dict[str, Any]:
    matches = [case for case in HOLDOUT_CASES if case["case_id"] == case_id]
    if len(matches) != 1:
        raise ValueError(f"unknown Q2 qualification case: {case_id!r}")
    return copy.deepcopy(matches[0])


def role_contract(role: str) -> dict[str, Any]:
    common: dict[str, Any] = {
        "role": role,
        "candidate_binding_state": CANDIDATE_BINDING_STATE,
        "candidate_id": CANDIDATE_ID,
        "primary_load_contract_id": PRIMARY_LOAD_CONTRACT_ID,
        "morphology_weight": MORPHOLOGY_WEIGHT,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    if role == BASELINE_ROLE:
        common.update(
            {
                "actor_id": BASELINE_ACTOR_ID,
                "contract_id": BASELINE_CONTRACT_ID,
                "actor_input_view": "historical_analog",
                "observation_semantics": "counterfactual_analog",
                "phase_fsm_input_mode": "legacy_events",
                "event_contract_id": "legacy_events",
                "target_contract_id": BASELINE_CONTRACT_ID,
                "binary_phase_fsm_mode": "disabled",
                "actor_module": copy.deepcopy(SOURCE_H0_MODULE),
            }
        )
        return common
    if role == CANDIDATE_ROLE:
        common.update(
            {
                "actor_id": None,
                "contract_id": TARGET_CONTRACT_ID,
                "actor_input_view": "primary_split",
                "observation_semantics": PRIMARY_LOAD_CONTRACT_ID,
                "phase_fsm_input_mode": "legacy_events",
                "event_contract_id": EVENT_CONTRACT_ID,
                "target_contract_id": TARGET_CONTRACT_ID,
                "binary_phase_fsm_mode": "binary_active",
                "actor_module": None,
                "binding_policy": candidate_binding_policy(),
            }
        )
        return common
    raise ValueError(f"unknown Q2 qualification role: {role!r}")


def canonical_rollout(role: str, case_id: str) -> dict[str, Any]:
    if role not in ROLE_ORDER:
        raise ValueError(f"unknown Q2 qualification role: {role!r}")
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


def pair_gate_path(case_id: str) -> PurePosixPath:
    canonical_case(case_id)
    return PAIR_ROOT / f"{case_id}.json"


def worker_claim_path(stage_id: str) -> PurePosixPath:
    if stage_id not in STAGE_IDS:
        raise ValueError(f"unknown Q2 qualification stage: {stage_id!r}")
    return WORKER_CLAIMS_ROOT / f"{STAGE_IDS.index(stage_id) + 1:02d}_{stage_id}.json"


def declared_mutation_paths() -> dict[str, PurePosixPath]:
    paths: dict[str, PurePosixPath] = {
        "qualification_design_freeze": QUALIFICATION_DESIGN_FREEZE_PATH,
        "protocol_freeze": PROTOCOL_FREEZE_PATH,
        "execution_lock": EXECUTION_LOCK_PATH,
        "noise_root": NOISE_ROOT,
        "noise_manifest": NOISE_MANIFEST_PATH,
        "run_root": RUN_ROOT,
        "pipeline_claim": PIPELINE_CLAIM_PATH,
        "pipeline_ledger": PIPELINE_LEDGER_PATH,
    }
    for case in HOLDOUT_CASES:
        noise_path = PurePosixPath(case["noise_tape"])
        paths[f"noise_{noise_path.name}"] = noise_path
    for row in ROLLOUT_MATRIX:
        root = PurePosixPath(row["destination"])
        prefix = f"{row['role']}_{row['case_id']}"
        paths[f"{prefix}_root"] = root
        for name in ("summary", "gate", "receipt", "failure"):
            paths[f"{prefix}_{name}"] = root / f"{name}.json"
    for case_id in CASE_IDS:
        paths[f"pair_gate_{case_id}"] = pair_gate_path(case_id)
    paths.update(
        {
            "final_summary": FINAL_SUMMARY_PATH,
            "final_gate": FINAL_GATE_PATH,
            "final_receipt": FINAL_RECEIPT_PATH,
            "final_failure": FINAL_FAILURE_PATH,
        }
    )
    return paths


def contract_self_check() -> dict[str, Any]:
    """Evaluate the source-only design without touching external state."""

    cases = canonical_cases()
    baseline = role_contract(BASELINE_ROLE)
    candidate = role_contract(CANDIDATE_ROLE)
    matrix_pairs = tuple((row["role"], row["case_id"]) for row in ROLLOUT_MATRIX)
    expected_pairs = tuple(
        (role, case_id) for role in ROLE_ORDER for case_id in CASE_IDS
    )
    mutation_paths = declared_mutation_paths()
    checks = {
        "schema_revision_exact": SCHEMA_VERSION == 140 and REVISION == "2026-08-09",
        "candidate_identity_deferred": CANDIDATE_BINDING_STATE == "DEFERRED"
        and CANDIDATE_ID is None
        and CANDIDATE_MODULE is None
        and candidate["actor_id"] is None
        and candidate["actor_module"] is None,
        "future_r4_binding_exact": len(FUTURE_PREREQUISITE_REQUIREMENTS) == 5
        and CANDIDATE_BINDING_POLICY["source_lineage"] == "V12R4"
        and CANDIDATE_BINDING_POLICY["selection_rule"]
        == "SOLE_FIT_P3_OUTPUT_FROM_LOCKED_R4_RUN"
        and CANDIDATE_BINDING_POLICY["terminal_pass_required"] is True
        and CANDIDATE_BINDING_POLICY["candidate_substitution_allowed"] is False
        and all(
            requirement["candidate_identity_required"] is False
            and requirement["candidate_tree_required"] is False
            and requirement["required_selection_rule"] == R4_CANDIDATE_SELECTION_RULE
            for requirement in FUTURE_PREREQUISITE_REQUIREMENTS[:2]
        )
        and all(
            requirement["candidate_identity_required"] is True
            and requirement["candidate_tree_required"] is True
            for requirement in FUTURE_PREREQUISITE_REQUIREMENTS[2:]
        ),
        "historical_candidates_excluded": HISTORICAL_EXCLUSIONS[
            "historical_candidate_fallback_allowed"
        ]
        is False
        and HISTORICAL_EXCLUSIONS["historical_candidate_promotion_allowed"] is False,
        "six_cases_exact": len(cases) == 6
        and tuple(case["case_id"] for case in cases) == CASE_IDS,
        "twelve_rollouts_baseline_first": len(ROLLOUT_MATRIX) == 12
        and matrix_pairs == expected_pairs,
        "q2_namespace_only": all(
            str(value).startswith(VALIDATION_ROOT.as_posix())
            for value in mutation_paths.values()
        ),
        "baseline_original_h0_exact": baseline["actor_id"] == "original_h0"
        and baseline["actor_module"] == SOURCE_H0_MODULE
        and baseline["event_contract_id"] == "legacy_events"
        and baseline["binary_phase_fsm_mode"] == "disabled",
        "candidate_v26_semantics_exact": candidate["event_contract_id"]
        == EVENT_CONTRACT_ID
        and candidate["target_contract_id"] == TARGET_CONTRACT_ID
        and candidate["binary_phase_fsm_mode"] == "binary_active",
        "zero_morphology_and_updates": MORPHOLOGY_WEIGHT == 0.0
        and baseline["actor_updates"] == candidate["actor_updates"] == 0
        and baseline["critic_updates"] == candidate["critic_updates"] == 0
        and baseline["ppo_updates"] == candidate["ppo_updates"] == 0,
        "rollout_gate_constants_exact": EXPECTED_STEPS == 500
        and EXPECTED_CONTROL_WINDOWS == 5_000
        and EXPECTED_RAW_SENSOR_SAMPLES == 5_000
        and MINIMUM_VALID_CYCLES == 2
        and PENETRATION_LIMIT_M == 0.025,
        "pairwise_tolerances_exact": RESERVE_TOLERANCES
        == (
            ("reserve_norm_nm.rms", 5.0, 0.05),
            ("reserve_norm_nm.abs_max", 5.0, 0.05),
            ("residual_norm_nm.rms", 1.0e-6, 0.05),
            ("residual_norm_nm.abs_max", 1.0e-6, 0.05),
        )
        and len(SEA_TOLERANCES) == 24
        and all(row[1:] == (1.0e-6, 0.05) for row in SEA_TOLERANCES),
        "tape_abi_and_hashes_preregistered": TAPE_ABI
        == {
            "array_key": "standard_normal",
            "shape": [500, 2],
            "dtype": "float32",
            "c_contiguous": True,
            "generator": "numpy.random.default_rng(seed).standard_normal",
            "cast": "astype(numpy.float32)",
            "standard_normal_pre_scaling": True,
            "hash_scheme": (
                "sha256(dtype_ascii+compact_json_shape+contiguous_c_bytes)"
            ),
        }
        and len(EXPECTED_TAPE_ARRAY_SHA256) == 5
        and all(len(value) == 64 for value in EXPECTED_TAPE_ARRAY_SHA256.values()),
        "holdout_independence_warning_explicit": HOLDOUT_PROVENANCE[
            "statistical_blindness_claimed"
        ]
        is False
        and HOLDOUT_PROVENANCE[
            "r4_may_consume_q2_condition_tuples_for_training_or_selection"
        ]
        is False,
        "post_pass_zero_update_separate": NEXT_STAGE_AFTER_Q2_PASS
        == "WAIT_SEPARATE_ZERO_UPDATE_PROTOCOL",
        "authority_design_only": AUTHORITY["source_scaffold_authorized"] is True
        and AUTHORITY["design_freeze_source_authorized"] is True
        and AUTHORITY["design_freeze_publication_authorized_now"] is True
        and AUTHORITY["noise_materialization_authorized"] is False
        and AUTHORITY["qualification_execution_authorized"] is False,
    }
    return {"passed": all(checks.values()), "checks": checks}


__all__ = [
    "AGGREGATE_FAIL_STATUS",
    "AGGREGATE_PASS_STATUS",
    "AUTHORITY",
    "AUTHORITY_SCOPE",
    "AUTHORITY_TEXT",
    "BASELINE_ROLE",
    "CANDIDATE_BINDING_POLICY",
    "CANDIDATE_BINDING_STATE",
    "CANDIDATE_ID",
    "CANDIDATE_MODULE",
    "CANDIDATE_ROLE",
    "CASE_IDS",
    "DESIGN_SOURCE_RELATIVE_PATHS",
    "EVENT_CONTRACT_ID",
    "EXECUTION_LOCK_PATH",
    "EXPECTED_TAPE_ARRAY_SHA256",
    "FROZEN_EXTERNAL_SOURCE_ARTIFACTS",
    "FUTURE_PREREQUISITE_REQUIREMENTS",
    "HISTORICAL_EXCLUSIONS",
    "HOLDOUT_CASES",
    "HOLDOUT_PROVENANCE",
    "JOINTS",
    "MORPHOLOGY_WEIGHT",
    "NEXT_STAGE_AFTER_Q2_PASS",
    "NOISE_MANIFEST_PATH",
    "NOISE_ROOT",
    "P1S_TERMINAL_FAIL_STATUS",
    "P1S_TERMINAL_LEDGER_ARTIFACT",
    "PAIR_FAIL_STATUS",
    "PAIR_PASS_STATUS",
    "PIPELINE_ID",
    "PROTOCOL_FREEZE_PATH",
    "PROTOCOL_ID",
    "Q1_DESIGN_FREEZE_ARTIFACT",
    "QUALIFICATION_DESIGN_FREEZE_FAIL_STATUS",
    "QUALIFICATION_DESIGN_FREEZE_PASS_STATUS",
    "QUALIFICATION_DESIGN_FREEZE_PATH",
    "RESERVE_TOLERANCES",
    "ROLE_ORDER",
    "ROLLOUT_MATRIX",
    "RUN_ROOT",
    "SAFE_SOURCE_REUSE",
    "SCHEMA_VERSION",
    "SEA_EXPECTED_SAMPLE_COUNTS",
    "SEA_SIGNALS",
    "SEA_TOLERANCES",
    "SOURCE_H0_MODULE",
    "STAGE_IDS",
    "TARGET_CONTRACT_ID",
    "TAPE_ABI",
    "VALIDATION_ROOT",
    "ZERO_REQUIRED_COUNTS",
    "candidate_binding_policy",
    "canonical_case",
    "canonical_cases",
    "canonical_rollout",
    "contract_self_check",
    "declared_mutation_paths",
    "pair_gate_path",
    "prerequisite_requirements",
    "role_contract",
    "rollout_root",
    "worker_claim_path",
]
