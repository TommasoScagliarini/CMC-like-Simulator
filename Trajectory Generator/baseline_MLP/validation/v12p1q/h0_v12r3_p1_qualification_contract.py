"""Pure source contract for independent V12R3-P1 qualification.

The six holdout conditions were preregistered by the V6 source before P1
existed and remain unmaterialized.  This additive contract preserves those
conditions while replacing the historical candidate routing with the exact
P1/V26 contract.  It identifies no completed salvage artifact by hash: the
four prerequisite records may be frozen only after the separate V12P1S
development lineage terminates with six of six PASS.

Importing this module performs no filesystem access, random-number generation,
model loading, rollout, fitting, labelling, publication, or promotion.
"""

from __future__ import annotations

import copy
from pathlib import PurePosixPath
from typing import Any


SCHEMA_VERSION = 125
REVISION = "2026-08-09"
AUTHORITY_TEXT = "esegui i punti 1-6"
AUTHORITY_SCOPE = "V12R3_P1_INDEPENDENT_QUALIFICATION_SOURCE_ONLY"
PROTOCOL_ID = "AB06_H0_V12R3_P1_V26_INDEPENDENT_QUALIFICATION"
PIPELINE_ID = "H0_V12R3_P1_V26_SIX_CASE_PAIRED_QUALIFICATION"

VALIDATION_ROOT = PurePosixPath("Trajectory Generator/baseline_MLP/validation/v12p1q")
RUN_ROOT = VALIDATION_ROOT / "h0_v12p1q_run_20260809"
NOISE_ROOT = VALIDATION_ROOT / "noise_tapes"
NOISE_MANIFEST_PATH = NOISE_ROOT / "manifest.json"
BASELINE_ROOT = RUN_ROOT / "baseline"
CANDIDATE_ROOT = RUN_ROOT / "candidate"
PAIR_ROOT = RUN_ROOT / "pairs"
FINAL_ROOT = RUN_ROOT / "finalize"
WORKER_CLAIMS_ROOT = RUN_ROOT / "claims"

PROTOCOL_FREEZE_PATH = (
    VALIDATION_ROOT / "h0_v12r3_p1_qualification_protocol_freeze.json"
)
QUALIFICATION_DESIGN_FREEZE_PATH = (
    VALIDATION_ROOT / "h0_v12r3_p1_qualification_design_freeze.json"
)
EXECUTION_LOCK_PATH = VALIDATION_ROOT / "h0_v12r3_p1_qualification_execution_lock.json"
PIPELINE_CLAIM_PATH = RUN_ROOT / "pipeline_claim.json"
PIPELINE_LEDGER_PATH = RUN_ROOT / "pipeline_ledger.json"
FINAL_GATE_PATH = FINAL_ROOT / "gate.json"
FINAL_RECEIPT_PATH = FINAL_ROOT / "receipt.json"
FINAL_SUMMARY_PATH = FINAL_ROOT / "summary.json"
FINAL_FAILURE_PATH = FINAL_ROOT / "failure.json"

PROTOCOL_FREEZE_PASS_STATUS = "PASS_H0_V12R3_P1_QUALIFICATION_PROTOCOL_FREEZE"
PROTOCOL_FREEZE_FAIL_STATUS = "FAIL_H0_V12R3_P1_QUALIFICATION_PROTOCOL_FREEZE"
QUALIFICATION_DESIGN_FREEZE_PASS_STATUS = "PASS_H0_V12R3_P1_QUALIFICATION_DESIGN_FREEZE"
QUALIFICATION_DESIGN_FREEZE_FAIL_STATUS = "FAIL_H0_V12R3_P1_QUALIFICATION_DESIGN_FREEZE"
EXECUTION_LOCK_PASS_STATUS = "PASS_H0_V12R3_P1_QUALIFICATION_EXECUTION_LOCK"
EXECUTION_LOCK_FAIL_STATUS = "FAIL_H0_V12R3_P1_QUALIFICATION_EXECUTION_LOCK"
PREREQUISITE_COMPLETE_STATUS = "COMPLETE_H0_V12R3_P1_QUALIFICATION_PREREQUISITES"
PREREQUISITE_PASS_STATUS = "PASS_H0_V12R3_P1_QUALIFICATION_PREREQUISITES"
PREREQUISITE_FAIL_STATUS = "FAIL_H0_V12R3_P1_QUALIFICATION_PREREQUISITES"
ROLLOUT_COMPLETE_STATUS = "COMPLETE_H0_V12R3_P1_QUALIFICATION_ROLLOUT"
ROLLOUT_PASS_STATUS = "PASS_H0_V12R3_P1_QUALIFICATION_ROLLOUT"
ROLLOUT_FAIL_STATUS = "FAIL_H0_V12R3_P1_QUALIFICATION_ROLLOUT"
PAIR_PASS_STATUS = "PASS_H0_V12R3_P1_QUALIFICATION_PAIR"
PAIR_FAIL_STATUS = "FAIL_H0_V12R3_P1_QUALIFICATION_PAIR"
AGGREGATE_COMPLETE_STATUS = "COMPLETE_H0_V12R3_P1_INDEPENDENT_QUALIFICATION"
AGGREGATE_PASS_STATUS = "PASS_H0_V12R3_P1_INDEPENDENT_QUALIFICATION"
AGGREGATE_FAIL_STATUS = "FAIL_H0_V12R3_P1_INDEPENDENT_QUALIFICATION"

AUTHORITY = {
    "authority_date": REVISION,
    "authority_text": AUTHORITY_TEXT,
    "authority_scope": AUTHORITY_SCOPE,
    "source_scaffold_authorized": True,
    "design_freeze_source_authorized": True,
    "future_prerequisite_hash_binding_required": True,
    "noise_materialization_authorized": False,
    "qualification_execution_authorized": False,
    "actor_fit_authorized": False,
    "offline_teacher_labeling_authorized": False,
    "actor_updates_authorized": False,
    "critic_updates_authorized": False,
    "ppo_updates_authorized": False,
    "retry_authorized": False,
    "rescue_authorized": False,
    "sweep_authorized": False,
    "post_hoc_tuning_authorized": False,
    "runtime_promotion_authorized": False,
    "checkpoint_zero_authorized": False,
    "positive_morphology_authorized": False,
}

P1_CANDIDATE_ID = (
    "AB06_H0_PRIMARY_SPLIT_V12R3_V26_AUTONOMY_RECOVERY:" "p1:ff34e153ae0ac9b6"
)
P1_CANDIDATE_MODULE = {
    "path": (
        "Trajectory Generator/baseline_MLP/validation/v12r3/"
        "h0_v12r3_run_20260809/fit/p1/rl_module_target_adapted"
    ),
    "tree_sha256": "ff34e153ae0ac9b6f7b8d7d92766e47eecf087285020ac5332d9bd41170ac3ed",
    "file_count": 3,
    "files": [
        {
            "path": "class_and_ctor_args.pkl",
            "sha256": "5c98f006d99a71a0f1ddcbb31d8d73fe0a6dade8401e679f6af5b1bc943b4228",
            "size_bytes": 2_262,
        },
        {
            "path": "metadata.json",
            "sha256": "3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12",
            "size_bytes": 197,
        },
        {
            "path": "module_state.pkl",
            "sha256": "a4331ebff277275abc049732baad49093634168f13efdc1873a45680c9a2fe07",
            "size_bytes": 604_772,
        },
    ],
}

SOURCE_H0_MODULE = {
    "path": (
        "validation/critic_warmup/"
        "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/rl_module_last"
    ),
    "tree_sha256": "f7f6c898975af109412af8c3f1a338b5076f9fefcec1e2723673fd821f1f13ee",
}

SALVAGE_ROOT = PurePosixPath("Trajectory Generator/baseline_MLP/validation/v12p1s")
SALVAGE_RUN_ROOT = SALVAGE_ROOT / "h0_v12p1s_run_20260809"
SALVAGE_PROTOCOL_FREEZE_PATH = SALVAGE_ROOT / "h0_v12r3_p1_salvage_protocol_freeze.json"
SALVAGE_EXECUTION_LOCK_PATH = SALVAGE_ROOT / "h0_v12r3_p1_salvage_execution_lock.json"
SALVAGE_FINAL_DEVELOPMENT_RECEIPT_PATH = SALVAGE_RUN_ROOT / "finalize/receipt.json"
SALVAGE_PIPELINE_LEDGER_PATH = SALVAGE_RUN_ROOT / "pipeline_ledger.json"

SALVAGE_PROTOCOL_FREEZE_PASS_STATUS = "PASS_H0_V12R3_P1_SALVAGE_PROTOCOL_FREEZE"
SALVAGE_EXECUTION_LOCK_PASS_STATUS = "PASS_H0_V12R3_P1_SALVAGE_EXECUTION_LOCK"
SALVAGE_FINAL_DEVELOPMENT_PASS_STATUS = "PASS_H0_V12R3_P1_SALVAGE_FINAL_DEVELOPMENT"
SALVAGE_PIPELINE_PASS_STATUS = "PASS_H0_V12R3_P1_SALVAGE_DEVELOPMENT_TERMINAL"
SALVAGE_STAGE_IDS = (
    "development__deterministic_offset_minus_0p20",
    "development__deterministic_offset_nominal",
    "development__deterministic_offset_plus_0p20",
    "development__stochastic_nominal_seed_126",
    "development__stochastic_nominal_seed_127",
    "development__stochastic_nominal_seed_128",
    "finalize_development",
)

# Hash and size are intentionally absent here.  A future protocol-freeze
# builder must resolve these four paths after the salvage ledger is terminal
# PASS, bind exact records, then make the qualification lock no-clobber.
FUTURE_PREREQUISITE_REQUIREMENTS = (
    {
        "name": "salvage_protocol_freeze",
        "path": SALVAGE_PROTOCOL_FREEZE_PATH.as_posix(),
        "required_status": SALVAGE_PROTOCOL_FREEZE_PASS_STATUS,
        "candidate_identity_required": True,
    },
    {
        "name": "salvage_execution_lock",
        "path": SALVAGE_EXECUTION_LOCK_PATH.as_posix(),
        "required_status": SALVAGE_EXECUTION_LOCK_PASS_STATUS,
        "candidate_identity_required": True,
    },
    {
        "name": "salvage_final_development_receipt",
        "path": SALVAGE_FINAL_DEVELOPMENT_RECEIPT_PATH.as_posix(),
        "required_status": SALVAGE_FINAL_DEVELOPMENT_PASS_STATUS,
        "candidate_identity_required": True,
    },
    {
        "name": "salvage_terminal_pass_ledger",
        "path": SALVAGE_PIPELINE_LEDGER_PATH.as_posix(),
        "required_status": SALVAGE_PIPELINE_PASS_STATUS,
        "candidate_identity_required": True,
    },
)

V6_HOLDOUT_SOURCE_ARTIFACTS = {
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
}

# The design freezer hashes exactly this closed list.  Future noise preparers,
# runners, and protocol-freeze sources are deliberately absent so that adding
# them after salvage development cannot mutate the preregistered conditions.
DESIGN_SOURCE_RELATIVE_PATHS = {
    "v6_qualification_contract": (
        "validation/h0_primary_split_v6_qualification_contract.py"
    ),
    "v6_qualification_gates": (
        "validation/compare_h0_primary_split_v6_qualification.py"
    ),
    "v6_noise_preparer": (
        "validation/prepare_h0_primary_split_v6_qualification_noise_tapes.py"
    ),
    "qualification_contract": (
        "Trajectory Generator/baseline_MLP/validation/v12p1q/"
        "h0_v12r3_p1_qualification_contract.py"
    ),
    "qualification_gates": (
        "Trajectory Generator/baseline_MLP/validation/v12p1q/"
        "h0_v12r3_p1_qualification_gates.py"
    ),
    "qualification_design_freezer": (
        "Trajectory Generator/baseline_MLP/validation/v12p1q/"
        "freeze_h0_v12r3_p1_qualification_design.py"
    ),
    "qualification_contract_and_gates_tests": (
        "Trajectory Generator/baseline_MLP/validation/v12p1q/"
        "test_h0_v12r3_p1_qualification_contract_and_gates.py"
    ),
    "qualification_design_freezer_tests": (
        "Trajectory Generator/baseline_MLP/validation/v12p1q/"
        "test_freeze_h0_v12r3_p1_qualification_design.py"
    ),
}
HOLDOUT_PROVENANCE = {
    "source_revision": "2026-08-06",
    "source_protocol_id": "AB06_H0_PRIMARY_SPLIT_V6_BINARY_EVENT_QUALIFICATION",
    "conditions_selected_before_p1_existed": True,
    "source_noise_root": "validation/h0_primary_split_v6_qualification_noise_tapes",
    "source_noise_root_absent_when_v12p1q_scaffolded": True,
    "noise_tapes_materialized": False,
    "qualification_rollouts_opened": False,
    "candidate_outcomes_used_to_select_holdout": False,
    "v25_candidate_routing_reused": False,
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
    return copy.deepcopy(FUTURE_PREREQUISITE_REQUIREMENTS)


def canonical_cases() -> tuple[dict[str, Any], ...]:
    return copy.deepcopy(HOLDOUT_CASES)


def canonical_case(case_id: str) -> dict[str, Any]:
    matches = [case for case in HOLDOUT_CASES if case["case_id"] == case_id]
    if len(matches) != 1:
        raise ValueError(f"unknown P1 qualification case: {case_id!r}")
    return copy.deepcopy(matches[0])


def role_contract(role: str) -> dict[str, Any]:
    common: dict[str, Any] = {
        "role": role,
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
                "candidate_id": P1_CANDIDATE_ID,
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
                "actor_id": P1_CANDIDATE_ID,
                "candidate_id": P1_CANDIDATE_ID,
                "contract_id": TARGET_CONTRACT_ID,
                "actor_input_view": "primary_split",
                "observation_semantics": PRIMARY_LOAD_CONTRACT_ID,
                "phase_fsm_input_mode": "legacy_events",
                "event_contract_id": EVENT_CONTRACT_ID,
                "target_contract_id": TARGET_CONTRACT_ID,
                "binary_phase_fsm_mode": "binary_active",
                "actor_module": copy.deepcopy(P1_CANDIDATE_MODULE),
            }
        )
        return common
    raise ValueError(f"unknown P1 qualification role: {role!r}")


def canonical_rollout(role: str, case_id: str) -> dict[str, Any]:
    if role not in ROLE_ORDER:
        raise ValueError(f"unknown P1 qualification role: {role!r}")
    case = canonical_case(case_id)
    matches = [
        row
        for row in ROLLOUT_MATRIX
        if row["role"] == role and row["case_id"] == case_id
    ]
    if len(matches) != 1:
        raise ValueError(f"unknown P1 qualification rollout: {role}/{case_id}")
    return {**case, **copy.deepcopy(matches[0]), **role_contract(role)}


def rollout_root(role: str, case_id: str) -> PurePosixPath:
    return PurePosixPath(canonical_rollout(role, case_id)["destination"])


def pair_gate_path(case_id: str) -> PurePosixPath:
    canonical_case(case_id)
    return PAIR_ROOT / f"{case_id}.json"


def worker_claim_path(stage_id: str) -> PurePosixPath:
    if stage_id not in STAGE_IDS:
        raise ValueError(f"unknown P1 qualification stage: {stage_id!r}")
    return WORKER_CLAIMS_ROOT / f"{STAGE_IDS.index(stage_id) + 1:02d}_{stage_id}.json"


def rollout_receipt_path(role: str, case_id: str) -> PurePosixPath:
    return rollout_root(role, case_id) / "receipt.json"


def declared_mutation_paths() -> dict[str, PurePosixPath]:
    paths: dict[str, PurePosixPath] = {
        "qualification_design_freeze": QUALIFICATION_DESIGN_FREEZE_PATH,
        "protocol_freeze": PROTOCOL_FREEZE_PATH,
        "execution_lock": EXECUTION_LOCK_PATH,
        "noise_root": NOISE_ROOT,
        "noise_manifest": NOISE_MANIFEST_PATH,
        "run_root": RUN_ROOT,
        "baseline_root": BASELINE_ROOT,
        "candidate_root": CANDIDATE_ROOT,
        "pair_root": PAIR_ROOT,
        "final_root": FINAL_ROOT,
        "worker_claims_root": WORKER_CLAIMS_ROOT,
        "pipeline_claim": PIPELINE_CLAIM_PATH,
        "pipeline_ledger": PIPELINE_LEDGER_PATH,
    }
    for case in HOLDOUT_CASES:
        noise_path = PurePosixPath(case["noise_tape"])
        paths[f"noise_{noise_path.name}"] = noise_path
    for stage_id in STAGE_IDS:
        paths[f"worker_claim_{stage_id}"] = worker_claim_path(stage_id)
    for row in ROLLOUT_MATRIX:
        root = PurePosixPath(row["destination"])
        prefix = f"{row['role']}_{row['case_id']}"
        paths[f"{prefix}_root"] = root
        paths[f"{prefix}_steps_root"] = root / "steps"
        for name in (
            "run_start",
            "trace",
            "partial_summary",
            "summary",
            "gate",
            "receipt",
            "failure",
        ):
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


__all__ = [
    "AGGREGATE_COMPLETE_STATUS",
    "AGGREGATE_FAIL_STATUS",
    "AGGREGATE_PASS_STATUS",
    "AUTHORITY",
    "AUTHORITY_SCOPE",
    "AUTHORITY_TEXT",
    "BASELINE_ROLE",
    "CANDIDATE_ROLE",
    "CASE_IDS",
    "CONTINUOUS_AGGREGATIONS",
    "DESIGN_SOURCE_RELATIVE_PATHS",
    "EVENT_CONTRACT_ID",
    "EXECUTION_LOCK_PATH",
    "FUTURE_PREREQUISITE_REQUIREMENTS",
    "HOLDOUT_CASES",
    "HOLDOUT_PROVENANCE",
    "JOINTS",
    "MORPHOLOGY_WEIGHT",
    "NOISE_MANIFEST_PATH",
    "NOISE_ROOT",
    "P1_CANDIDATE_ID",
    "P1_CANDIDATE_MODULE",
    "PAIR_FAIL_STATUS",
    "PAIR_PASS_STATUS",
    "PIPELINE_CLAIM_PATH",
    "PIPELINE_ID",
    "PIPELINE_LEDGER_PATH",
    "PREREQUISITE_COMPLETE_STATUS",
    "PREREQUISITE_FAIL_STATUS",
    "PREREQUISITE_PASS_STATUS",
    "QUALIFICATION_DESIGN_FREEZE_FAIL_STATUS",
    "QUALIFICATION_DESIGN_FREEZE_PASS_STATUS",
    "QUALIFICATION_DESIGN_FREEZE_PATH",
    "PROTOCOL_FREEZE_PATH",
    "PROTOCOL_ID",
    "RESERVE_TOLERANCES",
    "ROLE_ORDER",
    "ROLLOUT_COMPLETE_STATUS",
    "ROLLOUT_FAIL_STATUS",
    "ROLLOUT_MATRIX",
    "ROLLOUT_PASS_STATUS",
    "SALVAGE_STAGE_IDS",
    "SCHEMA_VERSION",
    "SEA_EXPECTED_SAMPLE_COUNTS",
    "SEA_SIGNALS",
    "SEA_TOLERANCES",
    "SOURCE_H0_MODULE",
    "STAGE_IDS",
    "TARGET_CONTRACT_ID",
    "V6_HOLDOUT_SOURCE_ARTIFACTS",
    "VALIDATION_ROOT",
    "ZERO_REQUIRED_COUNTS",
    "canonical_case",
    "canonical_cases",
    "canonical_rollout",
    "declared_mutation_paths",
    "pair_gate_path",
    "prerequisite_requirements",
    "role_contract",
    "rollout_receipt_path",
    "rollout_root",
    "worker_claim_path",
]
