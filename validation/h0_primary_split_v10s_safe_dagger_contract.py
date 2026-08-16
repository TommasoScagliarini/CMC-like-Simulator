"""Pure fail-closed contract for V10S full-mean safe DAgger.

V10S adapts the frozen H0 mean actor to the V26 observation/event contract.
It starts from the six passing V8R1P1 teacher-replay traces (6 x 500 rows),
uses the accepted V10 time-alignment adjudication as coherent-teacher
evidence, and performs one P0 fit followed by exactly three preregistered
safe-DAgger collection rounds.  Every fit restarts from H0 on the complete
cumulative corpus; no candidate is continued from the preceding fit.

Collection may use the diagnostic safety latch, but that latch never weakens
the physical gate.  The final six rollouts are V26-only, unblended, contain no
teacher query, and forbid the latch.  A failure is terminal for this protocol:
there is no retry, sweep, rescue, or protected-trial access path here.
"""

from __future__ import annotations

import copy
import hashlib
import math
from pathlib import PurePosixPath
from typing import Any, Mapping

try:
    from validation.h0_primary_split_v10s_blend import (
        SAFETY_LATCH_ACTIVATION_M,
        SAFETY_LATCH_RELEASE_M,
        SAFETY_LATCH_RELEASE_PHASE,
    )
except ModuleNotFoundError:  # Flat import when validation/ is on sys.path.
    from h0_primary_split_v10s_blend import (
        SAFETY_LATCH_ACTIVATION_M,
        SAFETY_LATCH_RELEASE_M,
        SAFETY_LATCH_RELEASE_PHASE,
    )


SCHEMA_VERSION = 100
REVISION = "2026-08-07"
PROTOCOL_ID = "AB06_H0_PRIMARY_SPLIT_V10S_V26_FULL_MEAN_SAFE_DAGGER"
PIPELINE_ID = "H0_V10S_V26_FULL_MEAN_THREE_ROUND_SAFE_DAGGER"

SOURCE_H0_ID = "H0_MARKOV35_PHASE_ALIGNED_SIGMA0005_ITER1_RETRY"
TARGET_CONTRACT_ID = (
    "primary_grf_split_v1+binary_point_v25+heel_qualified_fsm_v2"
)
EVENT_CONTRACT_ID = "binary_point_v25+heel_qualified_fsm_v2"
TEACHER_EVIDENCE_ID = "V10A_COHERENT_TEACHER_TIME_ALIGNMENT_ADJUDICATED"
TEACHER_ID = "H0_COHERENT_LEGACY_BLOCK_10_24_V1"
BASE_CORPUS_ID = "V8R1P1_TEACHER_REPLAY_6X500"
SO_POLICY_ID = "verified_status0_max_iter_v1"
TRAINABLE_SCOPE = "full_mean_network"
LOGSTD_POLICY = "frozen_bit_exact"

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

BASE_CORPUS_CASE_COUNT = 6
BASE_CORPUS_SAMPLES_PER_CASE = EXPECTED_STEPS
BASE_CORPUS_SAMPLE_COUNT = BASE_CORPUS_CASE_COUNT * BASE_CORPUS_SAMPLES_PER_CASE
COLLECTION_SAMPLES_PER_CASE = EXPECTED_STEPS
COLLECTION_CASE_COUNT_PER_ROUND = 2
COLLECTION_SAMPLES_PER_ROUND = (
    COLLECTION_CASE_COUNT_PER_ROUND * COLLECTION_SAMPLES_PER_CASE
)

# This is byte-for-byte the fixed V5 optimizer configuration.  The only V10S
# variation is the cumulative corpus supplied to each independent H0 refit.
FIT = {
    "seed": 123,
    "epochs": 400,
    "batch_size": 128,
    "learning_rate": 5.0e-5,
    "validation_fraction": 0.0,
    "patience": 0,
    "clip_weight": 1.0,
    "logstd_weight": 0.0,
    "anchor_weight": 1.0e-2,
    "selection_mode": "fixed_final_epoch",
    "trainable_first_layer_features": None,
    "freeze_logstd_head": True,
}
FIT_STAGES = ("p0", "p1", "p2", "p3")
ROUND_ALPHAS = {1: 0.25, 2: 0.50, 3: 0.75}
FIT_COMPLETED_ROUNDS = {
    "p0": (),
    "p1": (1,),
    "p2": (1, 2),
    "p3": (1, 2, 3),
}

# The originally considered 0.005 / 0.030 / 0.0001 envelope was impossible
# for the fixed V5 full-mean design: the preregistration audit measured about
# 0.00425 RMSE, 0.0474 max error, and 0.00234 reset max error.  These thresholds
# are an operational acceptance envelope around that sole design, not a new
# scientific performance claim.  Physical rollout limits remain unchanged.
OFFLINE_THRESHOLDS = {
    "rmse_max": 0.006,
    "max_abs_error_max": 0.060,
    "reset_max_abs_error_max": 0.003,
}
OFFLINE_THRESHOLD_PROVENANCE = {
    "kind": "PREREGISTERED_OPERATIONAL_ENVELOPE",
    "design": "FIXED_V5_FULL_MEAN_NO_SWEEP",
    "audit_rmse": 0.00425,
    "audit_max_abs_error": 0.0474,
    "audit_reset_max_abs_error": 0.00234,
    "scientific_claim": False,
    "physical_gate_changed": False,
}

RUN_ROOT = PurePosixPath(
    "validation/h0_primary_grf_split_adaptation_runs/"
    "2026-08-07_h0_primary_split_v10s_v26_full_mean_safe_dagger"
)
BASE_CORPUS_ROOT = PurePosixPath(
    "validation/h0_primary_grf_split_adaptation_runs/"
    "2026-08-07_h0_primary_split_v8r1p1_v26_residual/teacher_replay"
)
BASE_CORPUS_LEDGER_PATH = BASE_CORPUS_ROOT / "execution_ledger.json"
TEACHER_EVIDENCE_RECEIPT_PATH = PurePosixPath(
    "validation/h0_primary_split_v10_time_alignment_adjudication_receipt.json"
)
SOURCE_H0_MODULE_PATH = PurePosixPath(
    "validation/critic_warmup/"
    "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/rl_module_last"
)
ADAPTATION_ROOT = RUN_ROOT / "adaptation"
FIT_ROOTS = {
    stage: ADAPTATION_ROOT / stage
    for stage in FIT_STAGES
}
MODULE_PATHS = {
    stage: FIT_ROOTS[stage] / "rl_module_target_adapted"
    for stage in FIT_STAGES
}
FIT_RECEIPT_PATHS = {
    stage: FIT_ROOTS[stage] / "receipt.json"
    for stage in FIT_STAGES
}
COLLECTION_ROOT = RUN_ROOT / "safe_dagger_collection"
CANDIDATE_FREEZE_PATH = ADAPTATION_ROOT / "candidate_freeze.json"
FINAL_ROOT = RUN_ROOT / "final_development"
FINAL_DEVELOPMENT_RECEIPT_PATH = FINAL_ROOT / "receipt.json"
PREFLIGHT_PATH = PurePosixPath(
    "validation/h0_primary_split_v10s_safe_dagger_preflight_receipt.json"
)
LOCK_PATH = PurePosixPath(
    "validation/h0_primary_split_v10s_safe_dagger_execution_lock.json"
)
PIPELINE_CLAIM_PATH = RUN_ROOT / "pipeline_execution_claim.json"
PIPELINE_LEDGER_PATH = RUN_ROOT / "pipeline_execution_ledger.json"
WORKER_CLAIMS_ROOT = RUN_ROOT / "pipeline_worker_claims"

_CASE_ROWS = (
    (
        "deterministic_offset_minus_0p20",
        "deterministic",
        1.756870983805102,
        None,
        123,
    ),
    (
        "deterministic_offset_nominal",
        "deterministic",
        1.956870983805102,
        None,
        123,
    ),
    (
        "deterministic_offset_plus_0p20",
        "deterministic",
        2.156870983805102,
        None,
        123,
    ),
    (
        "stochastic_nominal_seed_126",
        "stochastic",
        1.956870983805102,
        126,
        126,
    ),
    (
        "stochastic_nominal_seed_127",
        "stochastic",
        1.956870983805102,
        127,
        127,
    ),
    (
        "stochastic_nominal_seed_128",
        "stochastic",
        1.956870983805102,
        128,
        128,
    ),
)

FINAL_CASES = tuple(
    {
        "case_id": case_id,
        "action_selection": selection,
        "episode_start_offset_s": offset,
        "action_seed": action_seed,
        "runtime_seed": runtime_seed,
        "sigma": EXPECTED_SIGMA if selection == "stochastic" else 0.0,
        "destination": (FINAL_ROOT / case_id).as_posix(),
    }
    for case_id, selection, offset, action_seed, runtime_seed in _CASE_ROWS
)
FINAL_CASE_IDS = tuple(case["case_id"] for case in FINAL_CASES)
COLLECTION_CASE_IDS = (
    "deterministic_offset_minus_0p20",
    "stochastic_nominal_seed_126",
)
COLLECTION_CASES = tuple(
    copy.deepcopy(
        next(case for case in FINAL_CASES if case["case_id"] == case_id)
    )
    for case_id in COLLECTION_CASE_IDS
)
BASE_CORPUS_CASES = tuple(
    {
        "case_id": case_id,
        "trace": (BASE_CORPUS_ROOT / case_id / "trace.json").as_posix(),
        "receipt": (BASE_CORPUS_ROOT / case_id / "receipt.json").as_posix(),
        "samples": BASE_CORPUS_SAMPLES_PER_CASE,
    }
    for case_id in FINAL_CASE_IDS
)

COLLECTION_BEHAVIOR = "V26_SAFE_DAGGER_BLEND_MEAN_THEN_SINGLE_NOISE"
FINAL_BEHAVIOR = "P3_V26_UNBLENDED_NO_TEACHER"

PREFLIGHT_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V10S_SAFE_DAGGER_PREFLIGHT"
LOCK_STATUS = "H0_PRIMARY_SPLIT_V10S_SAFE_DAGGER_FROZEN"
PIPELINE_CLAIM_STATUS = "H0_PRIMARY_SPLIT_V10S_PIPELINE_EXECUTION_CLAIMED"
WORKER_CLAIM_STATUS = "H0_PRIMARY_SPLIT_V10S_PIPELINE_WORKER_CLAIMED"
FIT_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V10S_FULL_MEAN_FIT"
FIT_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V10S_FULL_MEAN_FIT"
COLLECTION_STATUS = "H0_PRIMARY_SPLIT_V10S_COLLECTION_PERSISTED_UNGATED"
COLLECTION_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V10S_SAFE_DAGGER_COLLECTION"
COLLECTION_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V10S_SAFE_DAGGER_COLLECTION"
FREEZE_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V10S_P3_CANDIDATE_FREEZE"
FREEZE_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V10S_P3_CANDIDATE_FREEZE"
FINAL_ROLLOUT_STATUS = "H0_PRIMARY_SPLIT_V10S_FINAL_ROLLOUT_PERSISTED_UNGATED"
FINAL_ROLLOUT_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V10S_FINAL_ROLLOUT"
FINAL_ROLLOUT_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V10S_FINAL_ROLLOUT"
FINAL_DEVELOPMENT_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V10S_FINAL_DEVELOPMENT"
FINAL_DEVELOPMENT_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V10S_FINAL_DEVELOPMENT"
PIPELINE_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V10S_SAFE_DAGGER_PIPELINE"
PIPELINE_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V10S_SAFE_DAGGER_PIPELINE"

STAGE_IDS = (
    "fit_p0",
    *(f"collect_r1__{case_id}" for case_id in COLLECTION_CASE_IDS),
    "fit_p1",
    *(f"collect_r2__{case_id}" for case_id in COLLECTION_CASE_IDS),
    "fit_p2",
    *(f"collect_r3__{case_id}" for case_id in COLLECTION_CASE_IDS),
    "fit_p3",
    "freeze_p3",
    *(f"final__{case_id}" for case_id in FINAL_CASE_IDS),
    "finalize_development",
)

SOURCE_RELATIVE_PATHS = {
    "contract": "validation/h0_primary_split_v10s_safe_dagger_contract.py",
    "blend_helper": "validation/h0_primary_split_v10s_blend.py",
    "contract_tests": (
        "validation/test_h0_primary_split_v10s_safe_dagger_contract.py"
    ),
    "blend_tests": "validation/test_h0_primary_split_v10s_blend.py",
    "runner": "validation/run_h0_primary_split_v10s_safe_dagger.py",
    "runner_tests": (
        "validation/test_run_h0_primary_split_v10s_safe_dagger.py"
    ),
    "fit_engine": "validation/h0_primary_split_v10s_fit.py",
    "fit_engine_tests": "validation/test_h0_primary_split_v10s_fit.py",
    "coherent_teacher": "validation/h0_primary_split_v10_coherent_teacher.py",
    "forensic_writer": "validation/h0_forensic_rollout.py",
    "environment_config_source": (
        "validation/run_h0_primary_split_v9_causal_teacher.py"
    ),
    "v9_causal_teacher": "validation/h0_primary_split_v9_causal_teacher.py",
    "v9_causal_teacher_contract": (
        "validation/h0_primary_split_v9_causal_teacher_contract.py"
    ),
    "so_classifier": "validation/h0_v3_so_recovery_contract.py",
    "v26_event_collector": (
        "validation/run_h0_primary_grf_split_v8_teacher_replay.py"
    ),
    "v8_teacher_contract": (
        "validation/h0_primary_grf_split_v8_teacher_replay_contract.py"
    ),
    "v8r1_collector": (
        "validation/run_h0_primary_grf_split_v8r1_teacher_replay.py"
    ),
    "v8r1_contract": (
        "validation/h0_primary_grf_split_v8r1_teacher_replay_contract.py"
    ),
    "v8r1p1_collector": (
        "validation/run_h0_primary_grf_split_v8r1p1_teacher_replay.py"
    ),
    "legacy_runtime_helpers": "validation/run_h0_v25_abc_preflight.py",
    "legacy_runtime_comparator": "validation/compare_h0_v25_abc.py",
    "v6_teacher_engine": (
        "validation/run_h0_primary_grf_split_v6_teacher_replay.py"
    ),
    "v6_teacher_contract": (
        "validation/h0_primary_grf_split_v6_teacher_replay_contract.py"
    ),
    "v6_teacher_preflight": (
        "validation/build_h0_primary_grf_split_v6_teacher_replay_preflight.py"
    ),
    "v8r1p1_teacher_contract": (
        "validation/h0_primary_grf_split_v8r1p1_teacher_replay_contract.py"
    ),
    "primary_split_contract": (
        "Trajectory Generator/baseline_MLP/primary_grf_split_adaptation.py"
    ),
    "actor_fit": (
        "Trajectory Generator/baseline_MLP/target_domain_imitation.py"
    ),
    "asymmetric_rl_module": (
        "Trajectory Generator/baseline_MLP/asymmetric_rl_module.py"
    ),
    "warm_start": "Trajectory Generator/baseline_MLP/warm_start.py",
    "rollout_eval": "Trajectory Generator/baseline_MLP/rollout_eval.py",
    "training_config": (
        "Trajectory Generator/baseline_MLP/training_config.py"
    ),
    "environment_factory": "Trajectory Generator/baseline_MLP/env_factory.py",
    "reward_function": "Trajectory Generator/baseline_MLP/reward_function.py",
    "morphology_corridor": (
        "Trajectory Generator/baseline_MLP/experimental_morphology_corridor.py"
    ),
    "baseline_bootstrap": "Trajectory Generator/baseline_MLP/_bootstrap.py",
    "windows_runtime": "Trajectory Generator/baseline_MLP/win_runtime.py",
    "exploration_noise": (
        "Trajectory Generator/baseline_MLP/exploration_noise.py"
    ),
    "process_watchdog": (
        "Trajectory Generator/baseline_MLP/process_watchdog.py"
    ),
    "progress_display": (
        "Trajectory Generator/baseline_MLP/progress_display.py"
    ),
    "environment": "Trajectory Generator/osim_trj_cmc_like.py",
    "binary_phase_adapter": "Trajectory Generator/binary_phase_adapter.py",
    "binary_phase_adapter_v26": (
        "Trajectory Generator/binary_phase_adapter_v26.py"
    ),
    "binary_phase_fsm": "Trajectory Generator/binary_phase_fsm.py",
    "binary_phase_fsm_v26": "Trajectory Generator/binary_phase_fsm_v26.py",
    "prosthetic_phase_fsm": "Trajectory Generator/prosthetic_phase_fsm.py",
    "binary_phase_detector": "binary_phase_detector.py",
    "simulation_runner": "simulation_runner.py",
    "model_loader": "model_loader.py",
    "online_grf": "online_grf.py",
    "static_optimization": "static_optimization.py",
    "kinematics_interpolator": "kinematics_interpolator.py",
    "prosthesis_controller": "prosthesis_controller.py",
    "outer_loop": "outer_loop.py",
    "inverse_dynamics": "inverse_dynamics.py",
    "output": "output.py",
    "root_config": "config.py",
    "path_resolver": "path_resolver.py",
    "setup_io": "setup_io.py",
}

INPUT_RELATIVE_PATHS = {
    "teacher_evidence_receipt": TEACHER_EVIDENCE_RECEIPT_PATH.as_posix(),
    "base_corpus_ledger": BASE_CORPUS_LEDGER_PATH.as_posix(),
    "source_h0_config": (
        "validation/critic_warmup/"
        "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/"
        "training_cfg.resolved.yaml"
    ),
    "source_h0_state": (SOURCE_H0_MODULE_PATH / "module_state.pkl").as_posix(),
    "source_h0_ctor": (
        SOURCE_H0_MODULE_PATH / "class_and_ctor_args.pkl"
    ).as_posix(),
    "source_h0_metadata": (
        SOURCE_H0_MODULE_PATH / "metadata.json"
    ).as_posix(),
    "v26_development_receipt": (
        "validation/binary_phase_fsm_v26_development_receipt.json"
    ),
    "v26_v7_replay_receipt": (
        "validation/binary_phase_fsm_v26_v7_replay_receipt.json"
    ),
    "v25_candidate_freeze": (
        "validation/binary_phase_detector_v25_development_candidate_"
        "freeze_lock.json"
    ),
    "v25_profile": (
        "validation/binary_phase_detector_v25_geometry_runs/"
        "2026-08-04_local_reach_sweep_dev02_04_08/"
        "selected_candidate_profile.json"
    ),
    "primary_grf_core_lock": "validation/primary_grf_core_lock_2026-08-03.json",
    "primary_grf_profile": (
        "online_grf_profiles/"
        "AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json"
    ),
    "analog_detector_profile": (
        "online_grf_profiles/"
        "AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json"
    ),
    "morphology_profile": (
        "Trajectory Generator/baseline_MLP/morphology_profiles/"
        "ab06_prosthetic_mean_std_corridor.json"
    ),
    "simulator_setup": (
        "models/AB06_SEASEA_Threadmill/"
        "AB06_SEASEA_stiff321_500_pi_setup.xml"
    ),
    "runtime_model": (
        "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim"
    ),
    "runtime_kinematics": (
        "models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot"
    ),
    "external_loads": (
        "models/AB06_SEASEA_Threadmill/data/ExternalForces.xml"
    ),
    "prescribed_grf_data": (
        "models/AB06_SEASEA_Threadmill/data/AB06_SEASEA_GRF_FullSpan.mot"
    ),
    "reserve_actuators": (
        "models/AB06_SEASEA_Threadmill/data/CMC_Actuators.xml"
    ),
    "macos_sea_plugin": (
        "plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib"
    ),
    "macos_online_grf_plugin": "plugins/libOnlineGRFContact.dylib",
    "v8r1p1_preflight": (
        "validation/h0_primary_grf_split_v8r1p1_"
        "teacher_replay_preflight_receipt.json"
    ),
    "v8r1p1_execution_lock": (
        "validation/h0_primary_grf_split_v8r1p1_"
        "teacher_replay_execution_lock.json"
    ),
    "v8r1_execution_lock": (
        "validation/h0_primary_grf_split_v8r1_"
        "teacher_replay_execution_lock.json"
    ),
}

for _base_case in FINAL_CASE_IDS:
    for _artifact_name in ("trace", "summary", "gate", "receipt"):
        INPUT_RELATIVE_PATHS[
            f"base_{_base_case}_{_artifact_name}"
        ] = (
            BASE_CORPUS_ROOT / _base_case / f"{_artifact_name}.json"
        ).as_posix()

AUTHORITY = {
    "authority_date": REVISION,
    "authority_text": "ok allora procedi con la risoluzione",
    "fresh_v10s_development_lineage_authorized": True,
    "source_h0_required_for_every_refit": True,
    "full_mean_actor_only": True,
    "logstd_update_authorized": False,
    "critic_updates_authorized": False,
    "ppo_updates_authorized": False,
    "three_collection_rounds_only": True,
    "round_alphas": [ROUND_ALPHAS[index] for index in sorted(ROUND_ALPHAS)],
    "safe_latch_is_diagnostic_only": True,
    "physical_gate_relaxation_authorized": False,
    "final_teacher_query_authorized": False,
    "final_blending_authorized": False,
    "final_safety_latch_authorized": False,
    "retry_authorized": False,
    "sweep_authorized": False,
    "retuning_authorized": False,
    "rescue_authorized": False,
    "protected_trial_access_authorized": False,
    "reserve_trial_access_authorized": False,
    "runtime_promotion_authorized": False,
    "primary_grf_modification_authorized": False,
    "detector_or_fsm_modification_authorized": False,
    "sea_semantic_modification_authorized": False,
}

V10A_EVIDENCE_SCHEMA_VERSION = 94
V10A_EVIDENCE_STATUS = "PASS_H0_PRIMARY_SPLIT_V10_TIME_ALIGNMENT_ADJUDICATION"
V10A_EVIDENCE_SCOPE = "OFFLINE_TIME_ALIGNMENT_ONLY_NO_RERUN"
V10A_EVIDENCE_NEXT_STAGE = (
    "V10_COHERENT_TEACHER_EVIDENCE_READY_FOR_FRESH_ADAPTATION_PROTOCOL"
)


def _finite_number(value: Any) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def _nonnegative_int(value: Any) -> bool:
    return type(value) is int and value >= 0


def _canonical_relative_path(value: Any) -> str | None:
    if isinstance(value, PurePosixPath):
        raw = value.as_posix()
    elif isinstance(value, str):
        raw = value
    else:
        return None
    pure = PurePosixPath(raw)
    if (
        not raw
        or raw == "."
        or pure.is_absolute()
        or ".." in pure.parts
        or pure.as_posix() != raw
    ):
        return None
    return raw


def _sha256_is_canonical(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def artifact_record_matches(
    record: Any,
    expected_path: str | PurePosixPath,
) -> bool:
    """Validate one self-contained artifact binding without filesystem I/O."""

    canonical_path = _canonical_relative_path(expected_path)
    return (
        canonical_path is not None
        and isinstance(record, Mapping)
        and set(record) == {"path", "sha256", "size_bytes"}
        and record.get("path") == canonical_path
        and _sha256_is_canonical(record.get("sha256"))
        and type(record.get("size_bytes")) is int
        and record["size_bytes"] > 0
    )


def tree_record_matches(
    record: Any,
    expected_path: str | PurePosixPath,
) -> bool:
    """Validate and recompute one deterministic artifact-tree binding."""

    canonical_root = _canonical_relative_path(expected_path)
    if (
        canonical_root is None
        or not isinstance(record, Mapping)
        or set(record) != {"path", "tree_sha256", "file_count", "files"}
        or record.get("path") != canonical_root
        or not _sha256_is_canonical(record.get("tree_sha256"))
        or type(record.get("file_count")) is not int
        or record["file_count"] <= 0
        or not isinstance(record.get("files"), list)
        or len(record["files"]) != record["file_count"]
    ):
        return False

    digest = hashlib.sha256()
    paths: list[str] = []
    for file_record in record["files"]:
        if (
            not isinstance(file_record, Mapping)
            or set(file_record) != {"path", "sha256", "size_bytes"}
        ):
            return False
        relative = _canonical_relative_path(file_record.get("path"))
        sha256 = file_record.get("sha256")
        size = file_record.get("size_bytes")
        if (
            relative is None
            or not _sha256_is_canonical(sha256)
            or type(size) is not int
            or size < 0
        ):
            return False
        paths.append(relative)
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(sha256.encode("ascii"))
        digest.update(b"\0")
        digest.update(str(size).encode("ascii"))
        digest.update(b"\n")

    return (
        paths == sorted(paths)
        and len(paths) == len(set(paths))
        and digest.hexdigest() == record["tree_sha256"]
    )


def _candidate_id_matches(summary: Mapping[str, Any]) -> bool:
    module = summary.get("candidate_module")
    return (
        tree_record_matches(module, MODULE_PATHS["p3"])
        and isinstance(summary.get("candidate_id"), str)
        and summary["candidate_id"]
        == candidate_id_prefix("p3") + module["tree_sha256"][:16]
    )


def _fit_receipt_bindings_match(value: Any) -> bool:
    if not isinstance(value, list) or len(value) != len(FIT_STAGES):
        return False
    for binding, stage in zip(value, FIT_STAGES, strict=True):
        if (
            not isinstance(binding, Mapping)
            or set(binding) != {"fit_stage", "receipt"}
            or binding.get("fit_stage") != stage
            or not artifact_record_matches(
                binding.get("receipt"), FIT_RECEIPT_PATHS[stage]
            )
        ):
            return False
    return True


def _collection_receipt_bindings_match(value: Any) -> bool:
    expected = [
        (round_index, case_id)
        for round_index in (1, 2, 3)
        for case_id in COLLECTION_CASE_IDS
    ]
    if not isinstance(value, list) or len(value) != len(expected):
        return False
    for binding, (round_index, case_id) in zip(value, expected, strict=True):
        stage_id = f"collect_r{round_index}__{case_id}"
        if (
            not isinstance(binding, Mapping)
            or set(binding) != {"round_index", "case_id", "receipt"}
            or binding.get("round_index") != round_index
            or binding.get("case_id") != case_id
            or not artifact_record_matches(
                binding.get("receipt"), stage_receipt_path(stage_id)
            )
        ):
            return False
    return True


def _final_rollout_bindings_match(value: Any) -> bool:
    if not isinstance(value, list) or len(value) != len(FINAL_CASE_IDS):
        return False
    for binding, case_id in zip(value, FINAL_CASE_IDS, strict=True):
        destination = PurePosixPath(canonical_final_case(case_id)["destination"])
        if (
            not isinstance(binding, Mapping)
            or set(binding) != {"case_id", "passed", "receipt", "gate", "trace"}
            or binding.get("case_id") != case_id
            or binding.get("passed") is not True
            or not artifact_record_matches(
                binding.get("receipt"), destination / "receipt.json"
            )
            or not artifact_record_matches(
                binding.get("gate"), destination / "gate.json"
            )
            or not artifact_record_matches(
                binding.get("trace"), destination / "trace.json"
            )
        ):
            return False
    return True


def canonical_collection_case(case_id: str, round_index: int) -> dict[str, Any]:
    """Return one detached collection case with its preregistered alpha."""

    if type(round_index) is not int or round_index not in ROUND_ALPHAS:
        raise ValueError(f"unknown V10S collection round: {round_index!r}")
    matches = [case for case in COLLECTION_CASES if case["case_id"] == case_id]
    if len(matches) != 1:
        raise ValueError(f"unknown V10S collection case: {case_id!r}")
    result = copy.deepcopy(matches[0])
    result.update(
        {
            "round_index": round_index,
            "requested_alpha": ROUND_ALPHAS[round_index],
            "candidate_fit_stage": f"p{round_index - 1}",
            "destination": (
                COLLECTION_ROOT / f"round_{round_index}" / case_id
            ).as_posix(),
        }
    )
    return result


def canonical_final_case(case_id: str) -> dict[str, Any]:
    matches = [case for case in FINAL_CASES if case["case_id"] == case_id]
    if len(matches) != 1:
        raise ValueError(f"unknown V10S final case: {case_id!r}")
    return copy.deepcopy(matches[0])


def expected_fit_counts(stage: str) -> dict[str, Any]:
    if stage not in FIT_STAGES:
        raise ValueError(f"unknown V10S fit stage: {stage!r}")
    rounds = FIT_COMPLETED_ROUNDS[stage]
    dagger_samples = len(rounds) * COLLECTION_SAMPLES_PER_ROUND
    return {
        "base_sample_count": BASE_CORPUS_SAMPLE_COUNT,
        "dagger_sample_count": dagger_samples,
        "sample_count": BASE_CORPUS_SAMPLE_COUNT + dagger_samples,
        "reset_row_count": BASE_CORPUS_CASE_COUNT
        + len(rounds) * COLLECTION_CASE_COUNT_PER_ROUND,
        "completed_collection_rounds": list(rounds),
    }


def candidate_id_prefix(stage: str) -> str:
    if stage not in FIT_STAGES:
        raise ValueError(f"unknown V10S fit stage: {stage!r}")
    return f"H0_PRIMARY_SPLIT_V10S_{stage.upper()}_"


def teacher_evidence_gate(receipt: Mapping[str, Any]) -> dict[str, Any]:
    """Validate the narrow V10A adjudication used to authorize new labels."""

    evidence = receipt.get("scientific_evidence")
    counterfactual = receipt.get("counterfactual_gate")
    original = receipt.get("original_protocol")
    checks = {
        "schema": receipt.get("schema_version") == V10A_EVIDENCE_SCHEMA_VERSION,
        "status": receipt.get("status") == V10A_EVIDENCE_STATUS,
        "passed": receipt.get("passed") is True,
        "scope": receipt.get("scope") == V10A_EVIDENCE_SCOPE,
        "next_stage": receipt.get("next_stage") == V10A_EVIDENCE_NEXT_STAGE,
        "coherent_teacher": isinstance(evidence, Mapping)
        and evidence.get("coherent_teacher_evidence_accepted") is True,
        "teacher_rows": isinstance(evidence, Mapping)
        and evidence.get("teacher_action_byte_exact_count") == EXPECTED_STEPS
        and evidence.get("teacher_mean_byte_exact_count") == EXPECTED_STEPS
        and evidence.get("teacher_view_byte_exact_count") == EXPECTED_STEPS,
        "counterfactual_pass": isinstance(counterfactual, Mapping)
        and counterfactual.get("passed") is True,
        "frozen_fail_preserved": isinstance(original, Mapping)
        and original.get("preserved_as_fail") is True
        and original.get("retry_authorized") is False,
        "offline_only": receipt.get("rollout_rerun_count") == 0
        and receipt.get("candidate_created") is False,
        "zero_updates": receipt.get("actor_updates") == 0
        and receipt.get("critic_updates") == 0
        and receipt.get("ppo_updates") == 0,
        "protected_closed": receipt.get("protected_trials_opened") == [],
        "reserve_closed": receipt.get("reserve_trials_opened") == [],
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": (
            "PASS_H0_PRIMARY_SPLIT_V10S_TEACHER_EVIDENCE"
            if passed
            else "FAIL_H0_PRIMARY_SPLIT_V10S_TEACHER_EVIDENCE"
        ),
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "teacher_evidence_id": TEACHER_EVIDENCE_ID,
        "checks": checks,
    }


def stage_descriptor(stage_id: str) -> dict[str, Any]:
    """Parse a canonical stage ID without accepting aliases."""

    if stage_id not in STAGE_IDS:
        raise ValueError(f"unknown V10S pipeline stage: {stage_id!r}")
    if stage_id.startswith("fit_"):
        stage = stage_id.removeprefix("fit_")
        return {"kind": "fit", "fit_stage": stage}
    if stage_id.startswith("collect_r"):
        prefix, case_id = stage_id.split("__", 1)
        round_index = int(prefix.removeprefix("collect_r"))
        return {
            "kind": "collection",
            "round_index": round_index,
            "case": canonical_collection_case(case_id, round_index),
        }
    if stage_id == "freeze_p3":
        return {"kind": "freeze", "fit_stage": "p3"}
    if stage_id.startswith("final__"):
        case_id = stage_id.removeprefix("final__")
        return {"kind": "final", "case": canonical_final_case(case_id)}
    return {"kind": "finalize"}


def worker_claim_path(stage_id: str) -> PurePosixPath:
    if stage_id not in STAGE_IDS:
        raise ValueError(f"unknown V10S pipeline stage: {stage_id!r}")
    return WORKER_CLAIMS_ROOT / f"{STAGE_IDS.index(stage_id) + 1:02d}_{stage_id}.json"


def stage_receipt_path(stage_id: str) -> PurePosixPath:
    descriptor = stage_descriptor(stage_id)
    kind = descriptor["kind"]
    if kind == "fit":
        return FIT_RECEIPT_PATHS[descriptor["fit_stage"]]
    if kind == "collection":
        return PurePosixPath(descriptor["case"]["destination"]) / "receipt.json"
    if kind == "freeze":
        return CANDIDATE_FREEZE_PATH
    if kind == "final":
        return PurePosixPath(descriptor["case"]["destination"]) / "receipt.json"
    return FINAL_DEVELOPMENT_RECEIPT_PATH


def fit_gate(summary: Mapping[str, Any], *, stage: str) -> dict[str, Any]:
    """Accept one independent full-mean H0 refit on the exact cumulative set."""

    expected = expected_fit_counts(stage)
    metrics = summary.get("metrics")
    report_checks = summary.get("report_checks")
    corpus_audit = summary.get("corpus_audit")
    expected_dagger_receipts = (
        len(expected["completed_collection_rounds"])
        * COLLECTION_CASE_COUNT_PER_ROUND
    )
    checks = {
        "schema": summary.get("schema_version") == SCHEMA_VERSION,
        "protocol": summary.get("protocol_id") == PROTOCOL_ID,
        "stage": summary.get("fit_stage") == stage,
        "fit_exact": summary.get("fit") == FIT,
        "source_h0": summary.get("source_h0_id") == SOURCE_H0_ID,
        "restart_from_h0": summary.get("initial_checkpoint_id") == SOURCE_H0_ID
        and summary.get("continued_from_previous_candidate") is False,
        "full_mean": summary.get("trainable_scope") == TRAINABLE_SCOPE,
        "event_contract": summary.get("event_contract_id") == EVENT_CONTRACT_ID,
        "teacher_evidence": summary.get("teacher_evidence_id")
        == TEACHER_EVIDENCE_ID
        and summary.get("teacher_evidence_passed") is True,
        "base_cases": summary.get("base_corpus_case_ids")
        == list(FINAL_CASE_IDS),
        "cumulative_rounds": summary.get("completed_collection_rounds")
        == expected["completed_collection_rounds"],
        "sample_count": summary.get("sample_count") == expected["sample_count"],
        "base_samples": summary.get("base_sample_count")
        == expected["base_sample_count"],
        "dagger_samples": summary.get("dagger_sample_count")
        == expected["dagger_sample_count"],
        "dagger_receipts": summary.get("dagger_receipt_count")
        == expected_dagger_receipts,
        "reset_rows": summary.get("reset_row_count")
        == expected["reset_row_count"],
        "no_duplicates": summary.get("duplicate_sample_count") == 0,
        "report_checks": isinstance(report_checks, Mapping)
        and bool(report_checks)
        and all(value is True for value in report_checks.values()),
        "corpus_audit": isinstance(corpus_audit, Mapping)
        and corpus_audit.get("failed_v9_rows_used") == 0
        and corpus_audit.get("dagger_sample_count")
        == expected["dagger_sample_count"]
        and corpus_audit.get("same_state_dagger_sample_count")
        == expected["dagger_sample_count"],
        "metrics_object": isinstance(metrics, Mapping),
        "rmse": isinstance(metrics, Mapping)
        and _finite_number(metrics.get("rmse"))
        and float(metrics["rmse"]) <= OFFLINE_THRESHOLDS["rmse_max"],
        "max_abs": isinstance(metrics, Mapping)
        and _finite_number(metrics.get("max_abs_error"))
        and float(metrics["max_abs_error"])
        <= OFFLINE_THRESHOLDS["max_abs_error_max"],
        "reset_max_abs": isinstance(metrics, Mapping)
        and _finite_number(metrics.get("reset_max_abs_error"))
        and float(metrics["reset_max_abs_error"])
        <= OFFLINE_THRESHOLDS["reset_max_abs_error_max"],
        "all_finite": summary.get("all_finite") is True,
        "source_immutable": summary.get("source_h0_byte_exact") is True,
        "critic_immutable": summary.get("critic_byte_exact") is True,
        "logstd_immutable": summary.get("logstd_byte_exact") is True,
        "one_actor_fit": summary.get("actor_updates") == 1,
        "zero_critic_updates": summary.get("critic_updates") == 0,
        "zero_ppo_updates": summary.get("ppo_updates") == 0,
        "protected_closed": summary.get("protected_trials_opened") == [],
        "reserve_closed": summary.get("reserve_trials_opened") == [],
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": FIT_PASS_STATUS if passed else FIT_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "fit_stage": stage,
        "checks": checks,
        "offline_threshold_provenance": copy.deepcopy(
            OFFLINE_THRESHOLD_PROVENANCE
        ),
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def _condition_checks(
    summary: Mapping[str, Any],
    expected: Mapping[str, Any],
) -> dict[str, bool]:
    selection = expected["action_selection"]
    expected_draws = EXPECTED_STEPS if selection == "stochastic" else 0
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
        "routing_failure_count",
        "step_contract_failure_count",
        "binary_event_failure_count",
    )
    return {
        "condition_exact": summary.get("action_selection") == selection
        and summary.get("episode_start_offset_s")
        == expected["episode_start_offset_s"]
        and summary.get("action_seed") == expected["action_seed"]
        and summary.get("runtime_seed") == expected["runtime_seed"]
        and summary.get("sigma") == expected["sigma"],
        "steps": summary.get("steps") == EXPECTED_STEPS,
        "control_windows": summary.get("control_window_count")
        == EXPECTED_CONTROL_WINDOWS,
        "raw_samples": summary.get("raw_sensor_sample_count")
        == EXPECTED_RAW_SENSOR_SAMPLES,
        "normal_terminal": summary.get("end_reason") == "episode_time_limit"
        and summary.get("terminated") is False
        and summary.get("truncated") is True,
        "cycles": _nonnegative_int(summary.get("phase_valid_cycle_count"))
        and summary["phase_valid_cycle_count"] >= MINIMUM_VALID_CYCLES,
        "penetration": _finite_number(summary.get("grf_penetration_max_m"))
        and float(summary["grf_penetration_max_m"]) < PENETRATION_LIMIT_M,
        "zero_invalids": all(
            _nonnegative_int(summary.get(field)) and summary[field] == 0
            for field in zero_fields
        ),
        "layout": summary.get("n_actor") == EXPECTED_ACTOR_FEATURES
        and summary.get("n_observation") == EXPECTED_FULL_FEATURES
        and summary.get("observation_dtype") == EXPECTED_DTYPE,
        "v26_active": summary.get("binary_phase_fsm_mode") == "binary_active"
        and summary.get("event_contract_id") == EVENT_CONTRACT_ID,
        "morphology_zero": _finite_number(summary.get("morphology_weight"))
        and float(summary["morphology_weight"]) == MORPHOLOGY_WEIGHT,
        "noise_draw_count": summary.get("random_noise_draw_count")
        == expected_draws,
        "single_noise_application": summary.get(
            "single_noise_application_count"
        )
        == EXPECTED_STEPS,
        "no_updates": summary.get("actor_updates") == 0
        and summary.get("critic_updates") == 0
        and summary.get("ppo_updates") == 0,
        "protected_closed": summary.get("protected_trials_opened") == [],
        "reserve_closed": summary.get("reserve_trials_opened") == [],
    }


def collection_gate(
    summary: Mapping[str, Any],
    *,
    round_index: int,
) -> dict[str, Any]:
    """Accept a complete collection rollout; interventions are diagnostic."""

    case_id = summary.get("case_id")
    try:
        expected = canonical_collection_case(str(case_id), round_index)
    except ValueError:
        expected = None
    physical = _condition_checks(summary, expected) if expected else {}
    nonnegative_diagnostics = (
        "safety_latch_activation_count",
        "safety_latch_release_count",
        "safety_intervention_count",
    )
    checks = {
        "schema": summary.get("schema_version") == SCHEMA_VERSION,
        "ungated_status": summary.get("status") == COLLECTION_STATUS,
        "protocol": summary.get("protocol_id") == PROTOCOL_ID,
        "known_case": expected is not None,
        "round": summary.get("round_index") == round_index,
        "requested_alpha": expected is not None
        and summary.get("requested_alpha") == expected["requested_alpha"],
        "candidate_stage": expected is not None
        and summary.get("candidate_fit_stage")
        == expected["candidate_fit_stage"],
        "behavior": summary.get("behavior") == COLLECTION_BEHAVIOR,
        "teacher": summary.get("teacher_id") == TEACHER_ID
        and summary.get("teacher_evidence_id") == TEACHER_EVIDENCE_ID,
        "labels_complete": summary.get("sample_count") == EXPECTED_STEPS
        and summary.get("teacher_query_count") == EXPECTED_STEPS
        and summary.get("persisted_label_count") == EXPECTED_STEPS
        and summary.get("candidate_mean_query_count") == EXPECTED_STEPS
        and summary.get("same_state_teacher_label_count") == EXPECTED_STEPS
        and summary.get("candidate_selected_before_teacher_count")
        == EXPECTED_STEPS
        and summary.get("served_action_teacher_dependency_count")
        == EXPECTED_STEPS,
        "blend_then_noise": summary.get("mean_blend_count") == EXPECTED_STEPS
        and summary.get("blend_before_noise_count") == EXPECTED_STEPS
        and summary.get("noise_before_blend_count") == 0
        and summary.get("multiple_noise_application_count") == 0,
        "latch_contract": summary.get("safety_latch_activation_m")
        == SAFETY_LATCH_ACTIVATION_M
        and summary.get("safety_latch_release_m") == SAFETY_LATCH_RELEASE_M
        and summary.get("safety_latch_release_phase")
        == SAFETY_LATCH_RELEASE_PHASE
        and summary.get("safety_signal_lag_steps") == 1
        and summary.get("safety_intervention_diagnostic_only") is True,
        "latch_diagnostics": all(
            _nonnegative_int(summary.get(field))
            for field in nonnegative_diagnostics
        ),
        "latch_exact": summary.get("safety_latch_rule_violation_count") == 0
        and summary.get("alpha_mismatch_count") == 0
        and summary.get("mean_blend_mismatch_count") == 0
        and summary.get("noise_application_mismatch_count") == 0,
        "physical_not_bypassed": summary.get("physical_gate_bypass_count") == 0,
        **physical,
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": (
            COLLECTION_PASS_STATUS if passed else COLLECTION_FAIL_STATUS
        ),
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "round_index": round_index,
        "case_id": case_id,
        "checks": checks,
        "physical_penetration_limit_m": PENETRATION_LIMIT_M,
        "physical_gate_relaxed": False,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def candidate_freeze_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    checks = {
        "schema": summary.get("schema_version") == SCHEMA_VERSION,
        "protocol": summary.get("protocol_id") == PROTOCOL_ID,
        "stage": summary.get("candidate_fit_stage") == "p3",
        "candidate_module": tree_record_matches(
            summary.get("candidate_module"), MODULE_PATHS["p3"]
        ),
        "candidate_binding": _candidate_id_matches(summary),
        "fit_passed": summary.get("p3_fit_passed") is True,
        "fit_receipts": _fit_receipt_bindings_match(
            summary.get("fit_receipts")
        ),
        "collection_receipts": _collection_receipt_bindings_match(
            summary.get("collection_receipts")
        ),
        "source_h0": tree_record_matches(
            summary.get("source_h0"), SOURCE_H0_MODULE_PATH
        ),
        "teacher_evidence": artifact_record_matches(
            summary.get("teacher_evidence"), TEACHER_EVIDENCE_RECEIPT_PATH
        ),
        "target_contract": summary.get("target_contract_id")
        == TARGET_CONTRACT_ID,
        "four_independent_fits": summary.get("fit_actor_update_count")
        == len(FIT_STAGES)
        and summary.get("every_fit_restarted_from_h0") is True,
        "frozen": summary.get("candidate_frozen") is True,
        "logstd_frozen": summary.get("logstd_byte_exact") is True,
        "critic_frozen": summary.get("critic_byte_exact") is True,
        "no_updates": summary.get("actor_updates") == 0
        and summary.get("critic_updates") == 0
        and summary.get("ppo_updates") == 0,
        "protected_closed": summary.get("protected_trials_opened") == [],
        "reserve_closed": summary.get("reserve_trials_opened") == [],
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": FREEZE_PASS_STATUS if passed else FREEZE_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "checks": checks,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def final_rollout_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    """Require deployable V26 behavior with no teacher, blend, or latch."""

    case_id = summary.get("case_id")
    try:
        expected = canonical_final_case(str(case_id))
    except ValueError:
        expected = None
    physical = _condition_checks(summary, expected) if expected else {}
    checks = {
        "schema": summary.get("schema_version") == SCHEMA_VERSION,
        "ungated_status": summary.get("status") == FINAL_ROLLOUT_STATUS,
        "protocol": summary.get("protocol_id") == PROTOCOL_ID,
        "known_case": expected is not None,
        "behavior": summary.get("behavior") == FINAL_BEHAVIOR,
        "candidate_stage": summary.get("candidate_fit_stage") == "p3",
        "candidate": isinstance(summary.get("candidate_id"), str)
        and summary["candidate_id"].startswith(candidate_id_prefix("p3")),
        "candidate_queries": summary.get("candidate_mean_query_count")
        == EXPECTED_STEPS,
        "unblended": summary.get("blending_enabled") is False
        and summary.get("mean_blend_count") == 0,
        "no_teacher": summary.get("teacher_enabled") is False
        and summary.get("teacher_query_count") == 0
        and summary.get("served_action_teacher_dependency_count") == 0,
        "no_safety_latch": summary.get("safety_latch_enabled") is False
        and summary.get("safety_intervention_count") == 0
        and summary.get("safety_latch_activation_count") == 0,
        **physical,
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": (
            FINAL_ROLLOUT_PASS_STATUS if passed else FINAL_ROLLOUT_FAIL_STATUS
        ),
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "case_id": case_id,
        "candidate_id": summary.get("candidate_id"),
        "checks": checks,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def final_development_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    checks = {
        "schema": summary.get("schema_version") == SCHEMA_VERSION,
        "protocol": summary.get("protocol_id") == PROTOCOL_ID,
        "candidate_module": tree_record_matches(
            summary.get("candidate_module"), MODULE_PATHS["p3"]
        ),
        "candidate_binding": _candidate_id_matches(summary),
        "candidate_freeze_binding": artifact_record_matches(
            summary.get("candidate_freeze"), CANDIDATE_FREEZE_PATH
        ),
        "p3_fit_bindings": artifact_record_matches(
            summary.get("p3_fit_receipt"), FIT_RECEIPT_PATHS["p3"]
        )
        and artifact_record_matches(
            summary.get("p3_fit_gate"), FIT_ROOTS["p3"] / "gate.json"
        ),
        "cases_exact": summary.get("case_ids") == list(FINAL_CASE_IDS),
        "six_passes": summary.get("rollout_receipt_count") == len(FINAL_CASE_IDS)
        and summary.get("rollout_pass_count") == len(FINAL_CASE_IDS)
        and summary.get("all_rollouts_passed") is True,
        "rollout_bindings": _final_rollout_bindings_match(
            summary.get("rollout_bindings")
        ),
        "candidate_frozen": summary.get("candidate_fit_stage") == "p3"
        and summary.get("candidate_frozen_before_final") is True,
        "four_independent_fits": summary.get("fit_actor_update_count")
        == len(FIT_STAGES)
        and summary.get("every_fit_restarted_from_h0") is True,
        "candidate_queries": summary.get("final_candidate_mean_query_count")
        == len(FINAL_CASE_IDS) * EXPECTED_STEPS,
        "no_final_teacher": summary.get("final_teacher_query_count") == 0,
        "no_final_blend": summary.get("final_mean_blend_count") == 0,
        "no_final_latch": summary.get("final_safety_intervention_count") == 0,
        "zero_actor_updates": summary.get("actor_updates") == 0,
        "zero_critic_updates": summary.get("critic_updates") == 0,
        "zero_ppo_updates": summary.get("ppo_updates") == 0,
        "protected_closed": summary.get("protected_trials_opened") == [],
        "reserve_closed": summary.get("reserve_trials_opened") == [],
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": (
            FINAL_DEVELOPMENT_PASS_STATUS
            if passed
            else FINAL_DEVELOPMENT_FAIL_STATUS
        ),
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "checks": checks,
        "next_stage": (
            "V10Q_SEA_RESERVE_QUALIFICATION_REQUIRED"
            if passed
            else "STOP_V10S_TERMINAL_NO_RETRY_OR_SWEEP"
        ),
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


__all__ = [name for name in globals() if name.isupper()] + [
    "artifact_record_matches",
    "candidate_freeze_gate",
    "candidate_id_prefix",
    "canonical_collection_case",
    "canonical_final_case",
    "collection_gate",
    "expected_fit_counts",
    "final_development_gate",
    "final_rollout_gate",
    "fit_gate",
    "stage_descriptor",
    "stage_receipt_path",
    "teacher_evidence_gate",
    "tree_record_matches",
    "worker_claim_path",
]
