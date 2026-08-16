"""Pure contract for the additive V12R4 P3 coverage-recovery lineage.

V12R4 does not reopen the terminal V12R3 or P1-salvage executions.  It binds
their evidence and uses the unreceipted P2 module only as a non-promotable
student during four new, shielded data-collection episodes.  The sole P3 fit
starts again from immutable H0, consumes the exact P2 corpus plus the four new
same-state teacher-label episodes, and is followed by six frozen pure-policy
development cases.

This module is intentionally I/O free.  Importing it cannot publish a freeze,
claim an execution, fit an actor, load OpenSim, or reset/step an environment.
"""

from __future__ import annotations

import copy
import math
import sys
from pathlib import Path, PurePosixPath
from typing import Any, Mapping


_SOURCE_PATH = Path(__file__).resolve()
_LOCAL_VALIDATION_ROOT = _SOURCE_PATH.parent.parent
_REPO_ROOT = _SOURCE_PATH.parents[4]
for _IMPORT_ROOT in (
    _SOURCE_PATH.parent,
    _LOCAL_VALIDATION_ROOT / "v12r3",
    _LOCAL_VALIDATION_ROOT,
    _REPO_ROOT / "validation",
    _REPO_ROOT / "Trajectory Generator" / "baseline_MLP",
    _REPO_ROOT,
):
    if str(_IMPORT_ROOT) not in sys.path:
        sys.path.insert(0, str(_IMPORT_ROOT))

import h0_primary_split_v12r3_autonomy_recovery_contract as v12r3  # noqa: E402


SCHEMA_VERSION = 1240
REVISION = "2026-08-09"
PROTOCOL_ID = "AB06_H0_PRIMARY_SPLIT_V12R4_P3_COVERAGE_V26"
PIPELINE_ID = "H0_V12R4_P3_FOUR_CASE_COVERAGE_SAFE_DAGGER"
FIT_CONTRACT_ID = "h0_primary_split_v12r4_p3_coverage_full_mean_v1"
CANDIDATE_SELECTION_RULE = "SOLE_FIT_P3_OUTPUT_FROM_LOCKED_R4_RUN"
AUTHORITY_TEXT = "esegui i punti 1-6"

AUTHORITY = {
    "authority_date": REVISION,
    "authority_text": AUTHORITY_TEXT,
    "provenance_prior_authority_text": (
        "Fai tutti i passaggi che ti servono per arrivare ad essere training ready."
    ),
    "authority_scope": "V12R4_EXACT_ONE_SHOT_P3_DEVELOPMENT_EXECUTION",
    "source_implementation_authorized": True,
    "protocol_freeze_publication_authorized": True,
    "execution_lock_authorized": True,
    "collection_execution_authorized": True,
    "actor_fit_execution_authorized": True,
    "candidate_freeze_authorized": True,
    "development_execution_authorized": True,
    "qualification_execution_authorized": False,
    "retry_authorized": False,
    "resume_authorized": False,
    "rescue_authorized": False,
    "sweep_authorized": False,
    "detector_or_fsm_change_authorized": False,
    "runtime_promotion_authorized": False,
    "checkpoint_zero_authorized": False,
    "positive_morphology_authorized": False,
}

VALIDATION_ROOT = PurePosixPath("Trajectory Generator/baseline_MLP/validation/v12r4")
PROTOCOL_FREEZE_PATH = VALIDATION_ROOT / "h0_v12r4_p3_coverage_protocol_freeze.json"
DESIGN_AUDIT_PATH = VALIDATION_ROOT / "h0_v12r4_p3_coverage_design_audit.json"
EXECUTION_LOCK_PATH = VALIDATION_ROOT / "h0_v12r4_p3_coverage_execution_lock.json"
RUN_ROOT = VALIDATION_ROOT / "h0_v12r4_run_20260809"
PIPELINE_CLAIM_PATH = RUN_ROOT / "pipeline_claim.json"
PIPELINE_LEDGER_PATH = RUN_ROOT / "pipeline_ledger.json"
WORKER_CLAIMS_ROOT = RUN_ROOT / "worker_claims"
COLLECTION_ROOT = RUN_ROOT / "collect"
FIT_ROOT = RUN_ROOT / "fit_p3"
P3_CORPUS_PATH = FIT_ROOT / "corpus.npz"
P3_MODULE_PATH = FIT_ROOT / "rl_module_target_adapted"
CANDIDATE_FREEZE_PATH = RUN_ROOT / "candidate_freeze_receipt.json"
DEVELOPMENT_ROOT = RUN_ROOT / "development"
FINAL_DEVELOPMENT_RECEIPT_PATH = RUN_ROOT / "final_development_receipt.json"

# Q2 is a separate additive lineage.  Its source/design freeze may already be
# present, but protocol, lock, tapes, and execution root must remain unopened
# for the whole R4 execution.
Q2_DESIGN_FREEZE_PATH = PurePosixPath(
    "Trajectory Generator/baseline_MLP/validation/v12r4q2/"
    "h0_v12r4_q2_qualification_design_freeze.json"
)
Q2_DESIGN_FREEZE_ARTIFACT = {
    "path": Q2_DESIGN_FREEZE_PATH.as_posix(),
    "sha256": "d92fac765bd192f43ef2e0420a8529e8bb21860a3f47420ab5948863fb53eaf5",
    "size_bytes": 32_166,
}
Q2_UNOPENED_PATHS = {
    "protocol_freeze": PurePosixPath(
        "Trajectory Generator/baseline_MLP/validation/v12r4q2/"
        "h0_v12r4_q2_qualification_protocol_freeze.json"
    ),
    "execution_lock": PurePosixPath(
        "Trajectory Generator/baseline_MLP/validation/v12r4q2/"
        "h0_v12r4_q2_qualification_execution_lock.json"
    ),
    "run_root": PurePosixPath(
        "Trajectory Generator/baseline_MLP/validation/v12r4q2/"
        "h0_v12r4_q2_run_20260809"
    ),
    "noise_root": PurePosixPath(
        "Trajectory Generator/baseline_MLP/validation/v12r4q2/"
        "h0_v12r4_q2_qualification_noise_tapes"
    ),
}

P2_ROOT = PurePosixPath(
    "Trajectory Generator/baseline_MLP/validation/v12r3/" "h0_v12r3_run_20260809/fit/p2"
)
P2_CORPUS_ARTIFACT = {
    "path": (P2_ROOT / "corpus.npz").as_posix(),
    "sha256": "42a40869447aec0cdce62a6ba5fcb48da4e5f070fac9e8eb19c4f0a8fccc1990",
    "size_bytes": 18_358_010,
}
P2_ADAPTATION_REPORT_ARTIFACT = {
    "path": (P2_ROOT / "adaptation_report.json").as_posix(),
    "sha256": "776fb10eb10430953105f5e0b818a323449b43f43e70c6649dd7cb4294825324",
    "size_bytes": 10_097,
}
P2_ADAPTATION_HISTORY_ARTIFACT = {
    "path": (P2_ROOT / "adaptation_history.json").as_posix(),
    "sha256": "02341e3b20df60fa279f0a9e5b55b14a2e0c48dccc666e095b27e841c319b1e7",
    "size_bytes": 1_659,
}
P2_MODULE_TREE = {
    "path": (P2_ROOT / "rl_module_target_adapted").as_posix(),
    "tree_sha256": "be0c9711f1f6b7b9dba9cd4ab546a8379f96444ddf02a111d5f3002b9c7e8c4f",
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
            "sha256": "f6753be7ec55e5276c6be824b3a7550880f3ac4fa26a59f2a056a20f0a5ab3f6",
            "size_bytes": 604_772,
        },
    ],
}

V12R3_TERMINAL_LEDGER = {
    "path": (
        PurePosixPath(
            "Trajectory Generator/baseline_MLP/validation/v12r3/"
            "h0_v12r3_run_20260809/pipeline_ledger.json"
        ).as_posix()
    ),
    "sha256": "5961322102a6d3a159e472d23e9223ba053b7d632ed02200a6d6c763ec326ad8",
    "size_bytes": 10_856,
}
P1S_TERMINAL_LEDGER = {
    "path": (
        PurePosixPath(
            "Trajectory Generator/baseline_MLP/validation/v12p1s/"
            "h0_v12p1s_run_20260809/pipeline_ledger.json"
        ).as_posix()
    ),
    "sha256": "e0969d83ed36b4b7b10f50abde0a8f9d85c7d0aeb97a3318a42ded3e0aaa4b92",
    "size_bytes": 5_674,
}
P1S_NOMINAL_FAILURE = {
    "path": (
        PurePosixPath(
            "Trajectory Generator/baseline_MLP/validation/v12p1s/"
            "h0_v12p1s_run_20260809/development/"
            "deterministic_offset_nominal/failure.json"
        ).as_posix()
    ),
    "sha256": "cf0a4e61cb26e479de9eed2c61362256cbc56911dd0f8753282777e103233a80",
    "size_bytes": 56_099,
}

P2_DIAGNOSTIC_METRICS = {
    "p1_on_p2_corpus": {
        "rmse": 0.005957,
        "max_abs_error": 0.089472,
    },
    "p2_on_p2_corpus": {
        "rmse": 0.005370,
        "max_abs_error": 0.067215,
    },
    "p2_minus_p1_failure_prefix_max_abs": 0.012,
    "failure_prefix_rows": 179,
    "promotion_eligible": False,
    "purpose": "DESIGN_DIAGNOSTIC_ONLY_NOT_A_GATE_SUBSTITUTE",
}

SOURCE_H0_ID = v12r3.SOURCE_H0_ID
SOURCE_H0_MODULE_PATH = v12r3.SOURCE_H0_MODULE_PATH
SOURCE_H0_TREE_SHA256 = v12r3.SOURCE_H0_TREE_SHA256
TARGET_CONTRACT_ID = v12r3.TARGET_CONTRACT_ID
EVENT_CONTRACT_ID = v12r3.EVENT_CONTRACT_ID
TEACHER_ID = v12r3.TEACHER_ID
TEACHER_EVIDENCE_ID = v12r3.TEACHER_EVIDENCE_ID
TEACHER_EVIDENCE_ARTIFACT = copy.deepcopy(v12r3.TEACHER_EVIDENCE_ARTIFACT)
EXPECTED_STEPS = v12r3.EXPECTED_STEPS
EXPECTED_ACTOR_FEATURES = v12r3.EXPECTED_ACTOR_FEATURES
EXPECTED_FULL_FEATURES = v12r3.EXPECTED_FULL_FEATURES
EXPECTED_ACTION_DIM = v12r3.EXPECTED_ACTION_DIM
EXPECTED_DTYPE = v12r3.EXPECTED_DTYPE
EXPECTED_CONTROL_WINDOWS = v12r3.EXPECTED_CONTROL_WINDOWS
EXPECTED_RAW_SENSOR_SAMPLES = v12r3.EXPECTED_RAW_SENSOR_SAMPLES
EXPECTED_SIGMA = 0.005
MORPHOLOGY_WEIGHT = v12r3.MORPHOLOGY_WEIGHT
PENETRATION_LIMIT_M = v12r3.PENETRATION_LIMIT_M
MINIMUM_VALID_CYCLES = v12r3.MINIMUM_VALID_CYCLES
OFFLINE_THRESHOLDS = copy.deepcopy(v12r3.OFFLINE_THRESHOLDS)
ACTOR_ARCHITECTURE = copy.deepcopy(v12r3.ACTOR_ARCHITECTURE)
BASE_CORPUS_NORMALIZATION = copy.deepcopy(v12r3.BASE_CORPUS_NORMALIZATION)
RECOVERY_WEIGHTING = copy.deepcopy(v12r3.RECOVERY_WEIGHTING)
COVERAGE_WEIGHTING = copy.deepcopy(v12r3.COVERAGE_WEIGHTING)

P2_CORPUS_ROWS = 8_732
P2_EPISODE_COUNT = 18
P2_RESET_ROWS = 18
NEW_COLLECTION_ROWS_PER_CASE = 500
NEW_COLLECTION_CASE_COUNT = 4
NEW_COLLECTION_ROWS = NEW_COLLECTION_ROWS_PER_CASE * NEW_COLLECTION_CASE_COUNT
P3_CORPUS_ROWS = P2_CORPUS_ROWS + NEW_COLLECTION_ROWS
P3_EPISODE_COUNT = P2_EPISODE_COUNT + NEW_COLLECTION_CASE_COUNT
P3_RESET_ROWS = P2_RESET_ROWS + NEW_COLLECTION_CASE_COUNT
EPISODE_TARGET_MASS = 500.0
P3_NORMALIZED_TOTAL_MASS = P3_EPISODE_COUNT * EPISODE_TARGET_MASS

P2_STUDENT_WEIGHT = 0.50
SAFETY_LATCH_ACTIVATION_M = 0.015
SAFETY_LATCH_RELEASE_M = 0.010
SAFETY_LATCH_RELEASE_PHASE = "SWING_AFTER_TO"

_BASE_START_S = 1.956870983805102

COLLECTION_CASES = (
    {
        "case_id": "deterministic_offset_nominal",
        "action_selection": "deterministic",
        "episode_start_offset_s": _BASE_START_S,
        "action_seed": None,
        "runtime_seed": 123,
        "sigma": 0.0,
        "requested_alpha": P2_STUDENT_WEIGHT,
    },
    {
        "case_id": "deterministic_offset_plus_0p20",
        "action_selection": "deterministic",
        "episode_start_offset_s": _BASE_START_S + 0.20,
        "action_seed": None,
        "runtime_seed": 123,
        "sigma": 0.0,
        "requested_alpha": P2_STUDENT_WEIGHT,
    },
    {
        "case_id": "stochastic_nominal_seed_127",
        "action_selection": "stochastic",
        "episode_start_offset_s": _BASE_START_S,
        "action_seed": 127,
        "runtime_seed": 127,
        "sigma": EXPECTED_SIGMA,
        "requested_alpha": P2_STUDENT_WEIGHT,
    },
    {
        "case_id": "stochastic_nominal_seed_128",
        "action_selection": "stochastic",
        "episode_start_offset_s": _BASE_START_S,
        "action_seed": 128,
        "runtime_seed": 128,
        "sigma": EXPECTED_SIGMA,
        "requested_alpha": P2_STUDENT_WEIGHT,
    },
)
COLLECTION_CASE_IDS = tuple(case["case_id"] for case in COLLECTION_CASES)

DEVELOPMENT_CASES = (
    {
        "case_id": "deterministic_offset_nominal",
        "action_selection": "deterministic",
        "episode_start_offset_s": _BASE_START_S,
        "action_seed": None,
        "runtime_seed": 123,
        "sigma": 0.0,
    },
    {
        "case_id": "deterministic_offset_minus_0p20",
        "action_selection": "deterministic",
        "episode_start_offset_s": _BASE_START_S - 0.20,
        "action_seed": None,
        "runtime_seed": 123,
        "sigma": 0.0,
    },
    {
        "case_id": "deterministic_offset_plus_0p20",
        "action_selection": "deterministic",
        "episode_start_offset_s": _BASE_START_S + 0.20,
        "action_seed": None,
        "runtime_seed": 123,
        "sigma": 0.0,
    },
    {
        "case_id": "stochastic_nominal_seed_126",
        "action_selection": "stochastic",
        "episode_start_offset_s": _BASE_START_S,
        "action_seed": 126,
        "runtime_seed": 126,
        "sigma": EXPECTED_SIGMA,
    },
    {
        "case_id": "stochastic_nominal_seed_127",
        "action_selection": "stochastic",
        "episode_start_offset_s": _BASE_START_S,
        "action_seed": 127,
        "runtime_seed": 127,
        "sigma": EXPECTED_SIGMA,
    },
    {
        "case_id": "stochastic_nominal_seed_128",
        "action_selection": "stochastic",
        "episode_start_offset_s": _BASE_START_S,
        "action_seed": 128,
        "runtime_seed": 128,
        "sigma": EXPECTED_SIGMA,
    },
)
DEVELOPMENT_CASE_IDS = tuple(case["case_id"] for case in DEVELOPMENT_CASES)

COLLECTION_PATHS = {
    case_id: COLLECTION_ROOT / case_id for case_id in COLLECTION_CASE_IDS
}
DEVELOPMENT_PATHS = {
    case_id: DEVELOPMENT_ROOT / case_id for case_id in DEVELOPMENT_CASE_IDS
}

FIT = {
    "fit_contract_id": FIT_CONTRACT_ID,
    "initial_checkpoint_id": SOURCE_H0_ID,
    "continued_from_p2": False,
    "actor_architecture": copy.deepcopy(ACTOR_ARCHITECTURE),
    "normalization": copy.deepcopy(BASE_CORPUS_NORMALIZATION),
    "trainable_scope": "full_mean_network",
    "freeze_logstd_head": True,
    "disabled_clock_columns": [0, 1],
    "adamw": {
        "optimizer": "AdamW",
        "seed": 20260807,
        "full_batch": True,
        "epochs": 3000,
        "learning_rate_schedule": [
            {"start_epoch": 1, "end_epoch": 1500, "learning_rate": 3.0e-4},
            {"start_epoch": 1501, "end_epoch": 2500, "learning_rate": 1.0e-4},
            {"start_epoch": 2501, "end_epoch": 3000, "learning_rate": 3.0e-5},
        ],
        "weight_decay": 1.0e-7,
        "grad_clip_norm": 10.0,
    },
    "lbfgs": {
        "optimizer": "LBFGS",
        "deterministic": True,
        "lr": 0.7,
        "max_iter": 600,
        "max_eval": 1200,
        "tolerance_grad": 1.0e-10,
        "tolerance_change": 1.0e-12,
        "history_size": 50,
        "line_search_fn": "strong_wolfe",
    },
    "optimizer_phase_order": ["adamw", "lbfgs"],
    "row_loss": "MEAN_SQUARED_ERROR_OVER_TWO_ACTIONS",
    "corpus_loss_reduction": "SUM_WEIGHTED_ROW_LOSS_DIVIDED_BY_SUM_WEIGHTS",
    "anchor_enabled": False,
    "hard_polish_enabled": False,
    "fallback_enabled": False,
    "sweep_enabled": False,
}

STAGE_IDS = (
    "attest_p2_collection_source",
    *(f"collect_cov__{case_id}" for case_id in COLLECTION_CASE_IDS),
    "assemble_corpus_p3",
    "fit_p3",
    "freeze_candidate_p3",
    *(f"development__{case_id}" for case_id in DEVELOPMENT_CASE_IDS),
    "finalize_development",
)

PROTOCOL_FREEZE_STATUS = "PASS_H0_V12R4_P3_COVERAGE_PROTOCOL_FREEZE"
EXECUTION_LOCK_STATUS = "PASS_H0_V12R4_P3_COVERAGE_EXECUTION_LOCK"
SOURCE_ATTEST_PASS_STATUS = "PASS_H0_V12R4_P2_COLLECTION_SOURCE_ATTESTATION"
COLLECTION_COMPLETE_STATUS = "COMPLETE_H0_V12R4_SHIELDED_COLLECTION"
COLLECTION_PASS_STATUS = "PASS_H0_V12R4_SHIELDED_COLLECTION_DATA"
FIT_COMPLETE_STATUS = "COMPLETE_H0_V12R4_P3_FIT"
FIT_PASS_STATUS = "PASS_H0_V12R4_P3_FIT"
CANDIDATE_FREEZE_COMPLETE_STATUS = "COMPLETE_H0_V12R4_P3_CANDIDATE_FREEZE"
CANDIDATE_FREEZE_STATUS = "PASS_H0_V12R4_P3_CANDIDATE_FREEZE"
DEVELOPMENT_COMPLETE_STATUS = "COMPLETE_H0_V12R4_P3_DEVELOPMENT_CASE"
DEVELOPMENT_PASS_STATUS = "PASS_H0_V12R4_P3_DEVELOPMENT_CASE"
FINAL_DEVELOPMENT_PASS_STATUS = "PASS_H0_V12R4_P3_DEVELOPMENT"
PIPELINE_PASS_STATUS = "PASS_H0_V12R4_P3_COVERAGE_PIPELINE_TERMINAL"
TERMINAL_FAIL_STATUS = "FAIL_H0_V12R4_P3_COVERAGE_TERMINAL"

Q2_PREREQUISITES = {
    "protocol_freeze": {
        "path": PROTOCOL_FREEZE_PATH.as_posix(),
        "status": PROTOCOL_FREEZE_STATUS,
    },
    "execution_lock": {
        "path": EXECUTION_LOCK_PATH.as_posix(),
        "status": EXECUTION_LOCK_STATUS,
    },
    "candidate_freeze": {
        "path": CANDIDATE_FREEZE_PATH.as_posix(),
        "status": CANDIDATE_FREEZE_STATUS,
    },
    "final_development_receipt": {
        "path": FINAL_DEVELOPMENT_RECEIPT_PATH.as_posix(),
        "status": FINAL_DEVELOPMENT_PASS_STATUS,
    },
    "terminal_ledger": {
        "path": PIPELINE_LEDGER_PATH.as_posix(),
        "status": PIPELINE_PASS_STATUS,
    },
}


def canonical_collection_case(case_id: str) -> dict[str, Any]:
    matches = [case for case in COLLECTION_CASES if case["case_id"] == case_id]
    if len(matches) != 1:
        raise ValueError(f"unknown V12R4 collection case: {case_id!r}")
    return {
        **copy.deepcopy(matches[0]),
        "destination": COLLECTION_PATHS[case_id].as_posix(),
        "candidate_role": "P2_COLLECTION_STUDENT_ONLY_NONPROMOTABLE",
    }


def candidate_id(tree_sha256: str) -> str:
    if not isinstance(tree_sha256, str) or len(tree_sha256) != 64:
        raise ValueError("candidate tree hash must be a SHA-256 hex digest")
    try:
        int(tree_sha256, 16)
    except ValueError as exc:
        raise ValueError("candidate tree hash must be hexadecimal") from exc
    return f"h0_v12r4_p3::{tree_sha256}"


def canonical_development_case(case_id: str) -> dict[str, Any]:
    matches = [case for case in DEVELOPMENT_CASES if case["case_id"] == case_id]
    if len(matches) != 1:
        raise ValueError(f"unknown V12R4 development case: {case_id!r}")
    return {
        **copy.deepcopy(matches[0]),
        "destination": DEVELOPMENT_PATHS[case_id].as_posix(),
        "behavior": "P3_PURE_UNBLENDED_NO_TEACHER_NO_LATCH",
    }


def stage_descriptor(stage_id: str) -> dict[str, Any]:
    if stage_id not in STAGE_IDS:
        raise ValueError(f"unknown V12R4 stage: {stage_id!r}")
    if stage_id == "attest_p2_collection_source":
        return {"stage_id": stage_id, "kind": "attestation"}
    if stage_id.startswith("collect_cov__"):
        case_id = stage_id.removeprefix("collect_cov__")
        return {
            "stage_id": stage_id,
            "kind": "collection",
            "case": canonical_collection_case(case_id),
        }
    if stage_id == "assemble_corpus_p3":
        return {"stage_id": stage_id, "kind": "corpus"}
    if stage_id == "fit_p3":
        return {"stage_id": stage_id, "kind": "fit"}
    if stage_id == "freeze_candidate_p3":
        return {"stage_id": stage_id, "kind": "candidate_freeze"}
    if stage_id.startswith("development__"):
        case_id = stage_id.removeprefix("development__")
        return {
            "stage_id": stage_id,
            "kind": "development",
            "case": canonical_development_case(case_id),
        }
    return {"stage_id": stage_id, "kind": "finalize"}


def expected_corpus_counts() -> dict[str, Any]:
    return {
        "p2_sample_count": P2_CORPUS_ROWS,
        "new_collection_sample_count": NEW_COLLECTION_ROWS,
        "new_collection_case_count": NEW_COLLECTION_CASE_COUNT,
        "sample_count": P3_CORPUS_ROWS,
        "episode_count": P3_EPISODE_COUNT,
        "reset_row_count": P3_RESET_ROWS,
        "episode_target_mass": EPISODE_TARGET_MASS,
        "normalized_total_sample_mass": P3_NORMALIZED_TOTAL_MASS,
        "component_order": ["p2_corpus", *COLLECTION_CASE_IDS],
    }


def _finite_number(value: Any) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def _metric_triplet_within_gate(value: Any) -> bool:
    if not isinstance(value, Mapping):
        return False
    names = ("rmse", "max_abs_error", "reset_max_abs_error")
    if not all(_finite_number(value.get(name)) for name in names):
        return False
    return (
        0.0 <= float(value["rmse"]) <= OFFLINE_THRESHOLDS["rmse_max"]
        and 0.0
        <= float(value["max_abs_error"])
        <= OFFLINE_THRESHOLDS["max_abs_error_max"]
        and 0.0
        <= float(value["reset_max_abs_error"])
        <= OFFLINE_THRESHOLDS["reset_max_abs_error_max"]
    )


def collection_gate(summary: Mapping[str, Any], *, case_id: str) -> dict[str, Any]:
    case = canonical_collection_case(case_id)
    checks = {
        "schema": summary.get("schema_version") == SCHEMA_VERSION,
        "status": summary.get("status") == COLLECTION_COMPLETE_STATUS,
        "protocol": summary.get("protocol_id") == PROTOCOL_ID,
        "case": summary.get("case") == case,
        "p2_student_only": summary.get("candidate_role")
        == "P2_COLLECTION_STUDENT_ONLY_NONPROMOTABLE"
        and summary.get("candidate_module") == P2_MODULE_TREE
        and summary.get("candidate_promoted") is False,
        "alpha": summary.get("requested_alpha") == P2_STUDENT_WEIGHT,
        "shield_semantics": summary.get("safety_latch_activation_m")
        == SAFETY_LATCH_ACTIVATION_M
        and summary.get("safety_latch_release_m") == SAFETY_LATCH_RELEASE_M
        and summary.get("safety_latch_release_phase") == SAFETY_LATCH_RELEASE_PHASE
        and summary.get("safety_signal_lag_steps") == 1
        and summary.get("safety_latch_rule_violation_count") == 0
        and summary.get("mean_blend_count") == EXPECTED_STEPS
        and summary.get("blend_before_noise_count") == EXPECTED_STEPS
        and summary.get("noise_before_blend_count") == 0,
        "labels": summary.get("sample_count") == EXPECTED_STEPS
        and summary.get("teacher_query_count") == EXPECTED_STEPS
        and summary.get("same_state_teacher_label_count") == EXPECTED_STEPS,
        "physical": summary.get("steps") == EXPECTED_STEPS
        and summary.get("control_window_count") == EXPECTED_CONTROL_WINDOWS
        and summary.get("raw_sensor_sample_count") == EXPECTED_RAW_SENSOR_SAMPLES
        and summary.get("phase_valid_cycle_count", -1) >= MINIMUM_VALID_CYCLES
        and _finite_number(summary.get("grf_penetration_max_m"))
        and float(summary["grf_penetration_max_m"]) < PENETRATION_LIMIT_M
        and summary.get("end_reason") == "episode_time_limit"
        and summary.get("terminated") is False
        and summary.get("truncated") is True,
        "zero_invalids": all(
            summary.get(name) == 0
            for name in (
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
                "alpha_mismatch_count",
                "mean_blend_mismatch_count",
                "noise_application_mismatch_count",
            )
        ),
        "v26_unchanged": summary.get("event_contract_id") == EVENT_CONTRACT_ID
        and summary.get("target_contract_id") == TARGET_CONTRACT_ID
        and summary.get("detector_or_fsm_modified") is False,
        "layout": summary.get("n_actor") == EXPECTED_ACTOR_FEATURES
        and summary.get("n_observation") == EXPECTED_FULL_FEATURES
        and summary.get("observation_dtype") == EXPECTED_DTYPE,
        "morphology_zero": summary.get("morphology_weight") == MORPHOLOGY_WEIGHT,
        "zero_updates": summary.get("actor_updates") == 0
        and summary.get("critic_updates") == 0
        and summary.get("ppo_updates") == 0,
        "q2_unopened": summary.get("q2_paths_opened") == [],
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": COLLECTION_PASS_STATUS if passed else TERMINAL_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "case_id": case_id,
        "checks": checks,
        "collection_data_reusable": passed,
        "autonomy_demonstrated": False,
        "next_stage": "NEXT_FROZEN_STAGE" if passed else "STOP_TERMINAL",
    }


def fit_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    counts = expected_corpus_counts()
    per_new_case = summary.get("new_collection_case_metrics")
    per_case_pass = (
        isinstance(per_new_case, Mapping)
        and set(per_new_case) == set(COLLECTION_CASE_IDS)
        and all(
            isinstance(value, Mapping)
            and _finite_number(value.get("rmse"))
            and 0.0 <= float(value["rmse"]) <= OFFLINE_THRESHOLDS["rmse_max"]
            for value in per_new_case.values()
        )
    )
    checks = {
        "schema": summary.get("schema_version") == SCHEMA_VERSION,
        "status": summary.get("status") == FIT_COMPLETE_STATUS,
        "protocol": summary.get("protocol_id") == PROTOCOL_ID,
        "fit_exact": summary.get("fit") == FIT,
        "fresh_h0": summary.get("initial_checkpoint_id") == SOURCE_H0_ID
        and summary.get("continued_from_p2") is False,
        "corpus": summary.get("fit_counts") == counts
        and summary.get("sample_count") == P3_CORPUS_ROWS
        and summary.get("episode_count") == P3_EPISODE_COUNT
        and summary.get("reset_row_count") == P3_RESET_ROWS
        and summary.get("normalized_total_sample_mass") == P3_NORMALIZED_TOTAL_MASS,
        "global_metrics": _metric_triplet_within_gate(summary.get("metrics")),
        "p2_subset_metrics": _metric_triplet_within_gate(
            summary.get("p2_subset_metrics")
        ),
        "new_case_metrics": per_case_pass,
        "worst_row_provenance": isinstance(summary.get("worst_row"), Mapping)
        and set(summary["worst_row"])
        == {"absolute_error", "action_dimension", "case_id", "step_index", "tranche_id"}
        and _finite_number(summary["worst_row"].get("absolute_error")),
        "optimizer": summary.get("adamw_epochs_run") == 3000
        and summary.get("lbfgs_max_iter") == 600
        and summary.get("lbfgs_max_eval") == 1200
        and summary.get("deterministic_algorithms_enabled") is True,
        "preservation": summary.get("source_h0_byte_exact") is True
        and summary.get("logstd_byte_exact") is True
        and summary.get("critic_present") is False
        and summary.get("disabled_clock_columns_bit_zero") is True
        and summary.get("save_reload_exact") is True,
        "one_fit": summary.get("actor_updates") == 1
        and summary.get("critic_updates") == 0
        and summary.get("ppo_updates") == 0,
        "no_rescue": summary.get("hard_polish_used") is False
        and summary.get("fallback_used") is False
        and summary.get("sweep_used") is False,
        "v26_unchanged": summary.get("event_contract_id") == EVENT_CONTRACT_ID
        and summary.get("target_contract_id") == TARGET_CONTRACT_ID
        and summary.get("detector_or_fsm_modified") is False,
        "q2_unopened": summary.get("q2_paths_opened") == [],
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": FIT_PASS_STATUS if passed else TERMINAL_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "checks": checks,
        "candidate_promoted": False,
        "next_stage": "FREEZE_CANDIDATE_P3" if passed else "STOP_TERMINAL",
    }


def candidate_freeze_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    module = summary.get("candidate_module")
    tree_sha256 = module.get("tree_sha256") if isinstance(module, Mapping) else None
    valid_tree = False
    if isinstance(tree_sha256, str) and len(tree_sha256) == 64:
        try:
            int(tree_sha256, 16)
            valid_tree = True
        except ValueError:
            pass
    checks = {
        "schema": summary.get("schema_version") == SCHEMA_VERSION,
        "status": summary.get("status") == CANDIDATE_FREEZE_COMPLETE_STATUS,
        "protocol": summary.get("protocol_id") == PROTOCOL_ID,
        "fit_passed": summary.get("fit_passed") is True,
        "candidate_identity": valid_tree
        and summary.get("candidate_id") == candidate_id(tree_sha256),
        "candidate_frozen": summary.get("candidate_frozen") is True,
        "preservation": summary.get("source_h0_byte_exact") is True
        and summary.get("logstd_byte_exact") is True
        and summary.get("critic_present") is False
        and summary.get("save_reload_exact") is True,
        "no_extra_updates": summary.get("actor_updates") == 0
        and summary.get("critic_updates") == 0
        and summary.get("ppo_updates") == 0,
        "no_promotion": summary.get("runtime_promoted") is False,
        "q2_unopened": summary.get("q2_paths_opened") == [],
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": CANDIDATE_FREEZE_STATUS if passed else TERMINAL_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "checks": checks,
        "candidate_id": summary.get("candidate_id"),
        "candidate_module": copy.deepcopy(module),
        "next_stage": "DEVELOPMENT_NOMINAL_FIRST" if passed else "STOP_TERMINAL",
    }


def development_gate(summary: Mapping[str, Any], *, case_id: str) -> dict[str, Any]:
    case = canonical_development_case(case_id)
    checks = {
        "schema": summary.get("schema_version") == SCHEMA_VERSION,
        "status": summary.get("status") == DEVELOPMENT_COMPLETE_STATUS,
        "protocol": summary.get("protocol_id") == PROTOCOL_ID,
        "case": summary.get("case") == case,
        "pure_candidate": summary.get("teacher_enabled") is False
        and summary.get("blending_enabled") is False
        and summary.get("safety_latch_enabled") is False,
        "full_duration": summary.get("steps") == EXPECTED_STEPS
        and summary.get("control_window_count") == EXPECTED_CONTROL_WINDOWS
        and summary.get("raw_sensor_sample_count") == EXPECTED_RAW_SENSOR_SAMPLES,
        "physical": summary.get("phase_valid_cycle_count", -1) >= MINIMUM_VALID_CYCLES
        and _finite_number(summary.get("grf_penetration_max_m"))
        and float(summary["grf_penetration_max_m"]) < PENETRATION_LIMIT_M
        and summary.get("end_reason") == "episode_time_limit"
        and summary.get("terminated") is False
        and summary.get("truncated") is True,
        "zero_invalids": all(
            summary.get(name) == 0
            for name in (
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
        ),
        "sea_reserve": summary.get("sea_reserve_gate_passed") is True,
        "v26_unchanged": summary.get("event_contract_id") == EVENT_CONTRACT_ID
        and summary.get("target_contract_id") == TARGET_CONTRACT_ID
        and summary.get("detector_or_fsm_modified") is False,
        "zero_updates": summary.get("actor_updates") == 0
        and summary.get("critic_updates") == 0
        and summary.get("ppo_updates") == 0,
        "q2_unopened": summary.get("q2_paths_opened") == [],
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": DEVELOPMENT_PASS_STATUS if passed else TERMINAL_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "case_id": case_id,
        "checks": checks,
        "next_stage": "NEXT_FROZEN_STAGE" if passed else "STOP_TERMINAL",
    }


def aggregate_development_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    bindings = summary.get("case_gates")
    ordered = (
        isinstance(bindings, list)
        and len(bindings) == len(DEVELOPMENT_CASE_IDS)
        and all(
            isinstance(binding, Mapping)
            and binding.get("case_id") == case_id
            and binding.get("passed") is True
            for binding, case_id in zip(bindings, DEVELOPMENT_CASE_IDS, strict=True)
        )
    )
    checks = {
        "schema": summary.get("schema_version") == SCHEMA_VERSION,
        "protocol": summary.get("protocol_id") == PROTOCOL_ID,
        "nominal_first": DEVELOPMENT_CASE_IDS[0] == "deterministic_offset_nominal",
        "six_of_six": ordered,
        "fixed_candidate": summary.get("candidate_tree_unique_count") == 1,
        "activity": summary.get("collection_count") == 4
        and summary.get("development_count") == 6
        and summary.get("environment_reset_calls") == 10
        and summary.get("environment_step_calls") == 5000
        and summary.get("raw_sensor_sample_count") == 50_000
        and summary.get("teacher_query_count") == 2000
        and summary.get("actor_updates") == 1
        and summary.get("critic_updates") == 0
        and summary.get("ppo_updates") == 0,
        "no_retry": summary.get("retry_authorized") is False
        and summary.get("resume_authorized") is False
        and summary.get("rescue_authorized") is False,
        "q2_unopened": summary.get("q2_paths_opened") == [],
        "no_promotion": summary.get("runtime_promoted") is False
        and summary.get("checkpoint_zero_created") is False
        and summary.get("positive_morphology_enabled") is False,
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": FINAL_DEVELOPMENT_PASS_STATUS if passed else TERMINAL_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "checks": checks,
        "next_stage": "WAIT_SEPARATE_Q2_PROTOCOL" if passed else "STOP_TERMINAL",
    }


def contract_self_check() -> dict[str, Any]:
    checks = {
        "additive_paths": all(
            str(path).startswith(f"{VALIDATION_ROOT.as_posix()}/")
            for path in (
                PROTOCOL_FREEZE_PATH,
                DESIGN_AUDIT_PATH,
                EXECUTION_LOCK_PATH,
                RUN_ROOT,
            )
        ),
        "p2_bound": P2_CORPUS_ARTIFACT["sha256"]
        == "42a40869447aec0cdce62a6ba5fcb48da4e5f070fac9e8eb19c4f0a8fccc1990"
        and P2_MODULE_TREE["tree_sha256"]
        == "be0c9711f1f6b7b9dba9cd4ab546a8379f96444ddf02a111d5f3002b9c7e8c4f",
        "terminal_evidence_bound": len(V12R3_TERMINAL_LEDGER["sha256"]) == 64
        and len(P1S_TERMINAL_LEDGER["sha256"]) == 64
        and len(P1S_NOMINAL_FAILURE["sha256"]) == 64,
        "collection_matrix": COLLECTION_CASE_IDS
        == (
            "deterministic_offset_nominal",
            "deterministic_offset_plus_0p20",
            "stochastic_nominal_seed_127",
            "stochastic_nominal_seed_128",
        )
        and all(case["requested_alpha"] == 0.5 for case in COLLECTION_CASES),
        "corpus_counts": expected_corpus_counts()
        == {
            "p2_sample_count": 8732,
            "new_collection_sample_count": 2000,
            "new_collection_case_count": 4,
            "sample_count": 10732,
            "episode_count": 22,
            "reset_row_count": 22,
            "episode_target_mass": 500.0,
            "normalized_total_sample_mass": 11000.0,
            "component_order": ["p2_corpus", *COLLECTION_CASE_IDS],
        },
        "fit_fresh_h0": FIT["initial_checkpoint_id"] == SOURCE_H0_ID
        and FIT["continued_from_p2"] is False
        and FIT["adamw"]["epochs"] == 3000
        and FIT["lbfgs"]["max_iter"] == 600
        and FIT["lbfgs"]["max_eval"] == 1200,
        "development_nominal_first": DEVELOPMENT_CASE_IDS
        == (
            "deterministic_offset_nominal",
            "deterministic_offset_minus_0p20",
            "deterministic_offset_plus_0p20",
            "stochastic_nominal_seed_126",
            "stochastic_nominal_seed_127",
            "stochastic_nominal_seed_128",
        ),
        "stage_order": len(STAGE_IDS) == 15
        and STAGE_IDS[0] == "attest_p2_collection_source"
        and STAGE_IDS[7] == "freeze_candidate_p3"
        and STAGE_IDS[-1] == "finalize_development",
        "one_shot_authority": all(
            AUTHORITY[name]
            for name in (
                "source_implementation_authorized",
                "protocol_freeze_publication_authorized",
                "execution_lock_authorized",
                "collection_execution_authorized",
                "actor_fit_execution_authorized",
                "candidate_freeze_authorized",
                "development_execution_authorized",
            )
        )
        and not any(
            AUTHORITY[name]
            for name in (
                "qualification_execution_authorized",
                "retry_authorized",
                "resume_authorized",
                "rescue_authorized",
                "sweep_authorized",
                "runtime_promotion_authorized",
                "checkpoint_zero_authorized",
                "positive_morphology_authorized",
            )
        ),
        "q2_separate": all(
            str(path).startswith(
                "Trajectory Generator/baseline_MLP/validation/v12r4q2/"
            )
            for path in Q2_UNOPENED_PATHS.values()
        )
        and len(Q2_DESIGN_FREEZE_ARTIFACT["sha256"]) == 64,
        "v26_unchanged": EVENT_CONTRACT_ID == "binary_point_v25+heel_qualified_fsm_v2",
    }
    return {
        "schema_version": SCHEMA_VERSION,
        "status": (
            "PASS_H0_V12R4_P3_COVERAGE_CONTRACT_SELF_CHECK"
            if all(checks.values())
            else "FAIL_H0_V12R4_P3_COVERAGE_CONTRACT_SELF_CHECK"
        ),
        "passed": all(checks.values()),
        "protocol_id": PROTOCOL_ID,
        "checks": checks,
    }


__all__ = [
    "ACTOR_ARCHITECTURE",
    "AUTHORITY",
    "COLLECTION_CASES",
    "COLLECTION_CASE_IDS",
    "DEVELOPMENT_CASES",
    "DEVELOPMENT_CASE_IDS",
    "EVENT_CONTRACT_ID",
    "FIT",
    "FIT_CONTRACT_ID",
    "CANDIDATE_SELECTION_RULE",
    "OFFLINE_THRESHOLDS",
    "P2_CORPUS_ARTIFACT",
    "P2_MODULE_TREE",
    "P3_CORPUS_ROWS",
    "PIPELINE_PASS_STATUS",
    "PROTOCOL_FREEZE_STATUS",
    "PROTOCOL_ID",
    "Q2_DESIGN_FREEZE_PATH",
    "Q2_DESIGN_FREEZE_ARTIFACT",
    "Q2_PREREQUISITES",
    "Q2_UNOPENED_PATHS",
    "SCHEMA_VERSION",
    "STAGE_IDS",
    "CANDIDATE_FREEZE_STATUS",
    "EXECUTION_LOCK_STATUS",
    "aggregate_development_gate",
    "candidate_freeze_gate",
    "candidate_id",
    "canonical_collection_case",
    "canonical_development_case",
    "collection_gate",
    "contract_self_check",
    "development_gate",
    "expected_corpus_counts",
    "fit_gate",
    "stage_descriptor",
]
