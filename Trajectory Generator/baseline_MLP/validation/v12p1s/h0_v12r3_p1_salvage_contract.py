"""Pure contract for prospective V12R3-P1 salvage development.

This additive lineage does not reopen or complete V12R3.  It binds the
terminal V12R3 attempt, selects the unique materialized candidate that passed
both its frozen fit and pure-probe gates, and permits only six development
rollouts followed by one aggregate gate.  It performs no filesystem access,
model loading, fitting, labelling, simulation, or publication on import.
"""

from __future__ import annotations

import copy
import math
from collections.abc import Mapping
from pathlib import PurePosixPath
from typing import Any


SCHEMA_VERSION = 124
REVISION = "2026-08-09"
AUTHORITY_TEXT = "esegui i punti 1-6"
AUTHORITY_SCOPE = "V12R3_P1_SALVAGE_DEVELOPMENT_ONLY"
PROTOCOL_ID = "AB06_H0_V12R3_P1_SALVAGE_V26_DEVELOPMENT"
PIPELINE_ID = "H0_V12R3_P1_SALVAGE_SIX_CASE_DEVELOPMENT"

VALIDATION_ROOT = PurePosixPath("Trajectory Generator/baseline_MLP/validation/v12p1s")
RUN_ROOT = VALIDATION_ROOT / "h0_v12p1s_run_20260809"
DEVELOPMENT_ROOT = RUN_ROOT / "development"
FINAL_ROOT = RUN_ROOT / "finalize"
WORKER_CLAIMS_ROOT = RUN_ROOT / "claims"

PROTOCOL_FREEZE_PATH = VALIDATION_ROOT / "h0_v12r3_p1_salvage_protocol_freeze.json"
EXECUTION_LOCK_PATH = VALIDATION_ROOT / "h0_v12r3_p1_salvage_execution_lock.json"
PIPELINE_CLAIM_PATH = RUN_ROOT / "pipeline_claim.json"
PIPELINE_LEDGER_PATH = RUN_ROOT / "pipeline_ledger.json"
FINAL_DEVELOPMENT_SUMMARY_PATH = FINAL_ROOT / "summary.json"
FINAL_DEVELOPMENT_GATE_PATH = FINAL_ROOT / "gate.json"
FINAL_DEVELOPMENT_RECEIPT_PATH = FINAL_ROOT / "receipt.json"
FINAL_DEVELOPMENT_FAILURE_PATH = FINAL_ROOT / "failure.json"

# This pre-salvage qualification design is an immutable external input.  It is
# never a V12P1S mutation path and must exist canonically before this lineage
# can freeze or execute.
QUALIFICATION_DESIGN_FREEZE_PATH = PurePosixPath(
    "Trajectory Generator/baseline_MLP/validation/v12p1q/"
    "h0_v12r3_p1_qualification_design_freeze.json"
)
QUALIFICATION_DESIGN_SCHEMA_VERSION = 125
QUALIFICATION_DESIGN_PROTOCOL_ID = "AB06_H0_V12R3_P1_V26_INDEPENDENT_QUALIFICATION"
QUALIFICATION_DESIGN_FREEZE_PASS_STATUS = "PASS_H0_V12R3_P1_QUALIFICATION_DESIGN_FREEZE"
QUALIFICATION_DESIGN_FREEZE_KIND = "PRE_SALVAGE_INDEPENDENT_QUALIFICATION_DESIGN"
QUALIFICATION_DESIGN_LOCKED_ACCESS_STATUS = "LOCKED_PENDING_V12P1S_SIX_OF_SIX_PASS"
QUALIFICATION_DESIGN_BINDING_PASS_STATUS = (
    "PASS_H0_V12R3_P1_QUALIFICATION_DESIGN_BINDING"
)
QUALIFICATION_DESIGN_BINDING_FAIL_STATUS = (
    "FAIL_H0_V12R3_P1_QUALIFICATION_DESIGN_BINDING"
)

PROTOCOL_FREEZE_PASS_STATUS = "PASS_H0_V12R3_P1_SALVAGE_PROTOCOL_FREEZE"
PROTOCOL_FREEZE_FAIL_STATUS = "FAIL_H0_V12R3_P1_SALVAGE_PROTOCOL_FREEZE"
EXECUTION_LOCK_PASS_STATUS = "PASS_H0_V12R3_P1_SALVAGE_EXECUTION_LOCK"
EXECUTION_LOCK_FAIL_STATUS = "FAIL_H0_V12R3_P1_SALVAGE_EXECUTION_LOCK"
CANDIDATE_SELECTION_COMPLETE_STATUS = "COMPLETE_H0_V12R3_P1_SALVAGE_CANDIDATE_SELECTION"
CANDIDATE_SELECTION_PASS_STATUS = "PASS_H0_V12R3_P1_SALVAGE_CANDIDATE_SELECTION"
CANDIDATE_SELECTION_FAIL_STATUS = "FAIL_H0_V12R3_P1_SALVAGE_CANDIDATE_SELECTION"
DEVELOPMENT_ROLLOUT_COMPLETE_STATUS = "COMPLETE_H0_V12R3_P1_SALVAGE_DEVELOPMENT_ROLLOUT"
DEVELOPMENT_ROLLOUT_PASS_STATUS = "PASS_H0_V12R3_P1_SALVAGE_DEVELOPMENT_ROLLOUT"
DEVELOPMENT_ROLLOUT_FAIL_STATUS = "FAIL_H0_V12R3_P1_SALVAGE_DEVELOPMENT_ROLLOUT"
FINAL_DEVELOPMENT_COMPLETE_STATUS = "COMPLETE_H0_V12R3_P1_SALVAGE_FINAL_DEVELOPMENT"
FINAL_DEVELOPMENT_PASS_STATUS = "PASS_H0_V12R3_P1_SALVAGE_FINAL_DEVELOPMENT"
FINAL_DEVELOPMENT_FAIL_STATUS = "FAIL_H0_V12R3_P1_SALVAGE_FINAL_DEVELOPMENT"
PIPELINE_CLAIM_STATUS = "CLAIM_H0_V12R3_P1_SALVAGE_PIPELINE"

AUTHORITY = {
    "authority_date": REVISION,
    "authority_text": AUTHORITY_TEXT,
    "authority_scope": AUTHORITY_SCOPE,
    "one_shot": True,
    "protocol_freeze_authorized": True,
    "execution_lock_authorized": True,
    "development_execution_authorized": True,
    "actor_fit_authorized": False,
    "offline_teacher_labeling_authorized": False,
    "actor_updates_authorized": False,
    "critic_updates_authorized": False,
    "ppo_updates_authorized": False,
    "p2_reuse_authorized": False,
    "retry_authorized": False,
    "sweep_authorized": False,
    "rescue_authorized": False,
    "runtime_promotion_authorized": False,
    "qualification_execution_authorized": False,
    "checkpoint_zero_authorized": False,
    "positive_morphology_authorized": False,
}

P1_CANDIDATE_ID = (
    "AB06_H0_PRIMARY_SPLIT_V12R3_V26_AUTONOMY_RECOVERY:p1:ff34e153ae0ac9b6"
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

_R3_ROOT = "Trajectory Generator/baseline_MLP/validation/v12r3"
_R3_RUN_ROOT = f"{_R3_ROOT}/h0_v12r3_run_20260809"

# Exactly eleven selected records.  Label P1 is deliberately absent.  P2 is
# bound only as part of the terminal run-tree digest and is never an input or
# candidate-selection source in this lineage.
R3_SELECTED_ARTIFACTS = {
    "r3_protocol_freeze": {
        "path": f"{_R3_ROOT}/h0_primary_split_v12r3_autonomy_recovery_protocol_freeze.json",
        "sha256": "d4788902b3a7c3bd54ff7cf05893bc302326015c34f42c7e5047b3ed1e764912",
        "size_bytes": 263_054,
    },
    "r3_design_audit": {
        "path": f"{_R3_ROOT}/h0_primary_split_v12r3_autonomy_recovery_design_audit.json",
        "sha256": "e0770b183d3f8e2cc97209a23e091af7f440c3d5f6a07b466cdc24591ddc11f4",
        "size_bytes": 34_618,
    },
    "r3_execution_lock": {
        "path": f"{_R3_ROOT}/h0_primary_split_v12r3_autonomy_recovery_execution_lock.json",
        "sha256": "9a28e3bfca9f0562a1a04f628d9d438133dbed50f3c11526d0143cf67bf1226e",
        "size_bytes": 92_164,
    },
    "r3_pipeline_claim": {
        "path": f"{_R3_RUN_ROOT}/pipeline_claim.json",
        "sha256": "16ce378f448298663d55a20373ddea467aec93a179cf91c6397d8b584fec7ee4",
        "size_bytes": 1_813,
    },
    "r3_pipeline_ledger": {
        "path": f"{_R3_RUN_ROOT}/pipeline_ledger.json",
        "sha256": "5961322102a6d3a159e472d23e9223ba053b7d632ed02200a6d6c763ec326ad8",
        "size_bytes": 10_856,
    },
    "r3_fit_p1_receipt": {
        "path": f"{_R3_RUN_ROOT}/fit/p1/receipt.json",
        "sha256": "59f62250edf512e8d760d85de82d3a686ad67f318d09c055011fadb2c4e8cb82",
        "size_bytes": 2_401,
    },
    "r3_fit_p1_gate": {
        "path": f"{_R3_RUN_ROOT}/fit/p1/gate.json",
        "sha256": "edb7290aaedcb5c04d6d5a4f96d9a4dbc823beb9ecfc3569c238dd65141bda55",
        "size_bytes": 1_839,
    },
    "r3_fit_p1_summary": {
        "path": f"{_R3_RUN_ROOT}/fit/p1/summary.json",
        "sha256": "a5aa138d4cc198ec7ad93202d35cefaf40c10c729f8ebd24c7ed9324518bf5d2",
        "size_bytes": 26_517,
    },
    "r3_probe_p1_receipt": {
        "path": f"{_R3_RUN_ROOT}/probe/p1/receipt.json",
        "sha256": "3fcd22c0777e17c7375f3580462bf9e3d5301f4376b88c1e963a4beab6e9fd79",
        "size_bytes": 4_915,
    },
    "r3_probe_p1_gate": {
        "path": f"{_R3_RUN_ROOT}/probe/p1/gate.json",
        "sha256": "851c691edd0d5f3e387203f0f5b191bf12207f4474e7d2cd0763b10ae6634e97",
        "size_bytes": 8_765,
    },
    "r3_probe_p1_summary": {
        "path": f"{_R3_RUN_ROOT}/probe/p1/summary.json",
        "sha256": "5be889ff3449f56637ea8aa0c7c03fb66e472e0053f3efd8e9acc357f57e21be",
        "size_bytes": 14_452,
    },
}

R3_TERMINAL_RUN_TREE = {
    "path": _R3_RUN_ROOT,
    "tree_sha256": "1248381ad63d990c59b9aaff812385b606bdbf1516814c497f32a53a0d1ee50e",
    "file_count": 2_819,
    "total_size_bytes": 79_261_095,
}

R3_TERMINAL_SEMANTICS = {
    "status": "FAIL_H0_PRIMARY_SPLIT_V12R3_PIPELINE_TERMINAL",
    "passed": False,
    "attempted_stage": "fit_p2",
    "completed_stages": [
        "fit_p0",
        "probe_p0",
        "label_p0",
        "collect_r1__deterministic_offset_minus_0p20",
        "collect_r1__stochastic_nominal_seed_126",
        "fit_p1",
        "probe_p1",
        "label_p1",
        "collect_r2__deterministic_offset_minus_0p20",
        "collect_r2__stochastic_nominal_seed_126",
    ],
    "error": {
        "type": "V12R3RecoveryWeightedFitError",
        "message": "p2 fit summary failed contract gate: ['offline_metrics']",
    },
    "actor_fit_stage_calls_attempted": 3,
    "actor_fit_executions_confirmed": 2,
    "actor_updates_confirmed": 2,
    "environment_reset_calls": 6,
    "environment_step_calls": 2_732,
    "offline_teacher_label_calls_confirmed": 732,
}

R3_TERMINAL_LINEAGE = {
    "selected_artifacts": copy.deepcopy(R3_SELECTED_ARTIFACTS),
    "run_tree": copy.deepcopy(R3_TERMINAL_RUN_TREE),
    "semantics": copy.deepcopy(R3_TERMINAL_SEMANTICS),
}

CANDIDATE_SELECTION_POLICY = {
    "rule": "UNIQUE_MATERIALIZED_CANDIDATE_WITH_FIT_AND_PURE_PROBE_PASS",
    "selection_is_post_terminal_and_prospective_for_new_data": True,
    "v12r3_is_not_reopened_or_completed": True,
    "p0": {"fit_passed": True, "pure_probe_passed": False, "eligible": False},
    "p1": {"fit_passed": True, "pure_probe_passed": True, "eligible": True},
    "p2": {
        "fit_passed": False,
        "pure_probe_materialized": False,
        "eligible": False,
    },
    "p3": {"materialized": False, "eligible": False},
    "eligible_candidates": ["p1"],
    "metric_ranking_between_passing_candidates_used": False,
    "label_p1_is_selection_evidence": False,
    "p2_is_lineage_evidence_only": True,
    "p2_reuse_authorized": False,
}

EXPECTED_STEPS = 500
EXPECTED_CONTROL_WINDOWS = 5_000
EXPECTED_RAW_SENSOR_SAMPLES = 5_000
MINIMUM_VALID_CYCLES = 2
PENETRATION_LIMIT_M = 0.025
EXPECTED_ACTOR_FEATURES = 35
EXPECTED_FULL_FEATURES = 84
EXPECTED_DTYPE = "float32"
MORPHOLOGY_WEIGHT = 0.0
DISABLED_CLOCK_COLUMN_INDICES = (0, 1)
EVENT_CONTRACT_ID = "binary_point_v25+heel_qualified_fsm_v2"
TARGET_CONTRACT_ID = "primary_grf_split_v1+binary_point_v25+heel_qualified_fsm_v2"
BEHAVIOR = "PURE_UNBLENDED_NO_TEACHER_NO_LATCH"
JOINTS = ("pros_knee_angle", "pros_ankle_angle")
SEA_SIGNALS = (
    "torque_error_nm",
    "tau_spring_nm",
    "tau_spring_rate_nm_s",
    "motor_speed_rad_s",
    "motor_accel_rad_s2",
    "motor_power_w",
)
SEA_EXPECTED_SAMPLE_COUNTS = {
    "torque_error_nm": EXPECTED_RAW_SENSOR_SAMPLES,
    "tau_spring_nm": EXPECTED_RAW_SENSOR_SAMPLES,
    "tau_spring_rate_nm_s": 4_500,
    "motor_speed_rad_s": EXPECTED_RAW_SENSOR_SAMPLES,
    "motor_accel_rad_s2": EXPECTED_RAW_SENSOR_SAMPLES,
    "motor_power_w": EXPECTED_RAW_SENSOR_SAMPLES,
}

DEVELOPMENT_EXECUTION_AUTHORITY = {
    "one_shot": True,
    "development_rollouts": 6,
    "aggregate_stages": 1,
    "retry_authorized": False,
    "resume_authorized": False,
    "fit_authorized": False,
    "offline_teacher_labeling_authorized": False,
    "actor_updates_authorized": False,
    "critic_updates_authorized": False,
    "ppo_updates_authorized": False,
    "teacher_authorized": False,
    "blending_authorized": False,
    "safety_latch_authorized": False,
    "protected_trials_authorized": False,
    "reserve_trials_authorized": False,
    "runtime_promotion_authorized": False,
}

_CASE_ROWS = (
    (
        "deterministic_offset_minus_0p20",
        "deterministic",
        1.756870983805102,
        None,
        123,
        0.0,
    ),
    (
        "deterministic_offset_nominal",
        "deterministic",
        1.956870983805102,
        None,
        123,
        0.0,
    ),
    (
        "deterministic_offset_plus_0p20",
        "deterministic",
        2.156870983805102,
        None,
        123,
        0.0,
    ),
    (
        "stochastic_nominal_seed_126",
        "stochastic",
        1.956870983805102,
        126,
        126,
        0.005,
    ),
    (
        "stochastic_nominal_seed_127",
        "stochastic",
        1.956870983805102,
        127,
        127,
        0.005,
    ),
    (
        "stochastic_nominal_seed_128",
        "stochastic",
        1.956870983805102,
        128,
        128,
        0.005,
    ),
)

DEVELOPMENT_CASES = tuple(
    {
        "case_id": case_id,
        "action_selection": action_selection,
        "episode_start_offset_s": offset,
        "action_seed": action_seed,
        "runtime_seed": runtime_seed,
        "sigma": sigma,
        "destination": (DEVELOPMENT_ROOT / case_id).as_posix(),
    }
    for case_id, action_selection, offset, action_seed, runtime_seed, sigma in _CASE_ROWS
)
CASE_IDS = tuple(case["case_id"] for case in DEVELOPMENT_CASES)
STAGE_IDS = (
    *(f"development__{case_id}" for case_id in CASE_IDS),
    "finalize_development",
)


def canonical_development_case(case_id: str) -> dict[str, Any]:
    """Return a fresh exact description of one frozen development case."""

    matches = [case for case in DEVELOPMENT_CASES if case["case_id"] == case_id]
    if len(matches) != 1:
        raise ValueError(f"unknown P1 salvage development case: {case_id!r}")
    return copy.deepcopy(matches[0])


def stage_descriptor(stage_id: str) -> dict[str, Any]:
    """Return the only seven executable stage descriptions."""

    if stage_id == "finalize_development":
        return {"kind": "finalize_development"}
    if stage_id.startswith("development__"):
        case_id = stage_id.removeprefix("development__")
        return {"kind": "development", "case": canonical_development_case(case_id)}
    raise ValueError(f"unknown P1 salvage stage: {stage_id!r}")


def worker_claim_path(stage_id: str) -> PurePosixPath:
    if stage_id not in STAGE_IDS:
        raise ValueError(f"unknown P1 salvage stage: {stage_id!r}")
    index = STAGE_IDS.index(stage_id) + 1
    return WORKER_CLAIMS_ROOT / f"{index:02d}_{stage_id}.json"


def stage_receipt_path(stage_id: str) -> PurePosixPath:
    descriptor = stage_descriptor(stage_id)
    if descriptor["kind"] == "development":
        return PurePosixPath(descriptor["case"]["destination"]) / "receipt.json"
    return FINAL_DEVELOPMENT_RECEIPT_PATH


def _sha256(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def _nonnegative_int(value: Any) -> bool:
    return type(value) is int and value >= 0


def _finite_number(value: Any) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def artifact_record_matches(value: Any, expected_path: str | PurePosixPath) -> bool:
    """Validate a canonical three-field artifact record without performing I/O."""

    if not isinstance(value, Mapping) or set(value) != {
        "path",
        "sha256",
        "size_bytes",
    }:
        return False
    path = PurePosixPath(expected_path).as_posix()
    return (
        value.get("path") == path
        and _sha256(value.get("sha256"))
        and type(value.get("size_bytes")) is int
        and value["size_bytes"] > 0
    )


def qualification_design_freeze_gate(payload: Any) -> dict[str, Any]:
    """Validate the canonical pre-salvage qualification design semantics."""

    data = dict(payload) if isinstance(payload, Mapping) else {}
    checks_payload = data.get("checks")
    zero_activity = data.get("zero_design_activity")
    zero_gate = data.get("zero_design_activity_gate")
    access = data.get("qualification_access")
    checks = {
        "schema": data.get("schema_version") == QUALIFICATION_DESIGN_SCHEMA_VERSION,
        "pass_status": data.get("status") == QUALIFICATION_DESIGN_FREEZE_PASS_STATUS
        and data.get("passed") is True,
        "protocol": data.get("protocol_id") == QUALIFICATION_DESIGN_PROTOCOL_ID,
        "freeze_kind": data.get("freeze_kind") == QUALIFICATION_DESIGN_FREEZE_KIND,
        "canonical_destination": data.get("publication_destination")
        == QUALIFICATION_DESIGN_FREEZE_PATH.as_posix(),
        "all_frozen_checks_pass": isinstance(checks_payload, Mapping)
        and bool(checks_payload)
        and all(value is True for value in checks_payload.values()),
        "salvage_absent_at_design_freeze": isinstance(checks_payload, Mapping)
        and checks_payload.get("salvage_run_root_absent") is True,
        "zero_activity": isinstance(zero_activity, Mapping)
        and bool(zero_activity)
        and all(type(value) is int and value == 0 for value in zero_activity.values())
        and isinstance(zero_gate, Mapping)
        and zero_gate.get("passed") is True,
        "qualification_locked": isinstance(access, Mapping)
        and access.get("status") == QUALIFICATION_DESIGN_LOCKED_ACCESS_STATUS
        and access.get("qualification_design_frozen") is True
        and access.get("qualification_protocol_freeze") is None
        and access.get("qualification_execution_lock") is None
        and access.get("future_prerequisite_hashes") is None
        and access.get("hash_binding_deferred_until_salvage_terminal_pass") is True
        and access.get("required_salvage_rollout_count") == 6
        and access.get("required_salvage_passing_rollout_count") == 6
        and access.get("required_salvage_failed_rollout_count") == 0
        and access.get("noise_materialization_authorized") is False
        and access.get("qualification_execution_authorized") is False
        and access.get("runtime_promotion_authorized") is False,
        "qualification_unopened": data.get("qualification_protocol_freeze") is None
        and data.get("qualification_execution_lock") is None
        and data.get("noise_manifest") is None
        and data.get("salvage_artifact_hashes") is None
        and data.get("runtime_promoted") is False,
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": (
            QUALIFICATION_DESIGN_BINDING_PASS_STATUS
            if passed
            else QUALIFICATION_DESIGN_BINDING_FAIL_STATUS
        ),
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "qualification_protocol_id": QUALIFICATION_DESIGN_PROTOCOL_ID,
        "qualification_design_freeze_kind": QUALIFICATION_DESIGN_FREEZE_KIND,
        "checks": checks,
    }


def _qualification_design_binding_matches(data: Mapping[str, Any]) -> bool:
    gate = data.get("qualification_design_freeze_gate")
    return (
        artifact_record_matches(
            data.get("qualification_design_freeze"),
            QUALIFICATION_DESIGN_FREEZE_PATH,
        )
        and isinstance(gate, Mapping)
        and gate.get("schema_version") == SCHEMA_VERSION
        and gate.get("status") == QUALIFICATION_DESIGN_BINDING_PASS_STATUS
        and gate.get("passed") is True
        and gate.get("protocol_id") == PROTOCOL_ID
        and gate.get("qualification_protocol_id") == QUALIFICATION_DESIGN_PROTOCOL_ID
        and gate.get("qualification_design_freeze_kind")
        == QUALIFICATION_DESIGN_FREEZE_KIND
        and isinstance(gate.get("checks"), Mapping)
        and bool(gate["checks"])
        and all(value is True for value in gate["checks"].values())
    )


def _continuous_metric_matches(value: Any, expected_samples: int) -> bool:
    if not isinstance(value, Mapping) or set(value) != {
        "sample_count",
        "rms",
        "abs_max",
    }:
        return False
    rms = value.get("rms")
    absolute = value.get("abs_max")
    return (
        value.get("sample_count") == expected_samples
        and _finite_number(rms)
        and _finite_number(absolute)
        and float(rms) >= 0.0
        and float(absolute) >= float(rms)
    )


def _diagnostic_metrics_gate(summary: Mapping[str, Any]) -> dict[str, bool]:
    episode = summary.get("episode_metrics")
    episode_valid = isinstance(episode, Mapping) and set(episode) == {
        "reserve_norm_nm",
        "residual_norm_nm",
    }
    if episode_valid:
        episode_valid = all(
            _continuous_metric_matches(episode[name], EXPECTED_STEPS)
            for name in ("reserve_norm_nm", "residual_norm_nm")
        )

    sea = summary.get("sea_episode_metrics")
    sea_valid = isinstance(sea, Mapping) and set(sea) == set(JOINTS)
    saturation_zero = sea_valid
    if sea_valid:
        for joint in JOINTS:
            joint_data = sea.get(joint)
            expected_fields = {*SEA_SIGNALS, "tau_input_saturated"}
            if (
                not isinstance(joint_data, Mapping)
                or set(joint_data) != expected_fields
            ):
                sea_valid = False
                saturation_zero = False
                break
            if not all(
                _continuous_metric_matches(
                    joint_data[signal], SEA_EXPECTED_SAMPLE_COUNTS[signal]
                )
                for signal in SEA_SIGNALS
            ):
                sea_valid = False
            saturation = joint_data.get("tau_input_saturated")
            if (
                not isinstance(saturation, Mapping)
                or set(saturation) != {"sample_count", "count", "fraction"}
                or saturation.get("sample_count") != EXPECTED_RAW_SENSOR_SAMPLES
                or saturation.get("count") != 0
                or not _finite_number(saturation.get("fraction"))
                or float(saturation["fraction"]) != 0.0
            ):
                saturation_zero = False
    return {
        "episode_metrics_finite_exact": episode_valid,
        "sea_metrics_finite_exact": sea_valid,
        "sea_saturation_zero": saturation_zero,
    }


def candidate_selection_gate(payload: Any) -> dict[str, Any]:
    """Select P1 for new development without promoting or modifying it."""

    data = dict(payload) if isinstance(payload, Mapping) else {}
    zero_fields = (
        "actor_fit_executions",
        "offline_teacher_label_calls",
        "environment_reset_calls",
        "environment_step_calls",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
    )
    checks = {
        "schema": data.get("schema_version") == SCHEMA_VERSION,
        "ungated_status": data.get("status") == CANDIDATE_SELECTION_COMPLETE_STATUS,
        "protocol": data.get("protocol_id") == PROTOCOL_ID,
        "candidate_exact": data.get("candidate_id") == P1_CANDIDATE_ID
        and data.get("candidate_fit_stage") == "p1"
        and data.get("candidate_module") == P1_CANDIDATE_MODULE,
        "qualification_design_bound": _qualification_design_binding_matches(data),
        "eleven_selected_records_exact": data.get("r3_selected_artifacts")
        == R3_SELECTED_ARTIFACTS
        and len(R3_SELECTED_ARTIFACTS) == 11,
        "terminal_tree_exact": data.get("r3_terminal_run_tree") == R3_TERMINAL_RUN_TREE,
        "terminal_semantics_exact": data.get("r3_terminal_semantics")
        == R3_TERMINAL_SEMANTICS,
        "selection_policy_exact": data.get("selection_policy")
        == CANDIDATE_SELECTION_POLICY,
        "unique_preexisting_gate_eligible_candidate": data.get(
            "unique_eligible_candidate"
        )
        is True
        and data.get("p1_fit_gate_passed") is True
        and data.get("p1_probe_integrity_passed") is True
        and data.get("p1_probe_autonomy_passed") is True,
        "v12r3_remains_terminal": data.get("v12r3_reopened") is False
        and data.get("v12r3_completed") is False
        and data.get("runtime_promoted") is False,
        "no_label_selection_or_p2_reuse": data.get(
            "label_p1_used_for_candidate_selection"
        )
        is False
        and data.get("p2_artifacts_used") == []
        and data.get("p2_module_loaded") is False
        and data.get("p2_corpus_loaded") is False,
        "zero_execution_or_updates": all(
            _nonnegative_int(data.get(field)) and data[field] == 0
            for field in zero_fields
        ),
        "closed_data": data.get("protected_trials_opened") == []
        and data.get("reserve_trials_opened") == [],
        "one_shot_no_retuning": data.get("retry_authorized") is False
        and data.get("sweep_authorized") is False
        and data.get("rescue_authorized") is False
        and data.get("post_hoc_retuning_authorized") is False,
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": (
            CANDIDATE_SELECTION_PASS_STATUS
            if passed
            else CANDIDATE_SELECTION_FAIL_STATUS
        ),
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "candidate_id": P1_CANDIDATE_ID,
        "checks": checks,
        "candidate_selected_for_development": passed,
        "candidate_promoted": False,
        "runtime_promoted": False,
        "next_stage": "DEVELOPMENT_ROLLOUTS" if passed else "STOP_P1_SALVAGE",
    }


_ZERO_ROLLOUT_COUNTERS = (
    "fit_executions",
    "actor_fit_executions",
    "offline_teacher_label_calls",
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
    "binary_event_failure_count",
    "physical_gate_bypass_count",
    "multiple_noise_application_count",
    "noise_application_mismatch_count",
    "served_action_teacher_dependency_count",
    "teacher_query_count",
    "teacher_queries",
    "mean_blend_count",
    "blend_count",
    "latch_count",
    "actor_updates",
    "critic_updates",
    "ppo_updates",
)


def development_rollout_gate(
    summary: Any, *, case_id: str | None = None
) -> dict[str, Any]:
    """Validate one persisted, autonomous, V26-active P1 development rollout."""

    data = dict(summary) if isinstance(summary, Mapping) else {}
    observed_case_id = data.get("case_id")
    requested_case_id = observed_case_id if case_id is None else case_id
    try:
        expected = canonical_development_case(str(requested_case_id))
    except ValueError:
        expected = None
    root = (
        PurePosixPath(expected["destination"])
        if expected is not None
        else DEVELOPMENT_ROOT / "invalid"
    )
    binary_gate = data.get("binary_phase_event_gate")
    prefix_gate = data.get("binary_event_prefix_integrity")
    expected_draws = (
        EXPECTED_STEPS
        if expected is not None and expected["action_selection"] == "stochastic"
        else 0
    )
    binary_checks = (
        isinstance(binary_gate, Mapping)
        and binary_gate.get("passed") is True
        and binary_gate.get("sample_count") == EXPECTED_RAW_SENSOR_SAMPLES
        and binary_gate.get("duplicate_event_count") == 0
        and binary_gate.get("out_of_order_event_count") == 0
        and binary_gate.get("left_non_v26_source_count") == 0
        and binary_gate.get("fallback_count") == 0
        and binary_gate.get("hard_invalid_count") == 0
    )
    prefix_checks = (
        isinstance(prefix_gate, Mapping)
        and prefix_gate.get("passed") is True
        and prefix_gate.get("sample_count") == EXPECTED_RAW_SENSOR_SAMPLES
        and prefix_gate.get("raw_sensor_sample_count") == EXPECTED_RAW_SENSOR_SAMPLES
        and prefix_gate.get("control_window_count") == EXPECTED_CONTROL_WINDOWS
        and prefix_gate.get("expected_sample_count") == EXPECTED_RAW_SENSOR_SAMPLES
    )
    diagnostic_checks = _diagnostic_metrics_gate(data)
    checks = {
        "schema": data.get("schema_version") == SCHEMA_VERSION,
        "ungated_status": data.get("status") == DEVELOPMENT_ROLLOUT_COMPLETE_STATUS,
        "protocol": data.get("protocol_id") == PROTOCOL_ID,
        "pipeline_stage_exact": expected is not None
        and data.get("pipeline_id") == PIPELINE_ID
        and data.get("stage_id") == f"development__{expected['case_id']}",
        "known_case": expected is not None
        and observed_case_id == expected["case_id"]
        and requested_case_id == expected["case_id"],
        "condition_exact": expected is not None
        and data.get("action_selection") == expected["action_selection"]
        and data.get("episode_start_offset_s") == expected["episode_start_offset_s"]
        and data.get("action_seed") == expected["action_seed"]
        and data.get("runtime_seed") == expected["runtime_seed"]
        and data.get("sigma") == expected["sigma"],
        "candidate_exact": data.get("candidate_id") == P1_CANDIDATE_ID
        and data.get("candidate_fit_stage") == "p1"
        and data.get("candidate_module") == P1_CANDIDATE_MODULE
        and data.get("candidate_selection_gate_passed") is True,
        "v26_exact": data.get("binary_phase_fsm_mode") == "binary_active"
        and data.get("event_contract_id") == EVENT_CONTRACT_ID
        and data.get("target_contract_id") == TARGET_CONTRACT_ID
        and data.get("behavior") == BEHAVIOR
        and data.get("n_actor") == EXPECTED_ACTOR_FEATURES
        and data.get("n_observation") == EXPECTED_FULL_FEATURES
        and data.get("observation_dtype") == EXPECTED_DTYPE
        and data.get("morphology_weight") == MORPHOLOGY_WEIGHT,
        "pure_autonomy": data.get("teacher_enabled") is False
        and data.get("teacher_loaded_during_rollout") is False
        and data.get("blending_enabled") is False
        and data.get("safety_latch_enabled") is False
        and data.get("latch_active_at_episode_end") is False
        and data.get("candidate_mean_query_count") == EXPECTED_STEPS,
        "physical": data.get("steps") == EXPECTED_STEPS
        and data.get("trace_step_count") == EXPECTED_STEPS
        and data.get("control_window_count") == EXPECTED_CONTROL_WINDOWS
        and data.get("raw_sensor_sample_count") == EXPECTED_RAW_SENSOR_SAMPLES
        and data.get("end_reason") == "episode_time_limit"
        and data.get("terminated") is False
        and data.get("truncated") is True
        and _nonnegative_int(data.get("phase_valid_cycle_count"))
        and data["phase_valid_cycle_count"] >= MINIMUM_VALID_CYCLES
        and _finite_number(data.get("grf_penetration_max_m"))
        and 0.0 <= float(data["grf_penetration_max_m"]) < PENETRATION_LIMIT_M,
        "binary_v26_integrity": binary_checks and prefix_checks,
        **diagnostic_checks,
        "zero_invalids_fallbacks_updates": all(
            _nonnegative_int(data.get(field)) and data[field] == 0
            for field in _ZERO_ROLLOUT_COUNTERS
        ),
        "single_noise_application": data.get("random_noise_draw_count")
        == expected_draws
        and data.get("single_noise_application_count") == EXPECTED_STEPS,
        "normalization_invariants": data.get("normalization_folded_into_first_layer")
        is True
        and data.get("runtime_normalization_wrapper_present") is False
        and data.get("logstd_byte_exact") is True
        and data.get("disabled_clock_column_indices")
        == list(DISABLED_CLOCK_COLUMN_INDICES)
        and data.get("disabled_clock_columns_bit_zero") is True,
        "producer_activity_exact": data.get("rollout_executions") == 1
        and data.get("environment_reset_calls") == 1
        and data.get("environment_step_calls") == EXPECTED_STEPS
        and data.get("candidate_mean_queries") == EXPECTED_STEPS,
        "zero_authority_exact": data.get("execution_authority")
        == DEVELOPMENT_EXECUTION_AUTHORITY,
        "persisted_artifacts": artifact_record_matches(
            data.get("protocol_freeze"), PROTOCOL_FREEZE_PATH
        )
        and artifact_record_matches(data.get("execution_lock"), EXECUTION_LOCK_PATH)
        and artifact_record_matches(data.get("pipeline_claim"), PIPELINE_CLAIM_PATH)
        and artifact_record_matches(data.get("run_start"), root / "run_start.json")
        and artifact_record_matches(data.get("trace"), root / "trace.json")
        and artifact_record_matches(
            data.get("partial_summary"), root / "partial_summary.json"
        )
        and expected is not None
        and artifact_record_matches(
            data.get("worker_claim"),
            worker_claim_path(f"development__{expected['case_id']}"),
        ),
        "qualification_design_bound": _qualification_design_binding_matches(data),
        "no_p2": data.get("p2_artifacts_opened") == []
        and data.get("p2_module_loaded") is False
        and data.get("p2_corpus_loaded") is False,
        "closed_data": data.get("protected_trials_opened") == []
        and data.get("reserve_trials_opened") == [],
        "development_only": data.get("development_only") is True
        and data.get("runtime_promoted") is False
        and data.get("checkpoint_zero_created") is False,
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": (
            DEVELOPMENT_ROLLOUT_PASS_STATUS
            if passed
            else DEVELOPMENT_ROLLOUT_FAIL_STATUS
        ),
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "candidate_id": P1_CANDIDATE_ID,
        "case_id": observed_case_id,
        "checks": checks,
        "development_only": True,
        "candidate_promoted": False,
        "runtime_promoted": False,
        "next_stage": "NEXT_DEVELOPMENT_CASE" if passed else "STOP_P1_SALVAGE",
    }


def final_development_gate(summary: Any) -> dict[str, Any]:
    """Require six exact rollout PASS receipts with no averaging or promotion."""

    data = dict(summary) if isinstance(summary, Mapping) else {}
    bindings = data.get("rollout_bindings")
    bindings_pass = isinstance(bindings, list) and len(bindings) == len(
        DEVELOPMENT_CASES
    )
    if bindings_pass:
        for binding, case in zip(bindings, DEVELOPMENT_CASES, strict=True):
            destination = PurePosixPath(case["destination"])
            if (
                not isinstance(binding, Mapping)
                or set(binding) != {"case_id", "passed", "receipt", "gate", "summary"}
                or binding.get("case_id") != case["case_id"]
                or binding.get("passed") is not True
                or not artifact_record_matches(
                    binding.get("receipt"), destination / "receipt.json"
                )
                or not artifact_record_matches(
                    binding.get("gate"), destination / "gate.json"
                )
                or not artifact_record_matches(
                    binding.get("summary"), destination / "summary.json"
                )
            ):
                bindings_pass = False
                break
    zero_fields = (
        "fit_executions",
        "actor_fit_executions",
        "offline_teacher_label_calls",
        "environment_reset_calls_outside_rollouts",
        "environment_step_calls_outside_rollouts",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
        "teacher_queries",
        "blend_count",
        "latch_count",
    )
    checks = {
        "schema": data.get("schema_version") == SCHEMA_VERSION,
        "ungated_status": data.get("status") == FINAL_DEVELOPMENT_COMPLETE_STATUS,
        "protocol": data.get("protocol_id") == PROTOCOL_ID,
        "pipeline_stage_exact": data.get("pipeline_id") == PIPELINE_ID
        and data.get("stage_id") == "finalize_development",
        "candidate_exact": data.get("candidate_id") == P1_CANDIDATE_ID
        and data.get("candidate_fit_stage") == "p1"
        and data.get("candidate_module") == P1_CANDIDATE_MODULE
        and data.get("candidate_selection_gate_passed") is True,
        "exclusive_claims": artifact_record_matches(
            data.get("protocol_freeze"), PROTOCOL_FREEZE_PATH
        )
        and artifact_record_matches(data.get("execution_lock"), EXECUTION_LOCK_PATH)
        and artifact_record_matches(data.get("pipeline_claim"), PIPELINE_CLAIM_PATH)
        and artifact_record_matches(
            data.get("worker_claim"), worker_claim_path("finalize_development")
        ),
        "qualification_design_bound": _qualification_design_binding_matches(data),
        "six_of_six_exact_passes": bindings_pass
        and data.get("case_order") == list(CASE_IDS)
        and data.get("rollout_count") == len(DEVELOPMENT_CASES)
        and data.get("passing_rollout_count") == len(DEVELOPMENT_CASES)
        and data.get("failed_rollout_count") == 0,
        "terminal_activity_exact": data.get("rollout_environment_reset_calls") == 6
        and data.get("rollout_environment_step_calls") == 6 * EXPECTED_STEPS
        and data.get("candidate_mean_queries") == 6 * EXPECTED_STEPS,
        "zero_fit_label_or_updates": all(
            _nonnegative_int(data.get(field)) and data[field] == 0
            for field in zero_fields
        ),
        "no_p2": data.get("p2_artifacts_opened") == []
        and data.get("p2_module_loaded") is False
        and data.get("p2_corpus_loaded") is False,
        "closed_data": data.get("protected_trials_opened") == []
        and data.get("reserve_trials_opened") == [],
        "development_only": data.get("development_only") is True
        and data.get("runtime_promoted") is False
        and data.get("qualification_required") is True
        and data.get("qualification_executed") is False
        and data.get("checkpoint_zero_created") is False
        and data.get("positive_morphology_enabled") is False,
        "one_shot_no_retuning": data.get("retry_authorized") is False
        and data.get("resume_authorized") is False
        and data.get("sweep_authorized") is False
        and data.get("rescue_authorized") is False
        and data.get("post_hoc_retuning_authorized") is False,
        "producer_policy_exact": data.get("all_cases_required") is True
        and data.get("compensation_authorized") is False
        and data.get("best_k_authorized") is False
        and data.get("case_drop_authorized") is False
        and data.get("execution_authority") == DEVELOPMENT_EXECUTION_AUTHORITY,
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": (
            FINAL_DEVELOPMENT_PASS_STATUS if passed else FINAL_DEVELOPMENT_FAIL_STATUS
        ),
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "candidate_id": P1_CANDIDATE_ID,
        "checks": checks,
        "qualification_eligible": passed,
        "candidate_promoted": False,
        "runtime_promoted": False,
        "next_stage": (
            "WAIT_INDEPENDENT_QUALIFICATION_PROTOCOL" if passed else "STOP_P1_SALVAGE"
        ),
    }


def declared_mutation_paths() -> dict[str, PurePosixPath]:
    """Enumerate every allowed output root and exact canonical artifact path."""

    paths: dict[str, PurePosixPath] = {
        "protocol_freeze": PROTOCOL_FREEZE_PATH,
        "execution_lock": EXECUTION_LOCK_PATH,
        "run_root": RUN_ROOT,
        "development_root": DEVELOPMENT_ROOT,
        "final_root": FINAL_ROOT,
        "worker_claims_root": WORKER_CLAIMS_ROOT,
        "pipeline_claim": PIPELINE_CLAIM_PATH,
        "pipeline_ledger": PIPELINE_LEDGER_PATH,
    }
    for stage_id in STAGE_IDS:
        paths[f"worker_claim_{stage_id}"] = worker_claim_path(stage_id)
    for case in DEVELOPMENT_CASES:
        case_id = str(case["case_id"])
        root = PurePosixPath(case["destination"])
        prefix = f"development_{case_id}"
        paths[f"{prefix}_root"] = root
        paths[f"{prefix}_steps_root"] = root / "steps"
        paths[f"{prefix}_run_start"] = root / "run_start.json"
        paths[f"{prefix}_trace"] = root / "trace.json"
        paths[f"{prefix}_partial_summary"] = root / "partial_summary.json"
        paths[f"{prefix}_summary"] = root / "summary.json"
        paths[f"{prefix}_gate"] = root / "gate.json"
        paths[f"{prefix}_receipt"] = root / "receipt.json"
        paths[f"{prefix}_failure"] = root / "failure.json"
    paths.update(
        {
            "final_summary": FINAL_DEVELOPMENT_SUMMARY_PATH,
            "final_gate": FINAL_DEVELOPMENT_GATE_PATH,
            "final_receipt": FINAL_DEVELOPMENT_RECEIPT_PATH,
            "final_failure": FINAL_DEVELOPMENT_FAILURE_PATH,
        }
    )
    return paths


__all__ = [
    "AUTHORITY",
    "AUTHORITY_SCOPE",
    "AUTHORITY_TEXT",
    "BEHAVIOR",
    "CANDIDATE_SELECTION_COMPLETE_STATUS",
    "CANDIDATE_SELECTION_FAIL_STATUS",
    "CANDIDATE_SELECTION_PASS_STATUS",
    "CANDIDATE_SELECTION_POLICY",
    "CASE_IDS",
    "DEVELOPMENT_CASES",
    "DEVELOPMENT_EXECUTION_AUTHORITY",
    "DEVELOPMENT_ROLLOUT_COMPLETE_STATUS",
    "DEVELOPMENT_ROLLOUT_FAIL_STATUS",
    "DEVELOPMENT_ROLLOUT_PASS_STATUS",
    "EVENT_CONTRACT_ID",
    "DISABLED_CLOCK_COLUMN_INDICES",
    "EXECUTION_LOCK_PATH",
    "FINAL_DEVELOPMENT_COMPLETE_STATUS",
    "FINAL_DEVELOPMENT_FAIL_STATUS",
    "FINAL_DEVELOPMENT_PASS_STATUS",
    "FINAL_DEVELOPMENT_RECEIPT_PATH",
    "FINAL_ROOT",
    "MORPHOLOGY_WEIGHT",
    "JOINTS",
    "P1_CANDIDATE_ID",
    "P1_CANDIDATE_MODULE",
    "PIPELINE_CLAIM_PATH",
    "PIPELINE_ID",
    "PIPELINE_LEDGER_PATH",
    "PROTOCOL_FREEZE_PATH",
    "PROTOCOL_ID",
    "QUALIFICATION_DESIGN_BINDING_FAIL_STATUS",
    "QUALIFICATION_DESIGN_BINDING_PASS_STATUS",
    "QUALIFICATION_DESIGN_FREEZE_KIND",
    "QUALIFICATION_DESIGN_FREEZE_PASS_STATUS",
    "QUALIFICATION_DESIGN_FREEZE_PATH",
    "QUALIFICATION_DESIGN_LOCKED_ACCESS_STATUS",
    "QUALIFICATION_DESIGN_PROTOCOL_ID",
    "QUALIFICATION_DESIGN_SCHEMA_VERSION",
    "R3_SELECTED_ARTIFACTS",
    "R3_TERMINAL_LINEAGE",
    "R3_TERMINAL_RUN_TREE",
    "R3_TERMINAL_SEMANTICS",
    "REVISION",
    "RUN_ROOT",
    "SCHEMA_VERSION",
    "SEA_EXPECTED_SAMPLE_COUNTS",
    "SEA_SIGNALS",
    "STAGE_IDS",
    "TARGET_CONTRACT_ID",
    "VALIDATION_ROOT",
    "WORKER_CLAIMS_ROOT",
    "artifact_record_matches",
    "candidate_selection_gate",
    "canonical_development_case",
    "declared_mutation_paths",
    "development_rollout_gate",
    "final_development_gate",
    "qualification_design_freeze_gate",
    "stage_descriptor",
    "stage_receipt_path",
    "worker_claim_path",
]
