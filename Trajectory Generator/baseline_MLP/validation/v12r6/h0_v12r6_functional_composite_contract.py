"""Execution-free contract for the exact V12R6 functional composite.

V12R6 is a new immutable lineage.  It does not retry V12R5 and it does not
promote either of its two forensic inputs.  Instead, it synthesizes one wider
actor whose mean is exactly ``0.70 * P2 + 0.30 * R5`` while retaining the
common frozen log-standard-deviation head.  The actor must pass every locked
offline slice before six pure-policy physical development rollouts are opened.

Importing this module performs no file I/O, model loading, randomness,
environment reset, environment step, fitting, synthesis, or publication.
"""

from __future__ import annotations

import copy
import math
import sys
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence


_SOURCE_PATH = Path(__file__).resolve()
_LOCAL_VALIDATION_ROOT = _SOURCE_PATH.parent.parent
_REPO_ROOT = _SOURCE_PATH.parents[4]
for _IMPORT_ROOT in (
    _SOURCE_PATH.parent,
    _LOCAL_VALIDATION_ROOT / "v12r5",
    _LOCAL_VALIDATION_ROOT / "v12r3",
    _LOCAL_VALIDATION_ROOT,
    _REPO_ROOT / "validation",
    _REPO_ROOT / "Trajectory Generator" / "baseline_MLP",
    _REPO_ROOT,
):
    if str(_IMPORT_ROOT) not in sys.path:
        sys.path.insert(0, str(_IMPORT_ROOT))

import h0_v12r5_case_balanced_contract as v12r5  # noqa: E402


SCHEMA_VERSION = 1260
REVISION = "2026-08-14"
AUTHORITY_TEXT = "procedi col piano fino a un protocollo training ready"
PROTOCOL_ID = "AB06_H0_PRIMARY_SPLIT_V12R6_FUNCTIONAL_COMPOSITE_A030_W512_V26"
PIPELINE_ID = "H0_V12R6_FUNCTIONAL_COMPOSITE_A030_W512_CRITICAL_FIRST"
SYNTHESIS_CONTRACT_ID = "h0_v12r6_exact_functional_composite_a030_w512_v1"
CANDIDATE_SELECTION_RULE = (
    "SOLE_FUNCTIONAL_P2_R5_A030_W512_SYNTHESIS_OUTPUT_FROM_LOCKED_R6_RUN"
)

AUTHORITY = {
    "authority_date": REVISION,
    "authority_text": AUTHORITY_TEXT,
    "authority_scope": "V12R6_EXACT_COMPOSITE_SYNTHESIS_AND_DEVELOPMENT",
    "source_implementation_authorized": True,
    "protocol_freeze_publication_authorized": True,
    "execution_lock_authorized": True,
    "composite_synthesis_authorized": True,
    "candidate_freeze_authorized": True,
    "development_execution_authorized": True,
    "new_environment_collection_authorized": False,
    "actor_fit_execution_authorized": False,
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

VALIDATION_ROOT = PurePosixPath("Trajectory Generator/baseline_MLP/validation/v12r6")
PROTOCOL_FREEZE_PATH = (
    VALIDATION_ROOT / "h0_v12r6_functional_composite_protocol_freeze.json"
)
EXECUTION_LOCK_PATH = (
    VALIDATION_ROOT / "h0_v12r6_functional_composite_execution_lock.json"
)
RUN_ROOT = VALIDATION_ROOT / "h0_v12r6_run_20260814"
PIPELINE_CLAIM_PATH = RUN_ROOT / "pipeline_claim.json"
PIPELINE_LEDGER_PATH = RUN_ROOT / "pipeline_ledger.json"
ATTESTATION_RECEIPT_PATH = RUN_ROOT / "locked_input_attestation_receipt.json"
CANDIDATE_ROOT = RUN_ROOT / "candidate"
CANDIDATE_MODULE_PATH = CANDIDATE_ROOT / "rl_module_composite"
SYNTHESIS_SUMMARY_PATH = CANDIDATE_ROOT / "composite_synthesis_summary.json"
SYNTHESIS_GATE_PATH = CANDIDATE_ROOT / "composite_synthesis_gate.json"
SYNTHESIS_RECEIPT_PATH = CANDIDATE_ROOT / "composite_synthesis_receipt.json"
CANDIDATE_FREEZE_PATH = RUN_ROOT / "candidate_freeze_receipt.json"
DEVELOPMENT_ROOT = RUN_ROOT / "development"
FINAL_DEVELOPMENT_RECEIPT_PATH = RUN_ROOT / "final_development_receipt.json"

Q3_ROOT = PurePosixPath("Trajectory Generator/baseline_MLP/validation/v12r6q3")
Q3_CLOSED_PATHS = {
    "protocol_freeze": Q3_ROOT / "h0_v12r6_q3_qualification_protocol_freeze.json",
    "execution_lock": Q3_ROOT / "h0_v12r6_q3_qualification_execution_lock.json",
    "run_root": Q3_ROOT / "h0_v12r6_q3_run_20260814",
    "noise_root": Q3_ROOT / "h0_v12r6_q3_qualification_noise_tapes",
}
HISTORICAL_Q2_CLOSED_PATHS = copy.deepcopy(v12r5.Q2_UNOPENED_PATHS)
HISTORICAL_Q3_CLOSED_PATHS = copy.deepcopy(v12r5.Q3_UNOPENED_PATHS)


def _artifact(
    path: PurePosixPath | str, sha256: str, size_bytes: int
) -> dict[str, Any]:
    return {
        "path": path.as_posix() if isinstance(path, PurePosixPath) else path,
        "sha256": sha256,
        "size_bytes": size_bytes,
    }


P2_CORPUS_ARTIFACT = copy.deepcopy(v12r5.P2_CORPUS_ARTIFACT)
P2_MODULE_TREE = copy.deepcopy(v12r5.P2_MODULE_TREE)
P2_MODULE_PATH = PurePosixPath(str(P2_MODULE_TREE["path"]))
R5_MODULE_PATH = v12r5.CANDIDATE_MODULE_PATH
R5_MODULE_TREE = {
    "path": R5_MODULE_PATH.as_posix(),
    "tree_sha256": "91897e199b7da46c35bef626e3d7aad98f63f7e8815bd17a01ded5fda3ffb322",
    "file_count": 3,
    "files": [
        {
            "path": "class_and_ctor_args.pkl",
            "sha256": "5c98f006d99a71a0f1ddcbb31d8d73fe0a6dade8401e679f6af5b1bc943b4228",
            "size_bytes": 2262,
        },
        {
            "path": "metadata.json",
            "sha256": "3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12",
            "size_bytes": 197,
        },
        {
            "path": "module_state.pkl",
            "sha256": "dbca3808b974cae8f04de79d06c792229afd50f6cc53464b3f159c0bb8acad59",
            "size_bytes": 604772,
        },
    ],
}
R5_TERMINAL_LEDGER_ARTIFACT = _artifact(
    v12r5.PIPELINE_LEDGER_PATH,
    "6eb8b127a09b39403c189d6aa34d207a493469c81cdb8b7fd6da3dfd87993cd8",
    15015,
)
R5_FIT_GATE_ARTIFACT = _artifact(
    v12r5.FIT_ROOT / "gate.json",
    "42f32a0dec57d91973c4e266b3516bafce265e43ef49f4fefa992d482598c277",
    848,
)
R5_FIT_SUMMARY_ARTIFACT = _artifact(
    v12r5.FIT_ROOT / "summary.json",
    "0fcd5ce3b98e59d80882b5b113b9b911b2ab793cecd918fdde333b6b0aa1c5b2",
    14141,
)
R5_CORPUS_ARTIFACT = _artifact(
    v12r5.CORPUS_PATH,
    "db7014f5e8735062c2e340244babfaa13a4f571b14902d1a30fb9cd04cf23cf3",
    21771902,
)
R4_NOMINAL_TRACE_ARTIFACT = copy.deepcopy(v12r5.R4_NOMINAL_REUSABLE_ARTIFACTS["trace"])

SOURCE_RECORDS = {
    "p2_module": copy.deepcopy(P2_MODULE_TREE),
    "r5_forensic_module": copy.deepcopy(R5_MODULE_TREE),
    "p2_corpus": copy.deepcopy(P2_CORPUS_ARTIFACT),
    "locked_composite_gate_corpus": copy.deepcopy(R5_CORPUS_ARTIFACT),
    "r5_terminal_ledger": copy.deepcopy(R5_TERMINAL_LEDGER_ARTIFACT),
    "r5_fit_gate": copy.deepcopy(R5_FIT_GATE_ARTIFACT),
    "r5_fit_summary": copy.deepcopy(R5_FIT_SUMMARY_ARTIFACT),
    "r4_nominal_trace": copy.deepcopy(R4_NOMINAL_TRACE_ARTIFACT),
}

ALPHA_R5 = 0.30
ALPHA_P2 = 0.70
COMPOSITE_ARCHITECTURE = {
    "kind": "exact_parallel_mean_mlp",
    "input_dim": 35,
    "hidden_dims": [512, 512],
    "output_dim": 2,
    "activation": "tanh",
    "branch_width": 256,
    "branch_order": ["p2", "r5"],
    "mean_formula": "0.70*P2_MEAN+0.30*R5_MEAN",
    "logstd_source": "COMMON_BYTE_EXACT_P2_R5_HEAD",
    "residual_actor": False,
}
SYNTHESIS = {
    "synthesis_contract_id": SYNTHESIS_CONTRACT_ID,
    "candidate_selection_rule": CANDIDATE_SELECTION_RULE,
    "alpha_p2": ALPHA_P2,
    "alpha_r5": ALPHA_R5,
    "architecture": copy.deepcopy(COMPOSITE_ARCHITECTURE),
    "parameter_fit": False,
    "optimizer": None,
    "actor_updates": 0,
    "critic_updates": 0,
    "ppo_updates": 0,
    "clock_columns_disabled": [0, 1],
    "functional_equivalence_abs_tolerance": 2.0e-6,
}

SOURCE_H0_ID = v12r5.SOURCE_H0_ID
TARGET_CONTRACT_ID = v12r5.TARGET_CONTRACT_ID
EVENT_CONTRACT_ID = v12r5.EVENT_CONTRACT_ID
EXPECTED_STEPS = v12r5.EXPECTED_STEPS
EXPECTED_ACTOR_FEATURES = v12r5.EXPECTED_ACTOR_FEATURES
EXPECTED_FULL_FEATURES = v12r5.EXPECTED_FULL_FEATURES
EXPECTED_ACTION_DIM = v12r5.EXPECTED_ACTION_DIM
EXPECTED_CONTROL_WINDOWS = v12r5.EXPECTED_CONTROL_WINDOWS
EXPECTED_RAW_SENSOR_SAMPLES = v12r5.EXPECTED_RAW_SENSOR_SAMPLES
EXPECTED_RAW_SENSOR_SAMPLES_PER_STEP = v12r5.EXPECTED_RAW_SENSOR_SAMPLES_PER_STEP
EXPECTED_SIGMA = v12r5.EXPECTED_SIGMA
PENETRATION_LIMIT_M = v12r5.PENETRATION_LIMIT_M
MINIMUM_VALID_CYCLES = v12r5.MINIMUM_VALID_CYCLES
OFFLINE_THRESHOLDS = copy.deepcopy(v12r5.OFFLINE_THRESHOLDS)
CRITICAL_WINDOW = copy.deepcopy(v12r5.CRITICAL_WINDOW)
CASE_IDS = tuple(v12r5.CASE_IDS)
PURE_POLICY_COUNTER_FIELDS = tuple(v12r5.PURE_POLICY_COUNTER_FIELDS)

DEVELOPMENT_CASES = tuple(
    {key: copy.deepcopy(value) for key, value in case.items() if key != "destination"}
    for case in v12r5.DEVELOPMENT_CASES
)
DEVELOPMENT_CASE_IDS = tuple(case["case_id"] for case in DEVELOPMENT_CASES)
DEVELOPMENT_PATHS = {
    case_id: DEVELOPMENT_ROOT / case_id for case_id in DEVELOPMENT_CASE_IDS
}

STAGE_IDS = (
    "attest_terminal_inputs",
    "synthesize_functional_composite_candidate",
    "freeze_functional_composite_candidate",
    *(f"development__{case_id}" for case_id in DEVELOPMENT_CASE_IDS),
    "finalize_development",
)

PROTOCOL_FREEZE_STATUS = "PASS_H0_V12R6_FUNCTIONAL_COMPOSITE_PROTOCOL_FREEZE"
EXECUTION_LOCK_STATUS = "PASS_H0_V12R6_FUNCTIONAL_COMPOSITE_EXECUTION_LOCK"
ATTESTATION_PASS_STATUS = "PASS_H0_V12R6_LOCKED_TERMINAL_INPUT_ATTESTATION"
SYNTHESIS_COMPLETE_STATUS = "COMPLETE_H0_V12R6_FUNCTIONAL_COMPOSITE_SYNTHESIS"
SYNTHESIS_PASS_STATUS = "PASS_H0_V12R6_FUNCTIONAL_COMPOSITE_SYNTHESIS"
CANDIDATE_FREEZE_COMPLETE_STATUS = (
    "COMPLETE_H0_V12R6_FUNCTIONAL_COMPOSITE_CANDIDATE_FREEZE"
)
CANDIDATE_FREEZE_PASS_STATUS = "PASS_H0_V12R6_FUNCTIONAL_COMPOSITE_CANDIDATE_FREEZE"
DEVELOPMENT_COMPLETE_STATUS = "COMPLETE_H0_V12R6_FUNCTIONAL_COMPOSITE_DEVELOPMENT_CASE"
DEVELOPMENT_PASS_STATUS = "PASS_H0_V12R6_FUNCTIONAL_COMPOSITE_DEVELOPMENT_CASE"
FINAL_DEVELOPMENT_PASS_STATUS = "PASS_H0_V12R6_FUNCTIONAL_COMPOSITE_DEVELOPMENT"
PIPELINE_PASS_STATUS = "PASS_H0_V12R6_FUNCTIONAL_COMPOSITE_PIPELINE_TERMINAL"
TERMINAL_FAIL_STATUS = "FAIL_H0_V12R6_FUNCTIONAL_COMPOSITE_PIPELINE_TERMINAL"


def candidate_id(tree_sha256: str) -> str:
    if not isinstance(tree_sha256, str) or len(tree_sha256) != 64:
        raise ValueError("candidate tree hash must be a SHA-256 hex digest")
    try:
        int(tree_sha256, 16)
    except ValueError as exc:
        raise ValueError("candidate tree hash must be hexadecimal") from exc
    return f"AB06_H0_V12R6_FUNCTIONAL_COMPOSITE_A030_W512:{tree_sha256}"


def canonical_development_case(case_id: str) -> dict[str, Any]:
    matches = [case for case in DEVELOPMENT_CASES if case["case_id"] == case_id]
    if len(matches) != 1:
        raise ValueError(f"unknown V12R6 development case: {case_id!r}")
    return {
        **copy.deepcopy(matches[0]),
        "destination": DEVELOPMENT_PATHS[case_id].as_posix(),
        "behavior": "R6_COMPOSITE_PURE_UNBLENDED_NO_TEACHER_NO_LATCH",
    }


def stage_descriptor(stage_id: str) -> dict[str, Any]:
    if stage_id not in STAGE_IDS:
        raise ValueError(f"unknown V12R6 stage: {stage_id!r}")
    if stage_id == "attest_terminal_inputs":
        return {"stage_id": stage_id, "kind": "attestation"}
    if stage_id == "synthesize_functional_composite_candidate":
        return {"stage_id": stage_id, "kind": "synthesis"}
    if stage_id == "freeze_functional_composite_candidate":
        return {"stage_id": stage_id, "kind": "candidate_freeze"}
    if stage_id.startswith("development__"):
        case_id = stage_id.removeprefix("development__")
        return {
            "stage_id": stage_id,
            "kind": "development",
            "case": canonical_development_case(case_id),
        }
    return {"stage_id": stage_id, "kind": "finalize"}


def _finite_number(value: Any) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def _exact_int(value: Any, expected: int) -> bool:
    """Reject booleans while matching one exact activity-counter value."""

    return type(value) is int and value == expected


def _triplet_passes(value: Any) -> bool:
    if not isinstance(value, Mapping):
        return False
    names = ("rmse", "max_abs_error", "reset_max_abs_error")
    return all(_finite_number(value.get(name)) for name in names) and (
        0.0 <= float(value["rmse"]) <= OFFLINE_THRESHOLDS["rmse_max"]
        and 0.0
        <= float(value["max_abs_error"])
        <= OFFLINE_THRESHOLDS["max_abs_error_max"]
        and 0.0
        <= float(value["reset_max_abs_error"])
        <= OFFLINE_THRESHOLDS["reset_max_abs_error_max"]
    )


def _offline_evaluation_passes(value: Any) -> bool:
    if not isinstance(value, Mapping):
        return False
    metrics = value.get("metrics")
    expected_names = {
        "global",
        "p2_subset",
        "nominal_r4_pass",
        "nominal_r4_student_exposed",
        *(f"case::{case_id}" for case_id in CASE_IDS),
    }
    critical = value.get("critical_window_metrics")
    baseline = value.get("p2_critical_window_metrics")
    critical_ok = (
        isinstance(critical, Mapping)
        and isinstance(baseline, Mapping)
        and all(
            _finite_number(critical.get(name))
            and _finite_number(baseline.get(name))
            and 0.0 <= float(critical[name]) <= float(baseline[name])
            for name in ("rmse", "max_abs_error")
        )
    )
    return (
        value.get("passed") is True
        and value.get("failed_checks") == []
        and isinstance(metrics, Mapping)
        and set(metrics) == expected_names
        and all(_triplet_passes(metrics[name]) for name in expected_names)
        and critical_ok
    )


def synthesis_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    module = summary.get("candidate_module")
    tree_sha256 = module.get("tree_sha256") if isinstance(module, Mapping) else None
    valid_tree = False
    if isinstance(tree_sha256, str) and len(tree_sha256) == 64:
        try:
            int(tree_sha256, 16)
            valid_tree = True
        except ValueError:
            pass
    verification = summary.get("functional_verification")
    verification_map = verification if isinstance(verification, Mapping) else {}
    tolerance = SYNTHESIS["functional_equivalence_abs_tolerance"]
    equivalence_values = (
        verification_map.get("max_abs_error_before_save"),
        verification_map.get("max_abs_error_after_reload"),
    )
    checks = {
        "schema": summary.get("schema_version") == SCHEMA_VERSION,
        "status": summary.get("status") == SYNTHESIS_COMPLETE_STATUS,
        "identity": summary.get("protocol_id") == PROTOCOL_ID
        and summary.get("pipeline_id") == PIPELINE_ID,
        "contract": summary.get("synthesis") == SYNTHESIS,
        "sources": summary.get("source_records") == SOURCE_RECORDS,
        "candidate": valid_tree
        and summary.get("candidate_id") == candidate_id(str(tree_sha256)),
        "functional_equivalence": verification_map.get("passed") is True
        and all(
            _finite_number(value) and 0.0 <= float(value) <= tolerance
            for value in equivalence_values
        ),
        "preservation": verification_map.get("logstd_exact") is True
        and verification_map.get("disabled_clock_columns_bit_zero") is True
        and verification_map.get("save_reload_exact") is True
        and verification_map.get("actor_feature_manifest_valid") is True
        and verification_map.get("warm_start_target_512_compatible") is True,
        "offline": _offline_evaluation_passes(summary.get("offline_evaluation")),
        "zero_fit": _exact_int(summary.get("actor_updates"), 0)
        and _exact_int(summary.get("critic_updates"), 0)
        and _exact_int(summary.get("ppo_updates"), 0)
        and _exact_int(summary.get("adamw_epochs_completed"), 0)
        and _exact_int(summary.get("lbfgs_closure_calls"), 0),
        "one_synthesis": _exact_int(
            summary.get("actor_synthesis_stage_calls_attempted"), 1
        )
        and _exact_int(summary.get("actor_synthesis_executions_confirmed"), 1),
        "no_environment": _exact_int(summary.get("new_collection_count"), 0)
        and _exact_int(summary.get("environment_reset_calls"), 0)
        and _exact_int(summary.get("environment_step_calls"), 0),
        "v26_unchanged": summary.get("event_contract_id") == EVENT_CONTRACT_ID
        and summary.get("target_contract_id") == TARGET_CONTRACT_ID
        and summary.get("detector_or_fsm_modified") is False,
        "qualification_closed": summary.get("q2_paths_opened") == []
        and summary.get("q3_paths_opened") == [],
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": SYNTHESIS_PASS_STATUS if passed else TERMINAL_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "checks": checks,
        "candidate_id": summary.get("candidate_id"),
        "candidate_module": copy.deepcopy(module),
        "next_stage": "FREEZE_FUNCTIONAL_COMPOSITE_CANDIDATE"
        if passed
        else "STOP_TERMINAL",
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
        "identity": summary.get("protocol_id") == PROTOCOL_ID
        and summary.get("pipeline_id") == PIPELINE_ID,
        "selection_rule": summary.get("candidate_selection_rule")
        == CANDIDATE_SELECTION_RULE,
        "synthesis_passed": summary.get("synthesis_passed") is True,
        "candidate_identity": valid_tree
        and summary.get("candidate_id") == candidate_id(str(tree_sha256)),
        "candidate_frozen": summary.get("candidate_frozen") is True,
        "preservation": summary.get("functional_equivalence_passed") is True
        and summary.get("logstd_exact") is True
        and summary.get("save_reload_exact") is True
        and summary.get("warm_start_target_512_compatible") is True,
        "zero_updates": _exact_int(summary.get("actor_updates"), 0)
        and _exact_int(summary.get("critic_updates"), 0)
        and _exact_int(summary.get("ppo_updates"), 0),
        "no_promotion": summary.get("runtime_promoted") is False,
        "qualification_closed": summary.get("q2_paths_opened") == []
        and summary.get("q3_paths_opened") == [],
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": CANDIDATE_FREEZE_PASS_STATUS if passed else TERMINAL_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "checks": checks,
        "candidate_id": summary.get("candidate_id"),
        "candidate_module": copy.deepcopy(module),
        "next_stage": "DEVELOPMENT_CRITICAL_PLUS_FIRST" if passed else "STOP_TERMINAL",
    }


def pure_policy_trace_audit(trace: Any, *, case_id: str) -> dict[str, Any]:
    """Recompute pure-policy evidence from a development trace."""

    canonical_development_case(case_id)
    rows = (
        list(trace)
        if isinstance(trace, Sequence) and not isinstance(trace, (str, bytes))
        else []
    )
    counters = {name: 0 for name in PURE_POLICY_COUNTER_FIELDS}
    schema_exact = len(rows) == EXPECTED_STEPS
    identity_exact = schema_exact
    action_path_exact = schema_exact
    sensor_exact = schema_exact
    forbidden_payload_absent = schema_exact
    per_row_zero_counters = schema_exact
    forbidden = {
        "teacher_mean",
        "teacher_action",
        "blended_mean",
        "requested_alpha",
        "effective_alpha",
        "safety_latch_active",
    }
    for expected_step, row in enumerate(rows, start=1):
        if not isinstance(row, Mapping):
            schema_exact = False
            identity_exact = False
            action_path_exact = False
            sensor_exact = False
            forbidden_payload_absent = False
            continue
        identity_exact = identity_exact and (
            row.get("step") == expected_step
            and row.get("schema_version") == SCHEMA_VERSION
            and row.get("protocol_id") == PROTOCOL_ID
            and row.get("stage_id") == f"development__{case_id}"
            and row.get("case_id") == case_id
        )
        for name in PURE_POLICY_COUNTER_FIELDS:
            value = row.get(name)
            schema_exact = schema_exact and type(value) is int
            if type(value) is int:
                counters[name] += value
            per_row_zero_counters = per_row_zero_counters and value == 0
        schema_exact = schema_exact and (
            row.get("teacher_enabled") is False
            and row.get("blending_enabled") is False
            and row.get("safety_latch_enabled") is False
        )
        forbidden_payload_absent = forbidden_payload_absent and forbidden.isdisjoint(
            row
        )
        mean = row.get("candidate_mean")
        noise = row.get("single_noise")
        action = row.get("raw_action")
        vectors = (mean, noise, action)
        vectors_ok = all(
            isinstance(vector, list)
            and len(vector) == EXPECTED_ACTION_DIM
            and all(_finite_number(item) for item in vector)
            for vector in vectors
        )
        action_path_exact = action_path_exact and vectors_ok
        if vectors_ok:
            action_path_exact = action_path_exact and all(
                math.isclose(
                    float(action[index]),
                    float(mean[index]) + float(noise[index]),
                    rel_tol=0.0,
                    abs_tol=1.0e-7,
                )
                for index in range(EXPECTED_ACTION_DIM)
            )
        journal = row.get("observer_raw_sensor_journal")
        samples = journal.get("samples") if isinstance(journal, Mapping) else None
        sensor_exact = sensor_exact and (
            row.get("raw_sensor_sample_count") == EXPECTED_RAW_SENSOR_SAMPLES_PER_STEP
            and isinstance(samples, list)
            and len(samples) == EXPECTED_RAW_SENSOR_SAMPLES_PER_STEP
        )
    zero_counters = all(value == 0 for value in counters.values())
    passed = all(
        (
            schema_exact,
            identity_exact,
            action_path_exact,
            sensor_exact,
            forbidden_payload_absent,
            per_row_zero_counters,
            zero_counters,
        )
    )
    return {
        "passed": passed,
        "row_count": len(rows),
        "schema_exact": schema_exact,
        "identity_exact": identity_exact,
        "candidate_mean_plus_noise_exact": action_path_exact,
        "raw_sensor_samples_exact": sensor_exact,
        "forbidden_teacher_payload_absent": forbidden_payload_absent,
        "per_row_zero_counters": per_row_zero_counters,
        "zero_counters": zero_counters,
        "counters": counters,
    }


def development_gate(
    summary: Mapping[str, Any], *, case_id: str, trace: Any
) -> dict[str, Any]:
    case = canonical_development_case(case_id)
    trace_audit = pure_policy_trace_audit(trace, case_id=case_id)
    checks = {
        "schema": summary.get("schema_version") == SCHEMA_VERSION,
        "status": summary.get("status") == DEVELOPMENT_COMPLETE_STATUS,
        "protocol": summary.get("protocol_id") == PROTOCOL_ID,
        "case": summary.get("case") == case,
        "pure_candidate": summary.get("teacher_enabled") is False
        and summary.get("blending_enabled") is False
        and summary.get("safety_latch_enabled") is False
        and summary.get("pure_policy_trace_audit") == trace_audit
        and trace_audit.get("passed") is True
        and all(
            _exact_int(summary.get(name), 0) for name in PURE_POLICY_COUNTER_FIELDS
        ),
        "full_duration": _exact_int(summary.get("steps"), EXPECTED_STEPS)
        and _exact_int(summary.get("control_window_count"), EXPECTED_CONTROL_WINDOWS)
        and _exact_int(
            summary.get("raw_sensor_sample_count"), EXPECTED_RAW_SENSOR_SAMPLES
        ),
        "physical": type(summary.get("phase_valid_cycle_count")) is int
        and summary["phase_valid_cycle_count"] >= MINIMUM_VALID_CYCLES
        and _finite_number(summary.get("grf_penetration_max_m"))
        and float(summary["grf_penetration_max_m"]) < PENETRATION_LIMIT_M
        and summary.get("end_reason") == "episode_time_limit"
        and summary.get("terminated") is False
        and summary.get("truncated") is True,
        "zero_invalids": all(
            _exact_int(summary.get(name), 0)
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
        "zero_updates": _exact_int(summary.get("actor_updates"), 0)
        and _exact_int(summary.get("critic_updates"), 0)
        and _exact_int(summary.get("ppo_updates"), 0),
        "qualification_closed": summary.get("q2_paths_opened") == []
        and summary.get("q3_paths_opened") == [],
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": DEVELOPMENT_PASS_STATUS if passed else TERMINAL_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "case_id": case_id,
        "checks": checks,
        "pure_policy_trace_audit": trace_audit,
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
        "critical_plus_first": DEVELOPMENT_CASE_IDS[0]
        == "deterministic_offset_plus_0p20",
        "six_of_six": ordered,
        "fixed_candidate": _exact_int(summary.get("candidate_tree_unique_count"), 1),
        "activity": _exact_int(summary.get("new_collection_count"), 0)
        and _exact_int(summary.get("development_count"), 6)
        and _exact_int(summary.get("environment_reset_calls"), 6)
        and _exact_int(summary.get("environment_step_calls"), 3000)
        and _exact_int(summary.get("raw_sensor_sample_count"), 30_000)
        and _exact_int(summary.get("teacher_query_count"), 0)
        and _exact_int(summary.get("pure_policy_trace_row_count"), 3000)
        and all(_exact_int(summary.get(name), 0) for name in PURE_POLICY_COUNTER_FIELDS)
        and _exact_int(summary.get("actor_synthesis_stage_calls_attempted"), 1)
        and _exact_int(summary.get("actor_synthesis_executions_confirmed"), 1)
        and _exact_int(summary.get("actor_fit_stage_calls_attempted"), 0)
        and _exact_int(summary.get("actor_fit_executions_confirmed"), 0)
        and _exact_int(summary.get("actor_updates"), 0)
        and _exact_int(summary.get("adamw_epochs_completed"), 0)
        and _exact_int(summary.get("lbfgs_closure_calls"), 0)
        and _exact_int(summary.get("critic_updates"), 0)
        and _exact_int(summary.get("ppo_updates"), 0),
        "no_retry": summary.get("retry_authorized") is False
        and summary.get("resume_authorized") is False
        and summary.get("rescue_authorized") is False,
        "qualification_closed": summary.get("q2_paths_opened") == []
        and summary.get("q3_paths_opened") == [],
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
        "next_stage": "WAIT_SEPARATE_V12R6Q3_PROTOCOL" if passed else "STOP_TERMINAL",
    }


def contract_self_check() -> dict[str, Any]:
    checks = {
        "weights": math.isclose(ALPHA_P2 + ALPHA_R5, 1.0, abs_tol=0.0),
        "architecture": COMPOSITE_ARCHITECTURE["input_dim"] == EXPECTED_ACTOR_FEATURES
        and COMPOSITE_ARCHITECTURE["hidden_dims"] == [512, 512]
        and COMPOSITE_ARCHITECTURE["output_dim"] == EXPECTED_ACTION_DIM,
        "critical_first": DEVELOPMENT_CASE_IDS[0] == "deterministic_offset_plus_0p20",
        "six_cases": len(DEVELOPMENT_CASE_IDS) == 6
        and len(set(DEVELOPMENT_CASE_IDS)) == 6,
        "new_namespace": VALIDATION_ROOT != v12r5.VALIDATION_ROOT
        and RUN_ROOT != v12r5.RUN_ROOT,
        "no_fit": SYNTHESIS["parameter_fit"] is False
        and SYNTHESIS["actor_updates"] == 0,
        "qualification_closed": set(Q3_CLOSED_PATHS)
        == {"protocol_freeze", "execution_lock", "run_root", "noise_root"}
        and set(HISTORICAL_Q2_CLOSED_PATHS) == set(Q3_CLOSED_PATHS)
        and set(HISTORICAL_Q3_CLOSED_PATHS) == set(Q3_CLOSED_PATHS),
        "authority": all(
            AUTHORITY[name] is False
            for name in (
                "new_environment_collection_authorized",
                "actor_fit_execution_authorized",
                "qualification_execution_authorized",
                "retry_authorized",
                "resume_authorized",
                "rescue_authorized",
                "sweep_authorized",
                "detector_or_fsm_change_authorized",
                "runtime_promotion_authorized",
                "checkpoint_zero_authorized",
                "positive_morphology_authorized",
            )
        ),
    }
    return {"passed": all(checks.values()), "checks": checks}


if contract_self_check()["passed"] is not True:
    raise RuntimeError("V12R6 functional-composite contract self-check failed")
