"""Pure fail-closed contract for V11 weighted full-mean safe DAgger.

V11 is a fresh lineage.  It preserves the terminal V10S failure as immutable
input and changes only the preregistered actor-fit design: base-corpus
normalization, reset weighting, deterministic full-batch AdamW, and one
deterministic LBFGS phase.  Every P0--P3 fit starts again from frozen H0.

No function in this module performs I/O or training.  The contract authorizes
neither retry nor tuning, and it cannot open protected/reserve trials, update
the critic, run PPO, change the detector/GRF/SEA, or promote a runtime actor.
"""

from __future__ import annotations

import copy
import math
from pathlib import PurePosixPath
from typing import Any, Mapping

try:
    from validation import h0_primary_split_v10s_safe_dagger_contract as prior
except ModuleNotFoundError:  # Flat import when validation/ is on sys.path.
    import h0_primary_split_v10s_safe_dagger_contract as prior


SCHEMA_VERSION = 110
REVISION = "2026-08-09"
PROTOCOL_ID = "AB06_H0_PRIMARY_SPLIT_V11_V26_WEIGHTED_FULL_MEAN_SAFE_DAGGER"
PIPELINE_ID = "H0_V11_V26_WEIGHTED_FULL_MEAN_THREE_ROUND_SAFE_DAGGER"
FIT_CONTRACT_ID = "h0_primary_split_v11_weighted_full_mean_v1"
DESIGN_AUDIT_ID = (
    "h0_primary_split_v11_weighted_full_mean_p0_design_audit_v1"
)

SOURCE_H0_ID = prior.SOURCE_H0_ID
TARGET_CONTRACT_ID = prior.TARGET_CONTRACT_ID
EVENT_CONTRACT_ID = prior.EVENT_CONTRACT_ID
TEACHER_EVIDENCE_ID = prior.TEACHER_EVIDENCE_ID
TEACHER_ID = prior.TEACHER_ID
BASE_CORPUS_ID = prior.BASE_CORPUS_ID
SO_POLICY_ID = prior.SO_POLICY_ID
TRAINABLE_SCOPE = "full_mean_network"
LOGSTD_POLICY = "frozen_bit_exact"

EXPECTED_STEPS = prior.EXPECTED_STEPS
EXPECTED_ACTOR_FEATURES = prior.EXPECTED_ACTOR_FEATURES
EXPECTED_FULL_FEATURES = prior.EXPECTED_FULL_FEATURES
EXPECTED_ACTION_DIM = prior.EXPECTED_ACTION_DIM
EXPECTED_DTYPE = prior.EXPECTED_DTYPE
EXPECTED_CONTROL_WINDOWS = prior.EXPECTED_CONTROL_WINDOWS
EXPECTED_RAW_SENSOR_SAMPLES = prior.EXPECTED_RAW_SENSOR_SAMPLES
EXPECTED_POLICY_DT_S = prior.EXPECTED_POLICY_DT_S
EXPECTED_SAMPLE_DT_S = prior.EXPECTED_SAMPLE_DT_S
EXPECTED_SIGMA = prior.EXPECTED_SIGMA
MORPHOLOGY_WEIGHT = prior.MORPHOLOGY_WEIGHT
PENETRATION_LIMIT_M = prior.PENETRATION_LIMIT_M
MINIMUM_VALID_CYCLES = prior.MINIMUM_VALID_CYCLES

BASE_CORPUS_CASE_COUNT = prior.BASE_CORPUS_CASE_COUNT
BASE_CORPUS_SAMPLES_PER_CASE = prior.BASE_CORPUS_SAMPLES_PER_CASE
BASE_CORPUS_SAMPLE_COUNT = prior.BASE_CORPUS_SAMPLE_COUNT
COLLECTION_SAMPLES_PER_CASE = prior.COLLECTION_SAMPLES_PER_CASE
COLLECTION_CASE_COUNT_PER_ROUND = prior.COLLECTION_CASE_COUNT_PER_ROUND
COLLECTION_SAMPLES_PER_ROUND = prior.COLLECTION_SAMPLES_PER_ROUND

ACTOR_ARCHITECTURE = {
    "kind": "standard_mean_mlp",
    "input_dim": 35,
    "hidden_dims": [256, 256],
    "output_dim": 2,
    "activation": "tanh",
    "residual_actor": False,
}
BASE_CORPUS_NORMALIZATION = {
    "scope": "base_corpus_3000_rows_only",
    "feature_count": 35,
    "estimator": "population_mean_and_std_float64",
    "std_floor": 1.0e-4,
    "frozen_across_fit_stages": True,
    "fold_into_first_layer_before_save": True,
    "runtime_normalization_wrapper": False,
    "prescribed_clock": False,
}
SAMPLE_WEIGHTING = {
    "default_weight": 1.0,
    "reset_weight": 100.0,
    "reset_definition": "case_offset_eq_zero",
    "reduction": "sum_weighted_squared_error_div_sum_weights",
}
ADAMW = {
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
}
LBFGS = {
    "optimizer": "LBFGS",
    "deterministic": True,
    "lr": 0.7,
    "max_iter": 300,
    "max_eval": 600,
    "tolerance_grad": 1.0e-10,
    "tolerance_change": 1.0e-12,
    "history_size": 50,
    "line_search_fn": "strong_wolfe",
}
FIT = {
    "fit_contract_id": FIT_CONTRACT_ID,
    "actor_architecture": copy.deepcopy(ACTOR_ARCHITECTURE),
    "normalization": copy.deepcopy(BASE_CORPUS_NORMALIZATION),
    "sample_weighting": copy.deepcopy(SAMPLE_WEIGHTING),
    "adamw": copy.deepcopy(ADAMW),
    "lbfgs": copy.deepcopy(LBFGS),
    "optimizer_phase_order": ["adamw", "lbfgs"],
    "trainable_scope": TRAINABLE_SCOPE,
    "freeze_logstd_head": True,
    "disabled_clock_columns": [0, 1],
    "disabled_clock_policy": "bit_zero_before_and_after_save_reload",
    "anchor_enabled": False,
    "hard_polish_enabled": False,
}

FIT_STAGES = ("p0", "p1", "p2", "p3")
ROUND_ALPHAS = {1: 0.25, 2: 0.50, 3: 0.75}
FIT_COMPLETED_ROUNDS = {
    "p0": (),
    "p1": (1,),
    "p2": (1, 2),
    "p3": (1, 2, 3),
}

# V11 deliberately preserves the V10S numerical gates.  Feasibility must be
# established by the separately persisted V11 design audit; it is not inferred
# from the unsupported V10S threshold-provenance numbers.
OFFLINE_THRESHOLDS = {
    "rmse_max": 0.006,
    "max_abs_error_max": 0.060,
    "reset_max_abs_error_max": 0.003,
}
OFFLINE_THRESHOLD_PROVENANCE = {
    "kind": "V10S_GATES_PRESERVED_WITH_NEW_V11_DESIGN_AUDIT",
    "source_protocol_id": prior.PROTOCOL_ID,
    "fit_contract_id": FIT_CONTRACT_ID,
    "design_audit_id": DESIGN_AUDIT_ID,
    "thresholds_changed": False,
    "scientific_claim": False,
    "physical_gate_changed": False,
}
DESIGN_AUDIT_DETERMINISTIC_TOLERANCE = {
    "absolute": 1.0e-9,
    "relative": 1.0e-9,
    "metric_dtype": "float64",
    "scope": "same_frozen_inputs_software_platform",
}

RUN_ROOT = PurePosixPath(
    "validation/h0_primary_grf_split_adaptation_runs/"
    "2026-08-09_h0_primary_split_v11_v26_weighted_full_mean_safe_dagger"
)
BASE_CORPUS_ROOT = prior.BASE_CORPUS_ROOT
BASE_CORPUS_LEDGER_PATH = prior.BASE_CORPUS_LEDGER_PATH
TEACHER_EVIDENCE_RECEIPT_PATH = prior.TEACHER_EVIDENCE_RECEIPT_PATH
SOURCE_H0_MODULE_PATH = prior.SOURCE_H0_MODULE_PATH
ADAPTATION_ROOT = RUN_ROOT / "adaptation"
FIT_ROOTS = {stage: ADAPTATION_ROOT / stage for stage in FIT_STAGES}
MODULE_PATHS = {
    stage: FIT_ROOTS[stage] / "rl_module_target_adapted"
    for stage in FIT_STAGES
}
FIT_RECEIPT_PATHS = {
    stage: FIT_ROOTS[stage] / "receipt.json" for stage in FIT_STAGES
}
COLLECTION_ROOT = RUN_ROOT / "safe_dagger_collection"
CANDIDATE_FREEZE_PATH = ADAPTATION_ROOT / "candidate_freeze.json"
FINAL_ROOT = RUN_ROOT / "final_development"
FINAL_DEVELOPMENT_RECEIPT_PATH = FINAL_ROOT / "receipt.json"
PREFLIGHT_PATH = PurePosixPath(
    "validation/h0_primary_split_v11_weighted_full_mean_preflight_receipt.json"
)
LOCK_PATH = PurePosixPath(
    "validation/h0_primary_split_v11_weighted_full_mean_execution_lock.json"
)
DESIGN_AUDIT_RECEIPT_PATH = PurePosixPath(
    "validation/h0_primary_split_v11_design_audit_receipt.json"
)
PIPELINE_CLAIM_PATH = RUN_ROOT / "pipeline_execution_claim.json"
PIPELINE_LEDGER_PATH = RUN_ROOT / "pipeline_execution_ledger.json"
WORKER_CLAIMS_ROOT = RUN_ROOT / "pipeline_worker_claims"

V10S_TERMINAL_ROOT = prior.RUN_ROOT
V10S_TERMINAL_LEDGER_PATH = prior.PIPELINE_LEDGER_PATH
V10S_P0_GATE_PATH = prior.FIT_ROOTS["p0"] / "gate.json"
V10S_P0_SUMMARY_PATH = prior.FIT_ROOTS["p0"] / "summary.json"
V10S_P0_CORPUS_PATH = prior.FIT_ROOTS["p0"] / "corpus.npz"
V10S_TERMINAL_FAILURE_ID = "V10S_P0_FIXED_V5_FULL_MEAN_TERMINAL_FAILURE"

DESIGN_AUDIT_SOURCE_RELATIVE_PATHS = {
    "contract": "validation/h0_primary_split_v11_weighted_full_mean_contract.py",
    "fitter": "validation/h0_primary_split_v11_weighted_fit.py",
    "audit_cli": "validation/run_h0_primary_split_v11_design_audit.py",
    "validator": "validation/h0_primary_split_v10s_fit.py",
}

FINAL_CASES = tuple(
    {
        **{key: value for key, value in case.items() if key != "destination"},
        "destination": (FINAL_ROOT / case["case_id"]).as_posix(),
    }
    for case in prior.FINAL_CASES
)
FINAL_CASE_IDS = tuple(case["case_id"] for case in FINAL_CASES)
COLLECTION_CASE_IDS = prior.COLLECTION_CASE_IDS
COLLECTION_CASES = tuple(
    copy.deepcopy(next(case for case in FINAL_CASES if case["case_id"] == case_id))
    for case_id in COLLECTION_CASE_IDS
)
BASE_CORPUS_CASES = copy.deepcopy(prior.BASE_CORPUS_CASES)

COLLECTION_BEHAVIOR = prior.COLLECTION_BEHAVIOR
FINAL_BEHAVIOR = "P3_V11_V26_UNBLENDED_NO_TEACHER"

PREFLIGHT_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V11_WEIGHTED_PREFLIGHT"
LOCK_STATUS = "H0_PRIMARY_SPLIT_V11_WEIGHTED_FULL_MEAN_FROZEN"
DESIGN_AUDIT_PASS_STATUS = (
    "PASS_H0_PRIMARY_SPLIT_V11_WEIGHTED_FULL_MEAN_DESIGN_AUDIT"
)
DESIGN_AUDIT_FAIL_STATUS = (
    "FAIL_H0_PRIMARY_SPLIT_V11_WEIGHTED_FULL_MEAN_DESIGN_AUDIT"
)
V10S_HISTORY_PASS_STATUS = "PASS_V10S_TERMINAL_FAILURE_PRESERVED_FOR_V11"
V10S_HISTORY_FAIL_STATUS = "FAIL_V10S_TERMINAL_FAILURE_BINDING_FOR_V11"
PIPELINE_CLAIM_STATUS = "H0_PRIMARY_SPLIT_V11_PIPELINE_EXECUTION_CLAIMED"
WORKER_CLAIM_STATUS = "H0_PRIMARY_SPLIT_V11_PIPELINE_WORKER_CLAIMED"
FIT_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V11_WEIGHTED_FULL_MEAN_FIT"
FIT_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V11_WEIGHTED_FULL_MEAN_FIT"
FIT_COMPLETE_STATUS = "H0_PRIMARY_SPLIT_V11_WEIGHTED_FULL_MEAN_FIT_COMPLETE_UNGATED"
COLLECTION_STATUS = "H0_PRIMARY_SPLIT_V11_COLLECTION_PERSISTED_UNGATED"
COLLECTION_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V11_SAFE_DAGGER_COLLECTION"
COLLECTION_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V11_SAFE_DAGGER_COLLECTION"
FREEZE_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V11_P3_CANDIDATE_FREEZE"
FREEZE_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V11_P3_CANDIDATE_FREEZE"
FINAL_ROLLOUT_STATUS = "H0_PRIMARY_SPLIT_V11_FINAL_ROLLOUT_PERSISTED_UNGATED"
FINAL_ROLLOUT_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V11_FINAL_ROLLOUT"
FINAL_ROLLOUT_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V11_FINAL_ROLLOUT"
FINAL_DEVELOPMENT_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V11_FINAL_DEVELOPMENT"
FINAL_DEVELOPMENT_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V11_FINAL_DEVELOPMENT"
PIPELINE_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V11_SAFE_DAGGER_PIPELINE"
PIPELINE_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V11_SAFE_DAGGER_PIPELINE"

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

# Pin every source/input from V10S under a non-overwriting key, then add the
# fresh V11 implementation surface and the authoritative terminal evidence.
SOURCE_RELATIVE_PATHS = {
    f"v10s_{name}": path for name, path in prior.SOURCE_RELATIVE_PATHS.items()
}
SOURCE_RELATIVE_PATHS.update(
    {
        "contract": (
            "validation/h0_primary_split_v11_weighted_full_mean_contract.py"
        ),
        "contract_tests": (
            "validation/test_h0_primary_split_v11_weighted_full_mean_contract.py"
        ),
        "fit_engine": "validation/h0_primary_split_v11_weighted_fit.py",
        "fit_engine_tests": "validation/test_h0_primary_split_v11_weighted_fit.py",
        "design_audit_cli": (
            "validation/run_h0_primary_split_v11_design_audit.py"
        ),
        "design_audit_tests": (
            "validation/test_run_h0_primary_split_v11_design_audit.py"
        ),
        "runner": (
            "validation/run_h0_primary_split_v11_weighted_full_mean.py"
        ),
        "runner_tests": (
            "validation/test_run_h0_primary_split_v11_weighted_full_mean.py"
        ),
    }
)

INPUT_RELATIVE_PATHS = {
    f"v10s_{name}": path for name, path in prior.INPUT_RELATIVE_PATHS.items()
}
INPUT_RELATIVE_PATHS.update(
    {
        "design_audit_receipt": DESIGN_AUDIT_RECEIPT_PATH.as_posix(),
        "v10s_terminal_ledger": V10S_TERMINAL_LEDGER_PATH.as_posix(),
        "v10s_p0_gate": V10S_P0_GATE_PATH.as_posix(),
        "v10s_p0_summary": V10S_P0_SUMMARY_PATH.as_posix(),
        "v10s_p0_corpus": V10S_P0_CORPUS_PATH.as_posix(),
    }
)

AUTHORITY = {
    "authority_date": REVISION,
    "authority_text": "autorizzo V11 weighted full-mean",
    "fresh_v11_development_lineage_authorized": True,
    "v10s_terminal_failure_is_authoritative_input": True,
    "source_h0_required_for_every_refit": True,
    "four_fresh_weighted_full_mean_fits_only": True,
    "design_audit_required_before_prepare": True,
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
    "normalization_runtime_wrapper_authorized": False,
    "prescribed_clock_authorized": False,
}

V10A_EVIDENCE_SCHEMA_VERSION = prior.V10A_EVIDENCE_SCHEMA_VERSION
V10A_EVIDENCE_STATUS = prior.V10A_EVIDENCE_STATUS
V10A_EVIDENCE_SCOPE = prior.V10A_EVIDENCE_SCOPE
V10A_EVIDENCE_NEXT_STAGE = prior.V10A_EVIDENCE_NEXT_STAGE

artifact_record_matches = prior.artifact_record_matches
tree_record_matches = prior.tree_record_matches


def _finite_number(value: Any) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def _nonnegative_int(value: Any) -> bool:
    return type(value) is int and value >= 0


def _metric_triplet(value: Any) -> bool:
    return isinstance(value, Mapping) and all(
        _finite_number(value.get(name))
        for name in ("rmse", "max_abs_error", "reset_max_abs_error")
    )


def _metrics_within_thresholds(value: Mapping[str, Any]) -> bool:
    return (
        float(value["rmse"]) <= OFFLINE_THRESHOLDS["rmse_max"]
        and float(value["max_abs_error"])
        <= OFFLINE_THRESHOLDS["max_abs_error_max"]
        and float(value["reset_max_abs_error"])
        <= OFFLINE_THRESHOLDS["reset_max_abs_error_max"]
    )


def _audit_source_bindings_match(value: Any) -> bool:
    return (
        isinstance(value, Mapping)
        and set(value) == set(DESIGN_AUDIT_SOURCE_RELATIVE_PATHS)
        and all(
            artifact_record_matches(value.get(name), path)
            for name, path in DESIGN_AUDIT_SOURCE_RELATIVE_PATHS.items()
        )
    )


def _audit_corpus_matches(value: Any) -> bool:
    return (
        isinstance(value, Mapping)
        and artifact_record_matches(value.get("artifact"), V10S_P0_CORPUS_PATH)
        and value.get("rows") == BASE_CORPUS_SAMPLE_COUNT
        and value.get("observation_dim") == EXPECTED_ACTOR_FEATURES
        and value.get("action_dim") == EXPECTED_ACTION_DIM
        and value.get("reset_rows") == BASE_CORPUS_CASE_COUNT
        and value.get("observations_sha256")
        == "d367a4697606f7c5d721823c973aabbbc86fb314e9b74e1529afc22e45a4d9ad"
        and value.get("actions_sha256")
        == "06df49086f71283238842f9903cf94271c03b9b09311eefc859b706c4e66aaa6"
        and value.get("reset_mask_sha256")
        == "95addb5dda30f1a4c4830d2334c2d7b1d51fc1e3caf1a44e9f8027fbd5302b57"
    )


def v10s_terminal_failure_gate(
    ledger: Mapping[str, Any],
    p0_gate: Mapping[str, Any],
    p0_summary: Mapping[str, Any],
) -> dict[str, Any]:
    """Accept only the exact terminal scientific meaning of frozen V10S."""

    gate_checks = p0_gate.get("checks")
    metrics = p0_summary.get("metrics")
    checks = {
        "ledger_schema": ledger.get("schema_version") == prior.SCHEMA_VERSION,
        "ledger_identity": ledger.get("protocol_id") == prior.PROTOCOL_ID
        and ledger.get("pipeline_id") == prior.PIPELINE_ID,
        "ledger_terminal_fail": ledger.get("status") == prior.PIPELINE_FAIL_STATUS
        and ledger.get("passed") is False
        and ledger.get("next_stage")
        == "STOP_V10S_TERMINAL_NO_RETRY_SWEEP_OR_RESCUE",
        "ledger_stopped_at_p0": ledger.get("actor_updates") == 1
        and ledger.get("fit_actor_update_count") == 1
        and ledger.get("safe_dagger_rounds") == 0
        and ledger.get("completed_stages") == []
        and ledger.get("completed_receipts") == []
        and ledger.get("candidate_freeze") is None
        and ledger.get("final_development_receipt") is None,
        "ledger_no_retry_sweep": ledger.get("retry_authorized") is False
        and ledger.get("sweep_authorized") is False,
        "ledger_no_forbidden_updates": ledger.get("critic_updates") == 0
        and ledger.get("ppo_updates") == 0
        and ledger.get("protected_trials_opened") == []
        and ledger.get("reserve_trials_opened") == [],
        "p0_gate_identity": p0_gate.get("schema_version") == prior.SCHEMA_VERSION
        and p0_gate.get("protocol_id") == prior.PROTOCOL_ID
        and p0_gate.get("fit_stage") == "p0",
        "p0_gate_terminal_fail": p0_gate.get("status") == prior.FIT_FAIL_STATUS
        and p0_gate.get("passed") is False,
        "p0_gate_failure_reason": isinstance(gate_checks, Mapping)
        and gate_checks.get("rmse") is False
        and gate_checks.get("reset_max_abs") is False
        and gate_checks.get("max_abs") is True
        and all(
            value is True
            for name, value in gate_checks.items()
            if name not in {"rmse", "reset_max_abs"}
        ),
        "p0_summary_identity": p0_summary.get("schema_version")
        == prior.SCHEMA_VERSION
        and p0_summary.get("protocol_id") == prior.PROTOCOL_ID
        and p0_summary.get("fit_stage") == "p0",
        "p0_corpus": p0_summary.get("sample_count") == 3000
        and p0_summary.get("base_sample_count") == 3000
        and p0_summary.get("dagger_sample_count") == 0
        and p0_summary.get("reset_row_count") == 6,
        "p0_metrics": _metric_triplet(metrics)
        and float(metrics["rmse"]) > OFFLINE_THRESHOLDS["rmse_max"]
        and float(metrics["max_abs_error"])
        <= OFFLINE_THRESHOLDS["max_abs_error_max"]
        and float(metrics["reset_max_abs_error"])
        > OFFLINE_THRESHOLDS["reset_max_abs_error_max"],
        "p0_preservation": p0_summary.get("all_finite") is True
        and p0_summary.get("source_h0_byte_exact") is True
        and p0_summary.get("critic_byte_exact") is True
        and p0_summary.get("logstd_byte_exact") is True
        and p0_summary.get("actor_updates") == 1
        and p0_summary.get("critic_updates") == 0
        and p0_summary.get("ppo_updates") == 0
        and p0_summary.get("protected_trials_opened") == []
        and p0_summary.get("reserve_trials_opened") == [],
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": V10S_HISTORY_PASS_STATUS if passed else V10S_HISTORY_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "history_id": V10S_TERMINAL_FAILURE_ID,
        "checks": checks,
    }


def design_audit_gate(receipt: Mapping[str, Any]) -> dict[str, Any]:
    """Validate the one dry-run P0 feasibility receipt required by V11."""

    observed = receipt.get("observed_metrics")
    p0_reference = receipt.get("p0_reproduction_reference_metrics")
    preservation = receipt.get("preservation_audit")
    artifacts_written = receipt.get("artifacts_written")
    checks = {
        "schema": receipt.get("schema_version") == SCHEMA_VERSION,
        "identity": receipt.get("contract_id") == FIT_CONTRACT_ID
        and receipt.get("design_audit_id") == DESIGN_AUDIT_ID,
        "status": receipt.get("status") == DESIGN_AUDIT_PASS_STATUS
        and isinstance(receipt.get("gate"), Mapping)
        and receipt["gate"].get("passed") is True,
        "dry_run": receipt.get("dry_run") is True,
        "no_candidate_checkpoint": receipt.get("no_candidate_checkpoint") is True,
        "checkpoint_artifacts_absent": receipt.get("candidate_checkpoint_paths")
        == []
        and type(receipt.get("candidate_checkpoints_persisted")) is int
        and receipt.get("candidate_checkpoints_persisted") == 0,
        "source_bindings": _audit_source_bindings_match(
            receipt.get("source_bindings")
        ),
        "source_h0": tree_record_matches(
            receipt.get("source_h0"), SOURCE_H0_MODULE_PATH
        ),
        "corpus": _audit_corpus_matches(receipt.get("corpus")),
        "fit_design": receipt.get("fit_design") == FIT,
        "gates_unchanged": receipt.get("gates") == OFFLINE_THRESHOLDS,
        "observed_metrics": _metric_triplet(observed)
        and _metrics_within_thresholds(observed),
        "p0_reproduction_reference_metrics": _metric_triplet(p0_reference)
        and _metrics_within_thresholds(p0_reference),
        "deterministic_tolerance": receipt.get("deterministic_tolerance")
        == DESIGN_AUDIT_DETERMINISTIC_TOLERANCE,
        # The reference is intentionally a byte-for-JSON copy of this sole
        # audit observation, not an independently predicted metric triplet.
        # The tolerance above is consumed only by the later formal P0
        # reproduction check.
        "p0_reference_equals_observed": _metric_triplet(observed)
        and _metric_triplet(p0_reference)
        and p0_reference == observed,
        "preservation": isinstance(preservation, Mapping)
        and preservation.get("source_checkpoint_scope")
        == "actor_only_rl_module"
        and preservation.get("critic_present") is False
        and type(preservation.get("critic_parameter_count")) is int
        and preservation.get("critic_parameter_count") == 0
        and all(
            preservation.get(name) is True
            for name in (
                "source_h0_byte_exact",
                # Historical compatibility flag.  For this actor-only
                # RLModule it is vacuously true; the explicit scope/presence/
                # count checks above carry the scientific meaning.
                "critic_byte_exact",
                "logstd_byte_exact",
                "disabled_clock_columns_0_1_bit_zero",
                "normalization_folded_into_first_layer",
                "fold_equivalence_passed",
                "no_runtime_normalization_wrapper",
                "no_prescribed_clock",
            )
        ),
        "single_in_memory_actor_fit": receipt.get("actor_fit_executions") == 1
        and receipt.get("actor_updates") == 1,
        "zero_forbidden_updates": receipt.get("critic_updates") == 0
        and receipt.get("ppo_updates") == 0,
        "closed_data": receipt.get("protected_trials_opened") == []
        and receipt.get("reserve_trials_opened") == [],
        "no_retry_sweep": receipt.get("retry_authorized") is False
        and receipt.get("sweep_authorized") is False,
        "receipt_only_artifact": artifacts_written
        == [DESIGN_AUDIT_RECEIPT_PATH.as_posix()],
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": DESIGN_AUDIT_PASS_STATUS if passed else DESIGN_AUDIT_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "contract_id": FIT_CONTRACT_ID,
        "design_audit_id": DESIGN_AUDIT_ID,
        "checks": checks,
    }


def design_audit_current_binding_gate(
    receipt: Mapping[str, Any], current_bindings: Mapping[str, Any]
) -> dict[str, Any]:
    """Compare receipt hashes with records recomputed by V11 preflight."""

    corpus = receipt.get("corpus")
    checks = {
        "receipt_structural_gate": design_audit_gate(receipt).get("passed") is True,
        "current_shape": isinstance(current_bindings, Mapping)
        and set(current_bindings) == {"source_bindings", "source_h0", "corpus"},
        "source_bindings_current": isinstance(current_bindings, Mapping)
        and receipt.get("source_bindings") == current_bindings.get("source_bindings"),
        "source_h0_current": isinstance(current_bindings, Mapping)
        and receipt.get("source_h0") == current_bindings.get("source_h0"),
        "corpus_current": isinstance(corpus, Mapping)
        and isinstance(current_bindings, Mapping)
        and corpus.get("artifact") == current_bindings.get("corpus"),
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": DESIGN_AUDIT_PASS_STATUS if passed else DESIGN_AUDIT_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "contract_id": FIT_CONTRACT_ID,
        "design_audit_id": DESIGN_AUDIT_ID,
        "checks": checks,
    }


def canonical_collection_case(case_id: str, round_index: int) -> dict[str, Any]:
    if type(round_index) is not int or round_index not in ROUND_ALPHAS:
        raise ValueError(f"unknown V11 collection round: {round_index!r}")
    matches = [case for case in COLLECTION_CASES if case["case_id"] == case_id]
    if len(matches) != 1:
        raise ValueError(f"unknown V11 collection case: {case_id!r}")
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
        raise ValueError(f"unknown V11 final case: {case_id!r}")
    return copy.deepcopy(matches[0])


def expected_fit_counts(stage: str) -> dict[str, Any]:
    if stage not in FIT_STAGES:
        raise ValueError(f"unknown V11 fit stage: {stage!r}")
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
        raise ValueError(f"unknown V11 fit stage: {stage!r}")
    return f"H0_PRIMARY_SPLIT_V11_{stage.upper()}_"


def teacher_evidence_gate(receipt: Mapping[str, Any]) -> dict[str, Any]:
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
            "PASS_H0_PRIMARY_SPLIT_V11_TEACHER_EVIDENCE"
            if passed
            else "FAIL_H0_PRIMARY_SPLIT_V11_TEACHER_EVIDENCE"
        ),
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "teacher_evidence_id": TEACHER_EVIDENCE_ID,
        "checks": checks,
    }


def stage_descriptor(stage_id: str) -> dict[str, Any]:
    if stage_id not in STAGE_IDS:
        raise ValueError(f"unknown V11 pipeline stage: {stage_id!r}")
    if stage_id.startswith("fit_"):
        return {"kind": "fit", "fit_stage": stage_id.removeprefix("fit_")}
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
        raise ValueError(f"unknown V11 pipeline stage: {stage_id!r}")
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


def _fit_receipt_bindings_match(value: Any) -> bool:
    if not isinstance(value, list) or len(value) != len(FIT_STAGES):
        return False
    return all(
        isinstance(binding, Mapping)
        and set(binding) == {"fit_stage", "receipt"}
        and binding.get("fit_stage") == stage
        and artifact_record_matches(binding.get("receipt"), FIT_RECEIPT_PATHS[stage])
        for binding, stage in zip(value, FIT_STAGES, strict=True)
    )


def _collection_receipt_bindings_match(value: Any) -> bool:
    expected = [
        (round_index, case_id)
        for round_index in (1, 2, 3)
        for case_id in COLLECTION_CASE_IDS
    ]
    if not isinstance(value, list) or len(value) != len(expected):
        return False
    return all(
        isinstance(binding, Mapping)
        and set(binding) == {"round_index", "case_id", "receipt"}
        and binding.get("round_index") == round_index
        and binding.get("case_id") == case_id
        and artifact_record_matches(
            binding.get("receipt"),
            stage_receipt_path(f"collect_r{round_index}__{case_id}"),
        )
        for binding, (round_index, case_id) in zip(value, expected, strict=True)
    )


def _candidate_id_matches(summary: Mapping[str, Any]) -> bool:
    module = summary.get("candidate_module")
    return (
        tree_record_matches(module, MODULE_PATHS["p3"])
        and isinstance(summary.get("candidate_id"), str)
        and summary["candidate_id"]
        == candidate_id_prefix("p3") + module["tree_sha256"][:16]
    )


def fit_gate(summary: Mapping[str, Any], *, stage: str) -> dict[str, Any]:
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
        "ungated_status": summary.get("status") == FIT_COMPLETE_STATUS,
        "protocol": summary.get("protocol_id") == PROTOCOL_ID,
        "stage": summary.get("fit_stage") == stage,
        "fit_contract": summary.get("fit_contract_id") == FIT_CONTRACT_ID,
        "fit_exact": summary.get("fit") == FIT,
        "architecture": summary.get("actor_architecture") == ACTOR_ARCHITECTURE,
        "normalization": summary.get("normalization")
        == BASE_CORPUS_NORMALIZATION,
        "sample_weighting": summary.get("sample_weighting") == SAMPLE_WEIGHTING,
        "source_h0": summary.get("source_h0_id") == SOURCE_H0_ID,
        "restart_from_h0": summary.get("initial_checkpoint_id") == SOURCE_H0_ID
        and summary.get("continued_from_previous_candidate") is False,
        "full_mean": summary.get("trainable_scope") == TRAINABLE_SCOPE,
        "event_contract": summary.get("event_contract_id") == EVENT_CONTRACT_ID,
        "teacher_evidence": summary.get("teacher_evidence_id")
        == TEACHER_EVIDENCE_ID
        and summary.get("teacher_evidence_passed") is True,
        "v10s_terminal": summary.get("v10s_terminal_failure_id")
        == V10S_TERMINAL_FAILURE_ID
        and summary.get("v10s_terminal_failure_passed") is True,
        "design_audit": summary.get("design_audit_id") == DESIGN_AUDIT_ID
        and summary.get("design_audit_passed") is True
        and artifact_record_matches(
            summary.get("design_audit_receipt"), DESIGN_AUDIT_RECEIPT_PATH
        ),
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
        "optimizer_completion": summary.get("adamw_epochs_run") == 3000
        and summary.get("lbfgs_completed") is True
        and summary.get("deterministic_algorithms_enabled") is True,
        "clock_columns_zero": summary.get("disabled_clock_column_indices")
        == [0, 1]
        and summary.get("disabled_clock_columns_bit_zero") is True
        and summary.get("disabled_clock_columns_bit_zero_after_save_reload")
        is True,
        "normalization_folded": summary.get(
            "normalization_stats_from_base_corpus_only"
        )
        is True
        and summary.get("normalization_stats_frozen_across_stages") is True
        and summary.get("normalization_folded_into_first_layer") is True
        and summary.get("runtime_normalization_wrapper_present") is False
        and summary.get("prescribed_clock_present") is False,
        "fold_equivalence": summary.get("fold_equivalence_passed") is True,
        "no_anchor_polish": summary.get("anchor_used") is False
        and summary.get("hard_polish_used") is False,
        "metrics": _metric_triplet(metrics) and _metrics_within_thresholds(metrics),
        "p0_reproduces_audit": stage != "p0"
        or summary.get("design_audit_reproduction_within_tolerance") is True,
        "all_finite": summary.get("all_finite") is True,
        "source_immutable": summary.get("source_h0_byte_exact") is True,
        "actor_only_source": summary.get("source_checkpoint_scope")
        == "actor_only_rl_module"
        and summary.get("critic_present") is False
        and type(summary.get("critic_parameter_count")) is int
        and summary.get("critic_parameter_count") == 0,
        # Kept for receipt compatibility.  With no critic parameters this is
        # a vacuous byte-exactness assertion, not evidence of a frozen critic.
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
    summary: Mapping[str, Any], expected: Mapping[str, Any]
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
    summary: Mapping[str, Any], *, round_index: int
) -> dict[str, Any]:
    case_id = summary.get("case_id")
    try:
        expected = canonical_collection_case(str(case_id), round_index)
    except ValueError:
        expected = None
    physical = _condition_checks(summary, expected) if expected else {}
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
        == prior.SAFETY_LATCH_ACTIVATION_M
        and summary.get("safety_latch_release_m")
        == prior.SAFETY_LATCH_RELEASE_M
        and summary.get("safety_latch_release_phase")
        == prior.SAFETY_LATCH_RELEASE_PHASE
        and summary.get("safety_signal_lag_steps") == 1
        and summary.get("safety_intervention_diagnostic_only") is True,
        "latch_diagnostics": all(
            _nonnegative_int(summary.get(field))
            for field in (
                "safety_latch_activation_count",
                "safety_latch_release_count",
                "safety_intervention_count",
            )
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
        "status": COLLECTION_PASS_STATUS if passed else COLLECTION_FAIL_STATUS,
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
        "fit_contract": summary.get("fit_contract_id") == FIT_CONTRACT_ID,
        "stage": summary.get("candidate_fit_stage") == "p3",
        "candidate_module": tree_record_matches(
            summary.get("candidate_module"), MODULE_PATHS["p3"]
        ),
        "candidate_binding": _candidate_id_matches(summary),
        "fit_passed": summary.get("p3_fit_passed") is True,
        "fit_receipts": _fit_receipt_bindings_match(summary.get("fit_receipts")),
        "collection_receipts": _collection_receipt_bindings_match(
            summary.get("collection_receipts")
        ),
        "source_h0": tree_record_matches(
            summary.get("source_h0"), SOURCE_H0_MODULE_PATH
        ),
        "teacher_evidence": artifact_record_matches(
            summary.get("teacher_evidence"), TEACHER_EVIDENCE_RECEIPT_PATH
        ),
        "design_audit": summary.get("design_audit_id") == DESIGN_AUDIT_ID
        and summary.get("design_audit_passed") is True
        and artifact_record_matches(
            summary.get("design_audit_receipt"), DESIGN_AUDIT_RECEIPT_PATH
        ),
        "v10s_terminal": summary.get("v10s_terminal_failure_id")
        == V10S_TERMINAL_FAILURE_ID
        and summary.get("v10s_terminal_failure_passed") is True,
        "target_contract": summary.get("target_contract_id")
        == TARGET_CONTRACT_ID,
        "four_independent_fits": summary.get("fit_actor_update_count")
        == len(FIT_STAGES)
        and summary.get("every_fit_restarted_from_h0") is True,
        "frozen": summary.get("candidate_frozen") is True,
        "actor_only_source": summary.get("source_checkpoint_scope")
        == "actor_only_rl_module"
        and summary.get("critic_present") is False
        and type(summary.get("critic_parameter_count")) is int
        and summary.get("critic_parameter_count") == 0,
        "normalization_folded": summary.get(
            "normalization_folded_into_first_layer"
        )
        is True
        and summary.get("runtime_normalization_wrapper_present") is False,
        "no_prescribed_clock": summary.get("prescribed_clock_present") is False,
        "clock_zero": summary.get("disabled_clock_columns_0_1_bit_zero") is True,
        "logstd_frozen": summary.get("logstd_byte_exact") is True,
        # Historical compatibility flag only: this checkpoint is actor-only,
        # so critic byte equality is vacuous.  ``actor_only_source`` above is
        # the scientific gate.
        "critic_byte_exact_compatibility": summary.get("critic_byte_exact")
        is True,
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
        "fit_contract": summary.get("fit_contract_id") == FIT_CONTRACT_ID,
        "design_audit": summary.get("design_audit_id") == DESIGN_AUDIT_ID
        and summary.get("design_audit_passed") is True
        and artifact_record_matches(
            summary.get("design_audit_receipt"), DESIGN_AUDIT_RECEIPT_PATH
        ),
        "known_case": expected is not None,
        "behavior": summary.get("behavior") == FINAL_BEHAVIOR,
        "candidate_stage": summary.get("candidate_fit_stage") == "p3",
        "candidate": isinstance(summary.get("candidate_id"), str)
        and summary["candidate_id"].startswith(candidate_id_prefix("p3")),
        "actor_only_source": summary.get("source_checkpoint_scope")
        == "actor_only_rl_module"
        and summary.get("critic_present") is False
        and type(summary.get("critic_parameter_count")) is int
        and summary.get("critic_parameter_count") == 0,
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
        "folded_actor": summary.get("normalization_folded_into_first_layer")
        is True,
        "clock_columns_zero": summary.get(
            "disabled_clock_columns_0_1_bit_zero"
        )
        is True,
        "no_runtime_wrapper": summary.get("runtime_normalization_wrapper_present")
        is False,
        "no_prescribed_clock": summary.get("prescribed_clock_present") is False,
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
            or not artifact_record_matches(binding.get("gate"), destination / "gate.json")
            or not artifact_record_matches(
                binding.get("trace"), destination / "trace.json"
            )
        ):
            return False
    return True


def final_development_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    checks = {
        "schema": summary.get("schema_version") == SCHEMA_VERSION,
        "protocol": summary.get("protocol_id") == PROTOCOL_ID,
        "fit_contract": summary.get("fit_contract_id") == FIT_CONTRACT_ID,
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
        "design_audit": summary.get("design_audit_id") == DESIGN_AUDIT_ID
        and summary.get("design_audit_passed") is True
        and artifact_record_matches(
            summary.get("design_audit_receipt"), DESIGN_AUDIT_RECEIPT_PATH
        ),
        "v10s_terminal": summary.get("v10s_terminal_failure_id")
        == V10S_TERMINAL_FAILURE_ID
        and summary.get("v10s_terminal_failure_passed") is True,
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
        "actor_only_source": summary.get("source_checkpoint_scope")
        == "actor_only_rl_module"
        and summary.get("critic_present") is False
        and type(summary.get("critic_parameter_count")) is int
        and summary.get("critic_parameter_count") == 0,
        "candidate_queries": summary.get("final_candidate_mean_query_count")
        == len(FINAL_CASE_IDS) * EXPECTED_STEPS,
        "no_final_teacher": summary.get("final_teacher_query_count") == 0,
        "no_final_blend": summary.get("final_mean_blend_count") == 0,
        "no_final_latch": summary.get("final_safety_intervention_count") == 0,
        "folded_actor": summary.get("normalization_folded_into_first_layer")
        is True,
        "clock_columns_zero": summary.get(
            "disabled_clock_columns_0_1_bit_zero"
        )
        is True,
        "no_runtime_wrapper": summary.get("runtime_normalization_wrapper_present")
        is False,
        "no_prescribed_clock": summary.get("prescribed_clock_present") is False,
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
            else "STOP_V11_TERMINAL_NO_RETRY_OR_SWEEP"
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
    "design_audit_current_binding_gate",
    "design_audit_gate",
    "expected_fit_counts",
    "final_development_gate",
    "final_rollout_gate",
    "fit_gate",
    "stage_descriptor",
    "stage_receipt_path",
    "teacher_evidence_gate",
    "tree_record_matches",
    "v10s_terminal_failure_gate",
    "worker_claim_path",
]
