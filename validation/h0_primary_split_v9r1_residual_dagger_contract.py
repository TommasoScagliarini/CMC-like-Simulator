"""Fresh procedural V9R1 overlay after V9's missing feature-name aliases."""

from __future__ import annotations

import copy
from pathlib import PurePosixPath
from typing import Any, Mapping

from validation import h0_primary_split_v9_residual_dagger_contract as prior


SCHEMA_VERSION = 91
REVISION = "2026-08-07"
PROTOCOL_ID = "AB06_H0_PRIMARY_SPLIT_V9R1_V26_EVENT_CAUSAL_RESIDUAL"
PIPELINE_ID = "H0_PRIMARY_SPLIT_V9R1_V26_CAUSAL_RESIDUAL_ONE_ROUND_DAGGER"

for _name in (
    "SOURCE_H0_ID", "TARGET_CONTRACT_ID", "EVENT_CONTRACT_ID", "TEACHER_ID",
    "SO_POLICY_ID", "EXPECTED_STEPS", "EXPECTED_ACTOR_FEATURES",
    "EXPECTED_FULL_FEATURES", "EXPECTED_ACTION_DIM", "EXPECTED_DTYPE",
    "EXPECTED_CONTROL_WINDOWS", "EXPECTED_RAW_SENSOR_SAMPLES",
    "EXPECTED_POLICY_DT_S", "EXPECTED_SAMPLE_DT_S", "EXPECTED_SIGMA",
    "MORPHOLOGY_WEIGHT", "PENETRATION_LIMIT_M", "MINIMUM_VALID_CYCLES",
    "MIN_DAGGER_SAMPLES_PER_CASE", "RESIDUAL_INPUT_INDICES",
    "RESIDUAL_INPUT_COUNT", "RESIDUAL_HIDDEN_DIMS", "RESIDUAL_LIMITS",
    "NORMALIZATION_STD_FLOOR", "FIT_SEED", "WEIGHT_DECAY", "GRAD_CLIP_NORM",
    "RESET_ROW_WEIGHT", "P0_FIT", "P1_FIT", "OFFLINE_THRESHOLDS",
    "SOURCE_H0_MODULE_PATH",
):
    globals()[_name] = copy.deepcopy(getattr(prior, _name))

PIPELINE_CLAIM_STATUS = "H0_PRIMARY_SPLIT_V9R1_PIPELINE_EXECUTION_CLAIMED"
WORKER_CLAIM_STATUS = "H0_PRIMARY_SPLIT_V9R1_PIPELINE_WORKER_CLAIMED"
PIPELINE_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V9R1_RESIDUAL_DAGGER_PIPELINE"
PIPELINE_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V9R1_RESIDUAL_DAGGER_PIPELINE"
P0_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V9R1_RESIDUAL_P0_INTERIM"
P0_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V9R1_RESIDUAL_P0_INTERIM"
DAGGER_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V9R1_DAGGER_COLLECTION"
DAGGER_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V9R1_DAGGER_COLLECTION"
P1_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V9R1_RESIDUAL_P1_FINAL_FIT"
P1_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V9R1_RESIDUAL_P1_FINAL_FIT"
CANDIDATE_FREEZE_STATUS = "H0_PRIMARY_SPLIT_V9R1_CANDIDATE_FROZEN"
DEVELOPMENT_ROLLOUT_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V9R1_DEVELOPMENT_ROLLOUT"
DEVELOPMENT_ROLLOUT_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V9R1_DEVELOPMENT_ROLLOUT"
DEVELOPMENT_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V9R1_DEVELOPMENT"
DEVELOPMENT_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V9R1_DEVELOPMENT"

RUN_ROOT = PurePosixPath(
    "validation/h0_primary_grf_split_adaptation_runs/"
    "2026-08-07_h0_primary_split_v9r1_v26_causal_residual"
)
# The already gated causal corpus is immutable input, not regenerated.
TEACHER_REPLAY_ROOT = prior.TEACHER_REPLAY_ROOT
TEACHER_REPLAY_LEDGER_PATH = prior.TEACHER_REPLAY_LEDGER_PATH
ADAPTATION_ROOT = RUN_ROOT / "adaptation"
P0_ROOT = ADAPTATION_ROOT / "p0"
P0_MODULE_PATH = P0_ROOT / "rl_module_target_v26_causal_residual_p0"
P0_RECEIPT_PATH = P0_ROOT / "receipt.json"
P1_ROOT = ADAPTATION_ROOT / "p1"
P1_MODULE_PATH = ADAPTATION_ROOT / "rl_module_target_v26_causal_residual_p1"
P1_RECEIPT_PATH = P1_ROOT / "receipt.json"
CANDIDATE_FREEZE_PATH = ADAPTATION_ROOT / "candidate_freeze.json"
DAGGER_ROOT = RUN_ROOT / "dagger_round_1"
DEVELOPMENT_ROOT = RUN_ROOT / "development"
DEVELOPMENT_RECEIPT_PATH = DEVELOPMENT_ROOT / "receipt.json"
PIPELINE_CLAIM_PATH = RUN_ROOT / "pipeline_execution_claim.json"
PIPELINE_LEDGER_PATH = RUN_ROOT / "pipeline_execution_ledger.json"
WORKER_CLAIMS_ROOT = RUN_ROOT / "pipeline_worker_claims"
PREFLIGHT_PATH = PurePosixPath(
    "validation/h0_primary_split_v9r1_residual_dagger_preflight_receipt.json"
)
LOCK_PATH = PurePosixPath(
    "validation/h0_primary_split_v9r1_residual_dagger_execution_lock.json"
)

DEVELOPMENT_CASES = tuple(
    {
        **copy.deepcopy(case),
        "destination": (DEVELOPMENT_ROOT / str(case["case_id"])).as_posix(),
    }
    for case in prior.DEVELOPMENT_CASES
)
DEVELOPMENT_CASE_IDS = tuple(case["case_id"] for case in DEVELOPMENT_CASES)
DAGGER_CASE_IDS = prior.DAGGER_CASE_IDS
DAGGER_CASES = tuple(
    {
        **copy.deepcopy(
            next(case for case in DEVELOPMENT_CASES if case["case_id"] == case_id)
        ),
        "destination": (DAGGER_ROOT / case_id).as_posix(),
    }
    for case_id in DAGGER_CASE_IDS
)
STAGE_IDS = prior.STAGE_IDS

SOURCE_RELATIVE_PATHS = {
    "contract": "validation/h0_primary_split_v9r1_residual_dagger_contract.py",
    "runner": "validation/run_h0_primary_split_v9r1_residual_dagger.py",
    "contract_tests": (
        "validation/test_h0_primary_split_v9r1_residual_dagger_contract.py"
    ),
    "runner_tests": (
        "validation/test_run_h0_primary_split_v9r1_residual_dagger.py"
    ),
    "teacher_compat_contract": (
        "validation/h0_primary_split_v9r1_teacher_compat_contract.py"
    ),
    "v9_contract": "validation/h0_primary_split_v9_residual_dagger_contract.py",
    "v9_runner": "validation/run_h0_primary_split_v9_residual_dagger.py",
    "causal_teacher_helper": "validation/h0_primary_split_v9_causal_teacher.py",
    "residual_module": (
        "Trajectory Generator/baseline_MLP/primary_split_v25_residual.py"
    ),
    "numerical_engine": (
        "validation/run_h0_primary_split_v6_residual_dagger.py"
    ),
}

INPUT_RELATIVE_PATHS = {
    **prior.INPUT_RELATIVE_PATHS,
    "v9_terminal_fail_ledger": prior.PIPELINE_LEDGER_PATH.as_posix(),
    "v9_p0_receipt": prior.P0_RECEIPT_PATH.as_posix(),
}

AUTHORITY = {
    **prior.AUTHORITY,
    "fresh_h0_primary_split_v9r1_lineage_authorized": True,
    "v9_terminal_fail_preserved": True,
    "procedural_feature_name_alias_correction_only": True,
    "scientific_target_changed_from_v9": False,
    "fit_or_gate_changed_from_v9": False,
}


def canonical_case(case_id: str, *, dagger: bool = False) -> dict[str, Any]:
    rows = DAGGER_CASES if dagger else DEVELOPMENT_CASES
    matches = [case for case in rows if case["case_id"] == case_id]
    if len(matches) != 1:
        raise ValueError(f"unknown V9R1 case: {case_id!r}")
    return copy.deepcopy(matches[0])


def worker_claim_path(stage_id: str) -> PurePosixPath:
    index = STAGE_IDS.index(stage_id)
    return WORKER_CLAIMS_ROOT / f"{index + 1:02d}_{stage_id}.json"


def stage_receipt_path(stage_id: str) -> PurePosixPath:
    if stage_id == "fit_p0":
        return P0_RECEIPT_PATH
    if stage_id.startswith("collect_dagger__"):
        case_id = stage_id.removeprefix("collect_dagger__")
        canonical_case(case_id, dagger=True)
        return DAGGER_ROOT / case_id / "receipt.json"
    if stage_id == "fit_p1":
        return P1_RECEIPT_PATH
    if stage_id == "freeze_p1":
        return CANDIDATE_FREEZE_PATH
    if stage_id.startswith("develop__"):
        case_id = stage_id.removeprefix("develop__")
        canonical_case(case_id)
        return DEVELOPMENT_ROOT / case_id / "receipt.json"
    if stage_id == "finalize_development":
        return DEVELOPMENT_RECEIPT_PATH
    raise ValueError(f"unknown V9R1 stage: {stage_id!r}")


def learning_rate(stage: str, epoch: int) -> float:
    return prior.learning_rate(stage, epoch)


def fit_gate(summary: Mapping[str, Any], *, stage: str) -> dict[str, Any]:
    proxy = copy.deepcopy(dict(summary))
    proxy["schema_version"] = prior.SCHEMA_VERSION
    proxy["protocol_id"] = prior.PROTOCOL_ID
    result = prior.fit_gate(proxy, stage=stage)
    passed = result["passed"] is True
    return {
        **result,
        "schema_version": SCHEMA_VERSION,
        "status": (
            P0_PASS_STATUS if stage == "p0" and passed
            else P0_FAIL_STATUS if stage == "p0"
            else P1_PASS_STATUS if passed
            else P1_FAIL_STATUS
        ),
        "protocol_id": PROTOCOL_ID,
        "procedural_lineage": "V9R1_FEATURE_NAME_ALIAS_ONLY",
    }


def dagger_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    proxy = copy.deepcopy(dict(summary))
    proxy["schema_version"] = prior.SCHEMA_VERSION
    proxy["protocol_id"] = prior.PROTOCOL_ID
    result = prior.dagger_gate(proxy)
    passed = result["passed"] is True
    return {
        **result,
        "schema_version": SCHEMA_VERSION,
        "status": DAGGER_PASS_STATUS if passed else DAGGER_FAIL_STATUS,
        "protocol_id": PROTOCOL_ID,
        "procedural_lineage": "V9R1_FEATURE_NAME_ALIAS_ONLY",
    }


def development_rollout_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    proxy = copy.deepcopy(dict(summary))
    proxy["schema_version"] = prior.SCHEMA_VERSION
    proxy["protocol_id"] = prior.PROTOCOL_ID
    candidate = proxy.get("candidate_id")
    if isinstance(candidate, str):
        proxy["candidate_id"] = candidate.replace(
            "H0_PRIMARY_SPLIT_V9R1_P1_", "H0_PRIMARY_SPLIT_V9_P1_"
        )
    result = prior.development_rollout_gate(proxy)
    passed = result["passed"] is True
    return {
        **result,
        "schema_version": SCHEMA_VERSION,
        "status": (
            DEVELOPMENT_ROLLOUT_PASS_STATUS if passed
            else DEVELOPMENT_ROLLOUT_FAIL_STATUS
        ),
        "protocol_id": PROTOCOL_ID,
        "candidate_id": summary.get("candidate_id"),
        "procedural_lineage": "V9R1_FEATURE_NAME_ALIAS_ONLY",
    }


__all__ = [name for name in globals() if name.isupper()] + [
    "canonical_case", "dagger_gate", "development_rollout_gate", "fit_gate",
    "learning_rate", "stage_receipt_path", "worker_claim_path",
]
