"""Execution-free contract for the V12R8 adjudicated recovery successor.

V12R8 never rewrites or reruns the terminal V12R7 critical-plus attempt.  It
attests that immutable 179-step prefix, labels it offline in the R8 namespace,
collects only the other five preregistered cases with an explicitly normalized
V26 prefix summary, and performs the same single W512/13-stratum fit and six
pure-policy developments frozen by V12R7.

Importing this module performs no I/O, inference, fitting, randomness,
environment access, or publication.
"""

from __future__ import annotations

import copy
import sys
from pathlib import Path, PurePosixPath
from typing import Any, Mapping


_SOURCE = Path(__file__).resolve()
_LOCAL_VALIDATION = _SOURCE.parent.parent
_REPO_ROOT = _SOURCE.parents[4]
for _root in (
    _SOURCE.parent,
    _LOCAL_VALIDATION / "v12r7",
    _LOCAL_VALIDATION / "v12r6",
    _LOCAL_VALIDATION,
    _REPO_ROOT / "validation",
    _REPO_ROOT / "Trajectory Generator" / "baseline_MLP",
    _REPO_ROOT,
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_v12r7_recovery_contract as v12r7  # noqa: E402


SCHEMA_VERSION = 1280
REVISION = "2026-08-14"
PROTOCOL_ID = "AB06_H0_V12R8_ADJUDICATED_RECOVERY_W512_V26"
PIPELINE_ID = "H0_V12R8_R7_PREFIX_ADJUDICATION_SINGLE_FIT"
FIT_CONTRACT_ID = "h0_v12r8_adjudicated_recovery_full_mean_w512_v1"
TOPOLOGY_ID = "V12R8_STANDARD_RECOVERY_W512_V1"
CANDIDATE_SELECTION_RULE = "SOLE_R8_SINGLE_FIT_OUTPUT_IF_ALL_LOCKED_GATES_PASS"
PROTOCOL_FREEZE_PASS_STATUS = "PASS_H0_V12R8_RECOVERY_PROTOCOL_FREEZE"
EXECUTION_LOCK_PASS_STATUS = "PASS_H0_V12R8_RECOVERY_EXECUTION_LOCK"
CANDIDATE_FREEZE_PASS_STATUS = "PASS_H0_V12R8_RECOVERY_CANDIDATE_FREEZE"
DEVELOPMENT_PASS_STATUS = "PASS_H0_V12R8_RECOVERY_DEVELOPMENT"
PIPELINE_TERMINAL_PASS_STATUS = "PASS_H0_V12R8_RECOVERY_PIPELINE_TERMINAL"
ACTOR_FEATURE_MANIFEST_STATUS = "H0_V12R8_RECOVERY_ACTOR_FEATURE_CONTRACT"

AUTHORITY = {
    **copy.deepcopy(v12r7.AUTHORITY),
    "authority_date": REVISION,
    "historical_r7_prefix_adjudication_authorized": True,
    "historical_r7_prefix_rerun_authorized": False,
}

ROOT = PurePosixPath("Trajectory Generator/baseline_MLP/validation/v12r8")
PROTOCOL_FREEZE_PATH = ROOT / "h0_v12r8_recovery_protocol_freeze.json"
EXECUTION_LOCK_PATH = ROOT / "h0_v12r8_recovery_execution_lock.json"
RUN_ROOT = ROOT / "h0_v12r8_run_20260814"
CLAIM_PATH = RUN_ROOT / "pipeline_claim.json"
LEDGER_PATH = RUN_ROOT / "pipeline_ledger.json"
COLLECTION_ROOT = RUN_ROOT / "observer_collection"
FIT_ROOT = RUN_ROOT / "fit"
CORPUS_PATH = FIT_ROOT / "corpus.npz"
CANDIDATE_MODULE_PATH = FIT_ROOT / "rl_module_recovery"
FIT_SUMMARY_PATH = FIT_ROOT / "summary.json"
FIT_GATE_PATH = FIT_ROOT / "gate.json"
FIT_RECEIPT_PATH = FIT_ROOT / "receipt.json"
CANDIDATE_FREEZE_PATH = RUN_ROOT / "candidate_freeze_receipt.json"
DEVELOPMENT_ROOT = RUN_ROOT / "development"
FINAL_DEVELOPMENT_PATH = RUN_ROOT / "final_development_receipt.json"

R7_ROOT = v12r7.ROOT
R7_PROTOCOL_FREEZE_PATH = v12r7.PROTOCOL_FREEZE_PATH
R7_EXECUTION_LOCK_PATH = v12r7.EXECUTION_LOCK_PATH
R7_TERMINAL_LEDGER_PATH = v12r7.LEDGER_PATH
R7_PLUS_ROOT = v12r7.COLLECTION_ROOT / "deterministic_offset_plus_0p20"
R7_PLUS_TRACE_PATH = R7_PLUS_ROOT / "trace.json"
R7_PLUS_REPLAY_PATH = R7_PLUS_ROOT / "replay_boundaries.npz"
R7_PLUS_SUMMARY_PATH = R7_PLUS_ROOT / "summary.json"
R7_PLUS_GATE_PATH = R7_PLUS_ROOT / "gate.json"
R7_PLUS_RECEIPT_PATH = R7_PLUS_ROOT / "receipt.json"
R7_PLUS_ADJUDICATION_PATH = (
    COLLECTION_ROOT / "deterministic_offset_plus_0p20" / "adjudication.json"
)
R7_PLUS_ADJUDICATION_STAGE_RECEIPT_PATH = (
    COLLECTION_ROOT
    / "deterministic_offset_plus_0p20"
    / "adjudication_stage_receipt.json"
)
R7_PLUS_LABEL_STAGE_RECEIPT_PATH = (
    COLLECTION_ROOT
    / "deterministic_offset_plus_0p20"
    / "historical_label_stage_receipt.json"
)

R6_RUN_ROOT = v12r7.R6_RUN_ROOT
R6_CANDIDATE_MODULE_PATH = v12r7.R6_CANDIDATE_MODULE_PATH
R6_TERMINAL_LEDGER = v12r7.R6_TERMINAL_LEDGER
R6_CANDIDATE_FREEZE = v12r7.R6_CANDIDATE_FREEZE
BASE_CORPUS_PATH = v12r7.BASE_CORPUS_PATH
R4_PLUS_LABELS_PATH = v12r7.R4_PLUS_LABELS_PATH
SOURCE_H0_MODULE_PATH = v12r7.SOURCE_H0_MODULE_PATH
SOURCE_H0_ID = v12r7.v12r6.SOURCE_H0_ID
TEACHER_OBSERVATION_CONTRACT_ID = v12r7.v12r6.v12r5.TEACHER_ID

# The offline labeler consumes two mutable filesystem inputs after a probe has
# closed: the H0 checkpoint queried for the same-state target and the V11 P3
# corpus used by the coverage evaluator.  Their complete identities are
# protocol inputs, not merely diagnostic metadata.
LOCKED_SOURCE_H0_TREE = {
    "path": SOURCE_H0_MODULE_PATH.as_posix(),
    "tree_sha256": "f7f6c898975af109412af8c3f1a338b5076f9fefcec1e2723673fd821f1f13ee",
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
            "sha256": "44457ca5df7fa0e0e1f1d361d940136917fe8f71e984a1b0afaefb8ca3ced33b",
            "size_bytes": 604_772,
        },
    ],
}
COVERAGE_REFERENCE_CORPUS_PATH = PurePosixPath(
    "validation/h0_primary_grf_split_adaptation_runs/"
    "2026-08-09_h0_primary_split_v11_v26_weighted_full_mean_safe_dagger/"
    "adaptation/p3/corpus.npz"
)
LOCKED_COVERAGE_REFERENCE = {
    "path": COVERAGE_REFERENCE_CORPUS_PATH.as_posix(),
    "sha256": "232e0776f67e7a1425288c4f3979409df998ef34a7e60c618ec6c5d7cd9c4933",
    "size_bytes": 7_913_274,
    "rows": 6_000,
}

# Immutable inputs from the terminal R7 attempt.  Size and digest are part of
# the protocol rather than observational metadata.
LOCKED_R7_EVIDENCE = {
    "protocol_freeze": {
        "path": R7_PROTOCOL_FREEZE_PATH.as_posix(),
        "sha256": "ea6d4b795c33696f6fbdf54557e907b8a4ff819dbfdb1cc0edf59f69d07ea32b",
        "size_bytes": 32_496,
    },
    "execution_lock": {
        "path": R7_EXECUTION_LOCK_PATH.as_posix(),
        "sha256": "fe3fc9a0f4f06d3df83cd0bfe99eadf99d77ea21074053815fad47ef6e2fab1c",
        "size_bytes": 27_141,
    },
    "terminal_ledger": {
        "path": R7_TERMINAL_LEDGER_PATH.as_posix(),
        "sha256": "8c7761d09625241a80311535466c4acc2bb70f62fd895d1cb1b9627fc6292ae8",
        "size_bytes": 3_072,
    },
    "plus_trace": {
        "path": R7_PLUS_TRACE_PATH.as_posix(),
        "sha256": "d259b9a69ab63c3dab354ad7e228d628c58ee9fd5c972e64d8f95dded3483688",
        "size_bytes": 2_099_730,
    },
    "plus_replay": {
        "path": R7_PLUS_REPLAY_PATH.as_posix(),
        "sha256": "10b2b9e55364aeb3c4070c77dd8cc1cff12e3bd144da0ec916d7a739b69a14db",
        "size_bytes": 42_159,
    },
    "plus_summary": {
        "path": R7_PLUS_SUMMARY_PATH.as_posix(),
        "sha256": "b07d25996e144f1102dd98e483d5912903e06f400dc52ccb4db5ce5e4bfd399b",
        "size_bytes": 8_245,
    },
    "plus_gate": {
        "path": R7_PLUS_GATE_PATH.as_posix(),
        "sha256": "d6147af3404b1d8386718f2956de51fbb3efde5a5754d63ffe430607916a03f0",
        "size_bytes": 2_676,
    },
    "plus_receipt": {
        "path": R7_PLUS_RECEIPT_PATH.as_posix(),
        "sha256": "027a99250cc050a5c771fd9bb6056f17177779c051b7934ca88ee8a7f0d84f09",
        "size_bytes": 2_704,
    },
}

R7_CONTRACT_SOURCE_RECORD = {
    "path": (
        "Trajectory Generator/baseline_MLP/validation/v12r7/"
        "h0_v12r7_recovery_contract.py"
    ),
    "sha256": "3333d65cf20b238ef80637808a91c103b084edc889fb6acf890f5a754ad0974a",
    "size_bytes": 18_899,
}
R7_TERMINAL_FAIL_STATUS = "FAIL_H0_V12R7_RECOVERY_PIPELINE_TERMINAL"
R7_TERMINAL_STAGE = "collect_label__deterministic_offset_plus_0p20"

LOCKED_INPUTS = copy.deepcopy(v12r7.LOCKED_INPUTS)
LOCKED_INPUTS["source_h0"] = copy.deepcopy(LOCKED_SOURCE_H0_TREE)
LOCKED_INPUTS["coverage_reference_corpus"] = copy.deepcopy(LOCKED_COVERAGE_REFERENCE)
EXPECTED_ACTOR_FEATURES = v12r7.EXPECTED_ACTOR_FEATURES
EXPECTED_FULL_FEATURES = v12r7.EXPECTED_FULL_FEATURES
EXPECTED_ACTION_DIM = v12r7.EXPECTED_ACTION_DIM
EXPECTED_STEPS = v12r7.EXPECTED_STEPS
RAW_SAMPLES_PER_STEP = v12r7.RAW_SAMPLES_PER_STEP
EXPECTED_SIGMA = v12r7.EXPECTED_SIGMA
DISABLED_CLOCK_COLUMNS = v12r7.DISABLED_CLOCK_COLUMNS
PENETRATION_LIMIT_M = v12r7.PENETRATION_LIMIT_M
MINIMUM_VALID_CYCLES = v12r7.MINIMUM_VALID_CYCLES
MINIMUM_RECOVERABLE_PREFIX_STEPS = v12r7.MINIMUM_RECOVERABLE_PREFIX_STEPS
TARGET_CONTRACT_ID = v12r7.TARGET_CONTRACT_ID
EVENT_CONTRACT_ID = v12r7.EVENT_CONTRACT_ID

COLLECTION_CASE_IDS = tuple(v12r7.COLLECTION_CASE_IDS)
HISTORICAL_CASE_ID = COLLECTION_CASE_IDS[0]
NEW_COLLECTION_CASE_IDS = COLLECTION_CASE_IDS[1:]
DEVELOPMENT_CASE_IDS = COLLECTION_CASE_IDS


def _r8_collection_case(case_id: str) -> dict[str, Any]:
    case = v12r7.canonical_collection_case(case_id)
    case["destination"] = (COLLECTION_ROOT / case_id).as_posix()
    case["behavior"] = "FROZEN_R6_PURE_POLICY_R8_OBSERVER_ONLY"
    case["source_kind"] = (
        "IMMUTABLE_R7_ADJUDICATED_PREFIX"
        if case_id == HISTORICAL_CASE_ID
        else "NEW_R8_NORMALIZED_PROBE"
    )
    return case


COLLECTION_CASES = tuple(
    _r8_collection_case(case_id) for case_id in COLLECTION_CASE_IDS
)
DEVELOPMENT_CASES = tuple(
    {
        **v12r7.canonical_development_case(case_id),
        "destination": (DEVELOPMENT_ROOT / case_id).as_posix(),
        "behavior": "R8_PURE_UNBLENDED_NO_TEACHER_NO_LATCH",
    }
    for case_id in DEVELOPMENT_CASE_IDS
)

FIT = copy.deepcopy(v12r7.FIT)
FIT.update(
    {
        "fit_contract_id": FIT_CONTRACT_ID,
        "observer_cases": list(COLLECTION_CASE_IDS),
        "historical_adjudicated_cases": [HISTORICAL_CASE_ID],
        "new_probe_cases": list(NEW_COLLECTION_CASE_IDS),
    }
)
OFFLINE_THRESHOLDS = copy.deepcopy(v12r7.OFFLINE_THRESHOLDS)

STAGE_IDS = (
    "adjudicate_r7_plus_prefix",
    "label_r7_plus_prefix",
    *(f"collect_label__{case_id}" for case_id in NEW_COLLECTION_CASE_IDS),
    "fit_recovery_actor",
    "freeze_recovery_actor",
    *(f"development__{case_id}" for case_id in DEVELOPMENT_CASE_IDS),
    "finalize_development",
)


def canonical_collection_case(case_id: str) -> dict[str, Any]:
    if case_id not in COLLECTION_CASE_IDS:
        raise ValueError(f"unknown V12R8 collection case: {case_id!r}")
    return copy.deepcopy(_r8_collection_case(case_id))


def canonical_development_case(case_id: str) -> dict[str, Any]:
    for case in DEVELOPMENT_CASES:
        if case["case_id"] == case_id:
            return copy.deepcopy(case)
    raise ValueError(f"unknown V12R8 development case: {case_id!r}")


def collection_case_root(case_id: str) -> PurePosixPath:
    canonical_collection_case(case_id)
    return COLLECTION_ROOT / case_id


def observer_label_path(case_id: str) -> PurePosixPath:
    return collection_case_root(case_id) / "observer_labels" / "labels.npz"


def candidate_id(tree_sha256: str) -> str:
    if (
        not isinstance(tree_sha256, str)
        or len(tree_sha256) != 64
        or any(char not in "0123456789abcdef" for char in tree_sha256)
    ):
        raise ValueError("candidate tree digest must be lowercase SHA-256")
    return f"AB06_H0_V12R8_RECOVERY_W512:{tree_sha256}"


def _restat_gate(
    result: Mapping[str, Any], pass_status: str, fail_status: str
) -> dict[str, Any]:
    value = copy.deepcopy(dict(result))
    value["status"] = pass_status if value.get("passed") is True else fail_status
    return value


def collection_integrity_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    return _restat_gate(
        v12r7.collection_integrity_gate(summary),
        "PASS_H0_V12R8_COLLECTION_INTEGRITY",
        "FAIL_H0_V12R8_COLLECTION_INTEGRITY",
    )


def label_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    return _restat_gate(
        v12r7.label_gate(summary),
        "PASS_H0_V12R8_OBSERVER_LABELS",
        "FAIL_H0_V12R8_OBSERVER_LABELS",
    )


def fit_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    return _restat_gate(
        v12r7.fit_gate(summary),
        "PASS_H0_V12R8_RECOVERY_FIT",
        "FAIL_H0_V12R8_RECOVERY_FIT",
    )


def development_gate(summary: Mapping[str, Any], *, case_id: str) -> dict[str, Any]:
    return _restat_gate(
        v12r7.development_gate(summary, case_id=case_id),
        "PASS_H0_V12R8_PURE_DEVELOPMENT",
        "FAIL_H0_V12R8_PURE_DEVELOPMENT",
    )


def contract_self_check() -> dict[str, Any]:
    checks = {
        "new_namespace": ROOT != v12r7.ROOT,
        "identity_separated": PROTOCOL_ID != v12r7.PROTOCOL_ID
        and PIPELINE_ID != v12r7.PIPELINE_ID,
        "one_historical_five_new": HISTORICAL_CASE_ID
        == "deterministic_offset_plus_0p20"
        and len(NEW_COLLECTION_CASE_IDS) == 5
        and len(set(COLLECTION_CASE_IDS)) == 6,
        "historical_rerun_forbidden": AUTHORITY["historical_r7_prefix_rerun_authorized"]
        is False,
        "same_fit": FIT["architecture"] == v12r7.FIT["architecture"]
        and FIT["stratum_count"] == 13
        and FIT["actor_fit_count"] == 1,
        "same_six_developments": DEVELOPMENT_CASE_IDS == v12r7.DEVELOPMENT_CASE_IDS,
        "offline_label_inputs_locked": LOCKED_SOURCE_H0_TREE["path"]
        == SOURCE_H0_MODULE_PATH.as_posix()
        and LOCKED_SOURCE_H0_TREE["file_count"] == 3
        and {row["path"] for row in LOCKED_SOURCE_H0_TREE["files"]}
        == {"class_and_ctor_args.pkl", "metadata.json", "module_state.pkl"}
        and LOCKED_COVERAGE_REFERENCE["rows"] == 6_000,
        "v26_unchanged": EVENT_CONTRACT_ID == v12r7.EVENT_CONTRACT_ID
        and TARGET_CONTRACT_ID == v12r7.TARGET_CONTRACT_ID,
        "thresholds_unchanged": OFFLINE_THRESHOLDS == v12r7.OFFLINE_THRESHOLDS
        and PENETRATION_LIMIT_M == v12r7.PENETRATION_LIMIT_M,
        "q3_semantic_interface": isinstance(ACTOR_FEATURE_MANIFEST_STATUS, str)
        and CANDIDATE_MODULE_PATH.name == "rl_module_recovery",
        "no_retry_resume_sweep": AUTHORITY["retry_authorized"] is False
        and AUTHORITY["resume_authorized"] is False
        and AUTHORITY["alpha_sweep_authorized"] is False,
    }
    passed = all(checks.values())
    return {
        "status": (
            "PASS_H0_V12R8_RECOVERY_CONTRACT"
            if passed
            else "FAIL_H0_V12R8_RECOVERY_CONTRACT"
        ),
        "passed": passed,
        "checks": checks,
    }


__all__ = [name for name in globals() if name.isupper()] + [
    "candidate_id",
    "canonical_collection_case",
    "canonical_development_case",
    "collection_case_root",
    "collection_integrity_gate",
    "contract_self_check",
    "development_gate",
    "fit_gate",
    "label_gate",
    "observer_label_path",
]
