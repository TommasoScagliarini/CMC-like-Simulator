"""Execution-free contract for the additive V12R9 recovery successor.

V12R9 treats the terminal V12R8 one-shot as immutable evidence.  It imports
the already closed historical plus labels, adjudicates and labels the valid
252-step minus prefix offline, collects only the four cases that V12R8 never
started, then keeps the same single W512/13-stratum fit and six pure-policy
development design.  Importing this module performs no I/O or execution.
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
    _LOCAL_VALIDATION / "v12r8",
    _LOCAL_VALIDATION / "v12r7",
    _LOCAL_VALIDATION / "v12r6",
    _LOCAL_VALIDATION,
    _REPO_ROOT / "validation",
    _REPO_ROOT / "Trajectory Generator" / "baseline_MLP",
    _REPO_ROOT,
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_v12r8_recovery_contract as v12r8  # noqa: E402


SCHEMA_VERSION = 1290
REVISION = "2026-08-14"
PROTOCOL_ID = "AB06_H0_V12R9_ADDITIVE_RECOVERY_W512_V26"
PIPELINE_ID = "H0_V12R9_R8_PREFIX_IMPORT_SINGLE_FIT"
FIT_CONTRACT_ID = "h0_v12r9_additive_recovery_full_mean_w512_v1"
TOPOLOGY_ID = "V12R9_STANDARD_RECOVERY_W512_V1"
CANDIDATE_SELECTION_RULE = "SOLE_R9_SINGLE_FIT_OUTPUT_IF_ALL_LOCKED_GATES_PASS"
PROTOCOL_FREEZE_PASS_STATUS = "PASS_H0_V12R9_RECOVERY_PROTOCOL_FREEZE"
EXECUTION_LOCK_PASS_STATUS = "PASS_H0_V12R9_RECOVERY_EXECUTION_LOCK"
CANDIDATE_FREEZE_PASS_STATUS = "PASS_H0_V12R9_RECOVERY_CANDIDATE_FREEZE"
DEVELOPMENT_PASS_STATUS = "PASS_H0_V12R9_RECOVERY_DEVELOPMENT"
PIPELINE_TERMINAL_PASS_STATUS = "PASS_H0_V12R9_RECOVERY_PIPELINE_TERMINAL"
ACTOR_FEATURE_MANIFEST_STATUS = "H0_V12R9_RECOVERY_ACTOR_FEATURE_CONTRACT"

AUTHORITY = {
    **copy.deepcopy(v12r8.AUTHORITY),
    "authority_date": REVISION,
    "historical_r7_prefix_adjudication_authorized": False,
    "historical_r7_prefix_rerun_authorized": False,
    "terminal_r8_adjudication_authorized": True,
    "terminal_r8_retry_authorized": False,
    "terminal_r8_resume_authorized": False,
    "r8_plus_label_import_authorized": True,
    "r8_minus_offline_label_authorized": True,
}

ROOT = PurePosixPath("Trajectory Generator/baseline_MLP/validation/v12r9")
PROTOCOL_FREEZE_PATH = ROOT / "h0_v12r9_recovery_protocol_freeze.json"
EXECUTION_LOCK_PATH = ROOT / "h0_v12r9_recovery_execution_lock.json"
RUN_ROOT = ROOT / "h0_v12r9_run_20260814"
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

# Immutable V12R8 terminal evidence.  V12R9 never writes below R8_ROOT.
R8_ROOT = v12r8.ROOT
R8_PROTOCOL_FREEZE_PATH = v12r8.PROTOCOL_FREEZE_PATH
R8_EXECUTION_LOCK_PATH = v12r8.EXECUTION_LOCK_PATH
R8_RUN_ROOT = v12r8.RUN_ROOT
R8_PIPELINE_CLAIM_PATH = v12r8.CLAIM_PATH
R8_TERMINAL_LEDGER_PATH = v12r8.LEDGER_PATH
R8_CLAIMS_ROOT = R8_RUN_ROOT / "claims"
R8_MINUS_ROOT = v12r8.COLLECTION_ROOT / "deterministic_offset_minus_0p20"
R8_MINUS_RUN_START_PATH = R8_MINUS_ROOT / "run_start.json"
R8_MINUS_TRACE_PATH = R8_MINUS_ROOT / "trace.json"
R8_MINUS_PARTIAL_SUMMARY_PATH = R8_MINUS_ROOT / "partial_summary.json"
R8_MINUS_SUMMARY_PATH = R8_MINUS_ROOT / "summary.json"
R8_MINUS_GATE_PATH = R8_MINUS_ROOT / "gate.json"
R8_MINUS_REPLAY_PATH = R8_MINUS_ROOT / "replay_boundaries.npz"
R8_MINUS_RECEIPT_PATH = R8_MINUS_ROOT / "receipt.json"
R8_PLUS_ROOT = v12r8.COLLECTION_ROOT / "deterministic_offset_plus_0p20"
R8_PLUS_LABEL_ROOT = R8_PLUS_ROOT / "observer_labels"
R8_PLUS_LABELS_PATH = R8_PLUS_LABEL_ROOT / "labels.npz"
R8_PLUS_LABEL_SUMMARY_PATH = R8_PLUS_LABEL_ROOT / "summary.json"
R8_PLUS_LABEL_GATE_PATH = R8_PLUS_LABEL_ROOT / "gate.json"
R8_PLUS_LABEL_RECEIPT_PATH = R8_PLUS_LABEL_ROOT / "receipt.json"
R8_PLUS_LABEL_STAGE_RECEIPT_PATH = R8_PLUS_ROOT / "historical_label_stage_receipt.json"

R8_TERMINAL_FAIL_STATUS = "FAIL_H0_V12R8_RECOVERY_PIPELINE_TERMINAL"
R8_TERMINAL_STAGE = "collect_label__deterministic_offset_minus_0p20"
R8_TERMINAL_ERROR_TYPE = "V12R8RecoveryProbeError"
R8_TERMINAL_ERROR_MESSAGE = "probe summary identity drifted"

R8_MINUS_ADJUDICATION_PATH = (
    COLLECTION_ROOT / "deterministic_offset_minus_0p20" / "adjudication.json"
)
R8_MINUS_ADJUDICATION_STAGE_RECEIPT_PATH = (
    COLLECTION_ROOT
    / "deterministic_offset_minus_0p20"
    / "adjudication_stage_receipt.json"
)
R8_PLUS_IMPORT_STAGE_RECEIPT_PATH = (
    COLLECTION_ROOT / "deterministic_offset_plus_0p20" / "import_stage_receipt.json"
)
R8_MINUS_LABEL_STAGE_RECEIPT_PATH = (
    COLLECTION_ROOT
    / "deterministic_offset_minus_0p20"
    / "historical_label_stage_receipt.json"
)

R8_CONTRACT_SOURCE_RECORD = {
    "path": (
        "Trajectory Generator/baseline_MLP/validation/v12r8/"
        "h0_v12r8_recovery_contract.py"
    ),
    "sha256": "787a1af772198e7bfe486b1c92203d4cc6b504ecb6d34a876c82ccfe1b622328",
    "size_bytes": 14_845,
}
R8_PROBE_SOURCE_RECORD = {
    "path": (
        "Trajectory Generator/baseline_MLP/validation/v12r8/h0_v12r8_recovery_probe.py"
    ),
    "sha256": "d3b1775ee60959d69bd8b0eab82ae8c096463a8ef52ef7f62cd73767264ad758",
    "size_bytes": 76_133,
}
R8_RUNNER_SOURCE_RECORD = {
    "path": (
        "Trajectory Generator/baseline_MLP/validation/v12r8/run_h0_v12r8_recovery.py"
    ),
    "sha256": "4fede5bfd543ed4cc88b887658e74a11a52837df59867618016da5d5b72124a2",
    "size_bytes": 94_564,
}

LOCKED_R8_EVIDENCE = {
    "protocol_freeze": {
        "path": R8_PROTOCOL_FREEZE_PATH.as_posix(),
        "sha256": "fa593d9ffc213805d47fb9d1c58a63db7aae2fdc337f479eefdffcfb9a9c8710",
        "size_bytes": 43_805,
    },
    "execution_lock": {
        "path": R8_EXECUTION_LOCK_PATH.as_posix(),
        "sha256": "44ecb103398982cfa9f0c267748e2f6fec466f0fdafd5639cef20bc5a1f0c3f0",
        "size_bytes": 37_784,
    },
    "pipeline_claim": {
        "path": R8_PIPELINE_CLAIM_PATH.as_posix(),
        "sha256": "d1a277dac0d997e6eb996370365d71227b86c59da336f407aca119fbf7bb4b1b",
        "size_bytes": 2_527,
    },
    "attempted_worker_claim": {
        "path": (
            R8_CLAIMS_ROOT / "03_collect_label__deterministic_offset_minus_0p20.json"
        ).as_posix(),
        "sha256": "1d89657367c2792ef81c2ff82b0aa4d9972e605a81b538afeba997730a087722",
        "size_bytes": 1_176,
    },
    "terminal_ledger": {
        "path": R8_TERMINAL_LEDGER_PATH.as_posix(),
        "sha256": "d5564c9b96976453033885010cc23b709d7071d63c5380fac62f7822485344b9",
        "size_bytes": 4_454,
    },
    "minus_run_start": {
        "path": R8_MINUS_RUN_START_PATH.as_posix(),
        "sha256": "7fae2d264ec6a27c850c840e86c257d9537acaace3def35d3b6eb8492d90b77b",
        "size_bytes": 2_236,
    },
    "minus_trace": {
        "path": R8_MINUS_TRACE_PATH.as_posix(),
        "sha256": "d6ae27641ab7832c23832290cfd234f2758c02f0ad2afbf4b749404507f67087",
        "size_bytes": 2_967_372,
    },
    "minus_partial_summary": {
        "path": R8_MINUS_PARTIAL_SUMMARY_PATH.as_posix(),
        "sha256": "7bb4b7995eec218e728c2ed6d3e3641ffb0039aa8864955a877f1ecb01e164bb",
        "size_bytes": 375,
    },
    "minus_summary": {
        "path": R8_MINUS_SUMMARY_PATH.as_posix(),
        "sha256": "97cf526f612d3ee4f2a835660f31c0c634ef9d4550f2598660f38d077434d6d4",
        "size_bytes": 9_581,
    },
    "minus_gate": {
        "path": R8_MINUS_GATE_PATH.as_posix(),
        "sha256": "c189a4b19b4cdd2cc65b6117a9386068bd3877dcbfab6d1b31bc639111ab9f7f",
        "size_bytes": 2_427,
    },
    "minus_replay": {
        "path": R8_MINUS_REPLAY_PATH.as_posix(),
        "sha256": "f920419987b3c1f92ef1b99a661f04e5c6bf821473fbee7e2a8e28addb0a670f",
        "size_bytes": 55_448,
    },
    "minus_receipt": {
        "path": R8_MINUS_RECEIPT_PATH.as_posix(),
        "sha256": "9da6b83f6c2bde4d096f3c5e10120653f7e0aab4a35bf5ac34340a29132867d9",
        "size_bytes": 3_190,
    },
    "plus_labels": {
        "path": R8_PLUS_LABELS_PATH.as_posix(),
        "sha256": "c00a49b2515731eacc397872bc033f1ae00bf680bcae93d8a7433e4ab725e5ef",
        "size_bytes": 254_326,
    },
    "plus_label_summary": {
        "path": R8_PLUS_LABEL_SUMMARY_PATH.as_posix(),
        "sha256": "1a905814783c7089081acdafceb7028746e477320e6f9ad4f3b839a0680f015d",
        "size_bytes": 7_530,
    },
    "plus_label_gate": {
        "path": R8_PLUS_LABEL_GATE_PATH.as_posix(),
        "sha256": "9cb72ef5c159fa0a91b9e927cb526243c7eb2584fd6236f95f66fe94fc56b425",
        "size_bytes": 314,
    },
    "plus_label_receipt": {
        "path": R8_PLUS_LABEL_RECEIPT_PATH.as_posix(),
        "sha256": "c540af1b3f3055b46f84ddf9dd305781eec9adc17a0bc07f29b5e210f628145c",
        "size_bytes": 3_201,
    },
    "plus_label_stage_receipt": {
        "path": R8_PLUS_LABEL_STAGE_RECEIPT_PATH.as_posix(),
        "sha256": "995064c4fa2627a44740de1d806ef02a8c3fe155d3565e8bbea9f6c8a6b38376",
        "size_bytes": 3_671,
    },
}

FULL_R6_CANDIDATE_TREE = {
    "path": v12r8.R6_CANDIDATE_MODULE_PATH.as_posix(),
    "tree_sha256": "340c2c65c2300a90ce46c09837e679a99e5dea09ce3935574ef5345fafb709f3",
    "file_count": 5,
    "files": [
        {
            "path": "actor_feature_manifest.json",
            "sha256": "8589fb9b8226530a4ec3cb240257f544d1cd52abf2fea27b451a72815fd78076",
            "size_bytes": 1_603,
        },
        {
            "path": "class_and_ctor_args.pkl",
            "sha256": "a5cc979046fbb4dde936d4d3f2d913e16448e8cea5a8f03762f60145dbe674b0",
            "size_bytes": 2_262,
        },
        {
            "path": "composite_build_manifest.json",
            "sha256": "210789ce939d9b841f28584438c06f14f04fc80390d676c28d749dc790dc469b",
            "size_bytes": 16_921,
        },
        {
            "path": "metadata.json",
            "sha256": "3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12",
            "size_bytes": 197,
        },
        {
            "path": "module_state.pkl",
            "sha256": "d49640f160c925d0f049a9064fbb87cfe6d5e0a789af4af2d6bc2bb33795cdb1",
            "size_bytes": 2_257_526,
        },
    ],
}

LOCKED_SOURCE_H0_TREE = copy.deepcopy(v12r8.LOCKED_SOURCE_H0_TREE)
LOCKED_COVERAGE_REFERENCE = copy.deepcopy(v12r8.LOCKED_COVERAGE_REFERENCE)
LOCKED_INPUTS = copy.deepcopy(v12r8.LOCKED_INPUTS)
LOCKED_INPUTS["r6_candidate"] = copy.deepcopy(FULL_R6_CANDIDATE_TREE)
LOCKED_INPUTS["source_h0"] = copy.deepcopy(LOCKED_SOURCE_H0_TREE)
LOCKED_INPUTS["coverage_reference_corpus"] = copy.deepcopy(LOCKED_COVERAGE_REFERENCE)

R6_RUN_ROOT = v12r8.R6_RUN_ROOT
R6_CANDIDATE_MODULE_PATH = v12r8.R6_CANDIDATE_MODULE_PATH
R6_TERMINAL_LEDGER = v12r8.R6_TERMINAL_LEDGER
R6_CANDIDATE_FREEZE = v12r8.R6_CANDIDATE_FREEZE
BASE_CORPUS_PATH = v12r8.BASE_CORPUS_PATH
R4_PLUS_LABELS_PATH = v12r8.R4_PLUS_LABELS_PATH
SOURCE_H0_MODULE_PATH = v12r8.SOURCE_H0_MODULE_PATH
SOURCE_H0_ID = v12r8.SOURCE_H0_ID
TEACHER_OBSERVATION_CONTRACT_ID = v12r8.TEACHER_OBSERVATION_CONTRACT_ID
EXPECTED_ACTOR_FEATURES = v12r8.EXPECTED_ACTOR_FEATURES
EXPECTED_FULL_FEATURES = v12r8.EXPECTED_FULL_FEATURES
EXPECTED_ACTION_DIM = v12r8.EXPECTED_ACTION_DIM
EXPECTED_STEPS = v12r8.EXPECTED_STEPS
RAW_SAMPLES_PER_STEP = v12r8.RAW_SAMPLES_PER_STEP
EXPECTED_SIGMA = v12r8.EXPECTED_SIGMA
DISABLED_CLOCK_COLUMNS = v12r8.DISABLED_CLOCK_COLUMNS
PENETRATION_LIMIT_M = v12r8.PENETRATION_LIMIT_M
MINIMUM_VALID_CYCLES = v12r8.MINIMUM_VALID_CYCLES
MINIMUM_RECOVERABLE_PREFIX_STEPS = v12r8.MINIMUM_RECOVERABLE_PREFIX_STEPS
TARGET_CONTRACT_ID = v12r8.TARGET_CONTRACT_ID
EVENT_CONTRACT_ID = v12r8.EVENT_CONTRACT_ID
COVERAGE_REFERENCE_CORPUS_PATH = v12r8.COVERAGE_REFERENCE_CORPUS_PATH
OFFLINE_THRESHOLDS = copy.deepcopy(v12r8.OFFLINE_THRESHOLDS)

COLLECTION_CASE_IDS = tuple(v12r8.COLLECTION_CASE_IDS)
HISTORICAL_CASE_ID = COLLECTION_CASE_IDS[0]
IMPORTED_MINUS_CASE_ID = COLLECTION_CASE_IDS[1]
IMPORTED_CASE_IDS = COLLECTION_CASE_IDS[:2]
NEW_COLLECTION_CASE_IDS = COLLECTION_CASE_IDS[2:]
DEVELOPMENT_CASE_IDS = COLLECTION_CASE_IDS


def _r9_collection_case(case_id: str) -> dict[str, Any]:
    case = v12r8.canonical_collection_case(case_id)
    case["destination"] = (COLLECTION_ROOT / case_id).as_posix()
    case["behavior"] = "FROZEN_R6_PURE_POLICY_R9_OBSERVER_ONLY"
    if case_id == HISTORICAL_CASE_ID:
        source_kind = "IMMUTABLE_R8_BYTE_EXACT_LABEL_IMPORT"
    elif case_id == IMPORTED_MINUS_CASE_ID:
        source_kind = "IMMUTABLE_R8_ADJUDICATED_PREFIX_OFFLINE_LABEL"
    else:
        source_kind = "NEW_R9_FULL_TREE_CLOSED_PROBE"
    case["source_kind"] = source_kind
    return case


COLLECTION_CASES = tuple(
    _r9_collection_case(case_id) for case_id in COLLECTION_CASE_IDS
)
DEVELOPMENT_CASES = tuple(
    {
        **v12r8.canonical_development_case(case_id),
        "destination": (DEVELOPMENT_ROOT / case_id).as_posix(),
        "behavior": "R9_PURE_UNBLENDED_NO_TEACHER_NO_LATCH",
    }
    for case_id in DEVELOPMENT_CASE_IDS
)

FIT = copy.deepcopy(v12r8.FIT)
FIT.update(
    {
        "fit_contract_id": FIT_CONTRACT_ID,
        "observer_cases": list(COLLECTION_CASE_IDS),
        "imported_label_cases": [HISTORICAL_CASE_ID],
        "adjudicated_prefix_cases": [IMPORTED_MINUS_CASE_ID],
        "new_probe_cases": list(NEW_COLLECTION_CASE_IDS),
    }
)

STAGE_IDS = (
    "adjudicate_r8_terminal_and_minus_prefix",
    "import_r8_plus_labels",
    "label_r8_minus_prefix",
    *(f"collect_label__{case_id}" for case_id in NEW_COLLECTION_CASE_IDS),
    "fit_recovery_actor",
    "freeze_recovery_actor",
    *(f"development__{case_id}" for case_id in DEVELOPMENT_CASE_IDS),
    "finalize_development",
)


def canonical_collection_case(case_id: str) -> dict[str, Any]:
    if case_id not in COLLECTION_CASE_IDS:
        raise ValueError(f"unknown V12R9 collection case: {case_id!r}")
    return copy.deepcopy(_r9_collection_case(case_id))


def canonical_development_case(case_id: str) -> dict[str, Any]:
    for case in DEVELOPMENT_CASES:
        if case["case_id"] == case_id:
            return copy.deepcopy(case)
    raise ValueError(f"unknown V12R9 development case: {case_id!r}")


def collection_case_root(case_id: str) -> PurePosixPath:
    canonical_collection_case(case_id)
    return COLLECTION_ROOT / case_id


def observer_label_path(case_id: str) -> PurePosixPath:
    canonical_collection_case(case_id)
    if case_id == HISTORICAL_CASE_ID:
        return R8_PLUS_LABELS_PATH
    return collection_case_root(case_id) / "observer_labels" / "labels.npz"


def candidate_id(tree_sha256: str) -> str:
    if (
        not isinstance(tree_sha256, str)
        or len(tree_sha256) != 64
        or any(char not in "0123456789abcdef" for char in tree_sha256)
    ):
        raise ValueError("candidate tree digest must be lowercase SHA-256")
    return f"AB06_H0_V12R9_RECOVERY_W512:{tree_sha256}"


def _restat_gate(
    result: Mapping[str, Any], pass_status: str, fail_status: str
) -> dict[str, Any]:
    value = copy.deepcopy(dict(result))
    value["status"] = pass_status if value.get("passed") is True else fail_status
    return value


def collection_integrity_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    return _restat_gate(
        v12r8.collection_integrity_gate(summary),
        "PASS_H0_V12R9_COLLECTION_INTEGRITY",
        "FAIL_H0_V12R9_COLLECTION_INTEGRITY",
    )


def label_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    return _restat_gate(
        v12r8.label_gate(summary),
        "PASS_H0_V12R9_OBSERVER_LABELS",
        "FAIL_H0_V12R9_OBSERVER_LABELS",
    )


def fit_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    return _restat_gate(
        v12r8.fit_gate(summary),
        "PASS_H0_V12R9_RECOVERY_FIT",
        "FAIL_H0_V12R9_RECOVERY_FIT",
    )


def development_gate(summary: Mapping[str, Any], *, case_id: str) -> dict[str, Any]:
    return _restat_gate(
        v12r8.development_gate(summary, case_id=case_id),
        "PASS_H0_V12R9_PURE_DEVELOPMENT",
        "FAIL_H0_V12R9_PURE_DEVELOPMENT",
    )


def contract_self_check() -> dict[str, Any]:
    projection = {
        name: FULL_R6_CANDIDATE_TREE[name]
        for name in ("path", "tree_sha256", "file_count")
    }
    checks = {
        "new_namespace": ROOT != v12r8.ROOT,
        "identity_separated": PROTOCOL_ID != v12r8.PROTOCOL_ID
        and PIPELINE_ID != v12r8.PIPELINE_ID,
        "one_import_one_adjudicated_four_new": IMPORTED_CASE_IDS
        == (
            "deterministic_offset_plus_0p20",
            "deterministic_offset_minus_0p20",
        )
        and len(NEW_COLLECTION_CASE_IDS) == 4
        and len(set(COLLECTION_CASE_IDS)) == 6,
        "stage_order_unique_and_complete": len(STAGE_IDS) == 16
        and len(set(STAGE_IDS)) == len(STAGE_IDS)
        and STAGE_IDS.count("freeze_recovery_actor") == 1,
        "r8_immutable_no_retry": AUTHORITY["terminal_r8_retry_authorized"] is False
        and AUTHORITY["terminal_r8_resume_authorized"] is False,
        "full_candidate_lock": set(FULL_R6_CANDIDATE_TREE)
        == {"path", "tree_sha256", "file_count", "files"}
        and len(FULL_R6_CANDIDATE_TREE["files"]) == 5
        and LOCKED_INPUTS["r6_candidate"] == FULL_R6_CANDIDATE_TREE,
        "r8_projection_explained": projection == v12r8.LOCKED_INPUTS["r6_candidate"]
        and FULL_R6_CANDIDATE_TREE != v12r8.LOCKED_INPUTS["r6_candidate"],
        "same_fit": FIT["architecture"] == v12r8.FIT["architecture"]
        and FIT["stratum_count"] == 13
        and FIT["actor_fit_count"] == 1,
        "same_six_developments": DEVELOPMENT_CASE_IDS == v12r8.DEVELOPMENT_CASE_IDS,
        "offline_inputs_locked": LOCKED_SOURCE_H0_TREE["file_count"] == 3
        and LOCKED_COVERAGE_REFERENCE["rows"] == 6_000,
        "v26_unchanged": EVENT_CONTRACT_ID == v12r8.EVENT_CONTRACT_ID
        and TARGET_CONTRACT_ID == v12r8.TARGET_CONTRACT_ID,
        "q3_semantic_interface": isinstance(ACTOR_FEATURE_MANIFEST_STATUS, str)
        and CANDIDATE_MODULE_PATH.name == "rl_module_recovery",
        "no_retry_resume_sweep": AUTHORITY["retry_authorized"] is False
        and AUTHORITY["resume_authorized"] is False
        and AUTHORITY["alpha_sweep_authorized"] is False,
    }
    passed = all(checks.values())
    return {
        "status": (
            "PASS_H0_V12R9_RECOVERY_CONTRACT"
            if passed
            else "FAIL_H0_V12R9_RECOVERY_CONTRACT"
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
