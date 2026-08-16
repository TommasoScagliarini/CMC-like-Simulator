"""Execution-free contract for the canonical import-only H0 V12R10 recovery.

V12R10 never recollects or relabels data.  It imports the immutable terminal
V12R9 corpus and its six closed observer-label sets, uses the failed V12R9
actor only as hidden-layer initialization, and fits one ordinary standard
``35 -> 1024 -> 1024 -> 2`` actor.  The fit schedule is the byte-deterministic
two-phase W1024 strategy demonstrated by the locked diagnostic evidence.

Importing this module performs no I/O and starts no fit or environment.
"""

from __future__ import annotations

import copy
import math
import sys
from pathlib import Path, PurePosixPath
from typing import Any, Mapping


_SOURCE = Path(__file__).resolve()
_LOCAL_VALIDATION = _SOURCE.parent.parent
_REPO_ROOT = _SOURCE.parents[4]
for _root in (
    _SOURCE.parent,
    _LOCAL_VALIDATION / "v12r9",
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

import h0_v12r9_recovery_contract as v12r9  # noqa: E402


SCHEMA_VERSION = 1300
REVISION = "2026-08-15"
PROTOCOL_ID = "AB06_H0_V12R10_IMPORT_ONLY_RECOVERY_W1024_V26"
PIPELINE_ID = "H0_V12R10_R9_TERMINAL_IMPORT_ONLY_GATE_ALIGNED_SINGLE_FIT"
FIT_CONTRACT_ID = "h0_v12r10_import_only_gate_aligned_w1024_v1"
TOPOLOGY_ID = "V12R10_STANDARD_IMPORT_ONLY_RECOVERY_W1024_V1"
CANDIDATE_SELECTION_RULE = "SOLE_R10_SINGLE_FIT_OUTPUT_IF_ALL_LOCKED_GATES_PASS"
PROTOCOL_FREEZE_PASS_STATUS = "PASS_H0_V12R10_RECOVERY_PROTOCOL_FREEZE"
EXECUTION_LOCK_PASS_STATUS = "PASS_H0_V12R10_RECOVERY_EXECUTION_LOCK"
CANDIDATE_FREEZE_PASS_STATUS = "PASS_H0_V12R10_RECOVERY_CANDIDATE_FREEZE"
DEVELOPMENT_PASS_STATUS = "PASS_H0_V12R10_RECOVERY_DEVELOPMENT"
PIPELINE_TERMINAL_PASS_STATUS = "PASS_H0_V12R10_RECOVERY_PIPELINE_TERMINAL"
ACTOR_FEATURE_MANIFEST_STATUS = "H0_V12R10_RECOVERY_ACTOR_FEATURE_CONTRACT"

AUTHORITY = {
    **copy.deepcopy(v12r9.AUTHORITY),
    "authority_date": REVISION,
    "new_environment_collection_authorized": False,
    "observer_only_teacher_queries_authorized": False,
    "r9_terminal_import_authorized": True,
    "r9_terminal_retry_authorized": False,
    "r9_terminal_resume_authorized": False,
    "r9_candidate_promotion_authorized": False,
    "r9_candidate_initialization_only_authorized": True,
    "r9_corpus_import_authorized": True,
    "r9_label_import_authorized": True,
}

ROOT = PurePosixPath("Trajectory Generator/baseline_MLP/validation/v12r10")
PROTOCOL_FREEZE_PATH = ROOT / "h0_v12r10_recovery_protocol_freeze.json"
EXECUTION_LOCK_PATH = ROOT / "h0_v12r10_recovery_execution_lock.json"
RUN_ROOT = ROOT / "h0_v12r10_run_20260815"
CLAIM_PATH = RUN_ROOT / "pipeline_claim.json"
LEDGER_PATH = RUN_ROOT / "pipeline_ledger.json"
R9_IMPORT_ATTESTATION_PATH = RUN_ROOT / "r9_terminal_import_attestation.json"
FIT_ROOT = RUN_ROOT / "fit"
CANDIDATE_MODULE_PATH = FIT_ROOT / "rl_module_recovery"
FIT_SUMMARY_PATH = FIT_ROOT / "summary.json"
FIT_GATE_PATH = FIT_ROOT / "gate.json"
FIT_RECEIPT_PATH = FIT_ROOT / "receipt.json"
CANDIDATE_FREEZE_PATH = RUN_ROOT / "candidate_freeze_receipt.json"
DEVELOPMENT_ROOT = RUN_ROOT / "development"
FINAL_DEVELOPMENT_PATH = RUN_ROOT / "final_development_receipt.json"

# V12R9 is immutable terminal evidence.  No V12R10 output is below R9_ROOT.
R9_ROOT = v12r9.ROOT
R9_PROTOCOL_FREEZE_PATH = v12r9.PROTOCOL_FREEZE_PATH
R9_EXECUTION_LOCK_PATH = v12r9.EXECUTION_LOCK_PATH
R9_RUN_ROOT = v12r9.RUN_ROOT
R9_PIPELINE_CLAIM_PATH = v12r9.CLAIM_PATH
R9_TERMINAL_LEDGER_PATH = v12r9.LEDGER_PATH
R9_CORPUS_PATH = v12r9.CORPUS_PATH
R9_CORPUS_MANIFEST_PATH = v12r9.FIT_ROOT / "corpus_manifest.json"
R9_FIT_SUMMARY_PATH = v12r9.FIT_SUMMARY_PATH
R9_FIT_GATE_PATH = v12r9.FIT_GATE_PATH
R9_CANDIDATE_MODULE_PATH = v12r9.CANDIDATE_MODULE_PATH
R9_TERMINAL_FAIL_STATUS = "FAIL_H0_V12R9_RECOVERY_PIPELINE_TERMINAL"
R9_TERMINAL_STAGE = "fit_recovery_actor"

R9_TERMINAL_EVIDENCE = {
    "protocol_freeze": {
        "path": R9_PROTOCOL_FREEZE_PATH.as_posix(),
        "sha256": "b674ad45f88555fc1fa28766a90ba06d6494b07f12454508a24eaf7773e88e02",
        "size_bytes": 55_865,
    },
    "execution_lock": {
        "path": R9_EXECUTION_LOCK_PATH.as_posix(),
        "sha256": "4713a0a37c399dad4e2f324d7d4fdacc5e5bfe60707fc22b5143543111a24ec6",
        "size_bytes": 49_592,
    },
    "pipeline_claim": {
        "path": R9_PIPELINE_CLAIM_PATH.as_posix(),
        "sha256": "5335b53670dcf823199f1c4580a11a2e0ac07742eded7b72281835267c2b527a",
        "size_bytes": 2_737,
    },
    "terminal_ledger": {
        "path": R9_TERMINAL_LEDGER_PATH.as_posix(),
        "sha256": "591fdc6ebe6e2a553b24ff210232a0f16185c4bc7b5e33104e34b780a0a18785",
        "size_bytes": 8_787,
    },
    "corpus": {
        "path": R9_CORPUS_PATH.as_posix(),
        "sha256": "1b35d0789d11a0f3bca3cae15c5877ceaf68845bf69ed7231d5a4ecc4d5b9dfe",
        "size_bytes": 32_561_707,
        "rows": 11_875,
    },
    "corpus_manifest": {
        "path": R9_CORPUS_MANIFEST_PATH.as_posix(),
        "sha256": "aab06d0d5d51b98af009cc9aed30f28f41c9c4699d7491e4b357c564f3b773f4",
        "size_bytes": 60_639,
    },
    "fit_summary": {
        "path": R9_FIT_SUMMARY_PATH.as_posix(),
        "sha256": "88d8318e98e359061d6bb6cbc9cb0f15fffe4af3237474bcbc61a9f2b3e7f44b",
        "size_bytes": 7_424,
    },
    "fit_gate": {
        "path": R9_FIT_GATE_PATH.as_posix(),
        "sha256": "fe69de6cc8f2ae83bb5838104dd658353a897c95c7ed326a2ec5b605e3af9f2b",
        "size_bytes": 398,
    },
}

R9_TERMINAL_CANDIDATE_TREE = {
    "path": R9_CANDIDATE_MODULE_PATH.as_posix(),
    "tree_sha256": "8bc8554c573f8224ea3fa9d8682d315f9ff30c4aaa1557c4974e4fa422b5d1ff",
    "file_count": 5,
    "files": [
        {
            "path": "actor_feature_manifest.json",
            "sha256": "14479f424b46300fa45f5b453f6ec95d182717327e7c5e456bf6492c2f672724",
            "size_bytes": 1_627,
        },
        {
            "path": "candidate_build_manifest.json",
            "sha256": "40bf0bbb017eb05aaab4c7a594031a2663ee281182343922de1cb227ad126dcd",
            "size_bytes": 2_336,
        },
        {
            "path": "class_and_ctor_args.pkl",
            "sha256": "a5cc979046fbb4dde936d4d3f2d913e16448e8cea5a8f03762f60145dbe674b0",
            "size_bytes": 2_262,
        },
        {
            "path": "metadata.json",
            "sha256": "3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12",
            "size_bytes": 197,
        },
        {
            "path": "module_state.pkl",
            "sha256": "6d0d319b8c1de66adc45116319fba38abfe15890c207545bec05653760d0d0b3",
            "size_bytes": 2_257_526,
        },
    ],
    "role": "INITIALIZATION_ONLY_NOT_PROMOTED",
}

R6_CANDIDATE_MODULE_PATH = v12r9.R6_CANDIDATE_MODULE_PATH
FULL_R6_CANDIDATE_TREE = copy.deepcopy(v12r9.FULL_R6_CANDIDATE_TREE)
SOURCE_H0_MODULE_PATH = v12r9.SOURCE_H0_MODULE_PATH
SOURCE_H0_ID = v12r9.SOURCE_H0_ID
LOCKED_SOURCE_H0_TREE = copy.deepcopy(v12r9.LOCKED_SOURCE_H0_TREE)
TEACHER_OBSERVATION_CONTRACT_ID = v12r9.TEACHER_OBSERVATION_CONTRACT_ID
COVERAGE_REFERENCE_CORPUS_PATH = v12r9.COVERAGE_REFERENCE_CORPUS_PATH
LOCKED_COVERAGE_REFERENCE = copy.deepcopy(v12r9.LOCKED_COVERAGE_REFERENCE)
BASE_CORPUS_PATH = v12r9.BASE_CORPUS_PATH
R4_PLUS_LABELS_PATH = v12r9.R4_PLUS_LABELS_PATH

EXPECTED_ACTOR_FEATURES = v12r9.EXPECTED_ACTOR_FEATURES
EXPECTED_FULL_FEATURES = v12r9.EXPECTED_FULL_FEATURES
EXPECTED_ACTION_DIM = v12r9.EXPECTED_ACTION_DIM
EXPECTED_STEPS = v12r9.EXPECTED_STEPS
RAW_SAMPLES_PER_STEP = v12r9.RAW_SAMPLES_PER_STEP
EXPECTED_SIGMA = v12r9.EXPECTED_SIGMA
DISABLED_CLOCK_COLUMNS = tuple(v12r9.DISABLED_CLOCK_COLUMNS)
PENETRATION_LIMIT_M = v12r9.PENETRATION_LIMIT_M
MINIMUM_VALID_CYCLES = v12r9.MINIMUM_VALID_CYCLES
TARGET_CONTRACT_ID = v12r9.TARGET_CONTRACT_ID
EVENT_CONTRACT_ID = v12r9.EVENT_CONTRACT_ID
OFFLINE_THRESHOLDS = copy.deepcopy(v12r9.OFFLINE_THRESHOLDS)

COLLECTION_CASE_IDS = tuple(v12r9.COLLECTION_CASE_IDS)
DEVELOPMENT_CASE_IDS = tuple(v12r9.DEVELOPMENT_CASE_IDS)
IMPORTED_LABEL_RECORDS = {
    "deterministic_offset_plus_0p20": {
        "path": v12r9.observer_label_path("deterministic_offset_plus_0p20").as_posix(),
        "sha256": "c00a49b2515731eacc397872bc033f1ae00bf680bcae93d8a7433e4ab725e5ef",
        "size_bytes": 254_326,
        "rows": 179,
    },
    "deterministic_offset_minus_0p20": {
        "path": v12r9.observer_label_path("deterministic_offset_minus_0p20").as_posix(),
        "sha256": "a83e9e11dd9caefdf288d5509d6f52da09ecccfb89a80e3addc47561b4a18799",
        "size_bytes": 352_876,
        "rows": 252,
    },
    "deterministic_offset_nominal": {
        "path": v12r9.observer_label_path("deterministic_offset_nominal").as_posix(),
        "sha256": "11cb1c408456f9edf79916c9eda0cba953d1c5015f684bc232f52f8ad524810f",
        "size_bytes": 687_676,
        "rows": 500,
    },
    "stochastic_nominal_seed_126": {
        "path": v12r9.observer_label_path("stochastic_nominal_seed_126").as_posix(),
        "sha256": "8cfd3cc25681fa945ec1c102e72564345cc6ec1afcb89b17061bfa1fb08ff718",
        "size_bytes": 687_676,
        "rows": 500,
    },
    "stochastic_nominal_seed_127": {
        "path": v12r9.observer_label_path("stochastic_nominal_seed_127").as_posix(),
        "sha256": "f7593c7d39786ad891780e9c34edc4b7ce95fca33896f818cd79ac352387f589",
        "size_bytes": 687_676,
        "rows": 500,
    },
    "stochastic_nominal_seed_128": {
        "path": v12r9.observer_label_path("stochastic_nominal_seed_128").as_posix(),
        "sha256": "42d78f60dfdaa5eb6a0aaf448c606b02b8c675548cfb62452e85e6d8d7465f9e",
        "size_bytes": 687_676,
        "rows": 500,
    },
}

DIAGNOSTIC_ROOT = ROOT / "diagnostics"
LOCKED_DIAGNOSTIC_EVIDENCE = {
    "uniform_source": {
        "path": (DIAGNOSTIC_ROOT / "dry_fit_w1024_r6_residual.py").as_posix(),
        "sha256": "945a2181fc8427209a66eb44a5d5846b557b0383af039432ec72f4406b347c9f",
        "size_bytes": 39_828,
    },
    "uniform_result": {
        "path": (
            DIAGNOSTIC_ROOT / "results" / "w1024_r6_residual_reset3_dry_fit.json"
        ).as_posix(),
        "sha256": "e4ceb12a6c35276fe8e1cc30a08bb4e3e7a6dcce7ceb3a570bfff6d5afdc8981",
        "size_bytes": 37_266,
    },
    "gate_aligned_source": {
        "path": (DIAGNOSTIC_ROOT / "dry_fit_w1024_gate_aligned.py").as_posix(),
        "sha256": "587ffd8e4f8c2c5bf5b63c7880e51b2dc34512a2cf7047377f31c6428b7912dd",
        "size_bytes": 33_506,
    },
    "gate_aligned_result": {
        "path": (
            DIAGNOSTIC_ROOT / "results" / "w1024_gate_aligned_dry_fit.json"
        ).as_posix(),
        "sha256": "7c69175f7e60d05c2dba7dca7212cbc33ad2c0b8ac39618f9b59d685f1486fdc",
        "size_bytes": 59_839,
    },
    "observability_source": {
        "path": (DIAGNOSTIC_ROOT / "analyze_transition_observability.py").as_posix(),
        "sha256": "8602716b6aff073f49bcaac077596bdeace51b0e1fa063bf11b79722bc959ced",
        "size_bytes": 47_550,
    },
    "observability_result": {
        "path": (
            DIAGNOSTIC_ROOT / "results" / "transition_observability.json"
        ).as_posix(),
        "sha256": "bf1f1c59b9bd730208210b69b5c1d8a9cd0b37b3c8293684e60b4a582acce688",
        "size_bytes": 223_940,
    },
}

EXPECTED_UNIFORM_STATE_DIGEST = (
    "52aee29da6db7535e8fcfe14f66fa1c7eaa0b95a0ac74fbf5c73b25a1c8fe167"
)
EXPECTED_UNIFORM_PREDICTION_DIGEST = (
    "2aa5c64704163b949525a28a4fd9d3e6688e6f2973fb4fa137fa7ff01f4da48d"
)
EXPECTED_FINAL_STATE_DIGEST = (
    "a0ad3cd981162879069c1317f7973b74b4bbcafa9e244c3b6180da34b17b24c0"
)
EXPECTED_FINAL_PREDICTION_DIGEST = (
    "684ec03d2a9e5c3bc37ad98cad943754d394500d198c0efc6b0ff4eb10256cf0"
)
EXPECTED_RUNTIME_LOGITS_DIGEST = (
    "73a7c63d7942d3f28748d5251512a93a59f7332bb3434998a81a1d5a164021c4"
)
EXPECTED_METRICS_DIGEST = (
    "0600e4c969d0ea0733f4c88ae093de0b707431abe48c89fbdd453f9b9c3cdb15"
)
EXPECTED_HISTORY_DIGEST = (
    "5fecb4f13e81a4235b6a7573d9af6533c3f532a2b8e77f3e00a6dd238d60a09e"
)
EXPECTED_REPLICA_DIGEST = (
    "51925deaba7be0ce256e93226895904117807d498747513e9fafdf1ae2151bae"
)
EXPECTED_GATE_DIGEST = (
    "7a1b93ca63727103fdc31c8ed0a931145a43705a63d432ab66b3bcc945cca762"
)
EXPECTED_COMPATIBILITY_DIGEST = (
    "4000fd2f7c445dd63f3c27ced07819c7d61ce5d523bd812c9ff99bade963663b"
)
EXPECTED_UNIFORM_LBFGS_CLOSURES = 3_072
EXPECTED_GATE_LBFGS_CLOSURES = 3_020
EXPECTED_UNIFORM_TERMINAL_LOSS = 3.0605827798686175e-05
EXPECTED_GATE_TERMINAL_LOSS = 0.7536344049605196

DIAGNOSTIC_ATTESTATION = {
    "status": "COMPLETE_H0_V12R10_W1024_GATE_ALIGNED_DRY_FIT",
    "decision": "ACCEPT_DIAGNOSTIC_STRATEGY_FOR_CANONICAL_DESIGN",
    "strategy_id": "V12R10_STANDARD_W1024_UNIFORM_TERMINAL_GATE_ALIGNED_V1",
    "uniform_state_digest": EXPECTED_UNIFORM_STATE_DIGEST,
    "uniform_prediction_digest": EXPECTED_UNIFORM_PREDICTION_DIGEST,
    "final_state_digest": EXPECTED_FINAL_STATE_DIGEST,
    "final_prediction_digest": EXPECTED_FINAL_PREDICTION_DIGEST,
    "runtime_logits_digest": EXPECTED_RUNTIME_LOGITS_DIGEST,
    "metrics_digest": EXPECTED_METRICS_DIGEST,
    "history_digest": EXPECTED_HISTORY_DIGEST,
    "replica_digest": EXPECTED_REPLICA_DIGEST,
    "gate_digest": EXPECTED_GATE_DIGEST,
    "compatibility_digest": EXPECTED_COMPATIBILITY_DIGEST,
    "uniform_lbfgs_closure_calls": EXPECTED_UNIFORM_LBFGS_CLOSURES,
    "gate_lbfgs_closure_calls": EXPECTED_GATE_LBFGS_CLOSURES,
    "uniform_terminal_loss": EXPECTED_UNIFORM_TERMINAL_LOSS,
    "gate_terminal_loss": EXPECTED_GATE_TERMINAL_LOSS,
}

FIT = {
    "fit_contract_id": FIT_CONTRACT_ID,
    "topology_id": TOPOLOGY_ID,
    "initial_actor": (
        "EXACT_R6_W512_FUNCTION_PLUS_R9_W512_HIDDEN_INITIALIZATION_ONLY_ZERO_HEAD"
    ),
    "architecture": {
        "kind": "standard_mean_mlp",
        "input_dim": 35,
        "hidden_dims": [1024, 1024],
        "output_dim": 2,
        "activation": "tanh",
        "residual_actor": False,
    },
    "corpus_source": "IMMUTABLE_R9_TERMINAL_CORPUS_IMPORT",
    "corpus_rows": 11_875,
    "observer_cases": list(COLLECTION_CASE_IDS),
    "stratum_policy": "R9_THIRTEEN_EQUAL_MASS_STRATA_RESET_ROWS_X3",
    "stratum_target_mass": 500.0,
    "stratum_count": 13,
    "reset_multiplier": 3.0,
    "reset_row_count": 26,
    "weights_sha256": (
        "6f5b6ce4f874e4547d3cdca4a9747be305691aab04fb13849aa050b26beeec00"
    ),
    "gate_group_count": 15,
    "objective": (
        "RESET3_MSE_PLUS_SYMMETRIC_SMOOTH_WORST_GATE_RMSE_GLOBAL_TAIL_RESET_TAIL"
    ),
    "objective_limits": {
        "rmse": 0.006,
        "max_abs_error": 0.060,
        "reset_max_abs_error": 0.003,
        "smooth_max_temperature": 0.05,
        "safety_margin_fraction": 0.90,
    },
    "optimizer": {
        "name": "TWO_PHASE_ADAMW_THEN_LBFGS_TERMINAL_ONLY",
        "seed": 20260814,
        "torch_threads": 5,
        "deterministic_algorithms": True,
        "uniform_phase": {
            "adamw_epochs": 2_500,
            "adamw_boundaries": [1_000, 2_000, 2_500],
            "adamw_learning_rates": [3.0e-4, 1.0e-4, 3.0e-5],
            "weight_decay": 1.0e-7,
            "gradient_clip_norm": 10.0,
            "lbfgs_lr": 0.7,
            "lbfgs_max_iter": 3_000,
            "lbfgs_max_eval": 4_500,
            "lbfgs_history_size": 50,
            "expected_lbfgs_closure_calls": EXPECTED_UNIFORM_LBFGS_CLOSURES,
            "expected_terminal_loss": EXPECTED_UNIFORM_TERMINAL_LOSS,
        },
        "gate_aligned_phase": {
            "adamw_epochs": 1_500,
            "adamw_boundaries": [500, 1_000, 1_500],
            "adamw_learning_rates": [3.0e-5, 1.0e-5, 3.0e-6],
            "weight_decay": 1.0e-7,
            "gradient_clip_norm": 10.0,
            "lbfgs_lr": 0.5,
            "lbfgs_max_iter": 3_000,
            "lbfgs_max_eval": 4_500,
            "lbfgs_history_size": 50,
            "expected_lbfgs_closure_calls": EXPECTED_GATE_LBFGS_CLOSURES,
            "expected_terminal_loss": EXPECTED_GATE_TERMINAL_LOSS,
        },
    },
    "expected_digests": copy.deepcopy(DIAGNOSTIC_ATTESTATION),
    "logstd_frozen": True,
    "clock_columns_bit_zero": True,
    "r6_tower_frozen": True,
    "cross_blocks_positive_zero": True,
    "critic_present": False,
    "actor_fit_count": 1,
    "critic_update_count": 0,
    "ppo_update_count": 0,
    "offline_h0_teacher_query_count": 0,
    "retry": False,
    "resume": False,
    "sweep": False,
    "early_stopping": False,
    "best_state_selection": False,
}

DEVELOPMENT_CASES = tuple(
    {
        **v12r9.canonical_development_case(case_id),
        "destination": (DEVELOPMENT_ROOT / case_id).as_posix(),
        "behavior": "R10_W1024_PURE_UNBLENDED_NO_TEACHER_NO_LATCH",
    }
    for case_id in DEVELOPMENT_CASE_IDS
)

STAGE_IDS = (
    "attest_r9_terminal_imports",
    "fit_recovery_actor",
    "freeze_recovery_actor",
    *(f"development__{case_id}" for case_id in DEVELOPMENT_CASE_IDS),
    "finalize_development",
)

LIMITATIONS = {
    "legacy_shadow_label_transition_alias": True,
    "offline_fit_proves_training_readiness": False,
    "runtime_legacy_shadow_dependency_authorized": False,
    "required_resolution": (
        "PURE_POLICY_DEVELOPMENT_THEN_Q3_MUST_FAIL_CLOSED_ON_THE_FROZEN_V26_ROUTE"
    ),
    "interpretation": (
        "The locked observability audit found no exact conflicting labels but "
        "identified hidden LegacyGaitShadow timeout history near the worst rows."
    ),
}

LOCKED_INPUTS = {
    "r9_terminal_evidence": copy.deepcopy(R9_TERMINAL_EVIDENCE),
    "r9_corpus": copy.deepcopy(R9_TERMINAL_EVIDENCE["corpus"]),
    "r9_terminal_candidate": copy.deepcopy(R9_TERMINAL_CANDIDATE_TREE),
    "r6_candidate": copy.deepcopy(FULL_R6_CANDIDATE_TREE),
    "source_h0": copy.deepcopy(LOCKED_SOURCE_H0_TREE),
    "coverage_reference_corpus": copy.deepcopy(LOCKED_COVERAGE_REFERENCE),
    "observer_labels": copy.deepcopy(IMPORTED_LABEL_RECORDS),
    "diagnostic_evidence": copy.deepcopy(LOCKED_DIAGNOSTIC_EVIDENCE),
}


def canonical_development_case(case_id: str) -> dict[str, Any]:
    for case in DEVELOPMENT_CASES:
        if case["case_id"] == case_id:
            return copy.deepcopy(case)
    raise ValueError(f"unknown V12R10 development case: {case_id!r}")


def candidate_id(tree_sha256: str) -> str:
    if (
        not isinstance(tree_sha256, str)
        or len(tree_sha256) != 64
        or any(char not in "0123456789abcdef" for char in tree_sha256)
    ):
        raise ValueError("candidate tree digest must be lowercase SHA-256")
    return f"AB06_H0_V12R10_RECOVERY_W1024:{tree_sha256}"


def _finite(value: Any) -> bool:
    return type(value) in (int, float) and math.isfinite(float(value))


def _zero_int(value: Any) -> bool:
    return type(value) is int and value == 0


def _metric_pass(record: Any, threshold: Mapping[str, float]) -> bool:
    return isinstance(record, Mapping) and all(
        _finite(record.get(name)) and float(record[name]) <= limit
        for name, limit in threshold.items()
    )


def fit_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    """Apply unchanged R9 numerical gates plus the explicit W1024 identity."""

    per_case = summary.get("per_case_metrics")
    observer = summary.get("observer_case_metrics")
    checks = {
        "single_fit": type(summary.get("actor_fit_count")) is int
        and summary.get("actor_fit_count") == 1
        and type(summary.get("actor_updates")) is int
        and summary.get("actor_updates") == 1,
        "no_critic_or_ppo": _zero_int(summary.get("critic_updates"))
        and _zero_int(summary.get("ppo_updates")),
        "architecture_w1024_standard": summary.get("hidden_dims") == [1024, 1024]
        and summary.get("actor_feature_count") == EXPECTED_ACTOR_FEATURES,
        "logstd_exact": summary.get("logstd_byte_exact") is True,
        "clock_zero": summary.get("disabled_clock_columns_bit_zero") is True,
        "save_reload_exact": summary.get("save_reload_exact") is True
        and summary.get("runtime_save_reload_exact") is True,
        "warm_start_compatible": summary.get("warm_start_actor_exact") is True
        and summary.get("warm_start_critic_preserved") is True,
        "tower_isolation": summary.get("tower_a_r6_byte_exact") is True
        and summary.get("cross_blocks_positive_zero") is True,
        "no_legacy_runtime": summary.get("no_legacy_shadow_runtime_dependency") is True,
        "state_digest": summary.get("candidate_state_digest")
        == EXPECTED_FINAL_STATE_DIGEST,
        "prediction_digest": summary.get("candidate_predictions_sha256")
        == EXPECTED_FINAL_PREDICTION_DIGEST,
        "uniform_reproduction": summary.get("uniform_state_digest")
        == EXPECTED_UNIFORM_STATE_DIGEST
        and summary.get("uniform_predictions_sha256")
        == EXPECTED_UNIFORM_PREDICTION_DIGEST
        and summary.get("uniform_lbfgs_closure_calls")
        == EXPECTED_UNIFORM_LBFGS_CLOSURES
        and summary.get("uniform_terminal_loss") == EXPECTED_UNIFORM_TERMINAL_LOSS,
        "gate_terminal_exact": summary.get("gate_lbfgs_closure_calls")
        == EXPECTED_GATE_LBFGS_CLOSURES
        and summary.get("gate_terminal_loss") == EXPECTED_GATE_TERMINAL_LOSS,
        "global": _metric_pass(
            summary.get("global_metrics"), OFFLINE_THRESHOLDS["global"]
        ),
        "reset": _finite(summary.get("reset_max_abs_error"))
        and float(summary["reset_max_abs_error"])
        <= OFFLINE_THRESHOLDS["reset_max_abs_error"],
        "per_case": isinstance(per_case, Mapping)
        and set(per_case) == set(COLLECTION_CASE_IDS)
        and all(
            _metric_pass(per_case[case_id], OFFLINE_THRESHOLDS["per_case"])
            for case_id in COLLECTION_CASE_IDS
        ),
        "r4_recovery": _metric_pass(
            summary.get("r4_failed_plus_metrics"),
            OFFLINE_THRESHOLDS["r4_failed_plus"],
        ),
        "observer_recovery": isinstance(observer, Mapping)
        and set(observer) == set(COLLECTION_CASE_IDS)
        and all(
            _metric_pass(observer[case_id], OFFLINE_THRESHOLDS["observer_case"])
            for case_id in COLLECTION_CASE_IDS
        ),
        "observer_plus_late": _metric_pass(
            summary.get("observer_plus_late_metrics"),
            OFFLINE_THRESHOLDS["observer_case"],
        ),
    }
    passed = all(checks.values())
    return {
        "status": (
            "PASS_H0_V12R10_RECOVERY_FIT" if passed else "FAIL_H0_V12R10_RECOVERY_FIT"
        ),
        "passed": passed,
        "checks": checks,
    }


def development_gate(summary: Mapping[str, Any], *, case_id: str) -> dict[str, Any]:
    value = v12r9.development_gate(summary, case_id=case_id)
    result = copy.deepcopy(value)
    result["status"] = (
        "PASS_H0_V12R10_PURE_DEVELOPMENT"
        if result.get("passed") is True
        else "FAIL_H0_V12R10_PURE_DEVELOPMENT"
    )
    return result


def contract_self_check() -> dict[str, Any]:
    checks = {
        "schema_1300": SCHEMA_VERSION == 1300,
        "new_namespace": ROOT != v12r9.ROOT,
        "identity_separated": PROTOCOL_ID != v12r9.PROTOCOL_ID
        and PIPELINE_ID != v12r9.PIPELINE_ID,
        "identities_import_only": all(
            "IMPORT_ONLY" in identity
            for identity in (PROTOCOL_ID, PIPELINE_ID, TOPOLOGY_ID)
        ),
        "import_only": AUTHORITY["new_environment_collection_authorized"] is False
        and AUTHORITY["observer_only_teacher_queries_authorized"] is False
        and FIT["offline_h0_teacher_query_count"] == 0,
        "r9_terminal_no_retry_or_promotion": AUTHORITY["r9_terminal_retry_authorized"]
        is False
        and AUTHORITY["r9_terminal_resume_authorized"] is False
        and AUTHORITY["r9_candidate_promotion_authorized"] is False
        and R9_TERMINAL_CANDIDATE_TREE["role"] == "INITIALIZATION_ONLY_NOT_PROMOTED",
        "standard_w1024": FIT["architecture"]
        == {
            "kind": "standard_mean_mlp",
            "input_dim": 35,
            "hidden_dims": [1024, 1024],
            "output_dim": 2,
            "activation": "tanh",
            "residual_actor": False,
        },
        "thresholds_unchanged": OFFLINE_THRESHOLDS == v12r9.OFFLINE_THRESHOLDS,
        "ten_stages_hardest_first": len(STAGE_IDS) == len(set(STAGE_IDS)) == 10
        and STAGE_IDS[0] == "attest_r9_terminal_imports"
        and STAGE_IDS[3:9]
        == tuple(f"development__{case_id}" for case_id in DEVELOPMENT_CASE_IDS)
        and DEVELOPMENT_CASE_IDS[0] == "deterministic_offset_plus_0p20",
        "no_collection_or_label_stage": all(
            not stage.startswith(("collect", "label")) for stage in STAGE_IDS
        ),
        "six_closed_labels": set(IMPORTED_LABEL_RECORDS) == set(COLLECTION_CASE_IDS)
        and sum(record["rows"] for record in IMPORTED_LABEL_RECORDS.values()) == 2_431,
        "corpus_exact": R9_TERMINAL_EVIDENCE["corpus"]["rows"] == 11_875,
        "r6_r9_h0_locked": FULL_R6_CANDIDATE_TREE["file_count"] == 5
        and R9_TERMINAL_CANDIDATE_TREE["file_count"] == 5
        and LOCKED_SOURCE_H0_TREE["file_count"] == 3,
        "diagnostic_source_result_pairs_locked": set(LOCKED_DIAGNOSTIC_EVIDENCE)
        == {
            "uniform_source",
            "uniform_result",
            "gate_aligned_source",
            "gate_aligned_result",
            "observability_source",
            "observability_result",
        },
        "diagnostic_digests_complete": all(
            isinstance(DIAGNOSTIC_ATTESTATION[name], str)
            and len(DIAGNOSTIC_ATTESTATION[name]) == 64
            for name in (
                "uniform_state_digest",
                "uniform_prediction_digest",
                "final_state_digest",
                "final_prediction_digest",
                "runtime_logits_digest",
                "metrics_digest",
                "history_digest",
                "replica_digest",
                "gate_digest",
                "compatibility_digest",
            )
        ),
        "one_fit_no_sweep": FIT["actor_fit_count"] == 1
        and FIT["retry"] is False
        and FIT["resume"] is False
        and FIT["sweep"] is False,
        "v26_and_penetration_unchanged": EVENT_CONTRACT_ID == v12r9.EVENT_CONTRACT_ID
        and TARGET_CONTRACT_ID == v12r9.TARGET_CONTRACT_ID
        and PENETRATION_LIMIT_M == v12r9.PENETRATION_LIMIT_M,
        "semantic_risk_fail_closed": LIMITATIONS["legacy_shadow_label_transition_alias"]
        is True
        and LIMITATIONS["offline_fit_proves_training_readiness"] is False
        and LIMITATIONS["runtime_legacy_shadow_dependency_authorized"] is False,
    }
    passed = all(checks.values())
    return {
        "status": (
            "PASS_H0_V12R10_RECOVERY_CONTRACT"
            if passed
            else "FAIL_H0_V12R10_RECOVERY_CONTRACT"
        ),
        "passed": passed,
        "checks": checks,
    }


__all__ = [name for name in globals() if name.isupper()] + [
    "candidate_id",
    "canonical_development_case",
    "contract_self_check",
    "development_gate",
    "fit_gate",
]
