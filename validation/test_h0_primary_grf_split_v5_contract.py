from __future__ import annotations

from pathlib import PurePosixPath

import h0_primary_grf_split_v5_freeze_contract as contract


def test_v5_design_is_single_full_mean_fixed_final_candidate() -> None:
    assert contract.SCHEMA_VERSION == 5
    assert contract.TRAIN_SEEDS == (123, 124)
    assert contract.FINAL_HOLDOUT_SEED == 125
    assert contract.TRAINABLE_SCOPE == "full_mean_network"
    assert contract.LOGSTD_POLICY == "frozen_bit_exact"
    assert contract.FIT == {
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
    assert contract.AUTHORITY["retry_or_sweep"] is False
    assert contract.AUTHORITY["reuse_failed_v3_candidate"] is False
    assert contract.AUTHORITY["reuse_v4_candidate"] is False
    assert contract.AUTHORITY["v4_retry"] is False
    assert contract.AUTHORITY["seed125_semantic_access_before_candidate_freeze"] is False


def test_authorized_train_only_evidence_passes_closed_thresholds() -> None:
    evidence = contract.TRAIN_ONLY_PREFLIGHT_EVIDENCE
    thresholds = contract.OFFLINE_THRESHOLDS
    assert evidence["candidate_selection_count"] == 1
    assert evidence["holdout_seed_accessed"] is False
    assert evidence["student_rmse"] <= thresholds["student_rmse_max"]
    assert evidence["student_max_abs_error"] <= thresholds["student_max_abs_error_max"]
    assert evidence["teacher_rmse"] <= thresholds["teacher_rmse_max"]
    assert evidence["teacher_max_abs_error"] <= thresholds["teacher_max_abs_error_max"]


def test_all_contract_paths_are_canonical_repository_relative() -> None:
    values = [
        contract.RUN_ROOT_RELATIVE,
        contract.LOCK_RELATIVE,
        contract.PREFLIGHT_RECEIPT_RELATIVE,
        contract.V4_PREEXECUTION_FAILURE_RELATIVE,
        *contract.V5_SOURCE_RELATIVE_PATHS.values(),
        *contract.INHERITED_SOURCE_RELATIVE_PATHS.values(),
        *contract.H0_INPUT_RELATIVE_PATHS.values(),
        *contract.V3_INPUT_RELATIVE_PATHS.values(),
        *contract.HOLDOUT_BYTE_INPUT_RELATIVE_PATHS.values(),
    ]
    for value in values:
        pure = PurePosixPath(value)
        assert value
        assert not pure.is_absolute()
        assert ".." not in pure.parts
        assert pure.as_posix() == value


def test_public_artifact_schemas_have_closed_update_footers() -> None:
    for keys in (
        contract.ADAPTATION_RECEIPT_KEYS,
        contract.CANDIDATE_FREEZE_KEYS,
        contract.HOLDOUT_REPLAY_RECEIPT_KEYS,
        contract.HOLDOUT_RECEIPT_KEYS,
        contract.EXECUTION_LEDGER_KEYS,
    ):
        assert {"critic_updates", "ppo_updates", "protected_trials_opened"} <= keys
    assert "additional_actor_updates" in contract.HOLDOUT_REPLAY_RECEIPT_KEYS
    assert "additional_actor_updates" in contract.HOLDOUT_RECEIPT_KEYS
    for keys in (
        contract.ADAPTATION_RECEIPT_KEYS,
        contract.ACTOR_MANIFEST_KEYS,
        contract.CANDIDATE_FREEZE_KEYS,
        contract.HOLDOUT_ACCESS_CLAIM_KEYS,
        contract.HOLDOUT_REPLAY_RECEIPT_KEYS,
        contract.HOLDOUT_RECEIPT_KEYS,
        contract.EXECUTION_LEDGER_KEYS,
    ):
        assert "v4_preexecution_failure" in keys
    assert "v4_candidate_reused" in contract.EXECUTION_LEDGER_KEYS
    assert contract.V4_PREEXECUTION_EXPECTATION["v4_execution_started"] is False
    assert contract.V4_PREEXECUTION_EXPECTATION["actor_update_candidates"] == 0
