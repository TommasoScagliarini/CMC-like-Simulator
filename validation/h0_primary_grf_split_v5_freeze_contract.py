"""Pure no-I/O contract for H0 primary-split V5 full-mean adaptation.

V5 is a new one-shot lineage.  It inherits only the frozen, passing V3
train corpus (seeds 123/124), the terminal V3 failure receipt, and V4's
pre-execution integrity failure.  It never inherits a V3/V4 candidate and it
keeps seed 125 procedural: the freezer may hash the two historical files, but
only the post-candidate-freeze holdout worker may parse them.
"""

from __future__ import annotations


SCHEMA_VERSION = 5
REVISION = "2026-08-06"
PROTOCOL_ID = "AB06_H0_PRIMARY_GRF_SPLIT_V5_FULL_MEAN"
CANDIDATE_ID = "H0_primary_split_v5_full_mean"
TRAIN_SEEDS = (123, 124)
FINAL_HOLDOUT_SEED = 125
EXPECTED_STEPS = 500
EXPECTED_ACTOR_FEATURES = 35
EXPECTED_FULL_FEATURES = 84
EVENT_CONTRACT_ID = "primary_grf_split_v1+legacy_events_v1"
SO_POLICY_ID = "verified_status0_max_iter_v1"
TRAINABLE_SCOPE = "full_mean_network"
LOGSTD_POLICY = "frozen_bit_exact"

RUN_ROOT_RELATIVE = (
    "validation/h0_primary_grf_split_adaptation_runs/"
    "2026-08-06_h0_primary_split_v5_full_mean"
)
LOCK_RELATIVE = "validation/h0_primary_grf_split_v5_execution_lock.json"
PREFLIGHT_RECEIPT_RELATIVE = (
    "validation/h0_primary_grf_split_v5_preflight_receipt.json"
)
V4_PREEXECUTION_FAILURE_RELATIVE = (
    "validation/h0_primary_grf_split_v4_preexecution_failure.json"
)

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

OFFLINE_THRESHOLDS = {
    "student_rmse_max": 0.01,
    "student_max_abs_error_max": 0.10,
    "teacher_rmse_max": 0.005,
    "teacher_max_abs_error_max": 0.05,
    "student_rmse_reduction_min": 0.50,
}

# Train-only design evidence.  It selected the sole V5 design; it is not a
# promotion result and cannot substitute for the frozen seed-125 holdout.
TRAIN_ONLY_PREFLIGHT_EVIDENCE = {
    "student_rmse": 0.0030197,
    "student_max_abs_error": 0.02477,
    "teacher_rmse": 0.0030618,
    "teacher_max_abs_error": 0.02053,
    "source": "authorized_train_only_seed_123_124_preflight",
    "candidate_selection_count": 1,
    "holdout_seed_accessed": False,
}

V3_TERMINAL_EXPECTATION = {
    "status": "FAIL_H0_PRIMARY_SPLIT_V3_OFFLINE",
    "terminal_stage": "adaptation",
    "passed": False,
    "candidate_created": True,
    "candidate_frozen_before_holdout": False,
    "holdout_access_claimed": False,
    "final_holdout_completed": False,
    "next_stage": "STOP_WITHOUT_RETRY_OR_RETUNING",
}

V3_FAILED_METRICS = {
    "student_rmse": 0.0105039088860462,
    "teacher_rmse": 0.007882278226384947,
}

V4_PREEXECUTION_EXPECTATION = {
    "schema_version": 1,
    "status": "FAIL_H0_PRIMARY_SPLIT_V4_PREEXECUTION_INTEGRITY",
    "passed": False,
    "protocol_id": "AB06_H0_PRIMARY_GRF_SPLIT_V4_FULL_MEAN",
    "failure": "post_lock_runner_drift_and_stale_preflight_removal",
    "failure_stage": "verify_lock_before_attempt_claim",
    "v4_execution_started": False,
    "attempt_claim_created": False,
    "run_root_exists": False,
    "seed125_semantic_accessed": False,
    "actor_update_candidates": 0,
    "critic_updates": 0,
    "ppo_updates": 0,
    "protected_trials_opened": [],
    "v4_retry_allowed": False,
    "next_stage": "NEW_PROTOCOL_REQUIRED_NO_V4_RETRY",
    "observed_preflight_receipt_exists": False,
}

V5_SOURCE_RELATIVE_PATHS = {
    "plan_addendum": (
        "reports/plans/2026-08-06_addendum_h0_primary_split_v5_full_mean.md"
    ),
    "freeze_contract": "validation/h0_primary_grf_split_v5_freeze_contract.py",
    "execution_freezer": "validation/freeze_h0_primary_grf_split_v5_execution.py",
    "preflight_builder": (
        "validation/build_h0_primary_grf_split_v5_preflight_receipt.py"
    ),
    "runner": "validation/run_h0_primary_grf_split_v5_full_mean.py",
    "contract_tests": "validation/test_h0_primary_grf_split_v5_contract.py",
    "freezer_tests": (
        "validation/test_freeze_h0_primary_grf_split_v5_execution.py"
    ),
    "runner_tests": "validation/test_h0_primary_grf_split_v5_full_mean.py",
}

INHERITED_SOURCE_RELATIVE_PATHS = {
    "v4_hardened_runner_template": (
        "validation/run_h0_primary_grf_split_v4_full_mean.py"
    ),
    "v3_runner_replay_engine": (
        "validation/run_h0_primary_grf_split_v3_semantic_replay.py"
    ),
    "v3_freeze_contract": (
        "validation/h0_primary_grf_split_v3_freeze_contract.py"
    ),
    "primary_split_contract": (
        "Trajectory Generator/baseline_MLP/primary_grf_split_adaptation.py"
    ),
    "actor_fit": "Trajectory Generator/baseline_MLP/target_domain_imitation.py",
    "warm_start": "Trajectory Generator/baseline_MLP/warm_start.py",
}

H0_INPUT_RELATIVE_PATHS = {
    "config": (
        "validation/critic_warmup/"
        "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/"
        "training_cfg.resolved.yaml"
    ),
    "state": (
        "validation/critic_warmup/"
        "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/"
        "rl_module_last/module_state.pkl"
    ),
    "ctor": (
        "validation/critic_warmup/"
        "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/"
        "rl_module_last/class_and_ctor_args.pkl"
    ),
    "metadata": (
        "validation/critic_warmup/"
        "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/"
        "rl_module_last/metadata.json"
    ),
}

V3_INPUT_RELATIVE_PATHS = {
    "execution_lock": "validation/h0_primary_grf_split_v3_execution_lock.json",
    "execution_ledger": (
        "validation/h0_primary_grf_split_adaptation_runs/"
        "2026-08-06_h0_primary_split_v3_semantic_replay/execution_ledger.json"
    ),
    "corpus": (
        "validation/h0_primary_grf_split_adaptation_runs/"
        "2026-08-06_h0_primary_split_v3_semantic_replay/corpus/corpus.npz"
    ),
    "corpus_manifest": (
        "validation/h0_primary_grf_split_adaptation_runs/"
        "2026-08-06_h0_primary_split_v3_semantic_replay/corpus/manifest.json"
    ),
    "corpus_receipt": (
        "validation/h0_primary_grf_split_adaptation_runs/"
        "2026-08-06_h0_primary_split_v3_semantic_replay/corpus/receipt.json"
    ),
    "failed_candidate_receipt": (
        "validation/h0_primary_grf_split_adaptation_runs/"
        "2026-08-06_h0_primary_split_v3_semantic_replay/adaptation/receipt.json"
    ),
    "failed_offline_gate": (
        "validation/h0_primary_grf_split_adaptation_runs/"
        "2026-08-06_h0_primary_split_v3_semantic_replay/adaptation/offline_gate.json"
    ),
}

HOLDOUT_BYTE_INPUT_RELATIVE_PATHS = {
    "trace": (
        "validation/controller_memory_ablation/"
        "2026-07-13_markov35_corrected_full_sigma0005_seed125/"
        "rollout_policy_trace.json"
    ),
    "summary": (
        "validation/controller_memory_ablation/"
        "2026-07-13_markov35_corrected_full_sigma0005_seed125/"
        "rollout_summary.json"
    ),
}

DESTINATION_SUFFIXES = (
    "adaptation",
    "adaptation/candidate_freeze.json",
    "holdout_access_claim.json",
    "replay/seed_125",
    "holdout_replay_receipt.json",
    "holdout",
    "execution_ledger.json",
)

AUTHORITY = {
    "reuse_frozen_v3_train_corpus": True,
    "reuse_failed_v3_candidate": False,
    "reuse_v4_candidate": False,
    "v4_retry": False,
    "actor_only_adaptation": True,
    "full_mean_network_update": True,
    "logstd_update": False,
    "critic_updates": False,
    "ppo_updates": False,
    "seed125_semantic_access_before_candidate_freeze": False,
    "retry_or_sweep": False,
    "protected_trial_access": False,
    "primary_grf_modification": False,
    "sea_semantic_modification": False,
    "runtime_promotion": False,
}

# Public artifact schemas used by downstream zero-port/qualification tooling.
ADAPTATION_RECEIPT_KEYS = frozenset(
    {
        "schema_version",
        "status",
        "passed",
        "candidate_module_state",
        "candidate_module_ctor",
        "candidate_module_metadata",
        "actor_feature_manifest",
        "adaptation_report",
        "offline_gate",
        "execution_lock",
        "attempt_claim",
        "v3_corpus",
        "v3_corpus_receipt",
        "v3_corpus_manifest",
        "v3_terminal_ledger",
        "v4_preexecution_failure",
        "source_h0",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
        "protected_trials_opened",
    }
)

ACTOR_MANIFEST_KEYS = frozenset(
    {
        "schema_version",
        "candidate_id",
        "observation_contract_id",
        "event_contract_id",
        "actor_feature_count",
        "actor_feature_names",
        "trainable_scope",
        "logstd_policy",
        "actor_digest",
        "module_state_sha256",
        "execution_lock",
        "attempt_claim",
        "v3_corpus",
        "v3_terminal_ledger",
        "v4_preexecution_failure",
        "source_h0",
    }
)

CANDIDATE_FREEZE_KEYS = frozenset(
    {
        "schema_version",
        "status",
        "protocol_id",
        "candidate_id",
        "train_seeds",
        "final_holdout_seed",
        "fit",
        "trainable_scope",
        "logstd_policy",
        "candidate_receipt",
        "candidate_offline_gate",
        "candidate_module_state",
        "candidate_module_ctor",
        "candidate_module_metadata",
        "actor_feature_manifest",
        "candidate_actor_digest",
        "v3_terminal_ledger",
        "v4_preexecution_failure",
        "v3_corpus",
        "holdout_accessed_before_freeze",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
        "protected_trials_opened",
    }
)

HOLDOUT_ACCESS_CLAIM_KEYS = frozenset(
    {
        "schema_version",
        "status",
        "protocol_id",
        "candidate_id",
        "candidate_freeze",
        "holdout_seed",
        "fit_updates_complete_before_access",
        "additional_actor_updates_authorized",
        "replay_engine",
        "v4_preexecution_failure",
        "so_policy_id",
        "critic_updates",
        "ppo_updates",
        "protected_trials_opened",
    }
)

HOLDOUT_REPLAY_RECEIPT_KEYS = frozenset(
    {
        "schema_version",
        "status",
        "passed",
        "seed",
        "engine_schema",
        "engine_runner",
        "v5_wrapper_runner",
        "engine_receipt",
        "engine_gate",
        "engine_arrays",
        "candidate_freeze",
        "holdout_access_claim",
        "v4_preexecution_failure",
        "so_policy_id",
        "actor_updates",
        "additional_actor_updates",
        "critic_updates",
        "ppo_updates",
        "protected_trials_opened",
    }
)

HOLDOUT_RECEIPT_KEYS = frozenset(
    {
        "schema_version",
        "status",
        "passed",
        "gate",
        "candidate_freeze",
        "holdout_access_claim",
        "holdout_replay_receipt",
        "v4_preexecution_failure",
        "actor_updates",
        "additional_actor_updates",
        "critic_updates",
        "ppo_updates",
        "protected_trials_opened",
    }
)

EXECUTION_LEDGER_KEYS = frozenset(
    {
        "schema_version",
        "status",
        "passed",
        "terminal_stage",
        "error",
        "started_unix_s",
        "completed_unix_s",
        "execution_lock",
        "attempt_claim",
        "v3_terminal_ledger",
        "v4_preexecution_failure",
        "v3_corpus_reused",
        "v3_failed_candidate_reused",
        "v4_candidate_reused",
        "candidate_created",
        "candidate_frozen_before_holdout",
        "holdout_access_claimed",
        "holdout_replay_completed",
        "final_holdout_completed",
        "actor_update_candidates",
        "critic_updates",
        "ppo_updates",
        "protected_trials_opened",
        "retry_or_retuning_allowed",
        "next_stage",
    }
)
