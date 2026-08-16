"""Pure, candidate-deferred contract for the post-Q2 V12R4 zero port.

This module derives the target topology from the historical V5 standard
full-mean port, never from the V6 residual lineage.  Importing it performs no
filesystem access and grants no authority to freeze, build an Algorithm,
sample an environment, save a checkpoint, or publish a warm-start command.

The candidate remains deliberately unbound in source.  A future freezer may
bind exactly one actor-only V12R4 P3 tree only after both the R4 development
ledger and the independent Q2 qualification ledger are terminal PASS records
for that same tree.
"""

from __future__ import annotations

from pathlib import PurePosixPath
from typing import Any

try:
    from ..v12r4q2 import h0_v12r4_q2_qualification_contract as q2
except ImportError:  # Direct execution/import with validation on sys.path.
    from v12r4q2 import h0_v12r4_q2_qualification_contract as q2


SCHEMA_VERSION = 141
REVISION = "2026-08-10"
PROTOCOL_ID = "AB06_H0_V12R4_Q2_V26_ZERO_UPDATE_PORT"
SOURCE_TOPOLOGY_ID = "V5_STANDARD_FULL_MEAN_ZERO_PORT"
QUALIFICATION_PROTOCOL_ID = q2.PROTOCOL_ID
PIPELINE_ID = "H0_V12R4_Q2_V26_ZERO_UPDATE_CHECKPOINT"

VALIDATION_ROOT = PurePosixPath(
    "Trajectory Generator/baseline_MLP/validation/v12r4zero"
)
LOCK_PATH = VALIDATION_ROOT / "h0_v12r4_zero_update_execution_lock.json"
OUTPUT_ROOT = VALIDATION_ROOT / "h0_v12r4_zero_update_run_20260810"

LOCK_STATUS = "PASS_H0_V12R4_ZERO_UPDATE_EXECUTION_LOCK"
PASS_STATUS = "PASS_H0_V12R4_Q2_V26_ZERO_UPDATE_PORT"
FAIL_STATUS = "FAIL_H0_V12R4_Q2_V26_ZERO_UPDATE_PORT"
ATTEMPT_STATUS = "H0_V12R4_ZERO_UPDATE_ATTEMPT_CLAIMED"

CANDIDATE_BINDING_STATE = "DEFERRED_UNTIL_Q2_TERMINAL_PASS"
CANDIDATE_ID = None
CANDIDATE_MODULE = None
CANDIDATE_SELECTION_RULE = q2.R4_CANDIDATE_SELECTION_RULE
CANDIDATE_MODULE_PATH = q2.R4_CANDIDATE_MODULE_PATH
CANDIDATE_ID_PREFIX = "h0_v12r4_p3::"

# These paths intentionally come from the Q2 contract.  They are aliases, not
# duplicated literals, so later Q2 namespace drift fails contract tests.
Q2_PROTOCOL_FREEZE_PATH = q2.PROTOCOL_FREEZE_PATH
Q2_FINAL_RECEIPT_PATH = q2.FINAL_RECEIPT_PATH
Q2_PIPELINE_LEDGER_PATH = q2.PIPELINE_LEDGER_PATH

R4_CANDIDATE_FREEZE_RECEIPT_PATH = q2.R4_CANDIDATE_FREEZE_RECEIPT_PATH
R4_FINAL_DEVELOPMENT_RECEIPT_PATH = q2.R4_FINAL_DEVELOPMENT_RECEIPT_PATH
R4_PIPELINE_LEDGER_PATH = q2.R4_PIPELINE_LEDGER_PATH

SOURCE_TRAINING_CONFIG_PATH = PurePosixPath(
    "validation/critic_warmup/"
    "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/"
    "training_cfg.resolved.yaml"
)
V26_TARGET_CONFIG_PATH = PurePosixPath(
    "Trajectory Generator/baseline_MLP/experimental_configs/"
    "morphology_event_anchored_causal_v26_candidate.yaml"
)
V26_DETECTOR_PROFILE_PATH = PurePosixPath(
    "validation/binary_phase_detector_v25_geometry_runs/"
    "2026-08-04_local_reach_sweep_dev02_04_08/"
    "selected_candidate_profile.json"
)
MORPHOLOGY_PROFILE_PATH = PurePosixPath(
    "Trajectory Generator/baseline_MLP/morphology_profiles/"
    "ab06_prosthetic_event_warped_mean_std_corridor.json"
)

INPUT_RELATIVE_PATHS = {
    "q2_protocol_freeze": Q2_PROTOCOL_FREEZE_PATH.as_posix(),
    "q2_final_receipt": Q2_FINAL_RECEIPT_PATH.as_posix(),
    "q2_pipeline_ledger": Q2_PIPELINE_LEDGER_PATH.as_posix(),
    "r4_candidate_freeze_receipt": (R4_CANDIDATE_FREEZE_RECEIPT_PATH.as_posix()),
    "r4_final_development_receipt": (R4_FINAL_DEVELOPMENT_RECEIPT_PATH.as_posix()),
    "r4_pipeline_ledger": R4_PIPELINE_LEDGER_PATH.as_posix(),
    "source_training_config": SOURCE_TRAINING_CONFIG_PATH.as_posix(),
    "v26_target_config": V26_TARGET_CONFIG_PATH.as_posix(),
    "v26_detector_profile": V26_DETECTOR_PROFILE_PATH.as_posix(),
    "morphology_profile": MORPHOLOGY_PROFILE_PATH.as_posix(),
}

SOURCE_RELATIVE_PATHS = {
    "contract": (VALIDATION_ROOT / "h0_v12r4_zero_update_contract.py").as_posix(),
    "freezer": (VALIDATION_ROOT / "freeze_h0_v12r4_zero_update.py").as_posix(),
    "runner": (VALIDATION_ROOT / "run_h0_v12r4_zero_update_port.py").as_posix(),
    "tests": (VALIDATION_ROOT / "test_h0_v12r4_zero_update_port.py").as_posix(),
    "package_init": (VALIDATION_ROOT / "__init__.py").as_posix(),
    "q2_contract": (
        q2.VALIDATION_ROOT / "h0_v12r4_q2_qualification_contract.py"
    ).as_posix(),
    "training_entrypoint": "Trajectory Generator/baseline_MLP/train_ppo_mlp.py",
    "training_config": "Trajectory Generator/baseline_MLP/training_config.py",
    "warm_start": "Trajectory Generator/baseline_MLP/warm_start.py",
    "asymmetric_module": ("Trajectory Generator/baseline_MLP/asymmetric_rl_module.py"),
    "reward_wrapper": "Trajectory Generator/baseline_MLP/reward_function.py",
    "morphology_corridor": (
        "Trajectory Generator/baseline_MLP/experimental_morphology_corridor.py"
    ),
    "environment_factory": "Trajectory Generator/baseline_MLP/env_factory.py",
    "environment": "Trajectory Generator/osim_trj_cmc_like.py",
}

STANDARD_RL_MODULE_KIND = "standard"
DEFAULT_POLICY_ID = "default_policy"
EXPECTED_ACTOR_FEATURES = q2.EXPECTED_ACTOR_FEATURES
EXPECTED_FULL_FEATURES = q2.EXPECTED_FULL_FEATURES
PRIMARY_LOAD_CONTRACT_ID = q2.PRIMARY_LOAD_CONTRACT_ID
BINARY_EVENT_CONTRACT_ID = q2.EVENT_CONTRACT_ID
TARGET_CONTRACT_ID = q2.TARGET_CONTRACT_ID
LEGACY_EVENT_CONTRACT_ID = "legacy_events_v1"
MORPHOLOGY_WEIGHT = 0.0

TARGET_FIXED_CONFIG = {
    "iterations": 0,
    "num_env_runners": 0,
    "ray_num_cpus": 1,
    "tensorboard": False,
    "progress": False,
    "update_history": False,
    "exact_start_sampling": False,
    "asymmetric_actor_critic": True,
    "rl_module_kind": STANDARD_RL_MODULE_KIND,
    "phase_fsm_input_mode": "legacy_events",
    "event_contract_id": LEGACY_EVENT_CONTRACT_ID,
    "binary_phase_fsm_mode": "binary_active",
    "binary_phase_detector_profile": V26_DETECTOR_PROFILE_PATH.as_posix(),
    "binary_phase_debounce_s": 0.005,
    "binary_phase_event_contract_id": BINARY_EVENT_CONTRACT_ID,
}

TARGET_REWARD_CONFIG = {
    "morphology_profile": (
        "morphology_profiles/" "ab06_prosthetic_event_warped_mean_std_corridor.json"
    ),
    "morphology_profile_sha256": (
        "33b1dd7cb0db40110a4f9c1b8c0dd49a498662211e6e132f0f3cefe8edc02a55"
    ),
    "morphology_phase_mode": "event_anchored_causal_delayed_experimental",
    "morphology_canonical_to_phase": 0.6223299989,
    "morphology_reward_delay_s": 0.04,
    "morphology_max_delivery_latency_s": 0.01,
    "morphology_causal_max_samples": 4096,
    "morphology_causal_event_contract_id": BINARY_EVENT_CONTRACT_ID,
    "morphology_causal_allow_effects": 0.0,
    "morphology_weight": MORPHOLOGY_WEIGHT,
    "morphology_hard_termination_enabled": 0.0,
}

AUTHORITY = {
    "source_scaffold_authorized": True,
    "candidate_binding_deferred": True,
    "future_lock_publication_requires_q2_terminal_pass": True,
    "freeze_publication_authorized_now": False,
    "algorithm_build_authorized_now": False,
    "checkpoint_publication_authorized_now": False,
    "fresh_target_algorithm_build_authorized_after_lock": True,
    "candidate_actor_transplant_authorized_after_lock": True,
    "full_checkpoint_save_reload_authorized_after_lock": True,
    "source_critic_restore_authorized": False,
    "source_optimizer_restore_authorized": False,
    "actor_updates_authorized": False,
    "critic_updates_authorized": False,
    "ppo_updates_authorized": False,
    "environment_sampling_authorized": False,
    "protected_trial_access_authorized": False,
    "runtime_promotion_authorized": False,
    "positive_morphology_authorized": False,
}

ZERO_COUNTER_NAMES = (
    "training_iteration",
    "num_env_steps_sampled_lifetime",
    "num_agent_steps_sampled_lifetime",
    "num_env_steps_trained_lifetime",
    "num_agent_steps_trained_lifetime",
    "num_grad_updates_lifetime",
)

REQUIRED_CHECKS = (
    "q2_protocol_receipt_ledger_terminal_pass",
    "r4_exact_candidate_bound_after_q2",
    "standard_full_mean_actor_only_source",
    "v26_target_config_exact",
    "morphology_weight_zero",
    "source_closure_rehashed_before_build",
    "fresh_target_algorithm",
    "fresh_critic_created_without_source_restore",
    "critic_unchanged_by_actor_transplant",
    "optimizer_state_empty_before_save",
    "optimizer_param_groups_exact_before_save",
    "optimizer_param_groups_unchanged_by_transplant",
    "zero_progress_before_save",
    "local_actor_exact_before_save",
    "learner_actor_exact_before_save",
    "env_runner_actor_exact_before_save",
    "export_actor_exact_before_save",
    "full_rllib_checkpoint_saved_at_zero",
    "source_closure_rehashed_before_restore",
    "restored_local_actor_exact",
    "restored_learner_actor_exact",
    "restored_env_runner_actor_exact",
    "restored_export_actor_exact",
    "restored_fresh_critic_exact",
    "restored_optimizer_state_empty",
    "restored_optimizer_param_groups_exact",
    "zero_progress_after_restore",
    "source_closure_rehashed_after_restore",
    "no_algorithm_train_call",
    "no_actor_update",
    "no_critic_update",
    "no_ppo_update",
    "no_environment_sample",
    "no_protected_access",
)

CHECKPOINT_REQUIRED_SUFFIXES = frozenset(
    {
        "algorithm_state.pkl",
        "class_and_ctor_args.pkl",
        "rllib_checkpoint.json",
        "learner_group/learner/state.pkl",
        "learner_group/learner/rl_module/module_state.pkl",
        "env_runner/state.pkl",
    }
)

OUTPUT_NAMES = {
    "attempt_claim": "attempt_claim.json",
    "resolved_config": "training_cfg.resolved.yaml",
    "checkpoint": "checkpoint_zero",
    "initial_export": "rl_module_zero",
    "restored_export": "rl_module_reloaded",
    "audit": "zero_update_audit.json",
    "receipt": "receipt.json",
    "ledger": "execution_ledger.json",
}


def candidate_id_for_tree(tree_sha256: str) -> str:
    """Return the sole valid R4 candidate id for a hexadecimal tree hash."""

    if not isinstance(tree_sha256, str) or len(tree_sha256) != 64:
        raise ValueError("candidate tree hash must be a SHA-256 digest")
    try:
        int(tree_sha256, 16)
    except ValueError as exc:
        raise ValueError("candidate tree hash must be hexadecimal") from exc
    return f"{CANDIDATE_ID_PREFIX}{tree_sha256}"


def contract_self_check() -> dict[str, Any]:
    """Check the source-only contract without reading the filesystem."""

    checks = {
        "candidate_deferred": CANDIDATE_ID is None
        and CANDIDATE_MODULE is None
        and CANDIDATE_BINDING_STATE == "DEFERRED_UNTIL_Q2_TERMINAL_PASS",
        "q2_paths_are_aliases": Q2_PROTOCOL_FREEZE_PATH == q2.PROTOCOL_FREEZE_PATH
        and Q2_FINAL_RECEIPT_PATH == q2.FINAL_RECEIPT_PATH
        and Q2_PIPELINE_LEDGER_PATH == q2.PIPELINE_LEDGER_PATH,
        "r4_candidate_path_exact": CANDIDATE_MODULE_PATH == q2.R4_CANDIDATE_MODULE_PATH,
        "standard_not_residual": STANDARD_RL_MODULE_KIND == "standard"
        and "residual" not in SOURCE_TOPOLOGY_ID.lower(),
        "v26_exact": TARGET_FIXED_CONFIG["binary_phase_fsm_mode"] == "binary_active"
        and TARGET_FIXED_CONFIG["binary_phase_event_contract_id"]
        == q2.EVENT_CONTRACT_ID
        and TARGET_CONTRACT_ID == q2.TARGET_CONTRACT_ID,
        "zero_effects": MORPHOLOGY_WEIGHT == 0.0
        and TARGET_REWARD_CONFIG["morphology_weight"] == 0.0
        and TARGET_REWARD_CONFIG["morphology_causal_allow_effects"] == 0.0,
        "zero_sampling_updates": TARGET_FIXED_CONFIG["iterations"] == 0
        and TARGET_FIXED_CONFIG["num_env_runners"] == 0
        and AUTHORITY["actor_updates_authorized"] is False
        and AUTHORITY["critic_updates_authorized"] is False
        and AUTHORITY["ppo_updates_authorized"] is False
        and AUTHORITY["environment_sampling_authorized"] is False,
        "publication_blocked_now": AUTHORITY["freeze_publication_authorized_now"]
        is False
        and AUTHORITY["algorithm_build_authorized_now"] is False
        and AUTHORITY["checkpoint_publication_authorized_now"] is False,
    }
    return {"passed": all(checks.values()), "checks": checks}


__all__ = [
    "AUTHORITY",
    "BINARY_EVENT_CONTRACT_ID",
    "CANDIDATE_BINDING_STATE",
    "CANDIDATE_ID",
    "CANDIDATE_MODULE",
    "CANDIDATE_MODULE_PATH",
    "CHECKPOINT_REQUIRED_SUFFIXES",
    "EXPECTED_ACTOR_FEATURES",
    "EXPECTED_FULL_FEATURES",
    "FAIL_STATUS",
    "INPUT_RELATIVE_PATHS",
    "LOCK_PATH",
    "LOCK_STATUS",
    "MORPHOLOGY_WEIGHT",
    "OUTPUT_NAMES",
    "OUTPUT_ROOT",
    "PASS_STATUS",
    "PROTOCOL_ID",
    "Q2_FINAL_RECEIPT_PATH",
    "Q2_PIPELINE_LEDGER_PATH",
    "Q2_PROTOCOL_FREEZE_PATH",
    "REQUIRED_CHECKS",
    "SCHEMA_VERSION",
    "SOURCE_RELATIVE_PATHS",
    "SOURCE_TOPOLOGY_ID",
    "STANDARD_RL_MODULE_KIND",
    "TARGET_FIXED_CONFIG",
    "TARGET_REWARD_CONFIG",
    "ZERO_COUNTER_NAMES",
    "candidate_id_for_tree",
    "contract_self_check",
]
