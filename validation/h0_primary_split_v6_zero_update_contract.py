"""Pure contract for the qualified V6/P1 zero-update trainer port.

The contract is source-only: it authorizes neither qualification, Algorithm
execution, PPO updates nor publication.  A future execution lock must bind the
same immutable P1 candidate across its freeze, development PASS and
qualification PASS before the runner may build a fresh trainer at progress 0.
"""

from __future__ import annotations

from pathlib import PurePosixPath


SCHEMA_VERSION = 6
REVISION = "2026-08-06"
PROTOCOL_ID = "AB06_H0_PRIMARY_SPLIT_V6_P1_ZERO_UPDATE_PORT"
SOURCE_PROTOCOL_ID = "AB06_H0_PRIMARY_SPLIT_V6_V25_RESIDUAL_DAGGER"
QUALIFICATION_PROTOCOL_ID = "AB06_H0_PRIMARY_SPLIT_V6_BINARY_EVENT_QUALIFICATION"
LOCK_STATUS = "H0_PRIMARY_SPLIT_V6_P1_ZERO_UPDATE_PORT_FROZEN"
PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V6_P1_ZERO_UPDATE_PORT"
FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V6_P1_ZERO_UPDATE_PORT"
CANDIDATE_FREEZE_STATUS = "H0_PRIMARY_SPLIT_V6_CANDIDATE_FROZEN"
DEVELOPMENT_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V6_DEVELOPMENT"
QUALIFICATION_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V6_QUALIFICATION"

RL_MODULE_KIND = "primary_split_v25_residual"
EVENT_CONTRACT_ID = "legacy_events_v1"
BINARY_EVENT_CONTRACT_ID = "binary_point_v25+functional_contact_fsm_v1"
TARGET_BUNDLE_CONTRACT_ID = (
    "primary_grf_split_v1+binary_point_v25+functional_contact_fsm_v1"
)
SO_POLICY_ID = "verified_status0_max_iter_v1"
DEFAULT_POLICY_ID = "default_policy"
MORPHOLOGY_WEIGHT = 0.0
EXPECTED_ACTOR_FEATURES = 35
EXPECTED_FULL_FEATURES = 84
RESIDUAL_INPUT_INDICES = tuple(range(2, 35))
RESIDUAL_INPUT_COUNT = 33
RESIDUAL_LIMIT_COUNT = 2
RESIDUAL_LIMITS = (0.175, 0.12)
RESIDUAL_ARCHITECTURE = (33, 128, 128, 2)
RESIDUAL_INIT_SEED = 20260806

RUN_ROOT = PurePosixPath(
    "validation/h0_primary_grf_split_adaptation_runs/"
    "2026-08-06_h0_primary_split_v6_v25_residual"
)
CANDIDATE_ROOT = RUN_ROOT / "adaptation" / "rl_module_target_v25_residual_p1"
LOCK_PATH = PurePosixPath(
    "validation/h0_primary_split_v6_zero_update_execution_lock.json"
)
OUTPUT_ROOT = RUN_ROOT / "zero_update_port"

INPUT_PATHS = {
    "candidate_freeze": (RUN_ROOT / "adaptation" / "candidate_freeze.json").as_posix(),
    "development_receipt": (RUN_ROOT / "development" / "receipt.json").as_posix(),
    "qualification_receipt": (
        PurePosixPath("validation/h0_primary_split_v6_qualification_runs")
        / "receipt.json"
    ).as_posix(),
    "qualification_ledger": (
        PurePosixPath("validation/h0_primary_split_v6_qualification_runs")
        / "execution_ledger.json"
    ).as_posix(),
    "candidate_module_state": (CANDIDATE_ROOT / "module_state.pkl").as_posix(),
    "candidate_module_ctor": (CANDIDATE_ROOT / "class_and_ctor_args.pkl").as_posix(),
    "candidate_module_metadata": (CANDIDATE_ROOT / "metadata.json").as_posix(),
    "source_training_config": (
        "validation/critic_warmup/"
        "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/"
        "training_cfg.resolved.yaml"
    ),
    "v25_binary_profile": (
        "validation/binary_phase_detector_v25_geometry_runs/"
        "2026-08-04_local_reach_sweep_dev02_04_08/"
        "selected_candidate_profile.json"
    ),
    "legacy_analog_detector_profile": (
        "online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json"
    ),
}

SOURCE_PATHS = {
    "contract": "validation/h0_primary_split_v6_zero_update_contract.py",
    "freezer": "validation/freeze_h0_primary_split_v6_zero_update.py",
    "runner": "validation/run_h0_primary_split_v6_zero_update_port.py",
    "tests": "validation/test_h0_primary_split_v6_zero_update_port.py",
    "qualification_contract": "validation/h0_primary_split_v6_qualification_contract.py",
    "residual_module": (
        "Trajectory Generator/baseline_MLP/primary_split_v25_residual.py"
    ),
    "asymmetric_module": "Trajectory Generator/baseline_MLP/asymmetric_rl_module.py",
    "training_entrypoint": "Trajectory Generator/baseline_MLP/train_ppo_mlp.py",
    "training_config": "Trajectory Generator/baseline_MLP/training_config.py",
    "environment": "Trajectory Generator/osim_trj_cmc_like.py",
    "binary_active_adapter": "Trajectory Generator/binary_phase_adapter.py",
}

TARGET_FIXED_CONFIG = {
    "iterations": 0,
    "num_env_runners": 0,
    "ray_num_cpus": 1,
    "tensorboard": False,
    "progress": False,
    "update_history": False,
    "exact_start_sampling": False,
    "asymmetric_actor_critic": True,
    "rl_module_kind": RL_MODULE_KIND,
    "freeze_logstd": True,
    "freeze_actor": False,
    "phase_fsm_input_mode": "legacy_events",
    "event_contract_id": EVENT_CONTRACT_ID,
    "binary_phase_fsm_mode": "binary_active",
    "binary_phase_event_contract_id": BINARY_EVENT_CONTRACT_ID,
}

AUTHORITY = {
    "source_scaffold_authorized": True,
    "execution_requires_frozen_lock": True,
    "fresh_target_algorithm_build_authorized_after_lock": True,
    "qualified_full_policy_transplant_authorized_after_lock": True,
    "full_zero_update_checkpoint_save_reload_authorized_after_lock": True,
    "source_critic_restore_authorized": False,
    "source_optimizer_restore_authorized": False,
    "actor_updates_authorized": False,
    "critic_updates_authorized": False,
    "ppo_updates_authorized": False,
    "environment_sampling_authorized": False,
    "protected_trial_access_authorized": False,
    "runtime_promotion_authorized": False,
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
    "same_p1_freeze_development_qualification",
    "qualification_pass",
    "explicit_v25_residual_module_selected",
    "residual_config_serialized_exact",
    "morphology_weight_zero",
    "fresh_target_critic",
    "fresh_empty_optimizer",
    "base_actor_residual_logstd_exact",
    "residual_parameters_registered_in_optimizer",
    "zero_progress_before_save",
    "full_checkpoint_saved_at_zero",
    "save_reload_policy_exact",
    "save_reload_critic_exact",
    "save_reload_optimizer_empty",
    "zero_progress_after_restore",
    "no_algorithm_train_call",
    "no_actor_update",
    "no_critic_update",
    "no_ppo_update",
    "no_environment_sample",
    "no_protected_access",
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
