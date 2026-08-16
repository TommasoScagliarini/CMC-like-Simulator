"""Pure contract for the post-qualification H0 V3 zero-update trainer port.

This module is intentionally free of Ray, Torch, OpenSim, NumPy, and filesystem
side effects.  It defines the one-shot paths and closed authority consumed by
the freezer and execution driver.
"""

from __future__ import annotations


SCHEMA_VERSION = 1
PROTOCOL_ID = "AB06_H0_PRIMARY_GRF_SPLIT_V3_ZERO_UPDATE_PORT"
SOURCE_PROTOCOL_ID = "AB06_H0_PRIMARY_GRF_SPLIT_V3_SEMANTIC_REPLAY"
REVISION = "2026-08-06"
LOCK_STATUS = "H0_PRIMARY_SPLIT_V3_ZERO_UPDATE_PORT_FROZEN"
PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V3_ZERO_UPDATE_PORT"
FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V3_ZERO_UPDATE_PORT"
QUALIFICATION_PASS_STATUS = (
    "PASS_H0_PRIMARY_SPLIT_V3_AUTONOMOUS_QUALIFICATION"
)
SO_POLICY_ID = "verified_status0_max_iter_v1"
EVENT_CONTRACT_ID = "primary_grf_split_v1+legacy_events_v1"
CANDIDATE_ID = "H0_primary_split_v3_semantic_replay"
EXPECTED_ACTOR_FEATURES = 35
EXPECTED_FULL_FEATURES = 84
DEFAULT_POLICY_ID = "default_policy"

QUALIFICATION_CASE_IDS = (
    "deterministic_offset_minus_0p20",
    "deterministic_offset_nominal",
    "deterministic_offset_plus_0p20",
    "stochastic_nominal_seed_126",
    "stochastic_nominal_seed_127",
    "stochastic_nominal_seed_128",
)

V3_RUN_ROOT_RELATIVE = (
    "validation/h0_primary_grf_split_adaptation_runs/"
    "2026-08-06_h0_primary_split_v3_semantic_replay"
)
LOCK_RELATIVE_PATH = (
    "validation/h0_primary_grf_split_v3_zero_update_port_lock.json"
)
OUTPUT_ROOT_RELATIVE_PATH = f"{V3_RUN_ROOT_RELATIVE}/zero_update_port"

INPUT_RELATIVE_PATHS = {
    "v3_execution_lock": "validation/h0_primary_grf_split_v3_execution_lock.json",
    "v3_execution_ledger": f"{V3_RUN_ROOT_RELATIVE}/execution_ledger.json",
    "candidate_freeze": f"{V3_RUN_ROOT_RELATIVE}/adaptation/candidate_freeze.json",
    "candidate_receipt": f"{V3_RUN_ROOT_RELATIVE}/adaptation/receipt.json",
    "candidate_module_state": (
        f"{V3_RUN_ROOT_RELATIVE}/adaptation/rl_module_target_adapted/"
        "module_state.pkl"
    ),
    "candidate_module_ctor": (
        f"{V3_RUN_ROOT_RELATIVE}/adaptation/rl_module_target_adapted/"
        "class_and_ctor_args.pkl"
    ),
    "candidate_module_metadata": (
        f"{V3_RUN_ROOT_RELATIVE}/adaptation/rl_module_target_adapted/metadata.json"
    ),
    "candidate_actor_manifest": (
        f"{V3_RUN_ROOT_RELATIVE}/adaptation/rl_module_target_adapted/"
        "actor_feature_manifest.json"
    ),
    "holdout_receipt": f"{V3_RUN_ROOT_RELATIVE}/holdout/receipt.json",
    "qualification_lock": (
        "validation/h0_primary_grf_split_v3_qualification_lock.json"
    ),
    "qualification_execution_ledger": (
        f"{V3_RUN_ROOT_RELATIVE}/qualification/"
        "qualification_execution_ledger.json"
    ),
    "qualification_decision_receipt": (
        f"{V3_RUN_ROOT_RELATIVE}/qualification/"
        "baseline_tolerance_decision_receipt.json"
    ),
    "v3_training_config": (
        "validation/critic_warmup/"
        "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/"
        "training_cfg.resolved.yaml"
    ),
}

for _qualification_case_id in QUALIFICATION_CASE_IDS:
    INPUT_RELATIVE_PATHS[f"qualification_gate_{_qualification_case_id}"] = (
        f"{V3_RUN_ROOT_RELATIVE}/qualification/gates/"
        f"{_qualification_case_id}.json"
    )

SOURCE_RELATIVE_PATHS = {
    "contract": "validation/h0_primary_grf_split_v3_zero_update_contract.py",
    "freezer": "validation/freeze_h0_primary_grf_split_v3_zero_update.py",
    "driver": "validation/run_h0_primary_grf_split_v3_zero_update_port.py",
    "tests": "validation/test_h0_primary_grf_split_v3_zero_update_port.py",
    "training_entrypoint": "Trajectory Generator/baseline_MLP/train_ppo_mlp.py",
    "training_config": "Trajectory Generator/baseline_MLP/training_config.py",
    "warm_start": "Trajectory Generator/baseline_MLP/warm_start.py",
    "asymmetric_module": (
        "Trajectory Generator/baseline_MLP/asymmetric_rl_module.py"
    ),
    "environment_factory": "Trajectory Generator/baseline_MLP/env_factory.py",
    "environment": "Trajectory Generator/osim_trj_cmc_like.py",
}

# These values override only orchestration/runtime identity.  Network, physics,
# observation, and reward values still come from the frozen V3 resolved config.
TARGET_CONFIG_OVERRIDES = {
    "iterations": 0,
    "num_env_runners": 0,
    "ray_num_cpus": 1,
    "tensorboard": False,
    "progress": False,
    "update_history": False,
    "exact_start_sampling": False,
    "phase_fsm_input_mode": "legacy_events",
    "event_contract_id": EVENT_CONTRACT_ID,
    "binary_phase_fsm_mode": "disabled",
    "binary_phase_detector_profile": None,
}

AUTHORITY = {
    "fresh_target_algorithm_build_authorized": True,
    "candidate_actor_transplant_authorized": True,
    "full_zero_update_checkpoint_save_reload_authorized": True,
    "actor_gradient_updates_authorized": False,
    "critic_updates_authorized": False,
    "ppo_updates_authorized": False,
    "environment_sampling_authorized": False,
    "source_critic_restore_authorized": False,
    "source_optimizer_restore_authorized": False,
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
