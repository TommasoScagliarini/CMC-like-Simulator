"""Pure closed contract for V3 zero-port/V25 A/B/C qualification."""

from __future__ import annotations


SCHEMA_VERSION = 1
PROTOCOL_ID = "AB06_H0_V3_ZERO_PORT_V25_ABC"
REVISION = "2026-08-06"
LOCK_STATUS = "H0_V3_ZERO_PORT_V25_ABC_EXECUTION_FROZEN"
PASS_STATUS = "H0_BINARY_V25_CANDIDATE_READY"
FAIL_STATUS = "FAIL_H0_V3_ZERO_PORT_V25_ABC"
SO_POLICY_ID = "verified_status0_max_iter_v1"
SOURCE_EVENT_CONTRACT_ID = "primary_grf_split_v1+legacy_events_v1"
V25_SHADOW_CONTRACT_ID = "binary_point_v25+functional_contact_fsm_v1_shadow"
V25_ACTIVE_CONTRACT_ID = "binary_point_v25+functional_contact_fsm_v1"
EXPECTED_STEPS = 500
EXPECTED_CONTROL_WINDOWS = 5000
EXPECTED_RAW_SAMPLES = 5000
EXPECTED_ACTOR_FEATURES = 35
EXPECTED_FULL_FEATURES = 84
MORPHOLOGY_WEIGHT = 0.0
CANONICAL_OFFSET_S = 1.956870983805102
STOCHASTIC_SIGMA = 0.005

V3_RUN_ROOT = (
    "validation/h0_primary_grf_split_adaptation_runs/"
    "2026-08-06_h0_primary_split_v3_semantic_replay"
)
ZERO_PORT_ROOT = f"{V3_RUN_ROOT}/zero_update_port"
LOCK_RELATIVE_PATH = "validation/h0_v3_v25_abc_post_zero_port_lock.json"
RUN_ROOT_RELATIVE_PATH = (
    "validation/h0_v3_v25_abc_runs/"
    "2026-08-06_h0_v3_zero_port_v25_abc"
)

INPUT_RELATIVE_PATHS = {
    "v3_execution_lock": "validation/h0_primary_grf_split_v3_execution_lock.json",
    "v3_qualification_ledger": (
        f"{V3_RUN_ROOT}/qualification/qualification_execution_ledger.json"
    ),
    "zero_port_lock": "validation/h0_primary_grf_split_v3_zero_update_port_lock.json",
    "zero_port_receipt": f"{ZERO_PORT_ROOT}/receipt.json",
    "zero_port_ledger": f"{ZERO_PORT_ROOT}/execution_ledger.json",
    "zero_port_config": f"{ZERO_PORT_ROOT}/training_cfg.resolved.yaml",
    "zero_port_module_state": f"{ZERO_PORT_ROOT}/rl_module_reloaded/module_state.pkl",
    "zero_port_module_ctor": (
        f"{ZERO_PORT_ROOT}/rl_module_reloaded/class_and_ctor_args.pkl"
    ),
    "zero_port_module_metadata": f"{ZERO_PORT_ROOT}/rl_module_reloaded/metadata.json",
    "zero_port_actor_manifest": (
        f"{ZERO_PORT_ROOT}/rl_module_reloaded/actor_feature_manifest.json"
    ),
    "v25_candidate_freeze": (
        "validation/binary_phase_detector_v25_development_candidate_freeze_lock.json"
    ),
    "v25_shadow_readiness": (
        "validation/binary_phase_detector_v25_shadow_readiness_receipt.json"
    ),
    "v25_profile": (
        "validation/binary_phase_detector_v25_geometry_runs/"
        "2026-08-04_local_reach_sweep_dev02_04_08/selected_candidate_profile.json"
    ),
    "analog_detector_profile": (
        "online_grf_profiles/"
        "AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json"
    ),
    "noise_manifest": "validation/h0_primary_grf_split_noise_tapes/manifest.json",
    "noise_deterministic": (
        "validation/h0_primary_grf_split_noise_tapes/"
        "qualification_deterministic_all_zero.npz"
    ),
    "noise_seed126": (
        "validation/h0_primary_grf_split_noise_tapes/"
        "qualification_trial_02_standard_normal.npz"
    ),
    "noise_seed127": (
        "validation/h0_primary_grf_split_noise_tapes/"
        "qualification_trial_04_standard_normal.npz"
    ),
    "noise_seed128": (
        "validation/h0_primary_grf_split_noise_tapes/"
        "qualification_trial_08_standard_normal.npz"
    ),
}

SOURCE_RELATIVE_PATHS = {
    "contract": "validation/h0_v3_v25_abc_post_zero_port_contract.py",
    "driver": "validation/run_h0_v3_v25_abc_post_zero_port.py",
    "freezer": "validation/freeze_h0_v3_v25_abc_post_zero_port.py",
    "comparator": "validation/compare_h0_v3_v25_abc_post_zero_port.py",
    "contract_tests": "validation/test_h0_v3_v25_abc_post_zero_port_contract.py",
    "driver_tests": "validation/test_run_h0_v3_v25_abc_post_zero_port.py",
    "freezer_tests": "validation/test_freeze_h0_v3_v25_abc_post_zero_port.py",
    "comparator_tests": "validation/test_compare_h0_v3_v25_abc_post_zero_port.py",
    "legacy_abc_driver": "validation/run_h0_v25_abc_preflight.py",
    "legacy_abc_comparator": "validation/compare_h0_v25_abc.py",
    "so_classifier": "validation/h0_v3_so_recovery_contract.py",
    "zero_port_contract": "validation/h0_primary_grf_split_v3_zero_update_contract.py",
    "zero_port_driver": "validation/run_h0_primary_grf_split_v3_zero_update_port.py",
    "environment": "Trajectory Generator/osim_trj_cmc_like.py",
    "binary_phase_adapter": "Trajectory Generator/binary_phase_adapter.py",
    "binary_phase_fsm": "Trajectory Generator/binary_phase_fsm.py",
    "binary_phase_detector": "binary_phase_detector.py",
    "rollout_eval": "Trajectory Generator/baseline_MLP/rollout_eval.py",
    "training_config": "Trajectory Generator/baseline_MLP/training_config.py",
    "simulation_runner": "simulation_runner.py",
    "static_optimization": "static_optimization.py",
    "model_loader": "model_loader.py",
    "online_grf": "online_grf.py",
    "root_config": "config.py",
}

CASES = (
    {
        "case_id": "deterministic_offset_minus_0p20",
        "action_selection": "deterministic",
        "offset_s": CANONICAL_OFFSET_S - 0.2,
        "seed": 123,
        "sigma": 0.0,
        "noise_input": "noise_deterministic",
    },
    {
        "case_id": "deterministic_offset_nominal",
        "action_selection": "deterministic",
        "offset_s": CANONICAL_OFFSET_S,
        "seed": 123,
        "sigma": 0.0,
        "noise_input": "noise_deterministic",
    },
    {
        "case_id": "deterministic_offset_plus_0p20",
        "action_selection": "deterministic",
        "offset_s": CANONICAL_OFFSET_S + 0.2,
        "seed": 123,
        "sigma": 0.0,
        "noise_input": "noise_deterministic",
    },
    {
        "case_id": "stochastic_nominal_seed_126",
        "action_selection": "stochastic",
        "offset_s": CANONICAL_OFFSET_S,
        "seed": 126,
        "sigma": STOCHASTIC_SIGMA,
        "noise_input": "noise_seed126",
    },
    {
        "case_id": "stochastic_nominal_seed_127",
        "action_selection": "stochastic",
        "offset_s": CANONICAL_OFFSET_S,
        "seed": 127,
        "sigma": STOCHASTIC_SIGMA,
        "noise_input": "noise_seed127",
    },
    {
        "case_id": "stochastic_nominal_seed_128",
        "action_selection": "stochastic",
        "offset_s": CANONICAL_OFFSET_S,
        "seed": 128,
        "sigma": STOCHASTIC_SIGMA,
        "noise_input": "noise_seed128",
    },
)

MODES = {
    "A": {
        "binary_phase_fsm_mode": "disabled",
        "event_source": "legacy_events",
        "binary_phase_event_contract_id": V25_SHADOW_CONTRACT_ID,
    },
    "B": {
        "binary_phase_fsm_mode": "binary_shadow",
        "event_source": "legacy_events",
        "binary_phase_event_contract_id": V25_SHADOW_CONTRACT_ID,
    },
    "C": {
        "binary_phase_fsm_mode": "binary_active",
        "event_source": "v25_fsm_v20",
        "binary_phase_event_contract_id": V25_ACTIVE_CONTRACT_ID,
    },
}

AUTHORITY = {
    "full_environment_rollout_authorized": True,
    "zero_port_actor_only": True,
    "actor_updates_authorized": False,
    "critic_updates_authorized": False,
    "ppo_updates_authorized": False,
    "training_authorized": False,
    "protected_trial_access_authorized": False,
    "primary_grf_modification_authorized": False,
    "detector_retuning_authorized": False,
    "sea_semantic_modification_authorized": False,
    "runtime_promotion_authorized": False,
}

TERMINAL = {
    "pass": PASS_STATUS,
    "fail": FAIL_STATUS,
    "next_on_pass": PASS_STATUS,
    "next_on_fail": "STOP_WITHOUT_RETRY_RETUNING_OR_FALLBACK",
}
