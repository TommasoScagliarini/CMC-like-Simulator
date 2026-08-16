"""Pure immutable schema for the H0 primary-split V3 execution freeze.

This module intentionally imports no project runtime, numerical package,
OpenSim, Ray, or filesystem helper.  It defines only canonical POSIX-relative
paths and closed protocol values consumed by the V3 freezer and its tests.
"""

from __future__ import annotations


SCHEMA_VERSION = 1
TRAIN_SEEDS = (123, 124)
FINAL_HOLDOUT_SEED = 125
DIAGNOSTIC_REPLICATES = (1, 2)

STRICT_ZERO_POLICY = "strict_zero_all_fallbacks_v1"
VERIFIED_SUCCESS_POLICY = "verified_historical_bounded_ls_success_v1"
VERIFIED_STATUS0_MAX_ITER_POLICY = "verified_status0_max_iter_v1"

# The first two policies preserve the original strict choices.  The third is
# the explicitly authorized addendum: it does not rewrite SciPy's ``success``
# flag, but permits the exact independently verified status-0/max-iteration
# signature under frozen numerical and determinism limits.
SO_POLICIES = {
    STRICT_ZERO_POLICY: {
        "raw_fallback_count_may_be_nonzero": False,
        "verified_historical_bounded_ls_allowed": False,
        "bounded_lsq_success_required": True,
        "solver_status_override_allowed": False,
        "status_zero_without_success_allowed": False,
        "max_iter_without_success_allowed": False,
        "sea_plugin_fallbacks_allowed": False,
        "hard_so_fallbacks_allowed": False,
        "reuse_previous_allowed": False,
        "bounded_ls_unsuccessful_allowed": False,
        "bounds_violations_allowed": False,
        "nonfinite_solver_allowed": False,
        "selected_infeasible_allowed": False,
        "selected_solution_mismatch_allowed": False,
        "residual_contract_mismatch_allowed": False,
    },
    VERIFIED_SUCCESS_POLICY: {
        "raw_fallback_count_may_be_nonzero": True,
        "verified_historical_bounded_ls_allowed": True,
        "bounded_lsq_success_required": True,
        "solver_status_override_allowed": False,
        "status_zero_without_success_allowed": False,
        "max_iter_without_success_allowed": False,
        "sea_plugin_fallbacks_allowed": False,
        "hard_so_fallbacks_allowed": False,
        "reuse_previous_allowed": False,
        "bounded_ls_unsuccessful_allowed": False,
        "bounds_violations_allowed": False,
        "nonfinite_solver_allowed": False,
        "selected_infeasible_allowed": False,
        "selected_solution_mismatch_allowed": False,
        "residual_contract_mismatch_allowed": False,
    },
    VERIFIED_STATUS0_MAX_ITER_POLICY: {
        "raw_fallback_count_may_be_nonzero": True,
        "verified_historical_bounded_ls_allowed": True,
        "bounded_lsq_success_required": False,
        "solver_status_override_allowed": False,
        "status_zero_without_success_allowed": True,
        "max_iter_without_success_allowed": True,
        "required_bounded_lsq_status": 0,
        "required_bounded_lsq_iterations": 1000,
        "required_bounded_lsq_message": (
            "The maximum number of iterations is exceeded."
        ),
        "bounded_lsq_optimality_max": 1.0e-8,
        "input_and_output_finite_required": True,
        "bound_violation_max": 1.0e-9,
        "feasibility_abs_or_max_tolerance": 1.0e-6,
        "feasibility_relative_tolerance": 1.0e-3,
        "residual_telemetry_consistency_atol": 1.0e-12,
        "selected_solution_served_match_required": True,
        "determinism_bit_exact_required": True,
        "sea_plugin_fallbacks_allowed": False,
        "hard_so_fallbacks_allowed": True,
        "reuse_previous_allowed": False,
        "bounded_ls_unsuccessful_allowed": True,
        "bounds_violations_allowed": False,
        "nonfinite_solver_allowed": False,
        "selected_infeasible_allowed": False,
        "selected_solution_mismatch_allowed": False,
        "residual_contract_mismatch_allowed": False,
    },
}

LOCK_SOURCE_RELATIVE_PATHS = {
    "protocol_plan": (
        "reports/plans/2026-08-06_protocollo_h0_primary_split_v3_semantic_replay.md"
    ),
    "runner": "validation/run_h0_primary_grf_split_v3_semantic_replay.py",
    "tests": "validation/test_h0_primary_grf_split_v3.py",
}

PROTOCOL_SOURCE_RELATIVE_PATHS = {
    "execution_freezer": ("validation/freeze_h0_primary_grf_split_v3_execution.py"),
    "execution_freezer_tests": (
        "validation/test_freeze_h0_primary_grf_split_v3_execution.py"
    ),
    "status0_evidence_builder": (
        "validation/build_h0_primary_grf_split_v3_status0_evidence.py"
    ),
    "status0_evidence_builder_tests": (
        "validation/test_build_h0_primary_grf_split_v3_status0_evidence.py"
    ),
    "freeze_contract": "validation/h0_primary_grf_split_v3_freeze_contract.py",
    "diagnostic_comparator": (
        "validation/compare_h0_primary_grf_split_v3_diagnostics.py"
    ),
    "diagnostic_comparator_tests": (
        "validation/test_compare_h0_primary_grf_split_v3_diagnostics.py"
    ),
    "so_recovery_contract": "validation/h0_v3_so_recovery_contract.py",
    "so_recovery_tests": "validation/test_h0_v3_so_recovery_contract.py",
    "static_optimization_audit_tests": (
        "validation/test_static_optimization_solver_audit.py"
    ),
    "primary_split_contract_tests": ("validation/test_primary_grf_split_adaptation.py"),
    "imitation_split_tests": ("validation/test_target_domain_imitation_split.py"),
}

RUNTIME_SOURCE_RELATIVE_PATHS = {
    "strict_json": "validation/compare_h0_v25_abc.py",
    "v1_shared_adapter": ("validation/run_h0_primary_grf_split_v1_adaptation.py"),
    "h0_runtime_adapter": "validation/run_h0_v25_abc_preflight.py",
    "primary_split_contract": (
        "Trajectory Generator/baseline_MLP/primary_grf_split_adaptation.py"
    ),
    "actor_fit": ("Trajectory Generator/baseline_MLP/target_domain_imitation.py"),
    "selected_column_audit": (
        "Trajectory Generator/baseline_MLP/target_domain_markov_adaptation.py"
    ),
    "noise_adaptation_import": (
        "Trajectory Generator/baseline_MLP/target_domain_noise_adaptation.py"
    ),
    "warm_start": "Trajectory Generator/baseline_MLP/warm_start.py",
    "bootstrap": "Trajectory Generator/baseline_MLP/_bootstrap.py",
    "env_factory": "Trajectory Generator/baseline_MLP/env_factory.py",
    "rollout_eval": "Trajectory Generator/baseline_MLP/rollout_eval.py",
    "exploration_noise": ("Trajectory Generator/baseline_MLP/exploration_noise.py"),
    "training_config": ("Trajectory Generator/baseline_MLP/training_config.py"),
    "process_watchdog": ("Trajectory Generator/baseline_MLP/process_watchdog.py"),
    "progress_display": ("Trajectory Generator/baseline_MLP/progress_display.py"),
    "reward_function": ("Trajectory Generator/baseline_MLP/reward_function.py"),
    "experimental_morphology_corridor": (
        "Trajectory Generator/baseline_MLP/experimental_morphology_corridor.py"
    ),
    "asymmetric_rl_module": (
        "Trajectory Generator/baseline_MLP/asymmetric_rl_module.py"
    ),
    "win_runtime": "Trajectory Generator/baseline_MLP/win_runtime.py",
    "environment": "Trajectory Generator/osim_trj_cmc_like.py",
    "binary_phase_adapter": "Trajectory Generator/binary_phase_adapter.py",
    "binary_phase_fsm": "Trajectory Generator/binary_phase_fsm.py",
    "phase_fsm": "Trajectory Generator/prosthetic_phase_fsm.py",
    "binary_phase_detector": "binary_phase_detector.py",
    "root_config": "config.py",
    "path_resolver": "path_resolver.py",
    "setup_io": "setup_io.py",
    "model_loader": "model_loader.py",
    "online_grf": "online_grf.py",
    "simulation_runner": "simulation_runner.py",
    "static_optimization": "static_optimization.py",
    "kinematics_interpolator": "kinematics_interpolator.py",
    "outer_loop": "outer_loop.py",
    "inverse_dynamics": "inverse_dynamics.py",
    "prosthesis_controller": "prosthesis_controller.py",
    "output": "output.py",
}

LOCK_INPUT_RELATIVE_PATHS = {
    "h0_config": (
        "validation/critic_warmup/"
        "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/"
        "training_cfg.resolved.yaml"
    ),
    "h0_module_state": (
        "validation/critic_warmup/"
        "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/"
        "rl_module_last/module_state.pkl"
    ),
    "h0_module_ctor": (
        "validation/critic_warmup/"
        "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/"
        "rl_module_last/class_and_ctor_args.pkl"
    ),
    "h0_module_metadata": (
        "validation/critic_warmup/"
        "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/"
        "rl_module_last/metadata.json"
    ),
}

HISTORICAL_INPUT_RELATIVE_PATH_TEMPLATES = {
    "trace": (
        "validation/controller_memory_ablation/"
        "2026-07-13_markov35_corrected_full_sigma0005_seed{seed}/"
        "rollout_policy_trace.json"
    ),
    "summary": (
        "validation/controller_memory_ablation/"
        "2026-07-13_markov35_corrected_full_sigma0005_seed{seed}/"
        "rollout_summary.json"
    ),
}

RUNTIME_INPUT_RELATIVE_PATHS = {
    "actor_layout_reference": (
        "validation/critic_warmup/"
        "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/"
        "actor_transplant_report.json"
    ),
    "full_layout_reference": HISTORICAL_INPUT_RELATIVE_PATH_TEMPLATES["summary"].format(
        seed=123
    ),
    "runtime_setup": (
        "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml"
    ),
    "runtime_model": ("models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim"),
    "kinematics": ("models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot"),
    "external_loads": ("models/AB06_SEASEA_Threadmill/data/ExternalForces.xml"),
    "prescribed_grf": (
        "models/AB06_SEASEA_Threadmill/data/AB06_SEASEA_GRF_FullSpan.mot"
    ),
    "reserve_actuators": ("models/AB06_SEASEA_Threadmill/data/CMC_Actuators.xml"),
    "primary_grf_profile": (
        "online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json"
    ),
    "legacy_detector_profile": (
        "online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json"
    ),
    "morphology_profile_weight_zero": (
        "Trajectory Generator/baseline_MLP/morphology_profiles/"
        "ab06_prosthetic_mean_std_corridor.json"
    ),
    "primary_core_lock": "validation/primary_grf_core_lock_2026-08-03.json",
    "online_grf_contact_cpp": (
        "tools/online_grf_contact/OnlineGRFSphereHalfSpaceForce.cpp"
    ),
    "online_grf_contact_header": (
        "tools/online_grf_contact/OnlineGRFSphereHalfSpaceForce.h"
    ),
    "online_grf_plugin_interface": ("tools/online_grf_contact/Plugin_interface.cpp"),
    "online_grf_contact_cmake": "tools/online_grf_contact/CMakeLists.txt",
    "online_grf_macos_dylib": "plugins/libOnlineGRFContact.dylib",
    "sea_cpp": "tools/sea_plugin_relative_d/SeriesElasticActuator.cpp",
    "sea_header": "tools/sea_plugin_relative_d/SeriesElasticActuator.h",
    "sea_cmake": "tools/sea_plugin_relative_d/CMakeLists.txt",
    "sea_macos_dylib": ("plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib"),
}

EVIDENCE_RELATIVE_PATHS = {
    "so_policy_decision": (
        "validation/h0_primary_grf_split_v3_so_policy_decision.json"
    ),
    "diagnostic_determinism": (
        "validation/h0_primary_grf_split_v3_diagnostic_determinism_receipt.json"
    ),
    "instrumented_preflight": (
        "validation/h0_primary_grf_split_v3_instrumented_preflight_receipt.json"
    ),
    "preflight_tests": (
        "validation/h0_primary_grf_split_v3_preflight_test_receipt.json"
    ),
    "platform": "validation/h0_primary_grf_split_v3_platform_receipt.json",
}

LINEAGE_RELATIVE_PATHS = {
    "v1_terminal_ledger": (
        "validation/h0_primary_grf_split_adaptation_runs/"
        "2026-08-05_h0_primary_grf_split_v1_one_shot/"
        "collection_execution_ledger.json"
    ),
    "v2_terminal_ledger": (
        "validation/h0_primary_grf_split_adaptation_runs/"
        "2026-08-06_h0_primary_split_v2_prescribed_teacher/"
        "execution_ledger.json"
    ),
    "initial_v3_decision_required_receipt": (
        "validation/h0_primary_grf_split_v3_development_preflight_receipt.json"
    ),
}

DIAGNOSTIC_ROOT_RELATIVE_PATH = (
    "validation/h0_primary_grf_split_v3_diagnostic_runs/2026-08-06_instrumented_final"
)
DIAGNOSTIC_FILENAMES = (
    "paired_replay.npz",
    "summary.json",
    "gate.json",
    "fallback_journal.json",
    "solver_audit_journal.json",
    "receipt.json",
)


def diagnostic_artifact_relative_paths() -> dict[str, dict[str, str]]:
    """Return the exact four-run diagnostic artifact path schema."""
    return {
        f"seed_{seed}_rep{replicate}": {
            filename: (
                f"{DIAGNOSTIC_ROOT_RELATIVE_PATH}/seed_{seed}_rep{replicate}/{filename}"
            )
            for filename in DIAGNOSTIC_FILENAMES
        }
        for seed in TRAIN_SEEDS
        for replicate in DIAGNOSTIC_REPLICATES
    }


DIAGNOSTIC_CHECK_KEYS = frozenset(
    {
        "train_seeds_only",
        "two_replicates_per_seed",
        "per_array_hashes_exact",
        "summary_semantic_projection_exact",
        "solver_audit_canonical_exact",
        "historical_parity_exact",
        "selected_policy_gate_passed",
        "all_hard_conditions_zero",
        "all_bounded_ls_verified",
        "zero_residual_contract_mismatch",
        "seed125_not_semantically_opened",
    }
)
PREFLIGHT_CHECK_KEYS = frozenset(
    {
        "diagnostic_determinism_pass",
        "layout_35_84_float32",
        "primary_left_only",
        "legacy_events",
        "morphology_weight_zero",
        "source_h0_not_behavior",
        "all_hard_conditions_zero",
        "zero_residual_contract_mismatch",
        "selected_policy_gate_passed",
        "no_actor_update",
        "no_critic_update",
        "no_ppo_update",
        "no_protected_access",
    }
)
TEST_CHECK_KEYS = frozenset(
    {
        "unit_tests_pass",
        "compile_pass",
        "strict_json_pass",
        "portable_paths_pass",
        "no_clobber_pass",
        "seed125_not_semantically_opened",
    }
)
REQUIRED_TEST_MODULES = (
    "validation.test_build_h0_primary_grf_split_v3_status0_evidence",
    "validation.test_freeze_h0_primary_grf_split_v3_execution",
    "validation.test_compare_h0_primary_grf_split_v3_diagnostics",
    "validation.test_h0_primary_grf_split_v3",
    "validation.test_h0_v3_so_recovery_contract",
    "validation.test_static_optimization_solver_audit",
    "validation.test_primary_grf_split_adaptation",
    "validation.test_target_domain_imitation_split",
)
