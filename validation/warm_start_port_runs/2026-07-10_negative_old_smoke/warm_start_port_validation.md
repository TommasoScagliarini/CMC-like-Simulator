# Warm-start Port Validation

- Status: **FAIL**
- Hard failures: 9
- Warnings: 5
- Source: `/Users/tommy/Documents/CMC-like-Simulator - Claude/Trajectory Generator/runs/training/MLP_imitation_training_06-23-2026_grfsoft_knee1_ankle2_100iter/rl_module_best`
- Target: `/Users/tommy/Documents/CMC-like-Simulator - Claude/Trajectory Generator/runs/training/_warm_start_smoke_20260709_smallbatch/rl_module_initial_warm_start`

## Checks

- [PASS] `artifact_exists.source_state`
- [PASS] `artifact_exists.source_ctor`
- [PASS] `artifact_exists.source_metadata`
- [PASS] `artifact_exists.target_state`
- [PASS] `artifact_exists.target_ctor`
- [PASS] `artifact_exists.target_metadata`
- [PASS] `artifact_exists.source_config`
- [PASS] `artifact_exists.target_config`
- [PASS] `artifact_exists.transplant_report`
- [FAIL] `saved_target_matches_recomputed_transplant`
- [PASS] `source_checkpoint_is_actor_only`
- [FAIL] `target_critic_state_unchanged_during_transplant`
- [FAIL] `disabled_gait_clock_columns_zeroed`
- [FAIL] `learner_actor_matches_transplant`
- [FAIL] `env_runner_actors_synced_before_sampling`
- [FAIL] `saved_initial_actor_matches_live_actor`
- [FAIL] `source_optimizer_not_loaded`
- [PASS] `config_contract.model.num_hidden_layers`
- [PASS] `config_contract.model.dim_hidden_layers`
- [PASS] `config_contract.model.fcnet_activation`
- [PASS] `config_contract.simulation.segment_duration`
- [PASS] `config_contract.simulation.policy_knots`
- [PASS] `config_contract.simulation.action_mode`
- [PASS] `config_contract.simulation.max_delta_rad`
- [PASS] `config_contract.simulation.pros_ref_governor`
- [PASS] `config_contract.simulation.pros_ref_model`
- [PASS] `config_contract.simulation.pros_ref_cutoff_hz`
- [PASS] `config_contract.simulation.pros_knee_ref_velocity_limit_rad_s`
- [PASS] `config_contract.simulation.pros_ankle_ref_velocity_limit_rad_s`
- [PASS] `config_contract.simulation.pros_knee_ref_acceleration_limit_rad_s2`
- [PASS] `config_contract.simulation.pros_ankle_ref_acceleration_limit_rad_s2`
- [PASS] `config_contract.simulation.pros_knee_ref_jerk_limit_rad_s3`
- [PASS] `config_contract.simulation.pros_ankle_ref_jerk_limit_rad_s3`
- [PASS] `module_class_match`
- [PASS] `action_space_shape_match`
- [PASS] `hidden_architecture_match`
- [PASS] `ray_checkpoint_version_match`
- [PASS] `feature_manifest_widths_match_modules`
- [FAIL] `functional_equivalence_on_aligned_inputs`
- [PASS] `target_only_features_initially_inert`
- [PASS] `online_grf_physics_profile_equivalent`
- [WARN] `source_profile_recovered_from_git_not_worktree`
- [WARN] `online_event_detector_domain_shift`
- [WARN] `sound_gait_clock_domain_shift_adapted_by_zeroing`
- [WARN] `episode_start_domain_shift`
- [WARN] `target_slew_limiter_is_target_only_postprocessing`
- [FAIL] `functional_equivalence_on_real_target_observations`

## Decision

The actor port is not ready for H1; resolve hard failures first.
