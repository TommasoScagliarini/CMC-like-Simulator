# Physical comparison after one PPO batch

Status: **all_requested_cases_finalized**, 9/9 requested cases finalized.

Paired standard normals; each policy uses its own state-dependent sigma. Reward and target equality are checked on the shared completed prefix.

| Case | Role | Scenario | Completed steps | End | Return | Penetration max, step summary (mm) |
|---|---|---|---:|---|---:|---:|
| before_nominal | before | nominal | 500 | episode_time_limit | 305.060 | 14.795 |
| before_noisy_123 | before | noisy | 500 | episode_time_limit | 264.206 | 23.716 |
| before_noisy_125 | before | noisy | 500 | episode_time_limit | 212.452 | 26.622 |
| before_noisy_127 | before | noisy | 458 | grf_penetration | 229.465 | 28.285 |
| after_nominal | after | nominal | 500 | episode_time_limit | 255.186 | 17.101 |
| after_noisy_123 | after | noisy | 500 | episode_time_limit | 223.887 | 26.820 |
| after_noisy_125 | after | noisy | 500 | episode_time_limit | 231.485 | 24.415 |
| after_noisy_127 | after | noisy | 500 | episode_time_limit | 267.620 | 24.183 |
| first_pass_nominal | first_pass | nominal | 500 | episode_time_limit | 272.857 | 14.795 |

## nominal

500 completed common steps; full-return comparison eligible: True.

Common analysis window: 13.150–17.950 s.

| Metric | Before | After |
|---|---:|---:|
| knee actual-target RMSE (deg) | 6.3758 | 6.3277 |
| knee normalized shape error, time sequence (dimensionless) | 0.38819 | 0.40186 |
| knee served-target RMSE (deg) | 6.3847 | 6.3732 |
| knee torque raw−LP20 RMS (Nm) | 3.7833 | 3.4296 |
| knee LP10 normalized total variation (/s) | 4.8309 | 4.6382 |
| ankle actual-target RMSE (deg) | 4.0401 | 5.3866 |
| ankle normalized shape error, time sequence (dimensionless) | 0.31462 | 0.39401 |
| ankle served-target RMSE (deg) | 4.0058 | 5.2974 |
| ankle torque raw−LP20 RMS (Nm) | 0.29864 | 0.31515 |
| ankle LP10 normalized total variation (/s) | 2.0149 | 1.7581 |
| Vertical impulse (Ns) | 354.28 | 363.99 |
| Logged contact fraction | 0.40438 | 0.3925 |
| Penetration max, native STO (mm) | 14.149 | 17.121 |
| pelvis_translation reserve vector RMS (N) | 418.95 | 421.39 |
| pelvis_rotation reserve vector RMS (Nm) | 92.065 | 92.538 |
| biological_rotation reserve vector RMS (Nm) | 49.925 | 49.343 |

## noisy_seed123

500 completed common steps; full-return comparison eligible: True.

Common analysis window: 13.150–17.950 s.

| Metric | Before | After |
|---|---:|---:|
| knee actual-target RMSE (deg) | 6.8666 | 6.3147 |
| knee normalized shape error, time sequence (dimensionless) | 0.47767 | 0.4226 |
| knee served-target RMSE (deg) | 6.826 | 6.3155 |
| knee torque raw−LP20 RMS (Nm) | 3.6061 | 3.4801 |
| knee LP10 normalized total variation (/s) | 6.9485 | 5.0745 |
| ankle actual-target RMSE (deg) | 3.6491 | 3.8864 |
| ankle normalized shape error, time sequence (dimensionless) | 0.41694 | 0.36029 |
| ankle served-target RMSE (deg) | 3.5277 | 3.7771 |
| ankle torque raw−LP20 RMS (Nm) | 0.31671 | 0.30205 |
| ankle LP10 normalized total variation (/s) | 3.0467 | 2.4927 |
| Vertical impulse (Ns) | 814.92 | 719.88 |
| Logged contact fraction | 0.47073 | 0.4924 |
| Penetration max, native STO (mm) | 23.719 | 26.838 |
| pelvis_translation reserve vector RMS (N) | 353.64 | 357.86 |
| pelvis_rotation reserve vector RMS (Nm) | 101.17 | 99.223 |
| biological_rotation reserve vector RMS (Nm) | 51.575 | 49.885 |

## noisy_seed125

500 completed common steps; full-return comparison eligible: True.

Common analysis window: 13.150–17.950 s.

| Metric | Before | After |
|---|---:|---:|
| knee actual-target RMSE (deg) | 9.7011 | 9.933 |
| knee normalized shape error, time sequence (dimensionless) | 0.56873 | 0.60317 |
| knee served-target RMSE (deg) | 9.6447 | 9.8777 |
| knee torque raw−LP20 RMS (Nm) | 3.5098 | 3.7803 |
| knee LP10 normalized total variation (/s) | 6.7109 | 7.4627 |
| ankle actual-target RMSE (deg) | 4.6236 | 4.7359 |
| ankle normalized shape error, time sequence (dimensionless) | 0.49489 | 0.53642 |
| ankle served-target RMSE (deg) | 4.5375 | 4.6841 |
| ankle torque raw−LP20 RMS (Nm) | 0.29875 | 0.32455 |
| ankle LP10 normalized total variation (/s) | 3.2085 | 2.9325 |
| Vertical impulse (Ns) | 478.32 | 412.7 |
| Logged contact fraction | 0.41833 | 0.4175 |
| Penetration max, native STO (mm) | 26.63 | 24.446 |
| pelvis_translation reserve vector RMS (N) | 414.27 | 415.53 |
| pelvis_rotation reserve vector RMS (Nm) | 96.828 | 94.609 |
| biological_rotation reserve vector RMS (Nm) | 50.643 | 50.521 |

## noisy_seed127

458 completed common steps; full-return comparison eligible: False.

Common analysis window: 13.150–17.419 s.

| Metric | Before | After |
|---|---:|---:|
| knee actual-target RMSE (deg) | 8.0426 | 7.779 |
| knee normalized shape error, time sequence (dimensionless) | 0.54374 | 0.47312 |
| knee served-target RMSE (deg) | 7.9108 | 7.7206 |
| knee torque raw−LP20 RMS (Nm) | 3.3192 | 3.5149 |
| knee LP10 normalized total variation (/s) | 6.6002 | 6.7373 |
| ankle actual-target RMSE (deg) | 3.7801 | 3.5783 |
| ankle normalized shape error, time sequence (dimensionless) | 0.43882 | 0.3922 |
| ankle served-target RMSE (deg) | 3.9327 | 3.5897 |
| ankle torque raw−LP20 RMS (Nm) | 0.31197 | 0.28861 |
| ankle LP10 normalized total variation (/s) | 2.9658 | 3.0232 |
| Vertical impulse (Ns) | 594.58 | 621.93 |
| Logged contact fraction | 0.48267 | 0.38077 |
| Penetration max, native STO (mm) | 27.737 | 24.203 |
| pelvis_translation reserve vector RMS (N) | 354.26 | 355.7 |
| pelvis_rotation reserve vector RMS (Nm) | 87.736 | 90.951 |
| biological_rotation reserve vector RMS (Nm) | 49.859 | 50.094 |

Kinematic shape in these tables uses the sampled time sequence in the stated window: RMS(zscore(actual) − zscore(target)), with no lag search, phase averaging or time warping. It is dimensionless. The earlier governor report also computed phase-mean shape errors; those are a different aggregation and must not be mixed with these values.

Detailed tracking, SEA torque shape, contact impulse and reserve comparisons: `metrics.json`.

- One PPO batch/update and three paired Gaussian seeds do not establish robustness, convergence, or statistical significance.
- Only standardized z is shared. State-dependent own sigma and realized action perturbations may change after PPO.
- All finalized episode returns, including true early terminations and their penalties, remain in own-policy descriptive summaries. The full-horizon flag only governs matched-horizon physical-quality comparison.
- Partial common-prefix rewards exclude terminal partial cost and missing future; they are separate from valid observed episode returns. Truncated 5s returns require a bootstrap convention for continuing-task value calibration.
- Operational wall timeouts, numerical failures, and max-step cutoffs retain their recorded return/status but are excluded from the task-episode mean as externally censored observations; true task terminations remain included.
- Noisy cases reproduce the RLlib training connector (normalize_actions=True, clip_actions=False, unsquash arithmetic); nominal cases retain the deployment evaluator pipeline for exact archived reuse. Nominal/noisy raw clipping penalties and arithmetic are therefore not interchangeable.
- Torque phase similarity to the before checkpoint measures change, not biomechanical correctness or agreement with a V26 torque target.
- Analysis window is 13.15–17.95s on common STO support; early failures use a 150ms filter boundary margin and may lack complete cycles.
- Contact and reserve diagnostics use existing logged contact/threshold definitions, with translation forces in N and rotations in Nm kept separate.
- No target-dependent gates or new reward weights are introduced.
