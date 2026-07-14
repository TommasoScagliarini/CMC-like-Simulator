# Reward Separation Matrix - 2026-07-03

Baseline positive gate:

- `prescribed_aligned` reward_mean: 0.33651620954243927
- 70 percent threshold: 0.2355613466797075

## Result

Status: PASS

All negative counterexamples are below 70 percent of `prescribed_aligned`.
The positive prescribed windows remain above their gates and close valid cycles.

## Matrix

| Scenario | Status | Reward Mean | Valid Cycles | Note |
| --- | --- | ---: | ---: | --- |
| prescribed_aligned | PASS | 0.336516 | 2 | positive reference |
| prescribed_long | PASS | 0.358097 | 4 | positive multi-cycle reference |
| static_leg | PASS | -0.465625 | 0 | negative gate |
| missing_to | PASS | -0.267500 | 0 | negative gate |
| missing_second_hs | PASS | -0.034375 | 0 | negative gate |
| swing_load | PASS | 0.153074 | 0 | negative gate |
| joint_oob | PASS | -2.101003 | 0 | negative gate |
| fake_cycle_ankle_only | PASS | 0.100000 | 0 | no cycle bonus, reject present |
| slip_injection_current | PASS | 0.000000 | 0 | current slip weight zero |
| slip_injection_enabled | PASS | -0.250000 | 0 | temporary slip weight penalizes |
| morphology_current | PASS | 0.000000 | 0 | current morphology weight zero |
| morphology_enabled_high_loss | PASS | -0.250000 | 0 | temporary morphology weight penalizes |

## Source Outputs

- `validation/reward_audit_runs/2026-07-03_173444_prescribed_aligned/`
- `validation/reward_audit_runs/2026-07-03_prescribed_long_13p946870984_21p0/`
- `validation/reward_audit_runs/2026-07-03_synthetic_negative_matrix/`

## Caveat

`morphology_corridor` is an algebraic diagnostic of reward weighting in this
matrix. A future prescribed/OpenSim replay with perturbed kinematics should
remain a separate morphology-specific validation before increasing
`morphology_weight` in training.
