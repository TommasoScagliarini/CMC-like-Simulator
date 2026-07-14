# Reward Audit - morphology_corridor

- Status: PASS
- Gate role: diagnostic_gate
- Output: `/Users/tommy/Documents/CMC-like-Simulator - Claude/validation/reward_audit_runs/2026-07-03_synthetic_negative_matrix/morphology_corridor`

## Criteria

- current_loss_finite: PASS
- current_term_zero: PASS
- enabled_high_reward_lower: PASS
- enabled_low_loss_below_high: PASS

## Metrics

- current_morphology_loss_mean: 0.2
- current_morphology_term_mean: 0.0
- enabled_high_morphology_loss_mean: 0.5
- enabled_high_reward_mean: -0.25
- enabled_low_morphology_loss_mean: 0.01
- enabled_low_reward_mean: -0.005

## Notes

- Synthetic diagnostic: verifies morphology_weight=0 is non-penalizing and a temporary positive weight is active.
- This does not replace a future prescribed/OpenSim corridor replay with perturbed kinematics.

## Files

- `summary.json`
- `summary.md`
- `trace.csv`
- `online_events.csv`
