# Reward Audit - joint_oob

- Status: PASS
- Gate role: negative_gate
- Output: `/Users/tommy/Documents/CMC-like-Simulator - Claude/validation/reward_audit_runs/2026-07-10_fsm_phase_expectation_synthetic/joint_oob`

## Criteria

- joint_range_loss_positive: PASS
- oob_loss_positive: PASS
- reward_below_prescribed_margin: PASS

## Metrics

- contact_load_score_mean: 0.0
- landing_window_contact_score_mean: 0.0
- oob_loss_mean: 0.05050138888888889
- oob_term_mean: 0.10100277777777777
- phase_event_progress_score_mean: 0.0
- phase_stance_timeout_loss_mean: 0.0
- phase_swing_timeout_loss_mean: 0.0
- phase_timeout_penalty_term_mean: 0.0
- prosthetic_joint_range_loss_mean: 1.0
- reward_mean: -2.1010027777777776
- reward_min: -2.1010027777777776
- swing_unloading_loss_mean: 0.0
- valid_cycle_count_final: 0.0
- valid_hs_count_final: 0.0
- valid_to_count_final: 0.0

## Notes

- Synthetic joint-OOB case injects actual joint range loss and out-of-band commanded reference.

## Files

- `summary.json`
- `summary.md`
- `trace.csv`
- `online_events.csv`
