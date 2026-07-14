# Reward Audit - missing_second_hs

- Status: PASS
- Gate role: negative_gate
- Output: `/Users/tommy/Documents/CMC-like-Simulator - Claude/validation/reward_audit_runs/2026-07-10_contact_support_final_synthetic/missing_second_hs`

## Criteria

- has_to: PASS
- landing_contact_low: PASS
- no_valid_cycles: PASS
- one_hs: PASS
- reward_mean_le_0p05: PASS
- reward_min_negative: PASS
- swing_timeout_loss_positive: PASS
- timeout_penalty_positive: PASS

## Metrics

- contact_load_score_mean: 0.14285714285714285
- landing_window_contact_score_mean: 0.0
- phase_event_progress_score_mean: 0.042857142857142864
- phase_stance_timeout_loss_mean: 0.0
- phase_swing_timeout_loss_mean: 0.25446428571428564
- phase_timeout_penalty_term_mean: 0.12723214285714282
- reward_mean: -0.09385021865889212
- reward_min: -1.0999999999999999
- swing_unloading_loss_mean: 0.0
- valid_cycle_count_final: 0.0
- valid_hs_count_final: 1.0
- valid_to_count_final: 1.0

## Notes

- Synthetic HS->TO sequence omits the second HS, so swing timeout and clawback must dominate.

## Files

- `summary.json`
- `summary.md`
- `trace.csv`
- `online_events.csv`
