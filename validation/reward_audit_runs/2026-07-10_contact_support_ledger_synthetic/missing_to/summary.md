# Reward Audit - missing_to

- Status: PASS
- Gate role: negative_gate
- Output: `/Users/tommy/Documents/CMC-like-Simulator - Claude/validation/reward_audit_runs/2026-07-10_contact_support_ledger_synthetic/missing_to`

## Criteria

- has_initial_hs: PASS
- no_to: PASS
- no_valid_cycles: PASS
- reward_mean_le_0p15: PASS
- reward_min_negative: PASS
- stance_timeout_loss_positive: PASS
- timeout_penalty_positive: PASS

## Metrics

- contact_load_score_mean: 0.2
- landing_window_contact_score_mean: 0.0
- phase_event_progress_score_mean: 0.02
- phase_stance_timeout_loss_mean: 1.075
- phase_swing_timeout_loss_mean: 0.0
- phase_timeout_penalty_term_mean: 0.5375
- reward_mean: -0.5475
- reward_min: -2.0
- swing_unloading_loss_mean: 0.0
- valid_cycle_count_final: 0.0
- valid_hs_count_final: 1.0
- valid_to_count_final: 0.0

## Notes

- Synthetic missing-TO case keeps stance load but never emits toe_off, exercising soft and hard stance timeout.

## Files

- `summary.json`
- `summary.md`
- `trace.csv`
- `online_events.csv`
