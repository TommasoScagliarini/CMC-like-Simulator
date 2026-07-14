# Reward Audit - static_leg

- Status: PASS
- Gate role: negative_gate
- Output: `/Users/tommy/Documents/CMC-like-Simulator - Claude/validation/reward_audit_runs/2026-07-10_contact_support_ledger_synthetic/static_leg`

## Criteria

- contact_load_not_compensating: PASS
- no_valid_cycles: PASS
- phase_event_progress_low: PASS
- reward_mean_le_0p10: PASS
- timeout_penalty_positive: PASS

## Metrics

- contact_load_score_mean: 0.0
- landing_window_contact_score_mean: 0.0
- phase_event_progress_score_mean: 0.025
- phase_stance_timeout_loss_mean: 0.90625
- phase_swing_timeout_loss_mean: 0.0
- phase_timeout_penalty_term_mean: 0.453125
- reward_mean: -0.465625
- reward_min: -1.9
- swing_unloading_loss_mean: 0.0
- valid_cycle_count_final: 0.0
- valid_hs_count_final: 1.0
- valid_to_count_final: 0.0

## Notes

- Synthetic static prosthesis: initial HS only, no contact/load progression and no TO.

## Files

- `summary.json`
- `summary.md`
- `trace.csv`
- `online_events.csv`
