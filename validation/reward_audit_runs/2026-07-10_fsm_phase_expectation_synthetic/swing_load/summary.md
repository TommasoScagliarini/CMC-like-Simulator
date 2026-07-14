# Reward Audit - swing_load

- Status: PASS
- Gate role: negative_gate
- Output: `/Users/tommy/Documents/CMC-like-Simulator - Claude/validation/reward_audit_runs/2026-07-10_fsm_phase_expectation_synthetic/swing_load`

## Criteria

- contact_load_not_compensating: PASS
- reward_below_prescribed_margin: PASS
- swing_unloading_loss_positive: PASS

## Metrics

- contact_load_score_mean: 0.16666666666666666
- landing_window_contact_score_mean: 0.0
- phase_event_progress_score_mean: 0.05000000000000001
- phase_stance_timeout_loss_mean: 0.0
- phase_swing_timeout_loss_mean: 0.0
- phase_timeout_penalty_term_mean: 0.0
- reward_mean: -0.06947005962878978
- reward_min: -0.5002469135802469
- swing_unloading_loss_mean: 8.333333333333334
- valid_cycle_count_final: 0.0
- valid_hs_count_final: 1.0
- valid_to_count_final: 1.0

## Notes

- Synthetic swing-load case injects vertical force shortly after TO, before the landing window should reward contact.

## Files

- `summary.json`
- `summary.md`
- `trace.csv`
- `online_events.csv`
