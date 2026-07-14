# Reward Audit - compressive_stance

- Status: PASS
- Gate role: negative_gate
- Output: `/Users/tommy/Documents/CMC-like-Simulator - Claude/validation/reward_audit_runs/2026-07-10_fsm_phase_expectation_synthetic/compressive_stance`

## Criteria

- episode_return_negative: PASS
- no_to: PASS
- no_to_confirmation_credit: PASS
- no_valid_cycles: PASS
- provisional_credit_clawed_back: PASS
- terminal_contact_credit_zero: PASS

## Metrics

- contact_load_score_last: 0.0
- contact_load_score_mean: 0.4
- contact_support_clawback_max: 0.2
- contact_support_to_score_max: 0.0
- episode_return: -1.95
- landing_window_contact_score_mean: 0.0
- phase_event_progress_score_mean: 0.02
- phase_stance_timeout_loss_mean: 0.0
- phase_swing_timeout_loss_mean: 0.0
- phase_timeout_penalty_term_mean: 0.0
- reward_mean: -0.39
- reward_min: -2.25
- swing_unloading_loss_mean: 0.0
- valid_cycle_count_final: 0.0
- valid_hs_count_final: 1.0
- valid_to_count_final: 0.0

## Notes

- Synthetic H2 failure: monotonic load/penetration, no prosthetic TO, terminal penetration guard.
- Dense support credit must stop after minimum evidence and be clawed back at failure.

## Files

- `summary.json`
- `summary.md`
- `trace.csv`
- `online_events.csv`
