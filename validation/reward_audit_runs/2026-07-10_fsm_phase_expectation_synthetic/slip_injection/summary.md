# Reward Audit - slip_injection

- Status: PASS
- Gate role: diagnostic_gate
- Output: `/Users/tommy/Documents/CMC-like-Simulator - Claude/validation/reward_audit_runs/2026-07-10_fsm_phase_expectation_synthetic/slip_injection`

## Criteria

- current_reward_unchanged_without_slip: PASS
- current_slip_diagnostic_present: PASS
- current_slip_term_zero: PASS
- enabled_reward_drop_ge_0p10: PASS
- enabled_slip_term_positive: PASS

## Metrics

- current_grf_slip_loss_mean: 0.25
- current_grf_slip_term_mean: 0.0
- current_reward_mean: 0.0
- current_reward_without_grf_slip_mean: 0.0
- current_trace_rows: 45.0
- enabled_grf_slip_term_mean: 0.25
- enabled_reward_mean: -0.25
- enabled_trace_rows: 45.0
- reward_drop_enabled_vs_current: 0.25

## Notes

- C6a uses the current config with grf_slip_weight=0.0.
- C6b uses an in-memory diagnostic config with grf_slip_weight=1.0.

## Files

- `summary.json`
- `summary.md`
- `trace.csv`
- `online_events.csv`
