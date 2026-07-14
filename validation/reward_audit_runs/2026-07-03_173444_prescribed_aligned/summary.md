# Reward Audit - prescribed_aligned

- Status: PASS
- Gate role: positive_gate
- Output: `/Users/tommy/Documents/CMC-like-Simulator - Claude/validation/reward_audit_runs/2026-07-03_173444_prescribed_aligned`

## Criteria

- episode_return_positive: PASS
- grf_slip_zero: PASS
- invalid_events_zero: PASS
- joint_range_loss_zero: PASS
- oob_term_zero: PASS
- phase_timeout_term_zero: PASS
- phase_timeout_zero: PASS
- probe_ok: PASS
- process_ok: PASS
- prosthetic_slip_zero: PASS
- reward_mean_ge_0p25: PASS
- reward_min_ge_minus_0p01: PASS
- valid_cycles_ge_2: PASS
- valid_hs_ge_3: PASS
- valid_to_ge_2: PASS

## Metrics

- episode_return: 136.2890648646879
- grf_slip_loss_mean: 0.0
- invalid_event_count_final: 0.0
- oob_term_mean: 0.0
- phase_timeout_loss_mean: 0.0
- phase_timeout_penalty_term_mean: 0.0
- prosthetic_joint_range_loss_mean: 0.0
- prosthetic_slip_speed_m_s_mean: 0.0
- returncode: 0.0
- reward_mean: 0.33651620954243927
- reward_min: -0.00010982029936525015
- valid_cycle_count_final: 2.0
- valid_hs_count_final: 3.0
- valid_to_count_final: 2.0

## Notes

- Executed via validation/prescribed_reward_probe.py.
- stdout: probe_stdout.txt; stderr: probe_stderr.txt

## Files

- `summary.json`
- `summary.md`
- `trace.csv`
- `online_events.csv`
