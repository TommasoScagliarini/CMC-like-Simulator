# Reward Audit - prescribed_aligned

- Status: FAIL
- Gate role: positive_gate
- Output: `/Users/tommy/Documents/CMC-like-Simulator - Claude/validation/reward_audit_runs/2026-07-10_contact_support_ledger_positive`

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
- reward_mean_ge_0p25: FAIL
- reward_min_ge_minus_0p01: FAIL
- valid_cycles_ge_2: PASS
- valid_hs_ge_3: PASS
- valid_to_ge_2: PASS

## Metrics

- episode_return: 51.28744184118793
- grf_slip_loss_mean: 0.0
- invalid_event_count_final: 0.0
- last_period_s: 1.5480071478688746
- last_stance_fraction: 0.6796622091373897
- oob_term_mean: 0.0
- phase_timeout_loss_mean: 0.0
- phase_timeout_penalty_term_mean: 0.0
- prosthetic_joint_range_loss_mean: 0.0
- prosthetic_slip_speed_m_s_mean: 0.0
- returncode: 0.0
- reward_mean: 0.12663565886713068
- reward_min: -0.9957307884691262
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
