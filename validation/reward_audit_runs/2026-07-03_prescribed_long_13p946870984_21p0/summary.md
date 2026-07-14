# Reward Audit - prescribed_long

- Status: PASS
- Gate role: positive_long_gate
- Output: `/Users/tommy/Documents/CMC-like-Simulator - Claude/validation/reward_audit_runs/2026-07-03_prescribed_long_13p946870984_21p0`

## Criteria

- episode_return_positive: PASS
- grf_slip_zero: PASS
- invalid_events_zero: PASS
- last_period_in_hard_band: PASS
- last_stance_fraction_in_band: PASS
- phase_timeout_zero: PASS
- probe_ok: PASS
- process_ok: PASS
- reward_mean_ge_0p20: PASS
- valid_cycles_ge_4: PASS

## Metrics

- episode_return: 252.81650162961316
- grf_slip_loss_mean: 0.0
- invalid_event_count_final: 0.0
- last_period_s: 1.5709874864018296
- last_stance_fraction: 0.6751950880539738
- oob_term_mean: 0.0
- phase_timeout_loss_mean: 0.0
- phase_timeout_penalty_term_mean: 0.0
- prosthetic_joint_range_loss_mean: 0.0
- prosthetic_slip_speed_m_s_mean: 0.0
- returncode: 0.0
- reward_mean: 0.35809702780398467
- reward_min: -0.0034142096773359847
- valid_cycle_count_final: 4.0
- valid_hs_count_final: 5.0
- valid_to_count_final: 4.0

## Notes

- Executed via validation/prescribed_reward_probe.py.
- stdout: probe_stdout.txt; stderr: probe_stderr.txt

## Files

- `summary.json`
- `summary.md`
- `trace.csv`
- `online_events.csv`
