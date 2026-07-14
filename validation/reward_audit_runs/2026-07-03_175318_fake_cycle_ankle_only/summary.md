# Reward Audit - fake_cycle_ankle_only

- Status: PASS
- Gate role: negative_gate
- Output: `/Users/tommy/Documents/CMC-like-Simulator - Claude/validation/reward_audit_runs/2026-07-03_175318_fake_cycle_ankle_only`

## Criteria

- ankle_oscillation_present: PASS
- knee_excursion_below_gate: PASS
- loaded_static_knee_rejected: PASS
- no_cycle_complete_bonus: PASS
- no_load_rejects_toe_off: PASS
- no_valid_cycles: PASS
- reward_mean_low: PASS

## Metrics

- cycle_complete_bonus_max: 0.0
- cycle_rejected_max: 1.0
- loaded_ankle_excursion_max_rad: 0.55
- loaded_final_valid_cycles: 0.0
- loaded_knee_excursion_max_rad: 0.015000000000000013
- phase_min_cycle_knee_excursion_rad: 0.12
- phase_min_stance_contact_fraction: 0.2
- phase_min_stance_load_bw_s: 0.04
- reward_mean: 0.09999999999999999

## Notes

- Synthetic HS-TO-HS sequences oscillate the ankle while keeping knee excursion below the configured gate.
- The loaded subcase has stance contact/load evidence, so the rejection must come from the knee-excursion gate.

## Files

- `summary.json`
- `summary.md`
- `trace.csv`
- `online_events.csv`
