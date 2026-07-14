# Prescribed Reward Probe

- Window: 12.990 -> 13.050 s
- Steps: 6
- Return: 2.15833
- Reward mean: 0.359722
- End: terminated=False, truncated=True, reason=episode_time_limit

## Dynamics Contract

- Contract: prescribed_pure
- Reward input source: prescribed_grf
- GRF mode: online_sensor
- Online applied sides: []
- Prescribed disabled sides: []

## FSM

- Final state: STANCE_AFTER_HS (1.0)
- Valid HS: 1.0
- Valid TO: 0.0
- Valid cycles: 0.0
- Last period: 0.0 s
- Last stance fraction: 0.0
- Timeout: 0.0 side=0.0

## Key Means

- reward_base: 0.359722
- reward_without_grf_slip: 0.359722
- grf_slip_term: 0
- grf_slip_loss: 0
- prosthetic_slip_speed_m_s: 0
- contact_load_score: 1
- landing_window_active: 0
- landing_window_contact_score: 0
- phase_regular_score: 0
- phase_timeout_loss: 0
- policy_action_clip_loss: 0
- prosthetic_joint_range_loss: 0
- oob_term: 0

## Files

- `summary.json`
- `trace.csv`
- `online_events.csv`
