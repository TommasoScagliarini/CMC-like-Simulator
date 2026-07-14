# Prescribed Reward Probe

- Window: 13.947 -> 17.990 s
- Steps: 405
- Return: 136.289
- Reward mean: 0.336516
- End: terminated=False, truncated=True, reason=episode_time_limit

## Dynamics Contract

- Contract: prescribed_pure
- Reward input source: prescribed_grf
- GRF mode: online_sensor
- Online applied sides: []
- Prescribed disabled sides: []

## FSM

- Final state: STANCE_AFTER_HS (1.0)
- Valid HS: 3.0
- Valid TO: 2.0
- Valid cycles: 2.0
- Last period: 1.5480071478688746 s
- Last stance fraction: 0.6796622091373897
- Timeout: 0.0 side=0.0

## Key Means

- reward_base: 0.336525
- reward_without_grf_slip: 0.336516
- grf_slip_term: 0
- grf_slip_loss: 0
- prosthetic_slip_speed_m_s: 0
- contact_load_score: 0.635579
- landing_window_active: 0.0691358
- landing_window_contact_score: 0
- phase_regular_score: 0.534493
- phase_timeout_loss: 0
- policy_action_clip_loss: 0
- prosthetic_joint_range_loss: 0
- oob_term: 0

## Files

- `summary.json`
- `trace.csv`
- `online_events.csv`
