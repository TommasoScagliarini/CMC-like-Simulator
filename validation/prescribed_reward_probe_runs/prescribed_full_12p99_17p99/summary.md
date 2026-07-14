# Prescribed Reward Probe

- Window: 12.990 -> 17.990 s
- Steps: 501
- Return: 25.3309
- Reward mean: 0.0505608
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

- reward_base: 0.177564
- reward_without_grf_slip: 0.0505608
- grf_slip_term: 0
- grf_slip_loss: 0
- prosthetic_slip_speed_m_s: 0
- contact_load_score: 0.507427
- landing_window_active: 0.0578842
- landing_window_contact_score: 0
- phase_regular_score: 0.0342374
- phase_timeout_loss: 0.17243
- policy_action_clip_loss: 0
- prosthetic_joint_range_loss: 0
- oob_term: 0

## Files

- `summary.json`
- `trace.csv`
- `online_events.csv`
