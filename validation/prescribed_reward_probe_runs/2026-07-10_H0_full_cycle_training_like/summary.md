# Prescribed Reward Probe

- Window: 13.947 -> 15.800 s
- Steps: 186
- Return: 4.9235
- Reward mean: 0.0264704
- End: terminated=False, truncated=True, reason=episode_time_limit

## Dynamics Contract

- Contract: training_like
- Reward input source: env_online_payload
- GRF mode: online_sensor
- Online applied sides: ['left']
- Prescribed disabled sides: ['left']

## FSM

- Final state: STANCE_AFTER_HS (1.0)
- Valid HS: 2.0
- Valid TO: 1.0
- Valid cycles: 1.0
- Last period: 1.4659999999991875 s
- Last stance fraction: 0.8178717598908595
- Timeout: 0.0 side=0.0

## Key Means

- reward_base: 0.0560856
- reward_without_grf_slip: 0.0264704
- grf_slip_term: 0
- grf_slip_loss: 19.8719
- prosthetic_slip_speed_m_s: 2.73621
- contact_load_score: 0.0975092
- landing_window_active: 0
- landing_window_contact_score: 0
- phase_regular_score: 0.181309
- phase_timeout_loss: 0
- policy_action_clip_loss: 0
- prosthetic_joint_range_loss: 0
- oob_term: 0

## Files

- `summary.json`
- `trace.csv`
- `online_events.csv`
