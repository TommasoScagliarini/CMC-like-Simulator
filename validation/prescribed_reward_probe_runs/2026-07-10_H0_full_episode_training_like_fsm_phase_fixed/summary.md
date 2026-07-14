# Prescribed Reward Probe

- Window: 13.947 -> 18.947 s
- Steps: 373
- Return: 42.258
- Reward mean: 0.113292
- End: terminated=True, truncated=False, reason=grf_penetration

## Dynamics Contract

- Contract: training_like
- Reward input source: env_online_payload
- GRF mode: online_sensor
- Online applied sides: ['left']
- Prescribed disabled sides: ['left']

## FSM

- Final state: STANCE_AFTER_HS (1.0)
- Valid HS: 3.0
- Valid TO: 2.0
- Valid cycles: 2.0
- Last period: 1.550999999999835 s
- Last stance fraction: 0.8368794326240686
- Timeout: 0.0 side=0.0

## Key Means

- reward_base: 0.135269
- reward_without_grf_slip: 0.113292
- grf_slip_term: 0
- grf_slip_loss: 20.6264
- prosthetic_slip_speed_m_s: 2.73409
- contact_load_score: 0.139536
- landing_window_active: 0
- landing_window_contact_score: 0
- phase_regular_score: 0.552358
- phase_timeout_loss: 0
- policy_action_clip_loss: 0
- prosthetic_joint_range_loss: 0
- oob_term: 0

## Files

- `summary.json`
- `trace.csv`
- `online_events.csv`
