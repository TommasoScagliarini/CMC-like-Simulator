# Prescribed Reward Probe

- Window: 13.947 -> 15.300 s
- Steps: 32
- Return: -1.19312
- Reward mean: -0.0372848
- End: terminated=True, truncated=False, reason=grf_penetration

## Dynamics Contract

- Contract: training_like
- Reward input source: env_online_payload
- GRF mode: online_sensor
- Online applied sides: ['left']
- Prescribed disabled sides: ['left']

## FSM

- Final state: STANCE_AFTER_HS (1.0)
- Valid HS: 1.0
- Valid TO: 0.0
- Valid cycles: 0.0
- Last period: 0.0 s
- Last stance fraction: 0.0
- Timeout: 0.0 side=0.0

## Key Means

- reward_base: 0.088892
- reward_without_grf_slip: -0.0372848
- grf_slip_term: 0
- grf_slip_loss: 25
- prosthetic_slip_speed_m_s: 2.6572
- contact_load_score: 0.254159
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
