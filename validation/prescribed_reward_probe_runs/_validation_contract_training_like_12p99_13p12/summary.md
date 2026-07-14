# Prescribed Reward Probe

- Window: 12.990 -> 13.120 s
- Steps: 13
- Return: -3.51067
- Reward mean: -0.270051
- End: terminated=False, truncated=True, reason=episode_time_limit

## Dynamics Contract

- Contract: training_like
- GRF mode: online_sensor
- Online applied sides: ['left']
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

- reward_base: 0.271907
- reward_without_grf_slip: 0.229949
- grf_slip_term: 0.5
- grf_slip_loss: 25
- prosthetic_slip_speed_m_s: 2.62017
- contact_load_score: 0.774619
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
