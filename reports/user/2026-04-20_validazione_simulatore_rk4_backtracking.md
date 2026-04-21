# Validazione simulatore - 2026-04-20 18:14

Risultato complessivo: **FAIL**

- Results dir: `/Users/tommy/Documents/CMC-like-Simulator - Claude/results/_codex_full_rk4_backtracking_100_20`
- Reference: `/Users/tommy/Documents/CMC-like-Simulator - Claude/data/3DGaitModel2392_Kinematics_q.sto`

## Checks

| Status | Check | Detail |
|---|---|---|
| PASS | run status | simulation complete at t=11.061 |
| PASS | SEA_Knee tau_ref - tau_spring | non-trivial torque tracking error; max_abs=2.726937e+02 Nm, rms=7.072581e+01 Nm, mean_abs=5.246215e+01 Nm |
| PASS | SEA_Knee algebraic motor state | motor_angle is not algebraically constrained; max_abs=2.726937e-01 rad, rms=7.072581e-02 rad, mean_abs=5.246215e-02 rad |
| PASS | SEA_Knee motor_speed - qdot | motor_speed differs from joint speed; max_abs=7.964515e+01 rad/s, rms=2.265887e+01 rad/s, mean_abs=1.593078e+01 rad/s |
| PASS | SEA_Ankle tau_ref - tau_spring | non-trivial torque tracking error; max_abs=6.585603e+02 Nm, rms=2.557928e+02 Nm, mean_abs=1.871290e+02 Nm |
| PASS | SEA_Ankle algebraic motor state | motor_angle is not algebraically constrained; max_abs=1.317121e+00 rad, rms=5.115856e-01 rad, mean_abs=3.742581e-01 rad |
| PASS | SEA_Ankle motor_speed - qdot | motor_speed differs from joint speed; max_abs=4.468855e+02 rad/s, rms=1.920291e+02 rad/s, mean_abs=1.543450e+02 rad/s |
| PASS | SEA_Knee derivatives | finite plugin derivatives; speed_dot max_abs=6.686766e+04 rad/s^2, rms=1.970604e+04 rad/s^2, mean_abs=1.193826e+04 rad/s^2 |
| PASS | SEA_Ankle derivatives | finite plugin derivatives; speed_dot max_abs=9.591224e+04 rad/s^2, rms=5.738994e+04 rad/s^2, mean_abs=5.028801e+04 rad/s^2 |
| PASS | SEA_Knee plugin/Python tau_input agreement | plugin output matches independently recomputed SEA law; max_abs=5.000000e-06 Nm, rms=1.844896e-06 Nm, mean_abs=1.133183e-06 Nm |
| PASS | SEA_Knee tau_error diagnostic | finite tau_ref - tau_spring; max_abs=2.726937e+02 Nm, rms=7.072581e+01 Nm, mean_abs=5.246215e+01 Nm |
| PASS | SEA_Knee motor speed diagnostics | motor_speed max_abs=1.282023e+02 rad/s, rms=2.464957e+01 rad/s, mean_abs=1.916341e+01 rad/s; speed_dot max_abs=6.686766e+04 rad/s^2, rms=1.970604e+04 rad/s^2, mean_abs=1.193826e+04 rad/s^2 |
| WARN | SEA_Knee tau_input saturation | tau_input reaches +/-500 Nm in 429 samples (6.308%); tuning issue, not an interface failure |
| PASS | SEA_Knee saturation source terms | sat median \|tau_ff\|=72.812 Nm, \|outer_PD\|=84.264 Nm, \|inner_prop\|=568.210 Nm, \|inner_damp\|=160.221 Nm; raw excess max_abs=5.122163e+02 Nm, rms=5.093295e+01 Nm, mean_abs=1.095323e+01 Nm; motor numerator max_abs=6.686766e+02 Nm, rms=1.970604e+02 Nm, mean_abs=1.193826e+02 Nm |
| PASS | SEA_Ankle plugin/Python tau_input agreement | plugin output matches independently recomputed SEA law; max_abs=5.000000e-06 Nm, rms=1.617395e-06 Nm, mean_abs=7.994663e-07 Nm |
| PASS | SEA_Ankle tau_error diagnostic | finite tau_ref - tau_spring; max_abs=6.585603e+02 Nm, rms=2.557928e+02 Nm, mean_abs=1.871290e+02 Nm |
| PASS | SEA_Ankle motor speed diagnostics | motor_speed max_abs=3.485586e+02 rad/s, rms=1.057012e+02 rad/s, mean_abs=8.628821e+01 rad/s; speed_dot max_abs=9.591224e+04 rad/s^2, rms=5.738994e+04 rad/s^2, mean_abs=5.028801e+04 rad/s^2 |
| WARN | SEA_Ankle tau_input saturation | tau_input reaches +/-500 Nm in 4314 samples (63.432%); tuning issue, not an interface failure |
| PASS | SEA_Ankle saturation source terms | sat median \|tau_ff\|=6.782 Nm, \|outer_PD\|=2069.157 Nm, \|inner_prop\|=2216.452 Nm, \|inner_damp\|=830.935 Nm; raw excess max_abs=5.037179e+03 Nm, rms=1.578532e+03 Nm, mean_abs=9.192944e+02 Nm; motor numerator max_abs=9.591224e+02 Nm, rms=5.738994e+02 Nm, mean_abs=5.028801e+02 Nm |
| WARN | pros_knee_angle reserve torque | prosthetic reserve is exactly zero; OK only if SEA path is validated; max_abs=0.000000e+00 Nm, rms=0.000000e+00 Nm, mean_abs=0.000000e+00 Nm |
| WARN | pros_ankle_angle reserve torque | prosthetic reserve is exactly zero; OK only if SEA path is validated; max_abs=0.000000e+00 Nm, rms=0.000000e+00 Nm, mean_abs=0.000000e+00 Nm |
| FAIL | pros_knee_angle output vs IK | tracking error exceeds stability threshold; max_abs=7.941722e+02 deg, rms=2.462593e+02 deg, mean_abs=1.378502e+02 deg |
| FAIL | pros_ankle_angle output vs IK | tracking error exceeds stability threshold; max_abs=1.150707e+03 deg, rms=3.882556e+02 deg, mean_abs=3.059134e+02 deg |
| FAIL | pelvis_tx output vs IK | tracking error exceeds stability threshold; max_abs=1.589472e-01 m, rms=6.768871e-02 m, mean_abs=5.309759e-02 m |
| FAIL | pelvis_ty output vs IK | tracking error exceeds stability threshold; max_abs=1.986465e-01 m, rms=8.434553e-02 m, mean_abs=7.011104e-02 m |
| FAIL | pelvis_tz output vs IK | tracking error exceeds stability threshold; max_abs=2.342125e-01 m, rms=1.098204e-01 m, mean_abs=9.000148e-02 m |
| FAIL | pelvis_tilt output vs IK | tracking error exceeds stability threshold; max_abs=6.240417e+01 deg, rms=1.922009e+01 deg, mean_abs=1.487580e+01 deg |
| FAIL | pelvis_list output vs IK | tracking error exceeds stability threshold; max_abs=3.261830e+01 deg, rms=1.321051e+01 deg, mean_abs=1.000199e+01 deg |
| FAIL | pelvis_rotation output vs IK | tracking error exceeds stability threshold; max_abs=9.357068e+01 deg, rms=2.798767e+01 deg, mean_abs=2.081067e+01 deg |
| FAIL | hip_flexion_r output vs IK | tracking error exceeds stability threshold; max_abs=2.994180e+02 deg, rms=5.717547e+01 deg, mean_abs=2.836523e+01 deg |
| FAIL | hip_adduction_r output vs IK | tracking error exceeds stability threshold; max_abs=8.369677e+01 deg, rms=2.198263e+01 deg, mean_abs=1.582698e+01 deg |
| FAIL | hip_rotation_r output vs IK | tracking error exceeds stability threshold; max_abs=2.301602e+02 deg, rms=4.527940e+01 deg, mean_abs=2.572320e+01 deg |
| FAIL | knee_angle_r output vs IK | tracking error exceeds stability threshold; max_abs=3.031648e+01 deg, rms=7.093415e+00 deg, mean_abs=4.815061e+00 deg |
| FAIL | ankle_angle_r output vs IK | tracking error exceeds stability threshold; max_abs=2.250388e+02 deg, rms=4.809150e+01 deg, mean_abs=2.844091e+01 deg |
| FAIL | subtalar_angle_r output vs IK | tracking error exceeds stability threshold; max_abs=1.496343e+02 deg, rms=4.495068e+01 deg, mean_abs=3.052470e+01 deg |
| FAIL | mtp_angle_r output vs IK | tracking error exceeds stability threshold; max_abs=2.766155e+02 deg, rms=8.169666e+01 deg, mean_abs=5.746670e+01 deg |
| FAIL | hip_flexion_l output vs IK | tracking error exceeds stability threshold; max_abs=1.929065e+02 deg, rms=6.688158e+01 deg, mean_abs=5.329622e+01 deg |
| FAIL | hip_adduction_l output vs IK | tracking error exceeds stability threshold; max_abs=6.631732e+01 deg, rms=2.206601e+01 deg, mean_abs=1.770234e+01 deg |
| FAIL | hip_rotation_l output vs IK | tracking error exceeds stability threshold; max_abs=1.251124e+02 deg, rms=2.888016e+01 deg, mean_abs=1.847128e+01 deg |
| FAIL | lumbar_extension output vs IK | tracking error exceeds stability threshold; max_abs=5.794946e+01 deg, rms=1.842593e+01 deg, mean_abs=1.401991e+01 deg |
| FAIL | lumbar_bending output vs IK | tracking error exceeds stability threshold; max_abs=7.724023e+01 deg, rms=2.190389e+01 deg, mean_abs=1.638403e+01 deg |
| FAIL | lumbar_rotation output vs IK | tracking error exceeds stability threshold; max_abs=5.563967e+01 deg, rms=2.037371e+01 deg, mean_abs=1.612286e+01 deg |
| WARN | SEA_Knee control saturation | SEA control reaches saturation; max \|u\|=1.000000 |
| PASS | SEA_Knee motor vs joint power | motor-joint diff max_abs=3.745772e+04 W, rms=4.086569e+03 W, mean_abs=2.187764e+03 W, corr=0.426899 |
| WARN | SEA_Ankle control saturation | SEA control reaches saturation; max \|u\|=1.000000 |
| PASS | SEA_Ankle motor vs joint power | motor-joint diff max_abs=1.728787e+05 W, rms=4.183371e+04 W, mean_abs=3.133809e+04 W, corr=0.475820 |

## Interpretazione

Almeno un controllo critico indica che il risultato non e validabile.
Un FAIL su `run status` indica output parziali; un FAIL su tracking
indica divergenza dinamica; un FAIL sulle metriche SEA indica possibile
tautologia o derivate plugin non finite.
