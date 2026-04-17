# Validazione simulatore - 2026-04-17 06:48

Risultato complessivo: **FAIL**

- Results dir: `C:\Users\tomma\Desktop\Opensim OMNIBUS\CMC-like-Simulator - Claude\results\_sea_parameter_sweep_20260416_205113\runs\full_k250_a1000`
- Reference: `C:\Users\tomma\Desktop\Opensim OMNIBUS\CMC-like-Simulator - Claude\data\3DGaitModel2392_Kinematics_q.sto`

## Checks

| Status | Check | Detail |
|---|---|---|
| PASS | run status | simulation complete at t=11.06 |
| PASS | SEA_Knee tau_ref - tau_spring | non-trivial torque tracking error; max_abs=3.564767e+01 Nm, rms=1.386683e+00 Nm, mean_abs=9.684441e-01 Nm |
| PASS | SEA_Knee algebraic motor state | motor_angle is not algebraically constrained; max_abs=1.425907e-01 rad, rms=5.546731e-03 rad, mean_abs=3.873777e-03 rad |
| PASS | SEA_Knee motor_speed - qdot | motor_speed differs from joint speed; max_abs=2.283441e+01 rad/s, rms=8.569848e-01 rad/s, mean_abs=4.819622e-01 rad/s |
| PASS | SEA_Ankle tau_ref - tau_spring | non-trivial torque tracking error; max_abs=2.528676e+01 Nm, rms=1.439818e+01 Nm, mean_abs=1.325981e+01 Nm |
| PASS | SEA_Ankle algebraic motor state | motor_angle is not algebraically constrained; max_abs=2.528677e-02 rad, rms=1.439818e-02 rad, mean_abs=1.325981e-02 rad |
| PASS | SEA_Ankle motor_speed - qdot | motor_speed differs from joint speed; max_abs=5.329689e+00 rad/s, rms=3.271613e-01 rad/s, mean_abs=1.891799e-01 rad/s |
| PASS | SEA_Knee derivatives | finite plugin derivatives; speed_dot max_abs=2.977575e+04 rad/s^2, rms=6.177343e+02 rad/s^2, mean_abs=9.766199e+01 rad/s^2 |
| PASS | SEA_Ankle derivatives | finite plugin derivatives; speed_dot max_abs=1.420479e+03 rad/s^2, rms=5.237881e+01 rad/s^2, mean_abs=1.763931e+01 rad/s^2 |
| PASS | SEA_Knee plugin/Python tau_input agreement | plugin output matches independently recomputed SEA law; max_abs=4.560000e-06 Nm, rms=1.689363e-07 Nm, mean_abs=8.419559e-08 Nm |
| PASS | SEA_Knee tau_error diagnostic | finite tau_ref - tau_spring; max_abs=3.564767e+01 Nm, rms=1.386683e+00 Nm, mean_abs=9.684441e-01 Nm |
| PASS | SEA_Knee motor speed diagnostics | motor_speed max_abs=2.723698e+01 rad/s, rms=2.017332e+00 rad/s, mean_abs=1.431172e+00 rad/s; speed_dot max_abs=2.977575e+04 rad/s^2, rms=6.177343e+02 rad/s^2, mean_abs=9.766199e+01 rad/s^2 |
| PASS | SEA_Knee tau_input saturation | tau_input never reaches the +/-500 Nm clamp |
| PASS | SEA_Knee saturation source terms | raw command below clamp; raw max_abs=2.809141e+02 Nm, rms=1.072014e+01 Nm, mean_abs=7.279204e+00 Nm; motor numerator max_abs=2.977575e+02 Nm, rms=6.177343e+00 Nm, mean_abs=9.766199e-01 Nm |
| PASS | SEA_Ankle plugin/Python tau_input agreement | plugin output matches independently recomputed SEA law; max_abs=4.900000e-06 Nm, rms=4.673571e-07 Nm, mean_abs=1.641162e-07 Nm |
| PASS | SEA_Ankle tau_error diagnostic | finite tau_ref - tau_spring; max_abs=2.528676e+01 Nm, rms=1.439818e+01 Nm, mean_abs=1.325981e+01 Nm |
| PASS | SEA_Ankle motor speed diagnostics | motor_speed max_abs=1.189086e+01 rad/s, rms=4.094134e+00 rad/s, mean_abs=3.274084e+00 rad/s; speed_dot max_abs=1.420479e+03 rad/s^2, rms=5.237881e+01 rad/s^2, mean_abs=1.763931e+01 rad/s^2 |
| PASS | SEA_Ankle tau_input saturation | tau_input never reaches the +/-500 Nm clamp |
| PASS | SEA_Ankle saturation source terms | raw command below clamp; raw max_abs=1.146137e+02 Nm, rms=3.812928e+01 Nm, mean_abs=2.258984e+01 Nm; motor numerator max_abs=1.420479e+01 Nm, rms=5.237881e-01 Nm, mean_abs=1.763931e-01 Nm |
| WARN | pros_knee_angle reserve torque | prosthetic reserve is exactly zero; OK only if SEA path is validated; max_abs=0.000000e+00 Nm, rms=0.000000e+00 Nm, mean_abs=0.000000e+00 Nm |
| WARN | pros_ankle_angle reserve torque | prosthetic reserve is exactly zero; OK only if SEA path is validated; max_abs=0.000000e+00 Nm, rms=0.000000e+00 Nm, mean_abs=0.000000e+00 Nm |
| FAIL | pros_knee_angle output vs IK | tracking error exceeds stability threshold; max_abs=2.405067e+01 deg, rms=1.067592e+01 deg, mean_abs=9.041215e+00 deg |
| FAIL | pros_ankle_angle output vs IK | tracking error exceeds stability threshold; max_abs=2.721308e+02 deg, rms=1.631055e+02 deg, mean_abs=1.498686e+02 deg |
| PASS | pelvis_tx output vs IK | non-zero tracking error; max_abs=3.187590e-03 m, rms=1.311941e-03 m, mean_abs=9.721119e-04 m |
| PASS | pelvis_ty output vs IK | non-zero tracking error; max_abs=4.762450e-03 m, rms=1.897818e-03 m, mean_abs=1.390072e-03 m |
| PASS | pelvis_tz output vs IK | non-zero tracking error; max_abs=1.208365e-02 m, rms=3.104287e-03 m, mean_abs=1.951093e-03 m |
| PASS | pelvis_tilt output vs IK | non-zero tracking error; max_abs=1.124116e+00 deg, rms=4.146356e-01 deg, mean_abs=2.977361e-01 deg |
| PASS | pelvis_list output vs IK | non-zero tracking error; max_abs=9.037651e-01 deg, rms=2.792060e-01 deg, mean_abs=1.843605e-01 deg |
| PASS | pelvis_rotation output vs IK | non-zero tracking error; max_abs=1.459879e+00 deg, rms=5.410764e-01 deg, mean_abs=3.912669e-01 deg |
| PASS | hip_flexion_r output vs IK | non-zero tracking error; max_abs=1.189857e+00 deg, rms=4.498110e-01 deg, mean_abs=3.287813e-01 deg |
| PASS | hip_adduction_r output vs IK | non-zero tracking error; max_abs=9.523975e-01 deg, rms=3.108286e-01 deg, mean_abs=2.328879e-01 deg |
| PASS | hip_rotation_r output vs IK | non-zero tracking error; max_abs=1.180979e+00 deg, rms=4.448464e-01 deg, mean_abs=3.215957e-01 deg |
| PASS | knee_angle_r output vs IK | non-zero tracking error; max_abs=1.045723e+00 deg, rms=1.936472e-01 deg, mean_abs=1.138140e-01 deg |
| PASS | ankle_angle_r output vs IK | non-zero tracking error; max_abs=6.625547e-01 deg, rms=2.200543e-01 deg, mean_abs=1.626372e-01 deg |
| PASS | subtalar_angle_r output vs IK | non-zero tracking error; max_abs=4.903423e-01 deg, rms=1.437190e-01 deg, mean_abs=1.019820e-01 deg |
| PASS | mtp_angle_r output vs IK | non-zero tracking error; max_abs=1.580415e+00 deg, rms=6.273507e-01 deg, mean_abs=4.847312e-01 deg |
| PASS | hip_flexion_l output vs IK | non-zero tracking error; max_abs=4.055954e+00 deg, rms=1.530211e+00 deg, mean_abs=1.120776e+00 deg |
| PASS | hip_adduction_l output vs IK | non-zero tracking error; max_abs=6.006317e-01 deg, rms=1.857618e-01 deg, mean_abs=1.338187e-01 deg |
| PASS | hip_rotation_l output vs IK | non-zero tracking error; max_abs=7.511128e-01 deg, rms=3.011335e-01 deg, mean_abs=2.277218e-01 deg |
| PASS | lumbar_extension output vs IK | non-zero tracking error; max_abs=1.127555e+00 deg, rms=4.071803e-01 deg, mean_abs=2.871870e-01 deg |
| PASS | lumbar_bending output vs IK | non-zero tracking error; max_abs=1.309375e+00 deg, rms=4.233706e-01 deg, mean_abs=2.782862e-01 deg |
| PASS | lumbar_rotation output vs IK | non-zero tracking error; max_abs=8.941437e-01 deg, rms=3.524483e-01 deg, mean_abs=2.733245e-01 deg |
| PASS | SEA_Knee control saturation | SEA control below saturation; max \|u\|=0.297319 |
| PASS | SEA_Knee motor vs joint power | motor-joint diff max_abs=2.362022e+03 W, rms=5.404940e+01 W, mean_abs=6.348194e+00 W, corr=0.250648 |
| PASS | SEA_Ankle control saturation | SEA control below saturation; max \|u\|=0.554873 |
| PASS | SEA_Ankle motor vs joint power | motor-joint diff max_abs=6.722229e+01 W, rms=1.440551e+01 W, mean_abs=7.881177e+00 W, corr=0.998706 |

## Interpretazione

Almeno un controllo critico indica che il risultato non e validabile.
Un FAIL su `run status` indica output parziali; un FAIL su tracking
indica divergenza dinamica; un FAIL sulle metriche SEA indica possibile
tautologia o derivate plugin non finite.
