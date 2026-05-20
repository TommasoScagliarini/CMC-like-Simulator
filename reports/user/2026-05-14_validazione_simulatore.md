# Validazione simulatore - 2026-05-14 01:24

Risultato complessivo: **FAIL**

- Results dir: `/Users/tommy/Documents/CMC-like-Simulator - Claude/results/_outer_pid_ab06_full`
- Reference: `/Users/tommy/Documents/CMC-like-Simulator - Claude/models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot`

## Checks

| Status | Check | Detail |
|---|---|---|
| PASS | run status | simulation complete at t=21 |
| PASS | SEA_Knee tau_ref - tau_spring | non-trivial torque tracking error; max_abs=8.692805e+00 Nm, rms=3.205758e+00 Nm, mean_abs=2.199088e+00 Nm |
| PASS | SEA_Knee algebraic motor state | motor_angle is not algebraically constrained; max_abs=8.692811e-03 rad, rms=3.205758e-03 rad, mean_abs=2.199088e-03 rad |
| PASS | SEA_Knee motor_speed - qdot | motor_speed differs from joint speed; max_abs=1.232427e+00 rad/s, rms=1.593028e-01 rad/s, mean_abs=1.015170e-01 rad/s |
| PASS | SEA_Ankle tau_ref - tau_spring | non-trivial torque tracking error; max_abs=4.176838e+00 Nm, rms=1.051801e+00 Nm, mean_abs=7.441365e-01 Nm |
| PASS | SEA_Ankle algebraic motor state | motor_angle is not algebraically constrained; max_abs=5.966912e-03 rad, rms=1.502573e-03 rad, mean_abs=1.063052e-03 rad |
| PASS | SEA_Ankle motor_speed - qdot | motor_speed differs from joint speed; max_abs=6.207883e+00 rad/s, rms=3.732166e-01 rad/s, mean_abs=2.067086e-01 rad/s |
| PASS | SEA_Knee derivatives | finite plugin derivatives; speed_dot max_abs=5.243587e+02 rad/s^2, rms=6.194495e+01 rad/s^2, mean_abs=4.285344e+01 rad/s^2 |
| PASS | SEA_Ankle derivatives | finite plugin derivatives; speed_dot max_abs=2.751856e+03 rad/s^2, rms=1.358101e+02 rad/s^2, mean_abs=7.868772e+01 rad/s^2 |
| PASS | SEA_Knee plugin/Python tau_input agreement | plugin output matches independently recomputed SEA law; max_abs=5.000000e-07 Nm, rms=1.530535e-07 Nm, mean_abs=8.442064e-08 Nm |
| PASS | SEA_Knee tau_error diagnostic | finite tau_ref - tau_spring; max_abs=8.692805e+00 Nm, rms=3.205758e+00 Nm, mean_abs=2.199088e+00 Nm |
| PASS | SEA_Knee motor speed diagnostics | motor_speed max_abs=4.065590e+00 rad/s, rms=1.610975e+00 rad/s, mean_abs=1.105020e+00 rad/s; speed_dot max_abs=5.243587e+02 rad/s^2, rms=6.194495e+01 rad/s^2, mean_abs=4.285344e+01 rad/s^2 |
| PASS | SEA_Knee tau_input saturation | tau_input never reaches the +/-500 Nm clamp |
| PASS | SEA_Knee saturation source terms | raw command below clamp; raw max_abs=3.020151e+01 Nm, rms=9.861828e+00 Nm, mean_abs=7.651952e+00 Nm; motor numerator max_abs=5.243587e+00 Nm, rms=6.194495e-01 Nm, mean_abs=4.285344e-01 Nm |
| PASS | SEA_Ankle plugin/Python tau_input agreement | plugin output matches independently recomputed SEA law; max_abs=4.740000e-06 Nm, rms=2.373492e-07 Nm, mean_abs=1.513119e-07 Nm |
| PASS | SEA_Ankle tau_error diagnostic | finite tau_ref - tau_spring; max_abs=4.176837e+00 Nm, rms=1.051801e+00 Nm, mean_abs=7.441365e-01 Nm |
| PASS | SEA_Ankle motor speed diagnostics | motor_speed max_abs=4.424228e+00 rad/s, rms=1.126176e+00 rad/s, mean_abs=8.046848e-01 rad/s; speed_dot max_abs=2.751856e+03 rad/s^2, rms=1.358101e+02 rad/s^2, mean_abs=7.868772e+01 rad/s^2 |
| PASS | SEA_Ankle tau_input saturation | tau_input never reaches the +/-500 Nm clamp |
| PASS | SEA_Ankle saturation source terms | raw command below clamp; raw max_abs=1.004821e+02 Nm, rms=4.130467e+01 Nm, mean_abs=2.912388e+01 Nm; motor numerator max_abs=2.751856e+01 Nm, rms=1.358101e+00 Nm, mean_abs=7.868772e-01 Nm |
| WARN | pros_knee_angle reserve torque | prosthetic reserve is exactly zero; OK only if SEA path is validated; max_abs=0.000000e+00 Nm, rms=0.000000e+00 Nm, mean_abs=0.000000e+00 Nm |
| WARN | pros_ankle_angle reserve torque | prosthetic reserve is exactly zero; OK only if SEA path is validated; max_abs=0.000000e+00 Nm, rms=0.000000e+00 Nm, mean_abs=0.000000e+00 Nm |
| PASS | pros_knee_angle output vs IK | non-zero tracking error; max_abs=9.428967e+00 deg, rms=3.010790e+00 deg, mean_abs=2.278381e+00 deg |
| PASS | pros_ankle_angle output vs IK | non-zero tracking error; max_abs=1.188888e+01 deg, rms=4.852215e+00 deg, mean_abs=3.815224e+00 deg |
| WARN | pelvis_tx output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=4.364630e-03 m, rms=5.812032e-04 m, mean_abs=4.452824e-04 m |
| WARN | pelvis_ty output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=3.689310e-03 m, rms=7.333025e-04 m, mean_abs=6.105992e-04 m |
| WARN | pelvis_tz output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=5.783770e-03 m, rms=5.634947e-04 m, mean_abs=4.382429e-04 m |
| PASS | pelvis_tilt output vs IK | non-zero tracking error; max_abs=2.747525e+00 deg, rms=1.850689e-01 deg, mean_abs=1.242295e-01 deg |
| PASS | pelvis_list output vs IK | non-zero tracking error; max_abs=3.774220e+00 deg, rms=3.264469e-01 deg, mean_abs=1.946422e-01 deg |
| PASS | pelvis_rotation output vs IK | non-zero tracking error; max_abs=4.241202e+00 deg, rms=2.779642e-01 deg, mean_abs=1.709424e-01 deg |
| PASS | hip_flexion_r output vs IK | non-zero tracking error; max_abs=3.543966e+00 deg, rms=3.576531e-01 deg, mean_abs=2.452808e-01 deg |
| PASS | hip_adduction_r output vs IK | non-zero tracking error; max_abs=4.068058e+00 deg, rms=2.052717e-01 deg, mean_abs=1.189357e-01 deg |
| PASS | hip_rotation_r output vs IK | non-zero tracking error; max_abs=4.963486e+00 deg, rms=3.650947e-01 deg, mean_abs=2.931628e-01 deg |
| PASS | knee_angle_r output vs IK | non-zero tracking error; max_abs=1.781001e+00 deg, rms=2.285286e-01 deg, mean_abs=1.703243e-01 deg |
| PASS | ankle_angle_r output vs IK | non-zero tracking error; max_abs=9.684461e+00 deg, rms=3.864127e+00 deg, mean_abs=3.171830e+00 deg |
| PASS | subtalar_angle_r output vs IK | non-zero tracking error; max_abs=1.001999e+01 deg, rms=2.783527e+00 deg, mean_abs=1.979408e+00 deg |
| FAIL | mtp_angle_r output vs IK | tracking error exceeds stability threshold; max_abs=2.979542e+01 deg, rms=1.411482e+01 deg, mean_abs=1.170947e+01 deg |
| PASS | hip_flexion_l output vs IK | non-zero tracking error; max_abs=4.053500e+00 deg, rms=6.269237e-01 deg, mean_abs=4.865121e-01 deg |
| PASS | hip_adduction_l output vs IK | non-zero tracking error; max_abs=2.631915e+00 deg, rms=1.943062e-01 deg, mean_abs=1.348145e-01 deg |
| PASS | hip_rotation_l output vs IK | non-zero tracking error; max_abs=5.535260e+00 deg, rms=4.029740e-01 deg, mean_abs=2.399638e-01 deg |
| PASS | lumbar_extension output vs IK | non-zero tracking error; max_abs=4.858373e-01 deg, rms=1.795235e-01 deg, mean_abs=1.517905e-01 deg |
| PASS | lumbar_bending output vs IK | non-zero tracking error; max_abs=2.609250e-01 deg, rms=1.239842e-01 deg, mean_abs=1.032073e-01 deg |
| PASS | lumbar_rotation output vs IK | non-zero tracking error; max_abs=5.136045e-01 deg, rms=1.703137e-01 deg, mean_abs=1.365842e-01 deg |
| PASS | SEA_Knee control saturation | SEA control below saturation; max \|u\|=0.307064 |
| PASS | SEA_Knee motor vs joint power | motor-joint diff max_abs=1.922158e+01 W, rms=2.040336e+00 W, mean_abs=1.093529e+00 W, corr=0.979810 |
| PASS | SEA_Ankle control saturation | SEA control below saturation; max \|u\|=0.399096 |
| PASS | SEA_Ankle motor vs joint power | motor-joint diff max_abs=1.179371e+02 W, rms=1.392569e+01 W, mean_abs=7.456044e+00 W, corr=0.983270 |

## Interpretazione

Almeno un controllo critico indica che il risultato non e validabile.
Un FAIL su `run status` indica output parziali; un FAIL su tracking
indica divergenza dinamica; un FAIL sulle metriche SEA indica possibile
tautologia o derivate plugin non finite.
