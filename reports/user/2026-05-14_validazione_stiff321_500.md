# Validazione simulatore - 2026-05-14 20:19

Risultato complessivo: **FAIL**

- Results dir: `/Users/tommy/Documents/CMC-like-Simulator - Claude/results/_stiff321_500_ab06_pd_full`
- Reference: `/Users/tommy/Documents/CMC-like-Simulator - Claude/models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot`

## Checks

| Status | Check | Detail |
|---|---|---|
| PASS | run status | simulation complete at t=21 |
| PASS | SEA_Knee tau_ref - tau_spring | non-trivial torque tracking error; max_abs=9.501728e+00 Nm, rms=3.109486e+00 Nm, mean_abs=2.132775e+00 Nm |
| PASS | SEA_Knee algebraic motor state | motor_angle is not algebraically constrained; max_abs=2.960040e-02 rad, rms=9.686872e-03 rad, mean_abs=6.644160e-03 rad |
| PASS | SEA_Knee motor_speed - qdot | motor_speed differs from joint speed; max_abs=3.921451e+00 rad/s, rms=4.527370e-01 rad/s, mean_abs=2.800503e-01 rad/s |
| PASS | SEA_Ankle tau_ref - tau_spring | non-trivial torque tracking error; max_abs=3.997810e+00 Nm, rms=9.760978e-01 Nm, mean_abs=6.833021e-01 Nm |
| PASS | SEA_Ankle algebraic motor state | motor_angle is not algebraically constrained; max_abs=7.995620e-03 rad, rms=1.952196e-03 rad, mean_abs=1.366604e-03 rad |
| PASS | SEA_Ankle motor_speed - qdot | motor_speed differs from joint speed; max_abs=8.662930e+00 rad/s, rms=4.856335e-01 rad/s, mean_abs=2.761866e-01 rad/s |
| PASS | SEA_Knee derivatives | finite plugin derivatives; speed_dot max_abs=9.703524e+02 rad/s^2, rms=9.224662e+01 rad/s^2, mean_abs=5.582461e+01 rad/s^2 |
| PASS | SEA_Ankle derivatives | finite plugin derivatives; speed_dot max_abs=2.570588e+03 rad/s^2, rms=1.280286e+02 rad/s^2, mean_abs=7.676927e+01 rad/s^2 |
| PASS | SEA_Knee plugin/Python tau_input agreement | plugin output matches independently recomputed SEA law; max_abs=5.000000e-07 Nm, rms=1.517033e-07 Nm, mean_abs=8.392009e-08 Nm |
| PASS | SEA_Knee tau_error diagnostic | finite tau_ref - tau_spring; max_abs=9.501728e+00 Nm, rms=3.109486e+00 Nm, mean_abs=2.132775e+00 Nm |
| PASS | SEA_Knee motor speed diagnostics | motor_speed max_abs=4.563513e+00 rad/s, rms=1.555588e+00 rad/s, mean_abs=1.065526e+00 rad/s; speed_dot max_abs=9.703524e+02 rad/s^2, rms=9.224662e+01 rad/s^2, mean_abs=5.582461e+01 rad/s^2 |
| PASS | SEA_Knee tau_input saturation | tau_input never reaches the +/-500 Nm clamp |
| PASS | SEA_Knee saturation source terms | raw command below clamp; raw max_abs=3.029443e+01 Nm, rms=9.794203e+00 Nm, mean_abs=7.561768e+00 Nm; motor numerator max_abs=9.703524e+00 Nm, rms=9.224662e-01 Nm, mean_abs=5.582461e-01 Nm |
| PASS | SEA_Ankle plugin/Python tau_input agreement | plugin output matches independently recomputed SEA law; max_abs=5.000000e-07 Nm, rms=2.178671e-07 Nm, mean_abs=1.459822e-07 Nm |
| PASS | SEA_Ankle tau_error diagnostic | finite tau_ref - tau_spring; max_abs=3.997811e+00 Nm, rms=9.760978e-01 Nm, mean_abs=6.833021e-01 Nm |
| PASS | SEA_Ankle motor speed diagnostics | motor_speed max_abs=4.160917e+00 rad/s, rms=1.041782e+00 rad/s, mean_abs=7.389459e-01 rad/s; speed_dot max_abs=2.570588e+03 rad/s^2, rms=1.280286e+02 rad/s^2, mean_abs=7.676927e+01 rad/s^2 |
| PASS | SEA_Ankle tau_input saturation | tau_input never reaches the +/-500 Nm clamp |
| PASS | SEA_Ankle saturation source terms | raw command below clamp; raw max_abs=9.776804e+01 Nm, rms=3.982858e+01 Nm, mean_abs=2.804541e+01 Nm; motor numerator max_abs=2.570588e+01 Nm, rms=1.280286e+00 Nm, mean_abs=7.676927e-01 Nm |
| WARN | pros_knee_angle reserve torque | prosthetic reserve is exactly zero; OK only if SEA path is validated; max_abs=0.000000e+00 Nm, rms=0.000000e+00 Nm, mean_abs=0.000000e+00 Nm |
| WARN | pros_ankle_angle reserve torque | prosthetic reserve is exactly zero; OK only if SEA path is validated; max_abs=0.000000e+00 Nm, rms=0.000000e+00 Nm, mean_abs=0.000000e+00 Nm |
| PASS | pros_knee_angle output vs IK | non-zero tracking error; max_abs=9.666178e+00 deg, rms=3.124600e+00 deg, mean_abs=2.326179e+00 deg |
| PASS | pros_ankle_angle output vs IK | non-zero tracking error; max_abs=1.311230e+01 deg, rms=5.416394e+00 deg, mean_abs=3.870163e+00 deg |
| WARN | pelvis_tx output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=4.379480e-03 m, rms=5.742058e-04 m, mean_abs=4.380280e-04 m |
| WARN | pelvis_ty output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=3.702820e-03 m, rms=7.080226e-04 m, mean_abs=5.870377e-04 m |
| WARN | pelvis_tz output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=5.795130e-03 m, rms=5.555680e-04 m, mean_abs=4.297033e-04 m |
| PASS | pelvis_tilt output vs IK | non-zero tracking error; max_abs=2.749772e+00 deg, rms=1.827753e-01 deg, mean_abs=1.214788e-01 deg |
| PASS | pelvis_list output vs IK | non-zero tracking error; max_abs=3.774767e+00 deg, rms=3.247778e-01 deg, mean_abs=1.916478e-01 deg |
| PASS | pelvis_rotation output vs IK | non-zero tracking error; max_abs=4.237351e+00 deg, rms=2.766156e-01 deg, mean_abs=1.690290e-01 deg |
| PASS | hip_flexion_r output vs IK | non-zero tracking error; max_abs=3.548775e+00 deg, rms=3.542708e-01 deg, mean_abs=2.409581e-01 deg |
| PASS | hip_adduction_r output vs IK | non-zero tracking error; max_abs=4.069634e+00 deg, rms=2.044639e-01 deg, mean_abs=1.174958e-01 deg |
| PASS | hip_rotation_r output vs IK | non-zero tracking error; max_abs=4.964400e+00 deg, rms=3.693300e-01 deg, mean_abs=2.981397e-01 deg |
| PASS | knee_angle_r output vs IK | non-zero tracking error; max_abs=1.780146e+00 deg, rms=2.282228e-01 deg, mean_abs=1.699354e-01 deg |
| PASS | ankle_angle_r output vs IK | non-zero tracking error; max_abs=9.721711e+00 deg, rms=3.873361e+00 deg, mean_abs=3.172866e+00 deg |
| PASS | subtalar_angle_r output vs IK | non-zero tracking error; max_abs=1.005707e+01 deg, rms=2.793642e+00 deg, mean_abs=1.974821e+00 deg |
| FAIL | mtp_angle_r output vs IK | tracking error exceeds stability threshold; max_abs=2.984091e+01 deg, rms=1.410606e+01 deg, mean_abs=1.168650e+01 deg |
| PASS | hip_flexion_l output vs IK | non-zero tracking error; max_abs=4.052800e+00 deg, rms=6.189287e-01 deg, mean_abs=4.771538e-01 deg |
| PASS | hip_adduction_l output vs IK | non-zero tracking error; max_abs=2.635637e+00 deg, rms=1.929554e-01 deg, mean_abs=1.330321e-01 deg |
| PASS | hip_rotation_l output vs IK | non-zero tracking error; max_abs=5.532548e+00 deg, rms=4.013604e-01 deg, mean_abs=2.370538e-01 deg |
| PASS | lumbar_extension output vs IK | non-zero tracking error; max_abs=4.890413e-01 deg, rms=1.761598e-01 deg, mean_abs=1.489587e-01 deg |
| PASS | lumbar_bending output vs IK | non-zero tracking error; max_abs=2.645512e-01 deg, rms=1.198170e-01 deg, mean_abs=1.000590e-01 deg |
| PASS | lumbar_rotation output vs IK | non-zero tracking error; max_abs=5.167638e-01 deg, rms=1.686473e-01 deg, mean_abs=1.357423e-01 deg |
| PASS | SEA_Knee control saturation | SEA control below saturation; max \|u\|=0.307552 |
| PASS | SEA_Knee motor vs joint power | motor-joint diff max_abs=5.921527e+01 W, rms=4.942252e+00 W, mean_abs=2.457457e+00 W, corr=0.874070 |
| PASS | SEA_Ankle control saturation | SEA control below saturation; max \|u\|=0.387439 |
| PASS | SEA_Ankle motor vs joint power | motor-joint diff max_abs=1.715192e+02 W, rms=1.938955e+01 W, mean_abs=1.008721e+01 W, corr=0.951314 |

## Interpretazione

Almeno un controllo critico indica che il risultato non e validabile.
Un FAIL su `run status` indica output parziali; un FAIL su tracking
indica divergenza dinamica; un FAIL sulle metriche SEA indica possibile
tautologia o derivate plugin non finite.
