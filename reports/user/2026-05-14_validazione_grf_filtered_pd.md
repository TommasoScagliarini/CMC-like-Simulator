# Validazione simulatore - 2026-05-14 19:29

Risultato complessivo: **FAIL**

- Results dir: `/Users/tommy/Documents/CMC-like-Simulator - Claude/results/_grf_filtered_ab06_pd_full`
- Reference: `/Users/tommy/Documents/CMC-like-Simulator - Claude/models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot`

## Checks

| Status | Check | Detail |
|---|---|---|
| PASS | run status | simulation complete at t=21 |
| PASS | SEA_Knee tau_ref - tau_spring | non-trivial torque tracking error; max_abs=8.641304e+00 Nm, rms=3.188139e+00 Nm, mean_abs=2.185177e+00 Nm |
| PASS | SEA_Knee algebraic motor state | motor_angle is not algebraically constrained; max_abs=8.641297e-03 rad, rms=3.188139e-03 rad, mean_abs=2.185177e-03 rad |
| PASS | SEA_Knee motor_speed - qdot | motor_speed differs from joint speed; max_abs=1.234407e+00 rad/s, rms=1.565461e-01 rad/s, mean_abs=9.877443e-02 rad/s |
| PASS | SEA_Ankle tau_ref - tau_spring | non-trivial torque tracking error; max_abs=4.165213e+00 Nm, rms=1.042513e+00 Nm, mean_abs=7.380512e-01 Nm |
| PASS | SEA_Ankle algebraic motor state | motor_angle is not algebraically constrained; max_abs=5.950307e-03 rad, rms=1.489304e-03 rad, mean_abs=1.054359e-03 rad |
| PASS | SEA_Ankle motor_speed - qdot | motor_speed differs from joint speed; max_abs=6.207495e+00 rad/s, rms=3.224750e-01 rad/s, mean_abs=1.860995e-01 rad/s |
| PASS | SEA_Knee derivatives | finite plugin derivatives; speed_dot max_abs=5.242818e+02 rad/s^2, rms=5.950923e+01 rad/s^2, mean_abs=4.197573e+01 rad/s^2 |
| PASS | SEA_Ankle derivatives | finite plugin derivatives; speed_dot max_abs=2.136385e+03 rad/s^2, rms=1.124460e+02 rad/s^2, mean_abs=7.198798e+01 rad/s^2 |
| PASS | SEA_Knee plugin/Python tau_input agreement | plugin output matches independently recomputed SEA law; max_abs=5.000000e-07 Nm, rms=1.496843e-07 Nm, mean_abs=8.223418e-08 Nm |
| PASS | SEA_Knee tau_error diagnostic | finite tau_ref - tau_spring; max_abs=8.641304e+00 Nm, rms=3.188139e+00 Nm, mean_abs=2.185177e+00 Nm |
| PASS | SEA_Knee motor speed diagnostics | motor_speed max_abs=4.067434e+00 rad/s, rms=1.601956e+00 rad/s, mean_abs=1.097910e+00 rad/s; speed_dot max_abs=5.242818e+02 rad/s^2, rms=5.950923e+01 rad/s^2, mean_abs=4.197573e+01 rad/s^2 |
| PASS | SEA_Knee tau_input saturation | tau_input never reaches the +/-500 Nm clamp |
| PASS | SEA_Knee saturation source terms | raw command below clamp; raw max_abs=3.017941e+01 Nm, rms=9.790529e+00 Nm, mean_abs=7.554068e+00 Nm; motor numerator max_abs=5.242818e+00 Nm, rms=5.950923e-01 Nm, mean_abs=4.197572e-01 Nm |
| PASS | SEA_Ankle plugin/Python tau_input agreement | plugin output matches independently recomputed SEA law; max_abs=5.000000e-07 Nm, rms=2.177616e-07 Nm, mean_abs=1.461376e-07 Nm |
| PASS | SEA_Ankle tau_error diagnostic | finite tau_ref - tau_spring; max_abs=4.165213e+00 Nm, rms=1.042513e+00 Nm, mean_abs=7.380512e-01 Nm |
| PASS | SEA_Ankle motor speed diagnostics | motor_speed max_abs=4.410115e+00 rad/s, rms=1.120325e+00 rad/s, mean_abs=8.001496e-01 rad/s; speed_dot max_abs=2.136385e+03 rad/s^2, rms=1.124460e+02 rad/s^2, mean_abs=7.198798e+01 rad/s^2 |
| PASS | SEA_Ankle tau_input saturation | tau_input never reaches the +/-500 Nm clamp |
| PASS | SEA_Ankle saturation source terms | raw command below clamp; raw max_abs=9.818859e+01 Nm, rms=4.000278e+01 Nm, mean_abs=2.811293e+01 Nm; motor numerator max_abs=2.136385e+01 Nm, rms=1.124460e+00 Nm, mean_abs=7.198798e-01 Nm |
| WARN | pros_knee_angle reserve torque | prosthetic reserve is exactly zero; OK only if SEA path is validated; max_abs=0.000000e+00 Nm, rms=0.000000e+00 Nm, mean_abs=0.000000e+00 Nm |
| WARN | pros_ankle_angle reserve torque | prosthetic reserve is exactly zero; OK only if SEA path is validated; max_abs=0.000000e+00 Nm, rms=0.000000e+00 Nm, mean_abs=0.000000e+00 Nm |
| PASS | pros_knee_angle output vs IK | non-zero tracking error; max_abs=9.681471e+00 deg, rms=3.134225e+00 deg, mean_abs=2.339627e+00 deg |
| PASS | pros_ankle_angle output vs IK | non-zero tracking error; max_abs=1.319736e+01 deg, rms=5.438842e+00 deg, mean_abs=3.879288e+00 deg |
| WARN | pelvis_tx output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=4.364710e-03 m, rms=5.775106e-04 m, mean_abs=4.417904e-04 m |
| WARN | pelvis_ty output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=3.682630e-03 m, rms=7.107584e-04 m, mean_abs=5.903321e-04 m |
| WARN | pelvis_tz output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=5.780160e-03 m, rms=5.560353e-04 m, mean_abs=4.306873e-04 m |
| PASS | pelvis_tilt output vs IK | non-zero tracking error; max_abs=2.750771e+00 deg, rms=1.834268e-01 deg, mean_abs=1.222548e-01 deg |
| PASS | pelvis_list output vs IK | non-zero tracking error; max_abs=3.778118e+00 deg, rms=3.246222e-01 deg, mean_abs=1.917424e-01 deg |
| PASS | pelvis_rotation output vs IK | non-zero tracking error; max_abs=4.239216e+00 deg, rms=2.767323e-01 deg, mean_abs=1.697996e-01 deg |
| PASS | hip_flexion_r output vs IK | non-zero tracking error; max_abs=3.551852e+00 deg, rms=3.545898e-01 deg, mean_abs=2.414929e-01 deg |
| PASS | hip_adduction_r output vs IK | non-zero tracking error; max_abs=4.072668e+00 deg, rms=2.045258e-01 deg, mean_abs=1.177406e-01 deg |
| PASS | hip_rotation_r output vs IK | non-zero tracking error; max_abs=4.967546e+00 deg, rms=3.693955e-01 deg, mean_abs=2.978749e-01 deg |
| PASS | knee_angle_r output vs IK | non-zero tracking error; max_abs=1.780427e+00 deg, rms=2.283833e-01 deg, mean_abs=1.700305e-01 deg |
| PASS | ankle_angle_r output vs IK | non-zero tracking error; max_abs=9.722561e+00 deg, rms=3.873411e+00 deg, mean_abs=3.172984e+00 deg |
| PASS | subtalar_angle_r output vs IK | non-zero tracking error; max_abs=1.003957e+01 deg, rms=2.792521e+00 deg, mean_abs=1.974788e+00 deg |
| FAIL | mtp_angle_r output vs IK | tracking error exceeds stability threshold; max_abs=2.984218e+01 deg, rms=1.409874e+01 deg, mean_abs=1.168152e+01 deg |
| PASS | hip_flexion_l output vs IK | non-zero tracking error; max_abs=4.063433e+00 deg, rms=6.190712e-01 deg, mean_abs=4.776316e-01 deg |
| PASS | hip_adduction_l output vs IK | non-zero tracking error; max_abs=2.635484e+00 deg, rms=1.931184e-01 deg, mean_abs=1.332591e-01 deg |
| PASS | hip_rotation_l output vs IK | non-zero tracking error; max_abs=5.534247e+00 deg, rms=4.016232e-01 deg, mean_abs=2.375744e-01 deg |
| PASS | lumbar_extension output vs IK | non-zero tracking error; max_abs=4.836497e-01 deg, rms=1.759287e-01 deg, mean_abs=1.486913e-01 deg |
| PASS | lumbar_bending output vs IK | non-zero tracking error; max_abs=2.583129e-01 deg, rms=1.211022e-01 deg, mean_abs=1.007211e-01 deg |
| PASS | lumbar_rotation output vs IK | non-zero tracking error; max_abs=5.089149e-01 deg, rms=1.686230e-01 deg, mean_abs=1.358758e-01 deg |
| PASS | SEA_Knee control saturation | SEA control below saturation; max \|u\|=0.306414 |
| PASS | SEA_Knee motor vs joint power | motor-joint diff max_abs=1.891857e+01 W, rms=1.979944e+00 W, mean_abs=1.059413e+00 W, corr=0.980113 |
| PASS | SEA_Ankle control saturation | SEA control below saturation; max \|u\|=0.389856 |
| PASS | SEA_Ankle motor vs joint power | motor-joint diff max_abs=1.143798e+02 W, rms=1.332461e+01 W, mean_abs=7.044615e+00 W, corr=0.983608 |

## Interpretazione

Almeno un controllo critico indica che il risultato non e validabile.
Un FAIL su `run status` indica output parziali; un FAIL su tracking
indica divergenza dinamica; un FAIL sulle metriche SEA indica possibile
tautologia o derivate plugin non finite.
