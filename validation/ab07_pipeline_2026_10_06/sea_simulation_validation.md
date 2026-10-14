# Validazione simulatore - 2026-10-06 11:25

Risultato complessivo: **FAIL**

- Results dir: `/Users/tommy/Documents/CMC-like-Simulator - Claude/results/AB07_prescribed_2026_10_06`
- Reference: `/Users/tommy/Documents/CMC-like-Simulator - Claude/models/AB07_SEASEA_Threadmill/data/IK_results_AB07_SEASEA.mot`

## Checks

| Status | Check | Detail |
|---|---|---|
| PASS | run status | simulation complete at t=19.37 |
| PASS | SEA_Knee tau_ref - tau_spring | non-trivial torque tracking error; max_abs=7.757929e+00 Nm, rms=1.260598e+00 Nm, mean_abs=9.169587e-01 Nm |
| PASS | SEA_Knee algebraic motor state | motor_angle is not algebraically constrained; max_abs=2.416799e-02 rad, rms=3.927096e-03 rad, mean_abs=2.856569e-03 rad |
| PASS | SEA_Knee motor_speed - qdot | motor_speed differs from joint speed; max_abs=1.160858e+01 rad/s, rms=1.419036e+00 rad/s, mean_abs=8.682131e-01 rad/s |
| PASS | SEA_Ankle tau_ref - tau_spring | non-trivial torque tracking error; max_abs=5.040651e+00 Nm, rms=7.927454e-01 Nm, mean_abs=5.071599e-01 Nm |
| PASS | SEA_Ankle algebraic motor state | motor_angle is not algebraically constrained; max_abs=1.008130e-02 rad, rms=1.585491e-03 rad, mean_abs=1.014320e-03 rad |
| PASS | SEA_Ankle motor_speed - qdot | motor_speed differs from joint speed; max_abs=5.495282e+00 rad/s, rms=7.013485e-01 rad/s, mean_abs=3.683650e-01 rad/s |
| PASS | SEA_Knee derivatives | finite plugin derivatives; speed_dot max_abs=5.910082e+03 rad/s^2, rms=1.024081e+03 rad/s^2, mean_abs=6.435015e+02 rad/s^2 |
| PASS | SEA_Ankle derivatives | finite plugin derivatives; speed_dot max_abs=3.500216e+03 rad/s^2, rms=4.237072e+02 rad/s^2, mean_abs=2.254703e+02 rad/s^2 |
| PASS | SEA_Knee plugin/Python tau_input agreement | plugin output matches independently recomputed SEA law; max_abs=5.000000e-07 Nm, rms=1.892944e-07 Nm, mean_abs=1.180544e-07 Nm |
| PASS | SEA_Knee tau_error diagnostic | finite tau_ref - tau_spring; max_abs=7.757929e+00 Nm, rms=1.260598e+00 Nm, mean_abs=9.169587e-01 Nm |
| PASS | SEA_Knee motor speed diagnostics | motor_speed max_abs=1.037540e+01 rad/s, rms=2.213831e+00 rad/s, mean_abs=1.601838e+00 rad/s; speed_dot max_abs=5.910082e+03 rad/s^2, rms=1.024081e+03 rad/s^2, mean_abs=6.435015e+02 rad/s^2 |
| PASS | SEA_Knee tau_input saturation | tau_input never reaches the +/-500 Nm clamp |
| PASS | SEA_Knee saturation source terms | raw command below clamp; raw max_abs=5.623123e+01 Nm, rms=1.360664e+01 Nm, mean_abs=1.032951e+01 Nm; motor numerator max_abs=5.910082e+01 Nm, rms=1.024081e+01 Nm, mean_abs=6.435015e+00 Nm |
| PASS | SEA_Ankle plugin/Python tau_input agreement | plugin output matches independently recomputed SEA law; max_abs=5.000000e-07 Nm, rms=1.893177e-07 Nm, mean_abs=1.149811e-07 Nm |
| PASS | SEA_Ankle tau_error diagnostic | finite tau_ref - tau_spring; max_abs=5.040650e+00 Nm, rms=7.927454e-01 Nm, mean_abs=5.071599e-01 Nm |
| PASS | SEA_Ankle motor speed diagnostics | motor_speed max_abs=5.138778e+00 rad/s, rms=8.842528e-01 rad/s, mean_abs=6.224142e-01 rad/s; speed_dot max_abs=3.500216e+03 rad/s^2, rms=4.237072e+02 rad/s^2, mean_abs=2.254703e+02 rad/s^2 |
| PASS | SEA_Ankle tau_input saturation | tau_input never reaches the +/-500 Nm clamp |
| PASS | SEA_Ankle saturation source terms | raw command below clamp; raw max_abs=8.445809e+01 Nm, rms=2.256731e+01 Nm, mean_abs=1.463584e+01 Nm; motor numerator max_abs=3.500216e+01 Nm, rms=4.237072e+00 Nm, mean_abs=2.254703e+00 Nm |
| WARN | pros_knee_angle reserve torque | prosthetic reserve is exactly zero; OK only if SEA path is validated; max_abs=0.000000e+00 Nm, rms=0.000000e+00 Nm, mean_abs=0.000000e+00 Nm |
| WARN | pros_ankle_angle reserve torque | prosthetic reserve is exactly zero; OK only if SEA path is validated; max_abs=0.000000e+00 Nm, rms=0.000000e+00 Nm, mean_abs=0.000000e+00 Nm |
| PASS | pros_knee_angle output vs IK | non-zero tracking error; max_abs=1.630846e+00 deg, rms=2.808493e-01 deg, mean_abs=1.727985e-01 deg |
| PASS | pros_ankle_angle output vs IK | non-zero tracking error; max_abs=3.294703e+00 deg, rms=7.480717e-01 deg, mean_abs=5.431060e-01 deg |
| WARN | pelvis_tx output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=1.153120e-03 m, rms=2.266590e-04 m, mean_abs=1.742484e-04 m |
| WARN | pelvis_ty output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=1.300210e-03 m, rms=2.975286e-04 m, mean_abs=2.411739e-04 m |
| WARN | pelvis_tz output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=1.487840e-03 m, rms=2.181673e-04 m, mean_abs=1.623649e-04 m |
| PASS | pelvis_tilt output vs IK | non-zero tracking error; max_abs=8.723995e-01 deg, rms=1.800571e-01 deg, mean_abs=1.404287e-01 deg |
| PASS | pelvis_list output vs IK | non-zero tracking error; max_abs=7.757090e-01 deg, rms=1.310497e-01 deg, mean_abs=9.805486e-02 deg |
| PASS | pelvis_rotation output vs IK | non-zero tracking error; max_abs=5.962704e-01 deg, rms=1.199406e-01 deg, mean_abs=9.422169e-02 deg |
| PASS | hip_flexion_r output vs IK | non-zero tracking error; max_abs=9.872919e-01 deg, rms=2.079271e-01 deg, mean_abs=1.561643e-01 deg |
| PASS | hip_adduction_r output vs IK | non-zero tracking error; max_abs=1.051128e+00 deg, rms=2.036440e-01 deg, mean_abs=1.557396e-01 deg |
| PASS | hip_rotation_r output vs IK | non-zero tracking error; max_abs=1.700824e+00 deg, rms=5.443667e-01 deg, mean_abs=4.375172e-01 deg |
| PASS | knee_angle_r output vs IK | non-zero tracking error; max_abs=1.304073e+00 deg, rms=2.446138e-01 deg, mean_abs=1.724390e-01 deg |
| PASS | ankle_angle_r output vs IK | non-zero tracking error; max_abs=7.425900e+00 deg, rms=2.800639e+00 deg, mean_abs=2.152457e+00 deg |
| PASS | subtalar_angle_r output vs IK | non-zero tracking error; max_abs=8.765217e+00 deg, rms=2.745853e+00 deg, mean_abs=2.015187e+00 deg |
| FAIL | mtp_angle_r output vs IK | tracking error exceeds stability threshold; max_abs=2.892874e+01 deg, rms=1.122400e+01 deg, mean_abs=8.616090e+00 deg |
| PASS | hip_flexion_l output vs IK | non-zero tracking error; max_abs=1.030018e+00 deg, rms=1.739264e-01 deg, mean_abs=1.304826e-01 deg |
| PASS | hip_adduction_l output vs IK | non-zero tracking error; max_abs=1.040380e+00 deg, rms=1.999287e-01 deg, mean_abs=1.536241e-01 deg |
| PASS | hip_rotation_l output vs IK | non-zero tracking error; max_abs=1.222601e+00 deg, rms=2.104854e-01 deg, mean_abs=1.552481e-01 deg |
| WARN | lumbar_extension output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=3.165179e-01 deg, rms=9.506809e-02 deg, mean_abs=6.817712e-02 deg |
| PASS | lumbar_bending output vs IK | non-zero tracking error; max_abs=3.610620e-01 deg, rms=1.618236e-01 deg, mean_abs=1.246054e-01 deg |
| WARN | lumbar_rotation output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=2.212041e-01 deg, rms=8.376673e-02 deg, mean_abs=6.511949e-02 deg |
| PASS | SEA_Knee control saturation | SEA control below saturation; max \|u\|=0.329967 |
| PASS | SEA_Knee motor vs joint power | motor-joint diff max_abs=3.831390e+02 W, rms=3.127085e+01 W, mean_abs=1.419247e+01 W, corr=0.197321 |
| PASS | SEA_Ankle control saturation | SEA control below saturation; max \|u\|=0.289922 |
| PASS | SEA_Ankle motor vs joint power | motor-joint diff max_abs=2.854716e+02 W, rms=2.717862e+01 W, mean_abs=1.022692e+01 W, corr=0.289565 |

## Interpretazione

Almeno un controllo critico indica che il risultato non e validabile.
Un FAIL su `run status` indica output parziali; un FAIL su tracking
indica divergenza dinamica; un FAIL sulle metriche SEA indica possibile
tautologia o derivate plugin non finite.
