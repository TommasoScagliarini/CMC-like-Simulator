# Validazione simulatore - 2026-04-16 23:51

Risultato complessivo: **FAIL**

- Results dir: `C:\Users\tomma\Desktop\Opensim OMNIBUS\CMC-like-Simulator - Claude\results\_sea_parameter_sweep_20260416_205113\runs\full_k75_a125`
- Reference: `C:\Users\tomma\Desktop\Opensim OMNIBUS\CMC-like-Simulator - Claude\data\3DGaitModel2392_Kinematics_q.sto`

## Checks

| Status | Check | Detail |
|---|---|---|
| PASS | run status | simulation complete at t=11.06 |
| PASS | SEA_Knee tau_ref - tau_spring | non-trivial torque tracking error; max_abs=2.703740e+01 Nm, rms=9.380242e-01 Nm, mean_abs=5.478358e-01 Nm |
| PASS | SEA_Knee algebraic motor state | motor_angle is not algebraically constrained; max_abs=3.604987e-01 rad, rms=1.250699e-02 rad, mean_abs=7.304478e-03 rad |
| PASS | SEA_Knee motor_speed - qdot | motor_speed differs from joint speed; max_abs=5.441423e+01 rad/s, rms=2.841581e+00 rad/s, mean_abs=1.603092e+00 rad/s |
| PASS | SEA_Ankle tau_ref - tau_spring | non-trivial torque tracking error; max_abs=9.469862e+00 Nm, rms=1.424340e+00 Nm, mean_abs=9.694571e-01 Nm |
| PASS | SEA_Ankle algebraic motor state | motor_angle is not algebraically constrained; max_abs=7.575890e-02 rad, rms=1.139472e-02 rad, mean_abs=7.755657e-03 rad |
| PASS | SEA_Ankle motor_speed - qdot | motor_speed differs from joint speed; max_abs=2.330167e+01 rad/s, rms=2.053379e+00 rad/s, mean_abs=1.226934e+00 rad/s |
| PASS | SEA_Knee derivatives | finite plugin derivatives; speed_dot max_abs=5.158057e+04 rad/s^2, rms=1.663871e+03 rad/s^2, mean_abs=3.363292e+02 rad/s^2 |
| PASS | SEA_Ankle derivatives | finite plugin derivatives; speed_dot max_abs=1.799274e+04 rad/s^2, rms=3.953108e+02 rad/s^2, mean_abs=1.588762e+02 rad/s^2 |
| PASS | SEA_Knee plugin/Python tau_input agreement | plugin output matches independently recomputed SEA law; max_abs=4.590000e-06 Nm, rms=2.178256e-07 Nm, mean_abs=9.535588e-08 Nm |
| PASS | SEA_Knee tau_error diagnostic | finite tau_ref - tau_spring; max_abs=2.703740e+01 Nm, rms=9.380242e-01 Nm, mean_abs=5.478358e-01 Nm |
| PASS | SEA_Knee motor speed diagnostics | motor_speed max_abs=6.037146e+01 rad/s, rms=3.642782e+00 rad/s, mean_abs=2.423880e+00 rad/s; speed_dot max_abs=5.158057e+04 rad/s^2, rms=1.663871e+03 rad/s^2, mean_abs=3.363292e+02 rad/s^2 |
| WARN | SEA_Knee tau_input saturation | tau_input reaches +/-500 Nm in 2 samples (0.029%); tuning issue, not an interface failure |
| PASS | SEA_Knee saturation source terms | sat median \|tau_ff\|=10.192 Nm, \|outer_PD\|=0.358 Nm, \|inner_prop\|=850.444 Nm, \|inner_damp\|=41.183 Nm; raw excess max_abs=3.671525e+02 Nm, rms=5.395918e+00 Nm, mean_abs=9.095933e-02 Nm; motor numerator max_abs=5.158057e+02 Nm, rms=1.663871e+01 Nm, mean_abs=3.363292e+00 Nm |
| PASS | SEA_Ankle plugin/Python tau_input agreement | plugin output matches independently recomputed SEA law; max_abs=3.630000e-06 Nm, rms=1.877301e-07 Nm, mean_abs=1.037221e-07 Nm |
| PASS | SEA_Ankle tau_error diagnostic | finite tau_ref - tau_spring; max_abs=9.469862e+00 Nm, rms=1.424340e+00 Nm, mean_abs=9.694571e-01 Nm |
| PASS | SEA_Ankle motor speed diagnostics | motor_speed max_abs=1.760868e+01 rad/s, rms=1.266877e+00 rad/s, mean_abs=9.274014e-01 rad/s; speed_dot max_abs=1.799274e+04 rad/s^2, rms=3.953108e+02 rad/s^2, mean_abs=1.588762e+02 rad/s^2 |
| PASS | SEA_Ankle tau_input saturation | tau_input never reaches the +/-500 Nm clamp |
| PASS | SEA_Ankle saturation source terms | raw command below clamp; raw max_abs=1.799274e+02 Nm, rms=3.039228e+01 Nm, mean_abs=1.843666e+01 Nm; motor numerator max_abs=1.799274e+02 Nm, rms=3.953108e+00 Nm, mean_abs=1.588762e+00 Nm |
| WARN | pros_knee_angle reserve torque | prosthetic reserve is exactly zero; OK only if SEA path is validated; max_abs=0.000000e+00 Nm, rms=0.000000e+00 Nm, mean_abs=0.000000e+00 Nm |
| WARN | pros_ankle_angle reserve torque | prosthetic reserve is exactly zero; OK only if SEA path is validated; max_abs=0.000000e+00 Nm, rms=0.000000e+00 Nm, mean_abs=0.000000e+00 Nm |
| PASS | pros_knee_angle output vs IK | non-zero tracking error; max_abs=8.917014e+00 deg, rms=4.383867e+00 deg, mean_abs=3.900182e+00 deg |
| FAIL | pros_ankle_angle output vs IK | tracking error exceeds stability threshold; max_abs=3.313357e+01 deg, rms=1.431599e+01 deg, mean_abs=1.029486e+01 deg |
| PASS | pelvis_tx output vs IK | non-zero tracking error; max_abs=3.040520e-03 m, rms=1.066104e-03 m, mean_abs=7.639837e-04 m |
| PASS | pelvis_ty output vs IK | non-zero tracking error; max_abs=4.748210e-03 m, rms=1.609566e-03 m, mean_abs=1.136086e-03 m |
| PASS | pelvis_tz output vs IK | non-zero tracking error; max_abs=1.158963e-02 m, rms=2.735555e-03 m, mean_abs=1.414296e-03 m |
| PASS | pelvis_tilt output vs IK | non-zero tracking error; max_abs=3.607197e-01 deg, rms=1.665502e-01 deg, mean_abs=1.348354e-01 deg |
| PASS | pelvis_list output vs IK | non-zero tracking error; max_abs=2.986786e-01 deg, rms=1.144506e-01 deg, mean_abs=8.716382e-02 deg |
| PASS | pelvis_rotation output vs IK | non-zero tracking error; max_abs=4.898765e-01 deg, rms=2.226759e-01 deg, mean_abs=1.824643e-01 deg |
| PASS | hip_flexion_r output vs IK | non-zero tracking error; max_abs=5.618144e-01 deg, rms=1.915213e-01 deg, mean_abs=1.510863e-01 deg |
| PASS | hip_adduction_r output vs IK | non-zero tracking error; max_abs=3.871994e-01 deg, rms=1.522946e-01 deg, mean_abs=1.241913e-01 deg |
| PASS | hip_rotation_r output vs IK | non-zero tracking error; max_abs=4.549846e-01 deg, rms=1.868829e-01 deg, mean_abs=1.499755e-01 deg |
| PASS | knee_angle_r output vs IK | non-zero tracking error; max_abs=1.029736e+00 deg, rms=1.756187e-01 deg, mean_abs=8.519329e-02 deg |
| PASS | ankle_angle_r output vs IK | non-zero tracking error; max_abs=6.537450e-01 deg, rms=2.184120e-01 deg, mean_abs=1.592221e-01 deg |
| WARN | subtalar_angle_r output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=3.050547e-01 deg, rms=9.662106e-02 deg, mean_abs=7.586720e-02 deg |
| PASS | mtp_angle_r output vs IK | non-zero tracking error; max_abs=1.576961e+00 deg, rms=6.221205e-01 deg, mean_abs=4.735357e-01 deg |
| PASS | hip_flexion_l output vs IK | non-zero tracking error; max_abs=1.468708e+00 deg, rms=6.534884e-01 deg, mean_abs=5.382663e-01 deg |
| WARN | hip_adduction_l output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=2.043923e-01 deg, rms=9.217257e-02 deg, mean_abs=7.528565e-02 deg |
| PASS | hip_rotation_l output vs IK | non-zero tracking error; max_abs=3.157553e-01 deg, rms=1.316855e-01 deg, mean_abs=1.095711e-01 deg |
| PASS | lumbar_extension output vs IK | non-zero tracking error; max_abs=3.626161e-01 deg, rms=1.561604e-01 deg, mean_abs=1.245813e-01 deg |
| PASS | lumbar_bending output vs IK | non-zero tracking error; max_abs=4.351845e-01 deg, rms=1.606278e-01 deg, mean_abs=1.288374e-01 deg |
| PASS | lumbar_rotation output vs IK | non-zero tracking error; max_abs=4.264461e-01 deg, rms=1.745978e-01 deg, mean_abs=1.445041e-01 deg |
| PASS | SEA_Knee control saturation | SEA control below saturation; max \|u\|=0.290655 |
| PASS | SEA_Knee motor vs joint power | motor-joint diff max_abs=2.495409e+04 W, rms=3.700712e+02 W, mean_abs=3.209955e+01 W, corr=0.016340 |
| PASS | SEA_Ankle control saturation | SEA control below saturation; max \|u\|=0.297822 |
| PASS | SEA_Ankle motor vs joint power | motor-joint diff max_abs=1.128047e+03 W, rms=6.362576e+01 W, mean_abs=3.358111e+01 W, corr=0.352318 |

## Interpretazione

Almeno un controllo critico indica che il risultato non e validabile.
Un FAIL su `run status` indica output parziali; un FAIL su tracking
indica divergenza dinamica; un FAIL sulle metriche SEA indica possibile
tautologia o derivate plugin non finite.
