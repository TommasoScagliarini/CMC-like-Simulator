# Validazione simulatore - 2026-04-18 23:59

Risultato complessivo: **FAIL**

- Results dir: `C:\Users\tomma\Desktop\Opensim OMNIBUS\CMC-like-Simulator - Claude\results\_sea_driver_sweep_20260418_213542\runs\screen_kk250_ka500_wn700_z0p7_td0p001`
- Reference: `C:\Users\tomma\Desktop\Opensim OMNIBUS\CMC-like-Simulator - Claude\data\3DGaitModel2392_Kinematics_q.sto`

## Checks

| Status | Check | Detail |
|---|---|---|
| PASS | run status | simulation complete at t=6.55 |
| PASS | SEA_Knee tau_ref - tau_spring | non-trivial torque tracking error; max_abs=2.850526e+01 Nm, rms=1.544414e+00 Nm, mean_abs=9.919264e-01 Nm |
| PASS | SEA_Knee algebraic motor state | motor_angle is not algebraically constrained; max_abs=1.140210e-01 rad, rms=6.177657e-03 rad, mean_abs=3.967705e-03 rad |
| PASS | SEA_Knee motor_speed - qdot | motor_speed differs from joint speed; max_abs=4.649176e+01 rad/s, rms=1.936529e+00 rad/s, mean_abs=5.824421e-01 rad/s |
| PASS | SEA_Ankle tau_ref - tau_spring | non-trivial torque tracking error; max_abs=1.097629e+01 Nm, rms=2.910921e+00 Nm, mean_abs=2.090999e+00 Nm |
| PASS | SEA_Ankle algebraic motor state | motor_angle is not algebraically constrained; max_abs=2.195258e-02 rad, rms=5.821842e-03 rad, mean_abs=4.181997e-03 rad |
| PASS | SEA_Ankle motor_speed - qdot | motor_speed differs from joint speed; max_abs=1.382830e+01 rad/s, rms=7.384441e-01 rad/s, mean_abs=3.549667e-01 rad/s |
| PASS | SEA_Knee derivatives | finite plugin derivatives; speed_dot max_abs=5.423014e+04 rad/s^2, rms=2.343959e+03 rad/s^2, mean_abs=3.408592e+02 rad/s^2 |
| PASS | SEA_Ankle derivatives | finite plugin derivatives; speed_dot max_abs=8.333479e+03 rad/s^2, rms=4.183612e+02 rad/s^2, mean_abs=1.027950e+02 rad/s^2 |
| FAIL | SEA_Knee plugin/Python tau_input agreement | plugin output does not match independently recomputed law; max_abs=2.755754e+02 Nm, rms=1.266032e+01 Nm, mean_abs=1.885038e+00 Nm |
| PASS | SEA_Knee tau_error diagnostic | finite tau_ref - tau_spring; max_abs=2.850526e+01 Nm, rms=1.544414e+00 Nm, mean_abs=9.919264e-01 Nm |
| PASS | SEA_Knee motor speed diagnostics | motor_speed max_abs=4.643867e+01 rad/s, rms=3.055446e+00 rad/s, mean_abs=1.812936e+00 rad/s; speed_dot max_abs=5.423014e+04 rad/s^2, rms=2.343959e+03 rad/s^2, mean_abs=3.408592e+02 rad/s^2 |
| WARN | SEA_Knee tau_input saturation | tau_input reaches +/-500 Nm in 1 samples (0.044%); tuning issue, not an interface failure |
| PASS | SEA_Knee saturation source terms | sat median \|tau_ff\|=12.942 Nm, \|outer_PD\|=0.915 Nm, \|inner_prop\|=530.198 Nm, \|inner_damp\|=1.812 Nm; raw excess max_abs=1.108551e+02 Nm, rms=2.391270e+00 Nm, mean_abs=6.080394e-02 Nm; motor numerator max_abs=5.423014e+02 Nm, rms=2.343959e+01 Nm, mean_abs=3.408592e+00 Nm |
| FAIL | SEA_Ankle plugin/Python tau_input agreement | plugin output does not match independently recomputed law; max_abs=5.483671e+01 Nm, rms=2.462062e+00 Nm, mean_abs=4.893714e-01 Nm |
| PASS | SEA_Ankle tau_error diagnostic | finite tau_ref - tau_spring; max_abs=1.097629e+01 Nm, rms=2.910921e+00 Nm, mean_abs=2.090999e+00 Nm |
| PASS | SEA_Ankle motor speed diagnostics | motor_speed max_abs=9.417262e+00 rad/s, rms=9.282671e-01 rad/s, mean_abs=6.656270e-01 rad/s; speed_dot max_abs=8.333479e+03 rad/s^2, rms=4.183612e+02 rad/s^2, mean_abs=1.027950e+02 rad/s^2 |
| PASS | SEA_Ankle tau_input saturation | tau_input never reaches the +/-500 Nm clamp |
| PASS | SEA_Ankle saturation source terms | raw command below clamp; raw max_abs=8.333479e+01 Nm, rms=2.956229e+01 Nm, mean_abs=1.940670e+01 Nm; motor numerator max_abs=8.333479e+01 Nm, rms=4.183612e+00 Nm, mean_abs=1.027950e+00 Nm |
| WARN | pros_knee_angle reserve torque | prosthetic reserve is exactly zero; OK only if SEA path is validated; max_abs=0.000000e+00 Nm, rms=0.000000e+00 Nm, mean_abs=0.000000e+00 Nm |
| WARN | pros_ankle_angle reserve torque | prosthetic reserve is exactly zero; OK only if SEA path is validated; max_abs=0.000000e+00 Nm, rms=0.000000e+00 Nm, mean_abs=0.000000e+00 Nm |
| PASS | pros_knee_angle output vs IK | non-zero tracking error; max_abs=5.270345e+00 deg, rms=2.629818e+00 deg, mean_abs=2.182948e+00 deg |
| PASS | pros_ankle_angle output vs IK | non-zero tracking error; max_abs=1.540720e+01 deg, rms=7.218506e+00 deg, mean_abs=5.343439e+00 deg |
| PASS | pelvis_tx output vs IK | non-zero tracking error; max_abs=2.854670e-03 m, rms=1.309006e-03 m, mean_abs=9.592297e-04 m |
| PASS | pelvis_ty output vs IK | non-zero tracking error; max_abs=4.363600e-03 m, rms=1.968017e-03 m, mean_abs=1.443777e-03 m |
| PASS | pelvis_tz output vs IK | non-zero tracking error; max_abs=1.175508e-02 m, rms=4.743959e-03 m, mean_abs=2.992956e-03 m |
| PASS | pelvis_tilt output vs IK | non-zero tracking error; max_abs=4.114644e-01 deg, rms=1.613819e-01 deg, mean_abs=1.179361e-01 deg |
| PASS | pelvis_list output vs IK | non-zero tracking error; max_abs=3.256801e-01 deg, rms=1.099893e-01 deg, mean_abs=6.829532e-02 deg |
| PASS | pelvis_rotation output vs IK | non-zero tracking error; max_abs=5.030595e-01 deg, rms=2.065853e-01 deg, mean_abs=1.554690e-01 deg |
| PASS | hip_flexion_r output vs IK | non-zero tracking error; max_abs=6.123347e-01 deg, rms=2.352442e-01 deg, mean_abs=1.762821e-01 deg |
| PASS | hip_adduction_r output vs IK | non-zero tracking error; max_abs=3.544291e-01 deg, rms=1.345663e-01 deg, mean_abs=9.882359e-02 deg |
| PASS | hip_rotation_r output vs IK | non-zero tracking error; max_abs=4.459935e-01 deg, rms=1.843170e-01 deg, mean_abs=1.409388e-01 deg |
| PASS | knee_angle_r output vs IK | non-zero tracking error; max_abs=1.040021e+00 deg, rms=2.932464e-01 deg, mean_abs=1.601997e-01 deg |
| PASS | ankle_angle_r output vs IK | non-zero tracking error; max_abs=6.411113e-01 deg, rms=2.292316e-01 deg, mean_abs=1.605487e-01 deg |
| PASS | subtalar_angle_r output vs IK | non-zero tracking error; max_abs=2.553879e-01 deg, rms=1.040474e-01 deg, mean_abs=7.392155e-02 deg |
| PASS | mtp_angle_r output vs IK | non-zero tracking error; max_abs=1.409962e+00 deg, rms=5.314287e-01 deg, mean_abs=4.007013e-01 deg |
| PASS | hip_flexion_l output vs IK | non-zero tracking error; max_abs=1.414033e+00 deg, rms=5.817546e-01 deg, mean_abs=4.383806e-01 deg |
| WARN | hip_adduction_l output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=2.158335e-01 deg, rms=7.473350e-02 deg, mean_abs=5.076684e-02 deg |
| PASS | hip_rotation_l output vs IK | non-zero tracking error; max_abs=2.968885e-01 deg, rms=1.249161e-01 deg, mean_abs=9.406157e-02 deg |
| PASS | lumbar_extension output vs IK | non-zero tracking error; max_abs=4.090710e-01 deg, rms=1.574161e-01 deg, mean_abs=1.102222e-01 deg |
| PASS | lumbar_bending output vs IK | non-zero tracking error; max_abs=4.741356e-01 deg, rms=1.651363e-01 deg, mean_abs=1.041950e-01 deg |
| PASS | lumbar_rotation output vs IK | non-zero tracking error; max_abs=3.743729e-01 deg, rms=1.482111e-01 deg, mean_abs=1.187492e-01 deg |
| PASS | SEA_Knee control saturation | SEA control below saturation; max \|u\|=0.308661 |
| PASS | SEA_Knee motor vs joint power | motor-joint diff max_abs=1.513186e+04 W, rms=4.035358e+02 W, mean_abs=2.713598e+01 W, corr=0.018183 |
| PASS | SEA_Ankle control saturation | SEA control below saturation; max \|u\|=0.290059 |
| PASS | SEA_Ankle motor vs joint power | motor-joint diff max_abs=5.005875e+02 W, rms=2.027730e+01 W, mean_abs=9.340536e+00 W, corr=0.822265 |

## Interpretazione

Almeno un controllo critico indica che il risultato non e validabile.
Un FAIL su `run status` indica output parziali; un FAIL su tracking
indica divergenza dinamica; un FAIL sulle metriche SEA indica possibile
tautologia o derivate plugin non finite.
