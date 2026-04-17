# Validazione simulatore - 2026-04-16 19:38

Risultato complessivo: **WARN**

- Results dir: `C:\Users\tomma\Desktop\Opensim OMNIBUS\CMC-like-Simulator - Claude\results\_tune_k18p5d10_a35d20_full`
- Reference: `C:\Users\tomma\Desktop\Opensim OMNIBUS\CMC-like-Simulator - Claude\data\3DGaitModel2392_Kinematics_q.sto`

## Checks

| Status | Check | Detail |
|---|---|---|
| PASS | run status | simulation complete at t=11.06 |
| PASS | SEA_Knee tau_ref - tau_spring | non-trivial torque tracking error; max_abs=2.728351e+01 Nm, rms=1.144282e+00 Nm, mean_abs=7.624067e-01 Nm |
| PASS | SEA_Knee algebraic motor state | motor_angle is not algebraically constrained; max_abs=1.091340e-01 rad, rms=4.577129e-03 rad, mean_abs=3.049627e-03 rad |
| PASS | SEA_Knee motor_speed - qdot | motor_speed differs from joint speed; max_abs=2.972127e+01 rad/s, rms=9.882581e-01 rad/s, mean_abs=4.830796e-01 rad/s |
| PASS | SEA_Ankle tau_ref - tau_spring | non-trivial torque tracking error; max_abs=9.469862e+00 Nm, rms=7.742691e-01 Nm, mean_abs=5.861302e-01 Nm |
| PASS | SEA_Ankle algebraic motor state | motor_angle is not algebraically constrained; max_abs=1.893972e-02 rad, rms=1.548538e-03 rad, mean_abs=1.172260e-03 rad |
| PASS | SEA_Ankle motor_speed - qdot | motor_speed differs from joint speed; max_abs=1.280093e+01 rad/s, rms=4.660801e-01 rad/s, mean_abs=2.665750e-01 rad/s |
| PASS | SEA_Knee derivatives | finite plugin derivatives; speed_dot max_abs=4.783530e+04 rad/s^2, rms=1.205836e+03 rad/s^2, mean_abs=1.827814e+02 rad/s^2 |
| PASS | SEA_Ankle derivatives | finite plugin derivatives; speed_dot max_abs=3.314452e+04 rad/s^2, rms=7.316466e+02 rad/s^2, mean_abs=3.532687e+02 rad/s^2 |
| PASS | SEA_Knee plugin/Python tau_input agreement | plugin output matches independently recomputed SEA law; max_abs=4.760000e-06 Nm, rms=1.992777e-07 Nm, mean_abs=8.691912e-08 Nm |
| PASS | SEA_Knee tau_error diagnostic | finite tau_ref - tau_spring; max_abs=2.728351e+01 Nm, rms=1.144282e+00 Nm, mean_abs=7.624067e-01 Nm |
| PASS | SEA_Knee motor speed diagnostics | motor_speed max_abs=3.433648e+01 rad/s, rms=2.169079e+00 rad/s, mean_abs=1.452336e+00 rad/s; speed_dot max_abs=4.783530e+04 rad/s^2, rms=1.205836e+03 rad/s^2, mean_abs=1.827814e+02 rad/s^2 |
| PASS | SEA_Knee tau_input saturation | tau_input never reaches the +/-500 Nm clamp |
| PASS | SEA_Knee saturation source terms | raw command below clamp; raw max_abs=4.654119e+02 Nm, rms=1.485571e+01 Nm, mean_abs=7.940624e+00 Nm; motor numerator max_abs=4.783530e+02 Nm, rms=1.205836e+01 Nm, mean_abs=1.827814e+00 Nm |
| PASS | SEA_Ankle plugin/Python tau_input agreement | plugin output matches independently recomputed SEA law; max_abs=5.000000e-07 Nm, rms=1.931664e-07 Nm, mean_abs=1.146868e-07 Nm |
| PASS | SEA_Ankle tau_error diagnostic | finite tau_ref - tau_spring; max_abs=9.469862e+00 Nm, rms=7.742691e-01 Nm, mean_abs=5.861302e-01 Nm |
| PASS | SEA_Ankle motor speed diagnostics | motor_speed max_abs=9.941405e+00 rad/s, rms=7.604531e-01 rad/s, mean_abs=5.901811e-01 rad/s; speed_dot max_abs=3.314452e+04 rad/s^2, rms=7.316466e+02 rad/s^2, mean_abs=3.532687e+02 rad/s^2 |
| PASS | SEA_Ankle tau_input saturation | tau_input never reaches the +/-500 Nm clamp |
| PASS | SEA_Ankle saturation source terms | raw command below clamp; raw max_abs=3.314452e+02 Nm, rms=3.412691e+01 Nm, mean_abs=2.110309e+01 Nm; motor numerator max_abs=3.314452e+02 Nm, rms=7.316466e+00 Nm, mean_abs=3.532687e+00 Nm |
| WARN | pros_knee_angle reserve torque | prosthetic reserve is exactly zero; OK only if SEA path is validated; max_abs=0.000000e+00 Nm, rms=0.000000e+00 Nm, mean_abs=0.000000e+00 Nm |
| WARN | pros_ankle_angle reserve torque | prosthetic reserve is exactly zero; OK only if SEA path is validated; max_abs=0.000000e+00 Nm, rms=0.000000e+00 Nm, mean_abs=0.000000e+00 Nm |
| PASS | pros_knee_angle output vs IK | non-zero tracking error; max_abs=1.697788e+01 deg, rms=7.957464e+00 deg, mean_abs=6.808829e+00 deg |
| PASS | pros_ankle_angle output vs IK | non-zero tracking error; max_abs=1.871167e+01 deg, rms=8.087823e+00 deg, mean_abs=6.532155e+00 deg |
| PASS | pelvis_tx output vs IK | non-zero tracking error; max_abs=3.660380e-03 m, rms=1.255328e-03 m, mean_abs=9.170876e-04 m |
| PASS | pelvis_ty output vs IK | non-zero tracking error; max_abs=4.777110e-03 m, rms=1.759658e-03 m, mean_abs=1.278488e-03 m |
| PASS | pelvis_tz output vs IK | non-zero tracking error; max_abs=1.170523e-02 m, rms=2.899536e-03 m, mean_abs=1.684895e-03 m |
| PASS | pelvis_tilt output vs IK | non-zero tracking error; max_abs=8.902446e-01 deg, rms=3.430796e-01 deg, mean_abs=2.490353e-01 deg |
| PASS | pelvis_list output vs IK | non-zero tracking error; max_abs=6.637968e-01 deg, rms=2.248654e-01 deg, mean_abs=1.438188e-01 deg |
| PASS | pelvis_rotation output vs IK | non-zero tracking error; max_abs=1.156634e+00 deg, rms=4.484111e-01 deg, mean_abs=3.298939e-01 deg |
| PASS | hip_flexion_r output vs IK | non-zero tracking error; max_abs=9.613033e-01 deg, rms=3.742523e-01 deg, mean_abs=2.747510e-01 deg |
| PASS | hip_adduction_r output vs IK | non-zero tracking error; max_abs=7.637256e-01 deg, rms=2.704040e-01 deg, mean_abs=2.018905e-01 deg |
| PASS | hip_rotation_r output vs IK | non-zero tracking error; max_abs=9.418596e-01 deg, rms=3.660126e-01 deg, mean_abs=2.675388e-01 deg |
| PASS | knee_angle_r output vs IK | non-zero tracking error; max_abs=1.017953e+00 deg, rms=1.817835e-01 deg, mean_abs=9.886731e-02 deg |
| PASS | ankle_angle_r output vs IK | non-zero tracking error; max_abs=6.692365e-01 deg, rms=2.219823e-01 deg, mean_abs=1.640759e-01 deg |
| PASS | subtalar_angle_r output vs IK | non-zero tracking error; max_abs=3.768946e-01 deg, rms=1.213245e-01 deg, mean_abs=8.999917e-02 deg |
| PASS | mtp_angle_r output vs IK | non-zero tracking error; max_abs=1.597156e+00 deg, rms=6.271001e-01 deg, mean_abs=4.827172e-01 deg |
| PASS | hip_flexion_l output vs IK | non-zero tracking error; max_abs=3.248776e+00 deg, rms=1.287704e+00 deg, mean_abs=9.681316e-01 deg |
| PASS | hip_adduction_l output vs IK | non-zero tracking error; max_abs=4.207097e-01 deg, rms=1.383525e-01 deg, mean_abs=9.574721e-02 deg |
| PASS | hip_rotation_l output vs IK | non-zero tracking error; max_abs=6.071628e-01 deg, rms=2.612730e-01 deg, mean_abs=2.026257e-01 deg |
| PASS | lumbar_extension output vs IK | non-zero tracking error; max_abs=8.862735e-01 deg, rms=3.360356e-01 deg, mean_abs=2.419597e-01 deg |
| PASS | lumbar_bending output vs IK | non-zero tracking error; max_abs=9.808110e-01 deg, rms=3.465400e-01 deg, mean_abs=2.328464e-01 deg |
| PASS | lumbar_rotation output vs IK | non-zero tracking error; max_abs=6.900733e-01 deg, rms=2.960583e-01 deg, mean_abs=2.325701e-01 deg |
| PASS | SEA_Knee control saturation | SEA control below saturation; max \|u\|=0.279480 |
| PASS | SEA_Knee motor vs joint power | motor-joint diff max_abs=1.231824e+04 W, rms=1.755426e+02 W, mean_abs=9.789733e+00 W, corr=0.078773 |
| PASS | SEA_Ankle control saturation | SEA control below saturation; max \|u\|=0.358911 |
| PASS | SEA_Ankle motor vs joint power | motor-joint diff max_abs=3.064395e+02 W, rms=1.577858e+01 W, mean_abs=8.188522e+00 W, corr=0.919811 |
