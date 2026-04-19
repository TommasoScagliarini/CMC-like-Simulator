# Validazione simulatore - 2026-04-17 19:57

Risultato complessivo: **WARN**

- Results dir: `C:\Users\tomma\Desktop\Opensim OMNIBUS\CMC-like-Simulator - Claude\results\_sea_driver_sweep_20260417_181301\runs\full_kk750_ka500_wn900_z0p7`
- Reference: `C:\Users\tomma\Desktop\Opensim OMNIBUS\CMC-like-Simulator - Claude\data\3DGaitModel2392_Kinematics_q.sto`

## Checks

| Status | Check | Detail |
|---|---|---|
| PASS | run status | simulation complete at t=11.06 |
| PASS | SEA_Knee tau_ref - tau_spring | non-trivial torque tracking error; max_abs=2.805141e+01 Nm, rms=2.359245e+00 Nm, mean_abs=1.754586e+00 Nm |
| PASS | SEA_Knee algebraic motor state | motor_angle is not algebraically constrained; max_abs=3.740187e-02 rad, rms=3.145660e-03 rad, mean_abs=2.339448e-03 rad |
| PASS | SEA_Knee motor_speed - qdot | motor_speed differs from joint speed; max_abs=1.488593e+01 rad/s, rms=3.909541e-01 rad/s, mean_abs=1.606387e-01 rad/s |
| PASS | SEA_Ankle tau_ref - tau_spring | non-trivial torque tracking error; max_abs=1.040813e+01 Nm, rms=1.805888e+00 Nm, mean_abs=1.244738e+00 Nm |
| PASS | SEA_Ankle algebraic motor state | motor_angle is not algebraically constrained; max_abs=2.081627e-02 rad, rms=3.611777e-03 rad, mean_abs=2.489476e-03 rad |
| PASS | SEA_Ankle motor_speed - qdot | motor_speed differs from joint speed; max_abs=1.224891e+01 rad/s, rms=6.046132e-01 rad/s, mean_abs=3.293610e-01 rad/s |
| PASS | SEA_Knee derivatives | finite plugin derivatives; speed_dot max_abs=2.794043e+04 rad/s^2, rms=6.831905e+02 rad/s^2, mean_abs=1.047859e+02 rad/s^2 |
| PASS | SEA_Ankle derivatives | finite plugin derivatives; speed_dot max_abs=1.439419e+04 rad/s^2, rms=3.852769e+02 rad/s^2, mean_abs=1.634758e+02 rad/s^2 |
| PASS | SEA_Knee plugin/Python tau_input agreement | plugin output matches independently recomputed SEA law; max_abs=4.940000e-06 Nm, rms=1.864685e-07 Nm, mean_abs=8.905147e-08 Nm |
| PASS | SEA_Knee tau_error diagnostic | finite tau_ref - tau_spring; max_abs=2.805141e+01 Nm, rms=2.359245e+00 Nm, mean_abs=1.754586e+00 Nm |
| PASS | SEA_Knee motor speed diagnostics | motor_speed max_abs=1.701054e+01 rad/s, rms=2.001456e+00 rad/s, mean_abs=1.376184e+00 rad/s; speed_dot max_abs=2.794043e+04 rad/s^2, rms=6.831905e+02 rad/s^2, mean_abs=1.047859e+02 rad/s^2 |
| PASS | SEA_Knee tau_input saturation | tau_input never reaches the +/-500 Nm clamp |
| PASS | SEA_Knee saturation source terms | raw command below clamp; raw max_abs=2.839338e+02 Nm, rms=1.138204e+01 Nm, mean_abs=7.369856e+00 Nm; motor numerator max_abs=2.794043e+02 Nm, rms=6.831905e+00 Nm, mean_abs=1.047859e+00 Nm |
| PASS | SEA_Ankle plugin/Python tau_input agreement | plugin output matches independently recomputed SEA law; max_abs=3.100000e-06 Nm, rms=1.914772e-07 Nm, mean_abs=1.102765e-07 Nm |
| PASS | SEA_Ankle tau_error diagnostic | finite tau_ref - tau_spring; max_abs=1.040813e+01 Nm, rms=1.805888e+00 Nm, mean_abs=1.244738e+00 Nm |
| PASS | SEA_Ankle motor speed diagnostics | motor_speed max_abs=7.976290e+00 rad/s, rms=6.697906e-01 rad/s, mean_abs=4.815822e-01 rad/s; speed_dot max_abs=1.439419e+04 rad/s^2, rms=3.852769e+02 rad/s^2, mean_abs=1.634758e+02 rad/s^2 |
| PASS | SEA_Ankle tau_input saturation | tau_input never reaches the +/-500 Nm clamp |
| PASS | SEA_Ankle saturation source terms | raw command below clamp; raw max_abs=1.439419e+02 Nm, rms=3.053243e+01 Nm, mean_abs=1.870549e+01 Nm; motor numerator max_abs=1.439419e+02 Nm, rms=3.852769e+00 Nm, mean_abs=1.634758e+00 Nm |
| WARN | pros_knee_angle reserve torque | prosthetic reserve is exactly zero; OK only if SEA path is validated; max_abs=0.000000e+00 Nm, rms=0.000000e+00 Nm, mean_abs=0.000000e+00 Nm |
| WARN | pros_ankle_angle reserve torque | prosthetic reserve is exactly zero; OK only if SEA path is validated; max_abs=0.000000e+00 Nm, rms=0.000000e+00 Nm, mean_abs=0.000000e+00 Nm |
| PASS | pros_knee_angle output vs IK | non-zero tracking error; max_abs=1.195909e+01 deg, rms=5.424257e+00 deg, mean_abs=4.396967e+00 deg |
| PASS | pros_ankle_angle output vs IK | non-zero tracking error; max_abs=9.782524e+00 deg, rms=4.469892e+00 deg, mean_abs=3.197552e+00 deg |
| PASS | pelvis_tx output vs IK | non-zero tracking error; max_abs=3.249400e-03 m, rms=1.120569e-03 m, mean_abs=8.187157e-04 m |
| PASS | pelvis_ty output vs IK | non-zero tracking error; max_abs=4.414620e-03 m, rms=1.633723e-03 m, mean_abs=1.148386e-03 m |
| PASS | pelvis_tz output vs IK | non-zero tracking error; max_abs=1.192151e-02 m, rms=2.946532e-03 m, mean_abs=1.592819e-03 m |
| PASS | pelvis_tilt output vs IK | non-zero tracking error; max_abs=8.229285e-01 deg, rms=3.138703e-01 deg, mean_abs=2.181078e-01 deg |
| PASS | pelvis_list output vs IK | non-zero tracking error; max_abs=6.170901e-01 deg, rms=2.029250e-01 deg, mean_abs=1.264117e-01 deg |
| PASS | pelvis_rotation output vs IK | non-zero tracking error; max_abs=1.074006e+00 deg, rms=4.081321e-01 deg, mean_abs=2.880554e-01 deg |
| PASS | hip_flexion_r output vs IK | non-zero tracking error; max_abs=8.933547e-01 deg, rms=3.478565e-01 deg, mean_abs=2.495788e-01 deg |
| PASS | hip_adduction_r output vs IK | non-zero tracking error; max_abs=5.997018e-01 deg, rms=2.188341e-01 deg, mean_abs=1.663629e-01 deg |
| PASS | hip_rotation_r output vs IK | non-zero tracking error; max_abs=9.173390e-01 deg, rms=3.478772e-01 deg, mean_abs=2.455988e-01 deg |
| PASS | knee_angle_r output vs IK | non-zero tracking error; max_abs=1.045886e+00 deg, rms=1.837797e-01 deg, mean_abs=9.788153e-02 deg |
| PASS | ankle_angle_r output vs IK | non-zero tracking error; max_abs=6.430948e-01 deg, rms=2.154321e-01 deg, mean_abs=1.583495e-01 deg |
| PASS | subtalar_angle_r output vs IK | non-zero tracking error; max_abs=3.624509e-01 deg, rms=1.084037e-01 deg, mean_abs=7.852478e-02 deg |
| PASS | mtp_angle_r output vs IK | non-zero tracking error; max_abs=1.572903e+00 deg, rms=6.228075e-01 deg, mean_abs=4.760305e-01 deg |
| PASS | hip_flexion_l output vs IK | non-zero tracking error; max_abs=2.824461e+00 deg, rms=1.131529e+00 deg, mean_abs=8.237274e-01 deg |
| PASS | hip_adduction_l output vs IK | non-zero tracking error; max_abs=3.971171e-01 deg, rms=1.263148e-01 deg, mean_abs=8.226433e-02 deg |
| PASS | hip_rotation_l output vs IK | non-zero tracking error; max_abs=5.735483e-01 deg, rms=2.365196e-01 deg, mean_abs=1.725608e-01 deg |
| PASS | lumbar_extension output vs IK | non-zero tracking error; max_abs=8.417859e-01 deg, rms=3.111862e-01 deg, mean_abs=2.100531e-01 deg |
| PASS | lumbar_bending output vs IK | non-zero tracking error; max_abs=9.071091e-01 deg, rms=3.118885e-01 deg, mean_abs=1.966750e-01 deg |
| PASS | lumbar_rotation output vs IK | non-zero tracking error; max_abs=6.472174e-01 deg, rms=2.699162e-01 deg, mean_abs=2.046233e-01 deg |
| PASS | SEA_Knee control saturation | SEA control below saturation; max \|u\|=0.316407 |
| PASS | SEA_Knee motor vs joint power | motor-joint diff max_abs=4.191275e+03 W, rms=5.977035e+01 W, mean_abs=3.521724e+00 W, corr=0.213076 |
| PASS | SEA_Ankle control saturation | SEA control below saturation; max \|u\|=0.301741 |
| PASS | SEA_Ankle motor vs joint power | motor-joint diff max_abs=5.024535e+02 W, rms=1.733188e+01 W, mean_abs=8.734134e+00 W, corr=0.822457 |
