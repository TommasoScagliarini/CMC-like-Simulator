# Validazione simulatore - 2026-04-19 06:26

Risultato complessivo: **FAIL**

- Results dir: `C:\Users\tomma\Desktop\Opensim OMNIBUS\CMC-like-Simulator - Claude\results\_sea_driver_sweep_20260418_213542\runs\screen_kk1000_ka1500_wn1400_z1p5_td0p001`
- Reference: `C:\Users\tomma\Desktop\Opensim OMNIBUS\CMC-like-Simulator - Claude\data\3DGaitModel2392_Kinematics_q.sto`

## Checks

| Status | Check | Detail |
|---|---|---|
| PASS | run status | simulation complete at t=6.55 |
| PASS | SEA_Knee tau_ref - tau_spring | non-trivial torque tracking error; max_abs=1.403640e+02 Nm, rms=8.470101e+01 Nm, mean_abs=7.540017e+01 Nm |
| PASS | SEA_Knee algebraic motor state | motor_angle is not algebraically constrained; max_abs=1.403640e-01 rad, rms=8.470101e-02 rad, mean_abs=7.540017e-02 rad |
| PASS | SEA_Knee motor_speed - qdot | motor_speed differs from joint speed; max_abs=2.817056e+02 rad/s, rms=1.878418e+02 rad/s, mean_abs=1.633810e+02 rad/s |
| PASS | SEA_Ankle tau_ref - tau_spring | non-trivial torque tracking error; max_abs=1.491788e+02 Nm, rms=9.074393e+01 Nm, mean_abs=8.044464e+01 Nm |
| PASS | SEA_Ankle algebraic motor state | motor_angle is not algebraically constrained; max_abs=9.945254e-02 rad, rms=6.049595e-02 rad, mean_abs=5.362976e-02 rad |
| PASS | SEA_Ankle motor_speed - qdot | motor_speed differs from joint speed; max_abs=2.570799e+02 rad/s, rms=1.762470e+02 rad/s, mean_abs=1.556391e+02 rad/s |
| PASS | SEA_Knee derivatives | finite plugin derivatives; speed_dot max_abs=5.152547e+05 rad/s^2, rms=4.207282e+05 rad/s^2, mean_abs=3.834395e+05 rad/s^2 |
| PASS | SEA_Ankle derivatives | finite plugin derivatives; speed_dot max_abs=5.256162e+05 rad/s^2, rms=3.969997e+05 rad/s^2, mean_abs=3.579502e+05 rad/s^2 |
| FAIL | SEA_Knee plugin/Python tau_input agreement | plugin output does not match independently recomputed law; max_abs=5.500000e+03 Nm, rms=3.977411e+03 Nm, mean_abs=3.655574e+03 Nm |
| PASS | SEA_Knee tau_error diagnostic | finite tau_ref - tau_spring; max_abs=1.403640e+02 Nm, rms=8.470101e+01 Nm, mean_abs=7.540017e+01 Nm |
| PASS | SEA_Knee motor speed diagnostics | motor_speed max_abs=2.826456e+02 rad/s, rms=1.874977e+02 rad/s, mean_abs=1.631572e+02 rad/s; speed_dot max_abs=5.152547e+05 rad/s^2, rms=4.207282e+05 rad/s^2, mean_abs=3.834395e+05 rad/s^2 |
| WARN | SEA_Knee tau_input saturation | tau_input reaches +/-500 Nm in 1997 samples (87.205%); tuning issue, not an interface failure |
| PASS | SEA_Knee saturation source terms | sat median \|tau_ff\|=5.823 Nm, \|outer_PD\|=2.790 Nm, \|inner_prop\|=1760.819 Nm, \|inner_damp\|=7752.475 Nm; raw excess max_abs=1.231371e+04 Nm, rms=7.911785e+03 Nm, mean_abs=6.896676e+03 Nm; motor numerator max_abs=5.152547e+03 Nm, rms=4.207282e+03 Nm, mean_abs=3.834395e+03 Nm |
| FAIL | SEA_Ankle plugin/Python tau_input agreement | plugin output does not match independently recomputed law; max_abs=5.500000e+03 Nm, rms=3.745125e+03 Nm, mean_abs=3.377022e+03 Nm |
| PASS | SEA_Ankle tau_error diagnostic | finite tau_ref - tau_spring; max_abs=1.491788e+02 Nm, rms=9.074393e+01 Nm, mean_abs=8.044464e+01 Nm |
| PASS | SEA_Ankle motor speed diagnostics | motor_speed max_abs=2.477693e+02 rad/s, rms=1.676557e+02 rad/s, mean_abs=1.475248e+02 rad/s; speed_dot max_abs=5.256162e+05 rad/s^2, rms=3.969997e+05 rad/s^2, mean_abs=3.579502e+05 rad/s^2 |
| WARN | SEA_Ankle tau_input saturation | tau_input reaches +/-500 Nm in 2138 samples (93.362%); tuning issue, not an interface failure |
| PASS | SEA_Ankle saturation source terms | sat median \|tau_ff\|=3.437 Nm, \|outer_PD\|=25.395 Nm, \|inner_prop\|=1078.751 Nm, \|inner_damp\|=6787.359 Nm; raw excess max_abs=1.027307e+04 Nm, rms=6.744209e+03 Nm, mean_abs=5.801354e+03 Nm; motor numerator max_abs=5.256162e+03 Nm, rms=3.969997e+03 Nm, mean_abs=3.579502e+03 Nm |
| WARN | pros_knee_angle reserve torque | prosthetic reserve is exactly zero; OK only if SEA path is validated; max_abs=0.000000e+00 Nm, rms=0.000000e+00 Nm, mean_abs=0.000000e+00 Nm |
| WARN | pros_ankle_angle reserve torque | prosthetic reserve is exactly zero; OK only if SEA path is validated; max_abs=0.000000e+00 Nm, rms=0.000000e+00 Nm, mean_abs=0.000000e+00 Nm |
| PASS | pros_knee_angle output vs IK | non-zero tracking error; max_abs=1.761489e+01 deg, rms=7.971889e+00 deg, mean_abs=6.310199e+00 deg |
| PASS | pros_ankle_angle output vs IK | non-zero tracking error; max_abs=1.734877e+01 deg, rms=8.294368e+00 deg, mean_abs=6.660406e+00 deg |
| PASS | pelvis_tx output vs IK | non-zero tracking error; max_abs=5.093730e-03 m, rms=2.267110e-03 m, mean_abs=1.790802e-03 m |
| PASS | pelvis_ty output vs IK | non-zero tracking error; max_abs=5.396800e-03 m, rms=2.076492e-03 m, mean_abs=1.590094e-03 m |
| PASS | pelvis_tz output vs IK | non-zero tracking error; max_abs=1.002380e-02 m, rms=3.843072e-03 m, mean_abs=2.602083e-03 m |
| PASS | pelvis_tilt output vs IK | non-zero tracking error; max_abs=8.700312e-01 deg, rms=4.850558e-01 deg, mean_abs=4.218598e-01 deg |
| PASS | pelvis_list output vs IK | non-zero tracking error; max_abs=6.269323e-01 deg, rms=3.004665e-01 deg, mean_abs=2.372212e-01 deg |
| PASS | pelvis_rotation output vs IK | non-zero tracking error; max_abs=1.183355e+00 deg, rms=6.152393e-01 deg, mean_abs=5.272207e-01 deg |
| PASS | hip_flexion_r output vs IK | non-zero tracking error; max_abs=1.006129e+00 deg, rms=5.010241e-01 deg, mean_abs=4.036088e-01 deg |
| PASS | hip_adduction_r output vs IK | non-zero tracking error; max_abs=9.576942e-01 deg, rms=4.312265e-01 deg, mean_abs=3.390014e-01 deg |
| PASS | hip_rotation_r output vs IK | non-zero tracking error; max_abs=1.087685e+00 deg, rms=5.062649e-01 deg, mean_abs=4.246624e-01 deg |
| PASS | knee_angle_r output vs IK | non-zero tracking error; max_abs=9.756324e-01 deg, rms=2.801478e-01 deg, mean_abs=1.643515e-01 deg |
| PASS | ankle_angle_r output vs IK | non-zero tracking error; max_abs=6.471943e-01 deg, rms=2.290065e-01 deg, mean_abs=1.621325e-01 deg |
| PASS | subtalar_angle_r output vs IK | non-zero tracking error; max_abs=3.560678e-01 deg, rms=1.644248e-01 deg, mean_abs=1.296710e-01 deg |
| PASS | mtp_angle_r output vs IK | non-zero tracking error; max_abs=1.414870e+00 deg, rms=5.278880e-01 deg, mean_abs=4.008034e-01 deg |
| PASS | hip_flexion_l output vs IK | non-zero tracking error; max_abs=3.415400e+00 deg, rms=1.855950e+00 deg, mean_abs=1.596631e+00 deg |
| PASS | hip_adduction_l output vs IK | non-zero tracking error; max_abs=5.452966e-01 deg, rms=2.627684e-01 deg, mean_abs=2.169596e-01 deg |
| PASS | hip_rotation_l output vs IK | non-zero tracking error; max_abs=8.029964e-01 deg, rms=3.727541e-01 deg, mean_abs=3.098375e-01 deg |
| PASS | lumbar_extension output vs IK | non-zero tracking error; max_abs=8.612956e-01 deg, rms=4.648970e-01 deg, mean_abs=4.031121e-01 deg |
| PASS | lumbar_bending output vs IK | non-zero tracking error; max_abs=9.575512e-01 deg, rms=4.711914e-01 deg, mean_abs=3.962965e-01 deg |
| PASS | lumbar_rotation output vs IK | non-zero tracking error; max_abs=1.077114e+00 deg, rms=4.348930e-01 deg, mean_abs=3.509815e-01 deg |
| PASS | SEA_Knee control saturation | SEA control below saturation; max \|u\|=0.294608 |
| PASS | SEA_Knee motor vs joint power | motor-joint diff max_abs=1.331498e+06 W, rms=6.246673e+05 W, mean_abs=5.172701e+05 W, corr=0.095150 |
| PASS | SEA_Ankle control saturation | SEA control below saturation; max \|u\|=0.523615 |
| PASS | SEA_Ankle motor vs joint power | motor-joint diff max_abs=8.929185e+05 W, rms=5.004560e+05 W, mean_abs=4.332443e+05 W, corr=0.396209 |

## Interpretazione

Almeno un controllo critico indica che il risultato non e validabile.
Un FAIL su `run status` indica output parziali; un FAIL su tracking
indica divergenza dinamica; un FAIL sulle metriche SEA indica possibile
tautologia o derivate plugin non finite.
