# Validazione simulatore - 2026-04-17 15:59

Risultato complessivo: **WARN**

- Results dir: `/Users/tommy/Documents/CMC-like-Simulator - Claude/results/_sea_driver_sweep_quick_smoke/runs/smoke_wn700`
- Reference: `/Users/tommy/Documents/CMC-like-Simulator - Claude/data/3DGaitModel2392_Kinematics_q.sto`

## Checks

| Status | Check | Detail |
|---|---|---|
| PASS | run status | simulation complete at t=4.3 |
| PASS | SEA_Knee tau_ref - tau_spring | non-trivial torque tracking error; max_abs=6.426865e+00 Nm, rms=1.550292e+00 Nm, mean_abs=7.618723e-01 Nm |
| PASS | SEA_Knee algebraic motor state | motor_angle is not algebraically constrained; max_abs=2.570746e-02 rad, rms=6.201168e-03 rad, mean_abs=3.047490e-03 rad |
| PASS | SEA_Knee motor_speed - qdot | motor_speed differs from joint speed; max_abs=6.899622e+00 rad/s, rms=1.968131e+00 rad/s, mean_abs=1.201536e+00 rad/s |
| PASS | SEA_Ankle tau_ref - tau_spring | non-trivial torque tracking error; max_abs=1.151181e+01 Nm, rms=5.210414e+00 Nm, mean_abs=4.336770e+00 Nm |
| PASS | SEA_Ankle algebraic motor state | motor_angle is not algebraically constrained; max_abs=2.302363e-02 rad, rms=1.042083e-02 rad, mean_abs=8.673538e-03 rad |
| PASS | SEA_Ankle motor_speed - qdot | motor_speed differs from joint speed; max_abs=1.041640e+01 rad/s, rms=4.832622e+00 rad/s, mean_abs=4.018905e+00 rad/s |
| PASS | SEA_Knee derivatives | finite plugin derivatives; speed_dot max_abs=1.195397e+04 rad/s^2, rms=2.045287e+03 rad/s^2, mean_abs=7.680562e+02 rad/s^2 |
| PASS | SEA_Ankle derivatives | finite plugin derivatives; speed_dot max_abs=8.333479e+03 rad/s^2, rms=2.704625e+03 rad/s^2, mean_abs=2.162985e+03 rad/s^2 |
| PASS | SEA_Knee plugin/Python tau_input agreement | plugin output matches independently recomputed SEA law; max_abs=4.630000e-06 Nm, rms=7.372364e-07 Nm, mean_abs=1.662500e-07 Nm |
| PASS | SEA_Knee tau_error diagnostic | finite tau_ref - tau_spring; max_abs=6.426865e+00 Nm, rms=1.550292e+00 Nm, mean_abs=7.618722e-01 Nm |
| PASS | SEA_Knee motor speed diagnostics | motor_speed max_abs=6.872812e+00 rad/s, rms=1.891796e+00 rad/s, mean_abs=1.041527e+00 rad/s; speed_dot max_abs=1.195397e+04 rad/s^2, rms=2.045287e+03 rad/s^2, mean_abs=7.680562e+02 rad/s^2 |
| PASS | SEA_Knee tau_input saturation | tau_input never reaches the +/-500 Nm clamp |
| PASS | SEA_Knee saturation source terms | raw command below clamp; raw max_abs=1.195397e+02 Nm, rms=1.991099e+01 Nm, mean_abs=7.017076e+00 Nm; motor numerator max_abs=1.195397e+02 Nm, rms=2.045287e+01 Nm, mean_abs=7.680562e+00 Nm |
| PASS | SEA_Ankle plugin/Python tau_input agreement | plugin output matches independently recomputed SEA law; max_abs=4.400000e-07 Nm, rms=2.312088e-07 Nm, mean_abs=1.767500e-07 Nm |
| PASS | SEA_Ankle tau_error diagnostic | finite tau_ref - tau_spring; max_abs=1.151181e+01 Nm, rms=5.210414e+00 Nm, mean_abs=4.336770e+00 Nm |
| PASS | SEA_Ankle motor speed diagnostics | motor_speed max_abs=5.870882e+00 rad/s, rms=2.912606e+00 rad/s, mean_abs=2.415956e+00 rad/s; speed_dot max_abs=8.333479e+03 rad/s^2, rms=2.704625e+03 rad/s^2, mean_abs=2.162985e+03 rad/s^2 |
| PASS | SEA_Ankle tau_input saturation | tau_input never reaches the +/-500 Nm clamp |
| PASS | SEA_Ankle saturation source terms | raw command below clamp; raw max_abs=8.333479e+01 Nm, rms=2.629056e+01 Nm, mean_abs=2.080286e+01 Nm; motor numerator max_abs=8.333479e+01 Nm, rms=2.704625e+01 Nm, mean_abs=2.162985e+01 Nm |
| WARN | pros_knee_angle reserve torque | prosthetic reserve is exactly zero; OK only if SEA path is validated; max_abs=0.000000e+00 Nm, rms=0.000000e+00 Nm, mean_abs=0.000000e+00 Nm |
| WARN | pros_ankle_angle reserve torque | prosthetic reserve is exactly zero; OK only if SEA path is validated; max_abs=0.000000e+00 Nm, rms=0.000000e+00 Nm, mean_abs=0.000000e+00 Nm |
| PASS | pros_knee_angle output vs IK | non-zero tracking error; max_abs=2.440904e-01 deg, rms=1.869941e-01 deg, mean_abs=1.740529e-01 deg |
| PASS | pros_ankle_angle output vs IK | non-zero tracking error; max_abs=1.226922e+00 deg, rms=8.233205e-01 deg, mean_abs=7.646189e-01 deg |
| WARN | pelvis_tx output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=2.300000e-05 m, rms=1.822920e-05 m, mean_abs=1.640350e-05 m |
| WARN | pelvis_ty output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=3.945000e-05 m, rms=3.083517e-05 m, mean_abs=2.886250e-05 m |
| WARN | pelvis_tz output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=3.019000e-05 m, rms=2.365098e-05 m, mean_abs=2.172800e-05 m |
| WARN | pelvis_tilt output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=1.635765e-02 deg, rms=1.286900e-02 deg, mean_abs=1.202728e-02 deg |
| WARN | pelvis_list output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=1.041156e-02 deg, rms=7.796774e-03 deg, mean_abs=7.381137e-03 deg |
| WARN | pelvis_rotation output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=2.000104e-02 deg, rms=1.546009e-02 deg, mean_abs=1.429765e-02 deg |
| WARN | hip_flexion_r output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=1.730719e-02 deg, rms=1.363101e-02 deg, mean_abs=1.271796e-02 deg |
| WARN | hip_adduction_r output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=1.726374e-02 deg, rms=1.336353e-02 deg, mean_abs=1.264582e-02 deg |
| WARN | hip_rotation_r output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=1.359190e-02 deg, rms=9.427381e-03 deg, mean_abs=8.370183e-03 deg |
| WARN | knee_angle_r output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=2.071291e-03 deg, rms=1.359826e-03 deg, mean_abs=1.164975e-03 deg |
| WARN | ankle_angle_r output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=9.075397e-03 deg, rms=4.863938e-03 deg, mean_abs=4.142085e-03 deg |
| WARN | subtalar_angle_r output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=4.495601e-03 deg, rms=2.279795e-03 deg, mean_abs=1.922371e-03 deg |
| WARN | mtp_angle_r output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=2.657644e-03 deg, rms=2.080009e-03 deg, mean_abs=1.934307e-03 deg |
| WARN | hip_flexion_l output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=6.824202e-02 deg, rms=5.352308e-02 deg, mean_abs=5.010316e-02 deg |
| WARN | hip_adduction_l output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=9.550382e-03 deg, rms=7.119198e-03 deg, mean_abs=6.745090e-03 deg |
| WARN | hip_rotation_l output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=1.602786e-02 deg, rms=1.255572e-02 deg, mean_abs=1.176783e-02 deg |
| WARN | lumbar_extension output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=1.562683e-02 deg, rms=1.236406e-02 deg, mean_abs=1.160877e-02 deg |
| WARN | lumbar_bending output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=1.677191e-02 deg, rms=1.305165e-02 deg, mean_abs=1.235160e-02 deg |
| WARN | lumbar_rotation output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=1.432852e-02 deg, rms=1.027392e-02 deg, mean_abs=9.255598e-03 deg |
| PASS | SEA_Knee control saturation | SEA control below saturation; max \|u\|=0.067261 |
| PASS | SEA_Knee motor vs joint power | motor-joint diff max_abs=1.330314e+02 W, rms=2.905573e+01 W, mean_abs=1.029988e+01 W, corr=0.172889 |
| PASS | SEA_Ankle control saturation | SEA control below saturation; max \|u\|=0.074808 |
| PASS | SEA_Ankle motor vs joint power | motor-joint diff max_abs=2.243470e+02 W, rms=7.402460e+01 W, mean_abs=5.874268e+01 W, corr=-0.188404 |
