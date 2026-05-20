# Validazione simulatore - 2026-05-14 20:40

Risultato complessivo: **FAIL**

- Results dir: `/Users/tommy/Documents/CMC-like-Simulator - Claude/results/_stiff321_500_matched_dynamics_ab06_pd_full`
- Reference: `/Users/tommy/Documents/CMC-like-Simulator - Claude/models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot`

## Checks

| Status | Check | Detail |
|---|---|---|
| PASS | run status | simulation complete at t=21 |
| PASS | SEA_Knee tau_ref - tau_spring | non-trivial torque tracking error; max_abs=4.568493e+00 Nm, rms=1.201797e+00 Nm, mean_abs=8.335622e-01 Nm |
| PASS | SEA_Knee algebraic motor state | motor_angle is not algebraically constrained; max_abs=1.423207e-02 rad, rms=3.743915e-03 rad, mean_abs=2.596767e-03 rad |
| PASS | SEA_Knee motor_speed - qdot | motor_speed differs from joint speed; max_abs=3.605715e+00 rad/s, rms=4.602597e-01 rad/s, mean_abs=2.970102e-01 rad/s |
| PASS | SEA_Ankle tau_ref - tau_spring | non-trivial torque tracking error; max_abs=3.592174e+00 Nm, rms=7.041884e-01 Nm, mean_abs=4.905157e-01 Nm |
| PASS | SEA_Ankle algebraic motor state | motor_angle is not algebraically constrained; max_abs=7.184345e-03 rad, rms=1.408377e-03 rad, mean_abs=9.810315e-04 rad |
| PASS | SEA_Ankle motor_speed - qdot | motor_speed differs from joint speed; max_abs=8.502958e+00 rad/s, rms=4.707210e-01 rad/s, mean_abs=2.724669e-01 rad/s |
| PASS | SEA_Knee derivatives | finite plugin derivatives; speed_dot max_abs=1.618866e+03 rad/s^2, rms=1.756730e+02 rad/s^2, mean_abs=1.128765e+02 rad/s^2 |
| PASS | SEA_Ankle derivatives | finite plugin derivatives; speed_dot max_abs=3.421653e+03 rad/s^2, rms=1.710546e+02 rad/s^2, mean_abs=1.035910e+02 rad/s^2 |
| PASS | SEA_Knee plugin/Python tau_input agreement | plugin output matches independently recomputed SEA law; max_abs=5.000000e-07 Nm, rms=1.534705e-07 Nm, mean_abs=8.569367e-08 Nm |
| PASS | SEA_Knee tau_error diagnostic | finite tau_ref - tau_spring; max_abs=4.568494e+00 Nm, rms=1.201797e+00 Nm, mean_abs=8.335622e-01 Nm |
| PASS | SEA_Knee motor speed diagnostics | motor_speed max_abs=5.469678e+00 rad/s, rms=1.611988e+00 rad/s, mean_abs=1.109266e+00 rad/s; speed_dot max_abs=1.618866e+03 rad/s^2, rms=1.756730e+02 rad/s^2, mean_abs=1.128765e+02 rad/s^2 |
| PASS | SEA_Knee tau_input saturation | tau_input never reaches the +/-500 Nm clamp |
| PASS | SEA_Knee saturation source terms | raw command below clamp; raw max_abs=3.195186e+01 Nm, rms=9.980386e+00 Nm, mean_abs=7.762708e+00 Nm; motor numerator max_abs=1.618866e+01 Nm, rms=1.756730e+00 Nm, mean_abs=1.128765e+00 Nm |
| PASS | SEA_Ankle plugin/Python tau_input agreement | plugin output matches independently recomputed SEA law; max_abs=5.000000e-07 Nm, rms=2.175970e-07 Nm, mean_abs=1.468080e-07 Nm |
| PASS | SEA_Ankle tau_error diagnostic | finite tau_ref - tau_spring; max_abs=3.592175e+00 Nm, rms=7.041884e-01 Nm, mean_abs=4.905157e-01 Nm |
| PASS | SEA_Ankle motor speed diagnostics | motor_speed max_abs=4.178495e+00 rad/s, rms=1.046891e+00 rad/s, mean_abs=7.424924e-01 rad/s; speed_dot max_abs=3.421653e+03 rad/s^2, rms=1.710546e+02 rad/s^2, mean_abs=1.035910e+02 rad/s^2 |
| PASS | SEA_Ankle tau_input saturation | tau_input never reaches the +/-500 Nm clamp |
| PASS | SEA_Ankle saturation source terms | raw command below clamp; raw max_abs=9.843909e+01 Nm, rms=3.997393e+01 Nm, mean_abs=2.830115e+01 Nm; motor numerator max_abs=3.421653e+01 Nm, rms=1.710546e+00 Nm, mean_abs=1.035910e+00 Nm |
| WARN | pros_knee_angle reserve torque | prosthetic reserve is exactly zero; OK only if SEA path is validated; max_abs=0.000000e+00 Nm, rms=0.000000e+00 Nm, mean_abs=0.000000e+00 Nm |
| WARN | pros_ankle_angle reserve torque | prosthetic reserve is exactly zero; OK only if SEA path is validated; max_abs=0.000000e+00 Nm, rms=0.000000e+00 Nm, mean_abs=0.000000e+00 Nm |
| PASS | pros_knee_angle output vs IK | non-zero tracking error; max_abs=9.595894e+00 deg, rms=3.089004e+00 deg, mean_abs=2.311422e+00 deg |
| PASS | pros_ankle_angle output vs IK | non-zero tracking error; max_abs=1.316377e+01 deg, rms=5.432719e+00 deg, mean_abs=3.897173e+00 deg |
| WARN | pelvis_tx output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=4.440220e-03 m, rms=5.777738e-04 m, mean_abs=4.333306e-04 m |
| WARN | pelvis_ty output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=3.768820e-03 m, rms=7.879548e-04 m, mean_abs=6.563193e-04 m |
| WARN | pelvis_tz output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=5.848120e-03 m, rms=5.860072e-04 m, mean_abs=4.567394e-04 m |
| PASS | pelvis_tilt output vs IK | non-zero tracking error; max_abs=2.743780e+00 deg, rms=1.847147e-01 deg, mean_abs=1.230114e-01 deg |
| PASS | pelvis_list output vs IK | non-zero tracking error; max_abs=3.758464e+00 deg, rms=3.333145e-01 deg, mean_abs=2.038935e-01 deg |
| PASS | pelvis_rotation output vs IK | non-zero tracking error; max_abs=4.248594e+00 deg, rms=2.822486e-01 deg, mean_abs=1.741858e-01 deg |
| PASS | hip_flexion_r output vs IK | non-zero tracking error; max_abs=3.531681e+00 deg, rms=3.679699e-01 deg, mean_abs=2.563269e-01 deg |
| PASS | hip_adduction_r output vs IK | non-zero tracking error; max_abs=4.064864e+00 deg, rms=2.055909e-01 deg, mean_abs=1.182890e-01 deg |
| PASS | hip_rotation_r output vs IK | non-zero tracking error; max_abs=4.962842e+00 deg, rms=3.683006e-01 deg, mean_abs=2.976520e-01 deg |
| PASS | knee_angle_r output vs IK | non-zero tracking error; max_abs=1.780454e+00 deg, rms=2.289691e-01 deg, mean_abs=1.709311e-01 deg |
| PASS | ankle_angle_r output vs IK | non-zero tracking error; max_abs=9.714366e+00 deg, rms=3.871717e+00 deg, mean_abs=3.171732e+00 deg |
| PASS | subtalar_angle_r output vs IK | non-zero tracking error; max_abs=1.005809e+01 deg, rms=2.795537e+00 deg, mean_abs=1.977473e+00 deg |
| FAIL | mtp_angle_r output vs IK | tracking error exceeds stability threshold; max_abs=2.982732e+01 deg, rms=1.409645e+01 deg, mean_abs=1.167932e+01 deg |
| PASS | hip_flexion_l output vs IK | non-zero tracking error; max_abs=3.999077e+00 deg, rms=6.543520e-01 deg, mean_abs=5.121250e-01 deg |
| PASS | hip_adduction_l output vs IK | non-zero tracking error; max_abs=2.630590e+00 deg, rms=1.934294e-01 deg, mean_abs=1.336689e-01 deg |
| PASS | hip_rotation_l output vs IK | non-zero tracking error; max_abs=5.544268e+00 deg, rms=4.106811e-01 deg, mean_abs=2.508820e-01 deg |
| PASS | lumbar_extension output vs IK | non-zero tracking error; max_abs=5.264084e-01 deg, rms=1.928835e-01 deg, mean_abs=1.619118e-01 deg |
| PASS | lumbar_bending output vs IK | non-zero tracking error; max_abs=2.744668e-01 deg, rms=1.212427e-01 deg, mean_abs=1.027717e-01 deg |
| PASS | lumbar_rotation output vs IK | non-zero tracking error; max_abs=5.626531e-01 deg, rms=1.770782e-01 deg, mean_abs=1.415490e-01 deg |
| PASS | SEA_Knee control saturation | SEA control below saturation; max \|u\|=0.299940 |
| PASS | SEA_Knee motor vs joint power | motor-joint diff max_abs=6.713552e+01 W, rms=5.961320e+00 W, mean_abs=2.910630e+00 W, corr=0.845156 |
| PASS | SEA_Ankle control saturation | SEA control below saturation; max \|u\|=0.388890 |
| PASS | SEA_Ankle motor vs joint power | motor-joint diff max_abs=1.666394e+02 W, rms=1.920924e+01 W, mean_abs=1.010588e+01 W, corr=0.958383 |

## Interpretazione

Almeno un controllo critico indica che il risultato non e validabile.
Un FAIL su `run status` indica output parziali; un FAIL su tracking
indica divergenza dinamica; un FAIL sulle metriche SEA indica possibile
tautologia o derivate plugin non finite.
