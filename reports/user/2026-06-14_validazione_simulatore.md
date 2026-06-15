# Validazione simulatore - 2026-06-14 18:35

Risultato complessivo: **FAIL**

- Results dir: `C:\Users\tomma\Desktop\Opensim OMNIBUS\CMC-like-Simulator - Claude\results\_windows_pi_alignment_11.99_16`
- Reference: `C:\Users\tomma\Desktop\Opensim OMNIBUS\CMC-like-Simulator - Claude\models\AB06_SEASEA_Threadmill\data\IK_results_AB06_SEASEA.mot`

## Checks

| Status | Check | Detail |
|---|---|---|
| PASS | run status | simulation complete at t=16 |
| PASS | SEA_Knee tau_ref - tau_spring | non-trivial torque tracking error; max_abs=7.853806e+00 Nm, rms=1.089784e+00 Nm, mean_abs=7.031899e-01 Nm |
| PASS | SEA_Knee algebraic motor state | motor_angle is not algebraically constrained; max_abs=2.446668e-02 rad, rms=3.394966e-03 rad, mean_abs=2.190623e-03 rad |
| PASS | SEA_Knee motor_speed - qdot | motor_speed differs from joint speed; max_abs=1.057280e+01 rad/s, rms=1.291217e+00 rad/s, mean_abs=7.283291e-01 rad/s |
| PASS | SEA_Ankle tau_ref - tau_spring | non-trivial torque tracking error; max_abs=1.654208e+01 Nm, rms=1.138305e+00 Nm, mean_abs=5.111651e-01 Nm |
| PASS | SEA_Ankle algebraic motor state | motor_angle is not algebraically constrained; max_abs=3.308415e-02 rad, rms=2.276610e-03 rad, mean_abs=1.022330e-03 rad |
| PASS | SEA_Ankle motor_speed - qdot | motor_speed differs from joint speed; max_abs=1.489543e+01 rad/s, rms=1.171833e+00 rad/s, mean_abs=4.790347e-01 rad/s |
| PASS | SEA_Knee derivatives | finite plugin derivatives; speed_dot max_abs=7.565879e+03 rad/s^2, rms=8.529232e+02 rad/s^2, mean_abs=4.771381e+02 rad/s^2 |
| PASS | SEA_Ankle derivatives | finite plugin derivatives; speed_dot max_abs=1.522830e+04 rad/s^2, rms=7.584539e+02 rad/s^2, mean_abs=2.613204e+02 rad/s^2 |
| PASS | SEA_Knee plugin/Python tau_input agreement | plugin output matches independently recomputed SEA law; max_abs=5.000000e-07 Nm, rms=1.819460e-07 Nm, mean_abs=1.111322e-07 Nm |
| PASS | SEA_Knee tau_error diagnostic | finite tau_ref - tau_spring; max_abs=7.853806e+00 Nm, rms=1.089784e+00 Nm, mean_abs=7.031899e-01 Nm |
| PASS | SEA_Knee motor speed diagnostics | motor_speed max_abs=1.046844e+01 rad/s, rms=1.865531e+00 rad/s, mean_abs=1.286326e+00 rad/s; speed_dot max_abs=7.565879e+03 rad/s^2, rms=8.529232e+02 rad/s^2, mean_abs=4.771381e+02 rad/s^2 |
| PASS | SEA_Knee tau_input saturation | tau_input never reaches the +/-500 Nm clamp |
| PASS | SEA_Knee saturation source terms | raw command below clamp; raw max_abs=6.996700e+01 Nm, rms=1.340743e+01 Nm, mean_abs=1.017866e+01 Nm; motor numerator max_abs=7.565879e+01 Nm, rms=8.529232e+00 Nm, mean_abs=4.771381e+00 Nm |
| PASS | SEA_Ankle plugin/Python tau_input agreement | plugin output matches independently recomputed SEA law; max_abs=5.000000e-06 Nm, rms=8.520777e-07 Nm, mean_abs=3.462594e-07 Nm |
| PASS | SEA_Ankle tau_error diagnostic | finite tau_ref - tau_spring; max_abs=1.654208e+01 Nm, rms=1.138305e+00 Nm, mean_abs=5.111651e-01 Nm |
| PASS | SEA_Ankle motor speed diagnostics | motor_speed max_abs=1.115642e+01 rad/s, rms=1.026342e+00 rad/s, mean_abs=6.064532e-01 rad/s; speed_dot max_abs=1.522830e+04 rad/s^2, rms=7.584539e+02 rad/s^2, mean_abs=2.613204e+02 rad/s^2 |
| PASS | SEA_Ankle tau_input saturation | tau_input never reaches the +/-500 Nm clamp |
| PASS | SEA_Ankle saturation source terms | raw command below clamp; raw max_abs=1.359497e+02 Nm, rms=4.898822e+01 Nm, mean_abs=3.407973e+01 Nm; motor numerator max_abs=1.522830e+02 Nm, rms=7.584539e+00 Nm, mean_abs=2.613204e+00 Nm |
| WARN | pros_knee_angle reserve torque | prosthetic reserve is exactly zero; OK only if SEA path is validated; max_abs=0.000000e+00 Nm, rms=0.000000e+00 Nm, mean_abs=0.000000e+00 Nm |
| WARN | pros_ankle_angle reserve torque | prosthetic reserve is exactly zero; OK only if SEA path is validated; max_abs=0.000000e+00 Nm, rms=0.000000e+00 Nm, mean_abs=0.000000e+00 Nm |
| PASS | pros_knee_angle output vs IK | non-zero tracking error; max_abs=7.783270e-01 deg, rms=1.803783e-01 deg, mean_abs=1.435151e-01 deg |
| PASS | pros_ankle_angle output vs IK | non-zero tracking error; max_abs=3.858532e+00 deg, rms=1.112887e+00 deg, mean_abs=7.692996e-01 deg |
| WARN | pelvis_tx output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=1.296730e-03 m, rms=2.924037e-04 m, mean_abs=2.304276e-04 m |
| WARN | pelvis_ty output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=7.211100e-04 m, rms=2.321609e-04 m, mean_abs=1.902045e-04 m |
| WARN | pelvis_tz output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=1.283330e-03 m, rms=2.871090e-04 m, mean_abs=2.205830e-04 m |
| PASS | pelvis_tilt output vs IK | non-zero tracking error; max_abs=4.496887e-01 deg, rms=1.025208e-01 deg, mean_abs=8.078767e-02 deg |
| PASS | pelvis_list output vs IK | non-zero tracking error; max_abs=5.426736e-01 deg, rms=1.064687e-01 deg, mean_abs=7.836922e-02 deg |
| WARN | pelvis_rotation output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=3.398235e-01 deg, rms=8.697970e-02 deg, mean_abs=6.954263e-02 deg |
| PASS | hip_flexion_r output vs IK | non-zero tracking error; max_abs=4.947678e-01 deg, rms=1.506933e-01 deg, mean_abs=1.231018e-01 deg |
| PASS | hip_adduction_r output vs IK | non-zero tracking error; max_abs=8.546339e-01 deg, rms=1.221499e-01 deg, mean_abs=9.440692e-02 deg |
| PASS | hip_rotation_r output vs IK | non-zero tracking error; max_abs=1.325920e+00 deg, rms=3.544902e-01 deg, mean_abs=2.856867e-01 deg |
| PASS | knee_angle_r output vs IK | non-zero tracking error; max_abs=1.349586e+00 deg, rms=1.999873e-01 deg, mean_abs=1.505846e-01 deg |
| PASS | ankle_angle_r output vs IK | non-zero tracking error; max_abs=9.683731e+00 deg, rms=3.792432e+00 deg, mean_abs=2.969217e+00 deg |
| PASS | subtalar_angle_r output vs IK | non-zero tracking error; max_abs=9.974967e+00 deg, rms=2.938517e+00 deg, mean_abs=2.078937e+00 deg |
| FAIL | mtp_angle_r output vs IK | tracking error exceeds stability threshold; max_abs=2.982952e+01 deg, rms=1.384244e+01 deg, mean_abs=1.101702e+01 deg |
| PASS | hip_flexion_l output vs IK | non-zero tracking error; max_abs=6.798546e-01 deg, rms=1.285142e-01 deg, mean_abs=9.852690e-02 deg |
| PASS | hip_adduction_l output vs IK | non-zero tracking error; max_abs=5.238374e-01 deg, rms=1.204945e-01 deg, mean_abs=9.306881e-02 deg |
| PASS | hip_rotation_l output vs IK | non-zero tracking error; max_abs=7.453104e-01 deg, rms=1.673335e-01 deg, mean_abs=1.297260e-01 deg |
| WARN | lumbar_extension output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=9.926265e-02 deg, rms=4.258122e-02 deg, mean_abs=3.275876e-02 deg |
| WARN | lumbar_bending output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=1.912229e-01 deg, rms=8.411118e-02 deg, mean_abs=6.369595e-02 deg |
| WARN | lumbar_rotation output vs IK | very tight tracking; result is controller-following, not predictive; max_abs=1.306223e-01 deg, rms=5.466658e-02 deg, mean_abs=4.236031e-02 deg |
| PASS | SEA_Knee control saturation | SEA control below saturation; max \|u\|=0.331577 |
| PASS | SEA_Knee motor vs joint power | motor-joint diff max_abs=4.472407e+02 W, rms=3.302645e+01 W, mean_abs=1.204192e+01 W, corr=0.234394 |
| PASS | SEA_Ankle control saturation | SEA control below saturation; max \|u\|=0.481835 |
| PASS | SEA_Ankle motor vs joint power | motor-joint diff max_abs=6.091425e+02 W, rms=4.230237e+01 W, mean_abs=1.897902e+01 W, corr=0.458588 |

## Interpretazione

Almeno un controllo critico indica che il risultato non e validabile.
Un FAIL su `run status` indica output parziali; un FAIL su tracking
indica divergenza dinamica; un FAIL sulle metriche SEA indica possibile
tautologia o derivate plugin non finite.
