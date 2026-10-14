# Offline masking rollout diagnosis

Common window: 13.19–17.94 s.

| Policy | Window end [s] | RMSE knee/ankle [deg] | Actual/served knee [deg] | Knee excursion [deg] | Ankle positive area [Nm s] | Knee >20Hz [Nm RMS] | Return |
|---|---:|---:|---:|---:|---:|---:|---:|
| cp90_visible | 17.94 | 6.64/3.94 | 0.18 | 41.71 | 2.88 | 3.62 | 301.14 |
| cp112_hidden | 17.94 | 13.47/10.63 | 0.20 | 19.04 | 8.41 | 3.37 | 138.19 |
| cp184_hidden | 17.94 | 12.43/8.89 | 0.22 | 21.36 | 3.42 | 3.50 | 210.42 |
| cp90_hidden | 17.94 | 13.28/9.55 | 0.30 | 11.55 | 3.20 | 3.23 | 146.06 |
| cp112_visible | 17.94 | 7.03/4.27 | 0.17 | 46.78 | 15.16 | 3.53 | 273.12 |
| cp184_visible | 13.96 | 10.90/5.66 | 0.11 | 38.36 | 0.41 | 3.78 | 45.88 |

Returns in the preceding table cover each entire recorded episode; CP184 visible lasts 0.98 s and its return is not comparable to five-second returns.

## Frozen-policy visibility controls

Both modes use identical checkpoints, initialization, seed and target. RMSE entries are knee/ankle in degrees.

| Checkpoint | Visible RMSE | Hidden RMSE | Visible/hidden steps | Interpretation |
|---|---:|---:|---:|---|
| CP90 | 6.64/3.94 | 13.28/9.55 | 500/500 | Large immediate loss when teacher inputs are removed before any masking updates. |
| CP112 | 7.03/4.27 | 13.47/10.63 | 500/500 | Visible imitation largely retained; hidden imitation still poor. |
| CP184 | 10.90/5.66 (short prefix) | 12.43/8.89 | 98/500 | Hidden recovery remains incomplete; restored visibility crosses knee extension guard at 0.98 s. |

CP184 matched surviving prefix 13.19–13.96 s (78 steps):

| Policy | RMSE knee/ankle [deg] | Mean reward |
|---|---:|---:|
| cp184_visible | 10.90/5.66 | 0.4276 |
| cp184_hidden | 16.46/13.87 | 0.3165 |
| cp90_visible | 6.55/3.13 | 0.6366 |

The terminal knee position is only +0.0004203 rad beyond the raw zero-extension boundary, with served reference +0.0029552 rad. This is a hard-guard crossing, not evidence of a large numerical explosion.

## Reward and torque interpretation

The low-level controller follows the served reference within about 0.2 degrees RMS; the hidden kinematic error is already present in that reference.
For the original CP90-visible / CP112-hidden / CP184-hidden trio: knee excursion 41.71 / 19.04 / 21.36 degrees; ankle excursion 30.67 / 8.98 / 17.62 degrees.
Ankle positive-time fraction falls 57.3% / 50.0% / 43.1%, whereas positive area is 2.88 / 8.41 / 3.42 Nm s. These quantify different aspects of the curve.
Knee spring high-frequency RMS >20 Hz is 3.62 / 3.37 / 3.50 Nm, while motor torque high-frequency RMS is 13.24 / 14.25 / 15.32 Nm. Smoother appearance is not a general reduction of motor chattering.
The command-rate penalty increases 78.17 / 87.99 / 96.47 despite flatter served kinematics; nearly all is the segment-delta component. Raw endpoint mean steps and governor jerk limiting also increase.
At CP184, 67/500 steps have base reward clipped to zero, all within the 112 common-window steps whose target knee exceeds 35 degrees. These target-flexion steps yield 5.69 return versus 52.63 for CP90 visible.
Lag-only alignment leaves CP184 RMS errors of 10.45 degrees knee and 7.19 degrees ankle on a fixed central support; the missing amplitude and offset are not removed by a time shift.

Sensitivity to the previous 13.15–17.95 s window is saved in `window_sensitivity.json`; no new simulation is required.

Reproduce with `python analyze.py`; optional completed controls: `--extra label=/absolute/rollout/path`.

- Original three rollouts confound checkpoint adaptation and actor visibility; CP90 was visible, CP112/184 hidden.
- One deterministic nominal start and seed per policy/visibility; no multi-start or robustness claim.
- Same simulation times and recorded targets verified numerically across every included rollout.
- Position/velocity traces are at 100 Hz, physical force and torque at 1000 Hz. No units or sign changes in sources.
- Displayed knee coordinates and spring torques have canonical sign reversal; ankle retains raw sign.
- Excursion is max-min over the common window, not a phase-warped cycle mean.
- Highpass measures are zero-phase 4th-order Butterworth applied to full recorded signal then cropped to common window.
- Early failures have a shorter per-rollout window and omitted lag metrics; visibility pairs are also compared on an identical surviving prefix.
- All reward components are recomputed with the saved configuration and exact per-step clipping, reproducing saved returns.
