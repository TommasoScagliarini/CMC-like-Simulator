# RIGHT39: rollout deterministici alle iterazioni 58 e 60

| Rollout | Passi | Return | RMSE knee / ankle [°] | Pen. max [mm] | Cicli validi | Invalid / resync |
|---|---:|---:|---:|---:|---:|---:|
| 39D RIGHT best58 | 500 | 278.721 | 7.223 / 3.118 | 17.694 | 2.0 | 0.0 / 0.0 |
| 39D RIGHT last60 | 500 | 324.151 | 7.066 / 2.414 | 16.238 | 2.0 | 0.0 / 0.0 |
| 39D RIGHT best35 | 500 | 261.938 | 9.595 / 3.632 | 16.238 | 1.0 | 2.0 / 2.0 |
| 39D RIGHT last40 | 500 | 286.318 | 9.386 / 2.757 | 16.238 | 1.0 | 3.0 / 1.0 |
| 39D LEFT best72 | 500 | 200.895 | 6.556 / 5.978 | 21.838 | 2.0 | 0.0 / 0.0 |
| V26 agosto best87 | 500 | 363.786 | 6.906 / 1.459 | 16.238 | 2.0 | 0.0 / None |

## Coppie e distribuzione delle azioni

| Rollout | Coppia ankle max [Nm] | Chattering knee >20Hz, dopo 0,2s [Nm RMS] | Std knee / ankle |
|---|---:|---:|---:|
| 39D RIGHT best58 | 31.852 | 3.226 | 0.657 / 0.606 |
| 39D RIGHT last60 | 25.887 | 3.266 | 0.632 / 0.576 |
| 39D RIGHT best35 | 5.665 | 3.074 | 0.694 / 0.597 |
| 39D RIGHT last40 | 3.301 | 3.097 | 0.671 / 0.543 |
| 39D LEFT best72 | 47.436 | 4.108 | 0.936 / 1.034 |
| V26 agosto best87 | 10.188 | 3.280 | 1.026 / 0.750 |

## Contributi alla reward cumulativa

| Rollout | Imitazione actual pesata | Imitazione served pesata | Penalità command rate | Penalità GRF ankle |
|---|---:|---:|---:|---:|
| 39D RIGHT best58 | 81.929 | 286.350 | 70.007 | 19.127 |
| 39D RIGHT last60 | 86.483 | 310.428 | 66.501 | 6.041 |
| 39D RIGHT best35 | 77.368 | 261.844 | 77.188 | 0.000 |
| 39D RIGHT last40 | 82.003 | 285.630 | 81.138 | 0.000 |
| 39D LEFT best72 | 74.272 | 248.301 | 77.831 | 43.108 |
| V26 agosto best87 | 90.631 | 334.719 | 61.412 | 0.010 |

Picchi di penetrazione dalla registrazione fisica STO; le metriche reward usano campioni a 100 Hz. Segni delle coppie coerenti con i plot canonici.

## Limiti

- One training seed and one deterministic start per checkpoint; no statistical-equivalence or stochastic-robustness test.
- Best58 and best35 are training-selected checkpoints, not winners of a separate deterministic evaluation selection.
- RIGHT/LEFT changes target template, clock, target velocities and target-derived normalization together.
- V26 best87 and LEFT best72 have more training experience; they are quality references, not matched-budget controls.
- FSM valid-cycle count is accepted non-degraded HS-to-HS cycles; rejected cycles may still contain limb motion.
- Conditional learned standard deviations and clipping probabilities use states from deterministic rollouts only.
- Lag alignment is descriptive and does not identify causal delay introduced by the reference filter.
