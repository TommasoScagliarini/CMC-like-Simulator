# SEA Parameter Sweep

Data: 2026-04-16

## Formula

`Kp = Jm * omega_n^2 / K - 1`

`Kd = 2 * zeta * Jm * omega_n - Bm`

con `omega_n = 500 rad/s` e `zeta = 0.7`.

## Risultato

Nessuna soluzione accettabile trovata.

Sweep root: `C:\Users\tomma\Desktop\Opensim OMNIBUS\CMC-like-Simulator - Claude\results\_sea_parameter_sweep_20260416_205113`

## Top Candidates

| Rank | Run | Acceptable | Score | Knee K | Ankle K | Knee RMS deg | Ankle RMS deg | Max raw Nm | Reason |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---|
| 1 | `full_k75_a75` | False | 7.8417 | 75.0 | 75.0 | 4.330 | 9.381 | 867.935 | tau_input saturated in 2 samples; tau_input_raw max 867.935 > 480 |
| 2 | `screen_combo_k75_a75` | False | 7.9692 | 75.0 | 75.0 | 4.157 | 9.585 | 867.935 | tau_input saturated in 2 samples; tau_input_raw max 867.935 > 480 |
| 3 | `full_k100_a75` | False | 8.1601 | 100.0 | 75.0 | 5.407 | 9.836 | 629.782 | tau_input saturated in 2 samples; tau_input_raw max 629.782 > 480 |
| 4 | `screen_combo_k100_a75` | False | 8.4059 | 100.0 | 75.0 | 5.292 | 10.208 | 629.782 | validator FAIL or process error; tau_input saturated in 2 samples; tau_input_raw max 629.782 > 480 |
| 5 | `full_k125_a75` | False | 8.5202 | 125.0 | 75.0 | 6.413 | 10.313 | 489.011 | validator FAIL or process error; tau_input_raw max 489.011 > 480 |
| 6 | `full_k150_a75` | False | 8.8190 | 150.0 | 75.0 | 7.340 | 10.795 | 396.610 | validator FAIL or process error |
| 7 | `screen_combo_k125_a75` | False | 8.8689 | 125.0 | 75.0 | 6.349 | 10.832 | 489.011 | validator FAIL or process error; tau_input_raw max 489.011 > 480 |
| 8 | `screen_combo_k150_a75` | False | 9.2643 | 150.0 | 75.0 | 7.327 | 11.450 | 396.610 | validator FAIL or process error |
| 9 | `full_k75_a100` | False | 9.4768 | 75.0 | 100.0 | 4.361 | 11.798 | 867.578 | validator FAIL or process error; tau_input saturated in 2 samples; tau_input_raw max 867.578 > 480 |
| 10 | `screen_combo_k75_a100` | False | 9.6265 | 75.0 | 100.0 | 4.189 | 12.035 | 867.578 | validator FAIL or process error; tau_input saturated in 2 samples; tau_input_raw max 867.578 > 480 |
| 11 | `full_k100_a100` | False | 9.8957 | 100.0 | 100.0 | 5.444 | 12.401 | 629.386 | validator FAIL or process error; tau_input saturated in 2 samples; tau_input_raw max 629.386 > 480 |
| 12 | `screen_combo_k100_a100` | False | 10.1957 | 100.0 | 100.0 | 5.331 | 12.853 | 629.386 | validator FAIL or process error; tau_input saturated in 2 samples; tau_input_raw max 629.386 > 480 |

## Files

- Sweep CSV: `C:\Users\tomma\Desktop\Opensim OMNIBUS\CMC-like-Simulator - Claude\results\_sea_parameter_sweep_20260416_205113\sweep_results.csv`
- Best JSON: `C:\Users\tomma\Desktop\Opensim OMNIBUS\CMC-like-Simulator - Claude\results\_sea_parameter_sweep_20260416_205113\best_candidate.json`
