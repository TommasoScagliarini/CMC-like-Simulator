# 2026-05-17 - PI inner nel plugin SEA con state variable RK4

## Problema

L'obiettivo era spostare il PI inner completamente dentro il plugin C++ `SeriesElasticActuator`, usando l'integrale dell'errore di coppia come state variable OpenSim, senza congelare stati plugin in RK4 e senza alterare il modello PD baseline.

Target di banda: mantenere la dinamica dominante del PD post-bump con `Kp` e `Kd` invariati e aggiungere un integratore lento, con `omega_i = Ki / (1 + Kp) = 10 rad/s`.

## Soluzione implementata

- Nel plugin SEA sono stati aggiunti:
  - property `Ki`, default `0.0`;
  - property `integral_torque_limit`, default `100 Nm`;
  - state variable `torque_error_integral`;
  - output `torque_error_integral_dot`.
- In modo non-impedance la legge inner ora e:

  ```text
  tau_input_raw = tau_ref
                + Kp * (tau_ref - tau_spring)
                + clamp(Ki * xi, -integral_torque_limit, +integral_torque_limit)
                - Kd * omega_m
  xi_dot        = tau_ref - tau_spring
  ```

- Anti-windup scelto:
  - clamp esplicito del contributo integrale `Ki*xi`;
  - integrazione condizionale: `xi_dot` viene congelato se il contributo integrale o il torque motor raw sono saturi e l'errore spingerebbe ulteriormente nella saturazione.
- Il modo impedance resta senza integrazione PI (`torque_error_integral_dot = 0`).
- RK4 e output sono stati resi generali: ogni state variable SEA sotto `/forceset/SEA_*` viene integrata se espone un output `<state_name>_dot`; se manca la derivata il runner fallisce esplicitamente.

## Modello PI

Creato modello separato:

- `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim`

Parametri inner:

| SEA | Kp | Kd | Ki | omega_i |
| --- | ---: | ---: | ---: | ---: |
| SEA_Knee | 18 | 11 | 190 | 10 rad/s |
| SEA_Ankle | 11.3 | 11 | 123 | 10 rad/s |

Il modello PD baseline `AB06_SEASEA_stiff321_500.osim` non e stato modificato.

## File modificati

- `tools/sea_plugin_relative_d/SeriesElasticActuator.h`
- `tools/sea_plugin_relative_d/SeriesElasticActuator.cpp`
- `model_loader.py`
- `simulation_runner.py`
- `output.py`
- `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim`
- `plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib`
  - backup precedente: `plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib.pre_pi_bandmatched_20260517`

## Verifiche

Build plugin:

```text
cmake --build tools/sea_plugin_relative_d/build --target SEA_Plugin_BlackBox_mCMC_impedence_ff -j 4
PASS
```

Smoke load OpenSim Python:

- plugin caricato da `plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff`;
- modello PI caricato e inizializzato;
- verificati `Ki`, `torque_error_integral` e output `torque_error_integral_dot` per `SEA_Knee` e `SEA_Ankle`.

Smoke run:

```text
results/_pi_inner_smoke_20260517
t = 13.1638 -> 13.1938 s
status = complete
steps = 30
```

Metriche smoke:

| SEA | xi_delta | RMS tau_input plugin-Python | max diff | saturazioni |
| --- | ---: | ---: | ---: | ---: |
| SEA_Knee | -0.063456 | 2.49e-06 Nm | 4.86e-06 Nm | 0 |
| SEA_Ankle | -0.127831 | 1.92e-06 Nm | 4.85e-06 Nm | 0 |

Controlli codice:

- `python -m py_compile model_loader.py simulation_runner.py output.py`: PASS
- `git diff --check -- model_loader.py simulation_runner.py output.py`: PASS

Validazione PI con `validation/validate_sim_results.py`:

- controlli specifici SEA: PASS per stato plugin, derivate, accordo `tau_input` plugin/Python e assenza di saturazioni;
- summary globale dello script: mantiene FAIL su soglie legacy di tracking whole-body non usate come criterio di accettazione del driver PI. Il confronto PD/PI sotto usa le metriche dedicate del gain-sweep.

## Full comparison PD/PI

Nota: la directory `results/_fast_inner_pid_20260516` indicata nel piano non era presente nella workspace. Ho quindi rigenerato un baseline equivalente con il modello PD invariato e la stessa configurazione:

- PD rerun: `results/_fast_inner_pid_20260516_rerun_20260517`
- PI full: `results/_pi_inner_bandmatched_full_20260517`

Entrambi:

```text
t = 11.99 -> 21.00 s
steps = 9010
status = complete
integration_scheme = rk4_bypass
sea_forward_mode = plugin
outer PID = knee 340/30/120, ankle 850/2/300
GRF filter = enabled
```

Risultati principali:

| Metrica | PD | PI | Delta PI-PD |
| --- | ---: | ---: | ---: |
| mean prosthetic RMSE | 1.8735 deg | 1.8903 deg | +0.0168 deg (+0.90%) |
| knee RMSE | 1.2958 deg | 1.3174 deg | +0.0216 deg (+1.67%) |
| ankle RMSE | 2.4512 deg | 2.4632 deg | +0.0119 deg (+0.49%) |
| knee tau_error RMS | 1.2295 Nm | 1.0522 Nm | -14.4% |
| ankle tau_error RMS | 1.0040 Nm | 0.9066 Nm | -9.7% |
| knee motor_speed_dot RMS | 769.5 rad/s^2 | 791.0 rad/s^2 | +2.8% |
| ankle motor_speed_dot RMS | 459.9 rad/s^2 | 473.5 rad/s^2 | +3.0% |
| tau_reserve_norm RMS | 114.0765 | 114.1229 | +0.04% |
| reserve_control_norm RMS | 1.1408 | 1.1412 | +0.04% |

Diagnostica inner PI:

| SEA | xi RMS | xi final | Ki*xi RMS | max | final |
| --- | ---: | ---: | ---: | ---: | ---: |
| SEA_Knee | 0.0686 | -0.0035 | 13.04 Nm | 34.27 Nm | -0.66 Nm |
| SEA_Ankle | 0.0534 | 0.0312 | 6.57 Nm | 24.56 Nm | 3.84 Nm |

Accordo plugin/Python:

| SEA | PD RMS diff | PI RMS diff | PI max diff |
| --- | ---: | ---: | ---: |
| SEA_Knee | 1.71e-07 Nm | 1.71e-07 Nm | 5.00e-07 Nm |
| SEA_Ankle | 5.17e-07 Nm | 5.09e-07 Nm | 5.00e-06 Nm |

Saturazioni inner `tau_input`: 0 per PD e 0 per PI, su entrambi i SEA.

## Plot generati

Plot standard del full run PI:

- `plot/05_17_2026_1/01_time_sea_control_reserve.png`
- `plot/05_17_2026_1/02_time_joint_motor_states.png`
- `plot/05_17_2026_1/03_gaitcycle_torque_angle_power.png`
- `plot/05_17_2026_1/04_gaitcycle_joint_velocity_power.png`
- `plot/05_17_2026_1/05_time_tau_input_tracking_error.png`
- `plot/05_17_2026_1/06_time_joint_ref_sea_error.png`

Il file `plot/05_17_2026_1/missing_channels.txt` riporta: `No missing channels.`

Plot comparativi PD-vs-PI:

- `plot/05_17_2026_2_pi_vs_pd/01_tracking_error_pd_vs_pi.png`
- `plot/05_17_2026_2_pi_vs_pd/02_tau_error_tau_input_pd_vs_pi.png`
- `plot/05_17_2026_2_pi_vs_pd/03_motor_dynamics_pd_vs_pi.png`
- `plot/05_17_2026_2_pi_vs_pd/04_integral_state_pd_vs_pi.png`
- `plot/05_17_2026_2_pi_vs_pd/05_recruitment_reserve_pd_vs_pi.png`
- `plot/05_17_2026_2_pi_vs_pd/06_plugin_python_diff_saturation_pd_vs_pi.png`

## Esito

Il PI inner dentro il plugin e funzionante. RK4 integra genericamente anche `torque_error_integral`, il modello PI viene letto correttamente da OpenSim, lo smoke test passa e il full run completa senza criticita SEA. Rispetto al PD, il PI riduce l'errore RMS di coppia inner con tracking protesico praticamente invariato e senza saturazioni del motor torque.

TODO: nessuno.
