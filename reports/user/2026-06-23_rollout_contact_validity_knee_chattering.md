# Rollout contact-validity GRF e diagnosi chattering knee - 2026-06-23

Instruction check token: CMC_AGENT_OK_2026

## Problema

Dopo l'introduzione del pacchetto reward GRF contact-validity, il rollout
deterministico del training `asym100 + GRF contact-validity package` mostra un
miglioramento marcato della coppia ankle. Resta pero' visibile un chattering
non trascurabile sul knee, soprattutto nei segnali di coppia SEA e nel comando
motore interno.

Run analizzato:

```text
training: Trajectory Generator/runs/training/MLP_imitation_training_06-22-2026
rollout : Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-22-2026
plot    : plot/06_23_2026_1
```

## Stato rollout

Il rollout e' stato monitorato fino a completamento. Esito:

```text
ok                         true
steps                      501
episode_return             333.66533305871303
reward_mean                0.6659986687798664
terminated                 false
truncated                  true
action_abs_max             1.2520431280136108
applied_action_abs_max     1.0
action_clipped_steps       6
action_clipped_fraction    0.005988023952095809
```

Checkpoint usato:

```text
Trajectory Generator/runs/training/MLP_imitation_training_06-22-2026/rl_module_best
```

Config reward rilevante:

```yaml
grf_penetration_weight: 5.0
grf_ankle_moment_flip_weight: 0.25
grf_ankle_moment_flip_tau_tol_nm: 8.0
grf_ankle_moment_flip_force_threshold_n: 50.0
```

## Plot generati

Comando eseguito:

```text
/opt/anaconda3/envs/envCMC-rllib/bin/python plot/plotter.py --mlp
```

Output:

```text
plot/06_23_2026_1/01_time_sea_control_reserve.png
plot/06_23_2026_1/02_time_joint_motor_states.png
plot/06_23_2026_1/03_gaitcycle_torque_angle_power.png
plot/06_23_2026_1/04_gaitcycle_joint_velocity_power.png
plot/06_23_2026_1/05_time_tau_input_tracking_error.png
plot/06_23_2026_1/06_time_joint_ref_sea_error.png
plot/06_23_2026_1/07_mlp_policy_vs_sound_leg_error.png
plot/06_23_2026_1/missing_channels.txt
```

Verifica plotter:

```text
No missing channels.
```

## Diagnosi

La penalty GRF ha migliorato in modo evidente la forma della coppia ankle. Nel
rollout analizzato il termine:

```text
grf_ankle_moment_flip_loss
```

risulta nullo nei campioni ispezionati, coerentemente con l'assenza del burst
positivo ankle osservato in precedenza.

Il knee mostra invece chattering visibile soprattutto in:

```text
SEA_Knee_tau_spring
SEA_Knee_tau_input_plugin
SEA_Knee_tau_error
SEA_Knee_cascade_velocity_error
SEA_Knee_cascade_inner_p_cmd
```

Metriche high-pass su finestra circa `13.15-17.95 s`, con detrending locale
`80 ms`:

```text
Signal                     RMS        high-pass RMS   high-pass / RMS   dominante
Knee tau_spring             7.021 Nm    4.312 Nm       0.614             14.16 Hz
Ankle tau_spring            2.815 Nm    0.371 Nm       0.132             15.21 Hz
Knee tau_input_plugin      10.786 Nm    9.277 Nm       0.860             46.03 Hz
Ankle tau_input_plugin      2.854 Nm    0.609 Nm       0.213             11.87 Hz
Knee tau_error              1.543 Nm    1.493 Nm       0.968             46.03 Hz
Ankle tau_error             0.501 Nm    0.198 Nm       0.396             12.50 Hz
```

La reference knee in posizione non appare come causa primaria:

```text
pros_knee_angle q_ref RMS        0.6022 rad
pros_knee_angle q_ref hp RMS     0.0039 rad
pros_knee_angle q_ref hp/RMS     0.0065
```

Il problema sembra quindi nascere da una piccola componente rapida nell'errore di
velocita/reference amplificata dal cascade knee e dal loop interno, non da una
reference di posizione evidentemente seghettata.

## Confronto storico

Confronto sintetico su `tau_spring`:

```text
run                         knee RMS   knee hp RMS   ankle RMS   ankle hp RMS   return   clip frac
contact_validity_06-22       7.019      4.318         2.816       0.372          333.7    0.0060
asym100_06-17               13.17       4.409         5.518       0.523          370.1    0.0429
sym60                        8.05       2.902         5.676       0.374          334.8    0.0000
```

Interpretazione:

- il chattering knee non nasce necessariamente dalla penalty GRF: `asym100`
  aveva una componente high-pass knee comparabile;
- nel nuovo rollout la coppia ankle e' molto piu' pulita e la coppia knee lenta
  e' piu' piccola, quindi la componente rapida del knee pesa di piu' visivamente;
- `sym60` resta una baseline utile per capire come ridurre il chattering knee.

## Strategia consigliata

Non conviene rollbackare la penalty GRF ne' introdurre una penalita' globale che
rischi di peggiorare l'ankle. La linea piu' conservativa e':

1. mantenere il pacchetto GRF contact-validity;
2. agire solo sul knee;
3. prima provare leve gia' configurabili;
4. solo se necessario aggiungere una reward diagnostica/penalty knee-specific.

Leve candidate:

```yaml
simulation:
  pros_knee_ref_jerk_limit_rad_s3: 2000.0   # da 3000.0
  pros_knee_ref_acceleration_limit_rad_s2: 45.0  # da 60.0
```

La leva globale:

```yaml
reward:
  sea_tau_spring_rate_weight: ...
```

e' disponibile ma oggi e' a peso zero. Usarla globalmente puo' penalizzare anche
l'ankle, che nel rollout corrente e' proprio il segnale migliorato. Se si vuole
andare in questa direzione, meglio introdurre prima una variante knee-specific
basata su diagnostiche gia' presenti:

```text
pros_knee_angle_sea_tau_spring_rate_rms_nm_s
pros_knee_angle_sea_tau_spring_rate_abs_max_nm_s
```

## File modificati o generati

Codice:

```text
Nessuna modifica al codice durante questa fase di monitoraggio/diagnosi.
```

Artefatti generati:

```text
plot/06_23_2026_1/
```

Report creato:

```text
reports/user/2026-06-23_rollout_contact_validity_knee_chattering.md
```

## Test e verifiche eseguite

- Monitoraggio `watchdog_state.json` fino a `phase: complete`.
- Lettura di `rollout_summary.json` e `watchdog_summary.json`.
- Esecuzione `plot/plotter.py --mlp`.
- Verifica `missing_channels.txt`: nessun canale mancante.
- Ispezione dei plot:
  - `01_time_sea_control_reserve.png`;
  - `05_time_tau_input_tracking_error.png`;
  - `06_time_joint_ref_sea_error.png`;
  - `07_mlp_policy_vs_sound_leg_error.png`.
- Calcolo diagnostico high-pass su `tau_spring`, `tau_input_plugin`,
  `tau_error`, `cascade_velocity_error`, `cascade_inner_p_cmd`,
  `cascade_inner_i_cmd`, `q_ref`, `qdot_ref`.
- Confronto sintetico con rollout storici `asym100_06-17` e `sym60`.

## TODO

- [ ] Eseguire una ablation knee-only con:
      `pros_knee_ref_jerk_limit_rad_s3: 2000.0` e, se necessario,
      `1500.0`.
- [ ] Valutare una seconda ablation con
      `pros_knee_ref_acceleration_limit_rad_s2: 45.0`, mantenendo invariato
      l'ankle.
- [ ] Dopo ogni ablation, generare rollout e plot MLP, confrontando:
      `tau_spring` knee/ankle, `tau_input_plugin`, tracking reference,
      episode return, clipping e GRF penetration.
- [ ] Non applicare subito `sea_tau_spring_rate_weight` globale: prima verificare
      se serve una penalty knee-specific per non danneggiare l'ankle.
- [ ] Se i limiti knee non bastano, introdurre in
      `Trajectory Generator/baseline_MLP/reward_function.py` una penalty
      opzionale knee-specific basata sulle diagnostiche per-giunto gia' esposte
      dall'env.
- [ ] Continuare a usare `sym60` come baseline anti-chattering knee, senza
      interpretarlo come successo globale se reserve, scala coppia e lavoro
      ankle restano lontani dal riferimento.
