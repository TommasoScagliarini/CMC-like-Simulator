# Analisi rollout GRF soft - 2026-06-24

Instruction check token: CMC_AGENT_OK_2026

## Problema

Dopo il daily del 2026-06-23, la priorita' era valutare il training da zero con
penalty GRF piu' morbida:

```text
grf_penetration_weight: 1.0
grf_ankle_moment_flip_weight: 0.10
imitation_knee_position_weight: 1.0
imitation_knee_velocity_weight: 0.02
imitation_ankle_position_weight: 2.0
imitation_ankle_velocity_weight: 0.04
```

L'obiettivo era verificare se il setup soft mantenesse il beneficio del
pacchetto contact-validity sull'ankle senza trasferire troppo costo sul knee,
come accaduto con il run hard-GRF.

## Artefatti analizzati

Training:

```text
Trajectory Generator/runs/training/MLP_imitation_training_06-23-2026_grfsoft_knee1_ankle2_100iter
```

Rollout:

```text
Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-23-2026_grfsoft_knee1_ankle2_100iter
```

Plot:

```text
plot/06_24_2026_1_asym100_GRFpenalty-lowered
```

Confronti principali:

```text
Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-22-2026
Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-23-2026_knee3_pos_40iter
Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-17-2026_asym_actor_critic_100
Trajectory Generator/runs/rollout/baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm
```

## Strategia di analisi

L'analisi ha separato tre livelli:

1. qualita' della reference servita dalla policy rispetto al target imitativo;
2. tracking fisico SEA/plant della reference servita;
3. compromesso tra contatto online, reserve, coppia SEA e clipping.

Sono stati usati:

- `rollout_summary.json` per return, clipping e config reward;
- `rollout_policy_trace.json` per losses, azioni raw/applied e reference
  target/served/actual;
- `rollout_episode_kinematics.sto` e
  `rollout_episode_kinematics_reference.sto`;
- `rollout_episode_sea_diagnostics.sto`;
- `rollout_episode_online_grf.sto`;
- `rollout_episode_reserve_torques.sto`;
- plot diagnostici MLP 05, 06, 07 e gait-cycle torque/power.

Per evitare che il transiente iniziale dominasse la lettura della dinamica SEA,
le metriche di coppia sono state calcolate sia sull'intero rollout sia
scartando i primi `0.2 s`.

## Risultati principali

Il rollout GRF soft e' completato senza terminazione anticipata:

```text
training completed: 100 / 100 iterations
best train return: 293.595, best iteration: 94
rollout return: 348.525
reward_mean: 0.695659
terminated: false
truncated: true
action_clipped_fraction: 0.032934
raw action max: 1.432
```

Confronto con i due run immediatamente precedenti:

```text
case              return    served loss   clip     left Fy mean   root reserve RMS
hard GRF 100      333.7     0.0591        0.0060   28.3 N         502.5
knee_pos=3 40     202.9     0.2844        0.0200   13.6 N         518.2
soft GRF latest   348.5     0.0298        0.0329   61.0 N         462.6
```

Il GRF soft migliora quindi return, imitation servita, carico online sul lato
sinistro e reserve root rispetto al GRF hard. Il costo nuovo e' un clipping piu'
alto dell'azione.

## Tracking cinematico

Il plant segue bene la reference servita:

```text
soft GRF latest:
knee served -> actual q RMSE   0.00260 rad
ankle served -> actual q RMSE  0.00474 rad
```

Quindi il collo di bottiglia non e' il tracking fisico SEA della reference, ma
la forma della reference scelta dalla policy e il compromesso contatto/GRF che
essa induce.

## Coppia SEA e chattering

Metriche SEA calcolate dopo aver scartato i primi `0.2 s`:

```text
case              knee RMS   knee HP20 ratio   ankle RMS   ankle area + / -
sym60             8.06 Nm    0.331             5.68 Nm     +1.46 / -15.90
asym100 no flip   13.17 Nm   0.231             5.52 Nm     +9.06 / -6.01
hard GRF 100      7.03 Nm    0.356             2.81 Nm     +2.36 / -5.95
soft GRF latest   6.73 Nm    0.466             4.11 Nm     +1.70 / -11.30
```

L'ankle del soft-GRF si sposta verso una forma piu' negativa, vicina a `sym60`,
e non mostra il burst positivo sospetto di `asym100`. Questo e' il beneficio
biomeccanico principale del run.

Il knee pero' paga il compromesso: RMS non enorme, ma quota high-pass piu' alta
del gruppo. Nei plot `05_time_tau_input_tracking_error.png` e
`01_time_sea_control_reserve.png` il knee mostra chiaramente `tau_input` e
`tau_spring` ad alta frequenza.

## Interpretazione

La lettura piu' solida e':

```text
GRF hard:
  troppo severo; la policy evita il problema alterando contatto e knee.

knee_pos=3:
  direzione sbagliata; peggiora tracking, return, command-rate e clipping.

GRF soft:
  direzione giusta; migliora imitation, contatto e reserve, ma sposta il costo
  su clipping e chattering knee.
```

Il run non va interpretato come soluzione finale. E' pero' la prima ablation
promettente dopo l'introduzione del pacchetto contact-validity: mantiene il
segno ankle piu' plausibile senza collassare il rollout.

## File modificati

Codice/config:

```text
Nessuna modifica.
```

Report creato:

```text
reports/user/2026-06-24_analisi_rollout_grf_soft.md
```

Nota operativa: `Trajectory Generator/baseline_MLP/training_cfg.yaml` risulta
ancora impostato sui pesi hard-GRF:

```text
grf_penetration_weight: 5.0
grf_ankle_moment_flip_weight: 0.25
```

Il run soft-GRF e' stato ottenuto tramite config risolto/override, non come
default persistito nel file YAML.

## Verifiche eseguite

- Lettura `rollout_summary.json` del run soft-GRF.
- Lettura `summary.json` del training soft-GRF.
- Confronto con hard-GRF 100, `knee_pos=3` 40 iter, `asym100` senza flip guard e
  `sym60`.
- Parsing degli `.sto`:
  - kinematics;
  - kinematics reference;
  - SEA diagnostics;
  - online GRF;
  - reserve torques;
  - power.
- Calcolo di:
  - return, reward mean, clipping;
  - served imitation loss;
  - RMSE target -> served -> actual;
  - `tau_spring` RMS, min/max, area positiva/negativa;
  - high-pass ratio a 20 Hz;
  - GRF left mean force, contact fraction e penetration;
  - reserve root RMS.
- Ispezione dei plot:
  - `01_time_sea_control_reserve.png`;
  - `03_gaitcycle_torque_angle_power.png`;
  - `05_time_tau_input_tracking_error.png`;
  - `06_time_joint_ref_sea_error.png`;
  - `07_mlp_policy_vs_sound_leg_error.png`.
- Verifica plot:
  - `plot/06_24_2026_1_asym100_GRFpenalty-lowered/missing_channels.txt`
    riporta `No missing channels.`

## TODO

- [ ] Non proseguire con ulteriori aumenti globali del peso knee prima di avere
      diagnosticato il chattering knee del run soft-GRF.
- [ ] Analizzare il knee soft-GRF per fase del passo: clipping, command-rate,
      `tau_spring` high-pass e `tau_input_plugin` high-pass, separando stance e
      swing.
- [ ] Confrontare knee soft-GRF contro `sym60` come baseline anti-chattering,
      senza considerare `sym60` un successo globale perche' reserve e scala
      della coppia ankle restano non risolte.
- [ ] Valutare una mitigazione knee-specific solo dopo la diagnosi per fase:
      jerk/acceleration limit knee-only, penalty knee-specific o bounded action
      head, evitando una penalty globale su `sea_tau_spring_rate_weight` che
      potrebbe danneggiare l'ankle.
- [ ] Decidere se aggiornare `Trajectory Generator/baseline_MLP/training_cfg.yaml`
      ai pesi soft-GRF oppure conservarlo hard-GRF finche' il confronto non e'
      formalmente chiuso.
