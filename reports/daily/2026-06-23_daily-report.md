# Daily report - 2026-06-23

Instruction check token: CMC_AGENT_OK_2026

## Sintesi

La giornata e' stata dedicata alla valutazione del training
`asym100 + GRF contact-validity package`, alla diagnosi del peggioramento del
tracking knee e alla falsificazione dell'ipotesi "basta aumentare il peso del
knee".

Risultato principale:

```text
Il pacchetto GRF contact-validity migliora molto la coppia ankle e rimuove il
burst positivo sospetto, ma sposta il compromesso sul knee, soprattutto in
stance/appoggio.
```

La prova con `imitation_knee_position_weight: 3.0` da zero a 40 iterazioni ha
peggiorato tracking, return, command-rate e clipping. La direzione successiva
non e' aumentare ancora il peso knee, ma testare una GRF penalty piu' morbida.

## Report utente creati

```text
reports/user/2026-06-23_rollout_contact_validity_knee_chattering.md
reports/user/2026-06-23_diagnosi_grf_penalty_tracking_knee_stance.md
reports/user/2026-06-23_aumento_peso_knee_peggiora_tracking.md
```

## Training e rollout analizzati

### GRF contact-validity 100 iter

Training:

```text
Trajectory Generator/runs/training/MLP_imitation_training_06-22-2026
```

Rollout:

```text
Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-22-2026
```

Plot:

```text
plot/06_23_2026_1_asym100_GRFpenalty
```

Metriche rollout:

```text
steps                      501
episode_return             333.66533305871303
reward_mean                0.6659986687798664
terminated                 false
truncated                  true
action_abs_max             1.2520431280136108
action_clipped_steps       6
action_clipped_fraction    0.005988023952095809
```

Effetto osservato:

- ankle `tau_spring` molto piu' pulita rispetto ad `asym100`;
- `grf_ankle_moment_flip_loss` nullo nel rollout ispezionato;
- knee con chattering visibile e peggioramento tracking, soprattutto nei tratti
  bassi del ciclo/appoggio.

Diagnostica high-pass:

```text
Signal                     RMS        high-pass RMS   high-pass / RMS
Knee tau_spring             7.021 Nm    4.312 Nm       0.614
Ankle tau_spring            2.815 Nm    0.371 Nm       0.132
Knee tau_input_plugin      10.786 Nm    9.277 Nm       0.860
Ankle tau_input_plugin      2.854 Nm    0.609 Nm       0.213
```

### Resume knee/ankle reweight

Resume da `checkpoint_best` con:

```json
{
  "imitation_knee_position_weight": 2.0,
  "imitation_knee_velocity_weight": 0.04,
  "imitation_ankle_position_weight": 1.0,
  "imitation_ankle_velocity_weight": 0.02
}
```

Rollout:

```text
Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-23-2026_resume_best_rebalance_knee2_ankle1_10iter_resume_best_rebalance_knee2_ankle1_10iter
plot/06_23_2026_3
```

Esito:

```text
episode_return             299.07750825450046
action_clipped_fraction    0.018962075848303395
knee served pos loss       0.043957
knee served vel loss       0.005655
ankle served pos loss      0.007981
ankle served vel loss      0.016270
```

Il reweight non ha recuperato il tracking knee e ha peggiorato return/clipping.

### Training da zero knee_pos=3, 40 iter

Training:

```text
Trajectory Generator/runs/training/MLP_imitation_training_06-23-2026
```

Config reward:

```yaml
imitation_knee_position_weight: 3.0
imitation_knee_velocity_weight: 0.02
imitation_ankle_position_weight: 2.0
imitation_ankle_velocity_weight: 0.04
grf_penetration_weight: 5.0
grf_ankle_moment_flip_weight: 0.25
```

Training summary:

```text
training da zero
target logico 40 iterazioni
best checkpoint iterazione 39
best_episode_return_mean 101.5160474100792
```

Rollout:

```text
Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-23-2026_knee3_pos_40iter
```

Plot:

```text
plot/06_23_2026_4
```

Metriche rollout:

```text
episode_return             202.91400486840143
reward_mean                0.40501797378922444
action_abs_max             1.1816991567611694
action_clipped_fraction    0.01996007984031936
terminated                 false
truncated                  true
```

Metriche imitation:

```text
knee served position loss         0.07751590324371488
knee served velocity loss         0.010257086146805754
ankle served position loss        0.0255369458004525
ankle served velocity loss        0.015377598228513494
served imitation loss             0.2844418469841263
command_rate_loss                 2.562754337982955
grf_penetration_loss              0.0009247479849812493
grf_ankle_moment_flip_loss        0.0
```

Confronto diretto:

```text
                                  GRFpenalty 100      knee_pos=3, 40iter
episode_return                    333.665             202.914
reward_mean                       0.6660              0.4050
action_clipped_fraction           0.0060              0.0200
knee served position loss         0.0383              0.0775
knee served velocity loss         0.0057              0.0103
ankle served position loss        0.0101              0.0255
ankle served velocity loss        0.0113              0.0154
served imitation loss             0.0591              0.2844
command_rate_loss                 1.77                2.56
```

Conclusione: aumentare il peso posizione knee peggiora, non migliora.

## Diagnosi consolidata

La reward imitation e' gia' normalizzata per range:

```text
loss = (errore / range_coord)^2
```

Range imitativi:

```text
pros_knee_angle  position range = 0.93256 rad
pros_ankle_angle position range = 0.35478 rad
pros_knee_angle  velocity range = 8.11114 rad/s
pros_ankle_angle velocity range = 3.37594 rad/s
```

Quindi non c'e' un bug banale di mancata normalizzazione. Il punto e' che il
knee sembra essere usato come leva posturale/contact-feasibility:

```text
la policy protegge ankle/GRF e paga il costo sul knee in stance.
```

Nel rollout GRFpenalty il tracking knee peggiora soprattutto nei tratti in cui
il piede protesico e' in appoggio o vicino all'appoggio. Il fenomeno non appare
come sottopeso globale del knee, ma come conflitto dinamico tra target imitativo,
contatto online e vincolo GRF.

Indizio importante:

```text
online left mean Fy:
asym100      circa 116 N
GRFpenalty   circa  28 N

campioni left_force_y > 50 N:
asym100      217 / 501
GRFpenalty   108 / 501
```

La GRF penalty potrebbe essere troppo forte o troppo severa: nel rollout finale
la loss e' piccola proprio perche' la policy ha gia' imparato a evitarla,
sacrificando il knee.

## Decisione operativa

Non proseguire con ulteriori aumenti del peso knee.

Prossima prova consigliata: training da zero con stessi pesi imitation del
GRFpenalty buono sull'ankle, ma GRF penalty piu' morbida.

Setup scelto:

```json
{
  "imitation_knee_position_weight": 1.0,
  "imitation_knee_velocity_weight": 0.02,
  "imitation_ankle_position_weight": 2.0,
  "imitation_ankle_velocity_weight": 0.04,
  "grf_penetration_weight": 1.0,
  "grf_ankle_moment_flip_weight": 0.10
}
```

Comando preparato:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python "Trajectory Generator/baseline_MLP/train_ppo_mlp.py" \
  --iterations 100 \
  --name _grfsoft_knee1_ankle2_100iter \
  --reward-mode imitation \
  --reward-json '{"imitation_knee_position_weight": 1.0, "imitation_knee_velocity_weight": 0.02, "imitation_ankle_position_weight": 2.0, "imitation_ankle_velocity_weight": 0.04, "grf_penetration_weight": 1.0, "grf_ankle_moment_flip_weight": 0.10}'
```

## File modificati o generati

Codice:

```text
Nessuna modifica al codice durante la giornata del 23/06.
```

Report creati:

```text
reports/user/2026-06-23_rollout_contact_validity_knee_chattering.md
reports/user/2026-06-23_diagnosi_grf_penalty_tracking_knee_stance.md
reports/user/2026-06-23_aumento_peso_knee_peggiora_tracking.md
reports/daily/2026-06-23_daily-report.md
```

Artefatti principali:

```text
plot/06_23_2026_1_asym100_GRFpenalty
plot/06_23_2026_2-asym100+10_GRFpenalty_knee
plot/06_23_2026_3
plot/06_23_2026_4
Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-22-2026
Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-23-2026_resume_best_knee2_v004_10iter
Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-23-2026_resume_best_rebalance_knee2_ankle1_10iter_resume_best_rebalance_knee2_ankle1_10iter
Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-23-2026_knee3_pos_40iter
```

## Test e verifiche eseguite

- Monitoraggio training/rollout fino a completamento.
- Rollout deterministici del `rl_module_best` per i run analizzati.
- Generazione plot con:

```text
/opt/anaconda3/envs/envCMC-rllib/bin/python plot/plotter.py --mlp
```

- Verifica `missing_channels.txt`: nessun canale mancante nei plot generati.
- Lettura `rollout_summary.json` e `summary.json`.
- Estrazione medie da `rollout_policy_trace.json`.
- Confronto knee/ankle served-target, command-rate, clipping, GRF loss.
- Ispezione qualitativa di:
  - `07_mlp_policy_vs_sound_leg_error.png`;
  - `05_time_tau_input_tracking_error.png`;
  - `06_time_joint_ref_sea_error.png`;
  - time-series reserve/SEA.

## TODO chiusi o avanzati il 23/06

- [x] Lanciare e completare il training `asym100 + GRF contact-validity package`.
- [x] Eseguire rollout deterministico del best checkpoint e generare plot MLP.
- [x] Verificare se il pacchetto contact-validity riduce il burst positivo ankle.
      Esito: si', ma peggiora tracking knee e lascia chattering knee visibile.
- [x] Monitorare `tau_spring`, `grf_ankle_moment_flip_loss`,
      `grf_penetration_loss`, served-reference tracking, clipping e return.
- [x] Verificare se aumentare il peso knee risolve il tracking.
      Esito: no; `knee_pos=3.0` peggiora tracking e reward.
- [x] Aggiornare la strategia: prima testare GRF soft, non ulteriori aumenti
      del peso knee.

## TODO aperti e propagati

### Prioritari dal 23/06

- [ ] Eseguire training da zero con GRF soft:
      `grf_penetration_weight: 1.0`,
      `grf_ankle_moment_flip_weight: 0.10`,
      pesi imitation originali del run GRFpenalty.
- [ ] Fare rollout del `rl_module_best` del training GRF soft e generare plot.
- [ ] Confrontare `GRFpenalty 100`, `knee_pos=3.0 40iter` e `GRF soft` su:
  - knee served-target RMS in stance;
  - knee bias in stance;
  - ankle `tau_spring`;
  - knee chattering / high-pass;
  - online left mean Fy;
  - durata/campioni di contatto;
  - `grf_penetration_loss`;
  - `grf_ankle_moment_flip_loss`;
  - clipping;
  - return.
- [ ] Non proseguire con ulteriori aumenti del peso knee prima di valutare GRF
      soft.
- [ ] Se GRF soft non risolve, rivalutare solo allora:
  - limiti knee-only su jerk/acceleration;
  - eventuale penalty knee-specific;
  - mai prima di verificare che non danneggi ankle.

### Ancora aperti dal 23/06

- [ ] Continuare a usare `sym60` come baseline anti-chattering knee, senza
      interpretarlo come successo globale se reserve, scala coppia e lavoro
      ankle restano lontani dal riferimento.
- [ ] Non applicare subito `sea_tau_spring_rate_weight` globale: prima
      verificare se serve una penalty knee-specific per non danneggiare ankle.
- [ ] Dopo ogni ablation, generare rollout e plot MLP, confrontando
      `tau_spring` knee/ankle, `tau_input_plugin`, tracking reference,
      episode return, clipping e GRF penetration.

### Propagati dal 22/06 e ancora rilevanti

- [ ] Progettare un candidato reward debole di contact confidence che fonda fase
      gait, eventi, cinematica del piede e `online_grf`, senza usare
      `online_grf` come segnale forte.
- [ ] Definire una versione quantitativa minimale dei termini di uso meccanico
      SEA senza target torque, morfologia della coppia o dettagli controller.
- [ ] Stabilire una baseline/floor per reserve e residual che tenga conto delle
      asimmetrie sane prescribed.
- [ ] Valutare offline, sui rollout disponibili, se i candidati reward 2/3/4
      distinguono `sym60` da `asym100` senza codificare la shape della coppia.
- [ ] Solo dopo la verifica offline, introdurre eventuali nuovi termini in
      `Trajectory Generator/baseline_MLP/reward_function.py` con pesi deboli e
      ablation dedicata.
- [ ] Chiarire perche' `online_grf_left_basis_03` prende carico dominante in
      `asym100` durante il burst e confrontare COP online vs prescribed/oracle
      nella stessa fase.

### Propagati dai report precedenti

- [ ] Nei confronti futuri loggare/confrontare sempre `qdot_ref`, `qdot_cas`,
      `cascade_velocity_error`, contributo P, contributo I e `tau_spring`.
- [ ] Aggiungere ai plot diagnostici MLP i termini interni del cascade, almeno
      per knee e ankle.
- [ ] Separare esplicitamente nei report futuri:
  - qualita' della reference servita;
  - tracking SEA della reference;
  - segno/scala della coppia prodotta dal cascade.
- [ ] Verificare ancora il nodo del target imitation ankle:
      `ankle_angle_r` sound-leg anti-phase vs `pros_ankle_angle` / offset-range
      protesico.
- [ ] Non interpretare una curva `tau_spring` piu' simile all'healthy come
      successo globale se reserve, scala della coppia e lavoro ankle restano
      lontani dal riferimento.
- [ ] Valutare una testa di output bounded/squashed, ad esempio `tanh` scalato
      ai bound fisici, per ridurre la dipendenza dal clipping hard dell'azione.
- [ ] Eseguire confronti piu' robusti simmetrico/asimmetrico:
  - training simmetrico da zero `1 -> 100` con stessa config;
  - piu' seed;
  - stesso OS e stesso numero di env runner.
- [ ] Monitorare in TensorBoard:
  - served/imitation/tracking losses;
  - `policy_action_clip_loss`;
  - `policy_action_clip_fraction`;
  - reward components;
  - value metrics;
  - diagnostici `grf_ankle_moment_flip_*`.
- [ ] Valutare una ablation separata su `vf_clip_param` solo dopo i confronti
      actor-critic e reward.
- [ ] Non introdurre nella reward profili prescribed di coppia protesica o
      termini controller-specifici come `outer_i_cmd`.
- [ ] Continuare a monitorare ankle torque, knee torque, command-rate, action
      clipping, reserve/root load e GRF penetration.
- [ ] Validare heel-strike online, `in_contact`, rocker/COP push-off e contatto
      protesico.
- [ ] Ridurre carico/penetrazione del piede protesico senza perdere contatto.
- [ ] Investigare reserve biologiche elevate e richieste sui DOF non attuati,
      incluso il FAIL di tracking biologico `mtp_angle_r`.
- [ ] Progettare e validare la reward ex-novo task-based prima del warm-start.
- [ ] Implementare prima le nuove metriche ex-novo in modalita diagnostica.
- [ ] Validare auto-recovery durante un crash nativo Ray reale.
- [ ] Verificare su macOS arm64 cleanup, resume, RLModule e reward mode.
- [ ] Pulire launcher/log temporanei e artefatti di smoke quando non servono piu'.
- [ ] Proseguire i TODO SNN/skrl propagati e i TODO storici SEA ancora aperti.
- [ ] Per la linea MuJoCo/MJX: verificare stato VCS, creare matrice
      cross-platform, produrre oracle OpenSim canonici, chiudere gate statici e
      integrare ambiente JAX/MJX batched prima di PPO JAX.
