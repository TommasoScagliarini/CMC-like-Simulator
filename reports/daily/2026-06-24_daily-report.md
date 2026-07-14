# Daily report - 2026-06-24

Instruction check token: CMC_AGENT_OK_2026

## Sintesi

La giornata e' stata dedicata alla chiusura dell'ablation GRF soft e alla
preparazione del prossimo training MLP imitativo.

Risultato principale:

```text
Il rollout GRF soft e' il miglior compromesso recente tra imitation, contatto
online e reserve, ma non e' ancora una soluzione finale: l'ankle migliora nel
segno della coppia, mentre il knee resta il canale critico per chattering e
clipping.
```

In chiusura e' stato aggiornato `Trajectory Generator/baseline_MLP/training_cfg.yaml`
per rendere persistente il setup GRF soft e preparare una nuova ablation
knee-specific sul limite di jerk:

```yaml
grf_penetration_weight: 1.0
grf_ankle_moment_flip_weight: 0.10
pros_knee_ref_acceleration_limit_rad_s2: 60.0
pros_knee_ref_jerk_limit_rad_s3: 2000.0
```

Non sono stati modificati plugin C++, semantica SEA o codice del simulatore
root.

## Report utente consolidati

Report creati oggi:

```text
reports/user/2026-06-24_analisi_rollout_grf_soft.md
reports/user/2026-06-24_config_grf_soft_knee_jerk2000.md
```

## Analisi rollout GRF soft

Artefatti principali analizzati:

```text
training: Trajectory Generator/runs/training/MLP_imitation_training_06-23-2026_grfsoft_knee1_ankle2_100iter
rollout : Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-23-2026_grfsoft_knee1_ankle2_100iter
plot    : plot/06_24_2026_1_asym100_GRFpenalty-lowered
```

Il training soft-GRF era partito il 2026-06-23 e risulta completato il
2026-06-24:

```text
training completed: 100 / 100 iterations
best train return: 293.595
best iteration: 94
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

Interpretazione:

- `GRF hard`: troppo severo; la policy evita il problema alterando contatto e
  knee.
- `knee_pos=3`: direzione sbagliata; peggiora tracking, return, command-rate e
  clipping.
- `GRF soft`: direzione giusta; migliora imitation, contatto e reserve, ma
  sposta il costo su clipping e chattering knee.

Il plant segue bene la reference servita:

```text
soft GRF latest:
knee served -> actual q RMSE   0.00260 rad
ankle served -> actual q RMSE  0.00474 rad
```

Quindi il collo di bottiglia non e' il tracking fisico SEA della reference, ma
la forma della reference scelta dalla policy e il compromesso contatto/GRF.

## Coppia SEA e chattering

Le metriche SEA sono state ricalcolate scartando i primi `0.2 s` di transiente:

```text
case              knee RMS   knee HP20 ratio   ankle RMS   ankle area + / -
sym60             8.06 Nm    0.331             5.68 Nm     +1.46 / -15.90
asym100 no flip   13.17 Nm   0.231             5.52 Nm     +9.06 / -6.01
hard GRF 100      7.03 Nm    0.356             2.81 Nm     +2.36 / -5.95
soft GRF latest   6.73 Nm    0.466             4.11 Nm     +1.70 / -11.30
```

Finding:

- l'ankle del soft-GRF si sposta verso una forma piu' negativa, vicina a
  `sym60`, e non mostra il burst positivo sospetto di `asym100`;
- il knee ha RMS non enorme, ma quota high-pass piu' alta del gruppo;
- nei plot `05_time_tau_input_tracking_error.png` e
  `01_time_sea_control_reserve.png`, il knee mostra chiaramente `tau_input` e
  `tau_spring` ad alta frequenza;
- il plot `missing_channels.txt` del set `06_24_2026_1_asym100_GRFpenalty-lowered`
  riporta `No missing channels.`

## Aggiornamento training_cfg

Il config sorgente e' stato aggiornato per evitare di lasciare il default sui
pesi hard-GRF:

```yaml
reward:
  grf_penetration_weight: 1.0
  grf_ankle_moment_flip_weight: 0.10
```

Inoltre e' stata preparata una ablation conservativa sul knee jerk:

```yaml
simulation:
  pros_knee_ref_acceleration_limit_rad_s2: 60.0
  pros_knee_ref_jerk_limit_rad_s3: 2000.0
```

Razionale:

- nel rollout soft-GRF il limite di accelerazione knee non era il vincolo
  dominante;
- il knee aveva `served_acceleration` con max circa `42.5 rad/s^2`, p95 circa
  `36.0 rad/s^2`, sotto il limite `60.0`;
- il jerk knee arrivava al cap `3000 rad/s^3` e il `jerk_limit_fraction` medio
  era circa `0.475`;
- quindi il primo test ragionevole e' abbassare solo il jerk knee a `2000`,
  senza scendere subito con acceleration a `30`.

Stato finale verificato:

```text
knee_acc   60.0
knee_jerk  2000.0
ankle_acc  55.0
ankle_jerk 2750.0
grf_pen    1.0
grf_flip   0.1
iterations 100
```

Il confronto programmatico con il resolved config del run soft-GRF ha confermato
che, sui campi condivisi, l'unica nuova differenza intenzionale e':

```text
simulation.pros_knee_ref_jerk_limit_rad_s3:
  current   = 2000.0
  reference = 3000.0
```

## File modificati oggi

Modificati:

```text
Trajectory Generator/baseline_MLP/training_cfg.yaml
```

Creati:

```text
reports/user/2026-06-24_analisi_rollout_grf_soft.md
reports/user/2026-06-24_config_grf_soft_knee_jerk2000.md
reports/daily/2026-06-24_daily-report.md
```

Artefatti gia' presenti o generati dal training/rollout analizzato:

```text
Trajectory Generator/runs/training/MLP_imitation_training_06-23-2026_grfsoft_knee1_ankle2_100iter
Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-23-2026_grfsoft_knee1_ankle2_100iter
plot/06_24_2026_1_asym100_GRFpenalty-lowered
```

Gia presenti nel worktree prima o comunque non oggetto di revert durante la
giornata:

```text
Trajectory Generator/baseline_MLP/README.md
Trajectory Generator/baseline_MLP/env_factory.py
Trajectory Generator/baseline_MLP/reward_function.py
Trajectory Generator/baseline_MLP/rollout_eval.py
Trajectory Generator/baseline_MLP/tb_logging.py
Trajectory Generator/osim_trj_cmc_like.py
Trajectory Generator/runs/historical_runs.index.json
Trajectory Generator/runs/historical_runs.md
paper/
reports/daily/2026-06-19_daily-report.md
reports/daily/2026-06-21_daily-report.md
reports/daily/2026-06-22_daily-report.md
reports/daily/2026-06-23_daily-report.md
reports/user/2026-06-19_*.md
reports/user/2026-06-21_*.md
reports/user/2026-06-22_*.md
reports/user/2026-06-23_*.md
```

## Verifiche eseguite

- Lettura e analisi di:
  - `rollout_summary.json`;
  - `summary.json`;
  - `rollout_policy_trace.json`;
  - `rollout_episode_kinematics.sto`;
  - `rollout_episode_kinematics_reference.sto`;
  - `rollout_episode_sea_diagnostics.sto`;
  - `rollout_episode_online_grf.sto`;
  - `rollout_episode_reserve_torques.sto`;
  - `rollout_episode_power.sto`.
- Confronto con:
  - hard-GRF 100;
  - `knee_pos=3` 40 iter;
  - `asym100` senza flip guard;
  - `sym60`.
- Calcolo di:
  - return e reward mean;
  - clipping totale e per giunto;
  - served imitation loss;
  - RMSE target -> served -> actual;
  - `tau_spring` RMS, min/max, area positiva/negativa;
  - high-pass ratio a 20 Hz;
  - left GRF mean force, contact fraction e penetration;
  - root reserve RMS.
- Ispezione dei plot diagnostici MLP:
  - `01_time_sea_control_reserve.png`;
  - `03_gaitcycle_torque_angle_power.png`;
  - `05_time_tau_input_tracking_error.png`;
  - `06_time_joint_ref_sea_error.png`;
  - `07_mlp_policy_vs_sound_leg_error.png`.
- Parsing YAML di `Trajectory Generator/baseline_MLP/training_cfg.yaml`.
- `git diff --check -- Trajectory Generator/baseline_MLP/training_cfg.yaml`.
- `git diff --check` sui due user report creati oggi.

## TODO chiusi o avanzati il 24/06

- [x] Eseguire e valutare il rollout del training GRF soft.
- [x] Generare e ispezionare i plot MLP del run GRF soft.
- [x] Confrontare `GRFpenalty 100`, `knee_pos=3.0 40iter` e `GRF soft`.
- [x] Confermare che ulteriori aumenti globali del peso knee non sono la
      direzione immediata.
- [x] Aggiornare `training_cfg.yaml` dai pesi hard-GRF ai pesi GRF soft.
- [x] Verificare se abbassare acceleration/jerk knee e' sensato. Esito:
      abbassare il jerk knee a `2000`, mantenere acceleration knee a `60`.
- [x] Preparare il prossimo training da zero come ablation:
      GRF soft + knee jerk `2000`.

## TODO aperti e propagati

### Prioritari dal 24/06

- [ ] Lanciare un nuovo training da zero con config attuale:
      GRF soft + `pros_knee_ref_jerk_limit_rad_s3: 2000.0`.
- [ ] Dopo il training, eseguire rollout deterministico del `rl_module_best` e
      generare plot MLP.
- [ ] Confrontare il nuovo run contro `soft_grf_latest` su:
  - return;
  - clipping totale e per giunto;
  - knee `tau_spring` RMS e high-pass ratio;
  - knee `tau_input_plugin` high-pass ratio;
  - served imitation loss;
  - knee target -> served -> actual;
  - ankle `tau_spring` area positiva/negativa;
  - online left Fy medio;
  - reserve root RMS;
  - GRF penetration e flip loss.
- [ ] Analizzare il knee soft-GRF e il futuro knee-jerk2000 per fase del passo:
      clipping, command-rate, `tau_spring` high-pass e `tau_input_plugin`
      high-pass, separando stance e swing.
- [ ] Confrontare il knee contro `sym60` come baseline anti-chattering, senza
      considerare `sym60` un successo globale perche' reserve, scala della
      coppia ankle e lavoro ankle restano non risolti.
- [ ] Se il knee chattering migliora senza peggiorare ankle e imitation,
      considerare una seconda ablation con acceleration knee moderata
      (`45 rad/s^2`), non `30 rad/s^2` come primo passo.
- [ ] Se clipping o lag peggiorano molto, tornare a jerk `3000` e valutare
      mitigazioni alternative: bounded action head o penalty knee-specific.

### Ancora rilevanti dal 23/06

- [ ] Non proseguire con ulteriori aumenti globali del peso knee prima di avere
      diagnosticato il chattering knee.
- [ ] Non applicare subito `sea_tau_spring_rate_weight` globale: prima
      verificare se serve una penalty knee-specific per non danneggiare ankle.
- [ ] Dopo ogni ablation, generare rollout e plot MLP, confrontando
      `tau_spring` knee/ankle, `tau_input_plugin`, tracking reference,
      episode return, clipping e GRF penetration.

### Propagati dai report precedenti

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
