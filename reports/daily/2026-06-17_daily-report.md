# Daily report - 2026-06-17

## Sintesi

Giornata centrata sul riordino storico dei training MLP e sull'analisi del
rollout del training imitativo `40+20` iterazioni con reward normalizzata.

Punti principali:

- introdotto un registry storico leggibile e rigenerabile per le run MLP;
- analizzato il rollout `baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm`;
- quantificata la debolezza del critic PPO;
- diagnosticata la forma non corretta dei grafici torque/angle;
- definita la prossima comparazione sperimentale: baseline simmetrica estesa
  contro asymmetric actor-critic;
- rilevato un resume baseline verso `100` iterazioni in corso.

Non sono state modificate la semantica SEA, il plugin C++ o la logica del
simulatore root.

## Report utente consolidati

Report creati il 17/06:

- `reports/user/2026-06-17_historical_runs_registry_training_mlp.md`;
- `reports/user/2026-06-17_analisi_rollout_fullobs_normreward_60iter.md`.

## Registry storico dei training MLP

### Problema

Il contesto dei training MLP era distribuito fra cartelle `runs/training`,
summary JSON, daily report e memoria operativa. Questo rendeva difficile capire
quale run fosse davvero una baseline completa, quale fosse un resume, e quali
esperimenti fossero interrotti o solo parzialmente validi.

### Soluzione

E' stato introdotto un registry storico sotto:

```text
Trajectory Generator/runs/historical_runs.md
Trajectory Generator/runs/historical_runs.index.json
Trajectory Generator/runs/historical_runs.manual.yaml
```

Il Markdown e' il documento principale da leggere. L'indice JSON e' generato,
mentre lo YAML manuale serve per annotazioni umane, lineage e backfill storico.

Lo script dedicato e':

```text
Trajectory Generator/baseline_MLP/update_historical_runs.py
```

`train_ppo_mlp.py` aggiorna il registry automaticamente in modalita' best-effort
dopo la scrittura di `summary.json`, con flag:

```text
--update-history
--no-update-history
```

### Backfill iniziale

Il registry iniziale contiene:

- 3 run rilevate da `Trajectory Generator/runs/training`;
- 6 note storiche estratte dai report.

Run rilevate:

- `baseline_mlp_imit_v4_c2_4hz_obs_target`;
- `baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm`;
- `MLP_imitation_training_06-16-2026`.

La run `MLP_imitation_training_06-16-2026` resta marcata come `interrupted` e
non viene presentata come baseline completata.

## Analisi rollout FullObs normReward 60iter

### Artefatti analizzati

```text
Trajectory Generator/runs/training/baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm
Trajectory Generator/runs/rollout/baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm_rollout
plot/06_17_2026_1 - imititation_FullObs_normReward_60iter
```

### Risultato rollout

Il rollout e' stabile e completa l'episodio da `5 s`:

```text
steps = 501
terminated = false
truncated = true
episode_return = 334.7528621460614
action_clipped_fraction = 0.0
pelvis_ty_min = 0.9518210480919542 m
```

Il tracking fisico SEA rispetto alla reference servita e' molto buono:

```text
knee actual-served RMSE  = 0.00336 rad
ankle actual-served RMSE = 0.00545 rad
```

L'errore principale resta quindi a monte, nella reference generata/servita dalla
policy rispetto al target sano anti-fase:

```text
knee served-target RMSE  = 0.119 rad
ankle served-target RMSE = 0.0449 rad
knee served-target corr  = 0.978
ankle served-target corr = 0.944
```

L'ankle non appare piu' semplicemente compresso verso la media:

```text
ankle target span = 0.355 rad
ankle served span = 0.442 rad
```

Il knee mantiene invece offset/compressione:

```text
knee target span = 0.933 rad
knee served span = 0.871 rad
knee mean served-target error ~= -0.073 rad
```

## Critic PPO

Metriche dal training
`baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm`:

```text
best iteration = 59
best return    = 277.75698812995

iter 59:
  vf_loss clipped    = 2.135979175567627
  vf_loss_unclipped  = 29.469728469848633
  vf_explained_var   = 0.10315835475921631

iter 60:
  vf_loss clipped    = 2.7540037631988525
  vf_loss_unclipped  = 54.7823600769043
  vf_explained_var   = 0.113750159740448

last 5 mean:
  vf_loss clipped    = 2.5935944080352784
  vf_loss_unclipped  = 41.98524894714355
  vf_explained_var   = 0.08470591306686401
```

La configurazione usa:

```yaml
ppo:
  vf_clip_param: 10.0
  vf_loss_coeff: 1.0
```

Nella versione RLlib installata, `vf_loss` e' la squared error del value
clampata fra `0` e `vf_clip_param`, mentre `vf_loss_unclipped` e' la MSE raw.
Con `vf_clip_param = 10`, errori value con modulo maggiore di circa `3.16`
sono cappati nella loss ottimizzata.

Interpretazione:

- il critic non e' nullo, ma resta debole;
- `vf_explained_var ~0.10` indica che spiega solo una piccola parte della
  varianza dei target;
- il grande gap fra clipped e unclipped mostra che molti errori raw sono
  nascosti dal clipping;
- alzare solo `vf_clip_param` non e' una soluzione pulita, perche' puo'
  aumentare il gradiente sugli errori grossi senza dare al critic informazione
  migliore;
- l'asymmetric actor-critic e' la prova piu' sensata da fare prima, lasciando
  actor e reward confrontabili.

## Diagnosi torque/angle

La figura:

```text
plot/06_17_2026_1 - imititation_FullObs_normReward_60iter/03_gaitcycle_torque_angle_power.png
```

non rappresenta un loop biomeccanico healthy corretto.

Cause principali:

- la policy e' ottimizzata soprattutto per imitation cinematica e tracking
  della served reference, non per un momento-angolo fisiologico;
- il grafico torque/angle e' parametrico nel tempo, quindi allo stesso angolo
  possono corrispondere coppie diverse in stance e swing;
- il plotter media `angle(gait%)` e `torque(gait%)`, poi disegna
  `mean_torque` contro `mean_angle`;
- con `x` non monotona, anche `fill_between` diventa visivamente fuorviante.

Nel rollout entrano solo due cicli sinistri completi:

```text
13.94687098 -> 15.61596084 s
15.61596084 -> 17.16396799 s
```

Numeri sui cicli usati:

```text
ankle:
  mean angle range  ~= 0.057 -> 0.473 rad
  mean torque range ~= -13.04 -> 1.67 Nm

knee:
  mean angle range  ~= 0.331 -> 1.060 rad
  mean torque range ~= -18.55 -> 13.95 Nm
```

Il knee e' il piu' sospetto: a pari angolo la coppia cambia spesso segno e la
motor power e' ancora rumorosa nella prima parte del gait cycle. Questo e'
coerente con una policy che segue bene una reference cinematica, ma non e'
ancora vincolata dinamicamente.

## Training e resume rilevati oggi

### Resume a 80 iterazioni

Run:

```text
Trajectory Generator/runs/training/MLP_imitation_training_06-17-2026_resume_best_to_80
```

Stato da `summary.json`:

```text
ok = true
logical target = 80
iterations_run = 45
iterations_completed = 80
iteration_start = 36
next_iteration = 81
best_episode_return_mean = 263.05089784113886
best logical iteration = 65
```

Ultima iterazione visibile:

```text
iter 80:
  episode_return_mean = 262.89026517433626
  vf_loss             = 2.3201889991760254
  vf_loss_unclipped   = 59.220951080322266
  vf_explained_var    = 0.08890056610107422
```

Nota operativa: alcuni campi di output path nel summary risultano duplicati
(`Trajectory Generator/runs/training/Trajectory Generator/runs/training/...`) e
includono path assoluti macOS nel `resume_from`. Questo non invalida
necessariamente la run, ma va considerato quando si costruiscono comandi futuri
o si aggiorna il registry.

### Resume verso 100 iterazioni

Run:

```text
Trajectory Generator/runs/training/MLP_imitation_training_06-17-2026_resume_best_to_100
```

Stato al momento del report:

- processo Python ancora attivo: PID `46164`;
- `watchdog_state.json` indica fase `algo.train iteration 66`;
- `summary.json` non ancora presente;
- `checkpoint_best_meta.json` non ancora presente;
- presenti `training_cfg.resolved.yaml`, `watchdog_state.json`,
  `faulthandler.log`, `rllib/` e `tensorboard/`.

Quindi il ramo `_to_100` e' da considerare in corso, non completato.

## File modificati o creati

File creati o aggiornati dal lavoro sul registry:

```text
.gitignore
Trajectory Generator/baseline_MLP/train_ppo_mlp.py
Trajectory Generator/baseline_MLP/update_historical_runs.py
Trajectory Generator/runs/historical_runs.md
Trajectory Generator/runs/historical_runs.index.json
Trajectory Generator/runs/historical_runs.manual.yaml
reports/user/2026-06-17_historical_runs_registry_training_mlp.md
```

File creato dall'analisi rollout:

```text
reports/user/2026-06-17_analisi_rollout_fullobs_normreward_60iter.md
```

File creato da questo comando `end_day`:

```text
reports/daily/2026-06-17_daily-report.md
```

## Verifiche eseguite

Registry:

- `python -m py_compile` su updater e training script;
- esecuzione di `update_historical_runs.py`;
- verifica rendering iniziale del Markdown;
- verifica raggruppamento della catena `40+20`;
- verifica stato `interrupted` della run non completata;
- verifica assenza di path assoluti in `historical_runs.index.json`;
- smoke test dell'updater con run temporanee mancanti/parziali;
- `git diff --check`, con soli warning CRLF/LF attesi su Windows.

Rollout e training:

- lettura di `rollout_summary.json`;
- lettura di `rollout_policy_trace.json`;
- lettura di `rollout_reset_diagnostics.json`;
- analisi visuale delle figure `01`-`07`;
- calcolo RMSE served-target e actual-served;
- calcolo range target/served/actual e correlazioni;
- verifica action clipping;
- analisi di `sim_outputs/*.sto` per torque SEA, motor power, reserve/root load
  e segnali GRF;
- verifica del codice RLlib locale per la semantica di `vf_loss`;
- verifica del codice `plot/plotter.py` per capire quali segnali entrano nella
  figura torque/angle;
- controllo dello stato dei run `_to_80` e `_to_100`.

## TODO chiusi o avanzati il 17/06

- [x] Creare un registro storico dei training MLP.
- [x] Automatizzare l'aggiornamento del registro storico dopo i training.
- [x] Analizzare il rollout del training `40+20` con reward normalizzata.
- [x] Quantificare `vf_loss`, `vf_loss_unclipped` e `vf_explained_var`.
- [x] Diagnosticare la forma anomala dei grafici torque/angle.
- [x] Preparare il confronto sperimentale baseline estesa vs asymmetric
      actor-critic.
- [ ] Monitorare fino a completamento il resume baseline `_to_100`, attualmente
      in corso.

## TODO aperti e propagati

### Training imitation e confronto

- [ ] Eseguire un training asymmetric actor-critic da zero a `100` iterazioni,
      mantenendo invariati reward e `vf_clip_param`.
- [ ] Completare o verificare il resume della baseline simmetrica dal best del
      run a `80` iterazioni fino a `100` iterazioni.
- [ ] Confrontare i due rami con rollout deterministici usando metriche fisiche
      e non solo il return.
- [ ] Dopo i rollout dai nuovi best, confrontare `served-target`,
      `actual-target`, saturazione SEA, clipping/aggressivita' del comando e
      forma cinematica rispetto al target sano anti-fase.
- [ ] Controllare in TensorBoard `sound_imitation_loss`,
      `served_imitation_loss`, sottotermini position/velocity knee/ankle,
      `tracking_position_loss`, `tracking_velocity_loss`,
      `grf_penetration_loss`, `segment_delta_loss`, entropy,
      predetto-vs-return e saturazione delle azioni.
- [ ] Valutare una ablation separata su `vf_clip_param` solo dopo il confronto
      asymmetric vs baseline estesa.
- [ ] Valutare anche un training da zero con reward normalizzata, se serve
      distinguere bias del resume e local optimum ereditati.

### Plot e metriche

- [ ] Creare o correggere una figura torque/angle piu' robusta: cicli
      individuali, colore per gait percentage, niente `fill_between` su `x` non
      monotona.
- [ ] Aggiungere al confronto metriche come served-target RMSE, actual-served
      RMSE, knee offset, action turn fraction, SEA torque error, motor power,
      reserve/root load e shape del gait-cycle torque/angle.

### Forma della served reference

- [ ] Verificare se i pesi ankle `2.0 / 0.04` recuperano ampiezza senza creare
      overshoot o peggiorare knee.
- [ ] Indagare ancora la polarizzazione/compressione della served reference
      ankle dopo il primo secondo.
- [ ] Valutare una penalita' inter-step esplicita su `q_cmd(t)-q_cmd(t-1)` o
      sull'endpoint consecutivo solo se il chattering raw degrada served
      reference, SEA o stabilita' numerica.
- [ ] Valutare `lam: 0.95` in un'ablation separata se il credito temporale resta
      troppo locale.

### Reset, fase e stati iniziali

- [ ] Implementare una selezione robusta degli stati iniziali che verifichi
      compatibilita' fra posa, velocita', target imitativo e pattern di contatto.
- [ ] Escludere o gestire esplicitamente il tratto precedente al primo heel
      strike invece di affidarsi alla retro-estrapolazione.
- [ ] Prima di abilitare `random_init=true`, validare molte gait phase e
      rifiutare stati iniziali dinamicamente incoerenti.
- [ ] Valutare se mantenere il target periodico medio oppure costruire target
      condizionati anche sullo stato di contatto.

### Dinamica, contatto e reserve

- [ ] Valutare separatamente penetrazioni GRF e uso elevato delle reserve, senza
      confonderli con il tracking SEA.
- [ ] Investigare reserve biologiche elevate e richieste sui DOF non attuati.
- [ ] Investigare il FAIL di tracking biologico `mtp_angle_r`.
- [ ] Validare heel-strike online, `in_contact`, rocker/COP push-off e contatto
      online del lato sano.
- [ ] Ridurre carico e penetrazione del piede protesico senza perdere contatto.

### Ex-novo, runtime e filoni storici

- [ ] Progettare e validare la reward ex-novo task-based prima del warm-start.
- [ ] Confermare formalmente impulso propulsivo protesico e coordinazione
      inter-limb come obiettivo primario V1.
- [ ] Implementare prima le nuove metriche ex-novo in modalita diagnostica.
- [ ] Validare auto-recovery durante un crash nativo Ray reale.
- [ ] Verificare su macOS arm64 cleanup, resume, RLModule e reward mode.
- [ ] Pulire launcher/log temporanei `run_imit_*.ps1` e `imit_*.log`.
- [ ] Pulire artefatti temporanei `results/_*`, `validation/_*` e smoke V4
      quando non piu' necessari.
- [ ] Proseguire i TODO SNN/skrl propagati e i TODO storici SEA ancora
      applicabili.

### Porting MuJoCo/MJX

- [ ] Verificare lo stato VCS di `C:\Users\tomma\Desktop\MuJoCo_env` prima di
      iniziare il porting.
- [ ] Creare la matrice obbligatoria
      `trajectory_generator/baseline_mlp/feature_parity.yaml`.
- [ ] Eseguire la Fase 0 e produrre il GO/NO-GO GPU/hybrid.
- [ ] Generare e congelare gli oracle OpenSim canonici.
- [ ] Chiudere gate statici, SEA, replay strict e contatto hybrid.
- [ ] Costruire l'ambiente JAX/MJX batched e chiudere i benchmark engine.
- [ ] Integrare PPO JAX, rollout, supervisione, checkpoint e UX.
