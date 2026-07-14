# 2026-06-21 daily report

Instruction check token: CMC_AGENT_OK_2026

## Sintesi

La giornata e' stata centrata sulla famiglia di training imitativi MLP in
`Trajectory Generator/`, in particolare:

- completamento valutazione del rollout piu recente;
- reset e successivo tuning controllato di `training_cfg.yaml`;
- diagnosi approfondita del caso `sym60`;
- confronto tra actor-critic simmetrico e asimmetrico;
- analisi cinematica posizione/velocita/accelerazione per scegliere i pesi reward;
- verifica dell'ipotesi "qdot chattery -> velocity weight -> clipping";
- aggiornamento dei TODO su clipping hard vs testa bounded/squashed.

Non sono stati modificati plugin C++, semantica SEA o codice del simulatore root.

## Report utente inclusi

Report del 21/06 inclusi:

- `reports/user/2026-06-21_analisi_sym60_coppia_ankle_e_reset_asym100.md`

Richiesta esplicita: includere anche gli user report di ieri. Controllo eseguito:
non risultano file `reports/user/2026-06-20_*.md`.

Sono stati quindi richiamati anche i due user report immediatamente precedenti e
ancora rilevanti del 19/06:

- `reports/user/2026-06-19_confronto_sym60_asym100_tau_spring_ankle.md`
- `reports/user/2026-06-19_diagnosi_tau_spring_target_imitativo_reserve.md`

## Rollout e plot del run recente

Il rollout `Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-19-2026`
risultava completato a 501 step.

Metriche principali:

```text
episode_return             325.449
steps                      501
terminated                 false
truncated                  true
action_clipped_fraction    0.1567
raw action max             1.6399
```

Tracking cinematico vs served reference:

```text
ankle q RMSE    0.00861 rad
knee  q RMSE    0.00408 rad
ankle qdot RMSE 0.171 rad/s
knee  qdot RMSE 0.147 rad/s
```

Plot generati con `plot/plotter.py --mlp` in:

```text
plot/06_20_2026_1
```

Verdetto operativo: non promuovere questo rollout. La cinematica e' tracciata,
ma la `tau_spring` ankle resta biomeccanicamente problematica e il clipping
dell'azione e' troppo alto.

## Reset config e stato finale

Prima e' stata ripristinata la famiglia `asym100` in:

```text
Trajectory Generator/baseline_MLP/training_cfg.yaml
```

Poi, dopo le analisi cinematiche e di clipping, il config e' stato aggiornato
dall'utente e verificato. Stato finale rilevante:

```yaml
model:
  asymmetric_actor_critic: true
  seed: 123

parallelism:
  num_env_runners: 13
  ray_num_cpus: 14

simulation:
  iterations: 100
  episode_duration: 5.0
  episode_start_offset_s: 1.0
  policy_knots: 1
  pros_ref_model: butterworth3_jerk_limited
  pros_ref_cutoff_hz: 4.0

reward:
  imitation_knee_position_weight: 1.0
  imitation_ankle_position_weight: 2.0
  imitation_knee_velocity_weight: 0.02
  imitation_ankle_velocity_weight: 0.12
  blend_served_imitation: 0.80
  blend_imitation: 0.20
  blend_imitation_tracking: 0
  smoothness_weight: 0.02
  command_rate_weight: 0.05
  sea_tau_spring_effort_weight: 0.0
  sea_tau_spring_rate_weight: 0.0
  policy_action_clip_weight: 0.0
```

Questa configurazione isola l'ablation principale: aumentare il peso velocita'
ankle senza aumentare il peso posizione knee.

## Diagnosi sym60

Run analizzati:

```text
training: Trajectory Generator/runs/training/baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm
rollout : Trajectory Generator/runs/rollout/baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm
```

Config `sym60`:

```text
asymmetric_actor_critic: false
iterations: 60
reward_mode: imitation
blend_served_imitation: 0.8
blend_imitation: 0.2
blend_imitation_tracking: 0.0
sea_tau_spring_effort_weight: 0.0
sea_tau_spring_rate_weight: 0.0
policy_action_clip_weight: 0.0
```

Il training locale e' un resume segment 41 -> 60. Best checkpoint alla
iterazione logica 59:

```text
iter 41 return  26.2
iter 49 return 220.4
iter 57 return 261.2
iter 58 return 275.7
iter 59 return 277.8  best
iter 60 return 270.8
```

Risultato rollout:

```text
episode_return             334.75
steps                      501
action_clipped_fraction    0.0
raw action max             0.966
ankle q RMSE vs served     0.00545 rad
knee  q RMSE vs served     0.00336 rad
```

Diagnosi principale: la forma della coppia ankle di `sym60` non deriva da una
reward fisica diretta o da un tracking SEA migliore. Deriva dalla reference
cinematica generata dalla policy, che dentro il cascade produce un integrale
ankle negativo.

Numeri chiave ankle:

```text
tau_spring RMS        6.16 Nm
tau_spring min/max    -21.78 / +1.76 Nm
cascade P RMS         0.86 Nm
cascade I RMS         6.02 Nm
corr(tau_spring, I)   0.985
corr(tau_spring, P)   0.164
I mean               -3.31 Nm
```

Istante diagnostico:

```text
t = 13.051 s
tau_spring   = -21.78 Nm
q            = 0.382 rad
q_ref        = 0.357 rad
qdot_ref     = +0.288 rad/s
qdot_cas     = -0.884 rad/s
velocity_err = -0.684 rad/s
P            = -1.94 Nm
I            = -19.96 Nm
```

Catena causale:

```text
policy reference -> reference model -> qdot_cas -> velocity PI integral -> tau_spring
```

Confronto ankle `tau_spring`:

```text
case          RMS Nm   area positiva   area negativa   I mean
sym60          6.16      +1.48 Nms      -18.03 Nms     -3.31
asym100        5.74      +9.09 Nms       -7.44 Nms     +0.33
asym200        7.33     +13.74 Nms       -5.59 Nms     +1.63
new asym100   13.05     +32.00 Nms       -4.35 Nms     +5.57
05/23 CMC     49.31      +1.78 Nms     -160.89 Nms    -31.79
```

`sym60` assomiglia al 05/23 nella direzione/forma negativa, ma non nella scala:
la coppia e' circa 8x troppo piccola e le reserve restano alte.

## Actor-critic simmetrico vs asimmetrico

Il dubbio era se la curva `sym60` derivasse dal critic simmetrico.

Dal codice:

- in modalita asimmetrica la policy legge sempre solo il prefisso actor;
- il critic legge il vettore completo privilegiato;
- quindi l'effetto dell'asimmetrico e' sulla qualita' degli advantage durante il
  training, non su segnali extra disponibili in inference.

Confronto disponibile:

```text
case          AC       iter  train_best  rollout  clip   tau_rms  area+   area-
sym60         sym       60      277.8     334.8   0.000    6.16    1.48  -18.03
sym80         sym       80      263.1     348.1   0.016   13.69   30.68   -5.92
sym100        sym      100      292.5     368.1   0.078    9.17   15.47   -7.46
asym100       asym     100      307.2     370.1   0.043    5.74    9.09   -7.44
asym200       asym     200      332.9     361.1   0.075    7.33   13.74   -5.59
```

Verdetto: usare actor-critic asimmetrico resta preferibile per stabilita' e
qualita' training. Pero' il critic simmetrico non spiega da solo la forma
specifica di `sym60`, perche `sym80` e `sym100` sono ancora simmetrici ma non
mantengono la stessa area ankle negativa.

## Analisi cinematica dei pesi reward

Artifact generati:

```text
Trajectory Generator/runs/analysis/kinematic_weight_diagnostics_2026-06-21/
```

File principali:

```text
kinematic_error_summary.csv
kinematic_signal_ranges.csv
reward_weight_balance.csv
command_regularization_terms.csv
ankle_served_minus_target_errors.png
knee_served_minus_target_errors.png
*_target_served_actual.png
```

Conclusione principale: la posizione e' gia' ragionevole; la velocita' e' il
canale che controlla fase, ampiezza dinamica e segno della coppia prodotta dal
cascade.

Per `asym100`:

```text
target -> served
           q RMSE     qdot RMSE   qdot corr
knee       0.0988     0.6954      0.968
ankle      0.0284     0.3656      0.870
```

Per `asym100_hiVel`:

```text
target -> served
           q RMSE     qdot RMSE   qdot corr
knee       0.0818     0.6728      0.940
ankle      0.0210     0.2232      0.962
```

Alzare il peso velocita' ankle migliora molto la cinematica ankle. Il run
`asym100_hiVel` pero' ha anche clipping elevato, soprattutto sul knee.

Reward balance `asym100`, target -> served:

```text
           peso vel   quota vel attuale   peso vel per quota 10%   20%
knee       0.020      1.29%              ~0.170                 ~0.382
ankle      0.040      3.53%              ~0.122                 ~0.274
```

Da qui la decisione finale di provare:

```yaml
imitation_knee_velocity_weight: 0.02
imitation_ankle_velocity_weight: 0.12
```

## Clipping, C2 e output bounded

E' stata chiarita la distinzione:

- clipping: agisce sulla action normalizzata `[-1, 1]` prima del mapping in
  endpoint angolare;
- filtro C2 / `butterworth3_jerk_limited`: smussa la traiettoria gia' clippata;
- il C2 non recupera informazione persa quando la raw action eccede il bound.

Nel run `asym100_hiVel`:

```text
asym100:
  knee clip  8.18%
  ankle clip 0.40%

asym100_hiVel:
  knee clip  30.74%
  ankle clip 0.60%
```

Il clipping alto deriva quasi tutto dal knee, non dall'ankle.

TODO aggiunto al report del 21/06: valutare una testa bounded/squashed, ad
esempio `tanh` scalato ai bound fisici, mantenendo diagnostica raw-vs-applied e
confronto con `policy_action_clip_loss`.

## Verifica ipotesi qdot chattery

Ipotesi verificata:

```text
qdot target/CMC-like nervosa
-> aumento velocity weight
-> la rete prova a inseguire chattering
-> endpoint piu estremi
-> clipping
```

Artifact generati:

```text
Trajectory Generator/runs/analysis/qdot_chattering_vs_clipping_2026-06-21/
```

File principali:

```text
trace_qdot_frequency_metrics.csv
policy_action_frequency_metrics.csv
clip_correlation_metrics.csv
highrate_qdot_hpf_metrics.csv
reward_velocity_loss_metrics.csv
ankle_trace_target_error_raw_action.png
ankle_qdot_hp10_ratio_bars.png
```

Risultato: l'ipotesi e' meccanicamente plausibile, ma non e' la causa primaria
del clipping osservato.

Il target ankle `qdot` e' lo stesso in tutti i run:

```text
ankle target qdot, trace 100 Hz:
RMS        = 0.741 rad/s
HP >10 Hz = 0.085 rad/s circa 11.4%
```

Il report storico del morning best conferma che il riferimento cinematico era
pulito e il chattering nasceva dal feedback reale:

```text
morning best ankle:
cascade qdot_ref HPF50 = 0.127 rad/s, 7.1%
joint_qdot HPF50       = 1.011 rad/s, 38.6%
velocity_err HPF50     = 1.013 rad/s
```

Nei rollout MLP high-rate:

```text
asym100 ankle:
served_qdot HPF50 ratio 0.021
actual_qdot HPF50 ratio 0.073

asym100_hiVel ankle:
served_qdot HPF50 ratio 0.021
actual_qdot HPF50 ratio 0.071
```

Quindi il target/reference non e' particolarmente chattery; il rumore HF resta
piu legato alla dinamica effettiva del joint/feedback. Il clipping alto del run
high-velocity resta soprattutto knee-side.

## File modificati oggi

Modificati direttamente durante la giornata:

```text
Trajectory Generator/baseline_MLP/training_cfg.yaml
reports/user/2026-06-21_analisi_sym60_coppia_ankle_e_reset_asym100.md
reports/daily/2026-06-21_daily-report.md
```

Creati:

```text
reports/user/2026-06-21_analisi_sym60_coppia_ankle_e_reset_asym100.md
Trajectory Generator/runs/analysis/kinematic_weight_diagnostics_2026-06-21/
Trajectory Generator/runs/analysis/qdot_chattering_vs_clipping_2026-06-21/
```

Gia presenti nel worktree prima o comunque non oggetto di revert:

```text
Trajectory Generator/baseline_MLP/rollout_eval.py
Trajectory Generator/baseline_MLP/training_cfg.v4_imitation.yaml
Trajectory Generator/runs/historical_runs.index.json
Trajectory Generator/runs/historical_runs.md
reports/daily/2026-06-19_daily-report.md
reports/user/2026-06-19_confronto_sym60_asym100_tau_spring_ankle.md
reports/user/2026-06-19_diagnosi_tau_spring_target_imitativo_reserve.md
paper/
```

## Verifiche eseguite

- Parsing YAML di `Trajectory Generator/baseline_MLP/training_cfg.yaml`.
- `git diff --check -- Trajectory Generator/baseline_MLP/training_cfg.yaml`.
- `git diff --check -- reports/user/2026-06-21_analisi_sym60_coppia_ankle_e_reset_asym100.md`.
- Analisi numeriche su:
  - `rollout_policy_trace.json`;
  - `rollout_episode_states.sto`;
  - `rollout_episode_kinematics_reference.sto`;
  - `rollout_episode_sea_diagnostics.sto`;
  - `results/sim_output_*`.
- Generazione CSV e PNG diagnostici per cinematica e chattering.
- Ispezione plot diagnostici principali.

Non sono stati lanciati nuovi training dopo il cambio finale del config.

## TODO chiusi o avanzati il 21/06

- [x] Valutare il rollout completato e generare plot.
- [x] Resettare la config sulla famiglia `asym100`.
- [x] Analizzare perche `sym60` produceva una forma ankle `tau_spring` diversa.
- [x] Verificare se actor-critic simmetrico fosse una causa sufficiente del caso
      `sym60`. Esito: concausa possibile, non spiegazione principale.
- [x] Analizzare posizione/velocita/accelerazione per scegliere i pesi reward.
- [x] Verificare l'ipotesi che il chattering del target `qdot` causasse clipping
      quando si aumenta `imitation_*_velocity_weight`. Esito: target non
      abbastanza chattery; clipping alto prevalentemente knee-side.
- [x] Aggiungere ai TODO la valutazione di una testa output bounded/squashed.

## TODO aperti e propagati

### Nuovi / aggiornati dal 21/06

- [ ] Eseguire training con config finale:
  `imitation_knee_velocity_weight: 0.02`,
  `imitation_ankle_velocity_weight: 0.12`, actor-critic asimmetrico, 100 iter.
- [ ] Dopo il training, eseguire rollout deterministico, plot MLP e valutazione
      con metriche:
  - episode return;
  - action clipping totale e per giunto;
  - `policy_action_clip_loss`;
  - ankle/knee `q`, `qdot`, `qddot` target -> served -> actual;
  - `tau_spring` RMS, min/max, area positiva/negativa;
  - reserve RMS e componenti principali.
- [ ] Monitorare specificamente il clipping knee: nei run precedenti il clipping
      alto non era ankle-side ma knee-side.
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
- [ ] Non interpretare una curva `tau_spring` piu simile all'healthy come
      successo globale se reserve, scala della coppia e lavoro ankle restano
      lontani dal riferimento.
- [ ] Valutare una testa di output bounded/squashed, ad esempio `tanh` scalato
      ai bound fisici, per ridurre la dipendenza dal clipping hard dell'azione.
      Mantenere comunque diagnostica raw-vs-applied e confrontare con
      `policy_action_clip_loss`.
- [ ] Se la velocity imitation migliora ma l'accelerazione/reference diventa
      aggressiva, valutare una loss target-vs-served su `qddot` invece di usare
      solo `qddot_ref_weight`, che penalizza magnitudine e non match del target.

### Propagati dai report utente del 19/06

- [ ] Verificare se il target imitation deve usare `pros_ankle_angle` / lato
      protesico/sinistro invece di `ankle_angle_r`, oppure se serve una
      conversione di offset/range tra coordinate.
- [ ] Generare un plot diagnostico dedicato con:
  - `ankle_angle_r`;
  - `pros_ankle_angle`;
  - target phase-based imitation;
  - served reference MLP;
  - actual prosthetic ankle.
- [ ] Valutare una metrica agnostica di fattibilita' globale basata su reserve
      norm, ma solo dopo la verifica del target cinematico.
- [ ] Se si introduce una reserve penalty, iniziare con peso basso e monitorare
      se entra in conflitto con l'imitation target.
- [ ] Nel prossimo confronto training, non valutare solo la posizione served:
      includere sempre anche velocita, accelerazione, comando `u` e `tau_spring`.
- [ ] Prima di introdurre nuove penalty fisiche pesanti, chiarire se la priorita'
      e migliorare la forma della coppia ankle, ridurre le reserve globali, o
      mantenere il tracking imitativo verso il target sound-leg corrente.

### Propagati dal daily 2026-06-19

- [ ] Eseguire confronti piu robusti simmetrico/asimmetrico:
  - training simmetrico da zero 1 -> 100 con stessa config;
  - piu seed;
  - stesso OS e stesso numero di env runner.
- [ ] Monitorare in TensorBoard:
  - served/imitation/tracking losses;
  - `policy_action_clip_loss`;
  - `policy_action_clip_fraction`;
  - reward components;
  - value metrics.
- [ ] Valutare una ablation separata su `vf_clip_param` solo dopo i confronti
      actor-critic e reward.
- [ ] Rivalutare pesi fisici (`sea_tau_spring_*`, reserve, clip penalty) solo
      dopo aver chiarito target cinematico e comportamento della velocity
      imitation.
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
- [ ] Pulire launcher/log temporanei e artefatti di smoke quando non servono piu.
- [ ] Proseguire i TODO SNN/skrl propagati e i TODO storici SEA ancora aperti.
- [ ] Per la linea MuJoCo/MJX: verificare stato VCS, creare matrice
      cross-platform, produrre oracle OpenSim canonici, chiudere gate statici e
      integrare ambiente JAX/MJX batched prima di PPO JAX.

