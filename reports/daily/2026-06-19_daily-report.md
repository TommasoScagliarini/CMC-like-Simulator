# Daily report 2026-06-19

## Sintesi

La giornata e' stata dedicata alla diagnosi della forma non plausibile della
`tau_spring` ankle nei rollout MLP imitativi e alla preparazione del prossimo
training asymmetric actor-critic da `100` iterazioni.

Sono emersi due punti principali:

- la valutazione rollout precedente era parzialmente cieca al clipping della
  raw action, perche' il rollout passava all'ambiente l'azione gia' clippata;
- la differenza tra coppia ankle "buona" e "brutta" non si spiega solo con la
  posizione served, ma soprattutto con velocita, accelerazione, fase e memoria
  integrale del controller cascade.

Operativamente e' stato corretto `rollout_eval.py`, rigenerato il rollout
`asym200`, analizzato il training simmetrico `60` iterazioni, confrontato
`sym60` con `asym100`, e aggiornata `training_cfg.yaml` per un nuovo run
asymmetric actor-critic da `100` iterazioni.

## Report utente consolidati

Report prodotti oggi:

- `reports/user/2026-06-19_diagnosi_tau_spring_target_imitativo_reserve.md`
- `reports/user/2026-06-19_confronto_sym60_asym100_tau_spring_ankle.md`

## Fix rollout action clipping

E' stato corretto il rollout evaluator:

- prima il rollout passava a `env.step()` l'azione gia' clippata;
- ora passa `raw_action`, lasciando al wrapper interno `FlattenClipAction` la
  protezione del simulatore;
- il `RewardShapingWrapper` vede quindi la differenza raw-vs-applied e puo'
  calcolare correttamente `policy_action_clip_loss`.

File modificato:

- `Trajectory Generator/baseline_MLP/rollout_eval.py`

Verifiche eseguite:

- `py_compile` su:
  - `Trajectory Generator/baseline_MLP/rollout_eval.py`;
  - `Trajectory Generator/baseline_MLP/reward_function.py`;
  - `Trajectory Generator/baseline_MLP/env_factory.py`.
- test sintetico con dummy env:
  - base env riceve azione clippata;
  - reward wrapper vede azione raw;
  - `policy_action_clip_loss` diventa non nullo quando la raw action supera i
    bound.
- `git diff --check` sul fix.

## Rollout asym200 rigenerato

Dopo il fix e' stato rigenerato il rollout `asym200`:

```text
Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-18-2026_asym_actor_critic_resume_200/
```

Metriche principali:

```text
episode_return              361.0964554402106
steps                       501
action_abs_max              1.30666
applied_action_abs_max      1.0
action_clipped_steps        75
action_clipped_fraction     0.07485
policy_action_clip_loss mean 0.00231764
policy_action_clip_loss max  0.0429788
```

Conclusione: la valutazione precedente era leggermente ottimistica, ma il
giudizio operativo non cambia. `asym200` non sostituisce `asym100` come miglior
baseline pratica.

Plot generato:

- `plot/06_19_2026_1_imitTraining_asymActCrit_200/`

## Diagnosi tau_spring, target imitativo e reserve

Il confronto tra `asym100/asym200` e il run `05_23` ha mostrato che:

- nei rollout MLP l'ankle `tau_spring` e molto piu piccola e meno
  biomeccanicamente plausibile;
- il SEA segue bene il comando/diagnostica di coppia richiesta;
- il problema non sembra essere un tracking interno SEA difettoso.

Confronto indicativo:

```text
asym100:
  SEA_Ankle_tau_ff_cmd RMS ~= 5.72 Nm
  SEA_Ankle_tau_spring RMS ~= 5.74 Nm

05_23:
  SEA_Ankle_tau_ff_cmd RMS ~= 48.06 Nm
  SEA_Ankle_tau_spring RMS ~= 48.07 Nm
```

La differenza principale sta nel target cinematico: il target imitation corrente
mappa la protesi ankle su `ankle_angle_r`, mentre il run `05_23` usa la
coordinata protesica/sinistra `pros_ankle_angle`.

Range nel riferimento IK `12.99-17.99 s`:

```text
ankle_angle_r:
  [4.005, 27.549] deg  ~= [0.070, 0.481] rad

pros_ankle_angle:
  [-7.949, 23.037] deg ~= [-0.139, 0.402] rad
```

Le reserve locali prostetiche sono zero:

```text
pros_knee_angle_reserve_torque  = 0
pros_ankle_angle_reserve_torque = 0
```

Quindi non falsano direttamente la coppia SEA. Le reserve globali, pero',
restano alte:

```text
asym100 tau_reserve_norm:
  mean 333.2
  RMS  402.2
  max  843.2

05_23/results tau_reserve_norm:
  mean 104.1
  RMS  114.4
  max  530.7
```

Decisione: non introdurre un profilo di coppia target. Prima va chiarito il
target cinematico e va valutata la fattibilita' globale tramite reserve in modo
agnostico.

## Training simmetrico 60 iterazioni

E' stato analizzato il training:

```text
Trajectory Generator/runs/training/baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm
```

Risultato training:

```text
completed
iterations_completed      60
best logical iteration    59
best train return mean    277.75698812995
```

Rollout generato:

```text
Trajectory Generator/runs/rollout/baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm/
```

Metriche rollout:

```text
episode_return             334.7505830458208
steps                      501
terminated                 false
truncated                  true
action_clipped_steps       0
action_clipped_fraction    0.0
```

Plot generato e rinominato:

```text
plot/06_19_2026_2 - imitTraining_sym_60/
```

Canali plot:

```text
No missing channels.
```

Valutazione: `sym60` non migliora globalmente `asym100` come reward, tracking o
reserve, ma l'ankle `tau_spring` e qualitativamente piu simile al run `05_23`
per forma/timing.

Confronto indicativo:

```text
sym60 ankle tau_spring:
  RMS circa 6.16 Nm
  min circa -21.8 Nm

05_23 ankle tau_spring:
  RMS circa 48.1 Nm
  min circa -120.5 Nm
```

La somiglianza e quindi di forma, non di scala.

## Confronto sym60 vs asym100

La domanda centrale e' stata: perche `sym60` e `asym100` possono avere una
posizione served ankle simile, ma una `tau_spring` molto diversa?

La risposta e' nella catena del controller cascade:

```text
qdot_cas = qdot_ref + Kp_outer * (q_ref - q)
tau_cmd  = Kp_inner * (qdot_cas - qdot)
         + Ki_inner * integral(qdot_cas - qdot)
u        = tau_cmd / F_opt
```

Per ankle:

```text
Kp_outer = 47.125
Kp_inner = 2.8275
Ki_inner = 213.0
F_opt    = 250 Nm
```

Confronto numerico `sym60` vs `asym100` su finestra `13-18 s`:

```text
ankle position corr      ~0.954
ankle velocity corr      ~0.795
ankle acceleration corr  ~0.355
ankle control u corr     ~0.118
ankle tau_spring corr    ~0.136
```

Quindi la posizione e simile, ma velocita, accelerazione, comando `u` e coppia
divergono molto.

Esempio a circa `t = 15.98 s`:

```text
sym60:
  q      = +0.260 rad
  qdot   = +0.506 rad/s
  u      = -0.033
  tau    = -8.3 Nm

asym100:
  q      = +0.211 rad
  qdot   = +0.735 rad/s
  u      = +0.086
  tau    = +21.5 Nm
```

Conclusione: l'ankle e fragile perche lavora vicino a cambi di segno della
coppia; piccole differenze di fase/velocita e la memoria integrale del PI
possono cambiare qualitativamente la `tau_spring`.

## Config per prossimo training asymmetric 100

E' stata aggiornata la configurazione operativa:

`Trajectory Generator/baseline_MLP/training_cfg.yaml`

Modifiche applicate:

```yaml
simulation:
  iterations: 100

reward:
  imitation_ankle_velocity_weight: 0.08
  smoothness_weight: 0.03
  command_rate_weight: 0.08
  sea_tau_spring_effort_weight: 0.0
  sea_tau_spring_rate_weight: 0.05
```

Rimasto attivo:

```yaml
model:
  asymmetric_actor_critic: true
```

Razionale:

- aumentare il peso sulla velocity imitation ankle per ridurre ambiguita' di
  fase/velocita;
- rafforzare leggermente smoothness e command-rate;
- evitare che `sea_tau_spring_effort_weight: 1.0` schiacci troppo l'ampiezza
  della coppia;
- mantenere un rate penalty basso e agnostico per scoraggiare forme troppo
  impulsive.

L'utente ha poi lanciato un training con la configurazione attuale dello YAML.

## File modificati oggi

Codice/configurazione:

- `Trajectory Generator/baseline_MLP/rollout_eval.py`
- `Trajectory Generator/baseline_MLP/training_cfg.yaml`

Report creati:

- `reports/user/2026-06-19_diagnosi_tau_spring_target_imitativo_reserve.md`
- `reports/user/2026-06-19_confronto_sym60_asym100_tau_spring_ankle.md`
- `reports/daily/2026-06-19_daily-report.md`

Output consultati/generati:

- `Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-18-2026_asym_actor_critic_resume_200/`
- `Trajectory Generator/runs/rollout/baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm/`
- `plot/06_19_2026_1_imitTraining_asymActCrit_200/`
- `plot/06_19_2026_2 - imitTraining_sym_60/`
- `plot/06_18_2026_2_imitTraining_asymActCrit_100/`
- `plot/05_23_2026_2/`
- `results/sim_output_kinematics.sto`
- `results/sim_output_sea_diagnostics.sto`
- `results/sim_output_recruitment.sto`

Nota worktree: risultano anche modifiche/cancellazioni gia presenti e non
revertite, tra cui `Trajectory Generator/baseline_MLP/training_cfg.v4_imitation.yaml`
e file registry storici.

## Verifiche eseguite

- `py_compile` su rollout/reward/env wrapper.
- Test sintetico sul clipping raw-vs-applied.
- `git diff --check` sul fix rollout.
- Rollout `asym200` rigenerato con `--record-outputs`.
- Rollout `sym60` generato dal checkpoint `rl_module_best`.
- Plot MLP generati per `sym60`; nessun missing channel.
- Confronto visivo dei plot `sym60`, `asym100`, `asym200`, `05_23`.
- Confronti numerici su:
  - `tau_spring` vs `tau_ff_cmd`;
  - range `ankle_angle_r` vs `pros_ankle_angle`;
  - reserve norm;
  - posizione, velocita, accelerazione, comando `u`, `tau_spring`;
  - errore cascade ricostruito dal trace.

## TODO chiusi o avanzati il 19/06

- [x] Verificare e correggere il fatto che `policy_action_clip_loss` nel rollout
      fosse cieco perche l'azione veniva clippata prima della reward.
- [x] Rigenerare rollout e plot per `asym200` dopo il fix.
- [x] Analizzare se la valutazione di `asym200` cambia dopo il fix.
- [x] Chiarire che `asym100` resta il plateau/baseline pratica migliore della
      configurazione precedente.
- [x] Analizzare i pesi fisici aggiunti alla reward in `training_cfg.yaml`.
- [x] Diagnosticare la discrepanza tra cinematica apparentemente buona e
      `tau_spring` ankle poco plausibile.
- [x] Verificare che le reserve locali prostetiche non falsino direttamente la
      coppia SEA.
- [x] Analizzare il training simmetrico `60` iterazioni, generare rollout e plot.
- [x] Confrontare `sym60` e `asym100` per capire perche la coppia ankle cambia
      molto nonostante una posizione served simile.
- [x] Creare i due user report del 19/06.
- [x] Aggiornare `training_cfg.yaml` per il prossimo asymmetric actor-critic
      `100` iterazioni.

## TODO aperti e propagati

### Nuovi TODO dal 19/06

- [ ] Verificare se il target imitation deve usare `pros_ankle_angle` / lato
      protesico-sinistro invece di `ankle_angle_r`, oppure se serve una
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
- [ ] Aggiungere ai log o ai plot diagnostici i termini interni del controller
      cascade: `e_q`, `e_qdot`, `qdot_cas`, `cascade_velocity_error`,
      contributo P e contributo I.
- [ ] Nei prossimi confronti training, non valutare solo la posizione served:
      includere sempre anche velocita, accelerazione, comando `u` e
      `tau_spring`.
- [ ] Prima di introdurre nuove penalty fisiche pesanti, chiarire se la
      priorita e migliorare la forma della coppia ankle, ridurre le reserve
      globali, o mantenere il tracking imitativo verso il target sound-leg
      corrente.
- [ ] Quando finisce il training lanciato con la nuova config, eseguire rollout
      del `rl_module_best`, generare plot e confrontare con `asym100`,
      `sym60` e `05_23`.

### Training imitation e confronto

- [ ] Eseguire un training simmetrico da zero `1 -> 100` con la stessa config
      del run asimmetrico, per isolare meglio l'effetto architetturale.
- [ ] Ripetere simmetrico e asimmetrico con piu seed e stesso OS / stesso numero
      di env runner.
- [ ] Monitorare in TensorBoard i termini:
      `sea_tau_spring_effort_loss`, `sea_tau_spring_rate_loss`,
      `policy_action_clip_loss`, `policy_action_clip_fraction`, diagnostics per
      giunto su `tau_spring`.
- [ ] Valutare una ablation separata su `vf_clip_param` solo dopo il confronto
      asymmetric vs baseline simmetrica da zero.
- [ ] Valutare un training da zero con reward normalizzata e pesi fisici, se
      serve distinguere bias del resume e local optimum ereditati.

### Plot e metriche fisiche

- [ ] Generare un confronto plot ufficiale sim100 vs asym100 con figure robuste,
      specialmente torque-angle senza `fill_between` su ascissa non monotona.
- [ ] Creare una diagnostica comparativa `60iter` vs `100iter` vs asymmetric
      `100iter` con metriche fisiche controller-agnostic.
- [ ] Aggiungere o esportare metriche aggregate su impulso positivo/negativo
      ankle, lavoro SEA e potenza SEA in modo piu diretto nei log.
- [ ] Continuare a monitorare ankle torque, knee torque, command-rate, action
      clipping, reserve/root load e GRF penetration.

### Reward e modularita

- [ ] Rivalutare i pesi fisici dopo il nuovo run: il 19/06 la config e stata
      portata a `sea_tau_spring_effort_weight: 0.0`,
      `sea_tau_spring_rate_weight: 0.05`, `policy_action_clip_weight: 2.0`.
- [ ] Non introdurre nella reward coppie prescribed protesiche, `outer_i_cmd`,
      `cascade_xi`, stati integrali o dettagli PI/PID.

### Forma della served reference

- [ ] Verificare se il maggiore peso ankle velocity `0.08` migliora fase e
      coppia senza creare overshoot o peggiorare knee.
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
