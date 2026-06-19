# Daily report 2026-06-18

## Sintesi

La giornata e' stata centrata sul training imitativo MLP, sul confronto tra
baseline simmetrica e asymmetric actor-critic a `100` iterazioni, e sulla
correzione del problema emerso nelle curve di coppia SEA: molti indici
cinematici miglioravano, ma la forma della `tau_spring`, soprattutto all'ankle,
diventava meno plausibile rispetto al comportamento CMC-like puro.

La decisione architetturale principale e' stata mantenere la reward modulare e
controller-agnostic. Non sono stati introdotti termini basati su stati interni
del controller protesico, integratori, comandi PI/PID, `outer_i_cmd`,
`cascade_xi` o imitazione diretta della coppia prescribed protesica.

Sono stati invece introdotti e poi attivati termini fisici generali su:

- effort della coppia elastica SEA;
- rate della coppia elastica SEA;
- clipping della raw action della policy.

## Report utente consolidati

Report prodotti o consolidati oggi:

- `reports/user/2026-06-18_reward_modulare_e_coppia_sea_mlp.md`
- `reports/user/2026-06-18_confronto_actor_critic_simmetrico_asimmetrico_100iter.md`
- `reports/user/2026-06-18_reward_sea_controller_agnostic_logging.md`

## Analisi coppia SEA 60 vs 100 iterazioni

Dal confronto tra:

- `plot/06_17_2026_1 - imititation_FullObs_normReward_60iter/01_time_sea_control_reserve.png`
- `plot/06_18_2026_1 - imititation_FullObs_normReward_100iter/01_time_sea_control_reserve.png`

e' emerso che il `100iter` migliora il return e la cinematica servita, ma
modifica la strategia dinamica, in particolare all'ankle.

Metriche principali:

```text
60 iter:
  return = 334.7528621460614
  action_clipped_fraction = 0.0
  ankle served-target RMSE = 0.04486 rad
  ankle tau_spring min/max = -21.78 / +1.76 Nm
  ankle tau_spring area positiva/negativa ~= +1.48 / -18.03 Nms

100 iter:
  return = 369.8268926867439
  action_clipped_fraction = 0.07285
  ankle served-target RMSE = 0.02632 rad
  ankle tau_spring min/max = -19.91 / +43.13 Nm
  ankle tau_spring area positiva/negativa ~= +14.90 / -7.32 Nms
```

Nel punto di picco ankle positivo del `100iter`, attorno a `t ~= 15.974 s`:

```text
SEA_Ankle_tau_spring ~= 43.13 Nm
SEA_Ankle_tau_ref    ~= 42.97 Nm
outer_p_cmd          ~= 0.51 Nm
outer_i_cmd          ~= 42.46 Nm
```

Questo dato e' stato usato per diagnosticare il fenomeno, ma non e' entrato
nella reward: resta un segnale controller-specifico e quindi fuori dal confine
ammesso per l'obiettivo della rete.

Conclusione: una buona traiettoria cinematica non garantisce automaticamente
una buona curva di coppia. La cinematica non identifica in modo univoco la
dinamica prodotta dal plant, dal contatto, dal reference governor e dal
controller intermedio.

## Registry storico training

E' stato aggiornato il registry storico includendo:

```text
Trajectory Generator/runs/training/MLP_imitation_training_06-17-2026_asym_actor_critic_100
```

Risultato registrato:

```text
status = completed
platform = mac
iterations = 1-100
best return = 307.1684731522426 @ iteration 100
asymmetric_actor_critic = true
```

File aggiornati:

- `Trajectory Generator/runs/historical_runs.manual.yaml`
- `Trajectory Generator/runs/historical_runs.md`
- `Trajectory Generator/runs/historical_runs.index.json`

Verifiche:

- esecuzione di `Trajectory Generator/baseline_MLP/update_historical_runs.py`;
- verifica presenza della run in YAML, Markdown e JSON;
- verifica `asymmetric_actor_critic: true`;
- `git diff --check` sui file del registry.

## Confronto simmetrico vs asymmetric actor-critic

Sono stati confrontati i due training MLP imitativi a `100` iterazioni:

- simmetrico:
  `Trajectory Generator/runs/training/MLP_imitation_training_06-17-2026_resume_best_to_100`;
- asimmetrico:
  `Trajectory Generator/runs/training/MLP_imitation_training_06-17-2026_asym_actor_critic_100`.

Il confronto non e' causalmente perfetto: il simmetrico riprende da un best
precedente e continua da `66` a `100`, mentre l'asimmetrico parte da zero e
arriva a `100`. Operativamente, pero', l'asimmetrico e' risultato il miglior
candidato emerso finora.

Metriche training principali:

| Metrica | Simmetrico 100 | Asimmetrico 100 |
|---|---:|---:|
| Best return | `292.508 @97` | `307.168 @100` |
| Return finale | `282.845` | `307.168` |
| Media return 66-100 | `252.061` | `269.862` |
| Deviazione standard return 66-100 | `33.598` | `17.115` |
| Iterazioni full-length 66-100 | `8/35` | `23/35` |
| Final `vf_explained_var` | `0.024` | `0.240` |
| Last10 `vf_explained_var` medio | `0.065` | `0.157` |
| Entropy finale | `1.177` | `1.477` |

Nella finestra `81-100`, l'asimmetrico ha anche ridotto le terminazioni da GRF
penetration rispetto al simmetrico.

## Rollout deterministico asymmetric

E' stato generato e analizzato il rollout deterministico:

```text
Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-17-2026_asym_actor_critic_100
```

Comando usato:

```powershell
C:\Users\tomma\anaconda3\Scripts\conda.exe run --no-capture-output -n envCMC-rllib python "Trajectory Generator\baseline_MLP\rollout_eval.py" --checkpoint "runs\training\MLP_imitation_training_06-17-2026_asym_actor_critic_100\rl_module_best" --output-dir "runs\rollout\MLP_imitation_rollout_06-17-2026_asym_actor_critic_100" --record-outputs
```

Confronto deterministico:

| Metrica | Simmetrico | Asimmetrico |
|---|---:|---:|
| Episode return | `369.827` | `370.374` |
| Reward medio | `0.73818` | `0.73927` |
| Reward minimo | `0.36271` | `0.39843` |
| Pelvis `ty` minimo | `0.95157 m` | `0.95171 m` |
| Action clipped steps | `73` | `47` |
| Action clipped fraction | `7.29%` | `4.69%` |
| Raw action max | `1.425` | `1.219` |

Tracking cinematico:

| Metrica | Simmetrico | Asimmetrico |
|---|---:|---:|
| Knee served-target RMSE | `0.1336 rad` | `0.1010 rad` |
| Ankle served-target RMSE | `0.0263 rad` | `0.0268 rad` |
| Knee actual-served RMSE | `0.00353 rad` | `0.00370 rad` |
| Ankle actual-served RMSE | `0.00788 rad` | `0.00594 rad` |
| Knee actual-target RMSE | `0.1336 rad` | `0.1007 rad` |
| Ankle actual-target RMSE | `0.0263 rad` | `0.0277 rad` |

SEA, potenza e reserve:

| Metrica | Simmetrico | Asimmetrico |
|---|---:|---:|
| Knee SEA torque error RMSE | `2.373 Nm` | `2.357 Nm` |
| Ankle SEA torque error RMSE | `0.385 Nm` | `0.449 Nm` |
| Knee motor power abs mean | `47.10 W` | `42.47 W` |
| Ankle motor power abs mean | `2.16 W` | `1.71 W` |
| SEA input saturation | `0%` | `0%` |
| `tau_reserve_norm_mean` | `331.91` | `336.90` |
| `reserve_control_norm_mean` | `3.319` | `3.369` |

Conclusione operativa: asymmetric actor-critic e' promosso a nuova baseline
operativa per il ramo imitativo MLP, pur richiedendo un A/B piu pulito con
training simmetrico da zero e piu seed.

## Reward SEA controller-agnostic

Sono stati implementati nuovi termini fisici nella reward MLP, inizialmente in
modalita "logging only", poi e' stata scelta e attivata una prima proposta di
pesi nel file `Trajectory Generator/baseline_MLP/training_cfg.yaml`.

Nuove diagnostics segment-level in `simulation_runner.py`:

- `time_s`;
- `tau_spring_nm`;
- `tau_spring_rms_nm`;
- `tau_spring_abs_max_nm`;
- `tau_spring_rate_rms_nm_s`;
- `tau_spring_rate_abs_max_nm_s`.

Il rate usa i tempi reali dei substep:

```text
d(tau_spring)/dt = diff(tau_spring_nm) / diff(time_s)
```

Nuovi loss in `Trajectory Generator/osim_trj_cmc_like.py`:

```text
sea_tau_spring_effort_loss = mean(x^2 / (1 + x^2))
x = tau_spring_rms_nm / abs(F_opt)

sea_tau_spring_rate_loss = mean(x^2 / (1 + x^2))
x = tau_spring_rate_rms_nm_s / (abs(F_opt) / segment_duration)
```

`sea_motor_power_loss` non e' stato modificato: resta basato su
`tau_input * omega_m`, cioe potenza lato motore. Non e' stato aggiunto nessun
termine `tau_spring * qdot_joint`.

In `Trajectory Generator/baseline_MLP/reward_function.py` sono stati aggiunti:

- `sea_tau_spring_effort_weight`;
- `sea_tau_spring_rate_weight`;
- `policy_action_clip_weight`;
- `policy_action_clip_loss`;
- `policy_action_clip_fraction`;
- `policy_action_clip_abs_max`.

Il clipping della policy misura l'escursione tra raw action e action clippata ai
bound dell'action space:

```text
policy_action_clip_loss = mean(x^2 / (1 + x^2))
x = abs(raw_action - clipped_action)
```

TensorBoard e' stato esteso in `Trajectory Generator/baseline_MLP/tb_logging.py`
per includere i nuovi loss e le principali diagnostics per giunto.

## Proposta e attivazione pesi reward

Sono stati stimati i nuovi loss su tre rollout:

| rollout | return | served imitation | action clip frac | effort loss stimato | rate loss stimato | clip loss |
|---|---:|---:|---:|---:|---:|---:|
| 60 iter sym | `334.75` | `0.04899` | `0.0%` | `0.00471` | `0.00369` | `0.00000` |
| 100 iter sym | `369.83` | `0.03209` | `7.29%` | `0.01073` | `0.00472` | `0.00293` |
| 100 iter asym AC | `370.37` | `0.02384` | `4.69%` | `0.00856` | `0.00478` | `0.00032` |

L'asymmetric AC migliora l'imitazione e riduce il clipping; l'ankle torque e'
molto meno degenerato rispetto al `100iter` simmetrico, mentre il knee resta piu
carico rispetto al `60iter`.

Setup attivato in `Trajectory Generator/baseline_MLP/training_cfg.yaml`:

```yaml
sea_tau_spring_effort_weight: 1.0
sea_tau_spring_rate_weight: 0.5
policy_action_clip_weight: 2.0
```

`Trajectory Generator/baseline_MLP/training_cfg.v4_imitation.yaml` e' rimasto a
`0.0` per questi nuovi termini.

Penalty stimata con i pesi attivati:

```text
60 iter sym:       ~3.3 punti episodio
100 iter sym:      ~9.5 punti episodio
100 iter asym AC:  ~5.8 punti episodio
```

Questo dovrebbe rendere visibile il costo fisico al PPO senza schiacciare il
vantaggio imitativo dell'asymmetric actor-critic.

## File modificati oggi

Codice e configurazione:

- `simulation_runner.py`
- `Trajectory Generator/osim_trj_cmc_like.py`
- `Trajectory Generator/baseline_MLP/reward_function.py`
- `Trajectory Generator/baseline_MLP/tb_logging.py`
- `Trajectory Generator/baseline_MLP/training_cfg.yaml`
- `Trajectory Generator/baseline_MLP/training_cfg.v4_imitation.yaml`

Registry storico:

- `Trajectory Generator/runs/historical_runs.manual.yaml`
- `Trajectory Generator/runs/historical_runs.md`
- `Trajectory Generator/runs/historical_runs.index.json`

Output rollout creati:

- `Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-17-2026_asym_actor_critic_100/`

Report creati:

- `reports/user/2026-06-18_reward_modulare_e_coppia_sea_mlp.md`
- `reports/user/2026-06-18_confronto_actor_critic_simmetrico_asimmetrico_100iter.md`
- `reports/user/2026-06-18_reward_sea_controller_agnostic_logging.md`
- `reports/daily/2026-06-18_daily-report.md`

## Verifiche eseguite

Per la reward SEA controller-agnostic:

- `python -m py_compile` sui file:
  - `simulation_runner.py`;
  - `Trajectory Generator/osim_trj_cmc_like.py`;
  - `Trajectory Generator/baseline_MLP/reward_function.py`;
  - `Trajectory Generator/baseline_MLP/tb_logging.py`.
- Compilazione ripetuta nell'ambiente `envCMC-rllib`.
- Test sintetico `compute_reward`:
  - con pesi `0.0`, reward invariata;
  - con pesi positivi, penalty sottratta;
  - componenti nuovi presenti.
- Test sintetico action clipping:
  - `policy_action_clip_loss` positivo quando la raw action supera i bound;
  - `policy_action_clip_fraction` e `policy_action_clip_abs_max` coerenti.
- Test sintetico diagnostics SEA:
  - `tau_spring` costante produce rate zero;
  - rampa lineare produce rate RMS positivo.
- Smoke env `reset + step` con OpenSim/plugin disponibili:
  - nuove chiavi presenti in `info["reward_terms"]`;
  - nuove chiavi presenti in `info["reward_components"]`;
  - diagnostics per giunto presenti.
- `git diff --check` sui file modificati, con soli warning LF/CRLF attesi su
  Windows.

Per training/rollout:

- lettura e confronto di `summary.json`;
- lettura e confronto di `train_iterations.jsonl`;
- calcolo finestre `66-100`, `81-100`, `91-100`;
- analisi di `vf_explained_var`, `vf_loss_unclipped`, entropy, lunghezze
  episodio e terminazioni;
- analisi di `rollout_summary.json`;
- analisi di `rollout_policy_trace.json`;
- analisi di:
  - `rollout_episode_sea_diagnostics.sto`;
  - `rollout_episode_sea_torques.sto`;
  - `rollout_episode_sea_controls.sto`;
  - `rollout_episode_power.sto`;
  - `rollout_episode_recruitment.sto`;
  - `rollout_episode_reserve_torques.sto`;
  - `rollout_episode_gait_events.csv`;
  - `rollout_episode_gait_events_online.csv`.

## TODO chiusi o avanzati il 18/06

- [x] Verificare il completamento del resume baseline simmetrica `_to_100`.
- [x] Eseguire/registrare il training asymmetric actor-critic da zero a `100`
      iterazioni.
- [x] Aggiornare il registry storico con la run asymmetric actor-critic.
- [x] Generare rollout deterministico asymmetric actor-critic.
- [x] Confrontare simmetrico `100iter` e asymmetric `100iter` con metriche di
      training e rollout.
- [x] Diagnosticare perche' il `100iter` migliora la cinematica ma degrada la
      curva di coppia ankle.
- [x] Definire il vincolo architetturale: niente termini controller-specifici
      nella reward.
- [x] Implementare logging e reward support per `tau_spring_rms`,
      `tau_spring_rate_rms` e clipping action.
- [x] Mantenere `sea_motor_power_loss` basata su `tau_input * omega_m`.
- [x] Estendere TensorBoard con i nuovi loss e diagnostics SEA.
- [x] Proporre pesi iniziali per i nuovi termini fisici.
- [x] Attivare in `training_cfg.yaml`:
      `1.0 / 0.5 / 2.0` per effort/rate/clipping.

## TODO aperti e propagati

### Training imitation e confronto

- [ ] Eseguire un training simmetrico da zero `1 -> 100` con la stessa config
      del run asimmetrico, per isolare meglio l'effetto architetturale.
- [ ] Ripetere simmetrico e asimmetrico con piu seed e stesso OS / stesso numero
      di env runner.
- [ ] Lanciare un nuovo training con i pesi fisici attivati in
      `training_cfg.yaml` e confrontarlo con asymmetric AC `100iter`.
- [ ] Monitorare in TensorBoard i nuovi termini:
      `sea_tau_spring_effort_loss`, `sea_tau_spring_rate_loss`,
      `policy_action_clip_loss`, `policy_action_clip_fraction`,
      diagnostics per giunto su `tau_spring`.
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

- [ ] Valutare se `sea_tau_spring_effort_weight: 1.0`,
      `sea_tau_spring_rate_weight: 0.5`, `policy_action_clip_weight: 2.0`
      sono abbastanza forti da spostare il comportamento senza danneggiare
      imitation e stabilita.
- [ ] Se necessario, provare una variante prudente `0.5 / 0.25 / 1.0` o una
      variante piu aggressiva `2.0 / 0.75 / 3.0`.
- [ ] Non introdurre nella reward coppie prescribed protesiche, `outer_i_cmd`,
      `cascade_xi`, stati integrali o dettagli PI/PID.

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
