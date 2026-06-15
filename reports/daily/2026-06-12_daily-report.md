# Daily report - 2026-06-12

## Sintesi

La giornata del 12 giugno 2026 ha chiuso il ciclo di diagnosi e validazione del
training imitativo V3 con `SEA_Knee Kp=18`, mantenendo invariata la semantica
del plugin C++ SEA.

Le attivita principali sono state:

1. diagnosticato il primo training imitativo e separato l'errore della policy
   dai limiti del plant SEA;
2. corretto il target imitativo con una rappresentazione phase-based locale e
   specifica per giunto;
3. introdotto un reference governor fisicamente compatibile mantenendo
   `Kp=18`;
4. esposto alla reward e al logging lo stress SEA reale misurato sui substep;
5. completato il nuovo training imitativo V3 da 40 iterazioni;
6. valutato il rollout deterministico contro policy precedente e oracle;
7. corretto un errore diagnostico nel plot 06, che mostrava il prescribed al
   posto del riferimento realmente servito dalla policy;
8. identificato l'origine dello swing iniziale e del chattering attorno a
   `100 Hz`;
9. verificato il comportamento con orizzonte aumentato a `5 s`;
10. corretto il refresh duplicato della progress bar del training;
11. definito il piano definitivo, strict e gate-driven per il porting della
    Trajectory Generator `baseline_MLP` su MuJoCo/MJX GPU.

Il risultato complessivo e un miglioramento sostanziale rispetto al primo seed:
il clamp interno del SEA knee viene eliminato e il tracking del riferimento
realmente servito e buono. La policy non raggiunge ancora una corretta
imitazione del ginocchio e continua a eccitare la dinamica SEA attorno a
`100 Hz`.

## Report e piani prodotti

Report utente:

```text
reports/user/2026-06-12_diagnosi_training_imitativo_target_reward_saturazione_sea.md
reports/user/2026-06-12_training_imitativo_kp18_reference_governor.md
reports/user/2026-06-12_fix_righe_duplicate_progress_training.md
reports/user/2026-06-12_valutazione_rollout_training_imitativo_v3.md
reports/user/2026-06-12_diagnosi_served_reference_swing_chattering_orizzonte.md
```

Piani MuJoCo/MJX:

```text
reports/plans/2026-06-12_porting_trajectory_generator_mujoco_mjx_gpu.md
reports/plans/2026-06-12_porting_trajectory_generator_mujoco_mjx_gpu_v2_approvato.md
reports/plans/2026-06-12_porting_trajectory_generator_mujoco_mjx_gpu_v3.md
reports/plans/2026-06-12_porting_trajectory_generator_mujoco_mjx_gpu_definitivo.md
```

Il riferimento operativo finale per il porting e:

`reports/plans/2026-06-12_porting_trajectory_generator_mujoco_mjx_gpu_definitivo.md`

## 1. Diagnosi del primo training imitativo

Il primo run:

`Trajectory Generator/runs/baseline_mlp_imit_win`

aveva completato 40 iterazioni, ma il rollout deterministico mostrava:

- scarsa imitazione del ginocchio;
- clamp interno del motore SEA knee nel `33.93%` dei campioni;
- comando knee dominato da contenuto attorno a `100 Hz`;
- contatto protesico sinistro circa `6.9%`;
- reserve elevate e quota muscolare ridotta.

La diagnosi ha dimostrato che il SEA seguiva bene il riferimento prodotto dalla
policy, mentre la policy produceva un riferimento lontano dal target imitativo.

Metriche iniziali principali:

| Metrica | Knee | Ankle |
|---|---:|---:|
| RMSE totale vs target | `24.55 deg` | `4.16 deg` |
| Errore policy vs target | `24.44 deg` | `4.15 deg` |
| Errore SEA vs comando | `0.71 deg` | `0.37 deg` |

Le cause principali erano:

- target sano anti-fase basato su uno shift temporale globale errato;
- prefisso iniziale clampato e internamente incoerente;
- reward cieca al clamp interno `tau_input`;
- smoothness applicata ai knot interni ma non al command-rate reale;
- `Kp=18` come amplificatore dei transienti, non causa primaria.

## 2. Target phase-based e reference governor con Kp=18

Il target imitativo e stato sostituito con un target periodico locale indicizzato
sulla gait phase della gamba sana.

Shift calibrati:

| Giunto | Phase shift |
|---|---:|
| Knee | `0.465` |
| Ankle | `0.452` |

Posizione e velocita target derivano dalla stessa spline periodica. Gli episodi
imitativi vengono inizializzati sul target, eliminando il precedente prefisso
clampato.

Il modello nominale e stato mantenuto con:

```text
SEA_Knee Kp = 18
```

Il riferimento policy passa ora attraverso:

- modello di riferimento low-pass del secondo ordine a `6 Hz`;
- stato persistente di posizione e velocita;
- limiti di velocita;
- limiti di accelerazione;
- integrazione trapezoidale coerente.

Limiti introdotti:

| Giunto | Velocita | Accelerazione |
|---|---:|---:|
| Knee | `6.0 rad/s` | `60 rad/s^2` |
| Ankle | `3.5 rad/s` | `55 rad/s^2` |

Il governor modifica solamente il riferimento cinematico dentro
`Trajectory Generator/`; non modifica il plugin C++ o la semantica del comando
normalizzato SEA.

## 3. Reward e diagnostica SEA reale

La reward imitativa e il logging sono stati estesi con:

- command-rate tra knot e step;
- `qdot_ref` e `qddot_ref`;
- variazione del comando SEA `u`;
- intervento dei limiti del reference governor;
- clamp interno `tau_input`;
- errore di coppia `tau_ref - tau_spring`;
- velocita, accelerazione e potenza motore.

La diagnostica SEA viene raccolta su ogni substep di integrazione e resa
disponibile alla reward e a TensorBoard separatamente per knee e ankle.

L'oracle finale:

`Trajectory Generator/runs/imitation_oracle_kp18_governed_final`

ha confermato che `Kp=18` e compatibile con un riferimento valido:

| Metrica knee | Kp=18 non governato | Kp=10 | Kp=18 governato |
|---|---:|---:|---:|
| Errore coppia RMS | `12.95 Nm` | `11.59 Nm` | **`4.20 Nm`** |
| Clamp interno | `4.70%` | `0.65%` | **`0%`** |
| `tau_input_raw` RMS | `202.93 Nm` | `108.19 Nm` | **`59.14 Nm`** |
| Velocita motore RMS | `14.96 rad/s` | `8.59 rad/s` | **`5.21 rad/s`** |

L'oracle completa regolarmente `201` step con reward media `0.8432`, tracking
score `0.9900` e nessun clamp interno.

## 4. Training imitativo V3 completato

Il nuovo training da zero:

`Trajectory Generator/runs/baseline_mlp_imit_v3_governed_win`

e stato completato regolarmente:

```text
iterations completed       = 40/40
best episode return mean   = 103.0862
best logical iteration     = 39
elapsed wall time          = circa 8.04 h
stop reason                = completed
```

Il return medio e cresciuto lungo tutto il training:

| Iterazioni | Return medio del blocco |
|---|---:|
| `1-10` | `42.96` |
| `11-20` | `73.76` |
| `21-30` | `86.17` |
| `31-40` | `97.01` |
| Ultime 5 | `100.77` |

La progress bar e stata corretta eliminando la doppia emissione della riga live
nel percorso TTY. La modifica e esclusivamente cosmetica e non altera training
o checkpoint.

## 5. Valutazione del rollout V3

Il rollout deterministico:

`Trajectory Generator/runs/baseline_mlp_imit_v3_governed_win_rollout`

ha completato `201` step con normale time limit:

```text
episode return   = 124.9171
reward media     = 0.6215
pelvis_ty minimo = 0.9449 m
terminated       = false
truncated        = true
```

Confronto con policy precedente e oracle:

| Metrica | Policy precedente | V3 | Oracle |
|---|---:|---:|---:|
| Knee imitation RMSE | `24.07 deg` | **`15.55 deg`** | `5.90 deg` |
| Knee correlazione | `-0.061` | **`0.608`** | `0.938` |
| Ankle imitation RMSE | `6.62 deg` | **`4.52 deg`** | `2.04 deg` |
| Knee torque error RMS | `41.89 Nm` | **`11.83 Nm`** | `4.20 Nm` |
| Knee clamp interno | `33.93%` | **`0%`** | `0%` |
| Reserve norm media | `798.62` | **`346.14`** | `244.17` |

Il V3:

- elimina il clamp interno del ginocchio;
- riduce drasticamente il bang-bang high-level;
- ripristina il contatto protesico;
- riduce reserve e potenza motore;
- non raggiunge ancora l'ampiezza e la fase corrette del ginocchio;
- carica il piede protesico piu dell'oracle;
- raggiunge `u=+1` sul knee per circa l'`11%` del rollout.

Non e consigliato ridurre nuovamente `Kp`: il collo di bottiglia e la traiettoria
appresa e la sua rappresentazione temporale.

## 6. Correzione del plot 06 e tracking reale

E stato individuato un errore diagnostico nel precedente:

`plot/06_12_2026_1/06_time_joint_ref_sea_error.png`

La curva indicata come `kinematic ref` proveniva dall'IK prescribed, non dal
riferimento realmente generato e servito dalla policy.

Sono stati aggiunti:

- `*_kinematics_reference.sto`, con `q_ref` e `qdot_ref` realmente serviti;
- `rollout_policy_trace.json`, con azione grezza, knot convertiti, segmenti e
  target imitativo;
- fallback prescribed nel plotter solamente per output storici.

Il rollout diagnostico:

`Trajectory Generator/runs/baseline_mlp_imit_v3_governed_win_rollout_refdiag`

riproduce esattamente il return `124.9171`, quindi la registrazione non altera
la simulazione.

Il plot corretto e:

`plot/06_12_2026_2/06_time_joint_ref_sea_error.png`

Tracking rispetto al served reference:

| Errore `simulated - served reference` | Knee | Ankle |
|---|---:|---:|
| RMSE globale | **`2.50 deg`** | **`0.55 deg`** |
| Errore massimo | `5.33 deg` | `1.97 deg` |
| RMSE primi `250 ms` | `0.36 deg` | `0.91 deg` |

Il controllore high-level segue quindi bene il riferimento realmente servito.
Il riferimento scelto dalla policy resta distante dal prescribed e dal target
imitativo desiderato.

## 7. Swing iniziale, caviglia e chattering

### Swing iniziale

Lo swing knee iniziale e generato dalla policy:

| Istante | Knee simulated | Served reference | Prescribed |
|---|---:|---:|---:|
| Inizio | `-53.48 deg` | `-53.30 deg` | `-7.95 deg` |
| `+100 ms` | `-29.54 deg` | `-29.71 deg` | `-7.83 deg` |
| `+250 ms` | `-6.10 deg` | `-6.72 deg` | `-7.42 deg` |

L'episodio parte correttamente dal target imitativo a circa `-53 deg`, ma il
primo segmento policy contiene un knot knee clippato vicino a `0 deg`. Il
governor limita il transitorio, ma non elimina la grande estensione richiesta.

### Caviglia

La caviglia non e immobile:

- served reference: `3.04-24.23 deg`;
- simulated: `2.93-24.64 deg`;
- tracking RMSE: `0.55 deg`.

La sua escursione e meno evidente visivamente ed e concentrata durante il
contatto.

### Chattering

Il chattering knee deriva dall'interazione tra:

- policy-step ogni `10 ms`, equivalente a `100 Hz`;
- accelerazione discontinua ai confini dei segmenti;
- dinamica naturale interna SEA knee vicina a `124 Hz`;
- elevata richiesta sostenuta della traiettoria policy;
- feedback cascade di velocita.

Confronto corretto con prescribed in modalita `sea_forward_mode=plugin`:

| Metrica knee | Prescribed plugin | Policy V3 |
|---|---:|---:|
| `tau_ref` RMS | `11.68 Nm` | `55.62 Nm` |
| Torque error RMS | `1.30 Nm` | `11.83 Nm` |
| `tau_input_raw` RMS | `14.32 Nm` | `158.66 Nm` |
| Componente `tau_ref` a circa `100 Hz` | `0.03 Nm` | `11.45 Nm` |

Questo non contraddice la validazione col prescribed: il riferimento prescribed
non eccita la stessa banda e richiede coppie knee molto inferiori.

## 8. Verifica dell'orizzonte a 5 secondi

E stato eseguito:

`Trajectory Generator/runs/baseline_mlp_imit_v3_governed_win_rollout_5s_refdiag`

Il rollout termina dopo `0.68 s` per `grf_penetration`, con penetrazione
sinistra massima `28.1 mm`.

Il semplice aumento di `episode_duration` cambia la feature actor:

```text
phase = elapsed_time / episode_duration
```

Il checkpoint addestrato a `2 s` entra quindi fuori distribuzione quando viene
valutato a `5 s`.

Per aumentare correttamente l'orizzonte occorre rendere il clock indipendente
dalla durata totale dell'episodio e addestrare nuovamente con episodi piu
lunghi, usando principalmente il gait-phase clock ciclico.

## 9. Piano definitivo MuJoCo/MJX GPU

E stato definito il piano definitivo per portare la Trajectory Generator
`baseline_MLP` nella repository:

`C:\Users\tomma\Desktop\MuJoCo_env`

Decisioni principali:

- OpenSim resta l'oracolo scientifico;
- MuJoCo CPU e il backend di validazione/debug;
- MJX/JAX su RTX 4070 via WSL2/Linux e il backend batched di produzione;
- il runtime target non deve dipendere da import o file non versionati della
  repository OpenSim;
- tutto il nuovo codice di produzione vive in `MuJoCo_env`;
- il plugin C++ e la semantica OpenSim non vengono modificati per facilitare il
  porting;
- un risultato non parity puo essere dichiarato solamente
  `research_surrogate_only`, non porting 1:1.

Il piano introduce una sequenza gate-driven:

1. preflight, inventario e fattibilita GPU/hybrid;
2. oracle canonico e sincronizzazione bundle `321/500_pi`;
3. replay e parity scientifica strict;
4. calibrazione contatto hybrid;
5. ambiente JAX/MJX batched;
6. benchmark e milestone v1-engine;
7. PPO JAX, rollout, supervisione e UX;
8. gate finali scientifici, prestazionali e cross-platform.

Gate prestazionali principali:

- batch stabile almeno `32`;
- speedup engine MJX almeno `5x`;
- rollout batch con policy almeno `5x` rispetto a OpenSim 12-worker;
- training PPO end-to-end almeno `3x`;
- rollout deterministico MJX batch 1 non oltre `0.5x` del wall-clock OpenSim.

Il porting non e stato implementato durante la giornata: e stato prodotto e
approvato il piano operativo definitivo.

## File modificati o aggiunti

### Trajectory Generator e training

```text
Trajectory Generator/osim_trj_cmc_like.py
Trajectory Generator/baseline_MLP/env_factory.py
Trajectory Generator/baseline_MLP/reward_function.py
Trajectory Generator/baseline_MLP/reward_imitation.json
Trajectory Generator/baseline_MLP/tb_logging.py
Trajectory Generator/baseline_MLP/imitation_oracle_rollout.py
Trajectory Generator/baseline_MLP/training_config.py
Trajectory Generator/baseline_MLP/training_cfg.yaml
Trajectory Generator/baseline_MLP/train_ppo_mlp.py
Trajectory Generator/baseline_MLP/rollout_eval.py
Trajectory Generator/baseline_MLP/commands.txt
Trajectory Generator/baseline_MLP/progress_display.py
```

### Simulatore, diagnostica e plot

```text
simulation_runner.py
output.py
plot/plotter.py
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim
validation/test_progress_display.py
```

### Documentazione

```text
reports/user/2026-06-12_*.md
reports/plans/2026-06-12_*.md
reports/daily/2026-06-12_daily-report.md
```

Nessuna modifica e stata applicata al plugin C++ SEA o alla semantica del
comando SEA.

## Output principali generati

```text
Trajectory Generator/runs/imitation_oracle_kp18_governed_final
Trajectory Generator/runs/baseline_mlp_imit_v3_governed_win
Trajectory Generator/runs/baseline_mlp_imit_v3_governed_win_rollout
Trajectory Generator/runs/baseline_mlp_imit_v3_governed_win_rollout_refdiag
Trajectory Generator/runs/baseline_mlp_imit_v3_governed_win_rollout_5s_refdiag
plot/06_12_2026_1
plot/06_12_2026_2
```

## Verifiche eseguite

- analisi completa degli STO del primo rollout imitativo;
- confronto target phase-based, policy, served reference e prescribed;
- analisi FFT di riferimenti, controlli e dinamica SEA;
- misura di clamp interno, torque error, potenza e velocita motore;
- analisi di contatto, penetrazione, reserve e quota muscolare;
- test isolato del reference governor con gradino irrealistico;
- oracle completo con `Kp=18` e dinamica ibrida;
- confronto oracle `Kp=10`, `Kp=18` non governato e `Kp=18` governato;
- parsing XML modello e JSON reward;
- roundtrip configurazione training/rollout;
- `py_compile` dei moduli Python coinvolti;
- `git diff --check`;
- training V3 reale completato a 40/40;
- rollout deterministico V3 completo;
- rollout diagnostico equivalente con served reference;
- generazione e ispezione del plot 06 corretto;
- rollout diagnostico a `5 s`;
- test regressione progress display: `2/2` PASS.

## TODO chiusi il 12/6

- [x] Implementare target imitativo phase-based locale/ciclico e coerente.
- [x] Eliminare il prefisso iniziale clampato tramite inizializzazione coerente.
- [x] Esporre e penalizzare clamp interno, torque error e stress motore.
- [x] Aggiungere loss command-rate, `qdot_ref`, `qddot_ref` e variazione di `u`.
- [x] Eseguire rollout oracle con target corretto.
- [x] Valutare conservativamente `Kp=10` e mantenere il nominale `Kp=18`.
- [x] Eseguire da zero il training imitativo `TRAIN-IMIT-V3-GOV-WIN`.
- [x] Validare il rollout appreso contro oracle, clamp, contatto e reserve.
- [x] Eseguire un rollout piu lungo per testare la generalizzazione.
- [x] Correggere le righe duplicate della progress bar TTY.
- [x] Definire il piano definitivo del porting MuJoCo/MJX GPU.

## TODO aperti e propagati

### Training imitativo e traiettoria policy

- [ ] Rendere il clock actor indipendente dalla durata totale dell'episodio,
      privilegiando il gait-phase clock ciclico.
- [ ] Addestrare nuovamente con episodi da almeno `3.5-5 s`.
- [ ] Penalizzare la distanza tra primo knot policy e riferimento iniziale per
      eliminare lo swing iniziale.
- [ ] Rendere il reference governor continuo anche in accelerazione,
      preferibilmente con un profilo jerk-limited.
- [ ] Ridurre l'eccitazione a `100 Hz` introdotta dai segmenti policy senza
      modificare il plugin o nascondere il problema abbassando `Kp`.
- [ ] Ridurre o eliminare il clipping dell'output grezzo della policy,
      valutando una distribuzione d'azione bounded/squashed.
- [ ] Migliorare ampiezza e fase della flessione del ginocchio rispetto al
      target imitativo.
- [ ] Aggiungere una metrica/reward minima di supporto e carico protesico.
- [ ] Ridurre carico e penetrazione del piede protesico senza perdere il
      contatto ottenuto.
- [ ] Ripetere il rollout lungo dopo il nuovo training e produrre plot
      gait-cycle completi.
- [ ] Correggere l'asserzione obsoleta actor-only/full-observation nel test
      storico.
- [ ] Risolvere il problema locale Torch/OpenMP e completare uno smoke test
      RLlib.

### Recovery, runtime e cross-platform

- [ ] Validare l'auto-recovery interno durante un crash nativo Ray reale, con
      `--checkpoint-every 1` e `--max-consecutive-crash-restarts 5`.
- [ ] Rendere esplicito nei log quale componente ha eseguito ogni restart.
- [ ] Stabilizzare e validare Ray su Windows tramite una matrice controllata
      Python/Ray/grpcio/protobuf.
- [ ] Raccogliere un dump nativo piu informativo del prossimo access violation.
- [ ] Verificare su macOS arm64 cleanup process group, resume RLlib, round-trip
      RLModule, reward mode e warm-start; ricompilare onlineGRF `.dylib`.
- [ ] Valutare `training_cfg.mac.yaml` per la configurazione macOS.

### Critico, training e configurazione

- [ ] Nel prossimo training monitorare `vf_explained_var` e divario
      predetto-vs-return; modificare `vf_clip_param` solo in base ai dati.
- [ ] Valutare di esporre `vf_clip_param` e `vf_loss_coeff` come flag CLI.
- [ ] Lanciare e confrontare un training reale con asymmetric actor-critic.
- [ ] Rimuovere in futuro gli alias deprecati `--fcnet-hiddens` e
      `--critic-privileged-observation`.
- [ ] Usare il checkpoint imitativo come warm-start quando la reward ex-novo
      task-based sara implementata e validata.

### Reward ex-novo

- [ ] Confermare formalmente impulso propulsivo protesico e coordinazione
      inter-limb come obiettivo primario V1.
- [ ] Verificare assi, segni e normalizzazione delle GRF AP sul modello AB06.
- [ ] Definire finestre di braking/propulsion rispetto al gait clock.
- [ ] Decidere tra confronto diretto con GRF sana prescritta e bande funzionali.
- [ ] Calibrare `Fz_min`, `Fz_swing_max`, `Fz_max`, `slip_tolerance` e `u_soft`.
- [ ] Rendere lo slip aggregato contact-weighted.
- [ ] Esporre statistiche di saturazione su tutti i campioni del segmento.
- [ ] Mascherare target IK e `kin_ref` nel critic ex-novo preservando il
      warm-start imitativo.
- [ ] Implementare prima le nuove metriche in modalita diagnostica.
- [ ] Definire e automatizzare i test comportamentali della reward.
- [ ] Introdurre auto-periodicita solo dopo la validazione del task V1 e
      mantenere inizialmente `sea_energy_weight = 0`.

### Diagnostica biomeccanica e qualita

- [ ] Investigare ulteriormente reserve biologiche elevate e richieste sui DOF
      non attuati.
- [ ] Popolare o marcare esplicitamente come indisponibili i canali
      `SEA_*_torque_error_integral_dot`.
- [ ] Migrare in futuro il gait clock dal lato sano prescribed a un segnale
      ipsilaterale/IMU e valutare memoria o ricorrenza nell'attore.
- [ ] Validare heel-strike online, flag `in_contact`, rocker/COP push-off e
      contatto online del lato sano per traiettorie ex-novo divergenti.
- [ ] Ridurre il collo di bottiglia di throughput legato alla static
      optimization e ai fallback bounded least-squares.

### Porting MuJoCo/MJX

- [ ] Prima delle modifiche, verificare nuovamente lo stato VCS di
      `C:\Users\tomma\Desktop\MuJoCo_env` e ottenere approvazione prima di
      inizializzare Git.
- [ ] Creare la matrice obbligatoria
      `trajectory_generator/baseline_mlp/feature_parity.yaml`.
- [ ] Eseguire la Fase 0 e produrre il GO/NO-GO GPU/hybrid.
- [ ] Generare e congelare gli oracle OpenSim canonici.
- [ ] Creare il nuovo bundle target `AB06_SEASEA_stiff321_500_pi`.
- [ ] Chiudere i gate statici, SEA B1a/B1b e replay strict B2/D1.
- [ ] Eseguire il checkpoint strategico prima del porting hybrid.
- [ ] Portare e validare contatto hybrid C0/C/D2.
- [ ] Costruire l'ambiente JAX/MJX batched e chiudere il gate E.
- [ ] Superare benchmark v1-engine prima di integrare PPO.
- [ ] Integrare PPO JAX, rollout, supervisione, checkpoint e UX.
- [ ] Chiudere feature parity, gate finali scientifici, prestazionali e
      cross-platform.

### Housekeeping e filoni storici propagati

- [ ] Pulire launcher e log temporanei `run_imit_*.ps1` e `imit_*.log`.
- [ ] Pulire artefatti temporanei in `results/_*` e `validation/_*` e valutare
      la rimozione degli oggetti `build_online_grf/*.obj` da git.
- [ ] Proseguire i TODO SNN/skrl propagati: F1-F4/F7, cache `data_ptr`,
      reshape/BPTT, spike-gradient encoder, reader STO `inDegrees`,
      `sea_stiffness` per modello, riuso `InverseDynamicsSolver` e SNN come
      RLModule.
- [ ] Proseguire la revisione della letteratura: recuperare il paper Wrapyfi
      corretto e approfondire P17/P24/P14/P19/P23.
- [ ] Proseguire i TODO storici SEA: coupling knee-ankle, notch 28 Hz, cleanup
      modelli sperimentali, build/copia DLL PI Windows, secondo pass knee,
      confronto configurazioni e LPF qdot asimmetrico ankle.
