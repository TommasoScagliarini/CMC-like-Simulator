# Daily report - 2026-06-11

## Sintesi

La giornata ha chiuso il ciclo di pre-training imitativo e ha preparato in modo
piu robusto il passaggio verso il training ex-novo:

1. diagnosticato un secondo crash nativo Ray/Windows e reso il supervisore
   capace di recuperare anche uscite native non riportate;
2. completato il training imitativo a 40/40 iterazioni e validato il checkpoint
   finale;
3. eseguita e monitorata un'inference deterministica con GRF ibrida;
4. progettata la nuova reward ex-novo task-based sulla base della letteratura
   locale;
5. introdotto `training_cfg.yaml` come sorgente unica dei parametri, con
   snapshot risolto e auto-configurazione del rollout;
6. definiti i criteri di monitoraggio del critico per i prossimi training.

Report utente prodotti:

```text
reports/user/2026-06-11_diagnosi_crash_nativo_ray_e_recovery_training.md
reports/user/2026-06-11_monitoraggio_critico_vf_metrics_prossimo_training.md
reports/user/2026-06-11_progettazione_reward_ex_novo_task_based.md
reports/user/2026-06-11_completamento_run_imitativo_e_validazione_seed.md
reports/user/2026-06-11_inference_seed_imitativo_grf_ibrida.md
reports/user/2026-06-11_training_cfg_yaml_sorgente_unica_parametri.md
```

## 1. Crash nativo Ray e recovery

Il training imitativo si era arrestato durante l'iterazione logica 22/40. La
diagnosi sui log Ray ha confermato:

- uscita del `dashboard_agent` con exit code `1`;
- terminazione fate-shared del `raylet`;
- access violation native Windows durante disconnect/shutdown;
- perdita dei 12 EnvRunner;
- nessuna evidenza di OOM, timeout, divergenza della policy, errore reward o
  crash deterministico OpenSim.

La causa radice del crash del `dashboard_agent` resta sconosciuta. La
combinazione osservata e Python 3.12.13, Ray 2.55.1, grpcio 1.81.0, protobuf
7.35.0 e torch 2.7.0.

Il supervisore Python e stato esteso per:

- riconoscere summary non aggiornate, uscite non-zero ed errori Python;
- terminare eventuali discendenti Ray residui;
- validare `checkpoint_last` e relativi metadati;
- ripetere la stessa iterazione interrotta;
- registrare cronologia e conteggio dei crash;
- arrestare loop di crash senza avanzamento;
- esporre `--max-consecutive-crash-restarts`, default `5`.

I probe deterministici di crash e recovery sono passati. Resta da osservare il
nuovo percorso di auto-recovery durante un access violation reale.

## 2. Training imitativo completato

Il run:

`Trajectory Generator/runs/baseline_mlp_imit_win`

ha raggiunto regolarmente 40/40 iterazioni:

```text
ok                       = true
stop_reason              = completed
best return              = 47.614022485910034
best iteration           = 40
vf_explained_var finale  = 0.811668872833252
vf_loss finale           = 0.25241801142692566
```

Il return e cresciuto da `7.5496` a `47.6140`. Il critico e rimasto sano nella
parte finale del run, con explained variance positiva. Le terminazioni sono
state dominate dal normale limite temporale dell'episodio, con eventi
`grf_penetration` minoritari.

Il checkpoint da usare per inference e:

`Trajectory Generator/runs/baseline_mlp_imit_win/rl_module_best`

`rl_module_best` e `rl_module_last` coincidono e hanno SHA-256:

`9B42621B90BA40DF140C218E8A1C81F23E3C0D235CC49773F6C6A1647CF4118A`

Il TODO del daily precedente relativo al completamento e alla selezione del
seed finale e quindi chiuso.

## 3. Inference deterministica e GRF ibrida

E stata completata un'inference deterministica registrando gli output completi
in:

`Trajectory Generator/runs/baseline_mlp_imit_win_rollout`

Risultati principali:

```text
ok                  = true
return code         = 0
policy steps        = 201
episode return      = 92.20533942006523
reward mean         = 0.45873303194062304
action abs max      = 0.9862317442893982
pelvis_ty min       = 0.9263371789473765
terminated          = false
truncated           = true, normale time limit
wall time           = circa 371.25 s
```

Il log conferma la configurazione biomeccanica richiesta:

- lato protesico sinistro: GRF online fisicamente applicata e GRF prescritta
  disabilitata nella dinamica;
- lato sano destro: GRF prescritta applicata;
- 20 contatti totali, con 8 applicati sul lato sinistro e 12 sensor-only.

Sono stati prodotti 16 file STO da 2001 righe ciascuno e i CSV degli eventi del
passo.

La diagnostica ha evidenziato:

- saturazione SEA ginocchio: `679 / 2001`, circa `33.93%`;
- saturazione SEA caviglia: `0 / 2001`;
- reserve biologiche elevate, soprattutto sui DOF non attuati;
- residuo di equilibrio medio e massimo pari a zero;
- valori NaN limitati ai canali diagnostici opzionali
  `SEA_Knee_torque_error_integral_dot` e
  `SEA_Ankle_torque_error_integral_dot`.

Il TODO relativo al rollout deterministico del seed e chiuso. Restano aperte la
validazione quantitativa della qualita imitativa e l'analisi delle criticita
biomeccaniche emerse.

## 4. Configurazione YAML unica

E stato introdotto:

`Trajectory Generator/baseline_MLP/training_cfg.yaml`

come riferimento canonico dei parametri di training. Il loader condiviso
`training_config.py` applica la precedenza:

```text
default built-in < YAML < override CLI esplicito
```

Il training salva ora la configurazione effettivamente risolta in:

`<output_dir>/training_cfg.resolved.yaml`

Il rollout la carica automaticamente partendo dal checkpoint, evitando mismatch
silenziosi di env, rete, reward e GRF. `--no-auto-config` consente di
disabilitare il comportamento.

Altre decisioni:

- rete espressa come `num_hidden_layers` per `dim_hidden_layers`;
- `asymmetric_actor_critic` sostituisce il nome pubblico
  `critic_privileged_observation`;
- `action_mode`, `max_delta_rad` e limiti OOB restano fuori dal YAML canonico;
- alias deprecati preservati per compatibilita con launcher e checkpoint in
  corso.

Questa modifica chiude, per i run futuri, il TODO relativo al salvataggio della
configurazione accanto al checkpoint.

## 5. Reward ex-novo task-based

La reward `ex_novo` corrente e stata riconosciuta come ancora troppo imitativa.
E stata progettata una V1 task-based che non prescriva una cinematica
articolare sperimentale e separi:

- obiettivo funzionale;
- fattibilita fisica;
- regolarizzazione;
- sicurezza hard.

Obiettivo primario proposto:

- impulso propulsivo antero-posteriore protesico;
- coordinazione dinamica inter-limb durante la transizione;
- confronto iniziale con la GRF sana prescritta, da rendere meno prescrittivo
  in seguito.

Fattibilita e sicurezza:

- supporto verticale minimo durante stance;
- carico indesiderato durante swing e overload;
- slip pesato dal carico;
- penetrazione;
- saturazione SEA misurata su tutti i campioni interni del segmento;
- smoothness, command-rate e limiti articolari.

In modalita `ex_novo`, target IK protesici e `kin_ref` dovranno essere
mascherati per il critico. La nuova reward verra prima introdotta come
diagnostica e verificata con test comportamentali controllati.

## 6. Monitoraggio del critico

I nuovi default predisposti per i prossimi training sono:

```text
gamma = 0.95
lambda = 0.9
vf_clip_param = default 10, invariato
```

Il prossimo training dovra monitorare:

- `vf_explained_var`;
- divario tra valore predetto e return osservato;
- confronto tra critico simmetrico e asymmetric actor-critic.

`vf_clip_param` andra aumentato o disabilitato solo se i dati mostrano un
critico sistematicamente strozzato dal clipping.

## File modificati o aggiunti oggi

```text
Trajectory Generator/baseline_MLP/train_ppo_mlp.py
Trajectory Generator/baseline_MLP/rollout_eval.py
Trajectory Generator/baseline_MLP/README.md
Trajectory Generator/baseline_MLP/commands.txt
Trajectory Generator/baseline_MLP/training_cfg.yaml
Trajectory Generator/baseline_MLP/training_config.py
validation/validate_training_config.py
reports/user/2026-06-11_*.md
reports/daily/2026-06-11_daily-report.md
```

Nessuna modifica al plugin C++ SEA o alla semantica del comando SEA.

## Verifiche eseguite

- Analisi dei log Ray e ricostruzione del crash nativo all'iterazione 22.
- Probe recovery su crash nativo simulato, summary vecchia e crash ripetuti:
  PASS.
- `py_compile` dei moduli Python modificati: PASS.
- Training imitativo reale completato a 40/40: PASS.
- Checkpoint best/last e RLModule best/last verificati: PASS.
- Inference deterministica completa con registrazione output: PASS.
- Semantica GRF ibrida sinistra-online/destra-prescritta verificata: PASS.
- Loader configurazione YAML: 28/28 test di regressione PASS.
- Training funzionale solo YAML e con override CLI: PASS.
- Rollout con auto-match dallo snapshot: PASS.
- Analisi della letteratura locale e verifica dei segnali disponibili per la
  reward ex-novo completate.

## TODO aperti e propagati

### Recovery, runtime e cross-platform

- [ ] Validare l'auto-recovery interno durante un crash nativo Ray reale, con
      `--checkpoint-every 1` e `--max-consecutive-crash-restarts 5`.
- [ ] Rendere esplicito e inequivocabile nei log quale componente ha eseguito
      ogni restart.
- [ ] Stabilizzare e validare l'ambiente Ray su Windows tramite una matrice
      controllata Python/Ray/grpcio/protobuf.
- [ ] Raccogliere un dump nativo piu informativo del prossimo access violation.
- [ ] Verificare su macOS arm64 cleanup del process group, resume RLlib,
      round-trip RLModule, reward mode e warm-start; ricompilare onlineGRF
      `.dylib` e verificare il vincolo `setuptools<81`.

### Training, critico e configurazione

- [ ] Nel prossimo training monitorare `vf_explained_var` e divario
      predetto-vs-return; modificare `vf_clip_param` solo in base ai dati.
- [ ] Valutare di esporre `vf_clip_param` e `vf_loss_coeff` come flag CLI.
- [ ] Lanciare e confrontare un training reale con asymmetric actor-critic.
- [ ] Tarare `imitation_phase_shift` e i parametri `blend_*` su dati
      quantitativi del seed.
- [ ] Valutare `training_cfg.mac.yaml` per la configurazione macOS.
- [ ] Rimuovere in futuro gli alias deprecati `--fcnet-hiddens` e
      `--critic-privileged-observation`, dopo l'esaurimento dei launcher e
      checkpoint storici.
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

### Diagnostica biomeccanica e qualita del seed

- [ ] Validare la qualita imitativa con RMSE cinematico e coerenza di fase.
- [ ] Analizzare e ridurre la saturazione SEA del ginocchio, circa 34%.
- [ ] Investigare reserve biologiche elevate e richieste sui DOF non attuati.
- [ ] Analizzare `joint_divergence_pros_knee_angle`, penalita OOB e impatto del
      filtro di riferimento a 6 Hz.
- [ ] Popolare o marcare esplicitamente come indisponibili i canali
      `SEA_*_torque_error_integral_dot`.
- [ ] Migrare in futuro il gait clock dal lato sano prescribed a un segnale
      ipsilaterale/IMU e valutare memoria o ricorrenza nell'attore.
- [ ] Validare heel-strike online, flag `in_contact`, rocker/COP push-off e
      contatto online del lato sano per traiettorie ex-novo divergenti.
- [ ] Ridurre il collo di bottiglia di throughput, circa `1.18 s/env-step`,
      legato alla static optimization e ai fallback bounded least-squares.

### Housekeeping e filoni propagati

- [ ] Pulire launcher e log temporanei `run_imit_*.ps1` e `imit_*.log`, ora che
      il run imitativo e concluso.
- [ ] Pulire gli artefatti temporanei in `results/_*` e `validation/_*` e
      valutare la rimozione degli oggetti `build_online_grf/*.obj` da git.
- [ ] Proseguire i TODO SNN/skrl propagati: F1-F4/F7, cache `data_ptr`,
      reshape/BPTT, spike-gradient encoder, reader STO `inDegrees`,
      `sea_stiffness` per modello, riuso `InverseDynamicsSolver` e SNN come
      RLModule.
- [ ] Proseguire la revisione della letteratura: recuperare il paper Wrapyfi
      corretto e approfondire P17/P24/P14/P19/P23.
- [ ] Proseguire i TODO storici SEA: sweep `Kp_knee_motor`, coupling
      knee-ankle, notch 28 Hz, cleanup modelli sperimentali, build/copia DLL PI
      Windows, secondo pass knee, confronto configurazioni e LPF qdot
      asimmetrico ankle.
