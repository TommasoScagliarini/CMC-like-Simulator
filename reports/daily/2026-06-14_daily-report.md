# Daily report - 2026-06-14

## Sintesi

La giornata del 14 giugno 2026 ha consolidato la configurazione PI come default
Windows, caratterizzato frequenzialmente l'intera catena protesica e
implementato una nuova configurazione imitativa V4 pronta per il training.

Le attivita principali sono state:

1. allineato il plugin SEA PI Windows alla configurazione gia validata su
   macOS;
2. promosso il bundle AB06 `stiff321_500_pi` e l'outer cascade PI a default;
3. completato una simulazione Windows PI da `11.99` a `16.0 s`;
4. redatto un'analisi frequenziale multilivello di plant, outer cascade,
   motor driver PI, reference model e policy;
5. confermato che la frequenza policy e la banda del riferimento servito sono
   grandezze distinte;
6. progettato e implementato il training imitativo V4;
7. sostituito, per V4, il reference model del secondo ordine con un modello
   Butterworth del terzo ordine C2 e jerk-limited;
8. ridotto l'azione policy V4 a un solo comando posizione per giunto;
9. separato nella reward il riferimento generato, l'imitazione fisica e il
   tracking del riferimento servito;
10. corretto il reset SEA inizializzando anche `motor_speed` con la velocita del
    giunto;
11. validato il contratto V4 con test isolati, oracle OpenSim, smoke RLlib
    single-process, smoke parallelo e inference da checkpoint.

Il training lungo V4 non e stato eseguito durante la giornata. La
configurazione e il comando di lancio sono pronti e il nuovo run deve partire
da zero, senza resume dalla V3, perche action space, observation space e reward
sono cambiati.

## Report e piani prodotti

Report utente:

```text
reports/user/2026-06-13_progettazione_training_imitativo_v4_frequenze_reference_model_c2.md
reports/user/2026-06-14_allineamento_windows_plugin_pi.md
reports/user/2026-06-14_validazione_simulatore.md
reports/user/2026-06-14_analisi_frequenziale_configurazione_pi.md
reports/user/2026-06-14_aggiornamento_progettazione_v4_dopo_analisi_frequenziale.md
```

Piano operativo:

```text
reports/plans/2026-06-14_implementazione_training_imitativo_v4_lancio.md
```

## 1. Allineamento Windows del plugin PI

Il modello AB06 PI conteneva gia i parametri `Ki` e
`integral_torque_limit`, ma Windows caricava una DLL precedente priva dello
stato `torque_error_integral`.

E stata compilata, verificata e installata la DLL PI Windows:

```text
plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff.dll
```

La DLL precedente e stata conservata come:

```text
plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff.dll.bak_pre_pi_20260614_182653
```

Configurazione resa default:

| Livello | Knee | Ankle |
|---|---:|---:|
| Outer `Kp_outer` | `18.85` | `47.125` |
| Outer `Kp_inner` | `29.2` | `2.8275` |
| Outer `Ki_inner` | `1377.0` | `213.0` |
| Motor driver `Kp` | `18.0` | `11.3` |
| Motor driver `Kd` | `11.0` | `11.0` |
| Motor driver `Ki` | `190.0` | `123.0` |
| SEA stiffness | `321 Nm/rad` | `500 Nm/rad` |

La legge PI runtime validata e:

```text
tau_input_raw = tau_ref
              + Kp * (tau_ref - tau_spring)
              + clamp(Ki * xi, -integral_torque_limit, +integral_torque_limit)
              - Kd * omega_m
```

Il termine feedforward `+tau_ref` e parte della configurazione validata.

Sono stati allineati anche:

- modello e bundle AB06 PI default;
- setup XML fallback cross-platform;
- finestra default;
- stiffness fallback;
- documentazione del controller inner PI.

## 2. Validazione Windows PI

E stata completata una simulazione CMC-like Windows:

```text
t = 11.99-16.0 s
step = 4010
output = results/_windows_pi_alignment_11.99_16
```

Risultati specifici SEA/PI:

- tutti i 6 stati SEA integrati;
- integrali PI finiti e variabili;
- nessun clamp `tau_input`;
- accordo plugin/Python sulla legge SEA entro `5e-6 Nm`;
- tracking protesico stabile.

| Metrica | Knee | Ankle |
|---|---:|---:|
| Range integrale coppia | `0.3230` | `0.2054` |
| Max errore plugin/Python | `5e-7 Nm` | `5e-6 Nm` |
| Max controllo `|u|` | `0.3316` | `0.4818` |
| Saturazione `tau_input` | nessuna | nessuna |

Il validatore globale ha restituito `FAIL` esclusivamente per:

```text
mtp_angle_r output vs IK:
RMSE = 13.84 deg
max  = 29.83 deg
```

Questo FAIL riguarda il tracking biologico globale e non invalida
l'allineamento del plugin PI. Rimane comunque un problema biomeccanico da
investigare.

## 3. Analisi frequenziale della catena PI

E stata caratterizzata separatamente la frequenza di callback digitale, la
banda dinamica e le risonanze dei diversi livelli.

Gerarchia risultante:

| Livello | Knee | Ankle |
|---|---:|---:|
| Reference model V3, banda reale `-3 dB` | `3.86 Hz` | `3.86 Hz` |
| Outer position correction | circa `3.0 Hz` | circa `7.5 Hz` |
| Outer cascade rapido | circa `9-15 Hz` | circa `24-25 Hz` |
| Risonanza meccanica SEA | `28.5 Hz` | `35.6 Hz` |
| Modo oscillatorio motor driver | `86.6 Hz` | `87.4 Hz` |
| Banda motor driver PI `-3 dB` | `124.6 Hz` | `125.7 Hz` |
| Policy V3/V4 | `100 Hz` | `100 Hz` |
| Callback outer e integrazione | `1000 Hz` | `1000 Hz` |

Conclusioni principali:

- il polo/zero PI attorno a `1.6 Hz` descrive l'azione integrale lenta, non la
  banda complessiva del driver;
- il motor driver PI e rapido e non costituisce un filtro protettivo contro
  contenuto indesiderato tra `20` e `100 Hz`;
- la protezione principale deve essere il reference model causale;
- ridurre la frequenza policy non elimina automaticamente le armoniche o
  l'eccitazione delle risonanze;
- una policy a `100 Hz` e utilizzabile solo se il riferimento servito attenua
  la riga policy ed e continuo almeno in accelerazione.

Il reference model V3 del secondo ordine attenua fortemente `q_ref` ad alta
frequenza, ma non attenua sufficientemente il trasferimento verso
`qddot_ref`. Inoltre la ricostruzione Hermite e C1, non globalmente C2.

## 4. Decisioni progettuali V4

Per isolare l'effetto del nuovo reference model sono state bloccate le seguenti
decisioni:

```text
policy rate                = 100 Hz
segment_duration           = 0.01 s
gamma                      = 0.95
policy_knots               = 1
azioni                     = q_cmd_knee, q_cmd_ankle
reference model            = Butterworth terzo ordine jerk-limited
banda reale iniziale       = 4 Hz
episode_duration iniziale  = 2.0 s
```

Il modello V4 usa lo stato:

```text
x_ref = [q_ref, qdot_ref, qddot_ref]
```

e la dinamica nominale:

```text
qref''' = wc^3 * (q_cmd - q_ref)
        - 2*wc^2 * qdot_ref
        - 2*wc * qddot_ref
```

Il nuovo modello rende continui:

- posizione;
- velocita;
- accelerazione.

Il jerk resta finito e viene limitato separatamente per knee e ankle.

## 5. Implementazione training imitativo V4

E stata creata la configurazione dedicata:

```text
Trajectory Generator/baseline_MLP/training_cfg.v4_imitation.yaml
```

Parametri principali:

```yaml
ppo:
  gamma: 0.95

simulation:
  segment_duration: 0.01
  episode_duration: 2.0
  policy_knots: 1
  pros_ref_model: butterworth3_jerk_limited
  pros_ref_cutoff_hz: 4.0
  pros_knee_ref_jerk_limit_rad_s3: 3000.0
  pros_ankle_ref_jerk_limit_rad_s3: 2750.0
  actor_cyclic_phase_only: true
  include_reference_state_observation: true
  imitation_initialize_to_target: true
```

La policy V4 emette solamente un comando posizione assoluto per giunto. Il
reference model trasforma tale comando in `q_ref`, `qdot_ref` e `qddot_ref`
coerenti.

L'attore riceve lo stato del reference model per preservare la proprieta
Markoviana. Sono state rimosse le phase scalari discontinue:

- phase normalizzata dell'episodio;
- gait phase scalare;
- fase scalare del sensore protesico.

Restano solamente le rappresentazioni cicliche seno/coseno.

## 6. Reward imitativa V4

La reward imitativa e stata separata in tre obiettivi:

| Termine | Significato |
|---|---|
| `served_imitation_loss` | qualita del riferimento generato rispetto al target |
| `sound_imitation_loss` | imitazione fisica effettiva rispetto al target |
| `tracking_loss` | capacita del plant di seguire il riferimento servito |

Blend iniziale V4:

```text
served-target = 0.65
actual-target = 0.20
actual-served = 0.15
```

Sono stati aggiunti o collegati al logging:

- jerk e `jerk_ref_loss`;
- intervento del jerk limit;
- diagnostica del reference model;
- `served_imitation_loss`;
- metriche SEA reali gia disponibili.

I default legacy sono stati preservati: il nuovo comportamento viene attivato
dal file YAML V4.

## 7. Reset imitativo e swing iniziale

Il confronto tra reset al target e convergenza dalla cinematica prescribed ha
mostrato che disattivare `imitation_initialize_to_target` crea un grande errore
imitativo inevitabile all'inizio dell'episodio.

La V4 mantiene quindi:

```text
imitation_initialize_to_target = true
```

ma corregge il reset SEA inizializzando:

- `motor_angle` con la posizione del giunto;
- `motor_speed` con la velocita del giunto;
- reference model con posizione e velocita del target;
- accelerazione iniziale del reference model a zero.

Nel dato registrato di verifica:

```text
SEA_Knee_motor_speed iniziale = 3.27873142 rad/s
```

coincidente con la velocita iniziale del giunto/riferimento, invece di partire
artificialmente da zero.

## 8. Validazione V4

### Test isolato del reference model

Con comandi alternati:

| Gate | Risultato |
|---|---:|
| Salto `q` ai confini | `0` |
| Salto `qdot` ai confini | `2.8e-13` |
| Salto `qddot` ai confini | `2.7e-9` |
| Velocita massima | `6 rad/s` |
| Accelerazione massima | `60 rad/s^2` |
| Jerk massimo | `3000 rad/s^3` |

### Oracle OpenSim breve

L'oracle breve con `Kp_knee=18` e reference model V4:

- completa regolarmente;
- non raggiunge il clamp SEA;
- conferma che `Kp=18` resta compatibile con un riferimento governato.

### Smoke training e inference

Sono stati completati:

- `py_compile` dei file modificati;
- `git diff --check`;
- risoluzione e snapshot del file YAML V4;
- smoke environment con action space V4;
- smoke RLlib single-process;
- smoke RLlib parallelo con `2` EnvRunner;
- aggiornamento PPO;
- salvataggio checkpoint e RLModule;
- inference deterministica da checkpoint;
- ricostruzione automatica della configurazione V4 dallo snapshot.

Audit observation/action:

```text
actor observation shape = 31
RLlib action shape       = 2
phase scalari actor      = nessuna
```

Il validatore legacy `validation/validate_training_config.py` supera tutti i
controlli precedenti al test snapshot, ma il test basato su
`TemporaryDirectory` fallisce nel sandbox corrente per permessi Windows. Il
round-trip snapshot e stato verificato con training e inference reali.

## 9. Comando training V4

Il comando e stato aggiunto a:

```text
Trajectory Generator/baseline_MLP/commands.txt
```

Comando pronto:

```powershell
C:\Users\tomma\anaconda3\Scripts\conda.exe run --no-capture-output -n envCMC-rllib python "Trajectory Generator\baseline_MLP\train_ppo_mlp.py" --config "Trajectory Generator\baseline_MLP\training_cfg.v4_imitation.yaml" --output-dir "runs\baseline_mlp_imit_v4_c2_4hz"
```

Il run deve partire da zero. Non usare `--resume-from` dalla V3.

## File modificati o aggiunti

### Allineamento PI e simulatore

```text
config.py
setup_io.py
main.py
plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff.dll
plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff.dll.bak_pre_pi_20260614_182653
```

### Trajectory Generator V4

```text
Trajectory Generator/osim_trj_cmc_like.py
Trajectory Generator/baseline_MLP/reward_function.py
Trajectory Generator/baseline_MLP/training_config.py
Trajectory Generator/baseline_MLP/train_ppo_mlp.py
Trajectory Generator/baseline_MLP/rollout_eval.py
Trajectory Generator/baseline_MLP/imitation_oracle_rollout.py
Trajectory Generator/baseline_MLP/tb_logging.py
Trajectory Generator/baseline_MLP/commands.txt
Trajectory Generator/baseline_MLP/training_cfg.v4_imitation.yaml
simulation_runner.py
```

### Documentazione

```text
reports/user/2026-06-14_*.md
reports/plans/2026-06-14_implementazione_training_imitativo_v4_lancio.md
reports/daily/2026-06-14_daily-report.md
```

## Output principali generati

```text
results/_windows_pi_alignment_11.99_16
Trajectory Generator/runs/_v4_oracle_smoke
Trajectory Generator/runs/_v4_oracle_continuous_reset
Trajectory Generator/runs/_v4_train_smoke
Trajectory Generator/runs/_v4_final_smoke
Trajectory Generator/runs/_v4_parallel_smoke
Trajectory Generator/runs/_v4_release_smoke
Trajectory Generator/runs/_v4_rollout_smoke
```

Gli output `_v4_*_smoke` sono diagnostici e non costituiscono il training
lungo V4.

## TODO chiusi il 14/6

- [x] Allineare e installare il plugin PI Windows.
- [x] Rendere persistente come default il setup AB06 PI.
- [x] Verificare runtime reale delle property e degli stati PI.
- [x] Eseguire una simulazione Windows completa e verificare assenza di clamp.
- [x] Correggere la precedente interpretazione del polo PI a `1.6 Hz`.
- [x] Distinguere policy update rate e served-reference bandwidth.
- [x] Implementare reference model del terzo ordine C2/jerk-limited.
- [x] Esporre tipo, banda e jerk limit del reference model a YAML, CLI,
      snapshot e rollout.
- [x] Ridurre l'azione V4 a un comando posizione per giunto.
- [x] Esporre all'attore lo stato del reference model.
- [x] Rimuovere le phase scalari dall'osservazione actor V4.
- [x] Aggiungere `served_imitation_loss` e separare i tre errori imitativi.
- [x] Correggere l'inizializzazione di `motor_speed` SEA.
- [x] Validare continuita C2 e rispetto dei limiti.
- [x] Verificare assenza di clamp SEA nell'oracle breve V4.
- [x] Eseguire smoke training V4 single-process e parallelo.
- [x] Eseguire inference da checkpoint e round-trip snapshot reale.
- [x] Preparare configurazione e comando del training lungo V4.
- [x] Completare lo smoke RLlib/Torch/OpenSim locale.

## TODO aperti e propagati

### Training imitativo V4 e validazione frequenziale

- [ ] Lanciare da zero il training lungo
      `baseline_mlp_imit_v4_c2_4hz`.
- [ ] Eseguire rollout deterministico del checkpoint V4 migliore.
- [ ] Confrontare V4 contro V3 e oracle su imitazione, tracking, clamp, reserve,
      contatto, penetrazione e stress SEA.
- [ ] Misurare quantitativamente FFT/PSD di `q_cmd_raw`, `q_ref`,
      `qdot_ref`, `qddot_ref`, jerk, `tau_ref` e `tau_input`.
- [ ] Verificare l'attenuazione reale della riga a `100 Hz` in `qddot_ref`.
- [ ] Verificare energia nelle bande `20-40 Hz` e `80-140 Hz`.
- [ ] Confrontare il reference model V3 e il modello C2 sulla stessa
      traiettoria nota.
- [ ] Confrontare `fc=4 Hz` e `fc=6 Hz` tramite oracle OpenSim.
- [ ] Tarare definitivamente i jerk limit knee/ankle su rollout completi.
- [ ] Confrontare `20/50/100 Hz` solo dopo il superamento dei gate spettrali.
- [ ] Completare il logging persistente esplicito di tutti i segnali raw/served
      necessari all'analisi spettrale.
- [ ] Migliorare ampiezza e fase della flessione knee rispetto al target
      imitativo.
- [ ] Ripetere il rollout lungo con gait cycle protesici completi.
- [ ] Valutare una distribuzione d'azione bounded/squashed per ridurre clipping
      dell'output policy.

### Simulatore PI e biomeccanica

- [ ] Investigare il FAIL di tracking biologico `mtp_angle_r`.
- [ ] Investigare reserve biologiche elevate e richieste sui DOF non attuati.
- [ ] Verificare il plugin PI e il setup default anche su macOS arm64.
- [ ] Validare `random_init=true` dopo la correzione del reset SEA.
- [ ] Popolare o marcare esplicitamente come indisponibili i canali
      `SEA_*_torque_error_integral_dot`.
- [ ] Validare heel-strike online, `in_contact`, rocker/COP push-off e contatto
      online del lato sano.
- [ ] Migrare in futuro il gait clock prescribed verso un segnale
      ipsilaterale/IMU.
- [ ] Ridurre il collo di bottiglia di throughput della static optimization e
      dei fallback bounded least-squares.

### Training, reward e runtime

- [ ] Monitorare nel training lungo V4 `vf_explained_var`, entropy,
      predetto-vs-return e saturazione delle azioni.
- [ ] Valutare di esporre `vf_clip_param` e `vf_loss_coeff` come flag CLI.
- [ ] Lanciare e confrontare un training reale con asymmetric actor-critic.
- [ ] Aggiungere una metrica/reward minima di supporto e carico protesico.
- [ ] Ridurre carico e penetrazione del piede protesico senza perdere contatto.
- [ ] Progettare e validare la reward ex-novo task-based prima del warm-start.
- [ ] Validare auto-recovery durante un crash nativo Ray reale.
- [ ] Rendere esplicito nei log quale componente ha eseguito ogni restart.
- [ ] Stabilizzare e validare la matrice Windows Python/Ray/grpcio/protobuf.
- [ ] Verificare su macOS arm64 cleanup, resume, RLModule e reward mode.
- [ ] Valutare `training_cfg.mac.yaml`.
- [ ] Rimuovere in futuro gli alias CLI deprecati.

### Reward ex-novo

- [ ] Confermare formalmente impulso propulsivo protesico e coordinazione
      inter-limb come obiettivo primario V1.
- [ ] Verificare assi, segni e normalizzazione delle GRF AP.
- [ ] Definire finestre braking/propulsion rispetto al gait clock.
- [ ] Decidere tra confronto con GRF sana prescritta e bande funzionali.
- [ ] Calibrare soglie di carico, swing, slip e controllo.
- [ ] Rendere lo slip aggregato contact-weighted.
- [ ] Esporre statistiche di saturazione su tutti i campioni del segmento.
- [ ] Mascherare target IK e `kin_ref` nel critic ex-novo.
- [ ] Implementare prima le nuove metriche in modalita diagnostica.
- [ ] Definire e automatizzare test comportamentali della reward.

### Porting MuJoCo/MJX

- [ ] Verificare lo stato VCS di `C:\Users\tomma\Desktop\MuJoCo_env` prima di
      iniziare il porting.
- [ ] Creare la matrice obbligatoria
      `trajectory_generator/baseline_mlp/feature_parity.yaml`.
- [ ] Eseguire la Fase 0 e produrre il GO/NO-GO GPU/hybrid.
- [ ] Generare e congelare gli oracle OpenSim canonici.
- [ ] Creare il bundle target `AB06_SEASEA_stiff321_500_pi`.
- [ ] Chiudere gate statici, SEA, replay strict e contatto hybrid.
- [ ] Costruire l'ambiente JAX/MJX batched e chiudere i benchmark engine.
- [ ] Integrare PPO JAX, rollout, supervisione, checkpoint e UX.
- [ ] Chiudere feature parity e gate finali scientifici, prestazionali e
      cross-platform.

### Housekeeping e filoni storici

- [ ] Pulire launcher/log temporanei `run_imit_*.ps1` e `imit_*.log`.
- [ ] Pulire artefatti temporanei `results/_*`, `validation/_*` e smoke V4
      quando non piu necessari.
- [ ] Correggere il test snapshot legacy che usa `TemporaryDirectory` nel
      sandbox Windows.
- [ ] Proseguire i TODO SNN/skrl propagati: F1-F4/F7, cache `data_ptr`,
      reshape/BPTT, spike-gradient encoder, reader STO `inDegrees`,
      `sea_stiffness`, riuso `InverseDynamicsSolver` e SNN come RLModule.
- [ ] Proseguire la revisione della letteratura e i TODO storici SEA ancora
      applicabili.
