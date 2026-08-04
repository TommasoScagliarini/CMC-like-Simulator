# Daily report - 2026-06-25

Instruction check token: CMC_AGENT_OK_2026

## Sintesi

La giornata e stata centrata su due linee di lavoro:

1. chiudere la diagnosi delle varianti imitative con penalita GRF/knee jerk;
2. impostare il passaggio concettuale e operativo verso la generazione `ex_novo`.

Sul lato imitativo, la variante con knee jerk limit a `2000 rad/s^3` ha ridotto clipping e contenuto ad alta frequenza della coppia al ginocchio, ma ha peggiorato imitazione, contatto/carico online, reserve e comportamento alla caviglia. Non e stata promossa.

La configurazione imitativa e stata riportata a knee jerk `3000 rad/s^3` e penalita GRF piu morbide:

- `grf_penetration_weight: 0.5`
- `grf_ankle_moment_flip_weight: 0.05`
- `iterations: 40`

Il training 40 iter `MLP_imitation_training_06-25-2026_grfhalf_40iter` e stato completato con successo. Il best checkpoint risulta alla logical iteration 37 con train return medio circa `230.62`. Manca ancora il rollout biomeccanico del best per decidere se questa run diventa la nuova baseline imitativa.

Sul lato `ex_novo`, e stato creato e separato un nuovo file di configurazione:

- `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml`

La reward ex-novo e stata resa piu libera dai dati prescribed disattivando:

- `reference_score` tramite `blend_reference: 0.0`
- `bio_score` tramite `blend_bio: 0.0`

Rimane attivo solo un termine positivo povero di trackability della reference generata, con `blend_tracking: 0.25`, piu penalita/guardrail. Questo e intenzionalmente un punto di partenza, non una reward task-based finale.

Nessuna modifica al plugin C++ `SeriesElasticActuator` o alla semantica low-level del comando SEA.

## Report e piani inclusi

Report utente del giorno:

- `reports/user/2026-06-25_analisi_knee_jerk2000_e_sweetspot_grf.md`
- `reports/user/2026-06-25_smoke_grfhalf_e_training_40iter.md`
- `reports/user/2026-06-25_transizione_ex_novo_warmstart_imitativo.md`
- `reports/user/2026-06-25_piano_migrazione_imitation_exnovo_cfg_transplant.md`
- `reports/user/2026-06-25_reward_exnovo_prescribed_free_fase_protesica_critic.md`

Piano scritto:

- `reports/plans/2026-06-25_migrazione_imitation_exnovo_actor_transplant.md`

## Diagnosi knee jerk 2000

Run analizzata:

- training: `Trajectory Generator/runs/training/MLP_imitation_training_06-24-2026`
- rollout: `Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-24-2026`
- plot: `plot/06_25_2026_1__asym100_GRFpenalty-lowered2`

Configurazione testata:

- `grf_penetration_weight: 1.0`
- `grf_ankle_moment_flip_weight: 0.10`
- `pros_knee_ref_jerk_limit_rad_s3: 2000.0`

Risultato principale:

- il limite jerk 2000 migliora la regolarita della coppia ginocchio e riduce il clipping;
- peggiora pero imitazione e biomeccanica complessiva;
- il bottleneck sembra stare nella reference servita dalla policy, non nel tracking SEA della reference servita.

Confronto sintetico:

- baseline soft GRF: rollout return `348.5`, served imitation loss `0.0298`, knee target-served RMSE `0.137 rad`, left Fy mean `61.0 N`, root reserve RMS `462.6`, ankle negative area `-11.30 Nms`;
- jerk2000: rollout return `334.8`, served imitation loss `0.0491`, knee target-served RMSE `0.181 rad`, left Fy mean `42.3 N`, root reserve RMS `486.3`, ankle negative area `-7.20 Nms`;
- knee tau_spring HP20 ratio migliora `0.460 -> 0.275`;
- knee clip fraction migliora `0.040 -> 0.008`.

Decisione:

- non promuovere jerk 2000;
- ripristinare `pros_knee_ref_jerk_limit_rad_s3: 3000.0`.

## GRF half e training 40 iter

Obiettivo:

- testare una penalita GRF piu morbida, per evitare che la policy sacrifichi troppo imitazione e carico protesico.

Configurazione applicata a `Trajectory Generator/baseline_MLP/training_cfg.yaml`:

- `iterations: 40`
- `pros_knee_ref_jerk_limit_rad_s3: 3000.0`
- `grf_penetration_weight: 0.5`
- `grf_ankle_moment_flip_weight: 0.05`

Problema operativo rilevato:

- lanciando da `envCMC-like` si ottiene `ModuleNotFoundError: No module named 'ray'`;
- PPO/RLlib deve essere lanciato da `envCMC-rllib`.

Smoke training:

- run: `Trajectory Generator/runs/training/MLP_imitation_training_06-25-2026_grfhalf_smoke_10iter`
- completate 9 iterazioni utili su 10 pianificate;
- best iter 9;
- best train return `56.355`;
- presenti `checkpoint_best`, `checkpoint_last`, `rl_module_best`, `rl_module_last`.

Smoke rollout:

- rollout: `Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-25-2026_grfhalf_smoke_9iter_best`
- rollout return `146.98`;
- reward mean `0.293`;
- action clip fraction `0.0`;
- end reason `episode_time_limit`;
- left Fy mean solo `5.4 N`;
- nessun campione valido nella fase critica `0.15-0.40` con Fy > `50 N`.

Interpretazione:

- lo smoke conferma la pipeline, ma non valida ancora la biomeccanica;
- la coppia di pesi `0.5/0.05` resta plausibile ma non confermata.

Training completo rilevato a fine giornata:

- run: `Trajectory Generator/runs/training/MLP_imitation_training_06-25-2026_grfhalf_40iter`
- completamento: `40/40`
- best logical iteration: `37`
- best train return medio: `230.61697734086567`
- start: `2026-06-25T16:13:43+02:00`
- finish: `2026-06-25T20:56:37+02:00`

Nota:

- il training 40 iter e completato, ma non risulta ancora valutato con rollout/plot biomeccanico. La decisione di promuoverlo a baseline imitativa resta aperta.

## Migrazione imitation -> ex_novo

E stato chiarito che un semplice:

```bash
--resume-from checkpoint_best --reward-mode ex_novo
```

non e una transizione pulita se il checkpoint imitativo e stato addestrato con quattro feature actor che in `ex_novo` non devono esserci:

- `healthy_knee_angle_imitation_target`
- `healthy_knee_angle_imitation_target_vel`
- `healthy_ankle_angle_imitation_target`
- `healthy_ankle_angle_imitation_target_vel`

Il problema e doppio:

- lo schema dell'actor cambia;
- il critic/value function e stato addestrato su una reward diversa.

La strategia proposta e:

- copiare i pesi dell'actor per nome feature, mantenendo solo le colonne compatibili del primo layer;
- gestire le quattro feature rimosse con modalita `drop` o `mean-bias`;
- re-inizializzare il critic nel primo esperimento;
- salvare un initializer ex-novo compatibile;
- confrontare random init, transplant drop e transplant mean-bias.

Razionale del bias compensation:

```text
h = activation(Wx + b)
```

Se si rimuovono feature che durante imitation avevano media non nulla, imporre implicitamente `x_removed = 0` puo spostare le attivazioni del primo layer. La compensazione:

```text
b_new = b_old + W_removed * mean(x_removed)
```

conserva meglio la distribuzione pre-attivazione attesa dal primo layer. Non e obbligatoria, ma e un ablation pulito e misurabile.

## Configurazione ex_novo

Creato:

- `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml`

Stato reward corrente:

- `reward_mode: ex_novo`
- `blend_tracking: 0.25`
- `blend_reference: 0.0`
- `blend_bio: 0.0`

Con questa configurazione:

```text
reward_base = clip(0.25 * tracking_score - penalty, 0.0, 1.0)
```

Il `reference_score` e disabilitato:

- non premia piu la vicinanza dei DOF protesici alla cinematica IK/prescribed base.

Il `bio_score` e disabilitato:

- non premia piu il tracking del contesto biologico verso i dati prescribed.

Le penalita ancora attive includono:

- banda OOB della reference comandata;
- effort/saturation/torque-error/motor stress SEA;
- command-rate/smoothness;
- safety;
- GRF penetration;
- GRF ankle moment flip guard.

La nuova penalita GRF e presente anche nel file ex-novo:

- `grf_penetration_weight: 0.5`
- `grf_ankle_moment_flip_weight: 0.05`
- `grf_ankle_moment_flip_tau_tol_nm: 8.0`
- `grf_ankle_moment_flip_force_threshold_n: 50.0`

Interpretazione:

- la reward attuale e prescribed-free rispetto a `reference_score` e `bio_score`;
- e pero ancora molto povera come reward di cammino;
- serve aggiungere termini task-based che non usino dati prescribed.

## Fase protesica e critic asimmetrico

Problema emerso:

- se non si vogliono usare dati della sound leg, non si puo continuare a dipendere da heel strike/toe off della gamba sana o da una fase prescribed per definire il gait cycle dell'actor.

Proposta:

- costruire una fase online della protesi usando HS/TO rilevati dalla GRF protesica;
- `HS_pros -> phi = 0`;
- `TO_pros -> transizione stance/swing`;
- HS successivo chiude il ciclo;
- stimare il periodo con EMA degli intervalli HS-HS;
- stimare anche stance fraction da TO-HS.

Rischi da gestire:

- periodo prima del primo HS;
- eventi rumorosi;
- debounce e durata minima del contatto;
- cicli mancati;
- possibilita che la policy manipoli GRF/eventi.

Critic onnisciente:

- in AC asimmetrico l'actor legge solo `obs[:n_actor]`;
- il critic legge l'osservazione completa;
- il critic non passa direttamente fase o dati privilegiati all'actor in inference;
- influenza pero gli advantage tramite `advantage = return - V_critic(full_obs)`.

Conclusione:

- un critic con fase sana/prescribed puo ridurre varianza e migliorare la stima del valore;
- non risolve il problema di rappresentazione della fase per l'actor;
- puo creare credit assignment incoerente se la reward e l'actor vogliono essere prescribed-free.

## File modificati o creati

File di configurazione:

- `Trajectory Generator/baseline_MLP/training_cfg.yaml`
- `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml`

Report utente:

- `reports/user/2026-06-25_analisi_knee_jerk2000_e_sweetspot_grf.md`
- `reports/user/2026-06-25_smoke_grfhalf_e_training_40iter.md`
- `reports/user/2026-06-25_transizione_ex_novo_warmstart_imitativo.md`
- `reports/user/2026-06-25_piano_migrazione_imitation_exnovo_cfg_transplant.md`
- `reports/user/2026-06-25_reward_exnovo_prescribed_free_fase_protesica_critic.md`

Piano:

- `reports/plans/2026-06-25_migrazione_imitation_exnovo_actor_transplant.md`

Run e output prodotti o analizzati:

- `Trajectory Generator/runs/training/MLP_imitation_training_06-25-2026_grfhalf_smoke_10iter`
- `Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-25-2026_grfhalf_smoke_9iter_best`
- `Trajectory Generator/runs/training/MLP_imitation_training_06-25-2026_grfhalf_40iter`
- `plot/06_25_2026_1__asym100_GRFpenalty-lowered2`

## Verifiche eseguite

Verifiche configurazione ex-novo:

- `git diff --check -- "Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml"`
- loader config con output atteso: `ex_novo 0.25 0.0 0.0`

Verifiche training:

- smoke training GRF half completato fino a iterazione utile 9;
- smoke rollout completato con `episode_time_limit`;
- training 40 iter `grfhalf_40iter` completato `40/40`;
- presenza di `summary.json`, checkpoint e moduli best/last.

Verifiche non ancora eseguite:

- rollout biomeccanico del best checkpoint `grfhalf_40iter`;
- confronto plot/metriche contro baseline soft GRF precedente;
- ablation ex-novo con actor transplant;
- test funzionale della reward ex-novo povera per degenerazioni.

## TODO chiusi o avanzati il 25/06

- Chiusa la valutazione knee jerk 2000: non promossa.
- Ripristinato knee jerk 3000 nel config imitativo.
- Applicata penalita GRF piu morbida `0.5/0.05`.
- Eseguito smoke training e rollout della configurazione GRF half.
- Lanciato e completato training imitativo 40 iter GRF half.
- Creato `training_exnovo_cfg.yaml`.
- Disattivati `reference_score` e `bio_score` nella reward ex-novo.
- Formalizzato il piano actor transplant imitation -> ex_novo.

## TODO aperti e propagati

### Baseline imitativa GRF half

- Eseguire rollout deterministico del best checkpoint `MLP_imitation_training_06-25-2026_grfhalf_40iter`, best logical iteration 37.
- Generare plot/diagnostica MLP per la run 40 iter.
- Ripetere il confronto in fase critica `0.15-0.40` con Fy > `50 N`.
- Promuovere `grfhalf_40iter` solo se:
  - stance protesica valida;
  - knee RMSE target-served migliore o non peggiore della soft `1.0/0.10`;
  - ankle tau ancora negativa in early/mid stance;
  - root reserve non peggiore della baseline soft matura.
- Confrontare contro:
  - baseline soft GRF `1.0/0.10`;
  - jerk2000;
  - hard/original se utile;
  - eventuale `sym60` come anti-chattering baseline.

### Diagnostica ginocchio e GRF

- Fare diagnosi data-driven stance/swing del degrado al ginocchio.
- Incrociare knee target, served, tau, action, clipping, COP e GRF.
- Separare errore di policy/reference da errore SEA tracking.
- Decidere se intervenire con:
  - ulteriore riduzione GRF;
  - penalty knee-specific;
  - bounded action head;
  - weak contact-confidence term.
- Non adottare ancora la variante acceleration knee `45/30` senza diagnosi dedicata.

### Reward ex_novo

- Definire i primi termini task-based prescribed-free.
- Candidati da progettare:
  - contact/load reward protesico;
  - swing unloading;
  - contact validity;
  - regolarita di fase/eventi;
  - range articolari protesici;
  - uso funzionale SEA;
  - stabilita/survival;
  - costo reserve/residui, con cautela.
- Testare la reward ex-novo attuale, anche se povera, per identificare degenerazioni immediate.
- Separare esplicitamente nei prossimi report:
  - reward prescribed-free;
  - actor prescribed-free;
  - critic privileged/prescribed-free.

### Fase protesica online

- Rimuovere dall'actor la dipendenza da `gait_phase_sin/cos` sound-side/prescribed oppure sostituirla.
- Implementare fase online protesica da HS/TO rilevati tramite GRF.
- Gestire periodo prima del primo HS.
- Aggiungere debounce e durata minima di contatto.
- Gestire eventi mancati o spuri.
- Loggare diagnostiche su:
  - HS/TO reliability;
  - contatti spuri;
  - cicli mancati;
  - periodo stimato;
  - stance fraction stimata.
- Decidere se il critic puo mantenere fase sana/prescribed o se deve essere ripulito per coerenza sperimentale.

### Migrazione imitation -> ex_novo

- Scegliere sorgente warm-start:
  - baseline `grfsoft_knee1_ankle2_100iter`;
  - oppure `grfhalf_40iter` dopo rollout e validazione.
- Creare `Trajectory Generator/baseline_MLP/training_imitation_cfg.yaml`.
- Decidere il ruolo legacy di `training_cfg.yaml`:
  - alias temporaneo;
  - deprecazione;
  - copia del config imitativo.
- Aggiornare CLI training:
  - `--imitation` usa `training_imitation_cfg.yaml`;
  - `--exnovo` usa `training_exnovo_cfg.yaml`;
  - default `--imitation`;
  - evitare combinazioni ambigue con `--config`.
- Aggiornare `training_config.py`.
- Aggiornare `train_ppo_mlp.py`.
- Aggiornare README/comandi.
- Implementare `Trajectory Generator/baseline_MLP/transfer_imitation_to_exnovo.py`.
- Il tool deve:
  - caricare `rl_module_best` imitativo;
  - ricostruire schema osservazioni sorgente da `training_cfg.resolved.yaml`;
  - ricostruire schema target da `training_exnovo_cfg.yaml`;
  - copiare pesi actor per nome feature;
  - gestire le quattro feature healthy target rimosse;
  - re-inizializzare critic nel primo esperimento;
  - salvare initializer compatibile ex-novo;
  - scrivere `actor_transplant_report.json`.
- Implementare modalita:
  - `--removed-feature-mode drop`;
  - `--removed-feature-mode mean-bias`.
- Stimare `mean(x_removed)` dai dati normalizzati usati in imitation.
- Validare iteration 0:
  - source imitation;
  - ex-novo random;
  - ex-novo transplant drop;
  - ex-novo transplant mean-bias.
- Eseguire ablation random vs transplant su training ex-novo.

### Documentazione e metodo

- Nei report futuri distinguere sempre:
  - target imitation;
  - reference IK/prescribed;
  - served reference;
  - tracking SEA;
  - reward functional/task-based.
- Propagare i TODO non chiusi nei daily successivi.

## TODO storico SEA propagato

- [ ] Valutare una deflessione SEA iniziale coerente con la coppia richiesta;
      il punto progettuale del 13/06 non risulta ancora formalmente chiuso.
