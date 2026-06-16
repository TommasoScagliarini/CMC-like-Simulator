# Daily report - 2026-06-15

## Sintesi

Il 15/06 e' stato dedicato alla chiusura del ciclo sperimentale sul training
imitativo V4 e alla preparazione del passo successivo.

Il risultato principale e' che il tracking SEA non e' piu il collo di bottiglia:
la reference servita viene inseguita con errori molto bassi. Il problema
residuo e' la forma della reference generata/servita dalla policy rispetto al
target sano anti-fase, soprattutto su piu cicli del passo e sulla caviglia.

Sono stati inoltre diagnosticati due aspetti importanti:

- l'inizializzazione a `11.99 s` cade prima del primo heel strike destro valido
  e produce un prefisso dinamicamente incoerente;
- la policy imitativa non osservava esplicitamente il target sano anti-fase,
  ma lo riceveva solo indirettamente tramite reward.

Questo ha portato alla decisione operativa successiva: avviare un nuovo training
imitativo da zero con observation space esteso, partenza a `12.99 s` e nessun
resume dal checkpoint precedente.

## Report consolidati

- `reports/user/2026-06-15_analisi_rollout_v4_e_piano_finetuning_served_reference.md`
- `reports/user/2026-06-15_analisi_rollout_finetuning_v4_served_reference.md`
- `reports/user/2026-06-15_diagnosi_reset_imitativo_clock_prefisso_iniziale.md`
- `reports/user/2026-06-15_target_imitativo_observation_space.md`

## 1. Analisi rollout V4 e piano di fine-tuning

Il rollout deterministico del training imitativo V4 ha mostrato un netto
miglioramento rispetto al V3:

- rollout completato: `201` step, cioe' `2 s`;
- knee tracking RMSE ridotto da circa `0.0436 rad` a `0.00394 rad`;
- knee torque-error RMSE ridotto da circa `11.83 Nm` a `0.96 Nm`;
- knee `tau_input` RMS ridotto da circa `158.7 Nm` a `18.4 Nm`;
- potenza motore assoluta media ridotta da circa `1561 W` a `15 W`;
- ankle torque-error RMSE circa `0.267 Nm`;
- `tracking_score` medio circa `0.993`.

La conclusione tecnica e' che il controllore SEA segue correttamente la
reference servita; quindi l'errore principale non sta nel tracking fisico ma
nella traiettoria che la policy sceglie di servire.

La reward e' stata quindi orientata maggiormente sulla served imitation:

```yaml
reward:
  imitation_weight: 4.0
  served_imitation_weight: 8.0
  blend_served_imitation: 0.80
  blend_imitation: 0.20
  blend_imitation_tracking: 0
```

Sono stati anche impostati episodi da `5 s` e `gamma: 0.99`, cosi' una reward a
1 secondo di distanza pesa circa `0.99^100 ~= 0.366` invece di
`0.95^100 ~= 0.006`.

## 2. Fine-tuning V4 e rollout esteso

Il fine-tuning V4 e' terminato correttamente:

- iterazione finale logica: `57`;
- best checkpoint: iterazione logica `52`;
- best episode return medio: `236.27`;
- episode length medio al best: `497.5` step.

Il rollout deterministico dal best checkpoint ha completato l'episodio da
`5.0 s`:

- step: `501`;
- episode return: `243.71`;
- terminazione: `episode_time_limit`;
- nessuna caduta o divergenza;
- altezza minima pelvis: circa `0.952 m`;
- nessuna saturazione dell'ingresso motore SEA.

Metriche sull'intero episodio:

| Metrica | Ginocchio | Caviglia |
|---|---:|---:|
| RMSE served-target imitativo | `0.229 rad` | `0.182 rad` |
| Correlazione served-target | `0.644` | `0.436` |
| RMSE actual-served | `0.0029 rad` | `0.0069 rad` |
| RMSE errore coppia SEA | `1.69 Nm` | `0.24 Nm` |

Il confronto sui primi 2 secondi rispetto al V4 precedente mostra solo un
miglioramento marginale della forma served-target:

- knee served-target RMSE: `0.221 -> 0.217 rad`;
- ankle served-target RMSE: `0.136 -> 0.137 rad`;
- tracking fisico leggermente migliore;
- torque-error ginocchio peggiore.

Decisione: mantenere il checkpoint iterazione `52` come baseline stabile, ma non
continuare semplicemente lo stesso training. Serve cambiare informazione
osservata, obiettivo o struttura.

## 3. Chattering, clipping e diagnostica rollout

Il comando raw della policy mostra ancora alternanza ad alta frequenza:

- inversioni consecutive comando caviglia: circa `93%`;
- energia comando caviglia sopra `20 Hz`: circa `81%`;
- variazione media inter-step endpoint caviglia: `0.450 rad`;
- variazione media inter-step endpoint ginocchio: `0.249 rad`.

Il reference governor filtra pero' gran parte di questa alternanza, quindi il
chattering raw e' stato classificato come inefficienza e possibile causa
indiretta, non come problema fisico dominante.

E' stata inoltre chiarita la semantica del clipping:

- la rete produceva azioni grezze fino a `|a|max = 1.883`;
- `FlattenClipAction` gia' clippava correttamente prima del simulatore;
- `rollout_eval.py` e' stato aggiornato per salvare separatamente
  `raw_policy_action` e `applied_policy_action`;
- nel rollout analizzato `119 / 501` step avevano almeno un valore clippato;
- circa `11.9%` dei valori totali risultava clippato, soprattutto sulla
  caviglia.

Gli output fisici prima e dopo la modifica diagnostica sono risultati identici
byte per byte.

## 4. Diagnosi reset imitativo e prefisso iniziale

E' stato diagnosticato il grande swing iniziale del ginocchio protesico.

Il primo heel strike destro valido nel file GRF avviene a:

```text
13.16376284 s
```

Il vecchio inizio a `11.99 s` precede quindi il primo heel strike destro di
circa `1.174 s`. In quel tratto `GaitPhaseClock` retro-estrapola la fase dal
primo ciclo futuro. Il clock e' continuo, ma la fase non e' necessariamente
compatibile con il prefisso iniziale del dataset.

Con `imitation_initialize_to_target=true` a `11.99 s`, il ginocchio protesico
veniva forzato vicino a una configurazione da swing mentre il resto della posa
e il contatto indicavano ancora doppio appoggio:

- prescribed knee: `-0.1388 rad`;
- target imitativo knee: `-0.9335 rad`;
- differenza: `-0.7947 rad`, circa `-45.5 deg`;
- GRF sinistra online dopo target-init: circa `0 N`;
- norma reserve: `428.9 Nm`;
- muscle share: `0.067`.

A `12.99 s`, invece, la posa prescribed e il target periodico sono molto piu
compatibili:

- differenza knee target-prescribed: `0.0240 rad`;
- differenza ankle target-prescribed: `0.0384 rad`;
- contatto sinistro conservato;
- rollout diagnostico completato senza terminazione.

Configurazione scelta per il prossimo esperimento:

```yaml
simulation:
  episode_start_offset_s: 1.0
  imitation_initialize_to_target: false
  random_init: false
```

Questa non e' la soluzione robusta definitiva: e' un punto iniziale verificato
per il prossimo training.

## 5. Target imitativo nell'observation space

Per ridurre l'ambiguita' del problema imitativo, e' stato aggiunto all'actor
observation il target sano anti-fase, solo quando `reward_mode: imitation`.

Feature aggiunte:

```text
healthy_knee_angle_imitation_target
healthy_knee_angle_imitation_target_vel
healthy_ankle_angle_imitation_target
healthy_ankle_angle_imitation_target_vel
```

Queste feature sono lette da:

```text
q_target["pros_knee_angle"]
qdot_target["pros_knee_angle"]
q_target["pros_ankle_angle"]
qdot_target["pros_ankle_angle"]
```

Il flag interno `include_imitation_target_observation` viene agganciato in
`env_factory.make_cmc_env` a:

```python
reward_cfg.reward_mode == "imitation"
```

Quindi:

- `reward_mode: imitation` vede le quattro feature;
- `reward_mode: ex_novo` resta invariato;
- non serve un nuovo flag nello YAML;
- la reward non e' stata modificata.

Smoke test:

```text
ex_novo:
  obs_shape = 31
  n_actor = 31
  new_fields = []

imitation:
  obs_shape = 35
  n_actor = 35
  new_fields = 4/4 presenti
```

Conseguenza: il prossimo training imitativo deve partire da zero. I checkpoint
precedenti non sono compatibili perche' cambia la dimensione dell'observation
space.

## File modificati nel lavoro del 15/06

- `Trajectory Generator/baseline_MLP/training_cfg.v4_imitation.yaml`
  - episodio portato a `5.0 s`;
  - `gamma: 0.99`;
  - reward imitativa orientata sulla served reference;
  - `iterations` spostato nella sezione `simulation`;
  - `episode_start_offset_s: 1.0`;
  - `imitation_initialize_to_target: false`;
  - `random_init: false`.
- `Trajectory Generator/baseline_MLP/training_config.py`
  - `simulation.iterations` reso percorso canonico;
  - compatibilita mantenuta con `supervision.iterations`.
- `Trajectory Generator/baseline_MLP/rollout_eval.py`
  - propagazione offset/reset;
  - salvataggio `rollout_reset_diagnostics.json`;
  - trace policy piu esplicito;
  - separazione tra azione raw e azione applicata;
  - metriche di clipping.
- `Trajectory Generator/osim_trj_cmc_like.py`
  - diagnostica reset;
  - supporto `episode_start_offset_s`;
  - nuovo flag `include_imitation_target_observation`;
  - quattro feature actor imitativo target-only.
- `Trajectory Generator/baseline_MLP/env_factory.py`
  - gating automatico delle nuove feature da `reward_mode`.
- `reports/user/2026-06-15_target_imitativo_observation_space.md`
  - report dedicato alla modifica observation-space.

## Verifiche eseguite

- Rollout deterministico V4 da 2 secondi.
- Fine-tuning V4 completato fino all'iterazione logica `57`.
- Rollout deterministico da 5 secondi dal best checkpoint iterazione `52`.
- Confronto numerico V4 vs fine-tuning sui primi 2 secondi.
- Analisi di served-target, actual-served, torque-error SEA, clipping,
  alternanza dei comandi, contatto, penetrazione e reserve.
- Confronto byte-per-byte degli output fisici prima/dopo il logging diagnostico
  del clipping.
- Diagnostiche reset a `11.99 s` e `12.99 s`, con e senza target-init.
- Verifica del primo heel strike destro a `13.16376284 s`.
- Rilettura e verifica dello YAML finale.
- `python -m py_compile` su:
  - `Trajectory Generator/baseline_MLP/training_config.py`;
  - `Trajectory Generator/baseline_MLP/rollout_eval.py`;
  - `Trajectory Generator/osim_trj_cmc_like.py`;
  - `Trajectory Generator/baseline_MLP/env_factory.py`.
- Smoke env `ex_novo`/`imitation` per confermare observation space `31 -> 35`.
- `git diff --check` sui file modificati.

## TODO chiusi o superseduti il 15/06

- [x] Lanciare da zero il training lungo `baseline_mlp_imit_v4_c2_4hz`.
- [x] Eseguire rollout deterministico del checkpoint V4 migliore.
- [x] Confrontare V4 contro V3 su tracking SEA, torque-error, potenza, contatto
      e penetrazione.
- [x] Lanciare il fine-tuning breve dal best checkpoint V4.
- [x] Analizzare la forma della served reference su piu cicli nel rollout da
      `5 s`.
- [x] Verificare la semantica del clipping dell'azione durante rollout.
- [x] Diagnosticare la causa dello swing iniziale del ginocchio protesico.
- [x] Evitare, per il prossimo esperimento, il prefisso iniziale precedente al
      primo heel strike destro usando `episode_start_offset_s: 1.0`.
- [x] Aggiungere il target imitativo esplicito nell'observation actor per la
      modalita `imitation`.

## TODO aperti e propagati

### Prossimo training imitativo con obs target

- [ ] Lanciare un nuovo training imitativo da zero con lo schema actor a `35`
      feature, senza `--resume-from`.
- [ ] Usare come output suggerito
      `runs/baseline_mlp_imit_v4_c2_4hz_obs_target`.
- [ ] Dopo il training, eseguire rollout breve e confrontare `served-target`,
      `actual-target`, saturazione SEA, clipping/aggressivita' del comando e
      forma cinematica rispetto al target sano anti-fase.
- [ ] Confrontare il nuovo rollout da `12.99 s` con il precedente verificando
      swing iniziale, GRF, reserve e generalizzazione di fase.

### Forma della served reference

- [ ] Decidere la prossima modifica specifica per migliorare la forma globale
      della served reference, evitando di continuare lo stesso training
      invariato.
- [ ] Indagare la polarizzazione positiva della served reference della caviglia
      dopo il primo secondo.
- [ ] Valutare una penalita inter-step esplicita su `q_cmd(t)-q_cmd(t-1)` o
      sull'endpoint consecutivo solo se il chattering raw degrada served
      reference, SEA o stabilita numerica.
- [ ] Valutare la rimozione di `smoothness_weight` finche' `policy_knots` resta
      uguale a `1`.
- [ ] Valutare `lam: 0.95` in un'ablation separata se il credito temporale resta
      troppo locale.
- [ ] Eseguire separatamente un training reale asymmetric actor-critic da zero.

### Reset, fase e stati iniziali

- [ ] Implementare una selezione robusta degli stati iniziali che verifichi
      compatibilita fra posa, velocita, target imitativo e pattern di contatto.
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
- [ ] Monitorare nei training futuri `vf_explained_var`, entropy,
      predetto-vs-return e saturazione delle azioni.
- [ ] Valutare di esporre `vf_clip_param` e `vf_loss_coeff` come flag CLI.
- [ ] Validare auto-recovery durante un crash nativo Ray reale.
- [ ] Verificare su macOS arm64 cleanup, resume, RLModule e reward mode.
- [ ] Pulire launcher/log temporanei `run_imit_*.ps1` e `imit_*.log`.
- [ ] Pulire artefatti temporanei `results/_*`, `validation/_*` e smoke V4
      quando non piu necessari.
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
- [ ] Chiudere feature parity e gate finali scientifici, prestazionali e
      cross-platform.

## Comando consigliato per il prossimo training

```powershell
C:\Users\tomma\anaconda3\Scripts\conda.exe run --no-capture-output -n envCMC-rllib python "Trajectory Generator\baseline_MLP\train_ppo_mlp.py" --config "Trajectory Generator\baseline_MLP\training_cfg.v4_imitation.yaml" --output-dir "runs\baseline_mlp_imit_v4_c2_4hz_obs_target"
```

Nota: non usare `--resume-from` con il vecchio checkpoint, perche' il nuovo
observation space imitativo passa da `31` a `35` feature.
