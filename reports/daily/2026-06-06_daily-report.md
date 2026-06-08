# Daily report - 2026-06-06

## Sintesi

Giornata dedicata a tre temi collegati del Trajectory Generator:

1. aggiunta di una penalty out-of-band sul riferimento generato dalla policy e
   di guard anti-divergenza articolari per-giunto;
2. correzione completa della semantica Gymnasium `terminated` / `truncated` sia
   nell'ambiente condiviso sia nel PPO_SNN custom;
3. definizione della direzione architetturale per passare da una reward
   fortemente imitativa alla generazione di traiettorie protesiche ex novo,
   prendendo ispirazione da GLiDE e identificando le GRF online come requisito
   fondamentale.

Report utente prodotti oggi:

```text
reports/user/2026-06-06_penalty_out_of_band_e_troncamento_per_giunto.md
reports/user/2026-06-06_correzione_semantica_terminated_truncated.md
reports/user/2026-06-06_dalla_reward_imitativa_alle_grf_online_glide.md
```

## 1. Penalty out-of-band e guard articolari

La precedente guard anti-divergenza usava un unico limite simmetrico molto
ampio per ginocchio e caviglia. È stata sostituita da bound asimmetrici
per-giunto sulla `q` simulata:

| Giunto | Banda reward soft | Guard anti-divergenza |
|---|---|---|
| knee | `[-1.35, 0.0]` rad | `[-2.0, 0.0]` rad |
| ankle | `[-0.5, 0.5]` rad | `[-0.9, 0.9]` rad |

Durante la verifica è stata scoperta e corretta la convenzione di segno del
ginocchio: nel modello AB06 la flessione è negativa e la IK misurata è circa
`[-1.06, -0.13]` rad.

In `baseline_MLP/reward_function.py` è stata aggiunta una penalty out-of-band
sul **riferimento comandato dalla policy**, non sulla `q` simulata:

```text
oob_loss = escursione quadratica media fuori dalla banda per-giunto
reward = reward_base - safety_term - oob_weight * oob_loss
```

La penalty è sottratta dopo il clip, resta osservabile in TensorBoard tramite
`oob_loss` / `oob_term` ed è attiva di default con `oob_weight = 2.0`.

## 2. Correzione `terminated` / `truncated`

L'ambiente condiviso classificava in modo invertito la fine artificiale
dell'episodio e gli stati unsafe. La semantica ora segue Gymnasium:

| Evento | `terminated` | `truncated` | `end_reason` |
|---|---:|---:|---|
| Caduta | `True` | `False` | `fall` |
| Divergenza articolare | `True` | `False` | `joint_divergence:<coord>` |
| Limite temporale | `False` | `True` | `episode_time_limit` |
| Fine dataset | `False` | `True` | `dataset_end` |
| Errore numerico catturato | `False` | `True` | `numerical_failure` |

Gli eventi unsafe ricevono la safety penalty e hanno priorità se coincidono con
la fine temporale. La baseline MLP RLlib usa direttamente i flag corretti
dell'ambiente.

Il PPO_SNN custom è stato corretto in modo coordinato:

- memorizza `next_values` per ogni transizione;
- effettua bootstrap sui troncamenti ma non sulle terminazioni;
- interrompe la propagazione GAE su entrambi i confini episodio;
- azzera gli stati ricorrenti su `terminated OR truncated`;
- registra gli `end_reason` reali nel training.

Questa modifica chiude il TODO storico **F5**.

## 3. Analisi della reward imitativa

È stato chiarito che la reward corrente e l'action space sono fortemente
imitativi:

- `reference_score` premia la vicinanza della protesi alla IK;
- `bio_score` premia la vicinanza del lato biologico alla IK;
- il default `action_mode="delta"` genera correzioni attorno alla IK.

`tracking_score` è invece interpretabile come termine di fattibilità: misura
quanto bene il SEA realizza la traiettoria comandata e può restare anche in una
reward non imitativa.

Sono state considerate:

- imitazione/pretraining seguito da RL task-based;
- curriculum con riduzione progressiva dei termini imitativi;
- RL task-based da zero;
- trajectory optimization seguita da distillazione/RL;
- policy goal-conditioned;
- traiettorie assolute o parametriche invece di delta rispetto alla IK.

## 4. Ispirazione GLiDE e architettura proposta

È stato analizzato il paper P12 GLiDE. La lezione trasferibile non è il modello
centroidale quadrupede in sé, ma la struttura:

```text
azione high-level funzionale
    -> livello deterministico vincolato/QP
    -> comando fisicamente realizzabile
    -> controllore low-level
```

GLiDE usa accelerazioni desiderate del corpo, un QP che produce GRF ammissibili
e una reward task-based semplice. La struttura del QP e del controllore impedisce
molti comportamenti indesiderati che una reward semplice, da sola, non
eliminerebbe.

Trasferimento proposto al progetto:

```text
policy high-level
    -> parametri funzionali della gait / obiettivi COM / GRF desiderate
    -> QP o generatore spline vincolato
    -> traiettorie assolute knee/ankle
    -> SEA + muscoli + reserve
    -> OpenSim full-order
```

Il livello vincolato dovrebbe imporre continuità, limiti di `q`, `qdot`,
`qddot`, jerk, banda passante SEA, coppia, potenza e saturazione.

## 5. Requisito GRF online

È stato verificato che il simulatore attuale carica le GRF sperimentali da file
`.mot` tramite `ExternalForce`. Le forze restano quindi legate alla gait
sperimentale anche quando la policy modifica la traiettoria protesica.

Questo impedisce di valutare pienamente una traiettoria ex novo: simmetria,
impulso, stabilità e qualità del contatto non emergono dalla dinamica prodotta
dalla policy.

Sono stati distinti due concetti:

- **GRF desiderate dal QP**: target fisicamente ammissibili usati dal planner o
  controllore;
- **GRF effettive online**: risultato dell'interazione piede-terreno nel modello
  completo.

La direzione proposta richiede una modalità simulatore alternativa con contatti
piede-terreno online, mantenendo inizialmente anche la modalità ExternalLoads
per regressione e confronto.

## File modificati oggi

```text
Trajectory Generator/osim_trj_cmc_like.py
Trajectory Generator/baseline_MLP/reward_function.py
Trajectory Generator/baseline_MLP/reward_overrides_example.json
Trajectory Generator/baseline_MLP/README.md
Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/ppo_snn.py
Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/cmc_ppo_train.py
Trajectory Generator/Prosthesis_SNN/tests/smoke_test.py
validation/rl_env_smoke_ab06_pi.py
validation/rl_env_rollout_ab06_pi.py
validation/_val_codex_diag.py
validation/_val_long_rollout.py
reports/user/2026-06-06_penalty_out_of_band_e_troncamento_per_giunto.md
reports/user/2026-06-06_correzione_semantica_terminated_truncated.md
reports/user/2026-06-06_dalla_reward_imitativa_alle_grf_online_glide.md
```

L'analisi GLiDE/GRF online non ha modificato il simulatore root: ha prodotto una
proposta architetturale e una roadmap.

## Verifiche eseguite

- `py_compile` sull'ambiente condiviso, reward baseline, PPO_SNN, training e
  diagnostica: **PASS**.
- `py_compile` nell'ambiente `envCMC-rllib`: **PASS**.
- PPO_SNN smoke test, inclusi i nuovi casi GAE per terminale/troncamento:
  **PASS**.
- Unit test della penalty out-of-band e round-trip `RewardConfig`: **PASS**.
- Verifica range IK reale e correzione segno knee: **PASS**.
- Episodio IK puro con bound corretti: nessuna penalità out-of-band e nessuna
  divergenza al primo step.
- Tiny training con nuova reward: **PASS**, metriche OOB presenti in TensorBoard.
- Audit call-site `_compute_gae` e vecchie aspettative sui flag: **PASS**.
- `git diff --check`: **PASS**, esclusi warning LF/CRLF.
- Lettura del paper P12 GLiDE e verifica del percorso GRF `ExternalLoads` nel
  simulatore.

Il rollout OpenSim completo successivo alla correzione
`terminated`/`truncated` non è stato completato: una prova diagnostica lunga ha
raggiunto il timeout.

## TODO chiusi oggi

- Guard anti-divergenza per-giunto con convenzione di segno verificata sui dati.
- Penalty out-of-band sul riferimento generato dalla policy.
- **F5**: semantica Gymnasium `terminated` / `truncated` corretta.
- Bootstrap GAE corretto sui troncamenti nel PPO_SNN.
- Diagnostica di fine episodio resa esplicita tramite `info["end_reason"]`.
- Direzione progettuale documentata per passare da reward imitativa a
  traiettorie ex novo.

## TODO aperti e propagati

### Priorità immediate: validazione RL

- Eseguire un rollout OpenSim completo e confermare:
  `episode_time_limit`/`dataset_end` come troncamenti e
  `fall`/`joint_divergence` come terminazioni.
- Eseguire training reali di almeno 50 iterazioni per verificare critic, curve
  di apprendimento e stabilità dopo la correzione F5.
- Tarare `oob_weight` usando `reward/oob_term` in TensorBoard.
- Rivalidare il filtro 6 Hz con una policy allenata e su rollout lungo.
- Verificare il lag causale del filtro 6 Hz rispetto alla IK a fase-zero ed
  eventualmente tarare `pros_ref_lpf_cutoff_hz` / `pros_ref_lpf_zeta`.

### Traiettorie ex novo / GLiDE-like

- Definire formalmente le metriche di suitability: velocità, stabilità,
  simmetria, effort biologico, energia SEA e qualità del passo.
- Progettare un action space assoluto o parametrico, indipendente dalla IK.
- Progettare un livello di trascrizione vincolato/QP tra obiettivi funzionali e
  traiettorie protesiche.
- Definire il curriculum da reward imitativa a reward task-based.
- Separare definitivamente tracking di fattibilità da imitazione della IK.
- Valutare un modello ridotto per pretraining GLiDE-like e fine-tuning OpenSim.

### GRF e contatto online

- Progettare una modalità `online_contact` alternativa agli `ExternalLoads`.
- Identificare e calibrare il modello di contatto piede-terreno OpenSim adatto
  ad AB06.
- Calcolare e registrare online GRF, stance/swing, heel strike e toe-off.
- Rendere coerenti inverse dynamics, Static Optimization e integrazione con le
  GRF online.
- Validare le GRF online sulla gait sperimentale prima di usarle per traiettorie
  ex novo.
- Mantenere la modalità ExternalLoads come baseline/regressione.

### Baseline MLP / reward / metriche

- Completare **F2 reward rebalancing**, ora orientandolo alla riduzione
  progressiva di `reference_score` e `bio_score`, non soltanto al tuning locale.
- Portare RMSE/NRMSE, Symmetry Angle e Trend Symmetry dal notebook esterno in un
  modulo riusabile.
- Integrare metriche funzionali basate su GRF/impulsi quando saranno disponibili
  online.
- Valutare l'incapsulamento della SNN come custom RLModule RLlib.

### Validazione generatore / SNN / PPO

- **F1 percorso SNN/skrl**: clippare la media in inference ai bound dell'azione.
- **F2 percorso SNN/skrl**: limitare la magnitudo azione e rivalidare la penalty
  di saturazione.
- **F3 percorso SNN/skrl**: rendere robusto e ordinato l'import
  torch/OpenSim/numpy su Windows.
- **F4**: chiamare `generator.reset()` al reset env nel rollout SNN.
- **F7**: validare `input_size == len(feature_names)` e architettura su
  resume/load.
- Validare il generatore su rollout lunghi per tracking, qualità della
  traiettoria e comportamento biologico.
- Definire e validare normalizzazione I/O, feature names, unità, scaling,
  output transform, action limits e metadata checkpoint.
- Aggiungere checkpoint periodici e gestione robusta di interruzioni/crash.
- **F6/T2**: irrobustire o rimuovere la cache su `data_ptr` in `actor_critic`.
- **T3**: validare reshape/BPTT prima di usare `num_envs > 1`.
- **T4**: passare lo spike gradient custom anche agli encoder rate/latency.
- **S1**: uniformare i reader `.sto` sul flag `inDegrees`.
- **S2/J2**: allineare il default `sea_stiffness` e valutare gain per-modello.
- **S3**: riusare `InverseDynamicsSolver` invece di ricrearlo ogni frame.

### Env RL / simulatore

- Risolvere target/bounds biologici della Static Optimization all'init AB06
  (`capable_share < 1`, warning QP).
- Confermare il layout Git di `osim_trj_cmc_like.py` in
  `Trajectory Generator/` e aggiornare riferimenti/packaging.
- Decidere se includere `LLM_SIMULATOR_OVERVIEW.md` in `CONTEXT.md` o nel flusso
  `start_day`.

### Knowledge base letteratura

- Recuperare il vero paper Wrapyfi: il file corrente in `paper/7` è il paper
  iCub.
- Approfondire, se utile, P17, P24, P14, P19 e P23.
- Collegare i finding della letteratura ai TODO della roadmap ex novo/GRF online.

### Propagati storici - controllo SEA

- Sweep `Kp_knee_motor` tra 3.9 e 18 mantenendo ankle best.
- Validare il coupling knee-ankle isolando la dinamica knee dal feedback ankle.
- Valutare un notch a 28 Hz sul feedback knee.
- Cleanup modelli sperimentali `slow_inner_pd_1405` e `pi_asym_knee1405`.
- Completare e documentare build/copia DLL plugin PI su Windows.
- Eseguire il secondo pass knee dello sweep locale.
- Consolidare il confronto finale tra configurazioni storiche PD, PI, cascade,
  retune PI, zeta07, pi-tuned, J_eff, Opzione D, slow inner e asym.
- Pulire gli artefatti sweep `_cascade_local_gain_sweep_20260517_233607` e
  `_cascade_local_gain_sweep_20260517_234151`.
- LPF qdot: test asimmetrico ankle, cutoff 30/35 Hz e run lunga 30+ s.
