# Daily report - 2026-06-04

## Sintesi

Giornata su tre linee:

1. **Knowledge base della letteratura** RL/protesi in `literature/` (28 documenti
   estratti come "second brain" per LLM), poi estesa con i **percorsi Windows**
   dei PDF accanto a quelli macOS;
2. **Trajectory generator RL eseguibile end-to-end** (PPO/SNN): chiuso il
   contratto env↔SNN, aggiunte CLI di training e inference, checkpoint best/last;
3. **Validazione intensa e indipendente** dello stesso generatore (creato con
   Codex), con report dedicato di findings e una ritrattazione per trasparenza.

Report utente prodotti oggi:

```text
reports/user/2026-06-04_knowledge_base_letteratura_rl_protesi.md
reports/user/2026-06-04_traj_generator_rl_training_inference.md
reports/user/2026-06-04_validazione_intensa_traj_generator_rl.md
```

## 1. Knowledge base letteratura (`literature/`)

Creata una KB **reference-only** e disaccoppiata dal progetto: `INDEX.md`
(router 28 voci), `TOPICS.md` (64 topic), `README_FOR_LLM.md` (protocollo +
ricetta ri-estrazione), `notes/Pxx_*.md` (una nota per documento). Estratti
P01–P25 (paper), D01–D02 (docs), S02 (sintesi Gemini); S01 linkata. Note in
inglese, frontmatter machine-readable, claim ancorati a pagina,
`extraction_confidence` dichiarata. Estrazione con PyMuPDF isolato in
`/tmp/pdflib` (zero impatto sugli env conda).

Anomalie corpus: **P25 mislabeled** (contiene il paper iCub, non Wrapyfi);
**P17** è la tesi Berkenkamp (205pp, solo abstract+TOC); **nessun paper su SNN**
nel corpus.

Estensione di oggi (percorsi Windows): aggiunto il campo `pdf_win:` a tutte le
**28 note** accanto a `pdf:` (macOS), e dichiarata la doppia radice in `INDEX.md`
/ `README_FOR_LLM.md`:

```text
macOS   : /Users/tommy/Desktop/report opensim+rl/
Windows : C:\Users\tomma\Desktop\report opensim+rl\
```

Verificato che la radice Windows esiste e la struttura cartelle combacia. Anche
`Trajectory Generator/literature/` (collegamento aggiunto dall'utente) allineato.

## 2. Trajectory generator RL: training/inference/checkpoint

Implementato un percorso PPO/SNN funzionante dentro `Trajectory Generator/`,
**senza toccare il plugin C++ SEA**. Chiuso il **mismatch di contratto** segnalato
dal code review del 2026-06-01 (T1/J1): l'azione SNN è `env_action` reshaped a
`(policy_knots, n_prosthetic_coords) = [3,2]`, con feature privilegiate reali
salvate per nome nel checkpoint. Inference **strict**: fallisce se action shape o
feature names non coincidono tra checkpoint ed env.

Output del training: `agent.pt`/`reference.pt` (alias ultimo), `last_*`, `best_*`
(per `episode_return`), `summary.json`. `best_reference.pt` è il preferito per
inference (solo SNN esportata + metadata). CLI documentate in
`Trajectory Generator/commands.txt`.

## 3. Validazione intensa del generatore

**Verdetto**: la macchina è solida — gira end-to-end, è **deterministica
bit-a-bit**, i checkpoint fanno round-trip con contract-check stretti, e un
**rollout da 3.5 s (351 step) non diverge** (pelvis dritto, tutto finito). I
problemi sono di **consistenza policy↔simulatore** e robustezza, non crash.

Verifiche: deps + smoke OK; attributi `ctx`/`cfg` dell'env tutti presenti;
training tiny + **path di update multi-minibatch** OK; determinismo confermato;
checkpoint export→reload→predict + contract OK; rollout 2.0 s e 3.5 s **senza
divergenza** (`pelvis_ty`min 0.95, finito).

**Findings** (dettaglio in
`reports/user/2026-06-04_validazione_intensa_traj_generator_rl.md`):

- **F1 [Media]** inference ritorna la media **non clippata** → supera i bound
  azione (probe 3.28 vs 1.0; reale `action_abs_max` 1.23–1.34 > 1.0).
- **F2 [Media]** SEA satura **~17–21%** con policy reale (`u_abs_max=1.0`): il
  filtro 6 Hz **da solo non basta** quando l'azione è ampia/erratica (il "0%"
  del 2026-06-01 era una sinusoide 0.10).
- **F3 [Bassa/Media]** import torch/OpenSim/numpy fragile su Windows
  (OMP #15 / `fbgemm.dll`); funziona solo via il workaround in
  `prosthesis_snn/__init__`, sensibile all'ordine di import.
- **F4 [Bassa]** `generator.reset()` mancante nel rollout `--reset-on-done`
  (membrana SNN sfora tra episodi).
- **F5 [Bassa]** `terminated`/`truncated` invertiti vs Gymnasium; GAE fa
  bootstrap solo su `terminated` (attenuante: `phase` osservata).
- **F6 [Bassa/latente]** cache forward su `data_ptr` (≡ T2 del 2026-06-01):
  unsound in linea di principio, **non riprodotto** un risultato errato.
- **F7 [Bassa]** nessuna validazione `input_size==len(feature_names)` né
  architettura su resume/load.
- **Nota design**: reward dominata da `reference_score` (peso 0.55 = "resta
  vicino all'IK"); una policy non allenata segna già ~0.90 → i numeri non
  dimostrano apprendimento.

**Ritrattazione (trasparenza)**: il sospetto che l'update PPO valutasse male la
policy ricorrente è stato **smentito** da un re-test fedele (per-sample corretto,
`max|diff|=6e-8`). Il ramo reshape-come-sequenza non è esercitato nel wiring
single-env. Non è un bug; resta solo da validare per `num_envs>1` (≡ T3).

## File creati / modificati oggi

```text
literature/notes/*.md (28)            # aggiunto campo pdf_win
literature/INDEX.md, README_FOR_LLM.md# doppia radice macOS/Windows
Trajectory Generator/literature/...   # collegamento + stesse modifiche pdf_win
Trajectory Generator/osim_trj_cmc_like.py                         (training-side)
Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/cmc_ppo_train.py
Trajectory Generator/Prosthesis_SNN/prosthesis_snn/cmc_policy_rollout.py
Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/ppo_snn.py
Trajectory Generator/Prosthesis_SNN/prosthesis_snn/{__init__,config,generator,reference_provider}.py
Trajectory Generator/Prosthesis_SNN/training/cmc_ppo_smoke.py, tests/smoke_test.py
Trajectory Generator/Prosthesis_SNN/README.md, commands.txt
validation/_val_codex_diag.py, _val_actor_critic_probe.py,
validation/_val_update_path_probe.py, _val_long_rollout.py        (probe validazione)
```

(Artefatti `runs/_val_*` cancellabili. La validazione non ha modificato il codice
del generatore né il plugin.)

## Verifiche eseguite

```text
SNN smoke_test.py                                  -> smoke tests passed
deps envCMC-like (torch/snntorch/skrl/gym/opensim) -> import OK
training tiny + update-path (mini_batches=2, ep=4) -> ok, reward finiti, 8/8 terminated
determinismo (2 run stesso seed)                   -> identici
checkpoint export->reload->predict + contract      -> ok, action_shape [3,2], 58 obs
inference breve (5 step)                            -> reward ~0.90, in range
rollout 200 step / 2.0 s                            -> no divergenza, sat 21%, act 1.23
rollout 351 step / 3.5 s                            -> no divergenza, sat 17%, act 1.34
semantica terminazioni (orizzonte)                 -> terminated=True, truncated=False
re-test update path (P2)                            -> per-sample corretto (6e-8)
literature: radice Windows + struttura             -> esiste e combacia
```

## TODO chiusi oggi

- Mismatch di contratto azione/output/feature (T1/J1 del 2026-06-01): chiuso —
  `env_action` `[3,2]`, feature per nome, validato.
- Script di training e inference **production** (oltre agli smoke): creati
  (`cmc_ppo_train`, `cmc_policy_rollout`); girano end-to-end.
- Export automatico del checkpoint inference-only a fine training: fatto
  (best/last/alias).
- "Validare il filtro 6 Hz con una policy reale": **risposto** — il filtro da
  solo è insufficiente (saturazione 17–21%). Genera F2.
- KB letteratura: aggiunti i percorsi Windows (`pdf_win`) e la doppia radice.

## TODO aperti e propagati

### Validazione generatore (nuovi, dal report 2026-06-04)
- **F1**: clippare la media in inference ai bound dell'azione (consistenza
  train/deploy).
- **F2**: limitare la magnitudo azione + penalità saturazione efficace
  (`reward_saturation_weight` oggi 0.1 troppo basso); rivalidare il filtro 6 Hz
  con una **policy allenata** su rollout lungo.
- **F3**: rendere robusto/ordinato l'import torch/OpenSim/numpy su Windows
  (shim obbligatorio per primo + doc).
- **F4**: `generator.reset()` al reset env nel rollout (`--reset-on-done`).
- **F5**: allineare `terminated`/`truncated` alla convenzione Gymnasium +
  bootstrap GAE sul troncamento.
- **F7**: validare `input_size==len(feature_names)` e l'architettura su
  resume/load.
- **Reward**: ribilanciare i pesi così che "riprodurre l'IK" non sia già
  quasi-ottimo; alzare il peso saturazione.

### Trajectory Generator / SNN / PPO (dal 2026-06-01, ancora aperti)
- Validare il generatore su rollout lunghi per **tracking protesico, qualità
  traiettoria e comportamento biologico** (oggi validata solo la stabilità di
  integrazione, non la qualità).
- Decidere la **tecnica di training** (RL puro / hybrid SL+RL / imitazione→RL /
  staged) e definire l'**observation space** (stato completo vs sensori
  deployable vs ibrido).
- Definire/validare normalizzazione I/O, feature names, unità, scaling, output
  transform, action limits, metadata checkpoint.
- Tarare/validare la reward su rollout lunghi (tracking, smoothness, effort,
  coerenza q/qdot/qddot, **penalità saturazione**, soglie truncation).
- Checkpoint periodici per training lunghi e gestione interruzioni/crash.
- **F6/T2**: irrobustire o rimuovere la cache su `data_ptr` in `actor_critic`.
- **T3**: validare il reshape/BPTT prima di passare a `num_envs>1`.
- **T4**: passare lo spike_grad custom anche a rate/latency encoder.
- **S1**: uniformare i reader `.sto` sul flag `inDegrees`.
- **S2/J2**: allineare `sea_stiffness` default e valutare gain per-modello.
- **S3**: riusare `InverseDynamicsSolver` invece di ricrearlo ogni frame.

### Env RL / simulatore (dal 2026-06-01, ancora aperti)
- Verificare il **lag causale** del filtro 6 Hz sul tracking dell'IK
  (IK a fase-zero, riferimento online no): lead/anticipo o termine reward.
- Eventuale taratura `pros_ref_lpf_cutoff_hz`/`pros_ref_lpf_zeta`.
- Target/bounds biologici della static optimization all'init AB06
  (`capable_share < 1`, warning QP).
- Layout Git: confermare lo spostamento di `osim_trj_cmc_like.py` in
  `Trajectory Generator/` e aggiornare riferimenti/packaging.
- Aggiungere `build/` al `.gitignore` o decidere come gestirla.
- Decidere se includere `LLM_SIMULATOR_OVERVIEW.md` in `CONTEXT.md` o nel flusso
  `start_day`.

### Knowledge base letteratura (dal report 2026-06-04, reference-only)
- Recuperare il vero paper **Wrapyfi** (il file in `paper/7` è il paper iCub).
- Approfondire i parziali se utile: **P17** (tesi Berkenkamp, Lyapunov/RoA),
  **P24** (survey RLVR §3), P14/P19/P23.
- (Solo su richiesta) sintesi trasversale per tema; (solo su richiesta) collegare
  i finding della letteratura ai TODO del progetto — per ora **non** fatto.

### Propagati storici (linea controllo SEA, dal 2026-05-31 e precedenti)
- Sweep `Kp_knee_motor` su valori intermedi tra 3.9 e 18 mantenendo ankle best.
- Validare diagnosi di coupling knee-ankle isolando la dinamica knee dal
  feedback ankle.
- Notch a 28 Hz sul feedback knee (lato controllore), opzione separata dal
  filtro env.
- Cleanup modelli sperimentali (`slow_inner_pd_1405`, `pi_asym_knee1405`).
- Windows: build/copia DLL plugin PI completa e documentata.
- Secondo pass knee dello sweep locale.
- Confronto consolidato finale tra configurazioni storiche (PD, PI, cascade,
  retune PI, zeta07, pi-tuned, J_eff, Opzione D, slow inner, asym).
- Pulizia artefatti sweep `_cascade_local_gain_sweep_20260517_233607` e
  `_cascade_local_gain_sweep_20260517_234151`.
- LPF qdot: test asimmetrico solo ankle, cutoff 30/35 Hz e run lunga 30+ s.
