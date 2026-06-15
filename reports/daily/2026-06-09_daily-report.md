# Daily report - 2026-06-09

## Sintesi

Giornata su due grandi fronti convergenti:

1. **Robustezza e diagnosi del training ibrido** (mattina/notte precedente):
   diagnosi del run notturno fallito, difese anti-stallo a più livelli, fix
   ambiente TensorBoard.
2. **Generatore ex-novo + usabilità CLI** (pomeriggio): analisi di design sulla
   durata degli episode e sul clock di fase, implementazione del **gait clock del
   lato sano** e dell'**output assoluto della rete** (ex-novo), e una serie di fix
   alla CLI (mojibake, buffering, log pulito) fino a training e inference con log
   pulito e singola barra live.

Report utente prodotti oggi:

```text
reports/user/2026-06-09_training_ibrido_notturno_stallo_timeout_iterazione.md
reports/user/2026-06-09_anti_stallo_timeout_iterazione_guardia_wall_time_cli_progress.md
reports/user/2026-06-09_tensorboard_pkg_resources_setuptools_fix.md
reports/user/2026-06-09_durata_episode_phase_clock_ex_novo.md
reports/user/2026-06-09_gait_phase_clock_e_output_assoluto_rete.md
reports/user/2026-06-09_fix_cosmetica_cli_e_timeout_training.md
reports/user/2026-06-09_log_pulito_cli_training_inference.md
```

## 1. Diagnosi del training ibrido notturno

Il primo training ibrido (12 worker, 80 iter target, episodi 2 s, lanciato il
2026-06-08 22:30) non è arrivato a fine: chiuso dal `run-timeout` totale (10 h).
Le iterazioni 41-43 erano in stallo (~8.5 h) con il driver fermo in
`synchronous_parallel_sample -> ray.wait` (sampling sincrono in attesa del worker
più lento). Causa probabile: un **episodio degenere** su un worker (segmento
OpenSim patologicamente lento, coerente con `joint_divergence_pros_knee_angle` e i
fallback bounded least-squares della Static Optimization). Salvati e utilizzabili
`rl_module_best`/`rl_module_last`.

## 2. Difese anti-stallo + CLI con progress (TODO notturni chiusi)

Difesa a più livelli, robusta a prescindere dal meccanismo esatto dello stallo:

- **Radice — guardia wall-time per env-step**: `SegmentWallClockTimeout` in
  `simulation_runner.py`; `CMCEnvConfig.step_wall_timeout_s` (default 30 s); lo
  `step()` tronca con grazia (`end_reason="step_wall_timeout"`, `truncated=True`)
  anche con `fail_fast=True`. Un episodio degenere consuma al più il budget e poi
  tronca.
- **In-process — self-timeout del child**: `_TrainingMonitor` (thread daemon) che
  allo scadere del timeout di fase dumpa gli stack, scrive `summary.json` di
  timeout e fa `os._exit(124)` — scatta anche se il main thread è bloccato in
  `ray.wait` a livello C. Verificato che il phase-timeout del supervisor esterno
  funziona (probe → exit 124); la conclusione notturna "il timer non può
  interrompere ray.wait" era imprecisa.
- **Ridondante — supervisor esterno** via `watchdog_state.json` invariato.
- **CLI progress**: nuovo modulo `progress_display.py` (solo stdlib): barra live
  in place su TTY (%/iter/elapsed/ETA/spinner), righe permanenti `[iter N/M] ...`,
  metriche complete in `train_iterations.jsonl`; barra step/ETA anche nel rollout.
- **Fix race heartbeat Windows** (`os.replace` WinError 5 nei rollout lunghi):
  retry 6×/50 ms + scrittura heartbeat best-effort (non solleva mai).

Eseguito il **rollout deterministico** di `rl_module_best` notturno (2 s):
`return 186.85`, `reward_mean 0.93`, nessuna caduta (`pelvis_ty_min 0.905`),
`.sto` salvati in `runs/baseline_mlp_hybrid_win_rollout_best`.

## 3. Fix ambiente: TensorBoard

TensorBoard non partiva (`ModuleNotFoundError: pkg_resources`): l'env aveva
**setuptools 82** (che ha rimosso `pkg_resources`). Fix mirato:
`pip install "setuptools<81"` → 80.10.2. Boot reale verificato (HTTP 200 + tag del
run notturno). Documentato in `commands.txt`; da applicare anche su macOS.

## 4. Analisi di design: durata episode, phase clock, ex-novo

Partendo da "durata episode variabile HS-to-HS", l'analisi è evoluta:

- Il reference è **time-indexed**, non phase-indexed → HS-to-HS è stop-condition
  (basso rischio) o re-fasatura (alto costo).
- Il rilevamento HS **online** del lato protesico è **ancora rotto** (timing
  sfasato, `in_contact` sempre attivo); ciò che era validato l'8/6 è la GRF online
  su cinematica oracolo, non il detector con policy viva.
- In regime **ex-novo** la sfasatura `~T/2` tra i due lati **non è garantita**
  (lato sano = clock fisso, lato protesico = libero): la coordinazione va
  assicurata (phase-clock) o selezionata (reward).
- Convergenza: **episodio lungo fisso** + **clock di fase `φ`** del lato sano
  (pacemaker deterministico) + reset bookkeeping (non stato fisico) +
  offset tunabile. Il clock è infrastruttura comune a imitazione e task-based;
  pre-training imitativo come opzione (curriculum). Finestra dati reale: il `.mot`
  AB06 va da 11.99 a 155.045 s (~143 s), quindi episodi lunghi/`random_init` sono
  disponibili.

## 5. Implementazione: gait clock del lato sano + output assoluto

(`Trajectory Generator/`, nessuna modifica a SEA/plugin C++.)

- **`GaitPhaseClock`** in `osim_trj_cmc_like.py`: fase `φ∈[0,1)` dagli heel-strike
  del lato **destro PRESCRIBED** (deterministici), riusando
  `output._cycles_from_vertical_grf` su `ctx.grf_vertical_force_columns["right"]`.
  Feature osservazione **`gait_phase`/`_sin`/`_cos`** (sempre presenti) →
  **obs dim 58→61**. `gait_clock_phase_offset` tunabile. Indipendente dal detector
  online rotto.
- **Output assoluto**: `action_mode` default **delta→absolute** (la rete emette
  una **traiettoria assoluta** ex-novo, non più una deviazione dalla IK).
  `absolute_bounds_rad` default knee[-1.5,0]/ankle[-0.7,0.7]. Flag `--action-mode`
  in train/rollout (default absolute). **Checkpoint delta vecchi incompatibili**.
- Verificato (smoke dedicato): clock su dati reali (124 cicli, periodo 1.13 s,
  φ=0 all'HS); mapping assoluto corretto e indipendente dall'IK; path delta
  intatto; stack reward/wrapper OK.

## 6. Fix CLI: mojibake, buffering, timeout, log pulito

- **Mojibake**: `progress_display.py` ora forza UTF-8 sulla console Windows
  (`SetConsoleOutputCP(65001)`) e verifica la codepage reale; fallback ASCII se
  non UTF-8. Mai mojibake.
- **"Nessuna cosmetica"**: causa = `conda run` **senza `--no-capture-output`**
  (bufferizza tutto fino a fine processo). Aggiunto `--no-capture-output` a tutti
  i comandi di `commands.txt` + nota.
- **"Non arriva alla fine"**: il run dell'utente è stato abortito a iter 12 dal
  `--iteration-timeout-s 1800` (da `summary.json`). Misura diretta: **~1.18 s/
  env-step** (SO QP) è il collo di bottiglia, reset 0.01 s; con absolute le pose
  ex-novo fanno fallire più spesso la QP → ~13-15 min/iter. Config incoerente
  (run-timeout 10 h < 60-80 iter × ~14 min). Corretto in `commands.txt`:
  `iteration-timeout 3600`, `sample-timeout 3000`, `iterations 40`, con nota sulle
  regole timeout↔iterazioni.
- **Log pulito**: `ray.init(log_to_driver=False, logging_level=ERROR)` di default
  (i log dei worker non spammano più il driver, era la causa della barra
  ristampata molte volte) + warning Python silenziati (driver, worker,
  `PYTHONWARNINGS`); flag `--verbose-workers`. Inference: `filterwarnings` +
  `logging.disable(WARNING)` (robusto all'import di rllib) + flag `--verbose`.
- Verificato: training breve rumore decine→1 (nota Ray benigna), inference
  rumore→0, entrambi `ok:true`, barra Unicode pulita e singola.

## File principali modificati oggi

```text
simulation_runner.py
Trajectory Generator/osim_trj_cmc_like.py
Trajectory Generator/baseline_MLP/train_ppo_mlp.py
Trajectory Generator/baseline_MLP/rollout_eval.py
Trajectory Generator/baseline_MLP/process_watchdog.py
Trajectory Generator/baseline_MLP/progress_display.py
Trajectory Generator/baseline_MLP/commands.txt
Trajectory Generator/baseline_MLP/README.md
```

## File principali aggiunti oggi

```text
Trajectory Generator/baseline_MLP/progress_display.py
validation/_gait_clock_absaction_smoke.py   (smoke clock + output assoluto, temp)
validation/_env_timing.py                    (timing step/reset single-process, temp)
reports/user/2026-06-09_*.md (7 report)
```

## Verifiche eseguite (sintesi)

- `py_compile` di tutti i file Python modificati (sistema + `envCMC-rllib`).
- Anti-stallo: self-guard child in subprocess (hard-exit 124), guardia wall-time
  reale (truncated, no raise), tiny training + rollout end-to-end, parsing flag.
- TensorBoard: import + boot reale HTTP 200 con i tag del run notturno.
- Gait clock + output assoluto: smoke A-D tutti PASS (clock sintetico+reale,
  mapping assoluto, regressione delta, stack reward).
- CLI: unit test ASCII non-tty; UTF-8 console Windows; training breve completo
  (`ok:true`, return 27.01) con log pulito; inference breve (`ok:true`, return
  29.12) con 0 righe di rumore.
- Dir temporanee di verifica rimosse; run reali intatti.

## TODO chiusi oggi

- Far sì che il timeout di iterazione/sampling uccida davvero un'iterazione in
  stallo (self-guard in-process del child).
- Troncare gli episodi degeneri che bloccano il sampling (guardia wall-time per
  env-step).
- Migliorare la CLI con progress bar (%/iter), ETA, tempo trascorso.
- Rollout deterministico di `rl_module_best` del run notturno.
- Race heartbeat Windows (`os.replace` WinError 5).
- TensorBoard non parte per `pkg_resources` (setuptools<81).
- Implementare il **gait clock del lato sano** (φ in osservazione, offset tunabile).
- Cambiare l'**output della rete da delta ad assoluto** (ex-novo) in env/train/rollout.
- **Mojibake CLI** (UTF-8 console Windows + fallback ASCII).
- **Cosmetica non visibile** (`--no-capture-output` su tutti i comandi).
- **Log pulito** training+inference (no log worker, warning silenziati).
- Verificare la barra in run parallelo non disturbata dai log worker (era TODO
  aperto in §2): risolto con `log_to_driver=False`.

## TODO aperti e propagati

### Generatore ex-novo (linea di lavoro principale)
- **Costruire la reward task-based che usa il gait clock**: termini di
  auto-periodicità (confronto col ciclo precedente alla stessa `φ`),
  coordinazione/anti-fase col lato sano, obiettivi di task (stabilità, fattibilità
  GRF, effort/energia SEA, ROM); definire la **gerarchia degli obiettivi** (cosa è
  vincolo duro, cosa driver primario, cosa contorno), confermare periodicità+
  coordinazione sempre attive, e i pesi/blend.
- **Approfondire la strategia di pre-training imitativo** (aggancio al template
  `φ`-indicizzato dalle colonne `pros_*` del `.mot`; criterio di passaggio a
  task-based). Riduce anche i fallback QP della SO.
- **Tarare `gait_clock_phase_offset`** (HS sano vs toe-off vs mid-stance) e
  valutare **episodi lunghi** / `random_init` con il clock attivo.
- Metriche di suitability (velocità, stabilità, simmetria, effort, energia SEA,
  fattibilità GRF); action space assoluto/parametrico; livello QP vincolato;
  curriculum imitativa→task-based; separare fattibilità da imitazione; modulo
  metriche RMSE/NRMSE/Symmetry; metriche funzionali GRF/impulsi.

### Training ibrido (esecuzione)
- **Lanciare il run completo (40 iter)** con la config corretta
  (`--no-capture-output`, `--iteration-timeout-s 3600`, `--run-timeout-s` coerente)
  e analizzare apprendimento, terminazioni `grf_penetration`, carico caviglia,
  stabilità; tarare le soglie di penetrazione con la policy attiva.
- **Analizzare gli `.sto`** del rollout già eseguito
  (`runs/baseline_mlp_hybrid_win_rollout_best/sim_outputs/`): carico caviglia/
  ginocchio SEA, eventi gait online, penetrazione, reserve bacino; eventualmente
  riprodurre con `visualize.py`.
- Prima del run macOS: **ricompilare/riconfermare il plugin onlineGRF `.dylib`**
  su arm64; applicare `setuptools<81` nell'env `envCMC-rllib` macOS.

### Throughput / dinamica
- **~1.18 s/env-step** (SO QP + fallback) è il collo di bottiglia: ridurre i
  fallback bounded least-squares accelererebbe direttamente le iterazioni.
- L'iteration-timeout aborta l'intero run invece della sola iterazione degenere:
  valutare sampling asincrono o meno worker.
- Ridurre alla radice divergenze `joint_divergence_pros_knee_angle`, saturazione
  knee SEA; analizzare/ridurre reserve biologiche ed equilibrium failures.
- Correggere il timing heel-strike **online** del lato sinistro (protesico) e il
  flag onlineGRF `in_contact` (sempre attivo) — prerequisito solo per varianti
  "evento discreto"/full task-based, non per il clock implementato.
- Migliorare il critic (explained variance negativa); tarare `oob_weight` con
  `reward/oob_term`; rivalidare il filtro 6 Hz su rollout lungo.

### Contatto online / COP
- Tarare il rocker (stiffness, altezza ground sulla forward, curvatura) con la
  policy che controlla la caviglia, per un rollover COP liscio senza spike.
- Valutare contatto online anche sul lato sano quando le traiettorie ex-novo
  faranno divergere troppo il prescribed dal moto reale.
- NOTA: l'obiettivo "reserve pure-online ≤ 1.5x" è superato dalla scelta ibrida
  (limite strutturale); non promuovere la modalità `online` pura a `validated`.

### Repository / housekeeping
- Pulire gli script/log temporanei `results/_*.py`, `results/_*.log`,
  `validation/_hybrid_env_smoke.py`, `validation/_env_timing.py`,
  `validation/_gait_clock_absaction_smoke.py`.
- Valutare se togliere da git i build artifact `build_online_grf/*.obj`.

### Validazione generatore / SNN / PPO (propagati)
- F1-F4, F7 percorso SNN/skrl; cache `data_ptr` (F6/T2); reshape/BPTT (T3); spike
  gradient encoder (T4); reader `.sto` `inDegrees` (S1); default `sea_stiffness`
  per-modello (S2/J2); riuso `InverseDynamicsSolver` (S3); SNN come RLModule.

### Knowledge base letteratura
- Recuperare il vero paper Wrapyfi (paper/7 è iCub); approfondire P17/P24/P14/
  P19/P23; collegare la letteratura alla roadmap ex-novo/GRF online.

### Propagati storici: controllo SEA
- Sweep `Kp_knee_motor` 3.9-18; coupling knee-ankle; notch 28 Hz knee; cleanup
  modelli sperimentali; build/copia DLL plugin PI Windows; secondo pass knee;
  confronto finale configurazioni storiche; cleanup artefatti sweep; LPF qdot
  asimmetrico ankle 30/35 Hz su run lunga.
