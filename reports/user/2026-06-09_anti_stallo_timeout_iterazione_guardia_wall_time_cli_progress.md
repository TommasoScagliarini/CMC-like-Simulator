# Anti-stallo: self-timeout del child, guardia wall-time per env-step e CLI con progress

Data: 2026-06-09

## Problema

Due problemi, emersi dal training ibrido notturno fallito
([[2026-06-09_training_ibrido_notturno_stallo_timeout_iterazione]]):

1. **Il timeout di iterazione/sampling non uccideva davvero un'iterazione in
   stallo.** Nel run del 2026-06-08 (12 worker, episodi 2 s, `run-timeout` 10 h)
   le iterazioni 41-43 sono rimaste bloccate per ~8.5 h e solo il `run-timeout`
   totale ha liberato la macchina. Lo stack mostrava il driver fermo in
   `synchronous_parallel_sample -> ... -> ray.wait` (sampling sincrono che
   aspetta il worker più lento). Causa probabile: un **episodio degenere** su un
   worker (segmento OpenSim patologicamente lento, coerente con le divergenze
   `joint_divergence_pros_knee_angle` e i fallback bounded least-squares della
   Static Optimization).

2. **L'interfaccia CLI di training/inference lasciava il terminale vuoto.**
   Durante i lunghi `algo.train()` (minuti) non si vedeva nulla: niente
   avanzamento, ETA o tempo trascorso.

## Diagnosi e strategia

Prima di scrivere il fix ho **testato il phase-timeout del supervisor esterno in
isolamento** con il probe integrato (`train_ppo_mlp.py --watchdog-probe
--watchdog-probe-timeout-s 1.5`): il supervisor ha ucciso il child a ~1.5 s con
exit `124`. Quindi **il meccanismo del supervisor esterno funziona**, e la
conclusione del report notturno ("il timer del watchdog non può interrompere un
`ray.wait` a livello C") è imprecisa: il supervisor è un processo separato e
*può* terminare l'albero di un child bloccato in C.

Non riuscendo a riprodurre l'esatta race notturna (probabile interazione
`sample_timeout_s` 1500 s vs `iteration_timeout_s` 1800 s con i retry interni di
RLlib, oppure una race sul file watchdog su Windows), ho scelto una **difesa a
più livelli** invece di toccare un solo punto, così da essere robusti a
prescindere dal meccanismo esatto:

- **alla radice**: troncare l'episodio degenere dentro l'ambiente, così nessun
  worker può gattare il sampling per più di pochi secondi;
- **in-process**: un self-guard nel child che lo hard-exita allo scadere del
  timeout di fase, senza dipendere dal supervisor esterno;
- **ridondante**: il supervisor esterno via file resta com'è.

## Soluzione

### 1. Guardia wall-time per env-step (TODO 2 — fix alla radice)

- `simulation_runner.py`: nuova eccezione `SegmentWallClockTimeout`;
  `step_until(t_stop, record, wall_timeout_s=0.0)` registra un deadline
  monotonic e lo controlla **tra i sub-step** del loop di integrazione. Se un
  segmento sfora il budget wall-clock, solleva l'eccezione.
- `osim_trj_cmc_like.py`: nuovo campo `CMCEnvConfig.step_wall_timeout_s`
  (default 30 s). Lo `step()` passa il budget a `step_until` e cattura
  `SegmentWallClockTimeout` **prima** del generico `except`, troncando con
  grazia (`end_reason="step_wall_timeout"`, `truncated=True`) **anche con
  `fail_fast=True`** — perché è una guardia deliberata, non un fault numerico.
  Il reason finisce in `episode_end/step_wall_timeout` via il callback esistente.

Effetto: un episodio degenere consuma al più `step_wall_timeout_s` (es. 60 s nel
comando overnight) e poi tronca, invece di bloccare l'iterazione per ore.

### 2. Self-timeout in-process del child (TODO 1 — il timeout che scatta davvero)

- `train_ppo_mlp.py`: nuova classe `_TrainingMonitor` con un thread daemon che:
  - tiene il deadline monotonic della fase corrente; allo scadere **dumpa tutti
    gli stack** (`faulthandler.log`), scrive un `summary.json` di timeout e fa
    `os._exit(124)`. Gira in un thread separato, quindi **scatta anche se il main
    thread è bloccato in un `ray.wait` a livello C** (che rilascia la GIL).
  - continua a scrivere il file `watchdog_state.json` a ogni cambio fase, così il
    **supervisor esterno resta attivo come livello ridondante**.
- Tutte le chiamate `_write_watchdog_state(...)` in `run()` sono passate a
  `monitor.set_phase(...)` (che scrive comunque il file). Dopo l'uscita dal loop
  si forza una fase `finalizing` per evitare che il self-guard hard-exiti mentre
  si scrive il summary con un deadline d'iterazione già scaduto.

### 3. CLI con progress bar, ETA e tempo trascorso (richiesta utente)

- Nuovo modulo `progress_display.py` (solo stdlib): `format_hms`, `render_bar` e
  la classe `LiveProgress` (barra in-place thread-safe). Refresh in place su TTY
  (carriage return), update periodici a riga nuova se rediretto a file, **fallback
  ASCII** quando lo stream non può codificare i glifi Unicode (console Windows
  cp1252). `log()` stampa righe permanenti **sopra** la barra senza corromperla.
- Training: la barra mostra **percentuale, contatore iterazioni, elapsed, ETA**,
  fase corrente e spinner, aggiornata anche durante i lunghi `algo.train()`. Le
  metriche complete per-iterazione vanno in `<output_dir>/train_iterations.jsonl`
  (terminale pulito); a fine iterazione una riga permanente
  `[iter N/total] return=… len=… steps=… time=…` e un messaggio finale.
- Rollout (`rollout_eval.py`): barra **step/ETA** per-step e riga finale con
  `end_reason`.
- Nuovi flag (default ON): `--step-wall-timeout-s`, `--child-self-timeout/--no-…`,
  `--progress/--no-progress` (su training; `--step-wall-timeout-s` e `--progress`
  anche su rollout).

### 4. Fix race heartbeat su Windows (emerso in verifica del TODO #5)

Lanciando il rollout reale (200 step) è emerso un bug **pre-esistente**: la
scrittura dell'heartbeat (`os.replace` del file `watchdog_state.json`) falliva con
`PermissionError [WinError 5]` quando il supervisore esterno leggeva il file nello
stesso istante. Il rollout scrive l'heartbeat **a ogni step**, quindi colpiva la
race (il training, che scrive per-fase, la mascherava). Fix: `os.replace` ora
**ritenta** (6×, 50 ms) e la scrittura dell'heartbeat è **best-effort, non solleva
mai** — un aggiornamento avviso mancato è innocuo, un crash nel loop degli step no.
Applicato sia in `process_watchdog.write_heartbeat` (nuovo helper
`atomic_write_json`) sia nella copia `_write_watchdog_state` del training.

## File modificati

```text
simulation_runner.py                                   (SegmentWallClockTimeout + wall_timeout_s in step_until)
Trajectory Generator/osim_trj_cmc_like.py              (campo step_wall_timeout_s + troncamento dedicato in step)
Trajectory Generator/baseline_MLP/train_ppo_mlp.py     (_TrainingMonitor: self-timeout + progress + jsonl + flag; _write_watchdog_state resiliente)
Trajectory Generator/baseline_MLP/rollout_eval.py      (progress bar per-step + flag step-wall-timeout/progress)
Trajectory Generator/baseline_MLP/process_watchdog.py  (atomic_write_json + write_heartbeat best-effort con retry su Windows)
Trajectory Generator/baseline_MLP/commands.txt         (flag nel comando overnight + note robustezza)
Trajectory Generator/baseline_MLP/README.md            (sezione "Robustezza anti-stallo e progress")
```

## File aggiunti

```text
Trajectory Generator/baseline_MLP/progress_display.py  (modulo progress, solo stdlib, cross-platform)
```

## Test e verifiche eseguite

- `py_compile` di tutti i file modificati, con il Python di sistema **e** con
  l'env `envCMC-rllib`: PASS.
- Unit test `progress_display`: `format_hms`/`render_bar`/clamp; `LiveProgress`
  non-TTY (update a riga nuova) e TTY simulato (carriage return + clear + log
  permanente + finish); fallback ASCII su stream senza encoding: PASS.
- **Self-guard del child** in subprocess: `set_phase` con budget 0.6 s e main
  thread bloccato in `time.sleep(10)` (chiamata C che rilascia la GIL, come
  `ray.wait`) → hard-exit `124`, `summary.json` di timeout scritto, stack dumpati
  in `faulthandler.log` (sia il thread monitor sia il main bloccato): PASS.
- **Plumbing `step_wall_timeout_s`** via `env_factory` + uno step reale: campo
  presente in `CMCEnvConfig`, propagato, step normale senza troncamento spurio
  (`end_reason=None`): PASS.
- **Guardia wall-time reale**: env con budget 1 ms e `fail_fast=True` →
  `truncated=True, end_reason="step_wall_timeout"` senza raise, con il tempo di
  simulazione riportato nell'info di failure: PASS.
- **Tiny training end-to-end** (`envCMC-rllib`, 2 iterazioni, in-process):
  barra live con %/iter/elapsed/fase, righe `[iter k/2] return=… time=…`,
  messaggio finale `Training complete: 2/2 …`, `train_iterations.jsonl` (2 righe,
  tutte le chiavi), `summary.json` con `step_wall_timeout_s` registrato: PASS.
- **Rollout end-to-end** sul checkpoint tiny: barra `rollout`, riga finale
  `Rollout done: 3 steps, return …, truncated=True (episode_time_limit)`: PASS.
- Parsing dei nuovi flag e negazioni (`--no-progress`, `--no-child-self-timeout`)
  con i default attesi (progress=on, child_self_timeout=on,
  step_wall_timeout_s=30): PASS.
- **Rollout reale TODO #5** di `runs/baseline_mlp_hybrid_win/rl_module_best`
  (online_sensor + applied left, episodio 2 s, `--record-outputs`): 201 step,
  `episode_return=186.85`, `reward_mean=0.93`, `terminated=False`,
  `truncated=True (episode_time_limit)`, `pelvis_ty_min=0.905` (nessuna caduta),
  watchdog `ok=True/timeout_reason=None/returncode=0`, `.sto` scritti per
  `visualize.py`. Barra live verificata: %, contatore step, elapsed ed **ETA
  decrescente** (3:59 → 0:14) presenti: PASS. Questo run ha fatto emergere e
  validato il fix della race heartbeat (§4).
- `git diff --check` (whitespace) pulito; artefatti di test temporanei rimossi.

## TODO chiusi oggi

- Far sì che il timeout di iterazione/sampling uccida davvero un'iterazione in
  stallo (self-guard in-process del child, indipendente dal supervisor esterno).
- Troncare prima gli episodi degeneri che bloccano il sampling (guardia wall-time
  per env-step nell'ambiente).
- Migliorare la CLI di training/inference con progress bar (%/iterazioni), ETA e
  tempo trascorso.
- Eseguire il rollout deterministico di `rl_module_best` del run notturno (~40
  iter): fatto, return 186.85 su 2 s, nessuna caduta, output `.sto` registrati in
  `runs/baseline_mlp_hybrid_win_rollout_best`.
- Sistemare la race heartbeat su Windows (`os.replace` WinError 5) che faceva
  crashare i rollout lunghi.

## TODO aperti e propagati

### Training ibrido (priorità immediata)
- **Rilanciare il training ibrido** ora che è a prova di stallo (Windows 12
  worker / macOS 5 worker; flag `--step-wall-timeout-s 60` già nei comandi).
  Tarare le soglie di penetrazione (penalty/terminazione) con la policy attiva.
- Prima del run macOS: **ricompilare/riconfermare il plugin onlineGRF `.dylib`**
  su arm64.
- **Analizzare gli output `.sto` del rollout** già eseguito
  (`runs/baseline_mlp_hybrid_win_rollout_best/sim_outputs/`): carico caviglia/
  ginocchio SEA, eventi gait online, penetrazione del piede protesico, reserve del
  bacino, ed eventualmente riprodurre con `visualize.py`.
- Verificare in un run reale parallelo (`--num-env-runners N`) che la barra non
  sia troppo disturbata dai log dei worker RLlib (`log_to_driver=True`); se serve,
  valutare di abbassare la verbosità dei worker.

### Training MLP / dinamica (dal full-gait diagnostico)
- Ridurre alla radice le divergenze `joint_divergence_pros_knee_angle`, la
  saturazione del knee SEA e i fallback bounded least-squares della Static
  Optimization (sono la causa probabile degli episodi degeneri ora troncati).
- Analizzare/ridurre reserve biologiche ed equilibrium failures.
- Correggere/ricalibrare il timing heel-strike online del lato sinistro nei
  rollout della policy; correggere il flag onlineGRF `in_contact` (sempre attivo).
- Migliorare il critic (explained variance negativa).
- Tarare `oob_weight` con `reward/oob_term`; rivalidare il filtro 6 Hz su rollout
  lungo con policy allenata.

### Contatto online / COP
- Tarare il rocker (stiffness, altezza ground sulla forward, curvatura) con la
  policy che controlla la caviglia, per un rollover COP liscio senza spike.
- Valutare contatto online anche sul lato sano quando le traiettorie ex-novo
  faranno divergere troppo il prescribed dal moto reale.
- NOTA: l'obiettivo "reserve pure-online ≤ 1.5x" è superato dalla scelta ibrida
  (limite strutturale); non promuovere la modalità `online` pura a `validated`.

### Repository / housekeeping
- Pulire gli script/log temporanei `results/_*.py`, `results/_*.log`,
  `validation/_hybrid_env_smoke.py`.
- Valutare se togliere da git i build artifact `build_online_grf/*.obj`.

### Traiettorie ex-novo / GLiDE-like
- Metriche di suitability (velocità, stabilità, simmetria, effort, energia SEA);
  action space assoluto/parametrico; livello QP vincolato; curriculum imitativa→
  task-based; separare fattibilità da imitazione; modulo metriche
  RMSE/NRMSE/Symmetry; metriche funzionali basate su GRF/impulsi.

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
