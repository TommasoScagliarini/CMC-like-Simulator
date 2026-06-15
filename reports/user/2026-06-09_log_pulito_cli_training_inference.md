# Log pulito della CLI: cosmetica live + soppressione rumore (training e inference)

Data: 2026-06-09

## Problema

Dopo il fix del mojibake ([[2026-06-09_fix_cosmetica_cli_e_timeout_training]]) la
CLI era comunque inutilizzabile durante i run:

1. **Nessuna cosmetica appariva** lanciando il training: la barra di avanzamento
   non si vedeva affatto durante l'esecuzione.
2. Una volta resa visibile, la console stampava **un sacco di rumore** e la
   **barra veniva ristampata moltissime volte**, invece di una singola riga live
   in fondo. L'utente voleva un log pulito con una sola riga di cosmetica in
   fondo (ispirazione: la barra `tqdm` singola dello `SequentialTrainer` di skrl
   nel progetto SNN di riferimento).

## Cause individuate

1. **Buffering di `conda run`**: i comandi usavano `conda run` **senza**
   `--no-capture-output`. In quella modalità conda **bufferizza** stdout/stderr e
   li stampa solo a fine processo → la barra live (e ogni log) non compare durante
   il run.
2. **Inoltro dei log dei worker Ray al driver** (`log_to_driver=True`): ogni
   env-runner inoltrava al driver i propri messaggi (RuntimeWarning della Static
   Optimization "QP did not converge → bounded least-squares fallback",
   DeprecationWarning/FutureWarning, init muscoli, righe `(SingleAgentEnvRunner
   pid=...)`). Questi messaggi **interrompevano** la barra in place (ogni riga di
   rumore la spingeva e la rendeva su una nuova riga) → "barra ristampata molte
   volte" + "sacco di roba".
3. **Inference**: il `WARNING rl_module.py:463 -- DeprecationWarning` di RLlib al
   caricamento del checkpoint è emesso dal **logger** di Ray (non da `warnings`),
   e un `setLevel` per-logger veniva **resettato** dall'import di rllib in
   `_load_inference_stack()`.

## Soluzione

### `--no-capture-output` (commands.txt)
Aggiunto `--no-capture-output` a **tutti** i comandi `conda run` di
`commands.txt` (training, inference, watchdog, validazioni, pip) + nota critica in
cima. Così l'output viene **streammato in tempo reale**.

### Log pulito nel training (`train_ppo_mlp.py`)
- `ray.init(log_to_driver=False, logging_level=logging.ERROR)` di **default**: i
  log dei worker non raggiungono più la console del driver (gli errori veri
  restano: emergono come eccezioni sul driver e nei file di log Ray).
- Warning Python silenziati sul driver e nei worker: `warnings.filterwarnings(
  "ignore")` + `PYTHONWARNINGS=ignore` nel `runtime_env`, e `filterwarnings`
  dentro `_worker_setup` (alla fonte, in ogni worker).
- Nuovo flag **`--verbose-workers`** (default off) per ripristinare tutto
  l'output dei worker in debug.

### Log pulito nell'inference (`rollout_eval.py`)
- All'avvio di `run()` (default): `warnings.filterwarnings("ignore")` +
  **`logging.disable(logging.WARNING)`**. Quest'ultima è una soglia **globale**
  che sopravvive alla riconfigurazione del logging fatta dall'import di rllib (un
  `setLevel` per-logger no), quindi sopprime anche il `WARNING` di deprecazione
  dell'RLModule.
- Nuovo flag **`--verbose`** (default off), simmetrico a `--verbose-workers`.

### Risultato
La barra live di `progress_display.LiveProgress` era **già** una singola riga in
place su terminale TTY (ridisegnata ogni 0.5 s con `\r`); era solo il rumore a
spezzarla. Rimosso il rumore, su console interattiva resta una **singola riga
live in fondo**, con sopra i soli `[iter N/M] ...` (training) o l'esito
(inference). Su stream non-TTY (output a file) degrada a righe periodiche, come
`tqdm`/skrl.

## File modificati

```text
Trajectory Generator/baseline_MLP/commands.txt
  - --no-capture-output su tutti i conda run + nota in testa
Trajectory Generator/baseline_MLP/train_ppo_mlp.py
  - import logging/warnings
  - ray.init: log_to_driver=args.verbose_workers, logging_level=ERROR (default quiet)
  - runtime_env: PYTHONWARNINGS=ignore; warnings.filterwarnings nel driver
  - _worker_setup: warnings.filterwarnings nel worker
  - nuovo flag --verbose-workers
Trajectory Generator/baseline_MLP/rollout_eval.py
  - run(): warnings.filterwarnings + logging.disable(WARNING) (default quiet)
  - nuovo flag --verbose
```

## Test e verifiche eseguite

- `py_compile` PASS su `train_ppo_mlp.py` e `rollout_eval.py`.
- **Training breve** (2 worker, 2 iter, online_sensor + applied left, absolute +
  clock, con `--no-capture-output`): righe di rumore ridotte da decine a **1**
  (una nota interna benigna di Ray `core_worker.cc` al boot; spariti tutti i
  `(SingleAgentEnvRunner ...)`, QP, deprecation, muscle init). `Training complete:
  2/2 iterations, best return 27.01`, `ok:true`.
- **Inference breve** (sul checkpoint reale `baseline_mlp_hybrid_win/
  rl_module_best`): righe di rumore = **0**. `Rollout done: 50 steps, return
  29.12`, `ok:true`. Barra live pulita.
- Conferma cosmetica Unicode pulita (nessun mojibake) in entrambi.
- Dir temporanee di verifica rimosse; i run reali (`baseline_mlp_hybrid_win`,
  `baseline_mlp_hybrid_win_rollout_best`) non toccati.

## Nota / limite onesto

La "singola riga in place" è una proprietà del **terminale interattivo** (TTY),
come `tqdm`/skrl. Su `conda run --no-capture-output` in un terminale normale è
TTY → riga singola. In un contesto non-TTY (output a file, alcuni terminali IDE)
la barra degrada a righe periodiche pulite; in alternativa `--no-progress`
(training) per i soli `[iter ...]`.

## TODO

### Prossimo passo diretto (linea di lavoro)
- **Reward task-based che usa il gait clock** (auto-periodicità + coordinazione +
  obiettivi di task) — deferred in
  [[2026-06-09_durata_episode_phase_clock_ex_novo]].
- **Pre-training imitativo** (template `φ`-indicizzato dalle colonne `pros_*`):
  riduce anche i fallback QP della SO → iterazioni più veloci.
- **Lanciare il run completo (40 iter)** con la config corretta
  (`--iteration-timeout-s 3600`, `--run-timeout-s` coerente, `--no-capture-output`)
  e analizzare apprendimento/terminazioni/carico caviglia.

### TODO ereditati e propagati (ancora aperti)
- ~1.18 s/env-step (SO QP + fallback) è il collo di bottiglia; ridurre i fallback
  bounded least-squares accelererebbe direttamente le iterazioni.
- L'iteration-timeout aborta l'intero run invece della sola iterazione degenere:
  valutare sampling asincrono o meno worker.
- Ridurre divergenze `joint_divergence_pros_knee_angle`, saturazione knee SEA,
  fallback Static Optimization.
- Correggere i due bug HS **online** (timing protesico; flag `in_contact` sempre
  attivo) — prerequisito solo per varianti "evento discreto"/full task-based.
- Allineamento macOS: `setuptools<81` in `envCMC-rllib`; ricompilare il plugin
  onlineGRF `.dylib` su arm64.
- Migliorare il critic (explained variance negativa); tarare `oob_weight`;
  rivalidare il filtro 6 Hz su rollout lungo.
- Metriche di suitability ex-novo (velocità, stabilità, simmetria, effort,
  energia SEA, fattibilità GRF).
- Housekeeping repo (script temp `validation/_*.py`: `_env_timing.py`,
  `_gait_clock_absaction_smoke.py`), knowledge base letteratura, controllo SEA
  storici.
```
