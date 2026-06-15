# Training ibrido notturno: stallo e timeout di iterazione non scattato

Data: 2026-06-09

## Problema

Il primo training con l'architettura GRF ibrida (prescribed sul lato sano +
online applicata sul piede protesico, profilo v2) è stato lanciato la sera del
2026-06-08 alle 22:30 su Windows con 12 env-runner, 80 iterazioni target,
episodi da 2.0 s, `run-timeout` 10 h. La mattina del 2026-06-09 il run non era
concluso e andava capito se fosse bloccato e quanto mancasse.

Comando lanciato:

```text
train_ppo_mlp.py --grf-mode online_sensor --online-grf-applied-side left
  --num-env-runners 12 --ray-num-cpus 13 --iterations 80
  --train-batch-size 4096 --minibatch-size 512 --num-epochs 10
  --episode-duration 2.0 --segment-duration 0.01
  --iteration-timeout-s 1800 --sample-timeout-s 1500 --run-timeout-s 36000
  --output-dir runs/baseline_mlp_hybrid_win
```

## Strategia di diagnosi

Ispezione read-only dello stato del run, senza interromperlo: processi attivi,
uso RAM/CPU, `watchdog_state.json`, `faulthandler.log`, timestamp di
checkpoint/TensorBoard, e infine `summary.json`.

## Risultati della diagnosi

### Risorse: non è un problema di memoria o CPU

- RAM totale `63.8 GB`, libera `30.9 GB` (52% usata), pagefile `241 MB` (nessuno
  swap significativo).
- CPU logici `24` per 12 worker.

Risorse abbondanti: lo stallo non è dovuto a saturazione.

### Stallo reale nel campionamento Ray

Lo stack in `faulthandler.log` (dump delle 08:22) mostrava il thread principale
bloccato in:

```text
synchronous_parallel_sample -> foreach_env_runner -> _fetch_result -> ray...wait
```

cioè il driver fermo in attesa che gli env-runner restituissero i rollout.
`watchdog_state.json` era fermo su `"phase": "algo.train iteration 43"`.
TensorBoard, watchdog e faulthandler senza aggiornamenti dopo le 08:22.

### Timeline

- Avvio 2026-06-08 22:30.
- Ultimo checkpoint 2026-06-08 23:49 = iterazione ~40 (checkpoint ogni 5).
- Iterazioni 41-43 estremamente lente (~2.8 h ciascuna vs ~6 min attese), con la
  43 completamente in stallo.
- Best checkpoint datato 22:45 (primi ~5-10 iter): il return è piccato presto e
  non è più migliorato.

### Terminazione

Il `summary.json` (scritto 2026-06-09 08:30:04) conferma la chiusura per
timeout totale:

```json
{ "ok": false, "timed_out": true, "timed_out_phase": "total_run",
  "error": "Training run exceeded the 36000 s total wall-clock timeout." }
```

Dopo la terminazione: **0 processi python attivi** (il watchdog ha killato
l'intero albero al tappo dei 10 h).

## Causa e bug individuato

- Il **run-timeout totale (10 h) ha funzionato** e ha liberato la macchina.
- Il **timeout di iterazione/sampling NON è scattato**: le iterazioni 41-43
  sono rimaste in stallo per ~8.5 h, mentre avrebbero dovuto essere uccise a
  ~1800 s. Il blocco è dentro `ray.wait` a livello C, non interrompibile dal
  timer del watchdog corrente.
- Causa probabile dello stallo: un **episodio degenere** su un worker (step
  OpenSim pateticamente lento o non terminante, coerente con le divergenze
  `joint_divergence_pros_knee_angle` e i fallback bounded least-squares della
  Static Optimization già osservati nel full-gait diagnostico). Con il sampling
  **sincrono**, l'iterazione attende il worker più lento: un solo episodio
  patologico blocca l'intera iterazione.

## Bilancio

- ~40 / 80 iterazioni utili completate; le restanti mai realmente avviate.
- Esito non riuscito (`ok: false`, `timed_out`).
- Salvati e utilizzabili: `rl_module_best`/`checkpoint_best` (22:45) e
  `rl_module_last`/`checkpoint_last` (iter ~40, 23:49).
- Gran parte della notte sprecata sulle iterazioni in stallo.

## File modificati

Nessuno: indagine puramente diagnostica e read-only sugli artefatti del run.

## Test e verifiche eseguite

- conteggio processi python (prima attivi: 12 worker + driver; dopo: 0);
- RAM/CPU/pagefile di sistema;
- lettura `watchdog_state.json` (fase "iteration 43");
- `faulthandler.log` (stack del thread principale in `ray.wait`);
- timestamp di `checkpoint_best/last` e dell'event file TensorBoard;
- lettura `summary.json` (timeout totale).

## TODO

- **Far sì che il timeout di iterazione/sampling uccida davvero** un'iterazione
  in stallo (oggi solo il run-timeout totale ha effetto): es. terminare gli
  env-runner via Job Object allo scadere di `sample-timeout-s`, o usare
  `synchronous_parallel_sample` con timeout per-attore.
- **Troncare prima gli episodi degeneri** che bloccano il sampling: aggiungere
  una guardia di wall-time per-step nell'ambiente (abort di uno step OpenSim
  troppo lungo) oltre alle guardie cinematiche.
- Ridurre alla radice le divergenze `joint_divergence_pros_knee_angle`, la
  saturazione knee SEA e i fallback Static Optimization (già propagati).
- Dopo i fix, rilanciare il training ibrido; valutare un sampling asincrono o un
  numero di worker più basso per ridurre la probabilità che un singolo episodio
  patologico gati l'intera iterazione.
- Eseguire un rollout deterministico di `rl_module_best` (2 s, record) per
  misurare cosa ha imparato l'ibrido in ~40 iter (carico caviglia, eventi gait,
  penetrazione, terminazioni `grf_penetration`).
