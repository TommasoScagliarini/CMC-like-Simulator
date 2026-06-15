# Restart da checkpoint del training PPO: implementazione e validazione

**Data**: 2026-06-10

## Problema

Lo skip in-place di una `algo.train()` bloccata non era robusto. Uccidere gli
EnvRunner non interrompe la chiamata: la fault tolerance di RLlib ricrea i
worker e il thread di training continua a raccogliere il batch. Dopo il timeout
di grazia il run veniva quindi abortito, invece di saltare l'iterazione.

## Soluzione

Implementata l'Opzione B: **restart del processo child da checkpoint**.

- `algo.train()` torna a essere una chiamata bloccante lineare nel main thread.
- Il monitor child applica il timeout per iterazione con `time.monotonic()`.
- Su timeout il child scrive `summary.json` con
  `stop_reason="iteration_timeout"`, iterazione scaduta, iterazione successiva e
  ultimo checkpoint, poi termina con exit code 124.
- Il supervisor termina l'intero process tree Ray, registra lo skip e rilancia
  un interprete fresco.
- Il nuovo child ripristina lo stato completo RLlib tramite
  `algo.restore_from_path(checkpoint_last)` e continua dalla successiva
  iterazione logica.
- Dopo `--max-consecutive-skips` timeout consecutivi il supervisor termina con
  `stop_reason="aborted_consecutive_skips"`.
- Il contatore consecutivo viene azzerato se il child ha completato almeno una
  iterazione sana prima del timeout.

È stato aggiunto il resume manuale:

```text
--resume-from <checkpoint RLlib completo>
```

I checkpoint salvano ora anche:

```text
checkpoint_last_meta.json
checkpoint_best_meta.json
```

Il metadata impedisce al supervisor di usare accidentalmente un vecchio
checkpoint presente in una directory riutilizzata. Per minimizzare la perdita
di apprendimento dopo uno skip, i comandi operativi usano
`--checkpoint-every 1`.

## Strategia

1. Rimossa interamente la logica skip in-place:
   thread daemon per `algo.train()`, kill degli EnvRunner e tentativo di heal.
2. Separata la responsabilità:
   il child rileva e descrive il timeout; il supervisor decide restart o abort.
3. Mantenuto il timeout di iterazione monotonic nel child, così sleep/sospensione
   del computer non genera un falso timeout.
4. Mantenuta la fault tolerance RLlib per guasti transitori di un singolo
   EnvRunner, senza usarla più come meccanismo di skip.
5. Resa persistente la diagnostica in `summary.json` e
   `supervisor_state.json`.

## File modificati

```text
Trajectory Generator/baseline_MLP/train_ppo_mlp.py
Trajectory Generator/baseline_MLP/README.md
Trajectory Generator/baseline_MLP/commands.txt
```

Nessuna modifica al plugin C++ SEA o alla semantica del comando SEA.

## Verifiche eseguite

### Statiche

- `py_compile` con Python host: PASS.
- `py_compile` nell'ambiente `envCMC-rllib`: PASS.
- parsing CLI `--help`: `--resume-from`, timeout e soglia skip presenti;
  `--run-timeout-s` rimosso dal training.
- `git diff --check`: PASS, esclusi i normali warning LF/CRLF.

### Training fresco con EnvRunner remoto

Output:

```text
Trajectory Generator/runs/_restart_checkpoint_verify
```

Configurazione ridotta: 1 EnvRunner remoto, 1 iterazione, batch 2.

Esito:

```text
stop_reason = completed
iterations_run = 1
checkpoint_last = creato
checkpoint_last_meta logical_iteration = 1
```

### Resume reale da checkpoint

Ripreso lo stesso run con `--resume-from checkpoint_last` e target 2.

Esito:

```text
restored_training_iteration = 1
iteration_start = 2
iterations_run = 2
checkpoint_last_meta logical_iteration = 2
```

La history contiene esattamente le iterazioni 1 e 2, senza duplicati.

### Timeout singolo, restart e completamento

Ripreso il checkpoint a iterazione 2 con timeout intenzionale di 1 secondo e
target logico 3.

Esito:

```text
iteration 3 -> iteration_timeout
supervisor -> termina process tree e registra skip
restart da checkpoint_last (RLlib iteration 2)
iteration_start = 4
stop_reason = completed
skipped_iterations = [3]
restart_count = 1
```

Questo verifica end-to-end il percorso scelto: timeout, uscita child, cleanup,
restart da checkpoint e continuazione.

### Abort per skip consecutivi

Output:

```text
Trajectory Generator/runs/_restart_abort_verify
```

La soglia è stata ridotta a K=2 per verificare rapidamente la stessa logica
usata dal default produttivo K=5.

Esito:

```text
skipped_iterations = [1, 2]
consecutive_skips = 2
restart_count = 2
stop_reason = aborted_consecutive_skips
```

### Processi residui

Dopo le verifiche non risultano nuovi processi `raylet`/`gcs` o processi Python
del training. Restano soltanto due processi Python preesistenti avviati il
2026-06-09, coerenti con il TensorBoard già attivo.

## TODO

- [ ] Eseguire il prossimo training lungo con `--checkpoint-every 1` e
      `--max-consecutive-skips 5`, verificando costo reale dei restart e
      stabilità su molte iterazioni.
- [ ] Verificare fisicamente su macOS arm64 il cleanup del process group e il
      resume RLlib; il percorso POSIX è implementato ma questa sessione è stata
      validata su Windows.
