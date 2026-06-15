# Skip iterazione via restart da checkpoint

**Data**: 2026-06-10
**Contesto**: Training ibrido PPO/MLP — robustezza del loop di training

---

## Problema

Il meccanismo di "skip in-place" per le iterazioni in stallo **non funziona** come
progettato. L'idea era:

1. Eseguire `algo.train()` in un thread di lavoro.
2. Allo scadere del timeout monotonic per-iterazione, fare `ray.kill` di tutti gli
   env-runner.
3. Aspettarsi che `algo.train()` ricevesse un `RayActorError`, il thread si
   sbloccasse e il training continuasse con l'iterazione successiva.

**Cosa succede in realtà**: la fault-tolerance di RLlib
(`restart_failed_env_runners=True`, default in Ray 2.55) **ricrea automaticamente**
i worker uccisi. `algo.train()` riprende a raccogliere il batch come se niente fosse
e **non si sblocca mai**. Dopo i 180s di grazia, il codice dichiarava
"unrecoverable" e stoppava il run — trasformando uno "skip" in un "abort".

### Verifica che ha esposto il problema

Run di verifica happy-path (2 worker, 2 iter, episodi 0.3s, GRF ibrido):

```
2026-06-10 12:13:28 ERROR actor_manager.py:973 -- Ray error [...], taking actor 1 out of service.
2026-06-10 12:13:28 ERROR actor_manager.py:973 -- Ray error [...], taking actor 2 out of service.
[... 11s di "No samples returned from remote workers" ...]
2026-06-10 12:13:38 WARNING actor_manager.py:986 -- Bringing previously unhealthy, now-healthy actor 1 back into service.
2026-06-10 12:13:38 WARNING actor_manager.py:986 -- Bringing previously unhealthy, now-healthy actor 2 back into service.
[... algo.train() riprende, non si sblocca mai ...]
Training STOPPED — iteration 1 timed out and its worker thread would not unwind within 180s
```

Il kill meccanico funziona (✅ i worker muoiono e tornano sani), ma `algo.train()`
non si interrompe (❌).

### Reframing importanti emersi

- Con N worker sani + `min_sample_timesteps_per_iteration = train_batch_size`, un
  singolo episodio divergente **non blocca** l'iterazione — gli altri worker
  completano il batch (solo un po' più lentamente). Lo stallo "vero" (tutti i
  worker incastrati) è il caso raro.
- Il killer reale del run notturno del 2026-06-08 era la **sospensione del PC**:
  il timeout avanzava durante lo sleep. Il timing `time.monotonic()` (che non
  avanza in sleep) risolve questo a prescindere dalla strategia di skip.

---

## Soluzione scelta: Opzione B — Restart da checkpoint

Invece di tentare di abortire `algo.train()` dall'interno (impossibile in modo
pulito), il **supervisor** (processo parent) gestisce lo skip rilanciando un
**processo child fresco** dall'ultimo checkpoint.

### Flusso

```
┌─────────────────────────────────────┐
│          SUPERVISOR (parent)        │
│  - lancia child --worker-process    │
│  - legge summary.json del child     │
│  - conta skip consecutivi           │
│  - se skip < K=5: rilancia da       │
│    checkpoint_last                  │
│  - se skip >= K=5: abort            │
│  - se completed/error: stop         │
└──────────────┬──────────────────────┘
               │ fork/exec
               ▼
┌─────────────────────────────────────┐
│           CHILD (worker)            │
│  - carica algo (o resume da ckpt)   │
│  - loop iterazioni con timeout      │
│    monotonic per-iterazione         │
│  - se timeout scade:                │
│    · scrive summary.json con        │
│      stop_reason="iteration_timeout"│
│    · esce con exit code specifico   │
│  - se tutte le iter completate:     │
│    · stop_reason="completed"        │
│  - se errore Python:                │
│    · stop_reason="error"            │
└─────────────────────────────────────┘
```

### Vantaggi

- **Processo fresco**: nessun wedge ereditato — stato RLlib, Ray, thread, memoria
  tutti puliti ad ogni restart.
- **Robusto**: funziona indipendentemente da dove `algo.train()` sia bloccata
  (sampling, learner, GPU sync — qualsiasi cosa).
- **Semplice**: il supervisor non ha bisogno di capire *cosa* è andato storto,
  solo che il child è uscito con `iteration_timeout`.
- **Mitigazione perdita**: con `--checkpoint-every 1` si perde al massimo 1
  iterazione per skip.

### Svantaggi

- **Costo di restart**: ~20-40s per ricaricare il modello, ricreare i worker e
  ripristinare il checkpoint. Accettabile dato che gli skip sono rari.
- **Logica di resume**: il child deve supportare un flag `--resume-from` per
  riprendere da un checkpoint arbitrario.

---

## Strategia di implementazione

### Modifiche al child (`run()`)

- Timeout per-iterazione basato su `time.monotonic()` (immune allo sleep del PC).
- Quando il timeout scade: scrive `summary.json` con `stop_reason="iteration_timeout"`,
  `iterations_completed=N`, `checkpoint_last=<path>`, ed esce con exit code dedicato.
- Supporto per `--resume-from <checkpoint_path>` per riprendere un training
  interrotto.

### Modifiche al supervisor (`run_supervised()`)

- Legge il `summary.json` del child dopo ogni uscita.
- Logica di dispatch basata su `stop_reason`:
  - `completed` → fine, propaga il summary.
  - `error` → fine, propaga l'errore.
  - `user_interrupt` → fine.
  - `iteration_timeout` → incrementa skip consecutivi, rilancia da checkpoint se
    sotto soglia.
  - `aborted_consecutive_skips` → il supervisor stesso scrive questo se K=5.
- Comunicazione `iteration_skipped` come evento nel log (non come stop_reason del
  run complessivo, ma come riga permanente nella console).

### Tassonomia stop_reason (già implementata nel logging)

| `stop_reason`                | Quando                          | Esempio console                                    |
|------------------------------|----------------------------------|----------------------------------------------------|
| `completed`                  | tutte le iter fatte              | `Training COMPLETE — 40/40 iters, best return 72.3`|
| `error`                      | eccezione Python reale           | `Training STOPPED — ERROR (RuntimeError): <msg>`   |
| `iteration_timeout`          | child: singola iter scaduta      | `Training STOPPED — iteration 15 timed out`        |
| `aborted_consecutive_skips`  | supervisor: K skip di fila       | `Training ABORTED — 5 consecutive skips`           |
| `startup_timeout`            | non è mai partito                | `Training STOPPED — startup phase exceeded <t>s`   |
| `user_interrupt`             | Ctrl-C                           | `Training INTERRUPTED by user`                     |

---

## Decisioni confermate

| Parametro                     | Valore                                              |
|-------------------------------|-----------------------------------------------------|
| Recupero su skip              | Processo child fresco (tutti i worker nuovi)         |
| Soglia K skip consecutivi     | **5**                                                |
| Run-timeout globale           | **Rimosso** (codice alleggerito)                     |
| Sorgente tempo timeout        | `time.monotonic()` (immune a sleep/sospensione PC)   |
| Checkpoint frequency minima   | `--checkpoint-every 1` (consigliato per minimizzare perdite) |

---

## File modificati (sessione precedente, parzialmente da rivedere)

```text
Trajectory Generator/baseline_MLP/train_ppo_mlp.py
```

- Tassonomia `stop_reason` + messaggi inglesi in console e `summary.json` ✅
- Fault-tolerance RLlib configurata (utile come infrastruttura, anche se lo skip
  in-place non funziona) ✅
- Run-timeout globale rimosso ✅
- Argparse aggiornato (`--max-consecutive-skips`, rimosso `--run-timeout-s`) ✅
- Logica di skip in-place (thread + kill) → **da sostituire** con restart da checkpoint ❌
- Resume da checkpoint → **da implementare**
- Logica supervisor retry/skip → **da implementare**

---

## Verifiche eseguite

- **Run happy-path (2 worker, 2 iter, GRF ibrido)**: ha esposto il problema —
  `algo.train()` non si sblocca dopo il kill dei worker. Exit con messaggio chiaro.
- **Nessun processo orfano**: i 2 processi python trovati erano il TensorBoard
  preesistente, non residui del run di verifica.
- **AST parse + py_compile**: il file compila correttamente dopo le modifiche.

---

## TODO

- [ ] Implementare `--resume-from` nel child per riprendere da checkpoint
- [ ] Implementare la logica di retry/skip nel supervisor
- [ ] Rimuovere il codice dello skip in-place (thread + kill worker) che non funziona
- [ ] Verificare con run end-to-end: happy path (completed), timeout singolo
      (restart + continue), K=5 skip consecutivi (abort)
- [ ] Aggiornare `commands.txt` e `README.md` con i nuovi flag e il comportamento
