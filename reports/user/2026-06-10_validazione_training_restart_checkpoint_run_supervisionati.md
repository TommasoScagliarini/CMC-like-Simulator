# Validazione del training: restart da checkpoint, cosmetica e log

**Data**: 2026-06-10
**Contesto**: Training ibrido PPO/MLP — validazione indipendente end-to-end prima del run lungo

---

## Problema

Le modifiche al training dei giorni 2026-06-09/06-10 (restart del child da
checkpoint, supervisor con skip accounting, timeout monotonic, `--resume-from`,
metadata checkpoint, progress bar e log pulito) erano state verificate ciascuna
nel proprio perimetro, ma mancava una validazione indipendente complessiva che:

1. avviasse un training reale e lo supervisionasse dal vivo;
2. verificasse cosmetica e log (barra, righe permanenti, rumore, mojibake);
3. accertasse che le modifiche non avessero introdotto bug, con un'analisi
   approfondita sufficiente a **decretare il training validato**.

## Esito

**TRAINING VALIDATO.** Review statica completa + **5 run reali supervisionati**
(tutti sulla config ibrida di produzione: `--grf-mode online_sensor`
`--online-grf-applied-side left`, profilo v2, 2 EnvRunner remoti, episodi 0.3 s,
batch 16). Tutti i percorsi esercitati funzionano; nessun bug introdotto;
caveat residui minori/cosmetici o upstream, nessuno bloccante.

| Run | Scenario | Esito |
|---|---|---|
| A | Fresh, 3 iter, happy path | exit 0, 62 s (~14 s/iter), artefatti completi, 0 processi residui |
| B | Resume + timeout 5 s → skip → restart → completed | skip `[4]`, 1 restart, exit 0, messaggi corretti |
| C | Timeout ripetuti con K=2 → abort | skip `[4,5]`, `aborted_consecutive_skips`, exit 124 |
| D | Resume nella stessa dir + 2 iterazioni reali | continuità learning provata |
| E | Fresh, 5 iter → primo episodio completo | return 9.577 finito, `checkpoint_best` esportato |

## Strategia

1. **Review statica**: lettura integrale di `train_ppo_mlp.py` (1593 righe) e
   `progress_display.py`; diff vs HEAD di `process_watchdog.py`,
   `rollout_eval.py`, `simulation_runner.py`, `osim_trj_cmc_like.py`; coerenza
   `--help` / `commands.txt` / `README.md` (flag nuovi presenti,
   `--run-timeout-s` assente dal training, presente solo nel watchdog del
   rollout, intenzionale).
2. **Matrice di run mirata**: ogni run esercita un ramo diverso del
   supervisor/child (happy path, skip singolo, abort, resume con training reale,
   export del best), con timeout indotti deterministici (5 s contro ~14 s/iter).
3. **Analisi artefatti** dopo ogni run: `summary.json`,
   `train_iterations.jsonl`, `checkpoint_{last,best}` + `_meta.json`,
   `supervisor_state.json`, `watchdog_state.json`, `faulthandler.log`,
   TensorBoard, processi residui.
4. **Root-cause nel sorgente RLlib installato** (Ray 2.55) per i comportamenti
   anomali osservati.

## Prove di correttezza chiave

- **Restore completo reale** (Run D): `weights_seq_no` prosegue 3→4→5,
  `num_env_steps_trained_lifetime` 270→360→450,
  `num_env_steps_sampled_lifetime` 48→64→80; jsonl `[1..5]` senza duplicati né
  buchi; meta aggiornato a `logical_iteration=5`. Lo stato RLlib continua
  davvero, non è un restart mascherato.
- **Self-guard del child** (Run B/C): timeout monotonic scatta dentro
  `algo.train`, scrive `summary.json` con `stop_reason="iteration_timeout"`,
  iterazione scaduta e checkpoint, dumpa gli stack (`CHILD SELF-TIMEOUT` con
  traceback verificato in `faulthandler.log`) ed esce 124; il supervisor
  termina l'albero Ray, logga `Iteration N SKIPPED ... (n/K consecutive)` e
  riparte da `checkpoint_last`. **Costo restart misurato: ~25-30 s** (2 worker;
  coerente con la stima 20-40 s).
- **Protezione checkpoint stale**: in dir senza checkpoint proprio il
  supervisor non inventa un resume (richiede meta **e** directory); accounting
  (`skipped_iterations`, `consecutive_skips`, `restart_count`,
  `supervisor_state.json`) sempre coerente in tutti gli scenari.
- **Best checkpoint** (Run E): al primo return finito (iter 4: 9.577, len 30 —
  primi episodi completi da 30 step) esporta `checkpoint_best` +
  `rl_module_best` + meta `logical_iteration=4`; riga console
  `[iter 4/5] return=9.577 len=30` e messaggio finale `best return 9.577`.
  L'assenza del best nei run senza episodi completi è corretta (return `n/a`),
  non un bug.
- **Processi**: dopo ogni run restano solo i 2 Python preesistenti del
  TensorBoard del 2026-06-09. Zero residui `raylet`/`gcs_server`.

## Cosmetica e log: corretti

- Barra live: in modalità non-TTY (output catturato/rediretto) degrada a righe
  periodiche ~30 s come documentato; Unicode reso correttamente, mai mojibake;
  su TTY l'aggiornamento in place era già stato verificato il 2026-06-09 e
  `progress_display.py` non è cambiato oggi.
- Righe permanenti `[iter N/M] return=... len=... steps=... time=...` corrette;
  righe `[supervisor]` chiare su skip/abort; messaggio finale + JSON summary
  (machine-readable, by design).
- Nessuno spam dei worker (QP fallback SO, deprecation silenziati).
- Rumore residuo per avvio child: 2 righe note e benigne (vedi caveat 1-2).

## Caveat noti (nessuno bloccante)

1. **Nuova riga di rumore per avvio/restart child**:
   `core_worker.cc E ... max_restarts > 0` — avviso Ray (issue #53727) causato
   da `restart_failed_env_runners=True` introdotto oggi. Benigno; silenziarlo
   richiederebbe rinunciare alla fault-tolerance sui guasti transitori.
2. La nota RLlib "new API stack" (1 riga, preesistente, benigna).
3. **`curr_kl_coeff` non viene ripristinato dal checkpoint** — gap upstream
   RLlib 2.55, verificato nel sorgente: `PPOLearner.build()` ricrea
   `curr_kl_coeffs_per_module` dal default di config e non lo include nello
   state del Learner. Effetto: dopo ogni restart il KL coeff riparte da 0.2 e
   si riadatta in ~3 iterazioni. Trascurabile con skip rari.
4. **Semantica `iterations_completed`** nel `summary.json`: con resume in una
   dir senza history può contare un'iterazione saltata come "completata"
   (Run B: 4 con la 4 skippata, perché `default=iteration_start-1` su history
   vuota). Nel flusso standard stessa-dir il jsonl la rende corretta. Solo
   display.
5. Cosmetici minori: `next logical iteration 5/4` quando start > target
   (logicamente corretto, lettura strana); ETA leggermente ottimista nel child
   post-restart (elapsed del solo processo corrente vs iterazioni totali).
6. Già documentato: con `--checkpoint-every > 1` uno skip prima del primo
   checkpoint riparte da pesi vergini mantenendo il contatore logico — per
   questo `--checkpoint-every 1` resta il default operativo.
7. Con `--no-child-self-timeout` il timeout di iterazione non è applicato da
   nessuno (il supervisor delega volutamente le fasi `algo.train` al child):
   opt-out esplicito, comportamento atteso.

## File modificati

Nessuna modifica al codice in questa sessione (sessione di sola validazione).
Le 4 directory di test (`Trajectory Generator/runs/_validate_train_20260610_*`)
sono state rimosse a fine validazione; i run reali sono intatti. Nessuna
modifica al plugin C++ SEA o alla semantica del comando SEA.

## Verifiche eseguite

- `py_compile` di tutti i moduli `baseline_MLP` nell'env `envCMC-rllib`: PASS.
- `--help`: flag nuovi presenti, `--run-timeout-s` assente: PASS.
- Run A/B/C/D/E come da matrice sopra, con ispezione completa degli artefatti
  e dell'output console catturato: PASS.
- `faulthandler.log`: header di fase nei run sani (nessun dump spurio), stack
  dump completo nel self-timeout: PASS.
- Verifica processi residui dopo ogni run e a fine sessione: PASS.
- Root-cause `curr_kl_coeff` nel sorgente RLlib installato: confermato gap
  upstream.

## TODO

- [ ] Lanciare il **run lungo** (§11c di `commands.txt`) con
      `--checkpoint-every 1` e `--max-consecutive-skips 5`, verificando il
      costo reale dei restart su molte iterazioni (propagato dal 2026-06-10).
- [ ] Collaudo fisico su **macOS arm64**: cleanup del process group POSIX e
      resume RLlib (propagato dal 2026-06-10).
- [ ] (Minore) Correggere la semantica di `iterations_completed` nel summary
      quando si riprende in una dir senza `train_iterations.jsonl`.
- [ ] (Minore, opzionale) Valutare il silenziamento della riga
      `core_worker.cc E ... max_restarts > 0` senza perdere la fault-tolerance.
