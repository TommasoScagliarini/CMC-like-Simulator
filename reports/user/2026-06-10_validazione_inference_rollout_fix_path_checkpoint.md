# Validazione dell'inference (rollout_eval): run supervisionati e fix path checkpoint

**Data**: 2026-06-10
**Contesto**: Pipeline baseline MLP/RLlib — validazione end-to-end dell'inference,
speculare a quella del training dello stesso giorno
([[2026-06-10_validazione_training_restart_checkpoint_run_supervisionati]])

---

## Problema

Le modifiche all'inference del 2026-06-09 (progress bar live con step/ETA,
`--action-mode`, log pulito via `logging.disable`, risoluzione `--output-dir`
verso `Trajectory Generator/runs`, `step_wall_timeout_s`, fix race heartbeat
Windows) non erano mai state validate in modo indipendente e complessivo:
serviva avviare rollout reali supervisionati, verificare cosmetica e log, ed
escludere bug introdotti, con profondità sufficiente a **decretare l'inference
validata**.

## Esito

**INFERENCE VALIDATA**, con **2 bug reali trovati e corretti** durante la
validazione (entrambi introdotti/esposti dalle modifiche del 2026-06-09):

1. **`--checkpoint` (e `--resume-from` del training) con path relativo non
   seguivano la convenzione di `--output-dir`**: venivano risolti rispetto alla
   CWD (root del simulatore) invece che a `Trajectory Generator`. Dopo lo
   spostamento di `runs/` (2026-06-09), **tutti i comandi rollout documentati in
   `commands.txt` (§5, §8, §11b) che passano `--checkpoint "runs\..."` erano
   rotti** (`FileNotFoundError`); il rollout del 2026-06-09 funzionò solo perché
   usava il path completo. Fix: risoluzione "smart" (prima CWD, poi
   `Trajectory Generator`), retrocompatibile con entrambe le forme.
2. **`NameError` latente nel `finally` del rollout**: l'epilogo della progress
   bar (aggiunto il 2026-06-09) legge `info` nel `finally`; se `env.reset()`
   stesso solleva, `info` non era ancora definita → `NameError` che sporcava il
   traceback originale e saltava `env.close()`. Fix: pre-bind `info = {}` prima
   del `try`.

Matrice dei run (config ibrida di produzione: `online_sensor` + GRF applicata a
sinistra, profilo v2):

| Run | Scenario | Esito |
|---|---|---|
| Preflight | `process_watchdog.py --self-test` | stallo rilevato in 1.77 s, kill albero, `stall_timeout` |
| I-0 | Tiny training ibrido **single-process** (Fase A) per il checkpoint | exit 0, checkpoint+meta ok (copre anche la Fase A, non esercitata dalla validazione training) |
| I-A (pre-fix) | Rollout con `--checkpoint "runs\..."` documentato | `FileNotFoundError` → **bug 1 scoperto**; errore propagato pulito (watchdog `ok:false rc:1`, fase `load inference checkpoint`) |
| I-A (post-fix) | Stesso comando documentato + `--record-outputs` | exit 0, 29 step, return 15.28, `terminated=True (grf_penetration)`, 18 file output, watchdog ok |
| I-B | Negativo: `--no-online-grf-observation` (mismatch obs) | errore chiaro `mat1 1x61 vs mat2 73x64`, exit 1, env chiuso nel finally, watchdog coerente, no orfani |
| Regressione train | `--resume-from "runs\..."` relativo | restore (RLlib iter 1) + iterazione 2 completata, exit 0 |

## Strategia

1. **Review statica integrale** di `rollout_eval.py` (435 righe) e del percorso
   inference di `process_watchdog.py` (`supervise_process`: timeout
   startup/stall/run/phase da heartbeat, kill dell'albero via Job Object).
2. **Fix-first**: i due bug emersi (uno dalla review, uno dal primo run) sono
   stati corretti PRIMA dei run di validazione finali, così la validazione
   copre il codice corretto.
3. **Run mirati**: happy path con la forma di comando documentata e
   `--record-outputs` (percorso OutputRecorder/.sto completo); test negativo di
   mismatch osservazione (percorso d'errore + finally); regressione del fix
   speculare lato training.
4. **Analisi della console per categorie di righe** (warnings, logger nativo
   OpenSim, print storici, diagnostica) per separare regressioni da
   comportamento pre-esistente.

## Dettagli del rollout happy path (I-A post-fix)

- Console: **0 warnings Python/Ray** (la soppressione `filterwarnings` +
  `logging.disable(WARNING)` funziona); barra live `rollout` con step/percento/
  ETA; riga finale `Rollout done: 29 steps, return 15.28, terminated=True
  truncated=False (grf_penetration)` con `end_reason` corretto.
- La policy (1 sola iterazione, output assoluto) penetra il suolo protesico →
  **terminazione `grf_penetration` correttamente attiva anche in inference**
  (semantica ibrida del 2026-06-08).
- `rollout_summary.json` coerente (steps 29, return 15.284, reward min/max,
  `action_abs_max` 0.417, `pelvis_ty_min` 0.935, config ibrida registrata).
- `watchdog_summary.json`: `ok:true`, rc 0, nessun timeout, ultima fase
  `complete`; heartbeat per-step funzionante.
- `sim_outputs/`: set completo di 18 file (kinematics, states, activations,
  muscle_forces, SEA *, recruitment, tau_bio, reserve *, power, online_grf.sto,
  gait_events{,_online}.csv) → compatibile con `visualize.py`.
- Zero processi residui dopo ogni run (restano solo i 2 Python del TensorBoard
  del 2026-06-09).

## Caveat noti (nessuno bloccante)

1. **Righe di init del simulatore in console** (logger nativo OpenSim `[info]`,
   print storici `[Runner]`/`[KinInterp]`, banner `[DEBUG WIN]` del plugin C++):
   presenti solo a build/reset dell'env, pre-esistenti (sono `print`/logger
   nativi, non intercettabili da `logging.disable`). Non è una regressione; il
   claim "0 rumore" del 2026-06-09 riguardava warnings/logger Python.
2. **`[Recruit t=...]` periodici durante il rollout SOLO con
   `--record-outputs`**: diagnostica del recruitment emessa da `output.py`
   ogni `recruitment_diagnostics_interval` step di registrazione. Nei training
   non appare mai (`record_outputs=False`). Su TTY interrompe la barra live.
   Valutare se gating/instradamento (es. interval=0 di default nel rollout o
   print → logging) — TODO minore.
3. **`Rollout done: 0 steps...` stampato anche sul percorso d'errore** (epilogo
   nel `finally` prima del traceback): wording fuorviante, solo cosmetico.
4. Il watchdog esterno dell'inference usa `time.time()` (non monotonic): una
   sospensione del PC durante un rollout può produrre un falso
   `stall_timeout`/`run_timeout`. Pre-esistente; rilevante solo per rollout
   lunghi non presidiati.
5. **`--action-mode`/schema osservazione non verificabili automaticamente dal
   checkpoint** (RLModule non porta metadata): un mismatch dà errore chiaro di
   shape (obs) ma un `--action-mode` sbagliato a parità di shape NON viene
   rilevato (semantica silenziosamente errata). TODO: salvare/verificare un
   metadata accanto a `rl_module_*`.

## File modificati

```text
Trajectory Generator/baseline_MLP/rollout_eval.py   (fix 1 + fix 2)
Trajectory Generator/baseline_MLP/train_ppo_mlp.py  (fix 1 speculare su --resume-from)
```

Nessuna modifica al simulatore root, al plugin C++ SEA o alla semantica del
comando SEA. Directory di test rimosse
(`runs/_validate_inference_20260610_*`, `results/_watchdog_self_test_20260610`).

## Verifiche eseguite

- `py_compile` di `rollout_eval.py` e `train_ppo_mlp.py` (envCMC-rllib): PASS.
- `--help`: `--action-mode`, `--progress`, `--verbose`, `--step-wall-timeout-s`,
  timeout watchdog presenti: PASS.
- `process_watchdog.py --self-test`: PASS (kill su stallo in 1.77 s).
- Matrice I-0 / I-A(pre+post fix) / I-B / regressione resume: come da tabella.
- Analisi console per categorie: 0 warnings; rumore residuo solo init/diagnostica
  pre-esistente.
- Processi residui dopo ogni run: zero.

## TODO

- [ ] (Minore) Valutare il gating dei print `[Recruit]` nei rollout con
      `--record-outputs` (interval=0 di default in inference o instradamento via
      logging) per non interrompere la barra live su TTY.
- [ ] (Minore) Wording dell'epilogo progress su errore ("Rollout stopped" invece
      di "Rollout done" quando il loop esce per eccezione).
- [ ] (Minore) Metadata accanto a `rl_module_*` (action_mode, obs schema, flag
      GRF) e verifica automatica nel rollout per prevenire mismatch silenziosi
      di `--action-mode`.
- [ ] Valutare clock monotonic anche per il watchdog esterno dell'inference
      (falso stall/run timeout dopo sospensione del PC).
