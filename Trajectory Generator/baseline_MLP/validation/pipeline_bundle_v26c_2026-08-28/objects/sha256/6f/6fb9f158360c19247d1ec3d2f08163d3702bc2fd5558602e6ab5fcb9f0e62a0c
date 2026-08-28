# V26C — J20: esecuzione singola del restore audit — **FAIL_CLOSED 4/13**

**Data:** 2026-08-27
**Stadio:** `V26C_J20_RESTORE_AUDIT`
**Foglia:** `Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/j20_runs/j20_restore_audit_v26c_2026-08-27_r1/`
**Verdetto:** **`FAIL_CLOSED`** — 4/13 gate
**Marker finale:** **`TECHNICAL_INVALID` rimasto** — `RESTORE_AUDIT_PASSED` assente
**G9:** **NON chiuso** (`g9_closed: false`)
**Retry:** nessuno, come preregistrato

---

## 1. Esito in una riga

Il child è morto di **self-timeout durante `PPOConfig.build_algo`**, prima di
raggiungere `algo.restore_from_path`. La causa è un **difetto della mia derivazione
del comando**, non un problema del checkpoint, non una questione di risorse e non un
verdetto sulla ripristinabilità.

## 2. Esecuzione

GO verificato prima del lancio: `v26c_j20_restore_audit_architect_go.json`,
sha256 `29775d4265227a5ef3bfc0fca59313f1ffea56d9c1e34b0236440d2638c1c18a`, coincidente
con quello dichiarato; `status: "APPROVED"`, `authorises_execution: true`,
`validate_go` → `valid: true`, **63/63 pin**, preflight READY, foglia assente.

Un solo child, nessun retry:

- PID 62966, avvio `2026-08-27T19:40:44Z`, fine `2026-08-27T19:50:54Z` (≈ 609 s)
- `returncode = 124` (self-timeout)
- `stop_reason: phase_timeout`
- `stop_message`: *"Training STOPPED — PPOConfig.build_algo exceeded the 600 s
  wall-clock timeout (child self-guard)."*
- `watchdog_state.json`: `phase = "PPOConfig.build_algo"`, `timeout_s = 600.0`

## 3. I tredici gate

| Gate | Esito | Perché |
|---|---|---|
| R1 zero nuove iterazioni | **FAIL** | 0 righe e `..._this_process` 0, ma `iterations_run` è `None`: il run è abortito prima di produrre il campo. Assenza di evidenza ≠ evidenza |
| R2 history vuota | **FAIL** | `history` assente dal summary abortito |
| R3 iterazione ripristinata = 1 | **FAIL** | `restored_training_iteration` / `restored_logical_iteration` = `None`: il restore non è avvenuto |
| R4 start 2 / next 2 | **FAIL** | `iteration_start` e `next_iteration` = `None` |
| R5 percorso di restore preso | **FAIL** | `resume_from` è corretto, ma `initialization_mode` = `None` |
| R6 digest attore vivo | **FAIL** | `actor_freeze_audit` vuoto: il modulo vivo non è mai esistito |
| R7 digest critic vivo | **FAIL** | `critic_state_audit` vuoto, stessa ragione |
| **R8 restore ermetico del modulo** | **PASS** | vedi §5 |
| R9 ottimizzatore vivo byte-esatto | **FAIL** | `live_optimizer_audit.json` **non scritto**: il wrapper non è mai stato invocato |
| **R10 niente è stato addestrato** | **PASS** | nessun checkpoint, milestone, rl_module o report di trapianto |
| **R11 sorgenti invariate** | **PASS** | 72 file dell'albero warm-up identici prima/dopo; 63 pin invariati |
| R12 processo unico e pulito | **FAIL** | `returncode 124`, `stop_reason` `phase_timeout` |
| **R13 seconda opinione ermetica** | **PASS** | vedi §5 |

`gates_passed: 4`, `gates_total: 13`.

## 4. Causa radice — verificata, non inferita

Il log del child contiene **115 occorrenze** di:

```
ModuleNotFoundError: No module named 'train_ppo_mlp'
  in ray/_private/runtime_env/setup_hook.py, pickle.loads(worker_setup_func_info.function)
Worker exit detail: Failed to deserialize the setup hook method.
```

Ogni EnvRunner è morto così, quindi `build_algo` non è mai potuto terminare.

La catena, con l'evidenza a supporto di ciascun anello:

1. `train_ppo_mlp.py:1696` registra
   `runtime_env["worker_process_setup_hook"] = _worker_setup`. Ray serializza quella
   funzione **per riferimento** al modulo che la definisce.
2. Ogni worker Ray è un processo nuovo e deve quindi `import train_ppo_mlp` per
   deserializzarla. Ricostruisce `sys.path` dall'interprete e da `PYTHONPATH`, **non**
   dalle mutazioni runtime di `sys.path` del driver.
3. Nel warm-up riuscito lo script lanciato **era** `train_ppo_mlp.py`, quindi la sua
   directory (`baseline_MLP`) finiva in `sys.path[0]` e i worker importavano il
   modulo. Nel suo log l'errore compare **0 volte**.
4. Nella mia derivazione, l'**operazione 4** sostituisce lo script con il wrapper:
   `sys.path[0]` diventa la directory di validation. Il wrapper aggiunge `BASELINE` a
   `sys.path` **solo dentro il proprio processo**, e `launch_once` imposta nell'ambiente
   del child unicamente `PYTHONDONTWRITEBYTECODE` — `PYTHONPATH` risulta `None`.
5. Quindi i worker non hanno alcun modo di trovare `train_ppo_mlp`.

È un difetto dell'interazione fra l'operazione 4 e la serializzazione per riferimento
dell'hook di Ray. **Non è** un'affermazione sul checkpoint, sull'ottimizzatore o sulla
ripristinabilità: quel codice non è mai stato eseguito.

Il rimedio evidente è di una riga — aggiungere `baseline_MLP` a `PYTHONPATH`
nell'ambiente del child dentro `launch_once` — ma **non l'ho implementato e non ho
rilanciato nulla**: nessun retry è ammesso e la decisione è dell'architetto.

## 5. Che cosa è comunque stato dimostrato (R8 e R13)

La metà ermetica, eseguita dal runner **fuori processo**, senza Ray e senza ambiente,
è passata integralmente. Riguarda il checkpoint **come file**:

| Grandezza | Valore |
|---|---|
| classe del modulo | `AsymmetricActorCriticTorchRLModule` |
| chiavi caricate | **16**, `strict=True`, `missing []`, `unexpected []` |
| byte-identità | **tutte le 16 chiavi** |
| digest attore | `d4a13ff742266e9643012a27c57a6ea6b9205b030529d4c7a8af6d874ab26e96` |
| digest critic | `2fa9c124e7b49b679df6db35f6cd4577a70e543541feaa3e6b32bac7afa0a410` |
| righe log-std 2:4 del peso | esattamente zero |
| bias log-std | esattamente `−5.2983174324035645` |
| σ | `0.004999999670722372` su entrambe le dimensioni, esatta |
| ottimizzatore caricato | **sì**, nessun errore |
| indici Adam | **`[6, 7, 8, 9, 10, 11]`** |
| indici 6–11 == i sei tensori del critic | **sì** |
| indici 0–5 disgiunti dal critic | **sì** |
| momenti byte-identici | **12 / 12** (`exp_avg` e `exp_avg_sq` delle sei voci) |
| `step` Adam | `81.0` su tutte e sei |

Quindi: il file del checkpoint è integro e ricaricabile, l'ottimizzatore su disco
porta momenti solo sul critic, e gli indici `[6..11]` **sono** i sei tensori del
critic — affermazione identificata, non posizionale.

**Ciò che resta non dimostrato** è esattamente la parte che G9 esige: che quegli stessi
momenti siano presenti nell'**ottimizzatore vivo** creato da `algo.restore_from_path`.
Nessuna evidenza live esiste — `live_optimizer_audit.json` non è stato scritto — e
questo è il comportamento corretto: il gate ha fallito chiuso invece di dedurre dal
file una proprietà del processo.

## 6. Integrità e contenimento

- **Sorgenti invariate**: i 72 file dell'albero della foglia warm-up hanno hash
  identici prima e dopo (`source_tree_unchanged: true`,
  `source_tree_differences: []`); tutti i 63 pin invariati (`pins_unchanged: true`).
  La foglia warm-up conserva il marker `RESTORE_AUDIT_PENDING`.
- **Nessuna produzione toccata**: nessun file di produzione modificato; il ribinding
  del wrapper è morto con il processo. Nessun config alterato.
- **Contenimento dichiarato** (`inert`): `trained: false`, `sampled: false`,
  `transplanted: false`, `ppo_ex_novo: false`, `promotion: "NONE"`,
  `retried: false`, `supervisor_used: false`, `child_processes_launched: 1`.
- **Marker**: `TECHNICAL_INVALID` **è rimasto**, `RESTORE_AUDIT_PASSED` non è stato
  scritto. La foglia è nata invalida ed è rimasta invalida.
- **Commit verification**: `ok: false`, `all_gates_passed: false`,
  `problems: []`, `marker: TECHNICAL_INVALID`. Nessun problema di integrità: il
  fallimento è dei gate, non della scrittura.
- Nessun `supervisor_state.json`, nessun `live_optimizer_audit.json`, nessun
  checkpoint, nessuna milestone.

## 7. File

**Foglia** (10 elementi): `TECHNICAL_INVALID`, `v26c_j20_restore_audit_receipt.json`
(`de481f8d…`), `v26c_j20_restore_audit_result.json` (`7e069ac6…`),
`commit_verification.json`, `summary.json`, `training_cfg.resolved.yaml`,
`child_stdout_stderr.txt` (749 righe), `faulthandler.log`, `watchdog_state.json`,
`rllib/`.

**Non creati** perché il run è abortito: `live_optimizer_audit.json`, `checkpoint_*`,
`rl_module_*`, `milestone_iteration_*`, `train_iterations.jsonl`.

**Non modificati**: la foglia warm-up e ogni artefatto J0–J19C; i quattro documenti di
preregistrazione (base `1326944e…`, rev1 `3fa1ae23…`, rev2 `59e2e5e0…`,
rev3 `bfb6ef9b…`); runner, wrapper, test e configs; tutti i moduli di produzione.

## 8. Distinzione fra evidenza, inferenza e ipotesi

- **Evidenza**: returncode 124; `phase = PPOConfig.build_algo`; 115
  `ModuleNotFoundError: No module named 'train_ppo_mlp'` nei worker Ray; 0 occorrenze
  nel log del warm-up riuscito; `PYTHONPATH` non impostato nell'ambiente del child;
  `train_ppo_mlp.py:1696` registra l'hook; R8/R13 passati con i valori di §5.
- **Inferenza** (solida, poggia su tutti i punti sopra): la sostituzione dello script
  ha tolto `baseline_MLP` da `sys.path[0]`, i worker non hanno potuto importare il
  modulo che definisce l'hook, e per questo `build_algo` non è terminato.
- **Ipotesi da testare** (non verificata qui): che aggiungere `baseline_MLP` a
  `PYTHONPATH` nell'ambiente del child sia sufficiente a far completare `build_algo` e
  a raggiungere il restore. **Non l'ho verificata**: richiederebbe una nuova
  esecuzione, che non è autorizzata.

## 9. Che cosa questo esito **non** significa

- **Non** significa che il checkpoint sia corrotto o non ripristinabile: R8 e R13
  dicono il contrario sul file.
- **Non** significa che l'ottimizzatore vivo sia sbagliato: non è mai stato creato.
- **Non** chiude né apre G9: G9 resta dovuto, esattamente com'era prima.
- **Non** autorizza nulla: `promotion: NONE`, `training_ready: false`,
  `next_stage_authorized: false`.

## 10. TODO

- [ ] **Decisione dell'architetto** sull'esito FAIL_CLOSED e sul rimedio proposto
      (`PYTHONPATH` nell'ambiente del child in `launch_once`). Se approvato, servirà un
      emendamento immutabile e un nuovo GO operativo: la foglia
      `j20_restore_audit_v26c_2026-08-27_r1` resta evidenza immutabile e **non va
      sovrascritta né riusata**.
- [ ] **G9 resta aperto.** La foglia del warm-up conserva `RESTORE_AUDIT_PENDING`.
- [ ] TODO ereditati: generalizzazione multimodello (epic del 22/08); chiusura del
      gate finale di recupero AB06.

---

**Stato conclusivo:** eseguito **una sola volta**, nessun retry, fallito chiuso con
4/13. Il contenimento ha tenuto in ogni sua parte: sorgenti invariate, marker invalido
conservato, nessuna promozione, nessun addestramento. La causa è mia ed è
identificata con evidenza. **Nessuno stadio successivo preparato o avviato.** In
attesa della review dell'architetto.
