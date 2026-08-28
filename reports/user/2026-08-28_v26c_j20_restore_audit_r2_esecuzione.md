# V26C — J20 R2: esecuzione del secondo tentativo correttivo del restore audit

**Data:** 2026-08-28
**Stadio:** `V26C_J20_RESTORE_AUDIT_R2`
**Foglia:** `Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/j20_runs/j20_restore_audit_v26c_2026-08-27_r2/`
**Exit code:** **1**
**Verdetto:** **`FAIL_CLOSED` — 7/13 gate**
**Marker finale:** **`TECHNICAL_INVALID` conservato** — `RESTORE_AUDIT_PASSED` assente
**G9:** **non chiuso** — `training_ready: false`, `promotion: NONE`, `next_stage_authorized: false`

---

## 1. Autorizzazione ed esecuzione

rev5 richiede **entrambe** le autorizzazioni. Entrambe presenti:

- **utente**, 2026-08-28: *"autorizzo V26C_J20_RESTORE_AUDIT_R2"*, registrata nel GO
  sotto `user_authorisation`;
- **architetto**: GO operativo `v26c_j20_restore_audit_r2_architect_go.json`,
  sha256 **`e5c01b3f6d65842cbf184aee5e6a5cdd1dcda38c600b4402f2a3d9a1222d3d2a`**,
  `stage: V26C_J20_RESTORE_AUDIT_R2`, `status: "APPROVED"`,
  `authorises_execution: true`, verificato indipendentemente dall'architetto.

Prima del lancio: hash del GO coincidente, `validate_go` → `valid: true` con
**76/76 pin**, preflight **READY 76/76**, foglia di destinazione **assente**.

Comando eseguito **esattamente una volta**, dalla directory
`validation/v26c_july_replica_2026-08-26`:

```
env PYTHONDONTWRITEBYTECODE=1 /opt/anaconda3/envs/envCMC-rllib/bin/python \
    v26c_j20_restore_audit_r2.py --execute \
    --go-file v26c_j20_restore_audit_r2_architect_go.json
```

- child unico, PID 21629, `2026-08-27T22:32:07Z` → `2026-08-27T22:32:27Z`
- `elapsed_wall_time_s` 17,5 — **nessun timeout**, contro i 600 s di R1
- **nessun retry**, **nessun PPO**, **nessun ex-novo**, **nessun sampling**,
  **nessun rollout**, nessun supervisore
- `inert`: `trained: false`, `sampled: false`, `transplanted: false`,
  `ppo_ex_novo: false`, `retried: false`, `child_processes_launched: 1`
- nessun `gcs_server` toccato

## 2. La correzione ha funzionato

Il difetto che aveva ucciso R1 è risolto:

| grandezza | R1 | R2 |
|---|---|---|
| `ModuleNotFoundError: No module named 'train_ppo_mlp'` nel log child | **115** | **0** |
| esito di `PPOConfig.build_algo` | timeout a 600 s | **completato** |
| `algo.restore_from_path` | mai raggiunto | **raggiunto** |
| `live_optimizer_audit.json` | non scritto | **scritto** |
| `initialization_mode` | `None` | **`resume_from`** |
| primo entry di `PYTHONPATH` nel child | assente | `…/baseline_MLP` |

## 3. I tredici gate

| Gate | Esito |
|---|---|
| **R1** zero nuove iterazioni | **PASS** |
| **R2** history vuota | **PASS** |
| R3 iterazione ripristinata = 1 | FAIL (a valle, §6) |
| R4 start 2 / next 2 | FAIL (a valle, §6) |
| **R5** percorso di restore preso | **PASS** |
| R6 digest attore vivo | FAIL (a valle, §6) |
| R7 digest critic vivo | FAIL (a valle, §6) |
| **R8** restore ermetico del modulo | **PASS** |
| **R9** ottimizzatore vivo byte-esatto | **FAIL — causa primaria** |
| **R10** niente è stato addestrato | **PASS** |
| **R11** sorgenti invariate | **PASS** |
| R12 processo unico e pulito | FAIL (a valle, §6) |
| **R13** seconda opinione ermetica | **PASS** |

## 4. Causa primaria: R9, e cosa dice davvero

Il wrapper ha catturato l'ottimizzatore **vivo** subito dopo il restore e ha trovato
**8 differenze**, tutte concentrate negli **iperparametri di `param_groups`**:

```
optimizer.param_groups[0].amsgrad                : prim vs scalar
optimizer.param_groups[0].capturable             : prim vs scalar
optimizer.param_groups[0].decoupled_weight_decay : prim vs scalar
optimizer.param_groups[0].differentiable         : prim vs scalar
optimizer.param_groups[0].maximize               : prim vs scalar
optimizer.param_groups[0].betas[0]  0.9    vs 0.8999999761581421
optimizer.param_groups[0].betas[1]  0.999  vs 0.9990000128746033
optimizer.param_groups[0].eps       1e-08  vs 9.99999993922529e-09
```

Cinque sono booleani Python nel checkpoint contro tensori 0-dim nel vivo; tre sono
round-trip in float32.

### Tutto il resto dello stato ottimizzatore è integro

| grandezza | esito |
|---|---|
| momenti `exp_avg` / `exp_avg_sq` | **6/6 indici byte-identici** |
| indici di stato Adam, sorgente e vivo | **`[6, 7, 8, 9, 10, 11]` su entrambi** |
| chiavi top-level, sorgente e vivo | **`['param_groups', 'state']` su entrambi** |
| learner catturati | 1 |
| ottimizzatori catturati | 1, `default_optimizer`, tipo `Adam` |
| `source_state_sha256` | `51d97ef0…`, quello atteso |

E dal restore ermetico indipendente (R8/R13, fuori processo, senza Ray):

| grandezza | esito |
|---|---|
| modulo | 16 chiavi `strict=True`, **tutte byte-identiche** |
| digest attore | `d4a13ff742266e9643012a27c57a6ea6b9205b030529d4c7a8af6d874ab26e96` |
| digest critic | `2fa9c124e7b49b679df6db35f6cd4577a70e543541feaa3e6b32bac7afa0a410` |
| σ | `0.004999999670722372` su entrambe le dimensioni, esatta |
| indici Adam ermetici | `[6..11]`, **sono** i sei tensori del critic |
| `step` Adam | `81.0` su tutte e sei |

## 5. Prova indipendente dell'architetto, verificata in prima persona

L'architetto ha indicato il percorso di conversione dentro RLlib. L'ho **letto e
verificato empiricamente**, non solo citato.

`ray/rllib/core/learner/torch/torch_learner.py`, `_set_optimizer_state`:

```python
self._named_optimizers[name].load_state_dict(
    convert_to_torch_tensor(state_dict["state"], device=self._device)
)
```

`ray/rllib/utils/torch_utils.py`, `convert_to_torch_tensor`: applica
`tree.map_structure(mapping, x)`, cioè **a ogni foglia** della struttura, con
`tensor = torch.from_numpy(np.asarray(item))`, e subito dopo:

```python
# Convert floating-point tensors from float64 to float32 (unless they are float16).
if tensor.is_floating_point() and tensor.dtype != torch.float16:
    tensor = tensor.float()
```

`None` viene lasciato passare invariato.

**Controprova empirica** eseguita con la stessa funzione dell'ambiente:

```
amsgrad       False           -> tensor(False)
betas         (0.9, 0.999)    -> (tensor(0.9000), tensor(0.9990))
eps           1e-08           -> tensor(1.0000e-08)
weight_decay  0               -> tensor(0)
foreach       None            -> None
```

Questo spiega **esattamente e per intero** le 8 differenze: i booleani diventano
tensori 0-dim, i float diventano float32. Spiega anche **perché le altre chiavi non
differiscono**: `weight_decay` e la lista `params` sono interi, che la mia
normalizzazione manda già nel ramo scalare; `foreach` e `fused` restano `None`; i
momenti erano già float32 su entrambi i lati.

Spiega infine **perché `lr` non compare fra le differenze**: l'originale
`_reapply_optimizer_learning_rate` viene chiamato *prima* della cattura, come richiesto
da rev1, e riscrive `lr` con il valore Python esatto `1e-4`. È la conseguenza che rev2
aveva già dichiarato apertamente.

### Conclusione, con i livelli separati

- **Evidenza**: le 8 differenze esistono e sono tutte in `param_groups`; i momenti sono
  6/6 identici; RLlib converte ogni foglia in tensore e i float in float32.
- **Inferenza**: il mismatch è **coerente con una normalizzazione dell'audit troppo
  rigida**, non con una corruzione dimostrata del checkpoint. La mia regola rev2 tiene
  i booleani in forma `prim` tipata e li valuta per primi — corretto per distinguere
  `False` da `0`, ma incompatibile con un lato che li presenta come tensori 0-dim.
- **Ciò che NON è dimostrato**: che il checkpoint sia corrotto. Nulla in questa
  esecuzione lo indica, e R8/R13 indicano il contrario.
- **Ciò che resta comunque non dimostrato**: che l'ottimizzatore vivo sia byte-esatto
  *secondo un criterio corretto*. Il criterio applicato era difettoso, quindi il gate
  non ha prodotto un verdetto utilizzabile in nessuna delle due direzioni.

## 6. R3, R4, R6, R7, R12 sono conseguenze a valle

Non sono misure autonome fallite. Il wrapper solleva `LiveOptimizerMismatch` dentro
`_reapply_optimizer_learning_rate`, che `train_ppo_mlp.py:1744` chiama **prima** di
assegnare `restored_training_iteration` e **prima** del blocco che registra
`actor_freeze_audit` e `critic_state_audit`. Traceback nel summary:

```
File train_ppo_mlp.py, line 1744, in run
    "learners": _reapply_optimizer_learning_rate(algo, args.lr),
File v26c_j20_restore_audit_child.py, ...
    raise LiveOptimizerMismatch(...)
```

Di conseguenza: `restored_training_iteration` 0, `restored_logical_iteration` 0,
`iteration_start` 1, `next_iteration` `None`, `optimizer_lr_audit` `[]`, audit di
attore e critic vuoti, `stop_reason` `error`, child rc 1. Il comportamento è quello
progettato: un mismatch **fa fallire il run di produzione** invece di essere
declassato a warning.

## 7. Integrità e contenimento

- `commit_verification`: `ok: false`, `all_gates_passed: false`, **`problems: []`** —
  il fallimento è dei gate, non delle scritture; `marker: TECHNICAL_INVALID`
- marker: `TECHNICAL_INVALID` **presente**, `RESTORE_AUDIT_PASSED` **assente**
- sorgenti: `source_tree_unchanged: true`, `pins_unchanged: true`, **nessuna
  differenza**; foglia warm-up e foglia R1 invariate
- la foglia R1 non è stata né riusata né toccata
- nessun file di produzione o config modificato

## 8. Artefatti

**Foglia R2** (11 elementi): `TECHNICAL_INVALID`, `live_optimizer_audit.json`,
`v26c_j20_restore_audit_receipt.json`, `v26c_j20_restore_audit_result.json`,
`commit_verification.json`, `summary.json`, `training_cfg.resolved.yaml`,
`child_stdout_stderr.txt`, `faulthandler.log`, `watchdog_state.json`, `rllib/`.

| artefatto | sha256 |
|---|---|
| `v26c_j20_restore_audit_receipt.json` | `acb13a2cc9647b86e6e6b5d17c926790bf2e5aad1a07c0af030f49fdf7a69783` |
| `v26c_j20_restore_audit_result.json` | `28823153b6e46567dcc3a910951bc5f1360ff965f169b2920b3bb08eaa258724` |
| `live_optimizer_audit.json` | `4310e8c3c2207eb145d33c172b95e4c046d39f76097d4d5f349dd9763876d7ae` |
| GO operativo | `e5c01b3f6d65842cbf184aee5e6a5cdd1dcda38c600b4402f2a3d9a1222d3d2a` |

**Non creati** perché il run è abortito al restore: `checkpoint_*`, `rl_module_*`,
`milestone_iteration_*`, `train_iterations.jsonl`.

**File toccati da questa fase**:

- la nuova foglia R2 e i suoi 11 elementi, scritti dall'esecuzione;
- `reports/user/2026-08-28_v26c_j20_restore_audit_r2_esecuzione.md`, questo report,
  creato **dopo** l'esecuzione in applicazione dell'istruzione permanente «un user
  report dopo ogni fase»;
- il GO operativo, creato in una fase precedente già chiusa.

Nessun altro file è stato creato o modificato.

**Invariati, verificati per hash**: base `1326944e…`, rev1 `3fa1ae23…`,
rev2 `59e2e5e0…`, rev3 `bfb6ef9b…`, rev4 `71a6118f…`, rev5 `3d732153…`,
runner R2 `e5776700…`, suite R2 `9f0f69f1…`, wrapper `38a57b25…`,
runner R1 `322b9abc…`, suite R1 `07213d4d…`, GO R1 `29775d42…`.

## 9. Verifiche eseguite

- hash del GO confrontato con quello dichiarato dall'architetto: **coincidente**
- `validate_go` prima del lancio: **valid, 76/76 pin**
- preflight prima del lancio: **READY, 76/76**
- foglia di destinazione assente prima del lancio: **confermato**
- monitoraggio dell'unica esecuzione fino a terminazione
- lettura post-esecuzione di risultato, receipt, evidenza live, summary,
  `commit_verification`, marker e log del child
- conteggio `ModuleNotFoundError` nel log del child: **0**
- lettura diretta di `torch_learner.py` e `torch_utils.py` nell'ambiente locale, più
  controprova empirica di `convert_to_torch_tensor`
- ri-hash delle foglie warm-up e R1 e di tutti i pin: **nessun divergente**

## 10. Proposta futura — descritta, NON autorizzata

Registrata qui per la sola valutazione dell'architetto. **Non implementata, non
preparata, non autorizzata.**

- Un **emendamento additivo** alla preregistrazione che confronti la semantica
  **post-conversione** invece della rappresentazione grezza: normalizzare entrambi i
  lati attraverso la stessa conversione che RLlib applica, oppure confrontare i
  booleani per valore di verità e i float con l'esatta semantica float32, così che
  `False` e `tensor(False)`, e `0.9` e il suo float32, non risultino diversi.
- L'evidenza dovrebbe **registrare anche i valori reali** dei due lati, non solo la
  forma normalizzata, così che una differenza futura sia leggibile senza rieseguire
  nulla.
- Solo dopo, ed eventualmente, un **nuovo audit separato**, con propria foglia, propria
  preregistrazione e **nuova autorizzazione esplicita** di utente e architetto.

Nessun R3 e nessun rev6 sono stati preparati o modificati.

## 11. TODO

- [ ] **G9 resta aperto.** La foglia del warm-up conserva `RESTORE_AUDIT_PENDING`;
      `training_ready` resta `false`.
- [ ] Decisione dell'architetto sull'esito R2 e sulla proposta di §10.
- [ ] Decidere sui due `gcs_server` orfani del 18/08, tuttora **non toccati**.
- [ ] I due check `B06`/`I07` della suite R1 restano rossi per assenza-destinazione
      post-esecuzione: **annotati in rev5, non corretti**, file pinnato e invariato.
- [ ] TODO ereditati ancora aperti: generalizzazione multimodello (epic del 22/08);
      chiusura del gate finale di recupero AB06.

---

**Stato conclusivo:** eseguito **una sola volta**, con doppia autorizzazione, senza
retry, senza training, sampling o rollout. La correzione dell'ambiente ha funzionato e
il restore è stato raggiunto per la prima volta. Il verdetto resta `FAIL_CLOSED 7/13`
per un difetto del criterio di confronto, non per un difetto dimostrato del
checkpoint. **G9 resta aperto e nulla è promosso.**
