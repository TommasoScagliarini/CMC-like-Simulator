# V26C — J20 R3: esecuzione del terzo tentativo correttivo del restore audit

**Data:** 2026-08-28
**Stadio:** `V26C_J20_RESTORE_AUDIT_R3`
**Foglia:** `Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/j20_runs/j20_restore_audit_v26c_2026-08-28_r3/`
**Exit code:** **0**
**Verdetto:** **`RESTORE_AUDIT_PASS` — 13/13 gate**
**Marker finale:** **`RESTORE_AUDIT_PASSED`** — `TECHNICAL_INVALID` rimosso
**G9:** **CHIUSO** — e nient'altro. `training_ready: false`, `promotion: NONE`, `next_stage_authorized: false`

---

## 1. Verifiche read-only prima del lancio

| controllo | esito |
|---|---|
| sha256 del GO operativo | `fe090682f046a1ba74a3cc4f7d8c6b319c243920a375b8c19265e107bb9ca583` — **coincidente** |
| `validate_go` | **`valid=True`, 0 problemi**, pin **92/92** |
| preflight | **READY**, pin **92/92**, entry evidence OK |
| foglia di destinazione | **assente** |
| processi Ray preesistenti | **2** — i due `gcs_server` orfani del 18/08, non toccati |

## 2. Esecuzione

Comando eseguito **esattamente una volta**, dalla directory
`validation/v26c_july_replica_2026-08-26`:

```
env PYTHONDONTWRITEBYTECODE=1 /opt/anaconda3/envs/envCMC-rllib/bin/python \
    v26c_j20_restore_audit_r3.py --execute \
    --go-file v26c_j20_restore_audit_r3_architect_go.json
```

- child unico, **PID 42763**, `2026-08-27T23:38:48Z` → `2026-08-27T23:39:07Z`
- `elapsed_wall_time_s` **16,7** — nessun timeout (guard 600 s)
- **nessun retry**, **nessun supervisore**, nessun PPO, nessun ex-novo
- `inert`: `trained: false`, `sampled: false`, `transplanted: false`,
  `ppo_ex_novo: false`, `retried: false`, `supervisor_used: false`,
  `child_processes_launched: **1**`
- `pythonpath_first_entry` = `…/baseline_MLP` — la correzione rev4 ha retto
- stdout del runner: `{"leaf": …, "verdict": "RESTORE_AUDIT_PASS", "gates": "13/13"}`

## 3. I tredici gate

| Gate | Esito |
|---|---|
| R1 zero nuove iterazioni | **PASS** |
| R2 history vuota | **PASS** |
| R3 iterazione ripristinata = 1 | **PASS** |
| R4 start 2 / next 2 | **PASS** |
| R5 percorso di restore preso | **PASS** |
| R6 digest attore vivo | **PASS** |
| R7 digest critic vivo | **PASS** |
| R8 restore ermetico del modulo | **PASS** |
| **R9 ottimizzatore vivo esatto sotto le semantiche di conversione RLlib** | **PASS** |
| R10 niente è stato addestrato | **PASS** |
| R11 sorgenti invariate | **PASS** |
| R12 processo unico e pulito | **PASS** |
| R13 seconda opinione ermetica | **PASS** |

`gates_failed: []`, `gates_not_evaluated: []`, `live_restore.completed: **true**`
— quindi i cinque gate a valle di R9 sono stati **valutati davvero**, non
saltati: `evidence_written`, `evidence_at_after_restore`,
`wrapper_did_not_raise` e `production_recorded_the_reapply` tutti veri.

## 4. R9: la correzione rev6 ha funzionato, e la previsione era esatta

Il wrapper ha catturato l'ottimizzatore **vivo** subito dopo il restore.

**Lato grezzo — riproduce R2 alla lettera.** Otto differenze, le stesse:

```
optimizer.param_groups[0].amsgrad                : prim vs scalar
optimizer.param_groups[0].betas[0]  0.9    vs 0.8999999761581421
optimizer.param_groups[0].betas[1]  0.999  vs 0.9990000128746033
optimizer.param_groups[0].capturable             : prim vs scalar
optimizer.param_groups[0].decoupled_weight_decay : prim vs scalar
optimizer.param_groups[0].differentiable         : prim vs scalar
optimizer.param_groups[0].eps       1e-08  vs 9.99999993922529e-09
optimizer.param_groups[0].maximize               : prim vs scalar
```

`reproduces_r2_eight: true`. Digest grezzi: sorgente `c1a9e152…`, vivo
`a5487d5c…` — **identici a quelli che R2 aveva committato**.

**Lato canonico — zero differenze.**

| | |
|---|---|
| `canonical.difference_count` | **0**, lista vuota |
| digest canonico **sorgente** | `2d0a041b2b8c60fc3c68d47cb9fd4eb72cc6b5d6ce12fd93b89cff223cc142a6` |
| digest canonico **vivo** | `2d0a041b2b8c60fc3c68d47cb9fd4eb72cc6b5d6ce12fd93b89cff223cc142a6` |
| `param_groups` canonico | esatto, digest coincidenti |

**Questo digest era stato predetto offline.** La suite R3 lo aveva calcolato
ricostruendo il lato vivo dalla conversione, prima che la run esistesse: stesso
valore, `2d0a041b…`. La diagnosi non era una congettura.

**Le otto equivalenze accettate**, con i valori reali di entrambi i lati:

| percorso | checkpoint | vivo |
|---|---|---|
| `param_groups[0].amsgrad` | `False` | `tensor(False)` |
| `param_groups[0].betas[0]` | `0.9` | `tensor(0.9000)` |
| `param_groups[0].betas[1]` | `0.999` | `tensor(0.9990)` |
| `param_groups[0].capturable` | `False` | `tensor(False)` |
| `param_groups[0].decoupled_weight_decay` | `False` | `tensor(False)` |
| `param_groups[0].differentiable` | `False` | `tensor(False)` |
| `param_groups[0].eps` | `1e-08` | `tensor(1.0000e-08)` |
| `param_groups[0].maximize` | `False` | `tensor(False)` |

`accepted_equivalences_are_exactly_the_expected_eight: **true**`,
`unexplained_canonical_paths: []`.

**Momenti:** `moment_digests_match: true`;
`moments_unchanged_by_canonicalisation` **true su entrambi i lati** — la
canonicalizzazione non ha toccato un solo tensore dei momenti.

**Struttura:** indici `[6,7,8,9,10,11]` su entrambi i lati; chiavi top-level
`['param_groups','state']` su entrambi; `param_group_sizes_live: [12]`;
1 learner, 1 ottimizzatore, `default_optimizer` di tipo `Adam`;
`source_state_sha256` `51d97ef0…`, quello atteso.

## 5. Il gate rev7 sul learning rate: verde, e non vacuo

| campo | valore |
|---|---|
| `matches` | **`true`** |
| `gated` | `true` |
| `live_lr_before_reapply` | `9.999999747378752e-05` — cioè `float32(1e-4)` |
| `source_lr_raw` | `0.0001` (float Python) |
| `canonical_source_lr` | `('t','float32',(),'0f2f3591…')` |
| `canonical_live_lr_before_reapply` | `('t','float32',(),'0f2f3591…')` — **identico** |
| `canonical_dtype` | `float32` |
| `production_reports_seen` | 1 |

Il learning rate che il **restore** ha prodotto coincide canonicamente con
quello del checkpoint. È la prima volta che questo viene **misurato e preteso**:
sotto rev6 sarebbe passato comunque.

## 6. Provenienza della conversione, registrata

| | |
|---|---|
| `call_site_verified` | **true** |
| `device_parameter_present` | true |
| `downcast_clause_present` / `none_passthrough_clause_present` | true / true |
| `conversion_fixed_point_source` / `_live` | **true / true** |
| ray / torch / numpy | `2.55.1` / `2.10.0` / `2.2.6` |
| `torch_utils.py` sha256 | `2e10a4efbc06f1f1ae241caee750befbaf6b61c0c7f2a0490da3ff097f9c64eb` |
| `torch_learner.py` sha256 | `0d41e90403b6744985c90add1f77b05531e263601cf8740d3eb777b7d2d4fc3c` |
| `convert_to_torch_tensor` sorgente sha256 | `a1e5f39aac23534458ba63d59d45af58701ae866e330ce0a38530503c84cd179` |
| `_set_optimizer_state` sorgente sha256 | `a1cbd38fbdf47a73d83f0f7dfca6864837c75aabbade579700e86960edf532ca` |
| device | `cpu` |

## 7. Zero iterazioni, dimostrate

```
Resumed checkpoint …/checkpoint_last at RLlib iteration 1,
logical iteration 1; next logical iteration 2/1.
```

`range(2, 2)` è vuoto. Dal summary: `iterations_run: 0`,
`iterations_completed_this_process: 0`, `history: []`,
`restored_training_iteration: 1`, `restored_logical_iteration: 1`,
`iteration_start: 2`, `next_iteration: 2`,
`initialization_mode: 'resume_from'`, `warm_start_raw_transplant_applied_this_process: false`,
`stop_reason: 'completed'`, `interrupted: false`, `timed_out: false`, `error: null`.

`actor_freeze_audit` e `critic_state_audit` hanno **una sola** voce ciascuno,
stadio `before_training`: nessuna voce post-iterazione può esistere perché
nessuna iterazione è avvenuta. `optimizer_lr_audit`: `['after_restore']`.

**Nessun artefatto proibito** nella foglia: nessun `checkpoint_*`, `rl_module_*`,
`milestone_iteration_*`, `actor_transplant_report.json`, `train_iterations.jsonl`,
`supervisor_state.json`.

Costo dichiarato e sostenuto: 13 EnvRunner, `ray_num_cpus 14`, costruiti e
possibilmente resettati in setup. **Zero `algo.train`, zero sampling, zero
rollout.**

## 8. Seconda opinione ermetica (R8/R13)

Fuori processo, senza Ray e senza environment:

- 16 chiavi caricate con `strict=True`, **tutte byte-identiche**
- digest attore `d4a13ff742266e9643012a27c57a6ea6b9205b030529d4c7a8af6d874ab26e96`
- digest critic `2fa9c124e7b49b679df6db35f6cd4577a70e543541feaa3e6b32bac7afa0a410`
- σ `0.004999999670722372` su entrambe le dimensioni, esatta
- indici Adam `[6..11]`, **sono** i sei tensori del critic
- `step` `81.0` su tutti e sei; **tutti i momenti byte-identici**

I digest coincidono con quelli che il codice di produzione ha registrato dal
modulo vivo ripristinato (R6/R7). Due metodi indipendenti, stessa risposta.

## 9. Integrità e contenimento

- `commit_verification`: **`ok: true`**, `all_gates_passed: true`,
  `problems: []`, `marker: RESTORE_AUDIT_PASSED`
- `source_tree_unchanged: **true**` — 72 file della foglia warm-up ri-hashati
  prima e dopo, **zero differenze**
- `pins_unchanged: **true**` — **zero** pin divergenti
- `r1_leaf_unchanged: true`, `r2_leaf_unchanged: true`
- wrapper congelato invariato: `38a57b25…`
- log del child: **0** `ModuleNotFoundError`, **0** `Traceback`,
  **0** `LiveOptimizerMismatch`, **0** `SYSTEM_ERROR`
- processi dopo la run: **2**, gli stessi due `gcs_server` orfani del 18/08.
  Nessun processo Ray nuovo, nessun processo R3 residuo.
- nessun file di produzione, config, checkpoint o foglia precedente modificato

## 10. Artefatti della foglia R3

| artefatto | sha256 |
|---|---|
| `RESTORE_AUDIT_PASSED` | `56bac0767c80d4a861c28f97d9aafbdf6c1e7061783b0ca5bba317b121705f2e` |
| `v26c_j20_restore_audit_result.json` | `dbd1837a0271c831578e05bf18dc5f4badc83fdd5446632b8e9ac20e584ababb` |
| `v26c_j20_restore_audit_receipt.json` | `0426774a88b3793acc7a639a4817f8d008b730b835a63bc70dc4dbba7248c51c` |
| `live_optimizer_audit.json` | `9e6f2acc48f4a3f6d024b0ecfda5b57ff9643d9873d192349bc90b1d7af81c3c` |
| `commit_verification.json` | `e53c48a9d7a59da5ada3bac0bebbd7b42c8e8437fa1915ead5dbdca73c5b58fb` |
| `summary.json` | `748e25725beb264524e9f14170d9832bcf08ba3c0af96f4265ab3c5f91027f21` |
| `child_stdout_stderr.txt` | `8163d3cc5d0aa0a94b9dd07034a2346c4438952b6f8d2807a47599b3d6f28133` |
| `watchdog_state.json` | `90e6ea90a06325469da7246a8dbf1f3bafc0aab366298cccc0cae924c0349a48` |
| `faulthandler.log` | `5db53236c7f04fecdd1c18eafe871c7560ffa21851667df0e4b038c0a10e8e10` |
| `training_cfg.resolved.yaml` | `976f6c5050161f3edf610de1843c186186fdc56ea9d563cf0fc4c1958c14a810` |

**Input pinnati, invariati:** runner `23df351e…`, wrapper R3 `db4afc54…`,
suite `8ac2248a…`, rev6 `6575f64c…`, rev7 `32238daf…`, GO `fe090682…` con **92
pin**, precedenza `rev7 > rev6 > … > base`, `module_state.pkl` `57720e2e…`.

**File creati da questa fase:** la foglia R3 e i suoi 11 elementi, più questo
report. Nessun altro.

## 11. Osservazione, non un'azione

La foglia del warm-up conserva ancora il marker `RESTORE_AUDIT_PENDING`. È
**corretto e voluto**: il documento base e rev6 impongono che la foglia sorgente
sia **read-only**, e R11 lo verifica ri-hashando l'intero albero prima e dopo.
Il debito è saldato e registrato **nella foglia R3** (`g9_closed: true`), non
mutando la foglia sotto audit. Se l'architetto vuole che quel marker rifletta la
chiusura, serve una decisione separata: toccarlo adesso violerebbe R11.

## 12. TODO

- [x] ~~G9 aperto~~ — **CHIUSO** da questa esecuzione, `g9_closed: true`.
- [ ] **`training_ready` resta `false`.** Un R3 che passa chiude G9 e nient'altro.
      Portare la pipeline a `training_ready: true` **senza addestrare** richiede
      una **attestazione di aggregazione delle evidenze separata**, che ora può
      finalmente pinnare gli hash reali della foglia R3 elencati al §10. Non
      progettata, non preregistrata, non eseguita: spetta all'architetto.
- [ ] Decisione dell'architetto sul marker `RESTORE_AUDIT_PENDING` della foglia
      warm-up (§11).
- [ ] Le tre osservazioni «registrato ma non gateato» di rev7 §`correction_6`,
      segnalate e non implementate.
- [ ] Decidere sui due `gcs_server` orfani del 18/08, tuttora **non toccati**.
- [ ] I 2 check `B06`/`I07` della suite R1 e i 4 `C29`/`E01`/`H02`/`H04` della
      suite R2 restano rossi per assenza-destinazione post-esecuzione:
      **annotati in rev5 e rev6, non corretti**, file pinnati e invariati. Alla
      prossima riesecuzione la suite R3 mostrerà la stessa classe.
- [ ] TODO ereditati ancora aperti: generalizzazione multimodello (epic del
      22/08); chiusura del gate finale di recupero AB06.

---

**Stato conclusivo:** eseguito **una sola volta**, con doppia autorizzazione,
senza retry, senza training, sampling o rollout. **`RESTORE_AUDIT_PASS`, 13/13.**
Il checkpoint del warm-up critic-only si ricarica attraverso il percorso di
produzione reale e torna **esatto**: modulo, attore, critic, σ, momenti Adam,
step, e — per la prima volta preteso — il learning rate. **G9 è chiuso. Nulla è
promosso, nulla è training-ready, nessuno stadio successivo è autorizzato.**
