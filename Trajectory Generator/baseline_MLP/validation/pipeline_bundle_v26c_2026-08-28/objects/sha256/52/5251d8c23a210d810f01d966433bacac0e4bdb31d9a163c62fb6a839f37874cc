# V26C — J20 R2: preparazione del secondo tentativo correttivo del restore audit

**Data:** 2026-08-27
**Stadio:** `V26C_J20_RESTORE_AUDIT_R2`
**Stato:** **artefatti consegnati, NON eseguiti.** GO in stato **DRAFT doppiamente inerte**.
**Prereg vigente:** rev5 (`3d732153…`) — precedenza `rev5 > rev4 > rev3 > rev2 > rev1 > base`
**Foglia di destinazione:** `j20_runs/j20_restore_audit_v26c_2026-08-27_r2/` — **assente**
**Preflight:** READY, 76/76 pin — **Suite R2:** 223/223
**R1:** preservata intatta, `FAIL_CLOSED 4/13` sotto `TECHNICAL_INVALID`

---

## 1. Problema

R1 è fallita chiusa a 4/13 senza raggiungere `algo.restore_from_path`: il child è
morto di self-timeout durante `PPOConfig.build_algo`. R2 è il **secondo tentativo
correttivo** dello stesso audit — non un retry automatico — con una sola correzione e
R1 preservata come evidenza immutabile.

## 2. Evidenza diretta, inferenza causale, ipotesi

rev5 separa formalmente i tre livelli. Qui la sintesi.

### Evidenza diretta (letta dagli artefatti R1, senza interpretazione)

- `child_returncode` **124**; `stop_reason` `phase_timeout`
- `stop_message`: *"PPOConfig.build_algo exceeded the 600 s wall-clock timeout (child self-guard)"*
- `watchdog_state.json`: `phase = PPOConfig.build_algo`, `timeout_s = 600.0`
- **115 occorrenze** di `No module named 'train_ppo_mlp'` nel log del child R1
- ogni occorrenza con traceback in `ray/_private/runtime_env/setup_hook.py:244`,
  `setup_func = pickle.loads(worker_setup_func_info.function)`
- Ray: *"Worker exit type: SYSTEM_ERROR — Failed to deserialize the setup hook method"*
- **`PYTHONPATH` assente** nell'ambiente del child R1
- **0 occorrenze** dello stesso errore nel log del warm-up riuscito
- `train_ppo_mlp.py:1696` registra `runtime_env["worker_process_setup_hook"]`
- nessun `live_optimizer_audit.json` scritto

### Inferenza causale

La sostituzione dello script (operazione 4) ha tolto `baseline_MLP` da `sys.path[0]`;
i worker Ray, processi nuovi, non hanno potuto importare il modulo a cui il payload
dell'hook si riferisce.

**Meccanismo affinato in rev5** — cloudpickle serializza **per valore** una funzione
definita in `__main__` e **per riferimento** una definita in un modulo importabile. Il
docstring di `_worker_setup` lo dice esplicitamente: *"Picklable-by-value (defined in
`__main__` when run as a script)"*. Il warm-up lanciava `train_ppo_mlp.py` come
script → `__main__` → per valore → i worker non dovevano importare nulla. Il wrapper
lo **importa** → `__module__ == "train_ppo_mlp"` → per riferimento → ogni worker deve
importarlo.

Lo smoke test (§5) sposta buona parte di questa catena da argomento a **misura**.
Resta inferenziale l'attribuzione del successo del warm-up al pickling per valore:
quel run non era strumentato, e le sue 0 occorrenze sono *compatibili* con la
spiegazione senza dimostrarla.

### Ipotesi ancora aperta

Che con la correzione Ray deserializzi l'hook, `build_algo` completi e il restore
venga raggiunto. **Non verificata**: solo l'esecuzione di R2 può deciderlo.

## 3. Soluzione — una sola correzione

`child_environment()` antepone la directory **assoluta** `baseline_MLP` a `PYTHONPATH`
nell'ambiente del child: valore preesistente preservato **verbatim** dopo
`os.pathsep`, nessun separatore se non c'è nulla, costruito su una **copia** di
`os.environ`, e **fail-closed** se la directory non è assoluta o non contiene
`train_ppo_mlp.py`.

Il timeout di **600 s resta invariato**, per decisione dell'architetto.

## 4. Strategia — copia meccanica

Il runner R1 resta **byte-identico** (`322b9abc…`). R2 è una copia meccanica. I
conteggi AST esatti, ora **asseriti dalla suite** (check `A04a`–`A04g`) e non solo
descritti:

| grandezza | valore |
|---|---|
| funzioni definite nel runner R1 | **25** |
| funzioni definite nel runner R2 | **26** |
| funzioni condivise | **25** |
| di cui **byte-identiche** | **20** |
| di cui **modificate** | **5** — `launch_once`, `go_pin_targets`, `expected_pin_hashes`, `check_entry_evidence`, `run_execution` |
| **nuove** | **1** — `child_environment` |
| **differenti in totale** | **6** (5 modificate + 1 nuova) |

Una versione precedente di questo report diceva «1 nuova e 4 modificate» mentre ne
elencava 5: errore di conteggio mio, corretto qui e ora vincolato dai check `A04a`–`A04g`,
che falliscono se i numeri si discostano dal codice.

Restano byte-identiche `evaluate_gates`, `restore_command`, `hermetic_restore`,
`verify_commit`, `validate_go` — quindi 13 gate, quattro operazioni, 18 token delegati
e la regola `APPROVED` esatto non possono essersi mossi. R11 non è stato toccato: gli
8 file della foglia R1 sono entrati **nella mappa dei pin**, quindi `pins_unchanged`
li copre già.

**Stadio distinto** `V26C_J20_RESTORE_AUDIT_R2` (confermato dall'architetto): il GO di
R1 non può autorizzare R2 e viceversa. Uniche differenze argv: `--output-dir` e
`--audit-evidence`, conseguenze della foglia nuova.

## 5. Smoke test reale — non più solo `find_spec`

rev5 ha sostituito il controllo `find_spec`-only perché `find_spec` prova che il
modulo è **trovabile**, non che l'operazione realmente fallita — cloudpickle che
deserializza un payload per riferimento — ora riesca.

Il nuovo smoke, **senza `ray.init`, senza Algorithm, senza ambienti, senza training,
senza daemon**:

1. importa `train_ppo_mlp` nel processo preparatore (Ray **non** risulta
   inizializzato dopo l'import);
2. serializza `train_ppo_mlp._worker_setup` con **`ray.cloudpickle`**, lo stesso
   serializzatore che usa Ray;
3. **negativo** — interprete nuovo, cwd la directory di validation, **senza**
   `PYTHONPATH`: deve fallire e fallisce **precisamente** con
   `ModuleNotFoundError ... No module named 'train_ppo_mlp'`, la stessa identica
   eccezione dei 115 worker;
4. **positivo** — stesso interprete con l'ambiente di `child_environment()`:
   `LOADOK train_ppo_mlp _worker_setup True`, cioè callable con modulo e nome attesi;
5. la funzione deserializzata **non viene mai invocata**, solo ispezionata.

Osservazioni registrate: il payload è di **46 byte** e contiene la stringa
`train_ppo_mlp` — la forma di un pickle **per riferimento**; per contrasto, un pickle
per valore di una funzione equivalente è di **417 byte**. Registrato come
corroborazione dell'inferenza causale, non come gate.

`find_spec` è mantenuto come controllo preliminare (`MISSING` → `FOUND`, con `origin`
esattamente su `baseline_MLP/train_ppo_mlp.py`).

**Cosa prova**: che il payload dell'hook reale deserializza in un interprete nuovo con
la correzione e non senza. **Cosa non prova**: che `build_algo` completi. Resta ipotesi.

## 6. File

**Nuovi o aggiornati**

| file | sha256 | righe |
|---|---|---|
| `v26c_j20_prereg_restore_audit_rev5.json` | `3d732153fd51bb7956f91d194fc0f4b1e74c7b35cd451d7a24a0cc70255f675d` | 124 |
| `v26c_j20_restore_audit_r2.py` | `e5776700f9e12c8632d239898b673eba0f0cab6e08905001a9778bc12b75e81c` | 1490 |
| `test_v26c_j20_restore_audit_r2.py` | `9f0f69f1695ea64404587d6274079317925a877a729df6f589b05a8c5faabae8` | 1004 |
| `v26c_j20_restore_audit_r2_go_DRAFT.json` | `3977af5493e043c870390d1d065bd4eec7e9cdf5df78afe29853aa66fc9ecbe7` | 144 |

**Invariati, verificati per hash**

| file | sha256 |
|---|---|
| `v26c_j20_prereg_restore_audit.json` | `1326944e…` |
| `…_rev1.json` | `3fa1ae23…` |
| `…_rev2.json` | `59e2e5e0…` |
| `…_rev3.json` | `bfb6ef9b…` |
| `…_rev4.json` | `71a6118f…` |
| `v26c_j20_restore_audit_child.py` | `38a57b25…` (wrapper: **lo stesso** di R1) |
| `v26c_j20_restore_audit.py` | `322b9abc…` |
| `test_v26c_j20_restore_audit.py` | `07213d4d…` |
| `v26c_j20_restore_audit_architect_go.json` | `29775d42…` |

Foglia warm-up: 14 pin, **nessuno divergente**. Foglia R1: 8 pin, **nessuno
divergente**. Nessun file di produzione né config modificato in questo task: le tre
`M` di `git status` (`tb_logging.py`, `train_ppo_mlp.py`, `training_exnovo_cfg.yaml`)
preesistono e i loro hash coincidono con i pin, verificato dal preflight 76/76.

## 7. Verifiche eseguite

- `py_compile` su runner R2, suite R2, wrapper e runner R1: **OK**
- **suite R2: 223/223**, senza child, `ray.init`, Algorithm, ambiente o training
- preflight R2: **READY, 76/76 pin**, stadio `V26C_J20_RESTORE_AUDIT_R2`
- dry-run: stampa e basta, **foglia R2 assente dopo**
- smoke reale: negativo e positivo entrambi come attesi (§5)
- nessun processo Ray creato: conteggio identico prima e dopo lo smoke, e il check è
  esplicitamente non-vacuo (fallisce se il conteggio non è misurabile)
- DRAFT GO: `validate_go` → `valid: False`, con **due** rifiuti —
  `"authorises_execution is not exactly true"` e
  `"status is 'DRAFT', and only the exact string 'APPROVED' authorises execution"`

### Copertura dei requisiti

| Requisito | Check |
|---|---|
| BASELINE assoluta prima in PYTHONPATH | B02, B03, B04, B10 |
| valore esistente preservato | B09, B11, B12, B13 |
| `os.pathsep` | B14, B15, B12 |
| env realmente passato a Popen | B21, B24, B25, B26, B27 |
| nessuna mutazione env globale | B06, B07, B16 |
| un solo launch | B23, B28, B29, B30 |
| nessun train/sample/rollout | B31 (×4), A06, H09 |
| nuova foglia | A12, E01, E02, H06, H07 |
| smoke negativo/positivo reale | C10–C23 |
| hook mai invocato | C24 |
| nessun `ray.init` (AST, non sottostringa) | C25, C26, C26b, C26c, C26d |
| nessun processo Ray nuovo | C28a, C28 |
| nessuna foglia R2 dallo smoke | C29, C30 |
| rev5 sealed e coerente | F24–F44 |
| conteggi AST esatti | A04a–A04g |
| diagnostica processi Ray robusta | C28a–C28e |
| DRAFT doppiamente inerte | F18, F18b, F18c, F21b |
| R1 intatta | D01–D18 |

### Difetto trovato dalla review indipendente (Codex)

La suite **non passava** sulla macchina del revisore. `ray_process_count()` proteggeva
solo l'**import** di psutil, non l'**enumerazione**: nel loro ambiente
`ray.thirdparty_files.psutil` importa correttamente, ma `process_iter()` solleva
`PermissionError [Errno 1]` sotto la sandbox macOS **durante l'iterazione**. L'eccezione
sfuggiva e il fallback `ps` non veniva mai raggiunto, quindi la suite abortiva.

Nel mio ambiente psutil non è installato affatto, quindi il ramo psutil non veniva mai
esercitato e il difetto era invisibile: è esattamente il tipo di errore che solo
un'esecuzione su una macchina diversa può far emergere.

**Correzione** (solo diagnostica della suite; runner R2, rev5 e requisiti non toccati,
nessun rev6):

- `_load_psutil()` prova `psutil` e poi `ray.thirdparty_files.psutil`;
- `_ray_count_via_psutil()` racchiude **l'intera enumerazione** nella gestione errore e
  restituisce `None` se viene rifiutata;
- `_ray_count_via_platform()` è il fallback (`ps` su POSIX, `wmic` su Windows);
- `ray_process_count()` prova entrambe le vie e restituisce `None` solo se **nessuna**
  misura.

Il check resta **non-vacuo**: `C28a` fallisce se il conteggio non è misurabile, quindi
un'assenza di misura non può passare per successo. Aggiunti quattro check che
riproducono il difetto: `C28b` (uno psutil che rifiuta di enumerare degrada a `None`
invece di sollevare), `C28c` (il conteggio riesce comunque via fallback), `C28d` (con
entrambe le vie indisponibili il risultato è `None`, così il check fallisce chiuso),
`C28e` (il loader reale viene ripristinato).

### Due difetti dei miei test, trovati e corretti prima della review

- **C26** verificava l'assenza della stringa `"ray.init("` nel proprio file: l'unica
  occorrenza era il letterale dentro il check stesso, quindi non poteva mai passare.
  Sostituito con un controllo **AST**, più un check di non-vacuità che dimostra che
  rileva una vera chiamata.
- **F07** pretendeva che rev4 dichiarasse la precedenza *corrente*. rev4 dichiara
  quella del proprio momento; rev5 l'ha superata. Corretto: rev4 deve coincidere con
  la catena del runner **meno rev5**.

## 8. Punti che richiedono la tua review

1. **Due `gcs_server` Ray orfani** girano da sessioni del **18/08** (8-9 giorni),
   quindi **precedenti a R1 di nove giorni**: non sono suoi residui. Li **segnalo
   senza attribuire causalità** — la causa di R1 è provata ed è un'altra — e **non li
   ho toccati**. Decidi tu se vanno ripuliti prima di R2.
2. **I due check storici della suite R1** (`B06`, `I07`) restano rossi: asseriscono
   che la foglia R1 sia assente, vero solo prima dell'esecuzione. **Annotati in rev5,
   non corretti**; il file resta byte-identico e pinnato, come da tua decisione.
3. **L'ipotesi resta aperta**: lo smoke prova la condizione necessaria, non che
   `build_algo` completi.
4. **Rischio residuo sul timeout**: non ho una misura del tempo "sano" di
   `build_algo`, perché in R1 non è mai arrivato in fondo. Se dopo la correzione
   restasse lento, i 600 s potrebbero tornare a mordere. Segnalo il rischio; **il
   timeout resta invariato** come deciso.

## 9. Comando esatto proposto (da NON eseguire ora)

Dalla directory `Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26`:

```
PYTHONDONTWRITEBYTECODE=1 /opt/anaconda3/envs/envCMC-rllib/bin/python \
    v26c_j20_restore_audit_r2.py --execute \
    --go-file <GO operativo: status APPROVED, stage V26C_J20_RESTORE_AUDIT_R2>
```

Richiede, per rev5, **sia** l'autorizzazione esplicita dell'utente **sia** il GO
dell'architetto. Nessuno dei due da solo è sufficiente. Una sola esecuzione, nessun
retry qualunque sia l'esito.

## 10. TODO

- [ ] **Review dell'architetto + autorizzazione esplicita dell'utente**, poi GO
      operativo per `V26C_J20_RESTORE_AUDIT_R2`.
- [ ] Decidere sui due `gcs_server` orfani del 18/08.
- [ ] **G9 resta aperto**; la foglia del warm-up conserva `RESTORE_AUDIT_PENDING`.
- [ ] TODO ereditati: generalizzazione multimodello (epic del 22/08); chiusura del
      gate finale di recupero AB06.

---

**Stato conclusivo:** pacchetto R2 preparato e fermo. R1 intatta e pinnata, rev5
sigillata, preflight READY 76/76, suite 212/212, smoke reale superato senza Ray, GO in
DRAFT doppiamente inerte, foglia R2 assente. **Nessun GO APPROVED emesso, nessuna
esecuzione, nessun PPO.**
