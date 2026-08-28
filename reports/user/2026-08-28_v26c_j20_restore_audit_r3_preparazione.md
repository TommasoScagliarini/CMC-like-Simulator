# V26C — J20 R3: preparazione del terzo tentativo correttivo del restore audit

**Data:** 2026-08-28
**Stadio:** `V26C_J20_RESTORE_AUDIT_R3`
**Fase:** **PREPARAZIONE SOLTANTO — NIENTE È STATO ESEGUITO**
**Foglia di destinazione:** `Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/j20_runs/j20_restore_audit_v26c_2026-08-28_r3/` — **assente**
**GO:** **solo DRAFT, doppiamente inerte** — `status: "DRAFT"`, `authorises_execution: false`
**G9:** **resta aperto**, `training_ready: false`, `promotion: NONE`

---

## 1. Problema

R2 ha raggiunto per la prima volta `algo.restore_from_path` ed è fallito closed
a **7/13**. Il gate R9 ha riportato **8 differenze**, tutte dentro
`param_groups[0]`: cinque booleani Python contro tensori 0-dim, e `betas[0]`,
`betas[1]`, `eps` contro le loro immagini float32. **Tutti i 12 tensori dei
momenti coincidevano byte per byte**, gli indici di stato coincidevano, le
chiavi top-level coincidevano.

Il verdetto non era utilizzabile in **nessuna** delle due direzioni: il criterio
applicato era difettoso, quindi R9 non ha mai prodotto una risposta valida.

## 2. Causa radice — verificata in prima persona, non citata

Ho letto il codice installato prima di scrivere una riga.

`ray 2.55.1`, `torch 2.10.0`, `numpy 2.2.6`.
`ray/rllib/core/learner/torch/torch_learner.py`, `TorchLearner._set_optimizer_state`:

```python
self._named_optimizers[name].load_state_dict(
    convert_to_torch_tensor(state_dict["state"], device=self._device))
```

`ray/rllib/utils/torch_utils.py`, `convert_to_torch_tensor`: applica
`tree.map_structure(mapping, x)` a **ogni foglia**, e `mapping`:

- lascia passare `None` invariato (`if item is None: return item`);
- **preserva** un tensore già esistente (`if torch.is_tensor(item): tensor = item`);
- avvolge un `ndarray` con `torch.from_numpy`;
- **altrimenti** calcola `torch.from_numpy(np.asarray(item))`;
- poi `if tensor.is_floating_point() and tensor.dtype != torch.float16: tensor = tensor.float()`;
- infine `tensor.to(device)`.

L'oggetto che l'audit legge dal pickle, `entry["state"]`, è **esattamente**
l'argomento che RLlib passa a quella funzione. Quindi:

| foglia nel checkpoint | dopo la conversione |
|---|---|
| `False` (bool Python) | tensore 0-dim dtype **bool** |
| `0.9` (float Python) | tensore 0-dim **float32** = `0.8999999761581421` |
| `1e-08` | **float32** = `9.99999993922529e-09` |
| `0` (int) | tensore 0-dim **int64** |
| `None` | `None` |
| `ndarray` float32 | tensore float32, **stessi byte** |

La normalizzazione rev2 manda un bool Python nel ramo tipato
`('prim','bool',value)` — valutato per **primo** — e un tensore 0-dim nel ramo
`('scalar', value)`. I due rami non possono mai risultare uguali: è
letteralmente il testo `prim vs scalar` delle cinque differenze booleane. I tre
float differiscono per il downcast. I momenti erano già float32 da entrambi i
lati, ed è per questo che 12 su 12 coincidevano.

**Conclusione: le 8 differenze sono artefatti del confronto fra il checkpoint
PRIMA della conversione e l'ottimizzatore vivo DOPO. Non dicono nulla sul
restore.** Il checkpoint non è dimostrato corrotto, ma nemmeno l'ottimizzatore
vivo è dimostrato byte-esatto.

### Prova indipendente, sui numeri committati da R2

Ho ricostruito lo stato vivo applicando `convert_to_torch_tensor` al checkpoint
reale e riscrivendo `lr` con il float Python `1e-4`, come fa
`_set_optimizer_learning_rate_on_learner`. I due digest whole-tree **grezzi**
della ricostruzione sono **identici a quelli che R2 ha committato**:

| | ricostruzione | committato da R2 |
|---|---|---|
| digest raw sorgente | `c1a9e152bc0a3078…` | `c1a9e152bc0a3078…` |
| digest raw vivo | `a5487d5c02f8fd9d…` | `a5487d5c02f8fd9d…` |
| **digest canonico, entrambi i lati** | **`2d0a041b2b8c60fc…`** | — |

Non è compatibilità: è **lo stesso stato**. Le due asserzioni `B02a`/`B02b`
della suite lo verificano a ogni esecuzione.

## 3. Soluzione — decisione architetturale

R9 confronta i due lati **sotto le stesse semantiche post-conversione**.

- Entrambi i lati passano attraverso **`convert_to_torch_tensor` importata dalla
  libreria installata**, mai reimplementata.
- Il `device` è quello del learner vivo (`learner._device`), non uno scelto
  dall'audit.
- Il risultato è ridotto a un albero canonico in cui **ogni foglia** è
  `('t', dtype, shape, sha256 dei byte C-order)`, rango zero incluso.
- Il confronto resta **esatto**. **Nessuna tolleranza, nessun epsilon, nessun
  confronto approssimato** — asserito staticamente sul sorgente del child, con
  le stringhe svuotate via AST così che una menzione in docstring non possa
  passare per codice.

Tenere i tensori 0-dim come tensori tipati è ciò che **restituisce** la
discriminazione che rev2 aveva dovuto cedere: un `bool` e uno zero numerico ora
differiscono per **dtype** (`bool` vs `int64`), non per nome di ramo.

### Le otto equivalenze accettate, dai valori reali del checkpoint

| percorso | checkpoint (grezzo) | vivo (grezzo) | nodo canonico |
|---|---|---|---|
| `param_groups[0].amsgrad` | `False` | `tensor(False)` | `bool`/`6e340b9c…` |
| `param_groups[0].betas[0]` | `0.9` | `tensor(0.9000)` | `float32`/`d388666e…` |
| `param_groups[0].betas[1]` | `0.999` | `tensor(0.9990)` | `float32`/`cca6554f…` |
| `param_groups[0].capturable` | `False` | `tensor(False)` | `bool`/`6e340b9c…` |
| `param_groups[0].decoupled_weight_decay` | `False` | `tensor(False)` | `bool`/`6e340b9c…` |
| `param_groups[0].differentiable` | `False` | `tensor(False)` | `bool`/`6e340b9c…` |
| `param_groups[0].eps` | `1e-08` | `tensor(1.0000e-08)` | `float32`/`b2c6a8c8…` |
| `param_groups[0].maximize` | `False` | `tensor(False)` | `bool`/`6e340b9c…` |

La lista è **chiusa a otto**. Una nona equivalenza accettata è un cambiamento
dell'oggetto sotto audit e il wrapper la registra come **problema**, non come
comodità.

### Cosa continua a fallire — fail-closed preservato

| classe di corruzione | meccanismo che la cattura | verificata |
|---|---|---|
| flip booleano | byte diversi nel tensore `bool` | ✔ |
| bool vs zero numerico | **dtype** `bool` vs `int64` | ✔ |
| beta / eps / lr oltre il float32 | byte float32 diversi | ✔ |
| chiave rimossa | insiemi di chiavi canoniche | ✔ |
| chiave aggiunta | idem | ✔ |
| `params` accorciata | lunghezza della `seq` | ✔ |
| `params` riordinata | ordine della `seq` | ✔ |
| insieme indici di stato cambiato | chiavi di `state` **+** predicato esplicito rev3 | ✔ |
| valore di un momento | sha256 dei byte | ✔ |
| **dtype** di un momento (float64) | `moments_unchanged_by_canonicalisation_*` | ✔ |
| shape di un momento | shape nel nodo | ✔ |
| `None` diventato valore | `('none',)` vs `('t',…)` | ✔ |
| `step` cambiato | sha256 dei byte | ✔ |

Il caso **float64** merita una nota esplicita e onesta: la conversione lo
declassa a float32, quindi il solo confronto canonico **non lo vedrebbe**. È
per questo che esistono i due flag `moments_unchanged_by_canonicalisation_source`
e `..._live`, che confrontano dtype/shape/byte **grezzi** di ciascun lato con i
suoi canonici e falliscono se la canonicalizzazione ha spostato un momento. Il
test `C ... è catturato da moments_unchanged_by_canonicalisation` lo dimostra su
un momento realmente allargato a float64.

Una variazione **più piccola** della risoluzione float32 non viene riportata:
non è una tolleranza, è che **la conversione di RLlib l'ha già distrutta prima
che `load_state_dict` la vedesse**. Il test lo mostra con
`float32(1e-08 + 1e-24) == float32(1e-08)`.

### Nessuna deriva silenziosa della libreria

Il wrapper fallisce closed se:

- `_set_optimizer_state` non contiene più il letterale
  `convert_to_torch_tensor(state_dict["state"], device=self._device)`;
- `convert_to_torch_tensor` non accetta più `device`;
- la conversione **non è un punto fisso** su uno dei due lati, cioè
  `canonical(convert(x)) != canonical(convert(convert(x)))`.

E registra la provenienza: versioni, path e sha256 dei due file installati,
sha256 del sorgente delle due funzioni, firma, device.

## 4. I gate a valle di R9

In R2, R3/R4/R6/R7/R12 sono stati riportati come falliti. **Nessuno di essi
aveva misurato qualcosa.** Il wrapper solleva dentro
`_reapply_optimizer_learning_rate`, che `train_ppo_mlp.py:1744` chiama **prima**
della riga 1747 che assegna `restored_training_iteration` e prima dei blocchi
che registrano `actor_freeze_audit` e `critic_state_audit`.

rev6 impone: quei cinque gate sono **valutati** solo dopo un restore vivo
completato. La condizione **non** è «il child è uscito 0 e si è fermato pulito»
— renderebbe R12 vacuo, perché R12 asserisce esattamente quelle due cose. È
invece il marcatore che l'esecuzione ha superato la riga 1744: evidenza scritta,
istante `after_restore`, wrapper non sollevato, e `optimizer_lr_audit` con voce
`after_restore` scritta **da produzione**.

**Un gate non valutato non è un gate passato**: non conta nel totale, non entra
in `gates_failed`, compare in `gates_not_evaluated` con la motivazione, e il
verdetto resta `FAIL_CLOSED`. `run_execution` richiede
`passed == total == 13` **e** `not_evaluated == []`.

Il test `I09`/`I10` rigioca questa logica **sugli artefatti reali di R2** e
conferma `completed: false`.

## 5. File creati

Tutti nuovi. **Nessun file preesistente è stato modificato.**

| file | sha256 |
|---|---|
| `…/v26c_july_replica_2026-08-26/v26c_j20_prereg_restore_audit_rev6.json` | `6575f64ccadf30e53dfc5a1d7af2966541efe6911ad37550adf1e6a2346c6615` |
| `…/v26c_july_replica_2026-08-26/v26c_j20_restore_audit_r3_child.py` | `0638af22adfb9c473b627cf5670f8014f4a1404b23a557ef324bb6775f33aa64` |
| `…/v26c_july_replica_2026-08-26/v26c_j20_restore_audit_r3.py` | `185baf3917c163e4c29564682815a63b4734d4354ddeabe3d63bec0576366069` |
| `…/v26c_july_replica_2026-08-26/test_v26c_j20_restore_audit_r3.py` | `4d58071169eefc96ec041a8d43bc0788b55efbcc7039bfc39090cd4f8c93625f` |
| `…/v26c_july_replica_2026-08-26/v26c_j20_restore_audit_r3_go_DRAFT.json` | `dce5a521fb21a5e192d341aff0fbc977da87cf71b8a3b0e81bc522399fe2d908` |
| `reports/user/2026-08-28_v26c_j20_restore_audit_r3_preparazione.md` | questo report |

Il runner R3 è una **copia meccanica** del runner R2: **7 funzioni differiscono**
(`live_restore_completed` nuova; `preflight`, `go_pin_targets`,
`expected_pin_hashes`, `check_entry_evidence`, `evaluate_gates`,
`run_execution` modificate) e **20 sono byte-identiche**, fra cui
`restore_command`, `hermetic_restore`, `verify_commit`, `validate_go`,
`child_environment`, `launch_once`, `check_pins`, `check_destination`,
`read_child_artefacts`. La suite lo prova funzione per funzione via AST.

### File esplicitamente NON toccati, ri-hashati e invariati

| artefatto | sha256 |
|---|---|
| wrapper congelato `v26c_j20_restore_audit_child.py` | `38a57b25da27666a99891511666d63cac07534aa7602f20469048012f536caf4` |
| runner R2 | `e5776700f9e12c8632d239898b673eba0f0cab6e08905001a9778bc12b75e81c` |
| suite R2 | `9f0f69f1695ea64404587d6274079317925a877a729df6f589b05a8c5faabae8` |
| GO R2 | `e5c01b3f6d65842cbf184aee5e6a5cdd1dcda38c600b4402f2a3d9a1222d3d2a` |
| runner R1 | `322b9abc0a2feae7b2e77f7a51946ae19de34e7013901b6e9425e6ee7c5210ae` |
| suite R1 | `07213d4d13c2ffb0965673ec7960ce8db3153fb9c3393006a77d0aaba0530860` |
| GO R1 | `29775d4265227a5ef3bfc0fca59313f1ffea56d9c1e34b0236440d2638c1c18a` |
| base, rev1, rev2, rev3, rev4, rev5 | `1326944e…`, `3fa1ae23…`, `59e2e5e0…`, `bfb6ef9b…`, `71a6118f…`, `3d732153…` |

Le **due foglie** R1 (8 file) e R2 (10 file) sono state ri-hashate una per una:
**tutte identiche**. Nessun file di produzione, configurazione, checkpoint,
attore, critic, sigma, sorgente ottimizzatore, env, reward, C++ o SEA è stato
toccato. `train_ppo_mlp.py` re-hasha al suo pin.

## 6. Comando e contenimento

Il comando futuro deriva dallo stesso `sealed_command()` con le **stesse quattro
operazioni**. I tre conteggi sono invariati: **20** token derivati, **18**
delegati a `train_ppo_mlp`, **25** nel child argv completo. Il comando
*derivato* è **identico** a quello di R2; l'unica differenza nel child argv è il
token dello script — l'operazione 4 sostituisce il wrapper R3 al posto di quello
congelato (`J18`, `J19`, `J20`).

- `--iteration-start` **non** viene passato: `iteration_start` resta derivato
  come `restored_logical_iteration + 1 = 2` contro un target di 1, quindi
  `range(2, 2)` è vuoto → **zero iterazioni, zero sampling, zero rollout**.
- config critic-only invariata; timeout 600 s invariato.
- destinazione no-clobber, marker `TECHNICAL_INVALID` scritto per primo e
  rimosso per ultimo, **un solo `subprocess.Popen`**, nessun retry, nessun
  supervisore.
- l'esecuzione futura userà `train_ppo_mlp.run` e `algo.restore_from_path`
  esattamente come R2: il wrapper non emula il restore, non costruisce un
  `Algorithm`, non chiama `ray.init`, non nomina `restore_from_path` nel
  codice.

## 7. Test e verifiche eseguiti — risultati esatti

Solo test statici/unitari/preflight. **Nessun `ray.init`, nessun `Algorithm`,
nessun environment, nessun training, sampling, rollout o PPO.**

| comando | risultato |
|---|---|
| `python test_v26c_j20_restore_audit_r3.py` | **297/297 check passati** |
| `python v26c_j20_restore_audit_r3.py --preflight-only` | **READY, 91/91 pin** |
| `python test_v26c_j20_restore_audit_r2.py` | **219/223** — 4 fallimenti, tutti «destinazione R2 assente» |
| `python test_v26c_j20_restore_audit.py` | **264/266** — 2 fallimenti, i due già annotati in rev5 |
| `validate_go` sul DRAFT | **`valid: false`, rifiutato su entrambi i conteggi** |

Tutti eseguiti come:

```
cd "Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26"
env PYTHONDONTWRITEBYTECODE=1 /opt/anaconda3/envs/envCMC-rllib/bin/python <file>
```

### Copertura della suite R3 (297 check)

- **A** copia meccanica: 7 funzioni differiscono, 20 byte-identiche, 26 → 27
  funzioni, wrapper congelato distinto e al suo pin.
- **B** le otto equivalenze accettate, **una per una**, sui valori reali del
  checkpoint; più i due digest raw di R2 riprodotti byte per byte.
- **C** ogni classe di corruzione dell'elenco sopra, più i momenti byte-esatti
  12 su 12 e il caso float64.
- **D** nessuna tolleranza (scansione su **solo codice**, stringhe svuotate via
  AST), nessun `ray.init`, nessun `Algorithm`, punto fisso su entrambi i lati,
  provenienza della libreria, 0-dim non promosso a shape `(1,)`, chiavi int/str
  non collidenti, tipo ignoto → `opaque`.
- **E** R1 e R2 intatti: ogni file di entrambe le foglie al suo pin, entrambi
  ancora `FAIL_CLOSED` sotto `TECHNICAL_INVALID`, entrambi promuovono `NONE`.
- **F** foglia nuova, born-invalid, **un solo** `subprocess.Popen`, nessun loop
  di retry.
- **G** rev6 additiva e sigillata, i sei predecessori invariati, DRAFT
  doppiamente inerte e rifiutato su entrambi i conteggi, 91 pin tutti correnti e
  nessuno fuori scope, GO di altro stadio rifiutato, `approved`/`ok` rifiutati.
- **H** 13 gate, tutti non vacui, **21 mutazioni distinte su R9** tutte rosse.
- **I** disciplina a valle, inclusa la rigiocata sugli artefatti reali di R2.
- **J** preflight, conteggi token, argv delegato identico a R2.
- **K** contratto del wrapper: originale chiamato per primo, cattura subito
  dopo, evidenza scritta **prima** del raise, mai un warning, un solo attributo
  ribindato, un solo file scritto.

### I 4 + 2 fallimenti delle suite predecessori

Sono **stato atteso post-esecuzione**, non regressioni: ogni check asserisce che
la foglia di destinazione del **proprio** stadio non esista, cosa vera solo
prima che quello stadio girasse. R2 fallisce inoltre il proprio preflight per la
regola **no-clobber** — che è la regola che funziona, non che fallisce. rev5
aveva già annotato la coppia R1; rev6 annota il quartetto R2 con la stessa
classificazione. **Entrambe le suite restano byte-identiche e pinnate**, e la
suite R3 verifica che non si siano mosse.

## 8. Stato del GO

**Esiste solo il DRAFT**, `v26c_j20_restore_audit_r3_go_DRAFT.json`,
doppiamente inerte: `status: "DRAFT"` **e** `authorises_execution: false`.
`validate_go` lo rifiuta su **entrambi** i conteggi. Tutti e sei i flag proibiti
sono `false`. Pinna **91** artefatti, esattamente quelli della mappa chiusa del
runner, tutti correnti.

**Non mi sono auto-autorizzato nulla.** L'esecuzione richiede **entrambe**:
un'autorizzazione esplicita dell'utente **e** un GO operativo dell'architetto,
file separato, `status` esattamente `APPROVED`, stadio esattamente
`V26C_J20_RESTORE_AUDIT_R3`.

## 9. Preoccupazioni non risolte

1. **Il verdetto di R9 resta ignoto.** Questa fase corregge il criterio; non
   dimostra che l'ottimizzatore vivo sia byte-esatto. Solo l'esecuzione può
   dirlo, e può ancora dire di no.
2. **La ricostruzione dello stato vivo è una ricostruzione.** I due digest raw
   coincidono con quelli committati da R2, il che è forte, ma resta un
   argomento su un artefatto: nella suite il lato vivo è costruito applicando
   la conversione al checkpoint, non letto da un learner.
3. **`lr` nell'albero principale** continua a confrontare il checkpoint con il
   valore che lo stadio riapplica, perché rev1 impone che l'originale giri per
   primo. R3 **aggiunge** la misura del valore pre-sovrascrittura
   (`learning_rate_observation`), ma non cambia l'ordine.
4. **La lista chiusa a otto è deliberatamente rigida.** Una nona equivalenza
   benigna farebbe fallire l'audit. È la scelta fail-closed della casa; se
   l'architetto la ritiene troppo stretta, va deciso **prima** dell'esecuzione.
5. **Il caso float64 sui momenti** non è catturato dal cammino canonico ma dai
   due flag `moments_unchanged_by_canonicalisation_*`. È documentato e testato,
   ma è un meccanismo diverso dagli altri e va letto come tale.
6. `--iterations 1` con `iteration_start = 2` resta l'unico meccanismo che rende
   il loop vuoto. Invariato da R1, ma è una proprietà derivata, non asserita.

## 10. TODO

- [ ] **G9 resta aperto.** La foglia del warm-up conserva `RESTORE_AUDIT_PENDING`;
      `training_ready` resta `false`.
- [ ] Decisione dell'architetto su rev6 e sul DRAFT GO; senza GO `APPROVED` **e**
      autorizzazione esplicita dell'utente, R3 non parte.
- [ ] Decidere sui due `gcs_server` orfani del 18/08, tuttora **non toccati**.
- [ ] I 2 check `B06`/`I07` della suite R1 e i 4 `C29`/`E01`/`H02`/`H04` della
      suite R2 restano rossi per assenza-destinazione post-esecuzione:
      **annotati in rev5 e rev6, non corretti**, file pinnati e invariati.
- [ ] TODO ereditati ancora aperti: generalizzazione multimodello (epic del
      22/08); chiusura del gate finale di recupero AB06.

---

**Stato conclusivo:** pacchetto R3 additivo preparato e verificato,
**297/297** check e preflight **READY 91/91**. R1, R2, il wrapper congelato, le
sei preregistrazioni precedenti, il checkpoint e ogni file di produzione sono
invariati e ri-hashati. **Nessuna esecuzione, solo un GO DRAFT inerte. G9 resta
aperto e nulla è promosso.**
