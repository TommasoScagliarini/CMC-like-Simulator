# V26C — J20 R3: correzione rev7, il learning rate ripristinato diventa un gate

**Data:** 2026-08-28
**Stadio:** `V26C_J20_RESTORE_AUDIT_R3`
**Fase:** **CORREZIONE DELLA PREPARAZIONE — NIENTE È STATO ESEGUITO**
**Foglia di destinazione:** `…/j20_runs/j20_restore_audit_v26c_2026-08-28_r3/` — **assente**
**GO:** **solo DRAFT, doppiamente inerte** — `status: "DRAFT"`, `authorises_execution: false`
**G9:** **resta aperto**, `training_ready: false`, `promotion: NONE`

---

## 1. Valutazione indipendente del rilievo dell'architetto

**Il rilievo è corretto ed è vincolante. Concordo, e l'ho riprodotto.**

Nel pacchetto rev6:

- `compare()` calcolava `exact` alla propria `return`;
- `learning_rate_observation()` veniva chiamata **dopo**, dentro `wrapped()`, e
  il suo risultato veniva soltanto **assegnato** nel verdetto;
- nessuna clausola di `compare()` leggeva quel risultato, quindi un `matches`
  falso non poteva influenzare `exact`;
- il predicato R9 del runner citava `learning_rate_observation` in **un solo
  punto**: `measurements["live_learning_rate_observation"]`, che è un valore
  registrato, non un gate.

E `train_ppo_mlp.py:1744` chiama `_reapply_optimizer_learning_rate` **prima**
della cattura; `_set_optimizer_learning_rate_on_learner` scrive
`float(learning_rate)` in **ogni** param_group. Quindi `param_groups[0]["lr"]`
sul lato vivo è il valore **configurato dallo stadio**, qualunque cosa il
restore abbia caricato.

### Riproduzione misurata, prima di toccare il codice

Sul checkpoint reale, con `lr` vivo sovrascritto a `1e-4` e il report di
produzione che registra `before = 5e-4`:

```
canonical differences with a WRONG restored lr, after the overwrite: []
learning_rate_observation.matches = False | before = 0.0005
R9 with matches=False -> True
gates passed -> 13 / 13
```

Un learning rate ripristinato **cinque volte sbagliato** passava **13 su 13**.
La classe di corruzione «lr cambiato» non era coperta affatto, ed è una classe
che **rev6 stesso elenca** fra quelle che devono essere catturate. Difetto di
contratto, non solo di codice.

**Ciò che NON è dimostrato:** che il restore produca un lr sbagliato. R2 ha
committato `before = 9.999999747378752e-05`, che è `float32(1e-4)` e coincide
canonicamente con l'`1e-4` del checkpoint. Il difetto è che un valore sbagliato
**non sarebbe stato visto**, non che ne sia stato osservato uno.

## 2. Correzione, fail-closed

### Nel wrapper

Nuova funzione `enforce_learning_rate_observation(verdict, source_state,
reports, learning_rate)`, chiamata da `wrapped()` **prima** di
`write_evidence()` e **prima** del `raise`:

1. calcola l'osservazione con le **stesse** semantiche canoniche di tutto il
   resto (`convert_to_torch_tensor` di RLlib, poi dtype/shape/sha256 dei byte);
2. la allega al verdetto;
3. se `matches` **non è esattamente `True`**, appende un **problema esplicito**
   che nomina il learning rate e i due valori, e imposta **`exact = False`**;
4. l'evidenza viene scritta comunque, poi il percorso di `raise` già esistente
   scatta perché `exact` è falso.

**L'ordine di produzione non è cambiato**: l'originale gira ancora per primo,
come impone rev1. È cambiato soltanto che l'audit non accetta più la conseguenza
in silenzio.

Casi fail-closed: nessun report per `default_optimizer`; nessun `lr` nel
checkpoint; un valore che non si canonicalizza; due nodi canonici diversi.
Nessuna tolleranza, nessun epsilon, nessun confronto approssimato.

L'osservazione ora emette anche `canonical_dtype`, `gated`,
`live_lr_before_reapply_type`, `production_reports_seen`, e restituisce **`null`
JSON** — non la stringa `"None"` — quando un campo canonico è assente, così che
un predicato possa distinguere un valore dalla sua assenza.

### Nel runner

Al predicato R9 sono state aggiunte, **senza togliere nulla**:

```
isinstance(evidence["learning_rate_observation"], dict)
matches is True
gated is True
canonical_source_lr è una stringa che inizia con "('t'"
canonical_live_lr_before_reapply == canonical_source_lr
live_lr_before_reapply is not None
canonical_dtype == "float32"
```

`matches` da solo è un booleano che il payload calcola. Le clausole canoniche
applicano il principio di rev3: il predicato deve pretendere l'**evidenza**, non
la conclusione.

Il gate count resta **13**. Nessuna funzione nuova nel runner: la correzione
atterra dentro `evaluate_gates`, che già differiva da quella di R2. Il delta
resta **7 funzioni** (`live_restore_completed` nuova; `preflight`,
`go_pin_targets`, `expected_pin_hashes`, `check_entry_evidence`,
`evaluate_gates`, `run_execution` modificate), **20 byte-identiche**.

### Pin

`PIN_R3_CHILD` è ora una costante del runner e `expected_pin_hashes()` la
restituisce per il wrapper: sotto rev6 il wrapper era pinnato **solo** dal GO,
ora anche dal runner. Il conteggio pin passa da **91 a 92** (rev7 aggiunto).

Il wrapper R3 poteva essere modificato perché **non ha mai eseguito**: nessuna
foglia, nessuna receipt e nessun result lo nominano, e l'unico GO che lo citava
era il DRAFT inerte. Correggerlo non ha distrutto evidenza.

## 3. File creati o modificati

| file | stato | sha256 |
|---|---|---|
| `v26c_j20_prereg_restore_audit_rev7.json` | **creato** | `32238dafb13b25e90428538302d197c421acd69cf67f8884ab4622b3c27c38d8` |
| `v26c_j20_restore_audit_r3_go_DRAFT_rev7.json` | **creato** | `07a5bc4302f7618857511bd0105771de5025df8d6909ac87e8d652ad91d07943` |
| `v26c_j20_restore_audit_r3_child.py` | **modificato** | `db4afc547b4d4194ba828dadb76801ff1877659c1cec8e5d749b2e0ba94decdd` |
| `v26c_j20_restore_audit_r3.py` | **modificato** | `23df351e303ef9b28c89576d2134382af282db7e2a38ebb5b9b0df552ae624a7` |
| `test_v26c_j20_restore_audit_r3.py` | **modificato** | `8ac2248aad76245ea4fe40544f924c4fb10405d76d53d414ec60e05cd858a4b4` |
| `reports/user/2026-08-28_v26c_j20_restore_audit_r3_correzione_rev7_gate_learning_rate.md` | **creato** | questo report |

Tutti sotto `Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/`.

### Artefatti storici preservati, mai modificati

| artefatto | sha256 | stato |
|---|---|---|
| `v26c_j20_prereg_restore_audit_rev6.json` | `6575f64ccadf30e53dfc5a1d7af2966541efe6911ad37550adf1e6a2346c6615` | sigillato, invariato |
| `v26c_j20_restore_audit_r3_go_DRAFT.json` | `dce5a521fb21a5e192d341aff0fbc977da87cf71b8a3b0e81bc522399fe2d908` | **superato**, inerte, invariato |
| wrapper congelato `v26c_j20_restore_audit_child.py` | `38a57b25da27666a99891511666d63cac07534aa7602f20469048012f536caf4` | invariato |
| runner/suite/GO R1 e R2, base e rev1–rev5, foglie R1 e R2 | ai loro pin | invariati |

Il DRAFT rev6 è ora rifiutato da `validate_go` con **6 motivi**: status,
`authorises_execution`, e quattro pin diventati stale per costruzione (wrapper,
runner, suite, rev7 mancante). È preservato **proprio perché** è la traccia di
ciò che l'architetto ha rivisto.

Nessun file di produzione, configurazione, checkpoint, attore, critic, sigma,
sorgente ottimizzatore, env, reward, C++ o SEA è stato toccato.

## 4. Test e verifiche — risultati esatti

Solo test statici/unitari/preflight. **Nessun `ray.init`, nessun `Algorithm`,
nessun environment, nessun training, sampling, rollout o PPO. R3 non è stato
eseguito.**

| comando | risultato |
|---|---|
| `python test_v26c_j20_restore_audit_r3.py` | **344/344 check passati** (erano 297 sotto rev6) |
| `python v26c_j20_restore_audit_r3.py --preflight-only` | **READY, 92/92 pin**, destinazione assente |
| `python test_v26c_j20_restore_audit_r2.py` | **219/223** — invariato |
| `python test_v26c_j20_restore_audit.py` | **264/266** — invariato |
| `validate_go` sul DRAFT rev7 | `valid: false`, rifiutato su entrambi i conteggi |
| `validate_go` sul DRAFT rev6 storico | `valid: false`, **6** rifiuti |

Tutti eseguiti come:

```
cd "Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26"
env PYTHONDONTWRITEBYTECODE=1 /opt/anaconda3/envs/envCMC-rllib/bin/python <file>
```

### I 47 check nuovi

**D20–D24 — il caso reale passa.** Il checkpoint porta `lr` float Python `1e-4`;
il valore `before` che **R2 ha realmente committato**, `9.999999747378752e-05`,
coincide canonicamente; `exact` resta vero e nessun problema viene appeso; i
campi canonici sono popolati e coerenti, dtype `float32`. `D24` usa
direttamente i `production_lr_reports` committati nella foglia R2.

**D25 — un lr non corrispondente rende il verdetto del child non-exact.**
Quattro valori: `5e-4`, `1e-3`, `9e-5` (vicino ma oltre il float32), `0.0`
(ottimizzatore fresco). Tutti → `matches False`, `exact False`, problema che
nomina il learning rate.

**D26 — nessun report, report vuoto, report di un altro ottimizzatore, report
senza `before`.** Tutti falliscono closed con campo canonico `null`.

**D27–D29 — LA SCENA DI MASCHERAMENTO, riprodotta.** Con `lr` vivo riscritto al
valore configurato, il cammino canonico riporta **zero differenze** (`D27`), e
l'audit fallisce comunque perché il `before` era `5e-4` (`D28`), con un problema
che nomina sia il learning rate sia `0.0005` (`D29`).

**D30–D33 — l'ordinamento.** L'enforcement precede `write_evidence` e precede il
`raise`; il wrapper non assegna più l'osservazione senza gatearla; il simbolo
compare **esattamente due volte nel codice** (definizione e unica chiamata),
scansione su solo-codice con le stringhe svuotate via AST.

**D34–D37 — end-to-end sul runner.** L'evidenza che il child corretto
emetterebbe per la scena di mascheramento fallisce R9, l'audit non può
raggiungere 13/13, e il cammino canonico non riportava nulla — che è esattamente
perché il lr andava gateato a parte. `D37` mostra che anche un'evidenza in stile
rev6 — `exact` vero, nessun problema — viene **comunque** rifiutata, perché rev7
pretende `matches` in proprio.

**H05 ×9 — mutazioni dell'evidenza che ribaltano R9:** `matches` falso;
osservazione assente; osservazione non-dict; campo canonico `null`; la stringa
`"None"` al posto di un nodo canonico; campi canonici discordi;
`live_lr_before_reapply` mancante; dtype canonico sbagliato; `gated` falso.

**G05a–G05l** — rev7 additiva e coerente col runner, rev6 e il DRAFT storico
preservati ai loro hash, il DRAFT storico ancora inerte e ora rifiutato anche
per staleness, i due DRAFT file distinti.

**A13–A14** — precedenza `rev7 > rev6 > … > base`; il wrapper R3 pinnato per
hash nel runner oltre che nel GO.

## 5. Secondo audit: altri campi registrati ma non gateati

Ho confrontato **meccanicamente** ogni chiave che il wrapper scrive nell'evidenza
contro il predicato R9: **33 gateate, 26 no**.

**L'unico difetto di questa classe era `learning_rate_observation`**, corretto
qui. Le altre 26 si dividono in:

- **provenienza richiesta da rev6 di essere *registrata*, e le cui affermazioni
  sono gateate tramite booleani derivati**: `accepted_equivalences` (dettaglio),
  `canonical_node_*`, `raw_detail`, `raw_*_leaf`, `difference_records`,
  `raw_leaf_values_*`, `raw_moment_digests_*`, `semantics`, `method`,
  `note_on_moment_dtype`, `source_state_pkl`, `device`, `production_lr_reports`,
  `reapplied_learning_rate`;
- **aspettative riecheggiate**, di cui il runner ha già le proprie costanti:
  `expected_accepted_equivalence_paths`, `expected_optimizer_name`,
  `expected_param_count`, `expected_state_indices`;
- **chiavi interne** non emesse al livello superiore: `param_groups_digest_*`,
  `path`.

### Tre osservazioni segnalate, NON implementate

Non sono difetti dello stesso tipo e implementarle sarebbe inventare requisiti
nuovi. Le registro per la decisione dell'architetto, e sono scritte anche in
rev7 §`correction_6`.

1. **`conversion.downcast_clause_present` e
   `conversion.none_passthrough_clause_present`** sono registrate ma non gateate
   (verificato meccanicamente). rev6 nomina esattamente **tre** guardie di
   deriva in `fail_closed_on_drift` — letterale del call-site, parametro
   `device`, punto fisso — e **tutte e tre sono gateate**. Queste due sono
   corroborazione informativa che rev6 non ha mai promesso di imporre.
2. **`raw.reproduces_r2_eight`** è un booleano calcolato dal payload. Una forma
   più stretta confronterebbe `raw.differences` direttamente con le otto
   stringhe committate da R2 dentro il runner. È il principio di non-vacuità di
   rev3 spinto un passo oltre; le due clausole adiacenti già gateate
   (`raw.difference_count == 8` e la lista chiusa dei percorsi accettati)
   rendono rilevabile un payload che non ha calcolato nulla.
3. **`optimizer_types`** (`["Adam"]`) è registrato ma non gateato. Il **nome**
   dell'ottimizzatore lo è, e R13 stabilisce indipendentemente il comportamento
   Adam ricostruendolo fuori processo.

## 6. Conseguenza architetturale: `training_ready` resta falso — annotata, non implementata

Un R3 che passa imposta `promotion: NONE`, `training_ready: false`,
`next_stage_authorized: false`. È **deliberato** e identico nel documento base,
in rev1, rev4, rev6 e rev7. **Chiudere G9 non rende la pipeline
training-ready.**

Lo stato finale desiderato — `training_ready` **vero** fermandosi comunque prima
di ogni addestramento — richiede una **attestazione di aggregazione delle
evidenze separata**, che pesi i gate chiusi e ne tragga la conclusione.

**Non può essere pre-committata adesso**, e non l'ho fatto: dovrebbe pinnare il
`v26c_j20_restore_audit_result.json`, la receipt e `live_optimizer_audit.json`
della foglia R3, e **nessuno dei tre esiste**, perché R3 non ha girato. Pinnare
hash inesistenti sarebbe fabbricazione.

Registrato in rev7 sotto `note_on_training_readiness` come **RECORDED FOR THE
ARCHITECT — NOT designed, NOT preregistered, NOT implemented**, e nel DRAFT
rev7. Spetta a Codex progettarla **dopo** un PASS reale di R3.

## 7. Preoccupazioni non risolte

1. **Il verdetto di R9 resta ignoto.** Questa correzione chiude un buco nel
   criterio; non dice nulla su cosa il restore produrrà.
2. **Il lato vivo nella suite è ricostruito**, non letto da un learner. I due
   digest raw coincidono con quelli committati da R2, il che è forte, ma resta
   un argomento su un artefatto. `D24` mitiga usando i report reali di R2.
3. **Il gate lr dipende da un campo che produce produzione**, `before`. Se una
   versione futura di `_set_optimizer_learning_rate_on_learner` smettesse di
   riportarlo, il gate fallirebbe closed — corretto, ma per assenza di
   strumento, non per corruzione.
4. Le tre osservazioni del §5 restano aperte per decisione dell'architetto.
5. Restano valide le preoccupazioni 1, 4, 5 e 6 del report di preparazione del
   28/08 (lista chiusa a otto deliberatamente rigida; float64 catturato da un
   meccanismo diverso; `range(2,2)` proprietà derivata).

## 8. TODO

- [ ] **G9 resta aperto.** La foglia del warm-up conserva `RESTORE_AUDIT_PENDING`;
      `training_ready` resta `false`.
- [ ] Decisione dell'architetto su rev7 e sul DRAFT rev7; senza GO `APPROVED`
      **e** autorizzazione esplicita dell'utente, R3 non parte.
- [ ] Decisione sulle **tre osservazioni** del §5, segnalate e non implementate.
- [ ] **Dopo** un PASS reale di R3: progettare l'attestazione di aggregazione
      delle evidenze che possa portare `training_ready` a vero senza addestrare.
      Non pre-committabile ora.
- [ ] Decidere sui due `gcs_server` orfani del 18/08, tuttora **non toccati**.
- [ ] I 2 check `B06`/`I07` della suite R1 e i 4 `C29`/`E01`/`H02`/`H04` della
      suite R2 restano rossi per assenza-destinazione post-esecuzione:
      **annotati in rev5 e rev6, non corretti**, file pinnati e invariati.
- [ ] TODO ereditati ancora aperti: generalizzazione multimodello (epic del
      22/08); chiusura del gate finale di recupero AB06.

---

**Stato conclusivo:** rilievo dell'architetto **confermato e riprodotto**,
corretto fail-closed con rev7. Suite **344/344**, preflight **READY 92/92**.
rev6, il DRAFT rev6, il wrapper congelato, R1, R2 e ogni artefatto precedente
sono preservati e ri-hashati. **Nessuna esecuzione, solo un GO DRAFT inerte.
G9 resta aperto e nulla è promosso.**
