# V26C J19A — Readiness della riproduzione singola

**Data**: 2026-08-27
**Stadio**: `V26C_J19A_SINGLE_REPRODUCTION` — **readiness**
**Esecutore**: Opus 5, effort xhigh

**Nessun fit, nessun rollout, nessun environment, nessun critic, nessun PPO, nessuna collection,
nessun actor, nessuna `j19a_runs`, nessun seme sigillato, nessun GO.**
Eseguiti solo test statici e dry-run che non addestrano e non simulano.
L'ermeticita' del preflight e' **self-contained**: verificata invocando il runner **senza**
`PYTHONDONTWRITEBYTECODE`, con snapshot prima/dopo: **nessun file creato o modificato**.

**J16 e ogni runner storico non sono stati modificati** e restano evidenza immutabile.

---

## 1. Separazione delle fasi

Per decisione dell'architetto, fit e rollout **non sono concatenati nella stessa esecuzione**: ogni
fase chiude con report e audit.

| fase | contenuto | stato |
|---|---|---|
| **J19A** | rifit singolo del candidato 13 → riproducibilità → eligibilità offline → commit di **un solo** actor in **un solo** leaf | **implementata e testata, NON eseguita** |
| **J19B** | closed-loop A–F vincolante | **descritta, non eseguibile, non avviata** — sarà preregistrata e pinnata **solo dopo** il tuo audit del leaf J19A |

Il runner J19A **non può avviare** J19B: nessuna chiamata a macchinario di rollout, cella o matrice,
verificato strutturalmente via AST (S11a/b/c).

---

## 2. Le sette decisioni, codificate nel contratto

| id | decisione | dove |
|---|---|---|
| D1 | G1/G2 max-drift 0.005 restano **diagnostici, mai binding** | `diagnostic_only_never_binding` |
| D2 | G3/G4 usano il **bias empirico assoluto di J8**, precisione piena, stessi blocchi C/D, stesso parent J2 — criterio *non-worse-than-operational-J8*, non derivato da sigma | `binding` G3/G4 |
| D3 | E-C/E-D binding ai valori J8 esatti | `binding` E-C/E-D |
| D4 | POST-485 calcolato e registrato come **diagnostica non binding** | `diagnostic_only_never_binding` |
| D5 | max-drift resta terza chiave solo come tie-break diagnostico fra eleggibili; **ora irrilevante**, l'eleggibile è unico | annotato |
| D6 | riproducibilità: `best_epoch` esattamente 191, step/batch/seed/config/dataset/provenance esatti; sei metriche con `math.isclose(rel_tol=1e-06, abs_tol=1e-09)` — **tolleranza numerica, non gate prestazionale**; fallimento ⇒ fail-closed, nessun actor, nessun rollout | `reproducibility_contract` |
| D7 | closed-loop A–F userà **solo** 20 mm soft, 25 mm July diagnostica, **> 28 mm unica barra hard binding, 28 esatto passa**, e ogni altro criterio congelato J9R1 | registrato per J19B |

### I due ceiling binding di J8, precisione piena

Ricalcolati dal runner a ogni preflight e **asseriti** contro il contratto, mai solo creduti:

| blocco | n | knee signed mean shift | ankle signed mean shift | **ceiling = max(\|knee\|,\|ankle\|)** |
|---|---|---|---|---|
| C | 2497 | `4.620137560220626e-05` | `-0.0008992143952043874` | **`0.0008992143952043874`** |
| D | 1210 | `0.0006089726265389337` | `-0.0007264156156278919` | **`0.0007264156156278919`** |

RMSE, invariati: **E-C `0.00447396874790294`**, **E-D `0.005262698258768814`**.

Il candidato 13 li rispetta con margine: **bias C 0.433×**, **bias D 0.550×**. La soglia `0.0014557`
**non compare da nessuna parte nel runner** (S33).

---

## 3. Una scelta architetturale che dichiaro, non nascondo

**J19A importa il runner J18 congelato e ne riusa il percorso di training immutato, pinnato per
hash.** È l'**inverso** della regola di ermeticità imposta a J18, e la ragione è precisa: J18 doveva
non importare nulla perché il suo comportamento dovesse discendere dai sette artefatti pinnati dal
suo GO; lo scopo di J19A è l'opposto — **riprodurre** un risultato J18. Con un training loop
ritrascritto, un mismatch sarebbe **ambiguo**: irriproducibilità genuina ed errore di trascrizione
sarebbero indistinguibili.

L'import è verificato per hash **prima** dell'uso e `load_j18` **rifiuta** un runner J18 alterato,
con messaggio esplicito (S15). È dichiarato nel contratto sotto
`training_code_provenance.declared_for_architect_ruling`, per la tua pronuncia.

---

## 4. Nota misurata per il tuo futuro audit di J19B

Non implemento J19B, ma registro i fatti che ne condizionano il disegno:

- il runner J16 è **cablato su J15R1** (`THE ACTOR IS J15R1, AND ONLY J15R1`), **1737 righe**, 71
  riferimenti actor-specifici;
- l'accoppiamento all'actor è però concentrato in **due sole costanti** — `J15R1_LEAF` e
  `J15R1_MODULE_DIR` — usate in 13 punti;
- J16 **delega ogni soglia di penetrazione** a `v26c_penetration_contract` (`9257e9b8…`), che
  implementa **già** il contratto corrente: `> 0.020` soft, `>= 0.025` July, unica barra binding
  `> 0.028`, 28 esatto passa;
- J16 importa a sua volta J9R1 e J1 per provare campo per campo di essere J12 col solo actor
  cambiato: gli import di fase precedente sono il **pattern consolidato** di questa famiglia.

Riscrivere a mano un fork di 1737 righe di un percorso di sicurezza introdurrebbe divergenza
silenziosa; ritrascrivere localmente le soglie sarebbe peggio. **Non ho toccato J16** né alcun
runner storico.

---

## 5. Test

**J19A: 127/127 PASS.** Preflight **57/57**, inerte.

> **Da non confondere con il 248/249 della suite storica J18.** Quel singolo fallimento è
> `E54 no GO file exists in the validation root`: un'invariante di *readiness*, scritta prima che
> l'architetto emettesse il GO J18, e **legittimamente falsa** ora che il GO esiste ed è stato usato.
> Il file di test J18 è pinnato dal GO J18 (`6050f4c0…`, verificato invariato) e fa parte del record
> congelato dell'esecuzione: **non è stato modificato**, per decisione dell'architetto. Non è un
> difetto del codice, è una data di scadenza su un'asserzione.

| gruppo | cosa morde |
|---|---|
| S01–S08 | chiusura del namespace; il preflight lascia l'albero **byte per byte identico**, non addestra, non scrive, non crea directory, **non esegue rollout**, non importa torch |
| S09–S12 | nessun import di environment o closed-loop; import esattamente l'insieme atteso; **nessuna chiamata** a macchinario di rollout/cella/matrice; ogni menzione di «rollout» **nega** la sua presenza, verificato su finestra di due righe |
| S13–S16 | il runner J18 e i cinque artefatti del leaf J18 corrispondono ai pin; `load_j18` **rifiuta** byte alterati; nessun runner storico citato |
| S17–S20 | le sei metriche congelate sono **lette dal leaf pinnato, non hardcodate**; candidato 13, epoca 191, 6600 step su 33 batch |
| S21–S27 | riproduzione identica passa; 5e-07 relativo passa, **5e-05 fallisce**; **un risultato MIGLIORE fallisce** — prova che non è un gate prestazionale; mismatch esatto su epoca/step/batch fallisce |
| S28–S40 | i ceiling J8 ricalcolati coincidono col contratto; `0.0014557` assente dal runner; G1/G2/POST-485 dichiarati diagnostici; **nessun criterio binding è un max-drift**; c13 dentro i ceiling; **il suo max drift eccede il riferimento diagnostico e ciò non blocca più** |
| S41–S51 | GO valido in memoria; **un GO che autorizza un rollout è RIFIUTATO**; stadio errato, pin mancante, hash stantio, file assente rifiutati; nessun GO su disco; l'actor è scritto **solo** dentro la guardia `promote` |
| S52–S58 | il contratto parse, senza self-hash; tutte e sette le decisioni codificate; separazione delle fasi; bande di penetrazione esatte; inversione dell'import dichiarata; runner storici dichiarati immutabili |
| **S59–S72** | la comparazione esatta copre seed, protocollo, **tutti e otto** i campi candidate/config, **tutti e quattro** i conteggi di riga, **tutte e quattro** le coppie di hash observations/labels, byte e digest del parent, e **sette** campi di provenance; **un solo campo alterato (`w_C` 30→29) la fa fallire**; la distinzione storica è annotata sul campo `labels_sha256.D` e **assente** sui blocchi letti da file |
| **S73–S80** | `evaluate_eligibility` **rifiuta di girare** senza `g11_payload` **e rifiuta un payload cui manchi una qualsiasi delle sei famiglie**, nominandola; **G11 fallisce** su NaN/Inf in **ciascuna** delle sei famiglie — `candidate`, `history`, `post_485`, `best_objective`, `scale_absorption`, `j8_ceilings`, `raw_vs_scaled_equivalence`, `reproducibility`, `exact_fields` — e in ogni caso l'eligibilità cade; POST-485 e la comparazione esatta sono calcolati **prima** del gate; `promote` richiede anche la comparazione esatta ed è **impossibile** con findings non finiti |
| **S85–S93** | `finalise` sanitizza l'intero `g11_payload` in **un solo passaggio**; `history.json` è costruita dalla history **sanitizzata** e `trained["history"]` **non compare** dentro `finalise`; nessun oggetto grezzo non sanitizzato raggiunge il result né il receipt, verificato via AST sui nomi; ogni campo numerico o booleano del receipt è una **derivazione** |
| **S81–S84** | `sys.dont_write_bytecode` è impostato a **livello di modulo**, **prima** che l'import dinamico di J18 possa avvenire, non dentro una funzione |

---

## 6. Artefatti

| artefatto | SHA-256 |
|---|---|
| `v26c_j19a_prereg_single_reproduction.json` | `ece9f2cd4a6b1247c45588222406427cd68b2f22f3967a563f504d41ce5229b1` |
| `v26c_j19a_single_reproduction.py` | `d33b167306e328e085cbefb4bcd20985569461a0533ccc9fc4e8209da81b4e39` |
| `test_v26c_j19a_single_reproduction.py` | `3af7a9dd56281ddcc90d201e312da1a2290bae01f987ed337752d6a89a08233f` |

Un GO J19A dovrà pinnare **questi tre più il runner J18** `d3949631ddd135c50bbd91eec26d9f77b70bdf41692a4500e01d0d01a34e992b`. Il contratto non contiene il proprio hash.

---

## 7. Invarianti verificate

- `j19a_runs` e `j19b_runs`: **assenti**. Nessun GO su disco. Nessun actor.
- **Zero** fit, rollout, environment, critic, PPO, collection.
- **Non modificati**: `v26c_j16_closed_loop.py` (`6ac45854…`), `v26c_j18_b_only_update.py`
  (`d3949631…`), `v26c_penetration_contract.py` (`9257e9b8…`), il leaf J18, J8, J2, J4, J7, J9R1,
  J10R1, produzione, FSM v3, detector/morphology, reward, sigma, SEA/C++, architettura.
- Semi 126, 127, 128: non letti, non generati, non usati.
- Worktree sporco preservato. Un subagente usato, solo per lettura comparativa.

---

## 8. TODO propagati

- **J19A non eseguito**: serve un GO che pinni i quattro hash sopra.
- **J19B non preregistrato, non implementato, non avviabile**: solo dopo l'audit del leaf J19A.
- **L'inversione dell'import J18 e' stata APPROVATA dall'architetto** (§3).
- Il test storico J18 resta a 248/249 per `E54`, invariante di readiness scaduta: **non modificato**
  per decisione dell'architetto (§5).
- Il max-drift resta terza chiave come tie-break diagnostico, attualmente irrilevante.
- Il sidecar `actor_feature_manifest.json` di J8 resta stantio per decisione architetturale.
- La leaf J8 non ha `commit_verification.json`, come la leaf J2.
- `nominal_mean_shift` dichiarato e non misurato nel runner J15R1.
- `policy_std` sempre `null`, ereditato da J12.
- Il fit J11 non è bit-riproducibile dai propri artefatti; J15R1 lo è.
- `best_validation_mse` contaminato in entrambi i fit dalla ripetizione dei blocchi.
- La deviazione pre-breccia non è rilevabile su singola run: serve un confronto appaiato.
- **LOTO / LOCO / B1R1 / B1R2** e generalizzazione/Epic restano TODO futuri.
- **Semi 126–128 e fase G–I** restano sigillati.

---

## 9. STOP

J19A implementata e testata staticamente, **non eseguita**. J19B descritta e **non eseguibile**.

**Fermo in attesa dell'audit Codex.**
