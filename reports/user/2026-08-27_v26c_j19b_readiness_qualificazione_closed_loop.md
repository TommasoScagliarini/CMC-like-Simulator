# V26C J19B — Readiness della qualificazione closed-loop A–F

**Data**: 2026-08-27
**Stadio**: `V26C_J19B_J19A_CLOSED_LOOP_QUALIFICATION` — **readiness**
**Esecutore**: Opus 5, effort xhigh

**Stato: prereg SIGILLATA, readiness completa, NON eseguita.**

**Nessun rollout, nessun fit, nessun environment costruito, nessun PPO, nessun critic, nessuna
collection, nessuna `j19b_runs`, nessun GO, nessun seme sigillato.**
Eseguiti solo compile, test statici, derivazione in memoria e il preflight reale, che e' inerte.

La preregistrazione e' **immodificabile** al SHA-256
`31c2705a8c9501969ddc39a37db43c3a419187febcd5cc830ff3cbfce57347b8`, verificato contro il file
**prima** di scrivere il valore nello strumento di derivazione. Da questo momento la guardia in
`verify_prereg` non e' piu' un no-op: **qualunque modifica alla prereg fa rifiutare l'avvio al
runner**, e sei test lo verificano.

**J16 e ogni runner storico restano byte-invariati.**

---

## 1. Un disegno ritirato, e perché

La prima stesura di questa prereg proponeva un **wrapper con rebinding dinamico** di otto globali di
J16. **Era sbagliata**, e il tuo audit l'ha colta. Ho verificato ogni rilievo nel sorgente prima di
riscrivere:

| difetto | riscontro nel sorgente J16 |
|---|---|
| `verify_prereg` leggerebbe la prereg J16 | righe 78-79: `PREREG`, `PIN_PREREG`; riga 379 valida lo schema J16 |
| l'output non può diventare `j19b_runs` | riga 226 `RELATIVE_LEAF_PARTS = ("j16_runs", …)`; riga 1033 `authorized_leaf()` usa l'override solo come **root** e poi rijoina `j16_runs` |
| `run_matrix` **rifiuta** l'override | righe 1461-1463: `if OUTPUT_ROOT_OVERRIDE is not None and stack is None: raise` |
| `manifest["actor_feature_names"]` per subscript | riga 1472 — campo **assente** nel manifest J19A |
| receipt, sentinella e nomi `j16_*` resterebbero J16 | righe 230, 234 |

Ne segue che l'affermazione «differiscono esattamente otto simboli e `run_matrix`/`preflight` restano
per object identity» era **falsa**. La prereg è stata riscritta in loco — non era sigillata né
eseguita — e registra i quattro difetti.

---

## 2. La copia meccanica, e una scelta che dichiaro

Riscrivere a mano 1737 righe di un percorso di sicurezza è **esattamente** il rischio di divergenza
silenziosa che il requisito di copia meccanica esiste per prevenire: una sola costante mistypata non
si annuncerebbe. Ho quindi reso la copia meccanica **per costruzione**. L'architetto ha **approvato**
la derivazione meccanica nell'ambito del piano già autorizzato.

`v26c_j19b_derive_from_j16.py` deriva il runner da J16 con una **mappa chiusa e pinnata**:

| | voci | hit |
|---|---|---|
| sostituzioni testuali | **17** | **238** |
| innesti strutturali | **9** | **9** |
| **totale** | **26** | **247** |

Le testuali sono classificate in `stage`, `prereg`, `output`, `actor`, `naming`; gli innesti sono
ciascuno classificato e di sola provenienza, schema o naming. **Ogni voce della mappa colpisce
almeno una volta**, verificato.

Questi conteggi erano diventati **stantii** nella stesura precedente — dichiaravano 16/237 e la
lista ometteva due innesti — dopo che avevo aggiunto i blocchi `leaf-comment` e `training_data`. La
prereg ora li dichiara correttamente e **nove test (`B04a`–`B04i`) li verificano incrociando lo
strumento con il documento**, così una futura deriva non passa più inosservata.

Lo strumento **rifiuta di derivare da un J16 alterato**, legge J16 in sola lettura, scrive un solo
file e **non fa parte del percorso di esecuzione**. La trasformazione è essa stessa l'artefatto
auditabile.

**Residui `J16`/`J15R1` fuori dai blocchi di sostituzione: NESSUNO.** Quelli che restano *dentro* i
blocchi sono deliberati: la prosa dichiara da cosa è derivata, e il manifest J15R1 è una delle tre
fonti pinnate che devono concordare sui nomi delle feature.

### Gli innesti strutturali

| # | classe | cosa |
|---|---|---|
| 1 | naming | la docstring del modulo, che nominava il leaf J15R1, J11 come evidenza negativa e la chiave `pass` |
| 2 | provenance | il commento del leaf: **«Eight files, not six»** → **«SEVEN files»**, e non afferma più che J19A committi un dataset aggregato — J19A non addestra nulla |
| 3 | provenance | il campo **riportato** `multistart_disambiguation.training_data`: descriveva l'aggregato di training di J15R1 (25567 righe) e avrebbe scritto un'affermazione **falsa nel receipt J19B**; ora descrive l'aggregato J18 c13, quattro blocchi disgiunti, 4221 righe |
| 4 | provenance | `PIN_J19A`: sette file J19A al posto di otto J15R1, più le fonti pinnate dei nomi e il pin del parent J8 |
| 5 | provenance | le costanti di lineage J15R1 rimaste **morte** (`PIN_J11_FAILED_ACTOR`, `PIN_J14_DATASET`, `PIN_NOMINAL_DRIFT_ADDENDUM`, `NOMINAL_DRIFT_ADDENDUM`): **rimosse**, non lasciate penzolanti né ripuntate a qualcosa che non descrivono |
| 6 | naming | la prosa `actor_before` / `actor_now` in `verify_scientific_equivalence` |
| 7 | provenance | `verify_prereg`, riscritta per lo schema J19B |
| 8 | provenance | `verify_actor`, riscritta per lo schema del leaf J19A |
| 9 | glue | le quattro righe di `run_matrix` che leggono i 35 nomi |

### Un difetto che i miei test NON hanno colto

Il tuo preflight indipendente è fallito con `KeyError: 'deferred_todo'` alla riga 1062. La causa è
precisa: **`deferred_todo` è l'unico campo della prereg che il runner legge fuori da
`verify_prereg`**, dentro `preflight()`, e la mia prereg non lo conteneva.

La lacuna non era nel runner ma **nella suite**: verificavo il runner campo per campo e **non
chiamavo mai `preflight()` end-to-end**. È la stessa classe di difetto che mi morse in J15, dove 208
check passarono su un percorso di produzione non eseguibile. Correzione doppia:

1. la prereg ora porta `deferred_todo`, conforme ai TODO già congelati — LOTO, LOCO, B1R1, B1R2,
   l'Epic di generalizzazione, la fase G–I sui semi 126/127/128 solo se A–F passa, più i difetti
   noti già a verbale. **Nessun ampliamento di perimetro**: un test verifica che non compaiano PPO,
   critic, collection o nuovi fit;
2. **quindici test nuovi (`B62`–`B76`) invocano il preflight reale** e ne asseriscono
   l'inerzia, incluso `B76` che confronta *ogni* campo prereg letto dal runner con quelli
   effettivamente presenti nel documento — il controllo che avrebbe colto questo difetto.

Preflight reale, eseguito: **`verdict GO`, `blockers []`**, nessun modulo pesante importato,
sentinella mai creata, nessun leaf, albero **byte per byte invariato**.

### Difetti reali colti dai test durante il lavoro

- `PIN_J19A_ACTOR_DIGEST` era stato **rinominato ma non rivalutato**: portava ancora il digest di
  J15R1 `c3551341…`. Il test lo ha fatto fallire; corretto a `d4a13ff7…`.
- La docstring diceva «THE ACTOR IS J19A» ma dava il **path J15R1** e la chiave `pass`.
- I due commenti/campi fattualmente errati sopra.

---

## 3. Il percorso scientifico è invariato, e lo si prova

**Otto funzioni bytecode-identiche a J16**: `base_env_config`, `cell_env_config`,
`expected_reset_time`, `unit_correction`, `evaluate_cell_gate`, `cell_verdict`,
`penetration_report`, `production_stack`.

**`run_cell`**: **instruction stream `co_code` identico**. Le differenze sono enumerate e sono solo
naming:

| tipo | delta |
|---|---|
| nomi referenziati | `J16Error` → `J19BError`, `J15R1_MODULE_DIR` → `J19A_MODULE_DIR` |
| costanti non-code | `'j16_cell_'` → `'j19b_cell_'`, `'J16 cell '` → `'J19B cell '` |

**Non affermo object identity**: i due moduli tengono oggetti funzione distinti. L'affermazione è
**uguaglianza di bytecode**, che è ciò che conta e ciò che i test verificano. Per `run_matrix` la
verifica è **AST sezionale**, non identità.

**Identici per valore**: `MATRIX`, le gate table, `SIGMA`, `SIGMA_TOLERANCE`, `ACTOR_WIDTH`,
`CLOCK_COLUMNS`, `CONTROLLER_COLUMNS`, `EXPECTED_STEPS`, `NOISE_HOLD_STEPS`, i seed 123–125, il
fail-fast comportamentale `false`.

**Nessuna soglia di penetrazione compare come letterale numerico nel codice**: la valutazione resta
delegata a `PC.evaluate_series` del modulo pinnato `v26c_penetration_contract.py` (`9257e9b8…`).

---

## 4. Le sei celle A–F

| cella | modo | seed | offset_s | etichetta |
|---|---|---|---|---|
| **A** | deterministic | 123 | 1.956870983805102 | nominale |
| **B** | deterministic | 123 | 1.756870983805102 | −0.20 s |
| **C** | deterministic | 123 | 2.156870983805102 | +0.20 s |
| **D** | stochastic_held | 123 | 1.956870983805102 | nominale, σ 0.005 |
| **E** | stochastic_held | 124 | 1.956870983805102 | nominale, σ 0.005 |
| **F** | stochastic_held | 125 | 1.956870983805102 | nominale, σ 0.005 |

Tre partenze **deterministiche** — nominale e le due perturbate di ±0.20 s — e tre ripetizioni
**stocastiche** alla partenza nominale con la sigma congelata dell'attore stesso. **500 step per
cella**, eseguite **in sequenza nell'ordine A–F**, senza fail-fast comportamentale: una cella che
fallisce non interrompe le altre, così l'evidenza è completa.

Seed usati: **123, 124, 125**. I semi **126, 127, 128 restano sigillati e fuori perimetro**:
verificato via AST che non compaiano nel runner.

---

## 5. Gate: binding e diagnostici

**Binding:**

- i criteri per cella A–F ereditati da J16 senza modifica, le cui **fonti ultime** sono
  `v26c_j1_collect.J1_GATE` meno la sua barra di penetrazione a 0.020, e
  `v26c_j3_closed_loop.J3_KINEMATIC_GATE`;
- la penetrazione **> 28 mm**, **unica** barra binding;
- integrità di leaf e actor, come verificata da `verify_actor` adattata;
- l'accordo delle tre fonti pinnate dei nomi feature.

**Diagnostici, mai binding:**

- penetrazione **> 20 mm** (soft);
- penetrazione **≥ 25 mm** (confronto legacy July).

**28 mm esatti passano.** Nessuna soglia è inventata in questo stadio.

**Aggregazione**, ereditata invariata: **PASS se e solo se 6/6 celle passano comportamentalmente E
6/6 sono telemetry-valid.**

---

## 6. I 35 nomi delle feature

Il manifest J19A non li porta — deliberatamente, e `verify_actor` **rifiuta** un manifest J19A che
avesse acquisito `actor_feature_names` o `deployable`.

Sono risolti da **tre fonti indipendenti pinnate che devono concordare**, e concordano:

| fonte | SHA-256 |
|---|---|
| sidecar J8 | `0c88018d66a648c0a36826f6edbf5e5494ef0c9b496142e1e971e7ab3b1ade81` |
| manifest J15R1 — quello che J16 stesso consumava | `bb24bedca3f8572e370d92ff02640a3890171888215017cd2821d62245670653` |
| dataset teacher J10R1 cella B | `2f37fc7cb101550d2fc0f8709cfdfc44ae5e9ae53003bb7903fcb590406acc62` |

**Caveat dichiarato**: il sidecar J8 è **notoriamente stantio** — byte-identico a quello di J2 e
dichiara l'hash di modulo sbagliato. È usato **solo per ordine e nomi**, **mai** come prova di alcun
hash di modulo, e il suo accordo con due fonti indipendenti è **richiesto**. Una fonte sola non
sarebbe accettata qui.

Sono risolti **prima** che l'ambiente sia costruito: riga 1512 contro 1518, verificato via AST.

---

## 7. Perché questa è la stessa qualifica di J16

J16 era J12 con il solo attore cambiato, e lo dimostrava campo per campo contro J9R1 congelato.
J19B è **J16 con il solo attore cambiato**, e lo dimostra allo stesso modo: `verify_scientific_equivalence` è
ereditata invariata e continua a provare, campo per campo contro il modulo J9R1 pinnato, che matrice,
gate, sigma, tolleranze e bande di penetrazione sono **identiche per valore**.

Ciò che cambia è l'attore sotto test — da J15R1, che fallì questa stessa matrice **0/6**, a J19A — e
nient'altro.

---

## 8. Test eseguiti

**116/116 PASS.** Nessun rollout, nessun ambiente, nessun modulo pesante importato.

| gruppo | cosa morde |
|---|---|
| **B01–B06** | J16 corrisponde al pin; lo strumento pinna lo stesso hash; **il runner su disco è esattamente ciò che lo strumento produce**; ogni voce della mappa ha almeno un hit; **zero residui** fuori dai blocchi; lo strumento **rifiuta** un J16 alterato |
| **B04a–B04i** | i conteggi della mappa sono **pinnati e verificati**: **17 voci testuali / 238 hit**, **9 innesti / 9 hit**, **totale 247**; e la prereg dichiara **gli stessi** numeri, elenca **tutti e nove** gli innesti e nomina esplicitamente i blocchi `leaf-comment` e `training_data` |
| **B07–B14** | **8/8 funzioni bytecode-identiche**; `run_cell` `co_code` identico; i soli delta di nome sono i quattro dichiarati; le sole costanti diverse sono le due stringhe di output; `MATRIX`, gate table, `SIGMA`, tolleranze, step, noise-hold identici per valore; celle, modi e seed sono i congelati; solo 123/124/125 |
| **B15–B22c** | i **quattro difetti del disegno ritirato**: prereg J19B e non J16; `verify_prereg` non è quella di J16; leaf `j19b_runs`; staging, lock e sentinella coerenti; `authorized_leaf()` risolve sotto `j19b_runs` **senza override**; nessun subscript `manifest["actor_feature_names"]`; receipt J19B; **nessun identificatore `j16_` né `J16Error`** nel codice, con la docstring esclusa **per nodo AST** perché nomina la propria sorgente di proposito |
| **B23–B31** | 35 nomi da tre fonti concordi; coincidono con quelli usati da J16 e col dataset teacher; il sidecar J8 dichiarato stantio e usato solo per i nomi; **una fonte stantia è rifiutata**; risoluzione **prima** dello stack ambiente; il manifest J19A davvero non ha i due campi; `verify_actor` rifiuta un manifest che li acquisisse |
| **B32–B43d** | pin di actor e digest; sette file pinnati, ciascuno verificato; `verify_actor` accetta il leaf reale; legge la commit verification sotto la chiave **`ok`**; asserisce mask contract e sigma **con le costanti di J16**; lineage J19A: `PASS`, un solo actor, nessun rollout, parent J8; l'actor differisce dal parent; costanti di lineage morte rimosse; **«SEVEN files» e il leaf ne ha davvero sette**; non afferma più un dataset aggregato; l'aggregato riportato descrive **J18 c13 (4221 righe)** e non 25567; i quattro blocchi con i conteggi reali (500, 14, 2497, 1210) |
| **B44–B52b** | `j19b_runs` assente; nessun GO su disco; **`PIN_PREREG` sigillato all'hash reale della prereg** (`B45a`–`B45f`: non e' piu' `PENDING`, coincide col file su disco, e' l'hash sigillato dall'architetto, lo strumento sigilla lo stesso valore, **una prereg che non corrisponde e' RIFIUTATA**, e col sigillo reale `verify_prereg` passa); nessun modulo pesante importato; sentinella mai creata; **nessun seme 126/127/128** via AST; chiusura del namespace; contratto penetrazione pinnato; **nessuna banda come letterale numerico** e valutazione delegata al contratto |
| **B62–B76** | **il preflight REALE, chiamato end-to-end**: `verdict GO`, `blockers []`, nessun modulo pesante importato, sentinella mai creata, nessun leaf, albero byte per byte invariato; `deferred_todo` letto da *questa* prereg e contenente LOTO, LOCO, B1R1, B1R2, i semi 126/127/128, G–I ed Epic senza aprire perimetro; outcome policy che non promuove nulla; e **ogni campo prereg letto dal runner è presente nel documento** |
| **B53–B61** | la prereg parse senza self-hash; registra il ritiro del disegno e i **quattro** difetti; **dichiara uguaglianza di bytecode e NON object identity**; nomina le otto funzioni; celle e seed coincidono col runner; bande esatte con 28 che passa; caveat sul sidecar dichiarato; stage token coincidente; **zero soglie inventate** |

---

## 9. Artefatti

| artefatto | SHA-256 |
|---|---|
| `v26c_j19b_prereg_closed_loop_qualification.json` | `53198d3df150609a8eff90fdb2ef41d33df0905f19fd73cf35b030e537947185` |
| `v26c_j19b_derive_from_j16.py` | `742c9fd40dd418f466eb678d13049551be874d925215dec28bbade1f35abc7ab` |
| `v26c_j19b_closed_loop.py` | `327b8fe39edae290fd8ed806f6b34715313b8d82d7c08ba600054eb191bdcb19` |
| `test_v26c_j19b_closed_loop.py` | `80afceadbfbe640bcf52f2e1afb7eb8eb17cacdaf5f5f2d25c91813cc2f49ab4` |

Un GO J19B dovrà pinnare questi quattro più `v26c_j16_closed_loop.py` (`6ac45854…`) e
`v26c_penetration_contract.py` (`9257e9b8…`). Il contratto non contiene il proprio hash.

**Attore sotto test**: `j19a_runs/j19a_single_reproduction_v26c_2026-08-27_r1/rl_module/module_state.pkl`
— `8153dc9765cb984ae05502b57283c00c09b12de2c4b9d5128a0de0fc12566530`,
digest `d4a13ff742266e9643012a27c57a6ea6b9205b030529d4c7a8af6d874ab26e96`.

---

## 10. Invarianti verificate

- `j19b_runs`: **assente**. Nessun GO su disco. `PIN_PREREG` = `PENDING`: **nulla è sigillato**.
- **Zero** rollout, fit, environment, PPO, critic, collection.
- **Non modificati**: `v26c_j16_closed_loop.py` (`6ac45854…`), `v26c_penetration_contract.py`
  (`9257e9b8…`), il leaf J19A, il leaf J18, J8, J2, J4, J7, J9R1, J10R1, produzione, FSM v3,
  detector/morphology, reward, sigma, SEA/C++, architettura.
- Semi 126, 127, 128: non letti, non generati, non usati.
- Worktree sporco preservato. Nessun subagente usato in questa fase.

---

## 11. TODO propagati

- **J19B non eseguita**: serve un GO che pinni i sei hash sopra. La prereg e' gia' sigillata.
- La suite readiness J19A resta a 126/127 per `S47`, e quella J18 a 248/249 per `E54`: invarianti di
  readiness scadute dopo l'emissione dei rispettivi GO, **non modificate**.
- Il sidecar `actor_feature_manifest.json` di J8 resta stantio per decisione architetturale; qui è
  usato solo per i nomi, con due fonti concordi.
- La leaf J8 non ha `commit_verification.json`, come la leaf J2.
- `policy_std` sempre `null`, ereditato da J12.
- **LOTO / LOCO / B1R1 / B1R2** e generalizzazione/Epic restano fuori perimetro.
- **Semi 126–128 e fase G–I** restano sigillati.

---

## 12. STOP

J19B derivata, verificata, testata staticamente e con **prereg sigillata**. Preflight reale `GO`.
**Nessun GO architetto, nessuna `j19b_runs`, nessun rollout.**

**Fermo in attesa del tuo audit.**
