# V26C J18 — Addendum v3 e correzioni d'architetto

**Data**: 2026-08-27
**Stadio**: `V26C_J18_J8_B_ONLY_CONSTRAINED_UPDATE` — **preparazione esecuzione**
**Versione**: 3, **additiva**. La v1 e la **v2 restano byte-identiche** (v2 verificata da test).
**Esecutore**: Opus 5, effort xhigh

**Nessun fit, nessun rollout, nessun environment, nessun critic, nessun PPO, nessuna collection,
nessun candidato actor, nessuna `j18_runs`, nessun seme sigillato, nessun GO valido creato.**

Questo report **non sostituisce** quello dell'addendum v2
(`2026-08-27_v26c_j18_addendum_v2_preparazione_esecuzione.md`), che resta agli atti invariato.

Nota di ambiente: **`apply_patch` non esiste in questo ambiente**. Ho usato gli strumenti di editing
patch-based disponibili — nessun `cat`, nessuna redirezione, nessun trucco di shell. Unica
eccezione dichiarata: tre sostituzioni letterali in questo report (conteggio test e due hash) fatte
con uno script Python di find-and-replace mirato, non con una riscrittura cieca.

---

## 1. L'addendum v3 registra tre precisazioni pre-esecuzione

v3 non è più descritto come «un solo punto»: contiene **tre** precisazioni, tutte registrate prima
che alcun fit sia stato eseguito. Nessuna tocca ciò che governa l'esperimento — obiettivo, quattro
blocchi, griglia ed enumerazione, undici gate, soglie, ordine di classifica, regola fail-closed,
coefficienti per riga, semantica RNG e chunking restano invariati.

| id | punto | emenda |
|---|---|---|
| V3-1 | contratto di assorbimento delle scale | v2 `gap_2` |
| V3-2 | qualificazione statistica dello stimatore minibatch | v2 `gap_1` |
| V3-3 | eliminazione di ogni import runtime di modulo di fase precedente | vincola il runner |

### 1.1 Assorbimento delle scale

Una precisazione dovuta, perché la direttiva iniziale partiva da una premessa da correggere:
**quell'assert non era più nel codice**. L'avevo introdotto, il test double a 2 epoche lo ha fatto
abortire, e l'avevo già sostituito con un controllo funzionale — documentato nel report v2,
sezione 4.

La sostanza del rilievo resta però giusta e v3 la registra:

- il testo v2 poteva essere letto come se la bit-identità valesse **post-training**. Era misurata
  sui pesi del **parent** e nella direzione `(W·s)/s`: v3 **la circoscrive esplicitamente**;
- il contratto funzionale è **rafforzato**: tutte le **4221** righe, in **entrambi** i kernel
  (numpy float64 e torch float32), ciascun lato like-for-like, soglia **1e-06**;
- il delta del round-trip dei pesi è **registrato per candidato come diagnostica** e non governa
  nulla;
- i gate restano valutati **solo sull'actor raw**.

**La stima quantitativa dell'audit è confermata dalla misura.** Su 200 000 valori float32 per scala:

| scala | entry che **non** fanno round-trip |
|---|---|
| 3.5 | **8.25%** |
| 55.0 | **9.18%** |
| 60.0 | **4.37%** |

Su un actor perturbato realisticamente: **57 entry su 8960** non bit-identiche, delta massimo
4.657e-10, **mentre la funzione è preservata** a 3.136e-10 in numpy e 1.192e-07 in torch su tutte le
4221 righe. La bit-identità avrebbe bocciato un actor funzionalmente sano.

Corretto anche il testo: la tolleranza 1e-6 è **un milionesimo**, non un decimilionesimo.

### 1.2 Qualificazione statistica dello stimatore minibatch

**Il rilievo è matematicamente corretto e la mia formulazione precedente era sbagliata.** Avevo
scritto che ogni passo di gradiente è una stima non distorta dell'obiettivo registrato. Non lo è.

v3 ora distingue nettamente:

- **Vale**: `L_M` è uno stimatore **non distorto** di `Σ_b w_b·MSE_b` **se e solo se M è estratto
  uniformemente e indipendentemente da θ**.
- **Non vale nel training effettivo**: si usa **random reshuffling**, cioè campionamento **senza
  reimmissione** dentro l'epoca. Dopo il primo minibatch θ dipende dalle righe già visitate e il
  chunk successivo è estratto dal **resto**: chunk e θ non sono indipendenti. **Nessuna
  rivendicazione di unbiasedness condizionata per un passo reale**, e la formulazione per-passo
  della v2 è **ritirata**.

Ciò che è garantito, e che regge la registrazione:

1. **coefficienti per riga esatti**, `w_block(i)/N_block(i)` — l'influenza di ogni blocco è governata
   dal suo peso registrato e mai da quante sue righe capitano in un batch;
2. **copertura esatta**: una permutazione per epoca, ogni riga visitata **esattamente una volta**,
   chunk finale corto tenuto e scalato per `N/|M|`;
3. **obiettivo di selezione del checkpoint esatto**: valutato sui blocchi **pieni** a **θ congelato**
   dopo ogni epoca, quindi esattamente `Σ_b w_b·MSE_b`, **senza alcuno stimatore**.

Anche `everything_else_stands_unchanged` non parla più di «per-step unbiasedness»: riporta il
contratto corretto. Nessuna formula, nessun peso, nessuna soglia, nessun coefficiente, nessuna
misura cambia.

### 1.3 Nessun import runtime di fase precedente

Eliminati tutti e tre (`v26c_j11_multistart_fit`, `v26c_j8_recovery_fit`,
`v26c_j14_dagger_dataset`). Un GO J18 pinna **sette** artefatti — v3 lo dice ora correttamente, non
più «sei»: dipendere da codice che il GO non pinna metterebbe il comportamento del run fuori dal
controllo del GO.

| trascrizione locale | verifica |
|---|---|
| digest canonico (`warm_start.actor_state_digest`) | riproduce **J8 `6a879714…`** e **J2 `59d54240…`** dagli artefatti, **e coincide con l'implementazione canonica**, verificato importandola nel solo test |
| scale July `(1,1,4,60,1,1,1,3.5,55,1)` | colonne risolte **`[[27,4.0],[28,60.0],[32,3.5],[33,55.0]]`**; nome mancante = **fallimento duro**, mai un 1.0 silenzioso |
| predicato discreto J14 | sui 35 nomi risolve **esattamente `[11,12,13,17,18,19,20,21]`** |

---

## 2. Irrigidimenti nel runner

| # | correzione | perché |
|---|---|---|
| 1 | `rng = np.random.default_rng(123)` costruito **prima** di Adam | l'ordine registrato mette il Generator prima dell'optimizer. Adam oggi non consuma randomness NumPy, quindi l'ordine non è osservabile in questa versione di torch: il codice segue il contratto invece di dipendere dal fatto che resti vero |
| 2 | **G11 ricorsivo** su tensori **e** su ogni metrica riportata | la v1 registra «ogni tensore e ogni metrica riportata». L'implementazione controllava solo lo stato: un NaN annidato in una metrica per azione sarebbe passato |
| 3 | `encode_json(..., allow_nan=False)` | seconda barriera fail-closed dietro G11: se un valore non finito arrivasse alla serializzazione, solleva invece di scrivere i letterali non standard `NaN`/`Infinity` |
| 4 | docstring di `validate_go`: **sette** artefatti | diceva sei |
| 5 | **G7 realmente byte-exact** | vedi 2.1 |
| 6 | **G11 esteso ai diagnostici riportati** | vedi 2.2 |
| 7 | header del runner ristretto a `--preflight-only` / `--dry-run` | vedi 2.3 |
| 8 | docstring `minibatch_loss` qualificata a θ fisso | vedi 2.4 |

### 2.1 G7 non era byte-identità

**Il rilievo è fondato.** G7 diceva «byte-identical» ma confrontava con `logstd_tensors`, che
**converte in float64**, tramite `np.array_equal`, che confronta **numericamente**. Due differenze
reali sarebbero passate inosservate:

- **`-0.0` contro `+0.0`**: numericamente uguali, byte diversi. Un log-std con un bit di segno
  ribaltato sarebbe passato.
- **dtype diverso** a parità di shape e valori.

Ora `logstd_raw_slices` non fa **alcun cast** e `bytes_identical` confronta **dtype, shape e byte in
ordine C**. Il requisito di state-independence resta.

Test avversari: E110 verifica che l'actor con `-0.0` sia **numericamente identico** al parent, E111
che **G7 fallisca comunque**, E112 che sia ancora state-independent — cioè che *solo* la byte-identità
lo colga. E113 fa fallire G7 su un cambio di dtype a shape e valori identici. `np.array_equal` sul
log-std: **0 occorrenze** nel sorgente.

### 2.2 G11 non copriva tutti i numeri riportati

`evaluate_gates` vedeva solo stato e metriche A–D, ma il record del candidato pubblica anche
`best_objective`, `history[*].train_loss_mean` e `.composite_objective`, e
`scale_absorption`/`weight_round_trip`. Quei campi erano **fuori da qualunque controllo**.

Ora `run_fit` costruisce **prima** il dict `diagnostics` con tutti questi campi, lo passa al gate
come `reported_diagnostics`, e **riusa lo stesso dict** nel record: G11 ispeziona esattamente ciò che
verrà serializzato.

**Scelta fail-closed, dichiarata.** Un candidato con un valore non finito **fallisce G11** e non può
quindi essere selezionato, ma il suo record **viene comunque scritto**, con ogni valore non finito
sostituito da `{"non_finite": <NaN|Infinity|-Infinity>, "path": <path>}`. Ho scelto la
sanitizzazione invece dell'abort perché abortire il run distruggerebbe l'evidenza degli altri
quindici candidati, che la regola fail-closed impone di registrare. Nulla di non finito raggiunge
mai il JSON, e `allow_nan=False` resta l'ultima barriera.

Test: E115 fa fallire G11 su NaN annidato in `history`, Inf in `scale_absorption` e NaN in
`best_objective`; E116–E118 verificano che la sanitizzazione trovi **kind e path** di ognuno e che il
record serializzi senza NaN.

### 2.2b I `gates` non erano sanitizzati — il fail-closed si autodistruggeva

**Difetto reale trovato dall'audit, riprodotto e corretto.** Sanitizzavo `metrics` e `diagnostics`
ma **non** `gates`. Poiché `evaluate_gates` copia il valore misurato in `gate["measured"]`, un NaN in
una metrica **finisce anche lì**. Riproduzione, prima della correzione:

```
metrics["B"]["mse"] = NaN
  -> gate con measured non finito : [('G5', nan)]
  -> G11 passed                   : False        (corretto)
  -> passed_all_gates (raw)       : False        (corretto)
  -> encode_json ABORTISCE: Out of range float values are not JSON compliant: nan
```

Cioè: **l'intera run sarebbe morta** invece di registrare quel candidato come FAIL e proseguire con
gli altri quindici — l'esatto contrario del contratto dichiarato da `sanitise_non_finite` e della
regola «tutti e sedici i candidati registrati». La barriera `allow_nan=False`, pensata come ultima
rete, sarebbe diventata la causa della perdita di evidenza.

Correzione strettamente locale:

1. `passed_all_gates` calcolato sui gate **RAW**, prima di qualunque sanitizzazione: il verdetto
   viene da ciò che è stato misurato, mai dalla sua resa serializzabile;
2. anche `gates` sanitizzato, sotto il path `$.gates`;
3. `safe_gates` serializzato e `gate_findings` concatenato a `metric_findings + diagnostic_findings`;
4. **nulla** cambiato in gate, soglie, ranking, loss, griglia, seed.

La costruzione del record è estratta in `build_candidate_record`, così il test esercita il **codice
reale** invece di una sua copia.

Test avversario execution-like (E127–E147), su due contaminazioni distinte — NaN in `MSE_B` e Inf
nel drift di C:

- la contaminazione **si propaga** in `gate["measured"]` (`G5` e `G1` rispettivamente);
- **G11 fallisce** e `passed_all_gates` resta `False`, preso dai gate raw;
- il record **serializza** in strict JSON, **senza alcun letterale** NaN/Infinity;
- il finding porta il path preciso — `$.gates[4].measured` e `$.gates[0].measured` — e l'indice
  segnalato è verificato essere proprio il gate che ha misurato il valore;
- lo stesso non-finito è registrato anche sotto `$.metrics.B.mse` / `$.metrics.C.max_abs`;
- i booleani `passed` non sono toccati dalla sanitizzazione;
- **sedici record insieme, uno contaminato: tutti serializzano, nessuno perso**, solo il terzo porta
  findings, e i quindici puliti restano **numericamente invariati**;
- il percorso finito è verificato invariato valore per valore (E127–E131).

### 2.3 Header del runner

Diceva che torch non è mai caricato «né dalla test suite». È **falso**: i test execution-path lo
caricano. La frase è ora limitata a `--preflight-only` e `--dry-run`, con nota esplicita.

### 2.4 Docstring di `minibatch_loss`

Riportava l'identità d'epoca senza qualificarla. Ora dice **«FOR A FIXED theta»** per l'identità,
qualifica l'unbiasedness a M **uniforme e indipendente da θ**, e afferma esplicitamente che
**nessuna delle due** è rivendicata per il loop reale — dove valgono solo coefficienti per riga
esatti e copertura esatta una volta per epoca.

---

## 3. Test

**249/249 PASS** (erano 219, prima 195, prima 171). Preflight invariato a **42/42**.

| gruppo | cosa morde |
|---|---|
| E127–E147 | il percorso finito resta invariato valore per valore; una metrica NaN/Inf **contamina `gate["measured"]`**, G11 fallisce, `passed_all_gates` è False dai gate raw, il record **serializza comunque** in strict JSON, il finding porta il path preciso `$.gates[i].measured` con l'indice verificato, e **sedici record con uno contaminato serializzano tutti**, nessuno perso, i quindici puliti invariati |
| E105–E113 | `bytes_identical` rifiuta un cambio di dtype e `-0.0` contro `+0.0`, che `np.array_equal` accetta; `logstd_raw_slices` non converte; **G7 fallisce** sul log-std a zero-con-segno che resta numericamente identico e state-independent, e su un cambio di dtype |
| E114–E121 | **G11 fallisce** su NaN annidato in `history`, Inf in `scale_absorption`, NaN in `best_objective`; la sanitizzazione registra kind e path di ogni non-finito e serializza senza NaN; `run_fit` costruisce i diagnostici **prima** del gate e li riusa; G7 non usa `np.array_equal` |
| E122–E126 | l'header non afferma più che la test suite non carichi torch e resta limitato ai due modi read-only; `minibatch_loss` qualifica l'identità a θ fisso, l'unbiasedness a M uniforme e indipendente, e non rivendica nessuna delle due per il loop reale |
| E85–E94 | G11 fallisce su NaN in una RMSE per azione annidata, su Inf in una MSE di blocco, su −Inf in uno shift medio, e su un tensore non finito; la finitezza ricorsiva attraversa dict, liste e array; `encode_json` **rifiuta** NaN, +Inf e −Inf; il Generator precede Adam (verificato via AST sui numeri di riga); la docstring dice sette |
| E95–E104 | v3 dichiara **tre** punti e non «uno»; lo scope li nomina tutti e tre; l'unbiasedness è rivendicata **solo** per M uniforme e indipendente da θ; v3 dice esplicitamente che **non** vale sotto random reshuffling; al suo posto elenca coefficienti, copertura e θ congelato; **nessun claim superstite** di unbiasedness per-passo; «sei artefatti» e «ten-millionth» **non compaiono più**; le regole che governano l'esperimento restano intatte |
| E59–E72 | digest locale riproduce J8 e J2 e coincide con la canonica; predicato discreto e scale coincidono con v3; nome mancante = fallimento duro |
| E73–E84 | round-trip float32 avversariale 8.25/9.18/4.37%; la funzione tiene in entrambi i kernel; v2 byte-identica |
| E46a–c, E56a–c | il GO deve pinnare **sette** artefatti, un GO senza il pin v3 è rifiutato; i tre moduli di fase precedente vietati per import e per menzione |

---

## 4. Verifiche eseguite

- suite completa: **249/249**
- preflight read-only: **42/42**, «trained nothing, wrote nothing, created no directory»
- **parse JSON** dei cinque artefatti, con `parse_constant` che rifiuta ogni letterale NaN/Infinity:
  tutti OK
- runner e test **compilano**
- `j18_runs` **assente**, nessuna directory di staging, **nessun file GO**
- **nessun seme 126/127/128** fuori dalle due dichiarazioni consentite, via AST
- **nessun modulo di fase precedente** citato nel runner

---

## 5. Artefatti

| artefatto | SHA-256 | stato |
|---|---|---|
| `v26c_j18_prereg_b_only_constrained_update.json` | `f19de7b5c4fa1c4c6b2101e34013576d88a838032c5c490a788e784504a96720` | **invariato** |
| `v26c_j18_prereg_addendum_v2_minibatch_and_scaling_2026-08-27.json` | `bc50072835929961f85aa5ce955124491ca3c9e16d87ec2691b29ab7b996a71d` | **invariato, verificato da test** |
| `v26c_j18_prereg_addendum_v3_scale_absorption_2026-08-27.json` | `e344a5b31909189757e35ba24bb1c66c00848c5ebe59500463e3ed42be3c85d1` | corretto |
| `v26c_j18_dataset_manifest.json` | `ba9ebf9c77550595562d3047368483f8fab7d4c46a0032c177ffd278a07910d2` | invariato |
| `v26c_j18_provenance_overlay_j8_2026-08-27.json` | `dc4fd169ec95e4bcec1d472ab94f176b980f059ffdd98e28d009e83db0db9128` | invariato |
| `v26c_j18_b_only_update.py` | `d3949631ddd135c50bbd91eec26d9f77b70bdf41692a4500e01d0d01a34e992b` | aggiornato |
| `test_v26c_j18_b_only_update.py` | `6050f4c062f64c41198c2be0aeaa41c8b83dd35639319046bb4d4e13e3e468aa` | aggiornato |

Questi **sette** sono esattamente i pin che un GO valido dovrà portare. Nessun JSON contiene il
proprio hash.

---

## 6. Invarianti verificate

- `j18_runs`: **assente**. Nessuna staging. **Nessun file GO.**
- Nessun candidato actor. **Zero** fit, rollout, environment, critic, PPO, collection.
- **Nessuna mutazione** di J8, J2, J7, J9R1, J10R1; né della preregistrazione v1 né dell'addendum v2.
- Semi 126, 127, 128: non letti, non generati, non usati.
- Non modificati: FSM v3, detector/morphology, reward, sigma, SEA/C++, architettura, produzione.
- Worktree sporco **preservato**. **Nessun subagente usato.**

---

## 7. TODO propagati

- **J18 non eseguito**: serve un GO che pinni i sette hash sopra.
- I gate offline **filtrano, non certificano**: il verdetto vincolante resta la riqualifica
  closed-loop A–F, che questa fase non esegue né pre-autorizza.
- Il sidecar `actor_feature_manifest.json` di J8 resta stantio per decisione architetturale.
- La leaf J8 non ha `commit_verification.json`, come la leaf J2.
- `nominal_mean_shift` dichiarato e non misurato nel runner J15R1.
- `policy_std` sempre `null`, ereditato da J12.
- Nessuna leaf pinna il runner che l'ha scritta.
- Il fit J11 non è bit-riproducibile dai propri artefatti; J15R1 lo è.
- `best_validation_mse` contaminato in entrambi i fit dalla ripetizione dei blocchi.
- La deviazione pre-breccia non è rilevabile su singola run: serve un confronto appaiato.
- **LOTO / LOCO / B1R1 / B1R2** e generalizzazione/Epic restano TODO futuri.
- **Semi 126–128 e fase G–I** restano sigillati.

---

## 8. STOP

Tredici correzioni applicate (1-9, poi A-D), v3 aggiornato, v1 e v2 intatte. **Nessun fit avviato, nessun GO creato.**

**Fermo in attesa del tuo audit.**
