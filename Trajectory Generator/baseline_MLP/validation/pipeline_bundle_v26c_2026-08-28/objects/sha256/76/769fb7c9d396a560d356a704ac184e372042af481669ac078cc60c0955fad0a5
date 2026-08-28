# V26C J16 — Readiness: riqualifica closed-loop A–F dell'attore J15R1

**Data**: 2026-08-27
**Stadio**: `V26C_J16_J15R1_CLOSED_LOOP_REQUALIFICATION`
**Esito**: bundle di readiness completo e verificato. **Nessuna matrice eseguita.**
**Autorizzazione**: `execution_permitted_now: false`.
**Report obbligatorio di fine fase.**

---

## 1. Problema

J12 ha qualificato closed-loop l'attore J11 sulla matrice A–F e ha **fallito 4/6**: E per
`valid_cycles`, F per cinque criteri con l'episodio troncato a 354 passi su
`end_reason = phase_timeout:swing`. J13 ne ha stabilito la causa, J14 ha materializzato le 854
righe correttive, J15R1 ha rifittato e superato il gate supervisionato 17/17.

Questo stadio pone **una sola domanda**: *il refit correttivo recupera E ed F senza far regredire
A–D?* Nient'altro. E la pone sulla **stessa identica matrice**, con le stesse soglie.

---

## 2. Delta esatto rispetto a J12

L'unica variabile scientifica è l'attore: **J11 → J15R1**. Non è un'affermazione: è provato
confrontando i due runner **come programmi**.

### Prova meccanica

| prova | esito |
|---|---|
| bytecode di `evaluate_cell_gate`, `cell_verdict`, `penetration_report`, `expected_reset_time`, `base_env_config`, `cell_env_config`, `unit_correction` | **identico a J12**, costanti comprese |
| costanti **numeriche** sul percorso scientifico | **nessuna** differisce |
| funzioni dell'intero runner che cambiano un numero | **esattamente una**: `verify_actor` |
| e quel numero | **24713 → 25567**, il conteggio righe dell'aggregato J15R1 |
| 27 costanti congelate (`MATRIX`, i tre offset, `SIGMA`, `EXPECTED_STEPS`, maschere, `RESET_SEMANTICS`, `ENV_MUTATION_POLICY`…) | **identiche a J12** |
| `J16_COMMON_GATE` e `J16_KINEMATIC_GATE` | **identici a J12**, valore per valore |
| chiusura di namespace | **zero global non risolti** in ogni funzione di modulo |

`verify_scientific_equivalence` è ereditata **invariata**: a ogni esecuzione ri-confronta 11
costanti e le sei celle campo per campo contro il modulo J9R1 congelato, e ri-deriva entrambi i
gate da `J1_GATE` e `J3_KINEMATIC_GATE` alla fonte. È ciò che rende operativo «non inventa
soglie», e riporta `thresholds_invented_here: 0`.

### Le categorie di modifica, tutte dichiarate

1. **Naming**: stage token, preregistrazione, leaf di output, nome del receipt, artefatti per cella
   (`j16_cell_<ID>_*`), sentinella del preflight.
2. **Attore**: leaf, gli otto pin, il digest, il nome del file receipt.
3. **`verify_actor`, blocco di provenance**: adattato allo **schema del receipt J15R1** — il parent
   sta sotto `lineage.parent`, l'aggregato ha una **quarta sorgente** (l'incremento J14), le righe
   sono 25567, e il disclaimer su J11 è asserito da `lineage.not_j11`.
4. **Due campi di prosa** in `verify_scientific_equivalence`: `actor_before` e `actor_now`.
5. **Una nuova dichiarazione**: la riserva di semi sigillata (§7).

Un audit read-only indipendente aveva individuato sette punti che si sarebbero rotti in silenzio
con un semplice ri-puntamento dell'attore — schema del receipt, 24713, la quarta sorgente, i nomi
dei file pinnati, lo schema della preregistrazione, la sentinella e i nomi per cella. **Tutti e
sette sono chiusi**, e la suite li verifica.

---

## 3. La matrice, invariata

| cella | modo | seed | offset (piena precisione) | etichetta |
|---|---|---|---|---|
| A | deterministic | 123 | `1.956870983805102` | nominal |
| B | deterministic | 123 | `1.756870983805102` | −0.20 s |
| C | deterministic | 123 | `2.156870983805102` | +0.20 s |
| D | stochastic_held | 123 | `1.956870983805102` | nominal σ 0.005 |
| E | stochastic_held | 124 | `1.956870983805102` | nominal σ 0.005 |
| F | stochastic_held | 125 | `1.956870983805102` | nominal σ 0.005 |

Offset **non arrotondati**: presi dalle costanti congelate, non riscritti. 500 passi per cella,
19 file `sim_outputs` per cella, ordine A→F sequenziale.

**Ogni cella viene eseguita una volta anche dopo un FAIL.** `behavioural_fail_fast: false`, e un
test statico verifica che il loop della matrice non contenga né `break` né `continue`. Nessun
retry, nessuna selezione adattiva. Solo un'eccezione tecnica o di integrità ferma la matrice,
fail-closed.

---

## 4. Gate e soglie

**PASS aggregato se e solo se 6/6 comportamentali PASS *e* 6/6 telemetrie valide.** Qualsiasi
invalidità telemetrica produce `INVALID` a prescindere dal comportamento. Un FAIL **viene
committato** come evidenza, non scartato, ed esce con codice 1.

14 controlli binding per cella: gli 8 del gate comune (`steps == 500`, `end_reason ==
episode_time_limit`, `valid_cycles >= 2`, i tre timeout/morfologia/HS a `<= 0`, `resync <= 1`), la
penetrazione hard e i 5 cinematici di J3.

**Bande di penetrazione**, dal contratto pinnato, zero letterali nel runner:

| banda | regola | natura |
|---|---|---|
| soft | `> 0.020` | diagnostica |
| July legacy | `>= 0.025` | diagnostica |
| **hard** | **passa se `<= 0.028`** — quindi **esattamente 28 mm PASSA** | **binding, l'unica** |

Un test AST verifica che nessun letterale float in `{0.020, 0.025, 0.028}` compaia nel runner.

Diagnostici non vincolanti: `action_clipped_steps`, `episode_return`, `realized_noise_rms`,
`policy_std`, e le righe `wait_hs`.

---

## 5. L'attore

Leaf `j15_runs/j15_fresh_refit_v26c_2026-08-27_r1`, **otto file**, pinnati uno a uno.

Verificato dal preflight: `commit_verification.pass true`, nessun marker `TECHNICAL_INVALID`,
nessun lock o staging stantìo accanto, 8/8 hash, **35D con colonne clock 0 e 1 esattamente zero e
le dieci colonne controller LIVE**, log-std con pesi di output a zero e **σ = 0.004999999670722372**
(deviazione 3.29e-10, tolleranza 1e-6), manifest che descrive il modulo accanto a sé,
`deployable: false`.

Lineage asserita: parent **J2** `0f182ea9…`, J7 `bb9b21f0…`, celle B/C `2f37fc7c…`/`bd78e6ac…`,
**incremento J14** `54b2b8e8…`, **25567 righe**, verdetto `PASS`, e `j11_was_loaded: false`.

**J11 è pinnato come evidenza negativa**: `19bf8a43…`, l'attore che ha fallito questa stessa
matrice. Il runner lo nomina, un test verifica che esista, che sia diverso dall'attore sotto test e
che non sia mai caricato.

`verify_actor` gira **due volte**, prima e dopo la matrice, e confronta gli hash per rilevare
derive.

---

## 6. Rischio dichiarato: la deriva nominale

L'addendum J15R1 ha misurato `nominal_mean_shift = 0.11720377206802368` contro **0.026410309597849846**
di J11 — un fattore **4.44** — e `nominal_rmse` 0.015536 contro 0.005379.

**Le celle A–D esercitano esattamente la regione che il refit ha spostato di più.** Se quella deriva
costi comportamento lì è **ignoto**, ed è precisamente ciò che questo stadio misura. La soglia
storica July di 0.005 su quella grandezza **non è importata**: non era un gate in J11, non lo era in
J15R1, e non lo è qui. Importarla ora sarebbe inventare un gate.

Altri rischi registrati: un miglioramento supervisionato del 53.7% sulle righe correttive **non
implica** recupero closed-loop di E ed F; e **nessun esito è preregistrato come atteso** — questa
matrice può mostrare recupero, nessun cambiamento, o una regressione in A–D.

---

## 7. Semi, e una dichiarazione onesta

Letti: **123** (A, B, C, D), **124** (E), **125** (F). Sigillati e mai letti: **126, 127, 128**.

Va detto chiaramente: **J12 non dichiarava alcuna riserva sigillata.** Il concetto entra nel runner
A–F qui per la prima volta. È introdotto **deliberatamente, non ereditato**, e vincola solo questo
stadio: non cambia alcuna soglia, alcun gate, alcuna cella. Un test verifica che 126, 127 e 128 non
compaiano come letterali interi da nessuna parte nel runner.

La fase finale held-out G–I è **separata** e avviene solo se A–F passa.

---

## 8. Verifiche eseguite

Tutte read-only, statiche, selftest e preflight, con `PYTHONDONTWRITEBYTECODE=1`.

| controllo | esito |
|---|---|
| `py_compile` runner + test | **OK** |
| suite | **473 check PASS** |
| preflight | **GO**, 0 blocker, exit 0 |
| preflight inerte | nessun environment, nessun modulo caricato, nessun torch, nessuna scrittura, nessun lock, nessuno staging, `heavy_modules_introduced: []` |
| bytecode del percorso scientifico | **identico a J12** |
| unico cambio numerico | 24713 → 25567 in `verify_actor` |
| gate e 27 costanti | **identici a J12** |
| chiusura di namespace | **zero global non risolti** |
| sorgenti di produzione pinnate | **6/6 invariate** |
| `j16_runs` / leaf / lock / staging / sentinella | **assenti** |
| matrice eseguita | **mai** |
| worktree | preservato |

Il runner rifiuta fail-closed: token di stadio errato, `--out` non autorizzato o con componenti
symlink, leaf preesistente, lock già presente, staging stantìo, sorgente manomessa, attore
manomesso o marcato invalido, deriva dell'attore durante la matrice.

---

## 9. File

| file | SHA-256 |
|---|---|
| `v26c_j16_prereg_closed_loop_requalification.json` | `150f49b1bd2865d9224d43336d50098c2fd610b4b5d121dd1ce737213a0864aa` |
| `v26c_j16_closed_loop.py` | `6ac4585424cbce34957722e8fc64dc0669de14c57a0418e6aa940b1303cc2e34` |
| `test_v26c_j16_closed_loop.py` | `67f00fc6bc9899c94f6780a72371c869e7250b77680598c01ede4b2977783b0b` |
| `v26c_j16_closed_loop_authorization.json` | `228f7eb3264908a4579ee541d73362a6b674b03b5ef497b8eb85a0c29903adde` |

Attore pinnato (8 file), più provenance: addendum deriva nominale `9c50cbfb…`, incremento J14
`54b2b8e8…`, attore J11 fallito `19bf8a43…`, modulo J9R1 `f58b0d14…`, contratto penetrazione
`95a47d53…`.

**Nessun file esistente è stato modificato**: J12 e tutta la catena precedente restano
byte-identici.

---

## 10. Limitazioni

- Questo stadio **non promuove nulla** e non produce alcuna pretesa di deployability.
- Un PASS qualificherebbe l'attore su **AB06 e queste tre partenze di gait**, nient'altro.
- **Non è una pretesa di generalizzazione**: i semi 126–128 restano chiusi.
- Un difetto cosmetico ereditato da J12 e dichiarato: `policy_std` è elencato fra i diagnostici ma
  non è una chiave che il sommario restituisce, quindi viene registrato `null`. Non l'ho corretto —
  cambiarlo sarebbe una modifica non autorizzata al comportamento ereditato.

---

## 11. TODO propagati

- **LOTO / LOCO / B1R1 / B1R2** — restano TODO futuri.
- **Semi 126, 127, 128** — riserva held-out finale; la fase G–I è separata e solo se A–F passa.
- **`nominal_mean_shift` dichiarato e non misurato nel runner J15R1** — aperto, decisione tua.
- **`policy_std` sempre `null`** — difetto cosmetico ereditato, dichiarato e non corretto.
- **Regola di governance**: ogni fase si chiude con uno user report, auditato prima della
  successiva.

---

## 12. STOP

Readiness completa. **Nessuna matrice eseguita, nessun rollout, nessun environment, nessun critic,
nessun PPO, nessuna leaf.** `execution_permitted_now: false`.

Se approvi, serve un **nuovo record GO additivo** che pinni i tre hash di input e autorizzi **una
sola** esecuzione.

**Fermo in attesa del tuo audit.**
