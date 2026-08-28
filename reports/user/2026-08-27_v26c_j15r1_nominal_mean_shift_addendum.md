# V26C J15R1 — Addendum diagnostico: `nominal_mean_shift`

**Data**: 2026-08-27
**Stadio**: `V26C_J15_FRESH_35D_POST_MISMATCH_REFIT` — **J15R1**
**Natura**: **diagnostico soltanto**, `binding = false`.
**Il verdetto NON cambia**: resta **PASS**, **17/17 gate binding**.
**Nessun fit, rollout, environment, critic o PPO.** Leaf committata **non toccata**.

---

## 1. Problema

`gate_matrix()` dichiara **13** diagnostici non vincolanti; il receipt committato ne contiene
**12**. `nominal_mean_shift` era **dichiarato ma mai misurato**: `audit()` non lo calcola.

L'avevo segnalato io stesso nel report di esecuzione (§5.1) e tu l'hai confermato in modo
indipendente. Non vincola nulla — nessuna delle 17 decisioni binding vi dipende — quindi il PASS è
intatto. Ma una dichiarazione senza misura è un buco di osservabilità, e questo addendum lo colma
in modo **additivo e read-only**, senza alterare la leaf immutabile.

---

## 2. Metodo

Formula storica di J11, applicata invariata:

```
nominal_mean_shift = max( abs( candidate_mean − parent_mean ) )   sulle righe nominali [0, 16000)
```

- **`before`** = forward del **parent J2** sulle osservazioni grezze;
- **`after`** = forward del **candidato J15R1 committato** sulle stesse osservazioni grezze;
- solo le colonne `[0, 2)` della testa — le medie; le righe log-std non entrano nella metrica;
- differenza calcolata in float64 da forward float32, esattamente come J11;
- **16000 righe**, intervallo half-open, il blocco nominale J7.

Input: parent J2 **immutabile**, stato candidato **committato**, aggregato **committato**. Nulla è
stato rifittato, ricalcolato o riderivato — solo letto.

Doppio kernel: **numpy float32** e **torch 2.10.0**.

---

## 3. Risultato

| grandezza | valore |
|---|---|
| **`nominal_mean_shift`** | **0.11720377206802368** |
| riga che massimizza | **274** |
| colonna azione | **0** |
| parent @argmax (torch) | −0.29448288679122925 |
| candidato @argmax (torch) | −0.17727911472320557 |
| parent @argmax (numpy) | −0.29448291659355164 |
| candidato @argmax (numpy) | −0.17727914452552795 |

### Cross-check fra kernel

| kernel | valore | argmax |
|---|---|---|
| numpy float32 | 0.11720377206802368 | [274, 0] |
| torch 2.10.0 | 0.11720377206802368 | [274, 0] |
| **differenza assoluta** | **0.0** | argmax concorde |

Un dettaglio che vale la pena riportare invece di lisciare: **la metrica coincide fino all'ultimo
bit, mentre gli operandi no.** Il forward del parent al punto di massimo vale −0.29448288679122925
in torch e −0.29448291659355164 in numpy. L'arrotondamento float32 è comune ai due termini e si
cancella nella sottrazione, quindi la *differenza* è identica anche se i valori assoluti non lo
sono.

### Confronto con il tuo risultato indipendente

Atteso `0.11720377206802368` a `[274, 0]`, parent ≈ −0.2944828868, candidato ≈ −0.1772791147.
**Riprodotto bit per bit**, sia sulla metrica sia sull'indice che la massimizza.

---

## 4. Confronto fattuale, senza verdetti

| | J11 | J15R1 | rapporto |
|---|---|---|---|
| `nominal_mean_shift` | 0.026410309597849846 | **0.11720377206802368** | **4.44×** |
| `nominal_rmse` prima → dopo | 6.85e-08 → 0.005378645 | 7.0e-08 → 0.015535840 | 2.89× sul dopo |
| vincolante? | **no** | **no** | — |

**Contesto July**: il protocollo offline di luglio usava storicamente una soglia **0.005** su questa
grandezza. Non era un gate in J11 e **non è un gate qui**. La riporto come contesto e **non la
importo**: applicarla ora sarebbe inventare una soglia a posteriori su un fit già committato.

**Nessuna soglia inventata, nessun verdetto derivato.** Il blocco nominale è il self-anchor del
fit: J11 aveva già registrato che ci si aspetta che si muova e che **nessuna direzione è
preregistrata** per esso. Questo addendum non cambia quella posizione.

---

## 5. Implicazioni

Ciò che il numero dice, per quanto se ne può dire senza superare l'evidenza:

- il refit ha spostato il blocco nominale **più di quanto avesse fatto J11**, di un fattore 4.4;
- è **coerente** con l'aver aggiunto 854 righe la cui RMSE iniziale era 0.392, cioè un ordine di
  grandezza sopra quella dei blocchi già adattati;
- lo spostamento massimo è **una singola riga e una singola azione** — riga 274, colonna 0 — non
  una deriva diffusa: la metrica è un massimo, non una media, e da sola non descrive la
  distribuzione;
- non dice **nulla** sul comportamento closed-loop, in nessuna direzione.

Ciò che il numero **non** dice, e che non va inferito: se il compromesso fra imparare la regione
correttiva e restare vicino all'ancora nominale sia favorevole. Quella è esattamente la domanda che
il gate A–F di regressione esiste per rispondere.

---

## 6. Limitazioni

- **Diagnostico soltanto**: `binding = false`. Non valuta, aggiunge, rimuove o ripesa alcun gate.
- **Il verdetto resta PASS con 17/17.** Questo addendum non lo tocca e non lo può toccare.
- **Non autorizza nulla**: né la fase closed-loop, né il critic, né PPO, né alcuna readiness.
- **La leaf committata non è stata modificata**, né il suo receipt, né il report di esecuzione, né
  alcuna sorgente congelata, configurazione di produzione o evidenza precedente.
- La metrica è un **massimo su 32000 valori**: non è una misura di deriva media né una stima di
  generalizzazione.
- La discrepanza fra 13 diagnostici dichiarati e 12 misurati **resta nel runner**: qui è colmata
  con una misura esterna additiva, non con una correzione del codice. Correggere `audit()`
  richiederebbe una tua autorizzazione e un nuovo fit, che non chiedo e non propongo qui.

---

## 7. Artefatti

| artefatto | SHA-256 |
|---|---|
| **addendum JSON** `j15_runs/j15r1_nominal_mean_shift_addendum_2026-08-27.json` | `9c50cbfbe6e345dcd3625570d492c460da88009522e24822a7ca0951125c2cfc` |
| parent J2 `module_state.pkl` (immutabile) | `0f182ea9f8939e2b7824e85c12c57343309c444680682b9bce5858dd74f9d130` |
| candidato J15R1 `module_state.pkl` (committato) | `4d084a2a7f0012bd711f39a987dbb7af30b04e265484128c45b0a92b612ab928` |
| aggregato J15R1 (committato) | `f4d0c09c78e812a78910626789043e42a6ee851e99b56e59595f4887d7faebe8` |
| receipt J15R1 | `b7b3e9c7db116af4e19d2807359914a03d7d989824c97393dc997994213de349` |
| `commit_verification.json` J15R1 | `551a9cfd8d727ba181bb8ffcb7611d002b49dc77e4a0c78599c436e2378f8821` |
| receipt J11 (contesto) | `39228c5cf00a753f1d57f07d4794ac2996401e1b40587cf1ec1e5f5e2b0ae65f` |

L'addendum è scritto **fuori dalla leaf immutabile**, in `j15_runs/`, accanto ai log di esecuzione.
Dichiara al proprio interno `diagnostic_only: true`, `binding: false`,
`original_verdict_unchanged: "PASS"`, `closed_loop_still_required: true`.

---

## 8. TODO propagati

- **`nominal_mean_shift` dichiarato e non misurato nel runner** — colmato qui per misura esterna;
  la correzione del codice resta una tua decisione.
- **La deriva del blocco nominale, 4.44× J11** — registrata come osservazione, senza verdetto.
- **LOTO / LOCO / B1R1 / B1R2** — restano TODO futuri.
- **Semi 126, 127, 128** — riserva held-out finale, mai letti.
- **Il gate A–F di regressione closed-loop** — **non iniziato**, ed è lì che si decide se il
  problema J13 è stato toccato.
- **Regola di governance**: ogni fase si chiude con uno user report dedicato, auditato prima della
  fase successiva.

---

## 9. Prossimo gate

Il **gate A–F di regressione closed-loop**, che **non ho iniziato** e che non inizierò senza una
tua autorizzazione esplicita dopo l'audit.

**Fermo in attesa del tuo audit.**
