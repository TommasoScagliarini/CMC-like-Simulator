# V26B B1R2 — studio diagnostico: il difetto è di ottimizzazione, non di informazione

**Data**: 2026-08-25 · **Natura**: diagnostica. Nessun candidato materializzato o promosso, nessun rollout, nessuna collection, nessuna fase Markov, nessun fit correttivo.
**Receipt**: `runs/.../diagnostics/b1r2/v26b_b1r2_diagnostics.json` — `ebf0c8e88762c3bb826754753def2d1980ff0c43cfdbd2108062221cf1cb114f`
**Nessuna soglia toccata.** B0 (`aa7ea0fa…`), B1 (`e7328ac2…`) e B1R1 (`6a604ada…`, `9ffcdceb…`) verificati byte-identici. Nessun candidato o rollout B1R2 esiste.

---

## Sintesi

Quattro cause candidate sono state escluse per misura, non per argomento: **label**, **selezione dell'epoca**, **pull dell'ancora** e **saturazione della rete**. Ciò che resta è positivamente sostenuto dai dati: il residuo sui segmenti parziali è **spiegato al 90,2% da una regressione lineare sulle 23 feature che il modello già riceve**. Un modello al proprio limite informativo avrebbe residuo ortogonale ai propri ingressi; questo non lo è.

**Il vincolo che lega non è l'osservazione ma l'ottimizzazione.**

## Q1 — nessuna epoca soddisfa i gate di ricostruzione

Fit su tutte le 1500 righe da B0, misurato a ogni epoca 1..400 su quattro gruppi. Il fit strumentato è un gemello **bit-identico** di `fit_masked`: il test lo prova confrontando lo stato a epoca 1, 4 e 9 con `fit_masked` eseguita per lo stesso numero di epoche.

| gruppo | soddisfa i gate | best epoch | best RMSE | knee rmse/max | ankle rmse/max |
|---|---|---|---|---|---|
| tutte 1500 | **no** | 396 | **0,02979** | 0,03493 / 0,15591 | 0,02356 / 0,11063 |
| cicli completi 910 | **no** | 394 | 0,02386 | 0,02634 / 0,08757 | 0,02108 / 0,08808 |
| parziali 403 | **no** | 388 | 0,03344 | 0,03612 / 0,11177 | 0,03052 / 0,10925 |
| WAIT 187 | **sì** | 396 | 0,01763 | 0,01959 / 0,13874 | 0,01544 / 0,08201 |

Nemmeno l'epoca migliore su tutto il budget raggiunge 0,02 sulle 1500 righe: il minimo è **1,49× la soglia**.

Due precisazioni necessarie:

- **I "best epoch" sono in parte rumore.** Negli ultimi 100 epoch la curva su 1500 righe oscilla fra 0,02979 e 0,03805 (mediana 0,03229, dev 0,00159); sui parziali fra 0,03344 e 0,05244 (mediana 0,04098, dev 0,00443). Il minimo è −7,7% e −18,4% rispetto alla mediana di coda: **il livello affidabile è 0,0323, non 0,0298**.
- **WAIT supera i gate solo perché è ricostruzione**: quelle righe sono in training e vengono memorizzate. È esattamente ciò per cui hai imposto che quel gate non si chiami heldout.

Andamento su 1500: e50 0,0504 · e100 0,0476 · e200 0,0364 · e300 0,0328 · e358 0,0343 · e400 0,0340. Sui parziali: e300 0,0458 · e358 0,0488 · e400 0,0496 — **peggiorano dopo l'epoca 300**, mentre la curva globale è piatta. All'e\*=358 scelto da B1R1: 0,03433 / 0,02723 / 0,04878 / 0,02679.

## Q2 — copertura completa a 11 fold, quantificata

Sei fold di ciclo completo più cinque di segmento parziale. **Ogni riga non-WAIT è heldout esattamente una volta** (1313 righe); WAIT resta reconstruction-only.

| fold | tipo | traiettoria | val | train | drop | WAIT tr | dmin | firme | >p99 | nn |
|---|---|---|---|---|---|---|---|---|---|---|
| 0–5 | ciclo completo | tutte | 148–154 | 1307–1315 | 31–40 | 167–187 | 21 | ok | 0,0–12,8% | 0,030–0,064 |
| 6 | parziale coda | minus020 | 11 | 1469 | 20 | 187 | 21 | ok | 9,1% | 0,0126 |
| 7 | parziale testa | nominal | 164 | 1316 | 20 | 187 | 21 | ok | 32,9% | 0,0612 |
| 8 | parziale coda | nominal | 32 | 1448 | 20 | 187 | 21 | ok | 25,0% | 0,0219 |
| 9 | parziale testa | plus020 | 144 | 1336 | 20 | 187 | 21 | ok | 31,2% | 0,0536 |
| 10 | parziale coda | plus020 | 52 | 1428 | 20 | 187 | 21 | ok | 36,5% | 0,0490 |

Nessuna fuga temporale, distanza minima 21 ovunque, nessuna firma discreta mancante, 23D di supporto in ogni fold, scaler train-only, train minimo 1307, scarto mediano 31 righe.

Nota fattuale: nominal 0..163 e plus020 0..143 sono in realtà coppie STANCE+SWING **complete**; sono "parziali" per posizione, non per struttura. Le teste sono i fold più difficili (>p99 al 31–33%).

## Q3 — limite informativo, stratificato

Vicino più prossimo a distanza temporale superiore all'embargo, supporto 23D, disaccordo fra le label.

| stratum | n | dNN mediana | disaccordo mediano | p90 | >gate 0,03 | vicino stessa firma |
|---|---|---|---|---|---|---|
| tutte | 1500 | 0,431 | 0,0296 | 0,1444 | 49,3% | 97,6% |
| cicli completi | 910 | 0,381 | **0,0163** | 0,1000 | 36,0% | 99,7% |
| parziali | 403 | 0,496 | **0,0381** | 0,1192 | 59,1% | 91,8% |
| **WAIT** | 187 | **2,288** | **0,1475** | 0,6165 | **93,0%** | 100,0% |

Per firma discreta:

| firma (17,18,19,20,21,24) | n | disaccordo mediano | >gate |
|---|---|---|---|
| STANCE, expected_to, credit 0,25 | 449 | **0,0092** | 9,8% |
| SWING, expected_hs, credit 0,5 | 738 | 0,0356 | 56,0% |
| STANCE, expected_to, credit 0 | 120 | 0,0623 | 90,0% |
| **WAIT** | 187 | 0,1475 | 93,0% |

WAIT ha vicino più prossimo a distanza 2,288, cioè **5,3× la mediana globale**: è isolato nel supporto, coerente con l'unicità e non-ricorrenza del segmento già dimostrate.

Questi numeri indicano un pavimento informativo, ma **non sono un limite raggiunto** — lo mostra Q3-bis.

## Q3-bis — il test decisivo: il residuo è recuperabile dagli ingressi disponibili

R² di una regressione **lineare** del residuo del fit B1R1 sui predittori indicati:

| gruppo | R² knee su canale mascherato | R² knee su 23D vive | R² ankle su 23D vive |
|---|---|---|---|
| tutte 1500 | 0,2610 | 0,2679 | 0,2247 |
| cicli completi 910 | 0,3025 | 0,4792 | 0,2829 |
| **parziali 403** | 0,8771 | **0,9021** | 0,6511 |
| WAIT 187 | 0,6131 | 0,5526 | 0,4812 |
| **cluster di picco plus020 105–125** | 0,9995 | **0,9996** | — |

Sui parziali il **90,2%** del residuo del ginocchio è una funzione lineare delle 23 feature che la rete già riceve. Sul cluster di picco è il 99,96%, con bias medio **+0,1447** e deviazione 0,0231: un errore sistematico, non rumore.

Poiché una funzione *lineare* dei propri ingressi recupererebbe il 90% del residuo, e la rete contiene mappe lineari, **la rete ha sia gli ingressi sia l'espressività**: non è un limite di capacità né di informazione.

## Q4 — nessun difetto di label o di semantica delle azioni

- Ogni label ri-derivata dalle cache pinnate per lookup a uguaglianza float esatta è **bit-identica** al dataset; indici contigui e monotoni su tutte e tre le traiettorie.
- `encode`/`decode` round-trip: **2,22e-16**. Tutte le label in [−1,1]; tutti gli angoli decodificati dentro i bounds assoluti.
- Continuità: salto massimo fra righe consecutive **0,0548**, identico sulle tre traiettorie, p999 0,0546. Nessuna discontinuità.
- Le 5 righe peggiori sono plus020 locale 114–118, tutte con **label corrette** e salti temporali normali (0,038–0,046).

**Verdetto: nessun difetto di lookup o di semantica spiega i picchi.**

### Dove si concentra l'errore

| soglia su \|err knee\| | righe | parziali | cicli | WAIT |
|---|---|---|---|---|
| > 0,05 | 242 | 130 | 93 | 19 |
| > 0,08 | 86 | 71 | 12 | 3 |
| > 0,10 | 53 | **52** | **0** | 1 |
| > 0,15 | 11 | **11** | **0** | 0 |

Escludendo le 53 righe sopra 0,10, la RMSE del ginocchio scende da 0,04048 a 0,03285 — ancora sopra il gate di 0,03.

Il cluster plus020 105–125 è un **bias liscio e monotono** su 21 righe di uno swing: l'errore va da +0,093 a +0,171 e torna a +0,129, con la label che scende regolarmente da −1,0147 a −0,4897 rad. La rete non segue la flessione profonda dello swing nella testa di plus020.

## Cause escluse per misura

| causa | misura | esito |
|---|---|---|
| label o semantica azioni | Q4 | **esclusa** |
| selezione dell'epoca | nessuna epoca su 400 passa; il best è in parte rumore | **esclusa** |
| pull dell'ancora | valore ancora/dati 2,12e-06; **gradiente 1,04e-07** | **esclusa** |
| saturazione tanh | 0,00% di unità oltre \|0,99\| al primo strato, 0,11% al secondo | **esclusa** |
| limite informativo | residuo lineare dai propri ingressi a R² 0,90 sui parziali | **esclusa come vincolo attivo** |

L'attore finale si è mosso da B0 di ‖Δ‖/‖B0‖ = 0,377, quindi non è nemmeno rimasto ancorato all'inizializzazione.

## Raccomandazione tecnica

**Non raccomando di toccare alcuna soglia, e i gate non vanno cambiati: la misura dice che non sono irraggiungibili per ragioni informative.**

L'evidenza indica un solo meccanismo residuo compatibile con tutti i dati: **il fit non si assesta**. Con passo Adam costante a 3e-4 e nessun decadimento, negli ultimi 100 epoch la ricostruzione oscilla di ±0,004 sulle 1500 righe e ±0,009 sui parziali — **un'ampiezza dello stesso ordine dei 0,010 che mancano al gate** — mentre il residuo resta linearmente estraibile al 90% dal supporto disponibile e la rete non è né satura né ancorata. Le oscillazioni sui parziali crescono dopo l'epoca 300 proprio mentre la curva di validation pooled aveva già girato a 358: il modello continua a muoversi senza convergere.

La raccomandazione è quindi di **preregistrare, prima di qualunque nuovo fit, un criterio di convergenza sulla ricostruzione al posto del solo conteggio fisso di epoche** — nella forma che preferisci: un decadimento del learning rate dichiarato in anticipo, oppure un criterio di arresto sulla stabilità della curva di ricostruzione. Non è un rilassamento di soglia e non cambia cosa viene fittato: cambia solo quando il fit si considera finito.

Segnalo con chiarezza che **questa ipotesi l'ho derivata dai numeri di B1R2**. È legittimo per uno studio diagnostico il cui scopo è informare la prossima preregistrazione, ma non è una regola già preregistrata: la decisione di adottarla, e in quale forma esatta, è tua.

Una seconda osservazione, subordinata: il residuo è concentrato nelle teste dei parziali (fold 7 e 9, >p99 al 31–33%), che sono transitori di avvio. La MSE piatta di luglio non distingue quelle righe dalle altre. Non propongo una ponderazione — sarebbe una modifica del protocollo che spetta a te — ma la registro come il secondo candidato, dopo la convergenza.

## File

| file | SHA-256 |
|---|---|
| `v26b_b1r2_diagnostics.py` | `477df0d6e201cf84df3ce545a029b4a35b526cbac3c9f031457ea80ff3fcd248` |
| `test_v26b_b1r2_diagnostics.py` | `4cb9c8256f3d86af0da440755d09b8107b4715bacc0ae2cb7590da15f0b09faf` |
| receipt diagnostico | `ebf0c8e88762c3bb826754753def2d1980ff0c43cfdbd2108062221cf1cb114f` |
| log `b1r2_diag_20260825_154437.log` | `6423b3a354a2dccacaf2bf2afbb512bc12e30d78c59ef300265768752d9ff524` |

```
python test_v26b_b1r2_diagnostics.py       -> {"selftest": "PASS", "checks": 42}
python v26b_b1r2_diagnostics.py --run      -> receipt scritto
```

Il test prova la **bit-identità** del fit strumentato con `fit_masked` a epoca 1, 4 e 9; che ogni snapshot per epoca mantenga colonne mascherate e clock esattamente zero e la logstd byte-identica a B0; che i gate siano presi dall'executor e non riscritti a mano; che la copertura a 11 fold validi 1313 righe senza fuga né firme mancanti; e che il modulo non possa fare rollout, collection, promozione o PPO, né scrivere un modulo actor.

`git status` conferma produzione, FSM v3, contatto, reward, guardie e `tools/` intatti.

## TODO propagati

- **TODO-2** — σ non assunto. *(aperto)*
- **TODO-3** — B3 indeterminato. *(aperto)*
- **TODO-4** — Conflitto cammino/plantarflessione. *(aperto)*
- **TODO-5** — Tensione interpolazione↔quota. *(aperto)*
- **TODO-7** — Penetrazione a 0,22 mm dalla guardia dura in REV4E. *(aperto)*
- **TODO-9** — Swing al 13,5% di non eseguibili. *(aperto)*
- **TODO-10** — Proxy autoregressivo rimosso per costruzione. *(indirizzato)*
- **TODO-14** — 590 righe mai validate sotto LOCO: **risolto in progetto** dalla copertura a 11 fold, che ne valida 1313 su 1313 non-WAIT. *(risolto in progetto, non ancora eseguito)*
- **TODO-16** — Patience pooled fermata a 42 stale su 60. *(aperto)*
- **TODO-17** — Nuovo: la ricostruzione oscilla di ±0,004 (globale) e ±0,009 (parziali) negli ultimi 100 epoch, ampiezza confrontabile con il divario dal gate. *(nuovo)*
- **TODO-18** — Nuovo: il residuo sui parziali è linearmente estraibile al 90,2% dalle 23 feature vive; il modello non è al proprio limite informativo. *(nuovo)*
