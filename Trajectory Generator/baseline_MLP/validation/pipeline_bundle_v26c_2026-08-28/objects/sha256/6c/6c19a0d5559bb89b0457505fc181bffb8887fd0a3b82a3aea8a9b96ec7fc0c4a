# V26C J18 — Diagnosi della coda, ricalibrazione e selezione sui risultati congelati

**Data**: 2026-08-27
**Stadio**: `V26C_J18_J8_B_ONLY_CONSTRAINED_UPDATE` — **post-esecuzione, read-only**
**Esecutore**: Opus 5, effort xhigh

**Nessun fit, nessun rollout, nessun PPO, nessun environment, nessuna collection.** Nessun artefatto
esistente modificato. I sedici risultati J18 sono usati **congelati**, come registrati nel leaf.

---

## 1. Decisione architetturale registrata

> La soglia max-drift 0.005 di G1/G2 **non è più ammissibile come gate vincolante**: è smentita sia
> dall'attore operativo J8 sia dal precedente July riuscito, ed era stata derivata da rumore
> mean-zero non comparabile con un aggiornamento funzionale. **Non sostituirla con una nuova soglia
> ad hoc.**

Decisione dell'architetto, registrata qui e non ridiscussa. Le misure sotto la sostengono e ne
estendono la base.

---

## 2. Perché la coda violava G1/G2 con RMSE e bias bassi

### 2.1 La media non può controllare il massimo

| | valore |
|---|---|
| `MSE_C` candidato 13 | 9.23e-06 |
| MSE necessaria per garantire max ≤ 0.005 | **≤ 5.0e-09** — **1844× più piccola** |
| costo nella loss della riga anchor peggiore (λ=30) | 1.325e-05 |
| costo della riga B peggiore (w=1) | 1.122e-04 → **8.5×** |
| λ per pareggiarle | **≈ 254**, otto volte il massimo provato |

Con 2497 righe, una singola riga contribuisce `e²/(2n)`: l'ottimizzatore è **strutturalmente
indifferente** alla peggiore. Alzare λ da 1 a 30 riduce il massimo solo di 1.6–2.6×, misurato su
tutte e quattro le coppie (β, lr).

### 2.2 Il conflitto non è geometricamente inevitabile

| | distanza, spazio scalato |
|---|---|
| spaziatura interna di C (vicino più prossimo) | mediana **0.062** |
| **B** → C | min **1.867** — lontano |
| **A** → C | min **0.074** = **1.2×** la spaziatura interna |
| righe C entro 1.0 dalla correzione | **92.4 %** |

Dove A tocca gli anchor la correzione richiesta è **minuscola**: entro 0.15 da C, mediana **0.0068**,
max 0.0438. Le correzioni grandi (mediana 0.41, max 0.58) stanno a distanza 1.5–3.5, fuori dalla
nuvola. Correlazione distanza↔correzione **r = +0.680**.

Test conclusivo — pendenza locale richiesta contro quella che **J8 già realizza**:

| | mediana | max |
|---|---|---|
| richiesta alle 12 coppie A↔C più vicine | 0.106 | **0.455** |
| già realizzata da J8 fra vicini in C | 0.049 | **0.383** |

Una sola coppia su dodici eccede il massimo già realizzato da J8, di poco. **I target sono
soddisfacibili da una funzione con la regolarità che J8 stesso dimostra**: il drift non è imposto
dai dati, è una proprietà della loss. B non impone nulla — è semplicemente lontano; il vicino di
casa degli anchor è **A**.

### 2.3 Sotto-addestrare non è la direzione

Il learning rate **più alto** dà drift **più basso** in **8 coppie su 8** (fino a 2.46×), e **14/16**
checkpoint erano già prima dell'epoca 200 (minimo 179) fallendo comunque G1/G2.

---

## 3. Calibrazione estesa

Tutti gli attori confrontati sono **figli dello stesso parent J2** (`0f182ea9…`), quindi le
grandezze sono omogenee: `max |f_figlio(x) − f_J2(x)|` sugli **stessi identici insiemi di stati**.

| attore | esito closed-loop | max C | rmse C | max D | rmse D | max POST-485 | rmse POST-485 |
|---|---|---|---|---|---|---|---|
| **J4** | vedi §3.1 | **0.00191** | 0.00031 | 0.00171 | 0.00036 | 0.00122 | 0.00062 |
| **J8** | **5/6** | 0.01735 | 0.00447 | 0.02275 | 0.00526 | 0.01726 | 0.00896 |
| **J11** | **4/6** | 0.17734 | 0.00932 | 0.02721 | 0.00604 | 0.37754 | 0.17600 |
| **J15R1** | **0/6** | 0.25584 | 0.02678 | 0.13559 | 0.01627 | 1.51946 | 0.74291 |
| J18 c.13 | non testato | 0.04697 | **0.00304** | 0.01982 | 0.00264 | non calcolabile | non calcolabile |

### 3.1 Il caso J4/J5, riformulato secondo il contratto corrente

**Correzione di una mia sovrainterpretazione.** Avevo scritto che J4 «ha FALLITO il closed-loop».
J5 emise `verdict FAIL`, ma **secondo la propria guardia storica a 0.025 m**. Il contratto corrente
fissato dall'utente è diverso:

| banda | semantica |
|---|---|
| > 20 mm | soft, **diagnostico** |
| ≥ 25 mm | diagnostica July, **non binding** |
| > 28 mm | **unica barra hard binding** — 28 esatto **passa** |

La penetrazione massima di J4 fu **0.026913 m = 26.913 mm**. Sotto il contratto corrente questo è
una **violazione diagnostica July**, **non** un hard fail: 26.913 ≤ 28.

Formulazione corretta, senza sovrainterpretare in nessuna direzione:

- J4/J5 è un **FAIL storico sotto il vecchio contratto** a 25 mm;
- **non dimostra un hard fail** secondo il contratto corrente;
- dimostra soltanto che **un max-drift di 0.00191 non ha evitato la violazione diagnostica a
  25 mm**.

Quel che resta, e che regge da solo la decisione dell'architetto, è: **la soglia 0.005 respinge ogni
attore che questa pipeline abbia mai prodotto, incluso l'operativo J8 a 3.5×**, e il precedente
July riuscito la violava di 6.3×.

*Limiti dichiarati*: J5 è un protocollo deterministico a cella singola, **non** la matrice A–F, e il
suo esito non è commensurabile con «5/6» o «4/6». Con quattro punti non si costruisce alcuna curva.
Gli stati C sono on-policy per J8 e off-policy per J2/J4/J11/J15R1.

### 3.2 Le 485 righe post-mismatch

Sono gli stati che J8 visita nella cella B **oltre il prefisso**; non appartengono ad alcun blocco
J18. Separano molto più nettamente degli anchor: J4 0.00122, J8 0.01726, J11 **0.37754**,
J15R1 **1.51946**.

*Limite decisivo*: questa è la **regione bersaglio della correzione**. Un attore corretto *deve*
cambiare lì, quindi un drift alto non è di per sé negativo e la correlazione con il fallimento è
**confusa dal disegno**. Non la propongo come criterio. Per il candidato 13 il valore **non è
calcolabile**: il fail-closed non ha salvato alcun actor.

---

## 4. Regola di eligibilità e selezione, applicata una sola volta

### 4.1 La regola

Nessuna nuova soglia inventata. Ogni limite è o **preregistrato** o **un precedente misurato**.

| criterio | natura | valore |
|---|---|---|
| G5 `MSE_B` | preregistrato, invariato | ≤ 0.09546555953591015 |
| G6 `MSE_A` | preregistrato, invariato | ≤ 0.004108013186414973 |
| **G3 / G4** bias C / D | **precedente misurato** (decisione architetto) | C ≤ **0.0008992143952043874**, D ≤ **0.0007264156156278919** |
| G7–G11 strutturali | preregistrati, invariati | esatti |
| **E-C** rmse drift su C | **precedente misurato** | ≤ **0.00447396874790294** |
| **E-D** rmse drift su D | **precedente misurato** | ≤ **0.005262698258768814** |
| **G1 / G2 max-drift** | **DIAGNOSTICI, non vincolanti** | registrati, non usati per l'eligibilità |

`E-C`, `E-D`, `G3` e `G4` non sono numeri scelti: sono RMSE e bias dello **stesso aggiornamento che
ha prodotto J8**, misurati sugli **stessi blocchi** rispetto allo **stesso parent J2**. Il criterio è
*«non peggio dell'aggiornamento che ha prodotto l'attore operativo corrente»* —
**non-worse-than-operational-J8**, non derivato da alcuna sigma.

**Bias empirico assoluto di J8 vs J2, precisione piena** (nuovi ceiling binding per G3/G4, per
decisione dell'architetto, che sostituiscono `0.0014557`, la cui derivazione `0.014557/10` condivide
il rumore mean-zero dichiarato inammissibile):

| blocco | n | knee signed mean shift | ankle signed mean shift | **ceiling = max(\|knee\|,\|ankle\|)** |
|---|---|---|---|---|
| C | 2497 | `4.620137560220626e-05` | `-0.0008992143952043874` | **`0.0008992143952043874`** |
| D | 1210 | `0.0006089726265389337` | `-0.0007264156156278919` | **`0.0007264156156278919`** |

**Classifica preregistrata invariata e totale**: `MSE_B` ↑, `MSE_A` ↑, `max|drift| C∪D` ↑, indice ↑.
Il max-drift resta come **terza chiave di ordinamento** fra candidati già eleggibili: non è un gate.

### 4.2 Applicazione

| c | G5 | G6 | G3 | G4 | str | E-C | E-D | elig | MSE_B | MSE_A | rmse C | max C∪D |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | OK | OK | – | – | OK | – | – | | 0.000560 | 0.001135 | 0.022576 | 0.20684 |
| 1 | OK | OK | – | – | OK | – | – | | 0.000113 | 0.000522 | 0.011114 | 0.12247 |
| 2 | OK | OK | – | OK | OK | – | – | | 0.000344 | 0.001803 | 0.031204 | 0.26322 |
| 3 | OK | OK | – | OK | OK | – | – | | 0.000119 | 0.000913 | 0.019215 | 0.18793 |
| 4 | OK | OK | – | OK | OK | – | – | | 0.000868 | 0.001028 | 0.017918 | 0.15569 |
| 5 | OK | OK | – | – | OK | – | – | | 0.000181 | 0.000409 | 0.006558 | 0.09562 |
| 6 | OK | OK | – | OK | OK | – | – | | 0.000351 | 0.001551 | 0.024938 | 0.23625 |
| 7 | OK | OK | – | OK | OK | – | – | | 0.000156 | 0.000704 | 0.012842 | 0.11835 |
| 8 | OK | OK | OK | OK | OK | – | – | | 0.002095 | 0.000945 | 0.013206 | 0.12741 |
| 9 | OK | OK | – | – | OK | OK | OK | | 0.000202 | 0.000486 | 0.004440 | 0.05179 |
| 10 | OK | OK | OK | OK | OK | – | – | | 0.000586 | 0.001308 | 0.020069 | 0.18257 |
| 11 | OK | OK | – | – | OK | – | – | | 0.000146 | 0.000723 | 0.008573 | 0.10089 |
| 12 | OK | OK | OK | OK | OK | – | – | | 0.005533 | 0.000895 | 0.008709 | 0.09264 |
| **13** | OK | OK | OK | OK | OK | OK | OK | **ELIG** | **0.000307** | **0.000560** | **0.003038** | **0.04697** |
| 14 | OK | OK | OK | OK | OK | – | – | | 0.001481 | 0.001074 | 0.015538 | 0.14423 |
| 15 | OK | OK | OK | OK | OK | – | OK | | 0.000184 | 0.000648 | 0.004748 | 0.06090 |

**Eleggibili: 1.** Graduatoria: **[13]**.
**SELEZIONATO: candidato 13** — λ = 30.0, β = 1.0, lr = 5e-05, epoca 191, 6600 step.

**Il candidato 13 resta selezionato**, ora per eligibilità e non per default.

### 4.3 Robustezza della selezione

| variante del gate di bias | eleggibili | selezionato |
|---|---|---|
| **V2** ceiling empirico J8 (C ≤ 0.000899, D ≤ 0.000726) — **APPLICATA** | [13] | **13** |
| V1 soglia preregistrata 0.0014557 — *ritirata dall'architetto* | [13] | 13 |
| V3 nessun controllo di bias | [9, 13] | 9 |

La soglia `0.0014557` è `0.014557 / 10` e **condivideva la derivazione mean-zero dichiarata
inammissibile**: per decisione dell'architetto è sostituita dal ceiling empirico di J8. L'esito non
cambia. Il candidato 9 — l'unico che potrebbe spostare la selezione, con `MSE_B` 0.000202 migliore
di 0.000307 — è escluso da **entrambe** le formulazioni: bias C 0.002436, cioè 1.7× la ritirata e
**2.7×** il ceiling J8. Solo eliminando **ogni** controllo di bias la selezione passerebbe a 9.

Margini del candidato 13 sui ceiling binding: **C 0.433×**, **D 0.550×**.

**Terza chiave di ordinamento.** Il max-drift resta terza chiave della classifica preregistrata come
**tie-break puramente diagnostico fra candidati già eleggibili**, e non come gate. Nota: è **ora
irrilevante**, perché gli eleggibili sono uno solo.

Quasi-eleggibili, per trasparenza: c15 fallisce E-C per il **6 %** (0.004748 contro 0.004474);
c12 di 1.95×; c8 di 2.95×.

### 4.4 Come si colloca il candidato 13

RMSE drift **0.00304** su C, **inferiore a quello di J8 stesso** (0.00447): per la media è un
aggiornamento **più conservativo** di quanto lo fu J8. Max drift 0.04697, fra J8 (0.01735, 5/6) e
J11 (0.17734, 4/6). **Nessuno di questi numeri prevede l'esito closed-loop**: J4, col drift più
basso di tutti, ricevette `FAIL` **sotto il vecchio contratto a 25 mm**; nel contratto corrente i
suoi 26.913 mm sono una violazione **diagnostica**, non un hard fail (≤ 28 mm). Vedi §3.1.

---

## 5. Limitazioni

- **Nessun actor del candidato 13 esiste.** Il fail-closed non ne ha salvato alcuno. Segno del
  massimo, riga che lo realizza, conteggio delle righe violanti, drift sulle 485 post-mismatch e
  traiettoria del drift per epoca **non sono derivabili** dagli artefatti: `history.json` registra
  solo `train_loss_mean` e `composite_objective`.
- La selezione è **offline**. J17 ha già misurato che l'offline non certifica il closed-loop; la
  §3.1 lo conferma nella direzione opposta.
- Quattro punti closed-loop non definiscono alcuna barra, e uno dei quattro (J4/J5) proviene da un
  protocollo diverso dalla matrice A–F.
- `E-C`/`E-D` sono un **precedente**, non una garanzia: nulla dimostra che «non peggio di J8 in
  media» implichi un closed-loop accettabile.
- L'ipotesi che il drift agli anchor sia prevalentemente **positivo** sul ginocchio (il target chiede
  knee più alto nel 92.9 % delle righe di B e 64.0 % di A) **non è verificabile** senza l'actor.

---

## 6. J19, separato in due fasi per decisione dell'architetto

Fit e rollout **non sono concatenati nella stessa esecuzione**: ogni fase chiude con report e audit.

**J19A — `V26C_J19A_SINGLE_REPRODUCTION`** — implementato e testato, **non eseguito**:
rifit singolo del candidato 13 → controllo di riproducibilità → eligibilità offline → commit di un
solo actor in un solo leaf. Contratto e runner sono descritti nel report di readiness dedicato.

**J19B — closed-loop A–F vincolante** — **descritto, non eseguibile, non avviato**. Sarà
preregistrato e pinnato **solo dopo il tuo audit del leaf J19A**. Userà esclusivamente le soglie
correnti — 20 mm soft diagnostico, 25 mm diagnostica July, **> 28 mm unica barra hard binding, 28
esatto passa** — e tutti gli altri criteri congelati della matrice J9R1.

Nota architetturale misurata, per il tuo audit di J19B: il runner J16 è **cablato su J15R1**
(`THE ACTOR IS J15R1, AND ONLY J15R1`), 1737 righe, 71 riferimenti actor-specifici, ma
l'accoppiamento all'actor è concentrato in **due sole costanti** (`J15R1_LEAF`,
`J15R1_MODULE_DIR`) usate in 13 punti. J16 delega inoltre ogni soglia di penetrazione al modulo
`v26c_penetration_contract` (`9257e9b8…`), che implementa **già** il contratto corrente. Riscrivere
a mano un fork di 1737 righe di un percorso di sicurezza introdurrebbe divergenza silenziosa;
ritrascrivere localmente le soglie sarebbe peggio. **J16 e ogni runner storico restano evidenza
immutabile e non vengono modificati.**

---

## 7. TODO propagati

- **J19A implementato e testato, NON eseguito. J19B descritto, non eseguibile, non avviato**:
  sara' preregistrato solo dopo l'audit del leaf J19A.
- **G3/G4 ora usano il ceiling empirico di J8** (C 0.0008992143952043874, D 0.0007264156156278919)
  per decisione dell'architetto; la soglia 0.0014557 e' ritirata.
- Il max-drift resta **terza chiave di ordinamento** nella classifica preregistrata pur non essendo
  più un gate: coerente, ma da confermare.
- Nessun actor del candidato 13 esiste; ricostruirlo richiede il fit di J19.
- I gate offline filtrano, non certificano: la riqualifica closed-loop A–F resta il verdetto
  vincolante.
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

## 8. STOP

Calibrazione estesa e riformulata secondo il contratto di penetrazione corrente, regola di
eligibilita' applicata una sola volta con i ceiling empirici di J8, candidato 13 confermato unico
eleggibile e selezionato, J19 separato in J19A e J19B.

**Fermo in attesa dell'audit Codex.**
