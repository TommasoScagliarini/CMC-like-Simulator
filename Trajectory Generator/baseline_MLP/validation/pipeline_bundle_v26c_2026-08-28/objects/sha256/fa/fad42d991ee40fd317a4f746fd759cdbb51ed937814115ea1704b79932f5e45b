# Audit read-only del protocollo luglio R1→R3 e progetto REV4D

**Data**: 2026-08-24
**Natura**: audit e progettazione **read-only**. Nessun fit, rollout, candidato o cambio production. Unica scrittura: l'addendum correttivo rev4c.
**Addendum prodotto**: `v26b_addendum_rev4c_a_corrections.json` — SHA-256 `377fd4e0ff68fc9135f769d8ce2f23935ec9977e185feaa92624360e1942d9ad`

---

## 1. Sintesi per la decisione

L'audit degli artefatti originali di luglio smonta la premessa su cui poggiava rev4c. Tre risultati:

1. **R2 non è "un secondo round con lo stesso 35,23%".** R2 usò una configurazione dell'operatore diversa da R1 su quattro dimensioni simultanee, con quota **40,41%**, `trace_repeat = 1` e `interpolation_steps = 2`.
2. **La copertura negativa non spiega il recupero di luglio.** R2 — il round che produsse 356 step — copriva **9 indici temporali negativi distinti**, la stessa singola finestra `[6,14]` di R1 e di rev4c. Le 65 righe negative non erano in gioco. Quando arrivarono, a R3, la sopravvivenza **scese** da 356 a 221.
3. **rev4c non era un esperimento a una variabile.** Tagliando il prefisso da 392 a 68 ho mosso insieme quota on-policy *e* copertura negativa (65 → 9 indici distinti, 5 finestre → 1). Il risultato di 116 step non è attribuibile alla sola quota. Corretto per addendum.

Il pattern che discrimina davvero i cinque punti disponibili non è la quota e non è la copertura, ma **l'operatore**: ogni configurazione con `repeat 4, interp 0` è rimasta ≤ 116 step; entrambe quelle con `repeat 1, interp ≥ 1` hanno superato 220.

---

## 2. Protocollo luglio ricostruito dagli artefatti originali

Fonti primarie: `target_domain_dagger.py`, `target_domain_imitation.aggregate_dagger_traces` e `adapt_actor`, più i `dagger_dataset_report.json` / `run_summary.json` / `adaptation_report.json` delle quattro directory `runs/training/target_domain_dagger_2026-07-11_{r1,r1b,r2,r3}` e i `rollout_summary.json` dei rollout corrispondenti.

### 2.1 Semantica dell'operatore (dal codice)

- `--trace` è `action="append"`: più tracce sono ammesse e vengono **concatenate in un unico pool visited**, poi l'intero pool è tilato `trace_repeat` volte.
- Il **teacher dataset non si accumula mai**: è sempre lo stesso `teacher_run`. L'accumulo cumulativo riguarda solo le tracce.
- `interpolation_steps = k` genera, per ogni riga visitata, **k osservazioni sintetiche** sul segmento tra lo stato teacher e lo stato visitato allo stesso indice temporale, con le **componenti discrete forzate a quelle della riga visitata**. La label resta la stessa. Quindi `unique_dagger_samples = visited × (1+k)`.
- Aggregato: `concat(teacher, tile(visited_pool, repeat))`. Ordinamento: teacher prima, poi il pool nell'ordine delle tracce da riga di comando (attore più vecchio per primo). Nessun dedup, nessun troncamento.
- Labeling: `teacher_index = step − 1`, azione teacher allo stesso indice, con contiguità degli step imposta fail-closed.

### 2.2 Composizione e catena effettiva

| round | init | tracce (step) | repeat | interp | uniche | on-policy | aggregato | quota | sopravvivenza attore |
|---|---|---|---|---|---|---|---|---|---|
| v2 (BC) | — | — | — | — | — | — | 500 | — | **68** |
| **R1** (=r1b) | v2 `347af228…` | [68] | 4 | 0 | 68 | 272 | 772 | **35,2332%** | **45** |
| **R2** | R1b `b4ad6873…` | [68, 45] | 1 | **2** | 339 | 339 | 839 | **40,4052%** | **356** |
| **R3** | R2 `85394089…` | [68, 45, 356] | 1 | 1 | 938 | 938 | 1438 | **65,2295%** | **221** |

- La catena è **sequenziale**: ogni round parte dall'attore del round precedente, non dall'attore BC originale.
- La catena è **non monotona**: 68 → 45 → 356 → 221. R1 **peggiorò** rispetto al proprio init; il recupero avvenne a R2; R3 regredì.
- `r1` e `r1b` hanno prodotto moduli **byte-identici** (`4850d08c…`): r1b è la riesecuzione che ha anche scritto i summary. Non sono due round distinti.
- Iperparametri: seed 123, batch 64, val 0.20, patience 60, clip 1.0, logstd 0.1, anchor 1e-5 su tutti i round; **lr 3e-4 su R1 e R2, 1e-4 su R3**. R1 si è fermato in early stop a 246 epoche (migliore 186); R2 e R3 hanno esaurito le 400 (migliori 372 e 398).

### 2.3 Copertura negativa per round

La riga "negativa" è definita dalla **label**, quindi dall'**indice temporale**, non dall'attore. Il teacher di luglio (39D, 500 righe) e il corpus AB06 usato in agosto hanno struttura negativa **identica**, verificata riga per riga: 97 righe negative su 500 e le stesse sette finestre `[6,14] [112,132] [174,179] [267,290] [330,334] [423,445] [482,490]`. Il confronto luglio↔agosto è quindi esatto.

| round | indici negativi distinti | finestre coperte | righe negative on-policy | sopravvivenza |
|---|---|---|---|---|
| R1 | **9** | `[6,14]` | 36 | 45 |
| **R2** | **9** | `[6,14]` | 54 | **356** |
| R3 | 65 | 5 finestre | 166 | 221 |
| rev4b | 65 | 5 finestre | 260 | 42 |
| rev4c | **9** | `[6,14]` | 36 | 116 |

**R2 e R1 hanno identica copertura negativa distinta e differiscono di un fattore 7,9 in sopravvivenza.** R3 aggiunge le 65 righe e perde 135 step. La copertura negativa, da sola, non spiega nulla della sopravvivenza in questi dati.

### 2.4 Cosa causò i 356 step — attribuzione onesta

R2 differisce da R1 su **quattro dimensioni simultanee**:

1. init sequenziale dall'attore R1 anziché dal BC;
2. tracce cumulative: due tracce dagli stessi indici temporali ma da **due distribuzioni di stato diverse** (attore v2 e attore R1);
3. `interpolation_steps` 0 → **2**;
4. `trace_repeat` 4 → **1**.

Con conseguente quota 35,23% → 40,41%. **Non esiste alcuna ablazione in luglio**: nessun artefatto isola una di queste quattro. L'attribuzione causale dei 356 step **non è ricostruibile** dagli artefatti ed è segnalata come tale al §5.

Quello che si può dire strutturalmente, e che resta un'ipotesi: l'interpolazione è l'unico dei quattro meccanismi che **crea stati nuovi** invece di ripesare quelli esistenti. Densifica il corridoio tra la varietà del teacher e quella dello studente derivato, mantenendo fisse le componenti discrete della FSM. È il candidato più plausibile, ma un solo punto con quattro variabili mosse non lo dimostra.

### 2.5 Un difetto strutturale dell'operatore `repeat`

`adapt_actor` costruisce lo split di validazione come **permutazione casuale seminata sull'intero aggregato** (`target_domain_imitation.py:846-852`, `mode = seeded_random_fraction`). Le copie esatte prodotte da `trace_repeat` finiscono quindi su **entrambi i lati** dello split.

| configurazione | aggregato | validazione | repeat | P(copie su entrambi i lati) | righe uniche contaminate |
|---|---|---|---|---|---|
| R1 / rev4c | 772 | 154 | 4 | **0,5886** | ≈ 40 su 68 |
| rev4b | 2068 | 414 | 4 | **0,5895** | ≈ 231 su 392 |
| R2 | 839 | 168 | 1 | **0** | 0 |
| R3 | 1438 | 288 | 1 | **0** | 0 |

Con `repeat > 1` la MSE di validazione è **ottimisticamente distorta** e l'early stopping decide su un insieme contaminato. Questo spiega perché R1b si fermò con una validazione apparentemente eccellente (8,43e-05) e produsse comunque un attore peggiore del proprio init. **Conseguenza operativa: la MSE di validazione offline non può essere un criterio di promozione**, e `trace_repeat` non è variabile senza variare anche questa contaminazione.

---

## 3. Confronto numerico con rev4c e con il prefisso completo

| | prefisso | repeat | interp | on-policy | aggregato | quota | indici neg. distinti | init | sopravvivenza | rapporto vs init |
|---|---|---|---|---|---|---|---|---|---|---|
| July R1 | 68 | 4 | 0 | 272 | 772 | 35,2332% | 9 | 68 step | 45 | 0,662 |
| July R2 | 68+45 | 1 | 2 | 339 | 839 | 40,4052% | 9 | 45 step | 356 | 7,911 |
| July R3 | 68+45+356 | 1 | 1 | 938 | 1438 | 65,2295% | 65 | 356 step | 221 | 0,621 |
| **rev4b** | 392 | 4 | 0 | 1568 | 2068 | **75,8221%** | 65 | 392 step | **42** | 0,107 |
| **rev4c** | 68 | 4 | 0 | 272 | 772 | **35,2332%** | 9 | 392 step | **116** | 0,296 |

Osservazioni che reggono al confronto:

- **rev4c riproduce esattamente la composizione di July R1** (772 righe, 272 on-policy, 9 indici negativi, repeat 4, interp 0). Entrambi degradano rispetto al proprio init. Il rapporto è peggiore in agosto (0,296 contro 0,662) perché l'init S1A è molto più lungo.
- La famiglia `repeat 4, interp 0` conta tre osservazioni — 45, 42, 116 — **tutte ≤ 116**. La famiglia `repeat 1, interp ≥ 1` ne conta due — 356, 221 — **entrambe ≥ 221**. La separazione è netta su cinque punti, pur non essendo un esperimento controllato.
- La quota da sola non ordina i dati: 35,23% dà 45 e 116; 40,41% dà 356; 65,23% dà 221; 75,82% dà 42.

### 3.1 Curva copertura/quota per il prefisso S1A

Con prefisso di lunghezza N e repeat r la quota è `rN / (500 + rN)`. Gli indici negativi distinti dipendono solo da N.

| N | indici neg. distinti | r=1 | r=2 | r=3 | r=4 |
|---|---|---|---|---|---|
| 68 | 9 | 11,97% | 21,38% | 28,98% | **35,23%** |
| 113 | 11 | 18,43% | 31,13% | 40,41% | 47,48% |
| 200 | 36 | 28,57% | 44,44% | 54,55% | 61,54% |
| 300 | 60 | 37,50% | 54,55% | 64,29% | 70,59% |
| 356 | 65 | 41,59% | 58,75% | 68,11% | 74,01% |
| **392** | **65** | **43,95%** | 61,06% | 70,17% | **75,82%** |

**Vincolo aritmetico da mettere agli atti**: a copertura piena (N = 392) l'unico `repeat` intero che tiene la quota sotto il 50% è **r = 1**, che dà **43,9462%**. Qualunque interpolazione ≥ 1 a copertura piena spinge la quota ad almeno **61,06%**. Luglio poté permettersi l'interpolazione a quota moderata solo perché le sue tracce erano corte (68 e 113 step); con un prefisso da 392 righe **non è possibile** avere insieme copertura piena, interpolazione e quota moderata.

---

## 4. Proposta REV4D — un solo esperimento, una sola dimensione

### 4.1 Disegno

**Braccio di riferimento: rev4b.** REV4D differisce da rev4b **esclusivamente per `trace_repeat`, 4 → 1**. Prefisso, copertura, labeling, init, ancora, iperparametri e operatore restano byte-identici.

**Formula esatta del corpus**

```
corpus = concat( teacher_500 , tile( prefisso_S1A[1..392] , 1 ) )
       = 500 + 392 × 1 = 892 righe
quota on-policy = 392 / 892 = 0,439461883408071748...
interpolation_steps = 0
trace_repeat = 1
label = u_IK AB06 same-step, teacher_index = step − 1
init = ancora = policy di collection = attore S1A 8f3e0ce1… (nessuna nuova collection)
iperparametri luglio invariati: 400, 64, 3e-4, val 0.20, patience 60, clip 1.0, logstd 0.1, anchor 1e-5, seed 123, nessun grad clipping
```

**Copertura negativa preservata**: 65 indici negativi distinti su 5 finestre — il massimo disponibile dalla traccia S1A, contro i 9 di rev4c.
**Quota**: 43,95%, lontana dal 75,82% vietato e vicina al 40,41% dell'unico round di luglio che abbia recuperato.

### 4.2 Perché non confonde bilanciamento e copertura

rev4c abbassò la quota **tagliando** la copertura: le due si mossero insieme. REV4D abbassa la quota **tenendo la copertura al massimo**. Con i tre punti a disposizione le due dimensioni si separano:

| | copertura (indici neg. distinti) | quota | sopravvivenza |
|---|---|---|---|
| rev4b | 65 | 75,82% | 42 |
| rev4c | 9 | 35,23% | 116 |
| **REV4D** | **65** | **43,95%** | da misurare |

- REV4D **> 116** → a quota moderata la copertura aiuta: la lettura di rev4c era confusa dal confondimento.
- REV4D **≈ 116** → la quota è la leva e la copertura è inerte: la conclusione di rev4c sopravvive alla decorrelazione.
- REV4D **≈ 42** → non è la quota ma l'**operatore di tiling a prefisso pieno** a essere nocivo, indipendentemente dal peso.

I tre esiti sono distinguibili e ciascuno decide qualcosa. Il disegno è quindi ben posto.

**Confondimento residuo che dichiaro in anticipo**: `trace_repeat` controlla insieme il peso on-policy e la contaminazione dello split di validazione (§2.5). Le due non sono separabili dentro quel parametro. Per questo il gate primario deve essere closed-loop e la MSE di validazione va registrata come diagnostica, mai come criterio.

### 4.3 Gate primario e condizioni di STOP

- **Gate primario**: sopravvivenza in step del singolo rollout nominale deterministico, contro il braccio rev4b. **Soglia congelata: > 116 step**, cioè il massimo già ottenuto dalla famiglia `repeat 4, interp 0`. Superarla è l'unica evidenza che il cambio di operatore aggiunga qualcosa oltre a ciò che rev4c ha già mostrato.
- **STOP di preflight (no-write, bloccanti)**: contiguità 1..392; obs35 esatta e finita; allineamento temporale `max |Δt| = 0`; label finite same-step; nessuna collisione con label in conflitto; composizione esatta 892 righe e quota 0,4394618834…; copertura negativa attesa 65 indici distinti su 5 finestre. Al primo fallimento: **NO-GO e STOP**, nessun fit.
- **STOP offline (bloccanti)**: invarianti di integrità (10 chiavi, clock a zero, invarianza bit-identica, logstd byte-identica all'init, save/reload esatto, nessun critico); preservazione di funzione T1/T2 ≤ 1e-5; convergenza del fit sull'aggregato. Se uno fallisce: **nessun rollout**.
- **STOP di esecuzione**: un solo rollout, nessun retry, nessuna promozione automatica. Se la sopravvivenza è ≤ 116, la famiglia `interp 0` è chiusa e la decisione successiva riguarda l'operatore di interpolazione, non il bilanciamento.
- **Divieti**: nessun PPO, nessun test su sigma, nessun altro candidato, nessuna nuova collection, nessun cambio production/env/reward/FSM/morfologia/SEA/C++.

### 4.4 Alternativa esaminata e scartata

Replicare l'operatore di R2 (interpolazione 2, repeat 1) sul prefisso pieno darebbe quota **70,17%**, cioè un ritorno di fatto al regime vietato; sul prefisso da 68 righe darebbe quota 28,98% ma lascerebbe la copertura a 9 indici e muoverebbe **due** parametri rispetto a rev4c. Nessuna delle due soddisfa i vincoli posti. L'interpolazione resta il candidato più informativo per un round **successivo**, e va affrontata sapendo che a copertura piena costa almeno il 61% di quota: è una tensione che richiede una tua arbitrazione esplicita, non una scelta che possa fare io dentro i vincoli attuali.

---

## 5. Dati NON ricostruibili — segnalati esplicitamente

1. **La causa dei 356 step di R2 non è attribuibile.** Quattro dimensioni cambiarono insieme e nessuna ablazione fu eseguita in luglio. Ogni attribuzione a una singola dimensione, inclusa l'interpolazione, è ipotesi.
2. **`interpolation_steps` di R1 non è registrato.** Il `dagger_dataset_report.json` di r1/r1b non contiene i campi `visited_trace_samples`, `interpolation_steps`, `interpolated_samples` (versione precedente del codice). Il valore 0 è **derivato aritmeticamente** da `unique_dagger_samples = 68 = 68 × (1+k)`, non letto.
3. **Nessuna motivazione documentata** per le scelte di R2 (tracce cumulative, interpolazione 2, repeat 1) né per l'abbassamento di lr a 1e-4 a R3. Non è ricostruibile se fossero design deliberato o convenienza.
4. **Commensurabilità dei numeri di sopravvivenza.** Luglio girava sotto guardie di penetrazione 15/25 mm, FSM e corridoio diversi; il runtime v3 di oggi usa 20/28 mm con FSM v3 e corridoio morfologico. Le sopravvivenze luglio↔agosto **non sono direttamente commensurabili**; l'unico confronto rigoroso è quello di composizione dei dataset, che è esatto.
5. **Spazio osservativo diverso**: teacher luglio 39D, studente odierno 35D. Le *label* hanno struttura negativa identica (verificata riga per riga), le osservazioni no.
6. **σ = 0.005** resta un placeholder di serializzazione non risolto, in tutti i round.

---

## 6. Correzioni per addendum

`v26b_addendum_rev4c_a_corrections.json` (`377fd4e0…`) registra, senza mutare nulla:

- **Correzione 1 — difetto testuale A2** (quella che hai ordinato): `seven_gates.meaning` nel receipt rollout rev4c va letto *"closed-loop eligibility of REV4C"*. È una stringa di valore ereditata dal tool congelato `v26b_s1b_rollout.eligibility_gates:190`; tutti i valori numerici sotto `seven_gates` sono di rev4c. Regola in avanti: parametrizzare anche la prosa ed estendere il contratto di igiene alle stringhe di valore fuori dai sottoalberi di provenienza.
- **Correzione 2 — claim "single variable"** (che ho aggiunto, e che ti segnalo perché tu possa rifiutarla): l'emendamento rev4c dichiara un esperimento a una variabile, ma la quota fu manipolata tagliando il prefisso, muovendo insieme quota e copertura. Il verdetto primario preregistrato resta valido come registrato; ciò che **non** è stabilito è che la leva sia la quota anziché la copertura. Include la misura del confondimento e la contaminazione dello split da `repeat > 1`.

Artefatti immutabili verificati byte-identici: emendamento rev4c `b25b0fe4…`, receipt fit `c3fef012…`, receipt rollout `18857462…`, report rev4c `0b9a7bb1…`.

---

## 7. Raccomandazione

Autorizza **REV4D come descritto al §4.1**: prefisso pieno 392, `trace_repeat = 1`, `interpolation_steps = 0`, corpus 892 righe, quota 43,9462%, copertura negativa 65 indici distinti, gate primario sopravvivenza **> 116 step**.

È l'unica configurazione che soddisfa contemporaneamente i tre vincoli che hai posto — una sola dimensione variata rispetto a rev4b, copertura negativa preservata al massimo disponibile, nessun ritorno al 75,82% — e ha il pregio di decorrelare il confondimento che ho introdotto in rev4c.

Ti devo però la valutazione onesta della sua probabilità di successo: REV4D resta dentro la famiglia `interp 0`, che su tre osservazioni non ha mai superato i 116 step. Il segnale più forte di luglio punta all'**operatore di interpolazione**, non al bilanciamento. REV4D è il modo corretto di chiudere la domanda sul bilanciamento — e se la chiude con un esito ≤ 116, la conseguenza preregistrata è che la questione del bilanciamento è esaurita e la decisione successiva riguarda l'interpolazione, per la quale l'aritmetica del §3.1 dice che dovrai scegliere tra copertura piena e quota moderata, perché non si possono avere entrambe.

**Nessun esperimento eseguito. In attesa della tua decisione.**

## 8. TODO propagati

- **TODO-1** — Estendere il contratto di igiene alle stringhe di valore e parametrizzare la prosa di `eligibility_gates`. *(aperto)*
- **TODO-2** — σ = 0.005 placeholder non risolto. *(aperto, ereditato)*
- **TODO-3** — B3 indeterminato: campo di fase identicamente zero su ogni traccia v3. *(aperto, ereditato)*
- **TODO-4** — Conflitto cammino/plantarflessione irrisolto: nessun candidato completa 500/500 con caviglia negativa. *(aperto, ereditato)*
- **TODO-5** — Tensione aritmetica interpolazione↔quota a copertura piena (≥ 61,06%): richiede arbitrazione dell'architetto prima di qualunque round con `interpolation_steps ≥ 1`. *(nuovo)*
