# V26B S1C-3 — Diagnosi: il gate `P/max_abs = 0,25` è **matematicamente incompatibile** con il recupero del segno IK

**Token:** `V26B-S1C-3-DIAG` (sola diagnosi read-only) · **Data:** 2026-08-24 · Nessun fit, rollout, episodio, collection, checkpoint, modifica a codice/config/production/gate, nessuna scelta σ. Prodotti: questo report e una proposta JSON **non autorizzativa**.

## 1. Verifica preliminare: le due serie di energia di pendenza sono entrambe corrette, su domini diversi

Il report S1C-2Z §4 e l'aggregate riportano numeri diversi perché **misurano insiemi diversi di righe**. Ho ricalcolato entrambi:

| attore | 5 finestre **hinge-attive** (64 righe, tutte in train) | 7 finestre **totali** (97 righe, tutti gli split) |
|---|---|---|
| S0D | 87,30 | 103,13 |
| A2 | 61,21 | 72,44 |
| L05 | 40,76 | 53,29 |
| L10 | 32,47 | 45,35 |
| L20 | 25,14 | 37,94 |

La serie 87,3 → 25,1 del §4 è il dominio **hinge-attivo**; la serie 53,3 → 37,9 dell'aggregate è il campo `slope_energy_ratio_pred_over_ik`, calcolato su **tutte e sette** le finestre, incluse la finestra di holdout [267,290] e quella precoce [6,14] dove l'hinge non agisce. Nessuna contraddizione: due domini, entrambi verificati. Il report S1C-2Z avrebbe dovuto dichiarare il dominio accanto ai numeri, e non lo faceva con sufficiente chiarezza.

## 2. La prova di incompatibilità (punto 1)

Tutte le misure sono in **unità di azione**, che è la metrica del gate P (`|π − u_S0D|` sull'azione, non sull'angolo).

**|u_IK − u_S0D| per insieme:**

| insieme | knee mean | knee max | ankle mean | ankle max | righe > 0,25 (knee) | (ankle) |
|---|---|---|---|---|---|---|
| train (380) | 0,35687 | 1,43034 | 0,29363 | 1,18522 | 200/380 | 201/380 |
| embargo (20) | 0,14246 | 0,53903 | 0,20568 | 0,67513 | 4/20 | 7/20 |
| holdout (100) | 0,49445 | 1,43183 | 0,35031 | 1,19791 | 65/100 | 56/100 |
| **hinge-attive (64)** | 0,55167 | 1,43034 | **0,43094** | 1,09054 | 43/64 | **50/64** |
| tutte (500) | 0,37581 | 1,43183 | 0,30145 | 1,19791 | 269/500 | 264/500 |

**Spostamento minimo necessario**, separando i due requisiti (poiché `q_ankle = 0,7·a`, la condizione `q ≤ 0` equivale ad `a ≤ 0`, quindi lo spostamento minimo per il solo segno è `max(0, a_S0D)`):

| insieme | solo-segno: mean / max / righe > 0,25 | aderenza al target: mean / max / righe > 0,25 |
|---|---|---|
| train (380) | 0,33807 / 0,98053 / **229** | 0,29363 / 1,18522 / 201 |
| embargo (20) | 0,31023 / 0,74506 / 11 | 0,20568 / 0,67513 / 7 |
| holdout (100) | 0,39084 / 0,99474 / 72 | 0,35031 / 1,19791 / 56 |
| **hinge-attive (64)** | 0,33802 / 0,95466 / **42** | 0,43094 / 1,09054 / **50** |

**Il risultato è una dimostrazione, non una stima.** Il gate P impone `max_i |π_i − u_S0D,i| ≤ 0,25` su **tutte** le righe di ancora. Il recupero del segno sulla riga *i* impone `|π_i,ankle − u_S0D,i,ankle| ≥ max(0, u_S0D,i,ankle)`. Per **42 delle 64 righe hinge-attive (65,6 %)** quel minimo supera 0,25 — fino a 0,955. Quindi **nessun attore che soddisfi `P/max_abs ≤ 0,25` può avere comando di caviglia non positivo su quelle 42 righe.** I due requisiti sono simultaneamente insoddisfacibili per costruzione, indipendentemente da loss, architettura e taratura.

Sull'intero train la stessa aritmetica dà **229 righe su 380** che richiedono più di 0,25 per il solo segno.

**Una precisazione che mi obbliga a correggere la mia stessa lettura di ieri.** Il gate P è un criterio di **accettazione post-fit**, non un vincolo nel training: le loss non lo hanno mai visto. L20 si è mossa fino a 0,712 sulla caviglia e ha comunque ribaltato solo 10 righe su 64. Quindi il gate **non causa** il fallimento del segno: lo rende soltanto inaccettabile a posteriori. Sono due affermazioni distinte e vanno tenute separate — il gate è incompatibile *in linea di principio*, e le loss provate sono inefficaci *in pratica*.

## 3. Genesi del gate `max_abs = 0,25` e confronto con giugno-luglio (punto 2)

**Genesi.** Introdotto da rev3s, a priori, con questa motivazione testuale: «*a-priori. July achieved a nominal shift RMS of 0.004175 and max 0.031594 under CONSISTENT roles; our bounds are deliberately ~12x looser because we ask a real behaviour change, while max ≤ 0.25 keeps every single row below the 0.376 structural gap **so that no anchor row is converted into the IK target***».

L'ultima proposizione è decisiva: **il limite è stato scritto per impedire che una riga di ancora venga convertita nel target IK** — cioè esattamente la correzione che ora si vuole ottenere. Non è una misura di sicurezza: la sicurezza è altrove ed è closed-loop (penetrazione ≤ 20 mm, nessuna terminazione `grf_penetration`, contatori FSM, timeout di fase, `hs_cancelled`, `resync`). È un vincolo **di preservazione comportamentale**, il cui scopo dichiarato è bloccare la modifica in questione.

**Confronto con la pipeline buona.** I criteri offline obbligatori di F2R/T1R — l'omologo più vicino, e quello effettivamente usato — sono `P1_preservation_rmse_max_per_joint` ≤ 0,10, `P2` ankle RMSE ≤ 0,10/0,15, `P3` knee RMSE ≤ 0,50, `P3b` ≤ 0,75, `P4` invarianti. **Sono tutti criteri di tipo RMSE o invariante: nessun criterio di massimo per riga è vincolante.** La grandezza di tipo massimo esiste — `P5` registra `max_abs_dW1` — ed è marcata esplicitamente **`"informational": True`**. In più, la selezione di luglio del 13/07 fu fatta col **gate closed-loop**, con il gate offline del run scelto a `ok: false`.

**Conseguenza misurata.** Ho ri-valutato tutti i candidati della catena sotto un set di criteri in stile luglio (mean ≤ 0,10, rms ≤ 0,15, strati ≤ 0,10, miglioramento T ≥ 10 %, `psq` ≤ 0,5, drift azione ≤ 0,10, con il massimo per riga registrato ma **informativo**):

| cand. | mean ≤ 0,10 | rms ≤ 0,15 | max (info) | miglior. T % | psq | drift azione | set-luglio |
|---|---|---|---|---|---|---|---|
| A2 | 0,0592 / 0,0449 | 0,0771 / 0,0566 | 0,238 / 0,181 | 18,3 / 13,8 | 0,116 | 0,0657 / 0,0455 | **PASS** |
| W2 | 0,0622 / 0,0465 | 0,0899 / 0,0630 | 0,382 / 0,269 | 20,8 / 13,5 | 0,163 | 0,0687 / 0,0470 | **PASS** |
| W8 | 0,0641 / 0,0470 | 0,1237 / 0,0810 | 0,656 / 0,440 | 23,7 / 11,8 | 0,421 | 0,0704 / 0,0471 | **PASS** |
| L05 | 0,0587 / 0,0603 | 0,0778 / 0,0861 | 0,246 / 0,438 | 18,4 / 17,4 | 0,203 | 0,0651 / 0,0612 | **PASS** |
| L10 | 0,0585 / 0,0700 | 0,0786 / 0,1058 | 0,254 / 0,575 | 18,5 / 19,5 | 0,296 | 0,0650 / 0,0707 | **PASS** |
| L20 | 0,0584 / 0,0809 | 0,0792 / 0,1280 | 0,262 / 0,712 | 18,5 / **21,6** | 0,427 | 0,0649 / 0,0815 | **PASS** |

**Tutti e sei passano il set di luglio.** E ogni singolo fallimento P di questa catena è avvenuto **solo** su `max_abs > 0,25`: A1 aggiunge tre strati discreti fuori limite, tutti gli altri hanno esattamente quella violazione e nient'altro. Nessun candidato ha mai violato il criterio di preservazione in stile luglio.

## 4. Le tre alternative (punti 3 e 4)

Nessuna altera le etichette IK; nessuna usa JUL_H0 come init o label; l'init resta S0D derivato dal V26 di agosto.

**(A) Preservazione mascherata con gate separato sulle righe da correggere.** Il gate di preservazione resta invariato (mean 0,10 / rms 0,15 / max 0,25 / strati 0,10) sulle righe **non** da correggere; le 64 righe bersaglio ricevono un limite proprio, dichiarato.
*Identificabilità 35D*: sostenuta dal preflight — classi non aliasate (NN cross/intra 2,68), ~80 % di varianza spiegata, balanced accuracy 0,785. Riserva: recall 0,639, quindi circa un terzo delle righe di finestra resta ambiguo, e la finestra precoce [6,14] non è osservabile affatto (recall 0,000 nel fold 1).
*Rischi closed-loop/FSM*: le righe da correggere stanno nella fase di spinta, dove TO e contatto sono determinati; deviazioni fino a ~0,95 lì possono far perdere TO/HS validi. Va deciso dal gate closed-loop, non offline.
*Compatibilità luglio*: **alta** — luglio usava criteri **diversi per ruolo** (P1 per la preservazione, P2/P3 per il task): un criterio per insieme di righe è July-faithful.
*Fail-closed*: soglie esistenti intatte fuori dalle righe bersaglio; limite esplicito e preregistrato su quelle; arbitro finale closed-loop.

**(B) Trust region / distillazione soft globale senza `max_abs` sulle righe target.** Come (A) ma senza introdurre un nuovo oggetto: si mantengono mean/rms/strati globali e si degrada `max_abs` a **informativo**, esattamente come `P5` a luglio.
*Identificabilità*: identica ad (A).
*Rischi*: senza tetto per riga, una singola riga potrebbe muoversi fino a ~1,09 — il regime che ha rotto il cammino in S1A (drift medio 0,384). Restano però vincolanti mean, rms e drift azione ≤ 0,10, che S1A **violava** (0,384/0,289): l'inviluppo aggregato resta chiuso.
*Compatibilità luglio*: **massima** — è letteralmente il set di criteri di luglio, senza aggiunte.
*Fail-closed*: mean/rms/strati/T/D invariati + i sette gate closed-loop.

**(C) Nuova raccolta DAgger/teacher-student.** Richiede nuovi rollout e nuovi token.
*Identificabilità*: affronterebbe l'ambiguità residua (il terzo di righe con recall mancante e la finestra precoce), **non** il blocco principale, che il preflight ha già escluso essere l'osservabilità.
*Rischi*: costruire DAgger su un parent che non cammina ha prodotto la spirale documentata 493 → 242 → 197. Da S0D (che cammina) sarebbe July-faithful, ma è il passo più grande dei tre.
*Compatibilità luglio*: la più alta nello spirito (l'11/07 fu DAgger vero), ma non minimale.
*Fail-closed*: token separati per raccolta, fit e rollout, con i gate closed-loop invariati.

## 5. Raccomandazione: **una sola prova, e non è un fit** (punto 5)

Il dato che orienta la scelta è il §2: le loss provate non recuperano il segno neppure quando si muovono di 0,71, quindi **un altro fit ha bassa informatività attesa**. Ciò che invece non sappiamo, ed è cruciale, è **fin dove ci si può spostare continuando a camminare**. Oggi il bracket è: A2 (drift azione 0,0657 / 0,0455) cammina 500/500; S1A (0,384 / 0,289) non cammina. In mezzo, nulla.

**Prova raccomandata — un solo rollout nominale deterministico di L20.** L20 è già materializzato e pinnato; nessun nuovo fit. Rispetto ad A2 il ginocchio è praticamente immobile (drift 0,0649 vs 0,0657) e **si muove quasi solo la caviglia** (0,0815 vs 0,0455): la prova isola quindi un singolo asse. L20 è anche il candidato più avanzato verso l'obiettivo primario — miglioramento T sulla caviglia 21,6 % contro 13,8 % di A2, energia di pendenza sulle finestre 25,1 contro 61,2, RMSE puntuale caviglia 0,200 contro 0,228 — e **passa il set di criteri di luglio**.

- **Init / candidato**: `S1C2Z_L20_35D_NONDEPLOYABLE`, actor pinnato; nessun altro candidato.
- **Loss / split**: **nessuno** — non c'è un nuovo fit. Gli split restano quelli congelati e non vengono toccati.
- **Precondizione di accettazione offline**: `max_abs` degradato da vincolante a **informativo**, allineandosi a `P5` di luglio; mean ≤ 0,10, rms ≤ 0,15, strati ≤ 0,10, T ≥ 10 %, `psq` ≤ 0,5, drift azione ≤ 0,10 restano **invariati** e L20 li passa già. **Questo è un cambio di soglia e spetta soltanto a te**: io non l'ho applicato.
- **Gate closed-loop**: i sette invariati — 500/500 `episode_time_limit`; `phase_timeout` stance e swing 0; `morphology_causal_contract_failure` 0 per riga; `hs_cancelled` max e final 0; `resync` ≤ 1; ≥ 1 ciclo valido; penetrazione ≤ 0,020 m e nessuna terminazione `grf_penetration`.
- **Falsificabilità**: se L20 non cammina, l'intera direzione «allargare la trust region» è confutata con un solo esperimento, e resta solo la via (C). Se cammina, si ottiene un attore che cammina, più vicino a IK di A2 su tutte le metriche morfologiche, e la questione del segno si può attaccare da una base che cammina.
- **Fail-closed**: esecuzione unica, nessun retry, quarantena in caso di fallimento, nessuna promozione, σ non scelta.

**Avvertenza che devi avere prima di decidere:** anche se L20 camminasse, **non recupera il segno** — 10 righe su 64, contro le 4 di S0D. Questa prova risponde a «quanto ci si può spostare restando in cammino», non a «si può recuperare la spinta». Il §2 dice che la seconda domanda richiede spostamenti > 0,25 su 42 righe, cioè un regime che nessun esperimento di questa catena ha ancora esplorato in closed loop — ed è esattamente ciò che il rollout di L20 comincerebbe a delimitare.

## 6. Artefatti
Report: questo file. Proposta **non autorizzativa**: `Trajectory Generator/baseline_MLP/validation/v26b_bridge_2026-08-24/v26b_PROPOSAL_ONLY_rev4a_l20_rollout.json` — è una proposta, **non** un emendamento, non pinna nulla come autorizzato e non abilita alcuna esecuzione.

Nessun artefatto esistente è stato letto in scrittura, nessun gate modificato, nessun checkpoint creato. Production intatta.

## 7. Stato
**STOP in attesa del tuo audit.** Nessun fit, rollout, collection o promozione. B3 resta INDETERMINATE, σ irrisolta, S0D resta l'unico attore con evidenza closed-loop, e i sei candidati restano quarantinati sotto le regole attuali.
