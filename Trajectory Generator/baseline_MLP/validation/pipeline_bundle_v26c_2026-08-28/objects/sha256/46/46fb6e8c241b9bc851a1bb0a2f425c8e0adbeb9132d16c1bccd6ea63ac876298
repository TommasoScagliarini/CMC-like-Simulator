# V26B S1 (IK-AB06) — fit offline eseguito: **G_task PASS, Q1 preservazione FAIL** → REJECTED, nessun checkpoint

**Token:** `V26B-S1-FIT` (concesso dopo audit Codex PASS su rev3n) · **Data:** 2026-08-24 · **Ambito:** solo stage offline S1. Nessun rollout, DAgger, σ-sweep, critic/PPO, multistart, morphology fit; nessuna modifica a production env/reward/FSM/SEA/C++ né ad artefatti congelati.

## 1. Esito in una riga
Il fit ha **imparato il task** (RMSE vs u_IK sull'holdout out-of-sample 201–300 = **[0,13421 knee, 0,12155 ankle]** ≤ 0,15) ma ha **violato la preservazione** (Q1 sul source holdout rev3m = **[0,49825, 0,34616]** contro 0,10, cioè 5,0× e 3,5× oltre soglia). Per contratto: artefatto **REJECTED** scritto, **nessun checkpoint pubblicato**, nessuna soglia/metrica/label toccata, nessun retry, STOP.

## 2. Gate (tutti calcolati prima di qualunque pubblicazione)
| Gate | Valore per giunto | Soglia | Esito |
|---|---|---|---|
| **G_task** — RMSE vs u_IK su holdout **out-of-sample** 201–300 | [0,13421327, 0,12155384] | ≤ 0,15 | **PASS** |
| **Q1 rev3m** — RMSE vs media deterministica S0D sul source holdout (4 474 righe, mai in training) | [0,49825444, 0,34615961] | ≤ 0,10 | **FAIL** |
| **Q3** invarianti (10 chiavi, clock zero, invarianza bit-identica, logstd placeholder **e bit-identica all'init**, no critic, save/reload) | tutti True | — | **PASS** |
| **T1** pre-fit / **T2** export | 7,398673e-08 / 9,561586e-08 | ≤ 1e-5 | **PASS** |
| `pass_all` | — | — | **False** |

Diagnostici (mai gate): full-500 vs u_IK [0,08745, 0,07334] (contaminato in-sample: contiene le 380 righe di train, il cui RMSE è [0,06777, 0,05420]); drift vs S0D sulle 500 righe di task mean-abs [0,38390, 0,28909] / max [1,4928, 1,1243] — **atteso**, è il gap strutturale voluto; baseline S0D vs u_IK sullo stesso holdout [0,62144, 0,45692] e su full-500 [0,50350, 0,38201]; shift parametrico ‖θ_S1 − θ_S0D‖² = 2,4398. Contesto obbligatorio: JUL_H0 offline 0,008144 con 500/500 (guardie July); S0D holdout [0,0811, 0,0793] con 500/500 (guardie v3).

## 3. Che cosa dice il dato
**Il task è fattibile e il pre-gate rev3n era predittivo.** Sull'holdout mai visto, S1 passa da [0,62144, 0,45692] (S0D) a [0,13421, 0,12155]: miglioramento 4,6× knee e 3,8× ankle. La mediana kNN5 rev3n aveva stimato [0,12943, 0,09171]: il fit parametrico atterra esattamente in quella banda. L'ipotesi "u_IK non apprendibile a questa densità" è definitivamente esclusa, e la simmetrizzazione dello strumento si è rivelata una previsione corretta, non un salvataggio del gate.

**La preservazione è fallita per propagazione globale, non per un bug.** L'entità dello scostamento da S0D sul source holdout (0,498 / 0,346) è dello stesso ordine del gap S0D↔u_IK misurato sul task (full-500: 0,504 / 0,382): la rete ha spostato il proprio comportamento **ovunque**, non solo sulla varietà di stati del task. Precisazione necessaria: sulle righe del source holdout **u_IK non è definito** (non stanno sulla griglia temporale IK di AB06), quindi il dato misura solo *quanto* S1 si è allontanata da S0D, non che sia "andata su IK" lì.

**L'ancora era numericamente inerte.** Dalla curva registrata: termine di ancora 4,095e-09 → 3,718e-05, cioè contributo **pesato** da ~4e-14 a ~3,7e-10, contro un termine dati di 3,900e-03 all'epoca 300 — sette ordini di grandezza sotto. Con peso 1e-5 l'ancora non ha esercitato alcuna forza di richiamo misurabile: la preservazione, nel disegno rev3l/m/o, era affidata a un meccanismo che sui numeri non poteva vincolare nulla.

**Non è l'incompatibilità di rev3l.** G_task e Q1 vivono su insiemi di righe disgiunti e verificati bitwise-disgiunti: la loro soddisfazione congiunta è geometricamente possibile in linea di principio. Il FAIL è **empirico**: a questo regime (380 righe, 600 passi Adam, ancora 1e-5, tutti e 6 i tensori liberi) la rete non separa le due varietà.

## 4. Conformità al contratto
Init esclusivamente S0D (`481dd0d2…`, module_state `cda6d893…`), verificato positivamente (più forte di una blacklist: nessun altro attore della catena può passare il pin). Training **solo** sulle 380 righe steps 1–190 ∪ 311–500; assenza di righe in 191–310 verificata fail-closed; source holdout mai in training. Numerica congelata: 300 epoche, batch 256, Adam lr 1e-4, seed 2026, flat MSE + clip 1.0 + ancora 1e-5 sui 6 tensori FULL verso θ_S0D; nessun aux; logstd ripristinata e **bit-identica all'init**; scaling July assorbito all'export. rev3l/rev3m/rev3n, S0D e ogni receipt/log/report precedente **immutati** (riverificati in lineage a ogni esecuzione, inclusa la prova di additività sul receipt FAIL `a437f1ef…`). Nessuno staging/lock orfano; `student/S1_IK_AB06_35D/` **non esiste**.

## 5. Opzioni per l'architetto (nessuna eseguita; ciascuna richiede un token esplicito)
1. **Chiudere S1** dichiarando che l'adattamento IK e la preservazione di S0D sono incompatibili a questo regime. Onesto e senza costi.
2. **Ancora July 0,01** (valore storico del 13/07, già usato in rev3i — *non* un numero inventato per passare un gate; le soglie resterebbero identiche). Richiede emendamento perché cambia la numerica congelata del fit. Nota di trasparenza: 1e-5 → 0,01 è 1000×, ma il termine pesato resterebbe ~3,7e-7 contro 3,9e-3 di dati: **probabilmente ancora insufficiente da solo**.
3. **S1 multi-ruolo in stile F2R T1R** (raccomandata): obiettivo congiunto con maschere di ruolo — ruolo *task* = 380 righe con label u_IK; ruolo *preservazione* = righe di ancora prese dal **lato TRAIN di rev3k** (14 834 righe) con label = media deterministica S0D, β = 1 come in T1R. È una struttura già presente nella catena (T1R commissioning), non un'invenzione ad hoc; le soglie restano invariate e le 4 474 righe del source holdout **restano intatte** come gate indipendente. È l'unica opzione che dà alla preservazione una forza comparabile al termine di task.
4. Ridefinire come si misura la preservazione: **sconsigliata** — sarebbe ingegneria di gate a posteriori.

## 6. Artefatti e verifiche
| Artefatto | SHA-256 |
|---|---|
| `…/v26b_bridge_2026-08-24/v26b_amendment_rev3o_s1_fit_execution.json` | `abe463c3d64b4b91a0d5ef83517159bcc6fc417d029be4dd51db9c92a0dfc978` |
| `…/v26b_bridge_2026-08-24/v26b_s1_fit.py` | `a6d7163ef8089ca9efb5b8b31fe92d2132efa357b1c69d814f6d9531ea845916` |
| `…/v26b_bridge_2026-08-24/test_v26b_s1_fit.py` | `ec008ff735da5a4aaa01bb8ecb3a3b5f6a391d232f9232bc4a697b2da6587f82` |
| `…/2026-08-24_V26B_anchors_r1/v26b_s1_fit_REJECTED_20260824_190422.json` | `a559bd691eb5934f32a820e46e13660f010f0b54e445c82d4f13e485154d558a` |
| `…/2026-08-24_V26B_anchors_r1/s1_fit_20260824_190417.log` (`REAL_EXIT_CODE=1`) | `5603e9f19f2a139e627a8bf1f2c01700685eca21e09b5094dd8cf6aba59c44b6` |

**Self-test `test_v26b_s1_fit.py`: PASS, 42 check**, eseguito prima del fit e **ri-eseguito identico dopo il FAIL** (stage-aware): pin e tamper-test rev3o/receipt rev3n/module_state S0D; init positivamente pinnato e rifiuto di un attore diverso della catena (V1); split/leakage (380/100/20, nessuna riga di train in 191–310, holdout esattamente 201–300, digest di righe e label); source holdout separato (4 480 − 6 = 4 474, disgiunzione bitwise da tutte le 500 righe, label S0D riproducibili); ancora provata uguale alla media dei 6 tensori FULL con fattori 0,5 espliciti; guard su 4 token errati; assenza di primitive di esecuzione closed-loop nel sorgente; semantica dei gate su valori sintetici (G_task, Q1, Q3, T2 falliscono ciascuno isolatamente); T1, Q3 sull'init e round-trip save/reload bit-exact.

## 7. Stato
**STOP per audit Codex.** Nessun checkpoint S1 esiste; il prossimo passo richiede una decisione esplicita dell'architetto tra le opzioni §5 e un nuovo token. Invariati: seed 125 held-out, 126–128 sigillati; σ = 0,005 placeholder non deciso; nessun rollout autorizzato.
