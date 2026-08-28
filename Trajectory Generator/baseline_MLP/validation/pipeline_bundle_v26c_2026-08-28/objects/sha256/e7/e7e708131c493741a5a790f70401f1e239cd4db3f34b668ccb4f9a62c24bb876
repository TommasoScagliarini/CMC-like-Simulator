# V26B S1P (rev3p) — fit multi-ruolo eseguito: **entrambi i gate vincolanti FAIL** → REJECTED, nessun checkpoint

**Token:** `V26B-S1P-MULTIROLE-FIT` · **Data:** 2026-08-24 · **Ambito:** solo stage offline. Nessun rollout, DAgger, PPO/critic, V3, morphology; nessuna modifica a env/reward/FSM/SEA/C++/production; nessun artefatto rev3o toccato.

## 1. Esito
L'obiettivo a due ruoli group-balanced (β = 1) ha portato la rete in un **punto di compromesso che non soddisfa né il task né la preservazione**: G_task **[0,39686, 0,28632]** contro 0,15 e Q1 **[0,21460, 0,17226]** contro 0,10. Q3 e T1/T2 PASS. Per contratto: artefatto **REJECTED**, **nessun checkpoint candidato pubblicato**, staging/lock puliti, nessuna soglia/metrica/label modificata, STOP.

| Gate | rev3o (mono-ruolo, β = 0) | **rev3p (multi-ruolo, β = 1)** | Soglia |
|---|---|---|---|
| G_task vs u_IK, holdout out-of-sample 201–300 | [0,13421, 0,12155] **PASS** | **[0,39686, 0,28632] FAIL** (2,6× / 1,9×) | ≤ 0,15 |
| Q1 vs media S0D, source holdout rev3m (4 474 righe) | [0,49825, 0,34616] **FAIL** (5,0× / 3,5×) | **[0,21460, 0,17226] FAIL** (2,1× / 1,7×) | ≤ 0,10 |
| Q3 invarianti | PASS | **PASS** | — |
| T1 / T2 | 7,399e-08 / 9,562e-08 | **7,399e-08 / 1,103e-07** | ≤ 1e-5 |

## 2. Il risultato strutturale: il fronte di Pareto non attraversa la regione ammissibile
I due esecuzioni ora **bracketano** il compromesso. Passando da β = 0 a β = 1 la preservazione migliora 2,3× (knee) e 2,0× (ankle), ma il task peggiora 3,0× e 2,4×. Per portare Q1 da 0,215 a ≤ 0,10 servirebbe β > 1, che spingerebbe G_task **oltre** 0,397 — già 2,6 volte la soglia. Con due punti misurati e la monotonia del trade-off, la conclusione empirica (non una dimostrazione) è che **nessun β su questa classe di ipotesi soddisfa contemporaneamente G_task ≤ 0,15 e Q1 ≤ 0,10**.

**La prova più forte è in-sample.** Il ruolo di preservazione non riesce a fittare nemmeno le **proprie righe di training**: RMSE in-sample vs S0D **[0,20843, 0,17032]**, max-abs [1,157, 0,938], con `pres_mse` piatto (0,0301 all'epoca 1 → 0,0291 all'epoca 300) mentre il task scendeva 0,1006 → 0,0263. Non è un problema di ricetta, di pesi o di ottimizzatore: **la rete 35D→H→H→2 non può rappresentare simultaneamente le due mappe**, perché le due varietà di stati si sovrappongono nello spazio 35D proprio dove i due target differiscono di ~0,38. La causa architetturale diagnosticata dall'architetto (ancora inerte) era corretta ed è stata rimossa; ciò che resta è un **conflitto rappresentazionale**, ora misurato invece che inferito.

Diagnostici a corredo: full-500 vs u_IK [0,23046, 0,16915]; task in-sample [0,16727, 0,11339]; drift vs S0D sulle 500 righe di task mean-abs [0,24195, 0,20075] (contro 0,384/0,289 di rev3o: la preservazione ha effettivamente frenato lo spostamento, ma non abbastanza per Q1 e troppo per G_task); shift parametrico ‖θ−θ_S0D‖² = 22,334 (rev3o: 2,440); baseline S0D vs u_IK sull'holdout [0,62144, 0,45692].

## 3. Verifica del meccanismo (come richiesto dal contratto)
- **Passi di ottimizzazione**: **18 000** = 300 epoche × 60 batch (unione 15 214 righe / batch 256).
- **Masse per ruolo**: media **6,33 righe task** e **247,23 righe preservazione** per batch; totali 114 000 e 4 450 200 righe-viste. Group-balanced: quelle ~6 righe pesano quanto le ~247, esattamente come nel meccanismo T1R.
- **Batch a ruolo vuoto**: **52 su 18 000** (0,29 %) senza righe di task → contributo **esattamente zero** per quel ruolo in quel batch; **0** batch senza righe di preservazione.
- **β = 1,0** congelato a priori dalla ricetta F2R T1R; nessuna normalizzazione per varianza dei target, nessuna testa ausiliaria, nessuna massa.
- **Ancora**: confermata **secondaria e inerte** come diagnosticato — contributo pesato 3,641e-09 all'epoca 300 contro una loss totale di 0,0554. La preservazione è stata portata interamente dal termine di ruolo, come da disegno.
- **Numerica invariata**: 300 epoche fisse, Adam lr 1e-4, seed 2026, algoritmi deterministici, colonne clock ri-azzerate dopo ogni passo, logstd ripristinata e bit-identica all'init, nessun early stopping o model selection.

## 4. Leakage e collisioni (ri-verificati fail-closed all'esecuzione e registrati)
`pres_vs_500_task_states = 0`; `pres_vs_rev3m_source_holdout = 0`; `task_rows_inside_excluded_steps = 0`. Ruolo di preservazione: **14 834 righe bitwise uniche** dal `train_idx` di rev3k, etichette = media deterministica dell'S0D pinnato sugli **stessi** obs35. I tre insiemi (380 task / 14 834 preservazione / 4 474 gate rev3m) sono **a due a due bitwise disgiunti**, quindi nessuna collisione di etichetta è possibile per costruzione. Il gate Q1 resta indipendente: le sue 4 474 righe non sono mai entrate in training.

## 5. Conformità al contratto
Init esclusivamente l'S0D pinnato (actor `481dd0d2…`, module_state `cda6d893…`), enforcement **positivo**. Task = esattamente le 380 righe rev3o (steps 1–190 ∪ 311–500, label u_IK same-time); holdout 201–300 ed embargo esclusi. Nessuna pubblicazione: `student/S1P_IK_AB06_MULTIROLE_35D/` **non esiste**, `student/S1_IK_AB06_35D/` **resta assente**, nessuno staging/lock residuo. Artefatti rev3o riverificati byte-identici (REJECTED `a559bd69…`, receipt rev3n `923ffc5c…`, receipt FAIL rev3l `a437f1ef…`). **Questione σ esplicitamente irrisolta**: 0,005 è solo il placeholder di serializzazione della testa log-std, nessun gate lo legge, non va assunto per alcuno stage stocastico futuro.

## 6. Test
`test_v26b_s1p_multirole.py`: **PASS, 52 check**, eseguito prima del fit e **ri-eseguito identico dopo il FAIL**. Copertura richiesta: pin/tamper su rev3p, sull'artefatto REJECTED rev3o e sul tool rev3o, più l'invariante "rev3o non ha pubblicato nulla"; **5 token errati** respinti (incluso `V26B-S1-FIT` e la variante minuscola); init positivo con rifiuto di un attore diverso della catena; **leakage** (nessuna riga di task in 191–310, zero overlap dei tre insiemi) e **collisione** provata con un set di preservazione **deliberatamente avvelenato** che viene respinto con errore di leakage; **role-weight** (termine di ruolo = MSE piatto sulle sole righe del ruolo, invarianza group-balanced al triplicare delle righe, ruolo vuoto = zero esatto, L = task + β·pres); ancora = media dei 6 tensori FULL con fattori 0,5; semantica di pubblicazione (ciascuno dei 4 gate vincolanti, isolato, blocca la pubblicazione); **no-publish-on-fail end-to-end** con fit stub che restituisce l'init intatto → REJECTED scritto, nessuna directory candidata, nessun residuo; T1/Q3/save-reload; σ dichiarata irrisolta. Un errore nella costruzione di un mio test (maschera non negata) è stato individuato e corretto **prima** del fit; nessun codice di produzione era coinvolto.

## 7. Artefatti
| Artefatto | SHA-256 |
|---|---|
| `…/v26b_bridge_2026-08-24/v26b_amendment_rev3p_s1_multirole_fit.json` | `617440544a6193646b1e5f8e5d3e4d91f56f36e93b9accfe6fbfd5ebbf3d21e6` |
| `…/v26b_bridge_2026-08-24/v26b_s1p_multirole.py` | `22f0e3554023ef03c8c82674d48200000508d9aa197e0d2b2176d02138c75f23` |
| `…/v26b_bridge_2026-08-24/test_v26b_s1p_multirole.py` | `5ca01a244d0b88ef9522b92e6df4c71661c73505855712a385526ac95bbc1d8c` |
| `…/2026-08-24_V26B_anchors_r1/v26b_s1p_fit_REJECTED_20260824_191743.json` | `d5f7b80709ae3fd315cf145d69f750d7601786a490626759159013418bb5d8d0` |
| `…/2026-08-24_V26B_anchors_r1/s1p_multirole_fit_20260824_191700.log` (`REAL_EXIT_CODE=1`) | `ce6228104c670c1240b570bd492226ab2d58a08b36d92edab3bb0cef2b8a05a7` |

## 8. Stato e prossimo stadio sicuro
**STOP per audit Codex.** Nessun checkpoint S1 o S1P esiste; l'unico attore 35D deployabile della catena resta **S0D** (`481dd0d2…`, 500/500 nominale sotto v3). Non esiste uno stadio successivo eseguibile senza una decisione dell'architetto: un rollout closed-loop non ha oggetto (nessun candidato pubblicato) e qualunque nuovo tentativo di adattamento IK richiede una **scelta di modellazione**, non una nuova ricetta di training. Opzioni, tutte da preregistrare con nuovo token: (a) chiudere la linea di adattamento IK e consolidare S0D; (b) cambiare classe di ipotesi (input di contesto/task o testa residua) accettando che cambia il contratto deployabile; (c) rimettere in discussione se u_IK AB06 sia il target giusto, dato che S0D cammina e u_IK non proviene da una policy closed-loop che cammina; (d) ridefinire dove si misura la preservazione — sconsigliata, perché la sovrapposizione fra le due varietà di stati è intrinseca, non un artefatto della scelta dell'insieme.
