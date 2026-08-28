# V26B S1A (rev3q) — materializzato il candidato intermedio **NON deployable**: P0 bit-exact, G_task PASS

**Token:** `V26B-S1A-BC-FIT` · **Data:** 2026-08-24 · **Ambito:** solo stadio offline. Nessun rollout, DAgger, PPO/critic, multistart, σ-sweep, morphology; nessuna modifica a env/reward/FSM/SEA/C++ né ad alcun artefatto/tool/contratto preesistente.

## 1. Esito
Materializzazione **riuscita** (exit 0). Il gate P0 di riproducibilità ha confermato **bit-exact** tutti e cinque i valori rev3o, **digest actor incluso**; i gate vincolanti sono passati; Q1 è stato calcolato e registrato **come diagnostico non vincolante**, secondo la correzione architetturale. L'artefatto è pubblicato fuori da `student/`, marcato non-deployable e in attesa di rollout.

| Voce | Valore | Soglia | Esito |
|---|---|---|---|
| **P0** riproduzione bit-exact (5 elementi, `mismatch: {}`) | vedi §2 | uguaglianza esatta | **PASS** |
| **G_task** RMSE vs u_IK, holdout out-of-sample 201–300 | [0,13421327340983133, 0,12155383791334523] | ≤ 0,15 | **PASS** |
| **Q3** invarianti | 10 chiavi, clock zero, invarianza bit-identica, logstd placeholder **e bit-identica all'init**, no critic | — | **PASS** |
| **T1 / T2** | 7,398672874359136e-08 / 9,561586078454809e-08 | ≤ 1e-5 | **PASS** |
| **Q1** source holdout rev3m (4 474 righe) | [0,49825443853542484, 0,3461596068119536] | 0,10 di riferimento | **DIAGNOSTICO, non vincolante** |

Diagnostici registrati: full-500 vs u_IK [0,08745279, 0,07333816]; drift vs S0D sulle 500 righe di task mean-abs [0,38390109, 0,28908758]; curva di loss identica riga per riga a quella rev3o (epoca 1 `flat_mse` 0,17298733443021774 → epoca 300 0,003900352166965604), riproduzione esatta oltre i cinque elementi P0.

## 2. Gate P0 — perché è il cuore di questo stadio
Prima di qualunque materializzazione il tool ha ricalcolato il fit da zero e preteso uguaglianza **esatta** con i cinque valori registrati nell'artefatto immutabile rev3o `a559bd69…`:
`G_task [0.13421327340983133, 0.12155383791334523]` · `Q1 [0.49825443853542484, 0.3461596068119536]` · `T1 7.398672874359136e-08` · `T2 9.561586078454809e-08` · `actor digest 8f3e0ce17eff7c741dcf72de6d0fec0c372f9dbc7a9b3119d41c155ec8603e35`. Risultato: **`mismatch: {}`**.
Il digest che mi hai pinnato è stato **verificato prima di scrivere codice** contro quello registrato in `a559bd69…`: coincidono. Il test dimostra che P0 intercetta una deviazione di **1 ULP reale** (`np.nextafter`) su ciascuno dei quattro numeri e qualunque digest diverso, con `mismatch` di lunghezza 1.
La riproduzione è esatta **per costruzione**: il fit è eseguito chiamando la funzione rev3o non modificata (`v26b_s1_fit.fit_s1`, tool `a6d7163e…` verificato invariato in lineage), non una reimplementazione.

## 3. Semantica di pubblicazione (la correzione che hai accettato)
Percorso, **fuori da `student/`** e verificato tale in codice e nei test:
`Trajectory Generator/runs/rollout/validation/v26b_bridge_runs/2026-08-24_V26B_anchors_r1/candidates/S1A_IK_AB06_35D_NONDEPLOYABLE`
Receipt **e** manifest contengono entrambi: `deployable: false`, `rollout_pending: true`, `sigma_unresolved: true`, `promotion_requires: "closed-loop nominal rollout under a separate token"`.
Nessuna marcatura o alias deployable: il `contract` è `intermediate_candidate_state_NON_DEPLOYABLE` (la stringa storica `deployable_markov_controller_state` usata dai moduli della catena è **rifiutata** da uno scanner ricorsivo fail-closed, testato). Il receipt riporta esplicitamente che **S0D `481dd0d2…` resta l'unico attore della catena con evidenza closed-loop** (500/500 nominale sotto v3) e include la dichiarazione di estrapolazione: spostamento richiesto ~0,38 contro lo shift RMS 0,004175 dell'analogo di luglio (13/07), con la catena verso u_IK che ha fallito closed-loop 493/242/197. **I gate offline non sono evidenza di viabilità closed-loop.**

## 4. Conformità e stato del repository
Init esclusivamente S0D pinnato (`481dd0d2…`, module_state `cda6d893…`), enforcement positivo. Dati e numerica identici a rev3o: 380 righe (steps 1–190 ∪ 311–500), label u_IK same-time, holdout 201–300 + embargo 191–200/301–310 esclusi, 300 epoche, batch 256, Adam lr 1e-4, seed 2026, deterministico, clock ri-azzerato, logstd ripristinata, anchor 1e-5 dichiarato **regolarizzatore parametrico July-11-faithful** e non meccanismo di preservazione. Nessun retry, nessun tuning.
`student/` invariato (6 directory preesistenti, nessuna aggiunta); `student/S1_IK_AB06_35D/` e `student/S1P_…/` restano **assenti**; nessuno staging/lock residuo; catena rev3l→rev3q e artefatti REJECTED rev3o/rev3p riverificati intatti in lineage a ogni esecuzione.

## 5. Test
`test_v26b_s1a_bc.py`: **PASS 54 check pre-run**, **PASS 66 check post-run** (stage-aware). Copertura richiesta: token positivo (provato indirettamente — con il token corretto l'esecuzione supera il guard e si ferma sulla regola del percorso) e **7 token negativi** inclusi tutti quelli storici (`V26B-S1-FIT`, `V26B-S1P-MULTIROLE-FIT`, `V26B-S1-REV3N-PREREG-MEDIAN`, `V26B-S0D-FIT`, `V26B-S1-PREREG-READONLY`, `None`, minuscolo); pin/tamper dell'intera catena rev3l→q più i tool rev3o/rev3p e gli artefatti REJECTED; leakage/embargo (380/100/20, nessuna riga in 191–310, holdout esattamente 201–300); **P0** con provenienza verificata dall'artefatto rev3o e cinque deviazioni isolate (4 × 1 ULP reale + digest); **Q1 sintetico a 0,90 che NON blocca**; **ciascun gate vincolante che blocca isolatamente** (G_task, Q3, T1/T2, P0); no-publish-on-fail end-to-end con fit stub → REJECTED scritto in temp, nessuna directory candidata, nessun residuo, percorso reale intatto; rifiuto di qualunque percorso sotto `student/`; scanner anti-marcatura deployable (rifiuta `deployable:true` e la stringa storica); clock zero e logstd bit-identici sia sull'init sia sul modulo pubblicato; assenza di primitive closed-loop e di codice morto nel modulo.

**Trasparenza sugli errori miei, tutti trovati e corretti PRIMA dell'esecuzione**: nel tool avevo lasciato tre residui di codice morto (due espressioni `X if False else Y` e un confronto inerte), rimossi e sostituiti con un pin reale del tool rev3p; nel test avevo scritto una falsa perturbazione di 1 ULP (stesso float64) e un `CHECKS += 1` fuori scope. Nessun codice di produzione è stato coinvolto e nessuna soglia è stata toccata.

## 6. Artefatti
| Artefatto | SHA-256 |
|---|---|
| `…/v26b_bridge_2026-08-24/v26b_amendment_rev3q_s1a_bc_intermediate.json` | `74b23e6f5a202b260320718e43dd70f483dc898dc5805d91a28eeb52546bc667` |
| `…/v26b_bridge_2026-08-24/v26b_s1a_bc.py` | `5fc377b0570bd7a9db3b1bca8f6659e391e96a3741323eedaa9a55d593a88fb7` |
| `…/v26b_bridge_2026-08-24/test_v26b_s1a_bc.py` | `b0bf8909bce0206546d5aaeae4def6143aadd966adab2e8f03c4688f23d68828` |
| `…/candidates/S1A_IK_AB06_35D_NONDEPLOYABLE/v26b_s1a_bc_receipt.json` | `0d2aa071458c5ea855c1f33d7fcfb7689bc0560ea08b38321bada7d404eafe5a` |
| `…/S1A_IK_AB06_35D_NONDEPLOYABLE/rl_module/module_state.pkl` | `345539298a9cf369c98f24bc8ebc3e7ea06a112f980e83e4e0607bd7e1264c30` |
| `…/S1A_IK_AB06_35D_NONDEPLOYABLE/rl_module/actor_feature_manifest.json` | `8c3418769adf4cfa8c117e5f1aa94a0e66a674ac16e540f02fc14be332544ffd` |
| `…/2026-08-24_V26B_anchors_r1/s1a_bc_fit_20260824_193445.log` (`REAL_EXIT_CODE=0`) | `6d1c2cbe656678e9afacf334db5cd62affa6e838d385f37250be6ff4c4eb631e` |
| candidate actor digest | `8f3e0ce17eff7c741dcf72de6d0fec0c372f9dbc7a9b3119d41c155ec8603e35` |

## 7. Stato
**STOP per audit Codex.** Nessun rollout avviato, nessun altro stadio toccato. Il prossimo passo è, con **token separato**, un singolo rollout nominale deterministico di S1A col comando canonico rev3c/e/j: non retro-invalida questa materializzazione, ma è il **gate d'ingresso** a qualunque fase di preservazione source==init in stile luglio, perché la costruzione ancore del 13/07 richiede la traccia nominale **completa** dell'attore stesso. σ resta irrisolta: 0,005 è solo placeholder di serializzazione, nessun gate lo legge, e va scelto per lo stadio stocastico (luglio: 0,05 → 0,03 → 0,003 → 0,005; alternativa 11/07 = distillazione della log-std sorgente).
