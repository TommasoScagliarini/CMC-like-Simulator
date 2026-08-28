# V26B L20 (rev4a) — rollout nominale deterministico: **500/500, tutti e 7 i gate PASS**, e L20 è il miglior attore della catena sul target IK

**Token:** `V26B-L20-NOMINAL-ROLLOUT` · **Data:** 2026-08-24 · Un solo lancio, nessun retry, nessun fit, nessuna promozione, σ non scelta, nessuna modifica a production/FSM/reward/env/morphology/SEA. Esito: **`CLOSED_LOOP_ELIGIBLE_PENDING_ARCHITECT_AUDIT`**.

## 1. Correzione documentale obbligatoria (registrata, e verificata)
La mia tesi secondo cui «costruire DAgger su un parent che non cammina è smentito dalla catena» è **respinta** ed è ora corretta agli atti in rev4a. La ricostruzione autoritativa:

- a luglio il clone BC sopravviveva ~68 step e **il suo prefisso fu usato con successo** nel round 1 di DAgger; S1A ne fornisce **392**;
- la degradazione R0a 493 → R1 242 va attribuita al **protocollo specifico divergente** — 300 epoche senza early stop, batch 256, lr 1e-4, seed 2026, logstd placeholder, **anchor 1e-3**, testa ausiliaria di fase — **non** trattata come confutazione di un DAgger July-style autentico;
- **verifica mia**: l'anchor 1e-3 è **confermato**, i receipt `V2_R0a` e `V2_DAGGER_R1` registrano entrambi `0.001`; le altre voci coincidono con la numerica congelata registrata negli stessi receipt. Per contrasto, l'11/07 usava 400 epoche **con** early stopping su validation (best epoch 368) e aggregava il prefisso da 68 step;
- **stato autoritativo**: un DAgger July-faithful da un parent non camminante o parzialmente camminante **non è confutato** e resta architetturalmente aperto;
- **regola in avanti, registrata prima di conoscere l'esito**: se L20 fallisce, il prossimo candidato architetturale **resta** un DAgger July-faithful sul prefisso S1A (392 step); se passa, decidi tu dopo l'audit.

## 2. Esito dei gate — tutti e sette
| Gate vincolante | Osservato | Richiesto | Esito |
|---|---|---|---|
| completion | **500/500**, `episode_time_limit` | 500/500 + time limit | **PASS** |
| phase_timeout (tutte le righe) | stance 0, swing 0 | 0 e 0 | **PASS** |
| morphology_causal_contract_failure (per riga) | rows_positive 0, max 0,0, false | 0 / 0 / false | **PASS** |
| hs_cancelled_count | max 0, final 0 | 0 e 0 | **PASS** |
| resync_count | max 0, final 0 | ≤ 1 e ≤ 1 | **PASS** |
| valid_cycle_count | **2** | ≥ 1 | **PASS** |
| penetrazione | **0,016681 m** | ≤ 0,020 m, nessuna terminazione | **PASS** |

Contatori FSM registrati senza reinterpretazione: **HS 3, TO 3, cicli 2, invalid_event 0**, stato finale `STANCE_AFTER_HS`, **nessun evento** — L20 è più pulito di S0D e A2, che avevano entrambi un `to_too_early_after_hs` benigno (step 495 e 496). Return **+39,37**, 2 step clippati.

## 3. Qualità: L20 è il miglior attore della catena rispetto al target IK protesico
Confronto nella convenzione corretta stabilita da rev3v (protesico vs IK protesico; il segnale sano resta solo diagnostica di simmetria):

| | knee RMSE | ankle RMSE | knee r | ankle r | knee amp | ankle amp | ankle sign agr. |
|---|---|---|---|---|---|---|---|
| S0D | 0,12639 | 0,13458 | 0,9175 | 0,7794 | 0,9119 | 0,6982 | — |
| A2 | 0,12279 | 0,15229 | 0,9158 | 0,7898 | 0,8331 | 0,7246 | — |
| **L20** | **0,11713** | **0,11827** | **0,9189** | **0,8338** | 0,8737 | **0,8016** | **0,806** (valido) |

**L20 è il migliore su tutte e quattro le metriche primarie**, ed è il primo attore per cui l'accordo di segno rispetto al target IK è **non degenere e quindi calcolabile** (0,806). Sul gate rev3v **G2** (non-regressione vs S0D sul target IK, baseline [0,12639, 0,13458]) **L20 passerebbe su entrambi i giunti** — dove A2 falliva sulla caviglia (0,15229). Su **G4** (forma) passerebbe: Pearson sopra baseline − 0,02 su entrambi, ampiezze 0,874 e 0,802 dentro [0,80, 1,25].

**Ma su G3 fallirebbe**, e lo dico chiaramente: la distanza dal riferimento sano di simmetria peggiora (knee 0,15103 contro il limite 1,05 × 0,12057 = 0,12660; ankle 0,06354 contro 0,05001). È l'effetto atteso e già dimostrato in rev3v: muoversi verso l'IK allontana dal segnale sano, perché i due riferimenti distano 0,1348 sulla caviglia.

**Cinematica**: ginocchio [−1,00725, −0,17420], media −0,45888, sempre negativo, zero step fuori bounds; caviglia [+0,00615, +0,43818], media +0,20861, **mai negativa**. Il minimo di caviglia +0,00615 è però il **più vicino allo zero** mai raggiunto dalla catena (S0D +0,01528, A2 +0,03056).

## 4. Il segno resta non recuperato — come avevo avvertito
Sulle 97 righe della sua traccia dove il target IK è negativo, **l'88,66 % ha ancora comando di caviglia positivo**. B3 resta **non valutabile** (campo di fase identicamente zero, 0 righe nella finestra) e il minimo complessivo +0,00615 è riportato come diagnostico distinto, mai come B3. Questa prova risponde a «quanto ci si può spostare restando in cammino», **non** a «si può recuperare la spinta».

## 5. Che cosa questa prova stabilisce
Il bracket del cammino era: A2 (drift azione vs S0D 0,0657 / 0,0455) cammina; S1A (0,384 / 0,289) non cammina; in mezzo nulla. Ora: **L20 cammina con drift 0,0646 / 0,1022 sugli stati che visita** (0,0649 / 0,0815 in offline). Il confine è quindi **sopra 0,10 e sotto 0,289** sulla caviglia — e il guadagno di qualità in quell'intervallo è reale e misurabile, non marginale: la caviglia passa da 0,13458 (S0D) a 0,11827 di RMSE contro il target, con correlazione da 0,779 a 0,834.

## 6. Conformità
Demozione di `max_abs` a informativo **limitata a questa decisione di eligibility**: gli esiti storici **non sono riscritti** — il receipt di L20 registra ancora `OFFLINE_FAILED_QUARANTINED` con violazione `['max_abs>0.25']` e l'aggregate rev3z registra ancora tutti e tre i candidati come falliti, entrambi riverificati byte-identici (`03802ead…`, `8836e8b4…`). Nessun altro gate toccato. Harness identico a quello dei rollout S0D/S1A/A2: il comando differisce da quello congelato per A2 **solo** in `--checkpoint` e `--output-dir`, verificato fail-closed; pin di produzione confermati (`rollout_eval` `5433bcbc…`, YAML v3 `a870cc38…`, corridor `33b1dd7c…`). Media deterministica, **nessun sampling e nessuna scelta di σ**. Candidato byte-immutabile prima e dopo, flag mai toccati. `student/` invariato, nessuna promozione, nessuna ancora costruita. `osim_trj_cmc_like.py` `26458424…` invariato.

## 7. Test
`test_v26b_l20_rollout.py`: **PASS 51 pre-run**, **PASS 59 post-run**. Copertura: pin/tamper su quattro digest; **prova che gli esiti storici non sono riscritti** (receipt L20 e aggregate rev3z riletti e verificati); registrazione della correzione DAgger con la verifica dell'anchor 1e-3 e la regola in avanti; equivalenza harness con prova che una deviazione ulteriore viene rifiutata; candidato read-only e non-deployable; gli altri candidati non lanciati; 5 token negativi e token positivo provato alla guardia no-clobber; contatori whole-trace su fixture con eventi invisibili nell'ultima riga; **fixture isolata per ciascuno dei 7 gate**; assenza di fit, ancore e marcature deployable; σ dichiarata irrisolta; e post-run l'immutabilità del candidato, il digest della traccia e il rifiuto di un secondo lancio.

## 8. Artefatti
| Artefatto | SHA-256 |
|---|---|
| `…/v26b_amendment_rev4a_l20_nominal_rollout.json` | `014b922a44168c5853bd9e1a9549fae914cf676f6c9f15fc549526014449db6e` |
| `…/v26b_l20_rollout.py` | `01ea3478264f24be82760e3e4651c122f67485ce2a69c010dfdff23adeb546a0` |
| `…/test_v26b_l20_rollout.py` | `fb4aaf483f34fea769031f96587243123f4c000522cee44ad27097c44757e787` |
| `…/rollouts/l20_nominal_det/S1C2Z_L20_35D__v3_canonical__nominal__det/v26b_l20_rollout_receipt.json` | `67230964dbdcacff8a7a9398e41be6557748b82be380ed5aea116cba31b1a837` |
| `…/rollout_policy_trace.json` | `5b9371de1f7f32ef3919598dfe0621ca3d779cabd3b3b93cb4d72ea5bde6e7c3` |
| `…/rollout_summary.json` | `61c2ea44a122b259f62d5352c0592ab3fa86930947f4be3e8889f84576e50c3c` |
| `…/logs/l20_nominal_det_rollout.log` | `2446cc97111582088cb4bfa19748884625f221379517c414e6fa3e4e2993f324` |
| L20 actor digest | `71d21f309ccf1df7bf8aac60cbf8d1c4322586a6f0fe959f8f522624ee23db55` |
| receipt fit L20 (**invariato**) | `03802eadd3fc6371a1f959116b44694d5736ced59ff056e6e4162aae36ce71a9` |
| aggregate rev3z (**invariato**) | `8836e8b49e17055cebe53dff5de592bf7ca7f427cca18aa35b371b1c3b6956dc` |

## 9. Stato
**STOP per il tuo audit.** L20 è marcato **soltanto** `CLOSED_LOOP_ELIGIBLE_PENDING_ARCHITECT_AUDIT`: non promosso, resta NON-DEPLOYABLE, nessuna ancora costruita, σ irrisolta, B3 INDETERMINATE. Nessun altro stadio avviato.

Tre elementi per la tua decisione, senza sovrainterpretare: **(i)** la trust region può essere allargata — a drift caviglia 0,10 l'attore cammina, con FSM più pulita di S0D e A2 e con la migliore qualità della catena verso il target IK; **(ii)** il segno **non** è recuperato (88,7 % di righe di finestra ancora positive), quindi resta valida la prova di incompatibilità di S1C-3: servono spostamenti > 0,25 su 42 righe, ben oltre l'intervallo appena esplorato; **(iii)** G3 fallirebbe, cioè il guadagno verso l'IK si paga in simmetria — un compromesso che ora è quantificato e che spetta a te arbitrare.
