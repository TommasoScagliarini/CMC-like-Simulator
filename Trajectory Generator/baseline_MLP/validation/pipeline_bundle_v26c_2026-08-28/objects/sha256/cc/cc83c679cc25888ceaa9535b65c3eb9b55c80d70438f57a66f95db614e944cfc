# V26B S1B (rev3t) — fit offline dei 6 candidati congelati: **un solo survivor, A2**

**Token:** `V26B-S1B-FIT` · **Data:** 2026-08-24 · **Ambito:** solo fit offline e materializzazione NONDEPLOYABLE. Nessun rollout, collection, DAgger, PPO/critic, σ o modifica production. Esecuzione unica, nessun retry. `student/` invariato, nessuno staging/lock residuo.

## 1. Esito
Sei candidati eseguiti esattamente come congelati in rev3s. **Offline PASS: solo A2 (r = 5).** Gli altri cinque restano **quarantinati** e non potranno mai essere init, source o ancora di alcuno stadio successivo. Offline PASS significa **ammesso al gate closed-loop**, mai walking.

| ID | r | ep. | verdetto | livello fallito | drift medio (knee/ankle) | drift max knee | holdout RMSE (knee/ankle) | miglioramento (knee/ankle) | ‖Δθ‖² |
|---|---|---|---|---|---|---|---|---|---|
| A1 | 3 | 300 | FAIL | **P** preservazione | 0,0887 / 0,0681 | **0,3773** > 0,25 | 0,4474 / 0,3618 | 28,0 % / 20,8 % | 0,239 |
| **A2** | **5** | **300** | **PASS** | — | 0,0592 / 0,0449 | 0,2384 | **0,5080 / 0,3940** | **18,3 % / 13,8 %** | 0,116 |
| A3 | 8 | 300 | FAIL | **T** target | 0,0389 / 0,0329 | 0,1663 | 0,5457 / 0,4134 | 12,2 % / **9,5 %** | 0,058 |
| A4 | 12 | 300 | FAIL | **T** target | 0,0283 / 0,0220 | 0,1149 | 0,5661 / 0,4264 | 8,9 % / 6,7 % | 0,031 |
| A5 | 20 | 300 | FAIL | **T** target | 0,0171 / 0,0158 | 0,0758 | 0,5865 / 0,4374 | 5,6 % / 4,3 % | 0,014 |
| A6 | 8 | 60 | FAIL | **T** target | 0,0376 / 0,0298 | 0,1514 | 0,5490 / 0,4161 | 11,7 % / **8,9 %** | 0,029 |

**Il vincolo binding è la caviglia.** A3 e A6 superano il 10 % sul ginocchio (12,2 % e 11,7 %) ma falliscono sulla caviglia (9,5 % e 8,9 %): il gate T richiede ≥ 10 % **per giunto** e la caviglia generalizza sistematicamente peggio del ginocchio. A3 manca di mezzo punto percentuale. Nessuna soglia è stata toccata.

## 2. Predizione analitica vs osservato (punto 4)
Il modello di rev3s — `drift = gap/(1+r)`, miglioramento sulle righe di training `= 1/(1+r)` — ha retto con precisione notevole, ed è ora **falsificato o confermato con numeri**:

| ID | drift medio predetto (k/a) | osservato (k/a) | scostamento | miglioramento predetto (train) | osservato (holdout, k/a) |
|---|---|---|---|---|---|
| A1 | 0,093952 / 0,075362 | 0,088650 / 0,068060 | −0,0053 / −0,0073 | 25,00 % | 28,01 % / 20,82 % |
| A2 | 0,062634 / 0,050241 | 0,059229 / 0,044862 | −0,0034 / −0,0054 | 16,67 % | 18,25 % / 13,78 % |
| A3 | 0,041756 / 0,033494 | 0,038937 / 0,032910 | −0,0028 / −0,0006 | 11,11 % | 12,19 % / 9,53 % |
| A4 | 0,028908 / 0,023188 | 0,028341 / 0,021979 | −0,0006 / −0,0012 | 7,69 % | 8,90 % / 6,69 % |
| A5 | 0,017896 / 0,014355 | 0,017085 / 0,015834 | −0,0008 / +0,0015 | 4,76 % | 5,63 % / 4,26 % |
| A6 | 0,041756 / 0,033494 | 0,037563 / 0,029815 | −0,0042 / −0,0037 | 11,11 % | 11,65 % / 8,94 % |

Tre osservazioni registrate, non corrette:
1. **Il drift medio è sistematicamente *sotto* la predizione** (scostamenti da −0,0006 a −0,0073), coerente con l'ancora parametrica 1e-5 e con la convergenza non perfetta: entrambe tirano verso θ_S0D.
2. **Il drift massimo può eccedere la predizione**: A1 osserva 0,3773 contro 0,3580 predetto. Il massimo è dominato da singole righe dove la rete non realizza esattamente l'ottimo per riga; è per questo che A1 fallisce P anche se il modello lo dava già oltre soglia.
3. **Il miglioramento sull'holdout supera quello predetto sulle righe di training sul ginocchio** (fattore ~1,05–1,12) ma **resta sotto sulla caviglia**. Il modello prediceva le righe di training, non l'holdout: lo scarto è atteso ed è riportato tale e quale.

## 3. Il candidato A2 in dettaglio
- **Integrità (I)**: source==init provato, 10 chiavi, colonne clock a zero, invarianza clock bit-identica, logstd bit-identica all'init, save/reload esatto, no critic, **T1/T2 max 1,520e-07** ≤ 1e-5.
- **Preservazione (P)**: mean|Δ| [0,0592, 0,0449] ≤ 0,10; RMS [0,0771, 0,0566] ≤ 0,15; max|Δ| [0,2384, 0,1813] ≤ 0,25. Per **ogni** strato discreto presente: `online_left_in_contact` [0,039, 0,037], `online_left_heel_strike` [0,084, 0,058], `online_left_toe_off` [0,030, 0,010], `phase_fsm_stance_after_hs` [0,034, 0,040], `phase_fsm_swing_after_to` [0,081, 0,049], `phase_expected_hs` [0,081, 0,049], `phase_expected_to` [0,034, 0,040] — tutti ≤ 0,10: **nessuna fase del passo è stata sacrificata in silenzio**.
- **Target (T)**: holdout [0,5080, 0,3940] contro baseline S0D [0,6214, 0,4569] → 18,3 % / 13,8 %, entrambi ≥ 10 %. In-sample sulle righe di training [0,4033, 0,3087], riportato distinto.
- **Deriva (D)**: ‖Δθ‖² = 0,1160 ≤ 0,5; drift medio azioni sulle 500 righe [0,0657, 0,0455] ≤ 0,10 (max [0,2417, 0,1895]).
- Fit: λ_a = 5,0, λ_t = 1,0, 300 epoche, **900 passi**, unione 760 righe (380 + 380), **0 batch senza ruolo ancora e 0 senza ruolo task**, `no_hard_blend: true`. Loss: 0,17867 → 0,15190, con il ruolo task 0,16274 → 0,12824 e il ruolo ancora 0,00318 → 0,00473.
- Actor digest `cde2c8e6356f833d9e1108b542286c2c9265f8f69bb4c0a894c36136c5d0916c`.

## 4. Verifica del meccanismo a due ruoli (punto 3)
La loss implementata è `λ_a·mean_ancora + λ_t·mean_task + 1,0·clip + 1e-5·ancora parametrica` con λ_t = 1 e λ_a = r, e le medie per-ruolo sono calcolate **separatamente dentro ogni mini-batch** estratto da un'unica unione mescolata in modo deterministico dei due blocchi da 380 righe. Il self-test lo dimostra su valori sintetici: per r ∈ {3, 5, 8, 20} la loss è esattamente `r·mean_ancora + mean_task` (entro 1e-12 sui termini di ruolo), **duplicare le righe di un ruolo dentro il batch non cambia né la sua media né il rapporto r**, e un ruolo assente contribuisce esattamente zero. Nell'esecuzione reale nessun batch è mai risultato privo di un ruolo (0 e 0). **Nessuna scorciatoia hard blend**: il fit è discesa del gradiente a due ruoli, anche se l'ottimo per riga coincide analiticamente con la miscela — il test verifica anche l'assenza della scorciatoia nel sorgente.

## 5. Confronto con S0D, S1A e luglio
| Attore | Drift azioni vs S0D | Holdout vs u_IK | Evidenza closed-loop |
|---|---|---|---|
| **S0D** | 0 (riferimento) | 0,6214 / 0,4569 | **500/500, 2 cicli, 13,7 mm** |
| **S1A** (nessuna ancora) | 0,384 / 0,289 | 0,1342 / 0,1216 | **392/500, 0 cicli, 24,7 mm** |
| **S1B A2** | **0,0657 / 0,0455** | 0,5080 / 0,3940 | **nessuna** (rollout non autorizzato) |
| July 13/07 | shift RMS 0,004175 | n/a (target consistente) | 500/500 ×3 start, 2 cicli |

A2 sta esattamente dove il protocollo lo voleva: **~6× più vicino a S0D di quanto lo fosse S1A**, e ~16× più lontano di quanto lo fosse il raffinamento di luglio. È un passo controllato in una direzione che, presa per intero, distrugge il cammino; se la distrugga già a questa ampiezza è precisamente ciò che il gate closed-loop dovrà dire. **Nessuna metrica di questo report afferma che A2 cammina.**

## 6. Blocker e limiti dichiarati
- **Il gate T è il collo di bottiglia, e per la caviglia.** Con la banda ammissibile derivata a priori (r ∈ [4,727, 9]) sopravvive solo r = 5: r = 8 cade per 0,5 punti percentuali sulla caviglia. La finestra reale è quindi ancora più stretta di quella dichiarata in rev3s, ed è **una sola configurazione**. Non ho toccato né la soglia né la griglia.
- **Il blocker strutturale di rev3s resta intatto**: ancore e task sugli stessi 380 stati ⇒ sulle righe di training l'ottimo è la miscela convessa. È confermato empiricamente dalla precisione delle predizioni. La correzione dell'architetto è registrata: raccogliere azioni S0D alt-start come *sole ancore* **invertirebbe** i ruoli di luglio (dove erano gli alt-start a portare le label teacher e il nominale a portare le ancore), quindi non è di per sé una replica July-faithful — resta un ramo futuro possibile, non un prerequisito.
- **Costo residuo invariato**: A2 lascia l'holdout a 0,508/0,394, lontanissimo dal criterio 0,15. Chiudere il gap richiederebbe iterazioni successive, ciascuna con i propri token.

## 7. Test
`test_v26b_s1b_fit.py`: **PASS 43 check pre-run**, **PASS 69 post-run** (stage-aware). Copertura richiesta: pin/tamper rev3l→rev3t con verifica che tool e test rev3s sono byte-identici (additività); **6 token negativi** più le guardie di rollout e collection, e il token positivo provato **senza fittare** (con `out_dir_for` reindirizzata a una directory esistente il no-clobber scatta subito dopo la lineage, non scrive nulla); griglia immutabile (id, r, epoche) e numerica congelata per riferimento; **init S0D fresh ed esatto per candidato**, con prova che una copia mutata non contamina il caricamento successivo; **loss-ratio** su quattro r e su composizioni di batch diverse, ruolo assente = zero, assenza di hard blend nel sorgente; **ripetibilità deterministica** su un percorso micro (due fit da 1 epoca, 3 passi, bit-identici) senza raddoppiare i fit reali; save/reload bit-exact, logstd ricostruita bit-identica all'init, clock a zero, 10 chiavi; percorsi sotto `candidates/` e mai `student/`; assenza di primitive closed-loop, di collection e di tiling; post-run: coerenza receipt/manifest/verdetto/quarantena per tutti e sei, digest del modulo, e **rifiuto di una seconda esecuzione**.

## 8. Artefatti
| Artefatto | SHA-256 |
|---|---|
| `…/v26b_bridge_2026-08-24/v26b_amendment_rev3t_s1b_fit_execution.json` | `2002b68740b78435458a55a503e9161d1205c2acb0e9d6a3fd332466128bb8c2` |
| `…/v26b_bridge_2026-08-24/v26b_s1b_fit.py` | `6c63bf8be54034148441cdb6dae004e14991dba400aa66ca239523b37fcb265a` |
| `…/v26b_bridge_2026-08-24/test_v26b_s1b_fit.py` | `258e2eda82b1709813830b641f6ce57c12ef413aa2139601815f98f03a4f6d1a` |
| `…/candidates/v26b_s1b_fit_aggregate_20260824_201458.json` | `0539475cda88030e0af059b394974938f64a03b80490134295ca0ed486ae47e6` |
| `…/candidates/S1B_A1_35D_NONDEPLOYABLE/v26b_s1b_fit_receipt.json` (quarantinato) | `053a4fe1fb4c0aad5da7ba306ae9758cc06a7f6093661623ee259ce089160712` |
| `…/candidates/S1B_A2_35D_NONDEPLOYABLE/v26b_s1b_fit_receipt.json` (**survivor**) | `5430115d327ca9c0acd4ed992b7026bf4a940555544c8b81414fb72dfab78616` |
| `…/candidates/S1B_A3_35D_NONDEPLOYABLE/v26b_s1b_fit_receipt.json` (quarantinato) | `8570271c6703e7bddab1a782833446c932879b306b1409861886b0e149698142` |
| `…/candidates/S1B_A4_35D_NONDEPLOYABLE/v26b_s1b_fit_receipt.json` (quarantinato) | `5fe14964b2ab80da42665be0cb096f4d77c8a822895ea27084339323842bf613` |
| `…/candidates/S1B_A5_35D_NONDEPLOYABLE/v26b_s1b_fit_receipt.json` (quarantinato) | `d95db1cdef18a14f256b9f2f38a7eaaf98459e485c126adf109704ac50285e41` |
| `…/candidates/S1B_A6_35D_NONDEPLOYABLE/v26b_s1b_fit_receipt.json` (quarantinato) | `af9634406471c61dc040ef5c581a894016e0740063d9bef093fe32385f109024` |
| `…/S1B_A2_35D_NONDEPLOYABLE/rl_module/module_state.pkl` | `57254b82b92b7cb5ea0d3b51805c3e8724a2fa9dd18c08c8b12353a1200d2908` |
| A2 actor digest | `cde2c8e6356f833d9e1108b542286c2c9265f8f69bb4c0a894c36136c5d0916c` |
| `…/2026-08-24_V26B_anchors_r1/s1b_fit_20260824_201448.log` (`REAL_EXIT_CODE=0`) | `5a07f7a40e4f01731d6c5c496ab975e03b40ea53a9faccebd678a697b7761e35` |

Tutti i sei candidati portano nel receipt e nel manifest `deployable:false`, `rollout_pending:true`, `sigma_unresolved:true`, `promotion_requires: "closed-loop nominal rollout under a separate token"`; i cinque falliti portano inoltre `quarantined:true` e `may_not_be_source:true`. Nessun attore promosso: **S0D resta l'unico della catena con evidenza closed-loop**.

## 9. Stato
**STOP per audit Codex.** Lista ordinata dei soli survivor offline per il futuro gate closed-loop, nell'ordine congelato di rev3s ristretto ai sopravvissuti: **[A2]**. Il token `V26B-S1B-NOMINAL-ROLLOUT` non è concesso e non è stato invocato. σ resta irrisolta e non operativa.
