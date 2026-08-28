# V26B-REV4D-REPEAT — esperimento decorrelato sull'operatore di tiling

**Data**: 2026-08-24 · **Token**: `V26B-REV4D-REPEAT`
**Emendamento**: `v26b_amendment_rev4d_repeat.json` — SHA-256 `34f8a08ee3fbdb1a23ec5d87cfde6fb07562fef0733e32debdc49532fedc2f32`
**Esito**: preflight GO · fit e gate offline **PASS** · rollout eseguito · **gate primario FAIL (44 ≤ 116)** · candidato **NON promosso**.

---

## 1. Risultato in una riga

Tenendo la copertura negativa al massimo (65 indici distinti) e abbassando la quota on-policy dal 75,82% al 43,95%, la sopravvivenza è **44 step** contro i **42** di rev4b. Il cambiamento è nullo. **La quota on-policy non è la variabile operativa.**

Il risultato cade nella cella `about_42` della griglia preregistrata. Ne segue una conclusione che rovescia la lettura di rev4c: il miglioramento a 116 step **non veniva dal bilanciamento** ma dall'aver **tagliato il prefisso**.

---

## 2. Disegno eseguito

REV4D differisce da REV4B in **una sola dimensione**: `trace_repeat` 4 → 1. Prefisso, copertura, labeling, init, ancora, operatore e iperparametri di luglio byte-identici.

| | prefisso | repeat | interp | on-policy | aggregato | quota | indici neg. distinti |
|---|---|---|---|---|---|---|---|
| rev4b | 392 | 4 | 0 | 1568 | 2068 | 75,8221% | 65 |
| **REV4D** | **392** | **1** | **0** | **392** | **892** | **43,9462%** | **65** |
| rev4c | 68 | 4 | 0 | 272 | 772 | 35,2332% | 9 |

Lineage esclusiva rispettata: catena V26 agosto imitation 39D → student causale 35D; init = ancora = policy di collection = attore S1A `8f3e0ce1…`; nessuna nuova collection. Gli artefatti di luglio compaiono **solo** come benchmark e riferimento di protocollo, mai come init e mai come label.

## 3. Preflight bloccante (no-write) — GO

| item | esito |
|---|---|
| step 1..392 contigui | true |
| obs35 esatta vs vettori registrati, larghezza 35, finita | true |
| allineamento temporale `max abs(Δt)` | **0.0 s** |
| label finite same-step `teacher_index = step − 1` | true, SHA `b8f3f639f502ed394c702c9d087e6b39e225ae692d3a38986d423fbfdaf8b02e` |
| duplicati nel prefisso | 0 |
| collisioni bitwise / con label in conflitto | 2 / **0** |
| copertura negativa | **65 indici distinti**, finestre `[6,14] [112,132] [174,179] [267,290] [330,334]`, 97 nel corpus |
| formula corpus | 500 + 392×1 = **892** |
| quota on-policy | **392/892 = 0,43946188340807174** |

## 4. Fit — iperparametri di luglio invariati

Nessun iperparametro ridefinito: il modulo riusa `v26b_rev4b_dagger.fit_july` e le costanti `J_*`.

| parametro | valore |
|---|---|
| epoche richieste / eseguite / migliore | 400 / **318** (early stop) / **258** |
| batch, lr, val, patience | 64, 3e-4, 0.20, 60 |
| clip / logstd / anchor | 1.0 / 0.1 / 1e-5 |
| seed, grad clipping | 123, nessuno |
| righe train / validazione | 714 / 178 |
| best val MSE | 0,012959456071257591 |
| loss epoca 1 → 318 | train 0,100621 → 0,004859; val 0,085607 → 0,016762 |

σ resta **UNRESOLVED**: placeholder 0,004999999670722372, logstd byte-identica all'init. `logstd_weight = 0.1` è un peso di loss e non è una scelta di sigma.

**Nota sulla validazione**: con `repeat = 1` non esistono copie esatte a cavallo dello split, quindi questa MSE è *pulita*. Vale 0,01296 contro lo 0,00193 di rev4c, che era gonfiato dalla contaminazione (≈ 40 righe uniche su 68 con copie su entrambi i lati). Il confronto conferma quantitativamente l'analisi dell'addendum rev4c. La MSE resta comunque **diagnostica**, mai criterio.

### Gate offline vincolanti — tutti PASS

| gate | misura | esito |
|---|---|---|
| invarianti di integrità | 10 chiavi, clock zero, invarianza bit-identica, logstd byte-identica, save/reload esatto, nessun critico | **PASS** |
| preservazione di funzione | T1 3,972e-08, T2 1,015e-07 contro 1e-05 | **PASS** |
| convergenza | RMSE aggregato 0,360737 → **0,083245** | **PASS** |

### Misure senza soglie inventate

| insieme | knee RMSE / r / ampiezza | ankle RMSE / r / ampiezza |
|---|---|---|
| corpus 500 | 0,067293 / 0,98138 / 1,00094 | 0,041604 / 0,97994 / 0,99080 |
| prefisso completo 392 | 0,130303 / 0,92549 / 0,84546 | 0,081083 / 0,91353 / 0,88666 |
| **97 righe negative del corpus** | 0,063995 / 0,99209 / 1,02533 | 0,046706 / 0,83944 / 1,08394 |

Segno finestre negative: comando positivo 0,26804 → 0,25773; comando minimo caviglia −0,16650655573041748.
`max_abs` vs attore sorgente [1,50435 / 1,22361] **INFORMATIVO**; simmetria controlaterale [0,10841 / 0,13601] **DIAGNOSTICA**.

## 5. Rollout — uno solo, nominale deterministico, nessun retry

`returncode` 0, durata 44,134 s, comando canonico v3 con offset 1.956870983805102, `deterministic`, seed 123.

### Gate primario

| misura | valore |
|---|---|
| sopravvivenza | **44 step** |
| soglia | strettamente > 116 |
| **verdetto** | **FAIL** |

Riferimenti: rev4b (392/repeat4) 42 · rev4c (68/repeat4) 116 · S1A init 392 · benchmark luglio r1 45, r2 356, r3 221.

### I sette gate

| gate | valore | esito |
|---|---|---|
| completamento 500 | 44 step, `end_reason = grf_penetration` | **FAIL** |
| phase_timeout stance / swing | 0 / 0 | PASS |
| morphology causal contract failure | 0 righe positive su 44, max 0.0 | PASS |
| hs_cancelled | 0 | PASS |
| resync | 0 | PASS |
| ≥ 1 ciclo valido | 0 (0 HS, 0 TO, nessun evento) | **FAIL** |
| penetrazione | 0,02910222009566334 m vs limite 0,020 | **FAIL** |

Prosa dei gate correttamente etichettata *"closed-loop eligibility of REV4D"*, con dichiarazione esplicita che l'aritmetica proviene verbatim dal tool congelato S1B e che solo la stringa descrittiva è stata parametrizzata in scrittura.

### Cinematica

- knee q ∈ [−0,22273, −0,16684], media −0,18666, frazione negativa 1,0, 0 step fuori bounds.
- ankle q ∈ [**+0,02345**, +0,05233], media +0,03957, **frazione negativa 0,000**, 0 step fuori bounds.
- Penetrazione max 0,029102 m / media 0,009643 m; riserve max 491,55 N·m / media 281,54 N·m.
- Clipping azione 0 step; stato finale `STANCE_AFTER_HS`.

## 6. Interpretazione

REV4D riproduce rev4b quasi esattamente: 44 step contro 42, `grf_penetration` in entrambi i casi, e una caviglia che **non plantarflette mai** (min +0,02345 contro +0,02397 di rev4b). Aver quartuplicato o non quartuplicato il peso del prefisso è indifferente.

Ne seguono tre conclusioni, nell'ordine di robustezza:

1. **La quota on-policy non è la leva.** 75,82% → 43,95% a copertura costante: 42 → 44 step. Questo chiude la domanda che rev4c aveva lasciata aperta.
2. **Nemmeno la copertura negativa aiuta — la sua direzione è opposta a quella attesa.** Copertura 65 dà 42 e 44 step; copertura 9 dà 116. Più copertura negativa on-policy peggiora l'esito. La cella `above_116` della griglia è quindi esclusa.
3. **La variabile operativa è la lunghezza del prefisso**, e agisce nella direzione dannosa. Le righe 69..392 sono quelle in cui S1A si è già allontanato dalla varietà del teacher: etichettarle con l'azione teacher allineata nel tempo insegna una correzione che non è più appropriata allo stato visitato. Includerle, a qualunque peso, trascina l'attore. È coerente con luglio, dove R3 aggiunse la traccia lunga da 356 righe e **regredì** da 356 a 221 step.

**Precisazione sul testo preregistrato**: la cella `about_42` era formulata come «non la quota né la copertura ma l'operatore di tiling a prefisso pieno è dannoso». Con `repeat = 1` il tiling non avviene, quindi la lettura esatta è «**il prefisso pieno è dannoso indipendentemente dall'operatore**». L'assegnazione della cella non è ambigua; la sua formulazione lo era, e la correggo qui.

## 7. Stato

- **Candidato NON promosso** e NON-DEPLOYABLE. 3 dei 7 gate falliti; nessuna promozione automatica in ogni caso.
- Nessun PPO, test su sigma, altro candidato, nuova collection o modifica a production/env/reward/FSM/morfologia/SEA/C++.
- Artefatti rev4b e rev4c non mutati.

## 8. File e SHA-256

**Preregistrazione e tooling**

| file | SHA-256 |
|---|---|
| `v26b_amendment_rev4d_repeat.json` | `34f8a08ee3fbdb1a23ec5d87cfde6fb07562fef0733e32debdc49532fedc2f32` |
| `v26b_addendum_rev4c_a_corrections.json` | `377fd4e0ff68fc9135f769d8ce2f23935ec9977e185feaa92624360e1942d9ad` |

**Candidato** `candidates/REV4D_REPEAT_35D_NONDEPLOYABLE/`

| file | SHA-256 |
|---|---|
| `v26b_rev4d_receipt.json` | `cc62fd7013a9acca20bc8cd45b0c3a7f976e263c96fc3392bf24b1a7dd7ca263` |
| `rl_module/module_state.pkl` | `4d712293cb1762639d2ba31ed81141c2c5ff4bb9744722af15151d0d4c6f4fc8` |
| `rl_module/actor_feature_manifest.json` | `f5c458673873ed0686d83d7d0d5c7d445bb381e3dc6d9fece67da04127e95064` |
| `rl_module/metadata.json` | `3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12` |
| `rl_module/class_and_ctor_args.pkl` | `c9a6722ff95642795bfe1146d0087a68b5861fd508cbe3692195b2d820d810a7` |

**Rollout** `rollouts/rev4d_nominal_det/REV4D_35D__v3_canonical__nominal__det/`

| file | SHA-256 |
|---|---|
| `v26b_rev4d_rollout_receipt.json` | `684c46854030a046ac4114b6823e308850b4f3660e123df403e8cac54ae612d7` |
| `rollout_policy_trace.json` | `b2709924b01ad158e56bcd6081e9d1f02d85b68eccbb5daaf259c3abd1650ec1` |
| `rollout_summary.json` | `9408111fd35ddeac8aaac48d0e1db6312114e3fb83fc2d84355e916d819f38bf` |
| `rollout_reset_diagnostics.json` | `3d3b8dfa9164b2e63f0ba6ca795f2abee41ef6c8bd8929e19368ad83a325ed76` |
| `watchdog_summary.json` | `8f69aead520208ae66a0facf13d5e25b2740a5bfc1ae076fde89da728c877b05` |
| `watchdog_state.json` | `d64cea896cba76cd8363c53d050b2646762fa289cc17280a9bc145e92bf10db3` |

Log di stadio `rev4d_stage_20260824_225451.log` — `3eb59410aa350c8ff3f6386dedb4a5c39ac83d4a9258dcee13c558ce266af3f2`.

## 9. Test e verifiche

- Self-test pre-esecuzione 65/65 PASS; **post-esecuzione 77/77 PASS** con `offline_pass: true`, `survival_steps: 44`, `primary_gate: "FAIL"`.
- Igiene delle etichette estesa alla **prosa**: la stringa non riparametrizzata del tool congelato S1B (*"eligibility of A2"*) viene ora **rifiutata** dal contratto; la regressione lo dimostra, insieme al fatto che il tool congelato resta intatto su disco.
- Prova della variabile singola: `repeat` 1 qui contro 4 in rev4b; `B4.fit_july` riusato; nessun iperparametro di luglio ridefinito; blocco on-policy verificato pari al prefisso **senza tiling** e privo di duplicati esatti.
- Fail-closed: `repeat = 2`, `prefix = 391` e una copertura attesa errata fanno fallire il preflight; token errati rifiutati; seconda esecuzione rifiutata con `FileExistsError`.
- Nessun macchinario PPO o sigma (verifica su identificatori reali); esattamente una invocazione di harness.

## 10. TODO propagati

- **TODO-1** — *Chiuso per rev4d*: la parametrizzazione dell'etichetta d'attore copre ora chiavi e prosa. Resta aperto per i receipt storici, che non vanno mutati.
- **TODO-2** — σ = 0.005 placeholder non risolto. *(aperto, ereditato)*
- **TODO-3** — B3 indeterminato: campo di fase identicamente zero su ogni traccia v3. *(aperto, ereditato)*
- **TODO-4** — Conflitto cammino/plantarflessione irrisolto. *(aperto, ereditato)*
- **TODO-5** — Tensione aritmetica interpolazione↔quota a copertura piena (≥ 61,06%). *(aperto)*
- **TODO-6** — Nuovo: la lunghezza del prefisso è la variabile operativa e agisce in direzione dannosa. Un eventuale round successivo dovrebbe variare **quella**, non il bilanciamento. *(nuovo)*
