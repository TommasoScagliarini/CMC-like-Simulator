# V26B-REV4E-R2-REPLAY — replica del protocollo luglio R2 sul lineage V26 corrente

**Data**: 2026-08-24 · **Token**: `V26B-REV4E-R2-REPLAY`
**Preregistrazione**: `v26b_amendment_rev4e_r2_replay.json` — SHA-256 `6caf9b8b547d53ac6ce856de9339ab616a0d34191a7b33b1b4a82c531d38eb3f`
**Esito**: preflight **GO** · fit e gate offline **PASS** · rollout eseguito · gate primario **PASS (372 > 116)** · marcatore secondario **PASS (1 ciclo valido)** · **5 dei 7 gate superati** · candidato **NON promosso**.

---

## 1. Risultato

**372 step** contro i 116 del parent REV4C, con **1 ciclo valido** (2 HS validi, 2 TO validi) e caviglia che **plantarflette** (minimo −0,04402 rad, frazione negativa 0,110). È il primo candidato della catena 35D di agosto che chiude un ciclo di cammino *e* plantarflette insieme — la separazione fra le due famiglie, che nei report precedenti risultava netta e disgiunta, per la prima volta si attraversa.

Supera anche il benchmark di luglio R2 (356 step), con l'avvertenza che le sopravvivenze luglio↔agosto **non sono commensurabili** (guardie 15/25 mm contro 20/28 mm, FSM e corridoio diversi).

## 2. Le quattro dichiarazioni obbligatorie, come preregistrate

**(a) Replica di protocollo multivariata, non esperimento a una variabile.** Rispetto a rev4c cambiano insieme quattro dimensioni: init/ancora da S1A a REV4C, cumulatività (due tracce invece di una), `interpolation_steps` 0 → 2, `trace_repeat` 4 → 1.

**(b) Limiti di attribuzione.** Il successo **non attribuisce causalità all'interpolazione**: con quattro cambiamenti simultanei nessun meccanismo singolo è accreditabile, esattamente come i 356 step di luglio R2 restano non attribuibili. Il fallimento avrebbe chiuso la direzione R2-analoga; non è il caso.

**(c) Leakage dello split, atteso e dichiarato in anticipo.** L'interpolazione produce 3 righe distinte ma correlate per riga visitata, sullo stesso segmento e con la stessa label; circa **88 gruppi su 184** hanno righe su entrambi i lati. La **MSE di validazione è solo diagnostica** — il valore ottenuto (0,001475) va letto come ottimisticamente distorto.

**(d) Rischio noto della banda REV4C 69–116.** Deriva L2 misurata rispetto al teacher allo stesso indice: S1A[1–68] media 10,134 (banda già consumata), REV4C[69–116] media **22,267**, 2,2×, con 13 step oltre il massimo stesso della banda di riferimento (30,229). Il report di luglio dichiara che l'interpolazione serviva a insegnare invarianza alle *piccole* deviazioni, quindi quella banda era fuori dall'intento documentato. **Il rischio non si è materializzato**: le metriche per banda (§5) mostrano la banda 69–116 fittata bene (ankle RMSE 0,03243, r 0,98751).

## 3. Preflight bloccante NO-WRITE — GO

| item | esito |
|---|---|
| init/anchor = modulo REV4C, SHA byte-identici, 10 chiavi, clock a zero | **true** — digest `35fef304…` |
| traccia 1: S1A step 1..68 contigui, obs35 esatta, finita | true — SHA `6546befc…` |
| traccia 2: REV4C step 1..116 contigui, obs35 esatta, finita | true — SHA `8f231092…` |
| allineamento temporale `max abs(Δt)` su entrambe | **0.0 / 0.0 s** |
| righe raw visitate | **184** |
| righe interpolate | **368** |
| uniche DAgger / aggregato | **552** / **1052** |
| formula | `500 + (184 × 3) × 1 = 1052` |
| quota DAgger | **552/1052 = 0,5247148288973384** |
| interpolazione verificata riga per riga | **368 figli**: colonne discrete `[11,12,13,17,18,19,20,21]` bit-identiche al genitore, colonne continue esattamente su segmento ad alpha 1/3 e 2/3 |
| duplicati / con label in conflitto | 7 / **0** |
| collisioni col corpus / con label in conflitto | 9 / **0** |
| origine di duplicati e collisioni | righe di reset episodio dove lo stato visitato è bit-identico al teacher, quindi i figli interpolati collassano sullo stesso punto |
| copertura negativa | **14 indici distinti**, finestre `[6,14]` e `[112,116]` (seconda **parziale**: la finestra piena è 112–132) |
| nessun artefatto di luglio operativo | **true** |

Etichetta di labeling: `u_IK` same-step a `teacher_index = step − 1`, condivisa fra riga raw e figli interpolati; SHA delle label `14649d25c3f57e3708c30f719ca3c92e6d9328622395de9c24644f8991dafa01`.

## 4. Semantica dell'interpolazione — provata, non asserita

`target_domain_imitation.py:665-672`: `interpolation_steps = k` aggiunge **k punti**, con `alpha = i/(k+1)`, quindi fattore totale **1+k = 3** per k=2. Le componenti discrete sono forzate a quelle della riga visitata (`:670-671`).

Il self-test **non asserisce** questa equivalenza: la dimostra per confronto differenziale contro la funzione reale di luglio. `july_interpolate` riproduce il pool di `JULY.aggregate_dagger_traces` **bit-identicamente a k = 0, 1, 2, 3**, e gli indici discreti pinnati coincidono con quelli che la regola di luglio deriva dai nomi 35D. Il modulo di luglio è usato **solo** come riferimento di protocollo: nessun dato, checkpoint, traccia o label di luglio entra nella pipeline.

## 5. Fit — iperparametri di luglio invariati

Init e ancora coincidono: `fit_july` clona i parametri che riceve (`anchor = [p.detach().clone() for p in params]`), quindi passare REV4C come init lo rende l'ancora.

| parametro | valore |
|---|---|
| epoche richieste / eseguite / migliore | 400 / **266** (early stop) / **206** |
| batch, lr, val, patience | 64, 3e-4, 0.20, 60 |
| clip / logstd / anchor | 1.0 / 0.1 / 1e-5 |
| seed, grad clipping | 123, nessuno |
| righe train / validazione | 842 / 210 |
| best val MSE | 0,001475237077102065 *(diagnostica, distorta dal leakage dichiarato)* |
| loss epoca 1 → 266 | train 0,016124 → 0,000734; val 0,010353 → 0,001817 |

σ resta **UNRESOLVED**: placeholder 0,004999999670722372, logstd byte-identica all'init, mai interpretata come scelta.

### Gate offline vincolanti — tutti PASS

| gate | misura | esito |
|---|---|---|
| invarianti di integrità | 10 chiavi, clock zero, invarianza bit-identica, logstd byte-identica all'init REV4C, save/reload esatto, nessun critico | **PASS** |
| preservazione di funzione | T1 1,619e-07, T2 1,229e-07 contro 1e-05 | **PASS** |
| convergenza | RMSE aggregato 0,115390 → **0,027599** | **PASS** |

Digest attore risultante `81651dfbd4a32a889779e5518cf824e7a48549ceb06eb2a5ce5401a28bd46abc`.

### Metriche separate per banda

| insieme | knee RMSE / r / ampiezza | ankle RMSE / r / ampiezza |
|---|---|---|
| corpus 500 | 0,035751 / 0,99485 / 1,01422 | 0,028059 / 0,99099 / 0,97830 |
| **S1A righe 1–68** | 0,026168 / 0,79597 / 1,15667 | 0,020369 / 0,98993 / 0,98461 |
| **REV4C righe 1–68** | 0,015817 / 0,91720 / 1,05096 | 0,015097 / 0,99493 / 1,03941 |
| **REV4C righe 69–116** | 0,036062 / 0,99167 / 0,97466 | 0,032425 / 0,98751 / 0,96485 |
| 97 righe negative del corpus | 0,039729 / 0,99614 / 1,00162 | 0,031475 / 0,92469 / 1,06314 |
| pool interpolato 552 | 0,024477 / 0,99121 / 0,98894 | 0,020815 / 0,99298 / 0,99354 |

Segno finestre negative: comando positivo 0,24742 → 0,21649; comando minimo caviglia −0,15980308987736117.
`max_abs` vs attore sorgente [1,46618 / 1,18048] **INFORMATIVO**; simmetria controlaterale [0,09207 / 0,13274] **DIAGNOSTICA**.

## 6. Rollout — uno solo, nominale deterministico v3, nessun retry

`returncode` 0, durata 340,896 s, comando canonico v3 (offset 1.956870983805102, `deterministic`, seed 123).

| criterio | valore | esito |
|---|---|---|
| **gate primario** — sopravvivenza | **372 step**, soglia strettamente > 116 | **PASS** |
| **marcatore secondario** — `valid_cycle_count` | **1**, regola ≥ 1 | **PASS** |

Riferimenti: REV4C parent 116 · rev4d 44 · rev4b 42 · S1A 392 · benchmark luglio r1 45, r2 356, r3 221.

### I sette gate

| gate | valore | esito |
|---|---|---|
| completamento 500 | 372 step, `end_reason = grf_penetration` | **FAIL** |
| phase_timeout stance / swing | **0 / 0** | PASS |
| morphology causal contract failure | **0 righe positive** su 372, max 0.0 | PASS |
| hs_cancelled | **0** | PASS |
| resync | **0** | PASS |
| ≥ 1 ciclo valido | **1** | **PASS** |
| penetrazione | 0,02822357723279283 m vs limite 0,020 | **FAIL** |

Falliscono solo completamento e penetrazione. La penetrazione supera anche la guardia dura v3 di 0,028 m, ma di **0,22 mm**.

### Contatori ed eventi

`valid_hs_count` 2 · `valid_to_count` 2 · `valid_cycle_count` **1** · `invalid_event_count` 0 · `resync_count` 0 · `hs_cancelled_count` 0 · stato finale `STANCE_AFTER_HS`.

### Cinematica, forma e segno

- **knee q** ∈ [−0,46825, −0,17148], media −0,29231, frazione negativa 1,000, **0 step fuori bounds** `[-1.5, 0.0]`.
- **ankle q** ∈ [**−0,04402**, +0,41471], media +0,16194, **frazione negativa 0,11022**, **0 step fuori bounds** `[-0.7, 0.7]`.
- Penetrazione max 0,028224 m / media 0,011866 m; riserve max 654,78 N·m / media 198,79 N·m.
- Clipping azione **0 step**, `rows_raw_neq_applied` 0, saturazione |a|>1 nulla su entrambi i giunti.
- Ritorno di episodio −3,639322.
- `sign_agreement` caviglia contro target IK protesico **0,74194** (valido); knee resta `VOID_degenerate_reference`.
- Finestre negative in closed loop: 65 righe, frazione di comando caviglia positivo 0,81538, minimo realizzato −0,038704.
- **B3 non valutabile**: il campo di fase è identicamente zero su tutte le 372 righe, come su ogni traccia v3.

## 7. Posizione nella catena

| candidato | init | quota DAgger | step | cicli | ankle min | frazione neg. | penetrazione |
|---|---|---|---|---|---|---|---|
| rev4b | S1A | 75,82% | 42 | 0 | +0,02397 | 0,000 | 0,02821 |
| rev4d | S1A | 43,95% | 44 | 0 | +0,02345 | 0,000 | 0,02910 |
| rev4c | S1A | 35,23% | 116 | 0 | −0,06993 | 0,155 | 0,02866 |
| **REV4E** | **REV4C** | **52,47%** | **372** | **1** | **−0,04402** | **0,110** | **0,02822** |
| S1A (init originario) | — | — | 392 | 0 | −0,42306 | 0,640 | 0,02466 |

## 8. Stato

- **Candidato NON promosso** e NON-DEPLOYABLE. La promozione richiede 500/500, zero contatori critici, ≥ 2 cicli, penetrazione entro gate e gate cinematici: due mancano. Nessuna promozione automatica in ogni caso.
- Nessun PPO, test su sigma, round successivo, altro candidato o nuova collection.
- Nessuna modifica a production/env/reward/FSM/morfologia/SEA/C++.
- Artefatti rev4b, rev4c e rev4d non mutati; nessun artefatto di luglio usato operativamente.

## 9. File e SHA-256

| file | SHA-256 |
|---|---|
| `v26b_amendment_rev4e_r2_replay.json` | `6caf9b8b547d53ac6ce856de9339ab616a0d34191a7b33b1b4a82c531d38eb3f` |
| `v26b_amendment_rev4d_repeat.json` (parent) | `34f8a08ee3fbdb1a23ec5d87cfde6fb07562fef0733e32debdc49532fedc2f32` |
| `v26b_addendum_rev4c_a_corrections.json` | `377fd4e0ff68fc9135f769d8ce2f23935ec9977e185feaa92624360e1942d9ad` |

**Candidato** `candidates/REV4E_R2REPLAY_35D_NONDEPLOYABLE/`

| file | SHA-256 |
|---|---|
| `v26b_rev4e_receipt.json` | `80281756ff1a591027df33a64c6885856362bf67665a5fdb176b67145f4d67ce` |
| `rl_module/module_state.pkl` | `680f47b1802cbfd61eb0dcd974877a372e469367caadb4b50e9a51145094c651` |
| `rl_module/actor_feature_manifest.json` | `8447eab00bc725f6949051580c958da64d472db693ce46ef530fabb39f3402d4` |
| `rl_module/metadata.json` | `3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12` |
| `rl_module/class_and_ctor_args.pkl` | `c9a6722ff95642795bfe1146d0087a68b5861fd508cbe3692195b2d820d810a7` |

**Input pinnati** — init/anchor `module_state.pkl` `dc6a7dab9fe202776501f3010b17ac37c45e7d087da3de8df0531bdd9f1c202e` (digest `35fef3042211d9fbf6abcc6277c5f82269c44e98b633da4f825dab3e3bb6b0e9`); traccia 1 `6546befcd4a2e26711137a807cd43a797e47abd62500243fe6c015d7abcfcf21`; traccia 2 `8f2310928b586bed5750122478bd30cd08beba34e47e318ee95b0c2044474f63`.

**Rollout** `rollouts/rev4e_nominal_det/REV4E_35D__v3_canonical__nominal__det/`

| file | SHA-256 |
|---|---|
| `v26b_rev4e_rollout_receipt.json` | `fb58cffcd854b69002c8189e6101f083a54efde6cd312cdd4b8b5785cb3b7064` |
| `rollout_policy_trace.json` | `aa91a2e27e42ba87461e293b3cc6115cd1c957b9e1b7966259e99c8fb50e5811` |
| `rollout_summary.json` | `49bc8d6e7faf60404452e9b53cda4584ea9ac459721d67f1b97bc5c4fec1388d` |
| `rollout_reset_diagnostics.json` | `3d3b8dfa9164b2e63f0ba6ca795f2abee41ef6c8bd8929e19368ad83a325ed76` |
| `watchdog_summary.json` | `022ec4eba24468d8369bce835dc82776f66d2bf41ebfbdd0620ff97bf64656ed` |
| `watchdog_state.json` | `1e8d9a1069eb647ad26903b69f41578f7b24bb430213871f87fed61e58a4de9f` |

Log di stadio `rev4e_stage_20260824_231327.log` — `5e76b5cf7ec6ca75eb9ae2fbf8761801e8816550e8dbcb99b2006592968a50d1`.

## 10. Test e verifiche

- Self-test pre-esecuzione **75/75 PASS**; post-esecuzione **88/88 PASS**, con `offline_pass: true`, `survival_steps: 372`, `primary_gate: "PASS"`, `valid_cycles: 1`, `secondary_marker: "PASS"`.
- **Prova differenziale dell'interpolazione** contro `target_domain_imitation.aggregate_dagger_traces` a k = 0,1,2,3: pool bit-identico, e conferma dal summary della funzione stessa che il fattore è 1+k.
- Guardia anti-luglio: tre percorsi di artefatti di luglio (modulo r2, traccia r2, teacher dataset v2) vengono **rifiutati**; i percorsi del lineage corrente passano.
- Fail-closed: `TRACE2_ROWS=115`, `RAW_VISITED=183`, `INTERPOLATION_STEPS=1` e copertura negativa attesa errata fanno fallire il preflight; pin manomessi rifiutati; token errati rifiutati; seconda esecuzione rifiutata con `FileExistsError`.
- Igiene chiavi e prosa: la stringa non riparametrizzata del tool congelato S1B viene rifiutata; il receipt riporta `closed-loop eligibility of REV4E`; il tool congelato resta intatto su disco.
- Nessun macchinario PPO o sigma (verifica su identificatori reali); esattamente una invocazione di harness.

## 11. Correzioni

Nessuna correzione a dati o affermazioni precedenti. Una **precisazione** su una lettura che il presente risultato rende necessaria: nel report rev4d avevo osservato che le righe derivate di S1A erano dannose «a qualunque peso». L'osservazione resta valida **a interpolazione zero**, che è la condizione in cui fu misurata; REV4E mostra che con interpolazione 2 una banda di deriva confrontabile (REV4C 69–116, media 22,267 contro 19,778 di S1A 69–116) è invece assorbita senza danno. Le due affermazioni non sono in conflitto: la seconda delimita il dominio di validità della prima.

## 12. TODO propagati

- **TODO-2** — σ = 0.005 placeholder non risolto. *(aperto, ereditato)*
- **TODO-3** — B3 indeterminato: campo di fase identicamente zero su ogni traccia v3, incluse le 372 righe di REV4E. *(aperto, ereditato)*
- **TODO-4** — Conflitto cammino/plantarflessione: **parzialmente superato**. REV4E è il primo candidato che chiude un ciclo valido plantarflettendo. Resta aperto per il traguardo pieno 500/500 con ≥ 2 cicli. *(aggiornato)*
- **TODO-5** — Tensione aritmetica interpolazione↔quota a copertura piena (≥ 61,06%). *(aperto)*
- **TODO-6** — La lunghezza del prefisso è variabile operativa dannosa **a interpolazione zero**; il dominio va riesaminato alla luce di REV4E. *(aggiornato)*
- **TODO-7** — Nuovo: penetrazione 0,028224 m, oltre la guardia dura v3 di 0,028 per 0,22 mm, è ora il vincolo che termina l'episodio a 372 step. È il candidato più prossimo al completamento e il collo di bottiglia si è spostato dalla FSM al contatto. *(nuovo)*
