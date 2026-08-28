# V26B — Audit read-only su rev4b: semantica del taglio July, origine delle 392 righe, e correzioni

**Ambito:** solo audit read-only su artefatti autorevoli di luglio e sul codice corrente. **Nessun fit, rollout, retry o promozione.** rev4b resta quarantinato. Nessun artefatto immutabile modificato: le correzioni sono in un addendum separato content-addressed.

## 1. Semantica July del taglio — file, righe, ruoli

**Definizione**: `truncate_before_discrete_mismatch(nominal_rows, disturbed_rows, feature_names)` in **`Trajectory Generator/baseline_MLP/target_domain_noise_adaptation.py:75-101`** (sha `1665af4ded3c55d48d2ced8a62cd3bd40705f0e6b9aa294195374ea592b6ae1b`). Docstring: «*Keep only time-aligned rows whose deployable event/FSM state still agrees*». Confronta indice per indice le componenti discrete di `actor_observation_vector_before` fra le due tracce e restituisce `disturbed_rows[:limit]`, dove `limit` è l'indice della prima disuguaglianza; riporta `original_steps`, `retained_steps`, `first_discrete_mismatch_step`.

**Ruoli dei due argomenti, dai call-site**:
- **`target_domain_noise_adaptation.py:305-333`** — `nominal_rows = _load_json(nominal_trace_path)` e i `disturbed_traces` sono i rollout **perturbati dello stesso attore**; il taglio è applicato a ciascuna traccia disturbata contro la nominale.
- **`target_domain_markov_adaptation.py:325-352`** — identico con i `recovery_rows`: nominale dello stesso attore contro tracce di recovery.

**Due proprietà decisive**:
1. Il taglio è **opt-in**, dietro il flag `--stop-before-discrete-mismatch` (`if args.stop_before_discrete_mismatch:` in entrambi i call-site). Non è incondizionato.
2. Il taglio **non compare mai** nel percorso DAgger dell'11/07: `grep -rn truncate_before_discrete_mismatch *.py` restituisce **solo** la definizione e i due call-site sopra. `target_domain_dagger.py` (sha `6084d10f…`) non lo chiama, e `aggregate_dagger_traces` in `target_domain_imitation.py` concatena `teacher` + `tile(visited, repeat)` **senza alcun filtro**.

**Conferma dall'artefatto autorevole**: `runs/training/target_domain_dagger_2026-07-11_r1/dagger_dataset_report.json` registra `unique_dagger_samples: 68` con `steps: 68` per l'unica traccia — cioè **l'intero rollout del clone BC, che era finito proprio a 68**. Nessuna riga scartata.

**Semantica, in una frase**: a luglio il taglio serve ad allineare **tracce perturbate/di recovery** alla **nominale dello stesso attore** nelle fasi di raccolta di quei dataset (noise adaptation e markov 13/07), ed è opzionale; **non è parte del DAgger dell'11/07**, che aggrega il prefisso on-policy per intero.

## 2. Perché rev4b ha usato 392 righe

Perché il percorso replicato — il DAgger 11/07 — **non prevede alcun taglio**, come stabilito al §1. rev4b ha applicato l'operatore July verbatim: `concat(corpus 500, tile(prefisso 392, 4))` = 2068 righe, senza troncamento e senza dedup, con etichette time-aligned `teacher_index = step − 1`. Su questo punto **non c'è difetto** nella costruzione del dataset.

## 3. Il 13 non è autorevole, e non riguarda il prefisso S1A

I due numeri citati vivono nel **rollout receipt di rev4b** e descrivono **il rollout da 42 step di rev4b stesso**, non il prefisso usato per il fit:

| grandezza | rev4b (proprio rollout) | S1A (prefisso usato nel fit) |
|---|---|---|
| `compared_steps` | 42 | **392** |
| `first_mismatch_step` | 14 | **72** |
| `label_valid_prefix_rows` | **13** | **71** |

Quindi il 13 non è mai stato un input del dataset rev4b, e non lo sarebbe potuto essere: si riferisce a un attore diverso e a una traccia diversa.

**Sul riferimento**: entrambe le misure usano un **proxy**, e il receipt lo dichiara esso stesso: «*pinned det nominal V26 anchor trace (grid-aligned prescribed-cycle proxy; **July's reference was the source actor's OWN nominal det trace**, unavailable for R0a's first-ever rollout — declared in rev3c, audit-gated)*». Hai ragione: il riferimento July corretto è la nominale **dello stesso attore**, non la V26 anchor.

**Quale sarebbe il riferimento corretto, e può essere ricostruito senza nuovi rollout?** Qui emerge il punto sostanziale: nella semantica July il taglio confronta una traccia **perturbata** con la **nominale** dello stesso attore. Il prefisso S1A **è** la nominale deterministica di S1A: confrontarlo con sé stesso dà zero disuguaglianze e ritiene tutte e 392 le righe. **L'operazione è quindi degenere per questo oggetto** — non perché manchi un riferimento, ma perché il prefisso DAgger non è una traccia perturbata. Ciò che *non* esiste senza generare un nuovo rollout è una traccia **perturbata** di S1A, che sarebbe l'argomento `disturbed_rows`. Conclusione: nessuna ricostruzione è necessaria per il caso corretto, e nessuna è possibile per il caso perturbato senza nuova raccolta.

## 4. Composizione quantitativa: fedeltà dell'operatore contro fedeltà del bilanciamento

| ipotesi di prefisso | righe | dataset | quota on-policy |
|---|---|---|---|
| **rev4b eseguito** (nessun taglio, operatore July) | 392 | 500 + 392×4 = **2068** | **75,82 %** |
| proxy V26 applicato a S1A | 71 | 500 + 71×4 = 784 | 36,22 % |
| il numero citato (rev4b su sé stesso) | 13 | 500 + 13×4 = 552 | 9,42 % |
| **luglio r1** | 68 | 500 + 68×4 = **772** | **35,23 %** |

Le due fedeltà sono in tensione e **non possono valere insieme**:
- **Fedeltà dell'operatore** = aggregare il prefisso on-policy *intero* senza tagli. È ciò che luglio fece e ciò che rev4b ha fatto. Con un prefisso di 392 righe produce una quota del 75,8 %.
- **Fedeltà del bilanciamento** = riprodurre il 35,2 % di quota on-policy. Richiede un prefisso di circa 68 righe, cioè **scartare l'82 % del prefisso disponibile** — un'operazione che luglio non ha mai eseguito.

La radice è che a luglio le due coincidevano per un accidente del dato: il clone BC **moriva** a 68 step, quindi «tutto il prefisso» e «68 righe» erano la stessa cosa. S1A sopravvive a 392, e l'accidente non si ripete. Nota per completezza: la coincidenza fra il proxy V26 su S1A (71 righe, 36,2 %) e il bilanciamento di luglio (35,2 %) è numericamente notevole ma **non ha fondamento semantico** — quel proxy non è il riferimento July per le tracce DAgger.

**Opzioni per ricreare davvero la condizione July, valutate e non eseguite:**
- **(a) Prefisso troncato a 68 righe, tile ×4** → 772 righe, 35,23 %: riproduce esattamente la *composizione* di luglio. Costo: è un taglio arbitrario per conteggio, non per semantica, e scarta 324 righe on-policy valide. Fedele al bilanciamento, infedele all'operatore.
- **(b) `trace_repeat = 1` sul prefisso intero** → 892 righe, 43,94 %: mantiene tutto il prefisso e avvicina la quota, ma cambia il parametro `trace_repeat` che luglio fissava a 4. Infedele a entrambi, a metà strada.
- **(c) Accettare la divergenza** e trattare rev4b come il risultato dell'operatore July su un prefisso più lungo: è ciò che è già stato eseguito, ed è l'unica opzione senza scelte arbitrarie.
- **(d) Ottenere un prefisso corto per costruzione**: sarebbe necessario un attore che muore presto — cioè nuova raccolta, non autorizzata e comunque circolare.

## 5. Etichette errate negli artefatti rev4b — confermate

Il blocco `diagnostics.closed_loop_comparison` del rollout receipt rev4b è stato prodotto chiamando la funzione del modulo L20 (`v26b_l20_rollout.closed_loop_comparison`), quindi porta **chiavi con il nome L20 e valori di rev4b**:

| chiave presente | va letta come |
|---|---|
| `kinematics_L20` | `kinematics_REV4B` |
| `actions_on_visited_states.L20_vs_S0D_same_states` | `REV4B_vs_S0D_same_states` |
| `actions_on_visited_states.L20_vs_uIK_same_times` | `REV4B_vs_uIK_same_times` |
| `actions_on_visited_states.S0D_vs_uIK_on_L20_states` | `S0D_vs_uIK_on_REV4B_states` |
| `scope`: «states L20 actually visited» | «states REV4B actually visited» |

**Non è un errore di dati**: `kinematics_L20.ankle_q` vale min +0,023969903588294983, **identico** a `analysis.trajectory_quality.ankle_q` di rev4b. Solo i nomi sono sbagliati.

Un blocco invece contiene davvero dati di **altri attori**: `offline_reference_block` riporta `source: "L20 fit receipt"` con `action_drift_vs_S0D [0,0649, 0,0815]` (L20), `A2_action_drift_vs_S0D [0,0657, 0,0455]` e `S1A_action_drift_vs_S0D [0,384, 0,289]`. Sono costanti di contesto ereditate dal modulo L20 e **non vanno lette come misure di rev4b**; i valori propri di rev4b sono `max_abs vs S0D [1,45923, 1,18137]`, media `[0,37114, 0,30271]`, dal suo fit receipt.

Causa: riuso di una funzione diagnostica per-attore con nomi di chiave cablati. Regola in avanti registrata nell'addendum: parametrizzare l'etichetta dell'attore o rinominare le chiavi alla scrittura.

## 6. Correzione esplicita dell'affermazione «July-faithful»

**Non è dimostrabile come affermazione secca, e la ritiro in quella forma.** La formulazione corretta e verificabile è: **fedele all'operatore DAgger dell'11/07, divergente nel bilanciamento.**

Dimostrabilmente fedele: iperparametri verificati dai sorgenti (400 epoche, batch 64, lr 3e-4, val 0,20, patience 60, clip 1,0, logstd 0,1, anchor 1e-5, seed 123, `trace_repeat` 4, nessun gradient clipping), composizione della loss verbatim, operatore di aggregazione verbatim senza taglio né dedup, etichette time-aligned, logstd ripristinata e clock azzerato dopo ogni step.

Non fedele: quota on-policy **75,82 % contro 35,23 %**; holdout contiguo assorbito; scaling fisico July nella parametrizzazione (queste ultime due erano già dichiarate in rev4b).

Riclassificazione accettata: **rev4b è una diagnostica**, con protocollo fedele nell'operatore e non nel bilanciamento, e resta quarantinato.

## 7. Una sola mossa primaria, e un fallback

**Primaria — `rev4c`: ripetere il round variando SOLO il bilanciamento.** Dataset `500 + primi 68 × 4 = 772` righe, quota on-policy 35,23 %, cioè la composizione esatta di luglio r1. Tutto il resto byte-identico a rev4b: stesso init S1A nei tre ruoli, stessi iperparametri verificati, stessa loss, stesso operatore di aggregazione, stessi gate offline e closed-loop, stesso gate qualitativo caviglia, un solo candidato, nessun retry, nessuna σ.
*Perché è la mossa giusta*: isola **una sola variabile** — il bilanciamento — fra rev4b (75,8 %, collasso a 42 step) e luglio (35,2 %, 45 step al round 1 e 356 al round 2). È l'unico esperimento che può attribuire o escludere il collasso alla sovra-pesatura on-policy.
*Va dichiarato senza ambiguità*: il taglio a 68 è per **conteggio**, non per semantica; luglio non tagliava. rev4c sarebbe quindi fedele al bilanciamento e **infedele all'operatore** — l'immagine speculare di rev4b. Nessuna delle due può essere chiamata «July-faithful» senza qualificazione.
*GO*: preflight che confermi contiguità, allineamento temporale esatto e assenza di conflitti di label sulle prime 68 righe, più i gate di integrità invariati. *NO-GO*: qualunque item di preflight fallisce, oppure l'architetto giudica che un taglio per conteggio non sia accettabile.
*Falsificabilità*: se anche rev4c collassa sotto ~100 step, l'ipotesi «il collasso è causato dalla sovra-pesatura on-policy» è **confutata** e la direzione DAgger a round singolo va chiusa; se invece si avvicina ai 45–356 step di luglio, l'ipotesi è sostenuta e la questione diventa quanti round servono.

**Fallback (solo se rifiuti la primaria) — nessun fit**: analisi read-only di sensibilità sul bilanciamento, calcolando dagli artefatti già materializzati come varia la fedeltà offline all'IK al variare della lunghezza del prefisso, per delimitare il trade-off senza spendere un fit. *GO*: nessuna esecuzione richiesta oltre la lettura. *NO-GO*: non applicabile.

Non raccomando ulteriori round DAgger oltre questo: luglio stesso, con il bilanciamento corretto, ottenne 45 step al round 1, 356 al round 2 e una regressione a 221 al round 3, senza mai completare l'episodio; e il numero di round non è una variabile che un singolo esperimento possa risolvere.

## 8. Artefatti
| Artefatto | SHA-256 |
|---|---|
| questo audit | *(vedi riga finale)* |
| `…/v26b_bridge_2026-08-24/v26b_addendum_rev4b_a_corrections.json` | `74f7c50a83877a2005708e95ee911992f301bbe3799f41c823f807dbbb847ce1` |
| rev4b amendment (**invariato**) | `883ffc9a0217fa6fefcabf820b382717b8ad2e8f7ccd5814461b0bb4218f0f93` |
| rev4b fit receipt (**invariato**) | `7f7f10419c84b1a1d03d91082b579551c662ed4d8b579e1a26c79798328f2565` |
| rev4b rollout receipt (**invariato**) | `2032c5a9d5ee448cb45cebf65d13b40efeae062a58c8260fb7f4db413978c1b8` |
| rev4b report (**invariato**) | `fd55f6454052b5dbc770692dd05df366881702cfb805a4944d49a42b8ec2a4ab` |
| `baseline_MLP/target_domain_noise_adaptation.py` (letto) | `1665af4ded3c55d48d2ced8a62cd3bd40705f0e6b9aa294195374ea592b6ae1b` |
| `baseline_MLP/target_domain_dagger.py` (letto) | `6084d10fd518fc1503b2aaa8ebad4f119518734e074ecca08656037d7a64242a` |
| `baseline_MLP/target_domain_imitation.py` (letto) | `442be2222c935e18607e944d946d23bc8151b32c5e57d64036b8e5a00c1e9ed3` |

## 9. Stato
**STOP per il tuo audit.** Nessun fit, rollout, retry o promozione. rev4b resta quarantinato e riclassificato come diagnostica con protocollo fedele nell'operatore e divergente nel bilanciamento. L20 resta eligible-non-promosso, S0D resta l'unico attore con evidenza closed-loop piena, σ irrisolta, B3 INDETERMINATE, production invariata.
