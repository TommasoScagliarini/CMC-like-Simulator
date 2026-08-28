# V26B-REV4C-BALANCE — esperimento a una variabile sulla quota on-policy

**Data**: 2026-08-24
**Token autorizzato**: `V26B-REV4C-BALANCE`
**Emendamento**: `v26b_amendment_rev4c_balance.json` — SHA-256 `b25b0fe4f89a9ea4a7ac6b6b9ca8b5d2d393e4535a2a0944edd5386dd62790bb`
**Esito**: fit e gate offline **PASS**; rollout eseguito; **test causale primario SUPPORTED (116 step > 100)**; candidato **NON promosso**, 3 dei 7 gate chiusi falliti.

---

## 1. Problema

Il round rev4b — fedele all'*operatore* DAgger di luglio ma divergente nel *bilanciamento* (quota on-policy 75,82% contro il 35,23% del round r1 di luglio) — ha prodotto un fit offline quasi perfetto e un collasso in closed loop a **42 step**. L'ipotesi da testare era che il collasso fosse causato dal sovrappeso della componente on-policy, non dal DAgger in sé.

L'architetto ha autorizzato **un solo esperimento a una variabile**: ripristinare esattamente la quota di luglio lasciando tutto il resto byte-identico a rev4b.

## 2. Inquadramento vincolante (accolto dall'emendamento)

Il cap a 68 righe è registrato come **budget di collection DAgger controllato**, e l'emendamento nega esplicitamente le due letture che l'architetto ha rifiutato:

- **NON** è un "prefisso causalmente valido": l'audit ha stabilito che l'operatore di troncamento di luglio confronta una traccia *perturbata* con la nominale *dello stesso attore* ed è inapplicabile qui, e che le cifre 13/14 del receipt rev4b descrivono il rollout di 42 step di rev4b stesso, non il prefisso S1A.
- **NON** è una replica letterale dell'incidente di luglio: le 68 righe di luglio erano l'*intera* traccia di un clone morto allo step 68; qui 68 è un taglio deliberato di una traccia da 392 righe.

Enunciato di fedeltà registrato: **rev4c è fedele al BILANCIAMENTO di luglio e, per costruzione, NON fedele all'OPERATORE di luglio** (che non troncava mai un prefisso DAgger). È l'immagine speculare di rev4b, operator-faithful e balance-divergent. Nessuno dei due può essere chiamato "July-faithful" senza questa qualificazione.

## 3. Incidente di esecuzione — dichiarato

La **prima** esecuzione dello stadio si è interrotta **dopo il fit e prima della materializzazione**, con `Rev4cError` su `receipt.lineage.rev3z_aggregate_states_preserved`.

- **Causa**: falso positivo del mio stesso scanner di igiene. Le chiavi `L05/L10/L20` in quel blocco sono nomi legittimi di *provenienza* di altri candidati, non metriche rev4c mal etichettate.
- **Stato dopo l'abort**: nulla materializzato, staging rimosso, nessun lock residuo, rollout mai avviato (verificato su disco).
- **Correzione**: il contratto è ora enunciato correttamente — vietata l'etichetta estranea in una chiave che nomina **dati propri**; i sottoalberi di provenienza dichiarati (`lineage`, `parents_immutable`, `preflight`) sono esenti dalla scansione delle chiavi. Il divieto di `offline_reference_block` resta **globale**, perché quel campo incorpora le *metriche* di un altro attore, che è il difetto registrato dall'addendum rev4b.
- **Riesecuzione**: lo stadio è stato rieseguito da zero. Il fit è deterministico e la riesecuzione lo dimostra: le 9 righe di loss per epoca dei due log coincidono **byte per byte** (`diff` vuoto). Non è il retry di un esperimento il cui esito sia stato rimescolato, e nessun artefatto del tentativo abortito è sopravvissuto.
- Il record è inciso nel receipt sotto `execution_incident_disclosed`.

Log conservati entrambi (no-clobber):
| log | SHA-256 |
|---|---|
| `rev4c_stage_20260824_222513.log` (abortito) | `aadccdf9fd92fe61217c00a778c197f60d1cfea7549e8b7f878eb3a3d85f3c48` |
| `rev4c_stage_20260824_222945.log` (eseguito) | `24d08408f5baaed58b3fbb1812292f38cab8169fca0b914ac6920bf78040285a` |

## 4. Preflight bloccante (no-write) — GO

Tutti gli item richiesti sono passati sulle prime 68 righe contigue del rollout S1A congelato:

| item | esito |
|---|---|
| step 1..68 contigui | true |
| obs35 esatta rispetto ai vettori registrati, larghezza 35, finita | true |
| allineamento temporale `max abs(t_S1A − t_corpus)` | **0.0 s** |
| label finite, same-step `teacher_index = step − 1` | true, SHA `4779342c4d065b63d737234d8f28328ddfef03b6c26f9b70644d9607e4d21364` |
| duplicati esatti nel prefisso | 0 |
| collisioni bitwise col corpus / con label in conflitto | 2 / **0** |
| composizione | 500 + 68×4 = **772**, quota on-policy **0.35233160621761656** (luglio r1: 0.3523316062176166) |

**Conseguenza da riportare**: il cap conserva solo **9** righe on-policy a caviglia negativa (finestra [6,14]) contro le 65 del prefisso completo da 392 righe. La riduzione del sovrappeso costa quasi tutta la copertura on-policy della plantarflessione — è il prezzo strutturale dell'esperimento, non un difetto.

## 5. Fit — protocollo di luglio invariato

Nessun iperparametro di luglio è ridefinito da rev4c: il modulo **riusa** `v26b_rev4b_dagger.fit_july` e le costanti `J_*`, e il self-test lo dimostra. Unica variabile cambiata: il budget di collection.

| parametro | valore |
|---|---|
| epoche richieste / eseguite / migliore | 400 / 400 / **345** |
| batch, lr, val fraction, patience | 64, 3e-4, 0.20, 60 |
| clip / logstd / anchor weight | 1.0 / 0.1 / **1e-5** |
| seed, trace_repeat, grad clipping | 123, 4, nessuno |
| righe train / validazione | 618 / 154 |
| best val MSE | 0.0019297704566270113 |
| loss epoca 1 → 400 | train 0.023520 → 0.000582; val 0.011442 → 0.002140 |

**Init, anchor e collection policy**: esclusivamente l'attore S1A `8f3e0ce17eff7c741dcf72de6d0fec0c372f9dbc7a9b3119d41c155ec8603e35`. Nessuna nuova collection. **Sigma resta UNRESOLVED e non è mai stata scelta** (σ_effettiva 0.004999999670722372, logstd byte-identica all'init, entropia entro tolleranza).

### Gate offline vincolanti — tutti PASS

| gate | misura | esito |
|---|---|---|
| invarianti di integrità | 10 chiavi, clock a zero, invarianza clock bit-identica, logstd byte-identica a S1A, save/reload esatto, nessun critico | **PASS** |
| preservazione di funzione | T1 3.97e-08, T2 1.09e-07 contro tolleranza 1e-05 | **PASS** |
| convergenza del fit | RMSE aggregato 0.09622456 → **0.03266974** | **PASS** |

`failed: []`, `all_binding_pass: true`.

### Misure riportate senza soglie inventate

| insieme | knee RMSE / r / ampiezza | ankle RMSE / r / ampiezza |
|---|---|---|
| corpus 500 | 0.038737 / 0.99376 / 0.99995 | 0.029573 / 0.99023 / 0.98156 |
| prefisso capped 68 | 0.015908 / 0.92935 / 1.12817 | 0.010651 / 0.99780 / 0.99546 |
| **tutte le 97 righe negative del corpus** | 0.043945 / 0.99514 / 0.98110 | 0.035796 / 0.90835 / 1.09409 |

Segno sulle finestre negative (97 righe): frazione di comando positivo 0.26804 prima → 0.24742 dopo; comando minimo di caviglia −0.17144738967180218.
`max_abs` rispetto all'attore sorgente [1.46598, 1.21188] — **INFORMATIVO**; simmetria controlaterale sana [0.09457, 0.13150] — **DIAGNOSTICA**.

## 6. Rollout closed loop — uno solo, nominale deterministico, senza retry

Comando canonico v3 (`rollout_eval.py`, `--no-auto-config`, offset 1.956870983805102, `deterministic`, seed 123, `--record-outputs --record-policy-trace` + argomenti di timeout F1). `returncode` 0, durata 109,065 s.

### Test causale primario (preregistrato)

| misura | valore |
|---|---|
| sopravvivenza osservata | **116 step** |
| soglia congelata | > 100 |
| **verdetto** | **SUPPORTED** |

Punti di riferimento: rev4b @ 75,82% = 42 · luglio r1 @ 35,23% = 45 · luglio r2 = 356 · S1A init = 392.

**Lettura onesta del margine**: il superamento è di 16 step su una soglia di 100, da un singolo rollout deterministico senza retry, quindi senza stima di varianza. La soglia era però congelata prima dell'esecuzione e il verdetto segue la regola preregistrata. Il sostegno più forte all'ipotesi non è il superamento in sé ma il fatto che rev4c migliori rev4b **su entrambi gli assi contemporaneamente**: sopravvivenza 42 → 116 e caviglia che torna a plantarflettere (frazione negativa 0,0 → 0,155; minimo +0,0240 → **−0,0699 rad**). Ridurre il sovrappeso on-policy non ha barattato un asse contro l'altro.

### I sette gate di eleggibilità

| gate | valore | esito |
|---|---|---|
| completamento 500 / episode_time_limit | 116 step, `end_reason = grf_penetration` | **FAIL** |
| phase_timeout stance / swing | 0 / 0 | PASS |
| morphology causal contract failure (per riga) | 0 righe positive, max 0.0 | PASS |
| hs_cancelled | 0 | PASS |
| resync | 0 | PASS |
| almeno un ciclo valido | 0 | **FAIL** |
| penetrazione | 0.028665339192614346 m contro limite 0.020 | **FAIL** |

`failed: [completion_500_time_limit, valid_cycle_at_least_one, penetration]`.

### Cinematica e diagnostica

- knee q ∈ [−0.22848, −0.14163], media −0.19249, frazione negativa 1.0, 0 step fuori bounds.
- ankle q ∈ [**−0.06993**, 0.44982], media 0.20555, frazione negativa 0.15517, 0 step fuori bounds.
- Penetrazione max 0.028665 m / media 0.014792 m; riserve max 668,26 N·m / media 264,06 N·m.
- Clipping dell'azione: **0 step**; `rows_raw_neq_applied` 0; saturazione |a|>1 nulla su entrambi i giunti.
- Ritorno di episodio −13.940055417279044.
- B3 (finestra di fase 0.55–0.80): **non valutabile**, il campo di fase è identicamente zero su tutte le 116 righe — stessa indeterminatezza già registrata per le tracce v3.
- `sign_agreement` caviglia contro target IK protesico 0.77586 (valido); il knee resta **VOID_degenerate_reference**, come da convenzione già fissata.
- Primo mismatch discreto FSM allo step 14 su 116 confrontati (prefisso di label valide 13), contro il proxy àncora V26 pinnato.

## 7. Posizione di rev4c nella catena

| candidato | step | end_reason | ankle min | frazione negativa | cicli | penetrazione |
|---|---|---|---|---|---|---|
| REV4B | 42 | grf_penetration | +0.02397 | 0.000 | 0 | 0.02821 |
| **REV4C** | **116** | grf_penetration | **−0.06993** | **0.155** | 0 | 0.02866 |
| R2I | 197 | grf_penetration | −0.18176 | 0.305 | 0 | 0.02885 |
| DAGGER_R1 | 242 | grf_penetration | −0.37056 | 0.570 | 0 | 0.02941 |
| S1A (init) | 392 | phase_timeout:swing | −0.42306 | 0.640 | 0 | 0.02466 |
| R0A | 493 | joint_divergence | +0.02856 | 0.000 | 2 | 0.02734 |
| S1B A2 | 500 | episode_time_limit | +0.03056 | 0.000 | 2 | 0.01384 |
| S0D | 500 | episode_time_limit | +0.01528 | 0.000 | 2 | 0.01374 |
| S1C2Z L20 | 500 | episode_time_limit | +0.00615 | 0.000 | 2 | 0.01668 |

Il quadro resta diviso in due famiglie disgiunte: **ogni** attore che completa 500/500 con 2 cicli validi non plantarflette mai (frazione negativa esattamente 0), e ogni attore che plantarflette muore per penetrazione. rev4c non risolve questa separazione: la attraversa parzialmente, collocandosi tra rev4b e R2I. L'esperimento risponde alla domanda posta — il bilanciamento conta — senza rispondere alla domanda aperta, cioè se esista un attore che cammini *e* plantarfletta.

## 8. Difetto residuo dichiarato (non corretto per divieto di mutazione)

L'obbligo di parametrizzare le etichette d'attore è stato implementato dove ho scritto io il codice: il blocco `diagnostics.actor_comparison` porta `actor_label: "REV4C"` e chiavi neutre (`kinematics`, `vs_prosthetic_IK_target`, `actions_vs_source_actor`, `actions_vs_uIK_same_times`, `negative_window_sign`), e nessun `offline_reference_block` estraneo è presente. I test di regressione lo dimostrano, incluso il fatto che **il vecchio receipt in stile rev4b verrebbe rifiutato dal nuovo contratto**.

Resta però un difetto che devo dichiarare: il campo `seven_gates.meaning` contiene la prosa *"closed-loop eligibility of A2"*. È una **stringa di valore**, non una chiave, ereditata verbatim dal tool congelato `v26b_s1b_rollout.eligibility_gates:190`, che riuso deliberatamente perché l'aritmetica dei gate resti immutata. Il mio scanner sorveglia le chiavi e non l'ha intercettata.

- **I valori dei gate sono di rev4c**, non di A2: sono calcolati sull'analisi di questo rollout.
- **Lettura autoritativa**: in `seven_gates.meaning` si legga *"closed-loop eligibility of REV4C"*.
- Non ho mutato né il receipt (content-addressed) né il tool congelato. La correzione strutturale — parametrizzare anche le stringhe di prosa, o rinominarle in scrittura — richiede un emendamento successivo e la decisione dell'architetto.

## 9. File prodotti e SHA-256

**Emendamento e tooling**

| file | SHA-256 |
|---|---|
| `v26b_amendment_rev4c_balance.json` | `b25b0fe4f89a9ea4a7ac6b6b9ca8b5d2d393e4535a2a0944edd5386dd62790bb` |
| parent `v26b_amendment_rev4b_july_dagger_s1a.json` | `883ffc9a0217fa6fefcabf820b382717b8ad2e8f7ccd5814461b0bb4218f0f93` |
| parent `v26b_addendum_rev4b_a_corrections.json` | `74f7c50a83877a2005708e95ee911992f301bbe3799f41c823f807dbbb847ce1` |

**Candidato** `candidates/REV4C_BALANCE_35D_NONDEPLOYABLE/`

| file | SHA-256 |
|---|---|
| `v26b_rev4c_receipt.json` | `c3fef01223dd8aa5ebcc9c5208407bc78dec0bdac7e2d63197e8fa319a4b4f4a` |
| `rl_module/module_state.pkl` | `dc6a7dab9fe202776501f3010b17ac37c45e7d087da3de8df0531bdd9f1c202e` |
| `rl_module/metadata.json` | `3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12` |
| `rl_module/actor_feature_manifest.json` | `b5a7745abb5cfc4c7886f68e2570fdc09e4bcc13bba4cad87a1e60008749e6cc` |
| `rl_module/class_and_ctor_args.pkl` | `c9a6722ff95642795bfe1146d0087a68b5861fd508cbe3692195b2d820d810a7` |
| digest attore | `35fef3042211d9fbf6abcc6277c5f82269c44e98b633da4f825dab3e3bb6b0e9` |

**Rollout** `rollouts/rev4c_nominal_det/REV4C_35D__v3_canonical__nominal__det/`

| file | SHA-256 |
|---|---|
| `v26b_rev4c_rollout_receipt.json` | `188574622251931577d85fa5b1fe6ec7a6fa433b396aeb10f4a1dabd9f698a50` |
| `rollout_policy_trace.json` | `8f2310928b586bed5750122478bd30cd08beba34e47e318ee95b0c2044474f63` |
| `rollout_summary.json` | `8a045c72453bfce0f5cb00289beb4943444e9db90636a7e346100c30ba2e2d3c` |
| `rollout_reset_diagnostics.json` | `3d3b8dfa9164b2e63f0ba6ca795f2abee41ef6c8bd8929e19368ad83a325ed76` |
| `watchdog_summary.json` | `79e96fd79376dd1dcd0093b5a2a47310e1a9b1af62d7419e241c7c51c29724e7` |
| `watchdog_state.json` | `078d46d35cb96d8e15a46a01d3e4814bb7a8b513dd429ad9cc9cca570885c0b3` |

Git HEAD `04c0c470da8192c6fd8496b753fa061cac7dc270`, branch `main`, produzione invariata.

## 10. Test e verifiche

- Self-test **pre-esecuzione**: 50/50 PASS. Self-test **post-esecuzione**: **64/64 PASS**, con `offline_pass: true`, `survival_steps: 116`, `causal_verdict: "SUPPORTED"`.
- Regressioni sull'igiene delle etichette: rifiuto di `kinematics_L20`, di una chiave annidata `L20_*`, di un `offline_reference_block` a qualsiasi profondità, discesa nelle liste, rifiuto del **vecchio receipt rev4b** sotto il nuovo contratto; esenzione dei soli sottoalberi di provenienza; non-estensione dell'esenzione a `diagnostics`; permanenza del divieto di `offline_reference_block` anche dentro `lineage`.
- Prova della singola variabile: `B4.fit_july` riusato e nessun iperparametro di luglio ridefinito in rev4c.
- Guardie di token: `None`, il token di rev4b, quello di L20 e la variante minuscola sono tutti rifiutati.
- Fail-closed del cap: con `PREFIX_CAP = 67` il preflight fallisce perché l'aggregato atteso non torna.
- Seconda esecuzione dello stadio rifiutata con `FileExistsError` (no-clobber verificato).
- Assenza di macchinari PPO e sigma verificata su identificatori reali (`train_ppo`, `PPOConfig`, `sigma_sweep`, `choose_sigma`, `set_sigma`), non su prosa; esattamente una invocazione di harness.
- Artefatti rev4b **non mutati**; produzione, env, reward, FSM, morfologia, SEA e C++ invariati.

## 11. Stato e conseguenze

- **Il candidato NON è promosso** e resta NON-DEPLOYABLE. Non lo sarebbe stato comunque per regola («nessuna promozione automatica»), e in ogni caso 3 dei 7 gate sono falliti.
- **Ipotesi del sovrappeso on-policy: SUPPORTED.** Il bilanciamento è una variabile causale reale. La direzione DAgger a round singolo **non** è chiusa: la clausola di chiusura era condizionata a una sopravvivenza ≤ 100 step, che non si è verificata.
- Ogni ulteriore round DAgger, test su sigma, altro candidato o PPO resta **non autorizzato**. **STOP per l'audit dell'architetto.**

## 12. TODO propagati

- **TODO-1** — Difetto residuo: `seven_gates.meaning` porta l'etichetta "A2" ereditata dal tool congelato `v26b_s1b_rollout.eligibility_gates:190`. Estendere la parametrizzazione dell'etichetta d'attore alle stringhe di prosa, non solo alle chiavi, in un emendamento successivo. *(aperto)*
- **TODO-2** — σ = 0.005 resta un placeholder di serializzazione **non risolto**; nessun test su sigma è autorizzato. *(aperto, ereditato)*
- **TODO-3** — B3 resta **INDETERMINATO**: il campo di fase è identicamente zero su ogni traccia v3, incluse le 116 righe di rev4c. *(aperto, ereditato)*
- **TODO-4** — Il conflitto strutturale cammino/plantarflessione resta irrisolto: nessun candidato completa 500/500 con caviglia negativa. *(aperto, ereditato)*
