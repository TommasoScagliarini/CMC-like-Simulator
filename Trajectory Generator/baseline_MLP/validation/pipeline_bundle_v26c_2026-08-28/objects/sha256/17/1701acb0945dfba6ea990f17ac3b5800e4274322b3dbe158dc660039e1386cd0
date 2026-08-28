# V26B S1B A2 (rev3u) — rollout nominale deterministico: **500/500, tutti e 7 i gate PASS** → `CLOSED_LOOP_ELIGIBLE_PENDING_ARCHITECT_AUDIT`

**Token:** `V26B-S1B-NOMINAL-ROLLOUT` · **Data:** 2026-08-24 · **Un solo lancio**, nessun retry, nessun altro candidato, nessun sampling σ, nessun multistart/DAgger/PPO/morphology fit, nessuna modifica production. Candidato byte-immutabile prima e dopo; nulla sotto `student/`; nessuna promozione.

## 1. Esito
**A2 cammina.** 500/500 step con `end_reason = episode_time_limit` e **tutti e sette i gate vincolanti superati**. È il primo attore della catena, oltre a S0D, a completare il nominale sotto v3.

| Gate vincolante | Osservato | Richiesto | Esito |
|---|---|---|---|
| completion | **500/500**, `episode_time_limit` | 500/500 + time limit | **PASS** |
| phase_timeout (tutte le righe) | stance 0, swing 0 | 0 e 0 | **PASS** |
| morphology_causal_contract_failure (per riga) | rows_positive 0, max 0,0, failure false | 0 / 0 / false | **PASS** |
| hs_cancelled_count | max 0, final 0 | 0 e 0 | **PASS** |
| resync_count | max 0, final 0 | ≤ 1 e ≤ 1 | **PASS** |
| valid_cycle_count | **2** (max 2, final 2) | ≥ 1 | **PASS** |
| penetrazione | **0,013845 m** (13,84 mm), nessuna terminazione `grf_penetration` | ≤ 0,020 m | **PASS** |

Contatori FSM v3 registrati senza reinterpretazione: **HS validi 3, TO validi 3, cicli 2**, invalid_event 1, stato finale `STANCE_AFTER_HS`. L'unico evento invalido è un `to_too_early_after_hs` allo **step 496** — la **stessa firma benigna di S0D** (step 495). Diagnostici morphology: `dropped_wait_hs` max 1, `terminal_flushed` 1, `cancelled_transition` 0, `timeout_transition` 0, delay 0,04 s; segmenti settled 431, scartati 0. Gate report-level di luglio: episodio completo PASS, ≥1 ciclo PASS, penetrazione sotto soft e hard, clipping 4 step (non zero), **return +41,35** (positivo).

## 2. Confronto A2 / S0D / AB06 u_IK (diagnostico, mai un gate)
**Separazione esplicita.** Le metriche *offline* del fit sono state calcolate sulle righe congelate visitate da S0D; quelle *closed-loop* qui sotto sugli stati che **A2 ha effettivamente visitato**. Non sono confrontabili termine a termine e restano in blocchi separati nel receipt.

**Cinematica (closed-loop).** Ginocchio A2: min −0,9864, max −0,1742, media −0,4644, range 0,8122, **sempre negativo**, zero step fuori bounds. Caviglia A2: min +0,0306, max +0,4542, media +0,2450, range 0,4236, **mai negativa**, zero step fuori bounds. S0D per confronto: ginocchio [−0,9966, −0,1742] media −0,4761; caviglia [+0,0153, +0,4094] media +0,2202.

**Qualità morfologica vs target prescritti.**

| | A2 | S0D | S1A |
|---|---|---|---|
| RMSE ginocchio | 0,1724 | **0,1206** | 0,3264 |
| RMSE caviglia | 0,0677 | **0,0476** | 0,3893 |
| Pearson ginocchio | 0,832 | — | 0,201 |
| Pearson caviglia | 0,800 | — | 0,246 |
| accordo di segno caviglia | 1,000 | — | 0,360 |
| rapporto di ampiezza caviglia | **1,0014** | — | 1,764 |

Lettura onesta: rispetto ai target prescritti A2 è **leggermente peggiore di S0D** (+43 % RMSE ginocchio, +42 % caviglia) ma **incomparabilmente migliore di S1A** (−47 % e −83 %), con correlazioni alte (0,83 / 0,80 contro 0,20 / 0,25) e accordo di segno pieno. L'unico indicatore in cui A2 **migliora** morfologicamente è l'**ampiezza della caviglia**, che passa a un rapporto 1,0014 contro il prescritto — cioè A2 riproduce l'escursione corretta della caviglia, dove S1A la esagerava di 1,76×. La caviglia resta però **mai negativa** (min +0,0306): il tratto negativo di tardo appoggio non compare, quindi anche se B3 fosse valutabile non sarebbe soddisfatto.

**B3: non valutabile.** Campo `pros_ankle_angle_imitation_target_phase` identicamente 0 su tutte le 500 righe, 0 righe nella finestra [0,55, 0,80] — **nonostante 2 cicli validi**. Stessa semantica registrata per S0D, R2I e S1A; il minimo caviglia complessivo (+0,0306) è riportato come diagnostico distinto e non è mai usato come B3.

**Azioni sugli stati visitati.** A2 dista da S0D **0,0653 / 0,0493** (max 0,2095 / 0,1907) — praticamente identico al drift offline misurato in fase di fit (0,0657 / 0,0455), quindi la trust region ha tenuto anche in closed loop. A2 dista da u_IK **0,2328 / 0,2510** (RMSE 0,3147 / 0,3133), mentre S0D sugli stessi stati dista **0,2881 / 0,2967** (RMSE 0,3852 / 0,3695): sulla propria traiettoria A2 ha chiuso circa il **19 % del gap sul ginocchio e il 15 % sulla caviglia**, coerente con il miglioramento offline sull'holdout (18,3 % / 13,8 %). Azioni whole-trace: raw max |·| 0,997 / 1,078, saturazione 0 % / 0,8 %, 4 righe clippate.

**Nessuna di queste metriche ha modificato o rilassato un gate.**

## 3. Incidente di tooling e sua risoluzione (dichiarato per intero)
Il rollout è stato eseguito **una sola volta** e ha prodotto tutti gli artefatti, ma la **scrittura del receipt è fallita** nel blocco `finally`: lo scanner anti-marcatura di rev3q ha rifiutato una **prosa documentale ereditata verbatim** dal tool frozen `v26b_r0a_rollout.analyse_rollout` («the deployable 35D manifest carries no *_saturated feature…»), che usa la parola in senso descrittivo e non marca nulla.

Risoluzione, senza rilanciare né toccare artefatti congelati:
1. Ho aggiunto **nel modulo nuovo** uno scanner locale con **allow-list verbatim** (una stringa alla volta, mai per pattern): ogni altra occorrenza continua a fallire chiuso. Il tool frozen `v26b_s1a_bc.py` **non è stato modificato** — è pinnato in tutta la catena rev3r→rev3u.
2. Ho aggiunto `finalize_receipt`, che **non lancia nulla** (verificato dai test sull'assenza di `subprocess.run(` e `.mkdir(` nel corpo) e ricostruisce il receipt dagli artefatti della singola esecuzione; rifiuta se gli artefatti mancano o se un receipt esiste già.
3. Il receipt registra l'incidente per intero, incluso `rollout_relaunched: false`, la provenienza del return code (0, **provato dal percorso di codice**: `run_rollout` solleva se `returncode != 0` e l'esecuzione era proseguita fino all'analisi della traccia) e l'elenco delle stringhe allow-listed.

Al secondo tentativo lo scanner ha colpito **il mio stesso testo** dell'incidente: ho riformulato tre punti nella forma `NON-DEPLOYABLE` (riconosciuta dalla normalizzazione) invece di indebolire il controllo. Lezione registrata: una guardia lessicale severa richiede disciplina di vocabolario anche nel testo che essa stessa sorveglia.

## 4. Discrepanza sui digest nel messaggio di autorizzazione
Due hash nel tuo messaggio erano **più corti di 64 caratteri** (63 e 59) e sono sottosequenze per caduta di caratteri di quelli su disco: aggregate `039475c…` vs reale **`0539475cda88030e0af059b394974938f64a03b80490134295ca0ed486ae47e6`**; receipt A2 `54011d3…` vs reale **`5430115d327ca9c0acd4ed992b7026bf4a940555544c8b81414fb72dfab78616`**. Diagnosi: **troncatura di trascrizione**, non modifica di artefatti — l'actor digest A2, citato per intero, coincide esattamente. rev3u pinna i valori **verificati su disco** e riporta verbatim quelli trascritti, così l'audit trail mostra entrambi. Nulla è stato corretto in silenzio.

## 5. Conformità
Harness identico a quello dei rollout S0D e S1A: il comando differisce da quello congelato per S1A **solo** in `--checkpoint` e `--output-dir` (verificato fail-closed in codice e nei test). Pin di produzione verificati: `rollout_eval.py` `5433bcbc…`, YAML v3 `a870cc38…`, corridor `33b1dd7c…`. Media deterministica, seed 123, start 1,956870983805102; **nessun sampling σ** e σ **non** dichiarata risolta. Candidato A2 immutato: digest dei 4 file del modulo e del fit receipt identici prima e dopo, flag `deployable:false` / `rollout_pending:true` / `sigma_unresolved:true` / `quarantined:false` mai toccati. Gli altri cinque candidati restano quarantinati e non sono stati eseguiti. Nessuna promozione, nessuna ancora costruita (`anchors_not_built: true`), nulla scritto sotto `student/`.

## 6. Test
`test_v26b_s1b_rollout.py`: **PASS 50 check pre-run**, **PASS 73 post-run** (stage-aware). Copertura: lineage rev3l→rev3u con tamper su quattro pin; verifica che le stringhe trascritte troncate **non** siano accettate e che i pin siano a 64 caratteri; **equivalenza harness** (solo `--checkpoint` e `--output-dir` possono differire, con prova che una deviazione ulteriore viene rifiutata); candidato read-only, non-deployable e non quarantinato, con controllo dei flag dimostrato vivo; quarantena degli altri cinque; **5 token negativi** e token positivo provato alla guardia no-clobber; **single-rollout** verificato prima e dopo; parsing dei contatori whole-trace su fixture con eventi invisibili nell'ultima riga; **fixture isolata per ciascuno dei 7 gate**; scanner locale (prosa ereditata ammessa e registrata, `deployable:true` e alias storico ancora rifiutati, qualunque altra occorrenza rifiutata); `finalize_receipt` token-gated, senza lanci, che rifiuta senza artefatti e quando un receipt esiste già.

**Trasparenza sugli errori miei:** il test ha scoperto un **bug reale nel driver** (leggevo `command_resolved` a livello top mentre in rev3u è annidato sotto `harness_equivalence`) — sarebbe abortito prima del lancio, corretto prima di eseguire; e due miei controlli erano scan a sottostringa troppo grezzi, che colpivano prosa descrittiva invece di call-site, resi precisi.

## 7. Artefatti
| Artefatto | SHA-256 |
|---|---|
| `…/v26b_bridge_2026-08-24/v26b_amendment_rev3u_s1b_a2_nominal_rollout.json` | `a3b17a883fb43006f7a3d49b9a1a1e77334f0974e959241af3a496f62259a011` |
| `…/v26b_bridge_2026-08-24/v26b_s1b_rollout.py` | `8fc5447ed0b7aec11c0a49c172c549a3fd01b4c091bc0ed03f5a8edac2673110` |
| `…/v26b_bridge_2026-08-24/test_v26b_s1b_rollout.py` | `84495e5694e1c7f11b60a185f71a77b4fd1124108692ba4cefee17d6c0b9bbf2` |
| `…/rollouts/s1b_a2_nominal_det/S1B_A2_35D__v3_canonical__nominal__det/v26b_s1b_rollout_receipt.json` | `3dd5637bc864e99340bba91118d6fcd035cf37d3d5789fd650e977ee84891a17` |
| `…/S1B_A2_35D__v3_canonical__nominal__det/rollout_policy_trace.json` | `f9535754e99a6809b371af6f5ec6273ed2715459d2dc0b10e3dd8328d1ef6169` |
| `…/S1B_A2_35D__v3_canonical__nominal__det/rollout_summary.json` | `dcf34e6c7876f27d13cf92443186c5dfdbe22b6ce917622e973517ebb37fb7c5` |
| `…/logs/s1b_a2_nominal_det_rollout.log` | `2791f8b4e77d165b756d78c18290d06ded174e6ae12ba6f7c03897360fcacb52` |
| A2 actor digest (invariato) | `cde2c8e6356f833d9e1108b542286c2c9265f8f69bb4c0a894c36136c5d0916c` |

## 8. Stato
**STOP per audit Codex.** A2 è marcato **soltanto** `CLOSED_LOOP_ELIGIBLE_PENDING_ARCHITECT_AUDIT`: non è promosso, resta NON-DEPLOYABLE, non è stata costruita alcuna ancora e σ resta irrisolta. Nessun altro stadio è stato avviato.

**Significato del risultato, senza sovrainterpretazione:** l'ipotesi falsificabile di rev3s aveva chiesto se un passo limitato (≤ 0,10) preservasse il ciclo del passo che un passo di 0,384 aveva distrutto. La risposta di questa singola prova nominale è **sì**, con qualità FSM sostanzialmente identica a S0D e con circa il 15–19 % del gap verso u_IK effettivamente chiuso in closed loop. Resta vero che un singolo start nominale non dichiara V3 (servirebbero 3 start, non valutati) e che il grosso del gap — l'holdout resta a 0,508 / 0,394 — richiederebbe ulteriori iterazioni, ciascuna con i propri token.
