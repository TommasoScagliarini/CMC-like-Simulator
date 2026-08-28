# V26B — Diagnostica di fattibilità R0: conflitto di target dimostrato (report immutabile)

**Data:** 2026-08-24 · **Ordine dell'architetto post-audit R0** (R0 resta FAIL; vietati waiver/R1/V3/modifiche a soglie, β, budget, protocollo) · **Scope rispettato:** diagnostica pura, nessun export, nessun rollout, artefatti congelati intoccati (rev3 `5e0b6a5c…`/`7ef70cbf…`, rev3a `f74672ad…`, V1 `ae846220…`, V26 `0ba56eb7…`, dataset/receipt/log R0). Tooling **additivo**: `v26b_r0_diag.py` (`b60a9271d818683f…`), `test_v26b_r0_diag.py` (`11319888477c7df6…`) — SELFTEST 16/16. Risultati: `r0_feasibility_diag_20260824_144743.json` → SHA `61a09baf9a2ad3465312687180e4ad07fc7d6dd6f1480cf6a5637e8bb0f1b5f8`.

## (1) Q del V1 iniziale (sui dataset R0 congelati)
| | knee | ankle |
|---|---|---|
| Q1 vs u_T (anchor nominal) | 0.9271 | 0.6229 |
| Q2 vs u_IK (alt start) | 0.7049 | 0.5690 |

L'init parte lontano da **entrambe** le famiglie di target (Q3 PASS). Confronto storico: il 13/07 l'"RMSE aggregato prima" era **0.0197**.

## (2) Separabilità obs35 fra i ruoli (feature standardizzate; 33/35 usate — escluse le 2 colonne clock morte [0,1])
Distanze nearest-neighbour (unità di deviazione standard); **stati VICINI, mai chiamati identici** (0 righe bitwise-identiche cross-ruolo):

| distribuzione | p5 | p25 | p50 | p75 | p95 |
|---|---|---|---|---|---|
| cross task→pres | 0.286 | 0.434 | 0.605 | 1.503 | 5.723 |
| intra-task | 0.182 | 0.336 | 0.429 | 0.527 | 0.692 |
| cross pres→task | 0.247 | 0.365 | 0.480 | 0.660 | 1.605 |
| intra-pres | 0.232 | 0.386 | 0.474 | 0.573 | 0.734 |

Per circa metà delle righe alt-start il vicino nominale è **alla stessa distanza dei vicini del proprio ruolo** (p25 cross 0.434 ≈ p50 intra 0.429): sovrapposizione forte delle regioni di stato, con una coda (p75+) genuinamente separata (transitori specifici dello start). **Gap di target sulle coppie cross-NN**: knee p50 0.288 / media 0.376 / p95 1.082; ankle p50 0.269. Nel **decile più vicino** (1 287 coppie, distanza mediana 0.285) il gap knee **sale a 0.475**: proprio dove gli stati quasi coincidono, le etichette dei due ruoli confliggono di più.

## (2b) Bound empirici kNN (holdout deterministico ogni 5ª riga; train = unione con target del proprio ruolo)
| predictor | pres-holdout vs u_T (Q1-like) | task-holdout vs u_IK (Q2-like) |
|---|---|---|
| kNN k=1 union | **0.370 / 0.286** | **0.249 / 0.203** |
| kNN k=5 union | **0.293 / 0.228** | **0.176 / 0.137** |
| kNN k=1 intra-role | 0.112 / 0.117 | 0.036 / 0.021 |

Anche una **lookup table pura** sugli obs35, addestrata su entrambi i ruoli, resta ≫ Q1 0.10 sul lato preservazione e sopra Q2 0.15 (knee) sul lato task: qualunque funzione di obs35 deve dare UNA uscita per stato, e nelle regioni sovrapposte le etichette confliggono. (Nota di rigore: i floor kNN non sono lower bound stretti per un modello parametrico — il fit MLP mono-ruolo fa meglio del kNN intra-ruolo — quindi la prova sharp è al punto 3.)

## (3) Fit diagnostici mono-ruolo (stesso init V1, stesso budget congelato 300 ep/lr 1e-4/batch 256/seed 2026/λ identiche; MAI esportati)
| fit | Q1 vs u_T | Q2 vs u_IK | esito |
|---|---|---|---|
| **pres-only** (digest `c240ee39…`) | **[0.0859, 0.0800] → Q1 PASS** | [0.494, 0.374] FAIL | preservare V26 da solo è raggiungibile |
| **task-only** (digest `b8b6dd30…`) | [0.474, 0.379] FAIL | **[0.0344, 0.0300] → Q2 PASS** | inseguire u_IK da solo è raggiungibile |

**Ciascun criterio è individualmente raggiungibile col budget congelato; il fallimento congiunto di R0 NON è un limite di ottimizzazione: è un conflitto di target fra i ruoli** (fittare un ruolo spinge l'altro a ~0.37–0.49 ≈ il gap comportamentale u_T↔u_IK).

## (4) Verifica storica puntuale (luglio) — citazioni
- `reports/user/2026-07-11_riadattamento_imitativo_target_domain.md` — «Strategia 3: behavior cloning actor-only»: dataset **500 campioni** (400/100 train/val), etichetta = «azione assoluta normalizzata del teacher» (il **teacher prescribed** con lookahead, cfr. `reports/daily/2026-07-11_daily-report.md` r.52 «Anche il teacher prescribed falliva prima del TO», r.70 «Con il limiter corretto il teacher ha completato il contratto target»); «initial action RMSE 0.973332 → adapted 0.011002»; **mono-ruolo** (nessun ruolo dati di preservazione; solo «piccolo anchor loss rispetto ai pesi iniziali», cioè regolarizzazione parametrica). Source 31 feature → target 39.
- `reports/user/2026-07-13_sblocco_actor_markov35_e_warmup_critic.md` — dataset finale «ancore nominali del source actor 16000 / recovery stocastici phase-aligned 712 / teacher su start −0.20/+0.20 s 8000 = 24712»; «**RMSE aggregato prima 0.019657** → dopo 0.008144; RMSE −0.20 0.007830; +0.20 0.007209; shift nominale RMS 0.004175».
**Conclusione storica**: sì, l'assegnazione ruoli di luglio era start-split (ancore nominali + teacher alt-start), e **la preservazione era davvero il source actor == init**: l'init del 13/07 era l'attore già adattato l'11/07 **sul teacher prescribed** (BC mono-ruolo), quindi ancore nominali (le sue stesse azioni) e teacher alt-start erano **mutuamente consistenti per costruzione** — RMSE iniziale 0.0197, entrambi i ruoli chiusi sotto 0.01 insieme. R0 ha invertito quest'ordine: preservare un comportamento (mean V26) che l'init V1 non possiede (Q1 iniziale 0.93) contro un teacher (u_IK) inconsistente con esso di ~0.37.

## (5) Irrigidimento tooling (additivo, niente riscritto)
Digest **COMPLETI** delle 3 cache IK pinnati in `v26b_r0_diag.py` e verificati fail-closed (`verify_ik_caches_full`), con cross-check di coerenza dei prefissi congelati in `v26b_v2.py` (che resta intoccato): minus020 `f97ad154f75541626565f7d6fd392dface4f723c2861143f1dc8172536767230`, nominal `3dd878d4d6d2930d730c1a67f39d6799f20221e7659a435c02d062bfd553d9b0`, plus020 `f15d624cc910b815ee4c511e6702e1312cc4be0f67e93ae435b3e14b850f7d20`. Test: pin corretti PASS, pin manomesso rifiutato, coerenza prefissi, caricamento dataset congelati legato al receipt.

## (6) Conclusione e proposta (nessuna azione intrapresa)
**Conclusione: CONFLITTO DI TARGET, non limite di ottimizzazione né difetto di ruoli/tooling.** Evidenze convergenti: (a) mono-ruolo PASS ciascuno col budget congelato; (b) gap 0.475 knee proprio sulle coppie di stati più vicine; (c) anche il kNN-union viola le soglie; (d) luglio riuscì perché i ruoli erano consistenti per costruzione (ordine: prima BC sul teacher, poi preservazione di sé stesso).
**Proposta da sottoporre a decisione (emendamento richiesto, non implementato):** replicare l'ORDINE di luglio — R0a mono-ruolo BC sul teacher u_IK (il fit task-only mostra Q2 ≈ 0.034/0.030 raggiungibile; eventualmente esteso ai 3 start), poi R0b con preservazione ridefinita rispetto al RISULTATO di R0a (le sue azioni nominali, "source actor == init" come a luglio) al posto della mean V26 — mantenendo la mean V26 come controllo informativo. In alternativa: rinunciare al task IK a R0 (solo preservazione V26, Q1-only) e spostare il carico correttivo sul DAgger. Entrambe cambiano `dataset_roles`/semantica Q del rev3 → richiedono un tuo emendamento additivo.

**STOP** prima di qualsiasi emendamento o nuovo R0; rev3/rev3a/V1/V26 immutabili ri-verificati.
