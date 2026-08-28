# V26B rev4b — DAgger July-faithful da S1A: **fit offline quasi perfetto, closed-loop a 42 step**

**Token:** `V26B-REV4B-JULY-DAGGER` · **Data:** 2026-08-24 · Un solo round, nessuna nuova collection, nessun retry, nessun PPO, nessuna scelta σ, nessuna promozione, nessuna modifica a production/env/reward/FSM/morphology/SEA/C++.

## 1. Esito in due righe
Il fit July-faithful ha **funzionato**: RMSE aggregato **0,36074 → 0,03251**, e sulle finestre a target negativo la frazione di comando positivo scende da 0,268 a **0,0515** — il **segno è recuperato nello spazio del comando**. Il rollout closed-loop, eseguito automaticamente perché i gate offline sono passati, **collassa a 42/500 step** con `grf_penetration` a 28,2 mm, **0 HS, 0 TO, 0 cicli**. Esito: `CLOSED_LOOP_FAIL_QUARANTINED`.

## 2. Verifica dei valori July: l'anchor 1e-3 **non** è confermato
Come richiesto, ho letto i sorgenti e gli artefatti invece di fidarmi della memoria. `target_domain_imitation.py` (righe 1353-1361) e `target_domain_dagger.py` (righe 57-67) danno:

| parametro | valore July verificato | valore catena R* agosto |
|---|---|---|
| epochs | **400** | 300 |
| batch_size | **64** | 256 |
| learning_rate | **3e-4** | 1e-4 |
| validation_fraction | **0,20** (early stopping) | nessuna |
| patience | **60** | — |
| clip_weight | 1,0 | 1,0 |
| logstd_weight | **0,1** | assente |
| **anchor_weight** | **1e-5** | **1e-3** |
| seed | **123** | 2026 |
| trace_repeat | **4** | — |
| gradient clipping | **nessuno** (0 occorrenze di `clip_grad_*`) | nessuno |

**L'anchor 1e-3 non è confermato dal protocollo July**: è il valore della catena R* di agosto — quella a cui tu stesso hai attribuito la degradazione R0a→R1. Il July riuscito usa **1e-5**, ed è quello che ho impiegato. La condizione che avevi posto («mantienilo se confermato») si risolve quindi in negativo.

Loss July replicata alla lettera: `MSE(means, targets) + 1,0·relu(|means|−1)² + 0,1·MSE(logstd, source_logstd) + 1e-5·mean_p (p−p_init)²`, con `restore_logstd_head()` e azzeramento delle colonne clock **dopo ogni step**, nessuna testa ausiliaria di fase, nessun hinge, nessuna conservazione S0D. Aggregazione verbatim: `concat(teacher, tile(visited, 4))`, **nessun troncamento e nessun dedup**, etichette **time-aligned** con `teacher_index = step − 1`.

## 3. Preflight bloccante: GO su tutti gli item
- traccia S1A: **392 righe reali**, step contigui 1…392, digest `6546befc…` verificato, `obs35` tutti finiti e **identici ai vettori registrati** riga per riga;
- allineamento causale same-time: `max |t_S1A − t_corpus| = 0,0` esatto;
- etichette ricalcolate col **medesimo teacher** (u_IK AB06 same-time dalla cache pinnata), **non** healthy symmetry e **non** time-shifted;
- audit duplicati/collisioni: 0 duplicati esatti nel corpus e nel prefisso, 2 collisioni bitwise corpus↔prefisso, **0 con label in conflitto** (July non deduplicava: l'audit è diagnostico, solo il conflitto sarebbe fail-closed);
- dataset = corpus BC-IK 500 + prefisso S1A tiled ×4 = **2068 righe**, nessun dato S0D/L20/A2;
- copertura: il prefisso copre **65 delle 97 righe** a target ankle negativo, su **5 finestre** [6,14], [112,132], [174,179], [267,290], [330,334]; le label su quelle righe sono i comandi negativi del teacher (minimo **−0,13182 rad** decodificato) consegnati allo stesso indice temporale, quindi l'allineamento **permette realmente** il recupero del segno;
- `logstd` byte-identica al placeholder S1A, σ **UNRESOLVED**.

## 4. Fit e gate offline
400/400 epoche (best epoch 399: l'early stopping non è scattato, la validation stava ancora migliorando), 1654 righe di training e 414 di validation, loss 0,11982 → 0,00052, val MSE 0,07341 → 0,00172.

Gate vincolanti — **tutti PASS**: integrità (10 chiavi, clock zero, invarianza bit-identica, **logstd byte-identica a S1A**, save/reload esatto, no critic); preservazione di funzione T1 3,972e-08 e T2 1,186e-07 ≤ 1e-5; convergenza del fit 0,36074 → **0,03251** (l'unico criterio offline documentabile da luglio, che riporta 0,973332 → 0,011002).

Misure preregistrate senza soglie inventate:

| misura | valore |
|---|---|
| vs IK, corpus 500: RMSE knee/ankle | **0,04481 / 0,03381** |
| vs IK, corpus 500: Pearson knee/ankle | **0,9916 / 0,9871** |
| vs IK, corpus 500: rapporto di ampiezza | **0,9936 / 0,9992** |
| vs IK, prefisso 392: RMSE knee/ankle | 0,02163 / 0,01795 |
| finestre negative: frazione comando positivo | 0,268 → **0,0515** |
| finestre negative: comando minimo | **−0,18004** |
| max_abs vs S0D (**informativo**) | 1,45923 / 1,18137 (media 0,37114 / 0,30271) |
| simmetria sana (**diagnostica**) | 0,09472 / 0,13769 |

Nello spazio del comando questo attore **insegue l'IK quasi perfettamente** (correlazioni 0,99, ampiezze 1,00) e **recupera il segno** sul 94,8 % delle righe di finestra. Il `max_abs` vs S0D di 1,46 conferma, a titolo informativo, quanto S1C-3 aveva dimostrato: arrivare qui richiede spostamenti molto oltre 0,25 — ed è per questo che quel limite non è un gate qui.

## 5. Closed-loop: il fenomeno dell'11/07 riprodotto quasi alla lettera
| | luglio 11/07 (clone BC) | rev4b (DAgger da S1A) |
|---|---|---|
| errore offline | RMSE azione **0,011002** | RMSE aggregato **0,032507** |
| esito closed-loop | **68/500** | **42/500** |
| causa di fine | `grf_penetration`, 25,164 mm | `grf_penetration`, **28,217 mm** |
| cicli validi | 0 (1 HS, 0 TO) | **0** (0 HS, 0 TO) |

Cinematica del rollout: ginocchio in [−0,30258, −0,16753], range 0,135 — **praticamente immobile**; caviglia in [+0,02397, +0,07874], range 0,055. Reserve max 379,5 Nm, return −2,271. I contatori FSM e morphology sono puliti (phase_timeout 0/0, causal failure 0, hs_cancelled 0, resync 0): **non è un guasto dei meccanismi, è il collasso posturale prima ancora che il ciclo si stabilisca**.

**Gate qualitativo caviglia (vincolante): FAIL.** Nella finestra ex ante — le righe della sua traccia con target IK ankle negativo, 9 righe nei 42 step — il minimo realizzato è **+0,02397 > 0**: nessun valore negativo compare. La soglia documentata F2R B3 (≤ −0,03) non è raggiunta. La frazione di comando positivo in finestra è **0,667** contro **0,887** di L20 — un miglioramento di segno **nel comando**, riportato come misura preregistrata senza soglia inventata, che però non si traduce in nulla al giunto perché l'attore non cammina.

## 6. Lettura, senza andare oltre i dati
La catena ha ora quattro punti che legano fedeltà all'IK e capacità di camminare:

| attore | fedeltà all'IK | closed-loop |
|---|---|---|
| S0D | RMSE giunto 0,126 / 0,135 | **500/500**, 2 cicli |
| L20 | RMSE giunto 0,117 / 0,118 | **500/500**, 2 cicli |
| S1A | drift azione 0,384 / 0,289 | 392/500, 0 cicli |
| **rev4b** | **RMSE comando 0,045 / 0,034** | **42/500**, 0 cicli |

La relazione è monotona e ora quantificata su quattro punti: più l'attore insegue l'IK, meno cammina. rev4b è l'attore più fedele all'IK mai prodotto dalla catena ed è anche il peggiore in closed-loop. Il DAgger July-faithful ha fatto **esattamente ciò che luglio fece** — e ha riprodotto **anche** il fallimento che a luglio richiese round successivi di DAgger più PPO per essere superato. Un singolo round, come autorizzato, non basta: a luglio il round 1 diede 45 step, il round 2 356, il round 3 regredì a 221, e nessuno completò l'episodio.

Non traggo da qui alcuna conclusione sul se proseguire: il round successivo, o qualunque altra via, è una tua decisione.

## 7. Conformità e test
S1A usato nei tre ruoli richiesti — policy di collection (riuso del suo rollout congelato da 392 step, **nessuna nuova collection**), init esclusivo del fit, parametro di anchor. Nessun dato S0D/L20/A2 nel dataset; JUL_H0 mai init né label. Candidato materializzato NON-DEPLOYABLE e no-clobber. `max_abs` vs S0D registrato **informativo**, simmetria sana **diagnostica**, σ **UNRESOLVED**, B3 non valutabile sul campo di fase (nullo). `student/` invariato; receipt S1A, `osim_trj_cmc_like.py` (`26458424…`) e `target_domain_imitation.py` (`442be222…`) riverificati byte-identici.

`test_v26b_rev4b_dagger.py`: **PASS 40 pre-esecuzione**, **PASS 50 post-esecuzione**. Copertura: pin/tamper rev4b; verifica che le costanti siano i valori **July** e non quelli di agosto (anchor ≠ 1e-3, batch ≠ 256, seed ≠ 2026); assenza di gradient clipping, testa di fase, hinge e termini di preservazione nel sorgente; 4 token negativi; **tutti gli item di preflight**, con prova che digest o conteggio righe sbagliati lo fanno fallire chiuso; **semantica di aggregazione July** verificata riga per riga (primi 500 = corpus verbatim, le tre ripetizioni sono copie esatte, label = teacher time-aligned); una sola invocazione dell'harness e nessuna collection; assenza di marcature deployable; e post-esecuzione i parametri July nel receipt, `logstd` byte-identica dopo il fit, `max_abs` informativo, simmetria diagnostica, gate caviglia vincolante con finestra ex ante, e il rifiuto di una seconda esecuzione.

Trasparenza sui miei errori, tutti corretti prima o durante lo sviluppo: un costrutto residuo e un target sbagliato nel termine logstd, riscritti nella forma July (`source_logstd` = uscita logstd del modulo sorgente sulle stesse osservazioni); e un mio check di test che cercava `rollout_eval` minuscolo mentre la costante è `F1.ROLLOUT_EVAL`, reso preciso e rafforzato con il conteggio delle invocazioni.

## 8. Deviazioni dichiarate da luglio
1. **Bilanciamento on-policy**: `trace_repeat = 4` replicato verbatim, ma il nostro prefisso è 392 righe contro le 68 di luglio, quindi la quota on-policy è **75,8 %** contro il **35,2 %** di luglio. Non è possibile avere insieme il parametro e il bilanciamento.
2. **Holdout assorbito**: il corpus è quello completo da 500 righe come istruito, quindi l'holdout contiguo 201-300 entra nel fit; l'insieme trattenuto è quello July (split casuale 20 % con seed 123 per l'early stopping). Conseguenza dichiarata: misure S0D-ancorate su quelle righe sarebbero in-sample per questo candidato.
3. **Scaling fisico July** usato nella parametrizzazione del fit e assorbito all'export (T1/T2 ≤ 1e-5): è la correzione documentata dal 13/07 contro le accelerazioni non scalate fino a 60 rad/s²; non cambia loss, split, label né iperparametri.

## 9. Artefatti
| Artefatto | SHA-256 |
|---|---|
| `…/v26b_amendment_rev4b_july_dagger_s1a.json` | `883ffc9a0217fa6fefcabf820b382717b8ad2e8f7ccd5814461b0bb4218f0f93` |
| `…/v26b_rev4b_dagger.py` | `e354b38e326128e39b8615a4cff0bd3480cb10d7bc611a3b879f7ad7f72db041` |
| `…/test_v26b_rev4b_dagger.py` | `8c24a087c414a1cb77877cf4cba75bd908b6ef6801438a51bb33d0e2105cfdb9` |
| `…/candidates/REV4B_JULY_DAGGER_35D_NONDEPLOYABLE/v26b_rev4b_receipt.json` | `7f7f10419c84b1a1d03d91082b579551c662ed4d8b579e1a26c79798328f2565` |
| `…/REV4B_JULY_DAGGER_35D_NONDEPLOYABLE/rl_module/module_state.pkl` | `db11c0b1f24118b4ebc271a91e17230ff6132c093f1351514733603b242b64ec` |
| `…/rollouts/rev4b_nominal_det/REV4B_35D__v3_canonical__nominal__det/v26b_rev4b_rollout_receipt.json` | `2032c5a9d5ee448cb45cebf65d13b40efeae062a58c8260fb7f4db413978c1b8` |
| `…/rollout_policy_trace.json` | `c27d06e7eb21880893321bd3353086aa6ff3339ad1bce39eb797605fe0299ae8` |
| `…/logs/rev4b_nominal_det_rollout.log` | `5152aea978514947e9afeb68133ea18f7ea25bd9047c5bad6c839b14e72b0ed6` |
| `…/rev4b_stage_20260824_220918.log` | `2fa5304a3bbf73e7367dd36104921ed12237c59c44af19627b29af60a7398e54` |
| rev4b actor digest | `592604dedf6c5829dcc91d5e3a9595cda9fe0a0a81466580300804a5ebbe1035` |

## 10. Stato
**STOP per il tuo audit.** Il candidato rev4b è **quarantinato** (`CLOSED_LOOP_FAIL_QUARANTINED`), non promosso, NON-DEPLOYABLE; nessun retry e nessun secondo round eseguiti. S0D e L20 restano gli unici attori della catena che completano il nominale; L20 resta `CLOSED_LOOP_ELIGIBLE_PENDING_ARCHITECT_AUDIT` e non promosso. σ irrisolta, B3 INDETERMINATE, production invariata.
