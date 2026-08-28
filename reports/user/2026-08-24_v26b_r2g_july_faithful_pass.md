# V26B — R2G (rev3g, July-faithful): gate prefit+postfit PASS, export completato (report immutabile)

**Data:** 2026-08-24 · **Emendamento:** `rev3g_r2_july_faithful` → SHA `bb4401fc7003d1c8a5c30ed04fe75a1bd5d3fbec555fc91c2f5e681fa12656b5` (creato PRIMA dell'esecuzione; masse esatte correttive del testo rev3f; pin: catena rev3…rev3f, parent R1 `c7bcee1c…`, **parent storico JUL_H0 byte-provato** `…markov35_phase_aligned_scaled_full_r32_alt8_2026-07-13` con `module_state 44457ca5…`, dataset, tooling pre-esistente, interpretazione clip=penalty-azione dichiarata) · **Esito: PASS ed export atomico.**

## 0. Tentativo fallito — dichiarazione esplicita (ordine dell'architetto)
Log **`2026-08-24_V26B_anchors_r1/r2g_20260824_162935.log`**: le 300 epoche completarono (weighted_flat_mse 0.0103257 → 0.00175182) ma lo stage FALLÌ nella costruzione del receipt per `AttributeError: v26b_dagger_r1 non ha PIN_AMENDMENT_REV3B` (costante referenziata dal modulo sbagliato; proprietario reale: `v26b_r0a`). Staging rimosso dal fail-closed, lock rilasciato, NESSUN artefatto promosso da quel tentativo. Causa corretta usando il modulo proprietario (`A.PIN_AMENDMENT_REV3B`), rev3g intoccato. **Nota di sequenza**: il rilancio corretto è partito prima che l'ordine dell'architetto arrivasse (messaggi incrociati); il test anti-ricorrenza è stato aggiunto subito dopo e verifica ANCHE post-hoc che il blocco provenance del receipt canonico coincida col builder. Anti-ricorrenza: `provenance_block()` è ora una funzione costruita e validata (7 digest 64-hex dai moduli proprietari) **prima di qualsiasi fit** in `run_r2g`, e testata pre-fit nel selftest.

## 1. Esecuzione valida (nuovo log no-clobber, fit deterministico completo)
Log `r2g_20260824_163136…log` (REAL_EXIT_CODE=0). Prefit → fit → postfit → export in un'unica invocazione gated (`--authorized-stage V26B-R2G`).

## 2. Gate prefit (vincolanti PASS; diagnostici riportati)
- **G1**: invariato, PASS (98/98 prefisso ≡ R0a; 144 escluse, 0.595 < 0.80).
- **G3-blocked VINCOLANTE** (kNN k=5, split preregistrato group/block/embargo): aggregato **0.0358/0.0239**; base 0.0699/0.0586; prefix 0.0262/0.0707; alt 0.0206–0.0408 — tutti ≤ 0.15.
- **Envelope magnitudine VINCOLANTE** (κ_m=10, SF=2, Q99 a-priori): 59 350 coppie, **0 violazioni**, worst excess ×5.79 — PASS.
- Diagnostici (non auto-promuoventi): densità envelope 5.78% vs tail adjacent 0.029% (ratio 201×); G3 row-random 0.0299/0.0189.
- **Controfattuale obbligatorio**: le 144 righe post-mismatch → kNN-vs-label **0.452/0.248** → **RESPINTE** dal gate blocked ✓.

## 3. Fit July-faithful (spazio scalato, funzione R1 preservata)
- **Scaling fisico luglio** su 10 feature (indici 25–34; vel 4.0/3.5, accel 60/55, altre 1.0): training su `x/scale`, init `W1_R1·scale`, anchor su θ_R1_scaled, export `W1/scale` assorbito.
- **Test numerici preregistrati (tol 1e-5, TUTTE le righe)**: T1 pre-fit max abs **7.60e-08**; T2 export max abs **1.00e-07** — la funzione R1 iniziale e l'export raw sono numericamente identici alle controparti scalate.
- **Objective**: FLAT MSE pesata per massa (λ per-riga = w_r/n_r^train, Σλ=1 sul train; masse esatte 0.6474587245063127 / 0.028811913240530916 / 0.16186468112657817×2), niente normalizzazione per varianza, **niente aux**; anchor **semantica luglio** 0.01·Σ mean-squared-delta per tensore verso θ_R1_scaled; numerici di catena (300 fixed, b256, lr 1e-4, seed 2026, clip-azione 1.0 — nessun gradient clipping, mai esistito in nessuna delle due linee, dichiarato); logstd placeholder congelata; no critic. Fit sul SOLO train (10 727 righe; holdout+embargo esclusi). Loss 0.01033 → 0.00175.

## 4. Gate postfit (vincolante sul HOLDOUT mai visto dal fit)
| insieme | righe holdout | RMSE knee/ankle | esito |
|---|---|---|---|
| aggregato | 3 188 | **0.0675 / 0.0438** | PASS |
| base | 200 | 0.1062 / 0.0818 | PASS |
| r1_prefix | 20 | 0.0336 / 0.0264 | PASS |
| alt_minus020 | 1 489 | 0.0585 / 0.0378 | PASS |
| alt_plus020 | 1 479 | 0.0696 / 0.0422 | PASS |
Metriche fit-corpus (train, DISTINTE): aggregato 0.0607/0.0387; base 0.0424/0.0256; prefix 0.0444/0.0391. Q3 PASS (10 chiavi, clock zero+invarianza bit-identica, logstd placeholder, save/reload bit-exact, no critic); smoke test d'inferenza in spazio export PASS; shift parametri vs R1 (raw) 18.6; produzione/FSM/reward/morphology/SEA/C++ intoccati.

## 5. Export e SHA
- Modulo: `2026-08-24_V26B_anchors_r1/student/V2_R2G/rl_module/` — **actor digest `65276df60b20b0180cfea6695bbbf4fb82e685b0dd0ea52578297e0f147c891a`**, `module_state.pkl` `491a56a371c1b854…`; promozione atomica nativa.
- **Receipt canonico** `v26b_r2g_receipt.json` → **`5374f1af7529272827b8327b1a00483d4ffe9e81f24374c920d370373251bc3c`** (pinna: 7 emendamenti, lineage completa, parent storico, scaling+T1/T2, dataset+split, gate pre/post, history completa, code digests).
- Tooling: `v26b_r2g.py` → `f4ba832ae8e2c898…`, `test_v26b_r2g.py` → `280a691f7d34eb5f…` (post-refactor provenance; il receipt pinna la versione eseguita — evoluzione post-run documentata, pattern della catena).
- **Suite complete rieseguite**: student 83, v2 46, r0a 42, r0a_rollout 38, r1_postrun 23, dagger_r1 36, r2_offline 16, **r2g 20** (incl. provenance pre-fit + verifica post-hoc receipt==builder) — PASS. `test_v26b_anchors`: FAIL atteso e documentato (test V0 pre-collection non ri-eseguibile dopo la raccolta autorizzata: asseriva l'assenza delle dir che la collezione legittima ha poi creato; 256/256 storico agli atti; file pinnato nel protocollo → non mutato).

## 6. Stato
σ resta placeholder NON deciso (V4); nessun rollout/PPO/critic/sweep. **STOP per audit dell'architetto.**
