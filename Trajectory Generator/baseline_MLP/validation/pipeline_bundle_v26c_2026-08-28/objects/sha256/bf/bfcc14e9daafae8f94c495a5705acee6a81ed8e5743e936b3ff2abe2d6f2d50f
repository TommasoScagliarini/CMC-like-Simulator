# V26B — S0D-0/S0D-1 (rev3k): preregistrazione + pre-gate di identificabilità PASS (report immutabile)

**Data:** 2026-08-24 · **Emendamento:** `rev3k_s0d_pure_distillation` → SHA `8f34c95ee7c69029e0d3e8715f3e62fe9652533a03e0dd4b8fb7b38114377437` (creato PRIMA del tooling; congela le 8 decisioni: distillazione pura V26→35D su tutte le 19 314 righe, init SOLO V1 `ae846220…`, split per-JOB pre-dedup, pre-gate 0.15 riusato, fit futuro row-uniform con anchor storico 11/07 **1e-5** mean-6-tensori-FULL, postfit 0.10 = Q1 esistente, rollout futuro separato NON autorizzato, clausola DAgger V26-teacher con nuovo emendamento, IK automatico vietato) · **Eseguito SOLO S0D-1: pre-gate read-only. Nessun fit/export/rollout. Esito: PASS.**

## 1. Tooling e test (prima dell'esecuzione)
`v26b_s0d.py` → `66df172e43d07d51…`, `test_v26b_s0d.py` → `f261c88a2dd3fb63…` — **SELFTEST 27/27**: lineage con tamper, ordini dichiarati == librerie (contratto rng), selezione holdout deterministica (mai seed 123), funzione pura `dedup_and_assign` (conflitto label → abort; priorità holdout su provenienza condivisa; zero cross-split), split reale con scan esplicito di leakage, copertura delle 9 celle, label legate al teacher pinnato (fatto registrato: max |u_T| raw = **1.431** — mean pre-clip, può superare 1), **guardia fit**: rifiuto con nome del token futuro `V26B-S0D-FIT` non concesso.

## 2. Split leakage-free per JOB (pre-dedup, rng 2026, ordine dichiarato)
Held-out: 1 trace intera per cella (9), registrate nel receipt (es. `A26_s00025__minus020__seed1003`, …); det seed-123 tutte in train. Dopo dedup con priorità holdout: **train 14 834 / holdout 4 480** (= 19 314), 29 righe multi-provenienza, **zero obs bitwise cross-split** (scan esplicito), digest degli indici registrati.

## 3. Pre-gate S0D-1 (kNN k=5, scaling solo-train, 33 feature attive; soglia 0.15/giunto RIUSATA)
| insieme | righe | RMSE knee/ankle |
|---|---|---|
| aggregato | 4 480 | **0.0919 / 0.0945** |
| minus020 / nominal / plus020 | 1 494 / 1 488 / 1 498 | 0.0897–0.0964 / 0.0904–0.0984 |
| 9 celle start×σ | 490–509 | tutte ≤ **0.1023** (peggiore: minus020×0.0025) |
**PASS su ogni insieme.** Diagnostici (nessuna soglia nuova): la scala di robustezza σ dimostrata dalla collezione è 0.0025–0.01 — l'RMSE kNN (~0.09) resta un ordine sopra: registrato come il punto falsificabile che il futuro rollout dovrà dirimere; scan ambiguità firma causale su u_T: 27 gruppi con spread > 0.2 (stesso caveat ciclico noto, diagnostico).

## 4. Receipt e stato
Receipt pre-gate: `v26b_s0d_pregate_receipt_20260824_170731.json` → **`1a5c6b54dd70cb2a79d0768eaac3d97fb83863fbaf852fb1694c072c79348437`** (pin, split, gate completo, diagnostici, code digests). Log `s0d_pregate_*.log` REAL_EXIT_CODE=0. Nessun artefatto esistente toccato; fit bloccato dietro token distinto non concesso; σ operativo inesistente. **STOP per audit Codex.**
