# V26B — V2/R0: gate offline FAIL (report immutabile)

**Data:** 2026-08-24 · **Protocollo:** `V26B-bridge-rev3` (parents IMMUTABILI `5e0b6a5c…`/`7ef70cbf…`) + emendamento additivo `rev3a` (`f74672ad…`) · **Stage autorizzato:** `V26B-V2-R0` · **Esito: R0 NON supera il gate offline (Q1 FAIL, Q2 FAIL, Q3 PASS) → export e rollout RIFIUTATI fail-closed, STOP per audit dell'architetto.** Nessun retry, nessun tuning.

## 1. Correzione di lineage eseguita (ordine dell'architetto)
L'inserimento in-place di rev3a nei file rev3 è stato **revertito byte-identico** ai pin del receipt V1 (verifica esplicita: JSON `5e0b6a5cdbec…` ✓, MD `7ef70cbf2f72…` ✓). Rev3a vive nel **file additivo** `v26b_amendment_rev3a_q2b_causal_order.json` → SHA `f74672ad9d60775f18049bec058d793c246bac739fbf18a023a32798d9bb78a0` (parent path+hash, testo, autorità, timestamp). Ogni receipt V2 pinna entrambi.

## 2. Tooling V2 (`v26b_v2.py` `683895bb6b23cc7b…`, test `5913a8556b2e1946…` — SELFTEST PASS 46 check)
- **Fit = mirror fedele di `f2r_refit.fit_student_preserving`** (F2R immutabile: il suo contratto T1R rifiuta i seed anchor 1000–1003 e i purposes V2): equivalenza **bit-identica** dimostrata nel selftest (digest attore, ogni tensore, history) su input numericamente identici. Differiscono solo contratto dataset (ruoli/seed V26B) e criteri (Q invece di P).
- Contratto V2 fail-closed: seed ∈ {1000–1003} ∪ {123 (anchor det)}; 124/125/126–128 rifiutati; purposes per ruolo; larghezze/finitezza; ruoli mai vuoti.
- `verify_lineage()` fail-closed prima di ogni fit/export: parents rev3, emendamento, receipt V1 `65b447ea…`, modulo V1 (`ae846220…`/`16c2d1ae…`), V26 (`0ba56eb7…`), coverage (`5c812360…`).
- Budget congelato = `budget_frozen` rev3 (verificato contro il parent a runtime): Adam lr 1e-4, batch 256, 300 epoche, seed 2026, λ_clip 1, λ_φ 0.1 (testa ausiliaria solo training), λ_anchor 0.001 (‖θ−θ_V1‖²), β=1; varianze per giunto fissate a priori e registrate; logstd congelata costante (placeholder σ 0.005); nessun critic.

## 3. Dataset R0 (statico, costruito e persistito — receipt `febf65b5d1684096…`)
Assegnazione ruoli **start-split (July-faithful)**, motivata nel receipt: Q2 si chiama `alt_start_rows` nel rev3; la ricetta di luglio usava 16 000 anchor nominali + righe teacher alt-start; il gap misurato |u_T−u_IK| ≈ 0.37 medio/giunto su stati identici rende insoddisfacibile una lettura same-input dual-target; i ruoli sono bitwise disgiunti (0 collisioni cross-start, fail-closed).
- **Preservazione** = 6 438 righe nominal (u_T = mean deterministica V26); **task** = 12 876 righe minus020/plus020 (u_IK knee+ankle dalle cache privilegiate F2R pinnate, lookup esatto sulle griglie anchor); clock prescritto (unit circle) per la testa ausiliaria; dedup within-role verificato; collisioni cross-role = 0.
- File: `v26b_dataset_R0_task_20260824T143724.npz` `9862d5f83f0471f8…` (12 876), `_pres_` `4c42a9a285dc2f88…` (6 438).

## 4. Fit R0 e criteri (ri-esecuzione diagnostica deterministica per estrarre le metriche; stato NON esportato)
| criterio | knee | ankle | soglia | esito |
|---|---|---|---|---|
| **Q1** RMSE vs mean V26 (anchor nominal) | **0.2498** | **0.2312** | ≤ 0.10 | **FAIL** |
| **Q2** RMSE vs u_IK (alt start) | **0.2114** | 0.1330 | ≤ 0.15 | **FAIL** (knee) |
| **Q2b** | — | — | — | NON APPLICABILE a R0 (rev3a, righe on-policy inesistenti) |
| **Q3** invarianti (10 chiavi, clock zero+invarianza, logstd costante) | — | — | — | PASS |

Loss (media epoca): e1 3.993 → e50 0.955 → e150 0.920 → e300 0.899 (task_norm 0.352, pres_norm 0.530): plateau, non un problema di budget. Varianze fissate: task [0.1200, 0.0430], pres [0.1580, 0.0897]. Shift parametri ‖θ−θ_V1‖² = 15.42, max|ΔW1| = 0.347. Digest dello stato non esportato: `cf5f06d8c21db5b9255d5152296996a521863ebff26a0fe836e2faae65a7fb98`. Diagnostica: `r0_diagnostic_20260824_143910.json` `6b80340ca0150c5b…`; log run gated `r0_fit_20260824_143720.log` `6b854edb4500f5a1…` (REAL_EXIT_CODE=1, export rifiutato da `assert_q_r0`).

## 5. Diagnosi e rischi (fatti; le decisioni sono dell'architetto)
1. **Vincolo strutturale dominante**: |u_T−u_IK| ≈ 0.37 medio (knee p90 ≈ 0.87) su stati funzionalmente sovrapposti fra start; lo student stateless deve discriminare il contesto di start dai soli obs35. Il compromesso group-balanced si ferma a ~0.21–0.25 da CIASCUNA famiglia di target — coerente con ~metà del gap comportamentale. Q1 ≤ 0.10 e Q2 ≤ 0.15 simultanei non sono raggiungibili in questo assetto col budget congelato.
2. **Differenza strutturale rispetto a luglio**: a luglio la preservazione usava le azioni del SOURCE actor che era anche l'init (loss di preservazione iniziale ≈ 0); qui l'init V1 parte a RMSE 0.93/0.63 da u_T (degrado del transplant: persi target privilegiati + clock). R0 deve ricostruire la funzione V26 da un init degradato e contemporaneamente inseguire IK altrove.
3. **Q3/invarianti e infrastruttura**: tutto verde (contratto deployable rispettato; il fallimento è di accuratezza, non di struttura).
4. Opzioni possibili (tutte = decisione/emendamento dell'architetto, nessuna intrapresa): rilettura dei ruoli/target (es. u_IK anche su nominal, o preservazione u_T su tutti gli start senza task IK a R0), ricalibrazione β o soglie R0, budget diverso, o accettare R0 come ponte e spostare il carico correttivo sul DAgger R1 (richiederebbe emendamento del gate). Nessuna è stata implementata.

## 6. Immutabilità e stato
Parents rev3, emendamento, modulo V1, receipt V1, V26, coverage: tutti ri-verificati invariati. Nessun modulo V2 esportato; nessun rollout/V3/PPO eseguito. **STOP in attesa dell'audit dell'architetto.**
