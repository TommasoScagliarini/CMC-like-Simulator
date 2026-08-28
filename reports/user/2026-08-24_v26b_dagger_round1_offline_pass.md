# V26B — DAgger round 1 (rev3d, July-faithful): gate offline PASS ed export (report immutabile)

**Data:** 2026-08-24 · **Emendamento:** `rev3d_dagger_round1_july_faithful` → SHA `7b995277103bb5cf01ed8cbf46098425747a371e573b35ec44627b37907a5adf` (pinna rev3c `723c8981…`, receipt rollout `ae04bcda…`, addendum `0884a25b…`, trace `8e3aff5465f53f9d…`, report diagnosi `d4755b15…`, tooling/test rollout, init R0a `8567071e…`) · **Esito: GATE OFFLINE PASS su tutti i ruoli, EXPORT completato. NESSUN rollout del nuovo attore. STOP per audit Codex.**

## 1. Supersessione rev3d (solo la clausola 500/500)
Per QUESTO primo DAgger nominale deterministico il prefisso terminato presto è usato per intero (storia 11/07: `aggregate_dagger_traces` non troncava i rollout det; il round 1 aggregò il prefisso BC da 68 step); il troncamento discreto resta riservato alle future trace recovery/stocastiche e dovrà usare la nominale det **dello stesso attore**. Tutto il resto di rev3c invariato.

## 2. Protocollo storico citato e deviazioni preregistrate (in rev3d, PRIMA dell'esecuzione)
Flusso 11/07: `target_domain_dagger.py::main` (gate teacher r.85-87) → `aggregate_dagger_traces` (label `teacher_actions[step-1]`) → `adapt_actor`. Budget r2 verbatim (`…dagger_2026-07-11_r2/adaptation_report.json`): 839 righe (671/168), 400 epoche patience 60 best 372, batch 64, lr 3e-4, seed 123, clip 1.0, **logstd_weight 0.1 (distillazione soft, Δmax 0.011)**, anchor_weight 1e-05. Deviazioni preregistrate: 300 epoche fisse senza early stopping (= modalità storica fixed_final_epoch; split 20% SOLO diagnostico), batch 256/lr 1e-4/seed 2026 (budget congelato della catena), **logstd CONGELATA al placeholder** (più severo di luglio), λ_anchor 1e-3 verso θ_R0a, aux head λ_φ 0.1, init = R0a esattamente.

## 3. Dataset round 1 (`v26b_dataset_DAGGER_R1_*.npz`, receipt content-addressed)
500 righe BC (stati anchor det nominal, u_IK) + **493 righe on-policy R0a** (obs pre-azione dalla trace pinnata, ULTIMA riga inclusa: la guardia `joint_divergence` è scattata DOPO quell'azione — documentato) = 993 → **dedup bitwise: 1 collisione** (riga di reset condivisa, purposes `ik_nominal_det`/`ik_onpolicy_det`, **label identiche** — nessun conflitto) → **992 righe uniche**. Label = u_IK same-step sulla cache nominale FULL-pinnata (`3dd878d4…`); nessun riferimento V26 per troncamento; vietati e assenti: label raw V26, alt-start, JUL_H0, T1/T1R/B0820.

## 4. Refit e gate (soglia 0.15/giunto per ruolo e aggregato + Q3)
| insieme | righe | RMSE knee | RMSE ankle | init R0a (knee/ankle) | esito |
|---|---|---|---|---|---|
| aggregato | 992 | **0.0909** | **0.0494** | 0.3425 / 0.1894 | **PASS** |
| BC nominale | 500 | **0.0801** | **0.0394** | 0.0716 / 0.0501 | **PASS** |
| **on-policy** | 492 | **0.1007** | **0.0579** | 0.4810 / 0.2642 | **PASS** |
| holdout 20% (diagnostico, 198 righe, split leakage-free dichiarato) | — | 0.0929 | 0.0525 | — | ≈ aggregato: nessun overfitting |

**Requisito primario soddisfatto**: gli stati on-policy sono corretti (0.481→0.101 knee, 0.264→0.058 ankle) SENZA perdere la BC nominale (knee 0.0716→0.0801, ankle migliorata 0.0501→0.0394; entrambe ampiamente nel gate). Q3 PASS (10 chiavi, clock zero+invarianza bit-identica, logstd placeholder, save/reload bit-exact all'export). Loss 0.848→0.081; shift parametri 6.33. **σ=0.005 resta NON deciso e NON usato qui** (placeholder di serializzazione; nessun gate lo legge; il σ operativo sarà deciso dallo sweep V4) — dichiarato in fit report, gate, manifest del modulo e receipt.

## 5. Export e artefatti
- Modulo: `…/student/V2_DAGGER_R1/rl_module/` — **actor digest `c7bcee1c1165625d0c574a709a9444a98ea95edea8fcddd5d81f59dfa2e3d31e`**, `module_state.pkl` `12447f48b245…`; promozione atomica `renameatx_np RENAME_EXCL`.
- Receipt canonico: `v26b_dagger_r1_receipt.json` → **`f409e7880e5c8be3c17d81c00b1ddc48c4c4e6c25058b0b55e452aaebb346c42`** (pinna catena rev3…rev3d, init R0a, trace/receipt/addendum rollout, cache FULL, dataset npz `ce309b40…` + receipt, budget, history completa, gate, digest codice).
- Tooling: `v26b_dagger_r1.py` `a742855be1f3a16a…`, `test_v26b_dagger_r1.py` `c783ac20c004ab67…` — **SELFTEST 36/36** (lineage con 3 negativi, dataset reale con collisione verificata, equivalenza bit-identica del loop col fit R0a provato, gate per-ruolo isolante, export no-clobber, immutabilità frozen).
- Log no-clobber `dagger_r1_*.log`, REAL_EXIT_CODE=0.

## 6. Stato
Nessun rollout del nuovo attore, nessun PPO/critic, produzione intoccata; artefatti frozen ri-verificati. **STOP per l'audit Codex.**
