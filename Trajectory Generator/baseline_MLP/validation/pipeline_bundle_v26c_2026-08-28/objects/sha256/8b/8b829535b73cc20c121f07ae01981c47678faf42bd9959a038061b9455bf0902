# V26B — Rollout nominale deterministico S0D: GATE PRIMARIO PASS 500/500 (report immutabile)

**Data:** 2026-08-24 · Token `V26B-S0D-NOMINAL-ROLLOUT` · Comando/config ESATTI rev3c/e/j; tooling `v26b_s0d_rollout.py` `10f0c461d35a377e…` + test `35b7119112e0eeed…` (SELFTEST 12/12 pre-esecuzione; pin fit-receipt `1b2dbab4…`, module_state `cda6d893…`, actor `481dd0d2…`, addendum `177126d0…`) · **Esito: COMPLETE — 500/500 `episode_time_limit`. Primo student causale 35D della catena a completare il nominale sotto v3.**

## Receipt `cbec1a671b7cdf2980881ec4ca33f69534e66d94cbbfa86c7b9cd9c1a39412b7`
| voce | valore |
|---|---|
| **GATE primario** | **500/500, `episode_time_limit` → PASS** |
| cicli / HS / TO validi | **2 / 3 / 3** |
| eventi | 1 invalid `to_too_early_after_hs`@495 (STESSA firma benigna del V26 det anchor @491), resync 0, hs_cancelled 0 |
| contatori corretti whole-trace | causal-fail **0**, timeout stance **0** / swing **0** |
| knee q | [−0.997, −0.174], mean −0.476; RMSE vs prescritto **0.1206** |
| ankle q | [+0.015, +0.409], mean +0.220; RMSE vs prescritto **0.0476** |
| penetrazione max | **13.7 mm** (sotto soft 20 e hard 28) |
| clipping / return | 15 (V26 det: 12) / **+28.34** (V26 det: +32.75) |
| **B3 phase-window [0.55,0.80]** | `not_evaluable_no_valid_phase_or_cycle` (campo phase = 0 su tutte le righe); **min globale caviglia +0.0153 (diagnostica distinta: MAI negativa)** |
| mismatch discreto vs anchor V26 | step 14 (diverge presto ma si AGGANCIA al proprio ciclo, a differenza di R*) |

## Catena closed-loop (nominale det)
**S0D 500/500** ← R0a 493 (`joint_divergence`) ← R1 242 ← R2I 197 (`grf_penetration`). La tesi S0D (prima il comportamento, poi l'adattamento) è confermata al primo colpo closed-loop: la distillazione pura V26→35D produce andatura agganciata (3 HS validi), penetrazione dimezzata rispetto a ogni R* e firma FSM del lignaggio V26.

## Note oneste
1. Questo è un **pre-gate diagnostico su UN solo start**: il gate V3 pieno (3 start) resta NON VALUTATO (vincolo architetturale in vigore).
2. **B3 non valutabile** (campo phase a 0 anche con 2 cicli validi — comportamento del campo sotto v3 da chiarire) e comunque **la caviglia S0D non ha tratto negativo** (min +0.015): se B3 fosse valutabile col criterio ≤ −0.03 NON lo soddisferebbe — differenza cinematica reale dal riferimento protocollare, ereditata dal comportamento V26 distillato.
3. Nessuna IK automatica; nessun fit/PPO/critic/σ-sweep; artefatti preservati (log `s0d_rollout_driver_*.log` REAL_EXIT_CODE=0).

**STOP per l'audit dell'architetto.** Nulla promosso implicitamente.
