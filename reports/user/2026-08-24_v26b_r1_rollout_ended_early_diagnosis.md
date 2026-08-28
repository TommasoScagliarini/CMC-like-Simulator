# V26B — Rollout nominale deterministico R1 (rev3e): TERMINATO PRESTO a 242/500 — diagnosi (report immutabile)

**Data:** 2026-08-24 · **Emendamento:** `rev3e_r1_nominal_rollout` → SHA `8ac5af69f7914d2a8de19b7da73cd19351876ff3e561660c1b42f51a35b7c04c` (creato PRIMA dell'esecuzione; pinna rev3d `7b995277…`, receipt R1 `f409e788…`, report `61b8caa0…`, dataset npz `ce309b40…`+receipt `765c8a90…`, modulo R1 con SHA completi, tooling/test) · **Esito: 242/500, `grf_penetration` → preservato tutto, diagnosi, NESSUN DAgger automatico, STOP per audit Codex.**

## 1. Esecuzione (una sola, nessun retry)
Preflight rev3e: catena rev3…rev3e, digest attore R1 `c7bcee1c…` e manifest 35D verificati PRIMA del lancio; stesso runtime/config/protocollo di R0a (rollout_eval `5433bcbc…`, config v3 `a870cc38…`, det seed 123, start esatto); directory nuova no-clobber. Tooling: `v26b_r1_rollout.py` `c63949cde72638c9…` + `test_v26b_r1_rollout.py` `7cbcf622373b63af…` — **SELFTEST 19/19** (riuso read-only delle funzioni rev3c/rev3d validate; fixture R0a reale). Receipt content-addressed: **`0fc762fd3271b84907a0cc332f49f16461fa2c948d032eb21962c96038687611`**; trace `a73c4d8f8e26e3e5…`; log driver REAL_EXIT_CODE=3.

## 2. Post-processing corretto sull'INTERA trace (mai end_reason/ultima riga)
| metrica | valore |
|---|---|
| steps / end_reason | **242/500, `grf_penetration`** (max 29.41 mm > hard 28; media 7.2 mm) |
| knee q | min **−1.0205**, max −0.1742, mean −0.6055, 100% negativo, 0 fuori bounds; RMSE vs prescritto 0.3995 |
| ankle q | min −0.3706, max +0.2407, mean −0.1253, 57% negativo, 0 fuori bounds; RMSE vs prescritto 0.4289 |
| azioni (whole-trace) | max\|raw\| **0.828 / 0.629**, saturazione **0** righe, clipping **0**, raw==applied ovunque; final-20 knee mean +0.431 |
| return | **−29.85** |
| reserve | max 814.5 Nm, mean 299.8 |
| cicli/eventi | **valid HS 0** (max e finale), valid TO 1, **cicli validi 0**; invalid 0, resync 0 (max/finale), hs_cancelled 0 (max/finale); nessun evento |
| timeout (per-riga) | phase_timeout_stance **0**, phase_timeout_swing **0** |
| morphology causale (per-riga) | failed_closed rows **0** (max 0, failure False); dropped_wait_hs/terminal_flushed: 0 |
| mismatch discreto vs anchor V26 | step 12 (diagnostico) |

## 3. Confronto esplicito con R0a (e i soli omologhi di luglio)
| | R0a | R1 |
|---|---|---|
| steps / end | 493, `joint_divergence:pros_knee_angle` | **242, `grf_penetration`** |
| knee q range | [−0.339, −0.000], mean −0.19 (ROM ridotto, collasso in estensione) | **[−1.020, −0.174], mean −0.61** (flessione profonda) |
| RMSE vs prescritto (knee/ankle) | 0.365 / 0.130 | 0.400 / 0.429 |
| saturazione raw knee | 33 step clipped; final-20 mean **+1.11** | **0** clipped; final-20 mean +0.43 |
| penetrazione max | 27.3 mm (sotto hard) | **29.4 mm (oltre hard)** |
| cicli validi / HS validi | 2 / 3 | **0 / 0** |
| contatori FSM/morph/timeout | tutti 0 (corretti per-riga) | tutti 0 (corretti per-riga) |
| return | +0.53 | −29.85 |

Omologhi storici (solo il confrontabile: steps-to-termination in rollout nominale det, guardie DIVERSE 15/25 vs 20/28 mm): luglio BC 68 → DAgger r2 **356** (terminato anch'esso per penetrazione, 26.5 mm); catena V26B: R0a 493 → R1 **242**. **Nessuna conclusione di qualità biologica dal solo gate offline.**

## 4. Diagnosi (fatti)
Il round 1 ha **eliminato la modalità di fallimento di R0a** (nessuna saturazione, nessuna deriva in estensione: il ginocchio ora flette profondamente, fino a −1.02 rad) ma ha **sostituito** quella modalità con penetrazione precoce: eseguendo la cinematica prescritta via u_IK **senza mai stabilire un contatto valido** (0 HS validi in 242 step, FSM ferma in attesa), la flessione profonda affonda il piede fino a 29.4 mm. La sequenza discreta diverge dal riferimento V26 già allo step 12 (come R0a): lo student cammina subito fuori da entrambe le varietà di training (griglia nominale V26 + stati R0a). Struttura identica al percorso storico: anche il DAgger r2 di luglio terminava per penetrazione; luglio uscì da lì con più round/dati (recovery stocastici phase-aligned e teacher alt-start, 13/07) — ogni prosecuzione è però una TUA decisione; nessuna intrapresa qui.

## 5. Stato
Artefatti preservati (job dir, receipt, trace, log); catena rev3…rev3e, R0a, R1, dataset, produzione: intoccati e ri-verificati. σ operativo inesistente/inutilizzato (placeholder 0.005 NON deciso; decide V4). Nessun PPO/critic/training, nessun nuovo rollout, nessun DAgger. **STOP per l'audit Codex.**
