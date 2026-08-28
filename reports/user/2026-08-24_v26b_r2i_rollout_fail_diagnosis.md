# V26B — Rollout nominale deterministico R2I (rev3j): GATE FAIL a 197/500 — diagnosi (report immutabile)

**Data:** 2026-08-24 · **Emendamento:** `rev3j_r2i_nominal_rollout` → SHA `723474d3692db2c1199c1e535b05f0e217a37103ef076048198eaa8ea5d9e82b` (creato PRIMA del codice; pinna receipt/digest R2I, rev3i, tooling riusato, config di produzione; comando canonico rev3c/rev3e riusato ESATTAMENTE; gate preregistrato = 500/500 `episode_time_limit`, nessuna soglia inventata o allentata) · **Esito: `ENDED_EARLY_FAIL` — 197/500, `grf_penetration` (28.9 mm > hard 28). Nessun rilancio. Nulla promosso.**

## 1. Esecuzione (una sola; SELFTEST pre-rollout 19/19)
Tooling `v26b_r2i_rollout.py` `8a874e6a911741bf…` + `test_v26b_r2i_rollout.py` `32b5f9f4229f20b2…` (post-run-safe by construction: guardie su JOB_DIR monkeypatchata). Preflight: catena rev3…rev3j, digest attore `f6579a7f…`, manifest 35D, pin produzione (rollout_eval `5433bcbc…`, yaml v3 `a870cc38…`, corridor `33b1dd7c…`) tutti verificati. Log driver `r2i_rollout_driver_*.log` REAL_EXIT_CODE=3.

## 2. Receipt (gate preregistrato DISTINTO dalla diagnostica) — `6d31bc82137914540c6969540c621373206f51c336257f10852c66b47716e14b`, trace `035146abead1e834…`
| voce | valore |
|---|---|
| **GATE preregistrato** | steps 197/500 attesi, `grf_penetration` → **FAIL** |
| HS/TO/cicli validi | **0 / 1 / 0** |
| contatori corretti per-riga | causal-fail 0 (max 0), timeout stance 0 / swing 0; invalid/resync/hs_cancelled 0 (max e finale) |
| knee q | [−0.7857, −0.1742], mean −0.411; RMSE vs prescritto **0.3525** |
| ankle q | [−0.1818, +0.3509], mean +0.123; RMSE vs prescritto **0.1731** |
| **tratto negativo caviglia** | PRESENTE: min **−0.1818**, 30.5% step negativi, 2 run; diagnostico B3 (≤−0.03) SODDISFATTO |
| riferimento prescritto (griglia) | knee [−1.122, −0.190]; ankle [+0.086, +0.441] (mai negativo: il tratto negativo è proprietà protesica) |
| azioni | max|raw| 0.813/0.511, saturazione 0, clipping 0, raw==applied |
| penetrazione / reserve / return | max 28.9 mm (>20 soft, >28 hard), mean 11.5 mm / max 409 Nm / **−7.18** |
| mismatch discreto vs anchor V26 | step **14** |

## 3. Confronto a tre (diagnostico)
| | R0a | R1 | **R2I** |
|---|---|---|---|
| steps / end | 493, `joint_divergence` knee | 242, `grf_penetration` | **197, `grf_penetration`** |
| RMSE vs prescritto (knee/ankle) | 0.365 / 0.130 | 0.400 / 0.429 | **0.353 / 0.173** |
| saturazione knee | 33 step, final20 +1.11 | 0, +0.43 | **0, +0.27** |
| HS/cicli validi | 3 / 2 | 0 / 0 | **0 / 0** |
| penetrazione max | 27.3 mm | 29.4 mm | 28.9 mm |
| mismatch vs V26 | 12 | 12 | 14 |

## 4. Diagnosi onesta (fatti; decisioni all'architetto)
1. R2I insegue il riferimento MEGLIO di R1 (ankle RMSE 0.43→0.17, knee 0.40→0.35, tratto negativo di caviglia ora presente e B3-conforme, zero saturazioni) — l'obiettivo offline è stato raggiunto e trasferito.
2. **Ma il fallimento closed-loop è lo stesso di R1 e ANTICIPATO** (197 vs 242): la sequenza discreta diverge dal cammino V26 allo step 14, **nessun heel strike valido viene mai stabilito** (0 HS in 197 step), la flessione prescritta senza fase di contatto agganciata affonda il piede fino alla guardia. I contatori FSM/morphology/timeout sono puliti: non è un guasto dei meccanismi, è l'assenza del ciclo.
3. **Trend della catena**: 493 → 242 → 197. Il miglioramento offline (holdout 0.0679/0.0439) NON si traduce in sopravvivenza closed-loop; anzi correla negativamente negli ultimi due round. Contesto storico onesto: il parent di luglio che riuscì (13/07) partiva da un init che GIÀ camminava 500/500 nominale; l'intera catena V26B-R* parte da un init (V1/R0a) che non ha mai stabilito il ciclo sotto v3.
4. Nessuna opzione eseguita: ogni prosecuzione (altri round DAgger, revisione dell'approccio bridge, ritorno all'analisi del perché il ciclo non si aggancia — es. fase di contatto iniziale/reference governor) è decisione architetturale tua.

## 5. Stato
Artefatti preservati (job dir, receipt, trace, log); `V2_R2I` resta candidato NON promosso; `V2_R2G` resta AUDIT-REJECTED; nessun PPO/critic/training/DAgger/σ-sweep/stochastic rollout; produzione intoccata; σ placeholder NON deciso. **STOP per il tuo audit.**
