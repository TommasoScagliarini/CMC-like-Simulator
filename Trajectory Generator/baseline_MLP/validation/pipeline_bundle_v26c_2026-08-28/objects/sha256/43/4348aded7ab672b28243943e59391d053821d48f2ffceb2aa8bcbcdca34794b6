# V26B — Raccolta anchor: report di copertura (immutabile)

**Data:** 2026-08-24 · **Protocollo:** `V26B-bridge-rev3` (`5e0b6a5cdbec…`) · **Stage autorizzato:** `V26B-ANCHORS` (GO dell'architetto dopo approvazione V0) · **Esito: PASS** · Questo report è immutabile: ogni futura rettifica avviene in un nuovo file.

## 1. Esecuzione
- Comando esatto: `v26b_anchors.py --collect --workers 2 --authorized-stage V26B-ANCHORS`, log no-clobber `collect_driver_20260824_114126.log` (`62a34be46f220af8…`), **REAL_EXIT_CODE=0**.
- Output root: `Trajectory Generator/runs/rollout/validation/v26b_bridge_runs/2026-08-24_V26B_anchors_r1/`.
- 36/36 job eseguiti (nessun fallimento, nessun retry), durata per job 477–518 s, totale CPU 296 min, wall ≈ 148 min con 2 worker.
- Nessun artefatto V1/gate/dataset creato; manifest dry-run storici intatti.

## 2. Moduli σ derivati (mean bit-identica a V26 `5bbc6cbd…`)
| modulo | σ | actor digest derivato | module_state | via |
|---|---|---|---|---|
| `V26_39D_s00025` | 0.0025 | `9659565b17a352dc…` | `01eaa5be98bfaf5e…` | v26b mirror |
| `V26_39D_s0005` | 0.005 | `527e8b09ddf83d1a…` | `b8eb33afaac38ee4…` | F1 tool |
| `V26_39D_s001` | 0.01 | `1bdf505ee28352f3…` | `b9670c0662e89427…` | v26b mirror |

## 3. Copertura (target obbligatorio, nessun waiver)
- **Righe uniche bitwise: 19 314 ≥ 16 000 → PASS** (grezze 19 500 = 36×500 stocastiche + 1 500 det).
- **Trace complete per σ: 12/12/12 ≥ 3 → PASS** (tutte le 39 trace, det incluse, 500/500 step).
- Duplicati rimossi: 186 = **36 righe di reset** (39 trace → 3 reset unici per start) + **150 mid-trace** (righe iniziali bitwise-identiche fra trace dello stesso start prima della divergenza del rumore; per start: minus020 41, nominal 50, plus020 59). Nessun conflitto di label (dedup fail-closed superato).
- Report copertura del tooling: `coverage/anchor_coverage_20260824_140923.json` (`5c8123607bba3bf6…`), `status: PASS`.

## 4. Target (regola rev3)
Target = **mean deterministica V26** calcolata offline sugli obs39 visitati (route A_iso6clk), mai l'azione campionata. Verifica per trace mean offline vs `policy_action_mean` registrata: **max deviazione su 39 trace = 2.38e-07** (≤ 1e-5). Equivalenza teacher/runtime confermata funzionalmente su ogni trace.

## 5. End reasons e orizzonte
**39/39 trace `episode_time_limit` a 500/500 step** — nessuna terminazione per penetrazione, timeout o contract failure. Cicli validi: **2 per ogni trace** (39/39).

## 6. Trace con contatori FSM non-zero (8/39 — diagnostica esplicita, tutte INCLUSE dai filtri rev3)
| trace | σ | start | eventi | invalid | resync |
|---|---|---|---|---|---|
| A_ISO39CLK_V3 nominal det (anchor pinnato F1, seed 123) | det | — | `to_too_early_after_hs`@491 → resync@499 | 1 | 1 |
| A26_s00025 nominal seed1003 | 0.0025 | nominal | `to_too_early_after_hs`@493 (nessun resync) | 1 | 0 |
| A26_s00025 plus020 seed1000 | 0.0025 | plus020 | `to_too_early_after_hs`@473 → resync@481 | 1 | 1 |
| A26_s001 nominal seed1001 | 0.01 | nominal | `to_too_early_after_hs`@492 → resync@500 | 1 | 1 |
| A26_s001 nominal seed1003 | 0.01 | nominal | `to_too_early_after_hs`@492 → resync@500 | 1 | 1 |
| A26_s001 plus020 seed1000 | 0.01 | plus020 | `to_too_early_after_hs`@478 → resync@486 | 1 | 1 |
| A26_s001 plus020 seed1001 | 0.01 | plus020 | `to_too_early_after_hs`@474 → resync@482 | 1 | 1 |
| A26_s001 plus020 seed1003 | 0.01 | plus020 | `to_too_early_after_hs`@473 → resync@481 | 1 | 1 |

Fatti, senza interpretazione oltre l'evidenza:
- Firma comune: **TO del terzo ciclo giudicato troppo precoce a fine orizzonte** (step 473–493), sempre **rifiutato** dal debounce; nei 7 casi con resync la FSM si ri-arma esattamente **8 step dopo**. `hs_cancelled=0` e `timeout=0` ovunque; mai su minus020.
- **La trace det nominal pinnata (anchor F1 di luglio) porta già la stessa firma** (@491→@499): le anomalie stocastiche nominal/plus020 replicano un comportamento baseline del V26 det, non un artefatto del rumore in sé.
- Dipendenza da σ non monotona: σ=0.0025 → 2/12, **σ=0.005 → 0/12**, σ=0.01 → 5/12.
- Contatori aggregati stocastici+det: invalid_total=8, resync_total=7, hs_cancelled_total=0, timeout_total=0.

## 7. Morphology
`morphology_discarded_segments = 0` su tutte le 36 trace stocastiche (settled segments 309–449); `failed_closed` mai vero. Nessun evento di contract failure causale.

## 8. Pin e provenienza
- Tooling: `v26b_anchors.py` `20e6db93…`, protocollo `5e0b6a5c…` (pin a tre vie verificati nel dry-run 113751 pre-GO).
- Runtime: config `a870cc38…`, `rollout_eval` `5433bcbc…`, teacher V26 actor `5bbc6cbd…` / module_state `0ba56eb7…`.
- Seed anchor-only {1000, 1001, 1002, 1003}; seed 123–128 mai toccati (det anchor = trace 123 pinnate esistenti, sola lettura).

## 9. Stato
**STOP.** V1/student NON avviato: in attesa del gate dell'architetto sul presente report di copertura.
