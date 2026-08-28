# V26B S1 — rev3n: pre-gate kNN5 a 5 fold bloccati simmetrici, mediana ≤ 0,15 — ESITO PASS

**Token:** `V26B-S1-REV3N-PREREG-MEDIAN` (NON `V26B-S1-FIT`) · **Data:** 2026-08-24 · **Ambito:** addendum + tooling/test + UNA esecuzione del pre-gate + report. Nessun fit, export o rollout. Nessuna modifica a env/reward/FSM/morphology/SEA/C++ né ad artefatti già congelati.

## 1. Esito
Pre-gate rev3n **PASS** (exit code 0). Mediana dei 5 fold: **knee 0,12943**, **ankle 0,09171** — entrambe ≤ 0,15 (soglia invariata). Per-fold knee/ankle:

| Fold (step) | train | knee RMSE vs u_IK | ankle RMSE vs u_IK |
|---|---|---|---|
| [1, 100] | 390 | 0,12231363947516088 | 0,06477057368499389 |
| [101, 200] | 380 | 0,12943356017583720 | 0,09006443918628317 |
| **[201, 300]** | 380 | **0,15220686466232133** | **0,10626628039651334** |
| [301, 400] | 380 | 0,09296463559837395 | 0,11119583413241982 |
| [401, 500] | 390 | 0,15203741616628838 | 0,09171088852703550 |

Non vincolanti, sempre riportati: max per giunto [0,15221, 0,11120]; media [0,12979, 0,09280]; fold ≤ 0,15 = **3/5 knee**, **5/5 ankle**; feature costanti escluse per fold, identiche in tutti e 5: **[0, 1, 17]** (clock azzerato per costruzione + `phase_fsm_wait_hs`, costante 0 sul trace S0D).

## 2. Verifiche di integrità superate prima di ogni numero (fail-closed)
- **Fold 3 riproduce BIT-EXACT la coppia FAIL originale** `[0.15220686466232133, 0.10626628039651334]`: il fold [201,300] *è* la finestra di rev3l, calcolata dallo stesso identico percorso di codice (kNN k=5 batched, `argpartition(k-1)`, standardizzazione solo-train, esclusione feature costanti). Se non coincidesse, il tool solleva `Rev3nError` e non scrive nulla.
- **Additività provata**: `v26b_s1_prereg.py` e `test_v26b_s1_prereg.py` NON sono stati toccati — SHA attuali `858037282a…` e `1074e70f80…`, **identici ai `code_digests` registrati dentro il receipt FAIL** `a437f1ef…`, letti dal receipt stesso e confrontati a runtime. Il receipt FAIL resta quindi byte-riproducibile end-to-end. Lo strumento rev3n vive in un modulo separato.
- **Evidenza FAIL preservata**: `v26b_s1_pregate_receipt_20260824_173741.json` verificato ancora `a437f1ef0d7762db5914d14be19b390ddc6ada9a126d9db2ef8f810d08a43389`; **superseded soltanto come strumento di pre-gate**, mai come evidenza. rev3l `7545201d…` e rev3m `8af3ad65…` riverificati intatti.
- **Nessuna scorciatoia sui parametri**: `RMSE_MAX is v26b_s1_prereg.PRE_GATE_RMSE_MAX` (0,15) e `EMBARGO is …EMBARGO` (10) sono **riferimenti**, non copie — il test fallisce se qualcuno li ridefinisce. k=5 e le label u_IK (cache AB06 `3dd878d4…`) invariati.

## 3. Contesto diagnostico chiave (non vincolante)
Con lo **stesso strumento a 5 fold** applicato alle azioni **proprie di S0D** (comportamento certificato dal rollout 500/500) la mediana è **knee 0,16625 / ankle 0,20402**: la policy certificata **fallirebbe questo gate su entrambi i giunti**, mentre le label u_IK lo passano [0,12943 / 0,09171]. Le label prescritte AB06 sono quindi *più* identificabili in spazio-stato delle azioni della policy che già cammina — l'evidenza contro l'ipotesi "u_IK non è apprendibile a questa densità" è ora quantitativa, non argomentativa. Confronto di catena obbligatorio invariato: JUL_H0 offline 0,008144 con 500/500 (guardie July 15/25 mm); S0D fit holdout [0,0811, 0,0793] con 500/500 (guardie v3). Gap strutturale sulle 500 righe invariato (knee mean 0,3758 / ankle mean 0,3014): resta **diagnostica attesa**, mai gate.

## 4. Regole post-fit registrate nel receipt (vincolanti quando e se sarà emesso V26B-S1-FIT)
- `G_task_blocked_holdout`: holdout bloccato **201–300**, RMSE vs u_IK ≤ 0,15/giunto, **strettamente out-of-sample** — il fit dovrà escludere dal training le righe 201–300 **e** l'embargo 191–200 / 301–310 (`G_task_excluded_from_fit_training_rows`). Non può diventare metrica in-sample.
- Metriche full-500: **diagnostiche**, mai gate.
- **Q1 source-holdout rev3m ≤ 0,10/giunto: BINDING** — righe provenance-held rev3k, 4480 − 6 bitwise condivise escluse = **4474 usabili**, label = S0D deterministic mean same-state (riverificato costruibile in questa esecuzione).
- Parameter anchor July 1e-5 verso θ_S0D; T1/T2 solo equivalenza scaling/export; nessun gate di drift sulle righe di task; init esclusivamente S0D `481dd0d2…`.

## 5. Correzione richiesta al report diagnostico (punto 5 dell'ordine)
Nessuna riscrittura necessaria e nessuna evidenza immutabile toccata: il report diagnostico on-disk `reports/user/2026-08-24_v26b_s1_pregate_diagnostica_e_proposta_rev3n.md` (SHA **`9b40f32f2677e529e33528dc784f7a8f90008c055a24d669ff6206e3d85a0dbe`**) contiene già **1 occorrenza** dello SHA corretto `a90a6b1f27de4ea7b225f8701d89bcc912555e37b5226a92bba275d5e63bd735` e **0 occorrenze** di `4e01bfae` (verificato con `grep -c`). Il placeholder errato esisteva solo in una versione intermedia (SHA `c2334fc8…`), corretta prima della consegna e mai registrata come finale; lo SHA corretto coincide con il campo `code_sha256_self` interno al JSON diagnostico `f2098646…`.

## 6. Artefatti (path relativi alla radice del repo)
| Artefatto | SHA-256 |
|---|---|
| `Trajectory Generator/baseline_MLP/validation/v26b_bridge_2026-08-24/v26b_amendment_rev3n_s1_pregate_symmetric_5fold.json` | `0677ad577336240b26cf6947e9e3d60d74632a35724f49b5f06bf4bc5aa377e3` |
| `…/v26b_s1_pregate_rev3n.py` | `1fb8bae5fdfeff57b52ce52fb07ec09b09e55be8d473c14d329ab605de60295e` |
| `…/test_v26b_s1_pregate_rev3n.py` | `bd542faada99c223499695456c6d0f18088d6d1ad23ace33faeb5f44ad1a8a61` |
| `Trajectory Generator/runs/rollout/validation/v26b_bridge_runs/2026-08-24_V26B_anchors_r1/v26b_s1_pregate_rev3n_receipt_20260824_185328.json` | `923ffc5ca8fb4d3577d9894e90db151a92bad089d3669b38fb9553484e6f2634` |
| `…/2026-08-24_V26B_anchors_r1/s1_pregate_rev3n_20260824_185324.log` (no-clobber, `REAL_EXIT_CODE=0`) | `8e38e0818c8787c3b0d59edbf31499bdaca69af72473fc140fc1692c7802ea63` |

Selftest `test_v26b_s1_pregate_rev3n.py`: **PASS, 30 check** — pin e tamper-test di rev3n e del receipt FAIL, prova di additività, geometria dei fold (hold disgiunti che coprono esattamente le 500 righe; train 390/380/380/380/390; embargo 10/20/20/20/10 con lato singolo ai bordi), identità fold-3 ↔ finestra rev3l, bit-exactness, semantica della mediana provata anche su valori sintetici (caso in cui la mediana passa e il max no, riportando comunque il max), costruibilità del source holdout, regole post-fit, e tre guardie di fit (il token rev3n non sblocca né `run_fit` rev3n né quello rev3l).

Ripetuto per chiarezza d'audit: il selftest valida la **meccanica**; il gate è sui **dati**. Qui entrambi sono PASS, ma restano indipendenti.

## 7. Stato
**STOP per audit Codex.** `V26B-S1-FIT` non concesso e non richiesto in questa esecuzione. Invariati: seed 125 held-out e 126–128 sigillati; σ = 0,005 placeholder non deciso; niente DAgger/multistart/σ-sweep/critic; nessuna etichettatura IK automatica; un rollout nominale singolo non dichiarerà mai V3 (3 start).
