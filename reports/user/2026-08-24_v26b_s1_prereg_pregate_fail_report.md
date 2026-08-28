# V26B S1 (IK-AB06) — Prereg + pre-gate read-only: esito FAIL marginale (knee), STOP fail-closed

**Token:** V26B-S1-PREREG-READONLY · **Data:** 2026-08-24 · **Stadio:** prereg/tooling/pre-gate SOLO — nessun fit/export/rollout (V26B-S1-FIT mai concesso). Esecuzione unica del pre-gate, nessun retry.

## 1. Esito in una riga
Il pre-gate vincolante di identificabilità (kNN k=5, RMSE vs `u_IK` su holdout bloccato, soglia riusata rev3b ≤ 0,15/giunto) è **FAIL sul knee per +0,0022**: knee **0,15221** (> 0,15), ankle **0,10627** (PASS). `pregate_pass=false`, exit code 3. Fail-closed: nessun fit S1 può partire; artefatti preservati; decisione all'architetto.

## 2. Catena preregistrata completata prima del gate
- **rev3l** (prereg S1, immutabile): `7545201df4e5aefb4fdccca9efc1425a9a4dde581a4c20269c0d3fd86aa794d5` — include ricostruzione July byte-citata, dichiarazione di non-provabilità byte del 16k July con sostituto minimo, provenienza cache AB06 (`3dd878d4…`), gap strutturale congelato.
- **rev3m** (correzione architetto, ADDITIVA, rev3l intatto): `8af3ad654e49e19f5a6d2dbcf128faf5ce424206a5def715d177f7fbda93e0c0` — sostituisce SOLO la coppia di gate offline incompatibile (triangle inequality con gap ~0,38): preservazione = (a) parameter anchor July 1e-5, (b) T1/T2 solo equivalenza scaling/export, (c) Q1 ≤ 0,10/giunto su **source holdout separato** (righe provenance-held rev3k, etichette = S0D deterministic mean same-state); **nessun gate di drift vs S0D sulle 500 righe IK** (solo diagnostica attesa); gate di task invariato (RMSE ≤ 0,15 vs `u_IK`).
- **Supplemento semantica B3**: `9e57271a0eea28a782927cda2399b812a6ddedb8811808eea90a637a61b9318b` (not_evaluable = campo phase tutto zero NONOSTANTE 2 cicli validi; mai min globale).
- **Tooling/test**: `v26b_s1_prereg.py` `858037282a7928e79b664419f7d47302dfd7703beb68db5e95f74d1f32ac25a0`, `test_v26b_s1_prereg.py` `1074e70f8003501ea68acebf0fed0bb23d9b237ea831d92568e815edba4fa40c` — SELFTEST PASS (14 check), incluso: lineage con pin rev3l+rev3m, guardia `run_fit` (token assente ⇒ raise), disgiunzione bitwise source/task, feature costanti gestite per elenco effettivo.

## 3. Esecuzione del pre-gate (unica)
- **Receipt** (immutabile): `…/2026-08-24_V26B_anchors_r1/v26b_s1_pregate_receipt_20260824_173741.json` → sha256 `a437f1ef0d7762db5914d14be19b390ddc6ada9a126d9db2ef8f810d08a43389`.
- **Vista S1**: 500 righe del trace nominale S0D (receipt rollout `cbec1a67…`) + `u_IK` AB06 (cache `3dd878d4…`) + `u_own` S0D; split bloccato 380 train / 100 hold (step 201–300) / 20 embargo; standardizzazione solo-train.
- **Binding**: kNN k=5 RMSE vs `u_IK` su hold = **[0,15221 knee, 0,10627 ankle]**, soglia 0,15/giunto → **FAIL** (knee, margine +0,0022 ≈ 1,5 % sopra soglia).
- **Verifica costruibilità source holdout rev3m** (parte del receipt): 4 480 righe provenance-held rev3k; **6 righe bitwise condivise** con le 500 di task, escluse e conteggiate → **4 474 usabili**; `constructible_without_leakage=true`; nessun conflitto di label per costruzione (insiemi disgiunti, funzioni di label diverse). Il fallback dichiarato in rev3m non è necessario.
- **Feature escluse (costanti sul train)**: indici **[0, 1, 17]** = clock azzerato per costruzione (0,1) + `phase_fsm_wait_hs` (17), costante 0 sul trace S0D: la FSM non entra mai in WAIT_HS (parte in STANCE_AFTER_HS e alterna STANCE/SWING) — fatto del dato, non bug; registrato per elenco effettivo, con {0,1} ⊆ esclusi asserito nel test.

## 4. Diagnostica (non vincolante, per l'audit)
- **Strumento al limite di risoluzione su questa finestra**: lo stesso kNN k=5, stessa finestra di holdout, contro le **etichette proprie** di S0D (che il rollout 500/500 dimostra essere una funzione realizzabile e liscia dello stato) dà **[0,1474 knee, 0,1717 ankle]** — l'ankle "fallirebbe" la stessa soglia contro la propria policy. La finestra 201–300 contiene quindi vicinati di stato con azioni variabili per qualunque etichetta; il FAIL knee di +0,0022 va letto con questo contesto. Riportato come fatto; nessuna proposta di rilassamento.
- **Gap strutturale confermato sui 500 stati**: |u_S0D − u_IK| knee mean 0,3758 (p90 0,916, max 1,432), ankle mean 0,3014 (max 1,198) — identico ai valori congelati in rev3l; su queste righe il drift vs S0D del futuro fit resta **solo diagnostica attesa** (rev3m).
- **Contesto obbligatorio**: JUL_H0 finale July offline aggregate 0,008144 e 500/500 (guardie July 15/25 mm); S0D fit holdout [0,0811, 0,0793] e 500/500 (guardie v3). Questi numeri non sono confrontabili direttamente col kNN pre-gate (strumenti diversi); sono il contesto di catena richiesto dall'ordine.
- **Log** no-clobber: `s1_pregate_20260824_173741.log` (sha sotto), REAL_EXIT_CODE=3.

## 5. Stato e opzioni (decisione all'architetto; nessuna azione intrapresa)
FAIL vincolante ⇒ STOP. Opzioni possibili che SOLO l'architetto può autorizzare: (i) accettare il FAIL e chiudere S1; (ii) emendamento additivo che ridefinisca lo strumento di pre-gate (es. finestra/blocchi diversi o predittore di riferimento diverso) con giustificazione a priori — mai un rilassamento della soglia a posteriori; (iii) altra via. Nessun retry è stato eseguito; il receipt FAIL e il log sono immutabili.

**Vincoli invariati**: init futuro esclusivamente S0D (`481dd0d2…`); niente DAgger/multistart/σ-sweep/critic; σ=0,005 placeholder non deciso; seed 125 held-out, 126–128 sigillati; rollout nominale singolo ≠ V3 (3 start non valutati).
