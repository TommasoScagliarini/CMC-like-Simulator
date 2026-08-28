# V26B — Rollout nominale deterministico R0a (rev3c): TERMINATO PRESTO — diagnosi (report immutabile)

**Data:** 2026-08-24 · **Emendamento:** `rev3c_r0a_nominal_rollout` (additivo, `v26b_amendment_rev3c_r0a_nominal_rollout.json` → SHA `723c8981263fc74d2c6e66033619c6a4d44aa4392c607a524934dbc7e403df23`; parent: catena rev3/rev3a/rev3b + receipt R0a `ff92d2fc…` + report R0a `6b4dfac4…` + modulo `8567071e…`) · **Esito: rollout 493/500, `joint_divergence:pros_knee_angle` → STOP fail-closed, NESSUN manifest DAgger, nessun retry.**

## 1. Archeologia luglio (workflow a 3 reader paralleli; citazioni complete in rev3c)
(a) **Harness**: i rollout nominali di luglio erano `rollout_eval.py` CLI (watchdog), seed 123, deterministico = mean gaussiana senza sigma; offset 1.956870983805102 auto-caricato dallo yaml sibling; gate SOLO report-level sull'attore (full episode, ≥1 ciclo, pen < 25 mm, clipping, return), gate programmatico solo sul teacher (`gate_pass`, `target_domain_dagger.py:85-87`).
(b) **DAgger**: etichettatura **same-step su griglia fixed-start** (`teacher_actions[step-1]`, `target_domain_imitation.py:640-676`), NON same-state; l'11/07 NESSUN troncamento; il troncamento nasce il 13/07 per le trace STOCASTICHE: `truncate_before_discrete_mismatch` (`target_domain_noise_adaptation.py:75-101`) confronta le componenti DISCRETE dell'osservazione (nomi `*_in_contact/_heel_strike/_toe_off/_saturated`, `phase_fsm_*`, `phase_expected_*`) contro la **trace nominale det DELLO STESSO attore**, per indice di step; 500→119/118/119, primo mismatch 120/119/120, 76.27% fuori fase.
(c) **16000 anchor 13/07** = UN episodio det da 500 step × `np.tile` **32 copie identiche** (`--nominal-repeat 32`, `markov_dataset_report.json`); label = `raw_policy_action` registrate (= mean det) dell'attore sorgente; **source == init provato** (stesso `--checkpoint` per anchor e init, `target_domain_markov_adaptation.py:420-426`; catena digest `42837d48…`→`a0801a9e…`; identità float32 rmse 1.19e-07); NESSUN dedup, split 20% permutato su righe TILED (copie identiche sui due lati), e il gate offline del run scelto era `ok: false` (selezione via gate closed-loop) — pratiche registrate come fatto storico, vietate dal protocollo rev3.

## 2. Tooling ed esecuzione (SELFTEST 38/38, incl. la correzione ordinata)
`v26b_r0a_rollout.py` `06eebb28efb3bd7e…`, `test_v26b_r0a_rollout.py` `ece259f8c831c803…`. Comando esatto rev3c (rollout_eval pinnato `5433bcbc…`, config v3 `a870cc38…` esplicita, det, seed 123, start esatto, no-clobber, una sola esecuzione). **Correzione audit implementata**: `morphology_causal_contract_failure` da `reward_terms.morphology_causal_failed_closed` per riga (mai da `end_reason`); `phase_timeout_stance/swing` contati su TUTTE le righe da `phase_timeout_exceeded`+`phase_timeout_side` (1=stance, 2=swing); gating del manifest SOLO su 500/500 (contatori mai bloccanti — testato).

## 3. Esito del rollout (una sola esecuzione, driver rc=0, stage rc=3 = ENDED_EARLY)
| fatto | valore |
|---|---|
| step / end_reason | **493/500, `joint_divergence:pros_knee_angle`** |
| receipt preservato | `v26b_r0a_rollout_receipt.json` → `ae04bcda13e5cc45fc9199f644920c399db1de4fd542adba19e614e5e0933a14` |
| **addendum audit (metriche corrette)** | `v26b_r0a_rollout_receipt_audit_addendum.json` → **`0884a25be8c919e89992d75cee81bdb4a5996f57a5194b19046dae32ac7da41b`** |
| contatori corretti (per-riga) | causal failure **0** righe (max 0), timeout stance **0**, swing **0** |
| FSM | invalid 0, resync 0, hs_cancelled 0; HS/TO validi 3/3, **2 cicli validi**; stato finale STANCE_AFTER_HS |
| primo mismatch discreto vs anchor V26 | **step 12** (prefisso label-valido: 11 righe) |
| knee q | range [−0.339, 0.0], RMSE vs prescritto 0.365 — ROM ridotto, deriva verso estensione piena |
| ankle q | range [+0.029, +0.409], RMSE vs prescritto 0.130 |
| penetrazione | max 27.3 mm (< hard 28, > soft 20), alla fine 12–15 mm |
| clipping / return | **33 step clipped** (V26 anchor: 12); return +0.53 |

## 4. Diagnosi
La terminazione è una **divergenza del ginocchio verso l'estensione piena**: negli ultimi ~14 step il comando knee grezzo satura a +1.0…+1.19 (clippato), spingendo il ginocchio da −0.08 a −0.002 rad (bound 0) finché la guardia `joint_divergence` interviene allo step 493. Non è un fallimento FSM/morphology/timeout (tutti zero, anche con le metriche corrette) né penetrazione. **Causa strutturale: covariate shift del BC mono-ruolo** — R0a è addestrato SOLO sugli stati visitati dal V26 det; in closed loop la sua sequenza discreta diverge dal riferimento V26 **già allo step 12**, e da lì cammina su stati fuori copertura con un'andatura propria (2 cicli validi, contatori puliti, fase self-consistent ma shiftata) fino alla saturazione del ginocchio nel terzo ciclo. È il pattern testuale dell'11/07 (il clone BC completò 68/500; il DAgger nacque per questo): R0a fa molto meglio (493/500, 2 cicli) ma fallisce per lo stesso meccanismo.

## 5. Fatti per la tua prossima decisione (nessuna azione intrapresa)
1. Il mandato rev3c gating il manifest su 500/500: **manifest NON scritto** (verificato: `dagger_plan_manifest: null`).
2. Il troncamento vs il riferimento anchor V26 lascerebbe **11 righe** label-valide — conferma empirica che quel riferimento (dichiarato in rev3c come proxy, audit-gated) non è utilizzabile per il DAgger di questo attore. Storia: l'11/07 il DAgger NON troncava i rollout det (label same-step su tutto il prefisso; a luglio il round 1 aggregò il rollout BC da 68 step COSÌ); il troncamento 13/07 usava la nominale det **dello stesso attore** come riferimento — che per R0a non esiste finché non completa.
3. Opzioni possibili (tue): DAgger round-1 July-faithful **senza troncamento** sul rollout det (aggregazione same-step 493 righe col dataset BC, come l'11/07); oppure ridefinire il riferimento di troncamento; oppure altra correzione. Ogni scelta = emendamento additivo tuo.

## 6. Stato
Artefatti preservati (job dir, receipt, addendum, log driver `r0a_rollout_driver_*.log` REAL_EXIT_CODE=3). rev3/rev3a/rev3b/rev3c/V1/R0a/V26 immutabili. Nessun DAgger, refit, retry o ulteriore rollout. **STOP per il tuo audit.**
