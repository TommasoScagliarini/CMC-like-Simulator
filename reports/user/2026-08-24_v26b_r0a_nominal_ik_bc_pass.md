# V26B — R0a (rev3b): BC mono-ruolo sul teacher IK nominale — PASS ed esportato (report immutabile)

**Data:** 2026-08-24 · **Emendamento:** `rev3b_r0a_order_correction` (file additivo `v26b_amendment_rev3b_r0a_order_correction.json` → SHA `52c879996ce36302474d1f7efe0da8fa86271c82418ba2efa451b20ff5fb23e7`; parent: rev3 `5e0b6a5c…`/`7ef70cbf…`, rev3a `f74672ad…`, report diagnostico `a7a740d7…`, risultati `61a09baf…`, report R0 FAIL `994ec597…` come evidenza negativa immutabile) · **Stage:** `V26B-V2-R0A` · **Esito: GATE PASS, EXPORT COMPLETATO. STOP per audit dell'architetto prima di qualsiasi rollout/DAgger.**

## 1. Contenuto dell'emendamento (correzione dell'ORDINE, non tuning)
R0 FAIL resta evidenza negativa immutabile. R0a replica il primo stadio di luglio (11/07 «Strategia 3»): **BC mono-ruolo sul teacher IK NOMINALE**, init = V1 35D (`ae846220…`). V26 resta lineage/init-source e controllo **diagnostico**: Q1 vs azioni raw V26 NON è gate di R0a (incompatibilità dimostrata dalla diagnostica; l'11/07 non esisteva un ruolo dati di preservazione — solo l'anchor-loss sui pesi). Gate: RMSE vs u_IK ≤ 0.15/giunto su TUTTE le 500 righe della griglia nominale + Q3; logstd congelata, critic assente, full mean network, budget congelato 300 ep/lr 1e-4/batch 256/seed 2026 e regolarizzazioni identiche; lo stato diagnostico task-only NON è stato riusato (fit produttivo rieseguito). **Terminologia**: `continuity_anchor`/`target_anchor` del report 11/07 sono anchor del **slew limiter runtime** (fix del reference governor in `osim_trj_cmc_like.py`), NON ruoli del dataset — mai usati per giustificare la costruzione dati. Lo stadio successivo July-faithful è PREPARATO SOLO NEL TESTO dell'emendamento (rollout nominale → DAgger same-state con troncamento al primo mismatch → solo dopo, preservazione source==init + recovery + teacher alt-start), con prerequisito: identificare dal codice storico come furono costruiti i 16 000 anchor nominali del 13/07 (nessuna supposizione, nessun riuso automatico delle label raw V26).

## 2. Tooling produttivo e test (additivi; `v26b_v2.py` e artefatti congelati INTOCCATI)
`v26b_r0a.py` → `825614b05f9dc89f…`, `test_v26b_r0a.py` → `2e6a6c84c3f7b26d…` — **SELFTEST 42/42**: lineage rev3b (3 pin manomessi → rifiuto), **digest FULL 64-hex delle 3 cache IK fail-closed nel percorso produttivo** (pin manomesso → rifiuto anche nel dataset build), dataset (500 righe, 0 duplicati, label bitwise == cache, split deterministico con digest indici, citazioni esatte), determinismo del fit, logica gate (pass/fail/refusal), guardia token, export no-clobber/save-reload/receipt, immutabilità sorgenti.

## 3. Dataset R0a (500 righe, dedup e provenienza esatta)
obs35 = le 500 righe della **trace det nominal pinnata** (`3a66d109…`), bitwise-uniche (0 duplicati); label = **u_IK(t_pre)** dalla cache nominale pinnata FULL (`3dd878d4d6d2930d…`), lookup esatto sulla griglia; **niente alt-start, niente label raw-V26**. Deviazione da luglio documentata: le 500 righe dell'11/07 erano stati visitati DAL teacher nel suo episodio closed-loop (`collect_teacher_dataset`, `target_domain_imitation.py:328`); R0a usa gli stati visitati dal V26 det sulla stessa griglia (nessun env stepping autorizzato) = relabelling same-state. Split storico citato (`target_domain_imitation.py:794-918`: `rng.permutation` + `validation_fraction`, 11/07 400/100 con early stopping, best epoch 368; modalità storica `fixed_final_epoch` con `validation_fraction=0, patience=0`): rev3b addestra col budget congelato 300 epoche (= modalità fixed-final-epoch) e riporta lo split 400/100 deterministico (digest indici `ce037f1b84ef741b…`) **solo come diagnostica**, mai gate, nessuna augmentation.

## 4. Fit e gate
| metrica | knee | ankle | soglia | esito |
|---|---|---|---|---|
| **RMSE vs u_IK (tutte le 500)** | **0.0716** | **0.0501** | ≤ 0.15 | **PASS** |
| holdout 100 (diagnostica) | 0.0725 | 0.0514 | — | ≈ train (0.0713/0.0497): nessun overfitting |
| diagnostica vs mean V26 | 0.4808 | 0.3612 | — | registrata, NON criterio (≈ gap comportamentale atteso) |
| Q3 invarianti (10 chiavi, clock zero+invarianza bit-identica, logstd costante, save/reload) | — | — | — | **PASS** |

Loss 5.786 → 0.206 (e150) → **0.0643** (e300); varianze fissate a priori [0.1195, 0.0434]; shift parametri ‖θ−θ_V1‖² = 4.24 (R0 bi-ruolo: 15.4). Nota onesta: l'holdout misura interpolazione sulla stessa traiettoria nominale; la generalizzazione vera la testeranno rollout/DAgger dopo il tuo audit.

## 5. Export (transazionale, no-clobber)
`…/student/V2_R0a/rl_module/` — promozione **`darwin_renameatx_np_RENAME_EXCL` single-step atomica**; **actor digest `8567071e1185d6719290255cb1cb2f062d235a929d39b5b4767d1b613ee38959`**, `module_state.pkl` `7f1ba2ed…`; receipt canonico `v26b_r0a_receipt.json` → **SHA `ff92d2fc8e730d4a9ad61c3b0f4be64741b3988196d37040cb1841260e278040`** (pinna: parent rev3, rev3a, rev3b, evidenza negativa R0+diagnostica, digest FULL delle cache, V1/V26 con tabelle immutabilità pre/post, dataset, budget, history completa, gate, digest del codice). Log no-clobber `r0a_fit_*.log`, REAL_EXIT_CODE=0.

## 6. Stato
rev3/rev3a/V1/V26/report congelati: intoccati e ri-verificati. Nessun rollout, DAgger, V3 o PPO eseguito. **STOP: attendo il tuo audit del receipt/report prima dello stadio successivo.**
