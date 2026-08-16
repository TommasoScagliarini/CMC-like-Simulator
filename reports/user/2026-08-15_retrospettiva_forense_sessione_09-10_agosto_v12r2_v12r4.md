# Retrospettiva forense — sessione notturna 09–10/08: V12R2 → V12R4

Data: 2026-08-15

> **Nota di metodo.** Questo report è stato scritto retroattivamente il
> 2026-08-15, ricostruendo la sessione notturna del 09–10 agosto da ledger,
> receipt, docstring e mtime dei file. Nessun artefatto storico è stato
> modificato: l'audit è stato interamente read-only. La sessione non aveva
> prodotto alcun report utente né daily; questo documento colma quel buco.

## Contesto

Il report [2026-08-09_h0_v12r2_training_readiness](2026-08-09_h0_v12r2_training_readiness.md)
chiudeva alle ~22:00 del 09/08 dichiarando V12R2 «execution-ready ma non
eseguita». In realtà la one-shot è stata lanciata la sera stessa e la sessione
è proseguita fino alle 05:30 del 10/08, bruciando in cascata quattro lineage
esecutive (V12R2, V12R3, salvage P1, V12R4) e producendo tre design freeze
satellite. Nessun documento la registrava. I giorni 11–13/08 sono assenza
totale di attività (zero file modificati), non attività persa.

## Cronologia della sessione

| Orario | Attività | Esito |
|---|---|---|
| 09/08 20:06–22:28 | build + esecuzione one-shot V12R2 | FAIL terminale (bug integrità forense) |
| 09/08 22:51–10/08 00:30 | build + esecuzione one-shot V12R3 | FAIL terminale al fit_p2 (primo pure-probe P1 PASS della storia del progetto) |
| 10/08 01:07–02:05 | design qualification P1 (`v12p1q`) | design freeze PASS, mai eseguita |
| 10/08 01:31–02:00 | salvage P1 (`v12p1s`) | FAIL terminale al caso 2/6 (rifiuto FSM) |
| 10/08 02:31–03:00 | design Q2 (`v12r4q2`) + scaffold zero-update (`v12r4zero`) | design freeze PASS / scaffold inerte |
| 10/08 02:33–03:14 | build + esecuzione one-shot V12R4 | FAIL terminale (penetrazione a step 212) |
| 10/08 03:32–03:46 | design freeze Q3 (`v12r5q3`) | PASS, deferred |
| 10/08 04:00–05:30 | bozza contract + fitter V12R5 | solo sorgenti; eseguita il 14/08 |

## V12R2 — esecuzione e FAIL per bug di integrità forense

La one-shot (lock ~20:35, ultimo artefatto 22:28) ha eseguito:

- `fit_p0`: PASS (`PASS_H0_PRIMARY_SPLIT_V12R2_RECOVERY_WEIGHTED_FIT`);
- `probe_p0`: fermato fisicamente a step 232, poi chiuso **prima** della
  valutazione del gate con `V12R2ExecutionError: prospective rollout record
  drifted: run_start`.

Ledger: `FAIL_H0_PRIMARY_SPLIT_V12R2_PIPELINE_TERMINAL`, `passed=false`,
`attempted_stage=probe_p0`, `environment_step_calls=232`;
`failure.json`: `FAIL_H0_PRIMARY_SPLIT_V12R2_PROBE_INTEGRITY`,
`end_reason=v12r2_stage_failed_terminally`, `last_completed_step=232`.

Causa radice (documentata nel docstring del contract V12R3): bug della
pubblicazione forense in `run_h0_primary_split_v12r2_autonomy_recovery.py:1313`
— il runner confronta il record `run_start` prima/dopo `finalize_before_gate`,
ma il writer condiviso non restituisce quel record. Inoltre il gate V12R2 non
giudicava l'integrità del prefisso di un probe fermato fisicamente. Il failure
è quindi **spurio** (classe verificatore, non fisica): stessa classe dei
successivi fail R7/R8.

## V12R3 — dieci stage completati e primo pure-probe PASS

Protocollo `AB06_H0_PRIMARY_SPLIT_V12R3_V26_AUTONOMY_RECOVERY` (schema 123),
successore che chiude i due gap esecutivi di V12R2 (record `run_start` non
confrontabile; prefisso troncato giudicato sull'integrità detector alla
lunghezza osservata, indipendente dall'event gate full-horizon).

Stage completati (10):

1. `fit_p0` PASS;
2. `probe_p0`: fermato a 232 step per `grf_penetration` (max `0,02517 m`),
   autonomy FAIL ma `recoverable_for_data_collection=true`;
3. `label_p0`: 232 label observer;
4. `collect_r1` × 2 casi: 500 step ciascuno, PASS, teacher takeover
   `52,4%` / `55,4%`;
5. `fit_p1` PASS (RMSE `0,005272`, max `0,056857`, reset `0,000759` — vicini
   ai limiti 0,006 / 0,060 / 0,003);
6. **`probe_p1`: primo pure-probe PASS dell'intero progetto** — 500/500 step,
   `end_reason=episode_time_limit`, 2 cicli validi, penetrazione max
   `0,02356 m`, `PASS_H0_PRIMARY_SPLIT_V12R3_PURE_PROBE`;
7. `label_p1`: 500 label;
8. `collect_r2` × 2 casi: 500 step, PASS.

Terminale: `fit_p2` FAIL con `V12R3RecoveryWeightedFitError: p2 fit summary
failed contract gate: ['offline_metrics']` →
`FAIL_H0_PRIMARY_SPLIT_V12R3_PIPELINE_TERMINAL`.

In `fit/p2` restano `corpus.npz` (18,4 MB, SHA-256 `42a40869…`),
adaptation report/history e modulo, ma **nessun summary/gate/receipt**: il
raise precede la scrittura, quindi **non è persistito quale delle tre metriche
offline abbia superato la soglia né di quanto**. È l'unico fail della sessione
senza metriche su disco.

## v12p1q — design qualification P1, mai eseguita

Protocollo `AB06_H0_V12R3_P1_V26_INDEPENDENT_QUALIFICATION` (sei casi
appaiati). Unico artefatto: `h0_v12r3_p1_qualification_design_freeze.json`,
`PASS_H0_V12R3_P1_QUALIFICATION_DESIGN_FREEZE`, next stage
`WAIT_V12P1S_SIX_OF_SIX_THEN_FREEZE_QUALIFICATION_PROTOCOL`. La qualification
non si è mai aperta perché il salvage P1 è fallito.

## v12p1s — salvage P1, FAIL per rifiuto FSM

Sei rollout development puri del candidato P1
(`AB06_H0_V12R3_P1_SALVAGE_V26_DEVELOPMENT`), senza riaprire V12R3:

- caso 1/6 `deterministic_offset_minus_0p20`: PASS, 500 step,
  `episode_time_limit`;
- caso 2/6 `deterministic_offset_nominal`: **FAIL terminale a step 179** —
  `ValueError: Actor FSM rejected a V20 active event:
  invalid_event_type=to_too_early_after_hs, state_name=STANCE_AFTER_HS`
  (toe-off V26 troppo precoce dopo l'heel strike).

Ledger: `FAIL_H0_V12R3_P1_SALVAGE_DEVELOPMENT_TERMINAL`,
`end_reason=v12r3_p1_salvage_rollout_failed_terminal_no_retry`; eseguiti ~679
dei 3.000 step attesi. Il TO precoce in stance è lo stesso pattern fisico che
riapparirà in R6/R10/R11.

## V12R4 — coverage P3, FAIL per penetrazione nella raccolta shielded

Protocollo `AB06_H0_PRIMARY_SPLIT_V12R4_P3_COVERAGE_V26`
(`H0_V12R4_P3_FOUR_CASE_COVERAGE_SAFE_DAGGER`): usa il modulo P2 non
receiptato **solo** come studente non promovibile in 4 episodi shielded, poi
un unico fit P3 e sei development.

- stage 1 `attest_p2_collection_source`: PASS — attesta
  `v12r3/…/fit/p2/corpus.npz` (SHA-256 `42a40869…`, 18.358.010 byte);
- stage 2 collect nominale: PASS — 500 step full-episode, 500 label
  same-state, gate `PASS_H0_V12R4_SHIELDED_COLLECTION_DATA`;
- stage 3 collect `deterministic_offset_plus_0p20`: raccolta shielded
  terminata per `grf_penetration` a **step 212** (max `0,025064 m`, 0 cicli
  validi, 212 label) → collection gate FAIL →
  `FAIL_H0_V12R4_P3_COVERAGE_TERMINAL`, `next_stage=STOP_TERMINAL`.

`assemble_corpus_p3` e `fit_p3` non sono mai stati raggiunti.

**Precisazione storica**: nei report successivi (es. V12R5) il corpus P2 è
attribuito a V12R4. In realtà il corpus P2 è stato **materializzato da V12R3**
durante il `fit_p2` poi bocciato; V12R4 lo ha soltanto attestato come sorgente.
Le 500 label nominali PASS e lo stop shielded a step 212 sono invece
correttamente di V12R4. Il prefisso `+0.20` da 212 label di V12R4 è la fonte
dello «strato R4» usato da R7/R8/R9/R10.

## v12r4q2 e v12r4zero — satellite mai avanzati

- `v12r4q2`: design freeze Q2 a candidato differito
  (`AB06_H0_V12R4_Q2_V26_INDEPENDENT_QUALIFICATION_DESIGN`), PASS alle 02:46
  con `next_stage=WAIT_R4_TERMINAL_PASS_THEN_FREEZE_Q2_PROTOCOL`; R4 è fallita
  terminalmente 14 minuti dopo, quindi mai avanzata.
- `v12r4zero`: scaffold inerte (4 file `.py`, zero JSON) per il port
  zero-update RLlib del futuro P3; mai eseguito.

## Coda della sessione

Dopo il fail di V12R4 la sessione ha ancora prodotto il design freeze Q3
`v12r5q3` (03:50, `PRE_R5_DEFERRED_CANDIDATE_INDEPENDENT_Q3_DESIGN`) e la
bozza di contract + fitter V12R5 (04:00–05:30). L'esecuzione V12R5 è avvenuta
solo il 14/08. Nota: il run root V12R5 si chiama `h0_v12r5_run_20260809`
nonostante l'esecuzione del 14/08 — il suffisso è ancorato alla data
dell'evidenza sorgente, non all'esecuzione.

## File coinvolti (tutti sotto `Trajectory Generator/baseline_MLP/validation/`)

- `v12r2/` incluso run root `h0_v12r2_run_20260809/` (ledger + failure.json);
- `v12r3/` incluso `h0_v12r3_run_20260809/` (10 stage receipt + fit/p2
  parziale);
- `v12p1q/` (design freeze), `v12p1s/` (run root salvage);
- `v12r4/` incluso `h0_v12r4_run_20260809/`, `v12r4q2/` (design freeze),
  `v12r4zero/` (scaffold);
- `v12r5q3/h0_v12r5_q3_qualification_design_freeze.json`;
- `validation/test_morphology_corridor_v26_readiness.py` (root, 02:41 —
  collaterale della stessa sessione).

## Test e verifiche (dell'audit retroattivo)

- lettura read-only di ledger, receipt, gate e failure.json di tutti i
  namespace elencati: stati coerenti fra loro e con i docstring dei
  successori;
- cross-check del corpus P2: SHA-256 `42a40869…` identico fra `fit/p2` di
  V12R3, l'attestazione di V12R4 e l'attestazione nel freezer V12R5;
- timeline mtime ricostruita e coerente con l'ordine degli stage nei ledger;
- nessun file storico modificato.

## TODO

- [ ] Rieseguire (opzionale, read-only) l'eval offline sul `corpus.npz` +
  modulo salvati in `v12r3/…/fit/p2/` per stabilire quale metrica bocciò il
  `fit_p2` e di quanto: unico esito della sessione senza numeri persistiti.
- [x] Debito documentale della sessione 09–10/08: colmato da questo report e
  dal daily retroattivo `2026-08-10_daily-report.md`.
