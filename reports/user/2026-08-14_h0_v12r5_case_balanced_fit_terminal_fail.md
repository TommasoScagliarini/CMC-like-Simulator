# H0 V12R5 case-balanced: qualifica sorgente e FAIL terminale del fit

Data: 2026-08-14

## Problema

La preparazione del protocollo training-ready richiede un warm-start imitativo
robusto che preservi il detector binario V26 e che possa poi essere qualificato
con Q3, prima di abilitare il Morphology Corridor nella reward. Le lineage
V12R2/V12R3/P1S/V12R4 erano gia terminali e non riutilizzabili; V12R5 e stata
quindi definita come singola prova additiva, senza nuova raccolta, retry,
fallback, sweep o rescue.

## Soluzione e strategia applicata

- Riutilizzato esclusivamente il corpus P2 e le 500 label nominali PASS di
  V12R4, per un totale di 9.232 righe e sei casi bilanciati a massa 1.000.
- Congelato un fit fresh-H0 full-mean con 3.000 epoche AdamW e polish LBFGS,
  mantenendo log-std e parte non actor bit-exact.
- Rafforzata la terminalizzazione fail-closed del runner: claim e receipt di
  stage, intent terminale, inventario preterminale, osservazioni no-throw degli
  artifact e receipt di emergenza coprono anche file regolari illeggibili e
  failure durante la pubblicazione terminale.
- Pubblicati e riverificati protocol freeze, design audit ed execution lock
  prima di consumare il claim one-shot.
- Eseguita una sola volta la pipeline V12R5, come previsto dal contratto.

## Esito della singola esecuzione

La pipeline ha prodotto correttamente un esito terminale
`FAIL_H0_V12R5_CASE_BALANCED_PIPELINE_TERMINAL` nello stage
`fit_case_balanced_candidate`. Il candidato non e stato congelato o promosso e
nessun rollout OpenSim e stato avviato.

Limiti offline ereditati:

- RMSE <= 0,006
- max absolute error <= 0,060
- reset max absolute error <= 0,003

Metriche globali V12R5:

- RMSE: 0,00785148 — FAIL
- max absolute error: 0,06888364 — FAIL
- reset max absolute error: 0,00117202 — PASS

Dettaglio rilevante:

- la finestra critica `deterministic_offset_plus_0p20` e migliorata rispetto al
  P2: RMSE 0,00485704 contro 0,00562224 e max error 0,02741654 contro
  0,03706494;
- `deterministic_offset_minus_0p20` fallisce con RMSE 0,00987832 e max error
  0,06460945;
- `stochastic_nominal_seed_127` contiene la riga peggiore, step 315, azione 1,
  con errore 0,06888364;
- i seed 126 e 128 superano marginalmente il limite RMSE; il caso nominale e il
  caso critico `+0.20` rispettano i limiti.

Il risultato indica che la pesatura case-balanced ha migliorato il bersaglio
critico positivo, ma ha degradato la fedelta globale/P2 e soprattutto il caso
negativo. V12R5 e chiusa e non verra ritentata.

## File modificati o prodotti

Sorgenti e test sotto:

- `Trajectory Generator/baseline_MLP/validation/v12r5/`

Artifact canonici principali:

- `h0_v12r5_case_balanced_protocol_freeze.json`
- `h0_v12r5_case_balanced_design_audit.json`
- `h0_v12r5_case_balanced_execution_lock.json`
- `h0_v12r5_run_20260809/pipeline_claim.json`
- `h0_v12r5_run_20260809/fit/adaptation_report.json`
- `h0_v12r5_run_20260809/fit/gate.json`
- `h0_v12r5_run_20260809/pipeline_ledger.json`

## Test e verifiche

- Suite canonica V12R5: 99/99 PASS prima della pubblicazione.
- Source gate: PASS, closure di 68 file e tutte le assenze Q2/Q3 richieste.
- Ruff check e format check: PASS.
- Harness avversariale indipendente: 6/6 PASS.
- Audit indipendente finale: GO, con SHA delle sorgenti invariati.
- Verifica protocol freeze: PASS.
- Verifica execution lock: PASS.
- Verifica canonica del ledger terminale dopo la run: PASS.
- Contabilita terminale: 3.000 epoche AdamW, 613 closure LBFGS, zero reset,
  zero step, zero query teacher, zero apertura Q2/Q3.

## Stato rispetto al training-ready

Non ancora training-ready. Il detector binario V26 resta preservato e il
runtime Q3 puo essere completato come infrastruttura, ma Q3 non puo essere
eseguita senza un candidato imitativo PASS e congelato. Di conseguenza non e
ancora possibile creare il checkpoint-zero approvato ne validare il
Morphology Corridor positivo sul candidato finale.

## TODO

- Progettare una nuova lineage V12R6, senza riaprire o ritentare V12R5, che
  corregga il trade-off introdotto dal bilanciamento uniforme dei sei casi.
- Validare offline la strategia V12R6 prima del freeze one-shot, includendo
  esplicitamente il caso `-0.20`, seed 127 e i limiti globali/P2.
- Dopo un PASS imitativo: congelare il candidato, eseguire Q3, creare e
  validare checkpoint-zero con detector V26 attivo, quindi svolgere l'A/B
  positivo del Morphology Corridor.
