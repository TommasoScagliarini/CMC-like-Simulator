# Q3 runtime source-only e blocco corretto sulla lineage R5 terminale

Data: 2026-08-14

## Problema

Il design freeze Q3 esistente descrive la qualification indipendente del
candidato V12R5, ma non disponeva ancora del runtime locale completo. Nel
frattempo la singola esecuzione R5 e terminata in FAIL al fit: Q3 deve quindi
rifiutare l'esecuzione senza aprire noise tape, protocollo, lock o run root.

## Soluzione e strategia

E stato implementato un runtime Q3 interamente additivo e source-only sotto:

`Trajectory Generator/baseline_MLP/validation/v12r5q3/runtime/`

La copertura comprende:

- binding immutabile di candidate ID e tree hash;
- invocazione dei cinque verifier R5 ufficiali;
- riuso da Q1 del solo helper `artifact_record_matches`;
- gate rollout, paired e aggregate implementati localmente;
- matrice baseline-first con sei baseline e sei candidate rollout;
- detector V26 e gate fisici obbligatori;
- Morphology Corridor fissato a peso `0.0` durante la qualification;
- noise tape, protocol freeze, execution lock e runner one-shot locali;
- contatori actor, critic e PPO fissati a zero;
- no retry, resume, rescue o sweep;
- stato successivo dopo PASS limitato a
  `WAIT_SEPARATE_ZERO_UPDATE_PROTOCOL`.

## Esito sulla R5 corrente

Il preflight live restituisce correttamente:

`R5_TERMINAL_NOT_PASS`

Prima e dopo il preflight restano assenti:

- noise tape Q3;
- protocol freeze Q3 runtime;
- execution lock Q3;
- run root Q3.

Q3 V12R5 non puo essere eseguita perche mancano
`candidate_freeze_receipt.json` e `final_development_receipt.json`; inoltre il
ledger R5 ha `passed=false`, candidate ID/module nulli e status terminale FAIL.
Il design freeze Q3 vieta la sostituzione del candidato.

## File introdotti

Nella cartella `v12r5q3/runtime/`:

- `__init__.py`;
- `h0_v12r5_q3_artifacts.py`;
- `h0_v12r5_q3_runtime_contract.py`;
- `verify_h0_v12r5_q3_prerequisites.py`;
- `h0_v12r5_q3_qualification_gates.py`;
- `prepare_h0_v12r5_q3_qualification_noise_tapes.py`;
- `freeze_h0_v12r5_q3_qualification_protocol.py`;
- `h0_v12r5_q3_physical_rollout.py`;
- `run_h0_v12r5_q3_qualification.py`;
- `test_h0_v12r5_q3_runtime.py`.

## Test e verifiche

- Test runtime Q3: 13/13 PASS.
- Ruff check e format: PASS.
- Compilazione Python: PASS.
- Source closure: PASS, 22 record.
- Preflight live fail-closed sulla R5 terminale: PASS.
- Verifica di non mutazione degli output canonici Q3: PASS.

## Stato e riuso futuro

Il runtime e valido come infrastruttura ma non puo essere retargettato in-place.
Per R6 servira una nuova lineage `v12r6q3` con design freeze, namespace,
prerequisite status/path, candidate prefix e hash propri. Il design Q3 V12R5 e
il relativo SHA restano immutati e i suoi output canonici restano chiusi.

## TODO

- Creare `v12r6q3` soltanto dopo candidato e sei development PASS R6.
- Conservare la matrice baseline-first, detector V26 obbligatorio e morphology
  a peso zero nella qualification R6.
- Non aprire gli output canonici Q3 V12R5.
