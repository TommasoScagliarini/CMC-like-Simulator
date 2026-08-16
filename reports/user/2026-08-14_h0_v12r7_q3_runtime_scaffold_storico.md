# H0 V12R7-Q3: runtime completo, storico e fail-closed

Data: 2026-08-14

## Problema

La qualification indipendente Q3 richiedeva un runtime completo per verificare
il candidato warm-start con detector binario V26 attivo e Morphology Corridor
causale attivo a peso zero. Durante l'implementazione, V12R7 ha chiuso con
esito terminale `FAIL`: la lineage Q3 associata non puo quindi congelare
protocollo, materializzare noise tape o avviare rollout.

## Soluzione

E stato completato lo scaffold runtime source-only in
`Trajectory Generator/baseline_MLP/validation/v12r7q3/runtime/`, mantenendo il
namespace V12R7-Q3 storico e rendendolo esplicitamente non pubblicabile.

Il runtime implementa e testa:

- invocazione e cross-binding dei cinque verifier semantici della lineage;
- protocol freeze ed execution lock esclusivi e content-addressed;
- cinque noise tape deterministici, con ABI `float32`, shape `(500, 2)` e hash
  array preregistrati;
- adattatore al collector fisico maturo V12R5-Q3, senza importare
  RLlib/OpenSim durante l'import del modulo;
- configurazione candidate con detector V26 e Morphology Corridor
  `event_anchored_causal_delayed_experimental` attivi;
- verifica A/B a peso zero con identita byte di reward, azioni e osservazioni,
  5.000 campioni detector e campioni corridor finiti;
- dodici rollout in ordine baseline-first, sei gate condition-matched e un
  tredicesimo stadio aggregate;
- journal per-step e persistenza di trace/summary prima della valutazione dei
  gate;
- ledger terminale one-shot, senza retry, resume, rescue, sweep, update,
  checkpoint-zero, morphology positiva o promozione runtime;
- receipt finale e ledger con candidate identity, contatori update a zero,
  `morphology_weight=0.0` e handoff
  `WAIT_SEPARATE_CHECKPOINT_ZERO_PROTOCOL`.

## Chiusura fail-closed della lineage R7

Il contract espone:

- `LINEAGE_STATE = HISTORICAL_R7_TERMINAL_FAIL_NO_Q3_PUBLICATION`;
- `HISTORICAL_TERMINAL_FAILURE = True`;
- `SUCCESSOR_NAMESPACE = v12r8q3`.

Le entry point canoniche di freeze, tape preparation e live execution rifiutano
sempre l'operazione prima di qualsiasi scrittura. Le funzioni pure e la
materializzazione verso directory temporanee restano testabili.

Non sono stati creati file JSON/NPZ, lock, claim, run root o receipt canonici in
`v12r7q3`.

## Strategia per il successore

Il runtime non deve essere ribindato in-place. Dopo che V12R8 esporra i propri
endpoint terminal-PASS, andra creato il nuovo namespace `v12r8q3` aggiornando
insieme:

1. import del contract V12R8 e cinque hook verifier ufficiali;
2. path e status dei cinque prerequisite receipt;
3. candidate module path, selection rule, ID prefix e exact tree;
4. schema, protocol ID, pipeline ID, root, noise root, run root e ledger;
5. status rollout/pair/aggregate/terminale e nomi CLI/moduli;
6. source closure e test negativi cross-lineage.

Il protocollo Q3 V12R8 dovra essere congelato soltanto dopo cinque verifier
semantici V12R8 live tutti PASS sullo stesso exact candidate tree.

## File modificati o introdotti

- `Trajectory Generator/baseline_MLP/validation/v12r7q3/h0_v12r7_q3_qualification_contract.py`
- `Trajectory Generator/baseline_MLP/validation/v12r7q3/h0_v12r7_q3_qualification_gates.py`
- `Trajectory Generator/baseline_MLP/validation/v12r7q3/runtime/__init__.py`
- `Trajectory Generator/baseline_MLP/validation/v12r7q3/runtime/freeze_h0_v12r7_q3_qualification_protocol.py`
- `Trajectory Generator/baseline_MLP/validation/v12r7q3/runtime/prepare_h0_v12r7_q3_noise_tapes.py`
- `Trajectory Generator/baseline_MLP/validation/v12r7q3/runtime/h0_v12r7_q3_physical_rollout.py`
- `Trajectory Generator/baseline_MLP/validation/v12r7q3/runtime/run_h0_v12r7_q3_qualification.py`
- `Trajectory Generator/baseline_MLP/validation/v12r7q3/runtime/test_h0_v12r7_q3_runtime.py`
- `Trajectory Generator/baseline_MLP/validation/v12r7q3/test_h0_v12r7_q3_source_scaffold.py`

## Test e verifiche

- Q3 source scaffold + runtime: `43 passed`.
- Test congiunti V12R7 + V12R7-Q3: `75 passed`.
- Ruff check: PASS.
- Ruff format check: PASS.
- `py_compile`: PASS.
- `git diff --check`: PASS.
- Source closure reale, inclusi input e dipendenze riusate: PASS.
- Import pulito senza caricare `ray`, `torch` o `opensim`: PASS.
- Assenza di artefatti canonici JSON/NPZ in V12R7-Q3: verificata.

## TODO

- Attendere gli endpoint immutabili V12R8 e creare `v12r8q3` come nuova
  lineage; non clonare o congelare parzialmente prima del terminal PASS R8.
- Eseguire freeze, tape preparation e qualification fisica soltanto nel nuovo
  namespace V12R8-Q3.
- Dopo Q3 PASS, creare un protocollo checkpoint-zero separato; non incorporare
  checkpoint o update nella qualification Q3.
- Abilitare una reward morphology positiva soltanto con un protocollo A/B
  successivo, separato e autorizzato.
