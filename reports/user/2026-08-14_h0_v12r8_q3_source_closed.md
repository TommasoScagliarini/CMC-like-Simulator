# H0 V12R8-Q3 — protocollo canonico source-closed

Data: 2026-08-14

## Stato della milestone

Il protocollo canonico `V12R8-Q3` è **source-ready e fail-closed**. Tutti i
sorgenti necessari alla qualifica indipendente sono presenti, importabili e
inclusi nella source closure dinamica.

Questa milestone **non** dichiara ancora il candidato training-ready: non sono
stati pubblicati il freeze Q3, l'execution lock, i noise tape o risultati di
rollout. L'esecuzione Q3 resta bloccata finché i cinque artefatti R8 non esistono
e i rispettivi verifier live non dimostrano un ledger R8 terminale `PASS`.

## Problema

Lo scaffold Q3 storico era legato a V12R7, ma V12R7 è terminato in modo
irreversibile con un ledger `FAIL`. Non era quindi corretto riusarne namespace,
candidate ID, receipt o autorizzazioni per qualificare il detector binario V26
e il morphology corridor.

Inoltre, durante l'audit sono emersi tre punti da chiudere:

- lo stato deferred descriveva erroneamente come “mancanti” i cinque runtime
  Q3 già presenti;
- il builder del summary poteva inserire contatori zero assenti, producendo una
  falsa evidenza invece di fallire chiuso;
- il risultato restituito da un verifier R8 non era ancora legato byte-per-byte
  al file JSON canonico del quale veniva registrato l'hash.

## Soluzione

È stato creato un namespace canonico autonomo:

`Trajectory Generator/baseline_MLP/validation/v12r8q3/`

Il protocollo usa esclusivamente identità, path, status e candidate prefix R8.
Ogni riferimento V12R7 nei sorgenti operativi Q3 è escluso; l'unico controllo
R7 residuo è un test negativo che ne verifica il rifiuto.

La qualifica preregistra:

- cinque verifier semantici live R8: protocol freeze, execution lock, candidate
  freeze, final development receipt e terminal ledger;
- binding esatto del candidato standard `35 -> 512 -> 512 -> 2` e del suo albero
  a cinque file;
- matrice baseline-first di 12 rollout: sei baseline e sei candidate, con due
  offset deterministici (`-0.30 s`, `+0.30 s`) e quattro seed stocastici
  (`130-133`);
- detector binario V26 attivo sul ramo candidate, con profilo e contratto evento
  congelati;
- morphology corridor causale ritardato attivo sul ramo candidate, ma con peso,
  effetti causali e hard termination esattamente a zero;
- prova A/B di identità byte-per-byte della reward a peso zero, insieme a
  integrità di action/observation, campioni finiti del corridor e 5.000 campioni
  detector;
- gate fisici assoluti, non-inferiority condition-matched su reserve, residual e
  segnali SEA, e aggregazione obbligatoria `6/6` senza compensazione;
- divieto di retry, resume, rescue, sweep, teacher, blending, safety latch,
  actor/critic/PPO update, checkpoint-zero, morphology positiva e promozione.

## Strategia fail-closed

La source closure viene ricalcolata dai byte correnti al momento del preflight e
del freeze; gli hash transitori dei sorgenti R8 non sono cristallizzati nel
codice. I cinque payload restituiti dai verifier R8 devono inoltre avere hash e
dimensione identici ai rispettivi JSON canonici.

Il runtime è one-shot:

1. verifica nuovamente tutti i prerequisiti live R8;
2. verifica freeze/lock Q3, source closure e noise manifest;
3. esegue prima tutte le baseline e poi tutti i candidate;
4. persiste journal, trace e summary prima di valutare ogni gate;
5. termina definitivamente al primo errore, senza retry o resume;
6. dopo un eventuale `PASS` lascia comunque checkpoint-zero e morphology
   positiva a protocolli separati.

## File creati o modificati

Contratto e gate:

- `Trajectory Generator/baseline_MLP/validation/v12r8q3/__init__.py`
- `Trajectory Generator/baseline_MLP/validation/v12r8q3/h0_v12r8_q3_artifacts.py`
- `Trajectory Generator/baseline_MLP/validation/v12r8q3/h0_v12r8_q3_prerequisites.py`
- `Trajectory Generator/baseline_MLP/validation/v12r8q3/h0_v12r8_q3_qualification_contract.py`
- `Trajectory Generator/baseline_MLP/validation/v12r8q3/h0_v12r8_q3_qualification_gates.py`
- `Trajectory Generator/baseline_MLP/validation/v12r8q3/test_h0_v12r8_q3_source_scaffold.py`

Runtime e test, collocati nella cartella di validazione dedicata:

- `Trajectory Generator/baseline_MLP/validation/v12r8q3/runtime/__init__.py`
- `Trajectory Generator/baseline_MLP/validation/v12r8q3/runtime/freeze_h0_v12r8_q3_qualification_protocol.py`
- `Trajectory Generator/baseline_MLP/validation/v12r8q3/runtime/prepare_h0_v12r8_q3_noise_tapes.py`
- `Trajectory Generator/baseline_MLP/validation/v12r8q3/runtime/h0_v12r8_q3_physical_rollout.py`
- `Trajectory Generator/baseline_MLP/validation/v12r8q3/runtime/run_h0_v12r8_q3_qualification.py`
- `Trajectory Generator/baseline_MLP/validation/v12r8q3/runtime/test_h0_v12r8_q3_runtime.py`

## Test e verifiche eseguite

- suite congiunta V12R8 + V12R8-Q3: `90 passed` (`41 R8`, `49 Q3`), nessuno
  skip;
- `ruff check`: PASS;
- `ruff format --check`: PASS su 25 file R8/Q3;
- `compileall`: PASS;
- contract self-check Q3: `11/11` check PASS;
- source closure Q3: `PASS_H0_V12R8_Q3_SOURCE_CLOSURE`, 39 record, zero sorgenti
  mancanti;
- clean import: nessun caricamento anticipato di Ray, Torch o OpenSim;
- smoke test reale su macOS arm64: ambiente V26 costruito e resettato, observation
  `[84]` `float32`, detector `binary_active`, contratto evento V26 esatto;
- primo step fisico reale da 10 ms: 10 campioni detector, diagnostica morphology
  disponibile, morphology term zero e identità byte reward rispetto al ramo
  morphology-disabled;
- nessun file JSON/NPZ, protocol freeze, execution lock, noise root o run root
  creato sotto `v12r8q3`.

## Comandi preregistrati per la fase successiva

Questi comandi **non sono stati eseguiti**. Andranno lanciati in sequenza, con
stop immediato al primo exit code non-zero, solo dopo audit ed esecuzione R8 con
ledger terminale `PASS`:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python -m pytest -q 'Trajectory Generator/baseline_MLP/validation/v12r8' 'Trajectory Generator/baseline_MLP/validation/v12r8q3'
/opt/anaconda3/envs/envCMC-rllib/bin/python 'Trajectory Generator/baseline_MLP/validation/v12r8q3/runtime/freeze_h0_v12r8_q3_qualification_protocol.py' --live-preflight
/opt/anaconda3/envs/envCMC-rllib/bin/python 'Trajectory Generator/baseline_MLP/validation/v12r8q3/runtime/freeze_h0_v12r8_q3_qualification_protocol.py' --freeze
/opt/anaconda3/envs/envCMC-rllib/bin/python 'Trajectory Generator/baseline_MLP/validation/v12r8q3/runtime/prepare_h0_v12r8_q3_noise_tapes.py' --build-only
/opt/anaconda3/envs/envCMC-rllib/bin/python 'Trajectory Generator/baseline_MLP/validation/v12r8q3/runtime/prepare_h0_v12r8_q3_noise_tapes.py' --prepare
/opt/anaconda3/envs/envCMC-rllib/bin/python 'Trajectory Generator/baseline_MLP/validation/v12r8q3/runtime/prepare_h0_v12r8_q3_noise_tapes.py' --verify
/opt/anaconda3/envs/envCMC-rllib/bin/python 'Trajectory Generator/baseline_MLP/validation/v12r8q3/runtime/run_h0_v12r8_q3_qualification.py' --live-preflight
/opt/anaconda3/envs/envCMC-rllib/bin/python 'Trajectory Generator/baseline_MLP/validation/v12r8q3/runtime/run_h0_v12r8_q3_qualification.py' --execute
/opt/anaconda3/envs/envCMC-rllib/bin/python 'Trajectory Generator/baseline_MLP/validation/v12r8q3/runtime/run_h0_v12r8_q3_qualification.py' --verify
```

## TODO

- completare l'audit e l'esecuzione one-shot R8; Q3 resta bloccato se il ledger
  R8 non è terminale `PASS`;
- dopo R8 `PASS`, eseguire una sola volta freeze, tape materialization e qualifica
  V12R8-Q3 con la sequenza preregistrata;
- solo dopo Q3 terminale `PASS`, definire ed eseguire un protocollo separato per
  checkpoint-zero/warm-start;
- validare la morphology reward con peso positivo esclusivamente in una fase
  successiva separata: questa milestone valida integrazione e causalità a peso
  zero, non autorizza ancora effetti positivi sulla reward.
