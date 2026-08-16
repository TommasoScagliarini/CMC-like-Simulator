# H0 V12R9 checkpoint-zero: runtime latente e fail-closed

Data: 2026-08-14

## Problema

Il port `v12r8zero` non può essere riutilizzato dopo il terminal FAIL di V12R8: il suo contratto richiede terminal PASS nativi V12R8 e V12R8Q3. Serve quindi una lineage additiva che dipenda esclusivamente dai futuri terminal PASS V12R9 e V12R9Q3 dello stesso candidato, senza modificare o riaprire alcun artefatto V12R8.

Il comando di resume usa path relativi alla repository. Un'esecuzione dalla cartella `baseline_MLP` duplicherebbe il prefisso `Trajectory Generator/...`; la working directory deve quindi essere una parte esplicita e verificata del protocollo.

## Soluzione

È stata aggiunta la namespace `Trajectory Generator/baseline_MLP/validation/v12r9zero/`. Il contratto resta candidate-deferred e permette freeze e costruzione soltanto quando:

- il verifier semantico V12R9 restituisce terminal PASS;
- il verifier semantico V12R9Q3 restituisce terminal PASS;
- i due terminali espongono lo stesso `candidate_id` e lo stesso record full-tree a cinque file;
- Q3 usa la matrice indipendente canonica `-0.30/+0.30` e seed `130-133`, disgiunta dagli observer R9;
- manifest actor, input, profili e source closure sono integri e ri-hashati;
- la working directory è la root della repository.

Il runtime costruisce un Algorithm RLlib standard 35→512→512→2 nuovo, trapianta soltanto i dieci tensori actor qualificati e conserva il critic appena inizializzato. Verifica inoltre:

- clock feature disabilitate a bit-zero;
- pesi `logstd` a bit-zero e `sigma=[0.005, 0.005]`;
- critic byte-identico prima/dopo transplant, save e restore;
- optimizer privo di stato e identico su tutte le superfici;
- tutti i contatori sample/train/update a zero;
- checkpoint RLlib completo, save una sola volta e restore positivo;
- restore smoke con detector V26 attivo e morphology `0.0025`, senza `train()` né `sample()`.

Il solo handoff futuro è resume-only con `--resume-from checkpoint_zero`; `--warm-start` e `--warm-start-raw` sono vietati. Receipt e handoff fissano `required_working_directory=repository_root`, e freezer, runner e generazione dell'handoff rifiutano una cwd diversa prima di pubblicare output.

## Strategia fail-closed

- Nessun lock o checkpoint canonico è stato creato.
- Nessun terminale upstream è stato simulato nel percorso live.
- Output e lock sono no-clobber; symlink, junction e reparse point vengono rifiutati.
- Il port non autorizza training, actor update, critic update, PPO update o environment sampling.
- L'ABI source R9/R9Q3 è stata verificata contro le consegne definitive; freeze e runtime restano comunque subordinati ai terminal PASS nativi.

## File aggiunti

- `Trajectory Generator/baseline_MLP/validation/v12r9zero/__init__.py`
- `Trajectory Generator/baseline_MLP/validation/v12r9zero/h0_v12r9_zero_checkpoint_contract.py`
- `Trajectory Generator/baseline_MLP/validation/v12r9zero/h0_v12r9_zero_checkpoint_gates.py`
- `Trajectory Generator/baseline_MLP/validation/v12r9zero/freeze_h0_v12r9_zero_checkpoint.py`
- `Trajectory Generator/baseline_MLP/validation/v12r9zero/run_h0_v12r9_zero_checkpoint.py`
- `Trajectory Generator/baseline_MLP/validation/v12r9zero/test_h0_v12r9_zero_checkpoint.py`

Nessun file `v12r8*`, `v12r9/` o `v12r9q3/` è stato modificato da questo task.

## Test e verifiche

Eseguiti dalla root della repository:

- `pytest`: 24 test passati;
- `ruff check`: PASS;
- `ruff format --check`: PASS;
- `py_compile`: PASS;
- test negativo della cwd: freezer, runner e handoff rifiutano `baseline_MLP`;
- test parser reale: il comando macOS viene accettato da `train_ppo_mlp.py` dalla repository root;
- audit AST: il runner non contiene chiamate `.train()` o `.sample()` e contiene un solo save più i due restore previsti.

### Audit ABI upstream definitivo

Lo snapshot source R9 è vincolato a:

- schema `1290`;
- protocollo `AB06_H0_V12R9_ADDITIVE_RECOVERY_W512_V26`;
- pipeline `H0_V12R9_R8_PREFIX_IMPORT_SINGLE_FIT`;
- terminale `v12r9/h0_v12r9_run_20260814/pipeline_ledger.json` con stato richiesto `PASS_H0_V12R9_RECOVERY_PIPELINE_TERMINAL`;
- next stage `WAIT_SEPARATE_V12R9Q3_PROTOCOL`;
- contatori terminali attesi: actor `1`, critic `0`, PPO `0`.

Lo snapshot source R9Q3 è vincolato a:

- schema `1293`;
- protocollo `AB06_H0_V12R9_Q3_V26_MORPHOLOGY_ZERO_QUALIFICATION`;
- pipeline `H0_V12R9_Q3_BASELINE_FIRST_SIX_CASE_PAIRED`;
- terminale `v12r9q3/h0_v12r9_q3_run_20260814/pipeline_ledger.json` con stato richiesto `PASS_H0_V12R9_Q3_PIPELINE_TERMINAL`;
- receipt finale `finalize/receipt.json` con stato `PASS_H0_V12R9_Q3_INDEPENDENT_QUALIFICATION`;
- next stage `WAIT_SEPARATE_CHECKPOINT_ZERO_PROTOCOL`;
- matrice indipendente `-0.30`, `+0.30`, seed `130-133`, disgiunta dai casi R9;
- contatori terminali attesi: actor `0`, critic `0`, PPO `0`.

Il contratto zero importa questi valori dalle API pubbliche upstream e il suo self-check passa `9/9`. Il candidato resta intenzionalmente non valorizzato fino a quando i verifier terminali non restituiscono PASS per lo stesso record full-tree esatto composto da:

- `actor_feature_manifest.json`;
- `candidate_build_manifest.json`;
- `class_and_ctor_args.pkl`;
- `metadata.json`;
- `module_state.pkl`.

Regressioni indipendenti eseguite dalla root della repository sullo snapshot definitivo:

- R9: `76/76` pytest PASS, closure source `89`, Ruff lint/format PASS, compileall PASS;
- R9Q3: `57/57` pytest PASS, closure source `43`, Ruff lint/format PASS, compileall PASS;
- R9 checkpoint-zero: `24/24` pytest PASS, Ruff lint/format PASS, py_compile PASS;
- nessun protocol freeze, execution lock o run root canonico R9, R9Q3 o checkpoint-zero presente.

### Hash source checkpoint-zero definitivi

- `__init__.py`: `f27801fc3dffe6ff09077aff17df0c837c28d8fbd3ad282f8a9caa2b6d5eae6a`
- `h0_v12r9_zero_checkpoint_contract.py`: `9ad3889a53fea058972e641c3daf0e6cb1f8aee8801b6550aea15288188b7f70`
- `h0_v12r9_zero_checkpoint_gates.py`: `0816c465f544f5b9c19a807d03114d60e4cbd9ced21f7759571acadbcc580e33`
- `freeze_h0_v12r9_zero_checkpoint.py`: `5a8e93b3407f1f5e4b20cfa2df8fb896fc19bed92dc22631dfa9c87f6c3c3f46`
- `run_h0_v12r9_zero_checkpoint.py`: `580535d46eea1864b9f2a10521393f1d78245887a6316f3c535fd8cd794bceac`
- `test_h0_v12r9_zero_checkpoint.py`: `90b2c1d10dc03b45d8f19f9dfe09b10e92da002b474c6f8071ff4dcb04174e49`

## Stato milestone

Audit source definitivo completato: il port checkpoint-zero è source-ready contro le ABI stabili R9 e R9Q3. Il runtime resta intenzionalmente bloccato: freeze e checkpoint canonico sono vietati finché V12R9 e V12R9Q3 non hanno concluso con terminal PASS nativi dello stesso candidato e la root non autorizza esplicitamente il freeze zero.
