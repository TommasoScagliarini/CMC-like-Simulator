# H0 V12R8: runtime latente per il full checkpoint-zero

Data: 2026-08-14

## Stato della milestone

Il protocollo `v12r8zero` è ora implementato come runtime completo ma latente:
può costruire, salvare e verificare un full checkpoint RLlib a progresso zero
non appena V12R8 e V12R8-Q3 pubblicano entrambi un terminal PASS sul medesimo
candidato. Fino a quel momento resta fail-closed.

In questa milestone non sono stati congelati lock e non è stato eseguito il
runtime canonico. Non esistono né la directory di output né `checkpoint_zero`.
I terminal ledger R8 e Q3 sono ancora assenti, quindi il protocollo non è oggi
autorizzabile.

## Problema

Dopo il warm-start imitativo R8 e la qualification Q3 a corridor disattivato
serve convertire il solo actor qualificato in un checkpoint RLlib completo,
senza confondere il risultato con un nuovo warm-start o con un passo di
training. Il checkpoint deve contenere anche un critic fresco e lo stato
optimizer necessario al restore, ma deve attestare contemporaneamente:

- actor standard W512 esatto, con 35 feature actor, 84 feature complete e due
  azioni;
- critic non ereditato dal candidato e byte-identico prima e dopo il
  transplant/save/restore;
- optimizer vuoto e con registrazione parametrica esatta;
- tutti i contatori RLlib a zero;
- nessuna chiamata `train()` o sampling dell'environment;
- detector binario V26 realmente attivo;
- Morphology Corridor causale realmente configurato, ma con peso ed effetti a
  zero nel checkpoint;
- restore positivo di sola validazione con peso `0.0025`, senza update;
- futuro avvio esclusivamente da `--resume-from checkpoint_zero`.

## Soluzione

È stato aggiunto un namespace autonomo
`Trajectory Generator/baseline_MLP/validation/v12r8zero/` con contratto,
gate, freezer, runner one-shot e test.

Il lock non si limita a leggere lo stato degli upstream: invoca i verifier
terminali pubblici di R8 e Q3 e registra l'esito canonico di ciascuna verifica.
I gate richiedono status, schema, protocol/pipeline ID, artefatti finali e lo
stesso tree SHA-256 del candidato a cinque file. I binding stabili sono:

- R8 schema 1280, terminal status
  `PASS_H0_V12R8_RECOVERY_PIPELINE_TERMINAL`;
- Q3 schema 1283, terminal status
  `PASS_H0_V12R8_Q3_PIPELINE_TERMINAL`;
- verifier `verify_terminal_ledger` specifico di entrambi i runtime.

Il runner, dopo un lock valido, costruisce un Algorithm standard W512 fresco,
carica il candidato R8 e trapianta esclusivamente l'actor. Salva quindi un full
checkpoint RLlib, lo ripristina in una configurazione zero e infine in una
configurazione positiva `0.0025`. A ogni fase confronta le superfici actor,
critic, optimizer e progress. La configurazione live viene costruita prima del
restore, così il checkpoint non può sovrascrivere detector o reward target.

## Reward e handoff

Il checkpoint-zero usa:

- detector V26 in modalità `binary_active`;
- Morphology Corridor
  `event_anchored_causal_delayed_experimental`;
- delay `0.04 s` e latenza massima `0.01 s`;
- `morphology_weight = 0.0`;
- `morphology_causal_allow_effects = 0.0`.

Il positive restore smoke modifica soltanto:

- `morphology_weight = 0.0025`;
- `morphology_causal_allow_effects = 1.0`.

Il receipt checkpoint-zero non autorizza ancora il training. Pubblica soltanto
un handoff non autorizzante verso la qualification positiva `v12r8morph`. Il
comando finale resta fissato a 50 update, con `--resume-from checkpoint_zero` e
senza `--warm-start` o `--warm-start-raw`, ma potrà essere pubblicato soltanto
dopo il PASS separato del confronto morfologico positivo.

## File aggiunti

In `Trajectory Generator/baseline_MLP/validation/v12r8zero/`:

- `__init__.py`;
- `h0_v12r8_zero_checkpoint_contract.py`;
- `h0_v12r8_zero_checkpoint_gates.py`;
- `freeze_h0_v12r8_zero_checkpoint.py`;
- `run_h0_v12r8_zero_checkpoint.py`;
- `test_h0_v12r8_zero_checkpoint.py`.

## Test e verifiche

- suite locale V12R8zero: **22/22 PASS**;
- suite integrata V12R8 + V12R8-Q3 + V12R8zero: **122/122 PASS**;
- regressione dello scaffold storico V12R7zero: **20/20 PASS**;
- Ruff format check sui sei file: PASS;
- Ruff check sui sei file: PASS;
- compilazione sorgente Python dei sei file: PASS;
- parser reale del training compatibile con il solo comando resume: PASS;
- audit AST: un full `save_to_path`, due `restore_from_path`, zero chiamate
  `train()`/`sample()`: PASS;
- hash reali dei profili detector e morphology: PASS;
- lock canonico V12R8zero: assente;
- output root e checkpoint canonico V12R8zero: assenti;
- terminal ledger canonici R8 e Q3: assenti.

## TODO vincolanti

- [ ] Ottenere e verificare il terminal PASS V12R8.
- [ ] Ottenere e verificare il terminal PASS V12R8-Q3 sullo stesso candidate
      ID e sullo stesso tree SHA-256.
- [ ] Solo dopo entrambi i PASS, congelare una volta il lock V12R8zero.
- [ ] Eseguire una sola volta il runtime V12R8zero e ottenere il receipt PASS
      del full `checkpoint_zero` a progresso zero.
- [ ] Eseguire e superare la qualification positiva separata `v12r8morph`.
- [ ] Solo dopo il PASS morfologico, pubblicare ed eseguire il training da 50
      update tramite `--resume-from checkpoint_zero`.
