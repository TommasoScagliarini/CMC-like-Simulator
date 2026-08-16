# H0 V12R9 — recovery additiva da R8 source-ready

Data: 2026-08-14

## Problema

Il one-shot V12R8 si è chiuso correttamente come terminal `FAIL` durante
`collect_label__deterministic_offset_minus_0p20`. L'audit read-only ha isolato
un'unica divergenza: il summary persisted contiene il record completo a cinque
file del candidato R6, mentre il lock R8 ne conserva soltanto la proiezione
`path/tree_sha256/file_count`. I valori condivisi, l'hash dell'albero e tutti i
cinque file coincidono; il confronto strutturale stretto fallisce esclusivamente
per il campo extra `files`.

Il prefisso minus resta comunque evidenza valida e utile:

- 252 step, 253 boundary e 3 eventi replay;
- 2520 campioni detector V26;
- trace, journal e replay coerenti;
- tutti i contatori di anomalia detector/runtime a zero;
- rollout teacher-free, senza blend, latch o update;
- gate e receipt R8 esatti dopo la sola sostituzione controfattuale della
  proiezione candidato con il record full-tree già persistito.

R8 è terminale e immutabile: non può essere corretto, ripreso o ritentato nel
proprio namespace.

## Soluzione

È stato creato il successore additivo `validation/v12r9/`, schema 1290,
protocollo `AB06_H0_V12R9_ADDITIVE_RECOVERY_W512_V26`.

V12R9:

1. attesta e adjudica in sola lettura il ledger terminale e il prefisso minus
   R8;
2. importa per riferimento diretto, senza copia e senza nuove query H0, le 179
   label plus già chiuse in R8;
3. genera offline le 252 label same-state del replay minus soltanto dopo
   l'adjudication positiva, con zero reset e zero step ambiente;
4. raccoglie e labelizza soltanto i quattro casi mai avviati da R8;
5. esegue un unico fit full-mean W512 sugli stessi 13 strata bilanciati;
6. congela semanticamente il solo candidato risultante;
7. esegue sei development pure-policy con detector binario V26 attivo e
   morphology a peso zero;
8. chiude un ledger terminale one-shot senza retry, resume, sweep, checkpoint
   zero, morphology positiva o apertura anticipata di Q3.

Il lock del candidato R6 è stato corretto alla forma completa: path, tree hash,
file count e lista esatta dei cinque file con hash e size. La vecchia proiezione
ridotta viene accettata soltanto come spiegazione storica della causa R8, mai
come identità sufficiente del candidato.

## Strategia fail-closed

Il protocollo contiene 16 stage univoci: adjudication, import plus, label minus,
quattro collection/label nuove, fit, freeze, sei development e finalizzazione.
L'accounting distingue esplicitamente:

- prefissi R8: 0 reset e 0 step ambiente;
- nuove collection: quattro rollout fisici;
- development: sei rollout fisici;
- label importate plus, label adjudicated minus e label delle nuove collection;
- un actor fit/update, zero critic update e zero PPO update.

Ogni stage receipt è verificato semanticamente. In particolare, lo schema del
labeling minus richiede `imported_r8_prefix=true`, mentre quello delle nuove
collection vieta tale campo e richiede `single_collection_round=true`. Una
regressione percorre tutti i 17 confini osservabili del protocollo (16 stage più
terminale) e verifica la coerenza dell'activity ledger a ogni possibile punto di
arresto.

Le scritture sono limitate al namespace R9. Tutti gli artefatti R8 e i tre
sorgenti R8 necessari alla verifica sono hash-locked e usati soltanto in
lettura. La source closure R9 corrente contiene 89 file.

## ABI preservata per i successori

Sono disponibili i cinque endpoint pubblici attesi da Q3 e checkpoint-zero:

- `verify_protocol_freeze()`;
- `verify_execution_lock(require_pristine=False)`;
- `verify_candidate_freeze_receipt()`;
- `verify_final_development_receipt()`;
- `verify_terminal_ledger()`.

Q3-R9 e checkpoint-zero-R9 restano separati e deferred: non possono aprirsi
prima di un ledger terminale `PASS` R9 con candidato completo a cinque file.

## File creati

Produzione in `Trajectory Generator/baseline_MLP/validation/v12r9/`:

- `__init__.py`;
- `h0_v12r9_recovery_contract.py`;
- `h0_v12r9_prefix_adjudicator.py`;
- `h0_v12r9_recovery_probe.py`;
- `h0_v12r9_recovery_fitter.py`;
- `freeze_h0_v12r9_recovery.py`;
- `run_h0_v12r9_recovery.py`.

Test nella medesima cartella di validazione:

- `test_h0_v12r9_recovery_contract.py`;
- `test_h0_v12r9_prefix_adjudicator.py`;
- `test_h0_v12r9_recovery_probe.py`;
- `test_h0_v12r9_recovery_fitter.py`;
- `test_h0_v12r9_semantic_closure_mutations.py`;
- `test_freeze_h0_v12r9_recovery.py`;
- `test_run_h0_v12r9_recovery.py`.

## Test e verifiche

- contract self-check: 13/13 check PASS;
- attestation reale degli input locked: PASS;
- adjudication read-only dei veri artefatti R8:
  `PASS_H0_V12R9_R8_MINUS_PREFIX_ADJUDICATION`;
- causa unica R8 confermata:
  `STRICT_FULL_TREE_VS_LOCKED_PROJECTION_EQUALITY`;
- chiusura controfattuale del prefisso minus: PASS;
- test behavioral sullo stage receipt di una nuova collection e mutazione del
  campo import: PASS;
- regressione activity su tutti i confini stage/terminale: PASS;
- suite V12R9 completa: 76 PASS;
- `ruff check`: PASS;
- `ruff format --check`: PASS su 14 file;
- `compileall`: PASS.

## Stato della milestone

Il core V12R9 è **source-ready / pre-freeze**. Non sono stati creati protocol
freeze, execution lock o run root canonici R9 e non è stato avviato il one-shot.
Di conseguenza non esiste ancora un candidato R9 né un'autorizzazione a Q3,
checkpoint-zero o training warm-start.

## TODO

- completare l'audit indipendente finale su hash e ABI dello snapshot formattato;
- solo dopo GO dell'audit, pubblicare freeze e execution lock R9;
- eseguire una sola volta il runner R9 e richiedere ledger terminale `PASS`;
- mantenere Q3 e checkpoint-zero chiusi in caso di terminal `FAIL`;
- dopo R9 PASS, eseguire Q3 separato e poi il protocollo checkpoint-zero;
- abilitare morphology positiva esclusivamente nel protocollo causale dedicato,
  dopo la validazione a peso zero.
