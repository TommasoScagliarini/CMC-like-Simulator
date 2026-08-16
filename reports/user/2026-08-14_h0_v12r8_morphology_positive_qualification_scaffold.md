# H0 V12R8: scaffold per la qualification positiva del Morphology Corridor

Data: 2026-08-14

## Stato della milestone

La chiusura sorgente dello scaffold additivo `v12r8morph` è completa e
verificata. Il protocollo positivo del Morphology Corridor è definito in modo
fail-closed, ma non è ancora congelabile né eseguibile: servono prima i PASS
terminali della stessa lineage V12R8, V12R8-Q3 e V12R8-zero sul medesimo
candidato e checkpoint.

In questa milestone non sono stati creati lock, receipt, rollout, checkpoint o
training. Le entrypoint pubbliche di freeze ed execution terminano prima di
qualsiasi I/O finché gli upstream non sono finali.

## Problema

Dopo la qualification Q3 a peso morfologico zero serve dimostrare che il primo
effetto positivo del corridor:

- modifica esclusivamente la ricomposizione della reward;
- non altera osservazioni, azioni, dinamica o sequenza degli eventi V26;
- mantiene intatta la causalità con delay `0.04 s` e latenza massima `0.01 s`;
- non introduce failure fail-closed o drop inattesi;
- non effettua alcun aggiornamento actor, critic o PPO;
- non apre uno sweep di pesi né autorizza il peso `0.005`.

La difficoltà aggiuntiva è evitare sei rollout control ridondanti quando Q3 può
già fornire evidenza weight-zero sufficientemente dettagliata, senza però
accettare trace incomplete o mutate.

## Soluzione

È stato introdotto un protocollo source-only con un solo confronto ammesso:

- control `morphology_weight = 0.0`;
- positive `morphology_weight = 0.0025`;
- phase mode `event_anchored_causal_delayed_experimental`;
- `morphology_causal_allow_effects = 1.0` soltanto nel ramo positive;
- hard termination disabilitata;
- detector binario V26 e relativi profilo/hash fissati.

I sei case ID, le sei condizioni held-out e gli SHA-256 degli array dei noise
tape sono copiati per valore dal contratto V12R8-Q3 schema 1283 e protetti da
un test di identità esatta.

Per ogni coppia devono risultare byte-identici:

- observations;
- actions;
- dynamics;
- events;
- reward senza morphology;
- morphology loss.

La sola differenza ammessa è verificata con uguaglianza IEEE-754 esatta a ogni
step:

`positive_reward = control_reward - 0.0025 * morphology_loss`

Il gate richiede inoltre V26 integro, zero fallback/eventi duplicati o fuori
ordine, zero causal fail-closed, zero unexpected drop e diagnostica causale
identica tra i due rami.

## Strategia control

La modalità preferita riusa i sei control Q3 ed esegue soltanto sei rollout
positive. Ogni control riusato deve essere hash-bound nel lock tramite:

- summary;
- trace;
- noise tape;
- control receipt;
- ledger reward per-step;
- stream byte-stabili e binding completo di candidato, actor e checkpoint.

Il lock rifiuta qualunque drift successivo degli artefatti sorgente, degli
upstream o dell'evidenza Q3. Se Q3 non espone trace sufficienti, l'unico
fallback ammesso è un rerun locale esplicito di dodici rollout, sempre in
ordine control-prima-del-positive per ciascuno dei sei casi. Non sono ammessi
mode ambigui, retry, rescue o sweep.

## Autorità training

Soltanto un receipt terminale PASS di tutte e sei le coppie può pubblicare il
comando finale multipiattaforma. Il comando è fissato a 50 update e riparte dal
full checkpoint `checkpoint_zero` con `--resume-from`; `--warm-start` e
`--warm-start-raw` sono vietati. Sono presenti rendering separati per macOS
arm64 e Windows x86_64, e l'handoff deve hash-attestare il receipt terminale
PASS.

## File aggiunti

Nella cartella
`Trajectory Generator/baseline_MLP/validation/v12r8morph/`:

- `__init__.py`;
- `h0_v12r8_morphology_contract.py`;
- `h0_v12r8_morphology_gates.py`;
- `freeze_h0_v12r8_morphology.py`;
- `run_h0_v12r8_morphology.py`;
- `test_h0_v12r8_morphology_scaffold.py`.

## Test e verifiche

- test dello scaffold: **29/29 PASS**;
- Ruff format: PASS;
- Ruff check: PASS;
- compilazione Python dei sei file: PASS;
- `git diff --check`: PASS;
- hash dei profili V26 e Morphology Corridor verificati sui file reali: PASS;
- modalità control sconosciuta verificata fail-closed senza eccezioni spurie;
- freezer diretto: exit 1 deferred prima di I/O;
- runner diretto: exit 1 deferred prima di I/O;
- nessun artefatto runtime prodotto e nessun rollout/training eseguito.

## TODO vincolanti

- [ ] Attendere e verificare il terminal PASS V12R8.
- [ ] Attendere e verificare il terminal PASS V12R8-Q3 sullo stesso candidato.
- [ ] Attendere e verificare V12R8-zero e il full checkpoint `checkpoint_zero`
      sullo stesso candidate/tree/actor binding.
- [ ] In una nuova revisione immutabile, collegare path, status e validator
      terminali definitivi dei tre upstream senza indovinarli.
- [ ] Valutare la capability reale delle trace Q3: riuso di sei control se
      sufficiente, altrimenti fallback esplicito a dodici rollout locali.
- [ ] Solo dopo i prerequisiti, congelare ed eseguire una singola qualification
      positiva; pubblicare il comando da 50 update esclusivamente su PASS.
