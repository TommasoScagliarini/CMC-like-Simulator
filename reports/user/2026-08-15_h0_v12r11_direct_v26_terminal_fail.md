# H0 V12R11 diretto su V26: probe terminale FAIL

Data: 2026-08-15

## Obiettivo

Discriminare la causa del fallimento V12R10 verificando se il checkpoint H0
originale del 15 luglio, senza adapter imitativo o nuovo fit, potesse comandare
direttamente il runtime a 35 feature con detector binario V26 attivo.

Il caso prescritto era il più severo già fallito da V12R10:
`deterministic_offset_plus_0p20`, offset `2.156870983805102 s`, seed runtime
`123`, 500 step, rumore di rollout nullo e Morphology Corridor a peso `0.0`.

## Problema

V12R10 approssimava un teacher costruito tramite `LegacyGaitShadow`: le label
erano quindi `H0(P_legacy(x, history))`, non `H0(x)` sulla observation V26
reale. Il fit W1024 aveva superato i gate offline ma il primo rollout fisico si
era fermato a 212 step per penetrazione GRF. Restava da distinguere fra errore
di imitazione e incompatibilità fisica diretta H0/V26.

## Soluzione e strategia

È stato realizzato un probe one-shot V12R11 che:

- carica byte-exact il source H0 W256, tree SHA-256
  `f7f6c898975af109412af8c3f1a338b5076f9fefcec1e2723673fd821f1f13ee`;
- invia direttamente `observation[:35]` V26 a H0 una volta per step;
- vieta fit, teacher, blend, safety latch e update actor/critic/PPO;
- mantiene detector/FSM V26 attivi e Morphology Corridor a peso zero;
- attesta prima e dopo il rollout i 94 file della closure runtime congelata da
  V12R10, ancorata all'execution lock
  `e6601a67d251f19586f25f834df09957450a78fac18ace5d620902314888fb82`;
- salva un journal esclusivo per step e chiude trace, summary, gate e
  `closure_receipt.json` anche in caso di terminazione anticipata;
- separa l'integrità del diagnostico dall'esito fisico.

## Esito

Il risultato è terminale per la linea V12R11:

- stato gate: `FAIL_H0_V12R11_SOURCE_H0_ON_V26_PLUS_PROBE`;
- integrità diagnostica: PASS;
- esito fisico: FAIL;
- step completati: `209/500`;
- causa: `grf_penetration`;
- penetrazione massima: `0.025489193765034043 m`, oltre il limite stretto
  `< 0.025 m`;
- cicli validi: `0`;
- eventi accettati: `2`;
- clipping azioni: `0`;
- anomalie, timeout, eventi invalidi, fallback SEA e SO non accettate: `0`;
- sorgente eventi: esclusivamente `binary_active_v26`;
- teacher query, dipendenze teacher, blend, latch e update: `0`.

La closure forense è PASS e vieta retry/resume. H0 diretto fallisce quindi
senza l'errore di approssimazione e senza clipping: il blocker è la
compatibilità closed-loop tra la policy storica e la semantica V26, non il
solo fit V12R10.

## Diagnostica comparativa con il tape sicuro

Una seconda analisi deterministica ha confrontato V12R11, V12R10 e il replay
V8R1P1 `+0.20` che completa 500 step:

- lo stato iniziale sulle 18 feature invarianti è byte-identico;
- l'azione diverge già allo step 1 e lo stato invariante allo step 2;
- V12R11 contro tape: RMSE azioni `0.0714949`, massimo `0.261318445` sulla
  seconda azione allo step 207;
- V12R10 contro tape: RMSE azioni `0.1125717`, massimo `0.4574433`;
- il tape sicuro raggiunge una penetrazione massima di
  `0.024323924384327976 m` allo step 211, con margine di soli `0.676076 mm`,
  poi torna a scendere;
- V12R11 esce dal 99° percentile del supporto 18D del tape allo step 14 e
  supera l'intero envelope leave-one-out agli step 208–209;
- avere zero clipping non basta: V12R11 e V12R10 falliscono con errori azione
  diversi ma con la stessa deriva closed-loop fuori supporto.

La conseguenza per V12R12 è che il target corretto è il
`frozen_teacher_mean` dei tape V5/V26 sulle colonne `2:10` e `25:35`, ma il
solo fit statico on-tape non costituisce una prova di stabilità. La nuova
lineage deve includere dati candidate-exposed/recovery prima del primo test
autonomo puro.

Artefatti diagnostici:

- `compare_direct_h0_safe_tape.py`:
  `9276686f75f1fcf19399b88227537d860637d6570ccdb061aff2e80fa68c3abe`;
- `test_compare_direct_h0_safe_tape.py`:
  `043e5f2ed94df5aea86d91694f5c4cb5acf3e474af8df89acbdc815ed19a60af`;
- `results/comparison.json`:
  `e90dbd99b6670cdd9e267c6dc4209809506331a647068a60c105ab47eb763a32`.

## Artefatti principali

Root:

`Trajectory Generator/baseline_MLP/validation/v12r11/diagnostics/h0_on_v26_plus_probe/artifacts/20260815_deterministic_offset_plus_0p20`

- `run_start.json`: `66ea1fd5a56cd5f5d2344b238d0e3d43299906e3374f0b5fd68f1ac9896a69e1`;
- `trace.json`: `b5dc7a277fc013ecc1685354c969d9702d343d00925406fa04066032dae42a7d`;
- `summary.json`: `de39911c3b91a9d6fffc4f76d5a5671f68d3c8549ba2177d7034a8dd5201304e`;
- `gate.json`: `d93e6dc42278cf9b4c0a6e2d1713fb8ca7d5077c0f8b4cdc2b341f4e26782bef`;
- `closure_receipt.json`:
  `a0a694fae32b3edf41b0c159247dbb9602fefab52dd2428cdaa7b4a64f5e9ccf`.

## File aggiunti o modificati

- `Trajectory Generator/baseline_MLP/validation/v12r11/diagnostics/h0_on_v26_plus_probe/__init__.py`;
- `Trajectory Generator/baseline_MLP/validation/v12r11/diagnostics/h0_on_v26_plus_probe/probe.py`;
- `Trajectory Generator/baseline_MLP/validation/v12r11/diagnostics/h0_on_v26_plus_probe/run_probe.py`;
- `Trajectory Generator/baseline_MLP/validation/v12r11/diagnostics/h0_on_v26_plus_probe/test_probe.py`;
- artefatti one-shot elencati sopra.

## Test e verifiche

- 19 test `pytest`: PASS;
- Ruff lint: PASS;
- Ruff format check: PASS;
- `py_compile`: PASS;
- `git diff --check`: PASS;
- review avversaria indipendente finale: GO;
- verifica closure transitiva runtime: `94/94` file byte/hash esatti;
- closure receipt post-rollout: PASS.
- diagnostica comparativa: 6 test PASS, Ruff/format/compile PASS e
  `--check-only` deterministico PASS.

## Decisione e TODO propagato

V12R11 è chiuso senza retry. Non sono autorizzati nuovi widening o fit verso
lo stesso target legacy.

TODO successivo: costruire una nuova lineage V26-only W256 dai sei action tape
sicuri riprodotti 6/6 su V26, usando inizialmente soltanto le 18 feature
meccaniche/reference invarianti (`2:10` e `25:35`). Il primo gate fisico resta
lo stesso caso deterministico `+0.20`; solo dopo il suo PASS si potranno aprire
development completo, Q3, checkpoint-zero, Morphology Corridor e training.
