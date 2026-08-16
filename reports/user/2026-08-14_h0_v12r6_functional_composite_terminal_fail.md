# H0 V12R6 functional composite: sintesi PASS, sviluppo fisico terminal FAIL

Data: 2026-08-14

## Obiettivo

Recuperare un candidato imitativo utilizzabile come warm-start senza ripetere o
promuovere il lineage V12R5, mantenendo chiusi Q2/Q3, checkpoint-zero e reward
Morphology Corridor fino a una validazione fisica completa.

## Problema di partenza

V12R5 aveva migliorato la finestra critica `+0.20`, ma aveva fallito il gate
offline globale/per-case. L'analisi in prediction space ha mostrato che la
combinazione `0.70 * P2 + 0.30 * R5` superava tutte le soglie offline. Una
semplice interpolazione dei parametri non rappresentava però la stessa
funzione e la distillazione in una rete larga 256 introduceva nuovo errore.

## Soluzione implementata

È stato creato il nuovo lineage immutabile V12R6. Il candidato è un normale
`AsymmetricActorCriticTorchRLModule` con architettura `35 -> 512 -> 512 -> 2`:

- primo layer: concatenazione delle torri P2 e R5;
- secondo layer: matrice block-diagonal delle due torri;
- mean head: combinazione esatta `0.70 * P2 + 0.30 * R5`;
- log-std: preservato byte-exact, con sigma circa `0.005`;
- colonne clock 0-1: positive-zero e output invarianti;
- manifest delle 35 feature incluso nel checkpoint;
- trapianto warm-start verso un modulo learner largo 512 verificato con critic
  fresco preservato.

Il protocollo one-shot ha congelato 72 sorgenti e gli input P2/R5, ha vietato
fit, update, retry, qualification, promotion e reward morphology positivo, e
ha preregistrato sei rollout pure-policy nell'ordine critical-first.

## Verifiche source-only

- suite completa `validation/v12r6`: 79 test PASS;
- Ruff, format-check, `py_compile` e `git diff --check`: PASS;
- equivalenza funzionale sul corpus da 9.232 righe: errore massimo
  `1.1324882507324219e-06`, limite `2e-06`;
- equivalenza su 16 input deterministici: `9.5367431640625e-07`;
- compatibilità save/reload e warm-start/checkpoint-zero 512: PASS;
- freeze protocollo SHA-256:
  `d347f271aeddef3990bf6564550a2b7dab498988bf0132c64c5e22667b6e6cb8`;
- execution lock SHA-256:
  `8e8c04a195e962f2ec30cb81688edfc4370dace6b941c26d00197d95b17bc32f`.

## Gate offline del candidato canonico

La sintesi canonica ha superato tutti i gate:

- global: RMSE `0.0053635542`, max `0.05221927`, reset max `0.00092254`;
- `+0.20`: RMSE `0.0059064599`, max `0.04043931`;
- `-0.20`: RMSE `0.0058482193`, max `0.05221927`;
- seed 127: RMSE `0.0051694711`, max `0.03829579`;
- finestra critica `+0.20`: RMSE `0.0046853500`, max `0.02854511`,
  migliore del baseline P2 `0.0056222442 / 0.03706494`.

Candidate ID:

`AB06_H0_V12R6_FUNCTIONAL_COMPOSITE_A030_W512:340c2c65c2300a90ce46c09837e679a99e5dea09ce3935574ef5345fafb709f3`

## Esito dell'unica esecuzione fisica

Il primo rollout, `deterministic_offset_plus_0p20`, è terminato al passo 179:

- `end_reason = grf_penetration`;
- penetrazione finale `0.0257906750 m`, oltre il limite stretto `< 0.025 m`;
- `terminated = true`, `truncated = false`;
- nessun clipping azione;
- nessun hard invalid, timeout, fallback SEA o SO non accettata nell'ultimo
  step;
- zero teacher query, zero fit/update, zero Q2/Q3;
- 1 reset, 179 environment step, 1.790 raw sensor sample.

La trace non poteva quindi soddisfare il requisito di 500 step ed è stata
respinta dal pure-policy audit. Il runner ha pubblicato un ledger terminal FAIL
e non ha eseguito retry o stage successivi.

Ledger terminale SHA-256:

`ce67aea83b1f98aa251ad130af1da25982435381c0aeddeafa0a56bd3274e340`

## Interpretazione iniziale

Il gate offline è necessario ma non sufficiente per la stabilità closed-loop.
Nel tratto precedente al secondo contatto la traiettoria V12R6 diverge dalla
teacher sicura: al passo 150 l'azione caviglia è circa `-0.1095`, contro
`+0.1177` nella replay teacher sicura. La penetrazione cresce poi da
`0.01103 m` al passo 170 a `0.02579 m` al passo 179. Anche la precedente
raccolta shielded V12R4 era terminata per penetrazione, al passo 212: ridurre
soltanto il coefficiente R5 non è ancora giustificato come recupero sicuro.

## File aggiunti o modificati

- `Trajectory Generator/baseline_MLP/validation/v12r6/__init__.py`
- `build_h0_v12r6_composite_actor.py`
- `h0_v12r6_functional_composite_contract.py`
- `h0_v12r6_physical_development.py`
- `freeze_h0_v12r6_functional_composite.py`
- `run_h0_v12r6_functional_composite.py`
- test source-only associati nella stessa cartella;
- diagnostiche di design sotto `validation/v12r6/diagnostics/`;
- freeze, lock e artefatti canonici sotto
  `validation/v12r6/h0_v12r6_run_20260814/`.

## TODO vincolanti

- Non ritentare e non promuovere V12R6.
- Completare la diagnosi forense R6/V12R4/teacher replay.
- Aprire un nuovo lineage soltanto con un meccanismo che riduca l'errore
  closed-loop sulla distribuzione student-exposed; le opzioni in valutazione
  sono un fit 512 con recovery labels o una raccolta teacher-from-start
  preregistrata e sicura.
- Mantenere chiuso `v12r6q3`: detector V26 live, checkpoint-zero e Morphology
  Corridor restano subordinati a un futuro terminal PASS imitativo.
