# Trial 08 V22 development — errore booleano prima del replay

Data: 2026-08-04

## Esito

Il ciclo V22 sul trial 08 è stato aperto correttamente ma si è chiuso in:

`ERROR_V22_TRIAL08_OPEN_DEVELOPMENT_REPLAY_CONSUMED`

L'errore è avvenuto prima del campionamento del detector. Non è quindi un
FAIL della geometria V21 o della FSM V20 e non produce alcuna nuova evidenza
scientifica sulle prestazioni HS/TO.

Il trial 08 era già classificato come dato development aperto, non holdout e
non validazione indipendente. I trial protetti 05/06 e le riserve 03/07 non
sono stati aperti.

## Problema corretto da V22

V22 ha corretto il vincolo che aveva fermato V21: la griglia globale
dell'oracle non deve coincidere con quella della trace del detector.

La verifica congelata dimostra correttamente che:

- oracle: `[10.678, 154.900]`, 144223 campioni a 1 ms;
- trace detector: `[10.690, 154.890]`, 144201 campioni a 1 ms;
- margine iniziale: 12 campioni;
- margine finale: 10 campioni;
- identità: `144223 - 12 - 10 = 144201`;
- tutte le quattro view e tutte le finestre degli eventi scoreabili sono
  coperte;
- l'oracle è riutilizzato senza ricostruzione o nuova sogliatura della GRF.

Questa parte del fix ha superato i test ed è contenuta nel freeze SHA-256
`1a909f6a48aa5fece3521144e6e0829199d7c7d3831f6d355bdf176608926077`.

## Nuovo errore procedurale

Nel loader dell'oracle V22 è stato costruito un dizionario `checks` che
conteneva sia asserzioni sia un fatto descrittivo:

```python
"global_grid_equality_required": False
```

Subito dopo, il runner ha valutato `all(checks.values())`. Il valore `False`
era quello corretto per il nuovo contratto, ma è stato interpretato come gate
fallito. Tutte le altre voci del dizionario erano `True`, incluse
`coverage=True` e `coverage_exactly_frozen=True`.

La causa è quindi un errore booleano nel validatore, non nei dati, nel
detector binario, nella geometria o nella FSM.

La suite pre-esecuzione copriva il gate puro di coverage e i guard operativi,
ma non eseguiva end-to-end il binding dell'oracle reale attraverso
`_load_oracle`; per questo non ha intercettato la mescolanza fra fatto
descrittivo e asserzioni.

## Stato degli artefatti

- freeze V22:
  `validation/binary_phase_detector_v22_trial08_development_freeze_lock.json`;
- ledger V22:
  `validation/binary_phase_detector_v22_trial08_development_execution_ledger.json`,
  SHA-256
  `abf074400dc2fb112e9375ce26d89eb55eacea16d348789c73f876cb1004a917`;
- access receipt V22: byte-identica al ledger;
- failure V22:
  `validation/binary_phase_detector_v22_development_runs/2026-08-04_trial08_oracle_coverage_fix/failure.json`,
  SHA-256
  `19bd2edfd0a9ffca113308fc84845c838caf54fc4a21b645ec2e351fd4ea8898`;
- nessuna `packed_binary_trace.json`;
- nessun `manifest.json` di evidenza;
- nessuna decisione PASS/FAIL prestazionale;
- nessuna esecuzione H0, promozione runtime/training o PPO;
- nessuna modifica alla GRF primaria, al plugin C++ o alla semantica SEA.

Gli artefatti terminali V21 sono rimasti invariati:

- freeze `68774248090a071221eecdd1ca771d9f598c36b3f89d2f7497ede2cdd4cc2964`;
- ledger `37ce1f009e4f99904063ccb8d997e6bc2a8ad62d105a2ffe676164cc6eafebde`;
- failure `fba0255fd35e25ffc53a09af64221aa7d9868129a442ed0ee915784b4f2db5f4`.

## File introdotti nel ciclo V22

- `validation/freeze_binary_phase_detector_v22_trial08_development.py`;
- `validation/validate_binary_phase_detector_v22_trial08_development.py`;
- `validation/test_binary_phase_detector_v22_trial08_development.py`;
- `validation/binary_phase_detector_v22_trial08_development_freeze_lock.json`;
- ledger, receipt e failure elencati sopra.

I sorgenti V22 congelati non devono essere modificati o riutilizzati per un
retry.

## Test e verifiche

Prima del freeze:

- `py_compile`: PASS;
- 15/15 test V22: PASS, zero skip;
- verifica oracle reale e sottogriglia 12/10: PASS;
- preflight OpenSim/plugin/modello/profilo: PASS;
- freeze `--check`: PASS.

Dopo l'errore:

- freeze V22 ancora verificabile e byte-stabile: PASS;
- ledger e receipt byte-identici: PASS;
- assenza della trace del detector: confermata;
- assenza del manifest prestazionale: confermata;
- hash terminali V21 invariati: confermati.

## Soluzione proposta per V23

Non modificare V22. Creare un nuovo ciclo development V23 separato, sempre sul
trial 08 già aperto, mantenendo candidato, geometria, FSM, oracle e gate
numerici invariati. L'unica correzione deve essere la separazione fra:

- `assertions`, tutte attese `True` e sole partecipanti a `all(...)`;
- `facts`, che possono legittimamente contenere
  `global_grid_equality_required=False`.

Prima di un eventuale freeze V23 occorre aggiungere un test di integrazione che
esegua il binding completo dell'oracle reale e dimostri che il loader restituisce
PASS senza avviare il campionamento del detector. Deve inoltre esserci un audit
automatico che impedisca a campi descrittivi di entrare nell'aggregazione dei
gate.

## TODO

- **RISOLTO:** l'utente ha autorizzato esplicitamente V23. Il fix isolato e il
  test di integrazione oracle-binding sono stati implementati senza modificare
  V22.
- V23 ha poi completato il replay con un FAIL scientifico del candidato,
  documentato in `2026-08-04_trial08_v23_fail_gap_geometrico_12ms.md`.
