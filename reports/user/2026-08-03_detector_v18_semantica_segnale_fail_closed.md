# Detector V18 — confronto semantiche del segnale e stop fail-closed

Data: `2026-08-03`

## Esito

Il ciclo V18 development-only è terminato senza finalist:

`TERMINAL_NO_V18_SIGNAL_SEMANTIC_PASSES_SELECTION_PREREQUISITES`

Nessuna delle tre semantiche preregistrate supera i gate necessari sul trial
04. Non sono stati campionati né valutati nuovi segnali del trial 08; i trial
03/05/06/07 non sono stati letti. Non sono stati eseguiti H0, PPO, training o
promozione runtime.

La GRF primaria, il contatto C++, il plugin SEA, la geometria V17 e i parametri
`0.5/0.25 N`, dwell `30 ms` sono rimasti invariati. Il validatore statico del
contratto GRF primaria/V17 è ancora `PASS`.

## Problema

V17 aveva già dimostrato un HS tardivo di `+122 ms`, ma la distribuzione
completa chiarisce che non si tratta di un ritardo fisso: nel subset storico i
26 HS fuori `±50 ms` erano 25 anticipati e uno tardivo. Una correzione costante
del dwell avrebbe quindi peggiorato molti eventi.

È stato inoltre chiarito l'errore storico V15: il confronto usava un router
signal-only come se fosse equivalente al full FSM, pur non riproducendo
rejection e timeout dipendenti dallo stato. Non era un errore della GRF o delle
raw trace.

## Strategia V18

È stato congelato un protocollo separato con:

- trial 02 come diagnosi/challenge;
- trial 04 come selezione;
- trial 08 come holdout interno condizionale;
- tre sole semantiche, senza sweep:
  `heel_only`, `first_stable_regional`, `combined_load`;
- oracle esclusivamente dalle ledger canoniche V17;
- replay sequenziale a 1 ms e batch sugli stessi gruppi da 10 ms;
- stop prima del full FSM se ogni candidato fallisce già un gate necessario.

Sono state acquisite e congelate le raw trace:

- trial 02: 143.206 campioni;
- trial 04: 143.541 campioni;
- timestamp assoluti, unici e monotoni a 1 ms;
- carichi e penetrazioni tutti finiti;
- manifest e file legati da SHA-256.

## Risultati

Sul trial 04:

- `heel_only`: P1 ha 23 HS contro 24; P2/P3/P4 hanno errore HS massimo
  rispettivamente di 69/54/57 ms, oltre il limite di 50 ms;
- `first_stable_regional`: recupera il caso tardivo 02/P3 da +122 a +60 ms,
  ma resta fuori gate e introduce eventi/cicli extra; su 04/P2 produce 45 HS
  contro 33 e su P3 41 contro 38;
- `combined_load`: ha lo stesso esito necessario del primo sensore stabile;
  non ritarda i 14 HS anticipati del trial 04 e introduce conteggi extra.

Gli output di eventi ed edge sono identici tra processamento sequenziale e
batch. Nessun candidato è eleggibile e il protocollo ha quindi impedito
l'apertura del nuovo replay 08.

## Limiti emersi dalla review

La review indipendente ha ritirato il solo claim sul toe-clear: l'implementazione
V18 non scansiona un'eventuale riattivazione intermedia del latch toe come il
gate V17 autorevole. Lo stop resta invariato perché ogni candidato fallisce
indipendentemente count o timing HS. La parità provata è inoltre limitata ai
journal evento/edge e allo stato esplicitamente incluso nel digest, non è una
parità del full FSM runtime.

Il preflight ha letto l'identità del preprocessing lock e della ledger 08 già
esistenti, ma non ha prodotto raw trace o scoring V18 per 08. Il claim numerico
resta macOS arm64; `.gitattributes` forza LF sui nuovi artefatti per preservare
gli hash anche su checkout Windows.

## File principali aggiunti

- `reports/plans/2026-08-03_piano_detector_v18_semantica_segnale.md`
- `validation/two_sensor_v18_signal_semantics_protocol.json`
- `validation/acquire_two_sensor_v18_raw_traces.py`
- `validation/two_sensor_v18_selection_analysis_contract.json`
- `validation/evaluate_two_sensor_v18_signal_semantics.py`
- `validation/test_two_sensor_v18_raw_trace_acquisition.py`
- `validation/test_two_sensor_v18_signal_semantics.py`
- `validation/two_sensor_v18_signal_semantics_runs/`
- `.gitattributes`

## Verifiche

- 81 test mirati PASS, inclusi V18, V17, oracle canonico, FSM high-rate e data
  path detector;
- entrambi i preflight V18 `PASS`;
- audit indipendente di conteggi, hash, dimensioni e finitezza `PASS`;
- 3.641 record JSON/JSONL strict e privi di NaN/Inf;
- validazione statica del contratto GRF primaria/V17 `PASS`;
- nessun file temporaneo residuo.

Il passo successivo, se autorizzato, deve essere un nuovo ciclo preregistrato
con una semantica capace di ritardare causalmente l'ON. Le due alternative
monotone provate in V18 non possono risolvere gli anticipi dominanti.
