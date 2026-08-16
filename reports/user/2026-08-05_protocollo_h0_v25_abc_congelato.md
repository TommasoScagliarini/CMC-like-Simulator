# Protocollo H0/V25 A/B/C congelato

Data: 2026-08-05

## Esito

Il protocollo dichiarativo per verificare la compatibilità fra H0 e il
detector binario V25 è stato congelato, senza eseguire rollout H0.

- lock autorevole corretto:
  `validation/h0_v25_abc_protocol_corrected_lock.json`;
- SHA del lock: `04ae8e209ccae05075b625f89ac827b145d5149e4237fe2128b1c822d105fe8b`;
- dimensione: 41.029 byte;
- 18 rollout fisici organizzati in 12 unità di protocollo;
- tutte le 18 assertion sono vere;
- tutte le 11 autorità restano `false`;
- H0, training, PPO, corridor, promozione runtime e trial protetti non sono
  stati avviati.

Il candidato geometrico resta V25
`v25_4b351f67b5b86ab0`, profilo SHA
`db704e502b99e49bea6d89493812bafdac748f8ce8d3ce28214ff624078539a2`.
La GRF primaria online non è stata modificata.

## Problema

Prima di usare V25 come sorgente attiva degli eventi occorre dimostrare due
cose separate:

1. caricare e processare V25 in shadow non deve modificare azioni, dinamica o
   output della baseline H0;
2. usando V25 come unica sorgente degli eventi sinistri, H0 deve ancora
   completare gli episodi e rispettare i gate fisici, SEA e reserve.

Il primo lock prodotto per questo protocollo, SHA
`4f24deb43537632f461b67ceb8d04ca4339a6b8e65e904ce7bc039334fb06b8e`,
conteneva inoltre un difetto di schema: il campo ridondante
`scientific_bundle_contract_id` compariva nel solo caso B. Il lock non aveva
mai autorizzato né eseguito H0, ma la differenza impediva di affermare che
l'unico fattore A/B fosse l'esecuzione della FSM shadow.

## Soluzione

Il primo lock non è stato cancellato o sovrascritto. È conservato come
evidenza rigettata e il nuovo lock lo supersede esplicitamente.

La correzione:

- rimuove il campo ridondante dal record B e mantiene il bundle scientifico
  nel blocco globale `contracts`;
- verifica che A e B differiscano esclusivamente per `case_name` e
  `binary_phase_fsm_mode`;
- registra separatamente la data del protocollo originario, 2026-08-04, e la
  data della correzione, 2026-08-05;
- riproduce byte per byte il lock rigettato prima di derivare quello corretto;
- usa strict JSON e scrittura atomica esclusiva no-clobber;
- mantiene chiuse tutte le autorità e dichiara esplicitamente i nove
  prerequisiti di esecuzione ancora mancanti.

## Strategia A/B/C congelata

- A — baseline: eventi analogici legacy autorevoli, V25 caricato e campionato,
  FSM V20 disabilitata, morphology reward a zero.
- B — controllo di non interferenza: stessi input e stessa baseline di A, con
  la sola FSM V20 eseguita in shadow. Ogni coppia A/B dovrà essere bit-exact.
- C — futuro caso attivo: gli eventi sinistri proverranno soltanto da V25/V20
  e saranno adattati alla `ProstheticPhaseFSM`; il fallback legacy sinistro è
  vietato.

Le sei condizioni congelate sono tre partenze deterministiche e tre partenze
stocastiche con seed 123–125. A/B formano sei unità indivisibili; C forma altre
sei unità, eseguibili soltanto dopo il PASS di tutte le coppie A/B. In totale:
12 unità e 18 rollout reali, ciascuno da 500 step.

## File coinvolti

Integrazione shadow e readiness:

- `Trajectory Generator/osim_trj_cmc_like.py`;
- `model_loader.py`;
- `simulation_runner.py`;
- `validation/test_binary_phase_fsm_env_v20.py`;
- `validation/freeze_v25_shadow_integration_readiness.py`;
- `validation/test_freeze_v25_shadow_integration_readiness.py`;
- `validation/binary_phase_detector_v25_shadow_readiness_receipt.json`;
- `reports/plans/2026-08-04_addendum_v25_protocollo_h0_abc.md`.

Freeze del protocollo:

- `validation/freeze_h0_v25_abc_protocol.py`;
- `validation/test_freeze_h0_v25_abc_protocol.py`;
- `validation/h0_v25_abc_protocol_lock.json`, conservato come lock rigettato;
- `validation/freeze_h0_v25_abc_protocol_corrected.py`;
- `validation/test_freeze_h0_v25_abc_protocol_corrected.py`;
- `validation/h0_v25_abc_protocol_corrected_lock.json`, lock autorevole.

Hash principali:

- addendum normativo: `a8cea18b338b08c32225c3912561cefe953add3b9e4bb693009aba6b189e9835`;
- readiness receipt shadow: `8c7316d19f4fe08bd90cd3476434ed111566c4ba7669c6c763dc163b6df073c2`;
- freeze script rigettato, preservato:
  `0c6df5b231548de69cd7257d2f55be8ecc139dfdf41cbd084eccd293659d48e1`;
- lock rigettato, preservato:
  `4f24deb43537632f461b67ceb8d04ca4339a6b8e65e904ce7bc039334fb06b8e`;
- freeze script corretto:
  `e2e24721681fde942f1a86b4bd920d6a17a0027891dab4feca7a221aadb1df14`;
- test del freeze corretto:
  `b98153dee6e373e8692cf4b7db5107b5a637db12cd97c78cbf961aac5fdd10ab`;
- lock corretto:
  `04ae8e209ccae05075b625f89ac827b145d5149e4237fe2128b1c822d105fe8b`.

## Test e verifiche

- `py_compile`: PASS;
- regressioni runtime, trasporto detector e FSM: 65/65 PASS;
- readiness e freeze governance: 17/17 PASS;
- totale suite mirata: 82/82 PASS;
- audit indipendente pre-freeze: GO, nessun blocker;
- strict JSON, finitezza, riproducibilità e hash esatto del lock: PASS;
- secondo tentativo di freeze: rifiutato fail-closed con exit code 2;
- lock storico e relativo script: hash invariati dopo il freeze corretto;
- suite ampia precedente: 103 PASS su 105; i due esiti restanti sono rifiuti
  fail-closed attesi dei preflight storici V24/V25, che vincolano il vecchio
  SHA di `model_loader.py` e non devono essere riscritti.

## Scope ancora chiuso

- modalità active C e adapter V20 -> `ProstheticPhaseFSM`: non implementati;
- driver A/B/C, journal A, replay B e comparatore bit-exact: non implementati;
- mapping delle tracce SEA, bootstrap partial-stance e output destinations:
  non congelati;
- execution-unlock separato: assente;
- H0/H0_sep: non eseguiti;
- trial protetti 05/06 e reserve 03/07: non aperti;
- promozione V25, corridor, morphology reward positivo, PPO e training: chiusi;
- eventuale claim numerico H0 futuro: soltanto macOS arm64 dopo una futura
  esecuzione autorizzata; Windows x86_64 resta in attesa di DLL e parità.

## TODO

- Implementare e testare i nove prerequisiti elencati nel lock corretto.
- Congelare un receipt separato di execution-unlock con codice, destinazioni e
  comparator esatti.
- Solo dopo tale freeze, eseguire prima le sei coppie A/B e, se tutte passano,
  i sei casi C.
- Solo dopo il gate H0 decidere l'apertura one-shot dei trial protetti 05/06.
- Mantenere corridor, reward positivo e PPO chiusi fino ai rispettivi gate.
