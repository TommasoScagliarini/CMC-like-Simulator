# Iterazioni V6–V9 sul posizionamento del detector a due sensori

Data: 2026-07-22

## Esito esecutivo

Sono state completate quattro revisioni del protocollo di posizionamento,
mantenendo rigorosamente il detector nella forma richiesta:

- una sfera virtuale sul tallone;
- una sfera virtuale sull'avampiede;
- nessuna mesh di contatto usata dal detector;
- i due sensori producono soltanto le guardie Heel Strike e Toe Off;
- le evidenze di carico e contatto della FSM nel replay offline provengono dal
  profilo primary GRF separato;
- soglie `0.5/0.25 N`, dwell `30 ms`, FSM e minimo stance `300 ms` invariati.

Il risultato finale è un miglioramento sostanziale ma non un PASS formale.
La V9 elimina i difetti strutturali osservati nel V5:

- `51` Heel Strike, `50` Toe Off e `50` cicli validi;
- ordine HS–TO–HS esatto;
- zero campioni con entrambi i sensori OFF nel trasferimento heel→forefoot;
- zero candidati TO prima dei `300 ms`;
- zero transizioni invalide, timeout o eventi sensoriali non accettati;
- forefoot libero e detector riarmato almeno `31–33 ms` prima del successivo
  onset heel;
- TO massimo `32.10–35.10 ms`, ampiamente entro il limite di `80 ms`.

Rimane un solo limite formale: il miglior errore HS a `1 ms` è
`52.468852 ms`, contro il limite congelato di `50 ms`. Due dei 51 HS sono
fuori soglia:

- primo HS: `−52.468852 ms`, eccedenza `2.468852 ms`;
- terzo HS: `−50.480306 ms`, eccedenza `0.480306 ms`.

La V9 è quindi **FAIL secondo il protocollo**, benché il detector sia
operativamente molto vicino al gate. Nessun profilo è stato creato o promosso.
Il blocco sealed `100–155.045 s` è rimasto chiuso.

## Perché il detector resta semplice

Durante uno sweep il sampler OpenSim valuta in una sola passata le posizioni
uniche condivise fra più candidati. Per esempio, la V9 campiona nove sfere
detector uniche per confrontare otto candidati più due comparator. Questo è
soltanto un'ottimizzazione offline.

Ogni candidato è ricostruito e valutato come una coppia indipendente composta
da esattamente:

```text
1 heel sphere + 1 forefoot sphere
```

La mesh `AM_foot_l.STL` è letta soltanto offline per derivare coordinate locali
ragionevoli e applicare i pre-gate geometrici. Non entra nella FSM, non genera
eventi e non sostituisce i due sensori.

## Revisione del criterio di rilascio

Il V5 applicava come hard gate un margine di rilascio toe di `250 ms` rispetto
all'HS reference. L'audit ad alta risoluzione ha mostrato che tale criterio
mescolava due problemi distinti:

1. validità causale del detector;
2. foot-clearance/scuff fisico della traiettoria prescribed.

Il limite di `250 ms` non deriva dalla FSM ed equivale a oltre otto volte il
dwell di `30 ms`. È stato quindi sostituito, prima dei run V7–V9, da una misura
causale:

```text
accepted TO confirmed
    -> ultimo campione con toe latch attivo
    -> primo campione sicuramente toe-clear
    -> successivo accepted HS onset
```

Il gate richiede almeno `30 ms` continui di toe-clear prima del successivo
heel-onset. Gli eventuali ricontatti toe nello swing restano registrati come
diagnostica di foot-clearance, ma non vengono confusi con un errore del
detector.

Questa modifica riguarda solo il protocollo di validazione. Non cambia la FSM
di produzione.

## V5 di partenza e audit multirisoluzione

Il candidato V5 `H02/X2/F80/P35` sembrava pienamente valido a `10 ms`, ma il
replay diretto a `1 ms` ha rivelato:

| Metrica V5 | 10 ms | 1 ms |
|---|---:|---:|
| HS / TO / cicli | 51 / 50 / 50 | 51 / 50 / 50 |
| Errore HS massimo | 46.533 ms | 52.469 ms |
| Errore TO massimo | 42.532 ms | 37.100 ms |
| Campioni heel+toe OFF | 0 | 6 |
| Candidati TO precoci | 0 | 1 |
| Invalidi / non accettati | 0 / 0 | 1 / 1 |
| Clear causale prima di HS | 290 ms | 33 ms |

Il confronto dimostra che la sola validazione a `10 ms` nascondeva un gap di
`6 ms`. Da V6 in avanti tutti i candidati sono stati quindi valutati
direttamente a entrambe le risoluzioni, senza scegliere prima un solo winner a
`10 ms`.

Plot dedicato:

`plot/07_22_2026_two_sensor_timing_placement_v5_multiresolution/01_p35_10ms_vs_1ms_validation.png`

## Iterazione V6: errore del validation harness

La V6 aveva preregistrato otto posizioni con forefoot F76/F78 e tutti i replay
a `10/1 ms`. L'esecuzione si è arrestata prima della selezione per una chiave
tecnica mancante nel protocollo:

```text
KeyError: sealed_validation_gate
```

La V6 non produce quindi un risultato scientifico PASS/FAIL. È stata
conservata immutata come `ERROR_INVALID_HARNESS`; nessuna metrica è stata usata
per cambiare la griglia e il sealed non è stato aperto.

La V7 ha aggiunto soltanto la chiave ereditata mancante, mantenendo invariato
il contratto scientifico V6. Un test dedicato invoca realmente il gate
ereditato, impedendo la ricorrenza dello stesso errore.

## Iterazione V7: forefoot troppo posteriore

Griglia:

- heel offset `H2`;
- heel anteriore `X2.5/X3.0 mm`;
- forefoot `F76/F78`;
- protrusione `P34.75/P35`;
- otto candidati, sempre due sensori per candidato.

Esito: nessun winner multirisoluzione.

Il forefoot più posteriore riduce il gap iniziale, ma introduce un problema
peggiore nell'appoggio associato all'HS reference `98.179601 s`:

- il forefoot riconosce il nuovo appoggio prima dell'heel;
- il toe latch resta attivo fino all'heel-onset rilevato;
- il margine causale scende a `0 ms`;
- con X2.5 l'HS viene confermato con errore `+123.399 ms` a `1 ms`;
- con X3 viene perso un HS e rimangono 49 cicli.

Questa direzione è stata quindi respinta. Il problema non richiede più sensori:
richiede di non portare il singolo sensore forefoot troppo indietro.

Artefatti principali:

- manifest V7:
  `validation/two_sensor_timing_placement_sweep_runs/2026-07-22_ab06_50_100_multires_geometry_v7/manifest.json`;
- plot diagnostico leggibile:
  `plot/07_22_2026_two_sensor_timing_placement_v7_diagnostic/01_two_sensor_v7_multiresolution_diagnostic.png`.

## Iterazione V8: micro-tuning heel vicino al V5

La V8 ha riportato il forefoot vicino alla posizione V5 e ha variato soltanto
il sensore heel:

- heel offset `H2/H2.25 mm`;
- heel anteriore `X2.5/X2.75 mm`;
- forefoot `F79.5/F80`;
- `P35` fisso.

Il candidato più vicino al gate è:

```text
H2.25 / X2.75 / F79.5 / P35
```

Passa integralmente a `10 ms`. A `1 ms` conserva `51/50/50`, ma il solo
episodio iniziale produce:

- HS massimo `51.468852 ms`;
- gap heel→toe `3 ms`;
- un TO candidato prematuro;
- un evento invalido e uno non accettato;
- clear causale `33 ms`, ancora valido.

La V8 dimostra un trend locale utile: abbassare leggermente l'heel prolunga il
contatto e riduce il gap senza compromettere il rilascio toe.

## Iterazione V9: continuità risolta, timing marginalmente fuori gate

La V9 è stata preregistrata come ultima iterazione locale:

- heel offset `H2.5/H2.75 mm`;
- heel anteriore `X3.0/X3.25 mm`;
- forefoot `F79/F79.5`;
- `P35` fisso;
- otto candidati, tutti valutati direttamente a `10 ms` e `1 ms`.

Tutti gli otto candidati ottengono a entrambe le risoluzioni:

- `51 HS / 50 TO / 50 cicli`;
- zero gap heel+toe OFF;
- zero TO precoci;
- zero invalidi, timeout e non accettati;
- ordine esatto;
- clear causale `31–33 ms` a `1 ms`;
- TO massimo `32.10–35.10 ms`.

Non esiste tuttavia un winner strict perché ogni candidato supera di poco il
limite HS a `1 ms`.

Il miglior candidato diagnostico secondo il ranking congelato, ma **non
selezionato formalmente**, è:

```text
H2.50 / X3.25 / F79.0 / P35
```

Coordinate locali:

| Sensore | x [m] | y [m] | z [m] |
|---|---:|---:|---:|
| heel | -0.0981600475 | -0.0353195377 | +0.0139956700 |
| forefoot | +0.1157485850 | -0.0511292379 | +0.0030479022 |

Metriche:

| Metrica | 10 ms | 1 ms | Gate |
|---|---:|---:|---:|
| HS / TO / cicli | 51 / 50 / 50 | 51 / 50 / 50 | esatti |
| Errore HS massimo | 46.533 ms | **52.469 ms** | ≤ 50 ms |
| HS entro 50 ms | 51/51 | **49/51** | 51/51 |
| Errore TO massimo | 37.100 ms | 32.100 ms | ≤ 80 ms |
| Campioni heel+toe OFF | 0 | 0 | 0 |
| TO precoci | 0 | 0 | 0 |
| Invalidi / non accettati | 0 / 0 | 0 / 0 | 0 / 0 |
| F1 FSM | 0.96375 | 0.96537 | ≥ 0.95 |
| IoU FSM | 0.93004 | 0.93305 | ≥ 0.90 |
| Clear causale | 290 ms | 33 ms | ≥ 30 ms |
| Ricontatti toe | 0 | 1 | diagnostica |

Il solo check composito fallito a `1 ms` è
`imported_event_timing_phase_gate`, decomposto in:

- HS timing;
- precision e recall temporali, entrambe `0.960784` perché 49/51 HS sono entro
  tolleranza;
- un campione di mismatch di fase vietato, conseguenza dello stesso scarto
  temporale.

Tutti gli altri check strict sono PASS.

Plot:

`plot/07_22_2026_two_sensor_timing_placement_v9/01_timing_placement_v9_multiresolution.png`

## Interpretazione scientifica

Il posizionamento ha risolto la parte sostanziale del problema del detector:
conteggi, sequenza, continuità, assenza di falsi TO e riarmo causale. Il residuo
non è più una perdita di evento o una transizione FSM sbagliata, ma un margine
temporale di `2.469 ms` rispetto a un limite hard di `50 ms` su due eventi
prescribed.

Questo scarto può essere operativamente accettabile, soprattutto considerando
la risoluzione del replay e la rumorosità nota della reference GRF, ma non può
essere trasformato retroattivamente in un PASS. Per dichiararlo valido servono
una delle due decisioni seguenti, presa prima di nuovi dati:

1. mantenere il limite `50 ms` e rivedere il modello di contatto/reference del
   sensore invece di continuare il micro-tuning delle coordinate;
2. motivare fisicamente una tolleranza operativa leggermente più ampia, per
   esempio `55 ms`, congelarla e applicarla a un nuovo protocollo senza usare
   il sealed per scegliere la soglia.

Non è consigliato proseguire con micro-spostamenti ciechi: V8–V9 mostrano
plateau e cambi discreti del contatto. Continuare sulla stessa finestra
prescribed rischierebbe di adattare la geometria a due singoli HS anziché
validare un detector generalizzabile.

## Robustezza e runtime preservati

Non sono stati eseguiti training o rollout. Non sono stati modificati profili,
FSM, reward, policy, checkpoint o configurazioni runtime.

Hash verificati dopo i run:

```text
61ea948a...  current detector profile
09e04ab9...  primary GRF profile
847da698...  training_exnovo_cfg.yaml
ec726fdf...  prosthetic_phase_fsm.py
201cf8c4...  online_grf.py
50e4cc34...  simulation_runner.py
052270f7...  model_loader.py
e6bc3a1b...  Trajectory Generator/osim_trj_cmc_like.py
```

Il default `phase_fsm_input_mode: legacy_events` resta invariato. La robustezza
della policy ottenuta finora non è stata quindi cancellata o alterata.

## Gap runtime ancora aperto

Il replay offline separa correttamente:

- guardie HS/TO: due sensori candidati;
- `normal_force_bw`: aggregato primary GRF;
- `in_contact`: penetrazione fisica primary.

Nel runtime RL, invece, `Trajectory Generator/osim_trj_cmc_like.py`, funzione
`_phase_grf_sides()` intorno alla linea 2577, preferisce ancora l'aggregato del
profilo detector quando è presente. Questo è un problema preesistente: i due
sensori sono fisicamente `sensor-only`, ma il loro aggregato può ancora
alimentare `normal_force_bw/in_contact` della FSM runtime.

Per questo motivo anche un futuro PASS prescribed non sarà automaticamente
deployable. Prima della promozione servono:

1. correzione e test dedicato del routing runtime;
2. creazione esplicita di un profilo candidato, non sovrascrittura del current;
3. validazione sul blocco sealed;
4. rollout frozen-policy e gate di robustezza;
5. soltanto dopo, eventuale fine-tuning/training.

## File aggiunti

Validation harness e protocolli:

- `validation/sweep_two_sensor_timing_placements_prescribed_v6.py`;
- `validation/two_sensor_timing_placement_sweep_protocol_v6.json`;
- `validation/test_two_sensor_timing_placement_sweep_v6.py`;
- `validation/sweep_two_sensor_timing_placements_prescribed_v7.py`;
- `validation/two_sensor_timing_placement_sweep_protocol_v7.json`;
- `validation/test_two_sensor_timing_placement_sweep_v7.py`;
- `validation/sweep_two_sensor_timing_placements_prescribed_v8.py`;
- `validation/two_sensor_timing_placement_sweep_protocol_v8.json`;
- `validation/test_two_sensor_timing_placement_sweep_v8.py`;
- `validation/sweep_two_sensor_timing_placements_prescribed_v9.py`;
- `validation/two_sensor_timing_placement_sweep_protocol_v9.json`;
- `validation/test_two_sensor_timing_placement_sweep_v9.py`.

Plot diagnostici:

- `validation/plot_two_sensor_timing_placement_v5_multiresolution.py`;
- `validation/test_plot_two_sensor_timing_placement_v5_multiresolution.py`;
- `validation/plot_two_sensor_timing_placement_v7_diagnostic.py`;
- `validation/test_plot_two_sensor_timing_placement_v7_diagnostic.py`.

Run:

- `validation/two_sensor_timing_placement_sweep_runs/2026-07-22_ab06_50_100_multires_geometry_v6/`;
- `validation/two_sensor_timing_placement_sweep_runs/2026-07-22_ab06_50_100_multires_geometry_v7/`;
- `validation/two_sensor_timing_placement_sweep_runs/2026-07-22_ab06_50_100_heel_micro_v8/`;
- `validation/two_sensor_timing_placement_sweep_runs/2026-07-22_ab06_50_100_heel_micro_v9/`.

## Test e verifiche

Sono state eseguite in processi Python separati le suite relative a:

- FSM two-sensor e compatibilità legacy;
- path dati sensor-only;
- replay prescribed e forward states;
- geometria e profilo V4;
- placement sweep V1/V2;
- separazione dual-stream V1/V2;
- timing placement V3–V9;
- plot multirisoluzione V5 e diagnostico V7;
- audit geometrico prescribed.

Totale: `138` test passati.

Ulteriori verifiche:

- `py_compile`: PASS sui nuovi script;
- Ruff: PASS sui nuovi script;
- `git diff --check`: PASS;
- protocolli e validator V6–V9 hash-pinned;
- manifest V6 conservato come errore harness non scientifico;
- manifest V7–V9 immutabili dopo il rispettivo run;
- nessun campione `t >= 100 s` e sealed non aperto;
- nessuna modifica ai file di produzione o ai profili attivi.

Due suite che importano `validation` come package devono essere invocate con
`python -m unittest`; con tale invocazione entrambe sono PASS.

## TODO propagati

- decidere, prima di aprire nuovi dati, se conservare il gate HS `50 ms` o
  preregistrare una tolleranza operativa fisicamente motivata;
- non proseguire con tuning locale delle coordinate sullo stesso development
  senza una nuova ipotesi fisica;
- correggere il routing `normal_force_bw/in_contact` nel runtime RL e aggiungere
  un regression test dedicato;
- soltanto dopo un PASS prescribed creare un profilo candidato separato;
- mantenere chiuso il blocco sealed fino al congelamento di detector, routing e
  protocollo finale;
- dopo il sealed PASS, eseguire rollout frozen-policy e gate di robustezza prima
  di qualsiasi training.
