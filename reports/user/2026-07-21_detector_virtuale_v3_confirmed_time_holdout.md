# Detector virtuale V3: confirmed time e validazione holdout

Data: 2026-07-21

## Esito esecutivo

È stato implementato ed eseguito il protocollo V3 del detector virtuale a due
sensori senza training, policy o reward:

- `confirmed_time_s` è il timestamp autoritativo per matching, gate e ranking;
- `event_time_s` (onset del crossing) resta obbligatorio ma solo diagnostico
  rispetto al riferimento temporale;
- la reference di fase è costruita dagli stessi intervalli HS/TO prescribed
  validati dal filtro temporale GRF;
- il development `11.99-50 s` è PASS;
- la sensitivity development a `1 ms` è PASS;
- il holdout cronologico `50-100 s` è FAIL;
- il blocco sealed `100-155.045 s` non è stato aperto.

Il detector non può quindi essere dichiarato formalmente validato sul full-span
AB06. Il fallimento del holdout non è spiegato principalmente dal rumore GRF:
gli HS critici appartengono a contatti prescribed sostenuti. Il limite osservato
è l'interazione tra distribuzione locale toe-dominant, impulsi heel marginali,
dwell di 30 ms e campionamento runtime a 10 ms.

## Problema metodologico risolto dalla V3

La V2 confrontava il riferimento GRF con l'onset retrodatato del detector,
anche se la FSM riceve causalmente l'evento soltanto dopo il debounce. Il
candidato migliore aveva quindi:

- onset HS massimo: `72.96 ms`, limite `50 ms`;
- onset TO massimo: `92.52 ms`, limite `80 ms`;
- tempo confermato HS massimo: `42.96 ms`;
- tempo confermato TO massimo: `62.52 ms`.

La V3 rende esplicita la semantica fisicamente osservabile:

```text
primary_event_time_field    = confirmed_time_s
diagnostic_event_time_field = event_time_s
sensor_dwell_s              = 0.03
```

L'onset continua a essere usato internamente dalla FSM per debounce, durata,
anchor e controllo della latenza. È soltanto il suo errore rispetto alla GRF a
non poter più decidere selezione o PASS/FAIL temporale.

## Correzione della reference di fase

L'audit V2 ha trovato una contraddizione indipendente dal timestamp. Gli eventi
GRF richiedevano un contatto minimo di `50 ms`, mentre la fase reference era
calcolata campione per campione come `Fy > 20 N`. Due brevi riattraversamenti a
`48.08 s` e `48.14 s`, correttamente scartati dall'event extractor, diventavano
comunque campioni di stance e producevano due mismatch impossibili da eliminare
con una soglia del detector.

La V3 usa quindi una reference event-consistent congelata prima del holdout:

- stance da ogni HS prescribed accettato al TO successivo;
- swing dal TO all'HS successivo;
- crossing GRF più brevi di `50 ms` non creano fasi isolate;
- nessuna nuova tolleranza post-hoc è stata introdotta;
- usando `confirmed_time_s`, il dwell non viene sommato una seconda volta alla
  finestra di mismatch.

## Protocollo e protezioni anti-leakage

Il protocollo V3 è:

`validation/two_sensor_prescribed_threshold_sweep_protocol_v3.json`

SHA-256 eseguito:

```text
b8e580bf1b40849c4803adbaff19caa7bd862efdf13968c3fbe407435e365611
```

Il protocollo congela:

- griglia da 24 combinazioni low-force più baseline `5/2 N`;
- development `11.99-50 s`;
- validation `50-100 s`;
- sealed `100-155.045 s`;
- tolleranze `50 ms` HS e `80 ms` TO;
- dwell `30 ms`;
- timestamp primario e diagnostico;
- reference di fase;
- ranking e plateau;
- sorgenti dati, FSM e script di calcolo tramite hash.

Prima di aprire il holdout, il validator ha ricostruito deterministicamente la
selezione dal manifest development, controllando:

- protocol ID e hash;
- griglia completa e senza duplicati;
- corrispondenza ID/soglie di ogni candidato;
- numero minimo di cicli;
- selezione e plateau;
- sensitivity `1 ms` PASS;
- assenza di training e modifiche runtime.

Manifest development congelato:

```text
SHA-256 6a8d9f1904a1caee484c6bf269ac65fb04dd50ce89f594af8be6c419a4de5305
```

## Risultato development 11.99-50 s

Il development contiene `23` cicli completi. Il plateau stabile comprende otto
candidati con ON `0.50/0.75 N` e OFF `0.01/0.05/0.10/0.25 N`.

Il ranking preregistrato ha selezionato:

```text
ON    = 0.50 N
OFF   = 0.25 N
dwell = 0.03 s
ID    = on00p50_off00p25
```

### Gate primario a 10 ms

| Metrica | Risultato |
|---|---:|
| HS / TO / cicli | 24 / 23 / 23 |
| Precision / recall | 1.000 / 1.000 |
| Errore massimo HS confirmed | 42.96 ms |
| Errore massimo TO confirmed | 62.52 ms |
| F1 fase FSM | 0.97467 |
| IoU fase FSM | 0.95059 |
| Mismatch fase vietati | 0 |
| Invalidi / timeout | 0 / 0 |

### Sensitivity a 1 ms

| Metrica | Risultato |
|---|---:|
| HS / TO / cicli | 24 / 23 / 23 |
| Precision / recall | 1.000 / 1.000 |
| Errore massimo HS confirmed | 46.96 ms |
| Errore massimo TO confirmed | 65.52 ms |
| F1 fase FSM | 0.97398 |
| IoU fase FSM | 0.94927 |
| Mismatch fase vietati | 0 |

La sensitivity è stata eseguita soltanto dopo il lock del vincitore e soltanto
su vincitore più baseline; non poteva scegliere un candidato alternativo.

## Risultato holdout 50-100 s

Il holdout contiene `51` cicli reference. Sono stati valutati esclusivamente il
candidato bloccato e la baseline `5/2 N`.

### Gate complessivo

Il candidato produce:

| Metrica | Risultato |
|---|---:|
| HS reference / detector | 52 / 49 |
| TO reference / detector | 51 / 49 |
| Cicli completi osservati | 48 / 51 |
| F1 fase FSM | 0.93244 |
| IoU fase FSM | 0.87343 |
| Mismatch fase vietati | 212 campioni |
| Timeout | 1, swing a 98.85 s |

Poiché il gate primario a 10 ms è FAIL, la sensitivity holdout a 1 ms non è
stata eseguita, come preregistrato.

### Sequenza prima della perdita finale

Fino al reference HS a `98.179601 s`, la sequenza rimane cronologicamente
allineata con 49 HS e 49 TO. Tuttavia quattro HS precedenti sono confermati
troppo tardi:

| HS | Errore confirmed |
|---:|---:|
| #34 | +157.109 ms |
| #37 | +158.280 ms |
| #40 | +159.104 ms |
| #44 | +162.235 ms |

Gli altri 45 HS hanno errore massimo `36.533 ms`. Tutti i primi 49 TO sono
entro gli `80 ms`, con massimo assoluto `44.400 ms` e bias medio `-21.842 ms`.

Il holdout avrebbe quindi fallito anche senza il timeout finale, a causa dei
quattro HS tardivi indipendenti.

### Cascata terminale

La perdita finale deriva da un solo HS non confermato:

1. HS reference `98.179601 s`: nessun HS accettato;
2. TO reference `98.720176 s`: la FSM è ancora in `SWING_AFTER_TO`, quindi non
   può produrre un nuovo TO;
3. timeout swing a `98.850000 s`, `1.110 s` dopo l'onset dell'ultimo TO;
4. lo stato `TIMEOUT` resta latched;
5. diventano automaticamente mancanti HS `99.089744 s`, TO `99.616193 s` e HS
   `99.968787 s`.

I 212 mismatch di fase si decompongono in:

- 41 campioni associati ai quattro HS tardivi (`10+10+10+11`);
- 55 campioni tra l'HS mancato e il TO reference successivo;
- 116 campioni nello stato terminale `TIMEOUT`.

Non risultano eventi sensoriali rifiutati o transizioni invalide ordinarie;
l'unica transizione problematica registrata è il timeout.

## Diagnostica fisica dei cinque HS critici

I cinque eventi critici non sono micro-spike GRF:

- la GRF prescribed resta sopra `20 N` per almeno i 50 ms successivi;
- i contatti reference durano `526-556 ms`;
- il carico virtuale è invece inizialmente prevalentemente sull'avampiede.

| HS reference [s] | Heel [N] | Toe [N] |
|---:|---:|---:|
| 83.712891 | 0.048 | 16.242 |
| 86.461720 | 1.036 | 13.558 |
| 89.150896 | 1.476 | 3.572 |
| 92.727765 | 0.937 | 19.853 |
| 98.179601 | circa 0 | 42.363 |

Le prime finestre con heel `>=0.5 N` durano rispettivamente circa
`27/35/36/28/26 ms`.

- Gli impulsi da 27, 28 e 26 ms sono realmente più brevi del dwell di 30 ms e
  vengono correttamente scartati.
- Gli impulsi fisici da 35-36 ms falliscono a 10 ms per discretizzazione: la
  griglia runtime osserva solo tre campioni utili, quindi 20 ms trascorsi, prima
  che la forza ricada sotto soglia.
- La FSM era correttamente armata e in swing; il toe viene registrato come
  `forefoot_first`.
- Nei quattro casi tardivi l'HS viene accettato soltanto al successivo contatto
  heel stabile. A `98.179601 s` neppure il secondo impulso rimane sopra soglia
  per il dwell richiesto e non viene prodotto alcun HS.

Questa evidenza distingue il problema dalla semplice rumorosità EPIC: il
riferimento total-GRF indica contatti sostenuti, mentre il detector richiede un
carico locale heel persistente che in questi cicli non esiste abbastanza a
lungo al passo runtime.

## Modifiche implementate

File di validazione modificati o aggiunti:

- `validation/two_sensor_prescribed_threshold_sweep_protocol_v3.json`;
- `validation/sweep_two_sensor_prescribed_thresholds.py`;
- `validation/validate_two_sensor_prescribed_replay.py`;
- `validation/test_two_sensor_prescribed_threshold_sweep.py`;
- `validation/test_two_sensor_prescribed_phase_gate.py`.

Artefatti V3:

- `validation/two_sensor_prescribed_threshold_sweep_runs/2026-07-21_fullspan_v3_confirmed/development/manifest.json`;
- `validation/two_sensor_prescribed_threshold_sweep_runs/2026-07-21_fullspan_v3_confirmed/development/primary_candidate_metrics.csv`;
- `validation/two_sensor_prescribed_threshold_sweep_runs/2026-07-21_fullspan_v3_confirmed/development/selected_primary_event_timing.png`;
- `validation/two_sensor_prescribed_threshold_sweep_runs/2026-07-21_fullspan_v3_confirmed/validation/manifest.json`;
- `validation/two_sensor_prescribed_threshold_sweep_runs/2026-07-21_fullspan_v3_confirmed/validation/primary_candidate_metrics.csv`;
- `validation/two_sensor_prescribed_threshold_sweep_runs/2026-07-21_fullspan_v3_confirmed/validation/selected_primary_event_timing.png`.

Nessuna modifica è stata applicata a:

- checkpoint o actor;
- training/PPO;
- reward;
- semantica dei SEA;
- soglie runtime checked-in;
- default `legacy_events`;
- dati GRF o cinematica prescribed.

## Test e verifiche

Sono stati eseguiti:

- 21/21 test unitari dello sweep V3: PASS;
- 7/7 test del gate di fase: PASS;
- 32/32 test threshold + FSM two-sensor: PASS;
- `py_compile`: PASS;
- `ruff`: PASS;
- validazione JSON: PASS;
- pinning SHA-256 di dati, FSM e script: PASS;
- ricostruzione deterministica del prerequisite development: PASS;
- development OpenSim 10 ms: PASS;
- development sensitivity 1 ms: PASS;
- holdout OpenSim 10 ms: FAIL scientifico, correttamente persistito;
- holdout sensitivity 1 ms: non aperta dopo FAIL primario;
- sealed: non aperto.

I warning OpenSim relativi alle mesh `.vtp/.stl` assenti riguardano soltanto la
visualizzazione e non invalidano il calcolo dinamico usato dal replay.

## Stato e TODO

Stato corretto da riportare:

> Il detector V3 è validato sul development AB06 e sulla relativa sensitivity,
> ma non è validato sul holdout cronologico 50-100 s. Il sealed resta chiuso.

TODO per un eventuale protocollo successivo, da discutere prima di altre
modifiche:

1. scegliere se il debounce sensoriale debba funzionare a una frequenza più
   alta della policy/FSM oppure essere ridefinito esplicitamente per il passo
   runtime di 10 ms;
2. verificare se un breve ma reale impulso heel seguito da forte carico toe
   debba essere riconosciuto come HS dal design fisico dei due sensori;
3. utilizzare development e il holdout V3 ormai aperto soltanto come dati di
   progetto per una futura V4, senza reinterpretarli come nuova validazione;
4. non aprire `100-155.045 s` finché un nuovo metodo e un nuovo protocollo non
   siano congelati e autorizzati esplicitamente;
5. mantenere separata la successiva verifica della robustezza policy: nessun
   nuovo training è necessario per diagnosticare o progettare il detector, ma
   ogni modifica runtime approvata richiederà poi rollout A/B sul checkpoint
   congelato e, solo se necessario, adattamento della policy.

