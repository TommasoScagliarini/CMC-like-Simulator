# Trial 08 V23 — FAIL per gap geometrico di 12 ms

Data: 2026-08-04

## Esito

V23 ha completato correttamente l'intero replay development del trial 08 e ha
prodotto una decisione terminale valida:

`FAIL_V23_TRIAL08_OPEN_DEVELOPMENT_REPLAY_TERMINAL`

Il risultato è un vero FAIL prestazionale del candidato
`v21_678b0b5162b706dd`, non un errore del validatore:

- 8 unità valutate: quattro plateau per due modalità di consumo;
- 6 unità PASS;
- 2 unità FAIL, entrambe relative a `plateau_03`, velocità 2,05 m/s;
- parità esatta fra processamento sequenziale 1 ms e batch 10 ms: PASS;
- gate dei due canali grezzi: PASS;
- binding FSM, stato terminale, oracle e coverage: PASS.

Il candidato non viene congelato né promosso. V23 è consumato e dichiara
`rerun_allowed=false`.

## Correzione procedurale V23

V23 ha corretto l'errore booleano V22 separando strutturalmente:

- `assertions`: invarianti positivi, tutti attesi `True` e soli partecipanti
  all'aggregazione del gate;
- `facts`: descrizioni che possono essere legittimamente `False`, compreso
  `global_grid_equality_required=false`.

Il binding reale dell'oracle è stato eseguito nel preflight, prima di aprire il
ledger. Dopo l'apertura sono stati riverificati soltanto gli hash; l'oracle non
è stato riletto, ricostruito, risogliato o ribindato.

Il freeze V23 ha SHA-256
`6a958e90dcc7370adfae64c8bd7970bce92d9af227c12bf6019a93ae422072fd`.

## Risultati per plateau

I risultati sono identici nelle due modalità di consumo:

| Plateau | Velocità | Cicli osservati/attesi | Min flight | F1 | IoU | Esito |
|---|---:|---:|---:|---:|---:|---|
| 01 | 0,85 m/s | 25/25 | 403 ms | 0,9919 | 0,9839 | PASS |
| 02 | 1,65 m/s | 33/33 | 389 ms | 0,9733 | 0,9481 | PASS |
| 03 | 2,05 m/s | 39/38 | 7 ms | 0,9584 | 0,9200 | FAIL |
| 04 | 1,25 m/s | 30/30 | 385 ms | 0,9871 | 0,9746 | PASS |

Nel plateau 03 F1 e IoU superano comunque le soglie. Il FAIL deriva dai gate
esatti di eventi, cicli e durata minima del flight:

- HS predetti/riferimento: 40/39;
- TO predetti/riferimento: 39/38;
- cicli osservati/riferimento: 39/38;
- un TO e un HS spurii;
- flight accettato minimo: 7 ms, inferiore al requisito di 30 ms.

## Causa del FAIL

La trace binaria mostra un solo intervallo `heel=false, toe=false` più corto di
100 ms nell'intero trial. Si trova durante il trasferimento tacco-punta di uno
stance reale:

- `99.840–99.881 s`: solo tallone attivo, 42 campioni;
- `99.882–99.893 s`: entrambi i sensori OFF, 12 campioni;
- da `99.894 s`: sola punta attiva.

La FSM V20, coerentemente con il debounce congelato di 5 ms, interpreta il
buco di 12 ms come:

- TO fisico a `99.882 s`, confermato a `99.887 s`, consegnato a `99.890 s`;
- HS fisico a `99.894 s`, confermato a `99.899 s`, consegnato a `99.900 s`.

Gli eventi oracle più vicini appartengono invece al ciclo normale:

- HS precedente a `99.662 s`;
- TO reale successivo a `100.168 s`.

Il passaggio OFF di 12 ms avviene quindi nel mezzo dello stance e crea un ciclo
fittizio. Poiché il segnale grezzo è già OFF/OFF prima che intervenga la FSM,
la causa primaria è una lacuna geometrica di copertura fra tallone e punta alla
velocità più alta. La FSM non introduce il buco: lo trasforma in eventi perché
esso dura più del debounce previsto.

La trace V23 conserva i bit ma non la clearance continua per campione. È quindi
dimostrato il dropout del segnale geometrico/kinematico, ma con questi soli
artefatti non è ancora possibile distinguere definitivamente fra placement o
reach dei due punti e compatibilità fra geometria marker e modello runtime.

## Interpretazione

Il risultato conferma che:

- la correzione oracle V23 funziona;
- il processamento scalar/batch è deterministico e identico;
- la geometria V21 funziona sui plateau 01, 02 e 04;
- la geometria V21 non garantisce continuità di contatto nel trasferimento
  tacco-punta a 2,05 m/s;
- il trial 08, ormai development aperto, ha rivelato un caso che non era
  presente nei trial 02/04 usati dallo sweep V21.

Non è possibile dichiarare il detector V21 pronto per H0 o per i trial
protetti.

## File introdotti da V23

- `validation/freeze_binary_phase_detector_v23_trial08_development.py`;
- `validation/validate_binary_phase_detector_v23_trial08_development.py`;
- `validation/test_binary_phase_detector_v23_trial08_development.py`;
- `validation/binary_phase_detector_v23_trial08_development_freeze_lock.json`;
- `validation/binary_phase_detector_v23_trial08_development_execution_ledger.json`;
- directory risultati
  `validation/binary_phase_detector_v23_development_runs/2026-08-04_trial08_assertions_facts_fix/`.

Nessun file V21/V22 è stato modificato.

## Test e verifiche

Prima del freeze:

- `py_compile`: PASS;
- 16/16 test V23: PASS, zero skip;
- regression test con oracle reale, assertions tutte vere e facts false: PASS;
- preflight OpenSim, plugin, modello, profilo e sampler: PASS;
- controllo no-clobber, PID, receipt e ordering: PASS;
- revisione indipendente pre-freeze: nessun blocker.

Dopo l'esecuzione:

- 12 JSON strict, senza NaN/Inf: verificati;
- 9 artifact del manifest: hash e dimensioni verificati;
- ledger e access receipt: byte-identici;
- packed binary trace: SHA interno verificato;
- nessun `failure.json`: la procedura è terminata regolarmente;
- manifest SHA-256:
  `333dc729a88e458a81038a9a90155e4cf08b68daed8aa1ee780be974a8b51f1f`;
- decision lock SHA-256:
  `de55bebbd9b1a21bf3aaadd7132cebed34554e14f5034797b159b14969a4510c`;
- ledger/receipt SHA-256:
  `c77dd856503d377570cf095882156e33057cd9071a04f255561d91e3a8e4645c`.

## Scope rimasto chiuso

- nessun H0 eseguito;
- nessun candidate lock o promozione;
- trial protetti 05/06 chiusi;
- riserve 03/07 chiuse;
- nessuna modifica alla GRF primaria, al plugin C++ o alla semantica SEA;
- nessun runtime/training/corridor/PPO avviato.

## Prossima decisione

La strada coerente con l'evidenza è un nuovo ciclo development geometrico,
usando 02/04/08 come dati ormai aperti e aggiungendo all'obiettivo un vincolo
esplicito: nessun intervallo OFF/OFF inferiore a 30 ms durante uno stance.

Prima dello sweep conviene eseguire una diagnostica mirata, sempre sul dato 08
già aperto, che persista le due clearance continue attorno a
`99.840–99.940 s` e attesti la compatibilità marker/runtime. Questo permette di
scegliere quale parametro geometrico correggere senza usare la FSM come
maschera del problema.

Una modifica della FSM che ignori flight inferiori a 30 ms è tecnicamente
possibile, ma cambierebbe il contratto temporale congelato e maschererebbe il
gap grezzo invece di correggere la copertura dei sensori. La scelta fra queste
due strade deve essere esplicita prima di iniziare un nuovo ciclo; 05/06 non
devono essere usati per il tuning.

## TODO

- **RISOLTO — V24:** scelta la correzione geometrica; le clearance continue e
  l'audit marker/runtime hanno escluso una divergenza fra i due modelli.
- **RISOLTO — V25:** eseguito lo sweep development 02/04/08 con vincolo di
  continuità; selezionato il candidato `v25_4b351f67b5b86ab0`, 24/24 unità
  PASS e zero gap interni.
- **DA FARE:** integrare V25 in shadow e completare il gate H0 prima di aprire
  05/06. Vedere
  `2026-08-04_detector_v24_v25_clearance_e_sweep_reach.md`.
