# Detector V9: startup heel-only e baseline sperimentale

Data: 2026-07-22

## Esito esecutivo

È stata implementata e validata una nuova regola di inizializzazione del
detector protesico a due sensori. Se un episodio inizia con il tallone già
caricato, la FSM non scarta più automaticamente il primo Heel Strike. Il nuovo
contratto distingue la configurazione iniziale dei due sensori:

| Heel | Toe | Interpretazione all'avvio |
|---:|---:|---|
| 0 | 0 | piede libero; attesa del normale HS |
| 1 | 0 | candidato HS iniziale |
| 1 | 1 | stance già avviata; bootstrap parziale |
| 0 | 1 | stance terminale; bootstrap parziale |

Il pattern `heel=1, toe=0` deve essere stabile per l'intero dwell di `30 ms`.
Solo dopo tale conferma viene accettato un HS; il suo `event_time_s` rimane il
primo campione heel-on/toe-off disponibile, mentre il tempo causale di conferma
è successivo di `30 ms`.

La modifica risolve il difetto osservato nel replay congelato:

- prima: `3 HS`, `3 TO`, `2` cicli completi;
- dopo: `4 HS`, `3 TO`, `3` cicli completi;
- eventi invalidi: `0`;
- timeout: `0`;
- traiettoria served, azioni e stati rappresentati: invariati esattamente.

La validazione deve però essere interpretata su due livelli distinti:

1. **Regola di startup: PASS.** L'HS iniziale viene recuperato correttamente a
   10 ms e 1 ms, senza falsi HS negli avvii heel+toe caricati.
2. **Timing complessivo del profilo V9: FAIL secondo il gate formale.** Nel
   tratto prescribed corrispondente all'episodio, gli HS successivi anticipano
   il riferimento fino a circa `80–82 ms`, oltre il limite di `50 ms`.

V9 viene pertanto congelata come **baseline sperimentale di confronto**, non
come detector definitivo o già promosso per il deployment.

## Problema affrontato

Il replay usato per il morphology corridor comincia in corrispondenza di un HS
prescribed. Nel primo stato nativo disponibile il sensore V9 del tallone è già
caricato:

```text
t iniziale nativo = 13.946870984 s
heel              = 36.49 N circa
toe               = 0 N circa
```

La versione precedente della FSM inizializzava i latch come scarichi ma non
armati. Dopo il debounce, qualunque primo contatto già presente veniva
classificato come `partial_stance_bootstrap`. Questa scelta era conservativa
per un reset a metà stance, ma perdeva il vero HS posto esattamente al bordo
iniziale dell'episodio.

Il detector legacy disponeva invece dell'evento prescritto al confine e
contava:

```text
HS(t0) -> TO -> HS -> TO -> HS -> TO -> HS
```

cioè quattro HS delimitanti e tre cicli completi. Il replay V9 precedente
iniziava dal primo TO e poteva chiudere soltanto due cicli.

L'obiettivo non era inserire l'HS legacy nel nuovo detector, né introdurre un
pre-roll obbligatorio. Il requisito era permettere ai soli due sensori di
classificare in modo semplice e dichiarato un contatto heel-only già presente
all'inizializzazione.

## Soluzione implementata

### Pattern congiunto e margine temporale

La FSM esistente mantiene un timer dedicato al pattern grezzo congiunto:

```text
heel_normal_force >= sensor_on_threshold
toe_normal_force  <= sensor_off_threshold
```

Il timer:

- parte soltanto nello stato iniziale `WAIT_HS`;
- è attivo soltanto prima che il detector sia armato o abbia risolto il
  bootstrap;
- viene azzerato immediatamente se il toe si carica, il heel si scarica o il
  pattern diventa ambiguo;
- usa lo stesso `sensor_dwell_s` già applicato ai latch, senza introdurre un
  nuovo parametro;
- produce al massimo un evento di startup per episodio.

La verifica congiunta è più robusta del semplice controllo dei latch. Per
esempio, se l'episodio parte in configurazione `10` ma il toe passa a `1` prima
della fine del dwell, il candidato HS viene annullato e la FSM esegue un
bootstrap di stance parziale.

### Evento diagnostico

L'HS iniziale accettato conserva la normale semantica della FSM e aggiunge
soltanto diagnostica:

```text
startup_contact = 1
startup_pattern = "heel_on_toe_off"
```

Il flag è presente nel candidato sensoriale e nella transizione accettata di
startup. Non viene aggiunto alle transizioni ordinarie o legacy, così il journal
preesistente conserva lo stesso schema nei percorsi non interessati.

Il timestamp ha una semantica esplicita:

- per un normale HS: onset fisico del fronte di contatto osservato;
- per l'HS di startup: primo campione disponibile con pattern heel-on/toe-off;
- per entrambi: evento disponibile causalmente soltanto dopo il dwell.

### Nessuna nuova macchina a stati

La modifica vive dentro `ProstheticPhaseFSM` e non introduce una seconda FSM.
Restano invariati:

- stati `WAIT_HS`, `STANCE_AFTER_HS`, `SWING_AFTER_TO` e `TIMEOUT`;
- handler HS/TO;
- gate di durata, contatto, carico ed escursione cinematica;
- timeout e clawback;
- schema dell'osservazione actor;
- plugin e semantica SEA;
- reward;
- generazione delle GRF.

I due sensori restano esclusivamente guardie di evento. Nella validazione
prescribed, `normal_force_bw` e `in_contact` provengono dalla GRF prescribed
separata, non dalla somma dei due sensori.

### Compatibilità legacy e shadow

`legacy_events` resta il default nella configurazione di training. Il nuovo
ramo non viene eseguito in tale modalità.

In `shadow` il candidato heel-only di startup è disponibile come diagnostica,
ma soltanto l'evento legacy modifica lo stato attivo. Un test confronta
l'intera sequenza legacy e la stessa sequenza in shadow e verifica identità di:

- osservazione actor;
- stato e prossimo evento atteso;
- conteggi HS, TO e cicli;
- tempi degli ultimi eventi;
- crediti;
- transizioni accettate.

## Strategia di validazione

### 1. Test sintetici della FSM

Sono stati coperti esplicitamente i casi:

- startup `00`: nessun evento, normale arming dopo il dwell;
- startup `10` stabile: un solo HS dopo il dwell;
- mantenimento di `10`: nessuna duplicazione;
- impulso heel più breve del dwell: nessun HS;
- passaggio `10 -> 11` prima del dwell: nessun falso HS;
- startup `01`: bootstrap parziale, nessun HS;
- startup `11`: bootstrap parziale, nessun HS;
- normale sequenza piede libero -> heel: comportamento invariato;
- toe-only seguito da heel: il toe non genera HS, il heel successivo sì;
- contatto simultaneo dopo normale arming: un solo HS;
- shadow diagnostico senza interferenza;
- shadow e legacy attivi sulla stessa sequenza: risultati FSM identici.

Esito:

```text
17 / 17 test FSM PASS
```

### 2. Replay congelato del checkpoint best

Il detector V9 è stato ricalcolato offline sugli stati completi del rollout
congelato del checkpoint best. Le traiettorie della policy non sono state
rigenerate né modificate.

Sequenza accettata:

| Evento | Onset/anchor [s] | Conferma [s] | Startup |
|---|---:|---:|---:|
| HS | 13.956871 | 13.986871 | sì |
| TO | 15.126871 | 15.156871 | no |
| HS | 15.646871 | 15.676871 | no |
| TO | 16.636871 | 16.666871 | no |
| HS | 17.116871 | 17.146871 | no |
| TO | 18.286871 | 18.316871 | no |
| HS | 18.696871 | 18.726871 | no |

Il primo timestamp del plot è `10 ms` successivo al confine nativo perché il
trace policy salvato comincia a `13.956871 s`. Il detector usa quindi il primo
campione realmente disponibile in quella traccia; non copia l'evento legacy.

Cicli ricostruiti:

1. `13.956871 -> 15.126871 -> 15.646871 s`;
2. `15.646871 -> 16.636871 -> 17.116871 s`;
3. `17.116871 -> 18.286871 -> 18.696871 s`.

Invarianti del trace:

| Metrica | Risultato |
|---|---:|
| righe confrontate | 500 |
| errore massimo tempo | 0 |
| errore massimo stato protesico | 0 |
| errore massimo azione | 0 |
| campi plottati esattamente identici | sì |

### 3. Replay prescribed dedicato

È stato creato un validatore separato che non aggiorna gli hash dei protocolli
V6–V9 storici. Il primo HS viene derivato programmaticamente dalla GRF
prescribed tramite il parser già validato; nessun timestamp di evento è
hardcoded.

Il replay parte esattamente da:

```text
13.946870984 s
```

e copre cinque secondi a due cadenze:

- `10 ms`, corrispondente alla cadenza runtime attuale;
- `1 ms`, come controllo ad alta risoluzione.

Risultati della regola di startup a entrambe le cadenze:

| Gate | 10 ms | 1 ms |
|---|---:|---:|
| pattern heel-on/toe-off stabile per il dwell | PASS | PASS |
| candidati HS startup | 1 | 1 |
| HS startup accettati | 1 | 1 |
| errore onset rispetto all'oracolo | 0 ms | 0 ms |
| latenza di conferma | 30 ms | 30 ms |
| HS / TO / cicli | 4 / 3 / 3 | 4 / 3 / 3 |
| invalidi / timeout / candidati non accettati | 0 / 0 / 0 | 0 / 0 / 0 |

La coerenza multirisoluzione passa:

| Evento | Differenza massima 10 ms vs 1 ms |
|---|---:|
| HS | 2 ms |
| TO | 3 ms |

### 4. Controllo anti-falso-positivo a setup start

Il setup prescribed originale comincia a `11.99 s` con entrambi i sensori
caricati:

```text
heel = 78.11 N circa
toe  = 28.36 N circa
```

A 10 ms e 1 ms la FSM:

- entra nel bootstrap di stance parziale;
- non produce candidati HS startup;
- non accetta HS;
- non genera invalidi.

Questo controllo dimostra che la nuova regola non trasforma genericamente ogni
piede caricato all'avvio in un Heel Strike: è richiesto il pattern spaziale
specifico heel-on/toe-off.

### 5. Timing prescribed complessivo

Il gate temporale è stato mantenuto rigoroso anche se il cambiamento riguarda
soltanto lo startup:

```text
HS massimo ammesso = 50 ms
TO massimo ammesso = 80 ms
```

Risultati:

| Metrica | 10 ms | 1 ms | Gate |
|---|---:|---:|---:|
| errore HS massimo | 80.073 ms | 82.073 ms | 50 ms |
| errore TO massimo | 39.978 ms | 40.978 ms | 80 ms |
| timing HS | FAIL | FAIL | — |
| timing TO | PASS | PASS | — |

Gli HS successivi sono anticipati. L'HS iniziale introdotto dalla nuova regola
ha invece errore nullo. Il FAIL deriva quindi dal posizionamento/contatto V9
sugli eventi successivi e non dalla classificazione di startup.

Questo risultato non contraddice il precedente sweep V9 su `50–100 s`, dove il
miglior errore a 1 ms era circa `52.47 ms`: il nuovo replay usa una finestra
diversa, `13.95–18.95 s`, e mostra che la temporizzazione V9 non generalizza
ancora entro 50 ms su tutti i tratti AB06 osservati.

## Decisione su V9

V9 viene congelata come **baseline sperimentale ufficiale** per confrontare i
futuri miglioramenti. Non deve essere sovrascritta o reinterpretata come PASS.

Contratto congelato:

- candidato: `H2p50_X3p25_F79p0_P35p00`;
- esattamente una sfera heel e una forefoot;
- soglie diagnostiche V9 `0.5/0.25 N`;
- dwell `30 ms`;
- regola startup heel-on/toe-off;
- sensori detector-only;
- GRF primaria separata per carico e contatto della FSM;
- tre cicli nell'episodio congelato;
- zero invalidi e timeout;
- timing prescribed misurato e conservato, incluso il FAIL HS.

Coordinate della baseline:

| Sensore | Posizione locale [m] | Raggio [m] |
|---|---|---:|
| heel | `[-0.09816005, -0.03531954, 0.01399567]` | 0.02290536 |
| forefoot | `[0.11574859, -0.05112924, 0.00304790]` | 0.02290536 |

Stato formale:

```text
startup_rule_gate                  = PASS
downstream_prescribed_timing_gate = FAIL
V9 baseline status                = EXPERIMENTAL_NOT_PROMOTED
```

Le revisioni future dovranno usare un nuovo identificatore, per esempio V10,
e confrontarsi con gli stessi dati, soglie e gate senza modificare gli artifact
V9.

## Robustezza e impatto sul checkpoint

La robustezza già ottenuta dal checkpoint non viene cancellata nel percorso
attuale perché:

- `legacy_events` resta il default;
- lo schema actor è invariato;
- il journal legacy non riceve nuovi campi;
- il replay frozen conserva esattamente traiettorie served e azioni;
- non è stato eseguito training;
- non sono stati modificati actor, critic o checkpoint.

Questo non equivale a dimostrare la deployability closed-loop del checkpoint
con `two_sensor` attivo. La promozione del detector e l'eventuale adattamento
della policy restano fasi successive, da affrontare soltanto dopo il
superamento del gate temporale del detector.

## File modificati

### Codice di produzione

- `Trajectory Generator/prosthetic_phase_fsm.py`
  - timer congiunto heel-on/toe-off allo startup;
  - HS iniziale debounced;
  - diagnostica `startup_contact`;
  - compatibilità del journal ordinario e legacy.

### Test e validazione

- `validation/test_prosthetic_phase_fsm_two_sensor.py`
  - casi `00`, `10`, `11`, `01`, transitorio e `10 -> 11`;
  - prevenzione duplicati;
  - confronto legacy/shadow.
- `validation/validate_two_sensor_startup_hs.py`
  - replay prescribed a 10 ms e 1 ms;
  - oracolo derivato dalla GRF;
  - separazione fra startup gate e timing V9;
  - controllo setup-start heel+toe.
- `validation/test_two_sensor_startup_hs.py`
  - contratto V9 a due sfere;
  - griglie temporali;
  - crop degli eventi;
  - fail-closed e no-clobber.
- `validation/plot_morphology_corridor_two_sensor_v9.py`
  - rendering dell'HS di startup;
  - tre cicli completi;
  - semantica esplicita del primo campione;
  - hash del codice FSM e dello script nel sidecar.
- `validation/test_plot_morphology_corridor_two_sensor_v9.py`
  - contratto detector, allineamento e generazione plot.

## Artifact prodotti

- plot aggiornato:
  `plot/07_22_2026_morphology_two_sensor_v9_experimental/01_morphology_corridor_two_sensor_v9_hs_to_hs.png`;
- sidecar del plot:
  `plot/07_22_2026_morphology_two_sensor_v9_experimental/01_morphology_corridor_two_sensor_v9_hs_to_hs.json`;
- report machine-readable prescribed:
  `validation/two_sensor_startup_hs_runs/2026-07-22_ab06_v9_prescribed/summary.json`;
- profilo detector usato:
  `validation/experimental_detector_profiles/two_sensor_v9_H2p50_X3p25_F79p0_P35p00.json`.

Hash finali principali:

| Artifact | SHA-256 |
|---|---|
| plot PNG | `df66e9638476d027ce26621afa62dcbd2a7220da49e9f9ed100e0dfcb21e4e4f` |
| sidecar JSON | `a24bb8e2c21f4b06cf75a9aeb42dd36d4a0667cf2bc3a81a18640e0830f63a1d` |
| summary prescribed | `e12efdebec4610a570bd34cc5850de94d4e28f2a87bd4ebb90d0ee9c1237d7c9` |

## Test e verifiche eseguite

Sono passati:

- `17` test diretti della FSM a due sensori;
- `7` test del nuovo validator startup;
- `23` ulteriori test su plot, data path e core online GRF;
- `32` test della reward;
- `19` test morphology retrospettivi;
- `11` test morphology event-anchored;
- `4` test del replay del journal FSM;
- `py_compile` sui file coinvolti;
- Ruff sui file coinvolti;
- `git diff --check`.

Totale dei test rilevanti eseguiti con esito positivo: `113`.

I quattro test del vecchio protocollo V9 hash-pinned segnalano intenzionalmente
`source hash drift` dopo la modifica della FSM. Gli hash storici non sono stati
aggiornati: tale rifiuto preserva la riproducibilità degli sweep archiviati e
non rappresenta un errore inatteso del nuovo validator.

## Limiti attuali

- Il primo HS del plot congelato usa il primo campione policy disponibile,
  `10 ms` dopo il confine nativo.
- Il timing HS V9 non supera il gate di `50 ms` sulla finestra iniziale AB06.
- V9 non è promossa nel file di training.
- Non è stato eseguito un nuovo rollout closed-loop con `two_sensor` attivo.
- Non è stata dimostrata la deployability del checkpoint con il nuovo detector.
- La baseline deriva ancora dal modello virtuale AB06 e non costituisce una
  specifica hardware dei sensori reali.

## TODO

1. Creare V10 come ramo separato, senza sovrascrivere V9.
2. Ridurre l'errore massimo HS sotto `50 ms` sia sulla finestra iniziale
   `13.95–18.95 s` sia sul development `50–100 s`.
3. Conservare almeno le prestazioni V9 su TO, conteggi, ordine, clear causale,
   invalidi e coerenza 10 ms/1 ms.
4. Mantenere esattamente due sensori detector-only e la GRF primaria separata.
5. Aprire validation/holdout e sealed soltanto dopo una selezione development
   preregistrata che superi i gate.
6. Valutare la compatibilità closed-loop del checkpoint soltanto dopo la
   promozione del detector; decidere allora se sia necessario un fine-tuning.
7. Non cambiare `phase_fsm_input_mode` da `legacy_events` finché il detector non
   supera timing, robustezza e gate di promozione completi.
