# Daily Report - 2026-07-22

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

- `reports/user/2026-07-22_iterazioni_posizionamento_detector_due_sensori_v6_v9.md`
- `reports/user/2026-07-22_detector_v9_startup_heel_only_baseline_sperimentale.md`

## Sintesi

La giornata ha completato le iterazioni V6-V9 sul posizionamento dei due
sensori e ha risolto il caso limite in cui l'episodio inizia con il tallone
gia' caricato e il toe scarico.

La V9 conserva il sistema semplice richiesto: una sfera heel e una sfera
forefoot, entrambe detector-only. Sul blocco `50-100 s` ottiene conteggi,
ordine, continuita' heel-forefoot e clear causale corretti a `10 ms` e `1 ms`.
Rimane pero' `2.469 ms` oltre il gate HS da `50 ms` nel caso peggiore a `1 ms`.

La nuova regola di startup recupera correttamente il primo HS del rollout
congelato e permette di ricostruire tre cicli completi invece di due. La regola
di startup e' PASS, ma il profilo V9 fallisce ancora il timing HS complessivo
sulla finestra iniziale AB06, con errori massimi `80-82 ms`.

V9 e' stata quindi congelata come baseline sperimentale ufficiale per confronti
futuri, non promossa come detector definitivo. `legacy_events` resta il default.
Non e' stato eseguito training e non e' stato svolto un rollout active
closed-loop con V9 autoritativa.

## 1. Vincoli mantenuti

Tutte le iterazioni hanno mantenuto:

- una sfera virtuale heel e una forefoot per candidato;
- nessuna mesh complessa nel detector runtime;
- sensori usati solo come guardie HS/TO;
- GRF primaria separata nel replay prescribed per `normal_force_bw` e
  `in_contact`;
- soglie `0.5/0.25 N`;
- dwell `30 ms`;
- FSM, handler, timeout e minimo stance `300 ms` invariati;
- schema actor, reward, policy, checkpoint e plugin SEA invariati;
- sealed `100-155.045 s` chiuso.

La mesh e' stata usata soltanto offline per derivare posizioni locali
ragionevoli e applicare pre-gate geometrici. Non genera eventi o GRF.

## 2. Audit multirisoluzione del V5

Il candidato V5 sembrava valido a `10 ms`, ma il replay a `1 ms` ha mostrato un
gap di contatto che la cadenza runtime nascondeva:

| Metrica V5 | 10 ms | 1 ms |
| --- | ---: | ---: |
| HS / TO / cicli | `51/50/50` | `51/50/50` |
| errore HS massimo | `46.533 ms` | `52.469 ms` |
| errore TO massimo | `42.532 ms` | `37.100 ms` |
| campioni heel+toe OFF | 0 | 6 |
| candidati TO precoci | 0 | 1 |
| invalidi / non accettati | `0/0` | `1/1` |
| clear causale prima di HS | `290 ms` | `33 ms` |

Da V6 in avanti ogni candidato e' stato quindi valutato direttamente a
entrambe le cadenze, senza selezione preliminare a `10 ms`.

Il vecchio gate di release toe da `250 ms` e' stato inoltre sostituito con una
misura causale: dopo il TO accettato il toe deve restare libero per almeno il
dwell di `30 ms` prima del successivo onset heel. La modifica riguarda il
protocollo, non la FSM di produzione.

## 3. Iterazioni V6-V8

### V6

La V6 si e' arrestata prima della selezione per una chiave mancante nel
protocollo:

```text
KeyError: sealed_validation_gate
```

Il run e' stato conservato come `ERROR_INVALID_HARNESS`; non ha prodotto un
PASS/FAIL scientifico e non ha aperto il sealed. La V7 ha corretto soltanto
l'harness e ha aggiunto un test che invoca realmente il gate ereditato.

### V7

Il forefoot piu' posteriore ha introdotto contatto toe fino al successivo heel
onset. Il margine causale e' sceso a zero; un candidato ha ritardato l'HS di
`123.399 ms`, un altro ha perso un HS. Questa direzione e' stata respinta.

### V8

Il micro-tuning del heel ha prodotto un candidato vicino al gate:

```text
H2.25 / X2.75 / F79.5 / P35
```

A `10 ms` il candidato passa integralmente. A `1 ms` conserva `51/50/50`, ma
ha HS massimo `51.469 ms`, un gap heel-toe di `3 ms`, un TO candidato precoce,
un evento invalido e uno non accettato. La direzione ha mostrato che abbassare
leggermente il heel aumenta la continuita', ma non e' ancora promovibile.

## 4. V9: continuita' risolta, timing ancora fuori gate

La V9 ha valutato otto combinazioni heel/forefoot a entrambe le risoluzioni.
Tutti gli otto candidati hanno ottenuto:

- `51 HS / 50 TO / 50 cicli`;
- ordine HS-TO-HS esatto;
- zero gap con heel e toe entrambi OFF;
- zero TO precoci;
- zero invalidi, timeout o eventi non accettati;
- clear causale `31-33 ms` a `1 ms`;
- TO massimo `32.10-35.10 ms`.

Il miglior candidato diagnostico secondo il ranking congelato e':

```text
H2.50 / X3.25 / F79.0 / P35
```

| Metrica | 10 ms | 1 ms | Gate |
| --- | ---: | ---: | ---: |
| HS / TO / cicli | `51/50/50` | `51/50/50` | esatti |
| errore HS massimo | `46.533 ms` | `52.469 ms` | `<=50 ms` |
| HS entro gate | `51/51` | `49/51` | `51/51` |
| errore TO massimo | `37.100 ms` | `32.100 ms` | `<=80 ms` |
| heel+toe OFF | 0 | 0 | 0 |
| invalidi / non accettati | `0/0` | `0/0` | `0/0` |
| F1 / IoU FSM | `0.96375/0.93004` | `0.96537/0.93305` | PASS |
| clear causale | `290 ms` | `33 ms` | `>=30 ms` |

Il solo gate composito fallito a `1 ms` e' il timing HS. Due eventi anticipano
la reference di `52.469 ms` e `50.480 ms`, cioe' rispettivamente `2.469 ms` e
`0.480 ms` oltre il limite.

Il risultato e' operativamente vicino al gate, ma non e' stato reinterpretato
post-hoc come PASS. Non e' stato creato un profilo runtime promosso e il sealed
non e' stato aperto.

## 5. Regola heel-only allo startup

Il replay congelato inizia in corrispondenza di un HS prescribed, con heel gia'
caricato e toe scarico. La FSM precedente trattava ogni contatto presente al
primo campione come bootstrap parziale e perdeva quindi il primo ciclo.

E' stata aggiunta alla `ProstheticPhaseFSM` esistente una regola semplice e
debounced:

| Heel | Toe | Interpretazione iniziale |
| ---: | ---: | --- |
| 0 | 0 | piede libero, normale attesa HS |
| 1 | 0 | candidato HS iniziale |
| 1 | 1 | stance gia' avviata, bootstrap parziale |
| 0 | 1 | stance terminale, bootstrap parziale |

Il pattern heel-on/toe-off deve persistere per lo stesso dwell di `30 ms`. Il
timer viene annullato se il toe si carica, il heel si scarica o il pattern
diventa ambiguo. Puo' produrre al massimo un evento startup per episodio.

L'evento accettato registra:

```text
startup_contact = 1
startup_pattern = heel_on_toe_off
```

Non e' stata creata una nuova macchina a stati. Handler, gate, reward, schema
actor e generazione GRF restano invariati.

## 6. Validazione della regola startup

I test sintetici coprono startup `00`, `10`, `11`, `01`, impulsi brevi,
transizione `10 -> 11`, toe-only, contatto simultaneo, shadow e compatibilita'
legacy. Esito: `17/17 PASS`.

Sul replay frozen del checkpoint best:

| Sequenza | Prima | Dopo |
| --- | ---: | ---: |
| HS | 3 | 4 |
| TO | 3 | 3 |
| cicli completi | 2 | 3 |
| invalidi / timeout | `0/0` | `0/0` |

Il primo HS viene ancorato al primo campione policy disponibile e confermato
`30 ms` dopo. Traiettorie served, stati, azioni e tutti i 500 campioni
confrontati restano esattamente invariati.

Il replay prescribed dedicato parte da `13.946870984 s` e usa un oracolo
derivato dalla GRF, non timestamp hardcoded:

| Gate | 10 ms | 1 ms |
| --- | ---: | ---: |
| HS startup candidati/accettati | `1/1` | `1/1` |
| errore onset startup | `0 ms` | `0 ms` |
| latenza conferma | `30 ms` | `30 ms` |
| HS / TO / cicli | `4/3/3` | `4/3/3` |
| invalidi / timeout / non accettati | `0/0/0` | `0/0/0` |

La differenza multirisoluzione massima e' `2 ms` per HS e `3 ms` per TO. Il
setup originale con heel e toe entrambi caricati non genera invece alcun falso
HS e resta un bootstrap parziale.

## 7. Timing complessivo della V9

Il gate temporale e' rimasto invariato:

```text
HS massimo ammesso = 50 ms
TO massimo ammesso = 80 ms
```

Sulla finestra iniziale `13.95-18.95 s`:

| Metrica | 10 ms | 1 ms | Stato |
| --- | ---: | ---: | --- |
| errore HS massimo | `80.073 ms` | `82.073 ms` | FAIL |
| errore TO massimo | `39.978 ms` | `40.978 ms` | PASS |

Il nuovo HS startup ha errore nullo. Il FAIL deriva dagli HS successivi e
dimostra che la temporizzazione V9 non generalizza ancora entro `50 ms` fra la
finestra `50-100 s` e la finestra iniziale AB06.

## 8. Plot morphology con V9

Il plot aggiornato usa la traiettoria served congelata e ricalcola offline gli
eventi del detector V9. Mostra:

- quattro HS delimitanti;
- tre TO;
- tre cicli HS-TO-HS completi;
- onset e conferme causali distinti;
- coda finale incompleta tratteggiata;
- trace served, stati e azioni invariati esattamente.

Il sidecar dichiara correttamente:

```text
status                  PASS_DIAGNOSTIC_NOT_PROMOTED
accepted_event_count    7
detector sphere_count   2
detector generates GRF  false
```

## Decisione finale su V9

V9 e' congelata come baseline sperimentale ufficiale:

```text
startup_rule_gate                  = PASS
downstream_prescribed_timing_gate = FAIL
V9 baseline status                = EXPERIMENTAL_NOT_PROMOTED
phase_fsm_input_mode              = legacy_events
training con V9                   = NON ESEGUITO
active closed-loop con V9         = NON ESEGUITO
```

Il profilo e gli artefatti V9 non devono essere sovrascritti o reinterpretati
come PASS. Le revisioni successive devono usare un nuovo identificatore, per
esempio V10.

La robustezza storica non e' stata cancellata: il percorso legacy e' invariato,
lo schema actor non cambia e il replay frozen conserva esattamente le uscite
della policy. Questo non dimostra pero' la deployability del checkpoint con
`two_sensor` autoritativo.

## File modificati o aggiunti

### Codice di produzione

- `Trajectory Generator/prosthetic_phase_fsm.py`

### Placement sweep e protocolli

- `validation/sweep_two_sensor_timing_placements_prescribed_v6.py`
- `validation/two_sensor_timing_placement_sweep_protocol_v6.json`
- `validation/test_two_sensor_timing_placement_sweep_v6.py`
- `validation/sweep_two_sensor_timing_placements_prescribed_v7.py`
- `validation/two_sensor_timing_placement_sweep_protocol_v7.json`
- `validation/test_two_sensor_timing_placement_sweep_v7.py`
- `validation/sweep_two_sensor_timing_placements_prescribed_v8.py`
- `validation/two_sensor_timing_placement_sweep_protocol_v8.json`
- `validation/test_two_sensor_timing_placement_sweep_v8.py`
- `validation/sweep_two_sensor_timing_placements_prescribed_v9.py`
- `validation/two_sensor_timing_placement_sweep_protocol_v9.json`
- `validation/test_two_sensor_timing_placement_sweep_v9.py`

### Startup e plot

- `validation/validate_two_sensor_startup_hs.py`
- `validation/test_two_sensor_startup_hs.py`
- `validation/plot_morphology_corridor_two_sensor_v9.py`
- `validation/test_plot_morphology_corridor_two_sensor_v9.py`
- `validation/plot_two_sensor_timing_placement_v5_multiresolution.py`
- `validation/plot_two_sensor_timing_placement_v7_diagnostic.py`
- relativi test in `validation/`.

### Artefatti principali

- `validation/two_sensor_timing_placement_sweep_runs/2026-07-22_ab06_50_100_multires_geometry_v6/`
- `validation/two_sensor_timing_placement_sweep_runs/2026-07-22_ab06_50_100_multires_geometry_v7/`
- `validation/two_sensor_timing_placement_sweep_runs/2026-07-22_ab06_50_100_heel_micro_v8/`
- `validation/two_sensor_timing_placement_sweep_runs/2026-07-22_ab06_50_100_heel_micro_v9/`
- `validation/two_sensor_startup_hs_runs/2026-07-22_ab06_v9_prescribed/summary.json`
- `validation/experimental_detector_profiles/two_sensor_v9_H2p50_X3p25_F79p0_P35p00.json`
- `plot/07_22_2026_morphology_two_sensor_v9_experimental/01_morphology_corridor_two_sensor_v9_hs_to_hs.png`
- relativo sidecar JSON.

Manifest, CSV, profilo V9, summary startup, plot morphology e sidecar risultano
presenti. Alcuni plot diagnostici V5/V7/V9 citati nel report placement non
risultano presenti nelle rispettive directory `plot/` del workspace corrente;
gli output numerici e i manifest restano disponibili.

## Test e verifiche

- suite placement V6-V9 e regressioni correlate: `138` test PASS;
- test diretti FSM two-sensor dopo la regola startup: `17/17` PASS;
- validator startup: `7` test PASS;
- plot, data path e core online GRF aggiuntivi: `23` test PASS;
- reward: `32` test PASS;
- morphology retrospettiva: `19` test PASS;
- morphology event-anchored: `11` test PASS;
- replay journal FSM: `4` test PASS;
- totale dei test rilevanti della modifica startup: `113` PASS;
- `py_compile`, Ruff e `git diff --check`: PASS;
- protocolli e validator V6-V9 hash-pinned;
- nessun campione `t >= 100 s`; sealed non aperto;
- quattro test storici V9 riportano intenzionalmente `source hash drift`
  dopo la modifica della FSM: gli hash storici non sono stati riscritti e il
  rifiuto preserva la riproducibilita'.

## TODO chiusi il 2026-07-22

- [x] Iterare una geometria forefoot load-bearing mantenendo due soli sensori.
- [x] Eseguire il confronto diretto a `10 ms` e `1 ms` per ogni candidato.
- [x] Risolvere su V9 conteggi, ordine HS-TO-HS, continuita' heel-forefoot,
      candidati TO precoci, invalidi e clear causale sul blocco `50-100 s`.
- [x] Creare un profilo V9 separato senza sovrascrivere il profilo corrente.
- [x] Recuperare l'HS al bordo iniziale quando il pattern heel-on/toe-off e'
      stabile per il dwell.
- [x] Verificare che startup heel+toe o toe-only non generino falsi HS.
- [x] Rigenerare il plot morphology con quattro HS, tre TO e tre cicli completi
      mantenendo la served congelata.

## TODO aperti e propagati

### Detector V10 e routing runtime

- [ ] Creare V10 come ramo separato, senza sovrascrivere V9.
- [ ] Ridurre l'errore HS sotto `50 ms` sia su `13.95-18.95 s` sia sul
      development `50-100 s`, senza trasformare retroattivamente V9 in PASS.
- [ ] Conservare almeno le prestazioni V9 su TO, conteggi, ordine, continuita',
      clear causale, invalidi e coerenza `10 ms / 1 ms`.
- [ ] Evitare altro micro-tuning cieco sullo stesso blocco: una nuova revisione
      deve essere motivata da un'ipotesi fisica e preregistrata.
- [ ] Correggere `_phase_grf_sides()` nel runtime RL: attualmente preferisce
      ancora l'aggregato detector quando presente. `normal_force_bw` e
      `in_contact` devono provenire dalla GRF primaria separata anche nel
      closed-loop, come gia' avviene nei replay offline.
- [ ] Aggiungere un regression test dedicato al routing dual-stream e mantenere
      esattamente due sensori detector-only.
- [ ] Aprire validation/holdout e sealed soltanto dopo un PASS development con
      detector, routing e protocollo congelati.
- [ ] Dopo il PASS prescribed, eseguire shadow e active A/B frozen-policy;
      decidere soltanto allora se serva un fine-tuning.
- [ ] Non cambiare `phase_fsm_input_mode: legacy_events` finche' timing,
      robustezza e gate di promozione non siano tutti superati.

### Morphology Corridor, ancora aperti dal 2026-07-20

- [ ] Completare un rollout live shadow da 500 step della modalita'
      complete-segment in una sessione OpenSim pulita e ripetere il confronto
      A/B; il replay frozen V9 non sostituisce un rollout live.
- [ ] Scegliere il bordo esterno phase-dependent definitivo e decidere se la
      sicurezza debba essere immediata o retrospettiva.
- [ ] Implementare e testare la riscrittura complete-segment/complete-episode
      delle reward prima del GAE.
- [ ] Gestire esplicitamente segmenti incompleti, timeout e bootstrap nel
      ledger completed-segment.
- [ ] Definire `WAIT_HS`, disponibilita' morfologica, stato temporale robusto e
      gate specifico per il regime ankle lento.
- [ ] Soltanto dopo questi gate, eseguire un A/B corto a morphology weight
      `0 / 0.0025 / 0.005`, mantenendo invariati checkpoint, start, seed, batch
      e learning rate.
- [ ] Conservare ogni update e schermare loss morphology, uscite dal corridor,
      cicli, penetrazione, return, SEA, reserve e drift dell'actor.
- [ ] Aggiungere logging per-evento e partire soltanto con un peso piccolo,
      senza promuovere il termine sulla base del solo replay offline.
- [ ] Validare il pilot su start e seed held-out e almeno un profilo/modello
      esterno ad AB06 prima di training lungo o promozione.
- [ ] Mantenere le reserve come gate fisico, non come reward, finche' la GRF
      non sia validata o l'origine del residuo non sia attribuibile.

### Training, reward e robustezza

- [ ] Non avviare training sul detector finche' V10 e il routing runtime non
      superano i gate prescribed; la validazione del detector resta
      indipendente dal PPO.
- [ ] Conservare checkpoint best/H0, V9 e tutti i rollback come artefatti
      immutabili e confrontabili.
- [ ] Valutare i checkpoint futuri con multistart, seed held-out, worst-case
      recovery, cicli, penetrazione, reserve, SEA e clipping, non soltanto con
      return nominale.
- [ ] Non modificare reward, soglia hard da `25 mm` o feature actor per forzare
      la compatibilita' con il detector.
- [ ] Separare l'effetto della reward, della GRF online, dei reserve e della
      temporizzazione FSM prima di attribuire causalmente i failure alla
      policy.
- [ ] Eseguire prove di durata maggiore e su trial, velocita' e soggetti
      differenti prima di dichiarare generalizzazione o deployment validato.
- [ ] Ripartire da H0 o da una sorgente preregistrata, non da logical 51 o dal
      best selezionato per return.
- [ ] Mantenere exact-start, compaction, interleaving, singola epoca e gate
      reserve condition-matched.
- [ ] Concentrare recovery e raccolta dati sugli stati/eventi intorno agli
      step 210-230.
- [ ] Progettare e preregistrare la non-regressione delle reserve tramite
      reward, vincolo o criterio di update, senza confonderla con errori GRF.
- [ ] Mantenere i seed 126-128 sigillati finche' un candidato non supera tutta
      la matrice development; poi usare un solo candidato sugli held-out.
- [ ] Eseguire un nuovo pilot controllato prima di un altro run lungo.
- [ ] Analizzare return e advantage separatamente per start.
- [ ] Non eseguire altri dimezzamenti della learning rate o proiezioni sulla
      stessa trace/seed; raccogliere recovery data event-aligned indipendenti.
- [ ] Riprovare `sigma=0.0075` su almeno tre seed per start soltanto dopo un
      PASS completo a `sigma=0.005`.
- [ ] Mantenere differita una memoria ricorrente finche' non emerga un limite
      sequenziale non coperto dallo stato Markov.
- [ ] Spiegare il TO precoce rifiutato nella seconda stance dell'oracolo
      multi-ciclo.

### Audit ex-novo e contratto di deployment

- [ ] Confrontare H0 e logical 24 con protocollo identico e overlay di azioni,
      served, cinematica, SEA, GRF, eventi e reward.
- [ ] Eseguire ablation progressive del plant prescribed: reset sensor-based,
      perturbazioni encoder/carico, detector, GRF online bilaterale e riduzione
      della dipendenza dalla IK biologica.
- [ ] Eseguire training A/B da H0 modificando un solo gruppo reward per volta:
      detector/FSM, ramo GRF diretto, morphology e command-rate.
- [ ] Congelare ordine e scaling delle feature, frequenza, unita',
      normalizzazione BW, filtri, FSM, reset, governor, limiti, fallback e
      watchdog del contratto hardware.
- [ ] Validare export actor-only, equivalenza host-target, latenza worst-case,
      HIL e poi human-in-the-loop.

### Hardware e TODO storici

- [ ] Validare il detector su segnali con rumore e delay realistici e poi in
      HIL; V9 deriva dal piede virtuale AB06 e non specifica posizione o
      caratteristiche dei futuri sensori fisici.
- [ ] Ottenere una ground truth localizzata o dichiarare esplicitamente la
      semantica `initial_contact`: la GRF totale non dimostra da sola un heel
      strike anatomico.
- [ ] Portare il modello GRF online a uno stato production-ready prima del
      deployment e mantenere i due sensori separati dalla generazione GRF.
- [ ] Valutare una deflessione SEA iniziale coerente con la coppia richiesta,
      TODO storico ancora aperto.
