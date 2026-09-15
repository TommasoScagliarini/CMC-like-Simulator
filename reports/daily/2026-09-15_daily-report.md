# Daily Report - 2026-09-15

<!-- create_report_50:41b7d69e7228f7d2 -->

## Valutazione intermedia V1 a 50 iterazioni

[Report utente della valutazione](<../user/2026-09-15_pretraining_unificata_valutazione_intermedia_50.md>). Training 50/50 e valutazione last/best/CP5 sui tre start completati. Screening e parità restano distinti; dati e figure nel report.

- [x] Completare V1 fino a50 e valutarla sui tre start.

I precedenti TODO relativi esclusivamente al training e alla valutazione a 50 sono esauriti. Le altre sezioni di questo daily sono conservate.

### TODO aperti propagati

- [ ] Proseguire la stessa V1 da 50 a 100 iterazioni totali e svolgere la valutazione finale confrontando i risultati a 5, 50 e 100.
- [ ] Creare la variante parallela a 39 feature soltanto nella successiva fase dei test di fattibilità, conservando V1 a 35 e baseline storica; nessuna implementazione ora.
- [ ] Definire e verificare schema 39, maschera persistente e indipendenza ex-novo dalle copie dei target; fissare curriculum, budget, seed e criteri prima del confronto A/B/C.
- [ ] Eseguire diagnostica dell'osservabilità, confronto A/B/C completamente mascherato in valutazione e test di trasferimento ex-novo B/C, distinguendo avvio, regime e variabilità tra seed.
- [ ] Risolvere e verificare la sincronizzazione senza clock prescribed, distinguendo avvio e regime ai tre start.
- [ ] Costruire dati diagnostici temporalmente allineati e valutare osservabilità delle 35 feature su episodi indipendenti.
- [ ] Scegliere fra curriculum con target mascherati, avvio supervisionato e possibili soluzioni causali di fase/memoria sulla base di evidenze.
- [ ] Se si procede con la maschera, definire il nuovo schema, la normalizzazione, l'eventuale indicatore di disponibilità e lo stato persistente del curriculum, conservando la V1.
- [ ] Implementare e verificare isolamento dei target, coerenza raccolta/PPO e indipendenza degli aggiornamenti ex-novo dalle copie dei target.
- [ ] Definire budget, seed, progressione e criteri di accettazione prima del confronto A/B/C; eseguire poi il pilot e le repliche necessarie.
- [ ] Validare i candidati completamente mascherati ai tre start, con metriche separate di avvio, tracking, cicli validi e protezioni.
- [ ] Verificare trasferimento, warm-up e resume della variante scelta e confrontarne il beneficio nel successivo ex-novo.
- [ ] Misurare in un nuovo run imitativo l'effetto del peso di penetrazione 1.0, senza attribuirgli gli esiti dello smoke storico a 0.05.
- [ ] Completare il benchmark e il verdetto di parità qualitativa secondo il report del 6 settembre, includendo costo di apprendimento e variabilità tra seed.
- [ ] Eseguire le verifiche live Windows della pipeline scelta; riprendere predittività e compensazione del ritardo soltanto in una versione successiva.
- [ ] Fissare checkpoint baseline, manifest degli input e protocollo comune
  di confronto, mantenendo intatti gli artefatti storici.
- [ ] Implementare il valutatore comune sotto `Trajectory Generator/unified_MLP/`,
  con metriche per start e per giunto, copertura dei cicli e gestione esplicita
  dei fallimenti e dei dati mancanti.
- [ ] Completare e riprodurre il benchmark baseline deterministico sui tre start.
- [ ] Definire prima della selezione finale margini di regressione, condizioni
  riservate alla valutazione, numero di repliche e budget di addestramento.
- [ ] Eseguire un vero pretraining e training ex-novo della rete unificata,
  con valutazioni periodiche; pilot imitativo a cinque iterazioni concluso,
  prosecuzione e training ex-novo completo ancora da eseguire.
- [ ] Eseguire il confronto deterministico e stocastico finale, includendo
  variabilità tra seed di training e incertezza delle differenze.
- [ ] Produrre il verdetto quantitativo di parità, regressione o risultato
  inconclusivo, insieme al costo necessario per raggiungere la qualità ottenuta.
- [ ] Eseguire le verifiche live su Windows per attestare anche quella piattaforma.
- [ ] Riprendere actor predittivo e compensazione del ritardo in una versione
  successiva; restano esclusi dal recupero della parità della V1 attuale.
- [ ] Implementare il teacher inverse-governor e il generatore delle label raw.
- [ ] Validare frozen-policy gli orizzonti attorno a `90 ms` knee e `100 ms`
  ankle su più cadenze e stati del governor.
- [ ] Eseguire DAgger closed-loop sullo stesso actor 35D con nominali e
  recovery.
- [ ] Definire e preregistrare il primo rapporto morphology knee/ankle
  asimmetrico.
- [ ] Ripetere il critic-only rewarm quando la reward diventerà realmente
  asimmetrica.
- [ ] Valutare una fascia soft interna e una corretta assegnazione causale del
  credito, senza spostare il corridor finale.
- [ ] Progettare e validare un corridor cadence-conditioned su dati held-out.
- [ ] Preregistrare ed eseguire un pilot PPO separato di `5-10` iterazioni con
  `sigma=0.010` isotropico e `freeze_logstd=true`, senza modificare J20 o il
  canonico lungo.
- [ ] Decidere un'eventuale estensione oltre 10 iterazioni soltanto dopo i
  rollout deterministici e i gate del pilot.
- [ ] Monitorare penetrazione `20/25/28 mm`, served-vs-actual e contatori FSM
  v3 in tutti i nuovi candidati.
- [ ] Rivalutare sigma dopo l'actor predittivo o una modifica della logica
  causale del morphology corridor.
- [ ] Generalizzazione oltre AB06 tramite dataset EPIC e soggetto held-out,
  dopo il recupero della qualità AB06.
- [ ] LOTO, LOCO, B1R1 e B1R2 restano TODO futuri soggetti ad approvazione
  esplicita.
- [ ] Conservare sidecar J8 e artefatti storici senza modifiche retroattive.
- [ ] Riallineare in manutenzione separata le assertion legacy soggette a drift
  e gli altri debiti tecnici/amministrativi già propagati: osservabilità RLlib,
  `cycle_valid`, swing timeout, probe schema, pin V20, `best == last`,
  `fit_p2` V12R3, marker/audit storici e deflessione SEA iniziale.
- [ ] Completare V1 fino a100 e confrontare5/50/100.
- [ ] Creare la variante39 dopo valutazione100; superare isolamento e smoke.
- [ ] Completare A/B/C, trasferimento B/C ed eventuali repliche preregistrate.
- [ ] Rendicontare tutti gli esiti e costi, distinguendo fattibilità e parità.
- [ ] Eseguire la verifica live Windows separata.
- [ ] Proseguire la stessa V1 da 50 a 100 iterazioni totali e svolgere la valutazione finale 5/50/100.
- [ ] Svolgere la fase 5: variante 39, isolamento, smoke, confronto A/B/C e trasferimento ex-novo B/C secondo il protocollo.
- [ ] Completare la diagnostica dell'osservabilità e verificare la sincronizzazione senza clock prescribed, distinguendo avvio e regime.
- [ ] Completare il benchmark di parità con la baseline, includendo costi e variabilità tra seed.
- [ ] Eseguire le verifiche live Windows; mantenere aperti i debiti storici non risolti.


<!-- evaluation_50_recovery_2026-09-15 -->

## Ripristino della valutazione intermedia e ripresa verso 100

Il training imitativo ha completato le 50 iterazioni senza salti o crash restart nel summary del resume. La valutazione dei checkpoint 49 e 50 aveva completato i sei rollout, poi si era fermata alle 02:47 CEST: l'analisi richiedeva erroneamente che `valid_hs_count` fosse monotono. La FSM v3 può invece revocare un appoggio per rimbalzo (`hs_bounce_cancelled`), decrementando il conteggio netto e incrementando `hs_cancelled_count`.

Corretto il postprocessing in `Trajectory Generator/unified_MLP/validation/compare_pretraining_checkpoints.py` e `evaluate_training_milestone.py`: i cali HS sono ammessi solo quando coperti da cancellazioni osservate, anche al confine della finestra; gli altri contatori restano monotoni. I contatori devono essere interi nonnegativi; le variazioni HS delle sottofinestre possono avere saldo negativo. La telemetria legacy assente resta assente. Il generatore del report espone le cancellazioni separatamente e documenta questa semantica.

Verifiche: **75 test mirati superati**, comprese valutazione, report e coordinatore; analizzate con successo tutte le sei trace CP49/CP50 già salvate. Controllati gli hash delle sorgenti runtime e del checkpoint iniziale: invariati. Nessun nuovo training o rollout a 50 è stato necessario. Il nuovo processo ha completato la valutazione a nove casi, incluso CP5, e pubblicato il report utente alle **08:12 CEST**. Il coordinatore è ripartito dal checkpoint completo 50 con obiettivo 100; valutazione finale e fattibilità rimangono le fasi successive autorizzate.

La valutazione intermedia resta negativa sullo screening complessivo: CP49 produce tre cicli nello start center e CP50 due nello start late, ma entrambi quei rollout terminano per penetrazione. Lo start early raggiunge 5 secondi senza cicli validi. I risultati non stabiliscono ancora cammino stabile ai tre start; metriche e confronti a prefisso comune sono nel report utente.

La precedente esecuzione fallita è conservata nella directory di recovery del piano; [evidenza del ripristino](<../../Trajectory Generator/unified_MLP/runs/plans/pretraining_50_100_feasibility_2026-09-14/evaluation_50_recovery.json>). I TODO aperti sopra rimangono validi.
