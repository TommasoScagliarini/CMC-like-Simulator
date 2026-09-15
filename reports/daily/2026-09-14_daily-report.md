# Daily Report - 2026-09-14

Instruction check token: `CMC_AGENT_OK_2026`

Consolidamento della giornata e propagazione dei TODO tramite `create_report`.
Il pilot unificato a 35 feature è concluso a cinque iterazioni, seguito da
sei rollout e dal confronto storico V26. La prosecuzione a 50/100 e i test
della variante a 39 feature sono attività future. La variante verrà creata
quando si svolgeranno quei test, secondo il chiarimento esplicito dell'utente.

## Report e contesto consolidati

- [Osservabilità prescribed, mascheramento e test di fattibilità](../user/2026-09-14_osservabilita_prescribed_mascheramento_e_test_fattibilita.md).
- [Pilot unificato, strategia 50→100 e futura variante a 39 feature](../user/2026-09-14_pilot_unificata_strategia_50_100_e_test_39_feature.md).
- [Rete unificata V1 e protocollo di parità del 6 settembre](../user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md): contesto precedente e TODO di parità ancora aperti.
- [Ultimo daily precedente, 31 agosto](2026-08-31_daily-report.md): fonte dei TODO storici riportati sotto.

## Problema e direzione discussa

La V1 usa 35 ingressi actor senza target imitativi e con i due slot del clock
prescribed a zero. La reward segue i target prescribed sinistri al tempo
corrente, mentre la fase online necessita di un ciclo valido HS→TO→HS per
disporre di un periodo misurato. È ancora da verificare se le altre feature
permettano un pretraining robusto, soprattutto all'avvio.

È stata discussa una variante a 39 ingressi: quattro target temporaneamente
visibili all'actor e poi sostituiti da costanti in una quota crescente di
episodi, fino al mascheramento completo prima dell'ex-novo. La reward imitativa
continuerebbe a usare il target originale. Togliere il target dalla sola
reward ex-novo non garantisce l'indipendenza della policy.

Mascheramento, curriculum e variante architetturale non sono implementati.
Restano da valutare anche avvio supervisionato dello stesso actor finale e
soluzioni causali di fase/memoria. Predittività e compensazione del ritardo
rimangono fuori dalla V1.

## Modifiche e verifiche della giornata già documentate

- Peso di penetrazione nei due preset imitativi portato a 1.0 su richiesta
  dell'utente; ex-novo resta a 0.05, soglie 20/28 mm invariate. Provenienza,
  test di configurazione, README, VALIDATION e TODO aggiornati nella pipeline.
- Prima dell'avvio del pilot: 82 test superati in 5.82 s su macOS.
  Lo smoke del 6 settembre usava ancora peso 0.05; il successivo pilot
  della giornata ha usato 1.0, come documentato sotto.
- Audit in sola lettura delle tracce smoke: 1089 campioni, periodo online
  sempre zero e sin/cos `(0, 1)`, nessun ciclo valido. Altri segnali e reset
  differiscono; non è dimostrata ambiguità dell'intero vettore actor.
- Le tracce associano osservazione pre-step e target post-step: prima di
  costruire dataset diagnostici è necessario riallineare i timestamp.
- Definito un protocollo di isolamento software, diagnostica osservabilità,
  confronto A/B/C, rollout ai tre start e verifica del trasferimento ex-novo.
  Queste nuove prove non sono state eseguite.

Le redazioni `create_report` modificano soltanto report utente e daily.
I lavori di implementazione e simulazione descritti sono stati eseguiti
prima della redazione. Baseline, plugin C++, runtime condiviso e relativi
checkpoint rimangono conservati. Windows live resta non verificato.

## Pilot corretto, confronto dei checkpoint e strategia successiva

- Primo lancio fallito per Ray assente nel Python base; rilancio in
  `envCMC-rllib`. Un successivo tentativo è stato interrotto e archiviato
  per correggere due terminazioni morphology attive in imitazione.
- Fix limitato al wrapper unificato: guardie morphology solo in ex-novo,
  diagnostica e protezioni comuni conservate. Aggiunte 18 regressioni;
  suite di 100 test passata in 4.20 s. Nel primo batch ripetuto il digest
  actor è identico: non è dimostrato un effetto sui ritorni di quel batch.
- Pilot corretto da pesi nuovi, seed 123: **5/5 iterazioni, 23 040 transizioni,
  circa 45 min 53 s**, senza crash/restart o iterazioni saltate. Configurazione
  piena: 12 runner, batch 4608, tre start bilanciati, 10 epoche, penetrazione 1.0.
- Sei rollout deterministici dei checkpoint 3 e 5: esecuzione riuscita;
  early 5 s, center 2.80 s e late 2.83 s per entrambi. Zero HS/cicli validi;
  center/late terminano per timeout swing. Il checkpoint 5 migliora gli
  errori angolari ma aumenta la penetrazione. Il periodo online resta zero.
- RMSE actual–served 0.16–0.40°: l'errore maggiore è già nel riferimento
  servito rispetto al prescribed. Le cinque medie mobili TensorBoard di
  HS/cicli sono zero, senza dimostrare assenza esaustiva su ogni transizione.
- Analizzatore offline con sei nuovi test passati separatamente e grafici
  PNG/PDF verificati. Queste prove non attestano qualità del cammino o parità.
- Il V26 del 20 agosto aveva episodi di 2–3 s nelle prime cinque iterazioni,
  ma già alcuni cicli registrati; a 50 raggiungeva durata media 4.81 s,
  best di training a 87. Osservazioni, start, FSM e campionamento differiscono.

Strategia proposta: **altre 45 iterazioni fino a 50 totali → valutazione →
altre 50 fino a 100 → valutazione → test di fattibilità**. Resume completo
della V1 e configurazione costante. ETA residua dal checkpoint 5: circa
6 h 53 min fino a 50, 14 h 32 min fino a 100, esclusi rollout e variazioni
del costo della simulazione.

La futura variante a **39 feature** affiancherà la V1 a 35 e manterrà
contratto comune per A/B/C e per il passaggio imitazione/ex-novo. V1 resta
un riferimento separato dal controllo B sempre mascherato. **La variante
si creerà nella fase dei test; nessuna implementazione ora.** Nessun training
a 50/100 o test della variante è stato avviato con questa redazione.

## TODO della strategia 50→100 e variante futura

- [ ] Riprendere la V1 a 35 feature dal checkpoint 5 fino a 50 iterazioni totali, conservando configurazione e stato completo del training.
- [ ] Fissare i criteri e svolgere la valutazione intermedia a 50 sui tre start, includendo tracking, cicli, primo periodo online, terminazioni e penetrazione.
- [ ] Proseguire la stessa V1 da 50 a 100 iterazioni totali e svolgere la valutazione finale confrontando i risultati a 5, 50 e 100.
- [ ] Creare la variante parallela a 39 feature soltanto nella successiva fase dei test di fattibilità, conservando V1 a 35 e baseline storica; nessuna implementazione ora.
- [ ] Definire e verificare schema 39, maschera persistente e indipendenza ex-novo dalle copie dei target; fissare curriculum, budget, seed e criteri prima del confronto A/B/C.
- [ ] Eseguire diagnostica dell'osservabilità, confronto A/B/C completamente mascherato in valutazione e test di trasferimento ex-novo B/C, distinguendo avvio, regime e variabilità tra seed.

## TODO del report del 14 settembre

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

Aggiornamento dei TODO precedenti: è disponibile il pilot con peso 1.0,
ma un confronto che isoli causalmente l'effetto del peso e la valutazione
del pretraining completo restano aperti. Per la variante, la scelta corrente
è 39 ingressi e la creazione è differita alla fase dei test. Le alternative
supervisionate o di fase/memoria rimangono ipotesi da valutare.

## TODO di parità propagati dal report del 6 settembre

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

## TODO storici: aggiornamento e propagazione

La separazione integrale delle reward, le safety comuni e i termini inattivi
sono stati implementati e verificati nella V1 del 6 settembre: il relativo
TODO del 31 agosto è completato per la rete unificata.

- [x] Implementare e verificare il cambio integrale di modalità reward,
  mantenendo safety sempre attiva e termini spenti in shadow logging.

Le attività seguenti sono conservate dal daily del 31 agosto. Non sono state
riesaminate integralmente durante la redazione di questo report e non
costituiscono nuove istruzioni operative per la baseline. In particolare il
pilot e l'estensione PPO storici vanno riconciliati con il run J20 del
1 settembre prima di pianificare eventuali altre esecuzioni.

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

## Aggiornamento serale: esecuzione autorizzata del piano completo

Successivamente alla redazione precedente l'utente ha autorizzato tutte le
cinque fasi. Il [report di avvio](../user/2026-09-14_avvio_piano_pretraining_50_100_fattibilita.md)
documenta resume e automazione. Il checkpoint5 è stato ripristinato con
actor/medie/critic identici, proseguendo da6/50. Il primo tentativo sandbox
falliva prima degli update per `psutil/sysctl`; il rilancio runtime è attivo.

Coordinatore persistente avviato alle19:47 CEST, PID14612: attende50,
valuta, prosegue fino a100, valuta e avvia la fattibilità39. Stato corrente:
[STATUS.md](<../../Trajectory Generator/unified_MLP/runs/plans/pretraining_50_100_feasibility_2026-09-14/STATUS.md>).
Il [protocollo](../plans/2026-09-14_protocollo_fattibilita_unificata39.md) fissa
budget A/B/C e trasferimento, maschere, repliche condizionate e criteri.
La variante runtime viene creata soltanto dopo la valutazione100; il bundle
preparatorio è verificato in directory temporanee.

Verifiche preparatorie:165 test complessivi passati,18 test successivi
dell'orchestrazione e10 nel file esteso per sigma/rumore. Non sono risultati
di cammino a50/100. Il piano non è ancora concluso.

- [ ] Completare V1 fino a50 e valutarla sui tre start.
- [ ] Completare V1 fino a100 e confrontare5/50/100.
- [ ] Creare la variante39 dopo valutazione100; superare isolamento e smoke.
- [ ] Completare A/B/C, trasferimento B/C ed eventuali repliche preregistrate.
- [ ] Rendicontare tutti gli esiti e costi, distinguendo fattibilità e parità.
- [ ] Eseguire la verifica live Windows separata.
