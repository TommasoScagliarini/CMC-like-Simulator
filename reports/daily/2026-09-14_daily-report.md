# Daily Report - 2026-09-14

Instruction check token: `CMC_AGENT_OK_2026`

Aggiornamento documentale della giornata, creato con `create_report` per
conservare anche nel daily i TODO del report utente. Non attesta l'esecuzione
di nuovi training o la conclusione degli esperimenti proposti.

## Report e contesto consolidati

- [Osservabilità prescribed, mascheramento e test di fattibilità](../user/2026-09-14_osservabilita_prescribed_mascheramento_e_test_fattibilita.md).
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
- Suite aggiornata: 82 test superati in 5.82 s su macOS. Nessun nuovo run
  dinamico con il peso 1.0; lo smoke del 6 settembre usava ancora 0.05.
- Audit in sola lettura delle tracce smoke: 1089 campioni, periodo online
  sempre zero e sin/cos `(0, 1)`, nessun ciclo valido. Altri segnali e reset
  differiscono; non è dimostrata ambiguità dell'intero vettore actor.
- Le tracce associano osservazione pre-step e target post-step: prima di
  costruire dataset diagnostici è necessario riallineare i timestamp.
- Definito un protocollo di isolamento software, diagnostica osservabilità,
  confronto A/B/C, rollout ai tre start e verifica del trasferimento ex-novo.
  Queste nuove prove non sono state eseguite.

Con `create_report` sono creati soltanto il report utente collegato e questo
daily. Nessun codice, configurazione, checkpoint o artefatto baseline viene
modificato dal comando. I controlli della redazione riguardano documenti,
collegamenti locali e stato Git, non nuove verifiche della rete. Windows live
rimane non verificato.

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
  con valutazioni periodiche; attività successiva allo smoke V1, non eseguita.
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
