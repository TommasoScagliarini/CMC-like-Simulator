# Origine delle differenze tra unified39 e training imitativo V26

Data: 21 settembre 2026. Riferimenti: V26 del 20 agosto e unified39 riallineata, continuata fino alla 40. Analisi e diagnostica senza nuovi training e senza modificare FSM v3, reward, governor o controllori SEA.

## Problema e risultato dell'indagine

Il divario di robustezza permane a esperienza equivalente: nelle finestre confrontabili, 39D 36–40 e V26 29–33, gli episodi completi registrati sono rispettivamente **19/49 = 38,8%** e **38/43 = 88,4%**. Entrambe le finestre contengono 20.480 transizioni; i budget cumulativi finali sono 163.840 e 163.212. Il learner ha effettuato complessivamente 3.240 contro 3.224 aggiornamenti di minibatch. Non è più un problema di minore quantità di dati o aggiornamenti.

L'indagine distingue tre domande: se esista un disallineamento tecnico nascosto, perché la camminata prodotta sia peggiore e perché PPO abbia appreso proprio quella soluzione. Le prime due possono essere verificate direttamente con gli artefatti e gli interventi riportati sotto; la terza richiede ancora un confronto di apprendimento controllato per separare il contributo delle differenze rimaste.

**Non emerge un difetto nella capacità della rete, nella trasmissione del prescribed sinistro o nel calcolo della loss.** Il difetto cinematico è già nella reference prodotta a monte dei SEA. Nel contatto esaminato a 17,53 s, l'errore di caviglia produce il momento sfavorevole; una prova statica sul picco ne conferma il meccanismo locale. Questo non attribuisce automaticamente tutte le terminazioni stocastiche allo stesso evento.

Due nuove simulazioni diagnostiche mostrano inoltre che, con gli stessi controllori, comandare direttamente il riferimento sinistro permette di completare i 5 s e recuperare le escursioni mancanti, senza penalità sul momento GRF. Il problema non appare quindi un'impossibilità generale di eseguire quel movimento. Rimangono errore di fase da filtraggio e penetrazione geometrica: non è una soluzione ottimale già pronta.

## Controlli che escludono disallineamenti tecnici

| Componente | Verifica | Esito |
|---|---|---|
| Architettura | Checkpoint reali, non solo YAML | Actor 39, critic 88, 2×256 tanh, 165.893 parametri in entrambi |
| PPO | Confronto ricorsivo delle configurazioni serializzate | Stessi parametri di ottimizzazione, GAE, distribuzione azioni e sampling nominale |
| Fisica | Git, hash di modello, dati e plugin, setup e reset | Nessuna deriva trovata; reset fisico a 12,99 s identico |
| Prescribed e clock | Trace, disponibilità nei batch, ricostruzione azioni | Sei canali visibili; q/qdot e sin/cos coerenti con la fase sinistra |
| Loss imitativa | Ricalcolo indipendente da q/qdot fisici e target registrati | 2.000 step, errore massimo 5,30×10⁻⁸, compatibile con osservazioni float32 |
| Formula reward | Codice storico e attuale sulle stesse loss/reference | 2.000 step, differenza massima **0,0** |
| FSM nei rollout | Replay v2 e v3 degli stessi trace | Tutti gli otto ingressi identici sui 2.000 step, nessun repair |

Il valore `n_full=84` in alcuni metadata V26 era obsoleto: il critic effettivo aveva 88 ingressi, come quello attuale. Non è stata persa informazione privilegiata. Le quattro diagnostiche di controllo restavano fuori dai 39 ingressi dell'actor anche nello storico. Actor, logstd e critic vengono aggiornati; non c'è un congelamento involontario.

Il rollout V26 usato per il confronto cinematico è originale del 20 agosto, prodotto alle 18:06, con best87. I checkpoint intermedi V26 non sono conservati: il confronto degli episodi di training è a esperienza equivalente, quello cinematico finale è contro una policy storica più matura.

## Il compito della caviglia è cambiato davvero

Il nuovo target riusa la classe storica `PhaseBasedImitationTarget`, con coordinate sinistre e shift zero; le velocità derivano dalla stessa spline periodica. Non emerge un errore di segno o di mezzo ciclo. Lo scarto medio di fase rispetto al target storico è circa 3,2% al ginocchio e 4,5% alla caviglia, conseguenza dei diversi clock e degli shift storici calibrati a 0,465/0,452.

| Riferimento sui 5 s confrontati | V26, destro sfasato | Attuale, sinistro |
|---|---:|---:|
| Angolo minimo caviglia | +4,96° | **−6,61°** |
| Angolo massimo caviglia | +25,29° | +22,05° |
| Velocità negativa massima caviglia | −2,22 rad/s | **−3,22 rad/s** |
| Campioni caviglia sotto zero | 0/500 | 84/500 |
| RMSE target–IK sinistra, ginocchio | 6,74° | **2,97°** |
| RMSE target–IK sinistra, caviglia | 7,89° | **1,62°** |

Il movimento negativo oggi mancante non è una competenza che il V26 avesse già dimostrato. Allo stesso tempo, il target sinistro è più vicino ai dati originali della gamba protesizzata: non c'è evidenza che scegliere il lato sinistro sia un errore nei dati.

La normalizzazione per escursione rimane la scelta concordata. Essa mantiene l'obiettivo di ridurre l'errore relativo, ma non conserva il costo dello stesso errore espresso in gradi. Prima dello score non lineare, il coefficiente di posizione per rad² passa:

- ginocchio: 1,1499 → 1,6069, **+39,8%**;
- caviglia: 15,8897 → 8,0135, **−49,6%**;
- rapporto caviglia/ginocchio: 13,82 → 4,99.

Per esempio, 5° di errore alla caviglia contribuiscono 0,1210 alla loss V26 e 0,0610 a quella attuale. È una conseguenza accettata dei dati diversi, non un bug da correggere automaticamente. Può però cambiare il compromesso che PPO apprende fra i due giunti e le penalità fisiche. Il contributo causale di questa differenza al divario di training non è ancora isolato.

![Target e contatto](<../../Trajectory Generator/unified_MLP_39/validation/causal_difference_2026-09-21/target_audit/target_contact_comparison.png>)

## Dove nasce il peggioramento della camminata

Nella policy 40 l'errore RMSE rispetto al target è 6,16° al ginocchio e 5,64° alla caviglia. La reference già servita ha errori quasi identici, 6,14° e 5,62°. Il giunto segue invece la reference con errori di soli 0,27° e 0,41°. Quindi il principale errore cinematico è a monte dell'inseguimento SEA; questo dato da solo non separa policy e filtraggio del governor.

Durante il training la loss imitativa migliora, ma la robustezza non si consolida. Anche nel rollout deterministico, il ginocchio migliore della 40 si accompagna a una penalità sul momento GRF di **40,43 punti**, contro circa zero alla 25. Il termine era già presente nel V26 e non è stato modificato.

La semplice associazione fra più estensione del ginocchio e più carico non bastava a stabilire quale articolazione causasse il momento anomalo. Per questo è stato ricostruito lo stato a **t=17,53 s**, mantenendo tutte le 21 velocità e tutti gli angoli non oggetto di intervento invariati.

| Intervento istantaneo sullo stesso stato | Carico normale | Momento GRF caviglia | Penetrazione | Penalità sul momento |
|---|---:|---:|---:|---:|
| Posa effettiva della 40 | 539,19 N | **−36,75 Nm** | 17,37 mm | 1,2915 |
| Solo caviglia: 14,62° → target 9,33° | 541,77 N | **−4,18 Nm** | 13,54 mm | **0** |
| Ginocchio e caviglia ai target sinistri | 536,56 N | −1,39 Nm | 13,96 mm | 0 |
| Solo ginocchio al valore della 25, −19,45° | 284,55 N | −1,08 Nm | 9,99 mm | 0 |

**Il target sinistro non impone quel picco negativo:** correggere soltanto l'errore di caviglia lo elimina, con carico quasi invariato. La maggiore flessione del ginocchio è una compensazione alternativa, che riduce il carico. Questo spiega perché una policy con ginocchio meno aderente al target possa evitare la penalità pur conservando un errore di caviglia.

La prova riproduce il contatto registrato nello STO entro 7,8×10⁻⁵ N, 6,6×10⁻⁵ Nm e 1,1×10⁻⁸ m sul COP. Il trace della reward usa il wrench precedente di 1 ms e riporta −36,36 Nm, anziché il valore simultaneo −36,75 Nm: l'offset è documentato e non viene presentato come causa del divario fra i training. Gli stati muscolari e SEA ausiliari non sono ricostruiti; non entrano nella legge di contatto esaminata. La verifica è statica e non dimostra che l'intervento sia eseguibile istantaneamente o che tutta la traiettoria sia dinamicamente fattibile.

## Prova dinamica con i due riferimenti

Sono stati eseguiti due rollout diagnostici nuovi di 500 step: il primo comanda il target sinistro al prossimo endpoint, il secondo quello storico destro sfasato. Stessa configurazione fisica della 39D, stesso reset, stesso governor, stessi SEA, stessa FSM v3; nessuna rete genera le azioni e nessun peso viene appreso. Per ciascun braccio il target è stato verificato numericamente contro i 500 campioni già registrati nel rispettivo rollout di riferimento. Le normalizzazioni della reward seguono il target usato; il confronto fisico non dipende dal valore di reward.

| Metrica | Policy 39D, iter40 | Comando diretto LEFT | Comando diretto RIGHT storico |
|---|---:|---:|---:|
| Durata | 5 s | 5 s | 5 s |
| Cicli validi | 2 | 2 | 2 |
| Massima flessione ginocchio | 44,58° | **56,36°** | 64,47° |
| Minimo caviglia | +1,73° | **−6,93°** | +5,26° |
| RMSE ginocchio–proprio target | 6,16° | 6,24° | 7,78° |
| RMSE caviglia–proprio target | 5,64° | **3,97°** | 2,99° |
| Penalità cumulativa momento GRF | 40,43 | **0** | 34,67 |
| Penetrazione massima | 18,21 mm | **22,79 mm** | 21,67 mm |
| Picco forza normale | 579,47 N | 998,30 N | 811,69 N |
| Ritorno | 222,15 | 366,77 | 318,25 |

Il comando diretto sinistro recupera il picco richiesto di ginocchio e la porzione negativa della caviglia. Nessun limite di velocità, accelerazione o jerk del governor risulta attivo secondo la diagnostica registrata. Rimane il ritardo del filtro: il RMSE del ginocchio non migliora, pur recuperando l'intera escursione. Una buona policy deve anche compensare questo ritardo. Non è una dimostrazione di inseguimento cinematico perfetto.

La penetrazione sinistra è superiore a quella della policy 40 e supera la soglia di penalizzazione 20 mm, pur restando sotto la terminazione 28 mm; il carico arriva a circa 998 N. L'esito esclude l'idea che la porzione negativa della caviglia o la maggiore flessione richiesta siano semplicemente irraggiungibili in questa condizione. Non prova robustezza a esplorazione, altri start o altri cicli, né assenza di penetrazione.

Il braccio con target destro, eseguito attraverso la stessa pipeline, ha una penalità di momento non nulla. Il successo del V26 addestrato non era dunque una proprietà automatica del solo riferimento destro: la policy aveva appreso compensazioni. Anche il riferimento sinistro può richiedere una diversa soluzione di controllo.

Il ritorno LEFT 366,77 non va interpretato come equivalenza al V26 best87 da 363,79. Il comando diretto ha un costo cumulativo di variazione del comando di 20,03, contro 70,86 nella policy 40 e 61,41 nel V26 best87; i target sono diversi e gli errori cinematici restano diversi. Il valore scalare da solo non certifica qualità del passo.

![Rollout diagnostici con target comandato direttamente](<../../Trajectory Generator/unified_MLP_39/validation/causal_difference_2026-09-21/oracle_comparison.png>)

[Metriche complete](<../../Trajectory Generator/unified_MLP_39/validation/causal_difference_2026-09-21/oracle_metrics.json>), [rollout LEFT](<../../Trajectory Generator/unified_MLP_39/runs/rollout/causal_difference_2026-09-21_oracle_left/rollout_summary.json>) e [rollout RIGHT](<../../Trajectory Generator/unified_MLP_39/runs/rollout/causal_difference_2026-09-21_oracle_right/rollout_summary.json>).

Limite metodologico: l'azione diagnostica accede al target del prossimo endpoint, 10 ms avanti; la rete reale osserva fase, q e qdot correnti e deve apprendere la previsione/compensazione. L'osservazione e i metadata del braccio RIGHT restano quelli del contenitore 39D sinistro, ma nessuna rete li usa; `diagnostic_intervention` identifica esplicitamente la modifica. Non è un checkpoint valutato né un test di apprendimento.

## Differenze nel processo di apprendimento

### Gestione dei fallimenti

Dopo il V26 sono stati corretti errori di gestione della guardia hard a 30 mm e del trasporto degli eventi binari. Prima alcuni fallimenti facevano morire un worker e potevano perdere il frammento raccolto. Oggi la guardia hard diventa una terminazione MDP conservata con relativo trattamento del ritorno terminale; gli eventi binari scartabili vengono rifiutati mantenendo un avanzamento temporale coerente, mentre quelli non recuperabili terminano l'episodio. Non cambia soltanto il numero di campioni, ma anche quali fallimenti vengono appresi. Non è opportuno ripristinare quei bug.

Questa differenza può aver influito sulla traiettoria iniziale di apprendimento. Non spiega aritmeticamente il divario recente: delle 30 penetrazioni nelle iterazioni 36–40, **29 avvengono alla soglia ordinaria di 28 mm**, identica allo storico, e una è hard. Nella finestra V26 29–33 i batch sono 4096 e le durate regolari; mancano però i log Ray originali per certificare l'assenza assoluta di crash. Il possibile bias dei contatori storici resta un limite, non una spiegazione dimostrata dell'88,4%.

### FSM v3

Nei rollout deterministici esaminati v2 e v3 danno gli stessi ingressi. Durante training sono invece presenti 133 righe di telemetria con resync, su 31 iterazioni: la v3 può cambiare osservazioni, evitare timeout e permettere all'episodio di visitare altri stati. I pesi reward specifici FSM sono zero; non esiste una penalità FSM aggiunta di nascosto. Mancano trace stocastiche complete per collegare ogni repair a una successiva penetrazione. Non si può né accusare né escludere la v3 come contributo indiretto alla policy appresa.

### Inizializzazione e ordine degli ingressi

I quattro target sono passati dagli indici 2–5 agli indici 35–38; gli altri ingressi dell'actor sono stati riordinati. Con seed 123 i pesi iniziali numerici sono gli stessi, ma non sono state permutate insieme le colonne del primo layer. Pertanto i due esperimenti non partono dalla stessa funzione sugli ingressi semanticamente corrispondenti.

Questo non riduce la capacità della rete: la permutazione è compensabile nei pesi e un'inizializzazione iid è statisticamente invariante a essa. È però un motivo concreto per cui un singolo seed non costituisce una replica causale esatta, anche prima di cambiare target. Non è una causa dimostrata di prestazioni mediamente peggiori.

### Esplorazione

La policy 40 completa i 5 s deterministicamente, mentre gli episodi esplorativi recenti terminano spesso per penetrazione. Sugli stati deterministici, le deviazioni standard delle azioni sono circa 0,86 e 0,93 e la probabilità gaussiana di superare i limiti è circa 36% e 29%. Sono stime condizionate agli stati visitati, non percentuali misurate di caduta. L'entropia di training è leggermente maggiore del V26 a pari esperienza, ma coefficiente di entropia e configurazione della distribuzione coincidono.

L'esplorazione è quindi un candidato per spiegare la differenza tra completamento deterministico e stocastico. Per attribuirle una quota del divario occorrono valutazioni con rumore di azione controllato, non un cambiamento automatico della configurazione.

## Interpretazione e controllo successivo

Le prove non sostengono l'idea che la rete 39D abbia meno capacità del V26 o che il prescribed sinistro venga presentato o valutato in modo errato. Sostengono invece che la policy appresa abbia trovato una coordinazione insufficiente: nel punto esaminato il ginocchio è vicino al target, ma l'errore residuo alla caviglia produce il momento sfavorevole; una maggiore flessione del ginocchio può compensarlo. La robustezza esplorativa rimane incompleta.

Il cambio di traiettoria, il cambiamento del costo per grado implicito nella normalizzazione, i fallimenti ora conservati, l'effetto della v3 durante training e una funzione iniziale diversa sono differenze reali. Dai soli run esistenti non è possibile assegnare a ciascuna una percentuale del peggioramento, né dimostrare che una singola modifica risolva tutto.

Il controllo più netto per la prossima fase è una coppia di training nella stessa pipeline 39D attuale, mantenendo FSM v3 e fix: un controllo con target/clock storici e uno con target/clock sinistri, stessa inizializzazione semantica, stessi batch effettivi e stesso budget di transizioni. La normalizzazione derivata dal rispettivo target resta parte della modifica deliberata. Se il controllo storico recupera qualità e quello sinistro no, si restringe il problema al nuovo compito e alla sua ottimizzazione. Se falliscono entrambi, si indagano le differenze comuni della pipeline e la variabilità fra seed. Prima di ulteriori training, rollout stocastici con rumore controllato possono quantificare quanto il problema sia nella media della policy e quanto nella sua dispersione.

**Nessun nuovo training è stato avviato e nessuna configurazione operativa è stata cambiata.**

## Artefatti, file e verifiche

Tutto il codice diagnostico nuovo è in `Trajectory Generator/unified_MLP_39/validation/causal_difference_2026-09-21/`. I sorgenti di training e simulazione restano invariati.

- [Audit learner, ingressi e provenienza fisica](<../../Trajectory Generator/unified_MLP_39/validation/causal_difference_2026-09-21/training_audit/findings.md>) e [hash degli asset](<../../Trajectory Generator/unified_MLP_39/validation/causal_difference_2026-09-21/training_audit/physical_provenance.json>).
- [Audit reward e FSM](<../../Trajectory Generator/unified_MLP_39/validation/causal_difference_2026-09-21/fsm_audit/findings.md>), con script `audit.py`, metriche e replay degli otto ingressi.
- [Audit target e contatto](<../../Trajectory Generator/unified_MLP_39/validation/causal_difference_2026-09-21/target_audit/README.md>), con `analyze.py`, `static_contact_probe.py`, metriche e grafico.
- [Ricostruzione indipendente delle loss](<../../Trajectory Generator/unified_MLP_39/validation/causal_difference_2026-09-21/loss_reconstruction.json>), riproducibile con `verify_losses.py`.
- [Rollout diagnostici diretti](<../../Trajectory Generator/unified_MLP_39/validation/causal_difference_2026-09-21/oracle_probe.py>) e [script di confronto](<../../Trajectory Generator/unified_MLP_39/validation/causal_difference_2026-09-21/analyze_oracles.py>).
- [Risultati completi della continuazione a 40](2026-09-21_risultati_unified39_v26_aligned_40iter.md).

Restano necessari più seed e criteri di confronto predefiniti per stabilire equivalenza statistica. Le verifiche attuali localizzano differenze e meccanismi; non sostituiscono quel test.
