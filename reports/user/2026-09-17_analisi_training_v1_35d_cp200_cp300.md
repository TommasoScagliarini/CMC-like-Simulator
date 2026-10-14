# 2026-09-17 — Analisi del training V1 35D da CP200 a CP300

**Analisi completata. Le altre 100 iterazioni migliorano alcuni indicatori del training stocastico, ma non producono una camminata deterministica complessivamente migliore di CP200.** CP300 conserva i 5 s sui tre start, aumenta moderatamente l'escursione del ginocchio, ma non riduce l'errore del ginocchio rispetto al prescribed; peggiora il tracking della caviglia e la regolarità degli eventi. Anche CP283, best per return di training, non supera CP200 sul comportamento complessivo verificato.

La valutazione comprende CP200, CP250, CP283 e CP300 sui medesimi tre start early/center/late: 12 episodi da 500 step, 5 s ciascuno, seed 123, azione deterministica. CP200/250 e CP300 early sono riutilizzati dopo verifica; cinque rollout mancanti sono stati eseguiti durante questa analisi. Tutti raggiungono il limite temporale. Nessun training oltre 300 è stato avviato.

## Problema, strategia e soluzione adottata

La domanda sperimentale era verificare se altre 100 iterazioni della rete unificata V1 35D, senza il target prescribed direttamente osservato dall'actor, migliorassero il cammino ottenuto a CP200. Il completamento della valutazione automatica era inoltre bloccato da una differenza puramente descrittiva nella configurazione esportata.

La strategia è stata ricostruire l'andamento del training, valutare sia l'ultimo checkpoint sia il best per return, confrontarli con CP200 e CP250 sugli stessi start e separare durata, imitazione, eventi del passo e sicurezza. Il confronto distingue esplicitamente movimento effettivo, riferimento servito e target prescribed.

La soluzione operativa è consistita nel recuperare la valutazione dopo l'audit di compatibilità, completare i cinque rollout mancanti e produrre metriche, grafici e verifiche riproducibili. L'analisi è conclusa; il problema della qualità del cammino rimane aperto. Le proposte di ulteriori esperimenti riportate sotto sono raccomandazioni, non attività avviate o nuovi training autorizzati.

## Completamento e integrità del training

Il training è terminato il 17 settembre alle **11:20:45 Europe/Rome**, dopo **15 h 11 min 49 s**. Il resume completo da CP200 ha eseguito 100 nuovi aggiornamenti, dal 201 al 300, con 460.800 nuove transizioni: 153.600 per ciascuno start. Ogni aggiornamento campiona tutti e tre gli start in modo bilanciato; il numero globale di aggiornamenti non va diviso per tre. Il campionamento per start è invece un terzo del totale.

La linea ereditata da CP300 contiene 1.382.400 transizioni. La vecchia coppia di iterazioni 99–100 del run originale, esclusa dal precedente resume da CP98, porta il costo complessivo delle campagne a 1.391.616 transizioni. Le curve 1–300 rappresentano la linea effettivamente ereditata, non sovrappongono i due rami.

Nessun riavvio, nessun aggiornamento saltato, nessuna telemetria mancante o KL non finita. Le prime 200 righe di storia coincidono con il parent. Verificati 3.900 scalar TensorBoard e la loro concordanza con JSONL. Learning rate 1e-4, 10 epoche, 12 runner, reward e osservazioni congelate del run sono stati conservati. Il best per return di training è **CP283: 290,988**; CP300 riporta **251,336**. Un singolo massimo non misura la qualità generale della camminata.

## Cosa è migliorato durante il training

| Blocco | Return medio riportato | Durata media riportata | Episodi al limite di 5 s |
|---|---:|---:|---:|
| 101–200 | 223,81 | 3,757 s | 27,8% |
| 201–300 | 243,50 | 4,113 s | 44,4% |
| 201–250 | 243,68 | 4,202 s | 48,7% |
| 251–300 | 243,33 | 4,024 s | 40,4% |

Le prime due colonne sono medie delle metriche riportate per iterazione. Le percentuali derivano dai conteggi degli episodi conclusi, verificati contro i conteggi per start; non sono medie di percentuali. Le aggregazioni pesate per episodio sono disponibili nel JSON. Gli episodi possono attraversare aggiornamenti della policy; queste statistiche descrittive non sono repliche indipendenti.

Rispetto al blocco precedente il training produce episodi mediamente più lunghi. Le ultime 50 iterazioni, però, non consolidano il miglioramento: return pressoché invariato e minore quota di completamento rispetto alle prime 50. Sui 1.141 episodi terminati nel proseguimento, 507 arrivano a 5 s, 610 terminano per penetrazione (53,5%), 18 per divergenza del ginocchio, 4 per evento binario invalido e 2 per timeout swing. Early rimane lo start più difficile nonostante il campionamento bilanciato.

Gli score imitativi medi 101–200 → 201–300 cambiano poco: reference servita/prescribed **0,6273 → 0,6356**, movimento effettivo/prescribed **0,7289 → 0,7300**. Sono score adimensionali che includono posizione e velocità, aggregati su finestre mobili dei worker; non sono RMSE in gradi. Non emerge una traiettoria di miglioramento continuo fino a 300.

![Percorso effettivo 1–300](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/training_1_300.png>)

## Il confronto deterministico sui medesimi 5 secondi

In questa tabella gli errori sono medie aritmetiche dei tre RMSE per start; cicli ed eventi sono somme dei tre episodi. Ogni checkpoint completa 3/3 episodi di 5 s.

| Checkpoint | RMSE ginocchio actual–prescribed | RMSE caviglia actual–prescribed | Escursione ginocchio media | Cicli validi totali | Eventi invalidi totali | Return deterministico medio |
|---|---:|---:|---:|---:|---:|---:|
| CP200 | 14,74° | 10,45° | 17,54° | 7 | 6 | 272,44 |
| CP250 | 15,10° | 11,49° | 18,99° | 6 | 7 | 263,44 |
| CP283, best training | 14,99° | 13,99° | 22,04° | 5 | 15 | 232,71 |
| CP300 | 14,85° | 12,13° | 20,41° | 5 | 14 | 255,79 |

**CP300 non riduce l'errore angolare del ginocchio rispetto a CP200 su nessuno dei tre start. Anche la caviglia peggiora su tutti e tre.** L'RMSE medio della caviglia cresce di circa il 16%. L'escursione globale del ginocchio aumenta di circa il 16%, ma resta molto inferiore ai **48,85°** del prescribed. Un aumento dell'ampiezza globale non assicura né la forma corretta né la corretta sincronizzazione della flessione durante il passo.

| CP300 | Early | Center | Late |
|---|---:|---:|---:|
| Durata | 5,00 s | 5,00 s | 5,00 s |
| Cicli validi | 2 | 1 | 2 |
| Eventi invalidi | 5 | 3 | 6 |
| RMSE ginocchio actual–prescribed | 14,99° | 14,76° | 14,82° |
| RMSE caviglia actual–prescribed | 13,36° | 11,77° | 11,26° |
| Escursione globale ginocchio | 23,07° | 21,10° | 17,08° |
| Penetrazione massima | 16,77 mm | 19,55 mm | 19,46 mm |
| Primo periodo online disponibile | 1,88 s | 3,22 s | 3,01 s |

La penetrazione recupera rispetto a CP250, che raggiungeva 21,73–24,18 mm. A CP300 torna sotto 20 mm nei tre rollout; rispetto a CP200 migliora leggermente in early e peggiora leggermente negli altri due start. I limiti SEA dello screening sono rispettati. Questa parte del risultato è positiva, ma non compensa la perdita di regolarità.

Gli eventi invalidi compaiono anche dopo che il periodo online è disponibile: in CP300 early già a 2,40 s, center a 3,77 s e late a 3,21 s. Fra gli eventi compaiono appoggi insufficientemente caricati, contatti troppo ravvicinati, heel-strike cancellati come rimbalzi e cicli respinti per escursione insufficiente del ginocchio. Non si tratta soltanto di un transitorio iniziale senza clock.

![Confronto delle metriche sui tre start](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/deterministic_comparison_200_300.png>)

**Lo screening di fattibilità non è superato.** CP300 fallisce per eventi invalidi in tutti gli start e per un solo ciclo valido in center. CP283 produce 3/1/1 cicli e 5/6/4 eventi invalidi in early/center/late. CP200 falliva già la condizione di zero eventi invalidi, ma aveva almeno due cicli su ogni start e meno eventi invalidi. Nessuno di questi checkpoint è quindi qualificato come cammino corretto e robusto.

## Perché il movimento resta diverso dal prescribed

Qui occorre distinguere tre segnali: **prescribed**, la traiettoria obiettivo; **served**, il riferimento effettivamente consegnato al controllore dopo mappatura, limitatore della variazione del target e governor; **actual**, la cinematica effettiva simulata. Gli errori sono calcolati agli stessi istanti fisici, senza traslare le curve per allinearne la fase.

A CP300 il RMSE actual–served è soltanto **0,43–0,46° al ginocchio** e **0,35–0,46° alla caviglia**. L'esecuzione segue bene il riferimento ricevuto, ma il riferimento non riproduce bene il prescribed. Il problema osservato non è quindi spiegato principalmente da un grande errore di inseguimento del riferimento servito. Queste misure non separano le responsabilità della policy e dei filtri che trasformano la sua uscita.

Le traiettorie mostrano che manca ancora la flessione marcata del ginocchio presente nel prescribed. La caviglia presenta differenze di forma, ampiezza e fase, soprattutto nella seconda parte degli episodi. La penetrazione contenuta e il raggiungimento dei 5 s non bastano a certificare questa cinematica.

![Movimento effettivo e prescribed](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/kinematics_200_283_300.png>)

## Best, esplorazione e interpretazione del return

**CP283 è best soltanto secondo il return registrato durante il training.** Nelle prove deterministiche ha caviglia peggiore di CP300 su tutti gli start, più eventi invalidi complessivi e un solo ciclo valido sia in center sia in late. Non è il checkpoint migliore per una conclusione biomeccanica generale. Fra i quattro checkpoint confrontati, CP200 resta il riferimento più convincente per il compromesso fra tracking e regolarità, pur essendo ancora insufficiente.

Il return di training può crescere anche perché gli episodi durano più a lungo e la distribuzione degli stati cambia. La policy stocastica durante l'apprendimento e la policy media congelata nei rollout misurano comportamenti diversi. La selezione del best su un unico massimo rumoroso non garantisce superiorità nelle prove fisiche.

L'entropia media aumenta da **2,49** nelle iterazioni 181–200 a **3,07 nat** nelle 281–300. Sugli stati deterministici salvati di CP300, sigma mediana ginocchio/caviglia vale circa **2,25/1,80** in early, **1,05/0,77** in center e **0,99/0,76** in late, in coordinate di azione normalizzata. Non emerge un'esplorazione collassata. Queste sigma non sono misure della distribuzione completa degli stati di training né della variabilità fisica dopo clipping e filtri.

Il clipping interessa almeno una componente dell'azione media nel 17,0–22,6% degli step CP300, contro 2,6–5,6% a CP200. Resta il disallineamento noto: RLlib limita l'azione prima del wrapper di reward nel training; il rollout gli passa il valore grezzo e limita l'azione prima dell'esecuzione fisica. Per questo il valore zero della telemetria di clipping nel training non certifica assenza di saturazione. L'effetto diretto della diversa penalità su early è però inferiore a un punto di return: non spiega da solo il peggioramento cinematico o il calo del return deterministico. Dettagli nell'[audit delle metriche](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/runtime_and_metric_audit.md>).

## Conseguenza per il prossimo esperimento

Le 100 iterazioni aggiuntive sono state informative: mostrano che aumentare il budget, mantenendo questa procedura, non assicura un progresso nella qualità del cammino. Il beneficio comportamentale osservato passando da 100 a 200 non si ripete passando da 200 a 300.

**Non considererei ora giustificato un altro blocco identico di 100 iterazioni come scelta prioritaria.** Prima servirebbe un esperimento che discrimini il limite: confronto omogeneo con la rete della fase C, oppure analisi della generazione del riferimento e del ruolo dell'informazione di fase. La selezione dei checkpoint dovrebbe includere tracking, cicli ed eventi sui tre start, oltre al return. Nemmeno aumentare automaticamente l'esplorazione è sostenuto dalle misure attuali.

Questo non dimostra che la V1 senza prescribed nell'actor non possa mai raggiungere una buona camminata, né che 300 sia un limite matematico. Non distingue definitivamente informazione insufficiente, ottimizzazione, reward e filtri. Dimostra che i checkpoint finali misurati non migliorano globalmente il risultato a 200. Il confronto con C50 richiede inoltre protocolli e asset omogenei: i dati Windows non sono qui mescolati alla tabella deterministica locale.

## Recupero della valutazione, artefatti e verifiche

L'osservatore automatico aveva completato CP300 early, poi interrotto il confronto perché la configurazione esportata non includeva più il campo storico `max_delta_rad`. La modalità `absolute` usata dal run non utilizza quel parametro. Dopo l'audit, il confronto di configurazione è stato adattato esclusivamente negli strumenti di analisi: viene ignorata quella sola chiave nella modalità absolute, mentre differenze di modalità, reward e soglie continuano a essere rifiutate. Lo stato di errore dell'osservatore originale è conservato come documento storico; la valutazione recuperata è completa.

Sono state rilevate modifiche concorrenti al repository, documentate nell'[audit del runtime](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/runtime_and_metric_audit.md>) e nel [diff archiviato](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/runtime_changes_after_launch.diff>). La successiva centralizzazione dei default GRF è avvenuta dopo il caricamento dei cinque processi di recovery; configurazioni GRF e reward emesse coincidono nei sei rollout finali. I 30 artefatti parent controllati sono intatti. Non si dichiara il repository interamente immutato e non sono state annullate modifiche concorrenti.

Verifiche completate: identità e hash dei checkpoint e dei dati storici; 12 casi e 18 confronti su finestre comuni; contatori FSM coerenti; replay NumPy delle azioni medie su 6.000 osservazioni, errore massimo 8,35e-7; ispezione delle figure; controllo sintattico dei due nuovi strumenti. Queste verifiche non costituiscono una validazione statistica su nuovi start, più seed o durate maggiori di 5 s.

- [Dati training e provenienza](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/training_analysis.json>), [serie 1–300](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/training_series.csv>), [dettaglio del proseguimento](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/training_181_300.png>).
- [Valutazione completa](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/milestone_evaluation.md>), [metriche JSON](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/milestone_evaluation.json>), [sintesi CSV](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/evaluation_summary.csv>), [replay e diagnostica](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/evaluation_diagnostics.json>).
- [Grafico training in PDF](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/training_1_300.pdf>), [confronto in PDF](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/deterministic_comparison_200_300.pdf>), [cinematica in PDF](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/kinematics_200_283_300.pdf>).
- Nuovi strumenti: [evaluate_training_300.py](<../../Trajectory Generator/unified_MLP/validation/evaluate_training_300.py>) e [analyze_training_300.py](<../../Trajectory Generator/unified_MLP/validation/analyze_training_300.py>); calcolo training riproducibile con [training_compute.py](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/training_compute.py>).

Le modifiche eseguite per questa analisi riguardano strumenti di valutazione, report e stato del piano. Nessuna modifica a checkpoint, reward, architettura o plugin SEA; nessun nuovo aggiornamento della policy.

## File creati o modificati nel lavoro documentato

| Stato | File o cartella | Scopo |
|---|---|---|
| Creato | [validation/evaluate_training_300.py](<../../Trajectory Generator/unified_MLP/validation/evaluate_training_300.py>) | Recupero e confronto deterministico di CP200, CP250, CP283 e CP300. |
| Creato | [validation/analyze_training_300.py](<../../Trajectory Generator/unified_MLP/validation/analyze_training_300.py>) | Replay offline della policy, sintesi delle metriche e grafici comparativi. |
| Creato | [runs/validation/pretraining_v1_continuation_300_2026-09-17/training_compute.py](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/training_compute.py>) | Calcolo riproducibile delle statistiche di training e delle curve 1–300. |
| Creati | [runs/validation/pretraining_v1_continuation_300_2026-09-17](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17>) | Report, JSON/CSV, grafici PNG/PDF, piano di valutazione e audit di compatibilità/provenienza. |
| Aggiornato | [runs/plans/pretraining_cp200_to300_2026-09-16/STATUS.md](<../../Trajectory Generator/unified_MLP/runs/plans/pretraining_cp200_to300_2026-09-16/STATUS.md>) | Stato finale del training e della valutazione, con conclusioni e collegamenti. |
| Creato | [runs/plans/pretraining_cp200_to300_2026-09-16/completion_verification.json](<../../Trajectory Generator/unified_MLP/runs/plans/pretraining_cp200_to300_2026-09-16/completion_verification.json>) | Ricevuta delle verifiche di integrità, completamento e replay. |

Il presente comando `create_report` aggiunge soltanto questo report in `reports/user/`. I risultati e le verifiche descritti sono quelli dell’analisi già completata; non vengono rilanciati rollout o training. Le modifiche concorrenti del runtime appartengono ad altro lavoro e sono descritte separatamente negli audit.

## Verifica del report

Riletta l’analisi sorgente e verificata la ricevuta di completamento: training a 300, 12 casi valutati, 18 confronti temporali e nessun nuovo training. Verificati esistenza e risoluzione dei collegamenti locali, incluse le figure.
