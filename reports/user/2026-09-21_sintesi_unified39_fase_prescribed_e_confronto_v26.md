# Unified 39D: fase prescribed, correzioni software e confronto imitativo con V26

Data del report: 21 settembre 2026. Attività riepilogate: 17–20 settembre 2026.

Instruction check token: CMC_AGENT_OK_2026.

## Esito

La rete unificata 39D è operativa per il training: il problema della maschera nei worker è stato risolto, il target è indicizzato dalla fase prescribed sinistra e il clock sin/cos è nuovamente disponibile all'actor, predisposto per l'oscuramento insieme ai quattro target q/qdot.

**La qualità imitativa del V26 del 20 agosto non è stata recuperata.** Tre training completi da 100 iterazioni hanno isolato scala delle velocità e slew limiter, mantenendo il clock visibile. La variante con velocità non divise per 10 è la candidata più utile per proseguire l'indagine, ma completa soltanto due dei tre rollout e riproduce ancora male le ampiezze richieste.

Questo report consolida implementazione, strategia, verifiche e conclusioni. Il [resoconto analitico del 20 settembre](2026-09-20_risultati_ablazioni_unified39_clock_scala_slew.md) contiene tutte le metriche per start, la decomposizione della reward e i collegamenti ai 21 grafici canonici. Il 21 settembre è stato creato questo documento; non sono stati eseguiti ulteriori training o modifiche del runtime.

## Problema e obiettivo

L'obiettivo dell'utente è verificare che la 39D possa raggiungere risultati imitativi comparabili al training V26 del **20 agosto**, prima di studiare l'oscuramento progressivo delle feature prescribed.

Il riferimento identificato nei report è `MLP_imitation_native_v26_08-20-2026_june_equiv_100iter`, checkpoint best **87**. È la rete imitativa, non il successivo training ex-novo.

Il confronto richiesto mantiene inizialmente tutti i prescribed visibili. L'utente ha accettato l'utilizzo del target sinistro e le penalità aggiuntive rispetto al V26. La richiesta di equivalenza riguarda quindi il risultato imitativo rispetto al proprio target, senza pretendere identità dei dati cinematici fra i due esperimenti.

La normalizzazione degli errori rispetto alle escursioni del rispettivo target va distinta dalla scala degli ingressi della rete. L'ablazione `/10 → /1` riguarda esclusivamente le **velocità osservate dall'actor**, non i pesi o le scale delle loss imitative.

## Soluzione implementata

### Target basato sulla fase della gamba sinistra

Il V26 storico costruiva un target periodico dalla gamba destra e lo sfasava di mezzo ciclo. La 39D utilizza gli heel strike prescribed sinistri per determinare la fase e interrogare un template periodico delle coordinate protesiche sinistre, senza shift.

Il template riutilizza il ricampionamento in fase, la media dei cicli e la spline periodica del V26. La velocità target deriva dalla derivata rispetto alla fase, convertita in rad/s usando il periodo locale. Il riferimento è quindi una traiettoria indicizzata dalla fase prescribed, non un semplice lookup dell'angolo sinistro al tempo corrente. Il metronomo rimane ricavato dai dati registrati: non è la fase online prodotta autonomamente dalla protesi simulata.

La prima implementazione, contratto v2, usava il clock per costruire il target della reward ma lasciava nulli i due ingressi sin/cos dell'actor. Dopo la diagnosi del primo training, questi due ingressi sono stati reintrodotti nel contratto **`unified_mlp39_v3`**.

### Maschera comune a sei ingressi

Gli indici 0 e 1 contengono `sin(2πφ_left)` e `cos(2πφ_left)`; gli indici 35–38 contengono posizione e velocità target di ginocchio e caviglia. Un'unica estrazione episodica controlla la disponibilità di tutti e sei i canali. Le dimensioni restano **39 actor / 88 osservazione completa**.

Quando gli ingressi vengono oscurati, il teacher rimane disponibile al calcolo della reward imitativa. Nel passaggio ex-novo/critic warmup i sei ingressi vengono azzerati anche a livello del modulo, mentre il gating della reward esclude i termini imitativi attivi in ex-novo. Questa predisposizione software non dimostra che la policy sappia già camminare autonomamente senza prescribed.

La scala delle velocità è configurabile tramite `--target-velocity-scale` e `target_availability.target_velocity_scale`. Il contratto salva la scala e impedisce riprese o inizializzazioni con semantiche incompatibili. I checkpoint v1/v2 non vengono convertiti automaticamente in v3.

### Correzione della gestione dei worker

RLlib espone gli ambienti attraverso `DictInfoToList(SyncVectorEnv(...))`. Il callback precedente cercava gli ambienti al livello sbagliato e trattava il wrapper vettoriale come un ambiente singolo, impedendo l'avvio corretto di PPO.

La correzione rimuove il wrapper vettoriale, enumera gli ambienti interni e raggiunge il controller della maschera di ciascuno. Rimangono controlli su copertura dei worker, contatori, larghezza vettoriale e ripristino dello stato RNG. Lo stesso fix è stato applicato al payload del builder storico per evitare di rigenerare il difetto.

## Strategia sperimentale e sequenza eseguita

La prima fase ha verificato il software mediante test unitari, prove OpenSim e smoke PPO con salvataggio/ripresa. È seguito un training da 50 iterazioni e una continuazione dal best, arrivata al checkpoint 99; su richiesta è stata interrotta durante l'iterazione 100, conservando il best 54. Il suo rollout ha mostrato una qualità nettamente inferiore al V26.

Per separare le ipotesi emerse dalla diagnosi sono poi stati eseguiti tre run nuovi, tutti con clock sinistro visibile:

| Variante | Scala qdot osservata | Slew ginocchio/caviglia | Iterazioni | Best |
|---|---|---|---:|---:|
| A — baseline_phase | qdot / 10 | 2,5 / 2,0 rad/s | 100 | 99 |
| B — velocity_scale1 | qdot / 1 | 2,5 / 2,0 rad/s | 100 | 78 |
| C — no_slew | qdot / 10 | disattivato | 100 | 84 |

Condizioni comuni: pesi iniziali identici, seed 123, MLP 256×256 tanh, 12 worker, batch di 4.608 transizioni, minibatch 512, 10 epoche PPO per iterazione. Il governor Butterworth a 4 Hz e gli altri limiti successivi restano attivi anche in C. Reward, target sinistro, reset e start sono uguali fra le tre varianti.

La disponibilità dei sei ingressi prescribed è rimasta **100%**, senza curriculum. Ogni batch contiene esattamente 1.536 transizioni per ciascuno degli offset 1,756870983805102, 1,956870983805102 e 2,156870983805102 s.

Il best è selezionato dal massimo return medio di training. Per ogni best sono stati eseguiti tre rollout deterministici, fino a 500 step/5 s, e prodotti i sette grafici canonici sullo start centrale. La sequenza è terminata il **20 settembre alle 09:29**, dopo circa **45 ore e 13 minuti**: 300 iterazioni, 1.382.400 transizioni, nove rollout e 21 grafici canonici.

## Risultati

| Variante | Best return di training | Rollout completi | Durata early / center / late |
|---|---:|---:|---|
| A — Clock, scala 10, slew | 295,47 | 1/3 | 5,00 / 1,70 / 1,48 s |
| B — Clock, scala 1, slew | 281,15 | 2/3 | 5,00 / 5,00 / 1,51 s |
| C — Clock, scala 10, senza slew | 241,68 | 0/3 | 1,89 / 1,71 / 1,49 s |

Per confrontare episodi di diversa durata sono stati ricalcolati gli errori sui prefissi comuni a ciascuno start: 1,89 s early, 1,70 s center e 1,48 s late. B migliora entrambi i giunti in tutti e tre i confronti; C li peggiora tutti rispetto ad A. Il vantaggio di B non è uniforme per tutta la durata: sull'episodio early completo migliora il ginocchio di circa 4,4% ma peggiora la caviglia di circa 4,9%.

Il confronto storico seguente usa solo rollout completi di 5 s, ciascuno rispetto al proprio target:

| Policy | RMSE ginocchio | RMSE caviglia | Flessione massima realizzata / richiesta |
|---|---:|---:|---:|
| V26 agosto, best87 | 6,91° | 1,46° | 63,78° / 64,32° |
| 39D precedente senza clock, best54 | 14,21° | 8,49° | 25,67° / 56,24° |
| 39D B, best78, start centrale | 12,39° | 7,46° | 28,92° / 56,24° |

![Confronto dei rollout completi](<../../Trajectory Generator/unified_MLP_39/validation/ablation_report_2026-09-20/historical_comparison.png>)

Il confronto con V26 è descrittivo: oltre ai target diversi, lo storico aveva un solo start a offset 1,0 s, 13 worker e batch 4.096. Le differenze non consentono di attribuire il divario a una singola causa. Un solo seed e tre reset non costituiscono una verifica statistica di equivalenza.

## Diagnosi sostenuta dai dati

**Gli ingressi arrivano correttamente e influenzano la policy.** Nei nove trace sono state verificate fase, clock, scala e corrispondenza temporale fra osservazioni e target. Le sonde offline mostrano dipendenza dal clock reintrodotto e una maggiore sensibilità alle velocità target nella variante B. Non dimostrano però prestazioni in un rollout con feature oscurate.

**Il difetto dominante è già nella reference generata.** In B centrale l'RMSE reference servita–target è 12,49°/7,43°, quasi pari al difetto del giunto rispetto al target. Il tracking giunto–reference è invece circa 0,55°/0,59°. Questi dati non indicano il plugin SEA come primo componente da modificare.

**Le interruzioni non sono esplosioni numeriche.** Nei sei rollout terminati il ginocchio attraversa il limite superiore di 0°, con valori finali da +0,0246° a +0,1519°. La reference servita attraversa anch'essa lo zero, mentre il target richiederebbe ancora circa 11–12° di flessione. Una soglia più permissiva potrebbe allungare l'episodio, senza correggere l'imitazione già insufficiente prima dello stop.

**Il return non certifica parità cinematica.** A raggiunge un return medio nelle ultime 20 iterazioni di 257,09, quasi identico ai 257,43 del V26, ma con episodi più brevi e traiettorie peggiori. Inoltre il return che elegge il best proviene dal campionamento esplorativo, mentre il checkpoint viene salvato dopo l'aggiornamento PPO: non è una valutazione deterministica dedicata dei pesi appena salvati.

**Esplorazione, clipping e contatto restano temi da isolare.** In A/B lo slew modifica circa 85–92% dei comandi deterministici dei rollout centrali. In B aumentano la sensibilità ai target e la durata di alcuni episodi, ma anche l'escursione della caviglia e i costi di contatto. Questi meccanismi sono coerenti con il divario osservato, ma non è stata dimostrata una singola causa sufficiente.

## File principali modificati o aggiunti nel lavoro riepilogato

Tutti i percorsi della tabella sono relativi a `Trajectory Generator/unified_MLP_39/`, salvo dove specificato.

| File o gruppo | Scopo |
|---|---|
| `phase_target.py`, `env.py` | Template sinistro per fase, integrazione teacher/target e clock actor |
| `target_availability.py` | Maschera comune, scala qdot, contatori/RNG, correzione wrapper worker |
| `env_factory.py`, `asymmetric_rl_module.py` | Propagazione della scala e azzeramento dei sei canali in ex-novo/warmup |
| `actor_transition.py`, `runtime_contract.py` | Contratto v3 e incompatibilità fra semantiche/scale differenti |
| `training_config.py`, `training_runtime.py`, `train_ppo_mlp.py`, `rollout_eval.py` | CLI/YAML, checkpoint, ripresa e valutazione coerenti |
| `configs/*.yaml`, `README.md`, `tests/` | Preset, documentazione e verifiche della variante |
| `validation/verify_left_phase_runtime.py` | Verifica OpenSim dei sei ingressi e dell'indipendenza della reward dalla maschera |
| `validation/run_clock_ablations.py` | Sequenza automatica dei tre training, audit, rollout e confronto |
| `validation/render_canonical_rollout.py` | Sette grafici ancorati al rollout e al target sinistro effettivi |
| `validation/diagnosis_v26_2026-09-18/` | Prima diagnosi, sonde della policy e replay offline del percorso di comando |
| `validation/ablation_report_2026-09-20/` | Analisi finale, metriche JSON, hash, quattro grafici riassuntivi e generatore report |
| `runs/plans/phase_clock_ablation_2026-09-18/` | Configurazioni dei tre esperimenti, log, ricevute e snapshot |
| `Trajectory Generator/unified_MLP/validation/feasibility39_bundle/target_availability.py` | Medesima correzione worker nel payload storico |

Sono stati preservati plugin C++, semantica del comando SEA e checkpoint storici. Per il dettaglio della prima implementazione vedere la [ricevuta del 17 settembre](<../../Trajectory Generator/unified_MLP_39/validation/2026-09-17_left_phase_implementation.md>): descrive lo stato v2 di allora, superato dal clock actor e dalla maschera a sei canali del v3.

## Test e verifiche eseguite

- Prima implementazione di fase e worker: 61 test distinti documentati il 17 settembre, prove OpenSim sui tre start e smoke PPO con salvataggio/ripresa. Il conteggio si riferisce a quello stadio, non va sommato ai test successivi come copertura corrente.
- Contratto v3 e sei ingressi: 55 test unitari superati; OpenSim con scale 1 e 10, su tre start, disponibilità 1 e 0; target, loss e reward invariati a parità di azione quando cambia la maschera.
- Smoke v3: due aggiornamenti PPO da 192 transizioni, il secondo dopo ripresa completa; 384 transizioni visibili cumulative e digest actor/critic ripristinati esattamente. Rollout deterministico di cinque step verificato. Queste sono prove infrastrutturali, separate dai training completi.
- Esperimenti completi: 300 iterazioni consecutive senza skip, nessun riavvio del supervisore, batch bilanciati, loss finite e pesi iniziali identici. Le terminazioni fisiche degli episodi non sono conteggiate come crash del training.
- Analisi finale: nove rollout nuovi e due storici, ricalcolo degli RMSE e di undici return; errore massimo nella ricostruzione della reward circa **3,98×10⁻¹³**. Caricamento di cinque checkpoint e riproduzione delle azioni entro **8×10⁻⁷**.
- Manifest dei 100 file congelati verificato nell'analisi del 20 settembre; controllo dei 21 PNG canonici, produzione e ispezione di quattro figure comparative. La validazione live è stata svolta su macOS arm64; manca una verifica live Windows con le DLL corrispondenti.

Le prove e le metriche sopra sono quelle già eseguite e archiviate. Per questa richiesta `create_report` sono stati riletti i documenti e le ricevute e verificati i collegamenti del nuovo report; non è stata ripetuta la campagna sperimentale.

## Questioni aperte e seguito raccomandato

La prima questione è la fattibilità del target sinistro nell'intero loop fisico: è raccomandato un oracle che generi i comandi dal prescribed per fase, con anticipazione controllata per il filtro, sugli stessi tre start. Il replay offline del solo governor non risponde a questa domanda.

La seconda è la selezione della policy: servono valutazioni dedicate dei checkpoint con durata, errori cinematici e contatto, confrontando anche azioni deterministiche e campionate a seed fissati. Un insieme di valutazione separato deve poi evitare di confondere selezione e verifica finale.

B costituisce il riferimento proposto per ulteriori prove, mantenendo clock e target visibili; non è stata promossa automaticamente a configurazione predefinita. La combinazione scala 1 senza slew non è stata provata. Anche il significato della guardia a 0° rimane da valutare, distinguendo sicurezza articolare e fedeltà al target.

L'oscuramento progressivo è predisposto nel software ma **non è stato sperimentato**. La raccomandazione è rimandarlo finché l'imitazione con visibilità completa non è soddisfacente. Per dichiarare parità con V26 occorreranno margini espliciti, più seed indipendenti e condizioni di valutazione concordate. Queste sono proposte di seguito, non attività già avviate.

## Artefatti di riferimento

- [Analisi completa del 20 settembre](2026-09-20_risultati_ablazioni_unified39_clock_scala_slew.md).
- [Protocollo dei tre esperimenti](<../../Trajectory Generator/unified_MLP_39/runs/plans/phase_clock_ablation_2026-09-18/README.md>).
- [Ricevuta preflight v3](<../../Trajectory Generator/unified_MLP_39/runs/plans/phase_clock_ablation_2026-09-18/preflight.json>).
- [Risultati originali dei tre run](<../../Trajectory Generator/unified_MLP_39/runs/plans/phase_clock_ablation_2026-09-18/results.json>).
- [Metriche e diagnosi ricalcolate](<../../Trajectory Generator/unified_MLP_39/validation/ablation_report_2026-09-20/metrics.json>).
- [Ricevuta delle verifiche del resoconto](<../../Trajectory Generator/unified_MLP_39/validation/ablation_report_2026-09-20/report_verification.json>).
- [Riferimento imitativo V26 del 20 agosto](2026-08-20_verdetto_fresh_run_june_equivalent_nativo_v26.md).
