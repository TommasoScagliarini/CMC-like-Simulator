# Daily Report — 2026-09-29

Chiusura della giornata su richiesta `end_day`. Questo documento consolida il report utente del 29 settembre, gli esiti sperimentali precedenti in esso riepilogati e la verifica della copertura dei daily. Sostituisce il precedente aggiornamento parziale del 29 settembre.
Instruction check token: `CMC_AGENT_OK_2026`.

## Stato alla chiusura e punto di ripartenza

Consolidati gli esiti delle attività del 25–28 settembre nel [report utente](../user/2026-09-29_esiti_pilot_reward_left_e_diagnosi_kl.md). Oggi l'attività è documentale: nessun training o rollout nuovo.

Best80 LEFT con penetrazione peso 2 resta il riferimento. Ankle 10% e ankle 5% sono respinte dai vincoli; nessuna delle intermedie 82–83 supera lo screening. I sei nominali del 28 settembre sono conclusi in 17 min 24 s, con 42 plot canonici e nessun PPO. Le prove rumorose aggiuntive non sono scattate perché non vi sono candidate ammissibili.

Verificato il dimezzamento automatico del coefficiente KL 0,30 → 0,15 durante critic-only, nonostante la conservazione di actor/logstd/Adam. Preservarlo nel rewarm è un'ipotesi da isolare: nessun fix implementato né nuovo training predisposto. La diagnosi dei cicli FSM e dell'area per fase non modifica retroattivamente i criteri.

## Consolidamento del report della giornata

Fonte: [Esiti dei pilot reward LEFT e diagnosi KL](../user/2026-09-29_esiti_pilot_reward_left_e_diagnosi_kl.md), unico user report datato 29 settembre presente alla chiusura. La data del documento è distinta dalle esecuzioni del 25, 26 e 28 settembre.

### Problema e strategia

Il banco di tuning rapido separa il ricalcolo offline sui rollout esistenti dall'effetto appreso in brevi continuazioni. Il corpus comprende 17 rollout e nove intensità calibrate, una famiglia alla volta. Tracking e contatto sono vincoli; seguono area positiva SEA ankle, regolarità coppia knee e cinematica. V26 e RIGHT sono confronti descrittivi per la differenza dei target. Le scale dei nuovi costi sono esplicite, non soglie fisiologiche; i pesi predefiniti sono zero e i costi vengono sottratti dopo il clipping soltanto in imitazione.

I pilot ripartono dal checkpoint completo best80: un aggiornamento critic-only e tre PPO, batch 4096, episodi di 5 s e seed training 123. Prescribed e clock rimangono visibili. Le valutazioni nominali e rumorose 123/125/127 condividono gaussiane e sigma congelata best80, senza implicare identità delle traiettorie raccolte dai worker.

I vincoli originali restano invariati: aumento RMSE per giunto e fase al massimo `min(10% del riferimento, 0,5°)`, escursione e impulso almeno al 90%, completamento e cicli non ridotti, penetrazione nominale sotto 20 mm e incremento comparativo massimo 2 mm, nessuna nuova terminazione hard. Per il beneficio ankle servono almeno −10% di area nominale, nessun aumento mediano rumoroso e nessun peggioramento della regolarità oltre il 10%. Non si selezionano policy confrontando ritorni di reward diverse.

### Esiti fisici e costi

Metriche nominali sull'intero episodio 12,99–17,99 s; la finestra storica 13,15–17,95 s resta distinta nei report sorgente.

| Policy | Area ankle+ [Nm·s] | RMSE knee [°] | RMSE ankle [°] | Cicli FSM | Impulso [N·s] |
| --- | ---: | ---: | ---: | ---: | ---: |
| Best80 | 8,650 | 6,344 | 3,997 | 2 | 389,545 |
| Controllo 84 | 25,611 | 6,208 | 6,009 | 2 | 465,945 |
| Ankle 5%, iter. 84 | 9,029 | 5,591 | 3,293 | 1 | 468,214 |
| Ankle 10%, iter. 84 | 3,036 | 7,347 | 4,021 | 2 | 345,357 |

Ankle 10% (peso 1,3379186063656872) riduce l'area nominale del 64,9%, ma viola tracking/supporto e il seed 127 termina dopo 0,97 s con `invalid_binary_event`. L'area di un episodio interrotto non è un successo. Ankle 5% (peso 0,6689593031828436) completa tutti gli scenari, ma l'area cresce del 4,4% rispetto a best80; la mediana rumorosa delle differenze appaiate cresce di 1,931 Nm·s rispetto a best80 e 3,361 Nm·s rispetto al controllo. Entrambe sono `rejected_constraints`.

Il pilot controllo/ankle 10% del 25 settembre richiede 105,16 minuti; quello ankle 5% del 26 settembre 39,79 minuti, riutilizzando il controllo e otto rollout di riferimento. I sei nominali intermedi del 28 settembre richiedono 17,39 minuti. Tutti restano entro il proprio budget di 120 minuti; i riferimenti riutilizzati non sono repliche indipendenti.

Alle iterazioni 82/83/84 le aree sono rispettivamente 5,537/14,992/25,611 nel controllo, 15,860/17,203/9,029 in ankle 5% e 12,285/10,332/3,036 in ankle 10%. Il confronto con best80 e il controllo alla stessa iterazione non trova intermedie ammissibili. Anche il controllo 82, con area inferiore, fallisce supporto e altri vincoli. Non è emersa una soluzione da recuperare fermando prima questi training.

### Diagnosi e limiti

Il coefficiente KL dimezzato nel critic-only è un comportamento verificato di RLlib, non una causa dimostrata delle regressioni. Un eventuale esperimento che lo conservi a 0,30 richiede anche un controllo con lo stesso trattamento e il normale adattamento KL riattivato con PPO. Configurazioni e fix non sono ancora predisposti.

La convergenza del critic dopo un aggiornamento non è dimostrata: explained variance 0,671/0,663/0,658 e loss VF limitato non sostituiscono la distribuzione degli errori e una validazione indipendente. La forte deriva del controllo non permette di attribuire tutte le differenze alla nuova reward.

Il ciclo FSM in meno di ankle 5% deriva da HS precoce, resync e ciclo degradato; best80 conta anche un ciclo breve di 0,769 s. Il conteggio resta un vincolo, ma non dimostra un'oscillazione fisica persa. L'82,4% della riduzione d'area ankle 10% si concentra nella porzione prescribed 17,17–17,99 s, dove diminuisce anche l'impulso; nei due cicli completi precedenti l'impulso aumenta. Non si equiparano segno della coppia e segno della forza verticale.

Un seed di training e tre prove rumorose costituiscono screening operativo. Oscuramento, ex-novo da 100 iterazioni e tuning simultaneo delle famiglie restano fuori dall'esecuzione conclusa.

### File e verifiche

Il report odierno documenta le precedenti estensioni sotto `Trajectory Generator/unified_MLP_39/`: orchestrazione e report in `reward_tuning/`, configurazione `configs/reward_tuning_left_ankle5.yaml`, script `validation/evaluate_pilot_intermediates_2026_09_28.py`, `compare_pilot_intermediates_2026_09_28.py`, `diagnose_reward_phases_2026_09_28.py` e relativi test. Il cambiamento iniziale di `simulation_runner.py` è diagnostico; controller e plugin restano invariati.

Verifiche pregresse consolidate: replay di 17 ritorni entro 4,55e-13; smoke OpenSim 2×5 segmenti senza PPO con stati identici e reward entro 1e-12; 208 test della suite al preflight ankle 5% e otto test mirati degli intermedi superati. Audit di 12 checkpoint/403 hash; manifest intermedi con 631 input; sei rollout completi e 42 plot canonici sotto root `plot/`, verificati con SHA-256. Riconciliati 2000 segmenti e 20000 campioni SEA, somme positive entro 2,79e-8 Nm. Rumore, sigma, prescribed e conservazione actor/logstd/Adam verificati.

Le finestre semiaperte della diagnosi per fase e il campione superiore incluso nello screening storico sono espliciti. Nessun dato mancante è sostituito con zero o usato per promuovere una candidata. Windows live resta da verificare.

Oggi sono stati scritti il report utente e questo daily, controllando i collegamenti agli artefatti e la propagazione dei TODO. I test numerici sopra non sono stati rilanciati per la chiusura documentale; codice, checkpoint e manifest congelati non sono stati modificati.

## Verifica della copertura dei daily

Il confronto è basato sulle date nei nomi dei file, non sulle date di modifica del filesystem. A settembre risultano user report privi di un daily della stessa data:

| Data | User report |
| --- | ---: |
| 2026-09-06 | 1 |
| 2026-09-17 | 1 |
| 2026-09-20 | 1 |
| 2026-09-21 | 4 |
| 2026-09-22 | 4 |
| 2026-09-23 | 7 |

Sono 18 report su sei date; quello del 6 settembre è richiamato e consolidato nel daily del 14 settembre. L'assenza di un daily della stessa data non dimostra da sola l'assenza di qualunque riepilogo successivo. La verifica non certifica la copertura integrale di tutti questi report nei daily successivi.

Nell'archivio precedente mancano daily della stessa data per 17 aprile (quattro report), 18 aprile, 11 maggio e 13 giugno (un report ciascuno). Il 24 settembre ha una chiusura esplicita `end_day`; il 25 conserva un daily parziale. Il 29, inizialmente parziale, è ora chiuso con questo documento. Non sono stati creati daily retroattivi.

## Attività chiuse rispetto al 25 settembre

- [x] Verificato training RLlib live: restore, reward nei worker, prescribed visibili, riadattamento del critic, conservazione actor/Adam e tre PPO per ramo.
- [x] Completati i nominali e i tre scenari rumorosi dei pilot, i plot e il confronto fisico entro i rispettivi budget.
- [x] Eseguita la decisione successiva al pilot ankle 10%: provata ankle 5%, analizzati learner/fasi e valutati gli intermedi; best80 mantenuto.

## TODO del nuovo report

- [ ] Isolare in un nuovo esperimento la conservazione del coefficiente KL durante il critic-only, con controllo sottoposto allo stesso trattamento; definire configurazioni e budget prima dell'esecuzione. Il fix non è implementato né il training predisposto.
- [ ] Verificare la sufficienza del riadattamento del critic con diagnostica degli errori e validazione indipendente; un aggiornamento non dimostra convergenza.
- [ ] Sperimentare separatamente regolarità della coppia knee e cinematica, mantenendo tracking e contatto come vincoli e senza modificare contemporaneamente le famiglie.
- [ ] Eseguire la verifica live Windows della pipeline e, prima di conclusioni sulla robustezza, repliche di training e condizioni di valutazione più ampie.

## Backlog precedente conservato

Le seguenti voci aperte provengono dal daily del 25 settembre e restano propagate. Non sono state riesaminate integralmente: le formulazioni storiche, anche ripetute o parzialmente superate, non autorizzano nuove esecuzioni. Il confronto ankle e i criteri di questo pilot sono ora completati; knee e la selezione generale prima dell'oscuramento restano aperti. La variante 39D esiste già; i richiami storici alla sua creazione non significano che debba essere ricreata. Restano distinte le verifiche RIGHT, il benchmark storico e i piani ex-novo.

- [ ] Dopo l'ankle, sperimentare separatamente regolarità knee e cinematica
  usando lo stesso banco e controlli; non cambiare insieme le tre famiglie.
- [ ] Eseguire verifica live Windows e, se una candidata è promettente,
  repliche/condizioni più ampie prima di conclusioni sulla robustezza.
- [ ] Riconciliare la collocazione delle figure diagnostiche reward LEFT e
  coppie best80, assenti nei percorsi storici alla chiusura; se servono ancora,
  localizzarle o rigenerarle dagli script conservati. I plot canonici non
  sono coinvolti in questa mancanza.
- [ ] Eseguire lo screening con perturbazioni comuni a policy congelata di
  best80 e precedente last82 contro best72; includere eventuali nuovi
  candidati se ancora utili. Misurare completamento, cicli, penetrazione,
  carico, cinematica per fase e coppie. La parte deterministica è conclusa,
  quella rumorosa no; best80 non è ancora certificato come più robusto.
- [ ] Eseguire il ramo controllo LEFT72 peso 1, con lo stesso fix hard,
  protocollo di resume, inizializzazione e budget del peso 2, se si continua
  il confronto causale. Non usare ritorni di reward diverse come criterio
  comune e non attribuire al peso 2 l'effetto dell'intero insieme di cambiamenti.
- [ ] Distinguere il miglioramento in continuazione dalle cause del divario
  nell'apprendimento LEFT da zero; serve un controllo dedicato.
- [ ] Se si interviene sulle coppie, progettare un confronto dal medesimo
  checkpoint e con budget identico: regolarità della coppia knee e momento
  ankle sotto carico, con scale esplicite e verifica di tracking/contatto.
- [ ] Fissare criteri distinti per completamento, cicli validi, forma delle
  traiettorie e coppie prima della selezione e dell'oscuramento; il solo
  ritorno o completamento temporale non basta.
- [ ] Quantificare separatamente il trascinamento degli ultimi angoli nel
  reset dei diagnostici di escursione FSM e l'incidenza nelle configurazioni
  che li utilizzano; nessun fix FSM applicato oggi.
- [ ] Eseguire la verifica live Windows del fix hard, della ripresa e della
  pipeline scelta. Le prove odierne attestano macOS arm 64.
- [ ] Pianificare repliche di training e condizioni di valutazione sufficienti
  per comparabilità statistica/generalizzazione, distinguendo esperienza,
  target e reward. Nessuna equivalenza statistica V26 è già dimostrata.
- [ ] Se si considera RIGHT last60 per riuso/oscuramento, valutarla sugli
  stessi cinque seed già usati per best58 prima della scelta.
- [ ] Approfondire il transitorio seed 125 confrontando best35, last40, best58,
  last60 e V26 su reference, velocità, contatto e margine di penetrazione.
- [ ] Isolare su RIGHT l'origine del momento GRF invertito nell'ultimo
  appoggio prima di modificarne reference, governor o reward. La diagnosi
  locale LEFT non chiude automaticamente questo controllo.
- [ ] Proseguire la stessa V1 da 50 a 100 iterazioni totali e svolgere la valutazione finale confrontando i risultati a 5, 50 e 100.
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

## Addendum: indagine generale e audit del segnale RL39 LEFT

Successivamente alla chiusura riportata sopra sono state completate due
indagini, documentate in
[imitazione e forma delle coppie](../user/2026-09-29_indagine_generale_39d_imitazione_e_forma_coppie.md)
e [audit RL, bootstrap e aggiornamento PPO](../user/2026-09-29_audit_RL_39d_bootstrap_e_aggiornamento_PPO.md).
Questo addendum conserva il backlog precedente e aggiorna le priorità sulla
base della richiesta dell'utente: metodo generale, nessun tuning imitativo
specifico per AB06, analisi dell'apprendimento RL senza teacher.

L'indagine generale aveva trovato una sequenza di comandi ottenuta dal target
futuro capace di migliorare fortemente il tracking LEFT nominale. Il risultato
dimostra fattibilità di quella sequenza nella catena di controllo, non che
la policy causale 39D possa apprenderla o che la sua coppia sia equivalente
a V26. Cambiava anche il carico sul piede: regolarità e forma biomeccanica
delle coppie vanno giudicate separatamente. La sensibilità del feedback
autoregressivo resta una pista, senza causalità unica dimostrata.

L'audit RL ha raccolto un batch on-policy di 4096 transizioni dal checkpoint
completo LEFT best80 e osservato una sola iterazione PPO, senza critic rewarm
o modifica di reward/rete/governor. Restore verificato: actor, critic, Adam,
learning rate e coefficiente KL effettivo 0,3000000119. È stato accertato un
difetto generale di Ray/RLlib 2.55.1: bootstrap eliminato ai confini artificiali
dei frammenti e propagazione errata prima di una vera terminazione. Nel primo
confine il target era 0,7296 anziché 43,0225. Il riproduttore senza OpenSim
e i sorgenti ufficiali confermano il difetto, corretto upstream dalla
[PR Ray 59958](https://github.com/ray-project/ray/pull/59958).

Il controllo sul medesimo batch riproduce bit per bit l'update originale,
inclusi 81 minibatch e stato Adam. Il ramo sperimentale corregge solo target
reali e advantages, conservando le righe artificiali e la normalizzazione
stock: non è un backport completo. I target reali coincidono con il codice
del fix ufficiale entro 3,82e-6. Cambia il segno del vantaggio normalizzato
nel 23,999% delle righe, anche per effetto della centratura globale.

Sono concluse 13 valutazioni, con sette plot canonici ciascuna sotto
`plot/09_29_2026_rl_learning_audit/`. Il nominale corretto recupera parte
della regressione stock: RMSE ankle 4,040° prima, 5,387° stock e 4,707°
corretto; ritorno 305,060, 255,186 e 279,378. La forma knee peggiora e le
coppie restano irregolari. Nei tre seed rumorosi il ritorno medio è 235,375,
240,997 e 272,142; il corretto supera stock in due seed su tre e completa
tutti gli episodi. Il solo fallimento è best80 seed127 a 458/500 passi,
incluso nelle statistiche. Sono esiti descrittivi, non superiorità statistica.

V26 registra lo stesso stack Ray: il bug non basta a spiegare il divario
LEFT–V26. PPO migliora il proprio obiettivo sul batch, ma riceve target
parzialmente errati. Non è dimostrata l'incapacità della rete di apprendere
con RL, né la sufficienza della correzione per raggiungere V26. Un update
da best80 non ripara la storia del training e non prova convergenza da zero.

Codice e ricevute sono in
`Trajectory Generator/unified_MLP_39/validation/rl_learning_audit_2026-09-29/`:
probe e launcher, analisi learner/esplorazione, replay, valutatori fisici,
confronti e verificatori. Aggiornati il README dell'audit, il report utente
e questo daily. Nessuna modifica a controller, plugin, reward o dipendenze
installate; il training separato da zero non è stato riconfigurato o fermato.
Passati 17 test mirati; verifica finale PASS su 125 input iniziali, 91 PNG
canonici, riproduzione del controllo, 13 ricevute, 12 dataset critic esterni
al batch e hash delle analisi. Verifica nativa su macOS arm64, non Windows.

## TODO aggiornati dopo l'audit RL

Queste voci propagano i TODO dei due nuovi report. Il precedente TODO sul
teacher inverse-governor resta una possibilità storica condizionata: non è
stato eseguito e non costituisce un prerequisito o il prossimo passo della
presente indagine. Anche il tuning delle famiglie di reward nel backlog
non è la priorità dell'indagine generale richiesta dall'utente.

- [ ] Integrare in un esperimento separato la correzione generale dei confini
  PPO, verificando connector completi, restore e riproducibilità. La patch
  di produzione non è stata applicata dall'audit.
- [ ] Quantificare separatamente la normalizzazione delle advantages sulle
  sole transizioni reali rispetto all'inclusione delle righe artificiali.
- [ ] Confrontare training RL standard e corretto con stessa inizializzazione,
  reward, budget e valutazioni; distinguere continuazione da best80 e training
  da zero, con repliche prima di conclusioni sulla stabilità.
- [ ] Proseguire la diagnosi di esplorazione, clipping, nominale/stocastico e
  feedback autoregressivo con interventi separati, senza tuning della reward
  sul solo AB06.
- [ ] Verificare altri start e tratti disponibili di AB06, recupero dopo
  perturbazioni e carichi; distinguere regolarità e forma della coppia senza
  imporre le waveform V26 come target a LEFT.
- [ ] Completare la verifica live Windows; verificare il trasferimento ad
  altri modelli quando disponibili. L'assenza attuale di altri modelli non
  impedisce l'indagine diagnostica sul modello disponibile.

<!-- reconciled-report-todos-2026-10-05:start -->
## TODO riconciliati e propagati — integrazione del 5 ottobre 2026

Ricostruzione documentale dello stato a questa data. Le voci sotto precisano i TODO storici ripetuti nelle sezioni precedenti: una nuova esecuzione o un PASS su un altro ramo non chiude il debito originario. Le attività condizionate restano proposte; questa integrazione non avvia esperimenti. Le chiusure riportano una fonte esplicita.

### Attività aperte

- [ ] Completare equivalenza host-target, latenza worst-case, HIL e contratto hardware prima di qualsiasi claim di deployment. — [fonte 2026-08-09, riga 342](../user/2026-08-09_h0_v12r2_training_readiness.md).
- [ ] Validare i candidati futuri su multistart, seed held-out, recovery, soggetti/profili esterni, rumore e delay. — [fonte 2026-08-09, riga 340](../user/2026-08-09_h0_v12r2_training_readiness.md).
- [ ] Conservare come TODO storico l'inizializzazione della deflessione SEA coerente con la coppia richiesta. — [fonte 2026-08-09, riga 344](../user/2026-08-09_h0_v12r2_training_readiness.md).
- [ ] Compilare/distribuire e verificare su Windows x86_64 le DLL SEA `_ff` e OnlineGRF corrette, con provenienza e ABI OpenSim/Simbody/Python; claim numerico separato dalla prova macOS. — [fonte 2026-08-09, riga 337](../user/2026-08-09_h0_v12r2_training_readiness.md).
- [ ] Rieseguire (opzionale, read-only) l'eval offline sul `corpus.npz` + modulo salvati in `v12r3/…/fit/p2/` per stabilire quale metrica bocciò il `fit_p2` e di quanto: unico esito della sessione senza numeri persistiti. — [fonte 2026-08-15, riga 181](../user/2026-08-15_retrospettiva_forense_sessione_09-10_agosto_v12r2_v12r4.md).
- [ ] Riconciliare il pin V20/V26 in train_ppo_mlp.py. — [fonte 2026-08-16, riga 192](../user/2026-08-16_v12r12_fix_pre_freeze_applicati_e_verificati.md).
- [ ] Chiarire l’anomalia teacher retry best==last==warm_start. — [fonte 2026-08-16, riga 192](../user/2026-08-16_v12r12_fix_pre_freeze_applicati_e_verificati.md).
- [ ] Valutare un override esplicito dello schema in rollout_eval.py per il checkpoint39/43. — [fonte 2026-08-18, riga 149](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md).
- [ ] Promuovere il driver di qualifica da scratchpad a script versionato alla prossima esecuzione della catena. — [fonte 2026-08-20, riga 95](../user/2026-08-20_risultato_training_ready_exnovo_B0820.md).
- [ ] Follow-up v3 fuori scope: riportare lo swing timeout da 2,6 a 1,3 s. — [fonte 2026-08-21, riga 95](../user/2026-08-21_fsm_v3_resync_validazione_e_bug_tolleranza_corridoio.md).
- [ ] Qualificare generalizzazione oltre AB06 su almeno un soggetto/modello EPIC completamente held-out da teacher, DAgger/PPO, iperparametri e corridor; più trial dello stesso soggetto non bastano. — [fonte 2026-08-22, riga 15](../user/2026-08-22_todo_generalizzazione_multimodello_epic.md).
- [ ] **Contratto FSM phase-aligned:** correggere o rendere univoca la semantica di `cycle_valid` dopo una cancellazione HS; fino ad allora i prodotti phase-aligned restano fail-closed. — [fonte 2026-08-23, riga 109](../user/2026-08-23_f1_s1_protocollo_tooling_dryrun.md).
- [ ] **Osservabilità RLlib:** nel prossimo eventuale run registrare environment configurati/sani, batch corrente/lifetime e generation/replacement dei runner in campi distinti. — [fonte 2026-08-23, riga 109](../user/2026-08-23_f1_s1_protocollo_tooling_dryrun.md).
- [ ] Chiarire perché B3/target_phase vale zero sotto runtime v 3 anche con cicli validi. — [fonte 2026-08-24, riga 212](../user/2026-08-24_v26b_audit_luglio_r1r2_e_progetto_rev4d.md).
- [ ] Decidere se archiviare il bundle J10 rifiutato, preservandolo byte-identico come evidenza. — [fonte 2026-08-27, riga 243](../user/2026-08-27_v26c_j10r1_multistart_teacher_collection.md).
- [ ] Decidere documentalmente sui due gcs_server orfani segnalati il 18 agosto; verificare stato e necessità prima di eventuali interventi. — [fonte 2026-08-27, riga 273](../user/2026-08-27_v26c_j20_restore_audit_r2_preparazione.md).
- [ ] Mantenere esplicita l’assenza storica di commit_verification in J2/J8 e di actor_digest in J2; evitare attestazioni retroattive inventate. — [fonte 2026-08-27, riga 233](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).
- [ ] Consolidare additivamente la correzione rad/secondi nei record V26C congelati, senza modificarne gli artefatti numerici. — [fonte 2026-08-27, riga 353](../user/2026-08-27_qualifica_closed_loop_j9r1.md).
- [ ] Chiarire/correggere in manutenzione separata policy_std=null nelle vecchie telemetrie J12–J19, senza inventare valori o riscrivere evidenza congelata. — [fonte 2026-08-27, riga 219](../user/2026-08-27_v26c_j16_j15r1_closed_loop_qualification_readiness.md).
- [ ] Chiarire path di staging e valid_cycle_count dei receipt J9R1 e i falsi heel strike del canale legacy 20 N; preservare i record e spiegare che non alterarono FSM/reward/verdetto. — [fonte 2026-08-27, riga 356](../user/2026-08-27_qualifica_closed_loop_j9r1.md).
- [ ] Documentare la provenance mancante dei runner nei vecchi leaf e i due runner J15/J15R1 verso la stessa leaf; i pin introdotti in J18 non colmano i precedenti. — [fonte 2026-08-27, riga 231](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).
- [ ] Conservare il limite di riproducibilità J11: artefatti senza versione Torch/argomenti Adam sufficienti; il J15R1 riproducibile non colma retroattivamente J11. — [fonte 2026-08-27, riga 232](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).
- [ ] Mantenere espliciti nei riepiloghi DIAGNOSTIC_NOT_BINDING contraddittorio e assenza di post-commit storico J9R1; le correzioni dei successori non riscrivono quel leaf. — [fonte 2026-08-27, riga 341](../user/2026-08-27_v26c_j12_closed_loop_qualification_readiness.md).
- [ ] Conservare e spiegare additivamente le discrepanze July su label raw/mean, gate ok:false e numeri di riga, senza riscrivere J0–J10R1. — [fonte 2026-08-27, riga 337](../user/2026-08-27_v26c_j11_multistart_fit_execution.md).
- [ ] Decidere e verificare la correzione del campo nominal_mean_shift dichiarato ma non calcolato dal runner J15R1; misura esterna conclusa, debito di codice distinto. — [fonte 2026-08-27, riga 234](../user/2026-08-27_v26c_j15r1_fresh_refit_execution.md).
- [ ] Se si vuole attribuire causalmente la deviazione pre-breccia, svolgere un confronto appaiato; una sola trace non isola i fattori confondenti. — [fonte 2026-08-27, riga 235](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).
- [ ] Indagare il clock swing non azzerato su hs_bounce_cancelled, distinguendolo dalla semantica cycle_valid; nessun fix è attestato da J19/R3/J21. — [fonte 2026-08-27, riga 429](../user/2026-08-27_v26c_j13_j12_failure_diagnosis_and_recovery_plan.md).
- [ ] Separare o reinterpretare best_validation_mse contaminato dalla ripetizione dei blocchi nei fit storici; non usarlo come validazione indipendente. — [fonte 2026-08-27, riga 234](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).
- [ ] Decidere come spiegare/gestire il marker sorgente RESTORE_AUDIT_PENDING immutabile dopo la chiusura esterna G9, senza mutare retroattivamente il checkpoint. — [fonte 2026-08-28, riga 250](../user/2026-08-28_v26c_j20_restore_audit_r3_esecuzione.md).
- [ ] Valutare le tre osservazioni non vincolanti R3 rev 7 (conversione downcast/None, mismatch raw R2, tipi optimizer registrati), separandole dai gate 13/13 passati. — [fonte 2026-08-28, riga 299](../user/2026-08-28_v26c_j20_restore_audit_r3_correzione_rev7_gate_learning_rate.md).
- [ ] Progettare e validare corridor cadence-conditioned su dati held-out. — [fonte 2026-08-30, riga 346](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).
- [ ] Valutare fascia soft interna e assegnazione causale del credito senza spostare il corridor finale. — [fonte 2026-08-30, riga 360](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).
- [ ] Verificare uno sweep 0–180 ms di anticipo della sola loss e il rapporto con il ritardo phase-normalized 135–160 ms; distinto dalla decomposizione già conclusa. — [fonte 2026-08-30, riga 376](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).
- [ ] Preregistrare il primo rapporto morphology knee/ankle asimmetrico. — [fonte 2026-08-31, riga 606](../user/2026-08-31_progetto_rete_unificata_imitativa_exnovo_actor_predittivo.md).
- [ ] Validare frozen-policy gli orizzonti circa 90 ms knee / 100 ms ankle su più cadenze e stati del governor. — [fonte 2026-08-31, riga 603](../user/2026-08-31_progetto_rete_unificata_imitativa_exnovo_actor_predittivo.md).
- [ ] Definire la loss imitativa raw-space/served-predicted-space per il nuovo teacher, prima di costruire la rete predittiva. — [fonte 2026-08-31, riga 604](../user/2026-08-31_progetto_rete_unificata_imitativa_exnovo_actor_predittivo.md).
- [ ] Riprodurre il benchmark deterministico della stessa baseline sui tre start: la sola prova centrale storica non basta. — [fonte 2026-09-06, riga 247](../user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).
- [ ] Confrontare nominale e stocastico con seed indipendenti di training e incertezza delle differenze. — [fonte 2026-09-06, riga 252](../user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).
- [ ] Preregistrare margini, condizioni held-out, repliche e budget prima della selezione finale. — [fonte 2026-09-06, riga 248](../user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).
- [ ] Fissare baseline, manifest e protocollo comune di parità; conservare gli artefatti storici. — [fonte 2026-09-06, riga 242](../user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).
- [ ] Completare il confronto imitazione/ex-novo con valutazioni periodiche secondo il benchmark: il pretraining V1 è concluso, ma smoke e pilot non esauriscono il confronto finale. — [fonte 2026-09-06, riga 250](../user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).
- [ ] Completare il valutatore comune con metriche per start/giunto, cicli, fallimenti e dati mancanti; i valutatori di singole campagne non attestano l’intero benchmark. — [fonte 2026-09-06, riga 244](../user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).
- [ ] Rendere il verdetto quantitativo di parità/regressione/inconclusivo e il costo per raggiungere la qualità, senza confondere fattibilità tecnica con qualità biomeccanica. — [fonte 2026-09-06, riga 254](../user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).
- [ ] Eseguire e attestare separatamente le verifiche native Windows della pipeline scelta. — [fonte 2026-09-06, riga 256](../user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).
- [ ] Completare la verifica dell’indipendenza degli aggiornamenti ex-novo dalle copie dei target. L’assenza di nuove copie nel suffisso privilegiato e la conservazione della maschera raccolta/PPO sono documentate; non equivalgono da sole al test specifico degli aggiornamenti. — [fonte 2026-09-14, riga 295](../user/2026-09-14_osservabilita_prescribed_mascheramento_e_test_fattibilita.md).
- [ ] Validare candidati completamente mascherati ai tre start, con avvio/tracking/cicli/protezioni distinti; il primo pilot del 1 ottobre non esaurisce questa validazione. — [fonte 2026-09-14, riga 297](../user/2026-09-14_osservabilita_prescribed_mascheramento_e_test_fattibilita.md).
- [ ] Costruire dati allineati e misurare osservabilità delle 35 feature su episodi indipendenti. — [fonte 2026-09-14, riga 292](../user/2026-09-14_osservabilita_prescribed_mascheramento_e_test_fattibilita.md).
- [ ] Preregistrare ed eseguire A/B/C di osservabilità, trasferimento B/C e repliche: non confondere questo protocollo con le successive ablazioni clock/scala/slew. — [fonte 2026-09-14, riga 97](../user/2026-09-14_avvio_piano_pretraining_50_100_fattibilita.md).
- [ ] Confrontare curriculum, avvio supervisionato e soluzioni causali di fase/memoria con evidenze; l’adozione del masking non risolve da sola l’osservabilità. — [fonte 2026-09-14, riga 293](../user/2026-09-14_osservabilita_prescribed_mascheramento_e_test_fattibilita.md).
- [ ] Verificare sincronizzazione senza clock prescribed, distinguendo avvio e regime ai tre start. — [fonte 2026-09-14, riga 291](../user/2026-09-14_osservabilita_prescribed_mascheramento_e_test_fattibilita.md).
- [ ] Verificare trasferimento, warm-up e resume della variante scelta e misurare il beneficio nel successivo ex-novo; correttezza tecnica e vantaggio di apprendimento sono distinti. — [fonte 2026-09-14, riga 298](../user/2026-09-14_osservabilita_prescribed_mascheramento_e_test_fattibilita.md).
- [ ] Completare il confronto V1 5/50/100 sui tre start. Il training fino a100 è superato dalla continuazione fino a300; il confronto specifico 5/50/100 non risulta documentato e resta aperto. — [fonte 2026-09-14, riga 95](../user/2026-09-14_avvio_piano_pretraining_50_100_fattibilita.md).
- [ ] Manutenzione separata delle assertion legacy e debiti RLlib, cycle_valid, swing timeout, probe schema, pin V20, best==last, fit_p2 V12R3, marker/audit e deflessione SEA iniziale; chiudere solo con prova specifica. — [fonte 2026-09-15, riga 161](../user/2026-09-15_pretraining_unificata_valutazione_intermedia_50.md).
- [ ] **W02:** sostituire il controllo `os.kill(pid, 0)` del percorso attach con una verifica di vitalità non distruttiva e validarla su Windows. — [fonte 2026-09-15, riga 353](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).
- [ ] **W03:** predisporre un percorso Windows per i runner J20 warm-up e restore R3, conservando la provenance storica e rendendo esplicito l'interprete della nuova esecuzione. — [fonte 2026-09-15, riga 354](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).
- [ ] **W04:** canonicalizzare le chiavi dei percorsi nell'attestazione J21 e verificare su Windows i gate T12/T18 con artefatti integri e casi negativi. — [fonte 2026-09-15, riga 355](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).
- [ ] **W05:** definire una politica dei fine-riga coerente con gli hash dei sorgenti unified e verificarla su un checkout Windows reale senza riscrivere i pin storici per nascondere differenze. — [fonte 2026-09-15, riga 356](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).
- [ ] **W06:** rendere il generatore differito 39 compatibile con LF e CRLF e verificarne la materializzazione rispettando la separazione dalla V1. — [fonte 2026-09-15, riga 357](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).
- [ ] **W07:** definire e verificare la ricollocazione di run, checkpoint e trace per il riuso delle valutazioni CP50/CP100 mantenendo le verifiche di identità del contenuto. — [fonte 2026-09-15, riga 358](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).
- [ ] **W08:** distinguere nell'analisi sigma un report mancante da un sigma realmente nullo e verificare la rianalisi degli artefatti trasferiti. — [fonte 2026-09-15, riga 359](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).
- [ ] Fissare un ambiente Windows riproducibile con versioni Python/Ray/Torch/NumPy/OpenSim e dipendenze native, compatibile con i checkpoint scelti. — [fonte 2026-09-15, riga 361](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).
- [ ] Documentare e verificare root di checkout, supporto long-path Windows/Git e percorsi generati dal workflow selezionato. — [fonte 2026-09-15, riga 360](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).
- [ ] Verificare su Windows orchestrazione, valutazioni/report e, quando prevista, materializzazione della variante 39; attestare soltanto i percorsi realmente provati. — [fonte 2026-09-15, riga 363](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).
- [ ] Eseguire una verifica nativa Windows di caricamento plugin, reset/step, training breve con worker Ray, rollout, salvataggio/restore e warm-up, registrando gli esiti separatamente dalle evidenze macOS. — [fonte 2026-09-15, riga 362](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).
- [ ] Validare oracle LEFT nel loop fisico sui tre start; il controllo successivo su un solo start non chiude la richiesta. — [fonte 2026-09-20, riga 199](../user/2026-09-20_risultati_ablazioni_unified39_clock_scala_slew.md).
- [ ] Valutazioni dedicate dei checkpoint nominali e stocastiche, con durata/tracking/contatto e un insieme held-out distinto dalla selezione. — [fonte 2026-09-20, riga 200](../user/2026-09-20_risultati_ablazioni_unified39_clock_scala_slew.md).
- [ ] Ampliare la valutazione best72 oltre cinque seed per condizione: l’evaluation iniziale è conclusa, l’estensione resta aperta. — [fonte 2026-09-22, riga 156](../user/2026-09-22_risultati_finali_unified39_100iter.md).
- [ ] Isolare con prove controllate gli effetti di traiettoria, risposta dinamica e distribuzione delle azioni; confrontare in rollout comando atteso dopo clipping e media clippata. — [fonte 2026-09-22, riga 157](../user/2026-09-22_risultati_finali_unified39_100iter.md).
- [ ] Estendere confronti fail/success alla stessa fase e prima della soglia nei fallimenti di training, oltre ai due casi appaiati della 100. — [fonte 2026-09-22, riga 158](../user/2026-09-22_risultati_finali_unified39_100iter.md).
- [ ] Esplicitare nei riepiloghi automatici clipping raw a monte di RLlib e clipping interno all’ambiente; la distinzione nel testo non attesta l’implementazione automatica. — [fonte 2026-09-22, riga 159](../user/2026-09-22_risultati_finali_unified39_100iter.md).
- [ ] Separare nel software reward, visibilità dei sei canali ed esplorazione, consentendo ex-novo visibile e curriculum. — [fonte 2026-09-23, riga 124](../user/2026-09-23_strategia_best72_confronto_v26_e_oscuramento.md).
- [ ] Preparare confronto causale RIGHT/LEFT da zero con FSM v3, ordine ingressi, inizializzazione, raccolta, Adam/resume e budget comuni; due storie separate non provano il solo effetto del target. — [fonte 2026-09-23, riga 123](../user/2026-09-23_strategia_best72_confronto_v26_e_oscuramento.md).
- [ ] Verificare separatamente reset dei diagnostici di escursione FSM e incidenza nelle configurazioni che li usano. — [fonte 2026-09-24, riga 140](../user/2026-09-24_fix_contatto_terminale_hard_unified39.md).
- [ ] Valutare best80/last82 e successivi candidati contro best72 con perturbazioni comuni e misure fisiche; il piccolo pilot reward non esaurisce questo confronto. — [fonte 2026-09-24, riga 102](../user/2026-09-24_avvio_left72_penetrazione_peso2_10iter.md).
- [ ] Valutare RIGHT last60 sugli stessi cinque seed prima di scegliere un checkpoint per masking. — [fonte 2026-09-24, riga 266](../user/2026-09-24_resoconto_39d_right_60iter_e_robustezza.md).
- [ ] Isolare origine del momento GRF invertito nell’ultimo appoggio prima di cambiare reference/governor/reward. — [fonte 2026-09-24, riga 270](../user/2026-09-24_resoconto_39d_right_60iter_e_robustezza.md).
- [ ] Approfondire transitorio seed125 su best35/40/58/60 e V26 con reference, velocità, contatto e penetrazione. — [fonte 2026-09-24, riga 268](../user/2026-09-24_resoconto_39d_right_60iter_e_robustezza.md).
- [ ] Sperimentare separatamente regolarità knee e cinematica, mantenendo tracking/contatto come vincoli. — [fonte 2026-09-25, riga 214](../user/2026-09-25_banco_tuning_rapido_reward_left.md).
- [ ] Verificare sufficienza del riadattamento critic con diagnostica degli errori ed evaluation indipendente; un update non dimostra convergenza. — [fonte 2026-09-29, riga 114](../user/2026-09-29_esiti_pilot_reward_left_e_diagnosi_kl.md).
- [ ] Isolare conservazione KL nel critic-only con un controllo sottoposto allo stesso trattamento, configurazioni e budget preregistrati; fix non implementato. — [fonte 2026-09-29, riga 113](../user/2026-09-29_esiti_pilot_reward_left_e_diagnosi_kl.md).
- [ ] Quantificare normalizzazione advantages sulle sole transizioni reali rispetto a righe artificiali. — [fonte 2026-09-29, riga 377](../user/2026-09-29_audit_RL_39d_bootstrap_e_aggiornamento_PPO.md).
- [ ] Integrare in esperimento separato il fix generale confini PPO, verificando connector completi/restore/riproducibilità: replay isolato non è patch produzione. — [fonte 2026-09-29, riga 374](../user/2026-09-29_audit_RL_39d_bootstrap_e_aggiornamento_PPO.md).
- [ ] Confrontare RL stock/corretto con stessa inizializzazione/reward/budget/evaluation, distinguendo continuazione best80 e apprendimento da zero, con repliche. — [fonte 2026-09-29, riga 379](../user/2026-09-29_audit_RL_39d_bootstrap_e_aggiornamento_PPO.md).
- [ ] Valutare altri start/tratti AB06, recupero dopo perturbazione e carichi/contatti; distinguere generalizzazione da replay nominale. — [fonte 2026-09-29, riga 386](../user/2026-09-29_audit_RL_39d_bootstrap_e_aggiornamento_PPO.md).

### Attività condizionate e vincoli

- [ ] **Vincolo:** Per ogni candidata, eseguire qualification scientifica distinta dall’integrità tecnica del training; questa regola non è un’attestazione di robustezza generale. — [fonte 2026-08-14, riga 215](../user/2026-08-14_h0_v12r9_training_readiness_preflight_monitor.md).
- [ ] **Condizionato:** Probe `train_ppo` (39/43): correggere solo con migrazione di schema dichiarata (lineage futura), non per i training comparabili con giugno. — [fonte 2026-08-18, riga 146](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md).
- [ ] **Condizionato:** Arbitrare la tensione fra interpolazione, quota on-policy e copertura piena prima di un nuovo round interpolato. — [fonte 2026-08-24, riga 214](../user/2026-08-24_v26b_audit_luglio_r1r2_e_progetto_rev4d.md).
- [ ] **Condizionato:** Riesaminare l'effetto della lunghezza del prefisso distinguendo interpolazione zero e ricette interpolate. — [fonte 2026-08-24, riga 180](../user/2026-08-24_v26b_rev4d_repeat_esperimento_decorrelato.md).
- [ ] **Condizionato:** Affrontare il margine di contatto del candidato storico Rev 4E, oltre 28 mm di 0,22 mm, se il ramo viene riaperto. — [fonte 2026-08-24, riga 203](../user/2026-08-24_v26b_rev4e_r2_replay.md).
- [ ] **Condizionato:** Conservare il limite interpretativo: il braccio A non converge al proprio budget e il suo NO-GO non identifica un limite del contratto. — [fonte 2026-08-25, riga 112](../user/2026-08-25_v26b_b1r2b_addendum_correttivo_screening.md).
- [ ] **Condizionato:** Riesaminare il peggioramento del fold 9 dopo il minimo a epoca 100 se il ramo B1 viene riaperto. — [fonte 2026-08-25, riga 154](../user/2026-08-25_v26b_b1r2b_esito_nogo_ipotesi_falsificata.md).
- [ ] **Condizionato:** Identificare la causa dell'errore delle teste rispetto a ciclo 1 senza concludere non-identificabilità da vicinanza. — [fonte 2026-08-25, riga 167](../user/2026-08-25_v26b_b1r2_diagnostica.md).
- [ ] **Condizionato:** Trattare l'errore localizzato nelle 308 righe di testa, distinto dalla vecchia assenza di copertura LOCO. — [fonte 2026-08-25, riga 185](../user/2026-08-25_v26b_b1r2_diagnostica_rev1.md).
- [ ] **Condizionato:** Nei futuri confronti non usare come generalizzazione il best val MSE luglio da split casuale su righe ripetute 32×. — [fonte 2026-08-25, riga 199](../user/2026-08-25_v26b_b1r2a_revisione_indipendente_e_proposta.md).
- [ ] **Condizionato:** Rivedere l'igiene della selection/patience nel ramo B1, senza confonderla con convergenza o causa del NO-GO. — [fonte 2026-08-25, riga 124](../user/2026-08-25_v26b_b1r1_loco_fit_esito_nogo.md).
- [ ] **Condizionato:** Riesaminare eseguibilità delle label swing ammesse per raggiungibilità,13,5% oltre il criterio 10%. — [fonte 2026-08-25, riga 139](../user/2026-08-25_v26b_a0a1_protocollo_contratto25d_preflight.md).
- [ ] **Condizionato:** Risolvere max_abs WAIT oltre 0,15 nel ramo B1, se riaperto. — [fonte 2026-08-25, riga 198](../user/2026-08-25_v26b_b1r2a_revisione_indipendente_e_proposta.md).
- [ ] **Condizionato:** Conservare LOTO3 fold, LOCO6 fold,11 fold B1R2, braccio budgetA e braccio lrB come rami futuri non operativi; integrarli solo con approvazione esplicita. — [fonte 2026-08-26, riga 184](../user/2026-08-26_v26c_j0_audit_preregistrazione_replica_luglio.md).
- [ ] **Condizionato:** Verificare retrospettivamente l’equivalenza semantica del teacher prescritto sotto v3 nell’audit della collection storica. — [fonte 2026-08-26, riga 178](../user/2026-08-26_v26c_j0_audit_preregistrazione_replica_luglio.md).
- [ ] **Condizionato:** Attestare il gate della collection teacher storica sotto v3 e corridoio corrente con la relativa evidenza. — [fonte 2026-08-26, riga 179](../user/2026-08-26_v26c_j0_audit_preregistrazione_replica_luglio.md).
- [ ] **Condizionato:** Conservare i limiti J14: label open-loop su stati student-visited, phase-invalid senza soglia e 183 righe pre-mismatch mantenute senza contributo informativo; chiudere la qualità solo con closed-loop. — [fonte 2026-08-27, riga 256](../user/2026-08-27_v26c_j14_post_mismatch_dagger_dataset_readiness.md).
- [ ] **Condizionato:** Eventuale integrazione/riapertura dei rami LOTO, LOCO, B1R1/B1R2 con protocollo e autorizzazione; conservarne i verdetti storici, senza considerarli già operativi. — [fonte 2026-08-27, riga 343](../user/2026-08-27_qualifica_closed_loop_j9r1.md).
- [ ] **Condizionato:** Usare i ceiling empirici J8 C/D dichiarati dopo ricalibrazione, distinguendoli dalla soglia ritirata; nessun rilassamento retroattivo del FAIL J18. — [fonte 2026-08-27, riga 265](../user/2026-08-27_v26c_j18_ricalibrazione_eligibilita_e_selezione.md).
- [ ] **Condizionato:** Conservare la priorità A>B>C>D e la rimozione di 4 righe su 4225 come parte esplicita del design/pesi, non una scelta variabile dopo il risultato. — [fonte 2026-08-27, riga 376](../user/2026-08-27_v26c_j18_prereg_update_b_only.md).
- [ ] **Condizionato:** Mantenere max-drift come tie-break diagnostico nella selezione preregistrata; un futuro cambio del ranking richiede una nuova decisione. — [fonte 2026-08-27, riga 267](../user/2026-08-27_v26c_j18_ricalibrazione_eligibilita_e_selezione.md).
- [ ] **Condizionato:** Documentare ENV_MUTATION_POLICY come superset conservativa in J19C: autorizzava variazioni start non esercitate dalla prova nominale. — [fonte 2026-08-27, riga 227](../user/2026-08-27_v26c_j19c_esecuzione_heldout_g_i.md).
- [ ] **Condizionato:** Conservare e spiegare le assertion readiness diventate false dopo emissione GO/esecuzione (E54/S47 e successive), senza trattarle come regressioni scientifiche o riscrivere i file pinnati. — [fonte 2026-08-27, riga 192](../user/2026-08-27_v26c_j19a_esecuzione_riproduzione_singola.md).
- [ ] **Condizionato:** Valutare in un futuro protocollo se lo shift nominale debba essere un gate anticipato: J11=0.0264 rispetto al riferimento July 0.005 era deliberatamente non binding. — [fonte 2026-08-27, riga 332](../user/2026-08-27_v26c_j11_multistart_fit_execution.md).
- [ ] **Condizionato:** Monitorare bande 20/25/28 mm e margini di F/H (minimo storico circa 2.316 mm), senza trasformare diagnostici soft in nuovi hard gate a posteriori. — [fonte 2026-08-27, riga 247](../user/2026-08-27_v26c_j19b_esecuzione_qualificazione_closed_loop.md).
- [ ] **Condizionato:** Conservare la fragilità storica seed 125/F e monitorare i margini nelle qualifiche; il dato precedente non certifica nuove policy. — [fonte 2026-08-27, riga 431](../user/2026-08-27_v26c_j13_j12_failure_diagnosis_and_recovery_plan.md).
- [ ] **Condizionato:** Una qualifica non autorizza implicitamente warm-up/PPO: ogni nuovo stadio richiede la sua successiva autorizzazione pertinente. — [fonte 2026-08-27, riga 212](../user/2026-08-27_v26c_j19c_esecuzione_heldout_g_i.md).
- [ ] **Condizionato:** Ogni nuova fase deve avere report dedicato/audit e autorizzazione pertinente prima della successiva; i GO monouso storici non vengono riutilizzati. — [fonte 2026-08-27, riga 205](../user/2026-08-27_v26c_j15_fresh_refit_execution.md).
- [ ] **Vincolo:** Conservare immutati sidecar J8 e artefatti storici. — [fonte 2026-08-27, riga 356](../user/2026-08-27_qualifica_closed_loop_j9r1.md).
- [ ] **Condizionato:** Se si riusa questo design, giustificare repeat 8 anziché ereditarlo dalla metodologia July; le nuove righe J14 usano repeat 1, ogni diversa ponderazione richiede preregistrazione. — [fonte 2026-08-27, riga 184](../user/2026-08-27_v26c_j10_multistart_teacher_readiness.md).
- [ ] **Condizionato:** La forza vincolante del protocollo 23 agosto non autorizzato richiede decisione esplicita prima di usarlo in un nuovo esperimento. — [fonte 2026-08-27, riga 435](../user/2026-08-27_v26c_j13_j12_failure_diagnosis_and_recovery_plan.md).
- [ ] **Condizionato:** Sorvegliare la copertura WAIT_HS: 20 righe uniche solo B, 160 dopo tiling; non inventare una soglia di sufficienza o confondere dato ripetuto con indipendente. — [fonte 2026-08-27, riga 241](../user/2026-08-27_v26c_j10r1_multistart_teacher_collection.md).
- [ ] **Condizionato:** Conservare e spiegare i check R1 B06/I07 e R2 C29/E01/H02/H04 diventati falsi per destinazione presente dopo esecuzione, senza riscrivere gli artefatti pinnati. — [fonte 2026-08-28, riga 276](../user/2026-08-28_v26c_j20_restore_audit_r2_esecuzione.md).
- [ ] **Condizionato:** Per ogni nuovo pilot fissare config/comando/budget/stop/monitor/rollback e ottenere GO nuovo; la proposta storica 5–10 sigma 0.010 non viene considerata eseguita solo perché esistono run successivi con altro budget. — [fonte 2026-08-28, riga 1059](../user/2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md).
- [ ] **Condizionato:** Per una futura ricostruzione applicare l’intera checklist: parent agosto verificato, input July esclusi, actor 35/critic 84 e mapping per nome, sigma misurata, dataset tracciati, preservation funzionale, qualifica 6/6+3/3, FSM/penetrazione sane, actor congelato nel warm-up, critic/Adam/restore esatti e attestazione separata senza autorizzazione implicita al training. — [fonte 2026-08-28, riga 1029](../user/2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md).
- [ ] **Condizionato:** Rivalutare sigma dopo actor predittivo o modifica causale del morphology corridor. — [fonte 2026-08-28, riga 1055](../user/2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md).
- [ ] **Condizionato:** Mantenere la served contro corridor retrospettivo HS→TO→HS come metrica finale event-aligned, senza alterare reward passata o spostare il gate per accettare ritardi. — [fonte 2026-08-30, riga 358](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).
- [ ] **Condizionato:** Se si interviene sul governor, confrontare cutoff/slew ankle in un’ablation controllata a policy congelata; la misura dei lag correnti non prova il risultato di tale modifica. — [fonte 2026-08-30, riga 374](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).
- [ ] **Condizionato:** Se si riprende questa strategia, implementare teacher inverse-governor e label raw da stato effettivamente visitato; validare il beneficio closed-loop. Pista distinta dal RL senza teacher. — [fonte 2026-08-30, riga 392](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).
- [ ] **Condizionato:** Preregistrare gate quantitativi per toe-off, saturazione, FSM, tracking e condizioni held-out/multi-start prima di selezionare l’actor predittivo. — [fonte 2026-08-30, riga 394](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).
- [ ] **Condizionato:** Preregistrare pilot PPO separato 5–10 update, sigma 0,010 isotropico e logstd congelato, conservando J20/canonico; non sostituirlo con pilot di altra ricetta. — [fonte 2026-08-30, riga 388](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).
- [ ] **Condizionato:** Ripetere il critic-only rewarm quando la reward diventa realmente asimmetrica. — [fonte 2026-08-31, riga 346](../user/2026-08-31_separazione_morphology_knee_ankle_e_critic_rewarm.md).
- [ ] **Condizionato:** Ogni nuovo run usa config/namespace separati senza alterare J20, canonico lungo o GO consumati; è un invariante, non una creazione da rifare ora. — [fonte 2026-08-31, riga 415](../user/2026-08-31_smoke_test_sigma_esplorazione_v26c_j20.md).
- [ ] **Condizionato:** Decidere eventuale estensione oltre 10 soltanto dopo rollout deterministici e gate del pilot J20. — [fonte 2026-08-31, riga 419](../user/2026-08-31_smoke_test_sigma_esplorazione_v26c_j20.md).
- [ ] **Condizionato:** Nel pilot preregistrato verificare dopo ogni iterazione penetrazione, FSM, loss, clipping/saturazione e deriva actor; valutare deterministicamente i checkpoint prodotti. — [fonte 2026-08-31, riga 417](../user/2026-08-31_smoke_test_sigma_esplorazione_v26c_j20.md).
- [ ] **Condizionato:** Eseguire DAgger closed-loop sullo stesso actor 35D con nominali e recovery, se si riprende questa strategia. — [fonte 2026-08-31, riga 610](../user/2026-08-31_progetto_rete_unificata_imitativa_exnovo_actor_predittivo.md).
- [ ] **Condizionato:** Riprendere actor predittivo e compensazione del ritardo soltanto in una versione successiva; attività differita. — [fonte 2026-09-06, riga 257](../user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).
- [ ] **Vincolo:** Monitorare penetrazione 20/25/28 mm, served-vs-actual e FSM v3 in ogni nuovo candidato. — [fonte 2026-09-15, riga 152](../user/2026-09-15_pretraining_unificata_valutazione_intermedia_50.md).
- [ ] **Vincolo:** Preservare gli artefatti storici e distinguere portabilità dei pesi, migrazione dei risultati e riproduzione dell’esperimento. — [fonte 2026-09-15, riga 364](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).
- [ ] **Condizionato:** Eventuali ulteriori prove da B devono isolare un fattore per volta. I successivi rami allineati modificano più impostazioni insieme; non attestano la chiusura delle ablazioni proposte né autorizzano il loro avvio. — [fonte 2026-09-20, riga 201](../user/2026-09-20_risultati_ablazioni_unified39_clock_scala_slew.md).
- [ ] **Condizionato:** Valutare il significato della guardia knee a 0°, distinguendo sicurezza, durata e fedeltà; eventuale modifica richiede prova separata. — [fonte 2026-09-20, riga 202](../user/2026-09-20_risultati_ablazioni_unified39_clock_scala_slew.md).
- [ ] **Vincolo:** Separare masking dalla costruzione di una baseline affidabile e fissare seed/criteri prima di rivendicare parità. — [fonte 2026-09-20, riga 203](../user/2026-09-20_risultati_ablazioni_unified39_clock_scala_slew.md).
- [ ] **Condizionato:** Eventuale screening appaiato dei checkpoint LEFT25/29/40 e geometria di contatto nei tratti problematici; i soli rollout CP40 non completano tutte le celle. — [fonte 2026-09-21, riga 151](../user/2026-09-21_risultati_unified39_v26_aligned_40iter.md).
- [ ] **Vincolo:** Valutare masking come passaggio distinto senza presumere che ex-novo risolva automaticamente la robustezza. — [fonte 2026-09-23, riga 278](../user/2026-09-23_v26_ordine_aritmetico_e_robustezza.md).
- [ ] **Vincolo:** Distinguere RIGHT da zero, adattamento LEFT, trasferimento V26 e masking/ex-novo; non attribuire risultati di un braccio agli altri. — [fonte 2026-09-23, riga 125](../user/2026-09-23_avvio_training_39d_riferimento_destro.md).
- [ ] **Condizionato:** Se prioritario, confrontare V26 e best72 con rumore assoluto comune, distinguendo target e policy media: sigma proprie con gaussiani comuni non isolano la sola esplorazione. — [fonte 2026-09-23, riga 273](../user/2026-09-23_v26_ordine_aritmetico_e_robustezza.md).
- [ ] **Vincolo:** Fissare criteri distinti per completamento, cicli, forma e coppie prima della selezione e del masking; non usare il solo return. — [fonte 2026-09-24, riga 272](../user/2026-09-24_resoconto_39d_right_60iter_e_robustezza.md).
- [ ] **Vincolo:** Distinguere beneficio della continuazione da spiegazione del divario nell’apprendimento da zero, che richiede un controllo dedicato. — [fonte 2026-09-24, riga 309](../user/2026-09-24_audit_reward_left_fasi_contatto_e_clipping.md).
- [ ] **Condizionato:** Se si procede, eseguire il controllo LEFT72 peso 1 contro peso 2, con terminale hard corretto, stesso resume, budget e criteri fisici; il solo ramo peso 2 non isola causalità. — [fonte 2026-09-24, riga 307](../user/2026-09-24_audit_reward_left_fasi_contatto_e_clipping.md).
- [ ] **Vincolo:** Eventuali modifiche reward per le coppie devono usare stesso checkpoint/budget, tracking e contatto come vincoli e confronti separati knee/ankle. — [fonte 2026-09-24, riga 121](../user/2026-09-24_best80_cause_coppie_ankle_knee.md).
- [ ] **Vincolo:** Separare regolarità della coppia da forma legata a reference/carico, giudicando anche tracking e contatto senza copiare V26 o introdurre regole AB06. — [fonte 2026-09-29, riga 170](../user/2026-09-29_indagine_generale_39d_imitazione_e_forma_coppie.md).

### Chiusure documentate e consolidamento retrospettivo

- [x] Daily 2026-08-16 e 2026-08-17 da consolidare al prossimo `end_day`. — [fonte 2026-08-17, riga 121](../user/2026-08-17_diagnosi_divergenza_luglio_v26_e_proposta_retraining_imitativo_nativo.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-17_daily-report.md)).
- [x] Daily 2026-08-16 da consolidare al prossimo `end_day` (14 user report della giornata). — [fonte 2026-08-17, riga 95](../user/2026-08-17_h0_v12r17_verdetto_e_probe_capacita.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-17_daily-report.md)).
- [x] Daily 2026-08-16/17/18 da consolidare al prossimo `end_day`. — [fonte 2026-08-18, riga 154](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-18_daily-report.md)).
- [x] Daily 2026-08-18/19 da consolidare al prossimo `end_day`. — [fonte 2026-08-19, riga 159](../user/2026-08-19_indagine_iterazioni_lente_crash_storm_reject_continue_fix.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-19_daily-report.md)).
- [x] Daily 2026-08-18/19/20 da consolidare al prossimo `end_day`. — [fonte 2026-08-20, riga 104](../user/2026-08-20_audit_equivalenza_giugno_per_fresh_run_nativo_v26.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-20_daily-report.md)).
- [x] Decidere ankle10/ankle5 dall’esito e documentare risultato: tre update non assicurano una candidata risolutiva. — [fonte 2026-09-25, riga 212](../user/2026-09-25_banco_tuning_rapido_reward_left.md). **Chiuso:** Best80 resta il riferimento; ankle10 e ankle5 sono respinti dai vincoli fisici, nessun checkpoint intermedio supera lo screening. ([evidenza](../user/2026-09-29_esiti_pilot_reward_left_e_diagnosi_kl.md)).
- [x] Completare nominali/rumore, plot e confronto fisico del pilot senza estendere automaticamente il budget. — [fonte 2026-09-25, riga 210](../user/2026-09-25_banco_tuning_rapido_reward_left.md). **Chiuso:** Conclusi pilot e sei valutazioni intermedie; ultimi rollout il 28. ([evidenza](../user/2026-09-29_esiti_pilot_reward_left_e_diagnosi_kl.md)).
- [x] Avviare pilot, verificare restore/reward worker/rewarm/actor/Adam e tre update PPO per ramo. — [fonte 2026-09-25, riga 206](../user/2026-09-25_banco_tuning_rapido_reward_left.md). **Chiuso:** Entrambi i pilot conclusi, analisi learner e valutazioni documentate. ([evidenza](../user/2026-09-29_esiti_pilot_reward_left_e_diagnosi_kl.md)).

<!-- reconciled-report-todos-2026-10-05:end -->
