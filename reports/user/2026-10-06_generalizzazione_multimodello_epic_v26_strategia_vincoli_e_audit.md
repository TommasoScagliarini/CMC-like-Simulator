# Generalizzazione multi-modello EPIC della catena V26: decisioni, audit e protocollo da concretizzare

**Data di completamento:** 6 ottobre 2026, Europe/Rome.  
**Discussione e primo audit:** 5 ottobre 2026; stesura ripresa dopo interruzione.  
**Tipo:** report utente richiesto con `create_report`; documento di contesto e handover.  
**Stato:** analisi e scelte documentate; preparazione dei nuovi modelli, download, rollout e training in attesa del via dell'utente.  
**Instruction check token:** `CMC_AGENT_OK_2026`.

## 1. Punto di ripartenza

L'utente considera insufficienti le traiettorie cinematiche ottenute con l'oscuramento degli ingressi prescribed della rete `unified_MLP_39`. La scelta è **mettere temporaneamente da parte quel ramo e tornare alla catena V26 imitativa + adattamento a student deployable + training ex-novo**. Gli artefatti della 39D restano riferimenti storici: questa scelta non cancella checkpoint o risultati e non dichiara il metodo impossibile.

La nuova fase riguarda la generalizzazione del trajectory generator a modelli diversi da quello EPIC AB06. Il percorso immaginato dall'utente comprende inventario/download dei soggetti EPIC treadmill, preparazione mediante la pipeline usata per AB06 e adattamento della rete su più modelli. La parte di training è stata discussa in dettaglio, ma non esiste ancora una campagna multi-modello eseguibile e qualificata.

L'indicazione operativa esplicita è: **«aspetta il mio via prima di partire»**. La richiesta successiva `create_report` autorizza la scrittura di questo documento e la propagazione dei TODO nel daily; non autorizza le esecuzioni descritte. Durante questa attività non sono stati lanciati download, build dei nuovi soggetti, conversioni, rollout di generalizzazione o aggiornamenti della policy.

### 1.1 Handover minimo da conservare

1. La nuova priorità è V26 multi-modello; il ramo 39D con oscuramento è sospeso temporaneamente.
2. Si parte dai pesi imitativi V26 già allenati e si propone una continuazione del training su più ambienti/soggetti, conservando AB06 nel campionamento.
3. La policy finale deve utilizzare soltanto informazioni disponibili dalla protesi, senza descrittori anatomici del paziente o identificativi del soggetto.
4. Il passaggio al controllo deployable deve essere qualificato su tutti i domini di training prima del PPO ex-novo.
5. Nella prima istanza si conserva il morphology corridor attuale.
6. Nella prima fase non si aggiunge domain randomization sintetica.
7. Il pilot di preparazione sui tre soggetti locali AB06/AB07/AB08 è condiviso come direzione; attende il via.
8. Un rollout a pesi congelati su un nuovo modello, prima del nuovo training, è un primo test utile proposto; non è stato eseguito.
9. Split 70/30 oppure 80/20 per soggetto sono entrambe opzioni discusse; nessuna percentuale o assegnazione definitiva è stata scelta.
10. L'audit degli ingressi deve comprendere il preprocessing: il carico normalizzato per peso corporeo oggi dipende dalla massa OpenSim.

## 2. Problema e motivazione del cambio di direzione

Finora il trajectory generator operativo è stato sviluppato, allenato e qualificato principalmente su AB06. Funzionare su più trial, start o perturbazioni dello stesso soggetto amplia il bacino di condizioni, ma non dimostra trasferimento a un'anatomia nuova. Variazioni di massa, inerzie, lunghezze, geometria del piede, cadenza e comportamento del contatto modificano l'ambiente in cui la policy genera i target protesici.

Nel ramo 39D recente, il completamento temporale e il buon inseguimento SEA della reference servita non hanno garantito una forma cinematica sufficiente a ingressi oscurati. La decisione di sospensione deriva da questa insufficienza, non da un problema di caricamento dei checkpoint.

### 2.1 Evidenze pregresse da distinguere dalle prove future

Il [daily del 1° ottobre, integrato il 5 ottobre](../daily/2026-10-01_daily-report.md) documenta due esperimenti distinti:

- La continuazione CP90→190 con quattro blocchi di 25 aggiornamenti e oscuramento episodico 25/50/75/100%. I rollout CP112/CP184 completano la finestra nominale, ma persistono scostamenti di forma. Il solo completamento non qualifica il cammino autonomo.
- Il confronto separato imitazione/ex-novo da CP90, con un warm-up critic-only e dieci aggiornamenti actor per ramo. Il curriculum si arresta al primo livello: 75% episodi visibili, 25% oscurati. Nessun livello successivo è stato eseguito.

Nel confronto specifico CP90, su finestra comune 13,19–17,94 s:

| Policy | Escursione knee / ankle [°] | RMSE actual–served knee / ankle [°] | Cicli validi |
|---|---:|---:|---:|
| CP90 visibile | 41,71 / 30,67 | 0,18 / 0,26 | 2 |
| CP90 oscurato prima degli update actor | 12,02 / 10,26 | 0,27 / 0,29 | 1 |
| Imitazione dopo 10 update, oscurata | 16,36 / 10,64 | 0,27 / 0,27 | 1 |
| Ex-novo dopo 10 update, oscurata | 12,02 / 11,30 | 0,26 / 0,29 | 1 |

Entrambi i rami oscurati falliscono il gate di escursione e cicli. La diagnosi localizza il limite soprattutto nella traiettoria generata, con critic e attribuzione morphology ancora da approfondire. Questi numeri sono risultati storici di una ricetta, un seed e uno start: non sono stati rieseguiti per questo report e non costituiscono risultati della generalizzazione V26.

Fonti: [analisi del confronto](<../../Trajectory Generator/unified_MLP_39/validation/reward_curriculum_2026_10_01/README.md>) e [ricevuta finale](<../../Trajectory Generator/unified_MLP_39/validation/reward_curriculum_2026_10_01/verification.json>).

Best90 fresh resta la baseline storica scelta per quel ramo. La riapertura del workstream V26 non seleziona automaticamente un nuovo checkpoint V26 corrente, né promuove J19A/J20 o il pilot ex-novo come baseline definitiva del nuovo esperimento.

## 3. Registro delle scelte e del loro stato

La tabella separa ciò che l'utente ha deciso dalle proposte dell'assistente e dalle questioni ancora aperte. Un orientamento condiviso non equivale ad autorizzazione al lancio.

| ID | Tema | Stato al 5 ottobre | Conseguenza operativa |
|---|---|---|---|
| D01 | Rete 39D con oscuramento | Sospensione temporanea richiesta dall'utente | Non proseguire automaticamente quel curriculum; conservare gli artefatti |
| D02 | Famiglia di riferimento | Ritorno alla V26 imitativa + ex-novo | Riprendere la catena storica e identificarne i checkpoint precisi |
| D03 | Corpus | Obiettivo: soggetti EPIC con treadmill | Inventario e qualificazione ancora necessari; nessun download massivo avviato |
| D04 | Prima preparazione | Utente concorda sui tre soggetti locali | AB06/AB07/AB08 sono il pilot tecnico previsto, in attesa del via |
| D05 | Esecuzioni | Attendere il via esplicito | Questa attività è documentale; nessun lancio implicito |
| D06 | Split | 70/30 oppure 80/20 discussi dall'utente | Percentuale, soggetti e validazione da fissare; split per soggetto |
| D07 | Descrittori anatomici nell'actor | Esclusi dal percorso iniziale dopo il chiarimento dell'utente | Nessuna massa, lunghezza o inerzia del paziente fornita esplicitamente alla policy finale |
| D08 | Identificativo del soggetto nell'actor | Non previsto nella proposta | Gli ID servono a manifest e metriche; non diventano una scorciatoia della policy |
| D09 | Morphology corridor | Conservare quello attuale in prima istanza, scelta dell'utente | Verificarne copertura e applicazione; adattamento successivo distinto |
| D10 | Domain randomization | Rinviata; utente concorda | Prima fase sui modelli EPIC preparati, senza perturbazioni sintetiche aggiuntive |
| D11 | Zero-shot prima del training | Proposto dall'utente e ritenuto informativo | Predisporre il test dopo la qualificazione del modello; non ancora eseguito |
| D12 | Estensione imitativa | Strategia discussa: continuare la V26 esistente su più soggetti | Un solo insieme di pesi, dati/episodi multi-modello; budget e resume non definiti |
| D13 | Student deployable | Strategia discussa: estendere il metodo V26C/DAgger | Una policy finale comune, qualifica su tutti i domini; implementazione ancora necessaria |
| D14 | Ex-novo | Strategia discussa: stesso insieme di domini della fase imitativa/student | Evitare che il successivo PPO torni soltanto su AB06 |
| D15 | Storia di osservazioni/comandi | Possibile estensione futura, non scelta iniziale | Nessuna memoria, encoder o nuovo input implementato o autorizzato |
| D16 | Plugin e controllori SEA | Nessuna nuova modifica richiesta | Preservare architettura high-level Python / low-level C++ e semantica del comando |

## 4. Terminologia: che cosa verrà misurato

### 4.1 Soggetto, modello, trial e condizione

- **Soggetto:** individuo del dataset, per esempio AB06; costituisce l'unità principale dello split anatomico.
- **Modello:** rappresentazione OpenSim personalizzata del soggetto, poi trasformata nella configurazione protesica richiesta. Lo stesso soggetto non diventa un'anatomia nuova cambiando trial.
- **Trial:** registrazione di marker, cinematica, forze e condizioni.
- **Condizione:** velocità, intervallo, cadenza o fase selezionata all'interno di un trial. Un file può contenere più condizioni.
- **Dominio sperimentale:** combinazione esplicita di modello, trial/condizione, configurazione protesica, contatto, reset e convenzioni di osservazione/azione.

Nel report, «stessi domini» significa mantenere la distribuzione di soggetti e condizioni pertinente attraverso imitazione, student e PPO ex-novo. Non significa che reward, critic o disponibilità dei target privilegiati debbano essere identici tra fasi.

### 4.2 Portabilità, multi-modello, generalizzazione e adattamento

| Termine | Significato nel progetto | Evidenza necessaria |
|---|---|---|
| Portabilità software | Il codice carica un bundle diverso con schema compatibile | Setup, nomi, dimensioni, plugin e reset validi |
| Capacità multi-modello | Una policy comune funziona sui soggetti usati nel training | Valutazioni separate per soggetto e condizione |
| Trasferimento zero-shot | Pesi e inferenza congelati, nessun update sul nuovo soggetto | Rollout sul modello qualificato con protocollo e preprocessing dichiarati |
| Domain generalization | Funzionamento su soggetti esclusi dallo sviluppo della policy | Test per soggetto separato da teacher, DAgger, PPO e selezione |
| Domain adaptation / few-shot | La policy viene aggiornata usando il nuovo soggetto | Budget di adattamento e valutazione successiva separati dal zero-shot |

La letteratura definisce domain generalization come apprendimento dai domini sorgente con valutazione su domini non visti. Nel nostro caso, escludere soltanto alcuni stride di AB06 non realizza questa separazione anatomica. [Fonte primaria sulla definizione](https://arxiv.org/abs/2007.03304).

Il primo obiettivo è la generalizzazione fra soggetti della famiglia EPIC nella medesima struttura protesica e nel plant CMC-like. Modelli con topologie articolari differenti, protesi differenti, altri task o impiego hardware ampliano il perimetro e richiedono qualifiche proprie. Non si promette funzionamento su qualsiasi modello OpenSim.

## 5. Dataset EPIC: fonte e disponibilità

### 5.1 Corpus di riferimento

La [pagina ufficiale EPIC per Camargo et al. 2021](https://www.epic.gatech.edu/opensource-biomechanics-camargo-et-al/) descrive 22 soggetti sani e treadmill a 28 velocità, 0,50–1,85 m/s con incremento 0,05 m/s e intervalli di 30 s. Sono pubblicati dati cinematici/cinetici, marker e force plates, oltre a sensori wearable. La pagina permette accesso parziale ai dati.

Il deposito autore è diviso in tre parti Mendeley, versione 2: [parte 1](https://data.mendeley.com/datasets/fcgm3chfff/2), [parte 2](https://data.mendeley.com/datasets/k9kvm5tn3f/2), [parte 3](https://data.mendeley.com/datasets/jj3r5f9pnf/2). Le pagine riportano licenza CC BY 4.0. Questi collegamenti identificano la provenienza, non attestano un download effettuato oggi.

L'inventario remoto completo dei modelli individuali non è stato certificato file per file. La numerosità di 22 soggetti non garantisce automaticamente che tutti abbiano ogni componente o ogni velocità utilizzabile dal simulatore. Numero di soggetti eleggibili e copertura delle condizioni dovranno derivare dal manifest reale.

I template [OSL EPIC](https://www.epic.gatech.edu/oslmodels/) costituiscono materiale distinto dal corpus sano Camargo. Non vanno confusi con le anatomie dei soggetti o con la nostra trasformazione SEA.

### 5.2 Inventario locale verificato in questa sessione

| Soggetto | Cartella | Sessione individuata | Trial treadmill con componenti associati |
|---|---|---|---:|
| AB06 | `models/AB06-raw/` | `10_09_18` | 8 |
| AB07 | `models/AB07-raw/` | `10_14_18` | 9 |
| AB08 | `models/AB08-raw/` | `10_21_2018` | 7 |

L'audit dei nomi/stem trova modello sano e componenti `markers`, `fp`, `ik`, `id`, `conditions`, `gcLeft`, `gcRight` per questi 24 trial. Non è un controllo di integrità numerica del contenuto MATLAB: i file non sono stati tutti aperti né confrontati campione per campione.

Il bundle operativo presente è [AB06_SEASEA_Threadmill](../../models/AB06_SEASEA_Threadmill/). La disponibilità raw di AB07/AB08 non equivale alla presenza di due bundle protesici completi già qualificati.

### 5.3 Manifest da costruire

Per ciascun soggetto/trial il manifest futuro deve associare almeno:

- fonte e versione del dataset; identificativo soggetto/sessione/trial;
- modello sano, trasformazione protesica e modello risultante, con hash;
- statico, marker, IK, GRF/ExternalLoads, eventi e condizioni;
- convenzioni di coordinate, assi, segni, unità e lato protesico;
- massa/segmenti/inerzie per la fisica e l'audit, anche se esclusi dall'actor;
- donor e configurazione SEA/controller; profili GRF e detector;
- finestre temporali valide, velocità e start definiti rispetto agli eventi;
- destinazioni dei bundle per trial; stato dei gate di qualità;
- appartenenza a training, validazione o test e ragioni delle eventuali esclusioni.

Questo è un elenco di requisiti, non uno schema già implementato. Un identificativo nel manifest non autorizza a inserirlo fra le osservazioni della policy.

## 6. Pipeline AB06/ABx: disponibilità e limiti reali

### 6.1 Documentazione reperita

La pipeline è documentata nei seguenti report:

- [AB06 SEA/EPIC, 11 maggio](2026-05-11_ab06_seasea_epic_pipeline.md);
- [daily del 12 maggio](../daily/2026-05-12_daily-report.md);
- [pipeline unica ABx, 13 maggio](2026-05-13_pipeline_unica_abx_seasea.md).

L'entrypoint generale è [tools/build_abx_seasea_pipeline.py](../../tools/build_abx_seasea_pipeline.py). Usa utility di conversione, calibrazione marker e [scripts/run_opensim_sea_pipeline.py](../../scripts/run_opensim_sea_pipeline.py).

La catena presente nel codice comprende: scoperta del modello sano e dei dati → innesto della catena protesica sinistra → preservazione della massa secondo la convenzione storica → conversione static/treadmill → calibrazione marker sullo statico → preparazione cinematica, GRF e ExternalLoads → bundle CMC-like → overlay healthy; RRA e smoke sono opzionali.

### 6.2 Che cosa significa donor

Il donor è il modello sorgente dal quale vengono copiati segmenti, giunti e componenti di forza della protesi. Il modello sano del soggetto riceve tale catena; il donor non è il soggetto di test e non è una rete neurale.

La selezione automatica attuale cerca, nell'ordine, i candidati elencati in [default_donor_model](../../tools/build_abx_seasea_pipeline.py): il primo è `models/AB06_SEASEA-raw/osimxml/AB06_SEASEA.osim`, presente sul disco. Questo default non garantisce la configurazione PI-tuned usata dalla baseline V26.

Il graft copia anche componenti del ForceSet. Prima di una nuova build va quindi fissato esplicitamente un donor coerente con il checkpoint e con i parametri fisici che si vogliono conservare. Non è stato cambiato alcun donor né parametro in questa attività.

### 6.3 Geometria e massa non sono la stessa operazione

Il [graft dei corpi](../../tools/build_abx_seasea_pipeline.py) copia i segmenti protesici e modifica massa/inerzie; il graft dei giunti conserva l'anca del soggetto ma copia i giunti protesici del donor. Non adatta automaticamente lunghezze, COM e posizioni relative alle dimensioni del nuovo soggetto.

L'audit degli XML sani mostra offset di caviglia dalla tibia diversi: circa 0,4655 m AB06, 0,4177 m AB07 e 0,4479 m AB08. Sono misure locali degli XML, non una nuova stima antropometrica o una validazione della geometria innestata.

La preservazione della massa è una convenzione del benchmark: `transfemur` riceve la massa originale di `femur_l`; la massa restante della catena rimossa viene ripartita fra i tre segmenti distali secondo proporzioni del donor. Non identifica la massa reale di una protesi hardware. La scelta fra conservare questa convenzione o adottare parametri fisici diversi deve restare esplicita e comune ai confronti; non è stata modificata oggi.

Il fitting futuro deve chiarire quali componenti rappresentino hardware di dimensione fissa e quali debbano essere regolati per il soggetto. Non si deve assumere che scalare indiscriminatamente il piede, il pylon o le inerzie sia il fitting corretto.

### 6.4 Scaling biologico

Il runner generale salta automaticamente lo scaling quando il MarkerSet è il modello stesso o un file `.osim`, come nel richiamo corrente del builder. Usare un modello sano EPIC già personalizzato può rendere corretto evitare un secondo scaling biologico; non risolve il fitting protesico.

Occorre verificare personalizzazione e provenienza di ogni modello. «Eseguire la pipeline AB06» non significa necessariamente ripetere uno scaling completo già svolto dal dataset.

### 6.5 IK: preparazione dei file e calcolo effettivo sono distinti

`run_ik_pipeline()` passa `--dry-run` al runner OpenSim. Il runner non esegue gli strumenti; su una build nuova l'IK attesa non viene prodotta e il builder ripiega sulla IK EPIC convertita/rimappata.

Per una nuova campagna dobbiamo decidere se quella IK sia sufficiente per il modello innestato o se debba essere ricalcolata, e qualificare la decisione con errori marker e fattibilità dinamica. La semplice rimozione di `--dry-run` non è una correzione già approvata: il runner generale può proseguire anche con ID/RRA, quindi serve un percorso di esecuzione preciso.

Non è stata modificata questa logica. Il report non presenta il PASS discovery come una nuova esecuzione IK.

### 6.6 Un trial per task e rischio di sovrascrittura

`--all-subjects` significa tutti i soggetti locali trovati dal builder. Non significa scaricare tutti EPIC o processare automaticamente tutti i trial. La build attuale seleziona un trial per soggetto/task e le destinazioni non costituiscono ancora un registro multi-trial indipendente.

L'estensione deve assegnare output distinti a soggetto/sessione/trial/condizione e registrare il modello di ciascun bundle, evitando sostituzioni silenziose.

### 6.7 RRA e artefatti storici

Nel lavoro AB06, RRA full-span non fu adottato come modello operativo perché peggiorava i momenti residui. Va mantenuto come diagnostica con criteri propri, non come passaggio obbligatorio che garantisca un modello migliore.

Alcune directory di risultati OpenSim citate nei report di maggio oggi non sono presenti. I risultati smoke/RRA relativi a quelle directory sono evidenze storiche nei report, non artefatti numerici rianalizzati oggi. Sono presenti i raw/calibrated donor e il bundle AB06 Threadmill.

### 6.8 Gate da prevedere per ogni modello

Prima di un rollout della policy occorre verificare: caricamento/plugin, coordinate e range, marker e fitting, geometria e posa iniziale del piede, sincronismo modello–IK–GRF, segni/unità del free torque, massa/inerzie, stato SEA iniziale, detector, reset, completamento prescribed e richiesta di reserve biologiche.

Le soglie esatte dei gate non sono ancora state fissate per il corpus. Un modello che fallisce la fattibilità prescribed deve essere classificato e investigato prima di attribuire il fallimento alla generalizzazione della rete.

## 7. V26: quali reti e checkpoint vengono distinti

### 7.1 L'ambiguità del numero 39

Il parent imitativo V26 storico ha un actor con 39 ingressi, comprendenti quattro target healthy privilegiati. Questo non è il ramo sperimentale `unified_MLP_39` con curriculum di oscuramento che l'utente ha sospeso.

Il riferimento deployable V26C deriva invece un actor 35D, con clock prescribed neutralizzato e informazioni di stato del controllore. La proposta «ritornare alla V26» riguarda questa catena e non impone di usare direttamente il parent privilegiato in deployment.

### 7.2 Artefatti storici disponibili e selezione ancora aperta

L'audit trova gli artefatti del parent imitativo V26 agosto, degli export J19A, del critic/full checkpoint J20, del pilot ex-novo V26C e del critic rewarm di fine agosto. La loro esistenza non fissa quale checkpoint debba essere congelato per il nuovo zero-shot o usato per la continuazione imitativa.

La [guida del 28 agosto](2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md) raccoglie lineage, input e audit. Il parent imitativo best87 è un riferimento storico identificato; J19A/J20 sono riferimenti della costruzione deployable. Prima di eseguire vanno registrati path effettivo, hash, configurazione, feature, scaler, action bounds e versione del codice.

La qualifica closed-loop di J19A è documentata nelle fasi successive J19B/J19C; non va riscritta retroattivamente come stato originario del sidecar J19A. Analogamente `TRAINING_READY` certifica un input tecnico, non il successo della generalizzazione.

### 7.3 Le due inizializzazioni non sono intercambiabili

- Continuare i pesi **imitativi V26** amplia il teacher guidato dai riferimenti dei nuovi soggetti.
- Riprendere un actor **35D J19A/J20** amplia uno student già costruito per operare senza quei target.

Sono esperimenti distinti. La spiegazione data all'utente sceglie come percorso iniziale discusso la continuazione della rete imitativa esistente, seguita dal passaggio allo student comune e all'ex-novo. Riutilizzare l'actor 35D esistente può essere un'inizializzazione utile dello student, ma non è stato deciso il protocollo preciso.

## 8. Come includere nuovi modelli nella fase imitativa

### 8.1 Un unico insieme di pesi, più ambienti fisici

Non si aggiunge semplicemente un file `.osim` a un dataset di input della rete. Il file crea un ambiente fisico diverso; la rete riceve le osservazioni prodotte da quell'ambiente, genera target protesici e viene aggiornata sulla qualità degli episodi.

La strategia discussa è inizializzare dal checkpoint imitativo V26 esistente e proseguire il training con episodi distribuiti sui soggetti preparati. Esempio concettuale: alcuni episodi AB06, altri AB07, altri AB08; tutti aggiornano lo stesso insieme di pesi. L'imitazione nativa è training PPO con reward imitativa nel runtime del progetto; non viene automaticamente trasformata in una semplice regressione offline.

AB06 deve continuare a essere campionato. Allenare prima tutto AB07, poi tutto AB08 senza conservare esperienza dei domini precedenti potrebbe perdere capacità già apprese. Il curriculum può aggiungere soggetti progressivamente, ma il suo stato finale deve mantenere una distribuzione multi-modello.

### 8.2 Riferimenti corretti per il soggetto

Ogni ambiente deve usare target, eventi e convenzioni coerenti con il proprio trial. Gli shift/template V26 AB06 non sono automaticamente norme universali. La costruzione del riferimento controlaterale/protesico va verificata per ciascun soggetto e velocità, conservando la metodologia di trasformazione appropriata.

La policy imitativa può ricevere informazioni privilegiate durante il training. Sono strumenti del teacher, non ingressi autorizzati dell'actor finale. La qualifica multi-modello del teacher precede la raccolta di label per lo student.

### 8.3 Resume e budget restano da decidere

Non sono stati fissati numero di aggiornamenti, batch, learning rate, epoche, sigma, checkpoint iniziale, conservazione di Adam/scheduler/KL o calendario dei soggetti. Conservare i pesi non implica automaticamente conservare tutta la storia dell'ottimizzatore.

La nuova continuazione deve avere cartella e manifest propri. Il trainer attuale può rifiutare variazioni di configurazione nel resume: sarà necessario definire e verificare il trasferimento, senza aggirare silenziosamente i controlli di compatibilità.

### 8.4 Campionamento bilanciato

Il progetto proposto distingue almeno: soggetto, trial/velocità, fase, start, condizioni nominali e recovery. La scelta dei pesi deve essere esplicita. Un criterio iniziale possibile è campionare prima il soggetto, poi la condizione e lo start, ma non è già un sampler implementato.

Vanno controllati transizioni, episodi completi, terminazioni e contributo all'update per ciascun dominio. Il dominio più lento o con più fallimenti non deve sparire dal batch, né quello con più file dominare automaticamente il training.

## 9. Teacher, student e DAgger

### 9.1 Perché il passaggio deployable è necessario

Il parent imitativo V26 legge quattro target healthy che non sono disponibili nell'ex-novo. Il transplant diretto 39→35 del ramo B0820 eliminava informazioni utili e collassava già prima del PPO; critic warm-up e ulteriori aggiornamenti non hanno dimostrato un recupero sufficiente. La catena V26C introduce invece refit e qualifica closed-loop.

Se il nuovo teacher continua a usare ingressi privilegiati, la rimozione degli ingressi richiede ancora questo passaggio. Non si può dedurre che l'imitazione multi-modello risolva da sola la dipendenza dai target.

### 9.2 Metodo storico riusabile

La guida V26C descrive raccolta nel runtime target, derivazione/refit BASE 35D, riattivazione delle feature controller nello stesso actor, raccolta Markov/DAgger, preservazione funzionale dei casi riusciti e qualifica J19B/J19C; soltanto dopo vengono costruiti critic e stato J20.

Nella prima raccolta storica J1 le azioni erano prescritte dalla traiettoria base; il parent neurale V26 serviva a inizializzare J2. Non si deve riscrivere J1 come una raccolta di azioni del teacher neurale appreso. Per la nuova raccolta multi-modello restano da identificare precisamente oracolo, funzione che produce le label e stato/evento interrogato, per ciascun blocco nominale o DAgger.

Il metodo è riusabile. Gli script J storici, tuttavia, contengono path, hash, profili e condizioni AB06: non sono già un collector/fitter universale. La loro parametrizzazione multi-modello è lavoro futuro.

### 9.3 DAgger spiegato attraverso un errore di rollout

Uno student che anticipa l'appoggio su AB07 cambia posizione, velocità e contatto. Da quel momento visita stati diversi da quelli della dimostrazione nominale. Ripetere soltanto i campioni del cammino perfetto può lasciare scoperta questa distribuzione.

Il ciclo proposto è: eseguire lo student → registrare gli stati realmente visitati → interrogare il teacher/oracolo qualificato sullo stesso stato/evento → aggiungere coppie osservazione–azione → aggiornare lo student → ripetere su tutti i domini di training. È il principio di dataset aggregation, introdotto per affrontare la dipendenza fra errori di una policy e stati successivi. [DAgger, fonte primaria](https://proceedings.mlr.press/v15/ross11a.html).

La qualità delle label fuori dagli stati nominali non è garantita dal nome del metodo. Teacher, allineamento degli eventi, scala dell'azione e competenza sulle recovery devono essere verificati. Non basta associare a uno stato perturbato una riga cinematica arbitraria del trial.

### 9.4 Un solo student comune

L'obiettivo è uno student che raccolga capacità dai diversi domini. Non sono necessari 22 actor finali separati. Teacher specifici del dominio significa prima di tutto riferimenti/oracoli corretti; eventuali reti teacher separate o un teacher comune multi-modello sono scelte implementative da qualificare.

Nel percorso chiarito all'utente, la proposta iniziale è un teacher imitativo V26 comune continuato su più ambienti e uno student comune derivato/aggiornato sulla loro distribuzione.

### 9.5 Recovery e preservazione delle capacità

Nella storia V26C una raccolta recovery pari a circa 3,34% delle righe contribuiva a circa 84,81% dell'errore quadratico iniziale nel refit respinto. Questo valore era un indicatore della possibile influenza sull'update, non una quota del gradiente direttamente misurata. Mostra perché contare le righe non basta a controllare il peso delle condizioni difficili.

Per il nuovo training vanno misurati errori, scale delle label, pesi e contributi alla loss/gradiente per soggetto e blocco nominale/recovery. La preservazione dei casi già riusciti deve essere verificata con rollout a pesi congelati, non dedotta dal solo valore della loss supervisionata.

## 10. La policy finale non riceve anatomia o identità del paziente

### 10.1 Correzione della proposta iniziale

L'assistente aveva proposto di confrontare l'actor 35D con un actor condizionato da massa, lunghezze e inerzie. L'utente ha chiarito che nel caso reale la rete non conoscerebbe le caratteristiche del paziente. La proposta era fondata su un'assunzione non condivisa di disponibilità di quei dati e viene esclusa dal percorso iniziale.

Non saranno aggiunti all'actor finale descrittori anatomici espliciti o un `subject_id`. Questo vincolo non impedisce al simulatore di usare le masse/inerzie corrette, al manifest di identificare i soggetti o alle metriche di essere aggregate per modello.

Teacher e critic possono avere informazioni privilegiate durante il training. Il loro uso deve restare separato dall'inferenza dell'actor; la possibilità non equivale a un'autorizzazione a introdurre oggi nuovi ingressi del critic.

### 10.2 Strategia senza descrittori

La rete agisce nei diversi ambienti attraverso gli stessi segnali disponibili dalla protesi: angoli/velocità misurabili, stato SEA, carico ipsilaterale sensabile, eventi e stato causale del controllore/FSM, secondo il contratto da verificare. Deve imparare un comportamento efficace attraverso la variabilità fisica osservata durante il training.

Il fatto che un segnale esista nel simulatore non dimostra che sia disponibile sul dispositivo. «Causale», «presente nel vettore 35D» e «misurabile in deployment» sono proprietà da verificare separatamente.

### 10.3 Eventuale storia di osservazioni e comandi

Se due sistemi richiedono risposte differenti pur presentando osservazioni istantanee simili, una breve storia causale di sensori e comandi potrebbe aiutare a inferire la dinamica. È una possibilità futura; non è stata scelta una rete ricorrente, una finestra temporale, un encoder o un modulo di adattamento.

La letteratura sulle policy universali e su RMA motiva questa possibilità, ma non prova che sia necessaria o sufficiente per il nostro sistema. [Universal Policy/OSI](https://arxiv.org/abs/1702.02453), [RMA](https://arxiv.org/abs/2107.04034). Nella prima fase si osserva il trasferimento del sistema attuale prima di introdurre nuova architettura.

## 11. Dipendenza dal peso corporeo nel preprocessing: questione aperta concreta

### 11.1 Che cosa fa il codice attuale

In [osim_trj_cmc_like.py](<../../Trajectory Generator/osim_trj_cmc_like.py>) il peso corporeo usato dall'ambiente è calcolato come:

```text
BW = massa_totale_modello_OpenSim * 9.80665
carico_actor = forza_normale_piede_protesico / BW
```

La massa non appare come feature esplicita nell'actor 35D. Determina però la scala di `online_left_normal_grf_bw` e interviene nella trasformazione che alimenta l'adattatore degli eventi/FSM. L'audit ha verificato il calcolo di `_body_weight_n`, il payload gait e l'assemblaggio delle feature continue del carico.

Questo punto è distinto dal descriptor anatomico proposto e poi escluso. Non sono stati aggiunti nuovi dati del paziente, ma il preprocessing esistente presuppone una quantità fisica ricavata dal modello.

### 11.2 Perché interessa il vincolo dell'utente

Una policy può non leggere una colonna chiamata «massa» e dipenderne comunque se gli ingressi sono normalizzati con la massa. Per dichiarare funzionamento senza caratteristiche note del paziente, occorre qualificare tutta la catena di inferenza, inclusi normalizzazioni, eventi, soglie e reset.

Non è stata scelta la sostituzione. Possibili direzioni da confrontare in una futura attività sono una scala fissa non specifica del paziente, segnali in unità fisiche oppure una calibrazione/stima causale realmente disponibile dal dispositivo. Non si assume che il carico su una sola protesi identifichi correttamente il peso totale; una stima deve essere verificata su movimento e appoggio variabili.

### 11.3 Come interpretare il primo zero-shot

Se il primo test conserva il preprocessing storico BW, misurerà il trasferimento del checkpoint nel suo contratto originale, con la massa nota al simulatore. Sarà informativo sulla generalizzazione del sistema simulato, ma non dimostrerà ancora inferenza senza alcuna caratteristica del paziente.

Cambiare invece la normalizzazione prima del rollout, mantenendo gli stessi pesi, cambia la distribuzione degli ingressi. Un eventuale fallimento potrebbe derivare da questa migrazione e non dal nuovo corpo. I due effetti devono essere distinti nel protocollo, senza sostituire silenziosamente la scala.

La scelta sull'ordine dei due test e sull'eventuale migrazione resta aperta. Il vincolo finale dell'utente rimane quello di non fornire anatomia o identità del paziente all'actor e di verificare le dipendenze indirette.

## 12. Morphology corridor: conservazione iniziale e futura revisione

L'utente ha deciso di mantenere il corridor attuale in prima istanza, ritenendolo già abbastanza largo. Questa scelta prevale sulla proposta precedente di sostituirlo immediatamente con un profilo multi-soggetto. Non viene modificato né allargato per far passare i nuovi soggetti.

Conservarlo permette di limitare i cambiamenti del primo confronto. Non certifica che esso descriva correttamente ogni anatomia o cadenza. Per ogni nuovo modello bisogna separare:

- disponibilità del campione e motivo di eventuale scarto;
- correttezza di lato, segno, unità, coordinate e allineamento di fase;
- riferimento raw della policy, reference served e cinematica actual;
- violazione della banda, qualità fisica del movimento e terminazione;
- comportamento prescribed di controllo e comportamento della policy.

La copertura e le violazioni possono essere misurate anche sui soggetti di test come risultati di valutazione. Se si cambia il corridor in risposta a quei risultati, tali soggetti diventano dati di sviluppo per quella scelta; non si può conservare contemporaneamente l'affermazione di test finale sigillato.

La normalizzazione o ricostruzione multi-soggetto del corridor resta un lavoro futuro. Prima di un suo uso nel training, la costruzione dovrà dipendere dai soli soggetti di training/sviluppo ammessi e da un protocollo esplicito per cadenza e disponibilità dei campioni.

Il TODO storico del 22 agosto che prevedeva un corridor multi-soggetto non viene cancellato, ma è differito rispetto alla scelta iniziale attuale. Le bande di sicurezza fisica e i criteri di qualifica non vanno confusi con la larghezza del corridor.

## 13. Ex-novo multi-modello e warm-start

### 13.1 Significato di ex-novo nel repository

Qui ex-novo indica il cambio verso l'obiettivo di generazione protesica senza termini imitativi attivi e senza i target privilegiati nell'actor finale. Non significa necessariamente inizializzare i pesi della policy da zero: il percorso V26 usa uno student qualificato come warm-start.

Il successo della fase imitativa multi-modello non garantisce il successo dell'ex-novo. Il cambio di reward, gli ingressi disponibili e la distribuzione iniziale possono introdurre differenze aggiuntive.

### 13.2 Attore comune e distribuzione dei domini

Il passaggio previsto conserva l'actor deployable qualificato. Durante il warm-up del critic e durante PPO, il campionamento deve comprendere gli stessi soggetti e condizioni di training pertinenti. Un PPO successivo solo su AB06 potrebbe specializzare di nuovo una policy appena resa multi-modello.

Il cambio di reward non autorizza un cambio simultaneo non documentato di modello, reset, governor, detector, action bounds o normalizzazioni. Le differenze necessarie tra fasi devono essere esplicite e sottoposte a confronto.

### 13.3 Critic e ottimizzatore

La proposta discussa usa critic e stato di ottimizzazione adatti al nuovo obiettivo, con warm-up critic-only e controllo dell'immutabilità dell'actor. Le modalità precise di inizializzazione e di riuso degli stati sono ancora da fissare.

La verifica del critic deve coprire i domini e comprendere errori di valore, distribuzione degli errori, loss libera/clippata e validazione indipendente. Un aggiornamento critic-only o il solo valore di explained variance non dimostra convergenza.

Occorre verificare anche logstd/sigma, Adam, coefficiente KL, learning rate effettivo, contatori e restore. Il dimezzamento KL osservato nei pilot recenti durante critic-only resta un fenomeno da controllare, senza attribuirgli automaticamente tutte le regressioni.

### 13.4 Pilot prima della campagna lunga

Il primo PPO multi-modello dovrà essere breve, con budget e criteri fissati prima dell'esecuzione, seguito da valutazioni per soggetto. Non sono stati scelti iterazioni, durata, learning rate o sigma. I valori J20 storici non costituiscono automaticamente la ricetta adeguata alla nuova distribuzione.

La perdita di una capacità AB06 o il peggioramento su un dominio va resa visibile: ritorno medio globale e `checkpoint_best` possono nascondere regressioni su singoli soggetti.

## 14. Domain randomization rinviata

L'utente concorda di usare inizialmente i modelli EPIC preparati e verificati, senza perturbazioni sintetiche aggiuntive di dinamica o contatto. Le differenze naturali tra soggetti fanno già parte del training multi-modello.

La proposta futura di randomizzazione consiste nel simulare piccole variazioni plausibili attorno a un modello, per esempio massa/inerzie o attrito. Quando sarà affrontata, andrà mantenuta coerenza fra parametri correlati, geometria, carichi e GRF; non si possono cambiare arbitrariamente masse conservando ogni altra grandezza come se il sistema fisico fosse invariato.

La randomizzazione può ampliare la distribuzione di training, ma non sostituisce la valutazione su modelli esclusi. [Fonte primaria su dynamics randomization](https://arxiv.org/abs/1710.06537).

Non sono stati approvati range percentuali, perturbazioni sensoriali, cambi di parametri SEA o un curriculum randomizzato. La scelta iniziale non vieta di definire più avanti una valutazione stocastica della policy; il primo baseline proposto è però deterministico e senza nuovi interventi fisici sintetici.

## 15. Primo esperimento informativo: rollout congelato su un modello diverso

### 15.1 Domanda dell'utente

L'utente propone di eseguire la rete attuale non generalizzata su un modello diverso da AB06 prima del nuovo training. È un test zero-shot utile: può mostrare quanto trasferimento esiste già e quali aspetti richiedano adattamento.

Non è stato eseguito alcun rollout AB07/AB08 della policy V26 in questa attività. Non esistono quindi nuovi risultati di successo o fallimento cross-subject da riportare.

### 15.2 Prerequisito: modello qualificato

Il modello scelto deve avere protesi correttamente posizionata, coordinate canoniche, IK/GRF sincronizzati, donor/SEA fissati e profili di contatto/eventi verificati. Cambiare soltanto il path verso un modello sano raw non realizza un test valido della nostra policy protesica.

Un controllo prescribed deve chiarire la fattibilità dell'ambiente prima di interpretare il comportamento della rete. Un errore geometrico o un donor diverso dalla baseline può produrre un fallimento che non misura generalizzazione.

### 15.3 Confronti proposti, non ancora schedulati

| Prova | Policy / riferimento | Informazione principale |
|---|---|---|
| Controllo fisico | Reference prescribed del soggetto | Fattibilità di modello, protesi e catena di inseguimento |
| Trasferimento imitativo | Teacher V26 congelato, riferimenti corretti del nuovo soggetto | Trasferimento del teacher guidato dal target |
| Trasferimento deployable | Student/warm-start oppure policy ex-novo V26 congelata, checkpoint da selezionare | Trasferimento senza target privilegiati nell'actor |
| Riferimento AB06 | Stessi checkpoint nel dominio storico con protocollo dichiarato | Individuazione di regressioni runtime e punto di confronto |

La prova imitativa può usare i riferimenti del nuovo soggetto: per questo il suo successo non dimostra la stessa autonomia della policy deployable. Valutare anche lo student prima del PPO ex-novo può isolare il contributo del training successivo, ma non è stato deciso di rendere obbligatorie tutte le prove della tabella.

### 15.4 Congelamento e comparabilità

Per ogni prova vanno fissati checkpoint/hash, codice, scaler/preprocessing, modalità della policy, action bounds, configurazione fisica, target, start e finestra di analisi. Non devono avvenire update, fit DAgger, riadattamento del critic o calibrazioni della policy durante il test zero-shot.

Gli start devono essere comparabili rispetto agli eventi e alle condizioni del trial. Usare lo stesso numero di secondi assoluti su registrazioni diverse può corrispondere a fasi differenti. Velocità/cadenza e durata della finestra vanno rese esplicite.

L'uso del BW storico va dichiarato come discusso nella sezione 11. I profili fisici e il fitting del modello fanno parte della preparazione dell'ambiente, ma non autorizzano tuning della policy sui risultati del test.

### 15.5 Metriche da separare

- Forma e ampiezza delle traiettorie knee/ankle, con metriche per fase e allineamento dichiarato.
- Errori raw→served e served→actual: non attribuire al SEA un target già insufficiente.
- Completamento, numero/durata dei cicli, HS/TO e contatori invalidi/resync.
- Penetrazione, carico, contatto e impulso, distinguendo episodi interrotti.
- Coppie/potenza SEA, area positiva ankle e regolarità knee con scale dichiarate.
- Reserve e fattibilità del lato biologico; eventuale compensazione del plant.
- Copertura e costo morphology, campioni indisponibili e motivi di scarto.
- Statistiche di saturazione/clipping e variabilità, se successivamente inclusa.

Le soglie di promozione specifiche non sono state preregistrate. Un confronto descrittivo iniziale deve restare tale; il solo completamento o ritorno non dichiara una policy generalizzata.

### 15.6 Effetto sullo split futuro

AB07 è un candidato naturale per il pilot perché è già locale, ma la scelta finale non è fissata. Se il suo risultato orienta architecture, training, corridor o iperparametri, AB07 entra nel gruppo di sviluppo. Non deve poi essere presentato come test finale mai osservato.

## 16. Split 70/30 o 80/20 e controllo del leakage

### 16.1 Scelta condivisa e dettagli ancora aperti

L'utente propone training sul 70% e test sul 30% dei modelli EPIC, oppure 80/20. Entrambe le ripartizioni sono ammissibili; non è stata scelta una delle due né un elenco di soggetti.

L'unità dello split è il soggetto/modello anatomico, non campioni temporali o file scelti casualmente. Tutti i trial, velocità e stride di un soggetto appartengono allo stesso gruppo per la valutazione cross-subject.

AB06 deve restare nello sviluppo perché i pesi di partenza e il corridor derivano già da quel soggetto. Un suo eventuale uso come test non potrebbe essere descritto come soggetto mai visto.

### 16.2 Serve anche validazione

Checkpoint, learning rate e scelte di pipeline devono poter essere selezionati senza leggere il test finale. Per questo è proposta una componente di validazione, preferibilmente con soggetti distinti dal training, ricavata dal gruppo di sviluppo o definita come terzo insieme.

Le percentuali si applicheranno ai soggetti effettivamente eleggibili dopo l'inventario. Se fossero tutti 22, esempi di arrotondamento sarebbero 15/7 o 16/6 vicino a 70/30, e 18/4 vicino a 80/20, prima di esplicitare la quota di validazione. Questi sono esempi aritmetici, non assegnazioni approvate.

### 16.3 Che cosa deve restare escluso dai soggetti test

Nessun soggetto finale test deve contribuire a update imitativi, label/fit DAgger, PPO, fit degli scaler globali, costruzione/adattamento del corridor o selezione della ricetta. I suoi dati necessari a preparare il modello fisico e a misurare l'esito possono essere usati nel ruolo dichiarato di ambiente/evaluazione, senza convertirli in supervisione della policy.

Eventuali trasformazioni standard basate su informazioni del modello devono essere fissate prima dei risultati e compatibili con il vincolo sugli ingressi del dispositivo. La distinzione fra preparazione fisica e informazione dell'actor è essenziale.

### 16.4 Distribuzione di velocità e identità del soggetto

Soggetti e velocità possono risultare confusi se certi soggetti compaiono soltanto in alcune condizioni. Va inventariato il supporto comune e va dichiarato se il test misura nuova anatomia, nuova velocità o entrambe. I risultati non vanno aggregati come se queste domande fossero equivalenti.

### 16.5 Tutti i modelli e generalizzazione

Per usare tutti i soggetti anche in prove di generalizzazione si può prevedere leave-one-subject-out, come alternativa futura al singolo split. Non è stato scelto come protocollo iniziale. Un fit finale su tutti i soggetti può seguire le valutazioni held-out, ma la sua qualità non va stimata solo sugli stessi soggetti usati nel fit.

## 17. Cosa è riusabile e cosa manca nel software

| Componente | Disponibilità | Lavoro prima della campagna |
|---|---|---|
| Builder ABx e utility di conversione | Esistenti | Fitting, IK effettiva/fallback qualificato, selezione sessioni/trial e output indipendenti |
| Modulo actor/critic asimmetrico | Esistente | Verificare feature, dimensioni, scaler e separazione delle informazioni per ogni dominio |
| Teacher V26 | Allenato su AB06 | Continuazione multi-modello e qualifica per dominio |
| Metodo BASE/Markov/DAgger/preservation | Documentato e applicato su AB06 | Parametrizzare collector/fitter e bilanciare nuovi domini |
| Runtime CMC-like con setup | Esistente | Registro domini e scheduler modello/trial; reset e finestre coerenti |
| FSM v3 e dual-stream GRF/detector | Esistenti | Profili e geometria per soggetto, registri di provenienza e gate |
| Warm-up critic e audit restore | Esistenti | Distribuzione multi-modello e calibrazione del valore per dominio |
| Morphology corridor | Profilo AB06 corrente | Conservazione iniziale e diagnostica di copertura; revisione rinviata |
| Rollout/plot/metriche | Esistenti | Comparatore per modello/condizione e gestione esplicita dei fallimenti |

### 17.1 Accoppiamento reward e ingressi target

[env_factory.py](<../../Trajectory Generator/baseline_MLP/env_factory.py>) imposta `include_imitation_target_observation` in base a `reward_mode == imitation`. Il runtime accoppia inoltre il gait clock al teacher e alla disponibilità di colonne dell'actor. Spegnere il clock può attivare un fallback di target controlaterale allo stesso tempo.

Per un contratto deployable uniforme servono controlli espliciti: i target privilegiati necessari a teacher/reward non devono comparire accidentalmente nell'actor. Non basta eliminare quattro valori dalla factory senza verificare la costruzione dell'oracolo.

### 17.2 Reset e probe del trainer

Il reset attuale riusa il contesto del modello caricato e non seleziona soggetti da un registro. `imitation_initialize_to_target` può cambiare lo stato iniziale rispetto all'ex-novo. Le distribuzioni di reset devono essere dichiarate e qualificate.

Il probe del trainer costruisce l'ambiente grezzo in una parte del percorso storico, mentre i worker utilizzano factory/wrapper completi. Prima di una nuova lineage va verificato che observation/action space e feature riflettano l'ambiente realmente usato, senza slicing o dimensioni assunti per AB06.

### 17.3 Profili e coordinate

Il runtime usa nomi e lato protesico LEFT canonici; alcuni profili detector binario sono vincolati da path/hash di AB06. Cambiare soltanto il JSON o il setup non qualifica la nuova geometria. Occorre un registro esplicito dei profili validati, conservando la separazione tra GRF primaria per carico/contatto e detector per eventi.

La larghezza del vettore privilegiato può dipendere dalle coordinate disponibili nel modello. Il critic storico 84D non deve essere assunto compatibile senza verifica. Il contratto deve essere determinato per nome, ordine e unità, non soltanto per numero di colonne.

### 17.4 Perimetro delle modifiche future

Rete, policy, collector DAgger, training, registri e analisi devono vivere in `Trajectory Generator/`. Le utility di preparazione modello già in `tools/` e `scripts/` mantengono il proprio ruolo, senza introdurre logica neurale nel root del simulatore.

Le modifiche devono essere coerenti per Windows x86 e macOS arm64. Il plugin C++ o la semantica SEA richiedono richiesta esplicita; non sono stati modificati in questa attività. I sette plot canonici per rollout vanno sotto `plot/<rollout>/` nel root, non nelle cartelle run.

## 18. Roadmap discussa e dipendenze

Questa roadmap ordina il lavoro da rendere concreto dopo il via. Non contiene lanci schedulati, comandi autorizzati o budget già approvati.

| Fase | Contenuto | Uscita attesa | Stato |
|---|---|---|---|
| P0 | Report e recupero contesto | Decisioni, fonti e TODO coerenti | Completamento documentale odierno |
| P1 | Baseline e protocollo minimo | Checkpoint/hash, donor/SEA, ingressi/preprocessing, modelli e prove fissati | Da formalizzare |
| P2 | Pilot pipeline AB06/07/08 | Bundle e controlli prescribed qualificati per i tre soggetti | In attesa del via |
| P3 | Primo zero-shot | Metriche per policy/modello, interpretazione fisica e del preprocessing | Proposto, non eseguito |
| P4 | Corpus e split | Inventario soggetti eleggibili, download mirato, train/validation/test | Da predisporre dopo il pilot |
| P5 | Continuazione imitativa | Teacher comune aggiornato sui domini training e qualificato | Non implementata/lanciata |
| P6 | Student multi-modello | Actor causale comune, DAgger e preservazione verificati | Non implementato/lanciato |
| P7 | Critic + pilot ex-novo | Restore/calibrazione e PPO breve sulla distribuzione multi-modello | Non implementati/lanciati |
| P8 | Valutazione finale | Test per soggetto escluso, costi e variabilità documentati | Futuro |

I gate devono includere criteri fisici e qualità cinematica; nessun nuovo checkpoint viene promosso soltanto perché il software completa una fase. Un esito negativo può richiedere ritorno alla preparazione fisica, al teacher, agli ingressi o al training: queste cause vanno discriminate.

Il primo pilot con tre soggetti è una qualifica tecnica del metodo. Non dimostra da solo copertura del corpus o indipendenza dal modello. Per le conclusioni finali saranno necessarie condizioni e repliche compatibili con il budget da definire.

## 19. Debiti e rischi da mantenere visibili

### 19.1 Debiti già documentati

Il bug bootstrap PPO di Ray/RLlib 2.55.1, descritto nell'[audit del 29 settembre](2026-09-29_audit_RL_39d_bootstrap_e_aggiornamento_PPO.md), resta un TODO esplicitamente rinviato dall'utente. Non è stato corretto né reso un nuovo prerequisito imposto da questo report. Prima di una campagna PPO andrà dichiarato quale stack si usa e quale confronto separato si vuole eventualmente svolgere.

Restano inoltre da verificare calibrazione del critic, trattamento KL nel critic-only, probe dell'osservazione, reset e diagnostici FSM, profili vincolati ad AB06 e portabilità della pipeline corrente su Windows. I vecchi TODO di 39D sono sospesi con il ramo, non risolti dai piani di generalizzazione.

### 19.2 Interpretazioni da evitare

| Osservazione | Che cosa non dimostra da sola |
|---|---|
| `--validate-only` termina con PASS | IK calcolata, model fitting corretto o rollout biomeccanico qualificato |
| Builder riporta `complete` | Successo di tutti i tool, assenza di fallback o qualità di una falcata |
| Smoke di pochi millisecondi termina | Stabilità durante un episodio o capacità della policy |
| Modello sano già scalato | Protesi correttamente dimensionata e posizionata |
| Teacher esegue un trial | Competenza sulle recovery dello student |
| Loss imitativa bassa | Autonomia closed-loop o generalizzazione anatomica |
| Policy completa 5 s | Forma del passo sufficiente, cicli validi o robustezza |
| Actor senza colonna massa | Assenza di dipendenza dalla massa nel preprocessing |
| Ritorno medio maggiore | Miglioramento su tutti i soggetti o confronto valido fra reward diverse |
| Test AB07 usato per scegliere la ricetta | Test finale sigillato su AB07 |

La verifica del builder mostra anche che lo status `complete` dipende dallo smoke assente o dal suo returncode e può coesistere con IK fallback/RRA diagnostico fallito. I default storici comprendono uno smoke di 0,03 s: è una prova di eseguibilità iniziale, non il gate della nuova campagna.

## 20. TODO da propagare nei daily

Gli ID permettono di riprendere il lavoro senza confondere proposte, preparazione e risultati. Le voci di esecuzione restano subordinate al via e ai prerequisiti; nessuna checkbox aperta costituisce un comando di lancio.

### 20.1 Completamenti documentali

- [x] C01 — Recuperare le decisioni della conversazione e distinguere il ramo 39D sospeso dalla catena V26 da generalizzare.
- [x] C02 — Verificare la presenza della pipeline ABx e identificare i limiti di IK, fitting donor e gestione multi-trial.
- [x] C03 — Chiarire continuazione dei pesi imitativi, ruolo dello student/DAgger ed ex-novo sullo stesso insieme di domini.
- [x] C04 — Registrare l'esclusione dei descrittori anatomici dall'actor e la dipendenza BW ancora aperta nel preprocessing.
- [x] C05 — Documentare corridor iniziale invariato, assenza di domain randomization nella fase 1 e attesa del via.

### 20.2 Priorità e preparazione del pilot

- [ ] G01 — Attendere il via esplicito dell'utente prima di download, preparazione dei nuovi modelli, conversioni, rollout o training; rispettare l'ambito concreto del via.
- [ ] G02 — Selezionare e congelare checkpoint imitativo, student/warm-start ed ex-novo di riferimento, con path, hash, config, feature, scaler e action bounds.
- [ ] G03 — Fissare donor, trasformazione protesica e parametri SEA/controller effettivi coerenti con la baseline, verificando anche gli override del runtime.
- [ ] G04 — Identificare interprete OpenSim e dipendenze operative; predisporre verifiche coerenti per macOS arm64 e Windows x86.
- [ ] G05 — Costruire il manifest dei tre soggetti locali e verificare contenuto/schema numerico dei MAT, condizioni, statici e corrispondenza modello–IK–GRF.
- [ ] G06 — Definire e verificare il fitting geometrico protesico per soggetto, distinguendo hardware fisso, regolazioni e convenzione di massa.
- [ ] G07 — Definire il percorso IK effettivo o qualificare esplicitamente il fallback dataset; evitare il falso completamento da dry-run.
- [ ] G08 — Rendere selezione sessioni/trial e destinazioni dei bundle indipendenti, con provenance e senza sovrascritture silenziose.
- [ ] G09 — Qualificare caricamento, reset, posa, contatto, stato SEA, tracking prescribed e reserve sui tre modelli prima della policy.
- [ ] G10 — Predisporre e qualificare profili GRF/detector per modello mantenendo dual-stream, coordinate LEFT e contratti degli eventi.

### 20.3 Ingressi, zero-shot e corpus

- [ ] G11 — Auditare tutti gli ingressi e il preprocessing dell'actor per disponibilità dalla protesi, inclusi BW, eventi, clock, soglie e reset; scegliere e verificare l'eventuale migrazione senza metadati del paziente.
- [ ] G12 — Fissare il protocollo zero-shot iniziale: modello, checkpoint congelati, preprocessing, reference, start/eventi, finestre, modalità della policy e metriche.
- [ ] G13 — Dopo via e gate fisici, eseguire i rollout zero-shot e i controlli concordati, con sette plot canonici per rollout sotto `plot/` e analisi delle cause degli esiti.
- [ ] G14 — Inventariare il corpus EPIC treadmill remoto, scaricare dopo via i componenti necessari e determinare soggetti/condizioni realmente eleggibili.
- [ ] G15 — Scegliere 70/30 oppure 80/20, definire validazione, assegnazioni/seed e regole contro leakage; conservare AB06 nello sviluppo e classificare i soggetti pilot come sviluppo se usati per le scelte.

### 20.4 Training e qualifica multi-modello

- [ ] G16 — Formalizzare la continuazione della rete imitativa V26: pesi/stato optimizer, budget, reward, target per soggetto e distribuzione multi-modello con conservazione di AB06.
- [ ] G17 — Implementare e verificare registro domini, sampler/scheduler, reset, factory/probe e schema effettivo dei worker per tutti i modelli.
- [ ] G18 — Qualificare il teacher/oracolo e la produzione di label nominali e recovery per dominio, con unità, stato, fase e contratto azioni coerenti.
- [ ] G19 — Parametrizzare collector/refit/DAgger multi-modello, bilanciare soggetti e recovery, misurare contributi alla loss/update e preservare le condizioni riuscite.
- [ ] G20 — Preregistrare ed eseguire la qualifica dello student comune su modello×condizione×start, distinguendo forma, served/actual, cicli, contatto, SEA e reserve.
- [ ] G21 — Conservare inizialmente il corridor attuale e analizzarne copertura/allineamento per modello; tenere distinta la futura revisione dai risultati sul test.
- [ ] G22 — Definire critic/optimizer e warm-up multi-modello, verificando immutabilità actor/logstd, valore non clippato, errori per dominio, Adam/KL e restore.
- [ ] G23 — Preregistrare ed eseguire un pilot ex-novo breve sugli stessi domini di training, con budget/gate e valutazioni per soggetto prima di estensioni.
- [ ] G24 — Svolgere il test finale sui soggetti esclusi e documentare fallimenti, costi di apprendimento, condizioni, variabilità e limiti della generalizzazione.
- [ ] G25 — Eseguire la verifica live Windows della pipeline scelta; le verifiche storiche del plugin PI non certificano la nuova campagna.

### 20.5 Lavori rinviati o condizionati a nuove decisioni

- [ ] G26 — Mantenere rinviato il fix bootstrap PPO; definirne in attività separata ambito e confronto prima di modificarlo o attribuirgli gli esiti della campagna.
- [ ] G27 — Riprendere l'adattamento del morphology corridor solo dopo la diagnostica iniziale e con dati/protocollo ammessi dallo split.
- [ ] G28 — Valutare in una fase successiva l'utilità di una storia causale di osservazioni/comandi, senza introdurre descrittori anatomici espliciti o identità del paziente.
- [ ] G29 — Considerare domain randomization soltanto in una fase successiva, con range fisicamente coerenti e protocollo separato.

## 21. Verifiche eseguite e limiti di validazione

### 21.1 Audit del 5 ottobre

Sono stati letti report e codice della pipeline, della lineage V26C e del runtime. La ricerca su fonti primarie ha identificato il corpus EPIC e inquadrato DG, DAgger e possibili estensioni future. Sono stati controllati inventario locale, esistenza di bundle/checkpoint candidati e differenze dei donor XML.

È stata eseguita nella prima parte dell'audit la verifica read-only `python3 tools/build_abx_seasea_pipeline.py --all-subjects --validate-only`: discovery PASS sui tre soggetti. Non ha eseguito conversioni, OpenSim o nuovi rollout. Il Python di sistema 3.12 usato in quella verifica non importava OpenSim; ciò non dimostra assenza di un altro ambiente Python adatto.

Le corrispondenze per nome dei 24 trial sono state controllate, ma non tutta la loro integrità numerica. Gli artefatti runtime di alcuni test maggio sono assenti: i risultati relativi sono citati come storici.

### 21.2 Completamento documentale del 6 ottobre

La bozza iniziata il 5 ottobre è stata completata e rinominata con la data di completamento. Sono stati ricontrollati i punti del preprocessing BW e registrati hash di sorgenti/modelli per identificare il contesto dell'audit. La revisione indipendente ha corretto la distinzione tra J1 prescribed e teacher appreso, e la natura del dato 84,81%.

I controlli finali di questa attività sono documentali: struttura, collegamenti locali, presenza dei temi/decisioni, coerenza degli ID TODO e loro propagazione nel daily. Non sono test di generalizzazione, né un nuovo PASS OpenSim/Windows. Test numerici, training, rollout e plot citati nei report precedenti non sono stati rieseguiti.

Esito dei controlli documentali: tutti i 26 collegamenti locali del report e i 148 del daily risultano esistenti; le 29 voci aperte G01–G29 e i cinque completamenti C01–C05 corrispondono esattamente tra i due file. Il daily conserva anche le 231 voci aperte univoche ereditate dal 1° ottobre. Coincidono tutti i sei hash delle sorgenti elencate in appendice e risultano presenti i sette percorsi candidati di checkpoint/modulo. La revisione indipendente della pipeline e della lineage V26 non ha rilevato altre correzioni sostanziali.

### 21.3 File modificati da questa richiesta

| File | Modifica |
|---|---|
| `reports/user/2026-10-06_generalizzazione_multimodello_epic_v26_strategia_vincoli_e_audit.md` | Report esaustivo, completato dalla bozza del 5 ottobre |
| `reports/daily/2026-10-06_daily-report.md` | Daily parziale con collegamento, decisioni e propagazione dei TODO |

La bozza `reports/user/2026-10-05_generalizzazione_multimodello_epic_v26_strategia_vincoli_e_audit.md` è stata rinominata, non lasciata come secondo report incompleto. Codice, modelli, configurazioni e checkpoint non sono stati modificati da questa richiesta. Il daily del 6 ottobre non è una chiusura `end_day`.

## 22. Istruzioni per riprendere il contesto

1. Leggere prima sezioni 1 e 3 per priorità e stato delle decisioni. Il via alle esecuzioni resta mancante.
2. Leggere sezioni 6 e 11 prima di proporre un rollout su AB07: donor/fitting, IK e normalizzazione BW possono confondere il risultato.
3. Selezionare checkpoint dalla lista candidata in appendice, registrando hash e contratto; non assumere che il nome V26 identifichi univocamente una policy.
4. Rendere concreto il pilot richiesto dall'utente e distinguere preparazione dei modelli, controlli prescribed e zero-shot. Non partire direttamente con un training multi-modello lungo.
5. Dopo il via, rispettarne lo scope: il consenso a un rollout non autorizza automaticamente download di tutto il corpus o nuove campagne PPO.
6. Usare gli ID G01–G29 e conservare nei daily quelli aperti. Le voci rinviate restano tali fino a una nuova decisione, senza diventare blocchi artificiali per le verifiche autorizzate.
7. Conservare gli artefatti 39D come riferimento storico; una sospensione non chiude i suoi TODO e non impone nuove analisi di quel ramo.

## Appendice A. Checkpoint candidati presenti sul disco

I seguenti percorsi sono stati verificati come esistenti durante l'audit. Sono candidati, non selezioni già approvate per la generalizzazione. Le directory di modulo e di checkpoint completo hanno ruoli diversi.

| Ruolo | Percorso relativo al repository |
|---|---|
| Teacher V26 nativo best87, modulo actor | `Trajectory Generator/runs/training/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter/rl_module_best` |
| Teacher V26 nativo best87, checkpoint completo | `Trajectory Generator/runs/training/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter/checkpoint_best` |
| J19A, export actor35 | `Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/j19a_runs/j19a_single_reproduction_v26c_2026-08-27_r1/rl_module` |
| J20, actor35 invariato + critic | `Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/j20_runs/j20_critic_warmup_v26c_2026-08-27_r1/checkpoint_last` |
| Pilot ex-novo V26C/J20, modulo best | `Trajectory Generator/runs/training/MLP_ExNovo_V26C_J20_pilot_2026-08-28/rl_module_best` |
| Pilot ex-novo V26C/J20, checkpoint completo best | `Trajectory Generator/runs/training/MLP_ExNovo_V26C_J20_pilot_2026-08-28/checkpoint_best` |
| Rewarm morphology neutro di fine agosto | `Trajectory Generator/runs/training/MLP_V26C_J20_critic_rewarm_morphology_split_equal_1p05_2026-08-31_r2/checkpoint_last` |

Il best del pilot ex-novo storico è all'iterazione logica 24 delle 50 nuove. Quel risultato riguarda AB06 e non prova generalizzazione. Il rewarm a pesi morphology uguali 1,05/1,05 è un riferimento neutro, non una nuova policy universalizzata.

## Appendice B. Contratto actor35 storico da auditare

La guida V26C descrive la topologia `35 → 256 tanh → 256 tanh → 4 output`. Le quattro uscite parametrizzano due medie d'azione e due log-standard-deviation, non quattro target articolari. Le azioni sono due endpoint protesici assoluti normalizzati, poi convertiti secondo bounds e governor; non sono direttamente coppie SEA.

| Indici | Feature storiche |
|---|---|
| 0–1 | `gait_phase_sin`, `gait_phase_cos`, colonne clock neutralizzate nella lineage deployable |
| 2–5 | `pros_knee_angle`, relativa velocità, `pros_ankle_angle`, relativa velocità |
| 6–9 | Angolo/velocità motore `SEA_Knee` e `SEA_Ankle` |
| 10–11 | `online_left_normal_grf_bw`, `online_left_in_contact` |
| 12–15 | HS, TO e sin/cos della fase ipsilaterale online |
| 16 | `online_left_cycle_duration_s` |
| 17–24 | Stato FSM, evento atteso, tempi normalizzati e credito di progresso del ciclo |
| 25–29 | Endpoint precedente knee, served q/v/a knee e comando SEA knee |
| 30–34 | Endpoint precedente ankle, served q/v/a ankle e comando SEA ankle |

Nel BASE storico le colonne clock e controller sono neutralizzate; nel Markov le controller vengono riattivate nello stesso actor35. L'elenco completo e ordinato rimane nella guida del 28 agosto. Questa tabella è un punto di controllo, non una promessa che ogni valore sia già disponibile sul dispositivo senza preprocessing specifico.

## Appendice C. Donor XML e identità delle sorgenti

### C.1 Parametri nei file donor

| Parametro knee / ankle | Donor default raw | XML `stiff321_500_pi-tuned` |
|---|---:|---:|
| K [Nm/rad] | 1000 / 700 | 321 / 500 |
| Kp | 3,9 / 8,8 | 18 / 11,3 |
| Kd | 9,7 / 9,7 | 11 / 11 |
| Fopt [Nm] | 100 / 250 | 100 / 250 |
| Jm | 0,01 / 0,01 | 0,01 / 0,01 |
| Bm | 0,1 / 0,1 | 0,1 / 0,1 |
| Impedence | false / false | false / false |

Fonti locali: [donor raw](../../models/AB06_SEASEA-raw/osimxml/AB06_SEASEA.osim) e [modello PI-tuned](../../models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi-tuned.osim). Sono valori letti negli XML, non una misura dei parametri effettivi dopo gli override della configurazione runtime. Il pin futuro deve verificare entrambi; la tabella non esaurisce tutti i parametri del loop PI.

### C.2 SHA-256 ricontrollati il 6 ottobre

| Sorgente | SHA-256 |
|---|---|
| `tools/build_abx_seasea_pipeline.py` | `f4411732e845cf4d233a2129e8a3ca6419bc422b31bf023572f33392fe34895c` |
| `scripts/run_opensim_sea_pipeline.py` | `d51544b58c303f338cacab4400e8d6deb5477a6c86b1f8f2b6b479a00b96ef7d` |
| `Trajectory Generator/baseline_MLP/env_factory.py` | `a728e07b4343e71ccf515f0ef7947f13735e13c3e39a16a3b5d52ea85d90d559` |
| `Trajectory Generator/osim_trj_cmc_like.py` | `6b2e806dc71ea45f3f71483746adf489f279cecc56d94c80453c3fd8b5f8ed65` |
| `models/AB06_SEASEA-raw/osimxml/AB06_SEASEA.osim` | `402f83755075f4bd040b3e5a89b5bcb4e14a03950bdc2d26be2e8963eab34e50` |
| `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi-tuned.osim` | `33e67d84bf11740eac509f620a143ad3c57d98c6f765d857e69c1892513de0c1` |

Gli hash identificano le sorgenti lette; non certificano una build eseguita. Il report non fissa ancora hash di checkpoint selezionati perché quella selezione resta aperta.

## Appendice D. Fonti locali per il prossimo agente

- [AGENTS.md](../../AGENTS.md): perimetro del simulatore, SEA, compatibilità OS, report e plot.
- [Pipeline AB06, 11 maggio](2026-05-11_ab06_seasea_epic_pipeline.md) e [daily 12 maggio](../daily/2026-05-12_daily-report.md): baseline mass-preserving/marker-calibrated e decisione RRA.
- [Pipeline unica ABx, 13 maggio](2026-05-13_pipeline_unica_abx_seasea.md): discovery e prove storiche AB07/AB08; non una nuova qualifica completa.
- [TODO generalizzazione del 22 agosto](2026-08-22_todo_generalizzazione_multimodello_epic.md): precedente formalizzazione, ora riaperta come priorità. La proposta di descriptor nell'actor è superata dal vincolo attuale; il corridor multi-soggetto è rinviato.
- [Guida V26C del 28 agosto](2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md): feature, parent, refit, qualifica e critic J20.
- [Audit PPO del 29 settembre](2026-09-29_audit_RL_39d_bootstrap_e_aggiornamento_PPO.md): bug bootstrap e limiti del confronto, fix ancora rinviato.
- [Daily 1° ottobre](../daily/2026-10-01_daily-report.md): esperimenti 39D conclusi, backlog e limiti; il ramo è ora sospeso temporaneamente.
- [Daily parziale del 6 ottobre](../daily/2026-10-06_daily-report.md): propagazione operativa dei TODO di questo report.
