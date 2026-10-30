# Daily Report — 2026-10-05

> Aggiornamento successivo del 2026-10-19: i riferimenti di questo documento
> alla filiera V12, al bundle V26C, alla campagna sigma o ai file nella radice
> di `baseline_MLP/validation/` sono storici: quei materiali sono stati rimossi.
> La workspace V26C e i suoi checkpoint sono conservati. Per rieseguire
> le procedure con input o prerequisiti rimossi occorre il loro ripristino.
> Vedere il [registro della pulizia](../cleanup/2026-10-19_baseline_validation_removal/README.md).

**Stato:** giornata chiusa con `end_day`, registrata al **5 ottobre 2026** su indicazione esplicita dell'utente.  
**Data materiale di questa chiusura:** 6 ottobre 2026, Europe/Rome.  
**Instruction check token:** `CMC_AGENT_OK_2026`.

## Cronologia e documenti consolidati

Questa chiusura raccoglie la discussione e il primo audit del 5 ottobre, oltre alla documentazione completata il 6 ottobre dopo l'interruzione della stesura. La data del daily è il 5 ottobre perché l'utente ha richiesto «end_day (segnalo al 5/10)». Le verifiche documentali e gli hash ricontrollati il 6 ottobre mantengono la propria data effettiva: non vengono retrodatati come esecuzioni del 5.

Il riferimento dettagliato è il [report V26 multi-modello EPIC](../user/2026-10-06_generalizzazione_multimodello_epic_v26_strategia_vincoli_e_audit.md), composto da 22 sezioni e quattro appendici. Il suo nome conserva la data di completamento del 6 ottobre. Il [daily parziale del 6 ottobre](2026-10-06_daily-report.md) conserva la successiva integrazione documentale e la propagazione dei TODO; la chiusura richiesta per questa giornata è il presente file.

L'[ultimo daily precedente, del 1° ottobre](2026-10-01_daily-report.md), comprende anche un'integrazione del 5 ottobre sugli esperimenti 39D. I relativi risultati sono precedenti e non sono stati rieseguiti nella nuova attività. Tutte le voci aperte di quel daily sono conservate nel backlog sotto, senza dichiararle concluse per effetto del cambio di priorità.

## Problema, nuova priorità e decisioni

L'utente giudica ancora insufficienti le traiettorie cinematiche ottenute con l'oscuramento degli ingressi della `unified_MLP_39`. Il ramo viene **temporaneamente messo da parte**, conservando risultati e checkpoint. La nuova priorità è generalizzare la catena **V26 imitativa → student deployable → PPO ex-novo** oltre il modello EPIC AB06.

Il parent imitativo V26 con 39 feature, comprese quattro informazioni healthy privilegiate, è distinto dalla `unified_MLP_39` sospesa. L'actor deployable storico V26C è 35D. Il nome V26 non seleziona da solo uno dei checkpoint disponibili.

Scelte e vincoli da conservare alla ripresa:

- Il corpus previsto è EPIC treadmill. L'inventario remoto e i soggetti realmente eleggibili restano da determinare; i 22 soggetti descritti dalla fonte non equivalgono a 22 bundle completi già verificati.
- Prima istanza sui tre soggetti locali **AB06/AB07/AB08**, condivisa dall'utente come direzione di lavoro.
- **Attendere il via esplicito prima di partire:** nessun download, conversione, build di nuovi modelli, rollout o training della generalizzazione è autorizzato dalla sola discussione o dalla chiusura della giornata.
- Split **70/30 oppure 80/20 per soggetto/modello**; rapporto, assegnazioni, seed e validazione non ancora scelti. Trial e velocità dello stesso soggetto restano nello stesso insieme. AB06 resta nello sviluppo; un soggetto pilot usato per scegliere la ricetta non può poi essere dichiarato test finale sigillato.
- La rete finale non riceve **descrittori anatomici del paziente né subject ID**. Il simulatore usa comunque i parametri fisici del modello; manifest e metriche possono identificare i soggetti. Teacher/critic privilegiati devono restare separati dagli ingressi dell'actor.
- Conservare inizialmente il **morphology corridor attuale**. Diagnostica della copertura per i nuovi modelli e futuro adattamento restano attività distinte; non aggiustare il corridor sulla base del test per poi dichiararlo non visto.
- Nessuna **domain randomization sintetica aggiuntiva nella prima fase**. Storia causale di osservazioni/azioni o ulteriori architetture sono possibili estensioni future, non scelte già approvate.
- L'utente propone un **rollout zero-shot della rete attuale su un modello diverso da AB06 prima del training**. Il test è utile, ma non è stato eseguito; checkpoint, modello, finestre, preprocessing e metriche restano da fissare.

## Strategia discussa per le tre fasi V26

La parte imitativa parte dai pesi V26 esistenti e propone una continuazione con episodi distribuiti su più ambienti fisici. AB06 continua a essere campionato insieme ai nuovi soggetti; tutti gli episodi aggiornano lo stesso insieme di pesi. Il file `.osim` seleziona il plant, mentre target, eventi e riferimenti devono essere corretti per il trial del soggetto. L'imitazione nativa è PPO con reward imitativa: non si trasforma automaticamente in una regressione offline. Budget, optimizer, learning rate, resume e sampler non sono stati selezionati.

Il teacher multi-modello deve poi alimentare uno **student comune causale**, seguendo la metodologia V26C di refit, raccolta sul runtime target e DAgger. Lo student viene eseguito, gli stati effettivamente visitati vengono registrati e un oracolo qualificato produce le label sullo stesso stato/evento. I dati nominali e recovery devono essere bilanciati per soggetto e condizione; la loss offline non sostituisce la qualifica closed-loop.

Il transplant diretto storico 39→35, senza il passaggio di refit appropriato, collassava già prima del PPO. La continuazione imitativa multi-modello non elimina da sola questa dipendenza dagli ingressi privilegiati: refit/DAgger e qualifica deployable restano passaggi da verificare.

La storia V26C richiede due precisazioni conservate nel report: J1 usava azioni prescribed dalla traiettoria base, mentre il parent neurale inizializzava J2; il dato 3,34% di righe recovery / 84,81% di errore quadratico iniziale indicava una possibile influenza sull'update, non una quota del gradiente direttamente misurata. Gli script J storici restano legati ad AB06 e devono essere parametrizzati.

Il PPO ex-novo deve partire da uno student deployable qualificato e mantenere gli stessi domini di training. Critic warm-up, restore e stato optimizer devono essere verificati sulla distribuzione multi-modello, distinguendo actor invariato da calibrazione del valore. Un pilot breve precede la campagna lunga; nessun budget è ancora concordato.

La domain generalization sarà dimostrata soltanto sui soggetti esclusi dallo sviluppo di teacher, student/DAgger, PPO e selezione. Valutare i soggetti usati nel training dimostra capacità multi-modello; aggiornare la policy sul soggetto nuovo è un esperimento separato di adattamento.

## Audit della pipeline EPIC ABx

La pipeline è già presente e documentata nei report [AB06 dell'11 maggio](../user/2026-05-11_ab06_seasea_epic_pipeline.md), [daily del 12 maggio](2026-05-12_daily-report.md) e [ABx del 13 maggio](../user/2026-05-13_pipeline_unica_abx_seasea.md). Gli entrypoint sono [builder ABx](../../tools/build_abx_seasea_pipeline.py) e [runner OpenSim](../../scripts/run_opensim_sea_pipeline.py).

Il discovery locale trova 24 trial treadmill con componenti associati per nome: **8 AB06, 9 AB07, 7 AB08**, nelle sessioni rispettive `10_09_18`, `10_14_18`, `10_21_2018`. Sono presenti modelli sani, marker, force plates, IK/ID, condizioni ed eventi dei due lati. Non è stata verificata integralmente la struttura numerica dei MAT. Il bundle operativo presente è AB06; la disponibilità raw AB07/08 non equivale a nuovi bundle protesici completi qualificati.

Limiti della pipeline da risolvere o qualificare prima dei rollout:

- **Donor:** sorgente dei segmenti, giunti e componenti di forza della protesi. Il default raw AB06 differisce dal modello PI-tuned della baseline; vanno fissati XML e parametri effettivi dopo gli override runtime.
- **Fitting:** il graft cambia massa/inerzie ma conserva geometria dei segmenti e giunti copiati dal donor, eccetto l'anca del soggetto. Non adatta automaticamente lunghezze, COM e posa ai nuovi soggetti.
- **Convenzione di massa:** `transfemur` riceve la massa originale di `femur_l`; la massa restante è ripartita fra i segmenti distali secondo proporzioni donor. È una convenzione del benchmark, non l'identificazione di una protesi reale.
- **Scaling:** il richiamo corrente salta lo scaling biologico quando il MarkerSet è il modello `.osim`. Il modello EPIC può essere già personalizzato; ciò non qualifica il fitting protesico.
- **IK:** il builder passa `--dry-run` al runner e su una build nuova ripiega sulla IK EPIC convertita/rimappata. La preparazione dei file non dimostra una nuova IK eseguita; il percorso effettivo va scelto e qualificato.
- **Multi-trial:** oggi è selezionato un trial per task; `--all-subjects` riguarda i soggetti locali e non tutti i trial o il corpus remoto. Occorrono destinazioni indipendenti e provenance per evitare sovrascritture.
- **Status e RRA:** `complete` e smoke di 0,03 s non qualificano una falcata. RRA full-span AB06 resta una diagnostica storica non promossa a modello operativo; alcuni artefatti numerici di maggio oggi sono assenti.

Prima della policy vanno qualificati caricamento/plugin, coordinate LEFT, marker/fitting, sincronismo modello–IK–GRF, posa e contatto, massa/inerzie, reset e stato SEA, detector e tracking prescribed con reserve. Non sono ancora fissate tutte le soglie della campagna.

## Dipendenza BW e primo zero-shot

Il codice attuale calcola `BW = massa_totale_modello_OpenSim * 9.80665`; il carico actor è forza normale / BW. Il peso non è una colonna esplicita, ma determina il preprocessing del carico e gli ingressi dell'adattatore eventi/FSM. Il vincolo sui dati del paziente richiede quindi un audit dell'intera catena di inferenza, non solo delle feature della rete.

Un rollout congelato con BW storico misura il trasferimento nel contratto originale del simulatore, con massa nota; non dimostra ancora inferenza senza caratteristiche del paziente. Cambiare scaler prima del rollout modifica invece gli ingressi e può confondere la migrazione con il nuovo modello. Sostituzione e ordine dei confronti restano aperti.

Il protocollo proposto distingue controlli prescribed, teacher imitativo con riferimenti del nuovo soggetto e policy deployable/ex-novo congelata, affiancati dal riferimento AB06. Un teacher che legge target privilegiati non dimostra autonomia. Vanno separate qualità del passo, raw/served/actual, cicli, contatto e penetrazione, SEA/saturazione, reserve e terminazioni; il solo completamento non basta.

## Evidenze storiche 39D e debiti conservati

Nel confronto CP90 imitazione/ex-novo con dieci update actor, entrambi i rami oscurati falliscono il gate di escursione/cicli e il curriculum si arresta al primo livello. La continuazione CP90→190 a quattro blocchi di 25 update è un esperimento diverso. Completamento temporale e basso RMSE actual–served non hanno garantito la forma del passo. Questi risultati motivano il cambio di priorità e non vengono attribuiti alla nuova campagna V26.

Best90 fresh rimane il riferimento storico scelto per il ramo 39D sospeso. Nessun teacher, export J19A/J20 o pilot ex-novo V26 viene promosso automaticamente a baseline della nuova fase.

Il bug bootstrap PPO di Ray/RLlib 2.55.1 resta **esplicitamente rinviato dall'utente**. Restano aperti critic/KL, probe e dimensioni dell'osservazione, reset, profili GRF/detector vincolati ad AB06 e verifica live Windows. I TODO della 39D sono sospesi con il ramo, non chiusi. Le vecchie proposte di descriptor anatomici nell'actor sono superate; corridor multi-soggetto e randomizzazione sono rinviati rispetto alla fase iniziale attuale.

## Lavoro svolto, verifiche e file

Il 5 ottobre sono stati recuperati contesto e report, letti pipeline e runtime, controllati inventario e checkpoint candidati, confrontati i donor XML e consultate fonti primarie EPIC/DAgger/domain generalization. Il comando read-only `python3 tools/build_abx_seasea_pipeline.py --all-subjects --validate-only` passa il discovery dei tre soggetti e non esegue OpenSim. Il Python di sistema 3.12 usato non importava OpenSim; non è una prova dell'assenza di un altro ambiente operativo.

Il 6 ottobre sono stati completati il report e la propagazione documentale, ricontrollati preprocessing BW e sei hash sorgenti/modelli, verificati sette percorsi candidati, e svolte revisioni indipendenti della pipeline e della lineage V26. Nessuno dei candidati viene selezionato automaticamente come nuova baseline. I cinque completamenti C01–C05 e i 29 TODO G01–G29 sono identici al report; le 231 voci aperte univoche precedenti sono conservate.

Verifica documentale della chiusura, eseguita il 6 ottobre: PASS su tutti i collegamenti locali dei tre documenti, corrispondenza esatta delle voci C/G e conservazione delle 260 voci aperte nei due daily (29 nuove e 231 ereditate). Questo controllo non è una nuova validazione OpenSim o della generalizzazione.

| Documento | Ruolo e data effettiva |
|---|---|
| `reports/user/2026-10-06_generalizzazione_multimodello_epic_v26_strategia_vincoli_e_audit.md` | Report dettagliato completato il 6 ottobre, su discussione/audit del 5 |
| `reports/daily/2026-10-06_daily-report.md` | Integrazione documentale del 6 ottobre, ora collegata alla chiusura richiesta per il 5 |
| `reports/daily/2026-10-05_daily-report.md` | Questo daily finale `end_day`, attribuito al 5 ottobre su indicazione utente |

Nessun codice, modello, config o checkpoint è stato modificato da questa attività di documentazione/chiusura. Nessun download, conversione, build, rollout o training della nuova fase è stato avviato. Non sono stati prodotti nuovi plot; i sette plot canonici dei futuri rollout restano sotto `plot/<rollout>/` nel root. Rete/policy/training rimangono in `Trajectory Generator/`; architettura SEA e compatibilità macOS arm64/Windows x86 restano invarianti.

## Ripartenza successiva

Il contesto è consolidato e la giornata viene chiusa senza lanci in background della nuova fase. Alla ripresa leggere prima decisioni e TODO G01–G29 del report, poi concretizzare baseline/donor/preprocessing e il pilot sui tre soggetti **dopo il via dell'utente**. Il via a uno specifico rollout non amplia automaticamente lo scope a tutto il corpus o a nuove campagne PPO. La roadmap completa rimane: protocollo → pilot fisico → zero-shot → corpus/split → teacher multi-modello → student/DAgger → critic e pilot ex-novo → test per soggetti esclusi.

## TODO del nuovo report, propagati con gli stessi ID


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

## Backlog precedente conservato dal 1° ottobre

Le voci seguenti sono propagate dal daily precedente, rimuovendo soltanto duplicati testuali esatti. Non costituiscono nuove autorizzazioni né un riesame integrale. Gli incarichi del ramo 39D sono temporaneamente sospesi con quel ramo; le formulazioni sulla creazione della variante 39D o sulla prosecuzione V1 possono essere storiche e già superate. Le scelte attuali nel report prevalgono: descriptor anatomici nell'actor esclusi, corridor corrente inizialmente conservato, randomizzazione rinviata e via alle esecuzioni ancora atteso.

Il fix bootstrap PPO resta esplicitamente rinviato. Il TODO generale EPIC ora ha priorità di workstream, ma il suo completamento scientifico non è attestato dalla disponibilità dei tre soggetti locali. Qualifiche Windows, repliche e debiti tecnici non vengono chiusi dalla sola stesura documentale.

- [ ] Verificare una calibrazione effettiva del critic, con distribuzione
  degli errori, incidenza del clipping VF e validazione indipendente;
  il solo warm-up eseguito non dimostra adattamento del valore.

- [ ] Definire e verificare un adattamento actor sufficiente agli ingressi
  oscurati, fissando budget e criteri prima di ulteriori esecuzioni.

- [ ] Approfondire copertura, allineamento e attribuzione del costo morphology,
  distinguendo traiettoria servita, cinematica fisica e disponibilità dei
  campioni prima di giudicare il beneficio della reward.

- [ ] Verificare robustezza su condizioni e seed ulteriori e svolgere la
  verifica live Windows. Il fix bootstrap PPO resta il TODO rinviato già
  riportato sotto; nessuna nuova esecuzione è avviata da questa integrazione.

- [ ] Correggere in un’attività separata il bug bootstrap PPO di Ray/RLlib 2.55.1: bootstrap eliminato ai confini artificiali dei frammenti e propagazione errata prima di una vera terminazione. La correzione è stata esplicitamente rinviata dall’utente; non è applicata a questo pilot. Dettagli nell’[audit del 29 settembre](../user/2026-09-29_audit_RL_39d_bootstrap_e_aggiornamento_PPO.md).

- [ ] Approfondire quantitativamente l’effetto dell’oscuramento prima di adattare la progressione o qualificare un checkpoint per il successivo training ex-novo. I rollout CP112/CP184 e il confronto specifico CP90 con i rami imitazione/ex-novo a cinque e dieci update sono conclusi; resta distinto il confronto esteso con i checkpoint di fine blocco CP115/140/165/190.

- [ ] Verificare robustezza su condizioni e seed ulteriori; i tre start restano esclusi dal pilot corrente per scelta dell’utente.

- [ ] Eseguire la verifica live Windows della pipeline scelta.

- [ ] Dopo il pilot, analizzare CP90/115/140/165/190 con valutazioni visibili e
  oscurate e quote effettive delle transizioni; i plot canonici restano sotto `plot/`.

- [ ] Risolvere successivamente il bug bootstrap PPO e svolgere verifica live Windows.

- [ ] Confrontare best90 fresh con best80 su finestra comune: tracking per giunto/fase, contatto, cicli, impulso, area positiva ankle e regolarità knee.

- [ ] Valutare robustezza con perturbazioni comuni a policy congelate e repliche indipendenti di training.

- [ ] Eseguire verifica live Windows della pipeline scelta.

- [ ] Isolare in un nuovo esperimento la conservazione del coefficiente KL durante il critic-only, con controllo sottoposto allo stesso trattamento; definire configurazioni e budget prima dell'esecuzione. Il fix non è implementato né il training predisposto.

- [ ] Verificare la sufficienza del riadattamento del critic con diagnostica degli errori e validazione indipendente; un aggiornamento non dimostra convergenza.

- [ ] Sperimentare separatamente regolarità della coppia knee e cinematica, mantenendo tracking e contatto come vincoli e senza modificare contemporaneamente le famiglie.

- [ ] Eseguire la verifica live Windows della pipeline e, prima di conclusioni sulla robustezza, repliche di training e condizioni di valutazione più ampie.

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

- [ ] Qualificare il beneficio della variante scelta nel successivo ex-novo. Trasferimento tecnico, warm-up e resume sono verificati per il confronto CP90 documentato sopra; il recupero cinematico autonomo resta da dimostrare.

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

- [ ] Valutare robustezza best90/best80 con perturbazioni comuni a policy congelate, distinguendo seed evaluation e repliche training. — [fonte 2026-09-30, riga 114](../user/2026-09-30_training_fresh_100iter_rollout_best90.md).

- [ ] Confrontare best90 fresh e best80 sulla stessa finestra con tracking per giunto/fase, contatto, cicli, impulso/area positiva ankle e regolarità knee. — [fonte 2026-09-30, riga 113](../user/2026-09-30_training_fresh_100iter_rollout_best90.md).

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
