# Registro TODO del progetto

Registro canonico dei TODO aperti. Sostituisce la copia integrale del backlog
in ogni daily report (regola in `AGENTS.md`). Non va letto in blocco a
`start_day`: cercare per ID o argomento quando serve.

Regole:

- Ogni TODO ha un ID stabile in grassetto. I TODO nuovi usano un prefisso
  tematico (es. `STP-T01`); `BKL-###` identifica le voci ereditate senza ID.
- Un TODO nuovo da un report in `reports/user/` si aggiunge in fondo a
  «Aperti», con link al report di origine.
- Un TODO esaurito si sposta in «Chiusi» con data e link al report che lo
  chiude; non si cancella. Una voce superata si chiude allo stesso modo,
  indicandone il motivo.
- I testi ereditati sono quelli del daily
  [2026-10-13](daily/2026-10-13_daily-report.md), riportati senza modifiche a parte
  l'ID `BKL-###`. Alcuni citano materiali rimossi nella pulizia del
  14–20 ottobre (vedere `reports/cleanup/`).

Stato iniziale (2026-10-29): 278 voci aperte, cioè le 276
ereditate dal daily del 2026-10-13 più le 2 dei report del 2026-10-26.

## Aperti

### Dai report del 2026-10-26

- [ ] **STP-T01** — Definire e verificare il comportamento di arresto, stazione eretta e ripartenza: riconoscimento dell'intento di fermarsi, condizione di appoggio ed equilibrio verificata, stato standing nel contratto FSM actor (oggi `TIMEOUT` dopo 2,2 s di stance) e obiettivo di apprendimento nella reward; solo dopo introdurre episodi di transizione nel training. Aumentare il timeout o congelare la fase non basta. Fonti: [test](user/2026-10-26_test_camminata_fermo_ripartenza_best47.md), [diagnosi](user/2026-10-26_diagnosi_fermo_ripartenza_best47.md). Collegato a V26-T04.

- [ ] **SPD-T01** — Distinguere il ritardo del generatore dalla risposta del contatto nelle variazioni di velocità del treadmill, con plateau più lunghi e rampe più lente per qualificare separatamente rallentamento, regime veloce e recupero (la prova best47 termina per `grf_penetration` a 5,93 s durante l'accelerazione 0,8→1,2). Fonte: [test velocità](user/2026-10-26_test_variazioni_velocita_treadmill_best47.md).

### Ereditati dal daily del 2026-10-13

- [ ] **RPR-T01** — Conservare la causa fisica prioritaria nel wrapper reward V26 quando il contratto morphology fallisce dopo un segmento interrotto; qualificare separatamente la correzione di end_reason, senza cambiare reward, guardie o gli artefatti congelati di questa campagna.

- [ ] **RIK-T01** — Preregistrare ed eseguire un confronto PPO fra distribuzione reset attuale e ampliata, con stessi pesi iniziali, reward, learner, calibrazione iniziale del critic, budget e seed; valutare su start comuni held-out e distinguere copertura, accredito FSM e finestre biologiche/GRF. Non cambiare contemporaneamente learning rate o reward per attribuire l'eventuale beneficio ai reset.

- [ ] **RIK-T02** — Definire e qualificare un bootstrap iniziale causale e coerente di osservazioni/reference/governor/SEA/FSM, verificando in particolare qddot IK nell'actor a t0 e carico inizialmente zero; confrontare gli esiti con il contratto congelato senza riscrivere gli artefatti storici.

- [ ] AB07-P01 — Isolare l'interazione montaggio–mapping marker/IK–contatto–controllo che interrompe la variante con appoggio statico, distinguendo il confronto meccanico a cinematica comune dalla preparazione coerente di ogni modello; mantenere fissa la calibrazione del detector e non aumentare i limiti SEA come sostituto della diagnosi.

- [ ] AB07-P02 — Definire vincoli indipendenti e compatibili su dispositivo, allineamento, appoggio e piano di acquisizione per dimensionare la protesi con lo stesso criterio sui soggetti; verificare coerenza fra suola STL, sfere di forza e punti del detector prima di qualificare una lunghezza fisica.

- [ ] V26-T04 — Verificare la capacità della policy di avviare autonomamente il cammino da fermo, con posa statica e velocità iniziali nulle, distinguendo questa prova dai rollout inizializzati con postura e velocità di cammino ricavate dalla IK. Definire condizioni iniziali coerenti di contatto, SEA e governor, esplicitare il supporto fornito dal controllo biologico e misurare transizione al primo passo, stabilità e completamento dei cicli. Da propagare nei daily successivi fino alla verifica; attività futura, non eseguita con questa annotazione.

- [ ] V26-T01 — Correggere e validare separatamente la guardia dei confini campionati del detector binario esposta dall'ultimo callback AB06, mantenendo il rifiuto dei batch realmente non validi e verificando casi di rounding/horizon su macOS e Windows; conservare receipt e trace failed del test originale.

- [ ] V26-T02 — Predisporre ed eseguire, quando richiesto, un confronto learned AB06/AB07 a checkpoint e contratto policy congelati, stessi criteri HS/startup/tre cicli nominali e durate comuni; la coppia IK-driven corrente non è quel test della rete.

- [ ] V26-T03 — Estendere la qualifica del trasferimento con start/condizioni e repliche dichiarati, misurando separatamente carico, HS/TO, knee clipping, margine di penetrazione, tracking, MTP e reserve; preservare best47 e tenere distinta la qualifica dai futuri training.

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

- [ ] G11 — Auditare tutti gli ingressi e il preprocessing dell'actor per disponibilità dalla protesi, inclusi BW, eventi, clock, soglie e reset; scegliere e verificare l'eventuale migrazione senza metadati del paziente.

- [ ] G12 — Fissare il protocollo zero-shot iniziale: modello, checkpoint congelati, preprocessing, reference, start/eventi, finestre, modalità della policy e metriche.

- [ ] G13 — Dopo via e gate fisici, eseguire i rollout zero-shot e i controlli concordati, con sette plot canonici per rollout sotto `plot/` e analisi delle cause degli esiti.

- [ ] G14 — Inventariare il corpus EPIC treadmill remoto, scaricare dopo via i componenti necessari e determinare soggetti/condizioni realmente eleggibili.

- [ ] G15 — Scegliere 70/30 oppure 80/20, definire validazione, assegnazioni/seed e regole contro leakage; conservare AB06 nello sviluppo e classificare i soggetti pilot come sviluppo se usati per le scelte.

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

- [ ] G26 — Mantenere rinviato il fix bootstrap PPO; definirne in attività separata ambito e confronto prima di modificarlo o attribuirgli gli esiti della campagna.

- [ ] G27 — Riprendere l'adattamento del morphology corridor solo dopo la diagnostica iniziale e con dati/protocollo ammessi dallo split.

- [ ] G28 — Valutare in una fase successiva l'utilità di una storia causale di osservazioni/comandi, senza introdurre descrittori anatomici espliciti o identità del paziente.

- [ ] G29 — Considerare domain randomization soltanto in una fase successiva, con range fisicamente coerenti e protocollo separato.

- [ ] AB07-Q01 — Qualificare geometria del contatto, piano e riferimenti dei sensori su AB07, mantenendo comuni i parametri SEA, la control chain, le leggi FSM e le soglie degli eventi; rendere esplicito il binding per modello senza aggirare le guardie path/hash.

- [ ] AB07-Q02 — Dopo la correzione geometrica, rieseguire il diagnostico prescribed V26 con criteri dichiarati e verificare carico, HS/TO, durata stance, cicli e reserve; il completamento temporale non basta per qualificare AB07 prima della policy.

- [ ] AB07-Q03 — Investigare il limite biologico comune di tracking MTP, distinguendo osservabilità della IK e dinamica/SO, senza introdurre guadagni specifici per AB07.

- [ ] AB07-Q04 — Eseguire una verifica live della nuova pipeline AB07 su Windows con gli stessi parametri comuni e registrare gli esiti; le prove macOS e quelle Windows storiche AB06 non sostituiscono questa verifica.

- [ ] **BKL-001** — Verificare una calibrazione effettiva del critic, con distribuzione

- [ ] **BKL-002** — Definire e verificare un adattamento actor sufficiente agli ingressi

- [ ] **BKL-003** — Approfondire copertura, allineamento e attribuzione del costo morphology,

- [ ] **BKL-004** — Verificare robustezza su condizioni e seed ulteriori e svolgere la

- [ ] **BKL-005** — Correggere in un’attività separata il bug bootstrap PPO di Ray/RLlib 2.55.1: bootstrap eliminato ai confini artificiali dei frammenti e propagazione errata prima di una vera terminazione. La correzione è stata esplicitamente rinviata dall’utente; non è applicata a questo pilot. Dettagli nell’[audit del 29 settembre](user/2026-09-29_audit_RL_39d_bootstrap_e_aggiornamento_PPO.md).

- [ ] **BKL-006** — Approfondire quantitativamente l’effetto dell’oscuramento prima di adattare la progressione o qualificare un checkpoint per il successivo training ex-novo. I rollout CP112/CP184 e il confronto specifico CP90 con i rami imitazione/ex-novo a cinque e dieci update sono conclusi; resta distinto il confronto esteso con i checkpoint di fine blocco CP115/140/165/190.

- [ ] **BKL-007** — Verificare robustezza su condizioni e seed ulteriori; i tre start restano esclusi dal pilot corrente per scelta dell’utente.

- [ ] **BKL-008** — Eseguire la verifica live Windows della pipeline scelta.

- [ ] **BKL-009** — Dopo il pilot, analizzare CP90/115/140/165/190 con valutazioni visibili e

- [ ] **BKL-010** — Risolvere successivamente il bug bootstrap PPO e svolgere verifica live Windows.

- [ ] **BKL-011** — Confrontare best90 fresh con best80 su finestra comune: tracking per giunto/fase, contatto, cicli, impulso, area positiva ankle e regolarità knee.

- [ ] **BKL-012** — Valutare robustezza con perturbazioni comuni a policy congelate e repliche indipendenti di training.

- [ ] **BKL-013** — Eseguire verifica live Windows della pipeline scelta.

- [ ] **BKL-014** — Isolare in un nuovo esperimento la conservazione del coefficiente KL durante il critic-only, con controllo sottoposto allo stesso trattamento; definire configurazioni e budget prima dell'esecuzione. Il fix non è implementato né il training predisposto.

- [ ] **BKL-015** — Verificare la sufficienza del riadattamento del critic con diagnostica degli errori e validazione indipendente; un aggiornamento non dimostra convergenza.

- [ ] **BKL-016** — Sperimentare separatamente regolarità della coppia knee e cinematica, mantenendo tracking e contatto come vincoli e senza modificare contemporaneamente le famiglie.

- [ ] **BKL-017** — Eseguire la verifica live Windows della pipeline e, prima di conclusioni sulla robustezza, repliche di training e condizioni di valutazione più ampie.

- [ ] **BKL-018** — Dopo l'ankle, sperimentare separatamente regolarità knee e cinematica

- [ ] **BKL-019** — Eseguire verifica live Windows e, se una candidata è promettente,

- [ ] **BKL-020** — Riconciliare la collocazione delle figure diagnostiche reward LEFT e

- [ ] **BKL-021** — Eseguire lo screening con perturbazioni comuni a policy congelata di

- [ ] **BKL-022** — Eseguire il ramo controllo LEFT72 peso 1, con lo stesso fix hard,

- [ ] **BKL-023** — Distinguere il miglioramento in continuazione dalle cause del divario

- [ ] **BKL-024** — Se si interviene sulle coppie, progettare un confronto dal medesimo

- [ ] **BKL-025** — Fissare criteri distinti per completamento, cicli validi, forma delle

- [ ] **BKL-026** — Quantificare separatamente il trascinamento degli ultimi angoli nel

- [ ] **BKL-027** — Eseguire la verifica live Windows del fix hard, della ripresa e della

- [ ] **BKL-028** — Pianificare repliche di training e condizioni di valutazione sufficienti

- [ ] **BKL-029** — Se si considera RIGHT last60 per riuso/oscuramento, valutarla sugli

- [ ] **BKL-030** — Approfondire il transitorio seed 125 confrontando best35, last40, best58,

- [ ] **BKL-031** — Isolare su RIGHT l'origine del momento GRF invertito nell'ultimo

- [ ] **BKL-032** — Proseguire la stessa V1 da 50 a 100 iterazioni totali e svolgere la valutazione finale confrontando i risultati a 5, 50 e 100.

- [ ] **BKL-033** — Definire e verificare schema 39, maschera persistente e indipendenza ex-novo dalle copie dei target; fissare curriculum, budget, seed e criteri prima del confronto A/B/C.

- [ ] **BKL-034** — Eseguire diagnostica dell'osservabilità, confronto A/B/C completamente mascherato in valutazione e test di trasferimento ex-novo B/C, distinguendo avvio, regime e variabilità tra seed.

- [ ] **BKL-035** — Risolvere e verificare la sincronizzazione senza clock prescribed, distinguendo avvio e regime ai tre start.

- [ ] **BKL-036** — Costruire dati diagnostici temporalmente allineati e valutare osservabilità delle 35 feature su episodi indipendenti.

- [ ] **BKL-037** — Scegliere fra curriculum con target mascherati, avvio supervisionato e possibili soluzioni causali di fase/memoria sulla base di evidenze.

- [ ] **BKL-038** — Se si procede con la maschera, definire il nuovo schema, la normalizzazione, l'eventuale indicatore di disponibilità e lo stato persistente del curriculum, conservando la V1.

- [ ] **BKL-039** — Implementare e verificare isolamento dei target, coerenza raccolta/PPO e indipendenza degli aggiornamenti ex-novo dalle copie dei target.

- [ ] **BKL-040** — Definire budget, seed, progressione e criteri di accettazione prima del confronto A/B/C; eseguire poi il pilot e le repliche necessarie.

- [ ] **BKL-041** — Validare i candidati completamente mascherati ai tre start, con metriche separate di avvio, tracking, cicli validi e protezioni.

- [ ] **BKL-042** — Qualificare il beneficio della variante scelta nel successivo ex-novo. Trasferimento tecnico, warm-up e resume sono verificati per il confronto CP90 documentato sopra; il recupero cinematico autonomo resta da dimostrare.

- [ ] **BKL-043** — Misurare in un nuovo run imitativo l'effetto del peso di penetrazione 1.0, senza attribuirgli gli esiti dello smoke storico a 0.05.

- [ ] **BKL-044** — Completare il benchmark e il verdetto di parità qualitativa secondo il report del 6 settembre, includendo costo di apprendimento e variabilità tra seed.

- [ ] **BKL-045** — Eseguire le verifiche live Windows della pipeline scelta; riprendere predittività e compensazione del ritardo soltanto in una versione successiva.

- [ ] **BKL-046** — Fissare checkpoint baseline, manifest degli input e protocollo comune

- [ ] **BKL-047** — Implementare il valutatore comune sotto `Trajectory Generator/unified_MLP/`,

- [ ] **BKL-048** — Completare e riprodurre il benchmark baseline deterministico sui tre start.

- [ ] **BKL-049** — Definire prima della selezione finale margini di regressione, condizioni

- [ ] **BKL-050** — Eseguire un vero pretraining e training ex-novo della rete unificata,

- [ ] **BKL-051** — Eseguire il confronto deterministico e stocastico finale, includendo

- [ ] **BKL-052** — Produrre il verdetto quantitativo di parità, regressione o risultato

- [ ] **BKL-053** — Eseguire le verifiche live su Windows per attestare anche quella piattaforma.

- [ ] **BKL-054** — Riprendere actor predittivo e compensazione del ritardo in una versione

- [ ] **BKL-055** — Implementare il teacher inverse-governor e il generatore delle label raw.

- [ ] **BKL-056** — Validare frozen-policy gli orizzonti attorno a `90 ms` knee e `100 ms`

- [ ] **BKL-057** — Eseguire DAgger closed-loop sullo stesso actor 35D con nominali e

- [ ] **BKL-058** — Definire e preregistrare il primo rapporto morphology knee/ankle

- [ ] **BKL-059** — Ripetere il critic-only rewarm quando la reward diventerà realmente

- [ ] **BKL-060** — Valutare una fascia soft interna e una corretta assegnazione causale del

- [ ] **BKL-061** — Progettare e validare un corridor cadence-conditioned su dati held-out.

- [ ] **BKL-062** — Preregistrare ed eseguire un pilot PPO separato di `5-10` iterazioni con

- [ ] **BKL-063** — Decidere un'eventuale estensione oltre 10 iterazioni soltanto dopo i

- [ ] **BKL-064** — Monitorare penetrazione `20/25/28 mm`, served-vs-actual e contatori FSM

- [ ] **BKL-065** — Rivalutare sigma dopo l'actor predittivo o una modifica della logica

- [ ] **BKL-066** — Generalizzazione oltre AB06 tramite dataset EPIC e soggetto held-out,

- [ ] **BKL-067** — LOTO, LOCO, B1R1 e B1R2 restano TODO futuri soggetti ad approvazione

- [ ] **BKL-068** — Conservare sidecar J8 e artefatti storici senza modifiche retroattive.

- [ ] **BKL-069** — Riallineare in manutenzione separata le assertion legacy soggette a drift

- [ ] **BKL-070** — Completare V1 fino a100 e confrontare5/50/100.

- [ ] **BKL-071** — Creare la variante39 dopo valutazione100; superare isolamento e smoke.

- [ ] **BKL-072** — Completare A/B/C, trasferimento B/C ed eventuali repliche preregistrate.

- [ ] **BKL-073** — Rendicontare tutti gli esiti e costi, distinguendo fattibilità e parità.

- [ ] **BKL-074** — Eseguire la verifica live Windows separata.

- [ ] **BKL-075** — Proseguire la stessa V1 da 50 a 100 iterazioni totali e svolgere la valutazione finale 5/50/100.

- [ ] **BKL-076** — Svolgere la fase 5: variante 39, isolamento, smoke, confronto A/B/C e trasferimento ex-novo B/C secondo il protocollo.

- [ ] **BKL-077** — Completare la diagnostica dell'osservabilità e verificare la sincronizzazione senza clock prescribed, distinguendo avvio e regime.

- [ ] **BKL-078** — Completare il benchmark di parità con la baseline, includendo costi e variabilità tra seed.

- [ ] **BKL-079** — Eseguire le verifiche live Windows; mantenere aperti i debiti storici non risolti.

- [ ] **BKL-080** — Integrare in un esperimento separato la correzione generale dei confini

- [ ] **BKL-081** — Quantificare separatamente la normalizzazione delle advantages sulle

- [ ] **BKL-082** — Confrontare training RL standard e corretto con stessa inizializzazione,

- [ ] **BKL-083** — Proseguire la diagnosi di esplorazione, clipping, nominale/stocastico e

- [ ] **BKL-084** — Verificare altri start e tratti disponibili di AB06, recupero dopo

- [ ] **BKL-085** — Completare la verifica live Windows; verificare il trasferimento ad

- [ ] **BKL-086** — Completare equivalenza host-target, latenza worst-case, HIL e contratto hardware prima di qualsiasi claim di deployment. — [fonte 2026-08-09, riga 342](user/2026-08-09_h0_v12r2_training_readiness.md).

- [ ] **BKL-087** — Validare i candidati futuri su multistart, seed held-out, recovery, soggetti/profili esterni, rumore e delay. — [fonte 2026-08-09, riga 340](user/2026-08-09_h0_v12r2_training_readiness.md).

- [ ] **BKL-088** — Conservare come TODO storico l'inizializzazione della deflessione SEA coerente con la coppia richiesta. — [fonte 2026-08-09, riga 344](user/2026-08-09_h0_v12r2_training_readiness.md).

- [ ] **BKL-089** — Compilare/distribuire e verificare su Windows x86_64 le DLL SEA `_ff` e OnlineGRF corrette, con provenienza e ABI OpenSim/Simbody/Python; claim numerico separato dalla prova macOS. — [fonte 2026-08-09, riga 337](user/2026-08-09_h0_v12r2_training_readiness.md).

- [ ] **BKL-090** — Rieseguire (opzionale, read-only) l'eval offline sul `corpus.npz` + modulo salvati in `v12r3/…/fit/p2/` per stabilire quale metrica bocciò il `fit_p2` e di quanto: unico esito della sessione senza numeri persistiti. — [fonte 2026-08-15, riga 181](user/2026-08-15_retrospettiva_forense_sessione_09-10_agosto_v12r2_v12r4.md).

- [ ] **BKL-091** — Riconciliare il pin V20/V26 in train_ppo_mlp.py. — [fonte 2026-08-16, riga 192](user/2026-08-16_v12r12_fix_pre_freeze_applicati_e_verificati.md).

- [ ] **BKL-092** — Chiarire l’anomalia teacher retry best==last==warm_start. — [fonte 2026-08-16, riga 192](user/2026-08-16_v12r12_fix_pre_freeze_applicati_e_verificati.md).

- [ ] **BKL-093** — Valutare un override esplicito dello schema in rollout_eval.py per il checkpoint39/43. — [fonte 2026-08-18, riga 149](user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md).

- [ ] **BKL-094** — Promuovere il driver di qualifica da scratchpad a script versionato alla prossima esecuzione della catena. — [fonte 2026-08-20, riga 95](user/2026-08-20_risultato_training_ready_exnovo_B0820.md).

- [ ] **BKL-095** — Follow-up v3 fuori scope: riportare lo swing timeout da 2,6 a 1,3 s. — [fonte 2026-08-21, riga 95](user/2026-08-21_fsm_v3_resync_validazione_e_bug_tolleranza_corridoio.md).

- [ ] **BKL-096** — Qualificare generalizzazione oltre AB06 su almeno un soggetto/modello EPIC completamente held-out da teacher, DAgger/PPO, iperparametri e corridor; più trial dello stesso soggetto non bastano. — [fonte 2026-08-22, riga 15](user/2026-08-22_todo_generalizzazione_multimodello_epic.md).

- [ ] **BKL-097** — **Contratto FSM phase-aligned:** correggere o rendere univoca la semantica di `cycle_valid` dopo una cancellazione HS; fino ad allora i prodotti phase-aligned restano fail-closed. — [fonte 2026-08-23, riga 109](user/2026-08-23_f1_s1_protocollo_tooling_dryrun.md).

- [ ] **BKL-098** — **Osservabilità RLlib:** nel prossimo eventuale run registrare environment configurati/sani, batch corrente/lifetime e generation/replacement dei runner in campi distinti. — [fonte 2026-08-23, riga 109](user/2026-08-23_f1_s1_protocollo_tooling_dryrun.md).

- [ ] **BKL-099** — Chiarire perché B3/target_phase vale zero sotto runtime v 3 anche con cicli validi. — [fonte 2026-08-24, riga 212](user/2026-08-24_v26b_audit_luglio_r1r2_e_progetto_rev4d.md).

- [ ] **BKL-100** — Decidere se archiviare il bundle J10 rifiutato, preservandolo byte-identico come evidenza. — [fonte 2026-08-27, riga 243](user/2026-08-27_v26c_j10r1_multistart_teacher_collection.md).

- [ ] **BKL-101** — Decidere documentalmente sui due gcs_server orfani segnalati il 18 agosto; verificare stato e necessità prima di eventuali interventi. — [fonte 2026-08-27, riga 273](user/2026-08-27_v26c_j20_restore_audit_r2_preparazione.md).

- [ ] **BKL-102** — Mantenere esplicita l’assenza storica di commit_verification in J2/J8 e di actor_digest in J2; evitare attestazioni retroattive inventate. — [fonte 2026-08-27, riga 233](user/2026-08-27_v26c_j17_architect_audit_addendum.md).

- [ ] **BKL-103** — Consolidare additivamente la correzione rad/secondi nei record V26C congelati, senza modificarne gli artefatti numerici. — [fonte 2026-08-27, riga 353](user/2026-08-27_qualifica_closed_loop_j9r1.md).

- [ ] **BKL-104** — Chiarire/correggere in manutenzione separata policy_std=null nelle vecchie telemetrie J12–J19, senza inventare valori o riscrivere evidenza congelata. — [fonte 2026-08-27, riga 219](user/2026-08-27_v26c_j16_j15r1_closed_loop_qualification_readiness.md).

- [ ] **BKL-105** — Chiarire path di staging e valid_cycle_count dei receipt J9R1 e i falsi heel strike del canale legacy 20 N; preservare i record e spiegare che non alterarono FSM/reward/verdetto. — [fonte 2026-08-27, riga 356](user/2026-08-27_qualifica_closed_loop_j9r1.md).

- [ ] **BKL-106** — Documentare la provenance mancante dei runner nei vecchi leaf e i due runner J15/J15R1 verso la stessa leaf; i pin introdotti in J18 non colmano i precedenti. — [fonte 2026-08-27, riga 231](user/2026-08-27_v26c_j17_architect_audit_addendum.md).

- [ ] **BKL-107** — Conservare il limite di riproducibilità J11: artefatti senza versione Torch/argomenti Adam sufficienti; il J15R1 riproducibile non colma retroattivamente J11. — [fonte 2026-08-27, riga 232](user/2026-08-27_v26c_j17_architect_audit_addendum.md).

- [ ] **BKL-108** — Mantenere espliciti nei riepiloghi DIAGNOSTIC_NOT_BINDING contraddittorio e assenza di post-commit storico J9R1; le correzioni dei successori non riscrivono quel leaf. — [fonte 2026-08-27, riga 341](user/2026-08-27_v26c_j12_closed_loop_qualification_readiness.md).

- [ ] **BKL-109** — Conservare e spiegare additivamente le discrepanze July su label raw/mean, gate ok:false e numeri di riga, senza riscrivere J0–J10R1. — [fonte 2026-08-27, riga 337](user/2026-08-27_v26c_j11_multistart_fit_execution.md).

- [ ] **BKL-110** — Decidere e verificare la correzione del campo nominal_mean_shift dichiarato ma non calcolato dal runner J15R1; misura esterna conclusa, debito di codice distinto. — [fonte 2026-08-27, riga 234](user/2026-08-27_v26c_j15r1_fresh_refit_execution.md).

- [ ] **BKL-111** — Se si vuole attribuire causalmente la deviazione pre-breccia, svolgere un confronto appaiato; una sola trace non isola i fattori confondenti. — [fonte 2026-08-27, riga 235](user/2026-08-27_v26c_j17_architect_audit_addendum.md).

- [ ] **BKL-112** — Indagare il clock swing non azzerato su hs_bounce_cancelled, distinguendolo dalla semantica cycle_valid; nessun fix è attestato da J19/R3/J21. — [fonte 2026-08-27, riga 429](user/2026-08-27_v26c_j13_j12_failure_diagnosis_and_recovery_plan.md).

- [ ] **BKL-113** — Separare o reinterpretare best_validation_mse contaminato dalla ripetizione dei blocchi nei fit storici; non usarlo come validazione indipendente. — [fonte 2026-08-27, riga 234](user/2026-08-27_v26c_j17_architect_audit_addendum.md).

- [ ] **BKL-114** — Decidere come spiegare/gestire il marker sorgente RESTORE_AUDIT_PENDING immutabile dopo la chiusura esterna G9, senza mutare retroattivamente il checkpoint. — [fonte 2026-08-28, riga 250](user/2026-08-28_v26c_j20_restore_audit_r3_esecuzione.md).

- [ ] **BKL-115** — Valutare le tre osservazioni non vincolanti R3 rev 7 (conversione downcast/None, mismatch raw R2, tipi optimizer registrati), separandole dai gate 13/13 passati. — [fonte 2026-08-28, riga 299](user/2026-08-28_v26c_j20_restore_audit_r3_correzione_rev7_gate_learning_rate.md).

- [ ] **BKL-116** — Progettare e validare corridor cadence-conditioned su dati held-out. — [fonte 2026-08-30, riga 346](user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).

- [ ] **BKL-117** — Valutare fascia soft interna e assegnazione causale del credito senza spostare il corridor finale. — [fonte 2026-08-30, riga 360](user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).

- [ ] **BKL-118** — Verificare uno sweep 0–180 ms di anticipo della sola loss e il rapporto con il ritardo phase-normalized 135–160 ms; distinto dalla decomposizione già conclusa. — [fonte 2026-08-30, riga 376](user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).

- [ ] **BKL-119** — Preregistrare il primo rapporto morphology knee/ankle asimmetrico. — [fonte 2026-08-31, riga 606](user/2026-08-31_progetto_rete_unificata_imitativa_exnovo_actor_predittivo.md).

- [ ] **BKL-120** — Validare frozen-policy gli orizzonti circa 90 ms knee / 100 ms ankle su più cadenze e stati del governor. — [fonte 2026-08-31, riga 603](user/2026-08-31_progetto_rete_unificata_imitativa_exnovo_actor_predittivo.md).

- [ ] **BKL-121** — Definire la loss imitativa raw-space/served-predicted-space per il nuovo teacher, prima di costruire la rete predittiva. — [fonte 2026-08-31, riga 604](user/2026-08-31_progetto_rete_unificata_imitativa_exnovo_actor_predittivo.md).

- [ ] **BKL-122** — Riprodurre il benchmark deterministico della stessa baseline sui tre start: la sola prova centrale storica non basta. — [fonte 2026-09-06, riga 247](user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).

- [ ] **BKL-123** — Confrontare nominale e stocastico con seed indipendenti di training e incertezza delle differenze. — [fonte 2026-09-06, riga 252](user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).

- [ ] **BKL-124** — Preregistrare margini, condizioni held-out, repliche e budget prima della selezione finale. — [fonte 2026-09-06, riga 248](user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).

- [ ] **BKL-125** — Fissare baseline, manifest e protocollo comune di parità; conservare gli artefatti storici. — [fonte 2026-09-06, riga 242](user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).

- [ ] **BKL-126** — Completare il confronto imitazione/ex-novo con valutazioni periodiche secondo il benchmark: il pretraining V1 è concluso, ma smoke e pilot non esauriscono il confronto finale. — [fonte 2026-09-06, riga 250](user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).

- [ ] **BKL-127** — Completare il valutatore comune con metriche per start/giunto, cicli, fallimenti e dati mancanti; i valutatori di singole campagne non attestano l’intero benchmark. — [fonte 2026-09-06, riga 244](user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).

- [ ] **BKL-128** — Rendere il verdetto quantitativo di parità/regressione/inconclusivo e il costo per raggiungere la qualità, senza confondere fattibilità tecnica con qualità biomeccanica. — [fonte 2026-09-06, riga 254](user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).

- [ ] **BKL-129** — Eseguire e attestare separatamente le verifiche native Windows della pipeline scelta. — [fonte 2026-09-06, riga 256](user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).

- [ ] **BKL-130** — Completare la verifica dell’indipendenza degli aggiornamenti ex-novo dalle copie dei target. L’assenza di nuove copie nel suffisso privilegiato e la conservazione della maschera raccolta/PPO sono documentate; non equivalgono da sole al test specifico degli aggiornamenti. — [fonte 2026-09-14, riga 295](user/2026-09-14_osservabilita_prescribed_mascheramento_e_test_fattibilita.md).

- [ ] **BKL-131** — Validare candidati completamente mascherati ai tre start, con avvio/tracking/cicli/protezioni distinti; il primo pilot del 1 ottobre non esaurisce questa validazione. — [fonte 2026-09-14, riga 297](user/2026-09-14_osservabilita_prescribed_mascheramento_e_test_fattibilita.md).

- [ ] **BKL-132** — Costruire dati allineati e misurare osservabilità delle 35 feature su episodi indipendenti. — [fonte 2026-09-14, riga 292](user/2026-09-14_osservabilita_prescribed_mascheramento_e_test_fattibilita.md).

- [ ] **BKL-133** — Preregistrare ed eseguire A/B/C di osservabilità, trasferimento B/C e repliche: non confondere questo protocollo con le successive ablazioni clock/scala/slew. — [fonte 2026-09-14, riga 97](user/2026-09-14_avvio_piano_pretraining_50_100_fattibilita.md).

- [ ] **BKL-134** — Confrontare curriculum, avvio supervisionato e soluzioni causali di fase/memoria con evidenze; l’adozione del masking non risolve da sola l’osservabilità. — [fonte 2026-09-14, riga 293](user/2026-09-14_osservabilita_prescribed_mascheramento_e_test_fattibilita.md).

- [ ] **BKL-135** — Verificare sincronizzazione senza clock prescribed, distinguendo avvio e regime ai tre start. — [fonte 2026-09-14, riga 291](user/2026-09-14_osservabilita_prescribed_mascheramento_e_test_fattibilita.md).

- [ ] **BKL-136** — Verificare trasferimento, warm-up e resume della variante scelta e misurare il beneficio nel successivo ex-novo; correttezza tecnica e vantaggio di apprendimento sono distinti. — [fonte 2026-09-14, riga 298](user/2026-09-14_osservabilita_prescribed_mascheramento_e_test_fattibilita.md).

- [ ] **BKL-137** — Completare il confronto V1 5/50/100 sui tre start. Il training fino a100 è superato dalla continuazione fino a300; il confronto specifico 5/50/100 non risulta documentato e resta aperto. — [fonte 2026-09-14, riga 95](user/2026-09-14_avvio_piano_pretraining_50_100_fattibilita.md).

- [ ] **BKL-138** — Manutenzione separata delle assertion legacy e debiti RLlib, cycle_valid, swing timeout, probe schema, pin V20, best==last, fit_p2 V12R3, marker/audit e deflessione SEA iniziale; chiudere solo con prova specifica. — [fonte 2026-09-15, riga 161](user/2026-09-15_pretraining_unificata_valutazione_intermedia_50.md).

- [ ] **BKL-139** — **W02:** sostituire il controllo `os.kill(pid, 0)` del percorso attach con una verifica di vitalità non distruttiva e validarla su Windows. — [fonte 2026-09-15, riga 353](user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).

- [ ] **BKL-140** — **W03:** predisporre un percorso Windows per i runner J20 warm-up e restore R3, conservando la provenance storica e rendendo esplicito l'interprete della nuova esecuzione. — [fonte 2026-09-15, riga 354](user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).

- [ ] **BKL-141** — **W04:** canonicalizzare le chiavi dei percorsi nell'attestazione J21 e verificare su Windows i gate T12/T18 con artefatti integri e casi negativi. — [fonte 2026-09-15, riga 355](user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).

- [ ] **BKL-142** — **W05:** definire una politica dei fine-riga coerente con gli hash dei sorgenti unified e verificarla su un checkout Windows reale senza riscrivere i pin storici per nascondere differenze. — [fonte 2026-09-15, riga 356](user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).

- [ ] **BKL-143** — **W06:** rendere il generatore differito 39 compatibile con LF e CRLF e verificarne la materializzazione rispettando la separazione dalla V1. — [fonte 2026-09-15, riga 357](user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).

- [ ] **BKL-144** — **W07:** definire e verificare la ricollocazione di run, checkpoint e trace per il riuso delle valutazioni CP50/CP100 mantenendo le verifiche di identità del contenuto. — [fonte 2026-09-15, riga 358](user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).

- [ ] **BKL-145** — **W08:** distinguere nell'analisi sigma un report mancante da un sigma realmente nullo e verificare la rianalisi degli artefatti trasferiti. — [fonte 2026-09-15, riga 359](user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).

- [ ] **BKL-146** — Fissare un ambiente Windows riproducibile con versioni Python/Ray/Torch/NumPy/OpenSim e dipendenze native, compatibile con i checkpoint scelti. — [fonte 2026-09-15, riga 361](user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).

- [ ] **BKL-147** — Documentare e verificare root di checkout, supporto long-path Windows/Git e percorsi generati dal workflow selezionato. — [fonte 2026-09-15, riga 360](user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).

- [ ] **BKL-148** — Verificare su Windows orchestrazione, valutazioni/report e, quando prevista, materializzazione della variante 39; attestare soltanto i percorsi realmente provati. — [fonte 2026-09-15, riga 363](user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).

- [ ] **BKL-149** — Eseguire una verifica nativa Windows di caricamento plugin, reset/step, training breve con worker Ray, rollout, salvataggio/restore e warm-up, registrando gli esiti separatamente dalle evidenze macOS. — [fonte 2026-09-15, riga 362](user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).

- [ ] **BKL-150** — Validare oracle LEFT nel loop fisico sui tre start; il controllo successivo su un solo start non chiude la richiesta. — [fonte 2026-09-20, riga 199](user/2026-09-20_risultati_ablazioni_unified39_clock_scala_slew.md).

- [ ] **BKL-151** — Valutazioni dedicate dei checkpoint nominali e stocastiche, con durata/tracking/contatto e un insieme held-out distinto dalla selezione. — [fonte 2026-09-20, riga 200](user/2026-09-20_risultati_ablazioni_unified39_clock_scala_slew.md).

- [ ] **BKL-152** — Ampliare la valutazione best72 oltre cinque seed per condizione: l’evaluation iniziale è conclusa, l’estensione resta aperta. — [fonte 2026-09-22, riga 156](user/2026-09-22_risultati_finali_unified39_100iter.md).

- [ ] **BKL-153** — Isolare con prove controllate gli effetti di traiettoria, risposta dinamica e distribuzione delle azioni; confrontare in rollout comando atteso dopo clipping e media clippata. — [fonte 2026-09-22, riga 157](user/2026-09-22_risultati_finali_unified39_100iter.md).

- [ ] **BKL-154** — Estendere confronti fail/success alla stessa fase e prima della soglia nei fallimenti di training, oltre ai due casi appaiati della 100. — [fonte 2026-09-22, riga 158](user/2026-09-22_risultati_finali_unified39_100iter.md).

- [ ] **BKL-155** — Esplicitare nei riepiloghi automatici clipping raw a monte di RLlib e clipping interno all’ambiente; la distinzione nel testo non attesta l’implementazione automatica. — [fonte 2026-09-22, riga 159](user/2026-09-22_risultati_finali_unified39_100iter.md).

- [ ] **BKL-156** — Preparare confronto causale RIGHT/LEFT da zero con FSM v3, ordine ingressi, inizializzazione, raccolta, Adam/resume e budget comuni; due storie separate non provano il solo effetto del target. — [fonte 2026-09-23, riga 123](user/2026-09-23_strategia_best72_confronto_v26_e_oscuramento.md).

- [ ] **BKL-157** — Verificare separatamente reset dei diagnostici di escursione FSM e incidenza nelle configurazioni che li usano. — [fonte 2026-09-24, riga 140](user/2026-09-24_fix_contatto_terminale_hard_unified39.md).

- [ ] **BKL-158** — Valutare best80/last82 e successivi candidati contro best72 con perturbazioni comuni e misure fisiche; il piccolo pilot reward non esaurisce questo confronto. — [fonte 2026-09-24, riga 102](user/2026-09-24_avvio_left72_penetrazione_peso2_10iter.md).

- [ ] **BKL-159** — Valutare RIGHT last60 sugli stessi cinque seed prima di scegliere un checkpoint per masking. — [fonte 2026-09-24, riga 266](user/2026-09-24_resoconto_39d_right_60iter_e_robustezza.md).

- [ ] **BKL-160** — Isolare origine del momento GRF invertito nell’ultimo appoggio prima di cambiare reference/governor/reward. — [fonte 2026-09-24, riga 270](user/2026-09-24_resoconto_39d_right_60iter_e_robustezza.md).

- [ ] **BKL-161** — Approfondire transitorio seed125 su best35/40/58/60 e V26 con reference, velocità, contatto e penetrazione. — [fonte 2026-09-24, riga 268](user/2026-09-24_resoconto_39d_right_60iter_e_robustezza.md).

- [ ] **BKL-162** — Sperimentare separatamente regolarità knee e cinematica, mantenendo tracking/contatto come vincoli. — [fonte 2026-09-25, riga 214](user/2026-09-25_banco_tuning_rapido_reward_left.md).

- [ ] **BKL-163** — Verificare sufficienza del riadattamento critic con diagnostica degli errori ed evaluation indipendente; un update non dimostra convergenza. — [fonte 2026-09-29, riga 114](user/2026-09-29_esiti_pilot_reward_left_e_diagnosi_kl.md).

- [ ] **BKL-164** — Isolare conservazione KL nel critic-only con un controllo sottoposto allo stesso trattamento, configurazioni e budget preregistrati; fix non implementato. — [fonte 2026-09-29, riga 113](user/2026-09-29_esiti_pilot_reward_left_e_diagnosi_kl.md).

- [ ] **BKL-165** — Quantificare normalizzazione advantages sulle sole transizioni reali rispetto a righe artificiali. — [fonte 2026-09-29, riga 377](user/2026-09-29_audit_RL_39d_bootstrap_e_aggiornamento_PPO.md).

- [ ] **BKL-166** — Integrare in esperimento separato il fix generale confini PPO, verificando connector completi/restore/riproducibilità: replay isolato non è patch produzione. — [fonte 2026-09-29, riga 374](user/2026-09-29_audit_RL_39d_bootstrap_e_aggiornamento_PPO.md).

- [ ] **BKL-167** — Confrontare RL stock/corretto con stessa inizializzazione/reward/budget/evaluation, distinguendo continuazione best80 e apprendimento da zero, con repliche. — [fonte 2026-09-29, riga 379](user/2026-09-29_audit_RL_39d_bootstrap_e_aggiornamento_PPO.md).

- [ ] **BKL-168** — Valutare altri start/tratti AB06, recupero dopo perturbazione e carichi/contatti; distinguere generalizzazione da replay nominale. — [fonte 2026-09-29, riga 386](user/2026-09-29_audit_RL_39d_bootstrap_e_aggiornamento_PPO.md).

- [ ] **BKL-169** — Valutare robustezza best90/best80 con perturbazioni comuni a policy congelate, distinguendo seed evaluation e repliche training. — [fonte 2026-09-30, riga 114](user/2026-09-30_training_fresh_100iter_rollout_best90.md).

- [ ] **BKL-170** — Confrontare best90 fresh e best80 sulla stessa finestra con tracking per giunto/fase, contatto, cicli, impulso/area positiva ankle e regolarità knee. — [fonte 2026-09-30, riga 113](user/2026-09-30_training_fresh_100iter_rollout_best90.md).

- [ ] **BKL-171** — **Vincolo:** Per ogni candidata, eseguire qualification scientifica distinta dall’integrità tecnica del training; questa regola non è un’attestazione di robustezza generale. — [fonte 2026-08-14, riga 215](user/2026-08-14_h0_v12r9_training_readiness_preflight_monitor.md).

- [ ] **BKL-172** — **Condizionato:** Probe `train_ppo` (39/43): correggere solo con migrazione di schema dichiarata (lineage futura), non per i training comparabili con giugno. — [fonte 2026-08-18, riga 146](user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md).

- [ ] **BKL-173** — **Condizionato:** Arbitrare la tensione fra interpolazione, quota on-policy e copertura piena prima di un nuovo round interpolato. — [fonte 2026-08-24, riga 214](user/2026-08-24_v26b_audit_luglio_r1r2_e_progetto_rev4d.md).

- [ ] **BKL-174** — **Condizionato:** Riesaminare l'effetto della lunghezza del prefisso distinguendo interpolazione zero e ricette interpolate. — [fonte 2026-08-24, riga 180](user/2026-08-24_v26b_rev4d_repeat_esperimento_decorrelato.md).

- [ ] **BKL-175** — **Condizionato:** Affrontare il margine di contatto del candidato storico Rev 4E, oltre 28 mm di 0,22 mm, se il ramo viene riaperto. — [fonte 2026-08-24, riga 203](user/2026-08-24_v26b_rev4e_r2_replay.md).

- [ ] **BKL-176** — **Condizionato:** Conservare il limite interpretativo: il braccio A non converge al proprio budget e il suo NO-GO non identifica un limite del contratto. — [fonte 2026-08-25, riga 112](user/2026-08-25_v26b_b1r2b_addendum_correttivo_screening.md).

- [ ] **BKL-177** — **Condizionato:** Riesaminare il peggioramento del fold 9 dopo il minimo a epoca 100 se il ramo B1 viene riaperto. — [fonte 2026-08-25, riga 154](user/2026-08-25_v26b_b1r2b_esito_nogo_ipotesi_falsificata.md).

- [ ] **BKL-178** — **Condizionato:** Identificare la causa dell'errore delle teste rispetto a ciclo 1 senza concludere non-identificabilità da vicinanza. — [fonte 2026-08-25, riga 167](user/2026-08-25_v26b_b1r2_diagnostica.md).

- [ ] **BKL-179** — **Condizionato:** Trattare l'errore localizzato nelle 308 righe di testa, distinto dalla vecchia assenza di copertura LOCO. — [fonte 2026-08-25, riga 185](user/2026-08-25_v26b_b1r2_diagnostica_rev1.md).

- [ ] **BKL-180** — **Condizionato:** Nei futuri confronti non usare come generalizzazione il best val MSE luglio da split casuale su righe ripetute 32×. — [fonte 2026-08-25, riga 199](user/2026-08-25_v26b_b1r2a_revisione_indipendente_e_proposta.md).

- [ ] **BKL-181** — **Condizionato:** Rivedere l'igiene della selection/patience nel ramo B1, senza confonderla con convergenza o causa del NO-GO. — [fonte 2026-08-25, riga 124](user/2026-08-25_v26b_b1r1_loco_fit_esito_nogo.md).

- [ ] **BKL-182** — **Condizionato:** Riesaminare eseguibilità delle label swing ammesse per raggiungibilità,13,5% oltre il criterio 10%. — [fonte 2026-08-25, riga 139](user/2026-08-25_v26b_a0a1_protocollo_contratto25d_preflight.md).

- [ ] **BKL-183** — **Condizionato:** Risolvere max_abs WAIT oltre 0,15 nel ramo B1, se riaperto. — [fonte 2026-08-25, riga 198](user/2026-08-25_v26b_b1r2a_revisione_indipendente_e_proposta.md).

- [ ] **BKL-184** — **Condizionato:** Conservare LOTO3 fold, LOCO6 fold,11 fold B1R2, braccio budgetA e braccio lrB come rami futuri non operativi; integrarli solo con approvazione esplicita. — [fonte 2026-08-26, riga 184](user/2026-08-26_v26c_j0_audit_preregistrazione_replica_luglio.md).

- [ ] **BKL-185** — **Condizionato:** Verificare retrospettivamente l’equivalenza semantica del teacher prescritto sotto v3 nell’audit della collection storica. — [fonte 2026-08-26, riga 178](user/2026-08-26_v26c_j0_audit_preregistrazione_replica_luglio.md).

- [ ] **BKL-186** — **Condizionato:** Attestare il gate della collection teacher storica sotto v3 e corridoio corrente con la relativa evidenza. — [fonte 2026-08-26, riga 179](user/2026-08-26_v26c_j0_audit_preregistrazione_replica_luglio.md).

- [ ] **BKL-187** — **Condizionato:** Conservare i limiti J14: label open-loop su stati student-visited, phase-invalid senza soglia e 183 righe pre-mismatch mantenute senza contributo informativo; chiudere la qualità solo con closed-loop. — [fonte 2026-08-27, riga 256](user/2026-08-27_v26c_j14_post_mismatch_dagger_dataset_readiness.md).

- [ ] **BKL-188** — **Condizionato:** Eventuale integrazione/riapertura dei rami LOTO, LOCO, B1R1/B1R2 con protocollo e autorizzazione; conservarne i verdetti storici, senza considerarli già operativi. — [fonte 2026-08-27, riga 343](user/2026-08-27_qualifica_closed_loop_j9r1.md).

- [ ] **BKL-189** — **Condizionato:** Usare i ceiling empirici J8 C/D dichiarati dopo ricalibrazione, distinguendoli dalla soglia ritirata; nessun rilassamento retroattivo del FAIL J18. — [fonte 2026-08-27, riga 265](user/2026-08-27_v26c_j18_ricalibrazione_eligibilita_e_selezione.md).

- [ ] **BKL-190** — **Condizionato:** Conservare la priorità A>B>C>D e la rimozione di 4 righe su 4225 come parte esplicita del design/pesi, non una scelta variabile dopo il risultato. — [fonte 2026-08-27, riga 376](user/2026-08-27_v26c_j18_prereg_update_b_only.md).

- [ ] **BKL-191** — **Condizionato:** Mantenere max-drift come tie-break diagnostico nella selezione preregistrata; un futuro cambio del ranking richiede una nuova decisione. — [fonte 2026-08-27, riga 267](user/2026-08-27_v26c_j18_ricalibrazione_eligibilita_e_selezione.md).

- [ ] **BKL-192** — **Condizionato:** Documentare ENV_MUTATION_POLICY come superset conservativa in J19C: autorizzava variazioni start non esercitate dalla prova nominale. — [fonte 2026-08-27, riga 227](user/2026-08-27_v26c_j19c_esecuzione_heldout_g_i.md).

- [ ] **BKL-193** — **Condizionato:** Conservare e spiegare le assertion readiness diventate false dopo emissione GO/esecuzione (E54/S47 e successive), senza trattarle come regressioni scientifiche o riscrivere i file pinnati. — [fonte 2026-08-27, riga 192](user/2026-08-27_v26c_j19a_esecuzione_riproduzione_singola.md).

- [ ] **BKL-194** — **Condizionato:** Valutare in un futuro protocollo se lo shift nominale debba essere un gate anticipato: J11=0.0264 rispetto al riferimento July 0.005 era deliberatamente non binding. — [fonte 2026-08-27, riga 332](user/2026-08-27_v26c_j11_multistart_fit_execution.md).

- [ ] **BKL-195** — **Condizionato:** Monitorare bande 20/25/28 mm e margini di F/H (minimo storico circa 2.316 mm), senza trasformare diagnostici soft in nuovi hard gate a posteriori. — [fonte 2026-08-27, riga 247](user/2026-08-27_v26c_j19b_esecuzione_qualificazione_closed_loop.md).

- [ ] **BKL-196** — **Condizionato:** Conservare la fragilità storica seed 125/F e monitorare i margini nelle qualifiche; il dato precedente non certifica nuove policy. — [fonte 2026-08-27, riga 431](user/2026-08-27_v26c_j13_j12_failure_diagnosis_and_recovery_plan.md).

- [ ] **BKL-197** — **Condizionato:** Una qualifica non autorizza implicitamente warm-up/PPO: ogni nuovo stadio richiede la sua successiva autorizzazione pertinente. — [fonte 2026-08-27, riga 212](user/2026-08-27_v26c_j19c_esecuzione_heldout_g_i.md).

- [ ] **BKL-198** — **Condizionato:** Ogni nuova fase deve avere report dedicato/audit e autorizzazione pertinente prima della successiva; i GO monouso storici non vengono riutilizzati. — [fonte 2026-08-27, riga 205](user/2026-08-27_v26c_j15_fresh_refit_execution.md).

- [ ] **BKL-199** — **Vincolo:** Conservare immutati sidecar J8 e artefatti storici. — [fonte 2026-08-27, riga 356](user/2026-08-27_qualifica_closed_loop_j9r1.md).

- [ ] **BKL-200** — **Condizionato:** Se si riusa questo design, giustificare repeat 8 anziché ereditarlo dalla metodologia July; le nuove righe J14 usano repeat 1, ogni diversa ponderazione richiede preregistrazione. — [fonte 2026-08-27, riga 184](user/2026-08-27_v26c_j10_multistart_teacher_readiness.md).

- [ ] **BKL-201** — **Condizionato:** La forza vincolante del protocollo 23 agosto non autorizzato richiede decisione esplicita prima di usarlo in un nuovo esperimento. — [fonte 2026-08-27, riga 435](user/2026-08-27_v26c_j13_j12_failure_diagnosis_and_recovery_plan.md).

- [ ] **BKL-202** — **Condizionato:** Sorvegliare la copertura WAIT_HS: 20 righe uniche solo B, 160 dopo tiling; non inventare una soglia di sufficienza o confondere dato ripetuto con indipendente. — [fonte 2026-08-27, riga 241](user/2026-08-27_v26c_j10r1_multistart_teacher_collection.md).

- [ ] **BKL-203** — **Condizionato:** Conservare e spiegare i check R1 B06/I07 e R2 C29/E01/H02/H04 diventati falsi per destinazione presente dopo esecuzione, senza riscrivere gli artefatti pinnati. — [fonte 2026-08-28, riga 276](user/2026-08-28_v26c_j20_restore_audit_r2_esecuzione.md).

- [ ] **BKL-204** — **Condizionato:** Per ogni nuovo pilot fissare config/comando/budget/stop/monitor/rollback e ottenere GO nuovo; la proposta storica 5–10 sigma 0.010 non viene considerata eseguita solo perché esistono run successivi con altro budget. — [fonte 2026-08-28, riga 1059](user/2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md).

- [ ] **BKL-205** — **Condizionato:** Per una futura ricostruzione applicare l’intera checklist: parent agosto verificato, input July esclusi, actor 35/critic 84 e mapping per nome, sigma misurata, dataset tracciati, preservation funzionale, qualifica 6/6+3/3, FSM/penetrazione sane, actor congelato nel warm-up, critic/Adam/restore esatti e attestazione separata senza autorizzazione implicita al training. — [fonte 2026-08-28, riga 1029](user/2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md).

- [ ] **BKL-206** — **Condizionato:** Rivalutare sigma dopo actor predittivo o modifica causale del morphology corridor. — [fonte 2026-08-28, riga 1055](user/2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md).

- [ ] **BKL-207** — **Condizionato:** Mantenere la served contro corridor retrospettivo HS→TO→HS come metrica finale event-aligned, senza alterare reward passata o spostare il gate per accettare ritardi. — [fonte 2026-08-30, riga 358](user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).

- [ ] **BKL-208** — **Condizionato:** Se si interviene sul governor, confrontare cutoff/slew ankle in un’ablation controllata a policy congelata; la misura dei lag correnti non prova il risultato di tale modifica. — [fonte 2026-08-30, riga 374](user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).

- [ ] **BKL-209** — **Condizionato:** Se si riprende questa strategia, implementare teacher inverse-governor e label raw da stato effettivamente visitato; validare il beneficio closed-loop. Pista distinta dal RL senza teacher. — [fonte 2026-08-30, riga 392](user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).

- [ ] **BKL-210** — **Condizionato:** Preregistrare gate quantitativi per toe-off, saturazione, FSM, tracking e condizioni held-out/multi-start prima di selezionare l’actor predittivo. — [fonte 2026-08-30, riga 394](user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).

- [ ] **BKL-211** — **Condizionato:** Preregistrare pilot PPO separato 5–10 update, sigma 0,010 isotropico e logstd congelato, conservando J20/canonico; non sostituirlo con pilot di altra ricetta. — [fonte 2026-08-30, riga 388](user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).

- [ ] **BKL-212** — **Condizionato:** Ripetere il critic-only rewarm quando la reward diventa realmente asimmetrica. — [fonte 2026-08-31, riga 346](user/2026-08-31_separazione_morphology_knee_ankle_e_critic_rewarm.md).

- [ ] **BKL-213** — **Condizionato:** Ogni nuovo run usa config/namespace separati senza alterare J20, canonico lungo o GO consumati; è un invariante, non una creazione da rifare ora. — [fonte 2026-08-31, riga 415](user/2026-08-31_smoke_test_sigma_esplorazione_v26c_j20.md).

- [ ] **BKL-214** — **Condizionato:** Decidere eventuale estensione oltre 10 soltanto dopo rollout deterministici e gate del pilot J20. — [fonte 2026-08-31, riga 419](user/2026-08-31_smoke_test_sigma_esplorazione_v26c_j20.md).

- [ ] **BKL-215** — **Condizionato:** Nel pilot preregistrato verificare dopo ogni iterazione penetrazione, FSM, loss, clipping/saturazione e deriva actor; valutare deterministicamente i checkpoint prodotti. — [fonte 2026-08-31, riga 417](user/2026-08-31_smoke_test_sigma_esplorazione_v26c_j20.md).

- [ ] **BKL-216** — **Condizionato:** Eseguire DAgger closed-loop sullo stesso actor 35D con nominali e recovery, se si riprende questa strategia. — [fonte 2026-08-31, riga 610](user/2026-08-31_progetto_rete_unificata_imitativa_exnovo_actor_predittivo.md).

- [ ] **BKL-217** — **Condizionato:** Riprendere actor predittivo e compensazione del ritardo soltanto in una versione successiva; attività differita. — [fonte 2026-09-06, riga 257](user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).

- [ ] **BKL-218** — **Vincolo:** Monitorare penetrazione 20/25/28 mm, served-vs-actual e FSM v3 in ogni nuovo candidato. — [fonte 2026-09-15, riga 152](user/2026-09-15_pretraining_unificata_valutazione_intermedia_50.md).

- [ ] **BKL-219** — **Vincolo:** Preservare gli artefatti storici e distinguere portabilità dei pesi, migrazione dei risultati e riproduzione dell’esperimento. — [fonte 2026-09-15, riga 364](user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).

- [ ] **BKL-220** — **Condizionato:** Eventuali ulteriori prove da B devono isolare un fattore per volta. I successivi rami allineati modificano più impostazioni insieme; non attestano la chiusura delle ablazioni proposte né autorizzano il loro avvio. — [fonte 2026-09-20, riga 201](user/2026-09-20_risultati_ablazioni_unified39_clock_scala_slew.md).

- [ ] **BKL-221** — **Condizionato:** Valutare il significato della guardia knee a 0°, distinguendo sicurezza, durata e fedeltà; eventuale modifica richiede prova separata. — [fonte 2026-09-20, riga 202](user/2026-09-20_risultati_ablazioni_unified39_clock_scala_slew.md).

- [ ] **BKL-222** — **Vincolo:** Separare masking dalla costruzione di una baseline affidabile e fissare seed/criteri prima di rivendicare parità. — [fonte 2026-09-20, riga 203](user/2026-09-20_risultati_ablazioni_unified39_clock_scala_slew.md).

- [ ] **BKL-223** — **Condizionato:** Eventuale screening appaiato dei checkpoint LEFT25/29/40 e geometria di contatto nei tratti problematici; i soli rollout CP40 non completano tutte le celle. — [fonte 2026-09-21, riga 151](user/2026-09-21_risultati_unified39_v26_aligned_40iter.md).

- [ ] **BKL-224** — **Vincolo:** Valutare masking come passaggio distinto senza presumere che ex-novo risolva automaticamente la robustezza. — [fonte 2026-09-23, riga 278](user/2026-09-23_v26_ordine_aritmetico_e_robustezza.md).

- [ ] **BKL-225** — **Vincolo:** Distinguere RIGHT da zero, adattamento LEFT, trasferimento V26 e masking/ex-novo; non attribuire risultati di un braccio agli altri. — [fonte 2026-09-23, riga 125](user/2026-09-23_avvio_training_39d_riferimento_destro.md).

- [ ] **BKL-226** — **Condizionato:** Se prioritario, confrontare V26 e best72 con rumore assoluto comune, distinguendo target e policy media: sigma proprie con gaussiani comuni non isolano la sola esplorazione. — [fonte 2026-09-23, riga 273](user/2026-09-23_v26_ordine_aritmetico_e_robustezza.md).

- [ ] **BKL-227** — **Vincolo:** Fissare criteri distinti per completamento, cicli, forma e coppie prima della selezione e del masking; non usare il solo return. — [fonte 2026-09-24, riga 272](user/2026-09-24_resoconto_39d_right_60iter_e_robustezza.md).

- [ ] **BKL-228** — **Vincolo:** Distinguere beneficio della continuazione da spiegazione del divario nell’apprendimento da zero, che richiede un controllo dedicato. — [fonte 2026-09-24, riga 309](user/2026-09-24_audit_reward_left_fasi_contatto_e_clipping.md).

- [ ] **BKL-229** — **Condizionato:** Se si procede, eseguire il controllo LEFT72 peso 1 contro peso 2, con terminale hard corretto, stesso resume, budget e criteri fisici; il solo ramo peso 2 non isola causalità. — [fonte 2026-09-24, riga 307](user/2026-09-24_audit_reward_left_fasi_contatto_e_clipping.md).

- [ ] **BKL-230** — **Vincolo:** Eventuali modifiche reward per le coppie devono usare stesso checkpoint/budget, tracking e contatto come vincoli e confronti separati knee/ankle. — [fonte 2026-09-24, riga 121](user/2026-09-24_best80_cause_coppie_ankle_knee.md).

- [ ] **BKL-231** — **Vincolo:** Separare regolarità della coppia da forma legata a reference/carico, giudicando anche tracking e contatto senza copiare V26 o introdurre regole AB06. — [fonte 2026-09-29, riga 170](user/2026-09-29_indagine_generale_39d_imitazione_e_forma_coppie.md).

- [ ] **BKL-232** — **AB08-Q01:** Definire e implementare un controllo generale dei marker prima della IK, con politica esplicita per outlier/missing data, conservazione delle sorgenti e tracciabilità dei campioni trattati; ricalcolare e confrontare la IK senza cambiare parametri SEA/control chain.

- [ ] **BKL-233** — **AB08-Q02:** Analizzare separatamente i residui persistenti durante il cammino, soprattutto marker della coscia destra e `L_Shank_Upper`; verificare l'eventuale beneficio senza attribuire agli outlier la media RMS di 12 mm o introdurre correzioni specifiche AB08 prive di giustificazione.

- [ ] **BKL-234** — **AB08-Q03:** Qualificare AB08 su contatto/eventi V26, GRF online e tracking dinamico dopo la preparazione e gli eventuali interventi sulla qualità della IK. La build senza smoke test e gli audit statici non chiudono questa qualifica.

## Chiusi

Nessuna voce chiusa dopo la creazione del registro.
