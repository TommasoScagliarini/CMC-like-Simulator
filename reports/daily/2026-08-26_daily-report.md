# Daily Report - 2026-08-26

> Aggiornamento della struttura del 2026-10-19: le procedure conservate nella
> precedente `Trajectory Generator/baseline_MLP/validation/` sono ora in
> [`imitation_to_exnovo/`](<../../Trajectory Generator/baseline_MLP/imitation_to_exnovo/>).
> I percorsi e gli hash nel corpo restano quelli del documento originale.
> Mappa della rinomina, compatibilità e dipendenze già mancanti sono nel
> [registro della rinomina](../cleanup/2026-10-19_imitation_to_exnovo_rename/README.md).


> Aggiornamento successivo del 2026-10-19: i riferimenti di questo documento
> alla filiera V12, al bundle V26C, alla campagna sigma o ai file nella radice
> di `baseline_MLP/validation/` sono storici: quei materiali sono stati rimossi.
> La workspace V26C e i suoi checkpoint sono conservati. Per rieseguire
> le procedure con input o prerequisiti rimossi occorre il loro ripristino.
> Vedere il [registro della pulizia](../cleanup/2026-10-19_baseline_validation_removal/README.md).

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

- [V26C J0 — audit e preregistrazione della replica July-faithful](../user/2026-08-26_v26c_j0_audit_preregistrazione_replica_luglio.md)

## Sintesi

È stata ricostruita e preregistrata la pipeline di luglio come metodo, mantenendo il **parent operativo esclusivamente V26 imitation agosto**. La ricostruzione ha mostrato due stadi distinti: BASE su 500 coppie teacher con fit dell'intera mean actor, seguito da MARKOV con anchor auto-distillate, recovery student-visited e multistart teacher. Il nuovo lineage usa un unico actor **35→256→256→4**; nello stadio base le colonne clock 0–1 e controller-memory 25–34 sono hard-zero, poi le colonne 25–34 vengono riabilitate nello stesso actor.

Il preflight J0 ha anche rilevato che il builder storico non propagava tutte le chiavi FSM v3/morphology: J1 non può quindi riusarlo implicitamente e deve costruire l'environment corrente con configurazione completa e verifiche fail-closed. Sigma è rimasta esplicitamente non assunta e fuori scope fino allo stadio recovery.

Il verdetto è stato **GO-FOR-ARCHITECT-REVIEW**: nessun fit, rollout, dataset o training è stato eseguito in J0.

## Lavoro svolto

- Ricostruiti da codice/artefatti i parametri BASE e MARKOV di luglio.
- Verificata la lineage: nessun peso, dataset o trace luglio come input operativo.
- Creati audit, test e receipt J0 in `Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/`.
- Definito il contratto del futuro collector J1 con runtime V26, detector corrente, FSM v3 e Morphology Corridor completo.
- Conservati LOTO/LOCO/B1R1/B1R2 come TODO futuri, fuori dalla replica.

## Test e verifiche

- `test_v26c_j0_audit.py`: **156 check PASS**.
- Audit read-only e receipt no-clobber completati.
- Produzione, FSM, GRF, morphology, reward, SEA e plugin C++ invariati.

## TODO completati

- [x] Ricostruire i due stadi luglio senza usarne gli artefatti come parent.
- [x] Fissare un unico actor 35D senza adapter e senza widening.
- [x] Identificare l'inadeguatezza del builder storico rispetto al runtime v3/morphology corrente.

## TODO aperti e propagati

- [ ] Dopo approvazione, implementare ed eseguire J1 teacher collection con builder full-config e gate fail-closed.
- [ ] Eseguire J2 BASE fit e J3 qualifica closed-loop prima di aprire MARKOV/recovery.
- [ ] Misurare sigma nel lineage corrente prima di J4; non ereditare automaticamente 0,005.
- [ ] Applicare e registrare la policy di penetrazione corrente; le soglie verranno poi consolidate in 20 mm soft, 25 mm diagnostica luglio e 28 mm hard.
- [ ] Conservare LOTO/LOCO/B1R1/B1R2 come TODO non operativi.
- [ ] Generalizzazione multimodello EPIC e debiti tecnici/amministrativi ereditati ancora aperti.


## Addendum retrospettivo — integrazione del 5 ottobre 2026

Questa integrazione completa il consolidamento documentale dei report utente della giornata. Riporta soltanto esperimenti, audit e verifiche già descritti nelle fonti; il 5 ottobre non sono stati eseguiti nuovi fit, training, rollout o test del simulatore. Le rettifiche successive sono dichiarate come tali e non promuovono retroattivamente candidati respinti.

<a id="integrazione-2026-10-05-2026-08-26-v26c-j0-audit-preregistrazione-replica-luglio"></a>

### V26C J0 — audit e preflight della replica July-faithful su parent V26 agosto e runtime v3

Verifica di copertura: il daily già consolida BASE/MARKOV, unico 35D, parent agosto, builder full-config obbligatorio, GO-FOR-ARCHITECT-REVIEW e 156 checkPASS. Il gate closed-loop primario è distinto dal gate offline; le due contract-ID sono chiavi diverse e le guardie operative sono quelle correnti, non i 25 mm di luglio.

Fonte: [2026-08-26_v26c_j0_audit_preregistrazione_replica_luglio.md](../user/2026-08-26_v26c_j0_audit_preregistrazione_replica_luglio.md).

## TODO delle fonti — stato documentato al 5 ottobre 2026

Le chiusure sotto riportate riguardano lo scope indicato e non modificano il verdict degli esperimenti precedenti. Le voci condizionate sono rami o confronti futuri: richiedono approvazione e non costituiscono autorizzazione a eseguire. Conservazione/no-clobber dei receipt storici rimangono invarianti.

- [ ] **Condizionato** — Conservare LOTO3 fold, LOCO6 fold,11 fold B1R2, braccio budgetA e braccio lrB come rami futuri non operativi; integrarli solo con approvazione esplicita. Fonte: [2026-08-26_v26c_j0_audit_preregistrazione_replica_luglio.md](../user/2026-08-26_v26c_j0_audit_preregistrazione_replica_luglio.md), riga184. LOT O/LOCO/B1R1/B1R2 ancora futuri soggetti ad approvazione esplicita. Evidenza: [2026-10-01_daily-report.md](./2026-10-01_daily-report.md), riga238.

- [ ] **Condizionato — O1/O2**: auditare l’equivalenza semantica del teacher e il gate della collection storica sotto v3/corridoio. I punti aperti sono espliciti nel [report J0](../user/2026-08-26_v26c_j0_audit_preregistrazione_replica_luglio.md), righe178–179. Non è disponibile una evidenza user dedicata J1–J8; il successivo J19 non attesta automaticamente questi controlli. Non viene richiesta una nuova collection o un nuovo rollout.

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
- [ ] Completare il milestone di recupero AB06 qualificato e attestato come training-input-only. La ricostruzione J19–J21 sostituisce la vecchia proposta F0/F1–F7, senza dichiarare eseguite tutte quelle fasi né il benchmark generale di parità. — [fonte 2026-08-23, riga 200](../user/2026-08-23_fase0_gate_finale_recupero_ab06.md).
- [ ] **Osservabilità RLlib:** nel prossimo eventuale run registrare environment configurati/sani, batch corrente/lifetime e generation/replacement dei runner in campi distinti. — [fonte 2026-08-23, riga 109](../user/2026-08-23_f1_s1_protocollo_tooling_dryrun.md).
- [ ] Chiarire perché B3/target_phase vale zero sotto runtime v 3 anche con cicli validi. — [fonte 2026-08-24, riga 212](../user/2026-08-24_v26b_audit_luglio_r1r2_e_progetto_rev4d.md).
- [ ] Misurare sigma sul lineage corrente prima delle recovery, senza ereditare il placeholder 0,005. — [fonte 2026-08-24, riga 211](../user/2026-08-24_v26b_audit_luglio_r1r2_e_progetto_rev4d.md).
- [ ] Ottenere cammino completo con almeno due cicli e plantarflessione, preservando gli start richiesti. — [fonte 2026-08-24, riga 213](../user/2026-08-24_v26b_audit_luglio_r1r2_e_progetto_rev4d.md).

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

### Chiusure documentate e consolidamento retrospettivo

- [x] Daily 2026-08-16 e 2026-08-17 da consolidare al prossimo `end_day`. — [fonte 2026-08-17, riga 121](../user/2026-08-17_diagnosi_divergenza_luglio_v26_e_proposta_retraining_imitativo_nativo.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-17_daily-report.md)).
- [x] Daily 2026-08-16 da consolidare al prossimo `end_day` (14 user report della giornata). — [fonte 2026-08-17, riga 95](../user/2026-08-17_h0_v12r17_verdetto_e_probe_capacita.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-17_daily-report.md)).
- [x] Daily 2026-08-16/17/18 da consolidare al prossimo `end_day`. — [fonte 2026-08-18, riga 154](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-18_daily-report.md)).
- [x] Daily 2026-08-18/19 da consolidare al prossimo `end_day`. — [fonte 2026-08-19, riga 159](../user/2026-08-19_indagine_iterazioni_lente_crash_storm_reject_continue_fix.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-19_daily-report.md)).
- [x] Daily 2026-08-18/19/20 da consolidare al prossimo `end_day`. — [fonte 2026-08-20, riga 104](../user/2026-08-20_audit_equivalenza_giugno_per_fresh_run_nativo_v26.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-20_daily-report.md)).

<!-- reconciled-report-todos-2026-10-05:end -->
