# Daily Report — 2026-09-23

Instruction check token: `CMC_AGENT_OK_2026`.

Daily ricostruito retrospettivamente dai report utente; non è una chiusura `end_day` effettuata in quella data.

<!-- september-coverage-completion-2026-10-05 -->

## Integrazione della copertura — 5 ottobre 2026

Sintesi retrospettiva di attività già documentate nelle fonti. Questa integrazione non esegue training, simulazioni o test scientifici; i risultati citati sono quelli archiviati. Le rettifiche successive sono distinte dagli esiti originari.

<a id="coverage-2026-09-23_strategia_best72_confronto_v26_e_oscuramento"></a>

### Best72: scelta operativa, valutazione e strategie distinte

Fonte: [2026-09-23_strategia_best72_confronto_v26_e_oscuramento](../user/2026-09-23_strategia_best72_confronto_v26_e_oscuramento.md).

L’utente sceglie best 72 LEFT come punto di lavoro: cinematica qualitativamente accettabile, coppie e robustezza ancora inferiori alla V26. Il 22 settembre 18:27–19:00 sono conclusi 11 rollout (nominale+5 originali+5 dimezzati):500 passi nominali, 2 cicli, return 200,90, RMSE 6,56°/5,98°, penetrazione 21,83 mm, completamento 3/5 e 4/5. Confronto 60(254,16,4/5 e 4/5) e 100(229,02,3/5 e 5/5) non stabilisce graduatoria generale da robustezza. Checkpoint 72/milestone identici e preservati. Scelta utente non significa best su ogni criterio.

Per diagnosi si propone controllo V26 rimappato per RIGHT senza apprendimento, poi training RIGHT/LEFT fresh con clock/template/shift 0,465–0,452/normalizzazione coerenti, inizializzazione semantica comune, budget per transizioni e FSMv3; prima lettura 32 / budget 40, non 100 automatiche. La continuazione della best72 LEFT verso RIGHT sarebbe un test di transfer, non controllo da zero. I test RIGHT successivi descritti in questo daily sono autorizzazioni/esecuzioni separate rispetto alla discussione.

Per autonomia si separano reward, visibilità ed esplorazione: masking episodico spegne per intero 0/1/35–38, non attenua segnali; teacher imitativo resta interno. La versione di quel giorno imponeva ex_novo con visibilità 0: ex-novo visibile+curriculum richiedeva modifica esplicita, non solo YAML. Il preset exnovo cambiava inoltre sigma 0,01 freeze, batch 4608, LR 5e-7, 1 epoca, 3 start, slew e scala vel 10 contro 1 della 72: non era un resume “sola reward”. Proposto graduale: ex-novo visibile, poi biforcazione dal medesimo parent visibile/masked con uguale budget e sigma e gate sugli episodi hidden; un controllo imitativo visibile serve all’attribuzione causale.

Discussione documentale, nessun training/patch avviato dalla fonte; letti artefatti best 72, codice transizione e contatori V26. Restano TODO su comando atteso dopo clipping, failure alla stessa fase, semantica telemetria, repliche e 5+seed. I controlli V26/RIGHT risultano successivamente eseguiti, il decoupling reward-canali è implementato il 1 ottobre su CP90; questo stadio successivo non chiude retroattivamente robustezza/comparazioni della 72.

<a id="coverage-2026-09-23_riuso_pesi_v26_nella_unified39_riferimento_destro"></a>

### Trasferimento V26 per inferenza: primo esito e rettifica

Fonte: [2026-09-23_riuso_pesi_v26_nella_unified39_riferimento_destro](../user/2026-09-23_riuso_pesi_v26_nella_unified39_riferimento_destro.md).

Senza training, V26 best 87 imitativa viene esportata nella 39D con RIGHT storico, clock RIGHT non shiftato, shift knee 0,465/ankle 0,452, scala qdot 1, canali visibili, FSMv3 e reset 12,99 s. Target 0–5 storici vengono rimappati 35–38 con permutazione colonne primo layer `[0,1,6,…,38,2,3,4,5]`; bias/layer seguiti e alias preservati. Export solo actor: critic non presente nel sorgente e Adam non trasferito; contratto RIGHT diagnostico distinto da LEFT, non checkpoint training ready.

Il primo rollout completa 500 passi e recupera buon tracking(RMSE 6,68°/1,25° contro V26 6,91°/1,46°), ma ankle picco 22,16 Nm contro 10,19, un solo ciclo valido/evento invalido e flessione knee 55,63° contro 63,78°. Return 370,62 vs 363,79 non prova qualità superiore. Su 500 osservazioni+4096 input equivalenza logits entro 7,15e-7; controllo negativo senza remap errore>4,7. Durante inferenza runtime due reti coincidono entro 2,38e-7 e target/scale/reset sono identici. Sono state verificate 24 hash, reward, watchdog e 7 plot.

Le prime azioni differiscono di 5,96e-8 e feedback fisico amplifica: scarti>0,1° ankle step 37/knee step 41 precedono prima differenza FSM 95. La fonte iniziale formulava sensibilità numerica come ipotesi, non causa isolata provata. L’aggiornamento dello stesso 23 riporta il controllo con ordine aritmetico originale: 500 azioni e STO identici, dimostrando questa causa per il residuo del trasferimento; dettagli nella sezione seguente. Non estendere questa spiegazione alla 72 LEFT, con pesi/target/storia diversi. File nuovi in `validation/v26_weight_transfer_2026-09-23/`, nessuna modifica produzione o critic/optimizer.

<a id="coverage-2026-09-23_v26_ordine_aritmetico_e_robustezza"></a>

### Ordine dei calcoli V26 e batteria con due scale di rumore

Fonte: [2026-09-23_v26_ordine_aritmetico_e_robustezza](../user/2026-09-23_v26_ordine_aritmetico_e_robustezza.md).

Il controllo deterministico inverte il riordino degli ingressi prima dell’inferenza, conservando colonne/ordine somme originali V26. Con FSMv3 attuale riproduce esattamente 500 azioni/osservazioni, 5 s/2 cicli e STO cinematica(22 colonne), SEA(5), GRF(27), 5000 righe ciascuno con SHA identico. Return 363,785609 differisce ≈ 4e-13 solo per la somma; JSON metadata/float 32 non sono tutti byte identici. Recupera ankle picco 10,188 Nm e chattering HP20 3,280 Nm. Il residuo del primo trasferimento è quindi riconducibile al diverso ordine floating point, non all’inevitabile incapacità del contenitore/FSMv3; non vengono provati PPO/critic/resume.

La batteria separata usa l’export con colonne permutate( NON l’adattatore ordine originale):5 seed Gaussiani 123–127 ×scale 1/0,5, start unico. V26 trasferita completa 5/5 in entrambe scale, contro best 72: 3/5 e 4/5; 10 nuovi rollout, nessun timeout tecnico. Stessi gaussiani appaiati, ma sigma appresa propria e stati/target diversi. Massima pen V26 ai campioni reward 25,084 mm originale/17,857 dimezzata; completamento non è penetrazione zero.

Su 303 passi comuni, std ankle media V26 0,515 contro best 72: 0,766(≈49% più alta), clipping componenti 24,09% vs 33,47%; la distribuzione è una pista, non causa isolata. Dimezzando sigma cambia anche media dopo clipping e best 72 rimane un failure. RMSE ankle V26 migliore, knee non uniformemente; confronti descrittivi su LEFT vs RIGHT non attribuiscono effetto al solo target. Cinque seed non sono repliche training; intervalli ampi, nessuna equivalenza statistica.

Verificati 20 episodi incluso 10 archivi best 72, azioni esatte, reward entro 4,55e-13, 30 hash e 10 watchdog; grafici in root `plot/09_23_2026_2/` e comparativi, diagnostica in `validation/v26_robustness_2026-09-23/`. Nessun training o produzione cambiata. Seguito: eventuale rumore assoluto comune, apprendimento RIGHT/LEFT controllato e masking come passo distinto; il successo V26 dimostra eseguibilità, non che il learner ritrovi la soluzione.

<a id="coverage-2026-09-23_avvio_training_39d_riferimento_destro"></a>

### RIGHT39 da zero: selettore, contratti e avvio a 40

Fonte: [2026-09-23_avvio_training_39d_riferimento_destro](../user/2026-09-23_avvio_training_39d_riferimento_destro.md).

Introdotto `target_reference:right_v26` in YAML/CLI→ambiente/training/rollout/checkpoint; export RIGHT rifiutato come LEFT e viceversa. Target RIGHT storico SHIFT 0,465/0,452 e clock RIGHT non sfasato, scala qdot 1,39/88, FSMv3, slew off, 13 worker/batch 4096,10 epoche LR 1e-4, start 12,99 s e 40 iter(163840 campioni). Pesi casuali seed 123: actor/critic smoke uguali al LEFT iniziale, run actor uguale al smoke, ma non importa i pesi appresi dello smoke/V26.

107 test passati su sei mask canali, clock/target/contratto/factory/LEFT regression. Preflight 500 campioni target q/qdot zero errore e clock entro 2,98e-8, reward storica entro 3,98e-13; 20 passi storici visibili/hidden con stati/reward identici a pari azione. Smoke 2 × 192+resume 3=576 transizioni, 3 worker: actor/critic/logstd update finite, 12 Adam stati vivi esatti 8→12, KL PPO conservati; reload export 20 passi recupera automaticamente la referenza RIGHT. Non prova convergenza.

Run fresh avviato 23 settembre 12:50:29 con monitor 31255/train 31256,13 EnvRunner e watchdog stop 40. Congelati 48 hash e 14 input storici invariati. File: phase_target/env/factory, target availability/actor transition, contratto/config/trainer/rollout, config/test e validation right_reference_training. Live nativa macOS soltanto. Monitor/evaluation conclusi nelle sezioni successive, senza introdurre oscuramento o ex-novo in questo test.

<a id="coverage-2026-09-23_risultati_39d_right_40iter"></a>

### RIGHT40: confronto a pari esperienza e qualità residua

Fonte: [2026-09-23_risultati_39d_right_40iter](../user/2026-09-23_risultati_39d_right_40iter.md).

40/40 iterazioni concluse 18:18:21 in 5 h 27 m 48 s, 163840 transizioni, 3240 Adam update, zero restart/salti e hash 48 invariati. A esperienza comparabile su 20480 campioni: RIGHT39/43=90,7% completi, LEFT 19/49=38,8%, V26 38/43=88,4%(163212 cumulativi). RIGHT supera 80% dalla 27, mantenendolo fino 40; già 21–25 ha 76,2% contro LEFT 37,3%, quindi la ripresa LEFT non è unica spiegazione. Cambia il compito insieme clock/template/velocità/normalizzazione, non si isolano quattro effetti.

| Nominale | Best 35 | Last 40 | V26 best 87 più maturo |
|---|---:|---:|---:|
| Return |261,94|286,32|363,79|
| RMSE knee/ankle |9,59°/3,63°|9,39°/2,76°|6,91°/1,46°|
| Picco ankle positivo |5,66 Nm|3,30 Nm|10,19 Nm|
| Cicli validi |1|1|2|

Entrambi 500 passi, ma FSM conta 1 ciclo, invalidi 2/3 e resync 2/1, con HS precoci e qualifica degradata del ciclo; completamento 90,7% train non certifica regolarità a checkpoint fisso. Il knee max 51,51°/48,36° manca 64,32° target; last 40 migliora ankle/return ma riduce flessione rispetto 35. Chattering HP20≈3,07/3,10 Nm vicino V26, anti-flip zero, SEA actual-served ≈ 0,16°/0,29–0,30°: errore principale policy/governor. Ritardo descrittivo knee 0/ankle 50 ms non spiega il deficit. Sigma 0,69/0,60 alla 35,0,67/0,54 alla 40 è correlazione, non dimostrazione causale.

Verificati target/clock/reward/action/checkpoint, 14 plot canonici e figure confronto/FSM in `validation/right_reference_results_2026-09-23/`; nessuna patch controller/FSM. Esito: la 39D su RIGHT rende il training molto più stabile, non recupera ancora tutta la qualità V26 e una sola replica/start non prova equivalenza. Proposto screening 35/40 appaiato con noise/cicli/resync, poi decisione di continuazione; entrambi i passi sono effettivamente svolti nel seguito dello stesso 23, non erano esiti di questo report.

<a id="coverage-2026-09-23_screening_right39_e_continuazione_60"></a>

### Screening 35/40 e ripresa autorizzata fino a 60

Fonte: [2026-09-23_screening_right39_e_continuazione_60](../user/2026-09-23_screening_right39_e_continuazione_60.md).

Screening concluso 19:46:10 nuovi rollout best 35/last 40,5 seed 123–127 rumore originale, start 12,99 s, max 500 step; 10 archivi V26 best 87/LEFT best 72 aggiunti al confronto. Standard Gaussians identici, ogni policy conserva propria sigma, state-dependent: non rumore assoluto uguale. Completi/casi con ≥ 2 cicli/zero eventi con completamento e ≥ 2 cicli: best 35=5/5,5/5,1/5; last 40=4/5,3/5,1/5; V26=5/5,3/5,2/5; LEFT72=3/5,3/5,1/5. Cicli vs completamento vs eventi sono criteri distinti, anche V26 completa due rollout senza cicli.

Last 40 seed 125 si arresta step 301 per penetrazione ordinaria 28,815 mm, 1 ciclo/0 invalidi; nessun hard nei 10 nuovi casi e 35 max 23,757 mm. Sul prefisso 301 passi RMSE knee/ankle:35=10,98°/4,55°, 40=9,77°/4,60°, V26=6,38°/2,88°. V26 è più matura, e 5 seed fissi non sono prova d’equivalenza. Le metriche 299 passi del successivo best 58 hanno diverso supporto e non vanno combinate con queste 301.

L’utente autorizza ripresa dal checkpoint completo 40 fino 60 assoluta, senza 100 automatiche; avvio 19:47 monitor 56530/train 56531. Solo budget cambia:20 update/81920 transizioni, 245760 totali, stessa reward RIGHT/clock 100%/scala 1/governor/FSMv3. Best 35 preservato. Restore preflight 12 Adam stati esatti 3240, KL 0,20000000298, pesi identici e 145 input/36 milestone hash; startup audit PASS. Nuovi episodi fisici/RNG processi significano non continuazione bitwise di un processo mai interrotto. File diagnostici `right_reference_screening` e `right_reference_resume60_2026-09-23/`; niente codice produzione o plugin.

I TODO controllo fine 60/audit e analisi 41–60 sono poi conclusi nel resoconto seguente; nessun prolungamento 80/100 o oscuramento era accodato. Robustezza/generalizzazione e rumore assoluto restano aperture, se valutate utili.

<a id="coverage-2026-09-23_risultati_39d_right_60iter"></a>

### RIGHT60: consolidamento concluso dopo mezzanotte

Fonte: [2026-09-23_risultati_39d_right_60iter](../user/2026-09-23_risultati_39d_right_60iter.md).

Training finito 23 settembre 22:26, analisi/7 nuovi rollout conclusi dopo mezzanotte il 24; il daily 24 già consolida il resoconto. Qui è riportata la linea dell’esperimento 23 senza attribuire al 23 un’analisi già finita al 24. Ripresa 41–60: 81920 nuovi passi, 245760 totali, 153/157=97,5% completi, ultimi 37/40=92,5%; 4 failure penetrazioni ordinarie, zero crash e stop esatto 60. V26 a esperienza simile iter 53 con 245132 campioni.

Best training 58 return 251,974; nominale last 60 migliore: RMSE 7,066°/2,414° contro 58=7,223°/3,118° e V26=6,906°/1,459°. Tutti 500 passi 2 cicli zero eventi nominali, ma knee flex 60=51,44° contro 64,32° target e ankle burst 25,89 Nm contro 58=31,85/V26=10,19. Con feedback SEA actual-served 0,180°/0,277°, problema principale reference generata/contatto; ritardo descrittivo 20/30 ms solo parziale. Il return può migliorare mentre ritorna anti-flip nell’ultimo appoggio.

Best 58 rumore originale:4/5 completi, 3/5 con ≥ 2 cicli; seed 125 hard step 300, mentre 35/V26 completano e 40 fallisce 301 ordinario. Metriche fisiche sul prefisso 299 passi escludono terminale incompleto. La reference ankle rapida riduce il margine, seppure RMSE actual-target può apparire basso. Il caso fallisce senza resync; l’ultimo rumore o la sigma propria non isolano la causa. Restore actor/critic/Adam KL con 3240→3321→4860, target 500 campioni, reward/rumore/hash 561 verificati e 14 PNG canonici. File `validation/right_reference_results60_2026-09-23/`, nessuna patch produzione.

TODO specifici conservati: valutare last 60 sugli stessi 5 seed prima di usarla, indagare seed 125 su 35/40/58/60/V26, isolare origine momento GRF ultimo appoggio; distinguere forma/coppie/completamento e pianificare ampiezza eval/repliche. La priorità successiva LEFT non chiude questi controlli RIGHT.

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

### Chiusure documentate e consolidamento retrospettivo

- [x] Daily 2026-08-16 e 2026-08-17 da consolidare al prossimo `end_day`. — [fonte 2026-08-17, riga 121](../user/2026-08-17_diagnosi_divergenza_luglio_v26_e_proposta_retraining_imitativo_nativo.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-17_daily-report.md)).
- [x] Daily 2026-08-16 da consolidare al prossimo `end_day` (14 user report della giornata). — [fonte 2026-08-17, riga 95](../user/2026-08-17_h0_v12r17_verdetto_e_probe_capacita.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-17_daily-report.md)).
- [x] Daily 2026-08-16/17/18 da consolidare al prossimo `end_day`. — [fonte 2026-08-18, riga 154](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-18_daily-report.md)).
- [x] Daily 2026-08-18/19 da consolidare al prossimo `end_day`. — [fonte 2026-08-19, riga 159](../user/2026-08-19_indagine_iterazioni_lente_crash_storm_reject_continue_fix.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-19_daily-report.md)).
- [x] Daily 2026-08-18/19/20 da consolidare al prossimo `end_day`. — [fonte 2026-08-20, riga 104](../user/2026-08-20_audit_equivalenza_giugno_per_fresh_run_nativo_v26.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-20_daily-report.md)).
- [x] Valutare best72 con undici rollout e registrare la scelta dell’utente come punto di lavoro. — [fonte 2026-09-22, riga 156](../user/2026-09-22_risultati_finali_unified39_100iter.md). **Chiuso:** Valutazione best72 conclusa e scelta registrata. ([evidenza](../user/2026-09-23_strategia_best72_confronto_v26_e_oscuramento.md)).
- [x] Scegliere priorità RIGHT/ex-novo rispetto a best72. — [fonte 2026-09-23, riga 169](../user/2026-09-23_riuso_pesi_v26_nella_unified39_riferimento_destro.md). **Chiuso:** Smoke conclusi e training RIGHT da zero avviato; le scelte successive CP90 sono separate. ([evidenza](../user/2026-09-23_avvio_training_39d_riferimento_destro.md)).
- [x] Valutare best35/last40 con rumore appaiato e metriche FSM oltre al completamento. — [fonte 2026-09-23, riga 255](../user/2026-09-23_risultati_39d_right_40iter.md). **Chiuso:** Dieci rollout appaiati best35/last40 sui seed123–127; tabella righe28–33 distingue completamento, cicli validi ed eventi invalidi. ([evidenza](../user/2026-09-23_screening_right39_e_continuazione_60.md)).
- [x] Analizzare RIGHT40 a parità di esperienza con LEFT/V26, separando training/evaluation e includendo rumore, cinematica e coppie. — [fonte 2026-09-23, riga 122](../user/2026-09-23_avvio_training_39d_riferimento_destro.md). **Chiuso:** Analizzate tutte le iterazioni e valutati best35/last40; il confronto stocastico è attestato separatamente nello screening successivo. ([evidenza](../user/2026-09-23_risultati_39d_right_40iter.md)).
- [x] Decidere continuazione RIGHT, confermare arresto alla 60 e audit, analizzare 41–60 e selezionare valutazioni successive. — [fonte 2026-09-23, riga 257](../user/2026-09-23_risultati_39d_right_40iter.md). **Chiuso:** Risultati e screening best58 pubblicati, audit resume superati; nessuna estensione 80/100 richiesta. ([evidenza](../user/2026-09-23_risultati_39d_right_60iter.md)).
- [x] Separare sensibilità numerica/runtime conservando ordine aritmetico storico o azioni registrate. — [fonte 2026-09-23, riga 166](../user/2026-09-23_riuso_pesi_v26_nella_unified39_riferimento_destro.md). **Chiuso:** STO di cinematica/coppie/GRF identici byteperbyte allo storico; azioni e osservazioni coincidono nelle righe36–37. ([evidenza](../user/2026-09-23_v26_ordine_aritmetico_e_robustezza.md)).
- [x] Quantificare robustezza stocastica della V26 trasferita con riferimento storico e stesso protocollo. — [fonte 2026-09-23, riga 164](../user/2026-09-23_riuso_pesi_v26_nella_unified39_riferimento_destro.md). **Chiuso:** V26 completa5/5 episodi con rumore originale e5/5 dimezzato; confronto best72 nelle righe9–11, risultato circoscritto a questo screening. ([evidenza](../user/2026-09-23_v26_ordine_aritmetico_e_robustezza.md)).

<!-- reconciled-report-todos-2026-10-05:end -->
