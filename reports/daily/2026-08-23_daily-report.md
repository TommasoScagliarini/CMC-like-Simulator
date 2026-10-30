# Daily Report - 2026-08-23

> Aggiornamento successivo del 2026-10-19: i riferimenti di questo documento
> alla filiera V12, al bundle V26C, alla campagna sigma o ai file nella radice
> di `baseline_MLP/validation/` sono storici: quei materiali sono stati rimossi.
> La workspace V26C e i suoi checkpoint sono conservati. Per rieseguire
> le procedure con input o prerequisiti rimossi occorre il loro ripristino.
> Vedere il [registro della pulizia](../cleanup/2026-10-19_baseline_validation_removal/README.md).

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

- [Fase 0 — gate finale del recupero AB06](../user/2026-08-23_fase0_gate_finale_recupero_ab06.md)
- [F1/S1 — protocollo, tooling, dry-run e primo bridge F2R](../user/2026-08-23_f1_s1_protocollo_tooling_dryrun.md)

## Sintesi

La Fase 0 è stata chiusa con **PASS tecnico**: 58 job analizzati, provenienza content-addressed, overlay finali e deriva dell'attore verificati. Il verdetto scientifico sulla lineage B0820 è invece **FAIL come baseline AB06**: il collasso cinematico precede PPO, la trasformazione 39D→35D ha eliminato quattro feature privilegiate senza un refit target-domain sufficiente e il clock autonomo non riesce ad avviarsi in closed loop. I 50 update PPO non hanno ricostruito la forma biologica.

L'analisi isolata ha separato due cause. Con i target ricostruiti ma senza clock (`aiso_det`) non si completa alcun ciclo; con clock prescritto (`aiso_clk_diag`) si completano due cicli su tutti e tre gli start, dimostrando il **deadlock di bootstrap del clock**. È inoltre emersa un'ambiguità semantica della FSM v3 dopo la cancellazione di HS: i prodotti phase-aligned restano correttamente fail-closed finché il contratto non viene reso univoco.

È stato quindi preparato e auditato un bridge F2R. L'unico tentativo closed-loop T1R ha prodotto tre job tecnicamente validi, ma solo lo start nominale ha raggiunto 500 step; gli start −0,20 e +0,20 sono terminati per penetrazione a circa 28,8 mm. Il gate A è fallito, `unlock_t2=false`, quindi T2/T3, aggregazione e training sono rimasti bloccati. Nel gate B3, **−0,0099 rad non supera il requisito ≤ −0,03 rad**, perché −0,0099 è numericamente maggiore di −0,03.

## Lavoro svolto

- Completati matrice, overlay, actor drift e audit finale della Fase 0.
- Introdotti strumenti F1/F2R additivi per adapter teacher, raccolta, fit, receipt, gate e verifica no-clobber.
- Eseguite famiglie diagnostiche `aiso_det` e `aiso_clk_diag` per isolare l'effetto del clock.
- Preparato e sottoposto ad audit il protocollo T1R; eseguito una sola volta il rollout closed-loop autorizzato e poi il relativo gate.
- Preservati produzione, pesi di origine, plugin, modello, reward e semantica SEA; nessun PPO/ex-novo avviato.

## Test e verifiche

- F0 finale: **399 controlli PASS** e `py_compile` sui 19 file del perimetro.
- Matrice F0: 58 job censiti e verificati; overlay finale e actor drift con verifica strict PASS.
- T1R: **3/3 job tecnicamente validi**, ma gate A FAIL; nominale 500 step/2 cicli, start estremi terminati a 80 e 45 step per penetrazione.
- Contatori FSM v3 nel T1R: nessun resync, HS cancellato o timeout nei tratti eseguiti; il problema dominante è stato il contatto prima della chiusura dei cicli sugli start estremi.

## TODO completati

- [x] Chiudere tecnicamente la Fase 0 e produrre il verdetto comportamentale su B0820.
- [x] Provare causalmente il deadlock del clock tramite isolamento con clock prescritto.
- [x] Eseguire e fermare fail-closed il primo bridge T1R dopo il fallimento del gate A.

## TODO aperti e propagati

- [ ] Ridisegnare il bridge Teacher–Student/DAgger per riprodurre il metodo riuscito di luglio usando però esclusivamente il parent imitativo V26 di agosto.
- [ ] Correggere o rendere univoca la semantica `cycle_valid` dopo una cancellazione HS; mantenere fail-closed i prodotti phase-aligned fino alla risoluzione.
- [ ] Misurare sigma nel lineage corrente prima delle recovery; non assumere automaticamente 0,005.
- [ ] Risolvere insieme forma cinematica, bootstrap FSM/clock e margine di penetrazione closed-loop sugli start non nominali.
- [ ] Nei futuri run RLlib registrare separatamente runner configurati/sani, batch corrente, lifetime e replacement.
- [ ] Generalizzazione multimodello EPIC, differita rispetto al recupero AB06.
- [ ] TODO tecnici/amministrativi ereditati: swing timeout 1,3 s; driver di qualifica versionato; probe schema a `train_ppo_mlp.py:1408`; pin V20, `best == last`, `fit_p2` V12R3 e deflessione SEA iniziale.

## Integrazione documentale retrospettiva — 2026-10-05

Queste integrazioni consolidano risultati e TODO dei report utente già esistenti. La data di registrazione è il 5 ottobre 2026: non sono nuove simulazioni, test, training o modifiche del codice. Gli esiti terminali storici restano distinti dai risultati dei successori.

<a id="integrazione-2026-10-05-2026-08-23_f1_s1_protocollo_tooling_dryrun"></a>

### 2026-08-23_f1_s1_protocollo_tooling_dryrun

Fonte: [2026-08-23_f1_s1_protocollo_tooling_dryrun](../user/2026-08-23_f1_s1_protocollo_tooling_dryrun.md).

I risultati F1/F2R devono essere separati per gate. L’isolamento aiso4 senza clock termina senza cicli e rende G3 non valutabile; aiso6 con clock prescritto produce due cicli per ciascuno dei tre start, provando il deadlock di bootstrap. **P0 osservabilità è PASS2/3fold** secondo il contratto congelato: minus e nominale PASS, +0.20 FAIL con mediana0.0566>0.05/p90entro limite. Non scatta il verdetto preregistrato di contratto stateless insufficiente; il fold difficile rimane però un limite esplicito.

Il precedente **T1 commissioning**, distinto dal T1R finale, arriva a500step su tutti gli start con3/1/2cicli, ma gateAFAIL: A5 nominale clock online4.68s>3.5s, A6 periodi/deriva sugli estremi. Anche forma/caviglia e alcuni gate penetrazione/reserve falliscono come misure informative. L’amenda strumentale usa deriva unwrapped anziché la differenza centrata che attraversava il wrap±0.5; corregge l’interpretazione della misura ma lascia FAIL per le violazioni reali dei periodi. Non viene promossa T1.

Il refit correttivo **T1R** preserva la funzione JUL_H0 con1.176righe dedicate e task2.994righe, knee`u_T` e ankle`u_IK`; le collisioni tra ruoli sono gestite dando priorità alla preservazione. P1–4/P3b offline PASS non sono autonomia: l’unico closed-loop finale produce nominale500step/2cicli, estremi80/45step a circa28.8mm, gateAFAIL e `unlock_t2=false`; T2/T3/aggregazione/PPO restano bloccati. Questo esito finale e B3−0.0099non≤−0.03 erano già nel daily.

Le verifiche sorgente includono pubblicazione transazionale/no-replace con marker di provenienza, fixture coerenti col produttore e preflight che mantiene integrity su tutti gli artefatti ma applica il vincolo di revisione ai job selezionati: la deriva del vecchio driverT1 non deve inventare una nuova esecuzione T1R. Suite, dry-run e audit chiudono tali correzioni; gli appendici non sono altri successi scientifici. I TODO di questo tentativo vengono inquadrati dal ridisegno successivo del metodo luglio; cycle_valid, osservabilità RLlib, misure sigma ed EPIC restano debiti distinti.

<a id="integrazione-2026-10-05-2026-08-23_fase0_gate_finale_recupero_ab06"></a>

### 2026-08-23_fase0_gate_finale_recupero_ab06

Fonte: [2026-08-23_fase0_gate_finale_recupero_ab06](../user/2026-08-23_fase0_gate_finale_recupero_ab06.md).

La diagnosi F0 comprendeva anche la scala stocastica: B0820 eredita una testa log-std state-dependent con **sigma≈0.5**, mentre luglio usava sigma costante0.005. Il drift medio H0→best è piccolo, RMSEcirca0.0052–0.0063, ma la KL è minuscola(1.26e−4–1.90e−4) proprio a quella grande sigma. Questo spiega perché50update non recuperano la forma e perché il confronto deterministico non esaurisce il limite dei rollout stocastici. Il successivo TODO di misurare sigma nelle nuove lineage non sostituisce questa misura forense di B0820.

La decisione storica è **15luglio baseline di rollback**. Il controllo39D V26 mostra che la qualità è possibile con i target privilegiati; non è una policy35D deployable né una promozione del controllo. F0 conclude PASS tecnico58job/399controlli/19compile ma FAIL scientifico B0820, con arresto prima di F1; la F1 iniziata dopo nuova autorizzazione appartiene alla milestone successiva dello stesso23/08. L’ambiguità cycle_valid mantiene i prodotti phase-aligned fail-closed; RLlib ed EPIC rimangono TODO, senza nuovo training in questa integrazione.

<!-- reconciled-report-todos-2026-10-05:start -->
## TODO riconciliati e propagati — integrazione del 5 ottobre 2026

Ricostruzione documentale dello stato a questa data. Le voci sotto precisano i TODO storici ripetuti nelle sezioni precedenti: una nuova esecuzione o un PASS su un altro ramo non chiude il debito originario. Le attività condizionate restano proposte; questa integrazione non avvia esperimenti. Le chiusure riportano una fonte esplicita.

### Attività aperte

- [ ] Riferimento storico al TODO giugno sulla scelta warm-start: luglio ha usato il 23 giugno grfsoft, non il 24 giugno. — [fonte 2026-08-25, riga 40](../user/2026-08-25_ricostruzione_storica_architetture_actor_e_tentativi_rete_larga.md).
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

### Attività condizionate e vincoli

- [ ] **Vincolo:** Per ogni candidata, eseguire qualification scientifica distinta dall’integrità tecnica del training; questa regola non è un’attestazione di robustezza generale. — [fonte 2026-08-14, riga 215](../user/2026-08-14_h0_v12r9_training_readiness_preflight_monitor.md).
- [ ] **Condizionato:** Probe `train_ppo` (39/43): correggere solo con migrazione di schema dichiarata (lineage futura), non per i training comparabili con giugno. — [fonte 2026-08-18, riga 146](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md).

### Chiusure documentate e consolidamento retrospettivo

- [x] Daily 2026-08-16 e 2026-08-17 da consolidare al prossimo `end_day`. — [fonte 2026-08-17, riga 121](../user/2026-08-17_diagnosi_divergenza_luglio_v26_e_proposta_retraining_imitativo_nativo.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-17_daily-report.md)).
- [x] Daily 2026-08-16 da consolidare al prossimo `end_day` (14 user report della giornata). — [fonte 2026-08-17, riga 95](../user/2026-08-17_h0_v12r17_verdetto_e_probe_capacita.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-17_daily-report.md)).
- [x] Daily 2026-08-16/17/18 da consolidare al prossimo `end_day`. — [fonte 2026-08-18, riga 154](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-18_daily-report.md)).
- [x] Daily 2026-08-18/19 da consolidare al prossimo `end_day`. — [fonte 2026-08-19, riga 159](../user/2026-08-19_indagine_iterazioni_lente_crash_storm_reject_continue_fix.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-19_daily-report.md)).
- [x] Daily 2026-08-18/19/20 da consolidare al prossimo `end_day`. — [fonte 2026-08-20, riga 104](../user/2026-08-20_audit_equivalenza_giugno_per_fresh_run_nativo_v26.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-20_daily-report.md)).
- [x] Lancio del training ex-novo B0820 (durata/N a scelta utente). — [fonte 2026-08-20, riga 92](../user/2026-08-20_risultato_training_ready_exnovo_B0820.md). **Chiuso:** F0 finale:58job/overlay/actor drift e399controlli PASS; B0820 scientificamente FAIL. ([evidenza](2026-08-23_daily-report.md)).
- [x] Durante il run: monitorare la discesa di `phase_timeout_stance` e `morphology_causal_contract_failure` (i due muri policy-dipendenti). — [fonte 2026-08-20, riga 93](../user/2026-08-20_risultato_training_ready_exnovo_B0820.md). **Chiuso:** F0 finale:58job/overlay/actor drift e399controlli PASS; B0820 scientificamente FAIL. ([evidenza](2026-08-23_daily-report.md)).
- [x] Ricetta ex-novo: lr 5e-7 × 1 epoca era tarata sulla fragilità di H0; la baseline B0820 ha margini ampi — candidata a un regime più deciso sotto KL-guard (da decidere coi numeri del primo run pulito). — [fonte 2026-08-21, riga 173](../user/2026-08-21_report_approfondito_dal_training_imitativo_v26_alla_fsm_v3.md). **Superato:** F0 finale:58job/overlay/actor drift e399controlli PASS; B0820 scientificamente FAIL. ([evidenza](2026-08-23_daily-report.md)).
- [x] Training ex-novo B0820 con v3 + corridoio corretto: la prima misura onesta della ricetta (i 50 update precedenti erano troncati dal bug di tolleranza). Da lanciare su tua decisione. — [fonte 2026-08-21, riga 170](../user/2026-08-21_report_approfondito_dal_training_imitativo_v26_alla_fsm_v3.md). **Chiuso:** F0 finale:58job/overlay/actor drift e399controlli PASS; B0820 scientificamente FAIL. ([evidenza](2026-08-23_daily-report.md)).
- [x] Nuovo re-audit indipendente della settima correzione (sezione 18; la sesta è in sezione 17); solo dopo, su decisione del gate owner: eventuale esecuzione di `stoch` (27 job) e `ctrl39` (3 job) con receipt schema 5, poi l'analizzatore sui dati reali e le attività 6–8 (overlay, deriva actor). Nessun training. **Stato: STOP, in attesa di re-audit.** — [fonte 2026-08-22, riga 291](../user/2026-08-22_fase0_freeze_inventario_gate_intermedio.md). **Chiuso:** F0 finale:58job/overlay/actor drift e399controlli PASS; B0820 scientificamente FAIL. ([evidenza](2026-08-23_daily-report.md)).
- [x] **S2 per micro-gate del gate owner:** stage 1 — cross-check adapter **PASS** (r3, §9.4), derivazione C/A_iso_s005 **eseguita** (§9.5), micro-smoke A_iso det nominale **eseguito e auditato, audit Codex PASS** (§10); micro-gate `aiso_det` **eseguito** (minus020, plus020; nominale riusato): predicato di successo A_iso sui 3 start **FALSE** (§11) → per protocollo `G3 = NOT_EVALUABLE_AISO_FAILED`, nessuna attribuzione al drop, ridisegno F2 da decidere con il gate owner; le altre famiglie di stage 1 (incluso il diagnostico `aiso_clk_diag`), stage 2 (dataset, refit), stage 3 (12 rollout D) e stage 4 (analisi e gate) non sono avviate e richiedono una decisione esplicita. Ordine e comandi in §5; le receipt e il preflight rifiutano qualunque esecuzione con moduli pendenti, input drift o seed sigillati. — [fonte 2026-08-23, riga 108](../user/2026-08-23_f1_s1_protocollo_tooling_dryrun.md). **Superato:** Aiso senza clock non completa cicli; clock prescritto completa due cicli; bridge F2R/T1R poi FAIL. ([evidenza](2026-08-23_daily-report.md)).

<!-- reconciled-report-todos-2026-10-05:end -->
