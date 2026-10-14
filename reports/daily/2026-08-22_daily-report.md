# Daily Report - 2026-08-22

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

- [Fase 0 — freeze, inventario e gate intermedio](../user/2026-08-22_fase0_freeze_inventario_gate_intermedio.md)
- [TODO — generalizzazione multimodello con EPIC](../user/2026-08-22_todo_generalizzazione_multimodello_epic.md)

## Sintesi

È stata avviata la Fase 0 del recupero della qualità AB06: freeze degli artefatti, inventario, ricostruzione della provenienza e preparazione della matrice di rollout comparabili. La revisione r3 ha chiuso il cross-check degli artefatti risolvibili con **23 PASS, 0 FAIL e 10 N/A motivati**; al termine del giorno il gate era ancora intermedio e il documento è stato poi formalmente superato dal gate finale del 23 agosto.

È stato anche chiarito il dubbio sul numero di environment del training B0820: non vi era un numero di environment in crescita. La configurazione usava **12 remote EnvRunner** e un batch effettivo di **4608 step = 12 × 384**; il valore crescente osservato era `num_env_steps_sampled_lifetime`, cioè un contatore cumulativo. Il valore 4096 apparteneva ad altre configurazioni, tra cui il training imitativo V26.

La generalizzazione è stata separata dal recupero AB06. Teacher, dati, IK/GRF, dinamica e Morphology Corridor correnti rendono la policy statisticamente dipendente da AB06; il metodo Teacher–Student/DAgger non è però intrinsecamente limitato a quel modello. È stato aperto un TODO dedicato che richiede una validazione cross-subject su un modello EPIC mai usato per teacher, DAgger, PPO o selezione degli iperparametri.

## Lavoro svolto

- Costruito il perimetro additivo `Trajectory Generator/baseline_MLP/validation/f0_freeze_2026-08-22/` per inventario, source closure, matrici, receipt e analisi fail-closed.
- Conservate immutabili le revisioni F0 superate e gli artefatti utente già presenti.
- Eseguite le famiglie iniziali `replay` e `det`; `stoch` e `ctrl39` erano ancora sospese a fine giornata.
- Formalizzato in un report autonomo il futuro processo di generalizzazione multimodello con dataset EPIC.
- Nessun peso, checkpoint, reward, plugin C++, modello OpenSim o semantica SEA è stato modificato; nessun training è stato lanciato.

## Test e verifiche

- Cross-check inventario r3: **23 PASS / 0 FAIL / 10 N/A**.
- Replay iniziali: **9/9**; rollout deterministici iniziali: **19/19**.
- Test progressivi su receipt, provenienza, ABI, chiusura runtime e no-clobber descritti nel report F0; output già prodotti verificati byte-identici durante le revisioni.

## TODO completati

- [x] Chiarire il significato dei contatori RLlib e del batch 4608 rispetto a 4096.
- [x] Separare formalmente il recupero AB06 dalla generalizzazione multimodello.
- [x] Congelare e censire gli artefatti necessari alla Fase 0.

## TODO aperti e propagati

- [ ] Completare l'audit F0, le famiglie `stoch`/`ctrl39`, la matrice completa, gli overlay e la deriva dell'attore.
- [ ] Valutare il recupero AB06 solo dopo la chiusura fail-closed della Fase 0; nessun DAgger o training prima del gate.
- [ ] Rendere osservabili separatamente nei futuri run RLlib: runner configurati/sani, batch corrente, contatore lifetime e generation/replacement.
- [ ] Chiarire la semantica `cycle_valid` dopo la cancellazione di un heel strike nella FSM v3.
- [ ] Generalizzazione multimodello con EPIC: progettare split per soggetto e superare un test su almeno un soggetto completamente held-out.
- [ ] TODO tecnici ereditati dal 21/08: swing timeout 1,3 s; driver di qualifica versionato; migrazione dichiarata del probe `train_ppo_mlp.py:1408`; verifica del commit FSM/corridoio.
- [ ] TODO amministrativi storici: pin V20, anomalia `best == last`, metrica `fit_p2` V12R3 e deflessione SEA iniziale.

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
- [ ] Lancio del training ex-novo B0820 (durata/N a scelta utente). — [fonte 2026-08-20, riga 92](../user/2026-08-20_risultato_training_ready_exnovo_B0820.md).
- [ ] Durante il run: monitorare la discesa di `phase_timeout_stance` e `morphology_causal_contract_failure` (i due muri policy-dipendenti). — [fonte 2026-08-20, riga 93](../user/2026-08-20_risultato_training_ready_exnovo_B0820.md).
- [ ] Ricetta ex-novo: lr 5e-7 × 1 epoca era tarata sulla fragilità di H0; la baseline B0820 ha margini ampi — candidata a un regime più deciso sotto KL-guard (da decidere coi numeri del primo run pulito). — [fonte 2026-08-21, riga 173](../user/2026-08-21_report_approfondito_dal_training_imitativo_v26_alla_fsm_v3.md).
- [ ] Training ex-novo B0820 con v3 + corridoio corretto: la prima misura onesta della ricetta (i 50 update precedenti erano troncati dal bug di tolleranza). Da lanciare su tua decisione. — [fonte 2026-08-21, riga 170](../user/2026-08-21_report_approfondito_dal_training_imitativo_v26_alla_fsm_v3.md).
- [ ] Follow-up v3 fuori scope: riportare lo swing timeout da 2,6 a 1,3 s. — [fonte 2026-08-21, riga 95](../user/2026-08-21_fsm_v3_resync_validazione_e_bug_tolleranza_corridoio.md).
- [ ] Qualificare generalizzazione oltre AB06 su almeno un soggetto/modello EPIC completamente held-out da teacher, DAgger/PPO, iperparametri e corridor; più trial dello stesso soggetto non bastano. — [fonte 2026-08-22, riga 15](../user/2026-08-22_todo_generalizzazione_multimodello_epic.md).
- [ ] Nuovo re-audit indipendente della settima correzione (sezione 18; la sesta è in sezione 17); solo dopo, su decisione del gate owner: eventuale esecuzione di `stoch` (27 job) e `ctrl39` (3 job) con receipt schema 5, poi l'analizzatore sui dati reali e le attività 6–8 (overlay, deriva actor). Nessun training. **Stato: STOP, in attesa di re-audit.** — [fonte 2026-08-22, riga 291](../user/2026-08-22_fase0_freeze_inventario_gate_intermedio.md).

### Attività condizionate e vincoli

- [ ] **Vincolo:** Per ogni candidata, eseguire qualification scientifica distinta dall’integrità tecnica del training; questa regola non è un’attestazione di robustezza generale. — [fonte 2026-08-14, riga 215](../user/2026-08-14_h0_v12r9_training_readiness_preflight_monitor.md).
- [ ] **Condizionato:** Probe `train_ppo` (39/43): correggere solo con migrazione di schema dichiarata (lineage futura), non per i training comparabili con giugno. — [fonte 2026-08-18, riga 146](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md).

### Chiusure documentate e consolidamento retrospettivo

- [x] Daily 2026-08-16 e 2026-08-17 da consolidare al prossimo `end_day`. — [fonte 2026-08-17, riga 121](../user/2026-08-17_diagnosi_divergenza_luglio_v26_e_proposta_retraining_imitativo_nativo.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-17_daily-report.md)).
- [x] Daily 2026-08-16 da consolidare al prossimo `end_day` (14 user report della giornata). — [fonte 2026-08-17, riga 95](../user/2026-08-17_h0_v12r17_verdetto_e_probe_capacita.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-17_daily-report.md)).
- [x] Daily 2026-08-16/17/18 da consolidare al prossimo `end_day`. — [fonte 2026-08-18, riga 154](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-18_daily-report.md)).
- [x] Daily 2026-08-18/19 da consolidare al prossimo `end_day`. — [fonte 2026-08-19, riga 159](../user/2026-08-19_indagine_iterazioni_lente_crash_storm_reject_continue_fix.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-19_daily-report.md)).
- [x] Daily 2026-08-18/19/20 da consolidare al prossimo `end_day`. — [fonte 2026-08-20, riga 104](../user/2026-08-20_audit_equivalenza_giugno_per_fresh_run_nativo_v26.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-20_daily-report.md)).

<!-- reconciled-report-todos-2026-10-05:end -->
