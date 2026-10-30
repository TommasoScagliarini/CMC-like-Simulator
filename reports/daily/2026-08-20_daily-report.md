# Daily Report - 2026-08-20

> Aggiornamento successivo del 2026-10-19: i riferimenti di questo documento
> alla filiera V12, al bundle V26C, alla campagna sigma o ai file nella radice
> di `baseline_MLP/validation/` sono storici: quei materiali sono stati rimossi.
> La workspace V26C e i suoi checkpoint sono conservati. Per rieseguire
> le procedure con input o prerequisiti rimossi occorre il loro ripristino.
> Vedere il [registro della pulizia](../cleanup/2026-10-19_baseline_validation_removal/README.md).

> Aggiornamento del 2026-10-19: `Trajectory Generator/baseline_MLP/experimental_configs/`
> è stata rimossa. Due preset sono conservati come fixture dei test; gli altri
> preset e i comandi che li selezionavano sono storici e richiedono ripristino.
> La configurazione V26C corrente resta `training_exnovo_cfg.yaml`. Vedere il
> [registro della rimozione](../cleanup/2026-10-19_experimental_configs_removal/README.md).

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

- `2026-08-20_audit_equivalenza_giugno_per_fresh_run_nativo_v26.md`
- `2026-08-20_verdetto_fresh_run_june_equivalent_nativo_v26.md`
- `2026-08-20_runbook_catena_warm_start_imitativo_exnovo.md`
- `2026-08-20_risultato_training_ready_exnovo_B0820.md`

## Sintesi

**Audit di equivalenza** giugno ↔ fresh run: reward effettiva identica,
PPO identico, convenzione osservazioni identica (il bug del probe esisteva
già a giugno: modulo 31 su 35); due derive scovate e decise dall'utente —
soglie di penetrazione **20/28 mm** come giugno e profilo GRF applicato di
giugno (ripristinato da git; poi scoperto byte-identico a
`grf_correct_magnitude` dell'area ex-novo). **Fresh run june-equivalent**:
100/100 in 14,4 h, 0 off-grid, ultime 20 iter 257,4 vs 268,6 (96 %), len
494 vs 476, metà delle penetrazioni di giugno; rollout del best (iter 87)
return 363,8, orizzonte pieno, 7 figure piene, smorzamento del ginocchio
sparito; coppia SEA positiva alla caviglia in loading response spiegata
dalla heel-qualification e accettata come baseline. **Catena B0820**:
guardie di processo → terminazioni pulite (9/9 test), manifest actor,
critic warmup (5 iter, `vf_loss` 8,86→0,07), qualifica 3/3 fisica sicura
(stance-lock a ~3 s come limite), checkpoint-zero, corridor readiness 8/8,
preflight smoke pulito → **training-ready**. Runbook riproducibile della
catena; canonico `training_exnovo_cfg.yaml` promosso al contenuto B0820
(scelta utente b). Lanciato il training ex-novo B0820 da 50 iterazioni.

## Lavoro svolto

- `simulation_runner.py` (`OnlineGRFPenetrationLimitExceeded`),
  `osim_trj_cmc_like.py` (`grf_penetration_hard`, catch terminale
  BinaryPhaseTransferError), `binary_phase_adapter_v26.py` (3 raise
  tipizzati), `rollout_eval.py` (passthrough invalid-policy),
  `validation/test_process_guards_termination.py`.
- Config: `experimental_configs/exnovo_v26_B0820_chain.yaml`; canonico
  promosso; manifest actor nei moduli della baseline.
- Figure: `plot/08_20_2026_2_june_equiv_fresh_iter87_best`,
  `plot/08_20_2026_3_morphology_corridor_binary_v26` (+ script versionato
  `validation/plot_morphology_corridor_binary_v26.py`).

## Test e verifiche

- Suite guardie+policy 9/9; readiness corridoio 8/8; ruff; audit
  riproducibile da yaml/commit/pickle; SHA dei profili GRF.

## TODO completati

- [x] Smoke di equivalenza, fresh run 100 iter, confronto formale.
- [x] Rollout + 7 plot del best; catena di luglio sulla baseline nativa.
- [x] Passthrough invalid-policy in `rollout_eval`.
- [x] Decisione guardie di processo (approvata e implementata).

## TODO aperti e propagati

- [ ] Esito del training ex-novo B0820 da 50 iter (vedi 21/08).
- [ ] Promuovere il driver di qualifica da scratchpad a script versionato.
- [ ] Fix probe (condizionato alla migrazione di schema).
- [ ] Amministrativo (dal 15/08): pin V20, anomalia best==last, metrica
  `fit_p2` V12R3, deflessione SEA iniziale (storico).

## Integrazione documentale retrospettiva — 2026-10-05

Queste integrazioni consolidano risultati e TODO dei report utente già esistenti. La data di registrazione è il 5 ottobre 2026: non sono nuove simulazioni, test, training o modifiche del codice. Gli esiti terminali storici restano distinti dai risultati dei successori.

<a id="integrazione-2026-10-05-2026-08-20_risultato_training_ready_exnovo_B0820"></a>

### 2026-08-20_risultato_training_ready_exnovo_B0820

Fonte: [2026-08-20_risultato_training_ready_exnovo_B0820](../user/2026-08-20_risultato_training_ready_exnovo_B0820.md).

La catena B0820, il run50 e il verdetto successivo sono già consolidati. Il TODO **promuovere il driver di qualifica da scratchpad a script versionato alla prossima esecuzione della catena** resta aperto in assenza di una chiusura specifica: compare nei daily20–23, poi non è ripetuto esplicitamente. Il fatto che siano state costruite altre catene non attesta da solo questa promozione. È un debito di riproducibilità del driver, non un ordine di rilanciare la B0820 scientificamente fallita.

Gli altri TODO della milestone sono risolti o superati: il training50 viene lanciato, monitorato e valutato; F0 conclude che B0820 non è baseline AB06 valida. Questa integrazione ripristina il tracciamento documentale del driver; non modifica config, pesi, guardie o procedure di esecuzione.

<a id="integrazione-2026-10-05-2026-08-20_runbook_catena_warm_start_imitativo_exnovo"></a>

### 2026-08-20_runbook_catena_warm_start_imitativo_exnovo

Fonte: [2026-08-20_runbook_catena_warm_start_imitativo_exnovo](../user/2026-08-20_runbook_catena_warm_start_imitativo_exnovo.md).

Il runbook standard e l’esecuzione B0820 sono già descritti nel daily. La procedura indicava un driver parametrico `scratchpad/run_qualification_B0820.py`, con input checkpoint/config/offset, da promuovere a script versionato alla prossima catena. **Questo TODO resta aperto**: non si trova una promozione o supersessione esplicita entro il daily01/10; un nuovo protocollo di altra lineage non è prova di aver versionato quel driver.

Restano validi i limiti storici della procedura: qualification fisica non richiedeva full horizon e il checkpoint-zero era il warmup-last, non un artificiale azzeramento del progresso. La readiness tecnica B0820 non certificava la forma della policy; F0 del23/08 ne documenta poi il FAIL scientifico. Nessuna nuova esecuzione è autorizzata o avvenuta tramite questa integrazione.

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

### Attività condizionate e vincoli

- [ ] **Vincolo:** Per ogni candidata, eseguire qualification scientifica distinta dall’integrità tecnica del training; questa regola non è un’attestazione di robustezza generale. — [fonte 2026-08-14, riga 215](../user/2026-08-14_h0_v12r9_training_readiness_preflight_monitor.md).
- [ ] **Condizionato:** Probe `train_ppo` (39/43): correggere solo con migrazione di schema dichiarata (lineage futura), non per i training comparabili con giugno. — [fonte 2026-08-18, riga 146](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md).

### Chiusure documentate e consolidamento retrospettivo

- [x] Decisione dell'utente sul retraining imitativo nativo V26 (raccomandato), incluse: autorizzazione alla modifica runtime «invalid event → terminazione episodio» in modalità training. — [fonte 2026-08-17, riga 113](../user/2026-08-17_diagnosi_divergenza_luglio_v26_e_proposta_retraining_imitativo_nativo.md). **Chiuso:** Percorso nativo e catena B0820 eseguiti, senza garantire la qualità scientifica finale. ([evidenza](2026-08-20_daily-report.md)).
- [x] In caso di via libera: (i) modifica runtime + test, (ii) riesecuzione ricetta imitativa di giugno con V26 attivo (monitorata), (iii) catena di luglio sul risultato fino al comando di training ex-novo + corridor. — [fonte 2026-08-17, riga 116](../user/2026-08-17_diagnosi_divergenza_luglio_v26_e_proposta_retraining_imitativo_nativo.md). **Chiuso:** Percorso nativo e catena B0820 eseguiti, senza garantire la qualità scientifica finale. ([evidenza](2026-08-20_daily-report.md)).
- [x] Daily 2026-08-16 e 2026-08-17 da consolidare al prossimo `end_day`. — [fonte 2026-08-17, riga 121](../user/2026-08-17_diagnosi_divergenza_luglio_v26_e_proposta_retraining_imitativo_nativo.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-17_daily-report.md)).
- [x] Daily 2026-08-16 da consolidare al prossimo `end_day` (14 user report della giornata). — [fonte 2026-08-17, riga 95](../user/2026-08-17_h0_v12r17_verdetto_e_probe_capacita.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-17_daily-report.md)).
- [x] Daily 2026-08-16/17/18 da consolidare al prossimo `end_day`. — [fonte 2026-08-18, riga 154](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-18_daily-report.md)).
- [x] Aggiungere passthrough binary_phase_invalid_event_policy a rollout_eval.py. — [fonte 2026-08-18, riga 149](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md). **Chiuso:** Passthrough invalid-policy in rollout_eval completato. ([evidenza](2026-08-20_daily-report.md)).
- [x] A baseline raggiunta: catena di luglio (critic warmup → qualifica → checkpoint-zero → morphology → training ex-novo + corridor). — [fonte 2026-08-18, riga 152](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md). **Chiuso:** Percorso nativo e catena B0820 eseguiti, senza garantire la qualità scientifica finale. ([evidenza](2026-08-20_daily-report.md)).
- [x] Daily 2026-08-18/19 da consolidare al prossimo `end_day`. — [fonte 2026-08-19, riga 159](../user/2026-08-19_indagine_iterazioni_lente_crash_storm_reject_continue_fix.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-19_daily-report.md)).
- [x] Decisione utente: fresh run (A) vs reward shaping (B); per (A) applicare prima il fix probe (2 righe + verifica n_actor=43). — [fonte 2026-08-19, riga 68](../user/2026-08-19_verdetto_lineage_nativa_v26_a_200_confronto_giugno.md). **Chiuso:** Scelta fresh, audit June-equivalence e run100 completati; fix probe deliberatamente differito. ([evidenza](2026-08-20_daily-report.md)).
- [x] Verdetto smoke equivalenza (in corso). — [fonte 2026-08-20, riga 99](../user/2026-08-20_audit_equivalenza_giugno_per_fresh_run_nativo_v26.md). **Chiuso:** Fresh benchmark/rollout con sette plot e catena B0820 completati. ([evidenza](2026-08-20_daily-report.md)).
- [x] Daily 2026-08-18/19/20 da consolidare al prossimo `end_day`. — [fonte 2026-08-20, riga 104](../user/2026-08-20_audit_equivalenza_giugno_per_fresh_run_nativo_v26.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-20_daily-report.md)).
- [x] Fresh run 100 iter + confronto formale finale coi criteri di giugno. — [fonte 2026-08-20, riga 100](../user/2026-08-20_audit_equivalenza_giugno_per_fresh_run_nativo_v26.md). **Chiuso:** Fresh benchmark/rollout con sette plot e catena B0820 completati. ([evidenza](2026-08-20_daily-report.md)).
- [x] Decisione differita: guardie di processo → terminazioni pulite (30 mm e AIR-latch) per campionamento esatto 4096. — [fonte 2026-08-20, riga 90](../user/2026-08-20_verdetto_fresh_run_june_equivalent_nativo_v26.md). **Chiuso:** Guardie tipizzate e terminazioni pulite implementate e verificate. ([evidenza](2026-08-20_daily-report.md)).
- [x] Con la baseline nativa V26 stabilita: catena di luglio (critic warmup → qualifica → checkpoint-zero → morphology → ex-novo + corridor). — [fonte 2026-08-20, riga 92](../user/2026-08-20_verdetto_fresh_run_june_equivalent_nativo_v26.md). **Chiuso:** Fresh benchmark/rollout con sette plot e catena B0820 completati. ([evidenza](2026-08-20_daily-report.md)).
- [x] Rollout + 7 plot del best e confronto visivo con `06_24` (in corso). — [fonte 2026-08-20, riga 89](../user/2026-08-20_verdetto_fresh_run_june_equivalent_nativo_v26.md). **Chiuso:** Fresh benchmark/rollout con sette plot e catena B0820 completati. ([evidenza](2026-08-20_daily-report.md)).

<!-- reconciled-report-todos-2026-10-05:end -->
