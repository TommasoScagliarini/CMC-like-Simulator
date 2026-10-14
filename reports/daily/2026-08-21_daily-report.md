# Daily Report - 2026-08-21

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

- `2026-08-21_fsm_v3_resync_validazione_e_bug_tolleranza_corridoio.md`
- `2026-08-21_report_approfondito_dal_training_imitativo_v26_alla_fsm_v3.md`
- Piani: `reports/plans/2026-08-21_piano_test_fsm_v3_resync.md`,
  `reports/plans/2026-08-21_design_fsm_v3_resync.md`

## Sintesi

Il training ex-novo B0820 da 50 iter è pulito ma **piatto** (return −18 →
−35, muri costanti); il best allo start nominale è identico al
pre-training. Riformulazione: è l'esito tecnico del pilot del 15/07, il cui
H0 però già camminava agli start esatti grazie all'adattamento
supervisionato. L'utente rifiuta il tampone (fine-tune della policy) e
chiede robustezza architetturale: **FSM attore v3** (resync su
contraddizione col latch funzionale del detector, cancellazione HS
rimbalzato, bootstrap verificato), design approvato e validata su 4 livelli
— L0 23/23 con golden v2 byte-identico; L1 replay fedele 7/7 con desync
da ~2 s a ≤ 80 ms; L2 23 start × 2 versioni senza errori, v3 ≡ v2; L3 che
smaschera sotto la FSM il ledger causale del corridoio: ri-armo sui repair
e **bug preesistente di tolleranza temporale (1e-12 vs drift 1,1e-12 tra
due orologi float)**, verosimile causa del 43 % di contract failure anche
in v2. Esito: L3a 3/3 start a orizzonte pieno; L3b v3 contract failure +0,
stance timeout +0, +20 episodi a orizzonte (controllo v2 sullo stesso
corridoio: +5/+12/+7). Decisioni utente applicate: **v3 default**,
**candidato corridoio V26 promosso post-Q2** (test di governance riscritto,
guard runtime preservato), **nessun training lanciato**. Repo a zero test
rossi (57/57 + 101/102→102/102).

## Lavoro svolto

- `prosthetic_phase_fsm.py` (v3), `binary_phase_adapter.py` (latch,
  validatore tipizzato), `experimental_morphology_corridor.py` (repair
  whitelist, ri-armo causale, `_TIME_EPS` 1e-9, contatore repair),
  `osim_trj_cmc_like.py` / `training_config.py` / `train_ppo_mlp.py` /
  `rollout_eval.py` (chiave `binary_phase_actor_fsm_version`, default v3,
  telemetria `morphology_causal_diagnostics` nei trace).
- Test: `validation/test_prosthetic_phase_fsm_v3_resync.py`,
  `validation/test_causal_ledger_fsm_repair.py`, golden e scenari
  condivisi; readiness corridoio aggiornata al contratto promosso.
- Strumenti: `validation/replay_actor_fsm_v3_on_traces.py`,
  `validation/sweep_fsm_v3_prescribed_starts.py` (+ receipt).
- Config: canonico con v3; snapshot B0820 esplicitamente v2; candidato V26
  promosso. Figure corridoio: `plot/08_21_2026_1_*`, `plot/08_21_2026_2_*`.
- Commit presente in git: `c8d2ddca` "new warm start with v26" (utente).

## Test e verifiche

- Unit: FSM/adapter/corridoio 102/102 dopo la promozione; live env 36/36;
  receipt L1/L2; L3 su env reale (rollout e smoke) con log Ray a 0 morti.

## TODO completati

- [x] Esito training ex-novo 50 iter (piatto) e diagnosi.
- [x] Design, test ladder e validazione L0–L3 della FSM v3.
- [x] Decisioni: v3 default; promozione candidato V26 (opzione ii).

## TODO aperti e propagati

- [ ] Training ex-novo B0820 con v3 + corridoio corretto (prima misura
  onesta della ricetta) — su decisione utente; valutare con quei numeri un
  regime lr/epoche più deciso sotto KL-guard.
- [ ] Follow-up v3: riportare lo swing timeout da 2,6 a 1,3 s.
- [ ] Promuovere il driver di qualifica da scratchpad a script versionato.
- [ ] Fix probe `train_ppo_mlp.py:1408` solo con migrazione di schema
  dichiarata.
- [ ] Commit checkpoint del lavoro FSM v3 / corridoio / governance (verificare
  cosa resta fuori da `c8d2ddca`).
- [ ] Amministrativo (dal 15/08): pin V20, anomalia best==last, metrica
  `fit_p2` V12R3, deflessione SEA iniziale (storico).

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

### Attività condizionate e vincoli

- [ ] **Vincolo:** Per ogni candidata, eseguire qualification scientifica distinta dall’integrità tecnica del training; questa regola non è un’attestazione di robustezza generale. — [fonte 2026-08-14, riga 215](../user/2026-08-14_h0_v12r9_training_readiness_preflight_monitor.md).
- [ ] **Condizionato:** Probe `train_ppo` (39/43): correggere solo con migrazione di schema dichiarata (lineage futura), non per i training comparabili con giugno. — [fonte 2026-08-18, riga 146](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md).

### Chiusure documentate e consolidamento retrospettivo

- [x] Daily 2026-08-16 e 2026-08-17 da consolidare al prossimo `end_day`. — [fonte 2026-08-17, riga 121](../user/2026-08-17_diagnosi_divergenza_luglio_v26_e_proposta_retraining_imitativo_nativo.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-17_daily-report.md)).
- [x] Daily 2026-08-16 da consolidare al prossimo `end_day` (14 user report della giornata). — [fonte 2026-08-17, riga 95](../user/2026-08-17_h0_v12r17_verdetto_e_probe_capacita.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-17_daily-report.md)).
- [x] Daily 2026-08-16/17/18 da consolidare al prossimo `end_day`. — [fonte 2026-08-18, riga 154](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-18_daily-report.md)).
- [x] Daily 2026-08-18/19 da consolidare al prossimo `end_day`. — [fonte 2026-08-19, riga 159](../user/2026-08-19_indagine_iterazioni_lente_crash_storm_reject_continue_fix.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-19_daily-report.md)).
- [x] Daily 2026-08-18/19/20 da consolidare al prossimo `end_day`. — [fonte 2026-08-20, riga 104](../user/2026-08-20_audit_equivalenza_giugno_per_fresh_run_nativo_v26.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-20_daily-report.md)).
- [x] Controllo v2 su corridoio corretto (L3b): +5 contract failure, +12 stance timeout — attribuzione chiusa. — [fonte 2026-08-21, riga 91](../user/2026-08-21_fsm_v3_resync_validazione_e_bug_tolleranza_corridoio.md). **Chiuso:** Il report marca esplicitamente il TODO come completato. ([evidenza](../user/2026-08-21_fsm_v3_resync_validazione_e_bug_tolleranza_corridoio.md)).
- [x] Decisione 3: nessun training lanciato. — [fonte 2026-08-21, riga 94](../user/2026-08-21_fsm_v3_resync_validazione_e_bug_tolleranza_corridoio.md). **Chiuso:** Il report marca esplicitamente il TODO come completato. ([evidenza](../user/2026-08-21_fsm_v3_resync_validazione_e_bug_tolleranza_corridoio.md)).
- [x] Decisione 1: v3 default (EnvCfg, CLI trainer/rollout, canonico `binary_phase_actor_fsm_version: v3`; snapshot B0820 esplicitamente v2). — [fonte 2026-08-21, riga 92](../user/2026-08-21_fsm_v3_resync_validazione_e_bug_tolleranza_corridoio.md). **Chiuso:** Il report marca esplicitamente il TODO come completato. ([evidenza](../user/2026-08-21_fsm_v3_resync_validazione_e_bug_tolleranza_corridoio.md)).
- [x] Esito +0,20 (L3a): orizzonte pieno. — [fonte 2026-08-21, riga 90](../user/2026-08-21_fsm_v3_resync_validazione_e_bug_tolleranza_corridoio.md). **Chiuso:** Il report marca esplicitamente il TODO come completato. ([evidenza](../user/2026-08-21_fsm_v3_resync_validazione_e_bug_tolleranza_corridoio.md)).
- [x] Decisione 2 (opzione ii): candidato V26 promosso post-Q2 (`status`, autorizzazioni, `morphology_causal_allow_effects 1.0`, peso 0,0025) e test di governance riscritto sul contratto promosso (guard Q2 runtime preservato). — [fonte 2026-08-21, riga 93](../user/2026-08-21_fsm_v3_resync_validazione_e_bug_tolleranza_corridoio.md). **Chiuso:** Il report marca esplicitamente il TODO come completato. ([evidenza](../user/2026-08-21_fsm_v3_resync_validazione_e_bug_tolleranza_corridoio.md)).

<!-- reconciled-report-todos-2026-10-05:end -->
