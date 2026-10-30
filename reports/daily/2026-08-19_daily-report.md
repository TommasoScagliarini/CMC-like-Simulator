# Daily Report - 2026-08-19

> Aggiornamento successivo del 2026-10-19: i riferimenti di questo documento
> alla filiera V12, al bundle V26C, alla campagna sigma o ai file nella radice
> di `baseline_MLP/validation/` sono storici: quei materiali sono stati rimossi.
> La workspace V26C e i suoi checkpoint sono conservati. Per rieseguire
> le procedure con input o prerequisiti rimossi occorre il loro ripristino.
> Vedere il [registro della pulizia](../cleanup/2026-10-19_baseline_validation_removal/README.md).

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

- `2026-08-19_indagine_iterazioni_lente_crash_storm_reject_continue_fix.md`
- `2026-08-19_verdetto_lineage_nativa_v26_a_200_confronto_giugno.md`

## Sintesi

Su richiesta dell'utente (iterazioni a 36 min contro gli 8–15 di giugno)
l'indagine esclude costo per step, stati costosi, learner e config, e trova
nei log Ray una **tempesta di crash silenziosi** (1.044 errori off-grid,
639 file worker, ~30 morti/iterazione) causata da un **mio bug** nel
percorso `reject_continue` (cursore sensori avanzato senza commit delle FSM
→ `previous_time_s off-grid` allo step dopo). Corretto con il drop-mode
nell'adapter (commit dei candidati window-consistent) e un unico percorso
di commit nell'env; la regressione reale ha scoperto un **secondo bug
latente** (microstep degenere all'orizzonte, tolleranza 1e-12 vs accumulo
float), corretto col test dell'orizzonte a mezza finestra. Certificazione:
7,7 min/iter, 0 off-grid, primi episodi V26 a 500 step pieni. Revisione
onesta del verdetto smoke. Continuazioni 132→150→200 pulite: best 196,4,
poi plateau; **equivalenza con giugno non raggiunta** (65 % return, 84 %
len) per storia sporca della lineage; rollout del best a orizzonte pieno
(return 210,7) con ginocchio smorzato e caviglia scarica. Raccomandato il
fresh run con tutti i fix.

## Lavoro svolto

- `binary_phase_adapter.py` (`invalid_event_mode="drop"`),
  `binary_phase_adapter_v26.py` (passthrough), `osim_trj_cmc_like.py`
  (commit unico; orizzonte robusto); test di regressione "assorbe e
  sopravvive"; suite 6/6.
- Strumenti: probe per-step, watcher di fase, analisi timer tensorboard.
- Corse: verify3 (7,7 min/iter), cont150, cont200_final; rollout + 7 plot
  del best 200 (`plot/08_20_2026_1_imitation_native_v26_iter162_best`).

## Test e verifiche

- Suite invalid-event 6/6; regressione probe 2×500 step; 0 off-grid su
  tutte le sessioni Ray post-fix; durate iterazione da tensorboard.

## TODO completati

- [x] Causa delle iterazioni lente trovata e corretta; requisito 8–15 min
  certificato.
- [x] Continuazione a 200 e confronto formale coi criteri di giugno.
- [x] Rollout + 7 plot del best 200.

## TODO aperti e propagati

- [ ] Decisione fresh run vs reward shaping (poi: fresh, 20/08).
- [ ] Fix probe (condizionato alla migrazione di schema, vedi 18/08).
- [ ] Passthrough invalid-policy in `rollout_eval`.
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
- [ ] Decisione dell'utente sul retraining imitativo nativo V26 (raccomandato), incluse: autorizzazione alla modifica runtime «invalid event → terminazione episodio» in modalità training. — [fonte 2026-08-17, riga 113](../user/2026-08-17_diagnosi_divergenza_luglio_v26_e_proposta_retraining_imitativo_nativo.md).
- [ ] In caso di via libera: (i) modifica runtime + test, (ii) riesecuzione ricetta imitativa di giugno con V26 attivo (monitorata), (iii) catena di luglio sul risultato fino al comando di training ex-novo + corridor. — [fonte 2026-08-17, riga 116](../user/2026-08-17_diagnosi_divergenza_luglio_v26_e_proposta_retraining_imitativo_nativo.md).
- [ ] Aggiungere passthrough binary_phase_invalid_event_policy a rollout_eval.py. — [fonte 2026-08-18, riga 149](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md).
- [ ] A baseline raggiunta: catena di luglio (critic warmup → qualifica → checkpoint-zero → morphology → training ex-novo + corridor). — [fonte 2026-08-18, riga 152](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md).
- [ ] Valutare un override esplicito dello schema in rollout_eval.py per il checkpoint39/43. — [fonte 2026-08-18, riga 149](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md).
- [ ] Decisione utente: fresh run (A) vs reward shaping (B); per (A) applicare prima il fix probe (2 righe + verifica n_actor=43). — [fonte 2026-08-19, riga 68](../user/2026-08-19_verdetto_lineage_nativa_v26_a_200_confronto_giugno.md).

### Attività condizionate e vincoli

- [ ] **Vincolo:** Per ogni candidata, eseguire qualification scientifica distinta dall’integrità tecnica del training; questa regola non è un’attestazione di robustezza generale. — [fonte 2026-08-14, riga 215](../user/2026-08-14_h0_v12r9_training_readiness_preflight_monitor.md).
- [ ] **Condizionato:** Probe `train_ppo` (39/43): correggere solo con migrazione di schema dichiarata (lineage futura), non per i training comparabili con giugno. — [fonte 2026-08-18, riga 146](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md).

### Chiusure documentate e consolidamento retrospettivo

- [x] Daily 2026-08-16 e 2026-08-17 da consolidare al prossimo `end_day`. — [fonte 2026-08-17, riga 121](../user/2026-08-17_diagnosi_divergenza_luglio_v26_e_proposta_retraining_imitativo_nativo.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-17_daily-report.md)).
- [x] Daily 2026-08-16 da consolidare al prossimo `end_day` (14 user report della giornata). — [fonte 2026-08-17, riga 95](../user/2026-08-17_h0_v12r17_verdetto_e_probe_capacita.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-17_daily-report.md)).
- [x] Daily 2026-08-16/17/18 da consolidare al prossimo `end_day`. — [fonte 2026-08-18, riga 154](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-18_daily-report.md)).
- [x] Dopo il fix: smoke 10 iter dal checkpoint, poi continuazione e ri-confronto sui criteri; a pari lunghezza episodi, valutare anche il rollout di tracking (RMSE per giunto) contro i plot `06_24`. — [fonte 2026-08-18, riga 93](../user/2026-08-18_training_imitativo_nativo_v26_100iter_verdetto.md). **Chiuso:** Continuation fino a200 completata e valutata;72iter post-fix pulite, senza equivalenza della policy. ([evidenza](2026-08-19_daily-report.md)).
- [x] Verdetto smoke 10 iter (episode_len > 250?) → continuazione piena opzione (A) e ri-confronto coi criteri di giugno. — [fonte 2026-08-18, riga 144](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md). **Chiuso:** Continuation fino a200 completata e valutata;72iter post-fix pulite, senza equivalenza della policy. ([evidenza](2026-08-19_daily-report.md)).
- [x] Decisione utente sulla leva (A) (timeout swing in training + `reject_continue`) e sulla modalità di prosecuzione (resume vs fresh). — [fonte 2026-08-18, riga 91](../user/2026-08-18_training_imitativo_nativo_v26_100iter_verdetto.md). **Chiuso:** Continuation fino a200 completata e valutata;72iter post-fix pulite, senza equivalenza della policy. ([evidenza](2026-08-19_daily-report.md)).
- [x] Eventuale rollout + 7 plot del best 200 per confronto visivo. — [fonte 2026-08-19, riga 70](../user/2026-08-19_verdetto_lineage_nativa_v26_a_200_confronto_giugno.md). **Chiuso:** Rollout del best200 e directory dei sette plot documentati. ([evidenza](2026-08-19_daily-report.md)).
- [x] Ri-misura onesta della leva (A): vedi sezione sopra. — [fonte 2026-08-19, riga 154](../user/2026-08-19_indagine_iterazioni_lente_crash_storm_reject_continue_fix.md). **Chiuso:** Il report marca esplicitamente il TODO come completato. ([evidenza](../user/2026-08-19_indagine_iterazioni_lente_crash_storm_reject_continue_fix.md)).
- [x] Verifica 3 iter: 7,7 min/iter, 0 off-grid — requisito certificato. — [fonte 2026-08-19, riga 153](../user/2026-08-19_indagine_iterazioni_lente_crash_storm_reject_continue_fix.md). **Chiuso:** Il report marca esplicitamente il TODO come completato. ([evidenza](../user/2026-08-19_indagine_iterazioni_lente_crash_storm_reject_continue_fix.md)).
- [x] Daily 2026-08-18/19 da consolidare al prossimo `end_day`. — [fonte 2026-08-19, riga 159](../user/2026-08-19_indagine_iterazioni_lente_crash_storm_reject_continue_fix.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-19_daily-report.md)).
- [x] Prosecuzione 151→200 (~6,5–7 h) per il confronto formale coi criteri di giugno — richiede conferma utente. — [fonte 2026-08-19, riga 155](../user/2026-08-19_indagine_iterazioni_lente_crash_storm_reject_continue_fix.md). **Chiuso:** Continuation fino a200 completata e valutata;72iter post-fix pulite, senza equivalenza della policy. ([evidenza](2026-08-19_daily-report.md)).
- [x] Esito regressione probe per-step: PASS (2×500 step pieni). — [fonte 2026-08-19, riga 152](../user/2026-08-19_indagine_iterazioni_lente_crash_storm_reject_continue_fix.md). **Chiuso:** Il report marca esplicitamente il TODO come completato. ([evidenza](../user/2026-08-19_indagine_iterazioni_lente_crash_storm_reject_continue_fix.md)).

<!-- reconciled-report-todos-2026-10-05:end -->
