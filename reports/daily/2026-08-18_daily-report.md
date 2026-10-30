# Daily Report - 2026-08-18

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

- `2026-08-18_training_imitativo_nativo_v26_100iter_verdetto.md`
- `2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md`

## Sintesi

Il training imitativo nativo V26 completa 100/100 senza crash: tracking per
step ≈ 88 % di giugno, ma episodi dimezzati (~250 step) da un muro unico,
`phase_timeout_swing` (1,3 s senza HS heel-qualified), con tetto secondario
di eventi invalidi. Il rollout del best viene rifiutato dal validator di
schema (39 vs 43): **bug del probe** di `train_ppo_mlp` (dimensiona l'actor
saltando `make_cmc_env`), effetto benigno (persi solo `u_abs/u_saturated`,
ridondanti); rollout eseguito con bypass fedele e 7 plot prodotti
(`plot/08_18_2026_1_...`, fig. 3/4 vuote: nessun ciclo qualificato completo).
Avviata l'opzione (A): swing 2,6 s (via reward JSON, già plumbato) +
`reject_continue`; smoke 10 iter letto come PASS (len fino a 383, best 177)
e lanciata la continuazione a 200. *Nota a posteriori (19/08): il PASS
dello smoke era parzialmente un artefatto — gli episodi con evento invalido
uccidevano il worker invece di terminare.*

## Lavoro svolto

- Diagnosi empirica del mismatch di schema (diff dei nomi feature tra env
  di training e di rollout; larghezze dal pickle del modulo).
- Driver di rollout con bypass dichiarato del validator; plot canonici.
- `experimental_configs/reward_swing_timeout_2p6.json`; smoke + continuazione.

## Test e verifiche

- Rollout `ok: true` (return 136,9, fine per `phase_timeout:swing`,
  penetrazione max 14,8 mm); contatori del training letti dai jsonl reali.

## TODO completati

- [x] Verdetto training nativo 100 iter e confronto coi criteri di giugno.
- [x] Rollout del best + 7 plot.

## TODO aperti e propagati

- [ ] Verdetto onesto della leva (A) (poi rivisto il 19/08).
- [ ] Fix del probe `train_ppo_mlp.py:1408`: solo per training *fresh*
  con migrazione di schema dichiarata (decisione 20/08: non per i training
  comparabili con giugno, che condividono la stessa convenzione di slicing).
- [ ] Passthrough di `binary_phase_invalid_event_policy` in `rollout_eval`.
- [ ] A baseline raggiunta: catena di luglio.
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
- [ ] Dopo il fix: smoke 10 iter dal checkpoint, poi continuazione e ri-confronto sui criteri; a pari lunghezza episodi, valutare anche il rollout di tracking (RMSE per giunto) contro i plot `06_24`. — [fonte 2026-08-18, riga 93](../user/2026-08-18_training_imitativo_nativo_v26_100iter_verdetto.md).
- [ ] Verdetto smoke 10 iter (episode_len > 250?) → continuazione piena opzione (A) e ri-confronto coi criteri di giugno. — [fonte 2026-08-18, riga 144](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md).
- [ ] Decisione utente sulla leva (A) (timeout swing in training + `reject_continue`) e sulla modalità di prosecuzione (resume vs fresh). — [fonte 2026-08-18, riga 91](../user/2026-08-18_training_imitativo_nativo_v26_100iter_verdetto.md).
- [ ] A baseline raggiunta: catena di luglio (critic warmup → qualifica → checkpoint-zero → morphology → training ex-novo + corridor). — [fonte 2026-08-18, riga 152](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md).
- [ ] Valutare un override esplicito dello schema in rollout_eval.py per il checkpoint39/43. — [fonte 2026-08-18, riga 149](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md).

### Attività condizionate e vincoli

- [ ] **Vincolo:** Per ogni candidata, eseguire qualification scientifica distinta dall’integrità tecnica del training; questa regola non è un’attestazione di robustezza generale. — [fonte 2026-08-14, riga 215](../user/2026-08-14_h0_v12r9_training_readiness_preflight_monitor.md).
- [ ] **Condizionato:** Probe `train_ppo` (39/43): correggere solo con migrazione di schema dichiarata (lineage futura), non per i training comparabili con giugno. — [fonte 2026-08-18, riga 146](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md).

### Chiusure documentate e consolidamento retrospettivo

- [x] Daily 2026-08-16 e 2026-08-17 da consolidare al prossimo `end_day`. — [fonte 2026-08-17, riga 121](../user/2026-08-17_diagnosi_divergenza_luglio_v26_e_proposta_retraining_imitativo_nativo.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-17_daily-report.md)).
- [x] Daily 2026-08-16 da consolidare al prossimo `end_day` (14 user report della giornata). — [fonte 2026-08-17, riga 95](../user/2026-08-17_h0_v12r17_verdetto_e_probe_capacita.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-17_daily-report.md)).
- [x] Daily 2026-08-16/17/18 da consolidare al prossimo `end_day`. — [fonte 2026-08-18, riga 154](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-18_daily-report.md)).

<!-- reconciled-report-todos-2026-10-05:end -->
