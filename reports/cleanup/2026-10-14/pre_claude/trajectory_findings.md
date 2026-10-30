# Analisi sola lettura — Trajectory Generator — 2026-10-14

Instruction check token: CMC_AGENT_OK_2026. Nessuna modifica nella workspace.
Metodo: lettura AGENTS, README/preset/entrypoint, parsing AST dei 971 .py non-runs, ricerca globale di import/path, analisi manifest, dimensioni filesystem, disassemblaggio sicuro pickle con pickletools (senza deserializzazione/esecuzione), hash di checkpoint chiave. Non eseguiti training o rollout.

## Risultato

Non ho identificato un file Python top-level che possa essere dichiarato eliminabile immediatamente senza effetti su runtime, test, CLI, pickle o verifiche storiche. Questo non vieta la pulizia: distingue una rimozione banale da una migrazione deliberata. La maggior parte del peso è negli artefatti, mentre i 73 .py direttamente nelle quattro directory production occupano soltanto 2.171.291 B (~2,17 MB). I 971 .py non-runs/risultati complessivi occupano 23.576.643 B.

Dati du, arrotondati: baseline_MLP/validation 4,6 GB; runs baseline 9,3 GB (rollout 6,9 GB, training 2,3 GB); unified_MLP 2,4 GB (training 1,0 GB, rollout 1,3 GB); unified_MLP_39 11 GB (training 3,8 GB, rollout 6,3 GB, validation 814 MB); validation condivisa recente 1,5 GB. Lo spazio recuperabile da codice inutile sarebbe molto piccolo rispetto all'archiviazione degli esperimenti.

## Snapshot: checkpoint attuale escluso da Git

`git ls-files -- 'Trajectory Generator/runs/training/MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-01'` restituisce zero file. `git check-ignore -v` su rl_module_best/module_state.pkl e checkpoint_last/metadata.json restituisce `.gitignore:16:Trajectory Generator/runs/*`. Anche unified35/unified39 runs sono ignorati da `runs/` (linea 14). Il commit non protegge tali file. Backup fuori dal working tree e verifica degli hash sono prerequisite per cancellare questi artefatti, indipendentemente dal commit.

## KEEP: 7 moduli condivisi

Prefisso `Trajectory Generator/`:

- `osim_trj_cmc_like.py`: env Gymnasium/OpenSim usato da tutte le reti.
- `prosthetic_phase_fsm.py`: FSM actor e contratti eventi.
- `binary_phase_fsm.py`, `binary_phase_fsm_v26.py`: il secondo eredita il primo (`binary_phase_fsm_v26.py:19`).
- `binary_phase_adapter.py`, `binary_phase_adapter_v26.py`: il secondo eredita/importa il primo (`binary_phase_adapter_v26.py:14`). La variante nuova non sostituisce il file storico.
- `prescribed_action.py`: oracle IK usato da qualifiche AB07 e paired AB06/AB07 (`validation/AB07/qualify_prescribed_v26.py:246`; `validation/paired_prescribed_AB06_AB07_2026_10_07/run_pair.py:153`) e test delle azioni assolute.

## KEEP: baseline_MLP, 30 top-level .py

Prefisso `Trajectory Generator/baseline_MLP/`.

**19 moduli di runtime/training o supporto condiviso da mantenere:**
`__init__.py`, `_bootstrap.py`, `asymmetric_rl_module.py`, `env_factory.py`, `reward_function.py`, `experimental_morphology_corridor.py`, `training_config.py`, `rollout_eval.py`, `train_ppo_mlp.py`, `process_watchdog.py`, `win_runtime.py`, `progress_display.py`, `tb_logging.py`, `start_sampling.py`, `start_condition_metrics.py`, `configure_actor_exploration.py`, `warm_start.py`, `exploration_noise.py`, `update_historical_runs.py`.

`experimental_morphology_corridor.py` è importato incondizionatamente dalla reward baseline e dalle due reward unified: nome experimental non significa inutilizzato. Contiene CausalDelayedMorphologyBuffer attivo nella V26. `win_runtime.py` resta necessario per Windows e viene importato anche su Mac come shim portabile. `warm_start.py` non è solo un vecchio entrypoint: ha funzioni di lettura/verifica/digest utilizzate da moltissimi moduli e pipeline.

**2 utility diagnostiche da mantenere disponibili:**
`validate_online_grf_train_inference.py`, `imitation_oracle_rollout.py`. Sono entrypoint CLI; l'assenza di importatori non significa codice morto. Seconda utilizzabile per baseline oracle; prima gate/shim/watchdog del contratto sensore.

**3 moduli di compatibilità per rami V25/primary split, candidati ad archivio dopo migrazione:**
`primary_split_input_adapter.py`, `primary_split_v25_residual.py`, `primary_grf_split_adaptation.py`.
I primi due comprendono classi RLModule dei checkpoint residual/adapter e una import chain viva del trainer con `rl_module_kind=primary_split_v25_residual`. Non sono necessari alla rete standard best47, ma una rimozione blocca i checkpoint di quel ramo e la relativa CLI. Il terzo è importato da campagne v12p1q/v12r5q3 e compare in molte closure/check di provenance.

**4 moduli di adattamento storico, utili alla direzione student/DAgger e comunque pin di lineage:**
`target_domain_imitation.py`, `target_domain_dagger.py`, `target_domain_noise_adaptation.py`, `target_domain_markov_adaptation.py`.
Le campagne V26C importano e confrontano funzioni proprio da questi file: `test_v26c_j4_recovery.py`, `test_v26c_j7_markov_dataset.py`, `test_v26c_j8_recovery_fit.py`. È possibile archiviarli dopo decidere l'API student/DAgger attuale, chiudere dipendenze e prevedere restore dei protocolli storici; non sono quattro residui isolati.

**2 utility actor-only opzionali, candidate ad archivio dopo migrazione:**
`freeze_actor_checkpoint.py`, `project_actor_update.py`. La seconda è importata da `validation/test_project_actor_update.py` nel root. Entrambe sono protette anche dai manifest recenti; non eliminabili senza aggiornare/preservare quella closure.

**Vincolo trasversale sui 30 file:** il harness attuale `validation/reset_ik_audit_2026_10_12/frozen_reset_tests.py:160` aggiunge tutti i `.py` top-level baseline al set protetto. Il manifest random pose del 13 ottobre e quello AB07 transfer del 6 ottobre includono anche le utility non chiamate nel rollout. Se si rimuove una utility, la verifica dell'esperimento sigillato deve lavorare sul checkout/archivio originale; il protocollo futuro può avere una closure più stretta con una nuova versione esplicita.

## Checkpoint e input da proteggere prioritariamente

Run `Trajectory Generator/runs/training/MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-01/`:

- `rl_module_best/` è best47, tre file ~607 KB: metadata.json, class_and_ctor_args.pkl, module_state.pkl; conservare come unità.
- Il pickle constructor registra il modulo flat `asymmetric_rl_module` e la classe `AsymmetricActorCriticTorchRLModule`. Non basta salvare i pesi e rinominare/spostare il codice. Conservare il modulo/import path o migrare il formato con verifica live.
- `module_state.pkl` SHA256 `2d1d9b30f2b043f4b3b7b4347c06e9b3aace1c7d089b33e2893930554d84bd7c`, identico a `milestone_iteration_000047/rl_module_last/module_state.pkl`.
- `checkpoint_best/` e `checkpoint_last/` ~2,55 MB ciascuno: stato Algorithm/Learner/optimizer necessario per resume. Gli export `rl_module_*` sono inference-only e rimuovono value tower.
- `training_cfg.resolved.yaml`, `summary.json`, `train_iterations.jsonl`, manifest/schema/modulo/audit associati devono accompagnare checkpoint e provenance. La run ha 50 milestone iter2–51; quelle non usate da analisi possono essere archiviate solo dopo una lista di referenze concrete.
- Preset/pin fisici best47: `online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json`, `online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json`, `baseline_MLP/morphology_profiles/ab06_prosthetic_event_warped_mean_std_corridor.json`; setup/model/plugin AB06 e ground/assets associati.
- `baseline_MLP/train_ppo_mlp.py:59` ha il default `--warm-start` nel root `validation/critic_warmup/2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/checkpoint_last`; una directory root chiamata validation non è automaticamente eliminabile.
- `warm_start.py:24` default raw sorgente `runs/training/MLP_imitation_training_06-23-2026_grfsoft_knee1_ankle2_100iter/rl_module_best`; se si archivia la run June23, modificare intenzionalmente il preset raw o documentare restore.

## Unified35: 13 .py, ramo storico candidato archivio

Prefisso `Trajectory Generator/unified_MLP/`:
`__init__.py`, `_bootstrap.py`, `actor_transition.py`, `asymmetric_rl_module.py`, `env.py`, `env_factory.py`, `reward_function.py`, `rollout_eval.py`, `runtime_contract.py`, `tb_logging.py`, `train_ppo_mlp.py`, `training_config.py`, `training_runtime.py`.

Non è richiesto dal runtime V26 best47 o dal runtime unified39 attuale. È un candidato coerente ad archivio come ramo completo (codice, 4 YAML, test e relative evidenze), anziché smontarlo file per file. Eccezioni: i suoi checkpoint richiedono classi/import path `unified_MLP.*`; i test unified39 `tests/test_target_availability.py:15-22` caricano esplicitamente `unified_MLP/validation/feasibility39_bundle/target_availability.py`. Conservare quel bundle o migrare il test prima di rimuovere l'intera directory. Builder feasibility39 è storico per il contratto temporal-target V1, non genera automaticamente il 39D phase-target v3 attuale.

## Unified39: 23 .py, sospeso ma non abbandonato

Prefisso `Trajectory Generator/unified_MLP_39/`; mantenere disponibili tutti i core se si vuole riprendere il ramo:
`__init__.py`, `_bootstrap.py`, `actor_transition.py`, `asymmetric_rl_module.py`, `env.py`, `env_v1.py`, `env_factory.py`, `episode_telemetry.py`, `kl_audit_learner.py`, `learner_state_resume.py`, `optimizer_resume.py`, `phase_target.py`, `reward_function.py`, `reward_resume.py`, `rollout_eval.py`, `runtime_contract.py`, `target_availability.py`, `tb_logging.py`, `train_ppo_mlp.py`, `training_config.py`, `training_runtime.py`, `tuning_terms.py`, `tune_reward.py`.

- `env_v1.py` è una superclass viva, importata in `env.py:4`; eliminarla rompe immediatamente 39D.
- `phase_target.py`, `target_availability.py` definiscono la semantica sinistra/destra/mask e layout39/88 del checkpoint; non sostituibili con omonimi bundle legacy.
- `optimizer_resume.py` e `learner_state_resume.py` correggono/controllano restore Adam e stato KL che RLlib2.55 omette: togliere questi file degrada la fedeltà dei resume.
- `kl_audit_learner.py` introduce classi di learner configurate dal training e potenzialmente serializzate; non eliminare senza verificare i checkpoint cui è legato.
- `tuning_terms.py` è importato dalla reward online, anche se il tuning tool non viene eseguito.
- `tune_reward.py` + `reward_tuning/` sono il workbench CLI opzionale: archiviabili come unità dopo chiudere testi/test/pilot/report che lo chiamano; il file tuning_terms della reward va comunque conservato.
- `episode_telemetry.py` viene importato da tb_logging: nome diagnostico non significa scollegato.

I numerosi YAML datati smoke/ablation/weight possono uscire dalla superficie quotidiana dopo conservare configurazioni risolte e referenze di protocollo. I preset `pretraining`, `ex_novo`, gli smoke e preset per replica controllata left/right/mask restano parte della riattivazione possibile. La sospensione descritta negli ultimi report non è prova di dismissione del ramo.

## Campagne/evidenze: candidate ad archivio con dipendenze

- `baseline_MLP/validation/v12*`, F0/F1/F2R/V26B, sigma smoke sono prevalentemente sperimentazione storica. Molti import/path/hash li collegano fra loro e alla lineage attuale; archiviare pacchetti chiusi, non cancellare un file dal mezzo.
- `baseline_MLP/validation/v26c_july_replica_2026-08-26/` ~2,6 GB è la lineage J0–J21 sigillata; path e SHA sono verificati.
- `baseline_MLP/validation/pipeline_bundle_v26c_2026-08-28/` ~248 MB è uno store SHA256 deliberato con manifest. Non è un secondo runtime inutile. Le entry `context_snapshot_extra` sono priorità più bassa scientificamente, ma rimuoverle fa fallire l'attuale verifica completa e richiede un nuovo contratto di archivio.
- Le campagne ottobre in `validation/` restano evidenze correnti: AB06/AB07, paired, reset_ik, random_pose, TNT. Le vecchie date al loro interno non indicano codice morto: random_pose importa `reset_ik_audit_2026_10_12/frozen_reset_tests.py`; AB07 pylon4007ott importa `v26c_best47_transfer_2026_10_06/run_transfer.py`; paired numerical checker importa/pinna AB07transfer checker; AB06 compare dipende da AB07 compare e dal rollout2settembre.
- Gli attempt falliti e before/superseded snapshot sono candidati ad archivio di contesto. Non eliminarli senza preservare ragione dello scarto, hash e comparazione che li menziona.
- I file `runs/historical_runs.md` + `.index.json` sono indici generati; `historical_runs.manual.yaml` contiene annotazioni manuali non rigenerabili dal semplice scan. Tenerli insieme o riprodurre indice da archivio e mantenere manuale.

## Duplicati veri trovati: nessuno da cancellare ciecamente

Hash byte-identici nei .py >1KB:
1. random_pose `aggregate_results.py` e `classifier_v1_archive/aggregate_results.py`; anche rispettivi test. La correzione V2 dice espressamente che V1 resta in posizione originale e nel mirror; `compact_diagnostics.py:265` hasha entrambi i reader/seal/test. Il duplicato documenta un amendment; pochi37KB, costo trascurabile.
2. unified39 `validation/penetration_resume_2026-09-24/before/training_runtime.py` e `right_reference_training_2026-09-23/before/training_runtime.py`: due checkpoint source-before di esperimenti distinti.
3. unified39 `validation/rl_learning_audit_2026-09-29/upstream/ray-2.55.1/rllib/connectors/learner/general_advantage_estimation.py` e versione commit `1927658...`: copie della prova upstream per audit bootstrap. Possono essere deduplicate in un eventuale nuovo formato di archivio, mantenendo mappa path/hash.

La grande duplicazione apparente dei trainer baseline/unified35/unified39 è codice simile ma diverso. È refactor potenziale, non eliminazione di copie byte-identiche. Una libreria condivisa sarebbe un intervento funzionale con verifiche di equivalenza/restore Windows e Mac; non è necessario alla prima pulizia del filesystem.

## Residui eliminabili a basso rischio

Dentro Trajectory Generator: 993 file cache (__pycache__, .pytest_cache, .ruff_cache) = 26.926.347 B; 78 .DS_Store =563.512 B; 3 log incidentali opensim.log =815.672 B. Totale 28.305.531 B (~28,3 MB, ~27MiB). Conservare i run log/receipt scientifici: non sono gli opensim.log incidentali. I README della workspace attestano che cache/bytecode/.DS_Store/logOpenSim incidentali sono esclusi dai manifest.

`baseline_MLP/results/` è vuota (0B), rimozione solo cosmetica.

`Trajectory Generator/commands.txt` contiene esclusivamente comandi Prosthesis_SNN eliminata: entrypoint obsoleto, candidato chiaro a rimozione/sostituzione con link alla baseline. Altri riferimenti SNN sono READMEbaseline righe21–35, docstring win_runtime9–11 e note literature: aggiornare documentazione corrente, conservare note scientifiche se storico utile. Nessun import Python vivo verso prosthesis_snn/snntorch/skrl rilevato. Le31 cancellazioni SNN preesistono nell'albero di lavoro e sono tracciate da HEAD, non opera dell'audit.

`literature/` ~240KB è riferimento opzionale senza dipendenze runtime; archiviabile per alleggerire navigazione ma non è un residuo generato. Le note non includono i PDF originali, che vivono fuori repo.

## Sequenza consigliata

1. Proteggere run/checkpoint ignorati da Git e nativi/input necessari con backup separato verificabile, in particolare best47.
2. Pulire cache/DSStore/log incidentali e doc commands SNN.
3. Decidere una matrice di capacità da mantenere: simulatore, V26 standard, student/DAgger, unified39 sospesa, reload storici.
4. Archiviare esperimenti/milestone non più consultati come pacchetti con manifest, mantenendo il checkout originale per riprodurre hash/pickle e un indice di restore. Le campagne attuali non vanno private delle dipendenze datate senza migrazione.
5. Solo dopo, ridurre codice branch/compatibility/tooling non incluso nella matrice; niente delete-all validation, old, experimental, env_v1.
6. Verificare import/CLI, restore best47 in processo fresco, caricamento cfg/profile/morphology, e un rollout deterministico breve equivalente; test unitari dei branch mantenuti e verifica archive hash dove applicabile. Windows esige preservare shim e path portabili; i dati di provenance storici assoluti non vanno riscritti solo per estetica.
