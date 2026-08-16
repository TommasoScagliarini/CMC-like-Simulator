# H0 V12R9 — preflight 12 EnvRunner e monitor training 50 update

Data: 2026-08-14

## Esito della milestone

È stata completata la sorgente additiva che chiude il gap di evidenza tra il checkpoint-zero creato con `num_env_runners=0` e la topologia produttiva da 12 EnvRunner remoti.

Lo stato raggiunto è:

- `PASS_H0_V12R9_TRAINING_SOURCE_CHECK` per contratto, comando, parser e validator;
- preflight reale predisposto ma non eseguito;
- training reale non eseguito;
- audit post-run predisposto per accettare soltanto 50 update integri.

Questa milestone è quindi **source-ready**, non ancora un PASS live di training-readiness. Il PASS live richiede prima il terminal PASS delle pipeline V12R9, Q3, checkpoint-zero e morphology, poi il preflight reale `0 -> 12 EnvRunner`.

## Problema

Il checkpoint-zero V12R9 viene correttamente creato, salvato e ricaricato a progress zero usando un Algorithm senza EnvRunner remoti. Il comando finale risolve invece:

- 12 EnvRunner remoti;
- 1 EnvRunner locale, quindi 13 superfici da attestare;
- 13 CPU Ray;
- exact-start sampling con 4 runner per ciascuno dei tre start;
- actor W512, critic privilegiato e morphology causale positiva.

I test esistenti dimostravano il restore `0 -> 0`, ma non dimostravano che RLlib propagasse sul target `0 -> 12`:

- actor del checkpoint-zero al Learner, al modulo locale e a tutti i worker;
- critic, optimizer e contatori a zero;
- configurazione positiva morphology senza sostituzione durante il restore;
- hook causale strict-delay in ogni processo Ray.

Senza questa prova il primo `algo.train()` sarebbe stato anche il primo test della topologia produttiva, quindi il protocollo non era ancora training-ready in senso stretto.

## Soluzione

È stata creata la namespace:

`Trajectory Generator/baseline_MLP/validation/v12r9training/`

Il contratto ha schema `1296` e protocol ID:

`AB06_H0_V12R9_MORPH_TRAINING_50_UPDATE_READINESS`

Il validator espone quattro azioni separate:

1. `--print-plan`: stampa il piano senza Ray e senza scritture;
2. `--source-check`: valida sorgente, ABI e parsing senza Ray e senza scritture;
3. `--restore-preflight`: esegue l'unico preflight live autorizzato;
4. `--post-run-audit`: valida gli artefatti dopo i 50 update.

L'import del modulo è inerte. Il codice del preflight non contiene chiamate ai metodi `train` o `sample` dell'Algorithm.

## Preflight live predisposto

Il preflight può essere eseguito solo dalla root del repository e soltanto dopo i terminal PASS upstream. La sequenza è fail-closed:

1. riesegue i verifier semantici morphology e checkpoint-zero;
2. verifica l'handoff morphology e il solo launcher autorizzato;
3. ricalcola la source closure;
4. rifiuta un output finale già esistente e un receipt preflight già pubblicato;
5. risolve il comando finale con il parser reale di `train_ppo_mlp.py`;
6. avvia Ray con 13 CPU e costruisce l'esatta configurazione finale con 12 EnvRunner remoti;
7. ripristina `checkpoint_zero` senza update e senza sampling;
8. riapplica il learning rate `5e-7`, come farà il training reale;
9. verifica actor, critic, optimizer, progress, live config ed exact-start sampling;
10. interroga il local EnvRunner e i 12 remoti e richiede gli indici esatti `0..12`;
11. su ciascuno richiede actor identico e runtime causale `h0_v12r9_morph_strict_terminal_delay_v1` installato;
12. ricontrolla i sei contatori RLlib a zero dopo l'introspezione;
13. arresta Algorithm e Ray;
14. pubblica in no-clobber il receipt solo dopo shutdown e source re-hash riusciti.

Receipt richiesto:

`Trajectory Generator/baseline_MLP/validation/v12r9training/h0_v12r9_training_preflight_20260814/receipt.json`

Status richiesto:

`PASS_H0_V12R9_TRAINING_PREFLIGHT_12_RUNNER_RESTORE`

## Semantica del warm-start

L'inizializzazione rimane imitativa per provenienza dell'actor: il checkpoint-zero contiene l'actor W512 recuperato e qualificato dalla lineage imitativa R9, con critic e optimizer freschi e progress zero.

L'interfaccia operativa corretta, però, è:

`--resume-from <checkpoint_zero>`

Non si deve usare né `--warm-start` né `--warm-start-raw`: il primo ripristinerebbe il vecchio H0 canonico, il secondo eseguirebbe un nuovo trapianto actor-only. Entrambi cambierebbero la lineage appena qualificata.

Rispetto al 15 luglio viene mantenuto il regime PPO e di sampling, non l'identità dell'intero stato storico:

- 12 EnvRunner;
- tre start `1.756870983805102`, `1.956870983805102`, `2.156870983805102`;
- fragment 384;
- 1536 step per start;
- batch 4608;
- minibatch 512, 9 minibatch/update, 1 epoca;
- learning rate `5e-7`;
- `gamma=0.99`, `lambda=0.9`, clip `0.05`;
- guard KL hard a `0.01`;
- checkpoint milestone per ogni update;
- nessun restart o skip ammesso dall'audit finale.

La nuova lineage usa actor W512, critic full a 84 feature e checkpoint a progress zero; non è quindi byte-identica all'H0 del 15 luglio.

## Detector e morphology nel training finale

Il comando finale richiede:

- `binary_phase_fsm_mode=binary_active`;
- geometria dal profilo V25 selezionato;
- contratto heel-qualified FSM v2 `binary_point_v25+heel_qualified_fsm_v2`;
- debounce 5 ms e sampling detector 1 ms;
- morphology profile event-warped;
- `morphology_weight=0.0025`;
- `morphology_causal_allow_effects=1.0`;
- delay reward 40 ms;
- delivery latency massima 10 ms;
- hard termination morphology disabilitata;
- runtime strict-delay additivo installato dal launcher morphology.

Il profilo nel path conserva il nome V25 perché contiene la geometria selezionata; il comportamento runtime completo è la composizione V26 con heel-qualified FSM v2.

## Audit post-run dei 50 update

L'audit finale richiede:

- summary conclusa con logical update `1..50`, progress restore `0` e 50 update nel solo processo;
- JSONL con esattamente 50 righe uniche e ordinate;
- lifetime sampled steps esatti `4608 * i`, finale `230400`;
- balance esatto 1536/1536/1536 a ogni update;
- compaction/interleaving 4608 e run massimo tra start pari a 1;
- 9 minibatch KL, valori finiti, guard KL PASS a ogni update;
- Adam a learning rate `5e-7` a ogni update e dopo restore;
- 51 record critic: `before_training` più update `1..50`;
- logstd bit-exact rispetto al checkpoint-zero in tutte le 50 RLModule milestone;
- esattamente 50 milestone complete, con metadati logical/RLlib `1..50`;
- checkpoint/module finali uguali al contenuto della milestone 50;
- resolved config completa uguale a quella ricostruita dal comando congelato;
- hash correnti di detector, morphology config/profile, sorgenti, checkpoint-zero e handoff uguali a quelli attestati;
- nessun restart/skip del supervisor;
- nessun restart nascosto degli EnvRunner nei log Ray, usando `validation/audit_training_restarts.py`.

Audit richiesto:

`Trajectory Generator/runs/training/v12r9_morphology_0025_50update/v12r9_training_integrity_audit.json`

Status richiesto:

`PASS_H0_V12R9_TRAINING_50_UPDATE_INTEGRITY`

## Comandi canonici futuri

Tutti i comandi devono partire dalla root del repository.

Source check, già eseguito:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python 'Trajectory Generator/baseline_MLP/validation/v12r9training/validate_h0_v12r9_training.py' --source-check
```

Preflight, da eseguire solo dopo il terminal PASS morphology:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python 'Trajectory Generator/baseline_MLP/validation/v12r9training/validate_h0_v12r9_training.py' --restore-preflight
```

Training, da eseguire soltanto dopo il receipt PASS del preflight:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python 'Trajectory Generator/baseline_MLP/validation/v12r9morph/run_h0_v12r9_morphology_training.py' --config 'Trajectory Generator/runs/training/validation/warm_start_h1_runs/2026-07-15_h0_exact_interleaved_lr5e-7_iter2-51_pilot50/training_cfg.resolved.yaml' --resume-from 'Trajectory Generator/baseline_MLP/validation/v12r9zero/h0_v12r9_zero_checkpoint_run_20260814/checkpoint_zero' --output-dir 'Trajectory Generator/runs/training/v12r9_morphology_0025_50update' --iterations 50 --num-hidden-layers 2 --dim-hidden-layers 512 --freeze-logstd --asymmetric-actor-critic --rl-module-kind standard --phase-fsm-input-mode legacy_events --event-contract-id legacy_events_v1 --binary-phase-fsm-mode binary_active --binary-phase-detector-profile validation/binary_phase_detector_v25_geometry_runs/2026-08-04_local_reach_sweep_dev02_04_08/selected_candidate_profile.json --detector-sample-dt-s 0.001 --binary-phase-debounce-s 0.005 --binary-phase-event-contract-id binary_point_v25+heel_qualified_fsm_v2 --reward-json '{"morphology_causal_allow_effects":1.0,"morphology_causal_event_contract_id":"binary_point_v25+heel_qualified_fsm_v2","morphology_causal_max_samples":4096,"morphology_experimental_allow_effects":0.0,"morphology_hard_termination_enabled":0.0,"morphology_max_delivery_latency_s":0.01,"morphology_phase_mode":"event_anchored_causal_delayed_experimental","morphology_profile":"morphology_profiles/ab06_prosthetic_event_warped_mean_std_corridor.json","morphology_reward_delay_s":0.04,"morphology_weight":0.0025}'
```

Audit dopo il training, passando la directory `logs` della sessione Ray del processo di training, non `summary.rllib_log_root` e non una `session_latest` modificata da una successiva esecuzione Ray:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python 'Trajectory Generator/baseline_MLP/validation/v12r9training/validate_h0_v12r9_training.py' --post-run-audit --ray-log-dir '<RAY_SESSION_DEL_TRAINING>/logs'
```

Su Windows si usa l'interprete Python dell'ambiente RLlib e il comando renderizzato dal medesimo handoff; il contratto conserva path POSIX canonici e produce anche argv Windows con separatori nativi.

## File aggiunti

- `Trajectory Generator/baseline_MLP/validation/v12r9training/__init__.py`
- `Trajectory Generator/baseline_MLP/validation/v12r9training/h0_v12r9_training_contract.py`
- `Trajectory Generator/baseline_MLP/validation/v12r9training/validate_h0_v12r9_training.py`
- `Trajectory Generator/baseline_MLP/validation/v12r9training/test_h0_v12r9_training.py`
- `reports/user/2026-08-14_h0_v12r9_training_readiness_preflight_monitor.md`

Non sono stati modificati core training, checkpoint-zero, R9, Q3 o morphology.

## Test e verifiche eseguiti

- `29 passed` nella suite `test_h0_v12r9_training.py`;
- `1 passed` nel test consumer morphology dell'ABI pubblico;
- `ruff check`: PASS;
- `py_compile`: PASS;
- `git diff --check`: PASS;
- `--source-check`: `PASS_H0_V12R9_TRAINING_SOURCE_CHECK`;
- parser reale: 12 EnvRunner, 13 CPU, fragment 384, 4608/update, W512, detector binario attivo, morphology `0.0025`;
- AST: nessuna chiamata metodo `train` o `sample` nel validator.

Non sono stati avviati Ray, OpenSim, il preflight live o il training. Non è stato necessario richiedere supporto a Claude Code.

## TODO propagati

1. ottenere i terminal PASS della pipeline V12R9, della Q3, del checkpoint-zero e della qualification morphology;
2. eseguire una sola volta il preflight 12-EnvRunner e ottenere il receipt PASS;
3. avviare il comando canonico dei 50 update solo dopo quel receipt;
4. conservare il path esatto dei log Ray della sessione di training;
5. eseguire l'audit post-run e richiedere il relativo PASS prima di qualsiasi qualification downstream;
6. eseguire separatamente la qualification scientifica della policy: l'integrità tecnica del training non dimostra da sola robustezza o qualità del controllo.

Nota: `reserve_residual_weight` resta `0.0`, come nel protocollo ereditato del 15 luglio. Questa milestone non modifica l'obiettivo scientifico della reward oltre all'attivazione positiva del morphology corridor.
