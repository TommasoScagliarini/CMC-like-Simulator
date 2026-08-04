# Daily Report - 2026-07-04

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidato

- `reports/user/2026-07-04_validazione_reward_exnovo_dettagliata.md`

## Problema

Il rollout ex-novo da 100 iterazioni sembrava chiudere cicli validi, ma la
visualizzazione mostrava ginocchio quasi statico e oscillazione della sola
caviglia. La FSM premiava quindi una sequenza `HS -> TO -> HS` formalmente
ordinata ma biomeccanicamente fasulla.

## Soluzione e strategia

La FSM e' stata estesa con gate su:

- frazione di stance in contatto `>= 0.20`;
- integrale del carico `>= 0.04 BW*s`;
- escursione knee nel ciclo `>= 0.12 rad`;
- durate minime e rifiuto esplicito del ciclo.

La validazione e' stata separata in audit offline, baseline policy e training
diagnostico breve, evitando di confondere reward positiva con gait valido.

## Risultati

- prescribed aligned: `2` cicli, return `+136.289`, PASS;
- prescribed lungo: `4` cicli, return `+252.817`, PASS;
- separation matrix: tutti i negativi sotto il `70%` del positivo, PASS;
- `fake_cycle_ankle_only`: `0` cicli e bonus ciclo nullo, PASS;
- random policy: return `-2.085`, `0` cicli;
- vecchio checkpoint 100 iter: return `-12.280` con reward corrente;
- training 10 iter: PASS tecnico ma FAIL comportamentale;
- rollout best: `10` step, nessun TO, termine `grf_penetration`.

Decisione: training 20-50/100 iterazioni bloccati.

## File modificati o creati

- `Trajectory Generator/prosthetic_phase_fsm.py`
- `Trajectory Generator/osim_trj_cmc_like.py`
- `Trajectory Generator/baseline_MLP/reward_function.py`
- `Trajectory Generator/baseline_MLP/env_factory.py`
- `Trajectory Generator/baseline_MLP/tb_logging.py`
- `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml`
- `validation/reward_audit_suite.py`
- `validation/random_policy_reward_probe.py`
- `validation/test_reward_function.py`
- `validation/validate_training_config.py`

## Test e verifiche

- audit prescribed breve/lungo: PASS;
- matrice scenari negativi: PASS;
- random baseline e vecchio checkpoint: eseguiti;
- smoke training/rollout: PASS tecnico;
- `py_compile`, config smoke e `git diff --check`: PASS.

## TODO aperti e propagati

- [ ] Isolare la terminazione precoce per `grf_penetration`.
- [ ] Separare effetto di reset, azione iniziale, SEA, reference governor e
      shaping reward.
- [ ] Produrre un rollout registrato con GRF, penetrazione, azioni e FSM.
- [ ] Richiedere almeno un `HS -> TO` sinistro prima di sbloccare training
      piu' lunghi.

## TODO storico SEA propagato

- [ ] Valutare una deflessione SEA iniziale coerente con la coppia richiesta;
      il punto progettuale del 13/06 non risulta ancora formalmente chiuso.
