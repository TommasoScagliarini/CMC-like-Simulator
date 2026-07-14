# Daily report - 2026-07-01

## Sintesi

Il lavoro del 1/7 ha chiuso la diagnosi principale emersa dal training FSM del
30/6: la policy imparava una sequenza parziale `HS -> TO`, ma non chiudeva il
ciclo con il secondo `HS`. La FSM era corretta; il problema era che la reward
dava ancora credito persistente al mezzo ciclo incompleto.

Sono stati completati quattro passaggi:

1. implementazione del ledger/clawback nella FSM protesica;
2. rivalutazione del vecchio checkpoint 100 iterazioni con la reward aggiornata;
3. riabilitazione e verifica dell'osservazione actor ricca per ex-novo;
4. training smoke da 20 iterazioni e rollout diagnostico del best checkpoint.

Risultato finale: il ledger/clawback funziona e rimuove l'exploit FSM del mezzo
ciclo. Il training da 20 iterazioni non ha ancora chiuso `HS -> TO -> HS`, ma non
ha testato la configurazione ricca attuale. La prossima verifica critica e' un
training piu lungo con observation ricca.

## Report utente consolidati

Report del 1/7 inclusi in questo daily:

- `reports/user/2026-07-01_fsm_ledger_clawback_reward_hs_to_hs.md`
- `reports/user/2026-07-01_osservazione_exnovo_deployable_reference_state.md`
- `reports/user/2026-07-01_training_20iter_richobs_corridor_fsm.md`

## Ledger/clawback FSM

Problema diagnosticato:

```text
HS valido -> TO valido -> swing prolungato -> phase_timeout:swing
```

La vecchia reward assegnava:

- credito persistente `phase_event_progress_score = 0.5`;
- `phase_regular_score` positivo anche senza ciclo completo;
- nessuna sottrazione del credito gia' ottenuto se il ciclo falliva.

Soluzione implementata:

```text
HS valido iniziale    -> +0.10 event credit, +0.10 pending credit
TO valido             -> +0.20 event credit, pending totale 0.30
HS successivo valido  -> +0.70 cycle bonus, ciclo completato
timeout / invalid     -> clawback pending credit + extra failure penalty
```

Configurazione ex-novo:

```yaml
phase_hs_event_credit: 0.10
phase_to_event_credit: 0.20
phase_cycle_complete_bonus: 0.70
phase_failure_extra_penalty: 0.05
blend_phase_event_progress: 1.00
phase_clawback_penalty_weight: 1.00
```

Inoltre `phase_regular_score` ora richiede `valid_cycle_count > 0`, quindi un
mezzo ciclo non riceve piu' reward di regolarita'.

## Rivalutazione vecchio checkpoint 100 iter

Checkpoint rivalutato con nuova reward/FSM ledger:

```text
Trajectory Generator/runs/training/MLP_ExNovo_FSM_fullbatch_06-30-2026_100iter/rl_module_best
```

Rollout rivalutato:

```text
Trajectory Generator/runs/rollout/MLP_ExNovo_FSM_fullbatch_06-30-2026_100iter_rollout_ledger_eval_07-01-2026
```

Risultato:

```text
steps: 140
terminated: true
truncated: false
end_reason: phase_timeout:swing
episode_return: 15.9950
```

Confronto col rollout precedente dello stesso checkpoint:

```text
prima ledger/clawback:  episode_return = 43.4747
dopo ledger/clawback:   episode_return = 15.9950
```

Trace verificato:

```text
HS valido:       phase_event_progress_score = 0.10, pending = 0.10
TO valido:       phase_event_progress_score = 0.20, pending = 0.30
timeout swing:   phase_clawback_penalty = 0.30
                 phase_failure_extra_penalty = 0.05
                 phase_regular_score = 0.0
```

Conclusione: la vecchia policy resta dinamicamente nel failure mode, ma il
failure mode non riceve piu' il vecchio premio FSM persistente.

## Osservazione ex-novo ricca

La configurazione ex-novo e' stata aggiornata per non usare piu' l'actor
minimalista:

```yaml
include_reference_state_observation: true
deployable_minimal_observation: false
online_grf_observation: true
gait_clock_enable: false
actor_cyclic_phase_only: true
```

Questa scelta riabilita segnali realistici o interni al controllore:

- velocita' articolari protesiche;
- stati motore SEA;
- endpoint precedente;
- stato della reference servita;
- ultimo comando SEA e saturazione;
- online GRF/eventi e osservazione FSM.

Non riabilita i target imitativi sani, che restano disponibili solo con
`reward_mode: imitation`.

E' stato verificato che lo YAML risolve correttamente:

```text
include_reference_state_observation=True
deployable_minimal_observation=False
online_grf_observation=True
gait_clock_enable=False
actor_cyclic_phase_only=True
```

Costruendo un env singolo dallo YAML attuale:

```text
n_actor = 39
n_obs = 84
```

Feature actor confermate:

```text
pros_knee_angle_vel
pros_ankle_angle_vel
SEA_Knee_motor_angle
SEA_Knee_motor_speed
SEA_Ankle_motor_angle
SEA_Ankle_motor_speed
online_left_normal_grf_bw
online_left_in_contact
online_left_heel_strike
online_left_toe_off
phase_fsm_*
phase_expected_hs
phase_expected_to
phase_swing_elapsed_norm
pros_*_previous_endpoint
pros_*_served_ref
pros_*_served_ref_vel
pros_*_served_ref_accel
pros_*_sea_u
pros_*_sea_u_abs
pros_*_sea_u_saturated
```

Nota importante: il training smoke da 20 iterazioni eseguito il 1/7 non aveva
ancora usato questa observation ricca; nel suo resolved config risultavano
`include_reference_state_observation=false` e
`deployable_minimal_observation=true`.

## Training smoke 20 iterazioni

Run:

```text
Trajectory Generator/runs/training/MLP_ExNovo_training_07-01-2026_ledger_clawback_20iter_07-01-2026
```

Esito training:

```text
iterations_completed: 20 / 20
best checkpoint: logical_iteration 20
best_episode_return_mean: 5.4544
episode_return_mean trend: -14.16 -> 5.45
episode_len_mean finale: 141.8 step
skip/crash/restart: 0
```

Rollout del best checkpoint:

```text
Trajectory Generator/runs/rollout/MLP_ExNovo_ledger_clawback_20iter_07-01-2026_best_rollout
```

Esito:

```text
steps: 142
episode_return: 6.5989
reward_mean: 0.04647
terminated: true
truncated: false
end_reason: phase_timeout:swing
action_abs_max: 0.5876
action_clipped_fraction: 0.0
```

Sequenza FSM:

```text
step 6,   t=13.05: HS valido, event credit 0.10, pending 0.10
step 12,  t=13.11: TO valido, event credit 0.20, pending 0.30
step 142, t=14.41: timeout swing, clawback 0.30, extra penalty 0.05
```

Metriche FSM:

```text
phase_valid_hs_count: 1
phase_valid_to_count: 1
phase_valid_cycle_count: 0
phase_cycle_complete_bonus: 0.0
phase_regular_score: 0.0
```

Conclusione: lo smoke conferma che il ledger non rompe il training e riduce il
clipping, ma 20 iterazioni non bastano a validare l'apprendimento del ciclo
completo.

## Corridor morfologico

Il corridor morfologico resta diagnostico (`morphology_weight = 0.0`). Il suo
problema principale e' la sincronizzazione di fase: l'implementazione corrente
usa `online_left.gait_phase`, che prima del primo ciclo completo non e' una fase
affidabile.

Proposta discussa:

```text
prima di HS valido         -> corridor spento o diagnostico
HS valido -> TO            -> corridor attivo in stance
TO -> HS finale            -> corridor attivo in swing
HS finale valido           -> passaggio a periodo/ciclo misurato
timeout prima di HS finale -> episodio terminato dalla FSM
```

La fase corridor non dovrebbe usare direttamente la durata patologica del primo
mezzo ciclo, ma tempi nominali:

```text
dopo HS:
  phase = stance_fraction_nominal * stance_elapsed / nominal_stance_duration

dopo TO:
  phase = stance_fraction_nominal
          + (1 - stance_fraction_nominal) * swing_elapsed / nominal_swing_duration
```

Questa fase dovrebbe essere implementata come `fsm_morphology_phase`, separata
dalla fase `online_left.gait_phase`, e attivata con peso basso solo se il
training con observation ricca resta bloccato in `HS -> TO -> timeout`.

## File modificati

File principali modificati nella fase ledger/clawback e observation:

```text
Trajectory Generator/prosthetic_phase_fsm.py
Trajectory Generator/osim_trj_cmc_like.py
Trajectory Generator/baseline_MLP/reward_function.py
Trajectory Generator/baseline_MLP/env_factory.py
Trajectory Generator/baseline_MLP/tb_logging.py
Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml
validation/test_reward_function.py
validation/validate_training_config.py
```

Report creati il 1/7:

```text
reports/user/2026-07-01_fsm_ledger_clawback_reward_hs_to_hs.md
reports/user/2026-07-01_osservazione_exnovo_deployable_reference_state.md
reports/user/2026-07-01_training_20iter_richobs_corridor_fsm.md
reports/daily/2026-07-01_daily-report.md
```

Artifact principali:

```text
Trajectory Generator/runs/rollout/MLP_ExNovo_FSM_fullbatch_06-30-2026_100iter_rollout_ledger_eval_07-01-2026
Trajectory Generator/runs/training/MLP_ExNovo_training_07-01-2026_ledger_clawback_20iter_07-01-2026
Trajectory Generator/runs/rollout/MLP_ExNovo_ledger_clawback_20iter_07-01-2026_best_rollout
```

## Test e verifiche

Verifiche eseguite:

```text
python3 -m py_compile ...
python3 validation/validate_training_config.py
/opt/anaconda3/envs/envCMC-rllib/bin/python validation/test_reward_function.py
/opt/anaconda3/envs/envCMC-rllib/bin/python validation/test_phase_fsm_prescribed_env.py
CMC_FULL_FSM_PRESCRIBED_TEST=1 /opt/anaconda3/envs/envCMC-rllib/bin/python validation/test_phase_fsm_prescribed_env.py
git diff --check ...
```

Verifiche aggiuntive:

- replay/rivalutazione del vecchio rollout con reward ledger;
- rollout reale del vecchio checkpoint 100iter con nuova config;
- monitoraggio completo del training 20 iterazioni;
- rollout del best checkpoint del training 20 iterazioni;
- analisi di `rollout_policy_trace.json`;
- verifica del parsing YAML per observation ricca;
- costruzione env singolo e controllo delle feature actor.

## TODO chiusi o parzialmente chiusi oggi

- Rimuovere il credito persistente del mezzo ciclo `HS -> TO`.
- Implementare ledger/clawback del ciclo corrente.
- Fare in modo che `phase_regular_score` richieda almeno un ciclo completo.
- Propagare i nuovi termini FSM a env, reward wrapper, config e logging.
- Validare il ledger su test unitari, prescribed env e rollout reale.
- Riabilitare l'osservazione actor ricca in configurazione ex-novo.
- Verificare che l'observation ricca sia realmente presente nello schema actor.
- Eseguire uno smoke training post-ledger e rollout diagnostico.
- Verificare che il clipping non sia piu' presente nel rollout smoke 20iter
  (`action_clipped_fraction=0.0`), pur senza considerare il tema chiuso in
  generale.

## TODO aperti propagati

- Lanciare un training ex-novo piu lungo con `training_exnovo_cfg.yaml`
  aggiornato e observation ricca attiva.
- Eseguire rollout del best checkpoint del training lungo e verificare:
  - `phase_valid_cycle_count > 0`;
  - presenza del secondo `HS` dopo `TO`;
  - assenza o forte riduzione di `phase_timeout:swing`;
  - `phase_cycle_complete_bonus`;
  - `phase_clawback_penalty`;
  - `phase_pending_cycle_credit`;
  - episode length rispetto al regime attuale di circa 140-142 step;
  - `action_clipped_fraction`.
- Confrontare il training ricco contro il run ledger-eval precedente.
- Se il rollout resta in `HS -> TO -> timeout`, implementare
  `fsm_morphology_phase` e attivare il corridor morfologico con peso basso.
- Aggiornare `reward_exnovo_graph.html` se il corridor FSM entra nella reward
  effettiva.
- Tenere `morphology_weight: 0.0` finche' non sono presenti cicli protesici
  validi o una fase corridor FSM affidabile.
- Continuare a monitorare la landing window: il precedente rollout 100iter aveva
  `landing_window_contact_score` sempre nullo durante lo swing.

## Stato finale

La parte infrastrutturale e' piu solida: la FSM ora gestisce credito e fallimento
per ciclo, la reward non premia piu' il mezzo ciclo incompleto, e l'observation
ex-novo ricca e' verificata. La parte comportamentale resta aperta: serve un
training lungo con observation ricca per capire se la policy riesce finalmente a
chiudere `HS -> TO -> HS`.
