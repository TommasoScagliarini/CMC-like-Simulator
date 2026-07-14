# 2026-07-01 - Training 20 iter, rich observation e corridor sincronizzato FSM

## Problema

Dopo l'introduzione del ledger/clawback nella FSM protesica, e' stato eseguito
un training smoke da 20 iterazioni per verificare se la policy riuscisse a
chiudere la sequenza `HS -> TO -> HS`.

Il rollout del best checkpoint ha mostrato che:

- la FSM riconosce correttamente `HS` e `TO`;
- il ciclo non viene chiuso con il secondo `HS`;
- l'episodio termina ancora per `phase_timeout:swing`;
- il ledger funziona: il credito del mezzo ciclo viene recuperato con clawback.

Il risultato non dimostra che la reward sia sbagliata: 20 iterazioni sono poche
e il run non ha usato l'osservazione ricca attualmente impostata nel file YAML.

## Risultati del training 20 iterazioni

Run:

`Trajectory Generator/runs/training/MLP_ExNovo_training_07-01-2026_ledger_clawback_20iter_07-01-2026`

Esito:

- training completato: 20/20 iterazioni;
- nessuno skip, crash o restart;
- best checkpoint alla iterazione 20;
- `episode_return_mean`: da `-14.16` a `5.45`;
- `episode_len_mean` finale: circa `141.8` step.

Rollout del best checkpoint:

`Trajectory Generator/runs/rollout/MLP_ExNovo_ledger_clawback_20iter_07-01-2026_best_rollout`

Esito rollout:

- `steps=142`;
- `episode_return=6.599`;
- `terminated=true`;
- `truncated=false`;
- `end_reason=phase_timeout:swing`;
- `action_clipped_fraction=0.0`;
- `action_abs_max=0.588`.

Sequenza FSM nel rollout:

- step 6, `t=13.05`: `HS` valido, credito `0.1`;
- step 12, `t=13.11`: `TO` valido, pending credit `0.3`;
- nessun secondo `HS`;
- step 142, `t=14.41`: timeout swing;
- clawback applicato: `0.3`;
- extra penalty applicata: `0.05`;
- `phase_valid_cycle_count=0`;
- `phase_cycle_complete_bonus=0`;
- `phase_regular_score=0`.

Conclusione: il ledger/clawback funziona come previsto, ma in 20 iterazioni la
policy non ha ancora imparato la chiusura `HS -> TO -> HS`.

## Chiarimento sulla reward

Il return positivo del rollout non e' automaticamente un problema. Le componenti
di tracking, contatto, stabilita' e controllo SEA possono contribuire
correttamente anche se il gait cycle non e' completo. L'obiettivo non e' rendere
la reward verticale solo sulla sequenza `HS -> TO -> HS`, ma ottenere una reward
complessiva che favorisca locomozione valida e cicli protesici corretti.

La lettura corretta e':

- il training smoke ha verificato che il ledger non rompe il training;
- non basta per concludere che la policy non possa imparare;
- prima di ridisegnare la reward bisogna testare la configurazione con
  osservazione ricca realmente attiva.

## Osservazione ricca

Il file YAML corrente contiene:

```yaml
include_reference_state_observation: true
deployable_minimal_observation: false
online_grf_observation: true
gait_clock_enable: false
actor_cyclic_phase_only: true
```

E' stato verificato che il parsing dello YAML produce effettivamente:

```text
include_reference_state_observation=True
deployable_minimal_observation=False
online_grf_observation=True
gait_clock_enable=False
actor_cyclic_phase_only=True
```

E' stato costruito un env singolo dal file YAML attuale per controllare lo
schema dell'observation space:

```text
n_actor = 39
n_obs = 84
```

Nel prefisso actor compaiono effettivamente:

- `pros_knee_angle_vel`, `pros_ankle_angle_vel`;
- `SEA_Knee_motor_angle`, `SEA_Knee_motor_speed`;
- `SEA_Ankle_motor_angle`, `SEA_Ankle_motor_speed`;
- `online_left_normal_grf_bw`, `online_left_in_contact`, eventi HS/TO online;
- feature FSM: `phase_fsm_*`, `phase_expected_hs`, `phase_expected_to`,
  `phase_swing_elapsed_norm`;
- `pros_*_previous_endpoint`;
- `pros_*_served_ref`, `pros_*_served_ref_vel`, `pros_*_served_ref_accel`;
- `pros_*_sea_u`, `pros_*_sea_u_abs`, `pros_*_sea_u_saturated`.

Quindi i flag nel comando di training sono ridondanti se si usa lo YAML attuale;
possono essere omessi senza perdere l'osservazione ricca.

Comando consigliato per training notturno da 100 iterazioni:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python "Trajectory Generator/baseline_MLP/train_ppo_mlp.py" \
  --config "Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml" \
  --iterations 100 \
  --checkpoint-every 10 \
  --name _ledger_clawback_richobs_100iter_07-01-2026
```

## Corridor morfologico sincronizzato FSM

Il corridor morfologico esiste gia' come diagnostica, ma con peso nullo nella
reward. Attualmente usa `online_left.gait_phase` come fase. Questo valore pero'
dipende dalla disponibilita' di una durata ciclo stimata, quindi diventa fragile
prima del primo ciclo completo `HS -> TO -> HS`.

Dato che la policy e la FSM riescono gia' a produrre il primo `HS` e il primo
`TO`, e' ragionevole progettare una fase corridor sincronizzata sulla FSM:

```text
prima di HS valido        -> corridor spento o solo diagnostico
HS valido -> TO           -> corridor attivo in stance
TO -> HS finale           -> corridor attivo in swing
HS finale valido          -> passaggio a ciclo completo / periodo misurato
timeout prima di HS finale -> episodio terminato dalla FSM
```

La fase non dovrebbe essere normalizzata usando direttamente la durata patologica
`HS -> TO` del primo mezzo ciclo. Meglio usare tempi nominali:

```text
dopo HS:
  phase = stance_fraction_nominal * stance_elapsed / nominal_stance_duration

dopo TO:
  phase = stance_fraction_nominal
          + (1 - stance_fraction_nominal) * swing_elapsed / nominal_swing_duration
```

con clipping in `[0, 1]`.

Questa soluzione permetterebbe di usare il corridor anche prima del primo ciclo
completo, senza aspettare un secondo `HS`. Il corridor andrebbe comunque
introdotto con peso basso, come guardrail morfologica e non come imitazione
cinematica dominante.

## Strategia proposta

1. Lanciare il training da 100 iterazioni con lo YAML attuale, cioe' con
   osservazione ricca attiva.
2. Valutare il best checkpoint con rollout diagnostico.
3. Controllare non solo il return, ma soprattutto:
   - `phase_valid_cycle_count`;
   - presenza del secondo `HS` dopo `TO`;
   - `phase_cycle_complete_bonus`;
   - assenza/riduzione di `phase_timeout:swing`;
   - `action_clipped_fraction`;
   - durata episodio oltre i circa 142 step del fallimento attuale.
4. Se anche con osservazione ricca il best checkpoint resta su
   `HS -> TO -> timeout`, implementare il corridor morfologico sincronizzato
   sulla FSM.

## File modificati

In questa fase di analisi non sono state introdotte nuove modifiche al codice.

Modifiche gia' presenti dalla fase ledger/clawback precedente:

- `Trajectory Generator/prosthetic_phase_fsm.py`;
- `Trajectory Generator/osim_trj_cmc_like.py`;
- `Trajectory Generator/baseline_MLP/reward_function.py`;
- `Trajectory Generator/baseline_MLP/env_factory.py`;
- `Trajectory Generator/baseline_MLP/tb_logging.py`;
- `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml`;
- `validation/test_reward_function.py`;
- `validation/validate_training_config.py`.

## Test e verifiche eseguite

- Monitorato training 20 iterazioni fino a completamento.
- Letto `summary.json` del run 20 iterazioni.
- Eseguito rollout del best checkpoint del run 20 iterazioni.
- Analizzata `rollout_policy_trace.json` per eventi FSM, pending credit,
  clawback e cycle bonus.
- Verificato che lo YAML corrente risolva correttamente i flag di osservazione
  ricca.
- Costruito un env singolo da YAML e controllato lo schema actor/full
  observation (`n_actor=39`, `n_obs=84`).
- Verificato che le feature di velocita' giunto, stati motore SEA, FSM,
  online GRF, comando precedente e served reference compaiano nell'actor
  observation.

## TODO

- Lanciare training notturno da 100 iterazioni con osservazione ricca.
- Eseguire rollout del best checkpoint del training 100 iterazioni.
- Verificare se compare almeno un ciclo `HS -> TO -> HS`.
- Se il rollout resta bloccato su `HS -> TO -> timeout`, implementare
  `fsm_morphology_phase` e attivare il corridor con peso basso.
- Aggiornare il report del reward graph se il corridor FSM viene attivato nella
  reward effettiva.
