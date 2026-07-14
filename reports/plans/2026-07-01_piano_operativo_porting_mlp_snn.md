# Piano operativo - Porting baseline MLP verso SNN

Data: 2026-07-01

Instruction check token: CMC_AGENT_OK_2026

## Obiettivo

Rendere disponibile una variante SNN del `Trajectory Generator` che possa essere
allenata e valutata nello stesso perimetro sperimentale della baseline MLP
attuale.

Il porting deve cambiare una sola cosa nella prima versione validabile:

```text
actor MLP feed-forward -> actor SNN stateful
```

Tutto il resto deve restare uguale alla baseline validata:

- stesso `CMCLikeProsthesisTrajectoryEnv`;
- stesso `env_factory`;
- stessa `RewardShapingWrapper`;
- stessa reward ex-novo con FSM ledger/clawback;
- stesso action space;
- stesso observation contract actor/critic;
- stesso sistema di rollout, checkpoint, logging e validazione.

## Vincoli

- Tutto il lavoro su rete, policy e training deve restare dentro
  `Trajectory Generator/`.
- Non modificare il plugin C++ SEA.
- Non modificare la semantica del comando SEA.
- Non introdurre target sani/IK prescribed nell'actor ex-novo.
- Non trasformare il primo porting in un world model tipo P22: il primo target e'
  una SNN actor stateful, non SRM/SPM/future prediction.
- La baseline MLP deve restare funzionante e confrontabile.

## Decisione architetturale consigliata

Integrare la SNN nella pipeline `baseline_MLP` / RLlib, invece di migrare tutta
la baseline verso il vecchio scaffold `Prosthesis_SNN` / skrl.

Motivo:

- `baseline_MLP` ha gia' training supervisionato, checkpoint, rollout, TensorBoard,
  config YAML, reward centralizzata e validatori;
- il ramo SNN ha un core utile, ma la pipeline lunga e' ancora scaffold/smoke;
- il confronto MLP vs SNN deve essere il piu possibile A/B, non un confronto tra
  due infrastrutture diverse.

Topologia iniziale:

```text
actor:  SNN(obs_actor, mem_actor) -> Gaussian action
critic: MLP(obs_full)             -> V(s)
```

Questa topologia preserva l'asymmetric actor-critic attuale:

```text
policy / actor -> solo segnali deployable obs[:n_actor]
value / critic -> osservazione completa obs[:n_full]
```

Non usare un backbone SNN condiviso actor/critic nella prima versione, per evitare
leakage indiretto di feature privilegiate verso l'actor.

## Milestones operative

Il porting deve essere eseguito per milestone, non come modifica monolitica.
Ogni milestone ha un gate esplicito: Codex puo' avanzare solo quando il gate e'
soddisfatto o quando il blocco e' documentato nel file di stato del porting.

### Milestone 1 - Contratto MLP congelato

Obiettivo: fotografare esattamente il contratto ex-novo della baseline MLP.

Output richiesti:

- `Trajectory Generator/baseline_MLP/env_contract_exnovo.json`;
- lista `actor_feature_names`;
- lista `privileged_feature_names`;
- `action_shape` e `action_dim`;
- risultati dei validatori baseline.

Gate:

```text
validate_training_config.py PASS
test_reward_function.py PASS
test_phase_fsm_prescribed_env.py PASS
actor senza target imitativi sani
actor con qdot, served_ref, previous_endpoint, FSM e GRF
```

### Milestone 2 - Normalizzazione feature

Obiettivo: rendere gli input della futura SNN numericamente controllati.

Output richiesti:

- `Trajectory Generator/baseline_MLP/feature_normalization.py`;
- metadata normalizer serializzabile;
- `validation/test_snn_feature_normalization.py`;
- diagnostica su clipping e scale.

Gate:

```text
no NaN/Inf
normalized_abs_p95 <= 3.0
normalized_clip_fraction <= 0.05
feature_names ordine stabile
```

### Milestone 3 - SNN actor standalone

Obiettivo: verificare l'actor SNN fuori dal PPO.

Output richiesti:

- actor SNN `n_actor -> action_dim`;
- test memoria/reset;
- diagnostiche `firing_rate` e `mem_norm`.

Gate:

```text
forward PASS
action shape corretta
memoria cambia tra step
memoria resetta a zero
firing_rate finito, non sempre zero e non saturo
```

### Milestone 4 - RLlib module SNN actor + MLP critic

Obiettivo: sostituire solo l'actor della baseline, mantenendo il critic
privilegiato.

Output richiesti:

- `Trajectory Generator/baseline_MLP/snn_rl_module.py`;
- `validation/test_snn_rl_module.py`;
- checkpoint round-trip minimo.

Gate:

```text
actor legge solo obs[:n_actor]
critic legge obs[:n_full]
compute_values shape corretta
save/load stabile
nessun NaN/Inf
```

### Milestone 5 - Rollout stateful

Obiettivo: eseguire rollout con memoria SNN persistente tra step.

Output richiesti:

- estensione SNN di `rollout_eval.py` oppure `rollout_eval_snn.py`;
- trace con `snn_firing_rate`, `snn_mem_norm`, `snn_action_std`;
- test dedicato al reset della memoria.

Gate:

```text
rollout breve PASS
memoria persiste tra step
memoria resetta a episode reset
trace reward/FSM completo
```

### Milestone 6 - Smoke PPO SNN

Obiettivo: verificare training end-to-end minimo.

Output richiesti:

- `Trajectory Generator/baseline_MLP/training_exnovo_snn_cfg.yaml`;
- `train_ppo_snn.py` o equivalente `model_type: snn`;
- run smoke da almeno 2 iterazioni;
- checkpoint best/last caricabili.

Gate:

```text
2 iterazioni completate
checkpoint load PASS
rollout da checkpoint PASS
loss finite
nan_count == 0
```

### Milestone 7 - Training breve

Obiettivo: verificare che la SNN sia allenabile in un run breve.

Output richiesti:

- run SNN ex-novo da almeno 10 iterazioni;
- rollout del best checkpoint;
- summary confrontabile con random/untrained.

Gate:

```text
10 iterazioni completate
return non collassa
episode_len sensata
reward/FSM terms presenti
firing_rate e mem_norm sani
```

### Milestone 8 - Confronto A/B MLP vs SNN

Obiettivo: valutare se la SNN porta un vantaggio reale rispetto alla MLP.

Output richiesti:

- run MLP ex-novo con observation arricchita;
- run SNN ex-novo con stesso env/reward/action contract;
- rollout deterministici;
- report comparativo.

Gate:

```text
phase_valid_cycle_count confrontato
phase_timeout:swing confrontato
landing_window_contact_score confrontato
action_clip_fraction confrontata
SEA saturation confrontata
episode_return confrontato
```

La SNN puo' essere promossa solo se migliora almeno uno dei failure mode chiave
senza introdurre regressioni numeriche o di contratto.

## Configurazione sorgente da preservare

La config ex-novo corrente deve restare la sorgente per env/reward:

```text
Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml
```

Stato desiderato dell'actor ex-novo:

```yaml
reward:
  reward_mode: ex_novo

simulation:
  gait_clock_enable: false
  actor_cyclic_phase_only: true
  include_reference_state_observation: true
  deployable_minimal_observation: false

model:
  asymmetric_actor_critic: true
```

L'actor SNN deve vedere lo stesso prefix realistico dell'actor MLP con questa
config. Il critic SNN/MLP deve vedere lo stesso full observation usato dal critic
MLP attuale.

## Fase 0 - Snapshot e baseline di controllo

Prima di modificare codice SNN/RLlib, Codex deve acquisire una fotografia
riproducibile della baseline.

### Azioni

1. Eseguire i validatori leggeri correnti:

   ```text
   python3 validation/validate_training_config.py
   /opt/anaconda3/envs/envCMC-rllib/bin/python validation/test_reward_function.py
   /opt/anaconda3/envs/envCMC-rllib/bin/python validation/test_phase_fsm_prescribed_env.py
   ```

2. Se un comando fallisce per dipendenze o ambiente, registrare il motivo nel log
   del porting e non ignorarlo.

3. Creare o aggiornare un file macchina:

   ```text
   Trajectory Generator/baseline_MLP/snn_porting_status.json
   ```

   Contenuto minimo:

   ```json
   {
     "date": "2026-07-01",
     "baseline_config": "Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml",
     "baseline_tests": {},
     "current_stage": "phase_0",
     "open_failures": []
   }
   ```

### Metriche di uscita

- Validatori config/reward/FSM passano oppure hanno failure documentata.
- `training_exnovo_cfg.yaml` risulta parsabile.
- Lista actor/full feature della baseline estratta e salvata.

## Fase 1 - Probe observation/action contract

### Obiettivo

Garantire che SNN e MLP usino esattamente lo stesso contratto env.

### Azioni

Creare un probe leggero:

```text
Trajectory Generator/baseline_MLP/probe_env_contract.py
```

Il probe deve:

1. caricare una config YAML;
2. costruire l'env tramite `env_factory.make_cmc_env`;
3. chiamare `reset`;
4. salvare:
   - `n_actor`;
   - `n_full`;
   - `actor_feature_names`;
   - `privileged_feature_names`;
   - `observation_feature_names`;
   - `action_shape`;
   - `action_dim`;
   - `reward_mode`;
   - `include_reference_state_observation`;
   - `deployable_minimal_observation`.

Output:

```text
Trajectory Generator/baseline_MLP/env_contract_exnovo.json
```

### Test

Nuovo test:

```text
validation/test_snn_env_contract.py
```

Check minimi:

- `reward_mode == "ex_novo"`;
- `asymmetric_actor_critic == true`;
- `gait_clock_enable == false`;
- actor non contiene:
  - `healthy_knee_angle_imitation_target`;
  - `healthy_knee_angle_imitation_target_vel`;
  - `healthy_ankle_angle_imitation_target`;
  - `healthy_ankle_angle_imitation_target_vel`;
- actor contiene almeno:
  - `pros_knee_angle`;
  - `pros_knee_angle_vel`;
  - `pros_ankle_angle`;
  - `pros_ankle_angle_vel`;
  - `online_left_normal_grf_bw`;
  - `online_left_in_contact`;
  - `phase_fsm_state_id` o equivalente FSM observation;
  - `pros_knee_angle_previous_endpoint`;
  - `pros_ankle_angle_previous_endpoint`;
  - `pros_knee_angle_served_ref`;
  - `pros_ankle_angle_served_ref`;
- `action_dim == policy_knots * 2`;
- tutte le feature hanno ordine stabile tra reset e step.

### Metriche di uscita

- `env_contract_exnovo.json` creato.
- Test `test_snn_env_contract.py` passa.
- Nessuna differenza non spiegata tra contratto MLP e contratto SNN.

## Fase 2 - Normalizzazione feature

### Obiettivo

Evitare che la SNN saturi o resti muta per feature fuori scala.

### Azioni

Implementare un normalizer esplicito, preferibilmente in:

```text
Trajectory Generator/baseline_MLP/feature_normalization.py
```

Requisiti:

- input: `feature_names`, vettore raw;
- output: vettore normalizzato;
- supporto a scale statiche per feature note;
- supporto a clipping;
- serializzazione in checkpoint/config;
- nessuna dipendenza da OpenSim.

Prima versione consigliata: normalizzazione statica per categorie.

Esempio concettuale:

```text
angle_rad              -> scale 1.5
angular_velocity_rad_s -> scale 8.0
normal_grf_bw          -> scale 1.5
cycle_duration_s       -> scale 2.0
served_ref_accel       -> scale 80.0
binary flags           -> identity
sin/cos                -> identity
FSM normalized elapsed -> identity/clamp [0, 1]
```

Il normalizer deve salvare metadata:

```json
{
  "feature_names": [...],
  "center": [...],
  "scale": [...],
  "clip": 5.0,
  "mode": "static_v1"
}
```

### Test

Nuovo test:

```text
validation/test_snn_feature_normalization.py
```

Check:

- lunghezza output == lunghezza input;
- nessun `NaN`/`Inf`;
- binary feature restano in range sensato;
- sin/cos non vengono distorti;
- feature mancanti generano errore esplicito;
- round-trip metadata JSON.

### Metriche di uscita

Su uno smoke rollout o probe di almeno 100 step, registrare:

```text
normalized_abs_max
normalized_abs_p95
normalized_clip_fraction
per_feature_clip_fraction
```

Soglie iniziali di accettazione:

```text
normalized_abs_max finite
normalized_abs_p95 <= 3.0
normalized_clip_fraction <= 0.05
nessuna feature critica con clip_fraction > 0.20
```

Se le soglie falliscono, non procedere alla fase training: aggiornare scale/clip
e ripetere la fase 2.

## Fase 3 - RLModule SNN actor + MLP critic

### Obiettivo

Implementare un modulo RLlib che sostituisca solo l'actor MLP con SNN stateful.

### File consigliato

```text
Trajectory Generator/baseline_MLP/snn_rl_module.py
```

### Requisiti funzionali

Il modulo deve:

- leggere `obs[..., :n_actor]` per l'actor;
- leggere `obs[..., :n_full]` per il critic;
- applicare feature normalization solo all'actor input;
- mantenere membrane state dell'actor SNN;
- produrre `Columns.ACTION_DIST_INPUTS` compatibili con Gaussian Box action;
- implementare `compute_values`;
- implementare `get_initial_state`;
- supportare `forward_inference`, `forward_exploration`, `forward_train`;
- serializzare in checkpoint:
  - `SNNConfig`;
  - `feature_names`;
  - normalizer metadata;
  - `n_actor`;
  - `n_full`;
  - `action_dim`;
  - `sequence_length`.

### Architettura iniziale

```text
actor_input = normalize(obs[:n_actor])
actor_hidden, mem_next = SNN(actor_input, mem_prev)
action_mean_logstd = non_spiking_head(actor_hidden)

critic_input = obs[:n_full]
value = MLP_critic(critic_input)
```

Parametri iniziali consigliati:

```yaml
snn_hidden_size: 128
snn_num_layers: 1
snn_beta: 0.5
snn_threshold: 0.5
snn_encoding: direct
snn_learn_beta: false
snn_learn_threshold: true
snn_sequence_length: 16
```

Usare `direct` come primo encoder. Rate/latency possono essere ablation successive.

### Test unitari

Nuovo test:

```text
validation/test_snn_rl_module.py
```

Check:

- build modulo con `n_actor`, `n_full`, `action_dim` del probe;
- forward inference produce action dist inputs finiti;
- `compute_values` produce shape `(batch,)`;
- `get_initial_state` produce stati con shape coerente;
- lo stato cambia dopo un forward non nullo;
- lo stato torna zero dopo reset esplicito;
- nessun accesso actor al suffisso privilegiato;
- checkpoint save/load mantiene stessi output con seed fisso e memoria zero.

### Metriche di uscita

- `forward_nan_count == 0`;
- `action_dist_abs_max finite`;
- `mem_norm finite`;
- firing rate medio non degenerato:

  ```text
  0.001 <= firing_rate_mean <= 0.80
  ```

Soglia ampia intenzionale: serve solo evitare rete muta o sempre satura nel porting.

## Fase 4 - Config e CLI

### Obiettivo

Permettere training MLP e SNN dalla stessa infrastruttura.

### Azioni

Estendere `training_config.py` e `train_ppo_mlp.py` oppure creare un entrypoint
dedicato:

```text
Trajectory Generator/baseline_MLP/train_ppo_snn.py
```

Scelta consigliata: entrypoint dedicato per ridurre rischio sulla MLP.

Creare config:

```text
Trajectory Generator/baseline_MLP/training_exnovo_snn_cfg.yaml
```

che erediti semanticamente da `training_exnovo_cfg.yaml`, ma con sezione model:

```yaml
model:
  model_type: snn
  asymmetric_actor_critic: true
  seed: 123
  snn_hidden_size: 128
  snn_num_layers: 1
  snn_beta: 0.5
  snn_threshold: 0.5
  snn_encoding: direct
  snn_sequence_length: 16
```

### Test

Aggiornare o aggiungere:

```text
validation/validate_snn_training_config.py
```

Check:

- YAML parsabile;
- reward ex-novo identica alla config MLP salvo override espliciti;
- env config identica alla MLP salvo parametri SNN;
- `policy_knots` e `action_dim` coerenti con output SNN;
- `snn_sequence_length >= 1`;
- normalizer metadata presente o generabile.

## Fase 5 - Rollout stateful

### Obiettivo

Valutare una policy SNN mantenendo memoria tra step e resettandola a inizio
episodio.

### Azioni

Estendere:

```text
Trajectory Generator/baseline_MLP/rollout_eval.py
```

oppure creare:

```text
Trajectory Generator/baseline_MLP/rollout_eval_snn.py
```

Requisiti:

- caricare checkpoint SNN;
- inizializzare memoria a reset;
- passare memoria step-by-step;
- resettare memoria su `terminated`/`truncated`;
- registrare nel trace:
  - `snn_firing_rate_mean`;
  - `snn_mem_norm`;
  - `snn_mem_abs_max`;
  - `snn_action_mean_abs_max`;
  - `snn_action_std_mean`.

### Test

Nuovo test:

```text
validation/test_snn_rollout_state.py
```

Check:

- rollout di pochi step con checkpoint dummy/smoke;
- memoria cambia tra step;
- memoria si azzera a reset;
- trace contiene diagnostiche SNN;
- nessun `NaN` in action, obs, reward.

## Fase 6 - Smoke PPO SNN

### Obiettivo

Verificare che sampling, loss PPO, memoria, checkpoint e rollout funzionino end-to-end.

### Comando target

Esempio:

```text
/opt/anaconda3/envs/envCMC-rllib/bin/python \
  "Trajectory Generator/baseline_MLP/train_ppo_snn.py" \
  --config "Trajectory Generator/baseline_MLP/training_exnovo_snn_cfg.yaml" \
  --iterations 2 \
  --train-batch-size 512 \
  --num-env-runners 0 \
  --output-dir MLP_to_SNN_smoke_07-01-2026
```

### Metriche obbligatorie

Nel summary dello smoke salvare:

```json
{
  "ok": true,
  "iterations_completed": 2,
  "sampled_steps": 1024,
  "episode_return_mean": "...",
  "episode_len_mean": "...",
  "loss_policy": "...",
  "loss_value": "...",
  "entropy": "...",
  "snn_firing_rate_mean": "...",
  "snn_mem_norm_mean": "...",
  "nan_count": 0,
  "checkpoint_best_exists": true,
  "checkpoint_last_exists": true
}
```

### Soglie di accettazione smoke

- training termina senza crash;
- `iterations_completed >= 2`;
- `nan_count == 0`;
- almeno un checkpoint caricabile;
- rollout dal checkpoint caricabile;
- `snn_firing_rate_mean` finito e non esattamente zero per tutto il run;
- `snn_mem_norm_mean` finito;
- reward terms FSM presenti nel trace.

Se una soglia fallisce, Codex deve entrare nel loop di reiterazione.

## Fase 7 - Training breve comparativo

### Obiettivo

Capire se il porting produce una policy allenabile, non ancora se batte la MLP.

### Run target

```text
SNN_exnovo_10iter_smoke
```

Parametri:

```text
iterations: 10
train_batch_size: 2048 o 4096 se runtime accettabile
num_env_runners: 0 prima, poi parallelismo ridotto
sequence_length: 16
```

### Metriche da confrontare

Confrontare contro:

- random/untrained SNN rollout;
- MLP ex-novo ledger-eval precedente;
- MLP ex-novo con observation arricchita, quando disponibile.

Metriche:

```text
episode_return_mean
episode_len_mean
phase_valid_hs_count
phase_valid_to_count
phase_valid_cycle_count
phase_timeout_swing_count
landing_window_contact_score_mean
phase_clawback_penalty_mean
policy_action_clip_fraction
action_abs_max
sea_tau_input_saturation_fraction
snn_firing_rate_mean
snn_mem_norm_mean
normalized_clip_fraction
```

### Criteri di accettazione training breve

Il porting e' considerato tecnicamente valido se:

- il training completa 10 iterazioni;
- il rollout dal best checkpoint completa senza crash;
- tutti i reward terms attesi sono presenti;
- nessuna metrica numerica e' `NaN`/`Inf`;
- checkpoint best e last sono caricabili;
- il trace SNN mostra memoria non-degenerata;
- return medio migliora rispetto al rollout random/untrained oppure almeno non
  collassa a episodi immediatamente invalidi.

Non e' richiesto in questa fase che `phase_valid_cycle_count > 0`.

## Fase 8 - Training lungo A/B

### Obiettivo

Valutare se la SNN ha vantaggio reale rispetto alla MLP.

### Esperimenti minimi

1. MLP ex-novo observation arricchita:

   ```text
   training_exnovo_cfg.yaml
   ```

2. SNN ex-novo stesso env/reward:

   ```text
   training_exnovo_snn_cfg.yaml
   ```

Stesso seed iniziale dove possibile, stesso numero iterazioni, stesso
`train_batch_size`, stesso action space.

### Metriche di decisione

La SNN e' promossa a candidata reale se su rollout deterministico:

```text
phase_valid_cycle_count maggiore o uguale alla MLP
phase_timeout:swing minore o uguale alla MLP
episode_return >= MLP - tolleranza
policy_action_clip_fraction <= MLP + tolleranza
sea_tau_input_saturation_fraction <= MLP + tolleranza
nessuna degenerazione firing-rate/membrana
```

Tolleranze iniziali:

```text
episode_return_tolerance: 10%
action_clip_tolerance_abs: 0.10
sea_saturation_tolerance_abs: 0.10
```

La SNN e' considerata migliore solo se migliora almeno uno dei punti critici:

- chiude piu cicli `HS -> TO -> HS`;
- riduce `phase_timeout:swing`;
- riduce saturazione/clipping a parita' di return;
- produce landing window contact piu alto.

## Meccanismo di reiterazione obbligatorio

Codex non deve fermarsi al primo smoke fallito. Deve applicare un ciclo
standard finche' il porting non soddisfa la Definition of Done o finche' incontra
un blocco esterno non risolvibile.

### File di stato

Mantenere:

```text
Trajectory Generator/baseline_MLP/snn_porting_status.json
Trajectory Generator/baseline_MLP/snn_porting_iterations.jsonl
```

Ogni iterazione deve scrivere una riga JSONL:

```json
{
  "iteration_id": 3,
  "timestamp": "...",
  "stage": "phase_6_smoke_ppo",
  "change_summary": "...",
  "tests_run": [...],
  "metrics": {},
  "failures": [...],
  "next_action": "..."
}
```

### Classificazione failure

Ogni failure deve essere classificata in una delle categorie:

```text
contract        -> feature/action schema mismatch
dependency      -> pacchetti mancanti o versioni incompatibili
numerics        -> NaN/Inf, firing-rate zero, membrane blow-up
rl_api          -> RLlib recurrent/state API
checkpoint      -> save/load/export rotto
rollout         -> inferenza stateful non corretta
training        -> PPO loss/update non stabile
performance     -> tecnicamente funziona ma non apprende
```

### Regola di iterazione

Per ogni failure:

1. riprodurre con il comando minimo;
2. ridurre il problema a test unitario o smoke;
3. correggere il codice;
4. rieseguire tutti i test della fase corrente;
5. rieseguire anche i test delle fasi precedenti toccate;
6. aggiornare `snn_porting_iterations.jsonl`;
7. avanzare fase solo se tutte le soglie della fase sono rispettate.

### Ordine di priorita' dei fix

1. schema observation/action;
2. NaN/Inf e normalizzazione;
3. gestione memoria/reset;
4. checkpoint/rollout;
5. PPO/BPTT;
6. metriche di apprendimento.

Non ottimizzare reward o iperparametri SNN prima di avere:

```text
contratto corretto + numerica stabile + checkpoint/rollout funzionanti
```

## Definition of Done

Il porting MLP -> SNN e' completo quando sono veri tutti i punti:

- baseline MLP continua a passare i validatori esistenti;
- esiste `training_exnovo_snn_cfg.yaml`;
- esiste un RLModule/entrypoint SNN documentato;
- env/reward/action contract e' identico alla baseline ex-novo;
- normalizer feature e' serializzato in checkpoint;
- SNN actor non legge feature privilegiate;
- critic continua a leggere full obs;
- memoria SNN e' portata tra step e resettata correttamente;
- BPTT con `sequence_length >= 16` e' supportato o, se temporaneamente disattivato,
  il limite e' documentato nel report finale;
- training smoke da almeno 2 iterazioni passa;
- training breve da almeno 10 iterazioni passa;
- rollout dal best checkpoint passa;
- metriche SNN vengono salvate in summary/trace;
- `snn_porting_status.json` indica `complete`;
- e' stato scritto un report user finale con:
  - problema;
  - soluzione;
  - file modificati;
  - test eseguiti;
  - metriche ottenute;
  - confronto MLP/SNN;
  - TODO residui.

## Non-obiettivi della prima versione

- Non implementare P22 completo con SRM/SPM.
- Non passare subito a una SNN critic.
- Non cambiare reward ex-novo.
- Non cambiare low-level SEA.
- Non generare direttamente `q/qdot/qddot` nella prima versione; la SNN deve
  emettere la stessa action normalizzata della MLP.
- Non ottimizzare energia/metabolico finche' il porting base non e' stabile.

## Comandi minimi attesi

Durante il porting, Codex deve mantenere questi comandi verdi o documentare
esplicitamente il blocco:

```text
python3 validation/validate_training_config.py
python3 validation/validate_snn_training_config.py
python3 validation/test_snn_feature_normalization.py
python3 validation/test_snn_rl_module.py
python3 validation/test_snn_env_contract.py
python3 validation/test_snn_rollout_state.py
/opt/anaconda3/envs/envCMC-rllib/bin/python validation/test_reward_function.py
/opt/anaconda3/envs/envCMC-rllib/bin/python validation/test_phase_fsm_prescribed_env.py
```

I test che richiedono OpenSim/Ray/env `envCMC-rllib` devono essere marcati come
integration test e possono essere separati dai test puramente unitari, ma non
devono essere saltati nella validazione finale.

## Nota finale

Il criterio di successo non e' "la SNN batte subito la MLP". Il criterio di
successo del porting e':

```text
la SNN puo essere allenata, checkpointata, ricaricata e valutata
nello stesso esperimento della MLP, con memoria corretta e metriche affidabili.
```

Solo dopo questo punto ha senso giudicare il vantaggio scientifico della SNN.
