# Piano - Migrazione imitation -> ex-novo con actor transplant

Data: 2026-06-25

Instruction check token: CMC_AGENT_OK_2026

## Obiettivo

Portare la pipeline `baseline_MLP` dalla fase di pretraining imitativo alla fase
`ex_novo`, usando la rete imitativa come inizializzazione della policy senza
lasciare nell'actor i target sani anti-phase.

La migrazione non deve essere un resume RLlib completo, ma un warm-start
controllato:

```text
policy imitation-trained
-> policy ex-novo con observation corretta
-> actor inizializzato via feature-aligned weight transplant
```

## Decisioni di base

### Config separati

Creare due config sorgente:

```text
Trajectory Generator/baseline_MLP/training_imitation_cfg.yaml
Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml
```

Mantenere temporaneamente:

```text
Trajectory Generator/baseline_MLP/training_cfg.yaml
```

come alias/copia legacy del config imitativo, finche' comandi, README e
workflow non sono aggiornati.

### Flag di training

Aggiornare `train_ppo_mlp.py` per accettare:

```bash
--imitation   # default
--exnovo
```

Semantica desiderata:

```text
--imitation -> default config training_imitation_cfg.yaml
--exnovo    -> default config training_exnovo_cfg.yaml
```

Scelta consigliata: se `--config` viene passato insieme a `--imitation` o
`--exnovo`, produrre errore esplicito invece di accettare combinazioni ambigue.

## Fase 1 - Split dei config

1. Copiare l'attuale config imitativo in:

   ```text
   Trajectory Generator/baseline_MLP/training_imitation_cfg.yaml
   ```

2. Creare:

   ```text
   Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml
   ```

3. Differenze minime iniziali del config `ex_novo`:

   ```yaml
   reward:
     reward_mode: ex_novo
   ```

4. Mantenere invariati, salvo decisione esplicita:

   ```text
   asymmetric_actor_critic: true
   actor_cyclic_phase_only: true
   include_reference_state_observation: true
   online_grf_observation: true
   action_mode: absolute
   policy_knots: 1
   pros_ref_model: butterworth3_jerk_limited
   ```

5. Decidere separatamente se il primo `ex_novo` deve ancora mantenere
   `reference_score` o se ridurre/sostituire `blend_reference`.

## Fase 2 - Aggiornamento loader/CLI

Modificare `training_config.py`:

- introdurre path default:

  ```text
  IMITATION_CONFIG_PATH = training_imitation_cfg.yaml
  EXNOVO_CONFIG_PATH    = training_exnovo_cfg.yaml
  DEFAULT_CONFIG_PATH   = IMITATION_CONFIG_PATH
  ```

- mantenere `training_cfg.yaml` come fallback legacy se necessario.

Modificare `train_ppo_mlp.py`:

- fare un pre-parse di:

  ```bash
  --config
  --imitation
  --exnovo
  ```

- scegliere il config prima di popolare i defaults argparse;
- impostare `--imitation` come default;
- impedire combinazioni ambigue se questa resta la decisione finale.

Aggiornare almeno:

```text
Trajectory Generator/baseline_MLP/README.md
Trajectory Generator/baseline_MLP/commands.txt
```

con esempi di training imitativo ed ex-novo.

## Fase 3 - Tool di actor transplant

Creare:

```text
Trajectory Generator/baseline_MLP/transfer_imitation_to_exnovo.py
```

### Input

Il tool deve accettare almeno:

```bash
--source-checkpoint <run_imitativo>/rl_module_best
--source-config <run_imitativo>/training_cfg.resolved.yaml
--target-config Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml
--output-dir <initializer_output_dir>
--removed-feature-mode mean-bias
```

Modalita' supportate:

```bash
--removed-feature-mode drop
--removed-feature-mode mean-bias
```

### Responsabilita'

1. Caricare la rete sorgente imitativa.
2. Ricostruire `source_actor_feature_names` dal config sorgente.
3. Ricostruire `target_actor_feature_names` dal config ex-novo.
4. Creare una rete target con architettura ex-novo.
5. Copiare i pesi actor per nome feature condiviso.
6. Rimuovere esplicitamente le feature target sane:

   ```text
   healthy_knee_angle_imitation_target
   healthy_knee_angle_imitation_target_vel
   healthy_ankle_angle_imitation_target
   healthy_ankle_angle_imitation_target_vel
   ```

7. Gestire il bias del primo layer secondo la modalita' scelta.
8. Reinizializzare il critic per il primo esperimento.
9. Salvare il modulo/initializer target.
10. Scrivere un audit JSON.

### Audit richiesto

Scrivere:

```text
actor_transplant_report.json
```

Contenuto minimo:

```text
source_checkpoint
source_config
target_config
source_actor_feature_names
target_actor_feature_names
copied_features
removed_features
target_only_features
removed_feature_mode
weight_shapes
weight_norms_before_after
critic_init_mode
```

## Fase 4 - Bias compensation

Il primo layer calcola:

```text
h = activation(Wx + b)
```

Nel modello imitativo:

```text
x = [x_shared, x_removed]
```

e quindi:

```text
W_shared * x_shared + W_removed * x_removed + b
```

Nel modello ex-novo `x_removed` non esiste piu'. Con drop secco:

```text
W_shared * x_shared + b
```

che equivale a porre implicitamente:

```text
x_removed = 0
```

La modalita' `mean-bias` deve invece approssimare il comportamento medio:

```text
b_new = b_old + W_removed * mean(x_removed)
```

cosi':

```text
W_shared * x_shared + W_removed * mean_removed + b_old
```

La media delle feature rimosse puo' essere stimata:

1. da rollout/trace della sorgente imitativa, scelta preferita;
2. in prima implementazione, da un breve pass dell'env sorgente con policy
   imitativa;
3. come fallback documentato, usando zero e degradando di fatto a `drop`.

## Fase 5 - Integrazione nel training

Non usare:

```bash
--resume-from <checkpoint_best>
```

per la migrazione imitation -> ex-novo.

Serve invece un'opzione di inizializzazione, ad esempio:

```bash
--init-actor-from <initializer_or_source_checkpoint>
--init-actor-report <actor_transplant_report.json>
```

oppure:

```bash
--actor-initializer <output_del_tool_transplant>
```

Il caricamento deve avvenire:

```text
dopo config.build_algo()
prima del primo algo.train()
```

Il nuovo run ex-novo deve avere optimizer nuovo, history nuova e summary nuovo.

## Fase 6 - Validazione iteration 0

Prima di training lunghi, generare rollout deterministici brevi/completi per:

```text
1. source imitation rollout
2. ex-novo random init
3. ex-novo actor transplant drop
4. ex-novo actor transplant mean-bias
```

Metriche minime:

```text
action raw/applied range
action clipping
terminated/truncated/end_reason
left Fy mean e contact fraction
GRF penetration e flip loss
root reserve RMS
pelvis_ty reserve RMS
knee/ankle q range
SEA saturation
tau_spring knee/ankle RMS e high-pass ratio
served reference -> actual tracking
```

Gate minimo:

```text
la policy trapiantata non deve essere peggiore di random init su stabilita',
terminazioni e range fisici grossolani.
```

## Fase 7 - Ablation training

Lanciare due training confrontabili:

```text
A: ex_novo random init
B: ex_novo actor transplant
```

Stesse condizioni:

```text
config ex-novo identico
seed identico
stesso OS
stesso numero di env runner
stesso target iterazioni
stessi timeout/supervisor
```

Promuovere il transplant solo se migliora almeno uno tra:

```text
sample efficiency
stabilita' iniziale
qualita' biomeccanica a pari iterazioni
minor clipping/saturazione
minor reserve a pari reward
```

senza peggiorare in modo chiaro contatto, GRF, SEA o tracking.

## Rischi

- Negative transfer se la policy imitativa dipendeva troppo dai target sani.
- Drop iniziale per rimozione delle quattro feature imitation.
- Critic copiato non valido per reward ex-novo.
- Ambiguita' tra reference IK base, reference servita e target sano.
- `ex_novo` attuale usa ancora `reference_score`; una reward task-based pura e'
  una decisione successiva.

## TODO

- [ ] Valutare rollout del training imitativo `grfhalf_40iter` e decidere se
      usarlo come sorgente invece del `grfsoft_knee1_ankle2_100iter`.
- [ ] Creare `training_imitation_cfg.yaml`.
- [ ] Creare `training_exnovo_cfg.yaml`.
- [ ] Decidere destino di `training_cfg.yaml` come alias legacy o deprecazione.
- [ ] Aggiornare `training_config.py` con path imitation/ex-novo.
- [ ] Aggiornare `train_ppo_mlp.py` con `--imitation` e `--exnovo`.
- [ ] Aggiornare README/commands.
- [ ] Implementare `transfer_imitation_to_exnovo.py`.
- [ ] Implementare modalita' `drop` e `mean-bias`.
- [ ] Stimare `mean(x_removed)` da rollout/trace imitativo.
- [ ] Salvare `actor_transplant_report.json`.
- [ ] Integrare caricamento initializer nel training ex-novo.
- [ ] Validare rollout iteration 0 per random/drop/mean-bias.
- [ ] Lanciare ablation ex-novo random init vs actor transplant.
- [ ] Decidere in una fase separata la riduzione o rimozione di
      `reference_score` per l'ex-novo task-based puro.
