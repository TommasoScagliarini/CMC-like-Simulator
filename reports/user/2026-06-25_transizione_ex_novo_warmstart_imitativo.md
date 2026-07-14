# Transizione ex-novo e warm-start da policy imitativa - 2026-06-25

Instruction check token: CMC_AGENT_OK_2026

## Problema

Il progetto sta passando dalla generazione imitativa delle traiettorie
protesiche alla generazione `ex_novo`, cioe' senza usare il target anti-phase
della gamba sana come riferimento diretto per l'attore.

Le domande operative erano:

- cosa vedono actor e critic con asymmetric actor-critic (`AC asym`) e
  `reward_mode: ex_novo`;
- se la baseline imitativa, in particolare il run associato a
  `plot/06_24_2026_1_asym100_GRFpenalty-lowered`, puo' essere usata come
  pretraining per il successivo training `ex_novo`;
- se il trasferimento dei pesi della policy imitativa verso una policy
  `ex_novo` senza target sano nell'osservazione sia una scelta pulita,
  scientificamente difendibile e con effetto reale atteso.

Baseline imitativa citata:

```text
Trajectory Generator/runs/training/MLP_imitation_training_06-23-2026_grfsoft_knee1_ankle2_100iter
plot/06_24_2026_1_asym100_GRFpenalty-lowered
```

Nota aggiornata: il 25/06 e' stato lanciato anche un training imitativo a
40 iterazioni con penalita' GRF piu' bassa (`0.5 / 0.05`), che potrebbe
diventare la nuova baseline se il rollout maturo risulta migliore.

## Chiarimento su `ex_novo`

Nel codice attuale `reward_mode: ex_novo` non significa ancora "reward
task-based pura senza alcun riferimento cinematico".

La reward `ex_novo` usa ancora:

```python
base = (
    blend_tracking * tracking_score
    + blend_reference * reference_score
    + blend_bio * bio_score
    - penalty
)
```

Quindi:

- `tracking_score` premia il tracking plant/SEA della reference servita dalla
  policy;
- `reference_score` confronta ancora la cinematica protesica simulata con la
  cinematica IK/prostetica base;
- `bio_score` misura il tracking del contesto biologico rispetto alla stessa IK
  base;
- i termini fisici/GRF/SEA restano penalita' aggiuntive.

Il file IK/prostetico base e':

```text
models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot
```

ed e' referenziato dal setup:

```text
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml
```

Per una reward `ex_novo` davvero libera/task-based resta quindi da decidere se
azzerare, ridurre molto o sostituire `blend_reference`.

## Observation contract con AC asimmetrico

Con:

```yaml
model:
  asymmetric_actor_critic: true
simulation:
  actor_cyclic_phase_only: true
  include_reference_state_observation: true
grf:
  online_grf_observation: true
reward:
  reward_mode: ex_novo
```

l'ambiente emette una osservazione completa ordinata come:

```text
[actor prefix | privileged critic-only suffix]
```

Il modulo `AsymmetricActorCriticTorchRLModule` instrada:

```text
policy / actor: obs[:n_actor]
value / critic: obs[:n_full]
```

### Actor in `ex_novo`

L'actor vede segnali realistici/propriocettivi e di carico protesico:

```text
gait_phase_sin
gait_phase_cos

pros_knee_angle
pros_knee_angle_vel
pros_ankle_angle
pros_ankle_angle_vel

SEA_Knee_motor_angle
SEA_Knee_motor_speed
SEA_Ankle_motor_angle
SEA_Ankle_motor_speed

online_left_normal_grf_bw
online_left_in_contact
online_left_heel_strike
online_left_toe_off
online_left_gait_phase_sin
online_left_gait_phase_cos
online_left_cycle_duration_s

pros_knee_angle_previous_endpoint
pros_knee_angle_served_ref
pros_knee_angle_served_ref_vel
pros_knee_angle_served_ref_accel
pros_knee_angle_sea_u
pros_knee_angle_sea_u_abs
pros_knee_angle_sea_u_saturated

pros_ankle_angle_previous_endpoint
pros_ankle_angle_served_ref
pros_ankle_angle_served_ref_vel
pros_ankle_angle_served_ref_accel
pros_ankle_angle_sea_u
pros_ankle_angle_sea_u_abs
pros_ankle_angle_sea_u_saturated
```

In `ex_novo` l'actor **non** vede:

```text
healthy_knee_angle_imitation_target
healthy_knee_angle_imitation_target_vel
healthy_ankle_angle_imitation_target
healthy_ankle_angle_imitation_target_vel
```

Questi quattro campi vengono aggiunti solo quando `reward_mode == "imitation"`.

### Critic in `ex_novo`

Il critic vede tutto l'actor piu' il suffisso privilegiato:

```text
pros_knee_angle_target
pros_knee_angle_target_vel
pros_knee_angle_tracking_error
pros_ankle_angle_target
pros_ankle_angle_target_vel
pros_ankle_angle_tracking_error

pelvis_tx, pelvis_ty, pelvis_tz
pelvis_tilt, pelvis_list, pelvis_rotation
hip_flexion_r, knee_angle_r, ankle_angle_r
hip_flexion_l, hip_adduction_l
```

per ciascuna coordinata biologica disponibile vengono inclusi:

```text
q
q_vel
q_kin_ref
```

Inoltre, se `online_grf_observation` e' attivo, il critic vede anche il carico
del lato sano:

```text
online_right_normal_grf_bw
online_right_in_contact
online_right_heel_strike
online_right_toe_off
online_right_gait_phase
online_right_cycle_duration_s
```

Nota: i campi `*_target` del suffisso privilegiato sono la reference servita dal
reference model della policy, non il target sano anti-phase.

## Uso della rete imitativa

Il checkpoint completo per continuare un training RLlib e':

```text
checkpoint_best
```

mentre:

```text
rl_module_best
```

e' l'export inference-only usato per rollout e visualizzazione.

Un resume diretto imitation -> ex_novo con:

```bash
--resume-from .../checkpoint_best --reward-mode ex_novo
```

non e' pulito se il training imitativo aveva quattro feature target sane
nell'actor. In quel caso cambia `n_actor`, quindi cambia il primo layer della
policy. Inoltre il critic/value del training imitativo stima una reward diversa,
quindi non e' automaticamente affidabile per il nuovo objective.

## Soluzione scelta: weight transplant dell'actor

La soluzione piu' pulita per sfruttare la baseline imitativa gia' addestrata e':

```text
imitation-pretrained policy initialization with feature-aligned actor weight transfer
```

In pratica:

1. costruire un nuovo training `ex_novo` con observation corretta, senza target
   sano nell'actor;
2. caricare la policy imitativa sorgente;
3. copiare i pesi dell'actor per nome feature condiviso;
4. saltare le quattro colonne relative ai target sani rimossi;
5. inizializzare ex-novo il critic, oppure copiarlo solo dopo una verifica
   dedicata;
6. ripartire con un nuovo optimizer/training PPO, non come resume RLlib
   classico.

Per il primo layer e' preferibile non fare solo un drop secco. Meglio compensare
nel bias il contributo medio delle feature rimosse:

```text
W_new[:, shared_feature] = W_old[:, same_shared_feature]
b_new = b_old + W_old[:, removed_target_features] * mean(removed_target_features)
```

Questo evita di trattare implicitamente i target rimossi come se valessero zero.

## Perche' puo' avere effetto reale

Il trasferimento non conserva la reward imitativa, ma conserva una
inizializzazione utile della policy:

- scala ragionevole delle azioni;
- mapping iniziale tra stato protesico, SEA, GRF online e comando;
- coordinazione knee/ankle non casuale;
- regione dinamicamente fattibile del plant;
- minore probabilita' di esplorazione iniziale distruttiva;
- uso gia' appreso del reference governor e dei limiti SEA.

Il rischio principale e' negative transfer: se la policy imitativa dipendeva in
modo forte dalle quattro feature target sane, rimuoverle puo' produrre un drop
iniziale. Per questo il risultato va sempre confrontato contro un training
`ex_novo` da inizializzazione random.

## Lettura dalla letteratura

La scelta e' coerente con diverse linee note in RL per controllo fisico:

- dimostrazioni o imitation pretraining possono ridurre sample complexity e
  produrre policy iniziali piu' robuste in controllo continuo;
- motion imitation puo' funzionare come prior di comportamento prima di passare
  a objective piu' task-based;
- curriculum/transfer in RL sono utili quando il task sorgente prepara una
  regione dinamicamente plausibile per il task target;
- l'asymmetric actor-critic e' coerente con training-time privileged
  information, purche' la policy deployabile non legga feature privilegiate.

Riferimenti discussi:

- Rajeswaran et al., 2017, `Learning Complex Dexterous Manipulation with Deep
  Reinforcement Learning and Demonstrations`.
- Peng et al., 2018, `DeepMimic: Example-Guided Deep Reinforcement Learning of
  Physics-Based Character Skills`.
- Iscen et al., 2020, `Learning Agile Locomotion Skills with a Mentor`.
- Narvekar et al., 2020, `Curriculum Learning for Reinforcement Learning
  Domains: A Framework and Survey`.
- Zhu et al., 2020, `Transfer Learning in Deep Reinforcement Learning: A Survey`.
- Baisero and Amato, 2021, `Unbiased Asymmetric Reinforcement Learning under
  Partial Observability`.

Verdetto: il weight transplant e' scientificamente difendibile, ma va trattato
come warm-start sperimentale e validato con ablation.

## File modificati

Creato:

```text
reports/user/2026-06-25_transizione_ex_novo_warmstart_imitativo.md
```

Nessuna modifica a:

```text
Trajectory Generator/
config.py
plugin C++
modelli OpenSim
paper/
```

## Test e verifiche eseguite

- Letto il setup:
  `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml`.
- Verificato il file cinematico IK usato da `q_base`:
  `models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot`.
- Letti e verificati:
  - `Trajectory Generator/osim_trj_cmc_like.py`;
  - `Trajectory Generator/baseline_MLP/env_factory.py`;
  - `Trajectory Generator/baseline_MLP/reward_function.py`;
  - `Trajectory Generator/baseline_MLP/asymmetric_rl_module.py`;
  - `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`;
  - resolved config del run `grfsoft_knee1_ankle2_100iter`;
  - resolved config del run `grfhalf_40iter`.
- Verificato che `reward_mode: imitation` abilita
  `include_imitation_target_observation`, mentre `reward_mode: ex_novo` lo
  disabilita.
- Verificato che `checkpoint_best` e' il checkpoint completo per resume RLlib,
  mentre `rl_module_best` e' l'export inference-only.

## TODO

- [ ] Decidere se la baseline sorgente per il transplant sara'
      `grfsoft_knee1_ankle2_100iter` oppure il nuovo training imitativo
      `grfhalf_40iter` dopo valutazione del rollout.
- [ ] Implementare un tool di weight transplant feature-aligned per
      `AsymmetricActorCriticTorchRLModule`.
- [ ] Nel tool, copiare i pesi actor per nome feature condiviso e compensare il
      bias per le quattro feature target sane rimosse.
- [ ] Reinizializzare il critic per il primo esperimento ex-novo, salvo test
      separato di copia critic.
- [ ] Lanciare due ablation:
      `ex_novo` da random init vs `ex_novo` da actor transplant.
- [ ] Decidere la prima reward `ex_novo` reale: mantenere temporaneamente
      `reference_score`, ridurlo, oppure sostituirlo con termini task-based
      senza riferimento cinematico protesico.
- [ ] Nei report futuri distinguere sempre:
      target imitation sano, reference IK/prostetica base, reference servita
      dalla policy e tracking plant/SEA della reference servita.
