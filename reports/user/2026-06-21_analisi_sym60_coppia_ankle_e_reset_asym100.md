# Analisi sym60 coppia ankle e reset config asym100

Data: 2026-06-21

## Problema

Dopo i confronti tra i rollout MLP imitativi, il caso `sym60` mostrava una
curva di coppia ankle qualitativamente piu simile al riferimento CMC-like
`05/23` rispetto ad alcuni run asymmetric actor-critic, pur avendo reward e
metriche globali non migliori.

La domanda operativa era capire perche `sym60` producesse quella forma di
`tau_spring` ankle e se il motivo fosse nella config, nel training, nel rollout,
nel tracking SEA o nella dinamica del controller cascade.

In parallelo e' stato richiesto di riportare `training_cfg.yaml` alla
configurazione operativa `asym100`.

## Soluzione / diagnosi

Il reset della config e' stato fatto riportando i pesi reward alla famiglia
`asym100`:

```yaml
imitation_knee_position_weight: 1.0
imitation_knee_velocity_weight: 0.02
imitation_ankle_velocity_weight: 0.04
smoothness_weight: 0.02
command_rate_weight: 0.05
sea_tau_spring_effort_weight: 0.0
sea_tau_spring_rate_weight: 0.0
policy_action_clip_weight: 0.0
```

La diagnosi su `sym60` indica che le curve di coppia ankle non derivano da una
reward fisica diretta, ne da un migliore tracking interno SEA. Derivano invece
dalla reference cinematica trovata dalla policy, che dentro il controller
prostetico cascade genera un integrale ankle negativo.

Il punto chiave e' che la `tau_spring` ankle e quasi interamente governata dal
termine integrale del velocity-PI:

```text
sym60 ankle:
tau_spring RMS        6.16 Nm
cascade P RMS         0.86 Nm
cascade I RMS         6.02 Nm
corr(tau_spring, I)   0.985
corr(tau_spring, P)   0.164
I mean               -3.31 Nm
```

Il tracking della served reference e' buono:

```text
ankle q RMSE vs served ref   0.00545 rad
knee  q RMSE vs served ref   0.00336 rad
```

Quindi il collo di bottiglia non e' "la SEA non segue"; e' la trasformazione:

```text
policy reference -> reference model -> cascade qdot_cas -> velocity PI integral -> tau_spring
```

Un istante diagnostico rappresentativo:

```text
t = 13.051 s
tau_spring  = -21.78 Nm
q           = 0.382 rad
q_ref       = 0.357 rad
qdot_ref    = +0.288 rad/s
qdot_cas    = -0.884 rad/s
velocity_err= -0.684 rad/s
P           = -1.94 Nm
I           = -19.96 Nm
```

Con:

```text
qdot_cas = qdot_ref + Kp_outer * (q_ref - q)
Kp_outer ankle = 47.125
```

un errore piccolo di posizione puo ribaltare `qdot_cas` e far integrare il
velocity loop in direzione negativa.

## Strategia di analisi

1. Letta la config risolta del training `sym60`:

   ```text
   Trajectory Generator/runs/training/baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm/training_cfg.resolved.yaml
   ```

2. Letti `summary.json`, `checkpoint_best_meta.json` e
   `train_iterations.jsonl` del training.

3. Analizzato il rollout associato:

   ```text
   Trajectory Generator/runs/rollout/baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm/
   ```

4. Incrociati:

   - `rollout_policy_trace.json`
   - `rollout_episode_sea_diagnostics.sto`
   - `rollout_episode_states.sto`
   - `rollout_episode_kinematics_reference.sto`
   - `rollout_episode_sea_torques.sto`
   - `rollout_episode_recruitment.sto`

5. Confrontato `sym60` con:

   - `asym100`
   - `asym200`
   - il nuovo `asym100` del 20/06
   - il riferimento CMC-like `05/23` in `results/`

## Evidenze principali

Config `sym60`:

```text
asymmetric_actor_critic: false
iterations: 60
reward_mode: imitation
blend_served_imitation: 0.8
blend_imitation: 0.2
blend_imitation_tracking: 0.0
sea_tau_spring_effort_weight: assente/0.0
sea_tau_spring_rate_weight: assente/0.0
policy_action_clip_weight: assente/0.0
```

Il run `sym60` locale e' un segmento di resume 41 -> 60; il miglior checkpoint
e' alla iterazione logica 59:

```text
iter 41 return  26.2
iter 49 return 220.4
iter 57 return 261.2
iter 58 return 275.7
iter 59 return 277.8  best
iter 60 return 270.8
```

Il critic resta debole:

```text
vf_explained_var circa 0.10 alla fine del segmento
```

Rollout `sym60`:

```text
episode_return             334.75
steps                      501
action_clipped_fraction    0.0
raw action max             0.966
ankle tau_spring RMS       6.16 Nm
ankle tau_spring min/max   -21.78 / +1.76 Nm
reserve RMS                449.1
```

Confronto `tau_spring` ankle:

```text
case          RMS Nm   area positiva   area negativa   I mean
sym60          6.16      +1.48 Nms      -18.03 Nms     -3.31
asym100        5.74      +9.09 Nms       -7.44 Nms     +0.33
asym200        7.33     +13.74 Nms       -5.59 Nms     +1.63
new asym100   13.05     +32.00 Nms       -4.35 Nms     +5.57
05/23 CMC     49.31      +1.78 Nms     -160.89 Nms    -31.79
```

Interpretazione:

- `sym60` assomiglia al `05/23` nel segno e nella forma negativa dell'ankle;
- la scala e' pero circa 8x troppo piccola;
- le reserve globali restano molto alte;
- la forma non e' stata ottimizzata direttamente dalla reward.

## File modificati

Modificato:

- `Trajectory Generator/baseline_MLP/training_cfg.yaml`

Creato:

- `reports/user/2026-06-21_analisi_sym60_coppia_ankle_e_reset_asym100.md`

Non modificati:

- plugin C++ SEA;
- semantica del comando SEA;
- codice del simulatore root;
- codice training/rollout.

Nota worktree: esistevano gia modifiche non correlate o precedenti in
`rollout_eval.py`, registry storici e report del 19/06; non sono state revertite.

## Test / verifiche eseguite

- Parsing YAML di `training_cfg.yaml` con `yaml.safe_load`.
- `git diff --check -- Trajectory Generator/baseline_MLP/training_cfg.yaml`.
- Lettura e confronto config risolte:
  - `sym60`;
  - `asym100`.
- Analisi numerica ad hoc dei file `.sto` e `rollout_policy_trace.json`.
- Ispezione visuale dei plot `sym60`:
  - `07_mlp_policy_vs_sound_leg_error.png`;
  - `03_gaitcycle_torque_angle_power.png`;
  - `06_time_joint_ref_sea_error.png`.
- Verifica del mapping del target imitation in `osim_trj_cmc_like.py`:
  - `pros_ankle_angle -> ankle_angle_r`;
  - phase shift ankle `0.452`.

Non sono stati lanciati nuovi training o rollout durante questa analisi.

## TODO

- [ ] Nei confronti futuri loggare/confrontare sempre `qdot_ref`, `qdot_cas`,
      `cascade_velocity_error`, contributo P, contributo I e `tau_spring`.
- [ ] Aggiungere ai plot diagnostici MLP i termini interni del cascade, almeno
      per knee e ankle.
- [ ] Separare esplicitamente nei report futuri:
      1. qualita' della reference servita;
      2. tracking SEA della reference;
      3. segno/scala della coppia prodotta dal cascade.
- [ ] Verificare ancora il nodo del target imitation ankle:
      `ankle_angle_r` sound-leg anti-phase vs `pros_ankle_angle` / offset-range
      protesico.
- [ ] Non interpretare una curva `tau_spring` piu simile all'healthy come
      successo globale se reserve, scala della coppia e lavoro ankle restano
      lontani dal riferimento.
- [ ] Valutare una testa di output bounded/squashed, ad esempio `tanh` scalato ai
      bound fisici, per ridurre la dipendenza dal clipping hard dell'azione.
      Mantenere comunque diagnostica raw-vs-applied e confrontare con
      `policy_action_clip_loss`.
