# 2026-06-16 - Imitation loss normalizzata e value tuning PPO

## Problema

L'ultimo rollout imitativo con observation space allargato mostrava due segnali
principali:

- errore policy/served reference vs sound leg soprattutto sull'ankle;
- chattering del motor angle, soprattutto knee.

L'analisi dei plot e dei trace ha indicato che l'errore ankle nasce gia nella
reference generata/servita dalla policy, non nel tracking SEA. L'ankle risultava
compresso verso la media: range target circa 0.355 rad, range served circa
0.179 rad. Con una loss non normalizzata in rad^2, il knee domina facilmente
perche ha un range fisiologico molto piu ampio.

In parallelo, il critic aveva mostrato `vf_explained_var` quasi nullo a fine
training. Per poter intervenire sul comportamento del value function senza
toccare codice a ogni run, serviva esporre anche `vf_clip_param` e
`vf_loss_coeff` nello YAML.

## Soluzione

La loss imitation nell'ambiente RL ora normalizza separatamente gli errori di
posizione e velocita per ogni coordinata protesica:

```text
position_loss = ((target_q - current_or_served_q) / target_q_range)^2
velocity_loss = ((target_qdot - current_or_served_qdot) / target_qdot_range)^2
coord_loss = position_weight * position_loss + velocity_weight * velocity_loss
```

I range non sono hardcoded: vengono calcolati dal target/reference attivo
costruito dall'ambiente, quindi cambiano automaticamente se cambia modello,
dataset, setup XML o fase target. Questo rende la normalizzazione model-proof.

Sono stati esposti nello YAML:

```yaml
ppo:
  vf_clip_param: 10.0
  vf_loss_coeff: 1.0

simulation:
  imitation_knee_position_weight: 1.0
  imitation_ankle_position_weight: 1.0
  imitation_knee_velocity_weight: 0.02
  imitation_ankle_velocity_weight: 0.02
```

I pesi `imitation_weight` e `served_imitation_weight` restano nel blocco
`reward:` e continuano a controllare la sharpness dello score finale.

## Strategia

1. Mantenere la costruzione del target imitativo dentro
   `Trajectory Generator/osim_trj_cmc_like.py`, senza modificare plugin C++ o
   semantica SEA.
2. Calcolare i range campionando il target periodico sull'intervallo del
   dataset/setup caricato.
3. Usare floor numerici minimi per evitare divisioni per zero se un nuovo
   modello/reference non fornisce range valido.
4. Conservare compatibilita con la vecchia semantica: se i pesi per coordinata
   non sono specificati, posizione usa peso 1.0 e velocita usa il vecchio
   `imitation_vel_weight`.
5. Propagare i nuovi parametri sia al training sia al rollout tramite snapshot
   YAML risolta, cosi inference e training restano allineati.

## File modificati

- `Trajectory Generator/osim_trj_cmc_like.py`
  - aggiunti `imitation_position_weights` e `imitation_velocity_weights`;
  - aggiunto calcolo automatico di `position_ranges_rad` e
    `velocity_ranges_rad_s`;
  - sostituita la loss imitation raw rad^2 con loss normalizzata e pesata;
  - aggiunta diagnostica per sottotermini position/velocity, pesi e range.
- `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`
  - passati i nuovi pesi imitation all'env;
  - esposti `--vf-clip-param` e `--vf-loss-coeff`;
  - cablati i due parametri dentro `PPOConfig.training(...)`.
- `Trajectory Generator/baseline_MLP/rollout_eval.py`
  - passati i pesi imitation all'env anche in evaluation/rollout.
- `Trajectory Generator/baseline_MLP/training_config.py`
  - aggiunte le nuove chiavi YAML per PPO e simulation al loader/snapshot.
- `Trajectory Generator/baseline_MLP/tb_logging.py`
  - aggiunto logging TensorBoard dei nuovi sottotermini imitation.
- `Trajectory Generator/baseline_MLP/training_cfg.yaml`
  - aggiunti i nuovi campi PPO e imitation.
- `Trajectory Generator/baseline_MLP/training_cfg.v4_imitation.yaml`
  - aggiunti gli stessi campi per la variante V4 imitation.

## Verifiche eseguite

- Compilazione sintattica:

```text
python -m py_compile Trajectory Generator/osim_trj_cmc_like.py
python -m py_compile Trajectory Generator/baseline_MLP/train_ppo_mlp.py
python -m py_compile Trajectory Generator/baseline_MLP/rollout_eval.py
python -m py_compile Trajectory Generator/baseline_MLP/training_config.py
python -m py_compile Trajectory Generator/baseline_MLP/tb_logging.py
```

Esito: ok.

- Loader YAML:

```text
vf_clip_param = 10.0
vf_loss_coeff = 1.0
imitation_knee_position_weight = 1.0
imitation_ankle_position_weight = 1.0
imitation_knee_velocity_weight = 0.02
imitation_ankle_velocity_weight = 0.02
```

Esito: ok.

- Compatibilita RLlib PPO:

```text
PPOConfig().training(vf_clip_param=10.0, vf_loss_coeff=1.0)
```

Eseguito con `win_runtime` prima di importare RLlib. Esito: ok.

- Smoke runtime dell'env con il setup AB06 SEASEA PI:

```text
position_ranges_rad:
  pros_knee_angle  = 0.93256
  pros_ankle_angle = 0.35478

velocity_ranges_rad_s:
  pros_knee_angle  = 8.11114
  pros_ankle_angle = 3.37594
```

Esito: ok. I range sono coerenti con l'analisi precedente sul rollout.

## Note operative

Non e stato avviato un nuovo training. Al prossimo run conviene controllare in
TensorBoard i nuovi segnali `reward_diagnostic/*imitation_position*` e
`reward_diagnostic/*imitation_velocity*`. Poiche la loss ora e dimensionless, la
scala numerica di `sound_imitation_loss` e `served_imitation_loss` cambia: se gli
score imitation risultano troppo schiacciati verso zero, andra rivalutata la
sharpness `imitation_weight` / `served_imitation_weight`.
