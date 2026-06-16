# 2026-06-16 - Reward adimensionale e normalizzazione completa

## Problema

La reward del training MLP mescolava termini gia adimensionali con termini ancora
in unita fisiche:

- `tracking_loss`, `reference_loss` e `bio_loss` erano ancora basate su errori in
  rad^2;
- `grf_penetration_loss` era ancora in metri;
- `out_of_band_loss` era calcolata in rad^2;
- la imitation loss era gia stata normalizzata, ma i pesi knee/ankle dovevano
  essere trattati come pesi di reward, non come scale fisiche nascoste.

Questo rendeva difficile interpretare i pesi in `training_cfg.yaml`: un peso
poteva compensare sia la priorita del termine sia la sua scala/unita fisica.

## Soluzione

La reward ora usa loss adimensionali per tutti i termini che entrano nello
scalare della reward. Le diagnostiche raw con unita fisiche restano loggate, ma
non vengono moltiplicate direttamente dai pesi della reward.

Modifiche principali:

- `tracking_loss`: posizione e velocita protesiche normalizzate con range
  calcolati dalla reference caricata.
- `reference_loss`: errore rispetto alla prosthetic IK/base reference
  normalizzato con range reference per coordinata.
- `bio_loss`: coordinate biologiche normalizzate con range reference per
  coordinata.
- `smoothness_loss`: basata sul movimento fisico intra-segmento dei knot futuri,
  non sul raw action array.
- `grf_penetration_loss`: normalizzata tra soglia soft e soglia di termination.
- `out_of_band_loss`: normalizzata per ampiezza della banda ammessa.
- I pesi imitation per knee/ankle posizione/velocita sono stati spostati nel
  blocco `reward:` e poi iniettati nell'env da `env_factory.py`.

## Strategia

1. Calcolare range model-proof dalla reference caricata nell'env, con floor
   numerici per coordinate quasi costanti.
2. Mantenere adimensionali i termini gia normalizzati: effort, saturation,
   command-rate, governor, SEA stress, safety e imitation.
3. Conservare diagnostiche raw con suffissi espliciti (`_error`, `_range`,
   `_m`, ecc.) per debug e TensorBoard.
4. Lasciare i pesi di priorita nel blocco `reward:`; i normalizer fisici minimi
   restano in `simulation:`.
5. Propagare i nuovi campi tramite `training_config.py`, `train_ppo_mlp.py` e
   `rollout_eval.py`, cosi la snapshot risolta mantiene training e rollout
   allineati.

## File modificati

- `Trajectory Generator/osim_trj_cmc_like.py`
  - calcolo range reference posizione/velocita;
  - normalizzazione di tracking/reference/bio;
  - normalizzazione della penetrazione GRF;
  - diagnostiche per sottotermini e range.
- `Trajectory Generator/baseline_MLP/reward_function.py`
  - `out_of_band_loss` normalizzata;
  - pesi imitation per knee/ankle aggiunti a `RewardConfig`;
  - componenti reward estesi con loss normalizzate.
- `Trajectory Generator/baseline_MLP/env_factory.py`
  - ponte `RewardConfig -> CMCEnvConfig` per i pesi imitation.
- `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`
  - passaggio dei nuovi normalizer reference all'env.
- `Trajectory Generator/baseline_MLP/rollout_eval.py`
  - stesso passaggio in evaluation.
- `Trajectory Generator/baseline_MLP/training_config.py`
  - nuove chiavi YAML/snapshot.
- `Trajectory Generator/baseline_MLP/tb_logging.py`
  - logging dei nuovi sottotermini normalizzati.
- `Trajectory Generator/baseline_MLP/training_cfg.yaml`
  - aggiunti `reward_reference_range_floor` e
    `reward_reference_velocity_range_floor`;
  - pesi imitation collocati in `reward:`.
- `Trajectory Generator/baseline_MLP/training_cfg.v4_imitation.yaml`
  - stesso allineamento della variante V4.

## Verifiche eseguite

- Compilazione sintattica:

```text
python -m py_compile Trajectory Generator/osim_trj_cmc_like.py
python -m py_compile Trajectory Generator/baseline_MLP/env_factory.py
python -m py_compile Trajectory Generator/baseline_MLP/reward_function.py
python -m py_compile Trajectory Generator/baseline_MLP/train_ppo_mlp.py
python -m py_compile Trajectory Generator/baseline_MLP/rollout_eval.py
python -m py_compile Trajectory Generator/baseline_MLP/training_config.py
python -m py_compile Trajectory Generator/baseline_MLP/tb_logging.py
```

Esito: ok.

- Loader YAML:

```text
reward_reference_range_floor = 0.05
reward_reference_velocity_range_floor = 0.1
imitation_knee_position_weight = 1.0
imitation_ankle_position_weight = 1.0
imitation_knee_velocity_weight = 0.02
imitation_ankle_velocity_weight = 0.02
```

Esito: ok. I normalizer restano in `simulation:`, i pesi imitation sono letti
da `reward:`.

- Smoke `out_of_band_loss`:

```text
dentro banda = 0.0
errore pari a una banda = 1.0
```

Esito: ok.

- Smoke env tramite `env_factory.make_cmc_env` con setup AB06 SEASEA PI:

```text
bad = []
tracking_loss = 0.011907
tracking_position_loss = 0.000143
tracking_velocity_loss = 0.023671
reference_loss = 0.003538
bio_loss = 0.000022
smoothness_loss = 0.0
segment_delta_loss = 25.0
segment_knot_delta_loss = 0.0
command_rate_loss = 4.220567
grf_penetration_loss = 0.0
sound_imitation_loss = 0.001021
served_imitation_loss = 0.000755
reward = 0.973806
```

Esito: ok. Le loss controllate sono finite e non negative. Con
`policy_knots: 1`, `smoothness_loss` resta 0 mentre il salto anchor-endpoint
rimane visibile in `segment_delta_loss`.

- `git diff --check` sui file toccati:

```text
ok
```

Sono comparsi solo warning CRLF attesi su Windows.

## Note operative

Non e stato lanciato un training completo. Dopo il prossimo run conviene
controllare TensorBoard, in particolare:

- `reward_loss/tracking_position_loss`
- `reward_loss/tracking_velocity_loss`
- `reward_loss/reference_loss`
- `reward_loss/bio_loss`
- `reward_loss/sound_imitation_loss`
- `reward_loss/served_imitation_loss`
- `reward_loss/segment_delta_loss`
- `reward_loss/grf_penetration_loss`

I valori di scala dei pesi potrebbero richiedere tuning perche ora tutti vedono
loss adimensionali. Non e stato applicato alcun ulteriore tuning dei pesi dopo
questa normalizzazione.
