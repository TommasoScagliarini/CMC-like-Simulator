# Reward centralizzata e TensorBoard per baseline_MLP

## Problema

Due necessita' nella baseline MLP (Ray RLlib PPO, `Trajectory Generator/baseline_MLP/`):

1. **Reward distribuita**: la reward era definita dentro l'env (`osim_trj_cmc_like.py`,
   `_get_reward`) e ricomputata internamente a ogni step. Non c'era modo di tunarla
   senza toccare l'env. Il finding **F2** (reward dominata da `reference_score` con
   peso 0.55 → ~0.90 reward senza apprendimento) richiedeva un punto unico dove
   intervenire.

2. **Nessun TensorBoard**: il training non produceva log strutturati per monitorare
   le curve di apprendimento, le componenti della reward o i loss del learner. Ray
   poteva scrivere log propri in `~/ray_results`, ma scollegati dagli `output_dir`
   del nostro pipeline.

## Soluzione

### `reward_function.py` — unica fonte della reward

Introdotto un seam netto:

- **L'env** (`osim_trj_cmc_like.py`) calcola i *loss* fisici per-step (tracking,
  reference, biologico, effort, smoothness, saturazione SEA, safety) e li espone
  in `info["reward_terms"]`. **Non modificato.**
- **`reward_function.py`** combina quei loss nello scalare reward (lo shaping che
  si tuna qui). `RewardShapingWrapper` (wrapper gymnasium) ricalcola la reward dai
  loss e la **sostituisce** a quella dell'env, a valle della catena di wrapper.

`RewardConfig` (dataclass) contiene tutti i knob: pesi loss→score (`tracking_weight`
= 8, `reference_weight` = 6, `bio_weight` = 2), penalita' (`effort_weight` = 0.05,
`smoothness_weight` = 0.1, `saturation_weight` = 0.1), `safety_weight` = 2.0, blend
dei tre score (0.25 / 0.55 / 0.20), clip [0, 1].

I **default riproducono la reward originale dell'env in modo bit-exact** (verificato:
`max|diff| = 0.0` su step reali). Il return del tiny-train resta 3.566 (invariato).

Conseguenza importante: i campi `reward_*_weight` dentro l'env non influenzano piu'
la reward dell'agente (alimentano solo la reward interna ora scartata). Si tuna tutto
in `RewardConfig`.

Override runtime via `--reward-json` (file o JSON inline):

```
... train_ppo_mlp.py ... --reward-json '{"blend_reference": 0.35, "saturation_weight": 0.5}'
```

Lo stesso flag va passato a `rollout_eval.py` per coerenza train/eval. La
`reward_config` usata e' registrata in `summary.json` e `rollout_summary.json`.

### `tb_logging.py` — TensorBoard

Due componenti:

- `RewardComponentsCallback(RLlibCallback)`: legge `episode.get_infos(-1)` in
  `on_episode_step`, aggrega componenti reward (`info["reward_components"]`) e loss
  grezzi (`info["reward_terms"]`) via `metrics_logger.log_value(key, v,
  reduce="mean", window=100)`. Registrato con `PPOConfig.callbacks(cls)`. Nota: in
  Ray 2.55 `clear_on_reduce` e' **deprecato** → usare `window` o `ema_coeff`.
- `make_tb_writer` + `log_result_scalars`: scrivono uno
  `torch.utils.tensorboard.SummaryWriter` in `<output_dir>/tensorboard`. I tag
  `reward/*` (7: reward, reward_base, tracking/reference/bio_score, penalty,
  safety_term) e `reward_loss/*` (9 loss fisici + u_abs_max, u_saturation_fraction)
  sono **promossi a sezione top-level** in TB (separati da `env_runners/*` e
  `learners/*`). Totale: 96 scalari per iterazione; asse x =
  `num_env_steps_sampled_lifetime`.

Flag `--tensorboard` / `--no-tensorboard` (default on). Avvio:

```
python -m tensorboard.main --logdir runs   # http://localhost:6006
```

## Strategia

- Non modificare l'env ne' il plugin: tutta la logica di combinazione e'
  **downstream** della reward interna dell'env, nel wrapper piu' esterno della
  catena.
- Default bit-exact: nessuna regressione comportamentale senza override espliciti.
- Modulo separato e importabile (`tb_logging.py`, non `__main__`): il callback
  e' picklabile per i worker Ray in Fase B.
- Seam singolo per override: `env_config["reward"]` dict filtrato da `make_cmc_env`
  (la chiave `"reward"` non e' un campo `CMCEnvConfig` e viene ignorata dal filtro
  esistente).

## File modificati

- `Trajectory Generator/baseline_MLP/reward_function.py` — **nuovo**: `RewardConfig`,
  `compute_reward`, `RewardShapingWrapper`, `load_reward_overrides`.
- `Trajectory Generator/baseline_MLP/tb_logging.py` — **nuovo**: `RewardComponentsCallback`,
  `make_tb_writer`, `log_result_scalars`.
- `Trajectory Generator/baseline_MLP/reward_overrides_example.json` — **nuovo**: template
  JSON con tutti i default di `RewardConfig`.
- `Trajectory Generator/baseline_MLP/env_factory.py` — chain wrapper
  `FlattenClipAction → RewardShapingWrapper`; legge `env_config["reward"]`.
- `Trajectory Generator/baseline_MLP/train_ppo_mlp.py` — callback registrato;
  `SummaryWriter` per-iterazione; `--reward-json`; `--tensorboard` /
  `--no-tensorboard`; `reward_config` e `tensorboard_dir` in `summary.json`.
- `Trajectory Generator/baseline_MLP/rollout_eval.py` — `--reward-json`; `reward_config`
  in `rollout_summary.json`.
- `Trajectory Generator/baseline_MLP/README.md` — sezioni Reward e TensorBoard aggiunte;
  tabella file aggiornata.
- `Trajectory Generator/baseline_MLP/commands.txt` — sezioni 9 (TensorBoard) e 10
  (reward custom) aggiunte; compile check esteso ai nuovi moduli.

## Verifiche eseguite

Da `envCMC-rllib`, CWD = repo root:

- `py_compile` su tutti i moduli (`win_runtime`, `_bootstrap`, `env_factory`,
  `reward_function`, `tb_logging`, `train_ppo_mlp`, `rollout_eval`) → OK.
- **Parity unit-test** di `compute_reward` vs formula analitica: `unit_formula_match
  = true`, `unit_safety_subtracted = true`.
- **Parity env-level**: `RewardConfig()` default su step reali → `max|diff| = 0.0`
  con la reward interna dell'env (bit-exact). `wrapper_consistency_max_diff = 0.0`.
- Override e routing: `from_mapping` con chiavi sconosciute ignorata OK; `"reward"`
  dict in `env_config` raggiunge il wrapper con i valori attesi.
- `load_reward_overrides`: inline JSON e `None` OK.
- **Tiny train** con TensorBoard abilitato: `train_iter` OK (return 3.566 →
  invariato), nessun warning `clear_on_reduce`, event file scritto.
- **Ispezione event file**: 96 scalari totali, `reward/reward` (top-level, valore
  0.711 a step 64) e `reward_loss/saturation_loss` presenti; nessun tag
  `env_runners/reward` residuo (promozione top-level corretta).
- `summary.json`: `reward_config` (tutti i 12 campi) e `tensorboard_dir` presenti.

## TODO aperti

- **F2 / reward rebalancing**: ora e' possibile farlo senza toccare l'env. Proposta
  di partenza: abbassare `blend_reference` (0.55 → ~0.30–0.35) per ridurre il
  dominio del "baseline IK" nella reward, alzare `saturation_weight` (0.1 → ~0.5)
  e `blend_tracking` (0.25 → ~0.45) per mordere di piu' sulla dinamica reale. Da
  validare su training di almeno 20–50 iterazioni con le curve TB.
- **Training full-gait**: il training controllato da 50 iterazioni e la
  registrazione TensorBoard sono stati completati il 2026-06-07. Restano da
  validare le curve reward su orizzonti e batch sufficienti a coprire gait
  cycle completi.
- **Gait metrics**: porting RMSE/Symmetry/Trend dalla repo esterna.
- **SNN come RLModule**: rinviato per scelta utente.
