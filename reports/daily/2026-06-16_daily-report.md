# 2026-06-16 - Daily report

## Sintesi

La giornata e' stata centrata sulla pipeline baseline MLP imitation e sulla
diagnostica del rollout con observation space allargato. Il lavoro ha portato a
tre risultati principali:

- pipeline training/rollout/plot/visualize piu automatica e organizzata;
- reward del training MLP resa interamente adimensionale;
- resume del training dal best V4 con reward normalizzata, arrivato a un best
  superiore rispetto alla run precedente.

Non e' stata modificata la semantica del plugin C++ SEA.

## Report utente consolidati

Report del 16/06 raccolti in questo daily:

- `reports/user/2026-06-16_baseline_mlp_run_layout_e_comandi_snelli.md`
- `reports/user/2026-06-16_pipeline_mlp_training_rollout_plotter_visualizer.md`
- `reports/user/2026-06-16_imitation_loss_normalizzata_e_value_tuning.md`
- `reports/user/2026-06-16_reward_adimensionale_normalizzazione_completa.md`

## Pipeline MLP e layout run

E' stata introdotta una convenzione stabile per gli artifact della baseline MLP:

- training sotto `Trajectory Generator/runs/training`;
- rollout sotto `Trajectory Generator/runs/rollout`;
- video visualizer sotto `Trajectory Generator/runs/visualize`;
- plot PNG nelle cartelle `plot/MM_DD_YYYY_N`.

Comandi snelli principali:

```powershell
python "Trajectory Generator\baseline_MLP\train_ppo_mlp.py"
python "Trajectory Generator\baseline_MLP\rollout_eval.py"
python plot/plotter.py --mlp
python visualize.py --mlp
```

Il training usa di default `Trajectory Generator/baseline_MLP/training_cfg.yaml`
e genera nomi del tipo `MLP_[strategy]_training_[MM-DD-YYYY]`. Il rollout, se
non riceve un checkpoint esplicito, sceglie l'ultimo training valido e usa
`rl_module_best`. I run falliti o interrotti vengono ignorati.

Il plotter `--mlp` ora legge automaticamente l'ultimo rollout valido e ha una
Figure 7 dedicata a policy C2 servita vs sound-leg target shiftato, sound leg raw
ed errore target-policy. Il visualizer `--mlp` legge automaticamente
`rollout_episode_kinematics.sto` dall'ultimo rollout valido e puo salvare video
con `--save`, anche su Windows tramite `PIL.ImageGrab`.

## Diagnostica rollout imitation FullObs

L'analisi del rollout V4 con observation space allargato ha isolato due problemi
principali:

- errore policy/served reference rispetto alla sound leg, soprattutto ankle;
- chattering del motor angle, soprattutto knee.

L'errore ankle nasce gia nella traiettoria generata/servita dalla policy, non nel
tracking SEA a valle. Nel rollout analizzato l'ankle risultava compresso verso
la media: il range target era circa `0.355 rad`, mentre il range served era circa
`0.179 rad`. Questo ha motivato la normalizzazione per coordinata della imitation
loss, per evitare che il knee domini la loss solo per scala fisiologica piu
ampia.

Il chattering del motor angle e' stato interpretato come effetto secondario da
monitorare insieme a command-rate, SEA stress e served reference, piuttosto che
come causa primaria dell'errore policy-sound leg.

## Imitation loss e value tuning

La imitation loss nell'env RL e' stata normalizzata per posizione e velocita,
separatamente per knee e ankle:

```text
position_loss = ((target_q - current_or_served_q) / target_q_range)^2
velocity_loss = ((target_qdot - current_or_served_qdot) / target_qdot_range)^2
```

I range sono calcolati dalla reference/target caricata dall'ambiente, quindi non
sono hardcoded sul modello AB06.

Sono stati esposti nello YAML anche:

```yaml
ppo:
  vf_clip_param: 10.0
  vf_loss_coeff: 1.0
```

I pesi imitation sono stati poi spostati nel blocco `reward:` durante il lavoro
successivo sulla reward adimensionale. A fine giornata i valori scelti sono:

```yaml
reward:
  imitation_knee_position_weight: 1.0
  imitation_ankle_position_weight: 2.0
  imitation_knee_velocity_weight: 0.02
  imitation_ankle_velocity_weight: 0.04
```

La scelta raddoppia l'attenzione sull'ankle, coerentemente con il problema di
ampiezza/compressione osservato, senza rendere ancora l'ankle schiacciante.

## Reward adimensionale

La reward MLP ora usa termini adimensionali per tutte le loss che entrano nella
reward scalare. Le diagnostiche fisiche raw restano disponibili per debug, ma
non vengono moltiplicate direttamente dai pesi reward.

Modifiche principali:

- `tracking_loss`: posizione e velocita protesiche normalizzate con range
  reference per coordinata;
- `reference_loss`: errore rispetto alla reference/base IK normalizzato;
- `bio_loss`: coordinate biologiche normalizzate con range reference robusti;
- `smoothness_loss`: basata sul movimento fisico intra-segmento dei knot futuri;
- `grf_penetration_loss`: normalizzata tra soglia soft e soglia termination;
- `out_of_band_loss`: normalizzata rispetto all'ampiezza della banda ammessa;
- imitation losses: gia normalizzate, con pesi knee/ankle nel blocco `reward:`.

Nuovi normalizer fisici minimi esposti in `simulation:`:

```yaml
simulation:
  reward_reference_range_floor: 0.05
  reward_reference_velocity_range_floor: 0.1
```

## File modificati principali

- `Trajectory Generator/osim_trj_cmc_like.py`
- `Trajectory Generator/baseline_MLP/reward_function.py`
- `Trajectory Generator/baseline_MLP/env_factory.py`
- `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`
- `Trajectory Generator/baseline_MLP/rollout_eval.py`
- `Trajectory Generator/baseline_MLP/training_config.py`
- `Trajectory Generator/baseline_MLP/tb_logging.py`
- `Trajectory Generator/baseline_MLP/training_cfg.yaml`
- `Trajectory Generator/baseline_MLP/training_cfg.v4_imitation.yaml`
- `Trajectory Generator/baseline_MLP/commands.txt`
- `Trajectory Generator/baseline_MLP/README.md`
- `Trajectory Generator/baseline_MLP/imitation_oracle_rollout.py`
- `plot/plotter.py`
- `visualize.py`

## Verifiche eseguite

Verifiche principali completate:

- `py_compile` sui file Python toccati nella pipeline MLP, plotter e visualizer;
- test dei resolver training: default output, `reward_mode`, collisioni,
  `--name`, precedenza `--output-dir`;
- test dei resolver rollout: ultimo training valido, esclusione run falliti,
  naming automatico, `--name`, `--no-record-outputs`, override manuali;
- test plotter `python plot/plotter.py --mlp` con generazione Figure 7;
- test visualizer `python visualize.py --help`;
- smoke video reale con `visualize.py --mlp --save` in ambiente OpenSim;
- loader YAML per nuovi campi PPO, normalizer reward e pesi imitation;
- compatibilita RLlib PPO per `vf_clip_param` e `vf_loss_coeff`;
- smoke env AB06 tramite `env_factory.make_cmc_env`;
- smoke `out_of_band_loss`: dentro banda `0.0`, errore pari a una banda `1.0`;
- `git diff --check`, con soli warning CRLF/LF attesi su Windows.

Smoke env dopo reward adimensionale:

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

## Training e resume

La run precedente valida era:

```text
runs/training/baseline_mlp_imit_v4_c2_4hz_obs_target
best iteration: 40
best_episode_return_mean: 244.08135666176585
```

E' stato poi eseguito un resume con reward normalizzata:

```powershell
python "Trajectory Generator\baseline_MLP\train_ppo_mlp.py" --resume-from "runs\training\baseline_mlp_imit_v4_c2_4hz_obs_target\checkpoint_best" --output-dir "runs\training\baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm" --iterations 60
```

Risultato della run ripresa:

```text
run: baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm
status: completed
iterations_run: 20
iterations_completed: 60
best iteration: 59
best_episode_return_mean: 277.75698812995
iteration 60 return: 270.77606079304195
```

Il best e' quindi migliorato di circa `+33.7` punti rispetto alla run V4
precedente. La curva suggerisce ancora margine di miglioramento, perche dopo lo
shock iniziale del resume con reward nuova la policy ha recuperato e ha raggiunto
il best vicino alla fine della finestra.

Comando consigliato per prolungare di circa 20 iterazioni dal best `59`:

```powershell
python "Trajectory Generator\baseline_MLP\train_ppo_mlp.py" --resume-from "runs\training\baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm\checkpoint_best" --output-dir "runs\training\baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm_plus20" --iterations 79
```

Comando rollout dal best della run ripresa:

```powershell
python "Trajectory Generator\baseline_MLP\rollout_eval.py" --checkpoint "runs\training\baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm\rl_module_best" --output-dir "runs\rollout\baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm_rollout" --record-outputs
```

Nota: per il training resume si usa `checkpoint_best`; per rollout/inferenza si
usa `rl_module_best`.

## TODO chiusi o superseduti il 16/06

- [x] Lanciare e analizzare il training imitativo V4 con observation space target
      allargato.
- [x] Eseguire rollout e confrontare policy/served reference con sound-leg target.
- [x] Diagnosticare la compressione dell'ankle rispetto al target.
- [x] Esporre `vf_clip_param` e `vf_loss_coeff`.
- [x] Normalizzare la imitation loss per range di posizione e velocita.
- [x] Rendere la reward scalare interamente adimensionale.
- [x] Spostare i pesi imitation knee/ankle nel blocco `reward:`.
- [x] Rendere training, rollout, plotter e visualizer piu automatici per uso
      quotidiano.

## TODO aperti e propagati

### Training imitation e rollout

- [ ] Prolungare il training dal best della run normalizzata per altre circa 20
      iterazioni e confrontare il nuovo best con iteration `59`.
- [ ] Valutare anche un training da zero con reward normalizzata, per capire se
      il resume dal best vecchio introduce bias o local optimum ereditati.
- [ ] Dopo il rollout dal nuovo best, confrontare `served-target`,
      `actual-target`, saturazione SEA, clipping/aggressivita del comando e forma
      cinematica rispetto al target sano anti-fase.
- [ ] Controllare in TensorBoard `sound_imitation_loss`,
      `served_imitation_loss`, sottotermini position/velocity knee/ankle,
      `tracking_position_loss`, `tracking_velocity_loss`,
      `grf_penetration_loss` e `segment_delta_loss`.

### Forma della served reference

- [ ] Verificare se i pesi ankle `2.0 / 0.04` recuperano ampiezza senza creare
      overshoot o peggiorare knee.
- [ ] Indagare ancora la polarizzazione/compressione della served reference
      ankle dopo il primo secondo.
- [ ] Valutare una penalita inter-step esplicita su `q_cmd(t)-q_cmd(t-1)` o
      sull'endpoint consecutivo solo se il chattering raw degrada served
      reference, SEA o stabilita numerica.
- [ ] Valutare `lam: 0.95` in un'ablation separata se il credito temporale resta
      troppo locale.
- [ ] Eseguire separatamente un training reale asymmetric actor-critic da zero.

### Reset, fase e stati iniziali

- [ ] Implementare una selezione robusta degli stati iniziali che verifichi
      compatibilita fra posa, velocita, target imitativo e pattern di contatto.
- [ ] Escludere o gestire esplicitamente il tratto precedente al primo heel
      strike invece di affidarsi alla retro-estrapolazione.
- [ ] Prima di abilitare `random_init=true`, validare molte gait phase e
      rifiutare stati iniziali dinamicamente incoerenti.
- [ ] Valutare se mantenere il target periodico medio oppure costruire target
      condizionati anche sullo stato di contatto.

### Dinamica, contatto e reserve

- [ ] Valutare separatamente penetrazioni GRF e uso elevato delle reserve, senza
      confonderli con il tracking SEA.
- [ ] Investigare reserve biologiche elevate e richieste sui DOF non attuati.
- [ ] Investigare il FAIL di tracking biologico `mtp_angle_r`.
- [ ] Validare heel-strike online, `in_contact`, rocker/COP push-off e contatto
      online del lato sano.
- [ ] Ridurre carico e penetrazione del piede protesico senza perdere contatto.

### Ex-novo, runtime e filoni storici

- [ ] Progettare e validare la reward ex-novo task-based prima del warm-start.
- [ ] Confermare formalmente impulso propulsivo protesico e coordinazione
      inter-limb come obiettivo primario V1.
- [ ] Implementare prima le nuove metriche ex-novo in modalita diagnostica.
- [ ] Monitorare nei training futuri `vf_explained_var`, entropy,
      predetto-vs-return e saturazione delle azioni.
- [ ] Validare auto-recovery durante un crash nativo Ray reale.
- [ ] Verificare su macOS arm64 cleanup, resume, RLModule e reward mode.
- [ ] Pulire launcher/log temporanei `run_imit_*.ps1` e `imit_*.log`.
- [ ] Pulire artefatti temporanei `results/_*`, `validation/_*` e smoke V4
      quando non piu necessari.
- [ ] Proseguire i TODO SNN/skrl propagati e i TODO storici SEA ancora
      applicabili.

### Porting MuJoCo/MJX

- [ ] Verificare lo stato VCS di `C:\Users\tomma\Desktop\MuJoCo_env` prima di
      iniziare il porting.
- [ ] Creare la matrice obbligatoria
      `trajectory_generator/baseline_mlp/feature_parity.yaml`.
- [ ] Eseguire la Fase 0 e produrre il GO/NO-GO GPU/hybrid.
- [ ] Generare e congelare gli oracle OpenSim canonici.
- [ ] Chiudere gate statici, SEA, replay strict e contatto hybrid.
- [ ] Costruire l'ambiente JAX/MJX batched e chiudere i benchmark engine.
- [ ] Integrare PPO JAX, rollout, supervisione, checkpoint e UX.
- [ ] Chiudere feature parity e gate finali scientifici, prestazionali e
      cross-platform.

