# Daily report - 2026-06-05

## Sintesi

Giornata dedicata alla nuova baseline **MLP single-agent** per il trajectory
generator, separata dal percorso SNN/skrl e basata su **Ray RLlib PPO**. Il
risultato principale e' una pipeline eseguibile end-to-end:

1. `Trajectory Generator/baseline_MLP/` creato come percorso indipendente per
   training PPO MLP sul nostro env CMC-like;
2. env wrappato per RLlib con action space piatto, clipping ai bound e shim
   Windows torch/OpenSim nei worker Ray;
3. reward centralizzata in `reward_function.py`, default bit-exact rispetto
   all'env originale, con override runtime;
4. TensorBoard integrato per reward components, loss fisici dell'env e metriche
   learner/env-runner.

Report utente prodotti oggi:

```text
reports/user/2026-06-05_baseline_mlp_rllib_ppo_single_agent.md
reports/user/2026-06-05_reward_centralizzata_e_tensorboard.md
```

## 1. Baseline MLP RLlib PPO single-agent

Creata una baseline RLlib moderna in
`Trajectory Generator/baseline_MLP/`, usando la repo esterna
`Multi-Agent-RL-control-for-Transfemoral-Amputee` come template concettuale ma
non come codice copiato. La pipeline e' **single-agent**: la policy controlla
solo i due giunti protesici (`pros_knee_angle`, `pros_ankle_angle`), mentre il
lato biologico resta gestito dalla Static Optimization del simulatore.

La SNN resta fuori scope per questa baseline e continua a vivere nel percorso
separato `Trajectory Generator/Prosthesis_SNN/`.

Problemi risolti:

- action space 2D dell'env `(policy_knots, 2)` non accettato da RLlib: aggiunto
  `FlattenClipAction`, che espone un `Box((6,))`, fa reshape a `(3,2)` e clippa
  ai bound prima dello step;
- path relativi rifiutati da `pyarrow` nei checkpoint RLlib: setup XML, output e
  checkpoint risolti sempre a path assoluto;
- fragilita' import torch/OpenSim nei worker Ray su Windows: aggiunto
  `win_runtime.py`, importato per primo e riapplicato con
  `worker_process_setup_hook`.

Validata sia la Fase A single-process (`--num-env-runners 0`) sia la Fase B con
2 worker Ray. Il plugin SEA e il simulatore OpenSim sono rimasti invariati.

## 2. Reward centralizzata e TensorBoard

Introdotto `reward_function.py` come unica fonte della reward per la baseline
MLP. L'env continua a calcolare i loss fisici per-step in
`info["reward_terms"]`; `RewardShapingWrapper` ricalcola lo scalare reward a
valle e sostituisce la reward interna dell'env.

Formula default:

```text
tracking_score  = 1 / (1 + tracking_weight  * tracking_loss)
reference_score = 1 / (1 + reference_weight * reference_loss)
bio_score       = 1 / (1 + bio_weight       * bio_loss)

penalty = effort_weight*effort_loss
        + smoothness_weight*smoothness_loss
        + saturation_weight*saturation_loss

base   = clip(blend_tracking*tracking_score
              + blend_reference*reference_score
              + blend_bio*bio_score
              - penalty, 0, 1)
reward = base - safety_weight*safety_loss
```

I default riproducono la reward originale dell'env in modo bit-exact
(`max|diff| = 0.0` su step reali), quindi non c'e' regressione comportamentale
senza override espliciti.

Aggiunto anche `tb_logging.py`: TensorBoard scrive in
`<output_dir>/tensorboard`, con sezioni top-level `reward/*` e
`reward_loss/*` oltre alle metriche RLlib (`env_runners/*`, `learners/*`).
Gli override reward si passano con `--reward-json` sia in training sia in
rollout, e vengono registrati nei summary.

## File creati / modificati oggi

```text
Trajectory Generator/baseline_MLP/__init__.py
Trajectory Generator/baseline_MLP/_bootstrap.py
Trajectory Generator/baseline_MLP/win_runtime.py
Trajectory Generator/baseline_MLP/env_factory.py
Trajectory Generator/baseline_MLP/train_ppo_mlp.py
Trajectory Generator/baseline_MLP/rollout_eval.py
Trajectory Generator/baseline_MLP/reward_function.py
Trajectory Generator/baseline_MLP/tb_logging.py
Trajectory Generator/baseline_MLP/reward_overrides_example.json
Trajectory Generator/baseline_MLP/commands.txt
Trajectory Generator/baseline_MLP/README.md
.gitignore                                      # aggiunto runs/
```

## Verifiche eseguite

```text
import torch + opensim + ray nello stesso processo       -> OK
ray / torch / opensim / numpy version check              -> OK
make_cmc_env smoke tiny                                  -> obs (58,), action (3,2), reward finita
tiny train Fase A, num_env_runners=0                     -> checkpoint best/last + summary OK
rollout deterministico da checkpoint                     -> action_abs_max 0.67, terminated=true
training parallelo Fase B, num_env_runners=2             -> worker Ray + plugin SEA OK
py_compile moduli baseline_MLP                           -> OK
compute_reward parity unit-test                          -> formula OK, safety sottratta OK
reward wrapper parity vs env originale                   -> max|diff| = 0.0
reward override routing / load_reward_overrides          -> OK
tiny train con TensorBoard                               -> event file scritto, return invariato
ispezione TensorBoard event file                         -> 96 scalari, reward/* e reward_loss/* presenti
summary.json                                             -> reward_config e tensorboard_dir presenti
```

I return riportati nei report sono sanity check su configurazioni tiny
(orizzonti brevi e batch piccoli), non evidenza di apprendimento.

## TODO chiusi oggi

- Baseline MLP/RLlib PPO single-agent creata e verificata end-to-end sul nostro
  env CMC-like.
- F1 **nel ramo baseline_MLP**: azioni clippate ai bound tramite
  `FlattenClipAction`; rollout deterministico con `action_abs_max` entro bound.
- F3 **nel ramo baseline_MLP**: shim Windows torch/OpenSim applicato nel processo
  principale e nei worker Ray; validato con 2 EnvRunner.
- Reward della baseline resa tunabile senza toccare env, runner, Static
  Optimization o plugin SEA.
- TensorBoard integrato nel training baseline con metriche reward/loss leggibili
  sotto l'`output_dir` del run.
- `runs/` aggiunto a `.gitignore` come artefatto riproducibile di training e
  rollout.

## TODO aperti e propagati

### Baseline MLP / RLlib

- **Training reale**: lanciare run lunghi, almeno 20-50 iterazioni iniziali e poi
  50+ iterazioni, per validare curve di apprendimento e stabilita' del training.
- **F2 / reward rebalancing**: usare la nuova `RewardConfig` per ridurre il peso
  del `reference_score` (`blend_reference` circa 0.30-0.35), aumentare
  `blend_tracking` (circa 0.45) e rendere piu' severa la penalita' di saturazione
  (`saturation_weight` circa 0.5). Da validare con TensorBoard e rollout lunghi.
- **Gait metrics**: portare RMSE/NRMSE, Symmetry Angle e Trend Symmetry dal
  notebook `gait_metrics.ipynb` della repo esterna in un modulo riusabile.
- **SNN come RLModule**: eventuale re-incapsulamento della SNN come custom
  RLModule RLlib, rinviato per scelta dell'utente.

### Validazione generatore / SNN (dal 2026-06-04, ancora aperti dove non coperti dalla baseline)

- **F1 percorso SNN/skrl**: clippare la media in inference ai bound dell'azione
  se si continua a usare il generatore SNN.
- **F2**: limitare la magnitudo azione + penalita' saturazione efficace;
  rivalidare il filtro 6 Hz con una policy allenata su rollout lungo.
- **F3 percorso SNN/skrl**: rendere robusto/ordinato l'import
  torch/OpenSim/numpy su Windows anche nel percorso `Prosthesis_SNN`.
- **F4**: `generator.reset()` al reset env nel rollout SNN (`--reset-on-done`).
- **F5**: allineare `terminated`/`truncated` alla convenzione Gymnasium +
  bootstrap GAE sul troncamento.
- **F7**: validare `input_size==len(feature_names)` e architettura su
  resume/load.
- **Reward**: continuare il ribilanciamento in modo che "riprodurre l'IK" non
  sia gia' quasi-ottimo.

### Trajectory Generator / SNN / PPO

- Validare il generatore su rollout lunghi per tracking protesico, qualita'
  traiettoria e comportamento biologico.
- Decidere la tecnica di training (RL puro / hybrid SL+RL / imitazione->RL /
  staged) e definire l'observation space.
- Definire/validare normalizzazione I/O, feature names, unita', scaling, output
  transform, action limits e metadata checkpoint.
- Checkpoint periodici per training lunghi e gestione interruzioni/crash.
- **F6/T2**: irrobustire o rimuovere la cache su `data_ptr` in `actor_critic`.
- **T3**: validare reshape/BPTT prima di passare a `num_envs>1`.
- **T4**: passare lo spike_grad custom anche a rate/latency encoder.
- **S1**: uniformare i reader `.sto` sul flag `inDegrees`.
- **S2/J2**: allineare `sea_stiffness` default e valutare gain per-modello.
- **S3**: riusare `InverseDynamicsSolver` invece di ricrearlo ogni frame.

### Env RL / simulatore

- Verificare il lag causale del filtro 6 Hz sul tracking dell'IK
  (IK a fase-zero, riferimento online no): lead/anticipo o termine reward.
- Eventuale taratura `pros_ref_lpf_cutoff_hz` / `pros_ref_lpf_zeta`.
- Target/bounds biologici della static optimization all'init AB06
  (`capable_share < 1`, warning QP).
- Layout Git: confermare lo spostamento di `osim_trj_cmc_like.py` in
  `Trajectory Generator/` e aggiornare riferimenti/packaging.
- Decidere se includere `LLM_SIMULATOR_OVERVIEW.md` in `CONTEXT.md` o nel flusso
  `start_day`.

### Knowledge base letteratura

- Recuperare il vero paper **Wrapyfi**: il file in `paper/7` e' il paper iCub.
- Approfondire i parziali se utile: **P17** (tesi Berkenkamp, Lyapunov/RoA),
  **P24** (survey RLVR), P14/P19/P23.
- Solo su richiesta: sintesi trasversale per tema o collegamento dei finding
  della letteratura ai TODO del progetto.

### Propagati storici - controllo SEA

- Sweep `Kp_knee_motor` su valori intermedi tra 3.9 e 18 mantenendo ankle best.
- Validare diagnosi di coupling knee-ankle isolando la dinamica knee dal
  feedback ankle.
- Notch a 28 Hz sul feedback knee lato controllore.
- Cleanup modelli sperimentali (`slow_inner_pd_1405`, `pi_asym_knee1405`).
- Windows: build/copia DLL plugin PI completa e documentata.
- Secondo pass knee dello sweep locale.
- Confronto consolidato finale tra configurazioni storiche (PD, PI, cascade,
  retune PI, zeta07, pi-tuned, J_eff, Opzione D, slow inner, asym).
- Pulizia artefatti sweep `_cascade_local_gain_sweep_20260517_233607` e
  `_cascade_local_gain_sweep_20260517_234151`.
- LPF qdot: test asimmetrico solo ankle, cutoff 30/35 Hz e run lunga 30+ s.
