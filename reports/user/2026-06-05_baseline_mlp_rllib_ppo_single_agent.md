# Baseline MLP: pipeline Ray RLlib PPO single-agent sul nostro env CMC-like

## Problema

La repo esterna `Multi-Agent-RL-control-for-Transfemoral-Amputee` (in `Downloads`,
read-only) e' una pipeline RL gia' validata basata su Ray RLlib PPO. L'utente
voleva usarla come **punto di partenza** per il proprio progetto: riutilizzarne
l'orchestrazione di training (loop, parallelismo, checkpoint, eval) ma far
generare le traiettorie protesiche dal **nostro env CMC-like**
(`Trajectory Generator/osim_trj_cmc_like.py`), **single-agent**, con il lato
biologico gestito dalla Static Optimization del simulatore (non da un secondo
agente RL).

Due ostacoli noti dall'analisi a monte:

- **Plugin C++ SEA**: andava confermato non bloccante. Verificato (turni
  precedenti) leggendo la import table del `.dll`: dipende solo da OpenSim
  4.5.x + SimTK + runtime MSVC, **zero dipendenze Python/NumPy**. Con il nostro
  OpenSim-mCMC 4.5.2 (`C:\OpenSim-mCMC`, py3.12) il plugin gira invariato.
- **Collisione dipendenze**: la repo esterna usa `ray 2.34` + **vecchio API
  stack** (ModelV2, `sgd_minibatch_size`, metriche `episode_reward_mean`),
  incompatibile col nostro stack moderno (gymnasium 1.3, numpy 2.4, py3.12,
  torch 2.7). Il codice esterno **non gira as-is**: l'orchestrazione andava
  **riscritta sul nuovo API stack** usando la repo come *template*, non come
  base da copiare.

## Soluzione

Creato un percorso PPO **MLP single-agent** end-to-end (train -> checkpoint ->
rollout/eval) in una nuova cartella git-tracked `Trajectory Generator/baseline_MLP/`,
su un env conda dedicato e sul **nuovo API stack** di Ray. Env, `SimulationRunner`,
Static Optimization, plugin C++ SEA e OpenSim **non sono stati toccati**: la
baseline consuma l'env solo via la sua interfaccia gymnasium pubblica.

La SNN e' esplicitamente **fuori scope** (scelta utente: "non vincolante"); resta
nel percorso separato `Prosthesis_SNN/` (skrl). Questa e' la baseline con MLP
standard di RLlib.

## Strategia

- **Ambiente isolato**: env conda dedicato `envCMC-rllib` = clone di `envCMC-like`
  + `pip install ray[rllib]`. Eredita `opensim.pth -> C:\OpenSim-mCMC` e il
  plugin, isolando Ray dallo stack SNN (skrl/snntorch resta solo in `envCMC-like`).
- **Repo esterna come template, non come base**: riscritta l'orchestrazione sul
  nuovo API stack (RLModule + Learner). `PPOConfig().environment().framework().env_runners().learners().training(minibatch_size, num_epochs, ...).rl_module(DefaultModelConfig(...))`,
  build con `config.build_algo()`, metriche `result["env_runners"]["episode_return_mean"|"episode_len_mean"]`.
- **Path sempre assoluti**: i worker Ray possono avere CWD diversa; setup XML e
  output dir risolti ad assoluto.
- **Staging del rischio Windows**:
  - *Fase A — correttezza*: `--num-env-runners 0`, tutto in un processo, nessun
    worker, nessun problema DLL/CWD. Milestone "funziona end-to-end".
  - *Fase B — parallelismo*: `--num-env-runners N`, shim torch/OpenSim applicato
    in ogni worker via `worker_process_setup_hook`. Fallback documentato a
    `num_env_runners=0` se il clash dovesse persistere.

### Gotcha risolti (non ovvi, ricorrenti)

1. **Action space 2D non supportata da RLlib**: l'env espone `Box(policy_knots,2)`
   = `(3,2)` e RLlib la rifiuta ("Action space has multiple dimensions").
   Risolto con un wrapper `FlattenClipAction(gym.ActionWrapper)` che espone a
   RLlib un `Box((6,))` piatto e, prima dello step, fa reshape -> `(3,2)` + clip
   ai bound. **Risolve anche il finding F1** del 2026-06-04 (l'env valida ma non
   clippa): le azioni arrivano al simulatore sempre entro i bound.
2. **pyarrow rifiuta i path relativi**: `algo.save_to_path` e
   `RLModule.from_checkpoint` passano per `pyarrow.fs.FileSystem.from_uri`, che
   rifiuta i path relativi ("URI has empty scheme"). Risolto risolvendo a path
   assoluto (`.resolve()`) ovunque. Su questo Windows il drive `C:` non viene
   frainteso come scheme.
3. **OMP/fbgemm nei worker Ray (collegato a F3)**: ogni EnvRunner carica sia
   torch (policy) sia opensim (env). Lo shim `win_runtime` (toglie OpenSim dal
   PATH, importa torch, ripristina via `add_dll_directory`, `KMP_DUPLICATE_LIB_OK=TRUE`)
   e' importato per primo nel package e ri-applicato in ogni worker via
   `ray.init(runtime_env={"worker_process_setup_hook": _worker_setup, ...})`.

## File creati

In `Trajectory Generator/baseline_MLP/`:

- `__init__.py` — importa per primo `win_runtime`, poi `_bootstrap.ensure_sim_paths()`.
- `win_runtime.py` — shim Windows OMP/OpenSim autonomo (non importa snntorch/skrl):
  importa torch con OpenSim tolto dal PATH, poi ripristina; `apply()` idempotente.
- `_bootstrap.py` — mette `baseline_MLP/`, `Trajectory Generator/` e la repo root
  su `sys.path` risolvendo da `__file__`.
- `env_factory.py` — `FlattenClipAction` (wrapper 2D->1D + clip), `build_env_config`,
  `make_cmc_env` (RLlib env creator), `register_cmc_env` (`register_env("cmc_traj_env", ...)`).
- `train_ppo_mlp.py` — entrypoint training (nuovo API stack): loop `algo.train()`,
  checkpoint best/last + RLModule inference-only, `summary.json`, gestione
  `KeyboardInterrupt`, worker setup hook per la Fase B.
- `rollout_eval.py` — carica l'RLModule da checkpoint, rollout **deterministico**
  (greedy via `forward_inference` -> `to_deterministic().sample()`), metriche
  (return, action_abs_max, pelvis_ty_min, terminated/truncated); con
  `--record-outputs` salva .sto per `visualize.py`.
- `commands.txt` — comandi PowerShell pronti per `envCMC-rllib` (setup, smoke,
  tiny train, rollout, train standard, train parallelo).
- `README.md` — scopo, stack, setup, uso, staging e confine (env/plugin/SNN non
  toccati).

## File modificati

- `.gitignore` — aggiunto `runs/` (output di training/rollout: checkpoint,
  summary, riproducibili).

## Verifiche eseguite

Da `envCMC-rllib`, CWD = repo root, setup XML
`models\AB06_SEASEA_Threadmill\AB06_SEASEA_stiff321_500_pi_setup.xml`:

- **Import coesistenza** (de-risk OMP): torch + opensim + ray nello stesso
  processo, import OK; ray 2.55.1, torch 2.7.0+cpu, opensim 4.5.2, numpy 2.4.3.
  Nota: ray declassa gymnasium 1.3.0 -> 1.2.2 (innocuo: l'env usa solo `Env` +
  `spaces.Box`).
- **Env smoke**: `make_cmc_env({...tiny...})` -> obs `(58,)`, action `(3,2)`,
  reward 0.733, nessuna eccezione.
- **Tiny train Fase A** (`--num-env-runners 0`, 1 iterazione): return 3.57,
  checkpoint best/last + rl_module + `summary.json` scritti.
- **Rollout deterministico** da checkpoint: return 4.47, `action_abs_max` 0.67
  (entro bound -> conferma del clipping F1), `terminated=true`.
- **Parallelo Fase B** (`--num-env-runners 2`): entrambi i worker Ray caricano il
  plugin SEA **senza crash OMP**, return 3.49 -> shim + path assoluti validati
  nei processi figli.
- `runs/` aggiunto a `.gitignore`; artefatti smoke rimossi.

> Nota: i valori di return sopra sono **sanity check su config tiny** (orizzonti
> 0.05 s, batch 64), non curve di apprendimento. Servono a dimostrare che la
> pipeline gira end-to-end, non che la policy ha imparato.

## Risultato

La baseline MLP e' **eseguibile e verificata end-to-end** sul nostro env CMC-like,
sia in single-process (Fase A) sia in parallelo a 2 worker (Fase B), sul nostro
OpenSim validato, senza toccare env/plugin/SO/OpenSim. I tre gotcha Windows/RLlib
piu' insidiosi (action 2D, path pyarrow, OMP nei worker) sono risolti e
documentati. Il finding **F1** (azione non clippata in inference) e **F3**
(fragilita' import torch/OpenSim su Windows) del 2026-06-04 risultano gestiti in
questa pipeline.

## Aggiornamento 2026-06-07

Il training controllato `runs/_baseline_mlp_50iter_online_sensor` ha completato
`50/50` iterazioni reali con un EnvRunner, timeout rigidi e cleanup completo.
Return, critic, entropy e `end_reason` sono stati registrati. Il return medio
passa da `2.4356` nelle prime 10 iterazioni a `2.6363` nelle ultime 10; il
critic resta debole (`vf_explained_var` media ultime 10 `-0.0042`). Sono stati
osservati `48` `episode_time_limit` e `2`
`joint_divergence_pros_knee_angle`.

## TODO aperti

- **Training full-gait**: il requisito delle 50 iterazioni è soddisfatto per la
  validazione controllata della pipeline. Restano da eseguire run con orizzonti
  e batch sufficienti a coprire gait cycle completi, migliorando il critic e
  riducendo le terminazioni unsafe.
- **F2 / reward**: ribilanciare la reward (saturazione SEA, peso `reference_score`
  troppo dominante ~0.55 -> ~0.90 reward gia' senza apprendimento) cosi' che
  "riprodurre l'IK" non sia gia' quasi-ottimo; rivalidare il filtro 6 Hz con una
  policy allenata su rollout lungo. (Reward dell'env qui invariata.)
- **Gait metrics**: porting delle metriche di qualita' del passo
  (RMSE/NRMSE, Symmetry Angle, Trend Symmetry) dal notebook `gait_metrics.ipynb`
  della repo esterna come modulo riusabile.
- **SNN come RLModule**: re-incapsulamento della SNN come custom RLModule di
  RLlib — rinviato per scelta dell'utente.
