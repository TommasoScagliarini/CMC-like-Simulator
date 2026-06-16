# baseline_MLP — PPO (Ray RLlib) sul nostro env CMC-like

Baseline di reinforcement learning **single-agent** che usa **Ray RLlib PPO**
(nuovo API stack, RLModule MLP di default) per allenare una policy a generare i
**segmenti di traiettoria protesica** del nostro env CMC-like
(`Trajectory Generator/osim_trj_cmc_like.py`).

Nasce dall'idea della repo `Multi-Agent-RL-control-for-Transfemoral-Amputee`
(usata come *template* di orchestrazione), ma:

- è **single-agent**: la policy controlla solo i due giunti protesici
  (`pros_knee_angle`, `pros_ankle_angle`); il lato biologico resta muscle-driven
  via **Static Optimization** dentro il simulatore (non un secondo agente RL);
- gira sul **nostro stack moderno** (OpenSim-mCMC 4.5.2 / Python 3.12 / numpy 2 /
  torch 2.7) con **Ray 2.55** sul *nuovo* API stack — non sul vecchio stack
  (ModelV2) della repo originale, incompatibile con queste dipendenze;
- mantiene separati `SimulationRunner`, Static Optimization, plugin C++ SEA e
  OpenSim; la baseline consuma l'adapter condiviso solo tramite l'interfaccia
  gymnasium pubblica.

> La SNN è **fuori scope** qui (scelta dell'utente: "non vincolante"). Questa è
> la baseline con MLP standard di RLlib. La SNN resta nel percorso
> `Prosthesis_SNN/` (skrl), separato.

## Ambiente

Env conda dedicato **`envCMC-rllib`**, clone di `envCMC-like` + `ray[rllib]`:

```powershell
C:\Users\tomma\anaconda3\Scripts\conda.exe create --clone envCMC-like -n envCMC-rllib -y
C:\Users\tomma\anaconda3\Scripts\conda.exe run -n envCMC-rllib python -m pip install "ray[rllib]"
```

Eredita `opensim.pth → C:\OpenSim-mCMC` e il plugin SEA, isolando Ray dallo
stack SNN (skrl/snntorch resta solo in `envCMC-like`).

## File

| File | Ruolo |
|---|---|
| `win_runtime.py` | Shim Windows: importa torch con OpenSim tolto dal PATH (evita il clash OpenMP/fbgemm), poi ripristina OpenSim. Va importato per primo. |
| `_bootstrap.py` | Mette `baseline_MLP/`, `Trajectory Generator/` e la repo root su `sys.path`. |
| `env_factory.py` | `make_cmc_env(dict)` (RLlib env creator, con `FlattenClipAction` + `RewardShapingWrapper`) + `register_cmc_env()`. |
| `reward_function.py` | **Unica fonte della reward**: `RewardConfig` + `compute_reward(losses, cfg)` + `RewardShapingWrapper`. Ricalcola la reward dai *loss* esposti dall'env e la sostituisce. |
| `training_cfg.yaml` | **Sorgente unica dei parametri** (rete + simulazione + PPO + Ray + reward + supervisione). Letto come default da training e (per match) dal rollout. |
| `training_config.py` | Loader del YAML: `load`, `to_argparse_defaults` (YAML → default argparse), `dump_resolved` (snapshot del run) e `load_resolved_for_checkpoint` (auto-match in rollout). |
| `asymmetric_rl_module.py` | Custom RLModule per l'asymmetric actor-critic: policy su `obs[:n_actor]`, value sul vettore pieno. |
| `tb_logging.py` | TensorBoard: `RewardComponentsCallback` (componenti reward/loss → metriche env-runner) + `SummaryWriter` su `<output_dir>/tensorboard`. |
| `train_ppo_mlp.py` | Entrypoint training supervisionato: timeout rigidi e cleanup dell'albero Ray, `PPOConfig` + loop `algo.train()` + checkpoint best/last + RLModule inference-only + `summary.json` + TensorBoard. |
| `rollout_eval.py` | Carica l'RLModule nel figlio supervisionato, rollout deterministico, heartbeat per-step e metriche; di default salva .sto per `visualize.py` (`--no-record-outputs` per metriche leggere). |
| `process_watchdog.py` | Supervisore cross-platform con heartbeat, timeout di startup/stallo/run e self-test su figlio volutamente bloccato. |
| `validate_online_grf_train_inference.py` | Gate profilo-specifico e smoke brevi del contratto onlineGRF usato da training/inference. |
| `commands.txt` | Comandi PowerShell pronti (setup, smoke, train, rollout, tensorboard, reward custom). |

## Configurazione (`training_cfg.yaml`)

Tutti i parametri di **rete** e **simulazione** vivono in un unico file,
`training_cfg.yaml`, che è la **sorgente di riferimento** per lanciare un
training. Il comando default non richiede flag: output-dir e nome run vengono
generati automaticamente; eventuali flag CLI restano override.

- **Precedenza**: il YAML fornisce i *default*; un flag CLI esplicito (es.
  `--lr 5e-4`) **vince** sul YAML. Per una config alternativa (es. macOS) usare
  `--config <file.yaml>`.
- **Sezioni**: `model` (rete), `ppo`, `parallelism` (Ray), `simulation` (env),
  `grf`, `reward`, `supervision` (timeout/restart), `logging`, `run`.
- **Rete come num/dim**: si esprime con `num_hidden_layers` e `dim_hidden_layers`
  (larghezza uniforme) → `fcnet_hiddens = [dim] * num`; `fcnet_activation` è
  configurabile (default `tanh`). Override CLI: `--num-hidden-layers`,
  `--dim-hidden-layers`, `--fcnet-activation`.
- **Asymmetric actor-critic**: flag `--asymmetric-actor-critic` (chiave YAML
  `model.asymmetric_actor_critic`; alias deprecato `--critic-privileged-observation`).
- **`action_mode`**: `absolute` è l'unica modalità di produzione e **non** è nel
  YAML; `delta`/`raw` restano flag CLI solo per diagnostica.
- **Layout risultati**: training in `Trajectory Generator/runs/training`;
  rollout e oracle diagnostici in `Trajectory Generator/runs/rollout`.
- **Nome training default**: `MLP_[strategy]_training_[MM-DD-YYYY]`, con
  `strategy` derivata da `reward.reward_mode` (`imitation` oppure `ExNovo`).
  `--name _suffix` aggiunge un suffisso custom; se il nome esiste gia vengono
  usati `_02`, `_03`, ecc.
- **Nome rollout default**: se non passi `--checkpoint`, viene usato l'ultimo
  training valido in `runs/training`; la cartella rollout copia il nome del
  training sostituendo `training` con `rollout` (oppure aggiungendo `_rollout`
  ai run legacy). Anche qui `--name _suffix` e collisioni `_02`, `_03` sono
  gestiti automaticamente.
- **Snapshot + rollout auto-match**: il training scrive la config risolta (YAML +
  override) in `<output_dir>/training_cfg.resolved.yaml`. `rollout_eval.py`
  sceglie l'ultimo checkpoint `rl_module_best` se `--checkpoint` manca e carica
  automaticamente lo snapshot dalla cartella del run, cosi env/rete/reward/
  `action_mode` combaciano sempre con il training senza ri-specificare i flag.
  `--checkpoint` resta l'override esplicito, `--no-auto-config` disattiva
  l'auto-load, `--config` forza un file; i checkpoint **legacy** senza snapshot
  richiedono i flag a mano.

## Uso rapido

Dalla **root** del simulatore (vedi `commands.txt` per la lista completa):

```powershell
# training di riferimento: tutti i parametri da training_cfg.yaml
python "Trajectory Generator\baseline_MLP\train_ppo_mlp.py"

# training con suffisso custom sul nome auto-generato
python "Trajectory Generator\baseline_MLP\train_ppo_mlp.py" --name _example

# tiny train (single-process): override del YAML per un run rapido
... python "Trajectory Generator\baseline_MLP\train_ppo_mlp.py" --num-env-runners 0 --iterations 1 --train-batch-size 4 --minibatch-size 4 --num-epochs 1 --episode-duration 0.08 --segment-duration 0.02 --num-hidden-layers 2 --dim-hidden-layers 64 --startup-timeout-s 90 --iteration-timeout-s 90 --name _tiny

# resume manuale da un checkpoint completo RLlib
... python "Trajectory Generator\baseline_MLP\train_ppo_mlp.py" --resume-from "Trajectory Generator\runs\training\_baseline_mlp_tiny\checkpoint_last" --iterations 2 --output-dir runs\training\_baseline_mlp_tiny

# rollout di riferimento: ultimo training valido, output .sto completi
python "Trajectory Generator\baseline_MLP\rollout_eval.py"

# rollout leggero senza .sto
python "Trajectory Generator\baseline_MLP\rollout_eval.py" --no-record-outputs

# rollout manuale da checkpoint specifico
... python "Trajectory Generator\baseline_MLP\rollout_eval.py" --checkpoint runs\training\_baseline_mlp_tiny\rl_module_last --output-dir runs\rollout\_baseline_mlp_tiny_rollout --episode-duration 0.05
```

## Staging (riduce il rischio su Windows)

- L'entrypoint avvia il training in un processo figlio supervisionato. Se
  `algo.train()` supera `--iteration-timeout-s`, il child scrive
  `stop_reason="iteration_timeout"` ed esce; il supervisor elimina l'intero
  process tree Ray e rilancia un child fresco da `checkpoint_last`, avanzando
  alla successiva iterazione logica. Dopo `--max-consecutive-skips` timeout
  consecutivi il run termina con `aborted_consecutive_skips`.
- Se il child termina per un crash nativo/non riportato (per esempio una access
  violation di Ray), il supervisor riconosce che `summary.json` non è stata
  aggiornata, elimina gli eventuali discendenti Ray e ritenta la stessa
  iterazione da `checkpoint_last`. `--max-consecutive-crash-restarts` limita i
  tentativi senza avanzamento del checkpoint.
- `--resume-from <checkpoint>` ripristina manualmente lo stato completo RLlib.
  Usare `--checkpoint-every 1` per perdere al massimo una singola iterazione in
  caso di restart.
- `rollout_eval.py` importa Torch/RLlib/OpenSim solo nel figlio supervisionato
  e pubblica un heartbeat per ogni reset/step. Prima di run lunghi, eseguire il
  self-test di `process_watchdog.py` e il gate
  `validate_online_grf_train_inference.py`.
- Ray vede per default solo `num_env_runners + 1` CPU, configurabile con
  `--ray-num-cpus`, evitando worker inattivi per tutte le CPU host.
- I log standalone RLlib sono reindirizzati in `<output_dir>/rllib`; non viene
  più usato `~/ray_results`.
- **Fase A — correttezza**: `--num-env-runners 0`. Tutto in un processo, nessun
  worker Ray, nessun problema DLL/CWD. È il milestone "funziona end-to-end".
- **Fase B — parallelismo**: `--num-env-runners N`. Ogni EnvRunner applica lo
  shim torch/OpenSim via `worker_process_setup_hook`; i path del setup XML sono
  risolti ad assoluti. Se il clash torch/OpenSim nei worker dovesse persistere,
  fallback a `--num-env-runners 0` (più lento ma corretto).

## Robustezza anti-stallo e progress (dal 2026-06-09)

Tre difese a livelli contro l'episodio degenere che blocca il sampling sincrono
(causa dello spreco notturno del 2026-06-08):

- **Guardia wall-time per env-step (`--step-wall-timeout-s`, default 30 s).** Un
  singolo segmento di simulazione patologicamente lento (es. fallback bounded
  least-squares della Static Optimization) viene troncato con grazia
  (`end_reason="step_wall_timeout"`, conteggiato in `episode_end/*`), **anche con
  `fail_fast=True`**, così un worker lento non gata l'intera iterazione. Vale per
  training e rollout. `0` disabilita.
- **Self-guard monotonic del child (`--child-self-timeout`, default ON).** Un
  thread daemon hard-exita il processo di training (codice 124, dump degli stack
  in `faulthandler.log`, `summary.json` con `iteration_timeout`) quando
  `algo.train()` sfora il budget, anche se il main thread è bloccato in
  `ray.wait`. Il clock monotonic evita falsi timeout durante sleep/sospensione.
- **Supervisor restart da checkpoint.** Dopo un `iteration_timeout` termina
  l'intero albero del child, registra lo skip in `supervisor_state.json` e
  riparte da `checkpoint_last`. Dopo un crash nativo/non riportato ritenta
  invece la stessa iterazione, con limite anti-loop. Non tenta più di
  interrompere `algo.train()` uccidendo soltanto gli EnvRunner, perché RLlib li
  ricrea automaticamente.

- **Progress bar live (`--progress`, default ON).** Barra in-place con percentuale,
  contatore iterazioni/step, elapsed ed ETA, aggiornata anche durante i lunghi
  `algo.train()` (refresh in place su TTY, update periodici se rediretto a file;
  fallback ASCII su console senza Unicode). Le metriche complete per-iterazione
  vanno in `<output_dir>/train_iterations.jsonl`; il rollout mostra step/ETA e la
  riga finale con `end_reason`. `--no-progress` per logging semplice.

## GRF online e gait cycle

La baseline MLP/RLlib usa **di default** la modalità `online_sensor`: le GRF
prescribed continuano a guidare la dinamica, mentre le GRF online vengono usate
come sensore dalla rete durante training e inference.

```powershell
... train_ppo_mlp.py ...
```

Il profilo predefinito è
`online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_physical_basis_10mm_balanced.json`.
In modalita `online_sensor` supera i criteri plugin/sensore e mantiene la
penetrazione holdout sotto `15 mm`. Non e ancora autorizzato in modalita
`online` attiva: il gate fisico completo fallisce per reserve `pelvis_ty`
active/sensor p95 pari a `5.94x` rispetto al limite `1.5x`.
Il precedente `online_sensor_basis` non supera questo gate per nuovi run: pur
avendo un buon fit della forza, raggiunge circa `102 mm` di penetrazione
sinistra.
L'osservazione aggiunge per lato:

- GRF normale normalizzata per body weight e stato di contatto;
- impulsi per-step `heel_strike` e `toe_off`;
- durata dell'ultimo gait cycle completo;
- fase del gait cycle stimata dall'ultimo heel strike.

Il payload completo resta disponibile in `info["online_grf"]`,
`info["online_events"]` e `info["online_gait"]`. Gli eventi e le GRF normalizzate
sono anche registrati sotto `gait/*` in TensorBoard.

Training e rollout di uno stesso checkpoint usano gli stessi flag GRF e lo stesso
profilo: poiché `rollout_eval.py` **auto-carica** la config risolta dal run
(`training_cfg.resolved.yaml`), la corrispondenza è automatica. I checkpoint
legacy senza snapshot richiedono esplicitamente `--grf-mode prescribed
--no-online-grf-observation`. Questa integrazione non modifica ancora la reward.

Per test diagnostici e possibile rimuovere dalla dinamica la `ExternalForce`
prescribed di un lato, mantenendo comunque i dati prescribed caricati come oracle:

```powershell
... train_ppo_mlp.py ... --grf-mode online_sensor --disable-prescribed-grf-side left
... rollout_eval.py ... --grf-mode online_sensor --disable-prescribed-grf-side left
```

Il flag e ripetibile per i due lati. Non azzera il sensore GRF online: il sensore
continua a stimare il contatto virtuale, ma la forza prescribed selezionata non
viene applicata al modello. Questa modalita puo quindi evidenziare compensazioni
non fisiche tramite reserve; la reward corrente non penalizza direttamente ne le
reserve ne la coerenza tra GRF online e forze applicate.

## Reward (`reward_function.py`)

La reward vista dall'agente è definita **solo** in `reward_function.py` e usata da
tutta la baseline (training e rollout) tramite `RewardShapingWrapper`, applicato in
`make_cmc_env`. Confine netto:

- **l'env** (`osim_trj_cmc_like.py`) calcola i *loss* fisici per-step (tracking,
  reference, biologico, effort, smoothness, command/reference rate, stress SEA,
  penetrazione GRF e safety) e li espone in `info["reward_terms"]`.
- **`reward_function.py`** combina quei loss nello scalare reward (lo shaping che si
  tuna qui). `RewardShapingWrapper` ricalcola la reward dai loss e **sostituisce**
  quella interna dell'env.

Conseguenza: i campi `reward_*_weight` dentro l'env **non influenzano più** la reward
dell'agente (alimentano solo la reward interna ormai scartata). I pesi si toccano in
`RewardConfig`. La parte tracking/reference/bio/penalty/safety con i **default**
riproduce esattamente la reward originale dell'env.

**Penalty fuori-banda (out-of-band)**: in più, `compute_reward` penalizza il
**riferimento comandato** (l'output della policy, `info["policy_segment_values"]` —
la traiettoria che la SEA deve inseguire) quando esce dalla banda fisiologica di gait,
per giunto (ordine `pros_coords` = knee, ankle):

- `oob_q_min = (-1.35, -0.5)`, `oob_q_max = (0.0, 0.5)` rad (il `pros_knee_angle` è
  flessione-**negativa** nel modello, IK ~`[-1.06, -0.13]`; ankle ~`[-0.13, 0.40]`);
- escursione quadratica fuori banda × `oob_weight` (default **2.0**), sottratta
  **dopo il clip** (come la safety): resta attiva con gradiente anche quando la parte
  positiva è già 0. `oob_weight = 0` la disabilita.

Si penalizza il *riferimento* (controllato direttamente dalla rete), non la `q`
simulata: gradiente pulito sull'output della policy. Essendo attiva di default, la
reward NON è più bit-identica a quella dell'env (lo è solo con `oob_weight = 0`).
Questa banda (stretta) è distinta dai bound di **terminazione anti-divergenza**
dell'env (larghi: knee `[-2, 0]`, ankle `[-0.9, 0.9]` rad, su `q` simulata): la banda
*shapa* dolcemente, il guard *termina* solo una simulazione esplosa.

Semantica Gymnasium di fine episodio:

- `terminated=True`: stato unsafe appartenente al task (`fall`,
  `joint_divergence:<coord>`), con safety penalty;
- `truncated=True`: limite temporale/dataset (`episode_time_limit`, `dataset_end`)
  oppure errore numerico catturato (`numerical_failure`).

La causa precisa è disponibile in `info["end_reason"]`.

Override via `--reward-json` (file o JSON inline) su `train_ppo_mlp.py` e
`rollout_eval.py` (usare lo stesso valore per coerenza train/eval):

```powershell
# alleggerisce il dominio del reference (F2) e morde di più la saturazione
... train_ppo_mlp.py ... --reward-json '{\"blend_reference\": 0.35, \"saturation_weight\": 0.5}'
# tuning della penalty fuori-banda
... train_ppo_mlp.py ... --reward-json '{\"oob_weight\": 5.0, \"oob_q_min\": [-1.45, -0.6]}'
```

`summary.json` (training) e `rollout_summary.json` (rollout) registrano la
`reward_config` effettivamente usata.

I termini command/reference-rate restano disponibili sia come aggregato
`command_rate_loss` sia separatamente (`segment_delta_loss`, `qdot_ref_loss`,
`qddot_ref_loss`, `jerk_ref_loss`, `reference_governor_loss`, `u_rate_loss`).
I relativi pesi separati hanno default zero, quindi non alterano i checkpoint
legacy; permettono pero di colpire direttamente una singola causa, ad esempio
jerk o alternanza del comando. La penalita morbida `grf_penetration_loss` e
anch'essa pesata qui; le configurazioni che applicano il contatto online devono
abilitare esplicitamente `grf_penetration_weight`.

## TensorBoard

Il training scrive eventi in `<output_dir>/tensorboard` (attivo di default;
`--no-tensorboard` per disattivare). Vengono loggati per iterazione:

- `reward/*` — reward finale e le sue componenti (`reward`, `reward_base`,
  `tracking_score`, `reference_score`, `bio_score`, `penalty`, `safety_term`,
  `oob_loss`, `oob_term`);
- `reward_loss/*` — i loss grezzi dell'env (tracking/reference/bio/effort/
  smoothness/saturation/safety, `u_abs_max`, `u_saturation_fraction`);
- `gait/*` — GRF normale/BW, contatto, heel strike, toe-off, durata ciclo e
  fase online per lato, quando `online_sensor`/`online` sono attivi;
- `env_runners/*` — `episode_return_mean`, `episode_len_mean`, ...;
- `learners/default_policy/*` — `policy_loss`, `vf_loss`, `entropy`, KL, ...

```powershell
... python -m tensorboard.main --logdir runs\training --port 6006   # http://localhost:6006
```

Le componenti reward arrivano in TensorBoard tramite `RewardComponentsCallback`
(aggrega per-step → media per-iterazione lato env-runner); l'asse x è
`num_env_steps_sampled_lifetime`. I contatori cumulativi `episode_end/*`
distinguono le cause di terminazione e troncamento.

## Validazione training RLlib

La stabilizzazione è stata verificata con tiny training locale
(`num_env_runners=0`), tiny training parallelo (`num_env_runners=1`) e un
training controllato da `50` iterazioni sul simulatore OpenSim reale con
`online_sensor`, un EnvRunner, batch `4`, orizzonte `0.08 s` e timeout totale
`900 s`.

Il run `runs/training/_baseline_mlp_50iter_online_sensor` ha completato `50/50`
iterazioni in `716.42 s`, senza timeout o processi Ray residui. Tutte le
metriche core sono finite; il return medio passa da `2.4356` nelle prime 10
iterazioni a `2.6363` nelle ultime 10. Il value loss passa da `2.8313` a
`2.5147`, ma l'explained variance resta debole (`-0.1345` → `-0.0042`).
Sono stati osservati `48` `episode_time_limit` e `2`
`joint_divergence_pros_knee_angle`.

Questo valida pipeline, timeout, parallelismo, metriche e update PPO. Non
dimostra ancora apprendimento su gait cycle completi: restano da eseguire run
full-gait, migliorare il critic e analizzare i fallback bounded least-squares
della Static Optimization osservati durante il training.

## Note / TODO collegati (fuori scope di questa baseline)

- **F1**: le azioni sono clippate ai bound via `FlattenClipAction` (l'env valida
  ma non clippa). La media in inference è resa deterministica nel rollout.
- **F2 / reward**: il ribilanciamento della reward (saturazione SEA, peso
  `reference_score`) resta un TODO separato; qui la reward dell'env è invariata.
- **gait metrics**: il porting delle metriche di qualità del passo
  (RMSE/Symmetry/Trend) dal notebook della repo originale è un passo successivo.
