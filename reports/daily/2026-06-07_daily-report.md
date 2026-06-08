# Daily report - 2026-06-07

## Sintesi

Giornata dedicata a due risultati principali:

1. implementazione, calibrazione e validazione della modalità onlineGRF nel
   simulatore, mantenendo invariata la modalità CMC-like prescribed;
2. stabilizzazione della pipeline MLP/RLlib, inclusi timeout rigidi, cleanup
   dell'albero Ray, training parallelo e validazione controllata da `50`
   iterazioni.

Report utente prodotti oggi:

```text
reports/user/2026-06-07_modalita_online_grf_simulatore.md
reports/user/2026-06-07_stabilizzazione_training_rllib_mlp.md
```

## 1. Modalità onlineGRF

Il simulatore espone ora tre modalità:

- `prescribed`: comportamento storico basato su ExternalLoads;
- `online_sensor`: le prescribed guidano la dinamica, mentre le GRF online
  vengono calcolate come sensore;
- `online`: il contatto online guida direttamente la dinamica, anche senza
  ExternalLoads.

La modalità root del simulatore resta `prescribed` per retrocompatibilità. La
baseline MLP/RLlib usa invece `online_sensor` come default per training e
inference.

### Architettura

È stato aggiunto il componente OpenSim separato
`OnlineGRFSphereHalfSpaceForce`, senza modificare il plugin SEA.

Il componente:

- usa sfere collegate a qualsiasi `PhysicalFrame`;
- calcola forza normale, dissipazione e attrito;
- supporta superfici mobili per treadmill;
- può applicare il carico oppure operare in modalità sensor-only;
- espone forza, momento, COP, penetrazione, slip e normale del piano.

La geometria e i parametri dei contatti sono definiti in profili JSON
model-specific. Il runtime resta model-agnostic.

### Integrazione simulatore

Sono stati aggiornati loader, setup, runner e output per:

- selezionare e validare `grf_mode`;
- caricare il plugin onlineGRF solo quando necessario;
- mantenere o rimuovere le ExternalForce in base alla modalità;
- aggregare le forze dei contatti per lato a ogni substep;
- registrare GRF, eventi gait e configurazione attiva;
- esporre GRF ed eventi tramite `last_step_info`;
- interrompere la modalità `online` se le forze superano il limite plausibile.

Nuovi output principali:

```text
<prefix>_online_grf.sto
<prefix>_gait_events_online.csv
<prefix>_run_status.txt
```

## 2. Calibrazione e validazione onlineGRF

Sono state eseguite calibrazione globale, calibrazione intensiva e ricerca
sparsa di una patch di contatto.

Il profilo migliore è:

```text
online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_sensor_basis.json
```

Risultati holdout sul replay IK:

| Metrica | Sinistra | Destra |
|---|---:|---:|
| RMSE verticale | `60.70 N` | `31.81 N` |
| NRMSE sul picco | `7.72%` | `4.17%` |
| Correlazione verticale | `0.983` | `0.995` |
| Contact F1 | `0.921` | `0.958` |

L'audit fra formula Python e plugin C++ sullo stesso stato produce errore
massimo `0.0 N`: non è emerso un bug implementativo nel calcolatore.

La magnitudo completa non può essere resa identica alle prescribed sui rollout
forward divergenti, perché le prescribed restano time-driven mentre il sensore
online è state-based.

## 3. Heel strike e gait cycle

La priorità è stata spostata dalla riproduzione esatta della magnitudo alla
rilevazione concorde degli heel strike per scandire i gait cycle della rete.

Il detector streaming usa:

- crossing basso `20 N` per il timestamp;
- conferma alta model-specific, `190 N` per AB06;
- durata minima di contatto `0.05 s`.

Validazione:

| Dataset | Matched | Falsi positivi | Falsi negativi | Timing |
|---|---:|---:|---:|---:|
| Replay IK completo | `12/12` | `0` | `0` | MAE `10 ms` |
| Stati forward prescribed `2 s` | `3/3` | `0` | `0` | MAE `<1 ms` |

Per AB06, `online_sensor` è quindi validato per il rilevamento degli heel
strike nel perimetro testato. La validazione deve essere ripetuta per ogni
altro modello.

## 4. Integrazione onlineGRF nella rete

L'adapter condiviso `Trajectory Generator/osim_trj_cmc_like.py` espone ora:

- `info["online_grf"]`;
- `info["online_events"]`;
- `info["online_gait"]`.

Con `include_online_grf_observation=True`, l'osservazione della policy include
per lato:

- forza normale normalizzata per body weight;
- stato di contatto;
- impulsi heel-strike e toe-off;
- durata del gait cycle;
- fase stimata dall'ultimo heel strike.

Nella baseline MLP/RLlib, `online_sensor`, il profilo AB06 validato e
l'osservazione gait sono i default di training e inference. I checkpoint legacy
richiedono `--grf-mode prescribed --no-online-grf-observation`.

## 5. Stabilizzazione MLP/RLlib

Sono stati individuati due blocchi principali:

- Torch/Ray/OpenSim venivano importati prima che il watchdog potesse
  intervenire;
- `PPOConfig.build_algo()` tentava di creare log in `~/ray_results`; su un
  percorso non scrivibile, `tempfile.mkdtemp()` su Windows riprovava
  indefinitamente.

Correzioni implementate:

- launcher minimale e training in processo figlio supervisionato;
- import pesanti eseguiti solo nel worker;
- Windows Job Object per terminare l'intero albero Ray;
- timeout separati per startup, iterazione, checkpoint, cleanup, sampling e run
  totale;
- stack dump automatici in `faulthandler.log`;
- stato del watchdog in `watchdog_state.json`;
- log RLlib reindirizzati in `<output_dir>/rllib`;
- `PATH` Conda corretto automaticamente per il worker;
- CPU Ray limitate di default a `num_env_runners + 1`;
- metriche learner ed `episode_end/*` salvati in `summary.json`.

## 6. Training RLlib verificati

Sono stati completati:

- tiny training locale con `num_env_runners=0`;
- tiny training parallelo con `num_env_runners=1`;
- probe metriche da `3` iterazioni;
- training controllato da `50` iterazioni.

Run principale:

```text
runs/_baseline_mlp_50iter_online_sensor
```

Risultati:

| Metrica | Valore |
|---|---:|
| Iterazioni completate | `50/50` |
| Tempo wall-clock | `716.42 s` |
| Timeout | `0` |
| Return medio prime / ultime 10 | `2.4356 / 2.6363` |
| Miglior return | `3.3045` |
| Value loss prime / ultime 10 | `2.8313 / 2.5147` |
| Explained variance prime / ultime 10 | `-0.1345 / -0.0042` |
| Entropy prime / ultime 10 | `8.3611 / 8.2745` |
| `episode_time_limit` | `48` |
| `joint_divergence_pros_knee_angle` | `2` |
| Processi Ray residui | `0` |

Le metriche core sono rimaste finite, il return è migliorato modestamente e
l'entropia non è collassata. Il critic resta debole. Il run usa un orizzonte
breve e valida la pipeline PPO, non ancora l'apprendimento full-gait.

Durante il training sono comparsi ripetuti fallback bounded least-squares della
Static Optimization.

## File principali modificati oggi

```text
config.py
main.py
model_loader.py
output.py
path_resolver.py
setup_io.py
simulation_runner.py
onlineGRF.md
Trajectory Generator/osim_trj_cmc_like.py
Trajectory Generator/baseline_MLP/env_factory.py
Trajectory Generator/baseline_MLP/train_ppo_mlp.py
Trajectory Generator/baseline_MLP/rollout_eval.py
Trajectory Generator/baseline_MLP/tb_logging.py
Trajectory Generator/baseline_MLP/README.md
Trajectory Generator/baseline_MLP/commands.txt
validation/rl_env_smoke_ab06_pi.py
reports/user/2026-06-05_baseline_mlp_rllib_ppo_single_agent.md
reports/user/2026-06-05_reward_centralizzata_e_tensorboard.md
reports/user/2026-06-06_correzione_semantica_terminated_truncated.md
reports/user/2026-06-06_dalla_reward_imitativa_alle_grf_online_glide.md
```

## File principali aggiunti oggi

```text
online_grf.py
online_grf_profiles/README.md
online_grf_profiles/AB06_SEASEA_stiff321_500_pi_preliminary_calibrated.json
online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_sensor_basis.json
online_grf_profiles/Adjusted_newmarkers_marker_inferred.json
tools/online_grf_contact/CMakeLists.txt
tools/online_grf_contact/OnlineGRFSphereHalfSpaceForce.h
tools/online_grf_contact/OnlineGRFSphereHalfSpaceForce.cpp
tools/online_grf_contact/Plugin_interface.cpp
tools/online_grf_contact/README.md
validation/analyze_online_grf_heel_strikes.py
validation/calibrate_online_grf_basis.py
validation/calibrate_online_grf_intensive.py
validation/compare_online_grf_output.py
validation/generate_online_grf_profile.py
validation/test_online_grf_core.py
validation/validate_online_grf.py
validation/verify_online_grf_plugin.py
reports/user/2026-06-07_modalita_online_grf_simulatore.md
reports/user/2026-06-07_stabilizzazione_training_rllib_mlp.md
```

## Verifiche eseguite

- compilazione Windows Release del plugin `OnlineGRFContact`: completata;
- caricamento plugin e tipo custom tramite OpenSim Python: completato;
- unit test core onlineGRF: `5/5` passati;
- smoke `prescribed`, `online_sensor` e `online`: completati;
- modalità `online` senza ExternalLoads: completata;
- generazione output onlineGRF ed eventi: verificata;
- calibrazione globale, intensiva e sparsa con holdout: completata;
- audit formula Python/plugin C++: errore massimo `0.0 N`;
- validazione heel strike: `12/12` IK e `3/3` forward;
- smoke adapter RL con osservazione gait: completato;
- timeout rigido e cleanup dell'albero Ray: verificati;
- tiny training locale e parallelo: completati;
- training RLlib da `50` iterazioni: completato;
- checkpoint best/last, RLModule e TensorBoard: presenti;
- metriche core finite su tutte le iterazioni: verificato;
- processi Ray residui a fine run: `0`;
- `py_compile`: completato;
- `git diff --check`: completato senza errori.

## TODO chiusi oggi

- progettare e implementare una modalità onlineGRF alternativa alle
  ExternalLoads;
- mantenere la modalità prescribed come baseline/regressione;
- calcolare e registrare online GRF, contatto, heel strike e toe-off;
- integrare GRF ed eventi nell'adapter RL;
- rendere `online_sensor` il default della baseline MLP/RLlib;
- validare il rilevamento heel strike AB06 contro le prescribed;
- chiarire il limite della calibrazione della magnitudo completa;
- stabilizzare il tiny training RLlib;
- impedire la proliferazione incontrollata dei processi Ray;
- aggiungere timeout rigidi e cleanup dell'albero processi;
- rivalidare `num_env_runners > 0`;
- completare un training reale da almeno `50` iterazioni;
- verificare critic, return, stabilità ed `end_reason`;
- confermare `episode_time_limit` come troncamento e
  `joint_divergence` come terminazione nel training reale.

## TODO aperti e propagati

### Priorità immediate: training e simulatore

- eseguire training full-gait con episodi sufficientemente lunghi da osservare
  heel strike e gait cycle completi, mantenendo timeout e limiti CPU;
- migliorare il critic, la cui explained variance resta negativa;
- ridurre le terminazioni unsafe `joint_divergence_pros_knee_angle`;
- analizzare i fallback bounded least-squares ripetuti della Static
  Optimization e risolvere target/bounds biologici AB06;
- rivalidare training paralleli con più di un EnvRunner dopo aver misurato
  consumo CPU/RAM e scalabilità;
- tarare `oob_weight` usando `reward/oob_term` in TensorBoard;
- rivalidare il filtro 6 Hz con una policy allenata e su rollout lungo;
- verificare il lag causale del filtro rispetto alla IK a fase-zero.

### GRF online

- migliorare la magnitudo delle GRF online con patch plantare 3D e/o correttore
  residuale state-based;
- calibrare e validare separatamente ogni modello destinato al training;
- testare `grf_mode="online_sensor"` per il rilevamento dei gait cycle su ogni
  altro modello, verificando heel-strike e toe-off contro le prescribed;
- definire criteri quantitativi di accettazione holdout per magnitudo, timing e
  componenti tangenziali;
- migliorare il tracking forward/prescribed prima di validare sensori
  state-based su rollout lunghi;
- validare rollout `online` più lunghi e analizzare le reserve crescenti;
- misurare e configurare la velocità reale del treadmill;
- compilare e verificare il plugin onlineGRF su macOS arm64;
- definire se e come usare GRF ed eventi online nella reward o nella
  terminazione.

### Traiettorie ex novo / GLiDE-like

- definire formalmente metriche di suitability: velocità, stabilità, simmetria,
  effort biologico, energia SEA e qualità del passo;
- progettare un action space assoluto o parametrico indipendente dalla IK;
- progettare un livello vincolato/QP tra obiettivi funzionali e traiettorie
  protesiche;
- definire il curriculum da reward imitativa a reward task-based;
- separare definitivamente tracking di fattibilità da imitazione della IK;
- valutare un modello ridotto per pretraining GLiDE-like e fine-tuning OpenSim;
- completare il reward rebalancing F2;
- portare RMSE/NRMSE, Symmetry Angle e Trend Symmetry in un modulo riusabile;
- integrare metriche funzionali basate su GRF e impulsi.

### Validazione generatore / SNN / PPO

- **F1 percorso SNN/skrl**: clippare la media in inference ai bound dell'azione;
- **F2 percorso SNN/skrl**: limitare la magnitudo azione e rivalidare la
  penalty di saturazione;
- **F3 percorso SNN/skrl**: rendere robusto l'import torch/OpenSim/numpy su
  Windows;
- **F4**: chiamare `generator.reset()` al reset env nel rollout SNN;
- **F7**: validare `input_size == len(feature_names)` e architettura su
  resume/load;
- validare il generatore su rollout lunghi;
- definire e validare normalizzazione I/O, feature names, unità, scaling,
  output transform, action limits e metadata checkpoint;
- aggiungere checkpoint periodici e gestione robusta di interruzioni/crash nel
  percorso SNN;
- **F6/T2**: irrobustire o rimuovere la cache su `data_ptr` in `actor_critic`;
- **T3**: validare reshape/BPTT prima di usare `num_envs > 1`;
- **T4**: passare lo spike gradient custom agli encoder rate/latency;
- valutare l'incapsulamento della SNN come custom RLModule RLlib;
- **S1**: uniformare i reader `.sto` sul flag `inDegrees`;
- **S2/J2**: allineare il default `sea_stiffness` e valutare gain per-modello;
- **S3**: riusare `InverseDynamicsSolver` invece di ricrearlo ogni frame.

### Repository e knowledge base

- confermare il layout Git di `osim_trj_cmc_like.py` in
  `Trajectory Generator/` e aggiornare riferimenti/packaging;
- decidere se includere `LLM_SIMULATOR_OVERVIEW.md` in `CONTEXT.md` o nel
  flusso `start_day`;
- recuperare il vero paper Wrapyfi: il file corrente in `paper/7` è il paper
  iCub;
- approfondire, se utile, P17, P24, P14, P19 e P23;
- collegare i finding della letteratura alla roadmap ex novo/GRF online.

### Propagati storici: controllo SEA

- sweep `Kp_knee_motor` tra 3.9 e 18 mantenendo ankle best;
- validare il coupling knee-ankle isolando la dinamica knee dal feedback ankle;
- valutare un notch a 28 Hz sul feedback knee;
- cleanup modelli sperimentali `slow_inner_pd_1405` e `pi_asym_knee1405`;
- completare e documentare build/copia DLL plugin PI su Windows;
- eseguire il secondo pass knee dello sweep locale;
- consolidare il confronto finale tra configurazioni storiche;
- pulire gli artefatti sweep `_cascade_local_gain_sweep_20260517_233607` e
  `_cascade_local_gain_sweep_20260517_234151`;
- LPF qdot: test asimmetrico ankle, cutoff 30/35 Hz e run lunga 30+ s.
