# Modalità onlineGRF nel simulatore CMC-like

## Obiettivo

Implementare nel simulatore una modalità `onlineGRF` capace di calcolare le
Ground Reaction Forces in funzione dello stato corrente del modello durante il
gait.

I requisiti principali erano:

- mantenere disponibile e invariata la modalità CMC-like basata su GRF
  prescribed;
- evitare un'implementazione specifica per il modello AB06;
- supportare il futuro utilizzo delle GRF calcolate durante il training;
- validare il calcolatore confrontando le GRF online con quelle prescribed;
- mantenere separato il nuovo componente di contatto dal plugin SEA.

## Soluzione implementata

Il simulatore espone ora tre modalità GRF:

- `prescribed`: comportamento storico; le `ExternalForce` prescribed guidano
  la dinamica;
- `online_sensor`: le GRF prescribed continuano a guidare la dinamica, mentre
  le GRF online vengono calcolate e registrate senza applicare il carico al
  modello;
- `online`: le forze di contatto online guidano la dinamica. Le GRF prescribed
  sono un oracle opzionale e possono essere completamente omesse.

La modalità `prescribed` rimane il default, quindi setup e comandi esistenti
continuano a mantenere il comportamento precedente.

## Architettura

### Componente di contatto C++

È stato creato il componente OpenSim separato
`OnlineGRFSphereHalfSpaceForce`, senza modificare il plugin
`SeriesElasticActuator`.

Il componente:

- collega una sfera di contatto a qualsiasi `PhysicalFrame`;
- calcola forza normale smooth, dissipazione e attrito tangenziale;
- supporta una velocità della superficie per simulazioni treadmill;
- applica la forza al modello nella modalità `online`;
- restituisce comunque i record di contatto con `appliesForce=false`, necessari
  per la modalità `online_sensor`;
- espone forza, momento rispetto all'origine ground, punto di contatto,
  penetrazione, slip speed e normale del piano.

Il componente è costruibile su Windows x86-64 e macOS arm64 tramite CMake.

### Profili JSON model-specific

La geometria dei contatti non è hardcoded nel simulatore. È descritta da
profili JSON che definiscono:

- piano del terreno e velocità della superficie;
- materiale di contatto;
- sfere, lato, frame, posizione locale e raggio.

Questo mantiene model-agnostic il codice runtime. Per aggiungere un modello è
sufficiente generare o definire un nuovo profilo.

È stato aggiunto uno script che inferisce una configurazione iniziale
heel/toe dai marker del modello. L'inferenza è stata verificata su:

- `AB06_SEASEA_Threadmill`;
- `Adjusted_newmarkers_fp12_15889_19839_ready`.

### Loader e setup

`model_loader.py` ora:

- valida `grf_mode`;
- carica il plugin onlineGRF solo quando necessario;
- mantiene le ExternalLoads nella modalità `prescribed`;
- applica contemporaneamente prescribed e sensori non attivi in
  `online_sensor`;
- non aggiunge ExternalForce al modello in modalità `online`;
- consente `online` senza alcun file ExternalLoads;
- conserva eventuali GRF prescribed come oracle di validazione;
- aggiunge programmaticamente i contatti definiti dal profilo.

I setup XML supportano ora:

- `grf_mode`;
- `online_grf_profile_file`;
- ExternalLoads opzionali solo nella modalità `online`.

I vecchi setup XML restano retrocompatibili e vengono interpretati come
`prescribed`.

### Runner, output ed eventi

Il runner legge e aggrega le forze di tutte le sfere per lato a ogni substep.

Sono disponibili:

- forza GRF 3D sinistra e destra;
- momento rispetto al ground;
- COP proiettato sul piano;
- forza normale;
- massima penetrazione e slip speed;
- stato di contatto;
- eventi streaming heel-strike e toe-off;
- GRF ed eventi in `last_step_info` per gli utilizzatori step-wise.

In modalità `online` è presente un controllo di plausibilità che interrompe la
simulazione se la forza online supera il limite configurato in body weight.

Nuovi output:

- `<prefix>_online_grf.sto`;
- `<prefix>_gait_events_online.csv`;
- `grf_mode` e profilo attivo nel file `<prefix>_run_status.txt`.

## Calibrazione e validazione

La calibrazione è stata estesa e reiterata con tre strategie:

- fit globale iniziale;
- fit intensivo con materiale ed esponente forza-penetrazione per contatto;
- ricerca sparsa di una patch di contatto tramite regressione non-negativa.

Il profilo migliore è
`online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_sensor_basis.json`.

Su replay IK, con split calibration/holdout, il profilo migliore ottiene:

| Metrica holdout | Sinistra | Destra |
|---|---:|---:|
| RMSE verticale | `60.70 N` | `31.81 N` |
| NRMSE verticale sul picco | `7.72%` | `4.17%` |
| Correlazione verticale | `0.983` | `0.995` |
| Contact F1 | `0.921` | `0.958` |

Una griglia più densa e più contatti non migliorano l'holdout. Il risultato è
quindi il limite stabile osservato della legge di contatto state-based.

L'audit `validation/verify_online_grf_plugin.py` confronta formula Python e
plugin C++ sullo stesso stato. L'errore massimo osservato è `0.0 N`.

È stato inoltre provato il fit sugli stati salvati dal simulatore prescribed.
Su un frammento breve il fit arriva a circa `3.7/3.6 N` verticali nel tratto di
calibrazione, ma non generalizza al tratto successivo. Su un rollout di `2 s`,
quando lo stato forward diverge dall'IK, le ExternalLoads continuano a essere
applicate in funzione del tempo e non descrivono più il contatto dello stato
corrente. In questa condizione nessun sensore puramente state-based può
annullare l'errore rispetto alle prescribed senza introdurre dipendenza
esplicita dal tempo/oracolo.

Non è emerso un bug implementativo nel calcolatore onlineGRF. Il residuo è
causato dalla capacità della legge di contatto su IK e, nei rollout forward
lunghi, dall'incompatibilità tra stato simulato divergente e GRF prescribed
time-driven.

### Validazione heel strike per gait cycle

Poiché la rete deve usare le GRF online soprattutto per scandire i gait cycle,
è stato aggiunto `validation/analyze_online_grf_heel_strikes.py`.

Con la soglia singola storica di `20 N` tutti gli heel strike reali sono
rilevati con timing accurato, ma piccoli picchi durante lo swing causano falsi
positivi. Il detector streaming è quindi stato esteso con doppia soglia:

- crossing basso: `20 N`, usato come timestamp dell'heel strike;
- conferma alta AB06: `190 N`, salvata nel profilo model-specific;
- durata minima di contatto: `0.05 s`.

Risultati sul replay IK completo:

- heel strike prescribed osservati: `12`;
- heel strike online matched: `12/12`;
- falsi positivi: `0`;
- falsi negativi: `0`;
- F1: `1.0`;
- timing MAE rispetto alle prescribed: `10 ms`;
- timing massimo assoluto: `30 ms`;
- latenza media di conferma streaming: circa `67 ms`;
- intervallo di conferma perfetto osservato: `184-200 N`.

Risultati sugli stati forward prescribed di `2 s`:

- eventi osservabili matched: `3/3`;
- falsi positivi e falsi negativi: `0`;
- F1: `1.0`;
- timing MAE: inferiore a `1 ms`;
- la soglia comune `190 N` appartiene anche all'intervallo valido forward.

La forma e la magnitudo online non coincidono esattamente con le prescribed, ma
il segnale è adatto a identificare gli heel strike nel perimetro validato. La
validazione deve essere ripetuta su altri modelli e su rollout/policy più vari
prima dell'uso generalizzato nel training.

### Integrazione nell'ambiente RL

Gli output onlineGRF e gli eventi gait sono stati collegati all'adapter condiviso
`Trajectory Generator/osim_trj_cmc_like.py`.

Quando `online_sensor` o `online` sono attivi, ogni step espone:

- `info["online_grf"]`: ultimo campione GRF online per lato;
- `info["online_events"]`: eventi confermati durante lo step;
- `info["online_gait"]`: GRF normale normalizzata per body weight, contatto,
  impulsi heel-strike/toe-off, durata ciclo e fase stimata dall'ultimo
  heel-strike.

Con `include_online_grf_observation=True`, gli stessi segnali gait vengono
aggiunti all'osservazione della policy. Nella baseline MLP/RLlib,
`online_sensor`, il profilo AB06 validato e l'osservazione onlineGRF sono ora i
default sia per training sia per inference. Il simulatore root e l'adapter
condiviso mantengono invece i default conservativi prescribed/opt-in. I
checkpoint MLP legacy richiedono esplicitamente
`--grf-mode prescribed --no-online-grf-observation`. La reward non è stata
modificata; l'uso degli eventi nella reward o nella terminazione resta una
decisione successiva.

## Stabilizzazione training RLlib

La pipeline MLP/RLlib e stata stabilizzata mantenendo `online_sensor` come
default della rete.

I due blocchi individuati erano:

- gli import pesanti Torch/Ray/OpenSim avvenivano prima dell'avvio del
  supervisore, quindi un blocco durante l'import non poteva essere interrotto;
- `PPOConfig.build_algo()` tentava di creare i log standalone in
  `~/ray_results`. Quando la directory non era scrivibile, `tempfile.mkdtemp()`
  su Windows riprovava indefinitamente.

Le correzioni implementate sono:

- processo supervisore minimale, con import Torch/Ray/OpenSim eseguiti solo nel
  worker;
- timeout rigidi per import/startup, ogni iterazione, checkpoint, cleanup e run
  totale;
- Windows Job Object per terminare l'intero albero Ray allo scadere del timeout;
- limite esplicito delle CPU esposte a Ray;
- DLL dell'ambiente Conda aggiunte automaticamente al `PATH` del worker;
- log interni RLlib reindirizzati in `<output_dir>/rllib`;
- stack dump periodico in `faulthandler.log` durante le fasi bloccanti;
- metriche learner e contatori cumulativi `episode_end/*` salvati in
  `summary.json`.

Sono stati completati il tiny training locale (`num_env_runners=0`), il tiny
training parallelo (`num_env_runners=1`) e un training controllato di `50`
iterazioni sul simulatore OpenSim reale, con `grf_mode="online_sensor"` e
timeout totale di `900 s`.

Risultati del run `runs/_baseline_mlp_50iter_online_sensor`:

| Metrica | Risultato |
|---|---:|
| Iterazioni completate | `50/50` |
| Tempo wall-clock | `716.42 s` |
| Return medio prime / ultime 10 | `2.4356 / 2.6363` |
| Miglior return | `3.3045` |
| Value loss prime / ultime 10 | `2.8313 / 2.5147` |
| Explained variance critic prime / ultime 10 | `-0.1345 / -0.0042` |
| Entropy prime / ultime 10 | `8.3611 / 8.2745` |
| Fine per `episode_time_limit` | `48` |
| Terminazioni `joint_divergence_pros_knee_angle` | `2` |
| Errori numerici / timeout del training | `0 / 0` |

Tutte le metriche core sono rimaste finite, checkpoint best/last, RLModule e
file TensorBoard sono stati prodotti, e non sono rimasti processi Ray dopo il
cleanup. Il return mostra un miglioramento modesto e l'entropia non collassa.
Il critic resta pero debole: l'explained variance si avvicina a zero ma rimane
negativa. Il run usa un orizzonte breve (`0.08 s`) ed e quindi una validazione
reale della pipeline e degli update PPO, non ancora una prova di apprendimento
su gait cycle completi.

Durante il run sono comparsi ripetuti fallback bounded least-squares della
Static Optimization. Non hanno bloccato il training, ma devono essere
analizzati prima di training full-gait costosi.

## File modificati

- `config.py`
- `main.py`
- `model_loader.py`
- `output.py`
- `path_resolver.py`
- `setup_io.py`
- `simulation_runner.py`
- `onlineGRF.md`
- `Trajectory Generator/osim_trj_cmc_like.py`
- `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`
- `Trajectory Generator/baseline_MLP/rollout_eval.py`
- `Trajectory Generator/baseline_MLP/tb_logging.py`
- `Trajectory Generator/baseline_MLP/README.md`
- `Trajectory Generator/baseline_MLP/commands.txt`
- `validation/rl_env_smoke_ab06_pi.py`

## File aggiunti

- `online_grf.py`
- `online_grf_profiles/README.md`
- `online_grf_profiles/AB06_SEASEA_stiff321_500_pi_preliminary_calibrated.json`
- `online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_sensor_basis.json`
- `online_grf_profiles/Adjusted_newmarkers_marker_inferred.json`
- `tools/online_grf_contact/CMakeLists.txt`
- `tools/online_grf_contact/OnlineGRFSphereHalfSpaceForce.h`
- `tools/online_grf_contact/OnlineGRFSphereHalfSpaceForce.cpp`
- `tools/online_grf_contact/Plugin_interface.cpp`
- `tools/online_grf_contact/README.md`
- `validation/calibrate_online_grf_basis.py`
- `validation/calibrate_online_grf_intensive.py`
- `validation/analyze_online_grf_heel_strikes.py`
- `validation/compare_online_grf_output.py`
- `validation/generate_online_grf_profile.py`
- `validation/validate_online_grf.py`
- `validation/verify_online_grf_plugin.py`
- `validation/test_online_grf_core.py`

Il DLL Windows compilato è disponibile localmente in
`plugins/OnlineGRFContact.dll`; la cartella plugin ignora i binari nel
repository.

## Comandi principali

Modalità sensore, con GRF prescribed ancora attive:

```powershell
conda run -n envCMC-like python main.py `
  --setup path/to/setup.xml `
  --grf-mode online_sensor `
  --online-grf-profile online_grf_profiles/profile.json
```

Modalità online attiva senza GRF prescribed:

```powershell
conda run -n envCMC-like python main.py `
  --setup path/to/setup.xml `
  --grf-mode online `
  --no-external-loads `
  --online-grf-profile online_grf_profiles/profile_calibrated.json
```

Calibrazione e validazione:

```powershell
conda run -n envCMC-like python validation/validate_online_grf.py `
  --setup path/to/setup.xml `
  --profile online_grf_profiles/profile.json `
  --out-profile online_grf_profiles/profile_calibrated.json `
  --report results/online_grf_validation.json
```

Calibrazione sparsa raccomandata:

```powershell
conda run -n envCMC-like python validation/calibrate_online_grf_basis.py `
  --setup path/to/setup.xml `
  --profile online_grf_profiles/profile.json `
  --out-profile online_grf_profiles/profile_sensor.json `
  --report results/online_grf_basis.json
```

## Test e verifiche eseguite

- compilazione C++ Windows Release del plugin `OnlineGRFContact`: completata;
- caricamento del tipo custom tramite binding Python OpenSim: completato;
- `python -m py_compile` sui file Python modificati e aggiunti: completato;
- unit test core onlineGRF: `5/5` passati;
- round-trip setup XML `online` senza ExternalLoads: completato;
- smoke `prescribed` AB06: completato;
- smoke `online_sensor` AB06: completato;
- caricamento `online_sensor` su una seconda famiglia di modello: completato;
- caricamento `online` con `0` ExternalForce e `4` contatti attivi: completato;
- smoke `online` senza oracle prescribed: completato;
- generazione di `online_grf.sto`, eventi online e run status: verificata;
- validazione e calibrazione breve contro GRF prescribed: completata;
- calibrazione intensiva e sparsa con holdout: completata;
- audit formula Python/plugin C++: errore massimo `0.0 N`;
- rollout `online_sensor` di `0.5 s`: completato;
- rollout `online` puro senza ExternalLoads di `0.5 s`: completato;
- rollout prescribed di `2 s` e calibrazione sugli stati forward: completati;
- validazione heel strike a doppia soglia: `12/12` su IK e `3/3` sugli stati
  forward, senza falsi positivi o falsi negativi;
- smoke adapter RL `online_sensor`: onlineGRF/eventi presenti in `info`, heel
  strike confermati e osservazione gait opzionale verificata;
- timeout rigido verificato con terminazione dell'intero albero Ray;
- tiny training locale e parallelo RLlib: completati;
- training RLlib controllato da `50` iterazioni: completato, metriche learner e
  `end_reason` analizzati;
- `git diff --check`: nessun errore.

## TODO

- eseguire training full-gait con orizzonti sufficienti a osservare heel strike
  e gait cycle completi, mantenendo timeout e limiti CPU;
- migliorare il critic, che nel run controllato da 50 iterazioni mantiene
  explained variance negativa, e ridurre le terminazioni unsafe del ginocchio;
- analizzare i fallback bounded least-squares ripetuti della Static
  Optimization osservati durante il training;
- riprodurre con maggiore precisione la magnitudo completa delle GRF online
  tramite una patch plantare 3D e/o un correttore residuale state-based; per ora
  dare priorità alla rilevazione concorde degli heel strike per scandire i gait
  cycle della rete;
- calibrare e validare separatamente ogni modello destinato al training;
- testare e validare `grf_mode="online_sensor"` per il rilevamento dei gait
  cycle su ogni altro modello destinato a training/inference, verificando
  heel-strike e toe-off contro le rispettive GRF prescribed e ricalibrando le
  soglie model-specific quando necessario;
- definire criteri di accettazione quantitativi su holdout per magnitudo, timing
  degli eventi e componenti tangenziali;
- migliorare il tracking forward/prescribed prima di usare l'oracolo time-driven
  per validare sensori state-based su rollout lunghi;
- validare rollout `online` più lunghi: il test di `0.5 s` completa, ma mostra
  reserve più elevate e crescenti rispetto a `online_sensor`;
- misurare e configurare la velocità reale del treadmill nei profili che la
  richiedono;
- compilare e verificare il plugin su macOS arm64;
- definire se e come usare GRF ed eventi online nella reward o nella
  terminazione degli episodi; l'integrazione non invasiva in `info` e
  nell'osservazione opzionale è completata.
