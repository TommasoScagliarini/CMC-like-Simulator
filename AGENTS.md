# Guida per agenti AI
Instruction check token: CMC_AGENT_OK_2026

Questo repository contiene un simulatore OpenSim/Python in stile CMC-like per
testare controllori high-level su due attuatori SEA protesici, mantenendo il
lato biologico il piu possibile muscle-driven e usando le reserve come supporto
residuale.

## Plugin C++ — `SeriesElasticActuator`

I sorgenti attivi vivono nella repository separata `SEA_plugin_core - agent/`
(su questo Mac: `/Users/tommy/Documents/SEA_plugin_core - agent`). Dal
2026-10-15 non sono piu duplicati in `tools/sea_plugin_relative_d/`.
Il simulatore carica la libreria SEA compilata in `plugins/SEA/`.
Il plugin GRF separato vive in `plugins/online_grf_contact/`. I default sono
basename senza estensione, risolti da `path_resolver.py` per Windows/macOS;
i vecchi percorsi piatti dei due plugin correnti sono ancora riconosciuti.
Per compilazione macOS/Windows e replay delle prove storiche, leggere
`tools/README.md`; il percorso del checkout esterno va scelto localmente.

Eredita da `OpenSim::CoordinateActuator`. La versione corrente PI modella un
attuatore con molla in serie (SEA) con tre state variable interne:
`motor_angle`, `motor_speed` e `torque_error_integral`.

- **Proprieta**: `motor_inertia`, `motor_damping`, `stiffness` (K), `Kp`, `Kd`,
  `Ki`, `integral_torque_limit`, `Kd_Imp`, `Impedence` (bool).
  `optimal_force` (F_opt) e ereditata dal parent.
- **Controllo**: segnale normalizzato `u ∈ [-1, +1]` letto da `getControl(s)`.
- **Dinamica motore** (`computeStateVariableDerivatives`):
  - `tau_spring = K * (theta_m - theta_joint)`
  - Modo non-impedance: `tau_ref = u*F_opt`, `tau_i = clamp(Ki*xi, -integral_torque_limit, +integral_torque_limit)`,
    `tau_input = tau_ref + Kp * (tau_ref - tau_spring) + tau_i - Kd * omega_m`.
    `xi = torque_error_integral`; la sua derivata usa anti-windup condizionale.
  - Modo impedance: `tau_input = tau_ff + Kp*(theta_m_ref - theta_m) + Kd_Imp*(omega_m_ref - omega_m)`
    dove `theta_m_ref = theta_j + u*F_opt/K`, `tau_ff = tau_spring + Bm*omega_m`
  - Clamp a +/-500 N-m, poi `omega_m_dot = (tau_input - tau_spring - Bm*omega_m) / Jm`
- **Attuazione** (`computeActuation`):
  - Non-impedance: `K * (theta_m - theta_joint)` (coppia della molla)
  - Impedance: `F_opt * u` (coppia ideale)
- `computeForce` non e sovrascritto: il parent `CoordinateActuator` applica la
  forza generalizzata e fornisce il gradiente analitico.

### Equazioni della dinamica SEA

Il SEA e un sistema a due corpi (motore + molla) accoppiato al giunto.

**Variabili di stato** (integrate dal plugin):

    theta_m   : angolo motore [rad]
    omega_m   : velocita motore [rad/s]
    xi        : integrale dell'errore di coppia [N-m*s]

**Coppia elastica della molla** (trasmessa al giunto):

    tau_spring = K * (theta_m - theta_j)

dove `theta_j` e l'angolo del giunto letto dalla coordinata OpenSim.

**Coppia motore** (calcolata dal controllore interno del plugin):

- Modo **non-impedance** (`Impedence = false`):

      tau_ref   = u * F_opt
      error     = tau_ref - tau_spring
      tau_i     = clamp(Ki * xi, -integral_torque_limit, +integral_torque_limit)
      tau_input = tau_ref + Kp * error + tau_i - Kd * omega_m

  Il PI interno con damping motore insegue `tau_ref` attraverso la deflessione
  della molla. `xi_dot = error`, salvo anti-windup: viene congelato se
  l'integrale o la coppia motore raw sono saturi e l'errore spinge ulteriormente
  nella saturazione. Con `Ki = 0`, `xi_dot = 0`.

- Modo **impedance** (`Impedence = true`):

      tau_ref     = u * F_opt
      theta_m_ref = theta_j + tau_ref / K
      omega_m_ref = omega_j
      tau_ff      = tau_spring + Bm * omega_m
      tau_input   = tau_ff + Kp * (theta_m_ref - theta_m) + Kd_Imp * (omega_m_ref - omega_m)
      xi_dot      = 0

  Il PD interno insegue una posizione motore di riferimento che produrrebbe
  la coppia desiderata tramite la molla. Il termine feedforward compensa
  molla e smorzamento. E un loop in posizione.

**Saturazione**: `tau_input = clamp(tau_input, -500, +500)` N-m

**Dinamica del motore** (equazione del moto del rotore):

    d(theta_m)/dt = omega_m
    d(omega_m)/dt = (tau_input - tau_spring - Bm * omega_m) / Jm

dove `Jm` e l'inerzia del motore e `Bm` lo smorzamento viscoso.

**Forza applicata al giunto** (`computeActuation`):

- Non-impedance: `tau_out = tau_spring = K * (theta_m - theta_j)`
- Impedance: `tau_out = F_opt * u` (attuazione ideale, bypassa la molla)

Bug storici su scaling PD e baseline molla documentati in [[DIVERGENZA_BUGFIXES]].
La versione PI corrente e documentata in
[[2026-05-17_pi_inner_plugin_rk4_state_variable]] e
[[2026-06-14_allineamento_windows_plugin_pi]].

## Codice Python e cartelle condivise

Dal 2026-10-20 i due core vivono in `CMC-like Simulator/` e
`Trajectory Generator/`, nella stessa repository. I moduli elencati sotto
sono relativi a `CMC-like Simulator/`; i loro nomi di import canonici restano
`config`, `model_loader`, `path_resolver` e gli altri nomi flat originali.
I comandi del simulatore dalla radice usano il percorso quotato, per esempio
`python "CMC-like Simulator/main.py"` e
`python "CMC-like Simulator/visualize.py" --mlp`.

`workspace_bootstrap.py` resta nella radice e usa solo la libreria standard:
espone `REPO_ROOT`, `SIMULATOR_DIR`, `TRAJECTORY_GENERATOR_DIR` e
`ensure_simulator_paths()`. Gli script esterni al core lo chiamano prima di
importare i moduli del simulatore. Il bootstrap della baseline continua a
preparare anche i moduli del generatore e i nomi necessari ai pickle/worker.
Non duplicare i sorgenti e non ripristinare forwarding file Python nella root.

Le risorse condivise `models/`, `plugins/`, `online_GRF/`, `tools/`,
`validation/`, `results/`, `plot/` e `reports/` restano nella radice del
workspace. I percorsi delle risorse sono relativi a questa radice, non alla
cartella del core o alla working directory del processo. Il layout di `plot/`
resta invariato. Leggere `README.md` e `CMC-like Simulator/README.md`.

- `main.py`: entry point del simulatore. Carica configurazione, modello,
  cinematica, runner e avvia la simulazione.
- `config.py`: configurazione centrale del progetto. Contiene path, tempi di
  simulazione, parametri SEA, tracking biologico, static optimization e output.
  (modifiche rilevanti: [[DIVERGENZA_BUGFIXES]], [[2026-04-13_report_muscle_driven_reserve_residuali]])
- `model_loader.py`: carica modello OpenSim, plugin SEA, GRF e reserve
  actuators. Costruisce `SimulationContext` con nomi, indici e metadati usati
  dal resto della pipeline.
- `kinematics_interpolator.py`: legge il file IK `.sto` e costruisce spline per
  ottenere `q`, `qdot` e `qddot` a ogni istante.
- `outer_loop.py`: tracking cinematico biologico. Calcola le accelerazioni
  desiderate per i gradi di liberta biologici.
- `inverse_dynamics.py`: calcolo di inverse dynamics tramite baseline
  zero-actuator e proiezione con matrice di massa. Evita `realizeAcceleration()`
  e restituisce coppie biologiche e protesiche.
  (bug storici: [[DIVERGENZA_BUGFIXES]]; fix state cache: [[2026-04-14_output_espansi_e_visualizzatore]])
- `static_optimization.py`: recruitment biologico. Distribuisce `tau_bio` tra
  muscoli e reserve actuators, con diagnostica sul contributo muscolare e
  residuale. (riscrittura muscle-driven: [[2026-04-13_report_muscle_driven_reserve_residuali]])
- `prosthesis_controller.py`: controllore high-level Python dei due SEA. Calcola
  il comando normalizzato `u` da inviare al plugin low-level.
  (fix PD in coppia fisica: [[DIVERGENZA_BUGFIXES]]; fix tau_ff: [[2026-04-13_report_muscle_driven_reserve_residuali]])
- `output.py`: I/O centralizzato per i risultati. Contiene `write_sto`,
  `read_sto` e la classe `OutputRecorder` (buffer, registrazione per-step,
  salvataggio batch di tutti i file .sto abilitati).
  (introdotto in: [[2026-04-14_output_espansi_e_visualizzatore]])
- `simulation_runner.py`: loop principale di simulazione. Coordina interpolatore,
  outer loop, inverse dynamics, controllore SEA, static optimization,
  integrazione e delega il salvataggio risultati a `OutputRecorder`.
  (fix e refactor: [[DIVERGENZA_BUGFIXES]], [[2026-04-13_report_muscle_driven_reserve_residuali]])
- `visualize.py`: visualizzatore post-simulazione. Importa `read_sto` da
  `output.py`, carica modello/plugin e riproduce la cinematica con il visualizer
  OpenSim. (introdotto in: [[2026-04-14_output_espansi_e_visualizzatore]])
- `path_resolver.py`: risolve le risorse dalla radice del workspace e conserva
  gli alias dei percorsi salvati nelle configurazioni e nei checkpoint.
- `setup_io.py` e `create_setup.py`: lettura/scrittura e creazione dei setup XML
  del simulatore.
- `Trajectory Generator/`: perimetro dedicato a rete neurale, policy,
  training e generazione di traiettorie protesiche, parallelo a
  `CMC-like Simulator/`, responsabile del CMC-like/OpenSim e dell'integrazione
  dinamica. La separazione attuale riguarda l'organizzazione dei sorgenti:
  l'ambiente del generatore continua a usare il simulatore nello stesso processo.


## Organizzazione dei modelli

Dal 2026-10-15 `models/` contiene le cinque famiglie `AB06/`, `AB07/`,
`AB08/`, `SEASEA/` e `SEASEA_healthy/`. Le tre varianti di ciascun soggetto
(`*-raw`, `*_SEASEA-raw`, `*_SEASEA_Threadmill`) vivono dentro la relativa
famiglia. Leggere `models/README.md` per struttura e limiti dei dati healthy.
Le mesh STL condivise sono in `models/Geometry/`; usare
`path_resolver.SHARED_GEOMETRY_DIR` per i lookup, mantenendo i fallback alle
mesh anatomiche dell'installazione OpenSim.
Usare i percorsi nuovi nelle configurazioni operative e il resolver centrale
`path_resolver.py` per accettare i vecchi percorsi salvati nei checkpoint.
Non riscrivere percorsi o hash nei manifest e negli snapshot storici.

## Istruzioni operative per agenti

- Gli strumenti di preparazione modelli sono in `tools/model_preparation_pipeline/`,
  incluso `run_opensim_sea_pipeline.py`, prima in `scripts/`. Il builder pubblico
  e `build_abx_seasea_pipeline.py`. I sorgenti del plugin GRF sono in
  `online_GRF/sources/`; le librerie compilate restano in `plugins/online_grf_contact/`.
  Leggere `tools/README.md` per comandi e dipendenze dei workflow storici ritirati.

- La geometria binaria heel/toe operativa e in
  `online_GRF/binary_detector/operational/heel_toe_v25.json`.
  I preset e la preparazione modelli usano questo file. Il resolver accetta
  l'esatto vecchio percorso V25 salvato nei checkpoint; preservare gli hash e
  i riferimenti originali nei manifest/snapshot storici.
  Dal 2026-10-20 runtime, profili e strumenti GRF sono riuniti in `online_GRF/`.
  Il runtime di contatto e `online_GRF/runtime.py`, import canonico
  `online_GRF.runtime`; i tre profili di forza sono in `online_GRF/profiles/`.
  Il detector runtime e in
  `online_GRF/binary_detector/binary_phase_detector.py`, con import
  canonico `online_GRF.binary_detector.binary_phase_detector`.
  `model_loader.py` e `simulation_runner.py` lo usano nel simulatore;
  il package non lo importa automaticamente. L'import canonico registra
  gli alias `binary_phase_detector` e il precedente nome qualificato sotto
  `online_grf_profiles` per i pickle: importarlo prima di caricare manualmente
  un pickle che cita quei nomi. Per i pickle che citano `online_grf`, importare
  prima `online_GRF.runtime`. Il resolver riconosce i vecchi percorsi dei
  profili `online_grf_profiles/`; i manifest congelati restano invariati.
  Il vecchio file root
  e rimosso; gli inventari congelati, inclusa la closure F0, conservano il suo
  percorso e hash e richiedono il checkout originale per verificarli.

- Dal 2026-10-20 le FSM binarie base e V26 e i rispettivi adapter sono in
  `Trajectory Generator/phase_tracking/binary_phase.py`, che sostituisce i quattro moduli
  `binary_phase_fsm.py`, `binary_phase_fsm_v26.py`, `binary_phase_adapter.py`
  e `binary_phase_adapter_v26.py`. Le classi base e le sottoclassi V26 restano
  distinte, con gli stessi contratti, tempi e identificatori di provenienza.
  La FSM actor-facing e nel modulo separato
  `Trajectory Generator/phase_tracking/prosthetic_phase_fsm.py`; il detector
  geometrico e in `online_GRF/binary_detector/`. Il bootstrap aggiunge
  `phase_tracking/` a `sys.path`, mantenendo gli import canonici `binary_phase`
  e `prosthetic_phase_fsm` per i pickle e i worker Ray. L'import di
  `binary_phase` registra ancora gli alias dei quattro moduli precedenti.
  Gli inventari congelati che
  elencano i quattro vecchi sorgenti richiedono il checkout originale: non riscriverne
  percorsi o hash per certificare il nuovo layout.

- Dal 2026-10-20 gli strumenti GRF Python sono in
  `online_GRF/calibration/`, `contact/`, `detector/` e `validation/`.
  Provengono da `online_grf_profiles/`, dove erano stati raccolti il 2026-10-17.
  I sorgenti C++ sono in `online_GRF/sources/`, prima `tools/online_GRF/`.
  Compilare dalla radice con `-S online_GRF/sources -B online_GRF/build`
  e le opzioni SDK/OS descritte in `online_GRF/sources/README.md`.
  Il builder della suola legge
  `online_GRF/contact/primary_grf_candidate_grid_v1.json`; i record
  JSON congelati mantengono i loro percorsi/hash interni originali.
  I test del simulatore sono in `validation/test/`; il validatore lanciato da
  `python "CMC-like Simulator/main.py" --validate` resta
  `validation/validate_sim_results.py`.

- Dal 2026-10-19 le procedure di trasferimento dalla rete imitativa al training
  ex novo sono riunite in
  `Trajectory Generator/baseline_MLP/imitation_to_exnovo/procedures/`,
  prima `baseline_MLP/validation/`, poi direttamente in `imitation_to_exnovo/`.
  Comprendono F0/F1/F2R, V26B e V26C; `adaptation/` e `utilities/` restano
  direttamente sotto `imitation_to_exnovo/`.
  Usare il nuovo percorso per i comandi correnti; il resolver riconosce il
  vecchio prefisso nei percorsi salvati. Preservare i byte e gli hash dei
  manifest, delle configurazioni e dei protocolli storici. La rinomina e le
  dipendenze storiche gia mancanti sono documentate in
  `reports/cleanup/2026-10-19_imitation_to_exnovo_rename/README.md`.

- I moduli attivi `start_sampling.py`, `start_condition_metrics.py`,
  `process_watchdog.py`, `progress_display.py`, `tb_logging.py`,
  `exploration.py`, `win_runtime.py`, `prescribed_action.py` e
  `validate_online_grf_train_inference.py` sono in
  `Trajectory Generator/baseline_MLP/utility/`. Il bootstrap della baseline
  mantiene i nomi di import canonici per i pickle e i worker Ray su Windows e
  macOS. Questa cartella e distinta da `imitation_to_exnovo/utilities/`, che
  contiene le CLI per il trasferimento imitativo; i riferimenti nei manifest
  e nelle closure storiche mantengono percorsi e hash originali.

- `warm_start.py` e in
  `Trajectory Generator/baseline_MLP/imitation_to_exnovo/` e resta usato anche
  dal training ordinario. Il generatore del registro `update_historical_runs.py`
  e in `Trajectory Generator/runs/`, accanto ai suoi output. Il bootstrap
  conserva i nomi di import canonici dalle nuove cartelle. Nei processi e nei
  worker chiamare `_bootstrap.ensure_sim_paths()` (sola libreria standard),
  poi importare `win_runtime`, quindi NumPy, Torch, Ray e OpenSim.
  Il validatore GRF richiede ancora un report valido con `--acceptance-report`:
  il file predefinito in `results/` e stato eliminato nella pulizia.

- I quattro moduli `target_domain_imitation.py`, `target_domain_dagger.py`,
  `target_domain_noise_adaptation.py` e `target_domain_markov_adaptation.py`
  sono in `Trajectory Generator/baseline_MLP/imitation_to_exnovo/adaptation/`.
  Prima degli import condivisi chiamare `_bootstrap.ensure_sim_paths()`, che
  mantiene i nomi canonici dei moduli dalla nuova cartella. Le CLI usano il
  nuovo percorso; preservare i riferimenti e gli hash nei manifest e nei
  protocolli storici.

- Il codice morphology e in
  `Trajectory Generator/baseline_MLP/morphology_corridor/experimental_morphology_corridor.py`;
  i due profili JSON AB06 sono nella sottocartella `profiles/`.
  `_bootstrap.ensure_sim_paths()` mantiene il nome di import canonico
  `experimental_morphology_corridor`. Il preset corrente usa il nuovo
  percorso; il resolver accetta i vecchi percorsi `morphology_profiles/`
  salvati nei checkpoint. Preservare i byte dei profili e i percorsi/hash
  originali nelle configurazioni e nei manifest storici.

- Prima di modificare codice, leggere i file coinvolti e preservare le
  invarianti gia presenti, in particolare la struttura high-level Python /
  low-level plugin C++ dei SEA.
- Non modificare il plugin C++ o la semantica del comando SEA senza una richiesta
  esplicita.
- D'ora in poi tutto il lavoro relativo alla rete neurale, alla policy o al
  training deve vivere dentro `Trajectory Generator/`. Non aggiungere logica di
  rete in `CMC-like Simulator/` o nella root del workspace salvo richiesta esplicita.
- Questo progetto deve essere adatto sia per Windows x86 che per MacOS arm 64, ogni modifica al codice deve essere consistente per entrambi gli OS. 
- Salvare i sette plot canonici nella cartella `plot/` del root del repository,
  in una sottocartella dedicata al rollout; non salvarli dentro le cartelle dei run.
- Quando viene inserito il comando `start_day`, leggi questo file `AGENTS.md`,
  `reports/PROJECT_STATE.md` e il daily report più recente in
  `reports/daily/`. Non leggere in blocco gli altri daily report né
  `reports/TODO_BACKLOG.md`: consultali con ricerche mirate (ID del
  TODO, data, argomento) quando servono al lavoro in corso. Fino al daily
  del 2026-10-13 compreso, i report copiano l'intero backlog: leggerne
  solo le sezioni sul lavoro del giorno.
- Prima di lanciare simulazioni, rollout, training, test o build dei plugin,
  leggi `reports/RUNBOOK.md`. Aggiornalo nella stessa sessione in cui
  cambiano un comando, l'ambiente, un preset o una macchina, senza aspettare
  `end_day`. Il RUNBOOK non ricopia i README né `commands.txt`: li linka e
  raccoglie solo ricette correnti, dettagli delle macchine ed errori ricorrenti.
- Quando viene inserito il comando `create_report`, scrivi in `reports/user/` un report Markdown.
- Il titolo del report deve avere il formato:
  `[date]_[titolo significativo].md`
- Esempio:
  `2026-04-14_recruitment_muscle_driven.md`
- Il report deve riassumere almeno problema, soluzione, strategia, file
  modificati e test/verifiche eseguite.
- Quando viene inserito il comando `end_day`, scrivi in `reports/daily/` un report Markdown sulle modifiche fatte oggi e che raduna tutte le informazioni dei report del giorno attuale.
  Poi aggiorna `reports/TODO_BACKLOG.md` e riscrivi
  `reports/PROJECT_STATE.md` con lo stato corrente. Sezioni obbligatorie:
  lavoro in corso e prossimo passo; policy e artefatti di riferimento, con
  i percorsi; decisioni e vincoli vigenti, con data e fonte; struttura della
  codebase; risultati recenti; TODO prioritari; note aperte. Il file di stato
  si riscrive, non si accoda, e resta sotto circa 150 righe, con i dettagli
  nei report linkati. Criterio: una nuova conversazione deve poter ripartire
  senza leggere altro. Infine verifica che `reports/RUNBOOK.md` sia ancora
  corretto.
- i titoli dei daily report devono avere il formato: `[date]_daily-report.md`
- Esempio:
  `2026-04-14_daily-report.md`
- Se esiste un TODO in un user report, va registrato in
  `reports/TODO_BACKLOG.md` con un ID stabile e un link al report e
  citato nel daily report del giorno. Resta nel registro finché non viene
  esaurito; allora si sposta nella sezione «Chiusi» con data e report di
  chiusura. I daily report non copiano più l'intero backlog: elencano per
  ID solo i TODO aperti, chiusi o modificati nel giorno.
