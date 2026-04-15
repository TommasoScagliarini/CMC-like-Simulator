# Guida per agenti AI

Questo repository contiene un simulatore OpenSim/Python in stile CMC-like per
testare controllori high-level su due attuatori SEA protesici, mantenendo il
lato biologico il piu possibile muscle-driven e usando le reserve come supporto
residuale.

## Plugin C++ — `SeriesElasticActuator`

Eredita da `OpenSim::CoordinateActuator`. Modella un attuatore con molla in
serie (SEA) con due state variable interne: `motor_angle` e `motor_speed`.

- **Proprieta**: `motor_inertia`, `motor_damping`, `stiffness` (K), `Kp`, `Kd`,
  `Impedence` (bool). `optimal_force` (F_opt) e ereditata dal parent.
- **Controllo**: segnale normalizzato `u ∈ [-1, +1]` letto da `getControl(s)`.
- **Dinamica motore** (`computeStateVariableDerivatives`):
  - `tau_spring = K * (theta_m - theta_joint)`
  - Modo non-impedance: `tau_input = Kp * (u*F_opt - tau_spring) - Kd * omega_m`
  - Modo impedance: `tau_input = tau_ff + Kp*(theta_m_ref - theta_m) + Kd*(omega_m_ref - omega_m)`
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

**Coppia elastica della molla** (trasmessa al giunto):

    tau_spring = K * (theta_m - theta_j)

dove `theta_j` e l'angolo del giunto letto dalla coordinata OpenSim.

**Coppia motore** (calcolata dal controllore interno del plugin):

- Modo **non-impedance** (`Impedence = false`):

      tau_ref   = u * F_opt
      tau_input = Kp * (tau_ref - tau_spring) - Kd * omega_m

  Il PD interno insegue la coppia di riferimento `tau_ref` attraverso la
  deflessione della molla. E un loop in coppia.

- Modo **impedance** (`Impedence = true`):

      tau_ref     = u * F_opt
      theta_m_ref = theta_j + tau_ref / K
      omega_m_ref = omega_j
      tau_ff      = tau_spring + Bm * omega_m
      tau_input   = tau_ff + Kp * (theta_m_ref - theta_m) + Kd * (omega_m_ref - omega_m)

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

## File Python

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


## Istruzioni operative per agenti

- Prima di modificare codice, leggere i file coinvolti e preservare le
  invarianti gia presenti, in particolare la struttura high-level Python /
  low-level plugin C++ dei SEA.
- Non modificare il plugin C++ o la semantica del comando SEA senza una richiesta
  esplicita.
- Questo progetto deve essere adatto sia per Windows x86 che per MacOS arm 64, ogni modifica al codice deve essere consistente per entrambi gli OS. 
- Quando viene inserito il comando `start_day` leggi il contesto del progetto indficato in questo file `AGENT.md` e i daily report presenti nella cartella `reports/daily/`
- Quando viene inserito il comando `create_report`, scrivi in `reports/user/` un report Markdown.
- Il titolo del report deve avere il formato:
  `[date]_[titolo significativo].md`
- Esempio:
  `2026-04-14_recruitment_muscle_driven.md`
- Il report deve riassumere almeno problema, soluzione, strategia, file
  modificati e test/verifiche eseguite.
- Quando viene inserito il comando `end_day`, scrivi in `reports/daily/` un report Markdown sulle modifiche fatte oggi e che raduna tutte le informazioni dei report del giorno attuale.
- i titoli dei daily report devono avere il formato: `[date]_daily-report.md`
- Esempio:
  `2026-04-14_daily-report.md`

