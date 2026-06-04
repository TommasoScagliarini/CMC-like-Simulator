# Contesto repo CMC-like-Simulator

## Scopo del progetto

Simulatore OpenSim/Python in stile CMC-like (Computed Muscle Control)
per testare controllori high-level su due attuatori Series Elastic
(SEA) protesici, mantenendo il lato biologico il piu' possibile
muscle-driven e usando le reserve actuators come supporto residuale.

Il modello principale e' un soggetto trans-femorale (AB06) con
protesi knee + ankle a controllo SEA, su treadmill.

## Architettura a due livelli

```text
+----------------------------------------------+
|  Python: high-level controller (cascade)     |
|    - prosthesis_controller.py                |
|    - genera u in [-1, +1] = tau_ref/F_opt   |
+----------------------------------------------+
                       |  u (control vector OpenSim)
                       v
+----------------------------------------------+
|  C++ plugin: low-level motor PI              |
|    - SeriesElasticActuator                   |
|    - integra theta_m, omega_m (rotore)       |
|    - calcola tau_input dal PI interno        |
|    - applica tau_spring = K*(theta_m-theta_j)|
+----------------------------------------------+
                       |  forza generalizzata sul joint
                       v
+----------------------------------------------+
|  OpenSim multibody dynamics                   |
+----------------------------------------------+
```

**Vincolo importante**: il plugin C++ NON va modificato senza richiesta
esplicita. Tutta la logica di controllo high-level e' in Python.

## Dinamica del SEA (formule chiave)

Variabili di stato del plugin (per ogni SEA): `theta_m`, `omega_m`.

```text
tau_spring = K * (theta_m - theta_j)
tau_input  = Kp * (u*F_opt - tau_spring) - Kd * omega_m   (non-impedance)
d(theta_m)/dt = omega_m
d(omega_m)/dt = (tau_input - tau_spring - Bm*omega_m) / Jm
tau_out_to_joint = tau_spring   (non-impedance)
```

`tau_input` e' clampato a +/-500 N*m dal plugin.

Parametri tipici AB06_SEASEA_stiff321_500:

```text
SEA_Knee:  Jm=0.01, Bm=0.10, Ks=321, F_opt=100
SEA_Ankle: Jm=0.01, Bm=0.10, Ks=500, F_opt=250
```

Risonanza meccanica spring-rotor:

```text
omega_mech = sqrt(Ks/Jm)
  knee:  179 rad/s  (28.5 Hz)
  ankle: 224 rad/s  (35.6 Hz)
```

Cap di saturazione del motor PI per stabilita' a step pieno (`u=1`):

```text
omega_n_max = sqrt(500 * Ks / (Jm * F_opt))
  knee:  400 rad/s
  ankle: 316 rad/s   <-- vincolo dominante
```

## Pipeline di simulazione

```text
main.py
  -> config.py            (SimulatorConfig dataclass)
  -> model_loader.py      (carica .osim + plugin, costruisce SimulationContext)
  -> kinematics_interpolator.py (legge IK .sto, spline per q, qdot, qddot)
  -> simulation_runner.py (loop principale, time-stepping a 1 kHz)
       |
       |-- outer_loop.py            (tracking biologico)
       |-- prosthesis_controller.py (cascade SEA, produce u)
       |-- inverse_dynamics.py      (tau_bio via baseline + mass matrix)
       |-- static_optimization.py   (muscle recruitment + reserve)
       |
       -> output.py (OutputRecorder: scrive tutti i .sto)
visualize.py (post: ricostruisce in OpenSim Visualizer)
plot/plotter.py (post: genera PNG standard delle metriche)
```

Tempi di simulazione tipici (full window 11.99 -> 21.0 s = 9 s sim):
~13 min wall su Mac M5.

## Controllore high-level prosthesis

Tre modalita' selezionabili via `sea_outer_controller_mode`:

- **pd**: PD su errore di posizione (deprecata).
- **pid**: PD + integrale con anti-windup.
- **cascade**: cascade outer position -> velocity -> torque (raccomandata).

Cascade outer (modalita' attiva):

```text
e_q             = q_ref - q
qdot_cmd        = qdot_ref + Kp_outer * e_q                  (position loop, P)
e_v             = qdot_cmd - qdot                            (velocity error)
tau_cmd         = Kp_inner * e_v + Ki_inner * integral(e_v)  (velocity loop, PI)
u               = clamp(tau_cmd / F_opt, -1, +1)
```

Filtri opzionali implementati nel controller Python:

- `kinematics_lowpass`: LPF a 6 Hz su qdot_ref e qddot_ref dal IK
  (default abilitato).
- `sea_u_lpf_cutoff_hz`: LPF di primo ordine sul comando `u` in uscita
  (default disabilitato).
- `sea_qdot_feedback_lpf_cutoff_hz`: LPF di primo ordine sul feedback
  `qdot_cur` letto da OpenSim, prima del velocity loop. Aggiunto
  2026-05-19. **Default disabilitato**.

## File Python principali

```text
main.py
  - entry point, CLI parsing, override config, lancio simulazione.

config.py
  - SimulatorConfig dataclass: path, tempi, parametri SEA, tracking
    biologico, static optimization, output. Tutti i default sono qui.

model_loader.py
  - carica .osim, plugin SEA, GRF, reserve actuators.
  - costruisce SimulationContext (nomi, indici, metadati).

kinematics_interpolator.py
  - legge IK .sto, costruisce spline.
  - applica LPF cinematica se abilitato.

outer_loop.py
  - tracking biologico, accelerazioni desiderate per DOF biologici.

inverse_dynamics.py
  - baseline zero-actuator + proiezione mass matrix.
  - evita realizeAcceleration().
  - restituisce tau_bio e tau_pros.

static_optimization.py
  - muscle recruitment biologico.
  - distribuisce tau_bio tra muscoli e reserve.

prosthesis_controller.py
  - cascade SEA outer.
  - filtri u_lpf e qdot_lpf opzionali.
  - feasibility scaling per anti-saturazione.

output.py
  - I/O centralizzato.
  - write_sto, read_sto.
  - OutputRecorder: buffer per-step, save_results() finale.

simulation_runner.py
  - loop principale time-stepping.
  - coordina tutti i moduli sopra.

visualize.py
  - post-simulazione, OpenSim Visualizer.

plot/plotter.py
  - PNG standard (6 plot per run) salvati in plot/<data>_<n>/.

validation/
  - effective_joint_inertia.py: matrice di massa OpenSim lungo IK.
  - cascade_qdot_step_inertia.py: step in qdot_ref.
  - cascade_local_gain_sweep.py + cascade_jeff_todo_runner.py:
    sweep + runner per design alternativi.
```

## Layout file e directory

```text
/                                  repo root
  config.py, main.py, ...           moduli Python principali
  CLAUDE.md, AGENTS.md              istruzioni agenti
  CONTEXT.md                        questo file
  models/                           .osim + .xml setup + IK + GRF
  results/                          output .sto e .csv di ogni run
  plot/                             PNG generati da plotter.py
  reports/
    daily/[YYYY-MM-DD]_daily-report.md
    user/[YYYY-MM-DD]_titolo.md
  validation/                       script di validazione e sweep
  tools/                            tool ausiliari (mappe poli, locus)
```