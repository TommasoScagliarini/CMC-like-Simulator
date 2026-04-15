# Bug Che Causavano La Divergenza (e Fix Applicati)

Questo documento riassume in modo puntuale i bug che portavano la simulazione a
divergere (valori che esplodono nei primi istanti, poi `nan/inf`) e le
correzioni applicate nel progetto.

## Sintomo osservato

- Nei file `results/*.sto` i valori di molte coordinate crescevano a dismisura
  gia' tra `t=4.27` e `t=4.34 s`, poi comparivano `nan`.
- Nel log l'errore tipico era un `FloatingPointError` per accelerazioni non
  finite, ad esempio:
  `Non-finite accelerations at t=4.3000: ['pelvis_tilt', 'pelvis_list', ...]`.
- I comandi SEA (`u_sea`) tendevano a saturare a `-1/+1` subito (entro pochi
  step), innescando dinamiche violentissime.

## Root cause 1: indici delle mobilita' (U/UDot) sbagliati

### Problema

In `model_loader.py` il simulatore assumeva implicitamente che:

1. l'ordine del `CoordinateSet` (coordinate del modello), e
2. l'ordine del vettore globale Simbody delle mobilita' `U/UDot`

fossero lo stesso.

Nel modello in uso questo non vale. Risultato: quando il codice faceva accessi
del tipo:

- `udot[idx]` per prendere la qddot di una coordinata, oppure
- assegnava un torque ad una riga della mass matrix / residual,

il torque/accelerazione veniva "instradato" su un DOF diverso da quello atteso.

Questo e' un bug devastante: anche una coppia "ragionevole" applicata al DOF
sbagliato puo' generare accelerazioni enormi e destabilizzare subito lo stato.

### Fix

File: `model_loader.py`

- E' stata aggiunta una fase di inferenza della mappa `coord_name -> mobility_index`
  usando i reserve actuators (CoordinateActuator) come "probe":
  per ogni coordinata si applica un controllo unitario a un suo reserve actuator,
  si osserva in quale indice della residual di inverse dynamics compare la forza,
  e si usa quell'indice come mobility index.

Punti chiave dell'implementazione:

- funzione `_infer_coordinate_mobility_indices(...)` (vedi `model_loader.py:114`)
- uso di `InverseDynamicsSolver.solve(state, zero_udot)` e confronto residual
  baseline vs residual con probe
- check contro probe che non produce effetto e check contro indici duplicati

Effetto:

- Tutti i vettori in mobility-order (mass matrix, residual, udot) vengono ora
  indicizzati in modo consistente con Simbody.

## Root cause 2: controllo SEA PD scalato male (PD su u invece che su tau)

### Problema

Il plugin SEA (sorgente C++) usa esplicitamente (equazioni complete in [[AGENT]]):

- `tau_ref = u * F_opt`

Quindi `u` e' un comando normalizzato e la coppia fisica e' `tau_ref`.

Nel controllo high-level Python, inizialmente il PD veniva sommato direttamente
in spazio `u` (cioe' normalizzato), mentre il feed-forward veniva gia'
normalizzato. Questo rendeva i guadagni effettivi enormi:

- anche un piccolo errore di velocita' produceva una variazione enorme in `u`,
  causando saturazione e cambi di torque violentissimi.

Nella pratica questo e' cio' che innescava l'esplosione rapida dei DOF protesici
(e poi, per accoppiamento inerziale, anche di tutto il resto).

### Fix

File: `prosthesis_controller.py`

Il PD e' stato riportato in unita' fisiche di coppia, e solo alla fine viene
normalizzato:

- `tau_cmd = tau_ff + Kp*(q_ref - q) + Kd*(qdot_ref - qdot)`
- `u = clip(tau_cmd / F_opt, -1, +1)`

Riferimenti:

- formula descritta nella docstring (vedi `prosthesis_controller.py:16`)
- implementazione in `compute()` (vedi `prosthesis_controller.py:117`)

Conseguenza pratica:

- `sea_kp` e `sea_kd` in `config.py` ora sono guadagni di coppia:
  `Kp [N*m/rad]`, `Kd [N*m*s/rad]` (vedi `config.py:74`).

## Root cause 3: baseline "zero-actuator" in inverse dynamics non era zero per i SEA

### Problema

In non-impedance mode il SEA applica:

- `tau = K * (motor_angle - theta_joint)`

Quindi anche con `u=0`, se `motor_angle != theta_joint` esiste comunque coppia
di molla.

L'inverse dynamics frame-by-frame usa un baseline "zero actuation" per stimare
`qddot_0`, poi calcola `tau = M * (qddot_des - qddot_0)`. Se nel baseline il SEA
non e' davvero zero, `qddot_0` e' falsato e il feed-forward risulta errato.

### Fix

File: `inverse_dynamics.py`

Durante il baseline vengono azzerate anche le molle SEA, imponendo:

- `motor_angle = theta_joint` (deflessione molla zero)
- `motor_speed = 0` (per coerenza del baseline)

e poi lo stato del motore viene ripristinato a fine calcolo.

Riferimento:

- `InverseDynamicsComputer.compute_tau()` (vedi `inverse_dynamics.py:137`)

## Root cause 4: modello muscolare "lineare" nell'SO non era coerente con OpenSim

### Problema

La static optimization costruiva una matrice `R * Fmax` usando `computeMomentArm`
e `Fmax` come se la coppia muscolare istantanea fosse:

- `tau_muscle ~= moment_arm * (activation * Fmax)`

Ma nel modello OpenSim reale (Thelen) la forza istantanea dipende dagli stati
muscolari (lunghezza fibra, velocita', attivazione, pennation, ecc.). Quindi
la soluzione "matematicamente" corretta per quel `A x = tau_des` non produceva
le forze vere quando applicata al modello: il tracking diventava incoerente e
instabile nel closed-loop.

### Fix

File: `config.py`, `static_optimization.py`

Per evitare un modello fisicamente sbagliato:

- e' stato introdotto `use_muscles_in_so: bool = False` (default)
- quando `False`, il blocco muscolare della matrice A e' zero e tutta la
  coppia biologica richiesta e' applicata tramite reserve actuators (che sono
  esattamente coerenti con la dinamica OpenSim del bypass).

La riscrittura completa della SO muscle-driven (con `use_muscles_in_so=True`) e'
documentata in [[2026-04-13_report_muscle_driven_reserve_residuali]].

Riferimenti:

- flag e motivazione: `config.py:136`
- gating del blocco muscolare: `static_optimization.py:194`

Nota: questo rende il file `sim_output_activations.sto` tutto a zero (coerente
con la scelta). L'obiettivo del progetto (test controlli high-level sui SEA)
rimane pienamente supportato.

## Root cause 5 (stabilita' numerica): fallback QP e limiti riserve troppo stretti

### Problema

- Il QP (SLSQP) ogni tanto non converge.
- La versione precedente riusava la soluzione precedente o aveva limiti stretti
  su `reserve_u_max`, rendendo piu' facile entrare in stati non fisici durante
  transitori iniziali.

### Fix

File: `static_optimization.py`, `config.py`

- Fallback robusto: bounded least-squares (`lsq_linear`) pesato per rispettare
  i pesi del costo QP (vedi `static_optimization.py:160`).
- Cap numerico dei reserve controls aumentato: `reserve_u_max = 1000.0`
  (vedi `config.py:134`).

## Fix necessari per coerenza con il plugin SEA non-impedance

### Aggiornamento dello stato del motore (motor_angle/motor_speed)

Con `Impedence=false` il plugin applica la coppia di molla. Dato che la
simulazione evita `realizeAcceleration` (per evitare crash), le derivate degli
state variables del plugin non vengono integrate automaticamente. Per rendere
il SEA controllabile a livello alto, il runner imposta il `motor_angle`
all'equilibrio che produce la coppia richiesta:

- `motor_angle = theta_joint + tau_required / K`

File: `simulation_runner.py`

- helper `_update_sea_motor_state(...)` (vedi `simulation_runner.py:465`)
- chiamata sia prima del calcolo udot (forze applicate al passo corrente),
  sia dopo l'integrazione dello stato (coerenza al passo successivo)

### Ordine corretto: setStateVariableValues invalida cache

Ogni `model.setStateVariableValues(...)` invalida cache di stage; per questo
prima di `setControls` e prima di valutare forze viene rialzato lo stage con
`model.realizeVelocity(state)` (vedi `simulation_runner.py:263-265`).

## Verifica effettuata

Run completa con la configurazione default:

- intervallo: `t in [4.26, 11.06] s`, `dt=0.01`
- 680 step completati senza eccezioni
- `results/sim_output_kinematics.sto`: nessun `nan/inf`
- `results/sim_output_sea_controls.sto`: nessun `nan/inf`, `|u|` sotto saturazione
- `results/sim_output_tau_bio.sto`: nessun `nan/inf`

## Vedi anche

- [[2026-04-13_report_muscle_driven_reserve_residuali]] — riscrittura SO muscle-driven e fix tau_ff SEA
- [[2026-04-14_output_espansi_e_visualizzatore]] — fix state cache `multiplyByM` e output espansi
- [[AGENT]] — equazioni complete della dinamica SEA (PD, molla, dinamica motore)

## File toccati (logica)

- `model_loader.py`: inferenza mapping coordinate -> mobility index, cache SEA
- `inverse_dynamics.py`: ID per bio+pros, baseline con molle SEA azzerate
- `prosthesis_controller.py`: PD in coppia fisica + normalizzazione finale
- `simulation_runner.py`: ordine step, update stato motore SEA, guard non-finite udot
- `static_optimization.py`: gating muscoli + fallback lsq
- `config.py`: documentazione guadagni SEA, `use_muscles_in_so`, `reserve_u_max`, `sea_stiffness`

