# Report - Output espansi, fix state cache e visualizzatore

## Data e ora

- Data: 2026-04-14
- Ora: 12:15 CEST
- Workspace: `/Users/tommy/Documents/CMC-like-Simulator - Claude`

## Problemi

1. **Crash `multiplyByM` per state cache obsoleta**
   - A `t=4.26 s` la simulazione crashava con `RuntimeError: State Cache entry was out of date at Stage Time`.
   - La causa era in `inverse_dynamics.py`: dopo aver ripristinato i motor angle SEA con `model.setStateVariableValues()` (Step 3, riga 207), il secondo `build_mass_matrix()` (Step 5, riga 216) chiamava `matter.multiplyByM()` su uno state con cache Instance invalidata.
   - `setStateVariableValues()` invalida tutte le stage dipendenti, ma nessun `realize*()` veniva chiamato prima del secondo uso della mass matrix.
   - Il pattern `setStateVariableValues` + `realizeVelocity` necessario per il baseline SEA e' documentato in [[DIVERGENZA_BUGFIXES]] (root cause 3).

2. **Mancanza di un visualizzatore per ispezionare il movimento**
   - Non esisteva uno strumento per visualizzare la cinematica risultante dalla simulazione.

3. **Output `.sto` incompleti**
   - Mancavano: forze muscolari, coppie SEA (molla + motore), stati completi (q, qdot, qddot).
   - Non era possibile analizzare la catena di forza dal motore SEA al giunto senza ricostruire manualmente `tau_spring` e `tau_input`.

4. **Nessuna possibilita di esportare video della simulazione**
   - Il visualizzatore Simbody non ha funzionalita di registrazione video integrata.

## Soluzioni

### 1. Fix state cache in `inverse_dynamics.py`

Aggiunta una chiamata `model.realizePosition(state)` prima del secondo `build_mass_matrix()` in `compute_tau()`. Questo ri-realizza le stage Instance -> Time -> Position dopo che `setStateVariableValues()` le aveva invalidate.

### 2. Visualizzatore post-simulazione (`visualize.py`)

Creato script standalone che:
- Carica il plugin SEA e il modello `.osim` con `setUseVisualizer(True)`
- Legge qualsiasi file `.sto` (supporta sia `inDegrees=yes` che `no`)
- Riproduce la cinematica frame-by-frame tramite il Simbody visualizer
- Supporta: `--speed` (moltiplicatore), `--loop`, `--ik` (file IK diretto), `--t-start`/`--t-end` (finestra temporale), `--geometry-dir`

### 3. Nuovi output `.sto`

Aggiunti 3 nuovi file di output, controllati da flag in `config.py`:

| File | Flag | Contenuto |
|------|------|-----------|
| `sim_output_muscle_forces.sto` | `save_muscle_forces` | `F_i = a_i * F_max_i` per ogni muscolo [N] |
| `sim_output_sea_torques.sto` | `save_sea_torques` | `tau_spring` e `tau_motor` per ogni SEA [N-m] |
| `sim_output_states.sto` | `save_states` | `q`, `qdot`, `qddot` per tutte le 21 coordinate |

**Dettagli SEA torques** (replicati dal sorgente C++ del plugin, equazioni complete in [[AGENT]]):
- `tau_spring = K * (theta_motor - theta_joint)` -- coppia a valle della molla
- `tau_input` (coppia motore, a monte della molla):
  - Non-impedance: `Kp * (tau_ref - tau_spring) - Kd * omega_m`
  - Impedance: `tau_ff + Kp * (theta_m_ref - theta_m) + Kd * (omega_m_ref - omega_m)`
  - Clamped a +/-500 N-m come nel plugin

Le proprieta del plugin (K, Kp, Kd, Bm, F_opt, Impedence) vengono lette una sola volta via `getPropertyByName()` e cachate nell'`__init__` di `SimulationRunner`.

**Accelerazioni** (`qddot`): provengono dal `compute_udot_bypass()` (step H del loop), ovvero le accelerazioni calcolate dalla dinamica effettiva, non copiate dal riferimento IK.

### 4. Registrazione video (`--save-video`)

Aggiunto flag `--save-video` al visualizzatore:
- Cattura ogni frame con `screencapture -x -l <windowID>` (macOS)
- Trova la finestra Simbody tramite `CoreGraphics` (via `swift`)
- Combina i frame in MP4 con `ffmpeg` (libx264, crf=18)
- Salva in `results/video/HHMMSS_DDMMYYYY.mp4`

## File modificati

| File | Tipo modifica |
|------|---------------|
| `inverse_dynamics.py` | Fix: aggiunto `model.realizePosition(state)` prima del secondo `build_mass_matrix()` |
| `config.py` | Aggiunti 3 flag: `save_muscle_forces`, `save_sea_torques`, `save_states` |
| `simulation_runner.py` | Cache proprieta SEA, nuovi buffer, logica di recording e salvataggio per i 3 nuovi output |
| `visualize.py` | Creato da zero: visualizzatore + registrazione video |

## Verifiche eseguite

- Syntax check (`py_compile`) su tutti i file modificati: OK
- Import test di `visualize.py` con opensim: OK
- CLI `--help` verificato per `visualize.py`: OK
- Confronto numerico `output_kinematics.sto` vs riferimento IK: valori coincidenti (kinematic forcing confermato)
- Verifica disponibilita `ffmpeg` (homebrew) e `screencapture` (macOS nativo): OK
- Verifica lookup finestra Simbody via `swift`/`CoreGraphics`: funzionante

## Vedi anche

- [[DIVERGENZA_BUGFIXES]] — root cause storici della simulazione, incluso baseline SEA e invalidazione cache
- [[2026-04-13_report_muscle_driven_reserve_residuali]] — SO muscle-driven e diagnostica recruitment (output `sim_output_recruitment.sto`)
- [[AGENT]] — equazioni SEA (tau_spring, tau_input, dinamica motore)

## Note

- Il file `sim_output_kinematics.sto` contiene valori identici al riferimento IK (convertiti in radianti) quando la simulazione usa kinematic forcing puro. Con il loop Euler semi-implicito attuale (step J), le cinematiche sono integrate dinamicamente e possono divergere dal riferimento.
- La registrazione video e' piu lenta del real-time (~100ms per frame di overhead da `screencapture`), ma il video risultante viene riprodotto al FPS corretto.
