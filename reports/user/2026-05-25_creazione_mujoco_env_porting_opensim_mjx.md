# Creazione di MuJoCo_env e porting OpenSim verso MuJoCo/MJX

**Data originale:** 2026-05-25  
**Report recuperato il:** 2026-06-12

Questo report e stato ricostruito dalle cronologie locali Codex e dal piano
originale:

`C:\Users\tomma\.claude\plans\scrivimi-un-piano-in-staged-wilkes.md`

## Problema

Il simulatore OpenSim CMC-like aveva un throughput insufficiente per training
RL su larga scala. Il loop comprendeva:

- modello muscoloscheletrico AB06;
- muscoli Thelen2003 e reserve biologiche;
- due SEA protesici;
- controller protesico cascade;
- inverse dynamics e static optimization;
- integrazione a `1 kHz`.

L'obiettivo era creare un nuovo progetto in:

`C:\Users\tomma\Desktop\MuJoCo_env`

preservando parametri e semantica di controllo, ma sostituendo il backend
OpenSim/plugin C++ con MuJoCo CPU e MJX/JAX.

## Soluzione

E stato creato il package installabile `mujoco_env`, con ambiente Conda
dedicato `mujoco-cmc` e una struttura modulare equivalente al simulatore
OpenSim.

Componenti principali:

- conversione `.osim` verso MJCF tramite `tools/osim2mjcf.py`;
- loader MuJoCo e `SimulationContext`;
- dinamica SEA reimplementata in Python/JAX;
- controller PD/PID/cascade;
- outer loop biologico, inverse dynamics e static optimization;
- output `.sto` compatibili;
- backend MuJoCo CPU e predisposizione MJX;
- strumenti di validazione contro OpenSim.

La conversione iniziale ha prodotto:

```text
11 body OpenSim
21 coordinate indipendenti
14 joint dipendenti/coupled
76 muscoli Thelen2003 convertiti
21 reserve actuators
```

`SEA_Knee` e `SEA_Ankle` non sono attuatori MuJoCo: la coppia SEA viene
applicata tramite `qfrc_applied`, preservando la separazione tra controller
high-level e dinamica low-level.

## Strategia

Il porting e stato organizzato per fasi:

1. bootstrap del repository e ambiente `mujoco-cmc`;
2. conversione e validazione statica del modello MJCF;
3. porting SEA e controller cascade in JAX;
4. porting static optimization, cinematica, outer loop e inverse dynamics;
5. output recorder compatibile `.sto`;
6. runner CPU come ground truth;
7. backend MJX batched;
8. parity harness OpenSim-MuJoCo;
9. ambiente RL di traiettoria protesica.

La Fase 1 ha mantenuto come gate:

- masse identiche;
- errore CoM inferiore a `1 mm`;
- errore relativo mass matrix inferiore a `1%`;
- almeno il `90%` dei moment arm entro il `10%`.

## File creati principali

```text
C:\Users\tomma\Desktop\MuJoCo_env\AGENTS.md
C:\Users\tomma\Desktop\MuJoCo_env\CONTEXT.md
C:\Users\tomma\Desktop\MuJoCo_env\README.md
C:\Users\tomma\Desktop\MuJoCo_env\environment.yml
C:\Users\tomma\Desktop\MuJoCo_env\pyproject.toml
C:\Users\tomma\Desktop\MuJoCo_env\requirements.txt
C:\Users\tomma\Desktop\MuJoCo_env\requirements-gpu-linux.txt

C:\Users\tomma\Desktop\MuJoCo_env\mujoco_env\config.py
C:\Users\tomma\Desktop\MuJoCo_env\mujoco_env\model_loader.py
C:\Users\tomma\Desktop\MuJoCo_env\mujoco_env\sea_plugin.py
C:\Users\tomma\Desktop\MuJoCo_env\mujoco_env\prosthesis_controller.py
C:\Users\tomma\Desktop\MuJoCo_env\mujoco_env\outer_loop.py
C:\Users\tomma\Desktop\MuJoCo_env\mujoco_env\inverse_dynamics.py
C:\Users\tomma\Desktop\MuJoCo_env\mujoco_env\static_optimization.py
C:\Users\tomma\Desktop\MuJoCo_env\mujoco_env\simulation_runner.py
C:\Users\tomma\Desktop\MuJoCo_env\mujoco_env\output.py
C:\Users\tomma\Desktop\MuJoCo_env\mujoco_env\mjx_runner.py

C:\Users\tomma\Desktop\MuJoCo_env\tools\osim2mjcf.py
C:\Users\tomma\Desktop\MuJoCo_env\tools\validate_model.py
C:\Users\tomma\Desktop\MuJoCo_env\tools\parity_test.py
```

## Test e verifiche

La validazione statica finale della Fase 1 risulta completata:

```text
phase_1_complete = true
max_mass_abs_error = 0.0 kg
CoM max error = 2.78e-6 m
mass matrix max relative error = 3.75e-5
moment arm pass fraction entro 10% = 0.9667
```

Sono stati inoltre verificati:

- caricamento MJCF;
- presenza di tutte le coordinate della posa OpenSim;
- applicazione dei 14 coupling cinematici;
- conversione di 76 muscoli e 21 reserve;
- import del package nell'ambiente `mujoco-cmc`.

## Limiti aperti

- Gli wrap object OpenSim non sono convertiti esattamente.
- Moving/ConditionalPathPoint restano approssimazioni cinematiche.
- Il modello muscolare MuJoCo non replica esattamente l'equilibrio e la
  pennation Thelen2003.
- La parity dinamica end-to-end richiede validazioni separate rispetto alla
  parity geometrica/statica.

