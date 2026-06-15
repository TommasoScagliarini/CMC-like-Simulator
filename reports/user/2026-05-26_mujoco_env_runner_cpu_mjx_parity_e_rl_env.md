# MuJoCo_env: runner CPU, MJX, parity e ambiente RL

**Data originale:** 2026-05-26  
**Report recuperato il:** 2026-06-12

Questo report e stato ricostruito dalla sessione Codex che ha creato
`C:\Users\tomma\Desktop\MuJoCo_env` e dalle note:

`C:\Users\tomma\Desktop\MuJoCo_env\models\SEASEA\conversion_notes.md`

## Problema

Dopo il bootstrap e la conversione statica del modello servivano:

- un runner MuJoCo CPU end-to-end utilizzabile come ground truth;
- un percorso MJX batched per throughput elevato;
- un parity harness riproducibile contro OpenSim;
- un ambiente RL che controllasse traiettorie protesiche senza comandare
  direttamente muscoli o coppie.

Era inoltre necessario distinguere chiaramente:

- problemi infrastrutturali del porting;
- mismatch scientifici reali tra OpenSim e MuJoCo.

## Soluzione

### Runner CPU

E stato implementato `mujoco_env.simulation_runner.SimulationRunner` con:

- caricamento cinematica e coupling;
- GRF da ExternalLoads in modalita `replay`;
- outer loop biologico;
- inverse dynamics e static optimization;
- controller e dinamica SEA;
- integrazione e output `.sto`;
- diagnostica di parity e coppie SO per coordinata.

Il runner usa gravita Y-up `[0, -9.80665, 0]`. In modalita replay i contatti
MuJoCo sono disabilitati e vengono applicate le GRF prescritte.

### Backend MJX

E stato creato `mujoco_env/mjx_runner.py`:

- inizializzazione da IK e coupling cinematici;
- replica di `mjx.Data` su piu environment;
- avanzamento con `jax.vmap` e `jax.jit`;
- smoke N=1 contro MuJoCo CPU.

Windows resta il percorso CPU/dev. Il percorso GPU previsto e Linux/WSL2.

### Parity harness

Il comando `tools/parity_test.py`:

- esegue MuJoCo CPU/MJX;
- puo rigenerare gli output OpenSim;
- allinea le colonne `.sto` per nome;
- genera `manifest.json`, `parity_metrics.json` e `parity_report.html`.

### Ambiente RL MuJoCo

E stato creato:

`mujoco_env.trajectory_env.ProstheticTrajectoryEnv`

La policy emette un target assoluto:

```text
(pros_knee_angle, pros_ankle_angle)
```

Il target viene aggiornato a `50 Hz` e interpolato sui substep interni a
`1 kHz`. Il lato biologico resta controllato dallo stack CMC-like con muscoli,
reserve e static optimization.

Sono state introdotte due modalita fisiche esplicite:

- `replay`: GRF ExternalLoads attive, contatti MuJoCo disabilitati;
- `contact`: contatti MuJoCo attivi, GRF ExternalLoads disabilitate.

## Correzioni applicate

- Il runner CPU usa realmente il percorso `rk4_bypass`.
- La formula SEA non-impedance MuJoCo/JAX e stata allineata all'output
  effettivo OpenSim:

```text
tau_input = tau_ref + Kp*(tau_ref - tau_spring) - Kd*omega_m
```

- La feasibility prediction del controller usa la stessa formula.
- Le label delle reserve torque sono state corrette.
- E stato aggiunto `*_so_torque_diagnostics.sto`.
- L'ambiente RL passa correttamente le coppie biologiche allo step interno.

## File principali modificati o creati

```text
C:\Users\tomma\Desktop\MuJoCo_env\mujoco_env\simulation_runner.py
C:\Users\tomma\Desktop\MuJoCo_env\mujoco_env\model_loader.py
C:\Users\tomma\Desktop\MuJoCo_env\mujoco_env\sea_plugin.py
C:\Users\tomma\Desktop\MuJoCo_env\mujoco_env\prosthesis_controller.py
C:\Users\tomma\Desktop\MuJoCo_env\mujoco_env\static_optimization.py
C:\Users\tomma\Desktop\MuJoCo_env\mujoco_env\trajectory_env.py
C:\Users\tomma\Desktop\MuJoCo_env\mujoco_env\mjx_runner.py
C:\Users\tomma\Desktop\MuJoCo_env\mujoco_env\parity.py
C:\Users\tomma\Desktop\MuJoCo_env\tools\parity_test.py
C:\Users\tomma\Desktop\MuJoCo_env\scripts\run_simulation.py
C:\Users\tomma\Desktop\MuJoCo_env\scripts\run_mjx_batch.py
C:\Users\tomma\Desktop\MuJoCo_env\scripts\run_rl_smoke.py
C:\Users\tomma\Desktop\MuJoCo_env\models\SEASEA\conversion_notes.md
```

## Test e verifiche

Verifiche registrate:

```text
pytest: 31 passed
Fase 1 validation: passed
CPU smoke: completed
MJX raw N=1 parity: checked against CPU
q_pros_rmse gate: PASS
```

Metriche finali recuperate:

```text
pros_ankle_angle_q RMSE = 0.00869 rad, gate 0.01
ankle_angle_r_q RMSE = 0.00364 rad
q mean RMSE = 0.00631 rad
```

Artefatti principali:

```text
C:\Users\tomma\Desktop\MuJoCo_env\validation\model_validation_bootstrap.json
C:\Users\tomma\Desktop\MuJoCo_env\validation\parity_runs\final_rl_parity_cpu_50ms\
C:\Users\tomma\Desktop\MuJoCo_env\validation\parity_runs\staged_cpu_replay_opensim_latest\
```

## Stato della parity

Pass:

- parity geometrica/statica Fase 1;
- gate cinematico protesico `q_pros_rmse`.

Fail scientifici ancora aperti:

- `q_bio_rmse`;
- `sea_torque_rmse`;
- correlazione delle attivazioni.

La massima SEA torque RMSE resta circa `112.75 Nm`, guidata da
`SEA_Ankle_tau_motor`. La frazione di attivazioni con correlazione accettabile
resta circa `0.0217`.

Questi mismatch non sono stati nascosti ne trasformati in gate permissivi:
restano diagnostiche da risolvere prima di considerare MuJoCo un sostituto
scientificamente equivalente a OpenSim per effort e recruitment.

## TODO

- Diagnosticare la parity della coppia motore SEA.
- Migliorare la replica del force-gain/equilibrio Thelen2003.
- Validare la parity dei momenti ExternalLoads sul lato protesico.
- Eseguire test MJX multi-step e benchmark GPU Linux/WSL2.
- Validare rollout RL lunghi in modalita `contact`.
- Non usare le attivazioni MuJoCo come target o hard reward finche la parity
  del recruitment resta insufficiente.

