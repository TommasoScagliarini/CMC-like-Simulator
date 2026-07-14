# Daily Report - 2026-05-25

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidato

- `reports/user/2026-05-25_creazione_mujoco_env_porting_opensim_mjx.md`

## Problema

Il throughput del simulatore OpenSim CMC-like era insufficiente per training RL
su larga scala. Serviva un porting MuJoCo/MJX che preservasse modello AB06,
muscoli, reserve, SEA e semantica del controllo.

## Soluzione e strategia

Nel workspace esterno Windows `C:\Users\tomma\Desktop\MuJoCo_env` e' stato
creato il package `mujoco_env`, con conversione `.osim -> MJCF`, dinamica SEA
Python/JAX, controller, ID, static optimization, output `.sto`, backend CPU e
predisposizione MJX. La coppia SEA viene applicata tramite `qfrc_applied`, senza
trasformare i SEA in normali actuator MuJoCo.

La conversione iniziale comprendeva `11` body, `21` coordinate indipendenti,
`14` coupling, `76` muscoli Thelen2003 e `21` reserve.

## Risultati

La validazione statica di fase 1 ha superato i gate:

```text
phase_1_complete = true
mass error = 0 kg
CoM max error = 2.78e-6 m
mass matrix max relative error = 3.75e-5
moment-arm pass fraction entro 10% = 0.9667
```

## File principali

I file sono nel workspace esterno `MuJoCo_env`; i principali moduli sono
`model_loader.py`, `sea_plugin.py`, `prosthesis_controller.py`,
`inverse_dynamics.py`, `static_optimization.py`, `simulation_runner.py`,
`mjx_runner.py` e gli strumenti `osim2mjcf.py`/`parity_test.py`.

## Test e verifiche

- caricamento MJCF: PASS;
- masse, CoM e mass matrix: PASS;
- coupling cinematici: PASS;
- conversione muscoli/reserve: PASS;
- import package nell'ambiente `mujoco-cmc`: PASS.

## TODO propagati al 26 maggio

- [ ] Validare la dinamica end-to-end, non solo geometria e statica.
- [ ] Quantificare gli errori introdotti da wrap object e path point mobili.
- [ ] Chiudere parity SEA, muscoli e recruitment prima di usare MuJoCo come
      sostituto scientifico di OpenSim.

