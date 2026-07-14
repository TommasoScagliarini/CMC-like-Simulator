# Daily Report - 2026-05-26

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidato

- `reports/user/2026-05-26_mujoco_env_runner_cpu_mjx_parity_e_rl_env.md`

## Problema

Dopo la parity statica servivano un runner CPU end-to-end, un percorso MJX
batched, un harness OpenSim-MuJoCo e un ambiente RL che comandasse target
protesici invece di muscoli o coppie dirette.

## Soluzione e strategia

Sono stati implementati:

- runner MuJoCo CPU con GRF replay, ID, SO, SEA e output `.sto`;
- backend MJX con `jax.vmap`/`jax.jit`;
- parity harness con report JSON/HTML;
- `ProstheticTrajectoryEnv` a `50 Hz`, con substep fisici a `1 kHz`;
- modalita' separate `replay` e `contact` per evitare doppia applicazione GRF.

La formula SEA non-impedance e la feasibility prediction sono state allineate
all'output effettivo OpenSim.

## Risultati

```text
pytest = 31 passed
CPU smoke = complete
MJX raw N=1 = verificato
pros_ankle_angle RMSE = 0.00869 rad, gate PASS
q mean RMSE = 0.00631 rad
```

La parity geometrica e cinematica protesica passa. Restano FAIL scientifici su
`q_bio_rmse`, coppia SEA e attivazioni: la SEA torque RMSE massima e' circa
`112.75 Nm` e la frazione di attivazioni con correlazione accettabile circa
`0.0217`.

## File principali

Nel workspace esterno `MuJoCo_env`: `simulation_runner.py`, `mjx_runner.py`,
`trajectory_env.py`, `parity.py`, `tools/parity_test.py`, gli script CPU/MJX/RL
e `models/SEASEA/conversion_notes.md`.

## Test e verifiche

- suite: `31 passed`;
- fase 1 statica: PASS;
- CPU smoke: PASS;
- MJX N=1: verificato;
- gate cinematico protesico: PASS;
- gate torque/recruitment: FAIL documentato, non rilassato.

## TODO aperti e propagati

- [ ] Diagnosticare la parity della coppia motore SEA.
- [ ] Migliorare force-gain/equilibrio Thelen2003.
- [ ] Validare i momenti ExternalLoads sul lato protesico.
- [ ] Eseguire test MJX multi-step e benchmark GPU Linux/WSL2.
- [ ] Validare rollout RL lunghi in modalita' `contact`.
- [ ] Non usare attivazioni MuJoCo come target o hard reward finche' la parity
      recruitment resta insufficiente.

Il filone e' ripreso nella sezione MuJoCo/MJX del daily del 14 giugno.

