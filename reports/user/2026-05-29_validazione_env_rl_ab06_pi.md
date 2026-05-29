# Validazione env RL AB06 PI

Data: 2026-05-29

## Problema

L'ambiente RL `osim_trj_cmc_like.py` doveva essere validato usando
esplicitamente il setup:

```text
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml
```

Il primo blocco non era nel codice dell'env, ma nel layout dei dati AB06:
il setup XML cercava i file dentro `models/AB06_SEASEA_Threadmill/data/`,
mentre l'estrazione dell'archivio aveva creato un livello annidato:

```text
models/AB06_SEASEA_Threadmill/data/data/
```

Di conseguenza il preflight del test non trovava:

```text
IK_results_AB06_SEASEA.mot
ExternalForces.xml
CMC_Actuators.xml
```

Dopo aver sistemato il layout dati, il reset dell'env ha esposto un secondo
problema: `SimulationRunner.reset_to_time()` chiamava `model.setControls()`
quando lo `State` OpenSim era ancora realizzato solo fino a stage `Instance`.
OpenSim richiedeva almeno uno stato realizzato fino a `Position`/`Velocity`.

## Soluzione

Sono stati resi disponibili nel percorso atteso dal setup AB06 PI i file:

```text
models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot
models/AB06_SEASEA_Threadmill/data/ExternalForces.xml
models/AB06_SEASEA_Threadmill/data/CMC_Actuators.xml
models/AB06_SEASEA_Threadmill/data/AB06_SEASEA_GRF_FullSpan.mot
```

Il file GRF `.mot` e necessario perche `ExternalForces.xml` lo referenzia
come `datafile`.

In `simulation_runner.py` e stato corretto il reset step-wise usato dall'env RL:
dopo la riscrittura di coordinate, velocita, stati muscolari e stati SEA,
il runner realizza lo stato fino a `Velocity` prima di chiamare
`model.setControls()`.

La stessa protezione e stata aggiunta anche al fallback legacy Euler prima di
riapplicare i controlli dopo l'aggiornamento dello stato.

## Strategia

La validazione e stata implementata come smoke suite corta in:

```text
validation/rl_env_smoke_ab06_pi.py
```

La suite usa sempre il setup AB06 PI e non dipende da
`.simulator_last_setup.json`. I test sono intenzionalmente brevi, con episodio
da 2 ms, per separare errori di interfaccia/env da problemi di stabilita su
rollout lunghi.

La suite copre:

- import smoke di OpenSim, SciPy e `osim_trj_cmc_like`;
- preflight dei file dichiarati nel setup XML;
- costruzione env, `reset()`, singolo `step()` con azione zero;
- tre reset nello stesso processo con `rebuild_model_on_reset=False`;
- sanity check con azione `0.0`, `+0.1`, `-0.1`;
- smoke con output registrati e rilettura dei principali `.sto`;
- ispezione separata degli output gia presenti in `results/`, senza usarli
  come input AB06.

## File modificati

```text
simulation_runner.py
```

- Aggiunta realizzazione dello stato fino a `Velocity` prima di
  `model.setControls()` in `reset_to_time()`.
- Aggiunta la stessa precauzione nel fallback legacy Euler.

```text
validation/rl_env_smoke_ab06_pi.py
```

- Aggiunto smoke runner dedicato al setup AB06 PI.
- Aggiunto preflight dei file input.
- Aggiunti test reset/step/action sanity/recorded output.
- Aggiunta modalita `--inspect-existing-results` per validare `.sto` gia
  presenti in `results/`.

```text
models/AB06_SEASEA_Threadmill/data/
```

- Resi disponibili al livello atteso dal setup i file input AB06 PI e il
  file GRF referenziato da `ExternalForces.xml`.

## Test e verifiche

Compilazione Python:

```powershell
python -m py_compile simulation_runner.py osim_trj_cmc_like.py validation\rl_env_smoke_ab06_pi.py
```

Esito: passato.

Import smoke:

```powershell
C:\Users\tomma\anaconda3\Scripts\conda.exe run -n envCMC-like python -c "import opensim, scipy, osim_trj_cmc_like; print('env_import_ok')"
```

Esito: `env_import_ok`.

Smoke completo AB06 PI:

```powershell
C:\Users\tomma\anaconda3\Scripts\conda.exe run -n envCMC-like python validation\rl_env_smoke_ab06_pi.py
```

Esito: passato.

Risultati chiave:

```text
reset_time = 11.99
step_time = 11.992
obs_finite = True
reward finite
terminated = True
truncated = False
reset_reuse_times_s ~= [0.005, 0.005, 0.005]
action_sanity 0.0 = OK
action_sanity +0.1 = OK
action_sanity -0.1 = OK
```

Smoke con output registrati:

```powershell
C:\Users\tomma\anaconda3\Scripts\conda.exe run -n envCMC-like python validation\rl_env_smoke_ab06_pi.py --recorded-output
```

Esito: passato.

Gli output sono stati salvati in:

```text
results/_rl_env_ab06_pi_smoke_20260529
```

File principali generati:

```text
rl_episode_kinematics.sto
rl_episode_states.sto
rl_episode_sea_controls.sto
rl_episode_sea_states.sto
rl_episode_sea_torques.sto
rl_episode_reserve_controls.sto
rl_episode_reserve_torques.sto
rl_episode_tau_bio.sto
rl_episode_muscle_forces.sto
rl_episode_so_torque_diagnostics.sto
rl_episode_gait_events.csv
```

Ispezione output esistenti:

```powershell
C:\Users\tomma\anaconda3\Scripts\conda.exe run -n envCMC-like python validation\rl_env_smoke_ab06_pi.py --inspect-existing-results --max-result-files 10
```

Esito: 10 file `.sto` gia presenti in `results/` riletti correttamente con
valori finiti.

## Note operative

Durante alcuni step compare un warning dello static optimizer:

```text
QP did not converge. Using bounded least-squares fallback.
```

Questo non blocca lo smoke test e l'env restituisce osservazioni/reward finiti,
ma va monitorato prima di passare a rollout lunghi o PPO. Potrebbe indicare che
alcuni target biologici iniziali AB06 sono difficili da soddisfare con le
reserve/muscle bounds correnti.
