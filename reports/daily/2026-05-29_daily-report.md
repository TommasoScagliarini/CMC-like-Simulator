# Daily report - 2026-05-29

## Sintesi

Giornata dedicata alla trasformazione del simulatore CMC-like in un ambiente
RL step-wise per generare traiettorie cinematiche protesiche e alla sua prima
validazione sul setup AB06 PI:

```text
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml
```

Sono stati prodotti due user report:

```text
reports/user/2026-05-29_env_rl_traiettorie_cmc_like_e_overview_llm.md
reports/user/2026-05-29_validazione_env_rl_ab06_pi.md
```

## Lavoro svolto

### Env RL per traiettorie protesiche

Creato `osim_trj_cmc_like.py`, adapter Gymnasium-like in cui la policy non
comanda direttamente muscoli, reserve o coppie, ma genera segmenti di
traiettoria per:

```text
pros_knee_angle
pros_ankle_angle
```

Le coordinate non protesiche restano guidate dai dati cinematici `kin_ref`.
Il segmento generato dalla policy viene interpolato e inserito come riferimento
protesico per l'outer loop dei due SEA.

Caratteristiche principali:

- action space su knot di traiettoria dei due DOF protesici;
- modalita azione `delta`, `absolute`, `raw`;
- wrapper cinematico `ProstheticSegmentKinematics`;
- reward con termini di tracking policy, riferimento IK, tracking biologico,
  effort e smoothness;
- `fail_fast=True` per sviluppo e `fail_fast=False` per convertire eccezioni
  in `truncated=True` con traceback in `info`;
- default `rebuild_model_on_reset=False` per uso RL.

### API pubblica step-wise nel runner

Esteso `simulation_runner.py` per evitare che l'env RL dipenda da metodi
privati del runner. Sono state aggiunte API pubbliche:

```text
reset_to_time(t)
step_until(t_stop, record=True)
save_results()
reset_outputs()
state
current_time
last_step_info
```

Questa modifica sostituisce l'accoppiamento diretto a metodi/attributi
underscored come `_compute_controls_for_window`, `_integrate_evaluate`,
`_set_state`, `_so`, `_prosthesis_ctrl`.

### Overview per LLM

Creato `LLM_SIMULATOR_OVERVIEW.md`, documento sintetico ma completo per altri
LLM. Contiene:

- scopo del simulatore;
- pipeline CMC-like;
- distinzione parte biologica e protesica;
- dinamica SEA low-level nel plugin C++;
- controllo protesico cascade Python;
- ruolo dell'env RL di traiettoria;
- invarianti da non violare.

### Smoke test AB06 PI

Creato `validation/rl_env_smoke_ab06_pi.py`, suite di validazione corta che
usa esplicitamente il setup AB06 PI e non dipende da `.simulator_last_setup.json`.

La suite copre:

- import smoke;
- preflight dei file dichiarati nel setup XML;
- costruzione env, reset, singolo step con azione zero;
- reset reuse nello stesso processo;
- action sanity con `0.0`, `+0.1`, `-0.1`;
- recorded-output smoke;
- ispezione separata di `.sto` gia presenti in `results/`.

## Problemi incontrati e risolti

### Layout dati AB06

Il setup AB06 cercava i dati in:

```text
models/AB06_SEASEA_Threadmill/data/
```

ma l'archivio era stato estratto con un livello in piu':

```text
models/AB06_SEASEA_Threadmill/data/data/
```

Sono stati copiati al livello atteso:

```text
IK_results_AB06_SEASEA.mot
ExternalForces.xml
CMC_Actuators.xml
AB06_SEASEA_GRF_FullSpan.mot
```

Il `.mot` GRF e necessario perche `ExternalForces.xml` lo referenzia come
`datafile`.

### Stage OpenSim in `reset_to_time`

Dopo la correzione dei path dati, il reset dell'env falliva perche
`SimulationRunner.reset_to_time()` chiamava `model.setControls()` quando lo
`State` OpenSim era ancora a stage `Instance`.

Correzione:

- dopo `_set_state()` e `_init_muscle_states()`, il runner chiama
  `model.realizeVelocity(state)` prima di `model.setControls()`;
- stessa protezione aggiunta nel fallback legacy Euler prima di riapplicare i
  controlli dopo l'aggiornamento dello stato.

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

Esito:

```text
env_import_ok
```

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

Recorded-output smoke:

```powershell
C:\Users\tomma\anaconda3\Scripts\conda.exe run -n envCMC-like python validation\rl_env_smoke_ab06_pi.py --recorded-output
```

Esito: passato. Output salvati in:

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
valori finiti. In totale sono presenti 446 file `sim_output_*.sto`.

## Stato file

File modificati/creati rilevanti:

```text
simulation_runner.py
osim_trj_cmc_like.py
validation/rl_env_smoke_ab06_pi.py
LLM_SIMULATOR_OVERVIEW.md
reports/user/2026-05-29_env_rl_traiettorie_cmc_like_e_overview_llm.md
reports/user/2026-05-29_validazione_env_rl_ab06_pi.md
reports/daily/2026-05-29_daily-report.md
```

Stato Git osservato prima della chiusura:

```text
 M simulation_runner.py
?? LLM_SIMULATOR_OVERVIEW.md
?? osim_trj_cmc_like.py
?? reports/user/2026-05-29_env_rl_traiettorie_cmc_like_e_overview_llm.md
?? reports/user/2026-05-29_validazione_env_rl_ab06_pi.md
?? validation/rl_env_smoke_ab06_pi.py
```

I file AB06 copiati in `models/AB06_SEASEA_Threadmill/data/` non comparivano
nello stato Git, probabilmente per regole di ignore sui dati/modelli.

## Note operative

Durante gli smoke test compare talvolta:

```text
QP did not converge. Using bounded least-squares fallback.
```

Lo smoke resta valido: osservazioni e reward sono finiti e `truncated=False`.
Il warning va pero' monitorato prima di rollout lunghi o training PPO, perche
potrebbe indicare che alcuni target biologici iniziali AB06 sono difficili da
soddisfare con i bounds correnti di muscoli/reserve.

## TODO chiusi oggi

- Smoke dinamica breve dell'env RL: chiusa con episodio da 2 ms su AB06 PI.
- Verifica caricamento modello/plugin/GRF AB06: chiusa.
- Verifica `env.reset()` e `env.step()` singolo: chiusa.
- Verifica reset riusabile di base con `rebuild_model_on_reset=False`: chiusa
  su tre reset consecutivi nello stesso processo.
- Recorded-output smoke: chiuso, `.sto` generati e riletti.
- Preflight setup AB06 PI: chiuso dopo correzione layout dati.

## TODO aperti e propagati

### Aperti dal lavoro RL di oggi

- Misurare anche `reset()` con `rebuild_model_on_reset=True` per quantificare
  il costo relativo rispetto al reset riusabile.
- Validare rollout piu lunghi dell'env RL: prima 20-50 ms, poi 0.5-1 s, poi
  finestre piu vicine al training PPO.
- Verificare che il reset riusabile non lasci stato residuo nel plugin SEA,
  negli ExternalLoads o nei buffer del controller su rollout multipli lunghi.
- Tarare i pesi del reward dopo i primi rollout reali.
- Monitorare il fallback bounded least-squares dello static optimizer e capire
  se richiede tuning dei target biologici o dei bounds.
- Decidere se includere `LLM_SIMULATOR_OVERVIEW.md` in `CONTEXT.md` o nel
  flusso `start_day` come riferimento stabile per agenti futuri.

### Propagati dal 2026-05-20

- Decidere se promuovere configurazione D o restare a C per il motor driver.
- Sweep `Kp_knee_motor` su valori intermedi tra 3.9 e 18 mantenendo ankle
  morning best.
- Generare plot D vs C prima di eventuale promozione.
- Validare la diagnosi di coupling knee-ankle isolando la dinamica knee dal
  feedback ankle.
- Notch a 28 Hz sul feedback knee resta opzione aperta se si vuole evitare una
  modifica piu invasiva del motor driver.
- Cleanup modelli sperimentali, includendo `slow_inner_pd_1405` e
  `pi_asym_knee1405`.

### Propagati dal 2026-05-19/18

- Windows: build/copia DLL plugin PI completa e documentata. Oggi e stato
  verificato il load dell'attuale plugin su Windows e la lettura di `Ki` /
  `integral_torque_limit`, ma non e stata rifatta una build DLL.
- Secondo pass knee dello sweep locale.
- Confronto consolidato finale tra configurazioni storiche: PD, PI, cascade,
  retune PI, zeta07, pi-tuned, J_eff, Opzione D, slow inner, asym.
- Pulizia artefatti sweep `_cascade_local_gain_sweep_20260517_233607` e
  `_cascade_local_gain_sweep_20260517_234151`.
