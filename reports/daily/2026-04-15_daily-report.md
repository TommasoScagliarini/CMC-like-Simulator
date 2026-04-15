# Daily report - 2026-04-15

## Sintesi

La giornata e stata dedicata a rendere il simulatore robusto su Windows senza
rompere la versione macOS, e ad espandere la pipeline di output/plot per
produrre grafici diagnostici completi su SEA, reserve, potenza, gait cycle e
dati sani.

Le modifiche sono state implementate senza cambiare la semantica del plugin C++
SEA e senza alterare la logica numerica principale del simulatore. I nuovi
segnali vengono registrati come output diagnostici a partire da stati, controlli
e file gia presenti.

Report utente consolidati:

- `reports/user/2026-04-15_fix_cross_platform_windows_macos.md`
- `reports/user/2026-04-15_output_plotter_gait_power_healthy.md`

## Fix cross-platform Windows/macOS

Su Windows la simulazione falliva durante l'inizializzazione del
`SimulationRunner` con:

```text
Property 'Kp' not present in Object SEA_Knee
```

La causa era una differenza nei binding OpenSim/Python: su questa build Windows
le proprieta custom del plugin `SeriesElasticActuator` non erano accessibili via
`Object.getPropertyByName()`, anche se il plugin veniva caricato correttamente.

La correzione ha spostato la lettura delle proprieta custom SEA sul file `.osim`,
che e la stessa sorgente usata dal plugin:

- `stiffness` -> `K`
- `Kp`
- `Kd`
- `motor_damping` -> `Bm`
- `motor_inertia` -> `Jm`
- `optimal_force` -> `F_opt`
- `Impedence` -> `impedance`

Questi valori sono ora salvati in `SimulationContext.sea_props` e usati da
`SimulationRunner` e `OutputRecorder` per la diagnostica SEA.

Sono stati anche corretti i problemi di geometria in `visualize.py` e nel model
loader: il progetto registra piu cartelle `Geometry` possibili, includendo
cartelle locali, directory del modello, variabili ambiente OpenSim, path standard
Windows e path standard macOS.

Le stampe runtime principali sono state rese ASCII per evitare problemi di
encoding sulla console Windows.

## Output diagnostici aggiunti

Sono stati aggiunti nuovi output in `results/`:

- `sim_output_reserve_controls.sto`
- `sim_output_reserve_torques.sto`
- `sim_output_sea_states.sto`
- `sim_output_power.sto`
- `sim_output_gait_events.csv`

Gli stati motore SEA usano la nomenclatura:

- `SEA_Knee_motor_angle`
- `SEA_Knee_motor_speed`
- `SEA_Ankle_motor_angle`
- `SEA_Ankle_motor_speed`

Le potenze usano prefisso SEA:

- `SEA_Knee_joint_power`
- `SEA_Knee_motor_power`
- `SEA_Ankle_joint_power`
- `SEA_Ankle_motor_power`

I reserve protesici vengono registrati solo come diagnostica e restano fuori dal
QP, preservando la logica SEA-only sul lato protesico.

## Gait cycle da GRF

Il simulatore ora legge il `.mot` referenziato da `Externall_Loads.xml` e usa la
componente verticale della GRF con soglia default `20 N` per rilevare i crossing
rising heel-strike.

Gli eventi completi heel-strike -> heel-strike vengono salvati in:

```text
results/sim_output_gait_events.csv
```

con colonne:

```text
side,cycle_start,cycle_end,source_force,threshold_n
```

Sono salvati eventi sia `left` sia `right`; il plotter usa `left` come default
per i grafici dei giunti protesici.

## Plotter

E stato creato ed esteso `plot/plotter.py`. Il comando base e:

```powershell
python plot/plotter.py
```

Il plotter usa `results` come directory default, quindi `--results-dir results`
non e obbligatorio.

Sono generati cinque grafici:

1. `01_time_sea_control_reserve.png`
2. `02_time_joint_motor_states.png`
3. `03_gaitcycle_torque_angle_power.png`
4. `04_gaitcycle_joint_velocity_power.png`
5. `05_time_tau_input_tracking_error.png`

Il quinto grafico include:

- `tau_input`
- errore `tau_ref - tau_spring`
- errore `tau_ref - tau_input`

Le cartelle di output vengono create sotto `plot/` nel formato:

```text
DD_MM_YYYY - N
```

dove `N` incrementa durante la giornata.

## Convenzioni grafiche e dati sani

Il plotter mantiene ankle sempre a sinistra e knee sempre a destra.

Per aderire alla convenzione rappresentativa sagittale:

- il knee viene ribaltato per angle, velocity, motor angle, motor speed, torque,
  reserve torque e control input;
- la power non viene ribaltata, perche rappresenta uno scalare energetico.

I dati sani vengono caricati automaticamente da:

- `data/health`
- `data/healthy`

Nel repository corrente sono presenti:

- `data/healthy/3DGaitModel2392_Kinematics_q.sto`
- `data/healthy/3DGaitModel2392_Actuation_force.sto`

Il plotter usa i dati sani in arancione `#ffa500` con linea dashed. La healthy
velocity viene derivata da `Kinematics_q` con differenze finite; la healthy
power viene derivata come `healthy torque * healthy velocity`.

In chiusura giornata e stato rimosso un residuo testuale del plotter:

- la console non stampa piu `Missing channels / WIP inputs`;
- ora stampa `Missing channels / unavailable inputs`;
- il commento iniziale di `plotter.py` e stato aggiornato per riflettere il
  supporto reale ai dati sani.

## Integrazione CLI

`main.py` supporta ora:

```powershell
python main.py --plot
```

Il flag esegue la simulazione e, se completata con successo, lancia il plotter
sui risultati appena generati.

Comandi disponibili:

```powershell
python main.py
python main.py --plot
python plot/plotter.py
```

## File modificati

- `config.py`
- `model_loader.py`
- `output.py`
- `simulation_runner.py`
- `main.py`
- `kinematics_interpolator.py`
- `visualize.py`
- `plot/plotter.py`
- `reports/user/2026-04-15_fix_cross_platform_windows_macos.md`
- `reports/user/2026-04-15_output_plotter_gait_power_healthy.md`

## Verifiche eseguite

Compilazione/sintassi:

```powershell
python -m py_compile config.py model_loader.py simulation_runner.py visualize.py output.py prosthesis_controller.py main.py kinematics_interpolator.py
python -m py_compile model_loader.py simulation_runner.py output.py plot/plotter.py config.py
python -m py_compile main.py
python -m py_compile plot/plotter.py
```

Check diff:

```powershell
git diff --check -- model_loader.py simulation_runner.py visualize.py main.py kinematics_interpolator.py
```

Smoke run Windows:

```powershell
$env:PYTHONPATH = 'C:\OpenSim-mCMC\sdk\Python'
$env:PATH = 'C:\OpenSim-mCMC\bin;C:\OpenSim-mCMC\sdk\Simbody\bin;' + $env:PATH
& 'C:\Users\tomma\AppData\Local\Programs\Python\Python312\python.exe' main.py --t-start 4.26 --t-end 4.28 --output-dir results\_codex_smoke
```

Risultato smoke run:

- plugin caricato correttamente;
- modello caricato senza warning sui `.vtp`;
- proprieta SEA lette da `.osim`;
- `SimulationRunner` costruito senza errore `Property 'Kp' not present`;
- 2 step completati;
- output salvati in `results\_codex_smoke`.

Verifica plotter:

```powershell
python plot/plotter.py
```

Ultima run verificata:

```text
Healthy overlay loaded from: ...\data\healthy
No missing channels.
Plots saved to: ...\plot\15_04_2026 - 15
```

Il file:

```text
plot\15_04_2026 - 15\missing_channels.txt
```

contiene:

```text
No missing channels.
```

Verifica gait cycle su GRF:

- 5 cicli completi `left`;
- 5 cicli completi `right`;
- soglia usata: `20 N`.

## Note e stato finale

La compatibilita macOS non e stata eseguita fisicamente da questa macchina
Windows, ma le modifiche sono state costruite per essere cross-platform:

- i path Windows sono protetti da `os.name == "nt"`;
- i path macOS OpenSim.app restano supportati;
- la lettura `.osim` e indipendente dal sistema operativo;
- il nome plugin resta senza estensione, con risoluzione per `.dll`, `.dylib` e
  `.so`.

Nel working tree sono presenti anche file generati (`__pycache__`, output in
`results/`, cartelle PNG sotto `plot/`, log OpenSim). Sono artefatti di run e
verifiche, non modifiche logiche richieste al codice.
