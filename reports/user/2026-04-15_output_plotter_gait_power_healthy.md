# 2026-04-15 - Output, plotter, gait cycle e dati sani

## Problema

Il simulatore produceva solo una parte dei segnali necessari ai grafici:

- mancavano output espliciti per reserve actuators, stati motore SEA, potenze SEA e gait-cycle events;
- il plotter doveva annotare canali mancanti, ma poi supportare i nuovi output appena disponibili;
- i grafici dovevano rispettare la convenzione rappresentativa sagittale, con knee flexion positiva e ankle dorsiflexion positiva;
- i dati sani in `data/healthy` dovevano essere letti e sovrapposti ai grafici gait-cycle;
- serviva un comando unico `python main.py --plot` per simulare e generare i grafici.

## Soluzione

Sono stati aggiunti nuovi output al simulatore senza modificare la logica fisica o il reclutamento:

- `sim_output_reserve_controls.sto`
- `sim_output_reserve_torques.sto`
- `sim_output_sea_states.sto`
- `sim_output_power.sto`
- `sim_output_gait_events.csv`

Gli stati motore SEA sono salvati con nomenclatura SEA:

- `SEA_Knee_motor_angle`
- `SEA_Knee_motor_speed`
- `SEA_Ankle_motor_angle`
- `SEA_Ankle_motor_speed`

Le potenze SEA sono salvate con prefisso SEA:

- `SEA_Knee_joint_power`
- `SEA_Knee_motor_power`
- `SEA_Ankle_joint_power`
- `SEA_Ankle_motor_power`

Il plotter ora genera cinque figure:

1. `01_time_sea_control_reserve.png`
2. `02_time_joint_motor_states.png`
3. `03_gaitcycle_torque_angle_power.png`
4. `04_gaitcycle_joint_velocity_power.png`
5. `05_time_tau_input_tracking_error.png`

Il quinto grafico include:

- `tau_input`
- errore `tau_ref - tau_spring`
- errore `tau_ref - tau_input`

## Strategia

La modifica mantiene separati output diagnostici e dinamica del simulatore:

- i reserve protesici vengono letti e salvati come diagnostica, ma restano fuori dal QP;
- `OutputRecorder` registra i segnali già disponibili nello `State` e nel vettore `controls`;
- `tau_input`, `tau_spring`, motor state e power sono calcolati nello stesso punto in cui venivano già calcolate le coppie SEA;
- `tau_ref` nei grafici è ricostruito come `control * F_opt`, leggendo `F_opt` dal modello `.osim`;
- i gait cycles sono estratti dal `.mot` referenziato da `Externall_Loads.xml`, usando crossing rising della GRF verticale con soglia 20 N.

Per il plotter:

- ankle è sempre colonna sinistra, knee colonna destra;
- il knee viene ribaltato per angle, velocity, motor angle, motor speed, torque, reserve torque e control input;
- la power non viene ribaltata, perché è scalare energetico;
- i dati sani sono letti automaticamente da `data/health` o `data/healthy`;
- i sani sono plottati in arancione `#ffa500` con linea dashed;
- healthy velocity è derivata da `Kinematics_q`;
- healthy power è derivata come `healthy torque * healthy velocity`.

## File modificati

- `config.py`
  - aggiunti flag per reserve, SEA states, power, gait events e parametri GRF/plot.
- `model_loader.py`
  - aggiunti metadati per tutti i reserve actuator;
  - aggiunti metadati GRF e colonne verticali left/right;
  - mantenuta separazione tra reserve per SO e reserve diagnostici.
- `output.py`
  - aggiunti buffer e salvataggio dei nuovi `.sto`;
  - aggiunta estrazione e scrittura `sim_output_gait_events.csv`.
- `simulation_runner.py`
  - `OutputRecorder.record()` riceve il vettore `controls` finale.
- `main.py`
  - aggiunto flag `--plot`, che lancia il plotter dopo una simulazione riuscita.
- `plot/plotter.py`
  - creato e poi esteso con cinque grafici;
  - supporto nuovi output;
  - overlay dati sani;
  - convenzione rappresentativa knee/ankle;
  - report `missing_channels.txt`.

## Verifiche

Eseguite verifiche di sintassi:

```powershell
python -m py_compile model_loader.py simulation_runner.py output.py plot/plotter.py config.py
python -m py_compile main.py
python -m py_compile plot/plotter.py
```

Verificato il plotter:

```powershell
python plot/plotter.py
```

Output più recente generato:

```text
plot\15_04_2026 - 13
```

La cartella contiene tutti e cinque i PNG e `missing_channels.txt` riporta:

```text
No missing channels.
```

Verificata anche l'estrazione GRF su `Block1.mot`:

- 5 cicli completi `left`;
- 5 cicli completi `right`;
- soglia usata: 20 N.

## Note operative

Comandi disponibili:

```powershell
python main.py
python main.py --plot
python plot/plotter.py
```

`python plot/plotter.py` usa `results` come default, quindi `--results-dir results` non è necessario salvo voler plottare una cartella diversa.
