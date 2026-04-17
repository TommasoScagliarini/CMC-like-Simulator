# Runtime SEA Plugin Validato

Data: 2026-04-16

## Problema

Durante il passaggio al path `sea_forward_mode = "plugin"` su Windows erano emersi due problemi distinti:

- errori nativi Windows durante il caricamento/esecuzione del plugin, inclusi messaggi tipo `0xc06d007f` e access violation;
- risultati precedenti "troppo belli", perche il path legacy forzava algebricamente `motor_angle = q + tau_ref/K` e `motor_speed = qdot`, rendendo `tau_spring` praticamente uguale a `tau_ref`.

Il vincolo principale era non introdurre scorciatoie: la dinamica SEA doveva restare quella del plugin C++.

## Strategia

La strategia adottata e stata conservativa:

1. isolare il sorgente C++ in una copia agent;
2. ripulire il sorgente originale `SEA_plugin_core` dalle modifiche precedenti;
3. compilare la DLL solo dalla copia `SEA_plugin_core-agent`;
4. evitare le chiamate OpenSim piu rischiose per Windows (`realizeAcceleration()` e lettura diretta di state derivatives) nel runner Python;
5. esporre dal plugin C++ output diagnostici delle derivate motore;
6. integrare in Python gli stati SEA usando quelle derivate, senza cambiare la legge fisica del SEA.

## Soluzione

### Isolamento plugin

E stata creata la cartella:

```text
C:\Users\tomma\Desktop\Opensim OMNIBUS\SEA_plugin_core-agent
```

Da quel momento il lavoro C++ e la compilazione della DLL sono stati fatti solo da quella copia.

Il sorgente originale:

```text
C:\Users\tomma\Desktop\Opensim OMNIBUS\SEA_plugin_core
```

e stato verificato pulito per:

```text
SeriesElasticActuator.cpp
SeriesElasticActuator.h
```

con:

```powershell
git -C "..\SEA_plugin_core" status --short -- SeriesElasticActuator.cpp SeriesElasticActuator.h
```

Risultato: nessun output.

### Plugin agent

Nella copia agent il plugin espone output C++ per:

```text
tau_input
motor_angle_dot
motor_speed_dot
```

Questi output sono calcolati dalla stessa dinamica gia presente nel plugin:

```text
d(theta_m)/dt = omega_m
d(omega_m)/dt = (tau_input - tau_spring - Bm*omega_m) / Jm
```

Non e stata aggiunta una dinamica SEA alternativa in Python.

### DLL

La DLL e stata compilata dalla copia agent:

```text
C:\Users\tomma\Desktop\Opensim OMNIBUS\SEA_plugin_core-agent\build_agent\Release\SEA_Plugin_BlackBox_mCMC_impedence.dll
```

ed e stata copiata in:

```text
C:\Users\tomma\Desktop\Opensim OMNIBUS\CMC-like-Simulator - Claude\plugins\SEA_Plugin_BlackBox_mCMC_impedence.dll
```

La DLL precedente e stata salvata con backup in `plugins/`.

### Loader Windows

In `model_loader.py` e stato aggiunto un setup esplicito del path DLL Windows:

- inferenza dell'installazione OpenSim usata dal modulo Python `opensim`;
- aggiunta di `C:\OpenSim-mCMC\bin` con `os.add_dll_directory`;
- preload delle DLL SimTK/OpenSim dallo stesso albero;
- stampa del path scelto:

```text
[ModelLoader] OpenSim DLL path : C:\OpenSim-mCMC\bin
```

Questo riduce il rischio che Windows carichi dipendenze da un'altra installazione OpenSim presente nel `PATH`.

### Runner

In `simulation_runner.py` il path plugin ora:

- calcola `udot` una volta per il passo principale con `compute_udot_bypass`;
- registra i risultati al tempo corrente;
- integra le coordinate con quell'`udot`;
- sub-steppa solo gli stati motore SEA;
- legge le derivate motore dagli output C++ del plugin a Stage `Dynamics`;
- continua a evitare `realizeAcceleration()`.

Il punto importante e che non viene piu ricalcolato `udot` biologico dentro i substep SEA con controlli fissi non riottimizzati. Questo aveva prodotto instabilita artificiale, soprattutto su coordinate leggere come `mtp_angle_r`.

### Configurazione stabile

In `config.py` il default e stato portato a:

```python
dt = 0.001
sea_forward_mode = "plugin"
sea_motor_substeps = 5
sea_motor_max_substeps = 80
save_sea_derivatives = True
```

Motivo: con `dt=0.01` la dinamica plugin SEA e troppo rigida e puo causare saturazione/instabilita. Con `dt=0.001` il path validato resta stabile nei test brevi e medi.

In `main.py` sono state aggiunte opzioni:

```powershell
python main.py --sea-forward-mode plugin
python main.py --sea-forward-mode ideal_torque
python main.py --sea-motor-substeps 5
python main.py --sea-motor-max-substeps 80
python main.py --validate
```

## File Modificati

Nel repository simulatore:

- `config.py`
- `main.py`
- `model_loader.py`
- `simulation_runner.py`
- `validation/validate_sim_results.py`
- `validation/_debug_one_step.py`
- `validation/_probe_mobility.py`

Fuori dal repository simulatore, nella copia agent:

- `C:\Users\tomma\Desktop\Opensim OMNIBUS\SEA_plugin_core-agent\SeriesElasticActuator.cpp`
- `C:\Users\tomma\Desktop\Opensim OMNIBUS\SEA_plugin_core-agent\SeriesElasticActuator.h`

Output/binari aggiornati:

- `plugins/SEA_Plugin_BlackBox_mCMC_impedence.dll`
- backup DLL in `plugins/SEA_Plugin_BlackBox_mCMC_impedence.dll.bak_*`

## Verifiche Eseguite

### Compilazione Python

```powershell
python -m py_compile config.py main.py simulation_runner.py model_loader.py output.py validation\validate_sim_results.py
```

Esito: OK.

### Probe mapping mobilita

Eseguito:

```powershell
conda run -n envCMC-like python -X faulthandler validation\_probe_mobility.py
```

Esito: il mapping coordinate -> `U/UDot` e risultato coerente. Quindi le accelerazioni non venivano applicate alla coordinata sbagliata.

### Smoke breve

Comando:

```powershell
conda run -n envCMC-like python -X faulthandler main.py --t-start 4.26 --t-end 4.40 --output-dir results\_safe_agent_default_smoke --validate
```

Esito validator:

```text
Summary: PASS=16, WARN=20, FAIL=0
```

Status run:

```text
status=complete
dt=0.001
sea_forward_mode=plugin
sea_motor_substeps=5
sea_motor_max_substeps=80
```

### Smoke medio

Comando:

```powershell
conda run -n envCMC-like python -X faulthandler main.py --t-start 4.26 --t-end 4.80 --output-dir results\_safe_agent_medium_smoke --validate
```

Esito validator:

```text
Summary: PASS=18, WARN=18, FAIL=0
```

Status run:

```text
status=complete
t=4.8
step=540
dt=0.001
sea_forward_mode=plugin
sea_motor_substeps=5
sea_motor_max_substeps=80
```

## Risultato

Il problema nativo Windows non si e ripresentato nei test eseguiti.

Il path SEA plugin-driven ora supera i controlli anti-tautologia:

- `tau_ref - tau_spring` non e nullo a precisione macchina;
- `motor_angle` non coincide con `q + tau_ref/K`;
- `motor_speed` non coincide con `qdot`;
- le derivate `motor_angle_dot` e `motor_speed_dot` sono finite;
- i controlli SEA non saturano nei test validati.

## Note Aperte

Restano WARN del validator, non FAIL:

- molti tracking error sono molto stretti: il simulatore resta controller-following, non predittivo puro;
- le reserve protesiche sono nulle, accettabile solo perche il path SEA e stato validato;
- possono comparire fallback SLSQP verso bounded least-squares in alcuni frame.

## Prossimo Passo Consigliato

Eseguire il run completo con:

```powershell
python main.py --plot --validate
```

Con `dt=0.001` sara piu lento del vecchio default, ma e coerente con la dinamica SEA plugin-driven validata.
