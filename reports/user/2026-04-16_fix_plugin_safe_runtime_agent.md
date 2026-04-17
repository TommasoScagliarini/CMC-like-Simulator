# Fix Runtime Sicuro SEA Plugin-Driven

Data: 2026-04-16

## Obiettivo

Rendere il path `sea_forward_mode = "plugin"` eseguibile su Windows senza scorciatoie sulla dinamica SEA:

- niente accessi diretti allo Stage `Acceleration` che avevano prodotto errori nativi Windows;
- dinamica `motor_angle` / `motor_speed` derivata dal plugin C++;
- sorgente originale `SEA_plugin_core` lasciato pulito;
- lavoro C++ svolto solo nella copia `SEA_plugin_core-agent`.

## Isolamento Del Sorgente Plugin

E stata creata la copia:

```text
C:\Users\tomma\Desktop\Opensim OMNIBUS\SEA_plugin_core-agent
```

Il sorgente originale:

```text
C:\Users\tomma\Desktop\Opensim OMNIBUS\SEA_plugin_core
```

e stato ripulito per i due file SEA principali:

```text
SeriesElasticActuator.cpp
SeriesElasticActuator.h
```

Verifica finale:

```powershell
git -C "..\SEA_plugin_core" status --short -- SeriesElasticActuator.cpp SeriesElasticActuator.h
```

Risultato: nessun output, quindi quei due file sono puliti rispetto al repository originale.

## Modifiche Nella Copia Agent

Nella copia `SEA_plugin_core-agent` il plugin espone ora gli output diagnostici:

```text
motor_angle_dot
motor_speed_dot
tau_input
```

Questi output leggono la stessa dinamica gia presente nel plugin C++ e permettono al Python runner di integrare gli stati SEA senza chiamare:

```text
model.realizeAcceleration(state)
model.getStateVariableDerivativeValue(...)
```

Queste chiamate restano evitate perche erano la zona collegata agli errori nativi Windows.

## Build E DLL

La DLL e stata compilata dalla copia agent con Visual Studio x64:

```powershell
cmake -S . -B build_agent -G "Visual Studio 17 2022" -A x64 -DOPENSIM_INSTALL_DIR=C:/OpenSim-mCMC
cmake --build build_agent --config Release --target SEA_Plugin_BlackBox_mCMC_impedence
```

Output prodotto:

```text
C:\Users\tomma\Desktop\Opensim OMNIBUS\SEA_plugin_core-agent\build_agent\Release\SEA_Plugin_BlackBox_mCMC_impedence.dll
```

La DLL corrente del simulatore e stata aggiornata:

```text
C:\Users\tomma\Desktop\Opensim OMNIBUS\CMC-like-Simulator - Claude\plugins\SEA_Plugin_BlackBox_mCMC_impedence.dll
```

Backup presenti in `plugins/`, incluso:

```text
SEA_Plugin_BlackBox_mCMC_impedence.dll.bak_20260416_115302
```

## Fix Windows Loader

In `model_loader.py` e stato aggiunto un setup esplicito del path DLL Windows:

- inferisce l'installazione OpenSim usata dal modulo Python `opensim`;
- aggiunge `C:\OpenSim-mCMC\bin` con `os.add_dll_directory`;
- pre-carica le DLL SimTK/OpenSim dallo stesso albero;
- evita che Windows risolva dipendenze da un'altra installazione OpenSim in `PATH`.

Questo e pensato per ridurre errori loader tipo:

```text
0xc06d007f
```

senza modificare la dinamica.

## Fix Integrazione Numerica

In `simulation_runner.py` il path plugin ora:

1. calcola `udot` una volta per il passo principale con `compute_udot_bypass`;
2. registra gli output;
3. avanza le coordinate con quel solo `udot` di passo, come nello schema CMC-like semi-esplicito;
4. sub-steppa solo gli stati motore SEA usando gli output del plugin C++:

```text
SEA_Knee_motor_angle_dot
SEA_Knee_motor_speed_dot
SEA_Ankle_motor_angle_dot
SEA_Ankle_motor_speed_dot
```

Correzione importante: prima, durante i substep SEA, veniva ricalcolato anche `udot` biologico con controlli fissi non riottimizzati. Questo poteva produrre drift artificiale su coordinate leggere come `mtp_angle_r`.

La nuova integrazione non cambia la legge SEA: cambia solo come viene avanzato numericamente lo stato tra due frame di controllo.

## Default Stabilizzato

In `config.py`:

```python
dt = 0.001
sea_motor_substeps = 5
sea_motor_max_substeps = 80
```

Motivo: con `dt=0.01` la dinamica plugin e troppo rigida e il sistema va in saturazione/instabilita; con `dt=0.001` il validator passa senza FAIL nei test brevi e medi.

In `main.py` sono state aggiunte opzioni CLI:

```powershell
python main.py --sea-motor-substeps 5
python main.py --sea-motor-max-substeps 80
```

Il runner stampa anche un warning se si forza:

```text
sea_forward_mode = plugin
dt > 0.001
```

## Validazione Eseguita

Compilazione Python:

```powershell
python -m py_compile config.py main.py simulation_runner.py model_loader.py output.py validation\validate_sim_results.py
```

Smoke breve:

```powershell
conda run -n envCMC-like python -X faulthandler main.py --t-start 4.26 --t-end 4.40 --output-dir results\_safe_agent_default_smoke --validate
```

Esito:

```text
Summary: PASS=16, WARN=20, FAIL=0
status=complete
dt=0.001
sea_forward_mode=plugin
sea_motor_substeps=5
```

Smoke medio:

```powershell
conda run -n envCMC-like python -X faulthandler main.py --t-start 4.26 --t-end 4.80 --output-dir results\_safe_agent_medium_smoke --validate
```

Esito:

```text
Summary: PASS=18, WARN=18, FAIL=0
status=complete
t=4.8
step=540
dt=0.001
sea_forward_mode=plugin
sea_motor_substeps=5
sea_motor_max_substeps=80
```

## Stato Attuale

Il problema nativo Windows non si e ripresentato nei test.

Il path plugin non e piu tautologico:

- `tau_ref - tau_spring` non e piu nullo a precisione macchina;
- `motor_angle` non coincide con `q + tau_ref/K`;
- `motor_speed` non coincide con `qdot`;
- le derivate SEA sono finite.

Restano WARN attesi dal validator:

- tracking molto stretto su alcune coordinate, quindi risultato ancora controller-following;
- reserve protesiche nulle, accettabile solo perche il path SEA plugin e ora validato;
- alcuni fallback SLSQP occasionali verso bounded least-squares.

## Prossimo Test Consigliato

Run completo:

```powershell
conda run -n envCMC-like python -X faulthandler main.py --plot --validate
```

Nota: con `dt=0.001` il run completo sara piu lento, ma e il default coerente con la dinamica SEA plugin-driven validata.
