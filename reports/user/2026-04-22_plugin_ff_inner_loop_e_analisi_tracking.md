# 2026-04-22 - Plugin ff inner loop e analisi tracking

## Problema

Nel plugin C++ `SeriesElasticActuator`, il ramo non-impedance implementava la
legge:

```cpp
tau_input = Kp * (tau_ref - tau_spring) - Kd * omega_m;
```

Questa forma introduce un errore statico strutturale nel tracking di coppia:
a regime, con velocita nulla, `tau_spring` non converge a `tau_ref`, ma a una
frazione di esso.

L'obiettivo della sessione era:

- analizzare il sorgente del plugin e verificare la correttezza della dinamica
  SEA e della legge di controllo interna;
- introdurre una variante con termine feedforward realistico sul comando
  motore;
- compilare una DLL separata con suffisso `ff`;
- copiarla nel simulatore Python;
- aggiornare il simulatore per usare il nuovo plugin;
- verificare se il tracking migliorasse davvero nei risultati finali.

## Strategia

La strategia adottata e stata conservativa:

1. leggere `SeriesElasticActuator.cpp/.h` e separare:
   - dinamica meccanica del SEA;
   - legge di `tau_input`;
   - forza applicata al giunto in `computeActuation()`;
2. evitare refactor ampi del plugin e introdurre solo la correzione minima
   utile nel ramo non-impedance;
3. costruire una nuova DLL con nome distinto, senza sovrascrivere la versione
   gia in uso;
4. verificare il caricamento del plugin `ff` con una smoke run breve;
5. analizzare i risultati dei run completi confrontando:
   - caso `3 ms / K_ankle=700 / no-ff`;
   - caso `3 ms / K_ankle=700 / ff`;
   - caso `1 ms / K_ankle=700 / no-ff`;
   - caso `1 ms / K_ankle=700 / ff`.

## Analisi del plugin

### Dinamica SEA

La dinamica meccanica del plugin e coerente con un SEA classico:

```text
tau_spring = K * (theta_m - theta_joint)
d(theta_m)/dt = omega_m
d(omega_m)/dt = (tau_input - tau_spring - Bm * omega_m) / Jm
```

Dal punto di vista del plant, la struttura e corretta.

### Criticita della legge di controllo originale

Nel ramo non-impedance, il plugin usava:

```cpp
tau_input = Kp * (tau_ref - tau_spring) - Kd * omega_m;
```

A regime, con `omega_m = 0`, si ottiene:

```text
tau_spring = Kp / (Kp + 1) * tau_ref
```

quindi il tracciamento di coppia ha bias statico.

Per i parametri attuali del modello:

- knee: `Kp = 3.9`  -> circa `0.796 * tau_ref`
- ankle: `Kp = 8.8` -> circa `0.898 * tau_ref`

Questa e la ragione per cui il plugin e stato giudicato corretto come dinamica
meccanica, ma non pienamente corretto come inner torque driver trasparente.

## Soluzione implementata

Nel plugin `SeriesElasticActuator.cpp` e stato introdotto il termine
feedforward direttamente in `tau_input`:

```cpp
tau_input = tau_ref + Kp * (tau_ref - tau_spring) - Kd * omega_m;
```

Questa modifica:

- elimina il bias statico del ramo non-impedance;
- mantiene una struttura fisicamente realistica per un SEA;
- non trasforma il giunto in un attuatore ideale;
- e molto meno invasiva di una riscrittura completa del controller interno.

E stata poi costruita una DLL separata:

```text
SEA_Plugin_BlackBox_mCMC_impedence_ff.dll
```

e il simulatore Python e stato aggiornato per caricarla via `config.py`.

## File modificati

Nel repository plugin:

- `C:\Users\tomma\Desktop\Opensim OMNIBUS\SEA_plugin_core-agent\SeriesElasticActuator.cpp`
  - aggiunto il termine `tau_ref` in feedforward nella legge di `tau_input`.
- `C:\Users\tomma\Desktop\Opensim OMNIBUS\SEA_plugin_core-agent\CMakeLists.txt`
  - cambiato il target plugin in `SEA_Plugin_BlackBox_mCMC_impedence_ff`.

Nel repository simulatore:

- `config.py`
  - `plugin_name = "plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff"`

Binari aggiornati:

- `plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff.dll`

## Verifiche eseguite

### Compilazione plugin

Configurazione CMake:

```powershell
cmake -S . -B build_agent -G "Visual Studio 17 2022" -A x64 -DOPENSIM_INSTALL_DIR=C:/OpenSim-mCMC
```

Build target:

```powershell
cmake --build build_agent --config Release --target SEA_Plugin_BlackBox_mCMC_impedence_ff
```

Esito: build completata con successo.

### Copia DLL nel simulatore

La DLL compilata e stata copiata in:

```text
plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff.dll
```

### Smoke run del plugin ff

Comando:

```powershell
python main.py --t-start 4.26 --t-end 4.28 --output-dir results\_smoke_ff_plugin
```

Esito:

- plugin `ff` caricato correttamente;
- simulazione breve completata;
- output salvati in `results/_smoke_ff_plugin`.

## Analisi risultati

### Caso `3 ms / K_ankle = 700 / ff`

Run analizzata tramite `results/` e plot:

- `plot/22_04_2026 - 6`

Metriche principali:

- `knee_tracking_rms_deg = 2.665`
- `ankle_tracking_rms_deg = 4.135`
- `score_total = 13.179`

Confronto col caso `3 ms / K_ankle = 700 / no-ff`:

- knee RMS: `3.369 -> 2.665`
- ankle RMS: `4.645 -> 4.135`
- max `|u|` piu basso
- `max_tau_input_raw_abs`: `75.38 Nm`

Pero i side-effect distali restano quasi invariati:

- `ankle_r_tracking_rms_deg ~ 10.33`
- `mtp_r_tracking_rms_deg ~ 40.03`

Per questo il miglioramento numerico del tracking non produceva un grande salto
visivo nel grafico 4.

### Caso `1 ms / K_ankle = 700 / ff`

Ultima run disponibile in `results/` al momento del report:

- `status = complete`
- `T_control = 0.001 s`
- plot in `plot/22_04_2026 - 7`

Metriche principali:

- `score_total = 9.608`
- `knee_tracking_rms_deg = 2.640`
- `ankle_tracking_rms_deg = 4.086`
- `knee_tracking_max_deg = 8.205`
- `ankle_tracking_max_deg = 9.345`
- `ankle_r_tracking_rms_deg = 3.235`
- `mtp_r_tracking_rms_deg = 11.781`
- `max_tau_input_raw_abs = 20.869 Nm`

Confronto col caso `1 ms / K_ankle = 700 / no-ff`:

- `score_total: 11.111 -> 9.608`
- `knee RMS: 3.338 -> 2.640`
- `ankle RMS: 4.587 -> 4.086`
- `knee max error: 10.364 -> 8.205`
- `ankle max error: 10.594 -> 9.345`
- `max_tau_input_raw_abs: 70.350 -> 20.869 Nm`
- `knee max|u|: 0.363 -> 0.288`
- `ankle max|u|: 0.310 -> 0.274`

I side-effect distali invece restano quasi uguali:

- `ankle_r_tracking_rms_deg: 3.232 -> 3.235`
- `mtp_r_tracking_rms_deg: 11.776 -> 11.781`

## Conclusioni

1. La dinamica meccanica del SEA nel plugin e sostanzialmente corretta.
2. La legge originale di `tau_input` nel ramo non-impedance introduceva un
   errore statico strutturale.
3. L'aggiunta del termine feedforward in `tau_input` e stata implementata con
   successo in una variante separata del plugin (`ff`).
4. Il plugin `ff` migliora davvero il tracking protesico verso l'IK:
   - soprattutto sul knee;
   - in misura piu moderata sull'ankle.
5. Il caso migliore tra quelli analizzati nella sessione e:

```text
T_control = 1 ms
SEA_Ankle stiffness = 700
plugin = SEA_Plugin_BlackBox_mCMC_impedence_ff
```

6. Nonostante il miglioramento del tracking protesico, l'aspetto dei grafici
   rispetto all'healthy cambia meno del previsto perche i side-effect distali
   (`ankle_r`, `mtp_r`) non migliorano in modo sostanziale.

## Stato finale

Il simulatore e configurato per usare il plugin:

```text
plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff
```

La nuova DLL e compilata, copiata e caricabile correttamente dal simulatore.
