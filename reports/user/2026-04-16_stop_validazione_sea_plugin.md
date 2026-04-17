# Report Interruzione - Validazione SEA Plugin-Driven

Data: 2026-04-16  
Stato: lavoro interrotto su richiesta dell'utente, senza procedere oltre.

## Obiettivo Della Sessione

L'obiettivo era implementare il piano di fix minimo del plugin SEA C++ in `..\SEA_plugin_core`, limitando le modifiche a:

- `SeriesElasticActuator.cpp`;
- `SeriesElasticActuator.h`;

e preservando la build della DLL:

```text
SEA_Plugin_BlackBox_mCMC_impedence.dll
```

Il problema principale da correggere era il mismatch tra modello `.osim` e plugin:

- il modello usa `<Kp>` e `<Kd>`;
- il plugin, nella versione problematica, esponeva/leggeva `Kp_PD` e `Kd_PD`.

Il vincolo importante era non introdurre una dinamica motore SEA in Python: la dinamica del motore SEA deve restare nel plugin C++.

## Modifiche Python Gia Presenti Prima Del Fix C++

Prima di lavorare sul sorgente C++, era gia stata preparata una parte Python per validazione e modalita plugin-driven.

File toccati nel simulatore:

- `config.py`
- `main.py`
- `simulation_runner.py`
- `output.py`
- `validation/validate_sim_results.py`

### `config.py`

Aggiunto:

```python
sea_forward_mode: str = "plugin"
save_sea_derivatives: bool = True
```

Significato:

- `plugin`: modalita prevista per validazione reale, dove `motor_angle` e `motor_speed` dovrebbero evolvere dalle derivate prodotte dal plugin C++;
- `ideal_torque`: baseline legacy/diagnostica, cioe il vecchio comportamento algebrico in cui `motor_angle` viene forzato per produrre esattamente `tau_ref`.

### `main.py`

Aggiunte opzioni CLI:

```powershell
python main.py --validate
python main.py --sea-forward-mode plugin
python main.py --sea-forward-mode ideal_torque
```

Con `--validate`, `main.py` chiama:

```text
validation/validate_sim_results.py
```

e scrive un report in:

```text
reports/user/YYYY-MM-DD_validazione_simulatore.md
```

### `simulation_runner.py`

Aggiunta distinzione:

- `sea_forward_mode == "ideal_torque"`:
  - mantiene il vecchio `_update_sea_motor_state(state, tau_sea_cmd)`;
  - quindi forza ancora `motor_angle = q + tau_ref/K`;
  - serve solo come baseline diagnostica.

- `sea_forward_mode == "plugin"`:
  - prova a usare `model.realizeAcceleration(state)`;
  - legge `state.getUDot()`;
  - prova a leggere le derivate degli stati SEA con:

```python
model.getStateVariableDerivativeValue(state, "<path>/motor_angle")
model.getStateVariableDerivativeValue(state, "<path>/motor_speed")
```

  - integra `motor_angle` e `motor_speed` con quelle derivate.

### `output.py`

Aggiunto buffer e scrittura di:

```text
results/sim_output_sea_derivatives.sto
```

con colonne previste:

```text
SEA_Knee_motor_angle_dot
SEA_Knee_motor_speed_dot
SEA_Ankle_motor_angle_dot
SEA_Ankle_motor_speed_dot
```

Nota: questo file viene scritto solo se la simulazione arriva al salvataggio. Al momento il path `plugin` crasha prima.

### `validation/validate_sim_results.py`

Creato validator anti-tautologia. Legge `results/` e produce metriche su:

- `tau_ref - tau_spring`;
- `theta_m - (q + tau_ref/K)`;
- `omega_m - qdot`;
- reserve protesiche;
- tracking output vs IK;
- power;
- saturazione controlli SEA;
- presenza/finitezza delle derivate SEA.

Esecuzione baseline sui risultati esistenti:

```powershell
python validation/validate_sim_results.py --results-dir results --out reports/user/2026-04-16_validazione_simulatore.md
```

Esito atteso e ottenuto: il validator segnala FAIL/WARN per tautologia SEA, coerente col sospetto iniziale.

Metriche chiave osservate nei risultati correnti:

- `tau_ref - tau_spring` quasi nullo:
  - knee max circa `5e-7 Nm`;
  - ankle max circa `1.25e-6 Nm`.
- `theta_m - (q + tau_ref/K)` quasi nullo:
  - ordine `1e-8 rad`.
- `omega_m - qdot` esattamente zero.

Queste metriche confermano che i risultati attuali sono "troppo belli" perche il vecchio path algebrico forza il SEA a inseguire perfettamente il riferimento.

## Modifiche C++ Effettuate In `..\SEA_plugin_core`

File modificati:

```text
..\SEA_plugin_core\SeriesElasticActuator.cpp
..\SEA_plugin_core\SeriesElasticActuator.h
```

Non sono stati modificati intenzionalmente:

- `Plugin_interface.cpp`;
- `ForwardDynamics.cpp`;
- modello `.osim`;
- `CMakeLists.txt`;
- nome target `SEA_Plugin_BlackBox_mCMC_impedence`;
- nomi degli stati `motor_angle` e `motor_speed`.

Attenzione: `git -C ..\SEA_plugin_core status --short` mostra anche altri file gia sporchi o modificati nella cartella plugin core, inclusi `CMakeLists.txt`, `ForwardDynamics.cpp`, `SEATrackingController.h` e file di build. Non li ho modificati in questa fase e non li ho revertiti.

### 1. Proprietà `Kp` / `Kd`

Nel plugin sono state allineate le proprieta al modello `.osim`.

Ora in `SeriesElasticActuator.h` risultano:

```cpp
OpenSim_DECLARE_PROPERTY(Kp, double, "Inner-loop proportional gain");
OpenSim_DECLARE_PROPERTY(Kd, double, "Inner-loop derivative gain");
```

Nel `.cpp` risultano:

```cpp
constructProperty_Kp(1000.0);
constructProperty_Kd(20.0);
set_Kp(Kp);
set_Kd(Kd);
get_Kp();
get_Kd();
```

Non risultano piu riferimenti attivi a:

```text
Kp_PD
Kd_PD
```

Verifica eseguita:

```powershell
rg -n "Kp_PD|Kd_PD" ..\SEA_plugin_core\SeriesElasticActuator.cpp ..\SEA_plugin_core\SeriesElasticActuator.h
```

Risultato: nessun match.

### 2. `Kd_Imp`

Nel source corrente e presente anche `Kd_Imp`, coerente con il modello `.osim`, che contiene:

```xml
<Kd_Imp>5</Kd_Imp>
<Kd_Imp>10</Kd_Imp>
```

Non ho cambiato la semantica di `Kd_Imp`.

### 3. Guardie Numeriche

Sono state aggiunte guardie `std::isfinite` in:

- `computeStateVariableDerivatives()`;
- `getMotorTorque()`.

Guardie principali:

- `Jm` non finito o troppo piccolo -> clamp a `1e-9`;
- `Bm` non finito -> `0`;
- `K` non finito o quasi zero -> clamp a `1e-9`;
- `F_opt` non finito -> `0`;
- `Kp`, `Kd`, `Kd_Imp` non finiti -> `0`;
- `theta_joint` non finito -> `0`;
- `theta_m` non finito -> `theta_joint`;
- `omega_m` non finito -> `0`;
- controllo `u` non finito -> `0`;
- `tau_input` non finito -> `0`.

Queste guardie non cambiano la legge fisica nominale, ma evitano propagazione di NaN/Inf nel plugin.

### 4. Equazioni Mantenute

La legge del SEA e rimasta:

Non-impedance:

```cpp
tau_input = Kp * (tau_ref - tau_spring) - Kd * omega_m;
```

Impedance:

```cpp
tau_input = tau_ff
          + Kp * (theta_m_ref - theta_m)
          + Kd * (omega_m_ref - omega_m);
```

Saturazione:

```cpp
tau_input = clamp(tau_input, -500, 500);
```

### 5. `logTauInput()`

Durante le derivate, la chiamata a `logTauInput()` non viene usata.

Motivo: la scrittura file dentro il metodo delle derivate e rischiosa per stabilita e performance, specialmente quando OpenSim valuta derivate piu volte o con rollback. La funzione e rimasta nel sorgente, ma non viene chiamata dal path delle derivate.

## Build DLL

Build eseguita piu volte dalla cartella:

```text
C:\Users\tomma\Desktop\Opensim OMNIBUS\SEA_plugin_core
```

Comando:

```powershell
cmake --build build --config Release --target SEA_Plugin_BlackBox_mCMC_impedence
```

Esito: build completata senza errori.

DLL generata:

```text
..\SEA_plugin_core\build\Release\SEA_Plugin_BlackBox_mCMC_impedence.dll
```

DLL copiata nel simulatore:

```text
plugins\SEA_Plugin_BlackBox_mCMC_impedence.dll
```

Backup creati in `plugins/`:

```text
SEA_Plugin_BlackBox_mCMC_impedence.dll.bak_20260416_021317
SEA_Plugin_BlackBox_mCMC_impedence.dll.bak_20260416_021551
```

## Attenzione Importante Sulla DLL Corrente

Dopo alcuni test diagnostici, ho rimosso dal sorgente C++ la strumentazione temporanea `SEA_DERIV` / `SEA_TORQUE`.

Pero mi hai chiesto di interrompere prima che ricompilassi nuovamente la DLL pulita.

Quindi lo stato attuale e:

- sorgente `SeriesElasticActuator.cpp` pulito dai checkpoint temporanei;
- DLL in `plugins/SEA_Plugin_BlackBox_mCMC_impedence.dll` ancora corrispondente all'ultima build diagnostica;
- quella DLL potrebbe ancora stampare messaggi diagnostici `SEA_DERIV` / `SEA_TORQUE`.

Prima di riprendere i test veri bisogna fare una di queste due cose:

1. ricompilare il plugin dalla versione sorgente pulita e ricopiare la DLL in `plugins/`;
2. oppure ripristinare uno dei backup DLL se vogliamo tornare alla build precedente.

Comando da usare alla ripresa:

```powershell
cd "C:\Users\tomma\Desktop\Opensim OMNIBUS\SEA_plugin_core"
cmake --build build --config Release --target SEA_Plugin_BlackBox_mCMC_impedence
Copy-Item -LiteralPath "build\Release\SEA_Plugin_BlackBox_mCMC_impedence.dll" -Destination "..\CMC-like-Simulator - Claude\plugins\SEA_Plugin_BlackBox_mCMC_impedence.dll" -Force
```

## Test Eseguiti

### 1. Compilazione Python

Eseguito:

```powershell
python -m py_compile config.py simulation_runner.py output.py main.py validation\validate_sim_results.py
```

Esito: OK.

### 2. Validator Su Risultati Esistenti

Eseguito:

```powershell
python validation\validate_sim_results.py --results-dir results --out reports\user\2026-04-16_validazione_simulatore.md
```

Esito: FAIL atteso per tautologia SEA.

Report generato:

```text
reports/user/2026-04-16_validazione_simulatore.md
```

### 3. Smoke `main.py` Plugin Mode

Eseguito:

```powershell
conda run -n envCMC-like python -X faulthandler main.py --t-start 4.26 --t-end 4.27 --output-dir results\_validation_plugin_faulthandler
```

Esito: crash nativo.

Trace principale:

```text
Windows fatal exception: access violation

File "C:\OpenSim-mCMC\sdk\Python\opensim\simulation.py", line 26976 in realizeAcceleration
File "simulation_runner.py", line 306 in run
File "main.py", line 98 in main
```

Il crash avviene nella chiamata:

```python
model.realizeAcceleration(state)
```

### 4. Script Diagnostico One-Step

Creato temporaneamente:

```text
validation/_debug_one_step.py
```

Scopo:

- riprodurre una sola iterazione del runner;
- separare:
  - `realizeDynamics`;
  - output del plugin;
  - `computeStateVariableDerivatives`;
  - `realizeAcceleration`.

Risultato:

- `model.realizeDynamics(state)` passa;
- l'output C++ `tau_input` del plugin passa;
- `model.computeStateVariableDerivatives(state)` crasha;
- `model.getStateVariableDerivativeValue(...)` crasha;
- non viene stampato nessun checkpoint iniziale dentro `SeriesElasticActuator::computeStateVariableDerivatives()`.

Conclusione diagnostica:

```text
Il crash globale delle derivate avviene prima di entrare nel metodo
SeriesElasticActuator::computeStateVariableDerivatives().
```

Questo e importante: il problema residuo non sembra essere una riga interna della funzione SEA, almeno non nel path osservato. Sembra invece essere nel percorso globale OpenSim delle derivate di stato, probabilmente in un altro componente con stati, oppure nella gestione OpenSim 4.1 di `computeStateVariableDerivatives()` sul modello completo.

### 5. Verifica Output Plugin `tau_input`

Nel debug one-step ho letto l'output:

```python
component = model.getComponent(f"/forceset/{sea_name}")
output = component.getOutput("tau_input")
output.getValueAsString(state)
```

Risultato a stato iniziale e controlli zero:

```text
SEA_Knee 0
SEA_Ankle 0
```

Durante questo test il plugin entra correttamente in `getMotorTorque()`.

Questo dimostra che:

- la DLL viene caricata;
- il plugin e registrato;
- gli output C++ a `Stage::Dynamics` sono invocabili;
- il problema non e il caricamento base del plugin.

### 6. Test Su Flag Thelen

Ho provato localmente, poi revertito, a disattivare dinamiche interne dei muscoli Thelen prima di `initSystem()`:

```python
muscle.set_ignore_activation_dynamics(True)
muscle.set_ignore_tendon_compliance(True)
```

Entrambi i tentativi falliscono in questa build OpenSim:

```text
Thelen2003Muscle::ignore_activation_dynamics flag is not currently implemented.
Thelen2003Muscle::ignore_tendon_compliance flag is not currently implemented.
```

Quindi questa strada non e praticabile senza cambiare modello muscolare o versione OpenSim.

La modifica e stata rimossa da `model_loader.py`. Il file risulta senza diff sostanziale, anche se Git puo mostrarlo modificato per line-ending/pycache.

## Tentativo Non Completato / Respinto

Ho considerato una soluzione alternativa:

- aggiungere al plugin C++ due output diagnostici:
  - `motor_angle_dot`;
  - `motor_speed_dot`;
- leggerli dal runner come output C++ a `Stage::Dynamics`;
- continuare a usare `compute_udot_bypass()` per le accelerazioni generalizzate;
- integrare `motor_angle/motor_speed` con derivate prodotte comunque dal C++, non da Python.

Questa soluzione avrebbe mantenuto la dinamica SEA nel plugin, ma avrebbe cambiato l'interfaccia del plugin aggiungendo output nuovi.

Il tentativo di patch e stato respinto/interrotto. Non sono stati aggiunti questi output.

Stato attuale:

```text
Nessun output nuovo motor_angle_dot / motor_speed_dot e presente nel sorgente.
```

## Stato Corrente Dei File

### Repo simulatore

`git status --short` mostra modifiche in:

```text
config.py
main.py
output.py
simulation_runner.py
opensim.log
validation/
reports/user/2026-04-16_validazione_simulatore.md
validazione_sim.md
__pycache__/
```

Note:

- `validation/_debug_one_step.py` e temporaneo e serve solo per riprodurre il crash;
- `__pycache__/` e stato modificato dai comandi Python;
- `opensim.log` e stato modificato da OpenSim durante i run;
- non ho cancellato questi file perche mi hai chiesto di interrompere.

### Repo `SEA_plugin_core`

`git -C ..\SEA_plugin_core status --short` mostra modifiche anche non direttamente mie.

File sicuramente coinvolti da questa sessione:

```text
SeriesElasticActuator.cpp
SeriesElasticActuator.h
build/Release/SEA_Plugin_BlackBox_mCMC_impedence.dll
```

File gia sporchi o comunque da non toccare senza decisione esplicita:

```text
CMakeLists.txt
ForwardDynamics.cpp
SEATrackingController.h
build/CMakeCache.txt
build/CMakeFiles/...
build/cmake_install.cmake
```

## Diagnosi Provvisoria

La parte `Kp/Kd` del plugin e stata corretta.

Il plugin:

- carica;
- registra `SeriesElasticActuator`;
- espone `tau_input`;
- `getMotorTorque()` e invocabile;
- legge `Kp` e `Kd` coerenti col modello.

Il problema residuo e diverso:

```text
OpenSim crasha quando viene chiesto il calcolo globale delle derivate di stato.
```

La cosa piu importante osservata:

```text
Il crash avviene prima di entrare in SeriesElasticActuator::computeStateVariableDerivatives().
```

Quindi la frase "realizeAcceleration crasha nel plugin SEA" non e ancora dimostrata. Con la strumentazione temporanea, il metodo SEA non viene raggiunto.

Ipotesi piu probabili:

1. `model.computeStateVariableDerivatives()` crasha su un altro componente con stati, verosimilmente un muscolo Thelen2003 o un path OpenSim 4.1 legato ai muscle states.
2. `realizeAcceleration()` in questa build OpenSim non e compatibile col modello completo nello stato inizializzato dal runner.
3. La dinamica SEA C++ potrebbe essere corretta, ma non riusciamo a usarla tramite il path globale OpenSim perche il modello completo cade prima.

## Cosa Fare Alla Ripresa

### Primo Passo Obbligatorio

Ricompilare e ricopiare la DLL pulita, per allineare `plugins/` al sorgente C++ senza checkpoint temporanei:

```powershell
cd "C:\Users\tomma\Desktop\Opensim OMNIBUS\SEA_plugin_core"
cmake --build build --config Release --target SEA_Plugin_BlackBox_mCMC_impedence
Copy-Item -LiteralPath "build\Release\SEA_Plugin_BlackBox_mCMC_impedence.dll" -Destination "..\CMC-like-Simulator - Claude\plugins\SEA_Plugin_BlackBox_mCMC_impedence.dll" -Force
```

### Secondo Passo

Decidere quale strada seguire.

#### Strada A - Debug Del Path Globale OpenSim

Obiettivo: far funzionare davvero:

```python
model.computeStateVariableDerivatives(state)
model.realizeAcceleration(state)
```

Senza cambiare interfaccia plugin.

Azioni possibili:

- creare un test minimale che rimuove progressivamente componenti dal modello;
- verificare se il crash sparisce togliendo i muscoli;
- verificare se il crash sparisce togliendo ExternalLoads;
- creare una copia temporanea del modello senza muscoli solo per isolamento;
- usare debugger nativo Visual Studio sul processo Python per ottenere lo stack C++ reale.

Pro:

- aderisce al piano originale.

Contro:

- puo richiedere molto tempo;
- il crash potrebbe essere dentro OpenSim/Thelen, non nel nostro codice;
- non e detto che sia risolvibile solo con `SeriesElasticActuator.cpp/.h`.

#### Strada B - Output Derivate SEA Dal Plugin C++

Obiettivo: mantenere la dinamica motore SEA in C++, evitando pero il path globale delle derivate del modello.

Idea:

- aggiungere output C++:
  - `motor_angle_dot`;
  - `motor_speed_dot`;
- calcolarli dentro il plugin con la stessa formula usata da `computeStateVariableDerivatives()`;
- leggerli in Python via:

```python
component.getOutput("motor_angle_dot").getValueAsString(state)
component.getOutput("motor_speed_dot").getValueAsString(state)
```

- usare ancora `compute_udot_bypass()` per `qddot`, come gia faceva il runner storico;
- integrare `motor_angle/motor_speed` con valori prodotti dal plugin C++.

Pro:

- evita il crash globale OpenSim;
- non introduce dinamica SEA in Python;
- usa un path C++ gia verificato: gli output plugin funzionano.

Contro:

- cambia l'interfaccia del plugin aggiungendo output;
- serve approvazione esplicita, perche il piano corrente diceva di non modificare gli output.

#### Strada C - Restare In `ideal_torque` Per Ora

Obiettivo: continuare a lavorare su plotter/validator usando la baseline legacy.

Pro:

- simulatore operativo.

Contro:

- risultati ancora tautologici;
- non valida davvero la dinamica SEA;
- utile solo come confronto/regressione.

## Comandi Utili Alla Ripresa

Build plugin:

```powershell
cd "C:\Users\tomma\Desktop\Opensim OMNIBUS\SEA_plugin_core"
cmake --build build --config Release --target SEA_Plugin_BlackBox_mCMC_impedence
```

Copia DLL:

```powershell
Copy-Item -LiteralPath "build\Release\SEA_Plugin_BlackBox_mCMC_impedence.dll" -Destination "..\CMC-like-Simulator - Claude\plugins\SEA_Plugin_BlackBox_mCMC_impedence.dll" -Force
```

Smoke corto plugin:

```powershell
cd "C:\Users\tomma\Desktop\Opensim OMNIBUS\CMC-like-Simulator - Claude"
conda run -n envCMC-like python -X faulthandler main.py --t-start 4.26 --t-end 4.27 --output-dir results\_validation_plugin_faulthandler
```

Validator:

```powershell
python validation\validate_sim_results.py --results-dir results --out reports\user\2026-04-16_validazione_simulatore.md
```

Debug one-step:

```powershell
conda run -n envCMC-like python -X faulthandler validation\_debug_one_step.py
```

## Raccomandazione Per La Prossima Sessione

Non ripartire direttamente da `python main.py`.

Ripartire invece cosi:

1. rebuild/copia DLL pulita;
2. lanciare `validation/_debug_one_step.py`;
3. confermare se la DLL non stampa piu checkpoint temporanei;
4. decidere se procedere con:
   - debug nativo del crash globale OpenSim;
   - oppure nuova micro-interfaccia output C++ per le derivate SEA.

La cosa piu importante da ricordare:

```text
Il plugin C++ non e stato dimostrato come causa del crash residuo.
Il crash residuo accade prima di entrare in computeStateVariableDerivatives() del SEA.
```

