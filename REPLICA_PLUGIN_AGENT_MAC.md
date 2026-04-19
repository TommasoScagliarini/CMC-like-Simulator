# Replica modifiche SEA plugin su Mac

Questo documento serve per sincronizzare sul Mac il codice del simulatore creato il 16/04/2026 senza rompere il source originale del plugin. Il punto importante e che il simulatore Python puo essere sincronizzato con `git pull`, mentre il source C++ del plugin vive fuori dal repository del simulatore e deve essere replicato manualmente in una copia separata.

La regola operativa e questa:

- non modificare direttamente `SEA_plugin_core`;
- creare una copia sorella chiamata `SEA_plugin_core - agent`;
- applicare le modifiche C++ solo dentro `SEA_plugin_core - agent`;
- compilare la `.dylib` da `SEA_plugin_core - agent`;
- copiare la `.dylib` compilata nella cartella `plugins` del simulatore.

Sul Windows usato il 16/04/2026 la copia di lavoro si chiamava `SEA_plugin_core-agent`. Per il Mac, come richiesto, usa il suffisso con spazi:

```text
SEA_plugin_core - agent
```

## 1. Struttura attesa sul Mac

Si assume una struttura tipo questa:

```text
~/Desktop/Opensim OMNIBUS/
├── CMC-like-Simulator - Claude/
├── SEA_plugin_core/
└── SEA_plugin_core - agent/
```

`CMC-like-Simulator - Claude` e il repository Python del simulatore.

`SEA_plugin_core` e il source originale del plugin. Deve restare pulito.

`SEA_plugin_core - agent` e la copia modificata usata per compilare il plugin validato dal simulatore.

## 2. Sincronizzare il simulatore

Sul Mac, prima sincronizza normalmente il repository Python:

```bash
cd ~/Desktop/"Opensim OMNIBUS/CMC-like-Simulator - Claude"
git pull
```

Questo aggiorna `config.py`, `main.py`, `model_loader.py`, `simulation_runner.py`, `output.py`, `plot/plotter.py`, `validation/validate_sim_results.py` e gli altri file Python. Non aggiorna automaticamente il source C++ del plugin se questo non fa parte dello stesso repository.

Il `config.py` deve restare cosi:

```python
model_file = "models/Adjusted_SEASEA - Copia_tuned.osim"
plugin_name = "plugins/SEA_Plugin_BlackBox_mCMC_impedence"
sea_forward_mode = "plugin"
sea_motor_substeps = 5
sea_motor_max_substeps = 80
```

Non cambiare `plugin_name`: il loader aggiunge automaticamente `.dll`, `.dylib` o `.so` in base al sistema operativo.

## 3. Preparare la copia agent del plugin

Vai nella cartella padre:

```bash
cd ~/Desktop/"Opensim OMNIBUS"
```

Prima controlla se esiste gia una copia agent:

```bash
ls -d "SEA_plugin_core - agent" 2>/dev/null
```

Se non esiste, creala copiando il source originale:

```bash
cp -R SEA_plugin_core "SEA_plugin_core - agent"
```

Se esiste gia e vuoi ripartire pulito, fai prima un backup:

```bash
mv "SEA_plugin_core - agent" "SEA_plugin_core - agent.bak_$(date +%Y%m%d_%H%M%S)"
cp -R SEA_plugin_core "SEA_plugin_core - agent"
```

Se `SEA_plugin_core` e un repository Git, verifica che il source originale non abbia modifiche indesiderate prima di copiarlo:

```bash
git -C SEA_plugin_core status --short -- SeriesElasticActuator.cpp SeriesElasticActuator.h Plugin_interface.cpp CMakeLists.txt
```

Per questa procedura il source originale deve rimanere il riferimento pulito. Le modifiche vanno applicate solo a:

```text
SEA_plugin_core - agent/SeriesElasticActuator.cpp
SEA_plugin_core - agent/SeriesElasticActuator.h
```

Non modificare:

```text
SEA_plugin_core/SeriesElasticActuator.cpp
SEA_plugin_core/SeriesElasticActuator.h
SEA_plugin_core/Plugin_interface.cpp
SEA_plugin_core/CMakeLists.txt
```

## 4. Modifiche richieste in SeriesElasticActuator.h

Apri:

```text
~/Desktop/Opensim OMNIBUS/SEA_plugin_core - agent/SeriesElasticActuator.h
```

### 4.1 Proprieta Kp e Kd

Il modello `.osim` usa le proprieta:

```xml
<Kp>...</Kp>
<Kd>...</Kd>
```

Quindi il plugin deve dichiarare `Kp` e `Kd`, non `Kp_PD` e `Kd_PD`.

Nel file header devono esserci queste proprieta:

```cpp
OpenSim_DECLARE_PROPERTY(motor_inertia,  double, "Rotor inertia Jm [kg*m^2]");
OpenSim_DECLARE_PROPERTY(motor_damping,  double, "Viscous damping Bm [N*m*s/rad]");
OpenSim_DECLARE_PROPERTY(stiffness,      double, "Spring stiffness K [N*m/rad]");
OpenSim_DECLARE_PROPERTY(Kp,             double, "Inner-loop proportional gain");
OpenSim_DECLARE_PROPERTY(Kd,             double, "Inner-loop derivative gain");
OpenSim_DECLARE_PROPERTY(Kd_Imp,         double, "Middle-loop derivative gain");
OpenSim_DECLARE_PROPERTY(Impedence,      bool,   "If true the SEA is controlled by an impedence controller, otherwise by a PD controller");
OpenSim_DECLARE_PROPERTY(derivative_filter_tau, double,
    "Time constant [s] of first-order LPF on omega_m used in the D term of the non-impedance inner loop. 0 disables the filter.");
OpenSim_DECLARE_PROPERTY(max_motor_torque, double,
    "Saturation bound [N.m] on tau_input. Default 500.");
```

`derivative_filter_tau` e `max_motor_torque` sono state aggiunte il
2026-04-18. I default (`0.0` e `500.0`) garantiscono backward-compat
con modelli `.osim` precedenti. Per la struttura dello stato continuo
`motor_speed_filt` consultare [SeriesElasticActuator.cpp](../SEA_plugin_core-agent/SeriesElasticActuator.cpp).

Non devono comparire dichiarazioni attive di:

```cpp
OpenSim_DECLARE_PROPERTY(Kp_PD, ...)
OpenSim_DECLARE_PROPERTY(Kd_PD, ...)
```

### 4.2 Output C++ aggiunti per il simulatore Python

Il runner Python non deve ricostruire la dinamica SEA. Deve leggere dal plugin C++ i segnali dinamici necessari. Per questo il plugin agent espone tre output OpenSim:

```cpp
OpenSim_DECLARE_OUTPUT(tau_input,            double, getMotorTorque,       SimTK::Stage::Dynamics);
OpenSim_DECLARE_OUTPUT(motor_angle_dot,      double, getMotorAngleDot,     SimTK::Stage::Dynamics);
OpenSim_DECLARE_OUTPUT(motor_speed_dot,      double, getMotorSpeedDot,     SimTK::Stage::Dynamics);
OpenSim_DECLARE_OUTPUT(motor_speed_filt,     double, getMotorSpeedFilt,    SimTK::Stage::Dynamics);
OpenSim_DECLARE_OUTPUT(motor_speed_filt_dot, double, getMotorSpeedFiltDot, SimTK::Stage::Dynamics);
```

Subito sotto devono esserci le firme dei metodi:

```cpp
double getMotorTorque(const SimTK::State& s) const;
double getMotorAngleDot(const SimTK::State& s) const;
double getMotorSpeedDot(const SimTK::State& s) const;
double getMotorSpeedFilt(const SimTK::State& s) const;
double getMotorSpeedFiltDot(const SimTK::State& s) const;
```

Questi output sono fondamentali:

- `tau_input`: coppia motore a monte della molla, calcolata dal plugin;
- `motor_angle_dot`: derivata di `motor_angle`, cioe `motor_speed`;
- `motor_speed_dot`: accelerazione del motore SEA calcolata dal plugin;
- `motor_speed_filt`: valore corrente della state variable continua
  `motor_speed_filt` (LPF di `motor_speed` con costante di tempo
  `derivative_filter_tau`). A `tau_d=0` resta a 0 e non viene usata;
- `motor_speed_filt_dot`: derivata richiesta dal runner Python per
  integrare `motor_speed_filt` con Semi-Explicit Euler nei substep SEA.

## 5. Modifiche richieste in SeriesElasticActuator.cpp

Apri:

```text
~/Desktop/Opensim OMNIBUS/SEA_plugin_core - agent/SeriesElasticActuator.cpp
```

### 5.1 Include numerici

Deve essere disponibile `std::isfinite`. Se manca, aggiungi:

```cpp
#include <cmath>
```

Se nel file si usa `std::max` e `std::min` e non sono gia disponibili, assicurati che ci sia anche:

```cpp
#include <algorithm>
```

### 5.2 Costruzione proprieta

Nel metodo `constructProperties()` devono essere usate le proprieta `Kp` e `Kd`:

```cpp
constructProperty_motor_inertia(0.01);
constructProperty_motor_damping(0.1);
constructProperty_stiffness(250.0);
constructProperty_Kp(1000.0);
constructProperty_Kd(20.0);
constructProperty_Kd_Imp(20.0);
constructProperty_Impedence(false);
```

Non usare:

```cpp
constructProperty_Kp_PD(...)
constructProperty_Kd_PD(...)
```

### 5.3 Derivate degli stati SEA

Il metodo `computeStateVariableDerivatives()` deve delegare ai metodi C++ esposti:

```cpp
void SeriesElasticActuator::computeStateVariableDerivatives(const SimTK::State& s) const {
    setStateVariableDerivativeValue(s, "motor_angle", getMotorAngleDot(s));
    setStateVariableDerivativeValue(s, "motor_speed", getMotorSpeedDot(s));
}
```

Questa modifica non cambia la fisica: rende solo riutilizzabile la stessa dinamica anche tramite output OpenSim.

### 5.4 Derivata motor_angle

Aggiungi o verifica:

```cpp
double SeriesElasticActuator::getMotorAngleDot(const SimTK::State& s) const {
    double omega_m = getStateVariableValue(s, "motor_speed");
    return std::isfinite(omega_m) ? omega_m : 0.0;
}
```

### 5.5 Derivata motor_speed

Aggiungi o verifica:

```cpp
double SeriesElasticActuator::getMotorSpeedDot(const SimTK::State& s) const {
    double Jm = get_motor_inertia();
    double Bm = get_motor_damping();
    double K  = get_stiffness();

    if (!std::isfinite(Jm) || Jm < 1e-9) Jm = 1e-9;
    if (!std::isfinite(Bm)) Bm = 0.0;
    if (!std::isfinite(K) || std::abs(K) < 1e-9) K = 1e-9;

    const Coordinate* coord = getCoordinate();
    double theta_joint = (coord != nullptr) ? coord->getValue(s) : 0.0;
    double theta_m = getStateVariableValue(s, "motor_angle");
    double omega_m = getStateVariableValue(s, "motor_speed");

    if (!std::isfinite(theta_joint)) theta_joint = 0.0;
    if (!std::isfinite(theta_m)) theta_m = theta_joint;
    if (!std::isfinite(omega_m)) omega_m = 0.0;

    double tau_spring = K * (theta_m - theta_joint);
    double tau_input = getMotorTorque(s);
    if (!std::isfinite(tau_input)) tau_input = 0.0;

    double omega_m_dot = (tau_input - tau_spring - Bm * omega_m) / Jm;
    return std::isfinite(omega_m_dot) ? omega_m_dot : 0.0;
}
```

Questa e la dinamica SEA gia prevista:

```text
Jm * omega_dot = tau_input - tau_spring - Bm * omega_m
```

Non introdurre formule Python equivalenti nel simulatore: il valore deve arrivare da questo output C++.

### 5.6 Coppia motore tau_input

Aggiungi o verifica:

```cpp
double SeriesElasticActuator::getMotorTorque(const SimTK::State& s) const {
    double Jm    = get_motor_inertia();
    double Bm    = get_motor_damping();
    double K     = get_stiffness();
    double F_opt = getOptimalForce();
    double Kp    = get_Kp();
    double Kd    = get_Kd();
    double Kd_Imp = get_Kd_Imp();

    if (!std::isfinite(Jm) || Jm < 1e-9) Jm = 1e-9;
    if (!std::isfinite(Bm)) Bm = 0.0;
    if (!std::isfinite(K) || std::abs(K) < 1e-9) K = 1e-9;
    if (!std::isfinite(F_opt)) F_opt = 0.0;
    if (!std::isfinite(Kp)) Kp = 0.0;
    if (!std::isfinite(Kd)) Kd = 0.0;
    if (!std::isfinite(Kd_Imp)) Kd_Imp = 0.0;

    double theta_m = getStateVariableValue(s, "motor_angle");
    double omega_m = getStateVariableValue(s, "motor_speed");

    const Coordinate* coord = getCoordinate();
    double theta_joint = (coord != nullptr) ? coord->getValue(s) : 0.0;

    if (!std::isfinite(theta_joint)) theta_joint = 0.0;
    if (!std::isfinite(theta_m)) theta_m = theta_joint;
    if (!std::isfinite(omega_m)) omega_m = 0.0;

    double u = getControl(s);
    if (!std::isfinite(u)) u = 0.0;

    double tau_input = 0.0;
    double tau_spring = K * (theta_m - theta_joint);

    if (get_Impedence()) {
        double tau_ref = u * F_opt;
        double omega_joint = (coord != nullptr) ? coord->getSpeedValue(s) : 0.0;
        if (!std::isfinite(omega_joint)) omega_joint = 0.0;

        double omega_m_ref = omega_joint;
        double tau_ff = tau_spring + (Bm * omega_m);
        double theta_m_ref = theta_joint + tau_ref / K;

        tau_input = tau_ff + Kp * (theta_m_ref - theta_m) + Kd * (omega_m_ref - omega_m);
    } else {
        double tau_ref = u * F_opt;
        tau_input = Kp * (tau_ref - tau_spring) - Kd * omega_m;
    }

    const double MAX_MOTOR_TORQUE = 500.0;
    return std::max(-MAX_MOTOR_TORQUE, std::min(MAX_MOTOR_TORQUE, tau_input));
}
```

Nota: la saturazione a `+/-500 Nm` e rimasta identica. Il fix non cambia la legge fisica del SEA, rende solo robusta la lettura dei parametri e degli stati e rende `tau_input` osservabile dal simulatore.

### 5.7 computeActuation resta invariato come logica

Il metodo `computeActuation()` deve continuare a restituire la coppia generalizzata applicata dal SEA:

```cpp
double SeriesElasticActuator::computeActuation(const SimTK::State& s) const {
    if (get_Impedence()) {
        return getOptimalForce() * getControl(s);
    }

    double theta_m = getStateVariableValue(s, "motor_angle");
    const Coordinate& coord = getCoordinateSet().get(get_coordinate());
    double theta_joint = coord.getValue(s);
    return get_stiffness() * (theta_m - theta_joint);
}
```

Nel source Windows agent e presente una variante equivalente che usa `getCoordinate()` in altre parti del codice. L'importante e non sovrascrivere `computeForce()`: deve restare gestito da `CoordinateActuator`.

## 6. Cose da non modificare nel plugin

Non cambiare:

```text
CMakeLists.txt
Plugin_interface.cpp
ForwardDynamics.cpp
nome classe SeriesElasticActuator
ereditarieta da CoordinateActuator
nomi degli stati motor_angle e motor_speed
nome target SEA_Plugin_BlackBox_mCMC_impedence
```

Non aggiungere una dinamica SEA alternativa in Python.

Non reintrodurre:

```text
motor_angle = q + tau_ref / K
motor_speed = qdot
```

Quella era la scorciatoia che rendeva i risultati "troppo belli".

## 7. Build della dylib su Mac

Vai nella copia agent:

```bash
cd ~/Desktop/"Opensim OMNIBUS/SEA_plugin_core - agent"
```

Pulisci o crea una build separata:

```bash
rm -rf build_agent
```

Configura CMake. Il path `OPENSIM_INSTALL_DIR` dipende dalla tua installazione Mac di OpenSim. Esempi comuni:

```bash
cmake -S . -B build_agent -DCMAKE_BUILD_TYPE=Release -DOPENSIM_INSTALL_DIR="/Applications/OpenSim 4.5"
```

oppure, se OpenSim e installato altrove:

```bash
cmake -S . -B build_agent -DCMAKE_BUILD_TYPE=Release -DOPENSIM_INSTALL_DIR="/path/alla/tua/installazione/OpenSim"
```

Compila:

```bash
cmake --build build_agent --config Release --target SEA_Plugin_BlackBox_mCMC_impedence
```

Trova la libreria prodotta:

```bash
find build_agent -name '*SEA_Plugin_BlackBox_mCMC_impedence*.dylib'
```

Su macOS CMake potrebbe produrre un file con prefisso `lib`, per esempio:

```text
build_agent/libSEA_Plugin_BlackBox_mCMC_impedence.dylib
```

Il simulatore invece usa il basename:

```python
plugin_name = "plugins/SEA_Plugin_BlackBox_mCMC_impedence"
```

Quindi nella cartella `plugins` del simulatore il file finale deve chiamarsi:

```text
SEA_Plugin_BlackBox_mCMC_impedence.dylib
```

## 8. Copiare la dylib nel simulatore

Prima fai un backup della dylib vecchia, se esiste:

```bash
cd ~/Desktop/"Opensim OMNIBUS/CMC-like-Simulator - Claude"

if [ -f plugins/SEA_Plugin_BlackBox_mCMC_impedence.dylib ]; then
  cp plugins/SEA_Plugin_BlackBox_mCMC_impedence.dylib \
     plugins/SEA_Plugin_BlackBox_mCMC_impedence.dylib.bak_$(date +%Y%m%d_%H%M%S)
fi
```

Poi copia la nuova libreria. Adatta il path sorgente al risultato del comando `find`:

```bash
cp ../"SEA_plugin_core - agent"/build_agent/libSEA_Plugin_BlackBox_mCMC_impedence.dylib \
   plugins/SEA_Plugin_BlackBox_mCMC_impedence.dylib
```

Se il file generato non ha prefisso `lib`, usa il nome esatto generato da CMake.

Verifica architettura e nome:

```bash
file plugins/SEA_Plugin_BlackBox_mCMC_impedence.dylib
ls -lh plugins/SEA_Plugin_BlackBox_mCMC_impedence.dylib
```

Su Mac M1/M2/M3 deve risultare compatibile con `arm64`, oppure con l'architettura effettiva del tuo Python/OpenSim.

## 9. Smoke test dopo la copia

Dal simulatore:

```bash
cd ~/Desktop/"Opensim OMNIBUS/CMC-like-Simulator - Claude"
```

Compila i file Python principali:

```bash
python -m py_compile config.py main.py model_loader.py simulation_runner.py output.py validation/validate_sim_results.py
```

Esegui una simulazione corta:

```bash
python main.py --t-start 4.26 --t-end 4.40 --output-dir results/_mac_plugin_smoke --validate --log
```

Poi valida i risultati:

```bash
python validation/validate_sim_results.py --results-dir results/_mac_plugin_smoke
```

Output attesi nella cartella smoke:

```text
sim_output_sea_states.sto
sim_output_sea_torques.sto
sim_output_sea_derivatives.sto
sim_output_sea_diagnostics.sto
sim_output_power.sto
sim_output_run_status.txt
```

Il validator deve confermare che il path non e tautologico:

- `omega_m - qdot` non deve essere identicamente zero;
- `theta_m - (q + tau_ref/K)` non deve essere a precisione macchina;
- `tau_ref - tau_spring` non deve coincidere a precisione macchina;
- `tau_input`, `motor_angle_dot`, `motor_speed_dot` devono essere finiti.

## 10. Test completo con plot e validazione

Se lo smoke test passa:

```bash
python main.py --plot --validate --log
```

Controlla:

```text
results/sim_output_run_status.txt
plot/<data> - <indice>/missing_channels.txt
reports/user/<data>_validazione_simulatore.md
```

`missing_channels.txt` non deve segnalare canali SEA mancanti se tutti gli output sono stati generati.

## 11. Diagnosi errori comuni su Mac

### Plugin non trovato

Controlla che il file esista:

```bash
ls -lh plugins/SEA_Plugin_BlackBox_mCMC_impedence.dylib
```

Il nome deve essere esattamente:

```text
SEA_Plugin_BlackBox_mCMC_impedence.dylib
```

Non basta avere:

```text
libSEA_Plugin_BlackBox_mCMC_impedence.dylib
```

nella cartella `plugins`, a meno che il loader sia stato adattato esplicitamente.

### Architettura sbagliata

Controlla:

```bash
file plugins/SEA_Plugin_BlackBox_mCMC_impedence.dylib
python -c "import platform; print(platform.machine())"
```

La libreria e il Python/OpenSim devono avere architettura compatibile.

### Link a OpenSim sbagliato

Se la libreria viene compilata contro una installazione OpenSim diversa da quella usata dal Python environment, possono comparire crash nativi o errori di load.

Verifica le dipendenze:

```bash
otool -L plugins/SEA_Plugin_BlackBox_mCMC_impedence.dylib
```

Il path OpenSim deve essere coerente con l'ambiente Python che esegue il simulatore.

### Risultati di nuovo "troppo belli"

Se il validator segnala tautologia, cerca nel codice Python:

```bash
grep -R "tau_ref / K\\|motor_speed = qdot\\|ideal_torque\\|q + tau_ref" .
```

La modalita validata deve essere:

```python
sea_forward_mode = "plugin"
```

La modalita `ideal_torque` puo esistere solo come confronto diagnostico, non come simulazione validata.

## 12. Checklist finale

Prima di considerare il Mac allineato al Windows del 16/04/2026, verifica:

- `SEA_plugin_core` originale non e stato modificato;
- esiste `SEA_plugin_core - agent`;
- le modifiche C++ sono solo in `SEA_plugin_core - agent/SeriesElasticActuator.cpp`;
- le modifiche C++ sono solo in `SEA_plugin_core - agent/SeriesElasticActuator.h`;
- in `SeriesElasticActuator.h` ci sono `Kp` e `Kd`, non `Kp_PD` e `Kd_PD`;
- in `SeriesElasticActuator.h` ci sono gli output `tau_input`, `motor_angle_dot`, `motor_speed_dot`;
- in `SeriesElasticActuator.cpp` ci sono `getMotorTorque`, `getMotorAngleDot`, `getMotorSpeedDot`;
- `computeStateVariableDerivatives()` usa quei tre metodi e non duplica logiche diverse;
- `computeForce()` non e stato sovrascritto;
- il target CMake resta `SEA_Plugin_BlackBox_mCMC_impedence`;
- la libreria finale nel simulatore si chiama `plugins/SEA_Plugin_BlackBox_mCMC_impedence.dylib`;
- `config.py` continua a usare `plugin_name = "plugins/SEA_Plugin_BlackBox_mCMC_impedence"`;
- lo smoke test `results/_mac_plugin_smoke` termina senza crash;
- il validator non segnala la vecchia tautologia SEA.

## 13. Rollback rapido

Se dopo la copia della nuova `.dylib` il simulatore non parte, ripristina il backup:

```bash
cd ~/Desktop/"Opensim OMNIBUS/CMC-like-Simulator - Claude"
cp plugins/SEA_Plugin_BlackBox_mCMC_impedence.dylib.bak_YYYYMMDD_HHMMSS \
   plugins/SEA_Plugin_BlackBox_mCMC_impedence.dylib
```

Poi rilancia solo lo smoke:

```bash
python main.py --t-start 4.26 --t-end 4.40 --output-dir results/_mac_plugin_smoke_restore --validate --log
```

## 14. Sintesi operativa breve

Comandi principali:

```bash
cd ~/Desktop/"Opensim OMNIBUS"
cp -R SEA_plugin_core "SEA_plugin_core - agent"

# applica le modifiche a:
# "SEA_plugin_core - agent/SeriesElasticActuator.h"
# "SEA_plugin_core - agent/SeriesElasticActuator.cpp"

cd "SEA_plugin_core - agent"
cmake -S . -B build_agent -DCMAKE_BUILD_TYPE=Release -DOPENSIM_INSTALL_DIR="/Applications/OpenSim 4.5"
cmake --build build_agent --config Release --target SEA_Plugin_BlackBox_mCMC_impedence

cd ../"CMC-like-Simulator - Claude"
cp ../"SEA_plugin_core - agent"/build_agent/libSEA_Plugin_BlackBox_mCMC_impedence.dylib \
   plugins/SEA_Plugin_BlackBox_mCMC_impedence.dylib

python main.py --t-start 4.26 --t-end 4.40 --output-dir results/_mac_plugin_smoke --validate --log
python validation/validate_sim_results.py --results-dir results/_mac_plugin_smoke
```

Se CMake genera la `.dylib` in una sottocartella diversa, trovala con:

```bash
find ../"SEA_plugin_core - agent"/build_agent -name '*SEA_Plugin_BlackBox_mCMC_impedence*.dylib'
```

e copia quel file rinominandolo in:

```text
plugins/SEA_Plugin_BlackBox_mCMC_impedence.dylib
```
