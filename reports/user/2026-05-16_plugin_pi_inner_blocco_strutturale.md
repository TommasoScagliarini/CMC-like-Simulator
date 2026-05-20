# Tentativo plugin SEA con PI inner: ricompilazione completata, ma blocco strutturale del sub-stepper Python - 2026-05-16

## Problema

Chiusura della giornata 2026-05-16:

- Mattina: bump inner gains (`Kp_knee=18`, `Kp_ankle=11.3`,
  `Kd=11` entrambi) sul `.osim` AB06 con run full. Tracking pros
  RMSE -12.46% vs baseline best PID, `knee_tau_err_rms` -62.9%.
- Diagnosi del chattering a 88 Hz come **modo del loop chiuso outer-D
  ↔ driver-inner** (test con `Kd_knee_outer 30 → 15` ha confermato
  riducendo chattering del 60%). Vedi
  [reports/user/2026-05-16_diagnosi_chattering_88Hz_loop_outer_inner.md](./2026-05-16_diagnosi_chattering_88Hz_loop_outer_inner.md).
- Pomeriggio: driver isolation test suite (5 test) ha **certificato il
  driver PD attuale** come matematicamente corretto in isolamento,
  identificando un solo bias residuo: `-(Kd+Bm)*omega_j/(1+Kp)` a
  velocita' di giunto costante (-0.58 Nm/(rad/s) knee, -0.90 ankle).
  Vedi
  [reports/user/2026-05-16_driver_isolation_test_suite.md](./2026-05-16_driver_isolation_test_suite.md).

Decisione operativa di fine giornata: **implementare il PI inner nel
plugin C++** per cancellare quel bias DC, mantenendo BW e zeta correnti
(scelta `Ki_knee=570`, `Ki_ankle=369`, validata analiticamente per
restare entro 0.5 dB della FdT PD su 0-200 Hz).

## Soluzione tentata

Sequenza completata:

1. **Validazione Ki PRIMA del build** (Python standalone):
   `omega_int = 30 rad/s` (knee = 30·19 = 570, ankle = 30·12.3 = 369)
   garantisce `max |H_PI - H_PD| = 0.489 dB @ 37.1 Hz`. Sotto la
   soglia 0.5 dB richiesta. Coerente con l'analisi del
   2026-05-15.

2. **Modifica sorgenti C++** in
   `/Users/tommy/Documents/SEA_plugin_core - agent/SeriesElasticActuator.{cpp,h}`:
   - aggiunta property `Ki`
   - aggiunta state variable `torque_error_integral`
   - inizializzazione a `0.0`
   - `getMotorTorque()` non-impedance: `+ Ki * xi`
   - `computeStateVariableDerivatives()`: registra
     `xi_dot = tau_ref - tau_spring`

3. **Ricompilazione** in `build_agent/` (necessario rigenerare cmake
   perche' il `Makefile` cached aveva ancora il target legacy
   `SEA_Plugin_BlackBox_mCMC_impedence` senza `_ff`). Build OK,
   `.dylib` installato come
   `plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib`.

4. **Aggiornamento `.osim`**: aggiunti `<Ki>570</Ki>` e `<Ki>369</Ki>`
   ai due blocchi `SeriesElasticActuator`.

5. **Run full** sulla finestra `[11.99, 21.00] s` con outer PID
   quasi-best. Status: `complete`, `wall=770 s`, 0 saturazioni.

## Risultato: PI non operativo (bug architetturale rilevato)

Metriche IDENTICHE alla run FAST (no PI):

```text
metric                 FAST (no PI)    PI v1
knee_rmse_deg            1.296          1.296    invariato
ankle_rmse_deg           2.451          2.451    invariato
knee_tau_err_rms [Nm]    1.229          1.229    invariato
ankle_tau_err_rms [Nm]   1.004          1.004    invariato
knee_mdot_max            8962           8962     invariato
```

Diagnostica diretta sull'integrale del plugin:

```text
tau_input_plugin - tau_input_python   knee  RMS = 0.000000 Nm  max = 0.000000
                                      ankle RMS = 0.000001 Nm  max = 0.000005
```

Il Python predictor in `output.py:471-476` ha la formula PD vecchia
(senza `Ki*xi`); se il plugin facesse PI, ci sarebbe una differenza
≠ 0. Invece sono identici → il plugin sta producendo PD puro.

Verifica della state variable:

```text
opensim.LoadOpenSimLibrary(...).Model.initSystem() mostra:
  /forceset/SEA_Knee/motor_angle              value = 0.0
  /forceset/SEA_Knee/motor_speed              value = 0.0
  /forceset/SEA_Knee/torque_error_integral    value = 0.0   <-- mai aggiornato
  /forceset/SEA_Ankle/[same pattern]
```

`Ki` viene letto correttamente (570/369) — il problema NON e' il
.osim ne' il plugin. Il problema e' che `xi` resta `0.0` durante
tutta la simulazione.

## Diagnosi del blocco

Il simulatore custom usa `integration_scheme = rk4_bypass` selezionato
in `config.py`. Questo path bypassa `OpenSim::Manager` per ragioni di
performance e precisione (vedi
[reports/user/2026-04-20_fix_divergenza_gain_100_20_rk4_backtracking.md](./2026-04-20_fix_divergenza_gain_100_20_rk4_backtracking.md)).

Il bypass [simulation_runner.py:_advance_rk4_bypass_state:832](../../simulation_runner.py#L832):

```python
y0 = np.zeros(n_coords * 2 + n_sea * 2)    # <-- HARDCODED a 2 stati per SEA
for i, (sea_name, _coord_name) in enumerate(self._sea_pros_map):
    ma_idx = ctx.sea_motor_angle_sv_idx.get(sea_name)
    ms_idx = ctx.sea_motor_speed_sv_idx.get(sea_name)
    if ma_idx is not None:
        y0[sea_base + i * 2] = sv.get(ma_idx)
    if ms_idx is not None:
        y0[sea_base + i * 2 + 1] = sv.get(ms_idx)
```

E [simulation_runner.py:_sea_derivatives_from_plugin_outputs:743](../../simulation_runner.py#L743):

```python
def _sea_derivatives_from_plugin_outputs(self, sea_plugin_outputs):
    values = np.full(len(self._sea_pros_map) * 2, np.nan)   # <-- solo 2 derivate
    for i in range(len(self._sea_pros_map)):
        values[i * 2] = sea_plugin_outputs[i * 3 + 1]     # motor_angle_dot
        values[i * 2 + 1] = sea_plugin_outputs[i * 3 + 2] # motor_speed_dot
    return values
```

Il sub-stepper integra hardcoded solo `motor_angle` e `motor_speed`.
La nuova state variable `torque_error_integral`, anche se
correttamente registrata dal plugin via OpenSim API, **non viene mai
integrata** perche' `OpenSim::Manager` non gira (e' bypassato).

Risultato: `xi = 0` perpetuo → `Ki*xi = 0` sempre → driver effettivo =
PD puro = identico a FAST.

## Discussione architetturale (osservazione utente)

L'utente ha sollevato un'obiezione di principio: **`xi` non e' uno
stato fisico del motore** (gli stati del motore sono solo
`theta_m, omega_m`). Concettualmente `xi` e' lo stato interno del
**controllore PI**, non del plant. Mescolarli sotto la stessa API
`addStateVariable` viola la separazione plant/controller.

Inoltre l'utente ha aggiunto: **la logica del motor driver deve
restare interamente dentro il plugin** — quindi anche il PI vive nel
plugin (non in Python), ma `xi` non deve essere registrato come state
variable.

Il blocco e' quindi duplice:

- **Tecnico**: il sub-stepper Python ignora le state variables ulteriori
  del plugin.
- **Architetturale**: anche risolvendo il blocco tecnico (estendere il
  sub-stepper), la scelta di registrare `xi` come state variable e'
  contestata dall'utente.

La risoluzione richiede una decisione concettuale (dove sta `xi`?) +
implementazione coerente. Quattro alternative emerse:

```text
1. xi come state variable + estensione del rk4_bypass (rifiutato: viola
   separazione plant/controller secondo utente)

2. xi come mutable double dentro il plugin + integrazione Forward Euler
   interna al plugin (limite: RK4 4 stages × 5 sub-step = 20 chiamate
   per step, riconoscere lo step "nuovo" e' fragile)

3. Disabilitare rk4_bypass e tornare a OpenSim::Manager (rischio
   regressione performance/precisione, gia' validato fix del 2026-04-20)

4. Plugin separato SEA_PI_Controller + plugin SeriesElasticActuator
   invariato (richiede progettare nuovo Component OpenSim)
```

Nessuna scelta a oggi. Decisione deferita.

## Strategia finale per il 2026-05-16

Stato fine giornata: lavoro completato come scoperta (PI inner non e'
banale da integrare in questa pipeline), ma **PI inner non operativo**.

Tutte le modifiche sono **reversibili** tramite backup:

```text
plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib.no_pi_backup_20260517
SEA_plugin_core - agent/SeriesElasticActuator.cpp.pre_pi_backup_20260517
SEA_plugin_core - agent/SeriesElasticActuator.h.pre_pi_backup_20260517
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500.osim.no_ki_backup_20260517
```

(la data 20260517 dei backup e' wall-clock, ma il lavoro logico e'
chiuso col giorno 2026-05-16).

## File modificati / creati

```text
M  SEA_plugin_core - agent/SeriesElasticActuator.cpp         (+Ki, +xi state, +PI)
M  SEA_plugin_core - agent/SeriesElasticActuator.h           (+Ki property)
+  SEA_plugin_core - agent/SeriesElasticActuator.cpp.pre_pi_backup_20260517
+  SEA_plugin_core - agent/SeriesElasticActuator.h.pre_pi_backup_20260517
M  plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib    (ricompilato con PI)
+  plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib.no_pi_backup_20260517
M  models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500.osim  (+<Ki>570/369)
+  models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500.osim.no_ki_backup_20260517
+  results/_pi_inner_20260517/                                (run output, PI NON operativo)
+  /tmp/validate_ki.py                                        (validazione FdT analitica)
+  /tmp/compare_pi.py                                         (confronto metriche)
```

Nessuna modifica a `simulation_runner.py`, `model_loader.py`,
`output.py`, `main.py`, `config.py`, `prosthesis_controller.py`.

## Verifiche eseguite

1. Validazione analitica Ki: scelta `omega_int=30 rad/s` produce
   `max |H_PI - H_PD| = 0.489 dB` (knee) e `0.486 dB` (ankle) su
   0-200 Hz. Coerente coi vincoli BW/zeta.

2. Build C++: cmake regenerate + `make SEA_Plugin_BlackBox_mCMC_impedence_ff`
   completati senza errori. `.dylib` da 217776 byte (vs 217664 byte del
   plugin no-PI).

3. Caricamento OpenSim: `opensim.LoadOpenSimLibrary` riesce. Property
   `Ki` letta correttamente (570 knee, 369 ankle). State variable
   `torque_error_integral` registrata (path
   `/forceset/SEA_Knee/torque_error_integral`).

4. Simbolo `nm`: presente
   `__ZN21SeriesElasticActuator20constructProperty_KiERKd` nel binario.

5. Run full: `status=complete`, `wall=770 s`, `step=9010`.
   Saturazioni 0/0. **MA** reserve biologiche (non protesiche) crescono
   nel finale (`|u_res|` da 0.65 a 3.84 negli ultimi 0.75 s — non
   esplosione vera ma deterioramento, probabilmente correlato ad altri
   transienti del modello e indipendente dal PI dato che il PI e'
   inattivo).

6. Confronto numerico: metriche identiche a FAST. Plugin/python
   predictor identici a 1e-6 Nm → il plugin NON sta applicando `Ki*xi`.

## Prossimi passi

1. **Decidere l'architettura di xi**:
   - opzione 1 (state variable + estensione rk4_bypass);
   - opzione 2 (mutable in plugin + Euler interno);
   - opzione 3 (disabilitare rk4_bypass);
   - opzione 4 (Controller separato).

2. Indipendentemente dalla scelta, **riprendere dal punto attuale**:
   il plugin C++ ha gia' la formula `+Ki*xi` corretta; il `.osim` ha
   gia' i Ki giusti. Va solo:
   - implementare l'integrazione di `xi` coerentemente con la scelta,
   - rilanciare la run,
   - verificare `tau_err RMS` crolli sotto 0.3 Nm.

3. Una volta operativo, certificare il driver PI con la driver
   isolation suite (con formula PI nel Python standalone). Pass se
   step risponde con `e_ss = 0` su tutto il range di tau_ref e disturbo
   `omega_j`.

4. TODO Windows aperto: questo lavoro va replicato (build `.dll` su
   Windows del plugin) quando la macchina sara' allineata.

## TODO aperti (da propagare nei prossimi report)

- Decisione architetturale `xi` (4 opzioni discusse, nessuna ancora
  scelta).
- Implementazione coerente dell'integrazione `xi`.
- Validazione finale PI inner sulla run full.
- Aggiornamento del Python predictor in `output.py:471-476` per
  includere `Ki*xi` (per mantenere `tau_input_plugin ≡ tau_input_python`
  nella diagnostica).
- Driver isolation suite estesa con formula PI per regression test.
- TODO Windows: build `.dll` con PI quando la decisione architetturale
  e' presa.
- Decisione sull'intervento per il chattering 88 Hz (LPF su outer D,
  sweep Kd_outer, bump zeta) — separata dalla questione PI inner.
