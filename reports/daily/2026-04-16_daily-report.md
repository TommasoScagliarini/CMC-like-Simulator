# Daily Report - 2026-04-16

## Sintesi

La giornata del 16/04/2026 e stata concentrata sulla validazione del simulatore SEA plugin-driven, sulla stabilizzazione del runtime Windows, sul tuning dei parametri low-level SEA e sul primo sweep parametrico automatico.

Il punto tecnico piu importante e stato confermare che il simulatore non usa piu la scorciatoia algebrica precedente:

```text
motor_angle = q + tau_ref / K
motor_speed = qdot
```

La dinamica SEA ora resta plugin-driven. Le derivate motore vengono esposte dal plugin C++ e integrate nel runner senza introdurre una dinamica alternativa Python.

## Report Utente Del Giorno

Sono stati considerati i seguenti report in `reports/user/`:

- `2026-04-16_stop_validazione_sea_plugin.md`
- `2026-04-16_safe_path_plugin_dll_exception.md`
- `2026-04-16_fix_plugin_safe_runtime_agent.md`
- `2026-04-16_runtime_sea_plugin_validato.md`
- `2026-04-16_validazione_simulatore.md`
- `2026-04-16_tuning_sea_kp_kd.md`
- `2026-04-16_tuning_sea_kp_kd_validation.md`
- `2026-04-16_sea_parameter_sweep.md`
- `2026-04-16_parameter_sweep_sea.md`

## 1. Interruzione E Stato Di Ripresa

Nel report `2026-04-16_stop_validazione_sea_plugin.md` e stato documentato lo stato della sessione interrotta sulla validazione SEA plugin-driven.

Punti chiave:

- obiettivo: correggere il path plugin-driven senza implementare una dinamica SEA Python;
- problema iniziale: mismatch tra proprieta del modello `.osim` e plugin C++;
- vincolo: lavorare sul plugin solo se necessario e preservare la build DLL;
- stato: lavoro interrotto su richiesta dell'utente con report dettagliato per ripresa successiva.

Questo report e stato il punto di ripartenza della sessione seguente.

## 2. Diagnosi Errori Nativi Windows

Nel report `2026-04-16_safe_path_plugin_dll_exception.md` sono stati documentati gli errori nativi Windows osservati durante il passaggio al path plugin:

- access violation con memoria non leggibile;
- eccezione software sconosciuta `0xc06d007f`;
- rischio di caricamento DLL OpenSim non coerente.

Decisioni importanti:

- non continuare a modificare direttamente il sorgente originale `SEA_plugin_core`;
- isolare il lavoro C++ in una copia agent;
- rendere esplicito il path DLL OpenSim lato Windows;
- evitare chiamate OpenSim native ritenute fragili in quella configurazione.

## 3. Isolamento Plugin E Runtime Sicuro

Nei report `2026-04-16_fix_plugin_safe_runtime_agent.md` e `2026-04-16_runtime_sea_plugin_validato.md` e stato documentato il percorso stabile adottato.

Azioni principali:

- creata la copia:

```text
C:\Users\tomma\Desktop\Opensim OMNIBUS\SEA_plugin_core-agent
```

- sorgente originale `SEA_plugin_core` mantenuto pulito;
- DLL compilata dalla copia agent;
- aggiunti output diagnostici dal plugin per:

```text
tau_input
motor_angle_dot
motor_speed_dot
```

- runner Python aggiornato per integrare gli stati SEA usando le derivate esposte dal plugin;
- evitata la dipendenza da `realizeAcceleration()` nel path finale validato su Windows.

Risultato:

- gli errori nativi Windows non si sono ripresentati nei test eseguiti;
- il path `sea_forward_mode = "plugin"` e diventato eseguibile;
- il simulatore non forza piu `motor_angle` e `motor_speed` in modo algebrico.

## 4. Validazione Anti-Tautologia

Nel report `2026-04-16_validazione_simulatore.md` e nei report collegati e stata verificata la qualita del path plugin-driven.

Controlli introdotti o usati:

- `tau_ref - tau_spring`;
- `theta_m - (q + tau_ref/K)`;
- `omega_m - qdot`;
- derivate motore finite;
- confronto `tau_input` plugin vs ricostruzione Python della stessa legge;
- saturazioni `tau_input`;
- tracking output vs IK;
- power joint/motor;
- reserve protesiche diagnostiche.

Conclusione tecnica:

- i risultati non sono piu tautologici;
- il plugin produce effettivamente la dinamica motore;
- le reserve protesiche restano zero per design, ma sono accettabili solo perche il path SEA e stato validato;
- i problemi residui osservati sono parametrici/dinamici, non di interfaccia plugin.

## 5. Tuning Manuale Kp/Kd

Nel report `2026-04-16_tuning_sea_kp_kd.md` e nella validazione `2026-04-16_tuning_sea_kp_kd_validation.md` e stato eseguito un tuning manuale conservativo dei gain low-level SEA.

Vincoli:

- non modificare `Jm`;
- non modificare `Bm`;
- non modificare la dinamica plugin;
- non modificare il source C++.

Parametri finali scelti:

```text
SEA_Knee:
  K  = 250
  Kp = 18.5
  Kd = 10

SEA_Ankle:
  K  = 500
  Kp = 35
  Kd = 20
```

Risultato della full run tuned:

```text
Summary: PASS=44, WARN=2, FAIL=0
```

Effetti principali:

| Metrica | Prima | Tuned |
|---|---:|---:|
| Knee saturazioni `tau_input` | 19 | 0 |
| Ankle saturazioni `tau_input` | 5 | 0 |
| Knee `tau_input_raw max_abs` | 1898.927 Nm | 465.412 Nm |
| Ankle `tau_input_raw max_abs` | 946.986 Nm | 331.445 Nm |
| Knee `motor_speed max_abs` | 58.484 rad/s | 34.336 rad/s |
| Ankle `motor_speed max_abs` | 32.106 rad/s | 9.941 rad/s |

Tradeoff:

- tracking protesico meno perfetto;
- dinamica piu realistica;
- eliminazione delle saturazioni di `tau_input`;
- conferma che i problemi erano di tuning, non di interfaccia plugin.

## 6. Parameter Sweep K/Kp/Kd

Nei report `2026-04-16_sea_parameter_sweep.md` e `2026-04-16_parameter_sweep_sea.md` e stato documentato il primo sweep automatico.

Obiettivo:

- variare le stiffness `K` dei due SEA;
- ricavare `Kp` e `Kd` imponendo:

```text
omega_n = 500 rad/s
zeta    = 0.7
```

Formula usata:

```text
Kp = Jm * omega_n^2 / K - 1
Kd = 2 * zeta * Jm * omega_n - Bm
```

Con `Jm=0.01`, `Bm=0.1`:

```text
Kd = 6.9
Kp = 2500 / K - 1
```

Script creato:

```text
validation/_tmp_sea_parameter_sweep.py
```

Verifiche eseguite:

```powershell
python -m py_compile validation\_tmp_sea_parameter_sweep.py
python validation\_tmp_sea_parameter_sweep.py --dry-run
conda run -n envCMC-like python validation\_tmp_sea_parameter_sweep.py --workers 2 --quick-smoke
conda run -n envCMC-like python validation\_tmp_sea_parameter_sweep.py --workers 12
```

Risultato:

```text
Nessuna soluzione accettabile trovata.
```

Motivi principali:

- candidati con stiffness bassa migliorano lo score ma saturano `tau_input`;
- candidati con stiffness piu alta evitano parte dell'aggressivita ma falliscono tracking/validator;
- il vincolo `omega_n=500 rad/s`, `zeta=0.7` e troppo restrittivo nello spazio esplorato.

Policy rispettata:

- modello tuned non aggiornato;
- `config.py` non aggiornato dallo sweep;
- script temporaneo non cancellato;
- cartella sweep mantenuta per analisi.

## Stato Finale Del Progetto

Il simulatore resta configurato sul modello tuned:

```text
models/Adjusted_SEASEA - Copia_tuned.osim
```

Parametri SEA attivi:

```text
SEA_Knee:
  K  = 250
  Kp = 18.5
  Kd = 10

SEA_Ankle:
  K  = 500
  Kp = 35
  Kd = 20
```

Il path validato e:

```text
sea_forward_mode = plugin
dt = 0.001
sea_motor_substeps = 5
sea_motor_max_substeps = 80
```

La DLL plugin in uso deriva dalla copia agent, mentre il sorgente originale e stato lasciato pulito.

## File E Cartelle Rilevanti

Output tuning validato:

```text
results/_tune_k18p5d10_a35d20_full
plot/16_04_2026 - 5
reports/user/2026-04-16_tuning_sea_kp_kd_validation.md
```

Output sweep:

```text
results/_sea_parameter_sweep_20260416_205113
results/_sea_parameter_sweep_20260416_205113/sweep_results.csv
results/_sea_parameter_sweep_20260416_205113/best_candidate.json
```

Script temporaneo ancora presente perche nessuna soluzione e stata accettata:

```text
validation/_tmp_sea_parameter_sweep.py
```

## Prossimi Passi Consigliati

1. Non ripetere lo stesso sweep con `omega_n=500`, `zeta=0.7`: ha gia mostrato che lo spazio attuale non contiene una soluzione accettabile.

2. Pianificare uno sweep esteso su `omega_n`, ad esempio:

```text
omega_n = [200, 250, 300, 350, 400, 450, 500]
zeta = 0.7
```

3. Tenere la formula fisica completa:

```text
Kp = Jm * omega_n^2 / K - 1
Kd = 2 * zeta * Jm * omega_n - Bm
```

4. Valutare se includere il tuning del controller high-level `sea_kp/sea_kd`, per recuperare tracking senza irrigidire troppo il loop interno.

5. Prima di un nuovo sweep, decidere se il limite `tau_input_raw <= 480 Nm` debba restare rigido o diventare un criterio di score con soglia soft.

## Conclusione

Il 16/04 ha chiuso una fase importante: il simulatore e ora plugin-driven, validato contro la tautologia precedente, stabile su Windows nel path scelto e dotato di strumenti diagnostici per tuning e sweep.

Lo sweep automatico con pulsazione naturale fissa a `500 rad/s` non ha trovato una soluzione accettabile, ma ha fornito un risultato utile: il problema successivo non e piu correggere l'interfaccia sim/plugin, ma scegliere un vincolo dinamico meno rigido o ottimizzare anche il livello high-level.
