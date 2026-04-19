# Daily Report - 2026-04-19 (consolidato 17/04 → 19/04)

## Sintesi

Le tre giornate del 17, 18 e 19 aprile 2026 hanno tracciato un arco ben definito attorno all'inner loop del plugin SEA:

1. diagnosi del collasso del tracking protesico causato da gain inner troppo bassi nel modello tuned allora attivo;
2. ripristino della baseline inner storica, validazione formale del simulatore in full run, analisi quantitativa del rumore su `tau_input`;
3. sweep parametrico ampio su `(K, K_p, K_d, ω_n, ζ)` e poi su `derivative_filter_tau`, introduzione e costruzione di un filtro derivativo nel plugin C++ e successivo **rollback completo** quando l'analisi dei 1049 risultati screening disponibili ha dimostrato che la metrica di noise usata non reagiva favorevolmente al filtro.

Lo stato finale al 19/04 è:

- plugin `SEA_Plugin_BlackBox_mCMC_impedence.dll` ricompilato dalla versione pre-filtro e installato in `plugins/`;
- sorgenti C++ in `SEA_plugin_core-agent/` ripuliti da ogni traccia delle due property `derivative_filter_tau` e `max_motor_torque` e dallo stato `motor_speed_filt`;
- tutti gli `.osim` in `models/` ripuliti dai relativi tag (compatibili con il DLL pre-filtro che rifiuta property sconosciute);
- due operating point candidati identificati sulla frontiera di Pareto dello sweep esteso, entrambi con `τ_d = 0`, pronti per la full validation.

Non esiste ancora, sulla griglia esplorata, un operating point che soddisfi **contemporaneamente** `tracking < 5°` e `noise_fraction < 0.20` su entrambi i giunti; la direzione è chiara (griglia più larga in `ω_n` e/o metrica noise più onesta), ma non è stata ancora percorsa.

## Report Utente Considerati

Sono stati considerati i seguenti report in `reports/user/`:

- `2026-04-17_diagnosi_divergenza_sea_risultati_pessimi.md`
- `2026-04-17_validazione_simulatore.md`
- `2026-04-17_verifica_inner_loop_sea_tau_input.md`
- `2026-04-17_sea_driver_sweep.md`
- `2026-04-18_validazione_simulatore.md`
- `2026-04-19_validazione_simulatore.md`
- `2026-04-19_rollback_filtro_derivativo_sea.md`

## 1. Diagnosi divergenza SEA (17/04)

Il modello tuned attivo all'inizio del 17/04 aveva questi parametri inner loop:

```text
SEA_Knee:  K=1400, Kp=0.786, Kd=6.9
SEA_Ankle: K=900,  Kp=1.778, Kd=6.9
```

Il tracking dei DOF protesici collassava:

- `pros_knee_angle` RMS circa `25 deg`;
- `pros_ankle_angle` RMS circa `214 deg`, max circa `484 deg`.

La diagnosi nel report dedicato ha seguito una matrice a 7 varianti (4 inner loop × 2 outer loop + 1 full run) su finestra `t = 4.26 → 7.20 s`, con tutte le run salvate in `/tmp/cmc_sea_diagnostics_20260417` e nessuna modifica a codice, modello o `results/` tracked.

Conclusione:

- la causa primaria è il tuning dell'inner loop (categoria **b**), non la logica del plugin né l'outer loop;
- basta sostituire i gain low-level con la baseline 16/04 per recuperare un tracking accettabile;
- la categoria **c** (logica/modellazione) è stata esclusa formalmente: accordo plugin/Python su `tau_input` PASS, derivate finite, `motor_angle` non algebricamente forzato, clamp `±500 Nm` mai raggiunto nella run pessima.

Raccomandazione adottata: ripartire dalla baseline inner stabile

```text
SEA_Knee:  K=250, Kp=18.5, Kd=10
SEA_Ankle: K=500, Kp=35,   Kd=20
```

## 2. Validazione full run con baseline ripristinata (17/04)

Full run `T1_last_good_inner_full` su `t = 4.26 → 11.06 s`, 6800 step, con la baseline inner ripristinata e outer `sea_kp=20, sea_kd=2.0`:

```text
Validator: PASS=44, WARN=2, FAIL=0
```

Il report di validazione formale del 17/04 (`2026-04-17_validazione_simulatore.md`) conferma:

- `SEA_Knee` / `SEA_Ankle` `tau_ref − tau_spring` non banale; accordo plugin/Python sul `tau_input` a `1e-6 Nm`;
- `motor_angle` non vincolato algebricamente; `motor_speed ≠ qdot`;
- derivate finite; clamp `±500 Nm` mai raggiunto;
- tracking protesico `pros_knee_angle` RMS `5.42 deg`, `pros_ankle_angle` RMS `4.47 deg`;
- reserve protesiche ancora a zero (WARN atteso, ora accettabile perché il path SEA è validato).

## 3. Analisi inner PD e decomposizione tau_input (17/04)

Il report `2026-04-17_verifica_inner_loop_sea_tau_input.md` ha quantificato il rumore residuo su `tau_input` con la baseline ripristinata. Dati post-transiente (`t > 4.36 s`, 6700 campioni):

| Metrica                            | Knee         | Ankle        |
|------------------------------------|--------------|--------------|
| τ_ref RMS                          | 9.69 Nm      | 30.54 Nm     |
| τ_spring RMS                       | 9.60 Nm      | 29.78 Nm     |
| **τ_input RMS**                    | **15.49 Nm** | 30.55 Nm     |
| corr(τ_input, τ_ref)               | **0.628**    | 0.969        |
| **frazione di rumore (1 − R²)**    | **60.6 %**   | 6.2 %        |
| d(τ_input)/dt RMS                  | 19 023 Nm/s  | 4 780 Nm/s   |

Decomposizione dell'inner PD:

| Termine                      | Knee RMS (Nm) | Ankle RMS (Nm) |
|------------------------------|---------------|----------------|
| `Kp·(τ_ref − τ_spring)`      | 25.5          | 29.0           |
| `−Kd·ω_m`                    | 25.2          | 9.9            |

Al knee i due termini hanno magnitudine simile (25.5 vs 25.2) e si cancellano parzialmente: il residuo oscilla ad alta frequenza. All'ankle il damping è 3× più piccolo del proporzionale, il segnale domina. Pulsazione naturale e smorzamento del sistema linearizzato: `ω_n,knee ≈ 698 rad/s`, `ζ_knee ≈ 0.723`; `ω_n,ankle ≈ 1342 rad/s`, `ζ_ankle ≈ 0.749`.

**Verdetto**: implementazione corretta in tutte le sue parti (dinamica plugin, PD low-level, integrazione Python). Il problema è strutturale con `K_knee = 250` e `J_m = 0.01`: la molla, per produrre una stessa coppia, richiede più deflessione angolare, il motore sviluppa `ω_m` elevate e il termine `K_d·ω_m` sale fino a eguagliare il proporzionale.

## 4. Primo sweep esteso K/Kp/Kd con parametrizzazione fisica (17/04)

Avviato lo sweep `results/_sea_driver_sweep_20260417_181301` con 198 candidati screening + 5 full validation. Parametrizzazione:

```text
K_p = J_m · ω_n² / K − 1
K_d = 2 · ζ · J_m · ω_n − B_m
```

Griglia: `K_knee ∈ {250, 500, 750, 1000}`, `K_ankle ∈ {500, 750, 1000, 1500}`, `ω_n ∈ {300…1400}`, `ζ ∈ {0.7, 0.85}`.

Best candidate dallo scoring:

```text
full_kk500_ka500_wn900_z0p7
Knee:  K=500, Kp=15.2, Kd=12.5
Ankle: K=500, Kp=15.2, Kd=12.5
ω_n=900, ζ=0.7
```

Metriche full validation:

| Metrica               | Valore      |
|-----------------------|-------------|
| Knee tracking RMS     | 3.85 deg    |
| Ankle tracking RMS    | 4.37 deg    |
| Knee noise fraction   | **0.54**    |
| Ankle noise fraction  | 0.011       |
| max \|τ_input\|       | 444 Nm      |
| max \|u\|             | 0.31        |
| Saturazioni           | 0           |

Applicando le soglie rigide di `SWEEP_INSTRUCTIONS.md`: 16 candidati con tracking `< 5 deg` su entrambi i giunti, 36 con `worst_noise_fraction < 0.20`; **nessun candidato** soddisfa entrambe contemporaneamente. Il trade-off tracking/noise si è rivelato non risolvibile dalla sola reparametrizzazione `(K, ω_n, ζ)`.

## 5. Introduzione del filtro derivativo nel plugin (18/04)

Dalla conclusione del punto 3 (il knee chattering è D-kick amplificato dal rumore numerico su `ω_m`) è stato pianificato un intervento C++:

- due nuove property XML: `derivative_filter_tau` (LPF del primo ordine su `ω_m` nel termine D del ramo non-impedance) e `max_motor_torque` (promossa da costante hardcoded a property, con default `500`);
- nuova state variable continua `motor_speed_filt` gestita dall'integratore OpenSim (rollback-safe, backward-compat a `τ_d = 0`);
- estensione dello sweep con una griglia `DERIV_FILTER_TAU ∈ {0, 0.001, 0.002, 0.005}` s e `ζ = {0.7, 0.85, 1.0, 1.5}` per un totale di 1056 candidati dopo pre-filtro;
- DLL ricompilata e propagata anche alla replica macOS (`REPLICA_PLUGIN_AGENT_MAC.md` aggiornato) e al modello template (`models/Adjusted_SEASEA - Copia_tuned.osim`).

Il runner Python ([simulation_runner.py](../../simulation_runner.py)) è stato esteso per integrare il nuovo stato `motor_speed_filt` nel substep semi-implicito usando `motor_speed_filt_dot` esposto dal plugin, con guard su valori non finiti; backward-compat confermata dal run completo su finestra piena `[4.26, 11.06] s`.

## 6. Sweep allargato e bloccaggi deterministici (18/04)

Lo sweep esteso `results/_sea_driver_sweep_20260418_052206` è stato avviato su 12 worker ma si è bloccato in modo deterministico a **1049 / 1056** candidati screening per via di un `subprocess.run` senza timeout su `main.py`: alcuni candidati con gain estremi (es. `ω_n = 1400`, `ζ = 1.5`, `K = 1000/1500`, `τ_d = 1 ms`) generavano hang del solver SO/SLSQP. Il report `2026-04-19_validazione_simulatore.md` mostra cosa significa qui un candidato degenere: `tau_input` saturato nel 87–93 % dei campioni, `speed_dot` ≈ 5·10⁵ rad/s², mismatch plugin/Python fino a 5500 Nm. Lo screening era già diagnostico prima che il watchdog entrasse in scena.

Correzione portata a regime nello script `validation/sea_driver_sweep.py`: wall-clock `timeout_s = max(300, 120·sim_window)` sul subprocess, con `TimeoutExpired` trattato come `acceptable=False` e `score=∞`, `run_status="timeout"`.

Il report `2026-04-18_validazione_simulatore.md` cattura lo stato di un singolo candidato screening del batch fallito (`screen_kk250_ka500_wn700_z0p7_td0p001`): tracking `knee 2.63 deg / ankle 7.22 deg`, tuttavia **FAIL** su `plugin/Python tau_input agreement` (mismatch `RMS ≈ 12.7 Nm`), coerente con il fatto che il runner stava iniettando manualmente la traiettoria del filtro mentre la griglia testava tutte le combinazioni di `τ_d`.

## 7. Analisi 1049 risultati e rollback filtro (19/04)

In assenza di sweep completo, si è analizzato il CSV parziale via due script stdlib-only (`validation/_analyze_sweep.py` e `validation/_analyze_filter_effect.py`). Su 1049 righe:

| Filtro                                              | Candidati |
|-----------------------------------------------------|----------:|
| Totali                                              | 1049      |
| `complete` + `finite_outputs`                       | 673       |
| Inoltre `max\|τ\| < 4500` e `max\|u\| < 0.95`       | 588       |
| Tracking `< 5 deg` (entrambi i giunti)              | 30        |
| Noise fraction `< 0.20` (entrambi i giunti)         | 91        |
| **Tutte le soglie documentate**                     | **0**     |

Frontiera di Pareto `(worst_track, worst_noise)`: 33 punti.

Accoppiando i candidati con `(K_k, K_a, ω_n, ζ)` identici e `τ_d ∈ {0, 1, 2, 5} ms`, il **`noise_fraction = 1 − R²(τ_input, τ_ref)`** non migliora al variare di `τ_d` e a `5 ms` peggiora sensibilmente (es. `K=250/500, ω_n=300, ζ=1.5`: 0.156 → 0.162 → 0.168 → 0.531). Interpretazione fisica: la LPF introduce phase lag che `1 − R²` penalizza più del guadagno sulla riduzione del chattering HF. La metrica scelta confonde lag e rumore.

Conseguenze operative:

- nessun candidato top-15 per composite score `worst_track + 10·worst_noise` ha `τ_d > 0`;
- non ha senso mantenere il filtro finché la metrica non viene sostituita / affiancata (es. HPF-energy-ratio su `τ_input` in post-processing);
- rollback pulito del plugin alla versione pre-filtro.

Operating point scelti sulla frontiera di Pareto (entrambi con `τ_d = 0`, trasferibili al plugin pre-filtro senza perdita):

**Best noise** — `screen_kk1000_ka500_wn700_z0p7_td0`

| Giunto | K [N·m/rad] | Kp    | Kd    |
|--------|-------------|-------|-------|
| Knee   | 1000        | 3.90  | 9.70  |
| Ankle  | 500         | 8.80  | 9.70  |

Screening: `trk_k=8.67°`, `trk_a=8.90°`, `nf_k=0.175`, `nf_a=0.004`, `max|τ|=98 N·m`, `max|u|=0.40`.

**Least bad** — `screen_kk1000_ka500_wn900_z0p7_td0`

| Giunto | K [N·m/rad] | Kp    | Kd    |
|--------|-------------|-------|-------|
| Knee   | 1000        | 7.10  | 12.50 |
| Ankle  | 500         | 15.20 | 12.50 |

Screening: `trk_k=6.79°`, `trk_a=4.64°`, `nf_k=0.341`, `nf_a=0.011`, `max|τ|=204 N·m`, `max|u|=0.32`.

## 8. Rollback plugin e pulizia modelli (19/04)

Rimosso il filtro derivativo e promuovendo di nuovo a hardcoded il clamp 500 Nm:

| File | Modifica |
|---|---|
| [SEA_plugin_core-agent/SeriesElasticActuator.h](../../../SEA_plugin_core-agent/SeriesElasticActuator.h) | rimosse property `derivative_filter_tau`, `max_motor_torque`; rimossi output `motor_speed_filt`, `motor_speed_filt_dot` e accessor |
| [SEA_plugin_core-agent/SeriesElasticActuator.cpp](../../../SEA_plugin_core-agent/SeriesElasticActuator.cpp) | rimossi `constructProperty_*` relativi, state variable `motor_speed_filt` (add/init/derivative), `getMotorSpeedFilt(Dot)`, logica del ramo D filtrato; ripristinato `const double MAX_MOTOR_TORQUE = 500.0` |
| [plugins/SEA_Plugin_BlackBox_mCMC_impedence.dll](../../plugins/SEA_Plugin_BlackBox_mCMC_impedence.dll) | rebuild Release x64 dalla build dir fresca `SEA_plugin_core-agent/build_fresh`, 196 096 byte; backup `.bak_20260419_095419` |
| [models/Adjusted_SEASEA - Copia_tuned.osim](../../models/Adjusted_SEASEA%20-%20Copia_tuned.osim) | rimossi i 4 tag (`<derivative_filter_tau>0</…>` e `<max_motor_torque>500</…>` per SEA_Knee e SEA_Ankle) |

Verifiche eseguite:

- Grep `derivative_filter_tau|motor_speed_filt|max_motor_torque` sui sorgenti C++ → 0 match nel codice in linker (solo reference in `ScalarActuator/*.txt` storici);
- CMake configure + MSBuild Release della build dir fresca: build pulita;
- Grep sullo stesso pattern su tutti i 6 `.osim` di `models/` → 0 match.

## Stato Finale Del Progetto

Codice:

- plugin pre-filtro sia lato sorgente (`SEA_plugin_core-agent/`) sia lato binario (`plugins/*.dll`);
- simulation runner Python compatibile sia con il filtro attivo che con il plugin pre-filtro (la logica di integrazione di `motor_speed_filt` resta nel runner come dead code inerte quando il plugin non espone l'output — va pulita o lasciata inerte a seconda della scelta futura).

Modelli:

```text
models/Adjusted_SEASEA - Copia.osim
models/Adjusted_SEASEA - Copia_tuned.osim              ← target sweep, pulito
models/Adjusted_SEASEA - Copia_tuned_candidate{2,3,4}.osim
models/Adjusted_SEASEA.osim
```

Path di simulazione in `config.py`:

```text
sea_forward_mode = plugin
dt = 0.001
sea_motor_substeps = 5
sea_motor_max_substeps = 80
```

Parametri SEA suggeriti da applicare prima del prossimo full run (scegliere uno dei due):

```text
# Best noise
SEA_Knee:  K=1000, Kp=3.90, Kd=9.70
SEA_Ankle: K=500,  Kp=8.80, Kd=9.70

# Least bad
SEA_Knee:  K=1000, Kp=7.10,  Kd=12.50
SEA_Ankle: K=500,  Kp=15.20, Kd=12.50
```

## File E Cartelle Rilevanti

Sweep 17/04:

```text
results/_sea_driver_sweep_20260417_181301/
  sweep_results.csv
  best_candidate.json
  runs/full_kk500_ka500_wn900_z0p7/
```

Sweep esteso 18/04 (interrotto a 1049/1056):

```text
results/_sea_driver_sweep_20260418_052206/
  sweep_results.csv
  runs/...
```

Script d'analisi stdlib-only (permanenti):

```text
validation/_analyze_sweep.py
validation/_analyze_filter_effect.py
```

Script sweep con timeout subprocess:

```text
validation/sea_driver_sweep.py
```

## Prossimi Passi Consigliati

1. **Full validation** dei due candidati `Best noise` e `Least bad` su `[4.26, 11.06] s` con `main.py`, aggiornando `models/Adjusted_SEASEA - Copia_tuned.osim` e `config.py` in sequenza, una configurazione alla volta.
2. **Metrica noise onesta**: implementare `HPF-energy-ratio(τ_input, 50 Hz)` come post-processing degli `.sto` già prodotti, ri-analizzare il CSV 1049-rows e decidere se la variante con filtro derivativo vale un nuovo sweep.
3. **Grid expansion** tracking-dominato (se serve abbattere davvero il `noise < 0.20` senza sacrificare tracking): `ω_n ∈ {1400, 1800}` con `ζ = 0.7`, `K_ankle ∈ {250, 500}`; riservato al caso in cui i due candidati correnti non passino la full validation.
4. **Cleanup runner**: se si archivia definitivamente il filtro derivativo, rimuovere da `model_loader.py` e `simulation_runner.py` il cablaggio per `sea_motor_speed_filt_sv_idx` e `motor_speed_filt_dot`, che restano dead code sul plugin pre-filtro.

## Conclusione

Il tridente 17 → 19 aprile ha chiuso in modo netto due domande tecniche:

- il collasso del 16/04 era un problema di tuning inner loop, non di interfaccia plugin (17/04);
- la strada del filtro derivativo nel plugin, presa in risposta al trade-off tracking/noise, non è giustificata dalla metrica `1 − R²` attualmente in uso (19/04).

Il rollback del plugin è un risultato pulito: mantiene la dinamica verificata, elimina una property che non aiutava, e lascia il sweep CSV completo come patrimonio riusabile per scegliere l'operating point senza rilanciare simulazioni. Il passo successivo non richiede nuova codifica ma una scelta di metrica e una full validation dei due candidati di Pareto.
