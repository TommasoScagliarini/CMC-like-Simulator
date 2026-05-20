# 2026-05-17 Daily Report

## Sintesi

La giornata del 17/05/2026 ha chiuso due blocchi importanti:

1. il **PI inner e' stato implementato dentro il plugin SEA** come state
   variable OpenSim/RK4 (`torque_error_integral`) e validato con smoke,
   full run, confronto PD/PI e driver isolation suite;
2. il **controller prostetico cascade** position-P / velocity-PI e' stato
   aggiunto lato Python, testato in tre bande, plottato e poi preparato per
   tuning locale knee+ankle.

Sono stati anche corretti i plot con header modello errato, creato un setup XML
che punta al modello PI `321/500`, e impostata la configurazione finale verso
un cascade aggressivo con ankle tuning dedicato.

## 1. PI inner nel plugin SEA

Report sorgente:

- `reports/user/2026-05-17_pi_inner_plugin_rk4_state_variable.md`

Obiettivo:

- spostare completamente il PI inner nel plugin C++ `SeriesElasticActuator`;
- rappresentare l'integrale dell'errore di coppia come state variable reale;
- mantenere il modello PD baseline separato;
- tenere la banda del driver equivalente al PD post-bump.

Implementazione:

- aggiunte nel plugin:
  - property `Ki`;
  - property `integral_torque_limit`;
  - state variable `torque_error_integral`;
  - output `torque_error_integral_dot`;
- legge non-impedance:

```text
tau_input_raw = tau_ref
              + Kp * (tau_ref - tau_spring)
              + clamp(Ki * xi, -integral_torque_limit, +integral_torque_limit)
              - Kd * omega_m

xi_dot = tau_ref - tau_spring
```

- anti-windup:
  - clamp diretto del contributo `Ki*xi`;
  - freeze condizionale dell'integratore quando il clamp o la saturazione
    `tau_input` sarebbero spinti ulteriormente dall'errore;
- RK4 generalizzato:
  - integra ogni state variable SEA che espone output `<state_name>_dot`;
  - fallisce esplicitamente se una state SEA non ha derivata leggibile.

Modello PI creato:

- `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim`

Parametri:

```text
SEA_Knee:  K=321, Kp=18,   Kd=11, Ki=190, integral_torque_limit=100
SEA_Ankle: K=500, Kp=11.3, Kd=11, Ki=123, integral_torque_limit=100
```

Target banda:

```text
omega_i = Ki / (1 + Kp) = 10 rad/s
```

Verifiche:

- build plugin: PASS;
- smoke load OpenSim: PASS, modello letto con `Ki`,
  `torque_error_integral` e `torque_error_integral_dot`;
- smoke run `results/_pi_inner_smoke_20260517`: complete, 30 step;
- accordo `tau_input` plugin/Python su smoke:
  - knee RMS `2.49e-06 Nm`;
  - ankle RMS `1.92e-06 Nm`;
- full run PI `results/_pi_inner_bandmatched_full_20260517`: complete;
- rerun PD baseline `results/_fast_inner_pid_20260516_rerun_20260517`:
  complete.

Confronto full PD/PI:

| Metrica | PD | PI | Delta PI-PD |
| --- | ---: | ---: | ---: |
| mean prosthetic RMSE | 1.8735 deg | 1.8903 deg | +0.90 % |
| knee RMSE | 1.2958 deg | 1.3174 deg | +1.67 % |
| ankle RMSE | 2.4512 deg | 2.4632 deg | +0.49 % |
| knee tau_error RMS | 1.2295 Nm | 1.0522 Nm | -14.4 % |
| ankle tau_error RMS | 1.0040 Nm | 0.9066 Nm | -9.7 % |
| knee motor_speed_dot RMS | 769.5 rad/s2 | 791.0 rad/s2 | +2.8 % |
| ankle motor_speed_dot RMS | 459.9 rad/s2 | 473.5 rad/s2 | +3.0 % |
| tau_input saturations | 0 | 0 | invariato |

Esito:

- PI inner validato dentro plugin;
- RK4 non congela `xi`;
- il PI riduce l'errore di coppia interno con tracking cinematico quasi
  invariato rispetto al PD.

## 2. Driver isolation PI e analisi concettuale

Artefatti:

- `tools/driver_isolation_suite_pi.py`
- `results/_driver_isolation_pi_20260517/`
- `plot/05_17_2026_driver_isolation_pi/`

La suite PI ha replicato la filosofia della driver isolation PD, ma con stato
ODE a 3 componenti:

```text
[theta_m, omega_m, xi]
```

Risultati chiave documentati nel README dei plot:

- PI stabile con margine Routh ampio:
  - `Ki/Ki_max = 0.0090` per knee e ankle;
- polo lento reale:
  - circa `-10.19 rad/s`, cioe' `1.62 Hz`;
- poli complessi quasi sovrapposti al PD:
  - knee circa `775 rad/s`;
  - ankle circa `779 rad/s`;
- zero PI circa `1.59 Hz`, quasi cancellante il polo lento;
- step response con overshoot circa `6 %` e settling circa `10 ms`;
- nessun clamp integratore nei test nominali.

Discussioni tecniche chiarite durante la giornata:

- mantenere `tau_ff` feedforward va valutato separatamente dal PI inner:
  puo' migliorare il modello simulativo, ma introduce dipendenza esplicita
  da grandezze non sempre disponibili/robuste in protesi reale;
- il termine `Kd * omega_m` resta utile come smorzamento attivo del rotore:
  riduce ringing e limita energia alta frequenza del motor driver;
- la legge con

```text
theta_m_ref = theta_j + tau_ref / K
omega_m_ref = omega_j + tau_ref_dot / K
```

  e' di fatto un controllo di impedenza/posizione motore equivalente per
  generare la coppia richiesta tramite deflessione elastica;
- in una protesi reale l'uso di `theta_j` e' possibile solo se il giunto
  dispone di misura affidabile lato output o stima osservata.

## 3. Cascade outer position-P / velocity-PI

Report sorgente:

- `reports/user/2026-05-17_cascade_outer_position_p_velocity_pi.md`

Obiettivo:

- aggiungere un terzo mode:

```text
--sea-outer-controller cascade
```

- implementare il cascade solo lato Python high-level;
- lasciare invariato il motor driver PI nel plugin;
- ottenere disaccoppiamento frequenziale tra posizione, velocita e driver SEA.

Legge implementata:

```text
e_q      = q_ref - q
qdot_cas = qdot_ref + Kp_outer * e_q
e_v      = qdot_cas - qdot
xi_v_dot = e_v
tau_cmd  = Kp_inner * e_v + Ki_inner * xi_v
u        = clip(tau_cmd / F_opt, -1, 1)
```

File principali modificati:

- `prosthesis_controller.py`
- `config.py`
- `main.py`
- `output.py`
- `plot/plotter.py`
- `tools/cascade_outer_metrics.py`

Diagnostica nuova:

- `cascade_qdot_ref`;
- `cascade_velocity_error`;
- `cascade_inner_p_cmd`;
- `cascade_inner_i_cmd`;
- `cascade_xi_v`;
- `cascade_i_clamped`;
- `cascade_anti_windup_active`.

Set di banda testati:

| set | f_velocity | f_position |
| --- | ---: | ---: |
| conservative | 8 Hz | 1.5 Hz |
| balanced | 10 Hz | 2 Hz |
| aggressive | 15 Hz | 3 Hz |

Smoke run:

- `results/_cascade_conservative_smoke_20260517`: complete;
- `results/_cascade_balanced_smoke_20260517`: complete;
- `results/_cascade_aggressive_smoke_20260517`: complete.

Full run:

- `results/_cascade_conservative_full_20260517`: complete;
- `results/_cascade_balanced_full_20260517`: complete;
- `results/_cascade_aggressive_full_20260517`: complete.

Metriche confronto da `plot/05_17_2026_cascade_outer_comparison/summary.md`:

| run | RMSE mean rad | tau_error RMS Nm | HPF50 RMS | |u|>0.95 | reserve RMS Nm | sat count |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| PD | 0.03256 | 1.11672 | 468.57 | 0.00 % | 114.08 | 0 |
| PI | 0.03286 | 0.97941 | 486.42 | 0.00 % | 114.12 | 0 |
| cascade conservative | 0.34614 | 2.25125 | 147.64 | 0.00 % | 114.56 | 0 |
| cascade balanced | 0.18258 | 1.56082 | 203.99 | 0.00 % | 114.40 | 0 |
| cascade aggressive | 0.05886 | 0.95398 | 323.43 | 0.00 % | 113.94 | 0 |

Selezione automatica tra i tre set:

- `cascade_aggressive`

Motivo:

- e' quello piu' vicino al tracking PD/PI;
- resta senza saturazioni;
- riduce il chattering HPF rispetto al PI baseline;
- paga comunque un tracking peggiore di PD/PI, soprattutto lato ankle.

## 4. Plot e setup XML corretti per modello 321/500

Problema:

- alcuni plot della cartella `05_17_2026_3` riportavano stiffness vecchie
  (`knee=1000`, `ankle=700`) perche' il plotter leggeva il modello indicato
  dal setup/last setup e non quello realmente usato per la run.

Correzione:

- creato:

```text
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml
```

- il setup punta a:

```text
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim
models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot
models/AB06_SEASEA_Threadmill/data/ExternalForces.xml
models/AB06_SEASEA_Threadmill/data/CMC_Actuators.xml
```

- rigenerati i plot cascade con header coerente `K=321/500`:
  - `plot/05_17_2026_cascade_conservative`;
  - `plot/05_17_2026_cascade_balanced`;
  - `plot/05_17_2026_cascade_aggressive`;
- create backup directory per i plot con header sbagliato:
  - `plot/05_17_2026_3_wrong_header_backup`;
  - `plot/05_17_2026_cascade_conservative_wrong_header_backup`;
  - `plot/05_17_2026_cascade_balanced_wrong_header_backup`;
  - `plot/05_17_2026_cascade_aggressive_wrong_header_backup`.

Verifica:

- `missing_channels.txt`: `No missing channels.` per i plot rigenerati.

## 5. Config finale: cascade aggressive ankle tuned

Dopo i tre set standard, la configurazione centrale e' stata aggiornata a:

```text
sea_outer_controller_mode = cascade

knee:
  Kp_outer = 18.85
  Kp_inner = 29.2
  Ki_inner = 1377
  integral_torque_limit = 50 Nm

ankle:
  Kp_outer = 37.7
  Kp_inner = 3.77
  Ki_inner = 355
  integral_torque_limit = 200 Nm
```

Interpretazione:

- knee resta come aggressive;
- ankle usa `omega_v=15 Hz` ma con `I_eff=0.04`, quindi gain inner piu'
  alti rispetto al set aggressive originale ankle;
- `Kp_outer` ankle raddoppia a `37.7 1/s` per recuperare tracking posizione.

Verifica eseguita:

```text
python -m py_compile config.py
PASS
```

## 6. Sweep locale cascade knee+ankle

Richiesta finale di giornata:

- preparare uno script di sweep locale attorno ai gain cascade attuali;
- includere sia knee sia ankle;
- ordinare le metriche per priorita':
  1. tracking cinematico;
  2. chattering joint/motor velocity e `tau_input`;
  3. motor power.

Script creato:

- `validation/cascade_local_gain_sweep.py`

Funzionalita':

- `stage1_knee`: varia solo i gain knee, ankle fisso;
- `stage1_ankle`: varia solo i gain ankle, knee fisso;
- `full`: combina i migliori candidati knee/ankle;
- griglia locale default:

```text
Kp_outer multipliers = [0.75, 0.90, 1.00, 1.10, 1.25]
Kp_inner multipliers = [0.75, 0.90, 1.00, 1.10, 1.25]
Ki_inner multipliers = [0.60, 0.80, 1.00, 1.20, 1.50]
```

- CLI opzionali per sweep dei torque limit integrali;
- output previsti:
  - `baseline_metrics.csv`;
  - `stage1_knee_screen.csv`;
  - `stage1_ankle_screen.csv`;
  - `full_results.csv`;
  - `ranking.csv`;
  - `failures.csv`;
  - `best_candidate.json`;
  - `sweep_summary.json`.

Metriche implementate:

- `knee_tracking_rms_deg`, `ankle_tracking_rms_deg`, `mean_rmse_deg`;
- score cinematico:

```text
score_kinematic_deg = 0.3 * knee_rmse_deg + 0.7 * ankle_rmse_deg
```

- HPF `>50 Hz` su:
  - `pros_*_qdot`;
  - `SEA_*_motor_speed`;
  - `SEA_*_motor_speed_dot_plugin`;
  - `SEA_*_tau_input_plugin`;
- RMS/max di `tau_input_raw/plugin`;
- saturazioni `tau_input`;
- frazione `|u| > 0.95`;
- `motor_power_rms`, `motor_power_mean_abs`, `motor_power_peak_abs`,
  energia positiva/negativa/assoluta.

Verifiche script:

```text
python -m py_compile validation/cascade_local_gain_sweep.py
python validation/cascade_local_gain_sweep.py --dry-run
git diff --check -- validation/cascade_local_gain_sweep.py
```

Risultato:

- PASS.

Quick smoke:

```text
results/_cascade_local_gain_sweep_quick_smoke_20260517
smoke_pass = true
baseline_available = true
knee_rows = 1
ankle_rows = 1
acceptable_rows = 0
```

Nota:

- i due smoke sono `complete` e finiti, quindi il cablaggio funziona;
- entrambi sono marcati `REJECT` per i criteri fisici stretti:
  `tau_input_saturated;tau_input_raw_gt_500`.

Tentativo full sweep locale:

- `results/_cascade_local_gain_sweep_20260517_234151`

Risultato:

```text
stage1_knee rows = 125, acceptable = 0
stage1_ankle rows = 125, acceptable = 5
full combo = non lanciato
motivo = no acceptable stage1 candidates
```

Migliori ankle accettabili nello stage1:

| run | score | knee RMSE | ankle RMSE |
| --- | ---: | ---: | ---: |
| `apo47p125_api2p8275_aii213` | 1.7163 | 0.1828 deg | 2.3734 deg |
| `apo41p47_api2p8275_aii213` | 1.8740 | 0.1831 deg | 2.5986 deg |
| `apo37p7_api2p8275_aii213` | 1.9991 | 0.1832 deg | 2.7774 deg |

Interpretazione operativa:

- lo script e' pronto e testato;
- i criteri di rejection sono molto severi nel contesto `--filter-grf`;
- il baseline default dello script (`results`) non e' perfettamente omogeneo
  con le run sweep perche' `results` ha `enable_grf_contact_filter=False`,
  mentre lo sweep usa `--filter-grf`.

## 7. Artefatti principali

Modelli/setup:

- `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim`
- `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml`

Risultati:

- `results/_pi_inner_smoke_20260517`
- `results/_pi_inner_bandmatched_full_20260517`
- `results/_fast_inner_pid_20260516_rerun_20260517`
- `results/_driver_isolation_pi_20260517`
- `results/_cascade_conservative_smoke_20260517`
- `results/_cascade_balanced_smoke_20260517`
- `results/_cascade_aggressive_smoke_20260517`
- `results/_cascade_conservative_full_20260517`
- `results/_cascade_balanced_full_20260517`
- `results/_cascade_aggressive_full_20260517`
- `results/_cascade_local_gain_sweep_quick_smoke_20260517`
- `results/_cascade_local_gain_sweep_20260517_234151`

Plot:

- `plot/05_17_2026_1`
- `plot/05_17_2026_2_pi_vs_pd`
- `plot/05_17_2026_driver_isolation_pi`
- `plot/05_17_2026_cascade_conservative`
- `plot/05_17_2026_cascade_balanced`
- `plot/05_17_2026_cascade_aggressive`
- `plot/05_17_2026_cascade_outer_comparison`

Report utente:

- `reports/user/2026-05-17_pi_inner_plugin_rk4_state_variable.md`
- `reports/user/2026-05-17_cascade_outer_position_p_velocity_pi.md`

## Stato finale

- PI inner nel plugin: **operativo e validato**.
- RK4 state-variable SEA generico: **operativo**.
- Modello PI `321/500`: **modello corrente per driver PI**.
- Setup XML PI: **creato e usato per plot corretti**.
- Cascade outer: **implementato e stabile**.
- Config centrale: **cascade aggressive ankle tuned**.
- Script sweep locale cascade knee+ankle: **pronto e verificato**, ma il primo
  sweep completo non ha prodotto full combo per criteri di accettazione troppo
  stringenti su knee.

## TODO aperti

Propagati e aggiornati:

- **TODO Windows pendente**: compilare/copiare la DLL plugin aggiornata sulla
  macchina Windows di Tommy, verificare il caricamento del plugin PI, eseguire
  smoke load/run e controllare che il modello PI legga correttamente `Ki`,
  `integral_torque_limit` e `torque_error_integral`.

Chiusi il 17/05:

- **Decisione architetturale PI inner**: scelta state variable OpenSim +
  estensione RK4; implementata.
- **Aggiornamento predictor Python in `output.py` con `Ki*xi`**: implementato
  e verificato con `tau_input_plugin_minus_python`.
- **Driver isolation suite estesa con formula PI**: implementata e plottata.

Nuovi/ancora aperti:

- **Tuning cascade per ankle tracking**:
  - lo script locale esiste;
  - serve decidere se rilassare i criteri hard (`tau_input_raw_gt_500`,
    saturazione singola) oppure mantenere rejection rigida e cambiare griglia;
  - usare una baseline omogenea con `--filter-grf`, ad esempio
    `results/_cascade_aggressive_full_20260517`, invece di `results`.
- **Rieseguire sweep locale cascade** dopo la scelta sopra e produrre
  `ranking.csv`/`best_candidate.json` con almeno una full combo.
- **Confronto finale PD/PI/cascade tuned** dopo lo sweep:
  tracking cinematico, chattering HPF, `tau_input`, power motor, reserve norm.
- **Pulizia artefatti**: valutare se conservare o cancellare cartelle di sweep
  parziali/supersedute (`_cascade_local_gain_sweep_20260517_233607`).
