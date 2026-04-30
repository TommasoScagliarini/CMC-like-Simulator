# Report - Finestra FP12, modello SEASEA e simulazione locale

Data: 2026-04-30

## Problema

L'obiettivo era trovare una finestra con GRF misurate utilizzabile per una
simulazione CMC-like reale sul modello SEASEA, evitando di ricostruire le GRF
prima di avere escluso le finestre misurate disponibili.

Sono state considerate due finestre candidate nel file
`ground_walking_cw_fp.mot`:

- `FP1/FP2`: `15.889 - 19.839 s`, con force plate misurate.
- `Treadmill_L/R`: `30.065 - 34.275 s`, con forze treadmill misurate.

La domanda pratica era:

- quale finestra usare;
- se il modello generato fosse utilizzabile dal simulatore;
- come copiare il modello buono nella cartella `models/SEASEA - whealthy data`;
- come lanciare una simulazione e produrre i plot.

## Soluzione Scelta

La finestra consigliata e:

`17.500 - 19.000 s`

all'interno della finestra misurata:

`FP1/FP2 = 15.889 - 19.839 s`

La finestra treadmill e stata scartata per uso operativo immediato: anche dopo
correzione del mapping anatomico, le run CMC-like sono andate in timeout o sono
fallite con valori non finiti e reserve enormi.

Il modello operativo copiato nella cartella dati e:

`models/SEASEA - whealthy data/Adjusted_newmarkers_fp12_15889_19839_ready.osim`

Setup locale creato:

`models/SEASEA - whealthy data/Adjusted_newmarkers_fp12_15889_19839_setup.xml`

Risultati della simulazione locale:

`results/fp12_17500_19000_local_model`

Plot generati:

`plot/30_04_2026 - 1`

## Strategia

1. Analizzata l'intera disponibilita GRF misurata prima di ricostruire forze.

2. Implementato un runner dedicato per testare finestre misurate:

   `scripts/run_measured_grf_window_tests.py`

3. Il runner esegue per ogni finestra:

   - IK sulla finestra completa;
   - ExternalForces XML coerente con il profilo di forza;
   - ID diagnostico;
   - sweep RRA candidati;
   - promozione del miglior modello SEA-preserving;
   - CMC-like con `--solver osqp --sea-feasibility-scaling`;
   - report per finestra.

4. Corretti due problemi pratici emersi durante il test:

   - RRA usava una finestra hardcoded del vecchio pipeline runner; e stata
     aggiunta una funzione worker locale che usa la finestra richiesta.
   - Le cartelle CMC venivano riusate e potevano leggere status vecchi; ora la
     cartella della singola run viene pulita prima del rilancio.

5. Aggiunto un timeout per le run CMC-like:

   `--cmc-timeout-s`

   Questo evita blocchi lunghi quando una finestra numericamente patologica
   continua a usare CPU senza completare.

6. Corretto il mapping treadmill in base alla distanza CoP-marker osservata:

   - `Treadmill_L -> calcn_r`
   - `Treadmill_R -> foot_l`

   Nonostante questa correzione, la finestra treadmill non e risultata
   utilizzabile per una simulazione stabile.

7. Copiato il modello promosso dalla finestra FP in
   `models/SEASEA - whealthy data`, insieme ai dati necessari:

   - IK della finestra FP;
   - ExternalForces XML FP12;
   - CMC actuators;
   - file MOT GRF misurato.

8. Lanciata una simulazione locale sulla finestra piu pulita:

```powershell
C:\Users\tomma\anaconda3\Scripts\conda.exe run -n envCMC-like python main.py `
  --setup "models\SEASEA - whealthy data\Adjusted_newmarkers_fp12_15889_19839_setup.xml" `
  --output-dir "results\fp12_17500_19000_local_model" `
  --solver osqp `
  --sea-feasibility-scaling `
  --log `
  --plot
```

## Risultati FP1/FP2

Report finestra:

`C:\Users\tomma\Desktop\Opensim OMNIBUS\model_pipeline_dir\window_tests\fp12_15889_19839\window_report.md`

IK:

- RMS marker error max: `0.00970225 m`
- marker error max: `0.0223337 m`

RRA selezionato:

- candidato: `legacy_rra_tasks_torso`
- adjusted body: `torso`
- force norm: `175.929`
- moment norm: `66.1304`
- COM shift max component: `0.1 m`

CMC-like sulla finestra FP:

| run | status | reserve mean | reserve max | pelvis max | joint max | SEA sat |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `full_15889_19839` | `complete` | `271.226` | `1169.39` | `1092.37` | `106.737` | `0` |
| `early_15889_17500` | `complete` | `312.253` | `669.346` | `658.935` | `0.00033255` | `0` |
| `middle_17500_19000` | `complete` | `233.684` | `385.845` | `374.208` | `29.6342` | `0` |
| `late_19000_19839` | `complete` | `255.432` | `1156.57` | `1110.29` | `148.281` | `1` |

La sottofinestra migliore per una simulazione iniziale e quindi:

`17.500 - 19.000 s`

## Risultati Treadmill

Report finestra:

`C:\Users\tomma\Desktop\Opensim OMNIBUS\model_pipeline_dir\window_tests\treadmill_30065_34275\window_report.md`

IK:

- RMS marker error max: `0.0104392 m`
- marker error max: `0.0207057 m`

RRA selezionato:

- candidato: `legacy_rra_tasks_no_com`
- force norm: `93.8697`
- moment norm: `617.567`
- COM shift max component: `0 m`

CMC-like treadmill:

| run | status | note |
| --- | --- | --- |
| `full_30065_34275` | `timeout` | terminata dopo `120 s` |
| `single_to_overlap_30065_31166` | `timeout` | terminata dopo `120 s` |
| `both_active_31166_34275` | `failed` | non-finite RK4 accelerations a `t=31.2215` |

La run `both_active_31166_34275` ha prodotto:

- `tau_reserve_norm_mean = 30057.9`
- `tau_reserve_norm_max = 245611`
- `pelvis max = 100000`
- `joint max = 5000`
- `SEA sat = 28`

Conclusione: la finestra treadmill non e utilizzabile come base stabile per la
simulazione CMC-like corrente.

## Simulazione Locale Con Modello Copiato

Setup:

`models/SEASEA - whealthy data/Adjusted_newmarkers_fp12_15889_19839_setup.xml`

Output:

`results/fp12_17500_19000_local_model`

Stato run:

- `status = complete`
- `t_start = 17.5`
- `t_end = 19.0`
- `step = 1500`
- `wall_time_s = 271.1484529`
- `dt = 0.001`
- `integration_scheme = rk4_bypass`
- `sea_forward_mode = plugin`
- `sea_motor_substeps = 5`
- `sea_motor_max_substeps = 80`

Metriche principali:

- `warnings_total = 13`
- `SO feasibility warnings = 13`
- `nonfinite_total = 0`
- `tau_reserve_norm_mean = 233.68431480398667`
- `tau_reserve_norm_max = 385.84469983`
- `pelvis_reserve_torque_max_abs = 374.20812525`
- `joint_reserve_torque_max_abs = 29.63420099`
- `SEA saturation frames = 0`

File risultati principali:

- `sim_output_run_status.txt`
- `sim_output_kinematics.sto`
- `sim_output_states.sto`
- `sim_output_recruitment.sto`
- `sim_output_reserve_torques.sto`
- `sim_output_sea_diagnostics.sto`
- `sim_output_phase3_log_20260430_133212.txt`

## Plot Generati

Cartella:

`plot/30_04_2026 - 1`

File:

- `01_time_sea_control_reserve.png`
- `02_time_joint_motor_states.png`
- `03_gaitcycle_torque_angle_power.png`
- `04_gaitcycle_joint_velocity_power.png`
- `05_time_tau_input_tracking_error.png`
- `06_time_joint_ref_sea_error.png`
- `missing_channels.txt`

## File Creati o Modificati

Codice:

- `scripts/run_measured_grf_window_tests.py`
  - runner dedicato per finestre GRF misurate;
  - supporto a profili `fp12` e `treadmill_lr`;
  - CMC timeout con `--cmc-timeout-s`;
  - pulizia della cartella output CMC prima di ogni run;
  - clamp della finestra CMC al range reale della cinematica IK;
  - report Markdown e CSV per confronto finestre.

Modello e dati copiati:

- `models/SEASEA - whealthy data/Adjusted_newmarkers_fp12_15889_19839_ready.osim`
- `models/SEASEA - whealthy data/Adjusted_newmarkers_fp12_15889_19839_setup.xml`
- `models/SEASEA - whealthy data/data/IK_fp12_15889_19839.mot`
- `models/SEASEA - whealthy data/data/ExternalForces_fp12_15889_19839.xml`
- `models/SEASEA - whealthy data/data/CMC_Actuators_fp12_15889_19839.xml`
- `models/SEASEA - whealthy data/data/ground_walking_cw_fp.mot`

Output:

- `results/fp12_17500_19000_local_model`
- `plot/30_04_2026 - 1`

Report generati dalla pipeline:

- `C:\Users\tomma\Desktop\Opensim OMNIBUS\model_pipeline_dir\window_tests\fp12_15889_19839\window_report.md`
- `C:\Users\tomma\Desktop\Opensim OMNIBUS\model_pipeline_dir\window_tests\treadmill_30065_34275\window_report.md`
- `C:\Users\tomma\Desktop\Opensim OMNIBUS\model_pipeline_dir\window_tests\measured_window_summary.md`

## Verifiche Eseguite

- `python -m py_compile scripts/run_measured_grf_window_tests.py`
- test del timeout su comando Python dummy;
- run treadmill con `--cmc-timeout-s 120`;
- lettura dei report FP e treadmill;
- simulazione locale con `main.py` su `17.5 - 19.0 s`;
- generazione plot con `--plot`;
- analisi delle metriche tramite `scripts/evaluate_cmc_like_candidates.py`.

## Conclusioni

La finestra operativa consigliata e:

`17.500 - 19.000 s`

Il modello:

`models/SEASEA - whealthy data/Adjusted_newmarkers_fp12_15889_19839_ready.osim`

e utilizzabile dal simulatore e produce una run completa senza valori non
finiti e senza saturazioni SEA sulla finestra scelta.

La soluzione non va ancora considerata biomeccanicamente definitiva, perche le
reserve pelvis restano alte (`pelvis max circa 374 Nm` nella sottofinestra
scelta e molto piu alte agli estremi della finestra completa). Tuttavia e una
base stabile e riproducibile per ispezione visiva, debug del simulatore e
successivo miglioramento di GRF/modello.

Per estendere verso `17 - 23 s`, dopo `19.839 s` mancano GRF FP misurate
coerenti. Prima di usare quell'intervallo lungo sara necessario ricostruire GRF
oppure introdurre una strategia alternativa validata.
