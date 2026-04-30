# Daily report - 2026-04-29

## Sintesi

Oggi e stata costruita e validata una pipeline OpenSim per adattare il dataset sperimentale `ground_walking_cw` al modello SEA completo e produrre un candidato utilizzabile nel simulatore CMC-like.

Il risultato pratico e un modello SEA-preserving stabile:

`C:\Users\tomma\Desktop\Opensim OMNIBUS\model_pipeline_dir\05_cmc_like_ready\Adjusted_newmarkers_pipeline_ready.osim`

La finestra consigliata per il CMC-like sim e:

`19.05 - 19.70 s`

Questa finestra e stata scelta per evitare sia il transiente iniziale sia la coda problematica del contatto, dove le reserve di bacino crescono molto.

## Report Utente Creati

- `reports/user/2026-04-29_pipeline_newmarkers_sea_cmc_like.md`

Il report descrive problema, soluzione scelta, strategia pipeline, risultati RRA, valutazione CMC-like, simulazione finale, plot generati, file modificati e raccomandazioni operative.

## Diagnosi Dati Sperimentali

Il file GRF `ground_walking_cw_fp.mot` non copre in modo continuo tutto l'intervallo 17-23 s.

Mappatura usata per la finestra utile:

- `FP1 -> foot_l`
- `FP2 -> calcn_r`

Finestra RRA di base:

- `18.965 - 19.839 s`

Finestra CMC-like consigliata:

- `19.05 - 19.70 s`

Motivo: verso la fine del contatto, circa dopo `19.70 s`, le reserve di bacino aumentano sensibilmente; usare tutta la finestra fino a `19.839 s` e stabile numericamente ma meno credibile biomeccanicamente.

## Pipeline Implementata

E stata creata una pipeline isolata in:

`C:\Users\tomma\Desktop\Opensim OMNIBUS\model_pipeline_dir`

Struttura principale:

- `00_inputs`
- `01_scaling`
- `02_ik`
- `03_id`
- `04_rra`
- `05_cmc_like_ready`
- `06_cmc_like_evaluation`
- `07_best_model_plotted_run`
- `workers`

La pipeline usa worker separati per candidati indipendenti di scaling/RRA, evitando conflitti di stato OpenSim/plugin.

## Risultati Pipeline

Scaling selezionato:

- modalita: `marker_only`
- finestra: `17.25 - 17.30 s`
- RMS marker: `0.00012423 m`
- max scale delta: `0.0`

RRA selezionato:

- candidato: `legacy_rra_tasks_pelvis`
- adjusted body: `pelvis`
- residuali:
  - `FX = 1.56452`
  - `FY = -12.5998`
  - `FZ = 16.9503`
  - `MX = -39.9331`
  - `MY = -0.208054`
  - `MZ = 43.2876`
- force norm: `21.178`
- moment norm: `58.894`
- COM shift max component: circa `0.10 m`

Nota: il modello finale e il miglior candidato stabile trovato, non una soluzione biomeccanicamente perfetta. I residual moments restano sopra un target ideale.

## Valutazione CMC-like

Script aggiunto:

- `scripts/evaluate_cmc_like_candidates.py`

Funzione:

- esegue candidati nel simulatore CMC-like;
- valuta completamento, errori, non-finite, saturazioni SEA, warning SO e reserve torques;
- genera CSV/Markdown di ranking.

Risultato:

- miglior candidato: `pipeline_ready_pelvis`
- modelli manuali in `Downloads\Archive\TODO` e `newmarkersScaled.osim` non usabili direttamente per il CMC-like sim perche il loader non trova `SEA_Knee` nell'`ActuatorSet`.

## Simulazione Finale

Comando operativo validato:

```powershell
C:\Users\tomma\anaconda3\Scripts\conda.exe run -n envCMC-like python main.py `
  --setup "C:\Users\tomma\Desktop\Opensim OMNIBUS\model_pipeline_dir\05_cmc_like_ready\Adjusted_newmarkers_pipeline_ready_setup.xml" `
  --t-start 19.05 `
  --t-end 19.70 `
  --solver osqp `
  --sea-feasibility-scaling `
  --output-dir "C:\Users\tomma\Desktop\Opensim OMNIBUS\model_pipeline_dir\07_best_model_plotted_run\results_1905_1970" `
  --log `
  --plot
```

Risultati:

- `status = complete`
- step: `650`
- wall time: circa `304 s`
- errori/exception/non-finite: `0`
- fallback QP: `0`
- SO feasibility warnings: `15`
- saturazioni `SEA_Knee`: `0`
- saturazioni `SEA_Ankle`: `0`
- `tau_reserve_norm` medio: `159.19`
- `tau_reserve_norm` massimo: `326.08`
- `residual_norm` medio: `0.883`
- `residual_norm` massimo: `8.18`

Risultati salvati in:

`C:\Users\tomma\Desktop\Opensim OMNIBUS\model_pipeline_dir\07_best_model_plotted_run\results_1905_1970`

## Plot

Plot finali generati in:

`plot\29_04_2026 - 3`

File:

- `01_time_sea_control_reserve.png`
- `02_time_joint_motor_states.png`
- `03_gaitcycle_torque_angle_power.png`
- `04_gaitcycle_joint_velocity_power.png`
- `05_time_tau_input_tracking_error.png`
- `06_time_joint_ref_sea_error.png`
- `missing_channels.txt`

Nota: le figure `03` e `04` sono parziali perche nella finestra scelta non ci sono eventi gait-cycle completi. Manca anche un dataset healthy overlay.

## File Modificati o Aggiunti Oggi

- `scripts/run_opensim_sea_pipeline.py`
  - runner pipeline OpenSim per scaling, IK, ID, RRA, modello finale e smoke test.

- `scripts/evaluate_cmc_like_candidates.py`
  - valutatore dei candidati nel CMC-like sim.

- `plot/plotter.py`
  - supporto a `--model-bundle` e `--model`;
  - inferenza della finestra temporale dai risultati prima di caricare la reference IK.

- `reports/user/2026-04-29_pipeline_newmarkers_sea_cmc_like.md`
  - report operativo richiesto con `create_report`.

- `reports/daily/2026-04-29_daily-report.md`
  - questo daily report.

## Verifiche Eseguite

- Import OpenSim in ambiente `envCMC-like`.
- Caricamento plugin SEA.
- Verifica presenza `SEA_Knee` e `SEA_Ankle` nel modello finale.
- Scaling automatico sotto target RMS.
- IK su finestra RRA.
- RRA su piu candidati.
- Smoke test CMC-like.
- Candidate sweep CMC-like.
- Simulazione finale `19.05 - 19.70 s`.
- Generazione plot.
- Compilazione Python:
  - `python -m py_compile scripts/evaluate_cmc_like_candidates.py`
  - `python -m py_compile plot/plotter.py`

## Stato e Rischi Residui

- Il candidato e stabile nel simulatore, ma non biomeccanicamente perfetto.
- Il COM shift RRA arriva al limite pratico di circa `0.10 m`.
- I residual moments RRA sono ancora elevati.
- Le reserve di bacino restano non trascurabili.
- La finestra utile non contiene un ciclo completo di gait con GRF continue.

## Raccomandazioni per il Prossimo Step

- Usare `Adjusted_newmarkers_pipeline_ready.osim` solo sulla finestra `19.05 - 19.70 s` per analisi CMC-like con questo dataset.
- Evitare la coda fino a `19.839 s` per risultati biomeccanici robusti.
- Per una validazione piu forte, acquisire o selezionare un trial con GRF continue su almeno un ciclo completo.
- Se si vuole migliorare il modello, indagare massa/COM/inerzia della protesi e coerenza GRF/CoP/momenti esterni prima di ripetere altri pass RRA.
