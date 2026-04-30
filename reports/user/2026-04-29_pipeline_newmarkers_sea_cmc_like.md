# Pipeline newmarkers SEA per CMC-like sim

Data: 2026-04-29

## Problema

Il dataset sperimentale `ground_walking_cw` doveva essere usato sul modello SEA completo, ma la cinematica/GRF disponibile non copre in modo continuo tutto l'intervallo 17-23 s. Le forze esterne sono disponibili solo su finestre strumentate limitate:

- `FP1`: appoggio sinistro, circa 15.06-19.17 s.
- `FP2`: appoggio destro, circa 18.96-19.84 s.
- dopo 19.84 s il tratto 17-23 s non ha GRF affidabile per RRA/CMC-like.

Usare l'intervallo lungo portava RRA a compensare con residuali e spostamenti COM troppo grandi. L'obiettivo era ottenere il miglior modello SEA-preserving utilizzabile dal simulatore CMC-like senza crash o warning bloccanti.

## Soluzione Scelta

Il miglior candidato pratico e:

`C:\Users\tomma\Desktop\Opensim OMNIBUS\model_pipeline_dir\05_cmc_like_ready\Adjusted_newmarkers_pipeline_ready.osim`

Setup associato:

`C:\Users\tomma\Desktop\Opensim OMNIBUS\model_pipeline_dir\05_cmc_like_ready\Adjusted_newmarkers_pipeline_ready_setup.xml`

La finestra consigliata per il CMC-like sim e:

`19.05 - 19.70 s`

Comando usato per la simulazione finale:

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

## Strategia

1. Creata una pipeline OpenSim isolata in:

   `C:\Users\tomma\Desktop\Opensim OMNIBUS\model_pipeline_dir`

2. Organizzata la pipeline in sottocartelle:

   - `00_inputs`
   - `01_scaling`
   - `02_ik`
   - `03_id`
   - `04_rra`
   - `05_cmc_like_ready`
   - `06_cmc_like_evaluation`
   - `07_best_model_plotted_run`
   - `workers`

3. Usati 4 worker separati per candidati indipendenti di scaling/RRA.

4. External forces usate per la finestra RRA:

   - `FP1 -> foot_l`
   - `FP2 -> calcn_r`
   - finestra RRA di base: `18.965 - 19.839 s`

5. Confrontati candidati RRA con:

   - COM adjustment su `pelvis`
   - COM adjustment su `torso`
   - nessun COM adjustment
   - task profile repo CMC e task RRA legacy

6. Promosso un modello SEA-preserving: il modello finale parte dal modello SEA scalato e trasferisce solo la correzione COM/massa accettata dal miglior RRA.

7. Valutati i candidati nel simulatore CMC-like, non solo con residuali RRA. Il criterio pratico ha incluso:

   - completamento simulazione
   - assenza di crash
   - assenza di valori non finiti
   - saturazioni SEA
   - warning di static optimization
   - uso di reserve torques

## Risultati Pipeline

Report pipeline:

`C:\Users\tomma\Desktop\Opensim OMNIBUS\model_pipeline_dir\pipeline_report.md`

Scaling selezionato:

- modalita: `marker_only`
- finestra scaling: `17.25 - 17.30 s`
- RMS marker: `0.00012423 m`
- max scale delta: `0.0`

RRA selezionato:

- candidato: `legacy_rra_tasks_pelvis`
- adjusted body: `pelvis`
- residuali finali:
  - `FX = 1.56452`
  - `FY = -12.5998`
  - `FZ = 16.9503`
  - `MX = -39.9331`
  - `MY = -0.208054`
  - `MZ = 43.2876`
- force norm: `21.178`
- moment norm: `58.894`
- COM shift max component: circa `0.10 m`

Nota: i residual moments restano non perfetti; il modello e quindi il miglior candidato stabile, non una soluzione biomeccanicamente perfetta.

## Valutazione CMC-like

Script aggiunto per valutare i candidati:

`scripts/evaluate_cmc_like_candidates.py`

Report candidati:

`C:\Users\tomma\Desktop\Opensim OMNIBUS\model_pipeline_dir\06_cmc_like_evaluation\candidate_sweep\candidate_summary.md`

Risultato: il miglior candidato e `pipeline_ready_pelvis`.

I modelli manuali in `Downloads\Archive\TODO` e `newmarkersScaled.osim` non sono direttamente usabili nel CMC-like sim perche il loader non trova `SEA_Knee` nell'`ActuatorSet`.

## Simulazione Finale e Plot

Risultati simulazione finale:

`C:\Users\tomma\Desktop\Opensim OMNIBUS\model_pipeline_dir\07_best_model_plotted_run\results_1905_1970`

Stato run:

- `status = complete`
- finestra: `19.05 - 19.70 s`
- step: `650`
- wall time: circa `304 s`
- `dt = 0.001`
- `sea_forward_mode = plugin`
- solver SO: `osqp`

Metriche principali:

- errori/exception/non-finite: `0`
- fallback QP: `0`
- SO feasibility warnings: `15`
- `tau_reserve_norm` medio: `159.19`
- `tau_reserve_norm` massimo: `326.08`
- `muscle_share` medio: `0.488`
- `muscle_capable_share` medio: `0.725`
- `residual_norm` medio: `0.883`
- `residual_norm` massimo: `8.18`
- saturazioni `SEA_Knee`: `0`
- saturazioni `SEA_Ankle`: `0`

Reserve torque massime:

- `pelvis_tx_reserve_torque`: `248.58`
- `pelvis_tz_reserve_torque`: `199.15`
- `pelvis_ty_reserve_torque`: `194.52`
- `pelvis_list_reserve_torque`: `130.33`
- `ankle_angle_r_reserve_torque`: `115.11`

Plot finali:

`C:\Users\tomma\Desktop\Opensim OMNIBUS\CMC-like-Simulator - Claude\plot\29_04_2026 - 3`

File generati:

- `01_time_sea_control_reserve.png`
- `02_time_joint_motor_states.png`
- `03_gaitcycle_torque_angle_power.png`
- `04_gaitcycle_joint_velocity_power.png`
- `05_time_tau_input_tracking_error.png`
- `06_time_joint_ref_sea_error.png`
- `missing_channels.txt`

Nota sui plot: le figure `03` e `04` sono parziali perche nella finestra scelta non ci sono eventi gait-cycle completi. Il file `missing_channels.txt` segnala inoltre assenza del dataset healthy overlay.

## File Modificati

- `scripts/run_opensim_sea_pipeline.py`
  - runner completo per scaling, IK, ID, RRA, promozione modello finale e smoke test.

- `scripts/evaluate_cmc_like_candidates.py`
  - valutatore dei candidati direttamente nel CMC-like sim.
  - produce CSV/Markdown con ranking e metriche di stabilita/realismo.

- `plot/plotter.py`
  - aggiunto supporto a `--model-bundle` e `--model`.
  - corretta la reference IK: il plotter ora inferisce `t_start/t_end` dai risultati simulati prima di caricare la reference.

## Verifiche Eseguite

- Import OpenSim in ambiente `envCMC-like`.
- Caricamento plugin SEA.
- Verifica presenza `SEA_Knee` e `SEA_Ankle` nel modello finale.
- Scaling automatico con RMS marker sotto target.
- IK sulla finestra RRA.
- ID finite check implicito nel run della pipeline.
- RRA su candidati multipli.
- Smoke test CMC-like breve.
- Candidate sweep CMC-like su modelli disponibili.
- Simulazione finale `19.05 - 19.70 s`.
- Generazione plot PNG.
- Compilazione Python:
  - `python -m py_compile scripts/evaluate_cmc_like_candidates.py`
  - `python -m py_compile plot/plotter.py`

## Raccomandazioni

Usare il modello finale solo con dati/finestre dove le GRF sono complete e coerenti. Per questo dataset, la finestra operativa migliore trovata e `19.05 - 19.70 s`.

Non usare la coda fino a `19.839 s` per analisi biomeccanica robusta: verso la fine del contatto aumentano molto le reserve di bacino. La simulazione completa gira, ma la finestra corta e piu credibile.

Per arrivare a una soluzione biomeccanicamente piu forte servono dati sperimentali con GRF continue su almeno un ciclo completo, oppure una finestra treadmill/force-plate piu lunga e pulita.
