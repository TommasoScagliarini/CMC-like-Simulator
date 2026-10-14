# AB07_SEASEA Pipeline Report

Generated: 2026-10-06T11:00:24
Status: complete

## Inputs

- Subject: `AB07`
- Task/trial: `treadmill/treadmill_01_01`
- Static trial: `static01`
- Healthy model: `models/AB07-raw/osimxml/ab07.osim`
- Donor model: `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim`
- SEA plugin: `plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib`

## Outputs

- Bundle: `models/AB07_SEASEA_Threadmill`
- Setup: `models/AB07_SEASEA_Threadmill/AB07_SEASEA_stiff321_500_pi_setup.xml`
- Kinematics: `models/AB07_SEASEA_Threadmill/data/IK_results_AB07_SEASEA.mot`
- IK method: `opensim_marker_ik`
- Original IK output: `results/ab07_seasea_treadmill_treadmill_01_01_ik/02_ik/IK_full_10p360_153p415.mot`
- GRF: `models/AB07_SEASEA_Threadmill/data/AB07_SEASEA_GRF_FullSpan.mot`

## Diagnostics

- Mass before: `58.80765053000001` kg
- Mass after: `58.80765053000002` kg
- Removed left mass: `11.403651589977398` kg
- Marker old mean error: `0.09209757642210165` m
- Marker calibrated RMS: `0.00024937036957675663` m
- Kinematics range: `[10.36, 153.415]` s
- GRF range: `[10.359, 153.415]` s
- Smoke return code: `None`

## Notes

- Removed/replaced 43 healthy left-side force elements during graft.
- Smoke test skipped by user flag.

## Verifica dinamica e stato di qualifica

- Preparazione completata: fitting AB07, calibrazione statica, nuova IK OpenSim dai marker sull'intero trial (10.360–153.415 s), GRF ed overlay healthy.
- Parametri SEA, controlli Python, reserve e task biologici conservano AB06. Configurazione e plugin hanno fingerprint invariati.
- Prova CMC-like prescribed completata: 9.01 s, 9010 registrazioni; tracking ginocchio/caviglia protesici rispetto alla nuova IK: RMSE 0.281°/0.748°.
- Prova V26 prescribed con GRF sinistra online, detector e FSM v3 completata: 9.01 s; tracking rispetto al target governato servito: RMSE 0.151°/0.159°.
- **Contatto/eventi V26 non qualificati per AB07 con i profili comuni congelati**: 2 HS, 2 TO, 1 ciclo interno contro 4 cicli misurati; stance riconosciuta 0.132–0.171 s contro 0.970–1.075 s. Il completamento numerico non supera questo limite.
- MTP destro: RMSE circa 11.22° oltre la soglia comune 10°; limite già documentato nella baseline AB06. Nessuna soglia o gain ritoccati.
- Le prove usano riferimenti prescribed; non qualificano una policy appresa.
- Receipt finale: `models/AB07_SEASEA_Threadmill/pipeline_verification.json`.
- Grafici prescribed: `plot/AB07_prescribed_2026_10_06/10_06_2026_1/` (7 PNG).
- Grafici diagnostici V26: `plot/AB07_v26_left_prescribed_2026_10_06/10_06_2026_1/` (7 PNG).
