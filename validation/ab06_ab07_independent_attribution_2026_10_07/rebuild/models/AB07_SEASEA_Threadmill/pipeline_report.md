# AB07_SEASEA Pipeline Report

Generated: 2026-10-07T12:25:16
Status: complete

## Inputs

- Subject: `AB07`
- Task/trial: `treadmill/treadmill_01_01`
- Static trial: `static01`
- Healthy model: `models/AB07-raw/osimxml/ab07.osim`
- Donor model: `validation/ab06_ab07_independent_attribution_2026_10_07/rebuild/results/AB07/abx_reference_px9nj2m0/donor_model/AB06_SEASEA_stiff321_500_pi.osim`
- SEA plugin: `plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib`
- Frozen preparation references: `validation/ab06_ab07_independent_attribution_2026_10_07/rebuild/results/AB07/abx_reference_px9nj2m0/manifest.json`

## Outputs

- Bundle: `validation/ab06_ab07_independent_attribution_2026_10_07/rebuild/models/AB07_SEASEA_Threadmill`
- Setup: `validation/ab06_ab07_independent_attribution_2026_10_07/rebuild/models/AB07_SEASEA_Threadmill/AB07_SEASEA_stiff321_500_pi_setup.xml`
- Kinematics: `validation/ab06_ab07_independent_attribution_2026_10_07/rebuild/models/AB07_SEASEA_Threadmill/data/IK_results_AB07_SEASEA.mot`
- IK method: `opensim_marker_ik`
- Original IK output: `validation/ab06_ab07_independent_attribution_2026_10_07/rebuild/results/AB07/ab07_seasea_treadmill_treadmill_01_01_ik/02_ik/IK_full_10p360_153p415.mot`
- GRF: `validation/ab06_ab07_independent_attribution_2026_10_07/rebuild/models/AB07_SEASEA_Threadmill/data/AB07_SEASEA_GRF_FullSpan.mot`

## Diagnostics

- Mass before: `58.80765053000001` kg
- Mass after: `58.80765053000002` kg
- Removed left mass: `11.403651589977398` kg
- Marker old mean error: `0.09145446018383242` m
- Marker calibrated RMS: `0.000249901315597452` m
- Static calibration motion: `validation/ab06_ab07_independent_attribution_2026_10_07/rebuild/models/AB07_SEASEA-raw/osimxml/AB07_SEASEA_static_ground_registered.mot`
- Ground registration: `donor_relative_static_hardware_ground_v1`
- Kinematics range: `[10.36, 153.415]` s
- GRF range: `[10.359, 153.415]` s
- Smoke return code: `None`

## Notes

- Removed/replaced 43 healthy left-side force elements during graft.
- Smoke test skipped by user flag.
