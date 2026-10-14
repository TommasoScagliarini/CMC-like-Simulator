# AB06_SEASEA Pipeline Report

Generated: 2026-10-07T12:24:20
Status: complete

## Inputs

- Subject: `AB06`
- Task/trial: `treadmill/treadmill_01_01`
- Static trial: `static_01`
- Healthy model: `models/AB06-raw/osimxml/AB06.osim`
- Donor model: `validation/ab06_ab07_independent_attribution_2026_10_07/rebuild/results/AB06/abx_reference_b65hjs89/donor_model/AB06_SEASEA_stiff321_500_pi.osim`
- SEA plugin: `plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib`
- Frozen preparation references: `validation/ab06_ab07_independent_attribution_2026_10_07/rebuild/results/AB06/abx_reference_b65hjs89/manifest.json`

## Outputs

- Bundle: `validation/ab06_ab07_independent_attribution_2026_10_07/rebuild/models/AB06_SEASEA_Threadmill`
- Setup: `validation/ab06_ab07_independent_attribution_2026_10_07/rebuild/models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml`
- Kinematics: `validation/ab06_ab07_independent_attribution_2026_10_07/rebuild/models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot`
- IK method: `opensim_marker_ik`
- Original IK output: `validation/ab06_ab07_independent_attribution_2026_10_07/rebuild/results/AB06/ab06_seasea_treadmill_treadmill_01_01_ik/02_ik/IK_full_11p990_155p045.mot`
- GRF: `validation/ab06_ab07_independent_attribution_2026_10_07/rebuild/models/AB06_SEASEA_Threadmill/data/AB06_SEASEA_GRF_FullSpan.mot`

## Diagnostics

- Mass before: `78.09885033000002` kg
- Mass after: `78.09885033000002` kg
- Removed left mass: `15.14449345815604` kg
- Marker old mean error: `0.10075492656850471` m
- Marker calibrated RMS: `0.0006155301533277773` m
- Static calibration motion: `validation/ab06_ab07_independent_attribution_2026_10_07/rebuild/models/AB06_SEASEA-raw/osimxml/AB06_SEASEA_static_ground_registered.mot`
- Ground registration: `donor_relative_static_hardware_ground_v1`
- Kinematics range: `[11.99, 155.045]` s
- GRF range: `[11.99, 155.045]` s
- Smoke return code: `None`

## Notes

- Removed/replaced 43 healthy left-side force elements during graft.
- Smoke test skipped by user flag.
