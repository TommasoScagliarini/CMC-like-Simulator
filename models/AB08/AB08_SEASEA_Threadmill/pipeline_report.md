# AB08_SEASEA Pipeline Report

Generated: 2026-10-06T18:44:11
Status: complete

## Inputs

- Subject: `AB08`
- Task/trial: `treadmill/treadmill_01_01`
- Static trial: `static01`
- Healthy model: `models/AB08-raw/osimxml/AB08.osim`
- Donor model: `results/abx_reference_rrmf81lr/donor_model/AB06_SEASEA_stiff321_500_pi.osim`
- SEA plugin: `plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib`
- Frozen preparation references: `results/abx_reference_rrmf81lr/manifest.json`

## Outputs

- Bundle: `models/AB08_SEASEA_Threadmill`
- Setup: `models/AB08_SEASEA_Threadmill/AB08_SEASEA_stiff321_500_pi_setup.xml`
- Kinematics: `models/AB08_SEASEA_Threadmill/data/IK_results_AB08_SEASEA.mot`
- IK method: `opensim_marker_ik`
- Original IK output: `results/ab08_seasea_treadmill_treadmill_01_01_ik/02_ik/IK_full_8p630_151p525.mot`
- GRF: `models/AB08_SEASEA_Threadmill/data/AB08_SEASEA_GRF_FullSpan.mot`

## Diagnostics

- Mass before: `72.41236865000003` kg
- Mass after: `72.41236865000002` kg
- Removed left mass: `14.041802647231215` kg
- Marker old mean error: `0.09626556839664176` m
- Marker calibrated RMS: `0.00019870635133422418` m
- Static calibration motion: `models/AB08_SEASEA-raw/osimxml/AB08_SEASEA_static_ground_registered.mot`
- Ground registration: `donor_relative_static_hardware_ground_v1`
- Kinematics range: `[8.63, 151.525]` s
- GRF range: `[8.627, 151.525]` s
- Smoke return code: `None`

## Notes

- Removed/replaced 43 healthy left-side force elements during graft.
- Smoke test skipped by user flag.
