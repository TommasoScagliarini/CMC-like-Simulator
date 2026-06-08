# onlineGRF profiles

Profiles describe model-specific contact geometry separately from the
model-agnostic simulator code. Generate a marker-derived starting profile with:

```powershell
conda run -n envCMC-like python validation/generate_online_grf_profile.py `
  --model path/to/model.osim `
  --out online_grf_profiles/my_model.json
```

Calibrate and validate it against prescribed GRF with a holdout split:

```powershell
conda run -n envCMC-like python validation/validate_online_grf.py `
  --setup path/to/setup.xml `
  --profile online_grf_profiles/my_model.json `
  --out-profile online_grf_profiles/my_model_calibrated.json `
  --report results/my_model_online_grf_validation.json
```

For a higher-accuracy `online_sensor` profile, use sparse contact-basis
calibration. It searches a generic contact patch and retains only useful
contacts:

```powershell
conda run -n envCMC-like python validation/calibrate_online_grf_basis.py `
  --setup path/to/setup.xml `
  --profile online_grf_profiles/my_model.json `
  --out-profile online_grf_profiles/my_model_online_sensor.json `
  --report results/my_model_online_grf_basis.json
```

Verify the offline evaluator against the compiled plugin before accepting a
profile:

```powershell
conda run -n envCMC-like python validation/verify_online_grf_plugin.py `
  --setup path/to/setup.xml `
  --profile online_grf_profiles/my_model_online_sensor.json
```

For `online_sensor`, calibrate from an actual prescribed-dynamics rollout when
the simulated state differs materially from IK replay:

```powershell
conda run -n envCMC-like python validation/calibrate_online_grf_basis.py `
  --setup path/to/setup.xml `
  --profile online_grf_profiles/my_model.json `
  --states-sto results/sensor_rollout/sim_output_states.sto `
  --out-profile online_grf_profiles/my_model_sensor_runtime.json `
  --report results/my_model_sensor_runtime_calibration.json
```

Use `online_sensor` until the holdout metrics are acceptable. Then test active
contact with `--grf-mode online`; `--no-external-loads` proves the run is not
using prescribed GRF.

For gait-cycle segmentation, profiles may define
`heel_strike_confirmation_threshold_n`. The streaming detector timestamps the
low-threshold crossing from `grf_contact_threshold_n`, but emits the heel strike
only after the signal also reaches the confirmation threshold and satisfies the
minimum contact duration. This rejects swing-phase bumps without shifting the
reported heel-strike time.
