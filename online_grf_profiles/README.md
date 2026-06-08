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

If a memoryless Coulomb contact patch cannot reproduce the required
tangential force or free moment, fit a bounded state-only residual. The
runtime residual is scaled by instantaneous normal force and may additionally
depend on penetration and penetration rate; it never reads time, phase, or
prescribed GRF at runtime:

```powershell
conda run -n envCMC-like python validation/calibrate_online_grf_residual.py `
  --setup path/to/setup.xml `
  --profile online_grf_profiles/my_model_online_candidate.json `
  --states-sto results/active_probe/sim_output_states.sto `
  --lock-vertical-force `
  --out-profile online_grf_profiles/my_model_residual_candidate.json `
  --report results/my_model_residual_calibration.json
```

Use `--fit-vertical-state-gains` only after the tangential-only candidate has
passed a short active probe. The generated profile remains
`requires_forward_validation` until it passes the forward-dynamics acceptance
gate.

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

Sensor accuracy is not sufficient for active use. Create a bounded-penetration
multi-contact candidate and validate it progressively:

```powershell
conda run -n envCMC-like python validation/calibrate_online_grf_basis.py `
  --setup path/to/setup.xml --profile path/to/preliminary_profile.json `
  --maximum-penetration 0.01 --location-count 20 --radius-count 20 `
  --out-profile online_grf_profiles/my_model_online_candidate.json `
  --report results/my_model_online_candidate_calibration.json
```

Active runs abort by default above `0.03 m` penetration. Use
`validation/validate_online_grf_forward_drift.py` and
`validation/online_grf_acceptance.py` before extending a rollout. A profile is
not active-validated while root-reserve usage remains materially above the
same-window `online_sensor` baseline.

For AB06 network training/inference, the current `online_sensor` default is
`AB06_SEASEA_stiff321_500_pi_online_physical_basis_10mm_balanced.json`. Its
plugin and sensor criteria pass, including holdout penetration below `15 mm`.
It is not approved for active `online` dynamics because the same-window
`pelvis_ty` reserve p95 ratio is `5.94x` versus the `1.5x` acceptance limit.

For gait-cycle segmentation, profiles may define
`heel_strike_confirmation_threshold_n`. The streaming detector timestamps the
low-threshold crossing from `grf_contact_threshold_n`, but emits the heel strike
only after the signal also reaches the confirmation threshold and satisfies the
minimum contact duration. This rejects swing-phase bumps without shifting the
reported heel-strike time.
