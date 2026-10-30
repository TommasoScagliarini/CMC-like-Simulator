# Binary heel/toe detector

The operational geometry is
[`operational/heel_toe_v25.json`](operational/heel_toe_v25.json). The current
MLP presets and saved best47 configuration use this geometry. The ABx model
preparation builder also uses its `ground` plane as the registration benchmark.

The JSON is self-contained: ground-plane origin and unit normal, two points
fixed to `/bodyset/foot_l`, and the contact rule. Each point produces contact
when its signed clearance from the plane is zero or negative. This geometry
does not compute forces or assign heel-strike/toe-off events. The runtime
detector is implemented in
[`binary_phase_detector.py`](binary_phase_detector.py), alongside its geometry.
[`model_loader.py`](<../../CMC-like Simulator/model_loader.py>) loads the profile and
[`simulation_runner.py`](<../../CMC-like Simulator/simulation_runner.py>) samples the
detector from the OpenSim state. It does not apply forces or interpret gait
events.
Event interpretation and transfer to the actor-facing FSM live in
[`Trajectory Generator/phase_tracking/binary_phase.py`](<../../Trajectory Generator/phase_tracking/binary_phase.py>).
This module retains the base classes and their V26 subclasses: V26 requires
stable heel contact to start stance, while toe-only contact during swing is
diagnostic. Sampling remains 1 ms, debounce 5 ms, and the event-delivery
limit 10 ms. The actor-facing FSM is a separate module in the same directory:
[`prosthetic_phase_fsm.py`](<../../Trajectory Generator/phase_tracking/prosthetic_phase_fsm.py>).
Both modules are shared by trajectory generators, outside `baseline_MLP/`.

Since 2026-10-20, `binary_phase.py` replaces `binary_phase_fsm.py`,
`binary_phase_fsm_v26.py`, `binary_phase_adapter.py` and
`binary_phase_adapter_v26.py`. Event contracts and provenance identifiers
are unchanged. Frozen source inventories retain the four former filenames
and hashes; verifying those records requires the original checkout.
The baseline bootstrap exposes `phase_tracking/` while preserving the canonical
imports `binary_phase` and `prosthetic_phase_fsm`. Importing `binary_phase`
still registers the four former module names for legacy pickle loading.

## Layout

```text
binary_detector/
  binary_phase_detector.py
  operational/
    heel_toe_v25.json
  __init__.py
  README.md
```

The runtime detector and selected operational geometry live here. The 2026-10-17 cleanup
retired the old development candidates and sweep/campaign outputs from the
root `validation/` directory. Its retained tests and fixtures are described in
[`validation/README.md`](../../validation/README.md). Offline GRF tools now live
in the sibling `calibration/`, `contact/`, `detector/` and `validation/`
directories under `online_GRF/`.
An experimental configuration is not promoted by its filename or presence in
that directory. In particular, the historical V17 sphere/material profile has a
different, force-based schema and cannot replace this binary-point geometry.

## Identity and saved paths

Since 2026-10-20, the runtime module formerly at the repository root is imported
as `online_GRF.binary_detector.binary_phase_detector`. The package
initializer does not import it automatically. Importing the runtime module
registers the former `binary_phase_detector` and
`online_grf_profiles.binary_detector.binary_phase_detector` names for legacy
pickles; import the new module before manually loading a pickle that names
either old module.
No forwarding file remains at the root. Frozen source inventories, including
F0, retain that former path and hash and require the original checkout to
verify them. Moving the module does not change the JSON geometry below.

The 2026-10-16 reorganization moved the exact 670 original bytes from:

```text
validation/binary_phase_detector_v25_geometry_runs/2026-08-04_local_reach_sweep_dev02_04_08/selected_candidate_profile.json
```

SHA-256:

```text
db704e502b99e49bea6d89493812bafdac748f8ce8d3ce28214ff624078539a2
```

The strict JSON schema is unchanged. Provenance is recorded here rather than
added to the profile. No additional model, mesh or calibration file is opened
by the JSON itself; the loaded model must provide its named foot frame.

[`path_resolver.py`](<../../CMC-like Simulator/path_resolver.py>) maps both the
former `online_grf_profiles/` address and the
exact original validation profile address to the current one,
including known saved macOS/Windows checkout paths. Existing external files,
nested source snapshots and other candidates are not substituted. Presets
now name the current path; saved checkpoint configurations and their hashes
remain unchanged.

Frozen manifests and receipts retain their original paths and checksums.
Replaying scripts that directly open those old physical paths, bypassing the
runtime resolver, still requires their original checkout/layout and inputs.
