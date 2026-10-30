# OnlineGRFContact plugin

`OnlineGRFSphereHalfSpaceForce` is a separate OpenSim force component used by
the simulator's `online_sensor` and `online` GRF modes. It is intentionally
independent from the SEA plugin.

The component:

- attaches a contact sphere to any `PhysicalFrame`;
- supports a moving ground-surface velocity for treadmill simulations;
- supports a calibratable positive force-penetration exponent (`1.5` by default);
- supports an optional bounded state-only residual scaled by normal force,
  penetration, and penetration rate;
- reports force, ground-origin moment, contact point, penetration, and slip;
- reports contact values even when `appliesForce=false`, enabling sensor-only
validation while prescribed ExternalLoads still drive the model.

The residual properties are zero by default, preserving the original contact
law. They are intended for offline-calibrated corrections that remain purely
state-based at runtime; do not encode time, gait phase, or prescribed-force
lookups in them.

## Build

Run these commands from the repository root. The source directory is
`online_GRF/sources/`; the separate `online_GRF/build/` directory contains
generated build files. Starting the Python simulator does not invoke CMake.
The Python simulator core is in
[`CMC-like Simulator/`](<../../CMC-like Simulator/README.md>); source, build
and installed-plugin paths here remain relative to the shared workspace root.
Use a fresh build directory after moving from the former `tools/online_GRF/`
layout or when changing SDK, compiler or architecture.

Windows x86-64:

```powershell
cmake -S online_GRF/sources -B online_GRF/build `
  -G "Visual Studio 17 2022" -A x64 `
  -DOPENSIM_INSTALL_DIR=C:/OpenSim-mCMC
cmake --build online_GRF/build --config Release
New-Item -ItemType Directory -Force plugins/online_grf_contact | Out-Null
Copy-Item online_GRF/build/Release/OnlineGRFContact.dll plugins/online_grf_contact/
```

macOS arm64:

```bash
export OPENSIM_INSTALL_DIR=/Users/tommy/opensim-core-install

cmake -S online_GRF/sources -B online_GRF/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DOPENSIM_INSTALL_DIR="$OPENSIM_INSTALL_DIR"
cmake --build online_GRF/build --config Release --parallel
cmake -E make_directory plugins/online_grf_contact
cmake -E copy_if_different \
  online_GRF/build/libOnlineGRFContact.dylib \
  plugins/online_grf_contact/libOnlineGRFContact.dylib
```

Use the OpenSim installation that provides the active Python binding. The
plugin and binding must use the same OpenSim/Simbody ABI.

## macOS verification

```bash
file plugins/online_grf_contact/libOnlineGRFContact.dylib
lipo -archs plugins/online_grf_contact/libOnlineGRFContact.dylib
otool -L plugins/online_grf_contact/libOnlineGRFContact.dylib
codesign --verify --verbose=4 plugins/online_grf_contact/libOnlineGRFContact.dylib

/opt/anaconda3/envs/envCMC-like/bin/python -c \
  "import opensim; opensim.LoadOpenSimLibrary('plugins/online_grf_contact/OnlineGRFContact'); assert opensim.OpenSimObject.newInstanceOfType('OnlineGRFSphereHalfSpaceForce') is not None"

/opt/anaconda3/envs/envCMC-like/bin/python online_GRF/validation/verify_online_grf_plugin.py \
  --setup models/AB06/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml \
  --profile online_GRF/profiles/AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json \
  --report results/online_grf_plugin_audit.json
```

The command above uses the retained operational profile after the 2026-10-15
cleanup. The original audit used `online_sensor_basis`, now recoverable from
Git for historical replay.

The recorded qualification on 2026-06-08 covered the then-current macOS arm64
Release dylib: its architecture, OpenSim-library rpath, type registration and
AB06 Python/C++ numerical-equivalence audit. This is historical build evidence;
the directory reorganization on 2026-10-20 does not replace or requalify the
installed dylib or Windows DLL. The compiled libraries remain in
`plugins/online_grf_contact/`. A macOS arm64 build from the new source path was
checked in a temporary build directory; the installed plugin was not replaced.

See the [online GRF guide](../README.md) for runtime modes, profiles and tools.
Frozen source inventories keep their original paths and hashes; verifying them
requires the original checkout.
