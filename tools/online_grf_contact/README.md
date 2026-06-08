# OnlineGRFContact plugin

`OnlineGRFSphereHalfSpaceForce` is a separate OpenSim force component used by
the simulator's `online_sensor` and `online` GRF modes. It is intentionally
independent from the SEA plugin.

The component:

- attaches a contact sphere to any `PhysicalFrame`;
- supports a moving ground-surface velocity for treadmill simulations;
- supports a calibratable positive force-penetration exponent (`1.5` by default);
- reports force, ground-origin moment, contact point, penetration, and slip;
- reports contact values even when `appliesForce=false`, enabling sensor-only
  validation while prescribed ExternalLoads still drive the model.

## Build

Windows x86-64:

```powershell
cmake -S tools/online_grf_contact -B tools/online_grf_contact/build `
  -G "Visual Studio 17 2022" -A x64 `
  -DOPENSIM_INSTALL_DIR=C:/OpenSim-mCMC
cmake --build tools/online_grf_contact/build --config Release
Copy-Item tools/online_grf_contact/build/Release/OnlineGRFContact.dll plugins/
```

macOS arm64:

```bash
export OPENSIM_INSTALL_DIR=/Users/tommy/opensim-core-install

cmake -S tools/online_grf_contact -B tools/online_grf_contact/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DOPENSIM_INSTALL_DIR="$OPENSIM_INSTALL_DIR"
cmake --build tools/online_grf_contact/build --config Release --parallel
cmake -E copy_if_different \
  tools/online_grf_contact/build/libOnlineGRFContact.dylib \
  plugins/libOnlineGRFContact.dylib
```

Use the OpenSim installation that provides the active Python binding. The
plugin and binding must use the same OpenSim/Simbody ABI.

## macOS verification

```bash
file plugins/libOnlineGRFContact.dylib
lipo -archs plugins/libOnlineGRFContact.dylib
otool -L plugins/libOnlineGRFContact.dylib
codesign --verify --verbose=4 plugins/libOnlineGRFContact.dylib

/opt/anaconda3/envs/envCMC-like/bin/python -c \
  "import opensim; opensim.LoadOpenSimLibrary('plugins/OnlineGRFContact'); assert opensim.OpenSimObject.newInstanceOfType('OnlineGRFSphereHalfSpaceForce') is not None"

/opt/anaconda3/envs/envCMC-like/bin/python validation/verify_online_grf_plugin.py \
  --setup models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml \
  --profile online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_sensor_basis.json \
  --report results/online_grf_plugin_audit.json
```

Verified on macOS arm64 on 2026-06-08. The Release dylib is arm64, has an
OpenSim-library rpath, loads and registers the custom type, and passes the
AB06 Python/C++ numerical-equivalence audit.
