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
cmake -S tools/online_grf_contact -B tools/online_grf_contact/build \
  -DOPENSIM_INSTALL_DIR=/path/to/OpenSim
cmake --build tools/online_grf_contact/build --config Release
cp tools/online_grf_contact/build/libOnlineGRFContact.dylib plugins/
```
