# SEA plugin — current PI source

This repository is the source location for the `SeriesElasticActuator` used by
the CMC-like simulator. The simulator keeps its compiled runtime libraries in
`plugins/`; running a simulation does not configure or compile this project.

On 2026-10-15, `SeriesElasticActuator.cpp`, `SeriesElasticActuator.h` and
`Plugin_interface.cpp` were synchronized byte-for-byte with the simulator's
validated PI source formerly in `tools/sea_plugin_relative_d/`. The existing
`CMakeLists.txt`, including its other executable targets, was preserved.

The non-impedance torque controller includes feedforward `tau_ref`, the
proportional torque error, bounded integral action and motor-speed damping.
The `torque_error_integral` state is integrated by OpenSim/the simulator. Its
derivative uses conditional anti-windup; the final motor torque limit is
500 N m. The impedance mode and normalized SEA input semantics are unchanged.

## Build only the plugin

Choose an OpenSim installation with the same architecture and ABI as the
simulator's Python binding. Use a fresh build directory after changing SDK,
compiler or platform. Paths below are local variables; the Windows checkout
may have a different directory name from the macOS checkout.

macOS arm64, from this repository:

```sh
export OPENSIM_INSTALL_DIR="$HOME/opensim-core-install"
cmake -S . -B build_pi -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_OSX_ARCHITECTURES=arm64 \
  -DCMAKE_C_COMPILER=/usr/bin/clang -DCMAKE_CXX_COMPILER=/usr/bin/clang++ \
  -DOPENSIM_INSTALL_DIR="$OPENSIM_INSTALL_DIR"
cmake --build build_pi --config Release \
  --target SEA_Plugin_BlackBox_mCMC_impedence_ff --parallel 4
```

Windows x64, from this repository in PowerShell:

```powershell
$seaOpenSimInstall = 'C:/OpenSim-mCMC'
cmake -S . -B build_pi -G 'Visual Studio 17 2022' -A x64 `
  "-DOPENSIM_INSTALL_DIR=$seaOpenSimInstall"
Select-String -Path build_pi/CMakeCache.txt -Pattern '^OpenSim_DIR:'
cmake --build build_pi --config Release `
  --target SEA_Plugin_BlackBox_mCMC_impedence_ff --parallel 4
```

Expected outputs are `build_pi/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib`
on macOS and `build_pi/Release/SEA_Plugin_BlackBox_mCMC_impedence_ff.dll` on
Windows. These commands do not install or replace the simulator libraries.
Only deploy a rebuilt library to the simulator's `plugins/` after verification.
On Windows confirm that `OpenSim_DIR` belongs to the selected SDK. If another
installation was selected, reconfigure with `-DOpenSim_DIR=` pointing to the
chosen SDK directory containing `OpenSimConfig.cmake`. `build_pi/` is ignored.

The other legacy executable targets and their model paths are independent
from this plugin build; the migration did not qualify those applications.
The simulator migration report records native compilation and numerical
verification. Windows instructions retain the existing MSVC build branch but
were not run as part of the macOS verification.

## Migration provenance

`SEA_PI_SOURCE_MIGRATION_2026-10-15.json` records the old simulator paths,
source SHA256 values and the preceding commits of both repositories. The
previous external files, including uncommitted edits, are backed up under
`.migration_backups/2026-10-15_before_pi.tar.gz`. This backup is local and
ignored by Git; it is not uploaded by a source commit.

The simulator retains historical frozen scripts and manifests that name the
old source paths (including H0 V1/V2/V3/V7 and the AB07 prescribed-ground
audit). Replay those specific historical checks in a checkout of
the recorded simulator revision; do not rewrite sealed provenance to the new
path. The ordinary simulator runtime requires only the installed libraries.

The migration leaves the two repositories uncommitted. Before sharing the
layout change with another machine, commit and publish the updated external
sources so that machine can obtain them independently of the simulator.
