# Model preparation tools

The preparation workflow lives in `model_preparation_pipeline/`. Online GRF
runtime, profiles and tools live in the repository's [`online_GRF/`](../online_GRF/README.md);
the plugin sources are in [`online_GRF/sources/`](../online_GRF/sources/README.md).
Compiled libraries stay
in the simulator's `plugins/SEA/` and `plugins/online_grf_contact/` directories.
The Python simulator core is in
[`CMC-like Simulator/`](<../CMC-like Simulator/README.md>). These tools,
models, plugins and results remain at the shared workspace root. Preparation
commands keep their current paths; the simulator entry point is
`python "CMC-like Simulator/main.py"`.

```text
tools/
  model_preparation_pipeline/
    build_abx_seasea_pipeline.py
    fit_seasea_subject.py
    register_seasea_ground.py
    calibrate_ab06_seasea_markers.py
    convert_epic_ab06_to_opensim.m
    export_ab06_healthy_overlay.m
    convert_epic_ab06_tables.py
    run_opensim_sea_pipeline.py
```

## Prepare a subject model

`build_abx_seasea_pipeline.py` is the public entry point for preparing AB06,
AB07 and AB08. It converts the raw EPIC data with MATLAB, fits the prosthesis
mounts, registers the static foot pose, calibrates markers and runs fresh
OpenSim marker IK through `run_opensim_sea_pipeline.py`. It can also run RRA
diagnostics and a simulator smoke test. Existing model bundles remain in
`models/<subject>/`.

From the repository root, with a Python environment that has OpenSim:

```sh
python tools/model_preparation_pipeline/build_abx_seasea_pipeline.py --subject AB07 --validate-only
python tools/model_preparation_pipeline/build_abx_seasea_pipeline.py --help
python tools/model_preparation_pipeline/run_opensim_sea_pipeline.py --help
```

`--validate-only` checks the available inputs without building or overwriting
a model. Inspect the full build options before selecting output directories;
the builder's normal overwrite checks still apply. The same Python entry
points accept `python -m tools.model_preparation_pipeline.<module>` from the
repository root. Use forward slashes in the examples on Windows as well.

The two MATLAB functions are in the same directory. The builder adds that
directory to MATLAB's path. For interactive use:

```matlab
addpath(fullfile(repoRoot, 'tools', 'model_preparation_pipeline'));
```

`convert_epic_ab06_tables.py` is the separate Python conversion path used by
some validation workflows. Its contents are unchanged by the directory move,
including its original source hash. Its command-line help describes the
conversion and IK receipt commands. MATLAB remains the converter selected by
the ABx builder; the two paths are not interchangeable without validation.

## Removed tools and historical experiments

On 2026-10-16, the candidate and measured-GRF-window runners were removed,
together with the recovery converter, the PD/PI driver isolation suites,
the two pole analysis scripts and the cascade comparison script. The old
`scripts/` directory no longer exists. The retained OpenSim preparation
runner moved into `model_preparation_pipeline/`.

`tools/__init__.py` preserves former `tools.<module>` imports by searching the
new preparation directory. It does not recreate old filesystem paths or
duplicate the implementation. New callers should use the explicit
`tools.model_preparation_pipeline.<module>` imports. Frozen scripts that use
bare imports can find the retained converter with both the repository root
and `tools/model_preparation_pipeline/` on `PYTHONPATH` (use the platform's
path separator). Their exact file-path/hash contracts still require the
original layout.

The V14.1 recovery workflow and downstream historical checks that require
the deleted recovery converter need their original checkout and inputs.
Frozen manifests, source snapshots, receipts and reports retain the paths
and hashes of their original executions. Restore their original layout to
replay those experiments; do not rewrite their records to certify the new
layout. Ordinary simulator inputs and the installed plugin libraries are
not changed by this tools reorganization.

## Plugin sources and builds

The simulator loads SEA libraries from `plugins/SEA/` and the separate GRF
library from `plugins/online_grf_contact/`. Running Python does
not require C++ sources or trigger a CMake build.

## SEA plugin

The active `SeriesElasticActuator` PI sources live in the separate
`SEA_plugin_core - agent` repository. On the current Mac this is
`/Users/tommy/Documents/SEA_plugin_core - agent`; choose the actual checkout
path on each machine. Windows checkouts may use `SEA_plugin_core-agent`.

As of 2026-10-15 there is no live SEA source copy in this simulator. The
external checkout contains `SeriesElasticActuator.cpp`,
`SeriesElasticActuator.h`, `Plugin_interface.cpp`, its existing
`CMakeLists.txt`, a build README and `SEA_PI_SOURCE_MIGRATION_2026-10-15.json`.

Open the external repository to configure or compile the SEA. Automatic CMake
configuration on opening this simulator workspace is disabled. A local CMake
cache left in the simulator's `build/` is generated data, not a source or a
runtime dependency; do not use it to build the relocated plugin.

macOS arm64 (set the two paths to match your installation):

```sh
export SEA_PLUGIN_SOURCE_DIR="/path/to/SEA_plugin_core - agent"
export OPENSIM_INSTALL_DIR="$HOME/opensim-core-install"
cmake -S "$SEA_PLUGIN_SOURCE_DIR" -B "$SEA_PLUGIN_SOURCE_DIR/build_pi" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_OSX_ARCHITECTURES=arm64 \
  -DCMAKE_C_COMPILER=/usr/bin/clang -DCMAKE_CXX_COMPILER=/usr/bin/clang++ \
  -DOPENSIM_INSTALL_DIR="$OPENSIM_INSTALL_DIR"
cmake --build "$SEA_PLUGIN_SOURCE_DIR/build_pi" --config Release \
  --target SEA_Plugin_BlackBox_mCMC_impedence_ff --parallel 4
```

Windows x64 (PowerShell, use the SDK matching your Python binding):

```powershell
$seaSourceDir = 'C:/path/to/SEA_plugin_core-agent'
$seaOpenSimInstall = 'C:/OpenSim-mCMC'
cmake -S "$seaSourceDir" -B "$seaSourceDir/build_pi" `
  -G 'Visual Studio 17 2022' -A x64 `
  "-DOPENSIM_INSTALL_DIR=$seaOpenSimInstall"
Select-String -Path "$seaSourceDir/build_pi/CMakeCache.txt" -Pattern '^OpenSim_DIR:'
cmake --build "$seaSourceDir/build_pi" --config Release `
  --target SEA_Plugin_BlackBox_mCMC_impedence_ff --parallel 4
```

Use a fresh build directory when switching compiler, SDK or architecture.
On Windows confirm that the printed `OpenSim_DIR` belongs to the chosen SDK;
if another installation was selected, reconfigure with `-DOpenSim_DIR=` set
to the chosen SDK directory containing `OpenSimConfig.cmake`.
Verification should load the rebuilt library in a separate process before
deployment. These commands do not overwrite the simulator's installed
`plugins/SEA/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib` or
`plugins/SEA/SEA_Plugin_BlackBox_mCMC_impedence_ff.dll`.

The cleanup of 2026-10-15 removed the previous SEA variants and backup
libraries from the simulator. Install only the verified current build into
`plugins/SEA/`; the builders do not fall back to older SEA dynamics. The local
Mac checkout has no current Windows DLL. Build or recover the verified PI DLL
on Windows; do not rename an older DLL to make the requested path exist.

The migration preserved all installed libraries. Native verification was on
macOS arm64; the Windows branch remains in the external CMake project but was
not executed on this machine.

## Historical validation

Frozen H0 checks, including V1/V2/V3/V7, and the AB07 prescribed-ground numerical audit name the old
`tools/sea_plugin_relative_d/` files. Their sealed scripts and path/hash
records are intentionally unchanged. Re-running those specific historical
checks requires a separate checkout of simulator commit
`dba42388c32a441a5543f20881612a52781d7a99` (plus the same frozen inputs), where
the old paths still exist. The V26C pipeline bundle does not include these SEA
source files and is not their recovery archive.

The migration report and SHA256 mapping are in
`reports/user/2026-10-15_migrazione_sorgenti_sea_repository_esterna.md` and
`reports/cleanup/2026-10-15_sea_source_migration/`.

## Online GRF plugin

Its sources and CMake commands live in
[`../online_GRF/sources/`](../online_GRF/sources/README.md), moved from
`tools/online_GRF/` on 2026-10-20. Build from the repository root with
`cmake -S online_GRF/sources -B online_GRF/build` plus the platform and
OpenSim SDK options documented there. The build directory is generated data;
the runtime libraries remain in `plugins/online_grf_contact/`.
It is a separate component from the SEA plugin. Frozen inventories retain
their original source paths and hashes.
