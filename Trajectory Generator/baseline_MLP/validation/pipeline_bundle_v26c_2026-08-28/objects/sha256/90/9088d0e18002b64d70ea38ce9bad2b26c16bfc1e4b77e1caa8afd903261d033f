"""Windows torch/OpenSim runtime shim for the baseline_MLP RLlib pipeline.

PyTorch (fbgemm) and OpenSim both ship an OpenMP runtime. On Windows the
simulator puts the OpenSim ``bin`` directory first on ``PATH`` so that the C++
SEA plugin and its dependencies resolve. PyTorch then resolves OpenSim's
``libiomp``/``libomp`` while importing ``fbgemm``, aborting the process before
any project code runs (the classic ``OMP: Error #15`` / ``fbgemm.dll`` failure).

This module reproduces the fix used by ``prosthesis_snn`` but is **standalone**
(it does not import ``snntorch``/``skrl``) so the MLP baseline stays free of the
SNN stack:

    1. set ``KMP_DUPLICATE_LIB_OK=TRUE`` (fallback guard),
    2. remove OpenSim entries from ``PATH``,
    3. import ``torch`` while OpenSim is off ``PATH``,
    4. re-register OpenSim via ``os.add_dll_directory`` and restore ``PATH`` for
       the simulator/plugin loader.

Importing this module FIRST (before ``ray``/``opensim``) in every process that
touches both torch and OpenSim is what makes the coexistence safe. Each Ray
worker should run this via ``worker_process_setup_hook``.
"""

from __future__ import annotations

import os


_DLL_DIR_HANDLES: list = []
_APPLIED = False


def _prepare_windows_runtime() -> list[str]:
    if os.name != "nt":
        return []
    os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")
    entries = os.environ.get("PATH", "").split(os.pathsep)
    opensim_entries = [
        entry for entry in entries
        if "opensim" in entry.lower() and os.path.isdir(entry)
    ]
    if not opensim_entries:
        return []
    os.environ["PATH"] = os.pathsep.join(
        entry for entry in entries if entry not in opensim_entries
    )
    return opensim_entries


def _restore_windows_runtime(opensim_entries: list[str]) -> None:
    if os.name != "nt":
        return
    for entry in opensim_entries:
        if hasattr(os, "add_dll_directory"):
            try:
                _DLL_DIR_HANDLES.append(os.add_dll_directory(entry))
            except OSError:
                pass
        path_entries = os.environ.get("PATH", "").split(os.pathsep)
        if entry not in path_entries:
            os.environ["PATH"] = os.pathsep.join(path_entries + [entry])


def apply() -> None:
    """Import torch under the OpenSim-PATH shim. Idempotent."""
    global _APPLIED
    if _APPLIED:
        return
    _APPLIED = True
    opensim_entries = _prepare_windows_runtime()
    try:
        import torch  # noqa: F401  (imported for its DLL side effects/ordering)
    finally:
        _restore_windows_runtime(opensim_entries)


# Apply on import so that ``import win_runtime`` as the first line of any
# entrypoint (or via a Ray worker setup hook) is sufficient.
apply()
