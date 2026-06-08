"""Portable SNN tools for prosthetic kinematic reference generation."""

from __future__ import annotations

import os


_DLL_DIR_HANDLES = []


def _prepare_windows_torch_opensim_runtime() -> list[str]:
    """Avoid a Windows DLL clash between PyTorch and OpenSim.

    The simulator environment puts the OpenSim bin directory first on PATH.
    PyTorch can then resolve OpenSim's OpenMP DLL while importing fbgemm/torch,
    failing before any project code runs. Import torch with OpenSim temporarily
    removed from PATH, then add OpenSim back for the simulator/plugin loader.
    """

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
        entry for entry in entries
        if entry not in opensim_entries
    )
    return opensim_entries


def _restore_windows_opensim_runtime(opensim_entries: list[str]) -> None:
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


_OPEN_SIM_PATH_ENTRIES = _prepare_windows_torch_opensim_runtime()

from .config import SNNConfig
from .device import select_device
from .model import ProsthesisReferenceSNN
from .generator import ReferenceGenerator
from .reference_provider import (
    DictReferenceProvider,
    HybridReferenceProvider,
    ReferenceProvider,
    SNNProsthesisActionProvider,
    SNNProsthesisReferenceProvider,
)

_restore_windows_opensim_runtime(_OPEN_SIM_PATH_ENTRIES)

__all__ = [
    "DictReferenceProvider",
    "HybridReferenceProvider",
    "ProsthesisReferenceSNN",
    "ReferenceGenerator",
    "ReferenceProvider",
    "SNNConfig",
    "SNNProsthesisActionProvider",
    "SNNProsthesisReferenceProvider",
    "select_device",
]
