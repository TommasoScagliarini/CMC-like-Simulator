"""Baseline MLP RLlib pipeline for the CMC-like prosthetic trajectory env.

Single-agent PPO (Ray RLlib, new API stack, default MLP RLModule) that learns to
emit prosthetic trajectory segments for the CMC-like simulator. The biological
side stays muscle-driven via Static Optimization; the SEA C++ plugin and the
validated SimulationRunner are unchanged.

Importing the package applies the Windows torch/OpenSim runtime shim first, then
makes the simulator roots importable.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import win_runtime  # noqa: E402,F401  (apply OMP/OpenSim shim before torch/opensim)
import _bootstrap  # noqa: E402

_bootstrap.ensure_sim_paths()

__all__ = ["win_runtime"]
